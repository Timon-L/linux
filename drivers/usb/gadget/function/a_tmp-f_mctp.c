
// SPDX-License-Identifier: GPL-2.0+
/*
 * f_mctp.c - USB peripheral MCTP driver
 *
 * Copyright (C) 2024 Code Construct Pty Ltd
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/usb/composite.h>
#include <linux/skbuff.h>
#include <linux/err.h>
#include <linux/netdevice.h>
#include <linux/list.h>

#include <net/mctp.h>
#include <net/pkt_sched.h>

#include <linux/usb/mctp-usb.h>

#include <uapi/linux/if_arp.h>

#include <linux/usb/func_utils.h>

#define MCTP_USB_PREALLOC 4

struct f_mctp {
	struct usb_function function;

	struct usb_ep *in_ep;
	struct usb_ep *out_ep;

	struct net_device *dev;

	/* Updates to skb_free_list and the req lists are performed under
	 * ->lock
	 */
	spinlock_t lock;
	struct sk_buff_head skb_free_list;
	struct list_head rx_reqs;
	struct list_head tx_reqs;

	struct work_struct prealloc_work;

	unsigned int tx_batch_delay;
	struct sk_buff_head tx_batch;
	struct delayed_work tx_batch_work;
};

struct f_mctp_opts {
	struct usb_function_instance function_instance;
	unsigned int tx_batch_delay;
};

static inline struct f_mctp *func_to_mctp(struct usb_function *f)
{
	return container_of(f, struct f_mctp, function);
}

static struct usb_interface_descriptor mctp_usbg_intf = {
	.bLength = sizeof(mctp_usbg_intf),
	.bDescriptorType = USB_DT_INTERFACE,

	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_MCTP,
	.bInterfaceSubClass = 0x0, /* todo: allow host-interface mode? */
	.bInterfaceProtocol = 0x1, /* MCTP version 1 */
	/* .iInterface = DYNAMIC */
};

/* descriptors, full speed only */

static struct usb_endpoint_descriptor hs_mctp_source_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.wMaxPacketSize = cpu_to_le16(MCTP_USB_XFER_SIZE),

	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor hs_mctp_sink_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.wMaxPacketSize = cpu_to_le16(MCTP_USB_XFER_SIZE),

	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *hs_mctp_descs[] = {
	(struct usb_descriptor_header *)&mctp_usbg_intf,
	(struct usb_descriptor_header *)&hs_mctp_sink_desc,
	(struct usb_descriptor_header *)&hs_mctp_source_desc,
	NULL,
};

/* strings */
static struct usb_string mctp_usbg_strings[] = { { .s = "MCTP over USB" },
						 { 0 } };

static struct usb_gadget_strings mctp_usbg_stringtab = {
	.language = 0x0409, /* en-us */
	.strings = mctp_usbg_strings,
};

static struct usb_gadget_strings *mctp_usbg_gadget_strings[] = {
	&mctp_usbg_stringtab,
	NULL,
};

static int mctp_usbg_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_mctp *mctp = func_to_mctp(f);
	int id, rc;

	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	mctp_usbg_intf.bInterfaceNumber = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;

	mctp_usbg_strings[0].id = id;

	mctp->in_ep = usb_ep_autoconfig(cdev->gadget, &hs_mctp_source_desc);
	if (!mctp->in_ep) {
		ERROR(cdev, "%s in_ep autoconfig failed\n", f->name);
		return -ENODEV;
	}

	mctp->out_ep = usb_ep_autoconfig(cdev->gadget, &hs_mctp_sink_desc);
	if (!mctp->out_ep) {
		ERROR(cdev, "%s out_ep autoconfig failed\n", f->name);
		return -ENODEV;
	}

	rc = usb_assign_descriptors(f, NULL, hs_mctp_descs, NULL, NULL);
	if (rc) {
		ERROR(cdev, "assign_descriptors failed %d\n", rc);
		return rc;
	}

	DBG(cdev, "%s: in %s, out %s\n", f->name, mctp->in_ep->name,
	    mctp->out_ep->name);

	return 0;
}

static unsigned int __mctp_usbg_tx_batch_len(struct f_mctp *mctp)
	__must_hold(&mctp->lock)
{
	struct sk_buff_head *batch = &mctp->tx_batch;
	unsigned int len = 0;
	struct sk_buff *skb;

	for (skb = __skb_peek(batch); skb; skb = skb_peek_next(skb, batch))
		len += skb->len;

	return len;
}

/* skb already has USB headers, may contain multiple packets */
static int __mctp_usbg_tx(struct f_mctp *mctp, struct sk_buff *skb)
	__must_hold(&mctp->lock)
{
	struct net_device *dev = mctp->dev;
	struct usb_request *req;

	if (!mctp || !skb || !mctp->dev || !mctp->in_ep) {
		pr_err("mctp_usbg: NULL pointer detected! mctp=%p, skb=%p, dev=%p, in_ep=%p\n",
		       mctp, skb, mctp ? mctp->dev : NULL,
		       mctp ? mctp->in_ep : NULL);
		return -EINVAL;
	}

	req = list_first_entry_or_null(&mctp->tx_reqs, struct usb_request,
				       /*  */ list);
	if (req) {
		list_del(&req->list);
	}
	if (list_empty(&mctp->tx_reqs))
		netif_stop_queue(dev);

	if (!req) {
		printk("netdev_err path");
		netdev_err(dev, "no tx reqs available!\n");
		return -1;
	}

	req->context = skb;
	req->buf = skb->data;
	req->length = skb->len;

	printk("enqueue");
	int ret = usb_ep_queue(mctp->in_ep, req, GFP_ATOMIC);
	if (ret)
		printk("queue returned error: %d", ret);
	return 0;
}

static void __mctp_usbg_batch_tx(struct f_mctp *mctp) __must_hold(&mctp->lock)
{
	unsigned int i = 1, batch_len = 0;
	struct sk_buff *skb, *skb2;
	bool realloc = false;

	batch_len = __mctp_usbg_tx_batch_len(mctp);

	skb = __skb_dequeue(&mctp->tx_batch);
	if (!skb)
		return;

	/* if the first skb can't hold the batch, allocate a new one */
	if (batch_len - skb->len > skb_tailroom(skb)) {
		realloc = true;
		skb2 = skb;
		skb = netdev_alloc_skb(mctp->dev, batch_len);
		if (!skb) {
			netdev_warn_once(mctp->dev, "batch alloc failed\n");
			__skb_queue_purge(&mctp->tx_batch);
			return;
		}
		__skb_put_data(skb, skb2->data, skb2->len);
		consume_skb(skb2);
	}

	while ((skb2 = __skb_dequeue(&mctp->tx_batch)) != NULL) {
		__skb_put_data(skb, skb2->data, skb2->len);
		consume_skb(skb2);
		i++;
	}

	netdev_dbg(mctp->dev, "batch tx: %d for len %d%s. skb len %d\n", i,
		   batch_len, realloc ? ", realloced" : "", skb->len);

	__mctp_usbg_tx(mctp, skb);
}

static void mctp_usbg_batch_tx(struct f_mctp *mctp)
{
	unsigned long flags;

	spin_lock_irqsave(&mctp->lock, flags);
	__mctp_usbg_batch_tx(mctp);
	spin_unlock_irqrestore(&mctp->lock, flags);
}

static void mctp_usbg_batch_tx_work(struct work_struct *work)
{
	struct f_mctp *mctp =
		container_of(work, struct f_mctp, tx_batch_work.work);

	mctp_usbg_batch_tx(mctp);
}

//static int mctp_usbg_requeue(struct f_mctp *mctp, struct usb_ep *ep,
//			     struct usb_request *req)
//{
//	unsigned long flags;
//	struct sk_buff *skb;
//	int rc = 0;
//
//	req->buf = NULL;
//
//	spin_lock_irqsave(&mctp->lock, flags);
//
//	/* Do we have a preallocated skb available? if so, we can requeue
//	 * immediately; otherwise wait for the workqueue to populate.
//	 */
//	skb = __skb_dequeue(&mctp->skb_free_list);
//	if (skb) {
//		req->buf = skb->data;
//		req->context = skb;
//		rc = usb_ep_queue(ep, req, GFP_ATOMIC);
//	} else {
//		/* keep for later allocation */
//		list_add_tail(&req->list, &mctp->rx_reqs);
//	}
//
//	spin_unlock_irqrestore(&mctp->lock, flags);
//
//	schedule_work(&mctp->prealloc_work);
//
//	return rc;
///*  */}

static int mctp_usbg_requeue(struct f_mctp *mctp, struct usb_ep *ep,
			     struct usb_request *req)
{
	struct sk_buff *skb;
	int rc = 0;

	req->buf = NULL;

	skb = netdev_alloc_skb(mctp->dev, MCTP_USB_XFER_SIZE);
	if (!skb) {
		pr_warn("%s can't allocate skb", __func__);
		return -1;
	}

	req->buf = skb->data;
	req->context = skb;
	rc = usb_ep_queue(ep, req, GFP_ATOMIC);
	if (rc)
		pr_warn("usb_ep_queue err");
	return rc;
}

static void mctp_usbg_handle_rx_urb(struct f_mctp *mctp,
				    struct usb_request *req)
{
	struct device *dev = &mctp->function.config->cdev->gadget->dev;
	struct pcpu_dstats *dstats = this_cpu_ptr(mctp->dev->dstats);
	struct sk_buff *skb = req->context;
	struct mctp_usb_hdr *hdr;
	struct mctp_skb_cb *cb;
	unsigned int len;
	u16 id;

	len = req->actual;
	__skb_put(skb, len);

	hdr = skb_pull_data(skb, sizeof(*hdr));
	if (!hdr)
		goto err;

	id = be16_to_cpu(hdr->id);
	if (id != MCTP_USB_DMTF_ID) {
		dev_dbg(dev, "%s: invalid id %04x\n", __func__, id);
		goto err;
	}

	if (hdr->len < sizeof(struct mctp_hdr) + sizeof(struct mctp_usb_hdr)) {
		dev_dbg(dev, "%s: short packet (hdr) %d\n", __func__, hdr->len);
		goto err;
	}

	/* todo: multi-packet transfers */
	if (hdr->len - sizeof(struct mctp_usb_hdr) < skb->len) {
		dev_dbg(dev, "%s: short packet (xfer) %d, actual %d\n",
			__func__, hdr->len, skb->len);
		goto err;
	}

	skb->protocol = htons(ETH_P_MCTP);
	skb_reset_network_header(skb);
	cb = __mctp_cb(skb);
	cb->halen = 0;
	netif_rx(skb);

	u64_stats_update_begin(&dstats->syncp);
	u64_stats_inc(&dstats->rx_packets);
	u64_stats_add(&dstats->rx_bytes, len);
	u64_stats_update_end(&dstats->syncp);

	return;

err:
	/* todo: return to free list */
	kfree_skb(skb);
}

static void mctp_usbg_out_ep_complete(struct usb_ep *ep,
				      struct usb_request *req)
{
	struct f_mctp *mctp = ep->driver_data;
	struct usb_composite_dev *cdev = mctp->function.config->cdev;
	int rc;

	//printk("%s status:%d", __func__, req->status);
	switch (req->status) {
	case 0:
		mctp_usbg_handle_rx_urb(mctp, req);

		/* re-queue out request */
		rc = mctp_usbg_requeue(mctp, ep, req);
		if (rc) {
			WARNING(cdev, "%s: unable to re-queue out req\n",
				__func__);
			usb_ep_free_request(ep, req);
		}
		break;

	case -ECONNABORTED:
		printk("%s ECONNABORTED", __func__);
		break;
	case -ECONNRESET:
		printk("%s ECONNRESET", __func__);
		break;
	case -ESHUTDOWN:
		printk("%s ESHUTDOWN", __func__);
		kfree_skb(req->context);
		usb_ep_free_request(ep, req);
		break;

	default:
		WARNING(cdev, "%s: invalid status %d?", __func__, req->status);
		usb_ep_free_request(ep, req);
	}
}

static void mctp_usbg_in_ep_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_mctp *mctp = ep->driver_data;
	struct usb_composite_dev *cdev = mctp->function.config->cdev;
	struct sk_buff *skb = req->context;
	unsigned long flags;

	kfree_skb(skb);
	req->context = NULL;
	req->buf = NULL;

	/* todo: tx stats */

	switch (req->status) {
	case 0:
		spin_lock_irqsave(&mctp->lock, flags);
		if (list_empty(&mctp->tx_reqs))
			netif_start_queue(mctp->dev);
		list_add(&req->list, &mctp->tx_reqs);
		spin_unlock_irqrestore(&mctp->lock, flags);
		break;
	case -ECONNABORTED:
	case -ECONNRESET:
	case -ESHUTDOWN:
		usb_ep_free_request(ep, req);
		break;
	default:
		WARNING(cdev, "%s: invalid status %d?", __func__, req->status);
		usb_ep_free_request(ep, req);
	}
}

static int mctp_usbg_enable_ep(struct usb_gadget *gadget, struct f_mctp *mctp,
			       struct usb_ep *ep)
{
	int rc;

	rc = config_ep_by_speed(gadget, &mctp->function, ep);
	if (rc)
		return rc;

	rc = usb_ep_enable(ep);
	if (rc)
		return rc;

	ep->driver_data = mctp;

	return 0;
}

static int mctp_usbg_enable(struct usb_composite_dev *cdev, struct f_mctp *mctp)
{
	struct usb_request *out_req, *in_req;
	//unsigned long flags;
	struct sk_buff *skb;
	int rc;

	rc = mctp_usbg_enable_ep(cdev->gadget, mctp, mctp->out_ep);
	if (rc) {
		ERROR(cdev, "%s: out ep enable failed %d\n", __func__, rc);
		return rc;
	}

	rc = mctp_usbg_enable_ep(cdev->gadget, mctp, mctp->in_ep);
	if (rc) {
		ERROR(cdev, "%s: in ep enable failed %d\n", __func__, rc);
		goto err_disable_out;
	}

	/* todo: just one out queued req for now */
	out_req = alloc_ep_req(mctp->out_ep, MCTP_USB_XFER_SIZE);
	if (!out_req) {
		ERROR(cdev, "%s: out req alloc failed\n", __func__);
		goto err_disable_in;
	}

	//spin_lock_irqsave(&mctp->lock, flags);
	//skb = __skb_dequeue(&mctp->skb_free_list);
	//spin_unlock_irqrestore(&mctp->lock, flags);

	//if (!skb)
	skb = netdev_alloc_skb(mctp->dev, MCTP_USB_XFER_SIZE);

	if (!skb)
		goto err_free_req;

	out_req->context = skb;
	out_req->buf = skb->data;
	out_req->complete = mctp_usbg_out_ep_complete;

	rc = usb_ep_queue(mctp->out_ep, out_req, GFP_ATOMIC);
	if (rc) {
		ERROR(cdev, "%s: out req queue failed %d\b", __func__, rc);
		goto err_free_skb;
	}

	/* todo: and just one in the in queue too */
	in_req = usb_ep_alloc_request(mctp->in_ep, GFP_ATOMIC);
	if (!in_req) {
		ERROR(cdev, "%s: out req alloc failed\n", __func__);
		goto err_disable_in;
	}
	in_req->complete = mctp_usbg_in_ep_complete;

	/* spin_lock_irqsave(&mctp->lock, flags);
	list_add(&in_req->list, &mctp->tx_reqs);
	spin_unlock_irqrestore(&mctp->lock, flags); */

	return 0;

err_free_skb:
	kfree_skb(skb);
err_free_req:
	free_ep_req(mctp->out_ep, out_req);
err_disable_in:
	usb_ep_disable(mctp->in_ep);
err_disable_out:
	usb_ep_disable(mctp->out_ep);

	return rc;
}

static netdev_tx_t mctp_usbg_start_xmit(struct sk_buff *skb,
					struct net_device *dev)
{
	struct f_mctp *mctp = netdev_priv(dev);
	struct pcpu_dstats *dstats = this_cpu_ptr(mctp->dev->dstats);
	struct mctp_usb_hdr *hdr;
	unsigned long flags;
	unsigned int plen;

	if (mctp->dev != dev) {
		pr_err("1:different dev pointer, mctp:%pX, original:%pX\n",
		       mctp->dev, dev);
	}

	if (skb->len + sizeof(*hdr) > MCTP_USB_XFER_SIZE)
		goto drop;

	plen = skb->len;
	hdr = skb_push(skb, sizeof(*hdr));
	hdr->id = cpu_to_be16(MCTP_USB_DMTF_ID);
	hdr->rsvd = 0;
	hdr->len = plen + sizeof(*hdr);

	spin_lock_irqsave(&mctp->lock, flags);
	if (!mctp->tx_batch_delay) {
		if (mctp->dev != dev) {
			pr_err("2:different dev pointer, mctp:%pX, original:%pX\n",
			       mctp->dev, dev);
		}
		/* direct tx */
		__mctp_usbg_tx(mctp, skb);
	} else {
		unsigned int batch_len;

		batch_len = __mctp_usbg_tx_batch_len(mctp);
		if (batch_len + skb->len > MCTP_USB_XFER_SIZE)
			__mctp_usbg_batch_tx(mctp);

		__skb_queue_head(&mctp->tx_batch, skb);

		schedule_delayed_work(&mctp->tx_batch_work,
				      msecs_to_jiffies(mctp->tx_batch_delay));
	}
	spin_unlock_irqrestore(&mctp->lock, flags);
	u64_stats_update_begin(&dstats->syncp);
	u64_stats_inc(&dstats->tx_packets);
	u64_stats_add(&dstats->tx_bytes, plen);
	u64_stats_update_end(&dstats->syncp);

	return NETDEV_TX_OK;

drop:
	kfree_skb(skb);
	u64_stats_update_begin(&dstats->syncp);
	u64_stats_inc(&dstats->tx_drops);
	u64_stats_update_end(&dstats->syncp);
	return NETDEV_TX_OK;
}

static int mctp_usbg_open(struct net_device *net)
{
	return 0;
}

static int mctp_usbg_stop(struct net_device *net)
{
	return 0;
}

static const struct net_device_ops mctp_usbg_netdev_ops = {
	.ndo_open = mctp_usbg_open,
	.ndo_stop = mctp_usbg_stop,
	.ndo_start_xmit = mctp_usbg_start_xmit,
};

static void __mctp_usbg_disable(struct f_mctp *mctp)
{
	usb_ep_disable(mctp->in_ep);
	usb_ep_disable(mctp->out_ep);
}

static void mctp_usbg_disable(struct usb_function *f)
{
	struct f_mctp *mctp = func_to_mctp(f);

	__mctp_usbg_disable(mctp);
}

static int mctp_usbg_set_alt(struct usb_function *f, unsigned intf,
			     unsigned alt)
{
	struct usb_composite_dev *cdev = f->config->cdev;
	struct f_mctp *mctp = func_to_mctp(f);

	__mctp_usbg_disable(mctp);
	return mctp_usbg_enable(cdev, mctp);
}

static void mctp_usbg_free_func(struct usb_function *f)
{
	struct f_mctp *mctp = func_to_mctp(f);

	kfree(mctp);
}

static void mctp_usbg_netdev_setup(struct net_device *dev)
{
	dev->type = ARPHRD_MCTP;

	dev->mtu = MCTP_USB_MTU_MIN;
	dev->min_mtu = MCTP_USB_MTU_MIN;
	dev->max_mtu = MCTP_USB_MTU_MAX;

	dev->hard_header_len = 0;
	dev->addr_len = 0;
	dev->tx_queue_len = DEFAULT_TX_QUEUE_LEN;
	dev->flags = IFF_NOARP;
	dev->netdev_ops = &mctp_usbg_netdev_ops;
	dev->needs_free_netdev = true;
	dev->pcpu_stat_type = NETDEV_PCPU_STAT_DSTATS;
}

static struct usb_function *
mctp_usbg_alloc_func(struct usb_function_instance *fi)
{
	struct f_mctp_opts *opts;
	struct net_device *dev;
	struct f_mctp *mctp;
	int rc;

	opts = container_of(fi, struct f_mctp_opts, function_instance);

	dev = alloc_netdev(sizeof(*mctp), "mctpusbg%d", NET_NAME_ENUM,
			   mctp_usbg_netdev_setup);
	if (!dev)
		return ERR_PTR(-ENOMEM);

	mctp = netdev_priv(dev);
	mctp->dev = dev;
	mctp->tx_batch_delay = opts->tx_batch_delay;

	spin_lock_init(&mctp->lock);
	//INIT_LIST_HEAD(&mctp->rx_reqs);
	//INIT_LIST_HEAD(&mctp->tx_reqs);
	__skb_queue_head_init(&mctp->skb_free_list);
	__skb_queue_head_init(&mctp->tx_batch);
	//INIT_WORK(&mctp->prealloc_work, mctp_usbg_prealloc_work);
	INIT_DELAYED_WORK(&mctp->tx_batch_work, mctp_usbg_batch_tx_work);

	mctp->function.name = "mctp";
	mctp->function.bind = mctp_usbg_bind;
	mctp->function.set_alt = mctp_usbg_set_alt;
	mctp->function.disable = mctp_usbg_disable;
	mctp->function.strings = mctp_usbg_gadget_strings;
	mctp->function.free_func = mctp_usbg_free_func;

	/* this will allocate our first pool of out (rx) skbs */
	//mctp_usbg_prealloc(mctp);

	rc = register_netdev(dev);
	if (rc) {
		free_netdev(dev);
		return ERR_PTR(rc);
	}

	return &mctp->function;
}

static struct f_mctp_opts *to_f_mctp_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_mctp_opts,
			    function_instance.group);
}

static void mctp_usbg_attr_release(struct config_item *item)
{
	struct f_mctp_opts *opts = to_f_mctp_opts(item);

	usb_put_function_instance(&opts->function_instance);
}

static struct configfs_item_operations mctp_usbg_item_ops = {
	.release = mctp_usbg_attr_release,
};

static ssize_t mctp_usbg_tx_batch_delay_store(struct config_item *item,
					      const char *page, size_t count)
{
	struct f_mctp_opts *opts = to_f_mctp_opts(item);
	unsigned int tmp;
	int rc;

	rc = kstrtouint(page, 0, &tmp);
	if (!rc)
		opts->tx_batch_delay = tmp;

	return rc;
}

static ssize_t mctp_usbg_tx_batch_delay_show(struct config_item *item,
					     char *page)
{
	struct f_mctp_opts *opts = to_f_mctp_opts(item);

	return sprintf(page, "%u\n", opts->tx_batch_delay);
}

CONFIGFS_ATTR(mctp_usbg_, tx_batch_delay);

static struct configfs_attribute *mctp_usbg_attrs[] = {
	&mctp_usbg_attr_tx_batch_delay,
	NULL,
};

static const struct config_item_type mctp_usbg_func_type = {
	.ct_item_ops = &mctp_usbg_item_ops,
	.ct_attrs = mctp_usbg_attrs,
	.ct_owner = THIS_MODULE,
};

static void mctp_usbg_free_instance(struct usb_function_instance *fi)
{
	struct f_mctp_opts *opts;

	opts = container_of(fi, struct f_mctp_opts, function_instance);
	kfree(opts);
}

static struct usb_function_instance *mctp_usbg_alloc_instance(void)
{
	struct f_mctp_opts *opts;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);

	opts->function_instance.free_func_inst = mctp_usbg_free_instance;

	config_group_init_type_name(&opts->function_instance.group, "",
				    &mctp_usbg_func_type);

	return &opts->function_instance;
}

DECLARE_USB_FUNCTION_INIT(mctp, mctp_usbg_alloc_instance, mctp_usbg_alloc_func);
MODULE_LICENSE("GPL");
