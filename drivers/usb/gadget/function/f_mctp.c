#include <linux/kernel.h>
#include <linux/usb/composite.h>
#include <linux/netdevice.h>

#include <net/mctp.h>
#include "net/pkt_sched.h"

#include "linux/usb/mctp-usb.h"

#include <uapi/linux/if_arp.h>

#include <linux/usb/func_utils.h>

#define MAX_REQS_LEN 5

struct f_mctp {
	struct usb_function function;
	struct usb_ep *in_ep;
	struct usb_ep *out_ep;

	struct net_device *dev;

	spinlock_t rx_spinlock;
	struct list_head free_rx_reqs;
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
};

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
	[0] = (struct usb_descriptor_header *)&mctp_usbg_intf,
	[1] = (struct usb_descriptor_header *)&hs_mctp_sink_desc,
	[2] = (struct usb_descriptor_header *)&hs_mctp_source_desc,
	[3] = NULL,
};

static struct usb_string
	mctp_usbg_strings[] = { [0] = { .s = "MCTP over USB" }, [1] = { 0 } };

static struct usb_gadget_strings mctp_usbg_stringtab = {
	.language = 0x0409, /* en-us */
	.strings = mctp_usbg_strings,
};

static struct usb_gadget_strings *mctp_usbg_gadget_strings[] = {
	[0] = &mctp_usbg_stringtab,
	[1] = NULL,
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
		ERROR(cdev, "assign_descriptos failed %d\n", rc);
		return rc;
	}

	DBG(cdev, "%s: in %s, out %s\n", f->name, mctp->in_ep->name,
	    mctp->out_ep->name);
	return 0;
}

static int mctp_rx_submit(struct f_mctp *mctp, struct usb_ep *ep,
			  struct usb_request *req)
{
	struct sk_buff *skb = req->context;
	int rc = 0;

	if (!skb) {
		skb = netdev_alloc_skb(mctp->dev, MCTP_USB_XFER_SIZE);
		if (!skb) {
			pr_warn("%s: failed alloc skb", __func__);
			return -ENOMEM;
		}

		req->buf = skb->data;
		req->context = skb;
	}

	rc = usb_ep_queue(ep, req, GFP_ATOMIC);
	if (rc) {
		pr_warn("usb_ep_queue err");
		kfree(skb);
		req->context = NULL;
	}

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
	kfree_skb(skb);
}

static void mctp_usbg_out_ep_complete(struct usb_ep *ep,
				      struct usb_request *req)
{
	struct f_mctp *mctp = ep->driver_data;
	struct usb_composite_dev *cdev = mctp->function.config->cdev;
	struct sk_buff *skb = req->context;
	unsigned long flags;
	int rc;

	switch (req->status) {
	case 0:
		if (skb) {
			mctp_usbg_handle_rx_urb(mctp, req);
		}

		spin_lock_irqsave(&mctp->rx_spinlock, flags);
		list_add_tail(&req->list, &mctp->free_rx_reqs);
		spin_unlock_irqrestore(&mctp->rx_spinlock, flags);

		spin_lock_irqsave(&mctp->rx_spinlock, flags);
		if (!list_empty(&mctp->free_rx_reqs)) {
			struct usb_request *next_req = list_first_entry(
				&mctp->free_rx_reqs, struct usb_request, list);

			list_del(&next_req->list);
			spin_unlock_irqrestore(&mctp->rx_spinlock, flags);

			rc = mctp_rx_submit(mctp, ep, next_req);
			if (rc < 0) {
				ERROR(cdev, "resubmit failed\n");
				break;
			}
		} else {
			spin_unlock_irqrestore(&mctp->rx_spinlock, flags);
		}
		return;
	case -ESHUTDOWN:
		kfree_skb(req->context);
		break;
	case -ECONNABORTED:
	case -ECONNRESET:
	default:
		WARNING(cdev, "%s: invalid status %d?", __func__, req->status);
	}

	free_ep_req(ep, req);
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
	struct usb_request *out_req;
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

	for (int i = 0; i < MAX_REQS_LEN; i++) {
		out_req = alloc_ep_req(mctp->out_ep, MCTP_USB_XFER_SIZE);
		if (!out_req) {
			ERROR(cdev, "%s: out req alloc failed\n", __func__);
			goto err_disable_in;
		}

		out_req->complete = mctp_usbg_out_ep_complete;

		rc = mctp_rx_submit(mctp, mctp->out_ep, out_req);
		if (rc) {
			ERROR(cdev, "%s: out req queue failed %d\b", __func__,
			      rc);
			free_ep_req(mctp->out_ep, out_req);
			goto err_disable_in;
		}
	}

	return 0;

err_disable_in:
	usb_ep_disable(mctp->in_ep);
err_disable_out:
	usb_ep_disable(mctp->out_ep);

	return rc;
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
};

static void __mctp_usbg_disable(struct f_mctp *mctp)
{
	usb_ep_disable(mctp->in_ep);
	usb_ep_disable(mctp->out_ep);
}

static void mctp_usbg_disable(struct usb_function *f)
{
	struct f_mctp *mctp = func_to_mctp(f);
	struct usb_request *list, *next;
	unsigned long flags;

	if (mctp->out_ep) {
		spin_lock_irqsave(&mctp->rx_spinlock, flags);
		list_for_each_entry_safe(list, next, &mctp->free_rx_reqs,
					 list) {
			free_ep_req(mctp->out_ep, list);
			list_del(&list->list);
			kfree(list);
		}
		spin_unlock_irqrestore(&mctp->rx_spinlock, flags);
	}

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

	//spin_lock_init(&mctp->tx_spinlock);
	spin_lock_init(&mctp->rx_spinlock);
	//INIT_LIST_HEAD(&mctp->free_tx_reqs);
	INIT_LIST_HEAD(&mctp->free_rx_reqs);

	mctp->function.name = "mctp";
	mctp->function.bind = mctp_usbg_bind;
	mctp->function.set_alt = mctp_usbg_set_alt;
	mctp->function.disable = mctp_usbg_disable;
	mctp->function.strings = mctp_usbg_gadget_strings;
	mctp->function.free_func = mctp_usbg_free_func;

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
	[0] = &mctp_usbg_attr_tx_batch_delay,
	[1] = NULL,
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