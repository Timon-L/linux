#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/usb.h>
#include <linux/usb/mctp-usb.h>

#include <net/mctp.h>

#include <uapi/linux/if_arp.h>

#define MAX_TX_URB_COUNT 8
#define MCTP_USB_MAX_BULK_SIZE 1024

struct mctp_usb_tx_context {
	struct urb *urb;
	struct sk_buff_head skbs;
	char *buffer;
	size_t filled;
	struct list_head list;
};
struct mctp_usb {
	struct usb_device *usbdev;
	struct usb_interface *intf;

	struct net_device *netdev;

	spinlock_t tx_spinlock;
	struct list_head free_tx_urbs;

	__u8 ep_in;
	__u8 ep_out;

	struct list_head active_contexts;
	struct mcpt_usb_tx_context *current_ctx;
};

static void mctp_usb_stat_tx_dropped(struct net_device *dev)
{
	struct pcpu_dstats *dstats = this_cpu_ptr(dev->dstats);

	u64_stats_update_begin(&dstats->syncp);
	u64_stats_inc(&dstats->tx_drops);
	u64_stats_update_begin(&dstats->syncp);
}

static void mctp_usb_stat_tx_done(struct net_device *dev, unsigned int len)
{
	struct pcpu_dstats *dstats = this_cpu_ptr(dev->dstats);

	u64_stats_update_begin(&dstats->syncp);
	u64_stats_inc(&dstats->tx_packets);
	u64_stats_add(&dstats->tx_bytes, len);
	u64_stats_update_end(&dstats->syncp);
}

static void mctp_usb_out_complete(struct urb *urb)
{
	struct mctp_usb_tx_context *ctx = urb->context;
	struct sk_buff *skb = urb->context;
	struct net_device *netdev = skb->dev;
	struct mctp_usb *mctp_usb = netdev_priv(netdev);
	unsigned long flags;
	int status;

	status = urb->status;

	switch (status) {
	case 0:
		mctp_usb_stat_tx_done(netdev, skb->len);
		break;
	case -ENOENT:
		printk("%s ENOENT\n", __func__);
		break;
	case -ECONNRESET:
		printk("%s ECONNRESET\n", __func__);
		break;
	case -ESHUTDOWN:
		printk("%s ESHUTDOWN\n", __func__);
		break;
	case -EPROTO:
		printk("%s EPROTO\n", __func__);
		mctp_usb_stat_tx_dropped(netdev);
		break;
	default:
		dev_err(&mctp_usb->usbdev->dev, "%s: urb status: %d\n",
			__func__, status);
		mctp_usb_stat_tx_dropped(netdev);
		break;
	}

	skb_queue_purge(&ctx->skbs);
	kfree(ctx->buffer);

	spin_lock_irqsave(&mctp_usb->tx_spinlock, flags);
	if (list_empty(&mctp_usb->free_tx_urbs))
		netif_wake_queue(netdev);
	list_add_tail(&urb->urb_list, &mctp_usb->free_tx_urbs);
	spin_unlock_irqrestore(&mctp_usb->tx_spinlock, flags);

	kfree(ctx);
}

static netdev_tx_t mctp_usb_start_xmit(struct sk_buff *skb,
				       struct net_device *dev)
{
	struct mctp_usb *mctp_usb = netdev_priv(dev);
	struct mctp_usb_tx_context *ctx = NULL;
	struct mctp_usb_hdr *hdr;
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&mctp_usb->tx_spinlock, flags);

	list_for_each_entry(ctx, &mctp_usb->active_contexts, list) {
		if (ctx->filled + sizeof(*hdr) + skb->len <= MCTP_USB_XFER_SIZE)
			break;
		ctx = NULL;
	}

	if (!ctx) {
		if (list_empty(&mctp_usb->free_tx_urbs)) {
			spin_unlock_irqrestore(&mctp_usb->tx_spinlock, flags);
			return NETDEV_TX_BUSY;
		}

		ctx = kzalloc(sizeof(*ctx), GFP_ATOMIC);
		if (!ctx)
			goto err_drop;

		ctx->buffer = kmalloc(MCTP_USB_XFER_SIZE, GFP_ATOMIC);
		if (!ctx->buffer) {
			kfree(ctx);
			goto err_drop;
		}

		ctx->urb = list_first_entry(&mctp_usb->free_tx_urbs, struct urb,
					    urb_list);
		list_del(&ctx->urb->urb_list);
		skb_queue_head_init(&ctx->skbs);
		list_add_tail(&ctx->list, &mctp_usb->active_contexts);
	}

	hdr = (struct mctp_usb_hdr *)(ctx->buffer + ctx->filled);
	hdr->id = cpu_to_be16(MCTP_USB_DMTF_ID);
	hdr->rsvd = 0;
	hdr->len = skb->len + sizeof(*hdr);

	memcpy(ctx->buffer + ctx->filled + sizeof(*hdr), skb->data, skb->len);
	ctx->filled += hdr->len;
	skb_queue_tail(&ctx->skbs, skb);

	if (ctx->filled >= MCTP_USB_XFER_SIZE) {
		usb_fill_bulk_urb(
			ctx->urb, mctp_usb->usbdev,
			usb_sndbulkpipe(mctp_usb->usbdev, mctp_usb->ep_out),
			ctx->buffer, ctx->filled, mctp_usb_out_complete, ctx);

		rc = usb_submit_urb(ctx->urb, GFP_ATOMIC);
		if (rc) {
			skb_queue_purge(&ctx->skbs);
			kfree(ctx->buffer);
			list_add(&ctx->urb->urb_list, &mctp_usb->free_tx_urbs);
			list_del(&ctx->list);
			kfree(ctx);
			goto err_drop;
		}

		list_del(&ctx->list);
	}

	spin_unlock_irqrestore(&mctp_usb->tx_spinlock, flags);
	return NETDEV_TX_OK;

err_drop:
	printk("Msg dropped");
	spin_unlock_irqrestore(&mctp_usb->tx_spinlock, flags);
	mctp_usb_stat_tx_dropped(dev);
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static int mctp_usb_open(struct net_device *dev)
{
	return 0;
}

static int mctp_usb_stop(struct net_device *dev)
{
	netif_stop_queue(dev);
	return 0;
}

static const struct net_device_ops mctp_usb_netdev_ops = {
	.ndo_start_xmit = mctp_usb_start_xmit,
	.ndo_open = mctp_usb_open,
	.ndo_stop = mctp_usb_stop,
};

static void mctp_usb_netdev_setup(struct net_device *dev)
{
	dev->type = ARPHRD_MCTP;

	dev->mtu = MCTP_USB_MTU_MIN;
	dev->min_mtu = MCTP_USB_MTU_MIN;
	dev->max_mtu = MCTP_USB_MTU_MAX;
	dev->tx_queue_len = 1100;

	dev->hard_header_len = sizeof(struct mctp_usb_hdr);
	dev->addr_len = 0;
	dev->flags = IFF_NOARP;
	dev->netdev_ops = &mctp_usb_netdev_ops;
	dev->needs_free_netdev = true;
	dev->pcpu_stat_type = NETDEV_PCPU_STAT_DSTATS;
}

static int mctp_usb_probe(struct usb_interface *intf,
			  const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor *ep_in, *ep_out;
	struct usb_host_interface *iface_desc;
	struct net_device *netdev;
	struct mctp_usb *dev;
	unsigned long flags;
	int rc;

	iface_desc = intf->cur_altsetting;

	rc = usb_find_common_endpoints(iface_desc, &ep_in, &ep_out, NULL, NULL);
	if (rc) {
		dev_err(&intf->dev, "invalid endpoints on device?\n");
		return rc;
	}

	netdev = alloc_netdev(sizeof(*dev), "mctpusb%d", NET_NAME_ENUM,
			      mctp_usb_netdev_setup);
	if (!netdev)
		return -ENOMEM;

	dev = netdev_priv(netdev);
	dev->netdev = netdev;
	dev->usbdev = usb_get_dev(interface_to_usbdev(intf));
	dev->intf = intf;
	usb_set_intfdata(intf, dev);

	dev->ep_in = ep_in->bEndpointAddress;
	dev->ep_out = ep_out->bEndpointAddress;

	spin_lock_init(&dev->tx_spinlock);
	INIT_LIST_HEAD(&dev->free_tx_urbs);

	spin_lock_irqsave(&dev->tx_spinlock, flags);
	for (int i = 0; i < MAX_TX_URB_COUNT; i++) {
		struct urb *tx_urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!tx_urb) {
			rc = -ENOMEM;
			spin_unlock_irqrestore(&dev->tx_spinlock, flags);
			goto err_free_urbs;
		}

		list_add(&tx_urb->urb_list, &dev->free_tx_urbs);
	}
	spin_unlock_irqrestore(&dev->tx_spinlock, flags);

	rc = register_netdev(netdev);
	if (rc)
		goto err_free_urbs;

	return 0;

err_free_urbs:
	spin_lock_irqsave(&dev->tx_spinlock, flags);
	while (!list_empty(&dev->free_tx_urbs)) {
		struct urb *tx_urb = list_first_entry(&dev->free_tx_urbs,
						      struct urb, urb_list);
		list_del(&tx_urb->urb_list);

		usb_free_urb(tx_urb);
	}
	spin_unlock_irqrestore(&dev->tx_spinlock, flags);
	free_netdev(netdev);
	return rc;
}

static void mctp_usb_disconnect(struct usb_interface *intf)
{
	struct mctp_usb *dev = usb_get_intfdata(intf);

	unregister_netdev(dev->netdev);
}

static const struct usb_device_id mctp_usb_devices[] = {
	{ USB_INTERFACE_INFO(USB_CLASS_MCTP, 0x0, 0x1) },
	{ 0 },
};

static struct usb_driver mctp_usb_driver = {
	.name = "mctp",
	.id_table = mctp_usb_devices,
	.probe = mctp_usb_probe,
	.disconnect = mctp_usb_disconnect,
};

module_usb_driver(mctp_usb_driver);
MODULE_LICENSE("GPL");