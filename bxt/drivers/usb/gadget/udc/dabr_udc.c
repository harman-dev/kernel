/*
 * dabr_udc.c
 *
 * A driver to expose a device side "USB gadget" API, to direct requests to
 * Delphi's Dual Role USB Bridge/Hub.
 *
 *  Copyright (c) 2015, Delphi Automotive PLC
 *  Created on: Jan 16, 2015
 *      Author: Sam Yeda <sam.yeda@delphi.com>
 *
 *  Revision History
 *  Sam Yeda        26OCT2015,  Beta release.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include "dabridge.h"

#define Dev_Request    (USB_TYPE_STANDARD | USB_RECIP_DEVICE)
#define Dev_InRequest  (Dev_Request | USB_DIR_IN)
#define Intf_Request   (USB_TYPE_STANDARD | USB_RECIP_INTERFACE)
#define Intf_InRequest (Intf_Request | USB_DIR_IN)
#define Ep_Request     (USB_TYPE_STANDARD | USB_RECIP_ENDPOINT)
#define Ep_InRequest   (Ep_Request | USB_DIR_IN)

bool b_host_disconnected = false;
bool b_host_softdisconnect = false;
EXPORT_SYMBOL(b_host_disconnected);
EXPORT_SYMBOL(b_host_softdisconnect);

static const char dabridge_gadget_name[] = "dabr_udc";

static const char *const ctrl_state_name[] = {
	"SETUP     ",
	"DATA_IN   ",
	"DATA_OUT  ",
	"STATUS_IN ",
	"STATUS_OUT",
};

static const struct usb_endpoint_descriptor dabridge_ep0_desc = {
	.bLength          = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  = USB_DT_ENDPOINT,
	.bEndpointAddress = 0,
	.bmAttributes     = USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize   = MAX_USB_EP0_PACKET,
};

static int dabr_ep0_req_setup(struct dabridge_udc *cntroller);
static int dabr_ep0_read(struct dabridge_udc *cntroller,
		struct usb_ctrlrequest *ctlreq, struct usb_request *_req);
static int dabr_ep0_write(struct dabridge_udc *cntroller,
		struct usb_ctrlrequest *ctlreq, struct usb_request *_req);
static int dabr_epX_rw(struct dabridge_udc *cntroller,
		struct dabridge_request *dareq, struct dabr_g_ep *ep);
static int dabr_handle_ep_queue(struct dabridge_udc *cntroller,
		struct dabridge_request *req, struct dabr_g_ep *ep);
static void dabr_stop_all_transfer(struct dabridge_udc *cntroller);
static void dabr_udc_delete(struct kref *kref);

static struct dabridge_udc *dabr_udc = NULL;
static struct platform_device *dabr_udc_pdev;

/*-------------------------------------------------------------------------*/

static void dabr_bhost_state_work(struct work_struct *work)
{
	struct dabridge_udc *cntroller = state_work_to_cntroller(work);

	sysfs_notify(&cntroller->gadget.dev.kobj, NULL, "bhoststate");
}

static inline void dabr_set_bhost_state(struct dabridge_udc *cntroller,
		enum usb_device_state state)
{
	if(cntroller->bhoststate != state) {
		cntroller->bhoststate = state;
		schedule_work(&cntroller->state_work);
	}
}
static inline enum usb_device_state dabr_get_bhost_state(
		struct dabridge_udc *cntroller)
{
	return cntroller->bhoststate;
}

static inline void dabr_print_ep0_setup(struct dabridge_udc *cntroller,
		struct usb_ctrlrequest *setup, const char* prefix)
{

	dev_dbg(udc_dev(cntroller), "%s: %02x.%02x v%04x i%04x l%d %s\n",
			prefix,
			setup->bRequestType, setup->bRequest, setup->wValue,
			setup->wIndex,
			setup->wLength,
			(prefix[1] == 'E') ? ctrl_state_name[cntroller->ctrl_state] : "");

}

/* Empty endpoint request queue and call completion handler */
static void dabr_nuke(struct dabridge_udc *cntroller,
		struct dabr_g_ep *ep, struct urb *urb)
{
	struct dabridge_request *req, *tmp;

	/* iterate the queue reverse order in case we get lucky and the unlink
	 * happens before we reach to unlinked request.
	 * Consequences of calling the callback reverse is unknown */
	list_for_each_entry_safe_reverse (req, tmp, &ep->queue, queue) {
		list_del_init(&req->queue);
		req->req.status = -ESHUTDOWN;
		spin_unlock(&ep->lock);
		req->req.complete(&ep->ep, &req->req);
		spin_lock(&ep->lock);
		/*dev_info (udc_dev(cntroller), "%s nuke req=%p\n", ep->ep.name, &req->req);*/
	}
}

/*
 * Swaps the endpoint address between bridge and gadget
 * {bridge, gadget}
 * {0x81, 0x02},
 * {0x02, 0x81},
 * {0x83, 0x04},
 * {0x04, 0x83},
 * {0x85, 0x06},
 * {0x06, 0x85},
 * {0x87, 0x08},
 * {0x08, 0x87},
 */
static inline u8 dabr_epaddr_swap(u8 caddr)
{
	unsigned long addr = caddr;
	if ((addr & ~USB_DIR_IN) != 0) {
		/* odd to even and vice versa number conversion */
		(addr & 1) ? addr++ : addr--;
		/* Direction bit flip */
		change_bit(7, &addr);
	}
	return (u8)addr;
}

static const struct usb_endpoint_descriptor * g_ep_to_bridge_ep(
		struct dabridge_udc *cntroller, u8 addr)
{
	int i;
	struct usb_host_interface *iface_desc;

	if ((addr & ~USB_DIR_IN) == 0)
		return cntroller->eps[0].desc;

	iface_desc = cntroller->bridgedev->interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		if (iface_desc->endpoint[i].desc.bEndpointAddress == addr)
			return &iface_desc->endpoint[i].desc;
	}
	dev_err(udc_dev(cntroller), "No bridge endpoint for address=0x%02x\n",
			addr);
	return NULL;
}

/*----------------------GADGET SIDE DRIVER---------------------------*/

static int dabr_ep_enable (struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct dabridge_udc *cntroller;
	struct dabr_g_ep *ep;
	unsigned max;
	int retval;

	ep = usb_ep_to_dabr_g_ep (_ep);
	if (!_ep || !desc || ep->desc || _ep->name == dabr_ep0name
			|| desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;

	cntroller = ep_to_cntroller (ep);
	if(!cntroller->driver)
		return -ESHUTDOWN;

	dev_vdbg(udc_dev(cntroller), "enable %s\n",_ep->name);
	/*
	 * For HS/FS devices only bits 0..10 of the wMaxPacketSize
	 * represent the maximum packet size.
	 * For SS devices the wMaxPacketSize is limited by 1024.
	 */
	max = usb_endpoint_maxp(desc) & 0x7ff;

	retval = -EINVAL;
	/* TODO: Cross check with hardware reported max packet */
	switch (desc->bmAttributes & 0x03) {
	case USB_ENDPOINT_XFER_BULK:
		if (max > 512)
			goto done;
		break;
	case USB_ENDPOINT_XFER_INT:
		if (max > 64)
			goto done;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		if (max > 1024)
			goto done;
		break;
	default:
		goto done;
	}

	if(cntroller->bridgedev->info->isoc_out_ep &&
			(desc->bEndpointAddress == cntroller->bridgedev->info->isoc_out_ep)) {
		/*ISOC endpoint override*/
		ep->br_desc = g_ep_to_bridge_ep(cntroller,
				cntroller->bridgedev->info->isoc_in_ep);
	} else {
		ep->br_desc = g_ep_to_bridge_ep(cntroller,
				dabr_epaddr_swap(desc->bEndpointAddress)
				/*gadget to bridge ep address*/);
	}
	if (!ep->br_desc) {
		dev_err(udc_dev(cntroller),
				"Attempted to enable invalid ep=0x%02x\n",
				desc->bEndpointAddress);
		retval = -EINVAL;
		goto done;
	}

	_ep->maxpacket = max;
	ep->desc = desc;

	dev_dbg(udc_dev(cntroller),
		"enabled %s (bridge ep%d%s-%s) maxpacket %d\n",
		_ep->name,
		ep->br_desc->bEndpointAddress & 0x0f,
		(ep->br_desc->bEndpointAddress & USB_DIR_IN) ? "in" : "out",
		({ char *val;
		 switch (ep->br_desc->bmAttributes & 0x03) {
		 case USB_ENDPOINT_XFER_BULK: val = "bulk"; break;
		 case USB_ENDPOINT_XFER_ISOC: val = "iso"; break;
		 case USB_ENDPOINT_XFER_INT: val = "int"; break;
		 default: val = "ctrl"; break;
		 }; val; }),
		max);

	ep->halted = ep->wedged = 0;
	return 0;

done:
	dev_err(udc_dev(cntroller),
		"error enabling %s maxpacket %d\n", _ep->name, max);
	return retval;
}

static int dabr_ep_disable(struct usb_ep *_ep)
{
	struct dabr_g_ep *ep;
	struct dabridge_udc *cntroller;
	struct urb *unlinkedurb = NULL;
	unsigned long flags;
	int retval;

	ep = usb_ep_to_dabr_g_ep (_ep);
	if (!_ep || !ep->desc || _ep->name == dabr_ep0name)
		return -EINVAL;

	cntroller = ep_to_cntroller (ep);

	/* Stop any submitted URB's */
	if (cntroller->state == DABRIDGE_UDC_RUNNING) {
		usb_unlink_anchored_urbs(&ep->submitted);
	}

	spin_lock_irqsave(&ep->lock, flags);
	/* Empty endpoint request queue and call completion handler */
	dabr_nuke(cntroller, ep, unlinkedurb);

	ep->desc = NULL;
	ep->br_desc = NULL;
	retval = 0;
	spin_unlock_irqrestore(&ep->lock, flags);

	dev_dbg (udc_dev(cntroller), "ep_disable %s\n", _ep->name);
	return retval;
}

static struct usb_request *dabr_ep_alloc_request(struct usb_ep *_ep,
		gfp_t mem_flags)
{
	struct dabr_g_ep *ep;
	struct dabridge_request *req;

	if (!_ep)
		return NULL;
	ep = usb_ep_to_dabr_g_ep (_ep);

	req = kzalloc(sizeof(*req), mem_flags);
	if (!req) {
		dev_err(udc_dev(ep_to_cntroller(ep)),
				"Could not allocate %s\n", _ep->name);
		return NULL;
	}
	INIT_LIST_HEAD (&req->queue);
	usb_init_urb(&req->urb);
	req->g_ep = ep;

	dev_dbg (udc_dev(ep_to_cntroller(ep)),
		"ep_alloc_request %s, req %p, urb %p, mem_flags 0x%02X\n",
		_ep->name, &req->req, &req->urb, mem_flags);
	return &req->req;
}

static void dabr_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct dabr_g_ep *ep;
	struct dabridge_request *req;
	unsigned long flags;

	ep = usb_ep_to_dabr_g_ep (_ep);
	if (!ep || !_req) {
		WARN_ON(1);
		return;
	}

	req = usb_request_to_dabridge_request (_req);
	if(!list_empty (&req->queue))
	{
		spin_lock_irqsave(&ep->lock, flags);
		list_for_each_entry (req, &ep->queue, queue) {
			if (&req->req == _req) {
				list_del_init (&req->queue);
				_req->status = -ESHUTDOWN;
				break;
			}
		}
		spin_unlock_irqrestore(&ep->lock, flags);
	}
	WARN_ON (!list_empty (&req->queue));
	dev_dbg (udc_dev(ep_to_cntroller(ep)),
		"ep_free_request %s, req %p, urb %p\n",
		_ep->name, &req->req, &req->urb);
	kfree (req);
}

/* queues (submits) an I/O request to an endpoint */
static int dabr_ep_queue(struct usb_ep *_ep, struct usb_request *_req,
		gfp_t mem_flags)
{
	struct dabr_g_ep *ep;
	struct dabridge_request *req;
	struct dabridge_udc *cntroller;
	unsigned long flags;
	int retval = 0;
	req = usb_request_to_dabridge_request (_req);
	if (!_req || !list_empty (&req->queue) || !_req->complete)
		return -EINVAL;

	ep = usb_ep_to_dabr_g_ep (_ep);
	if (!_ep || (!ep->desc && _ep->name != dabr_ep0name))
		return -EINVAL;

	cntroller = ep_to_cntroller (ep);
	if (!cntroller->driver )
		return -ESHUTDOWN;

/* 	dev_vdbg (udc_dev(cntroller),
			"ep %p queue req %p, urb %p to %s, len %d buf %p, mem_flags 0x%02X\n",
			ep, _req, &req->urb, _ep->name, _req->length, _req->buf, mem_flags);
 */
	if (b_host_disconnected || b_host_softdisconnect) {
		if(printk_ratelimit()) {
			 dev_err(udc_dev(cntroller), "B-Host disconnected for %s\n", _ep->name);
		}
		return -ESHUTDOWN;
	}
	_req->status = -EINPROGRESS;
	_req->actual = 0;
	req->mem_flags = mem_flags;

	spin_lock_irqsave(&ep->lock, flags);
	list_add_tail(&req->queue, &ep->queue);
	spin_unlock_irqrestore(&ep->lock, flags);

	retval =  dabr_handle_ep_queue(cntroller, req, ep);
	if (retval < 0) {
		if(printk_ratelimit()) {
			dev_err(udc_dev(cntroller),"ep %s req %p queue failed with status %d", _ep->name, req, retval);
		}
		spin_lock_irqsave(&ep->lock, flags);
		list_del_init(&req->queue);
		spin_unlock_irqrestore(&ep->lock, flags);
	}

	return retval;
}

/* dequeues (cancels, unlinks) an I/O request from an endpoint */
static int dabr_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct dabr_g_ep *ep;
	struct dabridge_udc *cntroller;
	int retval = -EINVAL;
	unsigned long flags;
	struct dabridge_request *req = NULL;

	if (!_ep || !_req)
		return retval;
	ep = usb_ep_to_dabr_g_ep (_ep);
	cntroller = ep_to_cntroller (ep);

	if (!cntroller->driver)
		return -ESHUTDOWN;

	spin_lock_irqsave(&ep->lock, flags);
	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req) {
			list_del_init (&req->queue);
			_req->status = -ECONNRESET;
			retval = 0;
			break;
		}
	}

	if (ep != &cntroller->eps[0]) {
		usb_unanchor_urb(&req->urb);
		usb_unlink_urb(&req->urb);
	}

	if (retval == 0) {
		dev_dbg (udc_dev(cntroller),
			"dequeued req %p from %s, len %d buf %p\n",
			req, _ep->name, _req->length, _req->buf);
		spin_unlock(&ep->lock);
		_req->complete (_ep, _req);
		spin_lock(&ep->lock);
	}
	spin_unlock_irqrestore(&ep->lock, flags);
	return retval;
}

static int dabr_ep_set_halt_and_wedge(struct usb_ep *_ep, int value, int wedged)
{
	struct dabr_g_ep *ep;
	struct dabridge_udc *cntroller;

	if (!_ep)
		return -EINVAL;
	ep = usb_ep_to_dabr_g_ep (_ep);
	cntroller = ep_to_cntroller (ep);
	if (!cntroller->driver)
		return -ESHUTDOWN;
	if (!value)
		ep->halted = ep->wedged = 0;
	else if (ep->desc && (ep->desc->bEndpointAddress & USB_DIR_IN) &&
			!list_empty (&ep->queue))
		return -EAGAIN;
	else {
		ep->halted = 1;
		if (wedged)
			ep->wedged = 1;
	}
	dev_warn (udc_dev(cntroller),"%s %s%s\n", _ep->name,
			(value)?"halted":"halt and wedge cleared",
			(wedged)?" and wedged":"");
	return 0;
}

static int dabr_ep_set_halt(struct usb_ep *_ep, int value)
{
	return dabr_ep_set_halt_and_wedge(_ep, value, 0);
}

static int dabr_ep_set_wedge(struct usb_ep *_ep)
{
	if (!_ep || _ep->name == dabr_ep0name)
		return -EINVAL;
	return dabr_ep_set_halt_and_wedge(_ep, 1, 1);
}

static const struct usb_ep_ops dabr_ep_ops = {
	.enable    = dabr_ep_enable,
	.disable   = dabr_ep_disable,
	.alloc_request = dabr_ep_alloc_request,
	.free_request  = dabr_ep_free_request,
	.queue     = dabr_ep_queue,
	.dequeue   = dabr_ep_dequeue,
	.set_halt  = dabr_ep_set_halt,
	.set_wedge = dabr_ep_set_wedge,
};

/*-------------------------------------------------------------------------*/

/* there are both host and device side versions of this call ... */
static int dabr_udc_get_frame (struct usb_gadget *_gadget)
{
	struct timeval tv;

	do_gettimeofday (&tv);
	return tv.tv_usec / 1000;
}

static int dabr_udc_wakeup (struct usb_gadget *_gadget)
{
	struct dabridge_udc *cntroller;

	cntroller = gadget_to_cntroller(_gadget);
	if (!(cntroller->devstatus & ((1 << USB_DEVICE_B_HNP_ENABLE)
				| (1 << USB_DEVICE_REMOTE_WAKEUP))))
		return -EINVAL;

	cntroller->resuming = 1;
	return 0;
}

static int dabr_udc_set_selfpowered (struct usb_gadget *_gadget,
		int value)
{
	struct dabridge_udc *cntroller;

	cntroller = gadget_to_cntroller(_gadget);
	if (value)
		cntroller->devstatus |= (1 << USB_DEVICE_SELF_POWERED);
	else
		cntroller->devstatus &= ~(1 << USB_DEVICE_SELF_POWERED);

	return 0;
}

static void dabr_udc_update_ep0(struct dabridge_udc *cntroller)
{
	u32 i;
	for (i = 1; i < cntroller->bridgedev->info->num_of_ep_names; i++) {
		cntroller->eps[i].ep.max_streams = 0;
	}
	cntroller->eps[0].ep.maxpacket = MAX_USB_EP0_PACKET;
}

static int dabr_udc_pullup (struct usb_gadget *_gadget, int value)
{
	struct dabridge_udc *cntroller;
	struct dabr_g_ep *ep;
	unsigned long flags;

	cntroller = gadget_dev_to_cntroller(&_gadget->dev);

	if(!cntroller->bridgedev) {
		dev_err(udc_dev(cntroller),"%s : no dabridge available\n",__func__);
		return -ENODEV;
	}

	spin_lock_irqsave (&cntroller->lock, flags);
	if (value && cntroller->driver) {
		dabr_udc_update_ep0(cntroller);
		dev_dbg(udc_dev(cntroller),
				"speed info -> gadget(%s) and driver(%s)\n",
				usb_speed_string(cntroller->gadget.speed),
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
				usb_speed_string(cntroller->driver->max_speed));
#else
				usb_speed_string(cntroller->driver->speed));
#endif
	}

	cntroller->pullup = (value != 0);
	spin_unlock_irqrestore (&cntroller->lock, flags);

	if (value && cntroller->driver &&
			cntroller->state == DABRIDGE_UDC_RUNNING) {
		b_host_softdisconnect = false;
		b_host_disconnected = false;
	}

	/* Stop any pending Ctrl requests */
	if (!value && cntroller->driver &&
			cntroller->state == DABRIDGE_UDC_RUNNING) {

		if(!b_host_disconnected)
			b_host_softdisconnect = true;

		usb_kill_anchored_urbs(&cntroller->eps[0].submitted);

		/* unlink any submitted requests */
		list_for_each_entry(ep, &cntroller->gadget.ep_list, ep.ep_list) {
			if(ep->desc) {
				usb_kill_anchored_urbs(&ep->submitted);
			}
		}
	}

	dev_dbg (udc_dev(cntroller), "udc pulled %s\n", (value)?"up":"down");
	return 0;
}

/*
 * Driver registration/unregistration.
 */
static int dabr_udc_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct dabridge_udc *cntroller = gadget_to_cntroller(g);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
	if (driver->max_speed == USB_SPEED_UNKNOWN)
#else
	if (driver->speed == USB_SPEED_UNKNOWN)
#endif
		return -EINVAL;

	cntroller->devstatus = 0;

	cntroller->driver = driver;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
	cntroller->gadget.dev.driver = &driver->driver;
#endif
	dev_dbg (udc_dev(cntroller), "register gadget driver '%s'\n",
			driver->driver.name);

	/* increment our usage count */
	kref_get(&cntroller->kref);
	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(3,14,0)
static int dabr_udc_stop(struct usb_gadget *g) {
       struct dabridge_udc *cntroller = gadget_to_cntroller(g);

       dev_dbg (udc_dev(cntroller), "unregister gadget driver '%s'\n",
			(cntroller->driver) ? cntroller->driver->driver.name: "None");

       dabr_set_bhost_state(cntroller, USB_STATE_NOTATTACHED);
       cntroller->driver = NULL;

        /* decrement our usage count */
       kref_put(&cntroller->kref, dabr_udc_delete);
       return 0;
}
#else
static int dabr_udc_stop(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct dabridge_udc *cntroller = gadget_to_cntroller(g);

	dev_dbg (udc_dev(cntroller), "unregister gadget driver '%s'\n",
			(cntroller->driver) ? cntroller->driver->driver.name: "None");

	if(driver && cntroller->driver && cntroller->driver->disconnect) {
		cntroller->driver->disconnect(&cntroller->gadget);
	}
	dabr_set_bhost_state(cntroller, USB_STATE_NOTATTACHED);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
	cntroller->gadget.dev.driver = NULL;
#endif
	cntroller->driver = NULL;

	/* decrement our usage count */
	kref_put(&cntroller->kref, dabr_udc_delete);
	return 0;
}
#endif

static int dabr_udc_vbus_draw(struct usb_gadget *g, unsigned mA)
{
	struct dabridge_udc *cntroller = gadget_dev_to_cntroller(g);
	return 0;
}

static const struct usb_gadget_ops dabr_udc_ops = {
	.get_frame = dabr_udc_get_frame,
	.wakeup    = dabr_udc_wakeup,
	.set_selfpowered = dabr_udc_set_selfpowered,
	.vbus_draw = dabr_udc_vbus_draw,
	.pullup    = dabr_udc_pullup,
	.udc_start = dabr_udc_start,
	.udc_stop  = dabr_udc_stop,
};

/*-------------------------sysfs attributes--------------------------------*/

static ssize_t show_bhoststate(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dabridge_udc *cntroller = gadget_dev_to_cntroller (dev);

	return sprintf(buf, "%d\n", dabr_get_bhost_state(cntroller));
}
static DEVICE_ATTR (bhoststate, S_IRUGO|S_IRUSR|S_IRGRP, show_bhoststate, NULL);

static ssize_t show_function (struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dabridge_udc *cntroller = gadget_dev_to_cntroller (dev);

	if (!cntroller->driver || !cntroller->driver->function)
		return 0;
	return scnprintf (buf, PAGE_SIZE, "%s\n", cntroller->driver->function);
}
static DEVICE_ATTR (function, S_IRUGO|S_IRUSR|S_IRGRP, show_function, NULL);

static ssize_t show_bridgestatus(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dabridge_udc *cntroller = gadget_dev_to_cntroller(dev);
	struct dabridge_request *req;
	int i;
	unsigned long flags;

	spin_lock_irqsave(&cntroller->lock, flags);
	for (i = 1; i < cntroller->bridgedev->info->num_of_ep_names; i++) {
		struct dabr_g_ep *ep = &cntroller->eps [i];

		if (!ep->desc) {
			continue;
		}
		scnprintf (buf, PAGE_SIZE, "%sB-Host %s queue:\n",
				buf, ep->ep.name);
		list_for_each_entry (req, &ep->queue, queue) {
			scnprintf (buf, PAGE_SIZE,
					"%s  request %p (%d)\n",
					buf, &req->req, req->req.length);
		}
	}
	spin_unlock_irqrestore(&cntroller->lock, flags);

	return strlen(buf);
}
static DEVICE_ATTR(bridgestatus, S_IRUGO|S_IRUSR|S_IRGRP, show_bridgestatus, NULL);

static struct attribute *dabr_udc_attrs[] = {
	&dev_attr_function.attr,
	&dev_attr_bridgestatus.attr,
	&dev_attr_bhoststate.attr,
	NULL
};

static struct attribute_group dabr_udc_attr_grp = {
	.attrs = dabr_udc_attrs,
};

/*-------------------------------------------------------------------------*/

#undef is_enabled

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
/* The gadget structure is stored inside the hcd structure and will be
 * released along with it. */
static void dabr_gadget_release (struct device *dev) {
	return;
}
#endif

static int dabr_udc_init_eps(struct dabridge_udc *cntroller)
{
	int i;

	if(!cntroller->bridgedev->info->num_of_ep_names)
		return -1;

	INIT_LIST_HEAD(&cntroller->gadget.ep_list);
	for (i = 0; i < cntroller->bridgedev->info->num_of_ep_names; i++) {
		struct dabr_g_ep *ep = &cntroller->eps[i];

		ep->ep.name = cntroller->bridgedev->info->ep_names[i];
		dev_dbg(udc_dev(cntroller), "ep_list[%d] = %s\n",
				i, ep->ep.name);

		ep->ep.ops = &dabr_ep_ops;
		ep->ep.caps = cntroller->bridgedev->info->ep_cap[i];
		list_add_tail(&ep->ep.ep_list, &cntroller->gadget.ep_list);
		ep->halted = ep->wedged = 0;
		spin_lock_init(&ep->lock);
		init_usb_anchor(&ep->submitted);
		/*maxpacket is set by epautoconfig() called by gadget layer*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
		ep->ep.maxpacket = ~0;
#else
		usb_ep_set_maxpacket_limit(&ep->ep, ~0);
#endif
		ep->last_io = jiffies;
		ep->gadget = &cntroller->gadget;
		ep->desc = NULL;
		INIT_LIST_HEAD(&ep->queue);
	}
	/* for ep0: the desc defined here */
	cntroller->eps[0].br_desc = &dabridge_ep0_desc;
	cntroller->eps[0].desc = &dabridge_ep0_desc;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
	cntroller->eps[0].ep.maxpacket = MAX_USB_EP0_PACKET;
#else
	usb_ep_set_maxpacket_limit(&cntroller->eps[0].ep, MAX_USB_EP0_PACKET);
#endif
	cntroller->gadget.ep0 = &cntroller->eps[0].ep;
	/* don't add to gadget's ep_list */
	list_del_init(&cntroller->eps[0].ep.ep_list);

#ifdef CONFIG_USB_OTG
	cntroller->gadget.is_otg = 1;
#endif
	return 0;
}

/*-------------------------------------------------------------------------*/

static struct dabr_g_ep *dabr_find_endpoint (struct dabridge_udc *cntroller,
		u8 address)
{
	int i;

	/* Always allow Control endpoint since it is used for
	 * communicating with Bridge device */
	if ((address & ~USB_DIR_IN) == 0)
		return &cntroller->eps [0];

	for (i = 1; i < cntroller->bridgedev->info->num_of_ep_names; i++) {
		struct dabr_g_ep *ep = &cntroller->eps [i];

		if (!ep->desc) {
			continue;
		}
		if (ep->desc->bEndpointAddress == address)
			return ep;
	}
	dev_vdbg(udc_dev(cntroller), "No ep match found\n");
	return NULL;
}

static void dabr_stop_all_transfer(struct dabridge_udc *cntroller)
{
	struct dabr_g_ep *ep;

	/* unlink any submitted requests */
	list_for_each_entry(ep, &cntroller->gadget.ep_list, ep.ep_list) {
		if(ep->desc) {
			usb_unlink_anchored_urbs(&ep->submitted);
		}
	}
}

static int dabr_handle_ep_queue(struct dabridge_udc *cntroller,
		struct dabridge_request *req,
		struct dabr_g_ep *ep)
{
	int retval = -EBUSY;

	if (ep == &cntroller->eps[0]) {
		cntroller->ahost_ctrl_rsp.wLength = req->req.length;
		if (cntroller->ctrl_state == DABR_CTRL_DATA_IN
				|| cntroller->ctrl_state == DABR_CTRL_STATUS_IN) {

			cntroller->ahost_ctrl_rsp.wIndex = (
					(cntroller->ahost_ctrl_rsp.bRequestType &
					USB_TYPE_MASK) << 7) |
					(cntroller->ahost_ctrl_rsp.bRequest << 4) |
					PROCESS_WVALUE_DATA |
					SEND_DATA_TO_PHY_HOST;

			switch (cntroller->ahost_ctrl_rsp.bRequest) {
			case USB_REQ_SET_CONFIGURATION:
			case USB_REQ_SET_INTERFACE:
				cntroller->ahost_ctrl_rsp.wIndex |= STORE_WVALUE_IN_FPGA;
				dabr_set_bhost_state(cntroller, USB_STATE_CONFIGURED);
				break;
			}
			retval = dabr_ep0_write(cntroller, &cntroller->ahost_ctrl_rsp,
					&req->req);
		} else {
			retval = dabr_ep0_read(cntroller, &cntroller->ahost_ctrl_rsp,
					&req->req);
		}
	} else {
		retval = dabr_epX_rw(cntroller, req, ep);
	}

	return retval;
}

/**
 * dabr_handle_ctrl_req() - handles all incoming control transfers
 * @cntroller: pointer to dabridge_udc (dabr_udc)
 * @urb: the urb request to handle
 * @setup: pointer to the setup data for a USB device control
 *	 request
 *
 * Return 0 - if the request was handled
 *	  1 - if the request wasn't handles
 *	  error code on error
 */
static int dabr_handle_ctrl_req(struct dabridge_udc *cntroller,
		struct usb_ctrlrequest *setup)
{
	struct dabr_g_ep *ep2;
	int ret_val = 1;
	unsigned w_index;
	unsigned w_value;
	char buf[2];
	int save2fpga = 0;

	dabr_print_ep0_setup(cntroller, setup, ctrl_state_name[0]);

	w_index = le16_to_cpu(setup->wIndex);
	w_value = le16_to_cpu(setup->wValue);
	switch (setup->bRequest) {
	case USB_REQ_SET_ADDRESS:
		if (setup->bRequestType != Dev_Request)
			break;
		cntroller->address = w_value;
		dev_info(udc_dev(cntroller),
				"B-Host sent function address = %d\n",
				w_value);
		if(cntroller->driver && cntroller->driver->resume) {
			cntroller->driver->resume(&cntroller->gadget);
		}
		dabr_set_bhost_state(cntroller, USB_STATE_ADDRESS);
		save2fpga = 1;
		ret_val = 0;
		break;
	case USB_REQ_SET_FEATURE:
		if (setup->bRequestType == Dev_Request) {
			ret_val = 0;
			switch (w_value) {
			case USB_DEVICE_REMOTE_WAKEUP:
				break;
			case USB_DEVICE_B_HNP_ENABLE:
				cntroller->gadget.b_hnp_enable = 1;
				break;
			case USB_DEVICE_A_HNP_SUPPORT:
				cntroller->gadget.a_hnp_support = 1;
				break;
			case USB_DEVICE_A_ALT_HNP_SUPPORT:
				cntroller->gadget.a_alt_hnp_support = 1;
				break;
			default:
				ret_val = -EOPNOTSUPP;
			}
			if (ret_val == 0) {
				cntroller->devstatus |= (1 << w_value);
				dev_warn(udc_dev(cntroller),
						"set devstatus = 0x%x\n",
						cntroller->devstatus);
			}
		} else if (setup->bRequestType == Ep_Request) {
			/* endpoint halt */
			ep2 = dabr_find_endpoint(cntroller, w_index);
			if (!ep2 || ep2->ep.name == dabr_ep0name) {
				ret_val = -EOPNOTSUPP;
				break;
			}
			ep2->halted = 1;
			ret_val = 0;
			dev_warn(udc_dev(cntroller),
					"set %s halt\n",
					ep2->ep.name);
		}
		break;
	case USB_REQ_CLEAR_FEATURE:
		if (setup->bRequestType == Dev_Request) {
			ret_val = 0;
			switch (w_value) {
			case USB_DEVICE_REMOTE_WAKEUP:
				w_value = USB_DEVICE_REMOTE_WAKEUP;
				break;
			default:
				ret_val = -EOPNOTSUPP;
				break;
			}
			if (ret_val == 0) {
				cntroller->devstatus &= ~(1 << w_value);
				dev_warn(udc_dev(cntroller),
						"clear devstatus = 0x%x\n",
						cntroller->devstatus);
			}
		} else if (setup->bRequestType == Ep_Request) {
			/* endpoint halt */
			ep2 = dabr_find_endpoint(cntroller, w_index);
			if (!ep2) {
				ret_val = -EOPNOTSUPP;
				break;
			}
			if (!ep2->wedged)
				ep2->halted = 0;
			ret_val = 0;
			dev_warn(udc_dev(cntroller),
					"clear %s halt\n",
					ep2->ep.name);
		}
		break;
	case USB_REQ_GET_STATUS:
		if (setup->bRequestType == Dev_InRequest
				|| setup->bRequestType == Intf_InRequest
				|| setup->bRequestType == Ep_InRequest) {
			/*
			 * device: remote wakeup, selfpowered
			 * interface: nothing
			 * endpoint: halt
			 */
			if (setup->bRequestType == Ep_InRequest) {
				ep2 = dabr_find_endpoint(cntroller, w_index);
				if (!ep2) {
					ret_val = -EOPNOTSUPP;
					break;
				}
				buf[0] = ep2->halted;
			} else if (setup->bRequestType == Dev_InRequest) {
				buf[0] = (u8) cntroller->devstatus;
			} else {
				buf[0] = 0;
			}

			buf[1] = 0;
			setup->wLength = 2;
			ret_val = 0;
			dev_warn(udc_dev(cntroller),
					"get status 0x%x%x\n", buf[0], buf[1]);
			cntroller->ctrl_state = DABR_CTRL_DATA_IN;
		}
		break;
	case USB_REQ_BHOST_DISCONNECTED: /*B-Host(Relay Device) disconnected*/
		if ((setup->bRequestType == 0xff) &&
				(w_value == 0xffff) && (w_index == 0x0000)) {

			/* Don't report BHost disconnect if we never were
			 * in addressed state */
			if (dabr_get_bhost_state(cntroller) < USB_STATE_ADDRESS) {
				/*TODO: Start a delayed control request task */
				/* Queue up a control request from B-Host */
				dabr_ep0_req_setup(cntroller);
				return 0;
			}

			if (dabr_get_bhost_state(cntroller) == USB_STATE_ADDRESS) {
				dabr_ep0_req_setup(cntroller);
				return 0;
			}

			dev_info(udc_dev(cntroller),
					"B-Host (Relay Device) disconnected\n");
			if(cntroller->driver && cntroller->driver->suspend) {
				b_host_disconnected = true;
				cntroller->driver->suspend(&cntroller->gadget);
			}
			dabr_set_bhost_state(cntroller, USB_STATE_NOTATTACHED);
			dabr_stop_all_transfer(cntroller);
			return 0;
		}
		break;
	case USB_REQ_GET_DESCRIPTOR:
		/* In case we never receive USB_REQ_SET_ADDRESS*/
		if (dabr_get_bhost_state(cntroller) < USB_STATE_ADDRESS)
			dabr_set_bhost_state(cntroller, USB_STATE_ADDRESS);
		break;
	}

	if(ret_val == 0) {
		setup->wIndex = (setup->bRequest << 4) |
				PROCESS_WVALUE_DATA | SEND_DATA_TO_PHY_HOST;
		if(save2fpga)
			setup->wIndex |= STORE_WVALUE_IN_FPGA;

		cntroller->bhost_usb_req->length = setup->wLength;
		memcpy(cntroller->bhost_usb_req->buf, buf, sizeof(buf));
		dabr_ep0_write(cntroller, setup, cntroller->bhost_usb_req);
	}
	return ret_val;
}

/*
 * transfer data between URB and usb_request; caller must own lock
 * Return 1 to request setup
 */
static int dabr_urb_transfer(struct dabridge_udc *cntroller,
		struct urb *urb, struct dabr_g_ep *ep)
{
	struct dabridge_request *req;
	unsigned long flags;

	if (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct dabridge_request, queue);
/*		dev_vdbg(udc_dev(cntroller),
				"transfer between urb_ep%d%s(%d) and req %p to %s(%d) status(%d)\n",
				usb_endpoint_num(&urb->ep->desc),
				usb_pipein(urb->pipe) ? "in" : "out",
				urb->actual_length,
				&req->req,
				ep->ep.name,
				req->req.length,
				urb->status);
*/
		/* gadget driver completion */
		list_del_init(&req->queue);

		if(usb_pipeisoc(urb->pipe)) {
			dev_vdbg(udc_dev(cntroller),
					"iso sof=%d err_cnt=%d status=%d len=%d\n",
					urb->start_frame,
					urb->error_count,
					urb->iso_frame_desc[0].status,
					urb->iso_frame_desc[0].actual_length);

			req->req.actual = urb->iso_frame_desc[0].actual_length;
			req->req.status = urb->iso_frame_desc[0].status;
		} else {
			req->req.actual = urb->actual_length;
			if(req->req.status != -EINPROGRESS) {
				dev_warn(udc_dev(cntroller),
						"not a valid queued %s req %p\n",
						ep->ep.name, &req->req);
			}
			req->req.status = urb->status;
		}

		if(cntroller->driver && req->req.complete) {
			spin_unlock(&ep->lock);
			req->req.complete(&ep->ep, &req->req);
			spin_lock(&ep->lock);
		}
	}

	if(ep == &cntroller->eps[0]) {
		if(usb_pipeout(urb->pipe)) {
			/* Queue up a control request from B-Host */
			spin_unlock(&ep->lock);
			dabr_ep0_req_setup(cntroller);
			spin_lock(&ep->lock);
		} else if(cntroller->ctrl_state == DABR_CTRL_DATA_OUT) {
			cntroller->bhost_usb_req->length = 0;
			cntroller->ahost_ctrl_rsp.wIndex = (
					(cntroller->ahost_ctrl_rsp.bRequestType &
					USB_TYPE_MASK) << 7) |
					(cntroller->ahost_ctrl_rsp.bRequest << 4) |
					PROCESS_WVALUE_DATA |
					SEND_DATA_TO_PHY_HOST;
			cntroller->ahost_ctrl_rsp.wLength = 0;
			spin_unlock(&ep->lock);
			dabr_ep0_write(cntroller, &cntroller->ahost_ctrl_rsp,
					cntroller->bhost_usb_req);
			spin_lock(&ep->lock);
		}
	}

	return 0;
}

static void dabr_urb_handler(struct dabridge_udc *cntroller,
		struct urb *urb, u8 address)
{

	unsigned long flags;
	struct dabridge_request *req;
	struct dabr_g_ep *ep = NULL;

	spin_lock_irqsave(&cntroller->lock, flags);

	if (cntroller->state != DABRIDGE_UDC_RUNNING) {
		spin_unlock_irqrestore(&cntroller->lock, flags);
		return;
	}

	/* find the gadget's ep for this request (if configured) */
	ep = urb_to_dabridge_request(urb)->g_ep;

	spin_unlock_irqrestore(&cntroller->lock, flags);

	if (b_host_disconnected)
		return;

	if (urb->unlinked) {
		dev_dbg(udc_dev(cntroller),
				"%s unlinked %d, urb %p status %d\n",
				ep->ep.name, urb->unlinked, urb, urb->status);
		return;
	}

	spin_lock_irqsave(&ep->lock, flags);

	if (ep->halted) {
		/* NOTE: must not be iso! */
		dev_err(udc_dev(cntroller),
				"ep %s halted, urb %p\n", ep->ep.name, urb);
		/* TODO: Remove from queue */
		goto exit_handler;
	}

	if (ep == &cntroller->eps[0] && usb_pipein(urb->pipe)
			&& cntroller->ctrl_state != DABR_CTRL_DATA_OUT) {
		/* Received control URB without DABR_CTRL_DATA_OUT*/
		int value = 1;

		/* paranoia, in case of stale queued data */
		list_for_each_entry (req, &ep->queue, queue)
		{
			list_del_init(&req->queue);
			req->req.status = -EOVERFLOW;
			dev_warn(udc_dev(cntroller), "stale req = %p\n", req);
			if(cntroller->driver && req->req.complete) {
				spin_unlock(&ep->lock);
				req->req.complete(&ep->ep, &req->req);
				spin_lock(&ep->lock);
			}
			goto exit_handler;
		}

		/* gadget driver never sees set_address or operations
		 * on standard feature flags.  some hardware doesn't
		 * even expose them.
		 */
		ep->last_io = jiffies;
		ep->halted = 0;

		if(urb->status == -EPROTO) {
			dev_err(udc_dev(cntroller),
					"Setup packet protocol error\n");
			goto exit_handler;
		}

		if(urb->actual_length != sizeof(struct usb_ctrlrequest)) {
			/* Queue up a control request from B-Host */
			dev_err(udc_dev(cntroller),
					"Invalid setup packet length %d, status %d\n",
					urb->actual_length, urb->status);
			dabr_ep0_req_setup(cntroller);
			goto exit_handler;
		}

		memcpy(&cntroller->ahost_ctrl_rsp, urb->transfer_buffer,
				urb->actual_length);

		if (cntroller->ahost_ctrl_rsp.bRequestType & USB_DIR_IN) {
			/*
			 * The USB 2.0 spec states that "if wLength is
			 * zero, there is no data transfer phase."
			 * However, testusb #14 seems to actually
			 * expect a data phase even if wLength = 0...
			 */
			cntroller->ctrl_state = DABR_CTRL_DATA_IN;
		} else {
			if (cntroller->ahost_ctrl_rsp.wLength != cpu_to_le16(0)) {
				/* gadget driver is expected to issue
				 * a read control data request
				 */
				cntroller->ctrl_state = DABR_CTRL_DATA_OUT;
			} else {
				cntroller->ctrl_state = DABR_CTRL_STATUS_IN;
			}
		}

		value = dabr_handle_ctrl_req(cntroller, &cntroller->ahost_ctrl_rsp);

		if (value > 0) {
			/* gadget driver handles all other requests.  block
			 * until setup() returns; no reentrancy issues etc.
			 */
			if(cntroller->driver && cntroller->driver->setup) {
				spin_unlock(&ep->lock);
				value = cntroller->driver->setup(
						&cntroller->gadget,
						&cntroller->ahost_ctrl_rsp);
				spin_lock(&ep->lock);
			} else {
				value = -1;
			}

			/* error, see below */
		}

		if (value < 0) {
			dev_err(udc_dev(cntroller),
					"Unhandled Setup(%d): %02x.%02x v%04x i%04x l%d %s\n",
					value, cntroller->ahost_ctrl_rsp.bRequestType,
					cntroller->ahost_ctrl_rsp.bRequest,
					cntroller->ahost_ctrl_rsp.wValue,
					cntroller->ahost_ctrl_rsp.wIndex,
					cntroller->ahost_ctrl_rsp.wLength,
					ctrl_state_name[cntroller->ctrl_state]);
			/* Queue up a control request from B-Host */
			dabr_ep0_req_setup(cntroller);
			goto exit_handler;
		}

	} else {
		/* Transmitted URB */
		ep->last_io = jiffies;
		dabr_urb_transfer(cntroller, urb, ep);
	}

exit_handler:
	spin_unlock_irqrestore(&ep->lock, flags);

}

static void dabr_urb_callback(struct urb *urb)
{
	struct dabridge_udc *cntroller;
	u8 address;

	cntroller = urb->context;

	address = usb_pipeendpoint (urb->pipe);
	if (usb_pipein (urb->pipe))
		address |= USB_DIR_IN;

/* 	dev_vdbg(udc_dev(cntroller), "callback for ep%d%s xfered=%d\n",
			usb_pipeendpoint (urb->pipe),
			usb_pipein(urb->pipe) ? "in" : "out",
			urb->actual_length);
 */
	dabr_urb_handler(cntroller, urb, address);
}

static inline int dabr_epX_fill(struct dabridge_udc *cntroller,
		const struct usb_endpoint_descriptor *endpoint,
		struct urb *urb,
		struct usb_request *_req)
{

	unsigned int io_pipe;
	struct dabridge_usb *dev = cntroller->bridgedev;

	urb->transfer_flags = 0;

	switch (usb_endpoint_type(endpoint)) {
	case USB_ENDPOINT_XFER_BULK:
		if (usb_endpoint_dir_in(endpoint)) {
			io_pipe = usb_rcvbulkpipe(dev->usbdev,
					usb_endpoint_num(endpoint));
		} else {
			io_pipe = usb_sndbulkpipe(dev->usbdev,
					usb_endpoint_num(endpoint));
		}
		usb_fill_bulk_urb(urb, dev->usbdev, io_pipe,
				_req->buf, _req->length,
				dabr_urb_callback, cntroller);
		break;

	case USB_ENDPOINT_XFER_ISOC:
		/* NO isoc_mult support _req->length <= usb_endpoint_maxp(&ep->desc)*/
		if (usb_endpoint_dir_in(endpoint)) {
			io_pipe = usb_rcvisocpipe(dev->usbdev,
					usb_endpoint_num(endpoint));
		} else {
			io_pipe = usb_sndisocpipe(dev->usbdev,
					usb_endpoint_num(endpoint));
		}
		urb->dev = dev->usbdev;
		urb->pipe = io_pipe;
		urb->number_of_packets = 1;
		urb->transfer_buffer = _req->buf;
		urb->transfer_buffer_length = _req->length;
		urb->complete = dabr_urb_callback;
		urb->context = cntroller;
		urb->interval = 1 << (endpoint->bInterval - 1);
		urb->transfer_flags = URB_ISO_ASAP;

		urb->iso_frame_desc[0].length = _req->length;
		urb->iso_frame_desc[0].offset = 0;
		break;
	case USB_ENDPOINT_XFER_INT:
		if (usb_endpoint_dir_in(endpoint)) {
			io_pipe = usb_rcvintpipe(dev->usbdev,
					usb_endpoint_num(endpoint));
		} else {
			io_pipe = usb_sndintpipe(dev->usbdev,
					usb_endpoint_num(endpoint));
		}
		usb_fill_int_urb(urb, dev->usbdev, io_pipe,
				_req->buf, _req->length,
				dabr_urb_callback, cntroller,
				endpoint->bInterval);
		break;

	default:
		return -1;
	}

	if (usb_pipein(io_pipe) && _req->short_not_ok)
		urb->transfer_flags |= URB_SHORT_NOT_OK;
	else if (usb_pipeout(io_pipe) && _req->zero)
		urb->transfer_flags |= URB_ZERO_PACKET;

	return 0;
}

/*
 * If successful, it returns 0, otherwise a
 * negative error number.
 */
static int dabr_epX_rw(struct dabridge_udc *cntroller,
		struct dabridge_request *dareq, struct dabr_g_ep *ep)
{
	int rv;

	if (!ep)
		return -EINVAL;

	if (!ep->br_desc)
		return -EINVAL;

/* 	dev_vdbg(udc_dev(cntroller), "ep%d_%s...%d, req %p, urb %p\n",
			usb_endpoint_num(ep->br_desc),
			usb_endpoint_dir_in(ep->br_desc) ? "read" : "write",
			dareq->req.length,
			&dareq->req, &dareq->urb);
 */
	/* Fill URB based on endpoint's transfer type */
	if (dabr_epX_fill(cntroller, ep->br_desc, &dareq->urb, &dareq->req)
			< 0) {
		return -EINVAL;
	}

	if (dareq->urb.hcpriv) {
		dev_vdbg(udc_dev(cntroller), "%s : dareq->urb.hcpriv is not null\n",
					__func__);
		return -EBUSY;
	}

	/* Track submitted URB's */
	usb_anchor_urb(&dareq->urb, &ep->submitted);

	/* do it */
	rv = usb_submit_urb(&dareq->urb, dareq->mem_flags);

	if (rv < 0) {
		usb_unanchor_urb(&dareq->urb);
		ERR_USB(cntroller->bridgedev,
				"failed submitting ep%d_%s urb, error %d",
				usb_endpoint_num(ep->br_desc),
				usb_endpoint_dir_in(ep->br_desc) ? "read" : "write",
				rv);
		rv = (rv == -ENOMEM) ? rv : -EIO;
	}

	return rv;

}

/*
 * If successful, it returns 0, otherwise a
 * negative error number.
 */
static int dabr_ep0_rw(struct dabridge_udc *cntroller, bool isRead,
		struct usb_ctrlrequest *cmd, struct usb_request *_req)
{
	int rv;
	struct dabridge_usb *dev = cntroller->bridgedev;
	struct dabridge_request *dareq = usb_request_to_dabridge_request (_req);

	if (isRead) {
		usb_fill_control_urb(&dareq->urb, dev->usbdev,
				usb_rcvctrlpipe(dev->usbdev, 0),
				(unsigned char *) cmd, _req->buf,
				_req->length, dabr_urb_callback,
				cntroller);
	} else {
		if(_req->zero)
			dareq->urb.transfer_flags = URB_ZERO_PACKET;
		else
			dareq->urb.transfer_flags = 0;

		usb_fill_control_urb(&dareq->urb, dev->usbdev,
				usb_sndctrlpipe(dev->usbdev, 0),
				(unsigned char *) cmd, _req->buf,
				_req->length, dabr_urb_callback,
				cntroller);
	}


	/* Track submitted URB's */
	usb_anchor_urb(&dareq->urb, &cntroller->eps[0].submitted);

	/* do it */
	rv = usb_submit_urb(&dareq->urb, GFP_ATOMIC);

	if (rv < 0) {
		usb_unanchor_urb(&dareq->urb);
		ERR_USB(dev, "failed submitting control urb, error %d", rv);
		rv = (rv == -ENOMEM) ? rv : -EIO;
	}

	return rv;
}

static int dabr_ep0_req_setup(struct dabridge_udc *cntroller)
{
	dev_dbg(udc_dev(cntroller), "waiting for setup packet...\n");


	/*memset(cntroller->ahost_ctrl_req, 0, sizeof(struct usb_ctrlrequest));*/
	cntroller->ctrl_state = DABR_CTRL_SETUP;
	cntroller->ahost_ctrl_req.bRequestType = VEND_RD_BMREQTYPE;
	cntroller->ahost_ctrl_req.bRequest = VEND_RD_BREQ;
	cntroller->ahost_ctrl_req.wValue = 0;
	cntroller->ahost_ctrl_req.wIndex = 0;
	cntroller->ahost_ctrl_req.wLength = sizeof(struct usb_ctrlrequest);

	memset(cntroller->bhost_usb_req->buf, 0, sizeof(struct usb_ctrlrequest));
	cntroller->bhost_usb_req->length = sizeof(struct usb_ctrlrequest);

	return dabr_ep0_rw(cntroller, CTRL_READ, &cntroller->ahost_ctrl_req,
			cntroller->bhost_usb_req);
}

static int dabr_ep0_read(struct dabridge_udc *cntroller,
		struct usb_ctrlrequest *ctlreq, struct usb_request *_req)
{

	cntroller->ahost_ctrl_req.bRequestType = VEND_RD_BMREQTYPE;
	cntroller->ahost_ctrl_req.bRequest = VEND_RD_BREQ;
	cntroller->ahost_ctrl_req.wValue = ctlreq->wValue;
	cntroller->ahost_ctrl_req.wIndex = ctlreq->wIndex;
	cntroller->ahost_ctrl_req.wLength = ctlreq->wLength;

	dabr_print_ep0_setup(cntroller, &cntroller->ahost_ctrl_req,
			ctrl_state_name[cntroller->ctrl_state]);

	return dabr_ep0_rw(cntroller, CTRL_READ, &cntroller->ahost_ctrl_req, _req);
}

static int dabr_ep0_write(struct dabridge_udc *cntroller,
		struct usb_ctrlrequest *ctlreq, struct usb_request *_req)
{

	if ((ctlreq->wIndex & 0x0001) == 0)
		ctlreq->wLength = 0;

	cntroller->ahost_ctrl_req.bRequestType = VEND_WR_BMREQTYPE;
	cntroller->ahost_ctrl_req.bRequest = VEND_WR_BREQ;
	cntroller->ahost_ctrl_req.wValue = ctlreq->wValue;
	cntroller->ahost_ctrl_req.wIndex = ctlreq->wIndex;
	cntroller->ahost_ctrl_req.wLength = ctlreq->wLength;

	dabr_print_ep0_setup(cntroller, &cntroller->ahost_ctrl_req,
			ctrl_state_name[cntroller->ctrl_state]);

#ifdef VERBOSE_DEBUG
	print_hex_dump(KERN_DEBUG, "ep0 data: ", DUMP_PREFIX_NONE, 16, 1,
			_req->buf, _req->length, 0);
#endif

	return dabr_ep0_rw(cntroller, CTRL_WRITE, &cntroller->ahost_ctrl_req, _req);
}

static void dabr_udc_delete(struct kref *kref)
{
	struct dabridge_udc *udc = kref_to_dabridge_udc(kref);
	unsigned long flags;

	spin_lock(&udc->lock);
	udc->state = DABRIDGE_UDC_STOPPED;
	spin_unlock(&udc->lock);

	flush_work(&udc->state_work);
	/* Remove device attributes */
	sysfs_remove_group(&udc->gadget.dev.kobj, &dabr_udc_attr_grp);

	if(udc->driver) {
		usb_gadget_unregister_driver(udc->driver);
	}
	usb_del_gadget_udc(&udc->gadget);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
	device_unregister(&udc->gadget.dev);
#endif

	if (udc->bhost_usb_req->buf)
		kfree(udc->bhost_usb_req->buf);
	if (udc->bhost_usb_req)
		usb_ep_free_request(udc->gadget.ep0,
				udc->bhost_usb_req);
	/*if(udc->ahost_ctrl_req)
		kfree(udc->ahost_ctrl_req);*/

	platform_device_unregister(udc->pdev);

	kfree(dabr_udc);
	dabr_udc_pdev = NULL;
	dabr_udc = NULL;

	pr_info("%s deleted\n", dabridge_gadget_name);
}

/**
 * Stop/Start all endpoint transfers. Should not be called in interrupt context.
 * @param brdev
 * @param is_stop 1=Stop, 0=Start
 * @return negative error
 */
int dabridge_udc_endpoint_transfers(struct dabridge_usb *brdev, int is_start)
{
	struct dabr_g_ep *ep;
#if 0
	struct dabridge_request *req;
#endif

	if (!brdev || !brdev->udc)
		return -1;

	if (!is_start) {
		if (brdev == brdev->udc->bridgedev) {
			dabr_set_bhost_state(brdev->udc, USB_STATE_NOTATTACHED);
			/* Stop all transfers, the device is going away */
			usb_kill_anchored_urbs(&brdev->udc->eps[0].submitted);

			/* stop all submitted non control requests */
			list_for_each_entry(ep, &brdev->udc->gadget.ep_list, ep.ep_list) {
				if(ep->desc) {
					usb_kill_anchored_urbs(&ep->submitted);
					usb_ep_disable(&ep->ep);
				}
			}
		}
		return 0;
	}

	brdev->udc->bridgedev = brdev;

	/* Queue up a control request from B-Host */
	dabr_ep0_req_setup(brdev->udc);
#if 0
	/* start all queued non control requests */
	list_for_each_entry(ep, &brdev->udc->gadget.ep_list, ep.ep_list) {
		if(ep->desc) {
			list_for_each_entry (req, &ep->queue, queue) {
				dev_info(udc_dev(brdev->udc),
						"%s  req %p urb %p(%d)\n",
						ep->ep.name, &req->req, &req->urb,
						req->req.length);
			}
		}
	}
#endif
	return 0;
}

static struct dabridge_udc *dabridge_register_udc(struct dabridge_usb *brdev,
		int instanceid)
{
	int rc;

	if(dabr_udc) {
		INFO_USB(brdev, "Re using UDC\n");
		kref_get(&dabr_udc->kref);
		return dabr_udc;
	}

	dabr_udc = kzalloc(sizeof(struct dabridge_udc), GFP_KERNEL);
	if (!dabr_udc) {
		ERR_USB(brdev, "Unable to alloc UDC memory\n");
		return NULL;
	}

	dabr_udc_pdev = platform_device_register_simple(dabridge_gadget_name,
			instanceid, NULL, 0);
	if (!dabr_udc_pdev) {
		ERR_USB(brdev, "Unable to register platform device '%s.%d'\n",
				dabridge_gadget_name, instanceid);
		kfree(dabr_udc);
		return NULL;
	}
	dabr_udc->pdev = dabr_udc_pdev;

	/* Setup gadget structure */
	dabr_udc->gadget.name = dabridge_gadget_name;
	dabr_udc->gadget.ops = &dabr_udc_ops;
	dabr_udc->gadget.speed = brdev->usbdev->speed;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
	dabr_udc->gadget.max_speed = USB_SPEED_HIGH;
#else
	dabr_udc->gadget.is_dualspeed = 1;
#endif

	/* Setup gadget.dev and register with kernel */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
	dev_set_name(&dabr_udc->gadget.dev, "gadget");
	dabr_udc->gadget.dev.parent = &dabr_udc_pdev->dev;
	dabr_udc->gadget.dev.release = dabr_gadget_release;
	rc = device_register(&dabr_udc->gadget.dev);
	if (rc < 0) {
		ERR_USB(brdev, "Unable to register gadget '%s': %d\n",
				dabridge_gadget_name, rc);
		put_device(&dabr_udc->gadget.dev);
		goto err_reg;
	}
#else
	dabr_udc->gadget.dev.parent = &dabr_udc_pdev->dev;
#endif
	dabr_udc->bridgedev = brdev;
	dabr_udc_init_eps(dabr_udc);
/*
	dabr_udc->ahost_ctrl_req = kzalloc(sizeof(struct usb_ctrlrequest),
		GFP_KERNEL);
	if(!dabr_udc->ahost_ctrl_req) {
		ERR_USB(brdev, "dabr_udc ahost_ctrl_req is failed");
		rc = -ENOMEM;
		goto err_udc;
	}*/
	/* Create a USB request for receiving BHost setup packet */
	dabr_udc->bhost_usb_req = usb_ep_alloc_request(dabr_udc->gadget.ep0,
			GFP_KERNEL);
	if (!dabr_udc->bhost_usb_req) {
		rc = -ENOMEM;
		ERR_USB(brdev, "Unable to alloc bhost usb request\n");
		goto err_udc;
	}

	dabr_udc->bhost_usb_req->buf = kzalloc(sizeof(struct usb_ctrlrequest),
			GFP_KERNEL);
	if (!dabr_udc->bhost_usb_req->buf) {
		rc = -ENOMEM;
		ERR_USB(brdev, "Unable to alloc bhost request buf\n");
		goto err_udc;
	}

	rc = usb_add_gadget_udc(&dabr_udc_pdev->dev, &dabr_udc->gadget);
	if (rc < 0) {
		ERR_USB(brdev, "Unable to add UDC device '%s': %d\n",
				dabridge_gadget_name, rc);
		goto err_udc;
	}

	/* Create device attributes */
	rc = sysfs_create_group(&dabr_udc->gadget.dev.kobj, &dabr_udc_attr_grp);
	if (rc < 0) {
		ERR_USB(brdev, "Unable to register %s sysfs attributes: %d\n",
				dabridge_gadget_name, rc);
		sysfs_remove_group(&dabr_udc->gadget.dev.kobj,
				&dabr_udc_attr_grp);
		goto err_dev;
	}

	spin_lock_init(&dabr_udc->lock);
	INIT_WORK(&dabr_udc->state_work, dabr_bhost_state_work);
	dabr_udc->state = DABRIDGE_UDC_RUNNING;
	dabr_set_bhost_state(dabr_udc, USB_STATE_NOTATTACHED);

	kref_init(&dabr_udc->kref);
	return dabr_udc;

err_dev:
	usb_del_gadget_udc(&dabr_udc->gadget);
err_udc:
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
	device_unregister(&dabr_udc->gadget.dev);
err_reg:
#endif
	if(dabr_udc_pdev) {
		platform_device_unregister(dabr_udc_pdev);
		dabr_udc_pdev = NULL;
	}
	if (dabr_udc->bhost_usb_req->buf)
		kfree(dabr_udc->bhost_usb_req->buf);
	if (dabr_udc->bhost_usb_req)
		usb_ep_free_request(dabr_udc->gadget.ep0,
				dabr_udc->bhost_usb_req);
	/*
	if(dabr_udc->ahost_ctrl_req)
		kfree(dabr_udc->ahost_ctrl_req);*/
	if (dabr_udc) {
		kfree(dabr_udc);
		dabr_udc = NULL;
	}
	return NULL;
}

static void dabridge_unregister_udc(struct dabridge_usb *brdev)
{
	struct dabridge_usb *tmpdev = NULL;
	bool devfound = false;

	if (!brdev || !brdev->udc)
		return;

	dabridge_udc_endpoint_transfers(brdev, STOP_UDC_XFER);

	/* dabr_udc->bridgedev should be updated before freeing dabridge_usb on dabridge_disconnect() */
	mutex_lock(&device_list_lock);
	if(brdev->udc->bridgedev == brdev) {
		list_for_each_entry(tmpdev, &dabridge_device_list, device_list) {
			if(tmpdev != brdev) {
				devfound = true;
				break;
			}
		}
		brdev->udc->bridgedev = ((devfound) ? tmpdev : NULL);
	}
	mutex_unlock(&device_list_lock);

	/* decrement our usage count */
	kref_put(&brdev->udc->kref, dabr_udc_delete);
}
