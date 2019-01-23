/*
 * utbridge_udc.c
 *
 * A driver to expose a device side "USB gadget" API, to direct requests to an
 * external Unwired Technology USB Bridge/Hub.
 *
 * Copyright (C) 2013 - 2014, Unwired Technology LLC
 *
 * Author: Sam Yeda<sam@unwiredtechnology.com>
 *
 * Created on: Sep 10, 2013
 *
 * Originally it was the intention to re-use dummy_hcd.c with some
 * modifications however it ended being a complete re-write.
 *
 *  Revision History
 *   SY,  March 10, 2014,      Initial Release to OEM’s
 *   SY,  March 29, 2014,      Add support for future Unwired H2H Devices
 *   SY,  June  24, 2014,      Add device attribute 'portpower' to control port vbus.
 *                             Unlinked urb status log level changed to debug from info.
 *   SY,  August 1, 2014,      Check for valid ep request completion callback.
 *   SY,  August 1, 2014,      Validate the return status of utbridge_epX_rw().
 *   SY,  August 1, 2014,      EXPERIMENTAL: Submit ep request without spinlock in gadget
 *                             context and with spinlock in callback context.
 *   SY,  September 25, 2014,  Add support for gadget driver disconnect,
 *                             suspend and resume callbacks.
 *   SY,  October 2, 2014,     Start unlinking anchored urb's as early as possible.
 *                             If unlinking does not complete at the end nuke() then hold
 *                             the callback to prevent the buffer from freeing.
 *   SY,  January 15, 2015,    Reset the bhoststate on udc stop.
 *                             Correct the printed port number.
 *   SY,  May     27, 2015,    On some variations of Kernel/USB Host controller(s),
 *                             h2h setup packet data buffer is not updated. The suspect is
 *                             the DMA copy in the USB controller.
 *                             To debug this behavior zero out the ctrl_tmp buffer
 *                             so setup handler can catch the invalid content.
 *   SY,  May     27, 2015,    To work around the above issue without modifying HC driver,
 *                             kmalloc ctrl_tmp to get DMA aligned memory.
 *
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


#define Dev_Request    (USB_TYPE_STANDARD | USB_RECIP_DEVICE)
#define Dev_InRequest  (Dev_Request | USB_DIR_IN)
#define Intf_Request   (USB_TYPE_STANDARD | USB_RECIP_INTERFACE)
#define Intf_InRequest (Intf_Request | USB_DIR_IN)
#define Ep_Request     (USB_TYPE_STANDARD | USB_RECIP_ENDPOINT)
#define Ep_InRequest   (Ep_Request | USB_DIR_IN)

#define USB_MAX_CTRL_PAYLOAD 64

enum utbridge_ctrl_state {
	REQ_FOR_SETUP,
	DATA_STAGE_IN,
	DATA_STAGE_OUT,
	STATUS_STAGE_IN,
	STATUS_STAGE_OUT,
};
static const char *const ctrl_state_name[] = {
	"SETUP     ",
	"DATA_IN   ",
	"DATA_OUT  ",
	"STATUS_IN ",
	"STATUS_OUT",
};

enum utbridge_udc_state {
	UTBRIDGE_UDC_STOPPED,
	UTBRIDGE_UDC_SUSPENDED,
	UTBRIDGE_UDC_RUNNING
};

/* gadget side driver data structures */
struct utbridge_u_ep {
	struct list_head  queue;
	unsigned long     last_io;	/* jiffies timestamp */
	struct usb_gadget *gadget;
	const struct usb_endpoint_descriptor *desc;
	struct usb_ep ep;
	int br_epIdx;
	spinlock_t   lock;
	unsigned     inprogress : 1;
	unsigned     halted : 1;
	unsigned     wedged : 1;
	unsigned     wait_for_unlink : 1;
};
static inline struct utbridge_u_ep *usb_ep_to_utbridge_u_ep(struct usb_ep *_ep) {
	return container_of (_ep, struct utbridge_u_ep, ep);
}

struct utbridge_request {
	struct list_head queue; /* ep's requests */
	struct usb_request req;
};

static inline struct utbridge_request *usb_request_to_utbridge_request(
		struct usb_request *_req) {
	return container_of (_req, struct utbridge_request, req);
}

static const struct usb_endpoint_descriptor utbridge_ep0_desc = {
	.bLength          = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  = USB_DT_ENDPOINT,
	.bEndpointAddress = 0,
	.bmAttributes     = USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize   = USB_MAX_CTRL_PAYLOAD,
};

struct utbridge_core {

	/* GADGET side support */
	struct usb_gadget_driver *driver;
	struct utbridge_u_ep eps[UTBRIDGE_ENDPOINTS];
	struct usb_gadget    gadget;
	int      address;
	u16      devstatus;
	unsigned pullup:1;

	/* BRIDGE side support */
	struct usb_utbridge *br_dev;

	/* UDC */
	struct platform_device *pdev;
	struct work_struct     state_work;
	enum utbridge_udc_state state;
	spinlock_t lock;
	unsigned   active:1;
	unsigned   resuming:1;

	struct usb_ctrlrequest   ctrl_req;
	struct usb_ctrlrequest   ctrl_rsp;
	struct usb_ctrlrequest   *ctrl_tmp;
	enum utbridge_ctrl_state ctrl_state;
	enum usb_device_state    bhoststate;
};

static int utbridge_ep0_req_setup(struct utbridge_core *cntroller);
static int utbridge_ep0_read(struct utbridge_core *cntroller,
		struct usb_ctrlrequest *ctlreq, struct usb_request *_req);
static int utbridge_ep0_write(struct utbridge_core *cntroller,
		struct usb_ctrlrequest *ctlreq, struct usb_request *_req);
static int utbridge_epX_rw(struct utbridge_core *cntroller, int epIndex,
		struct usb_request *_req);
static int utbridge_handle_ep_queue(struct utbridge_core *cntroller,
		struct utbridge_u_ep *ep);
static void utbridge_stop_all_transfer(struct utbridge_core *cntroller);

static inline struct device *udc_dev(struct utbridge_core *cntroller) {
	return cntroller->gadget.dev.parent;
}

static inline struct utbridge_core *ep_to_controller(struct utbridge_u_ep *ep) {
	return container_of (ep->gadget, struct utbridge_core, gadget);
}

static inline struct utbridge_core *gadget_to_utbridge_core(
		struct usb_gadget *gadget) {
	return container_of(gadget, struct utbridge_core, gadget);
}

static inline struct utbridge_core *gadget_dev_to_controller(struct device *dev) {
	return container_of (dev, struct utbridge_core, gadget.dev);
}

static inline struct utbridge_core *state_work_to_core(struct work_struct *w) {
	return container_of(w, struct utbridge_core, state_work);
}

static struct utbridge_core *the_core;

/*-------------------------------------------------------------------------*/

static void utbridge_bhost_state_work(struct work_struct *work)
{
	struct utbridge_core *cntroller = state_work_to_core(work);

	sysfs_notify(&cntroller->gadget.dev.kobj, NULL, "bhoststate");
}

static void utbridge_bhost_set_state(struct utbridge_core *cntroller,
		enum usb_device_state state)
{
	if(cntroller->bhoststate != state) {
		cntroller->bhoststate = state;
		schedule_work(&cntroller->state_work);
	}
}

static inline void print_ep0_setup(struct utbridge_core *cntroller,
		struct usb_ctrlrequest *setup, const char* prefix) {

	dev_dbg(udc_dev(cntroller), "%s: %02x.%02x v%04x i%04x l%d %s\n", prefix,
			setup->bRequestType, setup->bRequest, setup->wValue, setup->wIndex,
			setup->wLength,
			(prefix[1] == 'E') ? ctrl_state_name[cntroller->ctrl_state] : "");

}

/* Empty endpoint request queue and call completion handler */
static void nuke(struct utbridge_core *cntroller, struct utbridge_u_ep *ep, struct urb *urb) {
	struct utbridge_request *req, *tmp;

	spin_lock(&ep->lock);
	/* iterate the queue reverse order in case we get lucky and the unlink happens
	 * before we reach to unlinked request.
	 * Consequences of calling the callback reverse is unknown */
	list_for_each_entry_safe_reverse (req, tmp, &ep->queue, queue) {
		list_del_init(&req->queue);
		req->req.status = -ESHUTDOWN;
		//dev_info (udc_dev(cntroller), "%s nuke req=%p\n", ep->ep.name, &req->req);
		if(urb && (urb->transfer_buffer == req->req.buf) && ep->wait_for_unlink) {
			dev_warn (udc_dev(cntroller), "holding back completion callback for %s req %p\n",
					ep->ep.name, &req->req);
		} else {
			spin_unlock(&ep->lock);
			req->req.complete(&ep->ep, &req->req);
			spin_lock(&ep->lock);
		}
	}
	spin_unlock(&ep->lock);
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
static inline u8 epaddr_swap(u8 caddr) {
	unsigned long addr = caddr;
	if ((addr & ~USB_DIR_IN) != 0) {
		/* odd to even and vice versa number conversion */
		(addr & 1) ? addr++ : addr--;
		/* Direction bit flip */
		change_bit(7, &addr);
	}
	return (u8)addr;
}

static int bridge_addr_to_index(struct utbridge_core *cntroller, u8 addr) {
	int i;
	if ((addr & ~USB_DIR_IN) == 0)
		return 0;
	for (i = 1; i < cntroller->br_dev->max_eps; i++) {
		if (cntroller->br_dev->eps[i].endpointDesc->bEndpointAddress
				== addr)
			return i;
	}
	dev_err(udc_dev(cntroller), "No index for ep address=0x%02x\n", addr);
	return -1;
}

/*----------------------GADGET SIDE DRIVER---------------------------*/

#define is_enabled(cntroller) (cntroller->pullup)

static int utbridge_ep_enable (struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc) {
	struct utbridge_core *cntroller;
	struct utbridge_u_ep *ep;
	unsigned max;
	int retval;

	ep = usb_ep_to_utbridge_u_ep (_ep);
	if (!_ep || !desc || ep->desc || _ep->name == ep0name
			|| desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;
	cntroller = ep_to_controller (ep);

	if (!cntroller->driver)
		return -ESHUTDOWN;

	if (!is_enabled(cntroller))
		return -ESHUTDOWN;

	dev_vdbg(udc_dev(cntroller), "enable %s\n",_ep->name);
	/*
	 * For HS/FS devices only bits 0..10 of the wMaxPacketSize represent the
	 * maximum packet size.
	 * For SS devices the wMaxPacketSize is limited by 1024.
	 */
	max = usb_endpoint_maxp(desc) & 0x7ff;

	/* drivers must not request bad settings, since lower levels
	 * (hardware or its drivers) may not check.
	 * There maybe have maxpacket limitations, etc.
	 *
	 */
	retval = -EINVAL;
	switch (desc->bmAttributes & 0x03) {
	case USB_ENDPOINT_XFER_BULK:
		if (strstr (ep->ep.name, "-iso") || strstr (ep->ep.name, "-int")) {
			goto done;
		}
		switch (cntroller->gadget.speed) {
		case USB_SPEED_HIGH:
			if (max == 512)
				break;
			goto done;
		case USB_SPEED_FULL:
			if (max == 8 || max == 16 || max == 32 || max == 64)
				/* we'll fake any legal size */
				break;
			/* save a return statement */
		default:
			goto done;
		}
		break;
	case USB_ENDPOINT_XFER_INT:
		if (strstr (ep->ep.name, "-iso")) /* bulk is ok */
			goto done;
		/* real hardware might not handle all packet sizes */
		switch (cntroller->gadget.speed) {
		case USB_SPEED_HIGH:
			if (max <= 1024)
				break;
			/* save a return statement */
		case USB_SPEED_FULL:
			if (max <= 64)
				break;
			/* save a return statement */
		default:
			if (max <= 8)
				break;
			goto done;
		}
		break;
	case USB_ENDPOINT_XFER_ISOC:
		if (strstr (ep->ep.name, "-bulk") || strstr (ep->ep.name, "-int"))
			goto done;
		/* real hardware might not handle all packet sizes */
		switch (cntroller->gadget.speed) {
		case USB_SPEED_HIGH:
			if (max <= 1024)
				break;
			/* save a return statement */
		case USB_SPEED_FULL:
			if (max <= 1023)
				break;
			/* save a return statement */
		default:
			goto done;
		}
		break;
	default:
		/* few chips support control except on ep0 */
		goto done;
	}
	if(cntroller->br_dev->info->isoc_out_ep &&
			(desc->bEndpointAddress == cntroller->br_dev->info->isoc_out_ep)) {
		/*ISOC endpoint override*/
		retval = bridge_addr_to_index(cntroller, cntroller->br_dev->info->isoc_in_ep);
	} else {
		retval = bridge_addr_to_index(cntroller,
				epaddr_swap(desc->bEndpointAddress) /*gadget to bridge ep address*/);
	}
	if( retval < 0) {
		dev_err(udc_dev(cntroller), "Attempted to enable invalid ep=0x%02x\n",
				desc->bEndpointAddress);
		retval = -EINVAL;
		goto done;
	}

	ep->br_epIdx = retval;
	_ep->maxpacket = max;
	ep->desc = desc;

	dev_dbg (udc_dev(cntroller),
		"enabled %s (ep%d%s-%s) maxpacket %d bridge_epIdx=%d\n",
		_ep->name,
		desc->bEndpointAddress & 0x0f,
		(desc->bEndpointAddress & USB_DIR_IN) ? "in" : "out",
		({ char *val;
		 switch (desc->bmAttributes & 0x03) {
		 case USB_ENDPOINT_XFER_BULK: val = "bulk"; break;
		 case USB_ENDPOINT_XFER_ISOC: val = "iso"; break;
		 case USB_ENDPOINT_XFER_INT: val = "int"; break;
		 default: val = "ctrl"; break;
		 }; val; }),
		max,
		ep->br_epIdx);

	/* at this point real hardware should be NAKing transfers
	 * to that endpoint, until a buffer is queued to it.
	 */
	ep->halted = ep->wedged = ep->inprogress = ep->wait_for_unlink = 0;
	retval = 0;
done:
	return retval;
}

static struct urb *usb_unlink_anchored_urbs_my(struct usb_anchor *anchor)
{
	struct urb *victim;

	while ((victim = usb_get_from_anchor(anchor)) != NULL) {
		usb_unlink_urb(victim);
		usb_put_urb(victim);
		/* only one urb is queued at a time, pick the first one */
		return victim;
	}
	return NULL;
}

static int utbridge_ep_disable (struct usb_ep *_ep) {
	struct utbridge_u_ep *ep;
	struct utbridge_core *cntroller;
	struct urb *unlinkedurb = NULL;
	unsigned long flags;
	int retval;

	ep = usb_ep_to_utbridge_u_ep (_ep);
	if (!_ep || !ep->desc || _ep->name == ep0name)
		return -EINVAL;
	cntroller = ep_to_controller (ep);

	/* Stop any submitted URB's */
	if (cntroller->state == UTBRIDGE_UDC_RUNNING) {
		if(!ep->wait_for_unlink) {
			ep->wait_for_unlink = 1;
			unlinkedurb = usb_unlink_anchored_urbs_my(&cntroller->br_dev->eps[ep->br_epIdx].submitted);
			if(!unlinkedurb)
				ep->wait_for_unlink = 0; /* Nothing to unlink */
		}
	}

	/* Empty endpoint request queue and call completion handler */
	nuke(cntroller, ep, unlinkedurb);

	spin_lock_irqsave(&cntroller->lock, flags);
	ep->desc = NULL;
	retval = 0;
	spin_unlock_irqrestore(&cntroller->lock, flags);

	dev_dbg (udc_dev(cntroller), "ep_disable %s\n", _ep->name);
	return retval;
}

static struct usb_request *utbridge_ep_alloc_request (struct usb_ep *_ep,
		gfp_t mem_flags) {
	struct utbridge_u_ep *ep;
	struct utbridge_request *req;

	if (!_ep)
		return NULL;
	ep = usb_ep_to_utbridge_u_ep (_ep);

	req = kzalloc(sizeof(*req), mem_flags);
	if (!req)
		return NULL;
	INIT_LIST_HEAD (&req->queue);

	dev_dbg (udc_dev(ep_to_controller(ep)), "ep_alloc_request %s\n", _ep->name);
	return &req->req;
}

static void utbridge_ep_free_request (struct usb_ep *_ep,
		struct usb_request *_req) {
	struct utbridge_u_ep *ep;
	struct utbridge_request *req;

	ep = usb_ep_to_utbridge_u_ep (_ep);
	if (!ep || !_req) {
		WARN_ON(1);
		return;
	}

	req = usb_request_to_utbridge_request (_req);
	WARN_ON (!list_empty (&req->queue));
	kfree (req);

	dev_dbg (udc_dev(ep_to_controller(ep)), "ep_free_request %s\n", _ep->name);
}

/* queues (submits) an I/O request to an endpoint */
static int utbridge_ep_queue (struct usb_ep *_ep, struct usb_request *_req,
		gfp_t mem_flags) {
	struct utbridge_u_ep *ep;
	struct utbridge_request *req;
	struct utbridge_core *cntroller;
	unsigned long flags;

	req = usb_request_to_utbridge_request (_req);
	if (!_req || !list_empty (&req->queue) || !_req->complete)
		return -EINVAL;

	ep = usb_ep_to_utbridge_u_ep (_ep);
	if (!_ep || (!ep->desc && _ep->name != ep0name))
		return -EINVAL;

	cntroller = ep_to_controller (ep);
	if (!cntroller->driver || !is_enabled(cntroller))
		return -ESHUTDOWN;

	dev_vdbg (udc_dev(cntroller), "ep %p queue req %p to %s, len %d buf %p\n",
			ep, _req, _ep->name, _req->length, _req->buf);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	spin_lock_irqsave(&ep->lock, flags);
	list_add_tail(&req->queue, &ep->queue);
	if (ep->inprogress && ep!= &cntroller->eps[0]) goto done;

	ep->inprogress = 1;
	spin_unlock_irqrestore(&ep->lock, flags);

	if (utbridge_handle_ep_queue(cntroller, ep) < 0) {
		dev_vdbg(udc_dev(cntroller), "Unable to queue req %p, %s...do it later\n", _req, _ep->name );
		spin_lock_irqsave(&ep->lock, flags);
		ep->inprogress = 0;
		spin_unlock_irqrestore(&ep->lock, flags);
	}

	return 0;

done:
	spin_unlock_irqrestore(&ep->lock, flags);

	return 0;
}

/* dequeues (cancels, unlinks) an I/O request from an endpoint */
static int utbridge_ep_dequeue (struct usb_ep *_ep, struct usb_request *_req) {
	struct utbridge_u_ep *ep;
	struct utbridge_core *cntroller;
	int retval = -EINVAL;
	unsigned long flags;
	struct utbridge_request *req = NULL;

	if (!_ep || !_req)
		return retval;
	ep = usb_ep_to_utbridge_u_ep (_ep);
	cntroller = ep_to_controller (ep);

	if (!cntroller->driver)
		return -ESHUTDOWN;

	local_irq_save (flags);
	spin_lock(&ep->lock);
	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req) {
			list_del_init (&req->queue);
			_req->status = -ECONNRESET;
			retval = 0;
			break;
		}
	}
	spin_unlock(&ep->lock);

	if (retval == 0) {
		dev_dbg (udc_dev(cntroller), "dequeued req %p from %s, len %d buf %p\n",
				req, _ep->name, _req->length, _req->buf);
		_req->complete (_ep, _req);
	}
	local_irq_restore (flags);
	return retval;
}

static int ep_set_halt_and_wedge(struct usb_ep *_ep, int value, int wedged) {
	struct utbridge_u_ep *ep;
	struct utbridge_core *cntroller;

	if (!_ep)
		return -EINVAL;
	ep = usb_ep_to_utbridge_u_ep (_ep);
	cntroller = ep_to_controller (ep);
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

static int utbridge_ep_set_halt(struct usb_ep *_ep, int value) {
	return ep_set_halt_and_wedge(_ep, value, 0);
}

static int utbridge_ep_set_wedge(struct usb_ep *_ep) {
	if (!_ep || _ep->name == ep0name)
		return -EINVAL;
	return ep_set_halt_and_wedge(_ep, 1, 1);
}

static const struct usb_ep_ops utbridge_ep_ops = {
	.enable    = utbridge_ep_enable,
	.disable   = utbridge_ep_disable,
	.alloc_request = utbridge_ep_alloc_request,
	.free_request  = utbridge_ep_free_request,
	.queue     = utbridge_ep_queue,
	.dequeue   = utbridge_ep_dequeue,
	.set_halt  = utbridge_ep_set_halt,
	.set_wedge = utbridge_ep_set_wedge,
};

/*-------------------------------------------------------------------------*/

/* there are both host and device side versions of this call ... */
static int utbridge_udc_get_frame (struct usb_gadget *_gadget) {
	struct timeval tv;

	do_gettimeofday (&tv);
	return tv.tv_usec / 1000;
}

static int utbridge_udc_wakeup (struct usb_gadget *_gadget) {
	struct utbridge_core *cntroller;

	cntroller = gadget_to_utbridge_core(_gadget);
	if (!(cntroller->devstatus & ((1 << USB_DEVICE_B_HNP_ENABLE)
				| (1 << USB_DEVICE_REMOTE_WAKEUP))))
		return -EINVAL;

	cntroller->resuming = 1;
	return 0;
}

static int utbridge_udc_set_selfpowered (struct usb_gadget *_gadget,
		int value) {
	struct utbridge_core *cntroller;

	cntroller = gadget_to_utbridge_core(_gadget);
	if (value)
		cntroller->devstatus |= (1 << USB_DEVICE_SELF_POWERED);
	else
		cntroller->devstatus &= ~(1 << USB_DEVICE_SELF_POWERED);

	return 0;
}

static void utbridge_udc_update_ep0(struct utbridge_core *cntroller) {
	u32 i;
	for (i = 1; i < cntroller->br_dev->max_eps; i++) {
		cntroller->eps[i].ep.max_streams = 0;
	}
	cntroller->eps[0].ep.maxpacket = USB_MAX_CTRL_PAYLOAD;
}

static int utbridge_udc_pullup (struct usb_gadget *_gadget, int value) {
	struct utbridge_core *cntroller;
	unsigned long flags;

	cntroller = gadget_dev_to_controller(&_gadget->dev);

	spin_lock_irqsave (&cntroller->lock, flags);
	if (value && cntroller->driver) {
		utbridge_udc_update_ep0(cntroller);
		dev_dbg(udc_dev(cntroller), "speed info -> gadget(%s) and driver(%s)\n",
				usb_speed_string(cntroller->gadget.speed),
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
				usb_speed_string(cntroller->driver->max_speed));
#else
				usb_speed_string(cntroller->driver->speed));
#endif

		/* Queue up a control request from B-Host */
		utbridge_ep0_req_setup(cntroller);
	} else {
		/* Stop any pending Ctrl requests */
		if (cntroller->state == UTBRIDGE_UDC_RUNNING) {
			usb_unlink_anchored_urbs(&cntroller->br_dev->eps[0].submitted);
			utbridge_stop_all_transfer(cntroller);
		}
	}

	cntroller->pullup = (value != 0);
	cntroller->active = (value != 0);
	spin_unlock_irqrestore (&cntroller->lock, flags);

	dev_dbg (udc_dev(cntroller), "udc pulled %s\n", (value)?"up":"down");
	return 0;
}

/*
 * Driver registration/unregistration.
 */
static int utbridge_udc_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver) {
	struct utbridge_core *cntroller = gadget_to_utbridge_core(g);
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
	return 0;
}

static int utbridge_udc_stop(struct usb_gadget *g,
		struct usb_gadget_driver *driver) {
	struct utbridge_core *cntroller = gadget_to_utbridge_core(g);

	dev_dbg (udc_dev(cntroller), "unregister gadget driver '%s'\n",
			(cntroller->driver) ? cntroller->driver->driver.name: "None");

	if(driver && cntroller->driver && cntroller->driver->disconnect) {
		cntroller->driver->disconnect(&cntroller->gadget);
	}
	utbridge_bhost_set_state(cntroller, USB_STATE_NOTATTACHED);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
	cntroller->gadget.dev.driver = NULL;
#endif
	cntroller->driver = NULL;

	return 0;
}

static const struct usb_gadget_ops utbridge_udc_ops = {
	.get_frame = utbridge_udc_get_frame,
	.wakeup    = utbridge_udc_wakeup,
	.set_selfpowered = utbridge_udc_set_selfpowered,
	.pullup    = utbridge_udc_pullup,
	.udc_start = utbridge_udc_start,
	.udc_stop  = utbridge_udc_stop,
};

/*-------------------------sysfs attributes--------------------------------*/
static ssize_t show_portpower(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	return sprintf(buf, "To control port power\n <port>=<1|0>\n");
}
static ssize_t store_portpower(struct device *dev, struct device_attribute *attr,
		   const char *buf, size_t count)
{
	struct utbridge_core *cntroller = gadget_dev_to_controller (dev);
	u8 port, power;
	int rstatus;

	if(!cntroller->br_dev->info->new_apis) {
		return -ENODEV;
	}

	if (sscanf(buf, "%hhu=%hhu", &port, &power) < 2) {
		return -EINVAL;
	}
	if(port > MAX_BRIDGE_PORT || port < 1) {
		ERR_USB(cntroller->br_dev, "Out of range %i, must be between 1-%d\n",
				port, MAX_BRIDGE_PORT);
		return -ERANGE;
	}

	spin_lock_irq(&cntroller->br_dev->eps[0].err_lock);
	the_bridge->eps[0].ongoing = 1;
	spin_unlock_irq(&cntroller->br_dev->eps[0].err_lock);
	usb_kill_anchored_urbs(&cntroller->br_dev->eps[0].submitted);

	if(port) {
		port += cntroller->br_dev->info->port_num_offset;
	}

	rstatus = utbridge_port_power(cntroller->br_dev, port, power);

	spin_lock_irq(&cntroller->br_dev->eps[0].err_lock);
	the_bridge->eps[0].ongoing = 0;
	spin_unlock_irq(&cntroller->br_dev->eps[0].err_lock);

	if(cntroller->br_dev->connectedport > 0) {
		/* Queue up a control request from B-Host if bridge mode was enabled */
		utbridge_ep0_req_setup(cntroller);
	}

	if(rstatus < 0) {
		return rstatus;
	} else {
		INFO_USB(cntroller->br_dev, "Downstream port %hhu power set to %s\n",
				(port - cntroller->br_dev->info->port_num_offset), power?"On":"Off");
		return count;
	}
}
static DEVICE_ATTR (portpower, S_IRUGO|S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP,
		show_portpower, store_portpower);

static ssize_t show_bhoststate(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct utbridge_core *cntroller = gadget_dev_to_controller (dev);

	return sprintf(buf, "%d\n", cntroller->bhoststate);
}
static DEVICE_ATTR (bhoststate, S_IRUGO|S_IRUSR|S_IRGRP, show_bhoststate, NULL);

static ssize_t show_function (struct device *dev,
		struct device_attribute *attr, char *buf) {
	struct utbridge_core *cntroller = gadget_dev_to_controller (dev);

	if (!cntroller->driver || !cntroller->driver->function)
		return 0;
	return scnprintf (buf, PAGE_SIZE, "%s\n", cntroller->driver->function);
}
static DEVICE_ATTR (function, S_IRUGO|S_IRUSR|S_IRGRP, show_function, NULL);

static ssize_t show_bridgestatus(struct device *dev, struct device_attribute *attr,
		char *buf) {
	struct utbridge_core *cntroller = gadget_dev_to_controller(dev);
	struct utbridge_request *req;
	int i;
	unsigned long flags;

	spin_lock_irqsave(&cntroller->lock, flags);
	for (i = 1; i < cntroller->br_dev->info->num_of_ep_names; i++) {
		struct utbridge_u_ep	*ep = &cntroller->eps [i];

		if (!ep->desc) {
			continue;
		}
		scnprintf (buf, PAGE_SIZE, "%sB-Host %s queue:\n", buf, ep->ep.name);
		list_for_each_entry (req, &ep->queue, queue) {
			scnprintf (buf, PAGE_SIZE, "%s  request %p (%d)\n", buf, &req->req, req->req.length);
		}
	}
	spin_unlock_irqrestore(&cntroller->lock, flags);

	return strlen(buf);
}
static DEVICE_ATTR(bridgestatus, S_IRUGO|S_IRUSR|S_IRGRP, show_bridgestatus, NULL);

static struct device_attribute *utbridge_udc_attr [] = {
	&dev_attr_function,
	&dev_attr_bridgestatus,
	&dev_attr_bhoststate,
	&dev_attr_portpower,
};
#define NUM_OF_ATTRS ARRAY_SIZE(utbridge_udc_attr)

/*-------------------------------------------------------------------------*/

#undef is_enabled

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
/* The gadget structure is stored inside the hcd structure and will be
 * released along with it. */
static void utbridge_gadget_release (struct device *dev) {
	return;
}
#endif

static int utbridge_udc_init_eps(struct utbridge_core *cntroller) {
	int i;

	if(!cntroller->br_dev->max_eps) return -1;

	INIT_LIST_HEAD(&cntroller->gadget.ep_list);
	for (i = 0; i < cntroller->br_dev->info->num_of_ep_names; i++) {
		struct utbridge_u_ep *ep = &cntroller->eps[i];

		ep->ep.name = cntroller->br_dev->info->ep_names[i];
		dev_dbg(udc_dev(cntroller), "ep_list[%d] = %s\n", i, ep->ep.name);

		ep->ep.ops = &utbridge_ep_ops;
		list_add_tail(&ep->ep.ep_list, &cntroller->gadget.ep_list);
		ep->halted = ep->wedged = ep->inprogress = ep->wait_for_unlink = 0;
		spin_lock_init(&ep->lock);
		/*maxpacket is set by epautoconfig() called by gadget layer*/
		ep->ep.maxpacket = ~0;
		if (i)
			usb_ep_set_maxpacket_limit(&ep->ep, (unsigned short)~0);
		ep->last_io = jiffies;
		ep->gadget = &cntroller->gadget;
		ep->desc = NULL;
		INIT_LIST_HEAD(&ep->queue);
	}
	/* for ep0: the desc defined here */
	cntroller->eps[0].desc = &utbridge_ep0_desc;
	cntroller->eps[0].ep.maxpacket = USB_MAX_CTRL_PAYLOAD;
	cntroller->gadget.ep0 = &cntroller->eps[0].ep;
	list_del_init(&cntroller->eps[0].ep.ep_list);

#ifdef CONFIG_USB_OTG
	cntroller->gadget.is_otg = 1;
#endif
	return 0;
}

/*-------------------------------------------------------------------------*/

static struct utbridge_u_ep *find_endpoint (struct utbridge_core *cntroller,
		u8 address) {
	int i;

	/* Always allow Control endpoint since it is used for
	 * communicating with Bridge device */
	if ((address & ~USB_DIR_IN) == 0)
		return &cntroller->eps [0];

	for (i = 1; i < cntroller->br_dev->info->num_of_ep_names; i++) {
		struct utbridge_u_ep	*ep = &cntroller->eps [i];

		if (!ep->desc) {
			continue;
		}
		if (ep->desc->bEndpointAddress == address)
			return ep;
	}
	dev_vdbg(udc_dev(cntroller), "No ep match found\n");
	return NULL;
}

static void utbridge_stop_all_transfer(struct utbridge_core *cntroller) {
	struct utbridge_u_ep	*ep;
	struct urb *unlinkedurb = NULL;

	/* unlink any submitted requests */
	list_for_each_entry(ep, &cntroller->gadget.ep_list, ep.ep_list) {
		if(ep->desc && !ep->wait_for_unlink) {
			ep->wait_for_unlink = 1;
			unlinkedurb = usb_unlink_anchored_urbs_my(&cntroller->br_dev->eps[ep->br_epIdx].submitted);
			if(!unlinkedurb)
				ep->wait_for_unlink = 0; /* Nothing to unlink */
		}
	}
}

/* called with spinlock held */
static int utbridge_handle_ep_queue(struct utbridge_core *cntroller,
		struct utbridge_u_ep *ep) {

	struct utbridge_request *req;
	int retval = -EBUSY;

	req = list_entry(ep->queue.next, struct utbridge_request, queue);

	//spin_unlock(&ep->lock);
	if (ep == &cntroller->eps[0]) {
		cntroller->ctrl_rsp.wLength = req->req.length;
		if (cntroller->ctrl_state == DATA_STAGE_IN
				|| cntroller->ctrl_state == STATUS_STAGE_IN) {

			cntroller->ctrl_rsp.wIndex = ((cntroller->ctrl_rsp.bRequestType
					& USB_TYPE_MASK) << 7) | (cntroller->ctrl_rsp.bRequest << 4)
					| PROCESS_WVALUE_DATA | SEND_DATA_TO_PHY_HOST;

			switch (cntroller->ctrl_rsp.bRequest) {
			case USB_REQ_SET_CONFIGURATION:
			case USB_REQ_SET_INTERFACE:
				cntroller->ctrl_rsp.wIndex |= STORE_WVALUE_IN_FPGA;
				utbridge_bhost_set_state(cntroller, USB_STATE_CONFIGURED);
				break;
			}
			retval = utbridge_ep0_write(cntroller, &cntroller->ctrl_rsp,
					&req->req);
		} else {
			retval = utbridge_ep0_read(cntroller, &cntroller->ctrl_rsp,
					&req->req);
		}
	} else {
			retval = utbridge_epX_rw(cntroller, ep->br_epIdx, &req->req);
	}
	//spin_lock(&ep->lock);

	return retval;
}

/**
 * utbridge_handle_ctrl_req() - handles all incoming control transfers
 * @cntroller: pointer to utbridge_core (the_core)
 * @urb: the urb request to handle
 * @setup: pointer to the setup data for a USB device control
 *	 request
 *
 * Return 0 - if the request was handled
 *	  1 - if the request wasn't handles
 *	  error code on error
 */
static int utbridge_handle_ctrl_req(struct utbridge_core *cntroller,
		struct usb_ctrlrequest *setup) {
	struct utbridge_u_ep *ep2;
	int ret_val = 1;
	unsigned w_index;
	unsigned w_value;
	char buf[2];
	int save2fpga = 0;
	struct usb_request req;

	print_ep0_setup(cntroller, setup, ctrl_state_name[0]);

	w_index = le16_to_cpu(setup->wIndex);
	w_value = le16_to_cpu(setup->wValue);
	switch (setup->bRequest) {
	case USB_REQ_SET_ADDRESS:
		if (setup->bRequestType != Dev_Request)
			break;
		cntroller->address = w_value;
		dev_info(udc_dev(cntroller), "B-Host sent function address = %d\n", w_value);
		if(cntroller->driver && cntroller->driver->resume) {
			cntroller->driver->resume(&cntroller->gadget);
		}
		utbridge_bhost_set_state(cntroller, USB_STATE_ADDRESS);
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
				dev_warn(udc_dev(cntroller), "set devstatus = 0x%x\n", cntroller->devstatus);
			}
		} else if (setup->bRequestType == Ep_Request) {
			/* endpoint halt */
			ep2 = find_endpoint(cntroller, w_index);
			if (!ep2 || ep2->ep.name == ep0name) {
				ret_val = -EOPNOTSUPP;
				break;
			}
			ep2->halted = 1;
			ret_val = 0;
			dev_warn(udc_dev(cntroller), "set %s halt\n", ep2->ep.name);
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
				dev_warn(udc_dev(cntroller), "clear devstatus = 0x%x\n", cntroller->devstatus);
			}
		} else if (setup->bRequestType == Ep_Request) {
			/* endpoint halt */
			ep2 = find_endpoint(cntroller, w_index);
			if (!ep2) {
				ret_val = -EOPNOTSUPP;
				break;
			}
			if (!ep2->wedged)
				ep2->halted = 0;
			ret_val = 0;
			dev_warn(udc_dev(cntroller), "clear %s halt\n", ep2->ep.name);
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
				ep2 = find_endpoint(cntroller, w_index);
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
			dev_warn(udc_dev(cntroller), "get status 0x%x%x\n", buf[0], buf[1]);
			cntroller->ctrl_state = DATA_STAGE_IN;
		}
		break;
	case 0xff: /*B-Host(Relay Device) disconnected*/
		if ((setup->bRequestType == 0xff) &&
				(w_value == 0xffff) && (w_index == 0x0000)) {
			dev_info(udc_dev(cntroller), "B-Host (Relay Device) disconnected\n");
			if(cntroller->driver && cntroller->driver->suspend) {
				cntroller->driver->suspend(&cntroller->gadget);
			}
			utbridge_bhost_set_state(cntroller, USB_STATE_NOTATTACHED);
			utbridge_stop_all_transfer(cntroller);
			/* Queue up a control request from B-Host */
			utbridge_ep0_req_setup(cntroller);
			return 0;
		}
		break;
	}

	if(ret_val == 0) {
		setup->wIndex = (setup->bRequest << 4) |
				PROCESS_WVALUE_DATA | SEND_DATA_TO_PHY_HOST;
		if(save2fpga)
			setup->wIndex |= STORE_WVALUE_IN_FPGA;

		req.length = setup->wLength;
		req.buf = buf;
		req.zero = 0;
		utbridge_ep0_write(cntroller, setup, &req);
	}
	return ret_val;
}

/*
 * transfer data between URB and usb_request; caller must own lock
 * Return 1 to request setup
 */
static int utbridge_urb_transfer(struct utbridge_core *cntroller,
		struct urb *urb, struct utbridge_u_ep *ep) {
	struct utbridge_request *req;

	if (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct utbridge_request, queue);
		dev_vdbg(udc_dev(cntroller),
				"transfer between urb_ep%d%s(%d) and req %p to %s(%d) status(%d)\n",
				usb_endpoint_num(&urb->ep->desc),
				usb_pipein(urb->pipe) ? "in" : "out",
				urb->actual_length,
				&req->req,
				ep->ep.name,
				req->req.length,
				urb->status);

		/* gadget driver completion */
		list_del_init(&req->queue);

		if(usb_pipeisoc(urb->pipe)) {
			dev_vdbg(udc_dev(cntroller), "iso sof=%d err_cnt=%d status=%d len=%d\n",
					urb->start_frame,
					urb->error_count,
					urb->iso_frame_desc[0].status,
					urb->iso_frame_desc[0].actual_length);

			req->req.actual = urb->iso_frame_desc[0].actual_length;
			req->req.status = urb->iso_frame_desc[0].status;
		} else {
			req->req.actual = urb->actual_length;
			if(req->req.status != -EINPROGRESS) {
				dev_warn(udc_dev(cntroller), "not a valid queued %s req %p\n",
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
			utbridge_ep0_req_setup(cntroller);
			spin_lock(&ep->lock);
		} else if(cntroller->ctrl_state == DATA_STAGE_OUT) {
			struct usb_request _req;
			_req.length = 0;
			_req.buf = NULL;
			_req.zero = 0;
			cntroller->ctrl_rsp.wIndex = ((cntroller->ctrl_rsp.bRequestType
					& USB_TYPE_MASK) << 7) | (cntroller->ctrl_rsp.bRequest << 4)
					| PROCESS_WVALUE_DATA | SEND_DATA_TO_PHY_HOST;
			cntroller->ctrl_rsp.wLength = 0;
			spin_unlock(&ep->lock);
			utbridge_ep0_write(cntroller, &cntroller->ctrl_rsp, &_req);
			spin_lock(&ep->lock);
		}
	}

	return 0;
}

static void utbridge_urb_handler(struct utbridge_core *cntroller,
		struct urb *urb, u8 address) {
	unsigned long flags;
	struct utbridge_request *req;
	struct utbridge_u_ep *ep = NULL;

	spin_lock_irqsave(&cntroller->lock, flags);

	if (cntroller->state != UTBRIDGE_UDC_RUNNING) {
		spin_unlock_irqrestore(&cntroller->lock, flags);
		return;
	}

	/* convert bridge to gadget endpoint address*/
	if(cntroller->br_dev->info->isoc_in_ep &&
			(address == cntroller->br_dev->info->isoc_in_ep)) {
		/*ISOC endpoint override*/
		address = cntroller->br_dev->info->isoc_out_ep;
	} else {
		address = epaddr_swap(address);
	}
	/* find the gadget's ep for this request (if configured) */
	ep = find_endpoint(cntroller, address);

	spin_unlock_irqrestore(&cntroller->lock, flags);

	if (!ep) {
		/* set_configuration() disagreement */
		dev_warn(udc_dev(cntroller), "no ep(0x%02x) configured for%s urb %p status %d\n",
				address,
				(urb->unlinked)?" unlinked!":"",
				urb,
				urb->status);

		return;
	}

	spin_lock_irqsave(&ep->lock, flags);

	if (ep->halted) {
		/* NOTE: must not be iso! */
		dev_err(udc_dev(cntroller), "ep %s halted, urb %p\n", ep->ep.name, urb);
		/* TODO: Remove from queue */
		goto exit_handler;
	}
	if (urb->unlinked) {
		dev_dbg(udc_dev(cntroller), "ep %s unlinked %d, urb %p status %d\n",
				ep->ep.name, urb->unlinked, urb, urb->status);
		ep->inprogress = 0;
		if(ep->wait_for_unlink) {
			ep->wait_for_unlink = 0;
			goto exit_handler;
		}
		/* Re-submit any pending request */
		if (!list_empty(&ep->queue)) {
			if(utbridge_handle_ep_queue(cntroller, ep) < 0) {
				dev_warn(udc_dev(cntroller), "Unable to request %s...do it later\n",
						ep->ep.name );
			} else ep->inprogress = 1;
		}
		goto exit_handler;
	}

	if (ep == &cntroller->eps[0] && usb_pipein(urb->pipe)
			&& cntroller->ctrl_state != DATA_STAGE_OUT) {
		/* Received control URB without DATA_STAGE_OUT*/
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
			goto exit_handler;
		}

		if(urb->actual_length != sizeof(struct usb_ctrlrequest)) {
			/* Queue up a control request from B-Host */
			dev_err(udc_dev(cntroller), "Invalid setup packet length %d, status %d\n",
					urb->actual_length, urb->status);
			utbridge_ep0_req_setup(cntroller);
			goto exit_handler;
		}

		memcpy(&cntroller->ctrl_rsp, urb->transfer_buffer, urb->actual_length);

		if (cntroller->ctrl_rsp.bRequestType & USB_DIR_IN) {
			/*
			 * The USB 2.0 spec states that "if wLength is
			 * zero, there is no data transfer phase."
			 * However, testusb #14 seems to actually
			 * expect a data phase even if wLength = 0...
			 */
			cntroller->ctrl_state = DATA_STAGE_IN;
		} else {
			if (cntroller->ctrl_rsp.wLength != cpu_to_le16(0)) {
				/* gadget driver is expected to issue
				 * a read control data request
				 */
				cntroller->ctrl_state = DATA_STAGE_OUT;
			} else {
				cntroller->ctrl_state = STATUS_STAGE_IN;
			}
		}

		value = utbridge_handle_ctrl_req(cntroller, &cntroller->ctrl_rsp);

		if (value > 0) {
			/* gadget driver handles all other requests.  block
			 * until setup() returns; no reentrancy issues etc.
			 */
			if(cntroller->driver && cntroller->driver->setup) {
				spin_unlock(&ep->lock);
				value = cntroller->driver->setup(&cntroller->gadget,
						&cntroller->ctrl_rsp);
				spin_lock(&ep->lock);
			} else {
				value = -1;
			}

			/* error, see below */
		}

		if (value < 0) {
			dev_err(udc_dev(cntroller),
					"Unhandled Setup(%d): %02x.%02x v%04x i%04x l%d %s\n",
					value, cntroller->ctrl_rsp.bRequestType,
					cntroller->ctrl_rsp.bRequest, cntroller->ctrl_rsp.wValue,
					cntroller->ctrl_rsp.wIndex, cntroller->ctrl_rsp.wLength,
					ctrl_state_name[cntroller->ctrl_state]);
			/* Queue up a control request from B-Host */
			utbridge_ep0_req_setup(cntroller);
			goto exit_handler;
		}

	} else {
		/* Transmitted URB */
		ep->last_io = jiffies;
		utbridge_urb_transfer(cntroller, urb, ep);
	}

	if(ep != &cntroller->eps[0]) {
		ep->inprogress = 0;
		if (!list_empty(&ep->queue)) {
			if(utbridge_handle_ep_queue(cntroller, ep) < 0) {
				dev_warn(udc_dev(cntroller), "Unable to submit %s...do it later\n",
						ep->ep.name );
			} else ep->inprogress = 1;
		}
	}

exit_handler:
	spin_unlock_irqrestore(&ep->lock, flags);

}

static void utbridge_urb_callback(struct urb *urb) {
	struct utbridge_core *cntroller;
	struct usb_utbridge *dev;
	int epIndex;
	u8 address;

	cntroller = urb->context;
	dev = cntroller->br_dev;

	address = usb_pipeendpoint (urb->pipe);
	if (usb_pipein (urb->pipe))
		address |= USB_DIR_IN;

	if ((epIndex = bridge_addr_to_index(cntroller, address)) < 0) {
		dev_err(udc_dev(cntroller), "Invalid ep address 0x%02x callback\n",
				address);
		return;
	}

	dev_vdbg(udc_dev(cntroller), "callback for ep%d%s xfered=%d\n",
			usb_pipeendpoint (urb->pipe), usb_pipein(urb->pipe) ? "in" : "out",
			urb->actual_length);

	spin_lock(&dev->eps[epIndex].err_lock);
	dev->eps[epIndex].ongoing = 0;
	spin_unlock(&dev->eps[epIndex].err_lock);

	utbridge_urb_handler(cntroller, urb, address);

}

static inline int utbridge_fill_X_urb(struct utbridge_core *cntroller,
		struct usb_endpoint_descriptor *endpoint, struct urb *urb,
		struct usb_request *_req) {

	unsigned int io_pipe;
	struct usb_utbridge *dev = cntroller->br_dev;

	urb->transfer_flags = 0;

	switch (usb_endpoint_type(endpoint)) {
	case USB_ENDPOINT_XFER_BULK:
		if (usb_endpoint_dir_in(endpoint)) {
			io_pipe = usb_rcvbulkpipe(dev->udev, usb_endpoint_num(endpoint));
		} else {
			io_pipe = usb_sndbulkpipe(dev->udev, usb_endpoint_num(endpoint));
		}
		usb_fill_bulk_urb(urb, dev->udev, io_pipe, _req->buf, _req->length,
				utbridge_urb_callback, cntroller);
		break;

	case USB_ENDPOINT_XFER_ISOC:
		/* NO isoc_mult support _req->length <= usb_endpoint_maxp(&ep->desc)*/
		if (usb_endpoint_dir_in(endpoint)) {
			io_pipe = usb_rcvisocpipe(dev->udev, usb_endpoint_num(endpoint));
		} else {
			io_pipe = usb_sndisocpipe(dev->udev, usb_endpoint_num(endpoint));
		}
		urb->dev = dev->udev;
		urb->pipe = io_pipe;
		urb->number_of_packets = 1;
		urb->transfer_buffer = _req->buf;
		urb->transfer_buffer_length = _req->length;
		urb->complete = utbridge_urb_callback;
		urb->context = cntroller;
		urb->interval = 1 << (endpoint->bInterval - 1);
		urb->transfer_flags = URB_ISO_ASAP;

		urb->iso_frame_desc[0].length = _req->length;
		urb->iso_frame_desc[0].offset = 0;
		break;
	case USB_ENDPOINT_XFER_INT:
		if (usb_endpoint_dir_in(endpoint)) {
			io_pipe = usb_rcvintpipe(dev->udev, usb_endpoint_num(endpoint));
		} else {
			io_pipe = usb_sndintpipe(dev->udev, usb_endpoint_num(endpoint));
		}
		usb_fill_int_urb(urb, dev->udev, io_pipe, _req->buf, _req->length,
				utbridge_urb_callback, cntroller, endpoint->bInterval);
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
static int utbridge_epX_rw(struct utbridge_core *cntroller, int epIndex,
		struct usb_request *_req) {
	int rv;
	bool ongoing_io;
	struct usb_endpoint_descriptor *endpoint;
	struct usb_utbridge *dev = cntroller->br_dev;

	spin_lock_irq(&dev->eps[epIndex].err_lock);
	ongoing_io = dev->eps[epIndex].ongoing;
	spin_unlock_irq(&dev->eps[epIndex].err_lock);

	if (ongoing_io)
		return -EAGAIN;

	endpoint = dev->eps[epIndex].endpointDesc;

	dev_vdbg(udc_dev(cntroller), "ep%d_%s...%d\n", usb_endpoint_num(endpoint),
			usb_endpoint_dir_in(endpoint) ? "read" : "write", _req->length);

	/* Fill URB based on endpoint's transfer type */
	if (utbridge_fill_X_urb(cntroller, endpoint, dev->eps[epIndex].urb, _req)
			< 0) {
		return -EINVAL;
	}

	/* Track submitted URB's */
	usb_anchor_urb(dev->eps[epIndex].urb, &dev->eps[epIndex].submitted);

	/* tell everybody to leave the URB alone */
	spin_lock_irq(&dev->eps[epIndex].err_lock);
	dev->eps[epIndex].ongoing = 1;
	spin_unlock_irq(&dev->eps[epIndex].err_lock);

	/* do it */
	rv = usb_submit_urb(dev->eps[epIndex].urb, GFP_ATOMIC);

	if (rv < 0) {
		ERR_USB(dev, "failed submitting control urb, error %d", rv);
		rv = (rv == -ENOMEM) ? rv : -EIO;
		spin_lock_irq(&dev->eps[epIndex].err_lock);
		dev->eps[epIndex].ongoing = 0;
		spin_unlock_irq(&dev->eps[epIndex].err_lock);
	}

	return rv;

}

/*
 * If successful, it returns 0, otherwise a
 * negative error number.
 */
static int utbridge_ep0_rw(struct utbridge_core *cntroller, bool isRead,
		struct usb_ctrlrequest *cmd, struct usb_request *_req) {
	int rv;
	bool ongoing_io;
	struct usb_utbridge *dev = cntroller->br_dev;

	spin_lock_irq(&dev->eps[0].err_lock);
	ongoing_io = dev->eps[0].ongoing;
	spin_unlock_irq(&dev->eps[0].err_lock);

	if (ongoing_io)
		return -EAGAIN;

	if (isRead) {
		usb_fill_control_urb(dev->eps[0].urb, dev->udev,
				usb_rcvctrlpipe(dev->udev, 0),
				(unsigned char *) cmd, _req->buf, _req->length, utbridge_urb_callback,
				cntroller);
	} else {
		if(_req->length) {
			memcpy(dev->eps[0].buffer, _req->buf, _req->length);
		}

		if(_req->zero)
			dev->eps[0].urb->transfer_flags = URB_ZERO_PACKET;
		else
			dev->eps[0].urb->transfer_flags = 0;

		usb_fill_control_urb(dev->eps[0].urb, dev->udev,
				usb_sndctrlpipe(dev->udev, 0),
				(unsigned char *) cmd, dev->eps[0].buffer,
				_req->length, utbridge_urb_callback,
				cntroller);
	}


	/* Track submitted URB's */
	usb_anchor_urb(dev->eps[0].urb, &dev->eps[0].submitted);

	/* tell everybody to leave the URB alone */
	spin_lock_irq(&dev->eps[0].err_lock);
	dev->eps[0].ongoing = 1;
	spin_unlock_irq(&dev->eps[0].err_lock);

	/* do it */
	rv = usb_submit_urb(dev->eps[0].urb, GFP_ATOMIC);

	if (rv < 0) {
		ERR_USB(dev, "failed submitting control urb, error %d", rv);
		rv = (rv == -ENOMEM) ? rv : -EIO;
		spin_lock_irq(&dev->eps[0].err_lock);
		dev->eps[0].ongoing = 0;
		spin_unlock_irq(&dev->eps[0].err_lock);
	}

	return rv;
}

static int utbridge_ep0_req_setup(struct utbridge_core *cntroller) {
	struct usb_request _req;
	dev_dbg(udc_dev(cntroller), "waiting for setup packet...\n");
	cntroller->ctrl_state = REQ_FOR_SETUP;
	cntroller->ctrl_req.bRequestType = VEND_RD_BMREQTYPE;
	cntroller->ctrl_req.bRequest = VEND_RD_BREQ;
	cntroller->ctrl_req.wValue = 0;
	cntroller->ctrl_req.wIndex = 0;
	cntroller->ctrl_req.wLength = sizeof(struct usb_ctrlrequest);

	memset(&_req, 0, sizeof(_req));
	memset(cntroller->ctrl_tmp, 0, sizeof(cntroller->ctrl_tmp));
	_req.length = cntroller->ctrl_req.wLength;
	_req.buf = cntroller->ctrl_tmp;
	_req.zero = 0;

	return utbridge_ep0_rw(cntroller, CTRL_READ, &cntroller->ctrl_req, &_req);
}

static int utbridge_ep0_read(struct utbridge_core *cntroller,
		struct usb_ctrlrequest *ctlreq, struct usb_request *_req) {

	cntroller->ctrl_req.bRequestType = VEND_RD_BMREQTYPE;
	cntroller->ctrl_req.bRequest = VEND_RD_BREQ;
	cntroller->ctrl_req.wValue = ctlreq->wValue;
	cntroller->ctrl_req.wIndex = ctlreq->wIndex;
	cntroller->ctrl_req.wLength = ctlreq->wLength;

	print_ep0_setup(cntroller, &cntroller->ctrl_req,
			ctrl_state_name[cntroller->ctrl_state]);

	return utbridge_ep0_rw(cntroller, CTRL_READ, &cntroller->ctrl_req, _req);
}

static int utbridge_ep0_write(struct utbridge_core *cntroller,
		struct usb_ctrlrequest *ctlreq, struct usb_request *_req) {

	if ((ctlreq->wIndex & 0x0001) == 0)
		ctlreq->wLength = 0;

	cntroller->ctrl_req.bRequestType = VEND_WR_BMREQTYPE;
	cntroller->ctrl_req.bRequest = VEND_WR_BREQ;
	cntroller->ctrl_req.wValue = ctlreq->wValue;
	cntroller->ctrl_req.wIndex = ctlreq->wIndex;
	cntroller->ctrl_req.wLength = ctlreq->wLength;

	print_ep0_setup(cntroller, &cntroller->ctrl_req,
			ctrl_state_name[cntroller->ctrl_state]);

#ifdef VERBOSE_DEBUG
	print_hex_dump(KERN_DEBUG, "ep0 data: ", DUMP_PREFIX_NONE, 16, 1,
			_req->buf, _req->length, 0);
#endif

	return utbridge_ep0_rw(cntroller, CTRL_WRITE, &cntroller->ctrl_req, _req);
}

static int utbridge_udc_probe(struct platform_device *pdev) {
	int rc, i;

	the_core = kzalloc(sizeof(struct utbridge_core), GFP_KERNEL);
	if (!the_core) {
		pr_err("utbridge_core alloc failure\n");
		return -ENOMEM;
	}
	the_core->ctrl_tmp = kzalloc(sizeof(the_core->ctrl_tmp), GFP_KERNEL);
	if (!the_core->ctrl_tmp) {
		pr_err("utbridge_core ctrl_tmp alloc failure\n");
		kfree(the_core);
		return -ENOMEM;
	}
	the_core->pdev = pdev;

	/* Setup gadget structure */
	the_core->gadget.name = gadget_name;
	the_core->gadget.ops = &utbridge_udc_ops;
	the_core->gadget.speed = the_bridge->udev->speed;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
	the_core->gadget.max_speed = USB_SPEED_HIGH;
#else
	the_core->gadget.is_dualspeed = 1;
#endif

	/* Setup gadget.dev and register with kernel */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
	dev_set_name(&the_core->gadget.dev, "gadget");
	the_core->gadget.dev.parent = &pdev->dev;
	the_core->gadget.dev.release = utbridge_gadget_release;
	rc = device_register(&the_core->gadget.dev);
	if (rc < 0) {
		pr_err("Error registering gadget device\n");
		put_device(&the_core->gadget.dev);
		return rc;
	}
#else
	the_core->gadget.dev.parent = &pdev->dev;
#endif
	the_core->br_dev = the_bridge;
	kref_get(&the_core->br_dev->kref);
	utbridge_udc_init_eps(the_core);

	rc = usb_add_gadget_udc(&pdev->dev, &the_core->gadget);
	if (rc < 0)
		goto err_udc;

	/* Create device attributes */
	for (i = 0; i < NUM_OF_ATTRS; i++) {
		rc = device_create_file(&the_core->gadget.dev, utbridge_udc_attr[i]);
		if (rc)
			break;
	}
	if (rc) {
		while (--i >= 0)
			device_remove_file(&the_core->gadget.dev, utbridge_udc_attr[i]);
		goto err_dev;
	}

	spin_lock_init(&the_core->lock);
	INIT_WORK(&the_core->state_work, utbridge_bhost_state_work);
	the_core->state = UTBRIDGE_UDC_RUNNING;
	utbridge_bhost_set_state(the_core, USB_STATE_NOTATTACHED);

	platform_set_drvdata(pdev, the_core);

	return rc;

err_dev:
	usb_del_gadget_udc(&the_core->gadget);
err_udc:
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
	device_unregister(&the_core->gadget.dev);
#endif
	if (the_core)
		kfree(the_core);
	return rc;
}

static int utbridge_udc_remove (struct platform_device *pdev)
{
	struct utbridge_core	*cntroller = platform_get_drvdata (pdev);
	int i;

	spin_lock(&cntroller->lock);
	cntroller->state = UTBRIDGE_UDC_STOPPED;
	spin_unlock(&cntroller->lock);

	flush_work(&cntroller->state_work);
	/* Remove device attributes */
	for (i = 0; i < NUM_OF_ATTRS; i++) {
		device_remove_file(&the_core->gadget.dev, utbridge_udc_attr[i]);
	}

	if(cntroller->driver) {
		usb_gadget_unregister_driver(cntroller->driver);
	}
	usb_del_gadget_udc(&cntroller->gadget);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
	platform_set_drvdata (pdev, NULL);
	device_unregister(&cntroller->gadget.dev);
#endif
	/* decrement our usage count */
	kref_put(&cntroller->br_dev->kref, utbridge_delete);
	return 0;
}

static struct platform_driver utbridge_udc_driver = {
	.probe  = utbridge_udc_probe,
	.remove = utbridge_udc_remove,
	.driver = {
	   .name  = (char *) gadget_name,
	   .owner = THIS_MODULE,
	},
};
