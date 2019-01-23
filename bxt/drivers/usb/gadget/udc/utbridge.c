/*
 * utbridge.c
 *
 * USB Driver for the Dual Role Unwired Technology USB Bridge/Hub.
 * Any single port on a single hub can be enabled for
 *  bridge mode (device role).
 *
 * Copyright (C) 2013 - 2014, Unwired Technology LLC
 *
 * Author: Sam Yeda<sam@unwiredtechnology.com>
 *
 *  Revision History
 *   SY,  March 10, 2014,  Initial Release to OEM’s
 *   SY,  March 29, 2014,  Add support for future Unwired H2H Devices
 *   SY,  June  24, 2014,  Add support for new vendor requests
 *                         WINDEX_O_SELECT_BRIDGE_PORT && WINDEX_O_PORT_POWER.
 *                         Add utbridge_port_power() to manually control port power.
 *                         Some devices require cycling vbus when switching from host
 *                         to device, enable this during bridge port selection.
 *   SY, January 15, 2015, Correct the printed port number.
 *   SY, May     27, 2015, Add Module Version.
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

/*#define DEBUG*/
/*#define VERBOSE_DEBUG*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/kthread.h>
#include <linux/version.h>	/* for KERNEL_VERSION MACRO	*/

#include <asm/byteorder.h>

#define H2H_DRIVER_VERSION "1.1.0"
#define DRIVER_DESC    "UTBridge driver with gadget/udc emulator"
#define DRIVER_VERSION "24 Aug 2013"

static const char driver_name[] = "utbridge";
static const char gadget_name[] = "utbridge_udc";

static const char ep0name[] = "ep0";
/*
 * The order of names is important for usb_ep_autoconfig matching
 */
static const char *const utbridge_ep_name[] = {
	ep0name, /* everyone has ep0 */
	"ep1in-bulk", "ep2out-bulk",
	"ep5in-int", "ep8out-iso",
	"ep3in-bulk", "ep4out-bulk",

	/* Fake endpoint(s) to satisfy test gadget drivers */
	"ep3in-iso", /* Gadget Zero */
};
static const char *const utbridge_ep_name_asic_1[] = {
	ep0name, /* everyone has ep0 */
	"ep1in-bulk", "ep2out-bulk",
	"ep5in-int", "ep6out-iso",
	"ep3in-bulk", "ep4out-bulk",

	/* Fake endpoint(s) to satisfy test gadget drivers */
	"ep3in-iso", /* Gadget Zero */
};
#define UTBRIDGE_ENDPOINTS ARRAY_SIZE(utbridge_ep_name)

struct usb_utbridge_info {
	const char **ep_names; /* endpoint names array*/
	int  num_of_ep_names; /* number of names */
	u8   isoc_in_ep;  /* isoc in endpoint override */
	u8   isoc_out_ep; /* isoc out endpoint override */
	int  port_num_offset;  /* port number offset */
	uint port_switch_wait; /* milliseconds to wait after port switch */
	u8   new_apis;         /* whether to use new vendor requests */
};

#define USB_UNWIRED_VENDOR_ID    0x2996
#define USB_HIGHLINE_PRODUCT_ID  0x0100 /* production */
#define USB_ASIC_101_PRODUCT_ID  0x0101 /* prototype  */
#define USB_ASIC_102_PRODUCT_ID  0x0102 /* production */

static const struct usb_utbridge_info highline_info = {
	.ep_names = (const char **)utbridge_ep_name,
	.num_of_ep_names = ARRAY_SIZE(utbridge_ep_name),
	.isoc_in_ep = 0,
	.isoc_out_ep = 0,
	.port_num_offset = 0,
	.port_switch_wait = 1,
	.new_apis = 0,
};
static const struct usb_utbridge_info asic_101_info = {
	.ep_names = (const char **)utbridge_ep_name_asic_1,
	.num_of_ep_names = ARRAY_SIZE(utbridge_ep_name_asic_1),
	.isoc_in_ep = 0x87,
	.isoc_out_ep = 0x06,
	.port_num_offset = -1,
	.port_switch_wait = 550,
	.new_apis = 1,
};
static const struct usb_utbridge_info asic_102_info = {
	.ep_names = (const char **)utbridge_ep_name,
	.num_of_ep_names = ARRAY_SIZE(utbridge_ep_name),
	.isoc_in_ep = 0,
	.isoc_out_ep = 0,
	.port_num_offset = -1,
	.port_switch_wait = 550,
	.new_apis = 1,
};

/* table of devices that work with this driver */
static const struct usb_device_id utbridge_table[] = {
	{ USB_DEVICE(USB_UNWIRED_VENDOR_ID, USB_HIGHLINE_PRODUCT_ID),
		.driver_info = (unsigned long)&highline_info, },
	{ USB_DEVICE(USB_UNWIRED_VENDOR_ID, USB_ASIC_101_PRODUCT_ID),
		.driver_info = (unsigned long)&asic_101_info, },
	{ USB_DEVICE(USB_UNWIRED_VENDOR_ID, USB_ASIC_102_PRODUCT_ID),
		.driver_info = (unsigned long)&asic_102_info, },
	{ USB_DEVICE(0x1772, 0x0002),
		.driver_info = (unsigned long)&highline_info, },
	{ } /* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, utbridge_table);

MODULE_DESCRIPTION (DRIVER_DESC);
MODULE_AUTHOR ("Sam Yeda <sam@unwiredtechnology.com>");
MODULE_AUTHOR ("Copyright (C) 2013-2014, Unwired Technology LLC");
MODULE_VERSION (H2H_DRIVER_VERSION);
MODULE_LICENSE ("GPL");

/*----------------------------MACROS---------------------------------------*/

#define ERR_USB(tdev, fmt, args...) \
        dev_err(&tdev->interface->dev, fmt, ## args)
#define DBG_USB(tdev, fmt, args...) \
        dev_dbg(&tdev->interface->dev, fmt, ## args)
#define INFO_USB(tdev, fmt, args...) \
        dev_info(&tdev->interface->dev, fmt, ## args)

#define MAX_BRIDGE_PORT  4
#define MAX_CTRL_BUFSIZE (512)

#define CTRL_MSG_TIMEOUT 500
#define VEND_WR_BMREQTYPE          (0x40)
#define VEND_RD_BMREQTYPE          (0xC0)
#define VEND_WR_BREQ               (0x02)
#define VEND_RD_BREQ               (0x01)

#define CTRL_READ  1
#define CTRL_WRITE 0

#define PROCESS_WVALUE_DATA         0x0001
#define STORE_WVALUE_IN_FPGA        0x0002
#define STORE_DATA_IN_FPGA          0x0004
#define SEND_DATA_TO_PHY_HOST       0x0008
#define CLASS_SPECIFIC_REQ          0x1000
#define VENDOR_SPECIFIC_REQ         0x2000

#define WINDEX_WRITE_REQUEST        (VENDOR_SPECIFIC_REQ | PROCESS_WVALUE_DATA | STORE_WVALUE_IN_FPGA)
#define WINDEX_IBREQUEST_SHIFT      4
#define WINDEX_I_FIRMWARE_VERSION   (VENDOR_SPECIFIC_REQ  | (0 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_I_THERMAL_GOOD       (VENDOR_SPECIFIC_REQ  | (1 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_O_MUX_SELECT_DN1     (WINDEX_WRITE_REQUEST | (2 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_O_MUX_SELECT_DN2     (WINDEX_WRITE_REQUEST | (3 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_O_MUX_BRIDGE_DN_SEL  (WINDEX_WRITE_REQUEST | (4 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_I_DN1_POWER_STATUS   (VENDOR_SPECIFIC_REQ  | (5 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_O_DN1_POWER_ENABLE   (WINDEX_WRITE_REQUEST | (6 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_I_DN2_POWER_STATUS   (VENDOR_SPECIFIC_REQ  | (7 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_O_DN2_POWER_ENABLE   (WINDEX_WRITE_REQUEST | (8 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_I_DN1_OVER_CURRENT   (VENDOR_SPECIFIC_REQ  | (9 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_I_DN2_OVER_CURRENT   (VENDOR_SPECIFIC_REQ  | (10 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_O_DIMMING_BRIDGE     (WINDEX_WRITE_REQUEST | (11 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_O_BRIDGE_OVERCURRENT (WINDEX_WRITE_REQUEST | (12 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_I_POWER_GOOD         (VENDOR_SPECIFIC_REQ  | (13 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_I_HW_IMAGE_VERSION   (VENDOR_SPECIFIC_REQ  | (14 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_O_FLUSH_EP_FIFO      (WINDEX_WRITE_REQUEST | (15 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_I_BHOST_STATUS       (VENDOR_SPECIFIC_REQ  | (16 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_O_SELECT_BRIDGE_PORT (WINDEX_WRITE_REQUEST | (17 << WINDEX_IBREQUEST_SHIFT))
#define WINDEX_O_PORT_POWER         (WINDEX_WRITE_REQUEST | (18 << WINDEX_IBREQUEST_SHIFT))

#define WVALUE_DN1_HUB    0
#define WVALUE_DN1_BRIDGE 1
#define WVALUE_DN2_HUB    1
#define WVALUE_DN2_BRIDGE 0

/*
typedef struct {
	u8 bridge;
	u8 gadget;
} ep_mapping_t;
static ep_mapping_t ep_mapping[] = {
	{0x81, 0x02},
	{0x02, 0x81},
	{0x83, 0x04},
	{0x04, 0x83},
	{0x85, 0x06},
	{0x06, 0x85},
	{0x87, 0x08},
	{0x08, 0x87},
};
*/

/* Device side endpoint data structures */
struct utbridge_d_ep {
	struct usb_endpoint_descriptor *endpointDesc; /* endpoint Descriptor*/
	struct usb_anchor submitted; /* in case we need to retract our submissions */
	unsigned char *buffer;  /* the buffer to receive data */
	size_t        size;     /* the size of the receive buffer */
	struct urb    *urb;     /* the urb to read/write data with */
	bool          ongoing;  /* a read/write is going on */
	int           errors;   /* the last request tanked */
	spinlock_t    err_lock; /* lock for errors */
};

/* Structure to hold all of our USB device specific stuff */
struct usb_utbridge {
	struct usb_device    *udev;      /* the usb device for this device */
	struct usb_interface *interface; /* the interface for this device */
	struct usb_utbridge_info *info;  /* usb device specific info */
	struct utbridge_d_ep *eps; /* pointer to endpoint data*/
	unsigned int max_eps;      /* maximum endpoints including ep0*/
	struct kref  kref;         /* reference counter */
	struct mutex io_mutex;     /* synchronize I/O with disconnect */
	int connectedport;         /* hub port connected to bridge*/
};
#define to_utbridge_dev(d) container_of(d, struct usb_utbridge, kref)

static struct usb_driver utbridge_driver;
static struct usb_utbridge *the_bridge;
static struct platform_device *the_udc_pdev;
static int bridgeport = 0; /* Module parameter : Default 0 for production*/

static int utbridge_port_select(struct usb_utbridge *dev, int portId);
static void utbridge_delete(struct kref *kref);
static int utbridge_epflush_fifo(struct usb_utbridge *dev, u16 ep_bits);
static int utbridge_port_power(struct usb_utbridge *dev, u8 port, u8 power);

/*-------------------------------------------------------------------------*/

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "utbridge_udc.c"

/*------------------MODULE PARAMETERS------------------------*/
static int bridgeport_set(const char *arg, const struct kernel_param *kp) {
	int port, ret = 0;
	u8 vbusport = 0;
	if (the_bridge == NULL || the_bridge->eps == NULL) {
		pr_err("%s err NODEV the_bridge or  the_bridge->eps NULL\n",__func__);
		return -ENODEV;
	}

	ret = kstrtoint(arg, 10, &port);
	if (ret) {
		pr_err("%s err %d while getting port no arg\n",__func__, ret);
		return ret;
	}
	if (port > MAX_BRIDGE_PORT || port < 0) {
		pr_err("Out of range %i, must be between 0-%d\n", port, MAX_BRIDGE_PORT);
		return -ERANGE;
	}

	spin_lock_irq(&the_bridge->eps[0].err_lock);
	the_bridge->eps[0].ongoing = 1;
	spin_unlock_irq(&the_bridge->eps[0].err_lock);
	usb_kill_anchored_urbs(&the_bridge->eps[0].submitted);

	if(port) {
		port += the_bridge->info->port_num_offset;
	}

	if(the_bridge->info->new_apis && (the_bridge->connectedport > 0) &&
			(the_bridge->connectedport != port)) {
		/* Switch VBUS OFF for the port that was previously in bridge mode */
		vbusport = (u8)the_bridge->connectedport;
		if((ret = utbridge_port_power(the_bridge, vbusport, 0)) < 0) {
		
			pr_err("%s err while port-power set %d\n",__func__, ret);
			goto ret_error;
		}
	}

	if((ret = utbridge_port_select(the_bridge, port)) < 0) {
		pr_err("%s err while port-select set %d\n",__func__, ret);
		goto ret_error;
	}

	if(port || vbusport) {
		msleep(the_bridge->info->port_switch_wait);
		if(vbusport) {
			/* Restore VBUS for the port that was previously in bridge mode */
			if((ret = utbridge_port_power(the_bridge, vbusport, 1)) < 0) {
				pr_err("%s err while port-power set to restore vbus %d\n",__func__, ret);
				goto ret_error;
			}
		}
	}

	spin_lock_irq(&the_bridge->eps[0].err_lock);
	the_bridge->eps[0].ongoing = 0;
	spin_unlock_irq(&the_bridge->eps[0].err_lock);

	/* Queue up a control request from B-Host if bridge mode enabled */
	if(port)
		utbridge_ep0_req_setup(the_core);
	else
		usb_reset_device(the_bridge->udev);

	bridgeport = port;

	smp_mb();
	return 0;

ret_error:
	spin_lock_irq(&the_bridge->eps[0].err_lock);
	the_bridge->eps[0].ongoing = 0;
	spin_unlock_irq(&the_bridge->eps[0].err_lock);
	return ret;
}
static struct kernel_param_ops bridgeport_ops = {
	.set = bridgeport_set,
	.get = param_get_int,
};
module_param_cb(bridgeport, &bridgeport_ops, &bridgeport, S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(bridgeport, "Port to configure for bridge mode (default 0):"
		"0=Off 1..3=Port");

/*-------------------------------------------------------------------------*/

static void utbridge_draw_down(struct usb_utbridge *dev)
{
	int time, i;

	for (i = 0; i < dev->max_eps; ++i) {
		time = usb_wait_anchor_empty_timeout(&dev->eps[i].submitted, 1000);
		if (!time)
			usb_kill_anchored_urbs(&dev->eps[i].submitted);
	}
}

static void utbridge_delete(struct kref *kref)
{
	int i;
	struct usb_utbridge *dev = to_utbridge_dev(kref);
	for (i = 0; i < dev->max_eps; ++i) {
		usb_free_urb(dev->eps[i].urb);
	}
	/*decrements the reference count of the usb device*/
	usb_put_dev(dev->udev);
	for (i = 0; i < dev->max_eps; ++i) {
		kfree(dev->eps[i].buffer);
	}
	kfree(dev->eps);
	kfree(dev);
	the_bridge = NULL;
	bridgeport = 0;
	pr_info("UTBridge now disconnected\n");
}

/*
 * If successful, it returns the number of bytes transferred, otherwise a
 * negative error number.
 */
static int utbridge_blocking_ctrl_rw(struct usb_utbridge *dev, bool isRead, struct usb_ctrlrequest *cmd, void *data)
{

	int rv;
	unsigned int iopipe;

	if(isRead) {
		iopipe = usb_rcvctrlpipe(dev->udev, 0);
	} else {
		iopipe = usb_sndctrlpipe(dev->udev, 0);
	}
	rv = usb_control_msg(dev->udev, iopipe,
			cmd->bRequest,
			cmd->bRequestType,
			cmd->wValue,
			cmd->wIndex,
			data,
			cmd->wLength,
			CTRL_MSG_TIMEOUT);
	return rv;

}

static void utbridge_print_version(struct usb_utbridge *dev)
{
	u8 buf[5];
	struct usb_ctrlrequest request;
	request.bRequestType = VEND_RD_BMREQTYPE;
	request.bRequest = VEND_RD_BREQ;
	request.wValue = 0;

	request.wIndex = cpu_to_le16(WINDEX_I_HW_IMAGE_VERSION);
	request.wLength = cpu_to_le16(2);
	if(utbridge_blocking_ctrl_rw(dev, CTRL_READ, &request, &buf[0]) < 0) {
		ERR_USB(dev, "Unable to read hardware revision\n");
		return;
	}
	request.wIndex = cpu_to_le16(WINDEX_I_FIRMWARE_VERSION);
	request.wLength = cpu_to_le16(3);
	if(utbridge_blocking_ctrl_rw(dev, CTRL_READ, &request, &buf[2]) < 0) {
		ERR_USB(dev, "Unable to read firmware revision\n");
		return;
	}
	INFO_USB(dev, "P(0x%04x) Driver(%s) Firmware(%hhd.%hhd.%hhd) Hardware(%hhd.%hhd) %s\n",
			dev->udev->descriptor.idProduct, DRIVER_VERSION,
			buf[4], buf[3], buf[2], buf[1], buf[0],
			usb_speed_string(dev->udev->speed));
}

static int utbridge_epflush_fifo(struct usb_utbridge *dev, u16 ep_bits)
{
	char buf[1];
	int rv;
	struct usb_ctrlrequest request;
	request.bRequestType = VEND_WR_BMREQTYPE;
	request.bRequest = VEND_WR_BREQ;
	request.wIndex = cpu_to_le16(WINDEX_O_FLUSH_EP_FIFO);
	request.wLength = 0;
	request.wValue = cpu_to_le16(ep_bits);
	rv = utbridge_blocking_ctrl_rw(dev, CTRL_WRITE, &request, buf);
	if(rv < 0) {
		ERR_USB(dev, "Unable to flush ep's 0x%02x fifo %d\n", ep_bits, rv);
	}
	return rv;
}

/**
 * Set Downstream Port Power.
 * Ensure there are no control messages in queue prior to calling this function
 */
static int utbridge_port_power(struct usb_utbridge *dev, u8 port, u8 power)
{
	char buf[1];
	struct usb_ctrlrequest request;
	int rv;

	request.bRequestType = VEND_WR_BMREQTYPE;
	request.bRequest = VEND_WR_BREQ;
	request.wLength = 0;

	request.wIndex = cpu_to_le16(WINDEX_O_PORT_POWER);
	request.wValue = cpu_to_le16((power << 8) | port);
	if((rv = utbridge_blocking_ctrl_rw(dev, CTRL_WRITE, &request, buf)) < 0) {
		ERR_USB(dev, "Failed setting port %hhu power to %s\n", port, power?"On":"Off");
	}

	return rv;
}

/**
 * Set a downstream port in Bridge mode.
 * Ensure there are no control messages in queue prior to calling this function.
 *
 * If successful returns the port Id, -1 on failure.
 */
static int utbridge_port_select(struct usb_utbridge *dev, int portId)
{
	char buf[1];
	struct usb_ctrlrequest request;
	unsigned w_value_dn1;
	unsigned w_value_dn2;
	int rv;

	request.bRequestType = VEND_WR_BMREQTYPE;
	request.bRequest = VEND_WR_BREQ;
	request.wIndex = 0;
	request.wLength = 0;

	/* Flush all endpoint fifo*/
	if((rv = utbridge_epflush_fifo(dev, 0xff)) < 0) {
		goto w_error;
	}

	if(dev->info->new_apis) {
		/* Using the new vendor request */
		request.wIndex = cpu_to_le16(WINDEX_O_SELECT_BRIDGE_PORT);
		request.wValue = cpu_to_le16(portId);
		if((rv = utbridge_blocking_ctrl_rw(dev, CTRL_WRITE, &request, buf)) < 0) {
			ERR_USB(dev, "Setting port %d into bridge mode failed\n", (portId - dev->info->port_num_offset));
			goto w_error;
		}
		goto w_done;
	}

	/* Using the legacy vendor request */
	switch (portId) {
	case 0: /* Bridge mode disabled */
		w_value_dn1 = WVALUE_DN1_HUB;
		w_value_dn2 = WVALUE_DN2_HUB;
		break;
	case 1:
		w_value_dn1 = WVALUE_DN1_BRIDGE;
		w_value_dn2 = WVALUE_DN2_HUB;
		break;
	case 2:
		w_value_dn1 = WVALUE_DN1_HUB;
		w_value_dn2 = WVALUE_DN2_BRIDGE;
		break;
	default:
		return -EINVAL;
		break;
	}

	if(portId) {
		request.wIndex = cpu_to_le16(WINDEX_O_MUX_BRIDGE_DN_SEL);
		request.wValue = cpu_to_le16(portId - 1);
		if((rv = utbridge_blocking_ctrl_rw(dev, CTRL_WRITE, &request, buf)) < 0) {
			ERR_USB(dev, "Selecting DN%d into bridge mode failed\n", portId);
			goto w_error;
		}
	} else {
		/* If the bridge was previously connected to a port,
		 * switch to another port to simulate a disconnect.
		 */
		if(dev->connectedport > 0) {
			request.wIndex = cpu_to_le16(WINDEX_O_MUX_BRIDGE_DN_SEL);
			request.wValue = cpu_to_le16((dev->connectedport == 2)?0:1);
			if((rv = utbridge_blocking_ctrl_rw(dev, CTRL_WRITE, &request, buf)) < 0) {
				ERR_USB(dev, "Selecting bridge mux failed\n");
				goto w_error;
			}
		}
	}

	msleep(10);

	/* Setup downstream port mux's*/
	request.wIndex = cpu_to_le16(WINDEX_O_MUX_SELECT_DN1);
	request.wValue = cpu_to_le16(w_value_dn1);
	if((rv = utbridge_blocking_ctrl_rw(dev, CTRL_WRITE, &request, buf)) < 0) {
		ERR_USB(dev, "DN1 mux select failed\n");
		goto w_error;
	}
	request.wIndex = cpu_to_le16(WINDEX_O_MUX_SELECT_DN2);
	request.wValue = cpu_to_le16(w_value_dn2);
	if((rv = utbridge_blocking_ctrl_rw(dev, CTRL_WRITE, &request, buf)) < 0) {
		ERR_USB(dev, "DN2 mux select failed\n");
		goto w_error;
	}

w_done:
	if(portId) {
		INFO_USB(dev, "Downstream port %d in bridge mode\n", (portId - dev->info->port_num_offset) );
	} else {
		INFO_USB(dev, "All downstream ports in host mode\n");
	}

	rv = portId;

w_error:
	dev->connectedport = rv;
	return rv;
}

static int utbridge_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_host_interface *iface_desc;
	size_t buffer_size;
	int i;
	int retval = -ENOMEM;

	if(!id->driver_info)
		return retval;

	/* allocate memory for our device state and initialize it */
	the_bridge = kzalloc(sizeof(*the_bridge), GFP_KERNEL);
	if (!the_bridge) {
		dev_err(&interface->dev, "Out of memory\n");
		goto error;
	}
	kref_init(&the_bridge->kref);
	mutex_init(&the_bridge->io_mutex);

	/*increments the reference count of the usb device*/
	the_bridge->udev = usb_get_dev(interface_to_usbdev(interface));
	the_bridge->interface = interface;
	the_bridge->info = (struct usb_utbrige_info *)id->driver_info;

	/* set up the endpoint information */
	iface_desc = interface->cur_altsetting;
	the_bridge->max_eps = iface_desc->desc.bNumEndpoints + 1;
	the_bridge->eps = kzalloc(sizeof(struct utbridge_d_ep) * the_bridge->max_eps, GFP_KERNEL);
	if (!the_bridge->eps) {
		ERR_USB(the_bridge, "malloc utbridge_ep failed\n");
		goto error;
	}

	spin_lock_init(&the_bridge->eps[0].err_lock);
	init_usb_anchor(&the_bridge->eps[0].submitted);
	the_bridge->eps[0].buffer = kmalloc(MAX_CTRL_BUFSIZE, GFP_KERNEL);
	the_bridge->eps[0].urb = usb_alloc_urb(0, GFP_ATOMIC);
	for (i = 1; i < the_bridge->max_eps; ++i) {
		the_bridge->eps[i].endpointDesc = &iface_desc->endpoint[i-1].desc;

		spin_lock_init(&the_bridge->eps[i].err_lock);
		init_usb_anchor(&the_bridge->eps[i].submitted);
		buffer_size = usb_endpoint_maxp(the_bridge->eps[i].endpointDesc);
		the_bridge->eps[i].size = buffer_size;
		the_bridge->eps[i].buffer = kmalloc(buffer_size, GFP_KERNEL);
		if (!the_bridge->eps[i].buffer) {
			ERR_USB(the_bridge, "Could not allocate bulk_in_buffer\n");
			goto error;
		}

		the_bridge->eps[i].urb = usb_alloc_urb(0, GFP_ATOMIC);

		if (!the_bridge->eps[i].urb) {
			ERR_USB(the_bridge, "Could not allocate urb\n");
			goto error;
		}

		DBG_USB(the_bridge, "eps[%d]= ep%d%s-%s maxpacket %d\n",
			i,
			the_bridge->eps[i].endpointDesc->bEndpointAddress & 0x0f,
			(the_bridge->eps[i].endpointDesc->bEndpointAddress & USB_DIR_IN) ? "in" : "out",
			({ char *val;
			 switch (the_bridge->eps[i].endpointDesc->bmAttributes & 0x03) {
			 case USB_ENDPOINT_XFER_BULK: val = "bulk"; break;
			 case USB_ENDPOINT_XFER_ISOC: val = "iso"; break;
			 case USB_ENDPOINT_XFER_INT: val = "int"; break;
			 default: val = "ctrl"; break;
			 }; val; }),
			 (int)buffer_size);

	}

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, the_bridge);

	/* let the user know about this device */
	DBG_USB(the_bridge,
		"UTBridge device(%d-%s) num_alt=%d max_eps=%d\n",
		the_bridge->udev->bus->busnum,
		the_bridge->udev->devpath,
		interface->num_altsetting,
		the_bridge->max_eps);

	/* Print Hardware Info*/
	utbridge_print_version(the_bridge);

	/* Configure downstream port for bridge mode */
	utbridge_port_select(the_bridge, bridgeport);

	the_udc_pdev = platform_device_alloc(gadget_name, -1);
	if (!the_udc_pdev) {
		ERR_USB(the_bridge, "Not able to register %s\n", gadget_name);
		retval = -ENOMEM;
		goto error;
	}
	retval = platform_driver_register(&utbridge_udc_driver);
	if (retval < 0)
		goto err_register_udc_driver;
	retval = platform_device_add(the_udc_pdev);
	if (retval < 0)
		goto err_add_udc;
	if (!platform_get_drvdata(the_udc_pdev)) {
		/*
		 * The udc was added successfully but its probe function failed
		 * for some reason.
		 */
		retval = -EINVAL;
		goto err_probe_udc;
	}

	return 0;

err_probe_udc:
	platform_device_del(the_udc_pdev);
err_add_udc:
	platform_driver_unregister(&utbridge_udc_driver);
err_register_udc_driver:
	platform_device_del(the_udc_pdev);
error:
	if (the_bridge)
		/* this frees allocated memory */
		kref_put(&the_bridge->kref, utbridge_delete);
	return retval;
}

static void utbridge_disconnect(struct usb_interface *interface)
{
	struct usb_utbridge *br_dev;

	platform_device_unregister(the_udc_pdev);
	platform_driver_unregister(&utbridge_udc_driver);

	br_dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	/* prevent more I/O from starting */
	mutex_lock(&br_dev->io_mutex);
	br_dev->interface = NULL;
	mutex_unlock(&br_dev->io_mutex);

	/* decrement our usage count */
	kref_put(&br_dev->kref, utbridge_delete);
}

static int utbridge_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_utbridge *dev = usb_get_intfdata(intf);

	if (!dev)
		return 0;
	utbridge_draw_down(dev);
	return 0;
}

static int utbridge_resume(struct usb_interface *intf)
{
	return 0;
}

static int utbridge_pre_reset(struct usb_interface *intf)
{
	struct usb_utbridge *dev = usb_get_intfdata(intf);

	mutex_lock(&dev->io_mutex);
	utbridge_draw_down(dev);

	return 0;
}

static int utbridge_post_reset(struct usb_interface *intf)
{
	struct usb_utbridge *dev = usb_get_intfdata(intf);
	int i;

	/* we are sure no URBs are active - no locking needed */
	for (i = 0; i < dev->max_eps; ++i) {
		dev->eps[i].errors = -EPIPE;
	}
	mutex_unlock(&dev->io_mutex);

	return 0;
}

static struct usb_driver utbridge_driver = {
	.name       = driver_name,
	.probe      = utbridge_probe,
	.disconnect = utbridge_disconnect,
	.suspend    = utbridge_suspend,
	.resume     = utbridge_resume,
	.pre_reset  = utbridge_pre_reset,
	.post_reset = utbridge_post_reset,
	.id_table   = utbridge_table,
	.supports_autosuspend = 1,
};
/*-------------------------------------------------------------------------*/

static int __init init (void)
{
	int	retval = -ENOMEM;

	if (usb_disabled ())
		return -ENODEV;

	/* register this driver with the USB subsystem */
	retval = usb_register(&utbridge_driver);
	if (retval) {
		pr_err("usb_register failed. Error number %d", retval);
		return retval;
	}
	return 0;
}
module_init (init);

static void __exit cleanup (void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&utbridge_driver);
}
module_exit (cleanup);
