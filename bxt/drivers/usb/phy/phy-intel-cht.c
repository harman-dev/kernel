/*
 * Intel CherryTrail USB OTG transceiver driver
 *
 * Copyright (C) 2014, Intel Corporation.
 *
 * Author: Wu, Hao
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program;
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/usb/hcd.h>
#include <linux/extcon.h>

#include "../host/xhci.h"
#include "../host/xhci-intel-cap.h"

#include "phy-intel-cht.h"

#define DRIVER_VERSION "Rev. 0.5"
#define DRIVER_AUTHOR "Hao Wu"
#define DRIVER_DESC "Intel CherryTrail USB OTG Transceiver Driver"
#define DRIVER_INFO DRIVER_DESC " " DRIVER_VERSION

static const char driver_name[] = "intel-cht-otg";

/* TODO: Remove hardcoded name */
#define CHT_TYPEC_EXTCON_DEV_NAME "bxt_wcove_usbc"

static struct cht_otg *cht_otg_dev;

static int cht_handle_extcon_otg_event(struct notifier_block *nb,
				       unsigned long event, void *param);

static int edev_state_to_id(struct extcon_dev *evdev)
{
	int state;

	state = extcon_get_cable_state(evdev, "USB-Host");

	return !evdev->state;
}

static int cht_otg_set_id_mux(struct cht_otg *otg_dev, int id)
{
	struct usb_bus *host = otg_dev->phy.otg->host;
	struct usb_gadget *gadget = otg_dev->phy.otg->gadget;
	struct usb_hcd *hcd;
	struct xhci_hcd *xhci;

	if (!host || !gadget || !gadget->dev.parent)
		return -ENODEV;

	hcd = bus_to_hcd(host);
	xhci = hcd_to_xhci(hcd);

	/* HACK: PHY used in Cherrytrail is shared by both host and device
	 * controller, it requires both host and device controller to be D0
	 * for any action related to PHY transition */
	pm_runtime_get_sync(host->controller);
	pm_runtime_get_sync(gadget->dev.parent);

	xhci_intel_phy_mux_switch(xhci, id);

	pm_runtime_put(gadget->dev.parent);
	pm_runtime_put(host->controller);

	return 0;
}

static int cht_otg_set_vbus_valid(struct cht_otg *otg_dev, int vbus_valid)
{
	struct usb_bus *host = otg_dev->phy.otg->host;
	struct usb_gadget *gadget = otg_dev->phy.otg->gadget;
	struct usb_hcd *hcd;
	struct xhci_hcd *xhci;

	if (!host || !gadget || !gadget->dev.parent)
		return -ENODEV;

	hcd = bus_to_hcd(host);
	xhci = hcd_to_xhci(hcd);

	/* HACK: PHY used in Cherrytrail is shared by both host and device
	 * controller, it requires both host and device controller to be D0
	 * for any action related to PHY transition */
	pm_runtime_get_sync(host->controller);
	pm_runtime_get_sync(gadget->dev.parent);

	xhci_intel_phy_vbus_valid(xhci, vbus_valid);

	pm_runtime_put(gadget->dev.parent);
	pm_runtime_put(host->controller);

	return 0;
}

static int cht_otg_start_host(struct otg_fsm *fsm, int on)
{
	struct usb_otg *otg = fsm->otg;
	struct cht_otg *otg_dev = container_of(otg->usb_phy,
			struct cht_otg, phy);
	int retval;

	dev_dbg(otg->usb_phy->dev, "%s --->\n", __func__);

	if (!otg->host)
		return -ENODEV;

	/* Just switch the mux to host path */
	retval = cht_otg_set_id_mux(otg_dev, !on);

	dev_dbg(otg->usb_phy->dev, "%s <---\n", __func__);

	return retval;
}

static int cht_otg_start_gadget(struct otg_fsm *fsm, int on)
{
	struct usb_otg *otg = fsm->otg;
	struct cht_otg *otg_dev = container_of(otg->usb_phy, struct cht_otg, phy);
	int retval;

	dev_dbg(otg->usb_phy->dev, "%s --->\n", __func__);

	if (!otg->gadget)
		return -ENODEV;

	retval = cht_otg_set_vbus_valid(otg_dev, on);

	dev_dbg(otg->usb_phy->dev, "%s <---\n", __func__);

	return retval;
}

/* SRP / HNP / ADP are not supported, only simple dual role function
 * start gadget function is not implemented as controller will take
 * care itself per VBUS event */
static struct otg_fsm_ops cht_otg_ops = {
	.start_host = cht_otg_start_host,
	.start_gadget = cht_otg_start_gadget,
};

static int cht_otg_set_power(struct usb_phy *phy, unsigned mA)
{
	dev_dbg(phy->dev, "%s --->\n", __func__);

	if (!cht_otg_dev)
		return -ENODEV;

	if (phy->otg->state != OTG_STATE_B_PERIPHERAL)
		dev_err(phy->dev, "ERR: Draw %d mA in state %s\n",
			mA, usb_otg_state_string(cht_otg_dev->phy.otg->state));

	/* Notify other drivers that device enumerated or not.
	 * e.g It is needed by some charger driver, to set
	 * charging current for SDP case */
	atomic_notifier_call_chain(&cht_otg_dev->phy.notifier,
					USB_EVENT_ENUMERATED, &mA);
	dev_info(phy->dev, "Draw %d mA\n", mA);

	dev_dbg(phy->dev, "%s <---\n", __func__);

	return 0;
}

static void cht_otg_setup_extcon(void)
{
	int status;
	struct extcon_dev *c_edev;

	if (cht_otg_dev->c_edev)
		return;

	c_edev = extcon_get_extcon_dev(CHT_TYPEC_EXTCON_DEV_NAME);
	if (!c_edev) {
		dev_info(cht_otg_dev->phy.dev, "No extcon device, OTG support stubbed\n");
		return;
	}

	cht_otg_dev->c_edev = c_edev;
	cht_otg_dev->id_nb.notifier_call = cht_handle_extcon_otg_event;

	status = extcon_register_notifier(cht_otg_dev->c_edev, EXTCON_USB_HOST,
			&cht_otg_dev->id_nb);
	if (status)
		dev_warn(cht_otg_dev->phy.dev, "failed to register extcon notifier\n");
}

/*
 * Called by initialization code of host driver. Register host
 * controller to the OTG.
 */
static int cht_otg_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct cht_otg *otg_dev;

	cht_otg_setup_extcon();

	if (!otg || !host)
		return -ENODEV;

	otg_dev = container_of(otg->usb_phy, struct cht_otg, phy);
	if (otg_dev != cht_otg_dev)
		return -EINVAL;

	otg->host = host;

	/* once host registered, then kick statemachine to move
	 * to A_HOST if id is grounded */
	otg_dev->fsm.a_bus_drop = 0;
	otg_dev->fsm.a_bus_req = 0;

	/* check default ID value when host registered */
	if (otg_dev->c_edev)
		otg_dev->fsm.id = edev_state_to_id(otg_dev->c_edev);

	/* if gadget is registered already then kick the state machine.
	 * Only trigger the mode switch once both host and device are
	 * registered */
	if (otg_dev->phy.otg->gadget)
		otg_statemachine(&otg_dev->fsm);

	return 0;
}

/*
 * Called by initialization code of udc. Register udc to OTG.
 */
static int cht_otg_set_peripheral(struct usb_otg *otg,
					struct usb_gadget *gadget)
{
	struct cht_otg *otg_dev;

	cht_otg_setup_extcon();

	if (!otg || !gadget)
		return -ENODEV;

	otg_dev = container_of(otg->usb_phy, struct cht_otg, phy);
	if (otg_dev != cht_otg_dev)
		return -EINVAL;

	otg->gadget = gadget;

	otg_dev->fsm.b_bus_req = 1;

	/* if host is registered already then kick the state machine.
	 * Only trigger the mode switch once both host and device are
	 * registered */
	if (otg_dev->phy.otg->host)
		otg_statemachine(&otg_dev->fsm);

	return 0;
}

static int cht_otg_start(struct platform_device *pdev)
{
	struct cht_otg *otg_dev;
	struct usb_phy *phy_dev;
	struct otg_fsm *fsm;

	phy_dev = usb_get_phy(USB_PHY_TYPE_USB2);
	if (!phy_dev)
		return -ENODEV;

	otg_dev = container_of(phy_dev, struct cht_otg, phy);
	fsm = &otg_dev->fsm;

	/* Initialize the state machine structure with default values */
	phy_dev->otg->state = OTG_STATE_UNDEFINED;
	fsm->otg = otg_dev->phy.otg;
	mutex_init(&fsm->lock);

	/* Initialize the event spinlock */
	spin_lock_init(&cht_otg_dev->event_lock);

	/* Initialize the id value to default 1 */
	fsm->id = 1;
	otg_statemachine(fsm);

	dev_dbg(&pdev->dev, "initial ID pin set to %d\n", fsm->id);

	return 0;
}

static void cht_otg_stop(struct platform_device *pdev)
{
	if (!cht_otg_dev)
		return;

	if (cht_otg_dev->regs)
		iounmap(cht_otg_dev->regs);
}

static void cht_otg_event_count_inc(struct cht_otg *cht_otg_dev)
{
	unsigned long flags;

	if (!cht_otg_dev)
		return;

	spin_lock_irqsave(&cht_otg_dev->event_lock, flags);
	cht_otg_dev->event_count++;
	spin_unlock_irqrestore(&cht_otg_dev->event_lock, flags);
}

static void cht_otg_schedule_work(struct cht_otg *cht_otg_dev)
{
	cht_otg_event_count_inc(cht_otg_dev);
	schedule_work(&cht_otg_dev->fsm_work);
}

static int cht_otg_handle_notification(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct usb_bus *host;
	struct usb_gadget *gadget;
	int state;

	if (!cht_otg_dev)
		return NOTIFY_BAD;

	host = cht_otg_dev->phy.otg->host;
	gadget = cht_otg_dev->phy.otg->gadget;

	switch (event) {
	/* USB_EVENT_VBUS: vbus valid event */
	case USB_EVENT_VBUS:
		dev_info(cht_otg_dev->phy.dev, "USB_EVENT_VBUS vbus valid\n");
		if (cht_otg_dev->fsm.id)
			cht_otg_dev->fsm.b_sess_vld = 1;
		/* don't kick the state machine if host or device controller
		 * is not registered. Just wait to kick it when set_host or
		 * set_peripheral.*/
		if (host && gadget)
			cht_otg_schedule_work(cht_otg_dev);
		state = NOTIFY_OK;
		break;
	/* USB_EVENT_ID: id was grounded */
	case USB_EVENT_ID:
		dev_info(cht_otg_dev->phy.dev, "USB_EVENT_ID id ground\n");
		cht_otg_dev->fsm.id = 0;
		/* Same as above */
		if (host && gadget)
			cht_otg_schedule_work(cht_otg_dev);
		state = NOTIFY_OK;
		break;
	/* USB_EVENT_NONE: no events or cable disconnected */
	case USB_EVENT_NONE:
		dev_info(cht_otg_dev->phy.dev,
					"USB_EVENT_NONE cable disconnected\n");
		if (cht_otg_dev->fsm.id == 0)
			cht_otg_dev->fsm.id = 1;
		else if (cht_otg_dev->fsm.b_sess_vld)
			cht_otg_dev->fsm.b_sess_vld = 0;
		else
			dev_err(cht_otg_dev->phy.dev, "why USB_EVENT_NONE?\n");
		/* Same as above */
		if (host && gadget)
			cht_otg_schedule_work(cht_otg_dev);
		state = NOTIFY_OK;
		break;
	default:
		dev_info(cht_otg_dev->phy.dev, "unknown notification\n");
		state = NOTIFY_DONE;
		break;
	}

	return state;
}

static void cht_otg_fsm_work(struct work_struct *work)
{
	unsigned long flags;

	if (!cht_otg_dev)
		return;

	spin_lock_irqsave(&cht_otg_dev->event_lock, flags);

	while (cht_otg_dev->event_count) {
		spin_unlock_irqrestore(&cht_otg_dev->event_lock, flags);
		otg_statemachine(&cht_otg_dev->fsm);
		spin_lock_irqsave(&cht_otg_dev->event_lock, flags);
		cht_otg_dev->event_count--;
	}

	spin_unlock_irqrestore(&cht_otg_dev->event_lock, flags);
}

static ssize_t show_cht_otg_state(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct otg_fsm *fsm = &cht_otg_dev->fsm;
	char *next = buf;
	unsigned size = PAGE_SIZE;
	int t;

	mutex_lock(&fsm->lock);

	/* OTG state */
	t = scnprintf(next, size,
		      "OTG state: %s\n\n",
		      usb_otg_state_string(cht_otg_dev->phy.otg->state));
	size -= t;
	next += t;

	/* State Machine Variables */
	t = scnprintf(next, size,
			"a_bus_req: %d\n"
			"b_bus_req: %d\n"
			"a_bus_resume: %d\n"
			"a_bus_suspend: %d\n"
			"a_conn: %d\n"
			"a_sess_vld: %d\n"
			"a_srp_det: %d\n"
			"a_vbus_vld: %d\n"
			"b_bus_resume: %d\n"
			"b_bus_suspend: %d\n"
			"b_conn: %d\n"
			"b_se0_srp: %d\n"
			"b_ssend_srp: %d\n"
			"b_sess_vld: %d\n"
			"id: %d\n",
			fsm->a_bus_req,
			fsm->b_bus_req,
			fsm->a_bus_resume,
			fsm->a_bus_suspend,
			fsm->a_conn,
			fsm->a_sess_vld,
			fsm->a_srp_det,
			fsm->a_vbus_vld,
			fsm->b_bus_resume,
			fsm->b_bus_suspend,
			fsm->b_conn,
			fsm->b_se0_srp,
			fsm->b_ssend_srp,
			fsm->b_sess_vld,
			fsm->id);

	size -= t;
	next += t;

	mutex_unlock(&fsm->lock);

	return PAGE_SIZE - size;
}

static DEVICE_ATTR(cht_otg_state, S_IRUGO, show_cht_otg_state, NULL);

static ssize_t show_mux_state(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct otg_fsm *fsm = &cht_otg_dev->fsm;

	if (fsm->id && fsm->a_vbus_vld)
		return sprintf(buf, "peripheral state\n");
	else if (!fsm->id && !fsm->a_vbus_vld)
		return sprintf(buf, "host state\n");
	else
		return sprintf(buf, "unknown state\n");

}

static ssize_t store_mux_state(struct device *_dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct otg_fsm *fsm;

	if (!cht_otg_dev)
		return -EINVAL;

	fsm = &cht_otg_dev->fsm;

	if (count != 2)
		return -EINVAL;

	switch (buf[0]) {
	case 'D':
	case 'd':
	case 'P':
	case 'p':
	case 'a':
	case 'A':
	case '1':
		dev_info(cht_otg_dev->phy.dev, "p: set PERIPHERAL mode\n");
		/* disable host mode */
		dev_info(cht_otg_dev->phy.dev, "ID = 1\n");
		atomic_notifier_call_chain(&cht_otg_dev->phy.notifier,
			USB_EVENT_NONE, NULL);
		/* enable device mode */
		dev_info(cht_otg_dev->phy.dev, "VBUS = 1\n");
		atomic_notifier_call_chain(&cht_otg_dev->phy.notifier,
			USB_EVENT_VBUS, NULL);
		return count;
	case 'H':
	case 'h':
	case 'b':
	case 'B':
	case '0':
		dev_info(cht_otg_dev->phy.dev, "h: set HOST mode\n");
		/* disable device mode */
		dev_info(cht_otg_dev->phy.dev, "VBUS = 0\n");
		atomic_notifier_call_chain(&cht_otg_dev->phy.notifier,
			USB_EVENT_NONE, NULL);
		/* enable host mode */
		dev_info(cht_otg_dev->phy.dev, "ID = 0\n");
		atomic_notifier_call_chain(&cht_otg_dev->phy.notifier,
			USB_EVENT_ID, NULL);
		return count;
	default:
		return -EINVAL;
	}
	return count;
}

static DEVICE_ATTR(mux_state, S_IRWXU|S_IRWXG, show_mux_state, store_mux_state);

static ssize_t store_vbus_evt(struct device *_dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct otg_fsm *fsm;

	if (!cht_otg_dev)
		return -EINVAL;

	fsm = &cht_otg_dev->fsm;

	if (count != 2)
		return -EINVAL;

	switch (buf[0]) {
	case '1':
		dev_info(cht_otg_dev->phy.dev, "VBUS = 1\n");
		atomic_notifier_call_chain(&cht_otg_dev->phy.notifier,
			USB_EVENT_VBUS, NULL);
		return count;
	case '0':
		dev_info(cht_otg_dev->phy.dev, "VBUS = 0\n");
		atomic_notifier_call_chain(&cht_otg_dev->phy.notifier,
			USB_EVENT_NONE, NULL);
		return count;
	default:
		return -EINVAL;
	}
	return count;
}
static DEVICE_ATTR(vbus_evt, S_IWUSR|S_IWGRP, NULL, store_vbus_evt);

static ssize_t store_otg_id(struct device *_dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct otg_fsm *fsm;

	if (!cht_otg_dev)
		return -EINVAL;

	fsm = &cht_otg_dev->fsm;

	if (count != 2)
		return -EINVAL;

	switch (buf[0]) {
	case '0':
	case 'a':
	case 'A':
		dev_info(cht_otg_dev->phy.dev, "ID = 0\n");
		atomic_notifier_call_chain(&cht_otg_dev->phy.notifier,
			USB_EVENT_ID, NULL);
		return count;
	case '1':
	case 'b':
	case 'B':
		dev_info(cht_otg_dev->phy.dev, "ID = 1\n");
		atomic_notifier_call_chain(&cht_otg_dev->phy.notifier,
			USB_EVENT_NONE, NULL);
		return count;
	default:
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(otg_id, S_IWUSR|S_IWGRP, NULL, store_otg_id);

static int cht_handle_extcon_otg_event(struct notifier_block *nb,
					unsigned long event, void *param)
{
	struct extcon_dev *edev = param;
	int id = edev_state_to_id(edev);
	struct usb_bus *host;
	struct usb_gadget *gadget;

	if (!cht_otg_dev)
		return NOTIFY_DONE;

	host = cht_otg_dev->phy.otg->host;
	gadget = cht_otg_dev->phy.otg->gadget;

	dev_info(cht_otg_dev->phy.dev, "[extcon notification]: USB-Host: %s\n",
			id ? "Disconnected" : "connected");

	/* update id value and schedule fsm work to start/stop host per id */
	cht_otg_dev->fsm.id = id;

	/* don't kick the state machine if host or device controller
	 * is not registered. Just wait to kick it when set_host or
	 * set_peripheral.*/
	if (host && gadget)
		schedule_work(&cht_otg_dev->fsm_work);

	return NOTIFY_OK;
}

static int cht_otg_probe(struct platform_device *pdev)
{
	struct cht_otg *cht_otg;
	int status;

	cht_otg = kzalloc(sizeof(struct cht_otg), GFP_KERNEL);
	if (!cht_otg && &pdev->dev) {
		dev_err(&pdev->dev, "Failed to alloc cht_otg structure\n");
		return -ENOMEM;
	}
	cht_otg->phy.otg = kzalloc(sizeof(struct usb_otg), GFP_KERNEL);
	if (!cht_otg->phy.otg) {
		kfree(cht_otg);
		return -ENOMEM;
	}

	/* Set OTG state machine operations */
	cht_otg->fsm.ops = &cht_otg_ops;

	/* initialize the otg and phy structure */
	cht_otg->phy.label = DRIVER_DESC;
	cht_otg->phy.dev = &pdev->dev;
	cht_otg->phy.set_power = cht_otg_set_power;

	cht_otg->phy.otg->usb_phy = &cht_otg->phy;
	cht_otg->phy.otg->set_host = cht_otg_set_host;
	cht_otg->phy.otg->set_peripheral = cht_otg_set_peripheral;

	/* No support for ADP, HNP and SRP */
	cht_otg->phy.otg->start_hnp = NULL;
	cht_otg->phy.otg->start_srp = NULL;


	INIT_WORK(&cht_otg->fsm_work, cht_otg_fsm_work);

	cht_otg_dev = cht_otg;

	status = usb_add_phy(&cht_otg_dev->phy, USB_PHY_TYPE_USB2);
	if (status) {
		dev_err(&pdev->dev, "failed to add cht otg usb phy\n");
		goto err1;
	}

	cht_otg_dev->nb.notifier_call = cht_otg_handle_notification;
	usb_register_notifier(&cht_otg_dev->phy, &cht_otg_dev->nb);

	/* init otg-fsm */
	status = cht_otg_start(pdev);
	if (status) {
		dev_err(&pdev->dev, "failed to add cht otg usb phy\n");
		goto err2;
	}

	status = device_create_file(&pdev->dev, &dev_attr_cht_otg_state);
	if (status) {
		dev_err(&pdev->dev, "failed to create fsm sysfs attribute\n");
		goto err2;
	}

	status = device_create_file(&pdev->dev, &dev_attr_mux_state);
	if (status) {
		dev_err(&pdev->dev, "failed to create mux_status sysfs attribute\n");
		goto err3;
	}

	status = device_create_file(&pdev->dev, &dev_attr_vbus_evt);
	if (status) {
		dev_err(&pdev->dev, "failed to create vbus sysfs attribute\n");
		goto err4;
	}

	status = device_create_file(&pdev->dev, &dev_attr_otg_id);
	if (status) {
		dev_err(&pdev->dev, "failed to create id sysfs attribute\n");
		goto err5;
	}

	return 0;

err5:
	device_remove_file(&pdev->dev, &dev_attr_vbus_evt);
err4:
	device_remove_file(&pdev->dev, &dev_attr_cht_otg_state);
err3:
	device_remove_file(&pdev->dev, &dev_attr_mux_state);
err2:
	cht_otg_stop(pdev);
	usb_remove_phy(&cht_otg_dev->phy);
err1:
	kfree(cht_otg->phy.otg);
	kfree(cht_otg);
	return status;
}

static int cht_otg_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_otg_id);
	device_remove_file(&pdev->dev, &dev_attr_vbus_evt);
	device_remove_file(&pdev->dev, &dev_attr_mux_state);
	device_remove_file(&pdev->dev, &dev_attr_cht_otg_state);

	cht_otg_stop(pdev);
	if (cht_otg_dev->c_edev)
		extcon_unregister_notifier(cht_otg_dev->c_edev, EXTCON_USB_HOST,
					   &cht_otg_dev->id_nb);
	usb_remove_phy(&cht_otg_dev->phy);

	kfree(cht_otg_dev->phy.otg);
	kfree(cht_otg_dev);

	return 0;
}

struct platform_driver intel_cht_otg_driver = {
	.probe = cht_otg_probe,
	.remove = cht_otg_remove,
	.driver = {
		.name = driver_name,
		.owner = THIS_MODULE,
	},
};

static int __init cht_otg_phy_init(void)
{
	return platform_driver_register(&intel_cht_otg_driver);
}
subsys_initcall(cht_otg_phy_init);

static void __exit cht_otg_phy_exit(void)
{
	platform_driver_unregister(&intel_cht_otg_driver);
}
module_exit(cht_otg_phy_exit);

MODULE_DESCRIPTION(DRIVER_INFO);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
