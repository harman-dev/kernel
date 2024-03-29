/*
 * Intel Vendor Defined XHCI extended capability
 *
 * Copyright (C) 2014 Intel Corp.
 *
 * Author: Wu, Hao
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program;
 */

#include <linux/usb/phy.h>
#include <linux/usb/otg.h>

#include "xhci.h"
#include "xhci-intel-cap.h"

int xhci_intel_vendor_cap_init(struct xhci_hcd *xhci)
{
	struct usb_hcd *hcd;
	int ext_offset, retval;
	int result;
	u32 data;

	ext_offset = XHCI_HCC_EXT_CAPS(readl(&xhci->cap_regs->hcc_params));
	ext_offset = xhci_find_ext_cap_by_id(&xhci->cap_regs->hc_capbase,
			ext_offset << 2, XHCI_EXT_CAPS_INTEL_HOST_CAP);
	if (!ext_offset)
		return -ENODEV;

	xhci->phy_mux_regs = &xhci->cap_regs->hc_capbase + (ext_offset >> 2);

	xhci_dbg(xhci, "Intel Vendor Defined Cap %d initialization\n",
					XHCI_EXT_CAPS_INTEL_HOST_CAP);
	xhci_dbg(xhci, "regs offset = 0x%x, phy_mux_regs = 0x%p\n",
					ext_offset, xhci->phy_mux_regs);

	/* If This capbility is found, register host on PHY for OTG purpose */
	hcd = xhci_to_hcd(xhci);
	hcd->usb_phy = usb_get_phy(USB_PHY_TYPE_USB2);

	if (!IS_ERR_OR_NULL(hcd->usb_phy)) {
		retval = otg_set_host(hcd->usb_phy->otg, &hcd->self);
		if (retval)
			usb_put_phy(hcd->usb_phy);
	} else {
		xhci_dbg(xhci, "No USB2 PHY transceiver found\n");
		result = PTR_ERR_OR_ZERO(hcd->usb_phy);
		hcd->usb_phy = NULL;
		return result;
	}

	xhci_dbg(xhci, "capability init done\n");

	/*force set the correct mux value*/
	data = readl(xhci->phy_mux_regs + DUAL_ROLE_CFG0);
	/* enable the VBUS */
	data |= SW_VBUS_VALID;
	/* enable SW_IDPIN_EN */
	data |= SW_IDPIN_EN;
	/* Select device mode [20]=0b */
	data |= SW_IDPIN;
	/* Select SW controlled DRD [16]=1b */
	data |= SW_SWITCH_EN;
	/* DRD config static device [1:0] = 10b */
	data = (data & (~SW_DRD_CFG_MASK))|SW_DRD_CFG_DEVICE;

	writel(data, xhci->phy_mux_regs + DUAL_ROLE_CFG0);

	return 0;
}
EXPORT_SYMBOL_GPL(xhci_intel_vendor_cap_init);

/* Only used for device mode */
int xhci_intel_phy_vbus_valid(struct xhci_hcd *xhci, int vbus_valid)
{
	u32		data;

	if (!xhci || !xhci->phy_mux_regs)
		return -ENODEV;

	xhci_dbg(xhci, "vbus valid for phy mux is %d\n", vbus_valid);

	data = readl(xhci->phy_mux_regs + DUAL_ROLE_CFG0);

	if (vbus_valid)
		data |= SW_VBUS_VALID;
	else
		data &= ~SW_VBUS_VALID;

	writel(data, xhci->phy_mux_regs + DUAL_ROLE_CFG0);

	return 0;
}
EXPORT_SYMBOL_GPL(xhci_intel_phy_vbus_valid);

int xhci_intel_phy_mux_switch(struct xhci_hcd *xhci, int is_device_on)
{
	unsigned long	timeout;
	u32		data;

	if (!xhci || !xhci->phy_mux_regs)
		pr_err("No XHCI or Not support phy mux capability\n");

	if (xhci == NULL) {
		return -ENOMEM;
	}
	xhci_dbg(xhci, "XHCI phy mux switch to %s path\n",
				is_device_on ? "dev" : "Host");


	/* Check and set mux to SW controlled mode */
	data = readl(xhci->phy_mux_regs + DUAL_ROLE_CFG0);
	if (!(data & SW_IDPIN_EN)) {
		data |= SW_IDPIN_EN;
		writel(data, xhci->phy_mux_regs + DUAL_ROLE_CFG0);
	}

	/* Configure CFG0 to switch the mux and VBUS_VALID bit is required
	 * for device mode */
	data = readl(xhci->phy_mux_regs + DUAL_ROLE_CFG0);
	if (is_device_on) {
		data |= SW_IDPIN;

		/* DRD config static device [1:0] = 10b */
		if(data & SW_SWITCH_EN)
			data = (data & (~SW_DRD_CFG_MASK))|SW_DRD_CFG_DEVICE;
	}
	else {
		data &= ~SW_IDPIN;

		/* DRD config static host [1:0] = 01b */
		if(data & SW_SWITCH_EN)
			data = (data & (~SW_DRD_CFG_MASK))|SW_DRD_CFG_HOST;
	}
	writel(data, xhci->phy_mux_regs + DUAL_ROLE_CFG0);

	/* Polling CFG1 for safety, most case it takes about 600ms to finish
	 * mode switching, set TIMEOUT long enough */
	timeout = jiffies + msecs_to_jiffies(DUAL_ROLE_CFG1_POLL_TIMEOUT);

	/* Polling on CFG1 register to confirm mode switch.*/
	while (!time_after(jiffies, timeout)) {
		data = readl(xhci->phy_mux_regs + DUAL_ROLE_CFG1);
		if (is_device_on) {
			if (!(data & SW_MODE))
				break;
		} else {
			if (data & SW_MODE)
				break;
		}
		/* interval for polling is set to about 5ms */
		usleep_range(5000, 5100);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(xhci_intel_phy_mux_switch);
