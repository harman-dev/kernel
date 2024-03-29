/*
 * Intel(R) Trace Hub pci driver
 *
 * Copyright (C) 2014-2015 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/types.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/pci.h>

#include "intel_th.h"

#define DRIVER_NAME "intel_th_pci"

#define BAR_MASK (BIT(TH_MMIO_CONFIG) | BIT(TH_MMIO_SW))

/*
 * PCI Configuration Registers
 */
enum {
	REG_PCI_NPKDSC		= 0x80, /* NPK Device Specific Control */
	REG_PCI_NPKDSD		= 0x90, /* NPK Device Specific Defeature */
};

/* Trace Hub software reset */
#define NPKDSC_RESET	BIT(1)

/* Force On */
#define NPKDSD_FON	BIT(0)

static void intel_th_pci_reset(struct intel_th *th)
{
	struct pci_dev *pdev = container_of(th->dev, struct pci_dev, dev);
	u32 val;

	/* Software reset */
	pci_read_config_dword(pdev, REG_PCI_NPKDSC, &val);
	val |= NPKDSC_RESET;
	pci_write_config_dword(pdev, REG_PCI_NPKDSC, val);

	/* Always set FON for S0ix flow */
	pci_read_config_dword(pdev, REG_PCI_NPKDSD, &val);
	val |= NPKDSD_FON;
	pci_write_config_dword(pdev, REG_PCI_NPKDSD, val);
}

static int intel_th_pci_probe(struct pci_dev *pdev,
			      const struct pci_device_id *id)
{
	struct intel_th *th;
	int err;

	err = pcim_enable_device(pdev);
	if (err)
		return err;

	/* Enable bus mastering */
	pci_set_master(pdev);

	err = pcim_iomap_regions_request_all(pdev, BAR_MASK, DRIVER_NAME);
	if (err)
		return err;

	th = intel_th_alloc(&pdev->dev, pdev->resource,
			    DEVICE_COUNT_RESOURCE, pdev->irq,
			    intel_th_pci_reset);
	if (IS_ERR(th))
		return PTR_ERR(th);

	pci_set_drvdata(pdev, th);

	return 0;
}

static void intel_th_pci_remove(struct pci_dev *pdev)
{
	struct intel_th *th = pci_get_drvdata(pdev);

	intel_th_free(th);
}

static const struct pci_device_id intel_th_pci_id_table[] = {
	{
		PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x9d26),
		.driver_data = (kernel_ulong_t)0,
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0xa126),
		.driver_data = (kernel_ulong_t)0,
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0X0a80),
		.driver_data = (kernel_ulong_t)0,
	},
	{
                PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1a8e),
                .driver_data = (kernel_ulong_t)0,
        },
	{
		/* Kaby Lake PCH-H */
		PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0xa2a6),
		.driver_data = (kernel_ulong_t)0,
	},
	{
		/* Apollo Lake */
		PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x5a8e),
		.driver_data = (kernel_ulong_t)0,
	},
	{
		/* Cannon Lake H */
		PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0xa326),
		.driver_data = (kernel_ulong_t)0,
	},
	{
		/* Cannon Lake LP */
		PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x9da6),
		.driver_data = (kernel_ulong_t)0,
	},
	{
		/* Gemini Lake */
		PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x318e),
		.driver_data = (kernel_ulong_t)0,
	},
	{ 0 },
};

MODULE_DEVICE_TABLE(pci, intel_th_pci_id_table);

static int intel_th_suspend(struct device *dev)
{
	/*
	 * Stub the call to avoid disabling the device.
	 * Suspend is fully handled by firmwares.
	 */
	return 0;
}

static int intel_th_resume(struct device *dev)
{
	/* Firmwares have already restored the device state. */
	return 0;
}

static const struct dev_pm_ops intel_th_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(intel_th_suspend,
				intel_th_resume)
};

static struct pci_driver intel_th_pci_driver = {
	.name		= DRIVER_NAME,
	.id_table	= intel_th_pci_id_table,
	.probe		= intel_th_pci_probe,
	.remove		= intel_th_pci_remove,
	.driver         = {
		.pm     = &intel_th_pm_ops,
	},
};

module_pci_driver(intel_th_pci_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel(R) Trace Hub PCI controller driver");
MODULE_AUTHOR("Alexander Shishkin <alexander.shishkin@intel.com>");
