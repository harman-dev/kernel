/*
 * Copyright (c) 2016--2017 Intel Corporation.
 *
 * Author: Jouni Ukkonen <jouni.ukkonen@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/pci.h>

#include <media/intel-ipu4-isys.h>
#include "../../../../include/media/crlmodule.h"
#include "../../pci/intel-ipu4/intel-ipu4.h"

#define ADV7481_HDMI_LANES      4
#define ADV7481_HDMI_I2C_ADDRESS 0xe0

#define ADV7481_LANES		1
/*
 * below i2c address is dummy one, to be able to register single
 * ADV7481 chip as two sensors
 */
#define ADV7481_I2C_ADDRESS	0xe1


#define GPIO_BASE		434

#ifdef CONFIG_INTEL_IPU4_DS90UB954
#define DS90UB954_LANES         2
#define DS90UB954_I2C_ADDRESS   0x30
#define CAM_LVDS_RST_GPIO       10
#define CAM_CHNL_SELECT_GPIO    18
#define DS90UB954_CSI_PORT_ID	2
#define DS90UB954_I2C_ADAPTER	6
#endif

#ifdef CONFIG_INTEL_IPU4_DS90UB954
/* The following ds90ub964 pdata for Deserializer */
static struct crlmodule_platform_data ds90ub954_pdata = {
       /* Since power on pin is pulled to 1.8v, gpio not required
       for power on / shut down */
       .lanes = DS90UB954_LANES,
       .ext_clk = 25000000,
       .op_sys_clock = (uint64_t []){ 400000000 },
       .module_name = "DS90UB954"
};

static struct intel_ipu4_isys_csi2_config ds90ub954_csi2_cfg = {
       .nlanes = DS90UB954_LANES,
       .port = DS90UB954_CSI_PORT_ID,
};

static struct intel_ipu4_isys_subdev_info ds90ub954_crl_sd = {
       .csi2 = &ds90ub954_csi2_cfg,
       .i2c = {
               .board_info = {
                         I2C_BOARD_INFO(CRLMODULE_NAME, DS90UB954_I2C_ADDRESS),
                         .platform_data = &ds90ub954_pdata,
               },
               .i2c_adapter_id = DS90UB954_I2C_ADAPTER,
       }
};
#endif

static struct crlmodule_platform_data adv7481_cvbs_pdata = {
	.ext_clk = 286363636,
	.xshutdown = GPIO_BASE + 64, /*dummy for now*/
	.lanes = ADV7481_LANES,
	.module_name = "ADV7481 CVBS"
};

static struct intel_ipu4_isys_csi2_config adv7481_cvbs_csi2_cfg = {
	.nlanes = ADV7481_LANES,
	.port = 4,
};

static struct intel_ipu4_isys_subdev_info adv7481_cvbs_crl_sd = {
	.csi2 = &adv7481_cvbs_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = CRLMODULE_NAME,
			 .flags = I2C_CLIENT_TEN,
			 .addr = ADV7481_I2C_ADDRESS,
			 .platform_data = &adv7481_cvbs_pdata,
		},
		.i2c_adapter_id = 0,
	}
};

static struct crlmodule_platform_data adv7481_hdmi_pdata = {
	/* FIXME: may need to revisit */
	.ext_clk = 286363636,
	.xshutdown = GPIO_BASE + 30,
	.lanes = ADV7481_HDMI_LANES,
	.module_name = "ADV7481 HDMI",
	.crl_irq_pin = GPIO_BASE + 22,
	.irq_pin_flags = (IRQF_TRIGGER_RISING | IRQF_ONESHOT),
	.irq_pin_name = "ADV7481_HDMI_IRQ",
};

static struct intel_ipu4_isys_csi2_config adv7481_hdmi_csi2_cfg = {
	.nlanes = ADV7481_HDMI_LANES,
	.port = 0,
};

static struct intel_ipu4_isys_subdev_info adv7481_hdmi_crl_sd = {
	.csi2 = &adv7481_hdmi_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = CRLMODULE_NAME,
			 .flags = I2C_CLIENT_TEN,
			 .addr = ADV7481_HDMI_I2C_ADDRESS,
			 .platform_data = &adv7481_hdmi_pdata,
		},
		.i2c_adapter_id = 0,
	}
};



/*
 * Map buttress output sensor clocks to sensors -
 * this should be coming from ACPI, in Gordon Peak
 * ADV7481 have its own oscillator, no buttres clock
 * needed.
 */
struct intel_ipu4_isys_clk_mapping clk_mapping[] = {
	{ CLKDEV_INIT(NULL, NULL, NULL), NULL }
};

static struct intel_ipu4_isys_subdev_pdata pdata = {
	.subdevs = (struct intel_ipu4_isys_subdev_info *[]) {
#ifdef CONFIG_INTEL_IPU4_ADV7481
		&adv7481_hdmi_crl_sd,
		&adv7481_cvbs_crl_sd,
#endif
#ifdef CONFIG_INTEL_IPU4_DS90UB954
                &ds90ub954_crl_sd,
#endif
		NULL,
	},
	.clk_map = clk_mapping,
};

static void ipu4_quirk(struct pci_dev *pci_dev)
{
	pci_dev->dev.platform_data = &pdata;
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, INTEL_IPU4_HW_BXT_P,
			ipu4_quirk);
