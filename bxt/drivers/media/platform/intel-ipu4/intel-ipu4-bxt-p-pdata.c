/*
 * Copyright (c) 2015--2017 Intel Corporation.
 *
 * Author: Jianxu Zheng <jian.xu.zheng@intel.com>
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
#include "../../../../include/media/ti964.h"
#include "../../pci/intel-ipu4/intel-ipu4.h"

#define GPIO_BASE		422

#ifdef CONFIG_INTEL_IPU4_OV2740
#define OV2740_LANES		2
#define OV2740_I2C_ADDRESS	0x36
static struct crlmodule_platform_data ov2740_pdata = {
	.xshutdown = GPIO_BASE + 64,
	.lanes = OV2740_LANES,
	.ext_clk = 19200000,
	.op_sys_clock = (uint64_t []){ 72000000 },
	.module_name = "INT3474",
	.id_string = "0x27 0x40",
};

static struct intel_ipu4_isys_csi2_config ov2740_csi2_cfg = {
	.nlanes = OV2740_LANES,
	.port = 0,
};

static struct intel_ipu4_isys_subdev_info ov2740_crl_sd = {
	.csi2 = &ov2740_csi2_cfg,
	.i2c = {
		.board_info = {
			 I2C_BOARD_INFO(CRLMODULE_NAME, OV2740_I2C_ADDRESS),
			.platform_data = &ov2740_pdata,
		},
		.i2c_adapter_id = 2,
	}
};
#endif

#ifdef CONFIG_INTEL_IPU4_IMX185
#define IMX185_LANES		4
#define IMX185_I2C_ADDRESS	0x1a

static struct crlmodule_platform_data imx185_pdata = {
	.xshutdown = GPIO_BASE + 71,
	.lanes = IMX185_LANES,
	.ext_clk = 27000000,
	.op_sys_clock = (uint64_t []){ 55687500, 111375000,
					111375000, 222750000 },
	.module_name = "IMX185",
	.id_string = "0x1 0x85",
};

static struct intel_ipu4_isys_csi2_config imx185_csi2_cfg = {
	.nlanes = IMX185_LANES,
	.port = 0,
};

static struct intel_ipu4_isys_subdev_info imx185_crl_sd = {
	.csi2 = &imx185_csi2_cfg,
	.i2c = {
		.board_info = {
			 I2C_BOARD_INFO(CRLMODULE_NAME, IMX185_I2C_ADDRESS),
			.platform_data = &imx185_pdata,
		},
		.i2c_adapter_id = 2,
	}
};

static struct crlmodule_platform_data imx185_b_pdata = {
	.xshutdown = GPIO_BASE + 73,
	.lanes = IMX185_LANES,
	.ext_clk = 27000000,
	.op_sys_clock = (uint64_t []){ 55687500, 111375000,
					111375000, 222750000 },
	.module_name = "IMX185",
	.id_string = "0x1 0x85",
};

static struct intel_ipu4_isys_csi2_config imx185_b_csi2_cfg = {
	.nlanes = IMX185_LANES,
	.port = 4,
};

static struct intel_ipu4_isys_subdev_info imx185_b_crl_sd = {
	.csi2 = &imx185_b_csi2_cfg,
	.i2c = {
		.board_info = {
			 I2C_BOARD_INFO(CRLMODULE_NAME, IMX185_I2C_ADDRESS),
			.platform_data = &imx185_b_pdata,
		},
		.i2c_adapter_id = 4,
	}
};

#endif

#ifdef CONFIG_INTEL_IPU4_IMX477
#define IMX477_LANES	   2

#define IMX477_I2C_ADDRESS 0x10

static struct crlmodule_platform_data imx477_pdata_master = {
	.xshutdown = GPIO_BASE + 71,
	.lanes = IMX477_LANES,
	.ext_clk = 19200000,
	.op_sys_clock = (uint64_t []){600000000},
	.module_name = "IMX477-MASTER",
	.id_string = "0x4 0x77",
};

static struct intel_ipu4_isys_csi2_config imx477_csi2_cfg_master = {
	.nlanes = IMX477_LANES,
	.port = 0,
};

static struct intel_ipu4_isys_subdev_info imx477_crl_sd_master = {
	.csi2 = &imx477_csi2_cfg_master,
	.i2c = {
		.board_info = {
			I2C_BOARD_INFO(CRLMODULE_NAME, IMX477_I2C_ADDRESS),
			.platform_data = &imx477_pdata_master,
		},
		.i2c_adapter_id = 2,
	}
};

static struct crlmodule_platform_data imx477_pdata_slave_1 = {
	.xshutdown = GPIO_BASE + 73,
	.lanes = IMX477_LANES,
	.ext_clk = 19200000,
	.op_sys_clock = (uint64_t []){600000000},
	.module_name = "IMX477-SLAVE-1",
	.id_string = "0x4 0x77",
};

static struct intel_ipu4_isys_csi2_config imx477_csi2_cfg_slave_1 = {
	.nlanes = IMX477_LANES,
	.port = 4,
};

static struct intel_ipu4_isys_subdev_info imx477_crl_sd_slave_1 = {
	.csi2 = &imx477_csi2_cfg_slave_1,
	.i2c = {
		.board_info = {
			I2C_BOARD_INFO(CRLMODULE_NAME, IMX477_I2C_ADDRESS),
			.platform_data = &imx477_pdata_slave_1,
		},
		.i2c_adapter_id = 4,
	}
};
#endif

#ifdef CONFIG_INTEL_IPU4_IMX274

#define IMX274_LANES		4
#define IMX274_I2C_ADDRESS	0x1a

static struct crlmodule_platform_data imx274_pdata = {
	.xshutdown = GPIO_BASE + 64,
	.lanes = IMX274_LANES,
	.ext_clk = 24000000,
	.op_sys_clock = (uint64_t []){720000000},
	.module_name = "IMX274",
	.id_string = "0x6 0x9",
};

static struct intel_ipu4_isys_csi2_config imx274_csi2_cfg = {
	.nlanes = IMX274_LANES,
	.port = 0,
};

static struct intel_ipu4_isys_subdev_info imx274_crl_sd = {
	.csi2 = &imx274_csi2_cfg,
	.i2c = {
		.board_info = {
			I2C_BOARD_INFO(CRLMODULE_NAME, IMX274_I2C_ADDRESS),
			.platform_data = &imx274_pdata
		},
		.i2c_adapter_id = 2,
	}
};

static struct crlmodule_platform_data imx274_b_pdata = {
	.xshutdown = GPIO_BASE + 67,
	.lanes = IMX274_LANES,
	.ext_clk = 24000000,
	.op_sys_clock = (uint64_t []){720000000},
	.module_name = "IMX274",
	.id_string = "0x6 0x9",
};

static struct intel_ipu4_isys_csi2_config imx274_b_csi2_cfg = {
	.nlanes = IMX274_LANES,
	.port = 4,
};

static struct intel_ipu4_isys_subdev_info imx274_b_crl_sd = {
	.csi2 = &imx274_b_csi2_cfg,
	.i2c = {
		.board_info = {
			I2C_BOARD_INFO(CRLMODULE_NAME, IMX274_I2C_ADDRESS),
			.platform_data = &imx274_b_pdata
		},
		.i2c_adapter_id = 4,
	}
};
#endif

#ifdef CONFIG_INTEL_IPU4_IMX290

#define IMX290_LANES		4
#define IMX290_I2C_ADDRESS	0x1a

static struct crlmodule_platform_data imx290_pdata = {
	.xshutdown = GPIO_BASE + 64,
	.lanes = IMX290_LANES,
	.ext_clk = 24000000,
	.op_sys_clock = (uint64_t []){111375000, 445500000},
	.module_name = "IMX290",
	.id_string = "0x30 0x11",
};

static struct intel_ipu4_isys_csi2_config imx290_csi2_cfg = {
	.nlanes = IMX290_LANES,
	.port = 0,
};

static struct intel_ipu4_isys_subdev_info imx290_crl_sd = {
	.csi2 = &imx290_csi2_cfg,
	.i2c = {
		.board_info = {
			I2C_BOARD_INFO(CRLMODULE_NAME, IMX290_I2C_ADDRESS),
			.platform_data = &imx290_pdata
		},
		.i2c_adapter_id = 2,
	}
};
#endif

#ifdef CONFIG_INTEL_IPU4_OV13860

#define OV13860_LANES		2
#define OV13860_I2C_ADDRESS	0x10

static struct crlmodule_platform_data ov13860_pdata = {
	.xshutdown = GPIO_BASE + 71,
	.lanes = OV13860_LANES,
	.ext_clk = 24000000,
	.op_sys_clock = (uint64_t []){ 600000000, 300000000},
	.module_name = "OV13860"
};

static struct intel_ipu4_isys_csi2_config ov13860_csi2_cfg = {
	.nlanes = OV13860_LANES,
	.port = 0,
};

static struct intel_ipu4_isys_subdev_info ov13860_crl_sd = {
	.csi2 = &ov13860_csi2_cfg,
	.i2c = {
		.board_info = {
			 I2C_BOARD_INFO(CRLMODULE_NAME, OV13860_I2C_ADDRESS),
			 .platform_data = &ov13860_pdata,
		},
		.i2c_adapter_id = 2,
	}
};
#endif

#ifdef CONFIG_VIDEO_BU64295

#define BU64295_VCM_ADDR	0x0c
#define BU64295_NAME		"bu64295"

static struct intel_ipu4_isys_subdev_info bu64295_sd = {
	.i2c = {
		.board_info = {
			I2C_BOARD_INFO(BU64295_NAME,  BU64295_VCM_ADDR),
		},
		.i2c_adapter_id = 2,
	}
};
#endif

#ifdef CONFIG_INTEL_IPU4_ADV7481

#define ADV7481_LANES		4
#define ADV7481_I2C_ADDRESS	0xe0
#define ADV7481B_I2C_ADDRESS	0xe2

static struct crlmodule_platform_data adv7481_pdata = {
	.xshutdown = GPIO_BASE + 63,
	.lanes = ADV7481_LANES,
	.ext_clk = 24000000,
	.op_sys_clock = (uint64_t []){600000000},
	.module_name = "ADV7481"
};

static struct intel_ipu4_isys_csi2_config adv7481_csi2_cfg = {
	.nlanes = ADV7481_LANES,
	.port = 0,
};

static struct intel_ipu4_isys_subdev_info adv7481_crl_sd = {
	.csi2 = &adv7481_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = CRLMODULE_NAME,
			 .flags = I2C_CLIENT_TEN,
			 .addr = ADV7481_I2C_ADDRESS,
			 .platform_data = &adv7481_pdata,
		},
		.i2c_adapter_id = 2,
	}
};
#endif

#ifdef CONFIG_INTEL_IPU4_ADV7481_EVAL

#define ADV7481_LANES		4
#define ADV7481_I2C_ADDRESS	0xe0
#define ADV7481B_I2C_ADDRESS	0xe2

static struct crlmodule_platform_data adv7481_eval_pdata = {
	.xshutdown = GPIO_BASE + 63,
	.lanes = ADV7481_LANES,
	.ext_clk = 24000000,
	.op_sys_clock = (uint64_t []){600000000},
	.module_name = "ADV7481_EVAL"
};

static struct intel_ipu4_isys_csi2_config adv7481_eval_csi2_cfg = {
	.nlanes = ADV7481_LANES,
	.port = 0,
};

static struct intel_ipu4_isys_subdev_info adv7481_eval_crl_sd = {
	.csi2 = &adv7481_eval_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = CRLMODULE_NAME,
			 .flags = I2C_CLIENT_TEN,
			 .addr = ADV7481_I2C_ADDRESS,
			 .platform_data = &adv7481_eval_pdata,
		},
		.i2c_adapter_id = 2,
	}
};

static struct crlmodule_platform_data adv7481b_eval_pdata = {
	.xshutdown = GPIO_BASE + 63,
	.lanes = ADV7481_LANES,
	.ext_clk = 24000000,
	.op_sys_clock = (uint64_t []){600000000},
	.module_name = "ADV7481B_EVAL"
};

static struct intel_ipu4_isys_csi2_config adv7481b_eval_csi2_cfg = {
	.nlanes = ADV7481_LANES,
	.port = 4,
};

static struct intel_ipu4_isys_subdev_info adv7481b_eval_crl_sd = {
	.csi2 = &adv7481b_eval_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = CRLMODULE_NAME,
			 .flags = I2C_CLIENT_TEN,
			 .addr = ADV7481B_I2C_ADDRESS,
			 .platform_data = &adv7481b_eval_pdata,
		},
		.i2c_adapter_id = 2,
	}
};
#endif

#ifdef CONFIG_VIDEO_AGGREGATOR_STUB

#define VIDEO_AGGRE_LANES	4
#define VIDEO_AGGRE_I2C_ADDRESS	0x3b
#define VIDEO_AGGRE_B_I2C_ADDRESS	0x3c

static struct intel_ipu4_isys_csi2_config video_aggre_csi2_cfg = {
	.nlanes = VIDEO_AGGRE_LANES,
	.port = 0,
};

static struct intel_ipu4_isys_subdev_info video_aggre_stub_sd = {
	.csi2 = &video_aggre_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = "video-aggre",
			 .addr = VIDEO_AGGRE_I2C_ADDRESS,
		},
		.i2c_adapter_id = 2,
	}
};

static struct intel_ipu4_isys_csi2_config video_aggre_b_csi2_cfg = {
	.nlanes = VIDEO_AGGRE_LANES,
	.port = 4,
};

static struct intel_ipu4_isys_subdev_info video_aggre_b_stub_sd = {
	.csi2 = &video_aggre_b_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = "video-aggre",
			 .addr = VIDEO_AGGRE_B_I2C_ADDRESS,
		},
		.i2c_adapter_id = 2,
	}
};
#endif

#ifdef CONFIG_INTEL_IPU4_OV10635
#define OV10635_LANES		4
#define OV10635_I2C_PHY_ADDR	0x60 /* 0x30 for 7bit addr */
#define OV10635A_I2C_ADDRESS	0x61
#define OV10635B_I2C_ADDRESS	0x62
#define OV10635C_I2C_ADDRESS	0x63
#define OV10635D_I2C_ADDRESS	0x64

static struct crlmodule_platform_data ov10635_pdata = {
	.lanes = OV10635_LANES,
	.ext_clk = 24000000,
	.op_sys_clock = (uint64_t []){ 400000000 },
	.module_name = "OV10635",
	.id_string = "0xa6 0x35",
	/*
	 * The pin number of xshutdown will be determined
	 * and replaced inside TI964 driver.
	 * The number here stands for which GPIO to connect with.
	 * 1 means to connect sensor xshutdown to GPIO1
	 */
	.xshutdown = 0,
};
#endif

#ifdef CONFIG_INTEL_IPU4_OV10640
#define OV10640_LANES			4
#define OV10640_I2C_PHY_ADDR	0x60 /* 0x30 for 7bit addr */
#define OV10640A_I2C_ADDRESS	0x61
#define OV10640B_I2C_ADDRESS	0x62
#define OV10640C_I2C_ADDRESS	0x63
#define OV10640D_I2C_ADDRESS	0x64

static struct crlmodule_platform_data ov10640_pdata = {
	.lanes = OV10640_LANES,
	.ext_clk = 24000000,
	.op_sys_clock = (uint64_t []){ 400000000 },
	.module_name = "OV10640",
	.id_string = "0xa6 0x40",
	/*
	 * The pin number of xshutdown will be determined
	 * and replaced inside TI964 driver.
	 * The number here stands for which GPIO to connect with.
	 * 1 means to connect sensor xshutdown to GPIO1
	 */
	.xshutdown = 1,
};
#endif

#ifdef CONFIG_VIDEO_TI964
#define TI964_I2C_ADAPTER	0
#define TI964_I2C_ADAPTER_2	7
#define TI964_I2C_ADDRESS	0x3d
#define TI964_LANES		4

static struct intel_ipu4_isys_csi2_config ti964_csi2_cfg = {
	.nlanes = TI964_LANES,
	.port = 0,
};

static struct intel_ipu4_isys_csi2_config ti964_csi2_cfg_2 = {
	.nlanes = TI964_LANES,
	.port = 4,
};

struct ti964_subdev_info ti964_subdevs[] = {
#ifdef CONFIG_INTEL_IPU4_OV10635
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10635A_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER,
		.rx_port = 0,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
	},
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10635B_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER,
		.rx_port = 1,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
	},
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10635C_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER,
		.rx_port = 2,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
	},
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10635D_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER,
		.rx_port = 3,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
	},
#endif
#ifdef CONFIG_INTEL_IPU4_OV10640
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10640A_I2C_ADDRESS,
			.platform_data = &ov10640_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER,
		.rx_port = 0,
		.phy_i2c_addr = OV10640_I2C_PHY_ADDR,
	},
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10640B_I2C_ADDRESS,
			.platform_data = &ov10640_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER,
		.rx_port = 1,
		.phy_i2c_addr = OV10640_I2C_PHY_ADDR,
	},
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10640C_I2C_ADDRESS,
			.platform_data = &ov10640_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER,
		.rx_port = 2,
		.phy_i2c_addr = OV10640_I2C_PHY_ADDR,
	},
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10640D_I2C_ADDRESS,
			.platform_data = &ov10640_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER,
		.rx_port = 3,
		.phy_i2c_addr = OV10640_I2C_PHY_ADDR,
	},
#endif
};

struct ti964_subdev_info ti964_subdevs_2[] = {
#ifdef CONFIG_INTEL_IPU4_OV10635
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10635A_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER_2,
		.rx_port = 0,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
	},
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10635B_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER_2,
		.rx_port = 1,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
	},
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10635C_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER_2,
		.rx_port = 2,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
	},
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10635D_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER_2,
		.rx_port = 3,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
	},
#endif
#ifdef CONFIG_INTEL_IPU4_OV10640
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10640A_I2C_ADDRESS,
			.platform_data = &ov10640_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER_2,
		.rx_port = 0,
		.phy_i2c_addr = OV10640_I2C_PHY_ADDR,
	},
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10640B_I2C_ADDRESS,
			.platform_data = &ov10640_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER_2,
		.rx_port = 1,
		.phy_i2c_addr = OV10640_I2C_PHY_ADDR,
	},
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10640C_I2C_ADDRESS,
			.platform_data = &ov10640_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER_2,
		.rx_port = 2,
		.phy_i2c_addr = OV10640_I2C_PHY_ADDR,
	},
	{
		.board_info = {
			.type = CRLMODULE_NAME,
			.addr = OV10640D_I2C_ADDRESS,
			.platform_data = &ov10640_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER_2,
		.rx_port = 3,
		.phy_i2c_addr = OV10640_I2C_PHY_ADDR,
	},
#endif
};

static struct ti964_pdata ti964_pdata = {
	.subdev_info = ti964_subdevs,
	.subdev_num = ARRAY_SIZE(ti964_subdevs),
	.reset_gpio = GPIO_BASE + 63,
};

static struct intel_ipu4_isys_subdev_info ti964_sd = {
	.csi2 = &ti964_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = "ti964",
			 .addr = TI964_I2C_ADDRESS,
			 .platform_data = &ti964_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER,
	}
};

static struct ti964_pdata ti964_pdata_2 = {
	.subdev_info = ti964_subdevs_2,
	.subdev_num = ARRAY_SIZE(ti964_subdevs_2),
	.reset_gpio = GPIO_BASE + 66,
};

static struct intel_ipu4_isys_subdev_info ti964_sd_2 = {
	.csi2 = &ti964_csi2_cfg_2,
	.i2c = {
		.board_info = {
			 .type = "ti964",
			 .addr = TI964_I2C_ADDRESS,
			 .platform_data = &ti964_pdata_2,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER_2,
	}
};
#endif

/*
 * Map buttress output sensor clocks to sensors -
 * this should be coming from ACPI
 */
struct intel_ipu4_isys_clk_mapping clk_mapping[] = {
	{ CLKDEV_INIT("2-0036", NULL, NULL), "OSC_CLK_OUT0" },
	{ CLKDEV_INIT("2-001a", NULL, NULL), "OSC_CLK_OUT0" },
	{ CLKDEV_INIT("4-001a", NULL, NULL), "OSC_CLK_OUT1" },
	{ CLKDEV_INIT("2-0010", NULL, NULL), "OSC_CLK_OUT0" },
	{ CLKDEV_INIT("4-0010", NULL, NULL), "OSC_CLK_OUT1" },
	{ CLKDEV_INIT("2-a0e0", NULL, NULL), "OSC_CLK_OUT0" },
	{ CLKDEV_INIT("2-a0e2", NULL, NULL), "OSC_CLK_OUT0" },
	{ CLKDEV_INIT(NULL, NULL, NULL), NULL }
};

static struct intel_ipu4_isys_subdev_pdata pdata = {
	.subdevs = (struct intel_ipu4_isys_subdev_info *[]) {
#ifdef CONFIG_INTEL_IPU4_OV2740
		&ov2740_crl_sd,
#endif
#ifdef CONFIG_INTEL_IPU4_IMX185
		&imx185_crl_sd,
		&imx185_b_crl_sd,
#endif
#ifdef CONFIG_INTEL_IPU4_IMX477
		&imx477_crl_sd_slave_1,
		&imx477_crl_sd_master,
#endif
#ifdef CONFIG_INTEL_IPU4_IMX274
		&imx274_crl_sd,
		&imx274_b_crl_sd,
#endif
#ifdef CONFIG_INTEL_IPU4_IMX290
		&imx290_crl_sd,
#endif
#ifdef CONFIG_INTEL_IPU4_OV13860
		&ov13860_crl_sd,
#endif
#ifdef CONFIG_VIDEO_BU64295
		&bu64295_sd,
#endif
#ifdef CONFIG_INTEL_IPU4_ADV7481
		&adv7481_crl_sd,
#endif
#ifdef CONFIG_INTEL_IPU4_ADV7481_EVAL
		&adv7481_eval_crl_sd,
		&adv7481b_eval_crl_sd,
#endif
#ifdef CONFIG_VIDEO_AGGREGATOR_STUB
		&video_aggre_stub_sd,
		&video_aggre_b_stub_sd,
#endif
#ifdef CONFIG_VIDEO_TI964
		&ti964_sd,
		&ti964_sd_2,
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
