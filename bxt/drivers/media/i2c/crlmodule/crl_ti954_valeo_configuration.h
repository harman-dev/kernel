/*
 * crl_ti954_valeo_configuration.h
 * Copyright (C) 2016 Harman International Ltd,
 *
 * Author: Sreeju Arumugan Selvaraj <sreeju.selvaraj@harman.com>
 * Created on: 18-08-2016
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

#ifndef __CRLMODULE_TI954_VALEO_CONFIGURATION_H__
#define __CRLMODULE_TI954_VALEO_CONFIGURATION_H__

#include "crlmodule-sensor-ds.h"
#include "serdes-regs.h"


#define DEBUG_954		0
static struct crl_register_write_rep ti954_valeo_onetime_init_regset[] = {
        /* Set FV MIN TIME */
	{0xbc, CRL_REG_LEN_08BIT, 0x00, 0x00},

	/* Enable only RX1 */
	{0x0c, CRL_REG_LEN_08BIT, 0x16, 0x00},

	/* Enable Read RX1 port regs */
	{0x4c, CRL_REG_LEN_08BIT, 0x12, 0x00},

	/* RAW10,  VC = 0 */
	{0x70, CRL_REG_LEN_08BIT, 0x1e, 0x00},

	/* 8 bit format, Set FV, LV Polarity  */
	{0x7c, CRL_REG_LEN_08BIT, 0x80, 0x00},

	/* Back channel freq: 2.5 */
	{0x58, CRL_REG_LEN_08BIT, 0x58, 0x00},

	/* Disable All port forwarding */
	{0x20, CRL_REG_LEN_08BIT, 0xf0, 0x00},

	/* Enable CSI */
	{0x32, CRL_REG_LEN_08BIT, 0x01, 0x00},
	{0x33, CRL_REG_LEN_08BIT, 0x21, 0x00},
	{0x34, CRL_REG_LEN_08BIT, 0x01, 0x00},

	/* Enable CSI Replicate */
	{0x21, CRL_REG_LEN_08BIT, 0x81, 0x00},

	/* Enable port forwarding only for RX1 */
	{0x20, CRL_REG_LEN_08BIT, 0xd0, 0x00},

	/* Revisit this delay value after checking with TI */
	/* {0x00, CRL_REG_LEN_DELAY, SER_DES_INIT_DELAY, 0x00}, */

	/* Force RAW10 FPD3 mode */
	{0x6d, CRL_REG_LEN_08BIT, 0x7f, 0x00},

    /*Back Channel GPIO0 Select.Camera switch to 15FPS after updating the camera firmware*/
	{0x6E, CRL_REG_LEN_08BIT, 0x80, 0x00},

	/* Reset pin */
	/*{0x6e, CRL_REG_LEN_08BIT, 0x89, 0x00},*/
};



static struct crl_register_write_rep ti954_valeo_streamon_regs[] = {
};

static struct crl_register_write_rep ti954_valeo_streamoff_regs[] = {
};


static struct crl_register_read_rep ti954_valeo_status_regset[] = {
	{ FPD3_DES_SER_ID, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ FPD3_DESER_CSI_STS, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ FPD3_DESER_RX_PORT_STS1, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ FPD3_DESER_RX_PORT_STS2, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ FPD3_DESER_CSI_RX_STS, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ FPD3_DES_FWD_CTL1, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ 0x1f, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ 0x6d, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ 0x22, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ 0x24, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ 0x70, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ 0xB8, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ 0x51, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ 0x52, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ 0x53, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ 0x54, CRL_REG_LEN_08BIT, 0x000000ff, 0x00 },
	{ FPD3_SER_GENERAL_STATUS, CRL_REG_LEN_08BIT, 0x000000ff, SER_ALIAS_ID },
	{ FPD3_SER_MODE_SELECT, CRL_REG_LEN_08BIT, 0x000000ff, SER_ALIAS_ID },
	{ 0x03, CRL_REG_LEN_08BIT, 0x000000ff, SER_ALIAS_ID },
	{ 0x35, CRL_REG_LEN_08BIT, 0x000000ff, SER_ALIAS_ID },
};


const s64 ti954_valeo_op_sys_clock[] =  { 400000000 };

static struct crl_pll_configuration ti954_valeo_pll_configurations[] = {
        {
                .input_clk = 25000000,
		.op_sys_clk = 400000000, /* Deserializer is configured for 800Mbps */
                .pixel_rate_csi = 200000000,
                .pixel_rate_pa = 200000000,
                .bitsperpixel = 16,
                .comp_items = 0,
                .ctrl_data = 0,
                .pll_regs_items = 0,
                .pll_regs = NULL,
		.csi_lanes = 2,
         },
};

static struct crl_subdev_rect_rep ti954_valeo_720p_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect.left = 0,
		.in_rect.top = 0,
		.in_rect.width = 1280,
		.in_rect.height = 720,
		.out_rect.left = 0,
		.out_rect.top = 0,
		.out_rect.width = 1280,
		.out_rect.height = 720,
	 },

        {
                .subdev_type = CRL_SUBDEV_TYPE_BINNER,
                .in_rect.left = 0,
                .in_rect.top = 0,
                .in_rect.width = 1280,
                .in_rect.height = 720,
                .out_rect.left = 0,
                .out_rect.top = 0,
                .out_rect.width = 1280,
                .out_rect.height = 720,
        },
};


struct crl_ctrl_data_pair ctrl_data_vpm[] = {
        {
                .ctrl_id = CRL_CID_MY20_VPM,
                .data = 0,
        },
};

static struct crl_mode_rep ti954_valeo_modes[] = {
	{
		.sd_rects_items = ARRAY_SIZE(ti954_valeo_720p_rects),
		.sd_rects = ti954_valeo_720p_rects,
		.binn_hor = 1,
		.binn_vert = 1,
		.scale_m = 1,
		.width = 1280,
		.height = 720,
                .min_llp = 1905,
                .min_fll = 840,
		.comp_items = 1,
		.ctrl_data = &ctrl_data_vpm[0],
	 },
};

static struct crl_sensor_subdev_config ti954_valeo_sensor_subdevs[] = {
        {
                .subdev_type = CRL_SUBDEV_TYPE_BINNER,
                .name = "valeo binner",
        },

	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.name = "valeo pixel array",
	 },
};

static struct crl_sensor_limits ti954_valeo_sensor_limits = {
	.x_addr_min = 0,
	.y_addr_min = 0,
	.x_addr_max = 1280,
	.y_addr_max = 720,
	.min_frame_length_lines = 280,
	.max_frame_length_lines = 840,
	.min_line_length_pixels = 440,
	.max_line_length_pixels = 1905,
        .scaler_m_min = 1,
        .scaler_m_max = 1,
        .scaler_n_min = 1,
        .scaler_n_max = 1,
        .min_even_inc = 1,
        .max_even_inc = 1,
        .min_odd_inc = 1,
        .max_odd_inc = 1,
};



static struct crl_csi_data_fmt ti954_valeo_crl_csi_data_fmt[] = {
#if 0
        {
                .code = MEDIA_BUS_FMT_YUYV8_1X16,
                .pixel_order = CRL_PIXEL_ORDER_GRBG,
                .bits_per_pixel = 16,
                .regs_items = ARRAY_SIZE(valeo_fmt_yuyv8),
                .regs = valeo_fmt_yuyv8,

        },
#endif
        {
                .code = MEDIA_BUS_FMT_UYVY8_1X16,
                .pixel_order = CRL_PIXEL_ORDER_GRBG,
                .bits_per_pixel = 16,
                .regs_items = 0,
                .regs = 0,
        },
};


static struct crl_arithmetic_ops ti954_portcfg_ops[] = {
        {
                .op = CRL_BITWISE_LSHIFT,
                .operand.entity_type = CRL_DYNAMIC_VAL_OPERAND_TYPE_CONST,
                .operand.entity_val = 6,
        },
        {
                .op = CRL_BITWISE_OR,
                .operand.entity_type = CRL_DYNAMIC_VAL_OPERAND_TYPE_CONST,
                .operand.entity_val = 0x80,
        },
};

static struct crl_dynamic_register_access ti954_portcfg_regs[] = {
        {
                .address = 0x7c,
                .len = CRL_REG_LEN_08BIT | CRL_REG_READ_AND_UPDATE,
                .ops_items = ARRAY_SIZE(ti954_portcfg_ops),
                .ops = ti954_portcfg_ops,
                .mask = 0xc0,
        },
};


static struct crl_v4l2_ctrl ti954_valeo_v4l2_ctrls[] = {

        {
                .sd_type = CRL_SUBDEV_TYPE_BINNER,
                .op_type = CRL_V4L2_CTRL_SET_OP,
                .context = SENSOR_IDLE,
                .ctrl_id = V4L2_CID_LINK_FREQ,
                .name = "V4L2_CID_LINK_FREQ",
                .type = CRL_V4L2_CTRL_TYPE_MENU_INT,
                .data.v4l2_int_menu.def = 0,
                .data.v4l2_int_menu.max = ARRAY_SIZE(ti954_valeo_pll_configurations) - 1,
                .data.v4l2_int_menu.menu = ti954_valeo_op_sys_clock,
                .flags = 0,
                .impact = CRL_IMPACTS_NO_IMPACT,
                .regs_items = 0,
                .regs = 0,
                .dep_items = 0,
                .dep_ctrls = 0,
        },
	{
                .sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
                .op_type = CRL_V4L2_CTRL_GET_OP,
                .context = SENSOR_POWERED_ON,
                .ctrl_id = V4L2_CID_PIXEL_RATE,
                .name = "V4L2_CID_PIXEL_RATE_PA",
                .type = CRL_V4L2_CTRL_TYPE_INTEGER,
                .data.std_data.min = 0,
                .data.std_data.max = 0,
                .data.std_data.step = 1,
                .data.std_data.def = 0,
                .flags = 0,
                .impact = CRL_IMPACTS_NO_IMPACT,
                .regs_items = 0,
                .regs = 0,
                .dep_items = 0,
                .dep_ctrls = 0,
        },
	{
                .sd_type = CRL_SUBDEV_TYPE_BINNER,
                .op_type = CRL_V4L2_CTRL_GET_OP,
                .context = SENSOR_POWERED_ON,
                .ctrl_id = V4L2_CID_PIXEL_RATE,
                .name = "V4L2_CID_PIXEL_RATE_CSI",
                .type = CRL_V4L2_CTRL_TYPE_INTEGER,
                .data.std_data.min = 0,
                .data.std_data.max = 0,
                .data.std_data.step = 1,
                .data.std_data.def = 0,
                .flags = 0,
                .impact = CRL_IMPACTS_NO_IMPACT,
                .regs_items = 0,
                .regs = 0,
                .dep_items = 0,
                .dep_ctrls = 0,
        },
        {
                .sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
                .op_type = CRL_V4L2_CTRL_SET_OP,
                .context = SENSOR_POWERED_ON,
                .ctrl_id = CRL_CID_MY20_VPM,
                .name = "Camera Input VPM",
                .type = CRL_V4L2_CTRL_TYPE_CUSTOM,
                .data.std_data.min = 0,
                .data.std_data.max = 1,
                .data.std_data.step = 1,
                .data.std_data.def = 0,
                .flags = V4L2_CTRL_FLAG_UPDATE,
                .impact = CRL_IMPACTS_NO_IMPACT,
                .ctrl = 0,
                .regs_items = ARRAY_SIZE(ti954_portcfg_regs),
                .regs = ti954_portcfg_regs,
                .dep_items = 0,
                .dep_ctrls = 0,
                .v4l2_type = V4L2_CTRL_TYPE_INTEGER,
        },
};


int ds90ub954_init(struct i2c_client *);
#if (DEBUG_954)
int ds90ub954_status_read(struct i2c_client *);
#endif

struct crl_sensor_configuration ti954_valeo_crl_configuration = {

        .custom_sensor_init = ds90ub954_init,
#if (DEBUG_954)
        .sensor_status_read = ds90ub954_status_read,
#endif
	.onetime_init_regs_items = ARRAY_SIZE(ti954_valeo_onetime_init_regset),
	.onetime_init_regs = ti954_valeo_onetime_init_regset,
	.poweroff_regs_items = 0,
	.poweroff_regs = 0,

	.subdev_items = ARRAY_SIZE(ti954_valeo_sensor_subdevs),
	.subdevs = ti954_valeo_sensor_subdevs,

	.sensor_limits = &ti954_valeo_sensor_limits,

	.pll_config_items = ARRAY_SIZE(ti954_valeo_pll_configurations),
	.pll_configs = ti954_valeo_pll_configurations,

	.modes_items = ARRAY_SIZE(ti954_valeo_modes),
	.modes = ti954_valeo_modes,

	.streamon_regs_items = ARRAY_SIZE(ti954_valeo_streamon_regs),
	.streamon_regs = ti954_valeo_streamon_regs,

	.streamoff_regs_items = ARRAY_SIZE(ti954_valeo_streamoff_regs),
	.streamoff_regs = ti954_valeo_streamoff_regs,

	.v4l2_ctrls_items = ARRAY_SIZE(ti954_valeo_v4l2_ctrls),
	.v4l2_ctrl_bank = ti954_valeo_v4l2_ctrls,

	.csi_fmts_items = ARRAY_SIZE(ti954_valeo_crl_csi_data_fmt),
	.csi_fmts = ti954_valeo_crl_csi_data_fmt,

	.addr_len = CRL_ADDR_8BIT,
};

#endif  /* __CRLMODULE_TI954_VALEO_CONFIGURATION_H__ */
