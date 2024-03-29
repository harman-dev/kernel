/*
 * Copyright (c) 2016--2017 Intel Corporation.
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
#ifndef TI964_REG_H
#define TI964_REG_H

struct ti964_register_write {
	u8 reg;
	u8 val;
};

static const struct ti964_register_write ti964_init_settings[] = {
	{0x8, 0x1c},
	{0xa, 0x79},
	{0xb, 0x79},
	{0xd, 0xb9},
	{0x10, 0x91},
	{0x11, 0x85},
	{0x12, 0x89},
	{0x13, 0xc1},
	{0x17, 0xe1},
	{0x18, 0x0}, /* Disable frame sync. */
	{0x19, 0x0}, /* Frame sync high time. */
	{0x1a, 0x2},
	{0x1b, 0xa}, /* Frame sync low time. */
	{0x1c, 0xd3},
	{0x21, 0x43}, /* Enable best effort mode. */
	{0xb0, 0x10},
	{0xb1, 0x14},
	{0xb2, 0x1f},
	{0xb3, 0x8},
	{0x32, 0x1}, /* Select CSI port 0 */
	{0x4c, 0x1}, /* Select RX port 0 */
	{0x58, 0x58},
	{0x5c, 0x18}, /* TI913 alias addr 0xc */
	{0x6d, 0x7f},
	{0x70, 0x1e}, /* YUV422_8 */
	{0x7c, 0x81}, /* Use RAW10 8bit mode */
	{0xd2, 0x84},
	{0x4c, 0x12}, /* Select RX port 1 */
	{0x58, 0x58},
	{0x5c, 0x1a}, /* TI913 alias addr 0xd */
	{0x6d, 0x7f},
	{0x70, 0x5e}, /* YUV422_8 */
	{0x7c, 0x81}, /* Use RAW10 8bit mode */
	{0xd2, 0x84},
	{0x4c, 0x24}, /* Select RX port 2*/
	{0x58, 0x58},
	{0x5c, 0x1c}, /* TI913 alias addr 0xe */
	{0x6d, 0x7f},
	{0x70, 0x9e}, /* YUV422_8 */
	{0x7c, 0x81}, /* Use RAW10 8bit mode */
	{0xd2, 0x84},
	{0x4c, 0x38}, /* Select RX port3 */
	{0x58, 0x58},
	{0x5c, 0x1e}, /* TI913 alias addr 0xf */
	{0x6d, 0x7f},
	{0x70, 0xde}, /* YUV422_8 */
	{0x7c, 0x81}, /* Use RAW10 8bit mode */
	{0xd2, 0x84},
};

static const struct ti964_register_write ti964_tp_settings[] = {
	{0xb0, 0x0},
	{0xb1, 0x02},
	{0xb2, 0xb3},
	{0xb1, 0x01},
};

#define TI964_DEVID		0x0
#define TI964_RESET		0x1
#define TI964_CSI_PLL_CTL	0x1f
#define TI964_FS_CTL		0x18
#define TI964_FWD_CTL1		0x20
#define TI964_RX_PORT_SEL	0x4c
#define TI964_SLAVE_ID0		0x5d
#define TI964_SLAVE_ALIAS_ID0	0x65
#define TI964_PORT_CONFIG	0x6d
#define TI964_BC_GPIO_CTL0	0x6e
#define TI964_PORT_CONFIG2	0x7c
#define TI964_IND_ACC_DATA	0xb2
#define TI964_CSI_CTL           0x33

#define TI964_POWER_ON		0x1
#define TI964_POWER_OFF		0x20
#define TI964_FPD3_RAW10_100MHz	0x7f
#define TI964_FPD3_RAW12_50MHz	0x7d
#define TI964_FPD3_RAW12_75MHz	0x7e
#define TI964_RAW10_NORMAL	0x1
#define TI964_RAW10_8BIT	0x81
#define TI964_GPIO0_HIGH	0x09
#define TI964_GPIO0_LOW		0x08
#define TI964_GPIO1_HIGH	0x90
#define TI964_GPIO1_LOW		0x80
#define TI964_GPIO0_FSIN	0x0a
#define TI964_GPIO1_FSIN	0xa0
#define TI964_GPIO0_MASK	0x0f
#define TI964_GPIO1_MASK	0xf0
#define TI964_MIPI_800MBPS	0x2
#define TI964_MIPI_1600MBPS	0x0
#define TI964_CSI_ENABLE	0x1
#define TI964_CSI_CONTS_CLOCK	0x2
#define TI964_CSI_SKEWCAL	0x40
#define TI964_FSIN_ENABLE	0x1

#endif
