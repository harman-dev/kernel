/*
 * Copyright (c) 2016 Harman International.
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
#ifndef TI949_H
#define TI949_H

#include <linux/i2c.h>
#include <linux/regmap.h>

#define RECONFIG_SERDES     _IOW('s', 1, unsigned char)

#define TI949_NAME					"ti949"
#define TI948_I2C_ADDR				0x2C
#define TI949_I2C_ADDR				0x0C
#define FACEPLATE_I2C_ADDR			0x12
#define TI949_I2C_ADAPTER			7


#define REG_TI949_MODE_SEL		0x04
#define REG_TI949_GEN_STS		0x0C
#define REG_TI949_CHECKSUM		0xC4
#define REG_TI949_ICR			0xC6
#define REG_TI949_ISR			0xC7
#define REG_TI949_ICR_1			0xC2

#define REG_TI949_LINK_LOSS_BIT		0x10
#define REG_TI949_LINK_DET_BIT		0x01
#define REG_TI948_LOCK_BIT			0x01

#define CRC_ERROR_RESET_NORMAL		0x80
#define CRC_ERROR_RESET_CLEAR		0xA0
#define CHECKSUM_CLEAR_VAL			0x00

#define REG_TI948_GEN_STS		0x1C

struct ti949_pdata {
    unsigned short ti949_i2c_addr;
	unsigned short ti948_i2c_addr;
	unsigned short faceplate_i2c_addr;
    unsigned short i2c_adapter;
	int irq_intb;
	char irq_intb_name[20];
	int pdb_gpio;

};

struct ti949 {
	struct ti949_pdata *pdata;
	const char *name;
	struct regmap *ti949_regmap8;
	struct regmap *ti948_regmap8;
	struct regmap *faceplate;
	unsigned int irq;
	void *dev_priv;
	dev_t major_num;
    struct cdev *serdes_cdev;
    struct class *serdes_class;
};

#endif
