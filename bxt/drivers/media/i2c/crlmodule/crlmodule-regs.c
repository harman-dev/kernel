/*
 * Copyright (c) 2014--2017 Intel Corporation.
 *
 * Author: Vinod Govindapillai <vinod.govindapillai@intel.com>
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
#include <linux/delay.h>

#include "crlmodule.h"
#include "crlmodule-nvm.h"
#include "crlmodule-regs.h"
#include "serdes-regs.h"

static DEFINE_MUTEX(crl_i2c_mutex);

static int crlmodule_i2c_read(struct crl_sensor *sensor, u16 dev_i2c_addr,
				u16 reg, u8 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->src->sd);
	struct i2c_msg msg[2];
	unsigned char data[4];
	int r;

	u8 addr_len = ((len & CRL_REG_ADDR_LEN_MASK) >> 3) + 1;
	u8 reg_len = len & CRL_REG_LEN_MASK;

	dev_dbg(&client->dev, "%s reg, len: [0x%04x, %d]", __func__, reg, len);

	if ( reg_len != CRL_REG_LEN_08BIT && reg_len != CRL_REG_LEN_16BIT &&
			reg_len != CRL_REG_LEN_24BIT && reg_len != CRL_REG_LEN_32BIT)
		return -EINVAL;

	if ( addr_len != CRL_REG_LEN_08BIT && addr_len != CRL_REG_LEN_16BIT )
		return -EINVAL;

	if (dev_i2c_addr == CRL_I2C_ADDRESS_NO_OVERRIDE)
		msg[0].addr = client->addr;
	else
		msg[0].addr = dev_i2c_addr;

	msg[1].addr = msg[0].addr;

	msg[0].flags = 0;
	msg[0].buf = data;

	if (sensor->sensor_ds->addr_len == CRL_ADDR_7BIT) {
		/* change address to 7bit format */
		msg[0].addr = msg[0].addr >> 1;
		msg[1].addr = msg[1].addr >> 1;
	}

	if(addr_len == CRL_REG_LEN_08BIT) {
		data[0] = (u8) (reg & 0xff);
		msg[0].len = 1;
	} else {
		/* high byte goes out first */
		data[0] = (u8) (reg >> 8);
		data[1] = (u8) (reg & 0xff);
		msg[0].len = 2;
	}

	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = reg_len;

	r = i2c_transfer(client->adapter, msg, 2);

	if (r < 0)
		goto err;

	*val = 0;
	/* high byte comes first */
	switch (reg_len) {
	case CRL_REG_LEN_32BIT:
		*val = (data[0] << 24) + (data[1] << 16) + (data[2] << 8) +
			data[3];
		break;
	case CRL_REG_LEN_24BIT:
		*val = (data[0] << 16) + (data[1] << 8) + data[2];
		break;
	case CRL_REG_LEN_16BIT:
		*val = (data[0] << 8) + data[1];
		break;
	case CRL_REG_LEN_08BIT:
		*val = data[0];
		break;
	}

	return 0;

err:
	dev_err(&client->dev, "read from offset 0x%x error %d\n", reg, r);

	return r;
}

static int crlmodule_i2c_write(struct crl_sensor *sensor, u16 dev_i2c_addr,
			       u16 reg, u8 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->src->sd);
	struct i2c_msg msg;
	unsigned char data[6];
	unsigned int retries;
	int r;
	unsigned char *data_offset;
	u8 reg_len = len & CRL_REG_LEN_MASK;
	u8 addr_len = ((len & CRL_REG_ADDR_LEN_MASK) >> 3) + 1;

	if (reg_len != CRL_REG_LEN_08BIT && reg_len != CRL_REG_LEN_16BIT &&
	    reg_len != CRL_REG_LEN_24BIT && reg_len != CRL_REG_LEN_32BIT)
		return -EINVAL;

	if (addr_len != CRL_REG_LEN_08BIT && addr_len != CRL_REG_LEN_16BIT)
		return -EINVAL;

	if (dev_i2c_addr == CRL_I2C_ADDRESS_NO_OVERRIDE)
		msg.addr = client->addr;
	else
		msg.addr = dev_i2c_addr;

	msg.flags = 0; /* Write */
	msg.buf = data;

	if (sensor->sensor_ds->addr_len == CRL_ADDR_7BIT)
		msg.addr = msg.addr >> 1;

	if(addr_len == CRL_REG_LEN_08BIT) {
		data[0] = (u8) (reg & 0xff);
		msg.len = 1 + reg_len;
		data_offset = &data[1];
	} else {
		/* high byte goes out first */
		data[0] = (u8) (reg >> 8);
		data[1] = (u8) (reg & 0xff);
		msg.len = 2 + reg_len;
		data_offset = &data[2];
	}

	dev_dbg(&client->dev, "%s len reg, val: [%d, 0x%04x, 0x%04x]",
			       __func__, len, reg, val);

	switch (reg_len) {
	case CRL_REG_LEN_08BIT:
		data_offset[0] = val;
		break;
	case CRL_REG_LEN_16BIT:
		data_offset[0] = val >> 8;
		data_offset[1] = val;
		break;
	case CRL_REG_LEN_24BIT:
		data_offset[0] = val >> 16;
		data_offset[1] = val >> 8;
		data_offset[2] = val;
		break;
	case CRL_REG_LEN_32BIT:
		data_offset[0] = val >> 24;
		data_offset[1] = val >> 16;
		data_offset[2] = val >> 8;
		data_offset[3] = val;
		break;
	}

	for (retries = 0; retries < 5; retries++) {
		/*
		 * Due to unknown reason sensor stops responding. This
		 * loop is a temporaty solution until the root cause
		 * is found.
		 */
		r = i2c_transfer(client->adapter, &msg, 1);
		if (r == 1) {
			if (retries)
				dev_err(&client->dev,
					"sensor i2c stall encountered. retries: %d\n",
					retries);
			return 0;
		}

		usleep_range(2000, 2000);
	}

	dev_err(&client->dev,
		"wrote 0x%x to offset 0x%x error %d\n", val, reg, r);

	return r;
}

int crlmodule_read_reg(struct crl_sensor *sensor,
		       const struct crl_register_read_rep reg, u32 *val)
{
	return crlmodule_i2c_read(sensor, reg.dev_i2c_addr, reg.address,
				  reg.len, val);
}

int crlmodule_write_reg(struct crl_sensor *sensor, u16 dev_i2c_addr, u16 reg,
			u8 len, u32 mask, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->src->sd);
	int ret;
	u32 val2;

	/*
	 * Sensor setting sequence may need some delay.
	 * delay value is specified by reg.val field
	 */
	if (len == CRL_REG_LEN_DELAY) {
		msleep(val);
		return 0;
	}

	/*
	 * If the same register is being used for two settings, updating
	 * one value should not overwrite the other one. Such registers
	 * must be marked as CRL_REG_READ_AND_UPDATE. For such registers
	 * first read the register and update it
	 */

	if (len & CRL_REG_READ_AND_UPDATE) {
		u32 tmp;
		/* Some rare cases 2 different devices can
		 * make i2c accesses to same physical i2c address,
		 * those read modify writes must be protected by static
		 * mutex
		 */
		if (sensor->sensor_ds->i2c_mutex_in_use)
			mutex_lock(&crl_i2c_mutex);

		ret = crlmodule_i2c_read(sensor, dev_i2c_addr, reg,
					 len & CRL_REG_LEN_READ_MASK, &val2);
		if (ret) {
			if (sensor->sensor_ds->i2c_mutex_in_use)
				mutex_unlock(&crl_i2c_mutex);
			return ret;
		}

		tmp = val2 & ~mask;
		tmp |= val & mask;
		val = tmp;
	}

	ret = crlmodule_i2c_write(sensor, dev_i2c_addr, reg,
				  len & CRL_REG_LEN_READ_MASK, val);

	if ((sensor->sensor_ds->i2c_mutex_in_use)
			&& (len & CRL_REG_READ_AND_UPDATE))
		mutex_unlock(&crl_i2c_mutex);

	if (ret < 0) {
		dev_err(&client->dev,
			"error %d writing reg 0x%4.4x, val 0x%2.2x",
			ret, reg, val);
		return ret;
	}

	return 0;
}

int crlmodule_write_regs(struct crl_sensor *sensor,
			 const struct crl_register_write_rep *regs, int len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = crlmodule_write_reg(sensor,
					regs[i].dev_i2c_addr,
					regs[i].address,
					regs[i].len,
					regs[i].mask,
					regs[i].val);
		if (ret < 0)
			return ret;
	};

	return 0;
}

int crlmodule_block_read(struct crl_sensor *sensor, u16 dev_i2c_addr, u16 addr,
			 u8 addr_mode, u16 len, u8 *buf)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->src->sd);
	struct i2c_msg msg[2];
	u8 data[2];
	u16 offset = 0;
	int r;

	memset(msg, 0, sizeof(msg));

	if (dev_i2c_addr == CRL_I2C_ADDRESS_NO_OVERRIDE) {
		msg[0].addr = client->addr;
		msg[1].addr = client->addr;
	} else {
		msg[0].addr = dev_i2c_addr;
		msg[1].addr = dev_i2c_addr;
	}

	if (addr_mode & CRL_NVM_ADDR_MODE_8BIT)
		msg[0].len = 1;
	else if (addr_mode & CRL_NVM_ADDR_MODE_16BIT)
		msg[0].len = 2;
	else
		return -EINVAL;

	msg[0].flags = 0;
	msg[1].flags = I2C_M_RD;

	while (offset < len) {
		if (addr_mode & CRL_NVM_ADDR_MODE_8BIT) {
			data[0] = addr & 0xff;
		} else {
			data[0] = (addr >> 8) & 0xff;
			data[1] = addr & 0xff;
		}

		msg[0].buf = data;
		msg[1].len = min(CRLMODULE_I2C_BLOCK_SIZE, len - offset);
		msg[1].buf = &buf[offset];
		r = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (r != ARRAY_SIZE(msg)) {
			if (r >= 0)
				r = -EIO;
			goto err;
		}
		addr += msg[1].len;
		offset += msg[1].len;
	}
	return 0;
err:
	dev_err(&client->dev, "read from offset 0x%x error %d\n", offset, r);
	return r;
}
