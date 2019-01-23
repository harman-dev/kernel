
/*
 * crl_ds90ub954_configuration.c
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

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include "crlmodule.h"
#include "crlmodule-regs.h"
#include "serdes-regs.h"

#define CAM_LVDS_RST	444
#define CHNL_SELECT		452
#define ERRATA_VER_ID      0x00
#define PRODUCTION_VER_ID  0x20

static void ds90ub954_status_read_cbk(unsigned long data);
int ds90ub954_status_read(struct i2c_client *client);
int ds90ub954_status_read_work(struct work_struct *work);


typedef struct {
        struct work_struct work;
        struct i2c_client *client;
} irq_task_t;

static struct workqueue_struct *irq_workqueue;

static int ds90ub954_i2c_write(struct i2c_client *client, u16 i2c_addr, u16 reg, u8 val)
{
        struct v4l2_subdev *subdev = i2c_get_clientdata(client);
        struct crl_sensor *sensor = to_crlmodule_sensor(subdev);

        return crlmodule_write_reg(sensor, i2c_addr, reg, 1, 0xFF, val);
}

static int ds90ub954_valeo_i2c_write(struct i2c_client *client, u16 i2c_addr, u16 reg, u8 val)
{
        struct v4l2_subdev *subdev = i2c_get_clientdata(client);
        struct crl_sensor *sensor = to_crlmodule_sensor(subdev);

        return crlmodule_write_reg(sensor, i2c_addr, reg, CRL_REG_LEN_08BIT | CRL_REG_ADDR_LEN_16BIT, 0xFF, val);
}


static int ds90ub954_i2c_read(struct i2c_client *client, u16 i2c_addr, u16 reg, u32 *val)
{
        struct v4l2_subdev *subdev = i2c_get_clientdata(client);
        struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
        struct crl_register_read_rep read_reg;

        read_reg.address = reg;
        read_reg.len = CRL_REG_LEN_08BIT;
        read_reg.dev_i2c_addr = i2c_addr;
        return crlmodule_read_reg(sensor, read_reg, val);
}

static int ds90ub954_valeo_i2c_read(struct i2c_client *client, u16 i2c_addr, u16 reg, u32 *val)
{
        struct v4l2_subdev *subdev = i2c_get_clientdata(client);
        struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
        struct crl_register_read_rep read_reg;

        read_reg.address = reg;
        read_reg.len = CRL_REG_LEN_08BIT | CRL_REG_ADDR_LEN_16BIT;
        read_reg.dev_i2c_addr = i2c_addr;
        return crlmodule_read_reg(sensor, read_reg, val);
}


int ds90ub954_init(struct i2c_client *client)
{
	int rev_id = 0;
	int ret = 0;
	if (devm_gpio_request_one(&client->dev, CAM_LVDS_RST, 0,
                                "CRL-954 PDB") != 0) {
                                dev_err(&client->dev, "unable to acquire: CRL-954 PDB %d\n",
                                CAM_LVDS_RST);
                                return -ENODEV;
	}

	if (devm_gpio_request_one(&client->dev, CHNL_SELECT, 0,
                                "CRL-954 REFCLK") != 0) {
                                dev_err(&client->dev, "unable to acquire: CRL-954 REFCLK %d\n",
                                CHNL_SELECT);
                                return -ENODEV;
    }

	/*
		Below sequence including i2c writes is as per TI errata for
		Engneering version of 954:
		REFCLK input needs to be applied after PDB is asserted
		TODO: Revisit this changes for 954 production version
	*/
	
	gpio_set_value(CAM_LVDS_RST, 0);
    dev_dbg(&client->dev, "Setting CAM_LVDS_RST to 0\n");
	usleep_range(5000, 5000 + 10);
	gpio_set_value(CAM_LVDS_RST, 1);
    dev_dbg(&client->dev, "Setting CAM_LVDS_RST to 1\n");
	usleep_range(5000, 5000 + 10);

	ret = ds90ub954_i2c_read(client, 0x00, 0x03, &rev_id);
	if(ret)
	{
        dev_err(&client->dev, "%s: Read REVISION ID/MASK ID reg error\n", __func__);
	}
	else
	{
		if(rev_id == ERRATA_VER_ID)
		{
			gpio_set_value(CHNL_SELECT, 0);
			dev_dbg(&client->dev, "Setting CHNL_SELECT to 0\n");
			usleep_range(5000, 5000 + 10);
			ds90ub954_i2c_write(client, 0x00, 0xb0, 0x1c);
			ds90ub954_i2c_write(client, 0x00, 0xb1, 0x15);
			ds90ub954_i2c_write(client, 0x00, 0xb2, 0x30);
			ds90ub954_i2c_write(client, 0x00, 0xb0, 0x1c);
			ds90ub954_i2c_write(client, 0x00, 0xb1, 0x15);
			ds90ub954_i2c_write(client, 0x00, 0xb2, 0x00);
			gpio_set_value(CHNL_SELECT, 1);
			dev_dbg(&client->dev, "Setting CHNL_SELECT to 1\n");
			usleep_range(5000, 5000 + 10);
		}
		else if(rev_id == PRODUCTION_VER_ID)
		{
			printk("Production Version\n");
			dev_dbg(&client->dev, "Prod. Ver of 954, REFCLK Not required\n");
		}
	}
	return 0;
}

static struct timer_list my_timer;
struct status_regs
{
	u16 addr;
	char name[20];
} status_regs_954[] = { { 0x02, "GENERAL_CONFIG" },
		 { 0x04, "DEVICE_STATUS" },
		 { 0x4d, "RX_PORT_STS_1" },
		 { 0x4e, "RX_PORT_STS_2" },
		 { 0x35, "CSI_STS" },
		 { 0x72, "CSI_VC_MAP"},
		 { 0x73, "LINE_COUNT_HI"},
		 { 0x74, "LINE_COUNT_LO"},
		 { 0x75, "LINE_LEN_1"},
		 { 0x76, "LINE_LEN_0"},
		};



int ds90ub954_status_read(struct i2c_client *client)
{
	if(client) {
	        dev_err(&client->dev, "%s: setting up timer for status read from 954\n", __func__);
		irq_workqueue = create_workqueue("954_irq_workqueue");
		setup_timer(&my_timer, ds90ub954_status_read_cbk, (unsigned long) client);
		/* setup timer interval to 2 secs */
	        mod_timer(&my_timer, jiffies + msecs_to_jiffies(2000));
	}
#if 0
	else {
	        printk(KERN_CRIT "%s: delete timer for status read from 954\n", __func__);
		del_timer_sync(&my_timer);
                flush_workqueue(irq_workqueue);
                destroy_workqueue(irq_workqueue);
                irq_workqueue = NULL;
	}
#endif
	return 0;
}

/*
 * Handles timer interrupt
 */
static void ds90ub954_status_read_cbk(unsigned long data)
{
        irq_task_t *task = NULL;

        task = (irq_task_t*) kmalloc(sizeof(irq_task_t), GFP_ATOMIC);
        if (task) {
                INIT_WORK( (struct work_struct *) task, ds90ub954_status_read_work);
                task->client = (struct i2c_client *) data;
		printk(KERN_CRIT "%s: Setup work fn to read status..\n", __func__);
                queue_work(irq_workqueue, (struct work_struct *)task);
        }
}


int ds90ub954_status_read_work(struct work_struct *work)
{
	int ret;
	int val;
	int reg_cnt;
	int i;
	static int call_cnt = 0;

	irq_task_t *task = (irq_task_t*) work;
        struct i2c_client *client = task->client;
        dev_err(&client->dev, "%s: Reading status read from 954\n", __func__);

	for (i = 0; i < ARRAY_SIZE(status_regs_954); i++) {
		ret = ds90ub954_i2c_read(client, 0x00, status_regs_954[i].addr, &val);
		if (ret)
			dev_err(&client->dev, "%s: Error reading 954 status register name: %s,  addr: 0x%x!\n", \
			__func__, status_regs_954[i].name, status_regs_954[i].addr);
		else
			dev_err(&client->dev, "%s: 954 status register name: %s,  addr: 0x%02x, value: 0x%02x\n", \
			__func__, status_regs_954[i].name, status_regs_954[i].addr, val);

        }

	if(call_cnt == 3) {
                dev_err(&client->dev, "%s: DUMPING 954 REGS 0x00 - 0x7F\n", __func__);
		for(reg_cnt = 0x00; reg_cnt <= 0x7f; reg_cnt++) {
			ret = ds90ub954_i2c_read(client, 0x00, reg_cnt, &val);
			if (ret)
				dev_err(&client->dev, "%s: reading error: addr: 0x%02x\n", __func__, reg_cnt);
			else
				dev_err(&client->dev, "%s: addr: 0x%02x, value: 0x%02x\n", __func__, reg_cnt, val);
		}

                dev_err(&client->dev, "%s: DUMPING 954 REGS 0xa0 - 0xfb\n", __func__);
		for(reg_cnt = 0xA0; reg_cnt <= 0xFB; reg_cnt++) {
                        ret = ds90ub954_i2c_read(client, 0x00, reg_cnt, &val);
                        if (ret)
                                dev_err(&client->dev, "%s: reading error: addr: 0x%02x\n", __func__, reg_cnt);
                        else
                                dev_err(&client->dev, "%s: addr: 0x%02x, value: 0x%02x\n", __func__, reg_cnt, val);
                }
	}


	if(call_cnt < 5) {
		call_cnt++;
		/* setup timer interval to 2 secs */
		mod_timer(&my_timer, jiffies + msecs_to_jiffies(2000));
	} else {
		del_timer(&my_timer);
                //flush_workqueue(irq_workqueue);
                //destroy_workqueue(irq_workqueue);
                //irq_workqueue = NULL;
	}

	return ret;
}
