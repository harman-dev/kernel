 /* Copyright (c) 2016 Harman International.
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
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <media/ti949.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/fs.h>

#define SER_SLAVE_ID_REG                    0x07
#define MXT640_SLAVE_DEVICE_ID              0x96
#define SER_SLAVE_ALIAS_REG                 0x08
#define SER_CONFIG0_REG                     0x03
#define DESER_CONFIG0_REG                   0x03
#define LVDS_BYPASS                         0xda
#define SER_GPIO0_REG                       0x0D
#define SER_GPIO0_VALUE                     0x0f /*Serializer gpio0 in Remote-default mode */
                                                 /*and output value after link loss is High*/
#define SER_GPIO_1_2_REG                    0x0E
#define SER_GPIO_1_2_VALUE                  0x77
#define DESER_GPIO0_REG                     0x1D
#define DESER_GPIO0_VALUE                   0x23
#define DESER_GPIO_1_2_REG                  0x1E
#define DESER_GPIO_1_2_VALUE                0x33
#define SER_SLAVE2_ID_REG                   0x71
#define NFC_SLAVE_DEVICE_ID                 0x50
#define SER_SLAVE2_ALIAS_REG                0x78

/* for NFC_PRG and NFC_RST */

#define DESER_GPIO_7_8_REG                  0x21
#define DESER_GPIO_7_8_VALUE                0x11

/* Try changing the Freq DSER */

#define DESER_CONFIG_SCL_HIGHTIME_REG       0x26
#define DESER_CONFIG_SCL_HIGHTIME_VALUE     0x20
#define DESER_CONFIG_SCL_LOWTIME_REG        0x27
#define DESER_CONFIG_SCL_LOWTIME_VALUE      0x20
#define SER_REG_SLAVE_ID3                   0x72
#define SER_REG_SLAVE_ALIAS                 0x79
#define FACEPLATE_SLAVE_ID                  0x24
#define FACEPLATE_REG_DIAGNOSTIC_STAT1      0x1A

#define DRIVER_NAME "serdes"

#define __DEBUG__	0

struct	ti949 *global_da = NULL;

struct register_write {
    u8 reg;
    u8 val;
};

/* TI949 Serializer */
static const struct register_write ti949_init_settings[] = {
    {SER_SLAVE_ID_REG, MXT640_SLAVE_DEVICE_ID},
    {SER_SLAVE_ALIAS_REG, MXT640_SLAVE_DEVICE_ID},
    {SER_SLAVE2_ID_REG, NFC_SLAVE_DEVICE_ID},
    {SER_SLAVE2_ALIAS_REG, NFC_SLAVE_DEVICE_ID},
    {SER_CONFIG0_REG, LVDS_BYPASS},
    {SER_GPIO0_REG, SER_GPIO0_VALUE},
    {SER_GPIO_1_2_REG, SER_GPIO_1_2_VALUE},
    {SER_REG_SLAVE_ID3, FACEPLATE_SLAVE_ID},
    {SER_REG_SLAVE_ALIAS, FACEPLATE_SLAVE_ID},
};

/* TI948 Deserializer */
static const struct register_write ti948_init_settings[] = {
    {DESER_GPIO0_REG, DESER_GPIO0_VALUE},
    {DESER_GPIO_1_2_REG, DESER_GPIO_1_2_VALUE},
    {DESER_GPIO_7_8_REG, DESER_GPIO_7_8_VALUE},
    {DESER_CONFIG_SCL_HIGHTIME_REG, DESER_CONFIG_SCL_HIGHTIME_VALUE},
    {DESER_CONFIG_SCL_LOWTIME_REG, DESER_CONFIG_SCL_LOWTIME_VALUE},
};

static struct regmap_config reg_config8 = {
    .reg_bits = 8,
    .val_bits = 8,
};

static int serdes_open(struct inode *i, struct file *f);
static int serdes_close(struct inode *i, struct file *f);
static long serdes_ioctl(struct file *fp, unsigned int cmd, unsigned long arg);

static struct file_operations serdes_fops =
{
    .owner          = THIS_MODULE,
    .open           = serdes_open,
    .release        = serdes_close,
    .unlocked_ioctl = serdes_ioctl,
};


DEFINE_MUTEX(serdes_mutex);

#if 0
static int ti948_reg_read(struct ti949 *da, u8 reg, u32 *val)
{
	int rval, retry;
	int timeout = 20;
	struct i2c_client *client = da->dev_priv;

	client->addr = da->pdata->ti948_i2c_addr;
	for (retry = 0; retry < timeout; retry++) {
		rval = regmap_read(da->ti948_regmap8, reg, val);
		if (rval)
			msleep(20);
		else
			break;
	}

	if (rval)
		dev_err(&client->dev, "%s: TI948 reading 0x%x failed, retried: %d \n",
				 __func__, reg, retry);

	return rval;
}
#endif

static int ti948_reg_write(struct ti949 *da, u8 reg, u32 val)
{
	int retry, rval;
	int timeout = 50;
	struct i2c_client *client = da->dev_priv;

	client->addr = da->pdata->ti948_i2c_addr;
	for (retry = 0; retry < timeout; retry++) {
		rval = regmap_write(da->ti948_regmap8, reg, val);
		if (rval)
			msleep(20);
		else
			break;
	}
	if (rval)
		dev_err(&client->dev,
			"%s: TI948 writing 0x%x failed rval: %d retried: %d \n",
				 __func__, reg, rval, retry);
#if (__DEBUG__)
	else {
		rval = ti948_reg_read(da, reg, &read_val);
		if (rval)
			dev_err(&client->dev, "%s: TI948 reading 0x%x failed\n",
				__func__, reg);
		else
			dev_err(&client->dev,
				"%s: TI948 read back reg 0x%x read val: 0x%x, write val: 0x%x \n",
					__func__, reg, read_val, val);
	}
#endif

	return rval;
}

static int ti949_reg_read(struct ti949 *da, u8 reg, u32 *val)
{
	int rval, retry;
	int timeout = 20;
	struct i2c_client *client = da->dev_priv;

	client->addr = da->pdata->ti949_i2c_addr;
	for (retry = 0; retry < timeout; retry++) {
		rval = regmap_read(da->ti949_regmap8, reg, val);
		if (rval)
			msleep(10);
		else
			break;
	}

	if (rval)
		dev_err(&client->dev,
			"%s: TI949 reading 0x%x failed, rval: %d retried: %d \n",
				 __func__, reg, rval, retry);

	return rval;
}

static int ti949_reg_write(struct ti949 *da, u8 reg, u8 val)
{
	int retry, rval;
	int timeout = 20;
	struct i2c_client *client = da->dev_priv;

	client->addr = da->pdata->ti949_i2c_addr;
	for (retry = 0; retry < timeout; retry++) {
		rval = regmap_write(da->ti949_regmap8, reg, val);
		if (rval)
			msleep(10);
		else
			break;
	}
	if (rval)
		dev_err(&client->dev,
			"%s: TI949 writing 0x%x failed, rval: %d retried: %d \n",
				 __func__, reg, rval, retry);
#if (__DEBUG__)
	else {
		rval = ti949_reg_read(da, reg, &read_val);
		if (rval)
			dev_err(&client->dev,
				"%s: TI949 reading 0x%x failed\n",
					__func__, reg);
		else
			dev_err(&client->dev,
				"%s: TI949 read back reg 0x%x read val: 0x%x, write val: 0x%x \n",
					__func__, reg, read_val, val);
	}
#endif

	return rval;
}

static int ti949_init(struct ti949 *da)
{
    int i, rval;

    for (i = 0; i < ARRAY_SIZE(ti949_init_settings); i++) {
		rval = ti949_reg_write(da, ti949_init_settings[i].reg,
								ti949_init_settings[i].val);

		if (rval)
			break;
    }

    return rval;
}

static int ti948_init(struct ti949 *da)
{
    int i, rval;

	for (i = 0; i < ARRAY_SIZE(ti948_init_settings); i++) {
		rval = ti948_reg_write(da, ti948_init_settings[i].reg,
                            ti948_init_settings[i].val);
	    if (rval)
				break;
	}

	return rval;
}

static int faceplate_init(struct ti949 *da)
{
    int rval;
    unsigned int val;

    struct i2c_client *client = da->dev_priv;

    client->addr = da->pdata->faceplate_i2c_addr;

    rval = regmap_read(da->faceplate, FACEPLATE_REG_DIAGNOSTIC_STAT1, &val);

    if (rval) {
        dev_err(&client->dev, "%s: faceplate reading 0x1a failed\n", __func__);
    }

    return rval;
}

static int ti949_pdb_reset(struct ti949 *da)
{
	struct i2c_client *client = da->dev_priv;

	dev_dbg(&client->dev, "%s: Entered\n", __func__);
	gpio_set_value(da->pdata->pdb_gpio, 0);
	msleep(10);
	gpio_set_value(da->pdata->pdb_gpio, 1);
	msleep(100);
	dev_dbg(&client->dev, "%s: Exited\n", __func__);

    return 0;
}

static int ti949_re_enable_intb(struct i2c_client *client)
{
	int rval;
	struct ti949 *da = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s: Entered \n", __func__);

	/* Clear Checksum errors..*/
	rval = ti949_reg_write(da, REG_TI949_CHECKSUM, CHECKSUM_CLEAR_VAL);
	if (rval) {
		dev_err(&client->dev, "%s: failed to write 0x%x \n",
			__func__, REG_TI949_CHECKSUM);
		return rval;
	}

	/* Enable RX Detect select, the interrupt will be
	 * asserted on rising edge of either FPD link detect or lock pin */
	rval = ti949_reg_write(da, REG_TI949_ICR_1, 0xAA);
	if (rval) {
		dev_err(&client->dev, "%s: failed to write 0x%x \n",
			__func__, REG_TI949_ICR_1);
		return rval;
	}

	/* Enable Interrupt on Receiver Detect */
	rval = ti949_reg_write(da, REG_TI949_ICR, 0x41);
	if (rval) {
		dev_err(&client->dev,
			"%s: failed to write 0x%x (interrupt config) \n",
				__func__, REG_TI949_ICR);
		return rval;
	}

	dev_dbg(&client->dev, "%s: Exit \n", __func__);

	return rval;
}

static int ti949_enable_intb(struct i2c_client *client)
{
	int rval;
	struct ti949 *da = i2c_get_clientdata(client);
	unsigned int val;

	dev_dbg(&client->dev, "%s: Enabled INTB interrupt \n", __func__);

	/* Clear Checksum errors..*/
	rval = ti949_reg_write(da, REG_TI949_CHECKSUM, CHECKSUM_CLEAR_VAL);
	if (rval) {
		dev_err(&client->dev, "%s: failed to write 0x%x \n",
				__func__, REG_TI949_CHECKSUM);
		return rval;
	}

	/* Read ISR to clear the interrupt */
	rval = ti949_reg_read(da, REG_TI949_ISR, &val);
	if (rval)
		dev_err(&client->dev, "%s: Read ISR reg error try 1\n", __func__);
	else
		dev_dbg(&client->dev,
			"%s: Read ISR reg value: 0x%x try 2\n", __func__, val);

	/* Enable RX Detect select, the interrupt will be
	 * asserted on rising edge of either FPD link detect or lock pin */
	rval = ti949_reg_write(da, REG_TI949_ICR_1, 0xAA);
	if (rval) {
		dev_err(&client->dev, "%s: failed to write 0x%x \n",
				__func__, REG_TI949_ICR_1);
		return rval;
	}

	/* Enable Interrupt on Receiver Detect */
	rval = ti949_reg_write(da, REG_TI949_ICR, 0x41);
	if (rval) {
		dev_err(&client->dev,
			"%s: failed to write 0x%x (interrupt config) \n",
				__func__, REG_TI949_ICR);
		return rval;
	}

	return rval;
}

static int cfg_touch_gpio0(struct ti949 *da, bool enable)
{
	if (!enable) {
		ti949_reg_write(da, SER_GPIO0_REG, 0x09);
		ti948_reg_write(da, DESER_GPIO0_REG, 0x29);
	} else {
		ti949_reg_write(da, SER_GPIO0_REG, SER_GPIO0_VALUE);
		ti948_reg_write(da, DESER_GPIO0_REG, DESER_GPIO0_VALUE);
	}
	return 0;
}

static int reinit_ti949_948(struct i2c_client *client)
{
	struct ti949 *da = i2c_get_clientdata(client);
	int rval = -1;
	dev_dbg(&client->dev, "%s: Entered\n", __func__);

    mutex_lock(&serdes_mutex);

	/* do the reset for ti949 */
	ti949_pdb_reset(da);

	if (!da)
		dev_err(&client->dev, "%s: da is null \n", __func__);

	rval = ti949_init(da);
	if (rval) {
		dev_err(&client->dev, "Failed to re-init TI949!\n");
		return rval;
	}

	/* Disable touch int GPIO0 (in local mode) */
	cfg_touch_gpio0(da, 0);
	msleep(50);
	rval = ti948_init(da);
	if (rval) {
		dev_err(&client->dev, "Failed to re-init TI948!\n");
		return rval;
	}
	/* Reconfigure TI949 INTB */
	rval = ti949_re_enable_intb(client);
	/* Re-enable touch int GPIO0 (in remote mode) */
	cfg_touch_gpio0(da, 1);
	dev_dbg(&client->dev, "%s: Exit\n", __func__);

    mutex_unlock(&serdes_mutex);

	return rval;
}



irqreturn_t ti949_threaded_irq_fn(int irq, void *data)
{
	struct i2c_client *client = data;
    unsigned int link_loss, isr_val, isr_c7, reg_val;
	int rval;
	struct ti949 *da = i2c_get_clientdata(client);
	bool do_reinit =  false;

	dev_dbg(&client->dev, "%s: Entered\n", __func__);

	if (!da)
		dev_err(&client->dev, "%s: da is NULL\n", __func__);

	if (!client)
		dev_err(&client->dev, "%s: client is NULL\n", __func__);

	/* Get ISR */
	rval = ti949_reg_read(da, REG_TI949_ISR, &isr_val);
	if (rval) {
		dev_err(&client->dev, "%s: Read ISR reg error\n", __func__);
		goto out;
	} else {
		dev_dbg(&client->dev, "%s: isr value at 0x%x: 0x%x\n",
			__func__, REG_TI949_ISR, isr_val);
	}

	isr_c7 = isr_val;

	/* Get link loss status */
	rval = ti949_reg_read(da, REG_TI949_GEN_STS, &link_loss);
	if (rval) {
		dev_err(&client->dev, "%s: Read link status reg error\n", __func__);
		goto out;
	} else {
		dev_dbg(&client->dev, "%s: value at 0x%x: 0x%x\n",
					__func__, REG_TI949_GEN_STS, link_loss);
	}

	/* Check for link detect after a link loss */
	if ((link_loss & REG_TI949_LINK_LOSS_BIT) &&
			(link_loss & REG_TI949_LINK_DET_BIT)) {
		/* Clear CRC error counter */
		rval = ti949_reg_write(da, REG_TI949_MODE_SEL, CRC_ERROR_RESET_CLEAR);
		if (rval)
			dev_err(&client->dev,
			"%s: failed to write 0x%x \n", __func__, REG_TI949_MODE_SEL);

		/* Set for nromal operation */
		rval = ti949_reg_write(da, REG_TI949_MODE_SEL, CRC_ERROR_RESET_NORMAL);
		if (rval)
			dev_err(&client->dev,
			"%s: failed to write 0x%x\n", __func__, REG_TI949_MODE_SEL);

		dev_err(&client->dev, "%s: setting do_reinit true: case 1\n", __func__);
		do_reinit = true;
	}


	/* Ensure interrupt only for lock pin assertion */
	if(isr_c7 && !(link_loss & REG_TI949_LINK_LOSS_BIT)) {
		rval = ti949_reg_read(da, REG_TI949_CHECKSUM, &reg_val);
		if (rval)
			dev_err(&client->dev, "%s: Read reg 0x%x error\n",
				__func__, REG_TI949_CHECKSUM);
		else
			dev_dbg(&client->dev, "%s: value at 0x%x: 0x%x\n",
				__func__, REG_TI949_CHECKSUM, reg_val);
		rval = ti949_reg_write(da, REG_TI949_CHECKSUM, CHECKSUM_CLEAR_VAL);
		if (rval)
			dev_err(&client->dev, "%s: failed to write 0x%x \n",
				__func__, REG_TI949_CHECKSUM);

		dev_err(&client->dev, "%s: lock pin assertion case 2\n", __func__);
	}

out:
	/* Reinitialize on link detect after a link loss */
	if (do_reinit) {
		dev_err(&client->dev, "%s: calling reinit here\n", __func__);
		rval = reinit_ti949_948(client);
		if (rval)
			dev_err(&client->dev, "%s: Reinit failed\n", __func__);
		else
			dev_err(&client->dev, "%s: Reinit success\n", __func__);
	}

	return IRQ_HANDLED;
}

static int ti949_config_intb_irq(struct i2c_client *client)
{
	int rval;
	struct ti949 *da = i2c_get_clientdata(client);
	int irq_intb;

	irq_intb = da->pdata->irq_intb;
	dev_dbg(&client->dev, "%s: Entered\n", __func__);

	rval = ti949_enable_intb(client);
	if (rval)
		return rval;

	if (!gpio_is_valid(irq_intb)) {
		dev_err(&client->dev, "%s: GPIO pin %d is invalid!\n",
											__func__, irq_intb);
		return -ENODEV;
	}

	dev_err(&client->dev,
		"%s: IRQ GPIO %d is valid.\n", __func__, irq_intb);

	rval = devm_gpio_request(&client->dev, irq_intb, da->pdata->irq_intb_name);
	if (rval) {
		dev_err(&client->dev,
			"%s:IRQ GPIO pin request failed!\n", __func__);
		return rval;
	}

	gpio_direction_input(irq_intb);
	da->irq = gpio_to_irq(irq_intb);
	rval = devm_request_threaded_irq(&client->dev, da->irq,
					NULL, ti949_threaded_irq_fn,
					(IRQF_TRIGGER_FALLING | IRQF_ONESHOT),
					da->pdata->irq_intb_name,
					client);
	if (rval)
		dev_err(&client->dev, "%s: request_threaded_irq failed: rval: %d\n",
				__func__, rval);
	else
		dev_err(&client->dev, "%s: request_threaded_irq success: rval: %d\n",
				__func__, rval);

	dev_dbg(&client->dev, "%s: Exit\n", __func__);
	return rval;
}

static int serdes_open(struct inode *i, struct file *f)
{
    printk(KERN_INFO "%s\n", __func__);
    return 0;
}
static int serdes_close(struct inode *i, struct file *f)
{
    printk(KERN_INFO "%s\n", __func__);
    return 0;
}

static long serdes_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int rval = -1;
    struct i2c_client *client;

    client = global_da->dev_priv;

    switch (cmd)
    {
        case RECONFIG_SERDES:

            rval = reinit_ti949_948(client);
            if (rval)
                dev_err(&client->dev, "%s: Reinit failed\n", __func__);
            else
                dev_err(&client->dev, "%s: Reinit success\n", __func__);
            break;

        default:
            dev_err(&client->dev, "%s Invalid IOCTL\n", __FUNCTION__);
            return -EINVAL;
    }

    return 0;
}

static int ti949_cdev_setup(struct ti949 *data)
{
    int    ret;
    struct device *dev_ret;

    ret = alloc_chrdev_region (&data->major_num, 0, 1, DRIVER_NAME);

    if (ret < 0) {
        printk(KERN_ERR "%s failed to reserve major/minor range\n", __FUNCTION__);
        return ret;
    }

    if (!(data->serdes_cdev = cdev_alloc())) {
        printk(KERN_ERR "%s cdev_alloc() failed\n", __FUNCTION__);
        unregister_chrdev_region (data->major_num, 1);
        return -ENOMEM;
     }

    cdev_init(data->serdes_cdev, &serdes_fops);

    ret = cdev_add(data->serdes_cdev, data->major_num, 1);
    if( ret < 0 ) {
        printk(KERN_ERR "%s cdev_add failed\n", __FUNCTION__);
        unregister_chrdev_region (data->major_num, 1);
        return ret;
    }

    if (IS_ERR(data->serdes_class = class_create(THIS_MODULE, DRIVER_NAME)))
    {
        printk(KERN_ERR "%s Error class_create\n", __FUNCTION__);
        cdev_del (data->serdes_cdev);
        unregister_chrdev_region (data->major_num, 1);
        return -EBUSY;
    }

    if (IS_ERR(dev_ret = device_create(data->serdes_class, NULL, data->major_num, NULL, DRIVER_NAME)))
    {
        printk(KERN_ERR "%s Error device_create\n", __FUNCTION__);
        class_destroy(data->serdes_class);
        cdev_del(data->serdes_cdev);
        unregister_chrdev_region(data->major_num, 1);
        return -EBUSY;
    }

    return 0;
}


static int ti949_remove(struct i2c_client *client)
{
    struct ti949 *da = i2c_get_clientdata(client);

    device_destroy(da->serdes_class, da->major_num);
    class_destroy(da->serdes_class);
    cdev_del(da->serdes_cdev);
    unregister_chrdev_region(da->major_num, 1);

    devm_kfree(&client->dev, da);
    return 0;
}

static int ti949_probe(struct i2c_client *client,
                        const struct i2c_device_id *devid)
{
    struct ti949 *da;
    int rval = 0;
    int error;

    if (client->dev.platform_data == NULL) {
        dev_err(&client->dev, "%s: No platform Data \n", __func__);
        return -ENODEV;
    }

    da = devm_kzalloc(&client->dev, sizeof(*da), GFP_KERNEL);
    if (!da)
        return -ENOMEM;

    da->pdata = client->dev.platform_data;
    i2c_set_clientdata(client, da);
    da->dev_priv = client;
    global_da = da;


	/* Reset TI949 */
	if (devm_gpio_request_one(&client->dev, da->pdata->pdb_gpio, 0,
												"TI949_PDB") != 0) {
		dev_err(&client->dev, "unable to acquire: TI949 PDB %d\n",
											da->pdata->pdb_gpio);
		return -ENODEV;
	}
	gpio_direction_output(da->pdata->pdb_gpio, 1);
	ti949_pdb_reset(da);

    client->addr = da->pdata->ti949_i2c_addr;
    da->ti949_regmap8 = devm_regmap_init_i2c(client, &reg_config8);
    if (IS_ERR(da->ti949_regmap8)) {
        dev_err(&client->dev, "Failed to init ti949 regmap8!\n");
        return -EIO;
    }

    rval = ti949_init(da);
    if (rval) {
        dev_err(&client->dev, "Failed to init TI949!\n");
        return rval;
    }

	msleep(10);

    client->addr = da->pdata->ti948_i2c_addr;
    da->ti948_regmap8 = devm_regmap_init_i2c(client, &reg_config8);
    if (IS_ERR(da->ti948_regmap8)) {
        dev_err(&client->dev, "Failed to init ti948 regmap8!\n");
        return -EIO;
    }

    rval = ti948_init(da);
    if (rval) {
        dev_err(&client->dev, "Failed to init TI948!\n");
        return rval;
    }

    client->addr = da->pdata->faceplate_i2c_addr;
    da->faceplate = devm_regmap_init_i2c(client, &reg_config8);
    if (IS_ERR(da->faceplate)) {
        dev_err(&client->dev, "Failed to init faceplate regmap8!\n");
        return -EIO;
    }

	error = ti949_cdev_setup(da);

    if (error) {
        dev_err(&client->dev, "Error %d registering serdes device\n", error);
        return -ENODEV;
    }

    /*lvds Hal will monitor faceplate reg 0x1a periodically                   */
    /* and if bit 3 or 4 set, it will call IOCTL to reconfigure serializer and*/
    /* deserializer. At power-up time this driver will configure  serializer  */
    /*and deserializer. If this driver dont read register 0x1a,LVDS hal also  */
    /*initiate a reconfigure at booting, which is not necessary               */
    faceplate_init(da);

	rval = ti949_config_intb_irq(client);

	return rval;
}

static const struct i2c_device_id ti949_id_table[] = {
    { TI949_NAME, 0 },
    {}
};

MODULE_DEVICE_TABLE(i2c, ti949_id_table);

static struct i2c_driver ti949_i2c_driver = {
    .driver = {
        .name = TI949_NAME,
    },
    .probe    = ti949_probe,
    .remove   = ti949_remove,
    .id_table = ti949_id_table,
};
module_i2c_driver(ti949_i2c_driver);

/* Module information */
MODULE_AUTHOR("Sreeju Selvaraj <Sreeju.Selvaraj@harman.com>");
MODULE_DESCRIPTION("TI949-TI948 display serdes driver");
MODULE_LICENSE("GPL");
