/*
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/input/faceplate.h>


#define FACEPLATE_WAKEUP_TIME         25    /* msec */
/* Registers */
#define ENCODER_A_REG_ADDR            0x10
#define ENCODER_B_REG_ADDR            0x11
#define DEBOUNCED_INPUT_STATUS        0x13
#define PROXIMITY_REG_ADDR            0x16
#define DIAGNOSTIC_REG_ADDR           0x1A
#define PROX_BUF_SIZE                    4
#define BIT_POWER_BUTTON              0x01

#define DRIVER_NAME "faceplate"


static int faceplate_keymap[] = {
    KEY_PREVIOUSSONG,
    KEY_NEXTSONG,
    KEY_MENU,
    KEY_HOMEPAGE,
    KEY_BACK
};


/* Each client has this additional data */
struct faceplate_data {
    struct i2c_client *client;
    struct input_dev *input_dev;
    unsigned int irq;
    unsigned int gpio_irq;
    int num_keys;
    const unsigned int *keymap;
    dev_t major_num;
    struct cdev *faceplate_cdev;
    struct class *faceplate_class;
    char phys[64];
    bool faceplate_disabled;
};

union key_val {
    unsigned int  btn_all;
    unsigned char btn[4];
};

static int faceplate_open(struct inode *i, struct file *f);
static int faceplate_close(struct inode *i, struct file *f);
static long faceplate_ioctl(struct file *fp, unsigned int cmd, unsigned long arg);
static int faceplate_cdev_setup(struct faceplate_data *data);

static int  faceplate_i2c_open(struct input_dev *dev);
static void faceplate_i2c_close(struct input_dev *dev);
static int __faceplate_read_reg(struct i2c_client *client, u8 reg, u8 len, void *val);
void faceplate_initialise(struct faceplate_data *data);

static struct file_operations faceplate_fops =
{
    .owner          = THIS_MODULE,
    .open           = faceplate_open,
    .release        = faceplate_close,
    .unlocked_ioctl = faceplate_ioctl,
};

DECLARE_WAIT_QUEUE_HEAD(power_button_wq);

static union key_val prev_val;
static unsigned char power_button_val;
static char power_btn_flag;
static unsigned int button_enable_state = BIT_ROTARY_VOLUME | BIT_TUNER | BIT_POWER | BIT_FACEPLATE_KEYS;
static struct i2c_client *faceplate_i2c_client;

static ssize_t faceplate_disable_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct faceplate_data *data = dev_get_drvdata(dev);
    char c;

    c = data->faceplate_disabled ? '1' : '0';
    return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}


static ssize_t faceplate_disable_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct faceplate_data *data = dev_get_drvdata(dev);
    u8 i;

    if (kstrtou8(buf, 0, &i) == 0) {
        i = !!i;
        if(data->faceplate_disabled != i) {
            data->faceplate_disabled = i;
            if (data->faceplate_disabled) {
                disable_irq(data->irq);
                dev_info(dev, "Faceplate Disbled\n");
            } else {
                faceplate_initialise(data);
                enable_irq(data->irq);
                dev_info(dev, "Faceplate Enabled\n");
            }

        } else {
            dev_info(dev, "%s already. Nothing to be Done\n", i ? "Faceplate Disbled" : "Faceplate Enabled");
        }

    } else {
        dev_err(dev, "faceplate_disabled write error\n");
        return -EINVAL;
    }


    return count;
}

/* /sys/bus/i2c/drivers/faceplate_control/2-0012 */
static DEVICE_ATTR(faceplate_disable, S_IWUSR | S_IRUGO, faceplate_disable_show, faceplate_disable_store);

static struct attribute *faceplate_attrs[] = {
    &dev_attr_faceplate_disable.attr,
    NULL
};

static const struct attribute_group faceplate_attr_group = {
    .attrs = faceplate_attrs,
};
static int faceplate_open(struct inode *i, struct file *f)
{
    printk(KERN_INFO "%s\n", __func__);
    return 0;
}
static int faceplate_close(struct inode *i, struct file *f)
{
    printk(KERN_INFO "%s\n", __func__);
    return 0;
}

static long faceplate_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    unsigned int button_state;
    int ret;
    int8_t power_button = 0;


    switch (cmd)
    {
        case GET_POWER_BUTTON_VALUE:
            ret = __faceplate_read_reg(faceplate_i2c_client, DEBOUNCED_INPUT_STATUS,
                                        sizeof(power_button), &power_button);
            if (ret) {
                printk(KERN_ERR "%s power button i2c read failed\n", __FUNCTION__);
                return -EFAULT;
            }

            power_button = power_button & 0x01; /*Bit 0: Power Button*/

            if (copy_to_user((unsigned char *)arg, &power_button, sizeof(power_button)) != 0) {
                printk(KERN_ERR "%s GET_POWER_BUTTON_VALUE failed\n", __FUNCTION__);
                return -EFAULT;
            }

            break;

        case WAIT_FOR_POWER_KEY_EVENT:
            wait_event_interruptible(power_button_wq, power_btn_flag != 0);
            if (copy_to_user((unsigned char *)arg, &power_button_val, sizeof(power_button_val)) != 0) {
                power_btn_flag = 0;
                printk(KERN_ERR "%s WAIT_FOR_POWER_KEY_EVENT failed\n", __FUNCTION__);
                return -EFAULT;
            }
            power_btn_flag = 0;
            break;

        case DISABLE_FACEPLATE_FUNCTION:
            if (copy_from_user(&button_state,  (unsigned int *) arg, sizeof(unsigned int)) != 0) {
                printk(KERN_ERR "%s DISABLE_FACEPLATE_FUNCTION failed\n", __FUNCTION__);
                return -EFAULT;
            }

            button_enable_state = button_enable_state & ~button_state;
            break;

        case ENABLE_FACEPLATE_FUNCTION:
            if (copy_from_user(&button_state,  (unsigned int *) arg, sizeof(unsigned int)) != 0) {
                printk(KERN_ERR "%s ENABLE_FACEPLATE_FUNCTION failed\n", __FUNCTION__);
                return -EFAULT;
            }

            /* Either rotary volume or slider will be present at a time                       */
            /* Below condition will make sure that when we enable one other will get disabled */
            if (button_state & (BIT_ROTARY_VOLUME | BIT_SLIDER)) {
                button_enable_state = button_enable_state & ~(BIT_ROTARY_VOLUME | BIT_SLIDER);
            }

            button_enable_state = button_enable_state | button_state;
            break;

        default:
            printk(KERN_ERR "%s Invalid IOCTL\n", __FUNCTION__);
            return -EINVAL;
    }

    return 0;
}


static int __faceplate_read_reg(struct i2c_client *client,
        u8 reg, u8 len, void *val)
{
    struct i2c_msg xfer[2];
    u8 buf[1];
    int ret;
    bool retry = false;

    buf[0] = reg & 0xff;
    /*    buf[1] = (reg >> 8) & 0xff; TODO: Recheck it */

    /* Write register */
    xfer[0].addr = client->addr;
    xfer[0].flags = 0;
    xfer[0].len = 1;
    xfer[0].buf = buf;

    /* Read data */
    xfer[1].addr = client->addr;
    xfer[1].flags = I2C_M_RD;
    xfer[1].len = len;
    xfer[1].buf = val;

retry_read:
    ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
    if (ret != ARRAY_SIZE(xfer)) {
        if (!retry) {
            dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
            msleep(FACEPLATE_WAKEUP_TIME);
            retry = true;
            goto retry_read;
        } else {
            dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
                    __func__, ret);
            return -EIO;
        }
    }

    return 0;
}
#if 0
static int __faceplate_write_reg(struct i2c_client *client, u16 reg, u16 len,
        const void *val)
{
    u8    *buf;
    size_t count;
    int    ret;
    bool   retry = false;

    count  = len + 2;
    buf    = kmalloc(count, GFP_KERNEL);

    if (!buf)
        return -ENOMEM;

    buf[0] = reg & 0xff;
    buf[1] = (reg >> 8) & 0xff;
    memcpy(&buf[2], val, len);

retry_write:
    ret = i2c_master_send(client, buf, count);
    if (ret != count) {
        if (!retry) {
            dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
            msleep(FACEPLATE_WAKEUP_TIME);
            retry = true;
            goto retry_write;
        } else {
            dev_err(&client->dev, "%s: i2c send failed (%d)\n",
                    __func__, ret);
            ret = -EIO;
        }
    } else {
        ret = 0;
    }

    kfree(buf);
    return ret;
}
#endif

/*
static int faceplate_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
    return __faceplate_write_reg(client, reg, 1, &val);
}
*/

static void report_key_press(struct input_dev *input, unsigned keycode)
{
    input_report_key(input, keycode, 1);
    input_sync(input);
}

static void report_key_release(struct input_dev *input, unsigned keycode)
{
    input_report_key(input, keycode, 0);
    input_sync(input);
}


static void faceplate_proc_keys(struct faceplate_data *data, int8_t *data_buf)
{
    struct input_dev *input_dev = data->input_dev;
    int key;
    union key_val cur_val;

    cur_val.btn[0] = data_buf[0];
    cur_val.btn[1] = data_buf[1];
    cur_val.btn[2] = data_buf[2];
    cur_val.btn[3] = data_buf[3];

    /*2 interrupts are coming for any button press*/
    /*below logic will avoid sending two input events to user space for a single button press*/
    if (cur_val.btn_all != prev_val.btn_all) {
        /*Tuner Knob */
        if ((data_buf[0] != 0) && (button_enable_state & BIT_TUNER)) {
            input_event(input_dev, EV_KEY, KEY_TUNER, data_buf[0]);
            input_sync(input_dev);
            input_report_key(input_dev, KEY_TUNER, 0);
            input_sync(input_dev);
        }

            /* Volume Knob */
        if (data_buf[1] != 0) {
            if (button_enable_state & BIT_SLIDER) {
                pr_debug("faceplate reg 0x10 (slider) value 0x%x\n", data_buf[1]);
                if (data_buf[1] & 0x80) {
                    input_event(input_dev, EV_KEY, KEY_SOUND, (data_buf[1] & 0x3F));
                    input_sync(input_dev);
                    input_report_key(input_dev, KEY_SOUND, 0);
                    input_sync(input_dev);
                }
            } else if(button_enable_state & BIT_ROTARY_VOLUME){
                if(data_buf[1] > 0) {
                    input_event(input_dev, EV_KEY, KEY_VOLUMEUP, data_buf[1]);
                    input_sync(input_dev);
                    input_report_key(input_dev, KEY_VOLUMEUP, 0);
                    input_sync(input_dev);
                } else {
                    input_event(input_dev, EV_KEY, KEY_VOLUMEDOWN, -data_buf[1]);
                    input_sync(input_dev);
                    input_report_key(input_dev, KEY_VOLUMEDOWN, 0);
                    input_sync(input_dev);
                }
            } else {
                printk(KERN_ERR "Neither slider nor rotary volume control is enabled\n");
            }
        }


        if (button_enable_state & BIT_FACEPLATE_KEYS) {
            /* Faceplate Keys */
            if ( prev_val.btn[2] != cur_val.btn[2] ) {
                /*Checking bit position of individual keys*/
                for (key = 0; key < data->num_keys; key++) {
                    /*if previous button value is 0 and current value is 1 => button is pressed*/
                    if (((1u << key) & cur_val.btn[2]) && (((1u << key) & prev_val.btn[2]) == 0)) {
                        report_key_press(input_dev, data->keymap[key]);
                    }

                    /*if previous button value is 1 and current value is 0 => button is released*/
                    if ((((1u << key) & cur_val.btn[2]) == 0) && ((1u << key) & prev_val.btn[2])) {
                        report_key_release(input_dev, data->keymap[key]);
                    }
                }
            }
        }

        /* Power Key */
        if (button_enable_state & BIT_POWER) {
            if (prev_val.btn[3] != cur_val.btn[3]) {
                if ((cur_val.btn[3] & BIT_POWER_BUTTON) && ((prev_val.btn[3] & BIT_POWER_BUTTON) == 0)) {
                    report_key_press( input_dev, KEY_POWER);
                    power_button_val = 1;
                    power_btn_flag   = 1;
                    wake_up_interruptible(&power_button_wq);
                } else if (((cur_val.btn[3] & BIT_POWER_BUTTON) == 0) && (prev_val.btn[3] & BIT_POWER_BUTTON)) {
                    report_key_release( input_dev, KEY_POWER);
                    power_btn_flag   = 1;
                    power_button_val = 0;
                    wake_up_interruptible(&power_button_wq);
                }
            }
        }

        prev_val.btn_all = cur_val.btn_all;
    }

}

static irqreturn_t faceplate_interrupt(int irq, void *dev_id)
{
    struct faceplate_data *data = dev_id;
    struct device *dev = &data->client->dev;
    int ret;
    int8_t data_buf[4] = {0}, prox_buf1[4] = {0}, data_buf2[2] = {0};


    ret = __faceplate_read_reg(data->client, ENCODER_A_REG_ADDR,
            4, data_buf);
    if (ret) {
        dev_err(dev, "Failed to read Face plate key data \n");
        return IRQ_NONE;
    }

    ret = __faceplate_read_reg(data->client, PROXIMITY_REG_ADDR,
            4, prox_buf1);
    if (ret) {
        dev_err(dev, "Failed to read Face plate key data \n");
        return IRQ_NONE;
    }

    ret = __faceplate_read_reg(data->client, DIAGNOSTIC_REG_ADDR,
            2, data_buf2);
    if (ret) {
        dev_err(dev, "Failed to read Face plate key data \n");
        return IRQ_NONE;
    }

    pr_debug("faceplate buf %x %x %x %x\n", data_buf[0], data_buf[1], data_buf[2], data_buf[3]);

    faceplate_proc_keys(data, data_buf);

    return IRQ_HANDLED;
}



void faceplate_initialise(struct faceplate_data *data)
{
    struct device *dev = &data->client->dev;
    int ret;
    int8_t data_buf[4] = {0}, data_buf1[4] = {0}, data_buf2[2] = {0};


    ret = __faceplate_read_reg(data->client, ENCODER_A_REG_ADDR,
            4, data_buf);
    if (ret) {
        dev_err(dev, "Failed to read Face plate key data \n");
    }

    ret = __faceplate_read_reg(data->client, PROXIMITY_REG_ADDR,
            4, data_buf1);
    if (ret) {
        dev_err(dev, "Failed to read Face plate PROXIMITY_REG data \n");
    }

    ret = __faceplate_read_reg(data->client, DIAGNOSTIC_REG_ADDR,
            2, data_buf2);
    if (ret) {
        dev_err(dev, "Failed to read Face plate DIAGNOSTIC_REG data \n");
    }
}



static int faceplate_i2c_open(struct input_dev *dev)
{
    pr_debug("%s()\n", __FUNCTION__);
    return 0;
}

static void faceplate_i2c_close(struct input_dev *dev)
{
    pr_debug("%s()\n", __FUNCTION__);
}

static int faceplate_cdev_setup(struct faceplate_data *data)
{
    int    ret;
    struct device *dev_ret;

    ret = alloc_chrdev_region (&data->major_num, 0, 1, DRIVER_NAME);

    if (ret < 0) {
        printk (KERN_ERR "%s failed to reserve major/minor range\n", __FUNCTION__);
        return ret;
    }

    if (!(data->faceplate_cdev = cdev_alloc())) {
        printk (KERN_ERR "%s cdev_alloc() failed\n", __FUNCTION__);
        unregister_chrdev_region (data->major_num, 1);
        return -ENOMEM;
     }

    cdev_init(data->faceplate_cdev, &faceplate_fops);

    ret = cdev_add(data->faceplate_cdev, data->major_num, 1);
    if( ret < 0 ) {
        printk(KERN_ERR "%s cdev_add failed\n", __FUNCTION__);
        unregister_chrdev_region (data->major_num, 1);
        return ret;
    }

    if (IS_ERR(data->faceplate_class = class_create(THIS_MODULE, DRIVER_NAME)))
    {
        printk(KERN_ERR "%s Error class_create\n", __FUNCTION__);
        cdev_del (data->faceplate_cdev);
        unregister_chrdev_region (data->major_num, 1);
        return -EBUSY;
    }

    if (IS_ERR(dev_ret = device_create(data->faceplate_class, NULL, data->major_num, NULL, DRIVER_NAME)))
    {
        printk(KERN_ERR "%s Error device_create\n", __FUNCTION__);
        class_destroy(data->faceplate_class);
        cdev_del(data->faceplate_cdev);
        unregister_chrdev_region(data->major_num, 1);
        return -EBUSY;
    }

    return 0;
}

static int faceplate_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct faceplate_data *data;
    struct input_dev *input_dev;
    int error;
    int i;

    printk(KERN_INFO "Faceplate driver V2\n");

    faceplate_i2c_client = client;
    data = devm_kzalloc(&client->dev,sizeof(struct faceplate_data), GFP_KERNEL);

    if (!data) {
        dev_err(&client->dev, "Failed to allocate memory\n");
        error = -ENOMEM;
    }

    input_dev = input_allocate_device();

    if (!input_dev) {
        dev_err(&client->dev, "Failed to allocate memory\n");
        error = -ENOMEM;
        goto err_free_mem;
    }

    data->client     = client;
    data->input_dev  = input_dev;
    data->gpio_irq   = client->irq;
    data->keymap     = faceplate_keymap;
    data->num_keys   = sizeof(faceplate_keymap) / sizeof(int);

    if( data->gpio_irq > 0) {
        if(gpio_is_valid(data->gpio_irq)) {
            if(gpio_request_one((data->gpio_irq), GPIOF_DIR_IN,"faceplate_interrupt")){
                dev_err(&client->dev,"gpio request is failed");
                error = -1;
                goto err_free_input_dev;
            }

            data->irq = gpio_to_irq(data->gpio_irq);
            if(data->irq < 0){
                dev_err(&client->dev, "Mapping GPIO to IRQ failed\n");
                error = data->irq;
                goto err_free_gpio;
            }

        } else {
            dev_err(&client->dev, "Gpio request from dts is failed\n");
            error = -1;
            goto err_free_input_dev;
        }
    }

    input_dev->name           = "Faceplate Controller";
    snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
            client->adapter->nr, client->addr);

    input_dev->phys           = data->phys;
    input_dev->id.bustype     = BUS_I2C;
    input_dev->dev.parent     = &client->dev;
    input_dev->open           = faceplate_i2c_open;
    input_dev->close          = faceplate_i2c_close;

    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(KEY_VOLUMEUP, input_dev->keybit);
    __set_bit(KEY_VOLUMEDOWN, input_dev->keybit);
    __set_bit(KEY_POWER, input_dev->keybit);
    __set_bit(KEY_TUNER, input_dev->keybit);
    __set_bit(KEY_SOUND, input_dev->keybit);

    /* Faceplate key array */

    for (i = 0; i < data->num_keys; i++) {
        input_set_capability(input_dev, EV_KEY, data->keymap[i]);
    }

    input_set_drvdata(input_dev, data);
    i2c_set_clientdata(client, data);

    error = request_threaded_irq(data->irq,
                                 NULL,
                                 faceplate_interrupt,
                                 IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
                                 client->name,
                                 data);
    if (error) {
        dev_err(&client->dev, "Failed to register interrupt\n");
        goto err_free_gpio;
    }

    error = input_register_device(input_dev);
    if (error) {
        dev_err(&client->dev, "Error %d registering input device\n", error);
        goto err_free_irq;
    }

    /* Read all the Faceplate Registers on Bootup*/
    faceplate_initialise(data);

    error = sysfs_create_group(&client->dev.kobj, &faceplate_attr_group);
    if (error) {
    dev_err(&client->dev, "Failure %d creating sysfs group\n", error);
        goto err_faceplate_cdev_setup;
    }

    error = faceplate_cdev_setup(data);

    if (error) {
        dev_err(&client->dev, "Error %d registering input device\n", error);
        goto err_faceplate_cdev_setup;
    }

    return 0;

err_faceplate_cdev_setup:
    input_unregister_device(input_dev);

err_free_irq:
    free_irq(data->irq, data);

err_free_gpio:
    gpio_free(data->gpio_irq);

err_free_input_dev:
    input_free_device(input_dev);

err_free_mem:
    kfree(data);

    return error;
}

static int faceplate_remove(struct i2c_client *client)
{
    struct faceplate_data *data = i2c_get_clientdata(client);

    sysfs_remove_group(&client->dev.kobj, &faceplate_attr_group);
    //kfree(data); TODO: Not required devm_kzalloc
    device_destroy(data->faceplate_class, data->major_num);
    class_destroy(data->faceplate_class);
    cdev_del(data->faceplate_cdev);
    unregister_chrdev_region(data->major_num, 1);

    input_unregister_device(data->input_dev);
    free_irq(data->irq, data);
    gpio_free(data->gpio_irq);
    input_free_device(data->input_dev);

    return 0;
}

static const struct i2c_device_id faceplate_id[] = {
    { "faceplate_control", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, faceplate_id);

static struct i2c_driver faceplate_driver = {
    .driver = {
        .name           = "faceplate_control",
        .owner          = THIS_MODULE,
    },
    .probe              = faceplate_probe,
    .remove             = faceplate_remove,
    .id_table           = faceplate_id,
};

module_i2c_driver(faceplate_driver);

/* Module information */
MODULE_AUTHOR("Dipin Kumar <dipin.kumar@harman.com>");
MODULE_DESCRIPTION("Faceplate I2C control driver 2.0");
MODULE_LICENSE("GPL");
