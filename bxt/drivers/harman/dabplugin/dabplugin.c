/*
 *  dabplugin.c
 *  Copyright (C) 2016 Harman International Ltd,
 *  by Vikram N <vikram.narayanarao@harman.com>
 *  Created on: 12-Feb-2016
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License v2.0 as published by
 * the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "dabplugin: "fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/kfifo.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/sched/rt.h>
#include <linux/uaccess.h>

#define CREATE_TRACE_POINTS
#include <trace/events/dab_spi.h>

#define DYNAMIC_MINORS          32      /* ... up to 256 */
#define FIFO_SIZE               8192
#define BUF_SIZE                6000
#define DAB_MAX_PKT_SIZE        5472
#define DAB_NUM_BOOT_MSG        2
#define DAB_MSG_HEADER_LEN      4


static unsigned int dab_cs_gpios[4][4] = {
	[1][1] = 420,
	[1][0] = 5,
	[2][0] = -1,
	[3][0] = 429
};

#ifdef CONFIG_X86_64_GM_MY20_HW
static unsigned int dab_irq_gpios[4][4] = {
	[1][1] = 461,
	[1][0] = 90,
	[2][0] = -1,
	[3][0] = 460
};
#else
static unsigned int dab_irq_gpios[4][4] = {
	[1][0] = 90,
	[2][0] = -1,
	[3][0] = 443
};
#endif

static unsigned char dabbootmsg[DAB_NUM_BOOT_MSG + 1][DAB_MSG_HEADER_LEN] = {
	{ 0xAB, 0xBC, 0xCD, 0xDE },
	{ 0xE2, 0x00, 0x10, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00 }
};

static DECLARE_BITMAP(minors, DYNAMIC_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

struct dabspi_data {
	dev_t			devt;
	struct spi_device	*spi;
	struct list_head	device_entry;
	u8			*tx_buf;
	u8			*rx_buf;
	struct mutex		rlock, wlock, txrx_lck;
	struct kfifo		fifo;
	struct completion	data_avail;
	u32			speed_hz;
	u32			cs_gpio;
	u32			irq_gpio;
	u32			gpio_irq_num;
	u32			users;
	u32			txlen;
	s32			status;
	bool			user_write;
	u8			*bootmsg;
};

static int dab_major;

static void cs_assert(unsigned int gpio)
{
	gpio_set_value(gpio, 0);
}

static void cs_deassert(unsigned int gpio)
{
	gpio_set_value(gpio, 1);
}

static int spi_xchange(struct spi_device *spi, unsigned char *txbuf,
		       unsigned char *rxbuf, int len)
{
	struct spi_transfer t = {
		.tx_buf = txbuf,
		.rx_buf = rxbuf,
		.len	= len,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	trace_dab_spi_xchange(jiffies);
	return (!spi) ? -ESHUTDOWN : spi_sync(spi, &m);
}

static unsigned int is_valid_dabheader(unsigned char *dh, int len)
{
	return dh && (dh[0] == 0xAD) && (len >= DAB_MSG_HEADER_LEN)
	       && ((((dh[2] << 8) | dh[3]) + 2) <= DAB_MAX_PKT_SIZE);
}

static irqreturn_t send_spi_data(int irq, void *data)
{
	struct dabspi_data *dabspi = data;
	int res, pktsize, pktlen;
	unsigned char *pkt = NULL, *dabpkt = NULL;

	mutex_lock(&dabspi->txrx_lck);
	trace_dab_spi_work_start(jiffies);
	cs_assert(dabspi->cs_gpio);

	if (dabspi->user_write) {
		res = spi_xchange(dabspi->spi, dabspi->tx_buf, dabspi->rx_buf,
				  dabspi->txlen);
	} else {
		dabspi->txlen = DAB_MSG_HEADER_LEN;
		res = spi_xchange(dabspi->spi, dabspi->bootmsg, dabspi->rx_buf,
				  dabspi->txlen);
	}
	if (res) {
		pr_err("xchange failed[%d]\n", res);
		goto ret;
	}
	print_hex_dump_debug("DAB PLUGIN: RX: ", DUMP_PREFIX_OFFSET, 16, 1,
			     dabspi->rx_buf, dabspi->txlen, false);
	res = is_valid_dabheader(dabspi->rx_buf, DAB_MSG_HEADER_LEN);
	if (!res) {
		if (!dabspi->user_write)
			pr_err("Invalid Packet from DAB\n");
		goto ret;
	}
	pktlen = (dabspi->rx_buf[2] << 8) | dabspi->rx_buf[3];
	pktlen += 2; /* for checksum */
	pr_debug("RXD Len = %d DAB PKT LEN = %d\n", dabspi->txlen, pktlen);
	if (dabspi->txlen >= (pktlen + DAB_MSG_HEADER_LEN)) {
		pr_debug("Full Pkt Rxd\n");
		pkt = dabspi->rx_buf;
		pktsize = pktlen + DAB_MSG_HEADER_LEN;
	} else {
		pr_debug("Partial Pkt Rxd. Pending %d bytes\n",
			 pktlen - DAB_MSG_HEADER_LEN);
		dabpkt = kzalloc(pktlen + dabspi->txlen, GFP_KERNEL);
		if (!dabpkt) {
			pr_err("Mem Alloc Failed for Partial Pkt\n");
			goto ret;
		}
		memcpy(dabpkt, dabspi->rx_buf, dabspi->txlen);
		res = spi_xchange(dabspi->spi, dabpkt + dabspi->txlen,
				  dabspi->rx_buf,
				  pktlen -
				  (dabspi->txlen - DAB_MSG_HEADER_LEN));
		if (res) {
			pr_err("Error reading pending data\n");
			goto free_buf_n_ret;
		}
		print_hex_dump_debug("DAB PLUGIN: REM RX: ",
				     DUMP_PREFIX_OFFSET, 16, 1, dabspi->rx_buf,
				     pktlen -
				     (dabspi->txlen - DAB_MSG_HEADER_LEN),
				     false);
		memcpy(dabpkt + dabspi->txlen, dabspi->rx_buf, pktlen);
		pktsize = DAB_MSG_HEADER_LEN + pktlen;
		pkt = dabpkt;
	}
	print_hex_dump_debug("DAB PLUGIN: RX PKT: ", DUMP_PREFIX_OFFSET, 16,
			     1, pkt, pktsize, false);
	if (kfifo_is_full(&dabspi->fifo)) {
		pr_err("RXFIFO IS FULL DROPING DATA!!\n");
		goto free_buf_n_ret;
	}
	res = kfifo_avail(&dabspi->fifo);
	if (res < pktsize) {
		pr_err("RXFIFO: NO REQ FREE SPACE[A:%d - R:%d] DROPING DATA!\n",
		       res, pktsize);
		goto free_buf_n_ret;
	}
	res = kfifo_in(&dabspi->fifo, pkt, pktsize);
	if (res != pktsize) {
		pr_err("RXFIFO: ERR [pktsize = %d, data put into fifo = %d]\n",
		       pktsize, res);
		goto free_buf_n_ret;
	}
	complete(&dabspi->data_avail);

free_buf_n_ret:
	kfree(dabpkt);
ret:
	cs_deassert(dabspi->cs_gpio);
	trace_dab_spi_work_end(jiffies);
	mutex_unlock(&dabspi->txrx_lck);
	memset(dabspi->rx_buf, 0, BUF_SIZE);
	dabspi->status = res;
	return IRQ_HANDLED;
}

static int dab_open(struct inode *i, struct file *f)
{
	struct dabspi_data *dabspi;
	int status = -ENXIO;

	mutex_lock(&device_list_lock);
	list_for_each_entry(dabspi, &device_list, device_entry) {
		if (dabspi->devt == i->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status) {
		pr_debug("nothing for minor %d\n", iminor(i));
		goto err_find_dev;
	}
	if (dabspi->users != 0) {
		status = -EBUSY;
		goto err_find_dev;
	}
	dabspi->users++;
	f->private_data = dabspi;

err_find_dev:
	mutex_unlock(&device_list_lock);
	return status;
}

static ssize_t dab_read(struct file *f, char __user *buf, size_t count,
			loff_t *offset)
{
	static int callseq;
	struct dabspi_data *dabspi = f->private_data;
	ssize_t status = 0;
	unsigned int copied = 0;

	if (!(dabspi && dabspi->spi))
		return -ESHUTDOWN;
	if (count > BUF_SIZE)
		return -EMSGSIZE;

	mutex_lock(&dabspi->rlock);

	while (kfifo_is_empty(&dabspi->fifo)) {
		if (f->f_flags & O_NONBLOCK) {
			copied = -EAGAIN;
			goto ret;
		}
		pr_debug("[%d]:No Data available in Kfifo, going to wait\n",
			 callseq);
		status = wait_for_completion_interruptible(&dabspi->data_avail);
		if (status) {
			pr_err("Read was interrupted [%zd]\n", status);
			copied = status;
			goto ret;
		}
		if (!dabspi->spi) {
			copied = -ESHUTDOWN;
			goto ret;
		}
	}
	status = kfifo_to_user(&dabspi->fifo, buf, count, &copied);
	if (status) {
		pr_err("[%d]ERROR : Reading %zu data from kfifo\n", callseq,
		       count);
		copied = status;
		goto ret;
	}
ret:
	mutex_unlock(&dabspi->rlock);
	if (!copied) {
		pr_err("[%d]ERROR : Read Returning 0!!!\n", callseq);
		pr_err("[%d]->buf[%p] count[%zu], offset[%p], res[%zd], copied[%d]\n",
			callseq, buf, count, offset, status, copied);
		pr_err("[%d]->kfifo is %s\n", callseq,
		       kfifo_is_empty(&dabspi->fifo) ? "empty" : "not empty");
		pr_err("[%d]->kfifo_len = %d\n", callseq,
		       kfifo_len(&dabspi->fifo));
		pr_err("[%d]->kfifo.in = %d kfifo.out = %d kfifo.mask = %d kfifo.esize = %d kfifo.data = %p\n",
			callseq, dabspi->fifo.kfifo.in,
			dabspi->fifo.kfifo.out, dabspi->fifo.kfifo.mask,
			dabspi->fifo.kfifo.esize, dabspi->fifo.kfifo.data);
	}
	return copied;
}

static ssize_t dab_write(struct file *f, const char __user *buf, size_t count,
			 loff_t *offset)
{
	struct dabspi_data *dabspi = f->private_data;
	ssize_t status = 0;

	if (!(dabspi && dabspi->spi))
		return -ESHUTDOWN;
	if (count > BUF_SIZE)
		return -EMSGSIZE;

	mutex_lock(&dabspi->wlock);
	disable_irq(dabspi->gpio_irq_num);
	mutex_lock(&dabspi->txrx_lck);

	memset(dabspi->tx_buf, 0, count + 1);
	status = copy_from_user(dabspi->tx_buf, buf, count);
	if (status) {
		pr_err("Error in accessing user buffer to write\n");
		mutex_unlock(&dabspi->txrx_lck);
		goto unlock_n_ret;
	}
	print_hex_dump_debug("DAB PLUGIN : TX APP : ", DUMP_PREFIX_OFFSET,
			     16, 1, dabspi->tx_buf, count, false);

	dabspi->txlen = count;
	dabspi->user_write = true;
	mutex_unlock(&dabspi->txrx_lck);
	send_spi_data(dabspi->gpio_irq_num, dabspi);

unlock_n_ret:
	enable_irq(dabspi->gpio_irq_num);
	mutex_unlock(&dabspi->wlock);
	return (dabspi->status < 0) ? dabspi->status : count;
}

static int dab_release(struct inode *i, struct file *f)
{
	struct dabspi_data *dabspi;

	mutex_lock(&device_list_lock);
	dabspi = f->private_data;
	f->private_data = NULL;
	dabspi->users--;
	mutex_unlock(&device_list_lock);
	return 0;
}

static irqreturn_t dab_irq_handler(int irq, void *data)
{
	struct dabspi_data *dabspi = data;
	static int irqs;

	trace_dab_spi_irq(jiffies);

	if (irqs < 2)
		dabspi->bootmsg = dabbootmsg[irqs];
	else
		dabspi->bootmsg = dabbootmsg[2];
	irqs++;
	dabspi->user_write = false;

	pr_debug("IRQ == %d\n", irqs);

	return IRQ_WAKE_THREAD;
}

static struct class *dabspi_class;

static int dab_add_dev(struct spi_device *spi, struct dabspi_data *dabspi)
{
	int status;
	unsigned long minor;
	struct device *dev;

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, DYNAMIC_MINORS);
	if (minor >= DYNAMIC_MINORS) {
		dev_err(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
		goto end;
	}
	dabspi->devt = MKDEV(dab_major, minor);
	dev = device_create(dabspi_class, &spi->dev, dabspi->devt, dabspi,
			    "spi%d", spi->master->bus_num);
	status = PTR_ERR_OR_ZERO(dev);
	if (status) {
		pr_err("Could not create device file for minor %lu\n", minor);
		goto end;
	}
	set_bit(minor, minors);
	list_add(&dabspi->device_entry, &device_list);
end:
	mutex_unlock(&device_list_lock);
	return status;
}

static int dab_alloc_res(struct dabspi_data *dabspi)
{
	int status;

	status = kfifo_alloc(&dabspi->fifo, FIFO_SIZE, GFP_KERNEL);
	if (status) {
		pr_err("kfifo_alloc failed\n");
		return status;
	}

	dabspi->tx_buf = kzalloc(BUF_SIZE, GFP_KERNEL);
	if (!dabspi->tx_buf)
		goto err_txbuf_alloc;

	dabspi->rx_buf = kzalloc(BUF_SIZE, GFP_KERNEL);
	if (!dabspi->rx_buf)
		goto err_rxbuf_alloc;

	return status;

err_rxbuf_alloc:
	kfree(dabspi->tx_buf);
err_txbuf_alloc:
	kfifo_free(&dabspi->fifo);
	return -ENOMEM;
}

static void dab_dealloc_res(struct dabspi_data *dabspi)
{
	kfifo_free(&dabspi->fifo);
	kfree(dabspi->tx_buf);
	kfree(dabspi->rx_buf);
}

static int dab_gpio_alloc(struct dabspi_data *dabspi)
{
	int status;
	struct irq_desc *dab_irq_desc;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };

	if (!gpio_is_valid(dabspi->cs_gpio)) {
		pr_err("Invalid cs gpio number %d\n", dabspi->cs_gpio);
		goto err_gpio_invalid;
	}
	if (!gpio_is_valid(dabspi->irq_gpio)) {
		pr_err("Invalid irq gpio number %d\n", dabspi->irq_gpio);
		goto err_gpio_invalid;
	}
	status = gpio_request(dabspi->cs_gpio, "dab_cs_pin");
	if (status) {
		pr_err("Claiming gpio pin %d failed!\n", dabspi->cs_gpio);
		goto err_gpio_invalid;
	}
	status = gpio_request(dabspi->irq_gpio, "dab_irq_pin");
	if (status) {
		pr_err("Claiming gpio pin %d failed!\n", dabspi->irq_gpio);
		goto err_gpio_invalid;
	}
	status = gpio_direction_output(dabspi->cs_gpio, 1);
	if (status) {
		pr_err("Setting direction gpio %d failed\n", dabspi->cs_gpio);
		goto err_gpio_dir;
	}
	status = gpio_direction_input(dabspi->irq_gpio);
	if (status) {
		pr_err("Setting direction gpio %d failed\n", dabspi->irq_gpio);
		goto err_gpio_dir;
	}
	dabspi->gpio_irq_num = gpio_to_irq(dabspi->irq_gpio);
	status = request_threaded_irq(dabspi->gpio_irq_num, dab_irq_handler,
				      send_spi_data,
				      IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				      "dabplugin",
				      dabspi);
	if (status) {
		pr_err("requesting irq failed for irqnum = %d (gpio num = %d)\n",
			dabspi->gpio_irq_num, dabspi->irq_gpio);
		goto err_gpio_dir;
	}
	dab_irq_desc = irq_to_desc(dabspi->gpio_irq_num);
	sched_setscheduler(dab_irq_desc->action->thread, SCHED_FIFO, &param);
	sched_setscheduler(dabspi->spi->master->kworker_task, SCHED_FIFO,
			   &param);

	return status;

err_gpio_dir:
	gpio_free(dabspi->cs_gpio);
	gpio_free(dabspi->irq_gpio);
err_gpio_invalid:
	status = -EIO;
	return status;
}

static void dab_gpio_dealloc(struct dabspi_data *dabspi)
{
	free_irq(dabspi->gpio_irq_num, dabspi);
	gpio_free(dabspi->cs_gpio);
	gpio_free(dabspi->irq_gpio);
}

static int dab_spi_probe(struct spi_device *spi)
{
	int status;
	struct dabspi_data *dabspi;

	dabspi = kzalloc(sizeof(*dabspi), GFP_KERNEL);
	if (!dabspi)
		return -ENOMEM;

	/* Initialize the driver data */
	dabspi->spi = spi;
	mutex_init(&dabspi->rlock);
	mutex_init(&dabspi->wlock);
	mutex_init(&dabspi->txrx_lck);
	INIT_LIST_HEAD(&dabspi->device_entry);
	init_completion(&dabspi->data_avail);

	dabspi->speed_hz = spi->max_speed_hz;
	dabspi->cs_gpio = dab_cs_gpios[spi->master->bus_num][spi->chip_select];
	dabspi->irq_gpio =
		dab_irq_gpios[spi->master->bus_num][spi->chip_select];
	spi_set_drvdata(spi, dabspi);

	status = dab_alloc_res(dabspi);
	if (status)
		goto err;

	status = dab_gpio_alloc(dabspi);
	if (status)
		goto err_res;

	status = dab_add_dev(spi, dabspi);
	if (status)
		goto err_dev;

	spi->mode = SPI_MODE_1;
	spi->max_speed_hz = 2000000;
	spi->cs_gpio = -1;
	status = spi_setup(spi);

	return status;

err_dev:
	dab_gpio_dealloc(dabspi);
err_res:
	dab_dealloc_res(dabspi);
err:
	kfree(dabspi);
	return status;
}

static int dab_spi_remove(struct spi_device *dev)
{
	struct dabspi_data *dabspi = spi_get_drvdata(dev);

	if (!dabspi)
		goto end;

	synchronize_irq(dabspi->gpio_irq_num);
	dabspi->spi = NULL;
	complete_all(&dabspi->data_avail);
	mutex_lock(&device_list_lock);
	list_del(&dabspi->device_entry);
	device_destroy(dabspi_class, dabspi->devt);
	clear_bit(MINOR(dabspi->devt), minors);
	mutex_unlock(&device_list_lock);
	dab_dealloc_res(dabspi);
	dab_gpio_dealloc(dabspi);
	kfree(dabspi);
end:
	return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id dabplugin_of_match[] = {
	{
		.compatible = "dabplugin",
	},
	{}
};
MODULE_DEVICE_TABLE(of, dabplugin_of_match);
#endif

static const struct spi_device_id dab_spi_id[] = {
	{ "dabplugin", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, dab_spi_id);

static struct spi_driver dab_spi_driver = {
	.driver			= {
		.name		= "dabplugin",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(dabplugin_of_match),
	},
	.probe			= dab_spi_probe,
	.remove			= dab_spi_remove,
	.id_table		= dab_spi_id,
};

static const struct file_operations dab_fops = {
	.owner		= THIS_MODULE,
	.open		= dab_open,
	.read		= dab_read,
	.write		= dab_write,
	.release	= dab_release,
	.llseek		= no_llseek,
};

static int __init dab_spi_init(void)
{
	int status;

	dab_major = register_chrdev(0, "dabspi", &dab_fops);
	if (dab_major < 0) {
		pr_err("Device registration failed\n");
		return dab_major;
	}

	dabspi_class = class_create(THIS_MODULE, "dabspi");
	if (IS_ERR(dabspi_class)) {
		status = PTR_ERR(dabspi_class);
		goto class_fail;
	}

	status = spi_register_driver(&dab_spi_driver);
	if (status < 0)
		goto spi_reg_fail;

	return status;

spi_reg_fail:
	class_destroy(dabspi_class);
class_fail:
	unregister_chrdev(dab_major, dab_spi_driver.driver.name);
	return status;
}
module_init(dab_spi_init);

static void __exit dab_spi_exit(void)
{
	spi_unregister_driver(&dab_spi_driver);
	class_destroy(dabspi_class);
	unregister_chrdev(dab_major, dab_spi_driver.driver.name);
}
module_exit(dab_spi_exit);

MODULE_AUTHOR("Vikram N <vikram.narayanarao@harman.com>");
MODULE_DESCRIPTION("Driver for DAB SPI interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidabplugin");
