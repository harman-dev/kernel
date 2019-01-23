/*
 * UART TLIN driver.
 *
 * Copyright (C) 2017 HARMAN.
 *
 * Authors:
 *  Nithin Prakash  <nithin.prakash@harman.com>
 *  Dipin Kumar     <dipin.kumar@harman.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#define DRIVER_NAME                   "tlin"
#define TLIN_BAUDRATE                  20000
#define UART_RX_FIFO_LVL                  33
#define LIN_MSG_BUF_SIZE                  20
#define TLIN_HEADER_LEN                    2
#define TLIN_IBIT_TIME                    50
#define TLIN_BRK_TIME_MIN                800
#define TLIN_BRK_TIME_MAX               1000
#define MILLISEC_TO_NANOSEC(x) (x * 1000000)


static int tlin_open(struct inode *i, struct file *f);
static int tlin_release(struct inode *i, struct file *f);
static ssize_t tlin_read(struct file *fp, char __user *buf,
                         size_t len, loff_t *off);
static ssize_t tlin_write(struct file *filep, const char *buffer,
                          size_t len, loff_t *offset);
static void  serial_out(struct uart_port *port, int offset, int value);
static unsigned int  serial_in(struct uart_port *port, int offset);
static int tlin_reg_config(void);
static void serial_clear_fifos(struct uart_port *port);
int validate_frame(char *rxBuf, char *txBuf, int len, int *match_bytes);

static DECLARE_WAIT_QUEUE_HEAD(tlin_read_wq);
static DEFINE_SPINLOCK(tlin_lock);

static struct        uart_port tlin_port;
static struct        uart_port *port;
static unsigned char tx_buf[LIN_MSG_BUF_SIZE];
static unsigned char rx_buf[LIN_MSG_BUF_SIZE];
static struct hrtimer hr_timer;
int timer_event;



static struct file_operations tlin_fops =
{
    .owner   = THIS_MODULE,
    .open    = tlin_open,
    .release = tlin_release,
    .read    = tlin_read,
    .write   = tlin_write,
};

typedef struct tlin_data {
    struct clk      *clk;
    struct clk      *pclk;
    dev_t            major_num;
    struct device   *dev_ret;
    struct cdev      tlin_cdev;
    struct class    *tlin_class;

} TLIN_DATA;

static void  serial_out(struct uart_port *port, int offset, int value)
{
    writel(value, port->membase + (offset << 2));
}

static unsigned int  serial_in(struct uart_port *port, int offset)
{
    return readl(port->membase + (offset << 2));
}

static int tlin_reg_config(void)
{
    unsigned char c;
    unsigned int  ier;
    unsigned int  divisor;

    //8 data bit, no parity, one stop bit , so final value LCR = 0x3
    //serial_out(port, UART_LCR, 0x3); /* 8n1 */
    serial_out(port, UART_LCR, UART_LCR_WLEN8); /* 8n1 */
    ier = serial_in(port, UART_IER);
    serial_out(port, UART_IER, 0x0); /* no interrupt */

    serial_clear_fifos(port);
    serial_out(port, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_01
                               | UART_FCR_DMA_SELECT);
    serial_out(port, UART_MCR, 0x0);
    divisor = DIV_ROUND_CLOSEST(port->uartclk, 16 * TLIN_BAUDRATE);
    c = serial_in(port, UART_LCR);
    serial_out(port, UART_LCR, c | UART_LCR_DLAB);
    serial_out(port, UART_DLL, divisor & 0xff);
    serial_out(port, UART_DLM, (divisor >> 8) & 0xff);
    serial_out(port, UART_LCR, c & ~UART_LCR_DLAB);

    return 0;
}

static void serial_break_ctl(int break_state)
{
    unsigned char lcr;

    lcr = serial_in(port, UART_LCR);
    if (break_state == 1) {
        lcr |= UART_LCR_SBC;
    }
    else {
        lcr &= ~UART_LCR_SBC;
    }
    serial_out(port, UART_LCR, lcr);
}

static void serial_clear_fifos(struct uart_port *port)
{
    //serial_out(port, UART_FCR, UART_FCR_ENABLE_FIFO);
    serial_out(port, UART_FCR, UART_FCR_ENABLE_FIFO |
                               UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
    //serial_out(port, UART_FCR, 0);
    serial_out(port, UART_FCR, UART_FCR_ENABLE_FIFO |
                               UART_FCR_R_TRIG_01 | UART_FCR_DMA_SELECT);
    udelay(1);

}

int validate_frame(char *rxBuf, char *txBuf, int len, int *match_bytes)
{
    int i, matchingBytes = 0;

    if((len >= LIN_MSG_BUF_SIZE) || (len <= 0)) {
        printk(KERN_INFO"LIN CMD Invalid Length\n");
        return -1;
    }

    for(i = 0; i < len; i++) {
        if(rxBuf[i] == txBuf[i])
            matchingBytes++;
    }
    #ifdef DEBUG_TLIN
        printk(KERN_INFO"matching bytes = 0x%x\n", matchingBytes);
    #endif
    if(matchingBytes == len){
        *match_bytes = matchingBytes;
        return 0;
    } else {
        matchingBytes = 0;
        for(i = 0; i < len; i++){
            if(rxBuf[i + 1] == txBuf[i])
                matchingBytes++;
        }

        #ifdef DEBUG_TLIN
            printk(KERN_INFO"matching bytes = 0x%x\n", matchingBytes);
        #endif
        if(matchingBytes == len) {
            *match_bytes = matchingBytes;
            return 0;
        } else {
            #ifdef DEBUG_TLIN
            for(i = 0; i < len; i++) {
                printk(KERN_INFO"[rxBuf[%d]=0x%x]\n", i, rxBuf[i]);
            }

            for(i = 0; i < len; i++) {
                printk(KERN_INFO"[txBuf[%d]=0x%x]\n", i, txBuf[i]);
            }
            #endif
            return -1;
        }
    }
}


static irqreturn_t serial_interrupt_handler(int irq, void *dev_id)
{
    serial_in(port, UART_IIR);

    return IRQ_HANDLED;
}

/*
 * lin_open - the tlin open() file operation
 *
 */
static int tlin_open(struct inode *i, struct file *f)
{
    return 0;
}

/*
 * tlin_release - the lin  release file operation
 *
 */
static int tlin_release(struct inode *i, struct file *f)
{
    return 0;
}

enum hrtimer_restart tlin_hrtimer_callback(struct hrtimer *timer)
{
    timer_event = 1;
    wake_up_interruptible(&tlin_read_wq);

    return HRTIMER_NORESTART;
}

/*
 * tin_read - the lin read() file operation
 * Expected commands are
 * Display Status Cmd = 0x9C. Response is 8 Bytes + Checksum
 * Touch Velocity Cmd = 0x1D. Response is 8 Bytes + Checksum
 * Software Version Cmd = 0x1F. Response is 3 Bytes + Checksum
 */
static ssize_t tlin_read(struct file *fp, char __user *buf,
                         size_t count, loff_t *pos)
{
    int           ret;
    int           i;
    char          fifo_lvl;
    unsigned long flags;
    unsigned char lsr;
    int match_bytes = 0;
    int len;
    ktime_t ktime;

    memset(tx_buf, 0, LIN_MSG_BUF_SIZE);
    memset(rx_buf, 0, LIN_MSG_BUF_SIZE);

    len = TLIN_HEADER_LEN;//transmitting command frame size

    if(copy_from_user(tx_buf, buf, TLIN_HEADER_LEN)) {
        dev_err(port->dev, "LIN write error\n");
        return -EIO;
    }

    //sending break
    serial_break_ctl(1);
    //Minimum break time required is 650uS
    usleep_range(TLIN_BRK_TIME_MIN, TLIN_BRK_TIME_MAX);

    serial_break_ctl(0);

    serial_clear_fifos(port);
    serial_in(port, UART_LSR);
    serial_in(port, UART_RX);
    serial_in(port, UART_IIR);
    serial_in(port, UART_MSR);

    //BREAK_DELIMITER Delay. At least 1 bit duration delay should be present
    //between BREAK and SYNC
    udelay(TLIN_IBIT_TIME);

    spin_lock_irqsave(&tlin_lock, flags);

    //transmitting header
    for(i = 0; i < len; i++) {
        serial_out(port, UART_TX, tx_buf[i]);
    }

    spin_unlock_irqrestore(&tlin_lock, flags);

    ktime = ktime_set(0, MILLISEC_TO_NANOSEC(3));
    timer_event = 0;
    hrtimer_start(&hr_timer, ktime, HRTIMER_MODE_REL);
    wait_event_interruptible_timeout(tlin_read_wq, timer_event == 1,
                                     msecs_to_jiffies(4));
    timer_event = 0;
    lsr = serial_in(port, UART_LSR);
    fifo_lvl = serial_in(port, UART_RX_FIFO_LVL);

    //validating transmitted frame
    if(fifo_lvl >= len) {
        spin_lock_irqsave(&tlin_lock, flags);

        for(i = 0; i < len; i++) {
            lsr = serial_in(port, UART_LSR);
            if (lsr & (UART_LSR_FE | UART_LSR_PE | UART_LSR_OE)) {
                serial_clear_fifos(port);
                spin_unlock_irqrestore(&tlin_lock, flags);
                printk(KERN_INFO"LIN Receive Error LSR=0x%x\n", lsr);
                return -1;
            } else {
                rx_buf[i] = serial_in(port, UART_RX);
            }
        }
        spin_unlock_irqrestore(&tlin_lock, flags);

        //check received data
        ret = validate_frame(rx_buf, tx_buf, len, &match_bytes);
        if(ret == 0)
             ;//successfully transmitted cmd and received
        else {
            printk(KERN_INFO"LIN Receive validation Error\n");
            serial_clear_fifos(port);
            return -1;//transmission error
        }
    } else {
        printk(KERN_INFO"LIN Header Receive Error\n");
        serial_clear_fifos(port);
        return -1;//transmission error
    }

    #ifdef DEBUG_TLIN
        printk(KERN_INFO"Timing[%s][%d]\n",__func__,__LINE__);
    #endif
    ktime = ktime_set(0, MILLISEC_TO_NANOSEC(5));
    hrtimer_start(&hr_timer, ktime, HRTIMER_MODE_REL);
    wait_event_interruptible_timeout(tlin_read_wq, timer_event == 1,
                                     msecs_to_jiffies(6));
    timer_event = 0;

    fifo_lvl = serial_in(port, UART_RX_FIFO_LVL);

    #ifdef DEBUG_TLIN
        printk(KERN_INFO"Timing + i=0x%d + fifo_lvl=0x%x [LINE=%d]\n",
                                          i,fifo_lvl,__LINE__);
    #endif

    //already read 2 bytes
    count = count - 2;

    //expected number of data not received
    //rechecking after some time before returning error
    if(fifo_lvl < count) {
        msleep(5);
    }

    fifo_lvl = serial_in(port, UART_RX_FIFO_LVL);

    if(fifo_lvl >= count) {
        //receive packet
        spin_lock_irqsave(&tlin_lock, flags);
        for(i = 0; i < fifo_lvl; i++) {
            lsr = serial_in(port, UART_LSR);
            if (lsr & (UART_LSR_FE | UART_LSR_PE | UART_LSR_OE)) {
                printk(KERN_INFO"LIN Receive Error LSR=0x%x\n",lsr);
                spin_unlock_irqrestore(&tlin_lock, flags);
                return -1;
            } else {
                rx_buf[i + 2] = serial_in(port, UART_RX);
            }
        }
        spin_unlock_irqrestore(&tlin_lock, flags);

        #ifdef DEBUG_TLIN
            printk(KERN_INFO"value of i = 0x%x",i);
        #endif

        if(copy_to_user(buf, rx_buf, fifo_lvl + 2)) {
            printk(KERN_INFO"Data not copied completely to user space");
            return -EFAULT;
        } else {
            #ifdef DEBUG_TLIN
                printk(KERN_INFO"value of i = 0x%x",i);
            #endif
            return i;
        }
    } else {
        printk(KERN_INFO"LIN Data Receive Error\n");
        serial_clear_fifos(port);
        return -2;//receive error
    }


}

/*
 * lin_write - the tlin write() file operation
 *
 */
static ssize_t tlin_write(struct file *filep, const char *buf,
                          size_t len, loff_t *offset)
{
    int ret;
    int i;
    unsigned long flags;
    unsigned char lsr;
    unsigned char fifo_lvl;
    size_t check;
    unsigned int index_tx = 0;
    int match_bytes = 0;
    unsigned long tlime;
    int retry = 2;
    ktime_t ktime;


    memset(tx_buf, 0, LIN_MSG_BUF_SIZE);
    memset(rx_buf, 0, LIN_MSG_BUF_SIZE);
    timer_event = 0;

    if(copy_from_user(tx_buf, buf, len)) {
        dev_info(port->dev, "write error\n");
        return -EFAULT;
    }

    serial_break_ctl(1);

    /*Break is minimum 650uS*/
    usleep_range(TLIN_BRK_TIME_MIN, TLIN_BRK_TIME_MAX);

    serial_break_ctl(0);

    serial_clear_fifos(port);
    serial_in(port, UART_LSR);
    serial_in(port, UART_RX);
    serial_in(port, UART_IIR);
    serial_in(port, UART_MSR);

    //BREAK_DELIMITER Delay. At least 1 bit duration delay should be present
    //between BREAK and SYNC
    udelay(TLIN_IBIT_TIME);

    spin_lock_irqsave(&tlin_lock, flags);

    //If Tx FIFO is empty then transmit data
    for(i = 0; i < len; i++) {
        serial_out(port, UART_TX, tx_buf[i]);
    }

    spin_unlock_irqrestore(&tlin_lock, flags);

    //wait for 2 byte time and check consistency of packet being transmitted
    //usleep_range(TLIN_IBIT_TIME * 16, TLIN_IBIT_TIME * 16);

    lsr = serial_in(port, UART_LSR);

    tlime = MILLISEC_TO_NANOSEC(2);

    while(check < len) {
        ktime = ktime_set(0, tlime);
        timer_event = 0;
        hrtimer_start(&hr_timer, ktime, HRTIMER_MODE_REL);
        wait_event_interruptible_timeout(tlin_read_wq, timer_event == 1,
                                         msecs_to_jiffies(3));
        timer_event = 0;
        tlime = 1200000;
        fifo_lvl = serial_in(port, UART_RX_FIFO_LVL);

        if(fifo_lvl > 0) {
            retry = 2;
            spin_lock_irqsave(&tlin_lock, flags);

            for(i = 0; i < fifo_lvl; i++) {
                lsr = serial_in(port, UART_LSR);
                if (lsr & (UART_LSR_FE | UART_LSR_PE | UART_LSR_OE)) {
                    serial_clear_fifos(port);
                    spin_unlock_irqrestore(&tlin_lock, flags);
                    dev_info(port->dev, "LIN Receive Error LSR = 0x%x\n", lsr);
                    return -1;
                } else {
                    rx_buf[i] = serial_in(port, UART_RX);
                }
            }

            spin_unlock_irqrestore(&tlin_lock, flags);

            //check received data
            ret = validate_frame(rx_buf, &tx_buf[index_tx], fifo_lvl, &match_bytes);

            if(ret == 0) {
                index_tx += match_bytes;
            } else {
                serial_clear_fifos(port);
                dev_info(port->dev, KERN_INFO"Lin validate_frame Error\n");
                return -1;//receive error
            }
            check += fifo_lvl;
        } else {
            if (retry <= 0) {
                serial_clear_fifos(port);
                dev_info(port->dev, "Lin write: Receive Error\n");
                return -2;//receive error
            } else {
                retry--;
            }
        }

    }

    return check;
}

#if 0
static void serial_disable_interrupt(void)
{
    serial_out(port, UART_IER, 0x00);
}

static void serial_enable_interrupt(void)
{
    serial_out(port, UART_IER, 0x01);
}
#endif
static int tlin_probe(struct platform_device *pdev)
{
    int    ret;
    struct resource *regs;
    int irq;
    TLIN_DATA *data;
    int err;

    dev_info(&pdev->dev, "%s V1\n", __FUNCTION__);

    regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);

    if (!regs) {
        dev_err(&pdev->dev, "no registers defined\n");
        return -EINVAL;
    }

    irq = platform_get_irq(pdev, 0);

    if (irq < 0) {
        if (irq != -EPROBE_DEFER)
            dev_err(&pdev->dev, "cannot get irq\n");
        return irq;
    }

    port = &tlin_port;

    port->mapbase  = regs->start;
    port->dev      = &pdev->dev;
    port->irq      = irq;

    port->membase = devm_ioremap(port->dev, port->mapbase, 0x200);
    if (!port->membase) {
        dev_err(&pdev->dev, "io remap failed\n");
        return -ENOMEM;
    }

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data) {
        return -ENOMEM;
    }

    /* If there is separate baudclk, get the rate from it. */
    data->clk = devm_clk_get(&pdev->dev, "baudclk");

    if (IS_ERR(data->clk) && PTR_ERR(data->clk) != -EPROBE_DEFER) {
        data->clk = devm_clk_get(&pdev->dev, NULL);
    }

    if (IS_ERR(data->clk) && PTR_ERR(data->clk) == -EPROBE_DEFER) {
        dev_err(&pdev->dev, "clk get err\n");
        return -EPROBE_DEFER;
    }

    if (!IS_ERR_OR_NULL(data->clk)) {
        err = clk_prepare_enable(data->clk);
        if (err) {
            dev_warn(&pdev->dev, "could not enable optional baudclk: %d\n", err);
        } else {
            port->uartclk = clk_get_rate(data->clk);
        }
    }
    /* If no clock rate is defined, fail. */
    if (!port->uartclk) {
        dev_err(&pdev->dev, "clock rate not defined\n");
        return -EINVAL;
    }

    data->pclk = devm_clk_get(&pdev->dev, "apb_pclk");
    if (IS_ERR(data->pclk) && PTR_ERR(data->pclk) == -EPROBE_DEFER) {
        dev_err(&pdev->dev, "could not enable apb_pclk\n");
        err = -EPROBE_DEFER;
        goto err_clk;
    }
    if (!IS_ERR(data->pclk)) {
        err = clk_prepare_enable(data->pclk);
        if (err) {
            dev_err(&pdev->dev, "could not enable apb_pclk\n");
            goto err_clk;
        }
    }

    pm_runtime_set_active(&pdev->dev);
    pm_runtime_enable(&pdev->dev);
    pm_runtime_forbid(&pdev->dev);

    tlin_reg_config();

    ret = devm_request_irq(&pdev->dev, port->irq, serial_interrupt_handler,
                           IRQF_SHARED, "tlin_interrupt", port->dev);
    if (ret < 0) {
        dev_err(&pdev->dev, "Interrupt request failed\n");
        goto err_clk;
    }

    hrtimer_init(&hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

    hr_timer.function = &tlin_hrtimer_callback;

    ret = alloc_chrdev_region (&data->major_num, 0, 1, DRIVER_NAME);

    if (ret < 0) {
        dev_err(&pdev->dev, "failed to reserve major/minor range\n");
        goto err_cancel_hrtimer;
    }

    cdev_init(&data->tlin_cdev, &tlin_fops);
    data->tlin_cdev.owner = THIS_MODULE;

    ret = cdev_add(&data->tlin_cdev, data->major_num, 1);
    if( ret < 0 ) {
        dev_err(&pdev->dev, "cdev_add failed\n");
        goto err_relese_chrdev_region;
    }

    data->tlin_class = class_create(THIS_MODULE, DRIVER_NAME);

    if (IS_ERR(data->tlin_class)) {
        dev_err(&pdev->dev, "Error class_create\n");
        goto err_cdev_del;
    }

    data->dev_ret = device_create(data->tlin_class, NULL, data->major_num, NULL,
                                  DRIVER_NAME);
    if (IS_ERR(data->dev_ret)) {
        dev_err(&pdev->dev, "Error device_create\n");
        goto err_class_distroy;
    }

    platform_set_drvdata(pdev, data);

    return 0;


err_class_distroy:
    class_destroy(data->tlin_class);

err_cdev_del:
    cdev_del(&data->tlin_cdev);

err_relese_chrdev_region:
    unregister_chrdev_region (data->major_num, 1);

err_cancel_hrtimer:
    hrtimer_cancel(&hr_timer);

err_clk:
    if (!IS_ERR(data->clk))
        clk_disable_unprepare(data->clk);

    return -1;
}

static int tlin_remove(struct platform_device *pdev)
{
    TLIN_DATA *data = platform_get_drvdata(pdev);

    pm_runtime_get_sync(&pdev->dev);

    hrtimer_cancel(&hr_timer);
    device_destroy(data->tlin_class, data->major_num);
    class_destroy(data->tlin_class);
    cdev_del(&data->tlin_cdev);
    unregister_chrdev_region(data->major_num, 1);

    if (!IS_ERR(data->pclk))
        clk_disable_unprepare(data->pclk);

    if (!IS_ERR(data->clk))
        clk_disable_unprepare(data->clk);

    pm_runtime_disable(&pdev->dev);
    pm_runtime_put_noidle(&pdev->dev);

    return 0;
}

static struct platform_driver tlin_platform_driver = {
    .driver = {
        .name       = "tlin",
    },
    .probe          = tlin_probe,
    .remove         = tlin_remove,
};

module_platform_driver(tlin_platform_driver);

MODULE_AUTHOR("HARMAN>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TLIN driver");
