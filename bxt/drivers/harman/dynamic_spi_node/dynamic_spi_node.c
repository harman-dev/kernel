#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/spi/pxa2xx_spi.h>

unsigned short mode;

module_param(mode, ushort, 0);
MODULE_PARM_DESC(mode, " 0-spi node for spidev, 1-spi node for DAB ");

static struct pxa2xx_spi_chip chip_data = {
	.gpio_cs = -EINVAL,
        .dma_burst_size = 1,
        .pio_dma_threshold = 8,
};

struct spi_board_info spi_device_info_spidev = {
	.modalias = "spidev",
	.max_speed_hz = 50000000, /*50MHz*/
	.bus_num = 1,
	.chip_select = 1,
	.controller_data = &chip_data,
	.mode = SPI_MODE_1,
};

struct spi_board_info spi_device_info_dab = {
	.modalias = "dabplugin",
	.max_speed_hz = 2000000, /*20MHz*/
	.bus_num = 1,
	.chip_select = 1,
	.controller_data = &chip_data,
	.mode = SPI_MODE_1,
};



/* SPI device*/
static struct spi_device *spi_device;

static int spi_init(void)
{
	struct spi_master *master;
	struct spi_board_info spidev_info;
	int ret;

	if (mode <= 1) {
		spidev_info = ((mode == 1) ? spi_device_info_dab  :
		       spi_device_info_spidev);
                       printk("bus_num:%d, mode:%d, chip_select:%d, max_speed_hz:%d =>\n",
                        spidev_info.bus_num, spidev_info.mode,
                        spidev_info.chip_select,
                        spidev_info.max_speed_hz);
                  
		pr_debug("bus_num:%d, mode:%d, chip_select:%d, max_speed_hz:%d =>\n",
			spidev_info.bus_num, spidev_info.mode,
			spidev_info.chip_select,
			spidev_info.max_speed_hz);
	} else {
		pr_err("module param mode not correct. Insert driver with correct mode\n");
		return -EPERM;
	}

	master = spi_busnum_to_master(spidev_info.bus_num);
	if (!master)
	{
		printk(" spi bus not availabe\n");
		return -ENODEV;
	}

	/*instantiate one new SPI device, provided the master and
	device information*/
	spi_device = spi_new_device(master, &spidev_info);


	if (!spi_device)
	{
		printk("new device cannot be added\n");
		return -ENODEV;
	}

	/*setup SPI mode and clock rate*/
	ret = spi_setup(spi_device);
	if (ret)
		spi_unregister_device(spi_device);
	return ret;
}

static void spi_exit(void)
{
	spi_unregister_device(spi_device);
}

module_init(spi_init);
module_exit(spi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dhananjay Kangude<dhananjay.kangude@harman.com>");
MODULE_DESCRIPTION("Driver for dynamic SPI device node switching");
