#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/can/platform/mcp251x.h>
#include <linux/pwm.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <media/ti949.h>

#ifdef CONFIG_X86_64_GM_MY20_HW
#include <linux/i2c.h>
#endif
#include <linux/interrupt.h>

// added for nfc
#include <nfc/ncf3340_pdata.h>
#define NCF3340_ADDR 0x28

#ifdef CONFIG_X86_64_OAKLAND_HW
#define AK4438_ADDR 0x10
#endif

#ifdef CONFIG_X86_64_GM_MY20_HW
#define ATMEL_TS_ADDR 0x4B
#endif

#define DRVNAME "bxt-board"

static struct pxa2xx_spi_chip chip_data = {
	.gpio_cs = -EINVAL,
        .dma_burst_size = 1,
        .pio_dma_threshold = 8,
};
static struct mcp251x_platform_data overo_mcp2515_pdata = {
        .oscillator_frequency   = 24*1000*1000,
	.gpio_int = 434+22, // GPIO base 434 + gpio 22
};


struct ti949_pdata ti949_serdes_pdata = {
    .ti949_i2c_addr     = TI949_I2C_ADDR,
    .ti948_i2c_addr     = TI948_I2C_ADDR,
    .faceplate_i2c_addr = FACEPLATE_I2C_ADDR,
    .i2c_adapter        = TI949_I2C_ADAPTER,
    .pdb_gpio           = (434+33),
    .irq_intb           = (434+15),
    .irq_intb_name      = "TI949_INTB",
};

static struct spi_board_info bxt_spi_slaves[] = {
	{
		.modalias = "spidev",
		.max_speed_hz = 50000000,
		.bus_num = 1,
		.chip_select = 0,
		.controller_data = &chip_data,
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 50000000,
		.bus_num = 2,
		.chip_select = 0,
		.controller_data = &chip_data,
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 50000000,
		.bus_num = 2,
		.chip_select = 1,
		.controller_data = &chip_data,
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "mcp2515",
		.platform_data  = &overo_mcp2515_pdata,
		.max_speed_hz = 10000000,
		.bus_num = 3,
		.chip_select = 0,
		.controller_data = &chip_data,
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 50000000,
		.bus_num = 3,
		.chip_select = 1,
		.controller_data = &chip_data,
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 50000000,
		.bus_num = 3,
		.chip_select = 2,
		.controller_data = &chip_data,
		.mode = SPI_MODE_0,
	}
};

static int bxt_spi_board_setup(void)
{
	int ret = -1;

	/* Register the SPI devices */
	ret = spi_register_board_info(bxt_spi_slaves,
				ARRAY_SIZE(bxt_spi_slaves));
	if (ret)
		pr_warn(DRVNAME ": failed to register the SPI slaves...(%d)\n",
				ret);
	else
		pr_debug(DRVNAME ": successfully registered the SPI slaves...\n");
	return ret;
}


#ifdef CONFIG_X86_64_GM_MY20_HW
static struct mxt_platform_data atmel_ts_i2c7_pdata = {
	.input_name = "atmel_mxt_ts",
	.irqflags = IRQF_TRIGGER_LOW,
	.t19_num_keys = 1,
        .t15_num_keys = 1,
        //.gpio_reset;
        .gpio_int = 434+14,
        //.gpio_switch;
        .cfg_name="maxtouch.cfg",
        .regulator_dis = 1,
};
static struct i2c_board_info oakland_i2c_devs7[] __initdata = {
         { I2C_BOARD_INFO("atmel_mxt_ts", ATMEL_TS_ADDR),
	.platform_data = &atmel_ts_i2c7_pdata},
	{
	I2C_BOARD_INFO("faceplate_control", 0x12),
	.irq = 441,
	},
         { I2C_BOARD_INFO(TI949_NAME, TI949_I2C_ADDR),
        .platform_data = &ti949_serdes_pdata },
};
#endif


// Added by CoC-Connectivity for NFC

static struct nxp7120_i2c_platform_data ncf3340_pdata = {
	.irq_gpio = (434 + 17), // GPIO 17 as per schematic
};

static struct i2c_board_info nfc_i2c_dev3340[] __initdata = {
	{ I2C_BOARD_INFO("nxp7120" , NCF3340_ADDR),
	.platform_data = &ncf3340_pdata },

};

static int bxt_clk_setup(void)
{
	struct clk *clk;

	/* Make clock tree required by the PWM driver */
	clk = clk_register_fixed_rate(NULL, "pwm_clk", "lpss_clk", 0, 25000000);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	clk_register_clkdev(clk, NULL, "0000:00:1a.0");

	return 0;
}

static int __init bxt_board_init(void)
{
	int ret;
	pr_info(DRVNAME ": registering BXT PWM devices...\n");
	ret = bxt_clk_setup();
	if (ret)
		goto exit;
	pr_info(DRVNAME ": registering BXT SPI devices...\n");
	ret = bxt_spi_board_setup();
	if (ret)
		goto exit;
#ifdef CONFIG_X86_64_OAKLAND_HW
#endif

#ifdef CONFIG_X86_64_GM_MY20_HW
       //Atmel MXT641T defn i nthe display controller
	ret = i2c_register_board_info(7, oakland_i2c_devs7,
                                ARRAY_SIZE(oakland_i2c_devs7));
	pr_info(DRVNAME "oakland_i2c_devs7 register ret value %d\n",ret);
	//Put the NFC defintions here
#endif

	pr_info(DRVNAME ": registering BXT NFC Device ... \n");
	i2c_register_board_info(7, nfc_i2c_dev3340,
					ARRAY_SIZE(nfc_i2c_dev3340));

exit:
	return ret;
}
arch_initcall(bxt_board_init);
MODULE_LICENSE(GPL);
