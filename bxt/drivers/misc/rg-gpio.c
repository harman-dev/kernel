/*
 * Generic GPIO reverse gear
 *
 * Copyright (c) 2017  Zhang Ning (ning.a.zhang@intel.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/input.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/gpio.h>

#define RG_MODNAME		"rg-gpio"

static const struct acpi_gpio_params rg_gpio    = { 0, 0, false };
static const struct acpi_gpio_mapping gpio_rg_acpi_gpios[] = {
	{"rg-gpios",    &rg_gpio,    1},
	{}
};

static ssize_t gpio_rg_value_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gpio_desc *desc = dev_get_drvdata(dev);
	int val, len;

	val = gpiod_get_value(desc);

	len = sprintf(buf, "%d", val);

	return len;
}

static DEVICE_ATTR(gpio_rg_state, S_IRUSR|S_IRGRP,
	gpio_rg_value_show,
	NULL);


static int gpio_rg_probe(struct platform_device *pdev)
{

	const struct acpi_device_id *id;
	struct acpi_device *acpi_dev;
	struct gpio_desc	*desc;
	struct device *dev = &pdev->dev;

	id = acpi_match_device(dev->driver->acpi_match_table, dev);
	if (!id)
		return -ENODEV;

	acpi_dev_add_driver_gpios(ACPI_COMPANION(&pdev->dev),
			gpio_rg_acpi_gpios);

	if (acpi_bus_get_device(ACPI_HANDLE(dev), &acpi_dev)) {
		dev_err(dev, "reverse gear GPIO device node not found\n");
		return -ENODEV;
	}

	desc = devm_gpiod_get_optional(dev, "rg", GPIOD_IN);

	if (IS_ERR(desc)) {
		dev_err(dev, "reverse Failed to request gpio: %ld\n",
					PTR_ERR(desc));
		return PTR_ERR(desc);
	}

	dev_set_drvdata(dev, desc);

	device_create_file(dev, &dev_attr_gpio_rg_state);

	return 0;
}

static const struct acpi_device_id gpio_rg_acpi_match[] = {
	{ "RGD0001", 0 },
	{ },
};

MODULE_DEVICE_TABLE(acpi, gpio_rg_acpi_match);

static struct platform_driver gpio_rg_platform_driver = {
	.driver	= {
		.name		= RG_MODNAME,
		.acpi_match_table = ACPI_PTR(gpio_rg_acpi_match),
	},
	.probe	= gpio_rg_probe,
};

static int __init gpio_rg_init(void)
{
	return platform_driver_register(&gpio_rg_platform_driver);
}
late_initcall(gpio_rg_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("");
MODULE_DESCRIPTION("Generic GPIO reverse gear driver");
