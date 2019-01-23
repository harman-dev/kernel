/*
 * board_thermal.c - Board thermal driver
 * Copyright (c) 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/thermal.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/smp.h>
#include <asm/mce.h>

/* The default value in module initilization */
#define INIT_TEMP 25000

struct phy_dev_entry {
	struct list_head list;
	struct list_head phy_dev_list;
	struct thermal_zone_device *tzone;
	struct mutex phy_dev_list_mutex;
};

static struct phy_dev_entry *phy_dev_obj;

static struct thermal_zone_params board_temp_tz_param = {
	.no_hwmon = true,
};

static int
sys_get_curr_temp(struct thermal_zone_device *tzd, int *temp)
{
	struct phy_dev_entry *phy_dev_obj;

	phy_dev_obj = tzd->devdata;
	*temp = tzd->temperature;

	return 0;
}

static int
sys_get_trip_temp(struct thermal_zone_device *tzd,
			int trip, int *temp)
{
	struct phy_dev_entry *phy_dev_entry;

	phy_dev_entry = tzd->devdata;
	*temp = 0;

	return 0;
}

static int
sys_get_trip_type(struct thermal_zone_device *thermal,
			int trip, enum thermal_trip_type *type)
{
	*type = THERMAL_TRIP_PASSIVE;

	return 0;
}

static struct thermal_zone_device_ops tzone_ops = {
	.get_temp = sys_get_curr_temp,
	.get_trip_temp = sys_get_trip_temp,
	.get_trip_type = sys_get_trip_type,
};

static int board_temp_thermal_device_add(void)
{
	phy_dev_obj = kzalloc(sizeof(*phy_dev_obj), GFP_KERNEL);
	if (!phy_dev_obj) {
		return  -ENOMEM;
	}

	mutex_init(&phy_dev_obj->phy_dev_list_mutex);
	mutex_lock(&phy_dev_obj->phy_dev_list_mutex);

	phy_dev_obj->tzone = thermal_zone_device_register("board_temp",
            1, 0, phy_dev_obj, &tzone_ops, &board_temp_tz_param, 0, 0);

	if (IS_ERR(phy_dev_obj->tzone)) {
		return PTR_ERR(phy_dev_obj->tzone);
	} else {
		phy_dev_obj->tzone->temperature = INIT_TEMP;
	}

	INIT_LIST_HEAD(&phy_dev_obj->phy_dev_list);
	list_add_tail(&phy_dev_obj->list, &phy_dev_obj->phy_dev_list);
	mutex_unlock(&phy_dev_obj->phy_dev_list_mutex);

	return 0;
}

static int __init board_temp_thermal_init(void)
{
	return board_temp_thermal_device_add();
}

static int __exit board_temp_thermal_exit(void)
{
	struct phy_dev_entry *phdev, *n;

	mutex_lock(&phy_dev_obj->phy_dev_list_mutex);
	list_for_each_entry_safe(phdev, n, &phy_dev_obj->phy_dev_list, list) {
		thermal_zone_device_unregister(phdev->tzone);
		list_del(&phdev->list);
		kfree(phdev);
	}
	mutex_unlock(&phy_dev_obj->phy_dev_list_mutex);

	return 0;
}

module_init(board_temp_thermal_init)
module_exit(board_temp_thermal_exit)

MODULE_AUTHOR("yanx.ren@intel.com");
