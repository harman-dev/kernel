/*
 * fan_cooling.c - Fan status driver
 * Copyright (c) 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/thermal.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/slab.h>

#define DUTY_CYCLE_SIZE 4

struct fan_cdev {
	unsigned short fan_state;
	unsigned long max_state;
	struct mutex fan_dev_mutex;
	struct list_head node;
	struct list_head fan_dev_list;
	struct thermal_cooling_device *thermal_cdev;
	struct kobj_attribute duty_cycle_obj;
	char duty_cycle[DUTY_CYCLE_SIZE];
};

static struct fan_cdev *fan_obj;

static int fan_get_max_state(struct thermal_cooling_device *cdev,
			unsigned long *state)
{
	mutex_lock(&fan_obj->fan_dev_mutex);
	*state = fan_obj->max_state;
	mutex_unlock(&fan_obj->fan_dev_mutex);

	return 0;
}

static int fan_get_cur_state(struct thermal_cooling_device *cdev,
			unsigned long *state)
{
	mutex_lock(&fan_obj->fan_dev_mutex);
	*state = fan_obj->fan_state;
	mutex_unlock(&fan_obj->fan_dev_mutex);

	return 0;
}

static int fan_set_cur_state(struct thermal_cooling_device *cdev,
			unsigned long state)
{
	mutex_lock(&fan_obj->fan_dev_mutex);
	fan_obj->fan_state = state;
	mutex_unlock(&fan_obj->fan_dev_mutex);

	return 0;
}

static const struct thermal_cooling_device_ops fan_cooling_ops = {
	.get_max_state = fan_get_max_state,
	.get_cur_state = fan_get_cur_state,
	.set_cur_state = fan_set_cur_state,
};

static ssize_t get_duty_cycle(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return snprintf(buf, DUTY_CYCLE_SIZE, "%s", fan_obj->duty_cycle);
}

static ssize_t set_duty_cycle(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	return snprintf(fan_obj->duty_cycle, DUTY_CYCLE_SIZE, "%s", buf);
}

static int fan_cdev_add(void)
{
	int ret;

	fan_obj = kmalloc(sizeof(struct fan_cdev), GFP_KERNEL);
	if (!fan_obj) {
	  return -ENOMEM;
	}

	mutex_init(&fan_obj->fan_dev_mutex);
	mutex_lock(&fan_obj->fan_dev_mutex);

	fan_obj->fan_state = -1;
	fan_obj->max_state = 100;
	fan_obj->thermal_cdev = thermal_cooling_device_register("Fan_ioc",
				&fan_obj->fan_state, &fan_cooling_ops);
	if (IS_ERR(fan_obj->thermal_cdev)) {
		ret = PTR_ERR(fan_obj->thermal_cdev);
		goto err;
	}

	INIT_LIST_HEAD(&fan_obj->fan_dev_list);
	list_add_tail(&fan_obj->node, &fan_obj->fan_dev_list);

	/*create a node named cur_state_info*/
	fan_obj->duty_cycle_obj.attr.name = "cur_state_info";
	fan_obj->duty_cycle_obj.attr.mode = VERIFY_OCTAL_PERMISSIONS(0644);
	fan_obj->duty_cycle_obj.show = get_duty_cycle;
	fan_obj->duty_cycle_obj.store = set_duty_cycle;

	ret = sysfs_create_file(&fan_obj->thermal_cdev->device.kobj,
			&fan_obj->duty_cycle_obj.attr);
	if (ret < 0)
		goto err;

	mutex_unlock(&fan_obj->fan_dev_mutex);

	return 0;

err:
	mutex_unlock(&fan_obj->fan_dev_mutex);
	kfree(fan_obj);

	return ret;
}

static int fan_cdev_remove(void)
{
	struct fan_cdev *n;

	mutex_lock(&fan_obj->fan_dev_mutex);
	list_for_each_entry_safe(fan_obj, n, &fan_obj->fan_dev_list, node) {
		thermal_cooling_device_unregister(fan_obj->thermal_cdev);
		list_del(&fan_obj->node);
	}
	mutex_unlock(&fan_obj->fan_dev_mutex);

	kfree(fan_obj);
	return 0;
}

static int __init fan_init(void)
{
	return fan_cdev_add();
}

static void __exit fan_exit(void)
{
	fan_cdev_remove();
}

module_init(fan_init);
module_exit(fan_exit);

MODULE_AUTHOR("yanx.ren@intel.com");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Fan thermal driver");
