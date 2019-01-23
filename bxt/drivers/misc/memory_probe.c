/*
 * Copyright (C) 2016 Intel Deutschland GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/
 */

#include <asm/e820.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/memory.h>
#include <linux/memory_hotplug.h>
#include <linux/delay.h>

#define MEMPROBE_CLASS_NAME	"memprobe"

struct mem_probe_info {
	int probe_enable;
	int online_type;
	struct e820map hotplug_regions;
};

static struct mem_probe_info info;

static void e820_print_type(u32 type)
{
	switch (type) {
	case E820_RAM:
	case E820_RESERVED_KERN:
		pr_cont("usable");
		break;
	case E820_RESERVED:
		pr_cont("reserved");
		break;
	case E820_ACPI:
		pr_cont("ACPI data");
		break;
	case E820_NVS:
		pr_cont("ACPI NVS");
		break;
	case E820_UNUSABLE:
		pr_cont("unusable");
		break;
	case E820_PMEM:
	case E820_PRAM:
		pr_cont("persistent (type %u)", type);
		break;
	default:
		pr_cont("type %u", type);
		break;
	}
}

static void e820_print_info(void *map_base, char *who)
{
	int i;
	struct e820map *table = (struct e820map *)map_base;

	for (i = 0; i < table->nr_map; i++) {
		pr_cont("%s: [mem %#018Lx-%#018Lx] ", who,
		       (unsigned long long) (table->map[i].addr),
		       (unsigned long long)
		       (table->map[i].addr + table->map[i].size - 1));
		e820_print_type(table->map[i].type);
		pr_cont("\n");
	}
}

static int find_e820_range(u64 addr, struct e820map *container)
{
	int i;
	int ret = -1;

	for (i = 0; i < container->nr_map; i++) {
		if (addr == container->map[i].addr) {
			ret = i;
			goto out;
		}
	}

out:
	pr_info("<%s> found e820_range with addr: %llx, index: %d.\n", __func__,
		addr, ret);
	return ret;
}

/**
 * compare BIOS with USER of e820 table and get the retained memory range.
 */
static void compare_e820_regions(struct e820map *bios, struct e820map *user)
{
	int i, j, index;

	for (i = 0; i < bios->nr_map; i++) {
		index = find_e820_range(bios->map[i].addr, user);
		if (index == -1 && bios->map[i].type == E820_RAM) {
			j = info.hotplug_regions.nr_map;
			info.hotplug_regions.map[j].addr = bios->map[i].addr;
			info.hotplug_regions.map[j].size = bios->map[i].size;
			info.hotplug_regions.map[j].type = bios->map[i].type;
			info.hotplug_regions.nr_map++;
			pr_info("<%s> usable e820_item with addr: %llx, type: %d\n",
				__func__, info.hotplug_regions.map[j].addr,
				info.hotplug_regions.map[j].type);
		}
	}

}

static void parse_e820(void)
{
	e820_print_info(&e820, "memprobe_e820");
	e820_print_info(&e820_saved, "memprobe_e820_saved");

	compare_e820_regions(&e820_saved, &e820);
}

/**
 * probe phys memory, add memory devices of retained memory range.
 * default status is offline, since in current kernel version not support auto
 * online, which is the same behavior with: echo $(phyical_address) >
 * /sys/devices/system/memory/probe
 */
static int probe_phys_addr(u64 phys_addr)
{
	int nid, ret;

	if (phys_addr & ((PAGES_PER_SECTION << PAGE_SHIFT) - 1)) {
		ret = -EINVAL;
		pr_err("<%s> fail to check phys_addr:%llx, ret:%d\n", __func__,
			phys_addr, ret);
		return ret;
	}

	nid = memory_add_physaddr_to_nid(phys_addr);
	ret = add_memory(nid, phys_addr, PAGES_PER_SECTION << PAGE_SHIFT);
	pr_debug("<%s> add_memory addr:%llx,ret:%d,size:0x%lx,step:0x%lx.\n",
		__func__, phys_addr, ret, PAGES_PER_SECTION << PAGE_SHIFT,
		MIN_MEMORY_BLOCK_SIZE);
	if (ret)
		return ret;

	return ret;
}

/**
 * online the physical memory range, find the memory device and set device
 * online/offline, which is the same behavior with: echo online_xxx >
 * /sys/devices/system/memory/memoryXX/state
 */
static int online_phys_addr(u64 phys_addr, int online_type)
{
	int ret;
	struct mem_section *section;
	struct memory_block *mem;

	ret = lock_device_hotplug_sysfs();
	if (ret) {
		pr_err("<%s> lock_device_sysfs failed, ret:%d.\n", __func__,
			ret);
		return ret;
	}

	section = __pfn_to_section(__phys_to_pfn(phys_addr));
	mem = find_memory_block(section);
	if (!mem) {
		pr_err("<%s> find_memory_block failed, phys_addr: %llx.\n",
		__func__, phys_addr);
		ret = -EFAULT;
		goto err;
	}

	mem_hotplug_begin();

	switch (online_type) {
	case MMOP_ONLINE_KERNEL:
	case MMOP_ONLINE_MOVABLE:
	case MMOP_ONLINE_KEEP:
		mem->online_type = online_type;
		ret = device_online(&mem->dev);
		pr_debug("<%s> device_online,mem(%llx),type:%d,addr:%llx,ret:%d\n",
			__func__, (u64)mem, online_type, phys_addr, ret);
		break;
	case MMOP_OFFLINE:
		ret = device_offline(&mem->dev);
		pr_debug("<%s> device_offline,mem(%llx),addr:%llx,ret:%d\n",
			__func__, (u64)mem, phys_addr, ret);
		break;
	default:
		ret = -EINVAL;
	}

	mem_hotplug_done();
err:
	unlock_device_hotplug();

	return ret;
}

static int memprobe_state_handle(int online_type)
{
	int i, j, ret;
	u64 phys_addr, align_phys_addr;
	u64 align_size = PAGES_PER_SECTION << PAGE_SHIFT;
	u64 align_map_size, map_size;

	if (info.hotplug_regions.nr_map == 0) {
		pr_err("<%s> nr_map is empty.\n", __func__);
		return 0;
	}

	for (i = 0; i < info.hotplug_regions.nr_map; i++) {
		phys_addr = info.hotplug_regions.map[i].addr;
		align_phys_addr = (phys_addr+align_size-1)&(~(align_size-1));
		map_size = info.hotplug_regions.map[i].size;
		align_map_size = (map_size - (align_phys_addr - phys_addr));

		for (j = 0; j < (align_map_size/(MIN_MEMORY_BLOCK_SIZE)); j++) {
			if (!info.probe_enable) {
				ret = probe_phys_addr(phys_addr);
				if (ret) {
					pr_err("<%s> probe physical address:%llx failed!\n",
						__func__, phys_addr);
					continue;
				}
			}

			ret = online_phys_addr(phys_addr, online_type);
			if (ret) {
				pr_err("<%s> online:%d physical address:%llx failed!\n",
					__func__, online_type, phys_addr);
				continue;
			}

			phys_addr += MIN_MEMORY_BLOCK_SIZE;
		}
	}

	return 0;
}

static ssize_t show_memprobe_state(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char *pstate = NULL;
	int n = 0;

	if (!info.probe_enable) {
		pstate = "NA";
		goto out;
	}

	switch (info.online_type) {
	case MMOP_OFFLINE:
		pstate = "offline";
		break;
	case MMOP_ONLINE_KEEP:
		pstate = "online";
		break;
	case MMOP_ONLINE_KERNEL:
		pstate = "online_kernel";
		break;
	case MMOP_ONLINE_MOVABLE:
		pstate = "online_movable";
		break;
	default:
		pstate = "NA";
	}

out:
	n = snprintf(buf, 32, "%s\n", pstate);
	return n;
}

static ssize_t store_memprobe_state(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = count;
	int online_type;

	if (sysfs_streq(buf, "online_kernel"))
		online_type = MMOP_ONLINE_KERNEL;
	else if (sysfs_streq(buf, "online_movable"))
		online_type = MMOP_ONLINE_MOVABLE;
	else if (sysfs_streq(buf, "online"))
		online_type = MMOP_ONLINE_KEEP;
	else if (sysfs_streq(buf, "offline"))
		online_type = MMOP_OFFLINE;
	else {
		ret = -EINVAL;
		return ret;
	}

	if (info.probe_enable == 0) {
		ret = memprobe_state_handle(online_type);
		if (!ret) {
			info.probe_enable = 1;
			info.online_type = online_type;
		}
	} else
		ret = -EINVAL;

	return ret;
}

static struct bus_type memprobe_subsys = {
	.name = MEMPROBE_CLASS_NAME,
	.dev_name = MEMPROBE_CLASS_NAME,
};

static DEVICE_ATTR(state, 0644, show_memprobe_state, store_memprobe_state);

static struct attribute *memprobe_attrs[] = {
	&dev_attr_state.attr,
	NULL,
};

static struct attribute_group memprobe_attr_group = {
	.attrs = memprobe_attrs,
};

static const struct attribute_group *memprobe_attr_groups[] = {
	&memprobe_attr_group,
	NULL,
};

static int create_sysfs_mem_probe(void)
{
	int ret;

	ret = subsys_system_register(&memprobe_subsys, memprobe_attr_groups);
	pr_debug("<%s> register_subsys ret:%d.\n", __func__, ret);

	return ret;
}

static int __init mem_probe_init(void)
{
	pr_debug("====== <%s> init module ======\n", __func__);
	parse_e820();
	create_sysfs_mem_probe();
	info.online_type = MMOP_OFFLINE;

	return 0;
}

static void __exit mem_probe_exit(void)
{
	pr_debug("====== <%s> exit module ======\n", __func__);
}

module_init(mem_probe_init);
module_exit(mem_probe_exit);

MODULE_DESCRIPTION("Memory probe virtual driver");
MODULE_LICENSE("GPL");
