/*
 * Copyright (c) 2014--2017 Intel Corporation.
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

#include <asm/cacheflush.h>
#include <linux/io.h>

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "intel-ipu4-bus.h"
#include "intel-ipu4-dma.h"
#include "intel-ipu4-mmu.h"
#include "intel-ipu4-wrapper.h"

struct wrapper_base {
	void __iomem *sys_base;
	const struct dma_map_ops *ops;
	spinlock_t lock;
	struct list_head buffers;
	uint32_t css_map_done;
	struct device *dev;
};

struct wrapper_base isys;
struct wrapper_base psys;

struct my_css_memory_buffer_item {
	struct list_head list;
	dma_addr_t iova;
	unsigned long *addr;
	size_t bytes;
	struct dma_attrs attrs;
};

unsigned long long get_hrt_base_address(void)
{
	return 0;
}
EXPORT_SYMBOL_GPL(get_hrt_base_address);

static struct wrapper_base *get_mem_sub_system(int mmid)
{
	if (mmid == ISYS_MMID)
		return &isys;

	if (mmid == PSYS_MMID)
		return &psys;
	BUG();
}

static struct wrapper_base *get_sub_system(int ssid)
{
	if (ssid == ISYS_SSID)
		return &isys;

	if (ssid == PSYS_SSID)
		return &psys;
	BUG();
}

/*
 * Subsystem access functions to access IUNIT MMIO space
 */
static void *host_addr(int ssid, u32 addr)
{
	if (ISYS_SSID == ssid)
		return isys.sys_base + addr;
	else if (PSYS_SSID == ssid)
		return psys.sys_base + addr;
	/*
	 * Calling BUG is a bit brutal but better to capture wrong register
	 * accesses immediately. We have no way to return an error here.
	 */
	BUG();
}

void vied_subsystem_store_32(int ssid,
			     u32 addr, uint32_t data)
{
	writel(data, host_addr(ssid, addr));
}
EXPORT_SYMBOL_GPL(vied_subsystem_store_32);

void vied_subsystem_store_16(int ssid,
			     u32 addr, uint16_t data)
{
	writew(data, host_addr(ssid, addr));
}
EXPORT_SYMBOL_GPL(vied_subsystem_store_16);

void vied_subsystem_store_8(int ssid,
			     u32 addr, uint8_t data)
{
	writeb(data, host_addr(ssid, addr));
}
EXPORT_SYMBOL_GPL(vied_subsystem_store_8);

void vied_subsystem_store(int ssid,
			  u32 addr,
			  const void *data, unsigned int size)
{
	void *dst = host_addr(ssid, addr);

	dev_dbg(get_sub_system(ssid)->dev, "access: %s 0x%x size: %d\n",
		__func__, addr, size);

	for (; size >= sizeof(uint32_t); size -= sizeof(uint32_t),
		     dst += sizeof(uint32_t), data += sizeof(uint32_t)) {
		writel(*(uint32_t *)data, dst);
	}
	if (size >= sizeof(uint16_t)) {
		writew(*(uint16_t *)data, dst);
		size -= sizeof(uint16_t), dst += sizeof(uint16_t),
			data += sizeof(uint16_t);
	}
	if (size)
		writeb(*(uint8_t *)data, dst);

}
EXPORT_SYMBOL_GPL(vied_subsystem_store);

uint32_t vied_subsystem_load_32(int ssid,
				u32 addr)
{
	return readl(host_addr(ssid, addr));
}
EXPORT_SYMBOL_GPL(vied_subsystem_load_32);

uint16_t vied_subsystem_load_16(int ssid,
				u32 addr)
{
	return readw(host_addr(ssid, addr));
}
EXPORT_SYMBOL_GPL(vied_subsystem_load_16);

uint8_t vied_subsystem_load_8(int ssid,
				u32 addr)
{
	return readb(host_addr(ssid, addr));
}
EXPORT_SYMBOL_GPL(vied_subsystem_load_8);

void vied_subsystem_load(int ssid,
			 u32 addr,
			 void *data, unsigned int size)
{
	void *src = host_addr(ssid, addr);

	dev_dbg(get_sub_system(ssid)->dev, "access: %s 0x%x size: %d\n",
		__func__, addr, size);

	for (; size >= sizeof(uint32_t); size -= sizeof(uint32_t),
		     src += sizeof(uint32_t), data += sizeof(uint32_t))
		*(uint32_t *)data = readl(src);
	if (size >= sizeof(uint16_t)) {
		*(uint16_t *)data = readw(src);
		size -= sizeof(uint16_t), src += sizeof(uint16_t),
			data += sizeof(uint16_t);
	}
	if (size)
		*(uint8_t *)data = readb(src);
}
EXPORT_SYMBOL_GPL(vied_subsystem_load);
/*
 * Initialize base address for subsystem
 */
void vied_subsystem_access_initialize(int system)
{

}
EXPORT_SYMBOL_GPL(vied_subsystem_access_initialize);

/*
 * Shared memory access codes written by Dash Biswait,
 * copied from FPGA environment
 */

/**
 * \brief Initialize the shared memory interface administration on the host.
 * \param mmid: id of ddr memory
 * \param host_ddr_addr: physical address of memory as seen from host
 * \param memory_size: size of ddr memory in bytes
 * \param ps: size of page in bytes (for instance 4096)
 */
int shared_memory_allocation_initialize(int mmid, u64 host_ddr_addr,
					size_t memory_size, size_t ps)
{
	return 0;
}
EXPORT_SYMBOL_GPL(shared_memory_allocation_initialize);

/**
 * \brief De-initialize the shared memory interface administration on the host.
 *
 */
void shared_memory_allocation_uninitialize(int mmid)
{
}
EXPORT_SYMBOL_GPL(shared_memory_allocation_uninitialize);

/**
 * \brief Initialize the shared memory interface administration on the host.
 * \param ssid: id of subsystem
 * \param mmid: id of ddr memory
 * \param mmu_ps: size of page in bits
 * \param mmu_pnrs: page numbers
 * \param ddr_addr: base address
 * \param inv_tlb: invalidate tbl
 * \param sbt: set l1 base address
 */
int shared_memory_map_initialize(int ssid, int mmid, size_t mmu_ps,
				 size_t mmu_pnrs, u64 ddr_addr,
				 int inv_tlb, int sbt)
{
	return 0;
}
EXPORT_SYMBOL_GPL(shared_memory_map_initialize);

/**
 * \brief De-initialize the shared memory interface administration on the host.
 */
void shared_memory_map_uninitialize(int ssid, int mmid)
{
}
EXPORT_SYMBOL_GPL(shared_memory_map_uninitialize);

static uint8_t alloc_cookie;

/**
 * \brief Allocate (DDR) shared memory space and return a host virtual address.
 * \Returns NULL when insufficient memory available
 */
u64 shared_memory_alloc(int mmid, size_t bytes)
{
	struct wrapper_base *mine = get_mem_sub_system(mmid);
	struct my_css_memory_buffer_item *buf;
	unsigned long flags;

	dev_dbg(mine->dev, "%s: in, size: %zu\n", __func__, bytes);

	if (!bytes)
		return (unsigned long)&alloc_cookie;

	might_sleep();

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return 0;

	/*alloc using intel_ipu4 dma driver*/
	buf->bytes = PAGE_ALIGN(bytes);

	buf->addr = dma_alloc_attrs(mine->dev, buf->bytes, &buf->iova,
				    GFP_KERNEL, NULL);
	if (!buf->addr) {
		kfree(buf);
		return 0;
	}

	spin_lock_irqsave(&mine->lock, flags);
	list_add(&buf->list, &mine->buffers);
	spin_unlock_irqrestore(&mine->lock, flags);

	return (unsigned long)buf->addr;
}
EXPORT_SYMBOL_GPL(shared_memory_alloc);

/**
 * \brief Free (DDR) shared memory space.
 */
void shared_memory_free(int mmid, u64 addr)
{
	struct wrapper_base *mine = get_mem_sub_system(mmid);
	struct my_css_memory_buffer_item *buf = NULL;
	unsigned long flags;

	if ((void *)addr == &alloc_cookie)
		return;

	might_sleep();

	dev_dbg(mine->dev, "looking for iova %8.8llx\n", addr);

	spin_lock_irqsave(&mine->lock, flags);
	list_for_each_entry(buf, &mine->buffers, list) {
		dev_dbg(mine->dev, "buffer addr %8.8lx\n", (long)buf->addr);
		if ((long)buf->addr != addr)
			continue;

		dev_dbg(mine->dev, "found it!\n");
		list_del(&buf->list);
		spin_unlock_irqrestore(&mine->lock, flags);
		dma_free_attrs(mine->dev, buf->bytes, buf->addr,
			       buf->iova, &buf->attrs);
		kfree(buf);
		return;
	}
	dev_warn(mine->dev, "Can't find mem object %8.8llx\n", addr);
	spin_unlock_irqrestore(&mine->lock, flags);

}
EXPORT_SYMBOL_GPL(shared_memory_free);

/**
 * \brief Convert a host virtual address to a CSS virtual address and
 * \update the MMU.
 */
u32 shared_memory_map(int ssid, int mmid, u64 addr)
{
	struct wrapper_base *mine = get_mem_sub_system(mmid);
	struct my_css_memory_buffer_item *buf = NULL;
	unsigned long flags;

	if ((void *)addr == &alloc_cookie)
		return 0;

	spin_lock_irqsave(&mine->lock, flags);
	list_for_each_entry(buf, &mine->buffers, list) {
		dev_dbg(mine->dev, "%s %8.8lx\n", __func__, (long)buf->addr);
		if ((long)buf->addr != addr)
			continue;

		dev_dbg(mine->dev, "mapped!!\n");
		spin_unlock_irqrestore(&mine->lock, flags);
		return buf->iova;
	}
	dev_err(mine->dev, "Can't find mapped object %8.8llx\n", addr);
	spin_unlock_irqrestore(&mine->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(shared_memory_map);

/**
 * \brief Free a CSS virtual address and update the MMU.
 */
void shared_memory_unmap(int ssid, int mmid, u32 addr)
{
}
EXPORT_SYMBOL_GPL(shared_memory_unmap);

/**
 * \brief Store a byte into (DDR) shared memory space using a host
 * \virtual address
 */
void shared_memory_store_8(int mmid, u64 addr,
			   uint8_t data)
{
	dev_dbg(get_mem_sub_system(mmid)->dev,
		"access: %s: Enter addr = 0x%llx data = 0x%x\n",
		__func__, addr, data);

	*((uint8_t *) addr) = data;
	/*Invalidate the cache lines to flush the content to ddr.*/
	clflush_cache_range((void *)addr, sizeof(uint8_t));
}
EXPORT_SYMBOL_GPL(shared_memory_store_8);

/**
 * \brief Store a 16-bit word into (DDR) shared memory space using a host
 * \virtual address
 */
void shared_memory_store_16(int mmid, u64 addr,
			    uint16_t data)
{
	dev_dbg(get_mem_sub_system(mmid)->dev,
		"access: %s: Enter addr = 0x%llx data = 0x%x\n",
		__func__, addr, data);

	*((uint16_t *) addr) = data;
	/*Invalidate the cache lines to flush the content to ddr. */
	clflush_cache_range((void *)addr, sizeof(uint16_t));
}
EXPORT_SYMBOL_GPL(shared_memory_store_16);

/**
 * \brief Store a 32-bit word into (DDR) shared memory space using a host
 * \virtual address
 */
void shared_memory_store_32(int mmid, u64 addr,
			    uint32_t data)
{
	dev_dbg(get_mem_sub_system(mmid)->dev,
		"access: %s: Enter addr = 0x%llx data = 0x%x\n",
		__func__, addr,  data);

	*((uint32_t *) addr) = data;
	/* Invalidate the cache lines to flush the content to ddr. */
	clflush_cache_range((void *)addr, sizeof(uint32_t));
}
EXPORT_SYMBOL_GPL(shared_memory_store_32);

/**
 * \brief Store a number of bytes into (DDR) shared memory space using a host
 * \virtual address
 */
void shared_memory_store(int mmid, u64 addr,
			 const void *data, size_t bytes)
{
	dev_dbg(get_mem_sub_system(mmid)->dev,
		"access: %s: Enter addr = 0x%lx bytes = 0x%lx\n", __func__,
		(unsigned long)addr, bytes);

	if (!data)
		dev_err(get_mem_sub_system(mmid)->dev,
			"%s: data ptr is null\n", __func__);
	else {
		const uint8_t *pdata = data;
		uint8_t *paddr = (uint8_t *) addr;
		size_t i = 0;

		for (; i < bytes; ++i)
			*paddr++ = *pdata++;

		/* Invalidate the cache lines to flush the content to ddr. */
		clflush_cache_range((void *)addr, bytes);
	}
}
EXPORT_SYMBOL_GPL(shared_memory_store);

/**
 * \brief Set a number of bytes of (DDR) shared memory space to 0 using a host
 * \virtual address
 */
void shared_memory_zero(int mmid, u64 addr,
			size_t bytes)
{
	dev_dbg(get_mem_sub_system(mmid)->dev,
		"access: %s: Enter addr = 0x%llx data = 0x%lu\n",
		__func__, addr, bytes);

	memset((void *)addr, 0, bytes);
	clflush_cache_range((void *)addr, bytes);
}
EXPORT_SYMBOL_GPL(shared_memory_zero);

/**
 * \brief Load a byte from (DDR) shared memory space using a host
 * \virtual address
 */
uint8_t shared_memory_load_8(int mmid, u64 addr)
{
	uint8_t data = 0;

	dev_dbg(get_mem_sub_system(mmid)->dev,
		"access: %s: Enter addr = 0x%llx\n", __func__, addr);

	/* Invalidate the cache lines to flush the content to ddr. */
	clflush_cache_range((void *)addr, sizeof(uint8_t));
	data = *(uint8_t *)addr;
	return data;
}
EXPORT_SYMBOL_GPL(shared_memory_load_8);

/**
 * \brief Load a 16-bit word from (DDR) shared memory space using a host
 * \virtual address
 */
uint16_t shared_memory_load_16(int mmid, u64 addr)
{
	uint16_t data = 0;

	dev_dbg(get_mem_sub_system(mmid)->dev,
		"access: %s: Enter addr = 0x%llx\n", __func__, addr);

	/* Invalidate the cache lines to flush the content to ddr. */
	clflush_cache_range((void *)addr, sizeof(uint16_t));
	data = *(uint16_t *) addr;
	return data;
}
EXPORT_SYMBOL_GPL(shared_memory_load_16);

/**
 * \brief Load a 32-bit word from (DDR) shared memory space using a host
 * \virtual address
 */
uint32_t shared_memory_load_32(int mmid, u64 addr)
{
	uint32_t data = 0;

	dev_dbg(get_mem_sub_system(mmid)->dev,
		"access: %s: Enter addr = 0x%llx\n", __func__, addr);

	/* Invalidate the cache lines to flush the content to ddr. */
	clflush_cache_range((void *)addr, sizeof(uint32_t));
	data = *(uint32_t *) addr;
	return data;
}
EXPORT_SYMBOL_GPL(shared_memory_load_32);

/**
 * \brief Load a number of bytes from (DDR) shared memory space using a host
 * \virtual address
 */
void shared_memory_load(int mmid, u64 addr,
			void *data, size_t bytes)
{
	dev_dbg(get_mem_sub_system(mmid)->dev,
		"access: %s: Enter addr = 0x%lx bytes = 0x%lx\n", __func__,
		(unsigned long)addr, bytes);

	if (!data)
		dev_err(get_mem_sub_system(mmid)->dev,
			"%s: data ptr is null\n", __func__);

	else {
		uint8_t *pdata = data;
		uint8_t *paddr = (uint8_t *) addr;
		size_t i = 0;

		/* Invalidate the cache lines to flush the content to ddr. */
		clflush_cache_range((void *)addr, bytes);
		for (; i < bytes; ++i)
			*pdata++ = *paddr++;
	}
}
EXPORT_SYMBOL_GPL(shared_memory_load);

static int __init init_wrapper(void)
{
	INIT_LIST_HEAD(&isys.buffers);
	spin_lock_init(&isys.lock);

	INIT_LIST_HEAD(&psys.buffers);
	spin_lock_init(&psys.lock);
	return 0;
}

static void __exit exit_wrapper(void)
{
	return;
}

/*
 * Wrapper driver set base address for library use
 */
void intel_ipu4_wrapper_init(int mmid, struct device *dev,
			     void __iomem *base)
{
	struct wrapper_base *sys = get_mem_sub_system(mmid);

	sys->dev = dev;
	sys->sys_base = base;
}
EXPORT_SYMBOL_GPL(intel_ipu4_wrapper_init);

module_init(init_wrapper);
module_exit(exit_wrapper);
MODULE_AUTHOR("Jouni Ukkonen <jouni.ukkonen@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CSS wrapper");
