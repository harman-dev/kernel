/*
 *  bxt-sst.c - DSP library functions for BXT platform
 *
 *  Copyright (C) 2015-16 Intel Corp
 *  Author:Rafal Redzimski <rafal.f.redzimski@intel.com>
 *	   Jeeja KP <jeeja.kp@intel.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/device.h>
#include <asm/cacheflush.h>

#include "../common/sst-dsp.h"
#include "../common/sst-dsp-priv.h"
#include "skl-sst-ipc.h"
#include "skl-tplg-interface.h"

#define BXT_BASEFW_TIMEOUT	3000
#define BXT_INIT_TIMEOUT	300
#define BXT_ROM_INIT_TIMEOUT	150
#define BXT_IPC_PURGE_FW	0x01004000

#define BXT_ROM_INIT		0x5
#define BXT_ADSP_SRAM0_BASE	0x80000

/* BXT SSP/I2S Registers */
#define I2S_SSC1_OFF  0x4
#define SET_SLAVE_MASK        0x03000000

/*BXT I2S Clock Gating*/
#define BXT_DSP_CLK_CTL 0x378
#define BXT_DISABLE_ALL_SSP_CLK_GT 0xFC0000
#define BXT_DISABLE_4_SSP_CLK_GT 0x3C0000
#define BXTP_NUM_I2S_PORTS      6

/* Firmware status window */
#define BXT_ADSP_FW_STATUS	BXT_ADSP_SRAM0_BASE
#define BXT_ADSP_ERROR_CODE     (BXT_ADSP_FW_STATUS + 0x4)

#define BXT_ADSP_SRAM1_BASE	0xA0000

#define BXT_INSTANCE_ID 0
#define BXT_BASE_FW_MODULE_ID 0

#define BXT_ADSP_FW_BIN_HDR_OFFSET 0x2000
#define BXT_FW_INIT_RETRY 3

static void bxt_set_ssp_slave(struct sst_dsp *ctx);

static unsigned int bxt_get_errorcode(struct sst_dsp *ctx)
{
	 return sst_dsp_shim_read(ctx, BXT_ADSP_ERROR_CODE);
}

static int
bxt_load_library(struct sst_dsp *ctx, struct skl_dfw_manifest *minfo)
{
	struct snd_dma_buffer dmab;
	struct skl_sst *skl = ctx->thread_context;
	const struct firmware *fw = NULL;
	struct firmware stripped_fw;
	int ret = 0, i, dma_id, stream_tag;

	/* library indices start from 1 to N. 0 represents base FW */
	for (i = 1; i < minfo->lib_count; i++) {
		ret = request_firmware(&fw, minfo->lib[i].name, ctx->dev);
		if (ret < 0) {
			dev_err(ctx->dev, "Request lib %s failed:%d\n",
					minfo->lib[i].name, ret);
			return ret;
		}

		if (skl->is_first_boot) {
			ret = snd_skl_parse_uuids(ctx, fw,
					BXT_ADSP_FW_BIN_HDR_OFFSET, i);
			if (ret < 0)
				goto load_library_failed;
		}

		stripped_fw.data = fw->data;
		stripped_fw.size = fw->size;
		skl_dsp_strip_extended_manifest(&stripped_fw);

		stream_tag = ctx->dsp_ops.prepare(ctx->dev, 0x40,
					stripped_fw.size, &dmab);
		if (stream_tag <= 0) {
			dev_err(ctx->dev, "Lib prepare DMA err: %x\n",
					stream_tag);
			ret = stream_tag;
			goto load_library_failed;
		}

		dma_id = stream_tag - 1;
		memcpy(dmab.area, stripped_fw.data, stripped_fw.size);

		ctx->dsp_ops.trigger(ctx->dev, true, stream_tag);
		ret = skl_sst_ipc_load_library(&skl->ipc, dma_id, i);
		if (ret < 0)
			dev_err(ctx->dev, "IPC Load Lib for %s fail: %d\n",
					minfo->lib[i].name, ret);

		ctx->dsp_ops.trigger(ctx->dev, false, stream_tag);
		ctx->dsp_ops.cleanup(ctx->dev, &dmab, stream_tag);
		release_firmware(fw);
		fw = NULL;
	}

	return ret;

load_library_failed:
	release_firmware(fw);
	return ret;
}

/*
 * First boot sequence has some extra steps. Core 0 waits for power
 * status on core 1, so power up core 1 also momentarily, keep it in
 * reset/stall and then turn it off
 */
static int sst_bxt_prepare_fw(struct sst_dsp *ctx,
			const void *fwdata, u32 fwsize)
{
	int stream_tag, ret, i;
	u32 reg;

	stream_tag = ctx->dsp_ops.prepare(ctx->dev, 0x40, fwsize, &ctx->dmab);
	if (stream_tag <= 0) {
		dev_err(ctx->dev, "Failed to prepare DMA FW loading err: %x\n",
				stream_tag);
		return stream_tag;
	}

	ctx->dsp_ops.stream_tag = stream_tag;
	memcpy(ctx->dmab.area, fwdata, fwsize);
	/* make sure FW is flushed to DDR */
	clflush_cache_range(ctx->dmab.area, fwsize);

	/* Step 1: Power up core 0 and core1 */
	ret = skl_dsp_core_power_up(ctx, SKL_DSP_CORE0_MASK |
				SKL_DSP_CORE_MASK(1));
	if (ret < 0) {
		dev_err(ctx->dev, "dsp core0/1 power up failed\n");
		goto base_fw_load_failed;
	}

	/* DSP is powered up, set all SSPs to slave mode */
	bxt_set_ssp_slave(ctx);

	/* Step 2: Purge FW request */
	sst_dsp_shim_write(ctx, SKL_ADSP_REG_HIPCI, SKL_ADSP_REG_HIPCI_BUSY |
				(BXT_IPC_PURGE_FW | ((stream_tag - 1) << 9)));

	/* Step 3: Unset core0 reset state & unstall/run core0 */
	ret = skl_dsp_start_core(ctx, SKL_DSP_CORE0_MASK);
	if (ret < 0) {
		dev_err(ctx->dev, "Start dsp core failed ret: %d\n", ret);
		ret = -EIO;
		goto base_fw_load_failed;
	}

	/* Step 4: Wait for DONE Bit */
	ret = sst_dsp_register_poll(ctx, SKL_ADSP_REG_HIPCIE,
					SKL_ADSP_REG_HIPCIE_DONE,
					SKL_ADSP_REG_HIPCIE_DONE,
					BXT_INIT_TIMEOUT, "HIPCIE Done");
	if (ret < 0) {
		dev_err(ctx->dev, "Timout for Purge Request%d\n", ret);
		goto base_fw_load_failed;
	}

	/* Step 5: power down core1 */
	ret = skl_dsp_core_power_down(ctx, SKL_DSP_CORE_MASK(1));
	if (ret < 0) {
		dev_err(ctx->dev, "dsp core1 power down failed\n");
		goto base_fw_load_failed;
	}

	/* Step 6: Enable Interrupt */
	skl_ipc_int_enable(ctx);
	skl_ipc_op_int_enable(ctx);

	/* Step 7: Wait for ROM init */
	ret = sst_dsp_register_poll(ctx, BXT_ADSP_FW_STATUS, SKL_FW_STS_MASK,
			SKL_FW_INIT, BXT_ROM_INIT_TIMEOUT, "ROM Load");
	if (ret < 0) {
		dev_err(ctx->dev, "Timeout for ROM init, ret:%d\n", ret);
		goto base_fw_load_failed;
	}

	return ret;

base_fw_load_failed:
	ctx->dsp_ops.cleanup(ctx->dev, &ctx->dmab, stream_tag);
	skl_dsp_core_power_down(ctx, SKL_DSP_CORE_MASK(1));
	skl_dsp_disable_core(ctx, SKL_DSP_CORE0_MASK);
	return ret;
}

static int sst_transfer_fw_host_dma(struct sst_dsp *ctx)
{
	int ret;

	ctx->dsp_ops.trigger(ctx->dev, true, ctx->dsp_ops.stream_tag);
	ret = sst_dsp_register_poll(ctx, BXT_ADSP_FW_STATUS, SKL_FW_STS_MASK,
			BXT_ROM_INIT, BXT_BASEFW_TIMEOUT, "Firmware boot");

	ctx->dsp_ops.trigger(ctx->dev, false, ctx->dsp_ops.stream_tag);
	ctx->dsp_ops.cleanup(ctx->dev, &ctx->dmab, ctx->dsp_ops.stream_tag);

	return ret;
}

#define GET_SSP_BASE(N) (N > 4 ? 0x2000 : 0x4000)

static void bxt_set_ssp_slave(struct sst_dsp *ctx)
{
	u32 reg;
	u32 mask, i2s_base_addr;
	int i;

	if (BXTP_NUM_I2S_PORTS == 4)
		mask = BXT_DISABLE_4_SSP_CLK_GT;
	else
		mask = BXT_DISABLE_ALL_SSP_CLK_GT;

	/* disable clock gating on all SSPs */
	reg = sst_dsp_shim_read_unlocked(ctx, BXT_DSP_CLK_CTL);
	reg |= mask;
	sst_dsp_shim_write_unlocked(ctx, BXT_DSP_CLK_CTL, reg);

	/* set all SSPs to slave */
	i2s_base_addr = GET_SSP_BASE(BXTP_NUM_I2S_PORTS);
	for (i = 0; i < BXTP_NUM_I2S_PORTS; i++) {
		reg = sst_dsp_shim_read_unlocked(ctx,
				(i2s_base_addr + (i * 0x1000) + I2S_SSC1_OFF));
		reg |= SET_SLAVE_MASK;
		sst_dsp_shim_write_unlocked(ctx,
			(i2s_base_addr + (i * 0x1000) + I2S_SSC1_OFF), reg);
	}

	/* re-enable clock gating */
	reg = sst_dsp_shim_read_unlocked(ctx, BXT_DSP_CLK_CTL);
	reg &= ~mask;
	sst_dsp_shim_write_unlocked(ctx, BXT_DSP_CLK_CTL, reg);
}

static int bxt_load_base_firmware(struct sst_dsp *ctx)
{
	struct firmware stripped_fw;
	struct skl_sst *skl = ctx->thread_context;
	int ret, i;

	ret = request_firmware(&ctx->fw, ctx->fw_name, ctx->dev);
	if (ret < 0) {
		dev_err(ctx->dev, "Request firmware failed %d\n", ret);
		goto sst_load_base_firmware_failed;
	}

	/* check for extended manifest */
	if (ctx->fw == NULL)
		goto sst_load_base_firmware_failed;

	/* prase uuids on first boot */
	if (skl->is_first_boot) {
		ret = snd_skl_parse_uuids(ctx, ctx->fw, BXT_ADSP_FW_BIN_HDR_OFFSET, 0);
		if (ret < 0)
			goto sst_load_base_firmware_failed;
	}

	stripped_fw.data = ctx->fw->data;
	stripped_fw.size = ctx->fw->size;
	skl_dsp_strip_extended_manifest(&stripped_fw);

	for (i = 0; i < BXT_FW_INIT_RETRY; i++) {
		ret = sst_bxt_prepare_fw(ctx, stripped_fw.data, stripped_fw.size);
		if (ret < 0) {
			dev_err(ctx->dev, "Error code=0x%x: FW status=0x%x\n",
				sst_dsp_shim_read(ctx, BXT_ADSP_ERROR_CODE),
				sst_dsp_shim_read(ctx, BXT_ADSP_FW_STATUS));

			dev_err(ctx->dev, "Itertion %d Core En/ROM load fail:%d\n", i,ret);
			continue;
		}
		dev_err(ctx->dev, "Itertion %d ROM load Success:%d,%d\n", i,ret);

		ret = sst_transfer_fw_host_dma(ctx);
		if (ret < 0) {
			dev_err(ctx->dev, "Itertion %d Transfer firmware failed %d\n", i,ret);
			dev_info(ctx->dev, "Error code=0x%x: FW status=0x%x\n",
				sst_dsp_shim_read(ctx, BXT_ADSP_ERROR_CODE),
				sst_dsp_shim_read(ctx, BXT_ADSP_FW_STATUS));

			skl_dsp_core_power_down(ctx, SKL_DSP_CORE_MASK(1));
			skl_dsp_disable_core(ctx, SKL_DSP_CORE0_MASK);
			continue;
		}
		dev_err(ctx->dev, "Itertion %d FW transfer Success:%d,%d\n", i,ret);

		if (ret == 0)
			break;
	}

        if (ret < 0) {
		dev_err(ctx->dev, "Firmware download failed\n");
		goto sst_load_base_firmware_failed;
	} else {
		dev_dbg(ctx->dev, "Firmware download successful\n");
		ret = wait_event_timeout(skl->boot_wait, skl->boot_complete,
					msecs_to_jiffies(SKL_IPC_BOOT_MSECS));
		if (ret == 0) {
			dev_err(ctx->dev, "DSP boot fail, FW Ready timeout\n");
			skl_dsp_disable_core(ctx, SKL_DSP_CORE0_MASK);
			ret = -EIO;
		} else {
			ret = 0;
			skl->fw_loaded = true;
			/* set dma config if available */
			if (skl->manifest.cfg.dmacfg.size) {
				skl_ipc_set_dma_cfg(&skl->ipc,
					BXT_INSTANCE_ID,
					BXT_BASE_FW_MODULE_ID,
					(u32 *)(&skl->manifest.cfg.dmacfg));
			}
		}
	}

sst_load_base_firmware_failed:
	release_firmware(ctx->fw);
	return ret;
}

static int bxt_set_dsp_D0(struct sst_dsp *ctx, unsigned int core_id)
{
	struct skl_sst *skl = ctx->thread_context;
	int ret;
	struct skl_ipc_dxstate_info dx;
	unsigned int core_mask = SKL_DSP_CORE_MASK(core_id);
	struct skl_dfw_manifest *minfo = &skl->manifest;

	if (skl->fw_loaded == false) {
		skl->boot_complete = false;
		ret = bxt_load_base_firmware(ctx);
		if (ret < 0) {
			dev_err(ctx->dev, "reload fw failed: %d\n", ret);
			return ret;
		}

		if (minfo->lib_count > 1) {
			ret = bxt_load_library(ctx, minfo);
			if (ret < 0) {
				dev_err(ctx->dev, "reload libs failed: %d\n", ret);
				return ret;
			}
		}
		return ret;
	}

	/* If core 0 is being turned on, turn on core 1 as well */
	if (core_id == SKL_DSP_CORE0_ID)
		ret = skl_dsp_core_power_up(ctx, core_mask |
				SKL_DSP_CORE_MASK(1));
	else
		ret = skl_dsp_core_power_up(ctx, core_mask);

	if (ret < 0)
		goto err;

	if (core_id == SKL_DSP_CORE0_ID) {

		/* set all SSPs to slave mode */
		bxt_set_ssp_slave(ctx);

		/*
		 * Enable interrupt after SPA is set and before
		 * DSP is unstalled
		 */
		skl_ipc_int_enable(ctx);
		skl_ipc_op_int_enable(ctx);
		skl->boot_complete = false;
	}

	ret = skl_dsp_start_core(ctx, core_mask);
	if (ret < 0)
		goto err;

	if (core_id == SKL_DSP_CORE0_ID) {
		ret = wait_event_timeout(skl->boot_wait,
				skl->boot_complete,
				msecs_to_jiffies(SKL_IPC_BOOT_MSECS));

	/* If core 1 was turned on for booting core 0, turn it off */
		skl_dsp_core_power_down(ctx, SKL_DSP_CORE_MASK(1));
		if (ret == 0) {
			dev_err(ctx->dev, "%s: DSP boot timeout\n", __func__);
			dev_err(ctx->dev, "Error code=0x%x: FW status=0x%x\n",
				sst_dsp_shim_read(ctx, BXT_ADSP_ERROR_CODE),
				sst_dsp_shim_read(ctx, BXT_ADSP_FW_STATUS));
			dev_err(ctx->dev, "Failed to set core0 to D0 state\n");
			ret = -EIO;
			goto err;
		}
	}

	/* Tell FW if additional core in now On */

	if (core_id != SKL_DSP_CORE0_ID) {
		dx.core_mask = core_mask;
		dx.dx_mask = core_mask;

		ret = skl_ipc_set_dx(&skl->ipc, BXT_INSTANCE_ID,
					BXT_BASE_FW_MODULE_ID, &dx);
		if (ret < 0) {
			dev_err(ctx->dev, "IPC set_dx for core %d fail: %d\n",
								core_id, ret);
			goto err;
		}
	} else {
		/* set dma config if available for CORE0 boot only */
		if (skl->manifest.cfg.dmacfg.size) {
			skl_ipc_set_dma_cfg(&skl->ipc, BXT_INSTANCE_ID,
				BXT_BASE_FW_MODULE_ID,
				(u32 *)(&skl->manifest.cfg.dmacfg));
		}
	}

	skl->cores.state[core_id] = SKL_DSP_RUNNING;
	return 0;
err:
	if (core_id == SKL_DSP_CORE0_ID)
		core_mask |= SKL_DSP_CORE_MASK(1);
	skl_dsp_disable_core(ctx, core_mask);

	return ret;
}

static int bxt_set_dsp_D3(struct sst_dsp *ctx, unsigned int core_id)
{
	int ret;
	struct skl_ipc_dxstate_info dx;
	struct skl_sst *skl = ctx->thread_context;
	unsigned int core_mask = SKL_DSP_CORE_MASK(core_id);

	dx.core_mask = core_mask;
	dx.dx_mask = SKL_IPC_D3_MASK;

	dev_dbg(ctx->dev, "core mask=%x dx_mask=%x\n",
			dx.core_mask, dx.dx_mask);

	ret = skl_ipc_set_dx(&skl->ipc, BXT_INSTANCE_ID,
				BXT_BASE_FW_MODULE_ID, &dx);
	if (ret < 0)
		dev_err(ctx->dev,
		"Failed to set DSP to D3:core id = %d;Continue reset\n",
		core_id);

	ret = skl_dsp_disable_core(ctx, core_mask);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to disable core %d\n", ret);
		return ret;
	}
	skl->cores.state[core_id] = SKL_DSP_RESET;
	return 0;
}

static struct skl_dsp_fw_ops bxt_fw_ops = {
	.set_state_D0 = bxt_set_dsp_D0,
	.set_state_D3 = bxt_set_dsp_D3,
	.load_fw = bxt_load_base_firmware,
	.get_fw_errcode = bxt_get_errorcode,
	.load_library = bxt_load_library,
};

static struct sst_ops skl_ops = {
	.irq_handler = skl_dsp_sst_interrupt,
	.write = sst_shim32_write,
	.read = sst_shim32_read,
	.ram_read = sst_memcpy_fromio_32,
	.ram_write = sst_memcpy_toio_32,
	.free = skl_dsp_free,
};

static struct sst_dsp_device skl_dev = {
	.thread = skl_dsp_irq_thread_handler,
	.ops = &skl_ops,
};

int bxt_sst_dsp_init(struct device *dev, void __iomem *mmio_base, int irq,
			const char *fw_name, struct skl_dsp_loader_ops dsp_ops,
			struct skl_sst **dsp)
{
	struct skl_sst *skl;
	struct sst_dsp *sst;
	int ret;

	skl = devm_kzalloc(dev, sizeof(*skl), GFP_KERNEL);
	if (skl == NULL)
		return -ENOMEM;

	skl->dev = dev;
	skl_dev.thread_context = skl;
	INIT_LIST_HEAD(&skl->uuid_list);

	skl->dsp = skl_dsp_ctx_init(dev, &skl_dev, irq);
	if (!skl->dsp) {
		dev_err(skl->dev, "skl_dsp_ctx_init failed\n");
		return -ENODEV;
	}

	sst = skl->dsp;
	sst->fw_name = fw_name;
	sst->dsp_ops = dsp_ops;
	sst->fw_ops = bxt_fw_ops;
	sst->addr.lpe = mmio_base;
	sst->addr.shim = mmio_base;

	/* Register the ISR */
	ret = request_threaded_irq(sst->irq, sst->ops->irq_handler,
		skl_dev.thread, IRQF_SHARED, "AudioDSP", sst);
	if (ret) {
		dev_err(sst->dev, "unable to grab threaded IRQ %d, disabling device\n",
			       sst->irq);
		return ret;
	}

	sst_dsp_mailbox_init(sst, (BXT_ADSP_SRAM0_BASE + SKL_ADSP_W0_STAT_SZ),
			SKL_ADSP_W0_UP_SZ, BXT_ADSP_SRAM1_BASE, SKL_ADSP_W1_SZ);

	INIT_LIST_HEAD(&sst->module_list);
	ret = skl_ipc_init(dev, skl);
	if (ret)
		return ret;

	skl->cores.count = 2;
	skl->boot_complete = false;
	init_waitqueue_head(&skl->boot_wait);
	skl->is_first_boot = true;

	if (dsp)
		*dsp = skl;

	return 0;
}
EXPORT_SYMBOL_GPL(bxt_sst_dsp_init);

int bxt_sst_init_fw(struct device *dev, struct skl_sst *ctx)
{
	int ret;
	struct sst_dsp *sst = ctx->dsp;

	ret = sst->fw_ops.load_fw(sst);
	if (ret < 0) {
		dev_err(dev, "Load base fw failed: %x\n", ret);
		return ret;
	}

	skl_dsp_init_core_state(sst);

	if (ctx->manifest.lib_count > 1) {
		ret = sst->fw_ops.load_library(sst, &ctx->manifest);
		if (ret < 0) {
			dev_err(dev, "Load Library failed : %x\n", ret);
			return ret;
		}
	}
	ctx->is_first_boot = false;

	return 0;
}
EXPORT_SYMBOL_GPL(bxt_sst_init_fw);

void bxt_sst_dsp_cleanup(struct device *dev, struct skl_sst *ctx)
{
	skl_freeup_uuid_list(ctx);
	skl_ipc_free(&ctx->ipc);
	ctx->dsp->cl_dev.ops.cl_cleanup_controller(ctx->dsp);

	if (ctx->dsp->addr.lpe)
		iounmap(ctx->dsp->addr.lpe);

	ctx->dsp->ops->free(ctx->dsp);
}
EXPORT_SYMBOL_GPL(bxt_sst_dsp_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel Broxton IPC driver");
