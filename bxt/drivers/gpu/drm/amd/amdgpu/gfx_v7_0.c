/*
 * Copyright 2014 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#include <linux/firmware.h>
#include "drmP.h"
#include "amdgpu.h"
#include "amdgpu_ih.h"
#include "amdgpu_gfx.h"
#include "cikd.h"
#include "cik.h"
#include "atom.h"
#include "amdgpu_ucode.h"
#include "clearstate_ci.h"

#include "uvd/uvd_4_2_d.h"

#include "dce/dce_8_0_d.h"
#include "dce/dce_8_0_sh_mask.h"

#include "bif/bif_4_1_d.h"
#include "bif/bif_4_1_sh_mask.h"

#include "gca/gfx_7_0_d.h"
#include "gca/gfx_7_2_enum.h"
#include "gca/gfx_7_2_sh_mask.h"

#include "gmc/gmc_7_0_d.h"
#include "gmc/gmc_7_0_sh_mask.h"

#include "oss/oss_2_0_d.h"
#include "oss/oss_2_0_sh_mask.h"

#define GFX7_NUM_GFX_RINGS     1
#define GFX7_NUM_COMPUTE_RINGS 8

static void gfx_v7_0_set_ring_funcs(struct amdgpu_device *adev);
static void gfx_v7_0_set_irq_funcs(struct amdgpu_device *adev);
static void gfx_v7_0_set_gds_init(struct amdgpu_device *adev);
int gfx_v7_0_get_cu_info(struct amdgpu_device *, struct amdgpu_cu_info *);

MODULE_FIRMWARE("radeon/bonaire_pfp.bin");
MODULE_FIRMWARE("radeon/bonaire_me.bin");
MODULE_FIRMWARE("radeon/bonaire_ce.bin");
MODULE_FIRMWARE("radeon/bonaire_rlc.bin");
MODULE_FIRMWARE("radeon/bonaire_mec.bin");

MODULE_FIRMWARE("radeon/hawaii_pfp.bin");
MODULE_FIRMWARE("radeon/hawaii_me.bin");
MODULE_FIRMWARE("radeon/hawaii_ce.bin");
MODULE_FIRMWARE("radeon/hawaii_rlc.bin");
MODULE_FIRMWARE("radeon/hawaii_mec.bin");

MODULE_FIRMWARE("radeon/kaveri_pfp.bin");
MODULE_FIRMWARE("radeon/kaveri_me.bin");
MODULE_FIRMWARE("radeon/kaveri_ce.bin");
MODULE_FIRMWARE("radeon/kaveri_rlc.bin");
MODULE_FIRMWARE("radeon/kaveri_mec.bin");
MODULE_FIRMWARE("radeon/kaveri_mec2.bin");

MODULE_FIRMWARE("radeon/kabini_pfp.bin");
MODULE_FIRMWARE("radeon/kabini_me.bin");
MODULE_FIRMWARE("radeon/kabini_ce.bin");
MODULE_FIRMWARE("radeon/kabini_rlc.bin");
MODULE_FIRMWARE("radeon/kabini_mec.bin");

MODULE_FIRMWARE("radeon/mullins_pfp.bin");
MODULE_FIRMWARE("radeon/mullins_me.bin");
MODULE_FIRMWARE("radeon/mullins_ce.bin");
MODULE_FIRMWARE("radeon/mullins_rlc.bin");
MODULE_FIRMWARE("radeon/mullins_mec.bin");

static const struct amdgpu_gds_reg_offset amdgpu_gds_reg_offset[] =
{
	{mmGDS_VMID0_BASE, mmGDS_VMID0_SIZE, mmGDS_GWS_VMID0, mmGDS_OA_VMID0},
	{mmGDS_VMID1_BASE, mmGDS_VMID1_SIZE, mmGDS_GWS_VMID1, mmGDS_OA_VMID1},
	{mmGDS_VMID2_BASE, mmGDS_VMID2_SIZE, mmGDS_GWS_VMID2, mmGDS_OA_VMID2},
	{mmGDS_VMID3_BASE, mmGDS_VMID3_SIZE, mmGDS_GWS_VMID3, mmGDS_OA_VMID3},
	{mmGDS_VMID4_BASE, mmGDS_VMID4_SIZE, mmGDS_GWS_VMID4, mmGDS_OA_VMID4},
	{mmGDS_VMID5_BASE, mmGDS_VMID5_SIZE, mmGDS_GWS_VMID5, mmGDS_OA_VMID5},
	{mmGDS_VMID6_BASE, mmGDS_VMID6_SIZE, mmGDS_GWS_VMID6, mmGDS_OA_VMID6},
	{mmGDS_VMID7_BASE, mmGDS_VMID7_SIZE, mmGDS_GWS_VMID7, mmGDS_OA_VMID7},
	{mmGDS_VMID8_BASE, mmGDS_VMID8_SIZE, mmGDS_GWS_VMID8, mmGDS_OA_VMID8},
	{mmGDS_VMID9_BASE, mmGDS_VMID9_SIZE, mmGDS_GWS_VMID9, mmGDS_OA_VMID9},
	{mmGDS_VMID10_BASE, mmGDS_VMID10_SIZE, mmGDS_GWS_VMID10, mmGDS_OA_VMID10},
	{mmGDS_VMID11_BASE, mmGDS_VMID11_SIZE, mmGDS_GWS_VMID11, mmGDS_OA_VMID11},
	{mmGDS_VMID12_BASE, mmGDS_VMID12_SIZE, mmGDS_GWS_VMID12, mmGDS_OA_VMID12},
	{mmGDS_VMID13_BASE, mmGDS_VMID13_SIZE, mmGDS_GWS_VMID13, mmGDS_OA_VMID13},
	{mmGDS_VMID14_BASE, mmGDS_VMID14_SIZE, mmGDS_GWS_VMID14, mmGDS_OA_VMID14},
	{mmGDS_VMID15_BASE, mmGDS_VMID15_SIZE, mmGDS_GWS_VMID15, mmGDS_OA_VMID15}
};

static const u32 spectre_rlc_save_restore_register_list[] =
{
	(0x0e00 << 16) | (0xc12c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc140 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc150 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc15c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc168 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc170 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc178 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc204 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2b4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2b8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2bc >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2c0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8228 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x829c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x869c >> 2),
	0x00000000,
	(0x0600 << 16) | (0x98f4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x98f8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x9900 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc260 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x90e8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3c000 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3c00c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8c1c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x9700 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xcd20 >> 2),
	0x00000000,
	(0x4e00 << 16) | (0xcd20 >> 2),
	0x00000000,
	(0x5e00 << 16) | (0xcd20 >> 2),
	0x00000000,
	(0x6e00 << 16) | (0xcd20 >> 2),
	0x00000000,
	(0x7e00 << 16) | (0xcd20 >> 2),
	0x00000000,
	(0x8e00 << 16) | (0xcd20 >> 2),
	0x00000000,
	(0x9e00 << 16) | (0xcd20 >> 2),
	0x00000000,
	(0xae00 << 16) | (0xcd20 >> 2),
	0x00000000,
	(0xbe00 << 16) | (0xcd20 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x89bc >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8900 >> 2),
	0x00000000,
	0x3,
	(0x0e00 << 16) | (0xc130 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc134 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc1fc >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc208 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc264 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc268 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc26c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc270 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc274 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc278 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc27c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc280 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc284 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc288 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc28c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc290 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc294 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc298 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc29c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2a0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2a4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2a8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2ac  >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2b0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x301d0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30238 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30250 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30254 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30258 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3025c >> 2),
	0x00000000,
	(0x4e00 << 16) | (0xc900 >> 2),
	0x00000000,
	(0x5e00 << 16) | (0xc900 >> 2),
	0x00000000,
	(0x6e00 << 16) | (0xc900 >> 2),
	0x00000000,
	(0x7e00 << 16) | (0xc900 >> 2),
	0x00000000,
	(0x8e00 << 16) | (0xc900 >> 2),
	0x00000000,
	(0x9e00 << 16) | (0xc900 >> 2),
	0x00000000,
	(0xae00 << 16) | (0xc900 >> 2),
	0x00000000,
	(0xbe00 << 16) | (0xc900 >> 2),
	0x00000000,
	(0x4e00 << 16) | (0xc904 >> 2),
	0x00000000,
	(0x5e00 << 16) | (0xc904 >> 2),
	0x00000000,
	(0x6e00 << 16) | (0xc904 >> 2),
	0x00000000,
	(0x7e00 << 16) | (0xc904 >> 2),
	0x00000000,
	(0x8e00 << 16) | (0xc904 >> 2),
	0x00000000,
	(0x9e00 << 16) | (0xc904 >> 2),
	0x00000000,
	(0xae00 << 16) | (0xc904 >> 2),
	0x00000000,
	(0xbe00 << 16) | (0xc904 >> 2),
	0x00000000,
	(0x4e00 << 16) | (0xc908 >> 2),
	0x00000000,
	(0x5e00 << 16) | (0xc908 >> 2),
	0x00000000,
	(0x6e00 << 16) | (0xc908 >> 2),
	0x00000000,
	(0x7e00 << 16) | (0xc908 >> 2),
	0x00000000,
	(0x8e00 << 16) | (0xc908 >> 2),
	0x00000000,
	(0x9e00 << 16) | (0xc908 >> 2),
	0x00000000,
	(0xae00 << 16) | (0xc908 >> 2),
	0x00000000,
	(0xbe00 << 16) | (0xc908 >> 2),
	0x00000000,
	(0x4e00 << 16) | (0xc90c >> 2),
	0x00000000,
	(0x5e00 << 16) | (0xc90c >> 2),
	0x00000000,
	(0x6e00 << 16) | (0xc90c >> 2),
	0x00000000,
	(0x7e00 << 16) | (0xc90c >> 2),
	0x00000000,
	(0x8e00 << 16) | (0xc90c >> 2),
	0x00000000,
	(0x9e00 << 16) | (0xc90c >> 2),
	0x00000000,
	(0xae00 << 16) | (0xc90c >> 2),
	0x00000000,
	(0xbe00 << 16) | (0xc90c >> 2),
	0x00000000,
	(0x4e00 << 16) | (0xc910 >> 2),
	0x00000000,
	(0x5e00 << 16) | (0xc910 >> 2),
	0x00000000,
	(0x6e00 << 16) | (0xc910 >> 2),
	0x00000000,
	(0x7e00 << 16) | (0xc910 >> 2),
	0x00000000,
	(0x8e00 << 16) | (0xc910 >> 2),
	0x00000000,
	(0x9e00 << 16) | (0xc910 >> 2),
	0x00000000,
	(0xae00 << 16) | (0xc910 >> 2),
	0x00000000,
	(0xbe00 << 16) | (0xc910 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc99c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x9834 >> 2),
	0x00000000,
	(0x0000 << 16) | (0x30f00 >> 2),
	0x00000000,
	(0x0001 << 16) | (0x30f00 >> 2),
	0x00000000,
	(0x0000 << 16) | (0x30f04 >> 2),
	0x00000000,
	(0x0001 << 16) | (0x30f04 >> 2),
	0x00000000,
	(0x0000 << 16) | (0x30f08 >> 2),
	0x00000000,
	(0x0001 << 16) | (0x30f08 >> 2),
	0x00000000,
	(0x0000 << 16) | (0x30f0c >> 2),
	0x00000000,
	(0x0001 << 16) | (0x30f0c >> 2),
	0x00000000,
	(0x0600 << 16) | (0x9b7c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8a14 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8a18 >> 2),
	0x00000000,
	(0x0600 << 16) | (0x30a00 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8bf0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8bcc >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8b24 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30a04 >> 2),
	0x00000000,
	(0x0600 << 16) | (0x30a10 >> 2),
	0x00000000,
	(0x0600 << 16) | (0x30a14 >> 2),
	0x00000000,
	(0x0600 << 16) | (0x30a18 >> 2),
	0x00000000,
	(0x0600 << 16) | (0x30a2c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc700 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc704 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc708 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc768 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc770 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc774 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc778 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc77c >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc780 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc784 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc788 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc78c >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc798 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc79c >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc7a0 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc7a4 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc7a8 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc7ac >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc7b0 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc7b4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x9100 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3c010 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x92a8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x92ac >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x92b4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x92b8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x92bc >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x92c0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x92c4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x92c8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x92cc >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x92d0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8c00 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8c04 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8c20 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8c38 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8c3c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xae00 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x9604 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac08 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac0c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac10 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac14 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac58 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac68 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac6c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac70 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac74 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac78 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac7c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac80 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac84 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac88 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac8c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x970c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x9714 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x9718 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x971c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x31068 >> 2),
	0x00000000,
	(0x4e00 << 16) | (0x31068 >> 2),
	0x00000000,
	(0x5e00 << 16) | (0x31068 >> 2),
	0x00000000,
	(0x6e00 << 16) | (0x31068 >> 2),
	0x00000000,
	(0x7e00 << 16) | (0x31068 >> 2),
	0x00000000,
	(0x8e00 << 16) | (0x31068 >> 2),
	0x00000000,
	(0x9e00 << 16) | (0x31068 >> 2),
	0x00000000,
	(0xae00 << 16) | (0x31068 >> 2),
	0x00000000,
	(0xbe00 << 16) | (0x31068 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xcd10 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xcd14 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88b0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88b4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88b8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88bc >> 2),
	0x00000000,
	(0x0400 << 16) | (0x89c0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88c4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88c8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88d0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88d4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88d8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8980 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30938 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3093c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30940 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x89a0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30900 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30904 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x89b4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3c210 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3c214 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3c218 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8904 >> 2),
	0x00000000,
	0x5,
	(0x0e00 << 16) | (0x8c28 >> 2),
	(0x0e00 << 16) | (0x8c2c >> 2),
	(0x0e00 << 16) | (0x8c30 >> 2),
	(0x0e00 << 16) | (0x8c34 >> 2),
	(0x0e00 << 16) | (0x9600 >> 2),
};

static const u32 kalindi_rlc_save_restore_register_list[] =
{
	(0x0e00 << 16) | (0xc12c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc140 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc150 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc15c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc168 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc170 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc204 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2b4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2b8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2bc >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2c0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8228 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x829c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x869c >> 2),
	0x00000000,
	(0x0600 << 16) | (0x98f4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x98f8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x9900 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc260 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x90e8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3c000 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3c00c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8c1c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x9700 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xcd20 >> 2),
	0x00000000,
	(0x4e00 << 16) | (0xcd20 >> 2),
	0x00000000,
	(0x5e00 << 16) | (0xcd20 >> 2),
	0x00000000,
	(0x6e00 << 16) | (0xcd20 >> 2),
	0x00000000,
	(0x7e00 << 16) | (0xcd20 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x89bc >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8900 >> 2),
	0x00000000,
	0x3,
	(0x0e00 << 16) | (0xc130 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc134 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc1fc >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc208 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc264 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc268 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc26c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc270 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc274 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc28c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc290 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc294 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc298 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2a0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2a4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2a8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc2ac >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x301d0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30238 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30250 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30254 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30258 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3025c >> 2),
	0x00000000,
	(0x4e00 << 16) | (0xc900 >> 2),
	0x00000000,
	(0x5e00 << 16) | (0xc900 >> 2),
	0x00000000,
	(0x6e00 << 16) | (0xc900 >> 2),
	0x00000000,
	(0x7e00 << 16) | (0xc900 >> 2),
	0x00000000,
	(0x4e00 << 16) | (0xc904 >> 2),
	0x00000000,
	(0x5e00 << 16) | (0xc904 >> 2),
	0x00000000,
	(0x6e00 << 16) | (0xc904 >> 2),
	0x00000000,
	(0x7e00 << 16) | (0xc904 >> 2),
	0x00000000,
	(0x4e00 << 16) | (0xc908 >> 2),
	0x00000000,
	(0x5e00 << 16) | (0xc908 >> 2),
	0x00000000,
	(0x6e00 << 16) | (0xc908 >> 2),
	0x00000000,
	(0x7e00 << 16) | (0xc908 >> 2),
	0x00000000,
	(0x4e00 << 16) | (0xc90c >> 2),
	0x00000000,
	(0x5e00 << 16) | (0xc90c >> 2),
	0x00000000,
	(0x6e00 << 16) | (0xc90c >> 2),
	0x00000000,
	(0x7e00 << 16) | (0xc90c >> 2),
	0x00000000,
	(0x4e00 << 16) | (0xc910 >> 2),
	0x00000000,
	(0x5e00 << 16) | (0xc910 >> 2),
	0x00000000,
	(0x6e00 << 16) | (0xc910 >> 2),
	0x00000000,
	(0x7e00 << 16) | (0xc910 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc99c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x9834 >> 2),
	0x00000000,
	(0x0000 << 16) | (0x30f00 >> 2),
	0x00000000,
	(0x0000 << 16) | (0x30f04 >> 2),
	0x00000000,
	(0x0000 << 16) | (0x30f08 >> 2),
	0x00000000,
	(0x0000 << 16) | (0x30f0c >> 2),
	0x00000000,
	(0x0600 << 16) | (0x9b7c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8a14 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8a18 >> 2),
	0x00000000,
	(0x0600 << 16) | (0x30a00 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8bf0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8bcc >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8b24 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30a04 >> 2),
	0x00000000,
	(0x0600 << 16) | (0x30a10 >> 2),
	0x00000000,
	(0x0600 << 16) | (0x30a14 >> 2),
	0x00000000,
	(0x0600 << 16) | (0x30a18 >> 2),
	0x00000000,
	(0x0600 << 16) | (0x30a2c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc700 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc704 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc708 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xc768 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc770 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc774 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc798 >> 2),
	0x00000000,
	(0x0400 << 16) | (0xc79c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x9100 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3c010 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8c00 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8c04 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8c20 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8c38 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8c3c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xae00 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x9604 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac08 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac0c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac10 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac14 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac58 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac68 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac6c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac70 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac74 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac78 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac7c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac80 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac84 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac88 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xac8c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x970c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x9714 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x9718 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x971c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x31068 >> 2),
	0x00000000,
	(0x4e00 << 16) | (0x31068 >> 2),
	0x00000000,
	(0x5e00 << 16) | (0x31068 >> 2),
	0x00000000,
	(0x6e00 << 16) | (0x31068 >> 2),
	0x00000000,
	(0x7e00 << 16) | (0x31068 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xcd10 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0xcd14 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88b0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88b4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88b8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88bc >> 2),
	0x00000000,
	(0x0400 << 16) | (0x89c0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88c4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88c8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88d0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88d4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x88d8 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8980 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30938 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3093c >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30940 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x89a0 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30900 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x30904 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x89b4 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3e1fc >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3c210 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3c214 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x3c218 >> 2),
	0x00000000,
	(0x0e00 << 16) | (0x8904 >> 2),
	0x00000000,
	0x5,
	(0x0e00 << 16) | (0x8c28 >> 2),
	(0x0e00 << 16) | (0x8c2c >> 2),
	(0x0e00 << 16) | (0x8c30 >> 2),
	(0x0e00 << 16) | (0x8c34 >> 2),
	(0x0e00 << 16) | (0x9600 >> 2),
};

static u32 gfx_v7_0_get_csb_size(struct amdgpu_device *adev);
static void gfx_v7_0_get_csb_buffer(struct amdgpu_device *adev, volatile u32 *buffer);
static void gfx_v7_0_init_cp_pg_table(struct amdgpu_device *adev);
static void gfx_v7_0_init_pg(struct amdgpu_device *adev);

/*
 * Core functions
 */
/**
 * gfx_v7_0_init_microcode - load ucode images from disk
 *
 * @adev: amdgpu_device pointer
 *
 * Use the firmware interface to load the ucode images into
 * the driver (not loaded into hw).
 * Returns 0 on success, error on failure.
 */
static int gfx_v7_0_init_microcode(struct amdgpu_device *adev)
{
	const char *chip_name;
	char fw_name[30];
	int err;

	DRM_DEBUG("\n");

	switch (adev->asic_type) {
	case CHIP_BONAIRE:
		chip_name = "bonaire";
		break;
	case CHIP_HAWAII:
		chip_name = "hawaii";
		break;
	case CHIP_KAVERI:
		chip_name = "kaveri";
		break;
	case CHIP_KABINI:
		chip_name = "kabini";
		break;
	case CHIP_MULLINS:
		chip_name = "mullins";
		break;
	default: BUG();
	}

	snprintf(fw_name, sizeof(fw_name), "radeon/%s_pfp.bin", chip_name);
	err = request_firmware(&adev->gfx.pfp_fw, fw_name, adev->dev);
	if (err)
		goto out;
	err = amdgpu_ucode_validate(adev->gfx.pfp_fw);
	if (err)
		goto out;

	snprintf(fw_name, sizeof(fw_name), "radeon/%s_me.bin", chip_name);
	err = request_firmware(&adev->gfx.me_fw, fw_name, adev->dev);
	if (err)
		goto out;
	err = amdgpu_ucode_validate(adev->gfx.me_fw);
	if (err)
		goto out;

	snprintf(fw_name, sizeof(fw_name), "radeon/%s_ce.bin", chip_name);
	err = request_firmware(&adev->gfx.ce_fw, fw_name, adev->dev);
	if (err)
		goto out;
	err = amdgpu_ucode_validate(adev->gfx.ce_fw);
	if (err)
		goto out;

	snprintf(fw_name, sizeof(fw_name), "radeon/%s_mec.bin", chip_name);
	err = request_firmware(&adev->gfx.mec_fw, fw_name, adev->dev);
	if (err)
		goto out;
	err = amdgpu_ucode_validate(adev->gfx.mec_fw);
	if (err)
		goto out;

	if (adev->asic_type == CHIP_KAVERI) {
		snprintf(fw_name, sizeof(fw_name), "radeon/%s_mec2.bin", chip_name);
		err = request_firmware(&adev->gfx.mec2_fw, fw_name, adev->dev);
		if (err)
			goto out;
		err = amdgpu_ucode_validate(adev->gfx.mec2_fw);
		if (err)
			goto out;
	}

	snprintf(fw_name, sizeof(fw_name), "radeon/%s_rlc.bin", chip_name);
	err = request_firmware(&adev->gfx.rlc_fw, fw_name, adev->dev);
	if (err)
		goto out;
	err = amdgpu_ucode_validate(adev->gfx.rlc_fw);

out:
	if (err) {
		printk(KERN_ERR
		       "gfx7: Failed to load firmware \"%s\"\n",
		       fw_name);
		release_firmware(adev->gfx.pfp_fw);
		adev->gfx.pfp_fw = NULL;
		release_firmware(adev->gfx.me_fw);
		adev->gfx.me_fw = NULL;
		release_firmware(adev->gfx.ce_fw);
		adev->gfx.ce_fw = NULL;
		release_firmware(adev->gfx.mec_fw);
		adev->gfx.mec_fw = NULL;
		release_firmware(adev->gfx.mec2_fw);
		adev->gfx.mec2_fw = NULL;
		release_firmware(adev->gfx.rlc_fw);
		adev->gfx.rlc_fw = NULL;
	}
	return err;
}

/**
 * gfx_v7_0_tiling_mode_table_init - init the hw tiling table
 *
 * @adev: amdgpu_device pointer
 *
 * Starting with SI, the tiling setup is done globally in a
 * set of 32 tiling modes.  Rather than selecting each set of
 * parameters per surface as on older asics, we just select
 * which index in the tiling table we want to use, and the
 * surface uses those parameters (CIK).
 */
static void gfx_v7_0_tiling_mode_table_init(struct amdgpu_device *adev)
{
	const u32 num_tile_mode_states = 32;
	const u32 num_secondary_tile_mode_states = 16;
	u32 reg_offset, gb_tile_moden, split_equal_to_row_size;

	switch (adev->gfx.config.mem_row_size_in_kb) {
	case 1:
		split_equal_to_row_size = ADDR_SURF_TILE_SPLIT_1KB;
		break;
	case 2:
	default:
		split_equal_to_row_size = ADDR_SURF_TILE_SPLIT_2KB;
		break;
	case 4:
		split_equal_to_row_size = ADDR_SURF_TILE_SPLIT_4KB;
		break;
	}

	switch (adev->asic_type) {
	case CHIP_BONAIRE:
		for (reg_offset = 0; reg_offset < num_tile_mode_states; reg_offset++) {
			switch (reg_offset) {
			case 0:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						TILE_SPLIT(ADDR_SURF_TILE_SPLIT_64B) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING));
				break;
			case 1:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						TILE_SPLIT(ADDR_SURF_TILE_SPLIT_128B) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING));
				break;
			case 2:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						TILE_SPLIT(ADDR_SURF_TILE_SPLIT_256B) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING));
				break;
			case 3:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						TILE_SPLIT(ADDR_SURF_TILE_SPLIT_512B) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING));
				break;
			case 4:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING) |
						TILE_SPLIT(split_equal_to_row_size));
				break;
			case 5:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING));
				break;
			case 6:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING) |
						TILE_SPLIT(split_equal_to_row_size));
				break;
			case 7:
				gb_tile_moden = (TILE_SPLIT(split_equal_to_row_size));
				break;

			case 8:
				gb_tile_moden = (ARRAY_MODE(ARRAY_LINEAR_ALIGNED) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16));
				break;
			case 9:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DISPLAY_MICRO_TILING));
				break;
			case 10:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DISPLAY_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_2));
				break;
			case 11:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DISPLAY_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_8));
				break;
			case 12:
				gb_tile_moden = (TILE_SPLIT(split_equal_to_row_size));
				break;
			case 13:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING));
				break;
			case 14:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_2));
				break;
			case 15:
				gb_tile_moden = (ARRAY_MODE(ARRAY_3D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_2));
				break;
			case 16:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_8));
				break;
			case 17:
				gb_tile_moden = (TILE_SPLIT(split_equal_to_row_size));
				break;
			case 18:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 19:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING));
				break;
			case 20:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 21:
				gb_tile_moden = (ARRAY_MODE(ARRAY_3D_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 22:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 23:
				gb_tile_moden = (TILE_SPLIT(split_equal_to_row_size));
				break;
			case 24:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 25:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_XTHICK) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 26:
				gb_tile_moden = (ARRAY_MODE(ARRAY_3D_TILED_XTHICK) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 27:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_ROTATED_MICRO_TILING));
				break;
			case 28:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_ROTATED_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_2));
				break;
			case 29:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_ROTATED_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_8));
				break;
			case 30:
				gb_tile_moden = (TILE_SPLIT(split_equal_to_row_size));
				break;
			default:
				gb_tile_moden = 0;
				break;
			}
			adev->gfx.config.tile_mode_array[reg_offset] = gb_tile_moden;
			WREG32(mmGB_TILE_MODE0 + reg_offset, gb_tile_moden);
		}
		for (reg_offset = 0; reg_offset < num_secondary_tile_mode_states; reg_offset++) {
			switch (reg_offset) {
			case 0:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_4) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_4) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 1:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_2) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_4) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 2:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 3:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 4:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 5:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_8_BANK));
				break;
			case 6:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_1) |
						NUM_BANKS(ADDR_SURF_4_BANK));
				break;
			case 8:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_2) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_8) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_4) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 9:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_2) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_4) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_4) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 10:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_4) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_4) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 11:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_2) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_4) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 12:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 13:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_8_BANK));
				break;
			case 14:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_1) |
						NUM_BANKS(ADDR_SURF_4_BANK));
				break;
			default:
				gb_tile_moden = 0;
				break;
			}
			adev->gfx.config.macrotile_mode_array[reg_offset] = gb_tile_moden;
			WREG32(mmGB_MACROTILE_MODE0 + reg_offset, gb_tile_moden);
		}
		break;
	case CHIP_HAWAII:
		for (reg_offset = 0; reg_offset < num_tile_mode_states; reg_offset++) {
			switch (reg_offset) {
			case 0:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						TILE_SPLIT(ADDR_SURF_TILE_SPLIT_64B) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING));
				break;
			case 1:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						TILE_SPLIT(ADDR_SURF_TILE_SPLIT_128B) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING));
				break;
			case 2:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						TILE_SPLIT(ADDR_SURF_TILE_SPLIT_256B) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING));
				break;
			case 3:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						TILE_SPLIT(ADDR_SURF_TILE_SPLIT_512B) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING));
				break;
			case 4:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING) |
						TILE_SPLIT(split_equal_to_row_size));
				break;
			case 5:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING) |
						TILE_SPLIT(split_equal_to_row_size));
				break;
			case 6:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING) |
						TILE_SPLIT(split_equal_to_row_size));
				break;
			case 7:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING) |
						TILE_SPLIT(split_equal_to_row_size));
				break;

			case 8:
				gb_tile_moden = (ARRAY_MODE(ARRAY_LINEAR_ALIGNED) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16));
				break;
			case 9:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DISPLAY_MICRO_TILING));
				break;
			case 10:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DISPLAY_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_2));
				break;
			case 11:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DISPLAY_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_8));
				break;
			case 12:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DISPLAY_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_8));
				break;
			case 13:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING));
				break;
			case 14:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_2));
				break;
			case 15:
				gb_tile_moden = (ARRAY_MODE(ARRAY_3D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_2));
				break;
			case 16:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_8));
				break;
			case 17:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_8));
				break;
			case 18:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 19:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THICK_MICRO_TILING));
				break;
			case 20:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THICK_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 21:
				gb_tile_moden = (ARRAY_MODE(ARRAY_3D_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THICK_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 22:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THICK_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 23:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THICK_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 24:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 25:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_XTHICK) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THICK_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 26:
				gb_tile_moden = (ARRAY_MODE(ARRAY_3D_TILED_XTHICK) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THICK_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 27:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_ROTATED_MICRO_TILING));
				break;
			case 28:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_ROTATED_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_2));
				break;
			case 29:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P16_32x32_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_ROTATED_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_8));
				break;
			case 30:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P4_16x16) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_ROTATED_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_8));
				break;
			default:
				gb_tile_moden = 0;
				break;
			}
			adev->gfx.config.tile_mode_array[reg_offset] = gb_tile_moden;
			WREG32(mmGB_TILE_MODE0 + reg_offset, gb_tile_moden);
		}
		for (reg_offset = 0; reg_offset < num_secondary_tile_mode_states; reg_offset++) {
			switch (reg_offset) {
			case 0:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_4) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 1:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_2) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 2:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_1) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 3:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_1) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 4:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_1) |
						NUM_BANKS(ADDR_SURF_8_BANK));
				break;
			case 5:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_1) |
						NUM_BANKS(ADDR_SURF_4_BANK));
				break;
			case 6:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_1) |
						NUM_BANKS(ADDR_SURF_4_BANK));
				break;
			case 8:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_4) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 9:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_2) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 10:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_1) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 11:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_1) |
						NUM_BANKS(ADDR_SURF_8_BANK));
				break;
			case 12:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 13:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_8_BANK));
				break;
			case 14:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_1) |
						NUM_BANKS(ADDR_SURF_4_BANK));
				break;
			default:
				gb_tile_moden = 0;
				break;
			}
			adev->gfx.config.macrotile_mode_array[reg_offset] = gb_tile_moden;
			WREG32(mmGB_MACROTILE_MODE0 + reg_offset, gb_tile_moden);
		}
		break;
	case CHIP_KABINI:
	case CHIP_KAVERI:
	case CHIP_MULLINS:
	default:
		for (reg_offset = 0; reg_offset < num_tile_mode_states; reg_offset++) {
			switch (reg_offset) {
			case 0:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						TILE_SPLIT(ADDR_SURF_TILE_SPLIT_64B) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING));
				break;
			case 1:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						TILE_SPLIT(ADDR_SURF_TILE_SPLIT_128B) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING));
				break;
			case 2:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						TILE_SPLIT(ADDR_SURF_TILE_SPLIT_256B) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING));
				break;
			case 3:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						TILE_SPLIT(ADDR_SURF_TILE_SPLIT_512B) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING));
				break;
			case 4:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING) |
						TILE_SPLIT(split_equal_to_row_size));
				break;
			case 5:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING));
				break;
			case 6:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DEPTH_MICRO_TILING) |
						TILE_SPLIT(split_equal_to_row_size));
				break;
			case 7:
				gb_tile_moden = (TILE_SPLIT(split_equal_to_row_size));
				break;

			case 8:
				gb_tile_moden = (ARRAY_MODE(ARRAY_LINEAR_ALIGNED) |
						PIPE_CONFIG(ADDR_SURF_P2));
				break;
			case 9:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DISPLAY_MICRO_TILING));
				break;
			case 10:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DISPLAY_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_2));
				break;
			case 11:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_DISPLAY_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_8));
				break;
			case 12:
				gb_tile_moden = (TILE_SPLIT(split_equal_to_row_size));
				break;
			case 13:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING));
				break;
			case 14:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_2));
				break;
			case 15:
				gb_tile_moden = (ARRAY_MODE(ARRAY_3D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_2));
				break;
			case 16:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_8));
				break;
			case 17:
				gb_tile_moden = (TILE_SPLIT(split_equal_to_row_size));
				break;
			case 18:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 19:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THICK_MICRO_TILING));
				break;
			case 20:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THICK_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 21:
				gb_tile_moden = (ARRAY_MODE(ARRAY_3D_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THICK_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 22:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THICK_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 23:
				gb_tile_moden = (TILE_SPLIT(split_equal_to_row_size));
				break;
			case 24:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THICK) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THIN_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 25:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_XTHICK) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THICK_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 26:
				gb_tile_moden = (ARRAY_MODE(ARRAY_3D_TILED_XTHICK) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_THICK_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_1));
				break;
			case 27:
				gb_tile_moden = (ARRAY_MODE(ARRAY_1D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_ROTATED_MICRO_TILING));
				break;
			case 28:
				gb_tile_moden = (ARRAY_MODE(ARRAY_2D_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_ROTATED_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_2));
				break;
			case 29:
				gb_tile_moden = (ARRAY_MODE(ARRAY_PRT_TILED_THIN1) |
						PIPE_CONFIG(ADDR_SURF_P2) |
						MICRO_TILE_MODE_NEW(ADDR_SURF_ROTATED_MICRO_TILING) |
						SAMPLE_SPLIT(ADDR_SURF_SAMPLE_SPLIT_8));
				break;
			case 30:
				gb_tile_moden = (TILE_SPLIT(split_equal_to_row_size));
				break;
			default:
				gb_tile_moden = 0;
				break;
			}
			adev->gfx.config.tile_mode_array[reg_offset] = gb_tile_moden;
			WREG32(mmGB_TILE_MODE0 + reg_offset, gb_tile_moden);
		}
		for (reg_offset = 0; reg_offset < num_secondary_tile_mode_states; reg_offset++) {
			switch (reg_offset) {
			case 0:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_4) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_4) |
						NUM_BANKS(ADDR_SURF_8_BANK));
				break;
			case 1:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_2) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_4) |
						NUM_BANKS(ADDR_SURF_8_BANK));
				break;
			case 2:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_2) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_8_BANK));
				break;
			case 3:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_8_BANK));
				break;
			case 4:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_8_BANK));
				break;
			case 5:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_8_BANK));
				break;
			case 6:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_8_BANK));
				break;
			case 8:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_4) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_8) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_4) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 9:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_4) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_4) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_4) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 10:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_2) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_4) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_4) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 11:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_2) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_2) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_4) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 12:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_2) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_4) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 13:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_4) |
						NUM_BANKS(ADDR_SURF_16_BANK));
				break;
			case 14:
				gb_tile_moden = (BANK_WIDTH(ADDR_SURF_BANK_WIDTH_1) |
						BANK_HEIGHT(ADDR_SURF_BANK_HEIGHT_1) |
						MACRO_TILE_ASPECT(ADDR_SURF_MACRO_ASPECT_2) |
						NUM_BANKS(ADDR_SURF_8_BANK));
				break;
			default:
				gb_tile_moden = 0;
				break;
			}
			adev->gfx.config.macrotile_mode_array[reg_offset] = gb_tile_moden;
			WREG32(mmGB_MACROTILE_MODE0 + reg_offset, gb_tile_moden);
		}
		break;
	}
}

/**
 * gfx_v7_0_select_se_sh - select which SE, SH to address
 *
 * @adev: amdgpu_device pointer
 * @se_num: shader engine to address
 * @sh_num: sh block to address
 *
 * Select which SE, SH combinations to address. Certain
 * registers are instanced per SE or SH.  0xffffffff means
 * broadcast to all SEs or SHs (CIK).
 */
void gfx_v7_0_select_se_sh(struct amdgpu_device *adev, u32 se_num, u32 sh_num)
{
	u32 data = GRBM_GFX_INDEX__INSTANCE_BROADCAST_WRITES_MASK;

	if ((se_num == 0xffffffff) && (sh_num == 0xffffffff))
		data |= GRBM_GFX_INDEX__SH_BROADCAST_WRITES_MASK |
			GRBM_GFX_INDEX__SE_BROADCAST_WRITES_MASK;
	else if (se_num == 0xffffffff)
		data |= GRBM_GFX_INDEX__SE_BROADCAST_WRITES_MASK |
			(sh_num << GRBM_GFX_INDEX__SH_INDEX__SHIFT);
	else if (sh_num == 0xffffffff)
		data |= GRBM_GFX_INDEX__SH_BROADCAST_WRITES_MASK |
			(se_num << GRBM_GFX_INDEX__SE_INDEX__SHIFT);
	else
		data |= (sh_num << GRBM_GFX_INDEX__SH_INDEX__SHIFT) |
			(se_num << GRBM_GFX_INDEX__SE_INDEX__SHIFT);
	WREG32(mmGRBM_GFX_INDEX, data);
}

/**
 * gfx_v7_0_create_bitmask - create a bitmask
 *
 * @bit_width: length of the mask
 *
 * create a variable length bit mask (CIK).
 * Returns the bitmask.
 */
static u32 gfx_v7_0_create_bitmask(u32 bit_width)
{
	u32 i, mask = 0;

	for (i = 0; i < bit_width; i++) {
		mask <<= 1;
		mask |= 1;
	}
	return mask;
}

/**
 * gfx_v7_0_get_rb_disabled - computes the mask of disabled RBs
 *
 * @adev: amdgpu_device pointer
 * @max_rb_num: max RBs (render backends) for the asic
 * @se_num: number of SEs (shader engines) for the asic
 * @sh_per_se: number of SH blocks per SE for the asic
 *
 * Calculates the bitmask of disabled RBs (CIK).
 * Returns the disabled RB bitmask.
 */
static u32 gfx_v7_0_get_rb_disabled(struct amdgpu_device *adev,
				    u32 max_rb_num_per_se,
				    u32 sh_per_se)
{
	u32 data, mask;

	data = RREG32(mmCC_RB_BACKEND_DISABLE);
	if (data & 1)
		data &= CC_RB_BACKEND_DISABLE__BACKEND_DISABLE_MASK;
	else
		data = 0;

	data |= RREG32(mmGC_USER_RB_BACKEND_DISABLE);

	data >>= GC_USER_RB_BACKEND_DISABLE__BACKEND_DISABLE__SHIFT;

	mask = gfx_v7_0_create_bitmask(max_rb_num_per_se / sh_per_se);

	return data & mask;
}

/**
 * gfx_v7_0_setup_rb - setup the RBs on the asic
 *
 * @adev: amdgpu_device pointer
 * @se_num: number of SEs (shader engines) for the asic
 * @sh_per_se: number of SH blocks per SE for the asic
 * @max_rb_num: max RBs (render backends) for the asic
 *
 * Configures per-SE/SH RB registers (CIK).
 */
static void gfx_v7_0_setup_rb(struct amdgpu_device *adev,
			      u32 se_num, u32 sh_per_se,
			      u32 max_rb_num_per_se)
{
	int i, j;
	u32 data, mask;
	u32 disabled_rbs = 0;
	u32 enabled_rbs = 0;

	mutex_lock(&adev->grbm_idx_mutex);
	for (i = 0; i < se_num; i++) {
		for (j = 0; j < sh_per_se; j++) {
			gfx_v7_0_select_se_sh(adev, i, j);
			data = gfx_v7_0_get_rb_disabled(adev, max_rb_num_per_se, sh_per_se);
			if (adev->asic_type == CHIP_HAWAII)
				disabled_rbs |= data << ((i * sh_per_se + j) * HAWAII_RB_BITMAP_WIDTH_PER_SH);
			else
				disabled_rbs |= data << ((i * sh_per_se + j) * CIK_RB_BITMAP_WIDTH_PER_SH);
		}
	}
	gfx_v7_0_select_se_sh(adev, 0xffffffff, 0xffffffff);
	mutex_unlock(&adev->grbm_idx_mutex);

	mask = 1;
	for (i = 0; i < max_rb_num_per_se * se_num; i++) {
		if (!(disabled_rbs & mask))
			enabled_rbs |= mask;
		mask <<= 1;
	}

	adev->gfx.config.backend_enable_mask = enabled_rbs;

	mutex_lock(&adev->grbm_idx_mutex);
	for (i = 0; i < se_num; i++) {
		gfx_v7_0_select_se_sh(adev, i, 0xffffffff);
		data = 0;
		for (j = 0; j < sh_per_se; j++) {
			switch (enabled_rbs & 3) {
			case 0:
				if (j == 0)
					data |= (RASTER_CONFIG_RB_MAP_3 <<
						PA_SC_RASTER_CONFIG__PKR_MAP__SHIFT);
				else
					data |= (RASTER_CONFIG_RB_MAP_0 <<
						PA_SC_RASTER_CONFIG__PKR_MAP__SHIFT);
				break;
			case 1:
				data |= (RASTER_CONFIG_RB_MAP_0 << (i * sh_per_se + j) * 2);
				break;
			case 2:
				data |= (RASTER_CONFIG_RB_MAP_3 << (i * sh_per_se + j) * 2);
				break;
			case 3:
			default:
				data |= (RASTER_CONFIG_RB_MAP_2 << (i * sh_per_se + j) * 2);
				break;
			}
			enabled_rbs >>= 2;
		}
		WREG32(mmPA_SC_RASTER_CONFIG, data);
	}
	gfx_v7_0_select_se_sh(adev, 0xffffffff, 0xffffffff);
	mutex_unlock(&adev->grbm_idx_mutex);
}

/**
 * gmc_v7_0_init_compute_vmid - gart enable
 *
 * @rdev: amdgpu_device pointer
 *
 * Initialize compute vmid sh_mem registers
 *
 */
#define DEFAULT_SH_MEM_BASES	(0x6000)
#define FIRST_COMPUTE_VMID	(8)
#define LAST_COMPUTE_VMID	(16)
static void gmc_v7_0_init_compute_vmid(struct amdgpu_device *adev)
{
	int i;
	uint32_t sh_mem_config;
	uint32_t sh_mem_bases;

	/*
	 * Configure apertures:
	 * LDS:         0x60000000'00000000 - 0x60000001'00000000 (4GB)
	 * Scratch:     0x60000001'00000000 - 0x60000002'00000000 (4GB)
	 * GPUVM:       0x60010000'00000000 - 0x60020000'00000000 (1TB)
	*/
	sh_mem_bases = DEFAULT_SH_MEM_BASES | (DEFAULT_SH_MEM_BASES << 16);
	sh_mem_config = SH_MEM_ALIGNMENT_MODE_UNALIGNED <<
			SH_MEM_CONFIG__ALIGNMENT_MODE__SHIFT;
	sh_mem_config |= MTYPE_NONCACHED << SH_MEM_CONFIG__DEFAULT_MTYPE__SHIFT;
	mutex_lock(&adev->srbm_mutex);
	for (i = FIRST_COMPUTE_VMID; i < LAST_COMPUTE_VMID; i++) {
		cik_srbm_select(adev, 0, 0, 0, i);
		/* CP and shaders */
		WREG32(mmSH_MEM_CONFIG, sh_mem_config);
		WREG32(mmSH_MEM_APE1_BASE, 1);
		WREG32(mmSH_MEM_APE1_LIMIT, 0);
		WREG32(mmSH_MEM_BASES, sh_mem_bases);
	}
	cik_srbm_select(adev, 0, 0, 0, 0);
	mutex_unlock(&adev->srbm_mutex);
}

/**
 * gfx_v7_0_gpu_init - setup the 3D engine
 *
 * @adev: amdgpu_device pointer
 *
 * Configures the 3D engine and tiling configuration
 * registers so that the 3D engine is usable.
 */
static void gfx_v7_0_gpu_init(struct amdgpu_device *adev)
{
	u32 gb_addr_config;
	u32 mc_shared_chmap, mc_arb_ramcfg;
	u32 dimm00_addr_map, dimm01_addr_map, dimm10_addr_map, dimm11_addr_map;
	u32 sh_mem_cfg;
	u32 tmp;
	int i;

	switch (adev->asic_type) {
	case CHIP_BONAIRE:
		adev->gfx.config.max_shader_engines = 2;
		adev->gfx.config.max_tile_pipes = 4;
		adev->gfx.config.max_cu_per_sh = 7;
		adev->gfx.config.max_sh_per_se = 1;
		adev->gfx.config.max_backends_per_se = 2;
		adev->gfx.config.max_texture_channel_caches = 4;
		adev->gfx.config.max_gprs = 256;
		adev->gfx.config.max_gs_threads = 32;
		adev->gfx.config.max_hw_contexts = 8;

		adev->gfx.config.sc_prim_fifo_size_frontend = 0x20;
		adev->gfx.config.sc_prim_fifo_size_backend = 0x100;
		adev->gfx.config.sc_hiz_tile_fifo_size = 0x30;
		adev->gfx.config.sc_earlyz_tile_fifo_size = 0x130;
		gb_addr_config = BONAIRE_GB_ADDR_CONFIG_GOLDEN;
		break;
	case CHIP_HAWAII:
		adev->gfx.config.max_shader_engines = 4;
		adev->gfx.config.max_tile_pipes = 16;
		adev->gfx.config.max_cu_per_sh = 11;
		adev->gfx.config.max_sh_per_se = 1;
		adev->gfx.config.max_backends_per_se = 4;
		adev->gfx.config.max_texture_channel_caches = 16;
		adev->gfx.config.max_gprs = 256;
		adev->gfx.config.max_gs_threads = 32;
		adev->gfx.config.max_hw_contexts = 8;

		adev->gfx.config.sc_prim_fifo_size_frontend = 0x20;
		adev->gfx.config.sc_prim_fifo_size_backend = 0x100;
		adev->gfx.config.sc_hiz_tile_fifo_size = 0x30;
		adev->gfx.config.sc_earlyz_tile_fifo_size = 0x130;
		gb_addr_config = HAWAII_GB_ADDR_CONFIG_GOLDEN;
		break;
	case CHIP_KAVERI:
		adev->gfx.config.max_shader_engines = 1;
		adev->gfx.config.max_tile_pipes = 4;
		adev->gfx.config.max_cu_per_sh = 8;
		adev->gfx.config.max_backends_per_se = 2;
		adev->gfx.config.max_sh_per_se = 1;
		adev->gfx.config.max_texture_channel_caches = 4;
		adev->gfx.config.max_gprs = 256;
		adev->gfx.config.max_gs_threads = 16;
		adev->gfx.config.max_hw_contexts = 8;

		adev->gfx.config.sc_prim_fifo_size_frontend = 0x20;
		adev->gfx.config.sc_prim_fifo_size_backend = 0x100;
		adev->gfx.config.sc_hiz_tile_fifo_size = 0x30;
		adev->gfx.config.sc_earlyz_tile_fifo_size = 0x130;
		gb_addr_config = BONAIRE_GB_ADDR_CONFIG_GOLDEN;
		break;
	case CHIP_KABINI:
	case CHIP_MULLINS:
	default:
		adev->gfx.config.max_shader_engines = 1;
		adev->gfx.config.max_tile_pipes = 2;
		adev->gfx.config.max_cu_per_sh = 2;
		adev->gfx.config.max_sh_per_se = 1;
		adev->gfx.config.max_backends_per_se = 1;
		adev->gfx.config.max_texture_channel_caches = 2;
		adev->gfx.config.max_gprs = 256;
		adev->gfx.config.max_gs_threads = 16;
		adev->gfx.config.max_hw_contexts = 8;

		adev->gfx.config.sc_prim_fifo_size_frontend = 0x20;
		adev->gfx.config.sc_prim_fifo_size_backend = 0x100;
		adev->gfx.config.sc_hiz_tile_fifo_size = 0x30;
		adev->gfx.config.sc_earlyz_tile_fifo_size = 0x130;
		gb_addr_config = BONAIRE_GB_ADDR_CONFIG_GOLDEN;
		break;
	}

	WREG32(mmGRBM_CNTL, (0xff << GRBM_CNTL__READ_TIMEOUT__SHIFT));

	mc_shared_chmap = RREG32(mmMC_SHARED_CHMAP);
	adev->gfx.config.mc_arb_ramcfg = RREG32(mmMC_ARB_RAMCFG);
	mc_arb_ramcfg = adev->gfx.config.mc_arb_ramcfg;

	adev->gfx.config.num_tile_pipes = adev->gfx.config.max_tile_pipes;
	adev->gfx.config.mem_max_burst_length_bytes = 256;
	if (adev->flags & AMD_IS_APU) {
		/* Get memory bank mapping mode. */
		tmp = RREG32(mmMC_FUS_DRAM0_BANK_ADDR_MAPPING);
		dimm00_addr_map = REG_GET_FIELD(tmp, MC_FUS_DRAM0_BANK_ADDR_MAPPING, DIMM0ADDRMAP);
		dimm01_addr_map = REG_GET_FIELD(tmp, MC_FUS_DRAM0_BANK_ADDR_MAPPING, DIMM1ADDRMAP);

		tmp = RREG32(mmMC_FUS_DRAM1_BANK_ADDR_MAPPING);
		dimm10_addr_map = REG_GET_FIELD(tmp, MC_FUS_DRAM1_BANK_ADDR_MAPPING, DIMM0ADDRMAP);
		dimm11_addr_map = REG_GET_FIELD(tmp, MC_FUS_DRAM1_BANK_ADDR_MAPPING, DIMM1ADDRMAP);

		/* Validate settings in case only one DIMM installed. */
		if ((dimm00_addr_map == 0) || (dimm00_addr_map == 3) || (dimm00_addr_map == 4) || (dimm00_addr_map > 12))
			dimm00_addr_map = 0;
		if ((dimm01_addr_map == 0) || (dimm01_addr_map == 3) || (dimm01_addr_map == 4) || (dimm01_addr_map > 12))
			dimm01_addr_map = 0;
		if ((dimm10_addr_map == 0) || (dimm10_addr_map == 3) || (dimm10_addr_map == 4) || (dimm10_addr_map > 12))
			dimm10_addr_map = 0;
		if ((dimm11_addr_map == 0) || (dimm11_addr_map == 3) || (dimm11_addr_map == 4) || (dimm11_addr_map > 12))
			dimm11_addr_map = 0;

		/* If DIMM Addr map is 8GB, ROW size should be 2KB. Otherwise 1KB. */
		/* If ROW size(DIMM1) != ROW size(DMIMM0), ROW size should be larger one. */
		if ((dimm00_addr_map == 11) || (dimm01_addr_map == 11) || (dimm10_addr_map == 11) || (dimm11_addr_map == 11))
			adev->gfx.config.mem_row_size_in_kb = 2;
		else
			adev->gfx.config.mem_row_size_in_kb = 1;
	} else {
		tmp = (mc_arb_ramcfg & MC_ARB_RAMCFG__NOOFCOLS_MASK) >> MC_ARB_RAMCFG__NOOFCOLS__SHIFT;
		adev->gfx.config.mem_row_size_in_kb = (4 * (1 << (8 + tmp))) / 1024;
		if (adev->gfx.config.mem_row_size_in_kb > 4)
			adev->gfx.config.mem_row_size_in_kb = 4;
	}
	/* XXX use MC settings? */
	adev->gfx.config.shader_engine_tile_size = 32;
	adev->gfx.config.num_gpus = 1;
	adev->gfx.config.multi_gpu_tile_size = 64;

	/* fix up row size */
	gb_addr_config &= ~GB_ADDR_CONFIG__ROW_SIZE_MASK;
	switch (adev->gfx.config.mem_row_size_in_kb) {
	case 1:
	default:
		gb_addr_config |= (0 << GB_ADDR_CONFIG__ROW_SIZE__SHIFT);
		break;
	case 2:
		gb_addr_config |= (1 << GB_ADDR_CONFIG__ROW_SIZE__SHIFT);
		break;
	case 4:
		gb_addr_config |= (2 << GB_ADDR_CONFIG__ROW_SIZE__SHIFT);
		break;
	}
	adev->gfx.config.gb_addr_config = gb_addr_config;

	WREG32(mmGB_ADDR_CONFIG, gb_addr_config);
	WREG32(mmHDP_ADDR_CONFIG, gb_addr_config);
	WREG32(mmDMIF_ADDR_CALC, gb_addr_config);
	WREG32(mmSDMA0_TILING_CONFIG + SDMA0_REGISTER_OFFSET, gb_addr_config & 0x70);
	WREG32(mmSDMA0_TILING_CONFIG + SDMA1_REGISTER_OFFSET, gb_addr_config & 0x70);
	WREG32(mmUVD_UDEC_ADDR_CONFIG, gb_addr_config);
	WREG32(mmUVD_UDEC_DB_ADDR_CONFIG, gb_addr_config);
	WREG32(mmUVD_UDEC_DBW_ADDR_CONFIG, gb_addr_config);

	gfx_v7_0_tiling_mode_table_init(adev);

	gfx_v7_0_setup_rb(adev, adev->gfx.config.max_shader_engines,
			  adev->gfx.config.max_sh_per_se,
			  adev->gfx.config.max_backends_per_se);

	/* set HW defaults for 3D engine */
	WREG32(mmCP_MEQ_THRESHOLDS,
			(0x30 << CP_MEQ_THRESHOLDS__MEQ1_START__SHIFT) |
			(0x60 << CP_MEQ_THRESHOLDS__MEQ2_START__SHIFT));

	mutex_lock(&adev->grbm_idx_mutex);
	/*
	 * making sure that the following register writes will be broadcasted
	 * to all the shaders
	 */
	gfx_v7_0_select_se_sh(adev, 0xffffffff, 0xffffffff);

	/* XXX SH_MEM regs */
	/* where to put LDS, scratch, GPUVM in FSA64 space */
	sh_mem_cfg = REG_SET_FIELD(0, SH_MEM_CONFIG, ALIGNMENT_MODE, 
				   SH_MEM_ALIGNMENT_MODE_UNALIGNED);

	mutex_lock(&adev->srbm_mutex);
	for (i = 0; i < 16; i++) {
		cik_srbm_select(adev, 0, 0, 0, i);
		/* CP and shaders */
		WREG32(mmSH_MEM_CONFIG, sh_mem_cfg);
		WREG32(mmSH_MEM_APE1_BASE, 1);
		WREG32(mmSH_MEM_APE1_LIMIT, 0);
		WREG32(mmSH_MEM_BASES, 0);
	}
	cik_srbm_select(adev, 0, 0, 0, 0);
	mutex_unlock(&adev->srbm_mutex);

	gmc_v7_0_init_compute_vmid(adev);

	WREG32(mmSX_DEBUG_1, 0x20);

	WREG32(mmTA_CNTL_AUX, 0x00010000);

	tmp = RREG32(mmSPI_CONFIG_CNTL);
	tmp |= 0x03000000;
	WREG32(mmSPI_CONFIG_CNTL, tmp);

	WREG32(mmSQ_CONFIG, 1);

	WREG32(mmDB_DEBUG, 0);

	tmp = RREG32(mmDB_DEBUG2) & ~0xf00fffff;
	tmp |= 0x00000400;
	WREG32(mmDB_DEBUG2, tmp);

	tmp = RREG32(mmDB_DEBUG3) & ~0x0002021c;
	tmp |= 0x00020200;
	WREG32(mmDB_DEBUG3, tmp);

	tmp = RREG32(mmCB_HW_CONTROL) & ~0x00010000;
	tmp |= 0x00018208;
	WREG32(mmCB_HW_CONTROL, tmp);

	WREG32(mmSPI_CONFIG_CNTL_1, (4 << SPI_CONFIG_CNTL_1__VTX_DONE_DELAY__SHIFT));

	WREG32(mmPA_SC_FIFO_SIZE,
		((adev->gfx.config.sc_prim_fifo_size_frontend << PA_SC_FIFO_SIZE__SC_FRONTEND_PRIM_FIFO_SIZE__SHIFT) |
		(adev->gfx.config.sc_prim_fifo_size_backend << PA_SC_FIFO_SIZE__SC_BACKEND_PRIM_FIFO_SIZE__SHIFT) |
		(adev->gfx.config.sc_hiz_tile_fifo_size << PA_SC_FIFO_SIZE__SC_HIZ_TILE_FIFO_SIZE__SHIFT) |
		(adev->gfx.config.sc_earlyz_tile_fifo_size << PA_SC_FIFO_SIZE__SC_EARLYZ_TILE_FIFO_SIZE__SHIFT)));

	WREG32(mmVGT_NUM_INSTANCES, 1);

	WREG32(mmCP_PERFMON_CNTL, 0);

	WREG32(mmSQ_CONFIG, 0);

	WREG32(mmPA_SC_FORCE_EOV_MAX_CNTS,
		((4095 << PA_SC_FORCE_EOV_MAX_CNTS__FORCE_EOV_MAX_CLK_CNT__SHIFT) |
		(255 << PA_SC_FORCE_EOV_MAX_CNTS__FORCE_EOV_MAX_REZ_CNT__SHIFT)));

	WREG32(mmVGT_CACHE_INVALIDATION,
		(VC_AND_TC << VGT_CACHE_INVALIDATION__CACHE_INVALIDATION__SHIFT) |
		(ES_AND_GS_AUTO << VGT_CACHE_INVALIDATION__AUTO_INVLD_EN__SHIFT));

	WREG32(mmVGT_GS_VERTEX_REUSE, 16);
	WREG32(mmPA_SC_LINE_STIPPLE_STATE, 0);

	WREG32(mmPA_CL_ENHANCE, PA_CL_ENHANCE__CLIP_VTX_REORDER_ENA_MASK |
			(3 << PA_CL_ENHANCE__NUM_CLIP_SEQ__SHIFT));
	WREG32(mmPA_SC_ENHANCE, PA_SC_ENHANCE__ENABLE_PA_SC_OUT_OF_ORDER_MASK);
	mutex_unlock(&adev->grbm_idx_mutex);

	udelay(50);
}

/*
 * GPU scratch registers helpers function.
 */
/**
 * gfx_v7_0_scratch_init - setup driver info for CP scratch regs
 *
 * @adev: amdgpu_device pointer
 *
 * Set up the number and offset of the CP scratch registers.
 * NOTE: use of CP scratch registers is a legacy inferface and
 * is not used by default on newer asics (r6xx+).  On newer asics,
 * memory buffers are used for fences rather than scratch regs.
 */
static void gfx_v7_0_scratch_init(struct amdgpu_device *adev)
{
	int i;

	adev->gfx.scratch.num_reg = 7;
	adev->gfx.scratch.reg_base = mmSCRATCH_REG0;
	for (i = 0; i < adev->gfx.scratch.num_reg; i++) {
		adev->gfx.scratch.free[i] = true;
		adev->gfx.scratch.reg[i] = adev->gfx.scratch.reg_base + i;
	}
}

/**
 * gfx_v7_0_ring_test_ring - basic gfx ring test
 *
 * @adev: amdgpu_device pointer
 * @ring: amdgpu_ring structure holding ring information
 *
 * Allocate a scratch register and write to it using the gfx ring (CIK).
 * Provides a basic gfx ring test to verify that the ring is working.
 * Used by gfx_v7_0_cp_gfx_resume();
 * Returns 0 on success, error on failure.
 */
static int gfx_v7_0_ring_test_ring(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;
	uint32_t scratch;
	uint32_t tmp = 0;
	unsigned i;
	int r;

	r = amdgpu_gfx_scratch_get(adev, &scratch);
	if (r) {
		DRM_ERROR("amdgpu: cp failed to get scratch reg (%d).\n", r);
		return r;
	}
	WREG32(scratch, 0xCAFEDEAD);
	r = amdgpu_ring_lock(ring, 3);
	if (r) {
		DRM_ERROR("amdgpu: cp failed to lock ring %d (%d).\n", ring->idx, r);
		amdgpu_gfx_scratch_free(adev, scratch);
		return r;
	}
	amdgpu_ring_write(ring, PACKET3(PACKET3_SET_UCONFIG_REG, 1));
	amdgpu_ring_write(ring, (scratch - PACKET3_SET_UCONFIG_REG_START));
	amdgpu_ring_write(ring, 0xDEADBEEF);
	amdgpu_ring_unlock_commit(ring);

	for (i = 0; i < adev->usec_timeout; i++) {
		tmp = RREG32(scratch);
		if (tmp == 0xDEADBEEF)
			break;
		DRM_UDELAY(1);
	}
	if (i < adev->usec_timeout) {
		DRM_INFO("ring test on %d succeeded in %d usecs\n", ring->idx, i);
	} else {
		DRM_ERROR("amdgpu: ring %d test failed (scratch(0x%04X)=0x%08X)\n",
			  ring->idx, scratch, tmp);
		r = -EINVAL;
	}
	amdgpu_gfx_scratch_free(adev, scratch);
	return r;
}

/**
 * gfx_v7_0_ring_emit_hdp - emit an hdp flush on the cp
 *
 * @adev: amdgpu_device pointer
 * @ridx: amdgpu ring index
 *
 * Emits an hdp flush on the cp.
 */
static void gfx_v7_0_ring_emit_hdp_flush(struct amdgpu_ring *ring)
{
	u32 ref_and_mask;
	int usepfp = ring->type == AMDGPU_RING_TYPE_COMPUTE ? 0 : 1;

	if (ring->type == AMDGPU_RING_TYPE_COMPUTE) {
		switch (ring->me) {
		case 1:
			ref_and_mask = GPU_HDP_FLUSH_DONE__CP2_MASK << ring->pipe;
			break;
		case 2:
			ref_and_mask = GPU_HDP_FLUSH_DONE__CP6_MASK << ring->pipe;
			break;
		default:
			return;
		}
	} else {
		ref_and_mask = GPU_HDP_FLUSH_DONE__CP0_MASK;
	}

	amdgpu_ring_write(ring, PACKET3(PACKET3_WAIT_REG_MEM, 5));
	amdgpu_ring_write(ring, (WAIT_REG_MEM_OPERATION(1) | /* write, wait, write */
				 WAIT_REG_MEM_FUNCTION(3) |  /* == */
				 WAIT_REG_MEM_ENGINE(usepfp)));   /* pfp or me */
	amdgpu_ring_write(ring, mmGPU_HDP_FLUSH_REQ);
	amdgpu_ring_write(ring, mmGPU_HDP_FLUSH_DONE);
	amdgpu_ring_write(ring, ref_and_mask);
	amdgpu_ring_write(ring, ref_and_mask);
	amdgpu_ring_write(ring, 0x20); /* poll interval */
}

/**
 * gfx_v7_0_ring_emit_fence_gfx - emit a fence on the gfx ring
 *
 * @adev: amdgpu_device pointer
 * @fence: amdgpu fence object
 *
 * Emits a fence sequnce number on the gfx ring and flushes
 * GPU caches.
 */
static void gfx_v7_0_ring_emit_fence_gfx(struct amdgpu_ring *ring, u64 addr,
					 u64 seq, unsigned flags)
{
	bool write64bit = flags & AMDGPU_FENCE_FLAG_64BIT;
	bool int_sel = flags & AMDGPU_FENCE_FLAG_INT;
	/* Workaround for cache flush problems. First send a dummy EOP
	 * event down the pipe with seq one below.
	 */
	amdgpu_ring_write(ring, PACKET3(PACKET3_EVENT_WRITE_EOP, 4));
	amdgpu_ring_write(ring, (EOP_TCL1_ACTION_EN |
				 EOP_TC_ACTION_EN |
				 EVENT_TYPE(CACHE_FLUSH_AND_INV_TS_EVENT) |
				 EVENT_INDEX(5)));
	amdgpu_ring_write(ring, addr & 0xfffffffc);
	amdgpu_ring_write(ring, (upper_32_bits(addr) & 0xffff) |
				DATA_SEL(1) | INT_SEL(0));
	amdgpu_ring_write(ring, lower_32_bits(seq - 1));
	amdgpu_ring_write(ring, upper_32_bits(seq - 1));

	/* Then send the real EOP event down the pipe. */
	amdgpu_ring_write(ring, PACKET3(PACKET3_EVENT_WRITE_EOP, 4));
	amdgpu_ring_write(ring, (EOP_TCL1_ACTION_EN |
				 EOP_TC_ACTION_EN |
				 EVENT_TYPE(CACHE_FLUSH_AND_INV_TS_EVENT) |
				 EVENT_INDEX(5)));
	amdgpu_ring_write(ring, addr & 0xfffffffc);
	amdgpu_ring_write(ring, (upper_32_bits(addr) & 0xffff) |
				DATA_SEL(write64bit ? 2 : 1) | INT_SEL(int_sel ? 2 : 0));
	amdgpu_ring_write(ring, lower_32_bits(seq));
	amdgpu_ring_write(ring, upper_32_bits(seq));
}

/**
 * gfx_v7_0_ring_emit_fence_compute - emit a fence on the compute ring
 *
 * @adev: amdgpu_device pointer
 * @fence: amdgpu fence object
 *
 * Emits a fence sequnce number on the compute ring and flushes
 * GPU caches.
 */
static void gfx_v7_0_ring_emit_fence_compute(struct amdgpu_ring *ring,
					     u64 addr, u64 seq,
					     unsigned flags)
{
	bool write64bit = flags & AMDGPU_FENCE_FLAG_64BIT;
	bool int_sel = flags & AMDGPU_FENCE_FLAG_INT;

	/* RELEASE_MEM - flush caches, send int */
	amdgpu_ring_write(ring, PACKET3(PACKET3_RELEASE_MEM, 5));
	amdgpu_ring_write(ring, (EOP_TCL1_ACTION_EN |
				 EOP_TC_ACTION_EN |
				 EVENT_TYPE(CACHE_FLUSH_AND_INV_TS_EVENT) |
				 EVENT_INDEX(5)));
	amdgpu_ring_write(ring, DATA_SEL(write64bit ? 2 : 1) | INT_SEL(int_sel ? 2 : 0));
	amdgpu_ring_write(ring, addr & 0xfffffffc);
	amdgpu_ring_write(ring, upper_32_bits(addr));
	amdgpu_ring_write(ring, lower_32_bits(seq));
	amdgpu_ring_write(ring, upper_32_bits(seq));
}

/**
 * gfx_v7_0_ring_emit_semaphore - emit a semaphore on the CP ring
 *
 * @ring: amdgpu ring buffer object
 * @semaphore: amdgpu semaphore object
 * @emit_wait: Is this a sempahore wait?
 *
 * Emits a semaphore signal/wait packet to the CP ring and prevents the PFP
 * from running ahead of semaphore waits.
 */
static bool gfx_v7_0_ring_emit_semaphore(struct amdgpu_ring *ring,
					 struct amdgpu_semaphore *semaphore,
					 bool emit_wait)
{
	uint64_t addr = semaphore->gpu_addr;
	unsigned sel = emit_wait ? PACKET3_SEM_SEL_WAIT : PACKET3_SEM_SEL_SIGNAL;

	amdgpu_ring_write(ring, PACKET3(PACKET3_MEM_SEMAPHORE, 1));
	amdgpu_ring_write(ring, addr & 0xffffffff);
	amdgpu_ring_write(ring, (upper_32_bits(addr) & 0xffff) | sel);

	if (emit_wait && (ring->type == AMDGPU_RING_TYPE_GFX)) {
		/* Prevent the PFP from running ahead of the semaphore wait */
		amdgpu_ring_write(ring, PACKET3(PACKET3_PFP_SYNC_ME, 0));
		amdgpu_ring_write(ring, 0x0);
	}

	return true;
}

/*
 * IB stuff
 */
/**
 * gfx_v7_0_ring_emit_ib - emit an IB (Indirect Buffer) on the ring
 *
 * @ring: amdgpu_ring structure holding ring information
 * @ib: amdgpu indirect buffer object
 *
 * Emits an DE (drawing engine) or CE (constant engine) IB
 * on the gfx ring.  IBs are usually generated by userspace
 * acceleration drivers and submitted to the kernel for
 * sheduling on the ring.  This function schedules the IB
 * on the gfx ring for execution by the GPU.
 */
static void gfx_v7_0_ring_emit_ib_gfx(struct amdgpu_ring *ring,
				  struct amdgpu_ib *ib)
{
	bool need_ctx_switch = ring->current_ctx != ib->ctx;
	u32 header, control = 0;
	u32 next_rptr = ring->wptr + 5;

	/* drop the CE preamble IB for the same context */
	if ((ib->flags & AMDGPU_IB_FLAG_PREAMBLE) && !need_ctx_switch)
		return;

	if (need_ctx_switch)
		next_rptr += 2;

	next_rptr += 4;
	amdgpu_ring_write(ring, PACKET3(PACKET3_WRITE_DATA, 3));
	amdgpu_ring_write(ring, WRITE_DATA_DST_SEL(5) | WR_CONFIRM);
	amdgpu_ring_write(ring, ring->next_rptr_gpu_addr & 0xfffffffc);
	amdgpu_ring_write(ring, upper_32_bits(ring->next_rptr_gpu_addr) & 0xffffffff);
	amdgpu_ring_write(ring, next_rptr);

	/* insert SWITCH_BUFFER packet before first IB in the ring frame */
	if (need_ctx_switch) {
		amdgpu_ring_write(ring, PACKET3(PACKET3_SWITCH_BUFFER, 0));
		amdgpu_ring_write(ring, 0);
	}

	if (ib->flags & AMDGPU_IB_FLAG_CE)
		header = PACKET3(PACKET3_INDIRECT_BUFFER_CONST, 2);
	else
		header = PACKET3(PACKET3_INDIRECT_BUFFER, 2);

	control |= ib->length_dw |
		(ib->vm ? (ib->vm->ids[ring->idx].id << 24) : 0);

	amdgpu_ring_write(ring, header);
	amdgpu_ring_write(ring,
#ifdef __BIG_ENDIAN
			  (2 << 0) |
#endif
			  (ib->gpu_addr & 0xFFFFFFFC));
	amdgpu_ring_write(ring, upper_32_bits(ib->gpu_addr) & 0xFFFF);
	amdgpu_ring_write(ring, control);
}

static void gfx_v7_0_ring_emit_ib_compute(struct amdgpu_ring *ring,
				  struct amdgpu_ib *ib)
{
	u32 header, control = 0;
	u32 next_rptr = ring->wptr + 5;

	control |= INDIRECT_BUFFER_VALID;
	next_rptr += 4;
	amdgpu_ring_write(ring, PACKET3(PACKET3_WRITE_DATA, 3));
	amdgpu_ring_write(ring, WRITE_DATA_DST_SEL(5) | WR_CONFIRM);
	amdgpu_ring_write(ring, ring->next_rptr_gpu_addr & 0xfffffffc);
	amdgpu_ring_write(ring, upper_32_bits(ring->next_rptr_gpu_addr) & 0xffffffff);
	amdgpu_ring_write(ring, next_rptr);

	header = PACKET3(PACKET3_INDIRECT_BUFFER, 2);

	control |= ib->length_dw |
			   (ib->vm ? (ib->vm->ids[ring->idx].id << 24) : 0);

	amdgpu_ring_write(ring, header);
	amdgpu_ring_write(ring,
#ifdef __BIG_ENDIAN
					  (2 << 0) |
#endif
					  (ib->gpu_addr & 0xFFFFFFFC));
	amdgpu_ring_write(ring, upper_32_bits(ib->gpu_addr) & 0xFFFF);
	amdgpu_ring_write(ring, control);
}

/**
 * gfx_v7_0_ring_test_ib - basic ring IB test
 *
 * @ring: amdgpu_ring structure holding ring information
 *
 * Allocate an IB and execute it on the gfx ring (CIK).
 * Provides a basic gfx ring test to verify that IBs are working.
 * Returns 0 on success, error on failure.
 */
static int gfx_v7_0_ring_test_ib(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;
	struct amdgpu_ib ib;
	struct fence *f = NULL;
	uint32_t scratch;
	uint32_t tmp = 0;
	unsigned i;
	int r;

	r = amdgpu_gfx_scratch_get(adev, &scratch);
	if (r) {
		DRM_ERROR("amdgpu: failed to get scratch reg (%d).\n", r);
		return r;
	}
	WREG32(scratch, 0xCAFEDEAD);
	memset(&ib, 0, sizeof(ib));
	r = amdgpu_ib_get(ring, NULL, 256, &ib);
	if (r) {
		DRM_ERROR("amdgpu: failed to get ib (%d).\n", r);
		goto err1;
	}
	ib.ptr[0] = PACKET3(PACKET3_SET_UCONFIG_REG, 1);
	ib.ptr[1] = ((scratch - PACKET3_SET_UCONFIG_REG_START));
	ib.ptr[2] = 0xDEADBEEF;
	ib.length_dw = 3;

	r = amdgpu_sched_ib_submit_kernel_helper(adev, ring, &ib, 1, NULL,
						 AMDGPU_FENCE_OWNER_UNDEFINED,
						 &f);
	if (r)
		goto err2;

	r = fence_wait(f, false);
	if (r) {
		DRM_ERROR("amdgpu: fence wait failed (%d).\n", r);
		goto err2;
	}
	for (i = 0; i < adev->usec_timeout; i++) {
		tmp = RREG32(scratch);
		if (tmp == 0xDEADBEEF)
			break;
		DRM_UDELAY(1);
	}
	if (i < adev->usec_timeout) {
		DRM_INFO("ib test on ring %d succeeded in %u usecs\n",
			 ring->idx, i);
		goto err2;
	} else {
		DRM_ERROR("amdgpu: ib test failed (scratch(0x%04X)=0x%08X)\n",
			  scratch, tmp);
		r = -EINVAL;
	}

err2:
	fence_put(f);
	amdgpu_ib_free(adev, &ib);
err1:
	amdgpu_gfx_scratch_free(adev, scratch);
	return r;
}

/*
 * CP.
 * On CIK, gfx and compute now have independant command processors.
 *
 * GFX
 * Gfx consists of a single ring and can process both gfx jobs and
 * compute jobs.  The gfx CP consists of three microengines (ME):
 * PFP - Pre-Fetch Parser
 * ME - Micro Engine
 * CE - Constant Engine
 * The PFP and ME make up what is considered the Drawing Engine (DE).
 * The CE is an asynchronous engine used for updating buffer desciptors
 * used by the DE so that they can be loaded into cache in parallel
 * while the DE is processing state update packets.
 *
 * Compute
 * The compute CP consists of two microengines (ME):
 * MEC1 - Compute MicroEngine 1
 * MEC2 - Compute MicroEngine 2
 * Each MEC supports 4 compute pipes and each pipe supports 8 queues.
 * The queues are exposed to userspace and are programmed directly
 * by the compute runtime.
 */
/**
 * gfx_v7_0_cp_gfx_enable - enable/disable the gfx CP MEs
 *
 * @adev: amdgpu_device pointer
 * @enable: enable or disable the MEs
 *
 * Halts or unhalts the gfx MEs.
 */
static void gfx_v7_0_cp_gfx_enable(struct amdgpu_device *adev, bool enable)
{
	int i;

	if (enable) {
		WREG32(mmCP_ME_CNTL, 0);
	} else {
		WREG32(mmCP_ME_CNTL, (CP_ME_CNTL__ME_HALT_MASK | CP_ME_CNTL__PFP_HALT_MASK | CP_ME_CNTL__CE_HALT_MASK));
		for (i = 0; i < adev->gfx.num_gfx_rings; i++)
			adev->gfx.gfx_ring[i].ready = false;
	}
	udelay(50);
}

/**
 * gfx_v7_0_cp_gfx_load_microcode - load the gfx CP ME ucode
 *
 * @adev: amdgpu_device pointer
 *
 * Loads the gfx PFP, ME, and CE ucode.
 * Returns 0 for success, -EINVAL if the ucode is not available.
 */
static int gfx_v7_0_cp_gfx_load_microcode(struct amdgpu_device *adev)
{
	const struct gfx_firmware_header_v1_0 *pfp_hdr;
	const struct gfx_firmware_header_v1_0 *ce_hdr;
	const struct gfx_firmware_header_v1_0 *me_hdr;
	const __le32 *fw_data;
	unsigned i, fw_size;

	if (!adev->gfx.me_fw || !adev->gfx.pfp_fw || !adev->gfx.ce_fw)
		return -EINVAL;

	pfp_hdr = (const struct gfx_firmware_header_v1_0 *)adev->gfx.pfp_fw->data;
	ce_hdr = (const struct gfx_firmware_header_v1_0 *)adev->gfx.ce_fw->data;
	me_hdr = (const struct gfx_firmware_header_v1_0 *)adev->gfx.me_fw->data;

	amdgpu_ucode_print_gfx_hdr(&pfp_hdr->header);
	amdgpu_ucode_print_gfx_hdr(&ce_hdr->header);
	amdgpu_ucode_print_gfx_hdr(&me_hdr->header);
	adev->gfx.pfp_fw_version = le32_to_cpu(pfp_hdr->header.ucode_version);
	adev->gfx.ce_fw_version = le32_to_cpu(ce_hdr->header.ucode_version);
	adev->gfx.me_fw_version = le32_to_cpu(me_hdr->header.ucode_version);
	adev->gfx.me_feature_version = le32_to_cpu(me_hdr->ucode_feature_version);
	adev->gfx.ce_feature_version = le32_to_cpu(ce_hdr->ucode_feature_version);
	adev->gfx.pfp_feature_version = le32_to_cpu(pfp_hdr->ucode_feature_version);

	gfx_v7_0_cp_gfx_enable(adev, false);

	/* PFP */
	fw_data = (const __le32 *)
		(adev->gfx.pfp_fw->data +
		 le32_to_cpu(pfp_hdr->header.ucode_array_offset_bytes));
	fw_size = le32_to_cpu(pfp_hdr->header.ucode_size_bytes) / 4;
	WREG32(mmCP_PFP_UCODE_ADDR, 0);
	for (i = 0; i < fw_size; i++)
		WREG32(mmCP_PFP_UCODE_DATA, le32_to_cpup(fw_data++));
	WREG32(mmCP_PFP_UCODE_ADDR, adev->gfx.pfp_fw_version);

	/* CE */
	fw_data = (const __le32 *)
		(adev->gfx.ce_fw->data +
		 le32_to_cpu(ce_hdr->header.ucode_array_offset_bytes));
	fw_size = le32_to_cpu(ce_hdr->header.ucode_size_bytes) / 4;
	WREG32(mmCP_CE_UCODE_ADDR, 0);
	for (i = 0; i < fw_size; i++)
		WREG32(mmCP_CE_UCODE_DATA, le32_to_cpup(fw_data++));
	WREG32(mmCP_CE_UCODE_ADDR, adev->gfx.ce_fw_version);

	/* ME */
	fw_data = (const __le32 *)
		(adev->gfx.me_fw->data +
		 le32_to_cpu(me_hdr->header.ucode_array_offset_bytes));
	fw_size = le32_to_cpu(me_hdr->header.ucode_size_bytes) / 4;
	WREG32(mmCP_ME_RAM_WADDR, 0);
	for (i = 0; i < fw_size; i++)
		WREG32(mmCP_ME_RAM_DATA, le32_to_cpup(fw_data++));
	WREG32(mmCP_ME_RAM_WADDR, adev->gfx.me_fw_version);

	return 0;
}

/**
 * gfx_v7_0_cp_gfx_start - start the gfx ring
 *
 * @adev: amdgpu_device pointer
 *
 * Enables the ring and loads the clear state context and other
 * packets required to init the ring.
 * Returns 0 for success, error for failure.
 */
static int gfx_v7_0_cp_gfx_start(struct amdgpu_device *adev)
{
	struct amdgpu_ring *ring = &adev->gfx.gfx_ring[0];
	const struct cs_section_def *sect = NULL;
	const struct cs_extent_def *ext = NULL;
	int r, i;

	/* init the CP */
	WREG32(mmCP_MAX_CONTEXT, adev->gfx.config.max_hw_contexts - 1);
	WREG32(mmCP_ENDIAN_SWAP, 0);
	WREG32(mmCP_DEVICE_ID, 1);

	gfx_v7_0_cp_gfx_enable(adev, true);

	r = amdgpu_ring_lock(ring, gfx_v7_0_get_csb_size(adev) + 8);
	if (r) {
		DRM_ERROR("amdgpu: cp failed to lock ring (%d).\n", r);
		return r;
	}

	/* init the CE partitions.  CE only used for gfx on CIK */
	amdgpu_ring_write(ring, PACKET3(PACKET3_SET_BASE, 2));
	amdgpu_ring_write(ring, PACKET3_BASE_INDEX(CE_PARTITION_BASE));
	amdgpu_ring_write(ring, 0x8000);
	amdgpu_ring_write(ring, 0x8000);

	/* clear state buffer */
	amdgpu_ring_write(ring, PACKET3(PACKET3_PREAMBLE_CNTL, 0));
	amdgpu_ring_write(ring, PACKET3_PREAMBLE_BEGIN_CLEAR_STATE);

	amdgpu_ring_write(ring, PACKET3(PACKET3_CONTEXT_CONTROL, 1));
	amdgpu_ring_write(ring, 0x80000000);
	amdgpu_ring_write(ring, 0x80000000);

	for (sect = adev->gfx.rlc.cs_data; sect->section != NULL; ++sect) {
		for (ext = sect->section; ext->extent != NULL; ++ext) {
			if (sect->id == SECT_CONTEXT) {
				amdgpu_ring_write(ring,
						  PACKET3(PACKET3_SET_CONTEXT_REG, ext->reg_count));
				amdgpu_ring_write(ring, ext->reg_index - PACKET3_SET_CONTEXT_REG_START);
				for (i = 0; i < ext->reg_count; i++)
					amdgpu_ring_write(ring, ext->extent[i]);
			}
		}
	}

	amdgpu_ring_write(ring, PACKET3(PACKET3_SET_CONTEXT_REG, 2));
	amdgpu_ring_write(ring, mmPA_SC_RASTER_CONFIG - PACKET3_SET_CONTEXT_REG_START);
	switch (adev->asic_type) {
	case CHIP_BONAIRE:
		amdgpu_ring_write(ring, 0x16000012);
		amdgpu_ring_write(ring, 0x00000000);
		break;
	case CHIP_KAVERI:
		amdgpu_ring_write(ring, 0x00000000); /* XXX */
		amdgpu_ring_write(ring, 0x00000000);
		break;
	case CHIP_KABINI:
	case CHIP_MULLINS:
		amdgpu_ring_write(ring, 0x00000000); /* XXX */
		amdgpu_ring_write(ring, 0x00000000);
		break;
	case CHIP_HAWAII:
		amdgpu_ring_write(ring, 0x3a00161a);
		amdgpu_ring_write(ring, 0x0000002e);
		break;
	default:
		amdgpu_ring_write(ring, 0x00000000);
		amdgpu_ring_write(ring, 0x00000000);
		break;
	}

	amdgpu_ring_write(ring, PACKET3(PACKET3_PREAMBLE_CNTL, 0));
	amdgpu_ring_write(ring, PACKET3_PREAMBLE_END_CLEAR_STATE);

	amdgpu_ring_write(ring, PACKET3(PACKET3_CLEAR_STATE, 0));
	amdgpu_ring_write(ring, 0);

	amdgpu_ring_write(ring, PACKET3(PACKET3_SET_CONTEXT_REG, 2));
	amdgpu_ring_write(ring, 0x00000316);
	amdgpu_ring_write(ring, 0x0000000e); /* VGT_VERTEX_REUSE_BLOCK_CNTL */
	amdgpu_ring_write(ring, 0x00000010); /* VGT_OUT_DEALLOC_CNTL */

	amdgpu_ring_unlock_commit(ring);

	return 0;
}

/**
 * gfx_v7_0_cp_gfx_resume - setup the gfx ring buffer registers
 *
 * @adev: amdgpu_device pointer
 *
 * Program the location and size of the gfx ring buffer
 * and test it to make sure it's working.
 * Returns 0 for success, error for failure.
 */
static int gfx_v7_0_cp_gfx_resume(struct amdgpu_device *adev)
{
	struct amdgpu_ring *ring;
	u32 tmp;
	u32 rb_bufsz;
	u64 rb_addr, rptr_addr;
	int r;

	WREG32(mmCP_SEM_WAIT_TIMER, 0x0);
	if (adev->asic_type != CHIP_HAWAII)
		WREG32(mmCP_SEM_INCOMPLETE_TIMER_CNTL, 0x0);

	/* Set the write pointer delay */
	WREG32(mmCP_RB_WPTR_DELAY, 0);

	/* set the RB to use vmid 0 */
	WREG32(mmCP_RB_VMID, 0);

	WREG32(mmSCRATCH_ADDR, 0);

	/* ring 0 - compute and gfx */
	/* Set ring buffer size */
	ring = &adev->gfx.gfx_ring[0];
	rb_bufsz = order_base_2(ring->ring_size / 8);
	tmp = (order_base_2(AMDGPU_GPU_PAGE_SIZE/8) << 8) | rb_bufsz;
#ifdef __BIG_ENDIAN
	tmp |= 2 << CP_RB0_CNTL__BUF_SWAP__SHIFT;
#endif
	WREG32(mmCP_RB0_CNTL, tmp);

	/* Initialize the ring buffer's read and write pointers */
	WREG32(mmCP_RB0_CNTL, tmp | CP_RB0_CNTL__RB_RPTR_WR_ENA_MASK);
	ring->wptr = 0;
	WREG32(mmCP_RB0_WPTR, ring->wptr);

	/* set the wb address wether it's enabled or not */
	rptr_addr = adev->wb.gpu_addr + (ring->rptr_offs * 4);
	WREG32(mmCP_RB0_RPTR_ADDR, lower_32_bits(rptr_addr));
	WREG32(mmCP_RB0_RPTR_ADDR_HI, upper_32_bits(rptr_addr) & 0xFF);

	/* scratch register shadowing is no longer supported */
	WREG32(mmSCRATCH_UMSK, 0);

	mdelay(1);
	WREG32(mmCP_RB0_CNTL, tmp);

	rb_addr = ring->gpu_addr >> 8;
	WREG32(mmCP_RB0_BASE, rb_addr);
	WREG32(mmCP_RB0_BASE_HI, upper_32_bits(rb_addr));

	/* start the ring */
	gfx_v7_0_cp_gfx_start(adev);
	ring->ready = true;
	r = amdgpu_ring_test_ring(ring);
	if (r) {
		ring->ready = false;
		return r;
	}

	return 0;
}

static u32 gfx_v7_0_ring_get_rptr_gfx(struct amdgpu_ring *ring)
{
	u32 rptr;

	rptr = ring->adev->wb.wb[ring->rptr_offs];

	return rptr;
}

static u32 gfx_v7_0_ring_get_wptr_gfx(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;
	u32 wptr;

	wptr = RREG32(mmCP_RB0_WPTR);

	return wptr;
}

static void gfx_v7_0_ring_set_wptr_gfx(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;

	WREG32(mmCP_RB0_WPTR, ring->wptr);
	(void)RREG32(mmCP_RB0_WPTR);
}

static u32 gfx_v7_0_ring_get_rptr_compute(struct amdgpu_ring *ring)
{
	u32 rptr;

	rptr = ring->adev->wb.wb[ring->rptr_offs];

	return rptr;
}

static u32 gfx_v7_0_ring_get_wptr_compute(struct amdgpu_ring *ring)
{
	u32 wptr;

	/* XXX check if swapping is necessary on BE */
	wptr = ring->adev->wb.wb[ring->wptr_offs];

	return wptr;
}

static void gfx_v7_0_ring_set_wptr_compute(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;

	/* XXX check if swapping is necessary on BE */
	adev->wb.wb[ring->wptr_offs] = ring->wptr;
	WDOORBELL32(ring->doorbell_index, ring->wptr);
}

/**
 * gfx_v7_0_cp_compute_enable - enable/disable the compute CP MEs
 *
 * @adev: amdgpu_device pointer
 * @enable: enable or disable the MEs
 *
 * Halts or unhalts the compute MEs.
 */
static void gfx_v7_0_cp_compute_enable(struct amdgpu_device *adev, bool enable)
{
	int i;

	if (enable) {
		WREG32(mmCP_MEC_CNTL, 0);
	} else {
		WREG32(mmCP_MEC_CNTL, (CP_MEC_CNTL__MEC_ME1_HALT_MASK | CP_MEC_CNTL__MEC_ME2_HALT_MASK));
		for (i = 0; i < adev->gfx.num_compute_rings; i++)
			adev->gfx.compute_ring[i].ready = false;
	}
	udelay(50);
}

/**
 * gfx_v7_0_cp_compute_load_microcode - load the compute CP ME ucode
 *
 * @adev: amdgpu_device pointer
 *
 * Loads the compute MEC1&2 ucode.
 * Returns 0 for success, -EINVAL if the ucode is not available.
 */
static int gfx_v7_0_cp_compute_load_microcode(struct amdgpu_device *adev)
{
	const struct gfx_firmware_header_v1_0 *mec_hdr;
	const __le32 *fw_data;
	unsigned i, fw_size;

	if (!adev->gfx.mec_fw)
		return -EINVAL;

	mec_hdr = (const struct gfx_firmware_header_v1_0 *)adev->gfx.mec_fw->data;
	amdgpu_ucode_print_gfx_hdr(&mec_hdr->header);
	adev->gfx.mec_fw_version = le32_to_cpu(mec_hdr->header.ucode_version);
	adev->gfx.mec_feature_version = le32_to_cpu(
					mec_hdr->ucode_feature_version);

	gfx_v7_0_cp_compute_enable(adev, false);

	/* MEC1 */
	fw_data = (const __le32 *)
		(adev->gfx.mec_fw->data +
		 le32_to_cpu(mec_hdr->header.ucode_array_offset_bytes));
	fw_size = le32_to_cpu(mec_hdr->header.ucode_size_bytes) / 4;
	WREG32(mmCP_MEC_ME1_UCODE_ADDR, 0);
	for (i = 0; i < fw_size; i++)
		WREG32(mmCP_MEC_ME1_UCODE_DATA, le32_to_cpup(fw_data++));
	WREG32(mmCP_MEC_ME1_UCODE_ADDR, 0);

	if (adev->asic_type == CHIP_KAVERI) {
		const struct gfx_firmware_header_v1_0 *mec2_hdr;

		if (!adev->gfx.mec2_fw)
			return -EINVAL;

		mec2_hdr = (const struct gfx_firmware_header_v1_0 *)adev->gfx.mec2_fw->data;
		amdgpu_ucode_print_gfx_hdr(&mec2_hdr->header);
		adev->gfx.mec2_fw_version = le32_to_cpu(mec2_hdr->header.ucode_version);
		adev->gfx.mec2_feature_version = le32_to_cpu(
				mec2_hdr->ucode_feature_version);

		/* MEC2 */
		fw_data = (const __le32 *)
			(adev->gfx.mec2_fw->data +
			 le32_to_cpu(mec2_hdr->header.ucode_array_offset_bytes));
		fw_size = le32_to_cpu(mec2_hdr->header.ucode_size_bytes) / 4;
		WREG32(mmCP_MEC_ME2_UCODE_ADDR, 0);
		for (i = 0; i < fw_size; i++)
			WREG32(mmCP_MEC_ME2_UCODE_DATA, le32_to_cpup(fw_data++));
		WREG32(mmCP_MEC_ME2_UCODE_ADDR, 0);
	}

	return 0;
}

/**
 * gfx_v7_0_cp_compute_start - start the compute queues
 *
 * @adev: amdgpu_device pointer
 *
 * Enable the compute queues.
 * Returns 0 for success, error for failure.
 */
static int gfx_v7_0_cp_compute_start(struct amdgpu_device *adev)
{
	gfx_v7_0_cp_compute_enable(adev, true);

	return 0;
}

/**
 * gfx_v7_0_cp_compute_fini - stop the compute queues
 *
 * @adev: amdgpu_device pointer
 *
 * Stop the compute queues and tear down the driver queue
 * info.
 */
static void gfx_v7_0_cp_compute_fini(struct amdgpu_device *adev)
{
	int i, r;

	for (i = 0; i < adev->gfx.num_compute_rings; i++) {
		struct amdgpu_ring *ring = &adev->gfx.compute_ring[i];

		if (ring->mqd_obj) {
			r = amdgpu_bo_reserve(ring->mqd_obj, false);
			if (unlikely(r != 0))
				dev_warn(adev->dev, "(%d) reserve MQD bo failed\n", r);

			amdgpu_bo_unpin(ring->mqd_obj);
			amdgpu_bo_unreserve(ring->mqd_obj);

			amdgpu_bo_unref(&ring->mqd_obj);
			ring->mqd_obj = NULL;
		}
	}
}

static void gfx_v7_0_mec_fini(struct amdgpu_device *adev)
{
	int r;

	if (adev->gfx.mec.hpd_eop_obj) {
		r = amdgpu_bo_reserve(adev->gfx.mec.hpd_eop_obj, false);
		if (unlikely(r != 0))
			dev_warn(adev->dev, "(%d) reserve HPD EOP bo failed\n", r);
		amdgpu_bo_unpin(adev->gfx.mec.hpd_eop_obj);
		amdgpu_bo_unreserve(adev->gfx.mec.hpd_eop_obj);

		amdgpu_bo_unref(&adev->gfx.mec.hpd_eop_obj);
		adev->gfx.mec.hpd_eop_obj = NULL;
	}
}

#define MEC_HPD_SIZE 2048

static int gfx_v7_0_mec_init(struct amdgpu_device *adev)
{
	int r;
	u32 *hpd;

	/*
	 * KV:    2 MEC, 4 Pipes/MEC, 8 Queues/Pipe - 64 Queues total
	 * CI/KB: 1 MEC, 4 Pipes/MEC, 8 Queues/Pipe - 32 Queues total
	 * Nonetheless, we assign only 1 pipe because all other pipes will
	 * be handled by KFD
	 */
	adev->gfx.mec.num_mec = 1;
	adev->gfx.mec.num_pipe = 1;
	adev->gfx.mec.num_queue = adev->gfx.mec.num_mec * adev->gfx.mec.num_pipe * 8;

	if (adev->gfx.mec.hpd_eop_obj == NULL) {
		r = amdgpu_bo_create(adev,
				     adev->gfx.mec.num_mec *adev->gfx.mec.num_pipe * MEC_HPD_SIZE * 2,
				     PAGE_SIZE, true,
				     AMDGPU_GEM_DOMAIN_GTT, 0, NULL, NULL,
				     &adev->gfx.mec.hpd_eop_obj);
		if (r) {
			dev_warn(adev->dev, "(%d) create HDP EOP bo failed\n", r);
			return r;
		}
	}

	r = amdgpu_bo_reserve(adev->gfx.mec.hpd_eop_obj, false);
	if (unlikely(r != 0)) {
		gfx_v7_0_mec_fini(adev);
		return r;
	}
	r = amdgpu_bo_pin(adev->gfx.mec.hpd_eop_obj, AMDGPU_GEM_DOMAIN_GTT,
			  &adev->gfx.mec.hpd_eop_gpu_addr);
	if (r) {
		dev_warn(adev->dev, "(%d) pin HDP EOP bo failed\n", r);
		gfx_v7_0_mec_fini(adev);
		return r;
	}
	r = amdgpu_bo_kmap(adev->gfx.mec.hpd_eop_obj, (void **)&hpd);
	if (r) {
		dev_warn(adev->dev, "(%d) map HDP EOP bo failed\n", r);
		gfx_v7_0_mec_fini(adev);
		return r;
	}

	/* clear memory.  Not sure if this is required or not */
	memset(hpd, 0, adev->gfx.mec.num_mec *adev->gfx.mec.num_pipe * MEC_HPD_SIZE * 2);

	amdgpu_bo_kunmap(adev->gfx.mec.hpd_eop_obj);
	amdgpu_bo_unreserve(adev->gfx.mec.hpd_eop_obj);

	return 0;
}

struct hqd_registers
{
	u32 cp_mqd_base_addr;
	u32 cp_mqd_base_addr_hi;
	u32 cp_hqd_active;
	u32 cp_hqd_vmid;
	u32 cp_hqd_persistent_state;
	u32 cp_hqd_pipe_priority;
	u32 cp_hqd_queue_priority;
	u32 cp_hqd_quantum;
	u32 cp_hqd_pq_base;
	u32 cp_hqd_pq_base_hi;
	u32 cp_hqd_pq_rptr;
	u32 cp_hqd_pq_rptr_report_addr;
	u32 cp_hqd_pq_rptr_report_addr_hi;
	u32 cp_hqd_pq_wptr_poll_addr;
	u32 cp_hqd_pq_wptr_poll_addr_hi;
	u32 cp_hqd_pq_doorbell_control;
	u32 cp_hqd_pq_wptr;
	u32 cp_hqd_pq_control;
	u32 cp_hqd_ib_base_addr;
	u32 cp_hqd_ib_base_addr_hi;
	u32 cp_hqd_ib_rptr;
	u32 cp_hqd_ib_control;
	u32 cp_hqd_iq_timer;
	u32 cp_hqd_iq_rptr;
	u32 cp_hqd_dequeue_request;
	u32 cp_hqd_dma_offload;
	u32 cp_hqd_sema_cmd;
	u32 cp_hqd_msg_type;
	u32 cp_hqd_atomic0_preop_lo;
	u32 cp_hqd_atomic0_preop_hi;
	u32 cp_hqd_atomic1_preop_lo;
	u32 cp_hqd_atomic1_preop_hi;
	u32 cp_hqd_hq_scheduler0;
	u32 cp_hqd_hq_scheduler1;
	u32 cp_mqd_control;
};

struct bonaire_mqd
{
	u32 header;
	u32 dispatch_initiator;
	u32 dimensions[3];
	u32 start_idx[3];
	u32 num_threads[3];
	u32 pipeline_stat_enable;
	u32 perf_counter_enable;
	u32 pgm[2];
	u32 tba[2];
	u32 tma[2];
	u32 pgm_rsrc[2];
	u32 vmid;
	u32 resource_limits;
	u32 static_thread_mgmt01[2];
	u32 tmp_ring_size;
	u32 static_thread_mgmt23[2];
	u32 restart[3];
	u32 thread_trace_enable;
	u32 reserved1;
	u32 user_data[16];
	u32 vgtcs_invoke_count[2];
	struct hqd_registers queue_state;
	u32 dequeue_cntr;
	u32 interrupt_queue[64];
};

/**
 * gfx_v7_0_cp_compute_resume - setup the compute queue registers
 *
 * @adev: amdgpu_device pointer
 *
 * Program the compute queues and test them to make sure they
 * are working.
 * Returns 0 for success, error for failure.
 */
static int gfx_v7_0_cp_compute_resume(struct amdgpu_device *adev)
{
	int r, i, j;
	u32 tmp;
	bool use_doorbell = true;
	u64 hqd_gpu_addr;
	u64 mqd_gpu_addr;
	u64 eop_gpu_addr;
	u64 wb_gpu_addr;
	u32 *buf;
	struct bonaire_mqd *mqd;

	r = gfx_v7_0_cp_compute_start(adev);
	if (r)
		return r;

	/* fix up chicken bits */
	tmp = RREG32(mmCP_CPF_DEBUG);
	tmp |= (1 << 23);
	WREG32(mmCP_CPF_DEBUG, tmp);

	/* init the pipes */
	mutex_lock(&adev->srbm_mutex);
	for (i = 0; i < (adev->gfx.mec.num_pipe * adev->gfx.mec.num_mec); i++) {
		int me = (i < 4) ? 1 : 2;
		int pipe = (i < 4) ? i : (i - 4);

		eop_gpu_addr = adev->gfx.mec.hpd_eop_gpu_addr + (i * MEC_HPD_SIZE * 2);

		cik_srbm_select(adev, me, pipe, 0, 0);

		/* write the EOP addr */
		WREG32(mmCP_HPD_EOP_BASE_ADDR, eop_gpu_addr >> 8);
		WREG32(mmCP_HPD_EOP_BASE_ADDR_HI, upper_32_bits(eop_gpu_addr) >> 8);

		/* set the VMID assigned */
		WREG32(mmCP_HPD_EOP_VMID, 0);

		/* set the EOP size, register value is 2^(EOP_SIZE+1) dwords */
		tmp = RREG32(mmCP_HPD_EOP_CONTROL);
		tmp &= ~CP_HPD_EOP_CONTROL__EOP_SIZE_MASK;
		tmp |= order_base_2(MEC_HPD_SIZE / 8);
		WREG32(mmCP_HPD_EOP_CONTROL, tmp);
	}
	cik_srbm_select(adev, 0, 0, 0, 0);
	mutex_unlock(&adev->srbm_mutex);

	/* init the queues.  Just two for now. */
	for (i = 0; i < adev->gfx.num_compute_rings; i++) {
		struct amdgpu_ring *ring = &adev->gfx.compute_ring[i];

		if (ring->mqd_obj == NULL) {
			r = amdgpu_bo_create(adev,
					     sizeof(struct bonaire_mqd),
					     PAGE_SIZE, true,
					     AMDGPU_GEM_DOMAIN_GTT, 0, NULL, NULL,
					     &ring->mqd_obj);
			if (r) {
				dev_warn(adev->dev, "(%d) create MQD bo failed\n", r);
				return r;
			}
		}

		r = amdgpu_bo_reserve(ring->mqd_obj, false);
		if (unlikely(r != 0)) {
			gfx_v7_0_cp_compute_fini(adev);
			return r;
		}
		r = amdgpu_bo_pin(ring->mqd_obj, AMDGPU_GEM_DOMAIN_GTT,
				  &mqd_gpu_addr);
		if (r) {
			dev_warn(adev->dev, "(%d) pin MQD bo failed\n", r);
			gfx_v7_0_cp_compute_fini(adev);
			return r;
		}
		r = amdgpu_bo_kmap(ring->mqd_obj, (void **)&buf);
		if (r) {
			dev_warn(adev->dev, "(%d) map MQD bo failed\n", r);
			gfx_v7_0_cp_compute_fini(adev);
			return r;
		}

		/* init the mqd struct */
		memset(buf, 0, sizeof(struct bonaire_mqd));

		mqd = (struct bonaire_mqd *)buf;
		mqd->header = 0xC0310800;
		mqd->static_thread_mgmt01[0] = 0xffffffff;
		mqd->static_thread_mgmt01[1] = 0xffffffff;
		mqd->static_thread_mgmt23[0] = 0xffffffff;
		mqd->static_thread_mgmt23[1] = 0xffffffff;

		mutex_lock(&adev->srbm_mutex);
		cik_srbm_select(adev, ring->me,
				ring->pipe,
				ring->queue, 0);

		/* disable wptr polling */
		tmp = RREG32(mmCP_PQ_WPTR_POLL_CNTL);
		tmp &= ~CP_PQ_WPTR_POLL_CNTL__EN_MASK;
		WREG32(mmCP_PQ_WPTR_POLL_CNTL, tmp);

		/* enable doorbell? */
		mqd->queue_state.cp_hqd_pq_doorbell_control =
			RREG32(mmCP_HQD_PQ_DOORBELL_CONTROL);
		if (use_doorbell)
			mqd->queue_state.cp_hqd_pq_doorbell_control |= CP_HQD_PQ_DOORBELL_CONTROL__DOORBELL_EN_MASK;
		else
			mqd->queue_state.cp_hqd_pq_doorbell_control &= ~CP_HQD_PQ_DOORBELL_CONTROL__DOORBELL_EN_MASK;
		WREG32(mmCP_HQD_PQ_DOORBELL_CONTROL,
		       mqd->queue_state.cp_hqd_pq_doorbell_control);

		/* disable the queue if it's active */
		mqd->queue_state.cp_hqd_dequeue_request = 0;
		mqd->queue_state.cp_hqd_pq_rptr = 0;
		mqd->queue_state.cp_hqd_pq_wptr= 0;
		if (RREG32(mmCP_HQD_ACTIVE) & 1) {
			WREG32(mmCP_HQD_DEQUEUE_REQUEST, 1);
			for (j = 0; j < adev->usec_timeout; j++) {
				if (!(RREG32(mmCP_HQD_ACTIVE) & 1))
					break;
				udelay(1);
			}
			WREG32(mmCP_HQD_DEQUEUE_REQUEST, mqd->queue_state.cp_hqd_dequeue_request);
			WREG32(mmCP_HQD_PQ_RPTR, mqd->queue_state.cp_hqd_pq_rptr);
			WREG32(mmCP_HQD_PQ_WPTR, mqd->queue_state.cp_hqd_pq_wptr);
		}

		/* set the pointer to the MQD */
		mqd->queue_state.cp_mqd_base_addr = mqd_gpu_addr & 0xfffffffc;
		mqd->queue_state.cp_mqd_base_addr_hi = upper_32_bits(mqd_gpu_addr);
		WREG32(mmCP_MQD_BASE_ADDR, mqd->queue_state.cp_mqd_base_addr);
		WREG32(mmCP_MQD_BASE_ADDR_HI, mqd->queue_state.cp_mqd_base_addr_hi);
		/* set MQD vmid to 0 */
		mqd->queue_state.cp_mqd_control = RREG32(mmCP_MQD_CONTROL);
		mqd->queue_state.cp_mqd_control &= ~CP_MQD_CONTROL__VMID_MASK;
		WREG32(mmCP_MQD_CONTROL, mqd->queue_state.cp_mqd_control);

		/* set the pointer to the HQD, this is similar CP_RB0_BASE/_HI */
		hqd_gpu_addr = ring->gpu_addr >> 8;
		mqd->queue_state.cp_hqd_pq_base = hqd_gpu_addr;
		mqd->queue_state.cp_hqd_pq_base_hi = upper_32_bits(hqd_gpu_addr);
		WREG32(mmCP_HQD_PQ_BASE, mqd->queue_state.cp_hqd_pq_base);
		WREG32(mmCP_HQD_PQ_BASE_HI, mqd->queue_state.cp_hqd_pq_base_hi);

		/* set up the HQD, this is similar to CP_RB0_CNTL */
		mqd->queue_state.cp_hqd_pq_control = RREG32(mmCP_HQD_PQ_CONTROL);
		mqd->queue_state.cp_hqd_pq_control &=
			~(CP_HQD_PQ_CONTROL__QUEUE_SIZE_MASK |
					CP_HQD_PQ_CONTROL__RPTR_BLOCK_SIZE_MASK);

		mqd->queue_state.cp_hqd_pq_control |=
			order_base_2(ring->ring_size / 8);
		mqd->queue_state.cp_hqd_pq_control |=
			(order_base_2(AMDGPU_GPU_PAGE_SIZE/8) << 8);
#ifdef __BIG_ENDIAN
		mqd->queue_state.cp_hqd_pq_control |=
			2 << CP_HQD_PQ_CONTROL__ENDIAN_SWAP__SHIFT;
#endif
		mqd->queue_state.cp_hqd_pq_control &=
			~(CP_HQD_PQ_CONTROL__UNORD_DISPATCH_MASK |
				CP_HQD_PQ_CONTROL__ROQ_PQ_IB_FLIP_MASK |
				CP_HQD_PQ_CONTROL__PQ_VOLATILE_MASK);
		mqd->queue_state.cp_hqd_pq_control |=
			CP_HQD_PQ_CONTROL__PRIV_STATE_MASK |
			CP_HQD_PQ_CONTROL__KMD_QUEUE_MASK; /* assuming kernel queue control */
		WREG32(mmCP_HQD_PQ_CONTROL, mqd->queue_state.cp_hqd_pq_control);

		/* only used if CP_PQ_WPTR_POLL_CNTL.CP_PQ_WPTR_POLL_CNTL__EN_MASK=1 */
		wb_gpu_addr = adev->wb.gpu_addr + (ring->wptr_offs * 4);
		mqd->queue_state.cp_hqd_pq_wptr_poll_addr = wb_gpu_addr & 0xfffffffc;
		mqd->queue_state.cp_hqd_pq_wptr_poll_addr_hi = upper_32_bits(wb_gpu_addr) & 0xffff;
		WREG32(mmCP_HQD_PQ_WPTR_POLL_ADDR, mqd->queue_state.cp_hqd_pq_wptr_poll_addr);
		WREG32(mmCP_HQD_PQ_WPTR_POLL_ADDR_HI,
		       mqd->queue_state.cp_hqd_pq_wptr_poll_addr_hi);

		/* set the wb address wether it's enabled or not */
		wb_gpu_addr = adev->wb.gpu_addr + (ring->rptr_offs * 4);
		mqd->queue_state.cp_hqd_pq_rptr_report_addr = wb_gpu_addr & 0xfffffffc;
		mqd->queue_state.cp_hqd_pq_rptr_report_addr_hi =
			upper_32_bits(wb_gpu_addr) & 0xffff;
		WREG32(mmCP_HQD_PQ_RPTR_REPORT_ADDR,
		       mqd->queue_state.cp_hqd_pq_rptr_report_addr);
		WREG32(mmCP_HQD_PQ_RPTR_REPORT_ADDR_HI,
		       mqd->queue_state.cp_hqd_pq_rptr_report_addr_hi);

		/* enable the doorbell if requested */
		if (use_doorbell) {
			mqd->queue_state.cp_hqd_pq_doorbell_control =
				RREG32(mmCP_HQD_PQ_DOORBELL_CONTROL);
			mqd->queue_state.cp_hqd_pq_doorbell_control &=
				~CP_HQD_PQ_DOORBELL_CONTROL__DOORBELL_OFFSET_MASK;
			mqd->queue_state.cp_hqd_pq_doorbell_control |=
				(ring->doorbell_index <<
				 CP_HQD_PQ_DOORBELL_CONTROL__DOORBELL_OFFSET__SHIFT);
			mqd->queue_state.cp_hqd_pq_doorbell_control |=
				CP_HQD_PQ_DOORBELL_CONTROL__DOORBELL_EN_MASK;
			mqd->queue_state.cp_hqd_pq_doorbell_control &=
				~(CP_HQD_PQ_DOORBELL_CONTROL__DOORBELL_SOURCE_MASK |
				CP_HQD_PQ_DOORBELL_CONTROL__DOORBELL_HIT_MASK);

		} else {
			mqd->queue_state.cp_hqd_pq_doorbell_control = 0;
		}
		WREG32(mmCP_HQD_PQ_DOORBELL_CONTROL,
		       mqd->queue_state.cp_hqd_pq_doorbell_control);

		/* read and write pointers, similar to CP_RB0_WPTR/_RPTR */
		ring->wptr = 0;
		mqd->queue_state.cp_hqd_pq_wptr = ring->wptr;
		WREG32(mmCP_HQD_PQ_WPTR, mqd->queue_state.cp_hqd_pq_wptr);
		mqd->queue_state.cp_hqd_pq_rptr = RREG32(mmCP_HQD_PQ_RPTR);

		/* set the vmid for the queue */
		mqd->queue_state.cp_hqd_vmid = 0;
		WREG32(mmCP_HQD_VMID, mqd->queue_state.cp_hqd_vmid);

		/* activate the queue */
		mqd->queue_state.cp_hqd_active = 1;
		WREG32(mmCP_HQD_ACTIVE, mqd->queue_state.cp_hqd_active);

		cik_srbm_select(adev, 0, 0, 0, 0);
		mutex_unlock(&adev->srbm_mutex);

		amdgpu_bo_kunmap(ring->mqd_obj);
		amdgpu_bo_unreserve(ring->mqd_obj);

		ring->ready = true;
		r = amdgpu_ring_test_ring(ring);
		if (r)
			ring->ready = false;
	}

	return 0;
}

static void gfx_v7_0_cp_enable(struct amdgpu_device *adev, bool enable)
{
	gfx_v7_0_cp_gfx_enable(adev, enable);
	gfx_v7_0_cp_compute_enable(adev, enable);
}

static int gfx_v7_0_cp_load_microcode(struct amdgpu_device *adev)
{
	int r;

	r = gfx_v7_0_cp_gfx_load_microcode(adev);
	if (r)
		return r;
	r = gfx_v7_0_cp_compute_load_microcode(adev);
	if (r)
		return r;

	return 0;
}

static void gfx_v7_0_enable_gui_idle_interrupt(struct amdgpu_device *adev,
					       bool enable)
{
	u32 tmp = RREG32(mmCP_INT_CNTL_RING0);

	if (enable)
		tmp |= (CP_INT_CNTL_RING0__CNTX_BUSY_INT_ENABLE_MASK |
				CP_INT_CNTL_RING0__CNTX_EMPTY_INT_ENABLE_MASK);
	else
		tmp &= ~(CP_INT_CNTL_RING0__CNTX_BUSY_INT_ENABLE_MASK |
				CP_INT_CNTL_RING0__CNTX_EMPTY_INT_ENABLE_MASK);
	WREG32(mmCP_INT_CNTL_RING0, tmp);
}

static int gfx_v7_0_cp_resume(struct amdgpu_device *adev)
{
	int r;

	gfx_v7_0_enable_gui_idle_interrupt(adev, false);

	r = gfx_v7_0_cp_load_microcode(adev);
	if (r)
		return r;

	r = gfx_v7_0_cp_gfx_resume(adev);
	if (r)
		return r;
	r = gfx_v7_0_cp_compute_resume(adev);
	if (r)
		return r;

	gfx_v7_0_enable_gui_idle_interrupt(adev, true);

	return 0;
}

/*
 * vm
 * VMID 0 is the physical GPU addresses as used by the kernel.
 * VMIDs 1-15 are used for userspace clients and are handled
 * by the amdgpu vm/hsa code.
 */
/**
 * gfx_v7_0_ring_emit_vm_flush - cik vm flush using the CP
 *
 * @adev: amdgpu_device pointer
 *
 * Update the page table base and flush the VM TLB
 * using the CP (CIK).
 */
static void gfx_v7_0_ring_emit_vm_flush(struct amdgpu_ring *ring,
					unsigned vm_id, uint64_t pd_addr)
{
	int usepfp = (ring->type == AMDGPU_RING_TYPE_GFX);
	uint32_t seq = ring->fence_drv.sync_seq;
	uint64_t addr = ring->fence_drv.gpu_addr;

	amdgpu_ring_write(ring, PACKET3(PACKET3_WAIT_REG_MEM, 5));
	amdgpu_ring_write(ring, (WAIT_REG_MEM_MEM_SPACE(1) | /* memory */
				 WAIT_REG_MEM_FUNCTION(3) | /* equal */
				 WAIT_REG_MEM_ENGINE(usepfp)));   /* pfp or me */
	amdgpu_ring_write(ring, addr & 0xfffffffc);
	amdgpu_ring_write(ring, upper_32_bits(addr) & 0xffffffff);
	amdgpu_ring_write(ring, seq);
	amdgpu_ring_write(ring, 0xffffffff);
	amdgpu_ring_write(ring, 4); /* poll interval */

	if (usepfp) {
		/* synce CE with ME to prevent CE fetch CEIB before context switch done */
		amdgpu_ring_write(ring, PACKET3(PACKET3_SWITCH_BUFFER, 0));
		amdgpu_ring_write(ring, 0);
		amdgpu_ring_write(ring, PACKET3(PACKET3_SWITCH_BUFFER, 0));
		amdgpu_ring_write(ring, 0);
	}

	amdgpu_ring_write(ring, PACKET3(PACKET3_WRITE_DATA, 3));
	amdgpu_ring_write(ring, (WRITE_DATA_ENGINE_SEL(usepfp) |
				 WRITE_DATA_DST_SEL(0)));
	if (vm_id < 8) {
		amdgpu_ring_write(ring,
				  (mmVM_CONTEXT0_PAGE_TABLE_BASE_ADDR + vm_id));
	} else {
		amdgpu_ring_write(ring,
				  (mmVM_CONTEXT8_PAGE_TABLE_BASE_ADDR + vm_id - 8));
	}
	amdgpu_ring_write(ring, 0);
	amdgpu_ring_write(ring, pd_addr >> 12);

	/* bits 0-15 are the VM contexts0-15 */
	amdgpu_ring_write(ring, PACKET3(PACKET3_WRITE_DATA, 3));
	amdgpu_ring_write(ring, (WRITE_DATA_ENGINE_SEL(0) |
				 WRITE_DATA_DST_SEL(0)));
	amdgpu_ring_write(ring, mmVM_INVALIDATE_REQUEST);
	amdgpu_ring_write(ring, 0);
	amdgpu_ring_write(ring, 1 << vm_id);

	/* wait for the invalidate to complete */
	amdgpu_ring_write(ring, PACKET3(PACKET3_WAIT_REG_MEM, 5));
	amdgpu_ring_write(ring, (WAIT_REG_MEM_OPERATION(0) | /* wait */
				 WAIT_REG_MEM_FUNCTION(0) |  /* always */
				 WAIT_REG_MEM_ENGINE(0))); /* me */
	amdgpu_ring_write(ring, mmVM_INVALIDATE_REQUEST);
	amdgpu_ring_write(ring, 0);
	amdgpu_ring_write(ring, 0); /* ref */
	amdgpu_ring_write(ring, 0); /* mask */
	amdgpu_ring_write(ring, 0x20); /* poll interval */

	/* compute doesn't have PFP */
	if (usepfp) {
		/* sync PFP to ME, otherwise we might get invalid PFP reads */
		amdgpu_ring_write(ring, PACKET3(PACKET3_PFP_SYNC_ME, 0));
		amdgpu_ring_write(ring, 0x0);

		/* synce CE with ME to prevent CE fetch CEIB before context switch done */
		amdgpu_ring_write(ring, PACKET3(PACKET3_SWITCH_BUFFER, 0));
		amdgpu_ring_write(ring, 0);
		amdgpu_ring_write(ring, PACKET3(PACKET3_SWITCH_BUFFER, 0));
		amdgpu_ring_write(ring, 0);
	}
}

/*
 * RLC
 * The RLC is a multi-purpose microengine that handles a
 * variety of functions.
 */
static void gfx_v7_0_rlc_fini(struct amdgpu_device *adev)
{
	int r;

	/* save restore block */
	if (adev->gfx.rlc.save_restore_obj) {
		r = amdgpu_bo_reserve(adev->gfx.rlc.save_restore_obj, false);
		if (unlikely(r != 0))
			dev_warn(adev->dev, "(%d) reserve RLC sr bo failed\n", r);
		amdgpu_bo_unpin(adev->gfx.rlc.save_restore_obj);
		amdgpu_bo_unreserve(adev->gfx.rlc.save_restore_obj);

		amdgpu_bo_unref(&adev->gfx.rlc.save_restore_obj);
		adev->gfx.rlc.save_restore_obj = NULL;
	}

	/* clear state block */
	if (adev->gfx.rlc.clear_state_obj) {
		r = amdgpu_bo_reserve(adev->gfx.rlc.clear_state_obj, false);
		if (unlikely(r != 0))
			dev_warn(adev->dev, "(%d) reserve RLC c bo failed\n", r);
		amdgpu_bo_unpin(adev->gfx.rlc.clear_state_obj);
		amdgpu_bo_unreserve(adev->gfx.rlc.clear_state_obj);

		amdgpu_bo_unref(&adev->gfx.rlc.clear_state_obj);
		adev->gfx.rlc.clear_state_obj = NULL;
	}

	/* clear state block */
	if (adev->gfx.rlc.cp_table_obj) {
		r = amdgpu_bo_reserve(adev->gfx.rlc.cp_table_obj, false);
		if (unlikely(r != 0))
			dev_warn(adev->dev, "(%d) reserve RLC cp table bo failed\n", r);
		amdgpu_bo_unpin(adev->gfx.rlc.cp_table_obj);
		amdgpu_bo_unreserve(adev->gfx.rlc.cp_table_obj);

		amdgpu_bo_unref(&adev->gfx.rlc.cp_table_obj);
		adev->gfx.rlc.cp_table_obj = NULL;
	}
}

static int gfx_v7_0_rlc_init(struct amdgpu_device *adev)
{
	const u32 *src_ptr;
	volatile u32 *dst_ptr;
	u32 dws, i;
	const struct cs_section_def *cs_data;
	int r;

	/* allocate rlc buffers */
	if (adev->flags & AMD_IS_APU) {
		if (adev->asic_type == CHIP_KAVERI) {
			adev->gfx.rlc.reg_list = spectre_rlc_save_restore_register_list;
			adev->gfx.rlc.reg_list_size =
				(u32)ARRAY_SIZE(spectre_rlc_save_restore_register_list);
		} else {
			adev->gfx.rlc.reg_list = kalindi_rlc_save_restore_register_list;
			adev->gfx.rlc.reg_list_size =
				(u32)ARRAY_SIZE(kalindi_rlc_save_restore_register_list);
		}
	}
	adev->gfx.rlc.cs_data = ci_cs_data;
	adev->gfx.rlc.cp_table_size = CP_ME_TABLE_SIZE * 5 * 4;

	src_ptr = adev->gfx.rlc.reg_list;
	dws = adev->gfx.rlc.reg_list_size;
	dws += (5 * 16) + 48 + 48 + 64;

	cs_data = adev->gfx.rlc.cs_data;

	if (src_ptr) {
		/* save restore block */
		if (adev->gfx.rlc.save_restore_obj == NULL) {
			r = amdgpu_bo_create(adev, dws * 4, PAGE_SIZE, true,
					     AMDGPU_GEM_DOMAIN_VRAM,
					     AMDGPU_GEM_CREATE_CPU_ACCESS_REQUIRED,
					     NULL, NULL,
					     &adev->gfx.rlc.save_restore_obj);
			if (r) {
				dev_warn(adev->dev, "(%d) create RLC sr bo failed\n", r);
				return r;
			}
		}

		r = amdgpu_bo_reserve(adev->gfx.rlc.save_restore_obj, false);
		if (unlikely(r != 0)) {
			gfx_v7_0_rlc_fini(adev);
			return r;
		}
		r = amdgpu_bo_pin(adev->gfx.rlc.save_restore_obj, AMDGPU_GEM_DOMAIN_VRAM,
				  &adev->gfx.rlc.save_restore_gpu_addr);
		if (r) {
			amdgpu_bo_unreserve(adev->gfx.rlc.save_restore_obj);
			dev_warn(adev->dev, "(%d) pin RLC sr bo failed\n", r);
			gfx_v7_0_rlc_fini(adev);
			return r;
		}

		r = amdgpu_bo_kmap(adev->gfx.rlc.save_restore_obj, (void **)&adev->gfx.rlc.sr_ptr);
		if (r) {
			dev_warn(adev->dev, "(%d) map RLC sr bo failed\n", r);
			gfx_v7_0_rlc_fini(adev);
			return r;
		}
		/* write the sr buffer */
		dst_ptr = adev->gfx.rlc.sr_ptr;
		for (i = 0; i < adev->gfx.rlc.reg_list_size; i++)
			dst_ptr[i] = cpu_to_le32(src_ptr[i]);
		amdgpu_bo_kunmap(adev->gfx.rlc.save_restore_obj);
		amdgpu_bo_unreserve(adev->gfx.rlc.save_restore_obj);
	}

	if (cs_data) {
		/* clear state block */
		adev->gfx.rlc.clear_state_size = dws = gfx_v7_0_get_csb_size(adev);

		if (adev->gfx.rlc.clear_state_obj == NULL) {
			r = amdgpu_bo_create(adev, dws * 4, PAGE_SIZE, true,
					     AMDGPU_GEM_DOMAIN_VRAM,
					     AMDGPU_GEM_CREATE_CPU_ACCESS_REQUIRED,
					     NULL, NULL,
					     &adev->gfx.rlc.clear_state_obj);
			if (r) {
				dev_warn(adev->dev, "(%d) create RLC c bo failed\n", r);
				gfx_v7_0_rlc_fini(adev);
				return r;
			}
		}
		r = amdgpu_bo_reserve(adev->gfx.rlc.clear_state_obj, false);
		if (unlikely(r != 0)) {
			gfx_v7_0_rlc_fini(adev);
			return r;
		}
		r = amdgpu_bo_pin(adev->gfx.rlc.clear_state_obj, AMDGPU_GEM_DOMAIN_VRAM,
				  &adev->gfx.rlc.clear_state_gpu_addr);
		if (r) {
			amdgpu_bo_unreserve(adev->gfx.rlc.clear_state_obj);
			dev_warn(adev->dev, "(%d) pin RLC c bo failed\n", r);
			gfx_v7_0_rlc_fini(adev);
			return r;
		}

		r = amdgpu_bo_kmap(adev->gfx.rlc.clear_state_obj, (void **)&adev->gfx.rlc.cs_ptr);
		if (r) {
			dev_warn(adev->dev, "(%d) map RLC c bo failed\n", r);
			gfx_v7_0_rlc_fini(adev);
			return r;
		}
		/* set up the cs buffer */
		dst_ptr = adev->gfx.rlc.cs_ptr;
		gfx_v7_0_get_csb_buffer(adev, dst_ptr);
		amdgpu_bo_kunmap(adev->gfx.rlc.clear_state_obj);
		amdgpu_bo_unreserve(adev->gfx.rlc.clear_state_obj);
	}

	if (adev->gfx.rlc.cp_table_size) {
		if (adev->gfx.rlc.cp_table_obj == NULL) {
			r = amdgpu_bo_create(adev, adev->gfx.rlc.cp_table_size, PAGE_SIZE, true,
					     AMDGPU_GEM_DOMAIN_VRAM,
					     AMDGPU_GEM_CREATE_CPU_ACCESS_REQUIRED,
					     NULL, NULL,
					     &adev->gfx.rlc.cp_table_obj);
			if (r) {
				dev_warn(adev->dev, "(%d) create RLC cp table bo failed\n", r);
				gfx_v7_0_rlc_fini(adev);
				return r;
			}
		}

		r = amdgpu_bo_reserve(adev->gfx.rlc.cp_table_obj, false);
		if (unlikely(r != 0)) {
			dev_warn(adev->dev, "(%d) reserve RLC cp table bo failed\n", r);
			gfx_v7_0_rlc_fini(adev);
			return r;
		}
		r = amdgpu_bo_pin(adev->gfx.rlc.cp_table_obj, AMDGPU_GEM_DOMAIN_VRAM,
				  &adev->gfx.rlc.cp_table_gpu_addr);
		if (r) {
			amdgpu_bo_unreserve(adev->gfx.rlc.cp_table_obj);
			dev_warn(adev->dev, "(%d) pin RLC cp_table bo failed\n", r);
			gfx_v7_0_rlc_fini(adev);
			return r;
		}
		r = amdgpu_bo_kmap(adev->gfx.rlc.cp_table_obj, (void **)&adev->gfx.rlc.cp_table_ptr);
		if (r) {
			dev_warn(adev->dev, "(%d) map RLC cp table bo failed\n", r);
			gfx_v7_0_rlc_fini(adev);
			return r;
		}

		gfx_v7_0_init_cp_pg_table(adev);

		amdgpu_bo_kunmap(adev->gfx.rlc.cp_table_obj);
		amdgpu_bo_unreserve(adev->gfx.rlc.cp_table_obj);

	}

	return 0;
}

static void gfx_v7_0_enable_lbpw(struct amdgpu_device *adev, bool enable)
{
	u32 tmp;

	tmp = RREG32(mmRLC_LB_CNTL);
	if (enable)
		tmp |= RLC_LB_CNTL__LOAD_BALANCE_ENABLE_MASK;
	else
		tmp &= ~RLC_LB_CNTL__LOAD_BALANCE_ENABLE_MASK;
	WREG32(mmRLC_LB_CNTL, tmp);
}

static void gfx_v7_0_wait_for_rlc_serdes(struct amdgpu_device *adev)
{
	u32 i, j, k;
	u32 mask;

	mutex_lock(&adev->grbm_idx_mutex);
	for (i = 0; i < adev->gfx.config.max_shader_engines; i++) {
		for (j = 0; j < adev->gfx.config.max_sh_per_se; j++) {
			gfx_v7_0_select_se_sh(adev, i, j);
			for (k = 0; k < adev->usec_timeout; k++) {
				if (RREG32(mmRLC_SERDES_CU_MASTER_BUSY) == 0)
					break;
				udelay(1);
			}
		}
	}
	gfx_v7_0_select_se_sh(adev, 0xffffffff, 0xffffffff);
	mutex_unlock(&adev->grbm_idx_mutex);

	mask = RLC_SERDES_NONCU_MASTER_BUSY__SE_MASTER_BUSY_MASK |
		RLC_SERDES_NONCU_MASTER_BUSY__GC_MASTER_BUSY_MASK |
		RLC_SERDES_NONCU_MASTER_BUSY__TC0_MASTER_BUSY_MASK |
		RLC_SERDES_NONCU_MASTER_BUSY__TC1_MASTER_BUSY_MASK;
	for (k = 0; k < adev->usec_timeout; k++) {
		if ((RREG32(mmRLC_SERDES_NONCU_MASTER_BUSY) & mask) == 0)
			break;
		udelay(1);
	}
}

static void gfx_v7_0_update_rlc(struct amdgpu_device *adev, u32 rlc)
{
	u32 tmp;

	tmp = RREG32(mmRLC_CNTL);
	if (tmp != rlc)
		WREG32(mmRLC_CNTL, rlc);
}

static u32 gfx_v7_0_halt_rlc(struct amdgpu_device *adev)
{
	u32 data, orig;

	orig = data = RREG32(mmRLC_CNTL);

	if (data & RLC_CNTL__RLC_ENABLE_F32_MASK) {
		u32 i;

		data &= ~RLC_CNTL__RLC_ENABLE_F32_MASK;
		WREG32(mmRLC_CNTL, data);

		for (i = 0; i < adev->usec_timeout; i++) {
			if ((RREG32(mmRLC_GPM_STAT) & RLC_GPM_STAT__RLC_BUSY_MASK) == 0)
				break;
			udelay(1);
		}

		gfx_v7_0_wait_for_rlc_serdes(adev);
	}

	return orig;
}

void gfx_v7_0_enter_rlc_safe_mode(struct amdgpu_device *adev)
{
	u32 tmp, i, mask;

	tmp = 0x1 | (1 << 1);
	WREG32(mmRLC_GPR_REG2, tmp);

	mask = RLC_GPM_STAT__GFX_POWER_STATUS_MASK |
		RLC_GPM_STAT__GFX_CLOCK_STATUS_MASK;
	for (i = 0; i < adev->usec_timeout; i++) {
		if ((RREG32(mmRLC_GPM_STAT) & mask) == mask)
			break;
		udelay(1);
	}

	for (i = 0; i < adev->usec_timeout; i++) {
		if ((RREG32(mmRLC_GPR_REG2) & 0x1) == 0)
			break;
		udelay(1);
	}
}

void gfx_v7_0_exit_rlc_safe_mode(struct amdgpu_device *adev)
{
	u32 tmp;

	tmp = 0x1 | (0 << 1);
	WREG32(mmRLC_GPR_REG2, tmp);
}

/**
 * gfx_v7_0_rlc_stop - stop the RLC ME
 *
 * @adev: amdgpu_device pointer
 *
 * Halt the RLC ME (MicroEngine) (CIK).
 */
void gfx_v7_0_rlc_stop(struct amdgpu_device *adev)
{
	WREG32(mmRLC_CNTL, 0);

	gfx_v7_0_enable_gui_idle_interrupt(adev, false);

	gfx_v7_0_wait_for_rlc_serdes(adev);
}

/**
 * gfx_v7_0_rlc_start - start the RLC ME
 *
 * @adev: amdgpu_device pointer
 *
 * Unhalt the RLC ME (MicroEngine) (CIK).
 */
static void gfx_v7_0_rlc_start(struct amdgpu_device *adev)
{
	WREG32(mmRLC_CNTL, RLC_CNTL__RLC_ENABLE_F32_MASK);

	gfx_v7_0_enable_gui_idle_interrupt(adev, true);

	udelay(50);
}

static void gfx_v7_0_rlc_reset(struct amdgpu_device *adev)
{
	u32 tmp = RREG32(mmGRBM_SOFT_RESET);

	tmp |= GRBM_SOFT_RESET__SOFT_RESET_RLC_MASK;
	WREG32(mmGRBM_SOFT_RESET, tmp);
	udelay(50);
	tmp &= ~GRBM_SOFT_RESET__SOFT_RESET_RLC_MASK;
	WREG32(mmGRBM_SOFT_RESET, tmp);
	udelay(50);
}

/**
 * gfx_v7_0_rlc_resume - setup the RLC hw
 *
 * @adev: amdgpu_device pointer
 *
 * Initialize the RLC registers, load the ucode,
 * and start the RLC (CIK).
 * Returns 0 for success, -EINVAL if the ucode is not available.
 */
static int gfx_v7_0_rlc_resume(struct amdgpu_device *adev)
{
	const struct rlc_firmware_header_v1_0 *hdr;
	const __le32 *fw_data;
	unsigned i, fw_size;
	u32 tmp;

	if (!adev->gfx.rlc_fw)
		return -EINVAL;

	hdr = (const struct rlc_firmware_header_v1_0 *)adev->gfx.rlc_fw->data;
	amdgpu_ucode_print_rlc_hdr(&hdr->header);
	adev->gfx.rlc_fw_version = le32_to_cpu(hdr->header.ucode_version);
	adev->gfx.rlc_feature_version = le32_to_cpu(
					hdr->ucode_feature_version);

	gfx_v7_0_rlc_stop(adev);

	/* disable CG */
	tmp = RREG32(mmRLC_CGCG_CGLS_CTRL) & 0xfffffffc;
	WREG32(mmRLC_CGCG_CGLS_CTRL, tmp);

	gfx_v7_0_rlc_reset(adev);

	gfx_v7_0_init_pg(adev);

	WREG32(mmRLC_LB_CNTR_INIT, 0);
	WREG32(mmRLC_LB_CNTR_MAX, 0x00008000);

	mutex_lock(&adev->grbm_idx_mutex);
	gfx_v7_0_select_se_sh(adev, 0xffffffff, 0xffffffff);
	WREG32(mmRLC_LB_INIT_CU_MASK, 0xffffffff);
	WREG32(mmRLC_LB_PARAMS, 0x00600408);
	WREG32(mmRLC_LB_CNTL, 0x80000004);
	mutex_unlock(&adev->grbm_idx_mutex);

	WREG32(mmRLC_MC_CNTL, 0);
	WREG32(mmRLC_UCODE_CNTL, 0);

	fw_data = (const __le32 *)
		(adev->gfx.rlc_fw->data + le32_to_cpu(hdr->header.ucode_array_offset_bytes));
	fw_size = le32_to_cpu(hdr->header.ucode_size_bytes) / 4;
	WREG32(mmRLC_GPM_UCODE_ADDR, 0);
	for (i = 0; i < fw_size; i++)
		WREG32(mmRLC_GPM_UCODE_DATA, le32_to_cpup(fw_data++));
	WREG32(mmRLC_GPM_UCODE_ADDR, adev->gfx.rlc_fw_version);

	/* XXX - find out what chips support lbpw */
	gfx_v7_0_enable_lbpw(adev, false);

	if (adev->asic_type == CHIP_BONAIRE)
		WREG32(mmRLC_DRIVER_CPDMA_STATUS, 0);

	gfx_v7_0_rlc_start(adev);

	return 0;
}

static void gfx_v7_0_enable_cgcg(struct amdgpu_device *adev, bool enable)
{
	u32 data, orig, tmp, tmp2;

	orig = data = RREG32(mmRLC_CGCG_CGLS_CTRL);

	if (enable && (adev->cg_flags & AMD_CG_SUPPORT_GFX_CGCG)) {
		gfx_v7_0_enable_gui_idle_interrupt(adev, true);

		tmp = gfx_v7_0_halt_rlc(adev);

		mutex_lock(&adev->grbm_idx_mutex);
		gfx_v7_0_select_se_sh(adev, 0xffffffff, 0xffffffff);
		WREG32(mmRLC_SERDES_WR_CU_MASTER_MASK, 0xffffffff);
		WREG32(mmRLC_SERDES_WR_NONCU_MASTER_MASK, 0xffffffff);
		tmp2 = RLC_SERDES_WR_CTRL__BPM_ADDR_MASK |
			RLC_SERDES_WR_CTRL__CGCG_OVERRIDE_0_MASK |
			RLC_SERDES_WR_CTRL__CGLS_ENABLE_MASK;
		WREG32(mmRLC_SERDES_WR_CTRL, tmp2);
		mutex_unlock(&adev->grbm_idx_mutex);

		gfx_v7_0_update_rlc(adev, tmp);

		data |= RLC_CGCG_CGLS_CTRL__CGCG_EN_MASK | RLC_CGCG_CGLS_CTRL__CGLS_EN_MASK;
	} else {
		gfx_v7_0_enable_gui_idle_interrupt(adev, false);

		RREG32(mmCB_CGTT_SCLK_CTRL);
		RREG32(mmCB_CGTT_SCLK_CTRL);
		RREG32(mmCB_CGTT_SCLK_CTRL);
		RREG32(mmCB_CGTT_SCLK_CTRL);

		data &= ~(RLC_CGCG_CGLS_CTRL__CGCG_EN_MASK | RLC_CGCG_CGLS_CTRL__CGLS_EN_MASK);
	}

	if (orig != data)
		WREG32(mmRLC_CGCG_CGLS_CTRL, data);

}

static void gfx_v7_0_enable_mgcg(struct amdgpu_device *adev, bool enable)
{
	u32 data, orig, tmp = 0;

	if (enable && (adev->cg_flags & AMD_CG_SUPPORT_GFX_MGCG)) {
		if (adev->cg_flags & AMD_CG_SUPPORT_GFX_MGLS) {
			if (adev->cg_flags & AMD_CG_SUPPORT_GFX_CP_LS) {
				orig = data = RREG32(mmCP_MEM_SLP_CNTL);
				data |= CP_MEM_SLP_CNTL__CP_MEM_LS_EN_MASK;
				if (orig != data)
					WREG32(mmCP_MEM_SLP_CNTL, data);
			}
		}

		orig = data = RREG32(mmRLC_CGTT_MGCG_OVERRIDE);
		data |= 0x00000001;
		data &= 0xfffffffd;
		if (orig != data)
			WREG32(mmRLC_CGTT_MGCG_OVERRIDE, data);

		tmp = gfx_v7_0_halt_rlc(adev);

		mutex_lock(&adev->grbm_idx_mutex);
		gfx_v7_0_select_se_sh(adev, 0xffffffff, 0xffffffff);
		WREG32(mmRLC_SERDES_WR_CU_MASTER_MASK, 0xffffffff);
		WREG32(mmRLC_SERDES_WR_NONCU_MASTER_MASK, 0xffffffff);
		data = RLC_SERDES_WR_CTRL__BPM_ADDR_MASK |
			RLC_SERDES_WR_CTRL__MGCG_OVERRIDE_0_MASK;
		WREG32(mmRLC_SERDES_WR_CTRL, data);
		mutex_unlock(&adev->grbm_idx_mutex);

		gfx_v7_0_update_rlc(adev, tmp);

		if (adev->cg_flags & AMD_CG_SUPPORT_GFX_CGTS) {
			orig = data = RREG32(mmCGTS_SM_CTRL_REG);
			data &= ~CGTS_SM_CTRL_REG__SM_MODE_MASK;
			data |= (0x2 << CGTS_SM_CTRL_REG__SM_MODE__SHIFT);
			data |= CGTS_SM_CTRL_REG__SM_MODE_ENABLE_MASK;
			data &= ~CGTS_SM_CTRL_REG__OVERRIDE_MASK;
			if ((adev->cg_flags & AMD_CG_SUPPORT_GFX_MGLS) &&
			    (adev->cg_flags & AMD_CG_SUPPORT_GFX_CGTS_LS))
				data &= ~CGTS_SM_CTRL_REG__LS_OVERRIDE_MASK;
			data &= ~CGTS_SM_CTRL_REG__ON_MONITOR_ADD_MASK;
			data |= CGTS_SM_CTRL_REG__ON_MONITOR_ADD_EN_MASK;
			data |= (0x96 << CGTS_SM_CTRL_REG__ON_MONITOR_ADD__SHIFT);
			if (orig != data)
				WREG32(mmCGTS_SM_CTRL_REG, data);
		}
	} else {
		orig = data = RREG32(mmRLC_CGTT_MGCG_OVERRIDE);
		data |= 0x00000003;
		if (orig != data)
			WREG32(mmRLC_CGTT_MGCG_OVERRIDE, data);

		data = RREG32(mmRLC_MEM_SLP_CNTL);
		if (data & RLC_MEM_SLP_CNTL__RLC_MEM_LS_EN_MASK) {
			data &= ~RLC_MEM_SLP_CNTL__RLC_MEM_LS_EN_MASK;
			WREG32(mmRLC_MEM_SLP_CNTL, data);
		}

		data = RREG32(mmCP_MEM_SLP_CNTL);
		if (data & CP_MEM_SLP_CNTL__CP_MEM_LS_EN_MASK) {
			data &= ~CP_MEM_SLP_CNTL__CP_MEM_LS_EN_MASK;
			WREG32(mmCP_MEM_SLP_CNTL, data);
		}

		orig = data = RREG32(mmCGTS_SM_CTRL_REG);
		data |= CGTS_SM_CTRL_REG__OVERRIDE_MASK | CGTS_SM_CTRL_REG__LS_OVERRIDE_MASK;
		if (orig != data)
			WREG32(mmCGTS_SM_CTRL_REG, data);

		tmp = gfx_v7_0_halt_rlc(adev);

		mutex_lock(&adev->grbm_idx_mutex);
		gfx_v7_0_select_se_sh(adev, 0xffffffff, 0xffffffff);
		WREG32(mmRLC_SERDES_WR_CU_MASTER_MASK, 0xffffffff);
		WREG32(mmRLC_SERDES_WR_NONCU_MASTER_MASK, 0xffffffff);
		data = RLC_SERDES_WR_CTRL__BPM_ADDR_MASK | RLC_SERDES_WR_CTRL__MGCG_OVERRIDE_1_MASK;
		WREG32(mmRLC_SERDES_WR_CTRL, data);
		mutex_unlock(&adev->grbm_idx_mutex);

		gfx_v7_0_update_rlc(adev, tmp);
	}
}

static void gfx_v7_0_update_cg(struct amdgpu_device *adev,
			       bool enable)
{
	gfx_v7_0_enable_gui_idle_interrupt(adev, false);
	/* order matters! */
	if (enable) {
		gfx_v7_0_enable_mgcg(adev, true);
		gfx_v7_0_enable_cgcg(adev, true);
	} else {
		gfx_v7_0_enable_cgcg(adev, false);
		gfx_v7_0_enable_mgcg(adev, false);
	}
	gfx_v7_0_enable_gui_idle_interrupt(adev, true);
}

static void gfx_v7_0_enable_sclk_slowdown_on_pu(struct amdgpu_device *adev,
						bool enable)
{
	u32 data, orig;

	orig = data = RREG32(mmRLC_PG_CNTL);
	if (enable && (adev->pg_flags & AMD_PG_SUPPORT_RLC_SMU_HS))
		data |= RLC_PG_CNTL__SMU_CLK_SLOWDOWN_ON_PU_ENABLE_MASK;
	else
		data &= ~RLC_PG_CNTL__SMU_CLK_SLOWDOWN_ON_PU_ENABLE_MASK;
	if (orig != data)
		WREG32(mmRLC_PG_CNTL, data);
}

static void gfx_v7_0_enable_sclk_slowdown_on_pd(struct amdgpu_device *adev,
						bool enable)
{
	u32 data, orig;

	orig = data = RREG32(mmRLC_PG_CNTL);
	if (enable && (adev->pg_flags & AMD_PG_SUPPORT_RLC_SMU_HS))
		data |= RLC_PG_CNTL__SMU_CLK_SLOWDOWN_ON_PD_ENABLE_MASK;
	else
		data &= ~RLC_PG_CNTL__SMU_CLK_SLOWDOWN_ON_PD_ENABLE_MASK;
	if (orig != data)
		WREG32(mmRLC_PG_CNTL, data);
}

static void gfx_v7_0_enable_cp_pg(struct amdgpu_device *adev, bool enable)
{
	u32 data, orig;

	orig = data = RREG32(mmRLC_PG_CNTL);
	if (enable && (adev->pg_flags & AMD_PG_SUPPORT_CP))
		data &= ~0x8000;
	else
		data |= 0x8000;
	if (orig != data)
		WREG32(mmRLC_PG_CNTL, data);
}

static void gfx_v7_0_enable_gds_pg(struct amdgpu_device *adev, bool enable)
{
	u32 data, orig;

	orig = data = RREG32(mmRLC_PG_CNTL);
	if (enable && (adev->pg_flags & AMD_PG_SUPPORT_GDS))
		data &= ~0x2000;
	else
		data |= 0x2000;
	if (orig != data)
		WREG32(mmRLC_PG_CNTL, data);
}

static void gfx_v7_0_init_cp_pg_table(struct amdgpu_device *adev)
{
	const __le32 *fw_data;
	volatile u32 *dst_ptr;
	int me, i, max_me = 4;
	u32 bo_offset = 0;
	u32 table_offset, table_size;

	if (adev->asic_type == CHIP_KAVERI)
		max_me = 5;

	if (adev->gfx.rlc.cp_table_ptr == NULL)
		return;

	/* write the cp table buffer */
	dst_ptr = adev->gfx.rlc.cp_table_ptr;
	for (me = 0; me < max_me; me++) {
		if (me == 0) {
			const struct gfx_firmware_header_v1_0 *hdr =
				(const struct gfx_firmware_header_v1_0 *)adev->gfx.ce_fw->data;
			fw_data = (const __le32 *)
				(adev->gfx.ce_fw->data +
				 le32_to_cpu(hdr->header.ucode_array_offset_bytes));
			table_offset = le32_to_cpu(hdr->jt_offset);
			table_size = le32_to_cpu(hdr->jt_size);
		} else if (me == 1) {
			const struct gfx_firmware_header_v1_0 *hdr =
				(const struct gfx_firmware_header_v1_0 *)adev->gfx.pfp_fw->data;
			fw_data = (const __le32 *)
				(adev->gfx.pfp_fw->data +
				 le32_to_cpu(hdr->header.ucode_array_offset_bytes));
			table_offset = le32_to_cpu(hdr->jt_offset);
			table_size = le32_to_cpu(hdr->jt_size);
		} else if (me == 2) {
			const struct gfx_firmware_header_v1_0 *hdr =
				(const struct gfx_firmware_header_v1_0 *)adev->gfx.me_fw->data;
			fw_data = (const __le32 *)
				(adev->gfx.me_fw->data +
				 le32_to_cpu(hdr->header.ucode_array_offset_bytes));
			table_offset = le32_to_cpu(hdr->jt_offset);
			table_size = le32_to_cpu(hdr->jt_size);
		} else if (me == 3) {
			const struct gfx_firmware_header_v1_0 *hdr =
				(const struct gfx_firmware_header_v1_0 *)adev->gfx.mec_fw->data;
			fw_data = (const __le32 *)
				(adev->gfx.mec_fw->data +
				 le32_to_cpu(hdr->header.ucode_array_offset_bytes));
			table_offset = le32_to_cpu(hdr->jt_offset);
			table_size = le32_to_cpu(hdr->jt_size);
		} else {
			const struct gfx_firmware_header_v1_0 *hdr =
				(const struct gfx_firmware_header_v1_0 *)adev->gfx.mec2_fw->data;
			fw_data = (const __le32 *)
				(adev->gfx.mec2_fw->data +
				 le32_to_cpu(hdr->header.ucode_array_offset_bytes));
			table_offset = le32_to_cpu(hdr->jt_offset);
			table_size = le32_to_cpu(hdr->jt_size);
		}

		for (i = 0; i < table_size; i ++) {
			dst_ptr[bo_offset + i] =
				cpu_to_le32(le32_to_cpu(fw_data[table_offset + i]));
		}

		bo_offset += table_size;
	}
}

static void gfx_v7_0_enable_gfx_cgpg(struct amdgpu_device *adev,
				     bool enable)
{
	u32 data, orig;

	if (enable && (adev->pg_flags & AMD_PG_SUPPORT_GFX_PG)) {
		orig = data = RREG32(mmRLC_PG_CNTL);
		data |= RLC_PG_CNTL__GFX_POWER_GATING_ENABLE_MASK;
		if (orig != data)
			WREG32(mmRLC_PG_CNTL, data);

		orig = data = RREG32(mmRLC_AUTO_PG_CTRL);
		data |= RLC_AUTO_PG_CTRL__AUTO_PG_EN_MASK;
		if (orig != data)
			WREG32(mmRLC_AUTO_PG_CTRL, data);
	} else {
		orig = data = RREG32(mmRLC_PG_CNTL);
		data &= ~RLC_PG_CNTL__GFX_POWER_GATING_ENABLE_MASK;
		if (orig != data)
			WREG32(mmRLC_PG_CNTL, data);

		orig = data = RREG32(mmRLC_AUTO_PG_CTRL);
		data &= ~RLC_AUTO_PG_CTRL__AUTO_PG_EN_MASK;
		if (orig != data)
			WREG32(mmRLC_AUTO_PG_CTRL, data);

		data = RREG32(mmDB_RENDER_CONTROL);
	}
}

static u32 gfx_v7_0_get_cu_active_bitmap(struct amdgpu_device *adev,
					 u32 se, u32 sh)
{
	u32 mask = 0, tmp, tmp1;
	int i;

	gfx_v7_0_select_se_sh(adev, se, sh);
	tmp = RREG32(mmCC_GC_SHADER_ARRAY_CONFIG);
	tmp1 = RREG32(mmGC_USER_SHADER_ARRAY_CONFIG);
	gfx_v7_0_select_se_sh(adev, 0xffffffff, 0xffffffff);

	tmp &= 0xffff0000;

	tmp |= tmp1;
	tmp >>= 16;

	for (i = 0; i < adev->gfx.config.max_cu_per_sh; i ++) {
		mask <<= 1;
		mask |= 1;
	}

	return (~tmp) & mask;
}

static void gfx_v7_0_init_ao_cu_mask(struct amdgpu_device *adev)
{
	uint32_t tmp, active_cu_number;
	struct amdgpu_cu_info cu_info;

	gfx_v7_0_get_cu_info(adev, &cu_info);
	tmp = cu_info.ao_cu_mask;
	active_cu_number = cu_info.number;

	WREG32(mmRLC_PG_ALWAYS_ON_CU_MASK, tmp);

	tmp = RREG32(mmRLC_MAX_PG_CU);
	tmp &= ~RLC_MAX_PG_CU__MAX_POWERED_UP_CU_MASK;
	tmp |= (active_cu_number << RLC_MAX_PG_CU__MAX_POWERED_UP_CU__SHIFT);
	WREG32(mmRLC_MAX_PG_CU, tmp);
}

static void gfx_v7_0_enable_gfx_static_mgpg(struct amdgpu_device *adev,
					    bool enable)
{
	u32 data, orig;

	orig = data = RREG32(mmRLC_PG_CNTL);
	if (enable && (adev->pg_flags & AMD_PG_SUPPORT_GFX_SMG))
		data |= RLC_PG_CNTL__STATIC_PER_CU_PG_ENABLE_MASK;
	else
		data &= ~RLC_PG_CNTL__STATIC_PER_CU_PG_ENABLE_MASK;
	if (orig != data)
		WREG32(mmRLC_PG_CNTL, data);
}

static void gfx_v7_0_enable_gfx_dynamic_mgpg(struct amdgpu_device *adev,
					     bool enable)
{
	u32 data, orig;

	orig = data = RREG32(mmRLC_PG_CNTL);
	if (enable && (adev->pg_flags & AMD_PG_SUPPORT_GFX_DMG))
		data |= RLC_PG_CNTL__DYN_PER_CU_PG_ENABLE_MASK;
	else
		data &= ~RLC_PG_CNTL__DYN_PER_CU_PG_ENABLE_MASK;
	if (orig != data)
		WREG32(mmRLC_PG_CNTL, data);
}

#define RLC_SAVE_AND_RESTORE_STARTING_OFFSET 0x90
#define RLC_CLEAR_STATE_DESCRIPTOR_OFFSET    0x3D

static void gfx_v7_0_init_gfx_cgpg(struct amdgpu_device *adev)
{
	u32 data, orig;
	u32 i;

	if (adev->gfx.rlc.cs_data) {
		WREG32(mmRLC_GPM_SCRATCH_ADDR, RLC_CLEAR_STATE_DESCRIPTOR_OFFSET);
		WREG32(mmRLC_GPM_SCRATCH_DATA, upper_32_bits(adev->gfx.rlc.clear_state_gpu_addr));
		WREG32(mmRLC_GPM_SCRATCH_DATA, lower_32_bits(adev->gfx.rlc.clear_state_gpu_addr));
		WREG32(mmRLC_GPM_SCRATCH_DATA, adev->gfx.rlc.clear_state_size);
	} else {
		WREG32(mmRLC_GPM_SCRATCH_ADDR, RLC_CLEAR_STATE_DESCRIPTOR_OFFSET);
		for (i = 0; i < 3; i++)
			WREG32(mmRLC_GPM_SCRATCH_DATA, 0);
	}
	if (adev->gfx.rlc.reg_list) {
		WREG32(mmRLC_GPM_SCRATCH_ADDR, RLC_SAVE_AND_RESTORE_STARTING_OFFSET);
		for (i = 0; i < adev->gfx.rlc.reg_list_size; i++)
			WREG32(mmRLC_GPM_SCRATCH_DATA, adev->gfx.rlc.reg_list[i]);
	}

	orig = data = RREG32(mmRLC_PG_CNTL);
	data |= RLC_PG_CNTL__GFX_POWER_GATING_SRC_MASK;
	if (orig != data)
		WREG32(mmRLC_PG_CNTL, data);

	WREG32(mmRLC_SAVE_AND_RESTORE_BASE, adev->gfx.rlc.save_restore_gpu_addr >> 8);
	WREG32(mmRLC_JUMP_TABLE_RESTORE, adev->gfx.rlc.cp_table_gpu_addr >> 8);

	data = RREG32(mmCP_RB_WPTR_POLL_CNTL);
	data &= ~CP_RB_WPTR_POLL_CNTL__IDLE_POLL_COUNT_MASK;
	data |= (0x60 << CP_RB_WPTR_POLL_CNTL__IDLE_POLL_COUNT__SHIFT);
	WREG32(mmCP_RB_WPTR_POLL_CNTL, data);

	data = 0x10101010;
	WREG32(mmRLC_PG_DELAY, data);

	data = RREG32(mmRLC_PG_DELAY_2);
	data &= ~0xff;
	data |= 0x3;
	WREG32(mmRLC_PG_DELAY_2, data);

	data = RREG32(mmRLC_AUTO_PG_CTRL);
	data &= ~RLC_AUTO_PG_CTRL__GRBM_REG_SAVE_GFX_IDLE_THRESHOLD_MASK;
	data |= (0x700 << RLC_AUTO_PG_CTRL__GRBM_REG_SAVE_GFX_IDLE_THRESHOLD__SHIFT);
	WREG32(mmRLC_AUTO_PG_CTRL, data);

}

static void gfx_v7_0_update_gfx_pg(struct amdgpu_device *adev, bool enable)
{
	gfx_v7_0_enable_gfx_cgpg(adev, enable);
	gfx_v7_0_enable_gfx_static_mgpg(adev, enable);
	gfx_v7_0_enable_gfx_dynamic_mgpg(adev, enable);
}

static u32 gfx_v7_0_get_csb_size(struct amdgpu_device *adev)
{
	u32 count = 0;
	const struct cs_section_def *sect = NULL;
	const struct cs_extent_def *ext = NULL;

	if (adev->gfx.rlc.cs_data == NULL)
		return 0;

	/* begin clear state */
	count += 2;
	/* context control state */
	count += 3;

	for (sect = adev->gfx.rlc.cs_data; sect->section != NULL; ++sect) {
		for (ext = sect->section; ext->extent != NULL; ++ext) {
			if (sect->id == SECT_CONTEXT)
				count += 2 + ext->reg_count;
			else
				return 0;
		}
	}
	/* pa_sc_raster_config/pa_sc_raster_config1 */
	count += 4;
	/* end clear state */
	count += 2;
	/* clear state */
	count += 2;

	return count;
}

static void gfx_v7_0_get_csb_buffer(struct amdgpu_device *adev,
				    volatile u32 *buffer)
{
	u32 count = 0, i;
	const struct cs_section_def *sect = NULL;
	const struct cs_extent_def *ext = NULL;

	if (adev->gfx.rlc.cs_data == NULL)
		return;
	if (buffer == NULL)
		return;

	buffer[count++] = cpu_to_le32(PACKET3(PACKET3_PREAMBLE_CNTL, 0));
	buffer[count++] = cpu_to_le32(PACKET3_PREAMBLE_BEGIN_CLEAR_STATE);

	buffer[count++] = cpu_to_le32(PACKET3(PACKET3_CONTEXT_CONTROL, 1));
	buffer[count++] = cpu_to_le32(0x80000000);
	buffer[count++] = cpu_to_le32(0x80000000);

	for (sect = adev->gfx.rlc.cs_data; sect->section != NULL; ++sect) {
		for (ext = sect->section; ext->extent != NULL; ++ext) {
			if (sect->id == SECT_CONTEXT) {
				buffer[count++] =
					cpu_to_le32(PACKET3(PACKET3_SET_CONTEXT_REG, ext->reg_count));
				buffer[count++] = cpu_to_le32(ext->reg_index - PACKET3_SET_CONTEXT_REG_START);
				for (i = 0; i < ext->reg_count; i++)
					buffer[count++] = cpu_to_le32(ext->extent[i]);
			} else {
				return;
			}
		}
	}

	buffer[count++] = cpu_to_le32(PACKET3(PACKET3_SET_CONTEXT_REG, 2));
	buffer[count++] = cpu_to_le32(mmPA_SC_RASTER_CONFIG - PACKET3_SET_CONTEXT_REG_START);
	switch (adev->asic_type) {
	case CHIP_BONAIRE:
		buffer[count++] = cpu_to_le32(0x16000012);
		buffer[count++] = cpu_to_le32(0x00000000);
		break;
	case CHIP_KAVERI:
		buffer[count++] = cpu_to_le32(0x00000000); /* XXX */
		buffer[count++] = cpu_to_le32(0x00000000);
		break;
	case CHIP_KABINI:
	case CHIP_MULLINS:
		buffer[count++] = cpu_to_le32(0x00000000); /* XXX */
		buffer[count++] = cpu_to_le32(0x00000000);
		break;
	case CHIP_HAWAII:
		buffer[count++] = cpu_to_le32(0x3a00161a);
		buffer[count++] = cpu_to_le32(0x0000002e);
		break;
	default:
		buffer[count++] = cpu_to_le32(0x00000000);
		buffer[count++] = cpu_to_le32(0x00000000);
		break;
	}

	buffer[count++] = cpu_to_le32(PACKET3(PACKET3_PREAMBLE_CNTL, 0));
	buffer[count++] = cpu_to_le32(PACKET3_PREAMBLE_END_CLEAR_STATE);

	buffer[count++] = cpu_to_le32(PACKET3(PACKET3_CLEAR_STATE, 0));
	buffer[count++] = cpu_to_le32(0);
}

static void gfx_v7_0_init_pg(struct amdgpu_device *adev)
{
	if (adev->pg_flags & (AMD_PG_SUPPORT_GFX_PG |
			      AMD_PG_SUPPORT_GFX_SMG |
			      AMD_PG_SUPPORT_GFX_DMG |
			      AMD_PG_SUPPORT_CP |
			      AMD_PG_SUPPORT_GDS |
			      AMD_PG_SUPPORT_RLC_SMU_HS)) {
		gfx_v7_0_enable_sclk_slowdown_on_pu(adev, true);
		gfx_v7_0_enable_sclk_slowdown_on_pd(adev, true);
		if (adev->pg_flags & AMD_PG_SUPPORT_GFX_PG) {
			gfx_v7_0_init_gfx_cgpg(adev);
			gfx_v7_0_enable_cp_pg(adev, true);
			gfx_v7_0_enable_gds_pg(adev, true);
		}
		gfx_v7_0_init_ao_cu_mask(adev);
		gfx_v7_0_update_gfx_pg(adev, true);
	}
}

static void gfx_v7_0_fini_pg(struct amdgpu_device *adev)
{
	if (adev->pg_flags & (AMD_PG_SUPPORT_GFX_PG |
			      AMD_PG_SUPPORT_GFX_SMG |
			      AMD_PG_SUPPORT_GFX_DMG |
			      AMD_PG_SUPPORT_CP |
			      AMD_PG_SUPPORT_GDS |
			      AMD_PG_SUPPORT_RLC_SMU_HS)) {
		gfx_v7_0_update_gfx_pg(adev, false);
		if (adev->pg_flags & AMD_PG_SUPPORT_GFX_PG) {
			gfx_v7_0_enable_cp_pg(adev, false);
			gfx_v7_0_enable_gds_pg(adev, false);
		}
	}
}

/**
 * gfx_v7_0_get_gpu_clock_counter - return GPU clock counter snapshot
 *
 * @adev: amdgpu_device pointer
 *
 * Fetches a GPU clock counter snapshot (SI).
 * Returns the 64 bit clock counter snapshot.
 */
uint64_t gfx_v7_0_get_gpu_clock_counter(struct amdgpu_device *adev)
{
	uint64_t clock;

	mutex_lock(&adev->gfx.gpu_clock_mutex);
	WREG32(mmRLC_CAPTURE_GPU_CLOCK_COUNT, 1);
	clock = (uint64_t)RREG32(mmRLC_GPU_CLOCK_COUNT_LSB) |
		((uint64_t)RREG32(mmRLC_GPU_CLOCK_COUNT_MSB) << 32ULL);
	mutex_unlock(&adev->gfx.gpu_clock_mutex);
	return clock;
}

static void gfx_v7_0_ring_emit_gds_switch(struct amdgpu_ring *ring,
					  uint32_t vmid,
					  uint32_t gds_base, uint32_t gds_size,
					  uint32_t gws_base, uint32_t gws_size,
					  uint32_t oa_base, uint32_t oa_size)
{
	gds_base = gds_base >> AMDGPU_GDS_SHIFT;
	gds_size = gds_size >> AMDGPU_GDS_SHIFT;

	gws_base = gws_base >> AMDGPU_GWS_SHIFT;
	gws_size = gws_size >> AMDGPU_GWS_SHIFT;

	oa_base = oa_base >> AMDGPU_OA_SHIFT;
	oa_size = oa_size >> AMDGPU_OA_SHIFT;

	/* GDS Base */
	amdgpu_ring_write(ring, PACKET3(PACKET3_WRITE_DATA, 3));
	amdgpu_ring_write(ring, (WRITE_DATA_ENGINE_SEL(0) |
				WRITE_DATA_DST_SEL(0)));
	amdgpu_ring_write(ring, amdgpu_gds_reg_offset[vmid].mem_base);
	amdgpu_ring_write(ring, 0);
	amdgpu_ring_write(ring, gds_base);

	/* GDS Size */
	amdgpu_ring_write(ring, PACKET3(PACKET3_WRITE_DATA, 3));
	amdgpu_ring_write(ring, (WRITE_DATA_ENGINE_SEL(0) |
				WRITE_DATA_DST_SEL(0)));
	amdgpu_ring_write(ring, amdgpu_gds_reg_offset[vmid].mem_size);
	amdgpu_ring_write(ring, 0);
	amdgpu_ring_write(ring, gds_size);

	/* GWS */
	amdgpu_ring_write(ring, PACKET3(PACKET3_WRITE_DATA, 3));
	amdgpu_ring_write(ring, (WRITE_DATA_ENGINE_SEL(0) |
				WRITE_DATA_DST_SEL(0)));
	amdgpu_ring_write(ring, amdgpu_gds_reg_offset[vmid].gws);
	amdgpu_ring_write(ring, 0);
	amdgpu_ring_write(ring, gws_size << GDS_GWS_VMID0__SIZE__SHIFT | gws_base);

	/* OA */
	amdgpu_ring_write(ring, PACKET3(PACKET3_WRITE_DATA, 3));
	amdgpu_ring_write(ring, (WRITE_DATA_ENGINE_SEL(0) |
				WRITE_DATA_DST_SEL(0)));
	amdgpu_ring_write(ring, amdgpu_gds_reg_offset[vmid].oa);
	amdgpu_ring_write(ring, 0);
	amdgpu_ring_write(ring, (1 << (oa_size + oa_base)) - (1 << oa_base));
}

static int gfx_v7_0_early_init(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	adev->gfx.num_gfx_rings = GFX7_NUM_GFX_RINGS;
	adev->gfx.num_compute_rings = GFX7_NUM_COMPUTE_RINGS;
	gfx_v7_0_set_ring_funcs(adev);
	gfx_v7_0_set_irq_funcs(adev);
	gfx_v7_0_set_gds_init(adev);

	return 0;
}

static int gfx_v7_0_late_init(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;
	int r;

	r = amdgpu_irq_get(adev, &adev->gfx.priv_reg_irq, 0);
	if (r)
		return r;

	r = amdgpu_irq_get(adev, &adev->gfx.priv_inst_irq, 0);
	if (r)
		return r;

	return 0;
}

static int gfx_v7_0_sw_init(void *handle)
{
	struct amdgpu_ring *ring;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;
	int i, r;

	/* EOP Event */
	r = amdgpu_irq_add_id(adev, 181, &adev->gfx.eop_irq);
	if (r)
		return r;

	/* Privileged reg */
	r = amdgpu_irq_add_id(adev, 184, &adev->gfx.priv_reg_irq);
	if (r)
		return r;

	/* Privileged inst */
	r = amdgpu_irq_add_id(adev, 185, &adev->gfx.priv_inst_irq);
	if (r)
		return r;

	gfx_v7_0_scratch_init(adev);

	r = gfx_v7_0_init_microcode(adev);
	if (r) {
		DRM_ERROR("Failed to load gfx firmware!\n");
		return r;
	}

	r = gfx_v7_0_rlc_init(adev);
	if (r) {
		DRM_ERROR("Failed to init rlc BOs!\n");
		return r;
	}

	/* allocate mec buffers */
	r = gfx_v7_0_mec_init(adev);
	if (r) {
		DRM_ERROR("Failed to init MEC BOs!\n");
		return r;
	}

	for (i = 0; i < adev->gfx.num_gfx_rings; i++) {
		ring = &adev->gfx.gfx_ring[i];
		ring->ring_obj = NULL;
		sprintf(ring->name, "gfx");
		r = amdgpu_ring_init(adev, ring, 1024 * 1024,
				     PACKET3(PACKET3_NOP, 0x3FFF), 0xf,
				     &adev->gfx.eop_irq, AMDGPU_CP_IRQ_GFX_EOP,
				     AMDGPU_RING_TYPE_GFX);
		if (r)
			return r;
	}

	/* set up the compute queues */
	for (i = 0; i < adev->gfx.num_compute_rings; i++) {
		unsigned irq_type;

		/* max 32 queues per MEC */
		if ((i >= 32) || (i >= AMDGPU_MAX_COMPUTE_RINGS)) {
			DRM_ERROR("Too many (%d) compute rings!\n", i);
			break;
		}
		ring = &adev->gfx.compute_ring[i];
		ring->ring_obj = NULL;
		ring->use_doorbell = true;
		ring->doorbell_index = AMDGPU_DOORBELL_MEC_RING0 + i;
		ring->me = 1; /* first MEC */
		ring->pipe = i / 8;
		ring->queue = i % 8;
		sprintf(ring->name, "comp %d.%d.%d", ring->me, ring->pipe, ring->queue);
		irq_type = AMDGPU_CP_IRQ_COMPUTE_MEC1_PIPE0_EOP + ring->pipe;
		/* type-2 packets are deprecated on MEC, use type-3 instead */
		r = amdgpu_ring_init(adev, ring, 1024 * 1024,
				     PACKET3(PACKET3_NOP, 0x3FFF), 0xf,
				     &adev->gfx.eop_irq, irq_type,
				     AMDGPU_RING_TYPE_COMPUTE);
		if (r)
			return r;
	}

	/* reserve GDS, GWS and OA resource for gfx */
	r = amdgpu_bo_create(adev, adev->gds.mem.gfx_partition_size,
			PAGE_SIZE, true,
			AMDGPU_GEM_DOMAIN_GDS, 0,
			NULL, NULL, &adev->gds.gds_gfx_bo);
	if (r)
		return r;

	r = amdgpu_bo_create(adev, adev->gds.gws.gfx_partition_size,
		PAGE_SIZE, true,
		AMDGPU_GEM_DOMAIN_GWS, 0,
		NULL, NULL, &adev->gds.gws_gfx_bo);
	if (r)
		return r;

	r = amdgpu_bo_create(adev, adev->gds.oa.gfx_partition_size,
			PAGE_SIZE, true,
			AMDGPU_GEM_DOMAIN_OA, 0,
			NULL, NULL, &adev->gds.oa_gfx_bo);
	if (r)
		return r;

	return r;
}

static int gfx_v7_0_sw_fini(void *handle)
{
	int i;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	amdgpu_bo_unref(&adev->gds.oa_gfx_bo);
	amdgpu_bo_unref(&adev->gds.gws_gfx_bo);
	amdgpu_bo_unref(&adev->gds.gds_gfx_bo);

	for (i = 0; i < adev->gfx.num_gfx_rings; i++)
		amdgpu_ring_fini(&adev->gfx.gfx_ring[i]);
	for (i = 0; i < adev->gfx.num_compute_rings; i++)
		amdgpu_ring_fini(&adev->gfx.compute_ring[i]);

	gfx_v7_0_cp_compute_fini(adev);
	gfx_v7_0_rlc_fini(adev);
	gfx_v7_0_mec_fini(adev);

	return 0;
}

static int gfx_v7_0_hw_init(void *handle)
{
	int r;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	gfx_v7_0_gpu_init(adev);

	/* init rlc */
	r = gfx_v7_0_rlc_resume(adev);
	if (r)
		return r;

	r = gfx_v7_0_cp_resume(adev);
	if (r)
		return r;

	adev->gfx.ce_ram_size = 0x8000;

	return r;
}

static int gfx_v7_0_hw_fini(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	amdgpu_irq_put(adev, &adev->gfx.priv_reg_irq, 0);
	amdgpu_irq_put(adev, &adev->gfx.priv_inst_irq, 0);
	gfx_v7_0_cp_enable(adev, false);
	gfx_v7_0_rlc_stop(adev);
	gfx_v7_0_fini_pg(adev);

	return 0;
}

static int gfx_v7_0_suspend(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	return gfx_v7_0_hw_fini(adev);
}

static int gfx_v7_0_resume(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	return gfx_v7_0_hw_init(adev);
}

static bool gfx_v7_0_is_idle(void *handle)
{
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	if (RREG32(mmGRBM_STATUS) & GRBM_STATUS__GUI_ACTIVE_MASK)
		return false;
	else
		return true;
}

static int gfx_v7_0_wait_for_idle(void *handle)
{
	unsigned i;
	u32 tmp;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	for (i = 0; i < adev->usec_timeout; i++) {
		/* read MC_STATUS */
		tmp = RREG32(mmGRBM_STATUS) & GRBM_STATUS__GUI_ACTIVE_MASK;

		if (!tmp)
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

static void gfx_v7_0_print_status(void *handle)
{
	int i;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	dev_info(adev->dev, "GFX 7.x registers\n");
	dev_info(adev->dev, "  GRBM_STATUS=0x%08X\n",
		RREG32(mmGRBM_STATUS));
	dev_info(adev->dev, "  GRBM_STATUS2=0x%08X\n",
		RREG32(mmGRBM_STATUS2));
	dev_info(adev->dev, "  GRBM_STATUS_SE0=0x%08X\n",
		RREG32(mmGRBM_STATUS_SE0));
	dev_info(adev->dev, "  GRBM_STATUS_SE1=0x%08X\n",
		RREG32(mmGRBM_STATUS_SE1));
	dev_info(adev->dev, "  GRBM_STATUS_SE2=0x%08X\n",
		RREG32(mmGRBM_STATUS_SE2));
	dev_info(adev->dev, "  GRBM_STATUS_SE3=0x%08X\n",
		RREG32(mmGRBM_STATUS_SE3));
	dev_info(adev->dev, "  CP_STAT = 0x%08x\n", RREG32(mmCP_STAT));
	dev_info(adev->dev, "  CP_STALLED_STAT1 = 0x%08x\n",
		 RREG32(mmCP_STALLED_STAT1));
	dev_info(adev->dev, "  CP_STALLED_STAT2 = 0x%08x\n",
		 RREG32(mmCP_STALLED_STAT2));
	dev_info(adev->dev, "  CP_STALLED_STAT3 = 0x%08x\n",
		 RREG32(mmCP_STALLED_STAT3));
	dev_info(adev->dev, "  CP_CPF_BUSY_STAT = 0x%08x\n",
		 RREG32(mmCP_CPF_BUSY_STAT));
	dev_info(adev->dev, "  CP_CPF_STALLED_STAT1 = 0x%08x\n",
		 RREG32(mmCP_CPF_STALLED_STAT1));
	dev_info(adev->dev, "  CP_CPF_STATUS = 0x%08x\n", RREG32(mmCP_CPF_STATUS));
	dev_info(adev->dev, "  CP_CPC_BUSY_STAT = 0x%08x\n", RREG32(mmCP_CPC_BUSY_STAT));
	dev_info(adev->dev, "  CP_CPC_STALLED_STAT1 = 0x%08x\n",
		 RREG32(mmCP_CPC_STALLED_STAT1));
	dev_info(adev->dev, "  CP_CPC_STATUS = 0x%08x\n", RREG32(mmCP_CPC_STATUS));

	for (i = 0; i < 32; i++) {
		dev_info(adev->dev, "  GB_TILE_MODE%d=0x%08X\n",
			 i, RREG32(mmGB_TILE_MODE0 + (i * 4)));
	}
	for (i = 0; i < 16; i++) {
		dev_info(adev->dev, "  GB_MACROTILE_MODE%d=0x%08X\n",
			 i, RREG32(mmGB_MACROTILE_MODE0 + (i * 4)));
	}
	for (i = 0; i < adev->gfx.config.max_shader_engines; i++) {
		dev_info(adev->dev, "  se: %d\n", i);
		gfx_v7_0_select_se_sh(adev, i, 0xffffffff);
		dev_info(adev->dev, "  PA_SC_RASTER_CONFIG=0x%08X\n",
			 RREG32(mmPA_SC_RASTER_CONFIG));
		dev_info(adev->dev, "  PA_SC_RASTER_CONFIG_1=0x%08X\n",
			 RREG32(mmPA_SC_RASTER_CONFIG_1));
	}
	gfx_v7_0_select_se_sh(adev, 0xffffffff, 0xffffffff);

	dev_info(adev->dev, "  GB_ADDR_CONFIG=0x%08X\n",
		 RREG32(mmGB_ADDR_CONFIG));
	dev_info(adev->dev, "  HDP_ADDR_CONFIG=0x%08X\n",
		 RREG32(mmHDP_ADDR_CONFIG));
	dev_info(adev->dev, "  DMIF_ADDR_CALC=0x%08X\n",
		 RREG32(mmDMIF_ADDR_CALC));
	dev_info(adev->dev, "  SDMA0_TILING_CONFIG=0x%08X\n",
		 RREG32(mmSDMA0_TILING_CONFIG + SDMA0_REGISTER_OFFSET));
	dev_info(adev->dev, "  SDMA1_TILING_CONFIG=0x%08X\n",
		 RREG32(mmSDMA0_TILING_CONFIG + SDMA1_REGISTER_OFFSET));
	dev_info(adev->dev, "  UVD_UDEC_ADDR_CONFIG=0x%08X\n",
		 RREG32(mmUVD_UDEC_ADDR_CONFIG));
	dev_info(adev->dev, "  UVD_UDEC_DB_ADDR_CONFIG=0x%08X\n",
		 RREG32(mmUVD_UDEC_DB_ADDR_CONFIG));
	dev_info(adev->dev, "  UVD_UDEC_DBW_ADDR_CONFIG=0x%08X\n",
		 RREG32(mmUVD_UDEC_DBW_ADDR_CONFIG));

	dev_info(adev->dev, "  CP_MEQ_THRESHOLDS=0x%08X\n",
		 RREG32(mmCP_MEQ_THRESHOLDS));
	dev_info(adev->dev, "  SX_DEBUG_1=0x%08X\n",
		 RREG32(mmSX_DEBUG_1));
	dev_info(adev->dev, "  TA_CNTL_AUX=0x%08X\n",
		 RREG32(mmTA_CNTL_AUX));
	dev_info(adev->dev, "  SPI_CONFIG_CNTL=0x%08X\n",
		 RREG32(mmSPI_CONFIG_CNTL));
	dev_info(adev->dev, "  SQ_CONFIG=0x%08X\n",
		 RREG32(mmSQ_CONFIG));
	dev_info(adev->dev, "  DB_DEBUG=0x%08X\n",
		 RREG32(mmDB_DEBUG));
	dev_info(adev->dev, "  DB_DEBUG2=0x%08X\n",
		 RREG32(mmDB_DEBUG2));
	dev_info(adev->dev, "  DB_DEBUG3=0x%08X\n",
		 RREG32(mmDB_DEBUG3));
	dev_info(adev->dev, "  CB_HW_CONTROL=0x%08X\n",
		 RREG32(mmCB_HW_CONTROL));
	dev_info(adev->dev, "  SPI_CONFIG_CNTL_1=0x%08X\n",
		 RREG32(mmSPI_CONFIG_CNTL_1));
	dev_info(adev->dev, "  PA_SC_FIFO_SIZE=0x%08X\n",
		 RREG32(mmPA_SC_FIFO_SIZE));
	dev_info(adev->dev, "  VGT_NUM_INSTANCES=0x%08X\n",
		 RREG32(mmVGT_NUM_INSTANCES));
	dev_info(adev->dev, "  CP_PERFMON_CNTL=0x%08X\n",
		 RREG32(mmCP_PERFMON_CNTL));
	dev_info(adev->dev, "  PA_SC_FORCE_EOV_MAX_CNTS=0x%08X\n",
		 RREG32(mmPA_SC_FORCE_EOV_MAX_CNTS));
	dev_info(adev->dev, "  VGT_CACHE_INVALIDATION=0x%08X\n",
		 RREG32(mmVGT_CACHE_INVALIDATION));
	dev_info(adev->dev, "  VGT_GS_VERTEX_REUSE=0x%08X\n",
		 RREG32(mmVGT_GS_VERTEX_REUSE));
	dev_info(adev->dev, "  PA_SC_LINE_STIPPLE_STATE=0x%08X\n",
		 RREG32(mmPA_SC_LINE_STIPPLE_STATE));
	dev_info(adev->dev, "  PA_CL_ENHANCE=0x%08X\n",
		 RREG32(mmPA_CL_ENHANCE));
	dev_info(adev->dev, "  PA_SC_ENHANCE=0x%08X\n",
		 RREG32(mmPA_SC_ENHANCE));

	dev_info(adev->dev, "  CP_ME_CNTL=0x%08X\n",
		 RREG32(mmCP_ME_CNTL));
	dev_info(adev->dev, "  CP_MAX_CONTEXT=0x%08X\n",
		 RREG32(mmCP_MAX_CONTEXT));
	dev_info(adev->dev, "  CP_ENDIAN_SWAP=0x%08X\n",
		 RREG32(mmCP_ENDIAN_SWAP));
	dev_info(adev->dev, "  CP_DEVICE_ID=0x%08X\n",
		 RREG32(mmCP_DEVICE_ID));

	dev_info(adev->dev, "  CP_SEM_WAIT_TIMER=0x%08X\n",
		 RREG32(mmCP_SEM_WAIT_TIMER));
	if (adev->asic_type != CHIP_HAWAII)
		dev_info(adev->dev, "  CP_SEM_INCOMPLETE_TIMER_CNTL=0x%08X\n",
			 RREG32(mmCP_SEM_INCOMPLETE_TIMER_CNTL));

	dev_info(adev->dev, "  CP_RB_WPTR_DELAY=0x%08X\n",
		 RREG32(mmCP_RB_WPTR_DELAY));
	dev_info(adev->dev, "  CP_RB_VMID=0x%08X\n",
		 RREG32(mmCP_RB_VMID));
	dev_info(adev->dev, "  CP_RB0_CNTL=0x%08X\n",
		 RREG32(mmCP_RB0_CNTL));
	dev_info(adev->dev, "  CP_RB0_WPTR=0x%08X\n",
		 RREG32(mmCP_RB0_WPTR));
	dev_info(adev->dev, "  CP_RB0_RPTR_ADDR=0x%08X\n",
		 RREG32(mmCP_RB0_RPTR_ADDR));
	dev_info(adev->dev, "  CP_RB0_RPTR_ADDR_HI=0x%08X\n",
		 RREG32(mmCP_RB0_RPTR_ADDR_HI));
	dev_info(adev->dev, "  CP_RB0_CNTL=0x%08X\n",
		 RREG32(mmCP_RB0_CNTL));
	dev_info(adev->dev, "  CP_RB0_BASE=0x%08X\n",
		 RREG32(mmCP_RB0_BASE));
	dev_info(adev->dev, "  CP_RB0_BASE_HI=0x%08X\n",
		 RREG32(mmCP_RB0_BASE_HI));
	dev_info(adev->dev, "  CP_MEC_CNTL=0x%08X\n",
		 RREG32(mmCP_MEC_CNTL));
	dev_info(adev->dev, "  CP_CPF_DEBUG=0x%08X\n",
		 RREG32(mmCP_CPF_DEBUG));

	dev_info(adev->dev, "  SCRATCH_ADDR=0x%08X\n",
		 RREG32(mmSCRATCH_ADDR));
	dev_info(adev->dev, "  SCRATCH_UMSK=0x%08X\n",
		 RREG32(mmSCRATCH_UMSK));

	/* init the pipes */
	mutex_lock(&adev->srbm_mutex);
	for (i = 0; i < (adev->gfx.mec.num_pipe * adev->gfx.mec.num_mec); i++) {
		int me = (i < 4) ? 1 : 2;
		int pipe = (i < 4) ? i : (i - 4);
		int queue;

		dev_info(adev->dev, "  me: %d, pipe: %d\n", me, pipe);
		cik_srbm_select(adev, me, pipe, 0, 0);
		dev_info(adev->dev, "  CP_HPD_EOP_BASE_ADDR=0x%08X\n",
			 RREG32(mmCP_HPD_EOP_BASE_ADDR));
		dev_info(adev->dev, "  CP_HPD_EOP_BASE_ADDR_HI=0x%08X\n",
			 RREG32(mmCP_HPD_EOP_BASE_ADDR_HI));
		dev_info(adev->dev, "  CP_HPD_EOP_VMID=0x%08X\n",
			 RREG32(mmCP_HPD_EOP_VMID));
		dev_info(adev->dev, "  CP_HPD_EOP_CONTROL=0x%08X\n",
			 RREG32(mmCP_HPD_EOP_CONTROL));

		for (queue = 0; queue < 8; queue++) {
			cik_srbm_select(adev, me, pipe, queue, 0);
			dev_info(adev->dev, "  queue: %d\n", queue);
			dev_info(adev->dev, "  CP_PQ_WPTR_POLL_CNTL=0x%08X\n",
				 RREG32(mmCP_PQ_WPTR_POLL_CNTL));
			dev_info(adev->dev, "  CP_HQD_PQ_DOORBELL_CONTROL=0x%08X\n",
				 RREG32(mmCP_HQD_PQ_DOORBELL_CONTROL));
			dev_info(adev->dev, "  CP_HQD_ACTIVE=0x%08X\n",
				 RREG32(mmCP_HQD_ACTIVE));
			dev_info(adev->dev, "  CP_HQD_DEQUEUE_REQUEST=0x%08X\n",
				 RREG32(mmCP_HQD_DEQUEUE_REQUEST));
			dev_info(adev->dev, "  CP_HQD_PQ_RPTR=0x%08X\n",
				 RREG32(mmCP_HQD_PQ_RPTR));
			dev_info(adev->dev, "  CP_HQD_PQ_WPTR=0x%08X\n",
				 RREG32(mmCP_HQD_PQ_WPTR));
			dev_info(adev->dev, "  CP_HQD_PQ_BASE=0x%08X\n",
				 RREG32(mmCP_HQD_PQ_BASE));
			dev_info(adev->dev, "  CP_HQD_PQ_BASE_HI=0x%08X\n",
				 RREG32(mmCP_HQD_PQ_BASE_HI));
			dev_info(adev->dev, "  CP_HQD_PQ_CONTROL=0x%08X\n",
				 RREG32(mmCP_HQD_PQ_CONTROL));
			dev_info(adev->dev, "  CP_HQD_PQ_WPTR_POLL_ADDR=0x%08X\n",
				 RREG32(mmCP_HQD_PQ_WPTR_POLL_ADDR));
			dev_info(adev->dev, "  CP_HQD_PQ_WPTR_POLL_ADDR_HI=0x%08X\n",
				 RREG32(mmCP_HQD_PQ_WPTR_POLL_ADDR_HI));
			dev_info(adev->dev, "  CP_HQD_PQ_RPTR_REPORT_ADDR=0x%08X\n",
				 RREG32(mmCP_HQD_PQ_RPTR_REPORT_ADDR));
			dev_info(adev->dev, "  CP_HQD_PQ_RPTR_REPORT_ADDR_HI=0x%08X\n",
				 RREG32(mmCP_HQD_PQ_RPTR_REPORT_ADDR_HI));
			dev_info(adev->dev, "  CP_HQD_PQ_DOORBELL_CONTROL=0x%08X\n",
				 RREG32(mmCP_HQD_PQ_DOORBELL_CONTROL));
			dev_info(adev->dev, "  CP_HQD_PQ_WPTR=0x%08X\n",
				 RREG32(mmCP_HQD_PQ_WPTR));
			dev_info(adev->dev, "  CP_HQD_VMID=0x%08X\n",
				 RREG32(mmCP_HQD_VMID));
			dev_info(adev->dev, "  CP_MQD_BASE_ADDR=0x%08X\n",
				 RREG32(mmCP_MQD_BASE_ADDR));
			dev_info(adev->dev, "  CP_MQD_BASE_ADDR_HI=0x%08X\n",
				 RREG32(mmCP_MQD_BASE_ADDR_HI));
			dev_info(adev->dev, "  CP_MQD_CONTROL=0x%08X\n",
				 RREG32(mmCP_MQD_CONTROL));
		}
	}
	cik_srbm_select(adev, 0, 0, 0, 0);
	mutex_unlock(&adev->srbm_mutex);

	dev_info(adev->dev, "  CP_INT_CNTL_RING0=0x%08X\n",
		 RREG32(mmCP_INT_CNTL_RING0));
	dev_info(adev->dev, "  RLC_LB_CNTL=0x%08X\n",
		 RREG32(mmRLC_LB_CNTL));
	dev_info(adev->dev, "  RLC_CNTL=0x%08X\n",
		 RREG32(mmRLC_CNTL));
	dev_info(adev->dev, "  RLC_CGCG_CGLS_CTRL=0x%08X\n",
		 RREG32(mmRLC_CGCG_CGLS_CTRL));
	dev_info(adev->dev, "  RLC_LB_CNTR_INIT=0x%08X\n",
		 RREG32(mmRLC_LB_CNTR_INIT));
	dev_info(adev->dev, "  RLC_LB_CNTR_MAX=0x%08X\n",
		 RREG32(mmRLC_LB_CNTR_MAX));
	dev_info(adev->dev, "  RLC_LB_INIT_CU_MASK=0x%08X\n",
		 RREG32(mmRLC_LB_INIT_CU_MASK));
	dev_info(adev->dev, "  RLC_LB_PARAMS=0x%08X\n",
		 RREG32(mmRLC_LB_PARAMS));
	dev_info(adev->dev, "  RLC_LB_CNTL=0x%08X\n",
		 RREG32(mmRLC_LB_CNTL));
	dev_info(adev->dev, "  RLC_MC_CNTL=0x%08X\n",
		 RREG32(mmRLC_MC_CNTL));
	dev_info(adev->dev, "  RLC_UCODE_CNTL=0x%08X\n",
		 RREG32(mmRLC_UCODE_CNTL));

	if (adev->asic_type == CHIP_BONAIRE)
		dev_info(adev->dev, "  RLC_DRIVER_CPDMA_STATUS=0x%08X\n",
			 RREG32(mmRLC_DRIVER_CPDMA_STATUS));

	mutex_lock(&adev->srbm_mutex);
	for (i = 0; i < 16; i++) {
		cik_srbm_select(adev, 0, 0, 0, i);
		dev_info(adev->dev, "  VM %d:\n", i);
		dev_info(adev->dev, "  SH_MEM_CONFIG=0x%08X\n",
			 RREG32(mmSH_MEM_CONFIG));
		dev_info(adev->dev, "  SH_MEM_APE1_BASE=0x%08X\n",
			 RREG32(mmSH_MEM_APE1_BASE));
		dev_info(adev->dev, "  SH_MEM_APE1_LIMIT=0x%08X\n",
			 RREG32(mmSH_MEM_APE1_LIMIT));
		dev_info(adev->dev, "  SH_MEM_BASES=0x%08X\n",
			 RREG32(mmSH_MEM_BASES));
	}
	cik_srbm_select(adev, 0, 0, 0, 0);
	mutex_unlock(&adev->srbm_mutex);
}

static int gfx_v7_0_soft_reset(void *handle)
{
	u32 grbm_soft_reset = 0, srbm_soft_reset = 0;
	u32 tmp;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	/* GRBM_STATUS */
	tmp = RREG32(mmGRBM_STATUS);
	if (tmp & (GRBM_STATUS__PA_BUSY_MASK | GRBM_STATUS__SC_BUSY_MASK |
		   GRBM_STATUS__BCI_BUSY_MASK | GRBM_STATUS__SX_BUSY_MASK |
		   GRBM_STATUS__TA_BUSY_MASK | GRBM_STATUS__VGT_BUSY_MASK |
		   GRBM_STATUS__DB_BUSY_MASK | GRBM_STATUS__CB_BUSY_MASK |
		   GRBM_STATUS__GDS_BUSY_MASK | GRBM_STATUS__SPI_BUSY_MASK |
		   GRBM_STATUS__IA_BUSY_MASK | GRBM_STATUS__IA_BUSY_NO_DMA_MASK))
		grbm_soft_reset |= GRBM_SOFT_RESET__SOFT_RESET_CP_MASK |
			GRBM_SOFT_RESET__SOFT_RESET_GFX_MASK;

	if (tmp & (GRBM_STATUS__CP_BUSY_MASK | GRBM_STATUS__CP_COHERENCY_BUSY_MASK)) {
		grbm_soft_reset |= GRBM_SOFT_RESET__SOFT_RESET_CP_MASK;
		srbm_soft_reset |= SRBM_SOFT_RESET__SOFT_RESET_GRBM_MASK;
	}

	/* GRBM_STATUS2 */
	tmp = RREG32(mmGRBM_STATUS2);
	if (tmp & GRBM_STATUS2__RLC_BUSY_MASK)
		grbm_soft_reset |= GRBM_SOFT_RESET__SOFT_RESET_RLC_MASK;

	/* SRBM_STATUS */
	tmp = RREG32(mmSRBM_STATUS);
	if (tmp & SRBM_STATUS__GRBM_RQ_PENDING_MASK)
		srbm_soft_reset |= SRBM_SOFT_RESET__SOFT_RESET_GRBM_MASK;

	if (grbm_soft_reset || srbm_soft_reset) {
		gfx_v7_0_print_status((void *)adev);
		/* disable CG/PG */
		gfx_v7_0_fini_pg(adev);
		gfx_v7_0_update_cg(adev, false);

		/* stop the rlc */
		gfx_v7_0_rlc_stop(adev);

		/* Disable GFX parsing/prefetching */
		WREG32(mmCP_ME_CNTL, CP_ME_CNTL__ME_HALT_MASK | CP_ME_CNTL__PFP_HALT_MASK | CP_ME_CNTL__CE_HALT_MASK);

		/* Disable MEC parsing/prefetching */
		WREG32(mmCP_MEC_CNTL, CP_MEC_CNTL__MEC_ME1_HALT_MASK | CP_MEC_CNTL__MEC_ME2_HALT_MASK);

		if (grbm_soft_reset) {
			tmp = RREG32(mmGRBM_SOFT_RESET);
			tmp |= grbm_soft_reset;
			dev_info(adev->dev, "GRBM_SOFT_RESET=0x%08X\n", tmp);
			WREG32(mmGRBM_SOFT_RESET, tmp);
			tmp = RREG32(mmGRBM_SOFT_RESET);

			udelay(50);

			tmp &= ~grbm_soft_reset;
			WREG32(mmGRBM_SOFT_RESET, tmp);
			tmp = RREG32(mmGRBM_SOFT_RESET);
		}

		if (srbm_soft_reset) {
			tmp = RREG32(mmSRBM_SOFT_RESET);
			tmp |= srbm_soft_reset;
			dev_info(adev->dev, "SRBM_SOFT_RESET=0x%08X\n", tmp);
			WREG32(mmSRBM_SOFT_RESET, tmp);
			tmp = RREG32(mmSRBM_SOFT_RESET);

			udelay(50);

			tmp &= ~srbm_soft_reset;
			WREG32(mmSRBM_SOFT_RESET, tmp);
			tmp = RREG32(mmSRBM_SOFT_RESET);
		}
		/* Wait a little for things to settle down */
		udelay(50);
		gfx_v7_0_print_status((void *)adev);
	}
	return 0;
}

static void gfx_v7_0_set_gfx_eop_interrupt_state(struct amdgpu_device *adev,
						 enum amdgpu_interrupt_state state)
{
	u32 cp_int_cntl;

	switch (state) {
	case AMDGPU_IRQ_STATE_DISABLE:
		cp_int_cntl = RREG32(mmCP_INT_CNTL_RING0);
		cp_int_cntl &= ~CP_INT_CNTL_RING0__TIME_STAMP_INT_ENABLE_MASK;
		WREG32(mmCP_INT_CNTL_RING0, cp_int_cntl);
		break;
	case AMDGPU_IRQ_STATE_ENABLE:
		cp_int_cntl = RREG32(mmCP_INT_CNTL_RING0);
		cp_int_cntl |= CP_INT_CNTL_RING0__TIME_STAMP_INT_ENABLE_MASK;
		WREG32(mmCP_INT_CNTL_RING0, cp_int_cntl);
		break;
	default:
		break;
	}
}

static void gfx_v7_0_set_compute_eop_interrupt_state(struct amdgpu_device *adev,
						     int me, int pipe,
						     enum amdgpu_interrupt_state state)
{
	u32 mec_int_cntl, mec_int_cntl_reg;

	/*
	 * amdgpu controls only pipe 0 of MEC1. That's why this function only
	 * handles the setting of interrupts for this specific pipe. All other
	 * pipes' interrupts are set by amdkfd.
	 */

	if (me == 1) {
		switch (pipe) {
		case 0:
			mec_int_cntl_reg = mmCP_ME1_PIPE0_INT_CNTL;
			break;
		default:
			DRM_DEBUG("invalid pipe %d\n", pipe);
			return;
		}
	} else {
		DRM_DEBUG("invalid me %d\n", me);
		return;
	}

	switch (state) {
	case AMDGPU_IRQ_STATE_DISABLE:
		mec_int_cntl = RREG32(mec_int_cntl_reg);
		mec_int_cntl &= ~CP_INT_CNTL_RING0__TIME_STAMP_INT_ENABLE_MASK;
		WREG32(mec_int_cntl_reg, mec_int_cntl);
		break;
	case AMDGPU_IRQ_STATE_ENABLE:
		mec_int_cntl = RREG32(mec_int_cntl_reg);
		mec_int_cntl |= CP_INT_CNTL_RING0__TIME_STAMP_INT_ENABLE_MASK;
		WREG32(mec_int_cntl_reg, mec_int_cntl);
		break;
	default:
		break;
	}
}

static int gfx_v7_0_set_priv_reg_fault_state(struct amdgpu_device *adev,
					     struct amdgpu_irq_src *src,
					     unsigned type,
					     enum amdgpu_interrupt_state state)
{
	u32 cp_int_cntl;

	switch (state) {
	case AMDGPU_IRQ_STATE_DISABLE:
		cp_int_cntl = RREG32(mmCP_INT_CNTL_RING0);
		cp_int_cntl &= ~CP_INT_CNTL_RING0__PRIV_REG_INT_ENABLE_MASK;
		WREG32(mmCP_INT_CNTL_RING0, cp_int_cntl);
		break;
	case AMDGPU_IRQ_STATE_ENABLE:
		cp_int_cntl = RREG32(mmCP_INT_CNTL_RING0);
		cp_int_cntl |= CP_INT_CNTL_RING0__PRIV_REG_INT_ENABLE_MASK;
		WREG32(mmCP_INT_CNTL_RING0, cp_int_cntl);
		break;
	default:
		break;
	}

	return 0;
}

static int gfx_v7_0_set_priv_inst_fault_state(struct amdgpu_device *adev,
					      struct amdgpu_irq_src *src,
					      unsigned type,
					      enum amdgpu_interrupt_state state)
{
	u32 cp_int_cntl;

	switch (state) {
	case AMDGPU_IRQ_STATE_DISABLE:
		cp_int_cntl = RREG32(mmCP_INT_CNTL_RING0);
		cp_int_cntl &= ~CP_INT_CNTL_RING0__PRIV_INSTR_INT_ENABLE_MASK;
		WREG32(mmCP_INT_CNTL_RING0, cp_int_cntl);
		break;
	case AMDGPU_IRQ_STATE_ENABLE:
		cp_int_cntl = RREG32(mmCP_INT_CNTL_RING0);
		cp_int_cntl |= CP_INT_CNTL_RING0__PRIV_INSTR_INT_ENABLE_MASK;
		WREG32(mmCP_INT_CNTL_RING0, cp_int_cntl);
		break;
	default:
		break;
	}

	return 0;
}

static int gfx_v7_0_set_eop_interrupt_state(struct amdgpu_device *adev,
					    struct amdgpu_irq_src *src,
					    unsigned type,
					    enum amdgpu_interrupt_state state)
{
	switch (type) {
	case AMDGPU_CP_IRQ_GFX_EOP:
		gfx_v7_0_set_gfx_eop_interrupt_state(adev, state);
		break;
	case AMDGPU_CP_IRQ_COMPUTE_MEC1_PIPE0_EOP:
		gfx_v7_0_set_compute_eop_interrupt_state(adev, 1, 0, state);
		break;
	case AMDGPU_CP_IRQ_COMPUTE_MEC1_PIPE1_EOP:
		gfx_v7_0_set_compute_eop_interrupt_state(adev, 1, 1, state);
		break;
	case AMDGPU_CP_IRQ_COMPUTE_MEC1_PIPE2_EOP:
		gfx_v7_0_set_compute_eop_interrupt_state(adev, 1, 2, state);
		break;
	case AMDGPU_CP_IRQ_COMPUTE_MEC1_PIPE3_EOP:
		gfx_v7_0_set_compute_eop_interrupt_state(adev, 1, 3, state);
		break;
	case AMDGPU_CP_IRQ_COMPUTE_MEC2_PIPE0_EOP:
		gfx_v7_0_set_compute_eop_interrupt_state(adev, 2, 0, state);
		break;
	case AMDGPU_CP_IRQ_COMPUTE_MEC2_PIPE1_EOP:
		gfx_v7_0_set_compute_eop_interrupt_state(adev, 2, 1, state);
		break;
	case AMDGPU_CP_IRQ_COMPUTE_MEC2_PIPE2_EOP:
		gfx_v7_0_set_compute_eop_interrupt_state(adev, 2, 2, state);
		break;
	case AMDGPU_CP_IRQ_COMPUTE_MEC2_PIPE3_EOP:
		gfx_v7_0_set_compute_eop_interrupt_state(adev, 2, 3, state);
		break;
	default:
		break;
	}
	return 0;
}

static int gfx_v7_0_eop_irq(struct amdgpu_device *adev,
			    struct amdgpu_irq_src *source,
			    struct amdgpu_iv_entry *entry)
{
	u8 me_id, pipe_id;
	struct amdgpu_ring *ring;
	int i;

	DRM_DEBUG("IH: CP EOP\n");
	me_id = (entry->ring_id & 0x0c) >> 2;
	pipe_id = (entry->ring_id & 0x03) >> 0;
	switch (me_id) {
	case 0:
		amdgpu_fence_process(&adev->gfx.gfx_ring[0]);
		break;
	case 1:
	case 2:
		for (i = 0; i < adev->gfx.num_compute_rings; i++) {
			ring = &adev->gfx.compute_ring[i];
			if ((ring->me == me_id) && (ring->pipe == pipe_id))
				amdgpu_fence_process(ring);
		}
		break;
	}
	return 0;
}

static int gfx_v7_0_priv_reg_irq(struct amdgpu_device *adev,
				 struct amdgpu_irq_src *source,
				 struct amdgpu_iv_entry *entry)
{
	DRM_ERROR("Illegal register access in command stream\n");
	schedule_work(&adev->reset_work);
	return 0;
}

static int gfx_v7_0_priv_inst_irq(struct amdgpu_device *adev,
				  struct amdgpu_irq_src *source,
				  struct amdgpu_iv_entry *entry)
{
	DRM_ERROR("Illegal instruction in command stream\n");
	// XXX soft reset the gfx block only
	schedule_work(&adev->reset_work);
	return 0;
}

static int gfx_v7_0_set_clockgating_state(void *handle,
					  enum amd_clockgating_state state)
{
	bool gate = false;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	if (state == AMD_CG_STATE_GATE)
		gate = true;

	gfx_v7_0_enable_gui_idle_interrupt(adev, false);
	/* order matters! */
	if (gate) {
		gfx_v7_0_enable_mgcg(adev, true);
		gfx_v7_0_enable_cgcg(adev, true);
	} else {
		gfx_v7_0_enable_cgcg(adev, false);
		gfx_v7_0_enable_mgcg(adev, false);
	}
	gfx_v7_0_enable_gui_idle_interrupt(adev, true);

	return 0;
}

static int gfx_v7_0_set_powergating_state(void *handle,
					  enum amd_powergating_state state)
{
	bool gate = false;
	struct amdgpu_device *adev = (struct amdgpu_device *)handle;

	if (state == AMD_PG_STATE_GATE)
		gate = true;

	if (adev->pg_flags & (AMD_PG_SUPPORT_GFX_PG |
			      AMD_PG_SUPPORT_GFX_SMG |
			      AMD_PG_SUPPORT_GFX_DMG |
			      AMD_PG_SUPPORT_CP |
			      AMD_PG_SUPPORT_GDS |
			      AMD_PG_SUPPORT_RLC_SMU_HS)) {
		gfx_v7_0_update_gfx_pg(adev, gate);
		if (adev->pg_flags & AMD_PG_SUPPORT_GFX_PG) {
			gfx_v7_0_enable_cp_pg(adev, gate);
			gfx_v7_0_enable_gds_pg(adev, gate);
		}
	}

	return 0;
}

const struct amd_ip_funcs gfx_v7_0_ip_funcs = {
	.early_init = gfx_v7_0_early_init,
	.late_init = gfx_v7_0_late_init,
	.sw_init = gfx_v7_0_sw_init,
	.sw_fini = gfx_v7_0_sw_fini,
	.hw_init = gfx_v7_0_hw_init,
	.hw_fini = gfx_v7_0_hw_fini,
	.suspend = gfx_v7_0_suspend,
	.resume = gfx_v7_0_resume,
	.is_idle = gfx_v7_0_is_idle,
	.wait_for_idle = gfx_v7_0_wait_for_idle,
	.soft_reset = gfx_v7_0_soft_reset,
	.print_status = gfx_v7_0_print_status,
	.set_clockgating_state = gfx_v7_0_set_clockgating_state,
	.set_powergating_state = gfx_v7_0_set_powergating_state,
};

static const struct amdgpu_ring_funcs gfx_v7_0_ring_funcs_gfx = {
	.get_rptr = gfx_v7_0_ring_get_rptr_gfx,
	.get_wptr = gfx_v7_0_ring_get_wptr_gfx,
	.set_wptr = gfx_v7_0_ring_set_wptr_gfx,
	.parse_cs = NULL,
	.emit_ib = gfx_v7_0_ring_emit_ib_gfx,
	.emit_fence = gfx_v7_0_ring_emit_fence_gfx,
	.emit_semaphore = gfx_v7_0_ring_emit_semaphore,
	.emit_vm_flush = gfx_v7_0_ring_emit_vm_flush,
	.emit_gds_switch = gfx_v7_0_ring_emit_gds_switch,
	.emit_hdp_flush = gfx_v7_0_ring_emit_hdp_flush,
	.test_ring = gfx_v7_0_ring_test_ring,
	.test_ib = gfx_v7_0_ring_test_ib,
	.insert_nop = amdgpu_ring_insert_nop,
};

static const struct amdgpu_ring_funcs gfx_v7_0_ring_funcs_compute = {
	.get_rptr = gfx_v7_0_ring_get_rptr_compute,
	.get_wptr = gfx_v7_0_ring_get_wptr_compute,
	.set_wptr = gfx_v7_0_ring_set_wptr_compute,
	.parse_cs = NULL,
	.emit_ib = gfx_v7_0_ring_emit_ib_compute,
	.emit_fence = gfx_v7_0_ring_emit_fence_compute,
	.emit_semaphore = gfx_v7_0_ring_emit_semaphore,
	.emit_vm_flush = gfx_v7_0_ring_emit_vm_flush,
	.emit_gds_switch = gfx_v7_0_ring_emit_gds_switch,
	.emit_hdp_flush = gfx_v7_0_ring_emit_hdp_flush,
	.test_ring = gfx_v7_0_ring_test_ring,
	.test_ib = gfx_v7_0_ring_test_ib,
	.insert_nop = amdgpu_ring_insert_nop,
};

static void gfx_v7_0_set_ring_funcs(struct amdgpu_device *adev)
{
	int i;

	for (i = 0; i < adev->gfx.num_gfx_rings; i++)
		adev->gfx.gfx_ring[i].funcs = &gfx_v7_0_ring_funcs_gfx;
	for (i = 0; i < adev->gfx.num_compute_rings; i++)
		adev->gfx.compute_ring[i].funcs = &gfx_v7_0_ring_funcs_compute;
}

static const struct amdgpu_irq_src_funcs gfx_v7_0_eop_irq_funcs = {
	.set = gfx_v7_0_set_eop_interrupt_state,
	.process = gfx_v7_0_eop_irq,
};

static const struct amdgpu_irq_src_funcs gfx_v7_0_priv_reg_irq_funcs = {
	.set = gfx_v7_0_set_priv_reg_fault_state,
	.process = gfx_v7_0_priv_reg_irq,
};

static const struct amdgpu_irq_src_funcs gfx_v7_0_priv_inst_irq_funcs = {
	.set = gfx_v7_0_set_priv_inst_fault_state,
	.process = gfx_v7_0_priv_inst_irq,
};

static void gfx_v7_0_set_irq_funcs(struct amdgpu_device *adev)
{
	adev->gfx.eop_irq.num_types = AMDGPU_CP_IRQ_LAST;
	adev->gfx.eop_irq.funcs = &gfx_v7_0_eop_irq_funcs;

	adev->gfx.priv_reg_irq.num_types = 1;
	adev->gfx.priv_reg_irq.funcs = &gfx_v7_0_priv_reg_irq_funcs;

	adev->gfx.priv_inst_irq.num_types = 1;
	adev->gfx.priv_inst_irq.funcs = &gfx_v7_0_priv_inst_irq_funcs;
}

static void gfx_v7_0_set_gds_init(struct amdgpu_device *adev)
{
	/* init asci gds info */
	adev->gds.mem.total_size = RREG32(mmGDS_VMID0_SIZE);
	adev->gds.gws.total_size = 64;
	adev->gds.oa.total_size = 16;

	if (adev->gds.mem.total_size == 64 * 1024) {
		adev->gds.mem.gfx_partition_size = 4096;
		adev->gds.mem.cs_partition_size = 4096;

		adev->gds.gws.gfx_partition_size = 4;
		adev->gds.gws.cs_partition_size = 4;

		adev->gds.oa.gfx_partition_size = 4;
		adev->gds.oa.cs_partition_size = 1;
	} else {
		adev->gds.mem.gfx_partition_size = 1024;
		adev->gds.mem.cs_partition_size = 1024;

		adev->gds.gws.gfx_partition_size = 16;
		adev->gds.gws.cs_partition_size = 16;

		adev->gds.oa.gfx_partition_size = 4;
		adev->gds.oa.cs_partition_size = 4;
	}
}


int gfx_v7_0_get_cu_info(struct amdgpu_device *adev,
								   struct amdgpu_cu_info *cu_info)
{
	int i, j, k, counter, active_cu_number = 0;
	u32 mask, bitmap, ao_bitmap, ao_cu_mask = 0;

	if (!adev || !cu_info)
		return -EINVAL;

	mutex_lock(&adev->grbm_idx_mutex);
	for (i = 0; i < adev->gfx.config.max_shader_engines; i++) {
		for (j = 0; j < adev->gfx.config.max_sh_per_se; j++) {
			mask = 1;
			ao_bitmap = 0;
			counter = 0;
			bitmap = gfx_v7_0_get_cu_active_bitmap(adev, i, j);
			cu_info->bitmap[i][j] = bitmap;

			for (k = 0; k < adev->gfx.config.max_cu_per_sh; k ++) {
				if (bitmap & mask) {
					if (counter < 2)
						ao_bitmap |= mask;
					counter ++;
				}
				mask <<= 1;
			}
			active_cu_number += counter;
			ao_cu_mask |= (ao_bitmap << (i * 16 + j * 8));
		}
	}

	cu_info->number = active_cu_number;
	cu_info->ao_cu_mask = ao_cu_mask;
	mutex_unlock(&adev->grbm_idx_mutex);
	return 0;
}
