/*
 * Copyright(c) 2011-2016 Intel Corporation. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _I915_PVINFO_H_
#define _I915_PVINFO_H_

/* The MMIO offset of the shared info between guest and host emulator */
#define VGT_PVINFO_PAGE	0x78000
#define VGT_PVINFO_SIZE	0x1000

/*
 * The following structure pages are defined in GEN MMIO space
 * for virtualization. (One page for now)
 */
#define VGT_MAGIC         0x4776544776544776ULL	/* 'vGTvGTvG' */
#define VGT_VERSION_MAJOR 1
#define VGT_VERSION_MINOR 0

#define INTEL_VGT_IF_VERSION_ENCODE(major, minor) ((major) << 16 | (minor))
#define INTEL_VGT_IF_VERSION \
	INTEL_VGT_IF_VERSION_ENCODE(VGT_VERSION_MAJOR, VGT_VERSION_MINOR)

/*
 * notifications from guest to vgpu device model
 */
enum vgt_g2v_type {
	VGT_G2V_PPGTT_L3_PAGE_TABLE_CREATE = 2,
	VGT_G2V_PPGTT_L3_PAGE_TABLE_DESTROY,
	VGT_G2V_PPGTT_L4_PAGE_TABLE_CREATE,
	VGT_G2V_PPGTT_L4_PAGE_TABLE_DESTROY,
	VGT_G2V_EXECLIST_CONTEXT_CREATE,
	VGT_G2V_EXECLIST_CONTEXT_DESTROY,
	VGT_G2V_SET_PTE_PAGE,
	VGT_G2V_SET_PTE,
	VGT_G2V_MAX,
};

/*
 * a job queue of pte set instructions is essentially a vector of job entries;
 */
struct set_pte_job_entry {
	u64 index;
	u64 pte;
} __packed;

/* shared page(4KB) between gvt and VM, located at the first page next
 * to MMIO region(2MB size normally).
 */
struct gvt_shared_page {
	u32 elsp_data[4];
	u32 reg_addr; /* register address of pv mmio read op */
	u32 flip_regs[6];
	u32 rsvd2[0x400 - 11];
};

#define VGPU_PVMMIO(vgpu) vgpu_vreg(vgpu, vgtif_reg(enable_pvmmio))
struct vgt_if {
	u64 magic;		/* VGT_MAGIC */
	uint16_t version_major;
	uint16_t version_minor;
	u32 vgt_id;		/* ID of vGT instance */
	u32 rsv1[12];		/* pad to offset 0x40 */
	/*
	 *  Data structure to describe the balooning info of resources.
	 *  Each VM can only have one portion of continuous area for now.
	 *  (May support scattered resource in future)
	 *  (starting from offset 0x40)
	 */
	struct {
		/* Aperture register balooning */
		struct {
			u32 base;
			u32 size;
		} mappable_gmadr;	/* aperture */
		/* GMADR register balooning */
		struct {
			u32 base;
			u32 size;
		} nonmappable_gmadr;	/* non aperture */
		/* allowed fence registers */
		u32 fence_num;
		u32 rsv2[3];
	} avail_rs;		/* available/assigned resource */
	u32 rsv3[0x200 - 24];	/* pad to half page */
	/*
	 * The bottom half page is for response from Gfx driver to hypervisor.
	 */
	u32 rsv4;
	u32 display_ready;	/* ready for display owner switch */

	u32 rsv5[4];

	u32 g2v_notify;
	u32 rsv6[7];

	struct {
		u32 lo;
		u32 hi;
	} pdp[4];

	struct {
		u32 lo;
		u32 hi;
	} pte_page;

	u32 pte_num;
	u32 execlist_context_descriptor_lo;
	u32 execlist_context_descriptor_hi;
	u32 enable_pvmmio;
	u32 pv_mmio; /* vgpu trapped mmio read will be redirected here */
	u32 mmio_flip;

	u32  rsv7[0x200 - 30];    /* pad to one page */
} __packed;

#define GVT_PV_MMIO_FLIP 1

#define vgtif_reg(x) \
	_MMIO((VGT_PVINFO_PAGE + offsetof(struct vgt_if, x)))

/* vGPU display status to be used by the host side */
#define VGT_DRV_DISPLAY_NOT_READY 0
#define VGT_DRV_DISPLAY_READY     1  /* ready for display switch */

#endif /* _I915_PVINFO_H_ */
