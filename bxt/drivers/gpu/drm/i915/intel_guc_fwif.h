/*
 * Copyright © 2014 Intel Corporation
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#ifndef _INTEL_GUC_FWIF_H
#define _INTEL_GUC_FWIF_H

#define GFXCORE_FAMILY_GEN9		12
#define GFXCORE_FAMILY_UNKNOWN		0x7fffffff

#define GUC_CTX_PRIORITY_KMD_HIGH	0
#define GUC_CTX_PRIORITY_HIGH		1
#define GUC_CTX_PRIORITY_KMD_NORMAL	2
#define GUC_CTX_PRIORITY_NORMAL		3
#define GUC_CTX_PRIORITY_NUM		4

#define GUC_MAX_GPU_CONTEXTS		1024
#define	GUC_INVALID_CTX_ID		GUC_MAX_GPU_CONTEXTS

#define GUC_RENDER_ENGINE		0
#define GUC_VIDEO_ENGINE		1
#define GUC_BLITTER_ENGINE		2
#define GUC_VIDEOENHANCE_ENGINE		3
#define GUC_VIDEO_ENGINE2		4
#define GUC_MAX_ENGINES_NUM		(GUC_VIDEO_ENGINE2 + 1)

/* Work queue item header definitions */
#define WQ_STATUS_ACTIVE		1
#define WQ_STATUS_SUSPENDED		2
#define WQ_STATUS_CMD_ERROR		3
#define WQ_STATUS_ENGINE_ID_NOT_USED	4
#define WQ_STATUS_SUSPENDED_FROM_RESET	5
#define WQ_TYPE_SHIFT			0
#define   WQ_TYPE_BATCH_BUF		(0x1 << WQ_TYPE_SHIFT)
#define   WQ_TYPE_PSEUDO		(0x2 << WQ_TYPE_SHIFT)
#define   WQ_TYPE_INORDER		(0x3 << WQ_TYPE_SHIFT)
#define WQ_TARGET_SHIFT			10
#define WQ_LEN_SHIFT			16
#define WQ_NO_WCFLUSH_WAIT		(1 << 27)
#define WQ_PRESENT_WORKLOAD		(1 << 28)
#define WQ_WORKLOAD_SHIFT		29
#define   WQ_WORKLOAD_GENERAL		(0 << WQ_WORKLOAD_SHIFT)
#define   WQ_WORKLOAD_GPGPU		(1 << WQ_WORKLOAD_SHIFT)
#define   WQ_WORKLOAD_TOUCH		(2 << WQ_WORKLOAD_SHIFT)

#define WQ_RING_TAIL_SHIFT		20
#define WQ_RING_TAIL_MAX		0x7FF	/* 2^11 QWords */
#define WQ_RING_TAIL_MASK		(WQ_RING_TAIL_MAX << WQ_RING_TAIL_SHIFT)

#define GUC_DOORBELL_ENABLED		1
#define GUC_DOORBELL_DISABLED		0

#define GUC_CTX_DESC_ATTR_ACTIVE	(1 << 0)
#define GUC_CTX_DESC_ATTR_PENDING_DB	(1 << 1)
#define GUC_CTX_DESC_ATTR_KERNEL	(1 << 2)
#define GUC_CTX_DESC_ATTR_PREEMPT	(1 << 3)
#define GUC_CTX_DESC_ATTR_RESET		(1 << 4)
#define GUC_CTX_DESC_ATTR_WQLOCKED	(1 << 5)
#define GUC_CTX_DESC_ATTR_PCH		(1 << 6)
#define GUC_CTX_DESC_ATTR_TERMINATED	(1 << 7)

/* The guc control data is 10 DWORDs */
#define GUC_CTL_CTXINFO			0
#define   GUC_CTL_CTXNUM_IN16_SHIFT	0
#define   GUC_CTL_BASE_ADDR_SHIFT	12

#define GUC_CTL_ARAT_HIGH		1
#define GUC_CTL_ARAT_LOW		2

#define GUC_CTL_DEVICE_INFO		3
#define   GUC_CTL_GTTYPE_SHIFT		0
#define   GUC_CTL_COREFAMILY_SHIFT	7

#define GUC_CTL_LOG_PARAMS		4
#define   GUC_LOG_VALID			(1 << 0)
#define   GUC_LOG_NOTIFY_ON_HALF_FULL	(1 << 1)
#define   GUC_LOG_ALLOC_IN_MEGABYTE	(1 << 3)
#define   GUC_LOG_CRASH_PAGES		1
#define   GUC_LOG_CRASH_SHIFT		4
#define   GUC_LOG_DPC_PAGES		7
#define   GUC_LOG_DPC_SHIFT		6
#define   GUC_LOG_ISR_PAGES		7
#define   GUC_LOG_ISR_SHIFT		9
#define   GUC_LOG_BUF_ADDR_SHIFT	12

#define GUC_CTL_PAGE_FAULT_CONTROL	5

#define GUC_CTL_WA			6
#define   GUC_CTL_WA_UK_BY_DRIVER	(1 << 3)

#define GUC_CTL_FEATURE			7
#define   GUC_CTL_VCS2_ENABLED		(1 << 0)
#define   GUC_CTL_KERNEL_SUBMISSIONS	(1 << 1)
#define   GUC_CTL_FEATURE2		(1 << 2)
#define   GUC_CTL_POWER_GATING		(1 << 3)
#define   GUC_CTL_DISABLE_SCHEDULER	(1 << 4)
#define   GUC_CTL_PREEMPTION_LOG	(1 << 5)
#define   GUC_CTL_ENABLE_CP		(1 << 6)
#define   GUC_CTL_ENABLE_SLPC		(1 << 7)
#define   GUC_CTL_RESET_ON_PREMPT_FAILURE	(1 << 8)

#define GUC_CTL_DEBUG			8
#define   GUC_LOG_VERBOSITY_SHIFT	0
#define   GUC_LOG_VERBOSITY_LOW		(0 << GUC_LOG_VERBOSITY_SHIFT)
#define   GUC_LOG_VERBOSITY_MED		(1 << GUC_LOG_VERBOSITY_SHIFT)
#define   GUC_LOG_VERBOSITY_HIGH	(2 << GUC_LOG_VERBOSITY_SHIFT)
#define   GUC_LOG_VERBOSITY_ULTRA	(3 << GUC_LOG_VERBOSITY_SHIFT)
/* Verbosity range-check limits, without the shift */
#define	  GUC_LOG_VERBOSITY_MIN		0
#define	  GUC_LOG_VERBOSITY_MAX		3
#define	  GUC_LOG_VERBOSITY_MASK	0x0000000f
#define	  GUC_LOG_DESTINATION_MASK	(3 << 4)
#define   GUC_LOG_DISABLED		(1 << 6)
#define   GUC_PROFILE_ENABLED		(1 << 7)
#define   GUC_WQ_TRACK_ENABLED		(1 << 8)
#define   GUC_ADS_ENABLED		(1 << 9)
#define   GUC_DEBUG_RESERVED		(1 << 10)
#define   GUC_ADS_ADDR_SHIFT		11
#define   GUC_ADS_ADDR_MASK		0xfffff800

#define GUC_CTL_SHARED_DATA		9

#define GUC_CTL_MAX_DWORDS		(SOFT_SCRATCH_COUNT - 2) /* [1..14] */

/**
 * DOC: GuC Firmware Layout
 *
 * The GuC firmware layout looks like this:
 *
 *     +-------------------------------+
 *     |         uc_css_header         |
 *     |                               |
 *     | contains major/minor version  |
 *     +-------------------------------+
 *     |             uCode             |
 *     +-------------------------------+
 *     |         RSA signature         |
 *     +-------------------------------+
 *     |          modulus key          |
 *     +-------------------------------+
 *     |          exponent val         |
 *     +-------------------------------+
 *
 * The firmware may or may not have modulus key and exponent data. The header,
 * uCode and RSA signature are must-have components that will be used by driver.
 * Length of each components, which is all in dwords, can be found in header.
 * In the case that modulus and exponent are not present in fw, a.k.a truncated
 * image, the length value still appears in header.
 *
 * Driver will do some basic fw size validation based on the following rules:
 *
 * 1. Header, uCode and RSA are must-have components.
 * 2. All firmware components, if they present, are in the sequence illustrated
 *    in the layout table above.
 * 3. Length info of each component can be found in header, in dwords.
 * 4. Modulus and exponent key are not required by driver. They may not appear
 *    in fw. So driver will load a truncated firmware in this case.
 *
 * HuC firmware layout is same as GuC firmware.
 *
 * HuC firmware css header is different. However, the only difference is where
 * the version information is saved. The uc_css_header is unified to support
 * both. Driver should get HuC version from uc_css_header.huc_sw_version, while
 * uc_css_header.guc_sw_version for GuC.
 */

struct uc_css_header {
	uint32_t module_type;
	/* header_size includes all non-uCode bits, including css_header, rsa
	 * key, modulus key and exponent data. */
	uint32_t header_size_dw;
	uint32_t header_version;
	uint32_t module_id;
	uint32_t module_vendor;
	union {
		struct {
			uint8_t day;
			uint8_t month;
			uint16_t year;
		};
		uint32_t date;
	};
	uint32_t size_dw; /* uCode plus header_size_dw */
	uint32_t key_size_dw;
	uint32_t modulus_size_dw;
	uint32_t exponent_size_dw;
	union {
		struct {
			uint8_t hour;
			uint8_t min;
			uint16_t sec;
		};
		uint32_t time;
	};

	char username[8];
	char buildnumber[12];
	union {
		struct {
			uint32_t branch_client_version;
			uint32_t sw_version;
	} guc;
		struct {
			uint32_t sw_version;
			uint32_t reserved;
	} huc;
	};
	uint32_t prod_preprod_fw;
	uint32_t reserved[12];
	uint32_t header_info;
} __packed;

struct guc_doorbell_info {
	u32 db_status;
	u32 cookie;
	u32 reserved[14];
} __packed;

union guc_doorbell_qw {
	struct {
		u32 db_status;
		u32 cookie;
	};
	u64 value_qw;
} __packed;

#define GUC_MAX_DOORBELLS		256
#define GUC_INVALID_DOORBELL_ID		(GUC_MAX_DOORBELLS)

#define GUC_DB_SIZE			(PAGE_SIZE)
#define GUC_WQ_SIZE			(PAGE_SIZE * 2)

/* Work item for submitting workloads into work queue of GuC. */
struct guc_wq_item {
	u32 header;
	u32 context_desc;
	u32 ring_tail;
	u32 fence_id;
} __packed;

struct guc_process_desc {
	u32 context_id;
	u64 db_base_addr;
	u32 head;
	u32 tail;
	u32 error_offset;
	u64 wq_base_addr;
	u32 wq_size_bytes;
	u32 wq_status;
	u32 engine_presence;
	u32 priority;
	u32 reserved[30];
} __packed;

/* engine id and context id is packed into guc_execlist_context.context_id*/
#define GUC_ELC_CTXID_OFFSET		0
#define GUC_ELC_ENGINE_OFFSET		29

/* The execlist context including software and HW information */
struct guc_execlist_context {
	u32 context_desc;
	u32 context_id;
	u32 ring_status;
	u32 ring_lcra;
	u32 ring_begin;
	u32 ring_end;
	u32 ring_next_free_location;
	u32 ring_current_tail_pointer_value;
	u8 engine_state_submit_value;
	u8 engine_state_wait_value;
	u16 pagefault_count;
	u16 engine_submit_queue_count;
} __packed;

/*Context descriptor for communicating between uKernel and Driver*/
struct guc_context_desc {
	u32 sched_common_area;
	u32 context_id;
	u32 pas_id;
	u8 engines_used;
	u64 db_trigger_cpu;
	u32 db_trigger_uk;
	u64 db_trigger_phy;
	u16 db_id;

	struct guc_execlist_context lrc[GUC_MAX_ENGINES_NUM];

	u8 attribute;

	u32 priority;

	u32 wq_sampled_tail_offset;
	u32 wq_total_submit_enqueues;

	u32 process_desc;
	u32 wq_addr;
	u32 wq_size;

	u32 engine_presence;

	u8 engine_suspended;

	u8 reserved0[3];
	u64 reserved1[1];

	u64 desc_private;
} __packed;

#define GUC_FORCEWAKE_RENDER	(1 << 0)
#define GUC_FORCEWAKE_MEDIA	(1 << 1)

#define GUC_POWER_UNSPECIFIED	0
#define GUC_POWER_D0		1
#define GUC_POWER_D1		2
#define GUC_POWER_D2		3
#define GUC_POWER_D3		4

/* Scheduling policy settings */

/* Reset engine upon preempt failure */
#define POLICY_RESET_ENGINE		(1<<0)
/* Preempt to idle on quantum expiry */
#define POLICY_PREEMPT_TO_IDLE		(1<<1)

#define POLICY_MAX_NUM_WI		15

struct guc_policy {
	/* Time for one workload to execute. (in micro seconds) */
	u32 execution_quantum;
	u32 reserved1;

	/* Time to wait for a preemption request to completed before issuing a
	 * reset. (in micro seconds). */
	u32 preemption_time;

	/* How much time to allow to run after the first fault is observed.
	 * Then preempt afterwards. (in micro seconds) */
	u32 fault_time;

	u32 policy_flags;
	u32 reserved[2];
} __packed;

struct guc_policies {
	struct guc_policy policy[GUC_CTX_PRIORITY_NUM][GUC_MAX_ENGINES_NUM];

	/* In micro seconds. How much time to allow before DPC processing is
	 * called back via interrupt (to prevent DPC queue drain starving).
	 * Typically 1000s of micro seconds (example only, not granularity). */
	u32 dpc_promote_time;

	/* Must be set to take these new values. */
	u32 is_valid;

	/* Max number of WIs to process per call. A large value may keep CS
	 * idle. */
	u32 max_num_work_items;

	u32 reserved[19];
} __packed;

/* GuC MMIO reg state struct */

#define GUC_REGSET_FLAGS_NONE		0x0
#define GUC_REGSET_POWERCYCLE		0x1
#define GUC_REGSET_MASKED		0x2
#define GUC_REGSET_ENGINERESET		0x4
#define GUC_REGSET_SAVE_DEFAULT_VALUE	0x8
#define GUC_REGSET_SAVE_CURRENT_VALUE	0x10

#define GUC_REGSET_MAX_REGISTERS	25
#define GUC_MMIO_WHITE_LIST_MAX		12
#define GUC_S3_SAVE_SPACE_PAGES		10

struct guc_mmio_regset {
	struct __packed {
		u32 offset;
		u32 value;
		u32 flags;
	} registers[GUC_REGSET_MAX_REGISTERS];

	u32 values_valid;
	u32 number_of_registers;
} __packed;

struct guc_mmio_reg_state {
	struct guc_mmio_regset global_reg;
	struct guc_mmio_regset engine_reg[GUC_MAX_ENGINES_NUM];

	/* MMIO registers that are set as non privileged */
	struct __packed {
		u32 mmio_start;
		u32 offsets[GUC_MMIO_WHITE_LIST_MAX];
		u32 count;
	} mmio_white_list[GUC_MAX_ENGINES_NUM];
} __packed;

/* GuC Additional Data Struct */

struct guc_ads {
	u32 reg_state_addr;
	u32 reg_state_buffer;
	u32 golden_context_lrca;
	u32 scheduler_policies;
	u32 reserved0[3];
	u32 eng_state_size[GUC_MAX_ENGINES_NUM];
	u32 reserved2[4];
} __packed;

/* GuC logging structures */

enum guc_log_buffer_type {
	GUC_ISR_LOG_BUFFER,
	GUC_DPC_LOG_BUFFER,
	GUC_CRASH_DUMP_LOG_BUFFER,
	GUC_MAX_LOG_BUFFER
};

/**
 * DOC: GuC Log buffer Layout
 *
 * Page0  +-------------------------------+
 *        |   ISR state header (32 bytes) |
 *        |      DPC state header         |
 *        |   Crash dump state header     |
 * Page1  +-------------------------------+
 *        |           ISR logs            |
 * Page9  +-------------------------------+
 *        |           DPC logs            |
 * Page17 +-------------------------------+
 *        |         Crash Dump logs       |
 *        +-------------------------------+
 *
 * Below state structure is used for coordination of retrieval of GuC firmware
 * logs. Separate state is maintained for each log buffer type.
 * read_ptr points to the location where i915 read last in log buffer and
 * is read only for GuC firmware. write_ptr is incremented by GuC with number
 * of bytes written for each log entry and is read only for i915.
 * When any type of log buffer becomes half full, GuC sends a flush interrupt.
 * GuC firmware expects that while it is writing to 2nd half of the buffer,
 * first half would get consumed by Host and then get a flush completed
 * acknowledgment from Host, so that it does not end up doing any overwrite
 * causing loss of logs. So when buffer gets half filled & i915 has requested
 * for interrupt, GuC will set flush_to_file field, set the sampled_write_ptr
 * to the value of write_ptr and raise the interrupt.
 * On receiving the interrupt i915 should read the buffer, clear flush_to_file
 * field and also update read_ptr with the value of sample_write_ptr, before
 * sending an acknowledgment to GuC. marker & version fields are for internal
 * usage of GuC and opaque to i915. buffer_full_cnt field is incremented every
 * time GuC detects the log buffer overflow.
 */
struct guc_log_buffer_state {
	u32 marker[2];
	u32 read_ptr;
	u32 write_ptr;
	u32 size;
	u32 sampled_write_ptr;
	union {
		struct {
			u32 flush_to_file:1;
			u32 buffer_full_cnt:4;
			u32 reserved:27;
		};
		u32 flags;
	};
	u32 version;
} __packed;

union guc_log_control {
	struct {
		u32 logging_enabled:1;
		u32 reserved1:3;
		u32 verbosity:4;
		u32 reserved2:24;
	};
	u32 value;
} __packed;

/* GuC Shared Context Data Struct */
struct guc_shared_ctx_data {
	u32 addr_of_last_preempted_data_low;
	u32 addr_of_last_preempted_data_high;
	u32 addr_of_last_preempted_data_high_tmp;
	u32 padding;
	u32 is_mapped_to_proxy;
	u32 proxy_ctx_id;
	u32 engine_reset_ctx_id;
	u32 media_reset_count;
	u32 reserved[8];
	u32 uk_last_ctx_switch_reason;
	u32 was_reset;
	u32 lrca_gpu_addr;
	u32 execlist_ctx;
	u32 reserved1[32];
} __packed;

/* This Action will be programmed in C180 - SOFT_SCRATCH_O_REG */
enum intel_guc_action {
	INTEL_GUC_ACTION_DEFAULT = 0x0,
	INTEL_GUC_ACTION_REQUEST_PREEMPTION = 0x2,
	INTEL_GUC_ACTION_REQUEST_ENGINE_RESET = 0x3,
	INTEL_GUC_ACTION_SAMPLE_FORCEWAKE = 0x6,
	INTEL_GUC_ACTION_ALLOCATE_DOORBELL = 0x10,
	INTEL_GUC_ACTION_DEALLOCATE_DOORBELL = 0x20,
	INTEL_GUC_ACTION_LOG_BUFFER_FILE_FLUSH_COMPLETE = 0x30,
	INTEL_GUC_ACTION_FORCE_LOG_BUFFER_FLUSH = 0x302,
	INTEL_GUC_ACTION_ENTER_S_STATE = 0x501,
	INTEL_GUC_ACTION_EXIT_S_STATE = 0x502,
	INTEL_GUC_ACTION_SLPC_REQUEST = 0x3003,
	INTEL_GUC_ACTION_AUTHENTICATE_HUC = 0x4000,
	INTEL_GUC_ACTION_UK_LOG_ENABLE_LOGGING = 0x0E000,
	INTEL_GUC_ACTION_LIMIT
};

enum intel_guc_preempt_options {
	INTEL_GUC_PREEMPT_OPTION_IMMEDIATE = 0x1,
	INTEL_GUC_PREEMPT_OPTION_DROP_WORK_Q = 0x4,
	INTEL_GUC_PREEMPT_OPTION_DROP_SUBMIT_Q = 0x8,
};

/*
 * The GuC sends its response to a command by overwriting the
 * command in SS0. The response is distinguishable from a command
 * by the fact that all the MASK bits are set. The remaining bits
 * give more detail.
 */
#define	INTEL_GUC_RECV_MASK	((u32)0xF0000000)
#define	INTEL_GUC_RECV_IS_RESPONSE(x)	((u32)(x) >= INTEL_GUC_RECV_MASK)
#define	INTEL_GUC_RECV_STATUS(x)	(INTEL_GUC_RECV_MASK | (x))

/* GUC will return status back to SOFT_SCRATCH_O_REG */
enum intel_guc_status {
	INTEL_GUC_STATUS_SUCCESS = INTEL_GUC_RECV_STATUS(0x0),
	INTEL_GUC_STATUS_ALLOCATE_DOORBELL_FAIL = INTEL_GUC_RECV_STATUS(0x10),
	INTEL_GUC_STATUS_DEALLOCATE_DOORBELL_FAIL = INTEL_GUC_RECV_STATUS(0x20),
	INTEL_GUC_STATUS_GENERIC_FAIL = INTEL_GUC_RECV_STATUS(0x0000F000)
};

/* This action will be programmed in C1BC - SOFT_SCRATCH_15_REG */
enum intel_guc_recv_message {
	INTEL_GUC_RECV_MSG_CRASH_DUMP_POSTED = BIT(1),
	INTEL_GUC_RECV_MSG_FLUSH_LOG_BUFFER = BIT(3)
};

#endif
