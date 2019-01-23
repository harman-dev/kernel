/*COPYRIGHT**
    Copyright (C) 2005-2017 Intel Corporation.  All Rights Reserved.

    This file is part of SEP Development Kit

    SEP Development Kit is free software; you can redistribute it
    and/or modify it under the terms of the GNU General Public License
    version 2 as published by the Free Software Foundation.

    SEP Development Kit is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SEP Development Kit; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

    As a special exception, you may use this file as part of a free software
    library without restriction.  Specifically, if other files instantiate
    templates or use macros or inline functions from this file, or you compile
    this file and link it with other files to produce an executable, this
    file does not by itself cause the resulting executable to be covered by
    the GNU General Public License.  This exception does not however
    invalidate any other reasons why the executable file might be covered by
    the GNU General Public License.
**COPYRIGHT*/

#include "lwpmudrv_defines.h"
#include <linux/version.h>
#include <linux/percpu.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <asm/segment.h>
#include <asm/page.h>

#include "lwpmudrv_types.h"
#include "rise_errors.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"
#include "lwpmudrv.h"
#include "control.h"
#include "core2.h"
#include "utility.h"
#include "output.h"
#include "ecb_iterators.h"
#include "pebs.h"

static PEBS_DISPATCH  pebs_dispatch           = NULL;
static PVOID          pebs_global_memory      = NULL;
static size_t         pebs_global_memory_size = 0;
static U32            pebs_record_num         = 0;
static U32            pebs_record_size        = 0;

extern DRV_BOOL       multi_pebs_enabled;

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID pebs_Corei7_Initialize_Threshold (dts, pebs_record_size)
 *
 * @brief       The nehalem specific initialization
 *
 * @param       dts  - dts description
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 */
static VOID
pebs_Corei7_Initialize_Threshold (
    DTS_BUFFER_EXT   dts
)
{
    DTS_BUFFER_EXT_pebs_threshold(dts)  = DTS_BUFFER_EXT_pebs_base(dts) + (pebs_record_size * pebs_record_num);

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID pebs_Corei7_Overflow ()
 *
 * @brief       The Nehalem specific overflow check
 *
 * @param       this_cpu        - cpu id
 *              overflow_status - overflow status
 *              rec_index       - record index
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *    Check the global overflow field of the buffer descriptor.
 *    Precise events can be allocated on any of the 4 general purpose
 *    registers.
 */
static U64
pebs_Corei7_Overflow (
    S32  this_cpu,
    U64  overflow_status,
    U32  rec_index
)
{
    DTS_BUFFER_EXT   dtes     = CPU_STATE_dts_buffer(&pcb[this_cpu]);
    S8              *pebs_base, *pebs_index, *pebs_ptr;
    PEBS_REC_EXT     pb;

    if (dtes) {
        pebs_base = (S8 *)(UIOP)DTS_BUFFER_EXT_pebs_base(dtes);
        pebs_index = (S8 *)(UIOP)DTS_BUFFER_EXT_pebs_index(dtes);
        pebs_ptr = (S8 *)((UIOP)DTS_BUFFER_EXT_pebs_base(dtes) + (rec_index * pebs_record_size));
        if (pebs_base != pebs_index && pebs_ptr < pebs_index) {
            pb = (PEBS_REC_EXT)pebs_ptr;
            overflow_status |= PEBS_REC_EXT_glob_perf_overflow(pb);
        }
    }

    return overflow_status;
}


/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID pebs_Core2_Initialize_Threshold (dts, pebs_record_size)
 *
 * @brief       The Core2 specific initialization
 *
 * @param       dts - dts description
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 */
static VOID
pebs_Core2_Initialize_Threshold (
    DTS_BUFFER_EXT   dts
)
{
    DTS_BUFFER_EXT_pebs_threshold(dts)  = DTS_BUFFER_EXT_pebs_base(dts);

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID pebs_Core2_Overflow (dts, pebs_record_size)
 *
 * @brief       The Core2 specific overflow check
 *
 * @param       this_cpu        - cpu id
 *              overflow_status - overflow status
 *              rec_index       - record index
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *    Check the base and the index fields of the circular buffer, if they are
 *    not the same, then a precise event has overflowed.  Precise events are
 *    allocated only on register#0.
 */
static U64
pebs_Core2_Overflow (
    S32  this_cpu,
    U64  overflow_status,
    U32  rec_index
)
{
    DTS_BUFFER_EXT   dtes     = CPU_STATE_dts_buffer(&pcb[this_cpu]);
    U8               status   = FALSE;

    if (!dtes) {
        return overflow_status;
    }
    status = (U8)((dtes) && (DTS_BUFFER_EXT_pebs_index(dtes) != DTS_BUFFER_EXT_pebs_base(dtes)));
    if (status) {
        // Merom allows only for general purpose register 0 to be precise capable
        overflow_status  |= 0x1;
    }

    return overflow_status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID pebs_Modify_IP (sample, is_64bit_addr)
 *
 * @brief       Change the IP field in the sample to that in the PEBS record
 *
 * @param       sample        - sample buffer
 * @param       is_64bit_addr - are we in a 64 bit module
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
static VOID
pebs_Modify_IP (
    void        *sample,
    DRV_BOOL     is_64bit_addr,
    U32          rec_index
)
{
    SampleRecordPC  *psamp = sample;
    DTS_BUFFER_EXT   dtes  = CPU_STATE_dts_buffer(&pcb[CONTROL_THIS_CPU()]);
    S8              *pebs_ptr;
    if (dtes && psamp) {
        S8   *pebs_base  = (S8 *)(UIOP)DTS_BUFFER_EXT_pebs_base(dtes);
        S8   *pebs_index = (S8 *)(UIOP)DTS_BUFFER_EXT_pebs_index(dtes);
        SEP_PRINT_DEBUG("In PEBS Fill Buffer: cpu %d\n", CONTROL_THIS_CPU());
        pebs_ptr = (S8 *)((UIOP)DTS_BUFFER_EXT_pebs_base(dtes) + (rec_index * pebs_record_size));
        if (pebs_base != pebs_index && pebs_ptr < pebs_index) {
            PEBS_REC_EXT  pb = (PEBS_REC_EXT)pebs_ptr;
            if (is_64bit_addr) {
                SAMPLE_RECORD_iip(psamp)    = PEBS_REC_EXT_linear_ip(pb);
                SAMPLE_RECORD_ipsr(psamp)   = PEBS_REC_EXT_r_flags(pb);
            }
            else {
                SAMPLE_RECORD_eip(psamp)    = PEBS_REC_EXT_linear_ip(pb) & 0xFFFFFFFF;
                SAMPLE_RECORD_eflags(psamp) = PEBS_REC_EXT_r_flags(pb) & 0xFFFFFFFF;
            }
        }
    }

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID pebs_Modify_IP_With_Eventing_IP (sample, is_64bit_addr)
 *
 * @brief       Change the IP field in the sample to that in the PEBS record
 *
 * @param       sample        - sample buffer
 * @param       is_64bit_addr - are we in a 64 bit module
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
static VOID
pebs_Modify_IP_With_Eventing_IP (
    void        *sample,
    DRV_BOOL     is_64bit_addr,
    U32          rec_index
)
{
    SampleRecordPC  *psamp = sample;
    DTS_BUFFER_EXT   dtes  = CPU_STATE_dts_buffer(&pcb[CONTROL_THIS_CPU()]);
    S8              *pebs_ptr;
    PEBS_REC_EXT1    pb    = NULL;

    if (dtes && psamp) {
        S8   *pebs_base  = (S8 *)(UIOP)DTS_BUFFER_EXT_pebs_base(dtes);
        S8   *pebs_index = (S8 *)(UIOP)DTS_BUFFER_EXT_pebs_index(dtes);
        SEP_PRINT_DEBUG("In PEBS Fill Buffer: cpu %d\n", CONTROL_THIS_CPU());
        pebs_ptr = (S8 *)((UIOP)DTS_BUFFER_EXT_pebs_base(dtes) + (rec_index * pebs_record_size));
        if (pebs_base != pebs_index && pebs_ptr < pebs_index) {
            pb = (PEBS_REC_EXT1)pebs_ptr;
            if (is_64bit_addr) {
                SAMPLE_RECORD_iip(psamp)    = PEBS_REC_EXT1_eventing_ip(pb);
                SAMPLE_RECORD_ipsr(psamp)   = PEBS_REC_EXT1_r_flags(pb);
            }
            else {
                SAMPLE_RECORD_eip(psamp)    = PEBS_REC_EXT1_eventing_ip(pb) & 0xFFFFFFFF;
                SAMPLE_RECORD_eflags(psamp) = PEBS_REC_EXT1_r_flags(pb) & 0xFFFFFFFF;
            }
        }
    }

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID pebs_Modify_TSC (sample)
 *
 * @brief       Change the TSC field in the sample to that in the PEBS record
 *
 * @param       sample        - sample buffer
 *              rec_index     - record index
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
static VOID
pebs_Modify_TSC (
    void        *sample,
    U32          rec_index
)
{
    SampleRecordPC  *psamp = sample;
    DTS_BUFFER_EXT   dtes  = CPU_STATE_dts_buffer(&pcb[CONTROL_THIS_CPU()]);
    S8              *pebs_base, *pebs_index, *pebs_ptr;
    PEBS_REC_EXT2    pb;

    if (dtes && psamp) {
        pebs_base  = (S8 *)(UIOP)DTS_BUFFER_EXT_pebs_base(dtes);
        pebs_index = (S8 *)(UIOP)DTS_BUFFER_EXT_pebs_index(dtes);
        pebs_ptr = (S8 *)((UIOP)DTS_BUFFER_EXT_pebs_base(dtes) + (rec_index * pebs_record_size));
        SEP_PRINT_DEBUG("In PEBS Fill Buffer: cpu %d\n", CONTROL_THIS_CPU());
        if (pebs_base != pebs_index && pebs_ptr < pebs_index) {
            pb = (PEBS_REC_EXT2)pebs_ptr;
            SAMPLE_RECORD_tsc(psamp) = PEBS_REC_EXT2_tsc(pb);
        }
    }

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          U32 pebs_Get_Num_Records_Filled ()
 *
 * @brief       get number of PEBS records filled in PEBS buffer
 *
 * @param       NONE
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
static U32
pebs_Get_Num_Records_Filled (
    VOID
)
{
    U32              num = 0;
    DTS_BUFFER_EXT   dtes  = CPU_STATE_dts_buffer(&pcb[CONTROL_THIS_CPU()]);
    S8              *pebs_base, *pebs_index;

    if (dtes) {
        pebs_base  = (S8 *)(UIOP)DTS_BUFFER_EXT_pebs_base(dtes);
        pebs_index = (S8 *)(UIOP)DTS_BUFFER_EXT_pebs_index(dtes);
        if (pebs_base != pebs_index) {
            num = (U32)(pebs_index - pebs_base) / pebs_record_size;
        }
    }
    return num;
}

/*
 * Initialize the pebs micro dispatch tables
 */
PEBS_DISPATCH_NODE  core2_pebs =
{
     pebs_Core2_Initialize_Threshold,
     pebs_Core2_Overflow,
     pebs_Modify_IP,
     NULL,
     pebs_Get_Num_Records_Filled
};

PEBS_DISPATCH_NODE  core2p_pebs =
{
     pebs_Corei7_Initialize_Threshold,
     pebs_Core2_Overflow,
     pebs_Modify_IP,
     NULL,
     pebs_Get_Num_Records_Filled
};

PEBS_DISPATCH_NODE  corei7_pebs =
{
     pebs_Corei7_Initialize_Threshold,
     pebs_Corei7_Overflow,
     pebs_Modify_IP,
     NULL,
     pebs_Get_Num_Records_Filled
};

PEBS_DISPATCH_NODE  haswell_pebs =
{
     pebs_Corei7_Initialize_Threshold,
     pebs_Corei7_Overflow,
     pebs_Modify_IP_With_Eventing_IP,
     NULL,
     pebs_Get_Num_Records_Filled
};

PEBS_DISPATCH_NODE  perfver4_pebs =
{
     pebs_Corei7_Initialize_Threshold,
     pebs_Corei7_Overflow,
     pebs_Modify_IP_With_Eventing_IP,
     pebs_Modify_TSC,
     pebs_Get_Num_Records_Filled
};

#define PER_CORE_BUFFER_SIZE(record_size, record_num)  (sizeof(DTS_BUFFER_EXT_NODE) +  (record_num + 1) * (record_size) + 64)

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID* pebs_Alloc_DTS_Buffer (VOID)
 *
 * @brief       Allocate buffers used for latency and pebs sampling
 *
 * @param       NONE
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              Allocate the memory needed to hold the DTS and PEBS records buffer.
 *              This routine is called by a thread that corresponds to a single core
 */
static VOID*
pebs_Alloc_DTS_Buffer (
	VOID
)
{
    UIOP            pebs_base;
    U32             buffer_size;
    U32             dts_size;
    VOID           *dts_buffer;
    DTS_BUFFER_EXT  dts;
    int             this_cpu;

    /*
     * one PEBS record... need 2 records so that
     * threshold can be less than absolute max
     */
    preempt_disable();
    this_cpu = CONTROL_THIS_CPU();
    preempt_enable();
    dts_size = sizeof(DTS_BUFFER_EXT_NODE);

    /*
     * account for extra bytes to align PEBS base to cache line boundary
     */
    buffer_size = PER_CORE_BUFFER_SIZE(pebs_record_size, pebs_record_num);
    dts_buffer = (char *)pebs_global_memory + (this_cpu * buffer_size);
    if (!dts_buffer) {
        SEP_PRINT_ERROR("Failed to allocate space for DTS buffer.\n");
        return NULL;
    }

    pebs_base = (UIOP)(dts_buffer) + dts_size;

    //  Make 32 byte aligned
    if ((pebs_base & 0x000001F) != 0x0) {
        pebs_base = ALIGN_32(pebs_base);
    }

    /*
     * Program the DTES Buffer for Precise EBS.
     * Set PEBS buffer for one PEBS record
     */
    dts = (DTS_BUFFER_EXT)dts_buffer;

    DTS_BUFFER_EXT_base(dts)            = 0;
    DTS_BUFFER_EXT_index(dts)           = 0;
    DTS_BUFFER_EXT_max(dts)             = 0;
    DTS_BUFFER_EXT_threshold(dts)       = 0;
    DTS_BUFFER_EXT_pebs_base(dts)       = pebs_base;
    DTS_BUFFER_EXT_pebs_index(dts)      = pebs_base;
    DTS_BUFFER_EXT_pebs_max(dts)        = pebs_base + (pebs_record_num + 1) * pebs_record_size;

    pebs_dispatch->initialize_threshold(dts);

    SEP_PRINT_DEBUG("base --- %llx\n", DTS_BUFFER_EXT_pebs_base(dts));
    SEP_PRINT_DEBUG("index --- %llu\n", DTS_BUFFER_EXT_pebs_index(dts));
    SEP_PRINT_DEBUG("max --- %llu\n", DTS_BUFFER_EXT_pebs_max(dts));
    SEP_PRINT_DEBUG("threahold --- %llu\n", DTS_BUFFER_EXT_pebs_threshold(dts));
    SEP_PRINT_DEBUG("DTES buffer allocated for PEBS: %p\n", dts_buffer);

    return dts_buffer;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID* pebs_Allocate_Buffers (VOID *params)
 *
 * @brief       Allocate memory and set up MSRs in preparation for PEBS
 *
 * @param       NONE
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              Set up the DS area and program the DS_AREA msrs in preparation
 *              for a PEBS run.  Save away the old value in the DS_AREA.
 *              This routine is called via the parallel thread call.
 */
static VOID
pebs_Allocate_Buffers (
    VOID  *params
)
{
    U64         value;
    CPU_STATE   pcpu = &pcb[CONTROL_THIS_CPU()];

    SYS_Write_MSR(IA32_PEBS_ENABLE, 0LL);
    value = SYS_Read_MSR(IA32_MISC_ENABLE);
    if ((value & 0x80) && !(value & 0x1000)) {
        CPU_STATE_old_dts_buffer(pcpu) = (PVOID)(UIOP)SYS_Read_MSR(IA32_DS_AREA);
        CPU_STATE_dts_buffer(pcpu)     = pebs_Alloc_DTS_Buffer();
        SEP_PRINT_DEBUG("Old dts buffer - %p\n", CPU_STATE_old_dts_buffer(pcpu));
        SEP_PRINT_DEBUG("New dts buffer - %p\n", CPU_STATE_dts_buffer(pcpu));
        SYS_Write_MSR(IA32_DS_AREA, (U64)(UIOP)CPU_STATE_dts_buffer(pcpu));
    }

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID pebs_Dellocate_Buffers (VOID *params)
 *
 * @brief       Clean up PEBS buffers and restore older values into the DS_AREA
 *
 * @param       NONE
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              Clean up the DS area and all restore state prior to the sampling run
 *              This routine is called via the parallel thread call.
 */
static VOID
pebs_Deallocate_Buffers (
    VOID  *params
)
{
    CPU_STATE   pcpu = &pcb[CONTROL_THIS_CPU()];

    SEP_PRINT_DEBUG("Entered deallocate buffers\n");
    SYS_Write_MSR(IA32_DS_AREA, (U64)(UIOP)CPU_STATE_old_dts_buffer(pcpu));

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          U64 PEBS_Overflowed (this_cpu, overflow_status)
 *
 * @brief       Figure out if the PEBS event caused an overflow
 *
 * @param       this_cpu        -- the current cpu
 *              overflow_status -- current value of the global overflow status
 *
 * @return      updated overflow_status
 *
 * <I>Special Notes:</I>
 *              Figure out if the PEBS area has data that need to be transferred
 *              to the output sample.
 *              Update the overflow_status that is passed and return this value.
 *              The overflow_status defines the events/status to be read
 */
extern U64
PEBS_Overflowed (
    S32  this_cpu,
    U64  overflow_status,
    U32  rec_index
)
{
    return pebs_dispatch->overflow(this_cpu, overflow_status, rec_index);
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID PEBS_Reset_Index (this_cpu)
 *
 * @brief       Reset the PEBS index pointer
 *
 * @param       this_cpu        -- the current cpu
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              reset index to next PEBS record to base of buffer
 */
extern VOID
PEBS_Reset_Index (
    S32    this_cpu
)
{
    DTS_BUFFER_EXT   dtes = CPU_STATE_dts_buffer(&pcb[this_cpu]);

    if (dtes) {
        SEP_PRINT_DEBUG("PEBS Reset Index: %d\n", this_cpu);
        DTS_BUFFER_EXT_pebs_index(dtes) = DTS_BUFFER_EXT_pebs_base(dtes);
    }

    return;
}

extern DRV_CONFIG   pcfg;
extern U32 pmi_Get_CSD (U32, U32*, U32*);
#define EFLAGS_V86_MASK       0x00020000L

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID PEBS_Flush_Buffer (VOID * param)
 *
 * @brief       generate sampling records from PEBS records in PEBS buffer
 *
 * @param       param        -- not used
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 */
extern VOID
PEBS_Flush_Buffer(
    VOID * param
)
{
    U32 i, this_cpu, index, desc_id;
    U64              pebs_overflow_status = 0;
    ECB              pecb;
    CPU_STATE        pcpu;
    EVENT_DESC       evt_desc;
    BUFFER_DESC      bd;
    SampleRecordPC  *psamp_pebs;
    U32              is_64bit_addr    = FALSE;
    U32              u32PebsRecordNumFilled;
#if defined(DRV_IA32)
    U32              seg_cs;
    U32              csdlo;
    U32              csdhi;
#endif

    if (!multi_pebs_enabled) {
        SEP_PRINT_DEBUG("PEBS_Flush_Buffer is not supported\n");
        return;
    }

    this_cpu = CONTROL_THIS_CPU();
    pcpu     = &pcb[this_cpu];
    bd       = &cpu_buf[this_cpu];

    u32PebsRecordNumFilled = PEBS_Get_Num_Records_Filled();
    for (i = 0; i < u32PebsRecordNumFilled; i++) {
        pebs_overflow_status = PEBS_Overflowed(this_cpu, 0, i);
        SEP_PRINT_DEBUG("PEBS_Flush_Buffer: pebs_overflow_status = 0x%llx, i=%d\n", pebs_overflow_status, i);

        pecb = PMU_register_data[CPU_STATE_current_group(pcpu)];
        FOR_EACH_DATA_REG(pecb, j) {
            if (!ECB_entries_is_gp_reg_get(pecb, j) || !ECB_entries_precise_get(pecb, j)) {
                continue;
            }
            index = ECB_entries_reg_id(pecb, j) - IA32_PMC0;
            if (pebs_overflow_status & ((U64)1 << index)) {
                desc_id  = ECB_entries_event_id_index(pecb, j);
                evt_desc = desc_data[desc_id];
                SEP_PRINT_DEBUG("PEBS_Flush_Buffer: event_id_index=%u, desc_id=%u\n", ECB_entries_event_id_index(pecb, j), desc_id);
                psamp_pebs = (SampleRecordPC *)OUTPUT_Reserve_Buffer_Space(bd, EVENT_DESC_sample_size(evt_desc), (NMI_mode)? TRUE:FALSE);
                if (!psamp_pebs) {
                    SEP_PRINT_ERROR("Could not generate samples from PEBS records\n");
                    continue;
                }

                CPU_STATE_num_samples(&pcb[this_cpu])      += 1;
                SAMPLE_RECORD_descriptor_id(psamp_pebs)     = desc_id;
                SAMPLE_RECORD_event_index(psamp_pebs)       = ECB_entries_event_id_index(pecb, j);
                SAMPLE_RECORD_pid_rec_index(psamp_pebs)     = (U32)-1;
                SAMPLE_RECORD_pid_rec_index_raw(psamp_pebs) = 1;
                SAMPLE_RECORD_tid(psamp_pebs)               = (U32)-1;
                SAMPLE_RECORD_cpu_num(psamp_pebs)           = (U16) this_cpu;
                SAMPLE_RECORD_osid(psamp_pebs)              = 0;

#if defined (DRV_IA32)
                PEBS_Modify_IP((S8 *)psamp_pebs, is_64bit_addr, i);
                SAMPLE_RECORD_cs(psamp_pebs)                 = __KERNEL_CS;
                if (SAMPLE_RECORD_eflags(psamp_pebs) & EFLAGS_V86_MASK) {
                    csdlo = 0;
                    csdhi = 0;
                }
                else {
                    seg_cs = SAMPLE_RECORD_cs(psamp_pebs);
                    SYS_Get_CSD(seg_cs, &csdlo, &csdhi);
                }
                SAMPLE_RECORD_csd(psamp_pebs).u1.lowWord  = csdlo;
                SAMPLE_RECORD_csd(psamp_pebs).u2.highWord = csdhi;
#elif defined (DRV_EM64T)
                SAMPLE_RECORD_cs(psamp_pebs)                = __KERNEL_CS;
                pmi_Get_CSD(SAMPLE_RECORD_cs(psamp_pebs),
                            &SAMPLE_RECORD_csd(psamp_pebs).u1.lowWord,
                            &SAMPLE_RECORD_csd(psamp_pebs).u2.highWord);
                is_64bit_addr = (SAMPLE_RECORD_csd(psamp_pebs).u2.s2.reserved_0 == 1);
                if (is_64bit_addr) {
                    SAMPLE_RECORD_ia64_pc(psamp_pebs)       = TRUE;
                }
                else {
                    SAMPLE_RECORD_ia64_pc(psamp_pebs)       = FALSE;

                    SEP_PRINT_DEBUG("SAMPLE_RECORD_eip(psamp_pebs) 0x%x\n", SAMPLE_RECORD_eip(psamp_pebs));
                    SEP_PRINT_DEBUG("SAMPLE_RECORD_eflags(psamp_pebs) %x\n", SAMPLE_RECORD_eflags(psamp_pebs));
                }
#endif
                if (EVENT_DESC_pebs_offset(evt_desc)
                    || EVENT_DESC_latency_offset_in_sample(evt_desc)) {
                    PEBS_Fill_Buffer((S8 *)psamp_pebs, evt_desc, DRV_CONFIG_virt_phys_translation(pcfg), i);
                }
                PEBS_Modify_IP((S8 *)psamp_pebs, is_64bit_addr, i);
                PEBS_Modify_TSC((S8 *)psamp_pebs, i);
            }
        } END_FOR_EACH_DATA_REG;
    }
    PEBS_Reset_Index(this_cpu);
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID PEBS_Reset_Counter (this_cpu, index, value)
 *
 * @brief       set reset value for PMC after overflow
 *
 * @param       this_cpu        -- the current cpu
 *              index           -- PMC register index
 *              value           -- reset value for PMC after overflow
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 */
extern VOID
PEBS_Reset_Counter (
    S32    this_cpu,
    U32    index,
    U64    value
)
{
    DTS_BUFFER_EXT dts = CPU_STATE_dts_buffer(&pcb[this_cpu]);
    if (dts) {
        SEP_PRINT_DEBUG("PEBS Reset Counter: cpu %d, index=%u, value=%llx\n",
            this_cpu, index, value);
        switch(index) {
            case 0:
                DTS_BUFFER_EXT_counter_reset0(dts)  = value;
                break;
            case 1:
                DTS_BUFFER_EXT_counter_reset1(dts)  = value;
                break;
            case 2:
                DTS_BUFFER_EXT_counter_reset2(dts)  = value;
                break;
            case 3:
                DTS_BUFFER_EXT_counter_reset3(dts)  = value;
                break;
        }
    }
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID PEBS_Modify_IP (sample, is_64bit_addr)
 *
 * @brief       Change the IP field in the sample to that in the PEBS record
 *
 * @param       sample        - sample buffer
 * @param       is_64bit_addr - are we in a 64 bit module
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
extern VOID
PEBS_Modify_IP (
    void        *sample,
    DRV_BOOL     is_64bit_addr,
    U32          rec_index
)
{
    pebs_dispatch->modify_ip(sample, is_64bit_addr, rec_index);
    return;
}


/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID PEBS_Modify_TSC (sample)
 *
 * @brief       Change the TSC field in the sample to that in the PEBS record
 *
 * @param       sample        - sample buffer
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
extern VOID
PEBS_Modify_TSC (
    void        *sample,
    U32          rec_index
)
{
    if (pebs_dispatch->modify_tsc != NULL) {
        pebs_dispatch->modify_tsc(sample, rec_index);
    }
    return;
}

extern U32
PEBS_Get_Num_Records_Filled (
    VOID
)
{
    U32 num = 0;

    if (pebs_dispatch->get_num_records_filled != NULL) {
        num = pebs_dispatch->get_num_records_filled();
        SEP_PRINT_DEBUG("PEBS_Get_Num_Records_Filled: num=%u\n", num);
    }
    return num;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID PEBS_Fill_Buffer (S8 *buffer, EVENT_CONFIG ec)
 *
 * @brief       Fill the buffer with the pebs data
 *
 * @param       buffer  -  area to write the data into
 *              ec      -  current event config
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
extern VOID
PEBS_Fill_Buffer (
    S8           *buffer,
    EVENT_DESC    evt_desc,
    DRV_BOOL      virt_phys_translation_ena,
    U32           rec_index
)
{
    DTS_BUFFER_EXT   dtes       = CPU_STATE_dts_buffer(&pcb[CONTROL_THIS_CPU()]);
    LATENCY_INFO_NODE   latency_info  = {0};
    PEBS_REC_EXT1    pebs_base_ext1;
    PEBS_REC_EXT2    pebs_base_ext2;
#if defined(DRV_EM64T) && LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
    U64                 lin_addr;
    U64                 offset;
    struct page        *page;
#endif

    if (dtes) {
        S8   *pebs_base  = (S8 *)(UIOP)DTS_BUFFER_EXT_pebs_base(dtes);
        S8   *pebs_index = (S8 *)(UIOP)DTS_BUFFER_EXT_pebs_index(dtes);
        S8   *pebs_ptr   = (S8 *)((UIOP)DTS_BUFFER_EXT_pebs_base(dtes) + (rec_index *  pebs_record_size));

        SEP_PRINT_DEBUG("In PEBS Fill Buffer: cpu %d\n", CONTROL_THIS_CPU());
        if (pebs_base != pebs_index && pebs_ptr < pebs_index){
            pebs_base = pebs_ptr;
            if (EVENT_DESC_pebs_offset(evt_desc)) {
                SEP_PRINT_DEBUG("PEBS buffer has data available\n");
                memcpy(buffer + EVENT_DESC_pebs_offset(evt_desc),
                       pebs_base,
                       EVENT_DESC_pebs_size(evt_desc));
            }
            if (EVENT_DESC_eventing_ip_offset(evt_desc)) {
                pebs_base_ext1 = (PEBS_REC_EXT1)pebs_base;
                *(U64*)(buffer + EVENT_DESC_eventing_ip_offset(evt_desc)) = PEBS_REC_EXT1_eventing_ip(pebs_base_ext1);
            }
            if (EVENT_DESC_hle_offset(evt_desc)) {
                pebs_base_ext1 = (PEBS_REC_EXT1)pebs_base;
                *(U64*)(buffer + EVENT_DESC_hle_offset(evt_desc)) = PEBS_REC_EXT1_hle_info(pebs_base_ext1);
            }
            if (EVENT_DESC_latency_offset_in_sample(evt_desc)) {
                pebs_base_ext1 = (PEBS_REC_EXT1)pebs_base;
                memcpy(&latency_info,
                        pebs_base + EVENT_DESC_latency_offset_in_pebs_record(evt_desc),
                        EVENT_DESC_latency_size_from_pebs_record(evt_desc));
                memcpy(&LATENCY_INFO_stack_pointer(&latency_info),
                       &PEBS_REC_EXT1_rsp(pebs_base_ext1),
                       sizeof(U64));
                LATENCY_INFO_phys_addr(&latency_info) = 0;
#if defined(DRV_EM64T) && LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
                if (virt_phys_translation_ena) {
                    lin_addr = (U64)LATENCY_INFO_linear_address(&latency_info);
                    if (lin_addr != 0) {
                        offset = (U64)(lin_addr & 0x0FFF);
                        if (__virt_addr_valid(lin_addr)) {
                            LATENCY_INFO_phys_addr(&latency_info) = (U64)__pa(lin_addr);
                        }
                        else if (lin_addr < __PAGE_OFFSET) {
                            pagefault_disable();
                            if (__get_user_pages_fast(lin_addr, 1, 1, &page)) {
                                LATENCY_INFO_phys_addr(&latency_info) = (U64)page_to_phys(page) + offset;
                            }
                            pagefault_enable();
                        }
                    }
                }
#endif
                memcpy(buffer + EVENT_DESC_latency_offset_in_sample(evt_desc),
                       &latency_info,
                       sizeof(LATENCY_INFO_NODE) );
            }
            if (EVENT_DESC_pebs_tsc_offset(evt_desc)) {
                pebs_base_ext2 = (PEBS_REC_EXT2)pebs_base;
                *(U64*)(buffer + EVENT_DESC_pebs_tsc_offset(evt_desc)) = PEBS_REC_EXT2_tsc(pebs_base_ext2);
            }
        }
    }

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID PEBS_Initialize (DRV_CONFIG pcfg)
 *
 * @brief       Initialize the pebs buffers
 *
 * @param       pcfg  -  Driver Configuration
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              If the user is asking for PEBS information.  Allocate the DS area
 */
extern VOID
PEBS_Initialize (
    DRV_CONFIG  pcfg
)
{

    if (DRV_CONFIG_pebs_mode(pcfg)) {
        pebs_record_num = DRV_CONFIG_pebs_record_num(pcfg);
        if (!pebs_record_num) {
            pebs_record_num = 1;
        }
        switch (DRV_CONFIG_pebs_mode(pcfg)) {
            case 1:
                SEP_PRINT_DEBUG("Set up the Core2 dispatch table\n");
                pebs_dispatch = &core2_pebs;
                pebs_record_size = sizeof(PEBS_REC_NODE);
                break;
            case 2:
                SEP_PRINT_DEBUG("Set up the Nehalem dispatch\n");
                pebs_dispatch = &corei7_pebs;
                pebs_record_size = sizeof(PEBS_REC_EXT_NODE);
                break;
            case 3:
                SEP_PRINT_DEBUG("Set up the Core2 (PNR) dispatch table\n");
                pebs_dispatch = &core2p_pebs;
                pebs_record_size = sizeof(PEBS_REC_NODE);
                break;
            case 4:
                SEP_PRINT_DEBUG("Set up the Haswell dispatch table\n");
                pebs_dispatch = &haswell_pebs;
                pebs_record_size = sizeof(PEBS_REC_EXT1_NODE);
                break;
            case 5:
                SEP_PRINT_DEBUG("Set up the Perf version4 dispatch table\n");
                pebs_dispatch = &perfver4_pebs;
                pebs_record_size = sizeof(PEBS_REC_EXT2_NODE);
                break;
            default:
                SEP_PRINT_DEBUG("Unknown PEBS type. Will not collect PEBS information\n");
                break;
        }
        if (pebs_dispatch) {
            DRV_CONFIG_pebs_record_num(pcfg) = pebs_record_num;
            pebs_global_memory_size = GLOBAL_STATE_num_cpus(driver_state) * PER_CORE_BUFFER_SIZE(pebs_record_size, pebs_record_num);
            pebs_global_memory = (PVOID)CONTROL_Allocate_KMemory(pebs_global_memory_size);
            CONTROL_Invoke_Parallel(pebs_Allocate_Buffers, (VOID *)NULL);
        }
    }

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID PEBS_Destroy (DRV_CONFIG pcfg)
 *
 * @brief       Clean up the pebs related buffers
 *
 * @param       pcfg  -  Driver Configuration
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *             Deallocated the DS area used for PEBS capture
 */
extern VOID
PEBS_Destroy (
    DRV_CONFIG  pcfg
)
{
    if (DRV_CONFIG_pebs_mode(pcfg)) {
        CONTROL_Invoke_Parallel(pebs_Deallocate_Buffers, (VOID *)(size_t)0);
        pebs_global_memory = CONTROL_Free_Memory(pebs_global_memory);
        pebs_global_memory_size = 0;
    }

    return;
}
