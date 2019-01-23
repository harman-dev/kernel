/*COPYRIGHT**
    Copyright (C) 2012-2017 Intel Corporation.  All Rights Reserved.

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
#include <linux/fs.h>

#include "lwpmudrv_types.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"

#include "lwpmudrv.h"
#include "utility.h"
#include "control.h"
#include "output.h"
#include "valleyview_sochap.h"
#include "inc/ecb_iterators.h"
#include "inc/pci.h"

#if defined(PCI_HELPERS_API)
#include <asm/intel_mid_pcihelpers.h>
#endif

extern U64           *read_counter_info;
static U64           *uncore_current_data = NULL;
static U64           *uncore_to_read_data = NULL;

extern VOID SOCPERF_Read_Data2(PVOID data_buffer);



/*!
 * @fn         static VOID valleyview_VISA_Initialize(PVOID)
 *
 * @brief      Initialize any registers or addresses
 *
 * @param      param
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID
valleyview_VISA_Initialize (
    VOID  *param
)
{
    // Allocate memory for reading GMCH counter values + the group id
    if (!uncore_current_data) {
        uncore_current_data = CONTROL_Allocate_Memory((VLV_CHAP_MAX_COUNTERS+1)*sizeof(U64));
        if (!uncore_current_data) {
            return;
        }
    }
    if (!uncore_to_read_data) {
        uncore_to_read_data = CONTROL_Allocate_Memory((VLV_CHAP_MAX_COUNTERS+1)*sizeof(U64));
        if (!uncore_to_read_data) {
            return;
        }
    }

    return;
}


/*!
 * @fn         static VOID valleyview_VISA_Enable_PMU(PVOID)
 *
 * @brief      Start counting
 *
 * @param      param - device index
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID
valleyview_VISA_Enable_PMU (
    PVOID  param
)
{
    U32 this_cpu  = CONTROL_THIS_CPU();
    CPU_STATE pcpu  = &pcb[this_cpu];

    if (!CPU_STATE_system_master(pcpu)) {
        return;
    }

    SEP_PRINT_DEBUG("Starting the counters...\n");
    if (uncore_current_data) {
        memset(uncore_current_data, 0, (VLV_CHAP_MAX_COUNTERS+1)*sizeof(U64));
    }
    if (uncore_to_read_data) {
        memset(uncore_to_read_data, 0, (VLV_CHAP_MAX_COUNTERS+1)*sizeof(U64));
    }

    return;
}


/*!
 * @fn         static VOID valleyview_VISA_Disable_PMU(PVOID)
 *
 * @brief      Unmap the virtual address when sampling/driver stops
 *
 * @param      param - device index
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID
valleyview_VISA_Disable_PMU (
    PVOID  param
)
{
    U32                   this_cpu  = CONTROL_THIS_CPU();
    CPU_STATE             pcpu      = &pcb[this_cpu];

    if (!CPU_STATE_system_master(pcpu)) {
        return;
    }
    SEP_PRINT_DEBUG("Stopping the counters...\n");
    if (GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_PREPARE_STOP) {
        uncore_current_data = CONTROL_Free_Memory(uncore_current_data);
        uncore_to_read_data = CONTROL_Free_Memory(uncore_to_read_data);
    }

    return;
}



/*!
 * @fn         static VOID valleyview_VISA_Clean_Up(PVOID)
 *
 * @brief      Reset any registers or addresses
 *
 * @param      param
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID
valleyview_VISA_Clean_Up (
    VOID   *param
)
{
    return;
}


/* ------------------------------------------------------------------------- */
/*!
 * @fn valleyview_VISA_Read_PMU_Data(param)
 *
 * @param    param    The device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore count data and store into the buffer param;
 *
 */
static VOID
valleyview_VISA_Read_PMU_Data (
    PVOID  param
)
{
    S32                   j;
    U64                  *buffer       = read_counter_info;
    U32                   dev_idx      = *((U32*)param);
    U32                   start_index;
    DRV_CONFIG            pcfg_unc;
    U32                   data_reg     = 0;
    U32                   this_cpu     = CONTROL_THIS_CPU();
    CPU_STATE             pcpu         = &pcb[this_cpu];
    U32                   event_index  = 0;
    U32                   cur_grp      = LWPMU_DEVICE_cur_group(&devices[(dev_idx)]);
    U64                   counter_buffer[VLV_CHAP_MAX_COUNTERS+1];

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    pcfg_unc    = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);
    start_index = DRV_CONFIG_emon_unc_offset(pcfg_unc, cur_grp);

    SOCPERF_Read_Data2((void*)counter_buffer);

    FOR_EACH_PCI_REG_RAW(pecb, i, dev_idx) {
        if (ECB_entries_reg_type(pecb,i) == PMU_REG_EVENT_SELECT) {
            data_reg           = i + ECB_operations_register_len(pecb, PMU_OPERATION_WRITE);
            if (ECB_entries_reg_type(pecb,data_reg) == PMU_REG_DATA) {
                j = start_index + ECB_entries_group_index(pecb, data_reg) + ECB_entries_emon_event_id_index_local(pecb, data_reg);
                buffer[j] = counter_buffer[event_index+1];
                event_index++;
            }
        }

    } END_FOR_EACH_PCI_REG_RAW;

}


/* ------------------------------------------------------------------------- */
/*!
 * @fn valleyview_Trigger_Read()
 *
 * @param    None
 *
 * @return   None     No return needed
 *
 * @brief    Read the SoCHAP counters when timer is triggered
 *
 */
static VOID
valleyview_Trigger_Read (
    PVOID  param,
    U32    id
)
{
    U64  *data         = (U64*) param;
    U32   cur_grp      = LWPMU_DEVICE_cur_group(&devices[id]);
    ECB   pecb         = LWPMU_DEVICE_PMU_register_data(&devices[id])[cur_grp];

    // group id
    data = (U64*)((S8*)data + ECB_group_offset(pecb));
    SOCPERF_Read_Data2((void*)data);

    return;
}


/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  valleyview_visa_dispatch =
{
    .init                     = valleyview_VISA_Initialize,
    .fini                     = NULL,
    .write                    = NULL,
    .freeze                   = valleyview_VISA_Disable_PMU,
    .restart                  = valleyview_VISA_Enable_PMU,
    .read_data                = valleyview_VISA_Read_PMU_Data,
    .check_overflow           = NULL,
    .swap_group               = NULL,
    .read_lbrs                = NULL,
    .cleanup                  = valleyview_VISA_Clean_Up,
    .hw_errata                = NULL,
    .read_power               = NULL,
    .check_overflow_errata    = NULL,
    .read_counts              = NULL,
    .check_overflow_gp_errata = NULL,
    .read_ro                  = NULL,
    .platform_info            = NULL,
    .trigger_read             = valleyview_Trigger_Read,
    .scan_for_uncore          = NULL
};
