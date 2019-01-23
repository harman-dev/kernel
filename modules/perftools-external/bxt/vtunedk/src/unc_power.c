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
#include <linux/wait.h>
#include <linux/fs.h>

#include "lwpmudrv_types.h"
#include "rise_errors.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"

#include "lwpmudrv.h"
#include "utility.h"
#include "control.h"
#include "output.h"
#include "ecb_iterators.h"
#include "unc_common.h"

extern U64           *read_counter_info;
extern U64           *prev_counter_data;
static U64          **prev_val_per_thread;
static U64          **acc_per_thread;


/*!
 * @fn unc_power_Allocate(param)
 *
 * @param    param    device index
 *
 * @return   None     No return needed
 *
 * @brief    Allocate arrays required for reading counts
 */
static VOID
unc_power_Allocate (
    PVOID  param
)
{
    U32    id      = *((U32*)param);
    U32    cur_grp = LWPMU_DEVICE_cur_group(&devices[id]);
    ECB    pecb    = LWPMU_DEVICE_PMU_register_data(&devices[id])[cur_grp];
    U32    i;
    U32    j;

    if (!pecb) {
        return;
    }

    acc_per_thread = CONTROL_Allocate_Memory(GLOBAL_STATE_num_cpus(driver_state) * sizeof(U64 *));
    if (acc_per_thread == NULL) {
        SEP_PRINT_ERROR("Unable to allocate memory for acc_per_thread\n");
        return;
    }

    prev_val_per_thread = CONTROL_Allocate_Memory(GLOBAL_STATE_num_cpus(driver_state) * sizeof(U64 *));
    if (prev_val_per_thread == NULL) {
        SEP_PRINT_ERROR("Unable to allocate memory for prev_val_per_thread\n");
        return;
    }

    for (i = 0; i < (U32)GLOBAL_STATE_num_cpus(driver_state); i++) {
        acc_per_thread[i] = CONTROL_Allocate_Memory(ECB_num_events(pecb) * sizeof(U64));
        if (acc_per_thread[i] == NULL) {
            SEP_PRINT_ERROR("Unable to allocate memory for acc_per_thread\n");
            return;
        }

        prev_val_per_thread[i] = CONTROL_Allocate_Memory(ECB_num_events(pecb) * sizeof(U64));
        if (prev_val_per_thread[i] == NULL) {
            SEP_PRINT_ERROR("Unable to allocate memory for prev_val_per_thread\n");
            return;
        }

        // initialize all values to 0
        for (j = 0; j < ECB_num_events(pecb); j++) {
            acc_per_thread[i][j]      = 0LL;
            prev_val_per_thread[i][j] = 0LL;
        }
    }

    return;

}

/*!
 * @fn unc_power_Free(param)
 *
 * @param    param    device index
 *
 * @return   None     No return needed
 *
 * @brief    Free arrays required for reading counts
 */
static VOID
unc_power_Free (
    PVOID  param
)
{
    U32    i;

    if (acc_per_thread) {
        for (i = 0; i < (U32)GLOBAL_STATE_num_cpus(driver_state); i++) {
            acc_per_thread[i] = CONTROL_Free_Memory(acc_per_thread[i]);
        }
        acc_per_thread = CONTROL_Free_Memory(acc_per_thread);
    }

    if (prev_val_per_thread) {
        for (i = 0; i < (U32)GLOBAL_STATE_num_cpus(driver_state); i++) {
            prev_val_per_thread[i] = CONTROL_Free_Memory(prev_val_per_thread[i]);
        }
        prev_val_per_thread = CONTROL_Free_Memory(prev_val_per_thread);
    }

    return;

}

/*!
 * @fn unc_power_Read_Counts(param, id, mask)
 *
 * @param    param    pointer to sample buffer
 * @param    id       device index
 * @param    mask     The mask bits for value
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore count data and store into the buffer param
 */
static VOID
unc_power_Trigger_Read (
    PVOID  param,
    U32    id
)
{
    U64  *data       = (U64*) param;
    U32   cur_grp    = LWPMU_DEVICE_cur_group(&devices[id]);
    ECB   pecb       = LWPMU_DEVICE_PMU_register_data(&devices[id])[cur_grp];
    U32   this_cpu   = CONTROL_THIS_CPU();
    U32   index      = 0;
    U64   diff       = 0;
    U64   value;

    // Write GroupID
    data    = (U64*)((S8*)data + ECB_group_offset(pecb));
    *data   = cur_grp + 1;

    FOR_EACH_REG_UNC_OPERATION(pecb, id, idx, PMU_OPERATION_READ) {
        data  = (U64 *)((S8*)param + ECB_entries_counter_event_offset(pecb,idx));
        value = SYS_Read_MSR(ECB_entries_reg_id(pecb,idx));
        if (ECB_entries_max_bits(pecb,idx)) {
            value &= ECB_entries_max_bits(pecb,idx);
        }
        //check for overflow if not a static counter
        if (ECB_entries_counter_type(pecb,idx) == STATIC_COUNTER) {
            *data = value;
        }
        else {
            if (value < prev_val_per_thread[this_cpu][index]) {
                diff = ECB_entries_max_bits(pecb,idx) - prev_val_per_thread[this_cpu][index];
                diff += value;
            }
            else {
                diff = value - prev_val_per_thread[this_cpu][index];
            }
            acc_per_thread[this_cpu][index] += diff;
            prev_val_per_thread[this_cpu][index] = value;
            *data = acc_per_thread[this_cpu][index];
        }
        index++;
    } END_FOR_EACH_REG_UNC_OPERATION;

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn unc_power_Enable_PMU(param)
 *
 * @param    None
 *
 * @return   None
 *
 * @brief    Capture the previous values to calculate delta later.
 */
static VOID
unc_power_Enable_PMU (
    PVOID  param
)
{
    S32                   j;
    U64                  *buffer              = prev_counter_data;
    U32                   dev_idx             = *((U32*)param);
    U32                   start_index;
    DRV_CONFIG            pcfg_unc;
    U32                   this_cpu            = CONTROL_THIS_CPU();
    CPU_STATE             pcpu                = &pcb[this_cpu];
    U32                   num_cpus            = GLOBAL_STATE_num_cpus(driver_state);
    U32                   cur_grp             = LWPMU_DEVICE_cur_group(&devices[(dev_idx)]);
    U32                   package_event_count = 0;
    U32                   thread_event_count  = 0;
    U32                   module_event_count  = 0;
    U64                   tmp_value           = 0;

    pcfg_unc    = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);

    // NOTE THAT the enable function currently captures previous values
    // for EMON collection to avoid unnecessary memory copy.
    if (!DRV_CONFIG_emon_mode(pcfg_unc)) {
        return;
    }

    start_index = DRV_CONFIG_emon_unc_offset(pcfg_unc, cur_grp);

    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_READ) {
        j =   start_index + ECB_entries_group_index(pecb,idx)  +
               package_event_count*num_packages +
               module_event_count*(GLOBAL_STATE_num_modules(driver_state)) +
               thread_event_count*num_cpus ;
        if (ECB_entries_event_scope(pecb,idx) == PACKAGE_EVENT) {
            j = j + core_to_package_map[this_cpu];
            package_event_count++;
            if (!CPU_STATE_socket_master(pcpu)) {
                continue;
            }
        }
        else if (ECB_entries_event_scope(pecb,idx) == MODULE_EVENT) {
            j = j + CPU_STATE_cpu_module_num(pcpu);
            module_event_count++;
            if (!CPU_STATE_cpu_module_master(pcpu)) {
                continue;
            }
        }
        else {
            j = j + this_cpu;
            thread_event_count++;
        }
        tmp_value = SYS_Read_MSR(ECB_entries_reg_id(pecb,idx));
        if (ECB_entries_max_bits(pecb,idx)) {
            tmp_value &= ECB_entries_max_bits(pecb,idx);
        }
        buffer[j] = tmp_value;
    } END_FOR_EACH_REG_UNC_OPERATION;

    return;
}


/* ------------------------------------------------------------------------- */
/*!
 * @fn unc_power_Read_PMU_Data(param)
 *
 * @param    param    The read thread node to process
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore count data and store into the buffer param;
 *           Uncore PMU does not support sampling, i.e. ignore the id parameter.
 */
static VOID
unc_power_Read_PMU_Data (
    PVOID  param
)
{
    S32                   j;
    U64                  *buffer              = read_counter_info;
    U64                  *prev_buffer         = prev_counter_data;
    U32                   dev_idx             = *((U32*)param);
    U32                   start_index;
    DRV_CONFIG            pcfg_unc;
    U32                   this_cpu            = CONTROL_THIS_CPU();
    CPU_STATE             pcpu                = &pcb[this_cpu];
    U32                   num_cpus            = GLOBAL_STATE_num_cpus(driver_state);
    U32                   cur_grp             = LWPMU_DEVICE_cur_group(&devices[(dev_idx)]);
    U32                   package_event_count = 0;
    U32                   thread_event_count  = 0;
    U32                   module_event_count  = 0;
    U64                   tmp_value;

    pcfg_unc    = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);
    start_index = DRV_CONFIG_emon_unc_offset(pcfg_unc, cur_grp);

    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_READ) {
        j =   start_index + ECB_entries_group_index(pecb,idx)  +
               package_event_count*num_packages +
               module_event_count*(GLOBAL_STATE_num_modules(driver_state)) +
               thread_event_count*num_cpus ;
        if (ECB_entries_event_scope(pecb,idx) == PACKAGE_EVENT) {
            j = j + core_to_package_map[this_cpu];
            package_event_count++;
            if (!CPU_STATE_socket_master(pcpu)) {
                continue;
            }
        }
        else if (ECB_entries_event_scope(pecb,idx) == MODULE_EVENT) {
            j = j + CPU_STATE_cpu_module_num(pcpu);
            module_event_count++;
            if (!CPU_STATE_cpu_module_master(pcpu)) {
                continue;
            }
        }
        else {
            j = j + this_cpu;
            thread_event_count++;
        }
        tmp_value = SYS_Read_MSR(ECB_entries_reg_id(pecb,idx));
        if (ECB_entries_max_bits(pecb,idx)) {
            tmp_value &= ECB_entries_max_bits(pecb,idx);
        }
        if (ECB_entries_counter_type(pecb,idx) == STATIC_COUNTER) {
            buffer[j] = tmp_value;
        }
        else {
            if (tmp_value >= prev_buffer[j]) {
                buffer[j] = tmp_value - prev_buffer[j];
            }
            else {
                buffer[j] = tmp_value + (ECB_entries_max_bits(pecb,idx) - prev_buffer[j]);
            }
        }
    } END_FOR_EACH_REG_UNC_OPERATION;

    return;
}

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  unc_power_dispatch =
{
    .init                     = unc_power_Allocate,
    .fini                     = unc_power_Free,
    .write                    = UNC_COMMON_Dummy_Func,
    .freeze                   = NULL,
    .restart                  = unc_power_Enable_PMU,
    .read_data                = unc_power_Read_PMU_Data,
    .check_overflow           = NULL,
    .swap_group               = NULL,
    .read_lbrs                = NULL,
    .cleanup                  = NULL,
    .hw_errata                = NULL,
    .read_power               = NULL,
    .check_overflow_errata    = NULL,
    .read_counts              = NULL,
    .check_overflow_gp_errata = NULL,
    .read_ro                  = NULL,
    .platform_info            = NULL,
    .trigger_read             = unc_power_Trigger_Read,
    .scan_for_uncore          = NULL
};
