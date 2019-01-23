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

extern U64                      *read_counter_info;

/*!
 * @fn          extern VOID UNC_COMMON_MSR_Write_PMU(VOID*)
 *
 * @brief       Initial write of PMU registers
 *              Walk through the enties and write the value of the register accordingly.
 *              When current_group = 0, then this is the first time this routine is called,
 *
 * @param       None
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
extern VOID
UNC_MSR_Write_PMU (
    PVOID            param
)
{
    U32            dev_idx       = *((U32*)param);
    U32            this_cpu      = CONTROL_THIS_CPU();
    CPU_STATE      pcpu          = &pcb[this_cpu];

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_WRITE) {
        SYS_Write_MSR(ECB_entries_reg_id(pecb,idx), ECB_entries_reg_value(pecb,idx));
        SEP_PRINT_DEBUG("UNC_MSR_Write_PMU Write reg = 0x%x ---> value 0x%llx\n",
                        ECB_entries_reg_id(pecb,idx), ECB_entries_reg_value(pecb,idx));

    } END_FOR_EACH_REG_UNC_OPERATION;

    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_READ) {
        SYS_Write_MSR(ECB_entries_reg_id(pecb,idx), 0ULL);
        SEP_PRINT_DEBUG("UNC_MSR_Write_PMU Write reg = 0x%x ---> value 0x0\n",
                        ECB_entries_reg_id(pecb,idx));
        if (LWPMU_DEVICE_counter_mask(&devices[dev_idx]) == 0) {
            LWPMU_DEVICE_counter_mask(&devices[dev_idx]) = (U64)ECB_entries_max_bits(pecb,idx);
        }
    } END_FOR_EACH_REG_UNC_OPERATION;

    return;
}

/*!
 * @fn         VOID UNC_MSR_Enable_PMU(PVOID)
 *
 * @brief      Set the enable bit for all the evsel registers
 *
 * @param      None
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
VOID
UNC_MSR_Enable_PMU (
    PVOID param
)
{
    U32            dev_idx       = *((U32*)param);
    U32            this_cpu      = CONTROL_THIS_CPU();
    CPU_STATE      pcpu          = &pcb[this_cpu];
    U64            reg_val       = 0;

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_ENABLE) {
        reg_val = ECB_entries_reg_value(pecb,idx);
        if (ECB_entries_reg_rw_type(pecb, idx)  == PMU_REG_RW_READ_WRITE) {
            reg_val = SYS_Read_MSR(ECB_entries_reg_id(pecb,idx));
            if (ECB_entries_reg_type(pecb,idx) == PMU_REG_UNIT_CTRL) {
                reg_val &= ECB_entries_reg_value(pecb,idx); 
            }
            else {
                reg_val |= ECB_entries_reg_value(pecb,idx); 
            }
        }
        SYS_Write_MSR(ECB_entries_reg_id(pecb,idx), reg_val);
        SEP_PRINT_DEBUG("UNC_MSR_Write_PMU Write reg = 0x%x ---> value 0x%llx\n",
                        ECB_entries_reg_id(pecb,idx), reg_val);
    } END_FOR_EACH_REG_UNC_OPERATION;

    return;
}


/*!
 * @fn         VOID UNC_MSR_Disable_PMU(PVOID)
 *
 * @brief      Set the enable bit for all the evsel registers
 *
 * @param      None
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
VOID
UNC_MSR_Disable_PMU (
    PVOID param
)
{
    U32            dev_idx       = *((U32*)param);
    U32            this_cpu      = CONTROL_THIS_CPU();
    CPU_STATE      pcpu          = &pcb[this_cpu];
    U64            reg_val       = 0;

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_DISABLE) {
        reg_val = ECB_entries_reg_value(pecb,idx);
        if (ECB_entries_reg_rw_type(pecb, idx)  == PMU_REG_RW_READ_WRITE) {
            reg_val = SYS_Read_MSR(ECB_entries_reg_id(pecb,idx));
            if (ECB_entries_reg_type(pecb,idx) == PMU_REG_UNIT_CTRL) {
                reg_val |= ECB_entries_reg_value(pecb,idx);
            }
            else {
                reg_val &= ECB_entries_reg_value(pecb,idx);
            }
        }
        SYS_Write_MSR(ECB_entries_reg_id(pecb,idx), reg_val);
        SEP_PRINT_DEBUG("UNC_MSR_Write_PMU Write reg = 0x%x ---> value 0x%llx\n",
                        ECB_entries_reg_id(pecb,idx), reg_val);
    } END_FOR_EACH_REG_UNC_OPERATION;

    return;
}

/*!
 * @fn UNC_MSR_Read_PMU_Data(param)
 *
 * @param    param    The read thread node to process
 * @param    id       The id refers to the device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore count data and store into the buffer
 *           Let us say we have 2 core events in a dual socket JKTN;
 *           The start_index will be at 32 as it will 2 events in 16 CPU per socket
 *           The position for first event of QPI will be computed based on its event
 *
 */
VOID
UNC_MSR_Read_PMU_Data (
    PVOID  param
)
{
    U32             dev_idx             = *((U32*)param);
    U32             this_cpu            = CONTROL_THIS_CPU();
    U32             package_num         = 0;
    U64            *buffer              = read_counter_info;
    DRV_CONFIG      pcfg_unc;
    U64             start_index;
    CPU_STATE       pcpu                = &pcb[this_cpu];
    U64             j                   = 0;
    U32             sub_evt_index       = 0;
    S32             prev_ei             = -1;
    S32             cur_ei              = 0;
    U32             cur_grp             = LWPMU_DEVICE_cur_group(&devices[(dev_idx)]);
    ECB             pecb                = LWPMU_DEVICE_PMU_register_data(&devices[(dev_idx)])[cur_grp];
    U32             num_events          = 0;

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }
    if (!pecb) {
        return;
    }
    num_events = ECB_num_events(pecb);
    package_num         = core_to_package_map[this_cpu];
    pcfg_unc            = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);
    start_index         = DRV_CONFIG_emon_unc_offset(pcfg_unc, cur_grp);
    SEP_PRINT_DEBUG("offset for uncore group %d is %d num_pkgs = 0x%x num_events = %d\n", cur_grp, (int)start_index, num_packages, num_events);

    //Read in the counts into temporary buffer
    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_READ) {
        cur_ei = (S32)ECB_entries_group_index(pecb, idx);
        //the buffer index for this PMU needs to account for each event
        j = start_index +  ECB_entries_group_index(pecb, idx) +
            ECB_entries_emon_event_id_index_local(pecb,idx) +
            sub_evt_index*num_packages*LWPMU_DEVICE_num_units(&devices[dev_idx])+
            package_num * LWPMU_DEVICE_num_units(&devices[dev_idx]);
            SEP_PRINT_DEBUG("%d + %d + %d + %d*%d*%d + %d * %d = j \n",
                  (int)start_index,ECB_entries_group_index(pecb, idx),ECB_entries_emon_event_id_index_local(pecb,idx),
                  sub_evt_index,num_packages,LWPMU_DEVICE_num_units(&devices[dev_idx]), package_num,LWPMU_DEVICE_num_units(&devices[dev_idx]));
        buffer[j] = SYS_Read_MSR(ECB_entries_reg_id(pecb,idx));
        SEP_PRINT_DEBUG("j = %d value = 0x%llx pkg = %d  e_id = %d\n",(int)j, buffer[j], package_num, ECB_entries_emon_event_id_index_local(pecb,idx));
        //Increment sub_evt_index so that the next event position is adjusted
        if ((prev_ei == -1 )|| (prev_ei != cur_ei)) {
             prev_ei = cur_ei;
             sub_evt_index++;
        }
        if (sub_evt_index == num_events) {
            sub_evt_index = 0;
        }
    } END_FOR_EACH_REG_UNC_OPERATION;

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       void UNC_MSR_Trigger_Read(id)
 *
 * @param    id       Device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore data from counters and store into buffer
 */
extern  VOID
UNC_MSR_Trigger_Read (
    PVOID  param,
    U32    id
)
{
    U32             this_cpu            = CONTROL_THIS_CPU();
    U32             package_num         = core_to_package_map[this_cpu];
    U32             cur_grp             = LWPMU_DEVICE_cur_group(&devices[id]);
    ECB             pecb                = LWPMU_DEVICE_PMU_register_data(&devices[id])[cur_grp];
    U32             index               = 0;
    U64             diff                = 0;
    U64             value;
    U64            *data;

    // Write GroupID
    data = (U64*)((S8*)param + ECB_group_offset(pecb));
    *data = cur_grp + 1;
    //Read in the counts into uncore buffer
    FOR_EACH_REG_UNC_OPERATION(pecb, id, idx, PMU_OPERATION_READ) {
        value = SYS_Read_MSR(ECB_entries_reg_id(pecb,idx));
        //check for overflow
        if (value < LWPMU_DEVICE_prev_value(&devices[id])[package_num][index]) {
            diff = LWPMU_DEVICE_counter_mask(&devices[id]) - LWPMU_DEVICE_prev_value(&devices[id])[package_num][index];
            diff += value;
        }
        else {
            diff = value - LWPMU_DEVICE_prev_value(&devices[id])[package_num][index];
        }
        LWPMU_DEVICE_acc_value(&devices[id])[package_num][cur_grp][index] += diff;
        LWPMU_DEVICE_prev_value(&devices[id])[package_num][index] = value;
        data  = (U64 *)((S8*)param + ECB_entries_counter_event_offset(pecb,idx));
        *data = LWPMU_DEVICE_acc_value(&devices[id])[package_num][cur_grp][index];
        index++;
        SEP_PRINT_DEBUG("UNC_MSR_Write_PMU Write reg = 0x%x ---> value 0x0\n",
                        ECB_entries_reg_id(pecb,idx));
    } END_FOR_EACH_REG_UNC_OPERATION;

    return;
}


/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  unc_msr_dispatch =
{
    .init                     = NULL,
    .fini                     = NULL,
    .write                    = UNC_MSR_Write_PMU,
    .freeze                   = UNC_MSR_Disable_PMU,
    .restart                  = UNC_MSR_Enable_PMU,
    .read_data                = UNC_MSR_Read_PMU_Data,
    .check_overflow           = NULL,
    .swap_group               = NULL,
    .read_lbrs                = NULL,
    .cleanup                  = UNC_COMMON_MSR_Clean_Up,
    .hw_errata                = NULL,
    .read_power               = NULL,
    .check_overflow_errata    = NULL,
    .read_counts              = NULL,
    .check_overflow_gp_errata = NULL,
    .read_ro                  = NULL,
    .platform_info            = NULL,
    .trigger_read             = UNC_MSR_Trigger_Read,
    .scan_for_uncore          = NULL
};
