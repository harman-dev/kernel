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
#include "unc_common.h"
#include "ecb_iterators.h"
#include "pebs.h"
#include "inc/pci.h"

extern U64                        *read_counter_info;
extern struct pci_bus            **unc_package_to_bus_map;
extern UNCORE_TOPOLOGY_INFO_NODE   uncore_topology;

#define  PCI_INVALID_VALUE    0xFFFFFFFF

/*!
 * @fn          extern VOID UNC_PCI_Write_PMU(VOID*)
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
UNC_PCI_Write_PMU (
    PVOID            param
)
{
    U32         pci_address;
    U32         device_id;
    U32         dev_idx       = *((U32*)param);
    U32         value;
    U32         vendor_id;
    U32         this_cpu      = CONTROL_THIS_CPU();
    CPU_STATE   pcpu          = &pcb[this_cpu];
    U32         package_num   = 0;
    U32         bus_map_index = 0;
    U32         dev_node      = 0;
    U32         cur_grp       = LWPMU_DEVICE_cur_group(&devices[(dev_idx)]);
    ECB         pecb          = LWPMU_DEVICE_PMU_register_data(&devices[(dev_idx)])[cur_grp];


    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }
    if (!pecb) {
        return;
    }

    // first, figure out which package maps to which bus
    SEP_PRINT_DEBUG("*********** SEP Write PMU START *******\n");

    dev_node            = ECB_dev_node(pecb);
    package_num         = core_to_package_map[this_cpu];
    bus_map_index       = package_num*MAX_DEVICES + dev_node;
    if (unc_package_to_bus_map[bus_map_index] == NULL) {
        return;
    }
    LWPMU_DEVICE_pci_dev_node_index(&devices[dev_idx]) = dev_node;

    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_WRITE) {
        if (ECB_entries_reg_type(pecb, idx)  == PMU_REG_GLOBAL_CTRL) {
             //Check if we need to zero this MSR out
             SYS_Write_MSR(ECB_entries_reg_id(pecb,idx), 0LL);
             SEP_PRINT_DEBUG("UNC_PCI_Write_PMU wrote GLOBAL_CONTROL_MSR 0x%x\n", ECB_entries_reg_id(pecb,idx));
             continue;
        }

        // otherwise, we have a valid entry
        // now we just need to find the corresponding bus #
        ECB_entries_bus_no(pecb,idx) = unc_package_to_bus_map[bus_map_index]->number;
        pci_address = FORM_PCI_ADDR(unc_package_to_bus_map[bus_map_index]->number,
                                    ECB_entries_dev_no(pecb,idx),
                                    ECB_entries_func_no(pecb,idx),
                                    0);
        value = PCI_Read_Ulong(pci_address);

        CONTINUE_IF_NOT_GENUINE_INTEL_DEVICE(value, vendor_id, device_id);

        if (ECB_entries_reg_type(pecb, idx)  == PMU_REG_UNIT_CTRL) {
             // busno can not be stored in ECB because different sockets have different bus no.
             PCI_Write(unc_package_to_bus_map[bus_map_index],
                       ECB_entries_dev_no(pecb,idx),
                       ECB_entries_func_no(pecb,idx),
                       ECB_entries_reg_id(pecb,idx),
                       (U32)ECB_entries_reg_value(pecb,idx));
             SEP_PRINT_DEBUG("UNC_PCI_Write_PMU bus=%d dev=%d func=%d cpu=%d, reg = 0x%x --- value 0x%llx\n",
                             ECB_entries_bus_no(pecb,idx),ECB_entries_dev_no(pecb,idx),ECB_entries_func_no(pecb,idx),
                             this_cpu, ECB_entries_reg_id(pecb,idx), ECB_entries_reg_value(pecb,idx));
             continue;
        }

        // now program at the corresponding offset
        PCI_Write(unc_package_to_bus_map[bus_map_index],
                  ECB_entries_dev_no(pecb,idx),
                  ECB_entries_func_no(pecb,idx),
                  ECB_entries_reg_id(pecb,idx),
                  (U32)ECB_entries_reg_value(pecb,idx));
        SEP_PRINT_DEBUG("UNC_COMMON_PCI_Write_PMU bus=%d dev=%d func=%d cpu=%d, reg = 0x%x --- value 0x%x\n",
                             ECB_entries_bus_no(pecb,idx),ECB_entries_dev_no(pecb,idx),ECB_entries_func_no(pecb,idx),
                             this_cpu, ECB_entries_reg_id(pecb,idx), (U32)ECB_entries_reg_value(pecb,idx));

    } END_FOR_EACH_REG_UNC_OPERATION;

    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_READ) {
        PCI_Write(unc_package_to_bus_map[bus_map_index],
                  ECB_entries_dev_no(pecb,idx),
                  ECB_entries_func_no(pecb,idx),
                  ECB_entries_reg_id(pecb,idx),
                  0);
        PCI_Write(unc_package_to_bus_map[bus_map_index],
                  ECB_entries_dev_no(pecb,idx),
                  ECB_entries_func_no(pecb,idx),
                  (ECB_entries_reg_id(pecb,idx) + NEXT_ADDR_OFFSET),
                  0);

        SEP_PRINT_DEBUG("UNC_PCI_Write_PMU bus=%d dev=%d func=%d cpu=%d, reg = 0x%x --- value 0x%x\n",
                         ECB_entries_bus_no(pecb,idx),ECB_entries_dev_no(pecb,idx),ECB_entries_func_no(pecb,idx),
                         this_cpu, ECB_entries_reg_id(pecb,idx), (U32)ECB_entries_reg_value(pecb,idx));
            // this is needed for overflow detection of the accumulators.
        if (LWPMU_DEVICE_counter_mask(&devices[dev_idx]) == 0) {
             LWPMU_DEVICE_counter_mask(&devices[dev_idx]) = (U64)ECB_entries_max_bits(pecb,idx);
        }
    } END_FOR_EACH_REG_UNC_OPERATION;
    SEP_PRINT_DEBUG("*********** SEP Write PMU END *******\n\n");

    return;
}

/*!
 * @fn         static VOID UNC_PCI_Enable_PMU(PVOID)
 *
 * @brief      Set the enable bit for all the EVSEL registers
 *
 * @param      Device Index of this PMU unit
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
extern VOID
UNC_PCI_Enable_PMU (
    PVOID               param
)
{
    U32            dev_idx       = *((U32 *)param);
    U32            this_cpu      = CONTROL_THIS_CPU();
    CPU_STATE      pcpu          = &pcb[this_cpu];
    U32            package_num   = 0;
    U32            dev_node      = LWPMU_DEVICE_pci_dev_node_index(&devices[dev_idx]);
    U32            bus_map_index = 0;
    U32            reg_val       = 0;

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    package_num         = core_to_package_map[this_cpu];
    bus_map_index       = package_num*MAX_DEVICES + dev_node;
    if (unc_package_to_bus_map[bus_map_index] == NULL) {
        return;
    }
    SEP_PRINT_DEBUG("*********** SEP Enable PMU START *******\n");
    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_ENABLE) {
        if (ECB_entries_reg_type(pecb, idx)  == PMU_REG_GLOBAL_CTRL) {
            SYS_Write_MSR(ECB_entries_reg_id(pecb,idx), ECB_entries_reg_value(pecb,idx));
            SEP_PRINT_DEBUG("UNC_PCI_Enable_PMU wrote GLOBAL_CONTROL_MSR 0x%x val=0x%llx\n",
                         ECB_entries_reg_id(pecb,idx), ECB_entries_reg_value(pecb,idx));
            continue;
        }
        reg_val = ECB_entries_reg_value(pecb,idx);
        if (ECB_entries_reg_rw_type(pecb, idx)  == PMU_REG_RW_READ_WRITE) {
            reg_val = PCI_Read(unc_package_to_bus_map[bus_map_index],
                             ECB_entries_dev_no(pecb,idx),
                             ECB_entries_func_no(pecb,idx),
                             ECB_entries_reg_id(pecb,idx));
            reg_val &= ECB_entries_reg_value(pecb,idx); 
        }
        PCI_Write(unc_package_to_bus_map[bus_map_index],
               ECB_entries_dev_no(pecb,idx),
               ECB_entries_func_no(pecb,idx),
               ECB_entries_reg_id(pecb,idx),
               reg_val);
        SEP_PRINT_DEBUG("UNC_PCI_Enable_PMU bus=%d dev=%d func=%d cpu=%d Event_reg = 0x%x --- value 0x%x\n",
                             ECB_entries_bus_no(pecb,idx),ECB_entries_dev_no(pecb,idx),ECB_entries_func_no(pecb,idx),
                  this_cpu, ECB_entries_reg_id(pecb,idx), reg_val);
    } END_FOR_EACH_REG_UNC_OPERATION;
    SEP_PRINT_DEBUG("*********** SEP Enable PMU END *******\n\n");

    return;
}

/*!
 * @fn           extern VOID UNC_PCI_Disable_PMU(PVOID)
 *
 * @brief        Disable the per unit global control to stop the PMU counters.
 *
 * @param        Device Index of this PMU unit
 * @control_msr  Control MSR address
 * @enable_val   If counter freeze bit does not work, counter enable bit should be cleared
 * @disable_val  Disable collection
 *
 * @return       None
 *
 * <I>Special Notes:</I>
 */
extern VOID
UNC_PCI_Disable_PMU (
    PVOID               param
)
{
    U32            dev_idx       = *((U32 *)param);
    U32            this_cpu      = CONTROL_THIS_CPU();
    CPU_STATE      pcpu          = &pcb[this_cpu];
    U32            package_num   = 0;
    U32            dev_node      = LWPMU_DEVICE_pci_dev_node_index(&devices[dev_idx]);
    U32            bus_map_index = 0;
    U32            reg_val       = 0;

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    package_num         = core_to_package_map[this_cpu];
    bus_map_index       = package_num*MAX_DEVICES + dev_node;
    if (unc_package_to_bus_map[bus_map_index] == NULL) {
        return;
    }
    SEP_PRINT_DEBUG("*********** SEP Disable PMU START *******\n");
    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_DISABLE) {
        if (ECB_entries_reg_type(pecb, idx)  == PMU_REG_GLOBAL_CTRL) {
            SYS_Write_MSR(ECB_entries_reg_id(pecb,idx), ECB_entries_reg_value(pecb,idx));
            SEP_PRINT_DEBUG("UNC_PCI_Disable_PMU wrote GLOBAL_CONTROL_MSR 0x%x val=0x%llx\n",
                         ECB_entries_reg_id(pecb,idx), ECB_entries_reg_value(pecb,idx));
            continue;
        }
        reg_val = ECB_entries_reg_value(pecb,idx);
        if (ECB_entries_reg_rw_type(pecb, idx)  == PMU_REG_RW_READ_WRITE) {
            reg_val = PCI_Read(unc_package_to_bus_map[bus_map_index],
                             ECB_entries_dev_no(pecb,idx),
                             ECB_entries_func_no(pecb,idx),
                             ECB_entries_reg_id(pecb,idx));
            reg_val |= ECB_entries_reg_value(pecb,idx); 
        }
        PCI_Write(unc_package_to_bus_map[bus_map_index],
               ECB_entries_dev_no(pecb,idx),
               ECB_entries_func_no(pecb,idx),
               ECB_entries_reg_id(pecb,idx),
               reg_val);
        SEP_PRINT_DEBUG("UNC_PCI_Disable_PMU bus=%d dev=%d func=%d cpu=%d Event_reg = 0x%x --- value 0x%x\n",
                        ECB_entries_bus_no(pecb,idx),ECB_entries_dev_no(pecb,idx),ECB_entries_func_no(pecb,idx),
                  this_cpu, ECB_entries_reg_id(pecb,idx), reg_val);
    } END_FOR_EACH_REG_UNC_OPERATION;
    SEP_PRINT_DEBUG("*********** SEP Disable PMU END *******\n\n");

    return;
}


/* ------------------------------------------------------------------------- */
/*!
 * @fn       void UNC_PCI_Trigger_Read(id)
 *
 * @param    id       Device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore data from counters and store into buffer
 */
extern  VOID
UNC_PCI_Trigger_Read (
    PVOID  param,
    U32    id
)
{
    U32             this_cpu      = CONTROL_THIS_CPU();
    U32             package_num   = core_to_package_map[this_cpu];
    U32             dev_node      = LWPMU_DEVICE_pci_dev_node_index(&devices[id]);
    U32             bus_map_index = package_num*MAX_DEVICES + dev_node;
    U32             cur_grp       = LWPMU_DEVICE_cur_group(&devices[id]);
    ECB             pecb          = LWPMU_DEVICE_PMU_register_data(&devices[id])[cur_grp];
    U32             index         = 0;
    U64             value_low     = 0;
    U64             value_high    = 0;
    U64             diff          = 0;
    U64             value;
    U64            *data;

    if (unc_package_to_bus_map[bus_map_index] == NULL) {
        return;
    }
    SEP_PRINT_DEBUG("*********** SEP Read PMU START *******\n");

    // Write GroupID
    data = (U64*)((S8*)param + ECB_group_offset(pecb));
    *data = cur_grp + 1;
    // Read the counts into uncore buffer
    FOR_EACH_REG_UNC_OPERATION(pecb, id, idx, PMU_OPERATION_READ) {
        // read lower 4 bytes
        value_low = PCI_Read(unc_package_to_bus_map[bus_map_index],
                             ECB_entries_dev_no(pecb,idx),
                             ECB_entries_func_no(pecb,idx),
                             ECB_entries_reg_id(pecb,idx));
        value = LOWER_4_BYTES_MASK & value_low;

        // read upper 4 bytes
        value_high = PCI_Read(unc_package_to_bus_map[bus_map_index],
                              ECB_entries_dev_no(pecb,idx),
                              ECB_entries_func_no(pecb,idx),
                              (ECB_entries_reg_id(pecb,idx) + NEXT_ADDR_OFFSET));
        value |= value_high << NEXT_ADDR_SHIFT;
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
    } END_FOR_EACH_REG_UNC_OPERATION;

    SEP_PRINT_DEBUG("*********** SEP Read PMU END *******\n\n");

    return;
}

/*!
 * @fn       extern   UNC_PCI_Read_PMU_Data(param)
 *
 * @param    param    The device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore count data and store into the buffer;
 */
extern VOID
UNC_PCI_Read_PMU_Data(
    PVOID           param
)
{
    U32             dev_idx             = *((U32*)param);
    U64             value_low           = 0;
    U64             value_high          = 0;
    U32             this_cpu            = CONTROL_THIS_CPU();
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
    U32             dev_node            = LWPMU_DEVICE_pci_dev_node_index(&devices[dev_idx]);
    U32             package_num         = 0;
    U32             bus_map_index       = 0;

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }
    if (!pecb) {
        return;
    }
    num_events = ECB_num_events(pecb);
    SEP_PRINT_DEBUG("*********** SEP Read PMU START *******\n");
    package_num         = core_to_package_map[this_cpu];
    bus_map_index       = package_num*MAX_DEVICES + dev_node;
    if (unc_package_to_bus_map[bus_map_index] == NULL) {
        return;
    }
    pcfg_unc            = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);
    start_index         = DRV_CONFIG_emon_unc_offset(pcfg_unc, cur_grp);

    //Read in the counts into temporary buffer
    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_READ) {
        cur_ei = (S32)ECB_entries_group_index(pecb, idx);
        //the buffer index for this PMU needs to account for each event
        j = start_index +  ECB_entries_group_index(pecb, idx) +
            ECB_entries_emon_event_id_index_local(pecb,idx) +
            sub_evt_index*num_packages*LWPMU_DEVICE_num_units(&devices[dev_idx])+
            package_num * LWPMU_DEVICE_num_units(&devices[dev_idx]);

        // read lower 4 bytes
        value_low = PCI_Read(unc_package_to_bus_map[bus_map_index],
                             ECB_entries_dev_no(pecb,idx),
                             ECB_entries_func_no(pecb,idx),
                             ECB_entries_reg_id(pecb,idx));
        value_low &= LOWER_4_BYTES_MASK;

        // read upper 4 bytes
        value_high = PCI_Read(unc_package_to_bus_map[bus_map_index],
                              ECB_entries_dev_no(pecb,idx),
                              ECB_entries_func_no(pecb,idx),
                              (ECB_entries_reg_id(pecb,idx) + NEXT_ADDR_OFFSET));
        buffer[j] = (value_high << NEXT_ADDR_SHIFT) | value_low;
        SEP_PRINT_DEBUG("bus=%d dev=%d func=%d cpu=%d j = %d value = %llu pkg = %d  e_id = %d\n", ECB_entries_bus_no(pecb,idx),ECB_entries_dev_no(pecb,idx),ECB_entries_func_no(pecb,idx),this_cpu, (int)j, buffer[j],package_num, ECB_entries_emon_event_id_index_local(pecb,idx));
        //Increment sub_evt_index so that the next event position is adjusted
        if ((prev_ei == -1 )|| (prev_ei != cur_ei)) {
             prev_ei = cur_ei;
             sub_evt_index++;
        }
        if (sub_evt_index == num_events) {
            sub_evt_index = 0;
        }
    } END_FOR_EACH_REG_UNC_OPERATION;
    SEP_PRINT_DEBUG("*********** SEP Read PMU END *******\n\n");

    return;
}

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  unc_pci_dispatch =
{
    .init                     = NULL,
    .fini                     = NULL,
    .write                    = UNC_PCI_Write_PMU,
    .freeze                   = UNC_PCI_Disable_PMU,
    .restart                  = UNC_PCI_Enable_PMU,
    .read_data                = UNC_PCI_Read_PMU_Data,
    .check_overflow           = NULL,
    .swap_group               = NULL,
    .read_lbrs                = NULL,
    .cleanup                  = UNC_COMMON_PCI_Clean_Up,
    .hw_errata                = NULL,
    .read_power               = NULL,
    .check_overflow_errata    = NULL,
    .read_counts              = NULL,
    .check_overflow_gp_errata = NULL,
    .read_ro                  = NULL,
    .platform_info            = NULL,
    .trigger_read             = UNC_PCI_Trigger_Read,
    .scan_for_uncore          = NULL
};

