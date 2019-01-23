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

extern EVENT_CONFIG   global_ec;
extern U64           *read_counter_info;
extern U64           *prev_counter_data;
extern DRV_CONFIG     pcfg;

extern U64 *virtual_address_table[];
extern U8  *fpga_gb_dev_valid[];

#define MASK_32BIT             0xffffffff
#define MASK_64BIT             0xffffffff00000000ULL

#define IS_MASTER(device_type, cpu)               (((device_type) == DRV_SINGLE_INSTANCE)? CPU_STATE_system_master(&pcb[cpu]): CPU_STATE_socket_master(&pcb[(cpu)]))
#define IS_MMIO_MAP_VALID(devid, pkg)             (virtual_address_table[(devid)][(pkg)] != 0)
#define GET_PACKAGE_NUM(device_type, cpu)         (((device_type) == DRV_SINGLE_INSTANCE)? 0:core_to_package_map[cpu])
#define IS_64BIT(mask)                            (((mask)>>32) != 0)

#define EVENT_COUNTER_MAX_TRY  30

struct FPGA_CONTROL_NODE_S {
    union {
        struct {
            U64 rst_ctrs         : 1;
            U64 rsvd1            : 7;
            U64 frz              : 1;
            U64 rsvd2            : 7;
            U64 event_select     : 4;
            U64 port_id          : 2;
            U64 rsvd3            : 1;
            U64 port_enable      : 1;
            U64 rsvd4            : 40;
        } bits;
        U64 bit_field;
    } u;
} control_node;
 

/*!
 * @fn          static VOID unc_mmio_Write_PMU(VOID*)
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
static VOID
unc_mmio_Write_PMU (
    VOID  *param
)
{


    U32              dev_idx     = *((U32*)param);
    U32              offset_delta;
    DRV_CONFIG       pcfg_unc    = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);
    U32              event_id    = 0;
    U64              tmp_value   = 0;
    U32              this_cpu    = CONTROL_THIS_CPU();
    U32              package_num = GET_PACKAGE_NUM(DRV_CONFIG_device_type(pcfg_unc), this_cpu);
    U32              cur_grp     = LWPMU_DEVICE_cur_group(&devices[(dev_idx)]);
    ECB              pecb        = LWPMU_DEVICE_PMU_register_data(&devices[(dev_idx)])[(cur_grp)];
    U64              virtual_addr = 0;
    U32              idx_w       = 0;
    U32              event_code  = 0;
    U32              counter     = 0;

    if (!IS_MASTER(DRV_CONFIG_device_type(pcfg_unc), this_cpu)) {
          return;
    }

    if (!pecb) {
        return;
    }

    if (!IS_MMIO_MAP_VALID(dev_idx, package_num)) {
        SEP_PRINT_DEBUG("unc_mmio_Write: device %d, no virtual address mapping available for this device, return\n", dev_idx);
        return;
    }

    virtual_addr = virtual_address_table[dev_idx][package_num];

    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_WRITE) {
        *(U64*)(long)(virtual_addr + ECB_entries_reg_id(pecb,idx)) = ECB_entries_reg_value(pecb,idx);
        SEP_PRINT_DEBUG("unc_mmio_Write_PMU Base address = 0x%llx Write reg = 0x%x ---> value 0x%llx\n",
                        virtual_addr, ECB_entries_reg_id(pecb,idx), ECB_entries_reg_value(pecb,idx));
    } END_FOR_EACH_REG_UNC_OPERATION;

    if (!DRV_CONFIG_event_based_counts(pcfg_unc)) {
        return;
    }

    idx_w = ECB_operations_register_start(pecb, PMU_OPERATION_WRITE);
    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_READ) {
        if (ECB_entries_pci_id_offset(pecb,idx) > DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(&ECB_pcidev_entry_node(pecb))) {
            offset_delta =  ECB_entries_pci_id_offset(pecb,idx) -
                              DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(&ECB_pcidev_entry_node(pecb));
        } 
        else {
            offset_delta =  ECB_entries_pci_id_offset(pecb,idx);
        }

        if ((ECB_entries_counter_type(pecb,idx) == PROG_FREERUN_COUNTER) &&
            (ECB_entries_event_id_index_local(pecb,idx) == 0)) {
            //Write event code before reading
            *(U64*)(long)(virtual_addr + ECB_entries_reg_id(pecb,idx_w)) = ECB_entries_reg_value(pecb,idx_w);
            control_node.u.bit_field = ECB_entries_reg_value(pecb,idx_w);
            event_code = (U32)control_node.u.bits.event_select;
            idx_w++;
        }

        // this is needed for overflow detection of the accumulators.
        if (IS_64BIT((U64)(ECB_entries_max_bits(pecb,idx)))) {
            if (ECB_entries_counter_type(pecb,idx) == PROG_FREERUN_COUNTER) {
                do {
                    if (counter > EVENT_COUNTER_MAX_TRY) {
                        break;
                    }
                    tmp_value = SYS_MMIO_Read64(virtual_addr, offset_delta);
                    counter++;
                } while (event_code != (tmp_value >>60));
            }
            tmp_value = SYS_MMIO_Read64(virtual_addr, offset_delta);
        }
        else {
            tmp_value = readl((U32*)(long)(virtual_addr + offset_delta));
        }
        tmp_value &= (U64)ECB_entries_max_bits(pecb,idx);
        tmp_value >>= ECB_entries_bit_position(pecb,idx);
        LWPMU_DEVICE_prev_value(&devices[dev_idx])[package_num][event_id] = tmp_value;
        event_id++;

        if (LWPMU_DEVICE_counter_mask(&devices[dev_idx]) == 0) {
            LWPMU_DEVICE_counter_mask(&devices[dev_idx]) = (U64)ECB_entries_max_bits(pecb,idx);
        }
    } END_FOR_EACH_REG_UNC_OPERATION;
    SEP_PRINT_DEBUG("unc_mmio_Write_PMU: BAR address is 0x%llx and virt is 0x%llx\n",DRV_PCI_DEVICE_ENTRY_bar_address(&ECB_pcidev_entry_node(pecb)), virtual_addr);

    return;
}

/*!
 * @fn         static VOID unc_mmio_Enable_PMU(PVOID)
 *
 * @brief      Capture the previous values to calculate delta later.
 *
 * @param      None
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static void
unc_mmio_Enable_PMU (
    PVOID  param
)
{
    S32            j;
    U64           *buffer       = prev_counter_data;
    U32            this_cpu     = CONTROL_THIS_CPU();
    U32            dev_idx      = *((U32*)param);
    U32            start_index;
    DRV_CONFIG     pcfg_unc     = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);
    U32            offset_delta;
    U32            package_event_count = 0;
    U32            package_num  = GET_PACKAGE_NUM(DRV_CONFIG_device_type(pcfg_unc), this_cpu);
    U32            cur_grp      = LWPMU_DEVICE_cur_group(&devices[(dev_idx)]);
    ECB            pecb         = LWPMU_DEVICE_PMU_register_data(&devices[(dev_idx)])[(cur_grp)];
    U64            virtual_addr = 0;
    U64            reg_val      = 0;
    U32            idx_w        = 0;
    U32            event_code   = 0;
    U32            counter      = 0;
    U32            sub_evt_index = 0;
    S32            prev_ei      = -1;
    S32            cur_ei       = 0;
    U32            num_events   = 0;

    if (!IS_MASTER(DRV_CONFIG_device_type(pcfg_unc), this_cpu)) {
          return;
    }

    if (!pecb) {
        return;
    }

    if (!IS_MMIO_MAP_VALID(dev_idx, package_num)) {
        return;
    }

    virtual_addr = virtual_address_table[dev_idx][package_num];

    // NOTE THAT the enable function currently captures previous values
    // for EMON collection to avoid unnecessary memory copy.
    if (DRV_CONFIG_emon_mode(pcfg_unc)) {
        num_events  = ECB_num_events(pecb);
        start_index = DRV_CONFIG_emon_unc_offset(pcfg_unc, cur_grp);
        idx_w = ECB_operations_register_start(pecb, PMU_OPERATION_WRITE);
        FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_READ) {
            cur_ei = (S32)ECB_entries_group_index(pecb, idx);
            if (prev_ei == -1) {
                prev_ei = cur_ei;
            }
            if (cur_ei != prev_ei) {
                sub_evt_index++;
                if (sub_evt_index == num_events) {
                    sub_evt_index = 0;
                }
                prev_ei = cur_ei;
            }
            if (ECB_entries_pci_id_offset(pecb,idx) > DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(&ECB_pcidev_entry_node(pecb))) {
                offset_delta =  ECB_entries_pci_id_offset(pecb,idx) -
                                  DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(&ECB_pcidev_entry_node(pecb));
            } 
            else {
                offset_delta =  ECB_entries_pci_id_offset(pecb,idx);
            }

            if ((ECB_entries_counter_type(pecb,idx) == PROG_FREERUN_COUNTER) &&
                (ECB_entries_event_id_index_local(pecb,idx) == 0)) {
                *(U64*)(long)(virtual_addr + ECB_entries_reg_id(pecb,idx_w)) = ECB_entries_reg_value(pecb,idx_w);
                control_node.u.bit_field = ECB_entries_reg_value(pecb,idx_w);
                event_code = (U32)control_node.u.bits.event_select;
                idx_w++;
            }

            if ((ECB_entries_event_scope(pecb,idx) == PACKAGE_EVENT) ||
                (ECB_entries_event_scope(pecb,idx) == SYSTEM_EVENT)) {
                j = start_index +  ECB_entries_group_index(pecb, idx) +
                    ECB_entries_emon_event_id_index_local(pecb,idx) +
                    sub_evt_index*num_packages*LWPMU_DEVICE_num_units(&devices[dev_idx])+
                    package_num * LWPMU_DEVICE_num_units(&devices[dev_idx]);
                if (IS_64BIT((U64)(ECB_entries_max_bits(pecb,idx)))) {
                    if (ECB_entries_counter_type(pecb,idx) == PROG_FREERUN_COUNTER) {
                        do {
                            if (counter > EVENT_COUNTER_MAX_TRY) {
                                break;
                            }
                            buffer[j] = SYS_MMIO_Read64(virtual_addr, offset_delta);
                            counter++;
                        } while (event_code != (buffer[j] >>60));
                    }
                    buffer[j] = SYS_MMIO_Read64(virtual_addr, offset_delta);
                }
                else {
                    buffer[j] = readl((U32*)(long)(virtual_addr) + offset_delta);
                }
                buffer[j] &= (U64)ECB_entries_max_bits(pecb,idx);
                SEP_PRINT_DEBUG("unc_mmio_Enable_PMU cpu=%d, ei=%d, eil=%d, MSR=0x%x, j=%d, si=%d, value=0x%llx\n",
                    this_cpu, ECB_entries_event_id_index(pecb, idx), ECB_entries_emon_event_id_index_local(pecb,idx),
                    ECB_entries_reg_id(pecb,idx),j, start_index,buffer[j]);
                package_event_count++;
            }
        } END_FOR_EACH_REG_UNC_OPERATION;
    }

    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_ENABLE) {
        if (ECB_entries_reg_rw_type(pecb, idx)  == PMU_REG_RW_READ_WRITE) {
            reg_val = *(U64*)(long)(virtual_addr + ECB_entries_reg_id(pecb,idx));
            reg_val &= ECB_entries_reg_value(pecb,idx);
            *(U64*)(long)(virtual_addr + ECB_entries_reg_id(pecb,idx)) = reg_val;
        }
        SEP_PRINT_DEBUG("unc_mmio_Enable_PMU Base address = 0x%llx Write reg = 0x%x ---> value 0x%llx\n",
                        virtual_addr, ECB_entries_reg_id(pecb,idx), reg_val);
    } END_FOR_EACH_REG_UNC_OPERATION;

    return;
}

/*!
 * @fn         static VOID unc_mmio_Disable_PMU(PVOID)
 *
 * @brief      Unmap the virtual address when you stop sampling.
 *
 * @param      None
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static void
unc_mmio_Disable_PMU (
    PVOID  param
)
{
    U32            dev_idx       = *((U32*)param);
    U32            this_cpu      = CONTROL_THIS_CPU();
    U64            virtual_addr  = 0;
    U64            reg_val       = 0;
    DRV_CONFIG     pcfg_unc      = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);
    U32            package_num   = GET_PACKAGE_NUM(DRV_CONFIG_device_type(pcfg_unc), this_cpu);

    if (!IS_MASTER(DRV_CONFIG_device_type(pcfg_unc), this_cpu)) {
          return;
    }

    if (!IS_MMIO_MAP_VALID(dev_idx, package_num)) {
        return;
    }

    virtual_addr = virtual_address_table[dev_idx][package_num];

    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_DISABLE) {
        if (ECB_entries_reg_rw_type(pecb, idx)  == PMU_REG_RW_READ_WRITE) {
            reg_val = *(U64*)(long)(virtual_addr + ECB_entries_reg_id(pecb,idx));
            reg_val |= ECB_entries_reg_value(pecb,idx);
            *(U64*)(long)(virtual_addr + ECB_entries_reg_id(pecb,idx)) = reg_val;
        }
        SEP_PRINT_DEBUG("unc_mmio_Disable_PMU Base address = 0x%llx Write reg = 0x%x ---> value 0x%llx\n",
                        virtual_addr, ECB_entries_reg_id(pecb,idx), reg_val);
    } END_FOR_EACH_REG_UNC_OPERATION;

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       void unc_mmio_Trigger_Read(id)
 *
 * @param    id       Device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore data from counters and store into buffer
 */
static  VOID
unc_mmio_Trigger_Read (
    PVOID  param,
    U32    id
)
{
    U32             this_cpu            = CONTROL_THIS_CPU();
    U32             cur_grp             = LWPMU_DEVICE_cur_group(&devices[id]);
    ECB             pecb                = LWPMU_DEVICE_PMU_register_data(&devices[id])[(cur_grp)];
    U32             index               = 0;
    U64             diff                = 0;
    U32             offset_delta;
    U64             value               = 0ULL;
    U64            *data;
    U64             virtual_addr        = 0; 
    DRV_CONFIG      pcfg_unc            = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[id]);
    U32             package_num         = GET_PACKAGE_NUM(DRV_CONFIG_device_type(pcfg_unc), this_cpu);
    U32             idx_w               = 0;
    U32             event_code          = 0;
    U32             counter             = 0;

    if (!IS_MASTER(DRV_CONFIG_device_type(pcfg_unc), this_cpu)) {
          return;
    }

    if (!IS_MMIO_MAP_VALID(id, package_num)) {
        SEP_PRINT_DEBUG("unc_mmio_Triggger_Read_PMU: no virtual address mapping available for this device, return\n");
        return;
    }

    virtual_addr = virtual_address_table[id][package_num];

    // Write GroupID
    data = (U64*)((S8*)param + ECB_group_offset(pecb));
    *data = cur_grp + 1;
    //Read in the counts into temporary buffer
    idx_w = ECB_operations_register_start(pecb, PMU_OPERATION_WRITE);
    FOR_EACH_REG_UNC_OPERATION(pecb, id, idx, PMU_OPERATION_READ) {
        if (ECB_entries_pci_id_offset(pecb,idx) > DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(&ECB_pcidev_entry_node(pecb))) {
            offset_delta =  ECB_entries_pci_id_offset(pecb,idx) -
                              DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(&ECB_pcidev_entry_node(pecb));
        } 
        else {
            offset_delta =  ECB_entries_pci_id_offset(pecb,idx);
        }

        if ((ECB_entries_counter_type(pecb,idx) == PROG_FREERUN_COUNTER) &&
            (ECB_entries_event_id_index_local(pecb,idx) == 0)) {
            *(U64*)(long)(virtual_addr + ECB_entries_reg_id(pecb,idx_w)) = ECB_entries_reg_value(pecb,idx_w);
            control_node.u.bit_field = ECB_entries_reg_value(pecb,idx_w);
            event_code = (U32)control_node.u.bits.event_select;
            idx_w++;
        }

        if (IS_64BIT((U64)(ECB_entries_max_bits(pecb,idx)))) {
            if (ECB_entries_counter_type(pecb,idx) == PROG_FREERUN_COUNTER) {
                do {
                    if (counter > EVENT_COUNTER_MAX_TRY) {
                        break;
                    }
                    value = SYS_MMIO_Read64(virtual_addr, offset_delta);
                    counter++;
                } while (event_code != (value >>60));
            }
            value = SYS_MMIO_Read64(virtual_addr, offset_delta);
        }
        else {
            value = readl((U32*)(long)(virtual_addr + offset_delta));
        }
        value &= (U64)ECB_entries_max_bits(pecb,idx);
        value >>= ECB_entries_bit_position(pecb,idx);

        data = (U64 *)((S8*)param + ECB_entries_counter_event_offset(pecb,idx));
        //check for overflow if not a static counter
        if (ECB_entries_counter_type(pecb,idx) == STATIC_COUNTER) {
            *data = value;
        }
        else {
            if (value < LWPMU_DEVICE_prev_value(&devices[id])[package_num][index]) {
                diff = LWPMU_DEVICE_counter_mask(&devices[id]) - LWPMU_DEVICE_prev_value(&devices[id])[package_num][index];
                diff += value;
            }
            else {
                diff = value - LWPMU_DEVICE_prev_value(&devices[id])[package_num][index];
            }
            LWPMU_DEVICE_acc_value(&devices[id])[package_num][cur_grp][index] += diff;
            LWPMU_DEVICE_prev_value(&devices[id])[package_num][index] = value;
            *data = LWPMU_DEVICE_acc_value(&devices[id])[package_num][cur_grp][index];
        }
        index++;
    } END_FOR_EACH_REG_UNC_OPERATION;

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn unc_mmio_Read_PMU_Data(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Read all the data MSR's into a buffer.  Called by the interrupt handler.
 *
 */
static VOID
unc_mmio_Read_PMU_Data (
     PVOID   param
)
{
    S32            j;
    U64           *buffer       = read_counter_info;
    U64           *prev_buffer  = prev_counter_data;
    U32            this_cpu     = CONTROL_THIS_CPU();
    U32            dev_idx      = *((U32*)param);
    U32            start_index;
    DRV_CONFIG     pcfg_unc     = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);
    U32            offset_delta;
    U32            package_num  = GET_PACKAGE_NUM(DRV_CONFIG_device_type(pcfg_unc), this_cpu);
    U32            cur_grp      = LWPMU_DEVICE_cur_group(&devices[(dev_idx)]);
    ECB            pecb         = LWPMU_DEVICE_PMU_register_data(&devices[(dev_idx)])[(cur_grp)];
    U32            package_event_count = 0;
    U64            tmp_value    = 0ULL;
    U64            virtual_addr = 0;
    U32            idx_w        = 0;
    U32            event_code   = 0;
    U32            counter      = 0;
    U32            sub_evt_index = 0;
    S32            prev_ei      = -1;
    S32            cur_ei       = 0;
    U32            num_events   = 0;

    if (!IS_MASTER(DRV_CONFIG_device_type(pcfg_unc), this_cpu)) {
          return;
    }

    if (!IS_MMIO_MAP_VALID(dev_idx, package_num)) {
        SEP_PRINT_DEBUG("unc_mmio_Read_PMU_Data: device %d no virtual address mapping available for this device, return\n", dev_idx);
        return;
    }

    if (!pecb) {
        return;
    }
    num_events = ECB_num_events(pecb);
    start_index = DRV_CONFIG_emon_unc_offset(pcfg_unc, cur_grp);
    virtual_addr = virtual_address_table[dev_idx][package_num];

    idx_w = ECB_operations_register_start(pecb, PMU_OPERATION_WRITE);
    FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_READ) {
        cur_ei = (S32)ECB_entries_group_index(pecb, idx);
        if (prev_ei == -1) {
            prev_ei = cur_ei;
        }
        if (cur_ei != prev_ei) {
            sub_evt_index++;
            if (sub_evt_index == num_events) {
                sub_evt_index = 0;
            }
            prev_ei = cur_ei;
        }
        if (ECB_entries_pci_id_offset(pecb,idx) > DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(&ECB_pcidev_entry_node(pecb))) {
            offset_delta =  ECB_entries_pci_id_offset(pecb,idx) -
                              DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(&ECB_pcidev_entry_node(pecb));
        }
        else {
            offset_delta =  ECB_entries_pci_id_offset(pecb,idx);
        }

        if ((ECB_entries_counter_type(pecb,idx) == PROG_FREERUN_COUNTER) &&
            (ECB_entries_event_id_index_local(pecb,idx) == 0)) {
            *(U64*)(long)(virtual_addr + ECB_entries_reg_id(pecb,idx_w)) = ECB_entries_reg_value(pecb,idx_w);
            control_node.u.bit_field = ECB_entries_reg_value(pecb,idx_w);
            event_code = (U32)control_node.u.bits.event_select; 
            idx_w++;
        }

        if ((ECB_entries_event_scope(pecb,idx) == PACKAGE_EVENT) ||
            (ECB_entries_event_scope(pecb,idx) == SYSTEM_EVENT)) {
            j = start_index +  ECB_entries_group_index(pecb, idx) +
                ECB_entries_emon_event_id_index_local(pecb,idx) +
                sub_evt_index*num_packages*LWPMU_DEVICE_num_units(&devices[dev_idx])+
                package_num * LWPMU_DEVICE_num_units(&devices[dev_idx]);
            if (IS_64BIT((U64)(ECB_entries_max_bits(pecb,idx)))) {
                if (ECB_entries_counter_type(pecb,idx) == PROG_FREERUN_COUNTER) {
                    do {
                        if (counter > EVENT_COUNTER_MAX_TRY) {
                            break;
                        }
                        tmp_value = SYS_MMIO_Read64(virtual_addr, offset_delta);
                        counter++;
                    } while (event_code != (tmp_value >>60));
                }
                tmp_value = SYS_MMIO_Read64(virtual_addr, offset_delta);
            }
            else {
                tmp_value = readl((U32*)(long)(virtual_addr + offset_delta));
            }
            tmp_value &= (U64)ECB_entries_max_bits(pecb,idx);
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
            SEP_PRINT_DEBUG("unc_mmio_Read_PMU_Data cpu=%d, ei=%d, eil=%d, MSR=0x%x, j=0x%d, si=0x%d, value=0x%llx\n",
                this_cpu, ECB_entries_event_id_index(pecb, idx), ECB_entries_emon_event_id_index_local(pecb,idx),
                ECB_entries_reg_id(pecb,idx),j, start_index, buffer[j]);
                package_event_count++;
        }
    } END_FOR_EACH_REG_UNC_OPERATION;

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn unc_mmio_Initialize(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Do the mapping of the physical address (to do the invalidates in the TLB)
 *           NOTE: this should never be done with SMP call
 *
 */
static VOID
unc_mmio_Initialize (
     PVOID   param
)
{
    DRV_PCI_DEVICE_ENTRY_NODE  dpden;
    U32                        pci_address;
    U32                        bar_lo;
    U64                        next_bar_offset;
    U64                        bar_hi;
    U64                        physical_address;
    U64                        final_bar;
    U32                        dev_idx   = *((U32*)param);
    U32                        cur_grp  = LWPMU_DEVICE_cur_group(&devices[(dev_idx)]);
    ECB                        pecb     = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];
    U32                        busno    = 0;
    U32                        dev_node;

    U32                        page_len = 4096;         // 4K

    if (!pecb) {
        return;
    }
    dev_node = ECB_dev_node(pecb);

    if (IS_MMIO_MAP_VALID(dev_idx, 0)) {
        SEP_PRINT_DEBUG("init:  device[%d] node %d already mapped\n", dev_idx, dev_node);
        return;    // already mapped
    }

    dpden = ECB_pcidev_entry_node(pecb);

    // use busno found from topology scan if available
    // otherwise use the default one 
    if (UNC_COMMON_Get_Device_Busno(0, dev_node, &busno) == OS_SUCCESS) {
        DRV_PCI_DEVICE_ENTRY_bus_no(&dpden) = busno;
    }

    pci_address = FORM_PCI_ADDR(DRV_PCI_DEVICE_ENTRY_bus_no(&dpden),
                                DRV_PCI_DEVICE_ENTRY_dev_no(&dpden),
                                DRV_PCI_DEVICE_ENTRY_func_no(&dpden),
                                DRV_PCI_DEVICE_ENTRY_bar_offset(&dpden));

    bar_lo      = PCI_Read_Ulong(pci_address);
    next_bar_offset     = DRV_PCI_DEVICE_ENTRY_bar_offset(&dpden) + NEXT_ADDR_OFFSET;

    pci_address         = FORM_PCI_ADDR(DRV_PCI_DEVICE_ENTRY_bus_no(&dpden),
                                        DRV_PCI_DEVICE_ENTRY_dev_no(&dpden),
                                        DRV_PCI_DEVICE_ENTRY_func_no(&dpden),
                                        next_bar_offset);
    bar_hi              = PCI_Read_Ulong(pci_address);
    final_bar = (bar_hi << NEXT_ADDR_SHIFT) | bar_lo;
    final_bar &= DRV_PCI_DEVICE_ENTRY_bar_mask(&dpden);

    DRV_PCI_DEVICE_ENTRY_bar_address(&ECB_pcidev_entry_node(pecb)) = final_bar;
    physical_address     = DRV_PCI_DEVICE_ENTRY_bar_address(&ECB_pcidev_entry_node(pecb))
                                 + DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(&ECB_pcidev_entry_node(pecb));

    virtual_address_table[dev_idx][0] = (U64)(long)ioremap_nocache(physical_address,page_len);

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn unc_mmio_fpga_Initialize(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Do the mapping of the physical address (to do the invalidates in the TLB)
 *           NOTE: this should never be done with SMP call
 *
 */
static VOID
unc_mmio_fpga_Initialize (
     PVOID   param
)
{
    struct pci_dev  *pdev        = NULL;
    U64              phys_addr;
    PVOID            virt_addr;
    U64              len;
    U64              dfh;
    U32              id;
    U32              gb_dev_index;
    U32              offset      = 0;
    S32              next_offset = -1;
    U32              dev_idx     = *((U32*)param);
    U32              cur_grp     = LWPMU_DEVICE_cur_group(&devices[(dev_idx)]);
    ECB              pecb        = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];
    U32              package_num = 0;
    DRV_PCI_DEVICE_ENTRY_NODE  dpden;

    if (!pecb) {
        return;
    }

    if (IS_MMIO_MAP_VALID(dev_idx, 0)) {
        SEP_PRINT_DEBUG("init:  device[%d] node already mapped\n", dev_idx);
        return;    // already mapped
    }

    dpden = ECB_pcidev_entry_node(pecb);

    if (DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(&dpden)) {
        offset = DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(&dpden);
        gb_dev_index = DRV_PCI_DEVICE_ENTRY_op_code(&dpden);
    }

    pdev = pci_get_device(DRV_IS_PCI_VENDOR_ID_INTEL, DRV_PCI_DEVICE_ENTRY_device_id(&dpden), NULL); 
    while (pdev != NULL) {
        phys_addr = pci_resource_start(pdev, DRV_PCI_DEVICE_ENTRY_bar_num(&dpden));
        len = pci_resource_len(pdev, DRV_PCI_DEVICE_ENTRY_bar_num(&dpden));
        if ((DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(&dpden) == 0) && (package_num == 0)) {
            virt_addr = ioremap_nocache(phys_addr, len);
            while (next_offset != 0) {
                dfh = SYS_MMIO_Read64((U64)virt_addr, offset);
                next_offset = (U32)((dfh >> 16) & 0xffffff);
                id = (U32)(dfh & 0xfff);
                if (offset && (id == DRV_PCI_DEVICE_ENTRY_feature_id(&dpden))) {
                    break;
                }
                offset += next_offset;
            }
            iounmap(virt_addr);
        }
        phys_addr += offset;
        if (!DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(&dpden) ||
            (DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(&dpden) && fpga_gb_dev_valid[gb_dev_index][package_num])) {
            virtual_address_table[dev_idx][package_num] = (U64)(long)ioremap_nocache(phys_addr,DRV_PCI_DEVICE_ENTRY_size(&dpden));
        }
        package_num++;
        pdev = pci_get_device(DRV_IS_PCI_VENDOR_ID_INTEL, DRV_PCI_DEVICE_ENTRY_device_id(&dpden), pdev);
    }

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn unc_mmio_Destroy(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Invalidate the entry in TLB of the physical address
 *           NOTE: this should never be done with SMP call
 *
 */
static VOID
unc_mmio_Destroy (
     PVOID   param
)
{
    U32            dev_idx      = *((U32*)param);
    U32            i;

    for (i = 0; i < num_packages; i++) {
        if (virtual_address_table[dev_idx][i]) {
            iounmap((PVOID)(UIOP)virtual_address_table[dev_idx][i]);
            virtual_address_table[dev_idx][i] = 0;
        }
    }
    return;
}


/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  unc_mmio_dispatch =
{
    .init                     = unc_mmio_Initialize,
    .fini                     = unc_mmio_Destroy,
    .write                    = unc_mmio_Write_PMU,
    .freeze                   = unc_mmio_Disable_PMU,
    .restart                  = unc_mmio_Enable_PMU,
    .read_data                = unc_mmio_Read_PMU_Data,
    .check_overflow           = NULL,
    .swap_group               = NULL,
    .read_lbrs                = NULL,
    .cleanup                  = UNC_COMMON_Dummy_Func,
    .hw_errata                = NULL,
    .read_power               = NULL,
    .check_overflow_errata    = NULL,
    .read_counts              = NULL,
    .check_overflow_gp_errata = NULL,
    .read_ro                  = NULL,
    .platform_info            = NULL,
    .trigger_read             = unc_mmio_Trigger_Read,
    .scan_for_uncore          = NULL
};

DISPATCH_NODE  unc_mmio_fpga_dispatch =
{
    .init                     = unc_mmio_fpga_Initialize,
    .fini                     = unc_mmio_Destroy,
    .write                    = unc_mmio_Write_PMU,
    .freeze                   = unc_mmio_Disable_PMU,
    .restart                  = unc_mmio_Enable_PMU,
    .read_data                = unc_mmio_Read_PMU_Data,
    .check_overflow           = NULL,
    .swap_group               = NULL,
    .read_lbrs                = NULL,
    .cleanup                  = UNC_COMMON_Dummy_Func,
    .hw_errata                = NULL,
    .read_power               = NULL,
    .check_overflow_errata    = NULL,
    .read_counts              = NULL,
    .check_overflow_gp_errata = NULL,
    .read_ro                  = NULL,
    .platform_info            = NULL,
    .trigger_read             = unc_mmio_Trigger_Read,
    .scan_for_uncore          = NULL
};
