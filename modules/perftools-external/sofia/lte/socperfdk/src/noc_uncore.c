/* ***********************************************************************************************

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2013-2014 Intel Corporation. All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution
  in the file called LICENSE.GPL.

  BSD LICENSE

  Copyright(c) 2013-2014 Intel Corporation. All rights reserved.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  ***********************************************************************************************
*/


#include "lwpmudrv_defines.h"
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/fs.h>

#include "lwpmudrv_types.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"

#include "inc/socperfdrv.h"
#include "inc/ecb_iterators.h"
#include "inc/pci.h"
#include "inc/control.h"
#include "inc/noc_uncore.h"


extern LWPMU_DEVICE   device_uncore;
static U32            counter_overflow[SOC_NOC_COUNTER_MAX_COUNTERS];
static U64            register_virtual_address = 0;


/*!
 * @fn          static ULONG read_From_Register(U64  bar_virtual_address,
                                                U64  mmio_offset,
                                                U32 *data_val)
 *
 * @brief       Reads register programming info
 *
 * @param       bar_virtual_address - memory address
 *              mmio_offset         - offset of the register
 *              data_val            - register value read
 *
 * @return      data from the counter register
 *
 * <I>Special Notes:</I>
 */
static void
read_From_Register (
    U64  bar_virtual_address,
    U64  mmio_offset,
    U32 *data_val
)
{

    if (data_val) {
        *data_val = readl((U32*)((char*)(UIOP)(bar_virtual_address) + mmio_offset));
    }
    return;
}


/*!
 * @fn          static ULONG write_To_Register(U64  bar_virtual_address,
                                               U64  mmio_offset,
                                               U32  value)
 *
 * @brief       Write register programming info
 *
 * @param       bar_virtual_address - memory address
 *              mmio_offset         - offset of the register
 *              value               - register value to be written
 *
 * @return      none
 *
 * <I>Special Notes:</I>
 */
static void
write_To_Register (
    U64   bar_virtual_address,
    U64   mmio_offset,
    ULONG value
)
{
    U32 read_reg = 0;

    writel(value, (U32*)(((char*)(UIOP)bar_virtual_address)+mmio_offset));
    read_From_Register(bar_virtual_address, mmio_offset, &read_reg);

    return;
}


/*!
 * @fn          static VOID uncore_Reset_Counters(U32 dev_idx)
 *
 * @brief       Reset counters
 *
 * @param       dev_idx - device index
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
static VOID
uncore_Reset_Counters (
   VOID
)
{
    if (register_virtual_address) {
        write_To_Register(register_virtual_address, SOC_NOC_MAIN_CTRL_OFFSET, 0x0);
        write_To_Register(register_virtual_address, SOC_NOC_MAIN_CTRL_OFFSET, SOC_NOC_MAIN_CTRL_STAT_ENABLE);
    }

    return;
}


/*!
 * @fn          static VOID uncore_Write_PMU(VOID*)
 *
 * @brief       Initial write of PMU registers
 *              Walk through the entries and write the value of the register accordingly.
 *              When current_group = 0, then this is the first time this routine is called,
 *
 * @param       param - device index
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
static VOID
uncore_Write_PMU (
    VOID  *param
)
{
    ECB                        pecb;
    U64                        physical_address;
    U32                        i               = 0;
    U32                        cur_grp;

    if (device_uncore == NULL) {
        SOCPERF_PRINT_ERROR("ERROR: NULL device_uncore!\n");
        return;
    }
    cur_grp  = LWPMU_DEVICE_cur_group(device_uncore);
    pecb     = (ECB)LWPMU_DEVICE_PMU_register_data(device_uncore)[cur_grp];
    if (pecb == NULL) {
        SOCPERF_PRINT_ERROR("ERROR: null pecb!\n");
        return;
    }

    // initialize the per-counter overflow numbers
    for (i = 0; i < SOC_NOC_COUNTER_MAX_COUNTERS; i++) {
        counter_overflow[i]                 = 0;
        socperf_pcb[0].last_uncore_count[i] = 0;
    }

    if (register_virtual_address == 0) {
        physical_address              = DRV_PCI_DEVICE_ENTRY_bar_address(&ECB_pcidev_entry_node(pecb));
        register_virtual_address      = (U64)(UIOP) ioremap_nocache(physical_address, 0x4000);
        SOCPERF_PRINT_DEBUG("Initialize: physical address=%llx virtual address=%llx\n", physical_address, register_virtual_address);
    }

    FOR_EACH_REG_ENTRY_UNC(pecb,dev_idx,idx) {
        write_To_Register(register_virtual_address, ECB_entries_reg_id(pecb,idx), ECB_entries_reg_value(pecb,idx));
        SOCPERF_PRINT_DEBUG("WRITEPMU virtual_address=%llx reg=%x, value=%x\n", register_virtual_address, ECB_entries_reg_id(pecb,idx), ECB_entries_reg_value(pecb,idx));
    } END_FOR_EACH_REG_ENTRY_UNC;

    return;
}

/*!
 * @fn         static VOID uncore_Enable_PMU(PVOID)
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
uncore_Enable_PMU (
    PVOID  param
)
{
    U32 data_val    = 0;

    if (register_virtual_address) {
        write_To_Register(register_virtual_address, SOC_NOC_MAIN_CTRL_OFFSET, SOC_NOC_MAIN_CTRL_STAT_ENABLE);
        write_To_Register(register_virtual_address, SOC_NOC_CFG_CTRL_OFFSET, SOC_NOC_CFG_CTRL_ENABLE);
        SOCPERF_PRINT_DEBUG("ENABLE virtual_address=%llx reg=0x240c, value=0x1\n", register_virtual_address);
        read_From_Register(register_virtual_address, SOC_NOC_CFG_CTRL_OFFSET, &data_val);
        SOCPERF_PRINT_DEBUG("ENABLE virtual_address=%llx reg=0x240c, value=0x%x\n", register_virtual_address, data_val);
    }

    return;
}


/*!
 * @fn         static VOID uncore_Disable_PMU(PVOID)
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
uncore_Disable_PMU (
    PVOID  param
)
{

    if (GLOBAL_STATE_current_phase(socperf_driver_state) == DRV_STATE_PREPARE_STOP) {
        if (register_virtual_address) {
            write_To_Register(register_virtual_address, SOC_NOC_CFG_CTRL_OFFSET, 0x0);
            SOCPERF_PRINT_DEBUG("DISABLE virtual_address=%llx reg=0x240c, value=0x0\n", register_virtual_address);
            iounmap((void*)(UIOP)(register_virtual_address));
            SOCPERF_PRINT_DEBUG("DISABLE Unmapping NOCBAR address=%x\n", register_virtual_address);
        }
        register_virtual_address = 0;
    }

    return;
}


/*!
 * @fn         static VOID uncore_Initialize(PVOID)
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
uncore_Initialize (
    VOID  *param
)
{

    return;
}


/*!
 * @fn         static VOID uncore_Clean_Up(PVOID)
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
uncore_Clean_Up (
    VOID   *param
)
{
    register_virtual_address = 0;
    return;
}



/* ------------------------------------------------------------------------- */
/*!
 * @fn uncore_Read_Data()
 *
 * @param    None
 *
 * @return   None     No return needed
 *
 * @brief    Read the counters
 *
 */
static VOID
uncore_Read_Data (
    PVOID data_buffer
)
{
    U32              event_id    = 0;
    U64             *data;
    int              data_index;
    U32              data_val    = 0;
    U32              data_val_hi = 0;
    U32              data_reg    = 0;
    U64              total_count = 0;
    U32              event_index = 0;
    U32              cur_grp;

    if (GLOBAL_STATE_current_phase(socperf_driver_state) == DRV_STATE_UNINITIALIZED ||
        GLOBAL_STATE_current_phase(socperf_driver_state) == DRV_STATE_IDLE          ||
        GLOBAL_STATE_current_phase(socperf_driver_state) == DRV_STATE_RESERVED      ||
        GLOBAL_STATE_current_phase(socperf_driver_state) == DRV_STATE_PREPARE_STOP  ||
        GLOBAL_STATE_current_phase(socperf_driver_state) == DRV_STATE_STOPPED) {
        SOCPERF_PRINT_ERROR("ERROR: RETURING EARLY from Read_Data\n");
        return;
    }
    cur_grp    = LWPMU_DEVICE_cur_group(device_uncore);
    data       = (U64*)data_buffer;
    data_index = 0;

    // Write GroupID
    data[data_index] = cur_grp + 1;
    // Increment the data index as the event id starts from zero
    data_index++;

    FOR_EACH_REG_ENTRY_UNC(pecb,dev_idx,idx) {
            data_reg           = idx;
            if (ECB_entries_reg_type(pecb,data_reg) == DATA) {
                read_From_Register(register_virtual_address,
                                   ECB_entries_reg_id(pecb,data_reg),
                                   &data_val);
                read_From_Register(register_virtual_address,
                                   ECB_entries_reg_id(pecb,data_reg)+0x14,
                                   &data_val_hi);
            total_count = data_val | data_val_hi << 16;
            if (total_count < socperf_pcb[0].last_uncore_count[event_index]) {
                    counter_overflow[event_index]++;
                SOCPERF_PRINT_DEBUG("Counter overflow detected\n");
                }
                socperf_pcb[0].last_uncore_count[event_index] = total_count;
            if (total_count == SOC_NOC_COUNTER_MAX_COUNT) {
               SOCPERF_PRINT_DEBUG("Resetting the counter\n");
               uncore_Reset_Counters();
            }
            total_count = total_count + counter_overflow[event_index]*SOC_NOC_COUNTER_MAX_COUNT;
                event_index++;
                data[data_index+event_id] = total_count;
            SOCPERF_PRINT_DEBUG("lo=%x hi=%x DATA[%d]=0x%x\n", data_val, data_val_hi, event_id, total_count);
                event_id++;
            }
        SOCPERF_PRINT_DEBUG("READ_DATA: virtual_address=%llx reg=%x, value=%x\n", register_virtual_address, ECB_entries_reg_id(pecb,idx), ECB_entries_reg_value(pecb,idx));
    } END_FOR_EACH_REG_ENTRY_UNC;

    return;
}




/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  noc_dispatch =
{
    uncore_Initialize,        // initialize
    NULL,                     // destroy
    uncore_Write_PMU,         // write
    uncore_Disable_PMU,       // freeze
    uncore_Enable_PMU,        // restart
    NULL,                     // read
    NULL,                     // check for overflow
    NULL,
    NULL,
    uncore_Clean_Up,
    NULL,
    NULL,
    NULL,
    NULL,                    // read counts
    NULL,
    NULL,
    NULL,
    NULL,
    uncore_Read_Data,
    NULL,
    NULL,
    NULL,
    NULL
};

