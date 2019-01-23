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

extern UNCORE_TOPOLOGY_INFO_NODE        uncore_topology;
extern PLATFORM_TOPOLOGY_PROG_NODE      platform_topology_prog_node;
extern FPGA_GB_DEV_NODE                 fpga_gb_dev_node;
extern U64                             *read_counter_info;
extern U8                              *fpga_gb_dev_valid[];

struct pci_bus                        **unc_package_to_bus_map;
#define PCI_INVALID_VALUE               0xFFFFFFFF

#define GET_PACKAGE_NUM(device_type, cpu)         (((device_type) == DRV_SINGLE_INSTANCE)? 0 : core_to_package_map[cpu])

/************************************************************/
/*
 * unc common Dispatch functions
 *
 ************************************************************/
extern  void
UNC_COMMON_Dummy_Func (
    PVOID  param
)
{
    return;
}

/************************************************************/
/*
 * UNC common PCI  based API
 *
 ************************************************************/

/*!
 * @fn          OS_STATUS UNC_COMMON_Do_Bus_to_Socket_Map(VOID)
 *
 * @brief       This code discovers which package's data is read off of which bus.
 *
 * @param       None
 *
 * @return      OS_STATUS
 *
 * <I>Special Notes:</I>
 *     This probably will move to the UBOX once that is programmed.
 */
OS_STATUS
UNC_COMMON_Do_Bus_to_Socket_Map(
    U32 uncore_did,
    U32 dev_node,
    U32 busno
)
{
    struct pci_dev  *pdev        = NULL;
    U32              package_num = 0;
    U32              dev         = 0;

    if (unc_package_to_bus_map == NULL) {
        unc_package_to_bus_map = CONTROL_Allocate_Memory(num_packages * MAX_DEVICES * sizeof(struct pci_bus*));
        if (unc_package_to_bus_map == NULL) {
            SEP_PRINT_DEBUG("UNC_COMMON_Do_Bus_to_Socket_Map allocated NULL by CONTROL_Allocate_Memory\n");
            return OS_NO_MEM;
        }
        for (dev = 0; dev < (num_packages * MAX_DEVICES); dev++) {
            unc_package_to_bus_map[dev] = 0;
        }
    }

    pdev = pci_get_device(DRV_IS_PCI_VENDOR_ID_INTEL, uncore_did, NULL);
    while (pdev != NULL) {
        if (pdev->bus->number == busno) {
            SEP_PRINT_DEBUG("BUS MAP package=%d device_id=0x%x dev_node=%d bus=0x%x\n", package_num, uncore_did, dev_node, pdev->bus->number);
            break;
        }
        pdev = pci_get_device(PCI_VENDOR_ID_INTEL, uncore_did, pdev);
    }
    if (!pdev) {
        return OS_FAULT;
    }

    while (package_num < num_packages && unc_package_to_bus_map[package_num*MAX_DEVICES+dev_node] != NULL) {
        if (busno == unc_package_to_bus_map[package_num*MAX_DEVICES+dev_node]->number) {
            break;
        }
        package_num++;
    }

    if (package_num < num_packages) {
        unc_package_to_bus_map[package_num*MAX_DEVICES+dev_node] = pdev->bus;
    }
  
    else {
        return OS_FAULT;
    }
    return OS_SUCCESS;
}

/*!
 * @fn          OS_STATUS UNC_COMMON_Get_Device_Busno(U32  package_num, U32  dev_node, U32 *busno) 
 *
 * @brief       This function will return the the bus number for a selected PCI device for a selected package.
 *              The bus number should be detected by the scan function and record in the unc_package_to_bus_map.
 *
 * @param       package_num    package number
 * @param       dev_node       device index
 * @param       busno          retuen value for bus number
 *
 * @return      OS_STATUS
 *
 */
OS_STATUS
UNC_COMMON_Get_Device_Busno(
    U32  package_num,
    U32  dev_node,
    U32 *busno
)
{
    if ((unc_package_to_bus_map != NULL) &&
        (package_num < num_packages)     && 
        (unc_package_to_bus_map[package_num*MAX_DEVICES+dev_node] != NULL)) {
        *busno = unc_package_to_bus_map[package_num*MAX_DEVICES+dev_node]->number;
        return OS_SUCCESS;
    }
    return OS_FAULT;
}
    



/*!
 * @fn         extern VOID UNC_COMMON_PCI_Clean_Up(PVOID)
 *
 * @brief      clear out out programming
 *
 * @param      None
 *
 * @return     None
 */
extern void
UNC_COMMON_PCI_Clean_Up (
    VOID   *param
)
{
    if (unc_package_to_bus_map) {
        unc_package_to_bus_map = CONTROL_Free_Memory(unc_package_to_bus_map);
    }

    return;
}


/*!
 * @fn          static VOID UNC_COMMON_PCI_Scan_For_Uncore(VOID*)
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
UNC_COMMON_PCI_Scan_For_Uncore(
    PVOID           param,
    U32             dev_node,
    DEVICE_CALLBACK callback
)
{
    U32                        pci_address;
    U32                        device_id;
    U32                        value;
    U32                        vendor_id;
    U32                        busno;
    U32                        j, k, l;
    U32                        device_found = 0;

    for (busno = 0; busno < 256; busno++) {
         for (j = 0; j < MAX_PCI_DEVNO; j++) {
             if (!(UNCORE_TOPOLOGY_INFO_pcidev_valid(&uncore_topology, dev_node, j))) {
                 continue;
             }
             for (k = 0; k < MAX_PCI_FUNCNO; k++) {
                 if (!(UNCORE_TOPOLOGY_INFO_pcidev_is_devno_funcno_valid(&uncore_topology,dev_node,j,k))) {
                     continue;
                 }
                 device_found = 0;
                 pci_address = FORM_PCI_ADDR(busno, j, k, 0);
                 value = PCI_Read_Ulong(pci_address);
                 CONTINUE_IF_NOT_GENUINE_INTEL_DEVICE(value, vendor_id, device_id);
                 SEP_PRINT_DEBUG("Uncore device ID = 0x%x\n",device_id);

                 for (l = 0; l <  UNCORE_TOPOLOGY_INFO_num_deviceid_entries(&uncore_topology, dev_node); l++) {
                     if (UNCORE_TOPOLOGY_INFO_deviceid(&uncore_topology, dev_node, l) == device_id) {
                         device_found = 1;
                         break;
                     }
                 }
                 if (device_found) {
                     if (UNC_COMMON_Do_Bus_to_Socket_Map(device_id, dev_node, busno) == OS_SUCCESS) {
                         UNCORE_TOPOLOGY_INFO_pcidev_is_found_in_platform(&uncore_topology, dev_node, j, k) = 1;
                         SEP_PRINT_DEBUG("found device 0x%x at B:D:F = %d:%d:%d\n", device_id, busno,j,k);
                     }
                 }
             }
         }
    }

    return;
}

/*!
 * @fn          extern VOID UNC_COMMON_Get_Platform_Topology()
 *
 * @brief       This function will walk through the platform registers to retrieve information and calculate the bus no.
 *              Reads appropriate pci_config regs and populates the PLATFORM_TOPOLOGY_PROG_NODE structure with the reg value.
 *
 * @param       U32             dev_node - Device no.
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 *                   device_num corresponds to Memory controller
 *                   func_num  corresponds to Channel number
 *                   reg_offset corresponds to dimm slot         
 */
extern VOID
UNC_COMMON_Get_Platform_Topology (
    U32             dev_node
)
{
    U32                 pci_address;
    U32                 intel_device_address;
    U32                 num_registers     = 0;
    U32                 device_index      = 0;
    U32                 bus_num           = 0;
    U32                 i                 = 0;
    U32                 func_num          = 0;
    U32                 num_pkgs          = num_packages;
    U32                 bus_map_index     = 0;
    U32                 device_num        = 0;
    U32                 reg_offset        = 0;
    U32                 len               = 0;
    U64                 reg_value         = 0;
    U32                 device_value      = 0;
    U64                 reg_mask          = 0;
    U32                 vendor_id;
    U32                 device_id;
    U32                 valid;

    PLATFORM_TOPOLOGY_REG        topology_regs = NULL;

    PLATFORM_TOPOLOGY_PROG_topology_device_prog_valid(&platform_topology_prog_node, dev_node) = 1;

    if (num_packages > MAX_PACKAGES) {
          SEP_PRINT_ERROR("UNC_COMMON_Get_Platform_Topology: num_packages %d exceeds MAX_PACKAGE %d, only retrieve info for %d packages\n",
                           num_packages, MAX_PACKAGES, MAX_PACKAGES);
          num_pkgs = MAX_PACKAGES;
    }

    num_registers     = PLATFORM_TOPOLOGY_PROG_topology_device_num_registers(&platform_topology_prog_node, dev_node);
    topology_regs     = PLATFORM_TOPOLOGY_PROG_topology_topology_regs(&platform_topology_prog_node, dev_node);
    device_index      = PLATFORM_TOPOLOGY_PROG_topology_device_device_index(&platform_topology_prog_node, dev_node);

    for (i = 0; i < num_pkgs; i++) {
        for (len = 0; len < num_registers; len++) {
            if (PLATFORM_TOPOLOGY_REG_reg_type(topology_regs, len)  == PMU_REG_PROG_MSR) {
                reg_value = SYS_Read_MSR(PLATFORM_TOPOLOGY_REG_reg_id(topology_regs, len));
                reg_mask = PLATFORM_TOPOLOGY_REG_reg_mask(topology_regs, len);
                PLATFORM_TOPOLOGY_REG_reg_value(topology_regs, len, i) = reg_value & reg_mask;
                SEP_PRINT_DEBUG("UNC_COMMON_PCI_Get_Platform_Topology read UNCORE_MSR_FREQUENCY 0x%x\n",PLATFORM_TOPOLOGY_REG_reg_id(topology_regs, len));
            }
            else {
                if (unc_package_to_bus_map == NULL) {
                    continue;
                }
                bus_map_index = i*MAX_DEVICES+device_index;
                if (!unc_package_to_bus_map[bus_map_index]) {
                    continue;
                }
                bus_num              = unc_package_to_bus_map[bus_map_index]->number;
                device_num           = PLATFORM_TOPOLOGY_REG_device(topology_regs, len);
                func_num             = PLATFORM_TOPOLOGY_REG_function(topology_regs, len);
                reg_offset           = PLATFORM_TOPOLOGY_REG_reg_id(topology_regs, len);
                pci_address          = FORM_PCI_ADDR(bus_num, device_num, func_num, reg_offset);
                intel_device_address = FORM_PCI_ADDR(bus_num, device_num, func_num, 0);
                device_value         = PCI_Read_Ulong(intel_device_address);
                CHECK_IF_GENUINE_INTEL_DEVICE(device_value, vendor_id, device_id, valid);
                if (!valid) {
                    PLATFORM_TOPOLOGY_REG_device_valid(topology_regs, len) = 0;
                }
                PLATFORM_TOPOLOGY_REG_reg_value(topology_regs, len, i) = PCI_Read_Ulong_Valid(pci_address, PCI_INVALID_VALUE);
            }
        }
        if (PLATFORM_TOPOLOGY_PROG_topology_device_scope(&platform_topology_prog_node, dev_node) == SYSTEM_EVENT) {
           break;
        }
    }
    return;
}

/*!
 * @fn          extern VOID UNC_COMMON_FPGA_GB_Scan()
 *
 * @brief       This function will walk through the FPGA AFUs and discover GB AFUs/features specified via XML.
 *
 * @param       U32             dev_node - Device no.
 *
 * @return      None
 */
extern VOID
UNC_COMMON_FPGA_GB_Scan(
    U32             dev_node
)
{
    struct pci_dev  *pdev = NULL;
    U64              phys_addr;
    PVOID            virt_addr;
    U64              len;
    U64              afu_id_l=0, afu_id_h=0;
    U64              dfh;
    U32              afu_offset      = 0, offset, feature_type;
    S32              next_afu_offset = -1,  next_feature_offset = -1;
    U16              feature_id = 0xffff;
    U32              package_num = 0;

    pdev = pci_get_device(DRV_IS_PCI_VENDOR_ID_INTEL, FPGA_GB_DEV_device_id(&fpga_gb_dev_node,dev_node), NULL);
    while (pdev != NULL) {
        phys_addr = pci_resource_start(pdev, FPGA_GB_DEV_bar_num(&fpga_gb_dev_node,dev_node));
        len = pci_resource_len(pdev, FPGA_GB_DEV_bar_num(&fpga_gb_dev_node,dev_node));
        SEP_PRINT_DEBUG("In %s dev_node: %d, PCIe device 0x%x BAR %d phys_addr: 0x%llx, len 0x%llx\n",
                          __FUNCTION__, dev_node, FPGA_GB_DEV_device_id(&fpga_gb_dev_node,dev_node),
                               FPGA_GB_DEV_bar_num(&fpga_gb_dev_node,dev_node), phys_addr, len);
        if (!phys_addr || !len) {
            SEP_PRINT_DEBUG("For FGPA GB returned ZERO address or length for PCIe device id 0x%x\n",
                        FPGA_GB_DEV_device_id(&fpga_gb_dev_node,dev_node));
            return;
        }
        virt_addr = ioremap_nocache(phys_addr, len);

        afu_offset = 0;
        next_afu_offset = -1;
        next_feature_offset = -1;
        // loop over AFUs
        while (next_afu_offset != 0) {
            dfh = SYS_MMIO_Read64((U64)virt_addr,  afu_offset);
            feature_type = dfh>>60 & 0xf; // it could be AFU (1), BBB (2) or Private Feature(3)
            if (1 == feature_type) { // if AFU - get next_afu_offset
                next_afu_offset = SYS_MMIO_Read64((U64)virt_addr, afu_offset + 0x18) & 0xffffff;
            }
            else {
                next_afu_offset = 0;
            }
            SEP_PRINT_DEBUG("next AFU offset: 0x%x\n", next_afu_offset);

            next_feature_offset = -1;
            offset = afu_offset;
            // loop over features within AFU
            while (next_feature_offset != 0) {
                dfh = SYS_MMIO_Read64((U64)virt_addr,  offset);
                next_feature_offset = dfh>>16 & 0xffffff;
                feature_type = dfh>>60 & 0xf;
                SEP_PRINT_DEBUG("dfh: 0x%llx, feature_type: %u, offset: 0x%x, next_feature_offset: 0x%x\n",
                                 dfh, feature_type, offset, next_feature_offset);

                if (1 == feature_type || 2 == feature_type || // either AFU or BBB then it has UID
                    (0 == feature_type && dfh == 0x0L)) { // this is hack to support AFU with zero (!) dfh
                    afu_id_l = SYS_MMIO_Read64((U64)virt_addr,  offset + 0x8);
                    afu_id_h = SYS_MMIO_Read64((U64)virt_addr,  offset + 0x10);
                    SEP_PRINT_DEBUG("AFU or BBB: id_l: 0x%llx, id_h: 0x%llx\n", afu_id_l, afu_id_h);

                    // match for BBB or for AFU itself
                    if ((afu_id_l == FPGA_GB_DEV_afu_id_low(&fpga_gb_dev_node,dev_node)) &&
                        (afu_id_h == FPGA_GB_DEV_afu_id_high(&fpga_gb_dev_node,dev_node)) &&
                        (FPGA_GB_DEV_feature_id(&fpga_gb_dev_node,dev_node) < 0 )) {
                        FPGA_GB_DEV_valid(&fpga_gb_dev_node,dev_node) = 1;
                        FPGA_GB_DEV_feature_offset(&fpga_gb_dev_node,dev_node) = offset;
                        fpga_gb_dev_valid[dev_node][package_num] = 1;
                        SEP_PRINT_DEBUG("\t\tBBB match! will use offset: 0x%x \n", offset);

                        goto the_end;
                    }
                }
                else {   /* match for Private Feature*/
                    feature_id = dfh & 0xfff;

                    SEP_PRINT_DEBUG("Private Feature: feature_id:  0x%x\n", feature_id);

                    if ((FPGA_GB_DEV_feature_id(&fpga_gb_dev_node,dev_node) >= 0) &&
                        (feature_id == (U16) FPGA_GB_DEV_feature_id(&fpga_gb_dev_node,dev_node)) &&
                        (afu_id_l == FPGA_GB_DEV_afu_id_low(&fpga_gb_dev_node,dev_node)) &&
                        (afu_id_h == FPGA_GB_DEV_afu_id_high(&fpga_gb_dev_node,dev_node))) {
                        
                        FPGA_GB_DEV_valid(&fpga_gb_dev_node,dev_node) = 1;
                        FPGA_GB_DEV_feature_offset(&fpga_gb_dev_node,dev_node) = offset;
                        fpga_gb_dev_valid[dev_node][package_num] = 1;
                        SEP_PRINT_DEBUG("\t\tPrivate Feature match\n");

                        goto the_end;
                    }
                }
                offset += next_feature_offset;
             }
             afu_offset += next_afu_offset;
        }

the_end:
        if (FPGA_GB_DEV_valid(&fpga_gb_dev_node,dev_node)) {
            if (next_feature_offset) {
                FPGA_GB_DEV_feature_len(&fpga_gb_dev_node,dev_node) = next_feature_offset;
            }
            else {
                if (next_afu_offset) {
                    FPGA_GB_DEV_feature_len(&fpga_gb_dev_node,dev_node) = next_afu_offset;
                }
                else {
                    FPGA_GB_DEV_feature_len(&fpga_gb_dev_node,dev_node) = len - offset;
                }
            }
            SEP_PRINT_DEBUG("\t\toffset: 0x%x, len: %d\n",
                    offset, FPGA_GB_DEV_feature_len(&fpga_gb_dev_node,dev_node));
        }

        iounmap(virt_addr);
        package_num++;
        pdev = pci_get_device(DRV_IS_PCI_VENDOR_ID_INTEL, FPGA_GB_DEV_device_id(&fpga_gb_dev_node,dev_node), pdev);
    }

    return;
}

/************************************************************/
/*
 * UNC common MSR  based API
 *
 ************************************************************/

/*!
 * @fn         VOID UNC_COMMON_MSR_Clean_Up(PVOID)
 *
 * @brief      clear out out programming
 *
 * @param      None
 *
 * @return     None
 */
VOID
UNC_COMMON_MSR_Clean_Up (
    VOID   *param
)
{
    U32 dev_idx = *((U32*)param);

    FOR_EACH_REG_ENTRY_UNC(pecb, dev_idx, i) {
        if (ECB_entries_clean_up_get(pecb,i)) {
            SYS_Write_MSR(ECB_entries_reg_id(pecb,i), 0LL);
        }
    } END_FOR_EACH_REG_ENTRY_UNC;

    return;
}

