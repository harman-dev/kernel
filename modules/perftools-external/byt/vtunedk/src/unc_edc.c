/*COPYRIGHT**
    Copyright (C) 2005-2015 Intel Corporation.  All Rights Reserved.

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
#include "knl_unc_edc.h"
#include "hsx_unc_ubox.h"
#include "ecb_iterators.h"
#include "pebs.h"
#include "inc/pci.h"

extern U64           *read_counter_info;
extern DRV_CONFIG     pcfg;
static U32            edc_dids[MAX_PCI_DEVS];
 
#define INVALID_VALUE 0xFFFFFFFF

/*
 * device specific functions
 */


 /***********************************************************************
 *
 * dispatch function for KNL EDC UCLK
 *
 ***********************************************************************/

static DRV_BOOL
unc_edc_uclk_knl_is_Valid(
    U32  device_id
)
{
    if (device_id != KNL_EDC_UCLK_DID) {
        return FALSE;
    }
    return TRUE;
}

static DRV_BOOL
unc_edc_uclk_knl_is_Valid_For_Write(
    U32  device_id,
    U32  reg_id
)
{
    return unc_edc_uclk_knl_is_Valid(device_id);
}

static DRV_BOOL
unc_edc_uclk_knl_is_Unit_Ctl(
    U32  reg_id
)
{
    return (IS_THIS_KNL_BOX_EDC_UCLK_PCI_UNIT_CTL(reg_id));
}

static DRV_BOOL
unc_edc_uclk_knl_is_PMON_Ctl(
    U32  reg_id
)
{
    return (IS_THIS_KNL_EDC_UCLK_PCI_PMON_CTL(reg_id));
}

static VOID
unc_edc_uclk_knl_init_Device_IDs(
    void
)
{
    edc_dids[0] = KNL_EDC_UCLK_DID;
}

static U32*
unc_edc_uclk_knl_get_Device_IDs(
    U32 *num_devs
)
{
    *num_devs = KNL_EDC_NUM_DEVS;
    return &edc_dids[0];
}

DEVICE_CALLBACK_NODE  unc_edc_uclk_knl_callback = {
    unc_edc_uclk_knl_is_Valid,
    unc_edc_uclk_knl_is_Valid_For_Write,
    unc_edc_uclk_knl_is_Unit_Ctl,
    unc_edc_uclk_knl_is_PMON_Ctl
};

CALLBACK_NODE_LIN  unc_edc_uclk_knl_callback_lin = {
    unc_edc_uclk_knl_init_Device_IDs,
    unc_edc_uclk_knl_get_Device_IDs,
    unc_edc_uclk_knl_is_Unit_Ctl,
    unc_edc_uclk_knl_is_PMON_Ctl
};


static VOID
unc_edc_uclk_knl_Write_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Lin_Write_PMU(param, 
                          KNL_SOCKETID_UBOX_DID, 
                          KNL_UBOX_GLOBAL_CONTROL_MSR, 
                          RESET_EDC_CTRS,
                          &unc_edc_uclk_knl_callback);
    return;
}

static VOID
unc_edc_uclk_knl_Enable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Lin_Enable_PMU(param, 
                              KNL_UBOX_GLOBAL_CONTROL_MSR,
                              ENABLE_KNL_EDC_COUNTERS,
                              DISABLE_KNL_EDC_COUNTERS,
                              &unc_edc_uclk_knl_callback);
    return;
}

static VOID
unc_edc_uclk_knl_Disable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Lin_Disable_PMU(param, 
                               KNL_UBOX_GLOBAL_CONTROL_MSR,
                               ENABLE_KNL_EDC_COUNTERS,
                               DISABLE_KNL_EDC_COUNTERS,
                               &unc_edc_uclk_knl_callback);
    return;
}

static VOID
unc_edc_uclk_knl_Scan_For_Uncore(
    PVOID param
)
{
    UNC_COMMON_PCI_Lin_Scan_For_Uncore(param, UNCORE_TOPOLOGY_INFO_NODE_EDC_UCLK, &unc_edc_uclk_knl_callback_lin);
}


 /***********************************************************************
 *
 * dispatch function for KNL EDC ECLK
 *
 ***********************************************************************/

static DRV_BOOL
unc_edc_eclk_knl_is_Valid(
    U32  device_id
)
{
    if (device_id != KNL_EDC_ECLK_DID) {
        return FALSE;
    }
    return TRUE;
}

static DRV_BOOL
unc_edc_eclk_knl_is_Valid_For_Write(
    U32  device_id,
    U32  reg_id
)
{
    return unc_edc_eclk_knl_is_Valid(device_id);
}

static DRV_BOOL
unc_edc_eclk_knl_is_Unit_Ctl(
    U32  reg_id
)
{
    return (IS_THIS_KNL_BOX_EDC_ECLK_PCI_UNIT_CTL(reg_id));
}

static DRV_BOOL
unc_edc_eclk_knl_is_PMON_Ctl(
    U32  reg_id
)
{
    return (IS_THIS_KNL_EDC_ECLK_PCI_PMON_CTL(reg_id));
}

static VOID
unc_edc_eclk_knl_init_Device_IDs(
    void
)
{
    edc_dids[0] = KNL_EDC_ECLK_DID;
}

static U32*
unc_edc_eclk_knl_get_Device_IDs(
    U32 *num_devs
)
{
    *num_devs = KNL_EDC_NUM_DEVS;
    return &edc_dids[0];
}

DEVICE_CALLBACK_NODE  unc_edc_eclk_knl_callback = {
    unc_edc_eclk_knl_is_Valid,
    unc_edc_eclk_knl_is_Valid_For_Write,
    unc_edc_eclk_knl_is_Unit_Ctl,
    unc_edc_eclk_knl_is_PMON_Ctl
};

CALLBACK_NODE_LIN  unc_edc_eclk_knl_callback_lin = {
    unc_edc_eclk_knl_init_Device_IDs,
    unc_edc_eclk_knl_get_Device_IDs,
    unc_edc_eclk_knl_is_Unit_Ctl,
    unc_edc_eclk_knl_is_PMON_Ctl
};


static VOID
unc_edc_eclk_knl_Write_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Lin_Write_PMU(param, 
                          KNL_SOCKETID_UBOX_DID, 
                          KNL_UBOX_GLOBAL_CONTROL_MSR, 
                          RESET_EDC_CTRS,
                          &unc_edc_eclk_knl_callback);
    return;
}

static VOID
unc_edc_eclk_knl_Enable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Lin_Enable_PMU(param, 
                              KNL_UBOX_GLOBAL_CONTROL_MSR,
                              ENABLE_KNL_EDC_COUNTERS,
                              DISABLE_KNL_EDC_COUNTERS,
                              &unc_edc_eclk_knl_callback);
    return;
}

static VOID
unc_edc_eclk_knl_Disable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Lin_Disable_PMU(param, 
                               KNL_UBOX_GLOBAL_CONTROL_MSR,
                               ENABLE_KNL_EDC_COUNTERS,
                               DISABLE_KNL_EDC_COUNTERS,
                               &unc_edc_eclk_knl_callback);
    return;
}

static VOID
unc_edc_eclk_knl_Scan_For_Uncore(
    PVOID param
)
{
    UNC_COMMON_PCI_Lin_Scan_For_Uncore(param, UNCORE_TOPOLOGY_INFO_NODE_EDC_ECLK, &unc_edc_eclk_knl_callback_lin);
}


/*************************************************/

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE knl_edc_uclk_dispatch =
{
    NULL,                              // initialize
    NULL,                              // destroy
    unc_edc_uclk_knl_Write_PMU,        // write
    unc_edc_uclk_knl_Disable_PMU,      // freeze
    unc_edc_uclk_knl_Enable_PMU,       // restart
    UNC_COMMON_PCI_Lin_Read_PMU_Data,  // read
    NULL,                              // check for overflow
    NULL,                              // swap_group
    NULL,                              // read_lbrs
    UNC_COMMON_PCI_Clean_Up,           // cleanup
    NULL,                              // hw_errata
    NULL,                              // read_power
    NULL,                              // check overflow errata
    UNC_COMMON_PCI_Lin_Read_Counts,    // read_counts
    NULL,                              // check overflow gp errata
    NULL,                              // read_ro
    NULL,                              // platform info
    NULL,
    unc_edc_uclk_knl_Scan_For_Uncore   // scan for uncore
};

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE knl_edc_eclk_dispatch =
{
    NULL,                              // initialize
    NULL,                              // destroy
    unc_edc_eclk_knl_Write_PMU,        // write
    unc_edc_eclk_knl_Disable_PMU,      // freeze
    unc_edc_eclk_knl_Enable_PMU,       // restart
    UNC_COMMON_PCI_Lin_Read_PMU_Data,  // read
    NULL,                              // check for overflow
    NULL,                              // swap_group
    NULL,                              // read_lbrs
    UNC_COMMON_PCI_Clean_Up,           // cleanup
    NULL,                              // hw_errata
    NULL,                              // read_power
    NULL,                              // check overflow errata
    UNC_COMMON_PCI_Lin_Read_Counts,    // read_counts
    NULL,                              // check overflow gp errata
    NULL,                              // read_ro
    NULL,                              // platform info
    NULL,
    unc_edc_eclk_knl_Scan_For_Uncore   // scan for uncore
};

