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
#include <linux/fs.h>
#include <asm/msr.h>
#include <linux/ptrace.h>

#include "lwpmudrv_types.h"
#include "rise_errors.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv.h"
#include "core2.h"
#include "silvermont.h"
#include "perfver4.h"
#include "valleyview_sochap.h"
#include "snb_unc_gt.h"
#if defined(SEP_ENABLE_NDA_CPUS)
#include "haswellunc_sa.h"
#endif
#if defined(BUILD_CHIPSET)
#include "chap.h"
#endif
#include "utility.h"
#if defined(BUILD_CHIPSET)
#include "lwpmudrv_chipset.h"
#include "gmch.h"
#endif

volatile int config_done;
extern  DISPATCH_NODE   unc_msr_dispatch;
extern  DISPATCH_NODE   unc_pci_dispatch;
extern  DISPATCH_NODE   unc_mmio_dispatch;
extern  DISPATCH_NODE   unc_mmio_fpga_dispatch;
extern  DISPATCH_NODE   unc_power_dispatch;

#if defined(BUILD_CHIPSET)
extern CHIPSET_CONFIG pma;
#endif

extern DRV_BOOL
UTILITY_down_read_mm (
    struct task_struct *p
)
{
#ifdef SUPPORTS_MMAP_READ
    mmap_down_read(p->mm);
#else
    down_read((struct rw_semaphore *) &p->mm->mmap_sem);
#endif
    return TRUE;
}

extern VOID
UTILITY_up_read_mm (
    struct task_struct *p
)
{
#ifdef SUPPORTS_MMAP_READ
    mmap_up_read(p->mm);
#else
    up_read((struct rw_semaphore *) &p->mm->mmap_sem);
#endif

    return;
}

extern VOID
UTILITY_Read_TSC (
    U64* pTsc
)
{
    rdtscll(*(pTsc));

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       VOID UTILITY_Read_Cpuid
 *
 * @brief    executes the cpuid_function of cpuid and returns values
 *
 * @param  IN   cpuid_function
 *         OUT  rax  - results of the cpuid instruction in the
 *         OUT  rbx  - corresponding registers
 *         OUT  rcx
 *         OUT  rdx
 *
 * @return   none
 *
 * <I>Special Notes:</I>
 *              <NONE>
 *
 */
extern VOID
UTILITY_Read_Cpuid (
    U64   cpuid_function,
    U64  *rax_value,
    U64  *rbx_value,
    U64  *rcx_value,
    U64  *rdx_value
)
{
    U32 function = (U32) cpuid_function;
    U32 *eax     = (U32 *) rax_value;
    U32 *ebx     = (U32 *) rbx_value;
    U32 *ecx     = (U32 *) rcx_value;
    U32 *edx     = (U32 *) rdx_value;

    *eax = function;

    __asm__("cpuid"
            : "=a" (*eax),
              "=b" (*ebx),
              "=c" (*ecx),
              "=d" (*edx)
            : "a"  (function),
              "b"  (*ebx),
              "c"  (*ecx),
              "d"  (*edx));

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       VOID UTILITY_Configure_CPU
 *
 * @brief    Reads the CPU information from the hardware
 *
 * @param    param   dispatch_id -  The id of the dispatch table.
 *
 * @return   Pointer to the correct dispatch table for the CPU architecture
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
extern  DISPATCH
UTILITY_Configure_CPU (
    U32 dispatch_id
)
{
    DISPATCH     dispatch = NULL;
    switch (dispatch_id) {
        case 1:
            SEP_PRINT_DEBUG("Set up the Core(TM)2 processor dispatch table\n");
            dispatch = &core2_dispatch;
            break;
        case 6:
            SEP_PRINT_DEBUG("Set up the Silvermont dispatch table\n");
            dispatch = &silvermont_dispatch;
            break;
        case 7:
            SEP_PRINT_DEBUG("Set up the perfver4 HTON dispatch table such as Skylake\n");
            dispatch = &perfver4_dispatch;
            break;
        case 8:
            SEP_PRINT_DEBUG("Set up the perfver4 HTOFF dispatch table such as Skylake\n");
            dispatch = &perfver4_dispatch_htoff_mode;
            break;
        case 700:
        case 701:
        case 1100:
            SEP_PRINT_DEBUG("Set up the Valleyview SA dispatch table\n");
            dispatch = &valleyview_visa_dispatch;
            break;
        case 2:
            dispatch = &corei7_dispatch;
            SEP_PRINT_DEBUG("Set up the Core i7(TM) processor dispatch table\n");
            break;
        case 3:
            SEP_PRINT_DEBUG("Set up the Core i7(TM) dispatch table\n");
            dispatch = &corei7_dispatch_htoff_mode;
            break;
        case 4:
            dispatch = &corei7_dispatch_2;
            SEP_PRINT_DEBUG("Set up the Sandybridge processor dispatch table\n");
            break;
        case 5:
            SEP_PRINT_DEBUG("Set up the Sandybridge dispatch table\n");
            dispatch = &corei7_dispatch_htoff_mode_2;
            break;
        case 9:
            dispatch = &corei7_dispatch_nehalem;
            SEP_PRINT_DEBUG("Set up the Nehalem, Westemere dispatch table\n");
            break;
        case 10:
            dispatch = &knights_dispatch;
            SEP_PRINT_DEBUG("Set up the Knights family dispatch table\n");
            break;
        case 100:
            SEP_PRINT_DEBUG("Set up the MSR based uncore dispatch table\n");
            dispatch = &unc_msr_dispatch;
            break;
        case 110:
            SEP_PRINT_DEBUG("Set up the PCI Based Uncore dispatch table\n");
            dispatch = &unc_pci_dispatch;
            break;
        case 120:
            SEP_PRINT_DEBUG("Set up the MMIO based uncore dispatch table\n");
            dispatch = &unc_mmio_dispatch;
            break;
        case 121:
            SEP_PRINT_DEBUG("Set up the MMIO based uncore dispatch table for FPGA\n");
            dispatch = &unc_mmio_fpga_dispatch;
            break;
        case 130:
            SEP_PRINT_DEBUG("Set up the Uncore Power dispatch table\n");
            dispatch = &unc_power_dispatch;
            break;
#if defined(SEP_ENABLE_NDA_CPUS)
        case 230:
            SEP_PRINT_DEBUG("Set up the Haswell SA dispatch table\n");
            dispatch = &hswunc_sa_dispatch;
            break;
#endif
        case 400:
            SEP_PRINT_DEBUG("Set up the SNB GT dispatch table\n");
            dispatch = &snbunc_gt_dispatch;
            break;
        default:
            dispatch = NULL;
            SEP_PRINT_ERROR("Architecture not supported (dispatch_id=%d)\n", dispatch_id);
            break;
    }

    return dispatch;
}

extern U64
SYS_MMIO_Read64(
    U64 baseAddress,
    U64 offset
)
{
    if (baseAddress) {
        char  volatile *p  = ((char volatile *) baseAddress) + offset; // offset is in bytes
        U64 volatile  u;
        U64 volatile *up = (U64 volatile *)p;
        u = *up;
        return u;
    } else {
        SEP_PRINT_ERROR("baseAddress is NULL in %s\n", __FUNCTION__);
        return  ( U64)- 1;          // typical value for undefined CSR
    }
}

extern U64
SYS_Read_MSR (
    U32   msr
)
{
    U64 val = 0;

#if defined(DRV_DEBUG_MSR)
    rdmsrl_safe(msr, &val);
#else
    rdmsrl(msr, val);
#endif

    return val;
}


#if defined(BUILD_CHIPSET)
/* ------------------------------------------------------------------------- */
/*!
 * @fn       VOID UTILITY_Configure_Chipset
 *
 * @brief    Configures the chipset information
 *
 * @param    none
 *
 * @return   none
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
extern  CS_DISPATCH
UTILITY_Configure_Chipset (
    void
)
{
    if (CHIPSET_CONFIG_gmch_chipset(pma)) {
        cs_dispatch = &gmch_dispatch;
        SEP_PRINT_DEBUG("UTLITY_Configure_Chipset: using GMCH dispatch table!\n");
    }
    else if (CHIPSET_CONFIG_mch_chipset(pma) || CHIPSET_CONFIG_ich_chipset(pma)) {
        cs_dispatch = &chap_dispatch;
        SEP_PRINT_DEBUG("UTLITY_Configure_Chipset: using CHAP dispatch table!\n");
    }
    else {
        SEP_PRINT_ERROR("UTLITY_Configure_Chipset: unable to map chipset dispatch table!\n");
    }

    SEP_PRINT_DEBUG("UTLITY_Configure_Chipset: exiting with cs_dispatch=0x%p\n", cs_dispatch);

    return cs_dispatch;
}

#endif
