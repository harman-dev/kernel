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
#include <linux/interrupt.h>
#include <asm/msr.h>
#if defined(DRV_USE_NMI) || defined(DRV_USE_LINUX_APIC_API)
#include <asm/apic.h>
#endif
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
#include <xen/xen.h>
#endif
#if defined(CONFIG_XEN_DOM0) && LINUX_VERSION_CODE > KERNEL_VERSION(3,3,0)
#include <xen/interface/platform.h>
#include <asm/xen/hypercall.h>
#endif

#include "lwpmudrv_types.h"
#include "rise_errors.h"
#include "lwpmudrv_ecb.h"
#include "apic.h"
#include "lwpmudrv.h"
#include "control.h"
#include "utility.h"

#if defined(DRV_USE_NMI) || defined(DRV_USE_LINUX_APIC_API)
static DEFINE_PER_CPU(unsigned long, saved_apic_lvtpc);
#endif

U32 drv_x2apic_enabled = 0;


/*!
 * @fn          VOID apic_Get_APIC_ID(S32 cpu)
 *
 * @brief       Obtain APIC ID
 *
 * @param       S32 cpuid - cpu index
 *
 * @return      U32 APIC ID
 */
VOID
apic_Get_APIC_ID(S32 cpu)
{
    U32             apic_id = 0;
    CPU_STATE       pcpu    = &pcb[cpu];

#if !defined(DRV_USE_NMI) && !defined(DRV_USE_LINUX_APIC_API)
    char           *apic;

    if (drv_x2apic_enabled) {
        apic_id    = SYS_Read_MSR(DRV_APIC_LCL_ID_MSR);
    }
    else {
        apic       = (char*) CPU_STATE_apic_linear_addr(pcpu);
        apic_id    = (*(U32*)&apic[DRV_APIC_LCL_ID]) >> 24;
    }
#else
#if defined(CONFIG_XEN_DOM0) && LINUX_VERSION_CODE > KERNEL_VERSION(3,3,0)
    if (xen_initial_domain()) {
        S32 ret = 0;
        struct xen_platform_op op = {
                .cmd                   = XENPF_get_cpuinfo,
                .interface_version     = XENPF_INTERFACE_VERSION,
                .u.pcpu_info.xen_cpuid = cpu,
        };

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
        ret = HYPERVISOR_platform_op(&op);
#else
        ret = HYPERVISOR_dom0_op(&op);
#endif
        if (ret) {
            SEP_PRINT_ERROR("apic_Get_APIC_ID: Error in reading APIC ID on Xen PV\n");
            apic_id = 0;
        } else {
            apic_id = op.u.pcpu_info.apic_id;
        }
    } else {
#endif
        apic_id = read_apic_id();
#if defined(CONFIG_XEN_DOM0) && LINUX_VERSION_CODE > KERNEL_VERSION(3,3,0)
    }
#endif
#endif

    CPU_STATE_apic_id(pcpu) = apic_id;

    SEP_PRINT_DEBUG("apic_Get_APIC_ID: apic_id[%d] is %d\n", cpu, CPU_STATE_apic_id(pcpu));
}


/*!
 * @fn          extern VOID APIC_DIsable_PMU(VOID)
 * 
 * @brief       mask the performance interrupt vector in the LVT
 *
 * @param       None
 * 
 * @return      None
 *
 * <I>Special Notes:</I>
 */
extern VOID
APIC_Disable_PMI(VOID)
{
#if !defined(DRV_USE_NMI)
#if defined(DRV_USE_LINUX_APIC_API)
    apic_write(APIC_LVTPC, CPU_PERF_VECTOR | DRV_LVT_MASK);
#else
    if (drv_x2apic_enabled) {
        SYS_Write_MSR(DRV_APIC_LVT_PMI_MSR, (U64)(CPU_PERF_VECTOR | DRV_LVT_MASK));
    }
    else {
        char *apic  = CPU_STATE_apic_linear_addr(&pcb[CONTROL_THIS_CPU()]);
        if (apic) {
            *(int*)&apic[DRV_APIC_LVT_PMI] = CPU_PERF_VECTOR | DRV_LVT_MASK;
        }
    }
#endif
#endif
}

/*!
 * @fn          extern VOID APIC_Deinit_Phase1(cpu_idx)
 * 
 * @brief       Part1 of removing the interrupt vector entry
 *
 * @param       int cpu_idx - The cpu to deinit
 * 
 * @return      None
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
extern VOID
APIC_Deinit_Phase1(int  cpu_idx)
{
#if !defined(DRV_USE_NMI) && !defined(DRV_USE_LINUX_APIC_API)
    if (drv_x2apic_enabled == 0) {
        PVOID apic  = CPU_STATE_apic_linear_addr(&pcb[cpu_idx]);
        if (apic) {
            CPU_STATE_apic_linear_addr(&pcb[cpu_idx]) = NULL;
        }
    }
#endif
}

/*!
 * @fn          extern VOID APIC_Init(param)
 *
 * @brief       initialize the local APIC
 *
 * @param       int cpu_idx - The cpu to deinit
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 *              This routine is expected to be called via the CONTROL_Parallel routine
 */
extern VOID
APIC_Init (PVOID param)
{
#if !defined(DRV_USE_NMI) && !defined(DRV_USE_LINUX_APIC_API)
    char           *apic;
    unsigned long   eflags;
    U64             addr;
    char            pic0, pic1;
    char           *linear = *(char **)param;
#endif
    int             me;
    CPU_STATE       pcpu;

    preempt_disable();
    me      = CONTROL_THIS_CPU();
    pcpu    = &pcb[me];
    preempt_enable();

#if !defined(DRV_USE_NMI) && !defined(DRV_USE_LINUX_APIC_API)
    addr = SYS_Read_MSR(DRV_APIC_BASE_MSR);
    apic = NULL;
    if ((addr & DRV_X2APIC_ENABLED) == DRV_X2APIC_ENABLED) {
        drv_x2apic_enabled      = 1;
    }
    else {
        CPU_STATE_apic_physical_addr(pcpu) = (PVOID)(UIOP)(addr & 0x00000000fffff000LL);

        if (linear == NULL) {
            CPU_STATE_apic_linear_addr(pcpu) = (char*)ioremap_nocache(
                        (unsigned long)CPU_STATE_apic_physical_addr(pcpu) & 0xfffff000, 0x1000);
            *(char **)param =  (char*)CPU_STATE_apic_linear_addr(pcpu);
        } 
        else {
            CPU_STATE_apic_linear_addr(pcpu) = (char*)linear;
        }

        apic = (char*)CPU_STATE_apic_linear_addr(pcpu);
        if (!apic) {
            SEP_PRINT_ERROR("APIC_init_phase1: Failed for cpu %d\n", me);
            return;
        }

        SEP_PRINT_DEBUG("phase 2: addr is 0x%llx, apic is 0x%p\n", addr, apic);

        if (!(DRV_APIC_BASE_GLOBAL_ENABLED(addr)) || 
            !(DRV_APIC_VIRTUAL_WIRE_ENABLED(*(int*)&apic[DRV_APIC_LCL_SVR]))) {

            SEP_PRINT_DEBUG("phase 2: setting up virtual wire\n" );
            // setup virtual wire
            SYS_Local_Irq_Save(eflags);
            SYS_Local_Irq_Disable();

            // mask PICs
            pic0 = SYS_Inb(0x21);
            SYS_IO_Delay();
            pic1 = SYS_Inb(0xa1);
            SYS_IO_Delay();
            SYS_Outb(0xff, 0xa1);
            SYS_IO_Delay();
            SYS_Outb(0xff, 0x21);
            SYS_Local_Irq_Enable();
            SYS_IO_Delay();
            SYS_IO_Delay();
            SYS_Local_Irq_Disable();

            // enable via APIC_BASE_MSR
            addr |= (1 << 11);
            SEP_PRINT_DEBUG(" about to set APIC_BASE_MSR to 0x%llx\n", addr);
            SYS_Write_MSR(DRV_APIC_BASE_MSR, addr);

            // enable in SVR and set VW
            *(int*)&apic[DRV_APIC_LCL_SVR]    = 0x01ff;
            *(int*)&apic[DRV_APIC_LCL_TSKPRI] = DRV_APIC_TSKPRI_HI;
            *(int*)&apic[DRV_APIC_LCL_ID]     = 0;
            *(int*)&apic[DRV_APIC_LCL_LDEST]  = 0;           /// set local dest. id
            *(int*)&apic[DRV_APIC_LCL_DSTFMT] = -1;          /// set dest. format
            *(int*)&apic[DRV_APIC_LVT_TIMER]  = DRV_LVT_MASK;    /// mask local timer
            *(int*)&apic[DRV_APIC_LVT_ERROR]  = DRV_LVT_MASK;    /// mask error

            // INTs are redirected to PICs
            *(int*)&apic[DRV_APIC_LVT_LINT0]  = DRV_LVT_EXTINT + DRV_LVT_EDGE;
            *(int*)&apic[DRV_APIC_LVT_LINT1]  = DRV_LVT_NMI + DRV_LVT_LEVEL;
            *(int*)&apic[DRV_APIC_LCL_TSKPRI] = DRV_APIC_TSKPRI_LO;


            // unmask PICs
            SYS_Outb(pic0, 0x21);
            SYS_Outb(pic1, 0xa1);
            SYS_Local_Irq_Restore(eflags);
        }
    }
#endif

    apic_Get_APIC_ID(me);
}

/*!
 * @fn          extern VOID APIC_Install_Interrupt_Handler(param)
 *
 * @brief       Install the interrupt handler
 *
 * @param       int param - The linear address of the Local APIC
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 *             The linear address is necessary if the LAPIC is used.  If X2APIC is
 *             used the linear address is not necessary.
 */
extern VOID
APIC_Install_Interrupt_Handler (PVOID param)
{
#if !defined(DRV_USE_NMI)
#if defined(DRV_USE_LINUX_APIC_API)
    per_cpu(saved_apic_lvtpc, CONTROL_THIS_CPU()) = apic_read(APIC_LVTPC);
    apic_write(APIC_LVTPC, CPU_PERF_VECTOR | DRV_LVT_MASK);
#else
    char           *apic;
    int             me = CONTROL_THIS_CPU();
    unsigned long   eflags;

    SYS_Local_Irq_Save(eflags);
    if (drv_x2apic_enabled) {
        SYS_Write_MSR(DRV_APIC_LVT_PMI_MSR, (U64)(CPU_PERF_VECTOR | DRV_LVT_MASK));
    }
    else {
        apic = (char*)CPU_STATE_apic_linear_addr(&pcb[me]);
        if (!apic) {
            SEP_PRINT_ERROR("APIC_init_phase1: Failed for cpu %d\n", me);
            goto cleanup;
        }

        // initialize perfmon vector in disabled state
        *(int*)&apic[DRV_APIC_LVT_PMI] = CPU_PERF_VECTOR | DRV_LVT_MASK;
    }
cleanup:
    SYS_Local_Irq_Restore(eflags);
#endif
#else
    per_cpu(saved_apic_lvtpc, CONTROL_THIS_CPU()) = apic_read(APIC_LVTPC);
    apic_write(APIC_LVTPC, APIC_DM_NMI);
#endif
}

/*!
 * @fn          extern VOID APIC_Unmap(apic_linear_address)
 *
 * @brief       Unmap the APIC region
 *
 * @param       PVOID apic_linear_address - The linear address of the Local APIC
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 *             Unmap all apic logical address ranges
 */
extern VOID
APIC_Unmap (PVOID apic_linear_addr)
{
#if !defined(DRV_USE_NMI) && !defined(DRV_USE_LINUX_APIC_API)
    if (drv_x2apic_enabled) {
        return;
    }

    if (apic_linear_addr) {
        iounmap(apic_linear_addr);
    }
#endif
    return;
}

/*!
 * @fn          extern VOID APIC_Ack_Eoi(void)
 * 
 * @brief       Acknowledge the End-Of-Interrupt
 *
 * @param       None
 * 
 * @return      None
 *
 * <I>Special Notes:</I>
 *             <NONE>
 */
extern VOID
APIC_Ack_Eoi(VOID)
{
#if !defined(DRV_USE_NMI)
#if defined(DRV_USE_LINUX_APIC_API)
    apic_write(APIC_EOI, 0);
#else
    if (drv_x2apic_enabled) {
        SYS_Write_MSR(DRV_APIC_LCL_EOI_MSR, 0LL);
    }
    else {
        char *apic  = (char*)CPU_STATE_apic_linear_addr(&pcb[CONTROL_THIS_CPU()]);
        if (apic) {
            *(int*)&apic[DRV_APIC_LCL_EOI] = 0;
        }
    }
#endif
#endif
    return;
}

/*!
 * @fn          extern VOID APIC_Enable_PMI(void)
 * 
 * @brief       Enable the PMU interrupt
 *
 * @param       None
 * 
 * @return      None
 *
 * <I>Special Notes:</I>
 *             <NONE>
 */
extern VOID
APIC_Enable_Pmi(VOID)
{
#if !defined(DRV_USE_NMI)
#if defined(DRV_USE_LINUX_APIC_API)
    U64 value = apic_read(APIC_LVTPC);
    value &= ~DRV_LVT_MASK;
    apic_write(APIC_LVTPC, value);
#else
    if (drv_x2apic_enabled) {
        U64 value = SYS_Read_MSR(DRV_APIC_LVT_PMI_MSR);
        value &= ~DRV_LVT_MASK;
        SYS_Write_MSR(DRV_APIC_LVT_PMI_MSR, value);
    }
    else {
        char *apic  = (char*)CPU_STATE_apic_linear_addr(&pcb[CONTROL_THIS_CPU()]);
        if (apic) {
            *(int*)&apic[DRV_APIC_LVT_PMI] &= ~DRV_LVT_MASK;
        }
    }
#endif
#else
    apic_write(APIC_LVTPC, APIC_DM_NMI);
#endif
}


#if defined(DRV_USE_NMI) || defined(DRV_USE_LINUX_APIC_API)
/*!
 * @fn          extern VOID APIC_Restore_LVTPC(void)
 *
 * @brief       Restore APIC LVTPC value
 *
 * @param       None
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 *             <NONE>
 */
extern VOID
APIC_Restore_LVTPC(PVOID param)
{
    apic_write(APIC_LVTPC, per_cpu(saved_apic_lvtpc, CONTROL_THIS_CPU()));
}
#endif
