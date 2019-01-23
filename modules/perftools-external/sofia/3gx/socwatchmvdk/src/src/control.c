/*COPYRIGHT**
    Copyright (C) 2005-2014 Intel Corporation.  All Rights Reserved.

    This file is part of SoCWatch Development Kit

    This program is free software; you can redistribute it
    and/or modify it under the terms of the GNU General Public License
    version 2 as published by the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
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

    # Contact Information:
    # SOCWatch Developer Team <socwatchdevelopers@intel.com>
*/

#include <linux/version.h>

#include "control.h"
#include <linux/sched.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#define SMP_CALL_FUNCTION(func,ctx,retry,wait)               smp_call_function((func),(ctx),(wait))
#define SMP_CALL_FUNCTION_SINGLE(cpuid,func,ctx,retry,wait)  smp_call_function_single((cpuid),(func),(ctx),(wait))
#define ON_EACH_CPU(func,ctx,retry,wait)                     on_each_cpu((func),(ctx),(wait))
#else
#define SMP_CALL_FUNCTION(func,ctx,retry,wait)               smp_call_function((func),(ctx),(retry),(wait))
#define SMP_CALL_FUNCTION_SINGLE(cpuid,func,ctx,retry,wait)  smp_call_function_single((cpuid),(func),(ctx),(retry),(wait))
#define ON_EACH_CPU(func,ctx,retry,wait)                     on_each_cpu((func),(ctx),(retry),(wait))
#endif

extern int num_CPUs;
/* ------------------------------------------------------------------------- */
/*!
 * @fn       VOID CONTROL_Invoke_Cpu (func, ctx, arg)
 *
 * @brief    Set up a DPC call and insert it into the queue
 *
 * @param    IN cpu_idx  - the core id to dispatch this function to
 *           IN func     - function to be invoked by the specified core(s)
 *           IN ctx      - pointer to the parameter block for each function
 *                         invocation
 *
 * @return   None
 *
 * <I>Special Notes:</I>
 *
 */
extern void
CONTROL_Invoke_Cpu (
    int     cpu_idx,
    void    (*func)(pvoid),
    pvoid   ctx
)
{
    SMP_CALL_FUNCTION_SINGLE(cpu_idx, func, ctx, 0, 1);

    return;
}

/* ------------------------------------------------------------------------- */
/*
 * @fn VOID CONTROL_Invoke_Parallel_Service(func, ctx, blocking, exclude)
 *
 * @param    func     - function to be invoked by each core in the system
 * @param    ctx      - pointer to the parameter block for each function invocation
 * @param    blocking - Wait for invoked function to complete
 * @param    exclude  - exclude the current core from executing the code
 *
 * @returns  None
 *
 * @brief    Service routine to handle all kinds of parallel invoke on all CPU calls
 *
 * <I>Special Notes:</I>
 *           Invoke the function provided in parallel in either a blocking or
 *           non-blocking mode.  The current core may be excluded if desired.
 *           NOTE - Do not call this function directly from source code.
 *           Use the aliases CONTROL_Invoke_Parallel(), CONTROL_Invoke_Parallel_NB(),
 *           or CONTROL_Invoke_Parallel_XS().
 *
 */
extern void
CONTROL_Invoke_Parallel_Service (
    void   (*func)(pvoid),
    pvoid  ctx,
    int    blocking,
    int    exclude
)
{
    if (num_CPUs == 1) {
        if (!exclude) {
            func(ctx);
        }
        return;
    }
    if (!exclude) {
        ON_EACH_CPU(func, ctx, 0, blocking);
        return;
    }

    preempt_disable();
    SMP_CALL_FUNCTION (func, ctx, 0, blocking);
    preempt_enable();

    return;
}
