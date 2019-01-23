/*
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

#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <linux/smp.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <asm/atomic.h>

#include "sp.h"
/****************************************************************************
 **  Handy Short cuts
 ***************************************************************************/

typedef void* pvoid;
#define TRUE 1
#define FALSE 0
/*
 *  These routines have macros defined in asm/system.h
 */
#define SYS_Local_Irq_Enable()       local_irq_enable()
#define SYS_Local_Irq_Disable()      local_irq_disable()
#define SYS_Local_Irq_Save(flags)    local_irq_save(flags)
#define SYS_Local_Irq_Restore(flags) local_irq_restore(flags)

/*
 * CONTROL_THIS_CPU()
 *     Parameters
 *         None
 *     Returns
 *         CPU number of the processor being executed on
 *
 */
#define CONTROL_THIS_CPU()     smp_processor_id()

/****************************************************************************
 **  Interface definitions
 ***************************************************************************/

/*
 *  Execution Control Functions
 */

extern void
CONTROL_Invoke_Cpu (
    s32   cpuid,
    void  (*func)(pvoid),
    pvoid ctx
);

/*
 * @fn VOID CONTROL_Invoke_Parallel_Service(func, ctx, blocking, exclude)
 *
 * @param    func     - function to be invoked by each core in the system
 * @param    ctx      - pointer to the parameter block for each function invocation
 * @param    blocking - Wait for invoked function to complete
 * @param    exclude  - exclude the current core from executing the code
 *
 * @returns  none
 *
 * @brief    Service routine to handle all kinds of parallel invoke on all CPU calls
 *
 * <I>Special Notes:</I>
 *         Invoke the function provided in parallel in either a blocking/non-blocking mode.
 *         The current core may be excluded if desired.
 *         NOTE - Do not call this function directly from source code.  Use the aliases
 *         CONTROL_Invoke_Parallel(), CONTROL_Invoke_Parallel_NB(), CONTROL_Invoke_Parallel_XS().
 *
 */
extern void
CONTROL_Invoke_Parallel_Service (
        void   (*func)(pvoid),
        pvoid  ctx,
        s32    blocking,
        s32    exclude
);

/*
 * @fn VOID CONTROL_Invoke_Parallel(func, ctx)
 *
 * @param    func     - function to be invoked by each core in the system
 * @param    ctx      - pointer to the parameter block for each function invocation
 *
 * @returns  none
 *
 * @brief    Invoke the named function in parallel. Wait for all the functions to complete.
 *
 * <I>Special Notes:</I>
 *        Invoke the function named in parallel, including the CPU that the control is
 *        being invoked on
 *        Macro built on the service routine
 *
 */
#define CONTROL_Invoke_Parallel(a,b)      CONTROL_Invoke_Parallel_Service((a),(b),TRUE,FALSE)

/*
 * @fn VOID CONTROL_Invoke_Parallel_NB(func, ctx)
 *
 * @param    func     - function to be invoked by each core in the system
 * @param    ctx      - pointer to the parameter block for each function invocation
 *
 * @returns  none
 *
 * @brief    Invoke the named function in parallel. DO NOT Wait for all the functions to complete.
 *
 * <I>Special Notes:</I>
 *        Invoke the function named in parallel, including the CPU that the control is
 *        being invoked on
 *        Macro built on the service routine
 *
 */
#define CONTROL_Invoke_Parallel_NB(a,b)   CONTROL_Invoke_Parallel_Service((a),(b),FALSE,FALSE)

/*
 * @fn VOID CONTROL_Invoke_Parallel_XS(func, ctx)
 *
 * @param    func     - function to be invoked by each core in the system
 * @param    ctx      - pointer to the parameter block for each function invocation
 *
 * @returns  none
 *
 * @brief    Invoke the named function in parallel. Wait for all the functions to complete.
 *
 * <I>Special Notes:</I>
 *        Invoke the function named in parallel, excluding the CPU that the control is
 *        being invoked on
 *        Macro built on the service routine
 *
 */
#define CONTROL_Invoke_Parallel_XS(a,b)   CONTROL_Invoke_Parallel_Service((a),(b),TRUE,TRUE)


#endif
