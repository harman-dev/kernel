/*
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
*/

#ifndef _CPUMON_H_
#define _CPUMON_H_

#include <linux/version.h>
#include "lwpmudrv_defines.h"

/*
 *  Defines
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)
#define GATE_STRUCT    struct gate_struct
#else
#define GATE_STRUCT    struct gate_struct64
#endif

/**
 * Function Declarations
 */

/*
 * CPUMON control functions
 */

extern VOID CPUMON_Install_Cpuhooks(VOID);
extern VOID CPUMON_Remove_Cpuhooks(VOID);

#endif 
