/*
    Copyright (C) 2012-2015 Intel Corporation.  All Rights Reserved.

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

#ifndef _KNL_IRP_H_INC_
#define _KNL_IRP_H_INC_

#define KNL_IRP_DID                            0x7814 // --- IRP PerfMon DID --- B:D 0:5:6

#define KNL_IRP_COUNTER_UNIT_CONTROL_OFFSET    0xf0
#define KNL_IRP_COUNTER_0_CONTROL_OFFSET       0xd8
#define KNL_IRP_COUNTER_1_CONTROL_OFFSET       0xdc

#define ENABLE_KNL_IRP_COUNTERS                0x400000
#define DISABLE_KNL_IRP_COUNTERS               0x100

#define IS_THIS_KNL_IRP_UNIT_CTL(x)            ( (x) == KNL_IRP_COUNTER_UNIT_CONTROL_OFFSET)
#define IS_THIS_KNL_IRP_PMON_CTL(x)            ( ((x) == KNL_IRP_COUNTER_0_CONTROL_OFFSET) || ((x) == KNL_IRP_COUNTER_1_CONTROL_OFFSET))
extern  DISPATCH_NODE                          knl_irp_dispatch;

#endif

