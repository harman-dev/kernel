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

#ifndef _KNL_EDC_H_INC_
#define _KNL_EDC_H_INC_

// Channel 0
#define KNL_EDC_UCLK_DID            0x7833
#define KNL_EDC_ECLK_DID            0x7835
#define KNL_EDC_NUM_DEVS            1

#define KNL_EDC_UNIT_COUNTER_UNIT_CONTROL_OFFSET_UCLK    0x430 
#define KNL_EDC_UNIT_COUNTER_0_CONTROL_OFFSET_UCLK       0x420
#define KNL_EDC_UNIT_COUNTER_1_CONTROL_OFFSET_UCLK       0x424
#define KNL_EDC_UNIT_COUNTER_2_CONTROL_OFFSET_UCLK       0x428
#define KNL_EDC_UNIT_COUNTER_3_CONTROL_OFFSET_UCLK       0x42c
#define KNL_EDC_UNIT_FIXED_COUNTER_CONTROL_OFFSET_UCLK   0x454

#define KNL_EDC_UNIT_COUNTER_UNIT_CONTROL_OFFSET_ECLK    0xA30 
#define KNL_EDC_UNIT_COUNTER_0_CONTROL_OFFSET_ECLK       0xA20
#define KNL_EDC_UNIT_COUNTER_1_CONTROL_OFFSET_ECLK       0xA24
#define KNL_EDC_UNIT_COUNTER_2_CONTROL_OFFSET_ECLK       0xA28
#define KNL_EDC_UNIT_COUNTER_3_CONTROL_OFFSET_ECLK       0xA2c
#define KNL_EDC_UNIT_FIXED_COUNTER_CONTROL_OFFSET_ECLK   0xA44

#define ENABLE_KNL_EDC_COUNTERS       0x400000
#define DISABLE_KNL_EDC_COUNTERS      0x100

#define IS_THIS_KNL_BOX_EDC_UCLK_PCI_UNIT_CTL(x)     (x == KNL_EDC_UNIT_COUNTER_UNIT_CONTROL_OFFSET_UCLK)
#define IS_THIS_KNL_EDC_UCLK_PCI_PMON_CTL(x)        (   (x == KNL_EDC_UNIT_COUNTER_0_CONTROL_OFFSET_UCLK)  \
                                                          || (x == KNL_EDC_UNIT_COUNTER_1_CONTROL_OFFSET_UCLK) \
                                                          || (x == KNL_EDC_UNIT_COUNTER_2_CONTROL_OFFSET_UCLK) \
                                                          || (x == KNL_EDC_UNIT_COUNTER_3_CONTROL_OFFSET_UCLK) \
                                                          || (x == KNL_EDC_UNIT_FIXED_COUNTER_CONTROL_OFFSET_UCLK))

#define IS_THIS_KNL_BOX_EDC_ECLK_PCI_UNIT_CTL(x)     (x == KNL_EDC_UNIT_COUNTER_UNIT_CONTROL_OFFSET_ECLK)
#define IS_THIS_KNL_EDC_ECLK_PCI_PMON_CTL(x)        (   (x == KNL_EDC_UNIT_COUNTER_0_CONTROL_OFFSET_ECLK) \
                                                          || (x == KNL_EDC_UNIT_COUNTER_1_CONTROL_OFFSET_ECLK) \
                                                          || (x == KNL_EDC_UNIT_COUNTER_2_CONTROL_OFFSET_ECLK) \
                                                          || (x == KNL_EDC_UNIT_COUNTER_3_CONTROL_OFFSET_ECLK) \
                                                          || (x == KNL_EDC_UNIT_FIXED_COUNTER_CONTROL_OFFSET_ECLK))

extern DISPATCH_NODE  knl_edc_uclk_dispatch;
extern DISPATCH_NODE  knl_edc_eclk_dispatch;
#define KNL_UBOX_GLOBAL_CONTROL_MSR            0x700
#define RESET_EDC_CTRS                         0x10002
#endif

