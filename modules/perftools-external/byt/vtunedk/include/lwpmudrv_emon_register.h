/*COPYRIGHT**
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
**COPYRIGHT*/

#ifndef _LWPMUDRV_EMON_H_
#define _LWPMUDRV_EMON_H_

#include <vector>
using namespace std;

typedef struct EMON_REGISTER_INFO_S     EMON_REGISTER_INFO_NODE;
typedef vector<EMON_REGISTER_INFO_NODE> EMON_REGISTER_INFO;
//a vector (i.e. array) of registers to read or write in the vector order

struct EMON_REGISTER_INFO_S
{
    U32     mode;               //read (0) or write (1)
    U32     reg;                //register    pcb = CONTROL_Free_Memory(pcb);
    U64     value;              //value returned from a read or value to use during a write 
};

#define EMON_REGISTER_INFO_mode(er)     (er)->mode
#define EMON_REGISTER_INFO_reg(er)      (er)->reg
#define EMON_REGISTER_INFO_value(er)    (er)->value

#endif

