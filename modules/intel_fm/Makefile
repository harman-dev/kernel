# Copyright (C) 2013 Intel Mobile Communications GmbH
#
# This software is licensed under the terms of the GNU General Public
# License version 2, as published by the Free Software Foundation, and
# may be copied, distributed, and modified under those terms.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# Overall
obj-m+= fmdrv_intel.o

# FM Radio Includes
## Common
CHIPID		:= lnp

## Search paths
ccflags-y	= -I$(M)
ccflags-y	+= -I$(M)/include
ccflags-y	+= -I$(M)/external_include
ccflags-y	+= -I$(M)/$(CHIPID)
ccflags-y	+= -I$(M)/$(CHIPID)/hw

## V4L2
ccflags-y	+= -Iarch/arm/mach-xgold
ccflags-y	+= -Iarch/arm/mach-xgold/include/mach
ccflags-y	+= -Iinclude/linux

## intel_ld (line discipline driver)
ccflags-y	+= -I../modules/intel_ld/dev/

# FM Radio objects
FMROBJS	= fmtrx_v4l2.o
FMROBJS	+= fmtrx_common.o
FMROBJS	+= fmtrx_platform.o
FMROBJS	+= $(CHIPID)/fmtrx_hw_rx.o

# Defines
#ccflags-y	+= -DFMR_HOST_TEST
ccflags-y	+= -DCONFIG_RADIO_I8X70
ccflags-y	+= -DFMR_INTR_MODE
ccflags-y	+= -DFMR_AUDIO_ENABLE
ccflags-y	+= -DFMR_ALIGN=""
ccflags-y	+= -DFMR_DEBUG_MEAS
# LVL1 - print high level information - lesser logs
ccflags-y	+= -DFMR_DEBUG_LVL1
# LVL2 - print low level information - detailed logs
ccflags-y	+= -DFMR_DEBUG_LVL2
ccflags-y	+= -DLD_DRIVER
ifneq ($(CONFIG_RADIO_I8X70),)
#ccflags-y	+= -DLNP_EVB
endif

fmdrv_intel-objs	:= $(FMROBJS)

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
