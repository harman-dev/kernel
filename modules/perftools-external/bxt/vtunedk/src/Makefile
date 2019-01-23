#
# Version: 1.6
#
# Copyright (C) 2008-2017 Intel Corporation.  All Rights Reserved.
#
#     This file is part of SEP Development Kit
#
#     SEP Development Kit is free software; you can redistribute it
#     and/or modify it under the terms of the GNU General Public License
#     version 2 as published by the Free Software Foundation.
#
#     SEP Development Kit is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
#
#     You should have received a copy of the GNU General Public License
#     along with SEP Development Kit; if not, write to the Free Software
#     Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#
#     As a special exception, you may use this file as part of a free software
#     library without restriction.  Specifically, if other files instantiate
#     templates or use macros or inline functions from this file, or you compile
#     this file and link it with other files to produce an executable, this
#     file does not by itself cause the resulting executable to be covered by
#     the GNU General Public License.  This exception does not however
#     invalidate any other reasons why the executable file might be covered by
#     the GNU General Public License.
#

# -------------------- user configurable options ------------------------

# base name of SEP driver
DRIVER_NAME = sep4_1

# location to install driver
INSTALL = .

# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
# Example flags are "-Werror", "-Wno-error", etc.
EXTRA_CFLAGS += -I$(M)/../include -I$(M)/inc

# if ARCH variable is set, unset it to avoid conflicts with kbuild
unexport ARCH

# platform details
ifeq ($(CONFIG_X86_64),y)
    MACH=x86_64
else
    MACH=i686
endif
ifeq ($(MACH),x86_64)
    PLATFORM=x32_64
endif
ifeq ($(PLATFORM),)
    PLATFORM=x32
endif

# Either PMI or NMI interrupt mode can be selected. PMI is by default.
# For NMI mode, either use the build-driver script with the option "--nmi"
# or set NMI in the below line.
INTERRUPT_MODE=NMI

ifeq ($(NMI_MODE),YES)
INTERRUPT_MODE=NMI
endif
NMI_SUPPORT_KERNEL=YES

# eventual filename of SEP driver
DRIVER_MODE=$(DRIVER_NAME)
DRIVER_FILENAME=$(DRIVER_NAME).ko

# By default, build the EMON portions of the driver
EMON=YES

KBUILD_EXTRA_SYMBOLS := ../modules/perftools-external/bxt/socperfdk/src/Module.symvers

# build options ...
ifneq ($(KERNELRELEASE),)
	obj-m := $(DRIVER_NAME).o

ifeq ($(PLATFORM),x32)
	arch-objs :=    core2.o            \
			perfver4.o         \
			sys32.o            \
			silvermont.o       \
			pci.o              \
			apic.o             \
			pebs.o             \
			unc_gt.o           \
			unc_mmio.o         \
			unc_msr.o          \
			unc_common.o       \
			unc_pci.o          \
			sepdrv_p_state.o
endif
ifeq ($(PLATFORM),x32_64)
	arch-objs :=    core2.o            \
			perfver4.o         \
			sys64.o            \
			silvermont.o       \
			pci.o              \
			apic.o             \
			pebs.o             \
			unc_gt.o           \
			unc_mmio.o         \
			unc_msr.o          \
			unc_common.o       \
			unc_pci.o          \
			sepdrv_p_state.o
endif

ifeq ($(EMON),YES)
    EXTRA_CFLAGS += -DEMON
endif
EXTRA_CFLAGS += -DDRV_ANDROID

ifeq ($(BOARD_HAVE_SMALL_RAM),true)
EXTRA_CFLAGS += -DSMALL_RAM
endif

ifeq ($(INTERRUPT_MODE),NMI)
    ifeq ($(NMI_SUPPORT_KERNEL),YES)
        EXTRA_CFLAGS += -DDRV_USE_NMI
        $(info ***************************************************************)
        $(info NOTE: NMI Interrupt Mode is set on $(KERNEL_VERSION) kernel)
        $(info ***************************************************************)
    else
        $(info ****************************************************************************)
        $(info WARNING: NMI Interrupt Mode is not supported on $(KERNEL_VERSION) kernel)
        $(info WARNING: Switching to PMI Mode.)
        $(info ****************************************************************************)
        INTERRUPT_MODE=
    endif
endif

	$(DRIVER_NAME)-objs :=              \
			lwpmudrv.o          \
			control.o           \
			cpumon.o            \
			eventmux.o          \
			linuxos.o           \
			output.o            \
			pmi.o               \
			sys_info.o          \
			utility.o           \
			valleyview_sochap.o \
			unc_power.o         \
			$(arch-objs)

# targets ...

# Otherwise, we were called directly from the command
# line, so the kernel build system will be used.
else
	PWD  := $(shell pwd)
	PATH := $(ANDROID_TOOLCHAIN):$(PATH)
	export PATH

all: modules

modules:
	$(MAKE) -C $(KERNEL_SRC_DIR) M=$(PWD) LDDINCDIR=$(PWD)/../include LDDINCDIR1=$(PWD)/inc modules PWD=$(PWD)
	@(cd pax && make all)
endif

install-modules: modules
	@cp $(DRIVER_NAME).ko $(INSTALL)/$(DRIVER_FILENAME)
	@echo "Installed $(DRIVER_NAME) driver to $(INSTALL)/$(DRIVER_FILENAME) ."
	@(cd pax && make install INSTALL=$(INSTALL)/pax)

clean:
	@rm -f *.o .*.o.cmd .*.o.d .*.ko.cmd .*.ko.unsigned.cmd *.gcno
	@rm -f $(DRIVER_NAME).ko $(DRIVER_NAME).ko.unsigned
	@rm -f $(DRIVER_MODE)*$(DRIVER_TYPE).ko
	@rm -f Module.symvers Modules.symvers modules.order Module.markers
	@rm -rf .tmp_versions
	@(cd pax && make clean)

distclean: clean
	@rm -f $(DRIVER_NAME)*.ko
	@(cd pax && make distclean)
