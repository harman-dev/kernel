# #
# Support for Intel Camera Imaging ISP subsystem.
# Copyright (c) 2010 - 2017, Intel Corporation.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details
#
#
# MODULE is ISYSAPI

ISYSAPI_DIR=$${MODULES_DIR}/isysapi

ISYSAPI_INTERFACE=$(ISYSAPI_DIR)/interface
ISYSAPI_SOURCES=$(ISYSAPI_DIR)/src
ISYSAPI_EXTINCLUDE=$${MODULES_DIR}/support
ISYSAPI_EXTINTERFACE=$${MODULES_DIR}/syscom/interface

ISYSAPI_HOST_FILES += $(ISYSAPI_SOURCES)/ia_css_isys_public.c

ISYSAPI_HOST_FILES += $(ISYSAPI_SOURCES)/ia_css_isys_private.c

# ISYSAPI Trace Log Level = ISYSAPI_TRACE_LOG_LEVEL_NORMAL
# Other options are [ISYSAPI_TRACE_LOG_LEVEL_OFF, ISYSAPI_TRACE_LOG_LEVEL_DEBUG]
ifndef ISYSAPI_TRACE_CONFIG_HOST
	ISYSAPI_TRACE_CONFIG_HOST=ISYSAPI_TRACE_LOG_LEVEL_NORMAL
endif
ifndef ISYSAPI_TRACE_CONFIG_FW
	ISYSAPI_TRACE_CONFIG_FW=ISYSAPI_TRACE_LOG_LEVEL_NORMAL
endif

ISYSAPI_HOST_CPPFLAGS += -DISYSAPI_TRACE_CONFIG=$(ISYSAPI_TRACE_CONFIG_HOST)
ISYSAPI_FW_CPPFLAGS += -DISYSAPI_TRACE_CONFIG=$(ISYSAPI_TRACE_CONFIG_FW)

ISYSAPI_HOST_FILES += $(ISYSAPI_SOURCES)/ia_css_isys_public_trace.c

ISYSAPI_HOST_CPPFLAGS += -I$(ISYSAPI_INTERFACE)
ISYSAPI_HOST_CPPFLAGS += -I$(ISYSAPI_EXTINCLUDE)
ISYSAPI_HOST_CPPFLAGS += -I$(ISYSAPI_EXTINTERFACE)
ISYSAPI_HOST_CPPFLAGS += -I$(HIVESDK)/systems/ipu_system/dai/include
ISYSAPI_HOST_CPPFLAGS += -I$(HIVESDK)/systems/ipu_system/dai/include/default_system
ISYSAPI_HOST_CPPFLAGS += -I$(HIVESDK)/include/ipu/dai
ISYSAPI_HOST_CPPFLAGS += -I$(HIVESDK)/include/ipu

ISYSAPI_FW_FILES += $(ISYSAPI_SOURCES)/isys_fw.c
ISYSAPI_FW_FILES += $(ISYSAPI_SOURCES)/isys_fw_utils.c

ISYSAPI_FW_CPPFLAGS += -I$(ISYSAPI_INTERFACE)
ISYSAPI_FW_CPPFLAGS += -I$(ISYSAPI_SOURCES)/$(IPU_SYSVER)
ISYSAPI_FW_CPPFLAGS += -I$(ISYSAPI_EXTINCLUDE)
ISYSAPI_FW_CPPFLAGS += -I$(ISYSAPI_EXTINTERFACE)
ISYSAPI_FW_CPPFLAGS += -I$(HIVESDK)/systems/ipu_system/dai/include
ISYSAPI_FW_CPPFLAGS += -I$(HIVESDK)/systems/ipu_system/dai/include/default_system
ISYSAPI_FW_CPPFLAGS += -I$(HIVESDK)/include/ipu/dai
ISYSAPI_FW_CPPFLAGS += -I$(HIVESDK)/include/ipu

