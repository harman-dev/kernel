/*
* Support for Intel Camera Imaging ISP subsystem.
* Copyright (c) 2010 - 2017, Intel Corporation.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*/

#ifndef __IA_CSS_PSYS_DATA_TRACE_H
#define __IA_CSS_PSYS_DATA_TRACE_H

#include "ia_css_psysapi_trace.h"

#define PSYS_DATA_TRACE_LEVEL_CONFIG_DEFAULT	PSYSAPI_TRACE_LOG_LEVEL_OFF

/* Default sub-module tracing config */
#if (!defined(PSYSAPI_DATA_TRACING_OVERRIDE))
     #define PSYS_DATA_TRACE_LEVEL_CONFIG PSYS_DATA_TRACE_LEVEL_CONFIG_DEFAULT
#endif

/* Module/sub-module specific trace setting will be used if
 * the trace level is not specified from the module or
  PSYSAPI_DATA_TRACING_OVERRIDE is defined
 */
#if (defined(PSYSAPI_DATA_TRACING_OVERRIDE))
	/* Module/sub-module specific trace setting */
	#if PSYSAPI_DATA_TRACING_OVERRIDE == PSYSAPI_TRACE_LOG_LEVEL_OFF
		/* PSYSAPI_TRACE_LOG_LEVEL_OFF */
		#define PSYSAPI_DATA_TRACE_METHOD \
			IA_CSS_TRACE_METHOD_NATIVE
		#define PSYSAPI_DATA_TRACE_LEVEL_ASSERT \
			IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_DATA_TRACE_LEVEL_ERROR \
			IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_DATA_TRACE_LEVEL_WARNING \
			IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_DATA_TRACE_LEVEL_INFO \
			IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_DATA_TRACE_LEVEL_DEBUG \
			IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_DATA_TRACE_LEVEL_VERBOSE \
			IA_CSS_TRACE_LEVEL_DISABLED
	#elif PSYSAPI_DATA_TRACING_OVERRIDE == PSYSAPI_TRACE_LOG_LEVEL_NORMAL
		/* PSYSAPI_TRACE_LOG_LEVEL_NORMAL */
		#define PSYSAPI_DATA_TRACE_METHOD \
			IA_CSS_TRACE_METHOD_NATIVE
		#define PSYSAPI_DATA_TRACE_LEVEL_ASSERT \
			IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_DATA_TRACE_LEVEL_ERROR \
			IA_CSS_TRACE_LEVEL_ENABLED
		#define PSYSAPI_DATA_TRACE_LEVEL_WARNING \
			IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_DATA_TRACE_LEVEL_INFO \
			IA_CSS_TRACE_LEVEL_ENABLED
		#define PSYSAPI_DATA_TRACE_LEVEL_DEBUG \
			IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_DATA_TRACE_LEVEL_VERBOSE \
			IA_CSS_TRACE_LEVEL_DISABLED
	#elif PSYSAPI_DATA_TRACING_OVERRIDE == PSYSAPI_TRACE_LOG_LEVEL_DEBUG
		/* PSYSAPI_TRACE_LOG_LEVEL_DEBUG */
		#define PSYSAPI_DATA_TRACE_METHOD \
			IA_CSS_TRACE_METHOD_NATIVE
		#define PSYSAPI_DATA_TRACE_LEVEL_ASSERT \
			IA_CSS_TRACE_LEVEL_ENABLED
		#define PSYSAPI_DATA_TRACE_LEVEL_ERROR \
			IA_CSS_TRACE_LEVEL_ENABLED
		#define PSYSAPI_DATA_TRACE_LEVEL_WARNING \
			IA_CSS_TRACE_LEVEL_ENABLED
		#define PSYSAPI_DATA_TRACE_LEVEL_INFO \
			IA_CSS_TRACE_LEVEL_ENABLED
		#define PSYSAPI_DATA_TRACE_LEVEL_DEBUG \
			IA_CSS_TRACE_LEVEL_ENABLED
		#define PSYSAPI_DATA_TRACE_LEVEL_VERBOSE \
			IA_CSS_TRACE_LEVEL_ENABLED
	#else
		#error "No PSYSAPI_DATA Tracing level defined"
	#endif
#else
	/* Inherit Module trace setting */
	#define PSYSAPI_DATA_TRACE_METHOD \
		PSYSAPI_TRACE_METHOD
	#define PSYSAPI_DATA_TRACE_LEVEL_ASSERT \
		PSYSAPI_TRACE_LEVEL_ASSERT
	#define PSYSAPI_DATA_TRACE_LEVEL_ERROR \
		PSYSAPI_TRACE_LEVEL_ERROR
	#define PSYSAPI_DATA_TRACE_LEVEL_WARNING \
		PSYSAPI_TRACE_LEVEL_WARNING
	#define PSYSAPI_DATA_TRACE_LEVEL_INFO \
		PSYSAPI_TRACE_LEVEL_INFO
	#define PSYSAPI_DATA_TRACE_LEVEL_DEBUG \
		PSYSAPI_TRACE_LEVEL_DEBUG
	#define PSYSAPI_DATA_TRACE_LEVEL_VERBOSE \
		PSYSAPI_TRACE_LEVEL_VERBOSE
#endif

#endif /* __IA_CSS_PSYSAPI_DATA_TRACE_H */
