/* lvsd_trace.h - LVSD Traces and Logging headers.
 * Maintained by: Abhijit Lamsoge <abhijit.lamsoge@harman.com>
 * Copyright 2016 Harman International.
 *
 * This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 *
*/

#ifndef __LVSD_TRACE_H__
#define __LVSD_TRACE_H__

/*------------------------------------------------------------------------------
 * INCLUDES
 *----------------------------------------------------------------------------*/
#include <linux/kernel.h>

#define LVSD_DEBUG_MSG
#define LVSD_INFO_MSG
#define LVSD_NOTICE_MSG
#define LVSD_ERR_MSG
#define LVSD_DEBUG_VERBOSE

#ifdef LVSD_DEBUG_VERBOSE
#define LVSD_KERN_TRACE(priority, fmt, args...) \
	do {                                    \
		printk(priority "%s:: %d:: %s:: "fmt "\n", __FILE__, __LINE__, __FUNCTION__, ## args);       \
	} while (0)
#else

#define LVSD_KERN_TRACE(fmt, args...)
#endif


/*Emergency Messages*/
#ifdef LVSD_EMERG_MSG
#define LVSD_EMERG(fmt, args...) \
	do {                            \
		LVSD_KERN_TRACE(KERN_EMERG, fmt, ## args); \
	} while (0)
#else
#define LVSD_EMERG(fmt, args...)
#endif


/*Alert Messages*/
#ifdef LVSD_ALERT_MSG
#define LVSD_ALERT(fmt, args...) \
	do {                            \
		LVSD_KERN_TRACE(KERN_ALERT, fmt, ## args); \
	} while (0)
#else
#define LVSD_ALERT(fmt, args...)
#endif


/*Critical Messages*/
#ifdef LVSD_CRIT_MSG
#define LVSD_CRIT(fmt, args...) \
	do {                            \
		LVSD_KERN_TRACE(KERN_CRIT, fmt, ## args); \
	} while (0)
#else
#define LVSD_CRIT(fmt, args...)
#endif


/*Error Messages*/
#ifdef LVSD_ERR_MSG
#define LVSD_ERR(fmt, args...) \
	do {                            \
		LVSD_KERN_TRACE(KERN_ERR, fmt, ## args); \
	} while (0)
#else
#define LVSD_ERR(fmt, args...)
#endif


/*Warning Messages*/
#ifdef LVSD_WARN_MSG
#define LVSD_WARN(fmt, args...) \
	do {                             \
		LVSD_KERN_TRACE(KERN_WARNING, fmt, ## args); \
	} while (0)
#else
#define LVSD_WARN(fmt, args...)
#endif


/*Notice Messages*/
#ifdef LVSD_NOTICE_MSG
#define LVSD_NOTICE(fmt, args...) \
	do {                            \
		LVSD_KERN_TRACE(KERN_NOTICE, fmt, ## args); \
	} while (0)
#else
#define LVSD_NOTICE(fmt, args...)
#endif

/*Info Messages*/
#ifdef LVSD_INFO_MSG
#define LVSD_INFO(fmt, args...) \
	do {                             \
		LVSD_KERN_TRACE(KERN_INFO, fmt, ## args); \
	} while (0)
#else
#define LVSD_INFO(fmt, args...)
#endif


/*Debug Messages*/

#ifdef LVSD_DEBUG_MSG
#define LVSD_DEBUG(fmt, args...) \
	do {                             \
		LVSD_KERN_TRACE(KERN_DEBUG, fmt, ## args); \
	} while (0)
#else
#define LVSD_DEBUG(fmt, args...)
#endif

#ifdef LVSD_DEBUG_MSG
#define ENTER() \
	do {                             \
		LVSD_DEBUG("Enter"); \
	} while (0)
#define LEAVE() \
	do {                             \
		LVSD_DEBUG("Leave"); \
	} while (0)
#else
#define ENTER()
#define LEAVE()
#endif
#endif

