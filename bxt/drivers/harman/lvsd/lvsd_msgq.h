/* lvsd_msgq.h - LVSD Message Queue Headers.
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

#ifndef __LVSD_MSGQ_H__
#define __LVSD_MSGQ_H__

/*------------------------------------------------------------------------------
 * INCLUDES
 *----------------------------------------------------------------------------*/
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/string.h>

/*------------------------------------------------------------------------------
 * MACROS
 *----------------------------------------------------------------------------*/
#define	EOK				0
#define ONEK_SCALER			(1000)
#define MILLI_TO_NANO_SEC(x)		((x) * (ONEK_SCALER) * (ONEK_SCALER))

#define LVSD_MSGQ_RSRC_SEM_AVAIL_INIT	(1 << 0)
#define LVSD_MSGQ_RSRC_SEM_FREE_INIT	(1 << 1)
#define LVSD_MSGQ_RSRC_MUTEX_INIT	(1 << 2)
#define LVSD_MSGQ_RSRC_MEM_INIT		(1 << 3)

enum {
	LVSD_TIMEOUT_INFINITE = -1,
	LVSD_NOBLOCK = 0
};

enum {
	LVSD_MSG_PRIORITY_NORMAL = 0,
	LVSD_MSG_PRIORITY_URGENT = 1
};

/*------------------------------------------------------------------------------
 * TYPEDEFS & STRUCTURES
 *----------------------------------------------------------------------------*/

typedef struct LvsdMsgQueueData *LvsdMsgQueue;
typedef struct LvsdMsgQueueData  LvsdMsgQueueData;

/**
 * @brief : Structure that contains all details of a Message Queue
 *
 * @head          : head position in queue
 * @tail          : tail position in queue
 * @uMaxElements  : Maximum number of elements that can be in queue at any moment
 * @uElemSize     : Size of an queue element
 * @uInQueue      : number of elements that are in queue at that moment
 * @tSemAvail     : the semaphore on which get call will be blocked on
 *                : if no data is available in queue
 * @tSemFree      : the semaphore on which put call will be blocked on to put
 *                  data in queue
 * @tMutex        : Queue locking mutex
 * @uRsrcStat     : Bitmap of resources allocated
 * @mem           : memory for storing message queue
 * @name          : name of the message queue
 */
struct LvsdMsgQueueData {
	int head;
	int tail;
	unsigned int uMaxElements;
	unsigned int uElemSize;
	unsigned int uInQueue;
	struct semaphore tSemAvail;
	struct semaphore tSemFree;
	struct mutex tMutex;
	unsigned int uRsrcStat;
	unsigned char *mem;
	char name[1];
};

/**
 * @brief   : Creates a message queue with the specified name.
 *            If the size is positive, that number of bytes is allocated for
 *            the queue on creation and the queue is limited to that size.
 * @param   : pId
 *            Place to store the new message queue id
 * @param   : maxMsgCount
 *		Maximum number of messages in queue
 * @param   : maxMsgSize
 * @param   : name
 * Returns  : EXXX macros defined in errno.h header file
 */
int LvsdMsgQueueCreate(LvsdMsgQueue *pId, unsigned int uMaxMsgCount,
	unsigned int uMaxMsgSize, const char *name);

/**
 * @brief   : Deletes message queue with the specified id.
 *
 * @param   : Id
 *            Queue Id of the message queue to be deleted
 * Returns  : EXXX macros defined in errno.h header file
 */
int LvsdMsgQueueDelete(LvsdMsgQueue id);

/**
 * @brief   : Puts a message into a message queue
 *
 * @param   : targetId
 *            Id of the Queue to which message needs to be put
 * @param   : msg
 *	Pointer to the message to place in the queue
 * @param   : length
 * @param   : priority
 *	Priority of the message
 * @param   : mstimeout
 *	Timeout in milliseconds, LVSD_TIMEOUT_INFINITE or LVSD_NOBLOCK
 * Returns  : EXXX macros defined in errno.h header file
 */
int LvsdMsgQueuePut(LvsdMsgQueue targetId, void *msg,
	unsigned int length, int priority, int mstimeout);

/** Get a message from the specified message queue. If a timeout is
 *            supplied, wait only that long for a message to become available.
 *            After that, return a -ETIMEDOUT error. If LVSD_TIMEOUT_INFINITE
 *            is passed, want forever for a message to become available.
 *            If LVSD_NOBLOCK  is passed, return an error immediately if a
 *            message is not available.
 *
 * @param   : pLength
 *            Place to return the length of the message retrieved
 * @param   : targetId
 *	Message queue id
 * @param   : msg
 *	Buffer to receive the message retrieved
 * @param   : maxLength
 *    Maximum length of message to receive (buffer size)
 * @param   : mstimeout
 *     Timeout in milliseconds, LVSD_TIMEOUT_INFINITE or LVSD_NOBLOCK
 * Returns  : EXXX macros defined in errno.h header file
 */
int LvsdMsgQueueGet(size_t *pLength, LvsdMsgQueue id, void *msg,
	unsigned int maxLength, int mstimeout);

/**
 * @brief   : Flushes all the pending messages from a message queue
 *
 * @param   : id
 *      id of message queue to flush
 * Returns  : EXXX macros defined in errno.h header file
 */
int LvsdMsgQueueFlush(LvsdMsgQueue id);

/**
 * @brief   : Modify the message entries in the message queue which match the token passed, with the newtoken
 *
 * @param   : tQid
 *	Id of message queue
 * @param   : oldtoken
 *	Token to match in the message queue entries
 * @param   : offset
 *	offset of the token to match in the message queue entry
 * @param   : newtoken
	Token which will replace the existing token
 * Returns  : EXXX macros defined in errno.h header file
 */
int LvsdMsgQueueModifyData(LvsdMsgQueue tQid, unsigned int *oldtoken,
	unsigned int offset, unsigned int newtoken);
#endif

