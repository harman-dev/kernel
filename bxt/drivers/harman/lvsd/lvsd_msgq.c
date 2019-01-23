/* lvsd_msgq.c - LVSD Message Queue Implementation.
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

/*------------------------------------------------------------------------------
 * INCLUDES
 *----------------------------------------------------------------------------*/
#include <linux/version.h>
#include <linux/time.h>
#include <linux/slab.h>
#include "lvsd_internal.h"
#include "lvsd_trace.h"

static void DestroyQueueObject(LvsdMsgQueue tQueueData)
{
	ENTER();

	if (LVSD_MSGQ_RSRC_MEM_INIT & tQueueData->uRsrcStat) {
		kfree(tQueueData->mem);
		tQueueData->mem = NULL;
		tQueueData->uRsrcStat &= ~LVSD_MSGQ_RSRC_MEM_INIT;
	}
	kfree(tQueueData);
	tQueueData = NULL;

	LEAVE();
}

static unsigned int IsQueueFull(LvsdMsgQueue tQ)
{
	ENTER();
	LEAVE();
	return tQ->uInQueue == tQ->uMaxElements;
}

static unsigned int IsQueueEmpty(LvsdMsgQueue tQ)
{
	ENTER();
	LEAVE();
	return tQ->uInQueue == 0;
}

static unsigned int GetNumOfQueueEntries(LvsdMsgQueue tQ)
{
	ENTER();
	LEAVE();
	return tQ->uInQueue;
}

static void QueueAdd(LvsdMsgQueue tTargetId, void *msg, size_t length, int priority)
{
	ENTER();

	if (LVSD_MSG_PRIORITY_NORMAL == priority) {
		memcpy((tTargetId->mem) + (tTargetId->tail) * (tTargetId->uElemSize), msg, length);
		tTargetId->tail++;
		if (tTargetId->tail == tTargetId->uMaxElements)
			tTargetId->tail = 0;
	}
	if (LVSD_MSG_PRIORITY_URGENT == priority) {
		tTargetId->head = (tTargetId->head + tTargetId->uMaxElements - 1) % (tTargetId->uMaxElements);
		memcpy((tTargetId->mem) + (tTargetId->head)*(tTargetId->uElemSize), msg, length);
	}
	tTargetId->uInQueue++;
	up(&tTargetId->tSemAvail); /* data available in queue */

	LEAVE();
}

static void QueueRemove(LvsdMsgQueue tTargetId, void *msg, size_t length)
{
	ENTER();

	memcpy(msg, tTargetId->mem + tTargetId->head * tTargetId->uElemSize, tTargetId->uElemSize);
	tTargetId->head++;
	if (tTargetId->head == tTargetId->uMaxElements)
		tTargetId->head = 0;
	tTargetId->uInQueue--;
	up(&tTargetId->tSemFree); /* record that something is free */

	LEAVE();
}

static int SemaphoreWait(struct semaphore *tSem, unsigned int uMsTimeOut)
{
	int rval = EOK;
	int result = 0;
	long jiffies;

	ENTER();

	if (LVSD_TIMEOUT_INFINITE == uMsTimeOut) {
		 if (down_interruptible(tSem))
			return -ERESTARTSYS;
	} else {
		jiffies = msecs_to_jiffies(uMsTimeOut);
		result = down_timeout(tSem, jiffies);
		if (0 > result)
			rval = -ETIMEDOUT;
	}
	LEAVE();
	return rval;
}

int LvsdMsgQueueCreate(LvsdMsgQueue *pId, unsigned int uMaxMsgCount, unsigned int uMaxMsgSize, const char *name)
{
	LvsdMsgQueue tnewQ = NULL;
	int rval = EOK;

	ENTER();

	tnewQ = kmalloc(sizeof(LvsdMsgQueueData) + strlen(name), GFP_KERNEL);
	LVSD_DEBUG("Message Queue Allocated");

	if (NULL != tnewQ) {
		memset(tnewQ, 0, sizeof(LvsdMsgQueueData));
		LVSD_DEBUG("Message Queue Initialized");
		strcpy(tnewQ->name, name);
		LVSD_DEBUG("Message Queue Name Assigned");
		tnewQ->mem = kmalloc((uMaxMsgCount * uMaxMsgSize), GFP_KERNEL);
		LVSD_DEBUG("Message Queue Mem Allocated");
		if (NULL != tnewQ->mem)	{
			tnewQ->uRsrcStat |= LVSD_MSGQ_RSRC_MEM_INIT;
			mutex_init(&tnewQ->tMutex);
			sema_init(&tnewQ->tSemAvail, 0);
			sema_init(&tnewQ->tSemFree, uMaxMsgCount);
			tnewQ->head = tnewQ->tail = 0 ;
			tnewQ->uMaxElements = uMaxMsgCount;
			tnewQ->uElemSize = uMaxMsgSize;
			tnewQ->uInQueue = 0;
			*pId = tnewQ;
			LVSD_DEBUG("Message Queue Semaphores, Mutex Initialized");
		} else
			rval = -ENOMEM;
	} else
		rval = -ENOMEM;
	if (EOK != rval)
		DestroyQueueObject(tnewQ);

	LEAVE();
	return rval;
}

int LvsdMsgQueueDelete(LvsdMsgQueue tQid)
{
	int  rval = EOK;

	ENTER();
	DestroyQueueObject(tQid);
	LEAVE();
	return rval;
}

int LvsdMsgQueuePut(LvsdMsgQueue targetId, void *msg, unsigned int length, int priority, int mstimeout)
{
	int   rval = EOK;

	ENTER();
	if (length > targetId->uElemSize)
		rval = -EINVAL;
	else {
		if ((LVSD_MSG_PRIORITY_NORMAL != priority) && (LVSD_MSG_PRIORITY_URGENT != priority))
			rval = -EINVAL;
		else {
			if (LVSD_NOBLOCK == mstimeout) {
				mutex_lock(&targetId->tMutex);
				if (!IsQueueFull(targetId)) {
					if (EOK != SemaphoreWait(&targetId->tSemFree, LVSD_NOBLOCK))
						rval = -EAGAIN;
					else
						QueueAdd(targetId, msg, length, priority);
					mutex_unlock(&targetId->tMutex);
				} else {
					rval = -ETIMEDOUT;
					mutex_unlock(&targetId->tMutex);
				}
			} else {
				rval = SemaphoreWait(&targetId->tSemFree, mstimeout);
				if (EOK == rval) {
					mutex_lock(&targetId->tMutex);
					QueueAdd(targetId, msg, length, priority);
					mutex_unlock(&targetId->tMutex);
				}
			}
		}
	}

	LEAVE();

	return rval;
}

int LvsdMsgQueueGet(size_t *pLength, LvsdMsgQueue tQid, void *msg, unsigned int uMsgLength, int mstimeout)
{
	int rval = EOK;

	ENTER();

	if (uMsgLength > tQid->uElemSize)
		rval = -EINVAL;
	if (LVSD_NOBLOCK == mstimeout) {
		mutex_lock(&tQid->tMutex);
		if (!IsQueueEmpty(tQid)) {
			if (EOK != SemaphoreWait(&tQid->tSemAvail, LVSD_NOBLOCK))
				rval = -EAGAIN;
			else
				QueueRemove(tQid, msg, uMsgLength);
			mutex_unlock(&tQid->tMutex);
		} else {
			rval = -ETIMEDOUT;
			mutex_unlock(&tQid->tMutex);
		}
	} else {
		rval = SemaphoreWait(&tQid->tSemAvail, mstimeout);
		if (EOK == rval) {
			mutex_lock(&tQid->tMutex);
			QueueRemove(tQid, msg, uMsgLength);
			mutex_unlock(&tQid->tMutex);
		}
	}
	LEAVE();
	return rval;
}

int LvsdMsgQueueFlush(LvsdMsgQueue tQid)
{
	int   rval = EOK;
	void *msg;

	ENTER();

	msg = kmalloc(tQid->uElemSize, GFP_KERNEL);

	if (!msg)
		rval = -ENOMEM;
	else {
		mutex_lock(&tQid->tMutex);
		while (GetNumOfQueueEntries(tQid))
			QueueRemove(tQid, msg, tQid->uElemSize);

		mutex_unlock(&tQid->tMutex);
		kfree(msg);
	}

	LEAVE();
	return rval;
}

int LvsdMsgQueueModifyData(LvsdMsgQueue tQid, unsigned int *oldtoken, unsigned int offset, unsigned int newtoken)
{
	int index, num_entries;
	int   rval = EOK;

	ENTER();

	mutex_lock(&tQid->tMutex);

	index = tQid->head;
	num_entries = GetNumOfQueueEntries(tQid);

	if (!num_entries) {
		LVSD_ERR("message queue is empty");
		rval = -EINVAL;
	}

	else {
		while (num_entries) {
			if (oldtoken == (unsigned int *)(tQid->mem + (index * tQid->uElemSize) + offset))
				*(unsigned int *)(tQid->mem + (index * tQid->uElemSize) + offset) = newtoken;
			index++;
			if (index == tQid->uMaxElements)
				index = 0;
			num_entries--;
		}
	}

	mutex_unlock(&tQid->tMutex);

	LEAVE();

	return rval;
}
