/* lvsd_serial.c - LVSD Serial Driver Core Implementation.
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
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/device.h>
#include <linux/tty_flip.h>
#include <linux/delay.h>

#include "lvsd_internal.h"
#include "lvsd_trace.h"

MODULE_AUTHOR("abhijit.lamsoge@harman.com");

/*------------------------------------------------------------------------------
 * MACROS
 *----------------------------------------------------------------------------*/
#define	LVSD_BUFFER_SIZE		12288

/* Writers on VSP are woken up, if number of free bytes in the Read Circular Buffer are more than 256*/
#define LVSD_FREE_CHARS_IN_RBUFFER	256

#define lvsd_uart_circ_chars_free(circ) \
	(CIRC_SPACE((circ)->head, (circ)->tail, LVSD_BUFFER_SIZE))

/*globals required for wbuffer maintainance*/
size_t g_count;
unsigned int g_fill_level;

/* The Global Character Device Structure, which has the Message Queue, Devices List etc.*/
extern tLvsd_Char_Dev_t vsp_char_device;
extern struct workqueue_struct *lvsd_workqueue_struct;

static int uart_chars_in_buffer(struct tty_struct *tty);
/*
 * This is used to lock changes in serial line configuration.
 */
static DEFINE_MUTEX(port_mutex);


/*This function checks the amount of data in Rbuffer and re-triggers a new VSAL_DATA receive event.*/
int trigger_pending_write(tLvsd_Uart_Port_t *up, unsigned int write_flag)
{
	struct uart_port *uport;
	struct circ_buf *circ;
	int ret = 0, event_created = 0;
	unsigned int data_pointer;
	unsigned int residual_data;
	unsigned long flags;
	tLvsd_Event_Entry_t data_event;

	if(!up) {
		LVSD_DEBUG("Somebody called a Pending write before the handle was even created");
		return -1;
	}

	circ = &up->rbuffer;
	uport = &up->port;

	LVSD_DEBUG("Inside trigger pending write");
	ENTER();

	if (!circ) {
		LVSD_DEBUG("Circ buffer acquire error");
		LVSD_ERR("VSP's Rbuffer is NULL or no Uart Port");
		return -1;
	}
	
	if (!uport) {
		LVSD_DEBUG("uport acquire error");
		LVSD_ERR("VSP's Rbuffer is NULL or no Uart Port");
		return -1;
	}

	LVSD_DEBUG("Acquiring spinlock for pending write");

	spin_lock_irqsave(&uport->lock, flags);

	LVSD_DEBUG("circ->tail: %d and circ->head: %d", circ->tail, circ->head);

	if (circ->head != circ->tail) {
		/*Some data is remaining in the Rbuffer*/
		data_pointer = circ->head;
		
		LVSD_DEBUG("Circular buffer has something");
		/*Calculate data remaining in Rbuffer to be re-triggered again*/
		residual_data = circ->head - circ->tail;
		if (residual_data) {
			LVSD_DEBUG("There is some valid residual data in Rbuffer - Sending event to upper layer for the same - Around %d",residual_data);
			data_event.vsp_handle = up;
			data_event.event = LVSD_EVENT_VSP_DATA;
			data_event.data_offset = data_pointer;
			data_event.size = residual_data;		
			//uart_circ_clear(&up->rbuffer);
			event_created = 1;
		}
			
	}
	
	spin_unlock_irqrestore(&uport->lock, flags);

	if (event_created) {
		LvsdMsgQueuePut(vsp_char_device.message_queue, &data_event, sizeof(tLvsd_Event_Entry_t), LVSD_MSG_PRIORITY_NORMAL, LVSD_TIMEOUT_INFINITE);
		ret = 1;
	}
		
	return ret;
	LEAVE();
	
}


/* This function updates the data read count in the RBuffer (Circular Buffer) - the data written from write, call, goes to VSAL and to BSS, eventually, BSS returns how much it has consumed and send update via VSAL to update Rbuffer*/
void update_circ_read_cnt(tLvsd_Uart_Port_t *up, unsigned int read_cnt)
{
	unsigned long flags;
	struct uart_port *uport;
	struct circ_buf *circ;

	ENTER();

	LVSD_DEBUG("Read count to update in Rbuffer is %d", read_cnt);
	/* Get the Circular buffer and the uart_port pointers*/
	circ = &up->rbuffer;
	uport = &up->port;

	if (!circ || !uport) {
		LVSD_ERR("VSP's Rbuffer or Uart Port is NULL");
		return;
	}

	/* Update the Tail in the Circular Buffer*/
	spin_lock_irqsave(&uport->lock, flags);

	LVSD_DEBUG("Circular tail before update is %d", circ->tail);
	circ->tail = (circ->tail + read_cnt) & (LVSD_BUFFER_SIZE - 1); /*incrementing tail as, item is removed from Circ-buffer*/
	LVSD_DEBUG("circ->tail: %d and circ->head: %d", circ->tail, circ->head);
	spin_unlock_irqrestore(&uport->lock, flags);
	LVSD_DEBUG("No. of Free bytes in Circular Buffer are %d", lvsd_uart_circ_chars_free(circ));
	if (lvsd_uart_circ_chars_free(circ) > LVSD_FREE_CHARS_IN_RBUFFER) {
		LVSD_DEBUG("Waking up writers on VSP");
		uart_write_wakeup(uport); /*using standard uart_write_wakeup, to wake writers on VSP,which internally calls, tty_wakeup(tty) , so we do not need tasklet for that.*/
	}
	LEAVE();
}

/* Get the uart_state, corresponding to the "line" of the uart_driver "drv"*/
static struct uart_state *uart_get(struct uart_driver *drv, int line)
{
	struct uart_state *state;
	struct tty_port *port;
	int ret = 0;

	ENTER();

	if (!drv->state) {
		LVSD_DEBUG("Uart Driver has no corresponding Uart State");
		return NULL;
	}

	/* Get the state and the tty port corresponding to the "line" of the uart_driver "drv"*/
	state = drv->state + line;

	port = &state->port;

	if (!port) {
		LVSD_DEBUG("No Corresponding TTY Port available in the Uart State");
		return NULL;
	}

	if (mutex_lock_interruptible(&port->mutex)) {
		ret = -ERESTARTSYS;
		goto err;
	}

	/* Increment the count variable for that tty port*/
	port->count++;
	/* If there is no corresponding uart_port for the state or if the uart_port is already removed from the driver structure then decrement the count variable for that tty port*/
	if (!state->uart_port || state->uart_port->flags & UPF_DEAD) {
		ret = -ENXIO;
		goto err_unlock;
	}

	LEAVE();
	return state;
err_unlock:
	port->count--;
	mutex_unlock(&port->mutex);
 err:
	return ERR_PTR(ret);
}

/**/
int store_wbuff_data(tLvsd_Uart_Port_t *vsp, size_t count, unsigned int fill_level)
{
	tLvsd_Event_Entry_t write_event;

	g_count = count;
	g_fill_level = fill_level;

	LVSD_DEBUG("BEfore updating our count our TTY Wbuffer is %p", vsp->wbuffer);

	/*Make sure that Write from vsal does no cross the 12KB limit of MMaped page*/
	if (g_fill_level < LVSD_VSAL_MAX_WRITE_SIZE) {
		/*Update the Wbuffer , use a lock here*/
		mutex_lock(&(vsp->wbuffer_mutex));
		vsp->wbuffer = vsp->wbuffer + g_count;
		mutex_unlock(&(vsp->wbuffer_mutex));
		LVSD_DEBUG("After updating with count our TTY Wbuffer is %p", vsp->wbuffer);
	} else {
		LVSD_DEBUG("Wbuffer is full with data, so no more writes are possible");
		return 0;
	}

	/*Send Event to VSAL to Update its WBuffer*/
	write_event.vsp_handle = vsp;
	write_event.event = LVSD_EVENT_VSP_WRITE;
	write_event.data_offset = g_fill_level;
	write_event.size = g_count;

	/*Push the Write Event in to the Message Queue*/
	LvsdMsgQueuePut(vsp_char_device.message_queue, &write_event, sizeof(tLvsd_Event_Entry_t), LVSD_MSG_PRIORITY_NORMAL, LVSD_TIMEOUT_INFINITE);
	return count;
}


/*If the write from VSAL side is done before there is a TTY for our Uart serial device, hold flushing of WBuffer to TTY buffer unless, we receive a uart_open for the device*/
unsigned int buffer_data(tLvsd_Uart_Port_t *vsp, size_t count, unsigned int current_fill_level)
{
	struct uart_state *state;
	struct tty_struct *tty;

	state = vsp->port.state;
	if (!state) {
		LVSD_ERR("No state for current VSP");
		return 0;
	}

	tty = state->port.tty;
	if (!tty) {
		LVSD_ERR("VSP still not open");
		return 0;
	}

	/*If data keeps getting buffer, till nobody read it, we need to pass fill level as count*/
	count = current_fill_level;

	/*Call receive_chars here*/
	if (receive_chars(vsp, count, current_fill_level) > 0)
		LVSD_DEBUG("Successfully sent the Vsal Wbuff data to TTY buffers");
	else
		LVSD_DEBUG("Failed miserably to do internal buffer of VSAL write data");

	return 0;
}

/* When there is a "write" on the Character Device(ctrlX), the data updated in the Mmapped "Wbuffer" is copied to the TTY buffers (which are of 256 bytes size each)
 * After every TTY buffer fills up, "tty_flip_buffer_push" is called, which copies the data from the TTY buffers to the RBUF of the VSP ?? shldn't this be WBuff.
 * When a Serial Application reads the data from the VSP using a "read", the data in the TTY Buffer will be copied to the User Serial Application - via flip-buffer mechanism of TTY Core
 * This is like an interrupt function, which is called, when data written comes back to us from hardware like VSAL*/
unsigned int receive_chars(tLvsd_Uart_Port_t *vsp, size_t count, unsigned int current_wbuf_fill_level)
{
	char ch, *buffer;
	unsigned int char_count, max_count, free_in_ttybuf;
	unsigned long flags, port_flags;
	struct tty_struct *tty;
	struct tty_buffer *tb;
	struct uart_state *state;
	struct tty_port *tty_port;

	/* "count" specifies the amount of data present in the Wbuffer that has to be copied to the RBUF
	* after copying the data from the Wbuffer to the TTY buffer, if the "max_count" reaches 256, then the data from the TTY buffer is copied to the RBUF of the VSP
	* "char_count" specifies the amount of data that has been copied to the TTY buffers from the Wbuffer of the VSP
	* "free_in_ttybuf" is the amount of freespace available in the TTY buffer which is at the tail of the TTY buffer linked list*/

	ENTER();
	LVSD_DEBUG("Amount of Data to be written: %d", (int)count);
	char_count = max_count = free_in_ttybuf = 0;

	if (!vsp) {
		LVSD_ERR("VSP is already destroyed");
		return 0;
	}

	state = vsp->port.state;

	if (!state) {
		LVSD_ERR("No state exists for the VSP");
		return 0;
	}

	tty = state->port.tty;
	tty_port = &state->port;
	if (!tty) {
		LVSD_ERR("VSP is already closed");
		return 0;
	}

	LVSD_DEBUG("tty : %p", tty);

	spin_lock_irqsave(&tty->ctrl_lock, flags);
	tb = tty->port->buf.tail;
	spin_unlock_irqrestore(&tty->ctrl_lock, flags);

	LVSD_DEBUG("vsp->wbuffer: %p", vsp->wbuffer);

	buffer = vsp->wbuffer;
	/* The data from the VSAL on the character device interface is copied to the tail of the TTY buffer linked list.
	* The data in the head of the TTY buffer linked list is copied to the RBUF of the VSP. TTY buffers are created based on the requirement */
	/* If a TTY buffer is present at the tail of the TTY buffer linked list. i.e. already some data has been copied from the Wbuffer to the TTY buffer*/
	if (tb) {
		/* Calculate the amount of free space in the TTY buffer*/
		LVSD_DEBUG("tb->size = %d, tb->used = %d", tb->size, tb->used);
		free_in_ttybuf = tb->size - tb->used;

		LVSD_DEBUG("Amount of space available in TTY Buffer is %d", free_in_ttybuf);

		/* If the amount of data that has to be copied from Wbuffer to TTY buffer, is more than or equal to the amount of free space in the TTY buffer*/
		if (count >= free_in_ttybuf) {
			spin_lock_irqsave(&vsp->port.lock, port_flags);
			/* Copy the amount of freespace sized data from the Wbuffer of the VSP, to the TTY buffer. Reduce the "count", "free_in_ttybuf".*/
			/* Increase the "char_count"*/
			while (free_in_ttybuf) {
				ch = *buffer++;
				tty_insert_flip_char(tty_port, ch, 0);
				count--;
				free_in_ttybuf--;
				char_count++;
			}
			spin_unlock_irqrestore(&vsp->port.lock, port_flags);

		}
		/* If the amount of data that has to be copied from Wbuffer to TTY buffer, is less than the amount of free space in the TTY buffer*/
		else {
			spin_lock_irqsave(&vsp->port.lock, port_flags);
			/* Copy the "count" number of bytes from the Wbuffer to TTY buffer and return the "char_count"*/
			while (count) {
				ch = *buffer++;
				tty_insert_flip_char(tty_port, ch, 0);
				count--;
				char_count++;
			}
			spin_unlock_irqrestore(&vsp->port.lock, port_flags);
			LVSD_DEBUG("Triggering Finish ");
			goto finish;
		}
	}
	/* If a TTY buffer is not available at the tail of the TTY buffer linked list. i.e. the first time data is copied from the Wbuffer to the TTY buffer
	* or If the amount of data that has to be copied from the Wbuffer to the TTY buffer
	* is beyond the amount of freespace in the TTY buffer which is at the tail of the TTY buffer linked list */
	while (count) {
		LVSD_DEBUG("Initial char count is %d and max count is %d", char_count, max_count);
		spin_lock_irqsave(&vsp->port.lock, port_flags);
		ch = *buffer++;
		LVSD_DEBUG("tty : %p", tty);
		tty_insert_flip_char(tty_port, ch, 0);
		/* After copying each byte of data from the Wbuffer to the TTY buffer, increase the "max_count", "char_count" and decrease the "count"*/
		count--;
		char_count++;
		max_count++;
		spin_unlock_irqrestore(&vsp->port.lock, port_flags);

		if (count == 0) {/*All characters are inserted in TTY flip buffers*/
			LVSD_DEBUG("New tty flip buffer push");
			tty_flip_buffer_push(tty_port);
			return char_count;
		}

	}
finish:
	tty_flip_buffer_push(tty_port);

	LEAVE();
	/* Return the number of bytes copied from the Wbuffer of the VSP to the TTY buffers*/
	return char_count;
}

/* Update the Modem control value of a VSP*/
void uart_update_mctrl(struct uart_port *port, unsigned int set, unsigned int clear)
{
	unsigned int old;
	struct uart_state *state = NULL;
	struct tty_port *ttyport = NULL;

	ENTER();

	state = port->state;

	if (NULL == state) {
		LVSD_ERR("Invalid Uart State");
		return;
	}

	ttyport = &state->port;

	if (NULL == ttyport) {
		LVSD_ERR("Invalid Uart TTY Port");
		return;
	}

	spin_lock_irq(&port->lock);

	old = port->mctrl;
	port->mctrl = (old & ~clear) | set;

	LVSD_DEBUG("VSP->Port: %p, VSP->Port->Mctrl: 0x%x", port, port->mctrl);

	spin_unlock_irq(&port->lock);

	LVSD_DEBUG("Waking up the Waiters for the Modem Status Update");
	wake_up_interruptible(&ttyport->delta_msr_wait);
	LVSD_DEBUG("Woke up the Waiters for the Modem Status Update");

	LEAVE();
}

#define uart_set_mctrl(port, set)	uart_update_mctrl(port, set, 0)
#define uart_clear_mctrl(port, clear)	uart_update_mctrl(port, 0, clear)

/* Get the Modem control value of a VSP*/
unsigned int uart_get_mctrl(struct uart_port *port)
{
	unsigned int mctrl;

	ENTER();
	spin_lock_irq(&port->lock);
	mctrl = port->mctrl;
	LVSD_DEBUG("VSP->Port: %p, VSP->Port->Mctrl: 0x%x", port, port->mctrl);
	spin_unlock_irq(&port->lock);

	LEAVE();
	return mctrl;
}

/* Get the amount of Freespace available in the RBUF of a VSP*/
unsigned int uart_get_freespace(struct uart_port *port)
{
	struct uart_state *state = NULL;
	struct tty_struct *tty = NULL;
	unsigned int freespace;
	unsigned long flags;

	ENTER();

	if (!port || !port->state) {
		LVSD_ERR("Invalid Uart Port or Uart State");
		return 0;
	}

	state = port->state;

	if (!state->port.tty) {
		LVSD_ERR("Uart Port already closed");
		return 0;
	}

	tty = state->port.tty;

	LVSD_DEBUG("tty : %p", tty);

	spin_lock_irqsave(&tty->ctrl_lock, flags);
	freespace = tty->receive_room;
	spin_unlock_irqrestore(&tty->ctrl_lock, flags);

	LVSD_DEBUG("tty: %p, freespace: %d", tty, freespace);

	LEAVE();
	return freespace;
}

/* The "throttle" function*/
static void uart_throttle(struct tty_struct *tty)
{
	ENTER();
	LVSD_DEBUG("tty : %p", tty);
	LEAVE();
}

/* The "unthrottle" function which creates an event (Buffer state change event) in the Message Queue. This will be later read by the VSAL.*/
static void uart_unthrottle(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *uport = state->uart_port;
	tLvsd_Uart_Port_t *up;
	tLvsd_Event_Entry_t bsc_event;

	ENTER();

	LVSD_DEBUG("tty : %p", tty);

	if (!uport) {
		LVSD_ERR("VSP already destroyed");
		return;
	}

	up = (tLvsd_Uart_Port_t *)uport;

	/*Create an event (Buffer state change event)*/
	bsc_event.vsp_handle = up;
	bsc_event.event = LVSD_EVENT_VSP_BUFFER_STATE_CHANGE;
	bsc_event.data_offset = 0;
	bsc_event.size = 0;

	/* Add the event created, to the Message Queue*/
	LvsdMsgQueuePut(vsp_char_device.message_queue, &bsc_event, sizeof(tLvsd_Event_Entry_t), LVSD_MSG_PRIORITY_NORMAL, LVSD_TIMEOUT_INFINITE);

	LEAVE();
}

/* Used by the TTY core, TTY Line discipline to know about the amount of freespace available in the Rbuffer (circular buffer) of a VSP*/
static int uart_write_room(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
	tLvsd_Uart_Port_t *up;
	unsigned long flags;
	int ret;

	ENTER();

	LVSD_DEBUG("tty : %p", tty);

	/* This means that the function has been called after the port was closed*/
	if (!state) {
		LVSD_ERR("VSP already destroyed");
		return -ENODEV;
	}

	if (!state->uart_port) {
		LVSD_ERR("No Valid Uart Port in the Uart State");
		return -EIO;
	}
	/* Checking tty device*/
	if (!tty->dev) {
		LVSD_ERR("No TTY device available for writing");
		return -ENODEV;
	}

	up = (tLvsd_Uart_Port_t *)(state->uart_port);

	/* Return the amount of freespace available in the Rbuffer (circular buffer) of the VSP*/
	spin_lock_irqsave(&state->uart_port->lock, flags);
	ret = lvsd_uart_circ_chars_free(&up->rbuffer);
	spin_unlock_irqrestore(&state->uart_port->lock, flags);
	LVSD_DEBUG("%d free space in circular/rbuffer", ret);

	LEAVE();
	return ret;
}

/* Used by the TTY core, TTY Line discipline to know about the amount of data present in the Rbuffer (circular buffer) of a VSP*/
static int uart_chars_in_buffer(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
	tLvsd_Uart_Port_t *up;
	unsigned long flags;
	int ret;

	ENTER();
	LVSD_DEBUG("tty : %p", tty);

	/* This means that the function has been called after the port was closed*/
	if (!state) {
		LVSD_ERR("VSP already destroyed");
		return 0;
	}

	if (!state->uart_port) {
		LVSD_ERR("No Valid Uart Port in the Uart State");
		return 0;
	}

	up = (tLvsd_Uart_Port_t *)(state->uart_port);
	if (!up) {
		LVSD_ERR("VSP already destroyed");
		return 0;
	}

	/*Return the amount of data present in the Rbuffer (circular buffer) of the VSP*/
	spin_lock_irqsave(&state->uart_port->lock, flags);
	ret = uart_circ_chars_pending(&up->rbuffer);
	spin_unlock_irqrestore(&state->uart_port->lock, flags);

	LEAVE();
	return ret;
}

/* This function is used to get the Modem control value of a VSP by the Serial Application which opens that VSP*/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38)
static int uart_tiocmget(struct tty_struct *tty)
{
#else
static int uart_tiocmget(struct tty_struct *tty, struct file *file)
{
#endif
	struct uart_state *state = tty->driver_data;
	struct tty_port *port = &state->port;
	struct uart_port *uport = state->uart_port;
	int result = -EIO;

	ENTER();
	LVSD_DEBUG("tty : %p", tty);

	/* Return the Modem control value of the VSP*/
	mutex_lock(&port->mutex);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38)
   if (!(tty->flags & (1 << TTY_IO_ERROR))) {
#else
	if (!file || !tty_hung_up_p(file)) {
#endif
		result = uart_get_mctrl(uport);
	}
	mutex_unlock(&port->mutex);

	LEAVE();
	return result;
}

/* This function is used to set the Modem control value of a VSP by the Serial Application which opens that VSP.
 * After setting the Modem control value, an event (Control event) is created and pushed in to the Message Queue */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38)
static int uart_tiocmset(struct tty_struct *tty, unsigned int set, unsigned int clear)
{
#else
static int uart_tiocmset(struct tty_struct *tty, struct file *file, unsigned int set, unsigned int clear)
{
#endif
	struct uart_state *state = tty->driver_data;
	struct uart_port *uport = state->uart_port;
	struct tty_port *port = &state->port;
	tLvsd_Uart_Port_t *up;
	tLvsd_Event_Entry_t control_event;

	int ret = -EIO;

	ENTER();
	LVSD_DEBUG("tty : %p, set: 0x%x, clear: 0x%x", tty, set, clear);

	up = (tLvsd_Uart_Port_t *)uport;

	/* Set the Modem control value of the VSP and create a Control event*/
	mutex_lock(&port->mutex);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38)
	if (!(tty->flags & (1 << TTY_IO_ERROR))) {
#else
	if (!file || !tty_hung_up_p(file)) {
#endif
		uart_update_mctrl(uport, set, clear);
		control_event.vsp_handle = up;
		control_event.event = LVSD_EVENT_VSP_CONTROL;
		control_event.data_offset = uport->mctrl;
		control_event.size = sizeof(uport->mctrl);

		/* Push the created Control event in to the Message Queue which will be read by the VSAL*/
		LvsdMsgQueuePut(vsp_char_device.message_queue, &control_event, sizeof(tLvsd_Event_Entry_t), LVSD_MSG_PRIORITY_NORMAL, LVSD_TIMEOUT_INFINITE);
		ret = 0;
	}
	mutex_unlock(&port->mutex);

	LEAVE();
	return ret;
}

/* This function is called at the time of uart_close or uart_hangup*/
static void uart_flush_buffer(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *uport;
	tLvsd_Uart_Port_t *up;
	unsigned long flags;

	ENTER();

	LVSD_DEBUG("tty : %p", tty);

	/* This means that the function has been called after the port was closed*/
	if (!state) {
		WARN_ON(1);
		return;
	}

	uport = state->uart_port;
	if (!uport) {
		LVSD_ERR("VSP already destroyed");
		return;
	}

	up = (tLvsd_Uart_Port_t *)uport;

	LVSD_DEBUG("uart_flush_buffer(%d) called", tty->index);

	/*set the head and tail as the same in the Rbuffer (circular buffer)*/
	spin_lock_irqsave(&uport->lock, flags);
	uart_circ_clear(&up->rbuffer);
	spin_unlock_irqrestore(&uport->lock, flags);

	/* Wakeup the writers on the VSP*/
	tty_wakeup(tty);

	LEAVE();
}


static void uart_port_shutdown(struct tty_port *port)
{
	struct uart_state *state = container_of(port, struct uart_state, port);
	struct uart_port *uport = state->uart_port;
	/** clear delta_msr_wait queue to avoid mem leaks: we may free
	* the irq here so the queue might never be woken up.  Note
	* that we won't end up waiting on delta_msr_wait again since
	* any outstanding file descriptors should be pointing at
	* hung_up_tty_fops now.
	*/
	wake_up_interruptible(&port->delta_msr_wait);

	/*
	* Free the IRQ and disable the port.
	*/
	uport->ops->shutdown(uport);
	/*
	* Ensure that the IRQ handler isn't running on another CPU.
	*/
	synchronize_irq(uport->irq);
}

static void uart_shutdown(struct tty_struct *tty, struct uart_state *state)
{
	struct uart_port *uport = state->uart_port;
	struct tty_port *port = &state->port;

	/*
	 * Set the TTY IO error marker
	 */
	if (tty)
		set_bit(TTY_IO_ERROR, &tty->flags);

	if (test_and_clear_bit(ASYNCB_INITIALIZED, &port->flags)) {
		/*
		 * Turn off DTR and RTS early.
		 */
		if (!tty || (tty->termios.c_cflag & HUPCL))
			uart_clear_mctrl(uport, TIOCM_DTR | TIOCM_RTS);

		uart_port_shutdown(port);
	}

	/*
	 * It's possible for shutdown to be called after suspend if we get
	 * a DCD drop (hangup) at just the right time.  Clear suspended bit so
	 * we don't try to resume a port that has been shutdown.
	 */
	clear_bit(ASYNCB_SUSPENDED, &port->flags);

	/*
	 * Free the transmit buffer page.
	 *
	if (state->xmit.buf) {
		free_page((unsigned long)state->xmit.buf);
		state->xmit.buf = NULL;
	}*/
}



/*
 * This is called with the BKL held in
 *  linux/drivers/char/tty_io.c:do_tty_hangup()
 * We're called from the eventd thread, so we can sleep for
 * a _short_ time only.
 */
static void uart_hangup(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
	struct tty_port *port = &state->port;
	unsigned long flags;

	ENTER();

	LVSD_DEBUG("tty : %p", tty);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
	BUG_ON(!kernel_locked());
#elif LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)
	BUG_ON(!tty_locked());
#endif

	LVSD_DEBUG("uart_hangup(%d)", state->uart_port->line);

	mutex_lock(&port->mutex);
	if (port->flags & ASYNC_NORMAL_ACTIVE) {
		uart_flush_buffer(tty);\
		/*tasklet kill, is dropped, and adjusting by disabling interrupts, as there is nothing cause waking tty writers will not be done here*/
		uart_shutdown(tty, state);
		spin_lock_irqsave(&port->lock, flags);
		port->count = 0;
		clear_bit(ASYNCB_NORMAL_ACTIVE, &port->flags);
		spin_unlock_irqrestore(&port->lock, flags);

		tty_port_tty_set(port, NULL);
		wake_up_interruptible(&port->open_wait);
		wake_up_interruptible(&port->delta_msr_wait);
	}
	mutex_unlock(&port->mutex);

	LEAVE();
}

#if 0
/* Work to push the data event in to the Message Queue which will be read by the VSAL*/
static void do_lvsd_work(struct work_struct *work)
{
	tLvsd_Work_Queue_Struct_Event_t *local_wq_struct_event;
	tLvsd_Event_Entry_t *lvsd_event;

	ENTER();

	if (work) {
		local_wq_struct_event = container_of(work, tLvsd_Work_Queue_Struct_Event_t, lvsd_work_struct);

		if (!local_wq_struct_event) {
			LVSD_ERR("No Work Structure Event in the Argument 'work'");
			return;
		}

		lvsd_event = &(local_wq_struct_event->lvsd_event);

		/*Push the data event in to the Message Queue which will be read by the VSAL*/
		LvsdMsgQueuePut(vsp_char_device.message_queue, lvsd_event, sizeof(tLvsd_Event_Entry_t), LVSD_MSG_PRIORITY_NORMAL, LVSD_TIMEOUT_INFINITE);

		kfree(local_wq_struct_event);
	} else {
		LVSD_ERR("No Work Structure in the Argument");
		return;
	}

	LEAVE();
}
#endif

/* This function is called when a Serial Application writes on a VSP - this data will be written to RBuff*/
static int uart_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *port;
	struct circ_buf *circ;
	tLvsd_Uart_Port_t *up;
	tLvsd_Event_Entry_t data_event;
	//tLvsd_Work_Queue_Struct_Event_t *work_queue_struct_event;
	unsigned int data_pointer;
	unsigned long flags;
	int c = 0, ret = 0, spaceinrbuff;

	ENTER();


	LVSD_DEBUG("tty: %p, buf: %p count: %d", tty, buf, count);

	/* This means that the function has been called after the port was closed*/
	if (!state) {
		LVSD_ERR("VSP already destroyed");
		return -ENODEV;
	}

	if (!state->uart_port) {
		LVSD_ERR("No Valid Uart Port in the Uart State");
		return -EIO;
	}

	/* Checking tty device*/
	if (!tty->dev) {
		LVSD_ERR("No TTY device available for writing");
		return -ENODEV;
	}

	/* This means that the function has been called after the port was closed
	if (!state) {
		WARN_ON(1);
		return -EL3HLT;
	}*/

	port = state->uart_port;
	up = (tLvsd_Uart_Port_t *)port;

	circ = &up->rbuffer;

	if (!circ->buf) /*buffer cannot be empty*/
		return 0;

	spin_lock_irqsave(&port->lock, flags);

	data_pointer = circ->head;
	LVSD_DEBUG(" circ-head or data_pointer: %d", data_pointer);
	LVSD_DEBUG("circ->tail: %d", circ->tail);

	/*Data does not come directly to RBuffer, we have to copy that
	* copy the data written by the serial application on the VSP to the Rbuffer (circular buffer) at the head pointer.*/
	while (1) {
		/*Freespace Available in RBuffer*/
		spaceinrbuff = lvsd_uart_circ_chars_free(circ);//CIRC_SPACE_TO_END(circ->head, circ->tail, LVSD_BUFFER_SIZE);
		//spaceinrbuff = CIRC_SPACE_TO_END(circ->head, circ->tail, LVSD_BUFFER_SIZE);
		LVSD_DEBUG("Current Possible usable space in circular RBuff is %d", spaceinrbuff);
		if (spaceinrbuff > count) {
			LVSD_DEBUG("We should always reach here");
			LVSD_DEBUG("We need to copy %d bytes to RBuffer",count);
			c = count;
			if (c <= 0) {
				LVSD_DEBUG("All Bytes in this iteration copied to RBuffer");
				break;
			}
			memcpy(circ->buf + circ->head, buf, c);
			circ->head = (circ->head + c); /*Do Not wrap around the circular buffer as it causes writes greater that 2K to overwrite our RBuff causing data loss*/
			/*Adjust buffer pointer and count values*/
			buf += c;
			count -= c;
			ret += c;
	
		} else {
			LVSD_DEBUG("No Free Space left in RBuffer");
			ret = -1;
			break;
		}

		LVSD_DEBUG("Copying Data to Rbuffer of VSP");

	}
		LVSD_DEBUG(" circ-head or data_pointer: %d", circ->head);

	spin_unlock_irqrestore(&port->lock, flags);
	/* If data has been copied to the Rbuffer, create a Data event*/
	if (ret != -1) {
		data_event.vsp_handle = up;
		data_event.event = LVSD_EVENT_VSP_DATA;
		data_event.data_offset = data_pointer;
		data_event.size = ret;
#if 0
		/* if in case, uart_write() is called from a function which already holds a spinlock, then it executes the code under in_atomic()*/
		if (in_interrupt()) { /*Check what we can do here may be in_interrupt() ??*/
			LVSD_DEBUG("Workqueue atomic context");
			/* Schedule a work to push the data event in to the Message Queue which will be read by the VSAL*/
			work_queue_struct_event = kmalloc(sizeof(*work_queue_struct_event), GFP_ATOMIC);

			if (!work_queue_struct_event) {
				ret = -ENOMEM;
				LVSD_ERR("Not able to allocate work queue struct event");
				return ret;
			}

			INIT_WORK(&work_queue_struct_event->lvsd_work_struct, &do_lvsd_work);

			memcpy(&work_queue_struct_event->lvsd_event, &data_event, sizeof(tLvsd_Event_Entry_t));

			queue_work(lvsd_workqueue_struct, &work_queue_struct_event->lvsd_work_struct);
		} else {
#endif
			/* Push the data event in to the Message Queue which will be read by the VSAL*/
			LvsdMsgQueuePut(vsp_char_device.message_queue, &data_event, sizeof(tLvsd_Event_Entry_t), LVSD_MSG_PRIORITY_NORMAL, LVSD_TIMEOUT_INFINITE);
//		}
	}

	LEAVE();

	return ret;
}

/* This function is called when a Serial Application opens a VSP*/
static int uart_open(struct tty_struct *tty, struct file *filp)
{
	tLvsd_Event_Entry_t open_event;
	struct uart_driver *drv = (struct uart_driver *)tty->driver->driver_state;
	struct uart_state *state;
	struct tty_port *port;
	tLvsd_Uart_Port_t *up;

	int retval, line = tty->index;

	ENTER();
	LVSD_DEBUG("tty : %p", tty);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
	BUG_ON(!kernel_locked());
#elif LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)
	BUG_ON(!tty_locked());
#endif

	LVSD_DEBUG("uart_open(%d) called", line);

	retval = -ENODEV;
	if (line >= tty->driver->num)
		goto fail;

	/*
	 * We take the mutex inside uart_get to guarantee that we won't
	 * be re-entered while allocating the state structure. This also has
	 * the nice side-effect that it delays the action of uart_hangup, so we can
	 * guarantee that state->port.tty will always contain something
	 * reasonable.
	 */
	state = uart_get(drv, line);
	if (IS_ERR(state)) {
		retval = PTR_ERR(state);
		goto fail;
	}
	port = &state->port;/*debug - why twice ??*/

	/*
	 * Once we set tty->driver_data here, we are guaranteed that
	 * uart_close() will decrement the driver module use count.
	 * Any failures from here onwards should not touch the count.
	 */
	tty->driver_data = state;
	state->uart_port->state = state;

	tty->alt_speed = 0;
	tty_port_tty_set(port, tty);

	/*
	 * If the port is in the middle of closing, bail out now.
	 */
	if (tty_hung_up_p(filp)) {
		retval = -EAGAIN;
		port->count--;
		mutex_unlock(&port->mutex);
		goto fail;
	}

	/* Set the flags that the port has been initialized*/
	set_bit(ASYNCB_INITIALIZED | ASYNCB_NORMAL_ACTIVE, &port->flags);


	mutex_unlock(&port->mutex);

	up = (tLvsd_Uart_Port_t *)(state->uart_port);

	/*Try to call receive_chars here so that on first open the data gets pushed to TTY buffers*/
	if ((port->count == 1)) {
		LVSD_DEBUG("Entered pushbuffer logic as this is first open of our serial mount point");
		/*Bring the wbuffer pointer to Wbuffer's original Location*/
		up->wbuffer = up->wbuffer - g_fill_level;
		if (buffer_data(up, g_count, g_fill_level)) {
			/*Reset fill level and wbuffer after this*/
			LVSD_DEBUG("Resetting Wbuffer's fill level");
			atomic_set(&(up->wbuf_fill_level), 0);
			g_fill_level = 0;
			g_count = 0;
		}
	}

	/* Create an Open event*/
	open_event.vsp_handle = up;
	open_event.event = LVSD_EVENT_VSP_OPEN;
	open_event.data_offset = 0;
	open_event.size = 0;

	/* Push the Open event in to the Message Queue*/
	LvsdMsgQueuePut(vsp_char_device.message_queue, &open_event, sizeof(tLvsd_Event_Entry_t), LVSD_MSG_PRIORITY_NORMAL, LVSD_TIMEOUT_INFINITE);

	retval = 0;

	LVSD_DEBUG("tty: %p", tty);

fail:
	LEAVE();
	return retval;
}

/* This function is called when a Serial Application closes a VSP*/
static void uart_close(struct tty_struct *tty, struct file *filp)
{
	tLvsd_Event_Entry_t close_event;
	struct uart_state *state = tty->driver_data;
	struct tty_port *port;
	struct uart_port *uport;
	unsigned long flags;
	tLvsd_Uart_Port_t *up;

	ENTER();
	LVSD_DEBUG("tty : %p", tty);

	/* filp is NULL when the VSPs are destroyed by the ioctl*/
	if (filp == NULL) {
		LVSD_DEBUG("We got a filp NULL pointer - Means we have to hangup - Means it is destroy sequence");
		goto done;
	}

	/* This means that the function has been called after the port was closed*/
	if (!state)
		return ;

	uport = state->uart_port;
	port = &state->port;

	if ((!uport) || (!port)) { /*state->uart_port, was set to NULL, So no uart_close_can succeed*/
		LVSD_DEBUG("Port or uport was NULL");
		return ;
	} else
		LVSD_DEBUG("Port and uport has some values");

	/* Take the per port mutex, so that no one can "write" to the Wbuffer*/
	mutex_lock(&port->mutex);
	spin_lock_irqsave(&port->lock, flags);

	/* filp is NULL when the VSPs are destroyed by the ioctl*/
	if (filp) {
		if (tty_hung_up_p(filp)) {
			spin_unlock_irqrestore(&port->lock, flags);
			mutex_unlock(&port->mutex);
			goto done;
		}
	}
	LVSD_DEBUG("tty-count = %d and Port count = %d", tty->count, port->count);

	if (port->count) {
		LVSD_DEBUG("Port count was up, so decrementing it");
		port->count--;
		LVSD_DEBUG("New port Count is %d", port->count);
	}

	if (port->count != 0) {
		LVSD_DEBUG("Somebody had opened the FD, ask them to close, or send BADF and kill");
		spin_unlock_irqrestore(&port->lock, flags);
		mutex_unlock(&port->mutex);
	}

	/*
	 * Now we wait for the Rbuffer to clear
	 */
	tty->closing = 1;
	spin_unlock_irqrestore(&port->lock, flags);

	/* wait till the timeout, for the data in the Read Circular Buffer to be taken by the VSAL*/
	if (port->closing_wait != ASYNC_CLOSING_WAIT_NONE)
		tty_wait_until_sent(tty, msecs_to_jiffies(port->closing_wait));

	/* set a flag in the VSP that the port is about to close, in the character device write, check the flag before u write*/
	uart_flush_buffer(tty);

	tty_ldisc_flush(tty);

	tty_port_tty_set(port, NULL);
	spin_lock_irqsave(&port->lock, flags);
	tty->closing = 0;
	spin_unlock_irqrestore(&port->lock, flags);

	/* Wake up anyone trying to open this port, only when the "close" by the serial application happens, else if the ioctl destroys the VSP, don't wake up*/
	if (filp) {
		LVSD_DEBUG("wake up interruptible");
		spin_lock_irqsave(&port->lock, flags);
		clear_bit(ASYNCB_NORMAL_ACTIVE, &port->flags);
		spin_unlock_irqrestore(&port->lock, flags);
		wake_up_interruptible(&port->open_wait);
	}

	mutex_unlock(&port->mutex);

	if (!uport) {
		LVSD_ERR("VSP already destroyed");
		goto done;
	}
	LVSD_DEBUG("tty-count = %d and Port count = %d", tty->count, port->count);

	/* create a CLOSE event and push it to the message queue*/
	up = (tLvsd_Uart_Port_t *)uport;

	LVSD_DEBUG("uart_close(%d) called", uport->line);
	/* create a Close event*/
	close_event.vsp_handle = up;
	close_event.event = LVSD_EVENT_VSP_CLOSE;
	close_event.data_offset = 0;
	close_event.size = 0;

	/* Push the Close event in to the Message Queue*/
	LvsdMsgQueuePut(vsp_char_device.message_queue, &close_event, sizeof(tLvsd_Event_Entry_t), LVSD_MSG_PRIORITY_NORMAL, LVSD_TIMEOUT_INFINITE);

	return ;
done:
	LEAVE();
	return ;
}


/* Wait for the Modem Status to Change for a VSP*/
static int uart_wait_modem_status(struct uart_state *state, unsigned long arg)
{
	struct uart_port *uport = state->uart_port;
	struct tty_port *port = &state->port;
	DECLARE_WAITQUEUE(wait, current);
	unsigned int mctrl_prev, mctrl_now;
	int ret;

	ENTER();
	/*
	 * note the counters on entry
	 */

	mctrl_prev = uart_get_mctrl(uport);

	LVSD_DEBUG("mctrl_prev: 0x%x", mctrl_prev);

	add_wait_queue(&port->delta_msr_wait, &wait);
	for (;;) {
		mctrl_now = uart_get_mctrl(uport);

		LVSD_DEBUG("mctrl_prev: 0x%x", mctrl_prev);
		LVSD_DEBUG("mctrl_now: 0x%x", mctrl_prev);

		set_current_state(TASK_INTERRUPTIBLE);

		if (mctrl_now != mctrl_prev) {
			ret = 0;
			break;
		}

		schedule();

		/* see if a signal did it */
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}

		mctrl_prev = mctrl_now;
		LVSD_DEBUG("mctrl_now copied to mctrl_now: 0x%x, mctrl_prev: 0x%x", mctrl_now, mctrl_prev);
	}

	current->state = TASK_RUNNING;
	remove_wait_queue(&port->delta_msr_wait, &wait);

	LEAVE();

	return ret;
}

/*
 * Called via sys_ioctl.  We can use spin_lock_irq() here.
 */
static int uart_ioctl(struct tty_struct *tty,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 1, 0)
		struct file *filp,
#endif
		unsigned int cmd,
		unsigned long arg)
{
	struct uart_state *state = tty->driver_data;
	struct tty_port *port = &state->port;
	int ret = -ENOIOCTLCMD;

	ENTER();

	/*
	 * The following should only be used when hardware is present.
	 */
	switch (cmd) {
	case TIOCMIWAIT:
		ret = uart_wait_modem_status(state, arg);
		break;
	}

	if (ret != -ENOIOCTLCMD)
		goto out;

	mutex_lock(&port->mutex);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
	if (tty->flags & (1 << TTY_IO_ERROR)) {
#else
	if (tty_hung_up_p(filp)) {
#endif
		ret = -EIO;
		goto out_up;
	}

out_up:
	mutex_unlock(&port->mutex);
out:
	LEAVE();
	return ret;
}

/* Copy a single character to the Rbuffer (circular buffer) of a VSP*/
static int uart_put_char(struct tty_struct *tty, unsigned char ch)
{
	int ret = 0;

	ENTER();

	LVSD_DEBUG("tty : %p, ch: 0x%x or %c", tty, ch, ch);

	ret = uart_write(tty, &ch, 1);

	LEAVE();

	return ret;
}

/* TTY operations structure*/
static const struct tty_operations vsp_uart_ops = {
	.open		= uart_open,
	.close		= uart_close,
	.write		= uart_write,
	.write_room	= uart_write_room,
	.chars_in_buffer = uart_chars_in_buffer,
	.throttle	= uart_throttle,
	.unthrottle	= uart_unthrottle,
	.hangup		= uart_hangup,
	.tiocmget	= uart_tiocmget,
	.tiocmset	= uart_tiocmset,
	.flush_buffer	= uart_flush_buffer,
	.ioctl		= uart_ioctl,
	.put_char	= uart_put_char,
};

/**
 *	lvsd_uart_register_driver - register a driver with the uart core layer
 *	@drv: low level driver structure
 *
 *	Register a uart driver with the core driver.  We in turn register
 *	with the tty layer, and initialise the core driver per-port state.
 *
 *	We have a proc file in /proc/tty/driver which is named after the
 *	normal driver.
 *
 *	drv->port should be NULL, and the per-port structures should be
 *	registered using lvsd_uart_add_one_port after this call has succeeded.
 */
int lvsd_uart_register_driver(struct uart_driver *drv)
{
	struct tty_driver *normal;
	int i, retval;

	ENTER();

	/*If the state has already been allocated*/
	BUG_ON(drv->state);

	/* Create the uart_state for max number of VSPs a uart_driver supports*/
	drv->state = kzalloc(sizeof(struct uart_state) * drv->nr, GFP_KERNEL);
	if (!drv->state) {
		LVSD_ERR("Allocation of the drv->state Failure");
		goto out;
	}

	/*Allocate a TTY driver*/
	normal = alloc_tty_driver(drv->nr);
	if (!normal) {
		LVSD_ERR("Allocation of the tty driver Failure");
		goto out_kfree;
	}

	drv->tty_driver = normal;

	LVSD_DEBUG("The device we are creating is %s", drv->dev_name);

	normal->owner		= drv->owner;
	normal->driver_name	= drv->driver_name;
	normal->name		= drv->dev_name;
	normal->major		= drv->major;
	normal->minor_start	= drv->minor;
	normal->type		= TTY_DRIVER_TYPE_SERIAL;
	normal->subtype		= SERIAL_TYPE_NORMAL;
	normal->init_termios	= tty_std_termios;
	normal->init_termios.c_lflag &= ~(ISIG | ICANON | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | IEXTEN);
	normal->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	normal->init_termios.c_ispeed = normal->init_termios.c_ospeed = 9600;
	normal->flags		= TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV | TTY_DRIVER_UNNUMBERED_NODE;
	normal->driver_state    = drv;
	tty_set_operations(normal, &vsp_uart_ops);

	/* initialize the uart state(s)*/
	for (i = 0; i < drv->nr; i++) {
		struct uart_state *state = drv->state + i;
		struct tty_port *port = &state->port;
		*(normal->ports) = port;

		tty_port_init(port);
		port->close_delay     = 500;	/* .5 seconds */
		port->closing_wait    = 3000;	/* 3 seconds */
	}

	LVSD_DEBUG("TTY num. of devices are %d", normal->num);
	/* register the TTY driver*/
	retval = tty_register_driver(normal);
	LVSD_DEBUG("tty driver registered");

	LEAVE();

	if (retval >= 0)
		return retval;

	put_tty_driver(normal);
out_kfree:
	kfree(drv->state);
out:
	LEAVE();
	return -ENOMEM;
}

/**
 *	lvsd_uart_add_one_port - attach a driver-defined port structure
 *	@drv: pointer to the uart low level driver structure for this port
 *	@uport: uart port structure to use for this port.
 *
 *	This allows the driver to register its own uart_port structure
 *	with the core driver.  The main purpose is to allow the low
 *	level uart drivers to expand uart_port, rather than having yet
 *	more levels of structures.
 */
int lvsd_uart_add_one_port(struct uart_driver *drv, struct uart_port *uport)
{
	struct uart_state *state;
	struct tty_port *port;
	struct device *tty_dev;
	int ret = 0;

	ENTER();

	if (!drv->state)
		return -EINVAL;

	if (uport->line >= drv->nr)
		return -EINVAL;

	/*Get the uart_state corresponding to the uart_port*/
	state = drv->state + uport->line;
	port = &state->port;

	if (!port)
		return -EINVAL;
	mutex_lock(&port_mutex);/*lock for line settings*/
	mutex_lock(&port->mutex);
	if (state->uart_port) {
		LVSD_ERR("state->uart_port already exists");
		ret = -EINVAL;
		goto out;
	}

	state->uart_port = uport;
	uport->state = state;
	mutex_unlock(&port->mutex);/*do we need to release it, so that, while transfer in progress - we can acquire this and operate - with long wait*/
	spin_lock_init(&uport->lock);

	/* adds the TTY device to the TTY layer*/
	tty_dev = tty_register_device(drv->tty_driver, uport->line, NULL);

	if (likely(!IS_ERR(tty_dev)))
		LVSD_DEBUG("tty device %s registered", drv->dev_name);
	else {
	LVSD_ERR("tty device %s registration failure", drv->dev_name);
		ret = -EINVAL;
		goto out;
	}

	/* ensure UPF_DEAD is not set*/
	uport->flags &= ~UPF_DEAD;

 out:
	mutex_unlock(&port->mutex);
	mutex_unlock(&port_mutex);

	LEAVE();

	return ret;
}


int lvsd_tty_register_device(tLvsd_Uart_Port_t *up)
{	
	struct uart_driver *drv;
	struct device *tty_dev;
	struct uart_port *uport;
	struct tty_port *port;
	struct uart_state *state;
	int ret = 0;
	
	tLvsd_Uart_Devices_t *device = up->uart_dev_struct;

	if (!device)
		return -ENODEV;

	drv = &device->driver;
	uport = &up->port;

	if (!drv) {
		LVSD_DEBUG("Empty driver");
		return -EINVAL;
	}

	if (!drv->state) {
		LVSD_DEBUG("Empty driver");
		return -EINVAL;
	}

	if (uport->line >= drv->nr) {
		LVSD_DEBUG("Empty driver");
		return -EINVAL;
	}

	state = drv->state + uport->line;
	port = &state->port;

	mutex_lock(&port->mutex);
	tty_dev = tty_register_device(drv->tty_driver, uport->line, NULL);
	if (likely(!IS_ERR(tty_dev)))
		LVSD_DEBUG("TTY device %s registered", drv->dev_name);
	else {
		LVSD_ERR("TTY device %s registration failure", drv->dev_name);
		ret = -EINVAL;
	}

	mutex_unlock(&port->mutex);

	return ret;


}

int lvsd_tty_unregister_device(struct uart_driver *drv, struct uart_port *uport)
{
	struct uart_state *state;
	struct tty_port *port;
	int ret = 0;

	if (!drv->state)
		return -EINVAL;

	if (uport->line >= drv->nr)
		return -EINVAL;

	state = drv->state + uport->line;
	port = &state->port;
	
	mutex_lock(&port->mutex);
	tty_unregister_device(drv->tty_driver, uport->line);
	mutex_unlock(&port->mutex);

	return ret;
}

/**
 *	lvsd_uart_remove_one_port - Remove a driver-defined port structure
 *	@drv: pointer to the uart low level driver structure for this port
 *	@uport: uart port structure to use for this port.
 *
 *	This allows the driver to unregister its own uart_port structure
 *	with the core driver.
 */
int lvsd_uart_remove_one_port(struct uart_driver *drv, struct uart_port *uport)
{
	struct uart_state *state = drv->state + uport->line;
	struct tty_port *port = &state->port;
	struct tty_struct *tty;
	int ret = 0;

	BUG_ON(in_interrupt());

	if (state->uart_port != uport)
		dev_alert(uport->dev, "Removing wrong port: %p != %p\n",
			state->uart_port, uport);

	mutex_lock(&port_mutex);

	/*
	 * Mark the port "dead" - this prevents any opens from
	 * succeeding while we shut down the port.
	 */
	mutex_lock(&port->mutex);
	if (!state->uart_port) {
		mutex_unlock(&port->mutex);
		ret = -EINVAL;
		goto out;
	}
	uport->flags |= UPF_DEAD;
	mutex_unlock(&port->mutex);

	/*
	 * Remove the devices from the tty layer
	 
	tty_unregister_device(drv->tty_driver, uport->line);*/

	tty = tty_port_tty_get(port);
	if (tty) {
		tty_vhangup(port->tty);
		tty_kref_put(tty);
	}


	/*
	 * Free the port IO and memory resources, if any.
	 */
	if (uport->type != PORT_UNKNOWN)
		uport->ops->release_port(uport);

	/*
	 * Indicate that there isn't a port here anymore.
	 */
	uport->type = PORT_UNKNOWN;

	state->uart_port = NULL;
out:
	mutex_unlock(&port_mutex);

	return ret;
}


/**
 *	lvsd_uart_unregister_driver - remove a driver from the uart core layer
 *	@drv: low level driver structure
 *
 *	Remove all references to a driver from the core driver.  The low
 *	level driver must have removed all its ports via the
 *	lvsd_uart_remove_one_port() if it registered them with lvsd_uart_add_one_port().
 *	(ie, drv->port == NULL)
 */
void lvsd_uart_unregister_driver(struct uart_driver *drv)
{
	struct tty_driver *p = drv->tty_driver;
	unsigned int i;

	ENTER();

	/* unregister the TTY driver*/
	tty_unregister_driver(p);
	put_tty_driver(p);
	/* free the uart states*/
	for (i = 0; i < drv->nr; i++)
		tty_port_destroy(&drv->state[i].port);
	kfree(drv->state);
	drv->state = NULL;
	drv->tty_driver = NULL;

	LEAVE();
}


/**
 *	add_port_to_list - Adds a VSP to the ports list in a tLvsd_Uart_Devices_t structure
 *	@device:	the tLvsd_Uart_Devices_t structure, to whose ports list, the VSP needs to be added
 *	@vsport:	the VSP that has to be added, to the ports list
 */
void add_port_to_list(tLvsd_Uart_Devices_t *device, tLvsd_Uart_Port_t *vsport)
{
	ENTER();

	mutex_lock(&device->ports_list_lock);

	vsport->uart_dev_struct = device;

	if (device->ports_head) {
		vsport->next = device->ports_head;
		vsport->next->prev = vsport;
	}

	/* always add to the head*/
	device->ports_head = vsport;

	mutex_unlock(&device->ports_list_lock);

	LEAVE();
}

/**
 *	remove_port_from_list - Removes a VSP from the ports list in a tLvsd_Uart_Devices_t structure
 *	@vsport:	the VSP that has to be removed, from the ports list
 */
void remove_port_from_list(tLvsd_Uart_Port_t *vsport)
{
	tLvsd_Uart_Devices_t *device = vsport->uart_dev_struct;

	ENTER();

	mutex_lock(&device->ports_list_lock);
	if (vsport == device->ports_head) {
		/*check if this is the last device, and then set device ports head to NULL*/
		device->ports_head = vsport->next;
	} else
		vsport->prev->next = vsport->next;

	if (vsport->next != NULL)
		vsport->next->prev = vsport->prev;

	/*vsport->uart_dev_struct = NULL;*/

	mutex_unlock(&device->ports_list_lock);

	LEAVE();
}

/**
 *	allocate_port - Allocates a VSP
 *	@index :	the index of the VSP that has to be created (for eg: if "/dev/spp15" has to be created, index is 15)
 *	@rsize  :	the size of the Read buffer of the VSP
 *	@wsize  :	the size of the Write buffer of the VSP
 *	Returns:	the Pointer to the VSP allocated in case of Success, NULL in case of Failure
 */
tLvsd_Uart_Port_t *allocate_port(unsigned char index, unsigned int rsize, unsigned int wsize)
{
	tLvsd_Uart_Port_t *vsport = NULL;

	ENTER();

	/* Allocate a VSP*/
	vsport = kzalloc(sizeof(tLvsd_Uart_Port_t), GFP_KERNEL);
	if (!vsport) {
		LVSD_ERR("Allocation of VSP structure Failure");
		return NULL;
	}

	/* this is for creation of the device using tty_register_device and for checking if the port has already been created*/
	vsport->dev_index = vsport->port.line = index;

	atomic_set (&(vsport->wbuf_fill_level), 0);

	/* initialize the VSP*/
	vsport->prev = vsport->next = NULL;
	vsport->rbuffer_size = rsize;
	vsport->wbuffer_size = wsize;

	LVSD_DEBUG("Rbuffer_Size: %d", vsport->rbuffer_size);
	LVSD_DEBUG("Wbuffer_Size: %d", vsport->wbuffer_size);

	/* allocate the read and write buffers (rbuffer, wbuffer) which are to be memory mapped*/
	vsport->wbuffer =  (char *) __get_free_pages(GFP_KERNEL, 0); /*order is 2^0 = 1 - this is used for write coming from VSAL Layer*/
	if (vsport->wbuffer == NULL) {
		LVSD_ERR("Allocation of Wbuffer Failure");
		return NULL;
	}
	LVSD_DEBUG("Page allocated for WBuffer: %p", vsport->wbuffer);

	vsport->rbuffer.buf = (char *) __get_free_pages(GFP_KERNEL, 0); /*this is used for write coming from Application*/
	if (vsport->rbuffer.buf == NULL) {
		free_pages((unsigned long)vsport->wbuffer, 0);
		LVSD_ERR("Allocation of Rbuffer Failure");
		return NULL;
	}
	LVSD_DEBUG("Page allocated for RBuffer: %p", vsport->rbuffer.buf);

	/* initialize wbuffer write mutex*/
	mutex_init(&(vsport->wbuffer_mutex));

	/* initialize the Rbuffer (circular buffer)*/
	uart_circ_clear(&vsport->rbuffer);

	LEAVE();

	return vsport;
}

/**
 *	deallocate_port - Deallocates a VSP
 *	@vsport:	the VSP that has to be deallocated
 */
void deallocate_port(tLvsd_Uart_Port_t *vsport)
{
	ENTER();

	if (!vsport)
		return;

	/* Free the read and write buffers (rbuffer, wbuffer) of the VSP*/
	free_pages((unsigned long)vsport->rbuffer.buf, 0);
	free_pages((unsigned long)vsport->wbuffer, 0);

	/* De-initialize the VSP*/
	vsport->prev = vsport->next = NULL;

	/* free the VSP*/
	kfree(vsport);

	LEAVE();
	return;
}

/**
 *	allocate_device - allocates a device of tLvsd_Uart_Devices_t structure
 *	@name  :	the name of the device structure that has to be created
 *	Returns:	pointer to the created device structure in case of Success, NULL in case of Failure
 */
tLvsd_Uart_Devices_t *allocate_device(char *name)
{
	tLvsd_Uart_Devices_t *device = NULL;
	ENTER();

	/* allocate a device structure*/
	device = kzalloc(sizeof(tLvsd_Uart_Devices_t), GFP_KERNEL);
	if (!device) {
		LVSD_ERR("No memory to allocate uart_devices structure");
		return NULL;
	}

	/*basic initialization of the uart_devices structure*/
	device->prev = device->next = NULL;
	strcpy(device->dev_name, name);

	LEAVE();
	return device;
}

/**
 *	init_uart_device - initialize the created tLvsd_Uart_Devices_t structure
 *	@device:	the tLvsd_Uart_Devices_t structure which needs to be initialized
 */
void init_uart_device(tLvsd_Uart_Devices_t *device)
{
	ENTER();

	device->ports_head = NULL;
	device->port_index_bitmap = 0;
	mutex_init(&device->ports_list_lock);

	/* initialize the uart_driver structure*/
	device->driver.owner = THIS_MODULE;
	device->driver.driver_name = "lvsd_serial";
	device->driver.dev_name = device->dev_name;
	device->driver.minor = 1;
	device->driver.nr = LVSD_MAX_INDEX_PER_DEVICE_NAME;

	LEAVE();
}

/**
 *	deallocate_device - deallocate a tLvsd_Uart_Devices_t structure
 *	@device:	the tLvsd_Uart_Devices_t structure that needs to be deallocated
 */
inline void deallocate_device(tLvsd_Uart_Devices_t *device)
{
	ENTER();

	kfree(device);

	LEAVE();
	return;
}

/* search for device whose name matches with the passed name. if matches return that device, else NULL*/
tLvsd_Uart_Devices_t *search_for_device(char *name)
{
	tLvsd_Uart_Devices_t	*devices_temp = NULL;

	ENTER();

	devices_temp = vsp_char_device.devices_head;

	if (!devices_temp)
		return NULL;

	else {
		/* parse the devices list*/
		while (devices_temp) {
			if (!strcmp(devices_temp->dev_name, name)) {
				LVSD_DEBUG("Device Match found - returning");
				LEAVE();
				return devices_temp;
			} else {
				devices_temp = devices_temp->next;
			}
		}
		/* no device name matches with the passed name*/

		LEAVE();
		return NULL;
	}
}

/**
 *	add_device_to_list - Adds a Device to the devices list in the global character device structure
 *	@device:	the tLvsd_Uart_Devices_t structure, which needs to be added to the list
 */
void add_device_to_list(tLvsd_Uart_Devices_t *device)
{
	ENTER();

	if (vsp_char_device.devices_head) {
		LVSD_DEBUG("Device head had some value");
		device->next = vsp_char_device.devices_head; //Current device(btspp) next shld hold value at device head lvsd
		device->next->prev = device; //lvsd
	}

	/* always add to the head*/
	vsp_char_device.devices_head = device;

	LEAVE();
}

/**
 *	remove_device_from_list - Removes a device from the devices list in the global character device structure
 *	@device:	the tLvsd_Uart_Devices_t structure, that needs to be removed from the list
 */
int remove_device_from_list(tLvsd_Uart_Devices_t *device)
{
	tLvsd_Uart_Devices_t *tmp_device;
	ENTER();


	if (!device) {
		LVSD_ERR("Device cannot be NULL");
		return -1;
	}

	LVSD_DEBUG("Device to be removed is %s", device->dev_name);

	tmp_device = search_for_device(device->dev_name);

	if(!tmp_device) {
		LVSD_DEBUG("Cannot remove Device which is not present in Linked List");
		return -1;
	}
	
	if (tmp_device == vsp_char_device.devices_head) {
		vsp_char_device.devices_head = tmp_device->next;
		LVSD_DEBUG("Device was at head");
	} else if (tmp_device->next != NULL) {
		tmp_device->next->prev = tmp_device->prev;
		tmp_device->prev->next = tmp_device->next;
		LVSD_DEBUG("Device somewhere in the middle, but definately not last");
	} else {
		tmp_device->prev->next = tmp_device->next;
		LVSD_DEBUG("This in the middle - checking is it is last");
	}

	
	LEAVE();
	return 0;
}

/* creates a VSP*/
tLvsd_Uart_Port_t *create_vsp(char *device_name, unsigned char device_index, unsigned int rsize, unsigned int wsize)
{
	int retval = 0;
	tLvsd_Uart_Port_t *vsp = NULL;
	tLvsd_Uart_Devices_t *new_device, *old_device;

	ENTER();

	new_device = old_device = NULL;

	/* take the lock devices_list_lock*/
	if (mutex_lock_interruptible(&vsp_char_device.devices_list_lock))
		LVSD_ERR("mutex lock failure");

	new_device = search_for_device(device_name);

	if (!new_device) { /*if device is not present or first device or empty linked list
		* release the lock devices_list_lock*/
		mutex_unlock(&vsp_char_device.devices_list_lock);

		new_device = allocate_device(device_name);

		if (!new_device) {
			LVSD_ERR("could not allocate a new device structure");
			return NULL;
		}

		/*initialize the new device structure*/
		init_uart_device(new_device);

		/* allocate a VSP*/
		vsp = allocate_port(device_index, rsize, wsize);

		if (!vsp) {
			LVSD_ERR("could not allocate a vsp");
			deallocate_device(new_device);
			return NULL;
		}

		/*add the allocated VSP to the ports list of the new device structure*/
		add_port_to_list(new_device, vsp);

		/* take the lock devices_list_lock*/
		if (mutex_lock_interruptible(&vsp_char_device.devices_list_lock)) {
			LVSD_ERR("mutex lock failure");
		}
		old_device = search_for_device(device_name);

		if (!old_device) {
			if (!vsp_char_device.devices_head)
				LVSD_DEBUG("minor start is 0");
			else {
				LVSD_DEBUG("minor start is LVSD MAX + 1");
				new_device->driver.minor = new_device->minor_start = ((vsp_char_device.devices_head->minor_start) + LVSD_MAX_INDEX_PER_DEVICE_NAME);
			}

			/* register the uart_driver in the new device structure*/
			if (lvsd_uart_register_driver(&new_device->driver)) {
				LVSD_ERR("Cannot Register the uart_driver in the new device structure properly");
				/* release the lock devices_list_lock*/
				mutex_unlock(&vsp_char_device.devices_list_lock);
				deallocate_port(vsp);
				deallocate_device(new_device);
				return NULL;
			}

			/* add the vsp's uart_port to the uart_driver in the new device structure*/
			retval = lvsd_uart_add_one_port(&new_device->driver, &vsp->port);
			if (retval) {
				LVSD_ERR("Cannot add one uart port to the uart driver in the new device structure ");
				lvsd_uart_unregister_driver(&new_device->driver);
				/* release the lock devices_list_lock*/
				mutex_unlock(&vsp_char_device.devices_list_lock);
				deallocate_port(vsp);
				deallocate_device(new_device);
				return NULL;
			}

			add_device_to_list(new_device);

			/* set the index of the vsp in the port bit map of the the new device structure*/
			new_device->port_index_bitmap |= (1 << vsp->dev_index);
			mutex_unlock(&vsp_char_device.devices_list_lock);

			LEAVE();
			return vsp;
		} else {
			if (old_device->port_index_bitmap & (1 << device_index)) {
				LVSD_ERR("VSP already created which has the passed name and index");
				/* release the lock devices_list_lock*/
				mutex_unlock(&vsp_char_device.devices_list_lock);
				deallocate_port(vsp);
				deallocate_device(new_device);
				LEAVE();
				return NULL;
			} else {
				/* remove the VSP from the ports list of new device structure*/
				remove_port_from_list(vsp);
				deallocate_device(new_device);

				/* add the allocated VSP to the ports list of the old device structure*/
				add_port_to_list(old_device, vsp);

				/* add the vsp's uart port to the uart driver in the old device structure*/
				retval = uart_add_one_port(&old_device->driver, &vsp->port);
				if (retval) {
					LVSD_ERR("Cannot add one uart port to the uart driver in the old device structure");
					/* remove the VSP from the ports list of old device structure*/
					remove_port_from_list(vsp);
					mutex_unlock(&vsp_char_device.devices_list_lock);
					deallocate_port(vsp);
					return NULL;
				}

				/* set the index of the vsp in the port bit map of the the old device structure*/
				old_device->port_index_bitmap |= (1 << vsp->dev_index);
				mutex_unlock(&vsp_char_device.devices_list_lock);

				LEAVE();
				return vsp;
			}
		}
	} else {
		if (new_device->port_index_bitmap & (1 << device_index)) {
			LVSD_ERR("VSP already created which has the passed name and index");
			/* release the lock devices_list_lock*/
			mutex_unlock(&vsp_char_device.devices_list_lock);
			return NULL;
		} else {
			/* allocate a VSP*/
			vsp = allocate_port(device_index, rsize, wsize);
			if (!vsp) {
				LVSD_ERR("could not allocate a vsp");
				/* release the lock devices_list_lock*/
				mutex_unlock(&vsp_char_device.devices_list_lock);
				return NULL;
			}

			/* add the allocated VSP to the ports list of the new device structure*/
			add_port_to_list(new_device, vsp);

			/* add the vsp's uart port to the uart driver of the new device structure*/
			retval = uart_add_one_port(&new_device->driver, &vsp->port);
			if (retval) {
				LVSD_ERR("cannot add one uart port to the uart driver in the new device structure");
				/* remove the VSP from the ports list of the new device structure*/
				remove_port_from_list(vsp);
				deallocate_port(vsp);
				/* release the lock devices_list_lock*/
				mutex_unlock(&vsp_char_device.devices_list_lock);
				return NULL;
			}

			/* set the index of the vsp in the port bit map of the the new device structure*/
			new_device->port_index_bitmap |= (1 << vsp->dev_index);
			mutex_unlock(&vsp_char_device.devices_list_lock);

			LEAVE();
			return vsp;
		}
	}
}

int shutdown_vsp(tLvsd_Uart_Port_t *vsp)
{
	int retval;
	tLvsd_Event_Entry_t shutdown_event;
	struct uart_state *state;
	struct tty_struct *tty;

	state = vsp->port.state;
	if (!state) {
		LVSD_ERR("No state for current VSP");
		return 0;
	}

	tty = state->port.tty;
	if (!tty) {
		LVSD_ERR("VSP still not open");
		return 0;
	}

	/*Cancel all the work in TTY core when we get this - We will reach here for operations in progress case, when somebody destroys, as TTY will always be valid then*/


	shutdown_event.vsp_handle = vsp;
	shutdown_event.event = LVSD_EVENT_VSP_SHUTDOWN;

	/*Push the shutdown event into the message queue*/
	LvsdMsgQueuePut(vsp_char_device.message_queue, &shutdown_event, sizeof(tLvsd_Event_Entry_t), LVSD_MSG_PRIORITY_NORMAL, LVSD_TIMEOUT_INFINITE);

	retval = destroy_vsp(vsp);
	if (retval == 0) {
		LVSD_DEBUG("Destroy VSP from Shutdown code was successful");
		retval = 1;
	}

	return retval;

}

/* destroys a VSP*/
int destroy_vsp(tLvsd_Uart_Port_t *vsp)
{
	tLvsd_Uart_Devices_t *temp_uart_device = NULL;
	struct uart_port *uport = NULL;
	struct tty_port	*ttyport = NULL;
	tLvsd_Event_Entry_t	dummy_event;

	ENTER();

	if (!vsp || !vsp->uart_dev_struct) {
		LVSD_ERR("Invalid VSP handle passed");
		return -1;
	}

	uport = &vsp->port;
	if (!uport || !uport->state) {
		LVSD_ERR("Invalid uart port / uart state in VSP");
		return -1;
	}

	ttyport = &uport->state->port;
	if (!ttyport) {
		LVSD_ERR("ttyport in the state of uart port in VSP is NULL");
		return -1;
	}
	LVSD_DEBUG("checked ttyport");

	if (!ttyport->tty)
		LVSD_DEBUG("uart port never opened or already closed");
	else {
		LVSD_DEBUG("Calling uart_close to close existing open port");
		uart_close(ttyport->tty, NULL);/*Sends Filp as NULL pointer, stating we have to serial mount point file*/
		--ttyport->count;
	}
	LVSD_DEBUG("checked tty for calling uart close");

	if (mutex_lock_interruptible(&vsp_char_device.devices_list_lock))
		LVSD_ERR("mutex lock failure");

	LVSD_DEBUG("invalidating all the message queue entries which correspond to the port to be destroyed");

	if (LvsdMsgQueueModifyData(vsp_char_device.message_queue, (unsigned int *)vsp, ((unsigned int *)(&dummy_event.vsp_handle) - (unsigned int *)(&dummy_event)), ((long int)vsp | 0x1)) != EOK)
		LVSD_ERR("error invalidating the message queue entries");
	temp_uart_device = vsp->uart_dev_struct;

	/* clear the index bit*/
	//temp_uart_device->port_index_bitmap &= ~(1 << vsp->dev_index);

	if (!temp_uart_device->port_index_bitmap) {
		if (remove_device_from_list(temp_uart_device)) {
			LVSD_ERR("Device Already Removed from the List or Device is NULL");
			mutex_unlock(&vsp_char_device.devices_list_lock);
			return -1;
		}

		if (lvsd_uart_remove_one_port(&temp_uart_device->driver, &vsp->port)) {
			LVSD_ERR("Error in Removing Uart Port");
			mutex_unlock(&vsp_char_device.devices_list_lock);
			return -1;
		}

		remove_port_from_list(vsp);
		lvsd_uart_unregister_driver(&temp_uart_device->driver);

		mutex_unlock(&vsp_char_device.devices_list_lock);
		return -1;
	}
		
	if (lvsd_tty_unregister_device(&temp_uart_device->driver, &vsp->port)) {
		LVSD_ERR("Error in unregister TTY device from FS/TTY layer");
		mutex_unlock(&vsp_char_device.devices_list_lock);
		return -1;
	} else {
		
		if (lvsd_uart_remove_one_port(&temp_uart_device->driver, &vsp->port)) {
			LVSD_ERR("Error in Removing Uart Port");
			mutex_unlock(&vsp_char_device.devices_list_lock);
			return -1;
		}
		LVSD_DEBUG("Unregistering port and uart driver");
		remove_port_from_list(vsp);
		lvsd_uart_unregister_driver(&temp_uart_device->driver);

		mutex_unlock(&vsp_char_device.devices_list_lock);

		deallocate_port(vsp);
		deallocate_device(temp_uart_device);
	}
	

	LEAVE();
	return 0;
}
