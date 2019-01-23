/* lvsd_internal.h - LVSD internal structure headers.
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

#ifndef __LVSD_INTERNAL_H__
#define __LVSD_INTERNAL_H__

/*------------------------------------------------------------------------------
 * INCLUDES
 *----------------------------------------------------------------------------*/
#include <linux/serial_core.h>
#include <asm/atomic.h>


#include "lvsd.h"
#include "lvsd_msgq.h"

/*------------------------------------------------------------------------------
 * MACROS
 *----------------------------------------------------------------------------*/
#define LVSD_MAX_INDEX_PER_DEVICE_NAME	1

/*------------------------------------------------------------------------------
 * TYPEDEFS & STRUCTURES
 *----------------------------------------------------------------------------*/
typedef struct tLvsd_Uart_Devices	tLvsd_Uart_Devices_t;
typedef struct tLvsd_Uart_Port		tLvsd_Uart_Port_t;

/**
 * @brief                : All the VSPs which have a common name are holded in a linked list. They have some common information.
			: This structure holds that common information
 * @ports_head           : head of the linked list of VSPs with the common name
 * @dev_name             : the common name of the VSPs (for eg: "spp", "btmodem" etc.)
 * @minor_start          : the minor start value of the common named VSPs (all the VSPs with a common name have a common TTY driver, which needs a unique minor start value)
 * @port_index_bitmap    : the bit map which specifies the index value of the created VSPs (for eg: if "/dev/spp12" is created, then the 13th bit is set)
 * @ports_list_lock      : the mutex which synchronizes the addition and deletion of VSPs in the linked list
 * @driver               : the driver which is common for all the VSPs with the common name
 * @prev                 : the previous devices structure in the linked list of device structures
 * @next                 : the next devices structure in the linked list of device structures
 */
struct tLvsd_Uart_Devices {
	tLvsd_Uart_Port_t		*ports_head;
	char				dev_name[LVSD_VSP_DEVICE_NAME_MAX_LEN];
	unsigned int			minor_start;
	unsigned int			port_index_bitmap;
	struct mutex			ports_list_lock;
	struct uart_driver		driver;
	tLvsd_Uart_Devices_t		*prev;
	tLvsd_Uart_Devices_t		*next;
};

/**
 * @brief                : Details related to a VSP
 *
 * @port                 : the uart_port structure which is unique to a VSP
 * @dev_index            : the index of the VSP (for eg: the VSP "/dev/spp12", has the index "12")
 * Read and Write are seen from perspective of VSAL Layer
 * @wbuffer              : the write buffer of a VSP which has the data updated by the VSAL
 * @rbuffer              : the read circular buffer of a VSP which has the data updated by a Serial Application
 * @rbuffer_size         : the size of the Read buffer of a VSP
 * @wbuffer_size         : the size of the Write buffer of a VSP
 * @uart_dev_struct      : the pointer to the parent tLvsd_Uart_Devices_t structure
 * @prev                 : the previous VSP in the linked list of VSPs in the tLvsd_Uart_Devices_t structure
 * @next                 : the next VSP in the linked list of VSPs in the tLvsd_Uart_Devices_t structure
 */
struct tLvsd_Uart_Port {
	struct uart_port	port;
	unsigned char		dev_index;
	char			*wbuffer;
	/*Lock to interrupt receive functionality*/
	spinlock_t		interrupt_lock;
	/*atomic var to show write buffer fill status. Set 0*/
	atomic_t wbuf_fill_level;
	struct mutex		wbuffer_mutex;/*To protect wbuffer concurrent access in user space and kernel space*/
	struct circ_buf		rbuffer;
	unsigned int		rbuffer_size;
	unsigned int		wbuffer_size;
	tLvsd_Uart_Devices_t	*uart_dev_struct;
	tLvsd_Uart_Port_t	*prev;
	tLvsd_Uart_Port_t	*next;
};

/**
 * @brief                : The Global Character Device Structure
 *
 * @devices_head         : head of the linked list of the tLvsd_Uart_Devices_t structures
 * @message_queue        : the message queue which has the events that will be updated to the VSAL
 * @lvsd_cdev            : the cdev structure which is unique to a Character Device
 * @devices_list_lock    : the mutex which synchronizes the addition and deletion of the tLvsd_Uart_Devices_t structures in the linked list
 * @open_count           : the variable which allows only one "open" to happen on the Character Device interface
 */
typedef struct tLvsd_Char_Dev {
	tLvsd_Uart_Devices_t	*devices_head;
	LvsdMsgQueue		message_queue;
	struct cdev		lvsd_cdev;
	struct mutex		devices_list_lock;
	atomic_t		open_count;
} tLvsd_Char_Dev_t;

/**
 * @brief		: The Structure which has a Work and the corresponding LVSD event
 * @lvsd_event		: The LVSD event which will be given to VSAL
 * @lvsd_work_struct	: The work structure
 */
typedef struct tLvsd_Work_Queue_Struct_Event {
	tLvsd_Event_Entry_t lvsd_event;
	struct work_struct lvsd_work_struct;
} tLvsd_Work_Queue_Struct_Event_t;

/**
 * @brief   : creates a VSP with the passed device name, device index; if there is already one created return error
 *
 * @param   : device_name
 *              Name of the VSP to be created (ex: spp)
 * @param   : device_index
 *	        Index of the VSP to be created (We ignore indexing)
 * @param   : rsize
 *              Size of the Read Buffer of the VSP
 * @param   : wsize
 *              Size of the Write Buffer of the VSP
 * Returns  : ptr to the new VSP in case of Success, NULL for Failure
 */
tLvsd_Uart_Port_t *create_vsp(char *device_name,
	unsigned char device_index, unsigned int rsize, unsigned int wsize);

/**
 * @brief   : Destroys a VSP
 *
 * @param   : handle
 *              Handle to the VSP which has to be destroyed
 * Returns  : 0 in case of Success, -1 in case of Failure
 */
int destroy_vsp(tLvsd_Uart_Port_t  *handle);

/**
 * @brief   : Shutdown a VSP
 *
 * @param   : handle
 *              Handle to the VSP which has to be destroyed
 * Returns  : 0 in case of Success
 */
int shutdown_vsp(tLvsd_Uart_Port_t  *handle);

/**
 * @brief   : Update the Modem Control Value of a VSP
 *
 * @param   : port, Uart Port of the VSP, whose Modem Control Value has to be updated
 * @param   : set
 *              The Bits of the Modem Control Value which have to be Set
 * @param   : clear
 *             The Bits of the Modem Control Value which have to be Cleared
 */
void uart_update_mctrl(struct uart_port *port,
		unsigned int set, unsigned int clear);

/**
 * @brief   : Get the Modem Control Value of a VSP
 *
 * @param   : port
 *              Uart Port of the VSP, whose Modem Control Value has to be got
 * Returns  : The Modem Control Value of the VSP
 */
unsigned int uart_get_mctrl(struct uart_port *port);

/**
 * @brief   : Copies the Data in the Wbuffer of the VSP, to the TTY Buffers of the VSP and then to the RBUF of the VSP
 *
 * @param   : vsp
 *              The VSP whose Wbuffer has the data updated by the VSAL
 * @param   : count - data written by VSAL,into the Wbuffer
 * @param   : current_wbuf_fill_level - bytes available in write buff.
 * Returns  : data copied from the Wbuffer of VSP, to TTY buffers of VSP
 */
unsigned int receive_chars(tLvsd_Uart_Port_t *vsp, size_t count,
	unsigned int current_wbuf_fill_level);

/*Function to buffer data written on ctrlX side*/
int store_wbuff_data(tLvsd_Uart_Port_t *vsp, size_t count,
	unsigned int fill_level);

/*Function to trigger pending write if upper layer has not consumed more data*/
int trigger_pending_write(tLvsd_Uart_Port_t *vsp,
	unsigned int write_flag);


/**
 * @brief   : Get the amount of freespace available, in the RBUF of the VSP
 *
 * @param   : port - Uart Port of VSP, whose freespace in RBUF is req.
 * Returns  : The amount of freespace available, in the RBUF of the VSP
 */
unsigned int uart_get_freespace(struct uart_port *port);

/**
 * @brief   : Updates the Data Read Count of the Read Circular Buffer of the VSP
 *
 * @param   : up
 *              The VSP, whose data read count needs to be updated
 * @param   : read_cnt - value by which data read count of the Read Circ Buff needs updation
 */
void update_circ_read_cnt(tLvsd_Uart_Port_t *up, unsigned int read_cnt);

/*Register a TTY device with FS and TTY layer*/
int lvsd_tty_register_device(tLvsd_Uart_Port_t *up);

#endif

