/* lvsd.h - LVSD general headers.
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

#ifndef __LVSD_H__
#define __LVSD_H__

/*------------------------------------------------------------------------------
* MACROS
*----------------------------------------------------------------------------*/
#define	LVSD_VSP_DEVICE_NAME_MAX_LEN	255
#define LVSD_VSAL_MAX_WRITE_SIZE	12288

/*
* Ioctl definitions
*/

#define LVSD_CREATE_VSP			0x1001
#define LVSD_DESTROY_VSP		0x1002
#define LVSD_SET_MODEM_CTRL		0x1003
#define LVSD_GET_MODEM_CTRL		0x1004
#define LVSD_GET_FREESPACE		0x1005
#define LVSD_WAIT_FOR_EVENTS		0x1006
#define	LVSD_UPDATE_READ_CNT		0x1007
#define	LVSD_DO_PENDING_WRITE		0x1008

/*------------------------------------------------------------------------------
 * TYPEDEFS & STRUCTURES
 *----------------------------------------------------------------------------*/

/**
 * @brief : The type of events, which shall be notified to the VSAL
 */
typedef enum tLvsd_Event {
	LVSD_EVENT_VSP_DATA,
	LVSD_EVENT_VSP_CONTROL,
	LVSD_EVENT_VSP_OPEN,
	LVSD_EVENT_VSP_WRITE,
	LVSD_EVENT_VSP_CLOSE,
	LVSD_EVENT_VSP_SHUTDOWN,
	LVSD_EVENT_VSP_BUFFER_STATE_CHANGE
} tLvsd_Event_t;

/**
 * @brief : Access Read Buffer / Write Buffer
 */
typedef enum tLvsd_Access_Buf {
	LVSD_ACCESS_READ_BUF,
	LVSD_ACCESS_WRITE_BUF
} tLvsd_Access_Buf_t;

/**
 * @brief          : Actual event related information, that will be notified to the VSAL
 *
 * @event          : the type of event
 * @vsp_handle     : the handle of the VSP on which the event happened
 * @data_offset    : the offset in the Read Circular buffer of the VSP, where data is written by the Serial Application (in case of LVSD_EVENT_VSP_DATA)
 * @size           : the amount of data written by the Serial Application in the Read Circular Buffer of the VSP (in case of LVSD_EVENT_VSP_DATA)
 */
typedef struct tLvsd_Event_Entry {
	tLvsd_Event_t		event;
	void			*vsp_handle;
	unsigned int		data_offset;
	unsigned int		size;
} tLvsd_Event_Entry_t;

/**
 * @brief                : Information related to the VSP that has to be created
 *
 * @vsp_handle           : the handle of the VSP that is created (this will be notified to the VSAL)
 * @vsp_device_name      : the name of the VSP that has to be created (for eg: "spp" if the VSP "/dev/spp12" has to be created)
 * @vsp_device_index     : the index of the VSP that has to be created (for eg: "12" if the VSP "/dev/spp12" has to be created)
 * @vsp_rbuffer_size     : the size of the Read buffer of the VSP which has to be created
 * @vsp_wbuffer_size     : the size of the Write buffer of the VSP which has to be created
 */
typedef struct tLvsd_Vsp {
	void			*vsp_handle;
	char			vsp_device_name[LVSD_VSP_DEVICE_NAME_MAX_LEN];
	unsigned char		vsp_device_index;
	unsigned int		vsp_rbuffer_size;
	unsigned int		vsp_wbuffer_size;
} tLvsd_Vsp_t;

/**
 * @brief                : To set / get the Modem Control Value of a VSP
 *
 * @vsp_handle           : the handle of the VSP whose Modem Control Value needs to be set / get
 * @vsp_mcr_val          : the Modem Control Value of the VSP
 */
typedef struct tLvsd_Modem_Ctrl_Val {
	void			*vsp_handle;
	unsigned int		vsp_mcr_val;
} tLvsd_Modem_Ctrl_Val_t;

/**
 * @brief                : Information related to the amount of freespace available in the RBUF of a VSP
 *
 * @vsp_handle           : the handle of the VSP whose freespace in the RBUF needs to be get
 * @freespace            : the amount of freespace available in the RBUF of the VSP (this will be notified to the VSAL)
 */
typedef struct tLvsd_Freespace {
	void *vsp_handle;
	unsigned int freespace;
} tLvsd_Freespace_t;

/**
 * @brief                : Amount of data read from the Rbuffer will be updated by the VSAL
 *
 * @vsp_handle           : the handle of the VSP whose Rbuffer's data read count will be updated
 * @read_cnt		: the amount of data read from the Rbuffer of a VSP
 */
typedef struct tLvsd_Update_Read_Count {
	void			*vsp_handle;
	unsigned int	read_cnt;
} tLvsd_Update_Read_Count_t;

typedef struct tLvsd_Update_Pending_Write {
	void *vsp_handle;
	unsigned int	write_flag;
} tLvsd_Update_Pending_Write_t;

/**
 * @brief                : The Read or Write Buffers of a VSP which are to be mmapped
 *
 * @vsp_handle           : the handle of the VSP whose Read/Write Buffer needs to be mmapped
 * @read_write           : the flag which specifies whether to mmap the Read or Write Buffer of the VSP
 */
typedef struct tLvsd_Mmap_Buf_Info {
	void			*vsp_handle;
	tLvsd_Access_Buf_t	read_write;
} tLvsd_Mmap_Buf_Info_t;

#endif

