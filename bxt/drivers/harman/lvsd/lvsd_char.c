/* lvsd_char.c - LVSD Character Driver, Control Interface Implementation.
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
#include <asm/io.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/version.h>
#include "lvsd.h"
#include "lvsd_internal.h"
#include "lvsd_trace.h"

/*------------------------------------------------------------------------------
 * MACROS
 *----------------------------------------------------------------------------*/
/* Max number of Messages in the Message Queue*/
#define	LVSD_MAX_NUMBER_OF_MSGS		256
#define  BYTES_ALIGNED              4
static struct class *vsp_char_class;
tLvsd_Char_Dev_t vsp_char_device = {0};/*TODO structure dynamic allocation*/
//struct workqueue_struct *lvsd_workqueue_struct;

/*
 * The OPEN implementation on the Character Device
 */
static int vsp_char_open (struct inode *inode, struct file *filp) {
	int retval;

	ENTER();

	/* To verify that port is opened only once (increment the vsp_char_device.open_count atomic variable) //for single device creation only*/
	atomic_inc(&vsp_char_device.open_count);
	/*lvsd_workqueue_struct = create_workqueue("lvsdwq");
	if (!lvsd_workqueue_struct) {
		retval = -ENOMEM;
		LVSD_ERR("No memory to create workqueue struct");
		atomic_dec(&vsp_char_device.open_count);
		return retval;
	}*/

	/* Create the Message Queue which is used to inform different events to the VSAL*/
	retval = LvsdMsgQueueCreate(&vsp_char_device.message_queue, LVSD_MAX_NUMBER_OF_MSGS, sizeof(tLvsd_Event_Entry_t), "VspMsgQ");
	if (EOK != retval) {
		atomic_dec(&vsp_char_device.open_count);
		return retval;
	}

	/* Initialization of the tLvsd_Char_Dev_t structure*/
	/* Set the Head of the Devices Linked List as NULL*/
	if (atomic_read(&vsp_char_device.open_count) == 1) {
		LVSD_DEBUG("Setting head of device LL to NULL, as this is the first device to be created");
		vsp_char_device.devices_head = NULL;
	} else
		LVSD_DEBUG("This is second device creation so, so no need to make head to NULL, as head will contain 1st created device");

	LVSD_INFO("vsp_char_device.open_count: %d", atomic_read(&vsp_char_device.open_count));
	LEAVE();
	return 0;          /* success */
}

/*
 * The CLOSE implementation on the Character Device
 */
static int vsp_char_release (struct inode *inode, struct file *filp) {
	int retval;

	ENTER();
	LVSD_DEBUG("Destroyed Ports which are created only by IOCTL");

	/*Decrement the vsp_char_device.open_count atomic variable*/
	 atomic_dec(&vsp_char_device.open_count);

	/* Destroy the Message Queue - Only if this is last device to be removed*/
	if (atomic_read(&vsp_char_device.open_count) == 0) {
		LVSD_DEBUG("This was the last device to be deleted - to delete msgq and flush/destroy workqueues ");/*for multiple device functionality*/
		retval = LvsdMsgQueueDelete(vsp_char_device.message_queue);
		if (EOK != retval)
			return retval;
		/*flush_workqueue(lvsd_workqueue_struct);
		destroy_workqueue(lvsd_workqueue_struct);*/
	}

	LVSD_INFO("vsp_char_device.open_count: %d", atomic_read(&vsp_char_device.open_count));
	LEAVE();
	return 0;
}

/*
 * The IOCTL implementation on the Character Device
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
static long vsp_char_ioctl (struct file *filp, unsigned int cmd, unsigned long arg) {
#else
static long vsp_char_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg) {
#endif
	int				retval;
	unsigned long			handle;
	tLvsd_Vsp_t			device;
	tLvsd_Modem_Ctrl_Val_t		set_modem_ctrl, get_modem_ctrl;
	tLvsd_Freespace_t		buffer_freespace;
	tLvsd_Event_Entry_t		event_entry;
	tLvsd_Update_Read_Count_t	buf_read_cnt;
	tLvsd_Update_Pending_Write_t	pending_write;

	ENTER();

	switch (cmd) {
		/* Create a VSP*/
	case LVSD_CREATE_VSP:
			LVSD_DEBUG("LVSD_CREATE_VSP");
			/* Copy from User, the Name, Index, Buffer Size of a VSP that is to be created*/
			if (copy_from_user(&device, (tLvsd_Vsp_t *) arg, sizeof(tLvsd_Vsp_t))) {
				LVSD_ERR("Copy from User Failed");
				return -EFAULT;
			}

			/* Check if the Index of the VSP to be created is more than the MAX number of devices per name*/
			if (device.vsp_device_index >= LVSD_MAX_INDEX_PER_DEVICE_NAME) {
				LVSD_ERR("Cannot Create More than 32 VSPs with the Same Name");
				return -EINVAL;
			}
			LVSD_DEBUG("Device name we are passing is %s", device.vsp_device_name);

			/* Create a VSP with the passed device name, index, read and write buffer sizes (for eg: spp5, name is spp, index is 5)*/
			device.vsp_handle = create_vsp(device.vsp_device_name, device.vsp_device_index, device.vsp_rbuffer_size, device.vsp_wbuffer_size);
			if (device.vsp_handle == NULL) {
				LVSD_ERR("VSP Creation Failure");
				return -EINVAL;
			}
			LVSD_DEBUG("Created VSP handle is %p", device.vsp_handle);
			/* We return success or failure in the ret, but in the User passed argument we update the VSP handle which has been created*/
			if (copy_to_user((tLvsd_Vsp_t *) arg, &device, sizeof(tLvsd_Vsp_t))) {
				LVSD_ERR("Copy to User Failed");
				return -EFAULT;
			}
			/*Setup filp to vsp handle for write ctrlx usage*/
			filp->private_data = device.vsp_handle;

			break;

		/* Destroy a VSP*/
	case LVSD_DESTROY_VSP:
			LVSD_DEBUG("LVSD_DESTROY_VSP");
			/* Get from User, the handle of the VSP that has to be destroyed*/
			retval = __get_user(handle, (unsigned long __user *) arg);
			if (retval)	{
				LVSD_ERR("Get from User Failed");
				return retval;
			}

			//LVSD_DEBUG("Delete VSP handle is %p", handle);
			/* Do a TTY Shutdown */
			retval = shutdown_vsp((tLvsd_Uart_Port_t *)handle);
			if (retval == 0) {
				LVSD_DEBUG("TTY Shutdown/Destroy failure as No TTY");
				/* Destroy the VSP*/
				retval = destroy_vsp((tLvsd_Uart_Port_t *)handle);
			} else if (retval) {
				LVSD_DEBUG("Destroy of VSP from shutdown was successful");
				retval = 0;
			} else
				LVSD_DEBUG("NTD");

			return retval;


			break;

		/* Set the Modem Control Value of a VSP*/
	case LVSD_SET_MODEM_CTRL:
			LVSD_DEBUG("LVSD_SET_MODEM_CTRL");
			/* Copy from User, the Handle, the Modem Control Value of the VSP*/
			if (copy_from_user(&set_modem_ctrl, (tLvsd_Modem_Ctrl_Val_t *) arg, sizeof(tLvsd_Modem_Ctrl_Val_t))) {
				LVSD_ERR("Copy from User Failed");
				return -EFAULT;
			}

			/* Update the Modem Control Value of the VSP*/
			uart_update_mctrl(&(((tLvsd_Uart_Port_t *)set_modem_ctrl.vsp_handle)->port), set_modem_ctrl.vsp_mcr_val, ~(set_modem_ctrl.vsp_mcr_val));
			break;

		/* Get the Modem Control Value of a VSP*/
	case LVSD_GET_MODEM_CTRL:
			LVSD_DEBUG("LVSD_GET_MODEM_CTRL");
			/* Copy from User, the Handle of the VSP, whose Modem Control Value needs to be known by the User*/
			if (copy_from_user(&get_modem_ctrl, (tLvsd_Modem_Ctrl_Val_t *) arg, sizeof(tLvsd_Modem_Ctrl_Val_t))) {
				LVSD_ERR("Copy from User Failed");
				return -EFAULT;
			}

			/* Get the Modem Control Value of the VSP*/
			get_modem_ctrl.vsp_mcr_val = uart_get_mctrl(&(((tLvsd_Uart_Port_t *)get_modem_ctrl.vsp_handle)->port));

			/* Copy to User, the Modem Control Value of the VSP. The Modem Control Value has been updated in the User passed argument*/
			if (copy_to_user((tLvsd_Modem_Ctrl_Val_t *) arg, &get_modem_ctrl, sizeof(tLvsd_Modem_Ctrl_Val_t))) {
				LVSD_ERR("Copy to User Failed");
				return -EFAULT;
			}
			break;

		/* Get the Free Space in the RBUF of a VSP*/
	case LVSD_GET_FREESPACE:
			LVSD_DEBUG("LVSD_GET_FREESPACE");
			/* Copy from User, the Handle of the VSP, whose Freespace in RBUF needs to be known by the User*/
			if (copy_from_user(&buffer_freespace, (tLvsd_Freespace_t *) arg, sizeof(tLvsd_Freespace_t))) {
				LVSD_ERR("Copy from User Failed");
				return -EFAULT;
			}

			/* Get the Freespace in the RBUF of the VSP*/
			buffer_freespace.freespace = uart_get_freespace(&(((tLvsd_Uart_Port_t *)buffer_freespace.vsp_handle)->port));

			/* Copy to User, the Freespace in the RBUF of the VSP. The Freespace in the RBUF has been updated in the User passed argument*/
			if (copy_to_user((tLvsd_Freespace_t *) arg, &buffer_freespace, sizeof(tLvsd_Freespace_t))) {
				LVSD_ERR("Copy to User Failed");
				return -EFAULT;
			}
			break;

		/* Various Events (like OPEN, CLOSE events on a VSP) are updated to the VSAL with this IOCTL (LVSD_WAIT_FOR_EVENTS)*/
	case LVSD_WAIT_FOR_EVENTS:
			LVSD_DEBUG("LVSD_WAIT_FOR_EVENTS");
			LVSD_DEBUG("The VSP handle is %p", event_entry.vsp_handle);

			do {
				/* Get the Event from the Message Queue*/
				retval = LvsdMsgQueueGet(NULL, vsp_char_device.message_queue, &event_entry, sizeof(tLvsd_Event_Entry_t), LVSD_TIMEOUT_INFINITE);
				if (EOK != retval)
					return retval;

				LVSD_DEBUG("event_entry.event: %d", event_entry.event);
				LVSD_DEBUG("event_entry.vsp_handle: %p", event_entry.vsp_handle);
				LVSD_DEBUG("event_entry.data_offset: %d", event_entry.data_offset);
				LVSD_DEBUG("event_entry.size: %d", event_entry.size);
			} while ((unsigned long)(event_entry.vsp_handle) % BYTES_ALIGNED != 0); /*the invalidated message entry has the vsp_handle's first bit set*/

			/* Copy to User, the Event from the Message Queue*/
			if (copy_to_user((tLvsd_Event_Entry_t *) arg, &event_entry, sizeof(tLvsd_Event_Entry_t))) {
				LVSD_ERR("Copy to User Failed");
				return -EFAULT;
			}
			break;

		/* Update the data read count in the Circular Buffer (RBuffer) of a VSP*/
	case LVSD_UPDATE_READ_CNT:
			LVSD_DEBUG("LVSD_UPDATE_READ_CNT");
			/* Copy from User, the Handle, read count value in the RBuffer of a VSP*/
			if (copy_from_user(&buf_read_cnt, (tLvsd_Update_Read_Count_t *) arg, sizeof(tLvsd_Update_Read_Count_t))) {
				LVSD_ERR("Copy from User Failed");
				return -EFAULT;
			}

			/* Update the data read count in the Circular Buffer (RBuffer) of the VSP*/
			update_circ_read_cnt((tLvsd_Uart_Port_t *)buf_read_cnt.vsp_handle, buf_read_cnt.read_cnt);

			break;

	case LVSD_DO_PENDING_WRITE:
			LVSD_DEBUG("LVSD_DO_PENDING_WRITE");
			/* Copy from user, the handle and pending write flag*/
			if (copy_from_user(&pending_write, (tLvsd_Update_Pending_Write_t *) arg, sizeof(tLvsd_Update_Pending_Write_t))) {
				LVSD_ERR("Copy from user Failed");
				return -EFAULT;
			}
			retval = trigger_pending_write((tLvsd_Uart_Port_t *)pending_write.vsp_handle, pending_write.write_flag);
			if (retval == 1)
				LVSD_DEBUG("Event already sent for residual data and we compensated it");
			else if (retval == 0)
				LVSD_DEBUG("Trigger pending write succeess");
			else
				LVSD_DEBUG("Something is wrong");
			
			break;

	default:
			LVSD_ERR("Invalid ioctl argument passed: cmd: %u", cmd);
			return -ENOTTY;
	}

	LEAVE();
	return 0;
}

/*
 * The MMAP implementation on the Character Device
 */
static int vsp_char_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret = 0;
	tLvsd_Uart_Port_t	*vsp;
	tLvsd_Mmap_Buf_Info_t	mmap_buf_struct;
	unsigned long wbuf_phy_addrs, rbuf_phy_addrs;

	ENTER();
	LVSD_DEBUG("vma->vm_pgoff: 0x%x", (unsigned int)vma->vm_pgoff);

	/* The Structure at the User space address (vma->vm_pgoff << PAGE_SHIFT), has the Handle of the VSP,
	 * the Value which specifies whether the Read/Write buffer needs to be Mmapped */
	if (copy_from_user(&mmap_buf_struct, (tLvsd_Mmap_Buf_Info_t *)(vma->vm_pgoff << PAGE_SHIFT), sizeof(tLvsd_Mmap_Buf_Info_t))) {
		LVSD_DEBUG("Copy from User Failed");
		return -EFAULT;
	}

	/* The Handle of the VSP*/
	vsp = (tLvsd_Uart_Port_t *)(mmap_buf_struct.vsp_handle);
	LVSD_DEBUG("vsp: %p", vsp);
	if (!vsp) {
		LVSD_DEBUG("VSP whose Rbuffer / Wbuffer that needs to the Mmapped cannot be NULL");
		return -EINVAL;
	}

	LVSD_DEBUG("vsp->rbuffer.buf: %p", vsp->rbuffer.buf);
	LVSD_DEBUG("vsp->wbuffer: %p", vsp->wbuffer);

	wbuf_phy_addrs = virt_to_phys(vsp->wbuffer);
	LVSD_DEBUG("temporary physicall wbuf addr is %ld", wbuf_phy_addrs);

	rbuf_phy_addrs = virt_to_phys(vsp->rbuffer.buf);
	LVSD_DEBUG("temporary physicall rbuf addr is %ld", rbuf_phy_addrs);
	/*Converting Kernel Space R/W buffer, to user space mapped memory*/
	/* If the RBuffer (Circular Buffer) needs to be Mmapped*/
	if (mmap_buf_struct.read_write == LVSD_ACCESS_READ_BUF) {
		LVSD_DEBUG("Mapping Read Buffer");
		if ((remap_pfn_range(vma, vma->vm_start, virt_to_phys((void *)vsp->rbuffer.buf) >> PAGE_SHIFT, vsp->rbuffer_size, vma->vm_page_prot)) < 0) {
			LVSD_DEBUG("Mapping Read Buffer failed");
			return -EIO;
		}

		LVSD_DEBUG("Mapping Read Buffer Success");
		LEAVE();
	}
	/* If the Wbuffer needs to be Mmapped*/
	else if (mmap_buf_struct.read_write == LVSD_ACCESS_WRITE_BUF) {
		LVSD_DEBUG("Mapping Write Buffer");
		if ((remap_pfn_range(vma, vma->vm_start, virt_to_phys((void *)vsp->wbuffer) >> PAGE_SHIFT, vsp->wbuffer_size, vma->vm_page_prot)) < 0) {
			LVSD_DEBUG("Mapping Write Buffer failed");
			return -EIO;
		}

		LVSD_DEBUG("Mapping Write Buffer Success");
	//	ret = 1;
		LEAVE();
	}
	/* If neither of the buffers needs to be Mmapped, then return error*/
	else {
		LVSD_ERR("Memory to be Mapped is neither Read nor Write Buffer");
		return -EINVAL;
	}

	/*Create the actual File system entry For our TTY device
	if (ret) {
		ret = lvsd_tty_register_device(vsp);
		LVSD_DEBUG("TTY Device Registered from MMAP Write function");
	}*/

	LEAVE();
	return ret;

}

/*
 * The WRITE implementation on the Character Device
 */
static ssize_t vsp_char_write (struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
	unsigned int	ret = 0;
	unsigned int	fill_level;
	unsigned char	*user_data;	/*This will get de-allocated once this function is returned*/
	tLvsd_Uart_Port_t	*vsp;

	ENTER();

	if (buf != NULL) {
		/*Allocate memory for copying user space buffer*/
		user_data = kzalloc(LVSD_VSAL_MAX_WRITE_SIZE, GFP_KERNEL);
		if (!user_data) {
			LVSD_ERR("Allocation of kernel memory for copying user space buffer failed");
			goto done;
		} else {
			/*Copy from User space the user data buffer*/
			if (copy_from_user(user_data, buf, (sizeof(char) * count))) {
				LVSD_ERR("Copy from User Failed");
				goto dealloc;
			}
		}
	} else {
		LVSD_DEBUG("User Space buffer cannot be NULL");
		return -1;
	}

	LVSD_DEBUG("filp->private_data %p", filp->private_data);

	vsp = (tLvsd_Uart_Port_t *)(filp->private_data);
	LVSD_DEBUG("filp->private_data %p, and vsp is %p and these must be equal for a vsp", filp->private_data, vsp);

	/*Copy the Data from the Wbuffer of the VSP to the RBUF of the VSP, which can be read by a Serial Application on that VSP*/
	if (!vsp) {
		LVSD_ERR("passed VSP handle cannot be NULL");
		return -EINVAL;
	} else {
		memcpy(vsp->wbuffer, user_data, count);
		/*Increment the write buffer fill status and pass a copy of it to receive_chars function. variable name is current fillstatus*/
		fill_level = atomic_add_return(count, &(vsp->wbuf_fill_level));
		ret = receive_chars(vsp, count, fill_level);
		if (ret == 0) {
			LVSD_DEBUG("Pushing data to TTY buffers failed - as there was no TTY\n\t\t Invoking Alternate method for buffering data on uart_open");
			ret = store_wbuff_data(vsp, count, fill_level);
		} else {
			LVSD_DEBUG("%d bytes written to TTY Buffers successfully", ret);
			goto done;
			LEAVE();
		}
	}
dealloc:
	kfree(user_data);
done:
	return ret;
}

/* Various File Operations on the Character Device Interface*/
static const struct file_operations vsp_char_fops = {
	.open	=	vsp_char_open,
	.release =	vsp_char_release,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
#ifdef HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl
#else
	.ioctl
#endif
#else
	.ioctl
#endif
		=	vsp_char_ioctl,
	.mmap	=	vsp_char_mmap,
	.write	=	vsp_char_write,
	.owner	=	THIS_MODULE,
};

/* Module Initialization Function of the LVSD*/
static int __init vsp_char_init(void)
{
	int ret;
	dev_t dev = MKDEV(500, 0);
	struct device *vsp_char_dev = NULL;

	ENTER();
	/* Register the Character Device Region for one Device*/
	ret = register_chrdev_region(dev, 1, "ctrlX");
	if (ret) {
		LVSD_ERR("register_chrdev_region for </dev/ctrlX> failure");
		goto error;
	}

	/* Initialize the Global Character Device structure*/
	cdev_init(&vsp_char_device.lvsd_cdev, &vsp_char_fops);
	vsp_char_device.lvsd_cdev.owner = THIS_MODULE;
	ret = cdev_add(&vsp_char_device.lvsd_cdev, dev, 1);
	if (ret) {
		LVSD_ERR("cdev_add for </dev/ctrlX> failure");
		goto error_region;
	}

	/* Create the ctrlX class*/
	vsp_char_class = class_create(THIS_MODULE, "ctrlX");
	if (IS_ERR(vsp_char_class)) {
		LVSD_ERR("error creating ctrlX class");
		cdev_del(&vsp_char_device.lvsd_cdev);
		ret = PTR_ERR(vsp_char_class);
		goto error_region;
	}

	/* Create the ctrlX device*/
	vsp_char_dev = device_create(vsp_char_class, NULL, MKDEV(500, 0), NULL, "ctrlX");
	if (IS_ERR(vsp_char_dev)) {
		LVSD_ERR("error creating ctrlX device");
		class_destroy(vsp_char_class);
		cdev_del(&vsp_char_device.lvsd_cdev);
		ret = PTR_ERR(vsp_char_dev);
		goto error_region;
	}

	/* Initialize the VSP Linked List Mutex*/
	mutex_init(&vsp_char_device.devices_list_lock); /* Initialize the mutex ll_minor_lock*/
	atomic_set(&vsp_char_device.open_count, 0);

	LEAVE();
	LVSD_DEBUG("LVSD Control Interface Driver Initialized -- Version 2.0");
	return 0;

error_region:
	unregister_chrdev_region(dev, 1);
error:
	return ret;
}

/*/ Module Exit Function of the LVSD*/
static void __exit vsp_char_exit(void)
{
	ENTER();

	/* Destroy the ctrlX device, ctrlX class and unregister the Character Device Region for that device*/
	device_destroy(vsp_char_class, MKDEV(500, 0));
	class_destroy(vsp_char_class);
	cdev_del(&vsp_char_device.lvsd_cdev);
	unregister_chrdev_region(MKDEV(500, 0), 1);
	LVSD_DEBUG("LVSD Control Interface Driver Exited -- Version 2.0");

	LEAVE();
}

module_init(vsp_char_init);
module_exit(vsp_char_exit);
MODULE_LICENSE("GPL v2");
