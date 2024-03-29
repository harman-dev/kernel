/** @file moal_pcie.c
 *
 *  @brief This file contains PCIE IF (interface) module
 *  related functions.
 *
 *  (C) Copyright 2014-2017 Marvell International Ltd. All Rights Reserved
 *
 *  MARVELL CONFIDENTIAL
 *  The source code contained or described herein and all documents related to
 *  the source code ("Material") are owned by Marvell International Ltd or its
 *  suppliers or licensors. Title to the Material remains with Marvell
 *  International Ltd or its suppliers and licensors. The Material contains
 *  trade secrets and proprietary and confidential information of Marvell or its
 *  suppliers and licensors. The Material is protected by worldwide copyright
 *  and trade secret laws and treaty provisions. No part of the Material may be
 *  used, copied, reproduced, modified, published, uploaded, posted,
 *  transmitted, distributed, or disclosed in any way without Marvell's prior
 *  express written permission.
 *
 *  No license under any patent, copyright, trade secret or other intellectual
 *  property right is granted to or conferred upon you by disclosure or delivery
 *  of the Materials, either expressly, by implication, inducement, estoppel or
 *  otherwise. Any license under such intellectual property rights must be
 *  express and approved by Marvell in writing.
 *
 */

/********************************************************
Change log:
    02/01/2012: initial version
********************************************************/

#include <linux/firmware.h>

#include	"moal_pcie.h"

/********************************************************
			Local Variables
********************************************************/
#define DRV_NAME        "Marvell mdriver PCIe"

/* PCIE resume handler */
static int woal_pcie_resume(struct pci_dev *pdev);

extern int pcie_int_mode;

/** WLAN IDs */
static const struct pci_device_id wlan_ids[] = {
	{
	 PCIE_VENDOR_ID_MARVELL, PCIE_DEVICE_ID_MARVELL_88W8897P,
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0,
	 },
	{},
};

MODULE_DEVICE_TABLE(pci, wlan_ids);

/********************************************************
			Global Variables
********************************************************/

/********************************************************
			Local Functions
********************************************************/
static mlan_status woal_pcie_preinit(struct pci_dev *pdev);

/**
 *  @brief This function handles PCIE driver probe
 *
 *  @param pdev     A pointer to pci_dev structure
 *  @param id       A pointer to pci_device_id structure
 *
 *  @return         error code
 */
int
woal_pcie_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	pcie_service_card *card;

	ENTER();

	PRINTM(MINFO, "vendor=0x%4.04X device=0x%4.04X rev=%d\n",
	       pdev->vendor, pdev->device, pdev->revision);

	/* Preinit PCIE device so allocate PCIE memory can be successful */
	if (woal_pcie_preinit(pdev)) {
		PRINTM(MFATAL, "MOAL PCIE preinit failed\n");
		LEAVE();
		return -EFAULT;
	}

	card = kzalloc(sizeof(pcie_service_card), GFP_KERNEL);
	if (!card) {
		PRINTM(MERROR, "%s: failed to alloc memory\n", __func__);
		pci_disable_device(pdev);
		LEAVE();
		return -ENOMEM;
	}

	card->dev = pdev;

	if (woal_add_card(card) == NULL) {
		PRINTM(MERROR, "%s: failed\n", __func__);
		if (pci_is_enabled(pdev))
			pci_disable_device(pdev);
		kfree(card);
		LEAVE();
		return -EFAULT;
	}

	LEAVE();
	return 0;
}

/**
 *  @brief This function handles PCIE driver remove
 *
 *  @param pdev     A pointer to pci_dev structure
 *
 *  @return         error code
 */
static void
woal_pcie_remove(struct pci_dev *dev)
{
	pcie_service_card *card;
	moal_handle *handle;

	ENTER();
	card = pci_get_drvdata(dev);
	if (!card) {
		PRINTM(MINFO, "PCIE card removed from slot\n");
		LEAVE();
		return;
	}

	handle = card->handle;
	if (!handle || !handle->priv_num) {
		PRINTM(MINFO, "PCIE card handle removed\n");
		LEAVE();
		return;
	}
	handle->surprise_removed = MTRUE;
/*#ifndef MFR_SUPPORT
	if (dev) {
		pci_set_power_state(dev, PCI_D3cold);
		PRINTM(MINFO, "Shut down PCIE\n");
	}
#endif*/
	woal_remove_card(card);
	kfree(card);

	LEAVE();
	return;
}

/**
 *  @brief Handle suspend
 *
 *  @param pdev     A pointer to pci_dev structure
 *  @param state    PM state message
 *
 *  @return         error code
 */
static int
woal_pcie_suspend(struct pci_dev *pdev, pm_message_t state)
{
	pcie_service_card *cardp;
	moal_handle *handle = NULL;
	int i;
	int ret = MLAN_STATUS_SUCCESS;
	int hs_actived;
	mlan_ds_ps_info pm_info;

	ENTER();
	PRINTM(MCMND, "<--- Enter woal_pcie_suspend --->\n");
	if (pdev) {
		cardp = (pcie_service_card *)pci_get_drvdata(pdev);
		if (!cardp || !cardp->handle) {
			LEAVE();
			return MLAN_STATUS_SUCCESS;
		}
	} else {
		PRINTM(MERROR, "PCIE device is not specified\n");
		LEAVE();
		return -ENOSYS;
	}

	handle = cardp->handle;
	if (handle->is_suspended == MTRUE) {
		PRINTM(MWARN, "Device already suspended\n");
		LEAVE();
		return MLAN_STATUS_SUCCESS;
	}

	handle->suspend_fail = MFALSE;
	memset(&pm_info, 0, sizeof(pm_info));
	if (MLAN_STATUS_SUCCESS ==
	    woal_get_pm_info(woal_get_priv(handle, MLAN_BSS_ROLE_ANY),
			     &pm_info)) {
		if (pm_info.is_suspend_allowed == MFALSE) {
			PRINTM(MMSG, "suspend not allowed!");
			goto done;
	    }
	}


	for (i = 0; i < handle->priv_num; i++)
		netif_device_detach(handle->priv[i]->netdev);

	/* Enable Host Sleep */
	hs_actived = woal_enable_hs(woal_get_priv(handle, MLAN_BSS_ROLE_ANY));
	if (hs_actived == MTRUE) {
		/* Indicate device suspended */
		handle->is_suspended = MTRUE;
	} else {
		PRINTM(MMSG, "HS not actived, suspend fail!");
		handle->suspend_fail = MTRUE;
		for (i = 0; i < handle->priv_num; i++)
			netif_device_attach(handle->priv[i]->netdev);
		goto done;
	}
	flush_workqueue(handle->workqueue);
	if (handle->rx_workqueue)
		flush_workqueue(handle->rx_workqueue);
done:
	PRINTM(MCMND, "<--- Leave woal_pcie_suspend --->\n");
	LEAVE();
	return ret;
}

/**
 *  @brief Handle resume
 *
 *  @param pdev     A pointer to pci_dev structure
 *
 *  @return         error code
 */
static int
woal_pcie_resume(struct pci_dev *pdev)
{
	moal_handle *handle;
	pcie_service_card *card;
	int i;

	ENTER();

	PRINTM(MCMND, "<--- Enter woal_pcie_resume --->\n");
	if (pdev) {
		card = (pcie_service_card *)pci_get_drvdata(pdev);
		if (!card || !card->handle) {
			PRINTM(MERROR, "Card or handle is not valid\n");
			LEAVE();
			return MLAN_STATUS_SUCCESS;
		}
	} else {
		PRINTM(MERROR, "PCIE device is not specified\n");
		LEAVE();
		return -ENOSYS;
	}

	handle = card->handle;

	if (handle->is_suspended == MFALSE) {
		PRINTM(MWARN, "Device already resumed\n");
		goto done;
	}

	handle->is_suspended = MFALSE;

	if (woal_check_driver_status(handle)) {
		PRINTM(MERROR, "Resuem, device is in hang state\n");
		LEAVE();
		return MLAN_STATUS_SUCCESS;
	}
	for (i = 0; i < handle->priv_num; i++)
		netif_device_attach(handle->priv[i]->netdev);

	woal_cancel_hs(woal_get_priv(handle, MLAN_BSS_ROLE_ANY), MOAL_NO_WAIT);

done:
	PRINTM(MCMND, "<--- Leave woal_pcie_resume --->\n");
	LEAVE();
	return 0;
}

/* PCI Device Driver */
static struct pci_driver REFDATA wlan_pcie = {
	.name = "wlan_pcie",
	.id_table = wlan_ids,
	.probe = woal_pcie_probe,
	.remove = woal_pcie_remove,
#ifdef CONFIG_PM
	/* Power Management Hooks */
	.suspend = woal_pcie_suspend,
	.resume = woal_pcie_resume,
#endif
};

/********************************************************
			Global Functions
********************************************************/

/**
 *  @brief This function writes data into card register
 *
 *  @param handle   A Pointer to the moal_handle structure
 *  @param reg      Register offset
 *  @param data     Value
 *
 *  @return    		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
woal_write_reg(moal_handle *handle, t_u32 reg, t_u32 data)
{
	pcie_service_card *card = (pcie_service_card *)handle->card;

	iowrite32(data, card->pci_mmap1 + reg);

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function reads data from card register
 *
 *  @param handle   A Pointer to the moal_handle structure
 *  @param reg      Register offset
 *  @param data     Value
 *
 *  @return    		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
woal_read_reg(moal_handle *handle, t_u32 reg, t_u32 *data)
{
	pcie_service_card *card = (pcie_service_card *)handle->card;
	*data = ioread32(card->pci_mmap1 + reg);

	if (*data == MLAN_STATUS_FAILURE)
		return MLAN_STATUS_FAILURE;

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function writes multiple bytes into card memory
 *
 *  @param handle   A Pointer to the moal_handle structure
 *  @param pmbuf	Pointer to mlan_buffer structure
 *  @param port		Port
 *  @param timeout 	Time out value
 *
 *  @return    		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
woal_write_data_sync(moal_handle *handle,
		     mlan_buffer *pmbuf, t_u32 port, t_u32 timeout)
{
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function reads multiple bytes from card memory
 *
 *  @param handle   A Pointer to the moal_handle structure
 *  @param pmbuf	Pointer to mlan_buffer structure
 *  @param port		Port
 *  @param timeout 	Time out value
 *
 *  @return    		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
woal_read_data_sync(moal_handle *handle,
		    mlan_buffer *pmbuf, t_u32 port, t_u32 timeout)
{
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function handles the interrupt.
 *
 *  @param irq	    The irq no. of PCIE device
 *  @param dev_id   A pointer to the pci_dev structure
 *
 *  @return         IRQ_HANDLED
 */
static irqreturn_t
woal_pcie_interrupt(int irq, void *dev_id)
{
	struct pci_dev *pdev;
	pcie_service_card *card;
	moal_handle *handle;

	pdev = (struct pci_dev *)dev_id;

	if (!pdev) {
		PRINTM(MFATAL, "%s: pdev is NULL\n", (t_u8 *)pdev);
		goto exit;
	}

	card = (pcie_service_card *)pci_get_drvdata(pdev);
	if (!card || !card->handle) {
		PRINTM(MFATAL, "%s: card=%p handle=%p\n", __func__, card,
		       card ? card->handle : NULL);
		goto exit;
	}
	handle = card->handle;

	PRINTM(MINFO, "*** IN PCIE IRQ ***\n");
	woal_interrupt(handle);

exit:
	return IRQ_HANDLED;
}

/**
 *  @brief This function pre-initializes the PCI-E host
 *  memory space, etc.
 *
 *  @param handle   A pointer to moal_handle structure
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
woal_pcie_preinit(struct pci_dev *pdev)
{
	int ret;

	ret = pci_enable_device(pdev);
	if (ret)
		goto err_enable_dev;

	pci_set_master(pdev);

	PRINTM(MINFO, "Try set_consistent_dma_mask(32)\n");
	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret) {
		PRINTM(MERROR, "set_dma_mask(32) failed\n");
		goto err_set_dma_mask;
	}

	ret = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret) {
		PRINTM(MERROR, "set_consistent_dma_mask(64) failed\n");
		goto err_set_dma_mask;
	}
	return MLAN_STATUS_SUCCESS;

err_set_dma_mask:
	pci_disable_device(pdev);
err_enable_dev:
	return MLAN_STATUS_FAILURE;
}

/**
 *  @brief This function initializes the PCI-E host
 *  memory space, etc.
 *
 *  @param handle   A pointer to moal_handle structure
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
woal_pcie_init(moal_handle *handle)
{
	pcie_service_card *card = NULL;
	struct pci_dev *pdev = NULL;
	int ret;

	if (!handle || !handle->card) {
		PRINTM(MINFO, "%s: handle=%p card=%p\n", __FUNCTION__, handle,
		       handle ? handle->card : NULL);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	card = (pcie_service_card *)handle->card;
	pdev = card->dev;
	pci_set_drvdata(pdev, card);
#if 0
	ret = pci_enable_device(pdev);
	if (ret)
		goto err_enable_dev;

	pci_set_master(pdev);

	PRINTM(MINFO, "Try set_consistent_dma_mask(32)\n");
	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret) {
		PRINTM(MERROR, "set_dma_mask(32) failed\n");
		goto err_set_dma_mask;
	}

	ret = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret) {
		PRINTM(MERROR, "set_consistent_dma_mask(64) failed\n");
		goto err_set_dma_mask;
	}
#endif

	ret = pci_request_region(pdev, 0, DRV_NAME);
	if (ret) {
		PRINTM(MERROR, "req_reg(0) error\n");
		goto err_req_region0;
	}
	card->pci_mmap = pci_iomap(pdev, 0, 0);
	if (!card->pci_mmap) {
		PRINTM(MERROR, "iomap(0) error\n");
		goto err_iomap0;
	}
	ret = pci_request_region(pdev, 2, DRV_NAME);
	if (ret) {
		PRINTM(MERROR, "req_reg(2) error\n");
		goto err_req_region2;
	}
	card->pci_mmap1 = pci_iomap(pdev, 2, 0);
	if (!card->pci_mmap1) {
		PRINTM(MERROR, "iomap(2) error\n");
		goto err_iomap2;
	}

	PRINTM(MINFO, "PCI memory map Virt0: %p PCI memory map Virt2: "
	       "%p\n", card->pci_mmap, card->pci_mmap1);

	return MLAN_STATUS_SUCCESS;

err_iomap2:
	pci_release_region(pdev, 2);
err_req_region2:
	pci_iounmap(pdev, card->pci_mmap);
err_iomap0:
	pci_release_region(pdev, 0);
err_req_region0:
#if 0
err_set_dma_mask:
#endif

#if 0
err_enable_dev:
#endif
	pci_set_drvdata(pdev, NULL);
	return MLAN_STATUS_FAILURE;
}

/**
 *  @brief This function registers the PCIE device
 *
 *  @param handle   A pointer to moal_handle structure
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
woal_register_dev(moal_handle *handle)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pcie_service_card *card = NULL;
	struct pci_dev *pdev = NULL;
	ENTER();

	if (!handle || !handle->card) {
		PRINTM(MINFO, "%s: handle=%p card=%p\n", __FUNCTION__, handle,
		       handle ? handle->card : NULL);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	card = (pcie_service_card *)handle->card;
	pdev = card->dev;
	/* save adapter pointer in card */
	card->handle = handle;

	if (woal_pcie_init(handle)) {
		PRINTM(MFATAL, "MOAL PCIE init failed\n");
		handle->card = NULL;
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	switch (pcie_int_mode) {

	case PCIE_INT_MODE_LEGACY:
		pcie_int_mode = PCIE_INT_MODE_LEGACY;
		ret = request_irq(pdev->irq,
				  woal_pcie_interrupt, IRQF_SHARED,
				  "mrvl_pcie", pdev);
		if (ret) {
			PRINTM(MFATAL, "request_irq failed: ret=%d\n", ret);
			handle->card = NULL;
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		break;

	default:
		PRINTM(MFATAL, "pcie_int_mode %d failed\n", pcie_int_mode);
		handle->card = NULL;
		ret = MLAN_STATUS_FAILURE;
		goto done;
		break;
	}

	handle->hotplug_device = &pdev->dev;

done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function cleans up the host memory spaces
 *
 *  @param handle   A pointer to moal_handle structure
 *
 *  @return         N/A
 */
static void
woal_pcie_cleanup(moal_handle *handle)
{
	pcie_service_card *card = NULL;
	struct pci_dev *pdev = NULL;

	if (!handle || !handle->card) {
		PRINTM(MINFO, "%s: handle=%p card=%p\n", __FUNCTION__, handle,
		       handle ? handle->card : NULL);
		return;
	}

	card = (pcie_service_card *)handle->card;
	pdev = card->dev;
	PRINTM(MINFO, "Clearing driver ready signature\n");

	if (pdev) {
		pci_iounmap(pdev, card->pci_mmap);
		pci_iounmap(pdev, card->pci_mmap1);

		if (pci_is_enabled(pdev))
			pci_disable_device(pdev);

		pci_release_regions(pdev);
		pci_set_drvdata(pdev, NULL);
	}
}

/**
 *  @brief This function unregisters the PCIE device
 *
 *  @param handle   A pointer to moal_handle structure
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
void
woal_unregister_dev(moal_handle *handle)
{
	pcie_service_card *card =
		handle ? (pcie_service_card *)handle->card : NULL;
	struct pci_dev *pdev = NULL;
	ENTER();

	if (card) {
		pdev = card->dev;
		woal_pcie_cleanup(handle);
		PRINTM(MINFO, "%s(): calling free_irq()\n", __func__);

		switch (pcie_int_mode) {

		case PCIE_INT_MODE_LEGACY:
			free_irq(card->dev->irq, pdev);
			break;

		default:
			PRINTM(MFATAL, "pcie_int_mode %d failed\n",
			       pcie_int_mode);
			break;
		}
	}
	LEAVE();
}

/**
 *  @brief This function registers the IF module in bus driver
 *
 *  @return	    MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
woal_bus_register(void)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	ENTER();

	/* API registers the Marvell PCIE driver */
	if (pci_register_driver(&wlan_pcie)) {
		PRINTM(MFATAL, "PCIE Driver Registration Failed \n");
		ret = MLAN_STATUS_FAILURE;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief This function de-registers the IF module in bus driver
 *
 *  @return 	   N/A
 */
void
woal_bus_unregister(void)
{
	ENTER();

	/* PCIE Driver Unregistration */
	pci_unregister_driver(&wlan_pcie);

	LEAVE();
}

#define DEBUG_DUMP_CTRL_REG               0xCF4
#define DEBUG_DUMP_START_REG              0xCF8
#define DEBUG_DUMP_END_REG                0xCFF
#define DEBUG_FW_DONE                     0xFF
#define MAX_POLL_TRIES                    100

typedef enum {
	DUMP_TYPE_ITCM = 0,
	DUMP_TYPE_DTCM = 1,
	DUMP_TYPE_SQRAM = 2,
	DUMP_TYPE_IRAM = 3,
	DUMP_TYPE_APU = 4,
	DUMP_TYPE_CIU = 5,
	DUMP_TYPE_ICU = 6,
	DUMP_TYPE_MAC = 7,

} dumped_mem_type;

#define MAX_NAME_LEN                     8
#define MAX_FULL_NAME_LEN               32
t_u8 *name_prefix = "/data/file_";

typedef struct {
	t_u8 mem_name[MAX_NAME_LEN];
	t_u8 *mem_Ptr;
	struct file *pfile_mem;
	t_u8 done_flag;
} memory_type_mapping;

typedef enum {
	RDWR_STATUS_SUCCESS = 0,
	RDWR_STATUS_FAILURE = 1,
	RDWR_STATUS_DONE = 2
} rdwr_status;

#define DEBUG_HOST_READY                  0xEE
#define DEBUG_MEMDUMP_FINISH              0xFE
memory_type_mapping mem_type_mapping_tbl[] = {
	{"ITCM", NULL, NULL, 0xF0},
	{"DTCM", NULL, NULL, 0xF1},
	{"SQRAM", NULL, NULL, 0xF2},
	{"IRAM", NULL, NULL, 0xF3},
	{"APU", NULL, NULL, 0xF4},
	{"CIU", NULL, NULL, 0xF5},
	{"ICU", NULL, NULL, 0xF6},
	{"MAC", NULL, NULL, 0xF7},

};

/**
 *  @brief This function reads data by 8 bit from card register
 *
 *  @param handle   A Pointer to the moal_handle structure
 *  @param reg      Register offset
 *  @param data     Value
 *
 *  @return    		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
woal_read_reg_eight_bit(moal_handle *handle, t_u32 reg, t_u8 *data)
{
	pcie_service_card *card = (pcie_service_card *)handle->card;
	*data = ioread8(card->pci_mmap1 + reg);
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function read/write firmware
 *
 *  @param phandle   A pointer to moal_handle
 *  @param doneflag  done flag
 *
 *  @return         MLAN_STATUS_SUCCESS
 */
rdwr_status
woal_pcie_rdwr_firmware(moal_handle *phandle, t_u8 doneflag)
{
	int ret = 0;
	int tries = 0;
	t_u8 ctrl_data = 0;
	t_u32 reg_data;

	ret = woal_write_reg(phandle, DEBUG_DUMP_CTRL_REG, DEBUG_HOST_READY);
	if (ret) {
		PRINTM(MERROR, "PCIE Write ERR\n");
		return RDWR_STATUS_FAILURE;
	}
	ret = woal_read_reg(phandle, DEBUG_DUMP_CTRL_REG, &reg_data);
	if (ret) {
		PRINTM(MERROR, "PCIE Read DEBUG_DUMP_CTRL_REG fail\n");
		return RDWR_STATUS_FAILURE;
	}
	for (tries = 0; tries < MAX_POLL_TRIES; tries++) {
		ret = woal_read_reg_eight_bit(phandle, DEBUG_DUMP_CTRL_REG,
					      &ctrl_data);
		if (ret) {
			PRINTM(MERROR, "PCIE READ ERR\n");
			return RDWR_STATUS_FAILURE;
		}
		if (ctrl_data == DEBUG_FW_DONE)
			break;
		if (doneflag && ctrl_data == doneflag)
			return RDWR_STATUS_DONE;
		if (ctrl_data != DEBUG_HOST_READY) {
			PRINTM(MMSG, "The ctrl reg was changed, try again!\n");
			ret = woal_write_reg(phandle, DEBUG_DUMP_CTRL_REG,
					     DEBUG_HOST_READY);
			if (ret) {
				PRINTM(MERROR, "PCIE Write ERR\n");
				return RDWR_STATUS_FAILURE;
			}
		}
		udelay(100);
	}
	if (ctrl_data == DEBUG_HOST_READY) {
		PRINTM(MERROR, "Fail to pull ctrl_data\n");
		return RDWR_STATUS_FAILURE;
	}

	return RDWR_STATUS_SUCCESS;
}

#define PCIE_SCRATCH_12_REG 0x0CF0;
#define PCIE_SCRATCH_14_REG 0x0CF8;
#define PCIE_SCRATCH_15_REG 0x0CFC;
 /**
  *  @brief This function save the log of pcie register value
  *
  *  @param phandle   A pointer to moal_handle
  *  @param buffer    A pointer to buffer saving log
  *
  *  @return         The length of this log
  */
int
woal_dump_pcie_reg_info(moal_handle *phandle, t_u8 *buffer)
{
	char *drv_ptr = (char *)buffer;
	t_u32 reg = 0, value = 0;
	t_u8 i;
	char buf[256], *ptr;
	pcie_service_card *card = (pcie_service_card *)phandle->card;
	int config_reg_table[] =
		{ 0x00, 0x04, 0x10, 0x18, 0x2c, 0x3c, 0x44, 0x80, 0x98, 0x170 };
	ENTER();
	drv_ptr +=
		sprintf(drv_ptr,
			"------------PCIe Registers dump-------------\n");
	drv_ptr += sprintf(drv_ptr, "Config Space Registers:\n");
	for (i = 0; i < ARRAY_SIZE(config_reg_table); i++) {
		pci_read_config_dword(card->dev, config_reg_table[i], &value);
		drv_ptr +=
			sprintf(drv_ptr, "reg:0x%02x value=0x%08x\n",
				config_reg_table[i], value);
	}
	drv_ptr += sprintf(drv_ptr, "FW Scrach Registers:\n");
	reg = PCIE_SCRATCH_12_REG;
	woal_read_reg(phandle, reg, &value);
	drv_ptr += sprintf(drv_ptr, "reg:0x%x value=0x%x\n", reg, value);
	for (i = 0; i < 2; i++) {
		reg = PCIE_SCRATCH_14_REG;
		woal_read_reg(phandle, reg, &value);
		drv_ptr +=
			sprintf(drv_ptr, "reg:0x%x value=0x%x\n", reg, value);

		reg = PCIE_SCRATCH_15_REG;
		woal_read_reg(phandle, reg, &value);
		drv_ptr +=
			sprintf(drv_ptr, "reg:0x%x value=0x%x\n", reg, value);

		mdelay(100);
	}
	drv_ptr +=
		sprintf(drv_ptr,
			"Interface registers dump from offset 0c00 to 0cfc\n");
	memset(buf, 0, sizeof(buf));
	ptr = buf;
	i = 1;
	for (reg = 0x0c00; reg <= 0xcfc; reg += 4) {
		woal_read_reg(phandle, reg, &value);
		ptr += sprintf(ptr, "%08x ", value);
		if (!(i % 8)) {
			drv_ptr += sprintf(drv_ptr, "%s\n", buf);
			memset(buf, 0, sizeof(buf));
			ptr = buf;
		}
		i++;
	}
	drv_ptr +=
		sprintf(drv_ptr,
			"-----------PCIe Registers dump End-----------\n");
	LEAVE();
	return drv_ptr - (char *)buffer;
}

/**
 *  @brief This function reads and displays PCIE scratch registers for debugging
 *
 *  @param phandle  A pointer to moal_handle
 *
 *  @return         N/A
 */
void
woal_pcie_reg_dbg(moal_handle *phandle)
{
	t_u32 reg = 0, value = 0;
	t_u8 i;
	char buf[256], *ptr;
	pcie_service_card *card = (pcie_service_card *)phandle->card;
	int config_reg_table[] =
		{ 0x00, 0x04, 0x10, 0x18, 0x2c, 0x3c, 0x44, 0x80, 0x98, 0x170 };
	PRINTM(MMSG, "Config Space Registers:\n");
	for (i = 0; i < ARRAY_SIZE(config_reg_table); i++) {
		pci_read_config_dword(card->dev, config_reg_table[i], &value);
		PRINTM(MERROR, "reg:0x%02x value=0x%08x\n", config_reg_table[i],
		       value);
	}
	PRINTM(MMSG, "FW Scrach Registers:\n");
	reg = PCIE_SCRATCH_12_REG;
	woal_read_reg(phandle, reg, &value);
	PRINTM(MERROR, "reg:0x%x value=0x%x\n", reg, value);
	for (i = 0; i < 2; i++) {
		reg = PCIE_SCRATCH_14_REG;
		woal_read_reg(phandle, reg, &value);
		PRINTM(MERROR, "reg:0x%x value=0x%x\n", reg, value);

		reg = PCIE_SCRATCH_15_REG;
		woal_read_reg(phandle, reg, &value);
		PRINTM(MERROR, "reg:0x%x value=0x%x\n", reg, value);

		mdelay(100);
	}
	PRINTM(MMSG, "Interface registers dump from offset 0c00 to 0cfc\n");
	memset(buf, 0, sizeof(buf));
	ptr = buf;
	i = 1;
	for (reg = 0x0c00; reg <= 0xcfc; reg += 4) {
		woal_read_reg(phandle, reg, &value);
		ptr += sprintf(ptr, "%08x ", value);
		if (!(i % 8)) {
			PRINTM(MMSG, "%s\n", buf);
			memset(buf, 0, sizeof(buf));
			ptr = buf;
		}
		i++;
	}
}

/**
 *  @brief This function dump firmware memory to file
 *
 *  @param phandle   A pointer to moal_handle
 *
 *  @return         N/A
 */
void
woal_dump_firmware_info(moal_handle *phandle)
{

	int ret = 0;
	unsigned int reg, reg_start, reg_end;
	t_u8 *dbg_ptr = NULL;
	t_u32 sec, usec;
	t_u8 dump_num = 0;
	t_u8 idx = 0;
	t_u8 doneflag = 0;
	rdwr_status stat;
	t_u8 i = 0;
	t_u8 read_reg = 0;
	t_u32 memory_size = 0;
	t_u8 path_name[64], file_name[32];
	t_u8 *end_ptr = NULL;

	if (!phandle) {
		PRINTM(MERROR, "Could not dump firmwware info\n");
		return;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
    /** Create dump directory*/
	woal_create_dump_dir(phandle, path_name, sizeof(path_name));
#else
	memset(path_name, 0, sizeof(path_name));
	strcpy(path_name, "/data");
#endif
	PRINTM(MMSG, "Create DUMP directory success:dir_name=%s\n", path_name);
	woal_dump_drv_info(phandle, path_name);

	/* start dump fw memory     */
	moal_get_system_time(phandle, &sec, &usec);
	PRINTM(MMSG, "====PCIE DEBUG MODE OUTPUT START: %u.%06u ====\n", sec,
	       usec);
	/* read the number of the memories which will dump */
	if (RDWR_STATUS_FAILURE == woal_pcie_rdwr_firmware(phandle, doneflag))
		goto done;
	reg = DEBUG_DUMP_START_REG;
	ret = woal_read_reg_eight_bit(phandle, reg, &dump_num);
	if (ret) {
		PRINTM(MMSG, "PCIE READ MEM NUM ERR\n");
		goto done;
	}

	/* read the length of every memory which will dump */
	for (idx = 0; idx < dump_num; idx++) {
		if (RDWR_STATUS_FAILURE ==
		    woal_pcie_rdwr_firmware(phandle, doneflag))
			goto done;
		memory_size = 0;
		reg = DEBUG_DUMP_START_REG;
		for (i = 0; i < 4; i++) {
			ret = woal_read_reg_eight_bit(phandle, reg, &read_reg);
			if (ret) {
				PRINTM(MMSG, "PCIE READ ERR\n");
				goto done;
			}
			memory_size |= (read_reg << i * 8);
			reg++;
		}
		if (memory_size == 0) {
			PRINTM(MMSG, "Firmware Dump Finished!\n");
			ret = woal_write_reg(phandle, DEBUG_DUMP_CTRL_REG,
					     DEBUG_MEMDUMP_FINISH);
			if (ret) {
				PRINTM(MERROR,
				       "PCIE Write MEMDUMP_FINISH ERR\n");
				goto done;
			}
			break;
		} else {
			PRINTM(MMSG, "%s_SIZE=0x%x\n",
			       mem_type_mapping_tbl[idx].mem_name, memory_size);
			ret = moal_vmalloc(phandle, memory_size + 1,
					   (t_u8 **)&mem_type_mapping_tbl[idx].
					   mem_Ptr);
			if ((ret != MLAN_STATUS_SUCCESS) ||
			    !mem_type_mapping_tbl[idx].mem_Ptr) {
				PRINTM(MERROR,
				       "Error: vmalloc %s buffer failed!!!\n",
				       mem_type_mapping_tbl[idx].mem_name);
				goto done;
			}
			dbg_ptr = mem_type_mapping_tbl[idx].mem_Ptr;
			end_ptr = dbg_ptr + memory_size;
		}
		doneflag = mem_type_mapping_tbl[idx].done_flag;
		moal_get_system_time(phandle, &sec, &usec);
		PRINTM(MMSG, "Start %s output %u.%06u, please wait...\n",
		       mem_type_mapping_tbl[idx].mem_name, sec, usec);
		do {
			stat = woal_pcie_rdwr_firmware(phandle, doneflag);
			if (RDWR_STATUS_FAILURE == stat)
				goto done;

			reg_start = DEBUG_DUMP_START_REG;
			reg_end = DEBUG_DUMP_END_REG;
			for (reg = reg_start; reg <= reg_end; reg++) {
				ret = woal_read_reg_eight_bit(phandle, reg,
							      dbg_ptr);
				if (ret) {
					PRINTM(MMSG, "PCIE READ ERR\n");
					goto done;
				}
				if (dbg_ptr < end_ptr)
					dbg_ptr++;
				else
					PRINTM(MINFO,
					       "pre-allocced buf is not enough\n");
			}
			if (RDWR_STATUS_DONE == stat) {
				PRINTM(MMSG, "%s done: size=0x%x\n",
				       mem_type_mapping_tbl[idx].mem_name,
				       (unsigned int)(dbg_ptr -
						      mem_type_mapping_tbl[idx].
						      mem_Ptr));
				memset(file_name, 0, sizeof(file_name));
				sprintf(file_name, "%s%s", "file_pcie_",
					mem_type_mapping_tbl[idx].mem_name);
				if (MLAN_STATUS_SUCCESS !=
				    woal_save_dump_info_to_file(path_name,
								file_name,
								mem_type_mapping_tbl
								[idx].mem_Ptr,
								memory_size))
					PRINTM(MMSG,
					       "Can't save dump file %s in %s\n",
					       file_name, path_name);
				moal_vfree(phandle,
					   mem_type_mapping_tbl[idx].mem_Ptr);
				mem_type_mapping_tbl[idx].mem_Ptr = NULL;
				break;
			}
		} while (1);
	}
	moal_get_system_time(phandle, &sec, &usec);
	PRINTM(MMSG, "====PCIE DEBUG MODE OUTPUT END: %u.%06u ====\n", sec,
	       usec);
	/* end dump fw memory */
done:
	for (idx = 0; idx < dump_num; idx++) {
		if (mem_type_mapping_tbl[idx].mem_Ptr) {
			moal_vfree(phandle, mem_type_mapping_tbl[idx].mem_Ptr);
			mem_type_mapping_tbl[idx].mem_Ptr = NULL;
		}
	}

	return;
}
