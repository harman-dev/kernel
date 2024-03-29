/** @file moal_pcie.h
 *
 *  @brief This file contains definitions for PCIE interface.
 *  driver.
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

#ifndef _MOAL_PCIE_H_
#define _MOAL_PCIE_H_

#define PCIE_VENDOR_ID_MARVELL              (0x11ab)
#define PCIE_VENDOR_ID_V2_MARVELL           (0x1b4b)
/** PCIE device ID for 8897 card */
#define PCIE_DEVICE_ID_MARVELL_88W8897P     (0x2b38)

#include    <linux/pci.h>
#include    <linux/pcieport_if.h>
#include    <linux/interrupt.h>

#include    "moal_main.h"

#ifdef STA_SUPPORT
/** Default firmware name */

/*#define DEFAULT_FW_NAME	"mrvl/pcie8897_uapsta.bin"*/
#define PCIE8897_A0_FW_NAME "mrvl/pcie8897_uapsta_a0.bin"
#define PCIE8897_B0_FW_NAME "mrvl/pcieuart8897_combo.bin"
#define PCIE8897_WLAN_B0_FW_NAME "mrvl/pcie8897_wlan.bin"

#ifndef DEFAULT_FW_NAME
#define DEFAULT_FW_NAME ""
#endif
#endif /* STA_SUPPORT */

#ifdef UAP_SUPPORT
/** Default firmware name */

#define DEFAULT_AP_FW_NAME "mrvl/pcieuart8897_combo.bin"
#define DEFAULT_WLAN_FW_NAME "mrvl/pcie8897_wlan.bin"

#ifndef DEFAULT_AP_FW_NAME
#define DEFAULT_AP_FW_NAME ""
#endif
#endif /* UAP_SUPPORT */

/** Default firmaware name */

#define DEFAULT_AP_STA_FW_NAME "mrvl/pcieuart8897_combo.bin"
#define DEFAULT_WLAN_FW_NAME "mrvl/pcie8897_wlan.bin"

#ifndef DEFAULT_AP_STA_FW_NAME
#define DEFAULT_AP_STA_FW_NAME ""
#endif

/** Structure: PCIE service card */
typedef struct _pcie_service_card {
    /** pci_dev structure pointer */
	struct pci_dev *dev;
    /** moal_handle structure pointer */
	moal_handle *handle;
    /** I/O memory regions pointer to the bus */
	void __iomem *pci_mmap;
    /** I/O memory regions pointer to the bus */
	void __iomem *pci_mmap1;

} pcie_service_card, *ppcie_service_card;

/** Function to write register */
mlan_status woal_write_reg(moal_handle *handle, t_u32 reg, t_u32 data);
/** Function to read register */
mlan_status woal_read_reg(moal_handle *handle, t_u32 reg, t_u32 *data);
/** Function to read register by eight bit*/
mlan_status woal_read_reg_eight_bit(moal_handle *handle, t_u32 reg, t_u8 *data);
/** Function to write data to IO memory */
mlan_status woal_write_data_sync(moal_handle *handle, mlan_buffer *pmbuf,
				 t_u32 port, t_u32 timeout);
/** Function to read data from IO memory */
mlan_status woal_read_data_sync(moal_handle *handle, mlan_buffer *pmbuf,
				t_u32 port, t_u32 timeout);

/** Register to bus driver function */
mlan_status woal_bus_register(void);
/** Unregister from bus driver function */
void woal_bus_unregister(void);
void woal_pcie_reg_dbg(moal_handle *phandle);
int woal_dump_pcie_reg_info(moal_handle *phandle, t_u8 *buffer);
/** Register device function */
mlan_status woal_register_dev(moal_handle *handle);
/** Unregister device function */
void woal_unregister_dev(moal_handle *handle);
#endif /* _MOAL_PCIE_H_ */
