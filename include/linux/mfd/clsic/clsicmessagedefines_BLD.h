/*****************************************************************************
 *
 * Copyright (c) 2018
 * Cirrus Logic, Inc. and Cirrus Logic International Semiconductor Ltd.
 * All rights reserved.
 * This software as well as any related documentation is furnished under
 * license and may only be used or copied in accordance with the terms of the
 * license. The information in this file is furnished for informational use
 * only, is subject to change without notice, and should not be construed as
 * a commitment by Cirrus Logic. Cirrus Logic assumes no responsibility or
 * liability for any errors or inaccuracies that may appear in this document or
 * any software that may be provided in association with this document.
 * Except as permitted by such license, no part of this document may be
 * reproduced, stored in a retrieval system, or transmitted in any form or by
 * any means without the express written consent of Cirrus Logic.
 *
 ******************************************************************************/

#ifndef CLSICMESSAGEDEFINES_BLD_H_
#define CLSICMESSAGEDEFINES_BLD_H_

#include "clsicmessagedefines.h"

/**
 *  Service type identifier.
 */
#define CLSIC_SRV_TYPE_BLD			(0x4C42)

/**
 *  Service version number.
 */
#define CLSIC_SRV_VERSION_BLD			(0x01000000)

/**
 *  Well-known service instance index.
 */
#define CLSIC_SRV_INST_BLD			(0x1F)

/**
 *  Boot Loader Service message identifiers.
 */
enum clsic_bl_msg_id {
	CLSIC_BL_MSG_CR_SET_FWU			= 33,
	CLSIC_BL_MSG_CR_SET_CPK			= 34,
	CLSIC_BL_MSG_CR_SET_MAB			= 35,
	CLSIC_BL_MSG_N_REQ_FWU			= 65,
	CLSIC_BL_MSG_N_REQ_CPK			= 66,
	CLSIC_BL_MSG_N_REQ_MAB			= 67,
	CLSIC_BL_MSG_N_NO_BOOTABLE_COMP		= 69,
	CLSIC_BL_MSG_N_FAILED_FLASH_AUTH	= 70,
	CLSIC_BL_MSG_N_FLASH_CORRUPTED		= 71,
};

/**
 *  Boot Loader component identifiers for notifications.
 */
enum clsic_bl_nty {
	CLSIC_BL_FAILED_AUTH_NBS2		= 0x10,
	CLSIC_BL_FAILED_AUTH_OSAPP		= 0x11,
	CLSIC_BL_NO_NBS2			= 0x20,
	CLSIC_BL_NO_OSAPP			= 0x21,
	CLSIC_BL_FLASH_CORRUPT_PRE_NBS2		= 0x30,
	CLSIC_BL_FLASH_CORRUPT_PRE_OSAPP	= 0x31,
	CLSIC_BL_FLASH_CORRUPT_FWU		= 0x32,
};

/**
 *  Boot Loader CLSIC_BL_MSG_CR_SET_MAB flags.
 */
enum clsic_bl_flags {
	CLSIC_BL_RESET_NOT_REQUIRED = 0x80,
};

/**
 *  Boot Loader Service messages.
 */
union clsic_bl_msg {
	uint8_t raw_msg[CLSIC_FIXED_MSG_SZ];

	/**
	 *  CLSIC_BL_MSG_CR_SET_FWU command structure.
	 */
	struct {
		struct clsic_blkcmd_hdr hdr;
	} PACKED blkcmd_set_fwu;

	/**
	 *  CLSIC_BL_MSG_CR_SET_FWU response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
	} PACKED rsp_set_fwu;

	/**
	 *  CLSIC_BL_MSG_CR_SET_CPK command structure.
	 */
	struct {
		struct clsic_blkcmd_hdr hdr;
	} PACKED blkcmd_set_cpk;

	/**
	 *  CLSIC_BL_MSG_CR_SET_CPK response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
	} PACKED rsp_set_cpk;

	/**
	 *  CLSIC_BL_MSG_CR_SET_MAB command structure.
	 */
	struct {
		struct clsic_blkcmd_hdr hdr;
	} PACKED blkcmd_set_mab;

	/**
	 *  CLSIC_BL_MSG_CR_SET_MAB response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
		uint8_t flags;
	} PACKED rsp_set_mab;

	/**
	 *  CLSIC_BL_MSG_N_REQ_FWU notification structure.
	 */
	struct {
		struct clsic_nty_hdr hdr;
	} PACKED nty_req_fwu;

	/**
	 *  CLSIC_BL_MSG_N_REQ_CPK notification structure.
	 */
	struct {
		struct clsic_nty_hdr hdr;
	} PACKED nty_req_cpk;

	/**
	 *  CLSIC_BL_MSG_N_REQ_MAB notification structure.
	 */
	struct {
		struct clsic_nty_hdr hdr;
	} PACKED nty_req_mab;

	/**
	 *  CLSIC_BL_MSG_N_NO_BOOTABLE_COMP notification structure.
	 */
	struct {
		struct clsic_nty_hdr hdr;
		uint8_t component;
	} PACKED nty_no_bootable_comp;

	/**
	 *  CLSIC_BL_MSG_N_FAILED_FLASH_AUTH notification structure.
	 */
	struct {
		struct clsic_nty_hdr hdr;
		uint8_t component;
	} PACKED nty_failed_flash_auth;

	/**
	 *  CLSIC_BL_MSG_N_FLASH_CORRUPTED notification structure.
	 */
	struct {
		struct clsic_nty_hdr hdr;
		uint8_t component;
	} PACKED nty_flash_corrupted;
} PACKED;

#endif /* CLSICMESSAGEDEFINES_BLD_H_ */
