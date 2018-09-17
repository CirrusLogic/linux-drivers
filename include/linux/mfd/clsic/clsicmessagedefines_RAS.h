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

#ifndef CLSICMESSAGEDEFINES_RAS_H_
#define CLSICMESSAGEDEFINES_RAS_H_

#include "clsicmessagedefines.h"

/**
 *  Service type identifier.
 */
#define CLSIC_SRV_TYPE_RAS			(0x4152)

/**
 *  Service version number.
 */
#define CLSIC_SRV_VERSION_RAS			(0x00030000)

/**
 *  Register Access Service message identifiers.
 */
enum clsic_ras_msg_id {
	CLSIC_RAS_MSG_CR_RDREG			= 1,
	CLSIC_RAS_MSG_CR_WRREG			= 2,
	CLSIC_RAS_MSG_CR_DEPRECATED_1		= 3,
	CLSIC_RAS_MSG_CR_RDREG_BULK		= 4,
	CLSIC_RAS_MSG_CR_WRREG_BULK		= 5,
	CLSIC_RAS_MSG_CR_SET_IRQ_NTY_MODE	= 6,
	CLSIC_RAS_MSG_N_IRQ			= 7,
	CLSIC_RAS_MSG_CR_DEPRECATED_2		= 8,
	CLSIC_RAS_MSG_CR_DEPRECATED_3		= 9,
	CLSIC_RAS_MSG_CR_GET_CAP		= 10,
	CLSIC_RAS_MSG_N_ERR_FAST_WRITE		= 11,
	CLSIC_RAS_MSG_CR_GET_DI_CATEGORY_COUNT	=
		CLSIC_GBL_MSG_CR_GET_DI_CATEGORY_COUNT,
	CLSIC_RAS_MSG_CR_GET_DI_PAGE_COUNT	=
		CLSIC_GBL_MSG_CR_GET_DI_PAGE_COUNT,
	CLSIC_RAS_MSG_CR_GET_DEBUG_INFO		=
		CLSIC_GBL_MSG_CR_GET_DEBUG_INFO,
};

enum clsic_ras_irq_nty_mode {
	CLSIC_RAS_NTY_REQ		= 1,
	CLSIC_RAS_NTY_FLUSH_AND_REQ	= 2,
	CLSIC_RAS_NTY_CANCEL		= 3,
};

enum clsic_ras_irq_id {
	CLSIC_RAS_IRQ_DSP2_0	= 0,
	CLSIC_RAS_IRQ_DSP2_1	= 1,
	CLSIC_RAS_IRQ_DSP2_2	= 2,
	CLSIC_RAS_IRQ_DSP2_3	= 3,
	CLSIC_RAS_IRQ_COUNT	= 4,
};

/**
 * Register Access Service capabilities bitmask queried using
 * CLSIC_RAS_MSG_CR_GET_CAP.
 */
enum clasic_ras_capability {
	CLSIC_RAS_CAP_FAST_WRITE	= (0x1 << 0),
};

/**
 *  Register Access Service messages.
 */
union clsic_ras_msg {
	uint8_t raw_msg[CLSIC_FIXED_MSG_SZ];

	/**
	 *  CLSIC_RAS_MSG_CR_RDREG command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
		uint32_t addr;
	} PACKED cmd_rdreg;

	/**
	 *  CLSIC_RAS_MSG_CR_RDREG response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
		uint32_t value;
	} PACKED rsp_rdreg;

	/**
	 *  CLSIC_RAS_MSG_CR_WRREG command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
		uint32_t addr;
		uint32_t value;
	} PACKED cmd_wrreg;

	/**
	 *  CLSIC_RAS_MSG_CR_WRREG response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
	} PACKED rsp_wrreg;

	/**
	 *  CLSIC_RAS_MSG_CR_GET_DI_CATEGORY_COUNT command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
	} PACKED cmd_get_di_category_count;

	/**
	 *  CLSIC_RAS_MSG_CR_GET_DI_CATEGORY_COUNT response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
		uint16_t category_count;
	} PACKED rsp_get_di_category_count;

	/**
	 *  CLSIC_RAS_MSG_CR_GET_DI_PAGE_COUNT command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
		uint16_t category;
	} PACKED cmd_get_di_page_count;

	/**
	 *  CLSIC_RAS_MSG_CR_GET_DI_PAGE_COUNT response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
		uint16_t category;
		uint16_t page_count;
	} PACKED rsp_get_di_page_count;

	/**
	 *  CLSIC_RAS_MSG_CR_GET_DEBUG_INFO command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
		uint16_t category;
		uint16_t page;
	} PACKED cmd_get_debug_info;

	/**
	 *  CLSIC_RAS_MSG_CR_GET_DEBUG_INFO response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
		uint16_t category;
		uint16_t page;
	} PACKED rsp_get_debug_info;

	/**
	 *  CLSIC_RAS_MSG_CR_GET_DEBUG_INFO response structure.
	 */
	struct {
		struct clsic_blkrsp_hdr hdr;
		uint16_t category;
		uint16_t page;
	} PACKED blkrsp_get_debug_info;

	/**
	 *  CLSIC_RAS_MSG_CR_RDREG_BULK command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
		uint32_t addr;
		uint32_t byte_count;
	} PACKED cmd_rdreg_bulk;

	/**
	 *  CLSIC_RAS_MSG_CR_RDREG_BULK response structure.
	 */
	struct {
		struct clsic_blkrsp_hdr hdr;
	} PACKED blkrsp_rdreg_bulk;

	/**
	 *  CLSIC_RAS_MSG_CR_RDREG_BULK error response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
	} PACKED rsp_rdreg_bulk;

	/**
	 *  CLSIC_RAS_MSG_CR_WRREG_BULK command structure.
	 */
	struct {
		struct clsic_blkcmd_hdr hdr;
		uint32_t addr;
	} PACKED blkcmd_wrreg_bulk;

	/**
	 *  CLSIC_RAS_MSG_CR_WRREG_BULK response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
	} PACKED rsp_wrreg_bulk;

	/**
	 *  CLSIC_RAS_MSG_CR_SET_IRQ_NTY_MODE command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
		uint32_t irq_id;
		uint32_t mode;
	} PACKED cmd_set_irq_nty_mode;

	/**
	 *  CLSIC_RAS_MSG_CR_SET_IRQ_NTY_MODE response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
	} PACKED rsp_set_irq_nty_mode;

	/**
	 *  CLSIC_RAS_MSG_N_IRQ notification structure.
	 */
	struct {
		struct clsic_nty_hdr hdr;
		uint32_t irq_id;
	} PACKED nty_irq;

	/**
	 *  CLSIC_RAS_MSG_N_ERR_FAST_WRITE notification structure.
	 */
	struct {
		struct clsic_nty_hdr hdr;
		uint8_t err;
		uint32_t reg_addr;
		uint32_t reg_val;
	} PACKED nty_err_fast_write;

	/**
	 * CLSIC_RAS_MSG_CR_GET_CAP command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
	} PACKED cmd_getcap;

	/**
	 * CLSIC_RAS_MSG_CR_GET_CAP response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
		uint32_t mask;
	} PACKED rsp_getcap;
} PACKED;

/**
 *  Register Access Service fast register write command.
 */
struct clsic_ras_fast_reg_write {
	uint32_t reg_addr;
	uint32_t reg_val;
} PACKED;

#endif /* CLSICMESSAGEDEFINES_RAS_H_ */
