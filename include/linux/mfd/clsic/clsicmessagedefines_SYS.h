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

#ifndef CLSICMESSAGEDEFINES_SYS_H_
#define CLSICMESSAGEDEFINES_SYS_H_

#include "clsicmessagedefines.h"

/**
 *  Service type identifier.
 */
#define CLSIC_SRV_TYPE_SYS			(0x5953)

/**
 *  Service version number.
 */
#define CLSIC_SRV_VERSION_SYS			(CONFIG_VERSION_NUMBER)

/**
 *  Well-known service instance index.
 */
#define CLSIC_SRV_INST_SYS			(0x00)

/**
 *  System Service message identifiers.
 */
enum clsic_sys_msg_id {
	CLSIC_SYS_MSG_CR_SYS_INFO		= 0,
	CLSIC_SYS_MSG_CR_SRV_INFO		= 1,
	CLSIC_SYS_MSG_CR_SP_SHDN		= 2,
	CLSIC_SYS_MSG_N_RXDMA_STS		= 3,
	CLSIC_SYS_MSG_N_INVAL_CMD		= 4,
	CLSIC_SYS_MSG_N_PANIC			= 5,
	CLSIC_SYS_MSG_CR_SET_TRACE_FILTER	= 6,
	CLSIC_SYS_MSG_CR_DEPRECATED_1		= 7,
	CLSIC_SYS_MSG_CR_GET_KEY_VAL		= 8,
	CLSIC_SYS_MSG_CR_DEPRECATED_2		= 9,
	CLSIC_SYS_MSG_CR_DEPRECATED_3		= 10,
	CLSIC_SYS_MSG_CR_IOCTL			= 11,
	CLSIC_SYS_MSG_CR_MAB_VERSION		= 12,
	CLSIC_SYS_MSG_CR_GET_DI_CATEGORY_COUNT	=
		CLSIC_GBL_MSG_CR_GET_DI_CATEGORY_COUNT,
	CLSIC_SYS_MSG_CR_GET_DI_PAGE_COUNT	=
		CLSIC_GBL_MSG_CR_GET_DI_PAGE_COUNT,
	CLSIC_SYS_MSG_CR_GET_DEBUG_INFO		=
		CLSIC_GBL_MSG_CR_GET_DEBUG_INFO,
};

/**
 *  System Service ioctl id's.
 */
enum clsic_sys_ioctl {
	CLSIC_SYS_IOCTL_EXT_CODEC_COMMS_HALT	= 0,
	CLSIC_SYS_IOCTL_EXT_CODEC_COMMS_RESUME	= 1,
};

/**
 *  Maximum Size in bytes of a registry key/value.
 */
#define CLSIC_MAX_REGISTRY_KEY_SZ                       (64)

/**
 *  Bulk part of the CLSIC_SYS_MSG_CR_GET_KEY_VAL command.
 *  The actual length of the key can be less than or equal to
 *  CLSIC_MAX_REGISTRY_KEY_SZ (key is not null terminated and the
 *  length is explicitly specified in the .length field).
 */
struct clsic_sys_registry_key {
	uint8_t val_idx;
	uint8_t length;
	int8_t key[CLSIC_MAX_REGISTRY_KEY_SZ];
	uint8_t pad[2];
} PACKED;

/**
 *  Registry Tags
 */
enum clsic_registry_tags {
	CLSIC_REGISTRY_KEY	= 1,
	CLSIC_REGISTRY_INT	= 2,
	CLSIC_REGISTRY_BOOL	= 3,
	CLSIC_REGISTRY_STRING	= 4,
};

/**
 *  Bulk part of the CLSIC_SYS_MSG_CR_GET_KEY_VAL response.
 *  The actual length of the key value can be less than or equal to
 *  CLSIC_MAX_REGISTRY_KEY_SZ (if key value is a string then it is not
 *  null terminated and the length is explicitly specified in the
 * .length field).
 */
struct clsic_sys_registry_value {
	uint8_t tag;
	uint8_t length;
	union {
		int8_t str_val[CLSIC_MAX_REGISTRY_KEY_SZ];
		int32_t int_val;
		int8_t bool_val;
	} PACKED clsic_key_val;
	uint8_t pad[2];
} PACKED;

/**
 *  System Service messages.
 */
union clsic_sys_msg {
	uint8_t raw_msg[CLSIC_FIXED_MSG_SZ];

	/**
	 *  CLSIC_SYS_MSG_CR_SYS_INFO command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
	} PACKED cmd_sys_info;

	/**
	 *  CLSIC_SYS_MSG_CR_SYS_INFO response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
		uint8_t srv_count;
	} PACKED rsp_sys_info;

	/**
	 *  CLSIC_SYS_MSG_CR_SRV_INFO command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
		uint8_t srv_inst;
	} PACKED cmd_srv_info;

	/**
	 *  CLSIC_SYS_MSG_CR_SRV_INFO response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
		uint16_t srv_type;
		uint32_t srv_ver;
	} PACKED rsp_srv_info;

	/**
	 *  CLSIC_SYS_MSG_CR_SP_SHDN command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
	} PACKED cmd_sp_shdn;

	/**
	 *  CLSIC_SYS_MSG_CR_SP_SHDN response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
		uint32_t srvs_hold_wakelock;
	} PACKED rsp_sp_shdn;

	/**
	 *  CLSIC_SYS_MSG_N_RXDMA_STS notification structure.
	 */
	struct {
		struct clsic_nty_hdr hdr;
		uint8_t err;
		uint8_t srv_inst;
		uint8_t msgid;
		uint8_t slice_num;
		uint32_t slice_sz;
	} PACKED nty_rxdma_sts;

	/**
	 *  CLSIC_SYS_MSG_N_INVAL_CMD notification structure.
	 */
	struct {
		struct clsic_nty_hdr hdr;
		uint8_t err;
		uint8_t srv_inst;
		uint8_t msgid;
	} PACKED nty_inval_cmd;

	/**
	 *  CLSIC_SYS_MSG_N_PANIC notification structure.
	 */
	struct {
		struct clsic_blknty_hdr hdr;
	} PACKED blknty_panic;

	/**
	 *  CLSIC_SYS_MSG_CR_SET_TRACE_FILTER command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
		uint32_t new_level;
		uint32_t new_mask;
	} PACKED cmd_set_trace_filter;

	/**
	 *  CLSIC_SYS_MSG_CR_SET_TRACE_FILTER response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
		uint32_t old_level;
		uint32_t old_mask;
	} PACKED rsp_set_trace_filter;

	/**
	 *  CLSIC_SYS_MSG_CR_GET_DI_CATEGORY_COUNT command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
	} PACKED cmd_get_di_category_count;

	/**
	 *  CLSIC_SYS_MSG_CR_GET_DI_CATEGORY_COUNT response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
		uint16_t category_count;
	} PACKED rsp_get_di_category_count;

	/**
	 *  CLSIC_SYS_MSG_CR_GET_DI_PAGE_COUNT command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
		uint16_t category;
	} PACKED cmd_get_di_page_count;

	/**
	 *  CLSIC_SYS_MSG_CR_GET_DI_PAGE_COUNT response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
		uint16_t category;
		uint16_t page_count;
	} PACKED rsp_get_di_page_count;

	/**
	 *  CLSIC_SYS_MSG_CR_GET_DEBUG_INFO command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
		uint16_t category;
		uint16_t page;
	} PACKED cmd_get_debug_info;

	/**
	 *  CLSIC_SYS_MSG_CR_GET_DEBUG_INFO response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
		uint16_t category;
		uint16_t page;
	} PACKED rsp_get_debug_info;

	/**
	 *  CLSIC_SYS_MSG_CR_GET_DEBUG_INFO response structure.
	 */
	struct {
		struct clsic_blkrsp_hdr hdr;
		uint16_t category;
		uint16_t page;
	} PACKED blkrsp_get_debug_info;

	/**
	 *  CLSIC_SYS_MSG_CR_GET_KEY_VAL command structure.
	 */
	struct {
		struct clsic_blkcmd_hdr hdr;
	} PACKED blkcmd_get_key_val;

	/**
	 *  CLSIC_SYS_MSG_CR_GET_KEY_VAL response structure.
	 */
	struct {
		struct clsic_blkrsp_hdr hdr;
	} PACKED blkrsp_get_key_val;

	/**
	 *  CLSIC_SYS_MSG_CR_GET_KEY_VAL response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
	} PACKED rsp_get_key_val;

	/**
	 *  CLSIC_SYS_MSG_CR_IOCTL command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
		uint8_t id;
		union {
			uint8_t raw_payload[9];
		} PACKED args;
	} PACKED cmd_ioctl;

	/**
	 *  CLSIC_SYS_MSG_CR_IOCTL response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
		uint8_t id;
		union {
			uint8_t raw_payload[8];
		} PACKED args;
	} PACKED rsp_ioctl;

	/**
	 *  CLSIC_SYS_MSG_CR_MAB_VERSION command structure.
	 */
	struct {
		struct clsic_cmd_hdr hdr;
	} PACKED cmd_mab_version;

	/**
	 *  CLSIC_SYS_MSG_CR_MAB_VERSION response structure.
	 */
	struct {
		struct clsic_rsp_hdr hdr;
		uint32_t mab_version;
	} PACKED rsp_mab_version;

} PACKED;

#endif /* CLSICMESSAGEDEFINES_SYS_H_ */
