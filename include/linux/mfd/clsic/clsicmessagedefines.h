/*****************************************************************************
 *
 * Copyright (c) 2017-2018
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

#ifndef CLSICMESSAGEDEFINES_H
#define CLSICMESSAGEDEFINES_H

/**
 *  Size in bytes of a fixed size message.
 */
#define CLSIC_FIXED_MSG_SZ			(12)

/**
 *  Values for the 2 bit CRAN field embedded in the header of all messages.
 */
#define CLSIC_CRAN_CMD				(0)
#define CLSIC_CRAN_RSP				(1)
#define CLSIC_CRAN_ACK				(2)
#define CLSIC_CRAN_NTY				(3)

/**
 *  Masks and positions for the individual fields in a
 *  serviceInstance-Bulk-Cran (sbc) field.
 */
#define CLSIC_SRV_INST_POS			(3)
#define CLSIC_SRV_INST_MASK			(0x1F << CLSIC_SRV_INST_POS)
#define CLSIC_BULK_POS				(2)
#define CLSIC_BULK_MASK				(0x1 << CLSIC_BULK_POS)
#define CLSIC_CRAN_POS				(0)
#define CLSIC_CRAN_MASK				(0x3 << CLSIC_CRAN_POS)

/**
 *  Given a pointer to a ServiceIntance-Bulk-Cran (sbc) field, sets to the
 *  instance to the provided value.
 */
static inline void clsic_set_srv_inst(uint8_t *sbc, uint8_t inst)
{
	inst = (inst) & (CLSIC_SRV_INST_MASK >> CLSIC_SRV_INST_POS);
	*sbc = (*sbc) & (~CLSIC_SRV_INST_MASK);
	*sbc = (*sbc) | (inst << CLSIC_SRV_INST_POS);
}

/**
 *  Extracts and returns the instance from the provided
 *  ServiceIntance-Bulk-Cran (sbc) field.
 */
static inline uint8_t clsic_get_srv_inst(uint8_t sbc)
{
	return (sbc & CLSIC_SRV_INST_MASK) >> CLSIC_SRV_INST_POS;
}

/**
 *  Given a pointer to a ServiceIntance-Bulk-Cran (sbc) field, sets to the
 *  bulk bit to the provided value.
 */
static inline void clsic_set_bulk(uint8_t *sbc, uint8_t bulk)
{
	bulk = (bulk) & (CLSIC_BULK_MASK >> CLSIC_BULK_POS);
	*sbc = (*sbc) & (~CLSIC_BULK_MASK);
	*sbc = (*sbc) | (bulk << CLSIC_BULK_POS);
}

/**
 *  Extracts and returns the bulk bit from the provided
 *  ServiceIntance-Bulk-Cran (sbc) field.
 */
static inline uint8_t clsic_get_bulk_bit(uint8_t sbc)
{
	return (sbc & CLSIC_BULK_MASK) >> CLSIC_BULK_POS;
}

/**
 *  Given a pointer to a ServiceIntance-Bulk-Cran (sbc) field, sets to the
 *  CRAN to the provided value.
 */
static inline void clsic_set_cran(uint8_t *sbc, uint8_t cran)
{
	cran = (cran) & (CLSIC_CRAN_MASK >> CLSIC_CRAN_POS);
	*sbc = (*sbc) & (~CLSIC_CRAN_MASK);
	*sbc = (*sbc) | (cran << CLSIC_CRAN_POS);
}

/**
 *  Extracts and returns the CRAN from the provided
 *  ServiceIntance-Bulk-Cran (sbc) field.
 */
static inline uint8_t clsic_get_cran(uint8_t sbc)
{
	return (sbc & CLSIC_CRAN_MASK) >> CLSIC_CRAN_POS;
}

/**
 *  Global message identifiers for messages that can be sent to any service.
 */
enum clsic_gbl_msg_id {
	CLSIC_GBL_MSG_CR_GET_DI_CATEGORY_COUNT	= 253,
	CLSIC_GBL_MSG_CR_GET_DI_PAGE_COUNT	= 254,
	CLSIC_GBL_MSG_CR_GET_DEBUG_INFO		= 255,
};

/**
 *  Header structures for fixed size messages.
 */
struct clsic_cmd_hdr {
	uint8_t sbc;
	uint8_t msgid;
} PACKED;

struct clsic_rsp_hdr {
	uint8_t sbc;
	uint8_t msgid;
	uint8_t err;
} PACKED;

struct clsic_ack_hdr {
	uint8_t sbc;
	uint8_t msgid;
} PACKED;

struct clsic_nty_hdr {
	uint8_t sbc;
	uint8_t msgid;
} PACKED;

/**
 *
 */
static inline void clsic_cmd_hdr_init(struct clsic_cmd_hdr *hdr, uint8_t msgid)
{
	clsic_set_bulk(&hdr->sbc, 0);
	clsic_set_cran(&hdr->sbc, CLSIC_CRAN_CMD);
	hdr->msgid = msgid;
}

static inline void clsic_rsp_hdr_init(struct clsic_rsp_hdr *hdr, uint8_t msgid,
				      uint8_t err)
{
	clsic_set_bulk(&hdr->sbc, 0);
	clsic_set_cran(&hdr->sbc, CLSIC_CRAN_RSP);
	hdr->msgid = msgid;
	hdr->err = err;
}

static inline void clsic_ack_hdr_init(struct clsic_ack_hdr *hdr, uint8_t msgid)
{
	clsic_set_bulk(&hdr->sbc, 0);
	clsic_set_cran(&hdr->sbc, CLSIC_CRAN_ACK);
	hdr->msgid = msgid;
}

static inline void clsic_nty_hdr_init(struct clsic_nty_hdr *hdr, uint8_t msgid)
{
	clsic_set_bulk(&hdr->sbc, 0);
	clsic_set_cran(&hdr->sbc, CLSIC_CRAN_NTY);
	hdr->msgid = msgid;
}

/**
 *  Header structures for bulk messages.
 */
struct clsic_blkcmd_hdr {
	uint8_t sbc;
	uint8_t msgid;
	uint32_t bulk_sz;
} PACKED;

struct clsic_blkrsp_hdr {
	uint8_t sbc;
	uint8_t msgid;
	uint32_t bulk_sz;
	uint8_t err;
} PACKED;

struct clsic_blknty_hdr {
	uint8_t sbc;
	uint8_t msgid;
	uint32_t bulk_sz;
} PACKED;

static inline void clsic_blkcmd_hdr_init(struct clsic_blkcmd_hdr *hdr,
					 uint8_t msgid, uint32_t bulk_sz)
{
	clsic_set_bulk(&hdr->sbc, 1);
	clsic_set_cran(&hdr->sbc, CLSIC_CRAN_CMD);
	hdr->msgid = msgid;
	hdr->bulk_sz = bulk_sz;
}

static inline void clsic_blkrsp_hdr_init(struct clsic_blkrsp_hdr *hdr,
					 uint8_t msgid, uint32_t bulk_sz,
					 uint8_t err)
{
	clsic_set_bulk(&hdr->sbc, 1);
	clsic_set_cran(&hdr->sbc, CLSIC_CRAN_RSP);
	hdr->msgid = msgid;
	hdr->err = err;
	hdr->bulk_sz = bulk_sz;
}

static inline void clsic_blknty_hdr_init(struct clsic_blknty_hdr *hdr,
					 uint8_t msgid, uint32_t bulk_sz)
{
	clsic_set_bulk(&hdr->sbc, 1);
	clsic_set_cran(&hdr->sbc, CLSIC_CRAN_NTY);
	hdr->msgid = msgid;
	hdr->bulk_sz = bulk_sz;
}

/**
 *  Structure output as the bulk part of all PANIC and GET_DEBUG_INFO messages.
 *  The payload starting at salt[0] to the end of the structure should be
 *  considered opaque and is encrypted if "encrypted" is non-zero. If
 *  encrypted, "key" contains the compressed public half of the ECC256 key
 *  generated by the SP for ECDH cryptography.
 */
struct clsic_debug_info {
	uint32_t version;
	uint8_t encrypted;
	uint8_t key[33];
	uint8_t salt[4];
	uint8_t info[2044];
	uint8_t pad[2];
} PACKED;

/**
 *  Error codes returned in response message err fields.
 */
enum clsic_err {
	CLSIC_ERR_NONE				= 0,
	CLSIC_ERR_NO_MEM			= 1,
	CLSIC_ERR_INVAL_SI			= 2,
	CLSIC_ERR_INVAL_MSGID			= 3,
	CLSIC_ERR_INVAL_CRAN			= 4,
	CLSIC_ERR_INVAL_BULK			= 5,
	CLSIC_ERR_CANCELLED			= 6,
	CLSIC_ERR_INVAL_ARG			= 7,
	CLSIC_ERR_INVAL_ADDR			= 8,
	CLSIC_ERR_ACCESS			= 9,
	CLSIC_ERR_HW				= 10,
	CLSIC_ERR_FLASH				= 11,
	CLSIC_ERR_TOO_SMALL			= 12,
	CLSIC_ERR_WAKELOCK_HELD			= 13,
	CLSIC_ERR_INVAL_MODE			= 14,
	CLSIC_ERR_INVAL_MODE_TRANSITION		= 15,
	CLSIC_ERR_INVAL_CMD_FOR_MODE		= 16,
	CLSIC_ERR_INVAL_USERID			= 17,
	CLSIC_ERR_INVAL_PHRASEID		= 18,
	CLSIC_ERR_INVAL_APP_ID			= 19,
	CLSIC_ERR_USER_NOT_INSTALLED		= 21,
	CLSIC_ERR_USER_ALREADY_INSTALLED	= 23,
	CLSIC_ERR_PHRASE_NOT_INSTALLED		= 24,
	CLSIC_ERR_VOICEID			= 25,
	CLSIC_ERR_INPUT_PATH			= 26,
	CLSIC_ERR_SECURITY_FAIL			= 27,
	CLSIC_ERR_AUTH_ABORT_BARGE_IN		= 28,
	CLSIC_ERR_AUTH_NOT_STARTED_BARGE_IN	= 29,
	CLSIC_ERR_INVAL_SECURITY_LVL		= 30,
	CLSIC_ERR_NO_USER_IDENTIFIED		= 31,
	CLSIC_ERR_NOT_INSTALLING_USER		= 33,
	CLSIC_ERR_ALREADY_INSTALLING_USER	= 34,
	CLSIC_ERR_INVAL_REP_COUNT		= 35,
	CLSIC_ERR_ONGOING_REP			= 36,
	CLSIC_ERR_REPS_COMPLETE			= 37,
	CLSIC_ERR_REP_TRGR_TIMEOUT		= 38,
	CLSIC_ERR_REP_UNEXPECTED_TRGR		= 39,
	CLSIC_ERR_REP_NOISE_LVL			= 40,
	CLSIC_ERR_REP_SPEECH_RATIO		= 41,
	CLSIC_ERR_REP_NET_SPEECH		= 42,
	CLSIC_ERR_REP_SATURATION		= 43,
	CLSIC_ERR_REP_FEATURE_OVERFLOW		= 44,
	CLSIC_ERR_REPS_NOT_ENOUGH_VALID		= 45,
	CLSIC_ERR_AUTH_IN_PROGRESS		= 46,
	CLSIC_ERR_INVAL_TRGR_DOMAIN		= 47,
	CLSIC_ERR_INVAL_ASR_BLOCK_SZ		= 48,
	CLSIC_ERR_ALREADY_LISTENING		= 49,
	CLSIC_ERR_INVAL_CMD_FOR_TRGR_DOMAIN	= 50,
	CLSIC_ERR_LISTEN_NOT_STARTED		= 51,
	CLSIC_ERR_INVAL_TRGR_INFO		= 52,
	CLSIC_ERR_INVAL_TRGR_ENGINEID		= 53,
	CLSIC_ERR_INVAL_TRGR_PHRASEID		= 54,
	CLSIC_ERR_ASR_PREV_REQUEST_PENDING	= 55,
	CLSIC_ERR_ASR_STREAM			= 56,
	CLSIC_ERR_ASR_STREAM_DISABLED		= 57,
	CLSIC_ERR_KEY_NOT_FOUND			= 58,
	CLSIC_ERR_BPB_SZ_TOO_SMALL		= 59,
	CLSIC_ERR_BPB_SZ_UNALIGNED		= 60,
	CLSIC_ERR_BPB_BAD_HDR			= 61,
	CLSIC_ERR_BPB_BAD_IMGMAP		= 62,
	CLSIC_ERR_BPB_SZ_INCONSISTENT		= 63,
	CLSIC_ERR_BPB_AUTH_FAILED		= 64,
	CLSIC_ERR_COV_DISABLED			= 65,
	CLSIC_ERR_INVALID_ENROL_DURATION	= 66,
	CLSIC_ERR_INVALID_AUTH_RESULT_FORMAT	= 67,
	CLSIC_ERR_BPB_ASSET_INVAL_VER		= 68,
	CLSIC_ERR_BPB_ASSET_INVAL_SZ		= 69,
	CLSIC_ERR_BPB_ASSET_INVAL_COMP_TYPE	= 70,
	CLSIC_ERR_BPB_ASSET_INVAL_COMP_TABLE_SZ = 71,
	CLSIC_ERR_BPB_ASSET_INVAL_FLAGS		= 72,
	CLSIC_ERR_AUTH_MAX_AUDIO_PROCESSED	= 73,
	CLSIC_ERR_AUTH_NO_USERS_TO_MATCH	= 74,
	CLSIC_ERR_AUTH_BIOM_DISABLED		= 75,
	CLSIC_ERR_REP_PLOSIVE			= 76,
	CLSIC_ERR_REP_SNR			= 77,
	CLSIC_ERR_REP_REWIND_OVF		= 78,
	CLSIC_ERR_INVALID_BIN_ID		= 79,
	CLSIC_ERR_INVALID_BIN_DATA		= 80,
	CLSIC_ERR_BIN_NOT_INSTALLED		= 81,
	CLSIC_ERR_BIOVTE_MAP_INVALID		= 82,
	CLSIC_ERR_BIOVTE_MAP_SZ_INVALID		= 83,
	CLSIC_ERR_BIOVTE_MAP_NOT_INSTALLED	= 84,
	CLSIC_ERR_BIOVTE_MAPPING_DOES_NOT_EXIST = 85,
	CLSIC_ERR_IOCTL_EXT_CODEC		= 86,
	CLSIC_ERR_BPB_PHRASE_CFG_FORMAT		= 87,
	CLSIC_ERR_BPB_PHRASE_CFG_KEY_ERR	= 88,
	CLSIC_ERR_RATE_LIMITED			= 89,
	CLSIC_ERR_BL_AUTH_FAILED		= 200,
	CLSIC_ERR_BL_INVAL_VERSION		= 201,
	CLSIC_ERR_BL_FLASH_WRITE_FAILED		= 202,
	CLSIC_ERR_BL_ARB_CHECK_FAILED		= 203,
	CLSIC_ERR_BL_CLUB_TOO_LARGE		= 204,
	CLSIC_ERR_BL_IMG_NAME_CLASH		= 205,
	CLSIC_ERR_BL_CAB_NOT_1ST_IN_MAB		= 206,
	CLSIC_ERR_BL_TOO_MANY_IMGS		= 207,
	CLSIC_ERR_BL_NO_MIN_SET_IN_MAB		= 208,
	CLSIC_ERR_BL_FLASH_ERASE_FAILED		= 209,
	CLSIC_ERR_BL_FLASH_READ_FAILED		= 210,
	CLSIC_ERR_BL_NBS2_NOT_1ST_IN_CAB	= 211,
	CLSIC_ERR_BL_OSAPP_NOT_2ND_IN_CAB	= 212,
};

#endif
