/* SPDX-License-Identifier: GPL-2.0
 *
 * cl_dsp.h -- DSP control for non-ALSA Cirrus Logic devices
 *
 * Copyright 2019 Cirrus Logic, Inc.
 *
 * Author: Fred Treven <fred.treven@cirrus.com>
 */

#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/regmap.h>
#include <linux/of_device.h>

#ifndef __CL_DSP_H__
#define __CL_DSP_H__

#define CL_DSP_BYTES_PER_WORD		4
#define CL_DSP_BITS_PER_BYTE		8

#define CL_DSP_MAGIC_ID_SIZE		4

#define CL_DSP_WMFW_MAGIC_ID		"WMFW"
#define CL_DSP_WMDR_MAGIC_ID		"WMDR"

#define CL_DSP_DBLK_HEADER_SIZE	8

#define CL_DSP_WORD_ALIGN	0x00000003

#define CL_DSP_ALGO_ID_SIZE		4
#define CL_DSP_COEFF_COUNT_SIZE	4
#define CL_DSP_COEFF_OFFSET_SIZE	2
#define CL_DSP_COEFF_TYPE_SIZE		2
#define CL_DSP_COEFF_LENGTH_SIZE	4
#define CL_DSP_COEFF_NAME_LEN_MAX	32

#define CL_DSP_ALGO_LIST_TERM		0xBEDEAD
#define CL_DSP_ALGO_REV_MASK		GENMASK(23, 16)
#define CL_DSP_ALGO_REV_SHIFT_RIGHT	8

#define CL_DSP_NUM_ALGOS_MAX		16

#define CL_DSP_MAX_WLEN			4096

#define CL_DSP_FW_FILE_HEADER_SIZE	40

#define CL_DSP_XM_UNPACKED_TYPE	0x0005
#define CL_DSP_YM_UNPACKED_TYPE	0x0006
#define CL_DSP_PM_PACKED_TYPE	0x0010
#define CL_DSP_XM_PACKED_TYPE	0x0011
#define CL_DSP_YM_PACKED_TYPE	0x0012
#define CL_DSP_ALGO_INFO_TYPE	0x00F2
#define CL_DSP_WMFW_INFO_TYPE	0x00FF

#define CL_DSP_MEM_REG_TYPE_MASK	GENMASK(27, 20)
#define CL_DSP_MEM_REG_TYPE_SHIFT	20

#define CL_DSP_PM_NUM_BYTES	5
#define CL_DSP_PACKED_NUM_BYTES	3
#define CL_DSP_UNPACKED_NUM_BYTES	4

#define CL_DSP_WMDR_DBLK_OFFSET_SIZE	2
#define CL_DSP_WMDR_DBLK_TYPE_SIZE	2
#define CL_DSP_WMDR_ALGO_ID_SIZE	4
#define CL_DSP_WMDR_ALGO_REV_SIZE	4
#define CL_DSP_WMDR_SAMPLE_RATE_SIZE	4
#define CL_DSP_WMDR_DBLK_LENGTH_SIZE	4
#define CL_DSP_WMDR_NAME_LEN		32
#define CL_DSP_WMDR_DATE_LEN		16

#define CL_DSP_WMDR_DATE_PREFIX	"Date: "
#define CL_DSP_WMDR_DATE_PREFIX_LEN	6

#define CL_DSP_WMDR_FILE_NAME_MISSING	"N/A"
#define CL_DSP_WMDR_FILE_DATE_MISSING	"N/A"

#define CL_DSP_WMDR_NAME_TYPE		0xFE00
#define CL_DSP_WMDR_INFO_TYPE		0xFF00

union cl_dsp_data_block_header {
	struct {
		u32 start_offset : 24;
		u8 block_type;
		u32 data_len;
	};
	u8 data[CL_DSP_DBLK_HEADER_SIZE];
};

struct cl_dsp_fw_desc {
	unsigned int id;
	unsigned int min_rev;
	unsigned int halo_state_run;
	unsigned int num_coeff_files;
	const char * const *coeff_files;
	const char *fw_file;
};

struct cl_dsp_coeff_desc {
	unsigned int parent_id;
	unsigned int block_offset;
	unsigned int block_type;
	unsigned char name[CL_DSP_COEFF_NAME_LEN_MAX];
	unsigned int reg;
	struct list_head list;
};

struct cl_dsp_wt_desc {
	unsigned int id;
	char wt_name_xm[CL_DSP_WMDR_NAME_LEN];
	char wt_name_ym[CL_DSP_WMDR_NAME_LEN];
	unsigned int wt_limit_xm;
	unsigned int wt_limit_ym;
	char wt_file[CL_DSP_WMDR_NAME_LEN];
	char wt_date[CL_DSP_WMDR_DATE_LEN];
};

struct cl_dsp_algo_info {
	unsigned int id;
	unsigned int rev;
	unsigned int xm_base;
	unsigned int xm_size;
	unsigned int ym_base;
	unsigned int ym_size;
};

struct cl_dsp_mem_reg_desc {
	unsigned int pm_base_reg;
	unsigned int xm_base_reg_packed;
	unsigned int xm_base_reg_unpacked_24;
	unsigned int xm_base_reg_unpacked_32;
	unsigned int ym_base_reg_packed;
	unsigned int ym_base_reg_unpacked_24;
	unsigned int ym_base_reg_unpacked_32;
};

struct cl_dsp_algo_params {
	unsigned int xm_fw_id_reg;
	unsigned int xm_num_algos_reg;
	unsigned int algo_id_offset;
	unsigned int algo_rev_offset;
	unsigned int algo_xm_base_offset;
	unsigned int algo_xm_size_offset;
	unsigned int algo_ym_base_offset;
	unsigned int algo_ym_size_offset;
	unsigned int algo_entry_size;
};

struct cl_dsp {
	struct device *dev;
	struct regmap *regmap;
	struct list_head coeff_desc_head;
	unsigned int num_algos;
	struct cl_dsp_algo_info algo_info[CL_DSP_NUM_ALGOS_MAX + 1];
	const struct cl_dsp_fw_desc *fw_desc;
	const struct cl_dsp_mem_reg_desc *mem_reg_desc;
	const struct cl_dsp_algo_params *algo_params;
	struct cl_dsp_wt_desc *wt_desc;
};

struct cl_dsp *cl_dsp_create(struct device *dev);
int cl_dsp_destroy(struct cl_dsp *dsp);
int cl_dsp_wavetable_create(struct cl_dsp *dsp, unsigned int id,
		const char *wt_name_xm, const char *wt_name_ym,
		const char *wt_file);
int cl_dsp_firmware_parse(struct cl_dsp *dsp, const struct firmware *fw);
int cl_dsp_coeff_file_parse(struct cl_dsp *dsp, const struct firmware *fw);
int cl_dsp_get_reg(struct cl_dsp *dsp, const char *coeff_name,
		const unsigned int block_type, const unsigned int algo_id,
		unsigned int *reg);

#endif /* __CL_DSP_H */
