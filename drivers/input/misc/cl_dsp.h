/* SPDX-License-Identifier: GPL-2.0
 *
 * cl_dsp.h -- DSP control for non-ALSA Cirrus Logic devices
 *
 * Copyright 2019 Cirrus Logic, Inc.
 *
 * Author: Fred Treven <fred.treven@cirrus.com>
 */

#ifndef __CL_DSP_H
#define __CL_DSP_H


#define CL_DSP_ALGO_ID_SIZE	4
#define CL_DSP_COEFF_COUNT_SIZE	4
#define CL_DSP_COEFF_OFFSET_SIZE	2
#define CL_DSP_COEFF_TYPE_SIZE	2
#define CL_DSP_COEFF_LENGTH_SIZE	4
#define CL_DSP_COEFF_NAME_LEN_MAX	32

#define CL_DSP_ALGO_LIST_TERM	0xBEDEAD
#define CL_DSP_ALGO_REV_MASK	GENMASK(23, 16)

#define CL_DSP_NUM_ALGOS_MAX	16

#define CL_DSP_MAX_WLEN	4096

#define CL_DSP_FW_FILE_HEADER_SIZE	40
#define CL_DSP_FW_DBLK_OFFSET_SIZE	3
#define CL_DSP_FW_DBLK_TYPE_SIZE	1
#define CL_DSP_FW_DBLK_LENGTH_SIZE	4

#define CL_DSP_XM_UNPACKED_TYPE	0x0005
#define CL_DSP_YM_UNPACKED_TYPE	0x0006
#define CL_DSP_PM_PACKED_TYPE	0x0010
#define CL_DSP_XM_PACKED_TYPE	0x0011
#define CL_DSP_YM_PACKED_TYPE	0x0012
#define CL_DSP_ALGO_INFO_TYPE	0x00F2
#define CL_DSP_WMFW_INFO_TYPE	0x00FF

#define CL_DSP_BITS_PER_BYTE	8

struct cl_dsp_fw_desc {
	unsigned int id;
	unsigned int min_rev;
	unsigned int halo_state_run;
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
	unsigned int xm_base_reg_unpacked;
	unsigned int ym_base_reg_packed;
	unsigned int ym_base_reg_unpacked;
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
	unsigned int wt_limit_xm;
	unsigned int wt_limit_ym;
};

int cl_dsp_firmware_parse(struct cl_dsp *dsp, const struct firmware *fw);
int cl_dsp_get_reg(struct cl_dsp *dsp, const char *coeff_name,
		const unsigned int block_type, const unsigned int algo_id,
		unsigned int *reg);

#endif /* __CL_DSP_H */
