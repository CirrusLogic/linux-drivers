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
#include <linux/slab.h>

#ifndef __CL_DSP_H__
#define __CL_DSP_H__

#define CL_DSP_BYTES_PER_WORD		4
#define CL_DSP_BITS_PER_BYTE		8

#define CL_DSP_BYTE_MASK		GENMASK(7, 0)
#define CL_DSP_NIBBLE_MASK		GENMASK(15, 0)

#define CL_DSP_FW_FILE_HEADER_SIZE	40

#define CL_DSP_MAGIC_ID_SIZE		4

#define CL_DSP_WMFW_MAGIC_ID		"WMFW"
#define CL_DSP_WMDR_MAGIC_ID		"WMDR"

#define CL_DSP_DBLK_HEADER_SIZE	8

#define CL_DSP_ALIGN			0x00000003

#define CL_DSP_TARGET_CORE_HALO	0x04
#define CL_DSP_MIN_FORMAT_VERSION	0x03
#define CL_DSP_API_REVISION		0x0300

#define CL_DSP_ALGO_NAME_LEN_SIZE	1
#define CL_DSP_ALGO_DESC_LEN_SIZE	2
#define CL_DSP_ALGO_ID_SIZE		4
#define CL_DSP_COEFF_COUNT_SIZE	4
#define CL_DSP_COEFF_OFFSET_SIZE	2
#define CL_DSP_COEFF_TYPE_SIZE		2
#define CL_DSP_COEFF_NAME_LEN_SIZE	1
#define CL_DSP_COEFF_FULLNAME_LEN_SIZE	1
#define CL_DSP_COEFF_DESC_LEN_SIZE	2
#define CL_DSP_COEFF_LEN_SIZE		4
#define CL_DSP_COEFF_FLAGS_SIZE	4
#define CL_DSP_COEFF_FLAGS_SHIFT	16
#define CL_DSP_COEFF_NAME_LEN_MAX	32

#define CL_DSP_ALGO_LIST_TERM		0xBEDEAD
#define CL_DSP_ALGO_REV_MASK		GENMASK(23, 16)
#define CL_DSP_ALGO_REV_SHIFT_RIGHT	8

#define CL_DSP_NUM_ALGOS_MAX		32

#define CL_DSP_MAX_WLEN			4096

#define CL_DSP_XM_UNPACKED_TYPE	0x0005
#define CL_DSP_YM_UNPACKED_TYPE	0x0006
#define CL_DSP_PM_PACKED_TYPE	0x0010
#define CL_DSP_XM_PACKED_TYPE	0x0011
#define CL_DSP_YM_PACKED_TYPE	0x0012
#define CL_DSP_ALGO_INFO_TYPE	0x00F2
#define CL_DSP_WMFW_INFO_TYPE	0x00FF

#define CL_DSP_MEM_REG_TYPE_MASK	GENMASK(27, 20)
#define CL_DSP_MEM_REG_TYPE_SHIFT	20

#define CL_DSP_PM_NUM_BYTES		5
#define CL_DSP_PACKED_NUM_BYTES	3
#define CL_DSP_UNPACKED_NUM_BYTES	4

#define CL_DSP_WMDR_DBLK_OFFSET_SIZE	2
#define CL_DSP_WMDR_DBLK_TYPE_SIZE	2
#define CL_DSP_WMDR_ALGO_ID_SIZE	4
#define CL_DSP_WMDR_ALGO_REV_SIZE	4
#define CL_DSP_WMDR_SAMPLE_RATE_SIZE	4
#define CL_DSP_WMDR_DBLK_LEN_SIZE	4
#define CL_DSP_WMDR_NAME_LEN		32
#define CL_DSP_WMDR_DATE_LEN		16
#define CL_DSP_WMDR_HEADER_LEN_SIZE	4

#define CL_DSP_WMDR_DATE_PREFIX	"Date: "
#define CL_DSP_WMDR_DATE_PREFIX_LEN	6

#define CL_DSP_WMDR_FILE_NAME_MISSING	"N/A"
#define CL_DSP_WMDR_FILE_DATE_MISSING	"N/A"

#define CL_DSP_WMDR_NAME_TYPE		0xFE00
#define CL_DSP_WMDR_INFO_TYPE		0xFF00

//HALO core specific registers
#define CL_DSP_HALO_XMEM_PACKED_BASE		0x02000000
#define CL_DSP_HALO_XROM_PACKED_BASE		0x02006000
#define CL_DSP_HALO_XMEM_UNPACKED32_BASE	0x02400000
#define CL_DSP_HALO_XMEM_UNPACKED24_BASE	0x02800000
#define CL_DSP_HALO_XROM_UNPACKED24_BASE	0x02808000
#define CL_DSP_HALO_YMEM_PACKED_BASE		0x02C00000
#define CL_DSP_HALO_YMEM_UNPACKED32_BASE	0x03000000
#define CL_DSP_HALO_YMEM_UNPACKED24_BASE	0x03400000
#define CL_DSP_HALO_PMEM_BASE			0x03800000
#define CL_DSP_HALO_PROM_BASE			0x03C60000

#define CL_DSP_HALO_XM_FW_ID_REG		0x0280000C
#define CL_DSP_HALO_NUM_ALGOS_REG		0x02800024

#define CL_DSP_HALO_ALGO_REV_OFFSET		4
#define CL_DSP_HALO_ALGO_XM_BASE_OFFSET	8
#define CL_DSP_HALO_ALGO_XM_SIZE_OFFSET	12
#define CL_DSP_HALO_ALGO_YM_BASE_OFFSET	16
#define CL_DSP_HALO_ALGO_YM_SIZE_OFFSET	20
#define CL_DSP_ALGO_ENTRY_SIZE			24

/* macros */
#define CL_DSP_WORD_ALIGN(n)	(CL_DSP_BYTES_PER_WORD +\
				(((n) / CL_DSP_BYTES_PER_WORD) *\
				CL_DSP_BYTES_PER_WORD))


union cl_dsp_wmfw_header {
	struct {
		char magic[CL_DSP_BYTES_PER_WORD];
		u32 header_len;
		u16 api_revision;
		u8 target_core;
		u8 file_format_version;
		u32 xm_size;
		u32 ym_size;
		u32 pm_size;
		u32 zm_size;
		time_t timestamp[2];
		u32 checksum;
	} __attribute__((__packed__));
	u8 data[CL_DSP_FW_FILE_HEADER_SIZE];
};

union cl_dsp_data_block_header {
	struct {
		u32 start_offset : 24;
		u8 block_type;
		u32 data_len;
	} __attribute__((__packed__));
	u8 data[CL_DSP_DBLK_HEADER_SIZE];
};

struct cl_dsp_data_block {
	union cl_dsp_data_block_header header;
	u8 *payload;
};

struct cl_dsp_fw_desc {
	unsigned int id;
	unsigned int min_rev;
	unsigned int halo_state_run;
	unsigned int num_coeff_files;
	const char * const *coeff_files;
	const char *fw_file;
	bool write_fw;
};

struct cl_dsp_coeff_desc {
	u32 parent_id;
	char *parent_name;
	u16 block_offset;
	u16 block_type;
	unsigned char name[CL_DSP_COEFF_NAME_LEN_MAX];
	unsigned int reg;
	unsigned int flags;
	unsigned int length;
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

struct cl_dsp *cl_dsp_create(struct device *dev, struct regmap *regmap);
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
