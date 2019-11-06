// SPDX-License-Identifier: GPL-2.0
//
// cl_dsp.c -- DSP Control for non-ALSA Cirrus Logic Devices
//
// Copyright 2019 Cirrus Logic, Inc.
//
// Author: Fred Treven <fred.treven@cirrus.com>


#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/regmap.h>
#include <linux/firmware.h>
#include <linux/of_device.h>

#include "cl_dsp.h"

static int cl_dsp_raw_write(struct cl_dsp *dsp, unsigned int reg,
		const void *val, size_t val_len, size_t limit)
{
	int i, ret = 0;

	/* Restrict write length to limit value */
	for (i = 0; i < val_len; i += limit) {
		ret = regmap_raw_write(dsp->regmap, (reg + i), (val + i),
				(val_len - i) > limit ? limit : (val_len - i));
		if (ret)
			break;
	}

	return ret;
}

int cl_dsp_get_reg(struct cl_dsp *dsp, const char *coeff_name,
		const unsigned int block_type,
		const unsigned int algo_id, unsigned int *reg)
{
	int ret;
	struct cl_dsp_coeff_desc *coeff_desc;

	list_for_each_entry(coeff_desc, &dsp->coeff_desc_head, list) {
		if (strncmp(coeff_desc->name, coeff_name,
				CL_DSP_COEFF_NAME_LEN_MAX))
			continue;
		if (coeff_desc->block_type != block_type)
			continue;
		if (coeff_desc->parent_id != algo_id)
			continue;

		*reg = coeff_desc->reg;
		if (*reg == 0x0) /* No corresponding DSP register found */
			ret = -ENXIO;
		else
			ret = 0;

		break;
	}

	return ret;
}
EXPORT_SYMBOL(cl_dsp_get_reg);

static int cl_dsp_process_data_be(const unsigned char *data,
		const unsigned int num_bytes, const unsigned int pos,
		unsigned int *val)
{
	int i;

	if (num_bytes > sizeof(unsigned int))
		return -EINVAL;

	*val = 0;
	for (i = 0; i < num_bytes; i++)
		*val += *(data + pos + i) << i * CL_DSP_BITS_PER_BYTE;

	return 0;
}

static int cl_dsp_algo_parse(struct cl_dsp *dsp, const unsigned char *data)
{
	struct cl_dsp_coeff_desc *coeff_desc;
	unsigned int pos = 0;
	unsigned int algo_id, algo_desc_length, coeff_count;
	unsigned int block_offset, block_type, block_length;
	unsigned int val;
	unsigned char algo_name_length;
	int i, ret;

	ret = cl_dsp_process_data_be(data, 4, pos, &algo_id);
	if (ret) {
		dev_err(dsp->dev, "Failed to read data\n");
		return ret;
	}
	pos += CL_DSP_ALGO_ID_SIZE;

	/* Skip algorithm name */
	ret = cl_dsp_process_data_be(data, 1, pos, &val);
	if (ret) {
		dev_err(dsp->dev, "Failed to read data\n");
		return ret;
	}
	algo_name_length = val; /* Cast to unsigned char */
	pos += ((algo_name_length / 4) * 4) + 4;

	/* Skip algorithm description */
	ret = cl_dsp_process_data_be(data, 2, pos, &algo_desc_length);
	if (ret) {
		dev_err(dsp->dev, "Failed to read data\n");
		return ret;
	}
	pos += ((algo_desc_length / 4) * 4) + 4;

	/* Record coefficient count */
	ret = cl_dsp_process_data_be(data, 4, pos, &coeff_count);
	if (ret) {
		dev_err(dsp->dev, "Failed to read data\n");
		return ret;
	}
	pos += CL_DSP_COEFF_COUNT_SIZE;

	for (i = 0; i < coeff_count; i++) {
		ret = cl_dsp_process_data_be(data, 2, pos, &block_offset);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_COEFF_OFFSET_SIZE;

		ret = cl_dsp_process_data_be(data, 2, pos, &block_type);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_COEFF_TYPE_SIZE;

		ret = cl_dsp_process_data_be(data, 4, pos, &block_length);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_COEFF_LENGTH_SIZE;

		coeff_desc = devm_kzalloc(dsp->dev, sizeof(*coeff_desc),
				GFP_KERNEL);
		if (!coeff_desc)
			return -ENOMEM;

		coeff_desc->parent_id = algo_id;
		coeff_desc->block_offset = block_offset;
		coeff_desc->block_type = block_type;

		memcpy(coeff_desc->name, data + pos + 1, *(data + pos));
		coeff_desc->name[*(data + pos)] = '\0';

		list_add(&coeff_desc->list, &dsp->coeff_desc_head);

		pos += block_length;
	}

	return 0;
}

static int cl_dsp_coeff_init(struct cl_dsp *dsp)
{
	struct regmap *regmap = dsp->regmap;
	struct device *dev = dsp->dev;
	struct cl_dsp_coeff_desc *coeff_desc;
	unsigned int reg = dsp->algo_params->xm_fw_id_reg;
	unsigned int val;
	int ret, i;

	ret = regmap_read(regmap, dsp->algo_params->xm_num_algos_reg, &val);
	if (ret) {
		dev_err(dev, "Failed to read number of algorithms\n");
		return ret;
	}

	if (val > CL_DSP_NUM_ALGOS_MAX) {
		dev_err(dev, "Invalid numer of algorithms\n");
		return -EINVAL;
	}
	dsp->num_algos = val + 1;

	for (i = 0; i < dsp->num_algos; i++) {
		ret = regmap_read(regmap, reg +
				dsp->algo_params->algo_id_offset,
				&dsp->algo_info[i].id);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d ID\n", i);
			return ret;
		}

		ret = regmap_read(regmap, reg +
				dsp->algo_params->algo_rev_offset,
				&dsp->algo_info[i].rev);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d revision\n", i);
			return ret;
		}

		ret = regmap_read(regmap, reg +
				dsp->algo_params->algo_xm_base_offset,
				&dsp->algo_info[i].xm_base);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d XM_BASE\n", i);
			return ret;
		}

		ret = regmap_read(regmap, reg +
				dsp->algo_params->algo_xm_size_offset,
				&dsp->algo_info[i].xm_size);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d XM_SIZE\n", i);
			return ret;
		}

		ret = regmap_read(regmap, reg +
				dsp->algo_params->algo_ym_base_offset,
				&dsp->algo_info[i].ym_base);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d YM_BASE\n", i);
			return ret;
		}

		ret = regmap_read(regmap, reg +
				dsp->algo_params->algo_ym_size_offset,
				&dsp->algo_info[i].ym_size);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d YM_SIZE\n", i);
			return ret;
		}

		list_for_each_entry(coeff_desc, &dsp->coeff_desc_head,
				list) {
			if (coeff_desc->parent_id != dsp->algo_info[i].id)
				continue;

			switch (coeff_desc->block_type) {
			case CL_DSP_XM_UNPACKED_TYPE:
				coeff_desc->reg =
					dsp->mem_reg_desc->xm_base_reg_unpacked
					+ dsp->algo_info[i].xm_base * 4
					+ coeff_desc->block_offset * 4;
				if (!strncmp(coeff_desc->name, "WAVETABLE",
						CL_DSP_COEFF_NAME_LEN_MAX))
					dsp->wt_limit_xm =
						(dsp->algo_info[i].xm_size -
						coeff_desc->block_offset) * 4;
				break;
			case CL_DSP_YM_UNPACKED_TYPE:
				coeff_desc->reg =
					dsp->mem_reg_desc->ym_base_reg_unpacked
					+ dsp->algo_info[i].ym_base * 4
					+ coeff_desc->block_offset * 4;
				if (!strncmp(coeff_desc->name, "WAVETABLEYM",
						CL_DSP_COEFF_NAME_LEN_MAX))
					dsp->wt_limit_ym =
						(dsp->algo_info[i].ym_size -
						coeff_desc->block_offset) * 4;
				break;
			}
			dev_dbg(dev, "Found control %s at 0x%08X\n",
					coeff_desc->name, coeff_desc->reg);
		}

		/* System algo. contains one extra register (num. algos.) */
		if (i)
			reg += dsp->algo_params->algo_entry_size;
		else
			reg += (dsp->algo_params->algo_entry_size + 4);
	}

	ret = regmap_read(regmap, reg, &val);
	if (ret) {
		dev_err(dev, "Failed to read list terminator\n");
		return ret;
	}

	if (val != CL_DSP_ALGO_LIST_TERM) {
		dev_err(dev, "Invalid list terminator: 0x%X\n", val);
		return -EINVAL;
	}

	if (dsp->algo_info[0].id != dsp->fw_desc->id) {
		dev_err(dev, "Invalid firmware ID\n: 0x%06X\n",
				dsp->algo_info[0].id);
		return -EINVAL;
	}

	if (dsp->algo_info[0].rev < dsp->fw_desc->min_rev) {
		dev_err(dev, "Invalid firmware revision: %d.%d.%d\n",
				(dsp->algo_info[0].rev & 0xFF0000) >> 16,
				(dsp->algo_info[0].rev & 0xFF00) >> 8,
				dsp->algo_info[0].rev & 0xFF);
		return -EINVAL;
	}

	dev_info(dev, "Firmware revision %d.%d.%d\n",
			(dsp->algo_info[0].rev & 0xFF0000) >> 16,
			(dsp->algo_info[0].rev & 0xFF00) >> 8,
			dsp->algo_info[0].rev & 0xFF);

	return 0;
}

int cl_dsp_firmware_parse(struct cl_dsp *dsp, const struct firmware *fw)
{
	struct device *dev = dsp->dev;
	unsigned int pos = CL_DSP_FW_FILE_HEADER_SIZE;
	unsigned int block_offset, block_type, block_length;
	int ret = -EINVAL;

	if (memcmp(fw->data, "WMFW", 4)) {
		dev_err(dev, "Failed to recognize firmware file\n");
		goto err_rls_fw;
	}

	if (fw->size % 4) {
		dev_err(dev, "Firmware file is not word-aligned\n");
		goto err_rls_fw;
	}

	while (pos < fw->size) {
		ret = cl_dsp_process_data_be(fw->data, 3, pos, &block_offset);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_FW_DBLK_OFFSET_SIZE;

		ret = cl_dsp_process_data_be(fw->data, 1, pos, &block_type);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_FW_DBLK_TYPE_SIZE;

		ret = cl_dsp_process_data_be(fw->data, 4, pos, &block_length);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_FW_DBLK_LENGTH_SIZE;

		switch (block_type) {
		case CL_DSP_WMFW_INFO_TYPE:
			break;
		case CL_DSP_PM_PACKED_TYPE:
			ret = cl_dsp_raw_write(dsp,
					dsp->mem_reg_desc->pm_base_reg +
					block_offset * 5, &fw->data[pos],
					block_length, CL_DSP_MAX_WLEN);
			if (ret) {
				dev_err(dev,
					"Failed to write PM_PACKED memory\n");
				goto err_rls_fw;
			}
			break;
		case CL_DSP_XM_PACKED_TYPE:
			ret = cl_dsp_raw_write(dsp,
					dsp->mem_reg_desc->xm_base_reg_packed +
					block_offset * 3, &fw->data[pos],
					block_length, CL_DSP_MAX_WLEN);
			if (ret) {
				dev_err(dev,
					"Failed to write XM_PACKED memory\n");
				goto err_rls_fw;
			}
			break;
		case CL_DSP_YM_PACKED_TYPE:
			ret = cl_dsp_raw_write(dsp,
					dsp->mem_reg_desc->ym_base_reg_packed +
					block_offset * 3, &fw->data[pos],
					block_length, CL_DSP_MAX_WLEN);
			if (ret) {
				dev_err(dev,
					"Failed to write YM_PACKED memory\n");
				goto err_rls_fw;
			}
			break;
		case CL_DSP_ALGO_INFO_TYPE:
			ret = cl_dsp_algo_parse(dsp, &fw->data[pos]);
			if (ret) {
				dev_err(dev, "Failed to parse algorithm %d\n",
						ret);
				goto err_rls_fw;
			}
			break;
		default:
			dev_err(dev, "Unexpected block type : 0x%02X\n",
					block_type);
			ret = -EINVAL;
			goto err_rls_fw;
		}

		/* Blocks are word-aligned */
		pos += (block_length + 3) & ~0x00000003;
	}

	ret = cl_dsp_coeff_init(dsp);

err_rls_fw:
	release_firmware(fw);

	return ret;
}
EXPORT_SYMBOL(cl_dsp_firmware_parse);

MODULE_LICENSE("GPL");
