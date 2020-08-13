// SPDX-License-Identifier: GPL-2.0
//
// cl_dsp.c -- DSP Control for non-ALSA Cirrus Logic Devices
//
// Copyright 2019 Cirrus Logic, Inc.
//
// Author: Fred Treven <fred.treven@cirrus.com>

#include <linux/mfd/cl_dsp.h>

static int cl_dsp_raw_write(struct cl_dsp *dsp, unsigned int reg,
		const void *val, size_t val_len, size_t limit)
{
	int i, ret = 0;

	if (!dsp)
		return -EPERM;

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
	int ret = 0;
	struct cl_dsp_coeff_desc *coeff_desc;
	unsigned int mem_region_prefix;

	if  (!dsp)
		return -EPERM;

	list_for_each_entry(coeff_desc, &dsp->coeff_desc_head, list) {
		if (strncmp(coeff_desc->name, coeff_name,
				CL_DSP_COEFF_NAME_LEN_MAX))
			continue;
		if (coeff_desc->block_type != block_type)
			continue;
		if (coeff_desc->parent_id != algo_id)
			continue;

		*reg = coeff_desc->reg;
		if (*reg == 0x0) {
			dev_err(dsp->dev,
				"No DSP control called %s for block 0x%X\n",
				coeff_name, block_type);
			return -ENXIO;
		}

		break;
	}

	/* verify register found in expected region */
	switch (block_type) {
	case CL_DSP_XM_PACKED_TYPE:
		mem_region_prefix = (CL_DSP_HALO_XMEM_PACKED_BASE
				& CL_DSP_MEM_REG_TYPE_MASK)
				>> CL_DSP_MEM_REG_TYPE_SHIFT;
		break;
	case CL_DSP_XM_UNPACKED_TYPE:
		mem_region_prefix = (CL_DSP_HALO_XMEM_UNPACKED24_BASE
				& CL_DSP_MEM_REG_TYPE_MASK)
				>> CL_DSP_MEM_REG_TYPE_SHIFT;
		break;
	case CL_DSP_YM_PACKED_TYPE:
		mem_region_prefix = (CL_DSP_HALO_YMEM_PACKED_BASE
				& CL_DSP_MEM_REG_TYPE_MASK)
				>> CL_DSP_MEM_REG_TYPE_SHIFT;
		break;
	case CL_DSP_YM_UNPACKED_TYPE:
		mem_region_prefix = (CL_DSP_HALO_YMEM_UNPACKED24_BASE
				& CL_DSP_MEM_REG_TYPE_MASK)
				>> CL_DSP_MEM_REG_TYPE_SHIFT;
		break;
	case CL_DSP_PM_PACKED_TYPE:
		mem_region_prefix = (CL_DSP_HALO_PMEM_BASE
				& CL_DSP_MEM_REG_TYPE_MASK)
				>> CL_DSP_MEM_REG_TYPE_SHIFT;
		break;
	default:
		dev_err(dsp->dev, "Unrecognized block type: 0x%X\n",
				block_type);
		return -EINVAL;
	}

	if (((*reg & CL_DSP_MEM_REG_TYPE_MASK) >> CL_DSP_MEM_REG_TYPE_SHIFT)
			!= mem_region_prefix) {
		dev_err(dsp->dev,
			"DSP control %s at 0x%X found in unexpected region\n",
			coeff_name, *reg);

		ret = -EFAULT;
	}

	return ret;
}
EXPORT_SYMBOL(cl_dsp_get_reg);

static int cl_dsp_process_data_be(const u8 *data,
		const unsigned int num_bytes, unsigned int *val)
{
	int i;

	if (num_bytes > sizeof(unsigned int))
		return -EINVAL;

	*val = 0;
	for (i = 0; i < num_bytes; i++)
		*val += *(data + i) << (i * CL_DSP_BITS_PER_BYTE);

	return 0;
}

int cl_dsp_coeff_file_parse(struct cl_dsp *dsp, const struct firmware *fw)
{
	struct device *dev = dsp->dev;
	char wt_date[CL_DSP_WMDR_DATE_LEN];
	bool wt_found = false;
	unsigned int pos;
	unsigned int block_offset, block_type, block_len;
	unsigned int algo_id, algo_rev;
	unsigned int reg, wt_reg;
	int ret = -EINVAL;
	int i;

	if  (!dsp)
		return -EPERM;

	*wt_date = '\0';

	if (memcmp(fw->data, CL_DSP_WMDR_MAGIC_ID, CL_DSP_MAGIC_ID_SIZE)) {
		dev_err(dev, "Failed to recognize coefficient file\n");
		return ret;
	}

	if (fw->size % CL_DSP_BYTES_PER_WORD) {
		dev_err(dev, "Coefficient file is not word-aligned\n");
		return ret;
	}

	ret = cl_dsp_process_data_be(&fw->data[CL_DSP_MAGIC_ID_SIZE],
			CL_DSP_WMDR_HEADER_LEN_SIZE, &pos);
	if (ret) {
		dev_err(dev, "Could not read WMDR file header size\n");
		return ret;
	}

	while (pos < fw->size) {
		ret = cl_dsp_process_data_be(&fw->data[pos],
				CL_DSP_WMDR_DBLK_OFFSET_SIZE, &block_offset);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_WMDR_DBLK_OFFSET_SIZE;

		ret = cl_dsp_process_data_be(&fw->data[pos],
				CL_DSP_WMDR_DBLK_TYPE_SIZE, &block_type);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_WMDR_DBLK_TYPE_SIZE;

		ret = cl_dsp_process_data_be(&fw->data[pos],
				CL_DSP_WMDR_ALGO_ID_SIZE, &algo_id);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_WMDR_ALGO_ID_SIZE;

		ret = cl_dsp_process_data_be(&fw->data[pos],
				CL_DSP_WMDR_ALGO_REV_SIZE, &algo_rev);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_WMDR_ALGO_REV_SIZE;

		/* sample rate not used */
		pos += CL_DSP_WMDR_SAMPLE_RATE_SIZE;

		ret = cl_dsp_process_data_be(&fw->data[pos],
				CL_DSP_WMDR_DBLK_LEN_SIZE, &block_len);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_WMDR_DBLK_LEN_SIZE;

		if (block_type != CL_DSP_WMDR_NAME_TYPE &&
				block_type != CL_DSP_WMDR_INFO_TYPE) {
			for (i = 0; i < dsp->num_algos; i++) {
				if (algo_id == dsp->algo_info[i].id)
					break;
			}

			if (i == dsp->num_algos) {
				dev_err(dev, "Invalid algo. ID: 0x%06X\n",
						algo_id);
				ret = -EINVAL;
				return ret;
			}

			if (((algo_rev >> CL_DSP_ALGO_REV_SHIFT_RIGHT)
					& CL_DSP_ALGO_REV_MASK) !=
				(dsp->algo_info[i].rev
					& CL_DSP_ALGO_REV_MASK)) {
				dev_err(dev, "Invalid algo. rev.: %d.%d.%d\n",
					(algo_rev & 0xFF000000) >> 24,
					(algo_rev & 0xFF0000) >> 16,
					(algo_rev & 0XFF00) >> 8);

				return -EINVAL;
			}

			if (dsp->wt_desc && algo_id == dsp->wt_desc->id)
				wt_found = true;
		}

		switch (block_type) {
		case CL_DSP_WMDR_NAME_TYPE:
		case CL_DSP_WMDR_INFO_TYPE:
			reg = 0;

			if (block_len < CL_DSP_WMDR_DATE_LEN)
				break;

			if (memcmp(&fw->data[pos], CL_DSP_WMDR_DATE_PREFIX,
					CL_DSP_WMDR_DATE_PREFIX_LEN))
				break; /* date already recorded */

			memcpy(wt_date,
				&fw->data[pos + CL_DSP_WMDR_DATE_PREFIX_LEN],
				CL_DSP_WMDR_DATE_LEN -
				CL_DSP_WMDR_DATE_PREFIX_LEN);

			wt_date[CL_DSP_WMDR_DATE_LEN -
					CL_DSP_WMDR_DATE_PREFIX_LEN] = '\0';
			break;
		case CL_DSP_XM_UNPACKED_TYPE:
			reg = CL_DSP_HALO_XMEM_UNPACKED24_BASE + block_offset +
					dsp->algo_info[i].xm_base *
					CL_DSP_BYTES_PER_WORD;

			if (dsp->wt_desc) {
				ret = cl_dsp_get_reg(dsp,
						dsp->wt_desc->wt_name_xm,
						CL_DSP_XM_UNPACKED_TYPE,
						dsp->wt_desc->id, &wt_reg);
				if (ret)
					return ret;

				if (reg == wt_reg) {
					if (block_len >
						dsp->wt_desc->wt_limit_xm) {
						dev_err(dev,
						"XM too large: %d bytes\n",
						block_len / 4 * 3);

						return -EINVAL;
					}

					dev_dbg(dev,
					"Wavetable found: %d bytes (XM)\n",
							block_len / 4 * 3);
				}
			}
			break;
		case CL_DSP_XM_PACKED_TYPE:
			reg = (CL_DSP_HALO_XMEM_PACKED_BASE + block_offset +
					dsp->algo_info[i].xm_base *
					CL_DSP_PACKED_NUM_BYTES) &
					~CL_DSP_ALIGN;
			break;
		case CL_DSP_YM_UNPACKED_TYPE:
			reg = CL_DSP_HALO_YMEM_UNPACKED24_BASE + block_offset +
					dsp->algo_info[i].ym_base *
					CL_DSP_UNPACKED_NUM_BYTES;
			if (dsp->wt_desc) {
				ret = cl_dsp_get_reg(dsp,
						dsp->wt_desc->wt_name_ym,
						CL_DSP_YM_UNPACKED_TYPE,
						dsp->wt_desc->id, &wt_reg);
				if (ret)
					return ret;

				if (reg == wt_reg) {
					if (block_len >
						dsp->wt_desc->wt_limit_ym) {
						dev_err(dev,
						"YM too large: %d bytes\n",
						block_len / 4 * 3);

						return -EINVAL;
					}

					dev_dbg(dev,
					"Wavetable found: %d bytes (YM)\n",
							block_len / 4 * 3);
				}
			}
			break;
		case CL_DSP_YM_PACKED_TYPE:
			reg = (CL_DSP_HALO_YMEM_PACKED_BASE + block_offset +
					dsp->algo_info[i].ym_base *
					CL_DSP_PACKED_NUM_BYTES) &
					~CL_DSP_ALIGN;
		break;
		default:
			dev_err(dev, "Unexpected block type: 0x%04X\n",
					block_type);
			return -EINVAL;
		}
		if (reg) {
			ret = cl_dsp_raw_write(dsp, reg, &fw->data[pos],
					block_len, CL_DSP_MAX_WLEN);
			if (ret) {
				dev_err(dev, "Failed to write coefficients\n");
				return ret;
			}
		}

		/* Blocks are word-aligned */
		pos += (block_len + 3) & ~CL_DSP_ALIGN;
	}

	if (dsp->wt_desc && wt_found) {
		if (*wt_date != '\0')
			strlcpy(dsp->wt_desc->wt_date, wt_date,
					CL_DSP_WMDR_DATE_LEN);
		else
			strlcpy(dsp->wt_desc->wt_date,
					CL_DSP_WMDR_FILE_DATE_MISSING,
					CL_DSP_WMDR_DATE_LEN);
	}

	return 0;
}
EXPORT_SYMBOL(cl_dsp_coeff_file_parse);

static int cl_dsp_algo_parse(struct cl_dsp *dsp, const unsigned char *data)
{
	struct cl_dsp_coeff_desc *coeff_desc;
	u8 algo_name_len;
	u16 algo_desc_len, block_offset, block_type;
	u32 algo_id, block_len;
	char *algo_name;
	unsigned int coeff_count, val, pos = 0, header_pos = 0;
	unsigned int coeff_name_len, coeff_fullname_len, coeff_desc_len;
	unsigned int coeff_flags, coeff_len;
	int i, ret;

	ret = cl_dsp_process_data_be(&data[pos], CL_DSP_ALGO_ID_SIZE,
			&algo_id);
	if (ret) {
		dev_err(dsp->dev, "Failed to read data\n");
		return ret;
	}
	pos += CL_DSP_ALGO_ID_SIZE;

	ret = cl_dsp_process_data_be(&data[pos], CL_DSP_ALGO_NAME_LEN_SIZE,
			&val);
	if (ret) {
		dev_err(dsp->dev, "Failed to read data\n");
		return ret;
	}
	algo_name_len = (u8) (val & CL_DSP_BYTE_MASK);

	algo_name = kzalloc(algo_name_len, GFP_KERNEL);
	if (!algo_name)
		return -ENOMEM;

	memcpy(algo_name, &data[pos + CL_DSP_ALGO_NAME_LEN_SIZE],
			algo_name_len);
	pos += CL_DSP_WORD_ALIGN(algo_name_len);

	ret = cl_dsp_process_data_be(&data[pos], CL_DSP_ALGO_DESC_LEN_SIZE,
			&val);
	if (ret) {
		dev_err(dsp->dev, "Failed to read data\n");
		goto err_free;
	}

	/* Nothing required for algo. description, so it is skipped */
	algo_desc_len = (u16) (val & CL_DSP_NIBBLE_MASK);
	pos += CL_DSP_WORD_ALIGN(algo_desc_len);

	ret = cl_dsp_process_data_be(&data[pos], CL_DSP_COEFF_COUNT_SIZE,
			&coeff_count);
	if (ret) {
		dev_err(dsp->dev, "Failed to read data\n");
		goto err_free;
	}
	pos += CL_DSP_COEFF_COUNT_SIZE;

	for (i = 0; i < coeff_count; i++) {
		ret = cl_dsp_process_data_be(&data[pos],
				CL_DSP_COEFF_OFFSET_SIZE, &val);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			goto err_free;
		}

		block_offset = (u16) (val & CL_DSP_NIBBLE_MASK);
		pos += CL_DSP_COEFF_OFFSET_SIZE;

		ret = cl_dsp_process_data_be(&data[pos],
				CL_DSP_COEFF_TYPE_SIZE, &val);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			goto err_free;
		}

		block_type = (u16) (val & CL_DSP_NIBBLE_MASK);
		pos += CL_DSP_COEFF_TYPE_SIZE;

		ret = cl_dsp_process_data_be(&data[pos], CL_DSP_COEFF_LEN_SIZE,
				&block_len);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			goto err_free;
		}
		pos += CL_DSP_COEFF_LEN_SIZE;
		header_pos = pos;

		coeff_desc = devm_kzalloc(dsp->dev, sizeof(*coeff_desc),
				GFP_KERNEL);
		if (!coeff_desc) {
			ret = -ENOMEM;
			goto err_free;
		}

		coeff_desc->parent_id = algo_id;
		coeff_desc->parent_name = algo_name;
		coeff_desc->block_offset = block_offset;
		coeff_desc->block_type = block_type;

		memcpy(coeff_desc->name, data + pos + 1, *(data + pos));
		coeff_desc->name[*(data + pos)] = '\0';

		ret = cl_dsp_process_data_be(&data[pos],
				CL_DSP_COEFF_NAME_LEN_SIZE, &coeff_name_len);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_WORD_ALIGN(coeff_name_len);

		ret = cl_dsp_process_data_be(&data[pos],
				CL_DSP_COEFF_FULLNAME_LEN_SIZE,
				&coeff_fullname_len);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_WORD_ALIGN(coeff_fullname_len);

		ret = cl_dsp_process_data_be(&data[pos],
				CL_DSP_COEFF_DESC_LEN_SIZE, &coeff_desc_len);

		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_WORD_ALIGN(coeff_desc_len);

		ret = cl_dsp_process_data_be(&data[pos],
				CL_DSP_COEFF_FLAGS_SIZE, &coeff_flags);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_COEFF_FLAGS_SIZE;

		ret = cl_dsp_process_data_be(&data[pos], CL_DSP_COEFF_LEN_SIZE,
				&coeff_len);
		if (ret) {
			dev_err(dsp->dev, "Failed to read data\n");
			return ret;
		}
		pos += CL_DSP_COEFF_LEN_SIZE;

		coeff_desc->flags = coeff_flags >> CL_DSP_COEFF_FLAGS_SHIFT;
		coeff_desc->length = coeff_len;

		list_add(&coeff_desc->list, &dsp->coeff_desc_head);

		pos = header_pos + block_len;
	}

err_free:
	kfree(algo_name);

	return ret;
}

static int cl_dsp_coeff_init(struct cl_dsp *dsp)
{
	struct regmap *regmap = dsp->regmap;
	struct device *dev = dsp->dev;
	struct cl_dsp_coeff_desc *coeff_desc;
	unsigned int reg = CL_DSP_HALO_XM_FW_ID_REG;
	unsigned int val;
	int ret, i;

	if  (!dsp)
		return -EPERM;

	ret = regmap_read(regmap, CL_DSP_HALO_NUM_ALGOS_REG, &val);
	if (ret) {
		dev_err(dev, "Failed to read number of algorithms\n");
		return ret;
	}

	if (val > CL_DSP_NUM_ALGOS_MAX) {
		dev_err(dev, "Invalid number of algorithms: %d\n", val);
		return -EINVAL;
	}
	dsp->num_algos = val + 1;

	for (i = 0; i < dsp->num_algos; i++) {
		ret = regmap_read(regmap, reg, &dsp->algo_info[i].id);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d ID\n", i);
			return ret;
		}

		ret = regmap_read(regmap, reg + CL_DSP_HALO_ALGO_REV_OFFSET,
				&dsp->algo_info[i].rev);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d revision\n", i);
			return ret;
		}

		ret = regmap_read(regmap, reg +
				CL_DSP_HALO_ALGO_XM_BASE_OFFSET,
				&dsp->algo_info[i].xm_base);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d XM_BASE\n", i);
			return ret;
		}

		ret = regmap_read(regmap, reg +
				CL_DSP_HALO_ALGO_XM_SIZE_OFFSET,
				&dsp->algo_info[i].xm_size);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d XM_SIZE\n", i);
			return ret;
		}

		ret = regmap_read(regmap, reg +
				CL_DSP_HALO_ALGO_YM_BASE_OFFSET,
				&dsp->algo_info[i].ym_base);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d YM_BASE\n", i);
			return ret;
		}

		ret = regmap_read(regmap, reg +
				CL_DSP_HALO_ALGO_YM_SIZE_OFFSET,
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
						CL_DSP_HALO_XMEM_UNPACKED24_BASE
						+ dsp->algo_info[i].xm_base
						* CL_DSP_UNPACKED_NUM_BYTES
						+ coeff_desc->block_offset
						* CL_DSP_UNPACKED_NUM_BYTES;

				if (dsp->wt_desc && !strncmp(coeff_desc->name,
						dsp->wt_desc->wt_name_xm,
						CL_DSP_COEFF_NAME_LEN_MAX))
					dsp->wt_desc->wt_limit_xm =
						(dsp->algo_info[i].xm_size -
						coeff_desc->block_offset) *
						CL_DSP_UNPACKED_NUM_BYTES;
				break;
			case CL_DSP_YM_UNPACKED_TYPE:
				coeff_desc->reg =
					CL_DSP_HALO_YMEM_UNPACKED24_BASE +
					dsp->algo_info[i].ym_base *
					CL_DSP_BYTES_PER_WORD +
					coeff_desc->block_offset *
					CL_DSP_BYTES_PER_WORD;

				if (dsp->wt_desc && !strncmp(coeff_desc->name,
						dsp->wt_desc->wt_name_ym,
						CL_DSP_COEFF_NAME_LEN_MAX))
					dsp->wt_desc->wt_limit_ym =
						(dsp->algo_info[i].ym_size -
						coeff_desc->block_offset) *
						CL_DSP_UNPACKED_NUM_BYTES;
				break;
			}
			dev_dbg(dev, "Found control %s at 0x%08X\n",
					coeff_desc->name, coeff_desc->reg);
		}

		/* System algo. contains one extra register (num. algos.) */
		if (i)
			reg += CL_DSP_ALGO_ENTRY_SIZE;
		else
			reg += (CL_DSP_ALGO_ENTRY_SIZE +
					CL_DSP_BYTES_PER_WORD);
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

	if (dsp->wt_desc)
		dev_info(dev,
			"Max. wavetable size: %d bytes (XM), %d bytes (YM)\n",
			dsp->wt_desc->wt_limit_xm / 4 * 3,
			dsp->wt_desc->wt_limit_ym / 4 * 3);

	return 0;
}

static void cl_dsp_coeff_free(struct cl_dsp *dsp)
{
	struct cl_dsp_coeff_desc *coeff_desc;

	if  (!dsp)
		return;

	while (!list_empty(&dsp->coeff_desc_head)) {
		coeff_desc = list_first_entry(&dsp->coeff_desc_head,
				struct cl_dsp_coeff_desc, list);
		list_del(&coeff_desc->list);
		kfree(coeff_desc->parent_name);
		devm_kfree(dsp->dev, coeff_desc);
	}
}

static int cl_dsp_header_parse(struct cl_dsp *dsp,
		union cl_dsp_wmfw_header header)
{
	struct device *dev = dsp->dev;

	if (memcmp(header.magic, CL_DSP_WMFW_MAGIC_ID, CL_DSP_MAGIC_ID_SIZE)) {
		dev_err(dev, "Failed to recognize firmware file\n");
		return -EINVAL;
	}

	if (header.header_len != CL_DSP_FW_FILE_HEADER_SIZE) {
		dev_err(dev, "Malformed fimrware header\n");
		return -EINVAL;
	}

	if (header.api_revision != CL_DSP_API_REVISION) {
		dev_err(dev, "Firmware API Revision Incompatible with Core\n");
		return -EINVAL;
	}

	if (header.target_core != CL_DSP_TARGET_CORE_HALO) {
		dev_err(dev, "Unexpected target core type: 0x%02X\n",
				header.target_core);
		return -EINVAL;
	}

	if (header.file_format_version < CL_DSP_MIN_FORMAT_VERSION) {
		dev_err(dev, "File Format Version 0x%02X is outdated",
				header.file_format_version);
		return -EINVAL;
	}

	dev_info(dev,
		"Loading memory (bytes): XM: %u, YM: %u, PM: %u, ZM: %u\n",
		header.xm_size, header.ym_size, header.pm_size, header.zm_size);

	return 0;
}

static void cl_dsp_handle_info_text(struct cl_dsp *dsp,
		const u8 *data, u32 len)
{
	char *info_str;

	info_str = kzalloc(len, GFP_KERNEL);
	if (!info_str)
		/* info block is empty or memory not allocated */
		return;

	memcpy(info_str, data, len);

	dev_info(dsp->dev, "WMFW Info: %s\n", info_str);

	kfree(info_str);
}

int cl_dsp_firmware_parse(struct cl_dsp *dsp, const struct firmware *fw)
{
	struct device *dev = dsp->dev;
	unsigned int pos = CL_DSP_FW_FILE_HEADER_SIZE, reg = 0;
	struct cl_dsp_data_block *data_block;
	union cl_dsp_wmfw_header wmfw_header;
	int ret;

	if (!dsp)
		return -EPERM;

	memcpy(wmfw_header.data, fw->data, CL_DSP_FW_FILE_HEADER_SIZE);

	ret = cl_dsp_header_parse(dsp, wmfw_header);
	if (ret)
		return ret;

	if (fw->size % CL_DSP_BYTES_PER_WORD) {
		dev_err(dev, "Firmware file is not word-aligned\n");
		return -EINVAL;
	}

	data_block = (struct cl_dsp_data_block *)
			kmalloc(sizeof(struct cl_dsp_data_block), GFP_KERNEL);
	if (!data_block)
		return -ENOMEM;

	while (pos < fw->size) {
		memcpy(data_block->header.data, &fw->data[pos],
				CL_DSP_DBLK_HEADER_SIZE);

		pos += CL_DSP_DBLK_HEADER_SIZE;

		data_block->payload =
			(u8 *)kmalloc(data_block->header.data_len, GFP_KERNEL);
		memcpy(data_block->payload, &fw->data[pos],
				data_block->header.data_len);

		switch (data_block->header.block_type) {
		case CL_DSP_WMFW_INFO_TYPE:
			reg = 0;
			cl_dsp_handle_info_text(dsp, data_block->payload,
					data_block->header.data_len);
			break;
		case CL_DSP_PM_PACKED_TYPE:
			reg = CL_DSP_HALO_PMEM_BASE +
					data_block->header.start_offset *
					CL_DSP_PM_NUM_BYTES;
			break;
		case CL_DSP_XM_PACKED_TYPE:
			reg = CL_DSP_HALO_XMEM_PACKED_BASE +
					data_block->header.start_offset *
					CL_DSP_PACKED_NUM_BYTES;
			break;
		case CL_DSP_XM_UNPACKED_TYPE:
			reg = CL_DSP_HALO_XMEM_UNPACKED24_BASE +
					data_block->header.start_offset *
					CL_DSP_UNPACKED_NUM_BYTES;
			break;
		case CL_DSP_YM_PACKED_TYPE:
			reg = CL_DSP_HALO_YMEM_PACKED_BASE +
					data_block->header.start_offset *
					CL_DSP_PACKED_NUM_BYTES;
			break;
		case CL_DSP_YM_UNPACKED_TYPE:
			reg = CL_DSP_HALO_YMEM_UNPACKED24_BASE +
					data_block->header.start_offset *
					CL_DSP_UNPACKED_NUM_BYTES;
			break;
		case CL_DSP_ALGO_INFO_TYPE:
			reg = 0;

			ret = cl_dsp_algo_parse(dsp, data_block->payload);
			if (ret)
				goto err_free;
			break;
		default:
			dev_err(dev, "Unexpected block type : 0x%02X\n",
					data_block->header.block_type);
			ret = -EINVAL;
			goto err_free;
		}

		if (dsp->fw_desc->write_fw && reg) {
			ret = cl_dsp_raw_write(dsp, reg, data_block->payload,
					data_block->header.data_len,
					CL_DSP_MAX_WLEN);
			if (ret) {
				dev_err(dev,
					"Failed to write to base 0x%X\n", reg);
				goto err_free;
			}
		}

		/* Blocks are word-aligned */
		pos += (data_block->header.data_len + 3) & ~CL_DSP_ALIGN;

		kfree(data_block->payload);
	}

	kfree(data_block);
	return cl_dsp_coeff_init(dsp);

err_free:
	if (data_block->payload)
		kfree(data_block->payload);

	kfree(data_block);

	return ret;
}
EXPORT_SYMBOL(cl_dsp_firmware_parse);

int cl_dsp_wavetable_create(struct cl_dsp *dsp, unsigned int id,
		const char *wt_name_xm, const char *wt_name_ym,
		const char *wt_file)
{
	struct cl_dsp_wt_desc *wt_desc;

	if (!dsp)
		return -EPERM;

	wt_desc = devm_kzalloc(dsp->dev, sizeof(struct cl_dsp_wt_desc),
			GFP_KERNEL);
	if (!wt_desc)
		return -ENOMEM;

	wt_desc->id = id;
	memcpy(wt_desc->wt_name_xm, wt_name_xm, CL_DSP_WMDR_NAME_LEN);
	memcpy(wt_desc->wt_name_ym, wt_name_ym, CL_DSP_WMDR_NAME_LEN);
	memcpy(wt_desc->wt_file, wt_file, CL_DSP_WMDR_NAME_LEN);

	dsp->wt_desc = wt_desc;

	return 0;
}
EXPORT_SYMBOL(cl_dsp_wavetable_create);

struct cl_dsp *cl_dsp_create(struct device *dev, struct regmap *regmap)
{
	struct cl_dsp *dsp;

	dsp = devm_kzalloc(dev, sizeof(struct cl_dsp), GFP_KERNEL);
	if (!dsp)
		return NULL;

	dsp->dev = dev;
	dsp->regmap = regmap;

	INIT_LIST_HEAD(&dsp->coeff_desc_head);

	return dsp;
}
EXPORT_SYMBOL(cl_dsp_create);

int cl_dsp_destroy(struct cl_dsp *dsp)
{
	if (!dsp)
		return -EPERM;

	if (!list_empty(&dsp->coeff_desc_head))
		cl_dsp_coeff_free(dsp);

	if (dsp->wt_desc)
		devm_kfree(dsp->dev, dsp->wt_desc);

	devm_kfree(dsp->dev, dsp);

	return 0;
}
EXPORT_SYMBOL(cl_dsp_destroy);

MODULE_DESCRIPTION("Cirrus Logic DSP Firmware Driver");
MODULE_AUTHOR("Fred Treven, Cirrus Logic Inc, <fred.treven@cirrus.com>");
MODULE_LICENSE("GPL");
