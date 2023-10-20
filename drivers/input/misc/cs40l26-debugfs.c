// SPDX-License-Identifier: GPL-2.0
//
// cs40l26-debugfs.c -- CS40L26 Boosted Haptic Driver with Integrated DSP and
// Waveform Memory with Advanced Closed Loop Algorithms and LRA protection
//
// Copyright 2022 Cirrus Logic, Inc.
//
// Author: Fred Treven <fred.treven@cirrus.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation.

#include <linux/mfd/cs40l26.h>

#ifdef CONFIG_DEBUG_FS
static ssize_t cs40l26_fw_ctrl_name_read(struct file *file, char __user *user_buf, size_t count,
		loff_t *ppos)
{
	struct cs40l26_private *cs40l26 = file->private_data;
	ssize_t error = 0;

	mutex_lock(&cs40l26->lock);

	if (cs40l26->dbg_fw_ctrl_name)
		error = simple_read_from_buffer(user_buf, count, ppos, cs40l26->dbg_fw_ctrl_name,
				strlen(cs40l26->dbg_fw_ctrl_name));

	mutex_unlock(&cs40l26->lock);

	return error;
}

static ssize_t cs40l26_fw_ctrl_name_write(struct file *file, const char __user *user_buf,
		size_t count, loff_t *ppos)
{
	struct cs40l26_private *cs40l26 = file->private_data;
	ssize_t error = 0;

	mutex_lock(&cs40l26->lock);

	kfree(cs40l26->dbg_fw_ctrl_name);
	cs40l26->dbg_fw_ctrl_name = NULL;

	cs40l26->dbg_fw_ctrl_name = kzalloc(count, GFP_KERNEL);
	if (!cs40l26->dbg_fw_ctrl_name) {
		error = -ENOMEM;
		goto err_mutex;
	}

	error = simple_write_to_buffer(cs40l26->dbg_fw_ctrl_name, count, ppos, user_buf, count);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	return error ? error : count;
}

static ssize_t cs40l26_fw_algo_id_read(struct file *file, char __user *user_buf, size_t count,
		loff_t *ppos)
{
	struct cs40l26_private *cs40l26 = file->private_data;
	ssize_t error;
	char *str;

	str = kzalloc(CS40L26_ALGO_ID_MAX_STR_LEN, GFP_KERNEL);
	if (!str)
		return -ENOMEM;

	mutex_lock(&cs40l26->lock);

	snprintf(str, count, "0x%06X\n", cs40l26->dbg_fw_algo_id);

	mutex_unlock(&cs40l26->lock);

	error = simple_read_from_buffer(user_buf, count, ppos, str, strlen(str));

	kfree(str);

	return error;
}

static ssize_t cs40l26_fw_algo_id_write(struct file *file, const char __user *user_buf,
		size_t count, loff_t *ppos)
{
	struct cs40l26_private *cs40l26 = file->private_data;
	ssize_t error;
	char *str;
	u32 val;

	str = kzalloc(count, GFP_KERNEL);
	if (!str)
		return -ENOMEM;

	simple_write_to_buffer(str, count, ppos, user_buf, count);

	error = kstrtou32(str, 16, &val);
	if (error)
		goto exit_free;

	mutex_lock(&cs40l26->lock);

	cs40l26->dbg_fw_algo_id = val;

	mutex_unlock(&cs40l26->lock);

exit_free:

	kfree(str);

	return error ? error : count;
}

static ssize_t cs40l26_fw_ctrl_val_read(struct file *file, char __user *user_buf, size_t count,
		loff_t *ppos)
{
	struct cs40l26_private *cs40l26 = file->private_data;
	u32 reg, val, mem_type;
	char *result, *input;
	ssize_t error;

	if (!cs40l26->dbg_fw_ctrl_name || !cs40l26->dbg_fw_algo_id)
		return -ENODEV;

	if (strlen(cs40l26->dbg_fw_ctrl_name) == 0)
		return -ENODATA;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	mem_type  = cs40l26->dbg_fw_ym ? CL_DSP_YM_UNPACKED_TYPE : CL_DSP_XM_UNPACKED_TYPE;

	input = kzalloc(strlen(cs40l26->dbg_fw_ctrl_name), GFP_KERNEL);
	if (!input) {
		error = -ENOMEM;
		goto err_mutex;
	}

	snprintf(input, strlen(cs40l26->dbg_fw_ctrl_name), "%s", cs40l26->dbg_fw_ctrl_name);

	error = cl_dsp_get_reg(cs40l26->dsp, input, mem_type, cs40l26->dbg_fw_algo_id, &reg);
	kfree(input);
	if (error)
		goto err_mutex;

	error = regmap_read(cs40l26->regmap, reg, &val);
	if (error) {
		dev_err(cs40l26->dev, "Failed to read fw control\n");
		goto err_mutex;
	}

	result = kzalloc(CS40L26_ALGO_ID_MAX_STR_LEN, GFP_KERNEL);
	if (!result) {
		error = -ENOMEM;
		goto err_mutex;
	}

	snprintf(result, CS40L26_ALGO_ID_MAX_STR_LEN, "0x%08X\n", val);
	error = simple_read_from_buffer(user_buf, count, ppos, result, strlen(result));

	kfree(result);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error;
}

static ssize_t cs40l26_power_on_seq_read(struct file *file, char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct cs40l26_private *cs40l26 = file->private_data;
	char *pseq_str = NULL;
	char str[CS40L26_PSEQ_STR_LINE_LEN];
	ssize_t error, pseq_str_size;
	struct cs40l26_pseq_op *op;
	u32 addr, data;

	mutex_lock(&cs40l26->lock);

	if (list_empty(&cs40l26->pseq_op_head) || cs40l26->pseq_num_ops <= 0) {
		dev_err(cs40l26->dev, "Power on sequence is empty\n");
		error = -ENODATA;
		goto err_mutex;
	}

	pseq_str_size = CS40L26_PSEQ_STR_LINE_LEN * cs40l26->pseq_num_ops;
	pseq_str = kzalloc(pseq_str_size, GFP_KERNEL);
	if (!pseq_str) {
		error = -ENOMEM;
		goto err_mutex;
	}

	list_for_each_entry_reverse(op, &cs40l26->pseq_op_head, list) {
		switch (op->operation) {
		case CS40L26_PSEQ_OP_WRITE_FULL:
			addr = CS40L26_PSEQ_FULL_ADDR_GET(op->words[0], op->words[1]);
			data = CS40L26_PSEQ_FULL_DATA_GET(op->words[1], op->words[2]);
			break;
		case CS40L26_PSEQ_OP_WRITE_H16:
		case CS40L26_PSEQ_OP_WRITE_L16:
			addr = CS40L26_PSEQ_X16_ADDR_GET(op->words[0], op->words[1]);
			if (op->operation == CS40L26_PSEQ_OP_WRITE_H16)
				data = CS40L26_PSEQ_H16_DATA_GET(op->words[1]);
			else
				data = CS40L26_PSEQ_L16_DATA_GET(op->words[1]);
			break;
		case CS40L26_PSEQ_OP_WRITE_ADDR8:
			addr = CS40L26_PSEQ_ADDR8_ADDR_GET(op->words[0]);
			data = CS40L26_PSEQ_ADDR8_DATA_GET(op->words[0], op->words[1]);
			break;
		case CS40L26_PSEQ_OP_END:
			addr = CS40L26_PSEQ_OP_END_ADDR;
			data = CS40L26_PSEQ_OP_END_DATA;
			break;
		default:
			dev_err(cs40l26->dev, "Unrecognized Op Code: 0x%02X\n", op->operation);
			error = -EINVAL;
			goto err_mutex;
		}

		error = snprintf(str, CS40L26_PSEQ_STR_LINE_LEN,
				"0x%08X: code = 0x%02X, Addr = 0x%08X, Data = 0x%08X\n",
				cs40l26->pseq_base + op->offset, op->operation, addr, data);
		if (error <= 0) {
			error = -EINVAL;
			goto err_mutex;
		}

		strncat(pseq_str, str, CS40L26_PSEQ_STR_LINE_LEN);
	}

	error = simple_read_from_buffer(user_buf, count, ppos, pseq_str, pseq_str_size);

err_mutex:
	kfree(pseq_str);

	mutex_unlock(&cs40l26->lock);

	return error;
}

static ssize_t cs40l26_hw_reg_read(struct file *file, char __user *user_buf, size_t count,
		loff_t *ppos)
{
	struct cs40l26_private *cs40l26 = file->private_data;
	char result[CS40L26_HW_STR_LEN];
	ssize_t error;

	mutex_lock(&cs40l26->lock);

	error = snprintf(result, count, "0x%08X\n", cs40l26->dbg_hw_reg);
	if (error <= 0) {
		error = -EINVAL;
		goto err_mutex;
	}

	error = simple_read_from_buffer(user_buf, count, ppos, result, strlen(result));

err_mutex:
	mutex_unlock(&cs40l26->lock);

	return error;
}

static ssize_t cs40l26_hw_reg_write(struct file *file, const char __user *user_buf,
		size_t count, loff_t *ppos)
{
	struct cs40l26_private *cs40l26 = file->private_data;
	char str[CS40L26_HW_STR_LEN];
	ssize_t error;
	u32 reg;

	memset(str, 0, CS40L26_HW_STR_LEN);

	mutex_lock(&cs40l26->lock);

	simple_write_to_buffer(str, count, ppos, user_buf, count);

	error = kstrtou32(str, CS40L26_HW_STR_LEN, &reg);
	if (error)
		goto err_mutex;

	cs40l26->dbg_hw_reg = reg;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	return error ? error : count;
}

static ssize_t cs40l26_hw_val_read(struct file *file, char __user *user_buf, size_t count,
		loff_t *ppos)
{
	struct cs40l26_private *cs40l26 = file->private_data;
	char result[CS40L26_HW_STR_LEN];
	ssize_t error;
	u32 val;

	if (cs40l26->dbg_hw_reg % CL_DSP_BYTES_PER_WORD) {
		dev_err(cs40l26->dev, "Reg. address 0x%08X not multiple of 4\n",
				cs40l26->dbg_hw_reg);
		return -EINVAL;
	}

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = regmap_read(cs40l26->regmap, cs40l26->dbg_hw_reg, &val);
	if (error)
		goto err_mutex;

	error = snprintf(result, CS40L26_HW_STR_LEN, "0x%08X\n", val);
	if (error <= 0) {
		error = -EINVAL;
		goto err_mutex;
	}

	error = simple_read_from_buffer(user_buf, count, ppos, result, strlen(result));

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error;
}

static ssize_t cs40l26_hw_val_write(struct file *file, const char __user *user_buf,
		size_t count, loff_t *ppos)
{
	struct cs40l26_private *cs40l26 = file->private_data;
	char str[CS40L26_HW_STR_LEN];
	ssize_t error;
	u32 val;

	if (cs40l26->dbg_hw_reg % CL_DSP_BYTES_PER_WORD) {
		dev_err(cs40l26->dev, "Reg. address 0x%08X not multiple of 4\n",
				cs40l26->dbg_hw_reg);
		return -EINVAL;
	}

	memset(str, 0, CS40L26_HW_STR_LEN);

	simple_write_to_buffer(str, count, ppos, user_buf, count);

	error = kstrtou32(str, CS40L26_HW_STR_LEN, &val);
	if (error)
		return error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = regmap_write(cs40l26->regmap, cs40l26->dbg_hw_reg, val);
	if (error)
		goto exit_mutex;

	error = cs40l26_pseq_write(cs40l26, cs40l26->dbg_hw_reg, (val & GENMASK(31, 16)) >> 16,
			true, CS40L26_PSEQ_OP_WRITE_H16);
	if (error)
		goto exit_mutex;

	error = cs40l26_pseq_write(cs40l26, cs40l26->dbg_hw_reg, (val & GENMASK(15, 0)),
			true, CS40L26_PSEQ_OP_WRITE_L16);

exit_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}

static const struct {
	const char *name;
	const struct file_operations fops;
} cs40l26_hw_debugfs_fops[] = {
	{
		.name = "register",
		.fops = {
			.open = simple_open,
			.read = cs40l26_hw_reg_read,
			.write = cs40l26_hw_reg_write,
		},
	},
	{
		.name = "value",
		.fops = {
			.open = simple_open,
			.read = cs40l26_hw_val_read,
			.write = cs40l26_hw_val_write,
		},
	},
};

static const struct {
	const char *name;
	const struct file_operations fops;
} cs40l26_debugfs_fops[] = {
	{
		.name = "fw_ctrl_name",
		.fops = {
			.open = simple_open,
			.read = cs40l26_fw_ctrl_name_read,
			.write = cs40l26_fw_ctrl_name_write,
		},
	},
	{
		.name = "fw_algo_id",
		.fops = {
			.open = simple_open,
			.read = cs40l26_fw_algo_id_read,
			.write = cs40l26_fw_algo_id_write,
		},
	},
	{
		.name = "fw_ctrl_val",
		.fops = {
			.open = simple_open,
			.read = cs40l26_fw_ctrl_val_read,
		},
	},
	{
		.name = "power_on_seq",
		.fops = {
			.open = simple_open,
			.read = cs40l26_power_on_seq_read,
		},
	},
};

static void cs40l26_hw_debugfs_init(struct cs40l26_private *cs40l26)
{
	int i;

	cs40l26->debugfs_hw_node = debugfs_create_dir("hardware", cs40l26->debugfs_root);
	if (IS_ERR_OR_NULL(cs40l26->debugfs_hw_node)) {
		dev_err(cs40l26->dev, "Failed to mount hardware debugfs directory\n");
		kfree(cs40l26->debugfs_hw_node);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(cs40l26_hw_debugfs_fops); i++)
		debugfs_create_file(cs40l26_hw_debugfs_fops[i].name, CL_DSP_DEBUGFS_RW_FILE_MODE,
				cs40l26->debugfs_hw_node, cs40l26,
				&cs40l26_hw_debugfs_fops[i].fops);
}

void cs40l26_debugfs_init(struct cs40l26_private *cs40l26)
{
	struct dentry *root = NULL;
	int i;

	cs40l26_debugfs_cleanup(cs40l26);

	root = debugfs_create_dir("cs40l26", NULL);
	if (!root)
		return;

	debugfs_create_bool("fw_ym_space", CL_DSP_DEBUGFS_RW_FILE_MODE, root, &cs40l26->dbg_fw_ym);

	for (i = 0; i < ARRAY_SIZE(cs40l26_debugfs_fops); i++)
		debugfs_create_file(cs40l26_debugfs_fops[i].name, CL_DSP_DEBUGFS_RW_FILE_MODE,
				root, cs40l26, &cs40l26_debugfs_fops[i].fops);

	cs40l26->dbg_fw_ym = false;
	cs40l26->dbg_fw_algo_id = CS40L26_VIBEGEN_ALGO_ID;
	cs40l26->debugfs_root = root;

	cs40l26_hw_debugfs_init(cs40l26);

	if (cs40l26->fw_id == CS40L26_FW_ID &&
			cl_dsp_algo_is_present(cs40l26->dsp, CS40L26_EVENT_LOGGER_ALGO_ID)) {
		cs40l26->cl_dsp_db = cl_dsp_debugfs_create(cs40l26->dsp, cs40l26->debugfs_root,
				(u32) CS40L26_EVENT_LOGGER_ALGO_ID);

		if (IS_ERR_OR_NULL(cs40l26->cl_dsp_db))
			dev_err(cs40l26->dev, "Failed to create CL DSP Debugfs\n");
	}
}
EXPORT_SYMBOL_GPL(cs40l26_debugfs_init);

void cs40l26_debugfs_cleanup(struct cs40l26_private *cs40l26)
{
	cl_dsp_debugfs_destroy(cs40l26->cl_dsp_db);
	cs40l26->cl_dsp_db = NULL;
	kfree(cs40l26->dbg_fw_ctrl_name);
	cs40l26->dbg_fw_ctrl_name = NULL;
	debugfs_remove_recursive(cs40l26->debugfs_root);
}
EXPORT_SYMBOL_GPL(cs40l26_debugfs_cleanup);

#endif /* CONFIG_DEBUG_FS */
