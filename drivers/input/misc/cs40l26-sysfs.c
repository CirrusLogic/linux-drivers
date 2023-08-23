// SPDX-License-Identifier: GPL-2.0
//
// cs40l26-sysfs.c -- CS40L26 Boosted Haptic Driver with Integrated DSP and
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

static ssize_t dsp_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u8 dsp_state;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = cs40l26_dsp_state_get(cs40l26, &dsp_state);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;
	else
		return snprintf(buf, PAGE_SIZE, "%u\n", (unsigned int) (dsp_state & 0xFF));
}
static DEVICE_ATTR_RO(dsp_state);

static ssize_t owt_lib_compat_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "1.0.0\n");
}
static DEVICE_ATTR_RO(owt_lib_compat);

static ssize_t overprotection_gain_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, op_gain;
	int error;

	if (!cl_dsp_algo_is_present(cs40l26->dsp, CS40L26_EP_ALGO_ID))
		return -EPERM;

	error = cl_dsp_get_reg(cs40l26->dsp, "PROTECTION_XM_OP_GAIN",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_EP_ALGO_ID, &reg);
	if (error)
		return error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = regmap_read(cs40l26->regmap, reg, &op_gain);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : snprintf(buf, PAGE_SIZE, "%d\n", op_gain);
}

static ssize_t overprotection_gain_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, op_gain;
	int error;

	if (!cl_dsp_algo_is_present(cs40l26->dsp, CS40L26_EP_ALGO_ID))
		return -EPERM;

	error = kstrtou32(buf, 10, &op_gain);

	if (error || op_gain < CS40L26_OVERPROTECTION_GAIN_MIN ||
			op_gain > CS40L26_UINT_24_BITS_MAX)
		return -EINVAL;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = cl_dsp_get_reg(cs40l26->dsp, "PROTECTION_XM_OP_GAIN",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_EP_ALGO_ID, &reg);
	if (error)
		goto err_pm;

	error = regmap_write(cs40l26->regmap, reg, op_gain);

err_pm:
	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(overprotection_gain);

static ssize_t halo_heartbeat_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, halo_heartbeat;
	int error;

	error = cl_dsp_get_reg(cs40l26->dsp, "HALO_HEARTBEAT", CL_DSP_XM_UNPACKED_TYPE,
			cs40l26->fw_id, &reg);
	if (error)
		return error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = regmap_read(cs40l26->regmap, reg, &halo_heartbeat);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;

	return snprintf(buf, PAGE_SIZE, "%d\n", halo_heartbeat);
}
static DEVICE_ATTR_RO(halo_heartbeat);

static ssize_t pm_stdby_timeout_ms_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 timeout_ms;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = cs40l26_pm_timeout_ms_get(cs40l26, CS40L26_DSP_STATE_STANDBY, &timeout_ms);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;

	return snprintf(buf, PAGE_SIZE, "%u\n", timeout_ms);
}

static ssize_t pm_stdby_timeout_ms_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 timeout_ms;
	int error;

	error = kstrtou32(buf, 10, &timeout_ms);
	if (error)
		return -EINVAL;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = cs40l26_pm_timeout_ms_set(cs40l26, CS40L26_DSP_STATE_STANDBY, timeout_ms);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;

	return count;
}
static DEVICE_ATTR_RW(pm_stdby_timeout_ms);

static ssize_t pm_active_timeout_ms_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 timeout_ms;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = cs40l26_pm_timeout_ms_get(cs40l26, CS40L26_DSP_STATE_ACTIVE, &timeout_ms);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;

	return snprintf(buf, PAGE_SIZE, "%u\n", timeout_ms);
}

static ssize_t pm_active_timeout_ms_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 timeout_ms;
	int error;

	error = kstrtou32(buf, 10, &timeout_ms);
	if (error)
		return -EINVAL;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = cs40l26_pm_timeout_ms_set(cs40l26, CS40L26_DSP_STATE_ACTIVE, timeout_ms);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;

	return count;
}
static DEVICE_ATTR_RW(pm_active_timeout_ms);

static ssize_t vibe_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int state;

	if (!cs40l26->vibe_state_reporting)  {
		dev_err(cs40l26->dev, "vibe_state not supported\n");
		return -EPERM;
	}

	mutex_lock(&cs40l26->lock);
	state = cs40l26->vibe_state;
	mutex_unlock(&cs40l26->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", state);
}
static DEVICE_ATTR_RO(vibe_state);

static ssize_t power_on_seq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct cs40l26_pseq_op *op;
	u32 addr, data, base;
	int error;

	mutex_lock(&cs40l26->lock);

	base = cs40l26->pseq_base;

	if (list_empty(&cs40l26->pseq_op_head)) {
		dev_err(cs40l26->dev, "Power on sequence is empty\n");
		error = -EINVAL;
		goto err_mutex;
	}

	list_for_each_entry_reverse(op, &cs40l26->pseq_op_head, list) {
		switch (op->operation) {
		case CS40L26_PSEQ_OP_WRITE_FULL:
			addr = ((op->words[0] & 0xFFFF) << 16) | ((op->words[1] & 0x00FFFF00) >> 8);
			data = ((op->words[1] & 0xFF) << 24) | (op->words[2] & 0xFFFFFF);
			break;
		case CS40L26_PSEQ_OP_WRITE_H16:
		case CS40L26_PSEQ_OP_WRITE_L16:
			addr = ((op->words[0] & 0xFFFF) << 8) | ((op->words[1] & 0xFF0000) >> 16);
			data = (op->words[1] & 0xFFFF);

			if (op->operation == CS40L26_PSEQ_OP_WRITE_H16)
				data <<= 16;
			break;
		case CS40L26_PSEQ_OP_WRITE_ADDR8:
			addr = (op->words[0] & 0xFF00) >> 8;
			data = ((op->words[0] & 0xFF) << 24) | (op->words[1] & 0xFFFFFF);
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

		dev_dbg(cs40l26->dev, "0x%08x: code = 0x%02X, Addr = 0x%08X, Data = 0x%08X\n",
				base + op->offset, op->operation, addr, data);
	}

	error = snprintf(buf, PAGE_SIZE, "%d\n", cs40l26->pseq_num_ops);

err_mutex:
	mutex_unlock(&cs40l26->lock);
	return error;
}
static DEVICE_ATTR_RO(power_on_seq);

static ssize_t owt_free_space_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, words;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = cl_dsp_get_reg(cs40l26->dsp, "OWT_SIZE_XM",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (error)
		goto err_pm;

	error = regmap_read(cs40l26->regmap, reg, &words);
	if (error) {
		dev_err(cs40l26->dev, "Failed to get remaining OWT space\n");
		goto err_pm;
	}

	error = snprintf(buf, PAGE_SIZE, "%d\n", words * CL_DSP_BYTES_PER_WORD);

err_pm:
	cs40l26_pm_exit(cs40l26->dev);

	return error;
}
static DEVICE_ATTR_RO(owt_free_space);

static ssize_t die_temp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct regmap *regmap = cs40l26->regmap;
	u16 die_temp;
	int error;
	u32 val;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = regmap_read(regmap, CS40L26_GLOBAL_ENABLES, &val);
	if (error) {
		dev_err(cs40l26->dev, "Failed to read GLOBAL_EN status\n");
		goto err_pm;
	}

	if (!(val & CS40L26_GLOBAL_EN_MASK)) {
		dev_err(cs40l26->dev, "Global enable must be set to get die temp.\n");
		error = -EPERM;
		goto err_pm;
	}

	error = regmap_read(regmap, CS40L26_ENABLES_AND_CODES_DIG, &val);
	if (error) {
		dev_err(cs40l26->dev, "Failed to get die temperature\n");
		goto err_pm;
	}

	die_temp = (val & CS40L26_TEMP_RESULT_FILT_MASK) >> CS40L26_TEMP_RESULT_FILT_SHIFT;

	error = snprintf(buf, PAGE_SIZE, "0x%03X\n", die_temp);

err_pm:
	cs40l26_pm_exit(cs40l26->dev);

	return error;
}
static DEVICE_ATTR_RO(die_temp);

static ssize_t num_waves_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 nwaves;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = cs40l26_get_num_waves(cs40l26, &nwaves);
	if (error)
		goto err_pm;

	error = snprintf(buf, PAGE_SIZE, "%u\n", nwaves);

err_pm:
	cs40l26_pm_exit(cs40l26->dev);

	return error;
}
static DEVICE_ATTR_RO(num_waves);

/* boost_disable_delay is in units of 125us, e.g. 8 ->  1ms */
static ssize_t boost_disable_delay_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, boost_disable_delay;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = cl_dsp_get_reg(cs40l26->dsp, "BOOST_DISABLE_DELAY",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_EXT_ALGO_ID, &reg);
	if (error)
		goto err_pm;

	error = regmap_read(cs40l26->regmap, reg, &boost_disable_delay);
	if (error)
		goto err_pm;

	error = snprintf(buf, PAGE_SIZE, "%d\n", boost_disable_delay);

err_pm:
	cs40l26_pm_exit(cs40l26->dev);

	return error;
}

static ssize_t boost_disable_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, boost_disable_delay;
	int error;

	dev_dbg(cs40l26->dev, "%s: %s", __func__, buf);

	error = kstrtou32(buf, 10, &boost_disable_delay);

	if (error || boost_disable_delay < CS40L26_BOOST_DISABLE_DELAY_MIN ||
			boost_disable_delay > CS40L26_BOOST_DISABLE_DELAY_MAX)
		return -EINVAL;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = cl_dsp_get_reg(cs40l26->dsp, "BOOST_DISABLE_DELAY",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_EXT_ALGO_ID, &reg);
	if (error)
		goto err_pm;

	error = regmap_write(cs40l26->regmap, reg, boost_disable_delay);

err_pm:
	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;
	else
		return count;
}
static DEVICE_ATTR_RW(boost_disable_delay);

static ssize_t f0_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int reg, val;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "F0_OFFSET", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (error)
		goto err_mutex;


	error = regmap_read(cs40l26->regmap, reg, &val);
	if (error)
		goto err_mutex;

	error = snprintf(buf, PAGE_SIZE, "%u\n", val);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error;
}

static ssize_t f0_offset_store(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int reg, val;
	int error;

	error = kstrtou32(buf, 10, &val);
	if (error)
		return -EINVAL;

	if (val > CS40L26_F0_OFFSET_MAX && val < CS40L26_F0_OFFSET_MIN)
		return -EINVAL;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "F0_OFFSET", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_write(cs40l26->regmap, reg, val);
	if (error)
		goto err_mutex;

	error = count;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error;
}
static DEVICE_ATTR_RW(f0_offset);

static ssize_t delay_before_stop_playback_us_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;

	mutex_lock(&cs40l26->lock);

	error = snprintf(buf, PAGE_SIZE, "%d\n", cs40l26->delay_before_stop_playback_us);

	mutex_unlock(&cs40l26->lock);

	return error;
}

static ssize_t delay_before_stop_playback_us_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 val;
	int error;

	error = kstrtou32(buf, 10, &val);
	if (error)
		return -EINVAL;

	mutex_lock(&cs40l26->lock);

	cs40l26->delay_before_stop_playback_us = val;

	mutex_unlock(&cs40l26->lock);

	return count;
}
static DEVICE_ATTR_RW(delay_before_stop_playback_us);

static ssize_t f0_comp_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;

	mutex_lock(&cs40l26->lock);

	if (cs40l26->fw_id == CS40L26_FW_CALIB_ID) {
		error = -EPERM;
		goto err_mutex;
	}

	if (cs40l26->comp_enable_pend) {
		error = -EIO;
		goto err_mutex;
	}

	error = snprintf(buf, PAGE_SIZE, "%d\n", cs40l26->comp_enable_f0);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	return error;
}

static ssize_t f0_comp_enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;
	unsigned int val;
	u32 reg, value;

	error = kstrtou32(buf, 10, &val);
	if (error)
		return -EINVAL;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	cs40l26->comp_enable_pend = true;
	cs40l26->comp_enable_f0 = val > 0;

	value = (cs40l26->comp_enable_redc << CS40L26_COMP_EN_REDC_SHIFT) |
			(cs40l26->comp_enable_f0 << CS40L26_COMP_EN_F0_SHIFT);

	if (cs40l26->fw_id == CS40L26_FW_CALIB_ID) {
		error = -EPERM;
	} else {
		error = cl_dsp_get_reg(cs40l26->dsp, "COMPENSATION_ENABLE", CL_DSP_XM_UNPACKED_TYPE,
				CS40L26_VIBEGEN_ALGO_ID, &reg);
		if (error)
			goto err_mutex;

		error = regmap_write(cs40l26->regmap, reg, value);
	}

	if (error)
		goto err_mutex;

	error = count;

err_mutex:
	cs40l26->comp_enable_pend = false;
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error;
}
static DEVICE_ATTR_RW(f0_comp_enable);

static ssize_t redc_comp_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;

	mutex_lock(&cs40l26->lock);

	if (cs40l26->fw_id == CS40L26_FW_CALIB_ID) {
		error = -EPERM;
		goto err_mutex;
	}

	if (cs40l26->comp_enable_pend) {
		error = -EIO;
		goto err_mutex;
	}

	error = snprintf(buf, PAGE_SIZE, "%d\n", cs40l26->comp_enable_redc);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	return error;
}

static ssize_t redc_comp_enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;
	unsigned int val;
	u32 reg, value;

	error = kstrtou32(buf, 10, &val);
	if (error)
		return -EINVAL;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	cs40l26->comp_enable_pend = true;
	cs40l26->comp_enable_redc = val > 0;

	value = (cs40l26->comp_enable_redc << CS40L26_COMP_EN_REDC_SHIFT) |
			(cs40l26->comp_enable_f0 << CS40L26_COMP_EN_F0_SHIFT);

	if (cs40l26->fw_id == CS40L26_FW_CALIB_ID) {
		error = -EPERM;
	} else {
		error = cl_dsp_get_reg(cs40l26->dsp, "COMPENSATION_ENABLE", CL_DSP_XM_UNPACKED_TYPE,
				CS40L26_VIBEGEN_ALGO_ID, &reg);
		if (error)
			goto err_mutex;

		error = regmap_write(cs40l26->regmap, reg, value);
	}

	if (error)
		goto err_mutex;

	error = count;

err_mutex:
	cs40l26->comp_enable_pend = false;
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error;
}
static DEVICE_ATTR_RW(redc_comp_enable);

static ssize_t swap_firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;

	mutex_lock(&cs40l26->lock);

	if (cs40l26->fw_id == CS40L26_FW_ID)
		error = snprintf(buf, PAGE_SIZE, "%d\n", 0);
	else if (cs40l26->fw_id == CS40L26_FW_CALIB_ID)
		error = snprintf(buf, PAGE_SIZE, "%d\n", 1);
	else
		error = -EINVAL;

	mutex_unlock(&cs40l26->lock);

	return error;
}

static ssize_t swap_firmware_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;
	unsigned int variant;

	error = kstrtou32(buf, 10, &variant);
	if (error)
		return error;

	if (variant == 0)
		error = cs40l26_fw_swap(cs40l26, CS40L26_FW_ID);
	else if (variant == 1)
		error = cs40l26_fw_swap(cs40l26, CS40L26_FW_CALIB_ID);
	else
		error = -EINVAL;

	return error ? error : count;
}
static DEVICE_ATTR_RW(swap_firmware);

static struct attribute *cs40l26_dev_attrs[] = {
	&dev_attr_num_waves.attr,
	&dev_attr_die_temp.attr,
	&dev_attr_owt_free_space.attr,
	&dev_attr_power_on_seq.attr,
	&dev_attr_dsp_state.attr,
	&dev_attr_halo_heartbeat.attr,
	&dev_attr_pm_stdby_timeout_ms.attr,
	&dev_attr_pm_active_timeout_ms.attr,
	&dev_attr_vibe_state.attr,
	&dev_attr_boost_disable_delay.attr,
	&dev_attr_f0_offset.attr,
	&dev_attr_delay_before_stop_playback_us.attr,
	&dev_attr_f0_comp_enable.attr,
	&dev_attr_redc_comp_enable.attr,
	&dev_attr_swap_firmware.attr,
	&dev_attr_owt_lib_compat.attr,
	&dev_attr_overprotection_gain.attr,
	NULL,
};

struct attribute_group cs40l26_dev_attr_group = {
	.name = "default",
	.attrs = cs40l26_dev_attrs,
};

static ssize_t dbc_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 val, reg;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "FLAGS", CL_DSP_XM_UNPACKED_TYPE, CS40L26_EXT_ALGO_ID,
			&reg);
	if (error)
		goto err_pm;

	error = regmap_read(cs40l26->regmap, reg, &val);
	if (error) {
		dev_err(cs40l26->dev, "Failed to get FLAGS\n");
		goto err_pm;
	}


	val &= CS40L26_DBC_ENABLE_MASK;
	val >>= CS40L26_DBC_ENABLE_SHIFT;

	error = snprintf(buf, PAGE_SIZE, "%u\n", val);

err_pm:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error;
}

static ssize_t dbc_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;
	u32 val;

	error = kstrtou32(buf, 10, &val);
	if (error)
		return error;

	if (val > 1)
		return -EINVAL;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dbc_enable(cs40l26, val);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(dbc_enable);

static ssize_t dbc_env_rel_coef_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int error;

	error = cs40l26_dbc_get(cs40l26, CS40L26_DBC_ENV_REL_COEF, &val);

	return error ? error : snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t dbc_env_rel_coef_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct device *cdev = cs40l26->dev;
	u32 val;
	int error;

	error = kstrtou32(buf, 10, &val);
	if (error)
		return error;

	error = cs40l26_pm_enter(cdev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dbc_set(cs40l26, CS40L26_DBC_ENV_REL_COEF, val);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cdev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(dbc_env_rel_coef);

static ssize_t dbc_rise_headroom_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int error;

	error = cs40l26_dbc_get(cs40l26, CS40L26_DBC_RISE_HEADROOM, &val);

	return error ? error : snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t dbc_rise_headroom_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct device *cdev = cs40l26->dev;
	u32 val;
	int error;

	error = kstrtou32(buf, 10, &val);
	if (error)
		return error;

	error = cs40l26_pm_enter(cdev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dbc_set(cs40l26, CS40L26_DBC_RISE_HEADROOM, val);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cdev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(dbc_rise_headroom);

static ssize_t dbc_fall_headroom_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int error;

	error = cs40l26_dbc_get(cs40l26, CS40L26_DBC_FALL_HEADROOM, &val);

	return error ? error : snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t dbc_fall_headroom_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct device *cdev = cs40l26->dev;
	u32 val;
	int error;

	error = kstrtou32(buf, 10, &val);
	if (error)
		return error;

	error = cs40l26_pm_enter(cdev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dbc_set(cs40l26, CS40L26_DBC_FALL_HEADROOM, val);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cdev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(dbc_fall_headroom);

static ssize_t dbc_tx_lvl_thresh_fs_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int error;

	error = cs40l26_dbc_get(cs40l26, CS40L26_DBC_TX_LVL_THRESH_FS, &val);

	return error ? error : snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t dbc_tx_lvl_thresh_fs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct device *cdev = cs40l26->dev;
	u32 val;
	int error;

	error = kstrtou32(buf, 10, &val);
	if (error)
		return error;

	error = cs40l26_pm_enter(cdev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dbc_set(cs40l26, CS40L26_DBC_TX_LVL_THRESH_FS, val);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cdev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(dbc_tx_lvl_thresh_fs);

static ssize_t dbc_tx_lvl_hold_off_ms_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int error;

	error = cs40l26_dbc_get(cs40l26, CS40L26_DBC_TX_LVL_HOLD_OFF_MS, &val);

	return error ? error : snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t dbc_tx_lvl_hold_off_ms_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct device *cdev = cs40l26->dev;
	u32 val;
	int error;

	error = kstrtou32(buf, 10, &val);
	if (error)
		return error;

	error = cs40l26_pm_enter(cdev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dbc_set(cs40l26, CS40L26_DBC_TX_LVL_HOLD_OFF_MS, val);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cdev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(dbc_tx_lvl_hold_off_ms);

static struct attribute *cs40l26_dev_attrs_dbc[] = {
	&dev_attr_dbc_enable.attr,
	&dev_attr_dbc_env_rel_coef.attr,
	&dev_attr_dbc_rise_headroom.attr,
	&dev_attr_dbc_fall_headroom.attr,
	&dev_attr_dbc_tx_lvl_thresh_fs.attr,
	&dev_attr_dbc_tx_lvl_hold_off_ms.attr,
	NULL,
};

struct attribute_group cs40l26_dev_attr_dbc_group = {
	.name = "dbc",
	.attrs = cs40l26_dev_attrs_dbc,
};

static ssize_t trigger_calibration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 mailbox_command, calibration_request_payload;
	int error;
	struct completion *completion;

	dev_dbg(cs40l26->dev, "%s: %s", __func__, buf);

	if (!cs40l26->calib_fw) {
		dev_err(cs40l26->dev, "Must use calibration firmware\n");
		return -EPERM;
	}

	error = kstrtou32(buf, 16, &calibration_request_payload);
	if (error)
		return -EINVAL;

	switch (calibration_request_payload) {
	case CS40L26_CALIBRATION_CONTROL_REQUEST_F0_AND_Q:
		completion = &cs40l26->cal_f0_cont;
		break;
	case CS40L26_CALIBRATION_CONTROL_REQUEST_REDC:
		completion = &cs40l26->cal_redc_cont;
		break;
	case CS40L26_CALIBRATION_CONTROL_REQUEST_DVL_PEQ:
		completion = &cs40l26->cal_dvl_peq_cont;
		break;
	case CS40L26_CALIBRATION_CONTROL_REQUEST_LS_CALIBRATION:
		completion = &cs40l26->cal_ls_cont;
		break;
	default:
		return -EINVAL;
	}

	mailbox_command = ((CS40L26_DSP_MBOX_CMD_INDEX_CALIBRATION_CONTROL <<
			CS40L26_DSP_MBOX_CMD_INDEX_SHIFT) & CS40L26_DSP_MBOX_CMD_INDEX_MASK) |
			(calibration_request_payload & CS40L26_DSP_MBOX_CMD_PAYLOAD_MASK);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);
	reinit_completion(completion);

	error = cs40l26_mailbox_write(cs40l26, mailbox_command);

	mutex_unlock(&cs40l26->lock);

	if (error) {
		dev_err(cs40l26->dev, "Failed to request calibration\n");
		goto err_pm;
	}

	if (!wait_for_completion_timeout(
			completion,
			msecs_to_jiffies(CS40L26_CALIBRATION_TIMEOUT_MS))) {
		error = -ETIME;
		dev_err(cs40l26->dev, "Failed to complete cal req, %d, err: %d",
				calibration_request_payload, error);
		goto err_pm;
	}

	mutex_lock(&cs40l26->lock);

	if (calibration_request_payload == CS40L26_CALIBRATION_CONTROL_REQUEST_F0_AND_Q)
		error = cs40l26_copy_f0_est_to_dvl(cs40l26);

	mutex_unlock(&cs40l26->lock);
err_pm:
	cs40l26_pm_exit(cs40l26->dev);
	return error ? error : count;
}
static DEVICE_ATTR_WO(trigger_calibration);

static ssize_t f0_measured_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, f0_measured;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "F0_EST", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_read(cs40l26->regmap, reg, &f0_measured);
	if (error)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;
	else
		return snprintf(buf, PAGE_SIZE, "%08X\n", f0_measured);
}
static DEVICE_ATTR_RO(f0_measured);

static ssize_t q_measured_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, q_measured;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "Q_EST", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_read(cs40l26->regmap, reg, &q_measured);
	if (error)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;
	else
		return snprintf(buf, PAGE_SIZE, "%08X\n", q_measured);
}
static DEVICE_ATTR_RO(q_measured);

static ssize_t redc_measured_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, redc_measured;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "RE_EST_STATUS", CL_DSP_YM_UNPACKED_TYPE,
			CS40L26_SVC_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_read(cs40l26->regmap, reg, &redc_measured);
	if (error)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;
	else
		return snprintf(buf, PAGE_SIZE, "%08X\n", redc_measured);
}
static DEVICE_ATTR_RO(redc_measured);

static ssize_t redc_est_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, redc_est;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "REDC", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_read(cs40l26->regmap, reg, &redc_est);
	if (error)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;
	else
		return snprintf(buf, PAGE_SIZE, "%08X\n", redc_est);
}

static ssize_t redc_est_store(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, redc_est;
	int error;

	dev_dbg(cs40l26->dev, "%s: %s", __func__, buf);

	error = kstrtou32(buf, 16, &redc_est);
	if (error)
		return error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "REDC", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_write(cs40l26->regmap, reg, redc_est);
	if (error)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;
	else
		return count;
}
static DEVICE_ATTR_RW(redc_est);

static ssize_t f0_stored_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, f0_stored;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "F0_OTP_STORED", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_read(cs40l26->regmap, reg, &f0_stored);
	if (error)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;
	else
		return snprintf(buf, PAGE_SIZE, "%08X\n", f0_stored);
}

static ssize_t f0_stored_store(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, f0_stored;
	int error;

	dev_dbg(cs40l26->dev, "%s: %s", __func__, buf);

	error = kstrtou32(buf, 16, &f0_stored);

	if (error || f0_stored < CS40L26_F0_EST_MIN || f0_stored > CS40L26_F0_EST_MAX)
		return -EINVAL;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "F0_OTP_STORED", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_write(cs40l26->regmap, reg, f0_stored);
	if (error)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;
	else
		return count;
}
static DEVICE_ATTR_RW(f0_stored);

static ssize_t q_stored_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, q_stored;
	int error;

	if (cs40l26->revid == CS40L26_REVID_B2) {
		dev_err(cs40l26->dev, "q_stored not support for revision %02X\n", cs40l26->revid);
		return -EPERM;
	}

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "Q_STORED", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_read(cs40l26->regmap, reg, &q_stored);
	if (error)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;
	else
		return snprintf(buf, PAGE_SIZE, "%08X\n", q_stored);
}

static ssize_t q_stored_store(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, q_stored;
	int error;

	if (cs40l26->revid == CS40L26_REVID_B2) {
		dev_err(cs40l26->dev, "q_stored not support for revision %02X\n", cs40l26->revid);
		return -EPERM;
	}

	dev_dbg(cs40l26->dev, "%s: %s", __func__, buf);

	error = kstrtou32(buf, 16, &q_stored);

	if (error || q_stored < CS40L26_Q_EST_MIN || q_stored > CS40L26_Q_EST_MAX)
		return -EINVAL;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "Q_STORED", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_write(cs40l26->regmap, reg, q_stored);
	if (error)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;
	else
		return count;
}
static DEVICE_ATTR_RW(q_stored);

static ssize_t redc_stored_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, redc_stored;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "REDC_OTP_STORED", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_read(cs40l26->regmap, reg, &redc_stored);
	if (error)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;
	else
		return snprintf(buf, PAGE_SIZE, "%08X\n", redc_stored);
}

static ssize_t redc_stored_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, redc_stored;
	int error;

	dev_dbg(cs40l26->dev, "%s: %s", __func__, buf);

	error = kstrtou32(buf, 16, &redc_stored);
	if (error)
		return error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "REDC_OTP_STORED", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_write(cs40l26->regmap, reg, redc_stored);
	if (error)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;
	else
		return count;
}
static DEVICE_ATTR_RW(redc_stored);

static ssize_t freq_centre_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 freq_centre, reg;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "FREQ_CENTRE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_read(cs40l26->regmap, reg, &freq_centre);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : snprintf(buf, PAGE_SIZE, "%08X\n", freq_centre);
}

static ssize_t freq_centre_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 freq_centre, reg;
	int error;

	error = kstrtou32(buf, 16, &freq_centre);
	if (error)
		return error;

	if (freq_centre < CS40L26_F0_FREQ_CENTRE_MIN ||
			freq_centre > CS40L26_F0_FREQ_CENTRE_MAX)
		return -EINVAL;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "FREQ_CENTRE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_write(cs40l26->regmap, reg, freq_centre);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(freq_centre);

static ssize_t freq_span_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error, freq_span;
	u32 reg;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "FREQ_SPAN", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_read(cs40l26->regmap, reg, &freq_span);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : snprintf(buf, PAGE_SIZE, "%08X\n", freq_span);
}

static ssize_t freq_span_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error, s_freq_span;
	u32 freq_span, reg;

	error = kstrtou32(buf, 16, &freq_span);
	if (error)
		return error;

	freq_span &= GENMASK(23, 0);
	s_freq_span = (freq_span & BIT(23)) ? (freq_span | GENMASK(31, 24)) : freq_span;

	if (abs(s_freq_span) < CS40L26_F0_FREQ_SPAN_MIN ||
			abs(s_freq_span) > CS40L26_F0_FREQ_SPAN_MAX)
		return -EINVAL;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "FREQ_SPAN", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_write(cs40l26->regmap, reg, freq_span);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(freq_span);

static ssize_t f0_and_q_cal_time_ms_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, tone_dur_ms, freq_centre, freq_span;
	int error, f0_and_q_cal_time_ms;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "TONE_DURATION_MS", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_read(cs40l26->regmap, reg, &tone_dur_ms);
	if (error) {
		dev_err(cs40l26->dev, "Failed to get tone duration\n");
		goto err_mutex;
	}

	if (tone_dur_ms == 0) { /* Calculate value */
		error = cl_dsp_get_reg(cs40l26->dsp, "FREQ_SPAN", CL_DSP_XM_UNPACKED_TYPE,
				CS40L26_F0_EST_ALGO_ID, &reg);
		if (error)
			goto err_mutex;

		error = regmap_read(cs40l26->regmap, reg, &freq_span);
		if (error) {
			dev_err(cs40l26->dev, "Failed to get FREQ_SPAN\n");
			goto err_mutex;
		}

		error = cl_dsp_get_reg(cs40l26->dsp, "FREQ_CENTRE", CL_DSP_XM_UNPACKED_TYPE,
				CS40L26_F0_EST_ALGO_ID, &reg);
		if (error)
			goto err_mutex;

		error = regmap_read(cs40l26->regmap, reg, &freq_centre);
		if (error) {
			dev_err(cs40l26->dev, "Failed to get FREQ_CENTRE\n");
			goto err_mutex;
		}

		f0_and_q_cal_time_ms = ((CS40L26_F0_CHIRP_DURATION_FACTOR *
				(int) (freq_span >> CS40L26_F0_EST_FREQ_FRAC_BITS)) /
				(int) (freq_centre >> CS40L26_F0_EST_FREQ_FRAC_BITS));
	} else if (tone_dur_ms < CS40L26_F0_AND_Q_CALIBRATION_MIN_MS) {
		f0_and_q_cal_time_ms = CS40L26_F0_AND_Q_CALIBRATION_MIN_MS;
	} else if (tone_dur_ms > CS40L26_F0_AND_Q_CALIBRATION_MAX_MS) {
		f0_and_q_cal_time_ms = CS40L26_F0_AND_Q_CALIBRATION_MAX_MS;
	} else {
		f0_and_q_cal_time_ms = tone_dur_ms;
	}

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", f0_and_q_cal_time_ms);
}
static DEVICE_ATTR_RO(f0_and_q_cal_time_ms);

static ssize_t redc_cal_time_ms_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* FIRMWARE_STUMPY_CALIB_REDC_PLAYTIME_MS + SVC_INIT + buffer */
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, redc_playtime_ms, redc_total_cal_time_ms;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "REDC_PLAYTIME_MS", CL_DSP_XM_UNPACKED_TYPE,
			cs40l26->fw_id, &reg);
	if (error)
		goto err_mutex;

	error = regmap_read(cs40l26->regmap, reg, &redc_playtime_ms);

	redc_total_cal_time_ms = redc_playtime_ms + CS40L26_SVC_INITIALIZATION_PERIOD_MS +
			CS40L26_REDC_CALIBRATION_BUFFER_MS;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", redc_total_cal_time_ms);
}
static DEVICE_ATTR_RO(redc_cal_time_ms);

static ssize_t dvl_peq_coefficients_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	u32 reg, dvl_peq_coefficients[CS40L26_DVL_PEQ_COEFFICIENTS_NUM_REGS];
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "PEQ_COEF1_X", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_DVL_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_bulk_read(cs40l26->regmap, reg, dvl_peq_coefficients,
			CS40L26_DVL_PEQ_COEFFICIENTS_NUM_REGS);
	if (error)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;

	return snprintf(buf, PAGE_SIZE, "%08X %08X %08X %08X %08X %08X\n",
			dvl_peq_coefficients[0], dvl_peq_coefficients[1], dvl_peq_coefficients[2],
			dvl_peq_coefficients[3], dvl_peq_coefficients[4], dvl_peq_coefficients[5]);
}

static ssize_t dvl_peq_coefficients_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg, dvl_peq_coefficients[CS40L26_DVL_PEQ_COEFFICIENTS_NUM_REGS];
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	char *coeffs_str, *coeffs_str_temp, *coeff_str;
	int error, coeffs_found = 0;

	coeffs_str = kstrdup(buf, GFP_KERNEL);
	if (!coeffs_str)
		return -ENOMEM;

	coeffs_str_temp = coeffs_str;
	while ((coeff_str = strsep(&coeffs_str_temp, " ")) != NULL) {
		error = kstrtou32(coeff_str, 16, &dvl_peq_coefficients[coeffs_found++]);
		if (error)
			goto err_free;
	}

	if (coeffs_found != CS40L26_DVL_PEQ_COEFFICIENTS_NUM_REGS) {
		dev_err(cs40l26->dev, "Num DVL PEQ coeffs, %d, expecting %d\n",
				coeffs_found, CS40L26_DVL_PEQ_COEFFICIENTS_NUM_REGS);
		error = -EINVAL;
		goto err_free;
	}

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		goto err_free;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "PEQ_COEF1_X", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_DVL_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_bulk_write(cs40l26->regmap, reg, dvl_peq_coefficients,
			CS40L26_DVL_PEQ_COEFFICIENTS_NUM_REGS);
	if (error)
		dev_err(cs40l26->dev, "Failed to write DVL PEQ coefficients,%d", error);

err_mutex:
	mutex_unlock(&cs40l26->lock);
	cs40l26_pm_exit(cs40l26->dev);
err_free:
	kfree(coeffs_str);
	return error ? error : count;
}
static DEVICE_ATTR_RW(dvl_peq_coefficients);

static ssize_t logging_en_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, enable;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "ENABLE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LOGGER_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_read(cs40l26->regmap, reg, &enable);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : snprintf(buf, PAGE_SIZE, "%u\n", enable);
}

static ssize_t logging_en_store(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 enable, reg;
	int error;

	error = kstrtou32(buf, 10, &enable);
	if (error)
		return error;

	enable &= CS40L26_LOGGER_EN_MASK;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "ENABLE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LOGGER_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	error = regmap_write(cs40l26->regmap, reg, enable);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(logging_en);

static ssize_t logging_max_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 rst;
	int error;

	error = kstrtou32(buf, 10, &rst);
	if (error)
		return error;

	if (rst != 1)
		return -EINVAL;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	cs40l26_mailbox_write(cs40l26, CS40L26_DSP_MBOX_CMD_LOGGER_MAX_RESET);

	cs40l26_pm_exit(cs40l26->dev);

	return count;
}
static DEVICE_ATTR_WO(logging_max_reset);

static int cs40l26_logger_max_get(struct cs40l26_private *cs40l26, u32 src_id, u32 *max)
{
	int error, reg, src_num;
	u32 offset;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	for (src_num = 0; src_num < cs40l26->num_log_srcs; src_num++) {
		if (cs40l26->log_srcs[src_num].id == src_id)
			break;
	}

	if (src_num == cs40l26->num_log_srcs) {
		error = -ENODEV;
		goto err_mutex;
	}

	error = cl_dsp_get_reg(cs40l26->dsp, "DATA", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LOGGER_ALGO_ID, &reg);
	if (error)
		goto err_mutex;

	offset = (src_num * CS40L26_LOGGER_DATA_MAX_STEP) + CS40L26_LOGGER_DATA_MAX_OFFSET;

	error = regmap_read(cs40l26->regmap, reg + offset, max);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error;
}

static ssize_t max_bemf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 max_bemf;
	int error;

	error = cs40l26_logger_max_get(cs40l26, CS40L26_LOGGER_SRC_ID_BEMF, &max_bemf);

	return error ? error : snprintf(buf, PAGE_SIZE, "0x%06X\n", max_bemf);
}
static DEVICE_ATTR_RO(max_bemf);

static ssize_t max_vbst_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 max_vbst;
	int error;

	error = cs40l26_logger_max_get(cs40l26, CS40L26_LOGGER_SRC_ID_VBST, &max_vbst);

	return error ? error : snprintf(buf, PAGE_SIZE, "0x%06X\n", max_vbst);
}
static DEVICE_ATTR_RO(max_vbst);

static ssize_t max_vmon_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 max_vmon;
	int error;

	error = cs40l26_logger_max_get(cs40l26, CS40L26_LOGGER_SRC_ID_VMON, &max_vmon);

	return error ? error : snprintf(buf, PAGE_SIZE, "0x%06X\n", max_vmon);
}
static DEVICE_ATTR_RO(max_vmon);

static ssize_t max_excursion_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 max_excursion;
	int error;

	error = cs40l26_logger_max_get(cs40l26, CS40L26_LOGGER_SRC_ID_EP, &max_excursion);

	return error ? error : snprintf(buf, PAGE_SIZE, "0x%06X\n", max_excursion);
}
static DEVICE_ATTR_RO(max_excursion);

static ssize_t svc_le_est_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int le;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cs40l26_svc_le_estimate(cs40l26, &le);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (error)
		return error;

	return snprintf(buf, PAGE_SIZE, "%u\n", le);
}
static DEVICE_ATTR_RO(svc_le_est);

static ssize_t svc_le_stored_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;

	mutex_lock(&cs40l26->lock);

	error = snprintf(buf, PAGE_SIZE, "%d\n", cs40l26->svc_le_est_stored);

	mutex_unlock(&cs40l26->lock);

	return error;
}

static ssize_t svc_le_stored_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 svc_le_stored;
	int error;

	error = kstrtou32(buf, 10, &svc_le_stored);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	cs40l26->svc_le_est_stored = svc_le_stored;

	mutex_unlock(&cs40l26->lock);

	return count;
}
static DEVICE_ATTR_RW(svc_le_stored);

static struct attribute *cs40l26_dev_attrs_cal[] = {
	&dev_attr_svc_le_est.attr,
	&dev_attr_svc_le_stored.attr,
	&dev_attr_max_vbst.attr,
	&dev_attr_max_bemf.attr,
	&dev_attr_max_vmon.attr,
	&dev_attr_max_excursion.attr,
	&dev_attr_logging_max_reset.attr,
	&dev_attr_logging_en.attr,
	&dev_attr_trigger_calibration.attr,
	&dev_attr_f0_measured.attr,
	&dev_attr_q_measured.attr,
	&dev_attr_redc_measured.attr,
	&dev_attr_dvl_peq_coefficients.attr,
	&dev_attr_redc_est.attr,
	&dev_attr_f0_stored.attr,
	&dev_attr_q_stored.attr,
	&dev_attr_redc_stored.attr,
	&dev_attr_freq_centre.attr,
	&dev_attr_freq_span.attr,
	&dev_attr_f0_and_q_cal_time_ms.attr,
	&dev_attr_redc_cal_time_ms.attr,
	NULL,
};

struct attribute_group cs40l26_dev_attr_cal_group = {
	.name = "calibration",
	.attrs = cs40l26_dev_attrs_cal,
};
