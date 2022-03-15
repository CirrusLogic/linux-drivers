// SPDX-License-Identifier: GPL-2.0
//
// cs40l26-syfs.c -- CS40L26 Boosted Haptic Driver with Integrated DSP and
// Waveform Memory with Advanced Closed Loop Algorithms and LRA protection
//
// Copyright 2021 Cirrus Logic, Inc.
//
// Author: Fred Treven <fred.treven@cirrus.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation.

#include <linux/mfd/cs40l26.h>

static ssize_t dsp_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u8 dsp_state;
	int ret;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	ret = cs40l26_dsp_state_get(cs40l26, &dsp_state);

	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;
	else
		return snprintf(buf, PAGE_SIZE, "%u\n",
				(unsigned int) (dsp_state & 0xFF));
}
static DEVICE_ATTR_RO(dsp_state);

static ssize_t halo_heartbeat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, halo_heartbeat;
	int ret;

	ret = cl_dsp_get_reg(cs40l26->dsp, "HALO_HEARTBEAT",
			CL_DSP_XM_UNPACKED_TYPE, cs40l26->fw.id, &reg);
	if (ret)
		return ret;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	ret = regmap_read(cs40l26->regmap, reg, &halo_heartbeat);

	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", halo_heartbeat);
}
static DEVICE_ATTR_RO(halo_heartbeat);

static ssize_t pm_stdby_timeout_ms_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 timeout_ms;
	int ret;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	ret = cs40l26_pm_stdby_timeout_ms_get(cs40l26, &timeout_ms);

	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%u\n", timeout_ms);
}

static ssize_t pm_stdby_timeout_ms_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 timeout_ms;
	int ret;

	ret = kstrtou32(buf, 10, &timeout_ms);
	if (ret)
		return -EINVAL;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	ret = cs40l26_pm_stdby_timeout_ms_set(cs40l26, timeout_ms);

	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(pm_stdby_timeout_ms);

static ssize_t pm_active_timeout_ms_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 timeout_ms;
	int ret;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	ret = cs40l26_pm_active_timeout_ms_get(cs40l26, &timeout_ms);

	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%u\n", timeout_ms);
}

static ssize_t pm_active_timeout_ms_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 timeout_ms;
	int ret;

	ret = kstrtou32(buf, 10, &timeout_ms);
	if (ret)
		return -EINVAL;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	ret = cs40l26_pm_active_timeout_ms_set(cs40l26, timeout_ms);

	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(pm_active_timeout_ms);

static ssize_t vibe_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int state;

	if (!cs40l26->pdata.vibe_state_reporting)  {
		dev_err(cs40l26->dev, "cirrus,vibe-state not in DT\n");
		return -EPERM;
	}

	mutex_lock(&cs40l26->lock);
	state = cs40l26->vibe_state;
	mutex_unlock(&cs40l26->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", state);
}
static DEVICE_ATTR_RO(vibe_state);

static ssize_t power_on_seq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct cs40l26_pseq_op *op;
	u32 addr, data, base;
	int ret;

	mutex_lock(&cs40l26->lock);

	base = cs40l26->pseq_base;

	list_for_each_entry_reverse(op, &cs40l26->pseq_op_head, list) {
		switch (op->operation) {
		case CS40L26_PSEQ_OP_WRITE_FULL:
			addr = ((op->words[0] & 0xFFFF) << 16) |
					((op->words[1] & 0x00FFFF00) >> 8);
			data = ((op->words[1] & 0xFF) << 24) |
					(op->words[2] & 0xFFFFFF);
			break;
		case CS40L26_PSEQ_OP_WRITE_H16:
		case CS40L26_PSEQ_OP_WRITE_L16:
			addr = ((op->words[0] & 0xFFFF) << 8) |
					((op->words[1] & 0xFF0000) >> 16);
			data = (op->words[1] & 0xFFFF);

			if (op->operation == CS40L26_PSEQ_OP_WRITE_H16)
				data <<= 16;
			break;
		case CS40L26_PSEQ_OP_WRITE_ADDR8:
			addr = (op->words[0] & 0xFF00) >> 8;
			data = ((op->words[0] & 0xFF) << 24) |
					(op->words[1] & 0xFFFFFF);
			break;
		case CS40L26_PSEQ_OP_END:
			addr = CS40L26_PSEQ_OP_END_ADDR;
			data = CS40L26_PSEQ_OP_END_DATA;
			break;
		default:
			dev_err(cs40l26->dev, "Unrecognized Op Code: 0x%02X\n",
					op->operation);
			ret = -EINVAL;
			goto err_mutex;
		}

		dev_dbg(cs40l26->dev,
		"0x%08x: code = 0x%02X, Addr = 0x%08X, Data = 0x%08X\n",
		base + op->offset, op->operation, addr, data);
	}

	ret = snprintf(buf, PAGE_SIZE, "%d\n", cs40l26->pseq_num_ops);

err_mutex:
	mutex_unlock(&cs40l26->lock);
	return ret;
}
static DEVICE_ATTR_RO(power_on_seq);

static ssize_t owt_free_space_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, words;
	int ret;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	ret = cl_dsp_get_reg(cs40l26->dsp, "OWT_SIZE_XM",
		CL_DSP_XM_UNPACKED_TYPE, CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		goto err_pm;

	ret = regmap_read(cs40l26->regmap, reg, &words);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to get remaining OWT space\n");
		goto err_pm;
	}

	ret = snprintf(buf, PAGE_SIZE, "%d\n", words * CL_DSP_BYTES_PER_WORD);

err_pm:
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return ret;
}
static DEVICE_ATTR_RO(owt_free_space);

static ssize_t die_temp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct regmap *regmap = cs40l26->regmap;
	u16 die_temp;
	int ret;
	u32 val;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	ret = regmap_read(regmap, CS40L26_GLOBAL_ENABLES, &val);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to read GLOBAL_EN status\n");
		goto err_pm;
	}

	if (!(val & CS40L26_GLOBAL_EN_MASK)) {
		dev_err(cs40l26->dev,
			"Global enable must be set to get die temp.\n");
		ret = -EPERM;
		goto err_pm;
	}

	ret = regmap_read(regmap, CS40L26_ENABLES_AND_CODES_DIG, &val);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to get die temperature\n");
		goto err_pm;
	}

	die_temp = (val & CS40L26_TEMP_RESULT_FILT_MASK) >>
			CS40L26_TEMP_RESULT_FILT_SHIFT;

	ret = snprintf(buf, PAGE_SIZE, "0x%03X\n", die_temp);

err_pm:
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return ret;
}
static DEVICE_ATTR_RO(die_temp);

static ssize_t num_waves_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 nwaves;
	int ret;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	ret = cs40l26_get_num_waves(cs40l26, &nwaves);
	if (ret)
		goto err_pm;

	ret = snprintf(buf, PAGE_SIZE, "%u\n", nwaves);

err_pm:
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return ret;
}
static DEVICE_ATTR_RO(num_waves);

/* boost_disable_delay is in units of 125us, e.g. 8 ->  1ms */
static ssize_t boost_disable_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, boost_disable_delay;
	int ret;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	ret = cl_dsp_get_reg(cs40l26->dsp, "BOOST_DISABLE_DELAY",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_EXT_ALGO_ID, &reg);
	if (ret)
		goto err_pm;

	ret = regmap_read(cs40l26->regmap, reg, &boost_disable_delay);
	if (ret)
		goto err_pm;

	ret = snprintf(buf, PAGE_SIZE, "%d\n", boost_disable_delay);

err_pm:
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return ret;
}

static ssize_t boost_disable_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, boost_disable_delay;
	int ret;

	dev_dbg(cs40l26->dev, "%s: %s", __func__, buf);

	ret = kstrtou32(buf, 10, &boost_disable_delay);

	if (ret ||
		boost_disable_delay < CS40L26_BOOST_DISABLE_DELAY_MIN ||
		boost_disable_delay > CS40L26_BOOST_DISABLE_DELAY_MAX)
		return -EINVAL;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	ret = cl_dsp_get_reg(cs40l26->dsp, "BOOST_DISABLE_DELAY",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_EXT_ALGO_ID, &reg);
	if (ret)
		goto err_pm;

	ret = regmap_write(cs40l26->regmap, reg, boost_disable_delay);

err_pm:
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;
	else
		return count;
}
static DEVICE_ATTR_RW(boost_disable_delay);

static ssize_t f0_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int reg, val;
	int ret;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "F0_OFFSET",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_VIBEGEN_ALGO_ID,
			&reg);
	if (ret)
		goto err_mutex;


	ret = regmap_read(cs40l26->regmap, reg, &val);
	if (ret)
		goto err_mutex;

	ret = snprintf(buf, PAGE_SIZE, "%u\n", val);

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return ret;
}

static ssize_t f0_offset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int reg, val;
	int ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return -EINVAL;

	if (val > CS40L26_F0_OFFSET_MAX && val < CS40L26_F0_OFFSET_MIN)
		return -EINVAL;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "F0_OFFSET",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_VIBEGEN_ALGO_ID,
			&reg);
	if (ret)
		goto err_mutex;

	ret = regmap_write(cs40l26->regmap, reg, val);
	if (ret)
		goto err_mutex;

	ret = count;

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return ret;
}
static DEVICE_ATTR_RW(f0_offset);

static ssize_t delay_before_stop_playback_us_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&cs40l26->lock);

	ret = snprintf(buf, PAGE_SIZE, "%d\n",
					cs40l26->delay_before_stop_playback_us);

	mutex_unlock(&cs40l26->lock);

	return ret;
}

static ssize_t delay_before_stop_playback_us_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 val;
	int ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return -EINVAL;

	mutex_lock(&cs40l26->lock);

	cs40l26->delay_before_stop_playback_us = val;

	mutex_unlock(&cs40l26->lock);

	return count;
}
static DEVICE_ATTR_RW(delay_before_stop_playback_us);

static ssize_t f0_comp_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&cs40l26->lock);

	if (cs40l26->fw.id == CS40L26_FW_CALIB_ID) {
		ret = -EPERM;
		goto err_mutex;
	}

	if (cs40l26->comp_enable_pend) {
		ret = -EIO;
		goto err_mutex;
	}

	ret = snprintf(buf, PAGE_SIZE, "%d\n", cs40l26->comp_enable_f0);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	return ret;
}

static ssize_t f0_comp_enable_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	unsigned int val;
	u32 reg, value;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return -EINVAL;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	cs40l26->comp_enable_pend = true;
	cs40l26->comp_enable_f0 = val > 0;

	value = (cs40l26->comp_enable_redc << CS40L26_COMP_EN_REDC_SHIFT) |
		(cs40l26->comp_enable_f0 << CS40L26_COMP_EN_F0_SHIFT);

	if (cs40l26->fw.id == CS40L26_FW_CALIB_ID) {
		ret = -EPERM;
	} else {
		ret = cl_dsp_get_reg(cs40l26->dsp, "COMPENSATION_ENABLE",
				CL_DSP_XM_UNPACKED_TYPE,
				CS40L26_VIBEGEN_ALGO_ID, &reg);
		if (ret)
			goto err_mutex;

		ret = regmap_write(cs40l26->regmap, reg, value);
	}

	if (ret)
		goto err_mutex;

	ret = count;

err_mutex:
	cs40l26->comp_enable_pend = false;
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return ret;
}
static DEVICE_ATTR_RW(f0_comp_enable);

static ssize_t redc_comp_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&cs40l26->lock);

	if (cs40l26->fw.id == CS40L26_FW_CALIB_ID) {
		ret = -EPERM;
		goto err_mutex;
	}

	if (cs40l26->comp_enable_pend) {
		ret = -EIO;
		goto err_mutex;
	}

	ret = snprintf(buf, PAGE_SIZE, "%d\n", cs40l26->comp_enable_redc);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	return ret;
}

static ssize_t redc_comp_enable_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	unsigned int val;
	u32 reg, value;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return -EINVAL;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}
	mutex_lock(&cs40l26->lock);

	cs40l26->comp_enable_pend = true;
	cs40l26->comp_enable_redc = val > 0;

	value = (cs40l26->comp_enable_redc << CS40L26_COMP_EN_REDC_SHIFT) |
		(cs40l26->comp_enable_f0 << CS40L26_COMP_EN_F0_SHIFT);

	if (cs40l26->fw.id == CS40L26_FW_CALIB_ID) {
		ret = -EPERM;
	} else {
		ret = cl_dsp_get_reg(cs40l26->dsp, "COMPENSATION_ENABLE",
				CL_DSP_XM_UNPACKED_TYPE,
				CS40L26_VIBEGEN_ALGO_ID, &reg);
		if (ret)
			goto err_mutex;

		ret = regmap_write(cs40l26->regmap, reg, value);
	}

	if (ret)
		goto err_mutex;

	ret = count;

err_mutex:
	cs40l26->comp_enable_pend = false;
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return ret;
}
static DEVICE_ATTR_RW(redc_comp_enable);

static ssize_t swap_firmware_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&cs40l26->lock);

	if (cs40l26->fw.id == CS40L26_FW_ID)
		ret = snprintf(buf, PAGE_SIZE, "%d\n", 0);
	else if (cs40l26->fw.id == CS40L26_FW_CALIB_ID)
		ret = snprintf(buf, PAGE_SIZE, "%d\n", 1);
	else
		ret = -EINVAL;

	mutex_unlock(&cs40l26->lock);

	return ret;
}

static ssize_t swap_firmware_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	unsigned int variant;

	ret = kstrtou32(buf, 10, &variant);
	if (ret)
		return ret;

	if (variant == 0)
		ret = cs40l26_fw_swap(cs40l26, CS40L26_FW_ID);
	else if (variant == 1)
		ret = cs40l26_fw_swap(cs40l26, CS40L26_FW_CALIB_ID);
	else
		ret = -EINVAL;

	return ret ? ret : count;
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
	int ret;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "FLAGS", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_EXT_ALGO_ID, &reg);
	if (ret)
		goto err_pm;

	ret = regmap_read(cs40l26->regmap, reg, &val);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to get FLAGS\n");
		goto err_pm;
	}


	val &= CS40L26_DBC_ENABLE_MASK;
	val >>= CS40L26_DBC_ENABLE_SHIFT;

	ret = snprintf(buf, PAGE_SIZE, "%u\n", val);

err_pm:
	mutex_unlock(&cs40l26->lock);

	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return ret;
}

static ssize_t dbc_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;
	u32 val;

	ret = kstrtou32(buf, 10, &val);
	if (ret || (val != 0 && val != 1))
		return -EINVAL;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "FLAGS", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_EXT_ALGO_ID, &reg);
	if (ret)
		goto err_pm;

	ret = regmap_update_bits(cs40l26->regmap, reg, CS40L26_DBC_ENABLE_MASK,
			val << CS40L26_DBC_ENABLE_SHIFT);
	if (ret)
		dev_err(cs40l26->dev, "Failed to update FLAGS\n");

err_pm:
	mutex_unlock(&cs40l26->lock);

	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(dbc_enable);

static ssize_t dbc_env_rel_coef_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = cs40l26_dbc_get(cs40l26, CS40L26_DBC_ENV_REL_COEF, &val);

	return ret ? ret : snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t dbc_env_rel_coef_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;

	ret = cs40l26_dbc_set(cs40l26, CS40L26_DBC_ENV_REL_COEF, buf);

	return ret ? ret : count;

}
static DEVICE_ATTR_RW(dbc_env_rel_coef);

static ssize_t dbc_rise_headroom_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = cs40l26_dbc_get(cs40l26, CS40L26_DBC_RISE_HEADROOM, &val);

	return ret ? ret : snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t dbc_rise_headroom_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;

	ret = cs40l26_dbc_set(cs40l26, CS40L26_DBC_RISE_HEADROOM, buf);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(dbc_rise_headroom);

static ssize_t dbc_fall_headroom_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = cs40l26_dbc_get(cs40l26, CS40L26_DBC_FALL_HEADROOM, &val);

	return ret ? ret : snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t dbc_fall_headroom_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;

	ret = cs40l26_dbc_set(cs40l26, CS40L26_DBC_FALL_HEADROOM, buf);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(dbc_fall_headroom);

static ssize_t dbc_tx_lvl_thresh_fs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = cs40l26_dbc_get(cs40l26, CS40L26_DBC_TX_LVL_THRESH_FS, &val);

	return ret ? ret : snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t dbc_tx_lvl_thresh_fs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;

	ret = cs40l26_dbc_set(cs40l26, CS40L26_DBC_TX_LVL_THRESH_FS, buf);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(dbc_tx_lvl_thresh_fs);

static ssize_t dbc_tx_lvl_hold_off_ms_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = cs40l26_dbc_get(cs40l26, CS40L26_DBC_TX_LVL_HOLD_OFF_MS, &val);

	return ret ? ret : snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t dbc_tx_lvl_hold_off_ms_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;

	ret = cs40l26_dbc_set(cs40l26, CS40L26_DBC_TX_LVL_HOLD_OFF_MS, buf);

	return ret ? ret : count;
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
	int ret;

	dev_dbg(cs40l26->dev, "%s: %s", __func__, buf);

	ret = kstrtou32(buf, 16, &calibration_request_payload);
	if (ret ||
		calibration_request_payload < 1 ||
		calibration_request_payload > 2)
		return -EINVAL;

	mailbox_command = ((CS40L26_DSP_MBOX_CMD_INDEX_CALIBRATION_CONTROL <<
				CS40L26_DSP_MBOX_CMD_INDEX_SHIFT) &
				CS40L26_DSP_MBOX_CMD_INDEX_MASK) |
				(calibration_request_payload &
				CS40L26_DSP_MBOX_CMD_PAYLOAD_MASK);

	/* pm_runtime_put occurs is irq_handler after diagnostic is finished */
	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
		mailbox_command, CS40L26_DSP_MBOX_RESET);

	if (ret) {
		dev_err(cs40l26->dev, "Failed to request calibration\n");
		cs40l26->cal_requested = 0;
	} else {
		cs40l26->cal_requested = calibration_request_payload;
		ret = count;
	}

	mutex_unlock(&cs40l26->lock);

	return ret;
}
static DEVICE_ATTR_WO(trigger_calibration);

static ssize_t f0_measured_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	u32 reg, f0_measured;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "F0_EST",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_read(cs40l26->regmap, reg, &f0_measured);
	if (ret)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;
	else
		return snprintf(buf, PAGE_SIZE, "%08X\n", f0_measured);
}
static DEVICE_ATTR_RO(f0_measured);

static ssize_t q_measured_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	u32 reg, q_measured;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "Q_EST",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_read(cs40l26->regmap, reg, &q_measured);
	if (ret)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;
	else
		return snprintf(buf, PAGE_SIZE, "%08X\n", q_measured);
}
static DEVICE_ATTR_RO(q_measured);

static ssize_t redc_measured_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	u32 reg, redc_measured;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "RE_EST_STATUS",
			CL_DSP_YM_UNPACKED_TYPE,
			CS40l26_SVC_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_read(cs40l26->regmap, reg, &redc_measured);
	if (ret)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;
	else
		return snprintf(buf, PAGE_SIZE, "%08X\n", redc_measured);
}
static DEVICE_ATTR_RO(redc_measured);

static ssize_t redc_est_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	u32 reg, redc_est;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "REDC",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_read(cs40l26->regmap, reg, &redc_est);
	if (ret)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;
	else
		return snprintf(buf, PAGE_SIZE, "%08X\n", redc_est);
}

static ssize_t redc_est_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	u32 reg, redc_est;

	dev_dbg(cs40l26->dev, "%s: %s", __func__, buf);

	ret = kstrtou32(buf, 16, &redc_est);
	if (ret)
		return ret;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "REDC",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_write(cs40l26->regmap, reg, redc_est);
	if (ret)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;
	else
		return count;
}
static DEVICE_ATTR_RW(redc_est);

static ssize_t f0_stored_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	u32 reg, f0_stored;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "F0_OTP_STORED",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_read(cs40l26->regmap, reg, &f0_stored);
	if (ret)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;
	else
		return snprintf(buf, PAGE_SIZE, "%08X\n", f0_stored);
}

static ssize_t f0_stored_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	u32 reg, f0_stored;

	dev_dbg(cs40l26->dev, "%s: %s", __func__, buf);

	ret = kstrtou32(buf, 16, &f0_stored);

	if (ret ||
		f0_stored < CS40L26_F0_EST_MIN ||
		f0_stored > CS40L26_F0_EST_MAX)
		return -EINVAL;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "F0_OTP_STORED",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_write(cs40l26->regmap, reg, f0_stored);
	if (ret)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;
	else
		return count;
}
static DEVICE_ATTR_RW(f0_stored);

static ssize_t q_stored_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	u32 reg, q_stored;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "Q_STORED",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_read(cs40l26->regmap, reg, &q_stored);
	if (ret)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;
	else
		return snprintf(buf, PAGE_SIZE, "%08X\n", q_stored);
}

static ssize_t q_stored_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	u32 reg, q_stored;

	dev_dbg(cs40l26->dev, "%s: %s", __func__, buf);

	ret = kstrtou32(buf, 16, &q_stored);

	if (ret || q_stored < CS40L26_Q_EST_MIN || q_stored > CS40L26_Q_EST_MAX)
		return -EINVAL;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "Q_STORED",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_write(cs40l26->regmap, reg, q_stored);
	if (ret)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;
	else
		return count;
}
static DEVICE_ATTR_RW(q_stored);

static ssize_t redc_stored_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	u32 reg, redc_stored;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "REDC_OTP_STORED",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_read(cs40l26->regmap, reg, &redc_stored);
	if (ret)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;
	else
		return snprintf(buf, PAGE_SIZE, "%08X\n", redc_stored);
}

static ssize_t redc_stored_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	u32 reg, redc_stored;

	dev_dbg(cs40l26->dev, "%s: %s", __func__, buf);

	ret = kstrtou32(buf, 16, &redc_stored);
	if (ret)
		return ret;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "REDC_OTP_STORED",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_write(cs40l26->regmap, reg, redc_stored);
	if (ret)
		goto err_mutex;

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;
	else
		return count;
}
static DEVICE_ATTR_RW(redc_stored);

static ssize_t f0_and_q_cal_time_ms_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	u32 reg, freq_span, freq_centre, f0_and_q_cal_time_ms;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "FREQ_SPAN",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_read(cs40l26->regmap, reg, &freq_span);
	if (ret)
		goto err_mutex;

	ret = cl_dsp_get_reg(cs40l26->dsp, "FREQ_CENTRE",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_read(cs40l26->regmap, reg, &freq_centre);

	f0_and_q_cal_time_ms = ((CS40L26_F0_CHIRP_DURATION_FACTOR *
				(freq_span >> CS40L26_F0_EST_FREQ_SHIFT)) /
				(freq_centre >> CS40L26_F0_EST_FREQ_SHIFT)) +
				CS40L26_F0_AND_Q_CALIBRATION_BUFFER_MS;

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", f0_and_q_cal_time_ms);
}
static DEVICE_ATTR_RO(f0_and_q_cal_time_ms);

static ssize_t redc_cal_time_ms_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	/* FIRMWARE_STUMPY_CALIB_REDC_PLAYTIME_MS + SVC_INIT + buffer */
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret;
	u32 reg, redc_playtime_ms, redc_total_cal_time_ms;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "REDC_PLAYTIME_MS",
			CL_DSP_XM_UNPACKED_TYPE,
			cs40l26->fw.id, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_read(cs40l26->regmap, reg, &redc_playtime_ms);

	redc_total_cal_time_ms = redc_playtime_ms +
					CS40L26_SVC_INITIALIZATION_PERIOD_MS +
					CS40L26_REDC_CALIBRATION_BUFFER_MS;

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", redc_total_cal_time_ms);
}
static DEVICE_ATTR_RO(redc_cal_time_ms);

static ssize_t logging_en_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, enable;
	int ret;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "ENABLE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LOGGER_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_read(cs40l26->regmap, reg, &enable);
	if (ret)
		dev_err(cs40l26->dev, "Failed to read logging enable\n");
	else
		ret = snprintf(buf, PAGE_SIZE, "%d\n", enable);

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return ret;
}

static ssize_t logging_en_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct regmap *regmap = cs40l26->regmap;
	struct device *cdev = cs40l26->dev;
	struct cl_dsp *dsp = cs40l26->dsp;
	u32 enable, reg, src_count, src;
	int ret, i;

	ret = kstrtou32(buf, 10, &enable);
	if (ret)
		return ret;

	if (enable != 0 && enable != 1)
		return -EINVAL;

	ret = pm_runtime_get_sync(cdev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cdev);
		return ret;
	}

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(dsp, "COUNT", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LOGGER_ALGO_ID, &reg);
	if (ret)
		goto exit_mutex;

	ret = regmap_read(regmap, reg, &src_count);
	if (ret) {
		dev_err(cdev, "Failed to get logger source count\n");
		goto exit_mutex;
	}

	ret = cl_dsp_get_reg(dsp, "SOURCE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LOGGER_ALGO_ID, &reg);
	if (ret)
		goto exit_mutex;

	if (cs40l26->fw.id == CS40L26_FW_ID) {
		if (src_count != CS40L26_LOGGER_SRC_COUNT) {
			dev_err(cdev, "Unexpected source count %u\n",
					src_count);
			ret = -EINVAL;
			goto exit_mutex;
		}

		ret = regmap_read(regmap, reg, &src);
		if (ret) {
			dev_err(cdev, "Failed to get Logger Source\n");
			goto exit_mutex;
		}

		src &= CS40L26_LOGGER_SRC_ID_MASK;
		src >>= CS40L26_LOGGER_SRC_ID_SHIFT;

		if (src != CS40L26_LOGGER_SRC_ID_VMON) {
			dev_err(cdev, "Invalid Logger Source %u\n", src);
			ret = -EINVAL;
			goto exit_mutex;
		}
	} else if (cs40l26->fw.id == CS40L26_FW_CALIB_ID) {
		if (src_count != CS40L26_LOGGER_SRC_COUNT_CALIB) {
			dev_err(cdev, "Unexpected source count %u\n",
					src_count);
			ret = -EINVAL;
			goto exit_mutex;
		}

		for (i = 0; i < src_count; i++) {
			ret = regmap_read(regmap, reg +
					(i * CL_DSP_BYTES_PER_WORD), &src);
			if (ret) {
				dev_err(dev, "Failed to get Logger Source\n");
				goto exit_mutex;
			}

			src &= CS40L26_LOGGER_SRC_ID_MASK;
			src >>= CS40L26_LOGGER_SRC_ID_SHIFT;

			if (src != (i + 1)) {
				dev_err(cdev, "Invalid Logger Source %u\n",
						src);
				ret = -EINVAL;
				goto exit_mutex;
			}
		}
	} else {
		dev_err(cdev, "Invalid firmware ID 0x%06X\n",
			cs40l26->fw.id);
		goto exit_mutex;
	}

	ret = cl_dsp_get_reg(dsp, "ENABLE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LOGGER_ALGO_ID, &reg);
	if (ret)
		goto exit_mutex;

	ret = regmap_write(regmap, reg, enable);
	if (ret)
		dev_err(cdev, "Failed to %s logging\n",
				enable ? "enable" : "disable");

exit_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cdev);
	pm_runtime_put_autosuspend(cdev);


	return ret ? ret : count;
}

static DEVICE_ATTR_RW(logging_en);

static ssize_t logging_max_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 rst;
	int ret;

	ret = kstrtou32(buf, 10, &rst);
	if (ret)
		return ret;

	if (rst != 1)
		return -EINVAL;

	ret = pm_runtime_get_sync(cs40l26->dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(cs40l26->dev);
		return ret;
	}

	cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
		CS40L26_DSP_MBOX_CMD_LOGGER_MAX_RESET, CS40L26_DSP_MBOX_RESET);

	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return count;
}
static DEVICE_ATTR_WO(logging_max_reset);

static ssize_t max_bemf_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{

	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, max_bemf;
	int ret;

	if (cs40l26->fw.id != CS40L26_FW_CALIB_ID) {
		dev_err(cs40l26->dev, "Calib. FW required for BEMF logging\n");
		return -EPERM;
	}

	pm_runtime_get_sync(cs40l26->dev);
	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "DATA", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LOGGER_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_read(cs40l26->regmap, reg +
			CS40L26_LOGGER_DATA_1_MAX_OFFSET, &max_bemf);
	if (ret)
		dev_err(cs40l26->dev, "Failed to get max. back EMF\n");

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "0x%06X\n", max_bemf);
}
static DEVICE_ATTR_RO(max_bemf);

static ssize_t max_vbst_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{

	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, max_vbst;
	int ret;

	if (cs40l26->fw.id != CS40L26_FW_CALIB_ID) {
		dev_err(cs40l26->dev, "Calib. FW required for VBST logging\n");
		return -EPERM;
	}

	pm_runtime_get_sync(cs40l26->dev);
	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "DATA", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LOGGER_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_read(cs40l26->regmap, reg +
			CS40L26_LOGGER_DATA_2_MAX_OFFSET, &max_vbst);
	if (ret)
		dev_err(cs40l26->dev, "Failed to get max. VBST\n");

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "0x%06X\n", max_vbst);
}
static DEVICE_ATTR_RO(max_vbst);

static ssize_t max_vmon_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{

	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, max_vmon;
	u8 offset;
	int ret;

	if (cs40l26->fw.id == CS40L26_FW_CALIB_ID)
		offset = CS40L26_LOGGER_DATA_3_MAX_OFFSET;
	else
		offset = CS40L26_LOGGER_DATA_1_MAX_OFFSET;

	pm_runtime_get_sync(cs40l26->dev);
	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, "DATA", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LOGGER_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_read(cs40l26->regmap, reg + offset, &max_vmon);
	if (ret)
		dev_err(cs40l26->dev, "Failed to get max. VMON\n");

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "0x%06X\n", max_vmon);
}
static DEVICE_ATTR_RO(max_vmon);

static struct attribute *cs40l26_dev_attrs_cal[] = {
	&dev_attr_max_vbst.attr,
	&dev_attr_max_bemf.attr,
	&dev_attr_max_vmon.attr,
	&dev_attr_logging_max_reset.attr,
	&dev_attr_logging_en.attr,
	&dev_attr_trigger_calibration.attr,
	&dev_attr_f0_measured.attr,
	&dev_attr_q_measured.attr,
	&dev_attr_redc_measured.attr,
	&dev_attr_redc_est.attr,
	&dev_attr_f0_stored.attr,
	&dev_attr_q_stored.attr,
	&dev_attr_redc_stored.attr,
	&dev_attr_f0_and_q_cal_time_ms.attr,
	&dev_attr_redc_cal_time_ms.attr,
	NULL,
};

struct attribute_group cs40l26_dev_attr_cal_group = {
	.name = "calibration",
	.attrs = cs40l26_dev_attrs_cal,
};
