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

static const struct regmap_config cs40l26_broadcast_regmap = {
	.reg_bits =		32,
	.reg_stride =		4,
	.val_bits =		32,
	.reg_format_endian =	REGMAP_ENDIAN_BIG,
	.val_format_endian =	REGMAP_ENDIAN_BIG,
	.writeable_reg =	cs40l26_broadcast_writeable_reg,
	.readable_reg =		cs40l26_broadcast_readable_reg,
};

static ssize_t broadcast_master_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 broadcast_master;
	int error = 0;

	mutex_lock(&cs40l26->lock);

	if (!cs40l26->broadcast_addr) {
		error = -EPERM;
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_HW, __func__);
		goto err_mutex;
	}

	broadcast_master = cs40l26->broadcast_client ? 1 : 0;

err_mutex:
	mutex_unlock(&cs40l26->lock);

	return error ? error : sysfs_emit(buf, "%u\n", broadcast_master);
}

static ssize_t broadcast_master_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct i2c_client *client;
	u32 broadcast;
	int error;

	error = kstrtou32(buf, 10, &broadcast);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	mutex_lock(&cs40l26->lock);

	if (!cs40l26->broadcast_addr) {
		error = -EPERM;
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_HW, __func__);
		goto err_mutex;
	}

	if (broadcast == 0) {
		if (cs40l26->broadcast_regmap) {
			regmap_exit(cs40l26->broadcast_regmap);
			cs40l26->broadcast_regmap = NULL;
		}

		if (cs40l26->broadcast_client) {
			i2c_unregister_device(cs40l26->broadcast_client);
			cs40l26->broadcast_client = NULL;
		}
	} else if (broadcast == 1) {
		if (cs40l26->broadcast_client) {
			error = -EINVAL;
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_HW, __func__);
			goto err_mutex;
		}

		client = of_find_i2c_device_by_node(cs40l26->dev->of_node);
		if (!client) {
			error = -ENODATA;
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_DT, __func__);
			goto err_mutex;
		}

		cs40l26->broadcast_client = i2c_new_dummy_device(client->adapter,
				cs40l26->broadcast_addr);
		if (IS_ERR(cs40l26->broadcast_client)) {
			error = PTR_ERR(cs40l26->broadcast_client);
			cs40l26->broadcast_client = NULL;
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_HW, __func__);
			goto err_mutex;
		}

		cs40l26->broadcast_regmap = regmap_init_i2c(cs40l26->broadcast_client,
				&cs40l26_broadcast_regmap);
		if (IS_ERR(cs40l26->broadcast_regmap)) {
			error = PTR_ERR(cs40l26->broadcast_regmap);
			i2c_unregister_device(cs40l26->broadcast_client);
			cs40l26->broadcast_client = NULL;
			cs40l26->broadcast_regmap = NULL;
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_HW, __func__);
		}
	} else {
		error = -EINVAL;
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);
	}

err_mutex:
	mutex_unlock(&cs40l26->lock);

	return error ? error : count;
}
static DEVICE_ATTR_RW(broadcast_master);

static ssize_t dsp_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u8 dsp_state;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_state_get(cs40l26, &dsp_state);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "%u\n", dsp_state);
}
static DEVICE_ATTR_RO(dsp_state);

static ssize_t owt_lib_compat_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "1.0.0\n");
}
static DEVICE_ATTR_RO(owt_lib_compat);

static ssize_t overprotection_gain_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 op_gain;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "PROTECTION_XM_OP_GAIN", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_EP_ALGO_ID, &op_gain);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "%u\n", op_gain);
}

static ssize_t overprotection_gain_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 op_gain;
	int error;

	error = kstrtou32(buf, 10, &op_gain);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (op_gain < CS40L26_OVERPROTECTION_GAIN_MIN || op_gain > CS40L26_OVERPROTECTION_GAIN_MAX)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_write_ctl_reg(cs40l26, "PROTECTION_XM_OP_GAIN", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_EP_ALGO_ID, op_gain);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(overprotection_gain);

static ssize_t halo_heartbeat_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 halo_heartbeat;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "HALO_HEARTBEAT", CL_DSP_XM_UNPACKED_TYPE,
			cs40l26->fw_id, &halo_heartbeat);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "%u\n", halo_heartbeat);
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
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_pm_timeout_ms_get(cs40l26, CS40L26_DSP_STATE_STANDBY, &timeout_ms);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "%u\n", timeout_ms);
}

static ssize_t pm_stdby_timeout_ms_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 timeout_ms;
	int error;

	error = kstrtou32(buf, 10, &timeout_ms);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (timeout_ms < CS40L26_PM_STDBY_TIMEOUT_MS_MIN || timeout_ms > CS40L26_PM_TIMEOUT_MS_MAX)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_pm_timeout_ms_set(cs40l26, CS40L26_DSP_STATE_STANDBY, timeout_ms);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(pm_stdby_timeout_ms);

static ssize_t pm_active_timeout_ms_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 timeout_ms;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_pm_timeout_ms_get(cs40l26, CS40L26_DSP_STATE_ACTIVE, &timeout_ms);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "%u\n", timeout_ms);
}

static ssize_t pm_active_timeout_ms_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 timeout_ms;
	int error;

	error = kstrtou32(buf, 10, &timeout_ms);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (timeout_ms < CS40L26_PM_ACTIVE_TIMEOUT_MS_MIN ||
			timeout_ms > CS40L26_PM_TIMEOUT_MS_MAX)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_pm_timeout_ms_set(cs40l26, CS40L26_DSP_STATE_ACTIVE, timeout_ms);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(pm_active_timeout_ms);

static ssize_t vibe_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);

	if (!cs40l26->vibe_state_reporting)
		return cs40l26_log_err(cs40l26, -EPERM, CS40L26_ERR_TYPE_FW, __func__);

	/* Intentionally emit without mutex lock to allow for reporting during haptic playback */
	return sysfs_emit(buf, "%u\n", cs40l26->vibe_state);
}
static DEVICE_ATTR_RO(vibe_state);

static ssize_t owt_free_space_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 words;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "OWT_SIZE_XM", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &words);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "%u\n", words * CL_DSP_BYTES_PER_WORD);
}
static DEVICE_ATTR_RO(owt_free_space);

static int cs40l26_get_die_temp(struct cs40l26_private *cs40l26, u32 *die_temp)
{
	u32 global_enable;
	int error;

	error = regmap_read(cs40l26->regmap, CS40L26_GLOBAL_ENABLES, &global_enable);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_CP, __func__);

	if (!(global_enable & CS40L26_GLOBAL_EN_MASK))
		return cs40l26_log_err(cs40l26, -EPERM, CS40L26_ERR_TYPE_HW, __func__);

	error = regmap_read(cs40l26->regmap, CS40L26_ENABLES_AND_CODES_DIG, die_temp);

	return error ? cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_CP, __func__) : 0;
}

static ssize_t die_temp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 die_temp = 0;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_get_die_temp(cs40l26, &die_temp);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	die_temp = (die_temp & CS40L26_TEMP_RESULT_FILT_MASK) >> CS40L26_TEMP_RESULT_FILT_SHIFT;

	return error ? error : sysfs_emit(buf, "0x%03X\n", die_temp);
}
static DEVICE_ATTR_RO(die_temp);

static ssize_t num_waves_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 nwaves;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_num_waves(cs40l26, &nwaves);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_COEFF, __func__) :
			sysfs_emit(buf, "%u\n", nwaves);
}
static DEVICE_ATTR_RO(num_waves);

static ssize_t f0_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 f0_offset;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "F0_OFFSET", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &f0_offset);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "0x%06X\n", f0_offset);
}

static ssize_t f0_offset_store(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 f0_offset;
	int error;

	error = kstrtou32(buf, 16, &f0_offset);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if ((f0_offset > CS40L26_F0_OFFSET_MAX && f0_offset < CS40L26_F0_OFFSET_MIN) ||
			f0_offset > CS40L26_F0_MASK)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_write_ctl_reg(cs40l26, "F0_OFFSET", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, f0_offset);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(f0_offset);

static ssize_t delay_before_stop_playback_us_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 delay;

	mutex_lock(&cs40l26->lock);

	delay = cs40l26->delay_before_stop_playback_us;

	mutex_unlock(&cs40l26->lock);

	return sysfs_emit(buf, "%u\n", delay);
}

static ssize_t delay_before_stop_playback_us_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;
	u32 delay;

	error = kstrtou32(buf, 10, &delay);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	mutex_lock(&cs40l26->lock);

	cs40l26->delay_before_stop_playback_us = delay;

	mutex_unlock(&cs40l26->lock);

	return count;
}
static DEVICE_ATTR_RW(delay_before_stop_playback_us);

static ssize_t f0_comp_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 comp_en;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "COMPENSATION_ENABLE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &comp_en);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "%lu\n",
			FIELD_GET(CS40L26_COMP_EN_F0_MASK, comp_en));
}

static ssize_t f0_comp_enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 comp_en;
	int error;

	error = kstrtou32(buf, 10, &comp_en);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (comp_en > CS40L26_COMP_EN_F0_MAX_OPTION)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_update_ctl_reg(cs40l26, "COMPENSATION_ENABLE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, comp_en, CS40L26_COMP_EN_F0_MASK);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(f0_comp_enable);

static ssize_t redc_comp_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 comp_en;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "COMPENSATION_ENABLE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &comp_en);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "%lu\n",
			FIELD_GET(CS40L26_COMP_EN_REDC_MASK, comp_en));
}

static ssize_t redc_comp_enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 comp_en;
	int error;

	error = kstrtou32(buf, 10, &comp_en);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (comp_en > CS40L26_COMP_EN_REDC_MAX_OPTION)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_update_ctl_reg(cs40l26, "COMPENSATION_ENABLE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, comp_en, CS40L26_COMP_EN_REDC_MASK);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(redc_comp_enable);

static ssize_t swap_firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 variant;

	mutex_lock(&cs40l26->lock);

	variant = cs40l26->fw_id;

	mutex_unlock(&cs40l26->lock);

	switch (variant) {
	case CS40L26_FW_ID:
		return sysfs_emit(buf, "%u\n", CS40L26_FW_RUNTIME);
	case CS40L26_FW_CALIB_ID:
		return sysfs_emit(buf, "%u\n", CS40L26_FW_CALIBRATION);
	default:
		break;
	}

	return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);
}

static ssize_t swap_firmware_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 variant;
	int error;

	error = kstrtou32(buf, 10, &variant);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	switch (variant) {
	case CS40L26_FW_RUNTIME:
		error = cs40l26_fw_swap(cs40l26, CS40L26_FW_ID);
		break;
	case CS40L26_FW_CALIBRATION:
		error = cs40l26_fw_swap(cs40l26, CS40L26_FW_CALIB_ID);
		break;
	default:
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);
	}

	return error ? error : count;
}
static DEVICE_ATTR_RW(swap_firmware);

static ssize_t swap_wavetable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 wt_num;

	mutex_lock(&cs40l26->lock);

	wt_num = cs40l26->wt_num;

	mutex_unlock(&cs40l26->lock);

	return sysfs_emit(buf, "%u\n", wt_num);
}

static int cs40l26_handle_wt_swap(struct cs40l26_private *cs40l26, const u32 wt_num)
{
	int error_pm, error_wt;

	/* Bypass PM runtime framework for DSP shutdown & wake */
	cs40l26_irq_enable(cs40l26, CS40L26_IRQ_DISABLE);
	cs40l26_pm_runtime_teardown(cs40l26);

	mutex_lock(&cs40l26->lock);

	error_wt = cs40l26_wt_swap(cs40l26, wt_num);

	mutex_unlock(&cs40l26->lock);

	if (error_wt)
		cs40l26_log_err(cs40l26, error_wt, CS40L26_ERR_TYPE_FW, __func__);
	else
		cs40l26->wt_num = wt_num;

	error_pm = cs40l26_pm_runtime_setup(cs40l26);
	if (error_pm)
		return cs40l26_log_err(cs40l26, error_pm, CS40L26_ERR_TYPE_PM, __func__);

	cs40l26_irq_enable(cs40l26, CS40L26_IRQ_ENABLE);

	return error_wt;
}

static ssize_t swap_wavetable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 nwaves, wt_num;
	int error;

	error = kstrtou32(buf, 10, &wt_num);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_handle_wt_swap(cs40l26, wt_num);
	if (error)
		return error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_mailbox_write(cs40l26, CS40L26_DSP_MBOX_CMD_OWT_RESET);
	if (error)
		goto err_mutex;

	error = cs40l26_num_waves(cs40l26, &nwaves);
	if (!error)
		dev_info(cs40l26->dev, "Loaded new wavetable with %d waveforms\n", nwaves);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(swap_wavetable);

static ssize_t fw_rev_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 fw_rev;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_fw_rev_get(cs40l26->dsp, &fw_rev);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__) :
			sysfs_emit(buf, "%d.%d.%d\n", (int) CL_DSP_GET_MAJOR(fw_rev),
					(int) CL_DSP_GET_MINOR(fw_rev),
					(int) CL_DSP_GET_PATCH(fw_rev));
}
static DEVICE_ATTR_RO(fw_rev);

static ssize_t init_rom_wavetable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 init_rom_wavetable;
	int error;

	error = kstrtou32(buf, 10, &init_rom_wavetable);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (init_rom_wavetable != CS40L26_INIT_ROM_WAVETABLE)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_rom_wt_init(cs40l26);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__) : count;
}
static DEVICE_ATTR_WO(init_rom_wavetable);

static int cs40l26_braking_time_find(struct cs40l26_private *cs40l26, struct cl_dsp_memchunk *ch,
		u32 *braking_time)
{
	u32 metadata_word, period = 0;
	int error;

	/*
	 * In the OWT wavetable, the metadata is appended to the end
	 * of the waveform header section.
	 */
	do {
		error = cl_dsp_memchunk_read(cs40l26->dsp, ch, 24, &metadata_word);
		if (error)
			return error;

		if (FIELD_GET(CL_DSP_MD_TYPE_MASK, metadata_word) == CL_DSP_SVC_ID &&
				FIELD_GET(CL_DSP_MD_LENGTH_MASK, metadata_word) == CL_DSP_SVC_LEN) {
			/* Braking period is second word of SVC metadata */
			error = cl_dsp_memchunk_read(cs40l26->dsp, ch, 24, &period);
			if (error)
				return error;

			break;
		}
	} while (metadata_word != CL_DSP_MD_TERMINATOR);

	/* Braking period is stored as milliseconds * 8 */
	*braking_time = period / 8;

	return 0;
}

static int cs40l26_get_owt_header_params(struct cs40l26_private *cs40l26,
		struct cl_dsp_memchunk *ch, u16 *flags, u8 *type, u32 *size)
{
	int error;

	error = cl_dsp_memchunk_read(cs40l26->dsp, ch, 16, flags);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__);

	error = cl_dsp_memchunk_read(cs40l26->dsp, ch, 8, type);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__);

	error = cl_dsp_memchunk_read(cs40l26->dsp, ch, 24, NULL);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__);

	error = cl_dsp_memchunk_read(cs40l26->dsp, ch, 24, size);

	return error ? cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__) : 0;
}

static int cs40l26_owt_size_get(struct cs40l26_private *cs40l26, u32 *owt_size, u32 *owt_base)
{
	u32 offset;
	int error;

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "OWT_BASE_XM", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, owt_base);
	if (error)
		return error;

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "OWT_NEXT_XM", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &offset);
	if (error)
		return error;

	if (*owt_base > offset)
		return cs40l26_log_err(cs40l26, -ENOMEM, CS40L26_ERR_TYPE_DSP, __func__);

	*owt_size = (offset - *owt_base) * CL_DSP_BYTES_PER_WORD;

	return 0;
}

static int cs40l26_owt_braking_time_get(struct cs40l26_private *cs40l26, u32 *braking_time)
{
	u32 current_index = 0, owt_base, owt_size_bytes = 0, size = 0, wavetable_reg;
	struct cl_dsp_memchunk ch;
	u8 type = 0, *wavetable;
	u16 flags;
	int error;

	error = cs40l26_owt_size_get(cs40l26, &owt_size_bytes, &owt_base);
	if (error)
		return error;

	wavetable = kmalloc(owt_size_bytes, GFP_KERNEL);
	if (!wavetable)
		return -ENOMEM;

	error = cl_dsp_get_reg(cs40l26->dsp, "WAVE_XM_TABLE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &wavetable_reg);
	if (error) {
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__);
		goto err_free;
	}

	error = regmap_raw_read(cs40l26->regmap, wavetable_reg +
			(owt_base * CL_DSP_BYTES_PER_WORD), wavetable, owt_size_bytes);
	if (error) {
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_CP, __func__);
		goto err_free;
	}

	ch = cl_dsp_memchunk_create(wavetable, owt_size_bytes);

	/* Ensure there's enough unread space to read at least one header */
	while ((ch.max - ch.data) >= CS40L26_WT_HEADER_PWLE_SIZE) {
		error = cs40l26_get_owt_header_params(cs40l26, &ch, &flags, &type, &size);
		if (error)
			goto err_free;

		if (current_index != cs40l26->braking_time_index) {
			/* Skip header terminator word for PWLE waveforms. */
			if (type == WT_TYPE_V6_PWLE) {
				error = cl_dsp_memchunk_read(cs40l26->dsp, &ch, 24, NULL);
				if (error)
					goto err_free;
			}

			/* Skip to the start of the next waveform's header */
			while (size-- > 0) {
				error = cl_dsp_memchunk_read(cs40l26->dsp, &ch, 24, NULL);
				if (error)
					goto err_free;
			}

			current_index++;

			continue;
		}

		if (type != WT_TYPE_V6_PWLE || !(flags & CL_DSP_MD_PRESENT)) {
			error = -EINVAL;
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);
			goto err_free;
		}

		error = cs40l26_braking_time_find(cs40l26, &ch, braking_time);
		if (error)
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_COEFF, __func__);

		break;
	}

err_free:
	kfree(wavetable);

	return error;
}

static ssize_t braking_time_bank_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;
	u32 bank;

	error = kstrtou32(buf, 10, &bank);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (bank != CS40L26_RAM_BANK_ID && bank != CS40L26_OWT_BANK_ID)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	mutex_lock(&cs40l26->lock);

	cs40l26->braking_time_bank = bank;

	mutex_unlock(&cs40l26->lock);

	return count;
}
static DEVICE_ATTR_WO(braking_time_bank);

static ssize_t braking_time_index_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;
	u32 index;

	error = kstrtou32(buf, 10, &index);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	mutex_lock(&cs40l26->lock);

	cs40l26->braking_time_index = index;

	mutex_unlock(&cs40l26->lock);

	return count;
}
static DEVICE_ATTR_WO(braking_time_index);

static int cs40l26_get_braking_time_ms(struct cs40l26_private *cs40l26, u32 *braking_time)
{
	u32 nwaves;
	int error;

	switch (cs40l26->braking_time_bank) {
	case CS40L26_RAM_BANK_ID:
		error = cs40l26_num_ram_waves(cs40l26, &nwaves);
		break;
	case CS40L26_OWT_BANK_ID:
		error = cs40l26_num_owt_waves(cs40l26, &nwaves);
		break;
	default:
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);
	}
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_COEFF, __func__);

	if (cs40l26->braking_time_index > nwaves - 1)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (cs40l26->braking_time_bank == CS40L26_OWT_BANK_ID)
		return cs40l26_owt_braking_time_get(cs40l26, braking_time);

	*braking_time = cs40l26->dsp->wt_desc->owt.waves[cs40l26->braking_time_index].braking_time;

	return error;
}

static ssize_t braking_time_ms_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 braking_time = 0;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_get_braking_time_ms(cs40l26, &braking_time);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "%u\n", braking_time);
}
static DEVICE_ATTR_RO(braking_time_ms);

static void cs40l26_clear_err_log(struct cs40l26_private *cs40l26)
{
	memset((void *) cs40l26->errs, 0, sizeof(cs40l26->errs));

	cs40l26->num_errs = 0;
}

static ssize_t error_log_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int at = 0, error = 0, i, nelements;
	char str[CS40L26_ERR_STR_MAX_LEN];

	mutex_lock(&cs40l26->lock);

	nelements = cs40l26->num_errs > CS40L26_ERR_LOG_SIZE ? CS40L26_ERR_LOG_SIZE :
			cs40l26->num_errs;

	if (!nelements) {
		error = -ENODATA;
		goto err_mutex;
	}

	for (i = 0; i < nelements; i++) {
		error = snprintf(str, CS40L26_ERR_STR_MAX_LEN, "%d. %s: code = %d, type = %u\n",
				cs40l26->errs[i].num + 1, cs40l26->errs[i].fxn_name,
				cs40l26->errs[i].code, cs40l26->errs[i].type);
		if (error < 0)
			break;

		if (at + error >= PAGE_SIZE) {
			dev_info(cs40l26->dev, "Error log truncated due to page size\n");
			break;
		}

		error = sysfs_emit_at(buf, at, str);
		if (error < 0)
			break;

		at += error;
	}

	if (cs40l26->err_clear_method == CS40L26_ERR_CLEAR_ON_READ)
		cs40l26_clear_err_log(cs40l26);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	return error < 0 ? cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__) : at;
}

static ssize_t error_log_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;
	u32 clear;

	if (cs40l26->err_clear_method == CS40L26_ERR_CLEAR_ON_READ)
		return cs40l26_log_err(cs40l26, -EPERM, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = kstrtou32(buf, 10, &clear);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (clear != CS40L26_ERR_LOG_CLEAR)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	mutex_lock(&cs40l26->lock);

	cs40l26_clear_err_log(cs40l26);

	mutex_unlock(&cs40l26->lock);

	return count;
}
static DEVICE_ATTR_RW(error_log);


static ssize_t error_log_clear_method_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	enum cs40l26_err_clear clear;

	mutex_lock(&cs40l26->lock);

	clear = cs40l26->err_clear_method;

	mutex_unlock(&cs40l26->lock);

	return sysfs_emit(buf, "%u\n", clear);
}

static ssize_t error_log_clear_method_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 clear;
	int error;

	error = kstrtou32(buf, 10, &clear);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	mutex_lock(&cs40l26->lock);

	switch (clear) {
	case CS40L26_ERR_CLEAR_ON_READ:
		cs40l26->err_clear_method = CS40L26_ERR_CLEAR_ON_READ;
		break;
	case CS40L26_ERR_CLEAR_ON_WRITE:
		cs40l26->err_clear_method = CS40L26_ERR_CLEAR_ON_WRITE;
		break;
	default:
		error = -EINVAL;
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);
	}

	mutex_unlock(&cs40l26->lock);

	return error ? error : count;
}
static DEVICE_ATTR_RW(error_log_clear_method);

static ssize_t lf0t_freq_centre_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 freq_centre;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "F_CENTRE_SET", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &freq_centre);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "0x%06X\n", freq_centre);
}

static ssize_t lf0t_freq_centre_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 lf0t_freq_centre;
	int error;

	error = kstrtou32(buf, 16, &lf0t_freq_centre);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (lf0t_freq_centre > CS40L26_LF0T_FREQ_CENTRE_MAX)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_write_ctl_reg(cs40l26, "F_CENTRE_SET", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, lf0t_freq_centre);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(lf0t_freq_centre);

static ssize_t lf0t_init_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 lf0t_init;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "COMPENSATION_ENABLE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &lf0t_init);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "%lu\n",
			FIELD_GET(CS40L26_LF0T_INIT_MASK, lf0t_init));
}

static ssize_t lf0t_init_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 lf0t_init;
	int error;

	error = kstrtou32(buf, 10, &lf0t_init);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (lf0t_init > CS40L26_LF0T_INIT_LAST_TRACKED)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_update_ctl_reg(cs40l26, "COMPENSATION_ENABLE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, lf0t_init, CS40L26_LF0T_INIT_MASK);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(lf0t_init);

static struct attribute *cs40l26_dev_attrs[] = {
	&dev_attr_broadcast_master.attr,
	&dev_attr_num_waves.attr,
	&dev_attr_die_temp.attr,
	&dev_attr_owt_free_space.attr,
	&dev_attr_dsp_state.attr,
	&dev_attr_halo_heartbeat.attr,
	&dev_attr_pm_stdby_timeout_ms.attr,
	&dev_attr_pm_active_timeout_ms.attr,
	&dev_attr_vibe_state.attr,
	&dev_attr_f0_offset.attr,
	&dev_attr_delay_before_stop_playback_us.attr,
	&dev_attr_f0_comp_enable.attr,
	&dev_attr_redc_comp_enable.attr,
	&dev_attr_swap_firmware.attr,
	&dev_attr_swap_wavetable.attr,
	&dev_attr_fw_rev.attr,
	&dev_attr_owt_lib_compat.attr,
	&dev_attr_overprotection_gain.attr,
	&dev_attr_init_rom_wavetable.attr,
	&dev_attr_braking_time_bank.attr,
	&dev_attr_braking_time_index.attr,
	&dev_attr_braking_time_ms.attr,
	&dev_attr_error_log.attr,
	&dev_attr_error_log_clear_method.attr,
	&dev_attr_lf0t_freq_centre.attr,
	&dev_attr_lf0t_init.attr,
	NULL,
};

static struct attribute_group cs40l26_dev_attr_group = {
	.name = "default",
	.attrs = cs40l26_dev_attrs,
};

static int cs40l26_run_calibration(struct cs40l26_private *cs40l26, struct completion *completion,
		const u32 calibration_request_payload)
{
	u32 mailbox_command;
	int error;

	mailbox_command = ((CS40L26_DSP_MBOX_CMD_INDEX_CALIBRATION_CONTROL <<
			CS40L26_DSP_MBOX_CMD_TYPE_SHIFT) & CS40L26_DSP_MBOX_CMD_TYPE_MASK) |
			(calibration_request_payload & CS40L26_DSP_MBOX_CMD_PAYLOAD_MASK);

	mutex_lock(&cs40l26->lock);

	reinit_completion(completion);

	error = cs40l26_mailbox_write(cs40l26, mailbox_command);
	if (!error)
		cs40l26->cal_ongoing = true;

	mutex_unlock(&cs40l26->lock);

	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_DSP, __func__);

	/* Cannot wait for completion under mutex lock */
	if (!wait_for_completion_timeout(completion,
				msecs_to_jiffies(CS40L26_CALIBRATION_TIMEOUT_MS))) {
		error = -ETIME;
		dev_err(cs40l26->dev, "Failed to complete cal req, %d, err: %d",
				calibration_request_payload, error);
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_DSP, __func__);
	}

	return 0;
}

static int cs40l26_copy_f0_est_to_dvl(struct cs40l26_private *cs40l26)
{
	u32 f0_measured, f0_normalized, global_sample_rate;
	int error, sample_rate;

	error = regmap_read(cs40l26->regmap, CS40L26_GLOBAL_SAMPLE_RATE, &global_sample_rate);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_CP, __func__);

	switch (global_sample_rate & CS40L26_GLOBAL_FS_MASK) {
	case CS40L26_GLOBAL_FS_48K:
		sample_rate = 48000;
		break;
	case CS40L26_GLOBAL_FS_96K:
		sample_rate = 96000;
		break;
	default:
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_DSP, __func__);
	}

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "F0_EST", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &f0_measured);
	if (error)
		return error;

	f0_normalized = (f0_measured << CS40L26_F0_NORM_SHIFT) / sample_rate;

	return cs40l26_dsp_write_ctl_reg(cs40l26, "LRA_NORM_F0", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_DVL_ALGO_ID, f0_normalized);
}

static ssize_t trigger_calibration_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 calibration_request_payload;
	struct completion *completion;
	int error;

	error = kstrtou32(buf, 16, &calibration_request_payload);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	if (!cs40l26->calib_fw) {
		error = -EPERM;
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__);
		goto err_mutex;
	}

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
		error = -EINVAL;
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);
		goto err_mutex;
	}

	mutex_unlock(&cs40l26->lock);

	/* Cannot run calibration under mutex lock */
	error = cs40l26_run_calibration(cs40l26, completion, calibration_request_payload);
	if (error)
		goto err_pm;

	mutex_lock(&cs40l26->lock);

	if (calibration_request_payload == CS40L26_CALIBRATION_CONTROL_REQUEST_F0_AND_Q)
		error = cs40l26_copy_f0_est_to_dvl(cs40l26);

err_mutex:
	mutex_unlock(&cs40l26->lock);

err_pm:
	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_WO(trigger_calibration);

static ssize_t cal_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	bool cal_status;

	mutex_lock(&cs40l26->lock);

	cal_status = cs40l26->cal_ongoing;

	mutex_unlock(&cs40l26->lock);

	return sysfs_emit(buf, "%u\n", cal_status);
}

static ssize_t cal_status_store(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 cal_reset;
	int error;

	error = kstrtou32(buf, 10, &cal_reset);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (cal_reset != CS40L26_CAL_STATUS_RESET)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	mutex_lock(&cs40l26->lock);

	cs40l26->cal_ongoing = false;

	mutex_unlock(&cs40l26->lock);

	return count;
}
static DEVICE_ATTR_RW(cal_status);

static ssize_t f0_measured_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 f0_measured;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "F0_EST", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &f0_measured);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "0x%06X\n", f0_measured);
}
static DEVICE_ATTR_RO(f0_measured);

static ssize_t q_measured_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 q_measured;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "Q_EST", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &q_measured);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "0x%06X\n", q_measured);
}
static DEVICE_ATTR_RO(q_measured);

static ssize_t redc_measured_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 redc_measured;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "RE_EST_STATUS", CL_DSP_YM_UNPACKED_TYPE,
			CS40L26_SVC_ALGO_ID, &redc_measured);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "0x%06X\n", redc_measured);
}
static DEVICE_ATTR_RO(redc_measured);

static ssize_t redc_est_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 redc_est;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "REDC", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &redc_est);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "0x%06X\n", redc_est);
}

static ssize_t redc_est_store(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 redc_est;
	int error;

	error = kstrtou32(buf, 16, &redc_est);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (redc_est > CS40L26_F0_EST_REDC_MAX)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_write_ctl_reg(cs40l26, "REDC", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, redc_est);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(redc_est);

static ssize_t f0_stored_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 f0_stored;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "F0_OTP_STORED", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &f0_stored);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "0x%06X\n", f0_stored);
}

static ssize_t f0_stored_store(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 f0_stored;
	int error;

	error = kstrtou32(buf, 16, &f0_stored);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (f0_stored < CS40L26_F0_FREQ_CENTRE_MIN || f0_stored > CS40L26_F0_FREQ_CENTRE_MAX)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_write_ctl_reg(cs40l26, "F0_OTP_STORED", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, f0_stored);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(f0_stored);

static ssize_t redc_stored_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 redc_stored;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "REDC_OTP_STORED", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &redc_stored);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "0x%06X\n", redc_stored);
}

static ssize_t redc_stored_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 redc_stored;
	int error;

	error = kstrtou32(buf, 16, &redc_stored);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (redc_stored > CS40L26_F0_EST_REDC_MAX)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_write_ctl_reg(cs40l26, "REDC_OTP_STORED", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, redc_stored);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(redc_stored);

static ssize_t freq_centre_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 freq_centre;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "FREQ_CENTRE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &freq_centre);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "0x%06X\n", freq_centre);
}

static ssize_t freq_centre_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 freq_centre;
	int error;

	error = kstrtou32(buf, 16, &freq_centre);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (freq_centre < CS40L26_F0_FREQ_CENTRE_MIN ||
			freq_centre > CS40L26_F0_FREQ_CENTRE_MAX)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_write_ctl_reg(cs40l26, "FREQ_CENTRE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, freq_centre);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(freq_centre);

static ssize_t freq_span_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 freq_span;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "FREQ_SPAN", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &freq_span);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "0x%06X\n", freq_span);
}

static ssize_t freq_span_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 freq_span;
	int error;

	error = kstrtou32(buf, 16, &freq_span);
	if (error)
		return error;

	if ((freq_span > CS40L26_F0_FREQ_SPAN_POS_MAX && freq_span < CS40L26_F0_FREQ_SPAN_NEG_MIN)
			|| freq_span < CS40L26_F0_FREQ_SPAN_POS_MIN
			|| freq_span > CS40L26_F0_FREQ_SPAN_NEG_MAX)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_write_ctl_reg(cs40l26, "FREQ_SPAN", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, freq_span);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(freq_span);

static int calc_f0_and_q_cal_time_ms(struct cs40l26_private *cs40l26, u32 *f0_and_q_cal_time_ms)
{
	u32 freq_centre, freq_span;
	int abs_freq_span, error;

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "FREQ_SPAN", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &freq_span);
	if (error)
		return error;

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "FREQ_CENTRE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &freq_centre);
	if (error)
		return error;

	abs_freq_span = (freq_span & BIT(23)) ? abs((int)(freq_span | GENMASK(31, 24)))
			: freq_span;

	*f0_and_q_cal_time_ms = (u32)((CS40L26_F0_CHIRP_DURATION_FACTOR *
			(int) (abs_freq_span >> CS40L26_F0_EST_FREQ_FRAC_BITS)) /
			(int) (freq_centre >> CS40L26_F0_EST_FREQ_FRAC_BITS));

	return 0;
}

static ssize_t f0_and_q_cal_time_ms_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 f0_and_q_cal_time_ms = 0, tone_dur_ms;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "TONE_DURATION_MS", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &tone_dur_ms);
	if (error)
		goto err_mutex;

	if (tone_dur_ms == CS40L26_F0_AND_Q_CALIBRATION_TIME_UNSET) {
		error = calc_f0_and_q_cal_time_ms(cs40l26, &f0_and_q_cal_time_ms);
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

	return error ? error : sysfs_emit(buf, "%u\n", f0_and_q_cal_time_ms);
}
static DEVICE_ATTR_RO(f0_and_q_cal_time_ms);

static ssize_t redc_cal_time_ms_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 redc_playtime_ms, redc_total_cal_time_ms;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "REDC_PLAYTIME_MS", CL_DSP_XM_UNPACKED_TYPE,
			cs40l26->fw_id, &redc_playtime_ms);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	if (!error)
		redc_total_cal_time_ms = redc_playtime_ms + CS40L26_SVC_INITIALIZATION_PERIOD_MS +
				CS40L26_REDC_CALIBRATION_BUFFER_MS;

	return error ? error : sysfs_emit(buf, "%u\n", redc_total_cal_time_ms);
}
static DEVICE_ATTR_RO(redc_cal_time_ms);

static ssize_t dvl_peq_coefficients_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	u32 dvl_peq_coefficients[CS40L26_DVL_PEQ_COEFFICIENTS_NUM_REGS], reg;
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "PEQ_COEF1_X", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_DVL_ALGO_ID, &reg);
	if (error) {
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__);
		goto err_mutex;
	}

	error = regmap_bulk_read(cs40l26->regmap, reg, dvl_peq_coefficients,
			CS40L26_DVL_PEQ_COEFFICIENTS_NUM_REGS);
	if (error)
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_CP, __func__);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "0x%06X\n0x%06X\n0x%06X\n0x%06X\n0x%06X\n0x%06X\n",
			dvl_peq_coefficients[0], dvl_peq_coefficients[1], dvl_peq_coefficients[2],
			dvl_peq_coefficients[3], dvl_peq_coefficients[4], dvl_peq_coefficients[5]);
}

static int cs40l26_process_word_buf(struct cs40l26_private *cs40l26, const char *buf,
		u32 *copy_buf, const u32 num_words, u32 *words_found, char *revision)
{
	char *word, *words, *words_temp;
	int error = 0;

	words = kstrdup(buf, GFP_KERNEL);
	if (!words)
		return -ENOMEM;

	words_temp = words;

	while ((word = strsep(&words_temp, "\n")) != NULL && *words_found < num_words) {
		error = kstrtou32(word, 16, &copy_buf[(*words_found)++]);
		if (error) {
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);
			goto err_free;
		}
	}

	if (word != NULL && revision != NULL) {
		error = snprintf(revision, CS40L26_ALGO_ID_MAX_STR_LEN, word);
		if (error < 0) {
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_DRIVER, __func__);
			goto err_free;
		}
	}

	if (*words_found != num_words) {
		error = -EINVAL;
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);
	}

err_free:
	kfree(words);

	return error < 0 ? error : 0;
}

static ssize_t dvl_peq_coefficients_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	u32 dvl_peq_coeffs[CS40L26_DVL_PEQ_COEFFICIENTS_NUM_REGS], reg;
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int coeffs_found = 0, error;

	error = cs40l26_process_word_buf(cs40l26, buf, dvl_peq_coeffs,
			CS40L26_DVL_PEQ_COEFFICIENTS_NUM_REGS, &coeffs_found, NULL);
	if (error)
		return error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, "PEQ_COEF1_X", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_DVL_ALGO_ID, &reg);
	if (error) {
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__);
		goto err_mutex;
	}

	error = regmap_bulk_write(cs40l26->regmap, reg, dvl_peq_coeffs,
			CS40L26_DVL_PEQ_COEFFICIENTS_NUM_REGS);
	if (error)
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_CP, __func__);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(dvl_peq_coefficients);

static ssize_t dvl_peq_coeff_apply_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 dvl_peq_coeff_apply;
	int error;

	error = kstrtou32(buf, 10, &dvl_peq_coeff_apply);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (dvl_peq_coeff_apply != CS40L26_DVL_PEQ_COEFF_APPLY)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_mailbox_write(cs40l26, CS40L26_DSP_MBOX_CMD_DVL_REINIT);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_DSP, __func__) : count;
}
static DEVICE_ATTR_WO(dvl_peq_coeff_apply);

static ssize_t ls_calibration_f0_closed_loop_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	bool ls_cal_f0_closed_loop;

	mutex_lock(&cs40l26->lock);

	ls_cal_f0_closed_loop = cs40l26->ls_cal_f0_closed_loop;

	mutex_unlock(&cs40l26->lock);

	return sysfs_emit(buf, "%u\n", ls_cal_f0_closed_loop);
}

static ssize_t ls_calibration_f0_closed_loop_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 closed_loop;
	int error;

	error = kstrtou32(buf, 10, &closed_loop);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (closed_loop > CS40L26_LS_CAL_F0_CL_MAX_OPTION)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	mutex_lock(&cs40l26->lock);

	cs40l26->ls_cal_f0_closed_loop = closed_loop;

	mutex_unlock(&cs40l26->lock);

	return count;
}
static DEVICE_ATTR_RW(ls_calibration_f0_closed_loop);

static ssize_t ls_calibration_params_temp_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 params_temp;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "PARAMS_TEMPERATURE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LS_ALGO_ID, &params_temp);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "0x%06X\n", params_temp);
}

static ssize_t ls_calibration_params_temp_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 params_temp;
	int error;

	error = kstrtou32(buf, 16, &params_temp);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if ((params_temp > CS40L26_LS_CAL_TEMP_MAX && params_temp < CS40L26_LS_CAL_TEMP_MIN) ||
			params_temp > CS40L26_LS_CAL_TEMP_MASK)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_write_ctl_reg(cs40l26, "PARAMS_TEMPERATURE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LS_ALGO_ID, params_temp);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(ls_calibration_params_temp);

static int cs40l26_ls_calibration_check_results(struct cs40l26_private *cs40l26, u32 *status)
{
	u32 return_code;
	int error;

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "STATE_CAL_RETURN_CODE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LS_ALGO_ID, &return_code);
	if (error)
		return error;

	switch (return_code) {
	case CS40L26_LS_CAL_OK:
		dev_dbg(cs40l26->dev, "LS Calibration Succeeded\n");
		break;
	case CS40L26_LS_CAL_IN_PROGRESS:
		dev_err(cs40l26->dev, "LS Calibration still in progress\n");
		break;
	case CS40L26_LS_CAL_FAIL_DET:
		dev_err(cs40l26->dev, "LS Calibration failed: matrix singular/nearly singular\n");
		break;
	case CS40L26_LS_CAL_FAIL_ROOTS:
		dev_err(cs40l26->dev, "LS Calibration failed: real roots instead of 1\n");
		break;
	case CS40L26_LS_CAL_SATURATION:
		dev_err(cs40l26->dev, "LS Calibration failed: saturation when publishing\n");
		break;
	case CS40L26_LS_CAL_STEP2_FREQ:
		dev_err(cs40l26->dev, "LS Calibration failed: frequency for step 2 out of range\n");
		break;
	default:
		dev_err(cs40l26->dev, "LS Calibration failed: unknown error code %u\n",
				return_code);
	}

	*status = return_code;

	return 0;
}

static ssize_t ls_calibration_status_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 status = 0;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_ls_calibration_check_results(cs40l26, &status);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "%u\n", status);
}
static DEVICE_ATTR_RO(ls_calibration_status);

static ssize_t ls_calibration_results_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int at = 0, error, i;
	u32 reg, rev, val;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	for (i = 0; i < CS40L26_LS_CAL_NUM_REGS; i++) {
		if (cs40l26->fw_id == CS40L26_FW_CALIB_ID) {
			error = cl_dsp_get_reg(cs40l26->dsp, cs40l26_ls_cal_params[i].calib_name,
					CL_DSP_XM_UNPACKED_TYPE, CS40L26_LS_ALGO_ID, &reg);
		} else {
			if (cs40l26_ls_cal_params[i].runtime_name == NULL)
				continue;

			error = cl_dsp_get_reg(cs40l26->dsp, cs40l26_ls_cal_params[i].runtime_name,
					CL_DSP_XM_UNPACKED_TYPE, CS40L26_EP_ALGO_ID, &reg);
		}
		if (error) {
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__);
			goto err_mutex;
		}

		if (cs40l26_ls_cal_params[i].word_num == 2)
			reg += sizeof(u32);

		error = regmap_read(cs40l26->regmap, reg, &val);
		if (error) {
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_CP, __func__);
			goto err_mutex;
		}

		error = sysfs_emit_at(buf, at, "0x%06X\n", val);
		if (error < 0) {
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);
			goto err_mutex;
		}

		at += error;
	}

	if (cs40l26->fw_id == CS40L26_FW_CALIB_ID) {
		error = cl_dsp_get_algo_rev(cs40l26->dsp, CS40L26_LS_ALGO_ID, &rev);
		if (error) {
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__);
			goto err_mutex;
		}

		error = sysfs_emit_at(buf, at, "v%ld.%ld.%ld\n", CL_DSP_GET_MAJOR(rev),
				CL_DSP_GET_MINOR(rev), CL_DSP_GET_PATCH(rev));
		if (error < 0) {
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);
			goto err_mutex;
		}

		at += error;
	}

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error < 0 ? error : at;
}

static ssize_t ls_calibration_results_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 ls_cal_results[CS40L26_LS_CAL_NUM_REGS], reg;
	char revision[CS40L26_ALGO_ID_MAX_STR_LEN];
	int error, f0_index, i, results_found = 0;

	error = cs40l26_process_word_buf(cs40l26, buf, ls_cal_results, CS40L26_LS_CAL_NUM_REGS,
			&results_found, revision);
	if (error)
		return error;

	dev_info(cs40l26->dev, "LS calibration %s", revision);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_set_ctl_reg(cs40l26, "CFG", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_EP_ALGO_ID, CS40L26_LS_CAL_REINIT_MASK);
	if (error)
		goto err_mutex;

	f0_index = cs40l26->ls_cal_f0_closed_loop ?
			CS40L26_LS_CAL_F0_CL_INDEX : CS40L26_LS_CAL_F0_OL_INDEX;

	for (i = 0; i < results_found; i++) {
		if (i == f0_index) {
			error = cs40l26_dsp_write_ctl_reg(cs40l26, "F0_OTP_STORED",
					CL_DSP_XM_UNPACKED_TYPE, CS40L26_VIBEGEN_ALGO_ID,
					ls_cal_results[i] << CS40L26_LS_CAL_F0_TO_VIB_SHIFT);
		} else if (i == CS40L26_LS_CAL_REDC_INDEX) {
			error = cs40l26_dsp_write_ctl_reg(cs40l26, "RE0", CL_DSP_XM_UNPACKED_TYPE,
					CS40L26_THERM_LIM_ALGO_ID, ls_cal_results[i]);
		} else if (i == CS40L26_LS_CAL_TEMP_INDEX) {
			error = cs40l26_dsp_write_ctl_reg(cs40l26, "T0", CL_DSP_XM_UNPACKED_TYPE,
					CS40L26_THERM_LIM_ALGO_ID, ls_cal_results[i]);
		}
		if (error)
			goto err_mutex;

		if (cs40l26_ls_cal_params[i].runtime_name == NULL)
			continue;

		error = cl_dsp_get_reg(cs40l26->dsp, cs40l26_ls_cal_params[i].runtime_name,
				CL_DSP_XM_UNPACKED_TYPE, CS40L26_EP_ALGO_ID, &reg);
		if (error) {
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__);
			goto err_mutex;
		}

		if (cs40l26_ls_cal_params[i].word_num == 2)
			reg += sizeof(u32);

		error = regmap_write(cs40l26->regmap, reg, ls_cal_results[i]);
		if (error) {
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_CP, __func__);
			goto err_mutex;
		}
	}

	error = cs40l26_dsp_set_ctl_reg(cs40l26, "CFG", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_EP_ALGO_ID, CS40L26_EP_REINIT);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(ls_calibration_results);

static ssize_t ls_calibration_results_name_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int at = 0, error, i;
	u32 reg, val;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	for (i = 0; i < CS40L26_LS_CAL_NUM_REGS; i++) {
		if (cs40l26->fw_id == CS40L26_FW_CALIB_ID) {
			error = cl_dsp_get_reg(cs40l26->dsp, cs40l26_ls_cal_params[i].calib_name,
					CL_DSP_XM_UNPACKED_TYPE, CS40L26_LS_ALGO_ID, &reg);
		} else {
			if (cs40l26_ls_cal_params[i].runtime_name == NULL)
				continue;

			error = cl_dsp_get_reg(cs40l26->dsp, cs40l26_ls_cal_params[i].runtime_name,
					CL_DSP_XM_UNPACKED_TYPE, CS40L26_EP_ALGO_ID, &reg);
		}
		if (error) {
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__);
			goto err_mutex;
		}

		if (cs40l26_ls_cal_params[i].word_num == 2)
			reg += sizeof(u32);

		error = regmap_read(cs40l26->regmap, reg, &val);
		if (error) {
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_CP, __func__);
			goto err_mutex;
		}

		if (cs40l26->fw_id == CS40L26_FW_CALIB_ID)
			error = sysfs_emit_at(buf, at, "%s: 0x%06X\n",
					cs40l26_ls_cal_params[i].calib_name, val);
		else
			error = sysfs_emit_at(buf, at, "%s: 0x%06X\n",
					cs40l26_ls_cal_params[i].runtime_name, val);
		if (error < 0) {
			cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);
			goto err_mutex;
		}

		at += error;
	}

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error < 0 ? error : at;
}
static DEVICE_ATTR_RO(ls_calibration_results_name);

static ssize_t svc_le_est_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;
	u32 le;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_svc_le_estimate(cs40l26, &le);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "0x%06X\n", le);
}
static DEVICE_ATTR_RO(svc_le_est);

static ssize_t svc_le_stored_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 svc_le_stored;

	mutex_lock(&cs40l26->lock);

	svc_le_stored = cs40l26->svc_le_est_stored;

	mutex_unlock(&cs40l26->lock);

	return sysfs_emit(buf, "0x%06X\n", svc_le_stored);
}

static ssize_t svc_le_stored_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 svc_le_stored;
	int error;

	error = kstrtou32(buf, 16, &svc_le_stored);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (svc_le_stored > CS40L26_SVC_LE_MAX)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	mutex_lock(&cs40l26->lock);

	cs40l26->svc_le_est_stored = svc_le_stored;

	mutex_unlock(&cs40l26->lock);

	return count;
}
static DEVICE_ATTR_RW(svc_le_stored);

static struct attribute *cs40l26_dev_attrs_cal[] = {
	&dev_attr_svc_le_est.attr,
	&dev_attr_svc_le_stored.attr,
	&dev_attr_trigger_calibration.attr,
	&dev_attr_cal_status.attr,
	&dev_attr_f0_measured.attr,
	&dev_attr_q_measured.attr,
	&dev_attr_redc_measured.attr,
	&dev_attr_ls_calibration_f0_closed_loop.attr,
	&dev_attr_ls_calibration_params_temp.attr,
	&dev_attr_ls_calibration_status.attr,
	&dev_attr_ls_calibration_results.attr,
	&dev_attr_ls_calibration_results_name.attr,
	&dev_attr_dvl_peq_coefficients.attr,
	&dev_attr_dvl_peq_coeff_apply.attr,
	&dev_attr_redc_est.attr,
	&dev_attr_f0_stored.attr,
	&dev_attr_redc_stored.attr,
	&dev_attr_freq_centre.attr,
	&dev_attr_freq_span.attr,
	&dev_attr_f0_and_q_cal_time_ms.attr,
	&dev_attr_redc_cal_time_ms.attr,
	NULL,
};

static struct attribute_group cs40l26_dev_attr_cal_group = {
	.name = "calibration",
	.attrs = cs40l26_dev_attrs_cal,
};

static ssize_t logging_en_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 logging_en;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_read_ctl_reg(cs40l26, "ENABLE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LOGGER_ALGO_ID, &logging_en);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : sysfs_emit(buf, "%u\n", logging_en);
}

static ssize_t logging_en_store(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 logging_en;
	int error;

	error = kstrtou32(buf, 10, &logging_en);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (logging_en > CS40L26_LOGGER_EN_MAX_OPTION)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_dsp_write_ctl_reg(cs40l26, "ENABLE", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LOGGER_ALGO_ID, logging_en);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(logging_en);

static ssize_t logging_reset_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 logging_reset;
	int error;

	error = kstrtou32(buf, 10, &logging_reset);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	if (logging_reset != CS40L26_LOGGER_RESET)
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_mailbox_write(cs40l26, CS40L26_DSP_MBOX_CMD_LOGGER_RESET);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_DSP, __func__) : count;
}
static DEVICE_ATTR_WO(logging_reset);

static ssize_t available_logger_srcs_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int at = 0, error = 0, i;

	mutex_lock(&cs40l26->lock);

	for (i = 0; i < cs40l26->num_log_srcs; i++) {
		switch (cs40l26->log_srcs[i].id) {
		case CS40L26_LOGGER_SRC_ID_BEMF:
			at += sysfs_emit_at(buf, at, "BEMF\n");
			break;
		case CS40L26_LOGGER_SRC_ID_VBST:
			at += sysfs_emit_at(buf, at, "VBST\n");
			break;
		case CS40L26_LOGGER_SRC_ID_VMON:
			at += sysfs_emit_at(buf, at, "VMON\n");
			break;
		case CS40L26_LOGGER_SRC_ID_PWR:
			at += sysfs_emit_at(buf, at, "PWR\n");
			break;
		case CS40L26_LOGGER_SRC_ID_EP:
			at += sysfs_emit_at(buf, at, "EP\n");
			break;
		case CS40L26_LOGGER_SRC_ID_IMON:
			at += sysfs_emit_at(buf, at, "IMON\n");
			break;
		default:
			error = -EINVAL;
			goto err_mutex;
		}
	}

err_mutex:
	mutex_unlock(&cs40l26->lock);

	return error ? cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__) : at;
}
static DEVICE_ATTR_RO(available_logger_srcs);

static int cs40l26_logger_data_get(struct cs40l26_private *cs40l26, enum cs40l26_logger_src_id id,
		enum cs40l26_logger_data_type type, u32 *val)
{
	int error, reg, src_num;
	u32 offset;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	for (src_num = 0; src_num < cs40l26->num_log_srcs; src_num++) {
		if (cs40l26->log_srcs[src_num].id == id)
			break;
	}

	if (src_num == cs40l26->num_log_srcs) {
		error = -ENODATA;
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);
		goto err_mutex;
	}

	error = cl_dsp_get_reg(cs40l26->dsp, "DATA", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_LOGGER_ALGO_ID, &reg);
	if (error) {
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__);
		goto err_mutex;
	}

	offset = (src_num * CS40L26_LOGGER_DATA_SRC_STEP) +
			(type * CS40L26_LOGGER_DATA_OFFSET_STEP);

	error = regmap_read(cs40l26->regmap, reg + offset, val);
	if (error)
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_CP, __func__);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error;
}

#define CS40L26_SYSFS_LOGGER_ATTR(name, id, data_type)						\
static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf)\
{											\
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);				\
	u32 val = 0;									\
	int error;									\
											\
	error = cs40l26_logger_data_get(cs40l26, id, data_type, &val);			\
											\
	return error ? error : sysfs_emit(buf, "0x%06X\n", val);			\
}											\
static DEVICE_ATTR_RO(name)

CS40L26_SYSFS_LOGGER_ATTR(avg_power, CS40L26_LOGGER_SRC_ID_PWR, CS40L26_LOGGER_DATA_TYPE_MEAN);
CS40L26_SYSFS_LOGGER_ATTR(max_power, CS40L26_LOGGER_SRC_ID_PWR, CS40L26_LOGGER_DATA_TYPE_MAX);

CS40L26_SYSFS_LOGGER_ATTR(min_bemf, CS40L26_LOGGER_SRC_ID_BEMF, CS40L26_LOGGER_DATA_TYPE_MIN);
CS40L26_SYSFS_LOGGER_ATTR(max_bemf, CS40L26_LOGGER_SRC_ID_BEMF, CS40L26_LOGGER_DATA_TYPE_MAX);
CS40L26_SYSFS_LOGGER_ATTR(mean_bemf, CS40L26_LOGGER_SRC_ID_BEMF, CS40L26_LOGGER_DATA_TYPE_MEAN);

CS40L26_SYSFS_LOGGER_ATTR(min_vbst, CS40L26_LOGGER_SRC_ID_VBST, CS40L26_LOGGER_DATA_TYPE_MIN);
CS40L26_SYSFS_LOGGER_ATTR(max_vbst, CS40L26_LOGGER_SRC_ID_VBST, CS40L26_LOGGER_DATA_TYPE_MAX);
CS40L26_SYSFS_LOGGER_ATTR(mean_vbst, CS40L26_LOGGER_SRC_ID_VBST, CS40L26_LOGGER_DATA_TYPE_MEAN);

CS40L26_SYSFS_LOGGER_ATTR(min_vmon, CS40L26_LOGGER_SRC_ID_VMON, CS40L26_LOGGER_DATA_TYPE_MIN);
CS40L26_SYSFS_LOGGER_ATTR(max_vmon, CS40L26_LOGGER_SRC_ID_VMON, CS40L26_LOGGER_DATA_TYPE_MAX);
CS40L26_SYSFS_LOGGER_ATTR(mean_vmon, CS40L26_LOGGER_SRC_ID_VMON, CS40L26_LOGGER_DATA_TYPE_MEAN);

CS40L26_SYSFS_LOGGER_ATTR(min_excursion, CS40L26_LOGGER_SRC_ID_EP, CS40L26_LOGGER_DATA_TYPE_MIN);
CS40L26_SYSFS_LOGGER_ATTR(max_excursion, CS40L26_LOGGER_SRC_ID_EP, CS40L26_LOGGER_DATA_TYPE_MAX);
CS40L26_SYSFS_LOGGER_ATTR(mean_excursion, CS40L26_LOGGER_SRC_ID_EP, CS40L26_LOGGER_DATA_TYPE_MEAN);

CS40L26_SYSFS_LOGGER_ATTR(min_imon, CS40L26_LOGGER_SRC_ID_IMON, CS40L26_LOGGER_DATA_TYPE_MIN);
CS40L26_SYSFS_LOGGER_ATTR(max_imon, CS40L26_LOGGER_SRC_ID_IMON, CS40L26_LOGGER_DATA_TYPE_MAX);
CS40L26_SYSFS_LOGGER_ATTR(mean_imon, CS40L26_LOGGER_SRC_ID_IMON, CS40L26_LOGGER_DATA_TYPE_MEAN);

static struct attribute *cs40l26_dev_attrs_dlog[] = {
	&dev_attr_logging_en.attr,
	&dev_attr_logging_reset.attr,
	&dev_attr_available_logger_srcs.attr,
	&dev_attr_avg_power.attr,
	&dev_attr_max_power.attr,
	&dev_attr_min_bemf.attr,
	&dev_attr_max_bemf.attr,
	&dev_attr_mean_bemf.attr,
	&dev_attr_min_vbst.attr,
	&dev_attr_max_vbst.attr,
	&dev_attr_mean_vbst.attr,
	&dev_attr_min_vmon.attr,
	&dev_attr_max_vmon.attr,
	&dev_attr_mean_vmon.attr,
	&dev_attr_min_excursion.attr,
	&dev_attr_max_excursion.attr,
	&dev_attr_mean_excursion.attr,
	&dev_attr_min_imon.attr,
	&dev_attr_max_imon.attr,
	&dev_attr_mean_imon.attr,
	NULL,
};

static struct attribute_group cs40l26_dev_attr_dlog_group = {
	.name = "data_logger",
	.attrs = cs40l26_dev_attrs_dlog,
};

static ssize_t fw_algo_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 algo_id;

	mutex_lock(&cs40l26->lock);

	algo_id = cs40l26->sysfs_fw.algo_id;

	mutex_unlock(&cs40l26->lock);

	return sysfs_emit(buf, "0x%06X\n", algo_id);
}

static ssize_t fw_algo_id_store(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 algo_id;
	int error;

	error = kstrtou32(buf, 16, &algo_id);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	switch (algo_id) {
	case CS40L26_A2H_ALGO_ID:
	case CS40L26_BUZZGEN_ALGO_ID:
	case CS40L26_DVL_ALGO_ID:
	case CS40L26_DYNAMIC_F0_ALGO_ID:
	case CS40L26_EP_ALGO_ID:
	case CS40L26_EVENT_HANDLER_ALGO_ID:
	case CS40L26_EVENT_LOGGER_ALGO_ID:
	case CS40L26_EXT_ALGO_ID:
	case CS40L26_F0_EST_ALGO_ID:
	case CS40L26_FW_CALIB_ID:
	case CS40L26_FW_ID:
	case CS40L26_GPIO_ALGO_ID:
	case CS40L26_LF0T_ALGO_ID:
	case CS40L26_LOGGER_ALGO_ID:
	case CS40L26_LS_ALGO_ID:
	case CS40L26_MAILBOX_ALGO_ID:
	case CS40L26_MDSYNC_ALGO_ID:
	case CS40L26_PM_ALGO_ID:
	case CS40L26_SVC_ALGO_ID:
	case CS40L26_THERM_LIM_ALGO_ID:
	case CS40L26_VIBEGEN_ALGO_ID:
		break;
	default:
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);
	}

	mutex_lock(&cs40l26->lock);

	cs40l26->sysfs_fw.algo_id = algo_id;

	mutex_unlock(&cs40l26->lock);

	return count;
}
static DEVICE_ATTR_RW(fw_algo_id);

static ssize_t fw_ctrl_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	char *ctrl_name;

	mutex_lock(&cs40l26->lock);

	ctrl_name = cs40l26->sysfs_fw.ctrl_name;

	mutex_unlock(&cs40l26->lock);

	return sysfs_emit(buf, "%s\n", ctrl_name);
}

static ssize_t fw_ctrl_name_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);

	if (strlen(buf) > CS40L26_COEFF_NAME_MAX_LEN) {
		dev_err(cs40l26->dev, "Control name %s longer than 64 char limit\n", buf);
		return cs40l26_log_err(cs40l26, -E2BIG, CS40L26_ERR_TYPE_SYSFS, __func__);
	}

	mutex_lock(&cs40l26->lock);

	memset(cs40l26->sysfs_fw.ctrl_name, 0, CS40L26_COEFF_NAME_MAX_LEN);

	strscpy(cs40l26->sysfs_fw.ctrl_name, buf, count);

	mutex_unlock(&cs40l26->lock);

	return count;
}
static DEVICE_ATTR_RW(fw_ctrl_name);

static ssize_t fw_ctrl_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int error;
	u32 reg;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_reg(cs40l26->dsp, cs40l26->sysfs_fw.ctrl_name,
			cs40l26->sysfs_fw.block_type, cs40l26->sysfs_fw.algo_id, &reg);

	mutex_unlock(&cs40l26->lock);

	return error ? cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__) :
			sysfs_emit(buf, "0x%08X\n", reg);
}
static DEVICE_ATTR_RO(fw_ctrl_reg);

static ssize_t fw_ctrl_size_words_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	size_t nbytes;
	int error;

	mutex_lock(&cs40l26->lock);

	error = cl_dsp_get_length(cs40l26->dsp, cs40l26->sysfs_fw.ctrl_name,
			cs40l26->sysfs_fw.block_type, cs40l26->sysfs_fw.algo_id, &nbytes);

	mutex_unlock(&cs40l26->lock);

	return error ? cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__) :
			sysfs_emit(buf, "%zd\n", nbytes / CL_DSP_BYTES_PER_WORD);
}
static DEVICE_ATTR_RO(fw_ctrl_size_words);

static int cs40l26_fw_ctrl_properties(struct cs40l26_private *cs40l26, u32 *reg,
		size_t *num_words, bool show)
{
	size_t nbytes;
	int error;
	u32 flags;

	error = cl_dsp_get_flags(cs40l26->dsp, cs40l26->sysfs_fw.ctrl_name,
			cs40l26->sysfs_fw.block_type, cs40l26->sysfs_fw.algo_id, &flags);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__);

	if (show && !(flags & CL_DSP_HALO_FLAG_READ))
		return cs40l26_log_err(cs40l26, -EPERM, CS40L26_ERR_TYPE_SYSFS, __func__);
	else if (!show && (flags & CL_DSP_HALO_FLAG_VOLATILE || !(flags & CL_DSP_HALO_FLAG_WRITE)))
		return cs40l26_log_err(cs40l26, -EPERM, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = cl_dsp_get_length(cs40l26->dsp, cs40l26->sysfs_fw.ctrl_name,
			cs40l26->sysfs_fw.block_type, cs40l26->sysfs_fw.algo_id, &nbytes);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__);

	*num_words = nbytes / CL_DSP_BYTES_PER_WORD;

	error = cl_dsp_get_reg(cs40l26->dsp, cs40l26->sysfs_fw.ctrl_name,
			cs40l26->sysfs_fw.block_type, cs40l26->sysfs_fw.algo_id, reg);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_FW, __func__);

	return 0;
}

static ssize_t fw_ctrl_val_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg = 0, *val = NULL;
	int at = 0, error, i;
	size_t num_words = 0;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_fw_ctrl_properties(cs40l26, &reg, &num_words, true);
	if (error)
		goto err_mutex;

	val = kcalloc(num_words, sizeof(u32), GFP_KERNEL);
	if (!val) {
		error = -ENOMEM;
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_INIT, __func__);
		goto err_mutex;
	}

	error = regmap_bulk_read(cs40l26->regmap, reg, val, num_words);
	if (error) {
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_CP, __func__);
		goto err_free;
	}

	for (i = 0; i < num_words; i++)
		at += sysfs_emit_at(buf, at, "0x%08X\n", val[i]);

err_free:
	kfree(val);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : at;
}

static ssize_t fw_ctrl_val_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg = 0, *val = NULL, wcount = 0;
	size_t num_words = 0;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_fw_ctrl_properties(cs40l26, &reg, &num_words, false);
	if (error)
		goto err_mutex;

	val = kcalloc(num_words, sizeof(u32), GFP_KERNEL);
	if (!val) {
		error = -ENOMEM;
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_INIT, __func__);
		goto err_mutex;
	}

	error = cs40l26_process_word_buf(cs40l26, buf, val, num_words, &wcount, NULL);
	if (error)
		goto err_free;

	error = regmap_bulk_write(cs40l26->regmap, reg, val, num_words);
	if (error)
		cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_CP, __func__);

err_free:
	kfree(val);

err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? error : count;
}
static DEVICE_ATTR_RW(fw_ctrl_val);

static ssize_t fw_mem_block_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 fw_mem_block_type;

	mutex_lock(&cs40l26->lock);

	fw_mem_block_type = cs40l26->sysfs_fw.block_type;

	mutex_unlock(&cs40l26->lock);

	return sysfs_emit(buf, "0x%04X\n", fw_mem_block_type);
}

static ssize_t fw_mem_block_type_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 block_type;
	int error;

	error = kstrtou32(buf, 16, &block_type);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	switch (block_type) {
	case CL_DSP_XM_UNPACKED_TYPE:
	case CL_DSP_YM_UNPACKED_TYPE:
	case CL_DSP_PM_PACKED_TYPE:
	case CL_DSP_XM_PACKED_TYPE:
	case CL_DSP_YM_PACKED_TYPE:
		break;
	default:
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);
	}

	mutex_lock(&cs40l26->lock);

	cs40l26->sysfs_fw.block_type = block_type;

	mutex_unlock(&cs40l26->lock);

	return count;
}
static DEVICE_ATTR_RW(fw_mem_block_type);

static ssize_t rth_latch_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 cmd, latch;
	int error;

	if (cs40l26->revid != CS40L26_REVID_A1 && cs40l26->revid != CS40L26_REVID_B1)
		return cs40l26_log_err(cs40l26, -EPERM, CS40L26_ERR_TYPE_SYSFS, __func__);

	error = kstrtou32(buf, 10, &latch);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_SYSFS, __func__);

	switch (latch) {
	case CS40L26_RTH_LATCH_MBOX:
		cmd = CS40L26_DSP_MBOX_CMD_RTH_UPDATE_MBOX;
		break;
	case CS40L26_RTH_LATCH_GPI:
		cmd = CS40L26_DSP_MBOX_CMD_RTH_UPDATE_GPI;
		break;
	default:
		return cs40l26_log_err(cs40l26, -EINVAL, CS40L26_ERR_TYPE_SYSFS, __func__);
	}

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_PM, __func__);

	mutex_lock(&cs40l26->lock);

	error = cs40l26_mailbox_write(cs40l26, cmd);

	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cs40l26->dev);

	return error ? cs40l26_log_err(cs40l26, error, CS40L26_ERR_TYPE_DSP, __func__) : count;
}
static DEVICE_ATTR_WO(rth_latch);

static struct attribute *cs40l26_dev_attrs_fw[] = {
	&dev_attr_fw_algo_id.attr,
	&dev_attr_fw_ctrl_name.attr,
	&dev_attr_fw_ctrl_reg.attr,
	&dev_attr_fw_ctrl_size_words.attr,
	&dev_attr_fw_ctrl_val.attr,
	&dev_attr_fw_mem_block_type.attr,
	&dev_attr_rth_latch.attr,
	NULL,
};

static struct attribute_group cs40l26_dev_attr_fw_group = {
	.name = "firmware",
	.attrs = cs40l26_dev_attrs_fw,
};

const struct attribute_group *cs40l26_attr_groups[] = {
	&cs40l26_dev_attr_group,
	&cs40l26_dev_attr_cal_group,
	&cs40l26_dev_attr_dlog_group,
	&cs40l26_dev_attr_fw_group,
	NULL,
};
