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

static ssize_t cs40l26_dsp_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	char str[CS40L26_DSP_STATE_STR_LEN];
	u8 dsp_state;
	int ret;

	pm_runtime_get_sync(cs40l26->dev);

	ret = cs40l26_dsp_state_get(cs40l26, &dsp_state);
	if (ret)
		return ret;

	switch (dsp_state) {
	case CS40L26_DSP_STATE_HIBERNATE:
		strncpy(str, "Hibernate", CS40L26_DSP_STATE_STR_LEN);
		break;
	case CS40L26_DSP_STATE_SHUTDOWN:
		strncpy(str, "Shutdown", CS40L26_DSP_STATE_STR_LEN);
		break;
	case CS40L26_DSP_STATE_STANDBY:
		strncpy(str, "Standby", CS40L26_DSP_STATE_STR_LEN);
		break;
	case CS40L26_DSP_STATE_ACTIVE:
		strncpy(str, "Active", CS40L26_DSP_STATE_STR_LEN);
		break;
	default:
		dev_err(cs40l26->dev, "DSP state %u is invalid\n", dsp_state);
		return -EINVAL;
	}

	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return snprintf(buf, PAGE_SIZE, "DSP state: %s\n", str);
}
static DEVICE_ATTR(dsp_state, 0660, cs40l26_dsp_state_show, NULL);

static ssize_t cs40l26_halo_heartbeat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, halo_heartbeat;
	int ret;

	pm_runtime_get_sync(cs40l26->dev);

	ret = cl_dsp_get_reg(cs40l26->dsp, "HALO_HEARTBEAT",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_FW_ID, &reg);
	if (ret)
		return ret;

	ret = regmap_read(cs40l26->regmap, reg, &halo_heartbeat);
	if (ret)
		return ret;

	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", halo_heartbeat);
}
static DEVICE_ATTR(halo_heartbeat, 0660, cs40l26_halo_heartbeat_show, NULL);

static ssize_t cs40l26_fw_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);

	pm_runtime_get_sync(cs40l26->dev);

	if (cs40l26->fw_mode != CS40L26_FW_MODE_ROM
			&& cs40l26->fw_mode != CS40L26_FW_MODE_RAM) {
		dev_err(cs40l26->dev, "Invalid firmware mode: %u\n",
				cs40l26->fw_mode);
		return -EINVAL;
	}

	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	return snprintf(buf, PAGE_SIZE, "Firmware is in %s mode\n",
		cs40l26->fw_mode == CS40L26_FW_MODE_ROM ? "ROM" : "RAM");
}
static DEVICE_ATTR(fw_mode, 0660, cs40l26_fw_mode_show, NULL);

static ssize_t cs40l26_pm_timeout_ms_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 timeout_ms;
	int ret;

	pm_runtime_get_sync(cs40l26->dev);

	ret = cs40l26_pm_timeout_ms_get(cs40l26, &timeout_ms);

	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%u\n", timeout_ms);
}

static ssize_t cs40l26_pm_timeout_ms_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 timeout_ms;
	int ret;

	ret = kstrtou32(buf, 10, &timeout_ms);
	if (ret || timeout_ms < CS40L26_PM_TIMEOUT_MS_MIN)
		return -EINVAL;

	pm_runtime_get_sync(cs40l26->dev);

	ret = cs40l26_pm_timeout_ms_set(cs40l26, timeout_ms);

	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);

	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR(pm_timeout_ms, 0660, cs40l26_pm_timeout_ms_show,
		cs40l26_pm_timeout_ms_store);

static ssize_t cs40l26_vibe_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int ret = 0;
	char str[10];

	mutex_lock(&cs40l26->lock);

	switch (cs40l26->vibe_state) {
	case CS40L26_VIBE_STATE_STOPPED:
		strncpy(str, "Stopped", 10);
		break;
	case CS40L26_VIBE_STATE_HAPTIC:
		strncpy(str, "Haptic", 10);
		break;
	case CS40L26_VIBE_STATE_ASP:
		strncpy(str, "ASP", 10);
		break;
	default:
		dev_err(cs40l26->dev, "Invalid vibe state: %u\n",
				cs40l26->vibe_state);
		ret = -EINVAL;
	}

	mutex_unlock(&cs40l26->lock);

	if (ret)
		return ret;
	else
		return snprintf(buf, PAGE_SIZE, "Vibe state: %s\n", str);
}
static DEVICE_ATTR(vibe_state, 0660, cs40l26_vibe_state_show, NULL);

static struct attribute *cs40l26_dev_attrs[] = {
	&dev_attr_dsp_state.attr,
	&dev_attr_halo_heartbeat.attr,
	&dev_attr_fw_mode.attr,
	&dev_attr_pm_timeout_ms.attr,
	&dev_attr_vibe_state.attr,
	NULL,
};

struct attribute_group cs40l26_dev_attr_group = {
	.name = "default",
	.attrs = cs40l26_dev_attrs,
};
