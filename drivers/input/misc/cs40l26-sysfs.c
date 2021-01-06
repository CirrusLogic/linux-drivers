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

	return snprintf(buf, PAGE_SIZE, "DSP state: %s\n", str);
}
static DEVICE_ATTR(dsp_state, 0660, cs40l26_dsp_state_show, NULL);

static ssize_t cs40l26_halo_heartbeat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, halo_heartbeat;
	int ret;

	ret = cl_dsp_get_reg(cs40l26->dsp, "HALO_HEARTBEAT",
			CL_DSP_XM_UNPACKED_TYPE, cs40l26->dsp->fw_desc->id,
			&reg);
	if (ret)
		return ret;

	ret = cs40l26_dsp_read(cs40l26, reg, &halo_heartbeat);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", halo_heartbeat);
}
static DEVICE_ATTR(halo_heartbeat, 0660, cs40l26_halo_heartbeat_show, NULL);

static ssize_t cs40l26_fw_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);

	if (cs40l26->fw_mode != CS40L26_FW_MODE_ROM
			&& cs40l26->fw_mode != CS40L26_FW_MODE_RAM) {
		dev_err(cs40l26->dev, "Invalid firmware mode: %u\n",
				cs40l26->fw_mode);
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE, "Firmware is in %s mode\n",
		cs40l26->fw_mode == CS40L26_FW_MODE_ROM ? "ROM" : "RAM");
}
static DEVICE_ATTR(fw_mode, 0660, cs40l26_fw_mode_show, NULL);

static struct attribute *cs40l26_dev_attrs[] = {
	&dev_attr_fw_mode.attr,
	&dev_attr_dsp_state.attr,
	&dev_attr_halo_heartbeat.attr,
	NULL,
};

struct attribute_group cs40l26_dev_attr_group = {
	.name = "default",
	.attrs = cs40l26_dev_attrs,
};

static ssize_t cs40l26_debug_hibernate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	switch (val) {
	case CS40L26_DISABLE:
		ret = cs40l26_pm_state_transition(cs40l26,
				CS40L26_PM_STATE_PREVENT_HIBERNATE);
		if (ret)
			return ret;
		break;
	case CS40L26_ENABLE:
		ret = cs40l26_pm_state_transition(cs40l26,
				CS40L26_PM_STATE_ALLOW_HIBERNATE);
		break;
	default:
		dev_err(cs40l26->dev, "Entry must be binary value\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t cs40l26_debug_hibernate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	int val;

	if (cs40l26->pm_state == CS40L26_PM_STATE_ALLOW_HIBERNATE)
		val = CS40L26_ENABLE;
	else
		val = CS40L26_DISABLE;

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}
static DEVICE_ATTR(debug_hibernate, 0660, cs40l26_debug_hibernate_show,
	cs40l26_debug_hibernate_store);

static struct attribute *cs40l26_debug_dev_attrs[] = {
	&dev_attr_debug_hibernate.attr,
	NULL,
};

struct attribute_group cs40l26_debug_dev_attr_group = {
	.name = "debug",
	.attrs = cs40l26_debug_dev_attrs,
};
