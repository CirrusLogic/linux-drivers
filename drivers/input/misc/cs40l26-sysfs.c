// SPDX-License-Identifier: GPL-2.0
//
// cs40l26-syfs.c -- CS40L26 Boosted Haptic Driver with Integrated DSP and
// Waveform Memory with Advanced Closed Loop Algorithms and LRA protection
//
// Copyright 2020 Cirrus Logic, Inc.
//
// Author: Fred Treven <fred.treven@cirrus.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation.

#include <linux/mfd/cs40l26.h>

static ssize_t cs40l26_halo_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	u32 reg, halo_state;
	int ret;

	ret = cl_dsp_get_reg(cs40l26->dsp, "HALO_STATE",
			CL_DSP_XM_UNPACKED_TYPE, cs40l26->dsp->fw_desc->id,
			&reg);
	if (ret)
		return ret;

	ret = cs40l26_dsp_read(cs40l26, reg, &halo_state);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", halo_state);
}

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
static DEVICE_ATTR(halo_state, 0660, cs40l26_halo_state_show, NULL);
static DEVICE_ATTR(halo_heartbeat, 0660, cs40l26_halo_heartbeat_show, NULL);

static struct attribute *cs40l26_dev_attrs[] = {
	&dev_attr_fw_mode.attr,
	&dev_attr_halo_state.attr,
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

static ssize_t cs40l26_debug_irq_mask1_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	ret = regmap_write(cs40l26->regmap, CS40L26_IRQ1_MASK_1, val);
	if (ret)
		return ret;

	ret = cs40l26_iseq_update(cs40l26, CS40L26_ISEQ_MASK1);
	if (ret)
		return ret;

	return count;
}

static ssize_t cs40l26_debug_irq_mask1_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(cs40l26->regmap, CS40L26_IRQ1_MASK_1, &val);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "0x%08X\n", val);
}

static ssize_t cs40l26_debug_irq_mask2_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	ret = regmap_write(cs40l26->regmap, CS40L26_IRQ1_MASK_2, val);
	if (ret)
		return ret;

	ret = cs40l26_iseq_update(cs40l26, CS40L26_ISEQ_MASK1);
	if (ret)
		return ret;

	return count;
}

static ssize_t cs40l26_debug_irq_mask2_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(cs40l26->regmap, CS40L26_IRQ1_MASK_2, &val);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "0x%08X\n", val);
}

static ssize_t cs40l26_debug_irq_frc1_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	ret = regmap_write(cs40l26->regmap, CS40L26_IRQ1_FRC_1, val);
	if (ret)
		return ret;

	ret = regmap_write(cs40l26->regmap, CS40L26_IRQ1_FRC_1, 0x0);
	if (ret)
		return ret;

	return count;
}

static ssize_t cs40l26_debug_irq_frc1_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(cs40l26->regmap, CS40L26_IRQ1_FRC_1, &val);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "0x%08X\n", val);
}

static ssize_t cs40l26_debug_irq_frc2_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	ret = regmap_write(cs40l26->regmap, CS40L26_IRQ1_FRC_2, val);
	if (ret)
		return ret;

	ret = regmap_write(cs40l26->regmap, CS40L26_IRQ1_FRC_2, 0x0);
	if (ret)
		return ret;

	return count;
}

static ssize_t cs40l26_debug_irq_frc2_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(cs40l26->regmap, CS40L26_IRQ1_FRC_2, &val);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "0x%08X\n", val);
}

static DEVICE_ATTR(debug_irq_frc2, 0660, cs40l26_debug_irq_frc2_show,
		cs40l26_debug_irq_frc2_store);
static DEVICE_ATTR(debug_irq_frc1, 0660, cs40l26_debug_irq_frc1_show,
		cs40l26_debug_irq_frc1_store);
static DEVICE_ATTR(debug_irq_mask2, 0660, cs40l26_debug_irq_mask2_show,
		cs40l26_debug_irq_mask2_store);
static DEVICE_ATTR(debug_irq_mask1, 0660, cs40l26_debug_irq_mask1_show,
		cs40l26_debug_irq_mask1_store);
static DEVICE_ATTR(debug_hibernate, 0660, cs40l26_debug_hibernate_show,
		cs40l26_debug_hibernate_store);

static struct attribute *cs40l26_debug_dev_attrs[] = {
	&dev_attr_debug_irq_frc2.attr,
	&dev_attr_debug_irq_frc1.attr,
	&dev_attr_debug_irq_mask2.attr,
	&dev_attr_debug_irq_mask1.attr,
	&dev_attr_debug_hibernate.attr,
	NULL,
};

struct attribute_group cs40l26_debug_dev_attr_group = {
	.name = "debug",
	.attrs = cs40l26_debug_dev_attrs,
};
