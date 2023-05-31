/*
 * cs40l20.c -- CS40L20 Haptics Driver
 *
 * Copyright 2017 Cirrus Logic, Inc.
 *
 * Author: Jeff LaBundy <jeff.labundy@cirrus.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/platform_data/cs40l20.h>

#include "cs40l20.h"

#ifdef CONFIG_ANDROID_TIMED_OUTPUT
#include "../staging/android/timed_output.h"
#include <linux/hrtimer.h>
#else
#include <linux/leds.h>
#endif /* CONFIG_ANDROID_TIMED_OUTPUT */

struct cs40l20_private {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_bulk_data supplies[2];
	unsigned int num_supplies;
	struct work_struct vibe_start_work;
	struct work_struct vibe_stop_work;
	struct workqueue_struct *vibe_workqueue;
	struct mutex lock;
	unsigned int cp_trigger_index;
	unsigned int cp_trailer_index;
	unsigned int num_waves;
	unsigned int vibegen_id;
	unsigned int vibegen_rev;
	unsigned int wt_limit_xm;
	unsigned int wt_limit_ym;
	bool vibe_init_success;
	struct gpio_desc *reset_gpio;
	struct cs40l20_platform_data pdata;
	struct list_head coeff_desc_head;
	unsigned char diag_state;
	unsigned int f0_measured;
	unsigned int redc_measured;
	unsigned int dig_scale;
#ifdef CONFIG_ANDROID_TIMED_OUTPUT
	struct timed_output_dev timed_dev;
	struct hrtimer vibe_timer;
#else
	struct led_classdev led_dev;
#endif /* CONFIG_ANDROID_TIMED_OUTPUT */
};

static const char * const cs40l20_supplies[] = {
	"VA",
	"VP",
};

static struct cs40l20_private *cs40l20_get_private(struct device *dev)
{
#ifdef CONFIG_ANDROID_TIMED_OUTPUT
	/* timed output device does not register under a parent device */
	return container_of(dev_get_drvdata(dev),
			struct cs40l20_private, timed_dev);
#else
	return dev_get_drvdata(dev);
#endif /* CONFIG_ANDROID_TIMED_OUTPUT */
}

static ssize_t cs40l20_cp_trigger_index_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", cs40l20->cp_trigger_index);
}

static ssize_t cs40l20_cp_trigger_index_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	int ret;
	unsigned int index;

	ret = kstrtou32(buf, 10, &index);
	if (ret)
		return -EINVAL;

	if ((index & CS40L20_INDEX_MASK) >= cs40l20->num_waves
				&& index != CS40L20_INDEX_DIAG)
		return -EINVAL;

	cs40l20->cp_trigger_index = index;

	return count;
}

static unsigned int cs40l20_dsp_reg(struct cs40l20_private *cs40l20,
			const char *coeff_name, const unsigned char block_type)
{
	struct cs40l20_coeff_desc *coeff_desc;

	list_for_each_entry(coeff_desc, &cs40l20->coeff_desc_head, list) {
		if (strcmp(coeff_desc->name, coeff_name))
			continue;
		if (coeff_desc->block_type != block_type)
			continue;

		return coeff_desc->reg;
	}

	/* return an identifiable register that is known to be read-only */
	return CS40L20_DEVID;
}

static ssize_t cs40l20_gpio1_rise_index_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	int ret;
	unsigned int val;

	mutex_lock(&cs40l20->lock);
	ret = regmap_read(cs40l20->regmap,
			cs40l20_dsp_reg(cs40l20, "INDEXBUTTONPRESS",
				CS40L20_XM_UNPACKED_TYPE), &val);
	mutex_unlock(&cs40l20->lock);

	if (ret) {
		pr_err("Failed to read index\n");
		return ret;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t cs40l20_gpio1_rise_index_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	int ret;
	unsigned int index;

	ret = kstrtou32(buf, 10, &index);
	if (ret)
		return -EINVAL;

	if (index > (cs40l20->num_waves - 1))
		return -EINVAL;

	mutex_lock(&cs40l20->lock);
	ret = regmap_write(cs40l20->regmap,
			cs40l20_dsp_reg(cs40l20, "INDEXBUTTONPRESS",
				CS40L20_XM_UNPACKED_TYPE), index);
	mutex_unlock(&cs40l20->lock);

	if (ret) {
		pr_err("Failed to write index\n");
		return ret;
	}

	return count;
}

static ssize_t cs40l20_gpio1_fall_index_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	int ret;
	unsigned int val;

	mutex_lock(&cs40l20->lock);
	ret = regmap_read(cs40l20->regmap,
			cs40l20_dsp_reg(cs40l20, "INDEXBUTTONRELEASE",
				CS40L20_XM_UNPACKED_TYPE), &val);
	mutex_unlock(&cs40l20->lock);

	if (ret) {
		pr_err("Failed to read index\n");
		return ret;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t cs40l20_gpio1_fall_index_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	int ret;
	unsigned int index;

	ret = kstrtou32(buf, 10, &index);
	if (ret)
		return -EINVAL;

	if (index > (cs40l20->num_waves - 1))
		return -EINVAL;

	mutex_lock(&cs40l20->lock);
	ret = regmap_write(cs40l20->regmap,
			cs40l20_dsp_reg(cs40l20, "INDEXBUTTONRELEASE",
				CS40L20_XM_UNPACKED_TYPE), index);
	mutex_unlock(&cs40l20->lock);

	if (ret) {
		pr_err("Failed to write index\n");
		return ret;
	}

	return count;
}

static ssize_t cs40l20_f0_measured_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	unsigned char diag_state;
	unsigned int f0_measured;

	mutex_lock(&cs40l20->lock);

	diag_state = cs40l20->diag_state;
	f0_measured = cs40l20->f0_measured;

	mutex_unlock(&cs40l20->lock);

	if (diag_state != CS40L20_DIAG_STATE_DONE)
		return -ENODATA;

	return snprintf(buf, PAGE_SIZE, "%d\n", f0_measured);
}

static ssize_t cs40l20_f0_stored_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	int ret;
	unsigned int val;

	mutex_lock(&cs40l20->lock);
	ret = regmap_read(cs40l20->regmap,
			cs40l20_dsp_reg(cs40l20, "F0_STORED",
				CS40L20_XM_UNPACKED_TYPE), &val);
	mutex_unlock(&cs40l20->lock);

	if (ret) {
		pr_err("Failed to read stored f0\n");
		return ret;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t cs40l20_f0_stored_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	int ret;
	unsigned int val;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return -EINVAL;

	mutex_lock(&cs40l20->lock);
	ret = regmap_write(cs40l20->regmap,
			cs40l20_dsp_reg(cs40l20, "F0_STORED",
				CS40L20_XM_UNPACKED_TYPE), val);
	mutex_unlock(&cs40l20->lock);

	if (ret) {
		pr_err("Failed to store f0\n");
		return ret;
	}

	return count;
}

static ssize_t cs40l20_redc_measured_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	unsigned char diag_state;
	unsigned int redc_measured;

	mutex_lock(&cs40l20->lock);

	diag_state = cs40l20->diag_state;
	redc_measured = cs40l20->redc_measured;

	mutex_unlock(&cs40l20->lock);

	if (diag_state != CS40L20_DIAG_STATE_DONE)
		return -ENODATA;

	return snprintf(buf, PAGE_SIZE, "%d\n", redc_measured);
}

static ssize_t cs40l20_redc_stored_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	int ret;
	unsigned int val;

	mutex_lock(&cs40l20->lock);
	ret = regmap_read(cs40l20->regmap,
			cs40l20_dsp_reg(cs40l20, "REDC_STORED",
				CS40L20_XM_UNPACKED_TYPE), &val);
	mutex_unlock(&cs40l20->lock);

	if (ret) {
		pr_err("Failed to read stored ReDC\n");
		return ret;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t cs40l20_redc_stored_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	int ret;
	unsigned int val;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return -EINVAL;

	mutex_lock(&cs40l20->lock);
	ret = regmap_write(cs40l20->regmap,
			cs40l20_dsp_reg(cs40l20, "REDC_STORED",
				CS40L20_XM_UNPACKED_TYPE), val);
	mutex_unlock(&cs40l20->lock);

	if (ret) {
		pr_err("Failed to store ReDC\n");
		return ret;
	}

	return count;
}

static ssize_t cs40l20_dig_scale_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", cs40l20->dig_scale);
}

static ssize_t cs40l20_dig_scale_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	int ret;
	unsigned int dig_scale;

	ret = kstrtou32(buf, 10, &dig_scale);
	if (ret)
		return -EINVAL;

	if (dig_scale > CS40L20_DIG_SCALE_MAX)
		return -EINVAL;

	mutex_lock(&cs40l20->lock);
	ret = regmap_update_bits(cs40l20->regmap, CS40L20_AMP_DIG_VOL_CTRL,
			CS40L20_AMP_VOL_PCM_MASK,
			((0x800 - dig_scale) & 0x7FF)
				<< CS40L20_AMP_VOL_PCM_SHIFT);
	mutex_unlock(&cs40l20->lock);

	if (ret) {
		pr_err("Failed to store digital scale\n");
		return ret;
	}

	cs40l20->dig_scale = dig_scale;

	return count;
}

static ssize_t cs40l20_heartbeat_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	int ret;
	unsigned int val;

	mutex_lock(&cs40l20->lock);
	ret = regmap_read(cs40l20->regmap,
			cs40l20_dsp_reg(cs40l20, "HALO_HEARTBEAT",
				CS40L20_XM_UNPACKED_TYPE), &val);
	mutex_unlock(&cs40l20->lock);

	if (ret) {
		pr_err("Failed to read heartbeat\n");
		return ret;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static DEVICE_ATTR(cp_trigger_index, 0660, cs40l20_cp_trigger_index_show,
		cs40l20_cp_trigger_index_store);
static DEVICE_ATTR(gpio1_rise_index, 0660, cs40l20_gpio1_rise_index_show,
		cs40l20_gpio1_rise_index_store);
static DEVICE_ATTR(gpio1_fall_index, 0660, cs40l20_gpio1_fall_index_show,
		cs40l20_gpio1_fall_index_store);
static DEVICE_ATTR(f0_measured, 0660, cs40l20_f0_measured_show, NULL);
static DEVICE_ATTR(f0_stored, 0660, cs40l20_f0_stored_show,
		cs40l20_f0_stored_store);
static DEVICE_ATTR(redc_measured, 0660, cs40l20_redc_measured_show, NULL);
static DEVICE_ATTR(redc_stored, 0660, cs40l20_redc_stored_show,
		cs40l20_redc_stored_store);
static DEVICE_ATTR(dig_scale, 0660, cs40l20_dig_scale_show,
		cs40l20_dig_scale_store);
static DEVICE_ATTR(heartbeat, 0660, cs40l20_heartbeat_show, NULL);

static struct attribute *cs40l20_dev_attrs[] = {
	&dev_attr_cp_trigger_index.attr,
	&dev_attr_gpio1_rise_index.attr,
	&dev_attr_gpio1_fall_index.attr,
	&dev_attr_f0_measured.attr,
	&dev_attr_f0_stored.attr,
	&dev_attr_redc_measured.attr,
	&dev_attr_redc_stored.attr,
	&dev_attr_dig_scale.attr,
	&dev_attr_heartbeat.attr,
	NULL,
};

static struct attribute_group cs40l20_dev_attr_group = {
	.attrs = cs40l20_dev_attrs,
};

static int cs40l20_diag_capture(struct cs40l20_private *cs40l20)
{
	struct regmap *regmap = cs40l20->regmap;
	int ret;

	/* this function expects to be called from a locked worker function */
	if (!mutex_is_locked(&cs40l20->lock))
		return -EACCES;

	if (cs40l20->diag_state != CS40L20_DIAG_STATE_RUN)
		return -ENODATA;

	ret = regmap_read(regmap, cs40l20_dsp_reg(cs40l20, "F0",
			CS40L20_XM_UNPACKED_TYPE), &cs40l20->f0_measured);
	if (ret)
		return ret;

	ret = regmap_read(regmap, cs40l20_dsp_reg(cs40l20, "REDC",
			CS40L20_XM_UNPACKED_TYPE), &cs40l20->redc_measured);
	if (ret)
		return ret;

	cs40l20->diag_state = CS40L20_DIAG_STATE_DONE;

	return 0;
}

static void cs40l20_vibe_start_worker(struct work_struct *work)
{
	struct cs40l20_private *cs40l20 =
		container_of(work, struct cs40l20_private, vibe_start_work);
	struct regmap *regmap = cs40l20->regmap;
	struct device *dev = cs40l20->dev;
	int ret;

	mutex_lock(&cs40l20->lock);

	/* gracefully exit if diagnostics stimulus is interrupted */
	if (cs40l20->cp_trailer_index == CS40L20_INDEX_DIAG) {
		cs40l20->diag_state = CS40L20_DIAG_STATE_INIT;
		goto err_mutex;
	}

	cs40l20->cp_trailer_index = cs40l20->cp_trigger_index;

	switch (cs40l20->cp_trailer_index) {
	case 0x0000:
	case 0x8000 ... 0xFFFE:
		ret = regmap_write(regmap,
				cs40l20_dsp_reg(cs40l20, "TRIGGER_MS",
					CS40L20_XM_UNPACKED_TYPE),
					cs40l20->cp_trailer_index
						& CS40L20_INDEX_MASK);
		if (ret)
			dev_err(dev, "Failed to start playback\n");
		break;

	case 0x0001 ... 0x7FFF:
		ret = regmap_write(regmap,
				cs40l20_dsp_reg(cs40l20, "TRIGGERINDEX",
					CS40L20_XM_UNPACKED_TYPE),
					cs40l20->cp_trailer_index);
		if (ret)
			dev_err(dev, "Failed to start playback\n");
		break;

	case CS40L20_INDEX_DIAG:
		cs40l20->diag_state = CS40L20_DIAG_STATE_INIT;

		ret = regmap_update_bits(cs40l20->regmap,
				CS40L20_AMP_DIG_VOL_CTRL,
				CS40L20_AMP_VOL_PCM_MASK, 0);
		if (ret) {
			dev_err(dev, "Failed to reset digital scale\n");
			goto err_mutex;
		}

		ret = regmap_write(regmap,
				cs40l20_dsp_reg(cs40l20, "CLOSED_LOOP",
					CS40L20_XM_UNPACKED_TYPE), 0);
		if (ret) {
			dev_err(dev, "Failed to disable closed-loop mode\n");
			goto err_mutex;
		}

		ret = regmap_write(regmap,
				cs40l20_dsp_reg(cs40l20, "STIMULUS_MODE",
					CS40L20_XM_UNPACKED_TYPE), 1);
		if (ret) {
			dev_err(dev, "Failed to enable stimulus mode\n");
			goto err_mutex;
		}

		msleep(CS40L20_DIAG_STATE_DELAY_MS);

		ret = regmap_write(regmap,
				cs40l20_dsp_reg(cs40l20, "CLOSED_LOOP",
					CS40L20_XM_UNPACKED_TYPE), 1);
		if (ret) {
			dev_err(dev, "Failed to enable closed-loop mode\n");
			goto err_mutex;
		}
		cs40l20->diag_state = CS40L20_DIAG_STATE_RUN;

		break;

	default:
		dev_err(dev, "Invalid wavetable index\n");
	}

err_mutex:
	mutex_unlock(&cs40l20->lock);
}

static void cs40l20_vibe_stop_worker(struct work_struct *work)
{
	struct cs40l20_private *cs40l20 =
		container_of(work, struct cs40l20_private, vibe_stop_work);
	struct regmap *regmap = cs40l20->regmap;
	struct device *dev = cs40l20->dev;
	int ret;

	mutex_lock(&cs40l20->lock);

	switch (cs40l20->cp_trailer_index) {
	case CS40L20_INDEX_DIAG:
		ret = cs40l20_diag_capture(cs40l20);
		if (ret)
			dev_err(dev, "Failed to capture f0 and ReDC\n");

		ret = regmap_write(regmap,
				cs40l20_dsp_reg(cs40l20, "STIMULUS_MODE",
					CS40L20_XM_UNPACKED_TYPE), 0);
		if (ret)
			dev_err(dev, "Failed to disable stimulus mode\n");

		ret = regmap_update_bits(cs40l20->regmap,
				CS40L20_AMP_DIG_VOL_CTRL,
				CS40L20_AMP_VOL_PCM_MASK,
				((0x800 - cs40l20->dig_scale) & 0x7FF)
					<< CS40L20_AMP_VOL_PCM_SHIFT);
		if (ret)
			dev_err(dev, "Failed to restore digital scale\n");
		break;

	default:
		ret = regmap_write(regmap,
				cs40l20_dsp_reg(cs40l20, "ENDPLAYBACK",
					CS40L20_XM_UNPACKED_TYPE), 1);
		if (ret)
			dev_err(dev, "Failed to stop playback\n");
	}

	cs40l20->cp_trailer_index = 0;

	mutex_unlock(&cs40l20->lock);
}

#ifdef CONFIG_ANDROID_TIMED_OUTPUT
/* vibration callback for timed output device */
static void cs40l20_vibe_enable(struct timed_output_dev *sdev, int timeout)
{
	struct cs40l20_private *cs40l20 =
		container_of(sdev, struct cs40l20_private, timed_dev);

	if (timeout > 0) {
		hrtimer_start(&cs40l20->vibe_timer,
			ktime_set(timeout / 1000, (timeout % 1000) * 1000000),
			HRTIMER_MODE_REL);
		queue_work(cs40l20->vibe_workqueue, &cs40l20->vibe_start_work);
	} else {
		hrtimer_cancel(&cs40l20->vibe_timer);
		queue_work(cs40l20->vibe_workqueue, &cs40l20->vibe_stop_work);
	}
}

static int cs40l20_vibe_get_time(struct timed_output_dev *sdev)
{
	struct cs40l20_private *cs40l20 =
		container_of(sdev, struct cs40l20_private, timed_dev);
	int ret = 0;

	if (hrtimer_active(&cs40l20->vibe_timer))
		ret = ktime_to_ms(hrtimer_get_remaining(&cs40l20->vibe_timer));

	return ret;
}

static enum hrtimer_restart cs40l20_vibe_timer(struct hrtimer *timer)
{
	struct cs40l20_private *cs40l20 =
		container_of(timer, struct cs40l20_private, vibe_timer);

	queue_work(cs40l20->vibe_workqueue, &cs40l20->vibe_stop_work);

	return HRTIMER_NORESTART;
}

static void cs40l20_create_timed_output(struct cs40l20_private *cs40l20)
{
	int ret;
	struct timed_output_dev *timed_dev = &cs40l20->timed_dev;
	struct hrtimer *vibe_timer = &cs40l20->vibe_timer;
	struct device *dev = cs40l20->dev;

	timed_dev->name = CS40L20_DEVICE_NAME;
	timed_dev->enable = cs40l20_vibe_enable;
	timed_dev->get_time = cs40l20_vibe_get_time;

	ret = timed_output_dev_register(timed_dev);
	if (ret) {
		dev_err(dev, "Failed to register timed output device: %d\n",
			ret);
		return;
	}

	hrtimer_init(vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer->function = cs40l20_vibe_timer;

	ret = sysfs_create_group(&cs40l20->timed_dev.dev->kobj,
			&cs40l20_dev_attr_group);
	if (ret) {
		dev_err(dev, "Failed to create sysfs group: %d\n", ret);
		return;
	}
}
#else
/* vibration callback for LED device */
static void cs40l20_vibe_brightness_set(struct led_classdev *led_cdev,
		enum led_brightness brightness)
{
	struct cs40l20_private *cs40l20 =
		container_of(led_cdev, struct cs40l20_private, led_dev);

	switch (brightness) {
	case LED_OFF:
		queue_work(cs40l20->vibe_workqueue, &cs40l20->vibe_stop_work);
		break;
	default:
		queue_work(cs40l20->vibe_workqueue, &cs40l20->vibe_start_work);
	}
}

static void cs40l20_create_led(struct cs40l20_private *cs40l20)
{
	int ret;
	struct led_classdev *led_dev = &cs40l20->led_dev;
	struct device *dev = cs40l20->dev;

	led_dev->name = CS40L20_DEVICE_NAME;
	led_dev->max_brightness = LED_FULL;
	led_dev->brightness_set = cs40l20_vibe_brightness_set;
	led_dev->default_trigger = "transient";

	ret = led_classdev_register(dev, led_dev);
	if (ret) {
		dev_err(dev, "Failed to register LED device: %d\n", ret);
		return;
	}

	ret = sysfs_create_group(&cs40l20->dev->kobj, &cs40l20_dev_attr_group);
	if (ret) {
		dev_err(dev, "Failed to create sysfs group: %d\n", ret);
		return;
	}
}
#endif /* CONFIG_ANDROID_TIMED_OUTPUT */

static void cs40l20_vibe_init(struct cs40l20_private *cs40l20)
{
#ifdef CONFIG_ANDROID_TIMED_OUTPUT
	cs40l20_create_timed_output(cs40l20);
#else
	cs40l20_create_led(cs40l20);
#endif /* CONFIG_ANDROID_TIMED_OUTPUT */

	cs40l20->vibe_workqueue =
		alloc_ordered_workqueue("vibe_workqueue", WQ_HIGHPRI);
	if (!cs40l20->vibe_workqueue) {
		dev_err(cs40l20->dev, "Failed to allocate workqueue\n");
		return;
	}

	INIT_WORK(&cs40l20->vibe_start_work, cs40l20_vibe_start_worker);
	INIT_WORK(&cs40l20->vibe_stop_work, cs40l20_vibe_stop_worker);

	cs40l20->vibe_init_success = true;
}

static int cs40l20_coeff_init(struct cs40l20_private *cs40l20)
{
	int ret, i;
	struct regmap *regmap = cs40l20->regmap;
	struct device *dev = cs40l20->dev;
	struct cs40l20_coeff_desc *coeff_desc;
	unsigned int val, num_algos, algo_id, algo_rev;
	unsigned int xm_base, xm_size, ym_base, ym_size;
	unsigned int reg = CS40L20_XM_FW_ID;

	ret = regmap_read(regmap, CS40L20_XM_NUM_ALGOS, &num_algos);
	if (ret) {
		dev_err(dev, "Failed to read number of algorithms\n");
		return ret;
	}

	if (num_algos > CS40L20_NUM_ALGOS_MAX) {
		dev_err(dev, "Invalid number of algorithms\n");
		return -EINVAL;
	}

	/* add one extra iteration to account for system algorithm */
	for (i = 0; i < (num_algos + 1); i++) {
		ret = regmap_read(regmap,
				reg + CS40L20_ALGO_ID_OFFSET, &algo_id);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d ID\n", i);
			return ret;
		}

		ret = regmap_read(regmap,
				reg + CS40L20_ALGO_REV_OFFSET, &algo_rev);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d revision\n", i);
			return ret;
		}

		/* discern firmware revision from system algorithm */
		if (i == 0) {
			if (algo_rev < CS40L20_FW_REV_MIN) {
				dev_err(dev,
					"Invalid firmware revision: %d.%d.%d\n",
					(algo_rev & 0xFF0000) >> 16,
					(algo_rev & 0xFF00) >> 8,
					algo_rev & 0xFF);
				return -EINVAL;
			}
			dev_info(dev, "Firmware revision %d.%d.%d\n",
					(algo_rev & 0xFF0000) >> 16,
					(algo_rev & 0xFF00) >> 8,
					algo_rev & 0xFF);
		}

		ret = regmap_read(regmap,
				reg + CS40L20_ALGO_XM_BASE_OFFSET, &xm_base);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d XM_BASE\n", i);
			return ret;
		}

		ret = regmap_read(regmap,
				reg + CS40L20_ALGO_XM_SIZE_OFFSET, &xm_size);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d XM_SIZE\n", i);
			return ret;
		}

		ret = regmap_read(regmap,
				reg + CS40L20_ALGO_YM_BASE_OFFSET, &ym_base);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d YM_BASE\n", i);
			return ret;
		}

		ret = regmap_read(regmap,
				reg + CS40L20_ALGO_YM_SIZE_OFFSET, &ym_size);
		if (ret) {
			dev_err(dev, "Failed to read algo. %d YM_SIZE\n", i);
			return ret;
		}

		list_for_each_entry(coeff_desc,
			&cs40l20->coeff_desc_head, list) {

			if (coeff_desc->parent_id != algo_id)
				continue;

			switch (coeff_desc->block_type) {
			case CS40L20_XM_UNPACKED_TYPE:
				coeff_desc->reg = CS40L20_DSP1_XMEM_UNPACK24_0
					+ xm_base * 4
					+ coeff_desc->block_offset * 4;
				if (!strcmp(coeff_desc->name, "WAVETABLE")) {
					cs40l20->wt_limit_xm = (xm_size
						- coeff_desc->block_offset) * 4;
					cs40l20->vibegen_id = algo_id;
					cs40l20->vibegen_rev = algo_rev;
				}
				break;
			case CS40L20_YM_UNPACKED_TYPE:
				coeff_desc->reg = CS40L20_DSP1_YMEM_UNPACK24_0
					+ ym_base * 4
					+ coeff_desc->block_offset * 4;
				if (!strcmp(coeff_desc->name, "WAVETABLEYM"))
					cs40l20->wt_limit_ym = (ym_size
						- coeff_desc->block_offset) * 4;
				break;
			}

			dev_dbg(dev, "Found control %s at 0x%08X\n",
				coeff_desc->name, coeff_desc->reg);
		}

		/* system algo. contains one extra register (num. algos.) */
		if (i)
			reg += CS40L20_ALGO_ENTRY_SIZE;
		else
			reg += (CS40L20_ALGO_ENTRY_SIZE + 4);
	}

	ret = regmap_read(regmap, reg, &val);
	if (ret) {
		dev_err(dev, "Failed to read list terminator\n");
		return ret;
	}

	if (val != CS40L20_ALGO_LIST_TERM) {
		dev_err(dev, "Invalid list terminator: 0x%X\n", val);
		return -EINVAL;
	}

	dev_info(dev, "Maximum wavetable size: %d bytes (XM), %d bytes (YM)\n",
			cs40l20->wt_limit_xm / 4 * 3,
			cs40l20->wt_limit_ym / 4 * 3);

	return 0;
}

static void cs40l20_dsp_start(struct cs40l20_private *cs40l20)
{
	int ret;
	struct regmap *regmap = cs40l20->regmap;
	struct device *dev = cs40l20->dev;
	unsigned int val;
	int dsp_timeout = CS40L20_DSP_TIMEOUT_COUNT;

	ret = regmap_update_bits(regmap, CS40L20_DSP1_CCM_CORE_CTRL,
			CS40L20_DSP1_EN_MASK, 1 << CS40L20_DSP1_EN_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to enable DSP\n");
		return;
	}

	while (dsp_timeout > 0) {
		usleep_range(10000, 10100);

		ret = regmap_read(regmap, cs40l20_dsp_reg(cs40l20, "HALO_STATE",
				CS40L20_XM_UNPACKED_TYPE), &val);
		if (ret) {
			dev_err(dev, "Failed to read DSP status\n");
			return;
		}

		if (val == CS40L20_HALO_STATE_RUNNING)
			break;

		dsp_timeout--;
	}

	if (dsp_timeout == 0) {
		dev_err(dev, "Timed out with DSP status = %d\n", val);
		return;
	}

	dev_info(dev, "Haptics algorithm started\n");

	ret = regmap_write(regmap, cs40l20_dsp_reg(cs40l20, "TIMEOUT_MS",
			CS40L20_XM_UNPACKED_TYPE), CS40L20_TIMEOUT_MS_MAX);
	if (ret) {
		dev_err(dev, "Failed to extend playback timeout\n");
		return;
	}

	ret = regmap_read(regmap, cs40l20_dsp_reg(cs40l20, "NUMBEROFWAVES",
			CS40L20_XM_UNPACKED_TYPE), &val);
	if (ret) {
		dev_err(dev, "Failed to count wavetable entries\n");
		return;
	}

	if (val) {
		dev_info(dev, "Counted %d entries in wavetable\n", val);
		cs40l20->num_waves = val;
	} else {
		dev_err(dev, "Wavetable is empty\n");
		return;
	}

	cs40l20_vibe_init(cs40l20);
}

static int cs40l20_raw_write(struct cs40l20_private *cs40l20, unsigned int reg,
		const void *val, size_t val_len, size_t limit)
{
	int ret, i;

	/* split "val" into smaller writes not to exceed "limit" in length */
	for (i = 0; i < val_len; i += limit) {
		ret = regmap_raw_write(cs40l20->regmap, (reg + i), (val + i),
				(val_len - i) > limit ? limit : (val_len - i));
		if (ret)
			break;
	}

	return ret;
}

static void cs40l20_waveform_load(const struct firmware *fw, void *context)
{
	int ret;
	struct cs40l20_private *cs40l20 = (struct cs40l20_private *)context;
	struct device *dev = cs40l20->dev;
	unsigned int pos = CS40L20_WT_FILE_HEADER_SIZE;
	unsigned int block_type, block_length;
	unsigned int algo_id, algo_rev;

	if (!fw) {
		dev_warn(dev, "Using default wavetable\n");
		goto skip_loading;
	}

	if (memcmp(fw->data, "WMDR", 4)) {
		dev_err(dev, "Failed to recognize waveform file\n");
		goto err_rls_fw;
	}

	dev_info(dev, "Found custom wavetable\n");

	while (pos < fw->size) {

		/* block offset is not used here */
		pos += CS40L20_WT_DBLK_OFFSET_SIZE;

		block_type = fw->data[pos]
				+ (fw->data[pos + 1] << 8);
		pos += CS40L20_WT_DBLK_TYPE_SIZE;

		algo_id = fw->data[pos]
				+ (fw->data[pos + 1] << 8)
				+ (fw->data[pos + 2] << 16)
				+ (fw->data[pos + 3] << 24);
		pos += CS40L20_WT_ALGO_ID_SIZE;

		algo_rev = fw->data[pos]
				+ (fw->data[pos + 1] << 8)
				+ (fw->data[pos + 2] << 16)
				+ (fw->data[pos + 3] << 24);
		pos += CS40L20_WT_ALGO_REV_SIZE;

		/* sample rate is not used here */
		pos += CS40L20_WT_SAMPLE_RATE_SIZE;

		block_length = fw->data[pos]
				+ (fw->data[pos + 1] << 8)
				+ (fw->data[pos + 2] << 16)
				+ (fw->data[pos + 3] << 24);
		pos += CS40L20_WT_DBLK_LENGTH_SIZE;

		switch (block_type) {
		case CS40L20_XM_UNPACKED_TYPE:
			if (algo_id != cs40l20->vibegen_id) {
				dev_err(dev, "Invalid algo. ID: 0x%06X\n",
					algo_id);
				goto err_rls_fw;
			}

			if (algo_rev != cs40l20->vibegen_rev) {
				dev_err(dev, "Invalid algo. rev.: %d.%d.%d\n",
					(algo_rev & 0xFF0000) >> 16,
					(algo_rev & 0xFF00) >> 8,
					algo_rev & 0xFF);
				goto err_rls_fw;
			}

			if (block_length > cs40l20->wt_limit_xm) {
				dev_err(dev,
					"Wavetable too large: %d bytes (XM)\n",
					block_length / 4 * 3);
				goto err_rls_fw;
			}

			ret = cs40l20_raw_write(cs40l20,
					cs40l20_dsp_reg(cs40l20, "WAVETABLE",
						CS40L20_XM_UNPACKED_TYPE),
					&fw->data[pos], block_length,
					CS40L20_MAX_WLEN);
			if (ret) {
				dev_err(dev,
					"Failed to write XM_UNPACKED memory\n");
				goto err_rls_fw;
			}
			break;
		case CS40L20_YM_UNPACKED_TYPE:
			if (algo_id != cs40l20->vibegen_id) {
				dev_err(dev, "Invalid algo. ID: 0x%06X\n",
					algo_id);
				goto err_rls_fw;
			}

			if (algo_rev != cs40l20->vibegen_rev) {
				dev_err(dev, "Invalid algo. rev.: %d.%d.%d\n",
					(algo_rev & 0xFF0000) >> 16,
					(algo_rev & 0xFF00) >> 8,
					algo_rev & 0xFF);
				goto err_rls_fw;
			}

			if (block_length > cs40l20->wt_limit_ym) {
				dev_err(dev,
					"Wavetable too large: %d bytes (YM)\n",
					block_length / 4 * 3);
				goto err_rls_fw;
			}

			ret = cs40l20_raw_write(cs40l20,
					cs40l20_dsp_reg(cs40l20, "WAVETABLEYM",
						CS40L20_YM_UNPACKED_TYPE),
					&fw->data[pos], block_length,
					CS40L20_MAX_WLEN);
			if (ret) {
				dev_err(dev,
					"Failed to write YM_UNPACKED memory\n");
				goto err_rls_fw;
			}
			break;
		}

		pos += block_length;
	}

skip_loading:
	cs40l20_dsp_start(cs40l20);
err_rls_fw:
	release_firmware(fw);
}

static int cs40l20_algo_parse(struct cs40l20_private *cs40l20,
		const unsigned char *data)
{
	struct cs40l20_coeff_desc *coeff_desc;
	unsigned int pos = 0;
	unsigned int algo_id, algo_desc_length, coeff_count;
	unsigned int block_offset, block_type, block_length;
	unsigned char algo_name_length;
	int i;

	/* record algorithm ID */
	algo_id = *(data + pos)
			+ (*(data + pos + 1) << 8)
			+ (*(data + pos + 2) << 16)
			+ (*(data + pos + 3) << 24);
	pos += CS40L20_ALGO_ID_SIZE;

	/* skip past algorithm name */
	algo_name_length = *(data + pos);
	pos += ((algo_name_length / 4) * 4) + 4;

	/* skip past algorithm description */
	algo_desc_length = *(data + pos)
			+ (*(data + pos + 1) << 8);
	pos += ((algo_desc_length / 4) * 4) + 4;

	/* record coefficient count */
	coeff_count = *(data + pos)
			+ (*(data + pos + 1) << 8)
			+ (*(data + pos + 2) << 16)
			+ (*(data + pos + 3) << 24);
	pos += CS40L20_COEFF_COUNT_SIZE;

	for (i = 0; i < coeff_count; i++) {
		block_offset = *(data + pos)
				+ (*(data + pos + 1) << 8);
		pos += CS40L20_COEFF_OFFSET_SIZE;

		block_type = *(data + pos)
				+ (*(data + pos + 1) << 8);
		pos += CS40L20_COEFF_TYPE_SIZE;

		block_length = *(data + pos)
				+ (*(data + pos + 1) << 8)
				+ (*(data + pos + 2) << 16)
				+ (*(data + pos + 3) << 24);
		pos += CS40L20_COEFF_LENGTH_SIZE;

		coeff_desc = devm_kzalloc(cs40l20->dev, sizeof(*coeff_desc),
				GFP_KERNEL);
		if (!coeff_desc)
			return -ENOMEM;

		coeff_desc->parent_id = algo_id;
		coeff_desc->block_offset = block_offset;
		coeff_desc->block_type = block_type;

		memcpy(coeff_desc->name, data + pos + 1, *(data + pos));
		coeff_desc->name[*(data + pos)] = '\0';

		list_add(&coeff_desc->list, &cs40l20->coeff_desc_head);

		pos += block_length;
	}

	return 0;
}

static void cs40l20_firmware_load(const struct firmware *fw, void *context)
{
	int ret;
	struct cs40l20_private *cs40l20 = (struct cs40l20_private *)context;
	struct device *dev = cs40l20->dev;
	unsigned int pos = CS40L20_FW_FILE_HEADER_SIZE;
	unsigned int block_offset, block_length;
	unsigned char block_type;

	if (!fw) {
		dev_err(dev, "Failed to request firmware file\n");
		return;
	}

	if (memcmp(fw->data, "WMFW", 4)) {
		dev_err(dev, "Failed to recognize firmware file\n");
		goto err_rls_fw;
	}

	while (pos < fw->size) {

		block_offset = fw->data[pos]
				+ (fw->data[pos + 1] << 8)
				+ (fw->data[pos + 2] << 16);
		pos += CS40L20_FW_DBLK_OFFSET_SIZE;

		block_type = fw->data[pos];
		pos += CS40L20_FW_DBLK_TYPE_SIZE;

		block_length = fw->data[pos]
				+ (fw->data[pos + 1] << 8)
				+ (fw->data[pos + 2] << 16)
				+ (fw->data[pos + 3] << 24);
		pos += CS40L20_FW_DBLK_LENGTH_SIZE;

		switch (block_type) {
		case CS40L20_PM_PACKED_TYPE:
			ret = cs40l20_raw_write(cs40l20,
					CS40L20_DSP1_PMEM_0
						+ block_offset * 5,
					&fw->data[pos], block_length,
					CS40L20_MAX_WLEN);
			if (ret) {
				dev_err(dev,
					"Failed to write PM_PACKED memory\n");
				goto err_rls_fw;
			}
			break;
		case CS40L20_XM_PACKED_TYPE:
			ret = cs40l20_raw_write(cs40l20,
					CS40L20_DSP1_XMEM_PACK_0
						+ block_offset * 3,
					&fw->data[pos], block_length,
					CS40L20_MAX_WLEN);
			if (ret) {
				dev_err(dev,
					"Failed to write XM_PACKED memory\n");
				goto err_rls_fw;
			}
			break;
		case CS40L20_YM_PACKED_TYPE:
			ret = cs40l20_raw_write(cs40l20,
					CS40L20_DSP1_YMEM_PACK_0
						+ block_offset * 3,
					&fw->data[pos], block_length,
					CS40L20_MAX_WLEN);
			if (ret) {
				dev_err(dev,
					"Failed to write YM_PACKED memory\n");
				goto err_rls_fw;
			}
			break;
		case CS40L20_ALGO_INFO_TYPE:
			ret = cs40l20_algo_parse(cs40l20, &fw->data[pos]);
			if (ret) {
				dev_err(dev,
					"Failed to parse algorithm: %d\n", ret);
				goto err_rls_fw;
			}
			break;
		}

		pos += block_length;
	}

	ret = cs40l20_coeff_init(cs40l20);
	if (ret)
		goto err_rls_fw;

	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "cs40l20.bin",
			dev, GFP_KERNEL, cs40l20, cs40l20_waveform_load);
err_rls_fw:
	release_firmware(fw);
}

static int cs40l20_boost_config(struct cs40l20_private *cs40l20,
		int boost_ind, int boost_cap, int boost_ipk)
{
	int ret;
	unsigned char bst_lbst_val, bst_cbst_range, bst_ipk_scaled;
	struct regmap *regmap = cs40l20->regmap;
	struct device *dev = cs40l20->dev;

	switch (boost_ind) {
	case 1000:	/* 1.0 uH */
		bst_lbst_val = 0;
		break;
	case 1200:	/* 1.2 uH */
		bst_lbst_val = 1;
		break;
	case 1500:	/* 1.5 uH */
		bst_lbst_val = 2;
		break;
	case 2200:	/* 2.2 uH */
		bst_lbst_val = 3;
		break;
	default:
		dev_err(dev, "Invalid boost inductor value: %d nH\n",
				boost_ind);
		return -EINVAL;
	}

	switch (boost_cap) {
	case 0 ... 19:
		bst_cbst_range = 0;
		break;
	case 20 ... 50:
		bst_cbst_range = 1;
		break;
	case 51 ... 100:
		bst_cbst_range = 2;
		break;
	case 101 ... 200:
		bst_cbst_range = 3;
		break;
	default:	/* 201 uF and greater */
		bst_cbst_range = 4;
	}

	ret = regmap_update_bits(regmap, CS40L20_BSTCVRT_COEFF,
			CS40L20_BST_K1_MASK,
			cs40l20_bst_k1_table[bst_lbst_val][bst_cbst_range]
				<< CS40L20_BST_K1_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to write boost K1 coefficient\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS40L20_BSTCVRT_COEFF,
			CS40L20_BST_K2_MASK,
			cs40l20_bst_k2_table[bst_lbst_val][bst_cbst_range]
				<< CS40L20_BST_K2_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to write boost K2 coefficient\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS40L20_BSTCVRT_SLOPE_LBST,
			CS40L20_BST_SLOPE_MASK,
			cs40l20_bst_slope_table[bst_lbst_val]
				<< CS40L20_BST_SLOPE_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to write boost slope coefficient\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS40L20_BSTCVRT_SLOPE_LBST,
			CS40L20_BST_LBST_VAL_MASK,
			bst_lbst_val << CS40L20_BST_LBST_VAL_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to write boost inductor value\n");
		return ret;
	}

	if ((boost_ipk < 1600) || (boost_ipk > 4500)) {
		dev_err(dev, "Invalid boost inductor peak current: %d mA\n",
				boost_ipk);
		return -EINVAL;
	}
	bst_ipk_scaled = ((boost_ipk - 1600) / 50) + 0x10;

	ret = regmap_update_bits(regmap, CS40L20_BSTCVRT_PEAK_CUR,
			CS40L20_BST_IPK_MASK,
			bst_ipk_scaled << CS40L20_BST_IPK_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to write boost inductor peak current\n");
		return ret;
	}

	return 0;
}

static const struct reg_sequence cs40l20_mpu_config[] = {
	{CS40L20_DSP1_MPU_LOCK_CONFIG,	CS40L20_MPU_UNLOCK_CODE1},
	{CS40L20_DSP1_MPU_LOCK_CONFIG,	CS40L20_MPU_UNLOCK_CODE2},
	{CS40L20_DSP1_MPU_XM_ACCESS0,	0xFFFFFFFF},
	{CS40L20_DSP1_MPU_YM_ACCESS0,	0xFFFFFFFF},
	{CS40L20_DSP1_MPU_WNDW_ACCESS0,	0xFFFFFFFF},
	{CS40L20_DSP1_MPU_XREG_ACCESS0,	0xFFFFFFFF},
	{CS40L20_DSP1_MPU_YREG_ACCESS0,	0xFFFFFFFF},
	{CS40L20_DSP1_MPU_WNDW_ACCESS1,	0xFFFFFFFF},
	{CS40L20_DSP1_MPU_XREG_ACCESS1,	0xFFFFFFFF},
	{CS40L20_DSP1_MPU_YREG_ACCESS1,	0xFFFFFFFF},
	{CS40L20_DSP1_MPU_WNDW_ACCESS2,	0xFFFFFFFF},
	{CS40L20_DSP1_MPU_XREG_ACCESS2,	0xFFFFFFFF},
	{CS40L20_DSP1_MPU_YREG_ACCESS2,	0xFFFFFFFF},
	{CS40L20_DSP1_MPU_WNDW_ACCESS3,	0xFFFFFFFF},
	{CS40L20_DSP1_MPU_XREG_ACCESS3,	0xFFFFFFFF},
	{CS40L20_DSP1_MPU_YREG_ACCESS3,	0xFFFFFFFF},
	{CS40L20_DSP1_MPU_LOCK_CONFIG,	0x00000000}
};

static int cs40l20_init(struct cs40l20_private *cs40l20)
{
	int ret;
	struct regmap *regmap = cs40l20->regmap;
	struct device *dev = cs40l20->dev;

	cs40l20->cp_trigger_index = 0;
	cs40l20->cp_trailer_index = 0;
	cs40l20->vibe_init_success = false;
	cs40l20->diag_state = CS40L20_DIAG_STATE_INIT;
	cs40l20->dig_scale = 0;

	if (cs40l20->pdata.refclk_gpio2) {
		ret = regmap_update_bits(regmap, CS40L20_GPIO_PAD_CONTROL,
				CS40L20_GP2_CTRL_MASK,
				CS40L20_GP2_CTRL_MCLK
					<< CS40L20_GP2_CTRL_SHIFT);
		if (ret) {
			dev_err(dev, "Failed to select GPIO2 function\n");
			return ret;
		}

		ret = regmap_update_bits(regmap, CS40L20_PLL_CLK_CTRL,
				CS40L20_PLL_REFCLK_SEL_MASK,
				CS40L20_PLL_REFCLK_SEL_MCLK
					<< CS40L20_PLL_REFCLK_SEL_SHIFT);
		if (ret) {
			dev_err(dev, "Failed to select clock source\n");
			return ret;
		}
	}

	ret = cs40l20_boost_config(cs40l20, cs40l20->pdata.boost_ind,
			cs40l20->pdata.boost_cap, cs40l20->pdata.boost_ipk);
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, CS40L20_DAC_PCM1_SRC,
			CS40L20_DAC_PCM1_SRC_MASK,
			CS40L20_DAC_PCM1_SRC_DSP1TX1
				<< CS40L20_DAC_PCM1_SRC_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to route DSP to amplifier\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS40L20_DSP1_RX2_SRC,
			CS40L20_DSP1_RXn_SRC_MASK,
			CS40L20_DSP1_RXn_SRC_VMON
				<< CS40L20_DSP1_RXn_SRC_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to route voltage monitor to DSP\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS40L20_DSP1_RX3_SRC,
			CS40L20_DSP1_RXn_SRC_MASK,
			CS40L20_DSP1_RXn_SRC_IMON
				<< CS40L20_DSP1_RXn_SRC_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to route current monitor to DSP\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS40L20_PWR_CTRL1,
			CS40L20_GLOBAL_EN_MASK, 1 << CS40L20_GLOBAL_EN_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to enable device\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS40L20_DSP1_CCM_CORE_CTRL,
			CS40L20_DSP1_RESET_MASK, 1 << CS40L20_DSP1_RESET_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to reset DSP\n");
		return ret;
	}

	ret = regmap_multi_reg_write(regmap, cs40l20_mpu_config,
			ARRAY_SIZE(cs40l20_mpu_config));
	if (ret) {
		dev_err(dev, "Failed to configure MPU\n");
		return ret;
	}

	INIT_LIST_HEAD(&cs40l20->coeff_desc_head);

	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "cs40l20.wmfw",
			dev, GFP_KERNEL, cs40l20, cs40l20_firmware_load);

	return 0;
}

static int cs40l20_otp_unpack(struct cs40l20_private *cs40l20)
{
	struct regmap *regmap = cs40l20->regmap;
	struct device *dev = cs40l20->dev;
	struct cs40l20_trim trim;
	unsigned char row_offset, col_offset;
	unsigned int val, otp_map;
	unsigned int otp_mem[CS40L20_NUM_OTP_WORDS];
	int otp_timeout = CS40L20_OTP_TIMEOUT_COUNT;
	int ret, i;

	while (otp_timeout > 0) {
		usleep_range(10000, 10100);

		ret = regmap_read(regmap, CS40L20_IRQ1_STATUS4, &val);
		if (ret) {
			dev_err(dev, "Failed to read OTP boot status\n");
			return ret;
		}

		if (val & CS40L20_OTP_BOOT_DONE)
			break;

		otp_timeout--;
	}

	if (otp_timeout == 0) {
		dev_err(dev, "Timed out waiting for OTP boot\n");
		return -ETIME;
	}

	ret = regmap_read(cs40l20->regmap, CS40L20_IRQ1_STATUS3, &val);
	if (ret) {
		dev_err(dev, "Failed to read OTP error status\n");
		return ret;
	}

	if (val & CS40L20_OTP_BOOT_ERR) {
		dev_err(dev, "Encountered fatal OTP error\n");
		return -EIO;
	}

	ret = regmap_read(cs40l20->regmap, CS40L20_OTPID, &val);
	if (ret) {
		dev_err(dev, "Failed to read OTP ID\n");
		return ret;
	}

	/* hard matching against known OTP IDs */
	for (i = 0; i < CS40L20_NUM_OTP_MAPS; i++) {
		if (cs40l20_otp_map[i].id == val) {
			otp_map = i;
			break;
		}
	}

	/* reject unrecognized IDs, including untrimmed devices (OTP ID = 0) */
	if (i == CS40L20_NUM_OTP_MAPS) {
		dev_err(dev, "Unrecognized OTP ID: 0x%01X\n", val);
		return -ENODEV;
	}

	dev_dbg(dev, "Found OTP ID: 0x%01X\n", val);

	ret = regmap_bulk_read(regmap, CS40L20_OTP_MEM0, otp_mem,
			CS40L20_NUM_OTP_WORDS);
	if (ret) {
		dev_err(dev, "Failed to read OTP contents\n");
		return ret;
	}

	ret = regmap_write(regmap, CS40L20_TEST_KEY_CTL,
			CS40L20_TEST_KEY_UNLOCK_CODE1);
	if (ret) {
		dev_err(dev, "Failed to unlock test space (step 1 of 2)\n");
		return ret;
	}

	ret = regmap_write(regmap, CS40L20_TEST_KEY_CTL,
			CS40L20_TEST_KEY_UNLOCK_CODE2);
	if (ret) {
		dev_err(dev, "Failed to unlock test space (step 2 of 2)\n");
		return ret;
	}

	row_offset = cs40l20_otp_map[otp_map].row_start;
	col_offset = cs40l20_otp_map[otp_map].col_start;

	for (i = 0; i < cs40l20_otp_map[otp_map].num_trims; i++) {
		trim = cs40l20_otp_map[otp_map].trim_table[i];

		if (col_offset + trim.size - 1 > 31) {
			/* trim straddles word boundary */
			val = (otp_mem[row_offset] &
					GENMASK(31, col_offset)) >> col_offset;
			val |= (otp_mem[row_offset + 1] &
					GENMASK(col_offset + trim.size - 33, 0))
					<< (32 - col_offset);
		} else {
			/* trim does not straddle word boundary */
			val = (otp_mem[row_offset] &
					GENMASK(col_offset + trim.size - 1,
						col_offset)) >> col_offset;
		}

		/* advance column marker and wrap if necessary */
		col_offset += trim.size;
		if (col_offset > 31) {
			col_offset -= 32;
			row_offset++;
		}

		/* skip blank trims */
		if (trim.reg == 0)
			continue;

		ret = regmap_update_bits(regmap, trim.reg,
				GENMASK(trim.shift + trim.size - 1, trim.shift),
				val << trim.shift);
		if (ret) {
			dev_err(dev, "Failed to write trim %d\n", i + 1);
			return ret;
		}

		dev_dbg(dev, "Trim %d: wrote 0x%X to 0x%08X bits [%d:%d]\n",
				i + 1, val, trim.reg,
				trim.shift + trim.size - 1, trim.shift);
	}

	ret = regmap_write(regmap, CS40L20_TEST_KEY_CTL,
			CS40L20_TEST_KEY_RELOCK_CODE1);
	if (ret) {
		dev_err(dev, "Failed to lock test space (step 1 of 2)\n");
		return ret;
	}

	ret = regmap_write(regmap, CS40L20_TEST_KEY_CTL,
			CS40L20_TEST_KEY_RELOCK_CODE2);
	if (ret) {
		dev_err(dev, "Failed to lock test space (step 2 of 2)\n");
		return ret;
	}

	return 0;
}

static int cs40l20_handle_of_data(struct i2c_client *i2c_client,
		struct cs40l20_platform_data *pdata)
{
	struct device_node *np = i2c_client->dev.of_node;
	struct device *dev = &i2c_client->dev;
	int ret;
	unsigned int out_val;

	if (!np)
		return 0;

	ret = of_property_read_u32(np, "cirrus,boost-ind-nanohenry", &out_val);
	if (ret) {
		dev_err(dev, "Boost inductor value not specified\n");
		return -EINVAL;
	}
	pdata->boost_ind = out_val;

	ret = of_property_read_u32(np, "cirrus,boost-cap-microfarad", &out_val);
	if (ret) {
		dev_err(dev, "Boost capacitance not specified\n");
		return -EINVAL;
	}
	pdata->boost_cap = out_val;

	ret = of_property_read_u32(np, "cirrus,boost-ipk-milliamp", &out_val);
	if (ret) {
		dev_err(dev, "Boost inductor peak current not specified\n");
		return -EINVAL;
	}
	pdata->boost_ipk = out_val;

	pdata->refclk_gpio2 = of_property_read_bool(np, "cirrus,refclk-gpio2");

	return 0;
}

static struct regmap_config cs40l20_regmap = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = CS40L20_LASTREG,
	.reg_defaults = cs40l20_reg,
	.num_reg_defaults = ARRAY_SIZE(cs40l20_reg),
	.precious_reg = cs40l20_precious_reg,
	.volatile_reg = cs40l20_volatile_reg,
	.readable_reg = cs40l20_readable_reg,
	.cache_type = REGCACHE_RBTREE,
};

static int cs40l20_i2c_probe(struct i2c_client *i2c_client,
				const struct i2c_device_id *id)
{
	int ret, i;
	struct cs40l20_private *cs40l20;
	struct device *dev = &i2c_client->dev;
	struct cs40l20_platform_data *pdata = dev_get_platdata(dev);
	unsigned int reg_devid, reg_revid;

	cs40l20 = devm_kzalloc(dev, sizeof(struct cs40l20_private), GFP_KERNEL);
	if (!cs40l20)
		return -ENOMEM;

	cs40l20->dev = dev;
	dev_set_drvdata(dev, cs40l20);
	i2c_set_clientdata(i2c_client, cs40l20);

	mutex_init(&cs40l20->lock);

	cs40l20->regmap = devm_regmap_init_i2c(i2c_client, &cs40l20_regmap);
	if (IS_ERR(cs40l20->regmap)) {
		ret = PTR_ERR(cs40l20->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(cs40l20_supplies); i++)
		cs40l20->supplies[i].supply = cs40l20_supplies[i];

	cs40l20->num_supplies = ARRAY_SIZE(cs40l20_supplies);

	ret = devm_regulator_bulk_get(dev, cs40l20->num_supplies,
			cs40l20->supplies);
	if (ret) {
		dev_err(dev, "Failed to request core supplies: %d\n", ret);
		return ret;
	}

	if (pdata) {
		cs40l20->pdata = *pdata;
	} else {
		pdata = devm_kzalloc(dev, sizeof(struct cs40l20_platform_data),
				GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		if (i2c_client->dev.of_node) {
			ret = cs40l20_handle_of_data(i2c_client, pdata);
			if (ret)
				return ret;

		}
		cs40l20->pdata = *pdata;
	}

	ret = regulator_bulk_enable(cs40l20->num_supplies, cs40l20->supplies);
	if (ret) {
		dev_err(dev, "Failed to enable core supplies: %d\n", ret);
		return ret;
	}

	cs40l20->reset_gpio = devm_gpiod_get_optional(dev, "reset",
			GPIOD_OUT_LOW);
	if (IS_ERR(cs40l20->reset_gpio))
		return PTR_ERR(cs40l20->reset_gpio);

	/* satisfy reset pulse width specification (with margin) */
	usleep_range(2000, 2100);

	gpiod_set_value_cansleep(cs40l20->reset_gpio, 1);

	/* satisfy control port delay specification (with margin) */
	usleep_range(1000, 1100);

	ret = regmap_read(cs40l20->regmap, CS40L20_DEVID, &reg_devid);
	if (ret) {
		dev_err(dev, "Failed to read device ID\n");
		goto err;
	}

	if (reg_devid != CS40L20_CHIP_ID) {
		dev_err(dev, "Failed to recognize device ID: %X\n", reg_devid);
		ret = -ENODEV;
		goto err;
	}

	ret = regmap_read(cs40l20->regmap, CS40L20_REVID, &reg_revid);
	if (ret) {
		dev_err(dev, "Failed to read revision ID\n");
		goto err;
	}

	ret = cs40l20_otp_unpack(cs40l20);
	if (ret)
		goto err;

	dev_info(dev, "Cirrus Logic CS40L20 revision %02X\n", reg_revid);

	ret = cs40l20_init(cs40l20);
	if (ret)
		goto err;

	return 0;
err:
	gpiod_set_value_cansleep(cs40l20->reset_gpio, 0);

	regulator_bulk_disable(cs40l20->num_supplies, cs40l20->supplies);

	return ret;
}

static int cs40l20_i2c_remove(struct i2c_client *i2c_client)
{
	struct cs40l20_private *cs40l20 = i2c_get_clientdata(i2c_client);

	if (cs40l20->vibe_init_success) {
#ifdef CONFIG_ANDROID_TIMED_OUTPUT
		timed_output_dev_unregister(&cs40l20->timed_dev);

		sysfs_remove_group(&cs40l20->timed_dev.dev->kobj,
				&cs40l20_dev_attr_group);
#else
		led_classdev_unregister(&cs40l20->led_dev);

		sysfs_remove_group(&cs40l20->dev->kobj,
				&cs40l20_dev_attr_group);
#endif /* CONFIG_ANDROID_TIMED_OUTPUT */

		cancel_work_sync(&cs40l20->vibe_start_work);
		cancel_work_sync(&cs40l20->vibe_stop_work);

		destroy_workqueue(cs40l20->vibe_workqueue);
	}

	gpiod_set_value_cansleep(cs40l20->reset_gpio, 0);

	regulator_bulk_disable(cs40l20->num_supplies, cs40l20->supplies);

	mutex_destroy(&cs40l20->lock);

	return 0;
}

static const struct of_device_id cs40l20_of_match[] = {
	{.compatible = "cirrus,cs40l20"},
	{},
};

MODULE_DEVICE_TABLE(of, cs40l20_of_match);

static const struct i2c_device_id cs40l20_id[] = {
	{"cs40l20", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cs40l20_id);

static struct i2c_driver cs40l20_i2c_driver = {
	.driver = {
		.name = "cs40l20",
		.of_match_table = cs40l20_of_match,
	},
	.id_table = cs40l20_id,
	.probe = cs40l20_i2c_probe,
	.remove = cs40l20_i2c_remove,
};

module_i2c_driver(cs40l20_i2c_driver);

MODULE_DESCRIPTION("CS40L20 Haptics Driver");
MODULE_AUTHOR("Jeff LaBundy, Cirrus Logic Inc, <jeff.labundy@cirrus.com>");
MODULE_LICENSE("GPL");
