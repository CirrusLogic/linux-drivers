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
	unsigned int num_waves;
	bool vibe_init_success;
	struct gpio_desc *reset_gpio;
	struct cs40l20_platform_data pdata;
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

static int cs40l20_index_get(struct cs40l20_private *cs40l20,
		unsigned int index_reg)
{
	int ret;
	unsigned int val;

	mutex_lock(&cs40l20->lock);

	switch (index_reg) {
	case CS40L20_VIBEGEN_GPIO1_RINDEX:
	case CS40L20_VIBEGEN_GPIO1_FINDEX:
		ret = regmap_read(cs40l20->regmap, index_reg, &val);
		if (!ret)
			ret = val;
		break;
	default:
		ret = cs40l20->cp_trigger_index;
	}

	mutex_unlock(&cs40l20->lock);

	return ret;
}

static int cs40l20_index_set(struct cs40l20_private *cs40l20,
		unsigned int index_reg, unsigned int index)
{
	int ret;

	mutex_lock(&cs40l20->lock);

	if (index > (cs40l20->num_waves - 1))
		ret = -EACCES;
	else {

		switch (index_reg) {
		case CS40L20_VIBEGEN_GPIO1_RINDEX:
		case CS40L20_VIBEGEN_GPIO1_FINDEX:
			ret = regmap_write(cs40l20->regmap, index_reg, index);
			break;
		default:
			cs40l20->cp_trigger_index = index;
			ret = 0;
		}
	}

	mutex_unlock(&cs40l20->lock);

	return ret;
}

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

	return snprintf(buf, PAGE_SIZE, "%d\n", cs40l20_index_get(cs40l20, 0));
}

static ssize_t cs40l20_cp_trigger_index_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	int ret, index;

	ret = kstrtoint(buf, 10, &index);
	if (ret) {
		pr_err("Invalid input: %d\n", ret);
		return ret;
	}

	ret = cs40l20_index_set(cs40l20, 0, index);
	if (ret) {
		pr_err("Failed to set index to %d: %d\n", index, ret);
		return ret;
	}

	return count;
}

static ssize_t cs40l20_gpio1_rise_index_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			cs40l20_index_get(cs40l20,
				CS40L20_VIBEGEN_GPIO1_RINDEX));
}

static ssize_t cs40l20_gpio1_rise_index_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	int ret, index;

	ret = kstrtoint(buf, 10, &index);
	if (ret) {
		pr_err("Invalid input: %d\n", ret);
		return ret;
	}

	ret = cs40l20_index_set(cs40l20, CS40L20_VIBEGEN_GPIO1_RINDEX, index);
	if (ret) {
		pr_err("Failed to set index to %d: %d\n", index, ret);
		return ret;
	}

	return count;
}

static ssize_t cs40l20_gpio1_fall_index_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			cs40l20_index_get(cs40l20,
				CS40L20_VIBEGEN_GPIO1_FINDEX));
}

static ssize_t cs40l20_gpio1_fall_index_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct cs40l20_private *cs40l20 = cs40l20_get_private(dev);
	int ret, index;

	ret = kstrtoint(buf, 10, &index);
	if (ret) {
		pr_err("Invalid input: %d\n", ret);
		return ret;
	}

	ret = cs40l20_index_set(cs40l20, CS40L20_VIBEGEN_GPIO1_FINDEX, index);
	if (ret) {
		pr_err("Failed to set index to %d: %d\n", index, ret);
		return ret;
	}

	return count;
}

static DEVICE_ATTR(cp_trigger_index, 0660, cs40l20_cp_trigger_index_show,
		cs40l20_cp_trigger_index_store);
static DEVICE_ATTR(gpio1_rise_index, 0660, cs40l20_gpio1_rise_index_show,
		cs40l20_gpio1_rise_index_store);
static DEVICE_ATTR(gpio1_fall_index, 0660, cs40l20_gpio1_fall_index_show,
		cs40l20_gpio1_fall_index_store);

static struct attribute *cs40l20_dev_attrs[] = {
	&dev_attr_cp_trigger_index.attr,
	&dev_attr_gpio1_rise_index.attr,
	&dev_attr_gpio1_fall_index.attr,
	NULL,
};

static struct attribute_group cs40l20_dev_attr_group = {
	.attrs = cs40l20_dev_attrs,
};

static void cs40l20_vibe_start_worker(struct work_struct *work)
{
	struct cs40l20_private *cs40l20 =
		container_of(work, struct cs40l20_private, vibe_start_work);
	int ret;
	unsigned int control_reg;

	mutex_lock(&cs40l20->lock);

	control_reg = (cs40l20->cp_trigger_index == 0) ?
			CS40L20_VIBEGEN_TRIG_MS : CS40L20_VIBEGEN_TRIG_INDEX;

	ret = regmap_write(cs40l20->regmap,
			control_reg, cs40l20->cp_trigger_index);
	if (ret)
		dev_err(cs40l20->dev, "Failed to start playback\n");

	mutex_unlock(&cs40l20->lock);
}

static void cs40l20_vibe_stop_worker(struct work_struct *work)
{
	struct cs40l20_private *cs40l20 =
		container_of(work, struct cs40l20_private, vibe_stop_work);
	int ret;

	mutex_lock(&cs40l20->lock);

	ret = regmap_write(cs40l20->regmap, CS40L20_VIBEGEN_STOP, 1);
	if (ret)
		dev_err(cs40l20->dev, "Failed to stop playback\n");

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

	mutex_init(&cs40l20->lock);

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

static void cs40l20_dsp_start(struct cs40l20_private *cs40l20)
{
	int ret;
	struct regmap *regmap = cs40l20->regmap;
	struct device *dev = cs40l20->dev;
	unsigned int val;

	ret = regmap_update_bits(regmap, CS40L20_DSP1_CCM_CORE_CTRL,
			CS40L20_DSP1_EN_MASK, 1 << CS40L20_DSP1_EN_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to enable DSP\n");
		return;
	}

	ret = regmap_read(regmap, CS40L20_VIBEGEN_FW_STATE, &val);
	if (ret) {
		dev_err(dev, "Failed to read haptics algorithm status\n");
		return;
	}

	if (val == CS40L20_VIBEGEN_FW_RUN)
		dev_info(dev, "Haptics algorithm started\n");
	else {
		dev_err(dev, "Failed to start haptics algorithm\n");
		return;
	}

	ret = regmap_write(regmap, CS40L20_VIBEGEN_TIME_MS,
			CS40L20_VIBEGEN_TIME_MS_MAX);
	if (ret) {
		dev_err(dev, "Failed to extend playback timeout\n");
		return;
	}

	ret = regmap_read(regmap, CS40L20_VIBEGEN_NUM_WAVES, &val);
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
	int ret;
	unsigned int i;

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
		pos += (CS40L20_WT_DBLK_TYPE_SIZE
				+ CS40L20_WT_DBLK_UNUSED_SIZE);

		block_length = fw->data[pos]
				+ (fw->data[pos + 1] << 8)
				+ (fw->data[pos + 2] << 16)
				+ (fw->data[pos + 3] << 24);
		pos += CS40L20_WT_DBLK_LENGTH_SIZE;

		if (block_type == CS40L20_XM_UNPACKED_TYPE) {
			ret = cs40l20_raw_write(cs40l20,
					CS40L20_VIBEGEN_WAVE_TABLE,
					&fw->data[pos], block_length,
					CS40L20_MAX_WLEN);
			if (ret) {
				dev_err(dev,
					"Failed to write XM_UNPACKED memory\n");
				goto err_rls_fw;
			}
		}

		pos += block_length;
	}

skip_loading:
	cs40l20_dsp_start(cs40l20);
err_rls_fw:
	release_firmware(fw);
}

static void cs40l20_firmware_load(const struct firmware *fw, void *context)
{
	int ret;
	struct cs40l20_private *cs40l20 = (struct cs40l20_private *)context;
	struct device *dev = cs40l20->dev;
	unsigned int pos = CS40L20_FW_FILE_HEADER_SIZE;
	unsigned int block_offset, block_length;
	char block_type;

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
		}

		pos += block_length;
	}

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
	cs40l20->vibe_init_success = false;

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

	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "cs40l20.wmfw",
			dev, GFP_KERNEL, cs40l20, cs40l20_firmware_load);

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
	.volatile_reg = cs40l20_volatile_reg,
	.readable_reg = cs40l20_readable_reg,
	.cache_type = REGCACHE_RBTREE,
};

static int cs40l20_i2c_probe(struct i2c_client *i2c_client,
				const struct i2c_device_id *id)
{
	int ret;
	struct cs40l20_private *cs40l20;
	struct device *dev = &i2c_client->dev;
	struct cs40l20_platform_data *pdata = dev_get_platdata(dev);
	unsigned int reg_devid, reg_revid, reg_otpid, i;

	cs40l20 = devm_kzalloc(dev, sizeof(struct cs40l20_private), GFP_KERNEL);
	if (!cs40l20)
		return -ENOMEM;

	cs40l20->dev = dev;
	dev_set_drvdata(dev, cs40l20);
	i2c_set_clientdata(i2c_client, cs40l20);

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

	ret = regmap_read(cs40l20->regmap, CS40L20_OTPID, &reg_otpid);
	if (ret) {
		dev_err(dev, "Failed to read OTP ID\n");
		goto err;
	}

	dev_info(dev, "Cirrus Logic CS40L20 revision %02X\n", reg_revid >> 8);

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

		mutex_destroy(&cs40l20->lock);
	}

	gpiod_set_value_cansleep(cs40l20->reset_gpio, 0);

	regulator_bulk_disable(cs40l20->num_supplies, cs40l20->supplies);

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
