/*
 * extcon-tacna.c - Extcon driver for Cirrus Logic Tacna codecs
 *
 * Copyright 2017-2018 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/math64.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>

#include <sound/soc.h>

#include <linux/extcon/extcon-tacna.h>
#include <linux/extcon/extcon-tacna-pdata.h>
#include <dt-bindings/extcon/extcon-tacna.h>

#include <linux/irqchip/irq-tacna.h>

#include <linux/mfd/tacna/core.h>
#include <linux/mfd/tacna/pdata.h>
#include <linux/mfd/tacna/registers.h>

#define TACNA_MAX_MICD_RANGE		8

#define TACNA_HPDET_MAX_OHM		10000
#define TACNA_HPDET_MAX_HOHM		(TACNA_HPDET_MAX_OHM * 100)
#define TACNA_HP_SHORT_IMPEDANCE_MIN	4
#define TACNA_EXTCON_LINEOUT_IMP_MIN	5000

#define TACNA_LVL_SEL_SHORT_OR_MIC	0x81

#define TACNA_DEFAULT_MICD_TIMEOUT_MS	2000

#define TACNA_HPDONE_PROBE_INTERVAL_MS	20
#define TACNA_HPDONE_PROBE_COUNT	15

#define TACNA_MICROPHONE_MIN_OHM	1258
#define TACNA_MICROPHONE_MAX_OHM	30000

#define TACNA_HP_TUNING_INVALID		-1

#define CS47L96_OUT_SEL_OUT1L_HP1	0
#define CS47L96_OUT_SEL_OUT1R_HP1	1
#define CS47L96_OUT_SEL_OUT1L_HP2	2
#define CS47L96_OUT_SEL_OUT1R_HP2	3

static const unsigned int tacna_cable[] = {
	EXTCON_MECHANICAL,
	EXTCON_JACK_MICROPHONE,
	EXTCON_JACK_HEADPHONE,
	EXTCON_JACK_LINE_OUT,
	EXTCON_NONE,
};

static const struct tacna_micd_config tacna_micd_default_modes[] = {
	{ TACNA_MICD_SENSE_MICDET1, TACNA_MICD_GND_MICDET2,
	  TACNA_MICD_BIAS_SRC_MICBIAS1A, 0, TACNA_HPD_GND_HPOUTFB2 },
	{ TACNA_MICD_SENSE_MICDET2, TACNA_MICD_GND_MICDET1,
	  TACNA_MICD_BIAS_SRC_MICBIAS1B, 1, TACNA_HPD_GND_HPOUTFB1 },
};

static const unsigned int tacna_default_hpd_pins[4] = {
	[0] = TACNA_HPD_OUT_OUT1L,
	[1] = TACNA_HPD_SENSE_HPDET1,
	[2] = TACNA_HPD_OUT_OUT1R,
	[3] = TACNA_HPD_SENSE_HPDET1,
};

static struct tacna_micd_range tacna_micd_default_ranges[] = {
	{ .max =  70, .key = KEY_MEDIA },
	{ .max = 186, .key = KEY_VOICECOMMAND },
	{ .max = 295, .key = KEY_VOLUMEUP },
	{ .max = 681, .key = KEY_VOLUMEDOWN },
};

/* The number of levels in tacna_micd_levels valid for button thresholds */
#define TACNA_NUM_MICD_BUTTON_LEVELS 64

/* ohms for each micd level */
static const int tacna_micd_levels[] = {
	3, 6, 8, 11, 13, 16, 18, 21, 23, 26, 28, 31, 34, 36, 39, 41, 44, 46,
	49, 52, 54, 57, 60, 62, 65, 67, 70, 73, 75, 78, 81, 83, 89, 94, 100,
	105, 111, 116, 122, 127, 139, 150, 161, 173, 186, 196, 209, 220, 245,
	270, 295, 321, 348, 375, 402, 430, 489, 550, 614, 681, 752, 903, 1071,
	1257, 30000,
};

/*
 * HP calibration data
 * See the datasheet for the meanings of the constants and their values
 */
struct tacna_hpdet_calibration_data {
	int	min;		/* ohms */
	int	max;		/* ohms */
	s64	C0;		/* value * 1000000 */
	s64	C1;		/* value * 10000 */
	s64	C2;		/* not multiplied */
	s64	C3;		/* value * 1000000 */
	s64	C4_x_C3;	/* value * 1000000 */
	s64	C5;		/* value * 1000000 */
	s64	dacval_adjust;
};

static const struct tacna_hpdet_calibration_data tacna_hpdet_ranges[] = {
	{ 4,    30,    1007000,   -72,   4005, 69300000, 381150, 600000, 500000 },
	{ 8,    100,   1007000,   -72,   7975, 69600000, 382800, 600000, 500000 },
	{ 100,  1000,  9744000,   -795,  7300, 62900000, 345950, 600000, 500000 },
	{ 1000, 10000, 100684000, -9494, 7300, 63200000, 347600, 600000, 500000 },
};

static ssize_t tacna_extcon_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tacna_extcon *info = platform_get_drvdata(pdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 info->tacna->hp_impedance_x100[0]);
}

static DEVICE_ATTR(hp1_impedance, 0444, tacna_extcon_show, NULL);

static inline bool tacna_is_lineout(struct tacna_extcon *info)
{
	return info->tacna->hp_impedance_x100[0] >=
		tacna_ohm_to_hohm(TACNA_EXTCON_LINEOUT_IMP_MIN);
}

static void tacna_extcon_input_event_report(struct tacna_extcon *info,
					    int which, bool attached)
{
	if (IS_ENABLED(CONFIG_EXTCON_TACNA_INPUT_EVENT)) {
		switch (which) {
		case EXTCON_MECHANICAL:
			input_report_switch(info->input,
					    SW_JACK_PHYSICAL_INSERT,
					    attached);
			break;
		case EXTCON_JACK_HEADPHONE:
			input_report_switch(info->input,
					    SW_HEADPHONE_INSERT,
					    attached);
			break;
		case EXTCON_JACK_MICROPHONE:
			input_report_switch(info->input,
					    SW_MICROPHONE_INSERT,
					    attached);
			break;
		default: /* ignore all other reports */
			return;
		}

		input_sync(info->input);
	}
}

inline void tacna_extcon_report(struct tacna_extcon *info,
				int which, bool attached)
{
	int ret;

	dev_dbg(info->dev, "Extcon report: %d is %s\n",
		which, attached ? "attached" : "removed");

	ret = extcon_set_state_sync(info->edev, which, attached);
	if (ret != 0)
		dev_warn(info->dev, "Failed to report cable state: %d\n", ret);

	tacna_extcon_input_event_report(info, which, attached);
}
EXPORT_SYMBOL_GPL(tacna_extcon_report);

inline void tacna_extcon_report_removal(struct tacna_extcon *info)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(tacna_cable) - 1; i++) {
		ret = extcon_set_state_sync(info->edev, tacna_cable[i],
					    false);
		if (ret != 0)
			dev_err(info->dev, "Removal report failed: %d\n", ret);

		tacna_extcon_input_event_report(info, tacna_cable[i], false);
	}
}

static enum tacna_accdet_mode tacna_jds_get_mode(struct tacna_extcon *info)
{
	if (info->state)
		return info->state->mode;
	else
		return TACNA_ACCDET_MODE_INVALID;
}

int tacna_jds_set_state(struct tacna_extcon *info,
			const struct tacna_jd_state *new_state)
{
	int ret = 0;

	if (new_state != info->state) {
		if (info->state)
			info->state->stop(info);

		info->state = new_state;

		if (info->state) {
			ret = info->state->start(info);
			if (ret < 0)
				info->state = NULL;
		}
	}

	return ret;
}
EXPORT_SYMBOL_GPL(tacna_jds_set_state);

static void tacna_jds_reading(struct tacna_extcon *info, int val)
{
	int ret;

	ret = info->state->reading(info, val);

	if (ret == -EAGAIN && info->state->restart)
		info->state->restart(info);
}

static inline bool tacna_jds_cancel_timeout(struct tacna_extcon *info)
{
	return cancel_delayed_work_sync(&info->state_timeout_work);
}

static void tacna_jds_start_timeout(struct tacna_extcon *info)
{
	const struct tacna_jd_state *state = info->state;

	if (!state)
		return;

	if (state->timeout_ms && state->timeout) {
		int ms = state->timeout_ms(info);

		schedule_delayed_work(&info->state_timeout_work,
				      msecs_to_jiffies(ms));
	}
}

static void tacna_jds_timeout_work(struct work_struct *work)
{
	struct tacna_extcon *info = container_of(work, struct tacna_extcon,
						 state_timeout_work.work);

	mutex_lock(&info->lock);

	if (!info->state) {
		dev_warn(info->dev, "Spurious timeout in idle state\n");
	} else if (!info->state->timeout) {
		dev_warn(info->dev, "Spurious timeout state.mode=%d\n",
			 info->state->mode);
	} else {
		info->state->timeout(info);
		tacna_jds_start_timeout(info);
	}

	mutex_unlock(&info->lock);
}

static void tacna_extcon_wait_output_wseq(struct tacna_extcon *info,
					  unsigned int mask,
					  unsigned int target)
{
	struct regmap *regmap = info->tacna->regmap;
	unsigned int val;
	int ret;

	ret = regmap_read_poll_timeout(regmap, TACNA_OUTPUT_STATUS_1,
				       val, ((val & mask) == target),
					1000, 20000);
	if (ret)
		dev_warn(info->dev, "Timed out waiting for output state.\n");

	/* HP_CTRL is changed by the write sequence to re-sync the cache */
	regcache_drop_region(regmap, TACNA_HP_CTRL, TACNA_HP_CTRL);
	ret = regmap_read(regmap, TACNA_HP_CTRL, &val);
	if (ret)
		dev_warn(info->dev, "Failed to resync HP_CTRL (%d)\n", ret);
}

static void cs47l96_extcon_hp_clamp(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;
	unsigned int hpd_out_sel;
	int ret;

	ret = regmap_read(tacna->regmap, TACNA_HPDET1_CONTROL1, &hpd_out_sel);
	if (ret)
		dev_warn(info->dev, "Failed to read HPD_OUT_SEL: %d\n", ret);

	hpd_out_sel = (hpd_out_sel & TACNA_HPD_OUT_SEL_MASK) >>
		      TACNA_HPD_OUT_SEL_SHIFT;

	switch (hpd_out_sel) {
	case CS47L96_OUT_SEL_OUT1L_HP1:
	case CS47L96_OUT_SEL_OUT1R_HP1:
		ret = regmap_update_bits(tacna->regmap,
				    TACNA_OUTH_CFG2,
				    TACNA_OUTH_HPAMP_B_FBRES_WELLBIAS_DIS_MASK |
				    TACNA_OUTH_HPAMP_A_FBRES_WELLBIAS_DIS_MASK,
				    TACNA_OUTH_HPAMP_B_FBRES_WELLBIAS_DIS |
				    TACNA_OUTH_HPAMP_A_FBRES_WELLBIAS_DIS);
		if (ret)
			dev_warn(info->dev,
				 "Failed to update OUTH_CFG2: %d\n",
				 ret);

		ret = regmap_update_bits(tacna->regmap,
					 TACNA_OUTH_CFG5,
					 TACNA_EDRE_B_MANUAL_MASK |
					 TACNA_EDRE_A_MANUAL_MASK |
					 TACNA_LOWRES_CLAMP_NICHOLLS_R_EN_MASK |
					 TACNA_LOWRES_CLAMP_NICHOLLS_L_EN_MASK |
					 TACNA_XTALK_CLAMP_NICHOLLS_R_EN_MASK |
					 TACNA_XTALK_CLAMP_NICHOLLS_L_EN_MASK |
					 TACNA_DS_CLAMP_SUMPB_GNDA_MASK |
					 TACNA_DS_CLAMP_SUMNB_GNDA_MASK |
					 TACNA_DS_CLAMP_SUMPA_GNDA_MASK |
					 TACNA_DS_CLAMP_SUMNA_GNDA_MASK,
					 TACNA_EDRE_B_MANUAL |
					 TACNA_EDRE_A_MANUAL);
		if (ret)
			dev_warn(info->dev,
				 "Failed to update OUTH_CFG5: %d\n",
				 ret);

		usleep_range(500, 1000);

		break;
	case CS47L96_OUT_SEL_OUT1L_HP2:
	case CS47L96_OUT_SEL_OUT1R_HP2:
		ret = regmap_update_bits(tacna->regmap,
					 TACNA_HP_CTRL,
					 TACNA_OUT1R_HP2_CLAMP_EN_MASK |
					 TACNA_OUT1L_HP2_CLAMP_EN_MASK,
					 0);
		if (ret)
			dev_warn(info->dev,
				 "Failed to clear OUT1_HP2 clamp: %d\n",
				 ret);
		break;
	default:
		dev_err(info->dev,
			"%u is not a valid HPD_OUT_SEL value.\n",
			hpd_out_sel);
		break;
	}
}

static void cs47l96_extcon_hp_unclamp(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;
	unsigned int hpd_out_sel;
	int ret;

	ret = regmap_read(tacna->regmap, TACNA_HPDET1_CONTROL1, &hpd_out_sel);
	if (ret)
		dev_warn(info->dev, "Failed to read HPD_OUT_SEL: %d\n", ret);

	hpd_out_sel = (hpd_out_sel & TACNA_HPD_OUT_SEL_MASK) >>
		      TACNA_HPD_OUT_SEL_SHIFT;

	switch (hpd_out_sel) {
	case CS47L96_OUT_SEL_OUT1L_HP1:
	case CS47L96_OUT_SEL_OUT1R_HP1:
		ret = regmap_update_bits(tacna->regmap,
					 TACNA_OUTH_CFG5,
					 TACNA_EDRE_B_MANUAL_MASK |
					 TACNA_EDRE_A_MANUAL_MASK,
					 0);
		if (ret)
			dev_warn(info->dev,
				 "Failed to clear EDRE manual: %d\n",
				 ret);

		ret = regmap_update_bits(tacna->regmap,
				    TACNA_OUTH_CFG2,
				    TACNA_OUTH_HPAMP_B_FBRES_WELLBIAS_DIS_MASK |
				    TACNA_OUTH_HPAMP_A_FBRES_WELLBIAS_DIS_MASK,
				    0);
		if (ret)
			dev_warn(info->dev,
				 "Failed to update OUTH_CFG2: %d\n",
				 ret);
		break;
	case CS47L96_OUT_SEL_OUT1L_HP2:
	case CS47L96_OUT_SEL_OUT1R_HP2:
		ret = regmap_update_bits(tacna->regmap,
					 TACNA_HP_CTRL,
					 TACNA_OUT1R_HP2_CLAMP_EN_MASK |
					 TACNA_OUT1L_HP2_CLAMP_EN_MASK,
					 TACNA_OUT1R_HP2_CLAMP_EN |
					 TACNA_OUT1L_HP2_CLAMP_EN);
		if (ret)
			dev_warn(info->dev,
				 "Failed to set OUT1_HP2 clamp: %d\n",
				 ret);
		break;
	default:
		dev_err(info->dev,
			"%u is not a valid HPD_OUT_SEL value.\n",
			hpd_out_sel);
		break;
	}
}

static int cs47l96_extcon_update_out1_state(struct tacna_extcon *info,
					    unsigned int out_state)
{
	struct regmap *regmap = info->tacna->regmap;
	unsigned int reg, mask, hpd_out_sel;
	int ret;
	bool outh_upd = false;

	ret = regmap_read(regmap, TACNA_HPDET1_CONTROL1, &hpd_out_sel);
	if (ret)
		dev_warn(info->dev, "Failed to read HPD_OUT_SEL: %d\n", ret);

	hpd_out_sel = (hpd_out_sel & TACNA_HPD_OUT_SEL_MASK) >>
		      TACNA_HPD_OUT_SEL_SHIFT;

	switch (hpd_out_sel) {
	case CS47L96_OUT_SEL_OUT1L_HP1:
	case CS47L96_OUT_SEL_OUT1R_HP1:
		reg = TACNA_OUTH_ENABLE_1;
		mask = TACNA_OUTH_EN_MASK;

		/* update OUTH state */
		ret = regmap_update_bits_check(regmap, reg, mask,
					       (out_state >> 31), &outh_upd);
		if (ret)
			dev_warn(info->dev, "Failed to update OUTH state: %d\n",
				 ret);

		/*
		 * fall through to update OUT1, as HP1 pins are shared between
		 * OUT1 and OUTH
		 */
	case CS47L96_OUT_SEL_OUT1L_HP2:
	case CS47L96_OUT_SEL_OUT1R_HP2:
		reg = TACNA_OUTPUT_ENABLE_1;
		mask = TACNA_OUT1L_EN_MASK | TACNA_OUT1R_EN_MASK;

		/* update OUT1 state */
		ret = regmap_update_bits(regmap, reg, mask, out_state);
		if (ret)
			dev_warn(info->dev, "Failed to update OUT1 state: %d\n",
				 ret);

		break;
	default:
		return 0;
	}

	if (outh_upd && (out_state >> 31 == 0))
		msleep(100); /* wait for disable to take effect */

	tacna_extcon_wait_output_wseq(info, mask, out_state & mask);

	return ret;
}

static int tacna_extcon_update_out_state(struct tacna_extcon *info,
					 unsigned int out_state)
{
	unsigned int mask;
	int ret;

	switch (info->tacna->type) {
	case CS47L96:
	case CS47L97:
		if (info->pdata->output == 1)
			return cs47l96_extcon_update_out1_state(info,
								out_state);

		/* for outputs other than 1 (OUT1/OUTH), fall through */
	default:
		mask = 0x3 << (2 * info->pdata->output - 1);
		break;
	}

	ret = regmap_update_bits(info->tacna->regmap, TACNA_OUTPUT_ENABLE_1,
				 mask, out_state);
	if (ret)
		dev_warn(info->dev, "Failed to write OUTPUT_ENABLE_1 (%d)\n",
			 ret);

	tacna_extcon_wait_output_wseq(info, mask, out_state & mask);

	return ret;
}

static void tacna_extcon_hp_clamp(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;
	int ret;

	dev_dbg(info->dev, "Setting HP clamp.\n");

	snd_soc_dapm_mutex_lock(tacna->dapm);

	tacna->hpdet_clamp[0] = true;

	/* Keep the HP output stages disabled while doing the clamp */
	ret = tacna_extcon_update_out_state(info, 0);
	if (ret)
		dev_warn(info->dev,
			 "Failed to disable headphone outputs: %d\n",
			 ret);

	ret = regmap_update_bits(tacna->regmap,
				 TACNA_HPDET1_CONTROL1,
				 TACNA_HPD_OVD_EN_MASK,
				 TACNA_HPD_OVD_EN);
	if (ret)
		dev_warn(info->dev, "Failed to write OVD_EN: %d\n", ret);

	switch (tacna->type) {
	case CS47L96:
	case CS47L97:
		cs47l96_extcon_hp_clamp(info);
		break;
	default:
		break;
	}

	snd_soc_dapm_mutex_unlock(tacna->dapm);
}

static void tacna_extcon_hp_unclamp(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;
	int ret;

	dev_dbg(info->dev, "Clearing HP clamp.\n");

	snd_soc_dapm_mutex_lock(tacna->dapm);

	tacna->hpdet_clamp[0] = false;

	switch (tacna->type) {
	case CS47L96:
	case CS47L97:
		cs47l96_extcon_hp_unclamp(info);
		break;
	default:
		break;
	}

	ret = regmap_update_bits(tacna->regmap,
				 TACNA_HPDET1_CONTROL1,
				 TACNA_HPD_OVD_EN_MASK,
				 0);
	if (ret)
		dev_warn(info->dev, "Failed to write OVD_EN: %d\n", ret);

	/* Restore the desired state while not doing the clamp */
	if (!tacna->hpdet_shorted[0]) {
		ret = tacna_extcon_update_out_state(info, tacna->hp_ena);
		if (ret)
			dev_warn(info->dev,
				 "Failed to restore headphone outputs: %d\n",
				 ret);
	}

	snd_soc_dapm_mutex_unlock(tacna->dapm);
}

static const char *tacna_extcon_get_micbias(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;
	unsigned int bias = info->micd_modes[info->micd_mode].bias;

	switch (tacna->type) {
	case CS47L96:
	case CS47L97:
		switch (bias) {
		case 0:
			return "MICBIAS1A";
		case 1:
			return "MICBIAS1B";
		case 2:
			return "MICBIAS1C";
		case 3:
			return "MICBIAS1D";
		case 4:
			return "MICBIAS2A";
		case 5:
			return "MICBIAS2B";
		default:
			return "VOUT_MIC";
		}
	default:
		return NULL;
	}
}

static void tacna_extcon_enable_micbias_pin(struct tacna_extcon *info,
					    const char *widget)
{
	struct tacna *tacna = info->tacna;
	struct snd_soc_dapm_context *dapm = tacna->dapm;
	struct snd_soc_component *component = snd_soc_dapm_to_component(dapm);
	int ret;

	ret = snd_soc_component_force_enable_pin(component, widget);
	if (ret)
		dev_warn(info->dev, "Failed to enable %s: %d\n", widget, ret);

	snd_soc_dapm_sync(dapm);

	dev_dbg(info->dev, "Enabled %s\n", widget);
}

static void tacna_extcon_disable_micbias_pin(struct tacna_extcon *info,
					     const char *widget)
{
	struct tacna *tacna = info->tacna;
	struct snd_soc_dapm_context *dapm = tacna->dapm;
	struct snd_soc_component *component = snd_soc_dapm_to_component(dapm);
	int ret;

	ret = snd_soc_component_disable_pin(component, widget);
	if (ret)
		dev_warn(info->dev, "Failed to enable %s: %d\n", widget, ret);

	snd_soc_dapm_sync(dapm);

	dev_dbg(info->dev, "Disabled %s\n", widget);
}

static void tacna_extcon_enable_micbias(struct tacna_extcon *info)
{
	const char *widget = tacna_extcon_get_micbias(info);

	tacna_extcon_enable_micbias_pin(info, widget);
}

static void tacna_extcon_disable_micbias(struct tacna_extcon *info)
{
	const char *widget = tacna_extcon_get_micbias(info);

	tacna_extcon_disable_micbias_pin(info, widget);
}

static void tacna_extcon_set_mode(struct tacna_extcon *info, int mode)
{
	struct tacna *tacna = info->tacna;
	unsigned int val;

	dev_dbg(info->dev,
		"set mic_mode[%d] src=0x%x gnd=0x%x bias=0x%x gpio=%d hp_gnd=%d\n",
		mode, info->micd_modes[mode].src, info->micd_modes[mode].gnd,
		info->micd_modes[mode].bias, info->micd_modes[mode].gpio,
		info->micd_modes[mode].hp_gnd);

	if (info->micd_pol_gpio)
		gpiod_set_value_cansleep(info->micd_pol_gpio,
					 info->micd_modes[mode].gpio);

	val = (info->micd_modes[mode].bias << TACNA_MICD1_BIAS_SRC_SHIFT) |
	      (info->micd_modes[mode].src << TACNA_MICD1_SENSE_SEL_SHIFT) |
	      (info->micd_modes[mode].gnd << TACNA_MICD1_GND_SEL_SHIFT);

	regmap_update_bits(tacna->regmap,
			   TACNA_MICDET1_CONTROL1,
			   TACNA_MICD1_BIAS_SRC_MASK |
			   TACNA_MICD1_SENSE_SEL_MASK |
			   TACNA_MICD1_GND_SEL_MASK,
			   val);
	/* HPD_GND_SEL should be the same as MICD1_GND_SEL */
	regmap_update_bits(tacna->regmap,
			   TACNA_HPDET1_CONTROL1,
			   TACNA_HPD_GND_SEL_MASK,
			   info->micd_modes[mode].gnd <<
			   TACNA_HPD_GND_SEL_SHIFT);
	regmap_update_bits(tacna->regmap,
			   TACNA_OUTPUT_PATH_CFG1,
			   TACNA_OUT1_GND_SEL_MASK,
			   info->micd_modes[mode].hp_gnd <<
			   TACNA_OUT1_GND_SEL_SHIFT);

	switch (tacna->type) {
	case CS47L96:
	case CS47L97:
		regmap_update_bits(tacna->regmap,
				   TACNA_OUTPUT_PATH_CFG3,
				   TACNA_OUTH_GND_SEL_MASK,
				   info->micd_modes[mode].hp_gnd <<
				   TACNA_OUTH_GND_SEL_SHIFT);
		break;
	default:
		break;
	}

	info->micd_mode = mode;
}

static void tacna_extcon_next_mode(struct tacna_extcon *info)
{
	int old_mode = info->micd_mode;
	int new_mode;
	bool change_bias = false;

	new_mode = (old_mode + 1) % info->num_micd_modes;

	dev_dbg(info->dev, "change micd mode %d->%d (bias %d->%d)\n",
		old_mode, new_mode,
		info->micd_modes[old_mode].bias,
		info->micd_modes[new_mode].bias);

	if (info->micd_modes[old_mode].bias !=
	    info->micd_modes[new_mode].bias) {
		change_bias = true;

		tacna_extcon_disable_micbias(info);
	}

	tacna_extcon_set_mode(info, new_mode);

	if (change_bias)
		tacna_extcon_enable_micbias(info);
}

static int tacna_micd_adc_read(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;
	unsigned int val = 0;
	int ret;

	/* Must disable MICD before we read the ADCVAL */
	ret = regmap_update_bits(tacna->regmap, TACNA_MICDET1_CONTROL1,
				 TACNA_MICD1_EN_MASK, 0);
	if (ret) {
		dev_err(info->dev, "Failed to disable MICD: %d\n", ret);
		return ret;
	}

	ret = regmap_read(tacna->regmap, TACNA_MICDET1_STATUS1, &val);
	if (ret) {
		dev_err(info->dev, "Failed to read MICDET_ADCVAL: %d\n", ret);
		return ret;
	}

	val = (val & TACNA_MICD1_ADCVAL_MASK) >> TACNA_MICD1_ADCVAL_SHIFT;

	dev_dbg(info->dev, "MICDET_ADCVAL: 0x%x\n", val);

	if (val < ARRAY_SIZE(tacna_micd_levels))
		return tacna_micd_levels[val];
	else
		return INT_MAX;
}

static int tacna_micd_read(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;
	unsigned int val = 0;
	int ret, i;

	for (i = 0; i < 10 && !(val & TACNA_MICD1_LVL_0_TO_8); i++) {
		ret = regmap_read(tacna->regmap,
				  TACNA_MICDET1_STATUS1, &val);
		if (ret) {
			dev_err(info->dev,
				"Failed to read MICDET: %d\n", ret);
			return ret;
		}

		dev_dbg(info->dev, "MICDET: 0x%x\n", val);

		if (!(val & TACNA_MICD1_VALID)) {
			dev_warn(info->dev,
				 "Microphone detection state invalid\n");
			return -EINVAL;
		}
	}

	if (i == 10 && !(val & TACNA_MICD1_LVL_0_TO_8)) {
		dev_warn(info->dev, "Failed to get valid MICDET value\n");
		return -EINVAL;
	}

	if (!(val & TACNA_MICD1_STS)) {
		val = INT_MAX;
	} else if (!(val & TACNA_MICD1_LVL_0_TO_7)) {
		val = tacna_micd_levels[ARRAY_SIZE(tacna_micd_levels) - 1];
	} else {
		int lvl;

		lvl = (val & TACNA_MICD1_LVL_MASK) >> TACNA_MICD1_LVL_SHIFT;
		lvl = ffs(lvl) - 1;

		if (lvl < info->num_micd_ranges) {
			val = info->micd_ranges[lvl].max;
		} else {
			i = ARRAY_SIZE(tacna_micd_levels) - 2;
			val = tacna_micd_levels[i];
		}
	}

	return val;
}

static void tacna_extcon_notify_micd(const struct tacna_extcon *info,
				     bool present,
				     unsigned int impedance)
{
	struct tacna_micdet_notify_data data;

	data.present = present;
	data.impedance_x100 = tacna_ohm_to_hohm(impedance);
	data.out_num = 1;

	tacna_call_notifiers(info->tacna, TACNA_NOTIFY_MICDET, &data);
}

static int tacna_hpdet_calc_calibration(
			int dacval,
			const struct tacna_hpdet_trims *trims,
			const struct tacna_hpdet_calibration_data *calib)
{
	int grad_x4 = trims->grad_x4;
	int off_x4 = trims->off_x4;
	s64 val = dacval;
	s64 n;

	val = (val * 1000000) + calib->dacval_adjust;
	val = div64_s64(val, calib->C2);

	n = div_s64(1000000000000LL, calib->C3 +
			((calib->C4_x_C3 * grad_x4) / 4));
	n = val - n;
	if (n <= 0)
		return TACNA_HPDET_MAX_HOHM;

	val = calib->C0 + ((calib->C1 * off_x4) / 4);
	val *= 1000000;

	val = div_s64(val, n);
	val -= calib->C5;

	/* Round up and divide to get hundredths of an ohm */
	val += 500000;
	val = div_s64(val, 10000);

	if (val < 0)
		return 0;
	else if (val > TACNA_HPDET_MAX_HOHM)
		return TACNA_HPDET_MAX_HOHM;

	return (int) val;
}

static int tacna_hpdet_calibrate(struct tacna_extcon *info,
				 unsigned int range, unsigned int *ohms_x100)
{
	struct tacna *tacna = info->tacna;
	unsigned int dacval;
	int ret;

	ret = regmap_read(tacna->regmap, TACNA_HPDET1_STATUS1, &dacval);
	if (ret) {
		dev_err(info->dev, "Failed to read HP DACVAL: %d\n", ret);
		return -EAGAIN;
	}

	dacval = (dacval & TACNA_HPD_DACVAL_MASK) >> TACNA_HPD_DACVAL_SHIFT;

	dev_dbg(info->dev, "hpdet_d calib range %d dac %d\n", range, dacval);

	*ohms_x100 = tacna_hpdet_calc_calibration(dacval,
						  &info->hpdet_trims[range],
						  &info->hpdet_ranges[range]);
	return 0;
}

static int tacna_hpdet_read(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;
	unsigned int val, range, sense_pin, ohms_x100;
	int ret;
	int hpdet_ext_res_x100;

	dev_dbg(info->dev, "HPDET read\n");

	ret = regmap_read(tacna->regmap, TACNA_HPDET1_STATUS1, &val);
	if (ret) {
		dev_err(info->dev, "Failed to read HPDET status: %d\n", ret);
		return ret;
	}

	if (!(val & TACNA_HPD_DONE_MASK)) {
		dev_warn(info->dev, "HPDET did not complete: %x\n", val);
		return -EAGAIN;
	}

	val &= TACNA_HPD_LVL_MASK;
	/* The value is in 0.5 ohm increments, get it in hundredths */
	ohms_x100 = val * 50;

	regmap_read(tacna->regmap, TACNA_HPDET1_CONTROL1, &sense_pin);
	sense_pin = (sense_pin & TACNA_HPD_SENSE_SEL_MASK) >>
		    TACNA_HPD_SENSE_SEL_SHIFT;

	switch (sense_pin) {
	case TACNA_HPD_SENSE_HPDET1:
	case TACNA_HPD_SENSE_HPDET2:
		break;
	default:
		dev_dbg(info->dev, "is_jdx_micdetx_pin\n");
		goto done;
	}

	regmap_read(tacna->regmap, TACNA_HPDET1_CONTROL1, &range);
	range = (range & TACNA_HPD_IMPEDANCE_RANGE_MASK) >>
		TACNA_HPD_IMPEDANCE_RANGE_SHIFT;

	/* Skip up a range, or report? */
	if (range < info->num_hpdet_ranges - 1 &&
	    ((val / 2) >= info->hpdet_ranges[range].max)) {
		range++;
		dev_dbg(info->dev, "Moving to HPDET range %d-%d\n",
			info->hpdet_ranges[range].min,
			info->hpdet_ranges[range].max);

		regmap_update_bits(tacna->regmap,
				   TACNA_HPDET1_CONTROL1,
				   TACNA_HPD_IMPEDANCE_RANGE_MASK,
				   range << TACNA_HPD_IMPEDANCE_RANGE_SHIFT);
		return -EAGAIN;
	}

	if (info->hpdet_trims) {
		/* Perform calibration */
		ret = tacna_hpdet_calibrate(info, range, &ohms_x100);
		if (ret)
			return ret;
	} else {
		/* Use uncalibrated reading */
		if (range && ((val / 2) < info->hpdet_ranges[range].min)) {
			dev_dbg(info->dev,
				"Reporting range boundary %d\n",
				info->hpdet_ranges[range].min);
			ohms_x100 =
			      tacna_ohm_to_hohm(info->hpdet_ranges[range].min);
		}
	}

	hpdet_ext_res_x100 = info->pdata->hpdet_ext_res_x100;
	if (hpdet_ext_res_x100) {
		if (hpdet_ext_res_x100 >= ohms_x100) {
			dev_dbg(info->dev,
				"External resistor (%d.%02d) >= measurement (%d.00)\n",
				hpdet_ext_res_x100 / 100,
				hpdet_ext_res_x100 % 100,
				val);
			ohms_x100 = 0;	/* treat as a short */
		} else {
			dev_dbg(info->dev,
				"Compensating for external %d.%02d ohm resistor\n",
				hpdet_ext_res_x100 / 100,
				hpdet_ext_res_x100 % 100);

			ohms_x100 -= hpdet_ext_res_x100;
		}
	}

done:
	dev_dbg(info->dev, "HP impedance %d.%02d ohms\n",
		ohms_x100 / 100, ohms_x100 % 100);

	return (int)ohms_x100;
}

static int cs47l96_tune_headphone(struct tacna_extcon *info, int reading)
{
	unsigned int val;

	if (info->pdata->output != 1)
		return 0;

	if (reading <= 10000) {
		val = 0x22;
	} else {
		val = 0x77;
	}

	return regmap_update_bits(info->tacna->regmap, TACNA_HP_XTALK_CONTROL1,
				  0x77, val);
}

static int tacna_tune_headphone(struct tacna_extcon *info, int reading)
{
	unsigned int short_imp = info->pdata->hpdet_short_circuit_imp;
	int ret;

	if (reading <= tacna_ohm_to_hohm(short_imp)) {
		/* Headphones are always off here */
		dev_warn(info->dev, "Possible HP short, disabling\n");
		return 0;
	}

	switch (info->tacna->type) {
	case CS47L96:
	case CS47L97:
		ret = cs47l96_tune_headphone(info, reading);
		break;
	default:
		return 0;
	}

	if (ret)
		dev_err(info->dev, "Failed to apply HP tuning %d\n", ret);

	return ret;
}

void tacna_set_headphone_imp(struct tacna_extcon *info, int ohms_x100)
{
	struct tacna *tacna = info->tacna;
	struct tacna_hpdet_notify_data data;

	tacna->hp_impedance_x100[0] = ohms_x100;
	tacna->hpdet_shorted[0] = tacna_hohm_to_ohm(ohms_x100) <=
				  info->pdata->hpdet_short_circuit_imp;

	data.impedance_x100 = ohms_x100;
	tacna_call_notifiers(tacna, TACNA_NOTIFY_HPDET, &data);

	tacna_tune_headphone(info, ohms_x100);
}
EXPORT_SYMBOL_GPL(tacna_set_headphone_imp);

int tacna_hpdet_start(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;
	int ret;
	unsigned int hpd_sense, hpd_clamp, val, hpd_gnd;

	dev_dbg(info->dev, "Starting HPDET\n");

	/* If we specified to assume a fixed impedance skip HPDET */
	if (info->pdata->fixed_hpdet_imp_x100) {
		tacna_set_headphone_imp(info,
					 info->pdata->fixed_hpdet_imp_x100);
		ret = -EEXIST;
		goto skip;
	}

	/* Make sure we keep the device enabled during the measurement */
	pm_runtime_get_sync(info->dev);

	if (info->state->mode == TACNA_ACCDET_MODE_HPL) {
		hpd_clamp = info->pdata->hpd_pins[0];
		hpd_sense = info->pdata->hpd_pins[1];
	} else {
		hpd_clamp = info->pdata->hpd_pins[2];
		hpd_sense = info->pdata->hpd_pins[3];
	}

	hpd_gnd = info->micd_modes[info->micd_mode].gnd;

	val = (hpd_sense << TACNA_HPD_SENSE_SEL_SHIFT) |
			(hpd_clamp << TACNA_HPD_OUT_SEL_SHIFT) |
			(hpd_sense << TACNA_HPD_FRC_SEL_SHIFT) |
			(hpd_gnd << TACNA_HPD_GND_SEL_SHIFT);

	ret = regmap_update_bits(tacna->regmap,
				 TACNA_MICDET1_CONTROL1,
				 TACNA_MICD1_GND_SEL_MASK,
				 hpd_gnd << TACNA_MICD1_GND_SEL_SHIFT);
	if (ret) {
		dev_err(tacna->dev, "Failed to set MICD_GND: %d\n",
			ret);
		goto err;
	}

	ret = regmap_update_bits(tacna->regmap,
				 TACNA_HPDET1_CONTROL1,
				 TACNA_HPD_GND_SEL_MASK |
				 TACNA_HPD_SENSE_SEL_MASK |
				 TACNA_HPD_FRC_SEL_MASK |
				 TACNA_HPD_OUT_SEL_MASK,
				 val);
	if (ret) {
		dev_err(info->dev,
			"Failed to set HPDET sense: %d\n", ret);
		goto err;
	}
	tacna_extcon_hp_clamp(info);

	ret = regmap_update_bits(tacna->regmap, TACNA_HPDET1_CONTROL1,
				 TACNA_HPD_POLL_MASK, TACNA_HPD_POLL);
	if (ret) {
		dev_err(info->dev, "Can't start HPDET measurement: %d\n", ret);
		goto err;
	}

	return 0;

err:
	tacna_extcon_hp_unclamp(info);

	pm_runtime_put_autosuspend(info->dev);

skip:
	return ret;
}
EXPORT_SYMBOL_GPL(tacna_hpdet_start);

void tacna_hpdet_restart(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;

	/* Reset back to starting range */
	regmap_update_bits(tacna->regmap, TACNA_HPDET1_CONTROL1,
			   TACNA_HPD_IMPEDANCE_RANGE_MASK | TACNA_HPD_POLL_MASK,
			   0);

	regmap_update_bits(tacna->regmap, TACNA_HPDET1_CONTROL1,
			   TACNA_HPD_POLL_MASK, TACNA_HPD_POLL);
}
EXPORT_SYMBOL_GPL(tacna_hpdet_restart);

static int tacna_hpdet_wait(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;
	unsigned int val;
	int i, ret;

	for (i = 0; i < TACNA_HPDONE_PROBE_COUNT; i++) {
		ret = regmap_read(tacna->regmap, TACNA_HPDET1_STATUS1,
				  &val);
		if (ret) {
			dev_err(tacna->dev, "Failed to read HPDET state: %d\n",
				ret);
			return ret;
		}

		if (val & TACNA_HPD_DONE_MASK)
			return 0;

		msleep(TACNA_HPDONE_PROBE_INTERVAL_MS);
	}

	dev_err(tacna->dev, "HPDET did not appear to complete\n");

	return -ETIMEDOUT;
}

void tacna_hpdet_stop(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;

	dev_dbg(info->dev, "Stopping HPDET\n");

	/*
	 * If the jack was removed we abort this state.
	 * Ensure that the detect hardware has returned to idle
	 */
	tacna_hpdet_wait(info);

	/* Reset back to starting range */
	regmap_update_bits(tacna->regmap, TACNA_HPDET1_CONTROL1,
			   TACNA_HPD_IMPEDANCE_RANGE_MASK | TACNA_HPD_POLL_MASK,
			   0);

	tacna_extcon_hp_unclamp(info);

	pm_runtime_mark_last_busy(info->dev);
	pm_runtime_put_autosuspend(info->dev);
}
EXPORT_SYMBOL_GPL(tacna_hpdet_stop);

int tacna_hpdet_reading(struct tacna_extcon *info, int val)
{
	dev_dbg(info->dev, "Reading HPDET %d\n", val);

	if (val < 0)
		return val;

	tacna_set_headphone_imp(info, val);

	/* Report high impedence cables as line outputs */
	if (val >= tacna_ohm_to_hohm(TACNA_EXTCON_LINEOUT_IMP_MIN))
		tacna_extcon_report(info, EXTCON_JACK_LINE_OUT, true);
	else
		tacna_extcon_report(info, EXTCON_JACK_HEADPHONE, true);

	if (info->have_mic)
		tacna_jds_set_state(info, &tacna_micd_button);
	else
		tacna_jds_set_state(info, NULL);

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_hpdet_reading);

int tacna_micd_start(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;
	int ret;
	unsigned int micd_mode;

	/* Microphone detection can't use idle mode */
	pm_runtime_get_sync(info->dev);

	dev_dbg(info->dev, "Disabling MICD_OVD\n");
	regmap_update_bits(tacna->regmap,
			   TACNA_MICD_CLAMP_CONTROL,
			   TACNA_MICD_CLAMP1_OVD_MASK, 0);

	ret = regulator_enable(info->micvdd);
	if (ret)
		dev_err(info->dev, "Failed to enable VOUT_MIC: %d\n", ret);

	if (info->state->mode == TACNA_ACCDET_MODE_ADC)
		micd_mode = TACNA_MICD1_ADC_MODE_MASK;
	else
		micd_mode = 0;

	regmap_update_bits(tacna->regmap,
			   TACNA_MICDET1_CONTROL1,
			   TACNA_MICD1_ADC_MODE_MASK, micd_mode);

	tacna_extcon_enable_micbias(info);

	regmap_update_bits(tacna->regmap, TACNA_MICDET1_CONTROL1,
			   TACNA_MICD1_EN_MASK, TACNA_MICD1_EN);

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_micd_start);

void tacna_micd_stop(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;

	regmap_update_bits(tacna->regmap, TACNA_MICDET1_CONTROL1,
			   TACNA_MICD1_EN_MASK, 0);

	tacna_extcon_disable_micbias(info);

	regulator_disable(info->micvdd);

	dev_dbg(info->dev, "Enabling MICD_OVD\n");
	regmap_update_bits(tacna->regmap, TACNA_MICD_CLAMP_CONTROL,
			   TACNA_MICD_CLAMP1_OVD_MASK, TACNA_MICD_CLAMP1_OVD);

	pm_runtime_mark_last_busy(info->dev);
	pm_runtime_put_autosuspend(info->dev);
}
EXPORT_SYMBOL_GPL(tacna_micd_stop);

static void tacna_micd_restart(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;

	regmap_update_bits(tacna->regmap, TACNA_MICDET1_CONTROL1,
			   TACNA_MICD1_EN_MASK, 0);
	regmap_update_bits(tacna->regmap, TACNA_MICDET1_CONTROL1,
			   TACNA_MICD1_EN_MASK, TACNA_MICD1_EN);
}

static int tacna_micd_button_debounce(struct tacna_extcon *info, int val)
{
	int debounce_lim = info->pdata->micd_manual_debounce;

	if (debounce_lim) {
		if (info->micd_debounce != val)
			info->micd_count = 0;

		info->micd_debounce = val;
		info->micd_count++;

		if (info->micd_count == debounce_lim) {
			info->micd_count = 0;
			if (val == info->micd_res_old)
				return 0;

			info->micd_res_old = val;
		} else {
			dev_dbg(info->dev, "Software debounce: %d,%x\n",
				info->micd_count, val);
			tacna_micd_restart(info);
			return -EAGAIN;
		}
	}

	return 0;
}

static int tacna_micd_button_process(struct tacna_extcon *info, int val)
{
	int i, key;

	if (val < TACNA_MICROPHONE_MIN_OHM) {
		dev_dbg(info->dev, "Mic button detected\n");

		for (i = 0; i < info->num_micd_ranges; i++)
			input_report_key(info->input,
					 info->micd_ranges[i].key, 0);

		for (i = 0; i < info->num_micd_ranges; i++) {
			if (val <= info->micd_ranges[i].max) {
				key = info->micd_ranges[i].key;
				dev_dbg(info->dev, "Key %d down\n", key);
				input_report_key(info->input, key, 1);
				input_sync(info->input);
				break;
			}
		}

		if (i == info->num_micd_ranges)
			dev_warn(info->dev,
				 "Button level %u out of range\n", val);
	} else {
		dev_dbg(info->dev, "Mic button released\n");

		for (i = 0; i < info->num_micd_ranges; i++)
			input_report_key(info->input,
					 info->micd_ranges[i].key, 0);
		input_sync(info->input);
	}

	return 0;
}

int tacna_micd_button_reading(struct tacna_extcon *info, int val)
{
	int ret;
	unsigned int ohms;

	if (val < 0)
		return val;

	ohms = tacna_hohm_to_ohm((unsigned int)val);

	ret = tacna_micd_button_debounce(info, ohms);
	if (ret < 0)
		return ret;

	return tacna_micd_button_process(info, ohms);
}
EXPORT_SYMBOL_GPL(tacna_micd_button_reading);

int tacna_micd_mic_start(struct tacna_extcon *info)
{
	int ret;

	info->detecting = true;

	ret = regulator_allow_bypass(info->micvdd, false);
	if (ret)
		dev_err(info->dev, "Failed to regulate VOUT_MIC: %d\n", ret);

	return tacna_micd_start(info);
}
EXPORT_SYMBOL_GPL(tacna_micd_mic_start);

void tacna_micd_mic_stop(struct tacna_extcon *info)
{
	int ret;

	tacna_micd_stop(info);

	ret = regulator_allow_bypass(info->micvdd, true);
	if (ret)
		dev_err(info->dev, "Failed to bypass VOUT_MIC: %d\n", ret);

	info->detecting = false;
}
EXPORT_SYMBOL_GPL(tacna_micd_mic_stop);

int tacna_micd_mic_reading(struct tacna_extcon *info, int val)
{
	unsigned int ohms;

	if (val < 0)
		return val;

	ohms = tacna_hohm_to_ohm((unsigned int)val);

	/* Due to jack detect this should never happen */
	if (ohms > TACNA_MICROPHONE_MAX_OHM) {
		dev_warn(info->dev, "Detected open circuit\n");
		info->have_mic = info->pdata->micd_open_circuit_declare;
		goto done;
	}

	/* If we got a high impedence we should have a headset, report it. */
	if (ohms >= TACNA_MICROPHONE_MIN_OHM) {
		dev_dbg(info->dev, "Detected headset\n");
		info->have_mic = true;
		goto done;
	}

	/*
	 * If we detected a lower impedence during initial startup
	 * then we probably have the wrong polarity, flip it.  Don't
	 * do this for the lowest impedences to speed up detection of
	 * plain headphones.  If both polarities report a low
	 * impedence then give up and report headphones.
	 */
	if (ohms > info->micd_ranges[0].max && info->num_micd_modes > 1) {
		if (info->jack_flips >= info->num_micd_modes * 10) {
			dev_dbg(info->dev, "Detected HP/line\n");
			goto done;
		} else {
			tacna_extcon_next_mode(info);

			info->jack_flips++;

			return -EAGAIN;
		}
	}

	/*
	 * If we're still detecting and we detect a short then we've
	 * got a headphone.
	 */
	dev_dbg(info->dev, "Headphone detected\n");

done:
	pm_runtime_mark_last_busy(info->dev);

	if (info->pdata->hpdet_channel)
		tacna_jds_set_state(info, &tacna_hpdet_right);
	else
		tacna_jds_set_state(info, &tacna_hpdet_left);

	tacna_extcon_report(info, EXTCON_JACK_MICROPHONE, info->have_mic);

	tacna_extcon_notify_micd(info, info->have_mic, ohms);

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_micd_mic_reading);

int tacna_micd_mic_timeout_ms(struct tacna_extcon *info)
{
	if (info->pdata->micd_timeout_ms)
		return info->pdata->micd_timeout_ms;
	else
		return TACNA_DEFAULT_MICD_TIMEOUT_MS;
}
EXPORT_SYMBOL_GPL(tacna_micd_mic_timeout_ms);

void tacna_micd_mic_timeout(struct tacna_extcon *info)
{
	int ret;

	dev_dbg(info->dev, "MICD timed out, reporting HP\n");

	if (info->pdata->hpdet_channel)
		ret = tacna_jds_set_state(info, &tacna_hpdet_right);
	else
		ret = tacna_jds_set_state(info, &tacna_hpdet_left);

	if (ret < 0)
		tacna_extcon_report(info, EXTCON_JACK_MICROPHONE, false);
}
EXPORT_SYMBOL_GPL(tacna_micd_mic_timeout);

static int tacna_jack_present(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;
	unsigned int val;
	int ret;

	ret = regmap_read(tacna->regmap, TACNA_IRQ1_STS_AOD, &val);
	if (ret) {
		dev_err(info->dev, "Failed to read jackdet status: %d\n", ret);
		return ret;
	}

	dev_dbg(info->dev, "IRQ1_STS_AOD=0x%x\n", val);

	return (val & TACNA_MICD_CLAMP_STS1) == 0;
}

static irqreturn_t tacna_hpdet_handler(int irq, void *data)
{
	struct tacna_extcon *info = data;
	int ret;

	dev_dbg(info->dev, "HPDET handler\n");

	tacna_jds_cancel_timeout(info);

	mutex_lock(&info->lock);

	switch (tacna_jds_get_mode(info)) {
	case TACNA_ACCDET_MODE_HPL:
	case TACNA_ACCDET_MODE_HPR:
	case TACNA_ACCDET_MODE_HPM:
		if (tacna_jack_present(info) <= 0)
			goto spurious;
		break;
	default:
		goto spurious;
	}

	ret = tacna_hpdet_read(info);
	if (ret != -EAGAIN)
		tacna_jds_reading(info, ret);

	tacna_jds_start_timeout(info);

	pm_runtime_mark_last_busy(info->dev);

	mutex_unlock(&info->lock);

	return IRQ_HANDLED;

spurious:
	dev_warn(info->dev, "Spurious HPDET IRQ\n");
	tacna_jds_start_timeout(info);
	mutex_unlock(&info->lock);
	return IRQ_NONE;
}

static void tacna_micd_handler(struct work_struct *work)
{
	struct tacna_extcon *info = container_of(work,
						 struct tacna_extcon,
						 micd_detect_work.work);
	enum tacna_accdet_mode mode;
	int ret;

	tacna_jds_cancel_timeout(info);

	mutex_lock(&info->lock);

	/*
	 * Must check that we are in a micd state before accessing
	 * any codec registers
	 */
	mode = tacna_jds_get_mode(info);
	switch (mode) {
	case TACNA_ACCDET_MODE_MIC:
	case TACNA_ACCDET_MODE_ADC:
		break;
	default:
		goto spurious;
	}

	if (tacna_jack_present(info) <= 0)
		goto spurious;

	switch (mode) {
	case TACNA_ACCDET_MODE_MIC:
		ret = tacna_micd_read(info);
		break;
	case TACNA_ACCDET_MODE_ADC:
		ret = tacna_micd_adc_read(info);
		break;
	default:	/* we can't get here but compiler still warns */
		ret = 0;
		break;
	}

	if (ret != -EAGAIN) {
		dev_dbg(info->dev, "Mic impedance %d ohms\n", ret);

		tacna_jds_reading(info, tacna_ohm_to_hohm((unsigned int)ret));
	}

	tacna_jds_start_timeout(info);

	pm_runtime_mark_last_busy(info->dev);

	mutex_unlock(&info->lock);

	return;

spurious:
	dev_warn(info->dev, "Spurious MICDET IRQ\n");
	tacna_jds_start_timeout(info);
	mutex_unlock(&info->lock);
}

static irqreturn_t tacna_micdet(int irq, void *data)
{
	struct tacna_extcon *info = data;
	int debounce = info->pdata->micd_detect_debounce_ms;

	dev_dbg(info->dev, "micdet IRQ");

	cancel_delayed_work_sync(&info->micd_detect_work);

	mutex_lock(&info->lock);

	if (!info->detecting)
		debounce = 0;

	mutex_unlock(&info->lock);

	/*
	 * Defer to the workqueue to ensure serialization
	 * and prevent race conditions if an IRQ occurs while
	 * running the delayed work
	 */
	schedule_delayed_work(&info->micd_detect_work,
			      msecs_to_jiffies(debounce));

	return IRQ_HANDLED;
}

const struct tacna_jd_state tacna_hpdet_left = {
	.mode = TACNA_ACCDET_MODE_HPL,
	.start = &tacna_hpdet_start,
	.reading = &tacna_hpdet_reading,
	.stop = &tacna_hpdet_stop,
};
EXPORT_SYMBOL_GPL(tacna_hpdet_left);

const struct tacna_jd_state tacna_hpdet_right = {
	.mode = TACNA_ACCDET_MODE_HPR,
	.start = &tacna_hpdet_start,
	.reading = &tacna_hpdet_reading,
	.stop = &tacna_hpdet_stop,
};
EXPORT_SYMBOL_GPL(tacna_hpdet_right);

const struct tacna_jd_state tacna_micd_button = {
	.mode = TACNA_ACCDET_MODE_MIC,
	.start = &tacna_micd_start,
	.reading = &tacna_micd_button_reading,
	.stop = &tacna_micd_stop,
};
EXPORT_SYMBOL_GPL(tacna_micd_button);

const struct tacna_jd_state tacna_micd_adc_mic = {
	.mode = TACNA_ACCDET_MODE_ADC,
	.start = &tacna_micd_mic_start,
	.restart = &tacna_micd_restart,
	.reading = &tacna_micd_mic_reading,
	.stop = &tacna_micd_mic_stop,

	.timeout_ms = &tacna_micd_mic_timeout_ms,
	.timeout = &tacna_micd_mic_timeout,
};
EXPORT_SYMBOL_GPL(tacna_micd_adc_mic);

const struct tacna_jd_state tacna_micd_microphone = {
	.mode = TACNA_ACCDET_MODE_MIC,
	.start = &tacna_micd_mic_start,
	.reading = &tacna_micd_mic_reading,
	.stop = &tacna_micd_mic_stop,

	.timeout_ms = &tacna_micd_mic_timeout_ms,
	.timeout = &tacna_micd_mic_timeout,
};
EXPORT_SYMBOL_GPL(tacna_micd_microphone);

static irqreturn_t tacna_jackdet(int irq, void *data)
{
	struct tacna_extcon *info = data;
	struct tacna *tacna = info->tacna;
	bool cancelled_state;
	int i, present;

	dev_dbg(info->dev, "jackdet IRQ");

	cancelled_state = tacna_jds_cancel_timeout(info);

	pm_runtime_get_sync(info->dev);

	mutex_lock(&info->lock);

	present = tacna_jack_present(info);
	if (present < 0) {
		mutex_unlock(&info->lock);
		pm_runtime_put_autosuspend(info->dev);
		return IRQ_NONE;
	}

	if (present == info->last_jackdet) {
		dev_dbg(info->dev, "Suppressing duplicate JACKDET\n");
		if (cancelled_state)
			tacna_jds_start_timeout(info);

		goto out;
	}
	info->last_jackdet = present;

	if (present) {
		dev_dbg(info->dev, "Detected jack\n");

		tacna_extcon_report(info, EXTCON_MECHANICAL, true);

		info->have_mic = false;
		info->jack_flips = 0;

		if (info->pdata->custom_jd)
			tacna_jds_set_state(info, info->pdata->custom_jd);
		else if (info->pdata->micd_software_compare)
			tacna_jds_set_state(info, &tacna_micd_adc_mic);
		else
			tacna_jds_set_state(info, &tacna_micd_microphone);

		tacna_jds_start_timeout(info);

		regmap_update_bits(tacna->regmap, TACNA_ACCDET_DEBOUNCE,
				   TACNA_MICD_CLAMP_DB_MASK, 0);
	} else {
		dev_dbg(info->dev, "Detected jack removal\n");

		info->have_mic = false;
		info->micd_res_old = 0;
		info->micd_debounce = 0;
		info->micd_count = 0;
		tacna_jds_set_state(info, NULL);

		for (i = 0; i < info->num_micd_ranges; i++)
			input_report_key(info->input,
					 info->micd_ranges[i].key, 0);
		input_sync(info->input);

		tacna_extcon_report_removal(info);

		regmap_update_bits(tacna->regmap,
				   TACNA_ACCDET_DEBOUNCE,
				   TACNA_MICD_CLAMP_DB_MASK,
				   TACNA_MICD_CLAMP_DB);

		tacna_set_headphone_imp(info, TACNA_HP_Z_OPEN);

		tacna_extcon_notify_micd(info, false, 0);
	}

out:
	mutex_unlock(&info->lock);

	pm_runtime_mark_last_busy(info->dev);
	pm_runtime_put_autosuspend(info->dev);

	return IRQ_HANDLED;
}

/* Map a level onto a slot in the register bank */
static void tacna_micd_set_level(struct tacna_extcon *info, int index,
				 unsigned int level)
{
	struct tacna *tacna = info->tacna;
	unsigned int reg, shift, mask;

	switch (index) {
	case 0:
		reg = TACNA_MICDET1_CONTROL4;
		mask = TACNA_MICD1_SWVAL7_MASK;
		shift = TACNA_MICD1_SWVAL7_SHIFT;
		break;
	case 1:
		reg = TACNA_MICDET1_CONTROL4;
		mask = TACNA_MICD1_SWVAL6_MASK;
		shift = TACNA_MICD1_SWVAL6_SHIFT;
		break;
	case 2:
		reg = TACNA_MICDET1_CONTROL4;
		mask = TACNA_MICD1_SWVAL5_MASK;
		shift = TACNA_MICD1_SWVAL5_SHIFT;
		break;
	case 3:
		reg = TACNA_MICDET1_CONTROL4;
		mask = TACNA_MICD1_SWVAL4_MASK;
		shift = TACNA_MICD1_SWVAL4_SHIFT;
		break;
	case 4:
		reg = TACNA_MICDET1_CONTROL3;
		mask = TACNA_MICD1_SWVAL3_MASK;
		shift = TACNA_MICD1_SWVAL3_SHIFT;
		break;
	case 5:
		reg = TACNA_MICDET1_CONTROL3;
		mask = TACNA_MICD1_SWVAL2_MASK;
		shift = TACNA_MICD1_SWVAL2_SHIFT;
		break;
	case 6:
		reg = TACNA_MICDET1_CONTROL3;
		mask = TACNA_MICD1_SWVAL1_MASK;
		shift = TACNA_MICD1_SWVAL1_SHIFT;
		break;
	case 7:
		reg = TACNA_MICDET1_CONTROL3;
		mask = TACNA_MICD1_SWVAL0_MASK;
		shift = TACNA_MICD1_SWVAL0_SHIFT;
		break;
	default:
		dev_err(info->dev, "Invalid micd level index: %d\n", index);
		return;
	}

	/* Program the level itself */
	regmap_update_bits(tacna->regmap, reg, mask, level << shift);
}

static void tacna_extcon_get_micd_configs(struct tacna_extcon *info,
					  struct fwnode_handle *node)
{
	struct tacna_micd_config *micd_configs;
	u32 *values;
	int nvalues, nconfigs, i, j;
	int ret;

	nvalues = fwnode_property_read_u32_array(node,
						 "cirrus,micd-configs",
						 NULL, 0);
	if (nvalues == -EINVAL) {
		return;	/* not found */
	} else if ((nvalues < 0) || (nvalues % 5)) {
		dev_warn(info->dev, "cirrus,micd-configs is malformed\n");
		return;
	}

	values = kmalloc_array(nvalues, sizeof(u32), GFP_KERNEL);
	if (!values)
		return;

	ret = fwnode_property_read_u32_array(node,
					     "cirrus,micd-configs",
					     values, nvalues);
	if (ret < 0)
		goto err;

	nconfigs = nvalues / 5;
	micd_configs = devm_kcalloc(info->dev,
				    nconfigs,
				    sizeof(struct tacna_micd_config),
				    GFP_KERNEL);
	if (!micd_configs)
		goto err;

	for (i = 0, j = 0; i < nconfigs; ++i) {
		micd_configs[i].src = values[j++];
		micd_configs[i].gnd = values[j++];
		micd_configs[i].bias = values[j++];
		micd_configs[i].gpio = values[j++];
		micd_configs[i].hp_gnd = values[j++];
	}

	info->micd_modes = micd_configs;
	info->num_micd_modes = nconfigs;

err:
	kfree(values);
}

static void tacna_extcon_get_hpd_pins(struct tacna_extcon *info,
				      struct fwnode_handle *node,
				      struct tacna_accdet_pdata *pdata)
{
	int i, ret;

	BUILD_BUG_ON(ARRAY_SIZE(pdata->hpd_pins) !=
		     ARRAY_SIZE(tacna_default_hpd_pins));

	memcpy(pdata->hpd_pins, tacna_default_hpd_pins,
	       sizeof(pdata->hpd_pins));

	ret = fwnode_property_read_u32_array(node,
					     "cirrus,hpd-pins",
					     pdata->hpd_pins,
					     ARRAY_SIZE(pdata->hpd_pins));
	if (ret) {
		if (ret != -EINVAL)
			dev_warn(info->dev,
				 "Malformed cirrus,hpd-pins: %d\n", ret);
		return;
	}

	/* supply defaults where requested */
	for (i = 0; i < ARRAY_SIZE(pdata->hpd_pins); ++i)
		if (pdata->hpd_pins[i] > 0xFFFF)
			pdata->hpd_pins[i] = tacna_default_hpd_pins[i];
}

static void tacna_extcon_process_accdet_node(struct tacna_extcon *info,
					     struct fwnode_handle *node)
{
	struct tacna *tacna = info->tacna;
	struct tacna_accdet_pdata *pdata;
	u32 out_num;
	int i, ret;
	enum gpiod_flags gpio_status;

	ret = fwnode_property_read_u32(node, "reg", &out_num);
	if (ret < 0) {
		dev_warn(info->dev,
			 "failed to read reg property (%d)\n",
			 ret);
		return;
	}

	if (out_num == 0) {
		dev_warn(info->dev, "accdet node illegal reg %u\n", out_num);
		return;
	}

	dev_dbg(info->dev, "processing accdet reg=%u\n", out_num);

	for (i = 0; i < ARRAY_SIZE(tacna->pdata.accdet); i++)
		if (!tacna->pdata.accdet[i].enabled)
			break;

	if (i == ARRAY_SIZE(tacna->pdata.accdet)) {
		dev_warn(tacna->dev, "Too many accdet nodes: %d\n", i + 1);
		return;
	}

	pdata = &tacna->pdata.accdet[i];
	pdata->enabled = true;	/* implied by presence of properties node */
	pdata->output = out_num;

	fwnode_property_read_u32(node, "cirrus,micd-detect-debounce-ms",
				 &pdata->micd_detect_debounce_ms);

	fwnode_property_read_u32(node, "cirrus,micd-manual-debounce",
				 &pdata->micd_manual_debounce);

	fwnode_property_read_u32(node, "cirrus,micd-bias-start-time",
				 &pdata->micd_bias_start_time);

	fwnode_property_read_u32(node, "cirrus,micd-rate",
				 &pdata->micd_rate);

	fwnode_property_read_u32(node, "cirrus,micd-dbtime",
				 &pdata->micd_dbtime);

	fwnode_property_read_u32(node, "cirrus,micd-timeout-ms",
				 &pdata->micd_timeout_ms);

	pdata->micd_software_compare =
		fwnode_property_present(node,
					"cirrus,micd-software-compare");

	pdata->micd_open_circuit_declare =
		fwnode_property_present(node,
					"cirrus,micd-open-circuit-declare");

	fwnode_property_read_u32(node, "cirrus,fixed-hpdet-imp",
				 &pdata->fixed_hpdet_imp_x100);

	fwnode_property_read_u32(node, "cirrus,hpdet-short-circuit-imp",
				 &pdata->hpdet_short_circuit_imp);

	fwnode_property_read_u32(node, "cirrus,hpdet-channel",
				 &pdata->hpdet_channel);

	fwnode_property_read_u32(node, "cirrus,micd-clamp-mode",
				 &pdata->micd_clamp_mode);

	fwnode_property_read_u32(node, "cirrus,hpdet-ext-res",
				 &pdata->hpdet_ext_res_x100);

	fwnode_property_read_u32(node, "cirrus,accdet-dbtime",
				 &pdata->accdet_dbtime);

	tacna_extcon_get_hpd_pins(info, node, pdata);
	tacna_extcon_get_micd_configs(info, node);

	if (info->micd_modes[0].gpio)
		gpio_status = GPIOD_OUT_HIGH;
	else
		gpio_status = GPIOD_OUT_LOW;

	info->micd_pol_gpio = devm_fwnode_get_gpiod_from_child(tacna->dev,
							"cirrus,micd-pol",
							node,
							gpio_status,
							"cirrus,micd-pol");
	if (IS_ERR(info->micd_pol_gpio))
		info->micd_pol_gpio = NULL;
}

static void tacna_extcon_get_device_pdata(struct tacna_extcon *info)
{
	struct device_node *parent, *child;
	struct tacna *tacna = info->tacna;

	/*
	 * a GPSW is not necessarily exclusive to a single accessory detect
	 * channel so is not in the subnodes
	 */
	device_property_read_u32_array(tacna->dev, "cirrus,gpsw",
				       info->tacna->pdata.gpsw,
				       ARRAY_SIZE(info->tacna->pdata.gpsw));

	parent = of_get_child_by_name(tacna->dev->of_node, "cirrus,accdet");
	if (!parent) {
		dev_dbg(tacna->dev, "No DT nodes\n");
		return;
	}

	for_each_child_of_node(parent, child)
		tacna_extcon_process_accdet_node(info, &child->fwnode);
}

#ifdef DEBUG
#define TACNA_EXTCON_PDATA_DUMP(x, f) \
	dev_dbg(info->dev, "\t" #x ": " f "\n", pdata->x)

static void tacna_extcon_dump_config(struct tacna_extcon *info)
{
	const struct tacna_accdet_pdata *pdata;
	int i, j;

	dev_dbg(info->dev, "extcon pdata gpsw=[0x%x 0x%x]\n",
		info->tacna->pdata.gpsw[0], info->tacna->pdata.gpsw[1]);

	for (i = 0; i < ARRAY_SIZE(info->tacna->pdata.accdet); ++i) {
		pdata = &info->tacna->pdata.accdet[i];

		dev_dbg(info->dev, "extcon pdata OUT%u\n", i + 1);
		TACNA_EXTCON_PDATA_DUMP(enabled, "%u");
		TACNA_EXTCON_PDATA_DUMP(fixed_hpdet_imp_x100, "%d");
		TACNA_EXTCON_PDATA_DUMP(hpdet_ext_res_x100, "%d");
		TACNA_EXTCON_PDATA_DUMP(hpdet_short_circuit_imp, "%d");
		TACNA_EXTCON_PDATA_DUMP(hpdet_channel, "%d");
		TACNA_EXTCON_PDATA_DUMP(micd_detect_debounce_ms, "%d");
		TACNA_EXTCON_PDATA_DUMP(hpdet_short_circuit_imp, "%d");
		TACNA_EXTCON_PDATA_DUMP(hpdet_channel, "%d");
		TACNA_EXTCON_PDATA_DUMP(micd_detect_debounce_ms, "%d");
		TACNA_EXTCON_PDATA_DUMP(micd_manual_debounce, "%d");
		TACNA_EXTCON_PDATA_DUMP(micd_bias_start_time, "%d");
		TACNA_EXTCON_PDATA_DUMP(micd_rate, "%d");
		TACNA_EXTCON_PDATA_DUMP(micd_dbtime, "%d");
		TACNA_EXTCON_PDATA_DUMP(micd_timeout_ms, "%d");
		TACNA_EXTCON_PDATA_DUMP(micd_clamp_mode, "%u");
		TACNA_EXTCON_PDATA_DUMP(micd_open_circuit_declare, "%u");
		TACNA_EXTCON_PDATA_DUMP(micd_software_compare, "%u");

		if (info->micd_pol_gpio)
			dev_dbg(info->dev, "micd_pol_gpio: %d\n",
				desc_to_gpio(info->micd_pol_gpio));
		else
			dev_dbg(info->dev, "micd_pol_gpio: none\n");

		dev_dbg(info->dev, "\tmicd_ranges {\n");
		for (j = 0; j < info->num_micd_ranges; ++j)
			dev_dbg(info->dev, "\t\tmax: %d key: %d\n",
				info->micd_ranges[j].max,
				info->micd_ranges[j].key);
		dev_dbg(info->dev, "\t}\n");

		dev_dbg(info->dev, "\tmicd_configs {\n");
		for (j = 0; j < info->num_micd_modes; ++j)
			dev_dbg(info->dev,
				"\t\tsrc: 0x%x gnd: 0x%x bias: %u gpio: %u hp_gnd: %d\n",
				info->micd_modes[j].src,
				info->micd_modes[j].gnd,
				info->micd_modes[j].bias,
				info->micd_modes[j].gpio,
				info->micd_modes[j].hp_gnd);
		dev_dbg(info->dev, "\t}\n");

		dev_dbg(info->dev, "\thpd_pins: %u %u %u %u\n",
			pdata->hpd_pins[0], pdata->hpd_pins[1],
			pdata->hpd_pins[2], pdata->hpd_pins[3]);
	}
}
#else
static inline void tacna_extcon_dump_config(struct tacna_extcon *info)
{
}
#endif

static void tacna_extcon_set_micd_clamp_mode(struct tacna_extcon *info)
{
	unsigned int clamp_ctrl_val;

	if (info->pdata->micd_clamp_mode) {
		clamp_ctrl_val = info->pdata->micd_clamp_mode;

		regmap_update_bits(info->tacna->regmap,
				   TACNA_MICD_CLAMP_CONTROL,
				   TACNA_MICD_CLAMP1_MODE_MASK,
				   clamp_ctrl_val <<
				   TACNA_MICD_CLAMP1_MODE_SHIFT);
	}

	regmap_update_bits(info->tacna->regmap,
			   TACNA_ACCDET_DEBOUNCE,
			   TACNA_MICD_CLAMP_DB,
			   TACNA_MICD_CLAMP_DB);
}

static int tacna_extcon_add_micd_levels(struct tacna_extcon *info)
{
	struct tacna *tacna = info->tacna;
	int i, j;
	unsigned int ranges = 0;

	BUILD_BUG_ON(ARRAY_SIZE(tacna_micd_levels) <
		     TACNA_NUM_MICD_BUTTON_LEVELS);

	/*
	 * Disable all buttons by default, but leave short circuit and mic
	 * detect enabled
	 */
	regmap_update_bits(tacna->regmap,
			   TACNA_MICDET1_CONTROL2,
			   TACNA_MICD1_LVL_SEL_MASK,
			   TACNA_LVL_SEL_SHORT_OR_MIC);

	/* Set up all the buttons the user specified */
	for (i = 0; i < info->num_micd_ranges; i++) {
		for (j = 0; j < TACNA_NUM_MICD_BUTTON_LEVELS; j++)
			if (tacna_micd_levels[j] >= info->micd_ranges[i].max)
				break;

		if (j == TACNA_NUM_MICD_BUTTON_LEVELS) {
			dev_err(info->dev, "Unsupported MICD level %d\n",
				info->micd_ranges[i].max);
			return -EINVAL;
		}

		dev_dbg(info->dev, "%d ohms for MICD threshold %d\n",
			tacna_micd_levels[j], i);

		tacna_micd_set_level(info, i, j);
		if (info->micd_ranges[i].key > 0)
			input_set_capability(info->input, EV_KEY,
					     info->micd_ranges[i].key);

		ranges |= 1 << i;
	}

	regmap_update_bits(tacna->regmap,
			   TACNA_MICDET1_CONTROL2,
			   ranges, ranges);

	/* Set all the remaining keys to a maximum */
	for (; i < TACNA_MAX_MICD_RANGE; i++)
		tacna_micd_set_level(info, i, 0x3f);

	return 0;
}

static int tacna_extcon_init_micd_ranges(struct tacna_extcon *info)
{
	const struct tacna_accdet_pdata *pdata = info->pdata;
	struct tacna_micd_range *ranges;
	int i;

	if (pdata->num_micd_ranges == 0) {
		info->micd_ranges = tacna_micd_default_ranges;
		info->num_micd_ranges =
			ARRAY_SIZE(tacna_micd_default_ranges);
		return 0;
	}

	if (pdata->num_micd_ranges > TACNA_MAX_MICD_RANGE) {
		dev_err(info->dev, "Too many MICD ranges: %d\n",
			pdata->num_micd_ranges);
		return -EINVAL;
	}

	ranges = devm_kmalloc_array(info->dev,
				    pdata->num_micd_ranges,
				    sizeof(struct tacna_micd_range),
				    GFP_KERNEL);
	if (!ranges) {
		dev_err(info->dev, "Failed to kalloc micd ranges\n");
		return -ENOMEM;
	}

	memcpy(ranges, pdata->micd_ranges,
	       sizeof(struct tacna_micd_range) * pdata->num_micd_ranges);
	info->micd_ranges = ranges;
	info->num_micd_ranges = pdata->num_micd_ranges;

	for (i = 0; i < info->num_micd_ranges - 1; i++) {
		if (info->micd_ranges[i].max > info->micd_ranges[i + 1].max) {
			dev_err(info->dev, "MICD ranges must be sorted\n");
			goto err_free;
		}
	}

	return 0;

err_free:
	devm_kfree(info->dev, ranges);

	return -EINVAL;
}

static void tacna_extcon_xlate_pdata(struct tacna_accdet_pdata *pdata)
{
	int i;

	BUILD_BUG_ON(ARRAY_SIZE(pdata->hpd_pins) !=
		     ARRAY_SIZE(tacna_default_hpd_pins));

	/* translate from pdata format where 0=default and >0xFFFF means 0 */
	for (i = 0; i < ARRAY_SIZE(pdata->hpd_pins); ++i) {
		if (pdata->hpd_pins[i] == 0)
			pdata->hpd_pins[i] = tacna_default_hpd_pins[i];
		else if (pdata->hpd_pins[i] > 0xFFFF)
			pdata->hpd_pins[i] = 0;
	}
}

static void tacna_configure_jds(struct tacna_extcon *info)
{
	unsigned int debounce_val, analogue_val;

	switch (info->pdata->micd_clamp_mode) {
	case TACNA_MICD_CLAMP_MODE_DISABLED:
	case TACNA_MICD_CLAMP_MODE_ACTIVE:
		debounce_val = 0;
		analogue_val = 0;
		break;
	case TACNA_MICD_CLAMP_MODE_JD1L:
	case TACNA_MICD_CLAMP_MODE_JD1H:
		debounce_val = TACNA_JD1_DB;
		analogue_val = TACNA_JD1_EN;
		break;
	case TACNA_MICD_CLAMP_MODE_JD2L:
	case TACNA_MICD_CLAMP_MODE_JD2H:
		debounce_val = TACNA_JD2_DB;
		analogue_val = TACNA_JD2_EN;
		break;
	case TACNA_MICD_CLAMP_MODE_JD1L_OR_JD2L:
	case TACNA_MICD_CLAMP_MODE_JD1L_OR_JD2H:
	case TACNA_MICD_CLAMP_MODE_JD1H_OR_JD2L:
	case TACNA_MICD_CLAMP_MODE_JD1H_OR_JD2H:
	case TACNA_MICD_CLAMP_MODE_JD1L_AND_JD2L:
	case TACNA_MICD_CLAMP_MODE_JD1L_AND_JD2H:
	case TACNA_MICD_CLAMP_MODE_JD1H_AND_JD2L:
	case TACNA_MICD_CLAMP_MODE_JD1H_AND_JD2H:
		debounce_val = TACNA_JD1_DB | TACNA_JD2_DB;
		analogue_val = TACNA_JD1_EN | TACNA_JD2_EN;
		break;
	default:
		debounce_val = 0;
		analogue_val = 0;
		dev_warn(info->dev,
			 "%u is not a valid MICD_CLAMP1_MODE.\n",
			 info->pdata->micd_clamp_mode);
		break;
	}

	regmap_update_bits(info->tacna->regmap, TACNA_ACCDET_DEBOUNCE,
			   TACNA_JD1_DB | TACNA_JD2_DB, debounce_val);
	regmap_update_bits(info->tacna->regmap, TACNA_JACK_DETECT,
			   TACNA_JD1_EN | TACNA_JD2_EN, analogue_val);
}

static int tacna_extcon_probe(struct platform_device *pdev)
{
	struct tacna *tacna = dev_get_drvdata(pdev->dev.parent);
	struct tacna_accdet_pdata *pdata = &tacna->pdata.accdet[0];
	struct tacna_extcon *info;
	unsigned int val;
	int ret, mode, hpdet_short_reading;

	/* quick exit if Tacna irqchip driver hasn't completed probe */
	if (!tacna->irq_dev) {
		dev_dbg(&pdev->dev, "irqchip driver not ready\n");
		return -EPROBE_DEFER;
	}

	if (!tacna->dapm || !tacna->dapm->card) {
		dev_dbg(&pdev->dev, "DAPM not ready\n");
		return -EPROBE_DEFER;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->pdata = pdata;
	info->tacna = tacna;
	info->dev = &pdev->dev;
	mutex_init(&info->lock);
	init_completion(&info->manual_mic_completion);
	INIT_DELAYED_WORK(&info->micd_detect_work, tacna_micd_handler);
	INIT_DELAYED_WORK(&info->state_timeout_work, tacna_jds_timeout_work);
	platform_set_drvdata(pdev, info);

	info->hpdet_ranges = tacna_hpdet_ranges;
	info->num_hpdet_ranges = ARRAY_SIZE(tacna_hpdet_ranges);
	info->micd_modes = tacna_micd_default_modes;
	info->num_micd_modes = ARRAY_SIZE(tacna_micd_default_modes);

	if (dev_get_platdata(tacna->dev)) {
		tacna_extcon_xlate_pdata(pdata);

		if (pdata->num_micd_configs) {
			info->micd_modes = pdata->micd_configs;
			info->num_micd_modes = pdata->num_micd_configs;
		}

		if (pdata->micd_pol_gpio > 0) {
			if (info->micd_modes[0].gpio)
				mode = GPIOF_OUT_INIT_HIGH;
			else
				mode = GPIOF_OUT_INIT_LOW;

			ret = devm_gpio_request_one(&pdev->dev,
						    pdata->micd_pol_gpio,
						    mode,
						    "MICD polarity");
			if (ret) {
				dev_err(info->dev,
					"Failed to request GPIO%d: %d\n",
					pdata->micd_pol_gpio, ret);
				return ret;
			}

			info->micd_pol_gpio =
				gpio_to_desc(pdata->micd_pol_gpio);
		}
	} else {
		tacna_extcon_get_device_pdata(info);
	}

	if (!pdata->enabled || pdata->output == 0) {
		return -ENODEV; /* no accdet output configured */
	} else if (pdata->output != 1) {
		dev_err(info->dev, "Only OUT1 is supported, OUT%d requested\n",
			pdata->output);
		return -ENODEV;
	}

	switch (tacna->type) {
	case CS47L96:
	case CS47L97:
		val = (pdata->output - 1) << TACNA_JD1_OUT_HP_SHIFT;
		regmap_update_bits(tacna->regmap,
				   TACNA_JACKDET1_COMP_CTRL,
				   TACNA_JD1_OUT_HP_MASK,
				   val);
		break;
	default:
		break;
	}

	/* Actual measured short is increased by external resistance */
	hpdet_short_reading = pdata->hpdet_short_circuit_imp +
			tacna_hohm_to_ohm(pdata->hpdet_ext_res_x100);

	if (hpdet_short_reading < TACNA_HP_SHORT_IMPEDANCE_MIN) {
		/*
		 * increase comparison threshold to minimum we can measure
		 * taking into account that threshold does not include external
		 * resistance
		 */
		pdata->hpdet_short_circuit_imp = TACNA_HP_SHORT_IMPEDANCE_MIN -
			       tacna_hohm_to_ohm(pdata->hpdet_ext_res_x100);
		dev_warn(info->dev,
			 "Increasing HP short circuit impedance to %d\n",
			 pdata->hpdet_short_circuit_imp);
	}

	info->micvdd = devm_regulator_get(&pdev->dev, "VOUT_MIC");
	if (IS_ERR(info->micvdd)) {
		ret = PTR_ERR(info->micvdd);
		dev_err(info->dev, "Failed to get VOUT_MIC: %d\n", ret);
		return ret;
	}

	info->last_jackdet = ~TACNA_MICD_CLAMP_STS1;

	info->edev = devm_extcon_dev_allocate(&pdev->dev, tacna_cable);
	if (IS_ERR(info->edev)) {
		dev_err(&pdev->dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(&pdev->dev, info->edev);
	if (ret < 0) {
		dev_err(info->dev, "extcon_dev_register() failed: %d\n", ret);
		return ret;
	}

	info->input = devm_input_allocate_device(&pdev->dev);
	if (!info->input) {
		dev_err(info->dev, "Can't allocate input dev\n");
		ret = -ENOMEM;
		goto err_register;
	}

	info->input->name = "Headset";
	info->input->phys = "tacna/extcon";
	info->input->dev.parent = &pdev->dev;

	if (tacna->pdata.gpsw[0] > 0)
		regmap_update_bits(tacna->regmap,
				   TACNA_GP_SWITCH,
				   TACNA_GPSW1_MODE_MASK,
				   tacna->pdata.gpsw[0] <<
				   TACNA_GPSW1_MODE_SHIFT);
	if (tacna->pdata.gpsw[1] > 0)
		regmap_update_bits(tacna->regmap,
				   TACNA_GP_SWITCH,
				   TACNA_GPSW2_MODE_MASK,
				   tacna->pdata.gpsw[1] <<
				   TACNA_GPSW2_MODE_SHIFT);

	if (info->pdata->micd_bias_start_time)
		regmap_update_bits(tacna->regmap,
				   TACNA_MICDET1_CONTROL1,
				   TACNA_MICD1_DELAY_MASK,
				   info->pdata->micd_bias_start_time
				   << TACNA_MICD1_DELAY_SHIFT);

	if (info->pdata->micd_rate)
		regmap_update_bits(tacna->regmap,
				   TACNA_MICDET1_CONTROL1,
				   TACNA_MICD1_RATE_MASK,
				   info->pdata->micd_rate
				   << TACNA_MICD1_RATE_SHIFT);

	if (info->pdata->micd_dbtime)
		regmap_update_bits(tacna->regmap,
				   TACNA_MICDET1_CONTROL1,
				   TACNA_MICD1_DBTIME_MASK,
				   info->pdata->micd_dbtime
				   << TACNA_MICD1_DBTIME_SHIFT);


	if (info->pdata->accdet_dbtime)
		regmap_update_bits(tacna->regmap,
				   TACNA_ACCDET_DEBOUNCE,
				   TACNA_ACCDET_DBTIME_MASK,
				   info->pdata->accdet_dbtime
				   << TACNA_ACCDET_DBTIME_SHIFT);

	ret = tacna_extcon_init_micd_ranges(info);
	if (ret)
		goto err_input;

	ret = tacna_extcon_add_micd_levels(info);
	if (ret)
		goto err_input;

	tacna_extcon_set_micd_clamp_mode(info);

	if ((info->num_micd_modes > 2) && !info->micd_pol_gpio)
		dev_warn(info->dev, "Have >1 mic_configs but no pol_gpio\n");

	tacna_extcon_set_mode(info, 0);

	/*
	 * Invalidate the tuning level so that the first detection
	 * will always apply a tuning
	 */
	info->hp_tuning_level = TACNA_HP_TUNING_INVALID;

	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);

	pm_runtime_get_sync(&pdev->dev);

	tacna_configure_jds(info);

	ret = tacna_request_irq(tacna, TACNA_IRQ_AOD_MICD_CLAMP1_RISE,
				"CLAMP rise", tacna_jackdet, info);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to get CLAMP rise IRQ: %d\n", ret);
		goto err_input;
	}

	ret = tacna_set_irq_wake(tacna, TACNA_IRQ_AOD_MICD_CLAMP1_RISE, 1);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to set JD rise IRQ wake: %d\n",	ret);
		goto err_rise;
	}

	ret = tacna_request_irq(tacna, TACNA_IRQ_AOD_MICD_CLAMP1_FALL,
				"CLAMP fall", tacna_jackdet, info);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get CLAMP fall IRQ: %d\n", ret);
		goto err_rise_wake;
	}

	ret = tacna_set_irq_wake(tacna, TACNA_IRQ_AOD_MICD_CLAMP1_FALL, 1);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to set CLAMP fall IRQ wake: %d\n", ret);
		goto err_fall;
	}

	ret = tacna_request_irq(tacna, TACNA_IRQ_MICDET1,
				 "MICDET", tacna_micdet, info);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get MICDET IRQ: %d\n", ret);
		goto err_fall_wake;
	}

	ret = tacna_request_irq(tacna, TACNA_IRQ_HPDET,
				 "HPDET", tacna_hpdet_handler, info);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get HPDET IRQ: %d\n", ret);
		goto err_micdet;
	}

	ret = regulator_allow_bypass(info->micvdd, true);
	if (ret)
		dev_warn(info->dev,
			 "Failed to set VOUT_MIC to bypass: %d\n", ret);

	pm_runtime_put(&pdev->dev);

	if (IS_ENABLED(CONFIG_EXTCON_TACNA_INPUT_EVENT)) {
		input_set_capability(info->input, EV_SW, SW_MICROPHONE_INSERT);
		input_set_capability(info->input, EV_SW, SW_HEADPHONE_INSERT);
		input_set_capability(info->input, EV_SW,
				     SW_JACK_PHYSICAL_INSERT);
	}

	ret = input_register_device(info->input);
	if (ret) {
		dev_err(&pdev->dev, "Can't register input device: %d\n", ret);
		goto err_hpdet;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_hp1_impedance);
	if (ret)
		dev_warn(&pdev->dev,
			 "Failed to create sysfs node for hp_impedance %d\n",
			 ret);

	tacna_extcon_dump_config(info);

	return 0;

err_hpdet:
	tacna_free_irq(tacna, TACNA_IRQ_HPDET, info);
err_micdet:
	tacna_free_irq(tacna, TACNA_IRQ_MICDET1, info);
err_fall_wake:
	tacna_set_irq_wake(tacna, TACNA_IRQ_AOD_MICD_CLAMP1_FALL, 0);
err_fall:
	tacna_free_irq(tacna, TACNA_IRQ_AOD_MICD_CLAMP1_FALL, info);
err_rise_wake:
	tacna_set_irq_wake(tacna, TACNA_IRQ_AOD_MICD_CLAMP1_RISE, 0);
err_rise:
	tacna_free_irq(tacna, TACNA_IRQ_AOD_MICD_CLAMP1_RISE, info);
err_input:
err_register:
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int tacna_extcon_remove(struct platform_device *pdev)
{
	struct tacna_extcon *info = platform_get_drvdata(pdev);
	struct tacna *tacna = info->tacna;

	pm_runtime_disable(&pdev->dev);

	tacna_set_irq_wake(tacna, TACNA_IRQ_AOD_MICD_CLAMP1_RISE, 0);
	tacna_set_irq_wake(tacna, TACNA_IRQ_AOD_MICD_CLAMP1_FALL, 0);
	tacna_free_irq(tacna, TACNA_IRQ_HPDET, info);
	tacna_free_irq(tacna, TACNA_IRQ_MICDET1, info);
	tacna_free_irq(tacna, TACNA_IRQ_AOD_MICD_CLAMP1_RISE, info);
	tacna_free_irq(tacna, TACNA_IRQ_AOD_MICD_CLAMP1_FALL, info);

	regmap_update_bits(tacna->regmap, TACNA_MICD_CLAMP_CONTROL,
			   TACNA_MICD_CLAMP1_MODE_MASK, 0);
	regmap_update_bits(tacna->regmap, TACNA_JACK_DETECT,
			   TACNA_JD1_EN | TACNA_JD2_EN, 0);

	device_remove_file(&pdev->dev, &dev_attr_hp1_impedance);

	return 0;
}

static struct platform_driver tacna_extcon_driver = {
	.driver		= {
		.name	= "tacna-extcon",
	},
	.probe		= &tacna_extcon_probe,
	.remove		= &tacna_extcon_remove,
};

module_platform_driver(tacna_extcon_driver);

MODULE_DESCRIPTION("Tacna switch driver");
MODULE_AUTHOR("Charles Keepax <ckeepax@opensource.wolfsonmicro.com>");
MODULE_AUTHOR("Richard Fitzgerald <rf@opensource.wolfsonmicro.com>");
MODULE_AUTHOR("Piotr Stankiewicz <piotrs@opensource.cirrus.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:tacna-extcon");
