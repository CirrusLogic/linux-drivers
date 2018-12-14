/*
 * cs47l96.c  --  ALSA SoC Audio driver for CS47L96/CS47L97 codecs
 *
 * Copyright 2017-2018 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/completion.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include <linux/irqchip/irq-tacna.h>

#include <linux/mfd/tacna/core.h>
#include <linux/mfd/tacna/registers.h>

#include "tacna.h"
#include "wm_adsp.h"

#define CS47L96_N_AUXPDM 3
#define CS47L96_N_FLL 2
#define CS47L96_NUM_DSP 1
#define CS47L96_DSP_N_RX_CHANNELS 12
#define CS47L96_DSP_N_TX_CHANNELS 8

#define CS47L96_OUTH_CP_POLL_US		1000
#define CS47L96_OUTH_CP_POLL_TIMEOUT_US	100000

static const DECLARE_TLV_DB_SCALE(cs47l96_outh_digital_tlv, -12750, 50, 0);

#define CS47L96_ANC_INPUT_ROUTES(widget, name) \
	{ widget, NULL, name " NG Mux" }, \
	{ name " NG Internal", NULL, "ANC NG Clock" }, \
	{ name " NG Internal", NULL, name " Channel" }, \
	{ name " NG External", NULL, "ANC NG External Clock" }, \
	{ name " NG External", NULL, name " Channel" }, \
	{ name " NG Mux", "None", name " Channel" }, \
	{ name " NG Mux", "Internal", name " NG Internal" }, \
	{ name " NG Mux", "External", name " NG External" }, \
	{ name " Channel", "Left", name " Left Input" }, \
	{ name " Channel", "Right", name " Right Input" }, \
	{ name " Left Input", "IN1", "IN1L PGA" }, \
	{ name " Right Input", "IN1", "IN1R PGA" }, \
	{ name " Left Input", "IN2", "IN2L PGA" }, \
	{ name " Right Input", "IN2", "IN2R PGA" }, \
	{ name " Left Input", "IN3", "IN3L PGA" }, \
	{ name " Right Input", "IN3", "IN3R PGA" }, \
	{ name " Left Input", "IN4", "IN4L PGA" }, \
	{ name " Right Input", "IN4", "IN4R PGA" }

#define CS47L96_ANC_OUTPUT_ROUTES(widget, name) \
	{ widget, NULL, name " ANC Source" }, \
	{ name " ANC Source", "ANC Left Channel", "ANCL" }

struct cs47l96 {
	struct tacna_priv core;
	struct tacna_fll fll[CS47L96_N_FLL];
	struct completion outh_enabled;
	struct completion outh_disabled;
	unsigned int outh_main_vol[2];
};

static const DECLARE_TLV_DB_SCALE(cs47l96_aux_tlv, -9600, 50, 0);

static const struct wm_adsp_region cs47l96_dsp1_regions[] = {
	{ .type = WMFW_HALO_PM_PACKED, .base = 0x3800000 },
	{ .type = WMFW_HALO_XM_PACKED, .base = 0x2000000 },
	{ .type = WMFW_ADSP2_XM, .base = 0x2800000 },
	{ .type = WMFW_HALO_YM_PACKED, .base = 0x2C00000 },
	{ .type = WMFW_ADSP2_YM, .base = 0x3400000 },
};

static int cs47l96_out_ev(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cs47l96 *cs47l96 = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = cs47l96->core.tacna;
	unsigned int val;
	int ret;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = tacna_read_poll_timeout(tacna->regmap,
				TACNA_OUTHL_CONTROL1,
				val,
				!(val & TACNA_CP_EN_OUTHL_EN_MASK),
				CS47L96_OUTH_CP_POLL_US,
				CS47L96_OUTH_CP_POLL_TIMEOUT_US);
		if (ret)
			goto err;

		ret = tacna_read_poll_timeout(tacna->regmap,
				TACNA_OUTHR_CONTROL1,
				val,
				!(val & TACNA_CP_EN_OUTHR_EN_MASK),
				CS47L96_OUTH_CP_POLL_US,
				CS47L96_OUTH_CP_POLL_TIMEOUT_US);
		if (ret)
			goto err;

		break;
	default:
		break;
	}

	return tacna_hp_ev(w, kcontrol, event);

err:
	/* Must not enable the output if couldn't confirm that CP is off */
	dev_warn(codec->dev, "Failed to get OUTH CP disabled (%d)\n", ret);
	return ret;
}

static int cs47l96_wait_for_cp_disable(struct tacna *tacna)
{
	struct regmap *regmap = tacna->regmap;
	unsigned int val;
	int ret;

	ret = tacna_read_poll_timeout(regmap, TACNA_HP1L_CTRL, val,
				       !(val & TACNA_CP_EN_HP1L_MASK),
				       CS47L96_OUTH_CP_POLL_US,
				       CS47L96_OUTH_CP_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	ret = tacna_read_poll_timeout(regmap, TACNA_HP1R_CTRL, val,
				       !(val & TACNA_CP_EN_HP1R_MASK),
				       CS47L96_OUTH_CP_POLL_US,
				       CS47L96_OUTH_CP_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	return 0;
}

static int cs47l96_outh_ev(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cs47l96 *cs47l96 = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = cs47l96->core.tacna;
	struct regmap *regmap = tacna->regmap;
	unsigned int val;
	int ret, accdet;
	long time_left;
	bool outh_upd = false;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = cs47l96_wait_for_cp_disable(tacna);
		if (ret) {
			dev_err(codec->dev,
				"OUTH enable failed (OUT1/2 CP enabled): %d\n",
				ret);
			return ret;
		}

		/* OUTH requires DAC clock to be set to its highest setting */
		ret = regmap_read(regmap, TACNA_DAC_CLK_CONTROL1, &val);
		if (ret) {
			dev_err(codec->dev, "Failed to read 0x%x: %d\n",
				TACNA_DAC_CLK_CONTROL1, ret);
			return ret;
		}

		val = (val & TACNA_DAC_CLK_SRC_FREQ_MASK) >>
			TACNA_DAC_CLK_SRC_FREQ_SHIFT;
		if (val != 2) {
			dev_err(codec->dev,
				"OUTH enable failed (incompatible DACCLK).\n");
			return -EINVAL;
		}

		ret = regmap_update_bits(regmap, TACNA_OUTHL_VOLUME_1,
					 TACNA_OUTHL_VOL_MASK,
					 cs47l96->outh_main_vol[0]);
		if (ret)
			dev_warn(codec->dev,
				 "Failed to apply cached OUTHL volume: %d\n",
				 ret);

		ret = regmap_update_bits(regmap, TACNA_OUTHR_VOLUME_1,
					 TACNA_OUTHR_VOL_MASK,
					 cs47l96->outh_main_vol[1]);
		if (ret)
			dev_warn(codec->dev,
				 "Failed to apply cached OUTHR volume: %d\n",
				 ret);

		val = 1 << TACNA_OUTH_EN_SHIFT;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		val = 0;
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* ensure the cache matches what the hardware will revert to */
		ret = regmap_update_bits(regmap, TACNA_OUTHL_VOLUME_1,
					 TACNA_OUTHL_VOL_MASK, 0xc0);
		if (ret)
			dev_warn(codec->dev,
				 "Failed to set OUTHL volume to default: %d\n",
				 ret);

		ret = regmap_update_bits(regmap, TACNA_OUTHR_VOLUME_1,
					 TACNA_OUTHR_VOL_MASK, 0xc0);
		if (ret)
			dev_warn(codec->dev,
				 "Failed to set OUTHR volume to default: %d\n",
				 ret);
		return 0;
	default:
		return 0;
	}

	/*
	 * save desired OUTH state (to avoid adding a specific member for OUTH
	 * a free bit (31) in hp_ena is used)
	 */
	if (val)
		tacna->hp_ena |= (1 << 31);
	else
		tacna->hp_ena &= ~(1 << 31);

	/*
	 * do not enable output if accessory detect is running on its pins
	 * or a short circuit was detected
	 */
	accdet = tacna_get_accdet_for_output(codec, 1);
	if (accdet >= 0 &&
	    (tacna->hpdet_clamp[accdet] || tacna->hpdet_shorted[accdet]))
		val = 0;

	if (accdet >= 0) {
		reinit_completion(&cs47l96->outh_enabled);
		reinit_completion(&cs47l96->outh_disabled);
	}

	ret = regmap_update_bits_check(regmap, TACNA_OUTH_ENABLE_1,
				       TACNA_OUTH_EN_MASK, val, &outh_upd);
	if (ret)
		dev_err(codec->dev, "Failed to toggle OUTH enable: %d\n", ret);

	if (outh_upd) { /* wait for enable/disable to take effect */
		if (val)
			time_left = wait_for_completion_timeout(
							&cs47l96->outh_enabled,
							msecs_to_jiffies(100));
		else
			time_left = wait_for_completion_timeout(
							&cs47l96->outh_disabled,
							msecs_to_jiffies(100));

		if (!time_left)
			dev_warn(codec->dev, "OUTH %s timed out.\n",
				 (val) ? "enable" : "disable");
	}

	return ret;
}

static int cs47l96_dsd_processor_ev(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cs47l96 *cs47l96 = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = cs47l96->core.tacna;
	int ret;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* this must be enabled for DSD+AUX */
		ret = regmap_update_bits(tacna->regmap, TACNA_OUTH_ENABLE_1,
					 TACNA_DSD1_IF_EN_MASK,
					 TACNA_DSD1_IF_EN);
		if (ret)
			dev_warn(codec->dev,
				"Failed to write to OUTH_ENABLE_1: %d\n", ret);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		ret = regmap_update_bits(tacna->regmap, TACNA_OUTH_ENABLE_1,
					 TACNA_DSD1_IF_EN_MASK, 0);
		if (ret)
			dev_warn(codec->dev,
				"Failed to write to OUTH_ENABLE_1: %d\n", ret);
		break;
	default:
		break;
	}

	return 0;
}

static irqreturn_t cs47l96_outh_enable(int irq, void *data)
{
	struct cs47l96 *cs47l96 = data;

	dev_dbg(cs47l96->core.dev, "OUTH enable interrupt\n");

	complete(&cs47l96->outh_enabled);

	return IRQ_HANDLED;
}

static irqreturn_t cs47l96_outh_disable(int irq, void *data)
{
	struct cs47l96 *cs47l96 = data;

	dev_dbg(cs47l96->core.dev, "OUTH disable interrupt\n");

	complete(&cs47l96->outh_disabled);

	return IRQ_HANDLED;
}

static irqreturn_t cs47l96_mpu_fault_irq(int irq, void *data)
{
	struct wm_adsp *dsp = data;

	return wm_halo_bus_error(dsp);
}

static const char * const cs47l96_out1_demux_texts[] = {
	"HP1", "HP2",
};

static SOC_ENUM_SINGLE_DECL(cs47l96_out1_demux_enum, SND_SOC_NOPM, 0,
			    cs47l96_out1_demux_texts);

static const struct snd_kcontrol_new cs47l96_out1_demux =
	SOC_DAPM_ENUM_EXT("OUT1 Demux", cs47l96_out1_demux_enum,
			  tacna_get_out1_demux, tacna_put_out1_demux);

static const char * const cs47l96_out5_mux_texts[] = {
	"OUT5", "OUT1",
};
static SOC_ENUM_SINGLE_DECL(cs47l96_out5_mux_enum,
			    TACNA_OUT5L_CONTROL_1, TACNA_OUT5_OUT1_SEL_SHIFT,
			    cs47l96_out5_mux_texts);

static const struct snd_kcontrol_new cs47l96_out5_mux =
	SOC_DAPM_ENUM("OUT5 Source", cs47l96_out5_mux_enum);

static const struct soc_enum cs47l96_outh_rate =
	SOC_VALUE_ENUM_SINGLE(TACNA_OUTH_CONFIG_1,
			      TACNA_OUTH_RATE_SHIFT,
			      TACNA_OUTH_RATE_MASK >> TACNA_OUTH_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val);

static int cs47l96_get_outh_main_volume(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct cs47l96 *cs47l96 = dev_get_drvdata(codec->dev);
	int min = mc->min, shift = mc->shift, rshift = mc->rshift;

	ucontrol->value.integer.value[0] =
		(cs47l96->outh_main_vol[0] - min) >> shift;

	ucontrol->value.integer.value[1] =
		(cs47l96->outh_main_vol[1] - min) >> rshift;

	return 0;
}

static int cs47l96_put_outh_main_volume(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct cs47l96 *cs47l96 = dev_get_drvdata(codec->dev);
	int min = mc->min, shift = mc->shift, rshift = mc->rshift;

	snd_soc_dapm_mutex_lock(dapm);

	cs47l96->outh_main_vol[0] =
		(ucontrol->value.integer.value[0] + min) << shift;

	cs47l96->outh_main_vol[1] =
		(ucontrol->value.integer.value[1] + min) << rshift;

	snd_soc_dapm_mutex_unlock(dapm);

	return snd_soc_put_volsw(kcontrol, ucontrol);
}

static const char * const cs47l96_dop_width_texts[] = {
	"auto", "32bit", "16bit",
};

static SOC_ENUM_SINGLE_DECL(cs47l96_dop_width_enum, TACNA_DOP1_CONTROL1,
			    TACNA_DOP1_WIDTH_SHIFT, cs47l96_dop_width_texts);

static const struct soc_enum tacna_aobridge1_rate[] = {
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE1_CH1_CTRL,
			TACNA_AOBRIDGE1_IN1_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE1_CH1_CTRL,
			      TACNA_AOBRIDGE1_OUT1_RATE_SHIFT,
			      TACNA_AOBRIDGE1_OUT1_RATE_MASK >>
			      TACNA_AOBRIDGE1_OUT1_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE1_CH2_CTRL,
			TACNA_AOBRIDGE1_IN2_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE1_CH2_CTRL,
			      TACNA_AOBRIDGE1_OUT2_RATE_SHIFT,
			      TACNA_AOBRIDGE1_OUT2_RATE_MASK >>
			      TACNA_AOBRIDGE1_OUT2_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE1_CH3_CTRL,
			TACNA_AOBRIDGE1_IN3_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE1_CH3_CTRL,
			      TACNA_AOBRIDGE1_OUT3_RATE_SHIFT,
			      TACNA_AOBRIDGE1_OUT3_RATE_MASK >>
			      TACNA_AOBRIDGE1_OUT3_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE1_CH4_CTRL,
			TACNA_AOBRIDGE1_IN4_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE1_CH4_CTRL,
			      TACNA_AOBRIDGE1_OUT4_RATE_SHIFT,
			      TACNA_AOBRIDGE1_OUT4_RATE_MASK >>
			      TACNA_AOBRIDGE1_OUT4_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE1_CH5_CTRL,
			TACNA_AOBRIDGE1_IN5_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE1_CH5_CTRL,
			      TACNA_AOBRIDGE1_OUT5_RATE_SHIFT,
			      TACNA_AOBRIDGE1_OUT5_RATE_MASK >>
			      TACNA_AOBRIDGE1_OUT5_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE1_CH6_CTRL,
			TACNA_AOBRIDGE1_IN6_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE1_CH6_CTRL,
			      TACNA_AOBRIDGE1_OUT6_RATE_SHIFT,
			      TACNA_AOBRIDGE1_OUT6_RATE_MASK >>
			      TACNA_AOBRIDGE1_OUT6_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE1_CH7_CTRL,
			TACNA_AOBRIDGE1_IN7_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE1_CH7_CTRL,
			      TACNA_AOBRIDGE1_OUT7_RATE_SHIFT,
			      TACNA_AOBRIDGE1_OUT7_RATE_MASK >>
			      TACNA_AOBRIDGE1_OUT7_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE1_CH8_CTRL,
			TACNA_AOBRIDGE1_IN8_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE1_CH8_CTRL,
			      TACNA_AOBRIDGE1_OUT8_RATE_SHIFT,
			      TACNA_AOBRIDGE1_OUT8_RATE_MASK >>
			      TACNA_AOBRIDGE1_OUT8_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
};

static const struct soc_enum tacna_aobridge2_rate[] = {
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE2_CH1_CTRL,
			      TACNA_AOBRIDGE2_IN1_RATE_SHIFT,
			      TACNA_AOBRIDGE2_IN1_RATE_MASK >>
			      TACNA_AOBRIDGE2_IN1_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE2_CH1_CTRL,
			TACNA_AOBRIDGE2_OUT1_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE2_CH2_CTRL,
			      TACNA_AOBRIDGE2_IN2_RATE_SHIFT,
			      TACNA_AOBRIDGE2_IN2_RATE_MASK >>
			      TACNA_AOBRIDGE2_IN2_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE2_CH2_CTRL,
			TACNA_AOBRIDGE2_OUT2_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE2_CH3_CTRL,
			      TACNA_AOBRIDGE2_IN3_RATE_SHIFT,
			      TACNA_AOBRIDGE2_IN3_RATE_MASK >>
			      TACNA_AOBRIDGE2_IN3_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE2_CH3_CTRL,
			TACNA_AOBRIDGE2_OUT3_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE2_CH4_CTRL,
			      TACNA_AOBRIDGE2_IN4_RATE_SHIFT,
			      TACNA_AOBRIDGE2_IN4_RATE_MASK >>
			      TACNA_AOBRIDGE2_IN4_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE2_CH4_CTRL,
			TACNA_AOBRIDGE2_OUT4_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE2_CH5_CTRL,
			      TACNA_AOBRIDGE2_IN5_RATE_SHIFT,
			      TACNA_AOBRIDGE2_IN5_RATE_MASK >>
			      TACNA_AOBRIDGE2_IN5_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE2_CH5_CTRL,
			TACNA_AOBRIDGE2_OUT5_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE2_CH6_CTRL,
			      TACNA_AOBRIDGE2_IN6_RATE_SHIFT,
			      TACNA_AOBRIDGE2_IN6_RATE_MASK >>
			      TACNA_AOBRIDGE2_IN6_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE2_CH6_CTRL,
			TACNA_AOBRIDGE2_OUT6_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE2_CH7_CTRL,
			      TACNA_AOBRIDGE2_IN7_RATE_SHIFT,
			      TACNA_AOBRIDGE2_IN7_RATE_MASK >>
			      TACNA_AOBRIDGE2_IN7_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE2_CH7_CTRL,
			TACNA_AOBRIDGE2_OUT7_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
	SOC_VALUE_ENUM_SINGLE(TACNA_AOBRIDGE2_CH8_CTRL,
			      TACNA_AOBRIDGE2_IN8_RATE_SHIFT,
			      TACNA_AOBRIDGE2_IN8_RATE_MASK >>
			      TACNA_AOBRIDGE2_IN8_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_ENUM_SINGLE(TACNA_AOBRIDGE2_CH8_CTRL,
			TACNA_AOBRIDGE2_OUT8_RATE_SHIFT,
			ARRAY_SIZE(tacna_ao_rate_text),
			tacna_ao_rate_text),
};

static const struct snd_kcontrol_new cs47l96_snd_controls[] = {
SOC_ENUM("IN1 OSR", tacna_in_dmic_osr[0]),
SOC_ENUM("IN2 OSR", tacna_in_dmic_osr[1]),
SOC_ENUM("IN3 OSR", tacna_in_dmic_osr[2]),
SOC_ENUM("IN4 OSR", tacna_in_dmic_osr[3]),

SOC_SINGLE_RANGE_TLV("IN1L Volume", TACNA_IN1L_CONTROL2,
		     TACNA_IN1L_PGA_VOL_SHIFT, 0x40, 0x5f, 0, tacna_ana_tlv),
SOC_SINGLE_RANGE_TLV("IN1R Volume", TACNA_IN1R_CONTROL2,
		     TACNA_IN1R_PGA_VOL_SHIFT, 0x40, 0x5f, 0, tacna_ana_tlv),
SOC_SINGLE_RANGE_TLV("IN2L Volume", TACNA_IN2L_CONTROL2,
		     TACNA_IN2L_PGA_VOL_SHIFT, 0x40, 0x5f, 0, tacna_ana_tlv),
SOC_SINGLE_RANGE_TLV("IN2R Volume", TACNA_IN2R_CONTROL2,
		     TACNA_IN2R_PGA_VOL_SHIFT, 0x40, 0x5f, 0, tacna_ana_tlv),

SOC_ENUM("IN HPF Cutoff Frequency", tacna_in_hpf_cut_enum),

SOC_SINGLE_EXT("IN1L LP Switch", TACNA_IN1L_CONTROL1, TACNA_IN1L_LP_MODE_SHIFT,
	       1, 0, snd_soc_get_volsw, tacna_low_power_mode_put),
SOC_SINGLE_EXT("IN1R LP Switch", TACNA_IN1R_CONTROL1, TACNA_IN1R_LP_MODE_SHIFT,
	       1, 0, snd_soc_get_volsw, tacna_low_power_mode_put),
SOC_SINGLE_EXT("IN2L LP Switch", TACNA_IN2L_CONTROL1, TACNA_IN2R_LP_MODE_SHIFT,
	       1, 0, snd_soc_get_volsw, tacna_low_power_mode_put),
SOC_SINGLE_EXT("IN2R LP Switch", TACNA_IN2R_CONTROL1, TACNA_IN2R_LP_MODE_SHIFT,
	       1, 0, snd_soc_get_volsw, tacna_low_power_mode_put),

SOC_SINGLE("IN1L HPF Switch", TACNA_IN1L_CONTROL1, TACNA_IN1L_HPF_SHIFT, 1, 0),
SOC_SINGLE("IN1R HPF Switch", TACNA_IN1R_CONTROL1, TACNA_IN1R_HPF_SHIFT, 1, 0),
SOC_SINGLE("IN2L HPF Switch", TACNA_IN2L_CONTROL1, TACNA_IN2L_HPF_SHIFT, 1, 0),
SOC_SINGLE("IN2R HPF Switch", TACNA_IN2R_CONTROL1, TACNA_IN2R_HPF_SHIFT, 1, 0),
SOC_SINGLE("IN3L HPF Switch", TACNA_IN3L_CONTROL1, TACNA_IN3L_HPF_SHIFT, 1, 0),
SOC_SINGLE("IN3R HPF Switch", TACNA_IN3R_CONTROL1, TACNA_IN3R_HPF_SHIFT, 1, 0),
SOC_SINGLE("IN4L HPF Switch", TACNA_IN4L_CONTROL1, TACNA_IN4L_HPF_SHIFT, 1, 0),
SOC_SINGLE("IN4R HPF Switch", TACNA_IN4R_CONTROL1, TACNA_IN4R_HPF_SHIFT, 1, 0),

SOC_SINGLE("IN1 PDM CLK AO Switch", TACNA_IN_PDMCLK_SEL,
	   TACNA_IN1_PDMCLK_SEL_SHIFT, 1, 0),
SOC_SINGLE("IN2 PDM CLK AO Switch", TACNA_IN_PDMCLK_SEL,
	   TACNA_IN2_PDMCLK_SEL_SHIFT, 1, 0),
SOC_SINGLE("IN3 PDM CLK AO Switch", TACNA_IN_PDMCLK_SEL,
	   TACNA_IN3_PDMCLK_SEL_SHIFT, 1, 0),

SOC_SINGLE_EXT_TLV("IN1L Digital Volume", TACNA_IN1L_CONTROL2,
		   TACNA_IN1L_VOL_SHIFT, 0xbf, 0,
		   snd_soc_get_volsw, tacna_in_put_volsw,
		   tacna_digital_tlv),
SOC_SINGLE_EXT_TLV("IN1R Digital Volume", TACNA_IN1R_CONTROL2,
		   TACNA_IN1R_VOL_SHIFT, 0xbf, 0,
		   snd_soc_get_volsw, tacna_in_put_volsw,
		   tacna_digital_tlv),
SOC_SINGLE_EXT_TLV("IN2L Digital Volume", TACNA_IN2L_CONTROL2,
		   TACNA_IN2L_VOL_SHIFT, 0xbf, 0,
		   snd_soc_get_volsw, tacna_in_put_volsw,
		   tacna_digital_tlv),
SOC_SINGLE_EXT_TLV("IN2R Digital Volume", TACNA_IN2R_CONTROL2,
		   TACNA_IN2R_VOL_SHIFT, 0xbf, 0,
		   snd_soc_get_volsw, tacna_in_put_volsw,
		   tacna_digital_tlv),
SOC_SINGLE_EXT_TLV("IN3L Digital Volume", TACNA_IN3L_CONTROL2,
		   TACNA_IN3L_VOL_SHIFT, 0xbf, 0,
		   snd_soc_get_volsw, tacna_in_put_volsw,
		   tacna_digital_tlv),
SOC_SINGLE_EXT_TLV("IN3R Digital Volume", TACNA_IN3R_CONTROL2,
		   TACNA_IN3R_VOL_SHIFT, 0xbf, 0,
		   snd_soc_get_volsw, tacna_in_put_volsw,
		   tacna_digital_tlv),
SOC_SINGLE_EXT_TLV("IN4L Digital Volume", TACNA_IN4L_CONTROL2,
		   TACNA_IN4L_VOL_SHIFT, 0xbf, 0,
		   snd_soc_get_volsw, tacna_in_put_volsw,
		   tacna_digital_tlv),
SOC_SINGLE_EXT_TLV("IN4R Digital Volume", TACNA_IN4R_CONTROL2,
	TACNA_IN4R_VOL_SHIFT, 0xbf, 0,
		   snd_soc_get_volsw, tacna_in_put_volsw,
		   tacna_digital_tlv),

SOC_ENUM("IN3 Swap Chan", tacna_in_swap_chan_ctrl[2]),
SOC_ENUM("IN4 Swap Chan", tacna_in_swap_chan_ctrl[3]),

SOC_ENUM("Input Ramp Up", tacna_in_vi_ramp),
SOC_ENUM("Input Ramp Down", tacna_in_vd_ramp),

TACNA_FRF_BYTES("FRF COEFF 1L", TACNA_FRF_COEFF_1L_1, TACNA_FRF_COEFF_LEN),
TACNA_FRF_BYTES("FRF COEFF 1R", TACNA_FRF_COEFF_1R_1, TACNA_FRF_COEFF_LEN),
TACNA_FRF_BYTES("FRF COEFF 5L", TACNA_FRF_COEFF_5L_1, TACNA_FRF_COEFF_LEN),
TACNA_FRF_BYTES("FRF COEFF 5R", TACNA_FRF_COEFF_5R_1, TACNA_FRF_COEFF_LEN),

TACNA_MIXER_CONTROLS("EQ1", TACNA_EQ1_INPUT1),
TACNA_MIXER_CONTROLS("EQ2", TACNA_EQ2_INPUT1),
TACNA_MIXER_CONTROLS("EQ3", TACNA_EQ3_INPUT1),
TACNA_MIXER_CONTROLS("EQ4", TACNA_EQ4_INPUT1),

SOC_ENUM_EXT("EQ1 Mode", tacna_eq_mode[0], tacna_eq_mode_get,
	     tacna_eq_mode_put),
TACNA_EQ_COEFF_CONTROLS(EQ1),
SOC_SINGLE_TLV("EQ1 B1 Volume", TACNA_EQ1_GAIN1, TACNA_EQ1_B1_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ1 B2 Volume", TACNA_EQ1_GAIN1, TACNA_EQ1_B2_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ1 B3 Volume", TACNA_EQ1_GAIN1, TACNA_EQ1_B3_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ1 B4 Volume", TACNA_EQ1_GAIN1, TACNA_EQ1_B4_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ1 B5 Volume", TACNA_EQ1_GAIN2, TACNA_EQ1_B5_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),

SOC_ENUM_EXT("EQ2 Mode", tacna_eq_mode[1], tacna_eq_mode_get,
	     tacna_eq_mode_put),
TACNA_EQ_COEFF_CONTROLS(EQ2),
SOC_SINGLE_TLV("EQ2 B1 Volume", TACNA_EQ2_GAIN1, TACNA_EQ2_B1_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ2 B2 Volume", TACNA_EQ2_GAIN1, TACNA_EQ2_B2_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ2 B3 Volume", TACNA_EQ2_GAIN1, TACNA_EQ2_B3_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ2 B4 Volume", TACNA_EQ2_GAIN1, TACNA_EQ2_B4_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ2 B5 Volume", TACNA_EQ2_GAIN2, TACNA_EQ2_B5_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),

SOC_ENUM_EXT("EQ3 Mode", tacna_eq_mode[2], tacna_eq_mode_get,
	     tacna_eq_mode_put),
TACNA_EQ_COEFF_CONTROLS(EQ3),
SOC_SINGLE_TLV("EQ3 B1 Volume", TACNA_EQ3_GAIN1, TACNA_EQ3_B1_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ3 B2 Volume", TACNA_EQ3_GAIN1, TACNA_EQ3_B2_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ3 B3 Volume", TACNA_EQ3_GAIN1, TACNA_EQ3_B3_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ3 B4 Volume", TACNA_EQ3_GAIN1, TACNA_EQ3_B4_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ3 B5 Volume", TACNA_EQ3_GAIN2, TACNA_EQ3_B5_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),

SOC_ENUM_EXT("EQ4 Mode", tacna_eq_mode[3], tacna_eq_mode_get,
	     tacna_eq_mode_put),
TACNA_EQ_COEFF_CONTROLS(EQ4),
SOC_SINGLE_TLV("EQ4 B1 Volume", TACNA_EQ4_GAIN1, TACNA_EQ4_B1_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ4 B2 Volume", TACNA_EQ4_GAIN1, TACNA_EQ4_B2_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ4 B3 Volume", TACNA_EQ4_GAIN1, TACNA_EQ4_B3_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ4 B4 Volume", TACNA_EQ4_GAIN1, TACNA_EQ4_B4_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),
SOC_SINGLE_TLV("EQ4 B5 Volume", TACNA_EQ4_GAIN2, TACNA_EQ4_B5_GAIN_SHIFT,
	       24, 0, tacna_eq_tlv),

TACNA_MIXER_CONTROLS("DRC1L", TACNA_DRC1L_INPUT1),
TACNA_MIXER_CONTROLS("DRC1R", TACNA_DRC1R_INPUT1),
TACNA_MIXER_CONTROLS("DRC2L", TACNA_DRC2L_INPUT1),
TACNA_MIXER_CONTROLS("DRC2R", TACNA_DRC2R_INPUT1),

SND_SOC_BYTES_MASK("DRC1 Coefficients", TACNA_DRC1_CONTROL1, 4,
		   TACNA_DRC1R_EN | TACNA_DRC1L_EN),
SND_SOC_BYTES_MASK("DRC2 Coefficients", TACNA_DRC2_CONTROL1, 4,
		   TACNA_DRC2R_EN | TACNA_DRC2L_EN),

TACNA_MIXER_CONTROLS("AOBRIDGE2IN1", TACNA_AOBRIDGE2_IN1_INPUT1),
TACNA_MIXER_CONTROLS("AOBRIDGE2IN2", TACNA_AOBRIDGE2_IN2_INPUT1),
TACNA_MIXER_CONTROLS("AOBRIDGE2IN3", TACNA_AOBRIDGE2_IN3_INPUT1),
TACNA_MIXER_CONTROLS("AOBRIDGE2IN4", TACNA_AOBRIDGE2_IN4_INPUT1),
TACNA_MIXER_CONTROLS("AOBRIDGE2IN5", TACNA_AOBRIDGE2_IN5_INPUT1),
TACNA_MIXER_CONTROLS("AOBRIDGE2IN6", TACNA_AOBRIDGE2_IN6_INPUT1),
TACNA_MIXER_CONTROLS("AOBRIDGE2IN7", TACNA_AOBRIDGE2_IN7_INPUT1),
TACNA_MIXER_CONTROLS("AOBRIDGE2IN8", TACNA_AOBRIDGE2_IN8_INPUT1),

TACNA_MIXER_CONTROLS("LHPF1", TACNA_LHPF1_INPUT1),
TACNA_MIXER_CONTROLS("LHPF2", TACNA_LHPF2_INPUT1),
TACNA_MIXER_CONTROLS("LHPF3", TACNA_LHPF3_INPUT1),
TACNA_MIXER_CONTROLS("LHPF4", TACNA_LHPF4_INPUT1),

TACNA_LHPF_CONTROL("LHPF1 Coefficients", TACNA_LHPF1_COEFF),
TACNA_LHPF_CONTROL("LHPF2 Coefficients", TACNA_LHPF2_COEFF),
TACNA_LHPF_CONTROL("LHPF3 Coefficients", TACNA_LHPF3_COEFF),
TACNA_LHPF_CONTROL("LHPF4 Coefficients", TACNA_LHPF4_COEFF),

SOC_ENUM("LHPF1 Mode", tacna_lhpf1_mode),
SOC_ENUM("LHPF2 Mode", tacna_lhpf2_mode),
SOC_ENUM("LHPF3 Mode", tacna_lhpf3_mode),
SOC_ENUM("LHPF4 Mode", tacna_lhpf4_mode),

SND_SOC_BYTES("ANC Coefficients", TACNA_ANC_CTRL_4,
	      (TACNA_ANC_CTRL_13 - TACNA_ANC_CTRL_4) / 4 + 1),
SND_SOC_BYTES("ANCL Config", TACNA_ANC_L_CTRL_1, 1),
SND_SOC_BYTES("ANCL Coefficients", TACNA_ANC_L_CTRL_3,
	      (TACNA_ANC_L_CTRL_66 - TACNA_ANC_L_CTRL_3) / 4 + 1),

TACNA_RATE_CONTROL("Sample Rate 1", 1),
TACNA_RATE_CONTROL("Sample Rate 2", 2),
TACNA_RATE_CONTROL("Sample Rate 3", 3),
TACNA_ASYNC_RATE_CONTROL("Async Sample Rate 1", 1),
TACNA_ASYNC_RATE_CONTROL("Async Sample Rate 2", 2),

TACNA_RATE_ENUM("FX Rate", tacna_fx_rate),

TACNA_RATE_ENUM("DFC1 Rate", tacna_dfc_rate[0]),
TACNA_RATE_ENUM("DFC2 Rate", tacna_dfc_rate[1]),
TACNA_RATE_ENUM("DFC3 Rate", tacna_dfc_rate[2]),
TACNA_RATE_ENUM("DFC4 Rate", tacna_dfc_rate[3]),
TACNA_RATE_ENUM("DFC5 Rate", tacna_dfc_rate[4]),
TACNA_RATE_ENUM("DFC6 Rate", tacna_dfc_rate[5]),
TACNA_RATE_ENUM("DFC7 Rate", tacna_dfc_rate[6]),
TACNA_RATE_ENUM("DFC8 Rate", tacna_dfc_rate[7]),

TACNA_RATE_ENUM("ISRC1 FSL", tacna_isrc_fsl[0]),
TACNA_RATE_ENUM("ISRC2 FSL", tacna_isrc_fsl[1]),
TACNA_RATE_ENUM("ISRC1 FSH", tacna_isrc_fsh[0]),
TACNA_RATE_ENUM("ISRC2 FSH", tacna_isrc_fsh[1]),
TACNA_RATE_ENUM("ASRC1 Rate 1", tacna_asrc1_rate[0]),
TACNA_RATE_ENUM("ASRC1 Rate 2", tacna_asrc1_rate[1]),
TACNA_RATE_ENUM("ASRC2 Rate 1", tacna_asrc2_rate[0]),
TACNA_RATE_ENUM("ASRC2 Rate 2", tacna_asrc2_rate[1]),

TACNA_RATE_ENUM("AOBRIDGE1 IN1 Rate", tacna_aobridge1_rate[0]),
TACNA_RATE_ENUM("AOBRIDGE1 OUT1 Rate", tacna_aobridge1_rate[1]),
TACNA_RATE_ENUM("AOBRIDGE1 IN2 Rate", tacna_aobridge1_rate[2]),
TACNA_RATE_ENUM("AOBRIDGE1 OUT2 Rate", tacna_aobridge1_rate[3]),
TACNA_RATE_ENUM("AOBRIDGE1 IN3 Rate", tacna_aobridge1_rate[4]),
TACNA_RATE_ENUM("AOBRIDGE1 OUT3 Rate", tacna_aobridge1_rate[5]),
TACNA_RATE_ENUM("AOBRIDGE1 IN4 Rate", tacna_aobridge1_rate[6]),
TACNA_RATE_ENUM("AOBRIDGE1 OUT4 Rate", tacna_aobridge1_rate[7]),
TACNA_RATE_ENUM("AOBRIDGE1 IN5 Rate", tacna_aobridge1_rate[8]),
TACNA_RATE_ENUM("AOBRIDGE1 OUT5 Rate", tacna_aobridge1_rate[9]),
TACNA_RATE_ENUM("AOBRIDGE1 IN6 Rate", tacna_aobridge1_rate[10]),
TACNA_RATE_ENUM("AOBRIDGE1 OUT6 Rate", tacna_aobridge1_rate[11]),
TACNA_RATE_ENUM("AOBRIDGE1 IN7 Rate", tacna_aobridge1_rate[12]),
TACNA_RATE_ENUM("AOBRIDGE1 OUT7 Rate", tacna_aobridge1_rate[13]),
TACNA_RATE_ENUM("AOBRIDGE1 IN8 Rate", tacna_aobridge1_rate[14]),
TACNA_RATE_ENUM("AOBRIDGE1 OUT8 Rate", tacna_aobridge1_rate[15]),

TACNA_RATE_ENUM("AOBRIDGE2 IN1 Rate", tacna_aobridge2_rate[0]),
TACNA_RATE_ENUM("AOBRIDGE2 OUT1 Rate", tacna_aobridge2_rate[1]),
TACNA_RATE_ENUM("AOBRIDGE2 IN2 Rate", tacna_aobridge2_rate[2]),
TACNA_RATE_ENUM("AOBRIDGE2 OUT2 Rate", tacna_aobridge2_rate[3]),
TACNA_RATE_ENUM("AOBRIDGE2 IN3 Rate", tacna_aobridge2_rate[4]),
TACNA_RATE_ENUM("AOBRIDGE2 OUT3 Rate", tacna_aobridge2_rate[5]),
TACNA_RATE_ENUM("AOBRIDGE2 IN4 Rate", tacna_aobridge2_rate[6]),
TACNA_RATE_ENUM("AOBRIDGE2 OUT4 Rate", tacna_aobridge2_rate[7]),
TACNA_RATE_ENUM("AOBRIDGE2 IN5 Rate", tacna_aobridge2_rate[8]),
TACNA_RATE_ENUM("AOBRIDGE2 OUT5 Rate", tacna_aobridge2_rate[9]),
TACNA_RATE_ENUM("AOBRIDGE2 IN6 Rate", tacna_aobridge2_rate[10]),
TACNA_RATE_ENUM("AOBRIDGE2 OUT6 Rate", tacna_aobridge2_rate[11]),
TACNA_RATE_ENUM("AOBRIDGE2 IN7 Rate", tacna_aobridge2_rate[12]),
TACNA_RATE_ENUM("AOBRIDGE2 OUT7 Rate", tacna_aobridge2_rate[13]),
TACNA_RATE_ENUM("AOBRIDGE2 IN8 Rate", tacna_aobridge2_rate[14]),
TACNA_RATE_ENUM("AOBRIDGE2 OUT8 Rate", tacna_aobridge2_rate[15]),

SOC_ENUM("AUXPDM1 Rate", tacna_auxpdm1_freq),
SOC_ENUM("AUXPDM2 Rate", tacna_auxpdm2_freq),
SOC_ENUM("AUXPDM3 Rate", tacna_auxpdm3_freq),

TACNA_MIXER_CONTROLS("OUT1L", TACNA_OUT1L_INPUT1),
TACNA_MIXER_CONTROLS("OUT1R", TACNA_OUT1R_INPUT1),
TACNA_MIXER_CONTROLS("OUT5L", TACNA_OUT5L_INPUT1),
TACNA_MIXER_CONTROLS("OUT5R", TACNA_OUT5R_INPUT1),
TACNA_MIXER_CONTROLS("OUTAUX1L", TACNA_OUTAUX1L_INPUT1),
TACNA_MIXER_CONTROLS("OUTAUX1R", TACNA_OUTAUX1R_INPUT1),
TACNA_MIXER_CONTROLS("OUTHL", TACNA_OUTHL_INPUT1),
TACNA_MIXER_CONTROLS("OUTHR", TACNA_OUTHR_INPUT1),

SOC_DOUBLE_R("OUT1 Digital Switch", TACNA_OUT1L_VOLUME_1,
	     TACNA_OUT1R_VOLUME_1, TACNA_OUT1L_MUTE_SHIFT, 1, 1),
SOC_DOUBLE_R("OUTAUX1 Digital Switch", TACNA_OUTAUX1L_VOLUME_1,
	     TACNA_OUTAUX1R_VOLUME_1, TACNA_OUTAUX1L_MUTE_SHIFT, 1, 1),
SOC_DOUBLE_R("OUT5 Digital Switch", TACNA_OUT5L_VOLUME_1,
	     TACNA_OUT5R_VOLUME_1, TACNA_OUT5L_MUTE_SHIFT, 1, 1),

SOC_DOUBLE_R_TLV("OUT1 Main Volume", TACNA_OUT1L_VOLUME_1,
		 TACNA_OUT1R_VOLUME_1, TACNA_OUT1L_VOL_SHIFT,
		 0xc0, 0, cs47l96_aux_tlv),
SOC_DOUBLE_R_TLV("OUTAUX1 Volume", TACNA_OUTAUX1L_VOLUME_1,
		 TACNA_OUTAUX1R_VOLUME_1, TACNA_OUTAUX1L_VOL_SHIFT,
		 0xc0, 0, cs47l96_aux_tlv),
SOC_DOUBLE_R_TLV("OUT1 Digital Volume", TACNA_OUT1L_VOLUME_3,
		 TACNA_OUT1R_VOLUME_3, TACNA_OUT1L_LVL_SHIFT,
		 0xbf, 0, tacna_digital_tlv),
SOC_DOUBLE_R_TLV("OUT5 Digital Volume", TACNA_OUT5L_VOLUME_1,
		 TACNA_OUT5R_VOLUME_1, TACNA_OUT5L_VOL_SHIFT,
		 0xbf, 0, tacna_digital_tlv),

SOC_ENUM("Output Ramp Up", tacna_out_vi_ramp),
SOC_ENUM("Output Ramp Down", tacna_out_vd_ramp),

SOC_ENUM_EXT("Output Rate 1", tacna_output_rate,
	     snd_soc_get_enum_double, tacna_dac_rate_put),

SOC_ENUM_EXT("IN1L Rate", tacna_input_rate[0],
	     snd_soc_get_enum_double, tacna_in_rate_put),
SOC_ENUM_EXT("IN1R Rate", tacna_input_rate[1],
	     snd_soc_get_enum_double, tacna_in_rate_put),
SOC_ENUM_EXT("IN2L Rate", tacna_input_rate[2],
	     snd_soc_get_enum_double, tacna_in_rate_put),
SOC_ENUM_EXT("IN2R Rate", tacna_input_rate[3],
	     snd_soc_get_enum_double, tacna_in_rate_put),
SOC_ENUM_EXT("IN3L Rate", tacna_input_rate[4],
	     snd_soc_get_enum_double, tacna_in_rate_put),
SOC_ENUM_EXT("IN3R Rate", tacna_input_rate[5],
	     snd_soc_get_enum_double, tacna_in_rate_put),
SOC_ENUM_EXT("IN4L Rate", tacna_input_rate[6],
	     snd_soc_get_enum_double, tacna_in_rate_put),
SOC_ENUM_EXT("IN4R Rate", tacna_input_rate[7],
	     snd_soc_get_enum_double, tacna_in_rate_put),

SOC_ENUM_EXT("OUTH Rate", cs47l96_outh_rate,
	     snd_soc_get_enum_double, tacna_dac_rate_put),

SOC_DOUBLE_R_EXT_TLV("OUTH Main Volume", TACNA_OUTHL_VOLUME_1,
		     TACNA_OUTHR_VOLUME_1, TACNA_OUTHL_VOL_SHIFT, 0xc0, 0,
		     cs47l96_get_outh_main_volume, cs47l96_put_outh_main_volume,
		     cs47l96_aux_tlv),

SOC_DOUBLE_TLV("OUTH DSD Digital Volume", TACNA_DSD1_VOLUME1,
	       TACNA_DSD1L_VOL_SHIFT, TACNA_DSD1R_VOL_SHIFT, 0xfe, 1,
	       cs47l96_outh_digital_tlv),
SOC_DOUBLE("OUTH DSD Digital Switch", TACNA_DSD1_VOLUME1,
	   TACNA_DSD1L_MUTE_SHIFT, TACNA_DSD1R_MUTE_SHIFT, 1, 1),

SOC_ENUM("DoP Data Width", cs47l96_dop_width_enum),

SOC_DOUBLE_TLV("OUTH PCM Digital Volume", TACNA_OUTH_PCM_CONTROL1,
	       TACNA_OUTHL_LVL_SHIFT, TACNA_OUTHR_LVL_SHIFT, 0xfe, 1,
	       cs47l96_outh_digital_tlv),
SOC_DOUBLE("OUTH PCM Digital Switch", TACNA_OUTH_PCM_CONTROL1,
	   TACNA_OUTHL_MUTE_SHIFT, TACNA_OUTHR_MUTE_SHIFT, 1, 1),

SOC_SINGLE_EXT("DFC1 Dither", TACNA_DFC1_CH1_CTRL,
	       TACNA_DFC1_CH1_DITH_EN_SHIFT, 1, 0,
	       snd_soc_get_volsw, tacna_dfc_dith_put),
SOC_SINGLE_EXT("DFC2 Dither", TACNA_DFC1_CH2_CTRL,
	       TACNA_DFC1_CH2_DITH_EN_SHIFT, 1, 0,
	       snd_soc_get_volsw, tacna_dfc_dith_put),
SOC_SINGLE_EXT("DFC3 Dither", TACNA_DFC1_CH3_CTRL,
	       TACNA_DFC1_CH3_DITH_EN_SHIFT, 1, 0,
	       snd_soc_get_volsw, tacna_dfc_dith_put),
SOC_SINGLE_EXT("DFC4 Dither", TACNA_DFC1_CH4_CTRL,
	       TACNA_DFC1_CH4_DITH_EN_SHIFT, 1, 0,
	       snd_soc_get_volsw, tacna_dfc_dith_put),
SOC_SINGLE_EXT("DFC5 Dither", TACNA_DFC1_CH5_CTRL,
	       TACNA_DFC1_CH5_DITH_EN_SHIFT, 1, 0,
	       snd_soc_get_volsw, tacna_dfc_dith_put),
SOC_SINGLE_EXT("DFC6 Dither", TACNA_DFC1_CH6_CTRL,
	       TACNA_DFC1_CH6_DITH_EN_SHIFT, 1, 0,
	       snd_soc_get_volsw, tacna_dfc_dith_put),
SOC_SINGLE_EXT("DFC7 Dither", TACNA_DFC1_CH7_CTRL,
	       TACNA_DFC1_CH7_DITH_EN_SHIFT, 1, 0,
	       snd_soc_get_volsw, tacna_dfc_dith_put),
SOC_SINGLE_EXT("DFC8 Dither", TACNA_DFC1_CH8_CTRL,
	       TACNA_DFC1_CH8_DITH_EN_SHIFT, 1, 0,
	       snd_soc_get_volsw, tacna_dfc_dith_put),

SOC_ENUM_EXT("DFC1RX Width", tacna_dfc_width[0],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC1RX Type", tacna_dfc_type[0],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC1TX Width", tacna_dfc_width[1],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC1TX Type", tacna_dfc_type[1],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC2RX Width", tacna_dfc_width[2],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC2RX Type", tacna_dfc_type[2],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC2TX Width", tacna_dfc_width[3],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC2TX Type", tacna_dfc_type[3],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC3RX Width", tacna_dfc_width[4],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC3RX Type", tacna_dfc_type[4],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC3TX Width", tacna_dfc_width[5],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC3TX Type", tacna_dfc_type[5],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC4RX Width", tacna_dfc_width[6],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC4RX Type", tacna_dfc_type[6],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC4TX Width", tacna_dfc_width[7],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC4TX Type", tacna_dfc_type[7],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC5RX Width", tacna_dfc_width[8],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC5RX Type", tacna_dfc_type[8],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC5TX Width", tacna_dfc_width[9],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC5TX Type", tacna_dfc_type[9],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC6RX Width", tacna_dfc_width[10],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC6RX Type", tacna_dfc_type[10],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC6TX Width", tacna_dfc_width[11],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC6TX Type", tacna_dfc_type[11],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC7RX Width", tacna_dfc_width[12],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC7RX Type", tacna_dfc_type[12],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC7TX Width", tacna_dfc_width[13],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC7TX Type", tacna_dfc_type[13],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC8RX Width", tacna_dfc_width[14],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC8RX Type", tacna_dfc_type[14],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC8TX Width", tacna_dfc_width[15],
	     snd_soc_get_enum_double, tacna_dfc_put),
SOC_ENUM_EXT("DFC8TX Type", tacna_dfc_type[15],
	     snd_soc_get_enum_double, tacna_dfc_put),

SOC_SINGLE_TLV("Noise Generator Volume", TACNA_COMFORT_NOISE_GENERATOR,
	       TACNA_NOISE_GEN_GAIN_SHIFT, 0x12, 0, tacna_noise_tlv),

TACNA_MIXER_CONTROLS("ASP1TX1", TACNA_ASP1TX1_INPUT1),
TACNA_MIXER_CONTROLS("ASP1TX2", TACNA_ASP1TX2_INPUT1),
TACNA_MIXER_CONTROLS("ASP1TX3", TACNA_ASP1TX3_INPUT1),
TACNA_MIXER_CONTROLS("ASP1TX4", TACNA_ASP1TX4_INPUT1),
TACNA_MIXER_CONTROLS("ASP1TX5", TACNA_ASP1TX5_INPUT1),
TACNA_MIXER_CONTROLS("ASP1TX6", TACNA_ASP1TX6_INPUT1),
TACNA_MIXER_CONTROLS("ASP1TX7", TACNA_ASP1TX7_INPUT1),
TACNA_MIXER_CONTROLS("ASP1TX8", TACNA_ASP1TX8_INPUT1),

TACNA_MIXER_CONTROLS("ASP2TX1", TACNA_ASP2TX1_INPUT1),
TACNA_MIXER_CONTROLS("ASP2TX2", TACNA_ASP2TX2_INPUT1),
TACNA_MIXER_CONTROLS("ASP2TX3", TACNA_ASP2TX3_INPUT1),
TACNA_MIXER_CONTROLS("ASP2TX4", TACNA_ASP2TX4_INPUT1),

TACNA_MIXER_CONTROLS("ASP3TX1", TACNA_ASP3TX1_INPUT1),
TACNA_MIXER_CONTROLS("ASP3TX2", TACNA_ASP3TX2_INPUT1),
TACNA_MIXER_CONTROLS("ASP3TX3", TACNA_ASP3TX3_INPUT1),
TACNA_MIXER_CONTROLS("ASP3TX4", TACNA_ASP3TX4_INPUT1),

TACNA_MIXER_CONTROLS("SLIMTX1", TACNA_SLIMTX1_INPUT1),
TACNA_MIXER_CONTROLS("SLIMTX2", TACNA_SLIMTX2_INPUT1),
TACNA_MIXER_CONTROLS("SLIMTX3", TACNA_SLIMTX3_INPUT1),
TACNA_MIXER_CONTROLS("SLIMTX4", TACNA_SLIMTX4_INPUT1),
TACNA_MIXER_CONTROLS("SLIMTX5", TACNA_SLIMTX5_INPUT1),
TACNA_MIXER_CONTROLS("SLIMTX6", TACNA_SLIMTX6_INPUT1),
TACNA_MIXER_CONTROLS("SLIMTX7", TACNA_SLIMTX7_INPUT1),
TACNA_MIXER_CONTROLS("SLIMTX8", TACNA_SLIMTX8_INPUT1),

WM_ADSP2_PRELOAD_SWITCH("DSP1", 1),

TACNA_MIXER_CONTROLS("DSP1RX1", TACNA_DSP1RX1_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX2", TACNA_DSP1RX2_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX3", TACNA_DSP1RX3_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX4", TACNA_DSP1RX4_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX5", TACNA_DSP1RX5_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX6", TACNA_DSP1RX6_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX7", TACNA_DSP1RX7_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX8", TACNA_DSP1RX8_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX9", TACNA_DSP1RX9_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX10", TACNA_DSP1RX10_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX11", TACNA_DSP1RX11_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX12", TACNA_DSP1RX12_INPUT1),

WM_ADSP_FW_CONTROL("DSP1", 0),
};

TACNA_MIXER_ENUMS(EQ1, TACNA_EQ1_INPUT1);
TACNA_MIXER_ENUMS(EQ2, TACNA_EQ2_INPUT1);
TACNA_MIXER_ENUMS(EQ3, TACNA_EQ3_INPUT1);
TACNA_MIXER_ENUMS(EQ4, TACNA_EQ4_INPUT1);

TACNA_MIXER_ENUMS(DRC1L, TACNA_DRC1L_INPUT1);
TACNA_MIXER_ENUMS(DRC1R, TACNA_DRC1R_INPUT1);
TACNA_MIXER_ENUMS(DRC2L, TACNA_DRC2L_INPUT1);
TACNA_MIXER_ENUMS(DRC2R, TACNA_DRC2R_INPUT1);

TACNA_MIXER_ENUMS(LHPF1, TACNA_LHPF1_INPUT1);
TACNA_MIXER_ENUMS(LHPF2, TACNA_LHPF2_INPUT1);
TACNA_MIXER_ENUMS(LHPF3, TACNA_LHPF3_INPUT1);
TACNA_MIXER_ENUMS(LHPF4, TACNA_LHPF4_INPUT1);

TACNA_MIXER_ENUMS(OUT1L, TACNA_OUT1L_INPUT1);
TACNA_MIXER_ENUMS(OUT1R, TACNA_OUT1R_INPUT1);
TACNA_MIXER_ENUMS(OUT5L, TACNA_OUT5L_INPUT1);
TACNA_MIXER_ENUMS(OUT5R, TACNA_OUT5R_INPUT1);
TACNA_MIXER_ENUMS(OUTAUX1L, TACNA_OUTAUX1L_INPUT1);
TACNA_MIXER_ENUMS(OUTAUX1R, TACNA_OUTAUX1R_INPUT1);
TACNA_MIXER_ENUMS(OUTHL, TACNA_OUTHL_INPUT1);
TACNA_MIXER_ENUMS(OUTHR, TACNA_OUTHR_INPUT1);

TACNA_MIXER_ENUMS(ASP1TX1, TACNA_ASP1TX1_INPUT1);
TACNA_MIXER_ENUMS(ASP1TX2, TACNA_ASP1TX2_INPUT1);
TACNA_MIXER_ENUMS(ASP1TX3, TACNA_ASP1TX3_INPUT1);
TACNA_MIXER_ENUMS(ASP1TX4, TACNA_ASP1TX4_INPUT1);
TACNA_MIXER_ENUMS(ASP1TX5, TACNA_ASP1TX5_INPUT1);
TACNA_MIXER_ENUMS(ASP1TX6, TACNA_ASP1TX6_INPUT1);
TACNA_MIXER_ENUMS(ASP1TX7, TACNA_ASP1TX7_INPUT1);
TACNA_MIXER_ENUMS(ASP1TX8, TACNA_ASP1TX8_INPUT1);

TACNA_MIXER_ENUMS(ASP2TX1, TACNA_ASP2TX1_INPUT1);
TACNA_MIXER_ENUMS(ASP2TX2, TACNA_ASP2TX2_INPUT1);
TACNA_MIXER_ENUMS(ASP2TX3, TACNA_ASP2TX3_INPUT1);
TACNA_MIXER_ENUMS(ASP2TX4, TACNA_ASP2TX4_INPUT1);

TACNA_MIXER_ENUMS(ASP3TX1, TACNA_ASP3TX1_INPUT1);
TACNA_MIXER_ENUMS(ASP3TX2, TACNA_ASP3TX2_INPUT1);
TACNA_MIXER_ENUMS(ASP3TX3, TACNA_ASP3TX3_INPUT1);
TACNA_MIXER_ENUMS(ASP3TX4, TACNA_ASP3TX4_INPUT1);

TACNA_MIXER_ENUMS(SLIMTX1, TACNA_SLIMTX1_INPUT1);
TACNA_MIXER_ENUMS(SLIMTX2, TACNA_SLIMTX2_INPUT1);
TACNA_MIXER_ENUMS(SLIMTX3, TACNA_SLIMTX3_INPUT1);
TACNA_MIXER_ENUMS(SLIMTX4, TACNA_SLIMTX4_INPUT1);
TACNA_MIXER_ENUMS(SLIMTX5, TACNA_SLIMTX5_INPUT1);
TACNA_MIXER_ENUMS(SLIMTX6, TACNA_SLIMTX6_INPUT1);
TACNA_MIXER_ENUMS(SLIMTX7, TACNA_SLIMTX7_INPUT1);
TACNA_MIXER_ENUMS(SLIMTX8, TACNA_SLIMTX8_INPUT1);

TACNA_MUX_ENUMS(ASRC1IN1L, TACNA_ASRC1_IN1L_INPUT1);
TACNA_MUX_ENUMS(ASRC1IN1R, TACNA_ASRC1_IN1R_INPUT1);
TACNA_MUX_ENUMS(ASRC1IN2L, TACNA_ASRC1_IN2L_INPUT1);
TACNA_MUX_ENUMS(ASRC1IN2R, TACNA_ASRC1_IN2R_INPUT1);

TACNA_MUX_ENUMS(ASRC2IN1L, TACNA_ASRC2_IN1L_INPUT1);
TACNA_MUX_ENUMS(ASRC2IN1R, TACNA_ASRC2_IN1R_INPUT1);
TACNA_MUX_ENUMS(ASRC2IN2L, TACNA_ASRC2_IN2L_INPUT1);
TACNA_MUX_ENUMS(ASRC2IN2R, TACNA_ASRC2_IN2R_INPUT1);

TACNA_MUX_ENUMS(ISRC1INT1, TACNA_ISRC1INT1_INPUT1);
TACNA_MUX_ENUMS(ISRC1INT2, TACNA_ISRC1INT2_INPUT1);

TACNA_MUX_ENUMS(ISRC1DEC1, TACNA_ISRC1DEC1_INPUT1);
TACNA_MUX_ENUMS(ISRC1DEC2, TACNA_ISRC1DEC2_INPUT1);

TACNA_MUX_ENUMS(ISRC2INT1, TACNA_ISRC2INT1_INPUT1);
TACNA_MUX_ENUMS(ISRC2INT2, TACNA_ISRC2INT2_INPUT1);

TACNA_MUX_ENUMS(ISRC2DEC1, TACNA_ISRC2DEC1_INPUT1);
TACNA_MUX_ENUMS(ISRC2DEC2, TACNA_ISRC2DEC2_INPUT1);

TACNA_MIXER_ENUMS(AOBRIDGE2IN1, TACNA_AOBRIDGE2_IN1_INPUT1);
TACNA_MIXER_ENUMS(AOBRIDGE2IN2, TACNA_AOBRIDGE2_IN2_INPUT1);
TACNA_MIXER_ENUMS(AOBRIDGE2IN3, TACNA_AOBRIDGE2_IN3_INPUT1);
TACNA_MIXER_ENUMS(AOBRIDGE2IN4, TACNA_AOBRIDGE2_IN4_INPUT1);
TACNA_MIXER_ENUMS(AOBRIDGE2IN5, TACNA_AOBRIDGE2_IN5_INPUT1);
TACNA_MIXER_ENUMS(AOBRIDGE2IN6, TACNA_AOBRIDGE2_IN6_INPUT1);
TACNA_MIXER_ENUMS(AOBRIDGE2IN7, TACNA_AOBRIDGE2_IN7_INPUT1);
TACNA_MIXER_ENUMS(AOBRIDGE2IN8, TACNA_AOBRIDGE2_IN8_INPUT1);

TACNA_MUX_ENUMS(DFC1, TACNA_DFC1_CH1_INPUT1);
TACNA_MUX_ENUMS(DFC2, TACNA_DFC1_CH2_INPUT1);
TACNA_MUX_ENUMS(DFC3, TACNA_DFC1_CH3_INPUT1);
TACNA_MUX_ENUMS(DFC4, TACNA_DFC1_CH4_INPUT1);
TACNA_MUX_ENUMS(DFC5, TACNA_DFC1_CH5_INPUT1);
TACNA_MUX_ENUMS(DFC6, TACNA_DFC1_CH6_INPUT1);
TACNA_MUX_ENUMS(DFC7, TACNA_DFC1_CH7_INPUT1);
TACNA_MUX_ENUMS(DFC8, TACNA_DFC1_CH8_INPUT1);

TACNA_MIXER_ENUMS(DSP1RX1, TACNA_DSP1RX1_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX2, TACNA_DSP1RX2_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX3, TACNA_DSP1RX3_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX4, TACNA_DSP1RX4_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX5, TACNA_DSP1RX5_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX6, TACNA_DSP1RX6_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX7, TACNA_DSP1RX7_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX8, TACNA_DSP1RX8_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX9, TACNA_DSP1RX9_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX10, TACNA_DSP1RX10_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX11, TACNA_DSP1RX11_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX12, TACNA_DSP1RX12_INPUT1);

static const char * const cs47l96_aec_loopback_texts[] = {
	"OUT1L", "OUT1R", "OUT5L", "OUT5R",
};

static const unsigned int cs47l96_aec_loopback_values[] = {
	0, 1, 8, 9,
};

static const struct soc_enum cs47l96_aec_loopback[] = {
	SOC_VALUE_ENUM_SINGLE(TACNA_OUTPUT_AEC_CONTROL_1,
			      TACNA_AEC_LOOPBACK1_SRC_SHIFT,
			      TACNA_AEC_LOOPBACK1_SRC_MASK >>
			      TACNA_AEC_LOOPBACK1_SRC_SHIFT,
			      ARRAY_SIZE(cs47l96_aec_loopback_texts),
			      cs47l96_aec_loopback_texts,
			      cs47l96_aec_loopback_values),
	SOC_VALUE_ENUM_SINGLE(TACNA_OUTPUT_AEC_CONTROL_1,
			      TACNA_AEC_LOOPBACK2_SRC_SHIFT,
			      TACNA_AEC_LOOPBACK2_SRC_MASK >>
			      TACNA_AEC_LOOPBACK2_SRC_SHIFT,
			      ARRAY_SIZE(cs47l96_aec_loopback_texts),
			      cs47l96_aec_loopback_texts,
			      cs47l96_aec_loopback_values),
};

static const struct snd_kcontrol_new cs47l96_aec_loopback_mux[] = {
	SOC_DAPM_ENUM("AEC1 Loopback", cs47l96_aec_loopback[0]),
	SOC_DAPM_ENUM("AEC2 Loopback", cs47l96_aec_loopback[1]),
};

static const struct snd_kcontrol_new cs47l96_anc_input_mux[] = {
	SOC_DAPM_ENUM("ANCL Input", tacna_mono_anc_input_src[0]),
	SOC_DAPM_ENUM("ANCL Channel", tacna_mono_anc_input_src[1]),
};

static const struct snd_kcontrol_new cs47l96_anc_ng_mux =
	SOC_DAPM_ENUM("ANC NG Source", tacna_anc_ng_enum);

static const struct snd_kcontrol_new cs47l96_output_anc_src[] = {
	SOC_DAPM_ENUM("OUT1L ANC Source", tacna_output_anc_src[0]),
	SOC_DAPM_ENUM("OUT1R ANC Source", tacna_output_anc_src[1]),
	SOC_DAPM_ENUM("OUT5L ANC Source", tacna_output_anc_src[4]),
	SOC_DAPM_ENUM("OUT5R ANC Source", tacna_output_anc_src[5]),
};

static const struct snd_kcontrol_new cs47l96_out1_aux_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
};

static const struct snd_kcontrol_new cs47l96_out1_dsd_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
};

static const char * const cs47l96_out_select_texts[] = {
	"OUT1", "OUTH",
};

static SOC_ENUM_SINGLE_DECL(cs47l96_output_select_enum, SND_SOC_NOPM, 0,
			    cs47l96_out_select_texts);

static const struct snd_kcontrol_new cs47l96_output_select =
	SOC_DAPM_ENUM("Output Select", cs47l96_output_select_enum);

static const struct snd_kcontrol_new cs47l96_outh_aux_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
};

static const char * const cs47l96_dsd_source_texts[] = {
	"DSD", "DoP",
};

static const unsigned int cs47l96_dsd_source_values[] = {
	0x0, 0x2,
};

static const struct soc_enum cs47l96_dsd_source_enum =
	SOC_VALUE_ENUM_SINGLE(TACNA_DSD1_CONTROL1,
			      TACNA_DSD1_SRC_SHIFT,
			      TACNA_DSD1_SRC_MASK >> TACNA_DSD1_SRC_SHIFT,
			      ARRAY_SIZE(cs47l96_dsd_source_texts),
			      cs47l96_dsd_source_texts,
			      cs47l96_dsd_source_values);

static const struct snd_kcontrol_new cs47l96_dsd_source_select =
	SOC_DAPM_ENUM("Source", cs47l96_dsd_source_enum);

static const struct snd_kcontrol_new cs47l96_dsd_switch =
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_soc_dapm_widget cs47l96_dapm_widgets[] = {
SND_SOC_DAPM_SUPPLY("SYSCLK", TACNA_SYSTEM_CLOCK1, TACNA_SYSCLK_EN_SHIFT,
		    0, tacna_sysclk_ev,
		    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_SUPPLY("ASYNCCLK", SND_SOC_NOPM, TACNA_ASYNCCLK_REQ,
		    0, tacna_asyncclk_ev,
		    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_SUPPLY("DAC_ASYNCCLK", SND_SOC_NOPM, TACNA_DACRATE1_ASYNCCLK_REQ,
		    0, tacna_asyncclk_ev,
		    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_SUPPLY("OUTH_ASYNCCLK", SND_SOC_NOPM, TACNA_OUTH_ASYNCCLK_REQ,
		    0, tacna_asyncclk_ev,
		    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_SUPPLY("OPCLK", TACNA_OUTPUT_SYS_CLK, TACNA_OPCLK_EN_SHIFT,
		    0, NULL, 0),
SND_SOC_DAPM_SUPPLY("ASYNCOPCLK", TACNA_OUTPUT_ASYNC_CLK,
		    TACNA_OPCLK_ASYNC_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_SUPPLY("DACCLK", TACNA_DAC_CLK_CONTROL2,
		    TACNA_DAC_CLK_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_SUPPLY("DSPCLK", TACNA_DSP_CLOCK1, TACNA_DSP_CLK_EN_SHIFT,
		    0, NULL, 0),

SND_SOC_DAPM_REGULATOR_SUPPLY("VDD1_CP", 20, 0),
SND_SOC_DAPM_REGULATOR_SUPPLY("VDD2_CP", 20, 0),
SND_SOC_DAPM_REGULATOR_SUPPLY("VDD3_CP", 20, 0),
SND_SOC_DAPM_REGULATOR_SUPPLY("VOUT_MIC", 0, SND_SOC_DAPM_REGULATOR_BYPASS),

SND_SOC_DAPM_REGULATOR_SUPPLY("VDD_IO2", 0, 0),

SND_SOC_DAPM_SUPPLY("MICBIAS1", TACNA_MICBIAS_CTRL1, TACNA_MICB1_EN_SHIFT,
		    0, NULL, 0),
SND_SOC_DAPM_SUPPLY("MICBIAS2", TACNA_MICBIAS_CTRL2, TACNA_MICB2_EN_SHIFT,
		    0, NULL, 0),

SND_SOC_DAPM_SUPPLY("MICBIAS1A", TACNA_MICBIAS_CTRL5, TACNA_MICB1A_EN_SHIFT,
		    0, NULL, 0),
SND_SOC_DAPM_SUPPLY("MICBIAS1B", TACNA_MICBIAS_CTRL5, TACNA_MICB1B_EN_SHIFT,
		    0, NULL, 0),
SND_SOC_DAPM_SUPPLY("MICBIAS1C", TACNA_MICBIAS_CTRL5, TACNA_MICB1C_EN_SHIFT,
		    0, NULL, 0),
SND_SOC_DAPM_SUPPLY("MICBIAS1D", TACNA_MICBIAS_CTRL5, TACNA_MICB1D_EN_SHIFT,
		    0, NULL, 0),

SND_SOC_DAPM_SUPPLY("MICBIAS2A", TACNA_MICBIAS_CTRL6, TACNA_MICB2A_EN_SHIFT,
		    0, NULL, 0),
SND_SOC_DAPM_SUPPLY("MICBIAS2B", TACNA_MICBIAS_CTRL6, TACNA_MICB2B_EN_SHIFT,
		    0, NULL, 0),

TACNA_DSP_FREQ_WIDGET("DSP1", 0),

SND_SOC_DAPM_SIGGEN("TONE"),
SND_SOC_DAPM_SIGGEN("NOISE"),

SND_SOC_DAPM_SUPPLY("MICD_COMP_FRC", TACNA_MICDET_COMP_CTRL,
		    TACNA_MICD_COMP_FRC_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_INPUT("IN1LN_1"),
SND_SOC_DAPM_INPUT("IN1LN_2"),
SND_SOC_DAPM_INPUT("IN1LP_1"),
SND_SOC_DAPM_INPUT("IN1LP_2"),
SND_SOC_DAPM_INPUT("IN1RN_1"),
SND_SOC_DAPM_INPUT("IN1RN_2"),
SND_SOC_DAPM_INPUT("IN1RP_1"),
SND_SOC_DAPM_INPUT("IN1RP_2"),
SND_SOC_DAPM_INPUT("IN1_PDMCLK"),
SND_SOC_DAPM_INPUT("IN1_PDMDATA"),

SND_SOC_DAPM_INPUT("IN2LN_1"),
SND_SOC_DAPM_INPUT("IN2LN_2"),
SND_SOC_DAPM_INPUT("IN2LP_1"),
SND_SOC_DAPM_INPUT("IN2LP_2"),
SND_SOC_DAPM_INPUT("IN2RN_1"),
SND_SOC_DAPM_INPUT("IN2RN_2"),
SND_SOC_DAPM_INPUT("IN2RP_1"),
SND_SOC_DAPM_INPUT("IN2RP_2"),
SND_SOC_DAPM_INPUT("IN2_PDMCLK"),
SND_SOC_DAPM_INPUT("IN2_PDMDATA"),

SND_SOC_DAPM_INPUT("IN3_PDMCLK"),
SND_SOC_DAPM_INPUT("IN3_PDMDATA"),

SND_SOC_DAPM_INPUT("IN4_PDMCLK"),
SND_SOC_DAPM_INPUT("IN4_PDMDATA"),

SND_SOC_DAPM_OUTPUT("DRC1 Signal Activity"),
SND_SOC_DAPM_OUTPUT("DRC2 Signal Activity"),

SND_SOC_DAPM_OUTPUT("DSP Trigger Out"),

SND_SOC_DAPM_MUX("IN1L Mux", SND_SOC_NOPM, 0, 0, &tacna_inmux[0]),
SND_SOC_DAPM_MUX("IN1R Mux", SND_SOC_NOPM, 0, 0, &tacna_inmux[1]),
SND_SOC_DAPM_MUX("IN2L Mux", SND_SOC_NOPM, 0, 0, &tacna_inmux[2]),
SND_SOC_DAPM_MUX("IN2R Mux", SND_SOC_NOPM, 0, 0, &tacna_inmux[3]),

SND_SOC_DAPM_MUX("IN1L Mode", SND_SOC_NOPM, 0, 0, &tacna_dmode_mux[0]),
SND_SOC_DAPM_MUX("IN1R Mode", SND_SOC_NOPM, 0, 0, &tacna_dmode_mux[0]),
SND_SOC_DAPM_MUX("IN2L Mode", SND_SOC_NOPM, 0, 0, &tacna_dmode_mux[1]),
SND_SOC_DAPM_MUX("IN2R Mode", SND_SOC_NOPM, 0, 0, &tacna_dmode_mux[1]),

SND_SOC_DAPM_MUX("IN1L Swap Chan", SND_SOC_NOPM, 0, 0,
		 &tacna_in_swap_chan[0]),
SND_SOC_DAPM_MUX("IN1R Swap Chan", SND_SOC_NOPM, 0, 0,
		 &tacna_in_swap_chan[0]),

SND_SOC_DAPM_MUX("IN2L Swap Chan", SND_SOC_NOPM, 0, 0,
		 &tacna_in_swap_chan[1]),
SND_SOC_DAPM_MUX("IN2R Swap Chan", SND_SOC_NOPM, 0, 0,
		 &tacna_in_swap_chan[1]),

SND_SOC_DAPM_AIF_OUT("ASP1TX1", NULL, 0, TACNA_ASP1_ENABLES1,
		     TACNA_ASP1_TX1_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP1TX2", NULL, 0, TACNA_ASP1_ENABLES1,
		     TACNA_ASP1_TX2_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP1TX3", NULL, 0, TACNA_ASP1_ENABLES1,
		     TACNA_ASP1_TX3_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP1TX4", NULL, 0, TACNA_ASP1_ENABLES1,
		     TACNA_ASP1_TX4_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP1TX5", NULL, 0, TACNA_ASP1_ENABLES1,
		     TACNA_ASP1_TX5_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP1TX6", NULL, 0, TACNA_ASP1_ENABLES1,
		     TACNA_ASP1_TX6_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP1TX7", NULL, 0, TACNA_ASP1_ENABLES1,
		     TACNA_ASP1_TX7_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP1TX8", NULL, 0, TACNA_ASP1_ENABLES1,
		     TACNA_ASP1_TX8_EN_SHIFT, 0),

SND_SOC_DAPM_AIF_OUT("ASP2TX1", NULL, 0, TACNA_ASP2_ENABLES1,
		     TACNA_ASP2_TX1_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP2TX2", NULL, 0, TACNA_ASP2_ENABLES1,
		     TACNA_ASP2_TX2_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP2TX3", NULL, 0, TACNA_ASP2_ENABLES1,
		     TACNA_ASP2_TX3_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP2TX4", NULL, 0, TACNA_ASP2_ENABLES1,
		     TACNA_ASP2_TX4_EN_SHIFT, 0),

SND_SOC_DAPM_AIF_OUT("ASP3TX1", NULL, 0, TACNA_ASP3_ENABLES1,
		     TACNA_ASP3_TX1_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP3TX2", NULL, 0, TACNA_ASP3_ENABLES1,
		     TACNA_ASP3_TX2_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP3TX3", NULL, 0, TACNA_ASP3_ENABLES1,
		     TACNA_ASP3_TX3_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP3TX4", NULL, 0, TACNA_ASP3_ENABLES1,
		     TACNA_ASP3_TX4_EN_SHIFT, 0),

SND_SOC_DAPM_AIF_OUT("SLIMTX1", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		     TACNA_SLIMTX1_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("SLIMTX2", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		     TACNA_SLIMTX2_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("SLIMTX3", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		     TACNA_SLIMTX3_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("SLIMTX4", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		     TACNA_SLIMTX4_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("SLIMTX5", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		     TACNA_SLIMTX5_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("SLIMTX6", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		     TACNA_SLIMTX6_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("SLIMTX7", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		     TACNA_SLIMTX7_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("SLIMTX8", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		     TACNA_SLIMTX8_EN_SHIFT, 0),

SND_SOC_DAPM_PGA_E("OUT1L PGA", SND_SOC_NOPM,
		   TACNA_OUT1L_EN_SHIFT, 0, NULL, 0, cs47l96_out_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("OUT1R PGA", SND_SOC_NOPM,
		   TACNA_OUT1R_EN_SHIFT, 0, NULL, 0, cs47l96_out_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA("OUT5L PGA", SND_SOC_NOPM, 0, 0, NULL, 0),
SND_SOC_DAPM_PGA("OUT5R PGA", SND_SOC_NOPM, 0, 0, NULL, 0),
SND_SOC_DAPM_PGA("OUTAUX1L PGA", TACNA_OUTAUX1L_ENABLE_1,
		 TACNA_OUTAUX1L_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("OUTAUX1R PGA", TACNA_OUTAUX1R_ENABLE_1,
		 TACNA_OUTAUX1R_EN_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_SWITCH("OUT1L AUX Mix", TACNA_OUT1L_CONTROL_1,
		    TACNA_OUT1L_AUX_SRC_SHIFT, 0, &cs47l96_out1_aux_switch[0]),
SND_SOC_DAPM_SWITCH("OUT1R AUX Mix", TACNA_OUT1R_CONTROL_1,
		    TACNA_OUT1R_AUX_SRC_SHIFT, 0, &cs47l96_out1_aux_switch[1]),

SND_SOC_DAPM_SWITCH("OUT1L DSD Mix", TACNA_OUT1L_CONTROL_1,
		      TACNA_OUT1L_DSD_EN_SHIFT, 0,
		      &cs47l96_out1_dsd_switch[0]),
SND_SOC_DAPM_SWITCH("OUT1R DSD Mix", TACNA_OUT1R_CONTROL_1,
		      TACNA_OUT1R_DSD_EN_SHIFT, 0,
		      &cs47l96_out1_dsd_switch[1]),

SND_SOC_DAPM_DEMUX("OUT1 Demux", SND_SOC_NOPM, 0, 0, &cs47l96_out1_demux),

SND_SOC_DAPM_MUX("OUT1L Output Select", SND_SOC_NOPM, 0, 0,
		 &cs47l96_output_select),
SND_SOC_DAPM_MUX("OUT1R Output Select", SND_SOC_NOPM, 0, 0,
		 &cs47l96_output_select),

SND_SOC_DAPM_MUX("OUT5L Source", TACNA_OUTPUT_ENABLE_1, TACNA_OUT5L_EN_SHIFT,
		 0, &cs47l96_out5_mux),
SND_SOC_DAPM_MUX("OUT5R Source", TACNA_OUTPUT_ENABLE_1, TACNA_OUT5R_EN_SHIFT,
		 0, &cs47l96_out5_mux),

SND_SOC_DAPM_MUX_E("OUTH Output Select", SND_SOC_NOPM, 0, 0,
		   &cs47l96_output_select, cs47l96_outh_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
		   SND_SOC_DAPM_POST_PMU),

SND_SOC_DAPM_SWITCH("OUTHL AUX Mix", TACNA_OUTH_AUX_MIX_CONTROL_1,
		    TACNA_OUTHL_AUX_SRC_SHIFT, 0, &cs47l96_outh_aux_switch[0]),
SND_SOC_DAPM_SWITCH("OUTHR AUX Mix", TACNA_OUTH_AUX_MIX_CONTROL_1,
		    TACNA_OUTHR_AUX_SRC_SHIFT, 0, &cs47l96_outh_aux_switch[1]),

SND_SOC_DAPM_PGA("OUTH PCM", TACNA_OUTH_ENABLE_1, TACNA_OUTH_PCM_EN_SHIFT,
		 0, NULL, 0),

SND_SOC_DAPM_MUX("DSD Processor Source", SND_SOC_NOPM, 0, 0,
		 &cs47l96_dsd_source_select),

SND_SOC_DAPM_SWITCH_E("DSD Processor", TACNA_DSD1_CONTROL1,
		      TACNA_DSD1_EN_SHIFT, 0,
		      &cs47l96_dsd_switch, cs47l96_dsd_processor_ev,
		      SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

SND_SOC_DAPM_SWITCH("AUXPDM1 Output", TACNA_AUXPDM_CONTROL1,
		    TACNA_AUXPDM1_EN_SHIFT, 0, &tacna_auxpdm_switch[0]),
SND_SOC_DAPM_SWITCH("AUXPDM2 Output", TACNA_AUXPDM_CONTROL1,
		    TACNA_AUXPDM2_EN_SHIFT, 0, &tacna_auxpdm_switch[1]),
SND_SOC_DAPM_SWITCH("AUXPDM3 Output", TACNA_AUXPDM_CONTROL1,
		    TACNA_AUXPDM3_EN_SHIFT, 0, &tacna_auxpdm_switch[2]),

SND_SOC_DAPM_MUX("AUXPDM1 Input", SND_SOC_NOPM, 0, 0,
		 &tacna_auxpdm_inmux[0]),
SND_SOC_DAPM_MUX("AUXPDM2 Input", SND_SOC_NOPM, 0, 0,
		 &tacna_auxpdm_inmux[1]),
SND_SOC_DAPM_MUX("AUXPDM3 Input", SND_SOC_NOPM, 0, 0,
		 &tacna_auxpdm_inmux[2]),

/*
 * mux_in widgets : arranged in the order of sources
 * specified in TACNA_MIXER_INPUT_ROUTES
 */

SND_SOC_DAPM_PGA("Tone Generator 1", TACNA_TONE_GENERATOR1,
		 TACNA_TONE1_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("Tone Generator 2", TACNA_TONE_GENERATOR1,
		 TACNA_TONE2_EN_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_MIC("HAPTICS", NULL),

SND_SOC_DAPM_MUX("AEC1 Loopback", TACNA_OUTPUT_AEC_ENABLE_1,
		 TACNA_AEC_LOOPBACK1_EN_SHIFT, 0,
		 &cs47l96_aec_loopback_mux[0]),
SND_SOC_DAPM_MUX("AEC2 Loopback", TACNA_OUTPUT_AEC_ENABLE_1,
		 TACNA_AEC_LOOPBACK2_EN_SHIFT, 0,
		 &cs47l96_aec_loopback_mux[1]),

SND_SOC_DAPM_PGA("Noise Generator", TACNA_COMFORT_NOISE_GENERATOR,
		 TACNA_NOISE_GEN_EN_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_PGA_E("IN1L PGA", TACNA_INPUT_CONTROL, TACNA_IN1L_EN_SHIFT,
		   0, NULL, 0, tacna_in_ev,
		   SND_SOC_DAPM_PRE_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("IN1R PGA", TACNA_INPUT_CONTROL, TACNA_IN1R_EN_SHIFT,
		   0, NULL, 0, tacna_in_ev,
		   SND_SOC_DAPM_PRE_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("IN2L PGA", TACNA_INPUT_CONTROL, TACNA_IN2L_EN_SHIFT,
		   0, NULL, 0, tacna_in_ev,
		   SND_SOC_DAPM_PRE_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("IN2R PGA", TACNA_INPUT_CONTROL, TACNA_IN2R_EN_SHIFT,
		   0, NULL, 0, tacna_in_ev,
		   SND_SOC_DAPM_PRE_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("IN3L PGA", TACNA_INPUT_CONTROL, TACNA_IN3L_EN_SHIFT,
		   0, NULL, 0, tacna_in_ev,
		   SND_SOC_DAPM_PRE_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("IN3R PGA", TACNA_INPUT_CONTROL, TACNA_IN3R_EN_SHIFT,
		   0, NULL, 0, tacna_in_ev,
		   SND_SOC_DAPM_PRE_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("IN4L PGA", TACNA_INPUT_CONTROL, TACNA_IN4L_EN_SHIFT,
		   0, NULL, 0, tacna_in_ev,
		   SND_SOC_DAPM_PRE_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("IN4R PGA", TACNA_INPUT_CONTROL, TACNA_IN4R_EN_SHIFT,
		   0, NULL, 0, tacna_in_ev,
		   SND_SOC_DAPM_PRE_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),

SND_SOC_DAPM_AIF_IN("ASP1RX1", NULL, 0, TACNA_ASP1_ENABLES1,
		    TACNA_ASP1_RX1_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP1RX2", NULL, 0, TACNA_ASP1_ENABLES1,
		    TACNA_ASP1_RX2_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP1RX3", NULL, 0, TACNA_ASP1_ENABLES1,
		    TACNA_ASP1_RX3_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP1RX4", NULL, 0, TACNA_ASP1_ENABLES1,
		    TACNA_ASP1_RX4_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP1RX5", NULL, 0, TACNA_ASP1_ENABLES1,
		    TACNA_ASP1_RX5_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP1RX6", NULL, 0, TACNA_ASP1_ENABLES1,
		    TACNA_ASP1_RX6_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP1RX7", NULL, 0, TACNA_ASP1_ENABLES1,
		    TACNA_ASP1_RX7_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP1RX8", NULL, 0, TACNA_ASP1_ENABLES1,
		    TACNA_ASP1_RX8_EN_SHIFT, 0),

SND_SOC_DAPM_AIF_IN("ASP2RX1", NULL, 0, TACNA_ASP2_ENABLES1,
		    TACNA_ASP2_RX1_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP2RX2", NULL, 0, TACNA_ASP2_ENABLES1,
		    TACNA_ASP2_RX2_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP2RX3", NULL, 0, TACNA_ASP2_ENABLES1,
		    TACNA_ASP2_RX3_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP2RX4", NULL, 0, TACNA_ASP2_ENABLES1,
		    TACNA_ASP2_RX4_EN_SHIFT, 0),

SND_SOC_DAPM_AIF_IN("ASP3RX1", NULL, 0, TACNA_ASP3_ENABLES1,
		    TACNA_ASP3_RX1_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP3RX2", NULL, 0, TACNA_ASP3_ENABLES1,
		    TACNA_ASP3_RX2_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP3RX3", NULL, 0, TACNA_ASP3_ENABLES1,
		    TACNA_ASP3_RX3_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP3RX4", NULL, 0, TACNA_ASP3_ENABLES1,
		    TACNA_ASP3_RX4_EN_SHIFT, 0),

SND_SOC_DAPM_AIF_IN("SLIMRX1", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		    TACNA_SLIMRX1_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("SLIMRX2", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		    TACNA_SLIMRX2_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("SLIMRX3", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		    TACNA_SLIMRX3_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("SLIMRX4", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		    TACNA_SLIMRX4_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("SLIMRX5", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		    TACNA_SLIMRX5_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("SLIMRX6", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		    TACNA_SLIMRX6_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("SLIMRX7", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		    TACNA_SLIMRX7_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("SLIMRX8", NULL, 0, TACNA_SLIMBUS_CHANNEL_ENABLE,
		    TACNA_SLIMRX8_EN_SHIFT, 0),

SND_SOC_DAPM_PGA("ASRC1IN1L", TACNA_ASRC1_ENABLE,
		 TACNA_ASRC1_IN1L_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ASRC1IN1R", TACNA_ASRC1_ENABLE,
		 TACNA_ASRC1_IN1R_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ASRC1IN2L", TACNA_ASRC1_ENABLE,
		 TACNA_ASRC1_IN2L_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ASRC1IN2R", TACNA_ASRC1_ENABLE,
		 TACNA_ASRC1_IN2R_EN_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_PGA("ASRC2IN1L", TACNA_ASRC2_ENABLE,
		 TACNA_ASRC2_IN1L_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ASRC2IN1R", TACNA_ASRC2_ENABLE,
		 TACNA_ASRC2_IN1R_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ASRC2IN2L", TACNA_ASRC2_ENABLE,
		 TACNA_ASRC2_IN2L_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ASRC2IN2R", TACNA_ASRC2_ENABLE,
		 TACNA_ASRC2_IN2R_EN_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_PGA("ISRC1DEC1", TACNA_ISRC1_CONTROL2,
		 TACNA_ISRC1_DEC1_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC1DEC2", TACNA_ISRC1_CONTROL2,
		 TACNA_ISRC1_DEC2_EN_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_PGA("ISRC1INT1", TACNA_ISRC1_CONTROL2,
		 TACNA_ISRC1_INT1_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC1INT2", TACNA_ISRC1_CONTROL2,
		 TACNA_ISRC1_INT2_EN_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_PGA("ISRC2DEC1", TACNA_ISRC2_CONTROL2,
		 TACNA_ISRC2_DEC1_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC2DEC2", TACNA_ISRC2_CONTROL2,
		 TACNA_ISRC2_DEC2_EN_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_PGA("ISRC2INT1", TACNA_ISRC2_CONTROL2,
		 TACNA_ISRC2_INT1_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC2INT2", TACNA_ISRC2_CONTROL2,
		 TACNA_ISRC2_INT2_EN_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_PGA_E("EQ1", TACNA_EQ_CONTROL1, TACNA_EQ1_EN_SHIFT, 0, NULL, 0,
		   tacna_eq_ev, SND_SOC_DAPM_PRE_PMU),
SND_SOC_DAPM_PGA_E("EQ2", TACNA_EQ_CONTROL1, TACNA_EQ2_EN_SHIFT, 0, NULL, 0,
		   tacna_eq_ev, SND_SOC_DAPM_PRE_PMU),
SND_SOC_DAPM_PGA_E("EQ3", TACNA_EQ_CONTROL1, TACNA_EQ3_EN_SHIFT, 0, NULL, 0,
		   tacna_eq_ev, SND_SOC_DAPM_PRE_PMU),
SND_SOC_DAPM_PGA_E("EQ4", TACNA_EQ_CONTROL1, TACNA_EQ4_EN_SHIFT, 0, NULL, 0,
		   tacna_eq_ev, SND_SOC_DAPM_PRE_PMU),

SND_SOC_DAPM_PGA("DRC1L", TACNA_DRC1_CONTROL1, TACNA_DRC1L_EN_SHIFT, 0,
		 NULL, 0),
SND_SOC_DAPM_PGA("DRC1R", TACNA_DRC1_CONTROL1, TACNA_DRC1R_EN_SHIFT, 0,
		 NULL, 0),
SND_SOC_DAPM_PGA("DRC2L", TACNA_DRC2_CONTROL1, TACNA_DRC2L_EN_SHIFT, 0,
		 NULL, 0),
SND_SOC_DAPM_PGA("DRC2R", TACNA_DRC2_CONTROL1, TACNA_DRC2R_EN_SHIFT, 0,
		 NULL, 0),

SND_SOC_DAPM_PGA("LHPF1", TACNA_LHPF_CONTROL1, TACNA_LHPF1_EN_SHIFT, 0,
		 NULL, 0),
SND_SOC_DAPM_PGA("LHPF2", TACNA_LHPF_CONTROL1, TACNA_LHPF2_EN_SHIFT, 0,
		 NULL, 0),
SND_SOC_DAPM_PGA("LHPF3", TACNA_LHPF_CONTROL1, TACNA_LHPF3_EN_SHIFT, 0,
		 NULL, 0),
SND_SOC_DAPM_PGA("LHPF4", TACNA_LHPF_CONTROL1, TACNA_LHPF4_EN_SHIFT, 0,
		 NULL, 0),

SND_SOC_DAPM_PGA("DFC1", TACNA_DFC1_CH1_CTRL, TACNA_DFC1_CH1_EN_SHIFT,
		 0, NULL, 0),
SND_SOC_DAPM_PGA("DFC2", TACNA_DFC1_CH2_CTRL, TACNA_DFC1_CH2_EN_SHIFT,
		 0, NULL, 0),
SND_SOC_DAPM_PGA("DFC3", TACNA_DFC1_CH3_CTRL, TACNA_DFC1_CH3_EN_SHIFT,
		 0, NULL, 0),
SND_SOC_DAPM_PGA("DFC4", TACNA_DFC1_CH4_CTRL, TACNA_DFC1_CH4_EN_SHIFT,
		 0, NULL, 0),
SND_SOC_DAPM_PGA("DFC5", TACNA_DFC1_CH5_CTRL, TACNA_DFC1_CH5_EN_SHIFT,
		 0, NULL, 0),
SND_SOC_DAPM_PGA("DFC6", TACNA_DFC1_CH6_CTRL, TACNA_DFC1_CH6_EN_SHIFT,
		 0, NULL, 0),
SND_SOC_DAPM_PGA("DFC7", TACNA_DFC1_CH7_CTRL, TACNA_DFC1_CH7_EN_SHIFT,
		 0, NULL, 0),
SND_SOC_DAPM_PGA("DFC8", TACNA_DFC1_CH8_CTRL, TACNA_DFC1_CH8_EN_SHIFT,
		 0, NULL, 0),

SND_SOC_DAPM_PGA("AOBRIDGE1IN1", TACNA_AOBRIDGE1_ENABLE,
		 TACNA_AOBRIDGE1_CH1_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("AOBRIDGE1IN2", TACNA_AOBRIDGE1_ENABLE,
		 TACNA_AOBRIDGE1_CH2_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("AOBRIDGE1IN3", TACNA_AOBRIDGE1_ENABLE,
		 TACNA_AOBRIDGE1_CH3_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("AOBRIDGE1IN4", TACNA_AOBRIDGE1_ENABLE,
		 TACNA_AOBRIDGE1_CH4_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("AOBRIDGE1IN5", TACNA_AOBRIDGE1_ENABLE,
		 TACNA_AOBRIDGE1_CH5_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("AOBRIDGE1IN6", TACNA_AOBRIDGE1_ENABLE,
		 TACNA_AOBRIDGE1_CH6_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("AOBRIDGE1IN7", TACNA_AOBRIDGE1_ENABLE,
		 TACNA_AOBRIDGE1_CH7_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("AOBRIDGE1IN8", TACNA_AOBRIDGE1_ENABLE,
		 TACNA_AOBRIDGE1_CH8_EN_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_PGA("AOBRIDGE2IN1", TACNA_AOBRIDGE2_ENABLE,
		 TACNA_AOBRIDGE2_CH1_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("AOBRIDGE2IN2", TACNA_AOBRIDGE2_ENABLE,
		 TACNA_AOBRIDGE2_CH2_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("AOBRIDGE2IN3", TACNA_AOBRIDGE2_ENABLE,
		 TACNA_AOBRIDGE2_CH3_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("AOBRIDGE2IN4", TACNA_AOBRIDGE2_ENABLE,
		 TACNA_AOBRIDGE2_CH4_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("AOBRIDGE2IN5", TACNA_AOBRIDGE2_ENABLE,
		 TACNA_AOBRIDGE2_CH5_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("AOBRIDGE2IN6", TACNA_AOBRIDGE2_ENABLE,
		 TACNA_AOBRIDGE2_CH6_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("AOBRIDGE2IN7", TACNA_AOBRIDGE2_ENABLE,
		 TACNA_AOBRIDGE2_CH7_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("AOBRIDGE2IN8", TACNA_AOBRIDGE2_ENABLE,
		 TACNA_AOBRIDGE2_CH8_EN_SHIFT, 0, NULL, 0),

WM_HALO("DSP1", 0, wm_halo_early_event),

/* end of ordered widget list */

TACNA_MIXER_WIDGETS(EQ1, "EQ1"),
TACNA_MIXER_WIDGETS(EQ2, "EQ2"),
TACNA_MIXER_WIDGETS(EQ3, "EQ3"),
TACNA_MIXER_WIDGETS(EQ4, "EQ4"),

TACNA_MIXER_WIDGETS(DRC1L, "DRC1L"),
TACNA_MIXER_WIDGETS(DRC1R, "DRC1R"),
TACNA_MIXER_WIDGETS(DRC2L, "DRC2L"),
TACNA_MIXER_WIDGETS(DRC2R, "DRC2R"),

SND_SOC_DAPM_SWITCH("DRC1 Activity Output", SND_SOC_NOPM, 0, 0,
		    &tacna_drc_activity_output_mux[0]),
SND_SOC_DAPM_SWITCH("DRC2 Activity Output", SND_SOC_NOPM, 0, 0,
		    &tacna_drc_activity_output_mux[1]),

TACNA_MIXER_WIDGETS(LHPF1, "LHPF1"),
TACNA_MIXER_WIDGETS(LHPF2, "LHPF2"),
TACNA_MIXER_WIDGETS(LHPF3, "LHPF3"),
TACNA_MIXER_WIDGETS(LHPF4, "LHPF4"),

TACNA_MIXER_WIDGETS(OUT1L, "OUT1L"),
TACNA_MIXER_WIDGETS(OUT1R, "OUT1R"),
TACNA_MIXER_WIDGETS(OUT5L, "OUT5L"),
TACNA_MIXER_WIDGETS(OUT5R, "OUT5R"),
TACNA_MIXER_WIDGETS(OUTAUX1L, "OUTAUX1L"),
TACNA_MIXER_WIDGETS(OUTAUX1R, "OUTAUX1R"),
TACNA_MIXER_WIDGETS(OUTHL, "OUTHL"),
TACNA_MIXER_WIDGETS(OUTHR, "OUTHR"),

TACNA_MIXER_WIDGETS(ASP1TX1, "ASP1TX1"),
TACNA_MIXER_WIDGETS(ASP1TX2, "ASP1TX2"),
TACNA_MIXER_WIDGETS(ASP1TX3, "ASP1TX3"),
TACNA_MIXER_WIDGETS(ASP1TX4, "ASP1TX4"),
TACNA_MIXER_WIDGETS(ASP1TX5, "ASP1TX5"),
TACNA_MIXER_WIDGETS(ASP1TX6, "ASP1TX6"),
TACNA_MIXER_WIDGETS(ASP1TX7, "ASP1TX7"),
TACNA_MIXER_WIDGETS(ASP1TX8, "ASP1TX8"),

TACNA_MIXER_WIDGETS(ASP2TX1, "ASP2TX1"),
TACNA_MIXER_WIDGETS(ASP2TX2, "ASP2TX2"),
TACNA_MIXER_WIDGETS(ASP2TX3, "ASP2TX3"),
TACNA_MIXER_WIDGETS(ASP2TX4, "ASP2TX4"),

TACNA_MIXER_WIDGETS(ASP3TX1, "ASP3TX1"),
TACNA_MIXER_WIDGETS(ASP3TX2, "ASP3TX2"),
TACNA_MIXER_WIDGETS(ASP3TX3, "ASP3TX3"),
TACNA_MIXER_WIDGETS(ASP3TX4, "ASP3TX4"),

TACNA_MIXER_WIDGETS(SLIMTX1, "SLIMTX1"),
TACNA_MIXER_WIDGETS(SLIMTX2, "SLIMTX2"),
TACNA_MIXER_WIDGETS(SLIMTX3, "SLIMTX3"),
TACNA_MIXER_WIDGETS(SLIMTX4, "SLIMTX4"),
TACNA_MIXER_WIDGETS(SLIMTX5, "SLIMTX5"),
TACNA_MIXER_WIDGETS(SLIMTX6, "SLIMTX6"),
TACNA_MIXER_WIDGETS(SLIMTX7, "SLIMTX7"),
TACNA_MIXER_WIDGETS(SLIMTX8, "SLIMTX8"),

TACNA_MUX_WIDGETS(ASRC1IN1L, "ASRC1IN1L"),
TACNA_MUX_WIDGETS(ASRC1IN1R, "ASRC1IN1R"),
TACNA_MUX_WIDGETS(ASRC1IN2L, "ASRC1IN2L"),
TACNA_MUX_WIDGETS(ASRC1IN2R, "ASRC1IN2R"),

TACNA_MUX_WIDGETS(ASRC2IN1L, "ASRC2IN1L"),
TACNA_MUX_WIDGETS(ASRC2IN1R, "ASRC2IN1R"),
TACNA_MUX_WIDGETS(ASRC2IN2L, "ASRC2IN2L"),
TACNA_MUX_WIDGETS(ASRC2IN2R, "ASRC2IN2R"),

TACNA_MUX_WIDGETS(ISRC1DEC1, "ISRC1DEC1"),
TACNA_MUX_WIDGETS(ISRC1DEC2, "ISRC1DEC2"),

TACNA_MUX_WIDGETS(ISRC1INT1, "ISRC1INT1"),
TACNA_MUX_WIDGETS(ISRC1INT2, "ISRC1INT2"),

TACNA_MUX_WIDGETS(ISRC2DEC1, "ISRC2DEC1"),
TACNA_MUX_WIDGETS(ISRC2DEC2, "ISRC2DEC2"),

TACNA_MUX_WIDGETS(ISRC2INT1, "ISRC2INT1"),
TACNA_MUX_WIDGETS(ISRC2INT2, "ISRC2INT2"),

TACNA_MIXER_WIDGETS(AOBRIDGE2IN1, "AOBRIDGE2IN1"),
TACNA_MIXER_WIDGETS(AOBRIDGE2IN2, "AOBRIDGE2IN2"),
TACNA_MIXER_WIDGETS(AOBRIDGE2IN3, "AOBRIDGE2IN3"),
TACNA_MIXER_WIDGETS(AOBRIDGE2IN4, "AOBRIDGE2IN4"),
TACNA_MIXER_WIDGETS(AOBRIDGE2IN5, "AOBRIDGE2IN5"),
TACNA_MIXER_WIDGETS(AOBRIDGE2IN6, "AOBRIDGE2IN6"),
TACNA_MIXER_WIDGETS(AOBRIDGE2IN7, "AOBRIDGE2IN7"),
TACNA_MIXER_WIDGETS(AOBRIDGE2IN8, "AOBRIDGE2IN8"),

TACNA_MUX_WIDGETS(DFC1, "DFC1"),
TACNA_MUX_WIDGETS(DFC2, "DFC2"),
TACNA_MUX_WIDGETS(DFC3, "DFC3"),
TACNA_MUX_WIDGETS(DFC4, "DFC4"),
TACNA_MUX_WIDGETS(DFC5, "DFC5"),
TACNA_MUX_WIDGETS(DFC6, "DFC6"),
TACNA_MUX_WIDGETS(DFC7, "DFC7"),
TACNA_MUX_WIDGETS(DFC8, "DFC8"),

TACNA_MIXER_WIDGETS(DSP1RX1, "DSP1RX1"),
TACNA_MIXER_WIDGETS(DSP1RX2, "DSP1RX2"),
TACNA_MIXER_WIDGETS(DSP1RX3, "DSP1RX3"),
TACNA_MIXER_WIDGETS(DSP1RX4, "DSP1RX4"),
TACNA_MIXER_WIDGETS(DSP1RX5, "DSP1RX5"),
TACNA_MIXER_WIDGETS(DSP1RX6, "DSP1RX6"),
TACNA_MIXER_WIDGETS(DSP1RX7, "DSP1RX7"),
TACNA_MIXER_WIDGETS(DSP1RX8, "DSP1RX8"),
TACNA_MIXER_WIDGETS(DSP1RX9, "DSP1RX9"),
TACNA_MIXER_WIDGETS(DSP1RX10, "DSP1RX10"),
TACNA_MIXER_WIDGETS(DSP1RX11, "DSP1RX11"),
TACNA_MIXER_WIDGETS(DSP1RX12, "DSP1RX12"),

SND_SOC_DAPM_SWITCH("DSP1 Trigger Output", SND_SOC_NOPM, 0, 0,
		    &tacna_dsp_trigger_output_mux[0]),

SND_SOC_DAPM_SUPPLY("ANC NG External Clock", SND_SOC_NOPM,
		    TACNA_ANC_EXT_NG_SET_SHIFT, 0, tacna_anc_ev,
		    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_PGA("ANCL NG External", SND_SOC_NOPM, 0, 0, NULL, 0),

SND_SOC_DAPM_SUPPLY("ANC NG Clock", SND_SOC_NOPM,
		    TACNA_ANC_NG_CLK_SET_SHIFT, 0, tacna_anc_ev,
		    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_PGA("ANCL NG Internal", SND_SOC_NOPM, 0, 0, NULL, 0),

SND_SOC_DAPM_MUX("ANCL Left Input", SND_SOC_NOPM, 0, 0,
		 &cs47l96_anc_input_mux[0]),
SND_SOC_DAPM_MUX("ANCL Right Input", SND_SOC_NOPM, 0, 0,
		 &cs47l96_anc_input_mux[0]),
SND_SOC_DAPM_MUX("ANCL Channel", SND_SOC_NOPM, 0, 0,
		 &cs47l96_anc_input_mux[1]),
SND_SOC_DAPM_MUX("ANCL NG Mux", SND_SOC_NOPM, 0, 0, &cs47l96_anc_ng_mux),

SND_SOC_DAPM_PGA_E("ANCL", SND_SOC_NOPM, TACNA_ANC_L_CLK_SET_SHIFT,
		   0, NULL, 0, tacna_anc_ev,
		   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

SND_SOC_DAPM_MUX("OUT1L ANC Source", SND_SOC_NOPM, 0, 0,
		 &cs47l96_output_anc_src[0]),
SND_SOC_DAPM_MUX("OUT1R ANC Source", SND_SOC_NOPM, 0, 0,
		 &cs47l96_output_anc_src[1]),
SND_SOC_DAPM_MUX("OUT5L ANC Source", SND_SOC_NOPM, 0, 0,
		 &cs47l96_output_anc_src[2]),
SND_SOC_DAPM_MUX("OUT5R ANC Source", SND_SOC_NOPM, 0, 0,
		 &cs47l96_output_anc_src[3]),

SND_SOC_DAPM_OUTPUT("OUT1L_HP1"),
SND_SOC_DAPM_OUTPUT("OUT1R_HP1"),
SND_SOC_DAPM_OUTPUT("OUT1L_HP2"),
SND_SOC_DAPM_OUTPUT("OUT1R_HP2"),
SND_SOC_DAPM_OUTPUT("OUT5_PDMCLK"),
SND_SOC_DAPM_OUTPUT("OUT5_PDMDATA"),
SND_SOC_DAPM_OUTPUT("AUXPDM1_CLK"),
SND_SOC_DAPM_OUTPUT("AUXPDM1_DOUT"),
SND_SOC_DAPM_OUTPUT("AUXPDM2_CLK"),
SND_SOC_DAPM_OUTPUT("AUXPDM2_DOUT"),
SND_SOC_DAPM_OUTPUT("AUXPDM3_CLK"),
SND_SOC_DAPM_OUTPUT("AUXPDM3_DOUT"),

SND_SOC_DAPM_OUTPUT("MICSUPP"),
};

#define TACNA_MIXER_INPUT_ROUTES(name) \
	{ name, "Tone Generator 1", "Tone Generator 1" }, \
	{ name, "Tone Generator 2", "Tone Generator 2" }, \
	{ name, "Haptics", "HAPTICS" }, \
	{ name, "AEC1", "AEC1 Loopback" }, \
	{ name, "AEC2", "AEC2 Loopback" }, \
	{ name, "Noise Generator", "Noise Generator" }, \
	{ name, "IN1L", "IN1L PGA" }, \
	{ name, "IN1R", "IN1R PGA" }, \
	{ name, "IN2L", "IN2L PGA" }, \
	{ name, "IN2R", "IN2R PGA" }, \
	{ name, "IN3L", "IN3L PGA" }, \
	{ name, "IN3R", "IN3R PGA" }, \
	{ name, "IN4L", "IN4L PGA" }, \
	{ name, "IN4R", "IN4R PGA" }, \
	{ name, "ASP1RX1", "ASP1RX1" }, \
	{ name, "ASP1RX2", "ASP1RX2" }, \
	{ name, "ASP1RX3", "ASP1RX3" }, \
	{ name, "ASP1RX4", "ASP1RX4" }, \
	{ name, "ASP1RX5", "ASP1RX5" }, \
	{ name, "ASP1RX6", "ASP1RX6" }, \
	{ name, "ASP1RX7", "ASP1RX7" }, \
	{ name, "ASP1RX8", "ASP1RX8" }, \
	{ name, "ASP2RX1", "ASP2RX1" }, \
	{ name, "ASP2RX2", "ASP2RX2" }, \
	{ name, "ASP2RX3", "ASP2RX3" }, \
	{ name, "ASP2RX4", "ASP2RX4" }, \
	{ name, "ASP3RX1", "ASP3RX1" }, \
	{ name, "ASP3RX2", "ASP3RX2" }, \
	{ name, "ASP3RX3", "ASP3RX3" }, \
	{ name, "ASP3RX4", "ASP3RX4" }, \
	{ name, "SLIMRX1", "SLIMRX1" }, \
	{ name, "SLIMRX2", "SLIMRX2" }, \
	{ name, "SLIMRX3", "SLIMRX3" }, \
	{ name, "SLIMRX4", "SLIMRX4" }, \
	{ name, "SLIMRX5", "SLIMRX5" }, \
	{ name, "SLIMRX6", "SLIMRX6" }, \
	{ name, "SLIMRX7", "SLIMRX7" }, \
	{ name, "SLIMRX8", "SLIMRX8" }, \
	{ name, "ASRC1IN1L", "ASRC1IN1L" }, \
	{ name, "ASRC1IN1R", "ASRC1IN1R" }, \
	{ name, "ASRC1IN2L", "ASRC1IN2L" }, \
	{ name, "ASRC1IN2R", "ASRC1IN2R" }, \
	{ name, "ASRC2IN1L", "ASRC2IN1L" }, \
	{ name, "ASRC2IN1R", "ASRC2IN1R" }, \
	{ name, "ASRC2IN2L", "ASRC2IN2L" }, \
	{ name, "ASRC2IN2R", "ASRC2IN2R" }, \
	{ name, "ISRC1DEC1", "ISRC1DEC1" }, \
	{ name, "ISRC1DEC2", "ISRC1DEC2" }, \
	{ name, "ISRC1INT1", "ISRC1INT1" }, \
	{ name, "ISRC1INT2", "ISRC1INT2" }, \
	{ name, "ISRC2DEC1", "ISRC2DEC1" }, \
	{ name, "ISRC2DEC2", "ISRC2DEC2" }, \
	{ name, "ISRC2INT1", "ISRC2INT1" }, \
	{ name, "ISRC2INT2", "ISRC2INT2" }, \
	{ name, "EQ1", "EQ1" }, \
	{ name, "EQ2", "EQ2" }, \
	{ name, "EQ3", "EQ3" }, \
	{ name, "EQ4", "EQ4" }, \
	{ name, "DRC1L", "DRC1L" }, \
	{ name, "DRC1R", "DRC1R" }, \
	{ name, "DRC2L", "DRC2L" }, \
	{ name, "DRC2R", "DRC2R" }, \
	{ name, "LHPF1", "LHPF1" }, \
	{ name, "LHPF2", "LHPF2" }, \
	{ name, "LHPF3", "LHPF3" }, \
	{ name, "LHPF4", "LHPF4" }, \
	{ name, "DFC1", "DFC1" }, \
	{ name, "DFC2", "DFC2" }, \
	{ name, "DFC3", "DFC3" }, \
	{ name, "DFC4", "DFC4" }, \
	{ name, "DFC5", "DFC5" }, \
	{ name, "DFC6", "DFC6" }, \
	{ name, "DFC7", "DFC7" }, \
	{ name, "DFC8", "DFC8" }, \
	{ name, "AOBRIDGE1IN1", "AOBRIDGE1IN1" }, \
	{ name, "AOBRIDGE1IN2", "AOBRIDGE1IN2" }, \
	{ name, "AOBRIDGE1IN3", "AOBRIDGE1IN3" }, \
	{ name, "AOBRIDGE1IN4", "AOBRIDGE1IN4" }, \
	{ name, "AOBRIDGE1IN5", "AOBRIDGE1IN5" }, \
	{ name, "AOBRIDGE1IN6", "AOBRIDGE1IN6" }, \
	{ name, "AOBRIDGE1IN7", "AOBRIDGE1IN7" }, \
	{ name, "AOBRIDGE1IN8", "AOBRIDGE1IN8" }, \
	{ name, "DSP1.1", "DSP1" }, \
	{ name, "DSP1.2", "DSP1" }, \
	{ name, "DSP1.3", "DSP1" }, \
	{ name, "DSP1.4", "DSP1" }, \
	{ name, "DSP1.5", "DSP1" }, \
	{ name, "DSP1.6", "DSP1" }, \
	{ name, "DSP1.7", "DSP1" }, \
	{ name, "DSP1.8", "DSP1" }

static const struct snd_soc_dapm_route cs47l96_dapm_routes[] = {
	{ "OUT1L PGA", NULL, "VDD1_CP" },
	{ "OUT1L PGA", NULL, "VDD2_CP" },
	{ "OUT1L PGA", NULL, "VDD3_CP" },
	{ "OUT1R PGA", NULL, "VDD1_CP" },
	{ "OUT1R PGA", NULL, "VDD2_CP" },
	{ "OUT1R PGA", NULL, "VDD3_CP" },

	{ "OUT1L PGA", NULL, "SYSCLK" },
	{ "OUT1R PGA", NULL, "SYSCLK" },
	{ "OUT5L PGA", NULL, "SYSCLK" },
	{ "OUT5R PGA", NULL, "SYSCLK" },
	{ "OUTH Output Select", NULL, "SYSCLK"},

	{ "OPCLK", NULL, "SYSCLK" },
	{ "ASYNCOPCLK", NULL, "ASYNCCLK" },

	{ "OUT1L PGA", NULL, "DACCLK" },
	{ "OUT1R PGA", NULL, "DACCLK" },
	{ "OUT5L PGA", NULL, "DACCLK" },
	{ "OUT5R PGA", NULL, "DACCLK" },
	{ "OUTH Output Select", NULL, "DACCLK"},

	{ "OUT1L PGA", NULL, "DAC_ASYNCCLK" },
	{ "OUT1R PGA", NULL, "DAC_ASYNCCLK" },
	{ "OUT5L PGA", NULL, "DAC_ASYNCCLK" },
	{ "OUT5R PGA", NULL, "DAC_ASYNCCLK" },
	{ "OUTH Output Select", NULL, "OUTH_ASYNCCLK"},

	{ "IN1LN_1", NULL, "SYSCLK" },
	{ "IN1LN_2", NULL, "SYSCLK" },
	{ "IN1LP_1", NULL, "SYSCLK" },
	{ "IN1LP_2", NULL, "SYSCLK" },
	{ "IN1RN_1", NULL, "SYSCLK" },
	{ "IN1RN_2", NULL, "SYSCLK" },
	{ "IN1RP_1", NULL, "SYSCLK" },
	{ "IN1RP_2", NULL, "SYSCLK" },
	{ "IN1_PDMCLK", NULL, "SYSCLK" },
	{ "IN1_PDMDATA", NULL, "SYSCLK" },
	{ "IN2LN_1", NULL, "SYSCLK" },
	{ "IN2LN_2", NULL, "SYSCLK" },
	{ "IN2LP_1", NULL, "SYSCLK" },
	{ "IN2LP_2", NULL, "SYSCLK" },
	{ "IN2RN_1", NULL, "SYSCLK" },
	{ "IN2RN_2", NULL, "SYSCLK" },
	{ "IN2RP_1", NULL, "SYSCLK" },
	{ "IN2RP_2", NULL, "SYSCLK" },
	{ "IN2_PDMCLK", NULL, "SYSCLK" },
	{ "IN2_PDMDATA", NULL, "SYSCLK" },
	{ "IN3_PDMCLK", NULL, "SYSCLK" },
	{ "IN3_PDMDATA", NULL, "SYSCLK" },
	{ "IN4_PDMCLK", NULL, "SYSCLK" },
	{ "IN4_PDMDATA", NULL, "SYSCLK" },

	{ "DSP1", NULL, "DSP1FREQ" },
	{ "Audio Trace DSP", NULL, "DSP1" },

	{ "IN4_PDMCLK", NULL, "VDD_IO2" },
	{ "IN4_PDMDATA", NULL, "VDD_IO2" },

	{ "MICBIAS1", NULL, "VOUT_MIC" },
	{ "MICBIAS2", NULL, "VOUT_MIC" },

	{ "MICBIAS1A", NULL, "MICBIAS1" },
	{ "MICBIAS1B", NULL, "MICBIAS1" },
	{ "MICBIAS1C", NULL, "MICBIAS1" },
	{ "MICBIAS1D", NULL, "MICBIAS1" },

	{ "MICBIAS2A", NULL, "MICBIAS2" },
	{ "MICBIAS2B", NULL, "MICBIAS2" },

	{ "Tone Generator 1", NULL, "SYSCLK" },
	{ "Tone Generator 2", NULL, "SYSCLK" },
	{ "Noise Generator", NULL, "SYSCLK" },

	{ "Tone Generator 1", NULL, "TONE" },
	{ "Tone Generator 2", NULL, "TONE" },
	{ "Noise Generator", NULL, "NOISE" },

	{ "ASP1 Capture", NULL, "ASP1TX1" },
	{ "ASP1 Capture", NULL, "ASP1TX2" },
	{ "ASP1 Capture", NULL, "ASP1TX3" },
	{ "ASP1 Capture", NULL, "ASP1TX4" },
	{ "ASP1 Capture", NULL, "ASP1TX5" },
	{ "ASP1 Capture", NULL, "ASP1TX6" },
	{ "ASP1 Capture", NULL, "ASP1TX7" },
	{ "ASP1 Capture", NULL, "ASP1TX8" },

	{ "ASP1RX1", NULL, "ASP1 Playback" },
	{ "ASP1RX2", NULL, "ASP1 Playback" },
	{ "ASP1RX3", NULL, "ASP1 Playback" },
	{ "ASP1RX4", NULL, "ASP1 Playback" },
	{ "ASP1RX5", NULL, "ASP1 Playback" },
	{ "ASP1RX6", NULL, "ASP1 Playback" },
	{ "ASP1RX7", NULL, "ASP1 Playback" },
	{ "ASP1RX8", NULL, "ASP1 Playback" },

	{ "ASP2 Capture", NULL, "ASP2TX1" },
	{ "ASP2 Capture", NULL, "ASP2TX2" },
	{ "ASP2 Capture", NULL, "ASP2TX3" },
	{ "ASP2 Capture", NULL, "ASP2TX4" },

	{ "ASP2RX1", NULL, "ASP2 Playback" },
	{ "ASP2RX2", NULL, "ASP2 Playback" },
	{ "ASP2RX3", NULL, "ASP2 Playback" },
	{ "ASP2RX4", NULL, "ASP2 Playback" },

	{ "ASP3 Capture", NULL, "ASP3TX1" },
	{ "ASP3 Capture", NULL, "ASP3TX2" },
	{ "ASP3 Capture", NULL, "ASP3TX3" },
	{ "ASP3 Capture", NULL, "ASP3TX4" },

	{ "ASP3RX1", NULL, "ASP3 Playback" },
	{ "ASP3RX2", NULL, "ASP3 Playback" },
	{ "ASP3RX3", NULL, "ASP3 Playback" },
	{ "ASP3RX4", NULL, "ASP3 Playback" },

	{ "Slim1 Capture", NULL, "SLIMTX1" },
	{ "Slim1 Capture", NULL, "SLIMTX2" },
	{ "Slim1 Capture", NULL, "SLIMTX3" },
	{ "Slim1 Capture", NULL, "SLIMTX4" },

	{ "SLIMRX1", NULL, "Slim1 Playback" },
	{ "SLIMRX2", NULL, "Slim1 Playback" },
	{ "SLIMRX3", NULL, "Slim1 Playback" },
	{ "SLIMRX4", NULL, "Slim1 Playback" },

	{ "Slim2 Capture", NULL, "SLIMTX5" },
	{ "Slim2 Capture", NULL, "SLIMTX6" },

	{ "SLIMRX5", NULL, "Slim2 Playback" },
	{ "SLIMRX6", NULL, "Slim2 Playback" },

	{ "Slim3 Capture", NULL, "SLIMTX7" },
	{ "Slim3 Capture", NULL, "SLIMTX8" },

	{ "SLIMRX7", NULL, "Slim3 Playback" },
	{ "SLIMRX8", NULL, "Slim3 Playback" },

	{ "ASP1 Playback", NULL, "SYSCLK" },
	{ "ASP2 Playback", NULL, "SYSCLK" },
	{ "ASP3 Playback", NULL, "SYSCLK" },
	{ "Slim1 Playback", NULL, "SYSCLK" },
	{ "Slim2 Playback", NULL, "SYSCLK" },
	{ "Slim3 Playback", NULL, "SYSCLK" },

	{ "ASP1 Capture", NULL, "SYSCLK" },
	{ "ASP2 Capture", NULL, "SYSCLK" },
	{ "ASP3 Capture", NULL, "SYSCLK" },
	{ "Slim1 Capture", NULL, "SYSCLK" },
	{ "Slim2 Capture", NULL, "SYSCLK" },
	{ "Slim3 Capture", NULL, "SYSCLK" },

	{ "ASRC1IN1L", NULL, "SYSCLK" },
	{ "ASRC1IN1R", NULL, "SYSCLK" },
	{ "ASRC1IN1L", NULL, "ASYNCCLK" },
	{ "ASRC1IN1R", NULL, "ASYNCCLK" },
	{ "ASRC1IN2L", NULL, "SYSCLK" },
	{ "ASRC1IN2R", NULL, "SYSCLK" },
	{ "ASRC1IN2L", NULL, "ASYNCCLK" },
	{ "ASRC1IN2R", NULL, "ASYNCCLK" },
	{ "ASRC2IN1L", NULL, "SYSCLK" },
	{ "ASRC2IN1R", NULL, "SYSCLK" },
	{ "ASRC2IN1L", NULL, "ASYNCCLK" },
	{ "ASRC2IN1R", NULL, "ASYNCCLK" },
	{ "ASRC2IN2L", NULL, "SYSCLK" },
	{ "ASRC2IN2R", NULL, "SYSCLK" },
	{ "ASRC2IN2L", NULL, "ASYNCCLK" },
	{ "ASRC2IN2R", NULL, "ASYNCCLK" },

	{ "IN1L Mux", "Analog 1", "IN1LN_1" },
	{ "IN1L Mux", "Analog 2", "IN1LN_2" },
	{ "IN1L Mux", "Analog 1", "IN1LP_1" },
	{ "IN1L Mux", "Analog 2", "IN1LP_2" },
	{ "IN1R Mux", "Analog 1", "IN1RN_1" },
	{ "IN1R Mux", "Analog 2", "IN1RN_2" },
	{ "IN1R Mux", "Analog 1", "IN1RP_1" },
	{ "IN1R Mux", "Analog 2", "IN1RP_2" },

	{ "IN2L Mux", "Analog 1", "IN2LN_1" },
	{ "IN2L Mux", "Analog 2", "IN2LN_2" },
	{ "IN2L Mux", "Analog 1", "IN2LP_1" },
	{ "IN2L Mux", "Analog 2", "IN2LP_2" },
	{ "IN2R Mux", "Analog 1", "IN2RN_1" },
	{ "IN2R Mux", "Analog 2", "IN2RN_2" },
	{ "IN2R Mux", "Analog 1", "IN2RP_1" },
	{ "IN2R Mux", "Analog 2", "IN2RP_2" },

	{ "IN1L Mux", NULL, "MICD_COMP_FRC" },
	{ "IN1R Mux", NULL, "MICD_COMP_FRC" },

	{ "IN1L Mode", "Analog", "IN1L Mux" },
	{ "IN1R Mode", "Analog", "IN1R Mux" },
	{ "IN1L Mode", "Digital", "IN1_PDMCLK" },
	{ "IN1L Mode", "Digital", "IN1_PDMDATA" },
	{ "IN1R Mode", "Digital", "IN1_PDMCLK" },
	{ "IN1R Mode", "Digital", "IN1_PDMDATA" },
	{ "IN1L Swap Chan", "Normal", "IN1L Mode" },
	{ "IN1R Swap Chan", "Normal", "IN1R Mode" },
	{ "IN1L Swap Chan", "Swap",   "IN1R Mode" },
	{ "IN1R Swap Chan", "Swap",   "IN1L Mode" },
	{ "IN1L Swap Chan", "Left",   "IN1L Mode" },
	{ "IN1R Swap Chan", "Left",   "IN1L Mode" },
	{ "IN1L Swap Chan", "Right",  "IN1R Mode" },
	{ "IN1R Swap Chan", "Right",  "IN1R Mode" },
	{ "IN1L PGA", NULL, "IN1L Swap Chan" },
	{ "IN1R PGA", NULL, "IN1R Swap Chan" },

	{ "IN1L PGA", NULL, "VOUT_MIC" },
	{ "IN1R PGA", NULL, "VOUT_MIC" },

	{ "IN2L Mux", NULL, "MICD_COMP_FRC" },
	{ "IN2R Mux", NULL, "MICD_COMP_FRC" },

	{ "IN2L Mode", "Analog", "IN2L Mux" },
	{ "IN2R Mode", "Analog", "IN2R Mux" },
	{ "IN2L Mode", "Digital", "IN2_PDMCLK" },
	{ "IN2L Mode", "Digital", "IN2_PDMDATA" },
	{ "IN2R Mode", "Digital", "IN2_PDMCLK" },
	{ "IN2R Mode", "Digital", "IN2_PDMDATA" },
	{ "IN2L Swap Chan", "Normal", "IN2L Mode" },
	{ "IN2R Swap Chan", "Normal", "IN2R Mode" },
	{ "IN2L Swap Chan", "Swap",   "IN2R Mode" },
	{ "IN2R Swap Chan", "Swap",   "IN2L Mode" },
	{ "IN2L Swap Chan", "Left",   "IN2L Mode" },
	{ "IN2R Swap Chan", "Left",   "IN2L Mode" },
	{ "IN2L Swap Chan", "Right",  "IN2R Mode" },
	{ "IN2R Swap Chan", "Right",  "IN2R Mode" },
	{ "IN2L PGA", NULL, "IN2L Swap Chan" },
	{ "IN2R PGA", NULL, "IN2R Swap Chan" },

	{ "IN2L PGA", NULL, "VOUT_MIC" },
	{ "IN2R PGA", NULL, "VOUT_MIC" },

	{ "IN3L PGA", NULL, "IN3_PDMCLK" },
	{ "IN3L PGA", NULL, "IN3_PDMDATA" },
	{ "IN3R PGA", NULL, "IN3_PDMCLK" },
	{ "IN3R PGA", NULL, "IN3_PDMDATA" },

	{ "IN4L PGA", NULL, "IN4_PDMCLK" },
	{ "IN4L PGA", NULL, "IN4_PDMDATA" },
	{ "IN4R PGA", NULL, "IN4_PDMCLK" },
	{ "IN4R PGA", NULL, "IN4_PDMDATA" },

	TACNA_MIXER_ROUTES("OUT1L PGA", "OUT1L"),
	TACNA_MIXER_ROUTES("OUT1R PGA", "OUT1R"),
	TACNA_MIXER_ROUTES("OUT5L PGA", "OUT5L"),
	TACNA_MIXER_ROUTES("OUT5R PGA", "OUT5R"),
	TACNA_MIXER_ROUTES("OUTAUX1L PGA", "OUTAUX1L"),
	TACNA_MIXER_ROUTES("OUTAUX1R PGA", "OUTAUX1R"),
	TACNA_MIXER_ROUTES("OUTH PCM", "OUTHL"),
	TACNA_MIXER_ROUTES("OUTH PCM", "OUTHR"),

	TACNA_MIXER_ROUTES("ASP1TX1", "ASP1TX1"),
	TACNA_MIXER_ROUTES("ASP1TX2", "ASP1TX2"),
	TACNA_MIXER_ROUTES("ASP1TX3", "ASP1TX3"),
	TACNA_MIXER_ROUTES("ASP1TX4", "ASP1TX4"),
	TACNA_MIXER_ROUTES("ASP1TX5", "ASP1TX5"),
	TACNA_MIXER_ROUTES("ASP1TX6", "ASP1TX6"),
	TACNA_MIXER_ROUTES("ASP1TX7", "ASP1TX7"),
	TACNA_MIXER_ROUTES("ASP1TX8", "ASP1TX8"),

	TACNA_MIXER_ROUTES("ASP2TX1", "ASP2TX1"),
	TACNA_MIXER_ROUTES("ASP2TX2", "ASP2TX2"),
	TACNA_MIXER_ROUTES("ASP2TX3", "ASP2TX3"),
	TACNA_MIXER_ROUTES("ASP2TX4", "ASP2TX4"),

	TACNA_MIXER_ROUTES("ASP3TX1", "ASP3TX1"),
	TACNA_MIXER_ROUTES("ASP3TX2", "ASP3TX2"),
	TACNA_MIXER_ROUTES("ASP3TX3", "ASP3TX3"),
	TACNA_MIXER_ROUTES("ASP3TX4", "ASP3TX4"),

	TACNA_MIXER_ROUTES("SLIMTX1", "SLIMTX1"),
	TACNA_MIXER_ROUTES("SLIMTX2", "SLIMTX2"),
	TACNA_MIXER_ROUTES("SLIMTX3", "SLIMTX3"),
	TACNA_MIXER_ROUTES("SLIMTX4", "SLIMTX4"),
	TACNA_MIXER_ROUTES("SLIMTX5", "SLIMTX5"),
	TACNA_MIXER_ROUTES("SLIMTX6", "SLIMTX6"),
	TACNA_MIXER_ROUTES("SLIMTX7", "SLIMTX7"),
	TACNA_MIXER_ROUTES("SLIMTX8", "SLIMTX8"),

	TACNA_MIXER_ROUTES("EQ1", "EQ1"),
	TACNA_MIXER_ROUTES("EQ2", "EQ2"),
	TACNA_MIXER_ROUTES("EQ3", "EQ3"),
	TACNA_MIXER_ROUTES("EQ4", "EQ4"),

	TACNA_MIXER_ROUTES("DRC1L", "DRC1L"),
	TACNA_MIXER_ROUTES("DRC1R", "DRC1R"),
	TACNA_MIXER_ROUTES("DRC2L", "DRC2L"),
	TACNA_MIXER_ROUTES("DRC2R", "DRC2R"),

	TACNA_MIXER_ROUTES("LHPF1", "LHPF1"),
	TACNA_MIXER_ROUTES("LHPF2", "LHPF2"),
	TACNA_MIXER_ROUTES("LHPF3", "LHPF3"),
	TACNA_MIXER_ROUTES("LHPF4", "LHPF4"),

	TACNA_MUX_ROUTES("ASRC1IN1L", "ASRC1IN1L"),
	TACNA_MUX_ROUTES("ASRC1IN1R", "ASRC1IN1R"),
	TACNA_MUX_ROUTES("ASRC1IN2L", "ASRC1IN2L"),
	TACNA_MUX_ROUTES("ASRC1IN2R", "ASRC1IN2R"),

	TACNA_MUX_ROUTES("ASRC2IN1L", "ASRC2IN1L"),
	TACNA_MUX_ROUTES("ASRC2IN1R", "ASRC2IN1R"),
	TACNA_MUX_ROUTES("ASRC2IN2L", "ASRC2IN2L"),
	TACNA_MUX_ROUTES("ASRC2IN2R", "ASRC2IN2R"),

	TACNA_MUX_ROUTES("ISRC1INT1", "ISRC1INT1"),
	TACNA_MUX_ROUTES("ISRC1INT2", "ISRC1INT2"),

	TACNA_MUX_ROUTES("ISRC1DEC1", "ISRC1DEC1"),
	TACNA_MUX_ROUTES("ISRC1DEC2", "ISRC1DEC2"),

	TACNA_MUX_ROUTES("ISRC2INT1", "ISRC2INT1"),
	TACNA_MUX_ROUTES("ISRC2INT2", "ISRC2INT2"),

	TACNA_MUX_ROUTES("ISRC2DEC1", "ISRC2DEC1"),
	TACNA_MUX_ROUTES("ISRC2DEC2", "ISRC2DEC2"),

	TACNA_MIXER_ROUTES("AOBRIDGE2IN1", "AOBRIDGE2IN1"),
	TACNA_MIXER_ROUTES("AOBRIDGE2IN2", "AOBRIDGE2IN2"),
	TACNA_MIXER_ROUTES("AOBRIDGE2IN3", "AOBRIDGE2IN3"),
	TACNA_MIXER_ROUTES("AOBRIDGE2IN4", "AOBRIDGE2IN4"),
	TACNA_MIXER_ROUTES("AOBRIDGE2IN5", "AOBRIDGE2IN5"),
	TACNA_MIXER_ROUTES("AOBRIDGE2IN6", "AOBRIDGE2IN6"),
	TACNA_MIXER_ROUTES("AOBRIDGE2IN7", "AOBRIDGE2IN7"),
	TACNA_MIXER_ROUTES("AOBRIDGE2IN8", "AOBRIDGE2IN8"),

	TACNA_DSP_ROUTES_1_12("DSP1"),

	{ "DSP Trigger Out", NULL, "DSP1 Trigger Output" },

	{ "DSP1 Trigger Output", "Switch", "DSP1" },

	{ "AEC1 Loopback", "OUT1L", "OUT1L PGA" },
	{ "AEC1 Loopback", "OUT1R", "OUT1R PGA" },
	{ "AEC2 Loopback", "OUT1L", "OUT1L PGA" },
	{ "AEC2 Loopback", "OUT1R", "OUT1R PGA" },
	{ "OUT1L Output Select", "OUT1", "OUT1L PGA" },
	{ "OUT1R Output Select", "OUT1", "OUT1R PGA" },
	{ "OUT1 Demux", NULL, "OUT1L Output Select" },
	{ "OUT1 Demux", NULL, "OUT1R Output Select" },
	{ "OUT1L_HP1", "HP1", "OUT1 Demux" },
	{ "OUT1R_HP1", "HP1", "OUT1 Demux" },
	{ "OUT1L_HP2", "HP2", "OUT1 Demux" },
	{ "OUT1R_HP2", "HP2", "OUT1 Demux" },

	{ "OUT1L AUX Mix", "Switch", "OUTAUX1L PGA"},
	{ "OUT1R AUX Mix", "Switch", "OUTAUX1R PGA"},
	{ "OUT1L DSD Mix", "Switch", "DSD Processor"},
	{ "OUT1R DSD Mix", "Switch", "DSD Processor"},
	{ "OUT1L PGA", NULL, "OUT1L AUX Mix" },
	{ "OUT1R PGA", NULL, "OUT1R AUX Mix" },
	{ "OUT1L PGA", NULL, "OUT1L DSD Mix" },
	{ "OUT1R PGA", NULL, "OUT1R DSD Mix" },

	{ "AEC1 Loopback", "OUT5L", "OUT5L PGA" },
	{ "AEC1 Loopback", "OUT5R", "OUT5R PGA" },
	{ "AEC2 Loopback", "OUT5L", "OUT5L PGA" },
	{ "AEC2 Loopback", "OUT5R", "OUT5R PGA" },
	{ "OUT5L Source", "OUT5", "OUT5L PGA" },
	{ "OUT5R Source", "OUT5", "OUT5R PGA" },
	/*
	 * OUT1 source comes from the output select widget so we don't power-up
	 * OUT1 path when OUTH is selected
	 */
	{ "OUT5L Source", "OUT1", "OUT1L Output Select" },
	{ "OUT5R Source", "OUT1", "OUT1R Output Select" },
	{ "OUT5_PDMCLK", NULL, "OUT5L Source" },
	{ "OUT5_PDMDATA", NULL, "OUT5L Source" },
	{ "OUT5_PDMCLK", NULL, "OUT5R Source" },
	{ "OUT5_PDMDATA", NULL, "OUT5R Source" },

	{ "DSD Processor Source", "DoP", "OUTH PCM" },
	{ "DSD Processor", "Switch", "DSD Processor Source" },

	{ "OUTH Output Select", "OUTH", "OUTH PCM" },
	{ "OUTH Output Select", "OUTH", "DSD Processor" },
	{ "OUT1L_HP1", NULL, "OUTH Output Select" },
	{ "OUT1R_HP1", NULL, "OUTH Output Select" },

	{ "OUTHL AUX Mix", "Switch", "OUTAUX1L PGA"},
	{ "OUTHR AUX Mix", "Switch", "OUTAUX1R PGA"},
	{ "OUTH Output Select", "OUTH", "OUTHL AUX Mix" },
	{ "OUTH Output Select", "OUTH", "OUTHR AUX Mix" },

	{ "AUXPDM1 Input", "IN1L", "IN1L PGA" },
	{ "AUXPDM1 Input", "IN1R", "IN1R PGA" },
	{ "AUXPDM1 Input", "IN2L", "IN2L PGA" },
	{ "AUXPDM1 Input", "IN2R", "IN2R PGA" },
	{ "AUXPDM1 Input", "IN3L", "IN3L PGA" },
	{ "AUXPDM1 Input", "IN3R", "IN3R PGA" },
	{ "AUXPDM1 Input", "IN4L", "IN4L PGA" },
	{ "AUXPDM1 Input", "IN4R", "IN4R PGA" },

	{ "AUXPDM2 Input", "IN1L", "IN1L PGA" },
	{ "AUXPDM2 Input", "IN1R", "IN1R PGA" },
	{ "AUXPDM2 Input", "IN2L", "IN2L PGA" },
	{ "AUXPDM2 Input", "IN2R", "IN2R PGA" },
	{ "AUXPDM2 Input", "IN3L", "IN3L PGA" },
	{ "AUXPDM2 Input", "IN3R", "IN3R PGA" },
	{ "AUXPDM2 Input", "IN4L", "IN4L PGA" },
	{ "AUXPDM2 Input", "IN4R", "IN4R PGA" },

	{ "AUXPDM3 Input", "IN1L", "IN1L PGA" },
	{ "AUXPDM3 Input", "IN1R", "IN1R PGA" },
	{ "AUXPDM3 Input", "IN2L", "IN2L PGA" },
	{ "AUXPDM3 Input", "IN2R", "IN2R PGA" },
	{ "AUXPDM3 Input", "IN3L", "IN3L PGA" },
	{ "AUXPDM3 Input", "IN3R", "IN3R PGA" },
	{ "AUXPDM3 Input", "IN4L", "IN4L PGA" },
	{ "AUXPDM3 Input", "IN4R", "IN4R PGA" },

	{ "AUXPDM1 Output", "Switch", "AUXPDM1 Input" },
	{ "AUXPDM1_CLK", NULL, "AUXPDM1 Output" },
	{ "AUXPDM1_DOUT", NULL, "AUXPDM1 Output" },

	{ "AUXPDM2 Output", "Switch", "AUXPDM2 Input" },
	{ "AUXPDM2_CLK", NULL, "AUXPDM2 Output" },
	{ "AUXPDM2_DOUT", NULL, "AUXPDM2 Output" },

	{ "AUXPDM3 Output", "Switch", "AUXPDM3 Input" },
	{ "AUXPDM3_CLK", NULL, "AUXPDM3 Output" },
	{ "AUXPDM3_DOUT", NULL, "AUXPDM3 Output" },

	{ "MICSUPP", NULL, "SYSCLK" },

	{ "DRC1 Signal Activity", NULL, "DRC1 Activity Output" },
	{ "DRC2 Signal Activity", NULL, "DRC2 Activity Output" },
	{ "DRC1 Activity Output", "Switch", "DRC1L" },
	{ "DRC1 Activity Output", "Switch", "DRC1R" },
	{ "DRC2 Activity Output", "Switch", "DRC2L" },
	{ "DRC2 Activity Output", "Switch", "DRC2R" },

	TACNA_MUX_ROUTES("DFC1", "DFC1"),
	TACNA_MUX_ROUTES("DFC2", "DFC2"),
	TACNA_MUX_ROUTES("DFC3", "DFC3"),
	TACNA_MUX_ROUTES("DFC4", "DFC4"),
	TACNA_MUX_ROUTES("DFC5", "DFC5"),
	TACNA_MUX_ROUTES("DFC6", "DFC6"),
	TACNA_MUX_ROUTES("DFC7", "DFC7"),
	TACNA_MUX_ROUTES("DFC8", "DFC8"),

	CS47L96_ANC_INPUT_ROUTES("ANCL", "ANCL"),

	CS47L96_ANC_OUTPUT_ROUTES("OUT1L PGA", "OUT1L"),
	CS47L96_ANC_OUTPUT_ROUTES("OUT1R PGA", "OUT1R"),
	CS47L96_ANC_OUTPUT_ROUTES("OUT5L PGA", "OUT5L"),
	CS47L96_ANC_OUTPUT_ROUTES("OUT5R PGA", "OUT5R"),
};

static const struct snd_soc_dapm_route cs47l96_out1_mono_routes[] = {
	{ "OUT1R PGA", NULL, "OUT1L PGA" },
};

static const struct tacna_mono_route cs47l96_mono_routes[] = {
	{
		.routes = cs47l96_out1_mono_routes,
		.n_routes = ARRAY_SIZE(cs47l96_out1_mono_routes),
		/* default demux position - initialize mono bit from pdata */
		.cfg_reg = TACNA_OUT1L_CONTROL_1,
	},
	{
		.routes = cs47l96_out1_mono_routes,
		.n_routes = ARRAY_SIZE(cs47l96_out1_mono_routes),
		/* non-default - demux handles mono bit */
		.cfg_reg = 0,
	},
};

static struct snd_soc_dai_driver cs47l96_dai[] = {
	{
		.name = "cs47l96-asp1",
		.id = 1,
		.base = TACNA_ASP1_ENABLES1,
		.playback = {
			.stream_name = "ASP1 Playback",
			.channels_min = 1,
			.channels_max = 8,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
		.capture = {
			.stream_name = "ASP1 Capture",
			.channels_min = 1,
			.channels_max = 8,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		 },
		.ops = &tacna_dai_ops,
		.symmetric_rates = 1,
		.symmetric_samplebits = 1,
	},
	{
		.name = "cs47l96-asp2",
		.id = 2,
		.base = TACNA_ASP2_ENABLES1,
		.playback = {
			.stream_name = "ASP2 Playback",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
		.capture = {
			.stream_name = "ASP2 Capture",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		 },
		.ops = &tacna_dai_ops,
		.symmetric_rates = 1,
		.symmetric_samplebits = 1,
	},
	{
		.name = "cs47l96-asp3",
		.id = 3,
		.base = TACNA_ASP3_ENABLES1,
		.playback = {
			.stream_name = "ASP3 Playback",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
		.capture = {
			.stream_name = "ASP3 Capture",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		 },
		.ops = &tacna_dai_ops,
		.symmetric_rates = 1,
		.symmetric_samplebits = 1,
	},
	{
		.name = "cs47l96-slim1",
		.id = 4,
		.playback = {
			.stream_name = "Slim1 Playback",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
		.capture = {
			.stream_name = "Slim1 Capture",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		 },
		.ops = &tacna_simple_dai_ops,
	},
	{
		.name = "cs47l96-slim2",
		.id = 5,
		.playback = {
			.stream_name = "Slim2 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
		.capture = {
			.stream_name = "Slim2 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		 },
		.ops = &tacna_simple_dai_ops,
	},
	{
		.name = "cs47l96-slim3",
		.id = 6,
		.playback = {
			.stream_name = "Slim3 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
		.capture = {
			.stream_name = "Slim3 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		 },
		.ops = &tacna_simple_dai_ops,
	},
	{
		.name = "cs47l96-cpu-trace",
		.capture = {
			.stream_name = "Audio Trace CPU",
			.channels_min = 1,
			.channels_max = 6,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
		.compress_new = &snd_soc_new_compress,
	},
	{
		.name = "cs47l96-dsp-trace",
		.capture = {
			.stream_name = "Audio Trace DSP",
			.channels_min = 1,
			.channels_max = 6,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
	},
};

static int cs47l96_init_outh(struct cs47l96 *cs47l96)
{
	struct tacna *tacna = cs47l96->core.tacna;
	int ret;

	ret = regmap_read(tacna->regmap, TACNA_OUTHL_VOLUME_1,
			  &cs47l96->outh_main_vol[0]);
	if (ret) {
		dev_err(cs47l96->core.dev, "Error reading OUTHL volume %d\n",
			ret);
		return ret;
	}
	cs47l96->outh_main_vol[0] &= TACNA_OUTHL_VOL_MASK;

	ret = regmap_read(tacna->regmap, TACNA_OUTHR_VOLUME_1,
			  &cs47l96->outh_main_vol[1]);
	if (ret) {
		dev_err(cs47l96->core.dev, "Error reading OUTHR volume %d\n",
			ret);
		return ret;
	}
	cs47l96->outh_main_vol[1] &= TACNA_OUTHR_VOL_MASK;

	return 0;
}

static int cs47l96_compr_open(struct snd_compr_stream *stream)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct cs47l96 *cs47l96 = snd_soc_platform_get_drvdata(rtd->platform);
	struct tacna_priv *priv = &cs47l96->core;
	int n_dsp;


	if (strcmp(rtd->codec_dai->name, "cs47l96-dsp-trace") == 0) {
		n_dsp = 0;
	} else {
		dev_err(priv->dev,
			"No suitable compressed stream for DAI '%s'\n",
			rtd->codec_dai->name);
		return -EINVAL;
	}

	return wm_adsp_compr_open(&priv->dsp[n_dsp], stream);
}

static irqreturn_t cs47l96_dsp1_irq(int irq, void *data)
{
	struct cs47l96 *cs47l96 = data;
	struct tacna_priv *priv = &cs47l96->core;
	int ret;

	ret = wm_adsp_compr_handle_irq(&priv->dsp[0]);
	if (ret == -ENODEV) {
		dev_err(priv->dev, "Spurious compressed data IRQ\n");
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int cs47l96_codec_probe(struct snd_soc_codec *codec)
{
	struct cs47l96 *cs47l96 = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = cs47l96->core.tacna;
	int ret;

	cs47l96->core.tacna->dapm = snd_soc_codec_get_dapm(codec);

	ret = tacna_init_inputs(codec);
	if (ret)
		return ret;

	ret = tacna_init_auxpdm(codec, CS47L96_N_AUXPDM);
	if (ret)
		return ret;

	BUILD_BUG_ON(ARRAY_SIZE(cs47l96_mono_routes) >
		     ARRAY_SIZE(tacna->pdata.codec.out_mono));
	ret = tacna_init_outputs(codec, cs47l96_mono_routes,
				 ARRAY_SIZE(cs47l96_mono_routes));
	if (ret)
		return ret;

	ret = tacna_init_eq(&cs47l96->core);
	if (ret)
		return ret;

	ret = cs47l96_init_outh(cs47l96);
	if (ret)
		return ret;

	snd_soc_dapm_disable_pin(tacna->dapm, "HAPTICS");

	ret = tacna_dsp_add_codec_controls(codec, CS47L96_NUM_DSP);
	if (ret)
		return ret;

	wm_adsp2_codec_probe(&cs47l96->core.dsp[0], codec);

	return 0;
}

static int cs47l96_codec_remove(struct snd_soc_codec *codec)
{
	struct cs47l96 *cs47l96 = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = cs47l96->core.tacna;

	wm_adsp2_codec_remove(&cs47l96->core.dsp[0], codec);

	tacna->dapm = NULL;

	return 0;
}

static int cs47l96_set_fll(struct snd_soc_codec *codec, int fll_id, int source,
			   unsigned int fref, unsigned int fout)
{
	struct cs47l96 *cs47l96 = snd_soc_codec_get_drvdata(codec);
	int idx;

	switch (fll_id) {
	case TACNA_FLL1_REFCLK:
		idx = 0;
		break;
	case TACNA_FLL2_REFCLK:
		idx = 1;
		break;
	default:
		return -EINVAL;
	}

	return tacna_fllhj_set_refclk(&cs47l96->fll[idx], source, fref, fout);
}

static int cs47l96_set_sysclk(struct snd_soc_codec *codec, int clk_id,
			      int source, unsigned int freq, int dir)
{
	struct cs47l96 *cs47l96 = snd_soc_codec_get_drvdata(codec);

	switch (clk_id) {
	case TACNA_CLK_SYSCLKAO:
		dev_err(cs47l96->core.dev,
			"SYSCLKAO must be set through cs47l96-ao codec\n");
		return -EINVAL;
	default:
		return tacna_set_sysclk(codec, clk_id, source, freq, dir);
	}
}

static struct regmap *cs47l96_get_regmap(struct device *dev)
{
	struct cs47l96 *cs47l96 = dev_get_drvdata(dev);

	return cs47l96->core.tacna->regmap;
}

static const struct snd_soc_codec_driver soc_codec_dev_cs47l96 = {
	.probe = &cs47l96_codec_probe,
	.remove = &cs47l96_codec_remove,
	.get_regmap = &cs47l96_get_regmap,

	.idle_bias_off = true,

	.set_sysclk = &cs47l96_set_sysclk,
	.set_pll = &cs47l96_set_fll,

	.component_driver = {
		.controls = cs47l96_snd_controls,
		.num_controls = ARRAY_SIZE(cs47l96_snd_controls),
		.dapm_widgets = cs47l96_dapm_widgets,
		.num_dapm_widgets = ARRAY_SIZE(cs47l96_dapm_widgets),
		.dapm_routes = cs47l96_dapm_routes,
		.num_dapm_routes = ARRAY_SIZE(cs47l96_dapm_routes),
	},
};

static const struct snd_compr_ops cs47l96_compr_ops = {
	.open = &cs47l96_compr_open,
	.free = &wm_adsp_compr_free,
	.set_params = &wm_adsp_compr_set_params,
	.get_caps = &wm_adsp_compr_get_caps,
	.trigger = &wm_adsp_compr_trigger,
	.pointer = &wm_adsp_compr_pointer,
	.copy = &wm_adsp_compr_copy,
};

static const struct snd_soc_platform_driver cs47l96_compr_platform = {
	.compr_ops = &cs47l96_compr_ops,
};

static const unsigned int cs47l96_out_vu_regs[] = {
	TACNA_OUT1L_VOLUME_1,
	TACNA_OUT1L_VOLUME_3,
	TACNA_OUT1R_VOLUME_1,
	TACNA_OUT1R_VOLUME_3,
	TACNA_OUT5L_VOLUME_1,
	TACNA_OUT5R_VOLUME_1,
	TACNA_OUTHL_VOLUME_1,
	TACNA_OUTHR_VOLUME_1,
	TACNA_OUTAUX1L_VOLUME_1,
	TACNA_OUTAUX1R_VOLUME_1,
};

static int cs47l96_probe(struct platform_device *pdev)
{
	struct tacna *tacna = dev_get_drvdata(pdev->dev.parent);
	struct cs47l96 *cs47l96;
	struct wm_adsp *dsp;
	int i, ret;

	BUILD_BUG_ON(ARRAY_SIZE(cs47l96_dai) > TACNA_MAX_DAI);

	/* quick exit if tacna irqchip driver hasn't completed probe */
	if (!tacna->irq_dev) {
		dev_dbg(&pdev->dev, "irqchip driver not ready\n");
		return -EPROBE_DEFER;
	}

	cs47l96 = devm_kzalloc(&pdev->dev, sizeof(struct cs47l96), GFP_KERNEL);
	if (!cs47l96)
		return -ENOMEM;

	platform_set_drvdata(pdev, cs47l96);
	pdev->dev.of_node = of_node_get(tacna->dev->of_node);

	cs47l96->core.tacna = tacna;
	cs47l96->core.dev = &pdev->dev;
	cs47l96->core.num_inputs = 8;
	cs47l96->core.max_analogue_inputs = 2;
	cs47l96->core.in_vu_reg = TACNA_INPUT_CONTROL3;

	ret = tacna_core_init(&cs47l96->core);
	if (ret)
		return ret;

	init_completion(&cs47l96->outh_enabled);
	init_completion(&cs47l96->outh_disabled);

	ret = tacna_request_irq(tacna, TACNA_IRQ_OUTH_ENABLE_DONE,
				"OUTH enable", cs47l96_outh_enable, cs47l96);
	if (ret)
		dev_warn(&pdev->dev, "Failed to get OUTH enable IRQ: %d\n",
			 ret);

	ret = tacna_request_irq(tacna, TACNA_IRQ_OUTH_DISABLE_DONE,
				"OUTH enable", cs47l96_outh_disable, cs47l96);
	if (ret)
		dev_warn(&pdev->dev, "Failed to get OUTH disable IRQ: %d\n",
			 ret);

	ret = tacna_request_irq(tacna, TACNA_IRQ_DSP1_IRQ0,
				"DSP1 Buffer IRQ", cs47l96_dsp1_irq,
				cs47l96);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to request DSP1_IRQ0: %d\n", ret);
		goto error_dsp1_irq;
	}

	ret = tacna_set_irq_wake(tacna, TACNA_IRQ_DSP1_IRQ0, 1);
	if(ret)
		dev_warn(&pdev->dev, "Failed to set DSP IRQ wake: %d\n", ret);

	dsp = &cs47l96->core.dsp[0];
	dsp->part = "cs47l96";
	dsp->num = 1;
	dsp->type = WMFW_HALO;
	dsp->rev = 0;
	dsp->dev = tacna->dev;
	dsp->regmap = tacna->dsp_regmap[0];

	dsp->base = TACNA_DSP1_CLOCK_FREQ;
	dsp->base_sysinfo = TACNA_DSP1_SYS_INFO_ID;
	dsp->mem = cs47l96_dsp1_regions;
	dsp->num_mems = ARRAY_SIZE(cs47l96_dsp1_regions);

	dsp->n_rx_channels = CS47L96_DSP_N_RX_CHANNELS;
	dsp->n_tx_channels = CS47L96_DSP_N_TX_CHANNELS;

	ret = wm_halo_init(dsp, &cs47l96->core.dsp_fw_lock, 
			   &cs47l96->core.rate_lock);
	if (ret != 0)
		goto error_core;

	ret = tacna_request_irq(tacna, TACNA_IRQ_DSP1_MPU_ERR,
				"DSP1 MPU", cs47l96_mpu_fault_irq,
				&cs47l96->core.dsp[0]);
	if (ret) {
		dev_warn(&pdev->dev, "Failed to get DSP1 MPU IRQ: %d\n", ret);
		goto error_dsp;
	}

	ret = tacna_request_irq(tacna, TACNA_IRQ_DSP1_WDT_EXPIRE,
				"DSP1 WDT", wm_halo_wdt_expire,
				&cs47l96->core.dsp[0]);
	if (ret) {
		dev_warn(&pdev->dev, "Failed to get DSP1 WDT IRQ: %d\n", ret);
		goto error_mpu_irq1;
	}

	for (i = 0; i < CS47L96_N_FLL; ++i) {
		cs47l96->fll[i].tacna_priv = &cs47l96->core;
		cs47l96->fll[i].id = i + 1;
		cs47l96->fll[i].base = TACNA_FLL1_CONTROL1 + (i * 0x100);
		cs47l96->fll[i].sts_addr = TACNA_IRQ1_STS_6;
		cs47l96->fll[i].sts_mask = TACNA_FLL1_LOCK_STS1_MASK << (2 * i);
		tacna_init_fll(&cs47l96->fll[i]);
	}

	for (i = 0; i < ARRAY_SIZE(cs47l96_dai); i++)
		tacna_init_dai(&cs47l96->core, i);

	/* Latch volume update bits */
	for (i = 0; i < ARRAY_SIZE(cs47l96_out_vu_regs); i++)
		regmap_update_bits(tacna->regmap, cs47l96_out_vu_regs[i],
				   TACNA_OUT_VU_MASK, TACNA_OUT_VU);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);

	ret = snd_soc_register_platform(&pdev->dev, &cs47l96_compr_platform);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register platform: %d\n", ret);
		goto error_wdt_irq1;
	}

	ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_cs47l96,
				     cs47l96_dai, ARRAY_SIZE(cs47l96_dai));
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register codec: %d\n", ret);
		goto error_plat;
	}

	return ret;
error_plat:
	snd_soc_unregister_platform(&pdev->dev);
error_wdt_irq1:
	tacna_free_irq(tacna, TACNA_IRQ_DSP1_WDT_EXPIRE,
		       &cs47l96->core.dsp[0]);
error_mpu_irq1:
	tacna_free_irq(tacna, TACNA_IRQ_DSP1_MPU_ERR, &cs47l96->core.dsp[0]);
error_dsp:
	wm_adsp2_remove(&cs47l96->core.dsp[0]);
error_core:
	tacna_set_irq_wake(tacna, TACNA_IRQ_DSP1_IRQ0, 0);
	tacna_free_irq(tacna, TACNA_IRQ_DSP1_IRQ0, cs47l96);
error_dsp1_irq:
	tacna_core_destroy(&cs47l96->core);
	tacna_free_irq(tacna, TACNA_IRQ_OUTH_ENABLE_DONE, cs47l96);
	tacna_free_irq(tacna, TACNA_IRQ_OUTH_DISABLE_DONE, cs47l96);

	return ret;
}

static int cs47l96_remove(struct platform_device *pdev)
{
	struct cs47l96 *cs47l96 = platform_get_drvdata(pdev);
	struct tacna *tacna = cs47l96->core.tacna;

	snd_soc_unregister_platform(&pdev->dev);
	snd_soc_unregister_codec(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	tacna_free_irq(tacna, TACNA_IRQ_DSP1_WDT_EXPIRE,
		       &cs47l96->core.dsp[0]);
	tacna_free_irq(tacna, TACNA_IRQ_DSP1_MPU_ERR, &cs47l96->core.dsp[0]);

	tacna_free_irq(tacna, TACNA_IRQ_OUTH_ENABLE_DONE, cs47l96);
	tacna_free_irq(tacna, TACNA_IRQ_OUTH_DISABLE_DONE, cs47l96);

	tacna_set_irq_wake(tacna, TACNA_IRQ_DSP1_IRQ0, 0);
	tacna_free_irq(tacna, TACNA_IRQ_DSP1_IRQ0, cs47l96);

	wm_adsp2_remove(&cs47l96->core.dsp[0]);

	tacna_core_destroy(&cs47l96->core);

	return 0;
}

static struct platform_driver cs47l96_codec_driver = {
	.driver = {
		.name = "cs47l96-codec",
		.owner = THIS_MODULE,
	},
	.probe = &cs47l96_probe,
	.remove = &cs47l96_remove,
};

module_platform_driver(cs47l96_codec_driver);

MODULE_DESCRIPTION("ASoC CS47L96 driver");
MODULE_AUTHOR("Stuart Henderson <stuarth@opensource.cirrus.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cs47l96-codec");
