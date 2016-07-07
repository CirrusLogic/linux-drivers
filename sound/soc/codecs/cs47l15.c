/*
 * cs47l15.c  --  ALSA SoC Audio driver for CS47L15
 *
 * Copyright 2016 Cirrus Logic
 *
 * Author: Jaswinder Jassal <jjassal@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include <linux/mfd/arizona/core.h>
#include <linux/mfd/arizona/registers.h>

#include "arizona.h"
#include "wm_adsp.h"
#include "cs47l15.h"

#define CS47L15_NUM_ADSP 1

/* Number of compressed DAI hookups, each pair of DSP and dummy CPU
 * are counted as one DAI
 */
#define CS47L15_NUM_COMPR_DAI 1

#define CS47L15_FRF_COEFFICIENT_LEN 4

#define CS47L15_FLL_COUNT 2

/* Mid-mode registers */
#define CS47L15_ADC_INT_BIAS_MASK	0x3800
#define CS47L15_ADC_INT_BIAS_SHIFT	11
#define CS47L15_PGA_BIAS_SEL_MASK	0x03
#define CS47L15_PGA_BIAS_SEL_SHIFT	0

/* 2 mixer inputs with a stride of n in the register address */
#define CS47L15_MIXER_INPUTS_2_N(_reg, n)	\
	(_reg),					\
	(_reg) + (1 * (n))

/* 4 mixer inputs with a stride of n in the register address */
#define CS47L15_MIXER_INPUTS_4_N(_reg, n)		\
	CS47L15_MIXER_INPUTS_2_N(_reg, n),		\
	CS47L15_MIXER_INPUTS_2_N(_reg + (2 * n), n)

#define CS47L15_DSP_MIXER_INPUTS(_reg) \
	CS47L15_MIXER_INPUTS_4_N(_reg, 2),		\
	CS47L15_MIXER_INPUTS_4_N(_reg + 8, 2),	\
	CS47L15_MIXER_INPUTS_4_N(_reg + 16, 8),	\
	CS47L15_MIXER_INPUTS_2_N(_reg + 48, 8)

static const int cs47l15_fx_inputs[] = {
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_EQ1MIX_INPUT_1_SOURCE, 2),
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_EQ2MIX_INPUT_1_SOURCE, 2),
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_EQ3MIX_INPUT_1_SOURCE, 2),
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_EQ4MIX_INPUT_1_SOURCE, 2),
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_DRC1LMIX_INPUT_1_SOURCE, 2),
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_DRC1RMIX_INPUT_1_SOURCE, 2),
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_DRC2LMIX_INPUT_1_SOURCE, 2),
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_DRC2RMIX_INPUT_1_SOURCE, 2),
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_HPLP1MIX_INPUT_1_SOURCE, 2),
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_HPLP2MIX_INPUT_1_SOURCE, 2),
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_HPLP3MIX_INPUT_1_SOURCE, 2),
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_HPLP4MIX_INPUT_1_SOURCE, 2),
};

static const int cs47l15_isrc1_fsl_inputs[] = {
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_ISRC1INT1MIX_INPUT_1_SOURCE, 8),
};

static const int cs47l15_isrc1_fsh_inputs[] = {
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_ISRC1DEC1MIX_INPUT_1_SOURCE, 8),
};

static const int cs47l15_isrc2_fsl_inputs[] = {
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_ISRC2INT1MIX_INPUT_1_SOURCE, 8),
};

static const int cs47l15_isrc2_fsh_inputs[] = {
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_ISRC2DEC1MIX_INPUT_1_SOURCE, 8),
};

static const int cs47l15_out_inputs[] = {
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_OUT1LMIX_INPUT_1_SOURCE, 2),
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_OUT1RMIX_INPUT_1_SOURCE, 2),
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_OUT4LMIX_INPUT_1_SOURCE, 2),
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_OUT5LMIX_INPUT_1_SOURCE, 2),
	CS47L15_MIXER_INPUTS_4_N(ARIZONA_OUT5RMIX_INPUT_1_SOURCE, 2),
};

static const int cs47l15_spd1_inputs[] = {
	CS47L15_MIXER_INPUTS_2_N(ARIZONA_SPDIFTX1MIX_INPUT_1_SOURCE, 8),
};

static const int cs47l15_dsp1_inputs[] = {
	CS47L15_DSP_MIXER_INPUTS(ARIZONA_DSP1LMIX_INPUT_1_SOURCE),
};

static int cs47l15_rate_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

#define CS47L15_RATE_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,\
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_get_enum_double, .put = cs47l15_rate_put, \
	.private_value = (unsigned long)&xenum }

struct cs47l15_priv;

struct cs47l15_compr {
	struct wm_adsp_compr adsp_compr;
	const char *dai_name;
	struct cs47l15_priv *priv;
};

struct cs47l15_priv {
	struct arizona_priv core;
	struct arizona_fll fll[CS47L15_FLL_COUNT];
	struct cs47l15_compr compr_info[CS47L15_NUM_COMPR_DAI];

	bool trig;
	struct mutex trig_lock;
	struct mutex fw_lock;
	bool in1_lp_mode;
};

static const struct {
	const char *dai_name;
	int adsp_num;
} compr_dai_mapping[CS47L15_NUM_COMPR_DAI] = {
	{
		.dai_name = "cs47l15-dsp-trace",
		.adsp_num = 0,
	},
};

static const struct wm_adsp_region cs47l15_dsp1_regions[] = {
	{ .type = WMFW_ADSP2_PM, .base = 0x080000 },
	{ .type = WMFW_ADSP2_ZM, .base = 0x0e0000 },
	{ .type = WMFW_ADSP2_XM, .base = 0x0a0000 },
	{ .type = WMFW_ADSP2_YM, .base = 0x0c0000 },
};

static const char * const cs47l15_inmux_texts[] = {
	"A",
	"B",
};

static int cs47l15_in1mux_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct cs47l15_priv *cs47l15 = snd_soc_codec_get_drvdata(codec);
	struct arizona *arizona = cs47l15->core.arizona;
	struct soc_enum *e = (struct soc_enum *) kcontrol->private_value;
	unsigned int mux, inmode;
	unsigned int mode_val, src_val;
	bool changed = false;
	int ret;

	mux = ucontrol->value.enumerated.item[0];
	if (mux > 1)
		return -EINVAL;

	/* L and R registers have same shift and mask */
	inmode = arizona->pdata.inmode[2 * mux];
	src_val = mux << ARIZONA_IN1L_SRC_SHIFT;
	if (inmode & ARIZONA_INMODE_SE)
		src_val |= 1 << ARIZONA_IN1L_SRC_SE_SHIFT;

	switch (arizona->pdata.inmode[0]) {
	case ARIZONA_INMODE_DMIC:
		if (mux)
			mode_val = 0; /* B always analogue */
		else
			mode_val = 1 << ARIZONA_IN1_MODE_SHIFT;

		ret = snd_soc_update_bits(codec,
					  ARIZONA_IN1L_CONTROL,
					  ARIZONA_IN1_MODE_MASK,
					  mode_val);
		if (ret < 0)
			return ret;
		else if (ret)
			changed = true;

		/* IN1A is digital so L and R must change together */
		/* src_val setting same for both registers */
		ret = snd_soc_update_bits(codec,
					  ARIZONA_ADC_DIGITAL_VOLUME_1L,
					  ARIZONA_IN1L_SRC_MASK |
					  ARIZONA_IN1L_SRC_SE_MASK,
					  src_val);
		if (ret < 0)
			return ret;
		else if (ret)
			changed = true;

		ret = snd_soc_update_bits(codec,
					  ARIZONA_ADC_DIGITAL_VOLUME_1R,
					  ARIZONA_IN1R_SRC_MASK |
					  ARIZONA_IN1R_SRC_SE_MASK,
					  src_val);

		if (ret < 0)
			return ret;
		else if (ret)
			changed = true;
		break;
	default:
		/* both analogue */
		ret = snd_soc_update_bits(codec,
					  e->reg,
					  ARIZONA_IN1L_SRC_MASK |
					  ARIZONA_IN1L_SRC_SE_MASK,
					  src_val);
		if (ret < 0)
			return ret;
		else if (ret)
			changed = true;
		break;
	}

	if (changed)
		return snd_soc_dapm_mux_update_power(widget, kcontrol,
						     mux, e);
	else
		return 0;
}

static const SOC_ENUM_SINGLE_DECL(cs47l15_in1muxl_enum,
			    ARIZONA_ADC_DIGITAL_VOLUME_1L,
			    ARIZONA_IN1L_SRC_SHIFT,
			    cs47l15_inmux_texts);

static const SOC_ENUM_SINGLE_DECL(cs47l15_in1muxr_enum,
			    ARIZONA_ADC_DIGITAL_VOLUME_1R,
			    ARIZONA_IN1R_SRC_SHIFT,
			    cs47l15_inmux_texts);

static const struct snd_kcontrol_new cs47l15_in1mux[2] = {
	SOC_DAPM_ENUM_EXT("IN1L Mux", cs47l15_in1muxl_enum,
			  snd_soc_dapm_get_enum_double, cs47l15_in1mux_put),
	SOC_DAPM_ENUM_EXT("IN1R Mux", cs47l15_in1muxr_enum,
			  snd_soc_dapm_get_enum_double, cs47l15_in1mux_put),
};

static const char * const cs47l15_outdemux_texts[] = {
	"HPOUT",
	"EPOUT",
};

static int cs47l15_put_demux(struct snd_kcontrol *kcontrol,
		     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct snd_soc_card *card = codec->card;
	struct arizona *arizona = dev_get_drvdata(codec->dev->parent);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int ep_sel, mux, change;
	unsigned int mask;
	int ret, demux_change_ret;
	bool restore_out = true, out_mono;

	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;
	mux = ucontrol->value.enumerated.item[0];
	ep_sel = mux << e->shift_l;
	mask = e->mask << e->shift_l;

	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_RUNTIME);

	change = snd_soc_test_bits(codec, e->reg, mask, ep_sel);
	/* if no change is required, skip */
	if (!change)
		goto end;

	/* EP_SEL and OUT1_MONO should not be modified while HP or EP driver
	 * is enabled
	 */
	ret = regmap_update_bits(arizona->regmap,
				 ARIZONA_OUTPUT_ENABLES_1,
				 ARIZONA_OUT1L_ENA |
				 ARIZONA_OUT1R_ENA, 0);
	if (ret)
		dev_warn(arizona->dev,
			 "Failed to disable outputs: %d\n", ret);

	usleep_range(2000, 3000); /* wait for wseq to complete */

	/* [1] if HP detection clamp is applied while switching to HPOUT, OUT1
	 * should remain disabled
	 */
	if (!ep_sel && (arizona->hpdet_clamp ||
			(arizona->hp_impedance_x100 <=
			 OHM_TO_HOHM(arizona->pdata.hpdet_short_circuit_imp))))
		restore_out = false;

	/* change demux setting */
	demux_change_ret = regmap_update_bits(arizona->regmap,
					      ARIZONA_OUTPUT_ENABLES_1,
					      ARIZONA_EP_SEL, ep_sel);
	if (demux_change_ret) {
		dev_err(arizona->dev, "Failed to set EP_SEL: %d\n",
			demux_change_ret);
	} else { /* provided the switch to HP/EP was successful, update output
		    mode accordingly */
		/* when switching to stereo headphone */
		if (!ep_sel && !arizona->pdata.out_mono[0])
			out_mono = false;
		/* when switching to mono headphone, or any earpiece */
		else
			out_mono = true;

		ret = arizona_set_output_mode(codec, 1, out_mono);
		if (ret < 0)
			dev_warn(arizona->dev,
				 "Failed to set output mode: %d\n", ret);
	}

	/* restore outputs to the desired state, or keep them disabled provided
	 * condition [1] arose
	 */
	if (restore_out) {
		ret = regmap_update_bits(arizona->regmap,
					 ARIZONA_OUTPUT_ENABLES_1,
					 ARIZONA_OUT1L_ENA |
					 ARIZONA_OUT1R_ENA,
					 arizona->hp_ena);
		if (ret) {
			dev_warn(arizona->dev,
				 "Failed to restore outputs: %d\n", ret);
		} else {
			/* wait for wseq */
			if (arizona->hp_ena)
				msleep(34); /* enable delay */
			else
				usleep_range(2000, 3000); /* disable delay */
		}
	}

end:
	mutex_unlock(&card->dapm_mutex);

	return snd_soc_dapm_put_enum_virt(kcontrol, ucontrol);
}

static const SOC_ENUM_SINGLE_DECL(cs47l15_outdemux_enum,
			    ARIZONA_OUTPUT_ENABLES_1,
			    ARIZONA_EP_SEL_SHIFT,
			    cs47l15_outdemux_texts);

static const struct snd_kcontrol_new cs47l15_outdemux =
	SOC_DAPM_ENUM_EXT("HPOUT1 Demux", cs47l15_outdemux_enum,
			snd_soc_dapm_get_enum_double, cs47l15_put_demux);

/* Allow the worst case number of sources (FX Rate currently) */
static unsigned int mixer_sources_cache[ARRAY_SIZE(cs47l15_fx_inputs)];

static int cs47l15_get_sources(unsigned int reg,
				const int **cur_sources, int *lim)
{
	int ret = 0;

	switch (reg) {
	case ARIZONA_FX_CTRL1:
		*cur_sources = cs47l15_fx_inputs;
		*lim = ARRAY_SIZE(cs47l15_fx_inputs);
		break;
	case ARIZONA_ISRC_1_CTRL_1:
		*cur_sources = cs47l15_isrc1_fsh_inputs;
		*lim = ARRAY_SIZE(cs47l15_isrc1_fsh_inputs);
		break;
	case ARIZONA_ISRC_1_CTRL_2:
		*cur_sources = cs47l15_isrc1_fsl_inputs;
		*lim = ARRAY_SIZE(cs47l15_isrc1_fsl_inputs);
		break;
	case ARIZONA_ISRC_2_CTRL_1:
		*cur_sources = cs47l15_isrc2_fsh_inputs;
		*lim = ARRAY_SIZE(cs47l15_isrc2_fsh_inputs);
		break;
	case ARIZONA_ISRC_2_CTRL_2:
		*cur_sources = cs47l15_isrc2_fsl_inputs;
		*lim = ARRAY_SIZE(cs47l15_isrc2_fsl_inputs);
		break;
	case ARIZONA_OUTPUT_RATE_1:
		*cur_sources = cs47l15_out_inputs;
		*lim = ARRAY_SIZE(cs47l15_out_inputs);
		break;
	case ARIZONA_SPD1_TX_CONTROL:
		*cur_sources = cs47l15_spd1_inputs;
		*lim = ARRAY_SIZE(cs47l15_spd1_inputs);
		break;
	case CLEARWATER_DSP1_CONFIG:
		*cur_sources = cs47l15_dsp1_inputs;
		*lim = ARRAY_SIZE(cs47l15_dsp1_inputs);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int cs47l15_rate_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int ret, err;
	int lim;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	struct cs47l15_priv *cs47l15 = snd_soc_codec_get_drvdata(codec);
	struct arizona_priv *priv = &cs47l15->core;
	struct arizona *arizona = priv->arizona;

	const int *cur_sources;

	unsigned int val, cur;
	unsigned int mask;

	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;

	val = e->values[ucontrol->value.enumerated.item[0]] << e->shift_l;
	mask = e->mask << e->shift_l;

	ret = regmap_read(arizona->regmap, e->reg, &cur);
	if (ret != 0) {
		dev_err(arizona->dev, "Failed to read current reg: %d\n", ret);
		return ret;
	}

	if ((cur & mask) == (val & mask))
		return 0;

	ret = cs47l15_get_sources((int)e->reg, &cur_sources, &lim);
	if (ret != 0) {
		dev_err(arizona->dev, "Failed to get sources for 0x%08x: %d\n",
			e->reg,
			ret);
		return ret;
	}

	mutex_lock(&arizona->rate_lock);

	ret = arizona_cache_and_clear_sources(arizona, cur_sources,
					      mixer_sources_cache, lim);
	if (ret != 0) {
		dev_err(arizona->dev,
			"%s Failed to cache and clear sources %d\n",
			__func__,
			ret);
		goto out;
	}

	/* Apply the rate through the original callback */
	clearwater_spin_sysclk(arizona);
	ret = snd_soc_update_bits_locked(codec, e->reg, mask, val);
	clearwater_spin_sysclk(arizona);

out:
	err = arizona_restore_sources(arizona, cur_sources,
				      mixer_sources_cache, lim);
	if (err != 0) {
		dev_err(arizona->dev,
			"%s Failed to restore sources %d\n",
			__func__,
			err);
	}

	mutex_unlock(&arizona->rate_lock);
	return ret;
}

static int cs47l15_adsp_rate_put_cb(struct wm_adsp *adsp,
				       unsigned int mask,
				       unsigned int val)
{
	int ret, err;
	int lim;
	const int *cur_sources;
	struct arizona *arizona = dev_get_drvdata(adsp->dev);
	unsigned int cur;

	ret = regmap_read(adsp->regmap, adsp->base, &cur);
	if (ret != 0) {
		dev_err(arizona->dev, "Failed to read current: %d\n", ret);
		return ret;
	}

	if ((val & mask) == (cur & mask))
		return 0;

	ret = cs47l15_get_sources(adsp->base, &cur_sources, &lim);
	if (ret != 0) {
		dev_err(arizona->dev, "Failed to get sources for 0x%08x: %d\n",
			adsp->base,
			ret);
		return ret;
	}

	dev_dbg(arizona->dev, "%s for DSP%d\n", __func__, adsp->num);

	mutex_lock(&arizona->rate_lock);

	ret = arizona_cache_and_clear_sources(arizona, cur_sources,
					      mixer_sources_cache, lim);

	if (ret != 0) {
		dev_err(arizona->dev,
			"%s Failed to cache and clear sources %d\n",
			__func__,
			ret);
		goto out;
	}

	clearwater_spin_sysclk(arizona);
	/* Apply the rate */
	ret = regmap_update_bits(adsp->regmap, adsp->base, mask, val);
	clearwater_spin_sysclk(arizona);

out:
	err = arizona_restore_sources(arizona, cur_sources,
				      mixer_sources_cache, lim);

	if (err != 0) {
		dev_err(arizona->dev,
			"%s Failed to restore sources %d\n",
			__func__,
			err);
	}

	mutex_unlock(&arizona->rate_lock);
	return ret;
}

static int cs47l15_sysclk_ev(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol,
			     int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct cs47l15_priv *cs47l15 = snd_soc_codec_get_drvdata(codec);
	struct arizona_priv *priv = &cs47l15->core;
	struct arizona *arizona = priv->arizona;

	clearwater_spin_sysclk(arizona);

	return 0;
}

static int cs47l15_adsp_power_ev(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol,
				    int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct cs47l15_priv *cs47l15 = snd_soc_codec_get_drvdata(codec);
	struct arizona_priv *priv = &cs47l15->core;
	struct arizona *arizona = priv->arizona;
	unsigned int freq;
	int ret;

	ret = regmap_read(arizona->regmap, CLEARWATER_DSP_CLOCK_2, &freq);
	if (ret != 0) {
		dev_err(arizona->dev,
			"Failed to read CLEARWATER_DSP_CLOCK_2: %d\n", ret);
		return ret;
	}

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mutex_lock(&cs47l15->trig_lock);
		cs47l15->trig = false;
		mutex_unlock(&cs47l15->trig_lock);
		break;
	default:
		break;
	}

	return wm_adsp2_early_event(w, kcontrol, event, freq);
}

static DECLARE_TLV_DB_SCALE(ana_tlv, 0, 100, 0);
static DECLARE_TLV_DB_SCALE(eq_tlv, -1200, 100, 0);
static DECLARE_TLV_DB_SCALE(digital_tlv, -6400, 50, 0);
static DECLARE_TLV_DB_SCALE(noise_tlv, -13200, 600, 0);
static DECLARE_TLV_DB_SCALE(ng_tlv, -12000, 600, 0);

#define CS47L15_NG_SRC(name, base) \
	SOC_SINGLE(name " NG HPOUT1L Switch",  base,  0, 1, 0), \
	SOC_SINGLE(name " NG HPOUT1R Switch",  base,  1, 1, 0), \
	SOC_SINGLE(name " NG SPKOUTL Switch",   base,  6, 1, 0), \
	SOC_SINGLE(name " NG SPKDAT1L Switch", base,  8, 1, 0), \
	SOC_SINGLE(name " NG SPKDAT1R Switch", base,  9, 1, 0)

static int cs47l15_in1_adc_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct cs47l15_priv *cs47l15 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = cs47l15->in1_lp_mode ? 1 : 0;

	return 0;
}

static int cs47l15_in1_adc_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct cs47l15_priv *cs47l15 = snd_soc_codec_get_drvdata(codec);

	switch (ucontrol->value.integer.value[0]) {
	case 0:
		/* Set IN1 to normal mode */
		snd_soc_update_bits(codec, ARIZONA_DMIC1L_CONTROL,
				    CLEARWATER_IN1_OSR_MASK,
				    5 << CLEARWATER_IN1_OSR_SHIFT);
		snd_soc_update_bits(codec, CS47L15_ADC_INT_BIAS,
				    CS47L15_ADC_INT_BIAS_MASK,
				    4 << CS47L15_ADC_INT_BIAS_SHIFT);
		snd_soc_update_bits(codec, CS47L15_PGA_BIAS_SEL,
				    CS47L15_PGA_BIAS_SEL_MASK,
				    0);
		cs47l15->in1_lp_mode = false;
		break;
	default:
		/* Set IN1 to LP mode */
		snd_soc_update_bits(codec, ARIZONA_DMIC1L_CONTROL,
				    CLEARWATER_IN1_OSR_MASK,
				    4 << CLEARWATER_IN1_OSR_SHIFT);
		snd_soc_update_bits(codec, CS47L15_ADC_INT_BIAS,
				    CS47L15_ADC_INT_BIAS_MASK,
				    1 << CS47L15_ADC_INT_BIAS_SHIFT);
		snd_soc_update_bits(codec, CS47L15_PGA_BIAS_SEL,
				    CS47L15_PGA_BIAS_SEL_MASK,
				    3 << CS47L15_PGA_BIAS_SEL_SHIFT);
		cs47l15->in1_lp_mode = true;
		break;
	}

	return 0;
}

static const struct snd_kcontrol_new cs47l15_snd_controls[] = {
SOC_VALUE_ENUM("IN1 OSR", clearwater_in_dmic_osr[0]),
SOC_VALUE_ENUM("IN2 OSR", clearwater_in_dmic_osr[1]),

SOC_SINGLE_RANGE_TLV("IN1L Volume", ARIZONA_IN1L_CONTROL,
		     ARIZONA_IN1L_PGA_VOL_SHIFT, 0x40, 0x5f, 0, ana_tlv),
SOC_SINGLE_RANGE_TLV("IN1R Volume", ARIZONA_IN1R_CONTROL,
		     ARIZONA_IN1R_PGA_VOL_SHIFT, 0x40, 0x5f, 0, ana_tlv),

SOC_ENUM("IN HPF Cutoff Frequency", arizona_in_hpf_cut_enum),

SOC_SINGLE("IN1L HPF Switch", ARIZONA_IN1L_CONTROL,
	   ARIZONA_IN1L_HPF_SHIFT, 1, 0),
SOC_SINGLE("IN1R HPF Switch", ARIZONA_IN1R_CONTROL,
	   ARIZONA_IN1R_HPF_SHIFT, 1, 0),
SOC_SINGLE("IN2L HPF Switch", ARIZONA_IN2L_CONTROL,
	   ARIZONA_IN2L_HPF_SHIFT, 1, 0),
SOC_SINGLE("IN2R HPF Switch", ARIZONA_IN2R_CONTROL,
	   ARIZONA_IN2R_HPF_SHIFT, 1, 0),

SOC_SINGLE_TLV("IN1L Digital Volume", ARIZONA_ADC_DIGITAL_VOLUME_1L,
	       ARIZONA_IN1L_DIG_VOL_SHIFT, 0xbf, 0, digital_tlv),
SOC_SINGLE_TLV("IN1R Digital Volume", ARIZONA_ADC_DIGITAL_VOLUME_1R,
	       ARIZONA_IN1R_DIG_VOL_SHIFT, 0xbf, 0, digital_tlv),
SOC_SINGLE_TLV("IN2L Digital Volume", ARIZONA_ADC_DIGITAL_VOLUME_2L,
	       ARIZONA_IN2L_DIG_VOL_SHIFT, 0xbf, 0, digital_tlv),
SOC_SINGLE_TLV("IN2R Digital Volume", ARIZONA_ADC_DIGITAL_VOLUME_2R,
	       ARIZONA_IN2R_DIG_VOL_SHIFT, 0xbf, 0, digital_tlv),

SOC_ENUM("Input Ramp Up", arizona_in_vi_ramp),
SOC_ENUM("Input Ramp Down", arizona_in_vd_ramp),

SND_SOC_BYTES("FRF COEFF 1L", CLEARWATER_FRF_COEFFICIENT_1L_1,
				 CS47L15_FRF_COEFFICIENT_LEN),
SND_SOC_BYTES("FRF COEFF 1R", CLEARWATER_FRF_COEFFICIENT_1R_1,
				 CS47L15_FRF_COEFFICIENT_LEN),
SND_SOC_BYTES("FRF COEFF 4L", CLEARWATER_FRF_COEFFICIENT_4L_1,
				 CS47L15_FRF_COEFFICIENT_LEN),
SND_SOC_BYTES("FRF COEFF 5L", CLEARWATER_FRF_COEFFICIENT_5L_1,
				 CS47L15_FRF_COEFFICIENT_LEN),
SND_SOC_BYTES("FRF COEFF 5R", CLEARWATER_FRF_COEFFICIENT_5R_1,
				 CS47L15_FRF_COEFFICIENT_LEN),

SND_SOC_BYTES("DAC COMP 1", CLEARWATER_DAC_COMP_1, 1),
SND_SOC_BYTES("DAC COMP 2", CLEARWATER_DAC_COMP_2, 1),

ARIZONA_MIXER_CONTROLS("EQ1", ARIZONA_EQ1MIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("EQ2", ARIZONA_EQ2MIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("EQ3", ARIZONA_EQ3MIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("EQ4", ARIZONA_EQ4MIX_INPUT_1_SOURCE),

ARIZONA_EQ_CONTROL("EQ1 Coefficients", ARIZONA_EQ1_2),
SOC_SINGLE_TLV("EQ1 B1 Volume", ARIZONA_EQ1_1, ARIZONA_EQ1_B1_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ1 B2 Volume", ARIZONA_EQ1_1, ARIZONA_EQ1_B2_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ1 B3 Volume", ARIZONA_EQ1_1, ARIZONA_EQ1_B3_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ1 B4 Volume", ARIZONA_EQ1_2, ARIZONA_EQ1_B4_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ1 B5 Volume", ARIZONA_EQ1_2, ARIZONA_EQ1_B5_GAIN_SHIFT,
	       24, 0, eq_tlv),

ARIZONA_EQ_CONTROL("EQ2 Coefficients", ARIZONA_EQ2_2),
SOC_SINGLE_TLV("EQ2 B1 Volume", ARIZONA_EQ2_1, ARIZONA_EQ2_B1_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ2 B2 Volume", ARIZONA_EQ2_1, ARIZONA_EQ2_B2_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ2 B3 Volume", ARIZONA_EQ2_1, ARIZONA_EQ2_B3_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ2 B4 Volume", ARIZONA_EQ2_2, ARIZONA_EQ2_B4_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ2 B5 Volume", ARIZONA_EQ2_2, ARIZONA_EQ2_B5_GAIN_SHIFT,
	       24, 0, eq_tlv),

ARIZONA_EQ_CONTROL("EQ3 Coefficients", ARIZONA_EQ3_2),
SOC_SINGLE_TLV("EQ3 B1 Volume", ARIZONA_EQ3_1, ARIZONA_EQ3_B1_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ3 B2 Volume", ARIZONA_EQ3_1, ARIZONA_EQ3_B2_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ3 B3 Volume", ARIZONA_EQ3_1, ARIZONA_EQ3_B3_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ3 B4 Volume", ARIZONA_EQ3_2, ARIZONA_EQ3_B4_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ3 B5 Volume", ARIZONA_EQ3_2, ARIZONA_EQ3_B5_GAIN_SHIFT,
	       24, 0, eq_tlv),

ARIZONA_EQ_CONTROL("EQ4 Coefficients", ARIZONA_EQ4_2),
SOC_SINGLE_TLV("EQ4 B1 Volume", ARIZONA_EQ4_1, ARIZONA_EQ4_B1_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ4 B2 Volume", ARIZONA_EQ4_1, ARIZONA_EQ4_B2_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ4 B3 Volume", ARIZONA_EQ4_1, ARIZONA_EQ4_B3_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ4 B4 Volume", ARIZONA_EQ4_2, ARIZONA_EQ4_B4_GAIN_SHIFT,
	       24, 0, eq_tlv),
SOC_SINGLE_TLV("EQ4 B5 Volume", ARIZONA_EQ4_2, ARIZONA_EQ4_B5_GAIN_SHIFT,
	       24, 0, eq_tlv),

ARIZONA_MIXER_CONTROLS("DRC1L", ARIZONA_DRC1LMIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("DRC1R", ARIZONA_DRC1RMIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("DRC2L", ARIZONA_DRC2LMIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("DRC2R", ARIZONA_DRC2RMIX_INPUT_1_SOURCE),

SND_SOC_BYTES_MASK("DRC1", ARIZONA_DRC1_CTRL1, 5,
		   ARIZONA_DRC1R_ENA | ARIZONA_DRC1L_ENA),
SND_SOC_BYTES_MASK("DRC2", CLEARWATER_DRC2_CTRL1, 5,
		   ARIZONA_DRC2R_ENA | ARIZONA_DRC2L_ENA),

ARIZONA_MIXER_CONTROLS("LHPF1", ARIZONA_HPLP1MIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("LHPF2", ARIZONA_HPLP2MIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("LHPF3", ARIZONA_HPLP3MIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("LHPF4", ARIZONA_HPLP4MIX_INPUT_1_SOURCE),

SND_SOC_BYTES("LHPF1 Coefficients", ARIZONA_HPLPF1_2, 1),
SND_SOC_BYTES("LHPF2 Coefficients", ARIZONA_HPLPF2_2, 1),
SND_SOC_BYTES("LHPF3 Coefficients", ARIZONA_HPLPF3_2, 1),
SND_SOC_BYTES("LHPF4 Coefficients", ARIZONA_HPLPF4_2, 1),

SOC_ENUM("LHPF1 Mode", arizona_lhpf1_mode),
SOC_ENUM("LHPF2 Mode", arizona_lhpf2_mode),
SOC_ENUM("LHPF3 Mode", arizona_lhpf3_mode),
SOC_ENUM("LHPF4 Mode", arizona_lhpf4_mode),

SOC_VALUE_ENUM("Sample Rate 2", arizona_sample_rate[0]),
SOC_VALUE_ENUM("Sample Rate 3", arizona_sample_rate[1]),

CS47L15_RATE_ENUM("FX Rate", arizona_fx_rate),

CS47L15_RATE_ENUM("ISRC1 FSL", arizona_isrc_fsl[0]),
CS47L15_RATE_ENUM("ISRC2 FSL", arizona_isrc_fsl[1]),
CS47L15_RATE_ENUM("ISRC1 FSH", arizona_isrc_fsh[0]),
CS47L15_RATE_ENUM("ISRC2 FSH", arizona_isrc_fsh[1]),

ARIZONA_MIXER_CONTROLS("DSP1L", ARIZONA_DSP1LMIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("DSP1R", ARIZONA_DSP1RMIX_INPUT_1_SOURCE),

SOC_SINGLE_TLV("Noise Generator Volume", CLEARWATER_COMFORT_NOISE_GENERATOR,
	       CLEARWATER_NOISE_GEN_GAIN_SHIFT, 0x16, 0, noise_tlv),

ARIZONA_MIXER_CONTROLS("HPOUT1L", ARIZONA_OUT1LMIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("HPOUT1R", ARIZONA_OUT1RMIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("SPKOUTL", ARIZONA_OUT4LMIX_INPUT_1_SOURCE),

ARIZONA_MIXER_CONTROLS("SPKDAT1L", ARIZONA_OUT5LMIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("SPKDAT1R", ARIZONA_OUT5RMIX_INPUT_1_SOURCE),

SOC_SINGLE("HPOUT1 SC Protect Switch", ARIZONA_HP1_SHORT_CIRCUIT_CTRL,
	   ARIZONA_HP1_SC_ENA_SHIFT, 1, 0),

SOC_SINGLE("SPKDAT1 High Performance Switch", ARIZONA_OUTPUT_PATH_CONFIG_5L,
	   ARIZONA_OUT5_OSR_SHIFT, 1, 0),

SOC_DOUBLE_R("HPOUT1 Digital Switch", ARIZONA_DAC_DIGITAL_VOLUME_1L,
	     ARIZONA_DAC_DIGITAL_VOLUME_1R, ARIZONA_OUT1L_MUTE_SHIFT, 1, 1),
SOC_DOUBLE_R("SPKDAT1 Digital Switch", ARIZONA_DAC_DIGITAL_VOLUME_5L,
	     ARIZONA_DAC_DIGITAL_VOLUME_5R, ARIZONA_OUT5L_MUTE_SHIFT, 1, 1),

SOC_SINGLE("Speaker Digital Switch", ARIZONA_DAC_DIGITAL_VOLUME_4L,
	   ARIZONA_OUT4L_MUTE_SHIFT, 1, 1),

SOC_DOUBLE_R_TLV("HPOUT1 Digital Volume", ARIZONA_DAC_DIGITAL_VOLUME_1L,
		 ARIZONA_DAC_DIGITAL_VOLUME_1R, ARIZONA_OUT1L_VOL_SHIFT,
		 0xbf, 0, digital_tlv),

SOC_SINGLE_TLV("Speaker Digital Volume", ARIZONA_DAC_DIGITAL_VOLUME_4L,
	       ARIZONA_OUT4L_VOL_SHIFT, 0xbf, 0, digital_tlv),

SOC_DOUBLE_R_TLV("SPKDAT1 Digital Volume", ARIZONA_DAC_DIGITAL_VOLUME_5L,
		 ARIZONA_DAC_DIGITAL_VOLUME_5R, ARIZONA_OUT5L_VOL_SHIFT,
		 0xbf, 0, digital_tlv),

SOC_DOUBLE("SPKDAT1 Switch", ARIZONA_PDM_SPK1_CTRL_1, ARIZONA_SPK1L_MUTE_SHIFT,
	   ARIZONA_SPK1R_MUTE_SHIFT, 1, 1),

SOC_DOUBLE_EXT("HPOUT1 DRE Switch", ARIZONA_DRE_ENABLE,
	   VEGAS_DRE1L_ENA_SHIFT, VEGAS_DRE1R_ENA_SHIFT, 1, 0,
	   snd_soc_get_volsw, clearwater_put_dre),
SOC_DOUBLE("HPOUT1 EDRE Switch", CLEARWATER_EDRE_ENABLE,
	   CLEARWATER_EDRE_OUT1L_THR1_ENA_SHIFT,
	   CLEARWATER_EDRE_OUT1R_THR1_ENA_SHIFT, 1, 0),

SOC_SINGLE("Speaker THR1 EDRE Switch", CLEARWATER_EDRE_ENABLE,
	   CLEARWATER_EDRE_OUT4L_THR1_ENA_SHIFT, 1, 0),

SOC_ENUM("Output Ramp Up", arizona_out_vi_ramp),
SOC_ENUM("Output Ramp Down", arizona_out_vd_ramp),

CS47L15_RATE_ENUM("SPDIF Rate", arizona_spdif_rate),

SOC_SINGLE("Noise Gate Switch", ARIZONA_NOISE_GATE_CONTROL,
	   ARIZONA_NGATE_ENA_SHIFT, 1, 0),
SOC_SINGLE_TLV("Noise Gate Threshold Volume", ARIZONA_NOISE_GATE_CONTROL,
	       ARIZONA_NGATE_THR_SHIFT, 7, 1, ng_tlv),
SOC_ENUM("Noise Gate Hold", arizona_ng_hold),

CS47L15_RATE_ENUM("Output Rate 1", arizona_output_rate),

SOC_ENUM_EXT("IN1L Rate", moon_input_rate[0],
	snd_soc_get_enum_double, moon_in_rate_put),
SOC_ENUM_EXT("IN1R Rate", moon_input_rate[1],
	snd_soc_get_enum_double, moon_in_rate_put),

SOC_SINGLE_BOOL_EXT("IN1 LP Mode Switch", 0,
	     cs47l15_in1_adc_get, cs47l15_in1_adc_put),

SOC_ENUM_EXT("IN2L Rate", moon_input_rate[2],
	snd_soc_get_enum_double, moon_in_rate_put),
SOC_ENUM_EXT("IN2R Rate", moon_input_rate[3],
	snd_soc_get_enum_double, moon_in_rate_put),

CS47L15_NG_SRC("HPOUT1L", ARIZONA_NOISE_GATE_SELECT_1L),
CS47L15_NG_SRC("HPOUT1R", ARIZONA_NOISE_GATE_SELECT_1R),

CS47L15_NG_SRC("SPKOUTL", ARIZONA_NOISE_GATE_SELECT_4L),

CS47L15_NG_SRC("SPKDAT1L", ARIZONA_NOISE_GATE_SELECT_5L),
CS47L15_NG_SRC("SPKDAT1R", ARIZONA_NOISE_GATE_SELECT_5R),

ARIZONA_MIXER_CONTROLS("AIF1TX1", ARIZONA_AIF1TX1MIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("AIF1TX2", ARIZONA_AIF1TX2MIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("AIF1TX3", ARIZONA_AIF1TX3MIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("AIF1TX4", ARIZONA_AIF1TX4MIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("AIF1TX5", ARIZONA_AIF1TX5MIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("AIF1TX6", ARIZONA_AIF1TX6MIX_INPUT_1_SOURCE),

ARIZONA_MIXER_CONTROLS("AIF2TX1", ARIZONA_AIF2TX1MIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("AIF2TX2", ARIZONA_AIF2TX2MIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("AIF2TX3", ARIZONA_AIF2TX3MIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("AIF2TX4", ARIZONA_AIF2TX4MIX_INPUT_1_SOURCE),

ARIZONA_MIXER_CONTROLS("AIF3TX1", ARIZONA_AIF3TX1MIX_INPUT_1_SOURCE),
ARIZONA_MIXER_CONTROLS("AIF3TX2", ARIZONA_AIF3TX2MIX_INPUT_1_SOURCE),

ARIZONA_GAINMUX_CONTROLS("SPDIFTX1", ARIZONA_SPDIFTX1MIX_INPUT_1_SOURCE),
ARIZONA_GAINMUX_CONTROLS("SPDIFTX2", ARIZONA_SPDIFTX2MIX_INPUT_1_SOURCE),
};

CLEARWATER_MIXER_ENUMS(EQ1, ARIZONA_EQ1MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(EQ2, ARIZONA_EQ2MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(EQ3, ARIZONA_EQ3MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(EQ4, ARIZONA_EQ4MIX_INPUT_1_SOURCE);

CLEARWATER_MIXER_ENUMS(DRC1L, ARIZONA_DRC1LMIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(DRC1R, ARIZONA_DRC1RMIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(DRC2L, ARIZONA_DRC2LMIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(DRC2R, ARIZONA_DRC2RMIX_INPUT_1_SOURCE);

CLEARWATER_MIXER_ENUMS(LHPF1, ARIZONA_HPLP1MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(LHPF2, ARIZONA_HPLP2MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(LHPF3, ARIZONA_HPLP3MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(LHPF4, ARIZONA_HPLP4MIX_INPUT_1_SOURCE);

CLEARWATER_MIXER_ENUMS(DSP1L, ARIZONA_DSP1LMIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(DSP1R, ARIZONA_DSP1RMIX_INPUT_1_SOURCE);
CLEARWATER_DSP_AUX_ENUMS(DSP1, ARIZONA_DSP1AUX1MIX_INPUT_1_SOURCE);

CLEARWATER_MIXER_ENUMS(PWM1, ARIZONA_PWM1MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(PWM2, ARIZONA_PWM2MIX_INPUT_1_SOURCE);

CLEARWATER_MIXER_ENUMS(OUT1L, ARIZONA_OUT1LMIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(OUT1R, ARIZONA_OUT1RMIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(SPKOUTL, ARIZONA_OUT4LMIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(SPKDAT1L, ARIZONA_OUT5LMIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(SPKDAT1R, ARIZONA_OUT5RMIX_INPUT_1_SOURCE);

CLEARWATER_MIXER_ENUMS(AIF1TX1, ARIZONA_AIF1TX1MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(AIF1TX2, ARIZONA_AIF1TX2MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(AIF1TX3, ARIZONA_AIF1TX3MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(AIF1TX4, ARIZONA_AIF1TX4MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(AIF1TX5, ARIZONA_AIF1TX5MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(AIF1TX6, ARIZONA_AIF1TX6MIX_INPUT_1_SOURCE);

CLEARWATER_MIXER_ENUMS(AIF2TX1, ARIZONA_AIF2TX1MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(AIF2TX2, ARIZONA_AIF2TX2MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(AIF2TX3, ARIZONA_AIF2TX3MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(AIF2TX4, ARIZONA_AIF2TX4MIX_INPUT_1_SOURCE);

CLEARWATER_MIXER_ENUMS(AIF3TX1, ARIZONA_AIF3TX1MIX_INPUT_1_SOURCE);
CLEARWATER_MIXER_ENUMS(AIF3TX2, ARIZONA_AIF3TX2MIX_INPUT_1_SOURCE);

CLEARWATER_MUX_ENUMS(SPD1TX1, ARIZONA_SPDIFTX1MIX_INPUT_1_SOURCE);
CLEARWATER_MUX_ENUMS(SPD1TX2, ARIZONA_SPDIFTX2MIX_INPUT_1_SOURCE);

CLEARWATER_MUX_ENUMS(ISRC1INT1, ARIZONA_ISRC1INT1MIX_INPUT_1_SOURCE);
CLEARWATER_MUX_ENUMS(ISRC1INT2, ARIZONA_ISRC1INT2MIX_INPUT_1_SOURCE);
CLEARWATER_MUX_ENUMS(ISRC1INT3, ARIZONA_ISRC1INT3MIX_INPUT_1_SOURCE);
CLEARWATER_MUX_ENUMS(ISRC1INT4, ARIZONA_ISRC1INT4MIX_INPUT_1_SOURCE);

CLEARWATER_MUX_ENUMS(ISRC1DEC1, ARIZONA_ISRC1DEC1MIX_INPUT_1_SOURCE);
CLEARWATER_MUX_ENUMS(ISRC1DEC2, ARIZONA_ISRC1DEC2MIX_INPUT_1_SOURCE);
CLEARWATER_MUX_ENUMS(ISRC1DEC3, ARIZONA_ISRC1DEC3MIX_INPUT_1_SOURCE);
CLEARWATER_MUX_ENUMS(ISRC1DEC4, ARIZONA_ISRC1DEC4MIX_INPUT_1_SOURCE);

CLEARWATER_MUX_ENUMS(ISRC2INT1, ARIZONA_ISRC2INT1MIX_INPUT_1_SOURCE);
CLEARWATER_MUX_ENUMS(ISRC2INT2, ARIZONA_ISRC2INT2MIX_INPUT_1_SOURCE);
CLEARWATER_MUX_ENUMS(ISRC2INT3, ARIZONA_ISRC2INT3MIX_INPUT_1_SOURCE);
CLEARWATER_MUX_ENUMS(ISRC2INT4, ARIZONA_ISRC2INT4MIX_INPUT_1_SOURCE);

CLEARWATER_MUX_ENUMS(ISRC2DEC1, ARIZONA_ISRC2DEC1MIX_INPUT_1_SOURCE);
CLEARWATER_MUX_ENUMS(ISRC2DEC2, ARIZONA_ISRC2DEC2MIX_INPUT_1_SOURCE);
CLEARWATER_MUX_ENUMS(ISRC2DEC3, ARIZONA_ISRC2DEC3MIX_INPUT_1_SOURCE);
CLEARWATER_MUX_ENUMS(ISRC2DEC4, ARIZONA_ISRC2DEC4MIX_INPUT_1_SOURCE);

static const char * const cs47l15_dsp_output_texts[] = {
	"None",
	"DSP1",
};

static const struct soc_enum cs47l15_dsp_output_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(cs47l15_dsp_output_texts),
			cs47l15_dsp_output_texts);

static const struct snd_kcontrol_new cs47l15_dsp_output_mux[] = {
	SOC_DAPM_ENUM_VIRT("DSP Virtual Output Mux", cs47l15_dsp_output_enum),
};

static const char * const cs47l15_aec_loopback_texts[] = {
	"HPOUT1L", "HPOUT1R", "SPKOUTL", "SPKDAT1L", "SPKDAT1R",
};

static const unsigned int cs47l15_aec_loopback_values[] = {
	0, 1, 6, 8, 9,
};

static const struct soc_enum cs47l15_aec_loopback =
	SOC_VALUE_ENUM_SINGLE(ARIZONA_DAC_AEC_CONTROL_1,
			      ARIZONA_AEC_LOOPBACK_SRC_SHIFT, 0xf,
			      ARRAY_SIZE(cs47l15_aec_loopback_texts),
			      cs47l15_aec_loopback_texts,
			      cs47l15_aec_loopback_values);

static const struct snd_kcontrol_new cs47l15_aec_loopback_mux =
	SOC_DAPM_VALUE_ENUM("AEC Loopback", cs47l15_aec_loopback);

static const struct snd_soc_dapm_widget cs47l15_dapm_widgets[] = {
SND_SOC_DAPM_SUPPLY("SYSCLK", ARIZONA_SYSTEM_CLOCK_1, ARIZONA_SYSCLK_ENA_SHIFT,
		    0, cs47l15_sysclk_ev,
		    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_SUPPLY("OPCLK", ARIZONA_OUTPUT_SYSTEM_CLOCK,
		    ARIZONA_OPCLK_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_SUPPLY("DSPCLK", CLEARWATER_DSP_CLOCK_1, 6,
		    0, NULL, 0),

SND_SOC_DAPM_REGULATOR_SUPPLY("CPVDD", 20, 0),
SND_SOC_DAPM_REGULATOR_SUPPLY("MICVDD", 0, SND_SOC_DAPM_REGULATOR_BYPASS),
SND_SOC_DAPM_REGULATOR_SUPPLY("SPKVDD", 0, 0),

SND_SOC_DAPM_SIGGEN("TONE"),
SND_SOC_DAPM_SIGGEN("NOISE"),

SND_SOC_DAPM_INPUT("IN1AL"),
SND_SOC_DAPM_INPUT("IN1BL"),
SND_SOC_DAPM_INPUT("IN1AR"),
SND_SOC_DAPM_INPUT("IN1BR"),
SND_SOC_DAPM_INPUT("IN2L"),
SND_SOC_DAPM_INPUT("IN2R"),

SND_SOC_DAPM_MUX("IN1L Mux", SND_SOC_NOPM, 0, 0, &cs47l15_in1mux[0]),
SND_SOC_DAPM_MUX("IN1R Mux", SND_SOC_NOPM, 0, 0, &cs47l15_in1mux[1]),

SND_SOC_DAPM_DEMUX("HPOUT1 Demux", SND_SOC_NOPM, 0, 0, &cs47l15_outdemux),

SND_SOC_DAPM_OUTPUT("DRC1 Signal Activity"),
SND_SOC_DAPM_OUTPUT("DRC2 Signal Activity"),

SND_SOC_DAPM_OUTPUT("DSP Virtual Output"),

SND_SOC_DAPM_SUPPLY("MICBIAS1", ARIZONA_MIC_BIAS_CTRL_1,
		    ARIZONA_MICB1_ENA_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_SUPPLY("MICBIAS1A", ARIZONA_MIC_BIAS_CTRL_5,
			ARIZONA_MICB1A_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_SUPPLY("MICBIAS1B", ARIZONA_MIC_BIAS_CTRL_5,
			ARIZONA_MICB1B_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_SUPPLY("MICBIAS1C", ARIZONA_MIC_BIAS_CTRL_5,
			ARIZONA_MICB1C_ENA_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_PGA("PWM1 Driver", ARIZONA_PWM_DRIVE_1, ARIZONA_PWM1_ENA_SHIFT,
		 0, NULL, 0),
SND_SOC_DAPM_PGA("PWM2 Driver", ARIZONA_PWM_DRIVE_1, ARIZONA_PWM2_ENA_SHIFT,
		 0, NULL, 0),

SND_SOC_DAPM_AIF_OUT("AIF1TX1", NULL, 0,
		     ARIZONA_AIF1_TX_ENABLES, ARIZONA_AIF1TX1_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("AIF1TX2", NULL, 0,
		     ARIZONA_AIF1_TX_ENABLES, ARIZONA_AIF1TX2_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("AIF1TX3", NULL, 0,
		     ARIZONA_AIF1_TX_ENABLES, ARIZONA_AIF1TX3_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("AIF1TX4", NULL, 0,
		     ARIZONA_AIF1_TX_ENABLES, ARIZONA_AIF1TX4_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("AIF1TX5", NULL, 0,
		     ARIZONA_AIF1_TX_ENABLES, ARIZONA_AIF1TX5_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("AIF1TX6", NULL, 0,
		     ARIZONA_AIF1_TX_ENABLES, ARIZONA_AIF1TX6_ENA_SHIFT, 0),

SND_SOC_DAPM_AIF_OUT("AIF2TX1", NULL, 0,
		     ARIZONA_AIF2_TX_ENABLES, ARIZONA_AIF2TX1_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("AIF2TX2", NULL, 0,
		     ARIZONA_AIF2_TX_ENABLES, ARIZONA_AIF2TX2_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("AIF2TX3", NULL, 0,
		     ARIZONA_AIF2_TX_ENABLES, ARIZONA_AIF2TX3_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("AIF2TX4", NULL, 0,
		     ARIZONA_AIF2_TX_ENABLES, ARIZONA_AIF2TX4_ENA_SHIFT, 0),

SND_SOC_DAPM_AIF_OUT("AIF3TX1", NULL, 0,
		     ARIZONA_AIF3_TX_ENABLES, ARIZONA_AIF3TX1_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("AIF3TX2", NULL, 0,
		     ARIZONA_AIF3_TX_ENABLES, ARIZONA_AIF3TX2_ENA_SHIFT, 0),

SND_SOC_DAPM_PGA_E("OUT1L", SND_SOC_NOPM,
		   ARIZONA_OUT1L_ENA_SHIFT, 0, NULL, 0, arizona_hp_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("OUT1R", SND_SOC_NOPM,
		   ARIZONA_OUT1R_ENA_SHIFT, 0, NULL, 0, arizona_hp_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),

SND_SOC_DAPM_PGA_E("OUT5L", ARIZONA_OUTPUT_ENABLES_1,
		   ARIZONA_OUT5L_ENA_SHIFT, 0, NULL, 0, arizona_out_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("OUT5R", ARIZONA_OUTPUT_ENABLES_1,
		   ARIZONA_OUT5R_ENA_SHIFT, 0, NULL, 0, arizona_out_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),

SND_SOC_DAPM_PGA("SPD1TX1", ARIZONA_SPD1_TX_CONTROL,
		   ARIZONA_SPD1_VAL1_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("SPD1TX2", ARIZONA_SPD1_TX_CONTROL,
		   ARIZONA_SPD1_VAL2_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_OUT_DRV("SPD1", ARIZONA_SPD1_TX_CONTROL,
		     ARIZONA_SPD1_ENA_SHIFT, 0, NULL, 0),

/* mux_in widgets : arranged in the order of sources
   specified in ARIZONA_MIXER_INPUT_ROUTES */

SND_SOC_DAPM_PGA("Noise Generator", CLEARWATER_COMFORT_NOISE_GENERATOR,
		 CLEARWATER_NOISE_GEN_ENA_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_PGA("Tone Generator 1", ARIZONA_TONE_GENERATOR_1,
		 ARIZONA_TONE1_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("Tone Generator 2", ARIZONA_TONE_GENERATOR_1,
		 ARIZONA_TONE2_ENA_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_MIC("HAPTICS", NULL),

SND_SOC_DAPM_MUX("AEC Loopback", ARIZONA_DAC_AEC_CONTROL_1,
		       ARIZONA_AEC_LOOPBACK_ENA_SHIFT, 0,
		       &cs47l15_aec_loopback_mux),

SND_SOC_DAPM_PGA_E("IN1L PGA", ARIZONA_INPUT_ENABLES, ARIZONA_IN1L_ENA_SHIFT,
		   0, NULL, 0, arizona_in_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("IN1R PGA", ARIZONA_INPUT_ENABLES, ARIZONA_IN1R_ENA_SHIFT,
		   0, NULL, 0, arizona_in_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("IN2L PGA", ARIZONA_INPUT_ENABLES, ARIZONA_IN2L_ENA_SHIFT,
		   0, NULL, 0, arizona_in_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("IN2R PGA", ARIZONA_INPUT_ENABLES, ARIZONA_IN2R_ENA_SHIFT,
		   0, NULL, 0, arizona_in_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),

SND_SOC_DAPM_AIF_IN("AIF1RX1", NULL, 0,
			ARIZONA_AIF1_RX_ENABLES, ARIZONA_AIF1RX1_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("AIF1RX2", NULL, 0,
			ARIZONA_AIF1_RX_ENABLES, ARIZONA_AIF1RX2_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("AIF1RX3", NULL, 0,
			ARIZONA_AIF1_RX_ENABLES, ARIZONA_AIF1RX3_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("AIF1RX4", NULL, 0,
			ARIZONA_AIF1_RX_ENABLES, ARIZONA_AIF1RX4_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("AIF1RX5", NULL, 0,
			ARIZONA_AIF1_RX_ENABLES, ARIZONA_AIF1RX5_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("AIF1RX6", NULL, 0,
			ARIZONA_AIF1_RX_ENABLES, ARIZONA_AIF1RX6_ENA_SHIFT, 0),

SND_SOC_DAPM_AIF_IN("AIF2RX1", NULL, 0,
			ARIZONA_AIF2_RX_ENABLES, ARIZONA_AIF2RX1_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("AIF2RX2", NULL, 0,
			ARIZONA_AIF2_RX_ENABLES, ARIZONA_AIF2RX2_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("AIF2RX3", NULL, 0,
			ARIZONA_AIF2_RX_ENABLES, ARIZONA_AIF2RX3_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("AIF2RX4", NULL, 0,
			ARIZONA_AIF2_RX_ENABLES, ARIZONA_AIF2RX4_ENA_SHIFT, 0),

SND_SOC_DAPM_AIF_IN("AIF3RX1", NULL, 0,
			ARIZONA_AIF3_RX_ENABLES, ARIZONA_AIF3RX1_ENA_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("AIF3RX2", NULL, 0,
			ARIZONA_AIF3_RX_ENABLES, ARIZONA_AIF3RX2_ENA_SHIFT, 0),

SND_SOC_DAPM_PGA("EQ1", ARIZONA_EQ1_1, ARIZONA_EQ1_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("EQ2", ARIZONA_EQ2_1, ARIZONA_EQ2_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("EQ3", ARIZONA_EQ3_1, ARIZONA_EQ3_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("EQ4", ARIZONA_EQ4_1, ARIZONA_EQ4_ENA_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_PGA("DRC1L", ARIZONA_DRC1_CTRL1, ARIZONA_DRC1L_ENA_SHIFT, 0,
		 NULL, 0),
SND_SOC_DAPM_PGA("DRC1R", ARIZONA_DRC1_CTRL1, ARIZONA_DRC1R_ENA_SHIFT, 0,
		 NULL, 0),
SND_SOC_DAPM_PGA("DRC2L", CLEARWATER_DRC2_CTRL1, ARIZONA_DRC2L_ENA_SHIFT, 0,
		 NULL, 0),
SND_SOC_DAPM_PGA("DRC2R", CLEARWATER_DRC2_CTRL1, ARIZONA_DRC2R_ENA_SHIFT, 0,
		 NULL, 0),

SND_SOC_DAPM_PGA("LHPF1", ARIZONA_HPLPF1_1, ARIZONA_LHPF1_ENA_SHIFT, 0,
		 NULL, 0),
SND_SOC_DAPM_PGA("LHPF2", ARIZONA_HPLPF2_1, ARIZONA_LHPF2_ENA_SHIFT, 0,
		 NULL, 0),
SND_SOC_DAPM_PGA("LHPF3", ARIZONA_HPLPF3_1, ARIZONA_LHPF3_ENA_SHIFT, 0,
		 NULL, 0),
SND_SOC_DAPM_PGA("LHPF4", ARIZONA_HPLPF4_1, ARIZONA_LHPF4_ENA_SHIFT, 0,
		 NULL, 0),

SND_SOC_DAPM_PGA("ISRC1DEC1", ARIZONA_ISRC_1_CTRL_3,
		 ARIZONA_ISRC1_DEC0_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC1DEC2", ARIZONA_ISRC_1_CTRL_3,
		 ARIZONA_ISRC1_DEC1_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC1DEC3", ARIZONA_ISRC_1_CTRL_3,
		 ARIZONA_ISRC1_DEC2_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC1DEC4", ARIZONA_ISRC_1_CTRL_3,
		 ARIZONA_ISRC1_DEC3_ENA_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_PGA("ISRC1INT1", ARIZONA_ISRC_1_CTRL_3,
		 ARIZONA_ISRC1_INT0_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC1INT2", ARIZONA_ISRC_1_CTRL_3,
		 ARIZONA_ISRC1_INT1_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC1INT3", ARIZONA_ISRC_1_CTRL_3,
		 ARIZONA_ISRC1_INT2_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC1INT4", ARIZONA_ISRC_1_CTRL_3,
		 ARIZONA_ISRC1_INT3_ENA_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_PGA("ISRC2DEC1", ARIZONA_ISRC_2_CTRL_3,
		 ARIZONA_ISRC2_DEC0_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC2DEC2", ARIZONA_ISRC_2_CTRL_3,
		 ARIZONA_ISRC2_DEC1_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC2DEC3", ARIZONA_ISRC_2_CTRL_3,
		 ARIZONA_ISRC2_DEC2_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC2DEC4", ARIZONA_ISRC_2_CTRL_3,
		 ARIZONA_ISRC2_DEC3_ENA_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_PGA("ISRC2INT1", ARIZONA_ISRC_2_CTRL_3,
		 ARIZONA_ISRC2_INT0_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC2INT2", ARIZONA_ISRC_2_CTRL_3,
		 ARIZONA_ISRC2_INT1_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC2INT3", ARIZONA_ISRC_2_CTRL_3,
		 ARIZONA_ISRC2_INT2_ENA_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("ISRC2INT4", ARIZONA_ISRC_2_CTRL_3,
		 ARIZONA_ISRC2_INT3_ENA_SHIFT, 0, NULL, 0),

WM_ADSP2("DSP1", 0, cs47l15_adsp_power_ev),

ARIZONA_MIXER_WIDGETS(EQ1, "EQ1"),
ARIZONA_MIXER_WIDGETS(EQ2, "EQ2"),
ARIZONA_MIXER_WIDGETS(EQ3, "EQ3"),
ARIZONA_MIXER_WIDGETS(EQ4, "EQ4"),

ARIZONA_MIXER_WIDGETS(DRC1L, "DRC1L"),
ARIZONA_MIXER_WIDGETS(DRC1R, "DRC1R"),
ARIZONA_MIXER_WIDGETS(DRC2L, "DRC2L"),
ARIZONA_MIXER_WIDGETS(DRC2R, "DRC2R"),

ARIZONA_MIXER_WIDGETS(LHPF1, "LHPF1"),
ARIZONA_MIXER_WIDGETS(LHPF2, "LHPF2"),
ARIZONA_MIXER_WIDGETS(LHPF3, "LHPF3"),
ARIZONA_MIXER_WIDGETS(LHPF4, "LHPF4"),

ARIZONA_MIXER_WIDGETS(PWM1, "PWM1"),
ARIZONA_MIXER_WIDGETS(PWM2, "PWM2"),

ARIZONA_MIXER_WIDGETS(OUT1L, "HPOUT1L"),
ARIZONA_MIXER_WIDGETS(OUT1R, "HPOUT1R"),

ARIZONA_MIXER_WIDGETS(SPKOUTL, "SPKOUTL"),

ARIZONA_MIXER_WIDGETS(SPKDAT1L, "SPKDAT1L"),
ARIZONA_MIXER_WIDGETS(SPKDAT1R, "SPKDAT1R"),

ARIZONA_MIXER_WIDGETS(AIF1TX1, "AIF1TX1"),
ARIZONA_MIXER_WIDGETS(AIF1TX2, "AIF1TX2"),
ARIZONA_MIXER_WIDGETS(AIF1TX3, "AIF1TX3"),
ARIZONA_MIXER_WIDGETS(AIF1TX4, "AIF1TX4"),
ARIZONA_MIXER_WIDGETS(AIF1TX5, "AIF1TX5"),
ARIZONA_MIXER_WIDGETS(AIF1TX6, "AIF1TX6"),

ARIZONA_MIXER_WIDGETS(AIF2TX1, "AIF2TX1"),
ARIZONA_MIXER_WIDGETS(AIF2TX2, "AIF2TX2"),
ARIZONA_MIXER_WIDGETS(AIF2TX3, "AIF2TX3"),
ARIZONA_MIXER_WIDGETS(AIF2TX4, "AIF2TX4"),

ARIZONA_MIXER_WIDGETS(AIF3TX1, "AIF3TX1"),
ARIZONA_MIXER_WIDGETS(AIF3TX2, "AIF3TX2"),

ARIZONA_MUX_WIDGETS(SPD1TX1, "SPDIFTX1"),
ARIZONA_MUX_WIDGETS(SPD1TX2, "SPDIFTX2"),

ARIZONA_DSP_WIDGETS(DSP1, "DSP1"),

SND_SOC_DAPM_VIRT_MUX("DSP Virtual Output Mux", SND_SOC_NOPM, 0, 0,
		      &cs47l15_dsp_output_mux[0]),

ARIZONA_MUX_WIDGETS(ISRC1DEC1, "ISRC1DEC1"),
ARIZONA_MUX_WIDGETS(ISRC1DEC2, "ISRC1DEC2"),
ARIZONA_MUX_WIDGETS(ISRC1DEC3, "ISRC1DEC3"),
ARIZONA_MUX_WIDGETS(ISRC1DEC4, "ISRC1DEC4"),

ARIZONA_MUX_WIDGETS(ISRC1INT1, "ISRC1INT1"),
ARIZONA_MUX_WIDGETS(ISRC1INT2, "ISRC1INT2"),
ARIZONA_MUX_WIDGETS(ISRC1INT3, "ISRC1INT3"),
ARIZONA_MUX_WIDGETS(ISRC1INT4, "ISRC1INT4"),

ARIZONA_MUX_WIDGETS(ISRC2DEC1, "ISRC2DEC1"),
ARIZONA_MUX_WIDGETS(ISRC2DEC2, "ISRC2DEC2"),
ARIZONA_MUX_WIDGETS(ISRC2DEC3, "ISRC2DEC3"),
ARIZONA_MUX_WIDGETS(ISRC2DEC4, "ISRC2DEC4"),

ARIZONA_MUX_WIDGETS(ISRC2INT1, "ISRC2INT1"),
ARIZONA_MUX_WIDGETS(ISRC2INT2, "ISRC2INT2"),
ARIZONA_MUX_WIDGETS(ISRC2INT3, "ISRC2INT3"),
ARIZONA_MUX_WIDGETS(ISRC2INT4, "ISRC2INT4"),

SND_SOC_DAPM_OUTPUT("HPOUT1L"),
SND_SOC_DAPM_OUTPUT("HPOUT1R"),

SND_SOC_DAPM_OUTPUT("EPOUTP"),
SND_SOC_DAPM_OUTPUT("EPOUTN"),

SND_SOC_DAPM_OUTPUT("SPKOUTLN"),
SND_SOC_DAPM_OUTPUT("SPKOUTLP"),

SND_SOC_DAPM_OUTPUT("SPKDAT1L"),
SND_SOC_DAPM_OUTPUT("SPKDAT1R"),
SND_SOC_DAPM_OUTPUT("SPDIF"),

SND_SOC_DAPM_OUTPUT("MICSUPP"),
};

#define ARIZONA_MIXER_INPUT_ROUTES(name)	\
	{ name, "Noise Generator", "Noise Generator" }, \
	{ name, "Tone Generator 1", "Tone Generator 1" }, \
	{ name, "Tone Generator 2", "Tone Generator 2" }, \
	{ name, "Haptics", "HAPTICS" }, \
	{ name, "AEC", "AEC Loopback" }, \
	{ name, "IN1L", "IN1L PGA" }, \
	{ name, "IN1R", "IN1R PGA" }, \
	{ name, "IN2L", "IN2L PGA" }, \
	{ name, "IN2R", "IN2R PGA" }, \
	{ name, "AIF1RX1", "AIF1RX1" }, \
	{ name, "AIF1RX2", "AIF1RX2" }, \
	{ name, "AIF1RX3", "AIF1RX3" }, \
	{ name, "AIF1RX4", "AIF1RX4" }, \
	{ name, "AIF1RX5", "AIF1RX5" }, \
	{ name, "AIF1RX6", "AIF1RX6" }, \
	{ name, "AIF2RX1", "AIF2RX1" }, \
	{ name, "AIF2RX2", "AIF2RX2" }, \
	{ name, "AIF2RX3", "AIF2RX3" }, \
	{ name, "AIF2RX4", "AIF2RX4" }, \
	{ name, "AIF3RX1", "AIF3RX1" }, \
	{ name, "AIF3RX2", "AIF3RX2" }, \
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
	{ name, "ISRC1DEC1", "ISRC1DEC1" }, \
	{ name, "ISRC1DEC2", "ISRC1DEC2" }, \
	{ name, "ISRC1DEC3", "ISRC1DEC3" }, \
	{ name, "ISRC1DEC4", "ISRC1DEC4" }, \
	{ name, "ISRC1INT1", "ISRC1INT1" }, \
	{ name, "ISRC1INT2", "ISRC1INT2" }, \
	{ name, "ISRC1INT3", "ISRC1INT3" }, \
	{ name, "ISRC1INT4", "ISRC1INT4" }, \
	{ name, "ISRC2DEC1", "ISRC2DEC1" }, \
	{ name, "ISRC2DEC2", "ISRC2DEC2" }, \
	{ name, "ISRC2DEC3", "ISRC2DEC3" }, \
	{ name, "ISRC2DEC4", "ISRC2DEC4" }, \
	{ name, "ISRC2INT1", "ISRC2INT1" }, \
	{ name, "ISRC2INT2", "ISRC2INT2" }, \
	{ name, "ISRC2INT3", "ISRC2INT3" }, \
	{ name, "ISRC2INT4", "ISRC2INT4" }, \
	{ name, "DSP1.1", "DSP1" }, \
	{ name, "DSP1.2", "DSP1" }, \
	{ name, "DSP1.3", "DSP1" }, \
	{ name, "DSP1.4", "DSP1" }, \
	{ name, "DSP1.5", "DSP1" }, \
	{ name, "DSP1.6", "DSP1" }

static const struct snd_soc_dapm_route cs47l15_dapm_routes[] = {
	{ "OUT1L", NULL, "CPVDD" },
	{ "OUT1R", NULL, "CPVDD" },

	{ "OUT4L", NULL, "SPKVDD" },

	{ "OUT1L", NULL, "SYSCLK" },
	{ "OUT1R", NULL, "SYSCLK" },
	{ "OUT4L", NULL, "SYSCLK" },
	{ "OUT5L", NULL, "SYSCLK" },
	{ "OUT5R", NULL, "SYSCLK" },

	{ "SPD1", NULL, "SYSCLK" },
	{ "SPD1", NULL, "SPD1TX1" },
	{ "SPD1", NULL, "SPD1TX2" },

	{ "IN1AL", NULL, "SYSCLK" },
	{ "IN1BL", NULL, "SYSCLK" },
	{ "IN1AR", NULL, "SYSCLK" },
	{ "IN1BR", NULL, "SYSCLK" },

	{ "IN2L", NULL, "SYSCLK" },
	{ "IN2R", NULL, "SYSCLK" },

	{ "DSP1", NULL, "DSPCLK"},

	{ "MICBIAS1", NULL, "MICVDD" },

	{ "MICBIAS1A", NULL, "MICBIAS1" },
	{ "MICBIAS1B", NULL, "MICBIAS1" },
	{ "MICBIAS1C", NULL, "MICBIAS1" },

	{ "Noise Generator", NULL, "SYSCLK" },
	{ "Tone Generator 1", NULL, "SYSCLK" },
	{ "Tone Generator 2", NULL, "SYSCLK" },

	{ "Noise Generator", NULL, "NOISE" },
	{ "Tone Generator 1", NULL, "TONE" },
	{ "Tone Generator 2", NULL, "TONE" },

	{ "AIF1 Capture", NULL, "AIF1TX1" },
	{ "AIF1 Capture", NULL, "AIF1TX2" },
	{ "AIF1 Capture", NULL, "AIF1TX3" },
	{ "AIF1 Capture", NULL, "AIF1TX4" },
	{ "AIF1 Capture", NULL, "AIF1TX5" },
	{ "AIF1 Capture", NULL, "AIF1TX6" },

	{ "AIF1RX1", NULL, "AIF1 Playback" },
	{ "AIF1RX2", NULL, "AIF1 Playback" },
	{ "AIF1RX3", NULL, "AIF1 Playback" },
	{ "AIF1RX4", NULL, "AIF1 Playback" },
	{ "AIF1RX5", NULL, "AIF1 Playback" },
	{ "AIF1RX6", NULL, "AIF1 Playback" },

	{ "AIF2 Capture", NULL, "AIF2TX1" },
	{ "AIF2 Capture", NULL, "AIF2TX2" },
	{ "AIF2 Capture", NULL, "AIF2TX3" },
	{ "AIF2 Capture", NULL, "AIF2TX4" },

	{ "AIF2RX1", NULL, "AIF2 Playback" },
	{ "AIF2RX2", NULL, "AIF2 Playback" },
	{ "AIF2RX3", NULL, "AIF2 Playback" },
	{ "AIF2RX4", NULL, "AIF2 Playback" },

	{ "AIF3 Capture", NULL, "AIF3TX1" },
	{ "AIF3 Capture", NULL, "AIF3TX2" },

	{ "AIF3RX1", NULL, "AIF3 Playback" },
	{ "AIF3RX2", NULL, "AIF3 Playback" },

	{ "AIF1 Playback", NULL, "SYSCLK" },
	{ "AIF2 Playback", NULL, "SYSCLK" },
	{ "AIF3 Playback", NULL, "SYSCLK" },

	{ "AIF1 Capture", NULL, "SYSCLK" },
	{ "AIF2 Capture", NULL, "SYSCLK" },
	{ "AIF3 Capture", NULL, "SYSCLK" },

	{ "Trace CPU", NULL, "Trace DSP" },
	{ "Trace DSP", NULL, "DSP1" },
	{ "Trace CPU", NULL, "SYSCLK" },
	{ "Trace DSP", NULL, "SYSCLK" },

	{ "IN1L Mux", "A", "IN1AL" },
	{ "IN1L Mux", "B", "IN1BL" },
	{ "IN1R Mux", "A", "IN1AR" },
	{ "IN1R Mux", "B", "IN1BR" },

	{ "IN1L PGA", NULL, "IN1L Mux" },
	{ "IN1R PGA", NULL, "IN1R Mux" },

	{ "IN2L PGA", NULL, "IN2L" },
	{ "IN2R PGA", NULL, "IN2R" },

	ARIZONA_MIXER_ROUTES("OUT1L", "HPOUT1L"),
	ARIZONA_MIXER_ROUTES("OUT1R", "HPOUT1R"),

	ARIZONA_MIXER_ROUTES("OUT4L", "SPKOUTL"),

	ARIZONA_MIXER_ROUTES("OUT5L", "SPKDAT1L"),
	ARIZONA_MIXER_ROUTES("OUT5R", "SPKDAT1R"),

	ARIZONA_MIXER_ROUTES("PWM1 Driver", "PWM1"),
	ARIZONA_MIXER_ROUTES("PWM2 Driver", "PWM2"),

	ARIZONA_MIXER_ROUTES("AIF1TX1", "AIF1TX1"),
	ARIZONA_MIXER_ROUTES("AIF1TX2", "AIF1TX2"),
	ARIZONA_MIXER_ROUTES("AIF1TX3", "AIF1TX3"),
	ARIZONA_MIXER_ROUTES("AIF1TX4", "AIF1TX4"),
	ARIZONA_MIXER_ROUTES("AIF1TX5", "AIF1TX5"),
	ARIZONA_MIXER_ROUTES("AIF1TX6", "AIF1TX6"),

	ARIZONA_MIXER_ROUTES("AIF2TX1", "AIF2TX1"),
	ARIZONA_MIXER_ROUTES("AIF2TX2", "AIF2TX2"),
	ARIZONA_MIXER_ROUTES("AIF2TX3", "AIF2TX3"),
	ARIZONA_MIXER_ROUTES("AIF2TX4", "AIF2TX4"),

	ARIZONA_MIXER_ROUTES("AIF3TX1", "AIF3TX1"),
	ARIZONA_MIXER_ROUTES("AIF3TX2", "AIF3TX2"),

	ARIZONA_MUX_ROUTES("SPD1TX1", "SPDIFTX1"),
	ARIZONA_MUX_ROUTES("SPD1TX2", "SPDIFTX2"),

	ARIZONA_MIXER_ROUTES("EQ1", "EQ1"),
	ARIZONA_MIXER_ROUTES("EQ2", "EQ2"),
	ARIZONA_MIXER_ROUTES("EQ3", "EQ3"),
	ARIZONA_MIXER_ROUTES("EQ4", "EQ4"),

	ARIZONA_MIXER_ROUTES("DRC1L", "DRC1L"),
	ARIZONA_MIXER_ROUTES("DRC1R", "DRC1R"),
	ARIZONA_MIXER_ROUTES("DRC2L", "DRC2L"),
	ARIZONA_MIXER_ROUTES("DRC2R", "DRC2R"),

	ARIZONA_MIXER_ROUTES("LHPF1", "LHPF1"),
	ARIZONA_MIXER_ROUTES("LHPF2", "LHPF2"),
	ARIZONA_MIXER_ROUTES("LHPF3", "LHPF3"),
	ARIZONA_MIXER_ROUTES("LHPF4", "LHPF4"),

	ARIZONA_DSP_ROUTES("DSP1"),

	{ "DSP Virtual Output", NULL, "DSP Virtual Output Mux" },
	{ "DSP Virtual Output Mux", "DSP1", "DSP1" },
	{ "DSP Virtual Output", NULL, "SYSCLK" },

	ARIZONA_MUX_ROUTES("ISRC1INT1", "ISRC1INT1"),
	ARIZONA_MUX_ROUTES("ISRC1INT2", "ISRC1INT2"),
	ARIZONA_MUX_ROUTES("ISRC1INT3", "ISRC1INT3"),
	ARIZONA_MUX_ROUTES("ISRC1INT4", "ISRC1INT4"),

	ARIZONA_MUX_ROUTES("ISRC1DEC1", "ISRC1DEC1"),
	ARIZONA_MUX_ROUTES("ISRC1DEC2", "ISRC1DEC2"),
	ARIZONA_MUX_ROUTES("ISRC1DEC3", "ISRC1DEC3"),
	ARIZONA_MUX_ROUTES("ISRC1DEC4", "ISRC1DEC4"),

	ARIZONA_MUX_ROUTES("ISRC2INT1", "ISRC2INT1"),
	ARIZONA_MUX_ROUTES("ISRC2INT2", "ISRC2INT2"),
	ARIZONA_MUX_ROUTES("ISRC2INT3", "ISRC2INT3"),
	ARIZONA_MUX_ROUTES("ISRC2INT4", "ISRC2INT4"),

	ARIZONA_MUX_ROUTES("ISRC2DEC1", "ISRC2DEC1"),
	ARIZONA_MUX_ROUTES("ISRC2DEC2", "ISRC2DEC2"),
	ARIZONA_MUX_ROUTES("ISRC2DEC3", "ISRC2DEC3"),
	ARIZONA_MUX_ROUTES("ISRC2DEC4", "ISRC2DEC4"),

	{ "AEC Loopback", "HPOUT1L", "OUT1L" },
	{ "AEC Loopback", "HPOUT1R", "OUT1R" },
	{ "HPOUT1 Demux", NULL, "OUT1L" },
	{ "HPOUT1 Demux", NULL, "OUT1R" },


	{ "HPOUT1L", "HPOUT", "HPOUT1 Demux" },
	{ "HPOUT1R", "HPOUT", "HPOUT1 Demux" },
	{ "EPOUTP", "EPOUT", "HPOUT1 Demux" },
	{ "EPOUTN", "EPOUT", "HPOUT1 Demux" },

	{ "AEC Loopback", "SPKOUTL", "OUT4L" },
	{ "SPKOUTLN", NULL, "OUT4L" },
	{ "SPKOUTLP", NULL, "OUT4L" },

	{ "AEC Loopback", "SPKDAT1L", "OUT5L" },
	{ "AEC Loopback", "SPKDAT1R", "OUT5R" },
	{ "SPKDAT1L", NULL, "OUT5L" },
	{ "SPKDAT1R", NULL, "OUT5R" },

	{ "SPDIF", NULL, "SPD1" },

	{ "MICSUPP", NULL, "SYSCLK" },

	{ "DRC1 Signal Activity", NULL, "DRC1L" },
	{ "DRC1 Signal Activity", NULL, "DRC1R" },
	{ "DRC2 Signal Activity", NULL, "DRC2L" },
	{ "DRC2 Signal Activity", NULL, "DRC2R" },
};

static int cs47l15_set_fll(struct snd_soc_codec *codec, int fll_id, int source,
			  unsigned int Fref, unsigned int Fout)
{
	struct cs47l15_priv *cs47l15 = snd_soc_codec_get_drvdata(codec);

	switch (fll_id) {
	case CS47L15_FLL1:
		return arizona_set_fll(&cs47l15->fll[0], source, Fref, Fout);
	case CS47L15_FLLAO:
		return arizona_set_fll_ao(&cs47l15->fll[1], source, Fref, Fout);
	case CS47L15_FLL1_REFCLK:
		return arizona_set_fll_refclk(&cs47l15->fll[0], source, Fref,
					      Fout);
	default:
		return -EINVAL;
	}
}

#define CS47L15_RATES SNDRV_PCM_RATE_KNOT

#define CS47L15_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver cs47l15_dai[] = {
	{
		.name = "cs47l15-aif1",
		.id = 1,
		.base = ARIZONA_AIF1_BCLK_CTRL,
		.playback = {
			.stream_name = "AIF1 Playback",
			.channels_min = 1,
			.channels_max = 6,
			.rates = CS47L15_RATES,
			.formats = CS47L15_FORMATS,
		},
		.capture = {
			 .stream_name = "AIF1 Capture",
			 .channels_min = 1,
			 .channels_max = 6,
			 .rates = CS47L15_RATES,
			 .formats = CS47L15_FORMATS,
		 },
		.ops = &arizona_dai_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "cs47l15-aif2",
		.id = 2,
		.base = ARIZONA_AIF2_BCLK_CTRL,
		.playback = {
			.stream_name = "AIF2 Playback",
			.channels_min = 1,
			.channels_max = 4,
			.rates = CS47L15_RATES,
			.formats = CS47L15_FORMATS,
		},
		.capture = {
			 .stream_name = "AIF2 Capture",
			 .channels_min = 1,
			 .channels_max = 4,
			 .rates = CS47L15_RATES,
			 .formats = CS47L15_FORMATS,
		 },
		.ops = &arizona_dai_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "cs47l15-aif3",
		.id = 3,
		.base = ARIZONA_AIF3_BCLK_CTRL,
		.playback = {
			.stream_name = "AIF3 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = CS47L15_RATES,
			.formats = CS47L15_FORMATS,
		},
		.capture = {
			 .stream_name = "AIF3 Capture",
			 .channels_min = 1,
			 .channels_max = 2,
			 .rates = CS47L15_RATES,
			 .formats = CS47L15_FORMATS,
		 },
		.ops = &arizona_dai_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "cs47l15-cpu-trace",
		.capture = {
			.stream_name = "Trace CPU",
			.channels_min = 2,
			.channels_max = 6,
			.rates = CS47L15_RATES,
			.formats = CS47L15_FORMATS,
		},
		.compress_dai = 1,
	},
	{
		.name = "cs47l15-dsp-trace",
		.capture = {
			.stream_name = "Trace DSP",
			.channels_min = 2,
			.channels_max = 6,
			.rates = CS47L15_RATES,
			.formats = CS47L15_FORMATS,
		},
	},
};

static irqreturn_t cs47l15_adsp_bus_error(int irq, void *data)
{
	struct wm_adsp *adsp = (struct wm_adsp *)data;
	return wm_adsp2_bus_error(adsp);
}

static void cs47l15_compr_irq(struct cs47l15_priv *cs47l15,
			   struct cs47l15_compr *compr)
{
	bool trigger = false;
	int ret;

	ret = wm_adsp_compr_irq(&compr->adsp_compr, &trigger);
	if (ret < 0)
		return;
}

static irqreturn_t cs47l15_adsp2_irq(int irq, void *data)
{
	struct cs47l15_priv *cs47l15 = data;
	struct arizona *arizona = cs47l15->core.arizona;
	struct cs47l15_compr *compr;
	int i;

	for (i = 0; i < ARRAY_SIZE(cs47l15->compr_info); ++i) {
		if (!cs47l15->compr_info[i].adsp_compr.dsp->running)
			continue;

		compr = &cs47l15->compr_info[i];
		cs47l15_compr_irq(cs47l15, compr);
	}

	if (arizona->pdata.ez2ctrl_trigger) {
		mutex_lock(&cs47l15->trig_lock);
		if (!cs47l15->trig) {
			cs47l15->trig = true;

			if (wm_adsp_fw_has_voice_trig(&cs47l15->core.adsp[0]))
				arizona->pdata.ez2ctrl_trigger();
		}
		mutex_unlock(&cs47l15->trig_lock);
	}
	return IRQ_HANDLED;
}

static struct cs47l15_compr *cs47l15_get_compr(struct snd_soc_pcm_runtime *rtd,
					 struct cs47l15_priv *cs47l15)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cs47l15->compr_info); ++i) {
		if (strcmp(rtd->codec_dai->name,
			   cs47l15->compr_info[i].dai_name) == 0)
			return &cs47l15->compr_info[i];
	}

	return NULL;
}

static int cs47l15_compr_open(struct snd_compr_stream *stream)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct cs47l15_priv *cs47l15 = snd_soc_codec_get_drvdata(rtd->codec);
	struct cs47l15_compr *compr;

	compr = cs47l15_get_compr(rtd, cs47l15);
	if (!compr) {
		dev_err(cs47l15->core.arizona->dev,
			"No compressed stream for dai '%s'\n",
			rtd->codec_dai->name);
		return -EINVAL;
	}

	return wm_adsp_compr_open(&compr->adsp_compr, stream);
}

static int cs47l15_codec_probe(struct snd_soc_codec *codec)
{
	struct cs47l15_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct arizona *arizona = priv->core.arizona;
	int ret;

	codec->control_data = priv->core.arizona->regmap;
	priv->core.arizona->dapm = &codec->dapm;

	ret = snd_soc_codec_set_cache_io(codec, 32, 16, SND_SOC_REGMAP);
	if (ret != 0)
		return ret;

	arizona_init_spk(codec);
	arizona_init_gpio(codec);
	arizona_init_mono(codec);
	arizona_init_input(codec);

	ret = wm_adsp2_codec_probe(&priv->core.adsp[0], codec);
	if (ret)
		return ret;

	ret = snd_soc_add_codec_controls(codec,
					 arizona_adsp2v2_rate_controls, 1);
	if (ret != 0)
		return ret;

	mutex_lock(&codec->card->dapm_mutex);
	snd_soc_dapm_disable_pin(&codec->dapm, "HAPTICS");
	mutex_unlock(&codec->card->dapm_mutex);

	priv->core.arizona->dapm = &codec->dapm;

	ret = arizona_request_irq(arizona, ARIZONA_IRQ_DSP_IRQ1,
				  "ADSP2 interrupt 1", cs47l15_adsp2_irq, priv);
	if (ret != 0) {
		dev_err(arizona->dev, "Failed to request DSP IRQ: %d\n", ret);
		return ret;
	}

	ret = irq_set_irq_wake(arizona->irq, 1);
	if (ret)
		dev_err(arizona->dev,
			"Failed to set DSP IRQ to wake source: %d\n",
			ret);

	ret = arizona_request_irq(arizona,
			MOON_IRQ_DSP1_BUS_ERROR,
			"ADSP2 bus error",
			cs47l15_adsp_bus_error,
			&priv->core.adsp[0]);
	if (ret != 0) {
		dev_err(arizona->dev,
			"Failed to request DSP Lock region IRQ: %d\n",
			ret);
		irq_set_irq_wake(arizona->irq, 0);
		arizona_free_irq(arizona, ARIZONA_IRQ_DSP_IRQ1, priv);
		return ret;
	}

	mutex_lock(&codec->card->dapm_mutex);
	snd_soc_dapm_enable_pin(&codec->dapm, "DRC2 Signal Activity");
	mutex_unlock(&codec->card->dapm_mutex);

	ret = regmap_update_bits(arizona->regmap, CLEARWATER_IRQ2_MASK_9,
				 CLEARWATER_DRC2_SIG_DET_EINT2,
				 0);
	if (ret != 0) {
		dev_err(arizona->dev,
			"Failed to unmask DRC2 IRQ for DSP: %d\n",
			ret);
		goto err_drc;
	}

	return 0;

err_drc:
	arizona_free_irq(arizona, ARIZONA_IRQ_DSP_IRQ1, priv);
	arizona_free_irq(arizona, MOON_IRQ_DSP1_BUS_ERROR,
			 &priv->core.adsp[0]);
	return ret;
}

static int cs47l15_codec_remove(struct snd_soc_codec *codec)
{
	struct cs47l15_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct arizona *arizona = priv->core.arizona;

	irq_set_irq_wake(arizona->irq, 0);
	arizona_free_irq(arizona, ARIZONA_IRQ_DSP_IRQ1, priv);
	arizona_free_irq(arizona, MOON_IRQ_DSP1_BUS_ERROR,
			 &priv->core.adsp[0]);
	regmap_update_bits(arizona->regmap, CLEARWATER_IRQ2_MASK_9,
			   CLEARWATER_DRC2_SIG_DET_EINT2,
			   CLEARWATER_DRC2_SIG_DET_EINT2);

	wm_adsp2_codec_remove(&priv->core.adsp[0], codec);

	priv->core.arizona->dapm = NULL;

	return 0;
}

#define CS47L15_DIG_VU 0x0200

static unsigned int cs47l15_digital_vu[] = {
	ARIZONA_DAC_DIGITAL_VOLUME_1L,
	ARIZONA_DAC_DIGITAL_VOLUME_1R,
	ARIZONA_DAC_DIGITAL_VOLUME_4L,
	ARIZONA_DAC_DIGITAL_VOLUME_5L,
	ARIZONA_DAC_DIGITAL_VOLUME_5R,
};

static struct snd_soc_codec_driver soc_codec_dev_cs47l15 = {
	.probe = cs47l15_codec_probe,
	.remove = cs47l15_codec_remove,

	.idle_bias_off = true,

	.set_sysclk = arizona_set_sysclk,
	.set_pll = cs47l15_set_fll,

	.controls = cs47l15_snd_controls,
	.num_controls = ARRAY_SIZE(cs47l15_snd_controls),
	.dapm_widgets = cs47l15_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cs47l15_dapm_widgets),
	.dapm_routes = cs47l15_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(cs47l15_dapm_routes),
};

static struct snd_compr_ops cs47l15_compr_ops = {
	.open = cs47l15_compr_open,
	.free = wm_adsp_compr_free,
	.set_params = wm_adsp_compr_set_params,
	.trigger = wm_adsp_compr_trigger,
	.pointer = wm_adsp_compr_pointer,
	.copy = wm_adsp_compr_copy,
	.get_caps = wm_adsp_compr_get_caps,
};

static struct snd_soc_platform_driver cs47l15_compr_platform = {
	.compr_ops = &cs47l15_compr_ops,
};

static void cs47l15_init_compr_info(struct cs47l15_priv *cs47l15)
{
	struct wm_adsp *dsp;
	int i;

	BUILD_BUG_ON(ARRAY_SIZE(cs47l15->compr_info) !=
		     ARRAY_SIZE(compr_dai_mapping));

	for (i = 0; i < ARRAY_SIZE(cs47l15->compr_info); ++i) {
		cs47l15->compr_info[i].priv = cs47l15;

		cs47l15->compr_info[i].dai_name =
			compr_dai_mapping[i].dai_name;

		dsp = &cs47l15->core.adsp[compr_dai_mapping[i].adsp_num],
		wm_adsp_compr_init(dsp, &cs47l15->compr_info[i].adsp_compr);
	}
}

static void cs47l15_destroy_compr_info(struct cs47l15_priv *cs47l15)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cs47l15->compr_info); ++i)
		wm_adsp_compr_destroy(&cs47l15->compr_info[i].adsp_compr);
}

static int cs47l15_probe(struct platform_device *pdev)
{
	struct arizona *arizona = dev_get_drvdata(pdev->dev.parent);
	struct cs47l15_priv *cs47l15;
	int i, ret;

	BUILD_BUG_ON(ARRAY_SIZE(cs47l15_dai) > ARIZONA_MAX_DAI);

	cs47l15 = devm_kzalloc(&pdev->dev, sizeof(struct cs47l15_priv),
			      GFP_KERNEL);
	if (cs47l15 == NULL)
		return -ENOMEM;
	platform_set_drvdata(pdev, cs47l15);

	/* Set of_node to parent from the SPI device to allow DAPM to
	 * locate regulator supplies */
	pdev->dev.of_node = arizona->dev->of_node;

	mutex_init(&cs47l15->fw_lock);
	mutex_init(&cs47l15->trig_lock);

	cs47l15->core.arizona = arizona;
	cs47l15->core.num_inputs = 4;

	cs47l15->core.adsp[0].part = "cs47l15";
	if (arizona->pdata.rev_specific_fw)
		cs47l15->core.adsp[0].part_rev = 'a' + arizona->rev;
	cs47l15->core.adsp[0].num = 1;
	cs47l15->core.adsp[0].type = WMFW_ADSP2;
	cs47l15->core.adsp[0].rev = 2;
	cs47l15->core.adsp[0].dev = arizona->dev;
	cs47l15->core.adsp[0].regmap = arizona->regmap_32bit;

	cs47l15->core.adsp[0].base = CLEARWATER_DSP1_CONFIG;
	cs47l15->core.adsp[0].mem = cs47l15_dsp1_regions;
	cs47l15->core.adsp[0].num_mems
		= ARRAY_SIZE(cs47l15_dsp1_regions);

	if (arizona->pdata.num_fw_defs[0]) {
		cs47l15->core.adsp[0].firmwares
			= arizona->pdata.fw_defs[0];

		cs47l15->core.adsp[0].num_firmwares
			= arizona->pdata.num_fw_defs[0];
	}

	cs47l15->core.adsp[0].rate_put_cb =
				cs47l15_adsp_rate_put_cb;

	cs47l15->core.adsp[0].lock_regions = WM_ADSP2_REGION_1_3;

	cs47l15->core.adsp[0].hpimp_cb = arizona_hpimp_cb;

	ret = wm_adsp2_init(&cs47l15->core.adsp[0], &cs47l15->fw_lock);
	if (ret != 0)
		return ret;

	cs47l15_init_compr_info(cs47l15);

	for (i = 0; i < ARRAY_SIZE(cs47l15->fll); i++) {
		cs47l15->fll[i].vco_mult = 3;
		cs47l15->fll[i].min_outdiv = 3;
		cs47l15->fll[i].max_outdiv = 3;
	}

	arizona_init_fll(arizona, 1, ARIZONA_FLL1_CONTROL_1 - 1,
			 ARIZONA_IRQ_FLL1_LOCK, ARIZONA_IRQ_FLL1_CLOCK_OK,
			 &cs47l15->fll[0]);
	arizona_init_fll(arizona, 4, MOON_FLLAO_CONTROL_1 - 1,
			 MOON_IRQ_FLLAO_CLOCK_OK, MOON_IRQ_FLLAO_CLOCK_OK,
			 &cs47l15->fll[1]);

	for (i = 0; i < ARRAY_SIZE(cs47l15_dai); i++)
		arizona_init_dai(&cs47l15->core, i);

	/* Latch volume update bits */
	for (i = 0; i < ARRAY_SIZE(cs47l15_digital_vu); i++)
		regmap_update_bits(arizona->regmap, cs47l15_digital_vu[i],
				   CS47L15_DIG_VU, CS47L15_DIG_VU);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);

	ret = snd_soc_register_platform(&pdev->dev, &cs47l15_compr_platform);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to register platform: %d\n",
			ret);
		goto error;
	}

	ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_cs47l15,
				      cs47l15_dai, ARRAY_SIZE(cs47l15_dai));
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to register codec: %d\n",
			ret);
		snd_soc_unregister_platform(&pdev->dev);
		goto error;
	}

	return ret;

error:
	cs47l15_destroy_compr_info(cs47l15);
	mutex_destroy(&cs47l15->fw_lock);

	return ret;
}

static int cs47l15_remove(struct platform_device *pdev)
{
	struct cs47l15_priv *cs47l15 = platform_get_drvdata(pdev);

	snd_soc_unregister_platform(&pdev->dev);
	snd_soc_unregister_codec(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	cs47l15_destroy_compr_info(cs47l15);

	wm_adsp2_remove(&cs47l15->core.adsp[0]);

	mutex_destroy(&cs47l15->fw_lock);

	return 0;
}

static struct platform_driver cs47l15_codec_driver = {
	.driver = {
		.name = "cs47l15-codec",
		.owner = THIS_MODULE,
	},
	.probe = cs47l15_probe,
	.remove = cs47l15_remove,
};

module_platform_driver(cs47l15_codec_driver);

MODULE_DESCRIPTION("ASoC CS47L15 driver");
MODULE_AUTHOR("Jaswinder Jassal <jjassal@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cs47l15-codec");
