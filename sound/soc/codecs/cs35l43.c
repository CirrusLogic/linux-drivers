// SPDX-License-Identifier: GPL-2.0

/*
 * cs35l43.c -- CS35l43 ALSA SoC audio driver
 *
 * Copyright 2021 Cirrus Logic, Inc.
 *
 * Author:	David Rhodes	<david.rhodes@cirrus.com>
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
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/gpio.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/of_irq.h>
#include <linux/completion.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/timekeeping.h>

#include "wm_adsp.h"
#include "cs35l43.h"
#include <sound/cs35l43.h>

static const char * const cs35l43_supplies[] = {
	"VA",
	"VP",
};

static const DECLARE_TLV_DB_RANGE(dig_vol_tlv,
		0, 0, TLV_DB_SCALE_ITEM(TLV_DB_GAIN_MUTE, 0, 1),
		1, 913, TLV_DB_SCALE_ITEM(-10200, 25, 0));
static DECLARE_TLV_DB_SCALE(amp_gain_tlv, 0, 1, 1);

static const char * const cs35l43_tx_input_texts[] = {"Zero", "ASPRX1",
							"ASPRX2", "VMON",
							"IMON", "VPMON",
							"VBSTMON",
							"DSPTX1", "DSPTX2"};
static const unsigned int cs35l43_tx_input_values[] = {0x00,
						CS35L43_INPUT_SRC_ASPRX1,
						CS35L43_INPUT_SRC_ASPRX2,
						CS35L43_INPUT_SRC_VMON,
						CS35L43_INPUT_SRC_IMON,
						CS35L43_INPUT_SRC_VPMON,
						CS35L43_INPUT_SRC_VBSTMON,
						CS35L43_INPUT_DSP_TX1,
						CS35L43_INPUT_DSP_TX2};

static SOC_VALUE_ENUM_SINGLE_DECL(cs35l43_asptx1_enum,
				CS35L43_ASPTX1_INPUT,
				0, CS35L43_INPUT_MASK,
				cs35l43_tx_input_texts,
				cs35l43_tx_input_values);

static const struct snd_kcontrol_new asp_tx1_mux =
	SOC_DAPM_ENUM("ASPTX1 SRC", cs35l43_asptx1_enum);

static SOC_VALUE_ENUM_SINGLE_DECL(cs35l43_asptx2_enum,
				CS35L43_ASPTX2_INPUT,
				0, CS35L43_INPUT_MASK,
				cs35l43_tx_input_texts,
				cs35l43_tx_input_values);

static const struct snd_kcontrol_new asp_tx2_mux =
	SOC_DAPM_ENUM("ASPTX2 SRC", cs35l43_asptx2_enum);

static SOC_VALUE_ENUM_SINGLE_DECL(cs35l43_asptx3_enum,
				CS35L43_ASPTX3_INPUT,
				0, CS35L43_INPUT_MASK,
				cs35l43_tx_input_texts,
				cs35l43_tx_input_values);

static const struct snd_kcontrol_new asp_tx3_mux =
	SOC_DAPM_ENUM("ASPTX3 SRC", cs35l43_asptx3_enum);

static SOC_VALUE_ENUM_SINGLE_DECL(cs35l43_asptx4_enum,
				CS35L43_ASPTX4_INPUT,
				0, CS35L43_INPUT_MASK,
				cs35l43_tx_input_texts,
				cs35l43_tx_input_values);

static const struct snd_kcontrol_new asp_tx4_mux =
	SOC_DAPM_ENUM("ASPTX4 SRC", cs35l43_asptx4_enum);

static SOC_VALUE_ENUM_SINGLE_DECL(cs35l43_dsprx1_enum,
				CS35L43_DSP1RX1_INPUT,
				0, CS35L43_INPUT_MASK,
				cs35l43_tx_input_texts,
				cs35l43_tx_input_values);

static const struct snd_kcontrol_new dsp_rx1_mux =
	SOC_DAPM_ENUM("DSPRX1 SRC", cs35l43_dsprx1_enum);

static SOC_VALUE_ENUM_SINGLE_DECL(cs35l43_dsprx2_enum,
				CS35L43_DSP1RX2_INPUT,
				0, CS35L43_INPUT_MASK,
				cs35l43_tx_input_texts,
				cs35l43_tx_input_values);

static const struct snd_kcontrol_new dsp_rx2_mux =
	SOC_DAPM_ENUM("DSPRX2 SRC", cs35l43_dsprx2_enum);

static const struct snd_kcontrol_new cs35l43_aud_controls[] = {
	SOC_SINGLE_SX_TLV("Digital PCM Volume", CS35L43_AMP_CTRL,
				CS35L43_AMP_VOL_PCM_SHIFT,
				0x4CF, 0x391, dig_vol_tlv),
	SOC_SINGLE_TLV("Amp Gain", CS35L43_AMP_GAIN,
			CS35L43_AMP_GAIN_PCM_SHIFT, 20, 0,
			amp_gain_tlv),
};

static int cs35l43_main_amp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
				snd_soc_dapm_to_component(w->dapm);
	struct cs35l43_private *cs35l43 =
				snd_soc_component_get_drvdata(component);
	int ret = 0;

	dev_dbg(cs35l43->dev, "%s\n", __func__);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		regmap_update_bits(cs35l43->regmap,
					CS35L43_BLOCK_ENABLES, 1, 1);
		regmap_write(cs35l43->regmap, CS35L43_GLOBAL_ENABLES, 1);

		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_write(cs35l43->regmap, CS35L43_GLOBAL_ENABLES, 0);
		regmap_write(cs35l43->regmap, CS35L43_BLOCK_ENABLES, 0);
		break;
	default:
		dev_err(cs35l43->dev, "Invalid event = 0x%x\n", event);
		ret = -EINVAL;
	}
	return ret;
}

static const struct snd_soc_dapm_widget cs35l43_dapm_widgets[] = {

	SND_SOC_DAPM_OUT_DRV_E("Main AMP", SND_SOC_NOPM, 0, 0, NULL, 0,
			cs35l43_main_amp_event,
			SND_SOC_DAPM_POST_PMD |	SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_OUTPUT("SPK"),

	SND_SOC_DAPM_AIF_IN("ASPRX1", NULL, 0, CS35L43_ASP_ENABLES1,
					CS35L43_ASP_RX1_EN_SHIFT, 0),
	SND_SOC_DAPM_AIF_IN("ASPRX2", NULL, 0, CS35L43_ASP_ENABLES1,
					CS35L43_ASP_RX2_EN_SHIFT, 0),
	SND_SOC_DAPM_AIF_OUT("ASPTX1", NULL, 0, CS35L43_ASP_ENABLES1,
					CS35L43_ASP_TX1_EN_SHIFT, 0),
	SND_SOC_DAPM_AIF_OUT("ASPTX2", NULL, 0, CS35L43_ASP_ENABLES1,
					CS35L43_ASP_TX2_EN_SHIFT, 0),

	SND_SOC_DAPM_MUX("ASP TX1 Source", SND_SOC_NOPM, 0, 0, &asp_tx1_mux),
	SND_SOC_DAPM_MUX("ASP TX2 Source", SND_SOC_NOPM, 0, 0, &asp_tx2_mux),
	SND_SOC_DAPM_MUX("ASP TX3 Source", SND_SOC_NOPM, 0, 0, &asp_tx3_mux),
	SND_SOC_DAPM_MUX("ASP TX4 Source", SND_SOC_NOPM, 0, 0, &asp_tx4_mux),
	SND_SOC_DAPM_MUX("DSP RX1 Source", SND_SOC_NOPM, 0, 0, &dsp_rx1_mux),
	SND_SOC_DAPM_MUX("DSP RX2 Source", SND_SOC_NOPM, 0, 0, &dsp_rx2_mux),

	SND_SOC_DAPM_ADC("VMON ADC", NULL, CS35L43_BLOCK_ENABLES,
					CS35L43_VMON_EN_SHIFT, 0),
	SND_SOC_DAPM_ADC("IMON ADC", NULL, CS35L43_BLOCK_ENABLES,
					CS35L43_IMON_EN_SHIFT, 0),
	SND_SOC_DAPM_ADC("VPMON ADC", NULL, CS35L43_BLOCK_ENABLES,
					CS35L43_VPMON_EN_SHIFT, 0),
	SND_SOC_DAPM_ADC("VBSTMON ADC", NULL, CS35L43_BLOCK_ENABLES,
					CS35L43_VBSTMON_EN_SHIFT, 0),
	SND_SOC_DAPM_ADC("TEMPMON ADC", NULL, CS35L43_BLOCK_ENABLES,
					CS35L43_TEMPMON_EN_SHIFT, 0),
};

static const struct snd_soc_dapm_route cs35l43_audio_map[] = {

	{"ASPRX1", NULL, "AMP Playback"},
	{"ASPRX2", NULL, "AMP Playback"},
	{"Main AMP", NULL, "ASPRX1"},
	{"Main AMP", NULL, "ASPRX2"},
	{"SPK", NULL, "Main AMP"},
	{"ASP TX1 Source", "ASPRX1", "ASPRX1"},
	{"ASP TX2 Source", "ASPRX1", "ASPRX1"},
	{"ASP TX3 Source", "ASPRX1", "ASPRX1"},
	{"ASP TX4 Source", "ASPRX1", "ASPRX1"},
	{"DSP RX1 Source", "ASPRX1", "ASPRX1"},
	{"DSP RX2 Source", "ASPRX1", "ASPRX1"},
	{"ASP TX1 Source", "ASPRX2", "ASPRX2"},
	{"ASP TX2 Source", "ASPRX2", "ASPRX2"},
	{"ASP TX3 Source", "ASPRX2", "ASPRX2"},
	{"ASP TX4 Source", "ASPRX2", "ASPRX2"},
	{"DSP RX1 Source", "ASPRX2", "ASPRX2"},
	{"DSP RX2 Source", "ASPRX2", "ASPRX2"},
	{"ASP TX1 Source", "VMON", "VMON ADC"},
	{"ASP TX2 Source", "VMON", "VMON ADC"},
	{"ASP TX3 Source", "VMON", "VMON ADC"},
	{"ASP TX4 Source", "VMON", "VMON ADC"},
	{"DSP RX1 Source", "VMON", "VMON ADC"},
	{"DSP RX2 Source", "VMON", "VMON ADC"},
	{"ASP TX1 Source", "IMON", "IMON ADC"},
	{"ASP TX2 Source", "IMON", "IMON ADC"},
	{"ASP TX3 Source", "IMON", "IMON ADC"},
	{"ASP TX4 Source", "IMON", "IMON ADC"},
	{"DSP RX1 Source", "IMON", "IMON ADC"},
	{"DSP RX2 Source", "IMON", "IMON ADC"},
	{"ASP TX1 Source", "VPMON", "VPMON ADC"},
	{"ASP TX2 Source", "VPMON", "VPMON ADC"},
	{"ASP TX3 Source", "VPMON", "VPMON ADC"},
	{"ASP TX4 Source", "VPMON", "VPMON ADC"},
	{"DSP RX1 Source", "VPMON", "VPMON ADC"},
	{"DSP RX2 Source", "VPMON", "VPMON ADC"},
	{"ASP TX1 Source", "VBSTMON", "VBSTMON ADC"},
	{"ASP TX2 Source", "VBSTMON", "VBSTMON ADC"},
	{"ASP TX3 Source", "VBSTMON", "VBSTMON ADC"},
	{"ASP TX4 Source", "VBSTMON", "VBSTMON ADC"},
	{"DSP RX1 Source", "VBSTMON", "VBSTMON ADC"},
	{"DSP RX2 Source", "VBSTMON", "VBSTMON ADC"},
	{"ASPTX1", NULL, "ASP TX1 Source"},
	{"ASPTX2", NULL, "ASP TX2 Source"},
	{"AMP Capture", NULL, "ASPTX1"},
	{"AMP Capture", NULL, "ASPTX2"},

	{"VMON ADC", NULL, "AMP Playback"},
	{"IMON ADC", NULL, "AMP Playback"},
	{"VPMON ADC", NULL, "AMP Playback"},
	{"VBSTMON ADC", NULL, "AMP Playback"},
	{"TEMPMON ADC", NULL, "AMP Playback"},
};


static irqreturn_t cs35l43_irq(int irq, void *data)
{
	struct cs35l43_private *cs35l43 = data;
	unsigned int status, mask;


	regmap_read(cs35l43->regmap, CS35L43_IRQ1_STS_1, &status);
	regmap_read(cs35l43->regmap, CS35L43_IRQ1_MASK_1, &mask);

	/* Check to see if unmasked bits are active */
	if (!(status & ~mask))
		return IRQ_NONE;

	/*
	 * The following interrupts require a
	 * protection release cycle to get the
	 * speaker out of Safe-Mode.
	 */
	if (status & CS35L43_AMP_ERR_EINT1_MASK) {
		dev_crit(cs35l43->dev, "Amp short error\n");
		regmap_write(cs35l43->regmap, CS35L43_IRQ1_STS_1,
					CS35L43_AMP_ERR_EINT1_MASK);
		regmap_write(cs35l43->regmap, CS35L43_ERROR_RELEASE, 0);
		regmap_update_bits(cs35l43->regmap, CS35L43_ERROR_RELEASE,
					CS35L43_AMP_SHORT_ERR_RLS_MASK,
					CS35L43_AMP_SHORT_ERR_RLS_MASK);
		regmap_update_bits(cs35l43->regmap, CS35L43_ERROR_RELEASE,
					CS35L43_AMP_SHORT_ERR_RLS_MASK, 0);
	}

	if (status & CS35L43_BST_OVP_ERR_EINT1_MASK) {
		dev_crit(cs35l43->dev, "VBST Over Voltage error\n");
		regmap_update_bits(cs35l43->regmap, CS35L43_BLOCK_ENABLES,
					CS35L43_BST_EN_MASK <<
					CS35L43_BST_EN_SHIFT, 0);
		regmap_write(cs35l43->regmap, CS35L43_IRQ1_STS_1,
					CS35L43_BST_OVP_ERR_EINT1_MASK);
		regmap_write(cs35l43->regmap, CS35L43_ERROR_RELEASE, 0);
		regmap_update_bits(cs35l43->regmap, CS35L43_ERROR_RELEASE,
					CS35L43_BST_OVP_ERR_RLS_MASK,
					CS35L43_BST_OVP_ERR_RLS_MASK);
		regmap_update_bits(cs35l43->regmap, CS35L43_ERROR_RELEASE,
					CS35L43_BST_OVP_ERR_RLS_MASK, 0);
		regmap_update_bits(cs35l43->regmap, CS35L43_BLOCK_ENABLES,
					CS35L43_BST_EN_MASK <<
					CS35L43_BST_EN_SHIFT,
					CS35L43_BST_EN_DEFAULT <<
					CS35L43_BST_EN_SHIFT);
	}

	if (status & CS35L43_BST_DCM_UVP_ERR_EINT1_MASK) {
		dev_crit(cs35l43->dev, "DCM VBST Under Voltage Error\n");
		regmap_update_bits(cs35l43->regmap, CS35L43_BLOCK_ENABLES,
					CS35L43_BST_EN_MASK <<
					CS35L43_BST_EN_SHIFT, 0);
		regmap_write(cs35l43->regmap, CS35L43_IRQ1_STS_1,
					CS35L43_BST_DCM_UVP_ERR_EINT1_MASK);
		regmap_write(cs35l43->regmap, CS35L43_ERROR_RELEASE, 0);
		regmap_update_bits(cs35l43->regmap, CS35L43_ERROR_RELEASE,
					CS35L43_BST_UVP_ERR_RLS_MASK,
					CS35L43_BST_UVP_ERR_RLS_MASK);
		regmap_update_bits(cs35l43->regmap, CS35L43_ERROR_RELEASE,
					CS35L43_BST_UVP_ERR_RLS_MASK, 0);
		regmap_update_bits(cs35l43->regmap, CS35L43_BLOCK_ENABLES,
					CS35L43_BST_EN_MASK <<
					CS35L43_BST_EN_SHIFT,
					CS35L43_BST_EN_DEFAULT <<
					CS35L43_BST_EN_SHIFT);
	}

	if (status & CS35L43_BST_SHORT_ERR_EINT1_MASK) {
		dev_crit(cs35l43->dev, "LBST error: powering off!\n");
		regmap_update_bits(cs35l43->regmap, CS35L43_BLOCK_ENABLES,
					CS35L43_BST_EN_MASK <<
					CS35L43_BST_EN_SHIFT, 0);
		regmap_write(cs35l43->regmap, CS35L43_IRQ1_STS_1,
					CS35L43_BST_SHORT_ERR_EINT1_MASK);
		regmap_write(cs35l43->regmap, CS35L43_ERROR_RELEASE, 0);
		regmap_update_bits(cs35l43->regmap, CS35L43_ERROR_RELEASE,
					CS35L43_BST_SHORT_ERR_RLS_MASK,
					CS35L43_BST_SHORT_ERR_RLS_MASK);
		regmap_update_bits(cs35l43->regmap, CS35L43_ERROR_RELEASE,
					CS35L43_BST_SHORT_ERR_RLS_MASK, 0);
		regmap_update_bits(cs35l43->regmap, CS35L43_BLOCK_ENABLES,
					CS35L43_BST_EN_MASK <<
					CS35L43_BST_EN_SHIFT,
					CS35L43_BST_EN_DEFAULT <<
					CS35L43_BST_EN_SHIFT);
	}

	return IRQ_HANDLED;
}

static int cs35l43_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct cs35l43_private *cs35l43 =
			snd_soc_component_get_drvdata(codec_dai->component);
	unsigned int asp_fmt, lrclk_fmt, sclk_fmt, slave_mode;

	dev_dbg(cs35l43->dev, "%s\n", __func__);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		slave_mode = 1;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		slave_mode = 0;
		break;
	default:
		dev_warn(cs35l43->dev,
			"%s: Mixed master mode unsupported\n", __func__);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		asp_fmt = 0;
		cs35l43->i2s_mode = false;
		cs35l43->dspa_mode = true;
		break;
	case SND_SOC_DAIFMT_I2S:
		asp_fmt = 2;
		cs35l43->i2s_mode = true;
		cs35l43->dspa_mode = false;
		break;
	default:
		dev_warn(cs35l43->dev,
			"%s: Invalid or unsupported DAI format\n", __func__);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_IF:
		lrclk_fmt = 1;
		sclk_fmt = 0;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		lrclk_fmt = 0;
		sclk_fmt = 1;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		lrclk_fmt = 1;
		sclk_fmt = 1;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		lrclk_fmt = 0;
		sclk_fmt = 0;
		break;
	default:
		dev_warn(cs35l43->dev,
			"%s: Invalid DAI clock INV\n", __func__);
		return -EINVAL;
	}

	return 0;
}

struct cs35l43_global_fs_config {
	int rate;
	int fs_cfg;
};

static const struct cs35l43_global_fs_config cs35l43_fs_rates[] = {
	{ 12000,	0x01 },
	{ 24000,	0x02 },
	{ 48000,	0x03 },
	{ 96000,	0x04 },
	{ 192000,	0x05 },
	{ 11025,	0x09 },
	{ 22050,	0x0A },
	{ 44100,	0x0B },
	{ 88200,	0x0C },
	{ 176400,	0x0D },
	{ 8000,		0x11 },
	{ 16000,	0x12 },
	{ 32000,	0x13 },
};

static int cs35l43_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	int i;
	unsigned int rate = params_rate(params);
	u8 asp_width, asp_wl;
	struct cs35l43_private *cs35l43 =
				snd_soc_component_get_drvdata(dai->component);

	dev_dbg(cs35l43->dev, "%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(cs35l43_fs_rates); i++) {
		if (rate == cs35l43_fs_rates[i].rate)
			break;
	}

	if (i < ARRAY_SIZE(cs35l43_fs_rates))
		regmap_update_bits(cs35l43->regmap, CS35L43_GLOBAL_SAMPLE_RATE,
			CS35L43_GLOBAL_FS_MASK, cs35l43_fs_rates[i].fs_cfg);
	else {
		dev_err(cs35l43->dev, "%s: Unsupported rate\n", __func__);
		return -EINVAL;
	}

	asp_wl = params_width(params);
	asp_width = params_physical_width(params);
	dev_dbg(cs35l43->dev, "%s\n wl=%d, width=%d, rate=%d", __func__,
				asp_wl, asp_width, rate);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_update_bits(cs35l43->regmap, CS35L43_ASP_CONTROL2,
				CS35L43_ASP_RX_WIDTH_MASK, asp_width <<
				CS35L43_ASP_RX_WIDTH_SHIFT);
		regmap_update_bits(cs35l43->regmap, CS35L43_ASP_DATA_CONTROL5,
				CS35L43_ASP_RX_WL_MASK, asp_wl);
	} else {
		regmap_update_bits(cs35l43->regmap, CS35L43_ASP_CONTROL2,
				CS35L43_ASP_TX_WIDTH_MASK, asp_width <<
				CS35L43_ASP_TX_WIDTH_SHIFT);
		regmap_update_bits(cs35l43->regmap, CS35L43_ASP_DATA_CONTROL1,
				CS35L43_ASP_TX_WL_MASK, asp_wl);
	}

	return 0;
}

static int cs35l43_get_clk_config(int freq)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cs35l43_pll_sysclk); i++) {
		if (cs35l43_pll_sysclk[i].freq == freq)
			return cs35l43_pll_sysclk[i].clk_cfg;
	}

	return -EINVAL;
}

static const unsigned int cs35l43_src_rates[] = {
	8000, 12000, 11025, 16000, 22050, 24000, 32000,
	44100, 48000, 88200, 96000, 176400, 192000
};

static const struct snd_pcm_hw_constraint_list cs35l43_constraints = {
	.count = ARRAY_SIZE(cs35l43_src_rates),
	.list = cs35l43_src_rates,
};

static int cs35l43_pcm_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct cs35l43_private *cs35l43 =
				snd_soc_component_get_drvdata(dai->component);

	dev_dbg(cs35l43->dev, "%s\n", __func__);

	if (substream->runtime)
		return snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE, &cs35l43_constraints);
	return 0;
}

static int cs35l43_component_set_sysclk(struct snd_soc_component *component,
				int clk_id, int source, unsigned int freq,
				int dir)
{
	struct cs35l43_private *cs35l43 =
				snd_soc_component_get_drvdata(component);

	dev_dbg(cs35l43->dev, "%s\n", __func__);
	dev_dbg(cs35l43->dev, "%s id = %d, freq=%d\n", __func__, clk_id, freq);

	cs35l43->extclk_cfg = cs35l43_get_clk_config(freq);

	if (cs35l43->extclk_cfg < 0) {
		dev_err(cs35l43->dev, "Invalid CLK Config: %d, freq: %u\n",
			cs35l43->extclk_cfg, freq);
		return -EINVAL;
	}

	regmap_update_bits(cs35l43->regmap, CS35L43_REFCLK_INPUT,
			CS35L43_PLL_OPEN_LOOP_MASK,
			CS35L43_PLL_OPEN_LOOP_MASK);
	regmap_update_bits(cs35l43->regmap, CS35L43_REFCLK_INPUT,
			CS35L43_PLL_REFCLK_FREQ_MASK,
			cs35l43->extclk_cfg << CS35L43_PLL_REFCLK_FREQ_SHIFT);
	regmap_update_bits(cs35l43->regmap, CS35L43_REFCLK_INPUT,
			CS35L43_PLL_REFCLK_EN_MASK, 0);
	regmap_update_bits(cs35l43->regmap, CS35L43_REFCLK_INPUT,
			CS35L43_PLL_REFCLK_SEL_MASK, clk_id);
	regmap_update_bits(cs35l43->regmap, CS35L43_REFCLK_INPUT,
			CS35L43_PLL_OPEN_LOOP_MASK,
			0);
	regmap_update_bits(cs35l43->regmap, CS35L43_REFCLK_INPUT,
			CS35L43_PLL_REFCLK_EN_MASK,
			CS35L43_PLL_REFCLK_EN_MASK);

	return 0;
}

static int cs35l43_dai_set_sysclk(struct snd_soc_dai *dai,
					int clk_id, unsigned int freq, int dir)
{
	struct cs35l43_private *cs35l43 =
				snd_soc_component_get_drvdata(dai->component);

	dev_dbg(cs35l43->dev, "%s\n", __func__);

	return 0;
}

static int cs35l43_irq_gpio_config(struct cs35l43_private *cs35l43)
{
	int irq_pol = IRQF_TRIGGER_NONE;

	if (cs35l43->pdata.gpio1_out_enable)
		regmap_update_bits(cs35l43->regmap,
					CS35L43_GPIO1_CTRL1,
					CS35L43_GP1_DIR_MASK,
					0);
	if (cs35l43->pdata.gpio1_src_sel)
		regmap_update_bits(cs35l43->regmap,
					CS35L43_GPIO_PAD_CONTROL,
					CS35L43_GP1_CTRL_MASK,
					cs35l43->pdata.gpio1_src_sel <<
					CS35L43_GP1_CTRL_SHIFT);

	if (cs35l43->pdata.gpio2_out_enable)
			regmap_update_bits(cs35l43->regmap,
						CS35L43_GPIO2_CTRL1,
						CS35L43_GP2_DIR_MASK,
						0);
	if (cs35l43->pdata.gpio2_src_sel)
		regmap_update_bits(cs35l43->regmap,
					CS35L43_GPIO_PAD_CONTROL,
					CS35L43_GP2_CTRL_MASK,
					cs35l43->pdata.gpio2_src_sel <<
					CS35L43_GP2_CTRL_SHIFT);

	if (cs35l43->pdata.gpio2_src_sel ==
		  (CS35L43_GP2_CTRL_OPEN_DRAIN_ACTV_LO | CS35L43_VALID_PDATA) ||
		  cs35l43->pdata.gpio2_src_sel ==
		  (CS35L43_GP2_CTRL_PUSH_PULL_ACTV_LO | CS35L43_VALID_PDATA))
		irq_pol = IRQF_TRIGGER_LOW;
	else if (cs35l43->pdata.gpio2_src_sel ==
		     (CS35L43_GP2_CTRL_PUSH_PULL_ACTV_HI | CS35L43_VALID_PDATA))
		irq_pol = IRQF_TRIGGER_HIGH;

	return irq_pol;
}

static int cs35l43_set_pdata(struct cs35l43_private *cs35l43)
{
	if (cs35l43->pdata.bst_vctrl)
		regmap_update_bits(cs35l43->regmap, CS35L43_VBST_CTL_1,
				CS35L43_BST_CTL_MASK, cs35l43->pdata.bst_vctrl);

	if (cs35l43->pdata.classh_disable)
		regmap_update_bits(cs35l43->regmap, CS35L43_VBST_CTL_2,
				CS35L43_BST_CTL_SEL_MASK, 0);
	else {
		if (cs35l43->pdata.bst_vctrl)
			regmap_update_bits(cs35l43->regmap, CS35L43_VBST_CTL_2,
				CS35L43_BST_CTL_LIM_EN_MASK,
				CS35L43_BST_CTL_LIM_EN_MASK);
	}

	if (cs35l43->pdata.dsp_ng_enable) {
		regmap_update_bits(cs35l43->regmap,
				CS35L43_MIXER_NGATE_CH1_CFG,
				CS35L43_AUX_NGATE_CH1_EN_MASK,
				CS35L43_AUX_NGATE_CH1_EN_MASK);
		regmap_update_bits(cs35l43->regmap,
				CS35L43_MIXER_NGATE_CH2_CFG,
				CS35L43_AUX_NGATE_CH2_EN_MASK,
				CS35L43_AUX_NGATE_CH2_EN_MASK);

		if (cs35l43->pdata.dsp_ng_pcm_thld) {
			regmap_update_bits(cs35l43->regmap,
				CS35L43_MIXER_NGATE_CH1_CFG,
				CS35L43_AUX_NGATE_CH1_THR_MASK,
				cs35l43->pdata.dsp_ng_pcm_thld);
			regmap_update_bits(cs35l43->regmap,
				CS35L43_MIXER_NGATE_CH2_CFG,
				CS35L43_AUX_NGATE_CH2_THR_MASK,
				cs35l43->pdata.dsp_ng_pcm_thld);
		}

		if (cs35l43->pdata.dsp_ng_delay) {
			regmap_update_bits(cs35l43->regmap,
				CS35L43_MIXER_NGATE_CH1_CFG,
				CS35L43_AUX_NGATE_CH1_HOLD_MASK,
				cs35l43->pdata.dsp_ng_delay <<
				CS35L43_AUX_NGATE_CH1_HOLD_SHIFT);
			regmap_update_bits(cs35l43->regmap,
				CS35L43_MIXER_NGATE_CH2_CFG,
				CS35L43_AUX_NGATE_CH2_HOLD_MASK,
				cs35l43->pdata.dsp_ng_delay <<
				CS35L43_AUX_NGATE_CH2_HOLD_SHIFT);
		}
	}

	if (cs35l43->pdata.hw_ng_sel)
		regmap_update_bits(cs35l43->regmap,
				CS35L43_NG_CONFIG,
				CS35L43_NG_EN_SEL_MASK,
				cs35l43->pdata.hw_ng_sel <<
				CS35L43_NG_EN_SEL_SHIFT);

	if (cs35l43->pdata.hw_ng_thld)
		regmap_update_bits(cs35l43->regmap,
				CS35L43_NG_CONFIG,
				CS35L43_NG_PCM_THLD_MASK,
				cs35l43->pdata.hw_ng_thld <<
				CS35L43_NG_PCM_THLD_SHIFT);

	if (cs35l43->pdata.hw_ng_delay)
		regmap_update_bits(cs35l43->regmap,
				CS35L43_NG_CONFIG,
				CS35L43_NG_DELAY_MASK,
				cs35l43->pdata.hw_ng_delay <<
				CS35L43_NG_DELAY_SHIFT);

	return 0;
}

static int cs35l43_handle_of_data(struct device *dev,
				  struct cs35l43_platform_data *pdata,
				  struct cs35l43_private *cs35l43)
{
	struct device_node *np = dev->of_node;
	int ret, val;

	if (!np)
		return 0;

	pdata->dsp_ng_enable = of_property_read_bool(np,
					"cirrus,dsp-noise-gate-enable");
	if (of_property_read_u32(np,
				"cirrus,dsp-noise-gate-threshold", &val) >= 0)
		pdata->dsp_ng_pcm_thld = val | CS35L43_VALID_PDATA;
	if (of_property_read_u32(np, "cirrus,dsp-noise-gate-delay", &val) >= 0)
		pdata->dsp_ng_delay = val | CS35L43_VALID_PDATA;

	if (of_property_read_u32(np, "cirrus,hw-noise-gate-select", &val) >= 0)
		pdata->hw_ng_sel = val | CS35L43_VALID_PDATA;
	if (of_property_read_u32(np,
				"cirrus,hw-noise-gate-threshold", &val) >= 0)
		pdata->hw_ng_thld = val | CS35L43_VALID_PDATA;
	if (of_property_read_u32(np, "cirrus,hw-noise-gate-delay", &val) >= 0)
		pdata->hw_ng_delay = val | CS35L43_VALID_PDATA;

	if (of_property_read_u32(np, "cirrus,gpio1-src-sel", &val) >= 0)
		pdata->gpio1_src_sel = val | CS35L43_VALID_PDATA;
	if (of_property_read_u32(np, "cirrus,gpio2-src-sel", &val) >= 0)
		pdata->gpio2_src_sel = val | CS35L43_VALID_PDATA;
	pdata->gpio1_out_enable = of_property_read_bool(np,
					"cirrus,gpio1-output-enable");
	pdata->gpio2_out_enable = of_property_read_bool(np,
					"cirrus,gpio2-output-enable");

	pdata->classh_disable = of_property_read_bool(np,
						"cirrus,classh-disable");
	ret = of_property_read_u32(np, "cirrus,boost-ctl-millivolt", &val);
	if (ret >= 0) {
		if (val < 2550 || val > 11000) {
			dev_err(dev,
				"Invalid Boost Voltage %u mV\n", val);
			return -EINVAL;
		}
		pdata->bst_vctrl = ((val - 2550) / 100) + 1;
	}

	return 0;
}


static int cs35l43_component_probe(struct snd_soc_component *component)
{
	int ret = 0;
	struct cs35l43_private *cs35l43 =
		snd_soc_component_get_drvdata(component);

	cs35l43_set_pdata(cs35l43);

	return ret;
}

static void cs35l43_component_remove(struct snd_soc_component *component)
{

}

static const struct snd_soc_dai_ops cs35l43_ops = {
	.startup = cs35l43_pcm_startup,
	.set_fmt = cs35l43_set_dai_fmt,
	.hw_params = cs35l43_pcm_hw_params,
	.set_sysclk = cs35l43_dai_set_sysclk,
};

static struct snd_soc_dai_driver cs35l43_dai[] = {
	{
		.name = "cs35l43-pcm",
		.id = 0,
		.playback = {
			.stream_name = "AMP Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_KNOT,
			.formats = CS35L43_RX_FORMATS,
		},
		.capture = {
			.stream_name = "AMP Capture",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_KNOT,
			.formats = CS35L43_TX_FORMATS,
		},
		.ops = &cs35l43_ops,
		.symmetric_rate = 1,
	},
};

static const struct snd_soc_component_driver soc_component_dev_cs35l43 = {
	.probe = cs35l43_component_probe,
	.remove = cs35l43_component_remove,

	.dapm_widgets = cs35l43_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cs35l43_dapm_widgets),
	.dapm_routes = cs35l43_audio_map,
	.num_dapm_routes = ARRAY_SIZE(cs35l43_audio_map),

	.controls = cs35l43_aud_controls,
	.num_controls = ARRAY_SIZE(cs35l43_aud_controls),

	.set_sysclk = cs35l43_component_set_sysclk,
};

int cs35l43_probe(struct cs35l43_private *cs35l43,
				struct cs35l43_platform_data *pdata)
{
	int ret, i;
	unsigned int regid, revid;
	int irq_pol = IRQF_TRIGGER_HIGH;

	for (i = 0; i < ARRAY_SIZE(cs35l43_supplies); i++)
		cs35l43->supplies[i].supply = cs35l43_supplies[i];

	cs35l43->num_supplies = ARRAY_SIZE(cs35l43_supplies);

	ret = devm_regulator_bulk_get(cs35l43->dev, cs35l43->num_supplies,
					cs35l43->supplies);
	if (ret != 0) {
		dev_err(cs35l43->dev,
			"Failed to request core supplies: %d\n",
			ret);
		return ret;
	}

	if (pdata) {
		cs35l43->pdata = *pdata;
	} else if (cs35l43->dev->of_node) {
		ret = cs35l43_handle_of_data(cs35l43->dev, &cs35l43->pdata,
					     cs35l43);
		if (ret != 0)
			return ret;
	} else {
		ret = -EINVAL;
		goto err;
	}

	ret = regulator_bulk_enable(cs35l43->num_supplies, cs35l43->supplies);
	if (ret != 0) {
		dev_err(cs35l43->dev,
			"Failed to enable core supplies: %d\n", ret);
		return ret;
	}

	/* returning NULL can be an option if in stereo mode */
	cs35l43->reset_gpio = devm_gpiod_get_optional(cs35l43->dev, "reset",
							GPIOD_OUT_LOW);
	if (IS_ERR(cs35l43->reset_gpio)) {
		ret = PTR_ERR(cs35l43->reset_gpio);
		cs35l43->reset_gpio = NULL;
		if (ret == -EBUSY) {
			dev_info(cs35l43->dev,
				 "Reset line busy, assuming shared reset\n");
		} else {
			dev_err(cs35l43->dev,
				"Failed to get reset GPIO: %d\n", ret);
			goto err;
		}
	}
	if (cs35l43->reset_gpio) {
		/* satisfy minimum reset pulse width spec */
		usleep_range(2000, 2100);
		gpiod_set_value_cansleep(cs35l43->reset_gpio, 1);
	}

	usleep_range(2000, 2100);

	ret = regmap_read(cs35l43->regmap, CS35L43_DEVID, &regid);
	if (ret < 0) {
		dev_err(cs35l43->dev, "Get Device ID failed\n");
		goto err;
	}

	ret = regmap_read(cs35l43->regmap, CS35L43_REVID, &revid);
	if (ret < 0) {
		dev_err(cs35l43->dev, "Get Revision ID failed\n");
		goto err;
	}

	irq_pol = cs35l43_irq_gpio_config(cs35l43);
	ret = devm_request_threaded_irq(cs35l43->dev, cs35l43->irq, NULL,
				cs35l43_irq, IRQF_ONESHOT | IRQF_SHARED |
				irq_pol, "cs35l43", cs35l43);

	regmap_update_bits(cs35l43->regmap, CS35L43_IRQ1_MASK_1,
				CS35L43_AMP_ERR_EINT1_MASK |
				CS35L43_BST_SHORT_ERR_EINT1_MASK |
				CS35L43_BST_DCM_UVP_ERR_EINT1_MASK |
				CS35L43_BST_OVP_ERR_EINT1_MASK, 0);

	ret = snd_soc_register_component(cs35l43->dev,
					&soc_component_dev_cs35l43,
					cs35l43_dai, ARRAY_SIZE(cs35l43_dai));
	if (ret < 0) {
		dev_err(cs35l43->dev, "%s: Register codec failed\n", __func__);
		goto err;
	}

	dev_info(cs35l43->dev, "Cirrus Logic cs35l43 (%x), Revision: %02X\n",
			regid, revid);

err:
	regulator_bulk_disable(cs35l43->num_supplies, cs35l43->supplies);
	return ret;
}

int cs35l43_remove(struct cs35l43_private *cs35l43)
{
	regulator_bulk_disable(cs35l43->num_supplies, cs35l43->supplies);
	snd_soc_unregister_component(cs35l43->dev);
	return 0;
}

MODULE_DESCRIPTION("ASoC CS35L43 driver");
MODULE_AUTHOR("David Rhodes, Cirrus Logic Inc, <david.rhodes@cirrus.com>");
MODULE_LICENSE("GPL");
