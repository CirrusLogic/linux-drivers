/*
 * cs35l35.c -- CS35l34 ALSA SoC audio driver
 *
 * Copyright 2016 Cirrus Logic, Inc.
 *
 * Author: Brian Austin <brian.austin@cirrus.com>
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
#include <sound/cs35l35.h>

#include "cs35l35.h"

static const struct reg_default cs35l35_reg[] = {
	{CS35L35_PWRCTL1,		0x01},
	{CS35L35_PWRCTL2,		0x11},
	{CS35L35_PWRCTL3,		0x00},
	{CS35L35_CLK_CTL1,		0x04},
	{CS35L35_CLK_CTL2,		0x10},
	{CS35L35_CLK_CTL3,		0xCF},
	{CS35L35_SP_FMT_CTL1,		0x20},
	{CS35L35_SP_FMT_CTL2,		0x00},
	{CS35L35_SP_FMT_CTL3,		0x02},
	{CS35L35_MAG_COMP_CTL,		0x00},
	{CS35L35_AMP_INP_DRV_CTL,	0x01},
	{CS35L35_AMP_DIG_VOL_CTL,	0x12},
	{CS35L35_AMP_DIG_VOL,		0x00},
	{CS35L35_ADV_DIG_VOL,		0x00},
	{CS35L35_PROTECT_CTL,		0x06},
	{CS35L35_AMP_GAIN_AUD_CTL,	0x13},
	{CS35L35_AMP_GAIN_PDM_CTL,	0x00},
	{CS35L35_AMP_GAIN_ADV_CTL,	0x00},
	{CS35L35_GPI_CTL,		0x00},
	{CS35L35_BST_CVTR_V_CTL,	0x00},
	{CS35L35_BST_PEAK_I,		0x07},
	{CS35L35_BST_RAMP_CTL,		0x85},
	{CS35L35_BST_CONV_COEF_1,	0x20},
	{CS35L35_BST_CONV_COEF_2,	0x20},
	{CS35L35_BST_CONV_SLOPE_COMP,	0x47},
	{CS35L35_BST_CONV_SW_FREQ,	0x04},
	{CS35L35_CLASS_H_CTL,		0x0B},
	{CS35L35_CLASS_H_HEADRM_CTL,	0x0B},
	{CS35L35_CLASS_H_RELEASE_RATE,	0x08},
	{CS35L35_CLASS_H_FET_DRIVE_CTL, 0x41},
	{CS35L35_CLASS_H_VP_CTL,	0xC5},
	{CS35L35_VPBR_CTL,		0x0A},
	{CS35L35_VPBR_VOL_CTL,		0x09},
	{CS35L35_VPBR_TIMING_CTL,	0x6A},
	{CS35L35_VPBR_MODE_VOL_CTL,	0x00},
	{CS35L35_SPKR_MON_CTL,		0xC0},
	{CS35L35_IMON_SCALE_CTL,	0x30},
	{CS35L35_AUDIN_RXLOC_CTL,	0x00},
	{CS35L35_ADVIN_RXLOC_CTL,	0x80},
	{CS35L35_VMON_TXLOC_CTL,	0x00},
	{CS35L35_IMON_TXLOC_CTL,	0x80},
	{CS35L35_VPMON_TXLOC_CTL,	0x04},
	{CS35L35_VBSTMON_TXLOC_CTL,	0x84},
	{CS35L35_VPBR_STATUS_LOC_CTL,	0x04},
	{CS35L35_ZERO_FILL_LOC_CTL,	0x00},
	{CS35L35_AUDIN_DEPTH_CTL,	0x0F},
	{CS35L35_SPKMON_DEPTH_CTL,	0x0F},
	{CS35L35_SUPMON_DEPTH_CTL,	0x0F},
	{CS35L35_ZEROFILL_DEPTH_CTL,	0x00},
	{CS35L35_MULT_DEV_SYNCH1,	0x02},
	{CS35L35_MULT_DEV_SYNCH2,	0x80},
	{CS35L35_PROT_RELEASE_CTL,	0x00},
	{CS35L35_DIAG_MODE_REG_LOCK,	0x00},
	{CS35L35_DIAG_MODE_CTL_1,	0x40},
	{CS35L35_DIAG_MODE_CTL_2,	0x00},
	{CS35L35_INT_MASK_1,		0xFF},
	{CS35L35_INT_MASK_2,		0xFF},
	{CS35L35_INT_MASK_3,		0xFF},
	{CS35L35_INT_MASK_4,		0xFF},

};

static bool cs35l35_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CS35L35_DEVID_AB ... CS35L35_REV_ID:
	case CS35L35_INT_STATUS_1:
	case CS35L35_INT_STATUS_2:
	case CS35L35_INT_STATUS_3:
	case CS35L35_INT_STATUS_4:
	case CS35L35_PLL_STATUS:
	case CS35L35_OTP_TRIM_STATUS:
		return true;
	default:
		return false;
	}
}

static bool cs35l35_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CS35L35_DEVID_AB ... CS35L35_PWRCTL3:
	case CS35L35_CLK_CTL1 ... CS35L35_SP_FMT_CTL3:
	case CS35L35_MAG_COMP_CTL ... CS35L35_AMP_GAIN_AUD_CTL:
	case CS35L35_AMP_GAIN_PDM_CTL ... CS35L35_BST_PEAK_I:
	case CS35L35_BST_RAMP_CTL ... CS35L35_BST_CONV_SW_FREQ:
	case CS35L35_CLASS_H_CTL ... CS35L35_CLASS_H_VP_CTL:
	case CS35L35_CLASS_H_STATUS:
	case CS35L35_VPBR_CTL ... CS35L35_VPBR_MODE_VOL_CTL:
	case CS35L35_VPBR_ATTEN_STATUS:
	case CS35L35_SPKR_MON_CTL:
	case CS35L35_IMON_SCALE_CTL ... CS35L35_ZEROFILL_DEPTH_CTL:
	case CS35L35_MULT_DEV_SYNCH1 ... CS35L35_PROT_RELEASE_CTL:
	case CS35L35_DIAG_MODE_REG_LOCK ... CS35L35_DIAG_MODE_CTL_2:
	case CS35L35_INT_MASK_1 ... CS35L35_PLL_STATUS:
	case CS35L35_OTP_TRIM_STATUS:
		return true;
	default:
		return false;
	}
}

static bool cs35l35_precious_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CS35L35_INT_STATUS_1:
	case CS35L35_INT_STATUS_2:
	case CS35L35_INT_STATUS_3:
	case CS35L35_INT_STATUS_4:
	case CS35L35_PLL_STATUS:
	case CS35L35_OTP_TRIM_STATUS:
		return true;
	default:
		return false;
	}
}

#if 0
static int cs35l35_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
				unsigned int rx_mask, int slots, int slot_width)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int reg, bit_pos, i;
	int slot, slot_num;

	if (slot_width != 8)
		return -EINVAL;
	/* scan rx_mask for aud slot */
	slot = ffs(rx_mask) - 1;
	if (slot >= 0)
		snd_soc_update_bits(codec, CS35L35_TDM_RX_CTL_1_AUDIN,
			X_LOC, slot);

	/* scan tx_mask: vmon(2 slots); imon (2 slots); vpmon (1 slot)
	 * vbstmon (1 slot)
	 */
	slot = ffs(tx_mask) - 1;
	slot_num = 0;

	for (i = 0; i < 2 ; i++) {
		/* disable vpmon/vbstmon: enable later if set in tx_mask */
		snd_soc_update_bits(codec, CS35L35_TDM_TX_CTL_3_VPMON + i,
			X_STATE | X_LOC, X_STATE | X_LOC);
	}

	/* disconnect {vp,vbst}_mon routes: eanble later if set in tx_mask*/
	while (slot >= 0) {
		/* configure VMON_TX_LOC */
		if (slot_num == 0)
			snd_soc_update_bits(codec, CS35L35_TDM_TX_CTL_1_VMON,
				X_STATE | X_LOC, slot);

		/* configure IMON_TX_LOC */
		if (slot_num == 4) {
			snd_soc_update_bits(codec, CS35L35_TDM_TX_CTL_2_IMON,
				X_STATE | X_LOC, slot);
		}
		/* configure VPMON_TX_LOC */
		if (slot_num == 3) {
			snd_soc_update_bits(codec, CS35L35_TDM_TX_CTL_3_VPMON,
				X_STATE | X_LOC, slot);
		}
		/* configure VBSTMON_TX_LOC */
		if (slot_num == 7) {
			snd_soc_update_bits(codec,
				CS35L35_TDM_TX_CTL_4_VBSTMON,
				X_STATE | X_LOC, slot);
		}

		/* Enable the relevant tx slot */
		reg = CS35L35_TDM_TX_SLOT_EN_4 - (slot/8);
		bit_pos = slot - ((slot / 8) * (8));
		snd_soc_update_bits(codec, reg,
			1 << bit_pos, 1 << bit_pos);

		tx_mask &= ~(1 << slot);
		slot = ffs(tx_mask) - 1;
		slot_num++;
	}

	return 0;
}
#endif

static int cs35l35_main_amp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cs35l35_private *cs35l35 = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (cs35l35->pdata.bst_pdn_fet_on)
			regmap_update_bits(cs35l35->regmap, CS35L35_PWRCTL2,
				CS35L35_PDN_BST_MASK,
				0 << CS35L35_PDN_BST_FETON_SHIFT);
		else
			regmap_update_bits(cs35l35->regmap, CS35L35_PWRCTL2,
				CS35L35_PDN_BST_MASK,
				0 << CS35L35_PDN_BST_FETOFF_SHIFT);
		usleep_range(5000, 5100);
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (cs35l35->pdata.bst_pdn_fet_on)
			regmap_update_bits(cs35l35->regmap, CS35L35_PWRCTL2,
				CS35L35_PDN_BST_MASK,
				1 << CS35L35_PDN_BST_FETON_SHIFT);
		else
			regmap_update_bits(cs35l35->regmap, CS35L35_PWRCTL2,
				CS35L35_PDN_BST_MASK,
				1 << CS35L35_PDN_BST_FETOFF_SHIFT);
		usleep_range(5000, 5100);
		break;
	default:
		pr_err("Invalid event = 0x%x\n", event);
	}
	return 0;
}

static DECLARE_TLV_DB_SCALE(amp_gain_tlv, 0, 1, 1);
static DECLARE_TLV_DB_SCALE(dig_vol_tlv, -10200, 50, 0);

static const char * const ch_vpman_txt[] = {
	"Disabled", "Enabled"
};

static const char * const ch_wkfet_txt[] = {
	"Weak FET", "ALC Path"
};

static const char * const ch_vprate_txt[] = {
	"128", "2048", "32768", "524288"
};

static const char * const ch_mem_depth_txt[] = {
	"8FS", "16FS"
};

static const char * const ch_vbst_ovr_txt[] = {
	"Internal", "VBST CTL"
};

static const char * const classh_ctl_txt[] = {
	"On", "Off"
};

static SOC_ENUM_SINGLE_DECL(ch_vpman, CS35L35_CLASS_H_VP_CTL,
				0, ch_vpman_txt);
static SOC_ENUM_SINGLE_DECL(ch_wkfet, CS35L35_CLASS_H_FET_DRIVE_CTL,
				7, ch_wkfet_txt);
static SOC_ENUM_SINGLE_DECL(ch_vprate, CS35L35_CLASS_H_VP_CTL,
				5, ch_vprate_txt);
static SOC_ENUM_SINGLE_DECL(ch_mem_depth, CS35L35_CLASS_H_CTL,
				0, ch_mem_depth_txt);
static SOC_ENUM_SINGLE_DECL(ch_vbst_ovr, CS35L35_CLASS_H_CTL,
				2, ch_vbst_ovr_txt);
static SOC_ENUM_SINGLE_DECL(classh_ctl, CS35L35_PWRCTL2,
				5, ch_vbst_ovr_txt);

static const struct snd_kcontrol_new cs35l35_aud_controls[] = {
	SOC_SINGLE_SX_TLV("Digital Audio Volume", CS35L35_AMP_DIG_VOL,
		      0, 0x34, 0xE4, dig_vol_tlv),
	SOC_SINGLE_TLV("AMP Audio Gain", CS35L35_AMP_GAIN_AUD_CTL, 0, 19, 0,
			amp_gain_tlv),
	SOC_SINGLE_TLV("AMP PDM Gain", CS35L35_AMP_GAIN_PDM_CTL, 0, 19, 0,
			amp_gain_tlv),
};

static const struct snd_kcontrol_new cs35l35_adv_controls[] = {
	SOC_SINGLE_SX_TLV("Digital Advisory Volume", CS35L35_ADV_DIG_VOL,
		      0, 0x34, 0xE4, dig_vol_tlv),
	SOC_SINGLE_TLV("AMP Advisory Gain", CS35L35_AMP_GAIN_ADV_CTL, 0, 19, 0,
			amp_gain_tlv),
};

static const struct snd_kcontrol_new cs35l35_classh_controls[] = {

	SOC_ENUM("ClassH", classh_ctl),
	SOC_SINGLE_RANGE("ClassH Headroom", CS35L35_CLASS_H_HEADRM_CTL,
		      0, 0x00, 0x3F, 0),
	SOC_SINGLE_RANGE("ClassH Release", CS35L35_CLASS_H_RELEASE_RATE,
			 0, 0x03, 0xFF, 0),
	SOC_ENUM("ClassH FET Switch", ch_wkfet),
	SOC_ENUM("ClassH VPCH Switch", ch_vpman),
	SOC_ENUM("ClassH VPCH Rate", ch_vprate),
	SOC_SINGLE_RANGE("ClassH VPCH", CS35L35_CLASS_H_VP_CTL,
			 0, 0x03, 0x1E, 0),
	SOC_ENUM("ClassH MEM Depth", ch_mem_depth),
	SOC_SINGLE_RANGE("VBST Control", CS35L35_BST_CVTR_V_CTL,
			 0, 0x00, 0x41, 0),
	SOC_ENUM("ClassH VBST Ctl Switch", ch_vbst_ovr),
};

static const struct snd_soc_dapm_widget cs35l35_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("SDIN", NULL, 0, CS35L35_PWRCTL3, 1, 1),
	SND_SOC_DAPM_AIF_OUT("SDOUT", NULL, 0, CS35L35_PWRCTL3, 2, 1),

	SND_SOC_DAPM_OUTPUT("SPK"),

	SND_SOC_DAPM_INPUT("VP"),
	SND_SOC_DAPM_INPUT("VPST"),
	SND_SOC_DAPM_INPUT("ISENSE"),
	SND_SOC_DAPM_INPUT("VSENSE"),

	SND_SOC_DAPM_ADC("VMON ADC", NULL, CS35L35_PWRCTL2, 7, 1),
	SND_SOC_DAPM_ADC("IMON ADC", NULL, CS35L35_PWRCTL2, 6, 1),
	SND_SOC_DAPM_ADC("VPMON ADC", NULL, CS35L35_PWRCTL3, 3, 1),
	SND_SOC_DAPM_ADC("VBSTMON ADC", NULL, CS35L35_PWRCTL3, 4, 1),
	SND_SOC_DAPM_ADC("CLASS H", NULL, CS35L35_PWRCTL2, 5, 1),

	SND_SOC_DAPM_OUT_DRV_E("Main AMP", CS35L35_PWRCTL2, 0, 1, NULL, 0,
		cs35l35_main_amp_event, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route cs35l35_audio_map[] = {
	{"IMON ADC", NULL, "ISENSE"},
	{"VMON ADC", NULL, "VSENSE"},
	{"SDOUT", NULL, "IMON ADC"},
	{"SDOUT", NULL, "VMON ADC"},
	{"AMP Capture", NULL, "SDOUT"},
};

static const struct snd_soc_dapm_route cs35l35_classh_man_map[] = {
	{"SDIN", NULL, "AMP Playback"},
	{"Main AMP", NULL, "SDIN"},
	{"SPK", NULL, "Main AMP"},
};

static const struct snd_soc_dapm_route cs35l35_classh_auto_map[] = {
	{"SDIN", NULL, "AMP Playback"},
	{"CLASS H", NULL, "SDIN"},
	{"Main AMP", NULL, "CLASS H"},
	{"SPK", NULL, "Main AMP"},
};

static int cs35l35_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct cs35l35_private *cs35l35 = snd_soc_codec_get_drvdata(codec);

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		regmap_update_bits(cs35l35->regmap, CS35L35_CLK_CTL1,
					CS35L35_MCLK_DIS_MASK,
					0 << CS35L35_MCLK_DIS_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_PWRCTL1,
					CS35L35_DISCHG_FILT_MASK,
					0 << CS35L35_DISCHG_FILT_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_PWRCTL1,
					  CS35L35_PDN_ALL_MASK, 0);
		usleep_range(5000, 5100);
		break;
	case SND_SOC_BIAS_STANDBY:
		regmap_update_bits(cs35l35->regmap, CS35L35_PWRCTL1,
					  CS35L35_PDN_ALL_MASK, 1);
		regmap_update_bits(cs35l35->regmap, CS35L35_PWRCTL1,
					CS35L35_DISCHG_FILT_MASK,
					0 << CS35L35_DISCHG_FILT_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_CLK_CTL1,
					CS35L35_MCLK_DIS_MASK,
					0 << CS35L35_MCLK_DIS_SHIFT);
		break;
	case SND_SOC_BIAS_OFF:
		break;
	default:
		return -EINVAL;
	}
	dapm->bias_level = level;
	return 0;
}


static int cs35l35_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cs35l35_private *cs35l35 = snd_soc_codec_get_drvdata(codec);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		regmap_update_bits(cs35l35->regmap, CS35L35_CLK_CTL1,
				    CS35L35_MS_MASK, 1 << CS35L35_MS_SHIFT);
		cs35l35->slave_mode = false;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		regmap_update_bits(cs35l35->regmap, CS35L35_CLK_CTL1,
				    CS35L35_MS_MASK, 0 << CS35L35_MS_SHIFT);
		cs35l35->slave_mode = true;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		cs35l35->tdm_mode = true;
		break;
	case SND_SOC_DAIFMT_I2S:
		cs35l35->i2s_mode = true;
		break;
	case SND_SOC_DAIFMT_PDM:
		cs35l35->pdm_mode = true;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

struct cs35l35_mclk_config {
	int mclk;
	int srate;
	u8 clk_cfg;
};

static struct cs35l35_mclk_config cs35l35_clk_ctl[] = {

	/* MCLK, Sample Rate, Serial Port Cfg */
	{5644800, 44100, 0x00},
	{5644800, 88200, 0x40},
	{6144000, 48000, 0x10},
	{6144000, 96000, 0x50},
	{11289600, 44100, 0x01},
	{11289600, 88200, 0x41},
	{11289600, 176400, 0x81},
	{12000000, 44100, 0x03},
	{12000000, 48000, 0x13},
	{12000000, 88200, 0x43},
	{12000000, 96000, 0x53},
	{12000000, 176400, 0x83},
	{12000000, 192000, 0x93},
	{12288000, 48000, 0x11},
	{12288000, 96000, 0x51},
	{12288000, 192000, 0x91},
	{13000000, 44100, 0x07},
	{13000000, 48000, 0x17},
	{13000000, 88200, 0x47},
	{13000000, 96000, 0x57},
	{13000000, 176400, 0x87},
	{13000000, 192000, 0x97},
	{22579200, 44100, 0x02},
	{22579200, 88200, 0x42},
	{22579200, 176400, 0x82},
	{24000000, 44100, 0x0B},
	{24000000, 48000, 0x1B},
	{24000000, 88200, 0x4B},
	{24000000, 96000, 0x5B},
	{24000000, 176400, 0x8B},
	{24000000, 192000, 0x9B},
	{24576000, 48000, 0x12},
	{24576000, 96000, 0x52},
	{24576000, 192000, 0x92},
	{26000000, 44100, 0x0F},
	{26000000, 48000, 0x1F},
	{26000000, 88200, 0x4F},
	{26000000, 96000, 0x5F},
	{26000000, 176400, 0x8F},
	{26000000, 192000, 0x9F},
};

static int cs35l35_get_clk_config(int mclk, int srate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cs35l35_clk_ctl); i++) {
		if (cs35l35_clk_ctl[i].mclk == mclk &&
			cs35l35_clk_ctl[i].srate == srate)
			return cs35l35_clk_ctl[i].clk_cfg;
	}
	return -EINVAL;
}

static int cs35l35_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cs35l35_private *cs35l35 = snd_soc_codec_get_drvdata(codec);
	int srate = params_rate(params);
	int ret;
	u8 sp_sclks;
	int audin_format, monitor_format;
	bool vpbrstat = false;

	int clk_ctl = cs35l35_get_clk_config(cs35l35->mclk, srate);

	if (clk_ctl < 0)
		return clk_ctl;

	ret = regmap_update_bits(cs35l35->regmap, CS35L35_CLK_CTL2,
				  CS35L35_CLK_CTL2_MASK, clk_ctl);
	if (ret != 0)
		dev_err(codec->dev, "Failed to set clock state %d\n", ret);

/*
 * You can pull more Monitor data from the SDOUT pin than going to SDIN
 * Just make sure your SCLK is fast enough to fill the frame
 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		switch (params_width(params)) {
		case 8:
			audin_format = CS35L35_SDIN_DEPTH_8;
			break;
		case 16:
			audin_format = CS35L35_SDIN_DEPTH_16;
			break;
		case 24:
			audin_format = CS35L35_SDIN_DEPTH_24;
			break;
		default:
			dev_err(codec->dev, "Unsupported Width %d\n",
				params_width(params));
		}
		regmap_update_bits(cs35l35->regmap,
					CS35L35_AUDIN_DEPTH_CTL,
					CS35L35_AUDIN_DEPTH_MASK,
					audin_format <<
					CS35L35_AUDIN_DEPTH_SHIFT);
		if (cs35l35->pdata.stereo)
			regmap_update_bits(cs35l35->regmap,
					CS35L35_AUDIN_DEPTH_CTL,
					CS35L35_ADVIN_DEPTH_MASK,
					audin_format <<
					CS35L35_ADVIN_DEPTH_SHIFT);
	} else {
		switch (params_width(params)) {
		case 16:
			monitor_format = CS35L35_SDOUT_DEPTH_8;
			break;
		case 24:
			monitor_format = CS35L35_SDOUT_DEPTH_12;
			vpbrstat = true;
			break;
		case 32:
			monitor_format = CS35L35_SDOUT_DEPTH_16;
			vpbrstat = true;
			break;
		default:
			dev_err(codec->dev, "Unsupported Width %d\n",
				params_width(params));
		}

		if (vpbrstat)
			regmap_update_bits(cs35l35->regmap,
					CS35L35_SUPMON_DEPTH_CTL,
					CS35L35_VPBRSTAT_DEPTH_MASK,
					monitor_format <<
					CS35L35_VPBRSTAT_DEPTH_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_SUPMON_DEPTH_CTL,
					CS35L35_VPMON_DEPTH_MASK,
					monitor_format <<
					CS35L35_VPMON_DEPTH_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_SUPMON_DEPTH_CTL,
					CS35L35_VBSTMON_DEPTH_MASK,
					monitor_format <<
					CS35L35_VBSTMON_DEPTH_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_SPKMON_DEPTH_CTL,
					CS35L35_VMON_DEPTH_MASK,
					monitor_format <<
					CS35L35_VMON_DEPTH_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_SPKMON_DEPTH_CTL,
					CS35L35_IMON_DEPTH_MASK,
					monitor_format <<
					CS35L35_IMON_DEPTH_SHIFT);
	}

/* We have to take the SCLK to derive num sclks
 * to configure the CLOCK_CTL3 register correctly
 */
	if ((cs35l35->sclk / srate) % 4) {
		dev_err(codec->dev, "Unsupported sclk/fs ratio %d:%d\n",
					cs35l35->sclk, srate);
		return -EINVAL;
	}
	sp_sclks = ((cs35l35->sclk / srate) / 4) - 1;

	if (cs35l35->i2s_mode) {
		/* Only certain ratios are supported in I2S Slave Mode */
		if (cs35l35->slave_mode == true) {
			switch (sp_sclks) {
			case CS35L35_SP_SCLKS_32FS:
			case CS35L35_SP_SCLKS_48FS:
			case CS35L35_SP_SCLKS_64FS:
				regmap_update_bits(cs35l35->regmap,
					CS35L35_CLK_CTL3,
					CS35L35_SP_SCLKS_MASK, sp_sclks <<
					CS35L35_SP_SCLKS_SHIFT);
			break;
			default:
				dev_err(codec->dev, "ratio not supported\n");
				return -EINVAL;
			};
		} else {
			/* Only certain ratios supported in I2S MASTER Mode */
			switch (sp_sclks) {
			case CS35L35_SP_SCLKS_32FS:
			case CS35L35_SP_SCLKS_64FS:
				regmap_update_bits(cs35l35->regmap,
					CS35L35_CLK_CTL3,
					CS35L35_SP_SCLKS_MASK, sp_sclks <<
					CS35L35_SP_SCLKS_SHIFT);
			break;
			default:
				dev_err(codec->dev, "ratio not supported\n");
				return -EINVAL;
			};
		}
	}
	if (cs35l35->pdm_mode) {
		regmap_update_bits(cs35l35->regmap, CS35L35_AMP_INP_DRV_CTL,
					CS35L35_PDM_MODE_MASK,
					1 << CS35L35_PDM_MODE_SHIFT);
	} else {
		cs35l35->pdm_mode = false;
		regmap_update_bits(cs35l35->regmap, CS35L35_AMP_INP_DRV_CTL,
					CS35L35_PDM_MODE_MASK,
					0 << CS35L35_PDM_MODE_SHIFT);
	}
	return ret;
}

static const unsigned int cs35l35_src_rates[] = {
	44100, 48000, 88200, 96000, 176400, 192000
};

static const struct snd_pcm_hw_constraint_list cs35l35_constraints = {
	.count  = ARRAY_SIZE(cs35l35_src_rates),
	.list   = cs35l35_src_rates,
};

static int cs35l35_pcm_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE, &cs35l35_constraints);
	return 0;
}

static const unsigned int cs35l35_pdm_rates[] = {
	44100, 48000, 88200, 96000
};

static const struct snd_pcm_hw_constraint_list cs35l35_pdm_constraints = {
	.count  = ARRAY_SIZE(cs35l35_pdm_rates),
	.list   = cs35l35_pdm_rates,
};

static int cs35l35_pdm_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				&cs35l35_pdm_constraints);
	return 0;
}

static int cs35l35_dai_set_sysclk(struct snd_soc_dai *dai,
				int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cs35l35_private *cs35l35 = snd_soc_codec_get_drvdata(codec);

	/* Need the SCLK Frequency */
	cs35l35->sclk = freq;

	return 0;
}

static const struct snd_soc_dai_ops cs35l35_ops = {
	.startup = cs35l35_pcm_startup,
	.set_fmt = cs35l35_set_dai_fmt,
	.hw_params = cs35l35_pcm_hw_params,
	.set_sysclk = cs35l35_dai_set_sysclk,
	/*.set_tdm_slot = cs35l35_set_tdm_slot,*/
};

static const struct snd_soc_dai_ops cs35l35_pdm_ops = {
	.startup = cs35l35_pdm_startup,
	.set_fmt = cs35l35_set_dai_fmt,
	.hw_params = cs35l35_pcm_hw_params,
	.set_sysclk = cs35l35_dai_set_sysclk,
};

static struct snd_soc_dai_driver cs35l35_dai[] = {
	{
		.name = "cs35l35-pcm",
		.id = 0,
		.playback = {
			.stream_name = "AMP Playback",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_KNOT,
			.formats = CS35L35_FORMATS,
		},
		.capture = {
			.stream_name = "AMP Capture",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_KNOT,
			.formats = CS35L35_FORMATS,
		},
		.ops = &cs35l35_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "cs35l35-pdm",
		.id = 1,
		.playback = {
			.stream_name = "PDM Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_KNOT,
			.formats = CS35L35_FORMATS,
		},
		.ops = &cs35l35_pdm_ops,
	},
};

static int cs35l35_codec_set_sysclk(struct snd_soc_codec *codec,
				int clk_id, int source, unsigned int freq,
				int dir)
{
	struct cs35l35_private *cs35l35 = snd_soc_codec_get_drvdata(codec);

	switch (freq) {
	case 5644800:
	case 6144000:
	case 11289600:
	case 12000000:
	case 12288000:
	case 13000000:
	case 22579200:
	case 24000000:
	case 24576000:
	case 26000000:
		cs35l35->mclk = freq;
		break;
	default:
		dev_err(codec->dev, "Unsupported mclk rate %d\n",
					cs35l35->mclk);
		return -EINVAL;
	}
	return 0;
}

static int cs35l35_codec_probe(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct cs35l35_private *cs35l35 = snd_soc_codec_get_drvdata(codec);
	struct classh_cfg *cs35l35_classh = &cs35l35->pdata.classh_algo;
	int ret;

	/* Set Platform Data */
	if (cs35l35->pdata.bst_vctl)
		regmap_update_bits(cs35l35->regmap, CS35L35_BST_CVTR_V_CTL,
				CS35L35_BST_CTL_MASK,
				cs35l35->pdata.bst_vctl);

	if (cs35l35->pdata.gain_zc)
		regmap_update_bits(cs35l35->regmap, CS35L35_PROTECT_CTL,
				CS35L35_AMP_GAIN_ZC_MASK,
				cs35l35->pdata.gain_zc <<
				CS35L35_AMP_GAIN_ZC_SHIFT);

	if (cs35l35->pdata.aud_channel)
		regmap_update_bits(cs35l35->regmap,
				CS35L35_AUDIN_RXLOC_CTL,
				CS35L35_AUD_IN_LR_MASK,
				cs35l35->pdata.aud_channel <<
				CS35L35_AUD_IN_LR_SHIFT);

	if (cs35l35->pdata.stereo) {
		regmap_update_bits(cs35l35->regmap,
				CS35L35_ADVIN_RXLOC_CTL,
				CS35L35_ADV_IN_LR_MASK,
				cs35l35->pdata.adv_channel <<
				CS35L35_ADV_IN_LR_SHIFT);
		ret = snd_soc_add_codec_controls(codec, cs35l35_adv_controls,
					ARRAY_SIZE(cs35l35_adv_controls));
		if (ret)
			return ret;
	}

	if (cs35l35->pdata.sp_drv_str)
		regmap_update_bits(cs35l35->regmap, CS35L35_CLK_CTL1,
				CS35L35_SP_DRV_MASK,
				cs35l35->pdata.sp_drv_str <<
				CS35L35_SP_DRV_SHIFT);

	if (cs35l35->pdata.shared_bst)
		regmap_update_bits(cs35l35->regmap, CS35L35_CLASS_H_CTL,
				CS35L35_CH_STEREO_MASK,
				1 << CS35L35_CH_STEREO_SHIFT);

	if (cs35l35_classh->classh_algo_enable) {
		regmap_update_bits(cs35l35->regmap, CS35L35_CLASS_H_CTL,
					CS35L35_CH_BST_OVR_MASK,
					cs35l35_classh->classh_bst_override <<
					CS35L35_CH_BST_OVR_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_CLASS_H_CTL,
					CS35L35_CH_BST_LIM_MASK,
					cs35l35_classh->classh_bst_max_limit <<
					CS35L35_CH_BST_LIM_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_CLASS_H_CTL,
					CS35L35_CH_MEM_DEPTH_MASK,
					cs35l35_classh->classh_mem_depth <<
					CS35L35_CH_MEM_DEPTH_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_CLASS_H_HEADRM_CTL,
					CS35L35_CH_HDRM_CTL_MASK,
					cs35l35_classh->classh_headroom <<
					CS35L35_CH_HDRM_CTL_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_CLASS_H_RELEASE_RATE,
					CS35L35_CH_REL_RATE_MASK,
					cs35l35_classh->classh_release_rate <<
					CS35L35_CH_REL_RATE_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_CLASS_H_FET_DRIVE_CTL,
					CS35L35_CH_WKFET_DIS_MASK,
					cs35l35_classh->classh_wk_fet_disable <<
					CS35L35_CH_WKFET_DIS_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_CLASS_H_FET_DRIVE_CTL,
					CS35L35_CH_WKFET_DEL_MASK,
					cs35l35_classh->classh_wk_fet_delay <<
					CS35L35_CH_WKFET_DEL_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_CLASS_H_FET_DRIVE_CTL,
					CS35L35_CH_WKFET_THLD_MASK,
					cs35l35_classh->classh_wk_fet_thld <<
					CS35L35_CH_WKFET_THLD_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_CLASS_H_VP_CTL,
					CS35L35_CH_VP_AUTO_MASK,
					cs35l35_classh->classh_vpch_auto <<
					CS35L35_CH_VP_AUTO_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_CLASS_H_VP_CTL,
					CS35L35_CH_VP_RATE_MASK,
					cs35l35_classh->classh_vpch_rate <<
					CS35L35_CH_VP_RATE_SHIFT);
		regmap_update_bits(cs35l35->regmap, CS35L35_CLASS_H_VP_CTL,
					CS35L35_CH_VP_MAN_MASK,
					cs35l35_classh->classh_vpch_man <<
					CS35L35_CH_VP_MAN_SHIFT);

		ret = snd_soc_dapm_add_routes(dapm, cs35l35_classh_auto_map,
				      ARRAY_SIZE(cs35l35_classh_auto_map));
		if (ret)
			return ret;

	} else {
		ret = snd_soc_dapm_add_routes(dapm, cs35l35_classh_man_map,
				      ARRAY_SIZE(cs35l35_classh_man_map));
		if (ret)
			return ret;
		ret = snd_soc_add_codec_controls(codec, cs35l35_classh_controls,
					ARRAY_SIZE(cs35l35_classh_controls));
		if (ret)
			return ret;
	}

	return ret;
}

static struct regmap *cs35l35_get_regmap(struct device *dev)
{
	struct cs35l35_private *cs35l35 = dev_get_drvdata(dev);

	return cs35l35->regmap;
}


static struct snd_soc_codec_driver soc_codec_dev_cs35l35 = {
	.probe = cs35l35_codec_probe,
	.get_regmap = cs35l35_get_regmap,

	.set_bias_level = cs35l35_set_bias_level,
	.set_sysclk = cs35l35_codec_set_sysclk,

	.component_driver = {
		.dapm_widgets = cs35l35_dapm_widgets,
		.num_dapm_widgets = ARRAY_SIZE(cs35l35_dapm_widgets),

		.dapm_routes = cs35l35_audio_map,
		.num_dapm_routes = ARRAY_SIZE(cs35l35_audio_map),

		.controls = cs35l35_aud_controls,
		.num_controls = ARRAY_SIZE(cs35l35_aud_controls),
	}
};

static struct regmap_config cs35l35_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = CS35L35_MAX_REGISTER,
	.reg_defaults = cs35l35_reg,
	.num_reg_defaults = ARRAY_SIZE(cs35l35_reg),
	.volatile_reg = cs35l35_volatile_register,
	.readable_reg = cs35l35_readable_register,
	.precious_reg = cs35l35_precious_register,
	.cache_type = REGCACHE_RBTREE,
};

static int cs35l35_handle_of_data(struct i2c_client *i2c_client,
				struct cs35l35_platform_data *pdata)
{
	struct device_node *np = i2c_client->dev.of_node;
	struct device_node *classh;
	struct classh_cfg *classh_config = &pdata->classh_algo;
	unsigned int val32 = 0;

	if (!np)
		return 0;

	if (of_property_read_bool(np, "cirrus,boost-pdn-fet-on"))
		pdata->bst_pdn_fet_on = true;

	if (of_property_read_u32(np, "cirrus,boost-ctl", &val32) >= 0)
		pdata->bst_vctl = val32;

	if (of_property_read_u32(np, "cirrus,sp-drv-strength", &val32) >= 0)
		pdata->sp_drv_str = val32;

	if (of_property_read_u32(np, "cirrus,audio-channel", &val32) >= 0)
		pdata->aud_channel = val32;

	if (of_property_read_bool(np, "cirrus,stereo-config")) {
		pdata->stereo = true;
		if (of_property_read_u32(np, "cirrus,advisory-channel",
					&val32) >= 0)
			pdata->adv_channel = val32;
		if (of_property_read_bool(np, "cirrus,shared-boost"))
			pdata->shared_bst = true;
	}

	if (of_property_read_bool(np, "cirrus,amp-gain-zc"))
		pdata->gain_zc = true;

	classh = of_get_child_by_name(np, "classh-internal-algo");
	classh_config->classh_algo_enable = classh ? true : false;

	if (classh_config->classh_algo_enable) {
		if (of_property_read_bool(np, "cirrus,classh-bst-overide"))
			classh_config->classh_bst_override = true;
		if (of_property_read_u32(classh, "classh-bst-max-limit",
					&val32) >= 0)
			classh_config->classh_bst_max_limit = val32;
		if (of_property_read_u32(classh, "classh-mem-depth",
					&val32) >= 0)
			classh_config->classh_mem_depth = val32;
		if (of_property_read_u32(classh, "classh-release-rate",
					&val32) >= 0)
			classh_config->classh_release_rate = val32;
		if (of_property_read_u32(classh, "classh-headroom",
					&val32) >= 0)
			classh_config->classh_headroom = val32;
		if (of_property_read_u32(classh, "classh-wk-fet-disable",
					&val32) >= 0)
			classh_config->classh_wk_fet_disable = val32;
		if (of_property_read_u32(classh, "classh-wk-fet-delay",
					&val32) >= 0)
			classh_config->classh_wk_fet_delay = val32;
		if (of_property_read_u32(classh, "classh-wk-fet-thld",
					&val32) >= 0)
			classh_config->classh_wk_fet_thld = val32;
		if (of_property_read_u32(classh, "classh-vpch-auto",
					&val32) >= 0)
			classh_config->classh_vpch_auto = val32;
		if (of_property_read_u32(classh, "classh-vpch-rate",
					&val32) >= 0)
			classh_config->classh_vpch_rate = val32;
		if (of_property_read_u32(classh, "classh-vpch-man",
					&val32) >= 0)
			classh_config->classh_vpch_man = val32;
	}

	of_node_put(classh);

	return 0;
}

static int cs35l35_i2c_probe(struct i2c_client *i2c_client,
			      const struct i2c_device_id *id)
{
	struct cs35l35_private *cs35l35;
	struct cs35l35_platform_data *pdata =
		dev_get_platdata(&i2c_client->dev);
	int i;
	int ret;
	unsigned int devid = 0;
	unsigned int reg;

	cs35l35 = devm_kzalloc(&i2c_client->dev,
			       sizeof(struct cs35l35_private),
			       GFP_KERNEL);
	if (!cs35l35) {
		dev_err(&i2c_client->dev, "could not allocate codec\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c_client, cs35l35);
	cs35l35->regmap = devm_regmap_init_i2c(i2c_client, &cs35l35_regmap);
	if (IS_ERR(cs35l35->regmap)) {
		ret = PTR_ERR(cs35l35->regmap);
		dev_err(&i2c_client->dev, "regmap_init() failed: %d\n", ret);
		goto err;
	}

	for (i = 0; i < ARRAY_SIZE(cs35l35_supplies); i++)
		cs35l35->supplies[i].supply = cs35l35_supplies[i];
		cs35l35->num_supplies = ARRAY_SIZE(cs35l35_supplies);

	ret = devm_regulator_bulk_get(&i2c_client->dev,
			cs35l35->num_supplies,
			cs35l35->supplies);
	if (ret != 0) {
		dev_err(&i2c_client->dev,
			"Failed to request core supplies: %d\n",
			ret);
		return ret;
	}


	if (pdata) {
		cs35l35->pdata = *pdata;
	} else {
		pdata = devm_kzalloc(&i2c_client->dev,
				     sizeof(struct cs35l35_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&i2c_client->dev,
				"could not allocate pdata\n");
			return -ENOMEM;
		}
		if (i2c_client->dev.of_node) {
			ret = cs35l35_handle_of_data(i2c_client, pdata);
			if (ret != 0)
				return ret;

		}
		cs35l35->pdata = *pdata;
	}

	ret = regulator_bulk_enable(cs35l35->num_supplies,
					cs35l35->supplies);
	if (ret != 0) {
		dev_err(&i2c_client->dev,
			"Failed to enable core supplies: %d\n",
			ret);
		return ret;
	}

	/* returning NULL can be an option if in stereo mode */
	cs35l35->reset_gpio = devm_gpiod_get_optional(&i2c_client->dev,
		"reset", GPIOD_OUT_LOW);
	if (IS_ERR(cs35l35->reset_gpio))
		return PTR_ERR(cs35l35->reset_gpio);

	if (cs35l35->reset_gpio)
		gpiod_set_value_cansleep(cs35l35->reset_gpio, 1);

	/* initialize codec */
	ret = regmap_read(cs35l35->regmap, CS35L35_DEVID_AB, &reg);

	devid = (reg & 0xFF) << 12;
	ret = regmap_read(cs35l35->regmap, CS35L35_DEVID_CD, &reg);
	devid |= (reg & 0xFF) << 4;
	ret = regmap_read(cs35l35->regmap, CS35L35_DEVID_E, &reg);
	devid |= (reg & 0xF0) >> 4;

	if (devid != CS35L35_CHIP_ID) {
		dev_err(&i2c_client->dev,
			"CS35L35 Device ID (%X). Expected ID %X\n",
			devid, CS35L35_CHIP_ID);
		ret = -ENODEV;
		goto err;
	}

	ret = regmap_read(cs35l35->regmap, CS35L35_REV_ID, &reg);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "Get Revision ID failed\n");
		goto err;
	}

	dev_info(&i2c_client->dev,
		 "Cirrus Logic CS35L35 (%x), Revision: %02X\n", devid,
		ret & 0xFF);

	/* ADC's and Boost are not powered down by default */
	regmap_update_bits(cs35l35->regmap, CS35L35_PWRCTL2,
			CS35L35_PWR2_PDN_MASK,
			CS35L35_PWR2_PDN_MASK);

	/* PDN Boost on FET usage */
	if (cs35l35->pdata.bst_pdn_fet_on)
		regmap_update_bits(cs35l35->regmap, CS35L35_PWRCTL2,
					CS35L35_PDN_BST_MASK,
					1 << CS35L35_PDN_BST_FETON_SHIFT);
	else
		regmap_update_bits(cs35l35->regmap, CS35L35_PWRCTL2,
					CS35L35_PDN_BST_MASK,
					1 << CS35L35_PDN_BST_FETOFF_SHIFT);

	regmap_update_bits(cs35l35->regmap, CS35L35_PWRCTL3,
			CS35L35_PWR3_PDN_MASK,
			CS35L35_PWR3_PDN_MASK);

	/* Set mute bit at startup */
	regmap_update_bits(cs35l35->regmap, CS35L35_PROTECT_CTL,
		CS35L35_AMP_MUTE_MASK, 1 << CS35L35_AMP_MUTE_SHIFT);

	ret =  snd_soc_register_codec(&i2c_client->dev,
			&soc_codec_dev_cs35l35, cs35l35_dai,
			ARRAY_SIZE(cs35l35_dai));
	if (ret < 0) {
		dev_err(&i2c_client->dev,
			"%s: Register codec failed\n", __func__);
		goto err;
	}

err:
	regulator_bulk_disable(cs35l35->num_supplies,
			       cs35l35->supplies);
	return ret;
}

static int cs35l35_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct of_device_id cs35l35_of_match[] = {
	{.compatible = "cirrus,cs35l35"},
	{},
};
MODULE_DEVICE_TABLE(of, cs35l35_of_match);

static const struct i2c_device_id cs35l35_id[] = {
	{"cs35l35", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cs35l35_id);

static struct i2c_driver cs35l35_i2c_driver = {
	.driver = {
		.name = "cs35l35",
		.owner = THIS_MODULE,
		.of_match_table = cs35l35_of_match,
	},
	.id_table = cs35l35_id,
	.probe = cs35l35_i2c_probe,
	.remove = cs35l35_i2c_remove,

};

static int __init cs35l35_modinit(void)
{
	int ret;

	ret = i2c_add_driver(&cs35l35_i2c_driver);
	if (ret != 0) {
		pr_err("Failed to register CS35L35 I2C driver: %d\n", ret);
		return ret;
	}
	return 0;
}

module_init(cs35l35_modinit);

static void __exit cs35l35_exit(void)
{
	i2c_del_driver(&cs35l35_i2c_driver);
}

module_exit(cs35l35_exit);

MODULE_DESCRIPTION("ASoC CS35L35 driver");
MODULE_AUTHOR("Brian Austin, Cirrus Logic Inc, <brian.austin@cirrus.com>");
MODULE_LICENSE("GPL");
