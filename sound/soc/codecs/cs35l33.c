/*
 * cs35l33.c -- CS35L33 ALSA SoC audio driver
 *
 * Copyright 2013 Cirrus Logic, Inc.
 *
 * Author: Paul Handrigan <paul.handrigan@cirrus.com>
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
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/gpio.h>
#include <sound/cs35l33.h>

#include "cs35l33.h"

struct  cs35l33_private {
	struct snd_soc_codec *codec;
	struct cs35l33_pdata pdata;
	struct regmap *regmap;
	bool amp_cal;
	int mclk_int;
};

static const struct reg_default cs35l33_reg[] = {
	{0x6, 0x85}, /* PWR CTL 1 */
	{0x7, 0xFE}, /* PWR CTL 2 */
	{0x8, 0x0C}, /* CLK CTL */
	{0x9, 0x90}, /* BST PEAK */
	{0xA, 0x55}, /* PROTECTION CTL */
	{0xB, 0x00}, /* BST CTL 1 */
	{0xC, 0x00}, /* BST CTL 2 */
	{0xD, 0x00}, /* ADSP CTL */
	{0xE, 0xC8}, /* ADC CTL */
	{0xF, 0x14}, /* DAC CTL */
	{0x10, 0x00}, /* DAC VOL */
	{0x11, 0x04}, /* AMP CTL */
	{0x12, 0x90}, /* AMP GAIN CTL */
	{0x13, 0xFF}, /* INT MASK 1 */
	{0x14, 0xFF}, /* INT MASK 2 */
	{0x17, 0x00}, /* Diagnostic Mode Register Lock */
	{0x18, 0x40}, /* Diagnostic Mode Register Control */
	{0x19, 0x00}, /* Diagnostic Mode Register Control 2 */
	{0x23, 0x62}, /* HG MEM/LDO CTL */
	{0x24, 0x03}, /* HG RELEASE RATE */
	{0x25, 0x12}, /* LDO ENTRY DELAY */
	{0x29, 0x0A}, /* HG HEADROOM */
	{0x2A, 0x05}, /* HG ENABLE/VP CTL2 */
	{0x2D, 0x00}, /* TDM TX Control 1 (VMON) */
	{0x2E, 0x03}, /* TDM TX Control 2 (IMON) */
	{0x2F, 0x02}, /* TDM TX Control 3 (VPMON) */
	{0x30, 0x05}, /* TDM TX Control 4 (VBSTMON) */
	{0x31, 0x06}, /* TDM TX Control 5 (FLAG) */
	{0x32, 0x00}, /* TDM TX Enable 1 */
	{0x33, 0x00}, /* TDM TX Enable 2 */
	{0x34, 0x00}, /* TDM TX Enable 3 */
	{0x35, 0x00}, /* TDM TX Enable 4 */
	{0x36, 0x40}, /* TDM RX Control 1 */
	{0x37, 0x03}, /* TDM RX Control 2 */
	{0x38, 0x04}, /* TDM RX Control 3 */
	{0x39, 0x63}, /* Boost Converter Control 4 */
};

static bool cs35l33_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CS35L33_DEVID_AB:
	case CS35L33_DEVID_CD:
	case CS35L33_DEVID_E:
	case CS35L33_REV_ID:
	case CS35L33_INT_STATUS_1:
	case CS35L33_INT_STATUS_2:
	case CS35L33_HG_STATUS:
		return true;
	default:
		return false;
	}
}

static bool cs35l33_writeable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	/* these are read only registers */
	case CS35L33_DEVID_AB:
	case CS35L33_DEVID_CD:
	case CS35L33_DEVID_E:
	case CS35L33_REV_ID:
	case CS35L33_INT_STATUS_1:
	case CS35L33_INT_STATUS_2:
	case CS35L33_HG_STATUS:
		return false;
	default:
		return true;
	}
}

static bool cs35l33_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CS35L33_DEVID_AB:
	case CS35L33_DEVID_CD:
	case CS35L33_DEVID_E:
	case CS35L33_FAB_ID:
	case CS35L33_REV_ID:
	case CS35L33_PWRCTL1:
	case CS35L33_PWRCTL2:
	case CS35L33_CLK_CTL:
	case CS35L33_BST_PEAK_CTL:
	case CS35L33_PROTECT_CTL:
	case CS35L33_BST_CTL1:
	case CS35L33_BST_CTL2:
	case CS35L33_ADSP_CTL:
	case CS35L33_ADC_CTL:
	case CS35L33_DAC_CTL:
	case CS35L33_DIG_VOL_CTL:
	case CS35L33_CLASSD_CTL:
	case CS35L33_AMP_CTL:
	case CS35L33_INT_MASK_1:
	case CS35L33_INT_MASK_2:
	case CS35L33_DIAG_LOCK:
	case CS35L33_DIAG_CTRL_1:
	case CS35L33_DIAG_CTRL_2:
	case CS35L33_HG_MEMLDO_CTL:
	case CS35L33_HG_REL_RATE:
	case CS35L33_LDO_DEL:
	case CS35L33_HG_HEAD:
	case CS35L33_HG_EN:
	case CS35L33_TX_VMON:
	case CS35L33_TX_IMON:
	case CS35L33_TX_VPMON:
	case CS35L33_TX_VBSTMON:
	case CS35L33_TX_FLAG:
	case CS35L33_TX_EN1:
	case CS35L33_TX_EN2:
	case CS35L33_TX_EN3:
	case CS35L33_TX_EN4:
	case CS35L33_RX_AUD:
	case CS35L33_RX_SPLY:
	case CS35L33_RX_ALIVE:
	case CS35L33_BST_CTL4:
		return true;
	default:
		return false;
	}
}

static DECLARE_TLV_DB_SCALE(classd_ctl_tlv, 900, 100, 0);
static DECLARE_TLV_DB_SCALE(dac_tlv, -10200, 50, 0);

static const struct snd_kcontrol_new cs35l33_snd_controls[] = {

	SOC_SINGLE_TLV("SPK Amp Volume", CS35L33_AMP_CTL,
		       4, 0x09, 0, classd_ctl_tlv),
	SOC_SINGLE_SX_TLV("DAC Volume", CS35L33_DIG_VOL_CTL,
			0, 0x34, 0xE4, dac_tlv),
	SOC_SINGLE("Monitor Select", CS35L33_PWRCTL2, 3, 1, 1),
};

static int cs35l33_spkrdrv_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct cs35l33_private *priv = snd_soc_codec_get_drvdata(codec);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		if (!priv->amp_cal) {
			msleep(8);
			priv->amp_cal = true;
			snd_soc_update_bits(codec, CS35L33_CLASSD_CTL,
				    AMP_CAL, 0);
		}
	break;
	default:
		pr_err("Invalid event = 0x%x\n", event);

	}

	return 0;
}

static int cs35l33_sdin_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct cs35l33_private *priv = snd_soc_codec_get_drvdata(codec);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, CS35L33_PWRCTL1,
				    PDN_BST, 0);
		break;
	case SND_SOC_DAPM_POST_PMU:
		if (!priv->amp_cal) {
			snd_soc_update_bits(codec, CS35L33_CLASSD_CTL,
				    AMP_CAL, AMP_CAL);
			msleep(10);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, CS35L33_PWRCTL1,
				    PDN_BST, PDN_BST);
		break;
	default:
		pr_err("Invalid event = 0x%x\n", event);

	}

	return 0;
}

static const struct snd_kcontrol_new imon_ctl =
	SOC_DAPM_SINGLE("Switch", CS35L33_PWRCTL2, 6, 1, 1);

static const struct snd_kcontrol_new vmon_ctl =
	SOC_DAPM_SINGLE("Switch", CS35L33_PWRCTL2, 7, 1, 1);

static const struct snd_kcontrol_new vpmon_ctl =
	SOC_DAPM_SINGLE("Switch", CS35L33_PWRCTL2, 5, 1, 1);

static const struct snd_kcontrol_new vbstmon_ctl =
	SOC_DAPM_SINGLE("Switch", CS35L33_PWRCTL2, 4, 1, 1);

static const struct snd_soc_dapm_widget cs35l33_dapm_widgets[] = {

	SND_SOC_DAPM_OUTPUT("SPK"),
	SND_SOC_DAPM_OUT_DRV_E("SPKDRV", CS35L33_PWRCTL1, 7, 1, NULL, 0,
		cs35l33_spkrdrv_event, SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_AIF_IN_E("SDIN", NULL, 0, CS35L33_PWRCTL2,
		2, 1, cs35l33_sdin_event, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_INPUT("VP"),
	SND_SOC_DAPM_INPUT("ISENSE"),
	SND_SOC_DAPM_INPUT("VSENSE"),
	SND_SOC_DAPM_INPUT("VBST"),

	SND_SOC_DAPM_SWITCH("VMON ADC", CS35L33_PWRCTL2, 7, 1, &vmon_ctl),
	SND_SOC_DAPM_SWITCH("IMON ADC", CS35L33_PWRCTL2, 6, 1, &imon_ctl),
	SND_SOC_DAPM_SWITCH("VPMON ADC", CS35L33_PWRCTL2, 5, 1, &vpmon_ctl),
	SND_SOC_DAPM_SWITCH("VBSTMON ADC", CS35L33_PWRCTL2, 4, 1,
		&vbstmon_ctl),

	SND_SOC_DAPM_AIF_OUT("SDOUT", NULL, 0, CS35L33_PWRCTL2, 3, 1),

};

static const struct snd_soc_dapm_route cs35l33_audio_map[] = {
	{"SDIN", NULL, "CS35L33 Playback"},
	{"SPKDRV", NULL, "SDIN"},
	{"SPK", NULL, "SPKDRV"},

	{"VMON ADC", "Switch", "VP"},
	{"IMON ADC", "Switch", "ISENSE"},
	{"VPMON ADC", "Switch", "VSENSE"},
	{"VBSTMON ADC", "Switch", "VBST"},

	{"SDOUT", NULL, "IMON ADC"},
	{"SDOUT", NULL, "VMON ADC"},
	{"SDOUT", NULL, "VBSTMON ADC"},
	{"CS35L33 Capture", NULL, "SDOUT"},
};

static int cs35l33_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		snd_soc_update_bits(codec, CS35L33_PWRCTL1,
				    PDN_ALL, 0);
		break;
	case SND_SOC_BIAS_STANDBY:
		snd_soc_update_bits(codec, CS35L33_PWRCTL1,
				    PDN_ALL, PDN_ALL);
		break;
	case SND_SOC_BIAS_OFF:
		break;
	default:
		return -EINVAL;
	}
	codec->dapm.bias_level = level;
	return 0;
}

struct cs35l33_mclk_div {
	int mclk;
	int srate;
	u8 adsp_rate;
	u8 int_fs_ratio;
};

static struct cs35l33_mclk_div cs35l33_mclk_coeffs[] = {

	/* MCLK, Sample Rate, adsp_rate, int_fs_ratio */
	{5644800, 11025, 0x4, INT_FS_RATE},
	{5644800, 22050, 0x8, INT_FS_RATE},
	{5644800, 44100, 0xC, INT_FS_RATE},

	{6000000,  8000, 0x1, 0},
	{6000000, 11025, 0x2, 0},
	{6000000, 11029, 0x3, 0},
	{6000000, 12000, 0x4, 0},
	{6000000, 16000, 0x5, 0},
	{6000000, 22050, 0x6, 0},
	{6000000, 22059, 0x7, 0},
	{6000000, 24000, 0x8, 0},
	{6000000, 32000, 0x9, 0},
	{6000000, 44100, 0xA, 0},
	{6000000, 44118, 0xB, 0},
	{6000000, 48000, 0xC, 0},

	{6144000,  8000, 0x1, INT_FS_RATE},
	{6144000, 12000, 0x4, INT_FS_RATE},
	{6144000, 16000, 0x5, INT_FS_RATE},
	{6144000, 24000, 0x8, INT_FS_RATE},
	{6144000, 32000, 0x9, INT_FS_RATE},
	{6144000, 48000, 0xC, INT_FS_RATE},
};

static int cs35l33_get_mclk_coeff(int mclk, int srate)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(cs35l33_mclk_coeffs); i++) {
		if (cs35l33_mclk_coeffs[i].mclk == mclk &&
			cs35l33_mclk_coeffs[i].srate == srate)
			return i;
	}
	return -EINVAL;
}

static int cs35l33_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		snd_soc_update_bits(codec, CS35L33_ADSP_CTL,
				    MS_MASK, MS_MASK);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		snd_soc_update_bits(codec, CS35L33_ADSP_CTL,
				    MS_MASK, 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int cs35l33_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cs35l33_private *priv = snd_soc_codec_get_drvdata(codec);
	int srate = params_rate(params);
	u8 clk_reg;
	int coeff = cs35l33_get_mclk_coeff(priv->mclk_int, srate);
	if (coeff < 0)
		return coeff;

	clk_reg = snd_soc_read(codec, CS35L33_CLK_CTL);
	clk_reg |= (cs35l33_mclk_coeffs[coeff].int_fs_ratio |
		cs35l33_mclk_coeffs[coeff].adsp_rate);

	snd_soc_write(codec, CS35L33_CLK_CTL, clk_reg);
	return 0;
}

static int cs35l33_src_rates[] = {
	8000, 11025, 11029, 12000, 16000, 22050,
	22059, 24000, 32000, 44100, 44118, 48000
};


static struct snd_pcm_hw_constraint_list cs35l33_constraints = {
	.count  = ARRAY_SIZE(cs35l33_src_rates),
	.list   = cs35l33_src_rates,
};

static int cs35l33_pcm_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	snd_pcm_hw_constraint_list(substream->runtime, 0,
					SNDRV_PCM_HW_PARAM_RATE,
					&cs35l33_constraints);
	return 0;
}


static int cs35l33_set_tristate(struct snd_soc_dai *dai, int tristate)
{
	struct snd_soc_codec *codec = dai->codec;
	if (tristate)
		snd_soc_update_bits(codec, CS35L33_PWRCTL2,
		SDOUT_3ST_I2S, SDOUT_3ST_I2S);
	else
		snd_soc_update_bits(codec, CS35L33_PWRCTL2,
		SDOUT_3ST_I2S, 0);
	return 0;
}

static int cs35l33_codec_set_sysclk(struct snd_soc_codec *codec,
		int clk_id, int source, unsigned int freq, int dir)
{
	struct cs35l33_private *cs35l33 = snd_soc_codec_get_drvdata(codec);
	switch (freq) {
	case CS35L33_MCLK_5644:
	case CS35L33_MCLK_6:
	case CS35L33_MCLK_6144:
		snd_soc_update_bits(codec, CS35L33_CLK_CTL,
			MCLKDIV2, 0);
		cs35l33->mclk_int = freq;
		break;
	case CS35L33_MCLK_11289:
	case CS35L33_MCLK_12:
	case CS35L33_MCLK_12288:
		snd_soc_update_bits(codec, CS35L33_CLK_CTL,
			MCLKDIV2, MCLKDIV2);
			cs35l33->mclk_int = freq/2;
		break;
	default:
		cs35l33->mclk_int = 0;
		return -EINVAL;
	}
	return 0;
}

static const struct snd_soc_dai_ops cs35l33_ops = {
	.startup = cs35l33_pcm_startup,
	.set_tristate = cs35l33_set_tristate,
	.set_fmt = cs35l33_set_dai_fmt,
	.hw_params = cs35l33_pcm_hw_params,
};

static struct snd_soc_dai_driver cs35l33_dai = {
		.name = "cs35l33",
		.id = 0,
		.playback = {
			.stream_name = "CS35L33 Playback",
			.channels_min = 1,
			.channels_max = 1,
			.rates = CS35L33_RATES,
			.formats = CS35L33_FORMATS,
		},
		.capture = {
			.stream_name = "CS35L33 Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = CS35L33_RATES,
			.formats = CS35L33_FORMATS,
		},
		.ops = &cs35l33_ops,
		.symmetric_rates = 1,
};

static int cs35l33_probe(struct snd_soc_codec *codec)
{
	struct cs35l33_private *cs35l33 = snd_soc_codec_get_drvdata(codec);
	u8 reg;

	reg = snd_soc_read(codec, CS35L33_PROTECT_CTL);
	reg &= ~(1 << 2);
	reg |= 1 << 3;
	snd_soc_write(codec, CS35L33_PROTECT_CTL, reg);
	snd_soc_update_bits(codec, CS35L33_BST_CTL2,
				    ALIVE_WD_DIS2, ALIVE_WD_DIS2);


	/* Set Platform Data */
	if (cs35l33->pdata.boost_ctl)
		snd_soc_update_bits(codec, CS35L33_BST_CTL1,
				   CS35L33_BST_CTL_MASK,
				cs35l33->pdata.boost_ctl);

	if (cs35l33->pdata.gain_zc)
		snd_soc_update_bits(codec, CS35L33_CLASSD_CTL,
				   GAIN_CHG_ZC_MASK ,
				cs35l33->pdata.gain_zc <<
				GAIN_CHG_ZC_SHIFT);

	if (cs35l33->pdata.amp_drv_sel)
		snd_soc_update_bits(codec, CS35L33_CLASSD_CTL,
				   AMP_DRV_SEL_MASK,
				cs35l33->pdata.amp_drv_sel <<
				AMP_DRV_SEL_SHIFT);

	return 0;
}

static int cs35l33_remove(struct snd_soc_codec *codec)
{
	cs35l33_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static struct regmap *cs35l33_get_regmap(struct device *dev)
{
       struct cs35l33_private *cs35l33 = dev_get_drvdata(dev);

       return cs35l33->regmap;
}

static struct snd_soc_codec_driver soc_codec_dev_cs35l33 = {
	.probe = cs35l33_probe,
	.remove = cs35l33_remove,

	.get_regmap = cs35l33_get_regmap,
	.set_bias_level = cs35l33_set_bias_level,
	.set_sysclk = cs35l33_codec_set_sysclk,

	.dapm_widgets = cs35l33_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cs35l33_dapm_widgets),
	.dapm_routes = cs35l33_audio_map,
	.num_dapm_routes = ARRAY_SIZE(cs35l33_audio_map),

	.controls = cs35l33_snd_controls,
	.num_controls = ARRAY_SIZE(cs35l33_snd_controls),
};

static struct regmap_config cs35l33_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = CS35L33_MAX_REGISTER,
	.reg_defaults = cs35l33_reg,
	.num_reg_defaults = ARRAY_SIZE(cs35l33_reg),
	.volatile_reg = cs35l33_volatile_register,
	.readable_reg = cs35l33_readable_register,
	.writeable_reg = cs35l33_writeable_register,
	.cache_type = REGCACHE_RBTREE,
};

static int cs35l33_i2c_probe(struct i2c_client *i2c_client,
				       const struct i2c_device_id *id)
{
	struct cs35l33_private *cs35l33;
	struct cs35l33_pdata *pdata = dev_get_platdata(&i2c_client->dev);
	int ret, devid;
	unsigned int reg;
	u32 val32;

	cs35l33 = kzalloc(sizeof(struct cs35l33_private), GFP_KERNEL);
	if (!cs35l33) {
		dev_err(&i2c_client->dev,
			"could not allocate codec\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c_client, cs35l33);
	cs35l33->regmap = regmap_init_i2c(i2c_client, &cs35l33_regmap);
	if (IS_ERR(cs35l33->regmap)) {
		ret = PTR_ERR(cs35l33->regmap);
		dev_err(&i2c_client->dev, "regmap_init() failed: %d\n", ret);
		goto err;
	}

	if (pdata) {
		cs35l33->pdata = *pdata;
	} else {
		pdata = devm_kzalloc(&i2c_client->dev,
				     sizeof(struct cs35l33_pdata),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&i2c_client->dev, "could not allocate pdata\n");
			return -ENOMEM;
		}
		if (i2c_client->dev.of_node) {
			if (of_property_read_u32(i2c_client->dev.of_node,
				"boost_ctl", &val32) >= 0)
				pdata->boost_ctl = val32;

			if (of_property_read_u32(i2c_client->dev.of_node,
				"gain_zc", &val32) >= 0)
				pdata->gain_zc = val32;

			if (of_property_read_u32(i2c_client->dev.of_node,
				"amp_drv_sel", &val32) >= 0)
				pdata->amp_drv_sel = val32;

			if (of_property_read_u32(i2c_client->dev.of_node,
				"gpio_nreset", &val32) >= 0)
				pdata->gpio_nreset = val32;
		}
		cs35l33->pdata = *pdata;
	}

	/* We could issue !RST or skip it based on AMP topology */
	if (cs35l33->pdata.gpio_nreset) {

		ret = gpio_request(cs35l33->pdata.gpio_nreset,
				   "CS35L33 /RST");
		if (ret < 0) {
			dev_err(&i2c_client->dev,
				"Failed to request /RST %d: %d\n",
				cs35l33->pdata.gpio_nreset, ret);
			return ret;
		}
		gpio_direction_output(cs35l33->pdata.gpio_nreset, 0);
		gpio_set_value_cansleep(cs35l33->pdata.gpio_nreset, 1);
	}

	/* initialize codec */
	ret = regmap_read(cs35l33->regmap, CS35L33_DEVID_AB, &reg);
	devid = (reg & 0xFF) << 12;
	ret = regmap_read(cs35l33->regmap, CS35L33_DEVID_CD, &reg);
	devid |= (reg & 0xFF) << 4;
	ret = regmap_read(cs35l33->regmap, CS35L33_DEVID_E, &reg);
	devid |= (reg & 0xF0) >> 4;

	if (devid != CS35L33_CHIP_ID) {
		dev_err(&i2c_client->dev,
			"CS35L33 Device ID (%X). Expected ID %X\n",
			devid, CS35L33_CHIP_ID);
		goto err_regmap;
	}

	ret = regmap_read(cs35l33->regmap, CS35L33_REV_ID, &reg);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "Get Revision ID failed\n");
		goto err_regmap;
	}

	dev_info(&i2c_client->dev,
		 "Cirrus Logic CS35L33, Revision: %02X\n", ret & 0xFF);

	ret =  snd_soc_register_codec(&i2c_client->dev,
			&soc_codec_dev_cs35l33, &cs35l33_dai, 1);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "%s: Register codec failed\n",
			__func__);
		goto err_regmap;
	}


	return 0;

err_regmap:
	regmap_exit(cs35l33->regmap);

err:
	return ret;
}

static int cs35l33_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct of_device_id cs35l33_of_match[] = {
	{ .compatible = "crus,cs35l33", },
	{},
};
MODULE_DEVICE_TABLE(of, cs35l33_of_match);

static const struct i2c_device_id cs35l33_id[] = {
	{"cs35l33", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cs35l33_id);

static struct i2c_driver cs35l33_i2c_driver = {
	.driver = {
		.name = "cs35l33",
		.owner = THIS_MODULE,
		.of_match_table = cs35l33_of_match,

		},
	.id_table = cs35l33_id,
	.probe = cs35l33_i2c_probe,
	.remove = cs35l33_i2c_remove,

};
module_i2c_driver(cs35l33_i2c_driver);

MODULE_DESCRIPTION("ASoC CS35L33 driver");
MODULE_AUTHOR("Paul Handrigan, Cirrus Logic Inc, <paul.handrigan@cirrus.com>");
MODULE_LICENSE("GPL");
