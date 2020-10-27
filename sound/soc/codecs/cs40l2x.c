/*
 * cs40l2x.h  --  ALSA SoC Audio driver for Cirrus Logic CS40L2x
 *
 * Copyright 2019 Cirrus Logic Inc.
 *
 * Author: Paul Handrigan <paul.handrigan@cirrus.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/mfd/cs40l2x.h>

#include "cs40l2x.h"

struct cs40l2x_codec;
struct cs40l2x_private;

enum cs40l2x_clk_src {
	CS40L2X_32KHZ_CLK,
	CS40L2X_SCLK
};

struct cs40l2x_codec {
	struct cs40l2x_private *core;
	struct device *dev;
	struct regmap *regmap;
	int codec_sysclk;
};

struct cs40l2x_pll_sysclk_config {
	int freq;
	int clk_cfg;
};

static const struct cs40l2x_pll_sysclk_config cs40l2x_pll_sysclk[] = {
	{32768, 0x00},
	{1536000, 0x1b},
	{3072000, 0x21},
};

static int cs40l2x_get_clk_config(int freq)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cs40l2x_pll_sysclk); i++) {
		if (cs40l2x_pll_sysclk[i].freq == freq)
			return cs40l2x_pll_sysclk[i].clk_cfg;
	}

	return -EINVAL;
}

static int cs40l2x_swap_ext_clk(struct cs40l2x_codec *cs40l2x_codec,
					const enum cs40l2x_clk_src src)
{
	struct device *dev = cs40l2x_codec->dev;
	struct regmap *regmap = cs40l2x_codec->regmap;
	int clk_cfg, ret;
	unsigned int ack;

	if (src == CS40L2X_32KHZ_CLK)
		clk_cfg = cs40l2x_get_clk_config(CS40L2X_MCLK_FREQ);
	else
		clk_cfg = cs40l2x_get_clk_config(cs40l2x_codec->codec_sysclk);

	if (clk_cfg < 0) {
		dev_err(dev, "Invalid SYS Clock Frequency\n");
		return -EINVAL;
	}

	ret = regmap_write(regmap, CS40L2X_DSP_VIRT1_MBOX_4,
					CS40L2X_PWRCTL_FORCE_STBY);
	if (ret)
		return ret;

	ret = regmap_read(regmap, CS40L2X_DSP_VIRT1_MBOX_4,
					&ack);
	if (ret)
		return ret;

	if (ack != CS40L2X_PWRCTL_NONE) {
		dev_err(dev, "Incorrect ACK from VIRT_MBOX 4 %d\n", ack);
		return -ENXIO;
	}

	regmap_update_bits(regmap, CS40L2X_PLL_CLK_CTRL,
		CS40L2X_PLL_OPENLOOP_MASK,
		1 << CS40L2X_PLL_OPENLOOP_SHIFT);

	regmap_update_bits(regmap, CS40L2X_PLL_CLK_CTRL,
			CS40L2X_REFCLK_FREQ_MASK,
			clk_cfg << CS40L2X_REFCLK_FREQ_SHIFT);

	if (src == CS40L2X_32KHZ_CLK)
		regmap_update_bits(regmap, CS40L2X_PLL_CLK_CTRL,
			CS40L2X_PLL_CLK_SEL_MASK, CS40L2X_PLLSRC_MCLK);
	else
		regmap_update_bits(regmap, CS40L2X_PLL_CLK_CTRL,
			CS40L2X_PLL_CLK_SEL_MASK, CS40L2X_PLLSRC_SCLK);

	regmap_update_bits(regmap, CS40L2X_PLL_CLK_CTRL,
			CS40L2X_PLL_OPENLOOP_MASK,
			0 << CS40L2X_PLL_OPENLOOP_SHIFT);
	regmap_update_bits(regmap, CS40L2X_PLL_CLK_CTRL,
			CS40L2X_PLL_CLK_EN_MASK,
			1 << CS40L2X_PLL_CLK_EN_SHIFT);

	usleep_range(1000, 1500);

	return regmap_write(regmap, CS40L2X_DSP_VIRT1_MBOX_4,
					CS40L2X_PWRCTL_WAKE);
}

static int cs40l2x_clk_en(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cs40l2x_codec *cs40l2x_codec = snd_soc_codec_get_drvdata(codec);
	struct device *dev = cs40l2x_codec->dev;
	int ret = 0;

	mutex_lock(&cs40l2x_codec->core->lock);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = cs40l2x_swap_ext_clk(cs40l2x_codec, CS40L2X_SCLK);
		if (ret)
			goto err;

		cs40l2x_codec->core->a2h_enable = true;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		cs40l2x_codec->core->a2h_enable = false;
		break;
	default:
		dev_err(dev, "Invalid event %d\n", event);
		ret = -EINVAL;
	}
err:
	mutex_unlock(&cs40l2x_codec->core->lock);
	return ret;
}

static int cs40l2x_a2h_en(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cs40l2x_codec *cs40l2x_codec = snd_soc_codec_get_drvdata(codec);
	struct cs40l2x_private *core = cs40l2x_codec->core;
	struct regmap *regmap = cs40l2x_codec->regmap;
	struct device *dev = cs40l2x_codec->dev;
	unsigned int reg;
	int ret = 0;

	if (core->dsp_reg) {
		reg = core->dsp_reg(core, "A2HENABLED",
				CS40L2X_XM_UNPACKED_TYPE,
				CS40L2X_ALGO_ID_A2H);
	} else {
		dev_warn(dev, "DSP is not ready\n");
		return 0;
	}

	if (!reg) {
		dev_err(dev, "Cannot find the A2HENABLED register\n");
		return -EINVAL;
	}

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* Enable I2S in the DSP */
		ret = regmap_update_bits(regmap, CS40L2X_SP_ENABLES,
					CS40L2X_ASP_RX_ENABLE_MASK,
					CS40L2X_ASP_RX_ENABLE_MASK);
		if (ret)
			return ret;

		ret = regmap_write(regmap, CS40L2X_DSP_VIRT1_MBOX_5,
					CS40L2X_A2H_I2S_START);
		if (ret)
			return ret;

		ret = regmap_write(regmap, reg, CS40L2X_A2H_ENABLE);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		ret = cs40l2x_swap_ext_clk(cs40l2x_codec, CS40L2X_32KHZ_CLK);
		if (ret)
			return ret;

		ret = regmap_write(regmap, CS40L2X_DSP_VIRT1_MBOX_5,
					CS40L2X_A2H_I2S_END);
		if (ret)
			return ret;

		ret = regmap_write(regmap, reg, CS40L2X_A2H_DISABLE);
		break;
	default:
		dev_err(dev, "Invalid event %d\n", event);
		return -EINVAL;
	}

	return ret;
}

static const struct snd_kcontrol_new cs40l2x_a2h =
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_soc_dapm_widget cs40l2x_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY_S("AIFCLK", 100, SND_SOC_NOPM, 0, 0,
		cs40l2x_clk_en, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	/* ASPRX1 is always used in A2H */
	SND_SOC_DAPM_AIF_IN_E("ASPRX1", NULL, 0, CS40L2X_SP_ENABLES, 16, 0,
		cs40l2x_a2h_en, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_AIF_IN("ASPRX2", NULL, 0, CS40L2X_SP_ENABLES, 17, 0),
	SND_SOC_DAPM_MIXER("A2H Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SWITCH("A2H", SND_SOC_NOPM, 0, 0, &cs40l2x_a2h),
	SND_SOC_DAPM_OUTPUT("LRA"),
};

static const struct snd_soc_dapm_route cs40l2x_dapm_routes[] = {
	{ "ASPRX1", NULL, "AIF Playback" },
	{ "ASPRX2", NULL, "AIF Playback" },
	{ "A2H Mixer", NULL, "ASPRX1" },
	{ "A2H Mixer", NULL, "ASPRX2" },
	{ "A2H", "Switch", "A2H Mixer" },
	{ "LRA", NULL, "A2H" },

	{ "AIF Playback", NULL, "AIFCLK" },
};

static int cs40l2x_codec_set_sysclk(struct snd_soc_codec *codec, int clk_id,
					int source, unsigned int freq, int dir)
{
	struct cs40l2x_codec *cs40l2x = snd_soc_codec_get_drvdata(codec);
	struct regmap *regmap = cs40l2x->regmap;
	struct device *dev = cs40l2x->dev;
	int clk_cfg;

	clk_cfg = cs40l2x_get_clk_config(freq);
	if (clk_cfg < 0) {
		dev_err(dev, "Invalid Clock Frequency %d\n", freq);
		return -EINVAL;
	}

	switch (clk_id) {
	case 0:
		break;
	default:
		dev_err(dev, "Invalid Input Clock\n");
		return -EINVAL;
	}
	cs40l2x->codec_sysclk = freq;

	return regmap_write(regmap, CS40L2X_SP_RATE_CTRL, clk_cfg);
}

static int cs40l2x_pcm_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	return 0;
}

static int cs40l2x_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct cs40l2x_codec *cs40l2x =
		snd_soc_codec_get_drvdata(codec_dai->codec);
	struct device *dev = cs40l2x->dev;
	struct regmap *regmap = cs40l2x->regmap;
	unsigned int lrclk_fmt, sclk_fmt;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		dev_err(dev, "This device can be slave only\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		dev_err(dev, "Invalid format. I2S only.\n");
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
		dev_warn(dev,
			"%s: Invalid DAI clock INV\n", __func__);
		return -EINVAL;
	}

	regmap_update_bits(regmap, CS40L2X_SP_FORMAT, CS40L2X_LRCLK_INV_MASK,
				lrclk_fmt << CS40L2X_LRCLK_INV_SHIFT);
	regmap_update_bits(regmap, CS40L2X_SP_FORMAT, CS40L2X_SCLK_INV_MASK,
				sclk_fmt << CS40L2X_SCLK_INV_SHIFT);

	return 0;
}

static int cs40l2x_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct cs40l2x_codec *cs40l2x = snd_soc_codec_get_drvdata(dai->codec);
	unsigned int asp_width, asp_wl;

	asp_wl = params_width(params);
	asp_width = params_physical_width(params);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	regmap_update_bits(cs40l2x->regmap, CS40L2X_SP_FORMAT,
			CS40L2X_ASP_WIDTH_RX_MASK,
			asp_width << CS40L2X_ASP_WIDTH_RX_SHIFT);
	regmap_update_bits(cs40l2x->regmap, CS40L2X_SP_RX_WL,
			CS40L2X_ASP_RX_WL_MASK,
			asp_wl);

	return 0;
}


#define CS40L2X_RATES SNDRV_PCM_RATE_48000

#define CS40L2X_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)

static const struct snd_soc_dai_ops cs40l2x_dai_ops = {
	.startup = cs40l2x_pcm_startup,
	.set_fmt = cs40l2x_set_dai_fmt,
	.hw_params = cs40l2x_pcm_hw_params,
};

static struct snd_soc_dai_driver cs40l2x_dai[] = {
	{
		.name = "cs40l2x-pcm",
		.id = 0,
		.playback = {
			.stream_name = "AIF Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = CS40L2X_RATES,
			.formats = CS40L2X_FORMATS,
		},
		.ops = &cs40l2x_dai_ops,
		.symmetric_rates = 1,
	},
};

static int cs40l2x_codec_probe(struct snd_soc_codec *codec)
{
	struct cs40l2x_codec *cs40l2x = snd_soc_codec_get_drvdata(codec);
	struct regmap *regmap = cs40l2x->regmap;

	/* ASPRX1 --> DSP1RX1_SRC */
	regmap_write(regmap, CS40L2X_DSP1RX1_INPUT, CS40L2X_ROUTE_ASPRX1);
	/* ASPRX2 --> DSP1RX5_SRC */
	regmap_write(regmap, CS40L2X_DSP1RX5_INPUT, CS40L2X_ROUTE_ASPRX2);

	cs40l2x->codec_sysclk = CS40L2X_SCLK_DEFAULT; /* 1.536MHz */

	return 0;
}

static const struct snd_soc_codec_driver soc_codec_dev_cs40l2x = {
	.probe = cs40l2x_codec_probe,
	.set_sysclk = cs40l2x_codec_set_sysclk,

	.component_driver = {
		.dapm_widgets		= cs40l2x_dapm_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(cs40l2x_dapm_widgets),
		.dapm_routes		= cs40l2x_dapm_routes,
		.num_dapm_routes	= ARRAY_SIZE(cs40l2x_dapm_routes),
	},
	.idle_bias_off = true,
	.ignore_pmdown_time = true,
};

static int cs40l2x_probe(struct platform_device *pdev)
{
	struct cs40l2x_private *cs40l2x = dev_get_drvdata(pdev->dev.parent);
	struct cs40l2x_codec *cs40l2x_codec;
	int ret;

	cs40l2x_codec = devm_kzalloc(&pdev->dev, sizeof(struct cs40l2x_codec),
					GFP_KERNEL);
	if (!cs40l2x_codec)
		return -ENOMEM;

	cs40l2x_codec->core = cs40l2x;
	cs40l2x_codec->regmap = cs40l2x->regmap;
	cs40l2x_codec->dev = &pdev->dev;

	platform_set_drvdata(pdev, cs40l2x_codec);

	ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_cs40l2x,
				      cs40l2x_dai, ARRAY_SIZE(cs40l2x_dai));
	if (ret < 0)
		dev_err(&pdev->dev, "Failed to register codec: %d\n", ret);

	return ret;
}

static int cs40l2x_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver cs40l2x_codec_driver = {
	.driver = {
		.name = "cs40l2x-codec",
	},
	.probe = cs40l2x_probe,
	.remove = cs40l2x_remove,
};

module_platform_driver(cs40l2x_codec_driver);

MODULE_DESCRIPTION("ASoC CS40L2X driver");
MODULE_AUTHOR("Paul Handrigan <paul.handrigan@cirrus.com");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cs40l2x-codec");
