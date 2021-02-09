// SPDX-License-Identifier: GPL-2.0
//
// cs40l26.c -- ALSA SoC Audio driver for Cirrus Logic Haptic Device: CS40L26
//
// Copyright 2021 Cirrus Logic. Inc.

#include <linux/mfd/cs40l26.h>

static const struct snd_kcontrol_new cs40l26_asp =
		SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct cs40l26_pll_sysclk_config cs40l26_pll_sysclk[] = {
	{CS40L26_PLL_CLK_FRQ0, CS40L26_PLL_CLK_CFG0},
	{CS40L26_PLL_CLK_FRQ1, CS40L26_PLL_CLK_CFG1},
	{CS40L26_PLL_CLK_FRQ2, CS40L26_PLL_CLK_CFG2},
	{CS40L26_PLL_CLK_FRQ3, CS40L26_PLL_CLK_CFG3},
	{CS40L26_PLL_CLK_FRQ4, CS40L26_PLL_CLK_CFG4},
	{CS40L26_PLL_CLK_FRQ5, CS40L26_PLL_CLK_CFG5},
};

static int cs40l26_get_clk_config(u32 freq, u8 *clk_cfg)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cs40l26_pll_sysclk); i++) {
		if (cs40l26_pll_sysclk[i].freq == freq) {
			*clk_cfg = cs40l26_pll_sysclk[i].clk_cfg;
			return 0;
		}
	}

	return -EINVAL;
}

static int cs40l26_swap_ext_clk(struct cs40l26_codec *codec, u8 clk_src)
{
	struct regmap *regmap = codec->regmap;
	struct device *dev = codec->dev;
	u8 clk_cfg, clk_sel;
	int ret;

	switch (clk_src) {
	case CS40L26_PLL_REFCLK_BCLK:
		clk_sel = CS40L26_PLL_CLK_SEL_BCLK;

		ret = cs40l26_get_clk_config(codec->sysclk_rate, &clk_cfg);
		break;
	case CS40L26_PLL_REFCLK_MCLK:
		clk_sel = CS40L26_PLL_CLK_SEL_MCLK;

		ret = cs40l26_get_clk_config(CS40L26_PLL_CLK_FRQ0, &clk_cfg);
		break;
	case CS40L26_PLL_REFCLK_FSYNC:
		ret = -EPERM;
		break;
	default:
		ret = -EINVAL;
	}

	if (ret) {
		dev_err(dev, "Failed to get clock configuration\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS40L26_REFCLK_INPUT,
			CS40L26_PLL_REFCLK_OPEN_LOOP_MASK, CS40L26_ENABLE <<
			CS40L26_PLL_REFCLK_OPEN_LOOP_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to set Open-Loop PLL\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS40L26_REFCLK_INPUT,
			CS40L26_PLL_REFCLK_FREQ_MASK |
			CS40L26_PLL_REFCLK_SEL_MASK, (clk_cfg <<
			CS40L26_PLL_REFCLK_FREQ_SHIFT) | clk_sel);
	if (ret) {
		dev_err(dev, "Failed to update REFCLK input\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS40L26_REFCLK_INPUT,
			CS40L26_PLL_REFCLK_OPEN_LOOP_MASK, CS40L26_DISABLE <<
			CS40L26_PLL_REFCLK_OPEN_LOOP_SHIFT);
	if (ret)
		dev_err(dev, "Failed to close PLL loop\n");

	return ret;
}

static int cs40l26_clk_en(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_dapm_to_component(w->dapm));
	struct cs40l26_private *cs40l26 = codec->core;
	struct device *dev = cs40l26->dev;
	int ret;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		mutex_lock(&cs40l26->lock);
		cs40l26->asp_enable = true;
		mutex_unlock(&cs40l26->lock);

		ret = cs40l26_swap_ext_clk(codec, CS40L26_PLL_REFCLK_BCLK);
		if (ret)
			return ret;

		break;
	case SND_SOC_DAPM_PRE_PMD:
		ret = cs40l26_swap_ext_clk(codec, CS40L26_PLL_REFCLK_MCLK);
		if (ret)
			return ret;

		mutex_lock(&cs40l26->lock);
		cs40l26->asp_enable = false;
		mutex_unlock(&cs40l26->lock);

		break;
	default:
		dev_err(dev, "Invalid event: %d\n", event);
		return -EINVAL;
	}

	return 0;
}

static int cs40l26_a2h_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_dapm_to_component(w->dapm));
	struct cs40l26_private *cs40l26 = codec->core;
	int ret;
	u32 reg;

	ret = cl_dsp_get_reg(cs40l26->dsp, "A2HEN", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_A2H_ALGO_ID, &reg);
	if (ret)
		return ret;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		return regmap_write(cs40l26->regmap, reg, CS40L26_ENABLE);
	case SND_SOC_DAPM_PRE_PMD:
		return regmap_write(cs40l26->regmap, reg, CS40L26_DISABLE);
	default:
		dev_err(cs40l26->dev, "Invalid A2H event: %d\n", event);
		return -EINVAL;
	}
}

static int cs40l26_pcm_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_dapm_to_component(w->dapm));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	u32 asp_en_mask = CS40L26_ASP_TX1_EN_MASK | CS40L26_ASP_TX2_EN_MASK |
			CS40L26_ASP_RX1_EN_MASK | CS40L26_ASP_RX2_EN_MASK;
	u32 asp_enables;
	int ret;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = regmap_update_bits(regmap, CS40L26_DACPCM1_INPUT,
			CS40L26_DATA_SRC_MASK, CS40L26_DATA_SRC_DSP1TX1);
		if (ret) {
			dev_err(dev, "Failed to set DAC PCM input\n");
			return ret;
		}

		ret = regmap_update_bits(regmap, CS40L26_ASPTX1_INPUT,
			CS40L26_DATA_SRC_MASK, CS40L26_DATA_SRC_DSP1TX1);
		if (ret) {
			dev_err(dev, "Failed to set ASPTX1 input\n");
			return ret;
		}

		asp_enables = CS40L26_ENABLE | (CS40L26_ENABLE <<
				CS40L26_ASP_TX2_EN_SHIFT) | (CS40L26_ENABLE <<
				CS40L26_ASP_RX1_EN_SHIFT) | (CS40L26_ENABLE <<
				CS40L26_ASP_RX2_EN_SHIFT);

		ret = regmap_update_bits(regmap, CS40L26_ASP_ENABLES1,
				asp_en_mask, asp_enables);
		if (ret) {
			dev_err(dev, "Failed to enable ASP channels\n");
			return ret;
		}

		ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				CS40L26_DSP_MBOX_CMD_START_I2S,
				CS40L26_DSP_MBOX_RESET);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				CS40L26_DSP_MBOX_CMD_STOP_I2S,
				CS40L26_DSP_MBOX_RESET);
		if (ret)
			return ret;

		asp_enables = CS40L26_DISABLE | (CS40L26_DISABLE <<
				CS40L26_ASP_TX2_EN_SHIFT) | (CS40L26_DISABLE <<
				CS40L26_ASP_RX1_EN_SHIFT) | (CS40L26_DISABLE <<
				CS40L26_ASP_RX2_EN_SHIFT);

		ret = regmap_update_bits(regmap, CS40L26_ASP_ENABLES1,
				asp_en_mask, asp_enables);
		if (ret) {
			dev_err(dev, "Failed to clear ASPTX1 input\n");
			return ret;
		}

		ret = regmap_update_bits(regmap, CS40L26_ASPTX1_INPUT,
			CS40L26_DATA_SRC_MASK, CS40L26_DATA_SRC_VMON);
		if (ret)
			dev_err(dev, "Failed to set ASPTX1 input\n");
		break;
	default:
		dev_err(dev, "Invalid PCM event: %d\n", event);
		ret = -EINVAL;
	}

	return ret;
}

static const char * const cs40l26_out_mux_texts[] = { "PCM", "A2H" };
static SOC_ENUM_SINGLE_VIRT_DECL(cs40l26_out_mux_enum, cs40l26_out_mux_texts);
static const struct snd_kcontrol_new cs40l26_out_mux =
	SOC_DAPM_ENUM("Haptics Source", cs40l26_out_mux_enum);

static const struct snd_soc_dapm_widget
		cs40l26_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY_S("ASP PLL", 0, SND_SOC_NOPM, 0, 0, cs40l26_clk_en,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_AIF_IN("ASPRX1", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ASPRX2", NULL, 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_PGA_E("PCM", SND_SOC_NOPM, 0, 0, NULL, 0, cs40l26_pcm_ev,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MIXER_E("A2H", SND_SOC_NOPM, 0, 0, NULL, 0, cs40l26_a2h_ev,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_MUX("Haptics Source", SND_SOC_NOPM, 0, 0,
			&cs40l26_out_mux),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route
		cs40l26_dapm_routes[] = {
	{ "ASP Playback", NULL, "ASP PLL" },
	{ "ASPRX1", NULL, "ASP Playback" },
	{ "ASPRX2", NULL, "ASP Playback" },

	{ "PCM", NULL, "ASPRX1" },
	{ "PCM", NULL, "ASPRX2" },
	{ "A2H", NULL, "PCM" },

	{ "Haptics Source", "PCM", "PCM" },
	{ "Haptics Source", "A2H", "A2H" },
	{ "OUT", NULL, "Haptics Source" },
};

static int cs40l26_component_set_sysclk(struct snd_soc_component *component,
		int clk_id, int source, unsigned int freq, int dir)
{
	struct cs40l26_codec *codec = snd_soc_component_get_drvdata(component);
	struct device *dev = codec->dev;
	u8 clk_cfg;
	int ret;

	ret = cs40l26_get_clk_config((u32) (CS40L26_PLL_CLK_FREQ_MASK & freq),
			&clk_cfg);
	if (ret) {
		dev_err(dev, "Invalid Clock Frequency: %u Hz\n", freq);
		return ret;
	}

	if (clk_id != 0) {
		dev_err(dev, "Invalid Input Clock (ID: %d)\n", clk_id);
		return -EINVAL;
	}

	codec->sysclk_rate = (u32) (CS40L26_PLL_CLK_FREQ_MASK & freq);

	return 0;
}

static int cs40l26_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(codec_dai->component);
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	u8 lrck_fmt, sclk_fmt;
	int ret;

	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS) {
		dev_err(codec->dev, "Device can not be master\n");
		return -EINVAL;
	}

	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S) {
		dev_err(codec->dev, "Format must be I2S\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		lrck_fmt = CS40L26_CLK_NORMAL;
		sclk_fmt = CS40L26_CLK_NORMAL;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		lrck_fmt = CS40L26_CLK_INV;
		sclk_fmt = CS40L26_CLK_NORMAL;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		lrck_fmt = CS40L26_CLK_NORMAL;
		sclk_fmt = CS40L26_CLK_INV;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		lrck_fmt = CS40L26_CLK_INV;
		sclk_fmt = CS40L26_CLK_INV;
		break;
	default:
		dev_err(codec->dev, "Invalid DAI clock INV\n");
		return -EINVAL;
	}

	pm_runtime_get_sync(codec->dev);

	ret = regmap_update_bits(regmap, CS40L26_ASP_CONTROL2,
			CS40L26_ASP_BCLK_INV_MASK | CS40L26_ASP_FSYNC_INV_MASK,
			(sclk_fmt << CS40L26_ASP_BCLK_INV_SHIFT) |
			(lrck_fmt << CS40L26_ASP_FSYNC_INV_SHIFT));
	if (ret)
		dev_err(codec->dev, "Failed to update ASP settings\n");

	pm_runtime_mark_last_busy(codec->dev);
	pm_runtime_put_autosuspend(codec->dev);

	return ret;
}

static int cs40l26_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(dai->component);
	u8 asp_rx_wl, asp_rx_width;
	int ret;

	asp_rx_wl = params_width(params);
	asp_rx_width = params_physical_width(params);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK) {
		dev_err(codec->dev, "Invalid stream type\n");
		return -EINVAL;
	}

	pm_runtime_get_sync(codec->dev);

	ret = regmap_update_bits(codec->regmap, CS40L26_ASP_CONTROL2,
			CS40L26_ASP_RX_WIDTH_MASK, asp_rx_width <<
			CS40L26_ASP_RX_WIDTH_SHIFT);
	if (ret) {
		dev_err(codec->dev, "Failed to update ASP RX width\n");
		goto err_pm;
	}

	ret = regmap_update_bits(codec->regmap, CS40L26_ASP_DATA_CONTROL5,
			CS40L26_ASP_RX_WL_MASK, asp_rx_wl);
	if (ret) {
		dev_err(codec->dev, "Failed to update ASP RX WL\n");
		goto err_pm;
	}

err_pm:
	pm_runtime_mark_last_busy(codec->dev);
	pm_runtime_put_autosuspend(codec->dev);

	return ret;
}

static const struct snd_soc_dai_ops cs40l26_dai_ops = {
	.set_fmt = cs40l26_set_dai_fmt,
	.hw_params = cs40l26_pcm_hw_params,
};

static struct snd_soc_dai_driver cs40l26_dai[] = {
	{
		.name = "cs40l26-pcm",
		.id = 0,
		.playback = {
			.stream_name = "ASP Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = CS40L26_FORMATS,
		},
		.ops = &cs40l26_dai_ops,
		.symmetric_rates = 1,
	},
};

static int cs40l26_codec_probe(struct snd_soc_component *component)
{
	struct cs40l26_codec *codec = snd_soc_component_get_drvdata(component);
	struct reg_sequence dsp1rx_config[2];
	int ret;

	codec->bin_file = devm_kzalloc(codec->dev, PAGE_SIZE, GFP_KERNEL);
	if (!codec->bin_file)
		return -ENOMEM;

	codec->bin_file[PAGE_SIZE - 1] = '\0';
	snprintf(codec->bin_file, PAGE_SIZE, CS40L26_A2H_TUNING_FILE_NAME);

	/* Default audio SCLK frequency */
	codec->sysclk_rate = CS40L26_PLL_CLK_FRQ1;

	dsp1rx_config[0].reg = CS40L26_DSP1RX1_INPUT;
	dsp1rx_config[0].def = CS40L26_DATA_SRC_ASPRX1;
	dsp1rx_config[1].reg = CS40L26_DSP1RX5_INPUT;
	dsp1rx_config[1].def = CS40L26_DATA_SRC_ASPRX2;

	ret = regmap_multi_reg_write(codec->regmap, dsp1rx_config, 2);
	if (ret)
		dev_err(codec->dev, "Failed to configure ASP path\n");

	return cs40l26_pseq_multi_add_pair(codec->core, dsp1rx_config, 2,
			CS40L26_PSEQ_REPLACE);
}

static const struct snd_soc_component_driver soc_codec_dev_cs40l26 = {
	.probe = cs40l26_codec_probe,
	.set_sysclk = cs40l26_component_set_sysclk,

	.dapm_widgets = cs40l26_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cs40l26_dapm_widgets),
	.dapm_routes = cs40l26_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(cs40l26_dapm_routes),
	.controls = NULL,
	.num_controls = 0,
};

static int cs40l26_codec_driver_probe(struct platform_device *pdev)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(pdev->dev.parent);
	struct cs40l26_codec *codec;
	int ret;

	codec = devm_kzalloc(&pdev->dev, sizeof(struct cs40l26_codec),
			GFP_KERNEL);
	if (!codec)
		return -ENOMEM;

	codec->core = cs40l26;
	codec->regmap = cs40l26->regmap;
	codec->dev = &pdev->dev;

	platform_set_drvdata(pdev, codec);

	pm_runtime_enable(&pdev->dev);

	ret = snd_soc_register_component(&pdev->dev, &soc_codec_dev_cs40l26,
			cs40l26_dai, ARRAY_SIZE(cs40l26_dai));
	if (ret < 0)
		dev_err(&pdev->dev, "Failed to register codec: %d\n", ret);

	return ret;
}

static int cs40l26_codec_driver_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

static struct platform_driver cs40l26_codec_driver = {
	.driver = {
		.name = "cs40l26-codec",
	},
	.probe = cs40l26_codec_driver_probe,
	.remove = cs40l26_codec_driver_remove,
};
module_platform_driver(cs40l26_codec_driver);

MODULE_DESCRIPTION("ASoC CS40L26 driver");
MODULE_AUTHOR("Fred Treven <fred.treven@cirrus.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cs40l26-codec");
