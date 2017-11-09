/*
 * cs35l36.c -- CS35L36 ALSA SoC audio driver
 *
 * Copyright 2017 Cirrus Logic, Inc.
 *
 * Author: Vlad Karpovich  <Vlad.Karpovich@cirrus.com>
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
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/cs35l36.h>
#include <linux/soundwire/soundwire.h>

#include "cs35l36.h"

#define CS35L36_MAX_SWR_PORTS	4
enum {
	SWR_DAC_PORT,
	SWR_COMP_PORT,
	SWR_BOOST_PORT,
	SWR_VISENSE_PORT,
};

#define CS35L36_MAX_SWR_PORTS	4
struct swr_port {
	u8 port_id;
	u8 ch_mask;
	u32 ch_rate;
	u8 num_ch;
};

struct  cs35l36_swr_private {
	struct device *dev;
	struct regmap *regmap;
	struct i2c_client *i2c_client;
	struct swr_device *swr_slave;
	struct swr_port port[CS35L36_MAX_SWR_PORTS];
};

static const struct snd_kcontrol_new swr_dac_port[] = {
		SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static int cs35l36_main_amp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cs35l36_swr_private *cs35l36 = snd_soc_codec_get_drvdata(codec);
	u32 reg;
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:

		regmap_update_bits(cs35l36->regmap, CS35L36_PWR_CTRL1,
					CS35L36_GLOBAL_EN_MASK,
					1 << CS35L36_GLOBAL_EN_SHIFT);
		usleep_range(2000, 2100);

		regmap_read(cs35l36->regmap, CS35L36_INT4_RAW_STATUS, &reg);
		if (reg & CS35L36_PLL_UNLOCK_MASK)
			dev_crit(cs35l36->dev, "PLL Unlocked\n");

		regmap_update_bits(cs35l36->regmap, CS35L36_AMP_OUT_MUTE,
					CS35L36_AMP_MUTE_MASK,
					0 << CS35L36_AMP_MUTE_SHIFT);
		break;
	case SND_SOC_DAPM_POST_PMD:

		regmap_update_bits(cs35l36->regmap, CS35L36_AMP_OUT_MUTE,
					CS35L36_AMP_MUTE_MASK,
					1 << CS35L36_AMP_MUTE_SHIFT);

		regmap_update_bits(cs35l36->regmap, CS35L36_PWR_CTRL1,
					CS35L36_GLOBAL_EN_MASK,
					0 << CS35L36_GLOBAL_EN_SHIFT);
		usleep_range(2000, 2100);
		break;
	default:
		dev_dbg(codec->dev, "Invalid event = 0x%x\n", event);
	}
	return ret;
}

static int cs35l36_swr_set_port(struct snd_soc_codec *codec, int port_idx,
			u8 *port_id, u8 *num_ch, u8 *ch_mask, u32 *ch_rate)
{
	struct cs35l36_swr_private *cs35l36 = snd_soc_codec_get_drvdata(codec);

	*port_id = cs35l36->port[port_idx].port_id;
	*num_ch = cs35l36->port[port_idx].num_ch;
	*ch_mask = cs35l36->port[port_idx].ch_mask;
	*ch_rate = cs35l36->port[port_idx].ch_rate;
	return 0;
}

static void cs35l36_swr_enable(struct cs35l36_swr_private *cs35l36)
{

	/* set input select to SWIRR RX */
	regmap_update_bits(cs35l36->regmap, CS35L36_ASP_RX1_SEL,
					CS35L36_PCM_RX_SEL_MASK,
					CS35L36_PCM_RX_SEL_SWIRE <<
					CS35L36_PCM_RX_SEL_SHIFT);
	 /* PLL_REFCLK_EN */
	regmap_update_bits(cs35l36->regmap, CS35L36_PLL_CLK_CTRL,
				CS35L36_PLL_REFCLK_EN_MASK,
				CS35L36_PLL_REFCLK_EN_MASK);

	/* Internal Rate = 48khz */
	regmap_update_bits(cs35l36->regmap,
			   CS35L36_GLOBAL_CLK_CTRL,
			   CS35L36_GLOBAL_FS_MASK,
			   CS35L36_GLOBAL_FS_48K <<
			   CS35L36_GLOBAL_FS_SHIFT);

	/* AMP uses PDM audio */
	regmap_update_bits(cs35l36->regmap, CS35L36_DAC_MSM_CFG,
				CS35L36_PDM_MODE_MASK,
				1 << CS35L36_PDM_MODE_SHIFT);

	regmap_update_bits(cs35l36->regmap,
				CS35L36_PWR_CTRL1,
				CS35L36_GLOBAL_EN_MASK,
				1 << CS35L36_GLOBAL_EN_SHIFT);

	regcache_sync(cs35l36->regmap);
}

int cs35l36_swr_enable_swr_dac_port(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct cs35l36_swr_private *cs35l36 = snd_soc_codec_get_drvdata(codec);
	u8 port_id[CS35L36_MAX_SWR_PORTS];
	u8 num_ch[CS35L36_MAX_SWR_PORTS];
	u8 ch_mask[CS35L36_MAX_SWR_PORTS];
	u32 ch_rate[CS35L36_MAX_SWR_PORTS];
	u8 num_port = 0;

	if (cs35l36 == NULL)
		return -EINVAL;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		cs35l36_swr_set_port(codec, SWR_DAC_PORT,
				&port_id[num_port], &num_ch[num_port],
				&ch_mask[num_port], &ch_rate[num_port]);
		dev_dbg(codec->dev,
			"set port id 0x%x, ch 0x%x,mask 0x%x,rate 0x%x\n",
			port_id[num_port], num_ch[num_port],
			ch_mask[num_port], ch_rate[num_port]);
		++num_port;
		swr_connect_port(cs35l36->swr_slave, &port_id[0], num_port,
				&ch_mask[0], &ch_rate[0], &num_ch[0]);
		cs35l36_swr_enable(cs35l36);

		regmap_update_bits(cs35l36->regmap, CS35L36_PWR_CTRL2,
					CS35L36_AMP_EN_MASK,
					1 << CS35L36_AMP_EN_SHIFT);
		break;
	case SND_SOC_DAPM_POST_PMU:
		break;
	case SND_SOC_DAPM_PRE_PMD:
		break;
	case SND_SOC_DAPM_POST_PMD:
		dev_dbg(codec->dev, "stop port %x\n",
			cs35l36->port[SWR_DAC_PORT].port_id);

		regmap_update_bits(cs35l36->regmap, CS35L36_PWR_CTRL2,
					CS35L36_AMP_EN_MASK,
					0 << CS35L36_AMP_EN_SHIFT);

		port_id[num_port] = cs35l36->port[SWR_DAC_PORT].port_id;
		++num_port;
		swr_disconnect_port(cs35l36->swr_slave, &port_id[0], num_port);
		break;
	default:
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget cs35l36_dapm_widgets[] = {
	SND_SOC_DAPM_OUT_DRV_E("Main AMP", CS35L36_PWR_CTRL2, 0, 0, NULL, 0,
		cs35l36_main_amp_event, SND_SOC_DAPM_POST_PMD |
				SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_OUTPUT("SPKR"),
	SND_SOC_DAPM_INPUT("IN"),
	SND_SOC_DAPM_MIXER_E("SWR DAC_Port", SND_SOC_NOPM, 0, 0, swr_dac_port,
		ARRAY_SIZE(swr_dac_port), cs35l36_swr_enable_swr_dac_port,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route cs35l36_audio_map[] = {
	{"SWR DAC_Port", "Switch", "IN"},
	{"Main AMP", NULL, "SWR DAC_Port"},
	{"SPKR", NULL, "Main AMP"},
};
static int cs35l36_codec_probe(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_cs35l36 = {
	.probe = &cs35l36_codec_probe,
	.component_driver = {
		.dapm_widgets = cs35l36_dapm_widgets,
		.num_dapm_widgets = ARRAY_SIZE(cs35l36_dapm_widgets),

		.dapm_routes = cs35l36_audio_map,
		.num_dapm_routes = ARRAY_SIZE(cs35l36_audio_map),
	},
	.ignore_pmdown_time = true,
};

static const struct of_device_id cs35l36_of_match[] = {
	{.compatible = "cirrus,cs35l36-swr"},
	{},
};

MODULE_DEVICE_TABLE(of, cs35l36_of_match);

int cs35l36_swr_set_channel_map(struct snd_soc_codec *codec,
				u8 *port, u8 num_port,
				unsigned int *ch_mask, unsigned int *ch_rate)
{
	struct cs35l36_swr_private *cs35l36 = snd_soc_codec_get_drvdata(codec);
	int i;

	if (!port || !ch_mask || !ch_rate ||
		(num_port > CS35L36_MAX_SWR_PORTS)) {
		dev_err(codec->dev,
			"%s: Invalid port=%p, ch_mask=%p, ch_rate=%p\n",
			__func__, port, ch_mask, ch_rate);
		return -EINVAL;
	}
	for (i = 0; i < num_port; i++) {
		cs35l36->port[i].port_id = port[i];
		cs35l36->port[i].ch_mask = ch_mask[i];
		cs35l36->port[i].ch_rate = ch_rate[i];
		cs35l36->port[i].num_ch = __sw_hweight8(ch_mask[i]);
	}
	return 0;
}
EXPORT_SYMBOL(cs35l36_swr_set_channel_map);

static int cs35l36_swr_startup(struct swr_device *swr_dev)
{
	int ret = 0;
	u8 devnum = 0;
	struct cs35l36_swr_private *cs35l36;

	dev_dbg(&swr_dev->dev, "%s: started\n", __func__);

	cs35l36 = swr_get_dev_data(swr_dev);
	if (!cs35l36) {
		dev_err(&swr_dev->dev, "%s: cs35l36 is NULL\n", __func__);
		return -EINVAL;
	}

	/*
	 * Add 5msec delay to provide sufficient time for
	 * soundwire auto enumeration of slave devices as
	 * as per HW requirement.
	 */
	usleep_range(5000, 5010);
	ret = swr_get_logical_dev_num(swr_dev, swr_dev->addr, &devnum);
	if (ret) {
		dev_err(&swr_dev->dev, "%s failed to get devnum, err:%d\n",
			__func__, ret);
		goto err;
	}
	swr_dev->dev_num = devnum;

	if (IS_ERR(cs35l36->regmap)) {
		ret = PTR_ERR(cs35l36->regmap);
		dev_err(&swr_dev->dev, "%s: regmap_init failed %d\n",
			__func__, ret);
		goto err;
	}

	ret = snd_soc_register_codec(&swr_dev->dev, &soc_codec_dev_cs35l36,
					NULL, 0);

	if (ret) {
		dev_err(&swr_dev->dev, "%s: Codec registration failed\n",
			__func__);
		goto err;
	}
err:
	dev_dbg(&swr_dev->dev, "%s: done with err %d  dev num %d\n",
				__func__, ret, swr_dev->dev_num);
	return ret;
}

static int cs35l36_swr_probe(struct swr_device *pdev)
{
	struct cs35l36_swr_private *cs35l36;
	struct cs35l36_private *cs35l36_i2c;
	struct device_node *i2c_node;
	struct i2c_client *i2c_client;
	int ret = 0;
	u32 reg_id;

	cs35l36 = devm_kzalloc(&pdev->dev, sizeof(struct cs35l36_swr_private),
			    GFP_KERNEL);
	if (!cs35l36) {
		dev_err(&pdev->dev, "%s: can't allocate cs35l36_swr\n",
			__func__);
		return -ENOMEM;
	}

	swr_set_dev_data(pdev, cs35l36);

	i2c_node = of_parse_phandle(pdev->dev.of_node, "i2c_device", 0);
	if (!i2c_node) {
		dev_err(&pdev->dev, "Failed to find i2c node in device tree\n");
		ret = -ENODEV;
		goto err;
	}

	i2c_client = of_find_i2c_device_by_node(i2c_node);
	if (!i2c_client) {
		dev_err(&pdev->dev, "Failed to get hdmi phy i2c client\n");
		ret = -EPROBE_DEFER;
	}

	of_node_put(i2c_node);

	cs35l36_i2c = dev_get_drvdata(&i2c_client->dev);
	if (!cs35l36) {
		dev_err(&pdev->dev, "Failed to get provate data\n");
		ret = -EPROBE_DEFER;
	}

	cs35l36->i2c_client = i2c_client;

	swr_set_dev_data(pdev, cs35l36);
	cs35l36->swr_slave = pdev;
	cs35l36->dev = &pdev->dev;
	cs35l36->regmap = cs35l36_i2c->regmap;

	/* Check device ID */
	ret = regmap_read(cs35l36->regmap, CS35L36_SW_RESET, &reg_id);
	if (ret < 0) {
		dev_err(&pdev->dev, "Get Device ID failed %d\n", ret);
		goto err;
	}

	if (reg_id != CS35L36_CHIP_ID) {
		dev_err(&pdev->dev,
			"CS35L36 Device ID (%X). Expected ID %X\n",
			reg_id, CS35L36_CHIP_ID);
		ret = -ENODEV;
		goto err;
	}

	dev_info(&pdev->dev, "CS35L36 Device ID (%X)", reg_id);
err:
	return ret;
}

static int cs35l36_swr_remove(struct swr_device *pdev)
{
	struct cs35l36_swr_private *cs35l36;

	cs35l36 = swr_get_dev_data(pdev);
	if (!cs35l36) {
		dev_err(&pdev->dev, "%s: cs35l36 is NULL\n", __func__);
		return -EINVAL;
	}
	snd_soc_unregister_codec(&pdev->dev);
	swr_set_dev_data(pdev, NULL);

	return 0;
}

static int cs35l36_swr_up(struct swr_device *pdev)
{
	dev_dbg(&pdev->dev, "%s: SWR up\n", __func__);

	return 0;
}

static int cs35l36_swr_down(struct swr_device *pdev)
{
	dev_dbg(&pdev->dev, "%s: SWR down\n", __func__);

	return 0;
}

static int cs35l36_swr_reset(struct swr_device *pdev)
{
	struct cs35l36_swr_private *cs35l36;

	cs35l36 = swr_get_dev_data(pdev);
	if (!cs35l36) {
		dev_err(&pdev->dev, "%s: cs35l36 is NULL\n", __func__);
		return -EINVAL;
	}

	regmap_update_bits(cs35l36->regmap, CS35L36_PLL_CLK_CTRL,
			CS35L36_PLL_CLK_SEL_MASK,
			CS35L36_PLLSRC_SWIRE << CS35L36_PLL_CLK_SEL_SHIFT);
	/* 12.288mhz */
	regmap_update_bits(cs35l36->regmap, CS35L36_PLL_CLK_CTRL,
			CS35L36_REFCLK_FREQ_MASK,
			CS35L36_REFCLK_FREQ_12_288M <<
			CS35L36_REFCLK_FREQ_SHIFT);

	regcache_sync(cs35l36->regmap);

	return 0;
}

static const struct swr_device_id cs35l36_swr_id[] = {
	{"cs35l36-swr", 0},
	{}
};

static struct swr_driver cs35l36_swr_codec_driver = {
	.driver = {
		.name = "cs35l36-swr",
		.owner = THIS_MODULE,
		.of_match_table = cs35l36_of_match,
	},
	.probe = cs35l36_swr_probe,
	.remove = cs35l36_swr_remove,
	.id_table = cs35l36_swr_id,
	.device_up = cs35l36_swr_up,
	.device_down = cs35l36_swr_down,
	.reset_device = cs35l36_swr_reset,
	.startup = cs35l36_swr_startup,
};

static int __init cs35l36_swr_codec_init(void)
{
	return swr_driver_register(&cs35l36_swr_codec_driver);
}

static void __exit cs35l36_swr_codec_exit(void)
{
	swr_driver_unregister(&cs35l36_swr_codec_driver);
}

module_init(cs35l36_swr_codec_init);
module_exit(cs35l36_swr_codec_exit);

MODULE_DESCRIPTION("ASoC CS35L36 SoundWire driver");
MODULE_AUTHOR("Vlad Karpovich, Cirrus Logic Inc, <Vlad.Karpovich@cirrus.com>");
MODULE_LICENSE("GPL");
