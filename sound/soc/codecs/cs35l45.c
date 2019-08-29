// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * cs35l45.c - CS35L45 ALSA SoC audio driver
 *
 * Copyright 2019 Cirrus Logic, Inc.
 *
 * Author: James Schulman <james.schulman@cirrus.com>
 *
 */

#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include "wm_adsp.h"
#include "cs35l45.h"
#include <sound/cs35l45.h>

static const struct snd_kcontrol_new cs35l45_aud_controls[] = {
	WM_ADSP2_PRELOAD_SWITCH("DSP1", 1),
	WM_ADSP_FW_CONTROL("DSP1", 0),
};

static const struct snd_soc_dapm_widget cs35l45_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("SPK"),
};

static const struct snd_soc_dapm_route cs35l45_dapm_routes[] = {
	{ "SPK", NULL, "Playback" },
};

static int cs35l45_dai_set_sysclk(struct snd_soc_dai *dai, int clk_id,
				  unsigned int freq, int dir)
{
	return 0;
}

static const struct snd_soc_dai_ops cs35l45_dai_ops = {
	.set_sysclk = cs35l45_dai_set_sysclk,
};

#define CS35L45_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
			 SNDRV_PCM_FMTBIT_S24_3LE| \
			 SNDRV_PCM_FMTBIT_S24_LE)

#define CS35L45_RATES	(SNDRV_PCM_RATE_8000  | \
			 SNDRV_PCM_RATE_16000 | \
			 SNDRV_PCM_RATE_44100 | \
			 SNDRV_PCM_RATE_48000 | \
			 SNDRV_PCM_RATE_88200 | \
			 SNDRV_PCM_RATE_96000)

static struct snd_soc_dai_driver cs35l45_dai = {
	.name = "cs35l45",
	.playback = { /* Support maximum range */
		      .stream_name = "Playback",
		      .channels_min = 1,
		      .channels_max = 8,
		      .rates = CS35L45_RATES,
		      .formats = CS35L45_FORMATS,
		    },
	.ops = &cs35l45_dai_ops,
};

static int cs35l45_component_set_sysclk(struct snd_soc_component *component,
					int clk_id, int source,
					unsigned int freq, int dir)
{
	return 0;
}

static int cs35l45_component_probe(struct snd_soc_component *component)
{
	struct cs35l45_private *cs35l45 =
			snd_soc_component_get_drvdata(component);

	component->regmap = cs35l45->regmap;

	return wm_adsp2_component_probe(&cs35l45->dsp, component);
}

static void cs35l45_component_remove(struct snd_soc_component *component)
{
	struct cs35l45_private *cs35l45 =
		snd_soc_component_get_drvdata(component);

	wm_adsp2_component_remove(&cs35l45->dsp, component);
}

static const struct snd_soc_component_driver cs35l45_component = {
	.probe = cs35l45_component_probe,
	.remove = cs35l45_component_remove,
	.set_sysclk = cs35l45_component_set_sysclk,

	.dapm_widgets = cs35l45_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cs35l45_dapm_widgets),

	.dapm_routes = cs35l45_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(cs35l45_dapm_routes),

	.controls = cs35l45_aud_controls,
	.num_controls = ARRAY_SIZE(cs35l45_aud_controls),
};

static const struct wm_adsp_region cs35l45_dsp1_regions[] = {
	{ .type = WMFW_HALO_PM_PACKED,	.base = CS35L45_DSP1_PMEM_0 },
	{ .type = WMFW_HALO_XM_PACKED,	.base = CS35L45_DSP1_XMEM_PACK_0 },
	{ .type = WMFW_HALO_YM_PACKED,	.base = CS35L45_DSP1_YMEM_PACK_0 },
	{. type = WMFW_ADSP2_XM,	.base = CS35L45_DSP1_XMEM_UNPACK24_0},
	{. type = WMFW_ADSP2_YM,	.base = CS35L45_DSP1_YMEM_UNPACK24_0},
};

static int cs35l45_dsp_init(struct cs35l45_private *cs35l45)
{
	struct wm_adsp *dsp;
	int ret, i;

	dsp = &cs35l45->dsp;
	dsp->part = "cs35l45";
	dsp->num = 1;
	dsp->type = WMFW_HALO;
	dsp->rev = 0;
	dsp->dev = cs35l45->dev;
	dsp->regmap = cs35l45->regmap;

	dsp->base = CS35L45_DSP1_CTRL_BASE;
	dsp->base_sysinfo = CS35L45_DSP1_SYS_ID;
	dsp->mem = cs35l45_dsp1_regions;
	dsp->num_mems = ARRAY_SIZE(cs35l45_dsp1_regions);
	dsp->lock_regions = 0xFFFFFFFF;

	dsp->n_rx_channels = CS35L45_DSP_N_RX_RATES;
	dsp->n_tx_channels = CS35L45_DSP_N_TX_RATES;

	mutex_init(&cs35l45->rate_lock);
	ret = wm_halo_init(dsp, &cs35l45->rate_lock);
	cs35l45->halo_booted = false;

	for (i = 0; i < CS35L45_DSP_N_RX_RATES; i++)
		dsp->rx_rate_cache[i] = 0x1;
	for (i = 0; i < CS35L45_DSP_N_TX_RATES; i++)
		dsp->tx_rate_cache[i] = 0x1;

	regmap_write(cs35l45->regmap, CS35L45_DSP1RX5_INPUT,
		     CS35L45_INPUT_SRC_VPMON);
	regmap_write(cs35l45->regmap, CS35L45_DSP1RX6_INPUT,
		     CS35L45_INPUT_SRC_CLASSH);
	regmap_write(cs35l45->regmap, CS35L45_DSP1RX7_INPUT,
		     CS35L45_INPUT_SRC_TEMPMON);
	regmap_write(cs35l45->regmap, CS35L45_DSP1RX8_INPUT,
		     CS35L45_INPUT_SRC_RSVD);

	return ret;
}

int cs35l45_initialize(struct cs35l45_private *cs35l45)
{
	struct device *dev = cs35l45->dev;
	int ret;
	u32 dev_id, rev_id;

	regmap_write(cs35l45->regmap, CS35L45_DSP1_CCM_CORE_CONTROL, 0);

	ret = cs35l45_dsp_init(cs35l45);
	if (ret < 0) {
		dev_err(dev, "dsp_init failed (%d)\n", ret);
		return ret;
	}

	ret = regmap_read(cs35l45->regmap, CS35L45_DEVID, &dev_id);
	if (ret < 0) {
		dev_err(dev, "Get Device ID failed\n");
		return ret;
	}

	ret = regmap_read(cs35l45->regmap, CS35L45_REVID, &rev_id);
	if (ret < 0) {
		dev_err(dev, "Get Revision ID failed\n");
		return ret;
	}

	dev_info(dev, "Cirrus Logic CS35L45 (%x), Revision: %02X\n", dev_id,
		 rev_id);

	cs35l45->initialized = true;

	return 0;
}
EXPORT_SYMBOL_GPL(cs35l45_initialize);

static const char * const cs35l45_supplies[] = {"VA", "VP"};

int cs35l45_probe(struct cs35l45_private *cs35l45)
{
	struct device *dev = cs35l45->dev;
	int ret;
	u32 i;

	for (i = 0; i < ARRAY_SIZE(cs35l45_supplies); i++)
		cs35l45->supplies[i].supply = cs35l45_supplies[i];

	ret = devm_regulator_bulk_get(dev, CS35L45_NUM_SUPPLIES,
				      cs35l45->supplies);
	if (ret) {
		dev_err(dev, "Failed to request core supplies: %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_enable(CS35L45_NUM_SUPPLIES, cs35l45->supplies);
	if (ret != 0) {
		dev_err(dev, "Failed to enable core supplies: %d\n", ret);
		return ret;
	}

	/* returning NULL can be an option if in stereo mode */
	cs35l45->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						      GPIOD_OUT_LOW);
	if (IS_ERR(cs35l45->reset_gpio)) {
		ret = PTR_ERR(cs35l45->reset_gpio);
		cs35l45->reset_gpio = NULL;
		if (ret == -EBUSY) {
			dev_info(dev,
				 "Reset line busy, assuming shared reset\n");
		} else {
			dev_err(dev, "Failed to get reset GPIO: %d\n", ret);
			goto err;
		}
	}

	if (cs35l45->reset_gpio) {
		/* satisfy minimum reset pulse width spec */
		usleep_range(2000, 2100);
		gpiod_set_value_cansleep(cs35l45->reset_gpio, 1);
	}

	return devm_snd_soc_register_component(dev, &cs35l45_component,
					       &cs35l45_dai, 1);

err:
	regulator_bulk_disable(CS35L45_NUM_SUPPLIES, cs35l45->supplies);
	return ret;
}
EXPORT_SYMBOL_GPL(cs35l45_probe);

int cs35l45_remove(struct cs35l45_private *cs35l45)
{
	if (cs35l45->reset_gpio)
		gpiod_set_value_cansleep(cs35l45->reset_gpio, 0);

	wm_adsp2_remove(&cs35l45->dsp);
	regulator_bulk_disable(CS35L45_NUM_SUPPLIES, cs35l45->supplies);

	return 0;
}
EXPORT_SYMBOL_GPL(cs35l45_remove);

MODULE_DESCRIPTION("ASoC CS35L45 driver");
MODULE_AUTHOR("James Schulman, Cirrus Logic Inc, <james.schulman@cirrus.com>");
MODULE_LICENSE("GPL");
