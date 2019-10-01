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

static bool cs35l45_is_csplmboxsts_correct(enum cspl_mboxcmd cmd,
					   enum cspl_mboxstate sts)
{
	switch (cmd) {
	case CSPL_MBOX_CMD_NONE:
	case CSPL_MBOX_CMD_UNKNOWN_CMD:
		return true;
	case CSPL_MBOX_CMD_PAUSE:
		return (sts == CSPL_MBOX_STS_PAUSED);
	case CSPL_MBOX_CMD_RESUME:
		return (sts == CSPL_MBOX_STS_RUNNING);
	case CSPL_MBOX_CMD_REINIT:
		return (sts == CSPL_MBOX_STS_RUNNING);
	case CSPL_MBOX_CMD_STOP_PRE_REINIT:
		return (sts == CSPL_MBOX_STS_RDY_FOR_REINIT);
	default:
		return false;
	}
}

int cs35l45_set_csplmboxcmd(struct cs35l45_private *cs35l45,
			    enum cspl_mboxcmd cmd)
{
	unsigned int sts, i;
	bool ack = false;

	/* Reset DSP sticky bit */
	regmap_write(cs35l45->regmap, CS35L45_IRQ2_EINT_2,
		     CS35L45_DSP_VIRT1_MBOX_MASK);

	/* Reset AP sticky bit */
	regmap_write(cs35l45->regmap, CS35L45_IRQ1_EINT_2,
		     CS35L45_DSP_VIRT2_MBOX_MASK);

	/* Unmask DSP INT */
	regmap_update_bits(cs35l45->regmap, CS35L45_IRQ2_MASK_2,
			   CS35L45_DSP_VIRT1_MBOX_MASK, 0);

	regmap_write(cs35l45->regmap, CS35L45_DSP_VIRT1_MBOX_1, cmd);

	/* Poll for DSP ACK */
	for (i = 0; i < 5; i++) {
		usleep_range(1000, 1100);

		regmap_read(cs35l45->regmap, CS35L45_IRQ1_EINT_2, &sts);
		if (!(sts & CS35L45_DSP_VIRT2_MBOX_MASK))
			continue;

		regmap_write(cs35l45->regmap, CS35L45_IRQ1_EINT_2,
			     CS35L45_DSP_VIRT2_MBOX_MASK);

		ack = true;
		break;
	}

	/* Mask DSP INT */
	regmap_update_bits(cs35l45->regmap, CS35L45_IRQ2_MASK_2,
			   CS35L45_DSP_VIRT1_MBOX_MASK,
			   CS35L45_DSP_VIRT1_MBOX_MASK);

	if (!ack) {
		dev_err(cs35l45->dev, "Timeout waiting for MBOX ACK\n");
		return -ETIMEDOUT;
	}

	regmap_read(cs35l45->regmap, CS35L45_DSP_MBOX_2, &sts);
	if (!cs35l45_is_csplmboxsts_correct(cmd, (enum cspl_mboxstate)sts)) {
		dev_err(cs35l45->dev, "Failed to set MBOX (cmd: %u, sts: %u)\n",
			cmd, sts);
		return -ENOMSG;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(cs35l45_set_csplmboxcmd);

static int cs35l45_dsp_loader_ev(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct cs35l45_private *cs35l45 =
		snd_soc_component_get_drvdata(component);
	enum cspl_mboxcmd mboxcmd = CSPL_MBOX_CMD_NONE;
	unsigned int sts, i;
	int ret;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (cs35l45->halo_booted == false) {
			regmap_update_bits(cs35l45->regmap,
				CS35L45_DSP1_CCM_CORE_CONTROL,
				CS35L45_CCM_PM_REMAP_MASK,
				CS35L45_CCM_PM_REMAP_MASK);

			regmap_update_bits(cs35l45->regmap, CS35L45_PWRMGT_CTL,
					   CS35L45_MEM_RDY_MASK, 0);

			wm_adsp_early_event(w, kcontrol, event);
		}
		break;
	case SND_SOC_DAPM_POST_PMU:
		if (cs35l45->halo_booted == false) {
			wm_adsp_event(w, kcontrol, event);

			regmap_update_bits(cs35l45->regmap, CS35L45_PWRMGT_CTL,
					   CS35L45_MEM_RDY_MASK,
					   CS35L45_MEM_RDY_MASK);

			regmap_write(cs35l45->regmap,
				     CS35L45_DSP1_CCM_CORE_CONTROL,
				     CS35L45_CCM_PM_REMAP_MASK |
				     CS35L45_CCM_CORE_RESET_MASK);

			regmap_write(cs35l45->regmap,
				     CS35L45_DSP1_CCM_CORE_CONTROL,
				     CS35L45_CCM_PM_REMAP_MASK |
				     CS35L45_CCM_CORE_EN_MASK);

			/* Poll for DSP ACK */
			for (i = 0; i < 5; i++) {
				usleep_range(1000, 1100);

				regmap_read(cs35l45->regmap,
					    CS35L45_IRQ1_EINT_2, &sts);
				if (!(sts & CS35L45_DSP_VIRT2_MBOX_MASK))
					continue;

				regmap_write(cs35l45->regmap,
					     CS35L45_IRQ1_EINT_2,
					     CS35L45_DSP_VIRT2_MBOX_MASK);

				break;
			}

			if (i == 5) {
				dev_err(cs35l45->dev,
					"Timeout waiting for MBOX ACK\n");
				return -ETIMEDOUT;
			}

			mboxcmd = CSPL_MBOX_CMD_PAUSE;
			ret = cs35l45_set_csplmboxcmd(cs35l45, mboxcmd);
			if (ret < 0)
				dev_err(cs35l45->dev, "MBOX failure (%d)\n",
					ret);

			cs35l45->halo_booted = true;
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if (cs35l45->halo_booted == false) {
			regmap_update_bits(cs35l45->regmap,
				CS35L45_DSP1_STREAM_ARB_TX1_CONFIG_0,
				CS35L45_DSP1_STREAM_ARB_TX1_EN_MASK, 0);

			regmap_update_bits(cs35l45->regmap,
				CS35L45_DSP1_STREAM_ARB_MSTR1_CONFIG_0,
				CS35L45_DSP1_STREAM_ARB_MSTR0_EN_MASK, 0);

			wm_adsp_early_event(w, kcontrol, event);
			wm_adsp_event(w, kcontrol, event);
		}
		break;
	default:
		dev_err(cs35l45->dev, "Invalid event = 0x%x\n", event);
		return -EINVAL;
	}

	return 0;
}

static int cs35l45_dsp_power_ev(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct cs35l45_private *cs35l45 =
		snd_soc_component_get_drvdata(component);
	enum cspl_mboxcmd mboxcmd = CSPL_MBOX_CMD_NONE;
	enum cspl_mboxstate fw_status = CSPL_MBOX_STS_RUNNING;
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		if (cs35l45->dsp.running) {
			regmap_read(cs35l45->regmap, CS35L45_DSP_MBOX_2,
				    (unsigned int *)&fw_status);

			switch (fw_status) {
			case CSPL_MBOX_STS_RDY_FOR_REINIT:
				mboxcmd = CSPL_MBOX_CMD_REINIT;
				break;
			case CSPL_MBOX_STS_RUNNING: /* First playback */
			case CSPL_MBOX_STS_PAUSED:
				mboxcmd = CSPL_MBOX_CMD_RESUME;
				break;
			default:
				dev_err(cs35l45->dev,
					"Invalid FW status (%u)\n", fw_status);
				return -EINVAL;
			}

			ret = cs35l45_set_csplmboxcmd(cs35l45, mboxcmd);
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if (cs35l45->dsp.running) {
			mboxcmd = CSPL_MBOX_CMD_PAUSE;
			ret = cs35l45_set_csplmboxcmd(cs35l45, mboxcmd);
			if (ret < 0)
				dev_err(cs35l45->dev, "MBOX failure (%d)\n",
					ret);
		}
		break;
	default:
		dev_err(cs35l45->dev, "Invalid event = 0x%x\n", event);
		ret = -EINVAL;
	}

	return ret;
}

static int cs35l45_amp_power_ev(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct cs35l45_private *cs35l45 =
			snd_soc_component_get_drvdata(component);
	unsigned int val;
	int i, ret = 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		regmap_update_bits(cs35l45->regmap, CS35L45_GLOBAL_ENABLES,
				   CS35L45_GLOBAL_EN_MASK,
				   CS35L45_GLOBAL_EN_MASK);

		for (i = 0; i < 10; i++) {
			regmap_read(cs35l45->regmap, CS35L45_IRQ1_EINT_1, &val);
			if (val & CS35L45_MSM_PUP_DONE_MASK)
				break;

			usleep_range(1000, 1100);
		}

		regmap_write(cs35l45->regmap, CS35L45_IRQ1_EINT_1,
			     CS35L45_MSM_PUP_DONE_MASK);

		if ((val & CS35L45_MSM_PUP_DONE_MASK) == 0) {
			dev_warn(cs35l45->dev, "PUP failed\n");
			return -ETIMEDOUT;
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_update_bits(cs35l45->regmap, CS35L45_GLOBAL_ENABLES,
				   CS35L45_GLOBAL_EN_MASK, 0);

		for (i = 0; i < 10; i++) {
			regmap_read(cs35l45->regmap, CS35L45_IRQ1_EINT_1, &val);
			if (val & CS35L45_MSM_PDN_DONE_MASK)
				break;

			usleep_range(1000, 1100);
		}

		regmap_write(cs35l45->regmap, CS35L45_IRQ1_EINT_1,
			     CS35L45_MSM_PDN_DONE_MASK);

		if ((val & CS35L45_MSM_PDN_DONE_MASK) == 0) {
			dev_warn(cs35l45->dev, "PDN failed\n");
			return -ETIMEDOUT;
		}
		break;
	default:
		dev_err(cs35l45->dev, "Invalid event = 0x%x\n", event);
		ret = -EINVAL;
	}

	return ret;
}

static const char * const pcm_texts[] = {"Zero", "ASP_RX1", "ASP_RX2", "VMON",
			"IMON", "VDD_BATTMON", "VDD_BSTMON", "DSP_TX1",
			"DSP_TX2", "SWIRE_RX1", "SWIRE_RX2"};

static const unsigned int pcm_values[] = {CS35L45_PCM_SRC_ZERO,
			CS35L45_PCM_SRC_ASP_RX1, CS35L45_PCM_SRC_ASP_RX2,
			CS35L45_PCM_SRC_VMON, CS35L45_PCM_SRC_IMON,
			CS35L45_PCM_SRC_VDD_BATTMON, CS35L45_PCM_SRC_VDD_BSTMON,
			CS35L45_PCM_SRC_DSP_TX1, CS35L45_PCM_SRC_DSP_TX2,
			CS35L45_PCM_SRC_SWIRE_RX1, CS35L45_PCM_SRC_SWIRE_RX2};

static const struct soc_enum mux_enums[] = {
	SOC_VALUE_ENUM_DOUBLE(CS35L45_ASPTX1_INPUT, 0, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(pcm_texts), pcm_texts, pcm_values),
	SOC_VALUE_ENUM_DOUBLE(CS35L45_ASPTX2_INPUT, 0, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(pcm_texts), pcm_texts, pcm_values),
	SOC_VALUE_ENUM_DOUBLE(CS35L45_ASPTX3_INPUT, 0, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(pcm_texts), pcm_texts, pcm_values),
	SOC_VALUE_ENUM_DOUBLE(CS35L45_ASPTX4_INPUT, 0, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(pcm_texts), pcm_texts, pcm_values),
	SOC_VALUE_ENUM_DOUBLE(CS35L45_DSP1RX1_INPUT, 0, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(pcm_texts), pcm_texts, pcm_values),
	SOC_VALUE_ENUM_DOUBLE(CS35L45_DSP1RX2_INPUT, 0, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(pcm_texts), pcm_texts, pcm_values),
	SOC_VALUE_ENUM_DOUBLE(CS35L45_DACPCM1_INPUT, 0, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(pcm_texts), pcm_texts, pcm_values),
};

static const struct snd_kcontrol_new muxes[] = {
	SOC_DAPM_ENUM("ASP_TX1 Source", mux_enums[ASP_TX1]),
	SOC_DAPM_ENUM("ASP_TX2 Source", mux_enums[ASP_TX2]),
	SOC_DAPM_ENUM("ASP_TX3 Source", mux_enums[ASP_TX3]),
	SOC_DAPM_ENUM("ASP_TX4 Source", mux_enums[ASP_TX4]),
	SOC_DAPM_ENUM("DSP_RX1 Source", mux_enums[DSP_RX1]),
	SOC_DAPM_ENUM("DSP_RX2 Source", mux_enums[DSP_RX2]),
	SOC_DAPM_ENUM("DACPCM Source", mux_enums[DACPCM]),
};

static const struct snd_soc_dapm_widget cs35l45_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("DSP1 Preload", NULL),

	SND_SOC_DAPM_SUPPLY_S("DSP1 Preloader", 100, SND_SOC_NOPM, 0, 0,
			      cs35l45_dsp_loader_ev, SND_SOC_DAPM_PRE_PMU |
			      SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_OUT_DRV_E("DSP", SND_SOC_NOPM, 0, 0, NULL, 0,
			       cs35l45_dsp_power_ev, SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_PGA_E("AMP", SND_SOC_NOPM, 0, 0, NULL, 0,
			   cs35l45_amp_power_ev, SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC("VMON ADC", NULL, CS35L45_BLOCK_ENABLES, 12, 0),
	SND_SOC_DAPM_ADC("IMON ADC", NULL, CS35L45_BLOCK_ENABLES, 13, 0),
	SND_SOC_DAPM_ADC("VDD_BATTMON ADC", NULL, CS35L45_BLOCK_ENABLES, 8, 0),
	SND_SOC_DAPM_ADC("VDD_BSTMON ADC", NULL, CS35L45_BLOCK_ENABLES, 9, 0),

	SND_SOC_DAPM_AIF_IN("ASP", NULL, 0, CS35L45_BLOCK_ENABLES2, 27, 0),
	SND_SOC_DAPM_AIF_IN("ASP_RX1", NULL, 0, CS35L45_ASP_ENABLES1, 16, 0),
	SND_SOC_DAPM_AIF_IN("ASP_RX2", NULL, 0, CS35L45_ASP_ENABLES1, 17, 0),

	SND_SOC_DAPM_AIF_OUT("ASP_TX1", NULL, 0, CS35L45_ASP_ENABLES1, 0, 0),
	SND_SOC_DAPM_AIF_OUT("ASP_TX2", NULL, 0, CS35L45_ASP_ENABLES1, 1, 0),
	SND_SOC_DAPM_AIF_OUT("ASP_TX3", NULL, 0, CS35L45_ASP_ENABLES1, 2, 0),
	SND_SOC_DAPM_AIF_OUT("ASP_TX4", NULL, 0, CS35L45_ASP_ENABLES1, 3, 0),
	SND_SOC_DAPM_AIF_OUT("ASP_TX5", NULL, 0, CS35L45_ASP_ENABLES1, 4, 0),

	SND_SOC_DAPM_MUX("ASP_TX1 Source", SND_SOC_NOPM, 0, 0, &muxes[ASP_TX1]),
	SND_SOC_DAPM_MUX("ASP_TX2 Source", SND_SOC_NOPM, 0, 0, &muxes[ASP_TX2]),
	SND_SOC_DAPM_MUX("ASP_TX3 Source", SND_SOC_NOPM, 0, 0, &muxes[ASP_TX3]),
	SND_SOC_DAPM_MUX("ASP_TX4 Source", SND_SOC_NOPM, 0, 0, &muxes[ASP_TX4]),
	SND_SOC_DAPM_MUX("DSP_RX1 Source", SND_SOC_NOPM, 0, 0, &muxes[DSP_RX1]),
	SND_SOC_DAPM_MUX("DSP_RX2 Source", SND_SOC_NOPM, 0, 0, &muxes[DSP_RX2]),
	SND_SOC_DAPM_MUX("DACPCM Source", SND_SOC_NOPM, 0, 0, &muxes[DACPCM]),

	SND_SOC_DAPM_OUTPUT("SPK"),
	SND_SOC_DAPM_OUTPUT("AP"),
};

static const struct snd_soc_dapm_route cs35l45_dapm_routes[] = {
	/* DSP */
	{"DSP1 Preload", NULL, "DSP1 Preloader"},

	{"DSP", NULL, "DSP1 Preloader"},
	{"DSP", NULL, "VMON ADC"},
	{"DSP", NULL, "IMON ADC"},
	{"DSP", NULL, "VDD_BATTMON ADC"},
	{"DSP", NULL, "VDD_BSTMON ADC"},

	/* Feedback */
	{"DSP", NULL, "Capture"},
	{"VMON ADC", NULL, "Capture"},
	{"IMON ADC", NULL, "Capture"},
	{"VDD_BATTMON ADC", NULL, "Capture"},
	{"VDD_BSTMON ADC", NULL, "Capture"},

	{"ASP_TX1 Source", "Zero", "Capture"},
	{"ASP_TX1 Source", "ASP_RX1", "Capture"},
	{"ASP_TX1 Source", "ASP_RX2", "Capture"},
	{"ASP_TX1 Source", "VMON", "VMON ADC"},
	{"ASP_TX1 Source", "IMON", "IMON ADC"},
	{"ASP_TX1 Source", "VDD_BATTMON", "VDD_BATTMON ADC"},
	{"ASP_TX1 Source", "VDD_BSTMON", "VDD_BSTMON ADC"},
	{"ASP_TX1 Source", "DSP_TX1", "DSP"},
	{"ASP_TX1 Source", "DSP_TX2", "DSP"},
	{"ASP_TX1 Source", "SWIRE_RX1", "Capture"},
	{"ASP_TX1 Source", "SWIRE_RX2", "Capture"},

	{"ASP_TX2 Source", "Zero", "Capture"},
	{"ASP_TX2 Source", "ASP_RX1", "Capture"},
	{"ASP_TX2 Source", "ASP_RX2", "Capture"},
	{"ASP_TX2 Source", "VMON", "VMON ADC"},
	{"ASP_TX2 Source", "IMON", "IMON ADC"},
	{"ASP_TX2 Source", "VDD_BATTMON", "VDD_BATTMON ADC"},
	{"ASP_TX2 Source", "VDD_BSTMON", "VDD_BSTMON ADC"},
	{"ASP_TX2 Source", "DSP_TX1", "DSP"},
	{"ASP_TX2 Source", "DSP_TX2", "DSP"},
	{"ASP_TX2 Source", "SWIRE_RX1", "Capture"},
	{"ASP_TX2 Source", "SWIRE_RX2", "Capture"},

	{"ASP_TX3 Source", "Zero", "Capture"},
	{"ASP_TX3 Source", "ASP_RX1", "Capture"},
	{"ASP_TX3 Source", "ASP_RX2", "Capture"},
	{"ASP_TX3 Source", "VMON", "VMON ADC"},
	{"ASP_TX3 Source", "IMON", "IMON ADC"},
	{"ASP_TX3 Source", "VDD_BATTMON", "VDD_BATTMON ADC"},
	{"ASP_TX3 Source", "VDD_BSTMON", "VDD_BSTMON ADC"},
	{"ASP_TX3 Source", "DSP_TX1", "DSP"},
	{"ASP_TX3 Source", "DSP_TX2", "DSP"},
	{"ASP_TX3 Source", "SWIRE_RX1", "Capture"},
	{"ASP_TX3 Source", "SWIRE_RX2", "Capture"},

	{"ASP_TX4 Source", "Zero", "Capture"},
	{"ASP_TX4 Source", "ASP_RX1", "Capture"},
	{"ASP_TX4 Source", "ASP_RX2", "Capture"},
	{"ASP_TX4 Source", "VMON", "VMON ADC"},
	{"ASP_TX4 Source", "IMON", "IMON ADC"},
	{"ASP_TX4 Source", "VDD_BATTMON", "VDD_BATTMON ADC"},
	{"ASP_TX4 Source", "VDD_BSTMON", "VDD_BSTMON ADC"},
	{"ASP_TX4 Source", "DSP_TX1", "DSP"},
	{"ASP_TX4 Source", "DSP_TX2", "DSP"},
	{"ASP_TX4 Source", "SWIRE_RX1", "Capture"},
	{"ASP_TX4 Source", "SWIRE_RX2", "Capture"},

	{"ASP_TX1", NULL, "ASP_TX1 Source"},
	{"ASP_TX2", NULL, "ASP_TX2 Source"},
	{"ASP_TX3", NULL, "ASP_TX3 Source"},
	{"ASP_TX4", NULL, "ASP_TX4 Source"},

	{"AP", NULL, "ASP_TX1"},
	{"AP", NULL, "ASP_TX2"},
	{"AP", NULL, "ASP_TX3"},
	{"AP", NULL, "ASP_TX4"},

	/* Playback */
	{"AMP", NULL, "Playback"},

	{"ASP", NULL, "AMP"},
	{"DSP", NULL, "AMP"},
	{"VMON ADC", NULL, "AMP"},
	{"IMON ADC", NULL, "AMP"},
	{"VDD_BATTMON ADC", NULL, "AMP"},
	{"VDD_BSTMON ADC", NULL, "AMP"},

	{"ASP_RX1", NULL, "ASP"},
	{"ASP_RX2", NULL, "ASP"},

	{"DSP_RX1 Source", NULL, "DSP"},
	{"DSP_RX1 Source", "Zero", "AMP"},
	{"DSP_RX1 Source", "ASP_RX1", "ASP_RX1"},
	{"DSP_RX1 Source", "ASP_RX2", "ASP_RX2"},
	{"DSP_RX1 Source", "VMON", "AMP"},
	{"DSP_RX1 Source", "IMON", "AMP"},
	{"DSP_RX1 Source", "VDD_BATTMON", "AMP"},
	{"DSP_RX1 Source", "VDD_BSTMON", "AMP"},
	{"DSP_RX1 Source", "DSP_TX1", "AMP"},
	{"DSP_RX1 Source", "DSP_TX2", "AMP"},
	{"DSP_RX1 Source", "SWIRE_RX1", "AMP"},
	{"DSP_RX1 Source", "SWIRE_RX2", "AMP"},

	{"DSP_RX2 Source", NULL, "DSP"},
	{"DSP_RX2 Source", "Zero", "AMP"},
	{"DSP_RX2 Source", "ASP_RX1", "ASP_RX1"},
	{"DSP_RX2 Source", "ASP_RX2", "ASP_RX2"},
	{"DSP_RX2 Source", "VMON", "AMP"},
	{"DSP_RX2 Source", "IMON", "AMP"},
	{"DSP_RX2 Source", "VDD_BATTMON", "AMP"},
	{"DSP_RX2 Source", "VDD_BSTMON", "AMP"},
	{"DSP_RX2 Source", "DSP_TX1", "AMP"},
	{"DSP_RX2 Source", "DSP_TX2", "AMP"},
	{"DSP_RX2 Source", "SWIRE_RX1", "AMP"},
	{"DSP_RX2 Source", "SWIRE_RX2", "AMP"},

	{"DACPCM Source", "Zero", "AMP"},
	{"DACPCM Source", "ASP_RX1", "ASP_RX1"},
	{"DACPCM Source", "ASP_RX2", "ASP_RX2"},
	{"DACPCM Source", "VMON", "VMON ADC"},
	{"DACPCM Source", "IMON", "IMON ADC"},
	{"DACPCM Source", "VDD_BATTMON", "VDD_BATTMON ADC"},
	{"DACPCM Source", "VDD_BSTMON", "VDD_BSTMON ADC"},
	{"DACPCM Source", "DSP_TX1", "DSP_RX1 Source"},
	{"DACPCM Source", "DSP_TX1", "DSP_RX2 Source"},
	{"DACPCM Source", "DSP_TX2", "DSP_RX1 Source"},
	{"DACPCM Source", "DSP_TX2", "DSP_RX2 Source"},
	{"DACPCM Source", "SWIRE_RX1", "AMP"},
	{"DACPCM Source", "SWIRE_RX2", "AMP"},

	{"SPK", NULL, "DACPCM Source"},
};

static const struct snd_kcontrol_new cs35l45_aud_controls[] = {
	WM_ADSP2_PRELOAD_SWITCH("DSP1", 1),
	WM_ADSP_FW_CONTROL("DSP1", 0),
};

static int cs35l45_dai_set_sysclk(struct snd_soc_dai *dai, int clk_id,
				  unsigned int freq, int dir)
{
	return 0;
}

static int cs35l45_dai_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct snd_soc_dapm_context *dapm =
			snd_soc_component_get_dapm(component);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		snd_soc_component_enable_pin(component, "SPK");
	else
		snd_soc_component_enable_pin(component, "AP");

	snd_soc_dapm_sync(dapm);

	return 0;
}

static void cs35l45_dai_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct snd_soc_dapm_context *dapm =
			snd_soc_component_get_dapm(component);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		snd_soc_component_disable_pin(component, "SPK");
	else
		snd_soc_component_disable_pin(component, "AP");

	snd_soc_dapm_sync(dapm);
}

static const struct snd_soc_dai_ops cs35l45_dai_ops = {
	.startup = cs35l45_dai_startup,
	.shutdown = cs35l45_dai_shutdown,
	.set_sysclk = cs35l45_dai_set_sysclk,
};

#define CS35L45_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
			 SNDRV_PCM_FMTBIT_S24_3LE| \
			 SNDRV_PCM_FMTBIT_S24_LE)

#define CS35L45_RATES (SNDRV_PCM_RATE_8000  | \
		       SNDRV_PCM_RATE_16000 | \
		       SNDRV_PCM_RATE_44100 | \
		       SNDRV_PCM_RATE_48000 | \
		       SNDRV_PCM_RATE_88200 | \
		       SNDRV_PCM_RATE_96000)

static struct snd_soc_dai_driver cs35l45_dai = {
	.name = "cs35l45",
	.playback = {
		      .stream_name = "Playback",
		      .channels_min = 1,
		      .channels_max = 8,
		      .rates = CS35L45_RATES,
		      .formats = CS35L45_FORMATS,
	},
	.capture = {
		      .stream_name = "Capture",
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
	struct snd_soc_dapm_context *dapm =
			snd_soc_component_get_dapm(component);

	snd_soc_component_disable_pin(component, "SPK");
	snd_soc_component_disable_pin(component, "AP");

	snd_soc_dapm_sync(dapm);

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

int cs35l45_initialize(struct cs35l45_private *cs35l45)
{
	struct device *dev = cs35l45->dev;
	int ret;
	u32 dev_id, rev_id;

	if (cs35l45->initialized)
		return -EPERM;

	regmap_write(cs35l45->regmap, CS35L45_SFT_RESET,
		     CS35L45_SOFT_RESET_TRIGGER);

	msleep(20);

	regmap_update_bits(cs35l45->regmap,
		CS35L45_DSP1_STREAM_ARB_TX1_CONFIG_0,
		CS35L45_DSP1_STREAM_ARB_TX1_EN_MASK, 0);

	regmap_update_bits(cs35l45->regmap,
		CS35L45_DSP1_STREAM_ARB_MSTR1_CONFIG_0,
		CS35L45_DSP1_STREAM_ARB_MSTR0_EN_MASK, 0);

	regmap_update_bits(cs35l45->regmap,
		CS35L45_DSP1_CCM_CORE_CONTROL,
		CS35L45_CCM_CORE_EN_MASK, 0);

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

static const struct wm_adsp_region cs35l45_dsp1_regions[] = {
	{ .type = WMFW_HALO_PM_PACKED,	.base = CS35L45_DSP1_PMEM_0 },
	{ .type = WMFW_HALO_XM_PACKED,	.base = CS35L45_DSP1_XMEM_PACK_0 },
	{ .type = WMFW_HALO_YM_PACKED,	.base = CS35L45_DSP1_YMEM_PACK_0 },
	{. type = WMFW_ADSP2_XM,	.base = CS35L45_DSP1_XMEM_UNPACK24_0},
	{. type = WMFW_ADSP2_YM,	.base = CS35L45_DSP1_YMEM_UNPACK24_0},
};

static int cs35l45_dsp_init(struct cs35l45_private *cs35l45)
{
	struct wm_adsp *dsp = &cs35l45->dsp;
	int ret, i;

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

	for (i = 0; i < CS35L45_DSP_N_RX_RATES; i++)
		dsp->rx_rate_cache[i] = 0x1;

	for (i = 0; i < CS35L45_DSP_N_TX_RATES; i++)
		dsp->tx_rate_cache[i] = 0x1;

	return ret;
}

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
	if (ret < 0) {
		dev_err(dev, "Failed to request core supplies: %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_enable(CS35L45_NUM_SUPPLIES, cs35l45->supplies);
	if (ret < 0) {
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
		gpiod_set_value_cansleep(cs35l45->reset_gpio, 0);
		usleep_range(2000, 2100);
		gpiod_set_value_cansleep(cs35l45->reset_gpio, 1);
	}

	ret = cs35l45_dsp_init(cs35l45);
	if (ret < 0) {
		dev_err(dev, "dsp_init failed: %d\n", ret);
		goto err;
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
