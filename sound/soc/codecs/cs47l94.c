/*
 * cs47l94.c  --  ALSA SoC Audio driver for CS47L94/CS47L95 codecs
 *
 * Copyright 2016-2017 Cirrus Logic
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

#define CS47L94_MONO_OUTPUTS 2
#define CS47L94_N_AUXPDM 3
#define CS47L94_N_FLL 3
#define CS47L94_NUM_DSP 2
#define CS47L94_DSP_N_RX_CHANNELS 8
#define CS47L94_DSP_N_TX_CHANNELS 8

#define CS47L94_OUTH_CP_POLL_US		1000
#define CS47L94_OUTH_CP_POLL_TIMEOUT_US	100000

static const DECLARE_TLV_DB_SCALE(cs47l94_outh_digital_tlv, -12750, 50, 0);

#define CS47L94_ANC_INPUT_ROUTES(widget, name) \
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

#define CS47L94_ANC_OUTPUT_ROUTES(widget, name) \
	{ widget, NULL, name " ANC Source" }, \
	{ name " ANC Source", "ANC Left Channel", "ANCL" }


#define TACNA_US_RATE_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,\
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_get_enum_double, .put = tacna_us_rate_put, \
	.private_value = (unsigned long)&xenum }

struct cs47l94 {
	struct tacna_priv core;
	struct tacna_fll fll[3];
	struct completion outh_enabled;
	struct completion outh_disabled;
	unsigned int outh_main_vol[2];
};

static const DECLARE_TLV_DB_SCALE(cs47l94_aux_tlv, -9600, 50, 0);

static const struct wm_adsp_region cs47l94_dsp1_regions[] = {
	{ .type = WMFW_HALO_PM_PACKED, .base = 0x3800000 },
	{ .type = WMFW_HALO_XM_PACKED, .base = 0x2000000 },
	{ .type = WMFW_ADSP2_XM, .base = 0x2800000 },
	{ .type = WMFW_HALO_YM_PACKED, .base = 0x2C00000 },
	{ .type = WMFW_ADSP2_YM, .base = 0x3400000 },
};

static const struct wm_adsp_region cs47l94_dsp2_regions[] = {
	{ .type = WMFW_HALO_PM_PACKED, .base = 0x5800000 },
	{ .type = WMFW_HALO_XM_PACKED, .base = 0x4000000 },
	{ .type = WMFW_ADSP2_XM, .base = 0x4800000 },
	{ .type = WMFW_HALO_YM_PACKED, .base = 0x4C00000 },
	{ .type = WMFW_ADSP2_YM, .base = 0x5400000 },
};

static const struct wm_adsp_region *cs47l94_dsp_regions[] = {
	cs47l94_dsp1_regions,
	cs47l94_dsp2_regions,
};

static const unsigned int cs47l94_dsp_control_bases[] = {
	TACNA_DSP1_CLOCK_FREQ,
	TACNA_DSP2_CLOCK_FREQ,
};

static const unsigned int cs47l94_dsp_sysinfo_bases[] = {
	TACNA_DSP1_SYS_INFO_ID,
	TACNA_DSP2_SYS_INFO_ID,
};

static const unsigned int cs47l94_dsp1_sram_power_regs[] = {
	TACNA_DSP1_XM_SRAM_IBUS_SETUP_0,
	TACNA_DSP1_YM_SRAM_IBUS_SETUP_0,
	TACNA_DSP1_PM_SRAM_IBUS_SETUP_0,
	0, /* end of list */
};

static const unsigned int cs47l94_dsp2_sram_power_regs[] = {
	TACNA_DSP2_XM_SRAM_IBUS_SETUP_0,
	TACNA_DSP2_YM_SRAM_IBUS_SETUP_0,
	TACNA_DSP2_PM_SRAM_IBUS_SETUP_0,
	0, /* end of list */
};

static int cs47l94_out_ev(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cs47l94 *cs47l94 = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = cs47l94->core.tacna;
	unsigned int vu_reg;
	unsigned int val;
	int ret;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		switch (w->shift) {
		case TACNA_OUT1L_EN_SHIFT:
			vu_reg = TACNA_OUT1L_VOLUME_1;
			break;
		case TACNA_OUT1R_EN_SHIFT:
			vu_reg = TACNA_OUT1R_VOLUME_1;
			break;
		case TACNA_OUT2L_EN_SHIFT:
			vu_reg = TACNA_OUT2L_VOLUME_1;
			break;
		case TACNA_OUT2R_EN_SHIFT:
			vu_reg = TACNA_OUT2R_VOLUME_1;
			break;
		default:
			dev_dbg(codec->dev, "Unrecognised output: %u\n",
				w->shift);
			return -EINVAL;
		}

		ret = regmap_read_poll_timeout(tacna->regmap,
				TACNA_OUTHL_CONTROL1,
				val,
				!(val & TACNA_CP_EN_OUTHL_EN_MASK),
				CS47L94_OUTH_CP_POLL_US,
				CS47L94_OUTH_CP_POLL_TIMEOUT_US);
		if (ret)
			goto err;

		ret = regmap_read_poll_timeout(tacna->regmap,
				TACNA_OUTHR_CONTROL1,
				val,
				!(val & TACNA_CP_EN_OUTHR_EN_MASK),
				CS47L94_OUTH_CP_POLL_US,
				CS47L94_OUTH_CP_POLL_TIMEOUT_US);
		if (ret)
			goto err;

		/* we must toggle VU to ensure mute and volume are updated */
		regmap_update_bits(tacna->regmap, vu_reg, TACNA_OUT_VU, 0);
		regmap_update_bits(tacna->regmap, vu_reg, TACNA_OUT_VU,
				   TACNA_OUT_VU);
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

static int cs47l94_out_aux_src_ev(struct snd_soc_dapm_widget *w,
				  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cs47l94 *cs47l94 = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = cs47l94->core.tacna;
	unsigned int reg;
	int ret;

	switch (w->reg) {
	case TACNA_OUT1L_CONTROL_1:
		reg = TACNA_OUT1L_VOLUME_1;
		break;
	case TACNA_OUT1R_CONTROL_1:
		reg = TACNA_OUT1R_VOLUME_1;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_update_bits(tacna->regmap, reg, TACNA_OUT_VU_MASK, 0);
	if (ret)
		dev_err(codec->dev, "Failed to toggle OUT_VU bit: %d\n", ret);
	ret = regmap_update_bits(tacna->regmap, reg, TACNA_OUT_VU_MASK,
				 TACNA_OUT_VU);
	if (ret)
		dev_err(codec->dev, "Failed to toggle OUT_VU bit: %d\n", ret);

	return ret;
}

static int cs47l94_outh_aux_src_ev(struct snd_soc_dapm_widget *w,
				   struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cs47l94 *cs47l94 = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = cs47l94->core.tacna;
	unsigned int reg;
	int ret;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		switch (w->shift) {
		case TACNA_OUTHL_AUX_SRC_SHIFT:
			reg = TACNA_OUT1L_VOLUME_1;
			break;
		case TACNA_OUTHR_AUX_SRC_SHIFT:
			reg = TACNA_OUT1R_VOLUME_1;
			break;
		default:
			return -EINVAL;
		}

		/* following writes must be done to enable AUX+DSD */
		ret = regmap_update_bits(tacna->regmap, TACNA_OUTH_CFG9,
					 TACNA_CLK_INTP_PREFILT_BYPASS_CG_MASK |
					 TACNA_CLK_INTP_BYPASS_CG_MASK,
					 TACNA_CLK_INTP_PREFILT_BYPASS_CG |
					 TACNA_CLK_INTP_BYPASS_CG);
		if (ret)
			dev_warn(codec->dev,
				 "Failed to write to OUTH_CFG9: %d\n", ret);
		ret = regmap_update_bits(tacna->regmap, TACNA_OUTH_DSD_PCM,
					 TACNA_OUTH_DSD_PCM_MIX_SETUP_MASK,
					 TACNA_OUTH_DSD_PCM_MIX_SETUP);
		if (ret)
			dev_warn(codec->dev,
				 "Failed to write to OUTH_DSD_PCM: %d\n", ret);
		ret = regmap_update_bits(tacna->regmap, TACNA_OUTH_DSD_PCM,
					 TACNA_OUTH_DSD_PCM_MIX_MASK,
					 TACNA_OUTH_DSD_PCM_MIX);
		if (ret)
			dev_warn(codec->dev,
				"Failed to write to OUTH_DSD_PCM: %d\n", ret);

		/* enable AUX */
		ret = regmap_update_bits(tacna->regmap, reg,
					 TACNA_OUT_VU_MASK, 0);
		if (ret)
			dev_err(codec->dev, "Failed to toggle OUT_VU bit: %d\n",
				ret);
		ret = regmap_update_bits(tacna->regmap, reg,
					 TACNA_OUT_VU_MASK, TACNA_OUT_VU);
		if (ret)
			dev_err(codec->dev, "Failed to toggle OUT_VU bit: %d\n",
				ret);
		return 0;
	case SND_SOC_DAPM_PRE_PMD:
		ret = regmap_update_bits(tacna->regmap, TACNA_OUTH_DSD_PCM,
					 TACNA_OUTH_DSD_PCM_MIX_MASK, 0);
		if (ret)
			dev_warn(codec->dev,
				"Failed to write to OUTH_DSD_PCM: %d\n", ret);
		ret = regmap_update_bits(tacna->regmap, TACNA_OUTH_DSD_PCM,
					 TACNA_OUTH_DSD_PCM_MIX_SETUP_MASK, 0);
		if (ret)
			dev_warn(codec->dev,
				 "Failed to write to OUTH_DSD_PCM: %d\n", ret);
		ret = regmap_update_bits(tacna->regmap, TACNA_OUTH_CFG9,
					 TACNA_CLK_INTP_PREFILT_BYPASS_CG_MASK |
					 TACNA_CLK_INTP_BYPASS_CG_MASK,
					 0);
		if (ret)
			dev_warn(codec->dev,
				 "Failed to write to OUTH_CFG9: %d\n", ret);
		return 0;
	default:
		return 0;
	}
}

static int cs47l94_outaux_ev(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cs47l94 *cs47l94 = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = cs47l94->core.tacna;
	unsigned int val;
	int ret;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		val = 0;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		val = TACNA_OUTAUX1L_MUTE_OVD;
		break;
	default:
		return 0;
	}

	ret = regmap_update_bits(tacna->regmap, w->reg,
				 TACNA_OUTAUX1L_MUTE_OVD_MASK, val);
	if (ret)
		dev_err(codec->dev,
			"Failed to toggle OUTAUX MUTE_OVD bit: %d\n", ret);

	return 0;
}

static int cs47l94_wait_for_cp_disable(struct tacna *tacna)
{
	struct regmap *regmap = tacna->regmap;
	unsigned int val;
	int ret;

	ret = regmap_read_poll_timeout(regmap, TACNA_HP1L_CTRL, val,
				       !(val & TACNA_CP_EN_HP1L_MASK),
				       CS47L94_OUTH_CP_POLL_US,
				       CS47L94_OUTH_CP_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(regmap, TACNA_HP1R_CTRL, val,
				       !(val & TACNA_CP_EN_HP1R_MASK),
				       CS47L94_OUTH_CP_POLL_US,
				       CS47L94_OUTH_CP_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(regmap, TACNA_HP2L_CTRL, val,
				       !(val & TACNA_CP_EN_HP2L_MASK),
				       CS47L94_OUTH_CP_POLL_US,
				       CS47L94_OUTH_CP_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(regmap, TACNA_HP2R_CTRL, val,
				       !(val & TACNA_CP_EN_HP2R_MASK),
				       CS47L94_OUTH_CP_POLL_US,
				       CS47L94_OUTH_CP_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	return 0;
}

static int cs47l94_outh_ev(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cs47l94 *cs47l94 = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = cs47l94->core.tacna;
	struct regmap *regmap = tacna->regmap;
	unsigned int val;
	int ret, accdet;
	long time_left;
	bool outh_upd = false;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = cs47l94_wait_for_cp_disable(tacna);
		if (ret) {
			dev_err(codec->dev,
				"OUTH enable failed (OUT1/2 CP enabled): %d\n",
				ret);
			return ret;
		}

		/*
		 * OUTH requires DAC clock to be set to it's highest
		 * setting, so check if that's the case
		 */
		ret = regmap_read(regmap, TACNA_DAC_CLK_CONTROL1, &val);
		val = (val & TACNA_DAC_CLK_SRC_FREQ_MASK) >>
			TACNA_DAC_CLK_SRC_FREQ_SHIFT;

		if (val != 2) {
			dev_err(codec->dev,
				"OUTH enable failed (incompatible DACCLK).\n");
			return -EINVAL;
		}

		ret = regmap_write(regmap, TACNA_OUTHL_VOLUME_1,
				   TACNA_OUTH_VU | cs47l94->outh_main_vol[0]);
		if (ret)
			dev_warn(codec->dev,
				 "Failed to apply cached OUTHL volume: %d\n",
				 ret);

		ret = regmap_write(regmap, TACNA_OUTHR_VOLUME_1,
				   TACNA_OUTH_VU | cs47l94->outh_main_vol[1]);
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
		ret = regmap_write(regmap, TACNA_OUTHL_VOLUME_1, 0xc0);
		if (ret)
			dev_warn(codec->dev,
				 "Failed to set OUTHL volume to default: %d\n",
				 ret);

		ret = regmap_write(regmap, TACNA_OUTHR_VOLUME_1, 0xc0);
		if (ret)
			dev_warn(codec->dev,
				 "Failed to set OUTHR volume to default: %d\n",
				 ret);
		break;
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
		reinit_completion(&cs47l94->outh_enabled);
		reinit_completion(&cs47l94->outh_disabled);
	}

	ret = regmap_update_bits_check(regmap, TACNA_OUTH_ENABLE_1,
				       TACNA_OUTH_EN_MASK, val, &outh_upd);
	if (ret)
		dev_err(codec->dev, "Failed to toggle OUTH enable: %d\n", ret);

	if (outh_upd) { /* wait for enable/disable to take effect */
		if (val)
			time_left = wait_for_completion_timeout(
							&cs47l94->outh_enabled,
							msecs_to_jiffies(100));
		else
			time_left = wait_for_completion_timeout(
							&cs47l94->outh_disabled,
							msecs_to_jiffies(100));

		if (!time_left)
			dev_warn(codec->dev, "OUTH %s timed out.\n",
				 (val) ? "enable" : "disable");
	}

	return ret;
}

static int cs47l94_dsd_processor_ev(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cs47l94 *cs47l94 = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = cs47l94->core.tacna;
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
		return 0;
	case SND_SOC_DAPM_PRE_PMD:
		ret = regmap_update_bits(tacna->regmap, TACNA_OUTH_ENABLE_1,
					 TACNA_DSD1_IF_EN_MASK, 0);
		if (ret)
			dev_warn(codec->dev,
				"Failed to write to OUTH_ENABLE_1: %d\n", ret);
		return 0;
	default:
		return 0;
	}
}

static irqreturn_t cs47l94_outh_enable(int irq, void *data)
{
	struct cs47l94 *cs47l94 = data;

	dev_dbg(cs47l94->core.dev, "OUTH enable interrupt\n");

	complete(&cs47l94->outh_enabled);

	return IRQ_HANDLED;
}

static irqreturn_t cs47l94_outh_disable(int irq, void *data)
{
	struct cs47l94 *cs47l94 = data;

	dev_dbg(cs47l94->core.dev, "OUTH disable interrupt\n");

	complete(&cs47l94->outh_disabled);

	return IRQ_HANDLED;
}

static irqreturn_t cs47l94_mpu_fault_irq(int irq, void *data)
{
	struct wm_adsp *dsp = data;

	return wm_halo_bus_error(dsp);
}

static const char * const cs47l94_out1_demux_texts[] = {
	"HP1", "HP2",
};

static int cs47l94_put_out1_demux(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
					snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_codec *codec = snd_soc_dapm_kcontrol_codec(kcontrol);
	struct cs47l94 *cs47l94 = dev_get_drvdata(codec->dev);
	struct tacna *tacna = cs47l94->core.tacna;
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int hp2_sel, mux, mask, cur;
	bool change, out_mono;
	int ret;

	if (ucontrol->value.enumerated.item[0] > e->items - 1)
		return -EINVAL;

	mux = ucontrol->value.enumerated.item[0];
	hp2_sel = mux << e->shift_l;
	mask = e->mask << e->shift_l;

	snd_soc_dapm_mutex_lock(dapm);

	if (!snd_soc_test_bits(codec, e->reg, mask, hp2_sel)) {
		snd_soc_dapm_mutex_unlock(dapm);
		return 0;
	}

	if (tacna_get_accdet_for_output(codec, 1) < 0) {
		ret = regmap_read(tacna->regmap, TACNA_OUTPUT_ENABLE_1, &cur);
		if (ret)
			dev_warn(codec->dev,
				 "Failed to read OUTPUT_ENABLE_1: %d\n",
				 ret);
	} else {
		/* OUT1 is associated with accessory detect activities */
		/*cur = tacna->hp_ena;*/
		dev_warn(codec->dev, "OUT1 demux handling of accdet incomplete\n");
	}

	/* Don't change demux and mono settings while OUT1 is enabled */
	ret = regmap_update_bits_check(tacna->regmap, TACNA_OUTPUT_ENABLE_1,
				       TACNA_OUT1L_EN_MASK | TACNA_OUT1R_EN_MASK,
				       0, &change);
	if (ret)
		dev_warn(codec->dev, "Failed to disable outputs: %d\n", ret);
	else if (change)
		tacna_wait_for_output_seq(&cs47l94->core,
					  TACNA_OUT1L_STS | TACNA_OUT1R_STS,
					  0);

	ret = regmap_update_bits(tacna->regmap, TACNA_HP_CTRL,
				 TACNA_OUT1_MODE_MASK,
				 hp2_sel << TACNA_OUT1_MODE_SHIFT);
	if (ret) {
		dev_warn(codec->dev, "Failed to set OUT1_MODE: %d\n", ret);
	} else {
		BUILD_BUG_ON(ARRAY_SIZE(tacna->pdata.codec.out_mono) < 3);
		out_mono = tacna->pdata.codec.out_mono[mux * 2];
		ret = tacna_set_output_mode(codec, 1, out_mono);
		if (ret < 0)
			dev_warn(codec->dev,
				 "Failed to set output mode: %d\n", ret);
	}

	ret = regmap_update_bits_check(tacna->regmap, TACNA_OUTPUT_ENABLE_1,
				       TACNA_OUT1L_EN_MASK | TACNA_OUT1R_EN_MASK,
				       cur, &change);
	if (ret)
		dev_warn(codec->dev, "Failed to restore outputs: %d\n", ret);
	else if (change)
		tacna_wait_for_output_seq(&cs47l94->core,
					  TACNA_OUT1L_STS | TACNA_OUT1R_STS,
					  cur);
	snd_soc_dapm_mutex_unlock(dapm);

	return snd_soc_dapm_mux_update_power(dapm, kcontrol, mux, e, NULL);
}

static SOC_ENUM_SINGLE_DECL(cs47l94_out1_demux_enum,
			    TACNA_HP_CTRL, TACNA_OUT1_MODE_SHIFT,
			    cs47l94_out1_demux_texts);

static const struct snd_kcontrol_new cs47l94_out1_demux =
	SOC_DAPM_ENUM_EXT("OUT1 Demux", cs47l94_out1_demux_enum,
			snd_soc_dapm_get_enum_double, cs47l94_put_out1_demux);

static int cs47l94_put_outaux_vu(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int ret;
	unsigned int reg, rreg = 0;

	switch (mc->reg) {
	case TACNA_OUTAUX1L_VOLUME_1:
		reg = TACNA_OUT1L_VOLUME_1;
		break;
	case TACNA_OUTAUX1R_VOLUME_1:
		reg = TACNA_OUT1R_VOLUME_1;
		break;
	default:
		return -EINVAL;
	}

	switch (mc->rreg) {
	case 0: /* => rreg not set, so just proceed */
		break;
	case TACNA_OUTAUX1L_VOLUME_1:
		rreg = TACNA_OUT1L_VOLUME_1;
		break;
	case TACNA_OUTAUX1R_VOLUME_1:
		rreg = TACNA_OUT1R_VOLUME_1;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_dapm_mutex_lock(dapm);

	snd_soc_component_update_bits(component, reg, TACNA_OUT_VU, 0);
	if (rreg)
		snd_soc_component_update_bits(component, rreg, TACNA_OUT_VU, 0);

	ret = snd_soc_put_volsw(kcontrol, ucontrol);

	snd_soc_component_update_bits(component, reg, TACNA_OUT_VU,
				      TACNA_OUT_VU);
	if (rreg)
		snd_soc_component_update_bits(component, rreg, TACNA_OUT_VU,
					      TACNA_OUT_VU);

	snd_soc_dapm_mutex_unlock(dapm);

	return ret;
}

static const struct soc_enum cs47l94_outh_rate =
	SOC_VALUE_ENUM_SINGLE(TACNA_OUTH_CONFIG_1,
			      TACNA_OUTH_RATE_SHIFT,
			      TACNA_OUTH_RATE_MASK >> TACNA_OUTH_RATE_SHIFT,
			      TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val);

static int cs47l94_get_outh_main_volume(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct cs47l94 *cs47l94 = dev_get_drvdata(codec->dev);
	int min = mc->min, shift = mc->shift, rshift = mc->rshift;

	ucontrol->value.integer.value[0] =
		(cs47l94->outh_main_vol[0] - min) >> shift;

	ucontrol->value.integer.value[1] =
		(cs47l94->outh_main_vol[1] - min) >> rshift;

	return 0;
}

static int cs47l94_put_outh_main_volume(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct cs47l94 *cs47l94 = dev_get_drvdata(codec->dev);
	int min = mc->min, shift = mc->shift, rshift = mc->rshift;

	snd_soc_dapm_mutex_lock(dapm);

	cs47l94->outh_main_vol[0] =
		(ucontrol->value.integer.value[0] + min) << shift;

	cs47l94->outh_main_vol[1] =
		(ucontrol->value.integer.value[1] + min) << rshift;

	snd_soc_dapm_mutex_unlock(dapm);

	return tacna_put_out_vu(kcontrol, ucontrol);
}

static const struct snd_kcontrol_new cs47l94_us1_switch =
		SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0);
static const struct snd_kcontrol_new cs47l94_us2_switch =
		SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0);

static const char * const tacna_us_in_texts[] = {
	"IN1L",
	"IN1R",
	"IN2L",
	"IN2R",
	"IN3L",
	"IN3R",
	"IN4L",
	"IN4R",
};

static irqreturn_t cs47l94_us1_activity(int irq, void *data)
{
	struct tacna *tacna = data;
	struct tacna_us_notify_data us_data;

	us_data.us_no = 1;
	tacna_call_notifiers(tacna, TACNA_NOTIFY_ULTRASONIC, &us_data);

	return IRQ_HANDLED;
}

static irqreturn_t cs47l94_us2_activity(int irq, void *data)
{
	struct tacna *tacna = data;
	struct tacna_us_notify_data us_data;

	us_data.us_no = 2;
	tacna_call_notifiers(tacna, TACNA_NOTIFY_ULTRASONIC, &us_data);

	return IRQ_HANDLED;
}

static SOC_ENUM_SINGLE_DECL(tacna_us1_in_enum,
			    TACNA_US1_CONTROL,
			    TACNA_US1_SRC_SHIFT,
			    tacna_us_in_texts);

static SOC_ENUM_SINGLE_DECL(tacna_us2_in_enum,
			    TACNA_US2_CONTROL,
			    TACNA_US2_SRC_SHIFT,
			    tacna_us_in_texts);

static const struct snd_kcontrol_new tacna_us_inmux[2] = {
	SOC_DAPM_ENUM("Ultrasonic 1 Input", tacna_us1_in_enum),
	SOC_DAPM_ENUM("Ultrasonic 2 Input", tacna_us2_in_enum),
};

static const char * const tacna_us_freq_texts[] = {
	"24.5-40.5kHz",
	"18-22kHz",
	"16-24kHz",
	"20-28kHz",
};

static const char * const tacna_us_gain_texts[] = {
	"No Signal",
	"-5dB",
	"+1dB",
	"+7dB",
};

static const char * const tacna_us_det_thr_texts[] = {
	"-6dB",
	"-9dB",
	"-12dB",
	"-15dB",
	"-18dB",
	"-21dB",
	"-24dB",
	"-27dB",
};

static SOC_ENUM_SINGLE_DECL(tacna_us1_det_thr_enum, TACNA_US1_DET_CONTROL,
			    TACNA_US1_DET_THR_SHIFT, tacna_us_det_thr_texts);
static SOC_ENUM_SINGLE_DECL(tacna_us2_det_thr_enum, TACNA_US2_DET_CONTROL,
			    TACNA_US2_DET_THR_SHIFT, tacna_us_det_thr_texts);

static const char * const tacna_us_det_num_texts[] = {
	"1 Sample",
	"2 Samples",
	"4 Samples",
	"8 Samples",
	"16 Samples",
	"32 Samples",
	"64 Samples",
	"128 Samples",
	"256 Samples",
	"512 Samples",
	"1024 Samples",
	"2048 Samples",
	"4096 Samples",
	"8192 Samples",
	"16384 Samples",
	"32768 Samples",
};

static SOC_ENUM_SINGLE_DECL(tacna_us1_det_num_enum, TACNA_US1_DET_CONTROL,
			    TACNA_US1_DET_NUM_SHIFT, tacna_us_det_num_texts);
static SOC_ENUM_SINGLE_DECL(tacna_us2_det_num_enum, TACNA_US2_DET_CONTROL,
			    TACNA_US2_DET_NUM_SHIFT, tacna_us_det_num_texts);

static const char * const tacna_us_det_hold_texts[] = {
	"0 Samples",
	"31 Samples",
	"63 Samples",
	"127 Samples",
	"255 Samples",
	"511 Samples",
	"1023 Samples",
	"2047 Samples",
	"4095 Samples",
	"8191 Samples",
	"16383 Samples",
	"32767 Samples",
	"65535 Samples",
	"131071 Samples",
	"262143 Samples",
	"524287 Samples",
};
static SOC_ENUM_SINGLE_DECL(tacna_us1_det_hold_enum,
			    TACNA_US1_DET_CONTROL,
			    TACNA_US1_DET_HOLD_SHIFT,
			    tacna_us_det_hold_texts);
static SOC_ENUM_SINGLE_DECL(tacna_us2_det_hold_enum,
			    TACNA_US2_DET_CONTROL,
			    TACNA_US2_DET_HOLD_SHIFT,
			    tacna_us_det_hold_texts);

static const char * const tacna_us_det_dcy_texts[] = {
	"0 Samples",
	"36 Samples",
	"73 Samples",
	"146 Samples",
	"293 Samples",
	"588 Samples",
	"1177 Samples",
	"2355 Samples",
};
static SOC_ENUM_SINGLE_DECL(tacna_us1_det_dcy_enum,
			    TACNA_US1_DET_CONTROL,
			    TACNA_US1_DET_DCY_SHIFT,
			    tacna_us_det_dcy_texts);
static SOC_ENUM_SINGLE_DECL(tacna_us2_det_dcy_enum,
			    TACNA_US2_DET_CONTROL,
			    TACNA_US2_DET_DCY_SHIFT,
			    tacna_us_det_dcy_texts);

static int tacna_us_rate_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	int ret;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_card *card = codec->component.card;
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = priv->tacna;

	unsigned int val, cur, mask, ena_mask;

	if (ucontrol->value.enumerated.item[0] > e->items - 1)
		return -EINVAL;

	val = e->values[ucontrol->value.enumerated.item[0]] << e->shift_l;
	mask = e->mask << e->shift_l;

	snd_soc_dapm_mutex_lock(&card->dapm);

	ret = regmap_read(tacna->regmap, e->reg, &cur);
	if (ret != 0) {
		dev_err(tacna->dev, "Failed to read current reg: %d\n", ret);
		goto end;
	}

	if ((cur & mask) == (val & mask))
		goto end;

	ret = regmap_read(tacna->regmap, e->reg, &cur);
	if (ret != 0) {
		dev_err(tacna->dev, "Failed to read enable reg: %d\n", ret);
		goto end;
	}

	switch (e->reg) {
	case TACNA_US1_CONTROL:
		ena_mask = TACNA_US1_EN_MASK;
		break;
	case TACNA_US2_CONTROL:
		ena_mask = TACNA_US2_EN_MASK;
		break;
	default:
		ret = -EINVAL;
		goto end;
	}

	if (cur & ena_mask) {
		dev_err(tacna->dev,
			"Can't change rate on active input 0x%08x: %d\n",
			e->reg, ret);
		ret = -EBUSY;
		goto end;
	}

	ret = snd_soc_update_bits(codec, e->reg, mask, val);

end:
	snd_soc_dapm_mutex_unlock(&card->dapm);

	return ret;
}

/*
 * TODO: auto depends on some other weird setting so will need to figure out
 *       how that works
 */
static const char* const cs47l94_dop_width_texts[] = {
	"auto", "32bit", "16bit",
};

static SOC_ENUM_SINGLE_DECL(cs47l94_dop_width_enum, TACNA_DOP1_CONTROL1,
			    TACNA_DOP1_WIDTH_SHIFT, cs47l94_dop_width_texts);

static const struct soc_enum tacna_us_output_rate[] = {
	SOC_VALUE_ENUM_SINGLE(TACNA_US1_CONTROL,
			      TACNA_US1_RATE_SHIFT,
			      TACNA_US1_RATE_MASK >> TACNA_US1_RATE_SHIFT,
			      TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_US2_CONTROL,
			      TACNA_US2_RATE_SHIFT,
			      TACNA_US2_RATE_MASK >> TACNA_US2_RATE_SHIFT,
			      TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
};

static SOC_ENUM_SINGLE_DECL(tacna_us1_freq_enum,
			    TACNA_US1_CONTROL,
			    TACNA_US1_FREQ_SHIFT,
			    tacna_us_freq_texts);
static SOC_ENUM_SINGLE_DECL(tacna_us2_freq_enum,
			    TACNA_US2_CONTROL,
			    TACNA_US2_FREQ_SHIFT,
			    tacna_us_freq_texts);

static SOC_ENUM_SINGLE_DECL(tacna_us1_gain_enum, TACNA_US1_CONTROL,
			    TACNA_US1_GAIN_SHIFT, tacna_us_gain_texts);
static SOC_ENUM_SINGLE_DECL(tacna_us2_gain_enum, TACNA_US2_CONTROL,
			    TACNA_US2_GAIN_SHIFT, tacna_us_gain_texts);

static const struct snd_kcontrol_new cs47l94_snd_controls[] = {
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

SOC_SINGLE_TLV("IN1L Digital Volume", TACNA_IN1L_CONTROL2,
	       TACNA_IN1L_VOL_SHIFT, 0xbf, 0, tacna_digital_tlv),
SOC_SINGLE_TLV("IN1R Digital Volume", TACNA_IN1R_CONTROL2,
	       TACNA_IN1R_VOL_SHIFT, 0xbf, 0, tacna_digital_tlv),
SOC_SINGLE_TLV("IN2L Digital Volume", TACNA_IN2L_CONTROL2,
	       TACNA_IN2L_VOL_SHIFT, 0xbf, 0, tacna_digital_tlv),
SOC_SINGLE_TLV("IN2R Digital Volume", TACNA_IN2R_CONTROL2,
	       TACNA_IN2R_VOL_SHIFT, 0xbf, 0, tacna_digital_tlv),
SOC_SINGLE_TLV("IN3L Digital Volume", TACNA_IN3L_CONTROL2,
	       TACNA_IN3L_VOL_SHIFT, 0xbf, 0, tacna_digital_tlv),
SOC_SINGLE_TLV("IN3R Digital Volume", TACNA_IN3R_CONTROL2,
	       TACNA_IN3R_VOL_SHIFT, 0xbf, 0, tacna_digital_tlv),
SOC_SINGLE_TLV("IN4L Digital Volume", TACNA_IN4L_CONTROL2,
	       TACNA_IN4L_VOL_SHIFT, 0xbf, 0, tacna_digital_tlv),
SOC_SINGLE_TLV("IN4R Digital Volume", TACNA_IN4R_CONTROL2,
	       TACNA_IN4R_VOL_SHIFT, 0xbf, 0, tacna_digital_tlv),

SOC_ENUM("Input Ramp Up", tacna_in_vi_ramp),
SOC_ENUM("Input Ramp Down", tacna_in_vd_ramp),

TACNA_FRF_BYTES("FRF COEFF 1L", TACNA_FRF_COEFF_1L_1, TACNA_FRF_COEFF_LEN),
TACNA_FRF_BYTES("FRF COEFF 1R", TACNA_FRF_COEFF_1R_1, TACNA_FRF_COEFF_LEN),
TACNA_FRF_BYTES("FRF COEFF 2L", TACNA_FRF_COEFF_2L_1, TACNA_FRF_COEFF_LEN),
TACNA_FRF_BYTES("FRF COEFF 2R", TACNA_FRF_COEFF_2R_1, TACNA_FRF_COEFF_LEN),
TACNA_FRF_BYTES("FRF COEFF 5L", TACNA_FRF_COEFF_5L_1, TACNA_FRF_COEFF_LEN),
TACNA_FRF_BYTES("FRF COEFF 5R", TACNA_FRF_COEFF_5R_1, TACNA_FRF_COEFF_LEN),

TACNA_MIXER_CONTROLS("EQ1", TACNA_EQ1MIX_INPUT1),
TACNA_MIXER_CONTROLS("EQ2", TACNA_EQ2MIX_INPUT1),
TACNA_MIXER_CONTROLS("EQ3", TACNA_EQ3MIX_INPUT1),
TACNA_MIXER_CONTROLS("EQ4", TACNA_EQ4MIX_INPUT1),

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

TACNA_MIXER_CONTROLS("DRC1L", TACNA_DRC1LMIX_INPUT1),
TACNA_MIXER_CONTROLS("DRC1R", TACNA_DRC1RMIX_INPUT1),
TACNA_MIXER_CONTROLS("DRC2L", TACNA_DRC2LMIX_INPUT1),
TACNA_MIXER_CONTROLS("DRC2R", TACNA_DRC2RMIX_INPUT1),

SND_SOC_BYTES_MASK("DRC1 Coefficients", TACNA_DRC1_CONTROL1, 4,
		   TACNA_DRC1R_EN | TACNA_DRC1L_EN),
SND_SOC_BYTES_MASK("DRC2 Coefficients", TACNA_DRC2_CONTROL1, 4,
		   TACNA_DRC2R_EN | TACNA_DRC2L_EN),

TACNA_MIXER_CONTROLS("LHPF1", TACNA_LHPF1MIX_INPUT1),
TACNA_MIXER_CONTROLS("LHPF2", TACNA_LHPF2MIX_INPUT1),
TACNA_MIXER_CONTROLS("LHPF3", TACNA_LHPF3MIX_INPUT1),
TACNA_MIXER_CONTROLS("LHPF4", TACNA_LHPF4MIX_INPUT1),

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

SOC_ENUM("Sample Rate 2", tacna_sample_rate[0]),
SOC_ENUM("Sample Rate 3", tacna_sample_rate[1]),
SOC_ENUM("Async Sample Rate 1", tacna_sample_rate[2]),
SOC_ENUM("Async Sample Rate 2", tacna_sample_rate[3]),

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

TACNA_US_RATE_ENUM("Ultrasonic 1 Rate", tacna_us_output_rate[0]),
TACNA_US_RATE_ENUM("Ultrasonic 2 Rate", tacna_us_output_rate[1]),

SOC_ENUM("Ultrasonic 1 Freq", tacna_us1_freq_enum),
SOC_ENUM("Ultrasonic 2 Freq", tacna_us2_freq_enum),

SOC_ENUM("Ultrasonic 1 Gain", tacna_us1_gain_enum),
SOC_ENUM("Ultrasonic 2 Gain", tacna_us2_gain_enum),

SOC_ENUM("Ultrasonic 1 Activity Detect Threshold", tacna_us1_det_thr_enum),
SOC_ENUM("Ultrasonic 2 Activity Detect Threshold", tacna_us2_det_thr_enum),

SOC_ENUM("Ultrasonic 1 Activity Detect Pulse Length", tacna_us1_det_num_enum),
SOC_ENUM("Ultrasonic 2 Activity Detect Pulse Length", tacna_us2_det_num_enum),

SOC_ENUM("Ultrasonic 1 Activity Detect Hold", tacna_us1_det_hold_enum),
SOC_ENUM("Ultrasonic 2 Activity Detect Hold", tacna_us2_det_hold_enum),

SOC_ENUM("Ultrasonic 1 Activity Detect Decay", tacna_us1_det_dcy_enum),
SOC_ENUM("Ultrasonic 2 Activity Detect Decay", tacna_us2_det_dcy_enum),

SOC_ENUM("AUXPDM1 Rate", tacna_auxpdm1_freq),
SOC_ENUM("AUXPDM2 Rate", tacna_auxpdm2_freq),
SOC_ENUM("AUXPDM3 Rate", tacna_auxpdm3_freq),

TACNA_MIXER_CONTROLS("OUT1L", TACNA_OUT1LMIX_INPUT1),
TACNA_MIXER_CONTROLS("OUT1R", TACNA_OUT1RMIX_INPUT1),
TACNA_MIXER_CONTROLS("OUT2L", TACNA_OUT2LMIX_INPUT1),
TACNA_MIXER_CONTROLS("OUT2R", TACNA_OUT2RMIX_INPUT1),
TACNA_MIXER_CONTROLS("OUT5L", TACNA_OUT5LMIX_INPUT1),
TACNA_MIXER_CONTROLS("OUT5R", TACNA_OUT5RMIX_INPUT1),
TACNA_MIXER_CONTROLS("OUTAUX1L", TACNA_OUTAUX1LMIX_INPUT1),
TACNA_MIXER_CONTROLS("OUTAUX1R", TACNA_OUTAUX1RMIX_INPUT1),

SOC_DOUBLE_R_EXT("OUT1 Digital Switch", TACNA_OUT1L_VOLUME_1,
		 TACNA_OUT1R_VOLUME_1, TACNA_OUT1L_MUTE_SHIFT, 1, 1,
		 snd_soc_get_volsw, tacna_put_out_vu),
SOC_DOUBLE_R_EXT("OUTAUX1 Digital Switch", TACNA_OUTAUX1L_VOLUME_1,
		 TACNA_OUTAUX1R_VOLUME_1, TACNA_OUTAUX1L_MUTE_SHIFT, 1, 1,
		 snd_soc_get_volsw, cs47l94_put_outaux_vu),
SOC_DOUBLE_R_EXT("OUT2 Digital Switch", TACNA_OUT2L_VOLUME_1,
		 TACNA_OUT2R_VOLUME_1, TACNA_OUT2L_MUTE_SHIFT, 1, 1,
		 snd_soc_get_volsw, tacna_put_out_vu),
SOC_DOUBLE_R_EXT("OUT5 Digital Switch", TACNA_OUT5L_VOLUME_1,
		 TACNA_OUT5R_VOLUME_1, TACNA_OUT5L_MUTE_SHIFT, 1, 1,
		 snd_soc_get_volsw, tacna_put_out_vu),

SOC_DOUBLE_R_EXT_TLV("OUT1 Main Volume", TACNA_OUT1L_VOLUME_1,
		     TACNA_OUT1R_VOLUME_1, TACNA_OUT1L_VOL_SHIFT,
		     0xc0, 0, snd_soc_get_volsw, tacna_put_out_vu,
		     cs47l94_aux_tlv),
SOC_DOUBLE_R_EXT_TLV("OUTAUX1 Volume", TACNA_OUTAUX1L_VOLUME_1,
		     TACNA_OUTAUX1R_VOLUME_1, TACNA_OUTAUX1L_VOL_SHIFT,
		     0xc0, 0, snd_soc_get_volsw, cs47l94_put_outaux_vu,
		     cs47l94_aux_tlv),
SOC_DOUBLE_R_EXT_TLV("OUT1 Digital Volume", TACNA_OUT1L_VOLUME_3,
		     TACNA_OUT1R_VOLUME_3, TACNA_OUT1L_LVL_SHIFT,
		     0xbf, 0, snd_soc_get_volsw, tacna_put_out_vu,
		     tacna_digital_tlv),
SOC_DOUBLE_R_EXT_TLV("OUT2 Digital Volume", TACNA_OUT2L_VOLUME_1,
		     TACNA_OUT2R_VOLUME_1, TACNA_OUT2L_VOL_SHIFT,
		     0xbf, 0, snd_soc_get_volsw, tacna_put_out_vu,
		     tacna_digital_tlv),
SOC_DOUBLE_R_EXT_TLV("OUT5 Digital Volume", TACNA_OUT5L_VOLUME_1,
		     TACNA_OUT5R_VOLUME_1, TACNA_OUT5L_VOL_SHIFT,
		     0xbf, 0, snd_soc_get_volsw, tacna_put_out_vu,
		     tacna_digital_tlv),

SOC_ENUM("Output Ramp Up", tacna_out_vi_ramp),
SOC_ENUM("Output Ramp Down", tacna_out_vd_ramp),

TACNA_RATE_ENUM("Output Rate 1", tacna_output_rate),

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

TACNA_RATE_ENUM("OUTH Rate", cs47l94_outh_rate),

SOC_DOUBLE_R_EXT_TLV("OUTH Main Volume", TACNA_OUTHL_VOLUME_1,
		     TACNA_OUTHR_VOLUME_1, TACNA_OUTHL_VOL_SHIFT, 0xc0, 0,
		     cs47l94_get_outh_main_volume, cs47l94_put_outh_main_volume,
		     cs47l94_aux_tlv),

SOC_DOUBLE_TLV("OUTH DSD Digital Volume", TACNA_DSD1_VOLUME1,
	       TACNA_DSD1L_VOL_SHIFT, TACNA_DSD1R_VOL_SHIFT, 0xfe, 1,
	       cs47l94_outh_digital_tlv),
SOC_DOUBLE("OUTH DSD Digital Switch", TACNA_DSD1_VOLUME1,
	   TACNA_DSD1L_MUTE_SHIFT, TACNA_DSD1R_MUTE_SHIFT, 1, 1),

SOC_ENUM("DoP Data Width", cs47l94_dop_width_enum),

SOC_DOUBLE_TLV("OUTH PCM Digital Volume", TACNA_OUTH_PCM_CONTROL1,
	       TACNA_OUTHL_LVL_SHIFT, TACNA_OUTHR_LVL_SHIFT, 0xfe, 1,
	       cs47l94_outh_digital_tlv),
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

TACNA_MIXER_CONTROLS("ASP1TX1", TACNA_ASP1TX1MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP1TX2", TACNA_ASP1TX2MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP1TX3", TACNA_ASP1TX3MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP1TX4", TACNA_ASP1TX4MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP1TX5", TACNA_ASP1TX5MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP1TX6", TACNA_ASP1TX6MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP1TX7", TACNA_ASP1TX7MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP1TX8", TACNA_ASP1TX8MIX_INPUT1),

TACNA_MIXER_CONTROLS("ASP2TX1", TACNA_ASP2TX1MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP2TX2", TACNA_ASP2TX2MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP2TX3", TACNA_ASP2TX3MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP2TX4", TACNA_ASP2TX4MIX_INPUT1),

TACNA_MIXER_CONTROLS("ASP3TX1", TACNA_ASP3TX1MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP3TX2", TACNA_ASP3TX2MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP3TX3", TACNA_ASP3TX3MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP3TX4", TACNA_ASP3TX4MIX_INPUT1),

TACNA_MIXER_CONTROLS("ASP4TX1", TACNA_ASP4TX1MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP4TX2", TACNA_ASP4TX2MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP4TX3", TACNA_ASP4TX3MIX_INPUT1),
TACNA_MIXER_CONTROLS("ASP4TX4", TACNA_ASP4TX4MIX_INPUT1),

TACNA_MIXER_CONTROLS("SLIMTX1", TACNA_SLIMTX1MIX_INPUT1),
TACNA_MIXER_CONTROLS("SLIMTX2", TACNA_SLIMTX2MIX_INPUT1),
TACNA_MIXER_CONTROLS("SLIMTX3", TACNA_SLIMTX3MIX_INPUT1),
TACNA_MIXER_CONTROLS("SLIMTX4", TACNA_SLIMTX4MIX_INPUT1),
TACNA_MIXER_CONTROLS("SLIMTX5", TACNA_SLIMTX5MIX_INPUT1),
TACNA_MIXER_CONTROLS("SLIMTX6", TACNA_SLIMTX6MIX_INPUT1),
TACNA_MIXER_CONTROLS("SLIMTX7", TACNA_SLIMTX7MIX_INPUT1),
TACNA_MIXER_CONTROLS("SLIMTX8", TACNA_SLIMTX8MIX_INPUT1),

WM_ADSP2_PRELOAD_SWITCH("DSP1", 1),
WM_ADSP2_PRELOAD_SWITCH("DSP2", 2),

TACNA_MIXER_CONTROLS("DSP1RX1", TACNA_DSP1RX1MIX_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX2", TACNA_DSP1RX2MIX_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX3", TACNA_DSP1RX3MIX_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX4", TACNA_DSP1RX4MIX_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX5", TACNA_DSP1RX5MIX_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX6", TACNA_DSP1RX6MIX_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX7", TACNA_DSP1RX7MIX_INPUT1),
TACNA_MIXER_CONTROLS("DSP1RX8", TACNA_DSP1RX8MIX_INPUT1),

TACNA_MIXER_CONTROLS("DSP2RX1", TACNA_DSP2RX1MIX_INPUT1),
TACNA_MIXER_CONTROLS("DSP2RX2", TACNA_DSP2RX2MIX_INPUT1),
TACNA_MIXER_CONTROLS("DSP2RX3", TACNA_DSP2RX3MIX_INPUT1),
TACNA_MIXER_CONTROLS("DSP2RX4", TACNA_DSP2RX4MIX_INPUT1),
TACNA_MIXER_CONTROLS("DSP2RX5", TACNA_DSP2RX5MIX_INPUT1),
TACNA_MIXER_CONTROLS("DSP2RX6", TACNA_DSP2RX6MIX_INPUT1),
TACNA_MIXER_CONTROLS("DSP2RX7", TACNA_DSP2RX7MIX_INPUT1),
TACNA_MIXER_CONTROLS("DSP2RX8", TACNA_DSP2RX8MIX_INPUT1),
};

TACNA_MIXER_ENUMS(EQ1, TACNA_EQ1MIX_INPUT1);
TACNA_MIXER_ENUMS(EQ2, TACNA_EQ2MIX_INPUT1);
TACNA_MIXER_ENUMS(EQ3, TACNA_EQ3MIX_INPUT1);
TACNA_MIXER_ENUMS(EQ4, TACNA_EQ4MIX_INPUT1);

TACNA_MIXER_ENUMS(DRC1L, TACNA_DRC1LMIX_INPUT1);
TACNA_MIXER_ENUMS(DRC1R, TACNA_DRC1RMIX_INPUT1);
TACNA_MIXER_ENUMS(DRC2L, TACNA_DRC2LMIX_INPUT1);
TACNA_MIXER_ENUMS(DRC2R, TACNA_DRC2RMIX_INPUT1);

TACNA_MIXER_ENUMS(LHPF1, TACNA_LHPF1MIX_INPUT1);
TACNA_MIXER_ENUMS(LHPF2, TACNA_LHPF2MIX_INPUT1);
TACNA_MIXER_ENUMS(LHPF3, TACNA_LHPF3MIX_INPUT1);
TACNA_MIXER_ENUMS(LHPF4, TACNA_LHPF4MIX_INPUT1);

TACNA_MIXER_ENUMS(PWM1, TACNA_PWM1MIX_INPUT1);
TACNA_MIXER_ENUMS(PWM2, TACNA_PWM2MIX_INPUT1);

TACNA_MIXER_ENUMS(OUT1L, TACNA_OUT1LMIX_INPUT1);
TACNA_MIXER_ENUMS(OUT1R, TACNA_OUT1RMIX_INPUT1);
TACNA_MIXER_ENUMS(OUT2L, TACNA_OUT2LMIX_INPUT1);
TACNA_MIXER_ENUMS(OUT2R, TACNA_OUT2RMIX_INPUT1);
TACNA_MIXER_ENUMS(OUT5L, TACNA_OUT5LMIX_INPUT1);
TACNA_MIXER_ENUMS(OUT5R, TACNA_OUT5RMIX_INPUT1);
TACNA_MIXER_ENUMS(OUTAUX1L, TACNA_OUTAUX1LMIX_INPUT1);
TACNA_MIXER_ENUMS(OUTAUX1R, TACNA_OUTAUX1RMIX_INPUT1);
TACNA_MIXER_ENUMS(OUTHL, TACNA_OUTHLMIX_INPUT1);
TACNA_MIXER_ENUMS(OUTHR, TACNA_OUTHRMIX_INPUT1);

TACNA_MIXER_ENUMS(ASP1TX1, TACNA_ASP1TX1MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP1TX2, TACNA_ASP1TX2MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP1TX3, TACNA_ASP1TX3MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP1TX4, TACNA_ASP1TX4MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP1TX5, TACNA_ASP1TX5MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP1TX6, TACNA_ASP1TX6MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP1TX7, TACNA_ASP1TX7MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP1TX8, TACNA_ASP1TX8MIX_INPUT1);

TACNA_MIXER_ENUMS(ASP2TX1, TACNA_ASP2TX1MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP2TX2, TACNA_ASP2TX2MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP2TX3, TACNA_ASP2TX3MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP2TX4, TACNA_ASP2TX4MIX_INPUT1);

TACNA_MIXER_ENUMS(ASP3TX1, TACNA_ASP3TX1MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP3TX2, TACNA_ASP3TX2MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP3TX3, TACNA_ASP3TX3MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP3TX4, TACNA_ASP3TX4MIX_INPUT1);

TACNA_MIXER_ENUMS(ASP4TX1, TACNA_ASP4TX1MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP4TX2, TACNA_ASP4TX2MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP4TX3, TACNA_ASP4TX3MIX_INPUT1);
TACNA_MIXER_ENUMS(ASP4TX4, TACNA_ASP4TX4MIX_INPUT1);

TACNA_MIXER_ENUMS(SLIMTX1, TACNA_SLIMTX1MIX_INPUT1);
TACNA_MIXER_ENUMS(SLIMTX2, TACNA_SLIMTX2MIX_INPUT1);
TACNA_MIXER_ENUMS(SLIMTX3, TACNA_SLIMTX3MIX_INPUT1);
TACNA_MIXER_ENUMS(SLIMTX4, TACNA_SLIMTX4MIX_INPUT1);
TACNA_MIXER_ENUMS(SLIMTX5, TACNA_SLIMTX5MIX_INPUT1);
TACNA_MIXER_ENUMS(SLIMTX6, TACNA_SLIMTX6MIX_INPUT1);
TACNA_MIXER_ENUMS(SLIMTX7, TACNA_SLIMTX7MIX_INPUT1);
TACNA_MIXER_ENUMS(SLIMTX8, TACNA_SLIMTX8MIX_INPUT1);

TACNA_MUX_ENUMS(ASRC1IN1L, TACNA_ASRC1_IN1L_INPUT1);
TACNA_MUX_ENUMS(ASRC1IN1R, TACNA_ASRC1_IN1R_INPUT1);
TACNA_MUX_ENUMS(ASRC1IN2L, TACNA_ASRC1_IN2L_INPUT1);
TACNA_MUX_ENUMS(ASRC1IN2R, TACNA_ASRC1_IN2R_INPUT1);

TACNA_MUX_ENUMS(ISRC1INT1, TACNA_ISRC1INT1_INPUT1);
TACNA_MUX_ENUMS(ISRC1INT2, TACNA_ISRC1INT2_INPUT1);

TACNA_MUX_ENUMS(ISRC1DEC1, TACNA_ISRC1DEC1_INPUT1);
TACNA_MUX_ENUMS(ISRC1DEC2, TACNA_ISRC1DEC2_INPUT1);

TACNA_MUX_ENUMS(ISRC2INT1, TACNA_ISRC2INT1_INPUT1);
TACNA_MUX_ENUMS(ISRC2INT2, TACNA_ISRC2INT2_INPUT1);

TACNA_MUX_ENUMS(ISRC2DEC1, TACNA_ISRC2DEC1_INPUT1);
TACNA_MUX_ENUMS(ISRC2DEC2, TACNA_ISRC2DEC2_INPUT1);

TACNA_MUX_ENUMS(DFC1, TACNA_DFC1_CH1_INPUT1);
TACNA_MUX_ENUMS(DFC2, TACNA_DFC1_CH2_INPUT1);
TACNA_MUX_ENUMS(DFC3, TACNA_DFC1_CH3_INPUT1);
TACNA_MUX_ENUMS(DFC4, TACNA_DFC1_CH4_INPUT1);
TACNA_MUX_ENUMS(DFC5, TACNA_DFC1_CH5_INPUT1);
TACNA_MUX_ENUMS(DFC6, TACNA_DFC1_CH6_INPUT1);
TACNA_MUX_ENUMS(DFC7, TACNA_DFC1_CH7_INPUT1);
TACNA_MUX_ENUMS(DFC8, TACNA_DFC1_CH8_INPUT1);

TACNA_MIXER_ENUMS(DSP1RX1, TACNA_DSP1RX1MIX_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX2, TACNA_DSP1RX2MIX_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX3, TACNA_DSP1RX3MIX_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX4, TACNA_DSP1RX4MIX_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX5, TACNA_DSP1RX5MIX_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX6, TACNA_DSP1RX6MIX_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX7, TACNA_DSP1RX7MIX_INPUT1);
TACNA_MIXER_ENUMS(DSP1RX8, TACNA_DSP1RX8MIX_INPUT1);

TACNA_MIXER_ENUMS(DSP2RX1, TACNA_DSP2RX1MIX_INPUT1);
TACNA_MIXER_ENUMS(DSP2RX2, TACNA_DSP2RX2MIX_INPUT1);
TACNA_MIXER_ENUMS(DSP2RX3, TACNA_DSP2RX3MIX_INPUT1);
TACNA_MIXER_ENUMS(DSP2RX4, TACNA_DSP2RX4MIX_INPUT1);
TACNA_MIXER_ENUMS(DSP2RX5, TACNA_DSP2RX5MIX_INPUT1);
TACNA_MIXER_ENUMS(DSP2RX6, TACNA_DSP2RX6MIX_INPUT1);
TACNA_MIXER_ENUMS(DSP2RX7, TACNA_DSP2RX7MIX_INPUT1);
TACNA_MIXER_ENUMS(DSP2RX8, TACNA_DSP2RX8MIX_INPUT1);

static const char * const cs47l94_aec_loopback_texts[] = {
	"OUT1L", "OUT1R", "OUT2L", "OUT2R", "OUT5L", "OUT5R",
};

static const unsigned int cs47l94_aec_loopback_values[] = {
	0, 1, 2, 3, 8, 9,
};

static const struct soc_enum cs47l94_aec_loopback[] = {
	SOC_VALUE_ENUM_SINGLE(TACNA_OUTPUT_AEC_CONTROL_1,
			      TACNA_AEC_LOOPBACK1_SRC_SHIFT,
			      TACNA_AEC_LOOPBACK1_SRC_MASK >>
			      TACNA_AEC_LOOPBACK1_SRC_SHIFT,
			      ARRAY_SIZE(cs47l94_aec_loopback_texts),
			      cs47l94_aec_loopback_texts,
			      cs47l94_aec_loopback_values),
	SOC_VALUE_ENUM_SINGLE(TACNA_OUTPUT_AEC_CONTROL_1,
			      TACNA_AEC_LOOPBACK2_SRC_SHIFT,
			      TACNA_AEC_LOOPBACK2_SRC_MASK >>
			      TACNA_AEC_LOOPBACK2_SRC_SHIFT,
			      ARRAY_SIZE(cs47l94_aec_loopback_texts),
			      cs47l94_aec_loopback_texts,
			      cs47l94_aec_loopback_values),
};

static const struct snd_kcontrol_new cs47l94_aec_loopback_mux[] = {
	SOC_DAPM_ENUM("AEC1 Loopback", cs47l94_aec_loopback[0]),
	SOC_DAPM_ENUM("AEC2 Loopback", cs47l94_aec_loopback[1]),
};

static const struct snd_kcontrol_new cs47l94_anc_input_mux[] = {
	SOC_DAPM_ENUM("ANCL Input", tacna_mono_anc_input_src[0]),
	SOC_DAPM_ENUM("ANCL Channel", tacna_mono_anc_input_src[1]),
};

static const struct snd_kcontrol_new cs47l94_anc_ng_mux =
	SOC_DAPM_ENUM("ANC NG Source", tacna_anc_ng_enum);

static const struct snd_kcontrol_new cs47l94_output_anc_src[] = {
	SOC_DAPM_ENUM("OUT1L ANC Source", tacna_output_anc_src[0]),
	SOC_DAPM_ENUM("OUT1R ANC Source", tacna_output_anc_src[1]),
	SOC_DAPM_ENUM("OUT2L ANC Source", tacna_output_anc_src[2]),
	SOC_DAPM_ENUM("OUT2R ANC Source", tacna_output_anc_src[3]),
	SOC_DAPM_ENUM("OUT5L ANC Source", tacna_output_anc_src[4]),
	SOC_DAPM_ENUM("OUT5R ANC Source", tacna_output_anc_src[5]),
};

static const struct snd_kcontrol_new cs47l94_out1_aux_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
};

static const struct snd_kcontrol_new cs47l94_out1_dsd_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
};

static const char * const cs47l94_out_select_texts[] = {
	"OUT1+OUT2", "OUTH",
};

static SOC_ENUM_SINGLE_DECL(cs47l94_output_select_enum, SND_SOC_NOPM, 0,
			    cs47l94_out_select_texts);

static const struct snd_kcontrol_new cs47l94_output_select =
	SOC_DAPM_ENUM("Output Select", cs47l94_output_select_enum);

static const struct snd_kcontrol_new cs47l94_outh_aux_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
};

static const char * const cs47l94_dsd_source_texts[] = {
	"DSD", "DoP",
};

static const unsigned int cs47l94_dsd_source_values[] = {
	0x0, 0x2,
};

static const struct soc_enum cs47l94_dsd_source_enum =
	SOC_VALUE_ENUM_SINGLE(TACNA_DSD1_CONTROL1,
			      TACNA_DSD1_SRC_SHIFT,
			      TACNA_DSD1_SRC_MASK >> TACNA_DSD1_SRC_SHIFT,
			      ARRAY_SIZE(cs47l94_dsd_source_texts),
			      cs47l94_dsd_source_texts,
			      cs47l94_dsd_source_values);

static const struct snd_kcontrol_new cs47l94_dsd_source_select =
	SOC_DAPM_ENUM("Source", cs47l94_dsd_source_enum);

static const struct snd_kcontrol_new cs47l94_dsd_switch =
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_soc_dapm_widget cs47l94_dapm_widgets[] = {
SND_SOC_DAPM_SUPPLY("SYSCLK", TACNA_SYSTEM_CLOCK1, TACNA_SYSCLK_EN_SHIFT,
		    0, tacna_sysclk_ev,
		    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_SUPPLY("ASYNCCLK", TACNA_ASYNC_CLOCK1, TACNA_ASYNC_CLK_EN_SHIFT,
		    0, NULL, 0),
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

SND_SOC_DAPM_SUPPLY("FXCLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_FX, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("ASRC1R1CLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_ASRC1_RATE_1, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("ASRC1R2CLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_ASRC1_RATE_2, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("ASRC2R1CLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_ASRC2_RATE_1, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("ASRC2R2CLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_ASRC2_RATE_2, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("ISRC1DECCLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_ISRC1_DEC, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("ISRC1INTCLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_ISRC1_INT, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("ISRC2DECCLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_ISRC2_DEC, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("ISRC2INTCLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_ISRC2_INT, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("DACOUTCLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_DAC, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("ASP1TXCLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_ASP1, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("ASP2TXCLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_ASP2, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("ASP3TXCLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_ASP3, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("ASP4TXCLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_ASP4, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("SLIMBUSCLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_SLIMBUS, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("PWMCLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_PWM, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("DFC1CLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_DFC1, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("DFC2CLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_DFC2, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("DFC3CLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_DFC3, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("DFC4CLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_DFC4, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("DFC5CLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_DFC5, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("DFC6CLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_DFC6, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("DFC7CLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_DFC7, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("DFC8CLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_DFC8, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("DSP1CLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_DSP1, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_SUPPLY("DSP2CLK", SND_SOC_NOPM,
		    TACNA_DOM_GRP_DSP2, 0,
		    tacna_domain_clk_ev,
		    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

SND_SOC_DAPM_SIGGEN("TONE"),
SND_SOC_DAPM_SIGGEN("NOISE"),

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

SND_SOC_DAPM_MUX("Ultrasonic 1 Input", SND_SOC_NOPM, 0, 0, &tacna_us_inmux[0]),
SND_SOC_DAPM_MUX("Ultrasonic 2 Input", SND_SOC_NOPM, 0, 0, &tacna_us_inmux[1]),

SND_SOC_DAPM_OUTPUT("DRC1 Signal Activity"),
SND_SOC_DAPM_OUTPUT("DRC2 Signal Activity"),

SND_SOC_DAPM_OUTPUT("Ultrasonic 1 Activity Output"),
SND_SOC_DAPM_OUTPUT("Ultrasonic 2 Activity Output"),

SND_SOC_DAPM_OUTPUT("DSP Trigger Out"),

SND_SOC_DAPM_MUX("IN1L Mux", SND_SOC_NOPM, 0, 0, &tacna_inmux[0]),
SND_SOC_DAPM_MUX("IN1R Mux", SND_SOC_NOPM, 0, 0, &tacna_inmux[1]),
SND_SOC_DAPM_MUX("IN2L Mux", SND_SOC_NOPM, 0, 0, &tacna_inmux[2]),
SND_SOC_DAPM_MUX("IN2R Mux", SND_SOC_NOPM, 0, 0, &tacna_inmux[3]),

SND_SOC_DAPM_SWITCH("IN1 Digital Mode", SND_SOC_NOPM, 0, 0,
		    &tacna_inmode_switch[0]),
SND_SOC_DAPM_SWITCH("IN2 Digital Mode", SND_SOC_NOPM, 0, 0,
		    &tacna_inmode_switch[1]),

SND_SOC_DAPM_PGA("PWM1 Driver", TACNA_PWM_DRIVE_1, TACNA_PWM1_EN_SHIFT,
		 0, NULL, 0),
SND_SOC_DAPM_PGA("PWM2 Driver", TACNA_PWM_DRIVE_1, TACNA_PWM2_EN_SHIFT,
		 0, NULL, 0),

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

SND_SOC_DAPM_AIF_OUT("ASP4TX1", NULL, 0, TACNA_ASP4_ENABLES1,
		     TACNA_ASP4_TX1_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP4TX2", NULL, 0, TACNA_ASP4_ENABLES1,
		     TACNA_ASP4_TX2_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP4TX3", NULL, 0, TACNA_ASP4_ENABLES1,
		     TACNA_ASP4_TX3_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP4TX4", NULL, 0, TACNA_ASP4_ENABLES1,
		     TACNA_ASP4_TX4_EN_SHIFT, 0),

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
		   TACNA_OUT1L_EN_SHIFT, 0, NULL, 0, cs47l94_out_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("OUT1R PGA", SND_SOC_NOPM,
		   TACNA_OUT1R_EN_SHIFT, 0, NULL, 0, cs47l94_out_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("OUT2L PGA", SND_SOC_NOPM,
		   TACNA_OUT2L_EN_SHIFT, 0, NULL, 0, cs47l94_out_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("OUT2R PGA", SND_SOC_NOPM,
		   TACNA_OUT2R_EN_SHIFT, 0, NULL, 0, cs47l94_out_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA("OUT5L PGA", TACNA_OUTPUT_ENABLE_1,
		   TACNA_OUT5L_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("OUT5R PGA", TACNA_OUTPUT_ENABLE_1,
		   TACNA_OUT5R_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA_E("OUTAUX1L PGA", TACNA_OUTAUX1L_ENABLE_1,
		   TACNA_OUTAUX1L_EN_SHIFT, 0, NULL, 0, cs47l94_outaux_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("OUTAUX1R PGA", TACNA_OUTAUX1R_ENABLE_1,
		   TACNA_OUTAUX1R_EN_SHIFT, 0, NULL, 0, cs47l94_outaux_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),

SND_SOC_DAPM_SWITCH_E("OUT1L AUX Mix", TACNA_OUT1L_CONTROL_1,
		      TACNA_OUT1L_AUX_SRC_SHIFT, 0,
		      &cs47l94_out1_aux_switch[0], cs47l94_out_aux_src_ev,
		      SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_SWITCH_E("OUT1R AUX Mix", TACNA_OUT1R_CONTROL_1,
		      TACNA_OUT1R_AUX_SRC_SHIFT, 0,
		      &cs47l94_out1_aux_switch[1], cs47l94_out_aux_src_ev,
		      SND_SOC_DAPM_POST_PMU),

SND_SOC_DAPM_SWITCH("OUT1L DSD Mix", TACNA_OUT1L_CONTROL_1,
		      TACNA_OUT1L_DSD_EN_SHIFT, 0,
		      &cs47l94_out1_dsd_switch[0]),
SND_SOC_DAPM_SWITCH("OUT1R DSD Mix", TACNA_OUT1R_CONTROL_1,
		      TACNA_OUT1R_DSD_EN_SHIFT, 0,
		      &cs47l94_out1_dsd_switch[1]),

SND_SOC_DAPM_DEMUX("OUT1 Demux", SND_SOC_NOPM, 0, 0, &cs47l94_out1_demux),

SND_SOC_DAPM_MUX("OUT1L Output Select", SND_SOC_NOPM, 0, 0,
		 &cs47l94_output_select),
SND_SOC_DAPM_MUX("OUT1R Output Select", SND_SOC_NOPM, 0, 0,
		 &cs47l94_output_select),
SND_SOC_DAPM_MUX("OUT2L Output Select", SND_SOC_NOPM, 0, 0,
		 &cs47l94_output_select),
SND_SOC_DAPM_MUX("OUT2R Output Select", SND_SOC_NOPM, 0, 0,
		 &cs47l94_output_select),
SND_SOC_DAPM_MUX_E("OUTH Output Select", SND_SOC_NOPM, 0, 0,
		   &cs47l94_output_select, cs47l94_outh_ev,
		   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD |
		   SND_SOC_DAPM_POST_PMU),

SND_SOC_DAPM_SWITCH_E("OUTHL AUX Mix", TACNA_OUTH_AUX_MIX_CONTROL_1,
		      TACNA_OUTHL_AUX_SRC_SHIFT, 0,
		      &cs47l94_outh_aux_switch[0], cs47l94_outh_aux_src_ev,
		      SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_SWITCH_E("OUTHR AUX Mix", TACNA_OUTH_AUX_MIX_CONTROL_1,
		      TACNA_OUTHR_AUX_SRC_SHIFT, 0,
		      &cs47l94_outh_aux_switch[1], cs47l94_outh_aux_src_ev,
		      SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

SND_SOC_DAPM_PGA("OUTH PCM", TACNA_OUTH_ENABLE_1, TACNA_OUTH_PCM_EN_SHIFT,
		 0, NULL, 0),

SND_SOC_DAPM_MUX("DSD Processor Source", SND_SOC_NOPM, 0, 0,
		 &cs47l94_dsd_source_select),

SND_SOC_DAPM_SWITCH_E("DSD Processor", TACNA_DSD1_CONTROL1,
		      TACNA_DSD1_EN_SHIFT, 0,
		      &cs47l94_dsd_switch, cs47l94_dsd_processor_ev,
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

SND_SOC_DAPM_SWITCH("Ultrasonic 1 Activity Detect", TACNA_US1_DET_CONTROL,
		    TACNA_US1_DET_EN_SHIFT, 0, &cs47l94_us1_switch),
SND_SOC_DAPM_SWITCH("Ultrasonic 2 Activity Detect", TACNA_US2_DET_CONTROL,
		    TACNA_US2_DET_EN_SHIFT, 0, &cs47l94_us2_switch),

/* mux_in widgets : arranged in the order of sources
   specified in TACNA_MIXER_INPUT_ROUTES */

SND_SOC_DAPM_PGA("Tone Generator 1", TACNA_TONE_GENERATOR1,
		 TACNA_TONE1_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("Tone Generator 2", TACNA_TONE_GENERATOR1,
		 TACNA_TONE2_EN_SHIFT, 0, NULL, 0),

SND_SOC_DAPM_MIC("HAPTICS", NULL),

SND_SOC_DAPM_MUX("AEC1 Loopback", TACNA_OUTPUT_AEC_ENABLE_1,
		 TACNA_AEC_LOOPBACK1_EN_SHIFT, 0,
		 &cs47l94_aec_loopback_mux[0]),
SND_SOC_DAPM_MUX("AEC2 Loopback", TACNA_OUTPUT_AEC_ENABLE_1,
		 TACNA_AEC_LOOPBACK2_EN_SHIFT, 0,
		 &cs47l94_aec_loopback_mux[1]),

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

SND_SOC_DAPM_AIF_IN("ASP4RX1", NULL, 0, TACNA_ASP4_ENABLES1,
		    TACNA_ASP4_RX1_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP4RX2", NULL, 0, TACNA_ASP4_ENABLES1,
		    TACNA_ASP4_RX2_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP4RX3", NULL, 0, TACNA_ASP4_ENABLES1,
		    TACNA_ASP4_RX3_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP4RX4", NULL, 0, TACNA_ASP4_ENABLES1,
		    TACNA_ASP4_RX4_EN_SHIFT, 0),

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

WM_HALO("DSP1", 0, tacna_dsp_power_ev),
WM_HALO("DSP2", 1, tacna_dsp_power_ev),

SND_SOC_DAPM_PGA("Ultrasonic 1", TACNA_US1_CONTROL,
		 TACNA_US1_EN_SHIFT, 0, NULL, 0),
SND_SOC_DAPM_PGA("Ultrasonic 2", TACNA_US2_CONTROL,
		 TACNA_US2_EN_SHIFT, 0, NULL, 0),

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

TACNA_MIXER_WIDGETS(PWM1, "PWM1"),
TACNA_MIXER_WIDGETS(PWM2, "PWM2"),

TACNA_MIXER_WIDGETS(OUT1L, "OUT1L"),
TACNA_MIXER_WIDGETS(OUT1R, "OUT1R"),
TACNA_MIXER_WIDGETS(OUT2L, "OUT2L"),
TACNA_MIXER_WIDGETS(OUT2R, "OUT2R"),
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

TACNA_MIXER_WIDGETS(ASP4TX1, "ASP4TX1"),
TACNA_MIXER_WIDGETS(ASP4TX2, "ASP4TX2"),
TACNA_MIXER_WIDGETS(ASP4TX3, "ASP4TX3"),
TACNA_MIXER_WIDGETS(ASP4TX4, "ASP4TX4"),

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

TACNA_MUX_WIDGETS(ISRC1DEC1, "ISRC1DEC1"),
TACNA_MUX_WIDGETS(ISRC1DEC2, "ISRC1DEC2"),

TACNA_MUX_WIDGETS(ISRC1INT1, "ISRC1INT1"),
TACNA_MUX_WIDGETS(ISRC1INT2, "ISRC1INT2"),

TACNA_MUX_WIDGETS(ISRC2DEC1, "ISRC2DEC1"),
TACNA_MUX_WIDGETS(ISRC2DEC2, "ISRC2DEC2"),

TACNA_MUX_WIDGETS(ISRC2INT1, "ISRC2INT1"),
TACNA_MUX_WIDGETS(ISRC2INT2, "ISRC2INT2"),

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

TACNA_MIXER_WIDGETS(DSP2RX1, "DSP2RX1"),
TACNA_MIXER_WIDGETS(DSP2RX2, "DSP2RX2"),
TACNA_MIXER_WIDGETS(DSP2RX3, "DSP2RX3"),
TACNA_MIXER_WIDGETS(DSP2RX4, "DSP2RX4"),
TACNA_MIXER_WIDGETS(DSP2RX5, "DSP2RX5"),
TACNA_MIXER_WIDGETS(DSP2RX6, "DSP2RX6"),
TACNA_MIXER_WIDGETS(DSP2RX7, "DSP2RX7"),
TACNA_MIXER_WIDGETS(DSP2RX8, "DSP2RX8"),

SND_SOC_DAPM_SWITCH("DSP1 Trigger Output", SND_SOC_NOPM, 0, 0,
		    &tacna_dsp_trigger_output_mux[0]),
SND_SOC_DAPM_SWITCH("DSP2 Trigger Output", SND_SOC_NOPM, 0, 0,
		    &tacna_dsp_trigger_output_mux[1]),

SND_SOC_DAPM_SUPPLY("ANC NG External Clock", SND_SOC_NOPM,
		    TACNA_ANC_EXT_NG_SET_SHIFT, 0, tacna_anc_ev,
		    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_PGA("ANCL NG External", SND_SOC_NOPM, 0, 0, NULL, 0),

SND_SOC_DAPM_SUPPLY("ANC NG Clock", SND_SOC_NOPM,
		    TACNA_ANC_NG_CLK_SET_SHIFT, 0, tacna_anc_ev,
		    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_PGA("ANCL NG Internal", SND_SOC_NOPM, 0, 0, NULL, 0),

SND_SOC_DAPM_MUX("ANCL Left Input", SND_SOC_NOPM, 0, 0,
		 &cs47l94_anc_input_mux[0]),
SND_SOC_DAPM_MUX("ANCL Right Input", SND_SOC_NOPM, 0, 0,
		 &cs47l94_anc_input_mux[0]),
SND_SOC_DAPM_MUX("ANCL Channel", SND_SOC_NOPM, 0, 0,
		 &cs47l94_anc_input_mux[1]),
SND_SOC_DAPM_MUX("ANCL NG Mux", SND_SOC_NOPM, 0, 0, &cs47l94_anc_ng_mux),

SND_SOC_DAPM_PGA_E("ANCL", SND_SOC_NOPM, TACNA_ANC_L_CLK_SET_SHIFT,
		   0, NULL, 0, tacna_anc_ev,
		   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

SND_SOC_DAPM_MUX("OUT1L ANC Source", SND_SOC_NOPM, 0, 0,
		 &cs47l94_output_anc_src[0]),
SND_SOC_DAPM_MUX("OUT1R ANC Source", SND_SOC_NOPM, 0, 0,
		 &cs47l94_output_anc_src[1]),
SND_SOC_DAPM_MUX("OUT2L ANC Source", SND_SOC_NOPM, 0, 0,
		 &cs47l94_output_anc_src[2]),
SND_SOC_DAPM_MUX("OUT2R ANC Source", SND_SOC_NOPM, 0, 0,
		 &cs47l94_output_anc_src[3]),
SND_SOC_DAPM_MUX("OUT5L ANC Source", SND_SOC_NOPM, 0, 0,
		 &cs47l94_output_anc_src[4]),
SND_SOC_DAPM_MUX("OUT5R ANC Source", SND_SOC_NOPM, 0, 0,
		 &cs47l94_output_anc_src[5]),

SND_SOC_DAPM_OUTPUT("OUT1L_HP1"),
SND_SOC_DAPM_OUTPUT("OUT1R_HP1"),
SND_SOC_DAPM_OUTPUT("OUT1L_HP2"),
SND_SOC_DAPM_OUTPUT("OUT1R_HP2"),
SND_SOC_DAPM_OUTPUT("OUT2L_HP"),
SND_SOC_DAPM_OUTPUT("OUT2R_HP"),
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
	{ name, "ASP4RX1", "ASP4RX1" }, \
	{ name, "ASP4RX2", "ASP4RX2" }, \
	{ name, "ASP4RX3", "ASP4RX3" }, \
	{ name, "ASP4RX4", "ASP4RX4" }, \
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
	{ name, "Ultrasonic 1", "Ultrasonic 1" }, \
	{ name, "Ultrasonic 2", "Ultrasonic 2" }, \
	{ name, "DFC1", "DFC1" }, \
	{ name, "DFC2", "DFC2" }, \
	{ name, "DFC3", "DFC3" }, \
	{ name, "DFC4", "DFC4" }, \
	{ name, "DFC5", "DFC5" }, \
	{ name, "DFC6", "DFC6" }, \
	{ name, "DFC7", "DFC7" }, \
	{ name, "DFC8", "DFC8" }, \
	{ name, "DSP1.1", "DSP1" }, \
	{ name, "DSP1.2", "DSP1" }, \
	{ name, "DSP1.3", "DSP1" }, \
	{ name, "DSP1.4", "DSP1" }, \
	{ name, "DSP1.5", "DSP1" }, \
	{ name, "DSP1.6", "DSP1" }, \
	{ name, "DSP1.7", "DSP1" }, \
	{ name, "DSP1.8", "DSP1" }, \
	{ name, "DSP2.1", "DSP2" }, \
	{ name, "DSP2.2", "DSP2" }, \
	{ name, "DSP2.3", "DSP2" }, \
	{ name, "DSP2.4", "DSP2" }, \
	{ name, "DSP2.5", "DSP2" }, \
	{ name, "DSP2.6", "DSP2" }, \
	{ name, "DSP2.7", "DSP2" }, \
	{ name, "DSP2.8", "DSP2" }

static const struct snd_soc_dapm_route cs47l94_dapm_routes[] = {
	/* Internal clock domains */
	{ "EQ1", NULL, "FXCLK" },
	{ "EQ2", NULL, "FXCLK" },
	{ "EQ3", NULL, "FXCLK" },
	{ "EQ4", NULL, "FXCLK" },
	{ "DRC1L", NULL, "FXCLK" },
	{ "DRC1R", NULL, "FXCLK" },
	{ "DRC2L", NULL, "FXCLK" },
	{ "DRC2R", NULL, "FXCLK" },
	{ "LHPF1", NULL, "FXCLK" },
	{ "LHPF2", NULL, "FXCLK" },
	{ "LHPF3", NULL, "FXCLK" },
	{ "LHPF4", NULL, "FXCLK" },
	{ "PWM1 Mixer", NULL, "PWMCLK" },
	{ "PWM2 Mixer", NULL, "PWMCLK" },
	{ "OUT1L PGA", NULL, "DACOUTCLK" },
	{ "OUT1R PGA", NULL, "DACOUTCLK" },
	{ "OUT2L PGA", NULL, "DACOUTCLK" },
	{ "OUT2R PGA", NULL, "DACOUTCLK" },
	{ "OUT5L PGA", NULL, "DACOUTCLK" },
	{ "OUT5R PGA", NULL, "DACOUTCLK" },
	{ "OUTH Output Select", NULL, "DACOUTCLK"},
	{ "ASP1TX1", NULL, "ASP1TXCLK" },
	{ "ASP1TX2", NULL, "ASP1TXCLK" },
	{ "ASP1TX3", NULL, "ASP1TXCLK" },
	{ "ASP1TX4", NULL, "ASP1TXCLK" },
	{ "ASP1TX5", NULL, "ASP1TXCLK" },
	{ "ASP1TX6", NULL, "ASP1TXCLK" },
	{ "ASP1TX7", NULL, "ASP1TXCLK" },
	{ "ASP1TX8", NULL, "ASP1TXCLK" },
	{ "ASP2TX1", NULL, "ASP2TXCLK" },
	{ "ASP2TX2", NULL, "ASP2TXCLK" },
	{ "ASP2TX3", NULL, "ASP2TXCLK" },
	{ "ASP2TX4", NULL, "ASP2TXCLK" },
	{ "ASP3TX1", NULL, "ASP3TXCLK" },
	{ "ASP3TX2", NULL, "ASP3TXCLK" },
	{ "ASP3TX3", NULL, "ASP3TXCLK" },
	{ "ASP3TX4", NULL, "ASP3TXCLK" },
	{ "ASP4TX1", NULL, "ASP4TXCLK" },
	{ "ASP4TX2", NULL, "ASP4TXCLK" },
	{ "ASP4TX3", NULL, "ASP4TXCLK" },
	{ "ASP4TX4", NULL, "ASP4TXCLK" },
	{ "SLIMTX1", NULL, "SLIMBUSCLK" },
	{ "SLIMTX2", NULL, "SLIMBUSCLK" },
	{ "SLIMTX3", NULL, "SLIMBUSCLK" },
	{ "SLIMTX4", NULL, "SLIMBUSCLK" },
	{ "SLIMTX5", NULL, "SLIMBUSCLK" },
	{ "SLIMTX6", NULL, "SLIMBUSCLK" },
	{ "SLIMTX7", NULL, "SLIMBUSCLK" },
	{ "SLIMTX8", NULL, "SLIMBUSCLK" },
	{ "ISRC1DEC1", NULL, "ISRC1DECCLK" },
	{ "ISRC1DEC2", NULL, "ISRC1DECCLK" },
	{ "ISRC1INT1", NULL, "ISRC1INTCLK" },
	{ "ISRC1INT2", NULL, "ISRC1INTCLK" },
	{ "ISRC2DEC1", NULL, "ISRC2DECCLK" },
	{ "ISRC2DEC2", NULL, "ISRC2DECCLK" },
	{ "ISRC2INT1", NULL, "ISRC2INTCLK" },
	{ "ISRC2INT2", NULL, "ISRC2INTCLK" },
	{ "ASRC1IN1L", NULL, "ASRC1R1CLK" },
	{ "ASRC1IN1R", NULL, "ASRC1R1CLK" },
	{ "ASRC1IN2L", NULL, "ASRC1R2CLK" },
	{ "ASRC1IN2R", NULL, "ASRC1R2CLK" },
	{ "DFC1", NULL, "DFC1CLK" },
	{ "DFC2", NULL, "DFC2CLK" },
	{ "DFC3", NULL, "DFC3CLK" },
	{ "DFC4", NULL, "DFC4CLK" },
	{ "DFC5", NULL, "DFC5CLK" },
	{ "DFC6", NULL, "DFC6CLK" },
	{ "DFC7", NULL, "DFC7CLK" },
	{ "DFC8", NULL, "DFC8CLK" },
	{ "DSP1", NULL, "DSP1CLK" },
	{ "DSP2", NULL, "DSP2CLK" },

	{ "OUT1L PGA", NULL, "VDD1_CP" },
	{ "OUT1L PGA", NULL, "VDD2_CP" },
	{ "OUT1L PGA", NULL, "VDD3_CP" },
	{ "OUT1R PGA", NULL, "VDD1_CP" },
	{ "OUT1R PGA", NULL, "VDD2_CP" },
	{ "OUT1R PGA", NULL, "VDD3_CP" },
	{ "OUT2L PGA", NULL, "VDD1_CP" },
	{ "OUT2L PGA", NULL, "VDD2_CP" },
	{ "OUT2L PGA", NULL, "VDD3_CP" },
	{ "OUT2R PGA", NULL, "VDD1_CP" },
	{ "OUT2R PGA", NULL, "VDD2_CP" },
	{ "OUT2R PGA", NULL, "VDD3_CP" },

	{ "OUT1L PGA", NULL, "SYSCLK" },
	{ "OUT1R PGA", NULL, "SYSCLK" },
	{ "OUT2L PGA", NULL, "SYSCLK" },
	{ "OUT2R PGA", NULL, "SYSCLK" },
	{ "OUT5L PGA", NULL, "SYSCLK" },
	{ "OUT5R PGA", NULL, "SYSCLK" },

	{ "DACOUTCLK", NULL, "DACCLK" },

	{ "IN1LN_1", NULL, "SYSCLK" },
	{ "IN1LN_2", NULL, "SYSCLK" },
	{ "IN1LP_1", NULL, "SYSCLK" },
	{ "IN1LP_2", NULL, "SYSCLK" },
	{ "IN1RN_1", NULL, "SYSCLK" },
	{ "IN1RN_2", NULL, "SYSCLK" },
	{ "IN1RP_1", NULL, "SYSCLK" },
	{ "IN1RP_2", NULL, "SYSCLK" },
	{ "IN2LN_1", NULL, "SYSCLK" },
	{ "IN2LN_2", NULL, "SYSCLK" },
	{ "IN2LP_1", NULL, "SYSCLK" },
	{ "IN2LP_2", NULL, "SYSCLK" },
	{ "IN2RN_1", NULL, "SYSCLK" },
	{ "IN2RN_2", NULL, "SYSCLK" },
	{ "IN2RP_1", NULL, "SYSCLK" },
	{ "IN2RP_2", NULL, "SYSCLK" },

	{ "IN1_PDMCLK", NULL, "SYSCLK" },
	{ "IN1_PDMDATA", NULL, "SYSCLK" },
	{ "IN2_PDMCLK", NULL, "SYSCLK" },
	{ "IN2_PDMDATA", NULL, "SYSCLK" },
	{ "IN3_PDMCLK", NULL, "SYSCLK" },
	{ "IN3_PDMDATA", NULL, "SYSCLK" },
	{ "IN4_PDMCLK", NULL, "SYSCLK" },
	{ "IN4_PDMDATA", NULL, "SYSCLK" },

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

	{ "ASP4 Capture", NULL, "ASP4TX1" },
	{ "ASP4 Capture", NULL, "ASP4TX2" },
	{ "ASP4 Capture", NULL, "ASP4TX3" },
	{ "ASP4 Capture", NULL, "ASP4TX4" },

	{ "ASP4RX1", NULL, "ASP4 Playback" },
	{ "ASP4RX2", NULL, "ASP4 Playback" },
	{ "ASP4RX3", NULL, "ASP4 Playback" },
	{ "ASP4RX4", NULL, "ASP4 Playback" },

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
	{ "ASP4 Playback", NULL, "SYSCLK" },
	{ "Slim1 Playback", NULL, "SYSCLK" },
	{ "Slim2 Playback", NULL, "SYSCLK" },
	{ "Slim3 Playback", NULL, "SYSCLK" },

	{ "ASP1 Capture", NULL, "SYSCLK" },
	{ "ASP2 Capture", NULL, "SYSCLK" },
	{ "ASP3 Capture", NULL, "SYSCLK" },
	{ "ASP4 Capture", NULL, "SYSCLK" },
	{ "Slim1 Capture", NULL, "SYSCLK" },
	{ "Slim2 Capture", NULL, "SYSCLK" },
	{ "Slim3 Capture", NULL, "SYSCLK" },

	{ "ASRC1R1CLK", NULL, "SYSCLK" },
	{ "ASRC1R2CLK", NULL, "SYSCLK" },
	{ "ASRC1R1CLK", NULL, "ASYNCCLK" },
	{ "ASRC1R2CLK", NULL, "ASYNCCLK" },

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

	{ "IN1L PGA", NULL, "IN1L Mux" },
	{ "IN1R PGA", NULL, "IN1R Mux" },

	{ "IN1L PGA", NULL, "IN1 Digital Mode" },
	{ "IN1R PGA", NULL, "IN1 Digital Mode" },
	{ "IN1 Digital Mode", "Switch", "IN1_PDMCLK" },
	{ "IN1 Digital Mode", "Switch", "IN1_PDMDATA" },

	{ "IN1L PGA", NULL, "VOUT_MIC" },
	{ "IN1R PGA", NULL, "VOUT_MIC" },

	{ "IN2L PGA", NULL, "IN2L Mux" },
	{ "IN2R PGA", NULL, "IN2R Mux" },

	{ "IN2L PGA", NULL, "IN2 Digital Mode" },
	{ "IN2R PGA", NULL, "IN2 Digital Mode" },
	{ "IN2 Digital Mode", "Switch", "IN2_PDMCLK" },
	{ "IN2 Digital Mode", "Switch", "IN2_PDMDATA" },

	{ "IN2L PGA", NULL, "VOUT_MIC" },
	{ "IN2R PGA", NULL, "VOUT_MIC" },

	{ "IN3L PGA", NULL, "IN3_PDMCLK" },
	{ "IN3R PGA", NULL, "IN3_PDMDATA" },

	{ "IN4L PGA", NULL, "IN4_PDMCLK" },
	{ "IN4R PGA", NULL, "IN4_PDMDATA" },

	{ "Ultrasonic 1", NULL, "Ultrasonic 1 Input" },
	{ "Ultrasonic 2", NULL, "Ultrasonic 2 Input" },

	{ "Ultrasonic 1 Input", "IN1L", "IN1L PGA" },
	{ "Ultrasonic 1 Input", "IN1R", "IN1R PGA" },
	{ "Ultrasonic 1 Input", "IN2L", "IN2L PGA" },
	{ "Ultrasonic 1 Input", "IN2R", "IN2R PGA" },
	{ "Ultrasonic 1 Input", "IN3L", "IN3L PGA" },
	{ "Ultrasonic 1 Input", "IN3R", "IN3R PGA" },
	{ "Ultrasonic 1 Input", "IN4L", "IN4L PGA" },
	{ "Ultrasonic 1 Input", "IN4R", "IN4R PGA" },

	{ "Ultrasonic 2 Input", "IN1L", "IN1L PGA" },
	{ "Ultrasonic 2 Input", "IN1R", "IN1R PGA" },
	{ "Ultrasonic 2 Input", "IN2L", "IN2L PGA" },
	{ "Ultrasonic 2 Input", "IN2R", "IN2R PGA" },
	{ "Ultrasonic 2 Input", "IN3L", "IN3L PGA" },
	{ "Ultrasonic 2 Input", "IN3R", "IN3R PGA" },
	{ "Ultrasonic 2 Input", "IN4L", "IN4L PGA" },
	{ "Ultrasonic 2 Input", "IN4R", "IN4R PGA" },

	TACNA_MIXER_ROUTES("OUT1L PGA", "OUT1L"),
	TACNA_MIXER_ROUTES("OUT1R PGA", "OUT1R"),
	TACNA_MIXER_ROUTES("OUT2L PGA", "OUT2L"),
	TACNA_MIXER_ROUTES("OUT2R PGA", "OUT2R"),
	TACNA_MIXER_ROUTES("OUT5L PGA", "OUT5L"),
	TACNA_MIXER_ROUTES("OUT5R PGA", "OUT5R"),
	TACNA_MIXER_ROUTES("OUTAUX1L PGA", "OUTAUX1L"),
	TACNA_MIXER_ROUTES("OUTAUX1R PGA", "OUTAUX1R"),
	TACNA_MIXER_ROUTES("OUTH PCM", "OUTHL"),
	TACNA_MIXER_ROUTES("OUTH PCM", "OUTHR"),

	TACNA_MIXER_ROUTES("PWM1 Driver", "PWM1"),
	TACNA_MIXER_ROUTES("PWM2 Driver", "PWM2"),

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

	TACNA_MIXER_ROUTES("ASP4TX1", "ASP4TX1"),
	TACNA_MIXER_ROUTES("ASP4TX2", "ASP4TX2"),
	TACNA_MIXER_ROUTES("ASP4TX3", "ASP4TX3"),
	TACNA_MIXER_ROUTES("ASP4TX4", "ASP4TX4"),

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

	TACNA_MUX_ROUTES("ISRC1INT1", "ISRC1INT1"),
	TACNA_MUX_ROUTES("ISRC1INT2", "ISRC1INT2"),

	TACNA_MUX_ROUTES("ISRC1DEC1", "ISRC1DEC1"),
	TACNA_MUX_ROUTES("ISRC1DEC2", "ISRC1DEC2"),

	TACNA_MUX_ROUTES("ISRC2INT1", "ISRC2INT1"),
	TACNA_MUX_ROUTES("ISRC2INT2", "ISRC2INT2"),

	TACNA_MUX_ROUTES("ISRC2DEC1", "ISRC2DEC1"),
	TACNA_MUX_ROUTES("ISRC2DEC2", "ISRC2DEC2"),

	TACNA_DSP_ROUTES("DSP1"),
	TACNA_DSP_ROUTES("DSP2"),

	{ "DSP Trigger Out", NULL, "DSP1 Trigger Output" },
	{ "DSP Trigger Out", NULL, "DSP2 Trigger Output" },

	{ "DSP1 Trigger Output", "Switch", "DSP1" },
	{ "DSP2 Trigger Output", "Switch", "DSP2" },

	{ "AEC1 Loopback", "OUT1L", "OUT1L PGA" },
	{ "AEC1 Loopback", "OUT1R", "OUT1R PGA" },
	{ "AEC2 Loopback", "OUT1L", "OUT1L PGA" },
	{ "AEC2 Loopback", "OUT1R", "OUT1R PGA" },
	{ "OUT1L Output Select", "OUT1+OUT2", "OUT1L PGA" },
	{ "OUT1R Output Select", "OUT1+OUT2", "OUT1R PGA" },
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

	{ "AEC1 Loopback", "OUT2L", "OUT2L PGA" },
	{ "AEC1 Loopback", "OUT2R", "OUT2R PGA" },
	{ "AEC2 Loopback", "OUT2L", "OUT2L PGA" },
	{ "AEC2 Loopback", "OUT2R", "OUT2R PGA" },
	{ "OUT2L Output Select", "OUT1+OUT2", "OUT2L PGA" },
	{ "OUT2R Output Select", "OUT1+OUT2", "OUT2R PGA" },
	{ "OUT2L_HP", NULL, "OUT2L Output Select" },
	{ "OUT2R_HP", NULL, "OUT2R Output Select" },

	{ "AEC1 Loopback", "OUT5L", "OUT5L PGA" },
	{ "AEC1 Loopback", "OUT5R", "OUT5R PGA" },
	{ "AEC2 Loopback", "OUT5L", "OUT5L PGA" },
	{ "AEC2 Loopback", "OUT5R", "OUT5R PGA" },
	{ "OUT5_PDMCLK", NULL, "OUT5L PGA" },
	{ "OUT5_PDMDATA", NULL, "OUT5L PGA" },
	{ "OUT5_PDMCLK", NULL, "OUT5R PGA" },
	{ "OUT5_PDMDATA", NULL, "OUT5R PGA" },

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

	CS47L94_ANC_INPUT_ROUTES("ANCL", "ANCL"),

	CS47L94_ANC_OUTPUT_ROUTES("OUT1L PGA", "OUT1L"),
	CS47L94_ANC_OUTPUT_ROUTES("OUT1R PGA", "OUT1R"),
	CS47L94_ANC_OUTPUT_ROUTES("OUT2L PGA", "OUT2L"),
	CS47L94_ANC_OUTPUT_ROUTES("OUT2R PGA", "OUT2R"),
	CS47L94_ANC_OUTPUT_ROUTES("OUT5L PGA", "OUT5L"),
	CS47L94_ANC_OUTPUT_ROUTES("OUT5R PGA", "OUT5R"),
};

static struct snd_soc_dai_driver cs47l94_dai[] = {
	{
		.name = "cs47l94-asp1",
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
		.name = "cs47l94-asp2",
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
		.name = "cs47l94-asp3",
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
		.name = "cs47l94-asp4",
		.id = 4,
		.base = TACNA_ASP4_ENABLES1,
		.playback = {
			.stream_name = "ASP4 Playback",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
		.capture = {
			.stream_name = "ASP4 Capture",
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
		.name = "cs47l94-slim1",
		.id = 5,
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
		.name = "cs47l94-slim2",
		.id = 6,
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
		.name = "cs47l94-slim3",
		.id = 7,
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
};

static int cs47l94_init_outh(struct cs47l94 *cs47l94)
{
	struct tacna *tacna = cs47l94->core.tacna;
	int ret;

	ret = regmap_read(tacna->regmap, TACNA_OUTHL_VOLUME_1,
			  &cs47l94->outh_main_vol[0]);
	if (ret) {
		dev_err(tacna->dev, "Error reading OUTHL volume %d\n", ret);
		return ret;
	}
	cs47l94->outh_main_vol[0] &= TACNA_OUTHL_VOL_MASK;

	ret = regmap_read(tacna->regmap, TACNA_OUTHR_VOLUME_1,
			  &cs47l94->outh_main_vol[1]);
	if (ret) {
		dev_err(tacna->dev, "Error reading OUTHR volume %d\n", ret);
		return ret;
	}
	cs47l94->outh_main_vol[1] &= TACNA_OUTHR_VOL_MASK;

	return 0;
}

static int cs47l94_codec_probe(struct snd_soc_codec *codec)
{
	struct cs47l94 *cs47l94 = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = cs47l94->core.tacna;
	int ret, i;

	cs47l94->core.tacna->dapm = snd_soc_codec_get_dapm(codec);

	ret = tacna_init_inputs(codec);
	if (ret)
		return ret;

	ret = tacna_init_auxpdm(codec, CS47L94_N_AUXPDM);
	if (ret)
		return ret;

	ret = tacna_init_outputs(codec, CS47L94_MONO_OUTPUTS);
	if (ret)
		return ret;

	ret = tacna_init_eq(&cs47l94->core);
	if (ret)
		return ret;

	ret = cs47l94_init_outh(cs47l94);
	if (ret)
		return ret;

	snd_soc_dapm_disable_pin(tacna->dapm, "HAPTICS");

	ret = tacna_request_irq(tacna, TACNA_IRQ_US1_ACT_DET_RISE,
				"Ultrasonic 1 activity",
				 cs47l94_us1_activity, tacna);
	if (ret) {
		dev_err(tacna->dev, "Failed to get Ultrasonic 1 IRQ: %d\n",
			ret);
		return ret;
	}

	ret = tacna_request_irq(tacna, TACNA_IRQ_US2_ACT_DET_RISE,
				"Ultrasonic 2 activity",
				 cs47l94_us2_activity, tacna);
	if (ret) {
		tacna_free_irq(tacna, TACNA_IRQ_US1_ACT_DET_RISE, cs47l94);
		dev_err(tacna->dev, "Failed to get Ultrasonic 2 IRQ: %d\n",
			ret);
		return ret;
	}

	ret = snd_soc_add_codec_controls(codec, tacna_dsp_rate_controls,
					 CS47L94_NUM_DSP *
					 (CS47L94_DSP_N_RX_CHANNELS +
					  CS47L94_DSP_N_TX_CHANNELS));
	if (ret)
		return ret;

	for (i = 0; i < CS47L94_NUM_DSP; ++i)
		wm_adsp2_codec_probe(&cs47l94->core.dsp[i], codec);

	return 0;
}

static int cs47l94_codec_remove(struct snd_soc_codec *codec)
{
	struct cs47l94 *cs47l94 = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = cs47l94->core.tacna;
	int i;

	for (i = 0; i < CS47L94_NUM_DSP; ++i) {
		wm_adsp2_codec_remove(&cs47l94->core.dsp[i], codec);
		/* TODO: destroy error irq */
	}

	tacna_free_irq(tacna, TACNA_IRQ_US1_ACT_DET_RISE, cs47l94);
	tacna_free_irq(tacna, TACNA_IRQ_US2_ACT_DET_RISE, cs47l94);
	tacna->dapm = NULL;

	return 0;
}

static const unsigned int cs47l94_in_vu[] = {
	TACNA_IN1L_CONTROL2,
	TACNA_IN1R_CONTROL2,
	TACNA_IN2L_CONTROL2,
	TACNA_IN2R_CONTROL2,
	TACNA_IN3L_CONTROL2,
	TACNA_IN3R_CONTROL2,
	TACNA_IN4L_CONTROL2,
	TACNA_IN4R_CONTROL2,
};

static int cs47l94_set_fll(struct snd_soc_codec *codec, int fll_id, int source,
			   unsigned int fref, unsigned int fout)
{
	struct cs47l94 *cs47l94 = snd_soc_codec_get_drvdata(codec);
	int idx;

	switch (fll_id) {
	case TACNA_FLL1_REFCLK:
		idx = 0;
		break;
	case TACNA_FLL2_REFCLK:
		idx = 1;
		break;
	case TACNA_FLL3_REFCLK:
		idx = 2;
		break;
	default:
		return -EINVAL;
	}

	return tacna_fllhj_set_refclk(&cs47l94->fll[idx], source, fref, fout);
}

static struct regmap *cs47l94_get_regmap(struct device *dev)
{
	struct cs47l94 *cs47l94 = dev_get_drvdata(dev);

	return cs47l94->core.tacna->regmap;
}

static struct snd_soc_codec_driver soc_codec_dev_cs47l94 = {
	.probe = cs47l94_codec_probe,
	.remove = cs47l94_codec_remove,
	.get_regmap = cs47l94_get_regmap,

	.idle_bias_off = true,

	.set_sysclk = tacna_set_sysclk,
	.set_pll = cs47l94_set_fll,

	.component_driver = {
		.controls = cs47l94_snd_controls,
		.num_controls = ARRAY_SIZE(cs47l94_snd_controls),
		.dapm_widgets = cs47l94_dapm_widgets,
		.num_dapm_widgets = ARRAY_SIZE(cs47l94_dapm_widgets),
		.dapm_routes = cs47l94_dapm_routes,
		.num_dapm_routes = ARRAY_SIZE(cs47l94_dapm_routes),
	},
};

static int cs47l94_probe(struct platform_device *pdev)
{
	struct tacna *tacna = dev_get_drvdata(pdev->dev.parent);
	struct cs47l94 *cs47l94;
	struct wm_adsp *dsp;
	int i, ret;

	BUILD_BUG_ON(ARRAY_SIZE(cs47l94_dai) > TACNA_MAX_DAI);

	/* quick exit if tacna irqchip driver hasn't completed probe */
	if (!tacna->irq_dev) {
		dev_dbg(&pdev->dev, "irqchip driver not ready\n");
		return -EPROBE_DEFER;
	}

	cs47l94 = devm_kzalloc(&pdev->dev, sizeof(struct cs47l94), GFP_KERNEL);
	if (!cs47l94)
		return -ENOMEM;

	platform_set_drvdata(pdev, cs47l94);
	pdev->dev.of_node = tacna->dev->of_node;

	cs47l94->core.tacna = tacna;
	cs47l94->core.dev = &pdev->dev;
	cs47l94->core.num_inputs = 8;
	cs47l94->core.max_analogue_inputs = 2;
	cs47l94->core.num_dmic_clksrc = 4;
	cs47l94->core.dsp_power_regs[0] = cs47l94_dsp1_sram_power_regs;
	cs47l94->core.dsp_power_regs[1] = cs47l94_dsp2_sram_power_regs;

	ret = tacna_core_init(&cs47l94->core);
	if (ret)
		return ret;

	init_completion(&cs47l94->outh_enabled);
	init_completion(&cs47l94->outh_disabled);

	ret = tacna_request_irq(tacna, TACNA_IRQ_OUTHL_ENABLE_DONE,
				"OUTH enable", cs47l94_outh_enable, cs47l94);
	if (ret)
		dev_warn(&pdev->dev, "Failed to get OUTH enable IRQ: %d\n",
			 ret);

	ret = tacna_request_irq(tacna, TACNA_IRQ_OUTHL_DISABLE_DONE,
				"OUTH enable", cs47l94_outh_disable, cs47l94);
	if (ret)
		dev_warn(&pdev->dev, "Failed to get OUTH disable IRQ: %d\n",
			 ret);

	BUILD_BUG_ON(ARRAY_SIZE(tacna->dsp_regmap) < CS47L94_NUM_DSP);

	for (i = 0; i < CS47L94_NUM_DSP; ++i) {
		dsp = &cs47l94->core.dsp[i];
		dsp->part = "cs47l94";
		dsp->num = i + 1;
		dsp->type = WMFW_HALO;
		dsp->rev = 0;
		dsp->dev = tacna->dev;
		dsp->regmap = tacna->dsp_regmap[i];

		dsp->base = cs47l94_dsp_control_bases[i];
		dsp->base_sysinfo = cs47l94_dsp_sysinfo_bases[i];
		dsp->mem = cs47l94_dsp_regions[i];
		dsp->num_mems = ARRAY_SIZE(cs47l94_dsp1_regions);

		dsp->n_rx_channels = CS47L94_DSP_N_RX_CHANNELS;
		dsp->n_tx_channels = CS47L94_DSP_N_TX_CHANNELS;

		ret = wm_halo_init(dsp);
		if (ret != 0) {
			for (--i; i >= 0; --i)
				wm_adsp2_remove(dsp);
			goto error_core;
		}
	}

	ret = tacna_request_irq(tacna, TACNA_IRQ_DSP1_MPU_ERR,
				"DSP1 MPU", cs47l94_mpu_fault_irq,
				&cs47l94->core.dsp[0]);
	if (ret) {
		dev_warn(&pdev->dev, "Failed to get DSP1 MPU IRQ: %d\n", ret);
		goto error_dsp;
	}

	ret = tacna_request_irq(tacna, TACNA_IRQ_DSP2_MPU_ERR,
				"DSP2 MPU", cs47l94_mpu_fault_irq,
				&cs47l94->core.dsp[1]);
	if (ret) {
		dev_warn(&pdev->dev, "Failed to get DSP2 MPU IRQ: %d\n", ret);
		goto error_mpu_irq1;
	}

	for (i = 0; i < CS47L94_N_FLL; ++i)
		tacna_init_fll(tacna,
			       i + 1,
			       TACNA_FLL1_CONTROL1 + i * 0x100,
			       TACNA_IRQ1_STS6,
			       TACNA_FLL1_LOCK_STS1_MASK << (2 * i),
			       &cs47l94->fll[i]);

	for (i = 0; i < ARRAY_SIZE(cs47l94_dai); i++)
		tacna_init_dai(&cs47l94->core, i);

	/* Latch volume update bits */
	for (i = 0; i < ARRAY_SIZE(cs47l94_in_vu); i++)
		regmap_update_bits(tacna->regmap, cs47l94_in_vu[i],
				   TACNA_IN_VU_MASK, TACNA_IN_VU);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);

	ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_cs47l94,
				     cs47l94_dai, ARRAY_SIZE(cs47l94_dai));
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register codec: %d\n", ret);
		snd_soc_unregister_platform(&pdev->dev);
		goto error_mpu_irq2;
	}

	return ret;

error_mpu_irq2:
	tacna_free_irq(tacna, TACNA_IRQ_DSP2_MPU_ERR, &cs47l94->core.dsp[1]);
error_mpu_irq1:
	tacna_free_irq(tacna, TACNA_IRQ_DSP1_MPU_ERR, &cs47l94->core.dsp[0]);
error_dsp:
	for (i = 0; i < CS47L94_NUM_DSP; ++i)
		wm_adsp2_remove(&cs47l94->core.dsp[i]);

error_core:
	tacna_core_destroy(&cs47l94->core);

	tacna_free_irq(tacna, TACNA_IRQ_OUTHL_ENABLE_DONE, cs47l94);
	tacna_free_irq(tacna, TACNA_IRQ_OUTHL_DISABLE_DONE, cs47l94);

	return ret;
}

static int cs47l94_remove(struct platform_device *pdev)
{
	struct cs47l94 *cs47l94 = platform_get_drvdata(pdev);
	struct tacna *tacna = cs47l94->core.tacna;
	int i;

	snd_soc_unregister_platform(&pdev->dev);
	snd_soc_unregister_codec(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	tacna_free_irq(tacna, TACNA_IRQ_DSP2_MPU_ERR, &cs47l94->core.dsp[1]);
	tacna_free_irq(tacna, TACNA_IRQ_DSP1_MPU_ERR, &cs47l94->core.dsp[0]);

	tacna_free_irq(tacna, TACNA_IRQ_OUTHL_ENABLE_DONE, cs47l94);
	tacna_free_irq(tacna, TACNA_IRQ_OUTHL_DISABLE_DONE, cs47l94);

	for (i = 0; i < CS47L94_NUM_DSP; ++i) {
		wm_adsp2_remove(&cs47l94->core.dsp[i]);
	}

	tacna_core_destroy(&cs47l94->core);

	return 0;
}

static struct platform_driver cs47l94_codec_driver = {
	.driver = {
		.name = "cs47l94-codec",
		.owner = THIS_MODULE,
	},
	.probe = cs47l94_probe,
	.remove = cs47l94_remove,
};

module_platform_driver(cs47l94_codec_driver);

MODULE_DESCRIPTION("ASoC CS47L94 driver");
MODULE_AUTHOR("Piotr Stankiewicz <piotrs@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cs47l94-codec");
