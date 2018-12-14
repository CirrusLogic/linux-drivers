/*
 * cs47l96-ao.c  --  ALSA SoC AO Audio driver for CS47L96/CS47L97 codecs
 *
 * Copyright 2018 Cirrus Logic, Inc.
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

#define CS47L96_AO_NUM_DSP 1
#define CS47L96_AO_DSP_N_RX_CHANNELS 8
#define CS47L96_AO_DSP_N_TX_CHANNELS 8
#define TACNA_NUM_MIXER_AO_INPUTS 30

#define TACNA_AO_MUX_ENUM_DECL(name, reg) \
	SOC_VALUE_ENUM_SINGLE_AUTODISABLE_DECL( \
		name, reg, 0, TACNA_MIXER_SRC_MASK, \
		cs47l96_ao_mixer_texts, cs47l96_ao_mixer_values)

#define TACNA_AO_MUX_ENUMS(name, base_reg) \
	static TACNA_AO_MUX_ENUM_DECL(name##_enum, base_reg);	\
	static TACNA_MUX_CTL_DECL(name)

#define TACNA_DSPAO_ROUTES_1_8(name)				\
	{ name, NULL, name " Preloader" },		\
	{ name " Preloader", NULL, "SYSCLKAO" },	\
	{ name " Preload", NULL, name " Preloader" },	\
	TACNA_MUX_ROUTES(name, name "RX1"),		\
	TACNA_MUX_ROUTES(name, name "RX2"),		\
	TACNA_MUX_ROUTES(name, name "RX3"),		\
	TACNA_MUX_ROUTES(name, name "RX4"),		\
	TACNA_MUX_ROUTES(name, name "RX5"),		\
	TACNA_MUX_ROUTES(name, name "RX6"),		\
	TACNA_MUX_ROUTES(name, name "RX7"),		\
	TACNA_MUX_ROUTES(name, name "RX8")		\

struct cs47l96_ao {
	struct tacna_priv core;
	struct tacna_fll fll;
};

static const struct wm_adsp_region cs47l96_dsp1ao_regions[] = {
	{ .type = WMFW_HALO_PM_PACKED, .base = 0x5800000 },
	{ .type = WMFW_HALO_XM_PACKED, .base = 0x4000000 },
	{ .type = WMFW_ADSP2_XM, .base = 0x4800000 },
	{ .type = WMFW_HALO_YM_PACKED, .base = 0x4C00000 },
	{ .type = WMFW_ADSP2_YM, .base = 0x5400000 },
};

static const struct soc_enum cs47l96_ao_sample_rate[] = {
	SOC_VALUE_ENUM_SINGLE(TACNA_AO_SAMPLE_RATE1,
			      TACNA_AO_SAMPLE_RATE_1_SHIFT,
			      TACNA_AO_SAMPLE_RATE_1_MASK >>
			      TACNA_AO_SAMPLE_RATE_1_SHIFT,
			      TACNA_SAMPLE_RATE_ENUM_SIZE,
			      tacna_sample_rate_text,
			      tacna_sample_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_AO_SAMPLE_RATE2,
			      TACNA_AO_SAMPLE_RATE_2_SHIFT,
			      TACNA_AO_SAMPLE_RATE_2_MASK >>
			      TACNA_AO_SAMPLE_RATE_2_SHIFT,
			      TACNA_SAMPLE_RATE_ENUM_SIZE,
			      tacna_sample_rate_text,
			      tacna_sample_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_AO_SAMPLE_RATE3,
			      TACNA_AO_SAMPLE_RATE_3_SHIFT,
			      TACNA_AO_SAMPLE_RATE_3_MASK >>
			      TACNA_AO_SAMPLE_RATE_3_SHIFT,
			      TACNA_SAMPLE_RATE_ENUM_SIZE,
			      tacna_sample_rate_text,
			      tacna_sample_rate_val),
};

static int cs47l96_ao_in_rate_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int reg, shift;
	int ret = 0;

	snd_soc_dapm_mutex_lock(dapm);

	/* Cannot change rate on an active input */
	reg = snd_soc_read(codec, TACNA_AO_INPUT_CONTROL);
	shift = (e->reg - TACNA_IN5LAO_CONTROL1) / 0x20;
	shift ^= 0x1; /* Flip bottom bit for channel order */

	if ((reg) & (1 << shift)) {
		ret = -EBUSY;
		goto exit;
	}

	ret = snd_soc_put_enum_double(kcontrol, ucontrol);
exit:
	snd_soc_dapm_mutex_unlock(dapm);
	return ret;
}

static SOC_ENUM_SINGLE_DECL(cs47l96_ao_in_vd_ramp,
			    TACNA_AO_INPUT_VOL_CONTROL,
			    TACNA_AO_IN_VD_RAMP_SHIFT,
			    tacna_vol_ramp_text);

static SOC_ENUM_SINGLE_DECL(cs47l96_ao_in_vi_ramp,
			    TACNA_AO_INPUT_VOL_CONTROL,
			    TACNA_AO_IN_VI_RAMP_SHIFT,
			    tacna_vol_ramp_text);

static SOC_ENUM_SINGLE_DECL(cs47l96_ao_in_hpf_cut_enum,
			    TACNA_AO_INPUT_HPF_CONTROL,
			    TACNA_AO_IN_HPF_CUT_SHIFT,
			    tacna_in_hpf_cut_text);

static const struct soc_enum cs47l96_ao_in_dmic_osr[] = {
	SOC_ENUM_SINGLE(TACNA_INPUT5AO_CONTROL1,
			TACNA_IN5AO_OSR_SHIFT,
			TACNA_OSR_ENUM_SIZE,
			tacna_in_dmic_osr_text),
	SOC_ENUM_SINGLE(TACNA_INPUT6AO_CONTROL1,
			TACNA_IN6AO_OSR_SHIFT,
			TACNA_OSR_ENUM_SIZE,
			tacna_in_dmic_osr_text),
	SOC_ENUM_SINGLE(TACNA_INPUT7AO_CONTROL1,
			TACNA_IN7AO_OSR_SHIFT,
			TACNA_OSR_ENUM_SIZE,
			tacna_in_dmic_osr_text),
};

static const struct soc_enum cs47l96_ao_input_rate[] = {
	SOC_VALUE_ENUM_SINGLE(TACNA_IN5LAO_CONTROL1,
			      TACNA_IN5LAO_RATE_SHIFT,
			      TACNA_IN5LAO_RATE_MASK >> TACNA_IN5LAO_RATE_SHIFT,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text,
			      tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_IN5RAO_CONTROL1,
			      TACNA_IN5RAO_RATE_SHIFT,
			      TACNA_IN5RAO_RATE_MASK >> TACNA_IN5RAO_RATE_SHIFT,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text,
			      tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_IN6LAO_CONTROL1,
			      TACNA_IN6LAO_RATE_SHIFT,
			      TACNA_IN6LAO_RATE_MASK >> TACNA_IN6LAO_RATE_SHIFT,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text,
			      tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_IN6RAO_CONTROL1,
			      TACNA_IN6RAO_RATE_SHIFT,
			      TACNA_IN6RAO_RATE_MASK >> TACNA_IN6RAO_RATE_SHIFT,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text,
			      tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_IN7LAO_CONTROL1,
			      TACNA_IN7LAO_RATE_SHIFT,
			      TACNA_IN7LAO_RATE_MASK >> TACNA_IN7LAO_RATE_SHIFT,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text,
			      tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_IN7RAO_CONTROL1,
			      TACNA_IN7RAO_RATE_SHIFT,
			      TACNA_IN7RAO_RATE_MASK >> TACNA_IN7RAO_RATE_SHIFT,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text,
			      tacna_ao_rate_val),
};

static const struct snd_kcontrol_new cs47l96_ao_snd_controls[] = {
SOC_ENUM("IN5AO OSR", cs47l96_ao_in_dmic_osr[0]),
SOC_ENUM("IN6AO OSR", cs47l96_ao_in_dmic_osr[1]),
SOC_ENUM("IN7AO OSR", cs47l96_ao_in_dmic_osr[2]),

SOC_ENUM("AO IN HPF Cutoff Frequency", cs47l96_ao_in_hpf_cut_enum),

SOC_SINGLE("IN5LAO HPF Switch", TACNA_IN5LAO_CONTROL1, TACNA_IN5LAO_HPF_SHIFT,
	   1, 0),
SOC_SINGLE("IN5RAO HPF Switch", TACNA_IN5RAO_CONTROL1, TACNA_IN5RAO_HPF_SHIFT,
	   1, 0),
SOC_SINGLE("IN6LAO HPF Switch", TACNA_IN6LAO_CONTROL1, TACNA_IN6LAO_HPF_SHIFT,
	   1, 0),
SOC_SINGLE("IN6RAO HPF Switch", TACNA_IN6RAO_CONTROL1, TACNA_IN6RAO_HPF_SHIFT,
	   1, 0),
SOC_SINGLE("IN7LAO HPF Switch", TACNA_IN7LAO_CONTROL1, TACNA_IN7LAO_HPF_SHIFT,
	   1, 0),
SOC_SINGLE("IN7RAO HPF Switch", TACNA_IN7RAO_CONTROL1, TACNA_IN7RAO_HPF_SHIFT,
	   1, 0),

SOC_SINGLE_EXT_TLV("IN5LAO Digital Volume", TACNA_IN5L_CONTROL2,
		   TACNA_IN5LAO_VOL_SHIFT, 0xbf, 0,
		   snd_soc_get_volsw, tacna_in_put_volsw,
		   tacna_digital_tlv),
SOC_SINGLE_EXT_TLV("IN5RAO Digital Volume", TACNA_IN5R_CONTROL2,
		   TACNA_IN5RAO_VOL_SHIFT, 0xbf, 0,
		   snd_soc_get_volsw, tacna_in_put_volsw,
		   tacna_digital_tlv),
SOC_SINGLE_EXT_TLV("IN6LAO Digital Volume", TACNA_IN6L_CONTROL2,
		   TACNA_IN6LAO_VOL_SHIFT, 0xbf, 0,
		   snd_soc_get_volsw, tacna_in_put_volsw,
		   tacna_digital_tlv),
SOC_SINGLE_EXT_TLV("IN6RAO Digital Volume", TACNA_IN6R_CONTROL2,
		   TACNA_IN6RAO_VOL_SHIFT, 0xbf, 0,
		   snd_soc_get_volsw, tacna_in_put_volsw,
		   tacna_digital_tlv),
SOC_SINGLE_EXT_TLV("IN7LAO Digital Volume", TACNA_IN7L_CONTROL2,
		   TACNA_IN7LAO_VOL_SHIFT, 0xbf, 0,
		   snd_soc_get_volsw, tacna_in_put_volsw,
		   tacna_digital_tlv),
SOC_SINGLE_EXT_TLV("IN7RAO Digital Volume", TACNA_IN7R_CONTROL2,
		   TACNA_IN7RAO_VOL_SHIFT, 0xbf, 0,
		   snd_soc_get_volsw, tacna_in_put_volsw,
		   tacna_digital_tlv),

SOC_ENUM("AO Input Ramp Up", cs47l96_ao_in_vi_ramp),
SOC_ENUM("AO Input Ramp Down", cs47l96_ao_in_vd_ramp),

SOC_ENUM("AO Sample Rate 1", cs47l96_ao_sample_rate[0]),
SOC_ENUM("AO Sample Rate 2", cs47l96_ao_sample_rate[1]),
SOC_ENUM("AO Sample Rate 3", cs47l96_ao_sample_rate[2]),

SOC_ENUM_EXT("IN5LAO Rate", cs47l96_ao_input_rate[0],
	     snd_soc_get_enum_double, cs47l96_ao_in_rate_put),
SOC_ENUM_EXT("IN5RAO Rate", cs47l96_ao_input_rate[1],
	     snd_soc_get_enum_double, cs47l96_ao_in_rate_put),
SOC_ENUM_EXT("IN6LAO Rate", cs47l96_ao_input_rate[2],
	     snd_soc_get_enum_double, cs47l96_ao_in_rate_put),
SOC_ENUM_EXT("IN6RAO Rate", cs47l96_ao_input_rate[3],
	     snd_soc_get_enum_double, cs47l96_ao_in_rate_put),
SOC_ENUM_EXT("IN7LAO Rate", cs47l96_ao_input_rate[4],
	     snd_soc_get_enum_double, cs47l96_ao_in_rate_put),
SOC_ENUM_EXT("IN7RAO Rate", cs47l96_ao_input_rate[5],
	     snd_soc_get_enum_double, cs47l96_ao_in_rate_put),

WM_ADSP2_PRELOAD_SWITCH("DSP1AO", 1),
WM_ADSP_FW_CONTROL("DSP1AO", 0),
};

static const char * const cs47l96_ao_mixer_texts[] = {
	"None",
	"ASP1AORX1",
	"ASP1AORX2",
	"ASP1AORX3",
	"ASP1AORX4",
	"DSP1AO.1",
	"DSP1AO.2",
	"DSP1AO.3",
	"DSP1AO.4",
	"DSP1AO.5",
	"DSP1AO.6",
	"DSP1AO.7",
	"DSP1AO.8",
	"Ultrasonic 1",
	"Ultrasonic 2",
	"Ultrasonic 3",
	"IN5LAO",
	"IN5RAO",
	"IN6LAO",
	"IN6RAO",
	"IN7LAO",
	"IN7RAO",
	"AOBRIDGE2IN1",
	"AOBRIDGE2IN2",
	"AOBRIDGE2IN3",
	"AOBRIDGE2IN4",
	"AOBRIDGE2IN5",
	"AOBRIDGE2IN6",
	"AOBRIDGE2IN7",
	"AOBRIDGE2IN8",
};

static unsigned int cs47l96_ao_mixer_values[] = {
	0x000, /* Silence (mute) */
	0x001, /* ASP_AO1 RX1 */
	0x002, /* ASP_AO1 RX2 */
	0x003, /* ASP_AO1 RX3 */
	0x004, /* ASP_AO1 RX4 */
	0x005, /* DSP1AO channel 1 */
	0x006, /* DSP1AO channel 2 */
	0x007, /* DSP1AO channel 3 */
	0x008, /* DSP1AO channel 4 */
	0x009, /* DSP1AO channel 5 */
	0x00A, /* DSP1AO channel 6 */
	0x00B, /* DSP1AO channel 7 */
	0x00C, /* DSP1AO channel 8 */
	0x00D, /* Ultrasonic 1 */
	0x00E, /* Ultrasonic 2 */
	0x00F, /* Ultrasonic 3 */
	0x010, /* IN5LAO signal path */
	0x011, /* IN5RAO signal path */
	0x012, /* IN6LAO signal path */
	0x013, /* IN6RAO signal path */
	0x014, /* IN7LAO signal path */
	0x015, /* IN7RAO signal path */
	0x016, /* AO Bridge 2 channel 1 */
	0x017, /* AO Bridge 2 channel 2 */
	0x018, /* AO Bridge 2 channel 3 */
	0x019, /* AO Bridge 2 channel 4 */
	0x01A, /* AO Bridge 2 channel 5 */
	0x01B, /* AO Bridge 2 channel 6 */
	0x01C, /* AO Bridge 2 channel 7 */
	0x01D, /* AO Bridge 2 channel 8 */
};

TACNA_AO_MUX_ENUMS(ASP1AOTX1, TACNA_ASP1AOTX1_INPUT1);
TACNA_AO_MUX_ENUMS(ASP1AOTX2, TACNA_ASP1AOTX2_INPUT1);
TACNA_AO_MUX_ENUMS(ASP1AOTX3, TACNA_ASP1AOTX3_INPUT1);
TACNA_AO_MUX_ENUMS(ASP1AOTX4, TACNA_ASP1AOTX4_INPUT1);

TACNA_AO_MUX_ENUMS(DSP1AORX1, TACNA_DSP1AORX1_INPUT1);
TACNA_AO_MUX_ENUMS(DSP1AORX2, TACNA_DSP1AORX2_INPUT1);
TACNA_AO_MUX_ENUMS(DSP1AORX3, TACNA_DSP1AORX3_INPUT1);
TACNA_AO_MUX_ENUMS(DSP1AORX4, TACNA_DSP1AORX4_INPUT1);
TACNA_AO_MUX_ENUMS(DSP1AORX5, TACNA_DSP1AORX5_INPUT1);
TACNA_AO_MUX_ENUMS(DSP1AORX6, TACNA_DSP1AORX6_INPUT1);
TACNA_AO_MUX_ENUMS(DSP1AORX7, TACNA_DSP1AORX7_INPUT1);
TACNA_AO_MUX_ENUMS(DSP1AORX8, TACNA_DSP1AORX8_INPUT1);

TACNA_AO_MUX_ENUMS(AOBRIDGE1IN1, TACNA_AOBRIDGE1_IN1_INPUT1);
TACNA_AO_MUX_ENUMS(AOBRIDGE1IN2, TACNA_AOBRIDGE1_IN2_INPUT1);
TACNA_AO_MUX_ENUMS(AOBRIDGE1IN3, TACNA_AOBRIDGE1_IN3_INPUT1);
TACNA_AO_MUX_ENUMS(AOBRIDGE1IN4, TACNA_AOBRIDGE1_IN4_INPUT1);
TACNA_AO_MUX_ENUMS(AOBRIDGE1IN5, TACNA_AOBRIDGE1_IN5_INPUT1);
TACNA_AO_MUX_ENUMS(AOBRIDGE1IN6, TACNA_AOBRIDGE1_IN6_INPUT1);
TACNA_AO_MUX_ENUMS(AOBRIDGE1IN7, TACNA_AOBRIDGE1_IN7_INPUT1);
TACNA_AO_MUX_ENUMS(AOBRIDGE1IN8, TACNA_AOBRIDGE1_IN8_INPUT1);

static int cs47l96_ao_in_ev(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol,
			    int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	unsigned int reg;

	if (w->shift % 2)
		reg = TACNA_IN1LAO_CONTROL2 + ((w->shift / 2) * 0x40);
	else
		reg = TACNA_IN1RAO_CONTROL2 + ((w->shift / 2) * 0x40);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		priv->in_up_pending++;
		break;
	case SND_SOC_DAPM_POST_PMU:
		priv->in_up_pending--;
		snd_soc_update_bits(codec, reg, TACNA_IN1LAO_MUTE, 0);

		/* Uncached write-only register, no need for update_bits */
		if (!priv->in_up_pending)
			snd_soc_write(codec, priv->in_vu_reg, TACNA_IN_VU);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, reg,
				    TACNA_IN1LAO_MUTE, TACNA_IN1LAO_MUTE);
		snd_soc_write(codec, priv->in_vu_reg, TACNA_IN_VU);
		break;
	default:
		break;
	}

	return 0;
}

static const struct snd_kcontrol_new cs47l96_ao_dspao_trigger_output_mux[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
};

static int cs47l96_ao_dsp_freq_ev(struct snd_soc_dapm_widget *w,
				  struct snd_kcontrol *kcontrol, int event)
{
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		return tacna_dsp_freq_update(w, TACNA_SYSTEM_CLOCK2AO,
					     TACNA_SYSTEM_CLOCK1AO);
	default:
		return 0;
	}
}

static const struct snd_soc_dapm_widget cs47l96_ao_dapm_widgets[] = {
SND_SOC_DAPM_SUPPLY("SYSCLKAO", TACNA_SYSTEM_CLOCK1AO, TACNA_SYSCLKAO_EN_SHIFT,
		    0, NULL, 0),

TACNA_DSP_FREQ_WIDGET_EV("DSP1AO", 0, cs47l96_ao_dsp_freq_ev),

SND_SOC_DAPM_INPUT("IN5AO_PDMCLK"),
SND_SOC_DAPM_INPUT("IN5AO_PDMDATA"),

SND_SOC_DAPM_INPUT("IN6AO_PDMCLK"),
SND_SOC_DAPM_INPUT("IN6AO_PDMDATA"),

SND_SOC_DAPM_INPUT("IN7AO_PDMCLK"),
SND_SOC_DAPM_INPUT("IN7AO_PDMDATA"),

SND_SOC_DAPM_OUTPUT("DSPAO Trigger Out"),

SND_SOC_DAPM_AIF_OUT("ASP1AOTX1", NULL, 0, TACNA_ASP1AO_ENABLES1,
		     TACNA_ASP1AO_TX1_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP1AOTX2", NULL, 0, TACNA_ASP1AO_ENABLES1,
		     TACNA_ASP1AO_TX2_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP1AOTX3", NULL, 0, TACNA_ASP1AO_ENABLES1,
		     TACNA_ASP1AO_TX3_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_OUT("ASP1AOTX4", NULL, 0, TACNA_ASP1AO_ENABLES1,
		     TACNA_ASP1AO_TX4_EN_SHIFT, 0),

/* mux_in widgets : arranged in the order of sources
 * specified in TACNA_MIXER_INPUT_ROUTES
 */

SND_SOC_DAPM_PGA_E("IN5LAO PGA", TACNA_AO_INPUT_CONTROL, TACNA_IN5LAO_EN_SHIFT,
		   0, NULL, 0, cs47l96_ao_in_ev,
		   SND_SOC_DAPM_PRE_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("IN5RAO PGA", TACNA_AO_INPUT_CONTROL, TACNA_IN5RAO_EN_SHIFT,
		   0, NULL, 0, cs47l96_ao_in_ev,
		   SND_SOC_DAPM_PRE_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("IN6LAO PGA", TACNA_AO_INPUT_CONTROL, TACNA_IN6LAO_EN_SHIFT,
		   0, NULL, 0, cs47l96_ao_in_ev,
		   SND_SOC_DAPM_PRE_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("IN6RAO PGA", TACNA_AO_INPUT_CONTROL, TACNA_IN6RAO_EN_SHIFT,
		   0, NULL, 0, cs47l96_ao_in_ev,
		   SND_SOC_DAPM_PRE_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("IN7LAO PGA", TACNA_AO_INPUT_CONTROL, TACNA_IN7LAO_EN_SHIFT,
		   0, NULL, 0, cs47l96_ao_in_ev,
		   SND_SOC_DAPM_PRE_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("IN7RAO PGA", TACNA_AO_INPUT_CONTROL, TACNA_IN7RAO_EN_SHIFT,
		   0, NULL, 0, cs47l96_ao_in_ev,
		   SND_SOC_DAPM_PRE_PMD |
		   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU),

SND_SOC_DAPM_AIF_IN("ASP1AORX1", NULL, 0, TACNA_ASP1AO_ENABLES1,
		    TACNA_ASP1AO_RX1_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP1AORX2", NULL, 0, TACNA_ASP1AO_ENABLES1,
		    TACNA_ASP1AO_RX2_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP1AORX3", NULL, 0, TACNA_ASP1AO_ENABLES1,
		    TACNA_ASP1AO_RX3_EN_SHIFT, 0),
SND_SOC_DAPM_AIF_IN("ASP1AORX4", NULL, 0, TACNA_ASP1AO_ENABLES1,
		    TACNA_ASP1AO_RX4_EN_SHIFT, 0),

WM_HALO("DSP1AO", 0, wm_halo_early_event),

/* end of ordered widget list */

TACNA_MUX_WIDGETS(ASP1AOTX1, "ASP1AOTX1"),
TACNA_MUX_WIDGETS(ASP1AOTX2, "ASP1AOTX2"),
TACNA_MUX_WIDGETS(ASP1AOTX3, "ASP1AOTX3"),
TACNA_MUX_WIDGETS(ASP1AOTX4, "ASP1AOTX4"),

TACNA_MUX_WIDGETS(DSP1AORX1, "DSP1AORX1"),
TACNA_MUX_WIDGETS(DSP1AORX2, "DSP1AORX2"),
TACNA_MUX_WIDGETS(DSP1AORX3, "DSP1AORX3"),
TACNA_MUX_WIDGETS(DSP1AORX4, "DSP1AORX4"),
TACNA_MUX_WIDGETS(DSP1AORX5, "DSP1AORX5"),
TACNA_MUX_WIDGETS(DSP1AORX6, "DSP1AORX6"),
TACNA_MUX_WIDGETS(DSP1AORX7, "DSP1AORX7"),
TACNA_MUX_WIDGETS(DSP1AORX8, "DSP1AORX8"),

TACNA_MUX_WIDGETS(AOBRIDGE1IN1, "AOBRIDGE1IN1"),
TACNA_MUX_WIDGETS(AOBRIDGE1IN2, "AOBRIDGE1IN2"),
TACNA_MUX_WIDGETS(AOBRIDGE1IN3, "AOBRIDGE1IN3"),
TACNA_MUX_WIDGETS(AOBRIDGE1IN4, "AOBRIDGE1IN4"),
TACNA_MUX_WIDGETS(AOBRIDGE1IN5, "AOBRIDGE1IN5"),
TACNA_MUX_WIDGETS(AOBRIDGE1IN6, "AOBRIDGE1IN6"),
TACNA_MUX_WIDGETS(AOBRIDGE1IN7, "AOBRIDGE1IN7"),
TACNA_MUX_WIDGETS(AOBRIDGE1IN8, "AOBRIDGE1IN8"),

SND_SOC_DAPM_SWITCH("DSP1AO Trigger Output", SND_SOC_NOPM, 0, 0,
		    &cs47l96_ao_dspao_trigger_output_mux[0]),
};

#define TACNA_MIXER_INPUT_ROUTES(name) \
	{ name, "IN5LAO", "IN5LAO PGA" }, \
	{ name, "IN5RAO", "IN5RAO PGA" }, \
	{ name, "IN6LAO", "IN6LAO PGA" }, \
	{ name, "IN6RAO", "IN6RAO PGA" }, \
	{ name, "IN7LAO", "IN7LAO PGA" }, \
	{ name, "IN7RAO", "IN7RAO PGA" }, \
	{ name, "ASP1AORX1", "ASP1AORX1" }, \
	{ name, "ASP1AORX2", "ASP1AORX2" }, \
	{ name, "ASP1AORX3", "ASP1AORX3" }, \
	{ name, "ASP1AORX4", "ASP1AORX4" }, \
	{ name, "AOBRIDGE2IN1", "AOBRIDGE2IN1" }, \
	{ name, "AOBRIDGE2IN2", "AOBRIDGE2IN2" }, \
	{ name, "AOBRIDGE2IN3", "AOBRIDGE2IN3" }, \
	{ name, "AOBRIDGE2IN4", "AOBRIDGE2IN4" }, \
	{ name, "AOBRIDGE2IN5", "AOBRIDGE2IN5" }, \
	{ name, "AOBRIDGE2IN6", "AOBRIDGE2IN6" }, \
	{ name, "AOBRIDGE2IN7", "AOBRIDGE2IN7" }, \
	{ name, "AOBRIDGE2IN8", "AOBRIDGE2IN8" }, \
	{ name, "DSP1AO.1", "DSP1AO" }, \
	{ name, "DSP1AO.2", "DSP1AO" }, \
	{ name, "DSP1AO.3", "DSP1AO" }, \
	{ name, "DSP1AO.4", "DSP1AO" }, \
	{ name, "DSP1AO.5", "DSP1AO" }, \
	{ name, "DSP1AO.6", "DSP1AO" }, \
	{ name, "DSP1AO.7", "DSP1AO" }, \
	{ name, "DSP1AO.8", "DSP1AO" }

static const struct snd_soc_dapm_route cs47l96_ao_dapm_routes[] = {
	/* Internal clock domains */
	{ "DSP1AO", NULL, "SYSCLKAO" },
	{ "DSP1AO", NULL, "DSP1AOFREQ" },

	{ "IN5AO_PDMCLK", NULL, "SYSCLKAO" },
	{ "IN5AO_PDMDATA", NULL, "SYSCLKAO" },
	{ "IN6AO_PDMCLK", NULL, "SYSCLKAO" },
	{ "IN6AO_PDMDATA", NULL, "SYSCLKAO" },
	{ "IN7AO_PDMCLK", NULL, "SYSCLKAO" },
	{ "IN7AO_PDMDATA", NULL, "SYSCLKAO" },

	{ "Audio Trace DSP", NULL, "DSP1AO" },
	{ "Voice Ctrl DSP", NULL, "DSP1AO" },

	{ "ASP1AO Capture", NULL, "ASP1AOTX1" },
	{ "ASP1AO Capture", NULL, "ASP1AOTX2" },
	{ "ASP1AO Capture", NULL, "ASP1AOTX3" },
	{ "ASP1AO Capture", NULL, "ASP1AOTX4" },

	{ "ASP1AORX1", NULL, "ASP1AO Playback" },
	{ "ASP1AORX2", NULL, "ASP1AO Playback" },
	{ "ASP1AORX3", NULL, "ASP1AO Playback" },
	{ "ASP1AORX4", NULL, "ASP1AO Playback" },

	{ "ASP1AO Playback", NULL, "SYSCLKAO" },

	{ "ASP1AO Capture", NULL, "SYSCLKAO" },

	{ "AOBRIDGE1IN1", NULL, "SYSCLKAO" },
	{ "AOBRIDGE1IN2", NULL, "SYSCLKAO" },
	{ "AOBRIDGE1IN3", NULL, "SYSCLKAO" },
	{ "AOBRIDGE1IN4", NULL, "SYSCLKAO" },
	{ "AOBRIDGE1IN5", NULL, "SYSCLKAO" },
	{ "AOBRIDGE1IN6", NULL, "SYSCLKAO" },
	{ "AOBRIDGE1IN7", NULL, "SYSCLKAO" },
	{ "AOBRIDGE1IN8", NULL, "SYSCLKAO" },

	{ "IN5LAO PGA", NULL, "IN5AO_PDMCLK" },
	{ "IN5LAO PGA", NULL, "IN5AO_PDMDATA" },
	{ "IN5RAO PGA", NULL, "IN5AO_PDMCLK" },
	{ "IN5RAO PGA", NULL, "IN5AO_PDMDATA" },

	{ "IN6LAO PGA", NULL, "IN6AO_PDMCLK" },
	{ "IN6LAO PGA", NULL, "IN6AO_PDMDATA" },
	{ "IN6RAO PGA", NULL, "IN6AO_PDMCLK" },
	{ "IN6RAO PGA", NULL, "IN6AO_PDMDATA" },

	{ "IN7LAO PGA", NULL, "IN7AO_PDMCLK" },
	{ "IN7LAO PGA", NULL, "IN7AO_PDMDATA" },
	{ "IN7RAO PGA", NULL, "IN7AO_PDMCLK" },
	{ "IN7RAO PGA", NULL, "IN7AO_PDMDATA" },

	TACNA_MUX_ROUTES("ASP1AOTX1", "ASP1AOTX1"),
	TACNA_MUX_ROUTES("ASP1AOTX2", "ASP1AOTX2"),
	TACNA_MUX_ROUTES("ASP1AOTX3", "ASP1AOTX3"),
	TACNA_MUX_ROUTES("ASP1AOTX4", "ASP1AOTX4"),

	TACNA_DSPAO_ROUTES_1_8("DSP1AO"),

	TACNA_MUX_ROUTES("AOBRIDGE1IN1", "AOBRIDGE1IN1"),
	TACNA_MUX_ROUTES("AOBRIDGE1IN2", "AOBRIDGE1IN2"),
	TACNA_MUX_ROUTES("AOBRIDGE1IN3", "AOBRIDGE1IN3"),
	TACNA_MUX_ROUTES("AOBRIDGE1IN4", "AOBRIDGE1IN4"),
	TACNA_MUX_ROUTES("AOBRIDGE1IN5", "AOBRIDGE1IN5"),
	TACNA_MUX_ROUTES("AOBRIDGE1IN6", "AOBRIDGE1IN6"),
	TACNA_MUX_ROUTES("AOBRIDGE1IN7", "AOBRIDGE1IN7"),
	TACNA_MUX_ROUTES("AOBRIDGE1IN8", "AOBRIDGE1IN8"),

	{ "DSPAO Trigger Out", NULL, "DSP1AO Trigger Output" },

	{ "DSP1AO Trigger Output", "Switch", "DSP1AO" },
};

static struct snd_soc_dai_driver cs47l96_ao_dai[] = {
	{
		.name = "cs47l96-ao-asp1",
		.id = 1,
		.base = TACNA_ASP1AO_ENABLES1,
		.playback = {
			.stream_name = "ASP1AO Playback",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
		.capture = {
			.stream_name = "ASP1AO Capture",
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
		.name = "cs47l96-ao-cpu-trace",
		.capture = {
			.stream_name = "Audio Trace CPU",
			.channels_min = 1,
			.channels_max = 6,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
		.compress_new = &snd_soc_new_compress,
	},
	{
		.name = "cs47l96-ao-dsp-trace",
		.capture = {
			.stream_name = "Audio Trace DSP",
			.channels_min = 1,
			.channels_max = 6,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
	},
	{
		.name = "cs47l96-ao-cpu-voicectrl",
		.capture = {
			.stream_name = "Voice Ctrl CPU",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
		.compress_new = &snd_soc_new_compress,
	},
	{
		.name = "cs47l96-ao-dsp-voicectrl",
		.capture = {
			.stream_name = "Voice Ctrl DSP",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
	},
};

static int cs47l96_ao_compr_open(struct snd_compr_stream *stream)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct cs47l96_ao *cs47l96_ao = snd_soc_platform_get_drvdata(rtd->platform);
	struct tacna_priv *priv = &cs47l96_ao->core;

	if (strcmp(rtd->codec_dai->name, "cs47l96-ao-dsp-trace") &&
	    strcmp(rtd->codec_dai->name, "cs47l96-ao-dsp-voicectrl")) {
		dev_err(priv->dev,
			"No suitable compressed stream for DAI '%s'\n",
			rtd->codec_dai->name);
		return -EINVAL;
	}

	return wm_adsp_compr_open(&priv->dsp[0], stream);
}

static irqreturn_t cs47l96_dsp1ao_irq0(int irq, void *data)
{
	struct cs47l96_ao *cs47l96_ao = data;
	struct tacna_priv *priv = &cs47l96_ao->core;
	int ret;

	ret = wm_adsp_compr_handle_irq(&priv->dsp[0]);
	if (ret == -ENODEV) {
		dev_err(priv->dev, "Spurious compressed data IRQ\n");
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static irqreturn_t cs47l96_ao_mpu_fault_irq(int irq, void *data)
{
	struct wm_adsp *dsp = data;

	return wm_halo_bus_error(dsp);
}

static const struct soc_enum cs47l96_ao_dsp1_rx_rate_enum[] = {
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      0 | TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text, tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      1 | TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text, tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      2 | TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text, tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      3 | TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text, tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      4 | TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text,  tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      5 | TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text, tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      6 | TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text, tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      7 | TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text, tacna_ao_rate_val),
};

static const struct soc_enum cs47l96_ao_dsp1_tx_rate_enum[] = {
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      0 | TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text, tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      1 | TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text, tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      2 | TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text, tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      3 | TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text, tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      4 | TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text, tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      5 | TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text, tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      6 | TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text, tacna_ao_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      7 | TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_AO_RATE_ENUM_SIZE,
			      tacna_ao_rate_text, tacna_ao_rate_val),
};

static const struct snd_kcontrol_new cs47l96_ao_dsp1ao_rate_controls[] = {
	SOC_ENUM_EXT("DSP1AORX1 Rate", cs47l96_ao_dsp1_rx_rate_enum[0],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1AORX2 Rate", cs47l96_ao_dsp1_rx_rate_enum[1],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1AORX3 Rate", cs47l96_ao_dsp1_rx_rate_enum[2],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1AORX4 Rate", cs47l96_ao_dsp1_rx_rate_enum[3],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1AORX5 Rate", cs47l96_ao_dsp1_rx_rate_enum[4],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1AORX6 Rate", cs47l96_ao_dsp1_rx_rate_enum[5],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1AORX7 Rate", cs47l96_ao_dsp1_rx_rate_enum[6],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1AORX8 Rate", cs47l96_ao_dsp1_rx_rate_enum[7],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1AOTX1 Rate", cs47l96_ao_dsp1_tx_rate_enum[0],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1AOTX2 Rate", cs47l96_ao_dsp1_tx_rate_enum[1],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1AOTX3 Rate", cs47l96_ao_dsp1_tx_rate_enum[2],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1AOTX4 Rate", cs47l96_ao_dsp1_tx_rate_enum[3],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1AOTX5 Rate", cs47l96_ao_dsp1_tx_rate_enum[4],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1AOTX6 Rate", cs47l96_ao_dsp1_tx_rate_enum[5],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1AOTX7 Rate", cs47l96_ao_dsp1_tx_rate_enum[6],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1AOTX8 Rate", cs47l96_ao_dsp1_tx_rate_enum[7],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
};

static int cs47l96_ao_codec_probe(struct snd_soc_codec *codec)
{
	struct cs47l96_ao *cs47l96_ao = snd_soc_codec_get_drvdata(codec);
	struct tacna_priv *priv = &cs47l96_ao->core;
	struct tacna *tacna = cs47l96_ao->core.tacna;
	int ret;

	BUILD_BUG_ON(ARRAY_SIZE(cs47l96_ao_mixer_texts) !=
		     TACNA_NUM_MIXER_AO_INPUTS);
	BUILD_BUG_ON(ARRAY_SIZE(cs47l96_ao_mixer_values) !=
		     TACNA_NUM_MIXER_AO_INPUTS);

	tacna->dapm = snd_soc_codec_get_dapm(codec);

	ret = snd_soc_add_codec_controls(codec,
					 cs47l96_ao_dsp1ao_rate_controls,
					 priv->dsp[0].n_rx_channels +
					 priv->dsp[0].n_tx_channels);
	if (ret)
		return ret;

	wm_adsp2_codec_probe(&cs47l96_ao->core.dsp[0], codec);

	return 0;
}

static int cs47l96_ao_codec_remove(struct snd_soc_codec *codec)
{
	struct cs47l96_ao *cs47l96_ao = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = cs47l96_ao->core.tacna;

	wm_adsp2_codec_remove(&cs47l96_ao->core.dsp[0], codec);

	tacna->dapm = NULL;

	return 0;
}

static int cs47l96_ao_set_fll(struct snd_soc_codec *codec, int fll_id, int source,
			   unsigned int fref, unsigned int fout)
{
	struct cs47l96_ao *cs47l96_ao = snd_soc_codec_get_drvdata(codec);

	switch (fll_id) {
	case TACNA_FLL1_REFCLK:
		break;
	default:
		return -EINVAL;
	}

	if (fout > 49152000) {
		dev_err(codec->dev, "%u not supported for FLL1AO\n", fout);
		return -EINVAL;
	}

	return tacna_fllhj_set_refclk(&cs47l96_ao->fll, source, fref, fout);
}

static int cs47l96_ao_set_sysclk(struct snd_soc_codec *codec, int clk_id,
				 int source, unsigned int freq, int dir)
{
	struct cs47l96_ao *cs47l96_ao = snd_soc_codec_get_drvdata(codec);

	switch (clk_id) {
	case TACNA_CLK_SYSCLKAO:
		if (freq > 49152000) {
			dev_err(codec->dev, "%u not supported for SYSCLKAO\n",
				freq);
			return -EINVAL;
		}
		return tacna_set_sysclk(codec, clk_id, source, freq, dir);
	default:
		dev_err(cs47l96_ao->core.dev, "Unknown clock id %u\n", clk_id);
		return -EINVAL;
	}
}

static struct regmap *cs47l96_ao_get_regmap(struct device *dev)
{
	struct cs47l96_ao *cs47l96_ao = dev_get_drvdata(dev);

	return cs47l96_ao->core.tacna->regmap;
}

static const struct snd_soc_codec_driver soc_codec_dev_cs47l96_ao = {
	.probe = &cs47l96_ao_codec_probe,
	.remove = &cs47l96_ao_codec_remove,
	.get_regmap = &cs47l96_ao_get_regmap,

	.idle_bias_off = true,

	.set_sysclk = &cs47l96_ao_set_sysclk,
	.set_pll = &cs47l96_ao_set_fll,

	.component_driver = {
		.controls = cs47l96_ao_snd_controls,
		.num_controls = ARRAY_SIZE(cs47l96_ao_snd_controls),
		.dapm_widgets = cs47l96_ao_dapm_widgets,
		.num_dapm_widgets = ARRAY_SIZE(cs47l96_ao_dapm_widgets),
		.dapm_routes = cs47l96_ao_dapm_routes,
		.num_dapm_routes = ARRAY_SIZE(cs47l96_ao_dapm_routes),
	},
};

static const struct snd_compr_ops cs47l96_ao_compr_ops = {
	.open = &cs47l96_ao_compr_open,
	.free = &wm_adsp_compr_free,
	.set_params = &wm_adsp_compr_set_params,
	.get_caps = &wm_adsp_compr_get_caps,
	.trigger = &wm_adsp_compr_trigger,
	.pointer = &wm_adsp_compr_pointer,
	.copy = &wm_adsp_compr_copy,
};

static const struct snd_soc_platform_driver cs47l96_ao_compr_platform = {
	.compr_ops = &cs47l96_ao_compr_ops,
};

static int cs47l96_ao_probe(struct platform_device *pdev)
{
	struct tacna *tacna = dev_get_drvdata(pdev->dev.parent);
	struct cs47l96_ao *cs47l96_ao;
	struct wm_adsp *dsp;
	int i, ret;

	BUILD_BUG_ON(ARRAY_SIZE(cs47l96_ao_dai) > TACNA_MAX_DAI);

	/* quick exit if tacna irqchip driver hasn't completed probe */
	if (!tacna->irq_dev) {
		dev_dbg(&pdev->dev, "irqchip driver not ready\n");
		return -EPROBE_DEFER;
	}

	/* Source CLK32KAO from MCLK2 */
	ret = regmap_update_bits(tacna->regmap, TACNA_CLOCK32KAO,
				 TACNA_CLK_32KAO_SRC_MASK,
				 TACNA_CLK_SRC_MCLK2);
	if (ret) {
		dev_err(tacna->dev, "Failed to init AO 32k clock: %d\n", ret);
		return ret;
	}

	/* Slave SYSCLKAO to SYSCLK */
	ret = regmap_update_bits(tacna->regmap, TACNA_SYSTEM_CLOCK6AO,
				 TACNA_SYSAO_FRAME_SLV_MASK,
				 1 << TACNA_SYSAO_FRAME_SLV_SHIFT);
	if (ret) {
		dev_err(tacna->dev, "Failed to init FRAME_SLV: %d\n", ret);
		return ret;
	}

	cs47l96_ao = devm_kzalloc(&pdev->dev, sizeof(struct cs47l96_ao),
				  GFP_KERNEL);
	if (!cs47l96_ao)
		return -ENOMEM;

	platform_set_drvdata(pdev, cs47l96_ao);

	cs47l96_ao->core.tacna = tacna;
	cs47l96_ao->core.dev = &pdev->dev;
	cs47l96_ao->core.num_inputs = 6;
	cs47l96_ao->core.max_analogue_inputs = 0;
	cs47l96_ao->core.in_vu_reg = TACNA_AO_INPUT_CONTROL3;

	mutex_init(&cs47l96_ao->core.rate_lock);
	mutex_init(&cs47l96_ao->core.dsp_fw_lock);

	/* DSP1AO uses the DSP2 interrupt registers */
	ret = tacna_request_irq(tacna, TACNA_IRQ_DSP2_IRQ0,
				"DSP1AO Buffer IRQ", cs47l96_dsp1ao_irq0,
				cs47l96_ao);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to request DSP2_IRQ0: %d\n", ret);
		return ret;
	}

	ret = tacna_set_irq_wake(tacna, TACNA_IRQ_DSP2_IRQ0, 1);
	if(ret)
		dev_warn(&pdev->dev, "Failed to set DSP IRQ wake: %d\n", ret);

	dsp = &cs47l96_ao->core.dsp[0];
	dsp->part = "cs47l96_ao";
	dsp->num = 1;
	dsp->type = WMFW_HALO;
	dsp->rev = 0;
	dsp->dev = tacna->dev;
	dsp->regmap = tacna->dsp_regmap[1];

	dsp->base = TACNA_DSP2_CLOCK_FREQ;
	dsp->base_sysinfo = TACNA_DSP2_SYS_INFO_ID;
	dsp->mem = cs47l96_dsp1ao_regions;
	dsp->num_mems = ARRAY_SIZE(cs47l96_dsp1ao_regions);

	dsp->n_rx_channels = CS47L96_AO_DSP_N_RX_CHANNELS;
	dsp->n_tx_channels = CS47L96_AO_DSP_N_TX_CHANNELS;

	ret = wm_halo_init(dsp, &cs47l96_ao->core.dsp_fw_lock,
			   &cs47l96_ao->core.rate_lock);
	if (ret != 0)
		goto error_dsp1ao_irq0;

	ret = tacna_request_irq(tacna, TACNA_IRQ_DSP2_MPU_ERR,
				"DSP1AO MPU", cs47l96_ao_mpu_fault_irq,
				&cs47l96_ao->core.dsp[0]);
	if (ret) {
		dev_warn(&pdev->dev, "Failed to get DSP1AO MPU IRQ: %d\n",
			 ret);
		goto error_dsp_core;
	}

	ret = tacna_request_irq(tacna, TACNA_IRQ_DSP2_WDT_EXPIRE,
				"DSP1AO WDT", wm_halo_wdt_expire,
				&cs47l96_ao->core.dsp[0]);
	if (ret) {
		dev_warn(&pdev->dev, "Failed to get DSP1AO WDT IRQ: %d\n",
			 ret);
		goto error_mpu_irq;
	}

	cs47l96_ao->fll.tacna_priv = &cs47l96_ao->core;
	cs47l96_ao->fll.id = 1;
	cs47l96_ao->fll.base = TACNA_FLL1AO_CONTROL1;
	cs47l96_ao->fll.sts_addr = TACNA_IRQ1_STS_6;
	cs47l96_ao->fll.sts_mask = TACNA_FLL1AO_LOCK_STS1_MASK;
	cs47l96_ao->fll.max_fref = 100000;
	cs47l96_ao->fll.integer_only = 1;
	cs47l96_ao->fll.has_lp = 1;
	tacna_init_fll(&cs47l96_ao->fll);

	for (i = 0; i < ARRAY_SIZE(cs47l96_ao_dai); i++)
		tacna_init_dai(&cs47l96_ao->core, i);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);

	ret = snd_soc_register_platform(&pdev->dev,
					&cs47l96_ao_compr_platform);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register platform: %d\n", ret);
		goto error_wdt_irq;
	}

	ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_cs47l96_ao,
				     cs47l96_ao_dai,
				     ARRAY_SIZE(cs47l96_ao_dai));
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register codec: %d\n", ret);
		goto error_unregister_platform;
	}

	return ret;

error_unregister_platform:
	snd_soc_unregister_platform(&pdev->dev);
error_wdt_irq:
	tacna_free_irq(tacna, TACNA_IRQ_DSP2_WDT_EXPIRE,
		       &cs47l96_ao->core.dsp[0]);
error_mpu_irq:
	tacna_free_irq(tacna, TACNA_IRQ_DSP2_MPU_ERR,
		       &cs47l96_ao->core.dsp[0]);
error_dsp_core:
	wm_adsp2_remove(&cs47l96_ao->core.dsp[0]);
error_dsp1ao_irq0:
	tacna_set_irq_wake(tacna, TACNA_IRQ_DSP2_IRQ0, 0);
	tacna_free_irq(tacna, TACNA_IRQ_DSP2_IRQ0, cs47l96_ao);

	return ret;
}

static int cs47l96_ao_remove(struct platform_device *pdev)
{
	struct cs47l96_ao *cs47l96_ao = platform_get_drvdata(pdev);
	struct tacna *tacna = cs47l96_ao->core.tacna;

	snd_soc_unregister_platform(&pdev->dev);
	snd_soc_unregister_codec(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	tacna_free_irq(tacna, TACNA_IRQ_DSP2_WDT_EXPIRE,
		       &cs47l96_ao->core.dsp[0]);
	tacna_free_irq(tacna, TACNA_IRQ_DSP2_MPU_ERR,
		       &cs47l96_ao->core.dsp[0]);

	tacna_set_irq_wake(tacna, TACNA_IRQ_DSP2_IRQ0, 0);
	tacna_free_irq(tacna, TACNA_IRQ_DSP2_IRQ0, cs47l96_ao);

	wm_adsp2_remove(&cs47l96_ao->core.dsp[0]);

	return 0;
}

static struct platform_driver cs47l96_ao_codec_driver = {
	.driver = {
		.name = "cs47l96-ao-codec",
		.owner = THIS_MODULE,
	},
	.probe = &cs47l96_ao_probe,
	.remove = &cs47l96_ao_remove,
};

module_platform_driver(cs47l96_ao_codec_driver);

MODULE_DESCRIPTION("ASoC CS47L96 AO driver");
MODULE_AUTHOR("Stuart Henderson <stuarth@opensource.cirrus.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cs47l96-ao-codec");
