/*
 * tacna.c - Cirrus Logic Tacna class codecs common support
 *
 * Copyright 2016-2017 Cirrus Logic
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/gcd.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>

#include <linux/mfd/tacna/core.h>
#include <linux/mfd/tacna/registers.h>
#include <linux/mfd/tacna/pdata.h>
#include <sound/tacna-pdata.h>

#include <dt-bindings/sound/tacna.h>

#include "tacna.h"

#define TACNA_ASP_ENABLES1			0x00
#define TACNA_ASP_CONTROL1			0x04
#define TACNA_ASP_CONTROL2			0x08
#define TACNA_ASP_CONTROL3			0x0c
#define TACNA_ASP_FRAME_CONTROL1		0x10
#define TACNA_ASP_FRAME_CONTROL2		0x14
#define TACNA_ASP_FRAME_CONTROL5		0x20
#define TACNA_ASP_FRAME_CONTROL6		0x24
#define TACNA_ASP_DATA_CONTROL1			0x30
#define TACNA_ASP_DATA_CONTROL5			0x40

#define TACNA_SYSCLK_RATE_6MHZ			0
#define TACNA_SYSCLK_RATE_12MHZ			1
#define TACNA_SYSCLK_RATE_24MHZ			2
#define TACNA_SYSCLK_RATE_49MHZ			3
#define TACNA_SYSCLK_RATE_98MHZ			4

#define TACNA_FLLHJ_INT_MAX_N			1023
#define TACNA_FLLHJ_INT_MIN_N			1
#define TACNA_FLLHJ_FRAC_MAX_N			255
#define TACNA_FLLHJ_FRAC_MIN_N			4
#define TACNA_FLLHJ_LOW_THRESH			192000
#define TACNA_FLLHJ_MID_THRESH			1152000
#define TACNA_FLLHJ_MAX_THRESH			13000000
#define TACNA_FLLHJ_LOW_GAINS			0x23f0
#define TACNA_FLLHJ_MID_GAINS			0x22f2
#define TACNA_FLLHJ_HIGH_GAINS			0x21f0
#define TACNA_FLL_MAX_FOUT			50000000
#define TACNA_FLL_MAX_REFDIV			8

#define TACNA_FLL_CONTROL1_OFFS			0x00
#define TACNA_FLL_CONTROL2_OFFS			0x04
#define TACNA_FLL_CONTROL3_OFFS			0x08
#define TACNA_FLL_CONTROL4_OFFS			0x0c
#define TACNA_FLL_CONTROL5_OFFS			0x10
#define TACNA_FLL_CONTROL6_OFFS			0x14
#define TACNA_FLL_DIGITAL_TEST2_OFFS		0x34
#define TACNA_FLL_GPIO_CLOCK_OFFS		0xa0

#define TACNA_DSP_CLOCK_FREQ_OFFS		0x00000

#define TACNA_ASP_FMT_DSP_MODE_A		0
#define TACNA_ASP_FMT_DSP_MODE_B		1
#define TACNA_ASP_FMT_I2S_MODE			2
#define TACNA_ASP_FMT_LEFT_JUSTIFIED_MODE	3

#define TACNA_CHANNEL_STATUS_POLL_US		1000
#define TACNA_CHANNEL_STATUS_POLL_TIMEOUT_US	20000

#define tacna_fll_err(_fll, fmt, ...) \
	dev_err(_fll->tacna->dev, "FLL%d: " fmt, _fll->id, ##__VA_ARGS__)
#define tacna_fll_warn(_fll, fmt, ...) \
	dev_warn(_fll->tacna->dev, "FLL%d: " fmt, _fll->id, ##__VA_ARGS__)
#define tacna_fll_dbg(_fll, fmt, ...) \
	dev_dbg(_fll->tacna->dev, "FLL%d: " fmt, _fll->id, ##__VA_ARGS__)

#define tacna_asp_err(_dai, fmt, ...) \
	dev_err(_dai->dev, "ASP%d: " fmt, _dai->id, ##__VA_ARGS__)
#define tacna_asp_warn(_dai, fmt, ...) \
	dev_warn(_dai->dev, "ASP%d: " fmt, _dai->id, ##__VA_ARGS__)
#define tacna_asp_dbg(_dai, fmt, ...) \
	dev_dbg(_dai->dev, "ASP%d: " fmt, _dai->id, ##__VA_ARGS__)

static const struct snd_soc_dapm_route tacna_mono_routes[] = {
	{ "OUT1R", NULL, "OUT1L" },
	{ "OUT2R", NULL, "OUT2L" },
	{ "OUT3R", NULL, "OUT3L" },
	{ "OUT4R", NULL, "OUT4L" },
};

const char * const tacna_mixer_texts[] = {
	"None",
	"Tone Generator 1",
	"Tone Generator 2",
	"Haptics",
	"AEC1",
	"AEC2",
	"Noise Generator",
	"IN1L",
	"IN1R",
	"IN2L",
	"IN2R",
	"IN3L",
	"IN3R",
	"IN4L",
	"IN4R",
	"HP_SENSE_L",
	"HP_SENSE_R",
	"ASP1RX1",
	"ASP1RX2",
	"ASP1RX3",
	"ASP1RX4",
	"ASP1RX5",
	"ASP1RX6",
	"ASP1RX7",
	"ASP1RX8",
	"ASP2RX1",
	"ASP2RX2",
	"ASP2RX3",
	"ASP2RX4",
	"ASP2RX5",
	"ASP2RX6",
	"ASP2RX7",
	"ASP2RX8",
	"ASP3RX1",
	"ASP3RX2",
	"ASP3RX3",
	"ASP3RX4",
	"ASP3RX5",
	"ASP3RX6",
	"ASP3RX7",
	"ASP3RX8",
	"ASP4RX1",
	"ASP4RX2",
	"ASP4RX3",
	"ASP4RX4",
	"ASP4RX5",
	"ASP4RX6",
	"ASP4RX7",
	"ASP4RX8",
	"SLIMRX1",
	"SLIMRX2",
	"SLIMRX3",
	"SLIMRX4",
	"SLIMRX5",
	"SLIMRX6",
	"SLIMRX7",
	"SLIMRX8",
	"ASRC1IN1L",
	"ASRC1IN1R",
	"ASRC1IN2L",
	"ASRC1IN2R",
	"ASRC2IN1L",
	"ASRC2IN1R",
	"ASRC2IN2L",
	"ASRC2IN2R",
	"ISRC1INT1",
	"ISRC1INT2",
	"ISRC1INT3",
	"ISRC1INT4",
	"ISRC1DEC1",
	"ISRC1DEC2",
	"ISRC1DEC3",
	"ISRC1DEC4",
	"ISRC2INT1",
	"ISRC2INT2",
	"ISRC2DEC1",
	"ISRC2DEC2",
	"EQ1",
	"EQ2",
	"EQ3",
	"EQ4",
	"DRC1L",
	"DRC1R",
	"DRC2L",
	"DRC2R",
	"LHPF1",
	"LHPF2",
	"LHPF3",
	"LHPF4",
	"DFC1",
	"DFC2",
	"DFC3",
	"DFC4",
	"DFC5",
	"DFC6",
	"DFC7",
	"DFC8",
	"Ultrasonic 1",
	"Ultrasonic 2",
	"DSP1.1",
	"DSP1.2",
	"DSP1.3",
	"DSP1.4",
	"DSP1.5",
	"DSP1.6",
	"DSP1.7",
	"DSP1.8",
	"DSP2.1",
	"DSP2.2",
	"DSP2.3",
	"DSP2.4",
	"DSP2.5",
	"DSP2.6",
	"DSP2.7",
	"DSP2.8",
	"SWIRE1DP3RX1",
	"SWIRE1DP3RX2",
	"SWIRE1DP3RX3",
	"SWIRE1DP3RX4",
	"SWIRE1DP3RX5",
	"SWIRE1DP3RX6",
	"SWIRE1DP4RX1",
	"SWIRE1DP4RX2",
	"SWIRE1DP4RX3",
	"SWIRE1DP4RX4",
	"SWIRE1DP4RX5",
	"SWIRE1DP4RX6",
};
EXPORT_SYMBOL_GPL(tacna_mixer_texts);

unsigned int tacna_mixer_values[] = {
	0x000, /* Silence (mute) */
	0x004, /* Tone generator 1 */
	0x005, /* Tone generator 2 */
	0x006, /* Haptic Generator */
	0x008, /* AEC loopback 1 */
	0x009, /* AEC loopback 2 */
	0x00C, /* Noise Generator */
	0x010, /* IN1L signal path */
	0x011, /* IN1R signal path */
	0x012, /* IN2L signal path */
	0x013, /* IN2R signal path */
	0x014, /* IN3L signal path */
	0x015, /* IN3R signal path */
	0x016, /* IN4L signal path */
	0x017, /* IN4R signal path */
	0x00E, /* HP_SENSE_L signal path */
	0x00F, /* HP_SENSE_R signal path */
	0x020, /* ASP1 RX1 */
	0x021, /* ASP1 RX2 */
	0x022, /* ASP1 RX3 */
	0x023, /* ASP1 RX4 */
	0x024, /* ASP1 RX5 */
	0x025, /* ASP1 RX6 */
	0x026, /* ASP1 RX7 */
	0x027, /* ASP1 RX8 */
	0x030, /* ASP2 RX1 */
	0x031, /* ASP2 RX2 */
	0x032, /* ASP2 RX3 */
	0x033, /* ASP2 RX4 */
	0x034, /* ASP2 RX5 */
	0x035, /* ASP2 RX6 */
	0x036, /* ASP2 RX7 */
	0x037, /* ASP2 RX8 */
	0x040, /* ASP3 RX1 */
	0x041, /* ASP3 RX2 */
	0x042, /* ASP3 RX3 */
	0x043, /* ASP3 RX4 */
	0x044, /* ASP3 RX5 */
	0x045, /* ASP3 RX6 */
	0x046, /* ASP3 RX7 */
	0x047, /* ASP3 RX8 */
	0x050, /* ASP4 RX1 */
	0x051, /* ASP4 RX2 */
	0x052, /* ASP4 RX3 */
	0x053, /* ASP4 RX4 */
	0x054, /* ASP4 RX5 */
	0x055, /* ASP4 RX6 */
	0x056, /* ASP4 RX7 */
	0x057, /* ASP4 RX8 */
	0x060, /* SLIMbus RX1 */
	0x061, /* SLIMbus RX2 */
	0x062, /* SLIMbus RX3 */
	0x063, /* SLIMbus RX4 */
	0x064, /* SLIMbus RX5 */
	0x065, /* SLIMbus RX6 */
	0x066, /* SLIMbus RX7 */
	0x067, /* SLIMbus RX8 */
	0x088, /* ASRC1 IN1 Left */
	0x089, /* ASRC1 IN1 Right */
	0x08A, /* ASRC1 IN2 Left */
	0x08B, /* ASRC1 IN2 Right */
	0x090, /* ASRC2 IN1 Left */
	0x091, /* ASRC2 IN1 Right */
	0x092, /* ASRC2 IN2 Left */
	0x093, /* ASRC2 IN2 Right */
	0x098, /* ISRC1 INT1 */
	0x099, /* ISRC1 INT2 */
	0x09a, /* ISRC1 INT3 */
	0x09b, /* ISRC1 INT4 */
	0x09C, /* ISRC1 DEC1 */
	0x09D, /* ISRC1 DEC2 */
	0x09e, /* ISRC1 DEC3 */
	0x09f, /* ISRC1 DEC4 */
	0x0A0, /* ISRC2 INT1 */
	0x0A1, /* ISRC2 INT2 */
	0x0A4, /* ISRC2 DEC1 */
	0x0A5, /* ISRC2 DEC2 */
	0x0B8, /* EQ1 */
	0x0B9, /* EQ2 */
	0x0BA, /* EQ3 */
	0x0BB, /* EQ4 */
	0x0C0, /* DRC1 Left */
	0x0C1, /* DRC1 Right */
	0x0C2, /* DRC2 Left */
	0x0C3, /* DRC2 Right */
	0x0C8, /* LHPF1 */
	0x0C9, /* LHPF2 */
	0x0CA, /* LHPF3 */
	0x0CB, /* LHPF4 */
	0x0D0, /* DFC1 channel 1 */
	0x0D1, /* DFC1 channel 2 */
	0x0D2, /* DFC1 channel 3 */
	0x0D3, /* DFC1 channel 4 */
	0x0D4, /* DFC1 channel 5 */
	0x0D5, /* DFC1 channel 6 */
	0x0D6, /* DFC1 channel 7 */
	0x0D7, /* DFC1 channel 8 */
	0x0D8, /* Ultrasonic 1 */
	0x0D9, /* Ultrasonic 2 */
	0x100, /* DSP1 channel 1 */
	0x101, /* DSP1 channel 2 */
	0x102, /* DSP1 channel 3 */
	0x103, /* DSP1 channel 4 */
	0x104, /* DSP1 channel 5 */
	0x105, /* DSP1 channel 6 */
	0x106, /* DSP1 channel 7 */
	0x107, /* DSP1 channel 8 */
	0x110, /* DSP2 channel 1 */
	0x111, /* DSP2 channel 2 */
	0x112, /* DSP2 channel 3 */
	0x113, /* DSP2 channel 4 */
	0x114, /* DSP2 channel 5 */
	0x115, /* DSP2 channel 6 */
	0x116, /* DSP2 channel 7 */
	0x117, /* DSP2 channel 8 */
	0x190, /* SWIRE1 DP3RX1 */
	0x191, /* SWIRE1 DP3RX2 */
	0x192, /* SWIRE1 DP3RX3 */
	0x193, /* SWIRE1 DP3RX4 */
	0x194, /* SWIRE1 DP3RX5 */
	0x195, /* SWIRE1 DP3RX6 */
	0x198, /* SWIRE1 DP4RX1 */
	0x199, /* SWIRE1 DP4RX2 */
	0x19A, /* SWIRE1 DP4RX3 */
	0x19B, /* SWIRE1 DP4RX4 */
	0x19C, /* SWIRE1 DP4RX5 */
	0x19D, /* SWIRE1 DP4RX6 */
};
EXPORT_SYMBOL_GPL(tacna_mixer_values);

const DECLARE_TLV_DB_SCALE(tacna_ana_tlv, 0, 100, 0);
EXPORT_SYMBOL_GPL(tacna_ana_tlv);

const DECLARE_TLV_DB_SCALE(tacna_eq_tlv, -1200, 100, 0);
EXPORT_SYMBOL_GPL(tacna_eq_tlv);

const DECLARE_TLV_DB_SCALE(tacna_digital_tlv, -6400, 50, 0);
EXPORT_SYMBOL_GPL(tacna_digital_tlv);

const DECLARE_TLV_DB_SCALE(tacna_noise_tlv, -10800, 600, 0);
EXPORT_SYMBOL_GPL(tacna_noise_tlv);

const DECLARE_TLV_DB_SCALE(tacna_mixer_tlv, -3200, 100, 0);
EXPORT_SYMBOL_GPL(tacna_mixer_tlv);

void tacna_spin_sysclk(struct tacna_priv *priv)
{
	struct tacna *tacna = priv->tacna;
	unsigned int val;
	int ret, i;

	/* Skip this if the chip is down */
	if (pm_runtime_suspended(tacna->dev))
		return;

	/*
	 * Just read a register a few times to ensure the internal
	 * oscillator sends out a few clocks.
	 */
	for (i = 0; i < 4; i++) {
		ret = regmap_read(tacna->regmap, TACNA_DEVID, &val);
		if (ret)
			dev_err(tacna->dev,
				"%s Failed to read register: %d (%d)\n",
				__func__, ret, i);
	}

	udelay(300);
}
EXPORT_SYMBOL_GPL(tacna_spin_sysclk);

static void tacna_debug_dump_domain_groups(const struct tacna_priv *priv)
{
	struct tacna *tacna = priv->tacna;
	int i;

	for (i = 0; i < ARRAY_SIZE(priv->domain_group_ref); ++i)
		dev_dbg(tacna->dev, "domain_grp_ref[%d]=%d\n", i,
			priv->domain_group_ref[i]);
}

static bool tacna_can_change_grp_rate(const struct tacna_priv *priv,
				      unsigned int reg,
				      unsigned char shift)
{
	int count;

	switch (reg) {
	case TACNA_FX_SAMPLE_RATE:
		count = priv->domain_group_ref[TACNA_DOM_GRP_FX];
		break;
	case TACNA_ASRC1_CONTROL1:
		switch (shift) {
		case TACNA_ASRC1_RATE1_SHIFT:
			count =
			    priv->domain_group_ref[TACNA_DOM_GRP_ASRC1_RATE_1];
			break;
		case TACNA_ASRC1_RATE2_SHIFT:
			count =
			    priv->domain_group_ref[TACNA_DOM_GRP_ASRC1_RATE_2];
			break;
		default:
			dev_warn(priv->tacna->dev,
				 "Unexpected shift 0x%x for rate reg 0x%x\n",
				 shift, reg);
			return false;
		}
		break;
	case TACNA_ASRC2_CONTROL1:
		switch (shift) {
		case TACNA_ASRC2_RATE1_SHIFT:
			count =
			    priv->domain_group_ref[TACNA_DOM_GRP_ASRC2_RATE_1];
			break;
		case TACNA_ASRC2_RATE2_SHIFT:
			count =
			    priv->domain_group_ref[TACNA_DOM_GRP_ASRC2_RATE_2];
			break;
		default:
			dev_warn(priv->tacna->dev,
				 "Unexpected shift 0x%x for rate reg 0x%x\n",
				 shift, reg);
			return false;
		}
		break;
	case TACNA_ISRC1_CONTROL1:
		switch (shift) {
		case TACNA_ISRC1_FSH_SHIFT:
			count = priv->domain_group_ref[TACNA_DOM_GRP_ISRC1_INT];
			break;
		case TACNA_ISRC1_FSL_SHIFT:
			count = priv->domain_group_ref[TACNA_DOM_GRP_ISRC1_DEC];
			break;
		default:
			dev_warn(priv->tacna->dev,
				 "Unexpected shift 0x%x for rate reg 0x%x\n",
				 shift, reg);
			return false;
		}
		break;
	case TACNA_ISRC2_CONTROL1:
		switch (shift) {
		case TACNA_ISRC2_FSH_SHIFT:
			count = priv->domain_group_ref[TACNA_DOM_GRP_ISRC2_INT];
			break;
		case TACNA_ISRC2_FSL_SHIFT:
			count = priv->domain_group_ref[TACNA_DOM_GRP_ISRC2_DEC];
			break;
		default:
			dev_warn(priv->tacna->dev,
				 "Unexpected shift 0x%x for rate reg 0x%x\n",
				 shift, reg);
			return false;
		}
		break;
	case TACNA_OUTPUT_CONTROL_1:
		count = priv->domain_group_ref[TACNA_DOM_GRP_DAC];
		break;
	case TACNA_ASP1_CONTROL1:
		count = priv->domain_group_ref[TACNA_DOM_GRP_ASP1];
		break;
	case TACNA_ASP2_CONTROL1:
		count = priv->domain_group_ref[TACNA_DOM_GRP_ASP2];
		break;
	case TACNA_ASP3_CONTROL1:
		count = priv->domain_group_ref[TACNA_DOM_GRP_ASP3];
		break;
	case TACNA_ASP4_CONTROL1:
		count = priv->domain_group_ref[TACNA_DOM_GRP_ASP4];
		break;
	case TACNA_SLIMBUS_RX_RATES1:
	case TACNA_SLIMBUS_RX_RATES2:
	case TACNA_SLIMBUS_TX_RATES1:
	case TACNA_SLIMBUS_TX_RATES2:
		count = priv->domain_group_ref[TACNA_DOM_GRP_SLIMBUS];
		break;
	case TACNA_PWM_DRIVE_1:
		count = priv->domain_group_ref[TACNA_DOM_GRP_PWM];
		break;
	case TACNA_DFC1_CH1_CTRL:
		count = priv->domain_group_ref[TACNA_DOM_GRP_DFC1];
		break;
	case TACNA_DFC1_CH2_CTRL:
		count = priv->domain_group_ref[TACNA_DOM_GRP_DFC2];
		break;
	case TACNA_DFC1_CH3_CTRL:
		count = priv->domain_group_ref[TACNA_DOM_GRP_DFC3];
		break;
	case TACNA_DFC1_CH4_CTRL:
		count = priv->domain_group_ref[TACNA_DOM_GRP_DFC4];
		break;
	case TACNA_DFC1_CH5_CTRL:
		count = priv->domain_group_ref[TACNA_DOM_GRP_DFC5];
		break;
	case TACNA_DFC1_CH6_CTRL:
		count = priv->domain_group_ref[TACNA_DOM_GRP_DFC6];
		break;
	case TACNA_DFC1_CH7_CTRL:
		count = priv->domain_group_ref[TACNA_DOM_GRP_DFC7];
		break;
	case TACNA_DFC1_CH8_CTRL:
		count = priv->domain_group_ref[TACNA_DOM_GRP_DFC8];
		break;
	case TACNA_DSP1_CLOCK_FREQ:
		count = priv->domain_group_ref[TACNA_DOM_GRP_DSP1];
		break;
	case TACNA_DSP2_CLOCK_FREQ:
		count = priv->domain_group_ref[TACNA_DOM_GRP_DSP2];
		break;
	default:
		return false;
	}

	dev_dbg(priv->tacna->dev, "Rate reg 0x%x group ref %d\n", reg, count);

	if (count)
		return false;
	else
		return true;
}

const char * const tacna_rate_text[TACNA_RATE_ENUM_SIZE] = {
	"Sample Rate 1",
	"Sample Rate 2",
	"Sample Rate 3",
	"Async Sample Rate 1",
	"Async Sample Rate 2",
};
EXPORT_SYMBOL_GPL(tacna_rate_text);

const unsigned int tacna_rate_val[TACNA_RATE_ENUM_SIZE] = {
	0x0, 0x1, 0x2, 0x8, 0x9,
};
EXPORT_SYMBOL_GPL(tacna_rate_val);

int tacna_rate_put(struct snd_kcontrol *kcontrol,
		   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	int ret;

	/* Prevent any mixer mux changes while we do this */
	mutex_lock(&priv->rate_lock);

	if (!tacna_can_change_grp_rate(priv, e->reg, e->shift_l)) {
		dev_warn(priv->tacna->dev,
			 "Cannot change '%s' while in use by active audio paths\n",
			 kcontrol->id.name);
		ret = -EBUSY;
	} else {
		/* The write must be guarded by a number of SYSCLK cycles */
		tacna_spin_sysclk(priv);
		ret = snd_soc_put_enum_double(kcontrol, ucontrol);
		tacna_spin_sysclk(priv);
	}

	mutex_unlock(&priv->rate_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tacna_rate_put);

const char * const tacna_sample_rate_text[TACNA_SAMPLE_RATE_ENUM_SIZE] = {
	"12kHz",
	"24kHz",
	"48kHz",
	"96kHz",
	"192kHz",
	"384kHz",
	"768kHz",
	"11.025kHz",
	"22.05kHz",
	"44.1kHz",
	"88.2kHz",
	"176.4kHz",
	"352.8kHz",
	"705.6kHz",
	"8kHz",
	"16kHz",
	"32kHz",
};
EXPORT_SYMBOL_GPL(tacna_sample_rate_text);

const unsigned int tacna_sample_rate_val[TACNA_SAMPLE_RATE_ENUM_SIZE] = {
	0x01, /* 12kHz */
	0x02, /* 24kHz */
	0x03, /* 48kHz */
	0x04, /* 96kHz */
	0x05, /* 192kHz */
	0x06, /* 384kHz */
	0x07, /* 768kHz */
	0x09, /* 11.025kHz */
	0x0a, /* 22.05kHz */
	0x0b, /* 44.1kHz */
	0x0c, /* 88.2kHz */
	0x0d, /* 176.4kHz */
	0x0e, /* 352.8kHz */
	0x0f, /* 705.6kHz */
	0x11, /* 8kHz */
	0x12, /* 16kHz */
	0x13, /* 32kHz */
};
EXPORT_SYMBOL_GPL(tacna_sample_rate_val);

const char *tacna_sample_rate_val_to_name(unsigned int rate_val)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tacna_sample_rate_val); ++i) {
		if (tacna_sample_rate_val[i] == rate_val)
			return tacna_sample_rate_text[i];
	}

	return "Illegal";
}
EXPORT_SYMBOL_GPL(tacna_sample_rate_val_to_name);

const struct soc_enum tacna_sample_rate[] = {
	SOC_VALUE_ENUM_SINGLE(TACNA_SAMPLE_RATE2,
			      TACNA_SAMPLE_RATE_2_SHIFT,
			      TACNA_SAMPLE_RATE_2_MASK >>
			      TACNA_SAMPLE_RATE_2_SHIFT,
			      TACNA_SAMPLE_RATE_ENUM_SIZE,
			      tacna_sample_rate_text,
			      tacna_sample_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_SAMPLE_RATE3,
			      TACNA_SAMPLE_RATE_3_SHIFT,
			      TACNA_SAMPLE_RATE_3_MASK >>
			      TACNA_SAMPLE_RATE_3_SHIFT,
			      TACNA_SAMPLE_RATE_ENUM_SIZE,
			      tacna_sample_rate_text,
			      tacna_sample_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_ASYNC_SAMPLE_RATE1,
			      TACNA_ASYNC_SAMPLE_RATE_1_SHIFT,
			      TACNA_ASYNC_SAMPLE_RATE_1_MASK >>
			      TACNA_ASYNC_SAMPLE_RATE_1_SHIFT,
			      TACNA_SAMPLE_RATE_ENUM_SIZE,
			      tacna_sample_rate_text,
			      tacna_sample_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_ASYNC_SAMPLE_RATE2,
			      TACNA_ASYNC_SAMPLE_RATE_2_SHIFT,
			      TACNA_ASYNC_SAMPLE_RATE_2_MASK >>
			      TACNA_ASYNC_SAMPLE_RATE_2_SHIFT,
			      TACNA_SAMPLE_RATE_ENUM_SIZE,
			      tacna_sample_rate_text,
			      tacna_sample_rate_val),
};
EXPORT_SYMBOL_GPL(tacna_sample_rate);

static int tacna_inmux_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_dapm_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm =
		snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct tacna *tacna = dev_get_drvdata(codec->dev->parent);
	struct soc_enum *e = (struct soc_enum *) kcontrol->private_value;
	unsigned int mux, src_val, inmode;
	int ret;

	mux = ucontrol->value.enumerated.item[0];
	if (mux > 1)
		return -EINVAL;

	switch (e->reg) {
	case TACNA_IN1L_CONTROL1:
		inmode = tacna->pdata.codec.inmode[0][2 * mux];
		break;
	case TACNA_IN1R_CONTROL1:
		inmode = tacna->pdata.codec.inmode[0][1 + (2 * mux)];
		break;
	case TACNA_IN2L_CONTROL1:
		inmode = tacna->pdata.codec.inmode[1][2 * mux];
		break;
	case TACNA_IN2R_CONTROL1:
		inmode = tacna->pdata.codec.inmode[1][1 + (2 * mux)];
		break;
	default:
		return -EINVAL;
	}

	src_val = mux << e->shift_l;

	if (inmode == TACNA_INMODE_SE)
		src_val |= 1 << TACNA_IN1L_SRC_SHIFT;

	dev_dbg(tacna->dev,
		"mux=%u reg=0x%x inmode=0x%x val=0x%x\n",
		mux, e->reg, inmode, src_val);

	ret = snd_soc_component_update_bits(dapm->component,
					    e->reg,
					    TACNA_IN1L_SRC_MASK,
					    src_val);
	if (ret < 0)
		return ret;
	else if (ret)
		return snd_soc_dapm_mux_update_power(dapm, kcontrol,
						     mux, e, NULL);
	else
		return 0;
}

static const char * const tacna_inmux_texts[] = {
	"Analog 1",
	"Analog 2",
};

static SOC_ENUM_SINGLE_DECL(tacna_in1muxl_enum,
			    TACNA_IN1L_CONTROL1,
			    TACNA_IN1L_SRC_SHIFT + 1,
			    tacna_inmux_texts);

static SOC_ENUM_SINGLE_DECL(tacna_in1muxr_enum,
			    TACNA_IN1R_CONTROL1,
			    TACNA_IN1R_SRC_SHIFT + 1,
			    tacna_inmux_texts);

static SOC_ENUM_SINGLE_DECL(tacna_in2muxl_enum,
			    TACNA_IN2L_CONTROL1,
			    TACNA_IN2L_SRC_SHIFT + 1,
			    tacna_inmux_texts);

static SOC_ENUM_SINGLE_DECL(tacna_in2muxr_enum,
			    TACNA_IN2R_CONTROL1,
			    TACNA_IN2R_SRC_SHIFT + 1,
			    tacna_inmux_texts);

const struct snd_kcontrol_new tacna_inmux[] = {
	SOC_DAPM_ENUM_EXT("IN1L Mux", tacna_in1muxl_enum,
			  snd_soc_dapm_get_enum_double, tacna_inmux_put),
	SOC_DAPM_ENUM_EXT("IN1R Mux", tacna_in1muxr_enum,
			  snd_soc_dapm_get_enum_double, tacna_inmux_put),
	SOC_DAPM_ENUM_EXT("IN2L Mux", tacna_in2muxl_enum,
			  snd_soc_dapm_get_enum_double, tacna_inmux_put),
	SOC_DAPM_ENUM_EXT("IN2R Mux", tacna_in2muxr_enum,
			  snd_soc_dapm_get_enum_double, tacna_inmux_put),
};
EXPORT_SYMBOL_GPL(tacna_inmux);

const struct snd_kcontrol_new tacna_inmode_switch[] = {
	SOC_DAPM_SINGLE("Switch", TACNA_INPUT1_CONTROL1, TACNA_IN1_MODE_SHIFT,
			1, 0),
	SOC_DAPM_SINGLE("Switch", TACNA_INPUT2_CONTROL1, TACNA_IN2_MODE_SHIFT,
			1, 0),
};
EXPORT_SYMBOL_GPL(tacna_inmode_switch);


static const char * const tacna_vol_ramp_text[] = {
	"0ms/6dB", "0.5ms/6dB", "1ms/6dB", "2ms/6dB", "4ms/6dB", "8ms/6dB",
	"16ms/6dB", "32ms/6dB",
};

SOC_ENUM_SINGLE_DECL(tacna_in_vd_ramp,
		     TACNA_INPUT_VOL_CONTROL,
		     TACNA_IN_VD_RAMP_SHIFT,
		     tacna_vol_ramp_text);
EXPORT_SYMBOL_GPL(tacna_in_vd_ramp);

SOC_ENUM_SINGLE_DECL(tacna_in_vi_ramp,
		     TACNA_INPUT_VOL_CONTROL,
		     TACNA_IN_VI_RAMP_SHIFT,
		     tacna_vol_ramp_text);
EXPORT_SYMBOL_GPL(tacna_in_vi_ramp);

static const char * const tacna_in_hpf_cut_text[] = {
	"2.5Hz", "5Hz", "10Hz", "20Hz", "40Hz"
};

SOC_ENUM_SINGLE_DECL(tacna_in_hpf_cut_enum,
		     TACNA_INPUT_HPF_CONTROL,
		     TACNA_IN_HPF_CUT_SHIFT,
		     tacna_in_hpf_cut_text);
EXPORT_SYMBOL_GPL(tacna_in_hpf_cut_enum);

static const char * const tacna_in_dmic_osr_text[TACNA_OSR_ENUM_SIZE] = {
	"384kHz", "768kHz", "1.536MHz", "2.048MHz", "2.4576MHz", "3.072MHz",
	"6.144MHz",
};

const struct soc_enum tacna_in_dmic_osr[] = {
	SOC_ENUM_SINGLE(TACNA_INPUT1_CONTROL1,
			TACNA_IN1_OSR_SHIFT,
			TACNA_OSR_ENUM_SIZE,
			tacna_in_dmic_osr_text),
	SOC_ENUM_SINGLE(TACNA_INPUT2_CONTROL1,
			TACNA_IN2_OSR_SHIFT,
			TACNA_OSR_ENUM_SIZE,
			tacna_in_dmic_osr_text),
	SOC_ENUM_SINGLE(TACNA_INPUT3_CONTROL1,
			TACNA_IN3_OSR_SHIFT,
			TACNA_OSR_ENUM_SIZE,
			tacna_in_dmic_osr_text),
	SOC_ENUM_SINGLE(TACNA_INPUT4_CONTROL1,
			TACNA_IN4_OSR_SHIFT,
			TACNA_OSR_ENUM_SIZE,
			tacna_in_dmic_osr_text),
};
EXPORT_SYMBOL_GPL(tacna_in_dmic_osr);

int tacna_in_rate_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int reg, shift;
	int ret = 0;

	snd_soc_dapm_mutex_lock(dapm);

	/* Cannot change rate on an active input */
	reg = snd_soc_read(codec, TACNA_INPUT_CONTROL);
	shift = (e->reg - TACNA_IN1L_CONTROL1) / 0x20;
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
EXPORT_SYMBOL_GPL(tacna_in_rate_put);

const struct soc_enum tacna_input_rate[] = {
	SOC_VALUE_ENUM_SINGLE(TACNA_IN1L_CONTROL1,
			      TACNA_IN1L_RATE_SHIFT,
			      TACNA_IN1L_RATE_MASK >> TACNA_IN1L_RATE_SHIFT,
			      TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_IN1R_CONTROL1,
			      TACNA_IN1R_RATE_SHIFT,
			      TACNA_IN1R_RATE_MASK >> TACNA_IN1R_RATE_SHIFT,
			      TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_IN2L_CONTROL1,
			      TACNA_IN2L_RATE_SHIFT,
			      TACNA_IN2L_RATE_MASK >> TACNA_IN2L_RATE_SHIFT,
			      TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_IN2R_CONTROL1,
			      TACNA_IN2R_RATE_SHIFT,
			      TACNA_IN2R_RATE_MASK >> TACNA_IN2R_RATE_SHIFT,
			      TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_IN3L_CONTROL1,
			      TACNA_IN3L_RATE_SHIFT,
			      TACNA_IN3L_RATE_MASK >> TACNA_IN3L_RATE_SHIFT,
			      TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_IN3R_CONTROL1,
			      TACNA_IN3R_RATE_SHIFT,
			      TACNA_IN3R_RATE_MASK >> TACNA_IN3R_RATE_SHIFT,
			      TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_IN4L_CONTROL1,
			      TACNA_IN4L_RATE_SHIFT,
			      TACNA_IN4L_RATE_MASK >> TACNA_IN4L_RATE_SHIFT,
			      TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_IN4R_CONTROL1,
			      TACNA_IN4R_RATE_SHIFT,
			      TACNA_IN4R_RATE_MASK >> TACNA_IN4R_RATE_SHIFT,
			      TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
};
EXPORT_SYMBOL_GPL(tacna_input_rate);

int tacna_low_power_mode_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	unsigned int reg, mask;
	int ret;

	snd_soc_dapm_mutex_lock(dapm);

	/* Cannot change low power mode on an active input */
	reg = snd_soc_read(codec, TACNA_INPUT_CONTROL);
	mask = (mc->reg - TACNA_IN1L_CONTROL1) / 0x20;
	mask ^= 0x1; /* Flip bottom bit for channel order */

	if ((reg) & (1 << mask)) {
		ret = -EBUSY;
		dev_err(codec->dev,
			"Can't change lp mode on an active input\n");
		goto exit;
	}

	ret = snd_soc_put_volsw(kcontrol, ucontrol);

exit:
	snd_soc_dapm_mutex_unlock(dapm);
	return ret;
}
EXPORT_SYMBOL_GPL(tacna_low_power_mode_put);

static const char * const tacna_auxpdm_freq_texts[] = {
	"3.072MHz",
	"2.048MHz",
	"1.536MHz",
	"768kHz",
};

SOC_ENUM_SINGLE_DECL(tacna_auxpdm1_freq,
		     TACNA_AUXPDM1_CONTROL1,
		     TACNA_AUXPDM1_CLK_FREQ_SHIFT,
		     tacna_auxpdm_freq_texts);
EXPORT_SYMBOL_GPL(tacna_auxpdm1_freq);

SOC_ENUM_SINGLE_DECL(tacna_auxpdm2_freq,
		     TACNA_AUXPDM2_CONTROL1,
		     TACNA_AUXPDM2_CLK_FREQ_SHIFT,
		     tacna_auxpdm_freq_texts);
EXPORT_SYMBOL_GPL(tacna_auxpdm2_freq);

SOC_ENUM_SINGLE_DECL(tacna_auxpdm3_freq,
		     TACNA_AUXPDM3_CONTROL1,
		     TACNA_AUXPDM3_CLK_FREQ_SHIFT,
		     tacna_auxpdm_freq_texts);
EXPORT_SYMBOL_GPL(tacna_auxpdm3_freq);

static const char * const tacna_auxpdm_in_texts[] = {
	"IN1L",
	"IN1R",
	"IN2L",
	"IN2R",
	"IN3L",
	"IN3R",
	"IN4L",
	"IN4R",
};

static SOC_ENUM_SINGLE_DECL(tacna_auxpdm1_in,
			    TACNA_AUXPDM1_CONTROL1,
			    TACNA_AUXPDM1_SRC_SHIFT,
			    tacna_auxpdm_in_texts);

static SOC_ENUM_SINGLE_DECL(tacna_auxpdm2_in,
			    TACNA_AUXPDM2_CONTROL1,
			    TACNA_AUXPDM2_SRC_SHIFT,
			    tacna_auxpdm_in_texts);

static SOC_ENUM_SINGLE_DECL(tacna_auxpdm3_in,
			    TACNA_AUXPDM3_CONTROL1,
			    TACNA_AUXPDM3_SRC_SHIFT,
			    tacna_auxpdm_in_texts);

const struct snd_kcontrol_new tacna_auxpdm_inmux[] = {
	SOC_DAPM_ENUM("AUXPDM1 Input", tacna_auxpdm1_in),
	SOC_DAPM_ENUM("AUXPDM2 Input", tacna_auxpdm2_in),
	SOC_DAPM_ENUM("AUXPDM3 Input", tacna_auxpdm3_in),
};
EXPORT_SYMBOL_GPL(tacna_auxpdm_inmux);

const struct snd_kcontrol_new tacna_auxpdm_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
};
EXPORT_SYMBOL_GPL(tacna_auxpdm_switch);

int tacna_put_out_vu(struct snd_kcontrol *kcontrol,
		     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int ret;

	snd_soc_dapm_mutex_lock(dapm);
	snd_soc_component_update_bits(component, mc->reg, TACNA_OUT_VU, 0);
	if (mc->rreg)
		snd_soc_component_update_bits(component, mc->rreg, TACNA_OUT_VU,
					      0);

	ret = snd_soc_put_volsw(kcontrol, ucontrol);

	snd_soc_component_update_bits(component, mc->reg, TACNA_OUT_VU,
				      TACNA_OUT_VU);
	if (mc->rreg)
		snd_soc_component_update_bits(component, mc->rreg, TACNA_OUT_VU,
					      TACNA_OUT_VU);
	snd_soc_dapm_mutex_unlock(dapm);

	return ret;
}
EXPORT_SYMBOL_GPL(tacna_put_out_vu);

const struct soc_enum tacna_output_rate =
	SOC_VALUE_ENUM_SINGLE(TACNA_OUTPUT_CONTROL_1,
			      TACNA_OUT_RATE_SHIFT,
			      TACNA_OUT_RATE_MASK >> TACNA_OUT_RATE_SHIFT,
			      TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val);
EXPORT_SYMBOL_GPL(tacna_output_rate);

SOC_ENUM_SINGLE_DECL(tacna_out_vd_ramp,
		     TACNA_OUTPUT_VOLUME_RAMP,
		     TACNA_OUT_VD_RAMP_SHIFT,
		     tacna_vol_ramp_text);
EXPORT_SYMBOL_GPL(tacna_out_vd_ramp);

SOC_ENUM_SINGLE_DECL(tacna_out_vi_ramp,
		     TACNA_OUTPUT_VOLUME_RAMP,
		     TACNA_OUT_VI_RAMP_SHIFT,
		     tacna_vol_ramp_text);
EXPORT_SYMBOL_GPL(tacna_out_vi_ramp);

static const char * const tacna_anc_input_src_text[] = {
	"None", "IN1", "IN2", "IN3", "IN4",
};

static const char * const tacna_mono_anc_channel_src_text[] = {
	"Left", "Right",
};

const struct soc_enum tacna_mono_anc_input_src[] = {
	SOC_ENUM_SINGLE(TACNA_ANC_SRC,
			TACNA_IN_ANC_L_SRC_SHIFT,
			ARRAY_SIZE(tacna_anc_input_src_text),
			tacna_anc_input_src_text),
	SOC_ENUM_SINGLE(TACNA_ANC_CTRL_3,
			TACNA_ANC_SWAP_CHAN_SHIFT,
			ARRAY_SIZE(tacna_mono_anc_channel_src_text),
			tacna_mono_anc_channel_src_text),
};
EXPORT_SYMBOL_GPL(tacna_mono_anc_input_src);

static const char * const tacna_anc_ng_texts[] = {
	"None", "Internal", "External",
};

SOC_ENUM_SINGLE_DECL(tacna_anc_ng_enum, SND_SOC_NOPM, 0, tacna_anc_ng_texts);
EXPORT_SYMBOL_GPL(tacna_anc_ng_enum);

static const char * const tacna_out_anc_src_text[] = {
	"None", "ANC Left Channel", "ANC Right Channel",
};

const struct soc_enum tacna_output_anc_src[] = {
	SOC_ENUM_SINGLE(TACNA_OUT1L_CONTROL_1,
			TACNA_OUT1L_ANC_SRC_SHIFT,
			ARRAY_SIZE(tacna_out_anc_src_text),
			tacna_out_anc_src_text),
	SOC_ENUM_SINGLE(TACNA_OUT1R_CONTROL_1,
			TACNA_OUT1R_ANC_SRC_SHIFT,
			ARRAY_SIZE(tacna_out_anc_src_text),
			tacna_out_anc_src_text),
	SOC_ENUM_SINGLE(TACNA_OUT2L_CONTROL_1,
			TACNA_OUT2L_ANC_SRC_SHIFT,
			ARRAY_SIZE(tacna_out_anc_src_text),
			tacna_out_anc_src_text),
	SOC_ENUM_SINGLE(TACNA_OUT2R_CONTROL_1,
			TACNA_OUT2R_ANC_SRC_SHIFT,
			ARRAY_SIZE(tacna_out_anc_src_text),
			tacna_out_anc_src_text),
	SOC_ENUM_SINGLE(TACNA_OUT5L_CONTROL_1,
			TACNA_OUT5L_ANC_SRC_SHIFT,
			ARRAY_SIZE(tacna_out_anc_src_text),
			tacna_out_anc_src_text),
	SOC_ENUM_SINGLE(TACNA_OUT5R_CONTROL_1,
			TACNA_OUT5R_ANC_SRC_SHIFT,
			ARRAY_SIZE(tacna_out_anc_src_text),
			tacna_out_anc_src_text),
};
EXPORT_SYMBOL_GPL(tacna_output_anc_src);

int tacna_frf_bytes_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_bytes *params = (void *)kcontrol->private_value;
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = priv->tacna;
	int ret, len;
	void *data;

	len = params->num_regs * component->val_bytes;

	data = kmemdup(ucontrol->value.bytes.data, len, GFP_KERNEL | GFP_DMA);
	if (!data)
		return -ENOMEM;

	ret = regmap_raw_write(tacna->regmap, params->base, data, len);

	kfree(data);
	return ret;
}
EXPORT_SYMBOL_GPL(tacna_frf_bytes_put);

const struct soc_enum tacna_asrc1_rate[] = {
	SOC_VALUE_ENUM_SINGLE(TACNA_ASRC1_CONTROL1,
			      TACNA_ASRC1_RATE1_SHIFT,
			      TACNA_ASRC1_RATE1_MASK >> TACNA_ASRC1_RATE1_SHIFT,
			      TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_ASRC1_CONTROL1,
			      TACNA_ASRC1_RATE2_SHIFT,
			      TACNA_ASRC1_RATE2_MASK >> TACNA_ASRC1_RATE2_SHIFT,
			      TACNA_ASYNC_RATE_ENUM_SIZE,
			      tacna_rate_text + TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_val + TACNA_SYNC_RATE_ENUM_SIZE),
};
EXPORT_SYMBOL_GPL(tacna_asrc1_rate);

const struct soc_enum tacna_asrc2_rate[] = {
	SOC_VALUE_ENUM_SINGLE(TACNA_ASRC2_CONTROL1,
			      TACNA_ASRC2_RATE1_SHIFT,
			      TACNA_ASRC2_RATE1_MASK >> TACNA_ASRC2_RATE1_SHIFT,
			      TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_ASRC2_CONTROL1,
			      TACNA_ASRC2_RATE2_SHIFT,
			      TACNA_ASRC2_RATE2_MASK >> TACNA_ASRC2_RATE2_SHIFT,
			      TACNA_ASYNC_RATE_ENUM_SIZE,
			      tacna_rate_text + TACNA_SYNC_RATE_ENUM_SIZE,
			      tacna_rate_val + TACNA_SYNC_RATE_ENUM_SIZE),
};
EXPORT_SYMBOL_GPL(tacna_asrc2_rate);

const struct soc_enum tacna_isrc_fsh[] = {
	SOC_VALUE_ENUM_SINGLE(TACNA_ISRC1_CONTROL1,
			      TACNA_ISRC1_FSH_SHIFT,
			      TACNA_ISRC1_FSH_MASK >> TACNA_ISRC1_FSH_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_ISRC2_CONTROL1,
			      TACNA_ISRC2_FSH_SHIFT,
			      TACNA_ISRC2_FSH_MASK >> TACNA_ISRC2_FSH_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
};
EXPORT_SYMBOL_GPL(tacna_isrc_fsh);

const struct soc_enum tacna_isrc_fsl[] = {
	SOC_VALUE_ENUM_SINGLE(TACNA_ISRC1_CONTROL1,
			      TACNA_ISRC1_FSL_SHIFT,
			      TACNA_ISRC1_FSL_MASK >> TACNA_ISRC1_FSL_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_ISRC2_CONTROL1,
			      TACNA_ISRC2_FSL_SHIFT,
			      TACNA_ISRC2_FSL_MASK >> TACNA_ISRC2_FSL_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
};
EXPORT_SYMBOL_GPL(tacna_isrc_fsl);

const struct soc_enum tacna_fx_rate =
	SOC_VALUE_ENUM_SINGLE(TACNA_FX_SAMPLE_RATE,
			      TACNA_FX_RATE_SHIFT,
			      TACNA_FX_RATE_MASK >> TACNA_FX_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val);
EXPORT_SYMBOL_GPL(tacna_fx_rate);

static const char * const tacna_lhpf_mode_text[] = {
	"Low-pass", "High-pass"
};

SOC_ENUM_SINGLE_DECL(tacna_lhpf1_mode,
		     TACNA_LHPF_CONTROL2,
		     TACNA_LHPF1_MODE_SHIFT,
		     tacna_lhpf_mode_text);
EXPORT_SYMBOL_GPL(tacna_lhpf1_mode);

SOC_ENUM_SINGLE_DECL(tacna_lhpf2_mode,
		     TACNA_LHPF_CONTROL2,
		     TACNA_LHPF2_MODE_SHIFT,
		     tacna_lhpf_mode_text);
EXPORT_SYMBOL_GPL(tacna_lhpf2_mode);

SOC_ENUM_SINGLE_DECL(tacna_lhpf3_mode,
		     TACNA_LHPF_CONTROL2,
		     TACNA_LHPF3_MODE_SHIFT,
		     tacna_lhpf_mode_text);
EXPORT_SYMBOL_GPL(tacna_lhpf3_mode);

SOC_ENUM_SINGLE_DECL(tacna_lhpf4_mode,
		     TACNA_LHPF_CONTROL2,
		     TACNA_LHPF4_MODE_SHIFT,
		     tacna_lhpf_mode_text);
EXPORT_SYMBOL_GPL(tacna_lhpf4_mode);

int tacna_lhpf_coeff_put(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tacna *tacna = dev_get_drvdata(codec->dev->parent);
	__be32 *data = (__be32 *)ucontrol->value.bytes.data;
	s16 val = (s16)be32_to_cpu(*data);

	if (abs(val) >= 4096) {
		dev_err(tacna->dev, "Rejecting unstable LHPF coefficients\n");
		return -EINVAL;
	}

	return snd_soc_bytes_put(kcontrol, ucontrol);
}
EXPORT_SYMBOL_GPL(tacna_lhpf_coeff_put);

static const char * const tacna_eq_mode_text[] = {
	"Low-pass", "High-pass",
};

const struct soc_enum tacna_eq_mode[] = {
	SOC_ENUM_SINGLE(TACNA_EQ_CONTROL2,
			TACNA_EQ1_B1_MODE_SHIFT,
			ARRAY_SIZE(tacna_eq_mode_text),
			tacna_eq_mode_text),
	SOC_ENUM_SINGLE(TACNA_EQ_CONTROL2,
			TACNA_EQ2_B1_MODE_SHIFT,
			ARRAY_SIZE(tacna_eq_mode_text),
			tacna_eq_mode_text),
	SOC_ENUM_SINGLE(TACNA_EQ_CONTROL2,
			TACNA_EQ3_B1_MODE_SHIFT,
			ARRAY_SIZE(tacna_eq_mode_text),
			tacna_eq_mode_text),
	SOC_ENUM_SINGLE(TACNA_EQ_CONTROL2,
			TACNA_EQ4_B1_MODE_SHIFT,
			ARRAY_SIZE(tacna_eq_mode_text),
			tacna_eq_mode_text),
};
EXPORT_SYMBOL_GPL(tacna_eq_mode);

int tacna_eq_mode_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *) kcontrol->private_value;
	unsigned int item;

	item = snd_soc_enum_val_to_item(e, priv->eq_mode[e->shift_l]);
	ucontrol->value.enumerated.item[0] = item;

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_eq_mode_get);

int tacna_eq_mode_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *) kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	unsigned int val;

	if (item[0] >= e->items)
		return -EINVAL;
	val = snd_soc_enum_item_to_val(e, item[0]);

	snd_soc_dapm_mutex_lock(dapm);
	priv->eq_mode[e->shift_l] = val;
	snd_soc_dapm_mutex_unlock(dapm);

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_eq_mode_put);

int tacna_eq_coeff_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	struct tacna_eq_control *ctl = (void *) kcontrol->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = ctl->max;

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_eq_coeff_info);

int tacna_eq_coeff_get(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna_eq_control *params = (void *)kcontrol->private_value;
	__be16 *coeffs;
	unsigned int coeff_idx;
	int block_idx;

	block_idx = ((int) params->block_base - (int) TACNA_EQ1_BAND1_COEFF1);
	block_idx /= 68;

	coeffs = &priv->eq_coefficients[block_idx][0];

	coeff_idx = (params->reg - params->block_base) / 2;
	coeff_idx += ((params->shift == 0) ? 1 : 0);

	ucontrol->value.integer.value[0] = be16_to_cpu(coeffs[coeff_idx]);

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_eq_coeff_get);

int tacna_eq_coeff_put(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna_eq_control *params = (void *)kcontrol->private_value;
	__be16 *coeffs;
	unsigned int coeff_idx;
	int block_idx;

	block_idx = ((int) params->block_base - (int) TACNA_EQ1_BAND1_COEFF1);
	block_idx /= 68;

	coeffs = &priv->eq_coefficients[block_idx][0];

	coeff_idx = (params->reg - params->block_base) / 2;
	coeff_idx += ((params->shift == 0) ? 1 : 0);

	snd_soc_dapm_mutex_lock(dapm);
	coeffs[coeff_idx] = cpu_to_be16(ucontrol->value.integer.value[0]);
	snd_soc_dapm_mutex_unlock(dapm);

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_eq_coeff_put);

const struct snd_kcontrol_new tacna_drc_activity_output_mux[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
};
EXPORT_SYMBOL_GPL(tacna_drc_activity_output_mux);

const char * const tacna_dfc_width_text[TACNA_DFC_WIDTH_ENUM_SIZE] = {
	"8bit",
	"9bit",
	"10bit",
	"11bit",
	"12bit",
	"13bit",
	"14bit",
	"15bit",
	"16bit",
	"17bit",
	"18bit",
	"19bit",
	"20bit",
	"21bit",
	"22bit",
	"23bit",
	"24bit",
	"25bit",
	"26bit",
	"27bit",
	"28bit",
	"29bit",
	"30bit",
	"31bit",
	"32bit",
};
EXPORT_SYMBOL_GPL(tacna_dfc_width_text);

const unsigned int tacna_dfc_width_val[TACNA_DFC_WIDTH_ENUM_SIZE] = {
	7,  /*  8bit */
	8,  /*  9bit */
	9,  /* 10bit */
	10, /* 11bit */
	11, /* 12bit */
	12, /* 13bit */
	13, /* 14bit */
	14, /* 15bit */
	15, /* 16bit */
	16, /* 17bit */
	17, /* 18bit */
	18, /* 19bit */
	19, /* 20bit */
	20, /* 21bit */
	21, /* 22bit */
	22, /* 23bit */
	23, /* 24bit */
	24, /* 25bit */
	25, /* 26bit */
	26, /* 27bit */
	27, /* 28bit */
	28, /* 29bit */
	29, /* 30bit */
	30, /* 31bit */
	31, /* 32bit */
};
EXPORT_SYMBOL_GPL(tacna_dfc_width_val);

const char * const tacna_dfc_type_text[TACNA_DFC_TYPE_ENUM_SIZE] = {
	"Fixed", "Unsigned Fixed", "Single Precision Floating",
	"Half Precision Floating", "Arm Alternative Floating",
};
EXPORT_SYMBOL_GPL(tacna_dfc_type_text);

const unsigned int tacna_dfc_type_val[TACNA_DFC_TYPE_ENUM_SIZE] = {
	0, 1, 2, 4, 5,
};
EXPORT_SYMBOL_GPL(tacna_dfc_type_val);

const struct soc_enum tacna_dfc_rate[] = {
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH1_CTRL,
			      TACNA_DFC1_CH1_RATE_SHIFT,
			      TACNA_DFC1_CH1_RATE_MASK >>
			      TACNA_DFC1_CH1_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH2_CTRL,
			      TACNA_DFC1_CH2_RATE_SHIFT,
			      TACNA_DFC1_CH2_RATE_MASK >>
			      TACNA_DFC1_CH2_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH3_CTRL,
			      TACNA_DFC1_CH3_RATE_SHIFT,
			      TACNA_DFC1_CH3_RATE_MASK >>
			      TACNA_DFC1_CH3_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH4_CTRL,
			      TACNA_DFC1_CH4_RATE_SHIFT,
			      TACNA_DFC1_CH4_RATE_MASK >>
			      TACNA_DFC1_CH4_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH5_CTRL,
			      TACNA_DFC1_CH5_RATE_SHIFT,
			      TACNA_DFC1_CH5_RATE_MASK >>
			      TACNA_DFC1_CH5_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH6_CTRL,
			      TACNA_DFC1_CH6_RATE_SHIFT,
			      TACNA_DFC1_CH6_RATE_MASK >>
			      TACNA_DFC1_CH6_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH7_CTRL,
			      TACNA_DFC1_CH7_RATE_SHIFT,
			      TACNA_DFC1_CH7_RATE_MASK >>
			      TACNA_DFC1_CH7_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH8_CTRL,
			      TACNA_DFC1_CH8_RATE_SHIFT,
			      TACNA_DFC1_CH8_RATE_MASK >>
			      TACNA_DFC1_CH8_RATE_SHIFT,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,
			      tacna_rate_val),
};
EXPORT_SYMBOL_GPL(tacna_dfc_rate);

const struct soc_enum tacna_dfc_width[] = {
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH1_RX,
			      TACNA_DFC1_CH1_RX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH1_RX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH1_RX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH1_TX,
			      TACNA_DFC1_CH1_TX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH1_TX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH1_TX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH2_RX,
			      TACNA_DFC1_CH2_RX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH2_RX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH2_RX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH2_TX,
			      TACNA_DFC1_CH2_TX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH2_TX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH2_TX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH3_RX,
			      TACNA_DFC1_CH3_RX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH3_RX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH3_RX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH3_TX,
			      TACNA_DFC1_CH3_TX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH3_TX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH3_TX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH4_RX,
			      TACNA_DFC1_CH4_RX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH4_RX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH4_RX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH4_TX,
			      TACNA_DFC1_CH4_TX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH4_TX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH4_TX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH5_RX,
			      TACNA_DFC1_CH5_RX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH5_RX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH5_RX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH5_TX,
			      TACNA_DFC1_CH5_TX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH5_TX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH5_TX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH6_RX,
			      TACNA_DFC1_CH6_RX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH6_RX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH6_RX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH6_TX,
			      TACNA_DFC1_CH6_TX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH6_TX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH6_TX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH7_RX,
			      TACNA_DFC1_CH7_RX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH7_RX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH7_RX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH7_TX,
			      TACNA_DFC1_CH7_TX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH7_TX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH7_TX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH8_RX,
			      TACNA_DFC1_CH8_RX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH8_RX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH8_RX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH8_TX,
			      TACNA_DFC1_CH8_TX_DATA_WIDTH_SHIFT,
			      TACNA_DFC1_CH8_TX_DATA_WIDTH_MASK >>
			      TACNA_DFC1_CH8_TX_DATA_WIDTH_SHIFT,
			      ARRAY_SIZE(tacna_dfc_width_text),
			      tacna_dfc_width_text,
			      tacna_dfc_width_val),
};
EXPORT_SYMBOL_GPL(tacna_dfc_width);

const struct soc_enum tacna_dfc_type[] = {
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH1_RX,
			      TACNA_DFC1_CH1_RX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH1_RX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH1_RX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH1_TX,
			      TACNA_DFC1_CH1_TX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH1_TX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH1_TX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH2_RX,
			      TACNA_DFC1_CH2_RX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH2_RX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH2_RX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH2_TX,
			      TACNA_DFC1_CH2_TX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH2_TX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH2_TX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH3_RX,
			      TACNA_DFC1_CH3_RX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH3_RX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH3_RX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH3_TX,
			      TACNA_DFC1_CH3_TX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH3_TX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH3_TX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH4_RX,
			      TACNA_DFC1_CH4_RX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH4_RX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH4_RX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH4_TX,
			      TACNA_DFC1_CH4_TX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH4_TX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH4_TX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH5_RX,
			      TACNA_DFC1_CH5_RX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH5_RX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH5_RX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH5_TX,
			      TACNA_DFC1_CH5_TX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH5_TX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH5_TX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH6_RX,
			      TACNA_DFC1_CH6_RX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH6_RX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH6_RX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH6_TX,
			      TACNA_DFC1_CH6_TX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH6_TX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH6_TX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH7_RX,
			      TACNA_DFC1_CH7_RX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH7_RX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH7_RX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH7_TX,
			      TACNA_DFC1_CH7_TX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH7_TX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH7_TX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH8_RX,
			      TACNA_DFC1_CH8_RX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH8_RX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH8_RX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
	SOC_VALUE_ENUM_SINGLE(TACNA_DFC1_CH8_TX,
			      TACNA_DFC1_CH8_TX_DATA_TYPE_SHIFT,
			      TACNA_DFC1_CH8_TX_DATA_TYPE_MASK >>
			      TACNA_DFC1_CH8_TX_DATA_TYPE_SHIFT,
			      ARRAY_SIZE(tacna_dfc_type_text),
			      tacna_dfc_type_text,
			      tacna_dfc_type_val),
};
EXPORT_SYMBOL_GPL(tacna_dfc_type);

int tacna_dfc_dith_put(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *) kcontrol->private_value;
	unsigned int val;
	int ret;

	snd_soc_dapm_mutex_lock(dapm);

	/* Cannot change dfc settings when its on */
	val = snd_soc_read(codec, mc->reg);
	if (val & TACNA_DFC1_CH1_EN) {
		ret = -EBUSY;
		goto exit;
	}

	ret = snd_soc_put_volsw(kcontrol, ucontrol);
exit:
	snd_soc_dapm_mutex_unlock(dapm);

	return ret;
}
EXPORT_SYMBOL_GPL(tacna_dfc_dith_put);

int tacna_dfc_put(struct snd_kcontrol *kcontrol,
		  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int val;
	int ret = 0;

	reg = TACNA_DFC1_CH1_CTRL + ((reg - TACNA_DFC1_CH1_CTRL) / 12) * 12;

	snd_soc_dapm_mutex_lock(dapm);

	/* Cannot change dfc settings when its on */
	val = snd_soc_read(codec, reg);
	if (val & TACNA_DFC1_CH1_EN) {
		ret = -EBUSY;
		goto exit;
	}

	ret = snd_soc_put_enum_double(kcontrol, ucontrol);
exit:
	snd_soc_dapm_mutex_unlock(dapm);
	return ret;
}
EXPORT_SYMBOL_GPL(tacna_dfc_put);

const struct snd_kcontrol_new tacna_dsp_trigger_output_mux[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0),
};
EXPORT_SYMBOL_GPL(tacna_dsp_trigger_output_mux);

#define TACNA_DSP_RATE_CTL_DIR_MASK	0x1
#define TACNA_DSP_RATE_CTL_DIR_RX	0x0
#define TACNA_DSP_RATE_CTL_DIR_TX	0x1
#define TACNA_DSP_RATE_CTL_NUM_SHIFT	1

int tacna_dsp_rate_get(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *) kcontrol->private_value;
	unsigned int cached_rate;
	const int dsp_num = e->shift_l;
	const unsigned int rate_num = e->mask >> TACNA_DSP_RATE_CTL_NUM_SHIFT;
	int item;

	snd_soc_dapm_mutex_lock(dapm);
	if (e->mask & TACNA_DSP_RATE_CTL_DIR_MASK)
		cached_rate = priv->dsp[dsp_num].tx_rate_cache[rate_num];
	else
		cached_rate = priv->dsp[dsp_num].rx_rate_cache[rate_num];
	snd_soc_dapm_mutex_unlock(dapm);

	item = snd_soc_enum_val_to_item(e, cached_rate);
	ucontrol->value.enumerated.item[0] = item;

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_dsp_rate_get);

int tacna_dsp_rate_put(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *) kcontrol->private_value;
	const int dsp_num = e->shift_l;
	const int rate_num = e->mask >> TACNA_DSP_RATE_CTL_NUM_SHIFT;
	const unsigned int item = ucontrol->value.enumerated.item[0];
	int ret;

	if (item >= e->items)
		return -EINVAL;

	snd_soc_dapm_mutex_lock(dapm);

	if (!tacna_can_change_grp_rate(priv, priv->dsp[dsp_num].base, 0)) {
		dev_warn(priv->tacna->dev,
			 "Cannot change '%s' while DSP%u is active\n",
			 kcontrol->id.name, priv->dsp[dsp_num].num);
		ret = -EBUSY;
	} else {
		if (e->mask & TACNA_DSP_RATE_CTL_DIR_MASK)
			priv->dsp[dsp_num].tx_rate_cache[rate_num] =
				e->values[item];
		else
			priv->dsp[dsp_num].rx_rate_cache[rate_num] =
				e->values[item];
		ret = 0;
	}

	snd_soc_dapm_mutex_unlock(dapm);

	return ret;
}
EXPORT_SYMBOL_GPL(tacna_dsp_rate_put);

static const struct soc_enum tacna_dsp_rate_enum[] = {
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (0 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (1 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (2 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (3 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (4 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,  tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (5 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (6 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (7 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (0 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (1 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (2 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (3 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (4 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (5 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (6 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0,
			      (7 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (0 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (1 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (2 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (3 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (4 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text,  tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (5 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (6 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (7 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_RX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (0 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (1 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (2 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (3 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (4 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (5 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (6 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 1,
			      (7 << TACNA_DSP_RATE_CTL_NUM_SHIFT) |
			      TACNA_DSP_RATE_CTL_DIR_TX,
			      TACNA_RATE_ENUM_SIZE,
			      tacna_rate_text, tacna_rate_val),
};

const struct snd_kcontrol_new tacna_dsp_rate_controls[] = {
	SOC_ENUM_EXT("DSP1RX1 Rate", tacna_dsp_rate_enum[0],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1RX2 Rate", tacna_dsp_rate_enum[1],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1RX3 Rate", tacna_dsp_rate_enum[2],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1RX4 Rate", tacna_dsp_rate_enum[3],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1RX5 Rate", tacna_dsp_rate_enum[4],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1RX6 Rate", tacna_dsp_rate_enum[5],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1RX7 Rate", tacna_dsp_rate_enum[6],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1RX8 Rate", tacna_dsp_rate_enum[7],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1TX1 Rate", tacna_dsp_rate_enum[8],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1TX2 Rate", tacna_dsp_rate_enum[9],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1TX3 Rate", tacna_dsp_rate_enum[10],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1TX4 Rate", tacna_dsp_rate_enum[11],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1TX5 Rate", tacna_dsp_rate_enum[12],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1TX6 Rate", tacna_dsp_rate_enum[13],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1TX7 Rate", tacna_dsp_rate_enum[14],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP1TX8 Rate", tacna_dsp_rate_enum[15],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2RX1 Rate", tacna_dsp_rate_enum[16],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2RX2 Rate", tacna_dsp_rate_enum[17],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2RX3 Rate", tacna_dsp_rate_enum[18],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2RX4 Rate", tacna_dsp_rate_enum[19],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2RX5 Rate", tacna_dsp_rate_enum[20],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2RX6 Rate", tacna_dsp_rate_enum[21],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2RX7 Rate", tacna_dsp_rate_enum[22],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2RX8 Rate", tacna_dsp_rate_enum[23],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2TX1 Rate", tacna_dsp_rate_enum[24],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2TX2 Rate", tacna_dsp_rate_enum[25],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2TX3 Rate", tacna_dsp_rate_enum[26],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2TX4 Rate", tacna_dsp_rate_enum[27],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2TX5 Rate", tacna_dsp_rate_enum[28],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2TX6 Rate", tacna_dsp_rate_enum[29],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2TX7 Rate", tacna_dsp_rate_enum[30],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
	SOC_ENUM_EXT("DSP2TX8 Rate", tacna_dsp_rate_enum[31],
		     tacna_dsp_rate_get, tacna_dsp_rate_put),
};
EXPORT_SYMBOL_GPL(tacna_dsp_rate_controls);

static int tacna_dsp_memory_enable(struct tacna_priv *priv,
				   unsigned int dsp_num)
{
	struct regmap *regmap = priv->tacna->regmap;
	const unsigned int *reg_list = priv->dsp_power_regs[dsp_num];
	int ret;

	for (; *reg_list != 0; ++reg_list) {
		ret = regmap_update_bits(regmap, *reg_list, 0x3, 0x1);
		if (ret)
			goto err;

		udelay(1); /* allow bank to power-up */

		ret = regmap_update_bits(regmap, *reg_list, 0x3, 0x3);
		if (ret)
			goto err;

		udelay(1); /* allow bank to power-up */
	}

	return 0;
err:
	dev_err(priv->dev, "Failed to write SRAM enable 0x%x (%d)\n",
		*reg_list, ret);

	return ret;
}

static void tacna_dsp_memory_disable(struct tacna_priv *priv,
				     unsigned int dsp_num)
{
	struct regmap *regmap = priv->tacna->regmap;
	const unsigned int *reg_list = priv->dsp_power_regs[dsp_num];
	int ret;

	for (; *reg_list != 0; ++reg_list) {
		ret = regmap_update_bits(regmap, *reg_list, 0x3, 0);
		if (ret)
			dev_warn(priv->dev,
				 "Failed to write SRAM disable 0x%x (%d)\n",
				 *reg_list, ret);
	}
}

int tacna_dsp_power_ev(struct snd_soc_dapm_widget *w,
		       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = priv->tacna;
	struct wm_adsp *dsps = snd_soc_codec_get_drvdata(codec);
	struct wm_adsp *dsp = &dsps[w->shift];
	unsigned int freq;
	int ret;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		ret = regmap_read(tacna->regmap, TACNA_DSP_CLOCK1, &freq);
		if (ret) {
			dev_err(codec->dev,
				"Failed to read TACNA_DSP_CLOCK1: %d\n", ret);
			return ret;
		}

		freq &= TACNA_DSP_CLK_FREQ_MASK;
		freq >>= TACNA_DSP_CLK_FREQ_SHIFT;
		ret = regmap_write(dsp->regmap,
				   dsp->base + TACNA_DSP_CLOCK_FREQ_OFFS,
				   freq);
		if (ret) {
			dev_err(codec->dev,
				"Failed to set HALO clock freq: %d\n", ret);
			return ret;
		}

		ret = tacna_dsp_memory_enable(priv, w->shift);
		if (ret)
			return ret;

		return wm_halo_early_event(w, kcontrol, event);
	case SND_SOC_DAPM_PRE_PMD:
		ret = wm_halo_early_event(w, kcontrol, event);
		tacna_dsp_memory_disable(priv, w->shift);
		return ret;
	default:
		return 0;
	};
}
EXPORT_SYMBOL_GPL(tacna_dsp_power_ev);

static const unsigned int tacna_opclk_ref_48k_rates[] = {
	6144000,
	12288000,
	24576000,
	49152000,
};

static const unsigned int tacna_opclk_ref_44k1_rates[] = {
	5644800,
	11289600,
	22579200,
	45158400,
};

static int tacna_set_opclk(struct snd_soc_codec *codec, unsigned int clk,
			   unsigned int freq)
{
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	unsigned int reg;
	const unsigned int *rates;
	int ref, div, refclk;

	BUILD_BUG_ON(ARRAY_SIZE(tacna_opclk_ref_48k_rates) !=
		     ARRAY_SIZE(tacna_opclk_ref_44k1_rates));

	switch (clk) {
	case TACNA_CLK_OPCLK:
		reg = TACNA_OUTPUT_SYS_CLK;
		refclk = priv->sysclk;
		break;
	case TACNA_CLK_ASYNC_OPCLK:
		reg = TACNA_OUTPUT_ASYNC_CLK;
		refclk = priv->asyncclk;
		break;
	default:
		return -EINVAL;
	}

	if (refclk % 4000)
		rates = tacna_opclk_ref_44k1_rates;
	else
		rates = tacna_opclk_ref_48k_rates;

	for (ref = 0; ref < ARRAY_SIZE(tacna_opclk_ref_48k_rates); ++ref) {
		if (rates[ref] > refclk)
			continue;

		div = 2;
		while ((rates[ref] / div >= freq) && (div <= 30)) {
			if (rates[ref] / div == freq) {
				dev_dbg(codec->dev, "Configured %dHz OPCLK\n",
					freq);
				snd_soc_update_bits(codec, reg,
						    TACNA_OPCLK_DIV_MASK |
						    TACNA_OPCLK_SEL_MASK,
						    (div <<
						     TACNA_OPCLK_DIV_SHIFT) |
						    ref);
				return 0;
			}
			div += 2;
		}
	}

	dev_err(codec->dev, "Unable to generate %dHz OPCLK\n", freq);
	return -EINVAL;
}

static int tacna_set_dacclk(struct snd_soc_codec *codec, int source,
			    unsigned int freq)
{
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	unsigned int val, div;
	int ret;

	switch (freq) {
	case 5644800:
	case 6144000:
		val = 0;
		div = 0; /* divide by 1 */
		break;
	case 11289600:
	case 12288000:
		val = 1 << TACNA_DAC_CLK_SRC_FREQ_SHIFT;
		div = 1 << TACNA_OUT_CLK_DIV_SHIFT; /* divide by 2 */
		break;
	case 22579200:
	case 24576000:
		val = 2 << TACNA_DAC_CLK_SRC_FREQ_SHIFT;
		div = 2 << TACNA_OUT_CLK_DIV_SHIFT; /* divide by 4 */
		break;
	default:
		dev_err(priv->dev, "invalid DACCLK freq %dHz\n", freq);
		return -EINVAL;
	}

	switch (source) {
	case TACNA_CLK_SRC_MCLK1:
	case TACNA_CLK_SRC_MCLK2:
	case TACNA_CLK_SRC_MCLK3:
	case TACNA_CLK_SRC_FLL1:
	case TACNA_CLK_SRC_FLL2:
	case TACNA_CLK_SRC_FLL3:
		val |= source << TACNA_DAC_CLK_SRC_SEL_SHIFT;
		break;
	default:
		dev_err(priv->dev, "invalid DACCLK src %d\n", source);
		return -EINVAL;
	}

	ret = regmap_update_bits(priv->tacna->regmap,
				 TACNA_DAC_CLK_CONTROL1,
				 TACNA_DAC_CLK_SRC_FREQ_MASK |
				 TACNA_DAC_CLK_SRC_SEL_MASK,
				 val);
	if (ret) {
		dev_err(priv->dev, "Error writing DACCLK control: %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(priv->tacna->regmap,
				 TACNA_OUTPUT_CONTROL_1,
				 TACNA_OUT_CLK_DIV_MASK |
				 TACNA_OUT_CLK_MODE_MASK,
				 div);
	if (ret) {
		dev_err(priv->dev, "Error writing OUTPUT_CONTROL_1 %d\n", ret);
		return ret;
	}

	switch (priv->tacna->type) {
	case CS47L94:
	case CS47L95:
		if (freq % 4000)
			val = TACNA_OUTH_CLK_FRAC; /* 44.1 group rate */
		else
			val = 0;

		ret = regmap_update_bits(priv->tacna->regmap,
					 TACNA_OUTH_CONFIG_1,
					 TACNA_OUTH_CLK_FRAC_MASK,
					 val);
		if (ret) {
			dev_err(priv->dev,
				"Error writing OUTH_CONFIG_1 %d\n", ret);
			return ret;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int tacna_get_dspclk_setting(struct tacna_priv *priv, unsigned int freq,
				    int src, unsigned int *val)
{
	unsigned int div_mask, div_val;
	int ret;

	switch (src) {
	case TACNA_CLK_SRC_FLL1:
		div_mask = TACNA_FLL1_DSPCLK_SEL;
		div_val = 0;
		break;
	case TACNA_CLK_SRC_FLL2:
		div_mask = TACNA_FLL2_DSPCLK_SEL;
		div_val = 0;
		break;
	case TACNA_CLK_SRC_FLL3:
		div_mask = TACNA_FLL3_DSPCLK_SEL;
		div_val = 0;
		break;
	case TACNA_CLK_SRC_FLL1_DSP_DIV2:
		div_mask = TACNA_FLL1_DSPCLK_SEL;
		div_val = 1;
		break;
	case TACNA_CLK_SRC_FLL2_DSP_DIV2:
		div_mask = TACNA_FLL2_DSPCLK_SEL;
		div_val = 1;
		break;
	case TACNA_CLK_SRC_FLL3_DSP_DIV2:
		div_mask = TACNA_FLL3_DSPCLK_SEL;
		div_val = 1;
		break;
	default:
		div_mask = 0;
		break;
	}

	if (div_mask) {
		ret = regmap_update_bits(priv->tacna->regmap,
					 TACNA_FLL_DSP_CTRL,
					 div_mask, div_val);
		if (ret) {
			dev_err(priv->dev,
				"Failed to write to FLL_DSP_CTRL: %d\n", ret);
			return ret;
		}
	}

	freq /= 15625; /* convert to 1/64ths of 1MHz */

	/* This can go to bit 31 so must return as a unsigned int */
	*val |= freq << TACNA_DSP_CLK_FREQ_SHIFT;

	return 0;
}

static int tacna_get_sysclk_setting(unsigned int freq)
{
	switch (freq) {
	case 0:
	case 5644800:
	case 6144000:
		return TACNA_SYSCLK_RATE_6MHZ;
	case 11289600:
	case 12288000:
		return TACNA_SYSCLK_RATE_12MHZ << TACNA_SYSCLK_FREQ_SHIFT;
	case 22579200:
	case 24576000:
		return TACNA_SYSCLK_RATE_24MHZ << TACNA_SYSCLK_FREQ_SHIFT;
	case 45158400:
	case 49152000:
		return TACNA_SYSCLK_RATE_49MHZ << TACNA_SYSCLK_FREQ_SHIFT;
	case 90316800:
	case 98304000:
		return TACNA_SYSCLK_RATE_98MHZ << TACNA_SYSCLK_FREQ_SHIFT;
	default:
		return -EINVAL;
	}
}

int tacna_set_sysclk(struct snd_soc_codec *codec, int clk_id, int source,
		     unsigned int freq, int dir)
{
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = priv->tacna;
	char *name;
	unsigned int reg;
	unsigned int mask = TACNA_SYSCLK_SRC_MASK;
	unsigned int val = source << TACNA_SYSCLK_SRC_SHIFT;
	int clk_freq_sel, *clk;

	switch (clk_id) {
	case TACNA_CLK_SYSCLK_1:
		name = "SYSCLK";
		reg = TACNA_SYSTEM_CLOCK1;
		clk = &priv->sysclk;
		clk_freq_sel = tacna_get_sysclk_setting(freq);
		mask |= TACNA_SYSCLK_FREQ_MASK | TACNA_SYSCLK_FRAC;
		break;
	case TACNA_CLK_ASYNCCLK_1:
		name = "ASYNCCLK";
		reg = TACNA_ASYNC_CLOCK1;
		clk = &priv->asyncclk;
		clk_freq_sel = tacna_get_sysclk_setting(freq);
		mask |= TACNA_SYSCLK_FREQ_MASK;
		break;
	case TACNA_CLK_DSPCLK:
		name = "DSPCLK";
		reg = TACNA_DSP_CLOCK1;
		clk = &priv->dspclk;
		clk_freq_sel = tacna_get_dspclk_setting(priv, freq,
							source, &val);
		mask |= TACNA_DSP_CLK_FREQ_MASK;
		break;
	case TACNA_CLK_OPCLK:
	case TACNA_CLK_ASYNC_OPCLK:
		return tacna_set_opclk(codec, clk_id, freq);
	case TACNA_CLK_DACCLK:
		return tacna_set_dacclk(codec, source, freq);
	default:
		return -EINVAL;
	}

	if (clk_freq_sel < 0) {
		dev_err(tacna->dev,
			"Failed to get %s setting for %dHZ\n", name, freq);
		return clk_freq_sel;
	}

	*clk = freq;

	if (freq == 0) {
		dev_dbg(tacna->dev, "%s cleared\n", name);
		return 0;
	}

	val |= clk_freq_sel;

	if (freq % 6144000)
		val |= TACNA_SYSCLK_FRAC;

	dev_dbg(tacna->dev, "%s set to %uHz", name, freq);

	return regmap_update_bits(tacna->regmap, reg, mask, val);
}
EXPORT_SYMBOL_GPL(tacna_set_sysclk);

static int tacna_is_enabled_fll(struct tacna_fll *fll, int base)
{
	struct tacna *tacna = fll->tacna;
	unsigned int reg;
	int ret;

	ret = regmap_read(tacna->regmap, base + TACNA_FLL_CONTROL1_OFFS, &reg);
	if (ret != 0) {
		tacna_fll_err(fll, "Failed to read current state: %d\n", ret);
		return ret;
	}

	return reg & TACNA_FLL1_EN;
}

static int tacna_wait_for_fll(struct tacna_fll *fll, bool requested)
{
	struct tacna *tacna = fll->tacna;
	unsigned int val = 0;
	int i;

	tacna_fll_dbg(fll, "Waiting for FLL...\n");

	for (i = 0; i < 30; i++) {
		regmap_read(tacna->regmap, fll->sts_addr, &val);
		if (!!(val & fll->sts_mask) == requested)
			return 0;

		switch (i) {
		case 0 ... 5:
			usleep_range(75, 125);
			break;
		case 11 ... 20:
			usleep_range(750, 1250);
			break;
		default:
			msleep(20);
			break;
		}
	}

	tacna_fll_warn(fll, "Timed out waiting for lock\n");

	return -ETIMEDOUT;
}

static int tacna_fllhj_disable(struct tacna_fll *fll)
{
	struct tacna *tacna = fll->tacna;
	bool change;

	tacna_fll_dbg(fll, "Disabling FLL\n");

	/*
	 * Disable lockdet, but don't set ctrl_upd update but. This allows the
	 * lock status bit to clear as normal, but should the FLL be enabled
	 * again due to a control clock being required, the lock won't re-assert
	 * as the FLL config registers are automatically applied when the FLL
	 * enables.
	 */
	regmap_update_bits(tacna->regmap,
			   fll->base + TACNA_FLL_CONTROL1_OFFS,
			   TACNA_FLL1_HOLD_MASK,
			   TACNA_FLL1_HOLD_MASK);
	regmap_update_bits(tacna->regmap,
			   fll->base + TACNA_FLL_CONTROL2_OFFS,
			   TACNA_FLL1_LOCKDET_MASK,
			   0);
	regmap_update_bits(tacna->regmap,
			   fll->base + TACNA_FLL_CONTROL5_OFFS,
			   TACNA_FLL1_FRC_INTEG_UPD_MASK,
			   TACNA_FLL1_FRC_INTEG_UPD);
	regmap_update_bits_check(tacna->regmap,
				 fll->base + TACNA_FLL_CONTROL1_OFFS,
				 TACNA_FLL1_EN_MASK,
				 0,
				 &change);

	tacna_wait_for_fll(fll, false);

	/*
	 * ctrl_up gates the writes to all the fll's registers, setting it to 0
	 * here ensures that after a runtime suspend/resume cycle when one
	 * enables the fll then ctrl_up is the last bit that is configured
	 * by the fll enable code rather than the cache sync operation which
	 * would have updated it much earlier before writing out all fll
	 * registers
	 */
	regmap_update_bits(tacna->regmap,
			   fll->base + TACNA_FLL_CONTROL1_OFFS,
			   TACNA_FLL1_CTRL_UPD_MASK,
			   0);

	if (change)
		pm_runtime_put_autosuspend(tacna->dev);

	return 0;
}

static int tacna_fllhj_apply(struct tacna_fll *fll, int fin)
{
	struct tacna *tacna = fll->tacna;
	int refdiv, fref, fout, lockdet_thr, fbdiv, fast_clk, fllgcd;
	bool frac = false;
	unsigned int fll_n, min_n, max_n, ratio, theta, lambda, hp, ord2;
	unsigned int gains, num;

	tacna_fll_dbg(fll, "fin=%d, fout=%d\n", fin, fll->fout);

	for (refdiv = 0; refdiv < 4; refdiv++)
		if ((fin / (1 << refdiv)) <= TACNA_FLLHJ_MAX_THRESH)
			break;

	fref = fin / (1 << refdiv);

	/*
	 * Use simple heuristic approach to find a configuration that
	 * should work for most input clocks.
	 */
	fast_clk = 0;
	fout = fll->fout;
	frac = fout % fref;

	if (fref < TACNA_FLLHJ_LOW_THRESH) {
		lockdet_thr = 2;
		gains = TACNA_FLLHJ_LOW_GAINS;
		if (frac)
			fbdiv = 256;
		else
			fbdiv = 4;
	} else if (fref < TACNA_FLLHJ_MID_THRESH) {
		lockdet_thr = 8;
		gains = TACNA_FLLHJ_MID_GAINS;
		fbdiv = (frac) ? 16 : 2;
	} else {
		lockdet_thr = 8;
		gains = TACNA_FLLHJ_HIGH_GAINS;
		fbdiv = 1;
		/*
		 * For high speed input clocks, enable 300MHz fast oscillator
		 * when we're in fractional divider mode.
		 */
		if (frac) {
			fast_clk = 0x3;
			fout = fll->fout * 6;
		}
	}
	/* Use high performance mode for fractional configurations. */
	if (frac) {
		hp = 0x3;
		ord2 = 1;
		min_n = TACNA_FLLHJ_FRAC_MIN_N;
		max_n = TACNA_FLLHJ_FRAC_MAX_N;
	} else {
		hp = 0x1;
		ord2 = 0;
		min_n = TACNA_FLLHJ_INT_MIN_N;
		max_n = TACNA_FLLHJ_INT_MAX_N;
	}

	ratio = fout / fref;

	tacna_fll_dbg(fll, "refdiv=%d, fref=%d, frac:%d\n",
		      refdiv, fref, frac);

	while (ratio / fbdiv < min_n) {
		fbdiv /= 2;
		if (fbdiv < min_n) {
			tacna_fll_err(fll, "FBDIV (%u) < minimum N (%u)\n",
				      fbdiv, min_n);
			return -EINVAL;
		}
	}
	while (frac && (ratio / fbdiv > max_n)) {
		fbdiv *= 2;
		if (fbdiv >= 1024) {
			tacna_fll_err(fll, "FBDIV (%u) >= 1024\n", fbdiv);
			return -EINVAL;
		}
	}

	tacna_fll_dbg(fll, "lockdet=%d, hp=0x%x, ord2=0x%x fbdiv:%d\n",
		       lockdet_thr, hp, ord2, fbdiv);

	/* Calculate N.K values */
	fllgcd = gcd(fout, fbdiv * fref);
	num = fout / fllgcd;
	lambda = (fref * fbdiv) / fllgcd;
	fll_n = num / lambda;
	theta = num % lambda;

	tacna_fll_dbg(fll, "fll_n=%d, gcd=%d, theta=%d, lambda=%d\n",
		      fll_n, fllgcd, theta, lambda);

	/* Some sanity checks before any registers are written. */
	if (fll_n < min_n || fll_n > max_n) {
		tacna_fll_err(fll, "N not in valid %s mode range %d-%d: %d\n",
			      frac ? "fractional" : "integer", min_n, max_n,
			      fll_n);
		return -EINVAL;
	}
	if (fbdiv < 1 || (frac && fbdiv >= 1024) || (!frac && fbdiv >= 256)) {
		tacna_fll_err(fll, "Invalid fbdiv for %s mode (%u)\n",
			      frac ? "fractional" : "integer", fbdiv);
		return -EINVAL;
	}

	/* clear the ctrl_upd bit to guarantee we write to it later. */
	regmap_update_bits(tacna->regmap,
			   fll->base + TACNA_FLL_CONTROL2_OFFS,
			   TACNA_FLL1_LOCKDET_THR_MASK |
			   TACNA_FLL1_CLK_VCO_FAST_SRC_MASK |
			   TACNA_FLL1_PHASEDET_MASK |
			   TACNA_FLL1_REFCLK_DIV_MASK |
			   TACNA_FLL1_N_MASK |
			   TACNA_FLL1_CTRL_UPD_MASK,
			   (lockdet_thr << TACNA_FLL1_LOCKDET_THR_SHIFT) |
			   (fast_clk << TACNA_FLL1_CLK_VCO_FAST_SRC_SHIFT) |
			   (1 << TACNA_FLL1_PHASEDET_SHIFT) |
			   (refdiv << TACNA_FLL1_REFCLK_DIV_SHIFT) |
			   (fll_n << TACNA_FLL1_N_SHIFT));

	regmap_update_bits(tacna->regmap,
			   fll->base + TACNA_FLL_CONTROL3_OFFS,
			   TACNA_FLL1_LAMBDA_MASK |
			   TACNA_FLL1_THETA_MASK,
			   (lambda << TACNA_FLL1_LAMBDA_SHIFT) |
			   (theta << TACNA_FLL1_THETA_SHIFT));

	regmap_update_bits(tacna->regmap,
			   fll->base + TACNA_FLL_CONTROL4_OFFS,
			   (0xffff << TACNA_FLL1_FD_GAIN_COARSE_SHIFT) |
			   TACNA_FLL1_HP_MASK |
			   TACNA_FLL1_FB_DIV_MASK,
			   (gains << TACNA_FLL1_FD_GAIN_COARSE_SHIFT) |
			   (hp << TACNA_FLL1_HP_SHIFT) |
			   (fbdiv << TACNA_FLL1_FB_DIV_SHIFT));
	regmap_update_bits(tacna->regmap,
			   fll->base + TACNA_FLL_DIGITAL_TEST2_OFFS,
			   TACNA_FLL1_FB_DIV_SDM_ORD2_EN_MASK,
			   ord2 << TACNA_FLL1_FB_DIV_SDM_ORD2_EN_SHIFT);

	return 0;
}

static int tacna_fllhj_enable(struct tacna_fll *fll)
{
	struct tacna *tacna = fll->tacna;
	int already_enabled = tacna_is_enabled_fll(fll, fll->base);
	int ret;

	if (already_enabled < 0)
		return already_enabled;

	if (!already_enabled)
		pm_runtime_get_sync(tacna->dev);

	tacna_fll_dbg(fll, "Enabling FLL, initially %s\n",
		      already_enabled ? "enabled" : "disabled");

	/* FLLn_HOLD must be set before configuring any registers */
	regmap_update_bits(fll->tacna->regmap,
			   fll->base + TACNA_FLL_CONTROL1_OFFS,
			   TACNA_FLL1_HOLD_MASK,
			   TACNA_FLL1_HOLD_MASK);

	/* Apply refclk */
	ret = tacna_fllhj_apply(fll, fll->ref_freq);
	if (ret) {
		tacna_fll_err(fll, "Failed to set FLL: %d\n", ret);
		goto out;
	}
	regmap_update_bits(tacna->regmap,
			   fll->base + TACNA_FLL_CONTROL2_OFFS,
			   TACNA_FLL1_REFCLK_SRC_MASK,
			   fll->ref_src << TACNA_FLL1_REFCLK_SRC_SHIFT);

	regmap_update_bits(tacna->regmap,
			   fll->base + TACNA_FLL_CONTROL1_OFFS,
			   TACNA_FLL1_EN_MASK,
			   TACNA_FLL1_EN_MASK);

out:
	regmap_update_bits(tacna->regmap,
			   fll->base + TACNA_FLL_CONTROL2_OFFS,
			   TACNA_FLL1_LOCKDET_MASK,
			   TACNA_FLL1_LOCKDET_MASK);

	regmap_update_bits(tacna->regmap,
			   fll->base + TACNA_FLL_CONTROL1_OFFS,
			   TACNA_FLL1_CTRL_UPD_MASK,
			   TACNA_FLL1_CTRL_UPD_MASK);

	/* Release the hold so that flln locks to external frequency */
	regmap_update_bits(tacna->regmap,
			   fll->base + TACNA_FLL_CONTROL1_OFFS,
			   TACNA_FLL1_HOLD_MASK,
			   0);

	if (!already_enabled)
		tacna_wait_for_fll(fll, true);

	return 0;
}

static int tacna_fllhj_validate(struct tacna_fll *fll,
				unsigned int ref_in,
				unsigned int fout)
{
	if (fout && !ref_in) {
		tacna_fll_err(fll, "fllout set without valid input clk\n");
		return -EINVAL;
	}

	if (fll->fout && fout != fll->fout) {
		tacna_fll_err(fll, "Can't change output on active FLL\n");
		return -EINVAL;
	}

	if (ref_in / TACNA_FLL_MAX_REFDIV > TACNA_FLLHJ_MAX_THRESH) {
		tacna_fll_err(fll, "Can't scale %dMHz to <=13MHz\n", ref_in);
		return -EINVAL;
	}

	if (fout > TACNA_FLL_MAX_FOUT) {
		tacna_fll_err(fll, "Fout=%dMHz exceeeds maximum %dMHz\n",
			      fout, TACNA_FLL_MAX_FOUT);
		return -EINVAL;
	}

	return 0;
}

int tacna_fllhj_set_refclk(struct tacna_fll *fll, int source,
			   unsigned int fin, unsigned int fout)
{
	int ret = 0;

	if (fll->ref_src == source && fll->ref_freq == fin &&
	    fll->fout == fout)
		return 0;

	if (fin && fout && tacna_fllhj_validate(fll, fin, fout))
		return -EINVAL;

	fll->ref_src = source;
	fll->ref_freq = fin;
	fll->fout = fout;

	if (fout)
		ret = tacna_fllhj_enable(fll);
	else
		tacna_fllhj_disable(fll);

	return ret;
}
EXPORT_SYMBOL_GPL(tacna_fllhj_set_refclk);

static int tacna_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = priv->tacna;
	unsigned int val = 0u;
	unsigned int base = dai->driver->base;
	unsigned int mask = TACNA_ASP1_FMT_MASK | TACNA_ASP1_BCLK_INV_MASK |
			    TACNA_ASP1_BCLK_MSTR_MASK |
			    TACNA_ASP1_FSYNC_INV_MASK |
			    TACNA_ASP1_FSYNC_MSTR_MASK;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		val |= (TACNA_ASP_FMT_DSP_MODE_A << TACNA_ASP1_FMT_SHIFT);
		break;
	case SND_SOC_DAIFMT_DSP_B:
		if ((fmt & SND_SOC_DAIFMT_MASTER_MASK)
				!= SND_SOC_DAIFMT_CBM_CFM) {
			tacna_asp_err(dai, "DSP_B not valid in slave mode\n");
			return -EINVAL;
		}
		val |= (TACNA_ASP_FMT_DSP_MODE_B << TACNA_ASP1_FMT_SHIFT);
		break;
	case SND_SOC_DAIFMT_I2S:
		val |= (TACNA_ASP_FMT_I2S_MODE << TACNA_ASP1_FMT_SHIFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		if ((fmt & SND_SOC_DAIFMT_MASTER_MASK)
				!= SND_SOC_DAIFMT_CBM_CFM) {
			tacna_asp_err(dai, "LEFT_J not valid in slave mode\n");
			return -EINVAL;
		}
		val |= (TACNA_ASP_FMT_LEFT_JUSTIFIED_MODE <<
			TACNA_ASP1_FMT_SHIFT);
		break;
	default:
		tacna_asp_err(dai, "Unsupported DAI format %d\n",
			      fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		val |= TACNA_ASP1_FSYNC_MSTR;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		val |= TACNA_ASP1_BCLK_MSTR;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		val |= TACNA_ASP1_BCLK_MSTR;
		val |= TACNA_ASP1_FSYNC_MSTR;
		break;
	default:
		tacna_asp_err(dai, "Unsupported master mode %d\n",
			      fmt & SND_SOC_DAIFMT_MASTER_MASK);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		val |= TACNA_ASP1_BCLK_INV;
		val |= TACNA_ASP1_FSYNC_INV;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		val |= TACNA_ASP1_BCLK_INV;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		val |= TACNA_ASP1_FSYNC_INV;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(tacna->regmap,
			   base + TACNA_ASP_CONTROL2,
			   mask,
			   val);

	return 0;
}

static const int tacna_sclk_rates[] = {
	-1,      /* 0 */
	-1,      /* 1 */
	-1,      /* 2 */
	-1,      /* 3 */
	-1,      /* 4 */
	-1,      /* 5 */
	-1,      /* 6 */
	-1,      /* 7 */
	-1,      /* 8 */
	-1,      /* 9 */
	-1,      /* 10 */
	-1,      /* 11 */
	128000,  /* 12 */
	176400,  /* 13 */
	192000,  /* 14 */
	256000,  /* 15 */
	352800,  /* 16 */
	384000,  /* 17 */
	512000,  /* 18 */
	705600,  /* 19 */
	-1,      /* 20 */
	768000,  /* 21 */
	-1,      /* 22 */
	1024000, /* 23 */
	-1,      /* 24 */
	1411200, /* 25 */
	-1,      /* 26 */
	1536000, /* 27 */
	-1,      /* 28 */
	2048000, /* 29 */
	-1,      /* 30 */
	2822400, /* 31 */
	-1,      /* 32 */
	3072000, /* 33 */
	-1,      /* 34 */
	-1,      /* 35 */
	4096000, /* 36 */
	-1,      /* 37 */
	5644800, /* 38 */
	-1,      /* 39 */
	6144000, /* 40 */
	-1,      /* 41 */
	-1,      /* 42 */
	-1,      /* 43 */
	-1,      /* 44 */
	-1,      /* 45 */
	-1,      /* 46 */
	8192000, /* 47 */
	-1,      /* 48 */
	11289600,/* 49 */
	-1,      /* 50 */
	12288000,/* 51 */
	-1,      /* 52 */
	-1,      /* 53 */
	-1,      /* 54 */
	-1,      /* 55 */
	-1,      /* 56 */
	22579200,/* 57 */
	-1,      /* 58 */
	24576000,/* 59 */
	-1,      /* 60 */
	-1,      /* 61 */
	-1,      /* 62 */
	-1,      /* 63 */
};

#define TACNA_48K_RATE_MASK	0x0e00fe
#define TACNA_44K1_RATE_MASK	0x00fe00
#define TACNA_RATE_MASK		(TACNA_48K_RATE_MASK | TACNA_44K1_RATE_MASK)

static const unsigned int tacna_sr_vals[] = {
	0,
	12000,  /* TACNA_48K_RATE_MASK */
	24000,  /* TACNA_48K_RATE_MASK */
	48000,  /* TACNA_48K_RATE_MASK */
	96000,  /* TACNA_48K_RATE_MASK */
	192000, /* TACNA_48K_RATE_MASK */
	384000, /* TACNA_48K_RATE_MASK */
	768000, /* TACNA_48K_RATE_MASK */
	0,
	11025,  /* TACNA_44K1_RATE_MASK */
	22050,  /* TACNA_44K1_RATE_MASK */
	44100,  /* TACNA_44K1_RATE_MASK */
	88200,  /* TACNA_44K1_RATE_MASK */
	176400, /* TACNA_44K1_RATE_MASK */
	352800, /* TACNA_44K1_RATE_MASK */
	705600, /* TACNA_44K1_RATE_MASK */
	0,
	8000,   /* TACNA_48K_RATE_MASK */
	16000,  /* TACNA_48K_RATE_MASK */
	32000,  /* TACNA_48K_RATE_MASK */
};

static const struct snd_pcm_hw_constraint_list tacna_constraint = {
	.count	= ARRAY_SIZE(tacna_sr_vals),
	.list	= tacna_sr_vals,
};

static int tacna_startup(struct snd_pcm_substream *substream,
			 struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna_dai_priv *dai_priv = &priv->dai[dai->id - 1];
	unsigned int base_rate;

	if (!substream->runtime)
		return 0;

	switch (dai_priv->clk) {
	case TACNA_CLK_SYSCLK_1:
	case TACNA_CLK_SYSCLK_2:
	case TACNA_CLK_SYSCLK_3:
		base_rate = priv->sysclk;
		break;
	case TACNA_CLK_ASYNCCLK_1:
	case TACNA_CLK_ASYNCCLK_2:
		base_rate = priv->asyncclk;
		break;
	default:
		return 0;
	}

	if (base_rate == 0)
		dai_priv->constraint.mask = TACNA_RATE_MASK;
	else if (base_rate % 4000)
		dai_priv->constraint.mask = TACNA_44K1_RATE_MASK;
	else
		dai_priv->constraint.mask = TACNA_48K_RATE_MASK;

	return snd_pcm_hw_constraint_list(substream->runtime, 0,
					  SNDRV_PCM_HW_PARAM_RATE,
					  &dai_priv->constraint);
}

static int tacna_hw_params_rate(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna_dai_priv *dai_priv = &priv->dai[dai->id - 1];
	unsigned int rate_addr = dai->driver->base + TACNA_ASP_CONTROL1;
	int ret = 0;
	unsigned int i, sr_val, sr_reg, sr_mask;
	unsigned int cur_asp_rate, tar_asp_rate, rate;
	bool change_rate_domain = false;

	rate = params_rate(params);
	for (i = 0; i < ARRAY_SIZE(tacna_sr_vals); i++)
		if (tacna_sr_vals[i] == rate)
			break;

	if (i == ARRAY_SIZE(tacna_sr_vals)) {
		tacna_asp_err(dai, "Unsupported sample rate %dHz\n", rate);
		return -EINVAL;
	}
	sr_val = i;

	switch (dai_priv->clk) {
	case TACNA_CLK_SYSCLK_1:
		tar_asp_rate = 0u << TACNA_ASP1_RATE_SHIFT;
		sr_reg = TACNA_SAMPLE_RATE1;
		sr_mask = TACNA_SAMPLE_RATE_1_MASK;
		break;
	case TACNA_CLK_SYSCLK_2:
		tar_asp_rate = 1u << TACNA_ASP1_RATE_SHIFT;
		sr_reg = TACNA_SAMPLE_RATE2;
		sr_mask = TACNA_SAMPLE_RATE_2_MASK;
		break;
	case TACNA_CLK_SYSCLK_3:
		tar_asp_rate = 2u << TACNA_ASP1_RATE_SHIFT;
		sr_reg = TACNA_SAMPLE_RATE3;
		sr_mask = TACNA_SAMPLE_RATE_3_MASK;
		break;
	case TACNA_CLK_ASYNCCLK_1:
		tar_asp_rate = 8u << TACNA_ASP1_RATE_SHIFT;
		sr_reg = TACNA_ASYNC_SAMPLE_RATE1;
		sr_mask = TACNA_ASYNC_SAMPLE_RATE_1_MASK;
		break;
	case TACNA_CLK_ASYNCCLK_2:
		tar_asp_rate = 9u << TACNA_ASP1_RATE_SHIFT;
		sr_reg = TACNA_ASYNC_SAMPLE_RATE2;
		sr_mask = TACNA_ASYNC_SAMPLE_RATE_2_MASK;
		break;
	default:
		return -EINVAL;
	}

	if (rate_addr) {
		ret = regmap_read(priv->tacna->regmap, rate_addr,
				  &cur_asp_rate);
		if (ret != 0) {
			tacna_asp_err(dai, "Failed to check rate: %d\n", ret);
			return ret;
		}

		if ((cur_asp_rate & TACNA_ASP1_RATE_MASK) !=
		    (tar_asp_rate & TACNA_ASP1_RATE_MASK)) {
			change_rate_domain = true;

			mutex_lock(&priv->rate_lock);

			if (!tacna_can_change_grp_rate(priv, rate_addr, 0)) {
				tacna_asp_warn(dai,
					       "Cannot change rate while active\n");
				ret = -EBUSY;
				goto out;
			}

			/* Guard the rate change with SYSCLK cycles */
			tacna_spin_sysclk(priv);
		}
	}

	snd_soc_update_bits(codec, sr_reg, sr_mask, sr_val);
	if (rate_addr)
		snd_soc_update_bits(codec, rate_addr,
				    TACNA_ASP1_RATE_MASK,
				    tar_asp_rate);

out:
	if (change_rate_domain) {
		tacna_spin_sysclk(priv);
		mutex_unlock(&priv->rate_lock);
	}

	return ret;
}

static bool tacna_asp_cfg_changed(struct snd_soc_codec *codec,
				  unsigned int base, unsigned int sclk,
				  unsigned int slotws, unsigned int dataw)
{
	unsigned int val;

	val = snd_soc_read(codec, base + TACNA_ASP_CONTROL1);
	if (sclk != (val & TACNA_ASP1_BCLK_FREQ_MASK))
		return true;

	val = snd_soc_read(codec, base + TACNA_ASP_CONTROL2);
	if (slotws != (val & (TACNA_ASP1_RX_WIDTH_MASK |
			      TACNA_ASP1_TX_WIDTH_MASK)))
		return true;

	val = snd_soc_read(codec, base + TACNA_ASP_DATA_CONTROL1);
	if (dataw != (val & (TACNA_ASP1_TX_WL_MASK)))
		return true;

	val = snd_soc_read(codec, base + TACNA_ASP_DATA_CONTROL5);
	if (dataw != (val & (TACNA_ASP1_RX_WL_MASK)))
		return true;

	return false;
}

static int tacna_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = priv->tacna;
	int base = dai->driver->base;
	const int *rates;
	int i, ret, val, rates_sz;
	unsigned int rate = params_rate(params);
	unsigned int channels = params_channels(params);
	unsigned int chan_limit =
			tacna->pdata.codec.max_channels_clocked[dai->id - 1];
	int dai_id = dai->id - 1;
	int tdm_width = priv->tdm_width[dai_id];
	int tdm_slots = priv->tdm_slots[dai_id];
	int sclk, sclk_target;
	bool reconfig;
	unsigned int asp_state = 0;
	unsigned int slotw, dataw;

	rates = &tacna_sclk_rates[0];
	rates_sz = ARRAY_SIZE(tacna_sclk_rates);

	/*
	 * NOTE: the following calculations hold only under the assumption that
	 *       symmetric_[rates|channels|samplebits] are set to 1
	 */
	slotw = snd_pcm_format_physical_width(params_format(params));
	dataw = snd_pcm_format_width(params_format(params));

	if (tdm_slots) {
		tacna_asp_dbg(dai, "Configuring for %d %d bit TDM slots\n",
			      tdm_slots, tdm_width);
		slotw = tdm_width;
		channels = tdm_slots;
	}

	sclk_target = slotw * rate * channels;

	if (chan_limit && chan_limit < channels) {
		tacna_asp_dbg(dai, "Limiting to %d channels\n", chan_limit);
		sclk_target /= channels;
		sclk_target *= chan_limit;
	}

	/* Force multiple of 2 channels for I2S mode */
	val = snd_soc_read(codec, base + TACNA_ASP_CONTROL2);
	val = (val & TACNA_ASP1_FMT_MASK) >> TACNA_ASP1_FMT_SHIFT;
	if ((channels & 1) && (val == TACNA_ASP_FMT_I2S_MODE)) {
		tacna_asp_dbg(dai, "Forcing stereo mode\n");
		sclk_target /= channels;
		sclk_target *= channels + 1;
	}

	for (i = 0; i < rates_sz; i++) {
		if (rates[i] >= sclk_target && rates[i] % rate == 0) {
			sclk = i;
			break;
		}
	}
	if (i == rates_sz) {
		tacna_asp_err(dai, "Unsupported sample rate %dHz\n", rate);
		return -EINVAL;
	}
	tacna_asp_dbg(dai, "SCLK %dHz\n", rates[sclk]);

	slotw = (slotw << TACNA_ASP1_TX_WIDTH_SHIFT) |
		(slotw << TACNA_ASP1_RX_WIDTH_SHIFT);

	reconfig = tacna_asp_cfg_changed(codec, base, sclk, slotw, dataw);

	if (reconfig) {
		/* Save ASP TX/RX state */
		asp_state = snd_soc_read(codec, base + TACNA_ASP_ENABLES1);
		/* Disable ASP TX/RX before reconfiguring it */
		regmap_update_bits(tacna->regmap,
				   base + TACNA_ASP_ENABLES1,
				   0xff00ff,
				   0x0);
	}

	ret = tacna_hw_params_rate(substream, params, dai);
	if (ret != 0)
		goto restore_asp;

	if (reconfig) {
		regmap_update_bits_async(tacna->regmap,
					 base + TACNA_ASP_CONTROL1,
					 TACNA_ASP1_BCLK_FREQ_MASK,
					 sclk);
		regmap_update_bits_async(tacna->regmap,
					 base + TACNA_ASP_CONTROL2,
					 TACNA_ASP1_RX_WIDTH_MASK |
					 TACNA_ASP1_TX_WIDTH_MASK,
					 slotw);
		regmap_update_bits_async(tacna->regmap,
					 base + TACNA_ASP_DATA_CONTROL1,
					 TACNA_ASP1_TX_WL_MASK,
					 dataw);
		regmap_update_bits(tacna->regmap,
				   base + TACNA_ASP_DATA_CONTROL5,
				   TACNA_ASP1_RX_WL_MASK,
				   dataw);
	}

restore_asp:
	if (reconfig) {
		/* Restore ASP TX/RX state */
		regmap_update_bits(tacna->regmap,
				   base + TACNA_ASP_ENABLES1,
				   0xff00ff,
				   asp_state);
	}
	return ret;
}

static const char * const tacna_dai_clk_str(int clk_id)
{
	switch (clk_id) {
	case TACNA_CLK_SYSCLK_1:
	case TACNA_CLK_SYSCLK_2:
	case TACNA_CLK_SYSCLK_3:
		return "SYSCLK";
	case TACNA_CLK_ASYNCCLK_1:
	case TACNA_CLK_ASYNCCLK_2:
		return "ASYNCCLK";
	default:
		return "Unknown clock";
	}
}

static int tacna_dai_set_sysclk(struct snd_soc_dai *dai,
				int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna_dai_priv *dai_priv = &priv->dai[dai->id - 1];
	struct snd_soc_dapm_route routes[2];

	switch (clk_id) {
	case TACNA_CLK_SYSCLK_1:
	case TACNA_CLK_SYSCLK_2:
	case TACNA_CLK_SYSCLK_3:
	case TACNA_CLK_ASYNCCLK_1:
	case TACNA_CLK_ASYNCCLK_2:
		break;
	default:
		return -EINVAL;
	}

	if (clk_id == dai_priv->clk)
		return 0;

	if (dai->active) {
		dev_err(codec->dev, "Can't change clock on active DAI %d\n",
			dai->id);
		return -EBUSY;
	}

	dev_dbg(codec->dev, "Setting ASP%d to %s\n", dai->id + 1,
		tacna_dai_clk_str(clk_id));

	memset(&routes, 0, sizeof(routes));
	routes[0].sink = dai->driver->capture.stream_name;
	routes[1].sink = dai->driver->playback.stream_name;

	switch (clk_id) {
	case TACNA_CLK_SYSCLK_1:
	case TACNA_CLK_SYSCLK_2:
	case TACNA_CLK_SYSCLK_3:
		routes[0].source = tacna_dai_clk_str(dai_priv->clk);
		routes[1].source = tacna_dai_clk_str(dai_priv->clk);
		snd_soc_dapm_del_routes(dapm, routes, ARRAY_SIZE(routes));
		break;
	default:
		break;
	}

	switch (clk_id) {
	case TACNA_CLK_ASYNCCLK_1:
	case TACNA_CLK_ASYNCCLK_2:
		routes[0].source = tacna_dai_clk_str(clk_id);
		routes[1].source = tacna_dai_clk_str(clk_id);
		snd_soc_dapm_add_routes(dapm, routes, ARRAY_SIZE(routes));
		break;
	default:
		break;
	}

	dai_priv->clk = clk_id;

	return snd_soc_dapm_sync(dapm);
}

static void tacna_set_channels_to_mask(struct snd_soc_dai *dai,
				       unsigned int base,
				       int channels, unsigned int mask)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = priv->tacna;
	int slot, i, j = 0, shift;
	unsigned int frame_ctls[2] = {0u, 0u};

	for (i = 0; i < channels; ++i) {
		slot = ffs(mask) - 1;
		if (slot < 0)
			return;

		if (i - (j * 4) >= 4) {
			++j;
			if (j >= 22)
				break;
		}

		shift = (8 * (i - j * 4));

		frame_ctls[j] |= slot << shift;

		mask &= ~(1 << slot); /* ? mask ^= 1 << slot ? */
	}

	regmap_write(tacna->regmap, base, frame_ctls[0]);
	regmap_write(tacna->regmap, base + 0x4, frame_ctls[1]);

	if (mask)
		tacna_asp_warn(dai, "Too many channels in TDM mask\n");
}

static int tacna_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
			      unsigned int rx_mask, int slots, int slot_width)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	int base = dai->driver->base;
	int rx_max_chan = dai->driver->playback.channels_max;
	int tx_max_chan = dai->driver->capture.channels_max;

	/* Only support TDM for the physical ASPs */
	if (dai->id > TACNA_MAX_ASP)
		return -ENOTSUPP;

	if (slots == 0) {
		tx_mask = (1 << tx_max_chan) - 1;
		rx_mask = (1 << rx_max_chan) - 1;
	}

	tacna_set_channels_to_mask(dai, base + TACNA_ASP_FRAME_CONTROL1,
				   tx_max_chan, tx_mask);
	tacna_set_channels_to_mask(dai, base + TACNA_ASP_FRAME_CONTROL5,
				   rx_max_chan, rx_mask);

	priv->tdm_width[dai->id - 1] = slot_width;
	priv->tdm_slots[dai->id - 1] = slots;

	return 0;
}

const struct snd_soc_dai_ops tacna_dai_ops = {
	.startup = tacna_startup,
	.set_fmt = tacna_set_fmt,
	.set_tdm_slot = tacna_set_tdm_slot,
	.hw_params = tacna_hw_params,
	.set_sysclk = tacna_dai_set_sysclk,
};
EXPORT_SYMBOL_GPL(tacna_dai_ops);

const struct snd_soc_dai_ops tacna_simple_dai_ops = {
	.startup = tacna_startup,
	.hw_params = tacna_hw_params_rate,
	.set_sysclk = tacna_dai_set_sysclk,
};
EXPORT_SYMBOL_GPL(tacna_simple_dai_ops);

/*
 * tacna_set_output_mode - Set the mode of the specified output
 *
 * @codec: Device to configure
 * @output: Output number
 * @diff: True to set the output to differential mode
 *
 * Some systems use external analogue switches to connect more
 * analogue devices to the CODEC than are supported by the device.  In
 * some systems this requires changing the switched output from single
 * ended to differential mode dynamically at runtime, an operation
 * supported using this function.
 *
 * Most systems have a single static configuration and should use
 * platform data instead.
 */
int tacna_set_output_mode(struct snd_soc_codec *codec, int output, bool diff)
{
	unsigned int reg, val;
	int ret;

	if (output < 1 || output > TACNA_MAX_OUTPUT)
		return -EINVAL;

	reg = TACNA_OUT1L_CONTROL_1 + (output - 1) * 0x40;

	if (diff)
		val = TACNA_OUT1_MONO;
	else
		val = 0;

	ret = snd_soc_update_bits(codec, reg, TACNA_OUT1_MONO, val);
	if (ret < 0)
		return ret;
	else
		return 0;
}
EXPORT_SYMBOL_GPL(tacna_set_output_mode);

int tacna_get_accdet_for_output(struct snd_soc_codec *codec, int output)
{
	struct tacna *tacna = dev_get_drvdata(codec->dev->parent);
	int i;

	for (i = 0; i < ARRAY_SIZE(tacna->pdata.accdet); i++)
		if (tacna->pdata.accdet[i].output == output)
			return i;

	return -ENODEV;
}
EXPORT_SYMBOL_GPL(tacna_get_accdet_for_output);

int tacna_sysclk_ev(struct snd_soc_dapm_widget *w,
		    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);

	tacna_spin_sysclk(priv);

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_sysclk_ev);

int tacna_domain_clk_ev(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol,
			int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	int dom_grp = w->shift;

	if (dom_grp >= ARRAY_SIZE(priv->domain_group_ref)) {
		WARN(true, "%s dom_grp exceeds array size\n", __func__);
		return -EINVAL;
	}

	/*
	 * We can't rely on the DAPM mutex for locking because we need a lock
	 * that can safely be called in hw_params
	 */
	mutex_lock(&priv->rate_lock);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		dev_dbg(priv->tacna->dev, "Inc ref on domain group %d\n",
			dom_grp);
		++priv->domain_group_ref[dom_grp];
		break;
	case SND_SOC_DAPM_POST_PMD:
		dev_dbg(priv->tacna->dev, "Dec ref on domain group %d\n",
			dom_grp);
		--priv->domain_group_ref[dom_grp];
		break;
	default:
		break;
	}

	tacna_debug_dump_domain_groups(priv);

	mutex_unlock(&priv->rate_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_domain_clk_ev);

int tacna_in_ev(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol,
		int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	unsigned int reg;

	if (w->shift % 2)
		reg = TACNA_IN1L_CONTROL2 + ((w->shift / 2) * 0x40);
	else
		reg = TACNA_IN1R_CONTROL2 + ((w->shift / 2) * 0x40);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		priv->in_pending++;
		break;
	case SND_SOC_DAPM_POST_PMU:
		priv->in_pending--;
		snd_soc_update_bits(codec, reg, TACNA_IN1L_MUTE, 0);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, reg,
				    TACNA_IN1L_MUTE, TACNA_IN1L_MUTE);
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_in_ev);

int tacna_wait_for_output_seq(struct tacna_priv *priv, unsigned int mask,
			      unsigned int target)
{
	unsigned int val;
	int ret;

	dev_dbg(priv->dev, "Polling output status for mask 0x%x = 0x%x\n",
		mask, target);

	ret = regmap_read_poll_timeout(priv->tacna->regmap,
				       TACNA_OUTPUT_STATUS_1,
				       val,
				       ((val & mask) == target),
				       TACNA_CHANNEL_STATUS_POLL_US,
				       TACNA_MAX_OUTPUT_CHANNELS *
				       TACNA_CHANNEL_STATUS_POLL_TIMEOUT_US);
	if (ret)
		dev_warn(priv->dev,
			 "Failed to get output sequence bits 0x%x = 0x%x (%d)\n",
			 mask, val, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(tacna_wait_for_output_seq);

int tacna_out_ev(struct snd_soc_dapm_widget *w,
		 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		switch (w->shift) {
		case TACNA_OUT1L_EN_SHIFT:
			priv->out_up_mask |= TACNA_OUT1L_STS;
			break;
		case TACNA_OUT1R_EN_SHIFT:
			priv->out_up_mask |= TACNA_OUT1R_STS;
			break;
		case TACNA_OUT2L_EN_SHIFT:
			priv->out_up_mask |= TACNA_OUT2L_STS;
			break;
		case TACNA_OUT2R_EN_SHIFT:
			priv->out_up_mask |= TACNA_OUT2R_STS;
			break;
		default:
			return 0;
		}
		priv->out_up_pending++;
		break;

	case SND_SOC_DAPM_POST_PMU:
		switch (w->shift) {
		case TACNA_OUT1L_EN_SHIFT:
		case TACNA_OUT1R_EN_SHIFT:
		case TACNA_OUT2L_EN_SHIFT:
		case TACNA_OUT2R_EN_SHIFT:
			priv->out_up_pending--;
			if (priv->out_up_pending == 0) {
				tacna_wait_for_output_seq(priv,
							  priv->out_up_mask,
							  priv->out_up_mask);
				priv->out_up_mask = 0;
			}
			break;

		default:
			break;
		}
		break;

	case SND_SOC_DAPM_PRE_PMD:
		switch (w->shift) {
		case TACNA_OUT1L_EN_SHIFT:
			priv->out_down_mask |= TACNA_OUT1L_STS;
			break;
		case TACNA_OUT1R_EN_SHIFT:
			priv->out_down_mask |= TACNA_OUT1R_STS;
			break;
		case TACNA_OUT2L_EN_SHIFT:
			priv->out_down_mask |= TACNA_OUT2L_STS;
			break;
		case TACNA_OUT2R_EN_SHIFT:
			priv->out_down_mask |= TACNA_OUT2R_STS;
			break;
		default:
			return 0;
		}
		priv->out_down_pending++;
		break;

	case SND_SOC_DAPM_POST_PMD:
		switch (w->shift) {
		case TACNA_OUT1L_EN_SHIFT:
		case TACNA_OUT1R_EN_SHIFT:
		case TACNA_OUT2L_EN_SHIFT:
		case TACNA_OUT2R_EN_SHIFT:
			priv->out_down_pending--;
			if (priv->out_down_pending == 0) {
				tacna_wait_for_output_seq(priv,
							  priv->out_down_mask,
							  0);
				priv->out_down_mask = 0;
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_out_ev);

int tacna_hp_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = priv->tacna;
	unsigned int mask = 1 << w->shift;
	unsigned int out_num = (w->shift / 2) + 1;
	unsigned int val;
	int ret, accdet;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		val = mask;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		val = 0;
		break;
	case SND_SOC_DAPM_PRE_PMU:
	case SND_SOC_DAPM_POST_PMD:
		return tacna_out_ev(w, kcontrol, event);
	default:
		return 0;
	}

	/* store desired output state */
	tacna->hp_ena = (tacna->hp_ena & ~mask) | val;

	/*
	 * disable output if clamp is active (output state will be applied when
	 * the clamp is disabled) or a short was detected
	 */
	accdet = tacna_get_accdet_for_output(codec, out_num);
	if (accdet >= 0 &&
	    (tacna->hpdet_clamp[accdet] || tacna->hpdet_shorted[accdet]))
		val = 0;

	ret = regmap_update_bits(tacna->regmap, TACNA_OUTPUT_ENABLE_1,
				 mask, val);
	if (ret)
		dev_warn(priv->dev, "Failed to write output enable: %d\n", ret);

	return tacna_out_ev(w, kcontrol, event);
}
EXPORT_SYMBOL_GPL(tacna_hp_ev);

static bool tacna_eq_filter_unstable(bool mode, __be16 _a, __be16 _b)
{
	s16 a = be16_to_cpu(_a);
	s16 b = be16_to_cpu(_b);

	if (!mode) {
		return abs(a) >= 4096;
	} else {
		if (abs(b) >= 4096)
			return true;

		return (abs((a << 16) / (4096 - b)) >= 4096 << 4);
	}
}

int tacna_eq_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol,
		int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = priv->tacna;
	unsigned int mode = priv->eq_mode[w->shift];
	unsigned int reg = TACNA_EQ1_BAND1_COEFF1 + (68 * w->shift);
	__be16 *data = &priv->eq_coefficients[w->shift][0];
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (tacna_eq_filter_unstable(!!mode, data[1], data[0]) ||
		    tacna_eq_filter_unstable(true, data[7], data[6]) ||
		    tacna_eq_filter_unstable(true, data[13], data[12]) ||
		    tacna_eq_filter_unstable(true, data[19], data[18]) ||
		    tacna_eq_filter_unstable(false, data[25], data[24])) {
			dev_err(tacna->dev,
				"Rejecting unstable EQ coefficients.\n"
				"Last stable coefficients will be used.\n");

			ret = -EINVAL;
		} else {
			ret = regmap_raw_write(tacna->regmap, reg, data,
					       TACNA_EQ_BLOCK_SZ);
			if (ret < 0) {
				dev_err(tacna->dev,
					"Error writing EQ coefficients: %d\n",
					ret);
				goto out;
			}

			ret = snd_soc_update_bits(codec, TACNA_EQ_CONTROL2,
						  w->mask, mode << w->shift);
			if (ret < 0)
				dev_err(tacna->dev,
					"Error writing EQ mode: %d\n",
					ret);
		}
		break;
	default:
		break;
	}

out:
	return ret;
}
EXPORT_SYMBOL_GPL(tacna_eq_ev);

int tacna_anc_ev(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol,
		 int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	unsigned int val;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		val = 1 << w->shift;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		val = 1 << (w->shift + 1);
		break;
	default:
		return 0;
	}

	snd_soc_write(codec, TACNA_ANC_CTRL_1, val);

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_anc_ev);

static int tacna_get_variable_u32_array(struct tacna_priv *priv,
					const char *propname,
					u32 *dest,
					int n_max,
					int multiple)
{
	struct tacna *tacna = priv->tacna;
	int n, ret;

	n = device_property_read_u32_array(tacna->dev, propname, NULL, 0);
	if (n == -EINVAL) {
		return 0;	/* missing, ignore */
	} else if (n < 0) {
		dev_warn(tacna->dev, "%s malformed (%d)\n", propname, n);
		return -EINVAL;
	} else if ((n % multiple) != 0) {
		dev_warn(tacna->dev, "%s not a multiple of %d entries\n",
			 propname, multiple);
		return -EINVAL;
	}

	if (n > n_max)
		n = n_max;

	ret = device_property_read_u32_array(tacna->dev, propname, dest, n);

	if (ret < 0)
		return ret;
	else
		return n;
}

static void tacna_prop_get_inmode(struct tacna_priv *priv)
{
	struct tacna *tacna = priv->tacna;
	u32 tmp[TACNA_MAX_INPUT * TACNA_MAX_MUXED_IN_CHANNELS];
	int n, i, in_idx, ch_idx;

	BUILD_BUG_ON(ARRAY_SIZE(tacna->pdata.codec.inmode) != TACNA_MAX_INPUT);
	BUILD_BUG_ON(ARRAY_SIZE(tacna->pdata.codec.inmode[0]) !=
		     TACNA_MAX_MUXED_IN_CHANNELS);

	n = tacna_get_variable_u32_array(priv,
					 "cirrus,inmode",
					 tmp,
					 ARRAY_SIZE(tmp),
					 TACNA_MAX_MUXED_IN_CHANNELS);
	if (n < 0)
		return;

	in_idx = 0;
	ch_idx = 0;
	for (i = 0; i < n; ++i) {
		tacna->pdata.codec.inmode[in_idx][ch_idx] = tmp[i];

		if (++ch_idx == TACNA_MAX_MUXED_IN_CHANNELS) {
			ch_idx = 0;
			++in_idx;
		}
	}
}

static void tacna_prop_get_pdata(struct tacna_priv *priv)
{
	struct tacna *tacna = priv->tacna;
	struct tacna_codec_pdata *pdata = &tacna->pdata.codec;
	u32 out_mono[ARRAY_SIZE(pdata->out_mono)];
	u32 auxpdm_slave_mode[ARRAY_SIZE(pdata->auxpdm_slave_mode)];
	u32 auxpdm_falling_edge[ARRAY_SIZE(pdata->auxpdm_falling_edge)];
	int i, ret;

	ret = tacna_get_variable_u32_array(priv,
					"cirrus,max-channels-clocked",
					pdata->max_channels_clocked,
					ARRAY_SIZE(pdata->max_channels_clocked),
					1);
	if (ret < 0)
		return;

	tacna_prop_get_inmode(priv);

	memset(out_mono, 0, sizeof(out_mono));
	device_property_read_u32_array(tacna->dev,
				       "cirrus,out-mono",
				       out_mono,
				       ARRAY_SIZE(out_mono));

	for (i = 0; i < ARRAY_SIZE(out_mono); ++i)
		pdata->out_mono[i] = !!out_mono[i];

	device_property_read_u32(tacna->dev, "cirrus,pdm-fmt", &pdata->pdm_fmt);

	device_property_read_u32(tacna->dev, "cirrus,pdm-mute",
				 &pdata->pdm_mute);

	memset(auxpdm_slave_mode, 0, sizeof(auxpdm_slave_mode));
	device_property_read_u32_array(tacna->dev,
				       "cirrus,auxpdm-slave-mode",
				       auxpdm_slave_mode,
				       ARRAY_SIZE(auxpdm_slave_mode));

	for (i = 0; i < ARRAY_SIZE(auxpdm_slave_mode); ++i)
		pdata->auxpdm_slave_mode[i] = !!auxpdm_slave_mode[i];

	memset(auxpdm_falling_edge, 0, sizeof(auxpdm_falling_edge));
	device_property_read_u32_array(tacna->dev,
				       "cirrus,auxpdm-falling-edge",
				       auxpdm_falling_edge,
				       ARRAY_SIZE(auxpdm_falling_edge));

	for (i = 0; i < ARRAY_SIZE(auxpdm_falling_edge); ++i)
		pdata->auxpdm_falling_edge[i] = !!auxpdm_falling_edge[i];
}

int tacna_init_fll(struct tacna *tacna,
		   int id,
		   int base,
		   unsigned int sts_addr,
		   unsigned int sts_mask,
		   struct tacna_fll *fll)
{
	init_completion(&fll->ok);

	fll->id = id;
	fll->base = base;
	fll->sts_addr = sts_addr;
	fll->sts_mask = sts_mask;
	fll->tacna = tacna;
	fll->ref_src = TACNA_FLL_SRC_NONE;
	fll->sync_src = TACNA_FLL_SRC_NONE;

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_init_fll);

int tacna_init_inputs(struct snd_soc_codec *codec)
{
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = priv->tacna;
	unsigned int ana_mode_l, ana_mode_r;
	int i;

	/*
	 * Initialize input modes from the A settings. For muxed inputs the
	 * B settings will be applied if the mux is changed
	 */
	for (i = 0; i < priv->max_analogue_inputs; i++) {
		dev_dbg(tacna->dev, "IN%d mode %u:%u:%u:%u\n", i + 1,
			tacna->pdata.codec.inmode[i][0],
			tacna->pdata.codec.inmode[i][1],
			tacna->pdata.codec.inmode[i][2],
			tacna->pdata.codec.inmode[i][3]);

		switch (tacna->pdata.codec.inmode[i][0]) {
		case TACNA_INMODE_DIFF:
			ana_mode_l = 0;
			break;
		case TACNA_INMODE_SE:
			ana_mode_l = 1 << TACNA_IN1L_SRC_SHIFT;
			break;
		default:
			dev_warn(tacna->dev,
				 "IN%dL_1 Illegal inmode %u ignored\n",
				 i + 1, tacna->pdata.codec.inmode[i][0]);
			continue;
		}

		switch (tacna->pdata.codec.inmode[i][1]) {
		case TACNA_INMODE_DIFF:
			ana_mode_r = 0;
			break;
		case TACNA_INMODE_SE:
			ana_mode_r = 1 << TACNA_IN1R_SRC_SHIFT;
			break;
		default:
			dev_warn(tacna->dev,
				 "IN%dR_1 Illegal inmode %u ignored\n",
				 i + 1, tacna->pdata.codec.inmode[i][1]);
			continue;
		}

		dev_dbg(tacna->dev,
			"IN%d_1 Analogue mode=0x%x,0x%x\n",
			i + 1, ana_mode_l, ana_mode_r);

		regmap_update_bits(tacna->regmap,
				   TACNA_IN1L_CONTROL1 + (i * 0x40),
				   TACNA_IN1L_SRC_MASK,
				   ana_mode_l);

		regmap_update_bits(tacna->regmap,
				   TACNA_IN1R_CONTROL1 + (i * 0x40),
				   TACNA_IN1R_SRC_MASK,
				   ana_mode_r);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_init_inputs);

int tacna_init_auxpdm(struct snd_soc_codec *codec, int n_auxpdm)
{
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = priv->tacna;
	const struct tacna_codec_pdata *pdata = &tacna->pdata.codec;
	unsigned int val;
	int i;

	for (i = 0; i < n_auxpdm; ++i)
	{
		if (pdata->auxpdm_slave_mode[i])
			val = 0;
		else
			val = TACNA_AUXPDM1_MSTR_MASK;

		if (pdata->auxpdm_falling_edge[i])
			val |= TACNA_AUXPDM1_TXEDGE_MASK;
		regmap_update_bits(tacna->regmap,
				   TACNA_AUXPDM1_CONTROL1 + (i * 0x8),
				   TACNA_AUXPDM1_TXEDGE_MASK |
				   TACNA_AUXPDM1_MSTR_MASK,
				   val);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_init_auxpdm);

int tacna_init_outputs(struct snd_soc_codec *codec, int n_mono_routes)
{
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct tacna_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct tacna *tacna = priv->tacna;
	const struct tacna_codec_pdata *pdata = &tacna->pdata.codec;
	unsigned int val;
	int i;

	if (n_mono_routes > TACNA_MAX_OUTPUT) {
		dev_warn(tacna->dev,
			 "Requested %d mono outputs, using maximum allowed %d\n",
			 n_mono_routes, TACNA_MAX_OUTPUT);
		n_mono_routes = TACNA_MAX_OUTPUT;
	}

	for (i = 0; i < n_mono_routes; i++) {
		/* Default is 0 so noop with defaults */
		if (pdata->out_mono[i]) {
			val = TACNA_OUT1_MONO;
			snd_soc_dapm_add_routes(dapm, &tacna_mono_routes[i], 1);
		} else {
			val = 0;
		}

		regmap_update_bits(tacna->regmap,
				   TACNA_OUT1L_CONTROL_1 + (i * 0x40),
				   TACNA_OUT1_MONO,
				   val);

		dev_dbg(tacna->dev, "OUT%d mono=0x%x\n", i + 1, val);
	}

	dev_dbg(tacna->dev, "OUT5 fmt=0x%x mute=0x%x\n",
		pdata->pdm_fmt, pdata->pdm_mute);

	if (pdata->pdm_mute)
		regmap_update_bits(tacna->regmap,
				   TACNA_OUT5PDM_CONTROL_1,
				   TACNA_OUT5PDM_MUTE_SEQ_MASK,
				   pdata->pdm_mute);

	if (pdata->pdm_fmt)
		regmap_update_bits(tacna->regmap,
				   TACNA_OUT5PDM_CONTROL_1,
				   TACNA_OUT5PDM_MUTE_ENDIAN_MASK |
				   TACNA_OUT5PDM_FMT_MASK,
				   pdata->pdm_fmt);

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_init_outputs);

int tacna_init_dai(struct tacna_priv *priv, int id)
{
	struct tacna_dai_priv *dai_priv = &priv->dai[id];

	dai_priv->clk = TACNA_CLK_SYSCLK_1;
	dai_priv->constraint = tacna_constraint;

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_init_dai);

int tacna_init_eq(struct tacna_priv *priv)
{
	struct tacna *tacna = priv->tacna;
	unsigned int reg = TACNA_EQ1_BAND1_COEFF1, mode;
	__be16 *data;
	int i, ret;

	ret = regmap_read(tacna->regmap, TACNA_EQ_CONTROL2, &mode);
	if (ret < 0) {
		dev_err(tacna->dev, "Error reading EQ mode: %d\n", ret);
		goto out;
	}

	for (i = 0; i < 4; ++i) {
		priv->eq_mode[i] = (mode >> i) & 0x1;

		data = &priv->eq_coefficients[i][0];
		ret = regmap_raw_read(tacna->regmap, reg + (i * 68), data,
				      TACNA_EQ_BLOCK_SZ);
		if (ret < 0) {
			dev_err(tacna->dev,
				"Error reading EQ coefficients: %d\n",
				ret);
			goto out;
		}
	}

out:
	return ret;
}
EXPORT_SYMBOL_GPL(tacna_init_eq);

int tacna_core_init(struct tacna_priv *priv)
{
	BUILD_BUG_ON(ARRAY_SIZE(tacna_mixer_texts) != TACNA_NUM_MIXER_INPUTS);
	BUILD_BUG_ON(ARRAY_SIZE(tacna_mixer_values) != TACNA_NUM_MIXER_INPUTS);
	BUILD_BUG_ON(tacna_sample_rate_text[TACNA_SAMPLE_RATE_ENUM_SIZE - 1]
		     == NULL);
	BUILD_BUG_ON(tacna_sample_rate_val[TACNA_SAMPLE_RATE_ENUM_SIZE - 1]
		     == 0);

	if (!dev_get_platdata(priv->tacna->dev))
		tacna_prop_get_pdata(priv);

	mutex_init(&priv->rate_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_core_init);

int tacna_core_destroy(struct tacna_priv *priv)
{
	mutex_destroy(&priv->rate_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(tacna_core_destroy);

MODULE_DESCRIPTION("ASoC Cirrus Logic Tacna codec support");
MODULE_AUTHOR("Charles Keepax <ckeepax@opensource.wolfsonmicro.com>");
MODULE_AUTHOR("Richard Fitzgerald <rf@opensource.wolfsonmicro.com>");
MODULE_AUTHOR("Piotr Stankiewicz <piotrs@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL v2");
