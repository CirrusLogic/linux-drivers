/*
 * tacna.h - Cirrus Logic Tacna class codecs common support
 *
 * Copyright 2016-2017 Cirrus Logic
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ASOC_TACNA_H
#define ASOC_TACNA_H

#include <linux/completion.h>

#include <linux/mfd/tacna/core.h>

#include <sound/soc.h>
#include <sound/tacna-pdata.h>

#include "wm_adsp.h"

#define TACNA_FLL1_REFCLK		1
#define TACNA_FLL2_REFCLK		2
#define TACNA_FLL3_REFCLK		3
#define TACNA_FLL1_SYNCCLK		4
#define TACNA_FLL2_SYNCCLK		5
#define TACNA_FLL3_SYNCCLK		6

#define TACNA_FLL_SRC_NONE		-1
#define TACNA_FLL_SRC_MCLK1		0
#define TACNA_FLL_SRC_MCLK2		1
#define TACNA_FLL_SRC_MCLK3		2
#define TACNA_FLL_SRC_DSDCLK		4
#define TACNA_FLL_SRC_DMICCLK		5
#define TACNA_FLL_SRC_SWCLK		6
#define TACNA_FLL_SRC_SLIMCLK		7
#define TACNA_FLL_SRC_ASP1SCLK		8
#define TACNA_FLL_SRC_ASP2SCLK		9
#define TACNA_FLL_SRC_ASP3SCLK		10
#define TACNA_FLL_SRC_ASP4SCLK		11
#define TACNA_FLL_SRC_ASP1FSYNC		12
#define TACNA_FLL_SRC_ASP2FSYNC		13
#define TACNA_FLL_SRC_ASP3FSYNC		14
#define TACNA_FLL_SRC_ASP4FSYNC		15

#define TACNA_CLK_SYSCLK_1		1
#define TACNA_CLK_SYSCLK_2		2
#define TACNA_CLK_SYSCLK_3		3
#define TACNA_CLK_ASYNCCLK_1		4
#define TACNA_CLK_ASYNCCLK_2		5
#define TACNA_CLK_DSPCLK		6
#define TACNA_CLK_DACCLK		7
#define TACNA_CLK_OPCLK			8
#define TACNA_CLK_ASYNC_OPCLK		9

#define TACNA_RATE_SAMPLE_RATE_1	0
#define TACNA_RATE_SAMPLE_RATE_2	1
#define TACNA_RATE_SAMPLE_RATE_3	2
#define TACNA_RATE_ASYNC_SAMPLE_RATE_1	8
#define TACNA_RATE_ASYNC_SAMPLE_RATE_2	9
#define TACNA_RATE_ASYNC2_SAMPLE_RATE_1	16
#define TACNA_RATE_ASYNC2_SAMPLE_RATE_2	17

#define TACNA_CLK_SRC_MCLK1		0x0
#define TACNA_CLK_SRC_MCLK2		0x1
#define TACNA_CLK_SRC_MCLK3		0x2
#define TACNA_CLK_SRC_FLL1		0x4
#define TACNA_CLK_SRC_FLL2		0x5
#define TACNA_CLK_SRC_FLL3		0x6
#define TACNA_CLK_SRC_ASP1SCLK		0x8
#define TACNA_CLK_SRC_ASP2SCLK		0x9
#define TACNA_CLK_SRC_ASP3SCLK		0xA
#define TACNA_CLK_SRC_ASP4SCLK		0xB
#define TACNA_CLK_SRC_FLL1_50MHZ	0xF
#define TACNA_CLK_SRC_FLL1_DSP_DIV2	0x14
#define TACNA_CLK_SRC_FLL2_DSP_DIV2	0x15
#define TACNA_CLK_SRC_FLL3_DSP_DIV2	0x16

#define TACNA_MIXER_VOL_MASK		0x00FE0000
#define TACNA_MIXER_VOL_SHIFT		17
#define TACNA_MIXER_VOL_WIDTH		7

#define TACNA_DOM_GRP_FX		0
#define TACNA_DOM_GRP_ASRC1_RATE_1	1
#define TACNA_DOM_GRP_ASRC1_RATE_2	2
#define TACNA_DOM_GRP_ASRC2_RATE_1	3
#define TACNA_DOM_GRP_ASRC2_RATE_2	4
#define TACNA_DOM_GRP_ISRC1_DEC		5
#define TACNA_DOM_GRP_ISRC1_INT		6
#define TACNA_DOM_GRP_ISRC2_DEC		7
#define TACNA_DOM_GRP_ISRC2_INT		8
#define TACNA_DOM_GRP_DAC		9
#define TACNA_DOM_GRP_ASP1		10
#define TACNA_DOM_GRP_ASP2		11
#define TACNA_DOM_GRP_ASP3		12
#define TACNA_DOM_GRP_ASP4		13
#define TACNA_DOM_GRP_SLIMBUS		14
#define TACNA_DOM_GRP_SOUND_WIRE	15
#define TACNA_DOM_GRP_PWM		16
#define TACNA_DOM_GRP_DFC1		17
#define TACNA_DOM_GRP_DFC2		18
#define TACNA_DOM_GRP_DFC3		19
#define TACNA_DOM_GRP_DFC4		20
#define TACNA_DOM_GRP_DFC5		21
#define TACNA_DOM_GRP_DFC6		22
#define TACNA_DOM_GRP_DFC7		23
#define TACNA_DOM_GRP_DFC8		24
#define TACNA_DOM_GRP_DSP1		25
#define TACNA_DOM_GRP_DSP2		26
#define TACNA_N_DOM_GRPS		27

#define TACNA_MAX_DAI			11
#define TACNA_MAX_DSP			2

#define TACNA_NUM_MIXER_INPUTS		123

#define TACNA_FRF_COEFF_LEN		2

#define TACNA_EQ_BLOCK_SZ		60
#define TACNA_N_EQ_BLOCKS		4

#define TACNA_OSR_ENUM_SIZE		7
#define TACNA_SYNC_RATE_ENUM_SIZE	3
#define TACNA_ASYNC_RATE_ENUM_SIZE	2
#define TACNA_RATE_ENUM_SIZE \
		(TACNA_SYNC_RATE_ENUM_SIZE + TACNA_ASYNC_RATE_ENUM_SIZE)
#define TACNA_SAMPLE_RATE_ENUM_SIZE	17
#define TACNA_DFC_TYPE_ENUM_SIZE	5
#define TACNA_DFC_WIDTH_ENUM_SIZE	25

#define TACNA_MIXER_CONTROLS(name, base) \
	SOC_SINGLE_RANGE_TLV(name " Input 1 Volume", base,		\
			     TACNA_MIXER_VOL_SHIFT, 0x20, 0x50, 0,	\
			     tacna_mixer_tlv),				\
	SOC_SINGLE_RANGE_TLV(name " Input 2 Volume", base + 4,		\
			     TACNA_MIXER_VOL_SHIFT, 0x20, 0x50, 0,	\
			     tacna_mixer_tlv),				\
	SOC_SINGLE_RANGE_TLV(name " Input 3 Volume", base + 8,		\
			     TACNA_MIXER_VOL_SHIFT, 0x20, 0x50, 0,	\
			     tacna_mixer_tlv),				\
	SOC_SINGLE_RANGE_TLV(name " Input 4 Volume", base + 12,		\
			     TACNA_MIXER_VOL_SHIFT, 0x20, 0x50, 0,	\
			     tacna_mixer_tlv)

#define TACNA_MUX_ENUM_DECL(name, reg) \
	SOC_VALUE_ENUM_SINGLE_AUTODISABLE_DECL( \
		name, reg, 0, 0x1ff, tacna_mixer_texts, tacna_mixer_values)

#define TACNA_MUX_CTL_DECL(name) \
	const struct snd_kcontrol_new name##_mux =	\
		SOC_DAPM_ENUM("Route", name##_enum)

#define TACNA_MUX_ENUMS(name, base_reg) \
	static TACNA_MUX_ENUM_DECL(name##_enum, base_reg);	\
	static TACNA_MUX_CTL_DECL(name)

#define TACNA_MIXER_ENUMS(name, base_reg) \
	TACNA_MUX_ENUMS(name##_in1, base_reg);     \
	TACNA_MUX_ENUMS(name##_in2, base_reg + 4); \
	TACNA_MUX_ENUMS(name##_in3, base_reg + 8); \
	TACNA_MUX_ENUMS(name##_in4, base_reg + 12)

#define TACNA_MUX(name, ctrl) \
	SND_SOC_DAPM_MUX(name, SND_SOC_NOPM, 0, 0, ctrl)

#define TACNA_MUX_WIDGETS(name, name_str) \
	TACNA_MUX(name_str " Input 1", &name##_mux)

#define TACNA_MIXER_WIDGETS(name, name_str)	\
	TACNA_MUX(name_str " Input 1", &name##_in1_mux), \
	TACNA_MUX(name_str " Input 2", &name##_in2_mux), \
	TACNA_MUX(name_str " Input 3", &name##_in3_mux), \
	TACNA_MUX(name_str " Input 4", &name##_in4_mux), \
	SND_SOC_DAPM_MIXER(name_str " Mixer", SND_SOC_NOPM, 0, 0, NULL, 0)

#define TACNA_MUX_ROUTES(widget, name) \
	{ widget, NULL, name " Input 1" }, \
	TACNA_MIXER_INPUT_ROUTES(name " Input 1")

#define TACNA_MIXER_ROUTES(widget, name)		\
	{ widget, NULL, name " Mixer" },		\
	{ name " Mixer", NULL, name " Input 1" },	\
	{ name " Mixer", NULL, name " Input 2" },	\
	{ name " Mixer", NULL, name " Input 3" },	\
	{ name " Mixer", NULL, name " Input 4" },	\
	TACNA_MIXER_INPUT_ROUTES(name " Input 1"),	\
	TACNA_MIXER_INPUT_ROUTES(name " Input 2"),	\
	TACNA_MIXER_INPUT_ROUTES(name " Input 3"),	\
	TACNA_MIXER_INPUT_ROUTES(name " Input 4")

#define TACNA_DSP_ROUTES(name)				\
	{ name, NULL, name " Preloader" },		\
	{ name " Preloader", NULL, "SYSCLK" },		\
	{ name " Preloader", NULL, "DSPCLK" },		\
	{ name " Preload", NULL, name " Preloader" },	\
	TACNA_MIXER_ROUTES(name, name "RX1"),		\
	TACNA_MIXER_ROUTES(name, name "RX2"),		\
	TACNA_MIXER_ROUTES(name, name "RX3"),		\
	TACNA_MIXER_ROUTES(name, name "RX4"),		\
	TACNA_MIXER_ROUTES(name, name "RX5"),		\
	TACNA_MIXER_ROUTES(name, name "RX6"),		\
	TACNA_MIXER_ROUTES(name, name "RX7"),		\
	TACNA_MIXER_ROUTES(name, name "RX8")		\

#define TACNA_SAMPLE_RATE_CONTROL(name, domain) \
	SOC_ENUM(name, tacna_sample_rate[(domain) - 2])

#define TACNA_RATE_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,\
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_get_enum_double, .put = tacna_rate_put, \
	.private_value = (unsigned long)&xenum }

#define TACNA_EQ_COEFF_CONTROL(xname, xreg, xbase, xshift)	\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
	.info = tacna_eq_coeff_info, .get = tacna_eq_coeff_get,	\
	.put = tacna_eq_coeff_put, .private_value =		\
	(unsigned long)&(struct tacna_eq_control) { .reg = xreg,\
	.shift = xshift, .block_base = xbase, .max = 65535 } }

#define TACNA_EQ_REG_NAME_PASTER(eq, band, type) \
    TACNA_ ## eq ## _ ## band ## _ ## type

#define TACNA_EQ_BAND_COEFF_CONTROLS(name, band)		\
	TACNA_EQ_COEFF_CONTROL(#name " " #band " A",		\
		TACNA_EQ_REG_NAME_PASTER(name, band, COEFF1),	\
		TACNA_EQ_REG_NAME_PASTER(name, BAND1, COEFF1),	\
		TACNA_EQ1_B1_A_SHIFT),				\
	TACNA_EQ_COEFF_CONTROL(#name " " #band " B",		\
		TACNA_EQ_REG_NAME_PASTER(name, band, COEFF1),	\
		TACNA_EQ_REG_NAME_PASTER(name, BAND1, COEFF1),	\
		TACNA_EQ1_B1_B_SHIFT),				\
	TACNA_EQ_COEFF_CONTROL(#name " " #band " C",		\
		TACNA_EQ_REG_NAME_PASTER(name, band, COEFF2),	\
		TACNA_EQ_REG_NAME_PASTER(name, BAND1, COEFF1),	\
		TACNA_EQ1_B1_C_SHIFT),				\
	TACNA_EQ_COEFF_CONTROL(#name " " #band " PG",		\
		TACNA_EQ_REG_NAME_PASTER(name, band, PG),	\
		TACNA_EQ_REG_NAME_PASTER(name, BAND1, COEFF1),	\
		TACNA_EQ1_B1_PG_SHIFT)

#define TACNA_EQ_COEFF_CONTROLS(name)				\
	TACNA_EQ_BAND_COEFF_CONTROLS(name, BAND1),		\
	TACNA_EQ_BAND_COEFF_CONTROLS(name, BAND2),		\
	TACNA_EQ_BAND_COEFF_CONTROLS(name, BAND3),		\
	TACNA_EQ_BAND_COEFF_CONTROLS(name, BAND4),		\
	TACNA_EQ_COEFF_CONTROL(#name " BAND5 A",		\
		TACNA_EQ_REG_NAME_PASTER(name, BAND5, COEFF1),	\
		TACNA_EQ_REG_NAME_PASTER(name, BAND1, COEFF1),	\
		TACNA_EQ1_B1_A_SHIFT),				\
	TACNA_EQ_COEFF_CONTROL(#name " BAND5 B",		\
		TACNA_EQ_REG_NAME_PASTER(name, BAND5, COEFF1),	\
		TACNA_EQ_REG_NAME_PASTER(name, BAND1, COEFF1),	\
		TACNA_EQ1_B1_B_SHIFT),				\
	TACNA_EQ_COEFF_CONTROL(#name " BAND5 PG",		\
		TACNA_EQ_REG_NAME_PASTER(name, BAND5, PG),	\
		TACNA_EQ_REG_NAME_PASTER(name, BAND1, COEFF1),	\
		TACNA_EQ1_B1_PG_SHIFT)

#define TACNA_LHPF_CONTROL(xname, xbase)			\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
	.info = snd_soc_bytes_info, .get = snd_soc_bytes_get,	\
	.put = tacna_lhpf_coeff_put, .private_value =		\
	((unsigned long)&(struct soc_bytes) { .base = xbase,	\
	 .num_regs = 1 }) }

#define TACNA_FRF_BYTES(xname, xbase, xregs)			\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
	.info = snd_soc_bytes_info, .get = snd_soc_bytes_get,	\
	.put = tacna_frf_bytes_put, .private_value =		\
	((unsigned long)&(struct soc_bytes) {.base = xbase,	\
	 .num_regs = xregs }) }

#define TACNA_RATES SNDRV_PCM_RATE_KNOT

#define TACNA_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

struct tacna;
struct tacna_jd_state;
struct wm_adsp;

/* Notify data structure for TACNA_NOTIFY_ULTRASONIC */
struct tacna_us_notify_data {
	unsigned int us_no;
};

struct tacna_dai_priv {
	int clk;
	struct snd_pcm_hw_constraint_list constraint;
};

struct tacna_priv {
	struct wm_adsp dsp[TACNA_MAX_DSP];
	struct tacna *tacna;
	struct device *dev;
	int sysclk;
	int asyncclk;
	int dspclk;
	struct tacna_dai_priv dai[TACNA_MAX_DAI];

	int num_inputs;
	int max_analogue_inputs;
	int num_dmic_clksrc;

	unsigned int in_pending;

	unsigned int out_up_pending;
	unsigned int out_up_mask;
	unsigned int out_down_pending;
	unsigned int out_down_mask;

	struct mutex rate_lock;

	int tdm_width[TACNA_MAX_ASP];
	int tdm_slots[TACNA_MAX_ASP];

	int domain_group_ref[TACNA_N_DOM_GRPS];

	unsigned int eq_mode[TACNA_N_EQ_BLOCKS];
	__be16 eq_coefficients[TACNA_N_EQ_BLOCKS][TACNA_EQ_BLOCK_SZ / 2];

	const unsigned int *dsp_power_regs[TACNA_MAX_DSP];
};

struct tacna_fll_cfg {
	int n;
	unsigned int theta;
	unsigned int lambda;
	int refdiv;
	int fratio;
	int gain;
	int alt_gain;
};

struct tacna_fll {
	struct tacna *tacna;
	int id;
	unsigned int base;
	struct completion ok;

	unsigned int sts_addr;
	unsigned int sts_mask;

	unsigned int fout;

	int sync_src;
	unsigned int sync_freq;

	int ref_src;
	unsigned int ref_freq;
	struct tacna_fll_cfg ref_cfg;
};

struct tacna_enum {
	struct soc_enum mixer_enum;
	int val;
};

struct tacna_eq_control {
	unsigned int reg;
	unsigned int shift;
	unsigned int block_base;
	unsigned int max;
};

extern const char * const tacna_mixer_texts[TACNA_NUM_MIXER_INPUTS];
extern unsigned int tacna_mixer_values[TACNA_NUM_MIXER_INPUTS];

extern const unsigned int tacna_ana_tlv[];
extern const unsigned int tacna_eq_tlv[];
extern const unsigned int tacna_digital_tlv[];
extern const unsigned int tacna_noise_tlv[];
extern const unsigned int tacna_mixer_tlv[];

void tacna_spin_sysclk(struct tacna_priv *priv);

extern const char * const tacna_rate_text[TACNA_RATE_ENUM_SIZE];
extern const unsigned int tacna_rate_val[TACNA_RATE_ENUM_SIZE];
int tacna_rate_put(struct snd_kcontrol *kcontrol,
		   struct snd_ctl_elem_value *ucontrol);

extern const char * const tacna_sample_rate_text[TACNA_SAMPLE_RATE_ENUM_SIZE];
extern const unsigned int tacna_sample_rate_val[TACNA_SAMPLE_RATE_ENUM_SIZE];
const char *tacna_sample_rate_val_to_name(unsigned int rate_val);
extern const struct soc_enum tacna_sample_rate[];

extern const struct snd_kcontrol_new tacna_inmux[];
extern const struct snd_kcontrol_new tacna_inmode_switch[];

extern const struct soc_enum tacna_in_vi_ramp;
extern const struct soc_enum tacna_in_vd_ramp;
extern const struct soc_enum tacna_in_hpf_cut_enum;
extern const struct soc_enum tacna_in_dmic_osr[];
int tacna_in_rate_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol);
extern const struct soc_enum tacna_input_rate[];
int tacna_low_power_mode_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol);

extern const struct soc_enum tacna_auxpdm1_freq;
extern const struct soc_enum tacna_auxpdm2_freq;
extern const struct soc_enum tacna_auxpdm3_freq;
extern const struct snd_kcontrol_new tacna_auxpdm_inmux[];
extern const struct snd_kcontrol_new tacna_auxpdm_switch[];

extern const struct soc_enum tacna_output_rate;
extern const struct soc_enum tacna_out_vi_ramp;
extern const struct soc_enum tacna_out_vd_ramp;
extern const struct soc_enum tacna_mono_anc_input_src[];
extern const struct soc_enum tacna_anc_ng_enum;
extern const struct soc_enum tacna_output_anc_src[];

int tacna_put_out_vu(struct snd_kcontrol *kcontrol,
		     struct snd_ctl_elem_value *ucontrol);

int tacna_frf_bytes_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol);

extern const struct soc_enum tacna_asrc1_rate[];

extern const struct soc_enum tacna_isrc_fsl[];
extern const struct soc_enum tacna_isrc_fsh[];

extern const struct soc_enum tacna_fx_rate;

extern const struct soc_enum tacna_lhpf1_mode;
extern const struct soc_enum tacna_lhpf2_mode;
extern const struct soc_enum tacna_lhpf3_mode;
extern const struct soc_enum tacna_lhpf4_mode;
int tacna_lhpf_coeff_put(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol);

extern const struct soc_enum tacna_eq_mode[];
int tacna_eq_mode_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol);
int tacna_eq_mode_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol);
int tacna_eq_coeff_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo);
int tacna_eq_coeff_get(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol);
int tacna_eq_coeff_put(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol);

extern const struct snd_kcontrol_new tacna_drc_activity_output_mux[];

extern const char * const tacna_dfc_width_text[TACNA_DFC_WIDTH_ENUM_SIZE];
extern const unsigned int tacna_dfc_width_val[TACNA_DFC_WIDTH_ENUM_SIZE];
extern const char * const tacna_dfc_type_text[TACNA_DFC_TYPE_ENUM_SIZE];
extern const unsigned int tacna_dfc_type_val[TACNA_DFC_TYPE_ENUM_SIZE];
extern const struct soc_enum tacna_dfc_rate[];
extern const struct soc_enum tacna_dfc_width[];
extern const struct soc_enum tacna_dfc_type[];
int tacna_dfc_dith_put(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol);
int tacna_dfc_put(struct snd_kcontrol *kcontrol,
		  struct snd_ctl_elem_value *ucontrol);

extern const struct snd_kcontrol_new tacna_dsp_trigger_output_mux[];
int tacna_dsp_rate_get(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol);
int tacna_dsp_rate_put(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol);
extern const struct snd_kcontrol_new tacna_dsp_rate_controls[];

int tacna_dsp_power_ev(struct snd_soc_dapm_widget *w,
		       struct snd_kcontrol *kcontrol, int event);

extern int tacna_set_sysclk(struct snd_soc_codec *codec, int clk_id,
			    int source, unsigned int freq, int dir);

extern const struct snd_soc_dai_ops tacna_dai_ops;
extern const struct snd_soc_dai_ops tacna_simple_dai_ops;

int tacna_set_output_mode(struct snd_soc_codec *codec, int output, bool diff);
int tacna_get_accdet_for_output(struct snd_soc_codec *codec, int output);

int tacna_sysclk_ev(struct snd_soc_dapm_widget *w,
		    struct snd_kcontrol *kcontrol,
		    int event);
int tacna_domain_clk_ev(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol,
			       int event);
int tacna_in_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol,
		int event);
int tacna_wait_for_output_seq(struct tacna_priv *priv, unsigned int mask,
			      unsigned int val);
int tacna_out_ev(struct snd_soc_dapm_widget *w,
		 struct snd_kcontrol *kcontrol,
		 int event);
int tacna_hp_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol,
		int event);
int tacna_eq_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol,
		int event);
int tacna_anc_ev(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol,
		 int event);

int tacna_fllhj_set_refclk(struct tacna_fll *fll, int source,
			   unsigned int fin, unsigned int fout);
int tacna_init_fll(struct tacna *tacna, int id, int base, unsigned int sts_addr,
		   unsigned int sts_mask, struct tacna_fll *fll);
int tacna_init_inputs(struct snd_soc_codec *codec);
int tacna_init_auxpdm(struct snd_soc_codec *codec, int n_auxpdm);
int tacna_init_outputs(struct snd_soc_codec *codec, int n_mono_routes);
int tacna_init_dai(struct tacna_priv *priv, int dai);
int tacna_init_eq(struct tacna_priv *priv);
int tacna_core_init(struct tacna_priv *priv);
int tacna_core_destroy(struct tacna_priv *priv);

/* Following functions are for use by machine drivers */
static inline int tacna_register_notifier(struct snd_soc_codec *codec,
					  struct notifier_block *nb)
{
	struct tacna *tacna = dev_get_drvdata(codec->dev->parent);

	return blocking_notifier_chain_register(&tacna->notifier, nb);
}

static inline int tacna_unregister_notifier(struct snd_soc_codec *codec,
					    struct notifier_block *nb)
{
	struct tacna *tacna = dev_get_drvdata(codec->dev->parent);

	return blocking_notifier_chain_unregister(&tacna->notifier, nb);
}

#endif
