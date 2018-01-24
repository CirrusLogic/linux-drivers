/*
 * madera.h - Cirrus Logic Madera class codecs common support
 *
 * Copyright 2015-2017 Cirrus Logic
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ASOC_MADERA_H
#define ASOC_MADERA_H

#include <linux/completion.h>

#include <sound/soc.h>
#include <sound/madera-pdata.h>

#include "wm_adsp.h"

#define MADERA_FLL1_REFCLK		1
#define MADERA_FLL2_REFCLK		2
#define MADERA_FLL3_REFCLK		3
#define MADERA_FLLAO_REFCLK		4
#define MADERA_FLL1_SYNCCLK		5
#define MADERA_FLL2_SYNCCLK		6
#define MADERA_FLL3_SYNCCLK		7
#define MADERA_FLLAO_SYNCCLK		8

#define MADERA_FLL_SRC_NONE		-1
#define MADERA_FLL_SRC_MCLK1		0
#define MADERA_FLL_SRC_MCLK2		1
#define MADERA_FLL_SRC_SLIMCLK		3
#define MADERA_FLL_SRC_FLL1		4
#define MADERA_FLL_SRC_FLL2		5
#define MADERA_FLL_SRC_AIF1BCLK		8
#define MADERA_FLL_SRC_AIF2BCLK		9
#define MADERA_FLL_SRC_AIF3BCLK		10
#define MADERA_FLL_SRC_AIF4BCLK		11
#define MADERA_FLL_SRC_AIF1LRCLK	12
#define MADERA_FLL_SRC_AIF2LRCLK	13
#define MADERA_FLL_SRC_AIF3LRCLK	14
#define MADERA_FLL_SRC_AIF4LRCLK	15

#define MADERA_CLK_SYSCLK_1		1
#define MADERA_CLK_ASYNCCLK_1		2
#define MADERA_CLK_OPCLK		3
#define MADERA_CLK_ASYNC_OPCLK		4
#define MADERA_CLK_SYSCLK_2		5
#define MADERA_CLK_SYSCLK_3		6
#define MADERA_CLK_ASYNCCLK_2		7
#define MADERA_CLK_DSPCLK		8

#define MADERA_CLK_SRC_MCLK1		0x0
#define MADERA_CLK_SRC_MCLK2		0x1
#define MADERA_CLK_SRC_FLL1		0x4
#define MADERA_CLK_SRC_FLL2		0x5
#define MADERA_CLK_SRC_FLL3		0x6
#define MADERA_CLK_SRC_FLLAO_HI		0x7
#define MADERA_CLK_SRC_FLL1_DIV6	0x7
#define MADERA_CLK_SRC_AIF1BCLK		0x8
#define MADERA_CLK_SRC_AIF2BCLK		0x9
#define MADERA_CLK_SRC_AIF3BCLK		0xA
#define MADERA_CLK_SRC_AIF4BCLK		0xB
#define MADERA_CLK_SRC_FLLAO		0xF

#define MADERA_MIXER_VOL_MASK		0x00FE
#define MADERA_MIXER_VOL_SHIFT		1
#define MADERA_MIXER_VOL_WIDTH		7

#define MADERA_DOM_GRP_FX		0
#define MADERA_DOM_GRP_ASRC1_RATE_1	1
#define MADERA_DOM_GRP_ASRC1_RATE_2	2
#define MADERA_DOM_GRP_ASRC2_RATE_1	3
#define MADERA_DOM_GRP_ASRC2_RATE_2	4
#define MADERA_DOM_GRP_ISRC1_DEC	5
#define MADERA_DOM_GRP_ISRC1_INT	6
#define MADERA_DOM_GRP_ISRC2_DEC	7
#define MADERA_DOM_GRP_ISRC2_INT	8
#define MADERA_DOM_GRP_ISRC3_DEC	9
#define MADERA_DOM_GRP_ISRC3_INT	10
#define MADERA_DOM_GRP_ISRC4_DEC	11
#define MADERA_DOM_GRP_ISRC4_INT	12
#define MADERA_DOM_GRP_OUT		13
#define MADERA_DOM_GRP_SPD		14
#define MADERA_DOM_GRP_DSP1		15
#define MADERA_DOM_GRP_DSP2		16
#define MADERA_DOM_GRP_DSP3		17
#define MADERA_DOM_GRP_DSP4		18
#define MADERA_DOM_GRP_DSP5		19
#define MADERA_DOM_GRP_DSP6		20
#define MADERA_DOM_GRP_DSP7		21
#define MADERA_DOM_GRP_AIF1		22
#define MADERA_DOM_GRP_AIF2		23
#define MADERA_DOM_GRP_AIF3		24
#define MADERA_DOM_GRP_AIF4		25
#define MADERA_DOM_GRP_SLIMBUS		26
#define MADERA_DOM_GRP_PWM		27
#define MADERA_DOM_GRP_DFC		28
#define MADERA_N_DOM_GRPS		29

#define MADERA_MAX_DAI			11
#define MADERA_MAX_ADSP			7

#define MADERA_NUM_MIXER_INPUTS		146

struct madera;
struct wm_adsp;

struct madera_voice_trigger_info {
	/** Which core triggered, 1-based (1 = DSP1, ...) */
	int core_num;
};

struct madera_dai_priv {
	int clk;
	struct snd_pcm_hw_constraint_list constraint;
};

struct madera_priv {
	struct wm_adsp adsp[MADERA_MAX_ADSP];
	struct madera *madera;
	struct device *dev;
	int sysclk;
	int asyncclk;
	int dspclk;
	struct madera_dai_priv dai[MADERA_MAX_DAI];

	int num_inputs;

	unsigned int in_pending;

	unsigned int out_up_pending;
	unsigned int out_up_delay;
	unsigned int out_down_pending;
	unsigned int out_down_delay;

	unsigned int adsp_rate_cache[MADERA_MAX_ADSP];

	struct mutex rate_lock;

	int tdm_width[MADERA_MAX_AIF];
	int tdm_slots[MADERA_MAX_AIF];

	int domain_group_ref[MADERA_N_DOM_GRPS];
};

struct madera_fll_cfg {
	int n;
	unsigned int theta;
	unsigned int lambda;
	int refdiv;
	int fratio;
	int gain;
	int alt_gain;
};

struct madera_fll {
	struct madera *madera;
	int id;
	unsigned int base;

	unsigned int fout;

	int sync_src;
	unsigned int sync_freq;

	int ref_src;
	unsigned int ref_freq;
	struct madera_fll_cfg ref_cfg;
};

struct madera_enum {
	struct soc_enum mixer_enum;
	int val;
};

extern const unsigned int madera_ana_tlv[];
extern const unsigned int madera_eq_tlv[];
extern const unsigned int madera_digital_tlv[];
extern const unsigned int madera_noise_tlv[];
extern const unsigned int madera_ng_tlv[];

extern const unsigned int madera_mixer_tlv[];
extern const char * const madera_mixer_texts[MADERA_NUM_MIXER_INPUTS];
extern unsigned int madera_mixer_values[MADERA_NUM_MIXER_INPUTS];

#define MADERA_GAINMUX_CONTROLS(name, base) \
	SOC_SINGLE_RANGE_TLV(name " Input Volume", base + 1,		\
			     MADERA_MIXER_VOL_SHIFT, 0x20, 0x50, 0,	\
			     madera_mixer_tlv)

#define MADERA_MIXER_CONTROLS(name, base) \
	SOC_SINGLE_RANGE_TLV(name " Input 1 Volume", base + 1,		\
			     MADERA_MIXER_VOL_SHIFT, 0x20, 0x50, 0,	\
			     madera_mixer_tlv),			\
	SOC_SINGLE_RANGE_TLV(name " Input 2 Volume", base + 3,		\
			     MADERA_MIXER_VOL_SHIFT, 0x20, 0x50, 0,	\
			     madera_mixer_tlv),			\
	SOC_SINGLE_RANGE_TLV(name " Input 3 Volume", base + 5,		\
			     MADERA_MIXER_VOL_SHIFT, 0x20, 0x50, 0,	\
			     madera_mixer_tlv),			\
	SOC_SINGLE_RANGE_TLV(name " Input 4 Volume", base + 7,		\
			     MADERA_MIXER_VOL_SHIFT, 0x20, 0x50, 0,	\
			     madera_mixer_tlv)

#define MADERA_MUX_ENUM_DECL(name, reg) \
	SOC_VALUE_ENUM_SINGLE_AUTODISABLE_DECL( \
		name, reg, 0, 0xff, madera_mixer_texts, madera_mixer_values)

#define MADERA_MUX_CTL_DECL(name) \
	const struct snd_kcontrol_new name##_mux =	\
		SOC_DAPM_ENUM("Route", name##_enum)

#define MADERA_MUX_ENUMS(name, base_reg) \
	static MADERA_MUX_ENUM_DECL(name##_enum, base_reg);	\
	static MADERA_MUX_CTL_DECL(name)

#define MADERA_MIXER_ENUMS(name, base_reg) \
	MADERA_MUX_ENUMS(name##_in1, base_reg);     \
	MADERA_MUX_ENUMS(name##_in2, base_reg + 2); \
	MADERA_MUX_ENUMS(name##_in3, base_reg + 4); \
	MADERA_MUX_ENUMS(name##_in4, base_reg + 6)

#define MADERA_DSP_AUX_ENUMS(name, base_reg) \
	MADERA_MUX_ENUMS(name##_aux1, base_reg);	\
	MADERA_MUX_ENUMS(name##_aux2, base_reg + 8);	\
	MADERA_MUX_ENUMS(name##_aux3, base_reg + 16);	\
	MADERA_MUX_ENUMS(name##_aux4, base_reg + 24);	\
	MADERA_MUX_ENUMS(name##_aux5, base_reg + 32);	\
	MADERA_MUX_ENUMS(name##_aux6, base_reg + 40)

#define MADERA_MUX(name, ctrl) \
	SND_SOC_DAPM_MUX(name, SND_SOC_NOPM, 0, 0, ctrl)

#define MADERA_MUX_WIDGETS(name, name_str) \
	MADERA_MUX(name_str " Input 1", &name##_mux)

#define MADERA_MIXER_WIDGETS(name, name_str)	\
	MADERA_MUX(name_str " Input 1", &name##_in1_mux), \
	MADERA_MUX(name_str " Input 2", &name##_in2_mux), \
	MADERA_MUX(name_str " Input 3", &name##_in3_mux), \
	MADERA_MUX(name_str " Input 4", &name##_in4_mux), \
	SND_SOC_DAPM_MIXER(name_str " Mixer", SND_SOC_NOPM, 0, 0, NULL, 0)

#define MADERA_DSP_WIDGETS(name, name_str)			\
	MADERA_MIXER_WIDGETS(name##L, name_str "L"),		\
	MADERA_MIXER_WIDGETS(name##R, name_str "R"),		\
	MADERA_MUX(name_str " Aux 1", &name##_aux1_mux),	\
	MADERA_MUX(name_str " Aux 2", &name##_aux2_mux),	\
	MADERA_MUX(name_str " Aux 3", &name##_aux3_mux),	\
	MADERA_MUX(name_str " Aux 4", &name##_aux4_mux),	\
	MADERA_MUX(name_str " Aux 5", &name##_aux5_mux),	\
	MADERA_MUX(name_str " Aux 6", &name##_aux6_mux)

#define MADERA_MUX_ROUTES(widget, name) \
	{ widget, NULL, name " Input 1" }, \
	MADERA_MIXER_INPUT_ROUTES(name " Input 1")

#define MADERA_MIXER_ROUTES(widget, name)		\
	{ widget, NULL, name " Mixer" },		\
	{ name " Mixer", NULL, name " Input 1" },	\
	{ name " Mixer", NULL, name " Input 2" },	\
	{ name " Mixer", NULL, name " Input 3" },	\
	{ name " Mixer", NULL, name " Input 4" },	\
	MADERA_MIXER_INPUT_ROUTES(name " Input 1"),	\
	MADERA_MIXER_INPUT_ROUTES(name " Input 2"),	\
	MADERA_MIXER_INPUT_ROUTES(name " Input 3"),	\
	MADERA_MIXER_INPUT_ROUTES(name " Input 4")

#define MADERA_DSP_ROUTES(name)				\
	{ name, NULL, name " Preloader"},		\
	{ name " Preloader", NULL, "SYSCLK"},		\
	{ name " Preloader", NULL, "DSPCLK"},		\
	{ name, NULL, name " Aux 1" },			\
	{ name, NULL, name " Aux 2" },			\
	{ name, NULL, name " Aux 3" },			\
	{ name, NULL, name " Aux 4" },			\
	{ name, NULL, name " Aux 5" },			\
	{ name, NULL, name " Aux 6" },			\
	MADERA_MIXER_INPUT_ROUTES(name " Aux 1"),	\
	MADERA_MIXER_INPUT_ROUTES(name " Aux 2"),	\
	MADERA_MIXER_INPUT_ROUTES(name " Aux 3"),	\
	MADERA_MIXER_INPUT_ROUTES(name " Aux 4"),	\
	MADERA_MIXER_INPUT_ROUTES(name " Aux 5"),	\
	MADERA_MIXER_INPUT_ROUTES(name " Aux 6"),	\
	MADERA_MIXER_ROUTES(name, name "L"),		\
	MADERA_MIXER_ROUTES(name, name "R")

#define MADERA_SAMPLE_RATE_CONTROL(name, domain) \
	SOC_ENUM(name, madera_sample_rate[(domain) - 2])

#define MADERA_RATE_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,\
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_get_enum_double, .put = madera_rate_put, \
	.private_value = (unsigned long)&xenum }

#define MADERA_EQ_CONTROL(xname, xbase)				\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
	.info = snd_soc_bytes_info, .get = snd_soc_bytes_get,	\
	.put = madera_eq_coeff_put, .private_value =		\
	((unsigned long)&(struct soc_bytes) { .base = xbase,	\
	 .num_regs = 20, .mask = ~MADERA_EQ1_B1_MODE }) }

#define MADERA_LHPF_CONTROL(xname, xbase)			\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
	.info = snd_soc_bytes_info, .get = snd_soc_bytes_get,	\
	.put = madera_lhpf_coeff_put, .private_value =		\
	((unsigned long)&(struct soc_bytes) { .base = xbase,	\
	 .num_regs = 1 }) }

#define MADERA_RATES SNDRV_PCM_RATE_KNOT

#define MADERA_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

#define MADERA_OSR_ENUM_SIZE		5
#define MADERA_SYNC_RATE_ENUM_SIZE	3
#define MADERA_ASYNC_RATE_ENUM_SIZE	2
#define MADERA_RATE_ENUM_SIZE \
		(MADERA_SYNC_RATE_ENUM_SIZE + MADERA_ASYNC_RATE_ENUM_SIZE)
#define MADERA_SAMPLE_RATE_ENUM_SIZE	16
#define MADERA_DFC_TYPE_ENUM_SIZE	5
#define MADERA_DFC_WIDTH_ENUM_SIZE	5

extern const struct snd_soc_dai_ops madera_dai_ops;
extern const struct snd_soc_dai_ops madera_simple_dai_ops;

extern const struct snd_kcontrol_new madera_inmux[];

extern const char * const madera_rate_text[MADERA_RATE_ENUM_SIZE];
extern const unsigned int madera_rate_val[MADERA_RATE_ENUM_SIZE];
extern const char * const madera_sample_rate_text[MADERA_SAMPLE_RATE_ENUM_SIZE];
extern const unsigned int madera_sample_rate_val[MADERA_SAMPLE_RATE_ENUM_SIZE];
extern const char * const madera_dfc_width_text[MADERA_DFC_WIDTH_ENUM_SIZE];
extern const unsigned int madera_dfc_width_val[MADERA_DFC_WIDTH_ENUM_SIZE];
extern const char * const madera_dfc_type_text[MADERA_DFC_TYPE_ENUM_SIZE];
extern const unsigned int madera_dfc_type_val[MADERA_DFC_TYPE_ENUM_SIZE];

extern const struct soc_enum madera_sample_rate[];
extern const struct soc_enum madera_isrc_fsl[];
extern const struct soc_enum madera_isrc_fsh[];
extern const struct soc_enum madera_asrc1_rate[];
extern const struct soc_enum madera_asrc2_rate[];
extern const struct soc_enum madera_dfc_width[];
extern const struct soc_enum madera_dfc_type[];
extern const struct soc_enum madera_spdif_rate;

extern const struct soc_enum madera_in_vi_ramp;
extern const struct soc_enum madera_in_vd_ramp;

extern const struct soc_enum madera_out_vi_ramp;
extern const struct soc_enum madera_out_vd_ramp;

extern const struct soc_enum madera_lhpf1_mode;
extern const struct soc_enum madera_lhpf2_mode;
extern const struct soc_enum madera_lhpf3_mode;
extern const struct soc_enum madera_lhpf4_mode;

extern const struct soc_enum madera_ng_hold;
extern const struct soc_enum madera_in_hpf_cut_enum;
extern const struct soc_enum madera_in_dmic_osr[];

extern const struct soc_enum madera_output_anc_src[];
extern const struct soc_enum madera_anc_input_src[];
extern const struct soc_enum madera_anc_ng_enum;

extern const struct snd_kcontrol_new madera_dsp_trigger_output_mux[];
extern const struct snd_kcontrol_new madera_drc_activity_output_mux[];

extern const struct snd_kcontrol_new madera_adsp_rate_controls[];

const char *madera_sample_rate_val_to_name(unsigned int rate_val);

int madera_dfc_put(struct snd_kcontrol *kcontrol,
		   struct snd_ctl_elem_value *ucontrol);

int madera_lp_mode_put(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol);

int madera_out1_demux_put(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol);

int madera_dre_put(struct snd_kcontrol *kcontrol,
		   struct snd_ctl_elem_value *ucontrol);

int madera_rate_put(struct snd_kcontrol *kcontrol,
		    struct snd_ctl_elem_value *ucontrol);

int madera_eq_coeff_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol);
int madera_lhpf_coeff_put(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol);

int madera_sysclk_ev(struct snd_soc_dapm_widget *w,
		     struct snd_kcontrol *kcontrol, int event);
int madera_spk_ev(struct snd_soc_dapm_widget *w,
		  struct snd_kcontrol *kcontrol, int event);
int madera_in_ev(struct snd_soc_dapm_widget *w,
		 struct snd_kcontrol *kcontrol, int event);
int madera_out_ev(struct snd_soc_dapm_widget *w,
		  struct snd_kcontrol *kcontrol, int event);
int madera_hp_ev(struct snd_soc_dapm_widget *w,
		 struct snd_kcontrol *kcontrol, int event);
int madera_anc_ev(struct snd_soc_dapm_widget *w,
		  struct snd_kcontrol *kcontrol, int event);
int madera_domain_clk_ev(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol,
			 int event);

int madera_adsp_rate_info(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_info *uinfo);
int madera_adsp_rate_get(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol);
int madera_adsp_rate_put(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol);
int madera_set_adsp_clk(struct madera_priv *priv, int dsp_num,
			unsigned int freq);

int madera_set_sysclk(struct snd_soc_codec *codec, int clk_id, int source,
		      unsigned int freq, int dir);
int madera_get_legacy_dspclk_setting(struct madera *madera, unsigned int freq);
void madera_spin_sysclk(struct madera_priv *priv);

int madera_init_fll(struct madera *madera, int id, int base,
		    struct madera_fll *fll);
int madera_set_fll_refclk(struct madera_fll *fll, int source,
			  unsigned int Fref, unsigned int Fout);
int madera_set_fll_syncclk(struct madera_fll *fll, int source,
			   unsigned int Fref, unsigned int Fout);
int madera_set_fll_ao_refclk(struct madera_fll *fll, int source,
			     unsigned int fin, unsigned int fout);
int madera_fllhj_set_refclk(struct madera_fll *fll, int source,
			    unsigned int fin, unsigned int fout);

int madera_core_init(struct madera_priv *priv);
int madera_core_destroy(struct madera_priv *priv);
int madera_init_overheat(struct madera_priv *priv);
int madera_free_overheat(struct madera_priv *priv);
int madera_init_inputs(struct snd_soc_codec *codec,
		       const char * const *dmic_inputs,
		       int n_dmic_inputs,
		       const char * const *dmic_refs,
		       int n_dmic_refs);
int madera_init_outputs(struct snd_soc_codec *codec, int n_mono_routes);
int madera_init_bus_error_irq(struct madera_priv *priv, int dsp_num,
			      irq_handler_t handler);
void madera_destroy_bus_error_irq(struct madera_priv *priv, int dsp_num);

int madera_init_dai(struct madera_priv *priv, int dai);

int madera_set_output_mode(struct snd_soc_codec *codec, int output, bool diff);

/* Following functions are for use by machine drivers */
static inline int madera_register_notifier(struct snd_soc_codec *codec,
					   struct notifier_block *nb)
{
	struct madera *madera = dev_get_drvdata(codec->dev->parent);

	return blocking_notifier_chain_register(&madera->notifier, nb);
}

static inline int madera_unregister_notifier(struct snd_soc_codec *codec,
					     struct notifier_block *nb)
{
	struct madera *madera = dev_get_drvdata(codec->dev->parent);

	return blocking_notifier_chain_unregister(&madera->notifier, nb);
}

#endif
