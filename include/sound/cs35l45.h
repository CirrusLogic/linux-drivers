/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * linux/sound/cs35l45.h -- Platform data for CS35L45
 *
 * Copyright 2019 Cirrus Logic, Inc.
 *
 * Author: James Schulman <james.schulman@cirrus.com>
 *
 */

#ifndef __CS35L45_H
#define __CS35L45_H

#define CS35L45_NUM_SUPPLIES 2

struct bst_bpe_iv_pair {
	unsigned int volt;
	unsigned int amp;
};

struct bst_bpe_voltage_config {
	bool is_present;
	struct bst_bpe_iv_pair l1;
	struct bst_bpe_iv_pair l2;
	struct bst_bpe_iv_pair l3;
	struct bst_bpe_iv_pair l4;
};

struct bst_bpe_ind_curr_config {
	bool is_present;
	unsigned int bst_bpe_il_lim_thld_del1;
	unsigned int bst_bpe_il_lim_thld_del2;
	unsigned int bst_bpe_il_lim1_thld;
	unsigned int bst_bpe_il_lim1_dly;
	unsigned int bst_bpe_il_lim2_dly;
	unsigned int bst_bpe_il_lim_dly_hyst;
	unsigned int bst_bpe_il_lim_thld_hyst;
	unsigned int bst_bpe_il_lim1_atk_rate;
	unsigned int bst_bpe_il_lim2_atk_rate;
	unsigned int bst_bpe_il_lim1_rls_rate;
	unsigned int bst_bpe_il_lim2_rls_rate;
};

struct hvlv_config {
	bool is_present;
	unsigned int hvlv_thld_hys;
	unsigned int hvlv_thld;
	unsigned int hvlv_dly;
};

struct ldpm_config {
	bool is_present;
	unsigned int ldpm_gp1_boost_sel;
	unsigned int ldpm_gp1_amp_sel;
	unsigned int ldpm_gp1_delay;
	unsigned int ldpm_gp1_pcm_thld;
	unsigned int ldpm_gp2_imon_sel;
	unsigned int ldpm_gp2_vmon_sel;
	unsigned int ldpm_gp2_delay;
	unsigned int ldpm_gp2_pcm_thld;
};

struct classh_config {
	bool is_present;
	unsigned int ch_hdrm;
	unsigned int ch_ratio;
	unsigned int ch_rel_rate;
	unsigned int ch_ovb_thld1;
	unsigned int ch_ovb_thlddelta;
	unsigned int ch_vdd_bst_max;
	unsigned int ch_ovb_setting_latch;
	unsigned int ch_ovb_ratio;
	unsigned int ch_thld1_offset;
	unsigned int aud_mem_depth;
};

struct gpio_ctrl {
	bool is_present;
	unsigned int dir;
	unsigned int lvl;
	unsigned int op_cfg;
	unsigned int pol;
	unsigned int ctrl;
	unsigned int invert;
};

struct cs35l45_irq_monitor {
	unsigned int reg;
	unsigned int mask;
	unsigned int bitmask;
	const char *description;
	int (*callback)(struct cs35l45_private *cs35l45);
};

struct cs35l45_platform_data {
	struct bst_bpe_voltage_config bst_bpe_voltage_cfg;
	struct bst_bpe_ind_curr_config bst_bpe_ind_curr_cfg;
	struct hvlv_config hvlv_cfg;
	struct ldpm_config ldpm_cfg;
	struct classh_config classh_cfg;
	struct gpio_ctrl gpio_ctrl1;
	struct gpio_ctrl gpio_ctrl2;
	struct gpio_ctrl gpio_ctrl3;
	const char *dsp_part_name;
	unsigned int asp_sdout_hiz_ctrl;
	bool use_tdm_slots;
};

struct cs35l45_private {
	struct wm_adsp dsp; /* needs to be first member */
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[CS35L45_NUM_SUPPLIES];
	struct cs35l45_platform_data pdata;
	struct mutex rate_lock;
	bool initialized;
	int irq;
	int slot_width;
};

int cs35l45_initialize(struct cs35l45_private *cs35l45);
int cs35l45_probe(struct cs35l45_private *cs35l45);
int cs35l45_remove(struct cs35l45_private *cs35l45);

struct of_entry {
	const char *name;
	unsigned int reg;
	unsigned int mask;
	unsigned int shift;
};

enum bst_bpe_voltage_of_param {
	L1 = 0,
	L2,
	L3,
	L4,
	BST_BPE_VOLTAGE_PARAMS
};

enum bst_bpe_ind_curr_of_param {
	BST_BPE_IL_LIM_THLD_DEL1 = 0,
	BST_BPE_IL_LIM_THLD_DEL2,
	BST_BPE_IL_LIM1_THLD,
	BST_BPE_IL_LIM1_DLY,
	BST_BPE_IL_LIM2_DLY,
	BST_BPE_IL_LIM_DLY_HYST,
	BST_BPE_IL_LIM_THLD_HYST,
	BST_BPE_IL_LIM1_ATK_RATE,
	BST_BPE_IL_LIM2_ATK_RATE,
	BST_BPE_IL_LIM1_RLS_RATE,
	BST_BPE_IL_LIM2_RLS_RATE,
	BST_BPE_IND_CURR_PARAMS
};

enum ldpm_of_param {
	LDPM_GP1_BOOST_SEL = 0,
	LDPM_GP1_AMP_SEL,
	LDPM_GP1_DELAY,
	LDPM_GP1_PCM_THLD,
	LDPM_GP2_IMON_SEL,
	LDPM_GP2_VMON_SEL,
	LDPM_GP2_DELAY,
	LDPM_GP2_PCM_THLD,
	LDPM_PARAMS
};

enum classh_of_param {
	CH_HDRM = 0,
	CH_RATIO,
	CH_REL_RATE,
	CH_OVB_THLD1,
	CH_OVB_THLDDELTA,
	CH_VDD_BST_MAX,
	CH_OVB_LATCH,
	CH_OVB_RATIO,
	CH_THLD1_OFFSET,
	AUD_MEM_DEPTH,
	CLASSH_PARAMS
};

static inline u32 *cs35l45_get_bst_bpe_ind_curr_param(
					struct cs35l45_private *cs35l45,
					enum bst_bpe_ind_curr_of_param param)
{
	struct bst_bpe_ind_curr_config *cfg =
			&cs35l45->pdata.bst_bpe_ind_curr_cfg;

	switch (param) {
	case BST_BPE_IL_LIM_THLD_DEL1:
		return &cfg->bst_bpe_il_lim_thld_del1;
	case BST_BPE_IL_LIM_THLD_DEL2:
		return &cfg->bst_bpe_il_lim_thld_del2;
	case BST_BPE_IL_LIM1_THLD:
		return &cfg->bst_bpe_il_lim1_thld;
	case BST_BPE_IL_LIM1_DLY:
		return &cfg->bst_bpe_il_lim1_dly;
	case BST_BPE_IL_LIM2_DLY:
		return &cfg->bst_bpe_il_lim2_dly;
	case BST_BPE_IL_LIM_DLY_HYST:
		return &cfg->bst_bpe_il_lim_dly_hyst;
	case BST_BPE_IL_LIM_THLD_HYST:
		return &cfg->bst_bpe_il_lim_thld_hyst;
	case BST_BPE_IL_LIM1_ATK_RATE:
		return &cfg->bst_bpe_il_lim1_atk_rate;
	case BST_BPE_IL_LIM2_ATK_RATE:
		return &cfg->bst_bpe_il_lim2_atk_rate;
	case BST_BPE_IL_LIM1_RLS_RATE:
		return &cfg->bst_bpe_il_lim1_rls_rate;
	case BST_BPE_IL_LIM2_RLS_RATE:
		return &cfg->bst_bpe_il_lim2_rls_rate;
	default:
		return NULL;
	}
}

static inline u32 *cs35l45_get_ldpm_param(struct cs35l45_private *cs35l45,
					  enum ldpm_of_param param)
{
	struct ldpm_config *cfg = &cs35l45->pdata.ldpm_cfg;

	switch (param) {
	case LDPM_GP1_BOOST_SEL:
		return &cfg->ldpm_gp1_boost_sel;
	case LDPM_GP1_AMP_SEL:
		return &cfg->ldpm_gp1_amp_sel;
	case LDPM_GP1_DELAY:
		return &cfg->ldpm_gp1_delay;
	case LDPM_GP1_PCM_THLD:
		return &cfg->ldpm_gp1_pcm_thld;
	case LDPM_GP2_IMON_SEL:
		return &cfg->ldpm_gp2_imon_sel;
	case LDPM_GP2_VMON_SEL:
		return &cfg->ldpm_gp2_vmon_sel;
	case LDPM_GP2_DELAY:
		return &cfg->ldpm_gp2_delay;
	case LDPM_GP2_PCM_THLD:
		return &cfg->ldpm_gp2_pcm_thld;
	default:
		return NULL;
	}
}

static inline u32 *cs35l45_get_classh_param(struct cs35l45_private *cs35l45,
					    enum classh_of_param param)
{
	struct classh_config *cfg = &cs35l45->pdata.classh_cfg;

	switch (param) {
	case CH_HDRM:
		return &cfg->ch_hdrm;
	case CH_RATIO:
		return &cfg->ch_ratio;
	case CH_REL_RATE:
		return &cfg->ch_rel_rate;
	case CH_OVB_THLD1:
		return &cfg->ch_ovb_thld1;
	case CH_OVB_THLDDELTA:
		return &cfg->ch_ovb_thlddelta;
	case CH_VDD_BST_MAX:
		return &cfg->ch_vdd_bst_max;
	case CH_OVB_LATCH:
		return &cfg->ch_ovb_setting_latch;
	case CH_OVB_RATIO:
		return &cfg->ch_ovb_ratio;
	case CH_THLD1_OFFSET:
		return &cfg->ch_thld1_offset;
	case AUD_MEM_DEPTH:
		return &cfg->aud_mem_depth;
	default:
		return NULL;
	}
}

#endif /* __CS35L45_H */
