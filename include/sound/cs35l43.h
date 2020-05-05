/* SPDX-License-Identifier: GPL-2.0 */

/*
 * linux/sound/cs35l43.h -- Platform data for CS35L43
 *
 * Copyright (c) 2021 Cirrus Logic Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CS35L43_H
#define __CS35L43_H


#define CS35L43_RX_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)
#define CS35L43_TX_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE \
				| SNDRV_PCM_FMTBIT_S32_LE)

#define CS35L43_VALID_PDATA		0x80000000

struct cs35l43_platform_data {
	bool gpio1_out_enable;
	bool gpio2_out_enable;
	bool classh_disable;
	bool dsp_ng_enable;
	int dsp_ng_pcm_thld;
	int dsp_ng_delay;
	int dout_hiz;
	int bst_vctrl;
	int hw_ng_sel;
	int hw_ng_delay;
	int hw_ng_thld;
	int gpio1_src_sel;
	int gpio2_src_sel;
};

struct cs35l43_pll_sysclk_config {
	int freq;
	int clk_cfg;
};

extern const struct cs35l43_pll_sysclk_config cs35l43_pll_sysclk[64];

struct cs35l43_private {
	struct snd_soc_codec *codec;
	struct cs35l43_platform_data pdata;
	struct device *dev;
	struct regmap *regmap;
	struct regulator_bulk_data supplies[2];
	int num_supplies;
	int irq;
	int extclk_cfg;
	bool i2s_mode;
	bool dspa_mode;
	/* GPIO for /RST */
	struct gpio_desc *reset_gpio;
};

int cs35l43_probe(struct cs35l43_private *cs35l43,
				struct cs35l43_platform_data *pdata);
int cs35l43_remove(struct cs35l43_private *cs35l43);

bool cs35l43_readable_reg(struct device *dev, unsigned int reg);
bool cs35l43_precious_reg(struct device *dev, unsigned int reg);
bool cs35l43_volatile_reg(struct device *dev, unsigned int reg);

extern const struct reg_default cs35l43_reg[1];

#endif /* __CS35L43_H */
