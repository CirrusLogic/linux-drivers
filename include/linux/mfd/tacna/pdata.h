/*
 * Platform data for Cirrus Logic Tacna codecs
 *
 * Copyright 2016-2018 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef TACNA_PDATA_H
#define TACNA_PDATA_H

#include <linux/extcon/extcon-tacna-pdata.h>
#include <linux/irqchip/irq-tacna-pdata.h>
#include <linux/kernel.h>
#include <linux/regulator/arizona-micsupp.h>
#include <sound/tacna-pdata.h>

#define CS47L96_NUM_GPIOS		27
#define CS48L32_NUM_GPIOS		16
#define CS48LX50_NUM_GPIOS		28

#define TACNA_MAX_MICBIAS		2
#define TACNA_MAX_CHILD_MICBIAS		4

#define TACNA_MAX_GPSW			2

struct pinctrl_map;
struct regulator_init_data;

/**
 * struct tacna_micbias_pin_pdata - MICBIAS pin configuration
 *
 * @init_data: initialization data for the regulator
 */
struct tacna_micbias_pin_pdata {
	struct regulator_init_data *init_data;
	u32 active_discharge;
};

/**
 * struct tacna_micbias_pdata - Regulator configuration for an on-chip MICBIAS
 *
 * @init_data: initialization data for the regulator
 * @ext_cap:   set to true if an external capacitor is fitted
 * @pin:       Configuration for each output pin from this MICBIAS
 */
struct tacna_micbias_pdata {
	struct regulator_init_data *init_data;
	u32 active_discharge;
	bool ext_cap;

	struct tacna_micbias_pin_pdata pin[TACNA_MAX_CHILD_MICBIAS];
};

/**
 * struct tacna_pdata - Configuration for Tacna MFD driver
 *
 * @reset:	    GPIO controlling RESET_B, if any
 * @micvdd:	    Substruct of pdata for the MICSUPP regulator
 *		    (see include/linux/regulator/arizonamicsupp.h)
 * @irqchip:	    Substruct of pdata for the irqchip driver
 * 		    (see include/linux/irqchip/irq-tacna-pdata.h)
 * @gpio_base:	    Base GPIO
 * @gpio_configs:   Array of GPIO configurations for pinctrl
 * @n_gpio_configs: Number of configs in gpio_configs
 * @micbias:	    MICBIAS configurations
 * @codec:	    Substructure of pdata for the ASoC codec driver
 * 		    (see include/sound/tacna-pdata.h)
 * @gpsw:	    General purpose switch mode setting. See the SW1_MODE field
 *		    in the datasheet for for information on the GPSW
 * @accdet:	    Accessory detection configurations
 */
struct tacna_pdata {
	int reset;

	struct arizona_micsupp_pdata micvdd;

	struct tacna_irqchip_pdata irqchip;

	int gpio_base;

	const struct pinctrl_map *gpio_configs;
	int n_gpio_configs;

	struct tacna_micbias_pdata micbias[TACNA_MAX_MICBIAS];

	struct tacna_codec_pdata codec;

	u32 gpsw[TACNA_MAX_GPSW];

	struct tacna_accdet_pdata accdet[TACNA_MAX_ACCESSORY];

	u32 clk32k_src;
};

#endif
