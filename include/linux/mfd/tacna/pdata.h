/*
 * Platform data for Cirrus Logic Tacna codecs
 *
 * Copyright 2016-2017 Cirrus Logic
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef TACNA_PDATA_H
#define TACNA_PDATA_H

#include <linux/kernel.h>

#include <linux/irqchip/irq-tacna-pdata.h>
#include <linux/regulator/arizona-micsupp.h>
/*
#include <linux/extcon/extcon-madera-pdata.h>
*/
#include <linux/irqchip/irq-tacna-pdata.h>
#include <sound/tacna-pdata.h>

#define CS47L94_NUM_GPIOS		23

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

struct tacna_pdata {
	/** GPIO controlling RESET_B, if any */
	int reset;

	/** Substruct of pdata for the MICSUPP regulator */
	struct arizona_micsupp_pdata micvdd;

	/** Substruct of pdata for the irqchip driver */
	struct tacna_irqchip_pdata irqchip;

	/** Base GPIO */
	int gpio_base;

	/**
	 * Array of GPIO configurations
	 * See Documentation/pinctrl.txt
	 */
	const struct pinctrl_map *gpio_configs;
	int n_gpio_configs;

	/** MICBIAS configurations */
	struct tacna_micbias_pdata micbias[TACNA_MAX_MICBIAS];

	/**
	 * Substructure of pdata for the ASoC codec driver
	 * See include/sound/tacna-pdata.h
	 */
	struct tacna_codec_pdata codec;

	/**
	 * General purpose switch mode setting
	 * See the SW1_MODE field in the datasheet for the available values
	 */
	u32 gpsw[TACNA_MAX_GPSW];

	/** Accessory detection configurations */
	/*struct tacna_accdet_pdata accdet[TACNA_MAX_ACCESSORY];*/
};

#endif
