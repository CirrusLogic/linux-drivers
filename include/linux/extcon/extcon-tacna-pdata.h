/*
 * Platform data for Tacna codecs
 *
 * Copyright 2017-2018 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef EXTCON_TACNA_PDATA_H
#define EXTCON_TACNA_PDATA_H

#include <linux/kernel.h>

#define TACNA_MAX_ACCESSORY		1

/* Treat INT_MAX impedance as open circuit */
#define TACNA_HP_Z_OPEN			INT_MAX

struct tacna_jd_state;

/**
 * struct tacna_micd_config - Bias and sense configuration for probing MICDET
 *
 * @src:	value of MICDn_SENSE_SEL field. See datasheet for the field values
 *			applicable to the codec you are using
 * @gnd:	Value of MICDn_GND_SEL. See datasheet for the field values
 *			applicable to the codec you are using
 * @bias:	Value of MICDn_BIAS_SRC. See datasheet for the field values
 *			applicable to the codec you are using
 * @gpio:	State of polarity gpio during probe (true == gpio asserted)
 * @hp_gnd:	Value of HPn_GND_SEL. See datasheet for the field values
 *			applicable to the codec you are using
 */
struct tacna_micd_config {
	u32 src;
	u32 gnd;
	u32 bias;
	bool gpio;
	u32 hp_gnd;
};

/**
 * struct tacna_micd_range - Definition of button detect range
 * @max: Range max value in ohms
 * @key: Key to report to input layer for this button
 */
struct tacna_micd_range {
	int max;
	int key;
};

/**
 * struct tacna_accdet_pdata - pdata for accessory detect configuration
 *
 * @enabled:		       set to true to enable this accessory detect
 * @output:		       which output this accdet is for (1 = OUT1, ...)
 * @fixed_hpdet_imp_x100:      If non-zero don't run headphone detection, just
 *			       report this value. Hundredths-of-an-ohm.
 * @hpdet_ext_res_x100:	       Impedance of external series resistor on hpdet.
 *			       In hundredths-of-an-ohm.
 * @hpdet_short_circuit_imp:   If non-zero, specifies the maximum impedance in
 *			       ohms that will be considered as a short circuit
 * @hpdet_channel:	       Channel to use for headphone detection, valid
 *			       values are 0 for left and 1 for right.
 * @micd_detect_debounce_ms:   Extra debounce timeout during initial mic detect
 *			       (milliseconds)
 * @micd_manual_debounce:      Extra software debounces during button detection
 * @micd_pol_gpio:	       GPIO for mic detection polarity
 * @micd_bias_start_time:      Mic detect startup ramp time
 * @micd_rate:		       Setting for the codec MICDn_RATE field. See the
 *			       datasheet for documentation of the field values
 * @micd_dbtime:	       Mic detect debounce level
 * @micd_timeout_ms:	       Mic detect timeout (milliseconds)
 * @micd_clamp_mode:	       Mic detect clamp function
 * @micd_open_circuit_declare: Declare an open circuit as a 4 pole jack
 * @micd_software_compare:     Use software comparison to determine mic presence
 * @micd_ranges:	       Mic detect level parameters
 * @num_micd_ranges:	       Number of ranges in micd_ranges
 * @micd_configs:	       Headset polarity configurations
 * @num_micd_configs:	       Number of configurations in micd_configs
 * @hpd_pins:		       4 entries:
 *				[HPL_output_pin (value of HPD_OUT_SEL),
 *				 HPDL_sense_pin (value of HPD_SENSE_SEL),
 *				 HPR_output_pin (value of HPD_OUT_SEL),
 *				 HPDR_sense_pin (value of HPD_SENSE_SEL)]
 *				See datasheet for field values
 * @custom_jd:		       Override the normal jack detection
 */
struct tacna_accdet_pdata {
	bool enabled;
	u32 output;

	u32 fixed_hpdet_imp_x100;
	u32 hpdet_ext_res_x100;
	u32 hpdet_short_circuit_imp;

	u32 hpdet_channel;

	u32 micd_detect_debounce_ms;
	u32 micd_manual_debounce;
	int micd_pol_gpio;
	u32 micd_bias_start_time;
	u32 micd_rate;
	u32 micd_dbtime;
	u32 micd_timeout_ms;
	u32 micd_clamp_mode;
	bool micd_open_circuit_declare;
	bool micd_software_compare;

	const struct tacna_micd_range *micd_ranges;
	int num_micd_ranges;

	const struct tacna_micd_config *micd_configs;
	int num_micd_configs;

	u32 hpd_pins[4];

	const struct tacna_jd_state *custom_jd;

	u32 accdet_dbtime;
};

#endif
