/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Platform data for Madera codec driver
 *
 * Copyright (C) 2016-2019 Cirrus Logic, Inc. and
 *                         Cirrus Logic International Semiconductor Ltd.
 */

#ifndef MADERA_CODEC_PDATA_H
#define MADERA_CODEC_PDATA_H

#include <linux/types.h>

#define MADERA_MAX_INPUT		6
#define MADERA_MAX_MUXED_CHANNELS	4
#define MADERA_MAX_OUTPUT		6
#define MADERA_MAX_AIF			4
#define MADERA_MAX_PDM_SPK		2
#define MADERA_MAX_DSP			7

#define MADERA_MAX_CHILD_MICBIAS	4

/**
 * struct madera_codec_pdata
 *
 * @max_channels_clocked: Maximum number of channels that I2S clocks will be
 *			  generated for. Useful when clock master for systems
 *			  where the I2S bus has multiple data lines.
 * @dmic_ref:		  Indicates how the MICBIAS pins have been externally
 *			  connected to DMICs on each input. A value of 0
 *			  indicates MICVDD and is the default. Other values are:
 *			  for all codecs one of the MADERA_DMIC_REF_xxx
 *			  Also see the datasheet for a description of the
 *			  INn_DMIC_SUP field.
 * @inmode:		  Mode for the ADC inputs. One of the MADERA_INMODE_xxx
 *			  values. Two-dimensional array
 *			  [input_number][channel number], with four slots per
 *			  input in the order
 *			  [n][0]=INnAL [n][1]=INnAR [n][2]=INnBL [n][3]=INnBR
 * @out_mono:		  For each output set the value to TRUE to indicate that
 *			  the output is mono. [0]=OUT1, [1]=OUT2, ...
 * @pdm_fmt:		  PDM speaker data format. See the PDM_SPKn_FMT field in
 *			  the datasheet for a description of this value.
 * @pdm_mute:		  PDM mute format. See the PDM_SPKn_CTRL_1 register
 *			  in the datasheet for a description of this value.
 */
struct madera_codec_pdata {
	u32 max_channels_clocked[MADERA_MAX_AIF];

	u32 dmic_ref[MADERA_MAX_INPUT];

	u32 inmode[MADERA_MAX_INPUT][MADERA_MAX_MUXED_CHANNELS];

	bool out_mono[MADERA_MAX_OUTPUT];

	u32 pdm_fmt[MADERA_MAX_PDM_SPK];
	u32 pdm_mute[MADERA_MAX_PDM_SPK];
};

#define MADERA_MAX_ACCESSORY		1

/* Treat INT_MAX impedance as open circuit */
#define MADERA_HP_Z_OPEN		INT_MAX

struct madera_jd_state;

/** Bias and sense configuration for probing MICDET */
struct madera_micd_config {
	/** Value of MICDn_SENSE_SEL field
	 *  See datasheet for field values
	 */
	u32 src;

	/** Value of MICDn_GND_SEL
	 *  See datasheet for field values
	 */
	u32 gnd;

	/** MICBIAS number to enable */
	u32 bias;

	/** State of polarity gpio during probe (true == gpio asserted) */
	bool gpio;

	/** Value of HPn_GND_SEL
	 *  See datasheet for field values
	 */
	u32 hp_gnd;
};

struct madera_micd_range {
	int max;  /** Ohms */
	int key;  /** Key to report to input layer */
};

/** pdata sub-structure for accessory detect configuration */
struct madera_accdet_pdata {
	/** set to true to enable this accessory detect */
	bool enabled;

	/** which output this accdet is for (1 = OUT1, ...) */
	u32 output;

	/** Set whether JD2 is used for jack detection */
	bool jd_use_jd2;

	/** set to true if jackdet contact opens on insert */
	bool jd_invert;

	/** If non-zero don't run headphone detection, just report this value
	 * Specified as hundredths-of-an-ohm, that is (ohms * 100)
	 */
	u32 fixed_hpdet_imp_x100;

	/** Impedance of external series resistor on hpdet.
	 * Specified as hundredths-of-an-ohm, that is (ohms * 100)
	 */
	u32 hpdet_ext_res_x100;

	/** If non-zero, specifies the maximum impedance in ohms
	 * that will be considered as a short circuit.
	 */
	u32 hpdet_short_circuit_imp;

	/**
	 * Channel to use for headphone detection, valid values are 0 for
	 * left and 1 for right
	 */
	u32 hpdet_channel;

	/** Extra debounce timeout during initial mic detect (milliseconds) */
	u32 micd_detect_debounce_ms;

	/** Extra software debounces during button detection */
	u32 micd_manual_debounce;

	/** GPIO for mic detection polarity */
	int micd_pol_gpio;

	/** Mic detect ramp rate */
	u32 micd_bias_start_time;

	/** Setting for the codec MICD_RATE field
	 * See the  datasheet for documentation of the field values
	 */
	u32 micd_rate;

	/** Mic detect debounce level */
	u32 micd_dbtime;

	/** Mic detect timeout (milliseconds) */
	u32 micd_timeout_ms;

	/** Mic detect clamp function */
	u32 micd_clamp_mode;

	/** Force MICBIAS on for mic detect */
	bool micd_force_micbias;

	/** Declare an open circuit as a 4 pole jack */
	bool micd_open_circuit_declare;

	/** Use software comparison to determine mic presence */
	bool micd_software_compare;

	/** Mic detect level parameters */
	const struct madera_micd_range *micd_ranges;
	int num_micd_ranges;

	/** Headset polarity configurations */
	const struct madera_micd_config *micd_configs;
	int num_micd_configs;

	/**
	 * 4 entries:
	 * [HPL_clamp_pin, HPL_impedance_measurement_pin,
	 * HPR_clamp_pin, HPR_impedance_measurement_pin]
	 *
	 * 0 = use default setting
	 * values 0x1..0xffff = set to this value
	 * >0xffff = set to 0
	 */
	u32 hpd_pins[4];

	/** Override the normal jack detection */
	const struct madera_jd_state *custom_jd;
};

/**
 * struct madera_micbias_pin_pdata - MICBIAS pin configuration
 *
 * @init_data: initialization data for the regulator
 */
struct madera_micbias_pin_pdata {
	struct regulator_init_data *init_data;
	u32 active_discharge;
};

/**
 * struct madera_micbias_pdata - Regulator configuration for an on-chip MICBIAS
 *
 * @init_data: initialization data for the regulator
 * @ext_cap:   set to true if an external capacitor is fitted
 * @pin:       Configuration for each output pin from this MICBIAS
 */
struct madera_micbias_pdata {
	struct regulator_init_data *init_data;
	u32 active_discharge;
	bool ext_cap;

	struct madera_micbias_pin_pdata pin[MADERA_MAX_CHILD_MICBIAS];
};

#endif
