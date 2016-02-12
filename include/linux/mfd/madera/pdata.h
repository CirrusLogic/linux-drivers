/*
 * Platform data for Madera codecs
 *
 * Copyright 2015 Cirrus Logic
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>

#ifndef _MADERA_PDATA_H
#define _MADERA_PDATA_H

#include <dt-bindings/mfd/madera.h>
#include <sound/madera-pdata.h>

#define MADERA_GPN_LVL_MASK		0x8000
#define MADERA_GPN_LVL_SHIFT		    15
#define MADERA_GPN_LVL_WIDTH		     1
#define MADERA_GPN_OP_CFG_MASK		0x4000
#define MADERA_GPN_OP_CFG_SHIFT		    14
#define MADERA_GPN_OP_CFG_WIDTH		     1
#define MADERA_GPN_DB_MASK		0x2000
#define MADERA_GPN_DB_SHIFT		    13
#define MADERA_GPN_DB_WIDTH		     1
#define MADERA_GPN_POL_MASK		0x0800
#define MADERA_GPN_POL_SHIFT		    11
#define MADERA_GPN_POL_WIDTH		     1
#define MADERA_GPN_FN_MASK		0x03FF
#define MADERA_GPN_FN_SHIFT		     0
#define MADERA_GPN_FN_WIDTH		    10


#define MADERA_GPN_DIR_MASK		0x8000
#define MADERA_GPN_DIR_SHIFT		    15
#define MADERA_GPN_DIR_WIDTH		     1
#define MADERA_GPN_PU_MASK		0x4000
#define MADERA_GPN_PU_SHIFT		    14
#define MADERA_GPN_PU_WIDTH		     1
#define MADERA_GPN_PD_MASK		0x2000
#define MADERA_GPN_PD_SHIFT		    13
#define MADERA_GPN_PD_WIDTH		     1
#define MADERA_GPN_DBTIME_MASK		0x000F
#define MADERA_GPN_DBTIME_SHIFT		     0
#define MADERA_GPN_DBTIME_WIDTH		     4

#define MADERA_MAX_GPIO_REGS		80

#define CS47L35_NUM_GPIOS	16
#define CS47L85_NUM_GPIOS	40
#define CS47L90_NUM_GPIOS	38

#define MADERA_MAX_MICBIAS		4
#define MADERA_MAX_CHILD_MICBIAS	4

#define MADERA_MAX_GPSW			2

#define MADERA_HAP_ACT_ERM		0
#define MADERA_HAP_ACT_LRA		2

/* Treat INT_MAX impedance as open circuit */
#define MADERA_HP_Z_OPEN		INT_MAX
#define MADERA_HP_SHORT_IMPEDANCE	4

struct regulator_init_data;
struct madera_jd_state;

struct madera_micbias {
	int mV;                    /** Regulated voltage */
	unsigned int ext_cap:1;    /** External capacitor fitted */
	unsigned int discharge[MADERA_MAX_CHILD_MICBIAS]; /** Actively discharge */
	unsigned int soft_start:1; /** Disable aggressive startup ramp rate */
	unsigned int bypass:1;     /** Use bypass mode */
};

struct madera_micd_config {
	unsigned int src;
	unsigned int gnd;
	unsigned int bias;
	bool gpio;
};

struct madera_micd_range {
	int max;  /** Ohms */
	int key;  /** Key to report to input layer */
};

struct madera_pdata {
	int reset;      /** GPIO controlling /RESET, if any */
	int ldoena;     /** GPIO controlling LODENA, if any */

	/** Regulator configuration for MICVDD */
	const struct regulator_init_data *micvdd;

	/** Regulator configuration for LDO1 */
	const struct regulator_init_data *ldo1;

	/** If a direct 32kHz clock is provided on an MCLK specify it here */
	unsigned int clk32k_src;

	/** Mode for primary IRQ (defaults to active low) */
	unsigned int irq_flags;

	/** Base GPIO */
	int gpio_base;

	/** GPIO for primary IRQ (used for edge triggered emulation) */
	int irq_gpio;

	/** Pin state for GPIO pins
	 * Defines default pin function and state for each GPIO. The values in
	 * here are written into the GPn_CONFIGx register block. There are
	 * two entries for each GPIO pin, for the GPn_CONFIG1 and GPn_CONFIG2
	 * registers.
	 *
	 * See the codec datasheet for a description of the contents of the
	 * GPn_CONFIGx registers.
	 *
	 * 0 = leave at chip default
	 * values 0x1..0xffff = set to this value
	 * 0xffffffff = set to 0
	 */
	unsigned int gpio_defaults[MADERA_MAX_GPIO_REGS];

	/** MICBIAS configurations */
	struct madera_micbias micbias[MADERA_MAX_MICBIAS];

	/** Substructure of pdata for the ASoC codec driver
	 * See include/sound/madera-pdata.h
	 */
	struct madera_codec_pdata codec;

	/** Time in milliseconds to keep wake lock during jack detection */
	int jd_wake_time;

	/** Set whether JD2 is used for jack detection */
	bool jd_use_jd2;

	/** set to true if jackdet contact opens on insert */
	bool jd_invert;

	/** General purpose switch mode setting
	 * See the SW1_MODE field in the datasheet for the available values
	 */
	unsigned int gpsw[MADERA_MAX_GPSW];

	/** If non-zero don't run headphone detection, just report this value
	 * Specified as hundredths-of-an-ohm, that is (ohms * 100)
	 */
	int fixed_hpdet_imp_x100;

	/** Impedance of external series resistor on hpdet.
	 * Specified as hundredths-of-an-ohm, that is (ohms * 100)
	 */
	int hpdet_ext_res_x100;

	/** If non-zero, specifies the maximum impedance in ohms
	 * that will be considered as a short circuit.
	 */
	int hpdet_short_circuit_imp;

	/** GPIO used for mic isolation with HPDET */
	int hpdet_id_gpio;

	/**
	 * Channel to use for headphone detection, valid values are 0 for
	 * left and 1 for right
	 */
	int hpdet_channel;

	/** Extra debounce timeout during initial mic detect (milliseconds) */
	int micd_detect_debounce_ms;

	/** Extra software debounces during button detection */
	int micd_manual_debounce;

	/** GPIO for mic detection polarity */
	int micd_pol_gpio;

	/** Mic detect ramp rate */
	int micd_bias_start_time;

	/** Setting for the codec MICD_RATE field
	 * See the  datasheet for documentation of the field values
	 */
	int micd_rate;

	/** Mic detect debounce level */
	int micd_dbtime;

	/** Mic detect timeout (milliseconds) */
	int micd_timeout_ms;

	/** Mic detect clamp function */
	unsigned int micd_clamp_mode;

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
	unsigned int hpd_pins[4];

	/** Override the normal jack detection */
	const struct madera_jd_state *custom_jd;

	/** Haptic actuator type */
	unsigned int hap_type;
};

#endif
