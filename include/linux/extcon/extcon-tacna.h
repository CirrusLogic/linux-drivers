/*
 * extcon-tacna.h - public extcon driver API for Cirrus Logic Tacna codecs
 *
 * Copyright 2017 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef EXTCON_TACNA_H
#define EXTCON_TACNA_H

#include <linux/mfd/tacna/registers.h>

#define TACNA_MICD1_LVL_8 0x00000400
#define TACNA_MICD1_LVL_7 0x00000200
#define TACNA_MICD1_LVL_6 0x00000100
#define TACNA_MICD1_LVL_5 0x00000080
#define TACNA_MICD1_LVL_4 0x00000040
#define TACNA_MICD1_LVL_3 0x00000020
#define TACNA_MICD1_LVL_2 0x00000010
#define TACNA_MICD1_LVL_1 0x00000008
#define TACNA_MICD1_LVL_0 0x00000004

#define TACNA_MICD1_LVL_1_TO_7 \
		(TACNA_MICD1_LVL_1 | TACNA_MICD1_LVL_2 | \
		 TACNA_MICD1_LVL_3 | TACNA_MICD1_LVL_4 | \
		 TACNA_MICD1_LVL_5 | TACNA_MICD1_LVL_6 | \
		 TACNA_MICD1_LVL_7)

#define TACNA_MICD1_LVL_0_TO_7 (TACNA_MICD1_LVL_0 | TACNA_MICD1_LVL_1_TO_7)

#define TACNA_MICD1_LVL_0_TO_8 (TACNA_MICD1_LVL_0_TO_7 | TACNA_MICD1_LVL_8)

/* Conversion between ohms and hundredths of an ohm. */
static inline unsigned int tacna_hohm_to_ohm(unsigned int hohms)
{
	if (hohms == INT_MAX)
		return hohms;
	else
		return (hohms + 50) / 100;
}

static inline unsigned int tacna_ohm_to_hohm(unsigned int ohms)
{
	if (ohms >= (INT_MAX / 100))
		return INT_MAX;
	else
		return ohms * 100;
}

/**
 * struct tacna_hpdet_notify_data - Notify data for TACNA_NOTIFY_HPDET
 *
 * @impedance_x100: ohms * 100
 */
struct tacna_hpdet_notify_data {
	unsigned int impedance_x100;
};

/**
 * struct tacna_micdet_notify_data - Notify data for TACNA_NOTIFY_MICDET
 *
 * @impedance_x100: ohms * 100
 * @present:	    true if mic is present
 * @out_num:	    output this applies to (1 = OUT1, 2 = OUT2...)
 */
struct tacna_micdet_notify_data {
	unsigned int impedance_x100;
	bool present;
	int out_num;
};

struct tacna_micd_bias {
	unsigned int bias;
	bool enabled;
};

struct tacna_hpdet_trims {
	int off_x4;
	int grad_x4;
};

struct tacna_extcon {
	struct device *dev;
	struct tacna *tacna;
	const struct tacna_accdet_pdata *pdata;
	struct mutex lock;
	struct regulator *micvdd;
	struct input_dev *input;
	struct extcon_dev *edev;
	struct gpio_desc *micd_pol_gpio;

	u32 last_jackdet;
	int hp_tuning_level;

	const struct tacna_hpdet_calibration_data *hpdet_ranges;
	int num_hpdet_ranges;
	const struct tacna_hpdet_trims *hpdet_trims;

	int micd_mode;
	const struct tacna_micd_config *micd_modes;
	int num_micd_modes;

	const struct tacna_micd_range *micd_ranges;
	int num_micd_ranges;

	int micd_res_old;
	int micd_debounce;
	int micd_count;

	struct completion manual_mic_completion;

	struct delayed_work micd_detect_work;

	bool have_mic;
	bool detecting;
	int jack_flips;

	const struct tacna_jd_state *state;
	const struct tacna_jd_state *old_state;
	struct delayed_work state_timeout_work;

	struct tacna_micd_bias micd_bias;
};

enum tacna_accdet_mode {
	TACNA_ACCDET_MODE_MIC,
	TACNA_ACCDET_MODE_HPL,
	TACNA_ACCDET_MODE_HPR,
	TACNA_ACCDET_MODE_HPM,
	TACNA_ACCDET_MODE_ADC,
	TACNA_ACCDET_MODE_INVALID,
};

struct tacna_jd_state {
	enum tacna_accdet_mode mode;

	int (*start)(struct tacna_extcon *);
	void (*restart)(struct tacna_extcon *);
	int (*reading)(struct tacna_extcon *, int);
	void (*stop)(struct tacna_extcon *);

	int (*timeout_ms)(struct tacna_extcon *);
	void (*timeout)(struct tacna_extcon *);
};

int tacna_jds_set_state(struct tacna_extcon *info,
			const struct tacna_jd_state *new_state);

void tacna_set_headphone_imp(struct tacna_extcon *info, int ohms_x100);

const struct tacna_jd_state tacna_hpdet_left;
const struct tacna_jd_state tacna_hpdet_right;
const struct tacna_jd_state tacna_micd_button;
const struct tacna_jd_state tacna_micd_microphone;
const struct tacna_jd_state tacna_micd_adc_mic;

int tacna_hpdet_start(struct tacna_extcon *info);
void tacna_hpdet_restart(struct tacna_extcon *info);
void tacna_hpdet_stop(struct tacna_extcon *info);
int tacna_hpdet_reading(struct tacna_extcon *info, int val);

int tacna_micd_start(struct tacna_extcon *info);
void tacna_micd_stop(struct tacna_extcon *info);
int tacna_micd_button_reading(struct tacna_extcon *info, int val);

int tacna_micd_mic_start(struct tacna_extcon *info);
void tacna_micd_mic_stop(struct tacna_extcon *info);
int tacna_micd_mic_reading(struct tacna_extcon *info, int val);
int tacna_micd_mic_timeout_ms(struct tacna_extcon *info);
void tacna_micd_mic_timeout(struct tacna_extcon *info);

void tacna_extcon_report(struct tacna_extcon *info, int which, bool attached);

#endif
