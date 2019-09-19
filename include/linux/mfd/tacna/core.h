/*
 * Tacna MFD internals
 *
 * Copyright 2016-2018 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef TACNA_CORE_H
#define TACNA_CORE_H

#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/mfd/tacna/pdata.h>
#include <linux/notifier.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

enum tacna_type {
	CS48LX50 = 3,
	CS47L96 = 4,
	CS47L97 = 5,
	CS48L31 = 6,
	CS48L32 = 7,
	CS48L33 = 8,
};

#define TACNA_MAX_CORE_SUPPLIES		2
#define TACNA_MAX_GPIOS			23
#define TACNA_MAX_DSPS			2

/* Notifier events */
#define TACNA_NOTIFY_VOICE_TRIGGER	0x1
#define TACNA_NOTIFY_HPDET		0x2
#define TACNA_NOTIFY_MICDET		0x4
#define TACNA_NOTIFY_ULTRASONIC		0x5

/* GPIO Function Definitions */
#define TACNA_GP_FN_ALTERNATE		0x00
#define TACNA_GP_FN_GPIO		0x01
#define TACNA_GP_FN_DSP_GPIO		0x02
#define TACNA_GP_FN_DRC1_SIGNAL_DETECT	0x88
#define TACNA_GP_FN_DRC2_SIGNAL_DETECT	0x89

struct snd_soc_dapm_context;
struct tacna_extcon_info;

struct tacna {
	struct regmap *regmap;
	struct regmap *dsp_regmap[TACNA_MAX_DSPS];

	struct device *dev;

	enum tacna_type type;
	unsigned int rev;
	unsigned int otp_rev;

	struct gpio_desc *reset_gpio;

	int num_core_supplies;
	struct regulator_bulk_data core_supplies[TACNA_MAX_CORE_SUPPLIES];
	struct regulator *vdd_d;
	bool vout_mic_regulated;
	bool bypass_cache;
	bool vdd_d_powered_off;

	struct tacna_pdata pdata;

	struct device *irq_dev;
	int irq;

	bool hpdet_clamp[TACNA_MAX_ACCESSORY];
	bool hpdet_shorted[TACNA_MAX_ACCESSORY];
	unsigned int hp_ena;
	unsigned int hp_impedance_x100[TACNA_MAX_ACCESSORY];

	struct tacna_extcon_info *extcon_info;

	struct snd_soc_dapm_context *dapm;

	struct blocking_notifier_head notifier;
};

unsigned int tacna_get_num_micbias(struct tacna *tacna);
unsigned int tacna_get_num_childbias(struct tacna *tacna, unsigned int micbias);

const char *tacna_name_from_type(enum tacna_type type);

static inline int tacna_call_notifiers(struct tacna *tacna,
				       unsigned long event,
				       void *data)
{
	return blocking_notifier_call_chain(&tacna->notifier, event, data);
}
#endif
