/*
 * Tacna MFD internals
 *
 * Copyright 2016-2017 Cirrus Logic
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef TACNA_CORE_H
#define TACNA_CORE_H

#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/notifier.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/tacna/pdata.h>

enum tacna_type {
	CS47L94 = 1,
	CS47L95 = 2,
};

#define TACNA_MAX_CORE_SUPPLIES		2
#define TACNA_MAX_GPIOS			23

/* Notifier events */
#define TACNA_NOTIFY_VOICE_TRIGGER	0x1
#define TACNA_NOTIFY_HPDET		0x2
#define TACNA_NOTIFY_MICDET		0x4

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

	struct device *dev;

	enum tacna_type type;
	unsigned int rev;
	unsigned int otp_rev;

	struct gpio_desc *reset_gpio;

	int num_core_supplies;
	struct regulator_bulk_data core_supplies[TACNA_MAX_CORE_SUPPLIES];
	struct regulator *vdd_d;
	struct notifier_block vdd_d_notifier;
	bool vout_mic_regulated;
	bool bypass_cache;
	bool vdd_d_powered_off;

	struct tacna_pdata pdata;

	struct device *irq_dev;
	int irq;

	bool hpdet_clamp[TACNA_MAX_ACCESSORY];
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
