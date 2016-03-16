/*
 * Madera MFD internals
 *
 * Copyright 2015 CirrusLogic
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _MADERA_CORE_H
#define _MADERA_CORE_H

#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/notifier.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/madera/pdata.h>

#define MADERA_IRQ_BOOT_DONE		0
#define MADERA_IRQ_CTRLIF_ERR		1
#define MADERA_IRQ_FLL1_LOCK		2
#define MADERA_IRQ_FLL2_LOCK		3
#define MADERA_IRQ_FLL3_LOCK		4
#define MADERA_IRQ_FLLAO_LOCK		5
#define MADERA_IRQ_CLK_SYS_ERR		6
#define MADERA_IRQ_CLK_ASYNC_ERR	7
#define MADERA_IRQ_CLK_DSP_ERR		8
#define MADERA_IRQ_HPDET		9
#define MADERA_IRQ_MICDET1		10
#define MADERA_IRQ_MICDET2		11
#define MADERA_IRQ_JD1_RISE		12
#define MADERA_IRQ_JD1_FALL		13
#define MADERA_IRQ_JD2_RISE		14
#define MADERA_IRQ_JD2_FALL		15
#define MADERA_IRQ_MICD_CLAMP_RISE	16
#define MADERA_IRQ_MICD_CLAMP_FALL	17
#define MADERA_IRQ_DRC2_SIG_DET		18
#define MADERA_IRQ_DRC1_SIG_DET		19
#define MADERA_IRQ_ASRC1_IN1_LOCK	20
#define MADERA_IRQ_ASRC1_IN2_LOCK	21
#define MADERA_IRQ_ASRC2_IN1_LOCK	22
#define MADERA_IRQ_ASRC2_IN2_LOCK	23
#define MADERA_IRQ_DSP_IRQ1		24
#define MADERA_IRQ_DSP_IRQ2		25
#define MADERA_IRQ_DSP_IRQ3		26
#define MADERA_IRQ_DSP_IRQ4		27
#define MADERA_IRQ_DSP_IRQ5		28
#define MADERA_IRQ_DSP_IRQ6		29
#define MADERA_IRQ_DSP_IRQ7		30
#define MADERA_IRQ_DSP_IRQ8		31
#define MADERA_IRQ_DSP_IRQ9		32
#define MADERA_IRQ_DSP_IRQ10		33
#define MADERA_IRQ_DSP_IRQ11		34
#define MADERA_IRQ_DSP_IRQ12		35
#define MADERA_IRQ_DSP_IRQ13		36
#define MADERA_IRQ_DSP_IRQ14		37
#define MADERA_IRQ_DSP_IRQ15		38
#define MADERA_IRQ_DSP_IRQ16		39
#define MADERA_IRQ_HP1L_SC		40
#define MADERA_IRQ_HP1R_SC		41
#define MADERA_IRQ_HP2L_SC		42
#define MADERA_IRQ_HP2R_SC		43
#define MADERA_IRQ_HP3L_SC		44
#define MADERA_IRQ_HP3R_SC		45
#define MADERA_IRQ_SPKOUTL_SC		46
#define MADERA_IRQ_SPKOUTR_SC		47
#define MADERA_IRQ_HP1L_ENABLE_DONE	48
#define MADERA_IRQ_HP1R_ENABLE_DONE	49
#define MADERA_IRQ_HP2L_ENABLE_DONE	50
#define MADERA_IRQ_HP2R_ENABLE_DONE	51
#define MADERA_IRQ_HP3L_ENABLE_DONE	52
#define MADERA_IRQ_HP3R_ENABLE_DONE	53
#define MADERA_IRQ_SPKOUTL_ENABLE_DONE	54
#define MADERA_IRQ_SPKOUTR_ENABLE_DONE	55
#define MADERA_IRQ_SPK_SHUTDOWN		56
#define MADERA_IRQ_SPK_OVERHEAT		57
#define MADERA_IRQ_SPK_OVERHEAT_WARN	58
#define MADERA_IRQ_GPIO1		59
#define MADERA_IRQ_GPIO2		60
#define MADERA_IRQ_GPIO3		61
#define MADERA_IRQ_GPIO4		62
#define MADERA_IRQ_GPIO5		63
#define MADERA_IRQ_GPIO6		64
#define MADERA_IRQ_GPIO7		65
#define MADERA_IRQ_GPIO8		66
#define MADERA_IRQ_DSP1_BUS_ERROR	67
#define MADERA_IRQ_DSP2_BUS_ERROR	68
#define MADERA_IRQ_DSP3_BUS_ERROR	69
#define MADERA_IRQ_DSP4_BUS_ERROR	70
#define MADERA_IRQ_DSP5_BUS_ERROR	71
#define MADERA_IRQ_DSP6_BUS_ERROR	72
#define MADERA_IRQ_DSP7_BUS_ERROR	73

#define MADERA_NUM_IRQ			74

enum madera_type {
	CS47L35 = 1,
	CS47L85 = 2,
	CS47L90 = 3,
	CS47L91 = 4,
	WM1840 = 7,
};

#define MADERA_MAX_CORE_SUPPLIES 2

/* Notifier events */
#define MADERA_NOTIFY_VOICE_TRIGGER	0x1
#define MADERA_NOTIFY_HPDET		0x2
#define MADERA_NOTIFY_MICDET		0x4

struct snd_soc_dapm_context;
struct madera_extcon_info;

struct madera {
	struct regmap *regmap;
	struct regmap *regmap_32bit;

	struct device *dev;

	enum madera_type type;
	unsigned int rev;

	int num_core_supplies;
	struct regulator_bulk_data core_supplies[MADERA_MAX_CORE_SUPPLIES];
	struct regulator *dcvdd;
	struct notifier_block dcvdd_notifier;
	bool internal_dcvdd;
	bool micvdd_regulated;
	bool bypass_cache;
	bool dcvdd_powered_off;

	struct madera_pdata pdata;

	unsigned int irq_sem;
	int irq;
	struct regmap_irq_chip_data *irq_data;
	struct irq_domain *edge_domain;

	bool hpdet_clamp;
	unsigned int hp_ena;
	unsigned int hp_impedance_x100;
	struct madera_extcon_info *extcon_info;

	struct snd_soc_dapm_context *dapm;

	struct mutex reg_setting_lock;

	struct blocking_notifier_head notifier;
};

extern int madera_request_irq(struct madera *madera, int irq, const char *name,
			      irq_handler_t handler, void *data);
extern void madera_free_irq(struct madera *madera, int irq, void *data);
extern int madera_set_irq_wake(struct madera *madera, int irq, int on);

extern int madera_of_read_uint_array(struct madera *madera, const char *prop,
				     bool mandatory,
				     unsigned int *dest,
				     int minlen, int maxlen);
extern int madera_of_read_uint(struct madera *madera, const char *prop,
				bool mandatory, unsigned int *data);

extern int madera_get_num_micbias(struct madera *madera,
				  unsigned int *n_micbiases,
				  unsigned int *n_child_micbiases);

extern const char *madera_name_from_type(enum madera_type type);

static inline int madera_of_read_int(struct madera *madera, const char *prop,
				     bool mandatory, int *data)
{
	return madera_of_read_uint(madera, prop, mandatory,
				   (unsigned int *)data);
}

static inline int madera_call_notifiers(struct madera *madera,
					unsigned long event,
					void *data)
{
	return blocking_notifier_call_chain(&madera->notifier, event, data);
}
#endif
