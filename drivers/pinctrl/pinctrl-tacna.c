/*
 * Pinctrl for Cirrus Logic Tacna codecs
 *
 * Copyright 2017-2018 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>

#include <linux/mfd/tacna/core.h>
#include <linux/mfd/tacna/registers.h>

#include "pinctrl-utils.h"

/*
 * Pins are named after their GPIO number
 * NOTE: IDs are zero-indexed for coding convenience
 */
static const struct pinctrl_pin_desc tacna_pins[] = {
	PINCTRL_PIN(0, "gpio1"),
	PINCTRL_PIN(1, "gpio2"),
	PINCTRL_PIN(2, "gpio3"),
	PINCTRL_PIN(3, "gpio4"),
	PINCTRL_PIN(4, "gpio5"),
	PINCTRL_PIN(5, "gpio6"),
	PINCTRL_PIN(6, "gpio7"),
	PINCTRL_PIN(7, "gpio8"),
	PINCTRL_PIN(8, "gpio9"),
	PINCTRL_PIN(9, "gpio10"),
	PINCTRL_PIN(10, "gpio11"),
	PINCTRL_PIN(11, "gpio12"),
	PINCTRL_PIN(12, "gpio13"),
	PINCTRL_PIN(13, "gpio14"),
	PINCTRL_PIN(14, "gpio15"),
	PINCTRL_PIN(15, "gpio16"),
	PINCTRL_PIN(16, "gpio17"),
	PINCTRL_PIN(17, "gpio18"),
	PINCTRL_PIN(18, "gpio19"),
	PINCTRL_PIN(19, "gpio20"),
	PINCTRL_PIN(20, "gpio21"),
	PINCTRL_PIN(21, "gpio22"),
	PINCTRL_PIN(22, "gpio23"),
	PINCTRL_PIN(23, "gpio24"),
	PINCTRL_PIN(24, "gpio25"),
	PINCTRL_PIN(25, "gpio26"),
	PINCTRL_PIN(26, "gpio27"),
	PINCTRL_PIN(27, "gpio28"),
};

/*
 * All single-pin functions can be mapped to any GPIO, however pinmux applies
 * functions to pin groups and only those groups declared as supporting that
 * function. To make this work we must put each pin in its own dummy group so
 * that the functions can be described as applying to all pins.
 * Since these do not correspond to anything in the actual hardware - they are
 * merely an adaptation to pinctrl's view of the world - we use the same name
 * as the pin to avoid confusion when comparing with datasheet instructions
 */
static const char * const tacna_pin_single_group_names[] = {
	"gpio1",  "gpio2",  "gpio3",  "gpio4",  "gpio5",  "gpio6",  "gpio7",
	"gpio8",  "gpio9",  "gpio10", "gpio11", "gpio12", "gpio13", "gpio14",
	"gpio15", "gpio16", "gpio17", "gpio18", "gpio19", "gpio20", "gpio21",
	"gpio22", "gpio23", "gpio24", "gpio25", "gpio26", "gpio27", "gpio28",
};

/* set of pin numbers for single-pin groups */
static const unsigned int tacna_pin_single_group_pins[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
	20, 21, 22, 23, 24, 25, 26, 27,
};

static const char * const tacna_asp1_group_names[] = { "asp1" };
static const char * const tacna_asp2_group_names[] = { "asp2" };
static const char * const tacna_asp3_group_names[] = { "asp3" };
static const char * const tacna_asp4_group_names[] = { "asp4" };
static const char * const tacna_asp1ao_group_names[] = { "asp1ao" };
static const char * const tacna_dsd1_group_names[] = { "dsd1" };
static const char * const tacna_in1pdm_group_names[] = { "in1-pdm" };
static const char * const tacna_in2pdm_group_names[] = { "in2-pdm" };
static const char * const tacna_in3pdm_group_names[] = { "in3-pdm" };
static const char * const tacna_in4pdm_group_names[] = { "in4-pdm" };
static const char * const tacna_out5pdm_group_names[] = { "out5-pdm" };
static const char * const tacna_spi2_group_names[] = { "spi2" };

/*
 * alt-functions always apply to only one group, other functions always
 * apply to all pins
 */
static const struct {
	const char *name;
	const char * const *group_names;
	u32 func;
} tacna_mux_funcs[] = {
	{
		.name = "asp1ao",
		.group_names = tacna_asp1ao_group_names,
		.func = 0x000
	},
	{
		.name = "asp1",
		.group_names = tacna_asp1_group_names,
		.func = 0x000
	},
	{
		.name = "asp2",
		.group_names = tacna_asp2_group_names,
		.func = 0x000
	},
	{
		.name = "asp3",
		.group_names = tacna_asp3_group_names,
		.func = 0x000
	},
	{
		.name = "asp4",
		.group_names = tacna_asp4_group_names,
		.func = 0x000
	},
	{
		.name = "dsd1",
		.group_names = tacna_dsd1_group_names,
		.func = 0x000
	},
	{
		.name = "in1-pdm",
		.group_names = tacna_in1pdm_group_names,
		.func = 0x000
	},
	{
		.name = "in2-pdm",
		.group_names = tacna_in2pdm_group_names,
		.func = 0x000,
	},
	{
		.name = "in3-pdm",
		.group_names = tacna_in3pdm_group_names,
		.func = 0x000
	},
	{
		.name = "in4-pdm",
		.group_names = tacna_in4pdm_group_names,
		.func = 0x000,
	},
	{
		.name = "out5-pdm",
		.group_names = tacna_out5pdm_group_names,
		.func = 0x000
	},
	{
		.name = "spi2",
		.group_names = tacna_spi2_group_names,
		.func = 0x000
	},
	{
		.name = "io",
		.group_names = tacna_pin_single_group_names,
		.func = 0x001
	},
	{
		.name = "dsp-gpio",
		.group_names = tacna_pin_single_group_names,
		.func = 0x002
	},
	{
		.name = "irq1",
		.group_names = tacna_pin_single_group_names,
		.func = 0x003
	},
	{
		.name = "fll1-clk",
		.group_names = tacna_pin_single_group_names,
		.func = 0x010
	},
	{
		.name = "fll2-clk",
		.group_names = tacna_pin_single_group_names,
		.func = 0x011
	},
	{
		.name = "fll3-clk",
		.group_names = tacna_pin_single_group_names,
		.func = 0x012
	},
	{
		.name = "fll1-lock",
		.group_names = tacna_pin_single_group_names,
		.func = 0x018
	},
	{
		.name = "fll2-lock",
		.group_names = tacna_pin_single_group_names,
		.func = 0x01a
	},
	{
		.name = "fll3-lock",
		.group_names = tacna_pin_single_group_names,
		.func = 0x01c
	},
	{
		.name = "opclk",
		.group_names = tacna_pin_single_group_names,
		.func = 0x048
	},
	{
		.name = "opclk-async",
		.group_names = tacna_pin_single_group_names,
		.func = 0x049
	},
	{
		.name = "opclk-dsp",
		.group_names = tacna_pin_single_group_names,
		.func = 0x04a
	},
	{
		.name = "uart",
		.group_names = tacna_pin_single_group_names,
		.func = 0x04c
	},
	{
		.name = "pwm1-out",
		.group_names = tacna_pin_single_group_names,
		.func = 0x080
	},
	{
		.name = "pwm2-out",
		.group_names = tacna_pin_single_group_names,
		.func = 0x081
	},
	{
		.name = "input-path-signal-detect",
		.group_names = tacna_pin_single_group_names,
		.func = 0x08c
	},
	{
		.name = "ultrasonic-in1-activity-detect",
		.group_names = tacna_pin_single_group_names,
		.func = 0x090
	},
	{
		.name = "ultrasonic-in2-activity-detect",
		.group_names = tacna_pin_single_group_names,
		.func = 0x092
	},
	{
		.name = "asrc1-in1-lock",
		.group_names = tacna_pin_single_group_names,
		.func = 0x098
	},
	{
		.name = "asrc1-in2-lock",
		.group_names = tacna_pin_single_group_names,
		.func = 0x09a
	},
	{
		.name = "dfc1-saturate",
		.group_names = tacna_pin_single_group_names,
		.func = 0x0fa
	},
	{
		.name = "dma-ch0-programmable-transfer-complete",
		.group_names = tacna_pin_single_group_names,
		.func = 0x190
	},
	{
		.name = "dma-ch1-programmable-transfer-complete",
		.group_names = tacna_pin_single_group_names,
		.func = 0x191
	},
	{
		.name = "dma-ch2-programmable-transfer-complete",
		.group_names = tacna_pin_single_group_names,
		.func = 0x192
	},
	{
		.name = "dma-ch3-programmable-transfer-complete",
		.group_names = tacna_pin_single_group_names,
		.func = 0x193
	},
	{
		.name = "dma-ch4-programmable-transfer-complete",
		.group_names = tacna_pin_single_group_names,
		.func = 0x194
	},
	{
		.name = "dma-ch5-programmable-transfer-complete",
		.group_names = tacna_pin_single_group_names,
		.func = 0x195
	},
	{
		.name = "dma-ch6-programmable-transfer-complete",
		.group_names = tacna_pin_single_group_names,
		.func = 0x196
	},
	{
		.name = "dma-ch7-programmable-transfer-complete",
		.group_names = tacna_pin_single_group_names,
		.func = 0x197
	},
	{
		.name = "out1r-hp1-enable-disable-sequence",
		.group_names = tacna_pin_single_group_names,
		.func = 0x1f8
	},
	{
		.name = "out1l-hp1-enable-disable-sequence",
		.group_names = tacna_pin_single_group_names,
		.func = 0x1fa
	},
	{
		.name = "out1r-hp2-enable-disable-sequence",
		.group_names = tacna_pin_single_group_names,
		.func = 0x1fc
	},
	{
		.name = "out1l-hp2-enable-disable-sequence",
		.group_names = tacna_pin_single_group_names,
		.func = 0x1fe
	},
	{
		.name = "out2r-enable-disable-sequence",
		.group_names = tacna_pin_single_group_names,
		.func = 0x200
	},
	{
		.name = "out2l-enable-disable-sequence",
		.group_names = tacna_pin_single_group_names,
		.func = 0x202
	},
	{
		.name = "outh-enable-disable-sequence",
		.group_names = tacna_pin_single_group_names,
		.func = 0x20c
	},
	{
		.name = "sample-rate-change-trigger-a",
		.group_names = tacna_pin_single_group_names,
		.func = 0x214
	},
	{
		.name = "sample-rate-change-trigger-b",
		.group_names = tacna_pin_single_group_names,
		.func = 0x215
	},
	{
		.name = "sample-rate-change-trigger-c",
		.group_names = tacna_pin_single_group_names,
		.func = 0x216
	},
	{
		.name = "sample-rate-change-trigger-d",
		.group_names = tacna_pin_single_group_names,
		.func = 0x217
	},
	{
		.name = "timer1-irq-ch1",
		.group_names = tacna_pin_single_group_names,
		.func = 0x230
	},
	{
		.name = "timer1-irq-ch2",
		.group_names = tacna_pin_single_group_names,
		.func = 0x231
	},
	{
		.name = "timer1-irq-ch3",
		.group_names = tacna_pin_single_group_names,
		.func = 0x232
	},
	{
		.name = "timer1-irq-ch4",
		.group_names = tacna_pin_single_group_names,
		.func = 0x233
	},
	{
		.name = "timer2-irq-ch1",
		.group_names = tacna_pin_single_group_names,
		.func = 0x234
	},
	{
		.name = "timer2-irq-ch2",
		.group_names = tacna_pin_single_group_names,
		.func = 0x235
	},
	{
		.name = "timer2-irq-ch3",
		.group_names = tacna_pin_single_group_names,
		.func = 0x236
	},
	{
		.name = "timer2-irq-ch4",
		.group_names = tacna_pin_single_group_names,
		.func = 0x237
	},
	{
		.name = "timer3-irq-ch1",
		.group_names = tacna_pin_single_group_names,
		.func = 0x238
	},
	{
		.name = "timer3-irq-ch2",
		.group_names = tacna_pin_single_group_names,
		.func = 0x239
	},
	{
		.name = "timer3-irq-ch3",
		.group_names = tacna_pin_single_group_names,
		.func = 0x23a
	},
	{
		.name = "timer3-irq-ch4",
		.group_names = tacna_pin_single_group_names,
		.func = 0x23b
	},
	{
		.name = "timer4-irq-ch1",
		.group_names = tacna_pin_single_group_names,
		.func = 0x23c
	},
	{
		.name = "timer4-irq-ch2",
		.group_names = tacna_pin_single_group_names,
		.func = 0x23d
	},
	{
		.name = "timer4-irq-ch3",
		.group_names = tacna_pin_single_group_names,
		.func = 0x23e
	},
	{
		.name = "timer4-irq-ch4",
		.group_names = tacna_pin_single_group_names,
		.func = 0x23f
	},
	{
		.name = "timer5-irq-ch1",
		.group_names = tacna_pin_single_group_names,
		.func = 0x240
	},
	{
		.name = "timer5-irq-ch2",
		.group_names = tacna_pin_single_group_names,
		.func = 0x241
	},
	{
		.name = "timer5-irq-ch3",
		.group_names = tacna_pin_single_group_names,
		.func = 0x242
	},
	{
		.name = "timer5-irq-ch4",
		.group_names = tacna_pin_single_group_names,
		.func = 0x243
	},
	{
		.name = "timer-1",
		.group_names = tacna_pin_single_group_names,
		.func = 0x250
	},
	{
		.name = "timer-2",
		.group_names = tacna_pin_single_group_names,
		.func = 0x251
	},
	{
		.name = "timer-3",
		.group_names = tacna_pin_single_group_names,
		.func = 0x252
	},
	{
		.name = "timer-4",
		.group_names = tacna_pin_single_group_names,
		.func = 0x253
	},
	{
		.name = "timer-5",
		.group_names = tacna_pin_single_group_names,
		.func = 0x254
	},
};

struct tacna_pin_groups {
	const char *name;
	const unsigned int *pins;
	unsigned int n_pins;
};

struct tacna_pin_chip {
	unsigned int n_pins;

	const struct tacna_pin_groups *pin_groups;
	unsigned int n_pin_groups;
};

struct tacna_pin_private {
	struct tacna *tacna;

	const struct tacna_pin_chip *chip; /* chip-specific groups */

	struct device *dev;
	struct pinctrl_dev *pctl;
};

/*
 * Chip-specific configs
 * The alt func groups available differ between codecs. Since these are the
 * most commonly used functions we place these at the lower function indexes
 * for convenience, and the less commonly used gpio functions at higher indexes
 *
 * To stay consistent with the datasheet the function names are the same as
 * the group names for that function's pins
 */
#ifdef CONFIG_PINCTRL_CS47L96
/* Note - all 1 less than in datasheet because these are zero-indexed */
static const unsigned int cs47l96_asp2_pins[] = { 8, 9, 10, 11 };
static const unsigned int cs47l96_asp3_pins[] = { 12, 13, 14, 15 };
static const unsigned int cs47l96_asp1ao_pins[] = { 19, 20, 21, 22 };
static const unsigned int cs47l96_dsd1_pins[] = { 16, 17, 18 };
static const unsigned int cs47l96_in3pdm_pins[] = { 4, 5 };
static const unsigned int cs47l96_in4pdm_pins[] = { 6, 7 };
static const unsigned int cs47l96_out5pdm_pins[] = { 2, 3 };

static const struct tacna_pin_groups cs47l96_pin_groups[] = {
	{ "asp2", cs47l96_asp2_pins, ARRAY_SIZE(cs47l96_asp2_pins) },
	{ "asp3", cs47l96_asp3_pins, ARRAY_SIZE(cs47l96_asp3_pins) },
	{ "asp1ao", cs47l96_asp1ao_pins, ARRAY_SIZE(cs47l96_asp1ao_pins) },
	{ "dsd1", cs47l96_dsd1_pins, ARRAY_SIZE(cs47l96_dsd1_pins) },
	{ "in3-pdm", cs47l96_in3pdm_pins, ARRAY_SIZE(cs47l96_in3pdm_pins) },
	{ "in4-pdm", cs47l96_in4pdm_pins, ARRAY_SIZE(cs47l96_in4pdm_pins) },
	{ "out5-pdm", cs47l96_out5pdm_pins, ARRAY_SIZE(cs47l96_out5pdm_pins) },
};

static const struct tacna_pin_chip cs47l96_pin_chip = {
	.n_pins = CS47L96_NUM_GPIOS,
	.pin_groups = cs47l96_pin_groups,
	.n_pin_groups = ARRAY_SIZE(cs47l96_pin_groups),
};
#endif

#ifdef CONFIG_PINCTRL_CS48L32
/* Note - all 1 less than in datasheet because these are zero-indexed */
static const unsigned int cs48l32_asp1_pins[] = { 2, 3, 4, 5 };
static const unsigned int cs48l32_asp2_pins[] = { 6, 7, 8, 9 };
static const unsigned int cs48l32_spi2_pins[] = { 10, 11, 12, 13, 14, 15 };

static const struct tacna_pin_groups cs48l32_pin_groups[] = {
	{ "asp1", cs48l32_asp1_pins, ARRAY_SIZE(cs48l32_asp1_pins) },
	{ "asp2", cs48l32_asp2_pins, ARRAY_SIZE(cs48l32_asp2_pins) },
	{ "spi2", cs48l32_spi2_pins, ARRAY_SIZE(cs48l32_spi2_pins) },
};

static const struct tacna_pin_chip cs48l32_pin_chip = {
	.n_pins = CS48L32_NUM_GPIOS,
	.pin_groups = cs48l32_pin_groups,
	.n_pin_groups = ARRAY_SIZE(cs48l32_pin_groups),
};
#endif

#ifdef CONFIG_PINCTRL_CS48LX50
/* Note - all 1 less than in datasheet because these are zero-indexed */
static const unsigned int cs48lx50_asp1_pins[] = { 4, 5, 6, 7 };
static const unsigned int cs48lx50_asp2_pins[] = { 8, 9, 10, 11 };
static const unsigned int cs48lx50_asp3_pins[] = { 12, 13, 14, 15 };
static const unsigned int cs48lx50_asp4_pins[] = { 16, 17, 18, 19 };
static const unsigned int cs48lx50_in1pdm_pins[] = { 20, 21 };
static const unsigned int cs48lx50_in2pdm_pins[] = { 22, 23 };
static const unsigned int cs48lx50_in3pdm_pins[] = { 24, 25 };
static const unsigned int cs48lx50_in4pdm_pins[] = { 26, 27 };

static const struct tacna_pin_groups cs48lx50_pin_groups[] = {
	{ "asp1", cs48lx50_asp1_pins, ARRAY_SIZE(cs48lx50_asp1_pins) },
	{ "asp2", cs48lx50_asp2_pins, ARRAY_SIZE(cs48lx50_asp2_pins) },
	{ "asp3", cs48lx50_asp3_pins, ARRAY_SIZE(cs48lx50_asp3_pins) },
	{ "asp4", cs48lx50_asp4_pins, ARRAY_SIZE(cs48lx50_asp4_pins) },
	{ "in1-pdm", cs48lx50_in1pdm_pins, ARRAY_SIZE(cs48lx50_in1pdm_pins) },
	{ "in2-pdm", cs48lx50_in2pdm_pins, ARRAY_SIZE(cs48lx50_in2pdm_pins) },
	{ "in3-pdm", cs48lx50_in3pdm_pins, ARRAY_SIZE(cs48lx50_in3pdm_pins) },
	{ "in4-pdm", cs48lx50_in4pdm_pins, ARRAY_SIZE(cs48lx50_in4pdm_pins) },
};

static const struct tacna_pin_chip cs48lx50_pin_chip = {
	.n_pins = CS48LX50_NUM_GPIOS,
	.pin_groups = cs48lx50_pin_groups,
	.n_pin_groups = ARRAY_SIZE(cs48lx50_pin_groups),
};
#endif

static unsigned int tacna_pin_make_drv_str(struct tacna_pin_private *priv,
					   unsigned int milliamps)
{
	switch (priv->tacna->type) {
	case CS47L96:
	case CS47L97:
	case CS48L31:
	case CS48L32:
	case CS48L33:
		switch (milliamps) {
		case 4:
			return 0;
		case 8:
			return 1 << TACNA_GP1_DRV_STR_SHIFT;
		default:
			break;
		}
		break;
	case CS48LX50:
		switch (milliamps) {
		case 2:
			return 0;
		case 4:
			return 1 << TACNA_GP1_DRV_STR_SHIFT;
		case 10:
			return 2 << TACNA_GP1_DRV_STR_SHIFT;
		case 12:
			return 3 << TACNA_GP1_DRV_STR_SHIFT;
		default:
			break;
		}
		break;
	default:
		break;
	}

	dev_warn(priv->dev, "%u mA is not a valid drive strength\n", milliamps);

	return 0;
}

static unsigned int tacna_pin_unmake_drv_str(struct tacna_pin_private *priv,
					     unsigned int regval)
{
	regval = (regval & TACNA_GP1_DRV_STR_MASK) >> TACNA_GP1_DRV_STR_SHIFT;

	switch (priv->tacna->type) {
	case CS47L96:
	case CS47L97:
	case CS48L31:
	case CS48L32:
	case CS48L33:
		switch (regval) {
		case 0:
			return 4;
		case 1:
			return 8;
		default:
			break;
		}
		break;
	case CS48LX50:
		switch (regval) {
		case 0:
			return 2;
		case 1:
			return 4;
		case 2:
			return 10;
		case 3:
			return 12;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int tacna_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct tacna_pin_private *priv = pinctrl_dev_get_drvdata(pctldev);

	/* Number of alt function groups plus number of single-pin groups */
	return priv->chip->n_pin_groups + priv->chip->n_pins;
}

static const char *tacna_get_group_name(struct pinctrl_dev *pctldev,
					unsigned int selector)
{
	struct tacna_pin_private *priv = pinctrl_dev_get_drvdata(pctldev);

	if (selector < priv->chip->n_pin_groups) {
		return priv->chip->pin_groups[selector].name;
	} else {
		selector -= priv->chip->n_pin_groups;
		return tacna_pin_single_group_names[selector];
	}
}

static int tacna_get_group_pins(struct pinctrl_dev *pctldev,
				unsigned int selector,
				const unsigned int **pins,
				unsigned int *num_pins)
{
	struct tacna_pin_private *priv = pinctrl_dev_get_drvdata(pctldev);

	if (selector < priv->chip->n_pin_groups) {
		*pins = priv->chip->pin_groups[selector].pins;
		*num_pins = priv->chip->pin_groups[selector].n_pins;
	} else {
		/* return the dummy group for a single pin */
		selector -= priv->chip->n_pin_groups;
		*pins = &tacna_pin_single_group_pins[selector];
		*num_pins = 1;
	}

	return 0;
}

static void tacna_pin_dbg_show_fn(struct tacna_pin_private *priv,
				   struct seq_file *s,
				   unsigned int pin, unsigned int fn)
{
	const struct tacna_pin_chip *chip = priv->chip;
	int i, g_pin;

	if (fn != 0) {
		for (i = 0; i < ARRAY_SIZE(tacna_mux_funcs); ++i) {
			if (tacna_mux_funcs[i].func == fn) {
				seq_printf(s, " FN=%s",
					   tacna_mux_funcs[i].name);
				return;
			}
		}
		return;	/* ignore unknown function values */
	}

	/* alt function */
	for (i = 0; i < chip->n_pin_groups; ++i) {
		for (g_pin = 0; g_pin < chip->pin_groups[i].n_pins; ++g_pin) {
			if (chip->pin_groups[i].pins[g_pin] == pin) {
				seq_printf(s, " FN=%s",
					   chip->pin_groups[i].name);
				return;
			}
		}
	}
}

static void tacna_pin_dbg_show(struct pinctrl_dev *pctldev,
			       struct seq_file *s, unsigned int pin)
{
	struct tacna_pin_private *priv = pinctrl_dev_get_drvdata(pctldev);
	unsigned int reg = TACNA_GPIO1_CTRL1 + (4 * pin);
	unsigned int conf, fn;
	int ret;

	ret = regmap_read(priv->tacna->regmap, reg, &conf);
	if (ret)
		return;

	seq_printf(s, "%08x", conf);

	fn = (conf & TACNA_GP1_FN_MASK) >> TACNA_GP1_FN_SHIFT;
	tacna_pin_dbg_show_fn(priv, s, pin, fn);

	/* State of direction bit is only relevant if function==1 */
	if (fn == 1) {
		if (conf & TACNA_GP1_DIR_MASK)
			seq_puts(s, " IN");
		else
			seq_puts(s, " OUT");
	}

	if (conf & TACNA_GP1_PU_MASK)
		seq_puts(s, " PU");

	if (conf & TACNA_GP1_PD_MASK)
		seq_puts(s, " PD");

	if (conf & TACNA_GP1_DB_MASK)
		seq_puts(s, " DB");

	if (conf & TACNA_GP1_OP_CFG_MASK)
		seq_puts(s, " OD");
	else
		seq_puts(s, " CMOS");

	seq_printf(s, " DRV=%umA", tacna_pin_unmake_drv_str(priv, conf));
}


static const struct pinctrl_ops tacna_pin_group_ops = {
	.get_groups_count = &tacna_get_groups_count,
	.get_group_name = &tacna_get_group_name,
	.get_group_pins = &tacna_get_group_pins,
	.pin_dbg_show = &tacna_pin_dbg_show,
	.dt_node_to_map = &pinconf_generic_dt_node_to_map_all,
	.dt_free_map = &pinctrl_utils_free_map,
};

static int tacna_mux_get_funcs_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(tacna_mux_funcs);
}

static const char *tacna_mux_get_func_name(struct pinctrl_dev *pctldev,
					    unsigned int selector)
{
	return tacna_mux_funcs[selector].name;
}

static int tacna_mux_get_groups(struct pinctrl_dev *pctldev,
				unsigned int selector,
				const char * const **groups,
				unsigned int * const num_groups)
{
	struct tacna_pin_private *priv = pinctrl_dev_get_drvdata(pctldev);

	*groups = tacna_mux_funcs[selector].group_names;

	if (tacna_mux_funcs[selector].func == 0) {
		/* alt func always maps to a single group */
		*num_groups = 1;
	} else {
		/* other funcs map to all available gpio pins */
		*num_groups = priv->chip->n_pins;
	}

	return 0;
}

static int tacna_mux_set_mux(struct pinctrl_dev *pctldev, unsigned int selector,
			     unsigned int group)
{
	struct tacna_pin_private *priv = pinctrl_dev_get_drvdata(pctldev);
	struct tacna *tacna = priv->tacna;
	const struct tacna_pin_groups *pin_group = priv->chip->pin_groups;
	unsigned int n_chip_groups = priv->chip->n_pin_groups;
	const char *func_name = tacna_mux_funcs[selector].name;
	unsigned int reg;
	int i, ret;

	dev_dbg(priv->dev, "%s selecting %u (%s) for group %u (%s)\n",
		__func__, selector, func_name, group,
		tacna_get_group_name(pctldev, group));

	if (tacna_mux_funcs[selector].func == 0) {
		/* alt func pin assignments are codec-specific */
		for (i = 0; i < n_chip_groups; ++i) {
			if (strcmp(func_name, pin_group->name) == 0)
				break;

			++pin_group;
		}

		if (i == n_chip_groups)
			return -EINVAL;

		for (i = 0; i < pin_group->n_pins; ++i) {
			reg = TACNA_GPIO1_CTRL1 + (4 * pin_group->pins[i]);

			dev_dbg(priv->dev, "%s setting 0x%x func bits to 0\n",
				__func__, reg);

			ret = regmap_update_bits(tacna->regmap, reg,
						 TACNA_GP1_FN_MASK, 0);
			if (ret)
				break;

		}
	} else {
		/*
		 * for other funcs the group will be the gpio number and will
		 * be offset by the number of chip-specific functions at the
		 * start of the group list
		 */
		group -= n_chip_groups;
		reg = TACNA_GPIO1_CTRL1 + (4 * group);

		dev_dbg(priv->dev, "%s setting 0x%x func bits to 0x%x\n",
			__func__, reg, tacna_mux_funcs[selector].func);

		ret = regmap_update_bits(tacna->regmap,
					 reg,
					 TACNA_GP1_FN_MASK,
					 tacna_mux_funcs[selector].func);
	}

	if (ret)
		dev_err(priv->dev, "Failed to write to 0x%x (%d)\n",
			reg, ret);

	return ret;
}

static const struct pinmux_ops tacna_pin_mux_ops = {
	.get_functions_count = &tacna_mux_get_funcs_count,
	.get_function_name = &tacna_mux_get_func_name,
	.get_function_groups = &tacna_mux_get_groups,
	.set_mux = &tacna_mux_set_mux,
};

static int tacna_pin_conf_get(struct pinctrl_dev *pctldev, unsigned int pin,
			      unsigned long *config)
{
	struct tacna_pin_private *priv = pinctrl_dev_get_drvdata(pctldev);
	unsigned int param = pinconf_to_config_param(*config);
	unsigned int result = 0;
	unsigned int conf;
	int ret;

	ret = regmap_read(priv->tacna->regmap, TACNA_GPIO1_CTRL1 + (4 * pin),
			  &conf);
	if (ret) {
		dev_err(priv->dev, "Failed to read GP%d conf (%d)\n",
			pin + 1, ret);
		return ret;
	}

	switch (param) {
	case PIN_CONFIG_BIAS_BUS_HOLD:
		conf &= TACNA_GP1_PU_MASK | TACNA_GP1_PD_MASK;
		if (conf == (TACNA_GP1_PU | TACNA_GP1_PD))
			result = 1;
		break;
	case PIN_CONFIG_BIAS_DISABLE:
		conf &= TACNA_GP1_PU_MASK | TACNA_GP1_PD_MASK;
		if (!conf)
			result = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		conf &= TACNA_GP1_PU_MASK | TACNA_GP1_PD_MASK;
		if (conf == TACNA_GP1_PD_MASK)
			result = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		conf &= TACNA_GP1_PU_MASK | TACNA_GP1_PD_MASK;
		if (conf == TACNA_GP1_PU_MASK)
			result = 1;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		if (conf & TACNA_GP1_OP_CFG_MASK)
			result = 1;
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		if (!(conf & TACNA_GP1_OP_CFG_MASK))
			result = 1;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		result = tacna_pin_unmake_drv_str(priv, conf);
		break;
	case PIN_CONFIG_INPUT_DEBOUNCE:
		dev_dbg(priv->dev,
			"Input debounce time not supported.");
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		if (conf & TACNA_GP1_DIR_MASK)
			result = 1;
		break;
	case PIN_CONFIG_OUTPUT:
		if ((conf & TACNA_GP1_DIR_MASK) &&
		    (conf & TACNA_GP1_LVL_MASK))
			result = 1;
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, result);

	return 0;
}

static int tacna_pin_conf_set(struct pinctrl_dev *pctldev, unsigned int pin,
			      unsigned long *configs, unsigned int num_configs)
{
	struct tacna_pin_private *priv = pinctrl_dev_get_drvdata(pctldev);
	unsigned int conf = 0;
	unsigned int mask = 0;
	unsigned int reg = TACNA_GPIO1_CTRL1 + (4 * pin);
	unsigned int val;
	int ret;

	while (num_configs) {
		dev_dbg(priv->dev, "%s config 0x%lx\n", __func__, *configs);

		switch (pinconf_to_config_param(*configs)) {
		case PIN_CONFIG_BIAS_BUS_HOLD:
			mask |= TACNA_GP1_PU_MASK | TACNA_GP1_PD_MASK;
			conf |= TACNA_GP1_PU | TACNA_GP1_PD;
			break;
		case PIN_CONFIG_BIAS_DISABLE:
			mask |= TACNA_GP1_PU_MASK | TACNA_GP1_PD_MASK;
			conf &= ~(TACNA_GP1_PU | TACNA_GP1_PD);
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			mask |= TACNA_GP1_PU_MASK | TACNA_GP1_PD_MASK;
			conf |= TACNA_GP1_PD;
			conf &= ~TACNA_GP1_PU;
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			mask |= TACNA_GP1_PU_MASK | TACNA_GP1_PD_MASK;
			conf |= TACNA_GP1_PU;
			conf &= ~TACNA_GP1_PD;
			break;
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			mask |= TACNA_GP1_OP_CFG_MASK;
			conf |= TACNA_GP1_OP_CFG;
			break;
		case PIN_CONFIG_DRIVE_PUSH_PULL:
			mask |= TACNA_GP1_OP_CFG_MASK;
			conf &= ~TACNA_GP1_OP_CFG;
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			val = pinconf_to_config_argument(*configs);
			mask |= TACNA_GP1_DRV_STR_MASK;
			conf &= ~TACNA_GP1_DRV_STR_MASK;
			conf |= tacna_pin_make_drv_str(priv, val);
			break;
		case PIN_CONFIG_INPUT_DEBOUNCE:
			dev_dbg(priv->dev,
				"Input debounce time not supported.");
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			val = pinconf_to_config_argument(*configs);
			mask |= TACNA_GP1_DIR_MASK;
			if (val)
				conf |= TACNA_GP1_DIR;
			else
				conf &= ~TACNA_GP1_DIR;
			break;
		case PIN_CONFIG_OUTPUT:
			val = pinconf_to_config_argument(*configs);
			mask |= TACNA_GP1_LVL_MASK;
			if (val)
				conf |= TACNA_GP1_LVL;
			else
				conf &= ~TACNA_GP1_LVL;

			mask |= TACNA_GP1_DIR_MASK;
			conf &= ~TACNA_GP1_DIR;
			break;
		default:
			return -ENOTSUPP;
		}

		++configs;
		--num_configs;
	}

	dev_dbg(priv->dev, "%s gpio%d 0x%x:0x%x\n",
		__func__, pin + 1, reg, conf);

	ret = regmap_update_bits(priv->tacna->regmap, reg, mask, conf);
	if (ret)
		dev_err(priv->dev,
			"Failed to write GPIO%d conf (%d) reg 0x%x\n",
			pin + 1, ret, reg);

	return ret;
}

static int tacna_pin_conf_group_set(struct pinctrl_dev *pctldev,
				    unsigned int selector,
				    unsigned long *configs,
				    unsigned int num_configs)
{
	struct tacna_pin_private *priv = pinctrl_dev_get_drvdata(pctldev);
	const struct tacna_pin_groups *pin_group;
	unsigned int n_groups = priv->chip->n_pin_groups;
	int i, ret;

	dev_dbg(priv->dev, "%s setting group %s\n", __func__,
		tacna_get_group_name(pctldev, selector));

	if (selector >= n_groups) {
		/* group is a single pin, convert to pin number and set */
		return tacna_pin_conf_set(pctldev,
					  selector - n_groups,
					  configs,
					  num_configs);
	} else {
		pin_group = &priv->chip->pin_groups[selector];

		for (i = 0; i < pin_group->n_pins; ++i) {
			ret = tacna_pin_conf_set(pctldev,
						 pin_group->pins[i],
						 configs,
						 num_configs);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static const struct pinconf_ops tacna_pin_conf_ops = {
	.is_generic = true,
	.pin_config_get = &tacna_pin_conf_get,
	.pin_config_set = &tacna_pin_conf_set,
	.pin_config_group_set = &tacna_pin_conf_group_set,

};

static struct pinctrl_desc tacna_pin_desc = {
	.name = "tacna-pinctrl",
	.pins = tacna_pins,
	.pctlops = &tacna_pin_group_ops,
	.pmxops = &tacna_pin_mux_ops,
	.confops = &tacna_pin_conf_ops,
	.owner = THIS_MODULE,
};

static int tacna_pin_probe(struct platform_device *pdev)
{
	struct tacna *tacna = dev_get_drvdata(pdev->dev.parent);
	const struct tacna_pdata *pdata = dev_get_platdata(tacna->dev);
	struct tacna_pin_private *priv;
	int ret;

	BUILD_BUG_ON(ARRAY_SIZE(tacna_pin_single_group_names) !=
		     ARRAY_SIZE(tacna_pin_single_group_pins));

	dev_dbg(&pdev->dev, "%s\n", __func__);

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->dev = &pdev->dev;
	priv->tacna = tacna;

	switch (tacna->type) {
	case CS47L96:
	case CS47L97:
#ifdef CONFIG_PINCTRL_CS47L96
		priv->chip = &cs47l96_pin_chip;
#endif
		break;
	case CS48L31:
	case CS48L32:
	case CS48L33:
#ifdef CONFIG_PINCTRL_CS48L32
		priv->chip = &cs48l32_pin_chip;
#endif
		break;
	case CS48LX50:
#ifdef CONFIG_PINCTRL_CS48LX50
		priv->chip = &cs48lx50_pin_chip;
#endif
		break;
	default:
		break;
	}

	if (!priv->chip)
		return -ENODEV;

	tacna_pin_desc.npins = priv->chip->n_pins;

	priv->pctl = pinctrl_register(&tacna_pin_desc, &pdev->dev, priv);
	if (IS_ERR(priv->pctl)) {
		ret = PTR_ERR(priv->pctl);
		dev_err(priv->dev, "Failed pinctrl register (%d)\n", ret);
		return ret;
	}

	if (pdata && pdata->gpio_configs) {
		ret = pinctrl_register_mappings(pdata->gpio_configs,
						pdata->n_gpio_configs);
		dev_err(priv->dev, "Failed to register pdata mappings (%d)\n",
			ret);
		return ret;
	}

	dev_dbg(priv->dev, "pinctrl registered\n");

	return 0;
}

static int tacna_pin_remove(struct platform_device *pdev)
{
	struct tacna_pin_private *priv = platform_get_drvdata(pdev);

	pinctrl_unregister(priv->pctl);

	return 0;
}

static struct platform_driver tacna_pin_driver = {
	.probe = &tacna_pin_probe,
	.remove = &tacna_pin_remove,
	.driver = {
		.name = "tacna-pinctrl",
	},
};

module_platform_driver(tacna_pin_driver);

MODULE_DESCRIPTION("Tacna pinctrl driver");
MODULE_AUTHOR("Richard Fitzgerald <rf@opensource.wolfsonmicro.com>");
MODULE_AUTHOR("Piotr Stankiewicz <piotrs@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL v2");
