/*
 * Core MFD support for Cirrus Logic Tacna codecs
 *
 * Copyright 2016-2018 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/property.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#include <linux/mfd/tacna/core.h>
#include <linux/mfd/tacna/registers.h>
#include <dt-bindings/mfd/tacna.h>

#include "tacna.h"

#define CS47L96_SILICON_ID	0x47a97
#define CS48L32_SILICON_ID	0x48a32
#define CS48L33_SILICON_ID	0x48a33

#define TACNA_32K_MCLK1		0
#define TACNA_32K_MCLK2		1

#define TACNA_SEEN_BOOT_DONE	0x1

#define TACNA_BOOT_TIMEOUT_MS	25
#define CS47L96_SYSCLK_POLL_MS	2

static const char * const tacna_core_supplies[] = {
	"VDD_A",
	"VDD_IO1",
};

static const struct mfd_cell tacna_pinctrl_dev[] = {
	{
		.name = "tacna-pinctrl",
		.of_compatible = "cirrus,tacna-pinctrl",
	},
};

static const char * const cs47l96_supplies[] = {
	"VOUT_MIC",	/* must be first entry */
	"VDD1_CP",
	"VDD2_CP",
	"VDD3_CP",
	"VDD_IO2",
};

static const struct mfd_cell cs47l96_devs[] = {
	{ .name = "tacna-irq", },
	{ .name = "tacna-micsupp", },
	{ .name = "tacna-gpio", },
	{
		.name = "tacna-extcon",
		.parent_supplies = cs47l96_supplies,
		.num_parent_supplies = 1,	/* only need VOUT_MIC */
	},
	{
		.name = "cs47l96-codec",
		.parent_supplies = cs47l96_supplies,
		.num_parent_supplies = ARRAY_SIZE(cs47l96_supplies),
	},
	{
		.name = "cs47l96-ao-codec",
		.parent_supplies = cs47l96_supplies,
		.num_parent_supplies = ARRAY_SIZE(cs47l96_supplies),
		.of_compatible = "cirrus,cs47l96-ao",
	},
};

static const char * const cs48l32_supplies[] = {
	"VOUT_MIC",	/* must be first entry */
	"VDD1_CP",
};

static const struct mfd_cell cs48l32_devs[] = {
	{ .name = "tacna-irq", },
	{ .name = "tacna-micsupp", },
	{ .name = "tacna-gpio", },
	{
		.name = "cs48l32-codec",
		.parent_supplies = cs48l32_supplies,
		.num_parent_supplies = ARRAY_SIZE(cs48l32_supplies),
	},
};

const char *tacna_name_from_type(enum tacna_type type)
{
	switch (type) {
	case CS47L96:
		return "CS47L96";
	case CS47L97:
		return "CS47L97";
	case CS48L32:
		return "CS48L32";
	case CS48L33:
		return "CS48L33";
	default:
		return "Unknown";
	}
}
EXPORT_SYMBOL_GPL(tacna_name_from_type);

static int tacna_wait_for_boot(struct tacna *tacna)
{
	unsigned int val, reg;
	int i, ret;

	switch (tacna->type) {
	case CS47L96:
	case CS47L97:
		reg = TACNA_HOST_SCRATCH;
		break;
	default:
		reg = TACNA_CTRL_IF_DEBUG3;
		break;
	}

	/* Reads could fail while the chip is booting so don't fail on errors */
	ret = regmap_read(tacna->regmap, reg, &val);
	if (ret == 0) {
		/* No need to wait for boot if VDD_D didn't power off */
		if (val & TACNA_SEEN_BOOT_DONE) {
			dev_warn(tacna->dev, "VDD_D didn't power off when expected\n");
			return 0;
		}
	}

	/* regmap_read_poll_timeout would fail on read errors so roll our own */
	for (i = 0; i < TACNA_BOOT_TIMEOUT_MS; ++i) {
		val = 0;
		regmap_read(tacna->regmap, TACNA_IRQ1_EINT_2, &val);
		if (val & TACNA_BOOT_DONE_EINT1_MASK)
			break;

		usleep_range(1000, 2000);
	}
	if (i == TACNA_BOOT_TIMEOUT_MS) {
		dev_err(tacna->dev, "BOOT_DONE timed out\n");
		return -ETIMEDOUT;
	}

	ret = regmap_read(tacna->regmap, TACNA_MCU_CTRL1, &val);
	if (ret) {
		dev_err(tacna->dev, "Failed to read MCU_CTRL1: %d\n", ret);
		return ret;
	}

	if (val & (1 << TACNA_MCU_STS_SHIFT)) {
		dev_err(tacna->dev, "MCU boot failed\n");
		return -EIO;
	}

	ret = regmap_update_bits(tacna->regmap, reg,
				 TACNA_SEEN_BOOT_DONE,
				 TACNA_SEEN_BOOT_DONE);
	if (ret) {
		dev_err(tacna->dev, "Failed to update 0x%x : %d\n", reg, ret);
		return ret;
	}

	switch (tacna->type) {
	case CS47L96:
	case CS47L97:
		/* wait for internal SYSCLK request and clear */
		regmap_read_poll_timeout(tacna->regmap, TACNA_IRQ1_EINT_1, val,
					 (val & TACNA_SYSCLK_ERR_EINT1),
					 500,
					 CS47L96_SYSCLK_POLL_MS * 1000);

		regmap_write(tacna->regmap, TACNA_IRQ1_EINT_1,
			     TACNA_SYSCLK_ERR_EINT1);
		break;
	default:
		break;
	}

	pm_runtime_mark_last_busy(tacna->dev);

	return 0;
}

static int tacna_soft_reset(struct tacna *tacna)
{
	int ret;

	ret = regmap_write(tacna->regmap, TACNA_SFT_RESET,
			   0x5a << TACNA_SFT_RESET_SHIFT);
	if (ret != 0) {
		dev_err(tacna->dev, "Failed to soft reset device: %d\n", ret);
		return ret;
	}

	usleep_range(2000, 3000);

	return 0;
}

static void tacna_enable_hard_reset(struct tacna *tacna)
{
	if (tacna->reset_gpio)
		gpiod_set_value_cansleep(tacna->reset_gpio, 0);
}

static void tacna_disable_hard_reset(struct tacna *tacna)
{
	if (tacna->reset_gpio) {
		gpiod_set_value_cansleep(tacna->reset_gpio, 1);
		usleep_range(2000, 3000);
	}
}

#ifdef CONFIG_PM
static int tacna_runtime_resume(struct device *dev)
{
	struct tacna *tacna = dev_get_drvdata(dev);
	int ret;

	dev_dbg(tacna->dev, "Leaving sleep mode\n");

	ret = regulator_enable(tacna->vdd_d);
	if (ret) {
		dev_err(tacna->dev, "Failed to enable VDD_D: %d\n", ret);
		return ret;
	}

	usleep_range(2000, 3000);

	regcache_cache_only(tacna->regmap, false);

	ret = tacna_wait_for_boot(tacna);
	if (ret)
		goto err;

	ret = regcache_sync(tacna->regmap);
	if (ret) {
		dev_err(tacna->dev,
			"Failed to restore register cache\n");
		goto err;
	}

	return 0;

err:
	regcache_cache_only(tacna->regmap, true);
	regulator_disable(tacna->vdd_d);
	return ret;
}

static int tacna_runtime_suspend(struct device *dev)
{
	struct tacna *tacna = dev_get_drvdata(dev);

	dev_dbg(tacna->dev, "Entering sleep mode\n");

	regcache_cache_only(tacna->regmap, true);
	regcache_mark_dirty(tacna->regmap);

	regulator_disable(tacna->vdd_d);

	return 0;
}
#endif

const struct dev_pm_ops tacna_pm_ops = {
	SET_RUNTIME_PM_OPS(tacna_runtime_suspend,
			   tacna_runtime_resume,
			   NULL)
};
EXPORT_SYMBOL_GPL(tacna_pm_ops);

unsigned int tacna_get_num_micbias(struct tacna *tacna)
{

	switch (tacna->type) {
	case CS47L96:
	case CS47L97:
		return 2;
	case CS48L32:
	case CS48L33:
		return 1;
	default:
		return 0;
	}
}
EXPORT_SYMBOL_GPL(tacna_get_num_micbias);

unsigned int tacna_get_num_childbias(struct tacna *tacna, unsigned int micbias)
{
	switch (tacna->type) {
	case CS47L96:
	case CS47L97:
		switch (micbias) {
		case 0:
			return 4;
		case 1:
			return 2;
		default:
			return 0;
		}
	case CS48L32:
	case CS48L33:
		switch (micbias) {
		case 0:
			return 3;
		default:
			return 0;
		}
	default:
		return 0;
	}
}
EXPORT_SYMBOL_GPL(tacna_get_num_childbias);

#ifdef CONFIG_OF
const struct of_device_id tacna_of_match[] = {
	{ .compatible = "cirrus,cs47l96", .data = (void *)CS47L96 },
	{ .compatible = "cirrus,cs47l97", .data = (void *)CS47L97 },
	{ .compatible = "cirrus,cs48l32", .data = (void *)CS48L32 },
	{ .compatible = "cirrus,cs48l33", .data = (void *)CS48L33 },
	{},
};
EXPORT_SYMBOL_GPL(tacna_of_match);

unsigned long tacna_of_get_type(struct device *dev)
{
	const struct of_device_id *id = of_match_device(tacna_of_match, dev);

	if (id)
		return (unsigned long)id->data;
	else
		return 0;
}
EXPORT_SYMBOL_GPL(tacna_of_get_type);
#endif

static void tacna_prop_report_error(struct tacna *tacna, const char *prop,
				    bool mandatory, int err)
{
	switch (err) {
	case -EPROBE_DEFER:
		return;
	case -EINVAL:
		/* property API uses -EINVAL if a property does not exist */
		if (mandatory)
			dev_err(tacna->dev,
				"Mandatory DT property %s is missing\n", prop);
		break;
	default:
		dev_err(tacna->dev,
			"DT property %s is malformed: %d\n", prop, err);
		break;
	}
}

static void tacna_prop_get_micbias_child(struct tacna *tacna,
					 const char *name,
					 struct tacna_micbias_pin_pdata *pdata)
{
	struct device_node *np;
	struct regulator_desc desc = { };

	np = of_get_child_by_name(tacna->dev->of_node, name);
	if (!np)
		return;

	desc.name = name;
	pdata->init_data = of_get_regulator_init_data(tacna->dev, np, &desc);
	of_property_read_u32(np, "regulator-active-discharge",
			     &pdata->active_discharge);
}

static void tacna_prop_get_micbias_gen(struct tacna *tacna,
				       const char *name,
				       struct tacna_micbias_pdata *pdata)
{
	struct device_node *np;
	struct regulator_desc desc = { };

	np = of_get_child_by_name(tacna->dev->of_node, name);
	if (!np)
		return;

	desc.name = name;
	pdata->init_data = of_get_regulator_init_data(tacna->dev, np, &desc);
	pdata->ext_cap = of_property_read_bool(np, "wlf,ext-cap");
	of_property_read_u32(np, "regulator-active-discharge",
			     &pdata->active_discharge);
}

static void tacna_prop_get_micbias(struct tacna *tacna)
{
	struct tacna_micbias_pdata *pdata;
	char name[10];
	int i, child;

	for (i = tacna_get_num_micbias(tacna) - 1; i >= 0; --i) {
		pdata = &tacna->pdata.micbias[i];

		snprintf(name, sizeof(name), "MICBIAS%d", i + 1);
		tacna_prop_get_micbias_gen(tacna, name, pdata);

		child = tacna_get_num_childbias(tacna, i) - 1;
		for (; child >= 0; --child) {
			snprintf(name, sizeof(name), "MICBIAS%d%c",
				 i + 1, 'A' + child);
			tacna_prop_get_micbias_child(tacna, name,
						     &pdata->pin[child]);
		}
	}
}

static int tacna_prop_get_core_pdata(struct tacna *tacna)
{
	int ret;

	tacna->reset_gpio = devm_gpiod_get_optional(tacna->dev,
						    "reset",
						    GPIOD_OUT_LOW);
	if (IS_ERR(tacna->reset_gpio)) {
		ret = PTR_ERR(tacna->reset_gpio);
		tacna_prop_report_error(tacna, "reset-gpio", false, ret);
		return ret;
	}

	tacna_prop_get_micbias(tacna);

	of_property_read_u32(tacna->dev->of_node, "cirrus,clk32k-src",
			     &tacna->pdata.clk32k_src);

	return 0;
}

static void tacna_configure_micbias(struct tacna *tacna)
{
	unsigned int num_micbias = tacna_get_num_micbias(tacna);
	struct tacna_micbias_pdata *pdata;
	struct regulator_init_data *init_data;
	unsigned int num_child_micbias;
	unsigned int val, mask, reg;
	int i, child, ret;

	for (i = 0; i < num_micbias; ++i) {
		pdata = &tacna->pdata.micbias[i];

		/* Configure the child micbias pins */
		val = 0;
		mask = 0;
		num_child_micbias = tacna_get_num_childbias(tacna, i);
		for (child = 0; child < num_child_micbias; ++child) {
			if (!pdata->pin[child].init_data)
				continue;

			mask |= TACNA_MICB1A_DISCH << (child * 4);
			if (pdata->pin[child].active_discharge)
				val |= TACNA_MICB1A_DISCH << (child * 4);
		}

		if (mask) {
			reg = TACNA_MICBIAS_CTRL5 + (i * 4);
			ret = regmap_update_bits(tacna->regmap, reg, mask, val);
			if (ret)
				dev_warn(tacna->dev,
					 "Failed to write 0x%x (%d)\n",
					 reg, ret);

			dev_dbg(tacna->dev,
				"Set MICBIAS_CTRL%d mask=0x%x val=0x%x\n",
				i + 5, mask, val);
		}

		/* configure the parent */
		init_data = pdata->init_data;
		if (!init_data)
			continue;

		mask = TACNA_MICB1_LVL_MASK | TACNA_MICB1_EXT_CAP |
			TACNA_MICB1_BYPASS | TACNA_MICB1_RATE;

		if (!init_data->constraints.max_uV)
			init_data->constraints.max_uV = 2800;

		val = (init_data->constraints.max_uV - 1500000) / 100000;
		val <<= TACNA_MICB1_LVL_SHIFT;

		if (pdata->ext_cap)
			val |= TACNA_MICB1_EXT_CAP;

		/* if no child biases the discharge is set in the parent */
		if (num_child_micbias == 0) {
			mask |= TACNA_MICB1_DISCH;

			if (pdata->active_discharge)
				val |= TACNA_MICB1_DISCH;
		}

		if (init_data->constraints.soft_start)
			val |= TACNA_MICB1_RATE;

		if (init_data->constraints.valid_ops_mask &
		    REGULATOR_CHANGE_BYPASS)
			val |= TACNA_MICB1_BYPASS;

		reg = TACNA_MICBIAS_CTRL1 + (i * 4);
		ret = regmap_update_bits(tacna->regmap, reg, mask, val);
		if (ret)
			dev_warn(tacna->dev, "Failed to write 0x%x (%d)\n",
				 reg, ret);

		dev_dbg(tacna->dev, "Set MICBIAS_CTRL%d mask=0x%x val=0x%x\n",
			i + 1, mask, val);
	}
}

static int tacna_dev_select_pinctrl(struct tacna *tacna,
				    struct pinctrl *pinctrl,
				    const char *name)
{
	struct pinctrl_state *pinctrl_state;
	int ret;

	pinctrl_state = pinctrl_lookup_state(pinctrl, name);

	/* it's ok if it doesn't exist */
	if (!IS_ERR(pinctrl_state)) {
		dev_dbg(tacna->dev, "Applying pinctrl %s state\n", name);
		ret = pinctrl_select_state(pinctrl, pinctrl_state);
		if (ret) {
			dev_err(tacna->dev,
				"Failed to select pinctrl %s state: %d\n",
				name, ret);
			return ret;
		}
	}

	return 0;
}

static int tacna_configure_clk32k(struct tacna *tacna)
{
	unsigned int mclk_src;
	int ret = 0;

	switch (tacna->pdata.clk32k_src) {
	case 0:
		/* Default to something typical for the part */
		switch (tacna->type) {
		case CS48L32:
		case CS48L33:
			mclk_src = TACNA_32K_MCLK1;
			break;
		default:
			mclk_src = TACNA_32K_MCLK2;
			break;
		}
		break;
	case TACNA_32KZ_MCLK1:
	case TACNA_32KZ_MCLK2:
	case TACNA_32KZ_SYSCLK:
		mclk_src = tacna->pdata.clk32k_src - 1;
		break;
	default:
		dev_err(tacna->dev, "Invalid 32kHz clock source: %d\n",
			tacna->pdata.clk32k_src);
		return -EINVAL;
	}

	ret = regmap_update_bits(tacna->regmap,
			TACNA_CLOCK32K,
			TACNA_CLK_32K_EN_MASK | TACNA_CLK_32K_SRC_MASK,
			TACNA_CLK_32K_EN | mclk_src);
	if (ret)
		dev_err(tacna->dev, "Failed to init 32k clock: %d\n", ret);

	return ret;
}

int tacna_dev_init(struct tacna *tacna)
{
	struct device *dev = tacna->dev;
	const char *name;
	unsigned int hwid;
	int (*patch_fn)(struct tacna *) = NULL;
	const struct mfd_cell *mfd_devs;
	struct pinctrl *pinctrl;
	int n_devs, i;
	int ret;

	dev_set_drvdata(tacna->dev, tacna);

	BLOCKING_INIT_NOTIFIER_HEAD(&tacna->notifier);

	regcache_cache_only(tacna->regmap, true);

	if (dev_get_platdata(tacna->dev)) {
		memcpy(&tacna->pdata, dev_get_platdata(tacna->dev),
		       sizeof(tacna->pdata));

		/* We use 0 in pdata to indicate a GPIO has not been set */
		if (tacna->pdata.reset > 0) {
			/* Start out with RESET_B asserted */
			ret = devm_gpio_request_one(tacna->dev,
						tacna->pdata.reset,
						GPIOF_DIR_OUT | GPIOF_INIT_LOW,
						"tacna reset");
			if (ret) {
				dev_err(dev, "Failed to request RESET_B: %d\n",
					ret);
				return ret;
			}

			tacna->reset_gpio = gpio_to_desc(tacna->pdata.reset);
		}
	} else {
		ret = tacna_prop_get_core_pdata(tacna);
		if (ret)
			return ret;
	}

	if (!tacna->reset_gpio)
		dev_warn(tacna->dev,
			 "Running without reset GPIO is not recommended\n");

	for (i = 0; i < ARRAY_SIZE(tacna_core_supplies); i++)
		tacna->core_supplies[i].supply = tacna_core_supplies[i];

	tacna->num_core_supplies = ARRAY_SIZE(tacna_core_supplies);

	ret = devm_regulator_bulk_get(dev, tacna->num_core_supplies,
				      tacna->core_supplies);
	if (ret) {
		dev_err(dev, "Failed to request core supplies: %d\n", ret);
		return ret;
	}

	tacna->vdd_d = devm_regulator_get(tacna->dev, "VDD_D");
	if (IS_ERR(tacna->vdd_d)) {
		ret = PTR_ERR(tacna->vdd_d);
		dev_err(dev, "Failed to request VDD_D: %d\n", ret);
		return ret;
	}

	/*
	 * Pinctrl subsystem only configures pinctrls if all referenced pins
	 * are registered. Create our pinctrl child now so that its pins exist
	 * otherwise external pinctrl dependencies will fail
	 * Note: Can't devm_ because it is cleaned up after children are already
	 * destroyed
	 */
	ret = mfd_add_devices(tacna->dev, PLATFORM_DEVID_NONE,
			      tacna_pinctrl_dev, 1, NULL, 0, NULL);
	if (ret) {
		dev_err(tacna->dev, "Failed to add pinctrl child: %d\n", ret);
		return ret;
	}

	pinctrl = pinctrl_get(dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		dev_err(tacna->dev, "Failed to get pinctrl: %d\n", ret);
		goto err_pinctrl_dev;
	}

	/* Use (optional) minimal config with only external pin bindings */
	ret = tacna_dev_select_pinctrl(tacna, pinctrl, "probe");
	if (ret)
		goto err_pinctrl;

	ret = regulator_set_voltage(tacna->vdd_d, 1200000, 1200000);
	if (ret) {
		dev_err(dev, "Failed to request VDD_D=1.2v: %d\n", ret);
		goto err_pinctrl;
	}

	ret = regulator_bulk_enable(tacna->num_core_supplies,
				    tacna->core_supplies);
	if (ret) {
		dev_err(dev, "Failed to enable core supplies: %d\n", ret);
		goto err_pinctrl;
	}

	ret = regulator_enable(tacna->vdd_d);
	if (ret) {
		dev_err(dev, "Failed to enable VDD_D: %d\n", ret);
		goto err_enable;
	}

	tacna_disable_hard_reset(tacna);

	regcache_cache_only(tacna->regmap, false);

	/* If we don't have a reset GPIO use a soft reset */
	if (!tacna->reset_gpio) {
		ret = tacna_soft_reset(tacna);
		if (ret)
			goto err_reset;
	}

	ret = tacna_wait_for_boot(tacna);
	if (ret) {
		dev_err(tacna->dev, "Device failed initial boot: %d\n", ret);
		goto err_reset;
	}

	/* Read the device ID information & do device specific stuff */
	ret = regmap_read(tacna->regmap, TACNA_DEVID, &hwid);
	if (ret) {
		dev_err(dev, "Failed to read ID register: %d\n", ret);
		goto err_reset;
	}
	hwid &= TACNA_DEVID_MASK;

	switch (hwid) {
	case CS47L96_SILICON_ID:
	case CS48L32_SILICON_ID:
	case CS48L33_SILICON_ID:
		break;
	default:
		dev_err(tacna->dev, "Unknown device ID: %x\n", hwid);
		ret = -EINVAL;
		goto err_reset;
	}

	ret = regmap_read(tacna->regmap, TACNA_REVID, &tacna->rev);
	if (ret) {
		dev_err(dev, "Failed to read revision register: %d\n", ret);
		goto err_reset;
	}
	tacna->rev &= TACNA_AREVID_MASK | TACNA_MTLREVID_MASK;

	ret = regmap_read(tacna->regmap, TACNA_OTPID, &tacna->otp_rev);
	if (ret) {
		dev_err(dev, "Failed to read OTP revision register: %d\n", ret);
		goto err_reset;
	}
	tacna->otp_rev &= TACNA_OTPID_MASK;

	name = tacna_name_from_type(tacna->type);

	n_devs = 0;

	switch (hwid) {
	case CS47L96_SILICON_ID:
		if (IS_ENABLED(CONFIG_MFD_CS47L96)) {
			switch (tacna->type) {
			case CS47L96:
			case CS47L97:
				patch_fn = cs47l96_patch;
				mfd_devs = cs47l96_devs;
				n_devs = ARRAY_SIZE(cs47l96_devs);
				break;
			default:
				break;
			}
		}
		break;
	case CS48L32_SILICON_ID:
		if (IS_ENABLED(CONFIG_MFD_CS48L32)) {
			switch (tacna->type) {
			case CS48L32:
				patch_fn = cs48l32_patch;
				mfd_devs = cs48l32_devs;
				n_devs = ARRAY_SIZE(cs48l32_devs);
				break;
			default:
				break;
			}
		}
		break;
	case CS48L33_SILICON_ID:
		if (IS_ENABLED(CONFIG_MFD_CS48L33)) {
			switch (tacna->type) {
			case CS48L33:
				patch_fn = cs48l32_patch;
				mfd_devs = cs48l32_devs;
				n_devs = ARRAY_SIZE(cs48l32_devs);
				break;
			default:
				break;
			}
		}
		break;
	default:
		break;
	}

	if (!n_devs) {
		dev_err(tacna->dev, "Device ID 0x%x is not a %s\n", hwid, name);
		ret = -ENODEV;
		goto err_reset;
	}

	dev_info(dev, "%s revision %X%u.%u\n", name,
		 tacna->rev >> TACNA_AREVID_SHIFT,
		 tacna->rev & TACNA_MTLREVID_MASK,
		 tacna->otp_rev);

	/* Apply hardware patch */
	if (patch_fn) {
		ret = patch_fn(tacna);
		if (ret) {
			dev_err(tacna->dev, "Failed to apply patch %d\n", ret);
			goto err_reset;
		}
	}

	/* Apply (optional) main pinctrl config, this will configure our pins */
	ret = tacna_dev_select_pinctrl(tacna, pinctrl, "active");
	if (ret)
		goto err_reset;

	ret = tacna_configure_clk32k(tacna);
	if (ret)
		goto err_reset;

	/* default headphone impedance in case the extcon driver is not used */
	for (i = 0; i < ARRAY_SIZE(tacna->hp_impedance_x100); ++i)
		tacna->hp_impedance_x100[i] = 3200;

	tacna_configure_micbias(tacna);

	pm_runtime_set_active(tacna->dev);
	pm_runtime_enable(tacna->dev);
	pm_runtime_set_autosuspend_delay(tacna->dev, 100);
	pm_runtime_use_autosuspend(tacna->dev);

	ret = mfd_add_devices(tacna->dev, PLATFORM_DEVID_NONE, mfd_devs, n_devs,
			      NULL, 0, NULL);
	if (ret) {
		dev_err(tacna->dev, "Failed to add subdevices: %d\n", ret);
		goto err_reset;
	}

	pinctrl_put(pinctrl);

	return 0;

err_reset:
	tacna_enable_hard_reset(tacna);
	regulator_disable(tacna->vdd_d);
err_enable:
	regulator_bulk_disable(tacna->num_core_supplies, tacna->core_supplies);
err_pinctrl:
	pinctrl_put(pinctrl);
err_pinctrl_dev:
	mfd_remove_devices(dev);

	return ret;
}
EXPORT_SYMBOL_GPL(tacna_dev_init);

int tacna_dev_exit(struct tacna *tacna)
{
	pm_runtime_disable(tacna->dev);

	regulator_disable(tacna->vdd_d);

	mfd_remove_devices(tacna->dev);
	tacna_enable_hard_reset(tacna);

	regulator_bulk_disable(tacna->num_core_supplies, tacna->core_supplies);
	return 0;
}
EXPORT_SYMBOL_GPL(tacna_dev_exit);
