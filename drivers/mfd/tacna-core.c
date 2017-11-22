/*
 * Core MFD support for Cirrus Logic Tacna codecs
 *
 * Copyright 2016-2017 Cirrus Logic
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
#include <linux/property.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#include <linux/mfd/tacna/core.h>
#include <linux/mfd/tacna/registers.h>

#include "tacna.h"

#define CS47L94_SILICON_ID	0x6372

#define TACNA_32K_MCLK2		1

#define TACNA_BOOT_POLL_MICROSECONDS    1000
#define TACNA_BOOT_TIMEOUT_MICROSECONDS 5000

static const char * const tacna_core_supplies[] = {
	"VDD_A",
	"VDD_IO1",
};

static const char * const cs47l94_supplies[] = {
	"VOUT_MIC",	/* must be first entry */
	"VDD1_CP",
	"VDD2_CP",
	"VDD3_CP",
	"VDD_IO2",
};

static const struct mfd_cell cs47l94_devs[] = {
	{ .name = "tacna-pinctrl", },
	{ .name = "tacna-irq", },
	{ .name = "tacna-micsupp", },
	{ .name = "tacna-gpio", },
	{
		.name = "tacna-extcon",
		.parent_supplies = cs47l94_supplies,
		.num_parent_supplies = 1,	/* only need VOUT_MIC */
	},
	{
		.name = "cs47l94-codec",
		.parent_supplies = cs47l94_supplies,
		.num_parent_supplies = ARRAY_SIZE(cs47l94_supplies),
	},
	{ .name = "tacna-haptics", },
};

const char *tacna_name_from_type(enum tacna_type type)
{
	switch (type) {
	case CS47L94:
		return "CS47L94";
	case CS47L95:
		return "CS47L95";
	default:
		return "Unknown";
	}
}
EXPORT_SYMBOL_GPL(tacna_name_from_type);

static int tacna_wait_for_boot(struct tacna *tacna)
{
	unsigned int val;
	int ret;

	/*
	 * We can't use an interrupt as we need to runtime resume to do so,
	 * we won't race with the interrupt handler as it'll be blocked on
	 * runtime resume.
	 */
	ret = regmap_read_poll_timeout(tacna->regmap, TACNA_IRQ1_EINT2, val,
				       (val & TACNA_BOOT_DONE_EINT1_MASK),
				       TACNA_BOOT_POLL_MICROSECONDS,
				       TACNA_BOOT_TIMEOUT_MICROSECONDS);
	if (ret) {
		dev_err(tacna->dev, "Failed to get BOOT_DONE: %d\n", ret);
		return ret;
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

	usleep_range(1000, 2000);

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
		usleep_range(1000, 2000);
	}
}

static int tacna_vdd_d_notify(struct notifier_block *nb,
			      unsigned long action, void *data)
{
	struct tacna *tacna = container_of(nb, struct tacna,
					   vdd_d_notifier);

	dev_dbg(tacna->dev, "VDD_D notify %lx\n", action);

	if (action & REGULATOR_EVENT_DISABLE)
		tacna->vdd_d_powered_off = true;

	return NOTIFY_DONE;
}

#ifdef CONFIG_PM
static int tacna_runtime_resume(struct device *dev)
{
	struct tacna *tacna = dev_get_drvdata(dev);
	bool force_reset = false;
	int ret;

	dev_dbg(tacna->dev, "Leaving sleep mode\n");

	/*
	 * If VDD_D didn't power off we must force a reset so that the
	 * cache syncs correctly. If we have a hardware reset this must
	 * be done before powering up VDD_D. If not, we'll use a software
	 * reset after powering-up VDD_D
	 */
	if (!tacna->vdd_d_powered_off) {
		dev_dbg(tacna->dev, "VDD_D did not power off, forcing reset\n");
		force_reset = true;
		tacna_enable_hard_reset(tacna);
	}

	ret = regulator_enable(tacna->vdd_d);
	if (ret) {
		dev_err(tacna->dev, "Failed to enable VDD_D: %d\n", ret);
		return ret;
	}

	regcache_cache_only(tacna->regmap, false);

	if (force_reset) {
		if (tacna->reset_gpio) {
			tacna_disable_hard_reset(tacna);
		} else {
			ret = tacna_soft_reset(tacna);
			if (ret)
				goto err;
		}
	}

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
	tacna->vdd_d_powered_off = false;
	regulator_disable(tacna->vdd_d);
	return ret;
}

static int tacna_runtime_suspend(struct device *dev)
{
	struct tacna *tacna = dev_get_drvdata(dev);

	dev_dbg(tacna->dev, "Entering sleep mode\n");

	regcache_cache_only(tacna->regmap, true);
	regcache_mark_dirty(tacna->regmap);

	tacna->vdd_d_powered_off = false;
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
	case CS47L94:
	case CS47L95:
		return 2;
	default:
		return 0;
	}
}
EXPORT_SYMBOL_GPL(tacna_get_num_micbias);

unsigned int tacna_get_num_childbias(struct tacna *tacna, unsigned int micbias)
{
	switch (tacna->type) {
	case CS47L94:
	case CS47L95:
		switch (micbias) {
		case 0:
			return 4;
		case 1:
			return 2;
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
	{ .compatible = "cirrus,cs47l94", .data = (void *)CS47L94 },
	{ .compatible = "cirrus,cs47l95", .data = (void *)CS47L95 },
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

int tacna_dev_init(struct tacna *tacna)
{
	struct device *dev = tacna->dev;
	const char *name;
	unsigned int hwid, reg;
	int (*patch_fn)(struct tacna *) = NULL;
	const struct mfd_cell *mfd_devs;
	int n_devs, i;
	int ret;

	dev_set_drvdata(tacna->dev, tacna);
	BLOCKING_INIT_NOTIFIER_HEAD(&tacna->notifier);

	/* default headphone impedance in case the extcon driver is not used */
	for (i = 0; i < ARRAY_SIZE(tacna->hp_impedance_x100); ++i)
		tacna->hp_impedance_x100[i] = 3200;

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

	regcache_cache_only(tacna->regmap, true);

	for (i = 0; i < ARRAY_SIZE(tacna_core_supplies); i++)
		tacna->core_supplies[i].supply = tacna_core_supplies[i];

	tacna->num_core_supplies = ARRAY_SIZE(tacna_core_supplies);

	ret = devm_regulator_bulk_get(dev, tacna->num_core_supplies,
				      tacna->core_supplies);
	if (ret) {
		dev_err(dev, "Failed to request core supplies: %d\n", ret);
		goto err_devs;
	}

	/*
	 * Don't use devres here because the only device we have to get
	 * against is the MFD device and VDD_D will likely be supplied by
	 * one of its children. Meaning that the regulator will be
	 * destroyed by the time devres calls regulator put.
	 */
	tacna->vdd_d = regulator_get_exclusive(tacna->dev, "VDD_D");
	if (IS_ERR(tacna->vdd_d)) {
		ret = PTR_ERR(tacna->vdd_d);
		dev_err(dev, "Failed to request VDD_D: %d\n", ret);
		goto err_devs;
	}

	tacna->vdd_d_notifier.notifier_call = tacna_vdd_d_notify;
	ret = regulator_register_notifier(tacna->vdd_d,
					  &tacna->vdd_d_notifier);
	if (ret) {
		dev_err(dev, "Failed to register VDD_D notifier %d\n", ret);
		goto err_vdd_d;
	}

	ret = regulator_bulk_enable(tacna->num_core_supplies,
				    tacna->core_supplies);
	if (ret) {
		dev_err(dev, "Failed to enable core supplies: %d\n", ret);
		goto err_notifier;
	}

	ret = regulator_enable(tacna->vdd_d);
	if (ret) {
		dev_err(dev, "Failed to enable VDD_D: %d\n", ret);
		goto err_enable;
	}

	tacna_disable_hard_reset(tacna);

	regcache_cache_only(tacna->regmap, false);

	/*
	 * Verify that this is a chip we know about before we
	 * starting doing any writes to its registers
	 */
	ret = regmap_read(tacna->regmap, TACNA_DEVID, &reg);
	if (ret) {
		dev_err(dev, "Failed to read ID register: %d\n", ret);
		goto err_reset;
	}

	switch (reg & TACNA_DEVID_MASK) {
	case CS47L94_SILICON_ID:
		break;
	default:
		dev_err(tacna->dev, "Unknown device ID: %x\n", reg);
		ret = -EINVAL;
		goto err_reset;
	}

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
	case CS47L94_SILICON_ID:
		if (IS_ENABLED(CONFIG_MFD_CS47L94)) {
			switch (tacna->type) {
			case CS47L94:
			case CS47L95:
				patch_fn = cs47l94_patch;
				mfd_devs = cs47l94_devs;
				n_devs = ARRAY_SIZE(cs47l94_devs);
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

	/* Init 32k clock sourced from MCLK2 */
	ret = regmap_update_bits(tacna->regmap,
			TACNA_CLOCK32K,
			TACNA_CLK_32K_EN_MASK | TACNA_CLK_32K_SRC_MASK,
			TACNA_CLK_32K_EN | TACNA_32K_MCLK2);
	if (ret) {
		dev_err(tacna->dev, "Failed to init 32k clock: %d\n", ret);
		goto err_reset;
	}

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

	return 0;

err_reset:
	tacna_enable_hard_reset(tacna);
	regulator_disable(tacna->vdd_d);
err_enable:
	regulator_bulk_disable(tacna->num_core_supplies, tacna->core_supplies);
err_notifier:
	regulator_unregister_notifier(tacna->vdd_d, &tacna->vdd_d_notifier);
err_vdd_d:
	regulator_put(tacna->vdd_d);
err_devs:
	mfd_remove_devices(dev);
	return ret;
}
EXPORT_SYMBOL_GPL(tacna_dev_init);

int tacna_dev_exit(struct tacna *tacna)
{
	pm_runtime_disable(tacna->dev);

	regulator_disable(tacna->vdd_d);
	regulator_unregister_notifier(tacna->vdd_d, &tacna->vdd_d_notifier);
	regulator_put(tacna->vdd_d);

	mfd_remove_devices(tacna->dev);
	tacna_enable_hard_reset(tacna);

	regulator_bulk_disable(tacna->num_core_supplies, tacna->core_supplies);
	return 0;
}
EXPORT_SYMBOL_GPL(tacna_dev_exit);
