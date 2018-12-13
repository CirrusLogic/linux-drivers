/*
 * clsic-tacna.c -- Core MFD support for codec aspect of CLSIC devices
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/mfd/clsic/core.h>
#include <linux/mfd/clsic/rassrv.h>
#include <linux/mfd/core.h>
#include <linux/mfd/tacna/core.h>
#include <linux/module.h>

#define CLSIC_32K_MCLK2		1

static struct mfd_cell clsic_tacna_devs[] = {
	{ .name = "clsic-codec", },
	{
		.name = "tacna-pinctrl",
		.of_compatible = "cirrus,tacna-pinctrl",
	},
};

/*
 * Apply the default state of the pins specified as "active" in the device tree
 * on startup.
 *
 * If there is no "active" state specified then this function will succeed.
 */
static int clsic_tacna_pinctrl_setactive(struct tacna *tacna)
{
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state;
	int ret = 0;

	pinctrl = pinctrl_get(tacna->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		dev_err(tacna->dev, "Failed to get pinctrl: %d\n", ret);
		return ret;
	}

	pinctrl_state = pinctrl_lookup_state(pinctrl, "active");
	if (!IS_ERR(pinctrl_state)) {
		ret = pinctrl_select_state(pinctrl, pinctrl_state);
		if (ret)
			dev_err(tacna->dev,
				"Failed to select pinctrl probe state: %d\n",
				ret);
	}
	pinctrl_put(pinctrl);

	return ret;
}

static int clsic_tacna_probe(struct platform_device *pdev)
{
	struct clsic *clsic = dev_get_drvdata(pdev->dev.parent);
	struct clsic_ras_struct *ras = dev_get_platdata(&pdev->dev);
	struct tacna *tacna;
	int ret = 0;

	/*
	 * Don't try to run this on the emulated platform as it does not have
	 * an interpreter for the resulting RAS messages it cannot simulate the
	 * mixer core.
	 */
	if (clsic->devid == CLSIC_SUPPORTED_ID_EMULATED_CODEC)
		return -EIO;

	tacna = devm_kzalloc(&pdev->dev, sizeof(struct tacna), GFP_KERNEL);
	if (!tacna)
		return -ENOMEM;

	tacna->type = CS48LX50;
	tacna->dev = &pdev->dev;
	/* share of_node with the tacna device */
	tacna->dev->of_node = of_node_get(clsic->dev->of_node);
	tacna->irq = 0;
	tacna->regmap = ras->regmap;
	tacna->dsp_regmap[0] = ras->regmap_dsp[0];
	tacna->dsp_regmap[1] = ras->regmap_dsp[1];

	dev_set_drvdata(tacna->dev, tacna);

	ret = regmap_update_bits(tacna->regmap,
				 TACNA_CLOCK32K,
				 TACNA_CLK_32K_EN_MASK | TACNA_CLK_32K_SRC_MASK,
				 TACNA_CLK_32K_EN | CLSIC_32K_MCLK2);
	if (ret) {
		dev_err(tacna->dev, "Failed to init 32k clock: %d\n", ret);
		return ret;
	}

	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);

	ret = mfd_add_devices(tacna->dev, PLATFORM_DEVID_NONE, clsic_tacna_devs,
			      ARRAY_SIZE(clsic_tacna_devs), NULL, 0, NULL);
	if (ret)
		dev_err(tacna->dev, "Failed to add subdevices: %d\n", ret);
	else
		ret = clsic_tacna_pinctrl_setactive(tacna);

	/*
	 * returning an error from the probe function will cause the device
	 * managed memory to be released.
	 *
	 * So if any part of the driver load fails remove all of the MFD
	 * children.
	 */
	if (ret)
		mfd_remove_devices(&pdev->dev);

	return ret;
}

static int clsic_tacna_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	mfd_remove_devices(&pdev->dev);

	return 0;
}

static struct platform_driver clsic_tacna_core_driver = {
	.driver = {
		.name = "clsic-tacna",
		.owner = THIS_MODULE,
	},
	.probe	= &clsic_tacna_probe,
	.remove	= &clsic_tacna_remove,
};

module_platform_driver(clsic_tacna_core_driver);

MODULE_AUTHOR("Piotr Stankiewicz <piotrs@opensource.wolfsonmicro.com>");
MODULE_DESCRIPTION("CLSIC Tacna MFD core");
MODULE_LICENSE("GPL v2");
