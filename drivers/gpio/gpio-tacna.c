/*
 * gpio-tacna.c - GPIO support for Cirrus Logic Tacna codecs
 *
 * Copyright 2017-2018 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/mfd/tacna/core.h>
#include <linux/mfd/tacna/pdata.h>
#include <linux/mfd/tacna/registers.h>

struct tacna_gpio {
	struct tacna *tacna;
	struct gpio_chip gpio_chip;
};

static inline struct tacna_gpio *to_tacna_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct tacna_gpio, gpio_chip);
}

static int tacna_gpio_direction_in(struct gpio_chip *chip, unsigned int offset)
{
	struct tacna_gpio *tacna_gpio = to_tacna_gpio(chip);
	struct tacna *tacna = tacna_gpio->tacna;

	return regmap_update_bits(tacna->regmap,
				  TACNA_GPIO1_CTRL1 + (4 * offset),
				  TACNA_GP1_DIR_MASK, TACNA_GP1_DIR);
}

static int tacna_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct tacna_gpio *tacna_gpio = to_tacna_gpio(chip);
	struct tacna *tacna = tacna_gpio->tacna;
	unsigned int val;
	int ret;

	ret = regmap_read(tacna->regmap, TACNA_GPIO_STATUS1, &val);
	if (ret < 0)
		return ret;

	if (val & (1 << offset))
		return 1;
	else
		return 0;
}

static int tacna_gpio_direction_out(struct gpio_chip *chip,
				    unsigned int offset, int value)
{
	struct tacna_gpio *tacna_gpio = to_tacna_gpio(chip);
	struct tacna *tacna = tacna_gpio->tacna;
	unsigned int level;

	if (value)
		level = TACNA_GP1_LVL;
	else
		level = 0;

	/* Clear DIR and set the level */
	return regmap_update_bits(tacna->regmap,
				  TACNA_GPIO1_CTRL1 + (4 * offset),
				  TACNA_GP1_DIR_MASK | TACNA_GP1_LVL_MASK,
				  level);
}

static int tacna_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct tacna_gpio *tacna_gpio = to_tacna_gpio(chip);
	struct tacna *tacna = tacna_gpio->tacna;
	unsigned int val;
	int ret;

	ret = regmap_read(tacna->regmap, TACNA_GPIO1_CTRL1 + (4 * offset),
			  &val);
	if (ret)
		return ret;

	return !!(val & TACNA_GP1_DIR_MASK);
}

static void tacna_gpio_set(struct gpio_chip *chip, unsigned int offset,
			   int value)
{
	struct tacna_gpio *tacna_gpio = to_tacna_gpio(chip);
	struct tacna *tacna = tacna_gpio->tacna;
	unsigned int level;
	int ret;

	if (value)
		level = TACNA_GP1_LVL;
	else
		level = 0;

	ret = regmap_update_bits(tacna->regmap,
				 TACNA_GPIO1_CTRL1 + (4 * offset),
				 TACNA_GP1_LVL_MASK, level);
	if (ret)
		dev_warn(chip->parent, "Failed to write register 0x%x: %d\n",
			 TACNA_GPIO1_CTRL1 + (4 * offset), ret);
}

static const struct gpio_chip template_chip = {
	.label			= "tacna",
	.owner			= THIS_MODULE,
	.get_direction		= &tacna_gpio_get_direction,
	.direction_input	= &tacna_gpio_direction_in,
	.direction_output	= &tacna_gpio_direction_out,
	.get			= &tacna_gpio_get,
	.set			= &tacna_gpio_set,
	.can_sleep		= true,
};

static int tacna_gpio_probe(struct platform_device *pdev)
{
	struct tacna *tacna = dev_get_drvdata(pdev->dev.parent);
	struct tacna_pdata *pdata = dev_get_platdata(tacna->dev);
	struct tacna_gpio *tacna_gpio;
	int ret;

	tacna_gpio = devm_kzalloc(&pdev->dev, sizeof(*tacna_gpio),
				   GFP_KERNEL);
	if (!tacna_gpio)
		return -ENOMEM;

	tacna_gpio->tacna = tacna;
	tacna_gpio->gpio_chip = template_chip;
	tacna_gpio->gpio_chip.parent = &pdev->dev;

#if defined(CONFIG_OF_GPIO)
	tacna_gpio->gpio_chip.of_node = tacna->dev->of_node;
#endif

	switch (tacna->type) {
	case CS47L96:
	case CS47L97:
		tacna_gpio->gpio_chip.ngpio = CS47L96_NUM_GPIOS;
		break;
	case CS48L31:
	case CS48L32:
	case CS48L33:
		tacna_gpio->gpio_chip.ngpio = CS48L32_NUM_GPIOS;
		break;
	default:
		dev_err(&pdev->dev, "Unknown chip variant %d\n", tacna->type);
		return -EINVAL;
	}

	if (pdata && pdata->gpio_base)
		tacna_gpio->gpio_chip.base = pdata->gpio_base;
	else
		tacna_gpio->gpio_chip.base = -1;

	ret = devm_gpiochip_add_data(&pdev->dev, &tacna_gpio->gpio_chip,
				    tacna_gpio);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n", ret);
		return ret;
	}

	return 0;
}

static struct platform_driver tacna_gpio_driver = {
	.driver.name	= "tacna-gpio",
	.driver.owner	= THIS_MODULE,
	.probe		= &tacna_gpio_probe,
};

module_platform_driver(tacna_gpio_driver);

MODULE_DESCRIPTION("GPIO interface for Cirrus Logic Tacna codecs");
MODULE_AUTHOR("Richard Fitzgerald <rf@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:tacna-gpio");
