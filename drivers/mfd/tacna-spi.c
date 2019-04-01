/*
 * SPI bus interface to Cirrus Logic Tacna codecs
 *
 * Copyright 2016-2018 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/mfd/tacna/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include "tacna.h"

static int tacna_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct tacna *tacna;
	int (*regmap_init_fn)(struct spi_device *spi, struct tacna *);
	unsigned long type;
	int ret;

	if (spi->dev.of_node)
		type = tacna_of_get_type(&spi->dev);
	else
		type = id->driver_data;

	regmap_init_fn = NULL;

	switch (type) {
	case CS47L96:
	case CS47L97:
		if (IS_ENABLED(CONFIG_MFD_CS47L96))
			regmap_init_fn = cs47l96_init_spi_regmap;
		break;
	case CS48L31:
		if (IS_ENABLED(CONFIG_MFD_CS48L31))
			regmap_init_fn = cs48l32_init_spi_regmap;
		break;
	case CS48L32:
		if (IS_ENABLED(CONFIG_MFD_CS48L32))
			regmap_init_fn = cs48l32_init_spi_regmap;
		break;
	case CS48L33:
		if (IS_ENABLED(CONFIG_MFD_CS48L33))
			regmap_init_fn = cs48l32_init_spi_regmap;
		break;
	default:
		dev_err(&spi->dev, "Unknown Tacna SPI device type %ld\n", type);
		return -EINVAL;
	}

	if (!regmap_init_fn) {
		dev_err(&spi->dev,
			"Kernel does not include support for %s\n",
			tacna_name_from_type(type));
		return -EINVAL;
	}

	tacna = devm_kzalloc(&spi->dev, sizeof(*tacna), GFP_KERNEL);
	if (!tacna)
		return -ENOMEM;

	tacna->type = type;

	ret = (*regmap_init_fn)(spi, tacna);
	if (ret) {
		dev_err(&spi->dev,
			"Failed to allocate register map: %d\n", ret);
		return ret;
	}

	tacna->dev = &spi->dev;
	tacna->irq = spi->irq;

	return tacna_dev_init(tacna);
}

static int tacna_spi_remove(struct spi_device *spi)
{
	struct tacna *tacna = spi_get_drvdata(spi);

	tacna_dev_exit(tacna);

	return 0;
}

static const struct spi_device_id tacna_spi_ids[] = {
	{ "cs47l96", CS47L96 },
	{ "cs47l97", CS47L97 },
	{ "cs48l31", CS48L31 },
	{ "cs48l32", CS48L32 },
	{ "cs48l33", CS48L33 },
	{ },
};
MODULE_DEVICE_TABLE(spi, tacna_spi_ids);

static struct spi_driver tacna_spi_driver = {
	.driver = {
		.name	= "tacna",
		.owner	= THIS_MODULE,
		.pm	= &tacna_pm_ops,
		.of_match_table	= of_match_ptr(tacna_of_match),
	},
	.probe		= &tacna_spi_probe,
	.remove		= &tacna_spi_remove,
	.id_table	= tacna_spi_ids,
};

module_spi_driver(tacna_spi_driver);

MODULE_DESCRIPTION("Tacna SPI bus interface");
MODULE_AUTHOR("Richard Fitzgerald <rf@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL v2");
