// SPDX-License-Identifier: GPL-2.0
//
// cs40l26-spi.c -- CS40L26 SPI Driver
//
// Copyright 2020 Cirrus Logic, Inc.
//
// Author: Fred Treven <fred.treven@cirrus.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation.

#include <linux/mfd/cs40l26.h>

static struct regmap_config cs40l26_regmap_spi = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = CS40L26_LASTREG,
	.num_reg_defaults = 0,
	.precious_reg = cs40l26_precious_reg,
	.readable_reg = cs40l26_readable_reg,
	.volatile_reg = cs40l26_volatile_reg,
	.cache_type = REGCACHE_NONE,
};

static const struct spi_device_id cs40l26_id_spi[] = {
	{"cs40l26", 0},
	{}
};

MODULE_DEVICE_TABLE(spi, cs40l26_id_spi);

static int cs40l26_spi_probe(struct spi_device *spi)
{
	int ret;
	struct cs40l26_private *cs40l26;
	struct device *dev = &spi->dev;
	struct cs40l26_platform_data *pdata = dev_get_platdata(&spi->dev);

	cs40l26 = devm_kzalloc(dev, sizeof(struct cs40l26_private), GFP_KERNEL);
	if (!cs40l26)
		return -ENOMEM;

	spi_set_drvdata(spi, cs40l26);

	cs40l26->regmap = devm_regmap_init_spi(spi, &cs40l26_regmap_spi);
	if (IS_ERR(cs40l26->regmap)) {
		ret = PTR_ERR(cs40l26->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	cs40l26->dev = dev;
	cs40l26->irq = spi->irq;

	return cs40l26_probe(cs40l26, pdata);
}

static int cs40l26_spi_remove(struct spi_device *spi)
{
	struct cs40l26_private *cs40l26 = spi_get_drvdata(spi);

	return cs40l26_remove(cs40l26);
}

static const struct of_device_id cs40l26_of_match[] = {
	{ .compatible = "cirrus,cs40l26" },
	{ }
};

MODULE_DEVICE_TABLE(of, cs40l26_of_match);

static struct spi_driver cs40l26_spi_driver = {
	.driver = {
		.name = "cs40l26",
		.of_match_table = cs40l26_of_match,
	},

	.id_table = cs40l26_id_spi,
	.probe = cs40l26_spi_probe,
	.remove = cs40l26_spi_remove,
};

module_spi_driver(cs40l26_spi_driver);

MODULE_DESCRIPTION("CS40L26 SPI Driver");
MODULE_AUTHOR("Fred Treven, Cirrus Logic Inc. <fred.treven@cirrus.com>");
MODULE_LICENSE("GPL");
