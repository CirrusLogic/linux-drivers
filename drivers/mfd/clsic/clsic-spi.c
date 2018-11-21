/*
 * clsic-spi.c -- CLSIC SPI bus interface
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * Author: Simon Trimmer <simont@opensource.cirrus.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/mfd/clsic/core.h>

/*
 * This bus driver is the root of the clsic driver tree.
 *
 * The driver communicates with the clsic device through a very limited
 * register map, basically consisting of a TX/RX FIFO and status registers.
 * Messages are sent to firmware running on the device via the FIFO and the
 * firmware performs activities on our behalf as a proxy.
 *
 * For this reason the regmap configuration is extremely limited, all volatile
 * and cacheless.
 */
static bool clsic_spi_regmap_readable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TACNA_DEVID:
	case TACNA_REVID:
	case TACNA_FABID:
	case TACNA_RELID:
	case TACNA_OTPID:
	case TACNA_SFT_RESET:
	case TACNA_IRQ1_EINT_2:
	case TACNA_IRQ1_MASK_2:
	case CLSIC_FW_UPDATE_REG:
	case TACNA_CPF1_RX_WRDATA:
	case TACNA_CPF1_TX_GPR_STATUS1:
	case TACNA_CPF1_TX_RDDATA1:
	case TACNA_CPF1_TX_RDDATA2:
	case CLSIC_FW_BOOT_PROGRESS:
	case CLSIC_FW_BOOT_PANIC:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config clsic_spi_regmap = {
	.name = "clsic",
	.reg_bits = 32,
	.pad_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,

	.max_register = CLSIC_FW_BOOT_PANIC,
	.readable_reg = &clsic_spi_regmap_readable,
	.cache_type = REGCACHE_NONE,
};

/*
 * The probe function starts the process of driver support - it takes the
 * device information passed in and uses it to configure the main clsic
 * structure.
 *
 * The driver from then on uses abstract concepts to interact with the device
 * (the provided regmap, the abstract irq number and a fifo_tx address.
 */
static int clsic_spi_probe(struct spi_device *spi)
{
	struct clsic *clsic;
	int ret;

	clsic = devm_kzalloc(&spi->dev, sizeof(*clsic), GFP_KERNEL);
	if (clsic == NULL)
		return -ENOMEM;

	clsic->regmap = devm_regmap_init_spi(spi, &clsic_spi_regmap);
	if (IS_ERR(clsic->regmap)) {
		ret = PTR_ERR(clsic->regmap);
		dev_err(&spi->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	clsic->dev = &spi->dev;
	clsic->irq = spi->irq;

	/*
	 * Assign the location of the FIFO TX register for this bus type
	 * SPI is using a prefetching serial control port.
	 */
	clsic->fifo_tx = TACNA_CPF1_TX_RDDATA2;

	return clsic_dev_init(clsic);
}

static int clsic_spi_remove(struct spi_device *spi)
{
	struct clsic *clsic = spi_get_drvdata(spi);

	clsic_dev_exit(clsic);

	return 0;
}

static const struct spi_device_id clsic_spi_ids[] = {
	{ "clsic", 1 },
	{ },
};

MODULE_DEVICE_TABLE(spi, clsic_spi_ids);

static struct spi_driver clsic_spi_driver = {
	.driver = {
		.name = "clsic",
		.owner = THIS_MODULE,
		.pm = &clsic_pm_ops,
		.of_match_table = of_match_ptr(clsic_of_match),
	},
	.probe = &clsic_spi_probe,
	.remove = &clsic_spi_remove,
	.id_table = clsic_spi_ids,
};

module_spi_driver(clsic_spi_driver);

MODULE_DESCRIPTION("CLSIC SPI bus interface");
MODULE_AUTHOR("Simon Trimmer <simont@opensource.cirrus.com>");
MODULE_LICENSE("GPL v2");
