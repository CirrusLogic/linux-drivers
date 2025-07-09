// SPDX-License-Identifier: GPL-2.0
/*
 * CS40L50 Advanced Haptic Driver with waveform memory,
 * integrated DSP, and closed-loop algorithms
 *
 * Copyright 2024 Cirrus Logic, Inc.
 *
 * Author: James Ogletree <james.ogletree@cirrus.com>
 */

#include <linux/i2c.h>
#include <linux/mfd/cs40l50.h>

const struct regmap_config cs40l50_broadcast_regmap = {
	.reg_bits =		32,
	.reg_stride =		4,
	.val_bits =		32,
	.reg_format_endian =	REGMAP_ENDIAN_BIG,
	.val_format_endian =	REGMAP_ENDIAN_BIG,
	.writeable_reg =	cs40l50_broadcast_writeable_reg,
	.readable_reg =		cs40l50_broadcast_readable_reg,
};

static int cs40l50_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct cs40l50 *cs40l50;
	int error;

	cs40l50 = devm_kzalloc(&i2c->dev, sizeof(*cs40l50), GFP_KERNEL);
	if (!cs40l50)
		return -ENOMEM;

	i2c_set_clientdata(i2c, cs40l50);

	cs40l50->dev = &i2c->dev;
	cs40l50->irq = i2c->irq;
	cs40l50->i2c_id = i2c->addr;

	cs40l50->regmap = devm_regmap_init_i2c(i2c, &cs40l50_regmap);
	if (IS_ERR(cs40l50->regmap))
		return dev_err_probe(cs40l50->dev, PTR_ERR(cs40l50->regmap),
				     "Failed to initialize register map\n");

	error = device_property_read_u32(cs40l50->dev, "cirrus,i2c-broadcast-addr",
			&cs40l50->broadcast_addr);
	if (error || cs40l50->broadcast_addr > CS40L50_I2C_BROADCAST_ADDR_MAX ||
			cs40l50->broadcast_addr < CS40L50_I2C_BROADCAST_ADDR_MIN) {
		cs40l50->broadcast_addr = 0;
	} else {
		cs40l50->broadcast_client = devm_i2c_new_dummy_device(cs40l50->dev,
				i2c->adapter, cs40l50->broadcast_addr);
		if (IS_ERR(cs40l50->broadcast_client)) {
			if (PTR_ERR(cs40l50->broadcast_client) == -EBUSY) {
				cs40l50->broadcast_client = NULL;
				goto probe;
			} else {
				return PTR_ERR(cs40l50->broadcast_client);
			}
		}

		cs40l50->broadcast_regmap = devm_regmap_init_i2c(cs40l50->broadcast_client,
				&cs40l50_broadcast_regmap);
		if (IS_ERR(cs40l50->broadcast_regmap)) {
			dev_err(cs40l50->dev, "Failed to allocate broadcast regmap\n");
			return PTR_ERR(cs40l50->broadcast_regmap);
		}

		dev_dbg(cs40l50->dev, "Designated broadcast I2C master\n");
	}
probe:
	return cs40l50_probe(cs40l50);
}

static void cs40l50_i2c_remove(struct i2c_client *i2c)
{
	struct cs40l50 *cs40l50 = i2c_get_clientdata(i2c);

	cs40l50_remove(cs40l50);
}

static const struct i2c_device_id cs40l50_id_i2c[] = {
	{ "cs40l50" },
	{}
};
MODULE_DEVICE_TABLE(i2c, cs40l50_id_i2c);

static const struct of_device_id cs40l50_of_match[] = {
	{ .compatible = "cirrus,cs40l50" },
	{}
};
MODULE_DEVICE_TABLE(of, cs40l50_of_match);

static struct i2c_driver cs40l50_i2c_driver = {
	.driver = {
		.name = "cs40l50",
		.of_match_table = cs40l50_of_match,
		.pm = pm_ptr(&cs40l50_pm_ops),
	},
	.id_table = cs40l50_id_i2c,
	.probe = cs40l50_i2c_probe,
	.remove = cs40l50_i2c_remove,
};
module_i2c_driver(cs40l50_i2c_driver);

MODULE_DESCRIPTION("CS40L50 I2C Driver");
MODULE_AUTHOR("James Ogletree, Cirrus Logic Inc. <james.ogletree@cirrus.com>");
MODULE_LICENSE("GPL");
