/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * linux/sound/cs35l45.h -- Platform data for CS35L45
 *
 * Copyright 2019 Cirrus Logic, Inc.
 *
 * Author: James Schulman <james.schulman@cirrus.com>
 *
 */

#ifndef __CS35L45_H
#define __CS35L45_H

#define CS35L45_NUM_SUPPLIES 2

struct cs35l45_private {
	struct wm_adsp dsp; /* needs to be first member */
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[CS35L45_NUM_SUPPLIES];
	bool initialized;
	bool halo_booted;
	struct mutex rate_lock;
};

int cs35l45_initialize(struct cs35l45_private *cs35l45);
int cs35l45_probe(struct cs35l45_private *cs35l45);
int cs35l45_remove(struct cs35l45_private *cs35l45);

#endif /* __CS35L45_H */
