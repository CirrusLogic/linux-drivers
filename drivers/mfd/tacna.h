/*
 * tacna.h  --  MFD internals for Cirrus Logic Tacna codecs
 *
 * Copyright 2016-2017 Cirrus Logic
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef TACNA_MFD_H
#define TACNA_MFD_H

#include <linux/of.h>
#include <linux/pm.h>

struct tacna;

extern const struct dev_pm_ops tacna_pm_ops;
extern const struct of_device_id tacna_of_match[];

int tacna_dev_init(struct tacna *tacna);
int tacna_dev_exit(struct tacna *tacna);

#ifdef CONFIG_OF
unsigned long tacna_of_get_type(struct device *dev);
#else
static inline unsigned long tacna_of_get_type(struct device *dev)
{
	return 0;
}
#endif

int cs47l94_init_spi_regmap(struct spi_device *spi, struct tacna *tacna);
int cs47l94_patch(struct tacna *tacna);

#endif
