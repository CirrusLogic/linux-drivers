/*
 * linux/sound/cs35l35.h -- Platform data for CS35l35
 *
 * Copyright (c) 2016 Cirrus Logic Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CS35L35_H
#define __CS35L35_H

struct cs35l35_platform_data {

	/* GPIO RESET */
	int gpio_nreset;
};

#endif /* __CS35L35_H */
