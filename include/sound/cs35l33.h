/*
 * linux/sound/cs35l33.h -- Platform data for CS35l33
 *
 * Copyright (c) 2013 Cirrus Logic Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CS35L33_H
#define __CS35L33_H

struct cs35l33_pdata {

	/* GPIO for !RST */
	int gpio_nreset;

	/* GPIO for IRQ */
	int gpio_irq;

	/* Boost Controller Voltage Setting */
	unsigned int boost_ctl;

	/* Boost Controller Peak Current */
	unsigned int boost_ipk;

	/* Gain Change Zero Cross */
	unsigned int gain_zc;

	/* Amplifier Drive Select */
	unsigned int amp_drv_sel;
};

#endif /* __CS35L33_H */
