/*
 * linux/platform_data/cs40l2x.h -- Platform data for
 * CS40L20/CS40L25/CS40L25A/CS40L25B
 *
 * Copyright 2018 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CS40L2X_H
#define __CS40L2X_H

struct cs40l2x_platform_data {
	unsigned int boost_ind;
	unsigned int boost_cap;
	unsigned int boost_ipk;
	unsigned int boost_ctl;
	unsigned int boost_ovp;
	bool refclk_gpio2;
	unsigned int f0_default;
	unsigned int f0_min;
	unsigned int f0_max;
	unsigned int redc_default;
	unsigned int redc_min;
	unsigned int redc_max;
	bool redc_comp_disable;
	unsigned int gpio1_rise_index;
	unsigned int gpio1_fall_index;
	unsigned int gpio1_fall_timeout;
	unsigned int gpio1_mode;
	unsigned int gpio2_rise_index;
	unsigned int gpio2_fall_index;
	unsigned int gpio3_rise_index;
	unsigned int gpio3_fall_index;
	unsigned int gpio4_rise_index;
	unsigned int gpio4_fall_index;
	bool hiber_enable;
	unsigned int asp_bclk_freq;
	unsigned int asp_width;
	unsigned int asp_timeout;
};

#endif /* __CS40L2X_H */
