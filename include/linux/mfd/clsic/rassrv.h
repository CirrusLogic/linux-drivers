/*
 * rassrv.h -- CLSIC Register Access Service
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef CIRRUS_CLSIC_RASSRV_H
#define CIRRUS_CLSIC_RASSRV_H

#include <linux/mfd/clsic/clsicmessagedefines_RAS.h>
#include <linux/mfd/tacna/pdata.h>
#define CLSIC_RAS_MAX_DSPS		2
#define CLSIC_RAS_MAX_FASTWRITES	50

struct clsic_ras_struct {
	struct tacna_pdata pdata;
	struct clsic *clsic;
	struct clsic_service *service;
	bool suspended;

	struct regmap *regmap;
	struct regmap *regmap_dsp[CLSIC_RAS_MAX_DSPS];

	struct mutex regmap_mutex;

	bool supports_fastwrites;
	uint8_t fastwrite_counter;
};

int clsic_ras_start(struct clsic *clsic, struct clsic_service *handler);

int clsic_ras_reg_write(void *context, unsigned int reg, unsigned int val);
int clsic_ras_reg_read(void *context, unsigned int reg, unsigned int *val);

#endif
