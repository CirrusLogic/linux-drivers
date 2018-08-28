/*
 * bootsrv.h -- CLSIC Bootloader Service
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef CIRRUS_CLSIC_BOOTSRV_H
#define CIRRUS_CLSIC_BOOTSRV_H
#include <linux/mfd/clsic/clsicmessagedefines_BLD.h>

int clsic_bootsrv_state_handler(struct clsic *clsic);
int clsic_bootsrv_service_start(struct clsic *clsic,
				struct clsic_service *handler);
#endif
