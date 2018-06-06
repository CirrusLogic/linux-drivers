/*
 * Platform data for Cirrus Logic Tacna codecs irqchip driver
 *
 * Copyright 2017 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef IRQCHIP_TACNA_PDATA_H
#define IRQCHIP_TACNA_PDATA_H

/**
 * struct tacna_irqchip_pdata - Configuration for Tacna irqchip driver
 *
 * @irq_flags: Mode for primary IRQ (defaults to active low)
 */
struct tacna_irqchip_pdata {
	unsigned int irq_flags;
};

#endif
