/*
 * Platform data for Cirrus Logic Madera codecs irqchip driver
 *
 * Copyright 2016 Cirrus Logic
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef IRQCHIP_MADERA_PDATA_H
#define IRQCHIP_MADERA_PDATA_H

struct madera_irqchip_pdata {
	/** Mode for primary IRQ (defaults to active low) */
	unsigned int irq_flags;
};

#endif
