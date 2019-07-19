/*
 * clsic-tacna.h -- CLSIC Tacna IRQ handling
 *
 * Copyright (C) 2015-2019 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef CIRRUS_CLSIC_TACNA_H
#define CIRRUS_CLSIC_TACNA_H

#include <linux/mfd/clsic/rassrv.h>

struct clsic_tacna {
	struct tacna tacna;
	struct clsic_ras_struct *ras;
};

/* Provide a mapping for clients to reference DSP2 IRQs */
#define CLSIC_TACNA_IRQ_DSP2_0		CLSIC_RAS_IRQ_DSP2_0
#define CLSIC_TACNA_IRQ_DSP2_1		CLSIC_RAS_IRQ_DSP2_1
#define CLSIC_TACNA_IRQ_DSP2_2		CLSIC_RAS_IRQ_DSP2_2
#define CLSIC_TACNA_IRQ_DSP2_3		CLSIC_RAS_IRQ_DSP2_3
#define CLSIC_TACNA_IRQ_DSP2_COUNT	CLSIC_RAS_IRQ_DSP2_COUNT

extern int clsic_tacna_request_irq(struct tacna *tacna,
				   unsigned int irq_id, const char *name,
				   irq_handler_t handler, void *data);

extern void clsic_tacna_free_irq(struct tacna *tacna,
				 unsigned int irq_id, void *data);
#endif
