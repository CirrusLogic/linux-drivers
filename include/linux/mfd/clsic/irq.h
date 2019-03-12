/*
 * irq.h -- CLSIC service handler notification interface
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef CIRRUS_CLSIC_IRQ_H
#define CIRRUS_CLSIC_IRQ_H

#include <linux/interrupt.h>

/*
 * Used throughout the message and interrupt handling code; indicates whether a
 * function fully handled a message.
 */
#define CLSIC_HANDLED			0
#define CLSIC_UNHANDLED			1

static inline void clsic_irq_messaging_enable(struct clsic *clsic)
{
	/* Unmask the control port fifo interrupt that is used for messaging */
	regmap_update_bits(clsic->regmap, TACNA_IRQ1_MASK_2,
			   TACNA_CPF1_IRQ_EXT_EINT1_MASK, 0);
}

static inline void clsic_irq_enable(struct clsic *clsic)
{
	enable_irq(clsic->irq);

#ifdef CONFIG_DEBUG_FS
	clsic->simirq_enabled = true;
#endif
}

static inline void clsic_irq_disable(struct clsic *clsic)
{
	disable_irq_nosync(clsic->irq);
#ifdef CONFIG_DEBUG_FS
	clsic->simirq_enabled = false;
#endif
}

int clsic_irq_init(struct clsic *clsic);
void clsic_irq_exit(struct clsic *clsic);
#endif
