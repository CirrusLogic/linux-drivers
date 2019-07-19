/*
 * rassrv.h -- CLSIC Register Access Service
 *
 * Copyright (C) 2015-2019 Cirrus Logic, Inc. and
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
#include <linux/irq_sim.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#define CLSIC_RAS_MAX_DSPS		2
#define CLSIC_RAS_MAX_FASTWRITES	50

/*
 * Each RAS IRQ is individually mapped to a simulated IRQ and has a dedicated
 * work structure.
 */
enum clsic_ras_irq_state {
	CLSIC_RAS_IRQ_STATE_IDLE,
	CLSIC_RAS_IRQ_STATE_ENABLING,
	CLSIC_RAS_IRQ_STATE_ENABLED,
	CLSIC_RAS_IRQ_STATE_DISABLING,
	CLSIC_RAS_IRQ_STATE_DISABLED,
	CLSIC_RAS_IRQ_STATE_PENDING
};

struct clsic_ras_irq {
	unsigned int id;
	int simirq_id;
	enum clsic_ras_irq_state state;

	struct work_struct work;
	struct clsic_ras_struct *ras;
};

struct clsic_ras_struct {
	struct clsic *clsic;
	struct clsic_service *service;
	bool suspended;

	struct regmap *regmap;
	struct regmap *regmap_dsp[CLSIC_RAS_MAX_DSPS];

	struct mutex regmap_mutex;

	bool supports_fastwrites;
	uint8_t fastwrite_counter;

	/*
	 * IRQ mutex guards state and protects against concurrent identical
	 * message ids
	 */
	struct mutex irq_mutex;
	struct clsic_ras_irq irqs[CLSIC_RAS_IRQ_COUNT];
	struct irq_sim irqsim;
};

int clsic_ras_start(struct clsic *clsic, struct clsic_service *handler);

int clsic_ras_reg_write(void *context, unsigned int reg, unsigned int val);
int clsic_ras_reg_read(void *context, unsigned int reg, unsigned int *val);

extern int clsic_ras_request_irq(struct clsic_ras_struct *ras,
				 unsigned int irq_id, const char *name,
				 irq_handler_t handler, void *data);
extern void clsic_ras_free_irq(struct clsic_ras_struct *ras,
			       unsigned int irq_id, void *data);

int clsic_ras_irq_init(struct clsic_ras_struct *ras);
void clsic_ras_irq_suspend(struct clsic_ras_struct *ras);
void clsic_ras_irq_resume(struct clsic_ras_struct *ras);
void clsic_ras_irq_handler(struct clsic *clsic,
			   struct clsic_ras_struct *ras,
			   union clsic_ras_msg *nty_msg);
#endif
