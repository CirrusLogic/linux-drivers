/*
 * clsic-irq.c -- CLSIC irq handling and notification interface
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include "clsic-trace.h"
#include <linux/mfd/clsic/core.h>
#include <linux/mfd/clsic/irq.h>
#include <linux/mfd/clsic/message.h>

/*
 * This threaded irq handler is called in response to the level based IRQs from
 * the device.
 *
 * If there are more interrupts pending when this handler returns it will be
 * called again.
 */
static irqreturn_t clsic_irq_thread(int irq, void *data)
{
	struct clsic *clsic = data;
	uint32_t reg = 0;
	int ret = regmap_read(clsic->regmap, TACNA_IRQ1_EINT_2, &reg);

	trace_clsic_irq(reg);

	if (ret != 0)
		return IRQ_NONE;

	if ((reg & CLSIC_PVT_TS_SD_RISE_EINT1_MASK) != 0 ||
	    (reg & CLSIC_PVT_TS_WARN_FALL_EINT1_MASK) != 0) {
		clsic_err(clsic, "Device HALTED due to thermal event: 0x%x\n",
			  reg);
		clsic_device_error(clsic, CLSIC_DEVICE_ERROR_LOCKNOTHELD);
		return IRQ_HANDLED;
	}

	if ((reg & TACNA_CPF1_IRQ_EXT_EINT1_MASK) != 0)
		clsic_handle_incoming_messages(clsic);

	if ((reg & TACNA_BOOT_DONE_EINT1_MASK) != 0) {
		regmap_write(clsic->regmap, TACNA_IRQ1_EINT_2,
			     TACNA_BOOT_DONE_EINT1);
		complete(&clsic->bootdone_completion);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_DEBUG_FS
/*
 * This debugfs mechanism triggers handling of incoming messages as would
 * happen if an SCP FIFO or Boot Done interrupt was asserted.
 *
 * It is intended to be used during driver testing.
 *
 * Reading the triggerirq debugfs file reveals the current simulated IRQ state
 *
 * Writing to the triggerirq debugfs file sets the simulated IRQ state
 *
 * Writing 0 clears the simulated IRQ - the timer is stopped synchronously
 * within this function because it may be used by the test framework as a
 * synchronisation point.
 *
 * Writing non-0 asserts the simulated IRQ - the timer will be started within
 * the simirq worker after calling the irq simulation.
 *
 * The effect of the timer mechanism is that the IRQ handler keeps being called
 * the irq if the IRQ remains high, though with a slight delay - the delay
 * gives userspace simulation components a chance to run (and deassert the irq)
 */

#define CLSIC_SIMIRQ_TIMER_PERIOD_MS 300
static void clsic_simirq_timer_cb(unsigned long data)
{
	struct clsic *clsic = (struct clsic *)data;

	schedule_work(&clsic->simirq_work);
}

static void clsic_simirq(struct work_struct *data)
{
	struct clsic *clsic = container_of(data, struct clsic,
					   simirq_work);

	trace_clsic_simirq(clsic->simirq_enabled);

	/* Simulate the irq being masked */
	if (clsic->simirq_enabled)
		clsic_irq_thread(0, clsic);

	/* If the simulated IRQ is still asserted restart the timer */
	if (clsic->simirq_state == CLSIC_SIMIRQ_STATE_ASSERTED)
		mod_timer(&clsic->simirq_timer,
			  jiffies + CLSIC_SIMIRQ_TIMER_PERIOD_MS);
}

static int clsic_simirq_read(void *data, u64 *val)
{
	struct clsic *clsic = data;

	*val = clsic->simirq_state;

	return 0;
}

static int clsic_simirq_write(void *data, u64 val)
{
	struct clsic *clsic = data;

	if (val == 0) {
		clsic->simirq_state = CLSIC_SIMIRQ_STATE_DEASSERTED;
		del_timer_sync(&clsic->simirq_timer);
		trace_clsic_simirq_write_deasserted(0);
	} else {
		clsic->simirq_state = CLSIC_SIMIRQ_STATE_ASSERTED;
		trace_clsic_simirq_write_asserted(0);
	}

	schedule_work(&clsic->simirq_work);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(clsic_simirq_fops, clsic_simirq_read,
			 clsic_simirq_write, "%llu\n");
#endif

int clsic_irq_init(struct clsic *clsic)
{
	struct irq_data *irq_data;
	unsigned long flags = IRQF_ONESHOT;
	int ret;

	irq_data = irq_get_irq_data(clsic->irq);
	if (irq_data == NULL) {
		clsic_err(clsic, "Invalid IRQ: %d\n", clsic->irq);
		return -EINVAL;
	}
	flags |= irqd_get_trigger_type(irq_data);

	ret = request_threaded_irq(clsic->irq, NULL, clsic_irq_thread,
				   flags, "clsic", clsic);
	if (ret != 0) {
		clsic_err(clsic, "Failed to request primary IRQ %d: %d\n",
			  clsic->irq, ret);
	}

#ifdef CONFIG_DEBUG_FS
	clsic->simirq_state = CLSIC_SIMIRQ_STATE_DEASSERTED;

	init_timer(&clsic->simirq_timer);
	clsic->simirq_timer.function = &clsic_simirq_timer_cb;
	clsic->simirq_timer.data = (unsigned long) clsic;

	INIT_WORK(&clsic->simirq_work, clsic_simirq);
	debugfs_create_file("triggerirq", 0220, clsic->debugfs_root, clsic,
			    &clsic_simirq_fops);
#endif

	return ret;
}

void clsic_irq_exit(struct clsic *clsic)
{
	clsic_irq_disable(clsic);

#ifdef CONFIG_DEBUG_FS
	clsic->simirq_state = CLSIC_SIMIRQ_STATE_DEASSERTED;
	del_timer_sync(&clsic->simirq_timer);
	cancel_work_sync(&clsic->simirq_work);
#endif

	free_irq(clsic->irq, clsic);
}
