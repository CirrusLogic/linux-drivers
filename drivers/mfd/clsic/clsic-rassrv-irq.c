/*
 * clsic-rassrv-irq.c -- CLSIC Register Access Service IRQ support
 *
 * Copyright (C) 2015-2019 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/mfd/core.h>

#include <linux/mfd/clsic/core.h>
#include "clsic-trace.h"
#include <linux/mfd/clsic/message.h>
#include <linux/mfd/clsic/irq.h>
#include <linux/mfd/clsic/rassrv.h>

/*
 * - The RAS service in the device can receive interrupts from DSP2.
 * - These interrupts are exposed to the host as optional notification
 *   messages.
 * - The notifications are not enabled when the device is cold booted.
 * - The CLSIC_RAS_MSG_CR_SET_IRQ_NTY_MODE command is used to control their
 *   state in the device.
 * - There are three modes;
 * - REQ: request an interrupt be enabled (may cause immediate notification)
 * - FLUSH_AND_REQ: clear any pending interrupt and then enable it
 * - CANCEL: disable sending interrupt notifications
 * - When an interrupt is signalled to the RAS service in the device a
 *   notification message is sent to the host and the interrupt is disabled.
 *   The host must enable the interrupt to receive another notification.
 * - CLSIC client drivers need to obtain the IRQ using
 *   clsic_tacna_request_irq() and release it with clsic_tacna_free_irq()
 * - The IRQ is either unbound and disabled, or bound and enabled
 * - When an notification is received, once the IRQ is delivered to the client
 *   the worker thread automatically rearms the device
 */

/*
 * This method allows client drivers to register their IRQ handler with a RAS
 * notification.
 */
int clsic_ras_request_irq(struct clsic_ras_struct *ras, unsigned int irq_id,
			  const char *devname,
			  irq_handler_t irq_handler, void *irq_handler_data)
{
	struct clsic_ras_irq *irq;

	if (irq_id >= CLSIC_RAS_IRQ_COUNT) {
		clsic_err(ras->clsic,
			  "error: Invalid NTY_IRQ index %d context %p",
			  irq_id, ras);
		return -EINVAL;
	}

	mutex_lock(&ras->irq_mutex);
	irq = &ras->irqs[irq_id];
	if (irq->state != CLSIC_RAS_IRQ_STATE_IDLE) {
		clsic_err(ras->clsic, "IRQ already bound (%d %p %d %pF)",
			  irq_id, ras, irq->state, irq_handler);
		mutex_unlock(&ras->irq_mutex);
		return -EBUSY;
	}

	irq->state = CLSIC_RAS_IRQ_STATE_ENABLING;

	irq->irq_handler = irq_handler;
	irq->irq_handler_data = irq_handler_data;

	queue_work(system_unbound_wq, &irq->work_statechange);

	mutex_unlock(&ras->irq_mutex);

	return 0;
}

/* Free RAS notification IRQ and its handler data */
void clsic_ras_free_irq(struct clsic_ras_struct *ras,
			unsigned int irq_id, void *data)
{
	struct clsic_ras_irq *irq = &ras->irqs[irq_id];

	if (irq_id >= CLSIC_RAS_IRQ_COUNT) {
		clsic_err(ras->clsic,
			  "error: Invalid NTY_IRQ index %d context %p",
			  irq_id, ras);
		return;
	}

	flush_work(&irq->work_callback);
	flush_work(&irq->work_statechange);

	mutex_lock(&ras->irq_mutex);
	irq->state = CLSIC_RAS_IRQ_STATE_DISABLING;
	queue_work(system_unbound_wq, &irq->work_statechange);
	mutex_unlock(&ras->irq_mutex);

	flush_work(&irq->work_statechange);

	mutex_lock(&ras->irq_mutex);

	if (irq->state == CLSIC_RAS_IRQ_STATE_DISABLED)
		irq->state = CLSIC_RAS_IRQ_STATE_IDLE;

	mutex_unlock(&ras->irq_mutex);
}

static void clsic_ras_irq_callback(struct work_struct *data)
{
	struct clsic_ras_irq *irq = container_of(data, struct clsic_ras_irq,
						 work_callback);
	struct clsic_ras_struct *ras = irq->ras;
	irqreturn_t ret;

	ret = irq->irq_handler(irq->id, irq->irq_handler_data);

	if (ret != IRQ_HANDLED)
		clsic_err(ras->clsic, "fn: %pF ret: %d", irq->irq_handler, ret);

	/*
	 * The RAS service automatically masks the IRQ as the notification is
	 * sent, trigger the worker to to re-enable the interrupt.
	 */
	queue_work(system_unbound_wq, &irq->work_statechange);
}

/*
 * Send a message to update the state of the RAS IRQ
 */
static void clsic_ras_irq_statechanger(struct work_struct *data)
{
	struct clsic_ras_irq *irq = container_of(data, struct clsic_ras_irq,
						 work_statechange);
	struct clsic_ras_struct *ras = irq->ras;
	struct clsic *clsic = ras->clsic;
	union clsic_ras_msg msg_cmd;
	union clsic_ras_msg msg_rsp;
	int ret;

	memset(&msg_rsp, 0, CLSIC_FIXED_MSG_SZ);

	if (clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
			       ras->service->service_instance,
			       CLSIC_RAS_MSG_CR_SET_IRQ_NTY_MODE))
		return;

	mutex_lock(&ras->irq_mutex);

	msg_cmd.cmd_set_irq_nty_mode.irq_id = irq->id;

	/* based on the current state determine the required mode to set */
	switch (irq->state) {
	case CLSIC_RAS_IRQ_STATE_ENABLING:
	case CLSIC_RAS_IRQ_STATE_ENABLED:
	case CLSIC_RAS_IRQ_STATE_PENDING:
		msg_cmd.cmd_set_irq_nty_mode.mode = CLSIC_RAS_NTY_REQ;
		break;
	case CLSIC_RAS_IRQ_STATE_DISABLING:
	case CLSIC_RAS_IRQ_STATE_IDLE:
	case CLSIC_RAS_IRQ_STATE_DISABLED:
	default:
		msg_cmd.cmd_set_irq_nty_mode.mode = CLSIC_RAS_NTY_CANCEL;
		break;
	}

	ret = clsic_send_msg_sync_pm(clsic,
				  (union t_clsic_generic_message *) &msg_cmd,
				  (union t_clsic_generic_message *) &msg_rsp,
				  CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				  CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN);

	/* When a command succeeds potentially change the state */
	if ((ret == 0) && (msg_rsp.rsp_set_irq_nty_mode.hdr.err == 0))
		switch (irq->state) {
		case CLSIC_RAS_IRQ_STATE_PENDING:
		case CLSIC_RAS_IRQ_STATE_ENABLING:
			irq->state = CLSIC_RAS_IRQ_STATE_ENABLED;
			break;
		case CLSIC_RAS_IRQ_STATE_DISABLING:
			irq->state = CLSIC_RAS_IRQ_STATE_DISABLED;
			break;
		default:
			break;
		}

	trace_clsic_ras_irq_change(msg_cmd.cmd_set_irq_nty_mode.irq_id,
				   msg_cmd.cmd_set_irq_nty_mode.mode,
				   irq->state,
				   ret,
				   msg_rsp.rsp_set_irq_nty_mode.hdr.err);

	mutex_unlock(&ras->irq_mutex);
}

/*
 * This method initialises the RAS IRQs and creates a mapping with the virtual
 * IRQ domain.
 */
int clsic_ras_irq_init(struct clsic_ras_struct *ras)
{
	struct clsic_ras_irq *irq;
	unsigned int i;

	mutex_init(&ras->irq_mutex);
	for (i = 0; i < CLSIC_RAS_IRQ_COUNT; i++) {
		irq = &ras->irqs[i];
		irq->id = CLSIC_RAS_IRQ_DSP2_0 + 0;
		irq->ras = ras;
		irq->state = CLSIC_RAS_IRQ_STATE_IDLE;

		INIT_WORK(&irq->work_callback, clsic_ras_irq_callback);
		INIT_WORK(&irq->work_statechange, clsic_ras_irq_statechanger);
	}

	return 0;
}

/*
 * Extract the ID of the notification and trigger the associated IRQ handler.
 *
 */
void clsic_ras_irq_handler(struct clsic *clsic,
			   struct clsic_ras_struct *ras,
			   union clsic_ras_msg *nty_msg)
{
	unsigned int irq_id;
	struct clsic_ras_irq *irq;
	bool trigger_worker = false;
	bool queued = false;

	irq_id = nty_msg->nty_irq.irq_id;
	if (irq_id >= CLSIC_RAS_IRQ_COUNT) {
		clsic_err(clsic, "Invalid RAS IRQ id: %d\n", irq_id);
		return;
	}
	irq = &ras->irqs[irq_id];

	trace_clsic_ras_irq_event(irq_id);

	mutex_lock(&ras->irq_mutex);
	switch (irq->state) {
	case CLSIC_RAS_IRQ_STATE_ENABLING:
	case CLSIC_RAS_IRQ_STATE_ENABLED:
	case CLSIC_RAS_IRQ_STATE_PENDING:
		irq->state = CLSIC_RAS_IRQ_STATE_PENDING;
		trigger_worker = true;
		break;
	case CLSIC_RAS_IRQ_STATE_IDLE:
	case CLSIC_RAS_IRQ_STATE_DISABLING:
	case CLSIC_RAS_IRQ_STATE_DISABLED:
	default:
		/* Do nothing / not interested */
		break;
	}
	mutex_unlock(&ras->irq_mutex);

	if (!trigger_worker)
		return;

	/*
	 * Invoke the callback function - make sure that this particular
	 * handler call causes a callback by making sure that queue_work adds
	 * to the work queue
	 */
	do {
		queued = queue_work(system_unbound_wq, &irq->work_callback);
		if (!queued)
			flush_work(&irq->work_callback);
	} while (!queued);

}

/*
 * Called when RAS is suspended
 *
 * Notification IRQs are automatically masked on suspend, flush pending
 * worker threads
 */
void clsic_ras_irq_suspend(struct clsic_ras_struct *ras)
{
	int i;

	for (i = 0; i < CLSIC_RAS_IRQ_COUNT; i++) {
		flush_work(&ras->irqs[i].work_callback);
		flush_work(&ras->irqs[i].work_statechange);
	}
}

/*
 * Called when the RAS service is resumed to re-enable unmasked RAS IRQs
 */
void clsic_ras_irq_resume(struct clsic_ras_struct *ras)
{
	int i;

	for (i = 0; i < CLSIC_RAS_IRQ_COUNT; i++)
		if (ras->irqs[i].state != CLSIC_RAS_IRQ_STATE_IDLE)
			queue_work(system_unbound_wq,
				   &ras->irqs[i].work_statechange);
}
