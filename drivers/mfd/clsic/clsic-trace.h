/*
 * clsic-trace.h -- CLSIC tracepoints header file
 *
 * Copyright (C) 2015-2019 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM clsic

#if !defined(CLSIC_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define CLSIC_TRACE_H

#include <linux/mfd/clsic/core.h>
#include <linux/mfd/clsic/message.h>
#include <linux/tracepoint.h>
#include <uapi/sound/compress_offload.h>

TRACE_EVENT(clsic_fifo_readmessage,
	TP_PROTO(struct clsic_message *msg),
	TP_ARGS(msg),
	TP_STRUCT__entry(
			__field(void *, msg_p)
			__array(uint8_t, fsm, CLSIC_FIXED_MSG_SZ)
			__field(uint8_t, state)
			__field(uint8_t, cran)
			__field(uint8_t, bulk)
			__field(uint8_t, servinst)
			__field(uint8_t, msgid)
			),
	TP_fast_assign(
			__entry->msg_p = msg;
			memcpy(__entry->fsm, msg->fsm.raw, CLSIC_FIXED_MSG_SZ);
			__entry->state = msg->state;
			__entry->bulk = clsic_get_bulkbit(msg);
			__entry->servinst = clsic_get_servinst(msg);
			__entry->msgid = clsic_get_messageid(msg);
			__entry->cran = clsic_cran_to_char(
						    clsic_get_cran_frommsg(msg))
		),
	TP_printk(
			" Msg %p (%d): %02x %02x [%d %c %c %d] %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
			__entry->msg_p,
			__entry->state,
			__entry->fsm[CLSIC_FSM0],
			__entry->fsm[CLSIC_FSM1],
			__entry->servinst,
			__entry->bulk ? 'b' : 'f',
			__entry->cran,
			__entry->msgid,
			__entry->fsm[CLSIC_FSM2],
			__entry->fsm[CLSIC_FSM3],
			__entry->fsm[CLSIC_FSM4],
			__entry->fsm[CLSIC_FSM5],
			__entry->fsm[CLSIC_FSM6],
			__entry->fsm[CLSIC_FSM7],
			__entry->fsm[CLSIC_FSM8],
			__entry->fsm[CLSIC_FSM9],
			__entry->fsm[CLSIC_FSM10],
			__entry->fsm[CLSIC_FSM11])
);

TRACE_EVENT(clsic_fifo_writemessage,
	TP_PROTO(struct clsic_message *msg),
	TP_ARGS(msg),
	TP_STRUCT__entry(
			__field(void *, msg_p)
			__array(uint8_t, fsm, CLSIC_FIXED_MSG_SZ)
			__field(uint8_t, state)
			__field(uint8_t, cran)
			__field(uint8_t, bulk)
			__field(uint8_t, servinst)
			__field(uint8_t, msgid)
			),
	TP_fast_assign(
			__entry->msg_p = msg;
			memcpy(__entry->fsm, msg->fsm.raw, CLSIC_FIXED_MSG_SZ);
			__entry->state = msg->state;
			__entry->bulk = clsic_get_bulkbit(msg);
			__entry->servinst = clsic_get_servinst(msg);
			__entry->msgid = clsic_get_messageid(msg);
			__entry->cran = clsic_cran_to_char(
						    clsic_get_cran_frommsg(msg))
		),
	TP_printk(
			"Msg %p (%d): %02x %02x [%d %c %c %d] %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
			__entry->msg_p,
			__entry->state,
			__entry->fsm[CLSIC_FSM0],
			__entry->fsm[CLSIC_FSM1],
			__entry->servinst,
			__entry->bulk ? 'b' : 'f',
			__entry->cran,
			__entry->msgid,
			__entry->fsm[CLSIC_FSM2],
			__entry->fsm[CLSIC_FSM3],
			__entry->fsm[CLSIC_FSM4],
			__entry->fsm[CLSIC_FSM5],
			__entry->fsm[CLSIC_FSM6],
			__entry->fsm[CLSIC_FSM7],
			__entry->fsm[CLSIC_FSM8],
			__entry->fsm[CLSIC_FSM9],
			__entry->fsm[CLSIC_FSM10],
			__entry->fsm[CLSIC_FSM11])
);

TRACE_EVENT(clsic_fifo_readbulk,
	TP_PROTO(uint32_t len),
	TP_ARGS(len),
	TP_STRUCT__entry(
			__field(uint32_t, len)
			),
	TP_fast_assign(
			__entry->len = len
		),
	TP_printk(
			"    Transferring bulk read %d",
			__entry->len
			)
);

TRACE_EVENT(clsic_fifo_writebulk,
	TP_PROTO(uint32_t len, uint32_t totallen),

	TP_ARGS(len, totallen),
	TP_STRUCT__entry(
			__field(uint32_t, len)
			__field(uint32_t, totallen)
			),
	TP_fast_assign(
			__entry->len = len;
			__entry->totallen = totallen
		),
	TP_printk(
			"   Transferring bulk write %d (%d remaining)",
			__entry->len,
			__entry->totallen
			)
);

TRACE_EVENT(clsic_msg_statechange,
	TP_PROTO(struct clsic_message *msg),
	TP_ARGS(msg),
	TP_STRUCT__entry(
			__field(void *, msg_p)
			__field(uint8_t, state)
			__field(uint8_t, cran)
			__field(uint8_t, bulk)
			__field(uint8_t, servinst)
			__field(uint8_t, msgid)
			),
	TP_fast_assign(
			__entry->msg_p = msg;
			__entry->state = msg->state;
			__entry->bulk = clsic_get_bulkbit(msg);
			__entry->servinst = clsic_get_servinst(msg);
			__entry->msgid = clsic_get_messageid(msg);
			__entry->cran = clsic_cran_to_char(
						    clsic_get_cran_frommsg(msg))
		),
	TP_printk(
			"  Msg %p (%d): [%d %c %c %d] %s",
			__entry->msg_p,
			__entry->state,
			__entry->servinst,
			__entry->bulk ? 'b' : 'f',
			__entry->cran,
			__entry->msgid,
			clsic_message_state_to_string(__entry->state)
			)
);

TRACE_EVENT(clsic_statechange,

	TP_PROTO(enum clsic_states state_from,
		 enum clsic_states state_to),
	TP_ARGS(state_from, state_to),
	TP_STRUCT__entry(
			__field(uint8_t, state_from)
			__field(uint8_t, state_to)
			),
	TP_fast_assign(
			__entry->state_from = state_from;
			__entry->state_to = state_to
		),
	TP_printk(
			"0x%x (%s)-> 0x%x (%s)",
			__entry->state_from,
			clsic_state_to_string(__entry->state_from),
			__entry->state_to,
			clsic_state_to_string(__entry->state_to)
			)
);

DECLARE_EVENT_CLASS(clsic_generic,
	TP_PROTO(uint8_t dummy),
	TP_ARGS(dummy),
	TP_STRUCT__entry(
			__field(uint8_t, dummy)
			),
	TP_fast_assign(
		),
	TP_printk("%s", " ")
);

TRACE_EVENT(clsic_dev_panic,
	TP_PROTO(enum clsic_states state),
	TP_ARGS(state),
	TP_STRUCT__entry(
			__field(uint8_t, state)
			),
	TP_fast_assign(
			__entry->state = state
		),
	TP_printk(
			"was in state %s (0x%x)",
			clsic_state_to_string(__entry->state),
			__entry->state
			)
);

TRACE_EVENT(clsic_ras_simplewrite,
	TP_PROTO(uint32_t address, uint32_t value, int ret, uint8_t err,
		 u64 elapsed_us),
	TP_ARGS(address, value, ret, err, elapsed_us),
	TP_STRUCT__entry(
			__field(uint32_t, address)
			__field(uint32_t, value)
			__field(int, ret)
			__field(uint8_t, err)
			__field(u64, elapsed_us)
			),
	TP_fast_assign(
			__entry->address = address;
			__entry->value = value;
			__entry->ret = ret;
			__entry->err = err;
			__entry->elapsed_us = elapsed_us;
		),
	TP_printk(
			"  addr: 0x%x val: 0x%x ret: %d (err: %d) elapsed: %llu us",
			__entry->address,
			__entry->value,
			__entry->ret,
			__entry->err,
			__entry->elapsed_us
			)
);

TRACE_EVENT(clsic_ras_simpleread,
	TP_PROTO(uint32_t address, uint32_t value, int ret, uint8_t err,
		 u64 elapsed_us),
	TP_ARGS(address, value, ret, err, elapsed_us),
	TP_STRUCT__entry(
			__field(uint32_t, address)
			__field(uint32_t, value)
			__field(int, ret)
			__field(uint8_t, err)
			__field(u64, elapsed_us)
			),
	TP_fast_assign(
			__entry->address = address;
			__entry->value = value;
			__entry->ret = ret;
			__entry->err = err;
			__entry->elapsed_us = elapsed_us;
		),
	TP_printk(
			" addr: 0x%x val: 0x%x ret: %d (err: %d) elapsed: %llu us",
			__entry->address,
			__entry->value,
			__entry->ret,
			__entry->err,
			__entry->elapsed_us
			)
);

TRACE_EVENT(clsic_ras_fastread,
	TP_PROTO(uint32_t address, uint32_t value, int ret, u64 elapsed_us),
	TP_ARGS(address, value, ret, elapsed_us),
	TP_STRUCT__entry(
			__field(uint32_t, address)
			__field(uint32_t, value)
			__field(int, ret)
			__field(u64, elapsed_us)
			),
	TP_fast_assign(
			__entry->address = address;
			__entry->value = value;
			__entry->ret = ret;
			__entry->elapsed_us = elapsed_us;
		),
	TP_printk(
			"   addr: 0x%x val: 0x%x ret: %d elapsed: %llu us",
			__entry->address,
			__entry->value,
			__entry->ret,
			__entry->elapsed_us
			)
);

TRACE_EVENT(clsic_ras_bulkwrite,
	TP_PROTO(uint32_t address, uint32_t count, int ret, uint8_t err,
		 ssize_t progress, ssize_t total, u64 elapsed_us),
	TP_ARGS(address, count, ret, err, progress, total, elapsed_us),
	TP_STRUCT__entry(
			__field(uint32_t, address)
			__field(uint32_t, count)
			__field(int, ret)
			__field(uint8_t, err)
			__field(ssize_t, progress)
			__field(ssize_t, total)
			__field(u64, elapsed_us)
			),
	TP_fast_assign(
			__entry->address = address;
			__entry->count = count;
			__entry->ret = ret;
			__entry->err = err;
			__entry->progress = progress;
			__entry->total = total;
			__entry->elapsed_us = elapsed_us;
		),
	TP_printk(
			"   addr: 0x%x count: %d ret: %d (err: %d) transfer: %zu of %zu total elapsed time: %llu us",
			__entry->address,
			__entry->count,
			__entry->ret,
			__entry->err,
			__entry->progress,
			__entry->total,
			__entry->elapsed_us
			)
);

TRACE_EVENT(clsic_ras_bulkread,
	TP_PROTO(uint32_t address, uint32_t count, int ret, uint8_t err,
		 ssize_t progress, ssize_t total, u64 elapsed_us),
	TP_ARGS(address, count, ret, err, progress, total, elapsed_us),
	TP_STRUCT__entry(
			__field(uint32_t, address)
			__field(uint32_t, count)
			__field(int, ret)
			__field(uint8_t, err)
			__field(ssize_t, progress)
			__field(ssize_t, total)
			__field(u64, elapsed_us)
			),
	TP_fast_assign(
			__entry->address = address;
			__entry->count = count;
			__entry->ret = ret;
			__entry->err = err;
			__entry->progress = progress;
			__entry->total = total;
			__entry->elapsed_us = elapsed_us;
		),
	TP_printk(
			"   addr: 0x%x count: %d ret: %d (err: %d) transfer: %zu of %zu total elapsed time: %llu us",
			__entry->address,
			__entry->count,
			__entry->ret,
			__entry->err,
			__entry->progress,
			__entry->total,
			__entry->elapsed_us
			)
);

TRACE_EVENT(clsic_ras_pm_handler,
	TP_PROTO(int pm_event, u64 elapsed_us),
	TP_ARGS(pm_event, elapsed_us),
	TP_STRUCT__entry(
			__field(int, pm_event)
			__field(u64, elapsed_us)
			),
	TP_fast_assign(
			__entry->pm_event = pm_event;
			__entry->elapsed_us = elapsed_us;
		),
	TP_printk(
			"    pm event: %d elapsed: %llu us",
			__entry->pm_event,
			__entry->elapsed_us
			)
);

TRACE_EVENT(clsic_ras_fastwrite,
	TP_PROTO(uint32_t address, uint32_t value, int ret, uint8_t counter,
		 u64 elapsed_us),
	TP_ARGS(address, value, ret, counter, elapsed_us),
	TP_STRUCT__entry(
			__field(uint32_t, address)
			__field(uint32_t, value)
			__field(int, ret)
			__field(uint8_t, counter)
			__field(u64, elapsed_us)
			),
	TP_fast_assign(
			__entry->address = address;
			__entry->value = value;
			__entry->ret = ret;
			__entry->counter = counter;
			__entry->elapsed_us = elapsed_us;
		),
	TP_printk(
			"  addr: 0x%x val: 0x%x ret: %d (counter: %d) elapsed: %llu us",
			__entry->address,
			__entry->value,
			__entry->ret,
			__entry->counter,
			__entry->elapsed_us
			)
);

TRACE_EVENT(clsic_ras_irq_change,
	TP_PROTO(uint32_t irq_id, uint32_t mode, uint32_t state, int ret,
		 uint8_t err),
	TP_ARGS(irq_id, mode, state, ret, err),
	TP_STRUCT__entry(
			__field(uint32_t, irq_id)
			__field(uint32_t, mode)
			__field(uint32_t, state)
			__field(int, ret)
			__field(uint8_t, err)
			),
	TP_fast_assign(
			__entry->irq_id = irq_id;
			__entry->mode = mode;
			__entry->state = state;
			__entry->ret = ret;
			__entry->err = err;
		),
	TP_printk(
			"  id: %d mode: %d state: %d ret: %d (err: %d)",
			__entry->irq_id,
			__entry->mode,
			__entry->state,
			__entry->ret,
			__entry->err
			)
);

TRACE_EVENT(clsic_ras_irq_event,
	TP_PROTO(uint32_t irq_id),
	TP_ARGS(irq_id),
	TP_STRUCT__entry(
			__field(uint32_t, irq_id)
			),
	TP_fast_assign(
			__entry->irq_id = irq_id;
		),
	TP_printk(
			"  id: %d",
			__entry->irq_id
			)
);

TRACE_EVENT(clsic_pm,
	TP_PROTO(int event),
	TP_ARGS(event),
	TP_STRUCT__entry(
			__field(int, event)
			),
	TP_fast_assign(
			__entry->event = event;
		),
	TP_printk(
			"%s (%d)",
			clsic_pm_rpm_to_string(__entry->event),
			__entry->event
			)
);

TRACE_EVENT(clsic_msgproc_shutdown_schedule,
	TP_PROTO(int ret),
	TP_ARGS(ret),
	TP_STRUCT__entry(
			__field(int, ret)
			),
	TP_fast_assign(
			__entry->ret = ret;
		),
	TP_printk(
			"ret = %d",
			__entry->ret
			)
);

TRACE_EVENT(clsic_msgproc_shutdown_cancel,
	TP_PROTO(int sync, int ret),
	TP_ARGS(sync, ret),
	TP_STRUCT__entry(
			__field(int, sync)
			__field(int, ret)
			),
	TP_fast_assign(
			__entry->sync = sync;
			__entry->ret = ret;
		),
	TP_printk(
			"sync: %d, ret = %d",
			__entry->sync,
			__entry->ret
			)
);

TRACE_EVENT(clsic_alg_simple_writeregister,
	TP_PROTO(uint32_t address, uint32_t value, int ret, uint8_t err),
	TP_ARGS(address, value, ret, err),
	TP_STRUCT__entry(
			__field(uint32_t, address)
			__field(uint32_t, value)
			__field(int, ret)
			__field(uint8_t, err)
			),
	TP_fast_assign(
			__entry->address = address;
			__entry->value = value;
			__entry->ret = ret;
			__entry->err = err;
		),
	TP_printk(
			"  addr: 0x%x val: 0x%x ret: %d (err: %d)",
			__entry->address,
			__entry->value,
			__entry->ret,
			__entry->err
			)
);

TRACE_EVENT(clsic_alg_simple_readregister,
	TP_PROTO(uint32_t address, uint32_t value, int ret, uint8_t err),
	TP_ARGS(address, value, ret, err),
	TP_STRUCT__entry(
			__field(uint32_t, address)
			__field(uint32_t, value)
			__field(int, ret)
			__field(uint8_t, err)
			),
	TP_fast_assign(
			__entry->address = address;
			__entry->value = value;
			__entry->ret = ret;
			__entry->err = err;
		),
	TP_printk(
			"   addr: 0x%x val: 0x%x ret: %d (err: %d)",
			__entry->address,
			__entry->value,
			__entry->ret,
			__entry->err
			)
);

TRACE_EVENT(clsic_alg_write,
	TP_PROTO(uint32_t address, uint32_t count, int ret, uint8_t err),
	TP_ARGS(address, count, ret, err),
	TP_STRUCT__entry(
			__field(uint32_t, address)
			__field(uint32_t, count)
			__field(int, ret)
			__field(uint8_t, err)
			),
	TP_fast_assign(
			__entry->address = address;
			__entry->count = count;
			__entry->ret = ret;
			__entry->err = err;
		),
	TP_printk(
			"    addr: 0x%x count: %d ret: %d (err: %d)",
			__entry->address,
			__entry->count,
			__entry->ret,
			__entry->err
			)
);

TRACE_EVENT(clsic_alg_read,
	TP_PROTO(uint32_t address, uint32_t count, int ret, uint8_t err),
	TP_ARGS(address, count, ret, err),
	TP_STRUCT__entry(
			__field(uint32_t, address)
			__field(uint32_t, count)
			__field(int, ret)
			__field(uint8_t, err)
			),
	TP_fast_assign(
			__entry->address = address;
			__entry->count = count;
			__entry->ret = ret;
			__entry->err = err;
		),
	TP_printk(
			"    addr: 0x%x count: %d ret: %d (err: %d)",
			__entry->address,
			__entry->count,
			__entry->ret,
			__entry->err
			)
);

TRACE_EVENT(clsic_alg_custom_msg,
	TP_PROTO(uint8_t msgid, ssize_t len, int ret, uint8_t err),
	TP_ARGS(msgid, len, ret, err),
	TP_STRUCT__entry(
			__field(uint32_t, msgid)
			__field(ssize_t, len)
			__field(int, ret)
			__field(uint8_t, err)
			),
	TP_fast_assign(
			__entry->msgid = msgid;
			__entry->len = len;
			__entry->ret = ret;
			__entry->err = err;
		),
	TP_printk(
			"    msgid: 0x%x msg len: %zu ret: %d (err: %d)",
			__entry->msgid,
			__entry->len,
			__entry->ret,
			__entry->err
			)
);

TRACE_EVENT(clsic_simirq,
	TP_PROTO(int simirq_enabled),
	TP_ARGS(simirq_enabled),
	TP_STRUCT__entry(
		__field(int, simirq_enabled)
	),
	TP_fast_assign(
		__entry->simirq_enabled = simirq_enabled;
	),
	TP_printk("%d", __entry->simirq_enabled)
);

DEFINE_EVENT(clsic_generic, clsic_simirq_write_asserted,
	TP_PROTO(uint8_t dummy),
	TP_ARGS(dummy)
);

DEFINE_EVENT(clsic_generic, clsic_simirq_write_deasserted,
	TP_PROTO(uint8_t dummy),
	TP_ARGS(dummy)
);

TRACE_EVENT(clsic_irq,
	TP_PROTO(uint32_t reg),
	TP_ARGS(reg),
	TP_STRUCT__entry(
		__field(uint32_t, reg)
	),
	TP_fast_assign(
		__entry->reg = reg;
	),
	TP_printk("0x%x", __entry->reg)
);

TRACE_EVENT(clsic_dev_init,
	TP_PROTO(uint8_t bootonload),
	TP_ARGS(bootonload),
	TP_STRUCT__entry(
		__field(uint8_t, bootonload)
	),
	TP_fast_assign(
		__entry->bootonload = bootonload;
	),
	TP_printk("bootonload: %d", __entry->bootonload)
);

TRACE_EVENT(clsic_dev_exit,
	TP_PROTO(enum clsic_states state, uint32_t service_states),
	TP_ARGS(state, service_states),
	TP_STRUCT__entry(
		__field(uint8_t, state)
		__field(uint32_t, service_states)
	),
	TP_fast_assign(
		__entry->state = state;
		__entry->service_states = service_states;
	),
	TP_printk("state: %s (%d) service_states: %d",
		  clsic_state_to_string(__entry->state),
		  __entry->state, __entry->service_states)
);

TRACE_EVENT(clsic_maintenance,
	TP_PROTO(enum clsic_states state,
		 enum clsic_blrequests blrequest,
		 uint32_t service_states
	),
	TP_ARGS(state, blrequest, service_states),
	TP_STRUCT__entry(
		__field(uint8_t, state)
		__field(uint8_t, blrequest)
		__field(uint32_t, service_states)
	),
	TP_fast_assign(
		__entry->state = state;
		__entry->blrequest = blrequest;
		__entry->service_states = service_states;
	),
	TP_printk("state: %s (%d) blrequest: %d service_states: %d",
		  clsic_state_to_string(__entry->state),
		  __entry->state, __entry->blrequest, __entry->service_states)
);

TRACE_EVENT(clsic_alg_handle_n_irq,
	TP_PROTO(uint32_t event, int ret),
	TP_ARGS(event, ret),
	TP_STRUCT__entry(
			__field(uint32_t, event)
			__field(int, ret)
			),
	TP_fast_assign(
			__entry->event = event;
			__entry->ret = ret;
		),
	TP_printk("event: %u; ret: %d", __entry->event, __entry->ret)
);

TRACE_EVENT(clsic_alg_set_irq_notify_mode,
	TP_PROTO(uint32_t irq_id, uint32_t irq_mode, int ret, uint8_t err),
	TP_ARGS(irq_id, irq_mode, ret, err),
	TP_STRUCT__entry(
			__field(uint32_t, irq_id)
			__field(uint32_t, irq_mode)
			__field(uint32_t, ret)
			__field(uint8_t, err)
			),
	TP_fast_assign(
			__entry->irq_id = irq_id;
			__entry->irq_mode = irq_mode;
			__entry->ret = ret;
			__entry->err = err;
		),
	TP_printk(
			"   irq_id: %d irq_mode: %d  ret: %d (err: %d)",
			__entry->irq_id,
			__entry->irq_mode,
			__entry->ret,
			__entry->err
			)
);

TRACE_EVENT(clsic_alg_compr_stream_open,
	TP_PROTO(enum snd_compr_direction  dir, int ret),
	TP_ARGS(dir, ret),
	TP_STRUCT__entry(
			__field(enum snd_compr_direction, dir)
			__field(int, ret)
			),
	TP_fast_assign(
			__entry->dir = dir;
			__entry->ret = ret;
		),
	TP_printk("direction: %d; ret: %d",
		  __entry->dir,
		  __entry->ret)
);

TRACE_EVENT(clsic_alg_compr_stream_free,
	TP_PROTO(enum snd_compr_direction dir, int ret),
	TP_ARGS(dir, ret),
	TP_STRUCT__entry(
			__field(enum snd_compr_direction, dir)
			__field(int, ret)
			),
	TP_fast_assign(
			__entry->dir = dir;
			__entry->ret = ret;
		),
	TP_printk("direction: %d; ret: %d",
		  __entry->dir,
		  __entry->ret)
);

TRACE_EVENT(clsic_alg_compr_stream_set_params,
	TP_PROTO(struct snd_compr_params *params, int ret),
	TP_ARGS(params, ret),
	TP_STRUCT__entry(
			__field(u32, codec_id)
			__field(u32, ch_in)
			__field(u32, ch_out)
			__field(u32, format)
			__field(u32, sample_rate)
			__field(size_t, frag_sz)
			__field(size_t, frag_n)
			__field(int, ret)
			),
	TP_fast_assign(
			__entry->codec_id = params->codec.id;
			__entry->ch_in = params->codec.ch_in;
			__entry->ch_out = params->codec.ch_out;
			__entry->format = params->codec.format;
			__entry->sample_rate = params->codec.sample_rate;
			__entry->frag_sz = params->buffer.fragment_size;
			__entry->frag_n = params->buffer.fragments;
			__entry->ret = ret;
		),
	TP_printk(
		  "codec id: %u; channels i/o %u/%u); format: %u; sample rate: %u; fragment size/count %zu/%zu; ret: %d",
		  __entry->codec_id,
		  __entry->ch_in,
		  __entry->ch_out,
		  __entry->format,
		  __entry->sample_rate,
		  __entry->frag_sz,
		  __entry->frag_n,
		  __entry->ret)
);

TRACE_EVENT(clsic_alg_compr_stream_get_caps,
	TP_PROTO(struct snd_compr_caps *caps, int ret),
	TP_ARGS(caps, ret),
	TP_STRUCT__entry(
			__field(u32, num_codecs)
			__field(u32, direction)
			__field(u32, min_fragment_size)
			__field(u32, max_fragment_size)
			__field(u32, min_fragments)
			__field(u32, max_fragments)
			__field(int, ret)
			),
	TP_fast_assign(
			__entry->num_codecs = caps->num_codecs;
			__entry->direction = caps->direction;
			__entry->min_fragment_size = caps->min_fragment_size;
			__entry->max_fragment_size = caps->max_fragment_size;
			__entry->min_fragments = caps->min_fragments;
			__entry->max_fragments = caps->max_fragments;
			__entry->ret = ret;
		),
	TP_printk(
		  "num_codecs: %u; direction %u; fragment_size: %u-%u words; fragments: %u-%u; ret: %d",
		  __entry->num_codecs,
		  __entry->direction,
		  __entry->min_fragment_size,
		  __entry->max_fragment_size,
		  __entry->min_fragments,
		  __entry->max_fragments,
		  __entry->ret)
);

TRACE_EVENT(clsic_alg_compr_stream_trigger,
	TP_PROTO(int cmd, int ret),
	TP_ARGS(cmd, ret),
	TP_STRUCT__entry(
			__field(int, cmd)
			__field(int, ret)
			),
	TP_fast_assign(
			__entry->cmd = cmd;
			__entry->ret = ret;
		),
	TP_printk("cmd: %d; ret: %d",
		  __entry->cmd,
		  __entry->ret)
);

TRACE_EVENT(clsic_alg_compr_stream_timestamp,
	TP_PROTO(struct snd_compr_tstamp *tstamp, int ret),
	TP_ARGS(tstamp, ret),
	TP_STRUCT__entry(
			__field(u32, byte_offset)
			__field(u32, copied_total)
			__field(u32, sampling_rate)
			__field(int, ret)
			),
	TP_fast_assign(
			__entry->byte_offset = tstamp->byte_offset;
			__entry->copied_total = tstamp->copied_total;
			__entry->sampling_rate = tstamp->sampling_rate;
			__entry->ret = ret;
		),
	TP_printk(
		  "byte_offset: %u; copied_total %u; sampling_rate: %u; ret: %d",
		  __entry->byte_offset,
		  __entry->copied_total,
		  __entry->sampling_rate,
		  __entry->ret)
);

TRACE_EVENT(clsic_alg_compr_stream_copy,
	TP_PROTO(size_t count, int ret),
	TP_ARGS(count, ret),
	TP_STRUCT__entry(
			__field(size_t, count)
			__field(int, ret)
			),
	TP_fast_assign(
			__entry->count = count;
			__entry->ret = ret;
		),
	TP_printk("copied: %zu bytes; ret: %d", __entry->count, __entry->ret)
);

#endif /* CLSIC_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE clsic-trace
#include <trace/define_trace.h>
