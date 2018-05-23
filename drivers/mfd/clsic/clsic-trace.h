/*
 * clsic-trace.h -- CLSIC tracepoints header file
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
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

TRACE_EVENT(clsic_ras_simpleread,
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

TRACE_EVENT(clsic_ras_bulkwrite,
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

TRACE_EVENT(clsic_ras_bulkread,
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

TRACE_EVENT(clsic_ras_pm_handler,
	TP_PROTO(int pm_event),
	TP_ARGS(pm_event),
	TP_STRUCT__entry(
			__field(int, pm_event)
			),
	TP_fast_assign(
			__entry->pm_event = pm_event;
		),
	TP_printk(
			"    pm event: %d",
			__entry->pm_event
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

DEFINE_EVENT(clsic_generic, clsic_simirq_write_asserted,
	TP_PROTO(uint8_t dummy),
	TP_ARGS(dummy)
);

DEFINE_EVENT(clsic_generic, clsic_simirq_write_deasserted,
	TP_PROTO(uint8_t dummy),
	TP_ARGS(dummy)
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

#endif /* CLSIC_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE clsic-trace
#include <trace/define_trace.h>
