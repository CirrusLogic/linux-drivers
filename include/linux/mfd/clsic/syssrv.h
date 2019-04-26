/*
 * syssrv.h -- CLSIC System Service
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef CIRRUS_CLSIC_SYSSRV_H
#define CIRRUS_CLSIC_SYSSRV_H

#include <linux/mfd/clsic/clsicmessagedefines_SYS.h>

#define CLSIC_SVCVER_MAJ_SHIFT	24
#define CLSIC_SVCVER_MAJ_MASK	0xFF000000

#define CLSIC_SVCVER_MIN_SHIFT	16
#define CLSIC_SVCVER_MIN_MASK	0x00FF0000

#define CLSIC_SVCVER_BLD_SHIFT	0
#define CLSIC_SVCVER_BLD_MASK	0x0000FFFF

/*
 * The trace level and mask for the system cannot be just read so these are the
 * current initial values.
 */
#define CLSIC_SYSSRV_DEFAULT_TRACE_LEVEL 0x2
#define CLSIC_SYSSRV_DEFAULT_TRACE_MASK 0xFFFFFFFF

int clsic_system_service_start(struct clsic *clsic,
			       struct clsic_service *handler);

int clsic_system_service_enumerate(struct clsic *clsic);
int clsic_send_shutdown_cmd(struct clsic *clsic);

/*
 * Returns true if the message is a notification to the system service of an
 * invalid command and the reported invalid command matches the provided
 * service instance and message id.
 */
static inline bool clsic_system_cmd_is_inval(union t_clsic_generic_message *msg,
					     const uint8_t service_instance,
					     const uint8_t msgid)
{
	union clsic_sys_msg *msg_sys = (union clsic_sys_msg *) msg;

	if ((clsic_get_cran(msg->notif.hdr.sbc) == CLSIC_CRAN_NTY) &&
	    (clsic_get_srv_inst(msg->notif.hdr.sbc) == CLSIC_SRV_INST_SYS) &&
	    (msg->notif.hdr.msgid == CLSIC_SYS_MSG_N_INVAL_CMD) &&
	    (msg_sys->nty_inval_cmd.srv_inst == service_instance) &&
	    (msg_sys->nty_inval_cmd.msgid == msgid))
		return true;

	return false;
}

#endif
