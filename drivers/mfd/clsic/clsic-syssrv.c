/*
 * clsic-syssrv.c -- CLSIC System Service
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <sound/soc.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <linux/mfd/clsic/core.h>
#include <linux/mfd/clsic/message.h>
#include <linux/mfd/clsic/irq.h>
#include <linux/mfd/clsic/syssrv.h>
#include <linux/mfd/clsic/rassrv.h>
#include <linux/mfd/clsic/bootsrv.h>
#include "clsic-trace.h"

/*
 * This handler function will be called frequently by the incoming messages
 * context when a system service notification is received, many of the system
 * service notifications are concerned with the operation of the messaging
 * protocol and this handler calls back to the messaging layer to do the actual
 * named work.
 */
static int clsic_system_service_handler(struct clsic *clsic,
					struct clsic_service *handler,
					struct clsic_message *msg)
{
	enum clsic_sys_msg_id system_msgid;
	int ret = CLSIC_UNHANDLED;

	/* Make sure it is a notification message */
	if (clsic_get_cran_frommsg(msg) != CLSIC_CRAN_NTY) {
		clsic_dump_message(clsic, msg, "unhandled message");
		return ret;
	}
	system_msgid = clsic_get_messageid(msg);
	switch (system_msgid) {
	case CLSIC_SYS_MSG_N_RXDMA_STS:
		clsic_handle_message_rxdma_status(clsic, msg);
		ret = CLSIC_HANDLED;
		break;
	case CLSIC_SYS_MSG_N_INVAL_CMD:
		clsic_handle_message_invalid_cmd(clsic, msg);
		ret = CLSIC_HANDLED;
		break;
	case CLSIC_SYS_MSG_N_PANIC:
		clsic_dev_panic(clsic, msg);
		ret = CLSIC_HANDLED;
		break;
	default:
		clsic_err(clsic, "unrecognised message\n");
		clsic_dump_message(clsic, msg, "Unrecognised message");
	}
	return ret;
}

static void clsic_system_service_stop(struct clsic *clsic,
				      struct clsic_service *handler)
{
	clsic_dbg(clsic, "%p %d", handler, clsic->msgproc_state);

	/*
	 * All the other services will have shutdown before this function is
	 * called and the device should now be idle.
	 *
	 * The system service is responsible for making sure that the device
	 * can have it's power removed, make sure the messaging processor is
	 * off.
	 */
	clsic_send_shutdown_cmd(clsic);

	if (handler->data != NULL) {
		kfree(handler->data);
		handler->data = NULL;
	}
}

/* Structure containing the System Service instance data */
struct clsic_syssrv_struct {
	struct clsic *clsic;

	struct clsic_service *srv;
	uint32_t trace_level;
	uint32_t trace_mask;
};

/*
 * There is a command that can set both the trace log level and mask in one go,
 * however there isn't a mechanism to read what the values were before changing
 * them so a read - modify - write message pattern can't be used.
 *
 * As the old values are returned in the response it would be possible to
 * derive what the value was by issuing the command with a dummy value and then
 * resetting it back to the original value, however the simplest design was
 * chosen.
 *
 * Note; this value is not persisted by the system firmware, so if the
 * messaging processor is shutdown then it will be lost. We can't use the
 * suspend and resume PM hooks as we can send a message to a device that has
 * had the messaging processor shutdown but has not been suspended.
 */
static int clsic_system_service_set_trace(struct clsic_syssrv_struct *syssrv)
{
	union clsic_sys_msg msg_cmd;
	union clsic_sys_msg msg_rsp;
	int ret;

	clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
			   CLSIC_SRV_INST_SYS,
			   CLSIC_SYS_MSG_CR_SET_TRACE_FILTER);

	msg_cmd.cmd_set_trace_filter.new_level = syssrv->trace_level;
	msg_cmd.cmd_set_trace_filter.new_mask = syssrv->trace_mask;

	memset(&msg_rsp, 0, CLSIC_FIXED_MSG_SZ);

	ret = clsic_send_msg_sync(syssrv->clsic,
				  (union t_clsic_generic_message *) &msg_cmd,
				  (union t_clsic_generic_message *) &msg_rsp,
				  CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				  CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN);

	clsic_info(syssrv->clsic,
		   "ret; %d, (err; %d) lvl: 0x%x -> 0x%x mask: 0x%x -> 0x%x\n",
		   ret,
		   msg_rsp.rsp_set_trace_filter.hdr.err,
		   msg_rsp.rsp_set_trace_filter.old_level,
		   msg_cmd.cmd_set_trace_filter.new_level,
		   msg_rsp.rsp_set_trace_filter.old_mask,
		   msg_cmd.cmd_set_trace_filter.new_mask);

	if (msg_rsp.rsp_set_trace_filter.hdr.err != 0)
		ret = -EINVAL;

	return ret;
}

static int clsic_system_service_set_trace_level(void *data, u64 val)
{
	struct clsic_syssrv_struct *syssrv = data;
	int ret;

	syssrv->trace_level = val;

	pm_runtime_get_sync(syssrv->clsic->dev);
	ret = clsic_system_service_set_trace(syssrv);
	pm_runtime_put_autosuspend(syssrv->clsic->dev);
	return ret;

}

static int clsic_system_service_get_trace_level(void *data, u64 *val)
{
	struct clsic_syssrv_struct *syssrv = data;

	*val = syssrv->trace_level;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(clsic_system_service_trace_level_fops,
			 clsic_system_service_get_trace_level,
			 clsic_system_service_set_trace_level, "%llx\n");

static int clsic_system_service_set_trace_mask(void *data, u64 val)
{
	struct clsic_syssrv_struct *syssrv = data;
	int ret;

	syssrv->trace_mask = val;

	pm_runtime_get_sync(syssrv->clsic->dev);
	ret = clsic_system_service_set_trace(syssrv);
	pm_runtime_put_autosuspend(syssrv->clsic->dev);
	return ret;
}

static int clsic_system_service_get_trace_mask(void *data, u64 *val)
{
	struct clsic_syssrv_struct *syssrv = data;

	*val = syssrv->trace_mask;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(clsic_system_service_trace_mask_fops,
			 clsic_system_service_get_trace_mask,
			 clsic_system_service_set_trace_mask, "%llx\n");

/*
 * Simple pm_handler that sets the device trace options if they have been
 * changed from the default.
 */
static int clsic_system_service_pm_handler(struct clsic_service *handler,
					   int pm_event)
{
	struct clsic_syssrv_struct *syssrv = handler->data;
	int ret = 0;

	switch (pm_event) {
	case PM_EVENT_RESUME:
		if ((syssrv->trace_level != CLSIC_SYSSRV_DEFAULT_TRACE_LEVEL) ||
		    (syssrv->trace_mask != CLSIC_SYSSRV_DEFAULT_TRACE_MASK))
			clsic_system_service_set_trace(syssrv);
		break;
	case PM_EVENT_SUSPEND:
	default:
		break;
	}

	return ret;
}


int clsic_system_service_start(struct clsic *clsic,
			       struct clsic_service *handler)
{
	struct clsic_syssrv_struct *syssrv;

	/*
	 * In the reenumeration case the system service handler structure will
	 * be allocated, but the service info should be regenerated.
	 */
	if ((handler->stop == &clsic_system_service_stop) &&
	    (handler->data != NULL)) {
		clsic_info(clsic, "System service fw version %d.%d.%d",
			   (handler->service_version & CLSIC_SVCVER_MAJ_MASK) >>
			   CLSIC_SVCVER_MAJ_SHIFT,
			   (handler->service_version & CLSIC_SVCVER_MIN_MASK) >>
			   CLSIC_SVCVER_MIN_SHIFT,
			   (handler->service_version & CLSIC_SVCVER_BLD_MASK) >>
			   CLSIC_SVCVER_BLD_SHIFT);
		return 0;
	}

	syssrv = kzalloc(sizeof(struct clsic_syssrv_struct), GFP_KERNEL);
	if (syssrv == NULL)
		return -ENOMEM;

	syssrv->clsic = clsic;
	syssrv->srv = handler;
	syssrv->trace_level = CLSIC_SYSSRV_DEFAULT_TRACE_LEVEL;
	syssrv->trace_mask = CLSIC_SYSSRV_DEFAULT_TRACE_MASK;

	debugfs_create_file("sysfwtrace_level", 0220, clsic->debugfs_root,
			    syssrv, &clsic_system_service_trace_level_fops);
	debugfs_create_file("sysfwtrace_mask", 0220, clsic->debugfs_root,
			    syssrv, &clsic_system_service_trace_mask_fops);

	handler->callback = &clsic_system_service_handler;
	handler->stop = &clsic_system_service_stop;
	handler->data = syssrv;
	handler->supports_debuginfo = true;
	handler->pm_handler = &clsic_system_service_pm_handler;

	return 0;
}

/*
 * Called by the core driver after receiving a boot done interrupt, enumerate
 * the services on a CLSIC device.
 */
int clsic_system_service_enumerate(struct clsic *clsic)
{
	union clsic_sys_msg msg_cmd;
	union clsic_sys_msg msg_rsp;
	uint8_t service_count = 0;
	int ret;
	uint8_t service_instance = 0;
	uint8_t services_found = 0;
	uint16_t service_type;
	uint32_t service_version;

	/*
	 * The "first touch" message that wakes the device may generate a
	 * bootloader notification so this message may fail message with
	 * CLSIC_MSG_INTERRUPTED.
	 *
	 * If the device is dead then this command may also timeout - in that
	 * case initiate recovery measures.
	 */
	clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
			   CLSIC_SRV_INST_SYS, CLSIC_SYS_MSG_CR_SYS_INFO);

	ret = clsic_send_msg_sync(clsic,
				  (union t_clsic_generic_message *) &msg_cmd,
				  (union t_clsic_generic_message *) &msg_rsp,
				  CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				  CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN);

	if (ret != 0) {
		clsic_err(clsic, "Sysinfo ret %d\n", ret);
		return ret;
	}

	if (clsic->service_states == CLSIC_ENUMERATED)
		/* Nothing to do (typical power on resume) */
		return 0;

	service_count = msg_rsp.rsp_sys_info.srv_count;
	clsic_dbg(clsic, "Sysinfo service count %d\n", service_count);

	if (service_count > CLSIC_SERVICE_COUNT) {
		clsic_err(clsic, "Sysinfo response larger than max %d\n",
			  service_count);
		return -EINVAL;
	}

	mutex_lock(&clsic->service_lock);

	/*
	 * If the device reports two services (0 and 1) then suspend services
	 * from instance 2 onwards as they cannot be present.
	 */
	clsic_suspend_services_from(clsic, service_count);

	/* Enumerate services */
	for (service_instance = 0;
	     (services_found < service_count) &&
	     (service_instance < CLSIC_SERVICE_COUNT);
	     service_instance++) {
		clsic_dbg(clsic, "Examine instance %d (found count %d)",
			  service_instance, services_found);
		/* Read the service type */
		clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
				   CLSIC_SRV_INST_SYS,
				   CLSIC_SYS_MSG_CR_SRV_INFO);
		msg_cmd.cmd_srv_info.srv_inst = service_instance;

		ret = clsic_send_msg_sync(clsic,
				     (union t_clsic_generic_message *) &msg_cmd,
				     (union t_clsic_generic_message *) &msg_rsp,
				     CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				     CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN);

		/* Any message sending error is fatal */
		if (ret != 0) {
			clsic_err(clsic,
				  "getserviceinfo %d: send_message %d\n",
				  service_instance, ret);

			mutex_unlock(&clsic->service_lock);
			return ret;
		}

		/* Services are not sparse so should never encounter this */
		if (msg_rsp.rsp_srv_info.hdr.err == CLSIC_ERR_INVAL_SI) {
			clsic_err(clsic, "getserviceinfo %d: no service\n",
				  service_instance);
			break;
		}

		services_found++;
		service_type = msg_rsp.rsp_srv_info.srv_type;
		service_version = msg_rsp.rsp_srv_info.srv_ver;

		clsic_dbg(clsic,
			  " Found service id %d type 0x%x version 0x%x (%d.%d.%d)",
			  service_instance, service_type, service_version,
			  (service_version & CLSIC_SVCVER_MAJ_MASK) >>
			  CLSIC_SVCVER_MAJ_SHIFT,
			  (service_version & CLSIC_SVCVER_MIN_MASK) >>
			  CLSIC_SVCVER_MIN_SHIFT,
			  (service_version & CLSIC_SVCVER_BLD_MASK) >>
			  CLSIC_SVCVER_BLD_SHIFT);

		clsic_update_service(clsic, service_instance, service_type,
				     service_version);
	}

	mutex_unlock(&clsic->service_lock);

	clsic_dbg(clsic, "Enumerate found %d services (error: %d)",
		  services_found, ret);

	clsic->service_states = CLSIC_ENUMERATED;
	return 0;
}

/*
 * This helper function is called when the driver desires the messaging
 * processor to be in a shutdown state.
 */
int clsic_send_shutdown_cmd(struct clsic *clsic)
{
	union clsic_sys_msg msg_cmd;
	union clsic_sys_msg msg_rsp;
	int ret = 0;

	if (clsic->msgproc_state == CLSIC_MSGPROC_OFF)
		return 0;

	clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
			   CLSIC_SRV_INST_SYS,
			   CLSIC_SYS_MSG_CR_SP_SHDN);

	ret = clsic_send_msg_sync(clsic,
				  (union t_clsic_generic_message *) &msg_cmd,
				  (union t_clsic_generic_message *) &msg_rsp,
				  CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				  CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN);

	clsic_dbg(clsic,
		  "Shutdown message returned 0x%x 0x%x: bitmap 0x%x\n",
		  ret,
		  msg_rsp.rsp_sp_shdn.hdr.err,
		  msg_rsp.rsp_sp_shdn.srvs_hold_wakelock);

	return ret;
}

/*
 * This helper function is called to issue general purpose IOCTLs to the system
 * service; it will initially be used to disable the external codec SPI bus
 * during codec power transitions.
 */
int clsic_system_service_ioctl(struct clsic *clsic, enum clsic_sys_ioctl ioctl)
{
	union clsic_sys_msg msg_cmd;
	union clsic_sys_msg msg_rsp;
	int ret;

	clsic_init_message((union t_clsic_generic_message *) &msg_cmd,
			   CLSIC_SRV_INST_SYS,
			   CLSIC_SYS_MSG_CR_IOCTL);

	msg_cmd.cmd_ioctl.args.raw_payload[0] = ioctl;

	ret = clsic_send_msg_sync(clsic,
				  (union t_clsic_generic_message *) &msg_cmd,
				  (union t_clsic_generic_message *) &msg_rsp,
				  CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				  CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN);

	if ((ret != 0) || (msg_rsp.rsp_ioctl.hdr.err != 0))
		clsic_err(clsic, "ret 0x%x err 0x%x\n", ret,
			  msg_rsp.rsp_ioctl.hdr.err);

	if (ret != 0)
		return ret;

	/*
	 * Given the wide spread of possible errors, this currently returns
	 * whether it succeeded or not.
	 */
	if (msg_rsp.rsp_ioctl.hdr.err)
		return -EIO;

	return 0;
}
