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

/**
 *
 * Compare the type of all the driver service handlers with that reported by
 * the device looking for differences (except the bootloader service handler
 * that isn't part of a running system). If the service type matches then the
 * service version is updated with that read from the device.
 *
 * If no services have changed type, notify the service handlers of a
 * reenumeration so they may update their state (and perhaps check their
 * discovered service version is supported).
 */
static int clsic_system_service_reenumerate(struct clsic *clsic)
{
	union clsic_sys_msg msg_cmd;
	union clsic_sys_msg msg_rsp;
	uint16_t read_service_type;
	uint16_t handler_service_type;
	uint8_t tmp_instance;
	struct clsic_service *tmp_handler;
	int ret;

	for (tmp_instance = 0;
	     tmp_instance < CLSIC_SRV_INST_BLD;
	     tmp_instance++) {
		read_service_type = 0x0;
		tmp_handler = clsic->service_handlers[tmp_instance];
		if (tmp_handler != NULL)
			handler_service_type = tmp_handler->service_type;
		else
			handler_service_type = 0x0;

		clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
				   CLSIC_SRV_INST_SYS,
				   CLSIC_SYS_MSG_CR_SRV_INFO);
		msg_cmd.cmd_srv_info.srv_inst = tmp_instance;
		ret = clsic_send_msg_sync(clsic,
				     (union t_clsic_generic_message *) &msg_cmd,
				     (union t_clsic_generic_message *) &msg_rsp,
				     CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				     CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN);

		if ((ret == 0) &&
		    (msg_rsp.rsp_srv_info.hdr.err == CLSIC_ERR_NONE))
			read_service_type = msg_rsp.rsp_srv_info.srv_type;

		if (handler_service_type != read_service_type) {
			clsic_err(clsic,
				  "id %d %p type read 0x%x != handler 0x%x",
				  tmp_instance, tmp_handler, read_service_type,
				  handler_service_type);
			clsic_err(clsic,
				  "Service configuration changed - device HALTED\n");
			clsic_device_error(clsic,
					   CLSIC_DEVICE_ERROR_LOCKNOTHELD);
			return -EINVAL;
		}

		/* Update the version of read services */
		if (tmp_handler != NULL)
			tmp_handler->service_version =
				msg_rsp.rsp_srv_info.srv_ver;
	}

	clsic->service_states = CLSIC_ENUMERATED;

	/* Services have not changed, notify all services of a restart */
	for (tmp_instance = 0;
	     tmp_instance < CLSIC_SRV_INST_BLD;
	     tmp_instance++) {
		tmp_handler = clsic->service_handlers[tmp_instance];
		if (tmp_handler != NULL &&
		    tmp_handler->start != NULL) {
			clsic_dbg(clsic, "%d: %pF",
				  tmp_instance, tmp_handler->start);
			tmp_handler->start(clsic, tmp_handler);
		}
	}

	return 0;
}

static int clsic_system_service_finder(struct clsic *clsic,
				uint8_t service_instance,
				uint16_t service_type,
				uint32_t service_version)
{
	unsigned int type;
	int ret;
	struct mfd_cell *dev;
	struct device_node *services_np, *child_np;

	services_np = of_get_child_by_name(clsic->dev->of_node,
					   "cirrus,services");
	for_each_child_of_node(services_np, child_np) {
		of_property_read_u32(child_np, "cirrus,service-type", &type);
		if (service_type != type)
			continue;
		if (clsic->service_handlers[service_instance] != NULL)
			return 0;
		dev = devm_kzalloc(clsic->dev, sizeof(struct mfd_cell),
				   GFP_KERNEL);
		if (!dev) {
			ret = -ENOMEM;
			goto error;
		}

		ret = of_property_read_string(child_np, "name", &dev->name);
		if (ret)
			goto error_withfree;

		ret = of_property_read_string(child_np, "compatible",
					      &dev->of_compatible);
		if (ret)
			goto error_withfree;

		clsic_register_service_handler(clsic, service_instance,
					       service_type,
					       service_version,
					       NULL);

		clsic->service_handlers[service_instance]->data = clsic;

		dev->platform_data = clsic->service_handlers[service_instance];
		dev->pdata_size = sizeof(struct clsic_service);

		ret = mfd_add_devices(clsic->dev, PLATFORM_DEVID_NONE, dev, 1,
				      NULL, 0, NULL);
		if (ret)
			goto error_withfree;
		return 0;
	}

	/* unrecognised */
	clsic_err(clsic,
		  " Unrecognised service (%d: type 0x%x ver 0x%x)",
		  service_instance, service_type,
		  service_version);

	return -ENODEV;

error_withfree:
	devm_kfree(clsic->dev, dev);
error:
	clsic_err(clsic,
		  " Failed to add device (%d: type 0x%x ver 0x%x)",
		  service_instance, service_type,
		  service_version);
	return ret;
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
	struct clsic_syssrv_struct *syssrv =
		clsic->service_handlers[CLSIC_SRV_INST_SYS]->data;

	if (syssrv == NULL) {
		clsic_err(clsic, "No system service data\n");
		return -EINVAL;
	}

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
		if (ret == -ETIMEDOUT) {
			/*
			 * TODO: there is an argument that the device should be
			 * left in the HALTED state at this point rather than
			 * resetting into firmware update.
			 *
			 * First touch message timed out - restart the device
			 * in firmware update mode to attempt recovery
			 */
			clsic_fwupdate_reset(clsic);
		} else if (ret == -EINVAL) {
			/*
			 *  An invalid command response would occur if the
			 *  device was in the bootloader
			 */
		}
		return ret;
	}

	switch (clsic->service_states) {
	case CLSIC_ENUMERATED:
		/* Nothing to do (typical power on resume) */
		return 0;
	case CLSIC_ENUMERATION_REQUIRED:
		/* Continue and perform first device enumeration */
		break;
	case CLSIC_REENUMERATION_REQUIRED:
		/* Check the services match (after a firmware update) */
		return clsic_system_service_reenumerate(clsic);
	default:
		clsic_info(clsic, "Skipping enumeration; service state %d\n",
			   clsic->service_states);
		return -EBUSY;
	}

	clsic_dbg(clsic, "Sysinfo ret 0x%x 0x%x 0x%x\n",
		  msg_rsp.rsp_sys_info.hdr.sbc,
		  msg_rsp.rsp_sys_info.hdr.msgid,
		  msg_rsp.rsp_sys_info.hdr.err);

	service_count = msg_rsp.rsp_sys_info.srv_count;

	clsic_dbg(clsic, "Sysinfo service count %d\n", service_count);

	/*
	 * The message size is stored in a byte, but there is only 5 bits of
	 * addressable services
	 */
	if (service_count > CLSIC_SERVICE_COUNT) {
		clsic_err(clsic, "Sysinfo response larger than max %d\n",
			  service_count);
		service_count = CLSIC_SERVICE_COUNT;
	}

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

		if (ret != 0) {
			/* XXX need to determine if this send message error was
			 * fatal
			 *
			 * If the command was cancelled due to a bootloader
			 * event then it should be considered fatal
			 */
			clsic_err(clsic,
				  "getserviceinfo %d: send_message %d\n",
				  service_instance, ret);
			continue;
		}

		/*
		 * Move on to examine the next service instance when
		 * getserviceinfo encounters an invalid service instance error
		 * code (this just means that the services are sparse)
		 */
		if (msg_rsp.rsp_srv_info.hdr.err == CLSIC_ERR_INVAL_SI) {
			clsic_dbg(clsic, "getserviceinfo %d: no service\n",
				  service_instance);
			continue;
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

		switch (service_type) {
		case CLSIC_SRV_TYPE_SYS:
			/*
			 * Print the version if changes (The version is
			 * initialised to zero so it should always be printed
			 * on first boot)
			 */
			if (clsic->service_handlers[service_instance]
					->service_version != service_version)
				clsic_info(clsic,
					   "System service fw version %d.%d.%d",
					   (service_version &
					    CLSIC_SVCVER_MAJ_MASK) >>
					   CLSIC_SVCVER_MAJ_SHIFT,
					   (service_version &
					    CLSIC_SVCVER_MIN_MASK) >>
					   CLSIC_SVCVER_MIN_SHIFT,
					   (service_version &
					    CLSIC_SVCVER_BLD_MASK) >>
					   CLSIC_SVCVER_BLD_SHIFT);
			/* fallthrough */
		case CLSIC_SERVICE_TYPE_BOOTLOADER:
			clsic_dbg(clsic,
				  " Service %d is a standard service (type 0x%x)",
				  service_instance, service_type);

			clsic->service_handlers[service_instance]
				->service_version = service_version;
			break;
		case CLSIC_SRV_TYPE_RAS:
			clsic_register_service_handler(clsic,
						    service_instance,
						    service_type,
						    service_version,
						    clsic_ras_start);
			break;
		default:
			ret = clsic_system_service_finder(clsic,
							  service_instance,
							  service_type,
							  service_version);
			if (!ret)
				break;

			clsic_register_service_handler(clsic,
						       service_instance,
						       service_type,
						       service_version, NULL);
		}
	}

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
