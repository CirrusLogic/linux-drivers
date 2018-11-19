/*
 * clsic-rassrv.c -- CLSIC Register Access Service
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
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
 * The regmap we expose is 32bits address and data width
 */
#define CLSIC_RAS_REG_BITS	32
#define CLSIC_RAS_REG_BYTES	(CLSIC_RAS_REG_BITS/BITS_PER_BYTE)
#define CLSIC_RAS_VAL_BITS	32
#define CLSIC_RAS_VAL_BYTES	(CLSIC_RAS_VAL_BITS/BITS_PER_BYTE)

/*
 * actually is 1024, but using a multiple of 3 and 5 solves issues with
 * accessing packed DSP memories
 */
#define CLSIC_RAS_MAX_BULK_SZ	960

/*
 * This service uses the handler data pointer to stash a instance specific data
 * structure so it must be released when the service is stopped.
 */
static void clsic_ras_stop(struct clsic *clsic, struct clsic_service *handler)
{
	struct clsic_ras_struct *ras;

	if (handler->data != NULL) {
		/*
		 * Data and regmap are devm_kzalloc'd and will be freed when
		 * the driver unloads. Make the regmap cache only so clients
		 * don't receive errors.
		 */
		ras = handler->data;
		regcache_cache_only(ras->regmap, true);
		handler->data = NULL;
	}
}

/*
 * The simple readregister and writeregister routines are the core of the
 * remote access service and they translates a simple register accesses into
 * messages sent to the remote access service present in the device.
 */
static int clsic_ras_simple_readregister(struct clsic_ras_struct *ras,
					 uint32_t address, __be32 *value)
{
	struct clsic *clsic;
	union clsic_ras_msg msg_cmd;
	union clsic_ras_msg msg_rsp;
	int ret = 0;

	if (ras == NULL)
		return -EINVAL;

	clsic = ras->clsic;

	/* Format and send a message to the remote access service */
	clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
			   ras->service_instance,
			   CLSIC_RAS_MSG_CR_RDREG);
	msg_cmd.cmd_rdreg.addr = address;

	/* Clear err to avoid confusion as it is always included in the trace */
	msg_rsp.rsp_wrreg.hdr.err = 0;

	ret = clsic_send_msg_sync(clsic,
				  (union t_clsic_generic_message *) &msg_cmd,
				  (union t_clsic_generic_message *) &msg_rsp,
				  CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				  CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN);

	/*
	 *  Clients to this function can't interpret detailed error codes so
	 *  map error to -EIO
	 */
	if (ret != 0) {
		clsic_dbg(clsic, "0x%x ret %d", address, ret);
		ret = -EIO;
	} else if (msg_rsp.rsp_rdreg.hdr.err != 0) {
		clsic_dbg(clsic, "addr: 0x%x status %d\n", address,
			  msg_rsp.rsp_rdreg.hdr.err);
		ret = -EIO;
	} else {
		/* The request succeeded */
		ret = 0;

		/*
		 * The regmap bus is declared as BIG endian but all the
		 * accesses this service makes are CPU native so the value may
		 * need to be converted.
		 */
		*value = cpu_to_be32(msg_rsp.rsp_rdreg.value);
	}

	trace_clsic_ras_simpleread(msg_cmd.cmd_rdreg.addr,
				   msg_rsp.rsp_rdreg.value, ret,
				   msg_rsp.rsp_rdreg.hdr.err);
	return ret;
}

static int clsic_ras_simple_writeregister(struct clsic_ras_struct *ras,
					  uint32_t address, uint32_t value)
{
	struct clsic *clsic;
	union clsic_ras_msg msg_cmd;
	union clsic_ras_msg msg_rsp;
	int ret = 0;

	if (ras == NULL)
		return -EINVAL;

	clsic = ras->clsic;

	/* Format and send a message to the remote access service */
	clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
			   ras->service_instance,
			   CLSIC_RAS_MSG_CR_WRREG);
	msg_cmd.cmd_wrreg.addr = address;
	msg_cmd.cmd_wrreg.value = value;

	/* Clear err to avoid confusion as it is always included in the trace */
	msg_rsp.rsp_wrreg.hdr.err = 0;

	ret = clsic_send_msg_sync(clsic,
				  (union t_clsic_generic_message *) &msg_cmd,
				  (union t_clsic_generic_message *) &msg_rsp,
				  CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				  CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN);
	/*
	 *  Clients to this function can't interpret detailed error codes so
	 *  map error to -EIO
	 */
	if (ret != 0) {
		clsic_dbg(clsic, "0x%x ret %d", address, ret);
		ret = -EIO;
	} else if (msg_rsp.rsp_wrreg.hdr.err != 0) {
		clsic_dbg(clsic, "addr: 0x%x status %d\n", address,
			  msg_rsp.rsp_wrreg.hdr.err);
		ret = -EIO;
	} else {
		/* The request succeeded */
		ret = 0;
	}

	trace_clsic_ras_simplewrite(msg_cmd.cmd_wrreg.addr,
				    msg_cmd.cmd_wrreg.value,
				    ret, msg_rsp.rsp_wrreg.hdr.err);
	return ret;
}

/*
 * This function is called when a single register write is performed on the
 * regmap, it translates the context back into a ras structure so the
 * request can be sent through the messaging layer and fulfilled
 */
int clsic_ras_reg_write(void *context, unsigned int reg, uint32_t val)
{
	struct clsic_ras_struct *ras = context;

	return clsic_ras_simple_writeregister(ras, reg, val);
}

/*
 * This function is called when a single register read is performed on the
 * regmap, it translates the context back into a ras structure so the
 * request can be sent through the messaging layer and fulfilled
 */
int clsic_ras_reg_read(void *context, unsigned int reg, uint32_t *val)
{
	struct clsic_ras_struct *ras = context;

	return clsic_ras_simple_readregister(ras, reg, (__be32 *) val);
}

/*
 * This function is called when a number of sequential register reads are
 * requested on the regmap, it has to iterate through the request making
 * individual register read requests.
 *
 * As with the sub functions it uses, the values passed to this function are in
 * the regmap bus order that we're exposing (big endian) so the function needs
 * to translate the address to read to the native cpu ordering before sending
 * on the request.
 *
 * If/when the register access service gets multiple register read/write
 * operations this could be made more efficient.
 */
static int clsic_ras_read(void *context, const void *reg_buf,
			  const size_t reg_size, void *val_buf,
			  const size_t val_size)
{
	struct clsic_ras_struct *ras = context;
	struct clsic *clsic;
	int ret = 0;
	u32 reg = be32_to_cpu(*(const __be32 *) reg_buf);
	size_t i, frag_sz;
	union clsic_ras_msg msg_cmd;
	union clsic_ras_msg msg_rsp;
	uint8_t err = 0;

	if (ras == NULL)
		return -EINVAL;

	clsic = ras->clsic;

	if (val_size == CLSIC_RAS_VAL_BYTES)
		return clsic_ras_simple_readregister(ras, reg,
						     (__be32 *) val_buf);

	for (i = 0; i < val_size; i += CLSIC_RAS_MAX_BULK_SZ) {
		/* Format and send a message to the remote access service */
		clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
				   ras->service_instance,
				   CLSIC_RAS_MSG_CR_RDREG_BULK);
		frag_sz = min(val_size - i, (size_t) CLSIC_RAS_MAX_BULK_SZ);
		msg_cmd.cmd_rdreg_bulk.addr = reg + i;
		msg_cmd.cmd_rdreg_bulk.byte_count = frag_sz;

		ret = clsic_send_msg_sync(
				    clsic,
				    (union t_clsic_generic_message *) &msg_cmd,
				    (union t_clsic_generic_message *) &msg_rsp,
				    CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				    val_buf + i, frag_sz);

		/*
		 *  Clients to this function can't interpret detailed error
		 *  codes so map error to -EIO
		 */
		if (ret != 0) {
			clsic_dbg(clsic, "0x%x ret %d", reg, ret);
			ret = -EIO;
		} else if ((clsic_get_bulk_bit(msg_rsp.rsp_rdreg_bulk.hdr.sbc)
			    == 0) && (msg_rsp.rsp_rdreg_bulk.hdr.err != 0)) {
			clsic_dbg(clsic, "addr: 0x%x status %d\n", reg,
				  msg_rsp.rsp_rdreg_bulk.hdr.err);
			err = msg_rsp.rsp_rdreg_bulk.hdr.err;
			ret = -EIO;
		} else if (msg_rsp.blkrsp_rdreg_bulk.hdr.err != 0) {
			clsic_dbg(clsic, "addr: 0x%x status %d\n", reg,
				  msg_rsp.blkrsp_rdreg_bulk.hdr.err);
			err = msg_rsp.blkrsp_rdreg_bulk.hdr.err;
			ret = -EIO;
		}

		trace_clsic_ras_bulkread(msg_cmd.cmd_rdreg_bulk.addr,
					 msg_cmd.cmd_rdreg_bulk.byte_count,
					 ret, err);

		if (ret != 0)
			return ret;
	}

	/*
	 * The regmap bus is declared as BIG endian but all the
	 * accesses this service makes are CPU native so the value may
	 * need to be converted.
	 */
	for (i = 0; i < (val_size / CLSIC_RAS_VAL_BYTES); ++i)
		((__be32 *) val_buf)[i] = cpu_to_be32(((u32 *) val_buf)[i]);

	return 0;
}

static int clsic_ras_write(void *context, const void *val_buf,
			   const size_t val_size)
{
	struct clsic_ras_struct *ras = context;
	struct clsic *clsic;
	const __be32 *buf = val_buf;
	u32 addr = be32_to_cpu(buf[0]);
	int ret = 0;
	size_t i, payload_sz;
	size_t frag_sz;
	union clsic_ras_msg msg_cmd;
	union clsic_ras_msg msg_rsp;
	u32 *values;

	if (ras == NULL)
		return -EINVAL;

	clsic = ras->clsic;

	payload_sz = val_size - CLSIC_RAS_REG_BYTES;
	if ((val_size & (CLSIC_RAS_REG_BYTES - 1)) != 0) {
		clsic_err(clsic,
			  "error: context %p val_buf %p, val_size %d",
			  context, val_buf, val_size);
		clsic_err(clsic, "0x%x 0x%x 0x%x ",
			  buf[CLSIC_FSM0], buf[CLSIC_FSM1], buf[CLSIC_FSM2]);
		return -EIO;
	}

	if (val_size == (CLSIC_RAS_VAL_BYTES + CLSIC_RAS_REG_BYTES))
		return clsic_ras_simple_writeregister(ras,
						      addr,
						      be32_to_cpu(buf[1]));

	values = kzalloc(payload_sz, GFP_KERNEL);
	if (values == NULL)
		return -ENOMEM;

	for (i = 1; i < (val_size / CLSIC_RAS_VAL_BYTES); ++i)
		values[i - 1] = be32_to_cpu(buf[i]);

	for (i = 0; i < payload_sz; i += CLSIC_RAS_MAX_BULK_SZ) {
		/* Format and send a message to the remote access service */
		clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
				   ras->service_instance,
				   CLSIC_RAS_MSG_CR_WRREG_BULK);
		frag_sz = min(payload_sz - i, (size_t) CLSIC_RAS_MAX_BULK_SZ);
		msg_cmd.blkcmd_wrreg_bulk.addr = addr + i;
		msg_cmd.blkcmd_wrreg_bulk.hdr.bulk_sz = frag_sz;

		/*
		 * Clear err to avoid confusion as it is always included in the
		 * trace
		 */
		msg_rsp.rsp_wrreg.hdr.err = 0;

		ret = clsic_send_msg_sync(
				    clsic,
				    (union t_clsic_generic_message *) &msg_cmd,
				    (union t_clsic_generic_message *) &msg_rsp,
				    ((const u8 *) values) + i, frag_sz,
				    CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN);

		trace_clsic_ras_bulkwrite(msg_cmd.blkcmd_wrreg_bulk.addr,
					  msg_cmd.blkcmd_wrreg_bulk.hdr.bulk_sz,
					  ret, msg_rsp.rsp_wrreg_bulk.hdr.err);

		/*
		 *  Clients to this function can't interpret detailed error
		 *  codes so map error to -EIO
		 */
		if (ret != 0) {
			clsic_dbg(clsic, "0x%x ret %d", addr, ret);
			ret = -EIO;
			goto error;
		} else if (msg_rsp.rsp_wrreg_bulk.hdr.err != 0) {
			clsic_dbg(clsic, "addr: 0x%x status %d\n", addr,
				  msg_rsp.rsp_wrreg_bulk.hdr.err);
			ret = -EIO;
			goto error;
		}
		/* The request succeeded */
		ret = 0;
	}

error:
	kfree(values);
	return ret;
}

static int clsic_ras_gather_write(void *context, const void *reg,
				  size_t reg_len, const void *val,
				  size_t val_len)
{
	return -ENOTSUPP;
}

/*
 * The RAS service exposes a big endian regmap bus, but when we send requests
 * we are cpu native.
 */
static struct regmap_bus regmap_bus_ras = {
	.reg_write = &clsic_ras_reg_write,
	.reg_read = &clsic_ras_reg_read,
	.read = &clsic_ras_read,
	.write = &clsic_ras_write,
	.gather_write = &clsic_ras_gather_write,

	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

/*
 * Implement own regmap locking in order to silence lockdep
 * recursive lock warning.
 */
static void clsic_ras_regmap_lock(void *context)
{
	struct clsic_ras_struct *ras = context;

	mutex_lock(&ras->regmap_mutex);
}

static void clsic_ras_regmap_unlock(void *context)
{
	struct clsic_ras_struct *ras = context;

	mutex_unlock(&ras->regmap_mutex);
}

/*
 * The regmap_config for the service is different to the one setup by the main
 * driver; as this is tunneling over the messaging protocol to access the
 * registers of the device the values can be cached.
 */
static struct regmap_config regmap_config_ras = {
	.reg_bits = CLSIC_RAS_REG_BITS,
	.val_bits = CLSIC_RAS_VAL_BITS,
	.reg_stride = CLSIC_RAS_REG_BYTES,

	.lock = &clsic_ras_regmap_lock,
	.unlock = &clsic_ras_regmap_unlock,

	.max_register = TACNA_DSP2_PMEM_0,

	.readable_reg = &clsic_readable_register,
	.volatile_reg = &clsic_volatile_register,

	.name = "clsic-ras",
	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = clsic_reg_defaults,
	.num_reg_defaults = CLSIC_REG_DEFAULTS_SZ,
};

/*
 * This table specifies the sub devices supported by this bus - the kernel will
 * match up device drivers .names and call the driver probe() callbacks
 */
static struct mfd_cell clsic_devs[] = {
	{ .name = "clsic-tacna", }
};

/*
 * The RAS service does nothing when we receive a notification that the device
 * has suspended
 *
 * The driver used to switch the regmap into cache_only and marked the cache as
 * dirty, but this caused some downstream drivers to have failed regmap calls.
 *
 * On resume, marking the cache as dirty and calling sync recommits the
 * previous contents of the regmap cache
 */
static int clsic_ras_pm_handler(struct clsic_service *handler, int pm_event)
{
	struct clsic_ras_struct *ras;

	/* Will always be populated when this handler could be called */
	ras = handler->data;

	switch (pm_event) {
	case PM_EVENT_SUSPEND:
		break;

	case PM_EVENT_RESUME:
		regcache_mark_dirty(ras->regmap);
		regcache_sync(ras->regmap);
		break;

	default:
		clsic_err(ras->clsic, "Unknown PM event %d",
			  pm_event);
		break;
	}

	trace_clsic_ras_pm_handler(pm_event);

	return 0;
}

/*
 * This function is called by the system service on discovery of a register
 * access service on the device.
 *
 * It starts MFD child devices and creates a regmap bus that they can use to
 * communicate back to this instance of the device.
 */
int clsic_ras_start(struct clsic *clsic, struct clsic_service *handler)
{
	struct clsic_ras_struct *ras;
	int ret = 0;

	/*
	 * In the reenumeration case the handler structure may already be
	 * correctly configured as the core service infrastructure will call
	 * stop() on services if they change.
	 */
	if ((handler->stop == &clsic_ras_stop) &&
	    (handler->data != NULL)) {
		ras = handler->data;

		/*
		 * Check the private data structure is correct
		 */
		if ((ras->clsic == clsic) &&
		    (ras->service_instance ==
		     handler->service_instance)) {
			clsic_dbg(clsic, "%p handler structure is a full match",
				  handler);

			/*
			 * Mark dirty, switch off cache only then sync to the
			 * hardware - this recommits the last known client
			 * state.
			 */
			regcache_mark_dirty(ras->regmap);
			regcache_cache_only(ras->regmap, false);
			regcache_sync(ras->regmap);

			return 0;
		}
		/* If they don't match then the structures are corrupt */
		return -EINVAL;
	}

	ras = kzalloc(sizeof(*ras), GFP_KERNEL);
	if (ras == NULL)
		return -ENOMEM;

	/*
	 * The regmap service does not expect to receive any notifications nor
	 * catch any messages from other clients accessing the service on the
	 * device so it does not need to register a callback.
	 */
	handler->stop = &clsic_ras_stop;

	/* set pm handler for RAS to manage reg-cache */
	handler->pm_handler = &clsic_ras_pm_handler;

	mutex_init(&ras->regmap_mutex);
	regmap_config_ras.lock_arg = ras;

	ras->clsic = clsic;
	handler->data = ras;
	ras->service_instance = handler->service_instance;
	ras->regmap = devm_regmap_init(clsic->dev,
				       &regmap_bus_ras,
				       ras,
				       &regmap_config_ras);

	clsic_dbg(clsic, "srv: %p regmap: %p\n",
		  ras, ras->regmap);

	clsic_devs[0].platform_data = ras;
	clsic_devs[0].pdata_size = sizeof(struct clsic_ras_struct);

	clsic_dbg(clsic, "mfd cell 0: %p %s %p %d\n",
		  &clsic_devs[0],
		  clsic_devs[0].name,
		  clsic_devs[0].platform_data,
		  clsic_devs[0].pdata_size);

	ret = mfd_add_devices(clsic->dev, PLATFORM_DEVID_NONE, clsic_devs,
			      ARRAY_SIZE(clsic_devs), NULL, 0, NULL);

	clsic_dbg(clsic, "mfd_add_devices: ret %d\n", ret);

	return ret;
}
