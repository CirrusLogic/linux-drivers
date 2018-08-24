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

	/*
	 * Clear all the handler callbacks under the protection of the service
	 * lock
	 */
	mutex_lock(&clsic->service_lock);
	handler->start = NULL;
	handler->stop = NULL;
	handler->pm_handler = NULL;
	handler->callback = NULL;
	mutex_unlock(&clsic->service_lock);
}

/*
 * Some of the device registers are available via an external regmap read and
 * don't need to be sent to the RAS service, these reads are considerably more
 * efficient than a regular RAS message exchange.
 *
 * In particular the codec accesses the DEVID register and FLL status bits in
 * IRQ2_STS6 when adjusting clocks.
 */
static bool clsic_ras_fastread(uint32_t address)
{
	switch (address) {
	case TACNA_DEVID:
	case TACNA_REVID:
	case TACNA_FABID:
	case TACNA_RELID:
	case TACNA_OTPID:
	case CLSIC_IRQ2_STS6:
		return true;
	default:
		return false;
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
	uint32_t tmp_value;
	int ret = 0;

	if (ras->suspended)
		return -EBUSY;

	clsic = ras->clsic;

	if (clsic_ras_fastread(address)) {
		ret = regmap_read(clsic->regmap, address, &tmp_value);
		trace_clsic_ras_fastread(address, tmp_value, ret);

		/*
		 * The RAS regmap expects the value to be big endian and will
		 * convert it to CPU native so switch it to the expected
		 * format.
		 */
		*value = cpu_to_be32(tmp_value);
		return ret;
	}

	/* Format and send a message to the remote access service */
	clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
			   ras->service->service_instance,
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
	 *  map errors to -EIO
	 */
	clsic_dbg(clsic, "ret: %d addr: 0x%x status %d\n", ret, address,
		  msg_rsp.rsp_rdreg.hdr.err);
	if ((ret != 0) || (msg_rsp.rsp_rdreg.hdr.err != 0)) {
		ret = -EIO;
	} else {
		/* The request succeeded */
		ras->fastwrite_counter = 0;

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

/*
 * The RAS fast write path does not use the regular RAS message exchange but
 * instead uses a small FIFO to send address+value pairs, these writes are not
 * acknowledged and the request is completed immediately.
 *
 * The RAS fast write path is limited in size so a count is maintained of the
 * potential number of outstanding RAS fast writes, the count is cleared when
 * the device is reset and when a regular RAS message exchange is completed
 * successfully.
 *
 * When the fast path is full then write requests will be performed using
 * regular RAS message exchange, which will guarantee the fast write path is
 * drained. RAS fast writes are serviced before regular RAS messages so the
 * ordering of RAS requests is maintained.
 *
 * The exposed RAS regmap uses a mutex so the counter is protected and there
 * cannot be simultaneous RAS accesses of different types.
 */

/*
 * Setup the RAS structure members relating to fast writes and query whether
 * this particular device firmware supports the RAS fast write path.
 *
 * Not checking the return code of the message sending as the test fails safe
 * (response structure is cleared and the test depends on the bit being set)
 */
void clsic_ras_write_fastpath_init(struct clsic_ras_struct *ras)
{
	struct clsic *clsic;
	union clsic_ras_msg msg_cmd;
	union clsic_ras_msg msg_rsp;

	clsic = ras->clsic;

	ras->supports_fastwrites = false;
	ras->fastwrite_counter = 0;

	clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
			   ras->service->service_instance,
			   CLSIC_RAS_MSG_CR_GET_CAP);

	memset(&msg_rsp, 0, CLSIC_FIXED_MSG_SZ);

	clsic_send_msg_sync(clsic, (union t_clsic_generic_message *) &msg_cmd,
			    (union t_clsic_generic_message *) &msg_rsp,
			    CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
			    CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN);

	ras->supports_fastwrites =
		(msg_rsp.rsp_getcap.mask & CLSIC_RAS_CAP_FAST_WRITE) != 0;
}

/*
 * Perform this single register write thorough the fast path if there is space.
 */
static int clsic_ras_fastwrite(struct clsic_ras_struct *ras,
			       uint32_t address, uint32_t value)
{
	struct clsic *clsic;
	struct clsic_ras_fast_reg_write tmp_fastwrite;
	int ret;

	if (ras->fastwrite_counter > CLSIC_RAS_MAX_FASTWRITES)
		return -EBUSY;

	clsic = ras->clsic;

	tmp_fastwrite.reg_addr = address;
	tmp_fastwrite.reg_val = value;

	ret = regmap_raw_write(clsic->regmap, CLSIC_CPF2_RX_WRDATA,
			       &tmp_fastwrite, sizeof(tmp_fastwrite));

	++ras->fastwrite_counter;

	trace_clsic_ras_fastwrite(address, value, ret, ras->fastwrite_counter);

	return ret;
}

static int clsic_ras_simple_writeregister(struct clsic_ras_struct *ras,
					  uint32_t address, uint32_t value)
{
	struct clsic *clsic;
	union clsic_ras_msg msg_cmd;
	union clsic_ras_msg msg_rsp;
	int ret = 0;

	if (ras->suspended)
		return -EBUSY;

	if (ras->supports_fastwrites &&
	    (clsic_ras_fastwrite(ras, address, value) == 0))
		return 0;

	clsic = ras->clsic;

	/* Format and send a message to the remote access service */
	clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
			   ras->service->service_instance,
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
	 *  map errors to -EIO
	 */
	clsic_dbg(clsic, "ret: %d addr: 0x%x status %d\n", ret, address,
		  msg_rsp.rsp_wrreg.hdr.err);
	if ((ret != 0) || (msg_rsp.rsp_wrreg.hdr.err != 0))
		ret = -EIO;
	else /* The request succeeded */
		ras->fastwrite_counter = 0;

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

	if (ras->suspended)
		return -EBUSY;

	clsic = ras->clsic;

	if (val_size == CLSIC_RAS_VAL_BYTES)
		return clsic_ras_simple_readregister(ras, reg,
						     (__be32 *) val_buf);

	for (i = 0; i < val_size; i += CLSIC_RAS_MAX_BULK_SZ) {
		/* Format and send a message to the remote access service */
		clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
				   ras->service->service_instance,
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
		 *  codes so map errors to -EIO
		 */
		if (ret != 0)
			ret = -EIO;
		else if ((clsic_get_bulk_bit(msg_rsp.rsp_rdreg_bulk.hdr.sbc)
			  == 0) && (msg_rsp.rsp_rdreg_bulk.hdr.err != 0)) {
			err = msg_rsp.rsp_rdreg_bulk.hdr.err;
			ret = -EIO;
		} else if (msg_rsp.blkrsp_rdreg_bulk.hdr.err != 0) {
			err = msg_rsp.blkrsp_rdreg_bulk.hdr.err;
			ret = -EIO;
		} else
			/* The request succeeded */
			ras->fastwrite_counter = 0;

		clsic_dbg(clsic, "ret: %d addr: 0x%x err: %d\n", ret,
			  msg_cmd.cmd_rdreg_bulk.addr,
			  err);

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

	if (ras->suspended)
		return -EBUSY;

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
				   ras->service->service_instance,
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
		 *  codes so map errors to -EIO
		 */
		if ((ret != 0) || (msg_rsp.rsp_wrreg_bulk.hdr.err != 0)) {
			clsic_dbg(clsic, "addr: 0x%x status %d\n", addr,
				  msg_rsp.rsp_wrreg_bulk.hdr.err);
			ret = -EIO;
			goto error;
		}
		/* The request succeeded */
		ras->fastwrite_counter = 0;
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

	.max_register = TACNA_DSP2_SAMPLE_RATE_TX8,

	.readable_reg = &clsic_readable_register,
	.volatile_reg = &clsic_volatile_register,

	.name = "clsic-ras",
	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = clsic_reg_defaults,
	.num_reg_defaults = CLSIC_REG_DEFAULTS_SZ,
};

static struct regmap_config regmap_config_ras_dsp2 = {
	.reg_bits = CLSIC_RAS_REG_BITS,
	.val_bits = CLSIC_RAS_VAL_BITS,
	.reg_stride = CLSIC_RAS_REG_BYTES,

	.lock = &clsic_ras_regmap_lock,
	.unlock = &clsic_ras_regmap_unlock,

	.name = "clsic-ras-dsp2",
};

/*
 * This table specifies the sub devices supported by this bus - the kernel will
 * match up device drivers .names and call the driver probe() callbacks
 */
static struct mfd_cell clsic_devs[] = {
	{ .name = "clsic-tacna", }
};

/*
 * CLSIC_RAS_MSG_N_ERR_FAST_WRITE: The RAS service in the device will send a
 * notification if any of the writes are rejected - there is no recovery
 * action, simply log the information for analysis. (The address must have
 * passed the regmap whitelist exposed by the clsic driver so a fast write
 * error notification indicates that the firmware in the device has restricted
 * those register addresses)
 */
static int clsic_ras_nty_handler(struct clsic *clsic,
				 struct clsic_service *handler,
				 struct clsic_message *msg)
{
	union clsic_ras_msg *nty_msg = (union clsic_ras_msg *)msg;
	int ret = 0;

	switch (clsic_get_messageid(msg)) {
	case CLSIC_RAS_MSG_N_ERR_FAST_WRITE:
		clsic_err(clsic, "Fast write err: %d addr: 0x%x val 0x%x\n",
			  nty_msg->nty_err_fast_write.err,
			  nty_msg->nty_err_fast_write.reg_addr,
			  nty_msg->nty_err_fast_write.reg_val);
		ret = CLSIC_HANDLED;
	default:
		clsic_err(clsic, "unrecognised notification\n");
		clsic_dump_message(clsic, msg, "Unrecognised message");
	}
	return ret;
}

/*
 * When the device is suspended the exposed regmap is set so that the cache is
 * used for accesses and clients using the regmap can read and write values
 * without causing the device to switch on.
 *
 * When the device is resumed, the cache is recommited to the device and client
 * accesses cause RAS messages to be sent.
 */
static int clsic_ras_pm_handler(struct clsic_service *handler, int pm_event)
{
	struct clsic_ras_struct *ras;
	int ret = 0;

	/* Will always be populated when this handler could be called */
	ras = handler->data;

	switch (pm_event) {
	case PM_EVENT_SUSPEND:
		mutex_lock(&ras->regmap_mutex);
		ras->suspended = true;
		mutex_unlock(&ras->regmap_mutex);

		regcache_cache_only(ras->regmap, true);
		regcache_mark_dirty(ras->regmap);
		break;

	case PM_EVENT_RESUME:
		mutex_lock(&ras->regmap_mutex);
		ras->suspended = false;
		ras->fastwrite_counter = 0;
		mutex_unlock(&ras->regmap_mutex);

		regcache_cache_only(ras->regmap, false);
		ret = regcache_sync(ras->regmap);
		break;

	default:
		clsic_err(ras->clsic, "Unknown PM event %d",
			  pm_event);
		break;
	}

	trace_clsic_ras_pm_handler(pm_event);

	return ret;
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

		mutex_lock(&ras->regmap_mutex);
		ras->suspended = false;
		mutex_unlock(&ras->regmap_mutex);

		/*
		 * Mark dirty and switch off cache-only, the sync of the regmap
		 * committing the last known client state will be performed
		 * when PM_EVENT_RESUME is received.
		 */
		regcache_mark_dirty(ras->regmap);
		regcache_cache_only(ras->regmap, false);

		return 0;
	}

	ras = kzalloc(sizeof(*ras), GFP_KERNEL);
	if (ras == NULL)
		return -ENOMEM;


	handler->supports_debuginfo = true;
	handler->stop = &clsic_ras_stop;

	/*
	 * Set pm handler for RAS to manage reg-cache and a message callback
	 * for notifications
	 */
	handler->pm_handler = &clsic_ras_pm_handler;
	handler->callback = &clsic_ras_nty_handler;

	mutex_init(&ras->regmap_mutex);
	regmap_config_ras.lock_arg = ras;

	ras->clsic = clsic;
	handler->data = ras;
	ras->service = handler;
	ras->regmap = devm_regmap_init(clsic->dev,
				       &regmap_bus_ras,
				       ras,
				       &regmap_config_ras);
	if (IS_ERR(ras->regmap))
		return PTR_ERR(ras->regmap);

	/* DSP1 is always not accessible so setup a regmap for just DSP2 */
	ras->regmap_dsp[0] = NULL;
	regmap_config_ras_dsp2.lock_arg = ras;
	regmap_config_ras_dsp2.max_register =
		clsic_regmap_config_dsp2_template.max_register;
	regmap_config_ras_dsp2.rd_table =
		clsic_regmap_config_dsp2_template.rd_table;
	regmap_config_ras_dsp2.precious_table =
		clsic_regmap_config_dsp2_template.precious_table;

	ras->regmap_dsp[1] = devm_regmap_init(clsic->dev,
					      &regmap_bus_ras,
					      ras,
					      &regmap_config_ras_dsp2);
	if (IS_ERR(ras->regmap_dsp[1]))
		return PTR_ERR(ras->regmap_dsp[1]);

	clsic_ras_write_fastpath_init(ras);

	ras->suspended = false;

	clsic_dbg(clsic, "srv: %p regmap: %p\n",
		  ras, ras->regmap);

	/* RAS regmaps start in cache-only mode */
	regcache_cache_only(ras->regmap, true);

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
