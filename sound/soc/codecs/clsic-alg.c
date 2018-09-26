/*
 * clsic-alg.c -- ALSA SoC CLSIC ALGORITHM SERVICE
 *
 * Copyright (C) 2015-2018 Cirrus Logic, Inc. and
 *			   Cirrus Logic International Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/regmap.h>

#include <linux/mfd/tacna/core.h>
#include <linux/mfd/tacna/registers.h>
#include "tacna.h"
#include "wm_adsp.h"

#include <linux/mfd/clsic/core.h>
#include <linux/mfd/clsic/clsicmessagedefines_RAS.h>
#include "clsic-alg-msg.h"
#include "../../../drivers/mfd/clsic/clsic-trace.h"
#include <linux/mfd/clsic/message.h>
#include <linux/mfd/clsic/irq.h>

#define CLSIC_ALG_MAX_BULK_SZ  (CLSIC_FIFO_TRANSACTION_MAX / BITS_PER_BYTE)

#define CLSIC_ALG_REG_BITS	32
#define CLSIC_ALG_REG_BYTES	(CLSIC_ALG_REG_BITS/BITS_PER_BYTE)
#define CLSIC_ALG_VAL_BITS	32
#define CLSIC_ALG_VAL_BYTES	(CLSIC_ALG_VAL_BITS/BITS_PER_BYTE)

#define CLSIC_DSP1			0
#define CLSIC_DSP2			1
#define CLSIC_VPU1			2

#define CLSIC_DSP1_N_RX_CHANNELS	9
#define CLSIC_DSP1_N_TX_CHANNELS	9
#define CLSIC_DSP2_N_RX_CHANNELS	8
#define CLSIC_DSP2_N_TX_CHANNELS	8

#define CLSIC_DAI_CPU_VOICECTRL		"clsic-cpu-voicectrl"
#define CLSIC_DAI_VPU_VOICECTRL		"clsic-vpu-voicectrl"

static const struct wm_adsp_region clsic_alg_vpu_regions[] = {
	{ .type = WMFW_VPU_DM, .base = 0x20000000 },
};

static const struct wm_adsp_region clsic_dsp1_regions[] = {
	{ .type = WMFW_HALO_PM_PACKED, .base = 0x3800000 },
	{ .type = WMFW_HALO_XM_PACKED, .base = 0x2000000 },
	{ .type = WMFW_ADSP2_XM, .base = 0x2800000 },
	{ .type = WMFW_HALO_YM_PACKED, .base = 0x2C00000 },
	{ .type = WMFW_ADSP2_YM, .base = 0x3400000 },
};

static const struct wm_adsp_region clsic_dsp2_regions[] = {
	{ .type = WMFW_HALO_PM_PACKED, .base = 0x5800000 },
	{ .type = WMFW_HALO_XM_PACKED, .base = 0x4000000 },
	{ .type = WMFW_ADSP2_XM, .base = 0x4800000 },
	{ .type = WMFW_HALO_YM_PACKED, .base = 0x4C00000 },
	{ .type = WMFW_ADSP2_YM, .base = 0x5400000 },
};

struct clsic_alg_compr_stream {
	struct work_struct triggered;
	unsigned int event_id;
};

struct clsic_alg {
	/*
	 * wm_adsp struct must be the first element in codec private data
	 * because adsp driver will cast this private data to wm_adsp to handle
	 * dsp calls
	 */
	struct wm_adsp dsp[3];
	struct clsic *clsic;

	/* Instance specific information about a service handler */
	struct clsic_service *service;

	/* SoC Audio Codec device */
	struct snd_soc_codec *codec;

	struct regmap *regmap;
	struct mutex regmapMutex;

#ifdef CONFIG_DEBUG_FS
	struct dentry *rawMsgFile;
#endif
	struct mutex dspRateLock;
	struct clsic_alg_compr_stream compr_stream;
};

/**
 * clsic_alg_set_irq_notify_mode() - Set the notification modes for IRQs.
 * @alg:	The main instance of struct clsic_alg used in this driver.
 * @id:		Irq id.
 * @mode:	Irq mode.
 *
 * Return: 0 success, -EIO on error.
 */
static int clsic_alg_set_irq_notify_mode(struct clsic_alg *alg,
					 enum clsic_ras_irq_id id,
					 enum clsic_ras_irq_nty_mode mode)
{
	union clsic_ras_msg msg_cmd;
	union clsic_ras_msg msg_rsp;
	int ret;

	clsic_init_message((union t_clsic_generic_message *) &msg_cmd,
			   alg->service->service_instance,
			   CLSIC_RAS_MSG_CR_SET_IRQ_NTY_MODE);

	msg_cmd.cmd_set_irq_nty_mode.irq_id = id;
	msg_cmd.cmd_set_irq_nty_mode.mode = mode;

	ret = clsic_send_msg_sync(alg->clsic,
				  (union t_clsic_generic_message *) &msg_cmd,
				  (union t_clsic_generic_message *) &msg_rsp,
				  CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				  CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN);

	/*
	 *  Clients to this function can't interpret detailed error codes so
	 *  map error to -EIO
	 */
	if (ret != 0) {
		clsic_dbg(alg->clsic, "irq_id:0x%x mode:0x%x ret %d\n",
			  id, mode, ret);
		ret = -EIO;
	} else if (msg_rsp.rsp_set_irq_nty_mode.hdr.err != 0) {
		clsic_dbg(alg->clsic, "irq_id:0x%x mode:0x%x status %d\n",
			  id, mode, msg_rsp.rsp_set_irq_nty_mode.hdr.err);
		ret = -EIO;
	} else {
		/* The request succeeded */
		ret = 0;

		clsic_dbg(alg->clsic, "irq_id:0x%x mode:0x%x status %d\n",
			  id, mode, msg_rsp.rsp_set_irq_nty_mode.hdr.err);
	}

	trace_clsic_alg_set_irq_notify_mode(id, mode, ret,
					msg_rsp.rsp_set_irq_nty_mode.hdr.err);

	return ret;
}

/**
 * clsic_alg_simple_readregister() - Single 32bit word read over SPI
 * @alg:	The main instance of struct clsic_alg used in this driver.
 * @address:	Address of 32bit word to be read.
 * @value:	Pointer to address at which read value is returned
 *
 * Return: 0 success, -EIO on error.
 */
static int clsic_alg_simple_readregister(struct clsic_alg *alg,
					 uint32_t address, __be32 *value)
{
	union clsic_ras_msg msg_cmd;
	union clsic_ras_msg msg_rsp;
	int ret;

	clsic_init_message((union t_clsic_generic_message *) &msg_cmd,
			   alg->service->service_instance,
			   CLSIC_RAS_MSG_CR_RDREG);

	msg_cmd.cmd_rdreg.addr = address;

	/* Clear err to avoid confusion as it is always included in the trace */
	msg_rsp.rsp_wrreg.hdr.err = 0;

	ret = clsic_send_msg_sync_pm(alg->clsic,
				     (union t_clsic_generic_message *) &msg_cmd,
				     (union t_clsic_generic_message *) &msg_rsp,
				     CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				     CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN);

	/*
	 *  Clients to this function can't interpret detailed error codes so
	 *  map error to -EIO
	 */
	if (ret != 0) {
		clsic_dbg(alg->clsic, "0x%x ret %d\n", address, ret);
		ret = -EIO;
	} else if (msg_rsp.rsp_rdreg.hdr.err != 0) {
		clsic_dbg(alg->clsic, "addr: 0x%x status %d\n", address,
			  msg_rsp.rsp_rdreg.hdr.err);
		ret = -EIO;
	} else {
		/* The request succeeded */
		ret = 0;

		clsic_dbg(alg->clsic, "addr: 0x%x value: 0x%x status %d\n",
			  address,
			  msg_rsp.rsp_rdreg.value,
			  msg_rsp.rsp_rdreg.hdr.err);

		*value = cpu_to_be32(msg_rsp.rsp_rdreg.value);
	}

	trace_clsic_alg_simple_readregister(msg_cmd.cmd_rdreg.addr,
					    msg_rsp.rsp_rdreg.value, ret,
					    msg_rsp.rsp_rdreg.hdr.err);

	return ret;
}

/**
 * clsic_alg_simple_writeregister() - Single 32bit word write over SPI
 * @alg:	The main instance of struct clsic_alg used in this driver.
 * @address:	Address of 32bit word to be written.
 * @value:	Value to be written.
 *
 * Return: 0 success, -EIO on error.
 */
static int clsic_alg_simple_writeregister(struct clsic_alg *alg,
					  uint32_t address, uint32_t value)
{
	struct clsic *clsic;
	union clsic_ras_msg msg_cmd;
	union clsic_ras_msg msg_rsp;
	int ret = 0;

	if (alg == NULL)
		return -EINVAL;

	clsic = alg->clsic;

	/* Format and send a message to the remote access service */
	clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
			   alg->service->service_instance,
			   CLSIC_RAS_MSG_CR_WRREG);
	msg_cmd.cmd_wrreg.addr = address;
	msg_cmd.cmd_wrreg.value = value;

	/* Clear err to avoid confusion as it is always included in the trace */
	msg_rsp.rsp_wrreg.hdr.err = 0;

	ret = clsic_send_msg_sync_pm(clsic,
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

	trace_clsic_alg_simple_writeregister(msg_cmd.cmd_wrreg.addr,
					     msg_cmd.cmd_wrreg.value,
					     ret, msg_rsp.rsp_wrreg.hdr.err);
	return ret;
}

/**
 * clsic_alg_read() - Read data over SPI
 *
 * This function is called via the regmap_bus interface when a number of
 * sequential register reads are requested on the regmap, it has to iterate
 * through the request making individual register read requests.
 *
 * As with the sub functions it uses, the values passed to this function are in
 * the regmap bus order that we're exposing (big endian) so the function needs
 * to translate the address to read to the native cpu ordering before sending
 * on the request.
 *
 * If/when the register access service gets multiple register read/write
 * operations this could be made more efficient.
 *
 * Return: 0 success, -EIO on error.
 */
static int clsic_alg_read(void *context, const void *reg_buf,
			  const size_t reg_size, void *val_buf,
			  const size_t val_size)
{
	struct clsic *clsic;
	struct clsic_alg *alg = context;
	u32 reg = be32_to_cpu(*(const __be32 *) reg_buf);
	int ret = 0;
	size_t i;
	size_t frag_sz;
	union clsic_ras_msg msg_cmd;
	union clsic_ras_msg msg_rsp;
	uint8_t err = 0;

	if (alg == NULL)
		return -EINVAL;

	clsic = alg->clsic;

	if (val_size == CLSIC_ALG_VAL_BYTES)
		return clsic_alg_simple_readregister(alg, reg,
						     (__be32 *) val_buf);

	for (i = 0; i < val_size; i += CLSIC_ALG_MAX_BULK_SZ) {
		/* Format and send a message to the remote access service */
		clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
				   alg->service->service_instance,
				   CLSIC_RAS_MSG_CR_RDREG_BULK);
		frag_sz = min(val_size - i, (size_t) CLSIC_ALG_MAX_BULK_SZ);
		msg_cmd.cmd_rdreg_bulk.addr = reg + i;
		msg_cmd.cmd_rdreg_bulk.byte_count = frag_sz;

		ret = clsic_send_msg_sync_pm(
				    clsic,
				    (union t_clsic_generic_message *) &msg_cmd,
				    (union t_clsic_generic_message *) &msg_rsp,
				    CLSIC_NO_TXBUF, CLSIC_NO_TXBUF_LEN,
				    (uint8_t *)val_buf + i, frag_sz);

		/*
		 *  Clients to this function can't interpret detailed error
		 *  codes so map error to -EIO
		 */
		if (ret != 0) {
			clsic_dbg(clsic, "0x%x ret %d", reg, ret);
			ret = -EIO;
		} else if ((clsic_get_bulk_bit(msg_rsp.rsp_rdreg_bulk.hdr.sbc)
			    == 0) && (msg_rsp.rsp_rdreg_bulk.hdr.err != 0)) {
			clsic_dbg(clsic, "rsp addr: 0x%x status %d\n", reg,
				  msg_rsp.rsp_rdreg_bulk.hdr.err);
			err = msg_rsp.rsp_rdreg_bulk.hdr.err;
			ret = -EIO;
		} else if (msg_rsp.blkrsp_rdreg_bulk.hdr.err != 0) {
			clsic_dbg(clsic, "blkrsp addr: 0x%x status %d\n", reg,
				  msg_rsp.blkrsp_rdreg_bulk.hdr.err);
			err = msg_rsp.blkrsp_rdreg_bulk.hdr.err;
			ret = -EIO;
		} else {
			/* The request succeeded */
			ret = 0;
		}

		trace_clsic_alg_read(msg_cmd.cmd_rdreg_bulk.addr,
		     msg_cmd.cmd_rdreg_bulk.byte_count, ret, err);

		if (ret != 0)
			return ret;
	}
	/*
	 * The regmap bus is declared as BIG endian but all the
	 * accesses this service makes are CPU native so the value may
	 * need to be converted.
	 */
	for (i = 0; i < (val_size / CLSIC_ALG_VAL_BYTES); ++i)
		((__be32 *) val_buf)[i] = cpu_to_be32(((u32 *) val_buf)[i]);

	return 0;
}

/**
 * clsic_alg_write() - Write data over SPI
 *
 * This function is called via the regmap_bus interface when a number of
 * sequential register reads are requested on the regmap, it has to iterate
 * through the request making individual register write requests.
 *
 * As with the sub functions it uses, the values passed to this function are in
 * the regmap bus order that we're exposing (big endian) so the function needs
 * to translate the address to write to the native cpu ordering before sending
 * on the request.
 *
 * If/when the register access service gets multiple register read/write
 * operations this could be made more efficient.
 *
 * Return: 0 success, -EIO on error.
 */
static int clsic_alg_write(void *context, const void *val_buf,
			   const size_t val_size)
{
	struct clsic_alg *alg = context;
	struct clsic *clsic;
	const __be32 *buf = val_buf;
	u32 addr = be32_to_cpu(buf[0]);
	int ret = 0;
	size_t i;
	size_t payload_sz;
	size_t frag_sz;
	union clsic_ras_msg msg_cmd;
	union clsic_ras_msg msg_rsp;
	u32 *values;

	if (alg == NULL)
		return -EINVAL;

	clsic = alg->clsic;

	payload_sz = val_size - CLSIC_ALG_REG_BYTES;
	if ((val_size & (CLSIC_ALG_REG_BYTES - 1)) != 0) {
		clsic_err(clsic,
			  "error: context %p val_buf %p, val_size %d",
			  context, val_buf, val_size);
		clsic_err(clsic, "0x%x 0x%x 0x%x ",
			  buf[CLSIC_FSM0], buf[CLSIC_FSM1], buf[CLSIC_FSM2]);
		return -EIO;
	}

	if (val_size == (CLSIC_ALG_VAL_BYTES + CLSIC_ALG_REG_BYTES))
		return clsic_alg_simple_writeregister(alg, addr,
						      be32_to_cpu(buf[1]));

	values = kzalloc(payload_sz, GFP_KERNEL);
	if (values == NULL)
		return -ENOMEM;

	for (i = 1; i < (val_size / CLSIC_ALG_VAL_BYTES); ++i)
		values[i - 1] = be32_to_cpu(buf[i]);

	for (i = 0; i < payload_sz; i += CLSIC_ALG_MAX_BULK_SZ) {
		/* Format and send a message to the remote access service */
		clsic_init_message((union t_clsic_generic_message *)&msg_cmd,
				   alg->service->service_instance,
				   CLSIC_RAS_MSG_CR_WRREG_BULK);
		frag_sz = min(payload_sz - i, (size_t) CLSIC_ALG_MAX_BULK_SZ);
		msg_cmd.blkcmd_wrreg_bulk.addr = addr + i;
		msg_cmd.blkcmd_wrreg_bulk.hdr.bulk_sz = frag_sz;

		/*
		 * Clear err to avoid confusion as it is always included in the
		 * trace
		 */
		msg_rsp.rsp_wrreg.hdr.err = 0;

		ret = clsic_send_msg_sync_pm(
				    clsic,
				    (union t_clsic_generic_message *) &msg_cmd,
				    (union t_clsic_generic_message *) &msg_rsp,
				    ((const u8 *) values) + i, frag_sz,
				    CLSIC_NO_RXBUF, CLSIC_NO_RXBUF_LEN);

		trace_clsic_alg_write(msg_cmd.blkcmd_wrreg_bulk.addr,
				      msg_cmd.blkcmd_wrreg_bulk.hdr.bulk_sz,
				      ret, msg_rsp.rsp_wrreg_bulk.hdr.err);
		/*
		 *  Clients to this function can't interpret detailed error
		 *  codes so map error to -EIO
		 */
		if (ret != 0) {
			clsic_dbg(clsic, "0x%x ret %d", addr, ret);
			ret = -EIO;
		} else if (msg_rsp.rsp_wrreg_bulk.hdr.err != 0) {
			clsic_dbg(clsic, "addr: 0x%x status %d\n", addr,
				  msg_rsp.rsp_wrreg_bulk.hdr.err);
			ret = -EIO;
		} else {
			/* The request succeeded */
			ret = 0;
		}

		if (ret != 0)
			goto error;
	}

error:
	kfree(values);
	return ret;
}

/*
 * This function is called when a single register write is performed on the
 * regmap, it translates the context back into a clsic_alg structure so the
 * request can be sent through the messaging layer and fulfilled
 */
static int clsic_alg_reg_write(void *context, unsigned int reg, uint32_t val)
{
	struct clsic_alg *alg = context;

	return clsic_alg_simple_writeregister(alg, reg, val);
}

/*
 * This function is called when a single register read is performed on the
 * regmap, it translates the context back into a clsic_alg structure so the
 * request can be sent through the messaging layer and fulfilled
 */
static int clsic_alg_reg_read(void *context, unsigned int reg, uint32_t *val)
{
	struct clsic_alg *alg = context;

	return clsic_alg_simple_readregister(alg, reg, (__be32 *) val);
}

static int clsic_alg_gather_write(void *context, const void *reg,
				  size_t reg_len, const void *val,
				  size_t val_len)
{
	return -ENOTSUPP;
}

/*
 * The Algorithm service exposes a big endian regmap bus, but when we send
 * requests we are cpu native.
 */
static struct regmap_bus regmap_bus_alg = {
	.reg_write = &clsic_alg_reg_write,
	.reg_read = &clsic_alg_reg_read,
	.read = &clsic_alg_read,
	.write = &clsic_alg_write,
	.gather_write = &clsic_alg_gather_write,

	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

/*
 * Implement own regmap locking in order to silence lockdep
 * recursive lock warning.
 */
static void clsic_alg_regmap_lock(void *context)
{
	struct clsic_alg *alg = context;

	mutex_lock(&alg->regmapMutex);
}

static void clsic_alg_regmap_unlock(void *context)
{
	struct clsic_alg *alg = context;

	mutex_unlock(&alg->regmapMutex);
}

/**
 * clsic_alg_readable_register()
 *
 * Check if a register is defined as readable by the algorithm service
 *
 * @dev:	Pointer to device structure
 * @reg:	Register address to be checked
 *
 * Return: 0 non-readable, 1 readable.
 */
static bool clsic_alg_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x02000000 ... 0x0201dff0:	/* DSP1 XM packed */
	case 0x02400000 ... 0x02413ff8:	/* DSP1 XM unpacked32 */
	case 0x025e0000 ... 0x025e413c:	/* DSP1 system info registers */
	case 0x02800000 ... 0x02827ff4:	/* DSP1 XM unpacked24 */
	case 0x02b80000 ... 0x02b80050:	/* DSP1 control registers */
	case 0x02b805c0 ... 0x02b805d0:	/* DSP1 scratch registers */
	case 0x02bc1000 ... 0x02bcd020:	/* DSP1 control registers */
	case 0x02c00000 ... 0x02c17ff0:	/* DSP1 YM packed */
	case 0x03000000 ... 0x0300fff8:	/* DSP1 YM unpacked32 */
	case 0x03400000 ... 0x0341fff4:	/* DSP1 YM unpacked24 */
	case 0x04000000 ... 0x0401dff0:	/* DSP2 XM packed */
	case 0x04400000 ... 0x04413ff8:	/* DSP2 XM unpacked32 */
	case 0x045e0000 ... 0x045e413c:	/* DSP2 system info registers */
	case 0x04800000 ... 0x04827ff4:	/* DSP2 XM unpacked24 */
	case 0x04b80000 ... 0x04b80050:	/* DSP2 control registers */
	case 0x04b805c0 ... 0x04b805d0:	/* DSP2 scratch registers */
	case 0x04bc1000 ... 0x04bcd020:	/* DSP2 control registers */
	case 0x04c00000 ... 0x04c17ff0:	/* DSP2 YM packed */
	case 0x05000000 ... 0x0500fff8:	/* DSP2 YM unpacked32 */
	case 0x05400000 ... 0x0541fff4:	/* DSP2 YM unpacked24 */
	case 0x20000000 ... 0x2fffffff:	/* VPU */
	return true;
	default:
		return false;
	}
}

/*
 * regmap_config for the algorithm service
 */
static struct regmap_config regmap_config_alg = {
	.reg_bits = CLSIC_ALG_REG_BITS,
	.val_bits = CLSIC_ALG_VAL_BITS,
	.reg_stride = CLSIC_ALG_REG_BYTES,

	.lock = &clsic_alg_regmap_lock,
	.unlock = &clsic_alg_regmap_unlock,

	.readable_reg = &clsic_alg_readable_register,
	.cache_type = REGCACHE_NONE,
	.max_register = 0x2fffffff,

};

/*
 * The algorithm service custom message interface in debugfs allows messages
 * to be sent and responses to be received by test programs, it is not intended
 * to be an interface used in production devices.
 *
 * Transfers are currently limited to at most PAGE_SIZE, for both the fixed
 * size message and the payload.
 */
#ifdef CONFIG_DEBUG_FS
#define CLSIC_ALG_CUSTOM_MESSAGE_MAX	PAGE_SIZE
struct clsic_alg_cstm_msg {
	ssize_t len;
	uint8_t buf[CLSIC_ALG_CUSTOM_MESSAGE_MAX];
};

static int clsic_alg_custom_message_open(struct inode *inode, struct file *file)
{
	if (file->private_data == NULL) {
		file->private_data = kzalloc(sizeof(struct clsic_alg_cstm_msg),
						    GFP_KERNEL);
		if (file->private_data == NULL)
			return -ENOMEM;
	}

	return 0;
}

static int clsic_alg_custom_message_release(struct inode *inode,
					    struct file *file)
{
	kfree(file->private_data);
	file->private_data = NULL;

	return 0;
}

/* Let user space read back a message sent from the device to the host. */
static ssize_t clsic_alg_custom_message_read(struct file *file,
					 char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct clsic_alg *alg = file_inode(file)->i_private;
	struct clsic *clsic = alg->clsic;
	struct clsic_alg_cstm_msg *custom_msg = file->private_data;
	ssize_t ret;

	if (custom_msg == NULL)
		return -ENOMEM;

	if (custom_msg->buf == NULL)
		return -ENOMEM;

	ret = simple_read_from_buffer(user_buf, count, ppos,
				      custom_msg->buf, custom_msg->len);

	if (ret < 0)
		clsic_err(clsic, "simple_read_from_buffer error %zd", ret);

	return ret;
}

/* Send a custom fixed size message from host user space to the device. */
static ssize_t clsic_alg_custom_message_write(struct file *file,
					  const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct clsic_alg *alg = file_inode(file)->i_private;
	struct clsic *clsic = alg->clsic;
	struct clsic_alg_cstm_msg *custom_msg = file->private_data;
	char *buf;
	ssize_t ret;
	union t_clsic_generic_message *msg_p;
	char *txbuf = NULL;
	ssize_t txcount = 0;
	char *rxbuf;
	ssize_t rxcount;

	if (custom_msg == NULL)
		return -ENOMEM;

	buf = custom_msg->buf;
	if (buf == NULL)
		return -ENOMEM;

	/* The whole message must be written by the client in one go */
	if (count > CLSIC_ALG_CUSTOM_MESSAGE_MAX) {
		clsic_err(clsic, "Message too big: %zd (MAX %ld)\n",
			  count, CLSIC_ALG_CUSTOM_MESSAGE_MAX);
		return -EINVAL;
	}

	/* There must be at least one fixed size message to send */
	if (count < CLSIC_FIXED_MSG_SZ) {
		clsic_err(clsic, "Message too small: %zd (MIN %d)\n",
			  count, CLSIC_FIXED_MSG_SZ);
		return -EINVAL;
	}

	/* Copy in message from debugfs */
	ret = simple_write_to_buffer(buf, CLSIC_ALG_CUSTOM_MESSAGE_MAX,
				     ppos, user_buf, count);
	if (ret < 0) {
		clsic_err(clsic, "simple_write_to_buffer error %zd\n", ret);
		return ret;
	}

	if (count != ret) {
		clsic_err(clsic, "mismatch count %zd ret %zd\n", count, ret);
		return ret;
	}

	/*
	 * Populate the bulk receive parameters in case the FSM sent causes a
	 * bulk response
	 */
	rxbuf = &buf[CLSIC_FIXED_MSG_SZ];
	rxcount = CLSIC_ALG_CUSTOM_MESSAGE_MAX - CLSIC_FIXED_MSG_SZ;

	if (count != CLSIC_FIXED_MSG_SZ) {
		clsic_dbg(clsic, "%zd bytes in - bulk message.\n", count);
		txbuf = &buf[CLSIC_FIXED_MSG_SZ];
		txcount = count - CLSIC_FIXED_MSG_SZ;
	} else {
		clsic_dbg(clsic, "%zd bytes in - fsm.\n", count);
	}

	msg_p = (union t_clsic_generic_message *) buf;

	if (clsic_get_srv_inst(msg_p->cmd.hdr.sbc) !=
	    alg->service->service_instance) {
		clsic_err(clsic, "Service instance %d not supported\n",
			  clsic_get_srv_inst(msg_p->cmd.hdr.sbc));
		return -EINVAL;
	}
#ifdef CONFIG_DEBUG_FS
	mutex_lock(&alg->regmapMutex);
#endif
	ret = clsic_send_msg_sync_pm(clsic, msg_p, msg_p, txbuf, txcount,
				     rxbuf, rxcount);
#ifdef CONFIG_DEBUG_FS
	mutex_unlock(&alg->regmapMutex);
#endif
	/*
	 * Response gets stored in private_data automatically but we need to
	 * know the size of the response so that userspace can read it out. The
	 * size will vary depending on whether it is a fsm or bulk response.
	 */
	if (clsic_get_bulk_bit(msg_p->rsp.hdr.sbc) == 1) {
		clsic_dbg(clsic, "bulk ret %d err 0x%xp size 0x%x\n",
			  ret, msg_p->bulk_rsp.hdr.err,
			  msg_p->bulk_rsp.hdr.bulk_sz);
		custom_msg->len = CLSIC_FIXED_MSG_SZ +
				  msg_p->bulk_rsp.hdr.bulk_sz;
	} else {
		clsic_dbg(clsic, "fsm ret %d err 0x%x\n",
			  ret, msg_p->rsp.hdr.err);
		custom_msg->len = CLSIC_FIXED_MSG_SZ;
	}

	/*
	 * Propagate errors if there was a transfer error, otherwise the
	 * function should return the length of data read from user space.
	 *
	 * It is the responsibility of the caller to examine the FSM portion of
	 * the response to determine whether there was a failure at the
	 * protocol level.
	 */
	if (ret != 0)
		return ret;

	return count;
}

static const struct file_operations clsic_alg_custom_message_fops = {
	.read = &clsic_alg_custom_message_read,
	.write = &clsic_alg_custom_message_write,
	.release = &clsic_alg_custom_message_release,
	.open = &clsic_alg_custom_message_open,
	.llseek = &default_llseek,
};
#endif

/**
 * clsic_alg_init_dsps()
 *
 * Initialise DSP configurations
 *
 * @dev:	Pointer to device structure.
 * @alg:	The main instance of struct clsic_alg used in this driver.
 *
 * Return: 0 success
 */
static int clsic_alg_init_dsps(struct device *dev, struct clsic_alg *alg)
{
	struct wm_adsp *dsp;
	int ret;

	mutex_init(&alg->dspRateLock);

	/* DSP1 */
	dsp = &alg->dsp[CLSIC_DSP1];
	dsp->part = "cs48lv40";
	dsp->type = WMFW_HALO;
	dsp->num = 1;
	dsp->dev = dev;
	dsp->mem = clsic_dsp1_regions;
	dsp->regmap = alg->regmap;
	dsp->running = true;
	dsp->num_mems = ARRAY_SIZE(clsic_dsp1_regions);
	dsp->no_preloader = true;

	dsp->n_rx_channels = CLSIC_DSP1_N_RX_CHANNELS;
	dsp->n_tx_channels = CLSIC_DSP1_N_TX_CHANNELS;

	ret = wm_halo_init(dsp, &alg->dspRateLock);
	if (ret) {
		clsic_err(alg->clsic, "Failed to initialise DSP1\n");
		return ret;
	}

	dsp->n_rx_channels = 0;
	dsp->n_tx_channels = 0;

	/* DSP2 */
	dsp = &alg->dsp[CLSIC_DSP2];
	dsp->part = "cs48lv40";
	dsp->type = WMFW_HALO;
	dsp->num = 2;
	dsp->dev = dev;
	dsp->mem = clsic_dsp2_regions;
	dsp->regmap = alg->regmap;
	dsp->running = true;
	dsp->num_mems = ARRAY_SIZE(clsic_dsp2_regions);
	dsp->no_preloader = true;

	dsp->n_rx_channels = CLSIC_DSP2_N_RX_CHANNELS;
	dsp->n_tx_channels = CLSIC_DSP2_N_TX_CHANNELS;

	ret = wm_halo_init(dsp, &alg->dspRateLock);
	if (ret) {
		clsic_err(alg->clsic, "Failed to initialise DSP2\n");
		return ret;
	}

	/* Number of dsp2 channels set to 0 as under management of VPU core */
	dsp->n_rx_channels = 0;
	dsp->n_tx_channels = 0;

	/* VPU1 */
	dsp = &alg->dsp[CLSIC_VPU1];
	dsp->part = "cs48lv40";
	dsp->type = WMFW_VPU;
	dsp->name = "VPU1";
	dsp->num = 1;
	dsp->dev = dev;
	dsp->mem = clsic_alg_vpu_regions;
	dsp->regmap = alg->regmap;
	dsp->running = true;
	dsp->num_mems = ARRAY_SIZE(clsic_alg_vpu_regions);
	dsp->no_preloader = true;
	ret = wm_vpu_init(dsp);
	if (ret) {
		clsic_err(alg->clsic, "Failed to initialise VPU1\n");
		return ret;
	}

	return 0;
}

/**
 * clsic_alg_handle_n_irq() - handle irqs destined for the algorithm service
 * @alg:	The main instance of struct clsic_alg used in this driver.
 * @msg:	The message notification itself as received from CLSIC.
 *
 * This is a standard CLSIC function that will be called in the interrupt
 * handler context in the core messaging driver to irq notifications for the
 * algorithm service and react accordingly.
 *
 * Return: CLSIC_HANDLED or CLSIC_UNHANDLED.
 */
static int clsic_alg_handle_n_irq(struct clsic_alg *alg,
				  struct clsic_message *msg)
{
	int ret = CLSIC_UNHANDLED;
	union clsic_ras_msg *msg_nty = (union clsic_ras_msg *) &msg->fsm;
	unsigned int event_id;

	event_id = msg_nty->nty_irq.irq_id;

	switch (event_id) {
	case CLSIC_ALGOSRV_EVENT_VTE:
		alg->compr_stream.event_id = event_id;
		schedule_work(&alg->compr_stream.triggered);
		ret = CLSIC_HANDLED;
		break;
	default:
		clsic_err(alg->clsic, "Unhandled event %d\n", event_id);
		break;
	}

	trace_clsic_alg_handle_n_irq(event_id, ret);

	return ret;
}

/**
 * alg_notification_handler() - handle notifications destined for the algorithm
 *				service
 * @clsic:	The main shared instance of struct clsic used in the CLSIC
 *		drivers.
 * @handler:	The handler struct for algorithm service.
 * @msg:	The message notification itself as received from CLSIC.
 *
 * This is a standard CLSIC function that will be called in the interrupt
 * handler context in the core messaging driver to examine notifications for the
 * algorithm service and react accordingly.
 *
 * Return: CLSIC_HANDLED or CLSIC_UNHANDLED.
 */
static int clsic_alg_notification_handler(struct clsic *clsic,
				    struct clsic_service *handler,
				    struct clsic_message *msg)
{
	struct clsic_alg *alg = (struct clsic_alg *) handler->data;
	enum clsic_ras_msg_id msgid;
	int ret;

	msgid = clsic_get_messageid(msg);

	switch (msgid) {
	case CLSIC_RAS_MSG_N_IRQ:
		ret = clsic_alg_handle_n_irq(alg, msg);
		break;
	default:
		clsic_err(clsic, "unrecognised message with message ID %d\n",
			  msgid);
		ret = CLSIC_UNHANDLED;
	}

	return ret;
}

static struct snd_soc_dai_driver clsic_alg_dai[] = {
	{
		.name = CLSIC_DAI_CPU_VOICECTRL,
		.capture = {
			.stream_name = "Voice Trigger CPU",
			.channels_min = 1,
			.channels_max = 8,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
		.compress_new = &snd_soc_new_compress,
	},
	{
		.name = CLSIC_DAI_VPU_VOICECTRL,
		.capture = {
			.stream_name = "Voice Trigger VPU",
			.channels_min = 1,
			.channels_max = 8,
			.rates = TACNA_RATES,
			.formats = TACNA_FORMATS,
		},
	},
};

/**
 * clsic_alg_compr_open() - open the stream
 * @stream:	Standard parameter as used by compressed stream infrastructure.
 *
 * Standard .open function - see struct snd_compr_ops for more details.
 *
 * Return: errno.
 */
static int clsic_alg_compr_open(struct snd_compr_stream *stream)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct clsic_alg *alg = snd_soc_codec_get_drvdata(rtd->codec);
	int ret;

	clsic_dbg(alg->clsic, "%s\n", rtd->codec_dai->name);

	ret = clsic_alg_set_irq_notify_mode(alg,
					   CLSIC_ALGOSRV_EVENT_VTE,
					   CLSIC_RAS_NTY_FLUSH_AND_REQ);

	if (ret) {
		clsic_err(alg->clsic,
			  "Set notify mode for DAI '%s' failed %d\n",
			  rtd->codec_dai->name, ret);
		return ret;
	}

	ret = wm_adsp_compr_open(&alg->dsp[CLSIC_VPU1], stream);

	if (ret)
		clsic_err(alg->clsic,
			  "Open compr stream for DAI '%s' failed %d\n",
			  rtd->codec_dai->name, ret);

	trace_clsic_alg_compr_stream_open(stream->direction, ret);

	return ret;
}

/**
 * clsic_alg_compr_free() - close the stream
 * @stream:	Standard parameter as used by compressed stream infrastructure.
 *
 * Standard .free function - see struct snd_compr_ops for more details.
 *
 * Return: errno.
 */
static int clsic_alg_compr_free(struct snd_compr_stream *stream)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct clsic_alg *alg = snd_soc_codec_get_drvdata(rtd->codec);
	int ret;

	clsic_dbg(alg->clsic, "%s\n", rtd->codec_dai->name);

	ret = clsic_alg_set_irq_notify_mode(alg,
					   CLSIC_ALGOSRV_EVENT_VTE,
					   CLSIC_RAS_NTY_CANCEL);

	if (ret)
		clsic_err(alg->clsic,
			  "Cancel notify mode for DAI '%s' failed %d\n",
			  rtd->codec_dai->name, ret);

	wm_adsp_compr_free(stream);

	trace_clsic_alg_compr_stream_free(stream->direction, ret);

	return ret;
}

/**
 * clsic_alg_compr_set_params() - set up internal stream parameters
 * @stream:	Standard parameter as used by compressed stream infrastructure.
 * @params:	Standard parameter as used by compressed stream infrastructure.
 *
 * Standard .set_params function - see struct snd_compr_ops for more details.
 *
 * Return: errno.
 */
static int clsic_alg_compr_set_params(struct snd_compr_stream *stream,
				      struct snd_compr_params *params)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct clsic_alg *alg = snd_soc_codec_get_drvdata(rtd->codec);
	int ret;

	clsic_dbg(alg->clsic, "%s\n", rtd->codec_dai->name);

	ret = wm_adsp_compr_set_params(stream, params);

	if (ret)
		clsic_err(alg->clsic,
			  "Set compr stream param '%s' failed %d\n",
			  rtd->codec_dai->name, ret);

	trace_clsic_alg_compr_stream_set_params(params, ret);

	return ret;
}

/**
 * clsic_alg_compr_get_caps() - copy stream data to userspace buffer
 * @stream:	Standard parameter as used by compressed stream infrastructure.
 * @caps:	Standard parameter as used by compressed stream infrastructure.
 *
 * Standard .get_caps function - see struct snd_compr_ops for more details.
 *
 * Return: errno.
 */
static int clsic_alg_compr_get_caps(struct snd_compr_stream *stream,
				    struct snd_compr_caps *caps)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct clsic_alg *alg = snd_soc_codec_get_drvdata(rtd->codec);
	int ret;

	clsic_dbg(alg->clsic, "%s\n", rtd->codec_dai->name);

	ret = wm_adsp_compr_get_caps(stream, caps);

	if (ret)
		clsic_err(alg->clsic,
			  "Get compr stream caps '%s' failed %d\n",
			  rtd->codec_dai->name, ret);

	trace_clsic_alg_compr_stream_get_caps(caps, ret);

	return ret;
}

/**
 * clsic_alg_compr_triggered() - worker thread handling compressed stream events
 *
 * The irq context can't be used as it would block the messaging thread.
 */
static void clsic_alg_compr_triggered(struct work_struct *data)
{
	struct clsic_alg_compr_stream *compr_stream =
		container_of(data, struct clsic_alg_compr_stream, triggered);
	struct clsic_alg *alg =
		container_of(compr_stream, struct clsic_alg, compr_stream);

	wm_adsp_compr_handle_irq(&alg->dsp[CLSIC_VPU1]);

	clsic_alg_set_irq_notify_mode(alg, compr_stream->event_id,
				      CLSIC_RAS_NTY_FLUSH_AND_REQ);
}

/**
 * clsic_alg_compr_trigger() - respond to userspace
 * @stream:	Standard parameter as used by compressed stream infrastructure.
 * @cmd:	A start or stop flag for compressed audio streaming.
 *
 * Standard .trigger function - see struct snd_compr_ops for more details. When
 * userspace (crec) starts reading an active compressed stream of audio, this
 * function is called with a relevant command regarding whether the stream has
 * just started or just stopped.
 *
 * Return: errno.
 */
static int clsic_alg_compr_trigger(struct snd_compr_stream *stream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct clsic_alg *alg = snd_soc_codec_get_drvdata(rtd->codec);
	int ret;

	clsic_dbg(alg->clsic, "%s %d\n", rtd->codec_dai->name, cmd);

	if (strcmp(rtd->codec_dai->name, CLSIC_DAI_VPU_VOICECTRL) == 0) {
		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			break;
		case SNDRV_PCM_TRIGGER_STOP:
			cancel_work_sync(&alg->compr_stream.triggered);
			break;
		default:
			ret = -EINVAL;
			clsic_err(alg->clsic,
				  "Trigger compr stream '%s' failed %d\n",
				  rtd->codec_dai->name, ret);
			break;
		}

		ret = wm_adsp_compr_trigger(stream, cmd);

		if (ret)
			clsic_err(alg->clsic,
				  "Trigger compr stream '%s' failed %d\n",
				  rtd->codec_dai->name, ret);
	} else {
		clsic_err(alg->clsic,
			  "No suitable compressed stream for DAI '%s'\n",
			  rtd->codec_dai->name);

		ret = -EINVAL;
	}

	trace_clsic_alg_compr_stream_trigger(cmd, ret);

	return ret;
}

/**
 * clsic_alg_compr_pointer() - get timestamp information about the ASR stream
 * @stream:	Standard parameter as used by compressed stream infrastructure.
 * @tstamp:	Standard parameter as used by compressed stream infrastructure.
 *
 * Standard .pointer function - see struct snd_compr_ops for more details.
 *
 * Return: 0 always.
 */
static int clsic_alg_compr_pointer(struct snd_compr_stream *stream,
				   struct snd_compr_tstamp *tstamp)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct clsic_alg *alg = snd_soc_codec_get_drvdata(rtd->codec);
	int ret;

	clsic_dbg(alg->clsic, "%s\n", rtd->codec_dai->name);

	if (strcmp(rtd->codec_dai->name, CLSIC_DAI_VPU_VOICECTRL) == 0) {
		ret = wm_adsp_compr_pointer(stream, tstamp);

		if (ret)
			clsic_err(alg->clsic,
				  "Set compr stream pointer '%s' failed %d\n",
				   rtd->codec_dai->name, ret);
	} else {
		clsic_err(alg->clsic, "No suitable compr stream for DAI '%s'\n",
				       rtd->codec_dai->name);

		ret = -EINVAL;
	}

	trace_clsic_alg_compr_stream_timestamp(tstamp, ret);

	return ret;
}

/**
 * clsic_alg_compr_copy() - copy data to userspace buffer
 * @stream:	Standard parameter as used by compressed stream infrastructure.
 * @buf:	Userspace buffer to copy compressed data to.
 * @count:	How many bytes to copy to userspace.
 *
 * Standard .copy function - see struct snd_compr_ops for more details.
 *
 * Return: errno.
 */
static int clsic_alg_compr_copy(struct snd_compr_stream *stream,
				char __user *buf, size_t count)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct clsic_alg *alg = snd_soc_codec_get_drvdata(rtd->codec);
	int ret;

	clsic_dbg(alg->clsic, "%s\n", rtd->codec_dai->name);

	if (strcmp(rtd->codec_dai->name, CLSIC_DAI_VPU_VOICECTRL) == 0) {
		ret = wm_adsp_compr_copy(stream, buf, count);
	} else {
		clsic_err(alg->clsic,
			  "No suitable compressed stream for DAI '%s'\n",
			  rtd->codec_dai->name);

		ret = -EINVAL;
	}

	trace_clsic_alg_compr_stream_copy_start(count, ret);

	return ret;
}

/**
 * clsic_alg_codec_probe() - probe function for the codec part of the driver
 * @codec:	The main shared instance of struct snd_soc_codec used in CLSIC.
 *
 * Initialise ALSA controls on VPU and DSPs
 *
 * Return: 0 success
 */
static int clsic_alg_codec_probe(struct snd_soc_codec *codec)
{
	struct clsic_alg *alg = snd_soc_codec_get_drvdata(codec);
	struct clsic_service *handler = alg->service;

	alg->codec = codec;
	handler->data = (void *)alg;
	handler->callback = &clsic_alg_notification_handler;

	INIT_WORK(&alg->compr_stream.triggered, clsic_alg_compr_triggered);

	wm_adsp2_codec_probe(&alg->dsp[0], codec);
	wm_adsp2_codec_probe(&alg->dsp[1], codec);
	wm_adsp2_codec_probe(&alg->dsp[2], codec);

	return 0;
}

/**
 * clsic_alg_codec_remove() - remove function for the codec part of the driver
 * @codec:	The main shared instance of struct snd_soc_codec used in CLSIC.
 *
 * Return: 0 success
 */
static int clsic_alg_codec_remove(struct snd_soc_codec *codec)
{
	struct clsic_alg *alg = snd_soc_codec_get_drvdata(codec);

	alg->service->callback = NULL;

	wm_adsp2_codec_remove(&alg->dsp[0], codec);
	wm_adsp2_codec_remove(&alg->dsp[1], codec);
	wm_adsp2_codec_remove(&alg->dsp[2], codec);

	return 0;
}

/**
 * clsic_alg_dapm_widgets[] - Here we define 3 widgets that will be created in
 * the card creation, "DSP1 Preloader", "DSP2 Preloader", "VPU1 Preloader".
 * Being a supply widget, and connect as a source to the main DSP widget, these
 * widgets will be enabled when the main DSP are enabled, calling the event
 * function set here.
 */
static const struct snd_soc_dapm_widget clsic_alg_dapm_widgets[] = {
	WM_ADSP_PRELOADER("DSP1", 0, wm_halo_early_event),
	WM_ADSP_PRELOADER("DSP2", 1, wm_halo_early_event),
	WM_ADSP_PRELOADER("VPU1", 2, wm_halo_early_event),
};

static const struct snd_kcontrol_new clsic_alg_snd_controls[] = {
	WM_ADSP_FW_CONTROL("DSP1", 0),
	WM_ADSP_FW_CONTROL("DSP2", 1),
	WM_ADSP_FW_CONTROL("VPU1", 2),
};

static const struct snd_soc_codec_driver soc_codec_clsic_alg = {
	.probe = clsic_alg_codec_probe,
	.remove = clsic_alg_codec_remove,

	.component_driver = {
		.controls = clsic_alg_snd_controls,
		.num_controls = ARRAY_SIZE(clsic_alg_snd_controls),
		.dapm_widgets = clsic_alg_dapm_widgets,
		.num_dapm_widgets = ARRAY_SIZE(clsic_alg_dapm_widgets),
	},
};

static const struct snd_compr_ops clsic_alg_compr_ops = {
	.open = &clsic_alg_compr_open,
	.free = &clsic_alg_compr_free,
	.set_params = &clsic_alg_compr_set_params,
	.get_caps = &clsic_alg_compr_get_caps,
	.trigger = &clsic_alg_compr_trigger,
	.pointer = &clsic_alg_compr_pointer,
	.copy = &clsic_alg_compr_copy,
};

static const struct snd_soc_platform_driver clsic_alg_compr_platform = {
	.compr_ops = &clsic_alg_compr_ops,
};

/**
 * clsic_alg_probe() - probe function for the module
 * @pdev:	Platform device struct.
 *
 * Standard module probe function.
 *
 * Return: errno.
 */
static int clsic_alg_probe(struct platform_device *pdev)
{
	struct clsic *clsic = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct clsic_service *clsic_service = dev_get_platdata(dev);
	struct clsic_alg *alg;
	int ret;

	BUILD_BUG_ON(ARRAY_SIZE(clsic_alg_dai) > TACNA_MAX_DAI);

	/* Allocate memory for device specific data */
	alg = devm_kzalloc(dev, sizeof(struct clsic_alg), GFP_KERNEL);
	if (alg == NULL)
		return -ENOMEM;

	/* Populate device specific data struct */
	alg->clsic = clsic;
	alg->service = clsic->service_handlers[clsic_service->service_instance];

#ifdef CONFIG_DEBUG_FS
	alg->rawMsgFile = debugfs_create_file("alg_raw_message", 0600,
					      clsic->debugfs_root, alg,
					      &clsic_alg_custom_message_fops);
#endif
	mutex_init(&alg->regmapMutex);
	regmap_config_alg.lock_arg = alg;

	/* Set device specific data */
	platform_set_drvdata(pdev, alg);

	alg->regmap = devm_regmap_init(dev, &regmap_bus_alg, alg,
				       &regmap_config_alg);
	if (IS_ERR(alg->regmap)) {
		ret = PTR_ERR(alg->regmap);
		clsic_err(clsic, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	ret = clsic_alg_init_dsps(dev, alg);
	if (ret) {
		clsic_err(clsic, "Failed to init dsps: %d.\n", ret);
		return ret;
	}

	ret = snd_soc_register_platform(dev, &clsic_alg_compr_platform);
	if (ret < 0) {
		clsic_err(clsic, "Failed to register platform: %d.\n", ret);
		return ret;
	}

	/* Register codec with the ASoC core */
	ret = snd_soc_register_codec(dev, &soc_codec_clsic_alg, clsic_alg_dai,
				     ARRAY_SIZE(clsic_alg_dai));
	if (ret) {
		clsic_err(clsic, "Failed to register codec: %d.\n", ret);
		snd_soc_unregister_platform(dev);
	}

	return ret;
}

/**
 * clsic_alg_remove() - remove function for the module
 * @pdev:	Platform device struct.
 *
 * Standard module remove function, removes debugfs interface
 *
 * Return: errno.
 */
static int clsic_alg_remove(struct platform_device *pdev)
{
	struct clsic_alg *alg = platform_get_drvdata(pdev);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove(alg->rawMsgFile);
#endif
	snd_soc_unregister_platform(&pdev->dev);
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static const struct of_device_id clsic_alg_of_match[] = {
	{ .compatible = "cirrus,clsic-alg", },
	{},
};
MODULE_DEVICE_TABLE(of, clsic_alg_of_match);

static struct platform_driver clsic_alg_driver = {
	.driver = {
		.name = "clsic-alg",
		.of_match_table = clsic_alg_of_match,
	},
	.probe = clsic_alg_probe,
	.remove = clsic_alg_remove,
};

module_platform_driver(clsic_alg_driver);

MODULE_DESCRIPTION("ASoC Cirrus Logic CLSIC Algorithm Service");
MODULE_AUTHOR("Andrew Ford <andrew.ford@cirrus.com>");
MODULE_AUTHOR("Lucas Tanure <tanureal@opensource.cirrus.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:clsic-alg");
