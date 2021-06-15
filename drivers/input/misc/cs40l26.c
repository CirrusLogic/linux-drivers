// SPDX-License-Identifier: GPL-2.0
//
// cs40l26.c -- CS40L26 Boosted Haptic Driver with Integrated DSP and
// Waveform Memory with Advanced Closed Loop Algorithms and LRA protection
//
// Copyright 2021 Cirrus Logic, Inc.
//
// Author: Fred Treven <fred.treven@cirrus.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation.

#include <linux/mfd/cs40l26.h>

static int cs40l26_dsp_read(struct cs40l26_private *cs40l26, u32 reg, u32 *val)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	int ret, i;
	u32 read_val;

	for (i = 0; i < CS40L26_DSP_TIMEOUT_COUNT; i++) {
		ret = regmap_read(regmap, reg, &read_val);
		if (ret)
			dev_dbg(dev, "Failed to read 0x%X, attempt(s) = %d\n",
					reg, i + 1);
		else
			break;

		usleep_range(CS40L26_DSP_TIMEOUT_US_MIN,
				CS40L26_DSP_TIMEOUT_US_MAX);
	}

	if (i >= CS40L26_DSP_TIMEOUT_COUNT) {
		dev_err(dev, "Timed out attempting to read 0x%X\n", reg);
		return -ETIME;
	}

	*val = read_val;

	return 0;
}

static int cs40l26_dsp_write(struct cs40l26_private *cs40l26, u32 reg, u32 val)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	int ret, i;

	for (i = 0; i < CS40L26_DSP_TIMEOUT_COUNT; i++) {
		ret = regmap_write(regmap, reg, val);
		if (ret)
			dev_dbg(dev,
				"Failed to write to 0x%X, attempt(s) = %d\n",
				reg, i + 1);
		else
			break;

		usleep_range(CS40L26_DSP_TIMEOUT_US_MIN,
				CS40L26_DSP_TIMEOUT_US_MAX);
	}

	if (i >= CS40L26_DSP_TIMEOUT_COUNT) {
		dev_err(dev, "Timed out attempting to write to 0x%X\n", reg);
		return -ETIME;
	}

	return 0;
}

static int cs40l26_ack_read(struct cs40l26_private *cs40l26, u32 reg,
		u32 ack_val)
{
	struct device *dev = cs40l26->dev;
	int ret, i;
	u32 val;

	for (i = 0; i < CS40L26_DSP_TIMEOUT_COUNT; i++) {
		ret = cs40l26_dsp_read(cs40l26, reg, &val);
		if (ret)
			return ret;

		if (val != ack_val)
			dev_dbg(dev, "Ack'ed value not equal to expected\n");
		else
			break;

		usleep_range(CS40L26_DSP_TIMEOUT_US_MIN,
				CS40L26_DSP_TIMEOUT_US_MAX);
	}

	if (i >= CS40L26_DSP_TIMEOUT_COUNT) {
		dev_err(dev, "Ack timed out (0x%08X != 0x%08X) reg. 0x%08X\n",
				val, ack_val, reg);
		return -ETIME;
	}

	return 0;
}

int cs40l26_ack_write(struct cs40l26_private *cs40l26, u32 reg, u32 write_val,
		u32 reset_val)
{
	int ret;

	ret = cs40l26_dsp_write(cs40l26, reg, write_val);
	if (ret)
		return ret;

	return cs40l26_ack_read(cs40l26, reg, reset_val);
}
EXPORT_SYMBOL(cs40l26_ack_write);

int cs40l26_class_h_set(struct cs40l26_private *cs40l26, bool class_h)
{
	int ret;

	ret = regmap_update_bits(cs40l26->regmap, CS40L26_BLOCK_ENABLES2,
			CS40L26_CLASS_H_EN_MASK, class_h <<
			CS40L26_CLASS_H_EN_SHIFT);
	if (ret)
		dev_err(cs40l26->dev, "Failed to update CLASS H tracking\n");

	return ret;
}
EXPORT_SYMBOL(cs40l26_class_h_set);

int cs40l26_dsp_state_get(struct cs40l26_private *cs40l26, u8 *state)
{
	u32 algo_id, reg, dsp_state;
	int ret;

	if (cs40l26->fw_loaded) {
		if (cs40l26->fw_mode == CS40L26_FW_MODE_RAM)
			algo_id = CS40L26_PM_ALGO_ID;
		else
			algo_id = CS40L26_PM_ROM_ALGO_ID;

		ret = cl_dsp_get_reg(cs40l26->dsp, "PM_CUR_STATE",
				CL_DSP_XM_UNPACKED_TYPE, algo_id, &reg);
		if (ret)
			return ret;
	} else {
		switch (cs40l26->revid) {
		case CS40L26_REVID_A0:
			reg = CS40L26_A0_PM_CUR_STATE_STATIC_REG;
			break;
		case CS40L26_REVID_A1:
			reg = CS40L26_A1_PM_CUR_STATE_STATIC_REG;
			break;
		default:
			dev_err(cs40l26->dev,
					"Revid ID not supported: 0x%02X\n",
					cs40l26->revid);
			return -EINVAL;
		}
	}

	ret = cs40l26_dsp_read(cs40l26, reg, &dsp_state);
	if (ret)
		return ret;

	switch (dsp_state) {
	case CS40L26_DSP_STATE_HIBERNATE:
		/* intentionally fall through */
	case CS40L26_DSP_STATE_SHUTDOWN:
		/* intentionally fall through */
	case CS40L26_DSP_STATE_STANDBY:
		/* intentionally fall through */
	case CS40L26_DSP_STATE_ACTIVE:
		*state = CS40L26_DSP_STATE_MASK & dsp_state;
		break;
	default:
		dev_err(cs40l26->dev, "DSP state %u is invalid\n", dsp_state);
		ret = -EINVAL;
	}

	return ret;
}
EXPORT_SYMBOL(cs40l26_dsp_state_get);

int cs40l26_pm_timeout_ms_set(struct cs40l26_private *cs40l26,
		u32 timeout_ms)
{
	u32 timeout_ticks = timeout_ms * CS40L26_PM_TICKS_MS_DIV;
	struct regmap *regmap = cs40l26->regmap;
	u32 lower_val, reg, algo_id;
	u8 upper_val;
	int ret;

	upper_val = (timeout_ticks >> CS40L26_PM_TIMEOUT_TICKS_UPPER_SHIFT) &
			CS40L26_PM_TIMEOUT_TICKS_UPPER_MASK;

	lower_val = timeout_ticks & CS40L26_PM_TIMEOUT_TICKS_LOWER_MASK;

	if (cs40l26->fw_mode == CS40L26_FW_MODE_RAM)
		algo_id = CS40L26_PM_ALGO_ID;
	else
		algo_id = CS40L26_PM_ROM_ALGO_ID;

	ret = cl_dsp_get_reg(cs40l26->dsp, "PM_TIMER_TIMEOUT_TICKS",
			CL_DSP_XM_UNPACKED_TYPE, algo_id, &reg);
	if (ret)
		return ret;

	ret = regmap_write(regmap, reg + CS40L26_PM_STDBY_TIMEOUT_LOWER_OFFSET,
			lower_val);
	if (ret)
		return ret;

	return regmap_write(regmap, reg + CS40L26_PM_STDBY_TIMEOUT_UPPER_OFFSET,
			upper_val);
}
EXPORT_SYMBOL(cs40l26_pm_timeout_ms_set);

int cs40l26_pm_timeout_ms_get(struct cs40l26_private *cs40l26,
		u32 *timeout_ms)
{
	u32 lower_val, upper_val, algo_id, reg;
	int ret;

	if (cs40l26->fw_loaded) {
		if (cs40l26->fw_mode == CS40L26_FW_MODE_RAM)
			algo_id = CS40L26_PM_ALGO_ID;
		else
			algo_id = CS40L26_PM_ROM_ALGO_ID;

		ret = cl_dsp_get_reg(cs40l26->dsp, "PM_TIMER_TIMEOUT_TICKS",
				CL_DSP_XM_UNPACKED_TYPE, algo_id, &reg);
		if (ret)
			return ret;
	} else {
		switch (cs40l26->revid) {
		case CS40L26_REVID_A0:
			reg = CS40L26_A0_PM_TIMEOUT_TICKS_STATIC_REG;
			break;
		case CS40L26_REVID_A1:
			reg = CS40L26_A1_PM_TIMEOUT_TICKS_STATIC_REG;
			break;
		default:
			dev_err(cs40l26->dev, "Revid ID not supported: %02X\n",
				cs40l26->revid);
			return -EINVAL;
		}
	}

	ret = regmap_read(cs40l26->regmap, reg +
			CS40L26_PM_STDBY_TIMEOUT_LOWER_OFFSET, &lower_val);
	if (ret)
		return ret;

	ret = regmap_read(cs40l26->regmap, reg +
			CS40L26_PM_STDBY_TIMEOUT_UPPER_OFFSET, &upper_val);
	if (ret)
		return ret;

	*timeout_ms =
		((lower_val & CS40L26_PM_TIMEOUT_TICKS_LOWER_MASK) |
		((upper_val & CS40L26_PM_TIMEOUT_TICKS_UPPER_MASK) <<
		CS40L26_PM_TIMEOUT_TICKS_UPPER_SHIFT)) /
		CS40L26_PM_TICKS_MS_DIV;

	return 0;
}
EXPORT_SYMBOL(cs40l26_pm_timeout_ms_get);

static int cs40l26_pm_runtime_setup(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	int ret;

	ret = cs40l26_pm_timeout_ms_set(cs40l26, CS40L26_PM_TIMEOUT_MS_MIN);
	if (ret)
		return ret;

	pm_runtime_mark_last_busy(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, CS40L26_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);

	cs40l26->pm_ready = true;

	return 0;
}

static void cs40l26_pm_runtime_teardown(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;

	pm_runtime_set_suspended(dev);
	pm_runtime_disable(dev);
	pm_runtime_dont_use_autosuspend(dev);

	cs40l26->pm_ready = false;
}

int cs40l26_pm_state_transition(struct cs40l26_private *cs40l26,
		enum cs40l26_pm_state state)
{
	struct device *dev = cs40l26->dev;
	u32 cmd;
	int ret;

	cmd = (u32) CS40L26_DSP_MBOX_PM_CMD_BASE + state;

	switch (state) {
	case CS40L26_PM_STATE_WAKEUP:
	/* intentionally fall through */
	case CS40L26_PM_STATE_PREVENT_HIBERNATE:
		ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				cmd, CS40L26_DSP_MBOX_RESET);
		break;
	case CS40L26_PM_STATE_ALLOW_HIBERNATE:
	/* intentionally fall through */
	case CS40L26_PM_STATE_SHUTDOWN:
		cs40l26->wksrc_sts = 0x00;
		ret = cs40l26_dsp_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				cmd);
		break;
	default:
		dev_err(dev, "Invalid PM state: %u\n", state);
		return -EINVAL;
	}

	if (ret)
		return ret;

	cs40l26->pm_state = state;

	return 0;
}

static int cs40l26_dsp_start(struct cs40l26_private *cs40l26)
{
	u8 dsp_state;
	int ret;

	ret = regmap_write(cs40l26->regmap, CS40L26_DSP1_CCM_CORE_CONTROL,
			CS40L26_DSP_CCM_CORE_RESET);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to reset DSP core\n");
		return ret;
	}

	ret = cs40l26_dsp_state_get(cs40l26, &dsp_state);
	if (ret)
		return ret;

	if (dsp_state != CS40L26_DSP_STATE_ACTIVE &&
			dsp_state != CS40L26_DSP_STATE_STANDBY) {
		dev_err(cs40l26->dev, "Failed to wake DSP core\n");
		return -EINVAL;
	}

	return 0;
}

static int cs40l26_dsp_wake(struct cs40l26_private *cs40l26)
{
	u8 dsp_state;
	int ret;

	ret = cs40l26_pm_state_transition(cs40l26, CS40L26_PM_STATE_WAKEUP);
	if (ret)
		return ret;

	ret = cs40l26_pm_state_transition(cs40l26,
			CS40L26_PM_STATE_PREVENT_HIBERNATE);
	if (ret)
		return ret;

	ret = cs40l26_dsp_state_get(cs40l26, &dsp_state);
	if (ret)
		return ret;

	if (dsp_state != CS40L26_DSP_STATE_STANDBY &&
			dsp_state != CS40L26_DSP_STATE_ACTIVE) {
		dev_err(cs40l26->dev, "Failed to wake DSP\n");
		return -EINVAL;
	}

	return 0;
}

static int cs40l26_dsp_shutdown(struct cs40l26_private *cs40l26)
{
	u32 timeout_ms;
	int ret;

	ret = cs40l26_pm_timeout_ms_get(cs40l26, &timeout_ms);
	if (ret)
		return ret;

	ret = cs40l26_pm_state_transition(cs40l26, CS40L26_PM_STATE_SHUTDOWN);
	if (ret)
		return ret;

	usleep_range(CS40L26_MS_TO_US(timeout_ms),
			CS40L26_MS_TO_US(timeout_ms) + 100);

	return 0;
}

static int cs40l26_dsp_pre_config(struct cs40l26_private *cs40l26)
{
	u8 dsp_state;
	u32 halo_state, halo_state_reg;
	int ret;

	switch (cs40l26->revid) {
	case CS40L26_REVID_A0:
		halo_state_reg = CS40L26_A0_DSP_HALO_STATE_REG;
		break;
	case CS40L26_REVID_A1:
		halo_state_reg = CS40L26_A1_DSP_HALO_STATE_REG;
		break;
	default:
		dev_err(cs40l26->dev, "Revid ID not supported: %02X\n",
			cs40l26->revid);
		return -EINVAL;
	}

	ret = regmap_read(cs40l26->regmap, halo_state_reg,
			&halo_state);
	if (ret)
		return ret;

	if (halo_state != CS40L26_DSP_HALO_STATE_RUN) {
		dev_err(cs40l26->dev, "DSP not Ready: HALO_STATE: %08X\n",
				halo_state);
		return -EINVAL;
	}

	ret = cs40l26_pm_state_transition(cs40l26,
			CS40L26_PM_STATE_PREVENT_HIBERNATE);
	if (ret)
		return ret;

	ret = cs40l26_dsp_state_get(cs40l26, &dsp_state);
	if (ret)
		return ret;

	if (dsp_state != CS40L26_DSP_STATE_SHUTDOWN &&
			dsp_state != CS40L26_DSP_STATE_STANDBY) {
		dev_err(cs40l26->dev, "DSP core not safe to kill\n");
		return -EINVAL;
	}

	/* errata write fixing indeterminent PLL lock time */
	ret = regmap_update_bits(cs40l26->regmap, CS40L26_PLL_REFCLK_DETECT_0,
			CS40L26_PLL_REFCLK_DET_EN_MASK, CS40L26_DISABLE);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to disable PLL refclk detect\n");
		return ret;
	}

	ret = regmap_write(cs40l26->regmap, CS40L26_DSP1_CCM_CORE_CONTROL,
			CS40L26_DSP_CCM_CORE_KILL);
	if (ret)
		dev_err(cs40l26->dev, "Failed to kill DSP core\n");

	return ret;
}

static int cs40l26_mbox_buffer_read(struct cs40l26_private *cs40l26, u32 *val)
{
	struct device *dev = cs40l26->dev;
	struct regmap *regmap = cs40l26->regmap;
	u32 base, last, len, write_ptr, read_ptr, mbox_response, reg;
	u32 buffer[CS40L26_DSP_MBOX_BUFFER_NUM_REGS];
	int ret;

	ret = cl_dsp_get_reg(cs40l26->dsp, "QUEUE_BASE",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_MAILBOX_ALGO_ID,
			&reg);
	if (ret)
		return ret;

	ret = regmap_bulk_read(regmap, reg, buffer,
			CS40L26_DSP_MBOX_BUFFER_NUM_REGS);
	if (ret) {
		dev_err(dev, "Failed to read buffer contents\n");
		return ret;
	}

	base = buffer[0];
	len = buffer[1];
	write_ptr = buffer[2];
	read_ptr = buffer[3];
	last = base + ((len - 1) * CL_DSP_BYTES_PER_WORD);

	if ((read_ptr - CL_DSP_BYTES_PER_WORD) == write_ptr) {
		dev_err(dev, "Mailbox buffer is full, info missing\n");
		return -ENOSPC;
	}

	if (read_ptr == write_ptr) {
		dev_dbg(dev, "Reached end of queue\n");
		return 1;
	}

	ret = regmap_read(regmap, read_ptr, &mbox_response);
	if (ret) {
		dev_err(dev, "Failed to read from mailbox buffer\n");
		return ret;
	}

	if (read_ptr == last)
		read_ptr = base;
	else
		read_ptr += CL_DSP_BYTES_PER_WORD;

	ret = cl_dsp_get_reg(cs40l26->dsp, "QUEUE_RD",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_MAILBOX_ALGO_ID,
			&reg);
	if (ret)
		return ret;

	ret = regmap_write(regmap, reg, read_ptr);
	if (ret) {
		dev_err(dev, "Failed to update read pointer\n");
		return ret;
	}

	*val = mbox_response;

	return 0;
}

static int cs40l26_handle_mbox_buffer(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	u32 val = 0;

	while (!cs40l26_mbox_buffer_read(cs40l26, &val)) {
		if ((val & CS40L26_DSP_MBOX_CMD_INDEX_MASK)
				== CS40L26_DSP_MBOX_PANIC) {
			dev_alert(dev, "DSP PANIC! Error condition: 0x%06X\n",
			(u32) (val & CS40L26_DSP_MBOX_CMD_PAYLOAD_MASK));
			return -ENOTRECOVERABLE;
		}

		switch (val) {
		case CS40L26_DSP_MBOX_TRIGGER_COMPLETE:
			if (cs40l26->vibe_state != CS40L26_VIBE_STATE_ASP)
				cs40l26_vibe_state_set(cs40l26,
						CS40L26_VIBE_STATE_STOPPED);
			dev_dbg(dev, "Trigger Complete\n");
			break;
		case CS40L26_DSP_MBOX_PM_AWAKE:
			cs40l26->wksrc_sts |= CS40L26_WKSRC_STS_EN;
			dev_dbg(dev, "HALO Core is awake\n");
			break;
		case CS40L26_DSP_MBOX_F0_EST_START:
			dev_dbg(dev, "F0_EST_START\n");
			break;
		case CS40L26_DSP_MBOX_F0_EST_DONE:
			dev_dbg(dev, "F0_EST_DONE\n");
			if (cs40l26->cal_requested &
				CS40L26_CALIBRATION_CONTROL_REQUEST_F0_AND_Q) {
				cs40l26->cal_requested &=
				~CS40L26_CALIBRATION_CONTROL_REQUEST_F0_AND_Q;
				/* for pm_runtime_get see trigger_calibration */
				pm_runtime_mark_last_busy(cs40l26->dev);
				pm_runtime_put_autosuspend(cs40l26->dev);
			} else {
				dev_err(dev, "Unexpected mbox msg: %d", val);
				return -EINVAL;
			}

			break;
		case CS40L26_DSP_MBOX_REDC_EST_START:
			dev_dbg(dev, "REDC_EST_START\n");
			break;
		case CS40L26_DSP_MBOX_REDC_EST_DONE:
			dev_dbg(dev, "REDC_EST_DONE\n");
			if (cs40l26->cal_requested &
				CS40L26_CALIBRATION_CONTROL_REQUEST_REDC) {
				cs40l26->cal_requested &=
				~CS40L26_CALIBRATION_CONTROL_REQUEST_REDC;
				/* for pm_runtime_get see trigger_calibration */
				pm_runtime_mark_last_busy(cs40l26->dev);
				pm_runtime_put_autosuspend(cs40l26->dev);
			} else {
				dev_err(dev, "Unexpected mbox msg: %d", val);
				return -EINVAL;
			}
			break;
		case CS40L26_DSP_MBOX_SYS_ACK:
			dev_err(dev, "Mbox buffer value (0x%X) not supported\n",
					val);
			return -EPERM;
		default:
			dev_err(dev, "MBOX buffer value (0x%X) is invalid\n",
					val);
			return -EINVAL;
		}
	}

	return 0;
}

void cs40l26_vibe_state_set(struct cs40l26_private *cs40l26,
		enum cs40l26_vibe_state new_state)
{
	if (cs40l26->vibe_state == new_state)
		return;

	if (new_state == CS40L26_VIBE_STATE_STOPPED && cs40l26->asp_enable) {
		/* Re-enable audio stream */
		cs40l26_class_h_set(cs40l26, true);
		if (!cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				CS40L26_DSP_MBOX_CMD_START_I2S,
				CS40L26_DSP_MBOX_RESET)) {
			cs40l26->vibe_state = CS40L26_VIBE_STATE_ASP;
			return;
		} else if (new_state == CS40L26_VIBE_STATE_HAPTIC &&
				cs40l26->vibe_state == CS40L26_VIBE_STATE_ASP) {
			cs40l26_class_h_set(cs40l26, false);
		}
	}

	cs40l26->vibe_state = new_state;
	sysfs_notify(&cs40l26->dev->kobj, NULL, "vibe_state");
}
EXPORT_SYMBOL(cs40l26_vibe_state_set);

static int cs40l26_event_count_get(struct cs40l26_private *cs40l26, u32 *count)
{
	unsigned int reg;
	int ret;

	ret = cl_dsp_get_reg(cs40l26->dsp, "EVENT_POST_COUNT",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_EVENT_HANDLER_ALGO_ID,
			&reg);
	if (ret)
		return ret;

	ret = regmap_read(cs40l26->regmap, reg, count);
	if (ret)
		dev_err(cs40l26->dev, "Failed to get event count\n");

	return ret;
}

static int cs40l26_error_release(struct cs40l26_private *cs40l26,
		unsigned int err_rls, bool bst_err)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	u32 err_sts, err_cfg;
	int ret;

	/* Boost related errors must be handled with DSP turned off */
	if (bst_err) {
		ret = cs40l26_dsp_shutdown(cs40l26);
		if (ret)
			return ret;
	}

	ret = regmap_read(regmap, CS40L26_ERROR_RELEASE, &err_sts);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to get error status\n");
		return ret;
	}

	err_cfg = err_sts & ~BIT(err_rls);

	ret = regmap_write(cs40l26->regmap, CS40L26_ERROR_RELEASE, err_cfg);
	if (ret) {
		dev_err(dev, "Actuator Safe Mode release sequence failed\n");
		return ret;
	}

	err_cfg |= BIT(err_rls);

	ret = regmap_write(regmap, CS40L26_ERROR_RELEASE, err_cfg);
	if (ret) {
		dev_err(dev, "Actuator Safe Mode release sequence failed\n");
		return ret;
	}

	err_cfg &= ~BIT(err_rls);

	ret = regmap_write(cs40l26->regmap, CS40L26_ERROR_RELEASE, err_cfg);
	if (ret) {
		dev_err(dev, "Actuator Safe Mode release sequence failed\n");
		return ret;
	}

	if (bst_err) {
		ret = cs40l26_dsp_wake(cs40l26);
		if (ret)
			return ret;
	}

	return ret;
}

static int cs40l26_iseq_update(struct cs40l26_private *cs40l26,
		enum cs40l26_iseq update)
{
	int ret;
	u32 val;

	ret = regmap_read(cs40l26->regmap, cs40l26->iseq_table[update].addr,
			&val);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to get IRQ seq. information\n");
		return ret;
	}

	cs40l26->iseq_table[update].val = val;

	return 0;
}

static int cs40l26_iseq_init(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	struct regmap *regmap = cs40l26->regmap;
	int ret, i;

	cs40l26->iseq_table[CS40L26_ISEQ_MASK1].addr = CS40L26_IRQ1_MASK_1;
	cs40l26->iseq_table[CS40L26_ISEQ_MASK2].addr = CS40L26_IRQ1_MASK_2;
	cs40l26->iseq_table[CS40L26_ISEQ_EDGE1].addr = CS40L26_IRQ1_EDGE_1;
	cs40l26->iseq_table[CS40L26_ISEQ_POL1].addr = CS40L26_IRQ1_POL_1;

	for (i = 0; i < CS40L26_ISEQ_MAX_ENTRIES; i++) {
		ret = regmap_read(regmap, cs40l26->iseq_table[i].addr,
				&cs40l26->iseq_table[i].val);
		if (ret) {
			dev_err(dev, "Failed to read IRQ settings\n");
			return ret;
		}
	}

	return ret;
}

static int cs40l26_iseq_populate(struct cs40l26_private *cs40l26)
{
	int ret, i;

	for (i = 0; i < CS40L26_ISEQ_MAX_ENTRIES; i++) {
		ret = regmap_write(cs40l26->regmap,
				cs40l26->iseq_table[i].addr,
				cs40l26->iseq_table[i].val);
		if (ret) {
			dev_err(cs40l26->dev,
				"Failed to update IRQ settings\n");
			return ret;
		}
	}

	return 0;
}

static int cs40l26_irq_update_mask(struct cs40l26_private *cs40l26, u32 irq_reg,
		u32 bits, bool mask)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	enum cs40l26_iseq update;
	u32 eint_reg;
	int ret;

	switch (irq_reg) {
	case CS40L26_IRQ1_MASK_1:
		update = CS40L26_ISEQ_MASK1;
		eint_reg = CS40L26_IRQ1_EINT_1;
		break;
	case CS40L26_IRQ1_MASK_2:
		update = CS40L26_ISEQ_MASK2;
		eint_reg = CS40L26_IRQ1_EINT_2;
		break;
	default:
		dev_err(dev, "Invalid IRQ mask register 0x%08X\n", irq_reg);
		return -EINVAL;
	}

	ret = regmap_write(regmap, eint_reg, bits);
	if (ret) {
		dev_err(dev,
			"Failed to clear status of bits 0x%08X\n",
			eint_reg);
		return ret;
	}

	ret = regmap_update_bits(regmap, irq_reg, bits, mask ? bits : ~bits);
	if (ret) {
		dev_err(dev, "Failed to update IRQ mask 0x%08X\n", irq_reg);
		return ret;
	}

	return cs40l26_iseq_update(cs40l26, update);
}

static int cs40l26_handle_irq1(struct cs40l26_private *cs40l26,
		enum cs40l26_irq1 irq1, bool trigger)
{
	struct device *dev = cs40l26->dev;
	u32 err_rls = 0;
	unsigned int reg, val;
	bool bst_err;
	int ret;

	switch (irq1) {
	case CS40L26_IRQ1_GPIO1_RISE:
	/* intentionally fall through */
	case CS40L26_IRQ1_GPIO1_FALL:
	/* intentionally fall through */
	case CS40L26_IRQ1_GPIO2_RISE:
	/* intentionally fall through */
	case CS40L26_IRQ1_GPIO2_FALL:
	/* intentionally fall through */
	case CS40L26_IRQ1_GPIO3_RISE:
	/* intentionally fall through */
	case CS40L26_IRQ1_GPIO3_FALL:
	/* intentionally fall through */
	case CS40L26_IRQ1_GPIO4_RISE:
	/* intentionally fall through */
	case CS40L26_IRQ1_GPIO4_FALL:
		if (cs40l26->wksrc_sts & CS40L26_WKSRC_STS_EN) {
			dev_dbg(dev, "GPIO%u %s edge detected\n",
					(irq1 / 2) + 1,
					irq1 % 2 ? "falling" : "rising");
			if (trigger)
				cs40l26_vibe_state_set(cs40l26,
						CS40L26_VIBE_STATE_HAPTIC);
		}

		cs40l26->wksrc_sts |= CS40L26_WKSRC_STS_EN;
		break;
	case CS40L26_IRQ1_WKSRC_STS_ANY:
		dev_dbg(dev, "Wakesource detected (ANY)\n");

		ret = cs40l26_iseq_populate(cs40l26);
		if (ret)
			goto err;

		ret = regmap_read(cs40l26->regmap, CS40L26_PWRMGT_STS, &val);
		if (ret) {
			dev_err(dev, "Failed to get Power Management Status\n");
			goto err;
		}

		cs40l26->wksrc_sts = (u8) ((val & CS40L26_WKSRC_STS_MASK) >>
				CS40L26_WKSRC_STS_SHIFT);

		ret = cl_dsp_get_reg(cs40l26->dsp, "LAST_WAKESRC_CTL",
				CL_DSP_XM_UNPACKED_TYPE,
				CS40L26_FW_ID, &reg);
		if (ret)
			goto err;

		ret = regmap_read(cs40l26->regmap, reg, &val);
		if (ret) {
			dev_err(dev, "Failed to read LAST_WAKESRC_CTL\n");
			goto err;
		}
		cs40l26->last_wksrc_pol =
				(u8) (val & CS40L26_WKSRC_GPIO_POL_MASK);
		break;
	case CS40L26_IRQ1_WKSRC_STS_GPIO1:
	/* intentionally fall through */
	case CS40L26_IRQ1_WKSRC_STS_GPIO2:
	/* intentionally fall through */
	case CS40L26_IRQ1_WKSRC_STS_GPIO3:
	/* intentionally fall through */
	case CS40L26_IRQ1_WKSRC_STS_GPIO4:
		dev_dbg(dev, "GPIO%u event woke device from hibernate\n",
				irq1 - CS40L26_IRQ1_WKSRC_STS_GPIO1 + 1);

		if (cs40l26->wksrc_sts & cs40l26->last_wksrc_pol) {
			dev_dbg(dev, "GPIO%u falling edge detected\n",
					irq1 - 8);
			cs40l26->wksrc_sts |= CS40L26_WKSRC_STS_EN;
		} else {
			dev_dbg(dev, "GPIO%u rising edge detected\n",
					irq1 - 8);
		}
		if (trigger)
			cs40l26_vibe_state_set(cs40l26,
					CS40L26_VIBE_STATE_HAPTIC);
		break;
	case CS40L26_IRQ1_WKSRC_STS_SPI:
		dev_dbg(dev, "SPI event woke device from hibernate\n");
		break;
	case CS40L26_IRQ1_WKSRC_STS_I2C:
		dev_dbg(dev, "I2C event woke device from hibernate\n");
		break;
	case CS40L26_IRQ1_GLOBAL_EN_ASSERT:
		dev_dbg(dev, "Started power up seq. (GLOBAL_EN asserted)\n");
		break;
	case CS40L26_IRQ1_PDN_DONE:
		dev_dbg(dev,
			"Completed power down seq. (GLOBAL_EN cleared)\n");
		break;
	case CS40L26_IRQ1_PUP_DONE:
		dev_dbg(dev,
			"Completed power up seq. (GLOBAL_EN asserted)\n");
		break;
	case CS40L26_IRQ1_BST_OVP_FLAG_RISE:
		dev_warn(dev, "BST overvoltage warning\n");
		break;
	case CS40L26_IRQ1_BST_OVP_FLAG_FALL:
		dev_warn(dev,
			"BST voltage returned below warning threshold\n");
		break;
	case CS40L26_IRQ1_BST_OVP_ERR:
		dev_alert(dev, "BST overvolt. error, CS40L26 shutting down\n");
		err_rls = CS40L26_BST_OVP_ERR_RLS;
		bst_err = true;
		break;
	case CS40L26_IRQ1_BST_DCM_UVP_ERR:
		dev_alert(dev,
			"BST undervolt. error, CS40L26 shutting down\n");
		err_rls = CS40L26_BST_UVP_ERR_RLS;
		bst_err = true;
		break;
	case CS40L26_IRQ1_BST_SHORT_ERR:
		dev_alert(dev, "LBST short detected, CS40L26 shutting down\n");
		err_rls = CS40L26_BST_SHORT_ERR_RLS;
		bst_err = true;
		break;
	case CS40L26_IRQ1_BST_IPK_FLAG:
		dev_warn(dev, "Current is being limited by LBST inductor\n");
		break;
	case CS40L26_IRQ1_TEMP_WARN_RISE:
		dev_err(dev, "Die overtemperature warning\n");
		err_rls = CS40L26_TEMP_WARN_ERR_RLS;
		break;
	case CS40L26_IRQ1_TEMP_WARN_FALL:
		dev_warn(dev, "Die temperature returned below threshold\n");
		break;
	case CS40L26_IRQ1_TEMP_ERR:
		dev_alert(dev,
			"Die overtemperature error, CS40L26 shutting down\n");
		err_rls = CS40L26_TEMP_ERR_RLS;
		break;
	case CS40L26_IRQ1_AMP_ERR:
		dev_alert(dev, "AMP short detected, CS40L26 shutting down\n");
		err_rls = CS40L26_AMP_SHORT_ERR_RLS;
		break;
	case CS40L26_IRQ1_DC_WATCHDOG_RISE:
		dev_err(dev, "DC level detected\n");
		break;
	case CS40L26_IRQ1_DC_WATCHDOG_FALL:
		dev_warn(dev, "Previously detected DC level removed\n");
		break;
	case CS40L26_IRQ1_VIRTUAL1_MBOX_WR:
		dev_dbg(dev, "Virtual 1 MBOX write occurred\n");
		break;
	case CS40L26_IRQ1_VIRTUAL2_MBOX_WR:
		ret = cs40l26_handle_mbox_buffer(cs40l26);
		if (ret)
			goto err;
		break;
	default:
		dev_err(dev, "Unrecognized IRQ1 EINT1 status\n");
		return -EINVAL;
	}

	if (err_rls)
		ret = cs40l26_error_release(cs40l26, err_rls, bst_err);

err:
	regmap_write(cs40l26->regmap, CS40L26_IRQ1_EINT_1, BIT(irq1));

	return ret;
}

static int cs40l26_handle_irq2(struct cs40l26_private *cs40l26,
		enum cs40l26_irq2 irq2)
{
	struct device *dev = cs40l26->dev;
	unsigned int val;
	u32 vbbr_status, vpbr_status;
	int ret;

	switch (irq2) {
	case CS40L26_IRQ2_PLL_LOCK:
		dev_dbg(dev, "PLL achieved lock\n");
		break;
	case CS40L26_IRQ2_PLL_PHASE_LOCK:
		dev_dbg(dev, "PLL achieved phase lock\n");
		break;
	case CS40L26_IRQ2_PLL_FREQ_LOCK:
		dev_dbg(dev, "PLL achieved frequency lock\n");
		break;
	case CS40L26_IRQ2_PLL_UNLOCK_RISE:
		dev_err(dev, "PLL has lost lock\n");
		break;
	case CS40L26_IRQ2_PLL_UNLOCK_FALL:
		dev_warn(dev, "PLL has regained lock\n");
		break;
	case CS40L26_IRQ2_PLL_READY:
		dev_dbg(dev, "PLL ready for use\n");
		break;
	case CS40L26_IRQ2_PLL_REFCLK_PRESENT:
		dev_warn(dev, "REFCLK present for PLL\n");
		break;
	case CS40L26_IRQ2_REFCLK_MISSING_RISE:
		dev_err(dev, "REFCLK input for PLL is missing\n");
		break;
	case CS40L26_IRQ2_REFCLK_MISSING_FALL:
		dev_warn(dev, "REFCLK reported missing is now present\n");
		break;
	case CS40L26_IRQ2_ASP_RXSLOT_CFG_ERR:
		dev_err(dev, "Misconfig. of ASP_RX 1 2 or 3 SLOT fields\n");
			break;
	case CS40L26_IRQ2_AUX_NG_CH1_ENTRY:
		dev_warn(dev,
			"CH1 data of noise gate has fallen below threshold\n");
		break;
	case CS40L26_IRQ2_AUX_NG_CH1_EXIT:
		dev_err(dev,
			"CH1 data of noise gate has risen above threshold\n");
		break;
	case CS40L26_IRQ2_AUX_NG_CH2_ENTRY:
		dev_warn(dev,
			"CH2 data of noise gate has fallen below threshold\n");
		break;
	case CS40L26_IRQ2_AUX_NG_CH2_EXIT:
		dev_err(dev,
			"CH2 data of noise gate has risen above threshold\n");
		break;
	case CS40L26_IRQ2_AMP_NG_ON_RISE:
		dev_warn(dev, "Amplifier entered noise-gated state\n");
		break;
	case CS40L26_IRQ2_AMP_NG_ON_FALL:
		dev_warn(dev, "Amplifier exited noise-gated state\n");
		break;
	case CS40L26_IRQ2_VPBR_FLAG:
		dev_alert(dev,
			"VP voltage has dropped below brownout threshold\n");
		ret = regmap_read(cs40l26->regmap, CS40L26_VPBR_STATUS, &val);
		if (ret) {
			dev_err(dev, "Failed to get VPBR_STATUS\n");
			return ret;
		}

		vpbr_status = (val & CS40L26_VXBR_STATUS_MASK);
		dev_alert(dev, "VPBR Attenuation applied = %u x 10^-4 dB\n",
				vpbr_status * CS40L26_VXBR_STATUS_DIV_STEP);
		break;
	case CS40L26_IRQ2_VPBR_ATT_CLR:
		dev_warn(dev,
			"Cleared attenuation applied by VP brownout event\n");
		break;
	case CS40L26_IRQ2_VBBR_FLAG:
		dev_alert(dev,
			"VBST voltage has dropped below brownout threshold\n");
		ret = regmap_read(cs40l26->regmap, CS40L26_VBBR_STATUS, &val);
		if (ret) {
			dev_err(dev, "Failed to get VPBR_STATUS\n");
			return ret;
		}

		vbbr_status = (val & CS40L26_VXBR_STATUS_MASK);
		dev_alert(dev, "VBBR Attenuation applied = %u x 10^-4 dB\n",
				vbbr_status * CS40L26_VXBR_STATUS_DIV_STEP);
		break;
	case CS40L26_IRQ2_VBBR_ATT_CLR:
		dev_warn(dev, "Cleared attenuation caused by VBST brownout\n");
		break;
	case CS40L26_IRQ2_I2C_NACK_ERR:
		dev_err(dev, "I2C interface NACK during Broadcast Mode\n");
		break;
	case CS40L26_IRQ2_VPMON_CLIPPED:
		dev_err(dev, "Input larger than full-scale value (VPMON)\n");
		break;
	case CS40L26_IRQ2_VBSTMON_CLIPPED:
		dev_err(dev, "Input larger than full-scale value (VBSTMON)\n");
		break;
	case CS40L26_IRQ2_VMON_CLIPPED:
		dev_err(dev, "Input larger than full-scale value (VMON)\n");
		break;
	case CS40L26_IRQ2_IMON_CLIPPED:
		dev_err(dev, "Input larger than full-scale value (IMON)\n");
		break;
	default:
		dev_err(dev, "Unrecognized IRQ1 EINT2 status\n");
		return -EINVAL;
	}

	/* write 1 to clear the interrupt flag */
	ret = regmap_write(cs40l26->regmap, CS40L26_IRQ1_EINT_2, BIT(irq2));
	if (ret)
		dev_err(dev, "Failed to clear IRQ1 EINT2 %u\n", irq2);

	return ret;
}

static irqreturn_t cs40l26_irq(int irq, void *data)
{
	struct cs40l26_private *cs40l26 = (struct cs40l26_private *)data;
	unsigned int sts, val, eint, mask, i, irq1_count = 0, irq2_count = 0;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned long num_irq;
	u32 event_count;
	bool trigger;
	int ret;

	if (cs40l26_dsp_read(cs40l26, CS40L26_IRQ1_STATUS, &sts)) {
		dev_err(dev, "Failed to read IRQ1 Status\n");
		return IRQ_NONE;
	}

	if (sts != CS40L26_IRQ_STATUS_ASSERT) {
		dev_err(dev, "IRQ1 asserted with no pending interrupts\n");
		return IRQ_NONE;
	}

	pm_runtime_get_sync(dev);

	ret = regmap_read(regmap, CS40L26_IRQ1_EINT_1, &eint);
	if (ret) {
		dev_err(dev, "Failed to read interrupts status 1\n");
		goto err;
	}

	ret = regmap_read(regmap, CS40L26_IRQ1_MASK_1, &mask);
	if (ret) {
		dev_err(dev, "Failed to get interrupts mask 1\n");
		goto err;
	}

	val = eint & ~mask;
	if (val) {
		ret = cs40l26_event_count_get(cs40l26, &event_count);
		if (ret)
			goto err;

		trigger = (event_count > cs40l26->event_count);

		num_irq = hweight_long(val);
		i = 0;
		while (irq1_count < num_irq && i < CS40L26_IRQ1_NUM_IRQS) {
			if (val & BIT(i)) {
				ret = cs40l26_handle_irq1(cs40l26, i, trigger);
				if (ret)
					goto err;
				else
					irq1_count++;
			}
			i++;
		}
	}

	ret = regmap_read(regmap, CS40L26_IRQ1_EINT_2, &eint);
	if (ret) {
		dev_err(dev, "Failed to read interrupts status 2\n");
		goto err;
	}

	ret = regmap_read(regmap, CS40L26_IRQ1_MASK_2, &mask);
	if (ret) {
		dev_err(dev, "Failed to get interrupts mask 2\n");
		goto err;
	}

	val = eint & ~mask;
	if (val) {
		num_irq = hweight_long(val);

		i = 0;
		while (irq2_count < num_irq && i < CS40L26_IRQ2_NUM_IRQS) {
			if (val & BIT(i)) {
				ret = cs40l26_handle_irq2(cs40l26, i);
				if (ret)
					goto err;
				else
					irq2_count++;
			}
			i++;
		}
	}

err:
	cs40l26_event_count_get(cs40l26, &cs40l26->event_count);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
	/* if an error has occurred, all IRQs have not been successfully
	 * processed; however, IRQ_HANDLED is still returned if at least one
	 * interrupt request generated by CS40L26 was handled successfully.
	 */
	if (ret)
		dev_err(dev, "Failed to process IRQ (%d): %u\n", irq, ret);

	return (irq1_count + irq2_count) ? IRQ_HANDLED : IRQ_NONE;
}

static bool cs40l26_pseq_v1_addr_exists(struct cs40l26_private *cs40l26,
							u16 addr, int *index)
{
	int i;

	if (cs40l26->pseq_v1_len == 0)
		return false;

	for (i = 0; i < cs40l26->pseq_v1_len; i++) {
		if (cs40l26->pseq_v1_table[i].addr == addr) {
			*index = i;
			return true;
		}
	}

	*index = -1;

	return false;
}

static int cs40l26_pseq_v1_write(struct cs40l26_private *cs40l26,
		unsigned int pseq_v1_offset)
{
	struct regmap *regmap = cs40l26->regmap;
	unsigned int len = cs40l26->pseq_v1_len;
	struct device *dev = cs40l26->dev;
	u32 val;
	u16 addr;
	int ret;

	addr = cs40l26->pseq_v1_table[pseq_v1_offset].addr;
	val = cs40l26->pseq_v1_table[pseq_v1_offset].val;

	/* the "upper half" first 24-bit word of the sequence pair is written
	 * to the write sequencer as: [23-16] addr{15-0},
	 * [15-0] val{31-24} with bits [24-31] acting as a buffer
	 */
	ret = regmap_write(regmap, cs40l26->pseq_base +
			(pseq_v1_offset * CS40L26_PSEQ_V1_STRIDE),
			(addr << CS40L26_PSEQ_V1_ADDR_SHIFT) |
			((val & ~CS40L26_PSEQ_V1_VAL_MASK)
			>> CS40L26_PSEQ_V1_VAL_SHIFT));
	if (ret) {
		dev_err(dev, "Failed to write power on seq. (upper half)\n");
		return ret;
	}

	/* the "lower half" of the address-value pair is written to the write
	 * sequencer as: [23-0] data{23-0} with bits [24-31] acting as a buffer
	 */
	ret = regmap_write(regmap, cs40l26->pseq_base + CL_DSP_BYTES_PER_WORD
			+ (pseq_v1_offset * CS40L26_PSEQ_V1_STRIDE),
			val & CS40L26_PSEQ_V1_VAL_MASK);
	if (ret) {
		dev_err(dev, "Failed to write power on seq. (lower half)\n");
		return ret;
	}

	/* end of sequence must be marked by list terminator */
	ret = regmap_write(regmap, cs40l26->pseq_base +
			(len * CS40L26_PSEQ_V1_STRIDE),
			CS40L26_PSEQ_V1_LIST_TERM);
	if (ret)
		dev_err(dev, "Failed to write power on seq. terminator\n");

	return ret;
}

static int cs40l26_pseq_v1_add_pair(struct cs40l26_private *cs40l26, u16 addr,
		u32 val, bool replace)
{
	unsigned int len = cs40l26->pseq_v1_len;
	struct device *dev = cs40l26->dev;
	unsigned int pseq_v1_offset, prev_val;
	int ret, index;

	if (len >= CS40L26_PSEQ_V1_MAX_ENTRIES) {
		dev_err(dev, "Power on seq. exceeded max number of entries\n");
		return -E2BIG;
	}

	if (cs40l26_pseq_v1_addr_exists(cs40l26, addr, &index) && replace) {
		prev_val = cs40l26->pseq_v1_table[index].val;
		cs40l26->pseq_v1_table[index].val = val;
		pseq_v1_offset = index;
	} else {
		cs40l26->pseq_v1_table[len].addr = addr;
		cs40l26->pseq_v1_table[len].val = val;
		cs40l26->pseq_v1_len++;
		pseq_v1_offset = len;
	}

	ret = cs40l26_pseq_v1_write(cs40l26, pseq_v1_offset);
	if (ret) { /* If an error occurs during write, reset the sequence */
		if (index < 0) { /* No previous value for this address */
			cs40l26->pseq_v1_table[len].addr = 0;
			cs40l26->pseq_v1_table[len].val = 0;
			cs40l26->pseq_v1_len--;
		} else {
			cs40l26->pseq_v1_table[index].val = prev_val;
		}
	}

	return ret;
}

int cs40l26_pseq_v1_multi_add_pair(struct cs40l26_private *cs40l26,
		const struct reg_sequence *reg_seq, int num_regs, bool replace)
{
	int ret, i;

	for (i = 0; i < num_regs; i++) {
		ret = cs40l26_pseq_v1_add_pair(cs40l26, (u16) (reg_seq[i].reg &
				CS40L26_PSEQ_V1_ADDR_MASK), reg_seq[i].def,
				replace);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(cs40l26_pseq_v1_multi_add_pair);

static int cs40l26_pseq_v2_add_op(struct cs40l26_private *cs40l26,
		int num_words, u32 *words)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	u32 offset_for_new_op, *op_words;
	int ret;
	struct cs40l26_pseq_v2_op *pseq_v2_op_end, *pseq_v2_op_new, *op;

	/* get location of the list terminator */
	list_for_each_entry(pseq_v2_op_end, &cs40l26->pseq_v2_op_head, list) {
		if (pseq_v2_op_end->operation == CS40L26_PSEQ_V2_OP_END)
			break;
	}

	if (pseq_v2_op_end->operation != CS40L26_PSEQ_V2_OP_END) {
		dev_err(dev, "Failed to find END_OF_SCRIPT\n");
		return -EINVAL;
	}

	offset_for_new_op = pseq_v2_op_end->offset;

	/* add new op to list */
	op_words = kzalloc(num_words * CL_DSP_BYTES_PER_WORD, GFP_KERNEL);
	if (!op_words)
		return -ENOMEM;
	memcpy(op_words, words, num_words * CL_DSP_BYTES_PER_WORD);
	pseq_v2_op_new = devm_kzalloc(dev, sizeof(*pseq_v2_op_new), GFP_KERNEL);
	if (!pseq_v2_op_new) {
		ret = -ENOMEM;
		goto err_free;
	}

	pseq_v2_op_new->size = num_words;
	pseq_v2_op_new->offset = offset_for_new_op;
	pseq_v2_op_new->words = op_words;
	list_add(&pseq_v2_op_new->list, &cs40l26->pseq_v2_op_head);

	cs40l26->pseq_v2_num_ops++;

	/* bump end of script offset to accomodate new operation */
	pseq_v2_op_end->offset += num_words * CL_DSP_BYTES_PER_WORD;

	list_for_each_entry(op, &cs40l26->pseq_v2_op_head, list) {
		if (op->offset >= offset_for_new_op) {
			ret = regmap_bulk_write(regmap, cs40l26->pseq_base +
							op->offset,
							op->words,
							op->size);
			if (ret) {
				dev_err(dev, "Failed to write op\n");
				return -EIO;
			}
		}
	}

	return ret;

err_free:
	kfree(op_words);

	return ret;
}

static int cs40l26_pseq_v2_add_write_reg_full(struct cs40l26_private *cs40l26,
		u32 addr, u32 data, bool update_if_op_already_in_seq)
{
	int ret;
	struct cs40l26_pseq_v2_op *op;
	u32 op_words[CS40L26_PSEQ_V2_OP_WRITE_REG_FULL_WORDS];

	op_words[0] = (CS40L26_PSEQ_V2_OP_WRITE_REG_FULL <<
						CS40L26_PSEQ_V2_OP_SHIFT);
	op_words[0] |= addr >> 16;
	op_words[1] = (addr & 0x0000FFFF) << 8;
	op_words[1] |= (data & 0xFF000000) >> 24;
	op_words[2] = (data & 0x00FFFFFF);

	if (update_if_op_already_in_seq) {
		list_for_each_entry(op, &cs40l26->pseq_v2_op_head, list) {
			/* check if op with same op and addr already exists */
			if ((op->words[0] == op_words[0]) &&
				((op->words[1] & 0xFFFFFF00) ==
				(op_words[1] & 0xFFFFFF00))) {
				/* update data in the existing op and return */
				ret = regmap_bulk_write(cs40l26->regmap,
						cs40l26->pseq_base + op->offset,
						op_words, op->size);
				if (ret)
					dev_err(cs40l26->dev,
						"Failed to update op\n");
				return ret;
			}
		}
	}

	/* if no matching op is found or !update, add the op */
	ret = cs40l26_pseq_v2_add_op(cs40l26,
				CS40L26_PSEQ_V2_OP_WRITE_REG_FULL_WORDS,
				op_words);

	return ret;
}

int cs40l26_pseq_v2_multi_add_write_reg_full(struct cs40l26_private *cs40l26,
		const struct reg_sequence *reg_seq, int num_regs,
		bool update_if_op_already_in_seq)
{
	int ret, i;

	for (i = 0; i < num_regs; i++) {
		ret = cs40l26_pseq_v2_add_write_reg_full(cs40l26,
						reg_seq[i].reg, reg_seq[i].def,
						update_if_op_already_in_seq);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(cs40l26_pseq_v2_multi_add_write_reg_full);

static int cs40l26_pseq_v1_init(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	int ret, i, index = 0;
	u8 upper_val = 0;
	u16 addr = 0;
	u32 val, word, algo_id;

	if (cs40l26->fw_mode == CS40L26_FW_MODE_RAM)
		algo_id = CS40L26_PM_ALGO_ID;
	else
		algo_id = CS40L26_PM_ROM_ALGO_ID;

	ret = cl_dsp_get_reg(cs40l26->dsp, "POWER_ON_SEQUENCE",
			CL_DSP_XM_UNPACKED_TYPE, algo_id, &cs40l26->pseq_base);
	if (ret)
		return ret;

	for (i = 0; i < CS40L26_PSEQ_V1_MAX_WRITES; i++) {
		ret = regmap_read(cs40l26->regmap,
				cs40l26->pseq_base +
				(i * CL_DSP_BYTES_PER_WORD), &word);
		if (ret) {
			dev_err(dev, "Failed to read from power on seq.\n");
			return ret;
		}

		if ((word & CS40L26_PSEQ_V1_LIST_TERM_MASK) ==
				CS40L26_PSEQ_V1_LIST_TERM)
			break;

		if (i % CS40L26_PSEQ_V1_PAIR_NUM_WORDS) { /* lower half */
			index = i / CS40L26_PSEQ_V1_PAIR_NUM_WORDS;
			val = (upper_val << CS40L26_PSEQ_V1_VAL_SHIFT) |
					(word & CS40L26_PSEQ_V1_VAL_MASK);

			cs40l26->pseq_v1_table[index].addr = addr;
			cs40l26->pseq_v1_table[index].val = val;
		} else { /* upper half */
			addr = (word & CS40L26_PSEQ_V1_ADDR_WORD_MASK) >>
					CS40L26_PSEQ_V1_ADDR_SHIFT;

			upper_val = word & CS40L26_PSEQ_V1_VAL_WORD_UPPER_MASK;
		}
	}

	if (i >= CS40L26_PSEQ_V1_MAX_WRITES) {
		dev_err(dev, "Original sequence exceeds max # of entries\n");
		return -E2BIG;
	}

	ret = regmap_write(cs40l26->regmap, cs40l26->pseq_base +
			(i * CL_DSP_BYTES_PER_WORD), CS40L26_PSEQ_V1_LIST_TERM);
	if (ret)
		return ret;

	cs40l26->pseq_v1_len = index + 1;
	return 0;
}

static int cs40l26_pseq_v2_init(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	int ret, i, j, num_words, read_size;
	u8 operation;
	u32 words[CS40L26_PSEQ_V2_MAX_WORDS], algo_id, *op_words;
	struct cs40l26_pseq_v2_op *pseq_v2_op;

	INIT_LIST_HEAD(&cs40l26->pseq_v2_op_head);
	cs40l26->pseq_v2_num_ops = 0;

	if (cs40l26->fw_mode == CS40L26_FW_MODE_RAM)
		algo_id = CS40L26_PM_ALGO_ID;
	else
		algo_id = CS40L26_PM_ROM_ALGO_ID;

	ret = cl_dsp_get_reg(cs40l26->dsp, "POWER_ON_SEQUENCE",
			CL_DSP_XM_UNPACKED_TYPE, algo_id, &cs40l26->pseq_base);
	if (ret)
		return ret;

	/* read pseq memory space */
	i = 0;
	while (i < CS40L26_PSEQ_V2_MAX_WORDS) {
		read_size = min(CS40L26_MAX_I2C_READ_SIZE_BYTES,
			CS40L26_PSEQ_V2_MAX_WORDS - i);
		ret = regmap_bulk_read(cs40l26->regmap,
			cs40l26->pseq_base + i * CL_DSP_BYTES_PER_WORD,
			words + i,
			read_size);
		if (ret) {
			dev_err(dev, "Failed to read from power on seq.\n");
			return ret;
		}
		i += read_size;
	}

	i = 0;
	while (i < CS40L26_PSEQ_V2_MAX_WORDS) {
		operation = (words[i] & CS40L26_PSEQ_V2_OP_MASK) >>
			CS40L26_PSEQ_V2_OP_SHIFT;

		/* get num words for given operation */
		for (j = 0; j < CS40L26_PSEQ_V2_NUM_OPS; j++) {
			if (cs40l26_pseq_v2_op_sizes[j][0] == operation) {
				num_words = cs40l26_pseq_v2_op_sizes[j][1];
				break;
			}
		}

		if (j == CS40L26_PSEQ_V2_NUM_OPS) {
			dev_err(dev, "Failed to determine pseq_v2 op size\n");
			return -EINVAL;
		}

		op_words = kzalloc(num_words * CL_DSP_BYTES_PER_WORD,
								GFP_KERNEL);
		if (!op_words)
			return -ENOMEM;
		memcpy(op_words, &words[i], num_words * CL_DSP_BYTES_PER_WORD);

		pseq_v2_op = devm_kzalloc(dev, sizeof(*pseq_v2_op), GFP_KERNEL);
		if (!pseq_v2_op) {
			ret = -ENOMEM;
			goto err_free;
		}

		pseq_v2_op->size = num_words;
		pseq_v2_op->offset = i * CL_DSP_BYTES_PER_WORD;
		pseq_v2_op->operation = operation;
		pseq_v2_op->words = op_words;
		list_add(&pseq_v2_op->list, &cs40l26->pseq_v2_op_head);

		cs40l26->pseq_v2_num_ops++;
		i += num_words;

		if (operation == CS40L26_PSEQ_V2_OP_END)
			break;

	}

	if (operation != CS40L26_PSEQ_V2_OP_END) {
		dev_err(dev, "PSEQ_V2 END_OF_SCRIPT not found\n");
		return -E2BIG;
	}

	return ret;

err_free:
	kfree(op_words);

	return ret;
}

static int cs40l26_pseq_init(struct cs40l26_private *cs40l26)
{
	int ret;

	switch (cs40l26->revid) {
	case CS40L26_REVID_A0:
		ret = cs40l26_pseq_v1_init(cs40l26);
		break;
	case CS40L26_REVID_A1:
		ret = cs40l26_pseq_v2_init(cs40l26);
		break;
	default:
		dev_err(cs40l26->dev, "Revid ID not supported: %02X\n",
			cs40l26->revid);
		return -EINVAL;
	}

	if (ret)
		dev_err(cs40l26->dev, "Failed to init pseq\n");

	return ret;
}

static void cs40l26_set_gain_worker(struct work_struct *work)
{
	struct cs40l26_private *cs40l26 =
		container_of(work, struct cs40l26_private, set_gain_work);
	u32 reg, val;
	int ret;

	pm_runtime_get_sync(cs40l26->dev);
	mutex_lock(&cs40l26->lock);

	switch (cs40l26->revid) {
	case CS40L26_REVID_A0:
		val = cs40l26_a0_gain_vals[cs40l26->gain_pct];

		ret = regmap_update_bits(cs40l26->regmap, CS40L26_AMP_CTRL,
				CS40L26_AMP_CTRL_VOL_PCM_MASK, val <<
				CS40L26_AMP_CTRL_VOL_PCM_SHIFT);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to update digtal gain\n");
			goto err_mutex;
		}

		ret = regmap_read(cs40l26->regmap, CS40L26_AMP_CTRL, &val);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to read AMP control\n");
			goto err_mutex;
		}

		ret = cs40l26_pseq_v1_add_pair(cs40l26,
						CS40L26_AMP_CTRL, val, true);
		if (ret)
			dev_err(cs40l26->dev, "Failed to set gain in pseq\n");
		break;
	case CS40L26_REVID_A1:
		val = cs40l26_attn_q21_2_vals[cs40l26->gain_pct];

		/* Write Q21.2 value to SOURCE_ATTENUATION */
		ret = cl_dsp_get_reg(cs40l26->dsp, "SOURCE_ATTENUATION",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_EXT_ALGO_ID, &reg);
		if (ret)
			goto err_mutex;

		ret = regmap_write(cs40l26->regmap, reg, val);
		if (ret)
			dev_err(cs40l26->dev, "Failed to set attenuation\n");
		break;
	default:
		dev_err(cs40l26->dev, "Revid ID not supported: %02X\n",
			cs40l26->revid);
	}

err_mutex:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);
}

static void cs40l26_vibe_start_worker(struct work_struct *work)
{
	struct cs40l26_private *cs40l26 = container_of(work,
			struct cs40l26_private, vibe_start_work);
	struct device *dev = cs40l26->dev;
	int ret = 0;
	unsigned int reg, freq;
	u32 index = 0, buzz_id;
	u16 duration;

	dev_dbg(dev, "%s\n", __func__);

	if (cs40l26->fw_mode == CS40L26_FW_MODE_RAM)
		buzz_id = CS40L26_BUZZGEN_ALGO_ID;
	else
		buzz_id = CS40L26_BUZZGEN_ROM_ALGO_ID;

	if (cs40l26->effect->u.periodic.waveform == FF_CUSTOM)
		index = cs40l26->trigger_indices[cs40l26->effect->id];

	if (index >= CS40L26_OWT_INDEX_START && index <= CS40L26_OWT_INDEX_END)
		duration = CS40L26_SAMPS_TO_MS((cs40l26->owt_wlength &
				CS40L26_WT_TYPE10_WAVELEN_MAX));
	else
		duration = cs40l26->effect->replay.length;

	pm_runtime_get_sync(dev);
	mutex_lock(&cs40l26->lock);

	if (duration > 0) /* Effect duration is known */
		hrtimer_start(&cs40l26->vibe_timer,
			ktime_set(CS40L26_MS_TO_SECS(duration),
			CS40L26_MS_TO_NS(duration % 1000)), HRTIMER_MODE_REL);

	cs40l26_vibe_state_set(cs40l26, CS40L26_VIBE_STATE_HAPTIC);

	switch (cs40l26->effect->u.periodic.waveform) {
	case FF_CUSTOM:
		ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				index, CS40L26_DSP_MBOX_RESET);
		if (ret)
			goto err_mutex;
		break;
	case FF_SINE:
		ret = cl_dsp_get_reg(cs40l26->dsp, "BUZZ_EFFECTS2_BUZZ_FREQ",
				CL_DSP_XM_UNPACKED_TYPE, buzz_id, &reg);
		if (ret)
			goto err_mutex;

		freq = CS40L26_MS_TO_HZ(cs40l26->effect->u.periodic.period);

		ret = regmap_write(cs40l26->regmap, reg, freq);
		if (ret)
			goto err_mutex;

		ret = regmap_write(cs40l26->regmap, reg +
				CS40L26_BUZZGEN_LEVEL_OFFSET,
				CS40L26_BUZZGEN_LEVEL_DEFAULT);
		if (ret)
			goto err_mutex;

		ret = regmap_write(cs40l26->regmap, reg +
				CS40L26_BUZZGEN_DURATION_OFFSET, duration /
				CS40L26_BUZZGEN_DURATION_DIV_STEP);
		if (ret)
			goto err_mutex;

		ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
					CS40L26_BUZZGEN_INDEX_CP_TRIGGER,
					CS40L26_DSP_MBOX_RESET);
		if (ret)
			goto err_mutex;
		break;
	default:
		dev_err(dev, "Invalid waveform type: 0x%X\n",
				cs40l26->effect->u.periodic.waveform);
		ret = -EINVAL;
		goto err_mutex;
	}

err_mutex:
	if (ret)
		cs40l26_vibe_state_set(cs40l26, CS40L26_VIBE_STATE_STOPPED);

	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
}

static void cs40l26_vibe_stop_worker(struct work_struct *work)
{
	struct cs40l26_private *cs40l26 = container_of(work,
			struct cs40l26_private, vibe_stop_work);
	unsigned int reg;
	u32 algo_id;
	int ret;

	dev_dbg(cs40l26->dev, "%s\n", __func__);

	pm_runtime_get_sync(cs40l26->dev);
	mutex_lock(&cs40l26->lock);

	if (cs40l26->effect->u.periodic.waveform == FF_SINE) {
		ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				CS40L26_STOP_PLAYBACK, CS40L26_DSP_MBOX_RESET);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to stop playback\n");
			goto mutex_exit;
		}
	} else {
		if (cs40l26->fw_mode == CS40L26_FW_MODE_ROM)
			algo_id = CS40L26_VIBEGEN_ROM_ALGO_ID;
		else
			algo_id = CS40L26_VIBEGEN_ALGO_ID;

		ret = cl_dsp_get_reg(cs40l26->dsp, "END_PLAYBACK",
				CL_DSP_XM_UNPACKED_TYPE, algo_id, &reg);
		if (ret)
			goto mutex_exit;

		ret = regmap_write(cs40l26->regmap, reg, 1);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to end VIBE playback\n");
			goto mutex_exit;
		}

		ret = regmap_write(cs40l26->regmap, reg, 0);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to reset END_PLAYBACK\n");
			goto mutex_exit;
		}
	}

	cs40l26_vibe_state_set(cs40l26, CS40L26_VIBE_STATE_STOPPED);

mutex_exit:
	mutex_unlock(&cs40l26->lock);
	pm_runtime_mark_last_busy(cs40l26->dev);
	pm_runtime_put_autosuspend(cs40l26->dev);
}

static enum hrtimer_restart cs40l26_vibe_timer(struct hrtimer *timer)
{
	struct cs40l26_private *cs40l26 =
		container_of(timer, struct cs40l26_private, vibe_timer);

	queue_work(cs40l26->vibe_workqueue, &cs40l26->vibe_stop_work);

	return HRTIMER_NORESTART;
}

static void cs40l26_set_gain(struct input_dev *dev, u16 gain)
{
	struct cs40l26_private *cs40l26 = input_get_drvdata(dev);

	if (gain < 0 || gain >= CS40L26_NUM_PCT_MAP_VALUES) {
		dev_err(cs40l26->dev, "Gain value %u out of bounds\n", gain);
		return;
	}

	cs40l26->gain_pct = gain;

	queue_work(cs40l26->vibe_workqueue, &cs40l26->set_gain_work);
}

static int cs40l26_playback_effect(struct input_dev *dev,
		int effect_id, int val)
{
	struct cs40l26_private *cs40l26 = input_get_drvdata(dev);
	struct ff_effect *effect;

	dev_dbg(cs40l26->dev, "%s: effect ID = %d, val = %d\n", __func__,
			effect_id, val);

	effect = &dev->ff->effects[effect_id];
	if (!effect) {
		dev_err(cs40l26->dev, "No such effect to playback\n");
		return -EINVAL;
	}

	cs40l26->effect = effect;

	if (val > 0) {
		queue_work(cs40l26->vibe_workqueue, &cs40l26->vibe_start_work);
	} else {
		hrtimer_cancel(&cs40l26->vibe_timer);
		queue_work(cs40l26->vibe_workqueue, &cs40l26->vibe_stop_work);
	}

	return 0;
}

static int cs40l26_owt_get_wlength(struct cs40l26_private *cs40l26, u8 index)
{
	struct device *dev = cs40l26->dev;
	struct cl_dsp_owt_header *entry;
	struct cl_dsp_memchunk ch;

	if (index == 0)
		return 0;

	entry = &cs40l26->dsp->wt_desc->owt.waves[index];

	switch (entry->type) {
	case WT_TYPE_V6_PCM_F0_REDC:
	case WT_TYPE_V6_PCM_F0_REDC_VAR:
	case WT_TYPE_V6_PWLE:
		break;
	default:
		dev_err(dev, "Cannot size waveform type %u\n", entry->type);
		return -EINVAL;
	}

	ch = cl_dsp_memchunk_create(entry->data, sizeof(u32));

	/* First 24 bits of each waveform is the length in samples @ 8 kHz */
	return cl_dsp_memchunk_read(&ch, 24);
}

static void cs40l26_owt_get_section_info(struct cs40l26_private *cs40l26,
		struct cl_dsp_memchunk *ch,
		struct cs40l26_owt_section *sections, u8 nsections)
{
	int i;

	for (i = 0; i < nsections; i++) {
		cl_dsp_memchunk_read(ch, 8); /* Skip amplitude */
		sections[i].index = cl_dsp_memchunk_read(ch, 8);
		sections[i].repeat = cl_dsp_memchunk_read(ch, 8);
		sections[i].flags = cl_dsp_memchunk_read(ch, 8);
		sections[i].delay = cl_dsp_memchunk_read(ch, 16);

		if (sections[i].flags & CS40L26_WT_TYPE10_COMP_DURATION_FLAG) {
			cl_dsp_memchunk_read(ch, 8); /* Skip padding */
			sections[i].duration = cl_dsp_memchunk_read(ch, 16);
		}
	}
}

static int cs40l26_owt_calculate_wlength(struct cs40l26_private *cs40l26,
		struct cl_dsp_memchunk *ch)
{
	u32 total_len = 0, section_len = 0, loop_len = 0;
	bool in_loop = false;
	struct cs40l26_owt_section *sections;
	int ret = 0, i, wlen_whole;
	u8 nsections, global_rep;
	u32 dlen, wlen;

	cl_dsp_memchunk_read(ch, 8); /* Skip padding */
	nsections = cl_dsp_memchunk_read(ch, 8);
	global_rep = cl_dsp_memchunk_read(ch, 8);

	if (nsections < 1) {
		dev_err(cs40l26->dev, "Not enough sections for composite\n");
		return -EINVAL;
	}

	sections = kcalloc(nsections, sizeof(struct cs40l26_owt_section),
			GFP_KERNEL);
	if (!sections) {
		dev_err(cs40l26->dev, "Failed to allocate OWT sections\n");
		return -ENOMEM;
	}

	cs40l26_owt_get_section_info(cs40l26, ch, sections, nsections);

	for (i = 0; i < nsections; i++) {
		wlen_whole = cs40l26_owt_get_wlength(cs40l26,
				sections[i].index);
		if (wlen_whole < 0) {
			dev_err(cs40l26->dev,
					"Failed to get wlength for index %u\n",
					sections[i].index);
			ret = wlen_whole;
			goto err_free;
		}

		if (wlen_whole & CS40L26_WT_TYPE10_WAVELEN_INDEF) {
			if (!(sections[i].flags &
					CS40L26_WT_TYPE10_COMP_DURATION_FLAG)) {
				dev_err(cs40l26->dev,
					"Indefinite entry needs duration\n");
				ret = -EINVAL;
				goto err_free;
			}

			wlen = CS40L26_WT_TYPE10_WAVELEN_MAX;
		} else {
			/* Length is 22 LSBs, filter out flags */
			wlen = wlen_whole & CS40L26_WT_TYPE10_WAVELEN_MAX;
		}

		dlen = 8 * sections[i].delay;

		if (sections[i].flags & CS40L26_WT_TYPE10_COMP_DURATION_FLAG) {
			if (wlen > (2 * sections[i].duration))
				wlen = 2 * sections[i].duration;
		}

		section_len = wlen + dlen;
		loop_len += section_len;

		if (sections[i].repeat == 0xFF) {
			in_loop = true;
		} else if (sections[i].repeat) {
			total_len += (loop_len * (sections[i].repeat + 1));

			in_loop = false;
			loop_len = 0;
		} else if (!in_loop) {
			total_len += section_len;
			loop_len = 0;
		}

		section_len = 0;
	}

	cs40l26->owt_wlength = (total_len * (global_rep + 1)) |
			CS40L26_WT_TYPE10_WAVELEN_CALCULATED;

err_free:
	kfree(sections);

	return ret;
}

static int cs40l26_owt_upload(struct cs40l26_private *cs40l26, s16 *data,
		u32 data_size)
{
	bool pwle = (data[0] == 0x0000) ? false : true;
	u32 data_size_bytes = data_size * 2;
	struct device *dev = cs40l26->dev;
	struct cl_dsp *dsp = cs40l26->dsp;
	u32 full_data_size, header_size = CL_DSP_OWT_HEADER_ENTRY_SIZE;
	unsigned int write_reg, reg, wt_offset, wt_size, wt_base;
	struct cl_dsp_memchunk header_ch, data_ch;
	u8 *full_data, *header;
	int ret = 0;

	data_ch = cl_dsp_memchunk_create((void *) data, data_size_bytes);

	if (pwle) {
		header_size += CS40L26_WT_TERM_SIZE;

		cs40l26->owt_wlength = cl_dsp_memchunk_read(&data_ch, 24);
	} else {
		header_size += CS40L26_WT_WLEN_TERM_SIZE;

		ret = cs40l26_owt_calculate_wlength(cs40l26, &data_ch);
		if (ret)
			return ret;
	}
	full_data_size = header_size + data_size_bytes;

	header = kcalloc(header_size, sizeof(u8), GFP_KERNEL);
	if (!header)
		return -ENOMEM;

	header_ch = cl_dsp_memchunk_create((void *) header, header_size);
	/* Header */
	cl_dsp_memchunk_write(&header_ch, 16,
			CS40L26_WT_HEADER_DEFAULT_FLAGS);

	if (pwle)
		cl_dsp_memchunk_write(&header_ch, 8, WT_TYPE_V6_PWLE);
	else
		cl_dsp_memchunk_write(&header_ch, 8, WT_TYPE_V6_COMPOSITE);

	cl_dsp_memchunk_write(&header_ch, 24, CS40L26_WT_HEADER_OFFSET);
	cl_dsp_memchunk_write(&header_ch, 24, full_data_size /
					CL_DSP_BYTES_PER_WORD);

	cl_dsp_memchunk_write(&header_ch, 24, CS40L26_WT_HEADER_TERM);

	if (!pwle) /* Wlength is included in PWLE raw data */
		cl_dsp_memchunk_write(&header_ch, 24, cs40l26->owt_wlength);

	full_data = kcalloc(full_data_size, sizeof(u8), GFP_KERNEL);
	if (!full_data) {
		ret = -ENOMEM;
		goto err_free;
	}

	memcpy(full_data, header, header_ch.bytes);
	memcpy(full_data + header_ch.bytes, data, data_size_bytes);

	pm_runtime_get_sync(dev);

	ret = cl_dsp_get_reg(dsp, "OWT_NEXT_XM", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		goto err_pm;

	ret = regmap_read(cs40l26->regmap, reg, &wt_offset);
	if (ret) {
		dev_err(dev, "Failed to get wavetable offset\n");
		goto err_pm;
	}

	ret = cl_dsp_get_reg(dsp, "OWT_SIZE_XM", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		goto err_pm;

	ret = regmap_read(cs40l26->regmap, reg, &wt_size);
	if (ret) {
		dev_err(dev, "Failed to get available WT size\n");
		goto err_pm;
	}

	if (wt_size < full_data_size) {
		dev_err(dev, "No space for OWT waveform\n");
		ret = -ENOSPC;
		goto err_pm;
	}

	ret = cl_dsp_get_reg(dsp, CS40L26_WT_NAME_XM, CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &wt_base);
	if (ret)
		goto err_pm;

	write_reg = wt_base + (wt_offset * 4);

	ret = cl_dsp_raw_write(cs40l26->dsp, write_reg, full_data,
			full_data_size, CL_DSP_MAX_WLEN);
	if (ret) {
		dev_err(dev, "Failed to sync OWT\n");
		goto err_pm;
	}

	ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
			CS40L26_DSP_MBOX_CMD_OWT_PUSH, CS40L26_DSP_MBOX_RESET);
	if (ret)
		goto err_pm;

	dev_dbg(dev, "Successfully wrote waveform (%u bytes) to 0x%08X\n",
			full_data_size, write_reg);

err_pm:
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

err_free:
	kfree(header);
	kfree(full_data);

	return ret;
}

static int cs40l26_upload_effect(struct input_dev *dev,
		struct ff_effect *effect, struct ff_effect *old)
{
	struct cs40l26_private *cs40l26 = input_get_drvdata(dev);
	struct device *cdev = cs40l26->dev;
	s16 *raw_custom_data = NULL;
	int ret = 0;
	u32 trigger_index, min_index, max_index;
	u16 index, bank;

	if (effect->type != FF_PERIODIC) {
		dev_err(cdev, "Effect type 0x%X not supported\n",
				effect->type);
		return -EINVAL;
	}

	if (effect->replay.length < 0 ||
			effect->replay.length > CS40L26_TIMEOUT_MS_MAX) {
		dev_err(cdev, "Invalid playback duration: %d ms\n",
				effect->replay.length);
			return -EINVAL;
	}

	switch (effect->u.periodic.waveform) {
	case FF_CUSTOM:
		raw_custom_data =
				kzalloc(sizeof(s16) *
				effect->u.periodic.custom_len, GFP_KERNEL);
		if (!raw_custom_data)
			return -ENOMEM;

		if (copy_from_user(raw_custom_data,
				effect->u.periodic.custom_data,
				sizeof(s16) * effect->u.periodic.custom_len)) {
			dev_err(cdev, "Failed to get user data\n");
			ret = -EFAULT;
			goto out_free;
		}

		if (effect->u.periodic.custom_len > CS40L26_CUSTOM_DATA_SIZE) {
			ret = cs40l26_ack_write(cs40l26,
					CS40L26_DSP_VIRTUAL1_MBOX_1,
					CS40L26_DSP_MBOX_CMD_OWT_RESET,
					CS40L26_DSP_MBOX_RESET);
			if (ret)
				goto out_free;

			ret = cs40l26_owt_upload(cs40l26, raw_custom_data,
					effect->u.periodic.custom_len);
			if (ret)
				goto out_free;

			bank = CS40L26_OWT_BANK_ID;
			index = cs40l26->num_owt_effects;
		} else {
			bank = ((u16) raw_custom_data[0]);
			index = ((u16) raw_custom_data[1]) &
					CS40L26_MAX_INDEX_MASK;
		}

		switch (bank) {
		case CS40L26_RAM_BANK_ID:
			min_index = CS40L26_RAM_INDEX_START;
			max_index = min_index + cs40l26->num_waves - 1;
			break;
		case CS40L26_ROM_BANK_ID:
			min_index = CS40L26_ROM_INDEX_START;
			max_index = CS40L26_ROM_INDEX_END;
			break;
		case CS40L26_OWT_BANK_ID:
			min_index = CS40L26_OWT_INDEX_START;
			max_index = CS40L26_OWT_INDEX_END;
			break;
		default:
			dev_err(cdev, "Bank ID (%u) out of bounds\n", bank);
			ret = -EINVAL;
			goto out_free;
		}

		trigger_index = index + min_index;

		if (trigger_index >= min_index && trigger_index <= max_index) {
			cs40l26->trigger_indices[effect->id] = trigger_index;
		} else {
			dev_err(cdev, "Trigger index (0x%X) out of bounds\n",
					trigger_index);
			ret = -EINVAL;
			goto out_free;
		}

		dev_dbg(cdev, "%s: ID = %u, index = 0x%08X\n",
				__func__, effect->id, trigger_index);
		break;
	case FF_SINE:
		if (effect->u.periodic.period) {
			if (effect->u.periodic.period
					< CS40L26_BUZZGEN_PERIOD_MIN
					|| effect->u.periodic.period
					> CS40L26_BUZZGEN_PERIOD_MAX) {
				dev_err(cdev,
				"%u ms period not within range (4-10 ms)\n",
				effect->u.periodic.period);
				return -EINVAL;
			}
		} else {
			dev_err(cdev, "Sine wave period not specified\n");
			return -EINVAL;
		}
		break;
	default:
		dev_err(cdev, "Periodic waveform type 0x%X not supported\n",
				effect->u.periodic.waveform);
		return -EINVAL;
	}

out_free:
	kfree(raw_custom_data);

	return ret;
}

static int cs40l26_input_init(struct cs40l26_private *cs40l26)
{
	int ret;
	struct device *dev = cs40l26->dev;

	cs40l26->input = devm_input_allocate_device(dev);
	if (!cs40l26->input)
		return -ENOMEM;

	cs40l26->input->name = "cs40l26_input";
	cs40l26->input->id.product = cs40l26->devid;
	cs40l26->input->id.version = cs40l26->revid;

	input_set_drvdata(cs40l26->input, cs40l26);
	input_set_capability(cs40l26->input, EV_FF, FF_PERIODIC);
	input_set_capability(cs40l26->input, EV_FF, FF_CUSTOM);
	input_set_capability(cs40l26->input, EV_FF, FF_SINE);
	input_set_capability(cs40l26->input, EV_FF, FF_GAIN);

	ret = input_ff_create(cs40l26->input, FF_MAX_EFFECTS);
	if (ret) {
		dev_err(dev, "Failed to create FF device: %d\n", ret);
		return ret;
	}

	/*
	 * input_ff_create() automatically sets FF_RUMBLE capabilities;
	 * we want to restrtict this to only FF_PERIODIC
	 */
	__clear_bit(FF_RUMBLE, cs40l26->input->ffbit);

	cs40l26->input->ff->upload = cs40l26_upload_effect;
	cs40l26->input->ff->playback = cs40l26_playback_effect;
	cs40l26->input->ff->set_gain = cs40l26_set_gain;

	ret = input_register_device(cs40l26->input);
	if (ret) {
		dev_err(dev, "Cannot register input device: %d\n", ret);
		return ret;
	}

	hrtimer_init(&cs40l26->vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cs40l26->vibe_timer.function = cs40l26_vibe_timer;

	ret = sysfs_create_group(&cs40l26->input->dev.kobj,
			&cs40l26_dev_attr_group);
	if (ret) {
		dev_err(dev, "Failed to create sysfs group: %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&cs40l26->input->dev.kobj,
			&cs40l26_dev_attr_cal_group);
	if (ret) {
		dev_err(dev, "Failed to create cal sysfs group: %d\n", ret);
		return ret;
	}

	cs40l26->vibe_init_success = true;

	return ret;
}

static int cs40l26_part_num_resolve(struct cs40l26_private *cs40l26)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	int ret;
	u32 val;

	ret = regmap_read(regmap, CS40L26_DEVID, &val);
	if (ret) {
		dev_err(dev, "Failed to read device ID\n");
		return ret;
	}

	val &= CS40L26_DEVID_MASK;
	if (val != CS40L26_DEVID_A && val != CS40L26_DEVID_B) {
		dev_err(dev, "Invalid device ID: 0x%06X\n", val);
		return -EINVAL;
	}

	cs40l26->devid = val;

	ret = regmap_read(regmap, CS40L26_REVID, &val);
	if (ret) {
		dev_err(dev, "Failed to read revision ID\n");
		return ret;
	}

	val &= CS40L26_REVID_MASK;
	switch (val) {
	case CS40L26_REVID_A0:
	case CS40L26_REVID_A1:
		cs40l26->revid = val;
		break;
	default:
		dev_err(dev, "Invalid device revision: 0x%02X\n", val);
		return -EINVAL;
	}

	dev_info(dev, "Cirrus Logic %s ID: 0x%06X, Revision: 0x%02X\n",
			CS40L26_DEV_NAME, cs40l26->devid, cs40l26->revid);

	return 0;
}

static int cs40l26_cl_dsp_init(struct cs40l26_private *cs40l26)
{
	int ret = 0;

	cs40l26->dsp = cl_dsp_create(cs40l26->dev, cs40l26->regmap);
	if (!cs40l26->dsp)
		return -ENOMEM;

	if (cs40l26->fw_mode == CS40L26_FW_MODE_ROM) {
		cs40l26->fw.min_rev = CS40L26_FW_ROM_MIN_REV;
		cs40l26->fw.num_coeff_files = 0;
		cs40l26->fw.coeff_files = NULL;
	} else {
		if (cs40l26->revid == CS40L26_REVID_A1)
			cs40l26->fw.min_rev = CS40L26_FW_A1_RAM_MIN_REV;
		else
			cs40l26->fw.min_rev = CS40L26_FW_A0_RAM_MIN_REV;

		cs40l26->fw.num_coeff_files =
				ARRAY_SIZE(cs40l26_ram_coeff_files);
		cs40l26->fw.coeff_files = cs40l26_ram_coeff_files;

		ret = cl_dsp_wavetable_create(cs40l26->dsp,
				CS40L26_VIBEGEN_ALGO_ID, CS40L26_WT_NAME_XM,
				CS40L26_WT_NAME_YM, "cs40l26.bin");
	}

	cs40l26->num_owt_effects = 0;

	return ret;
}

static int cs40l26_wksrc_config(struct cs40l26_private *cs40l26)
{
	u32 unmask_bits, mask_bits;
	int ret;

	unmask_bits = BIT(CS40L26_IRQ1_WKSRC_STS_ANY) |
			BIT(CS40L26_IRQ1_WKSRC_STS_GPIO1) |
			BIT(CS40L26_IRQ1_WKSRC_STS_I2C);

	/* SPI support is not yet available */
	mask_bits = BIT(CS40L26_IRQ1_WKSRC_STS_SPI);

	if (cs40l26->devid == CS40L26_DEVID_A)
		mask_bits |= (BIT(CS40L26_IRQ1_WKSRC_STS_GPIO2) |
				BIT(CS40L26_IRQ1_WKSRC_STS_GPIO3) |
				BIT(CS40L26_IRQ1_WKSRC_STS_GPIO4));
	else
		unmask_bits |= (BIT(CS40L26_IRQ1_WKSRC_STS_GPIO2) |
				BIT(CS40L26_IRQ1_WKSRC_STS_GPIO3) |
				BIT(CS40L26_IRQ1_WKSRC_STS_GPIO4));

	ret = cs40l26_irq_update_mask(cs40l26, CS40L26_IRQ1_MASK_1, mask_bits,
			CS40L26_IRQ_MASK);
	if (ret)
		return ret;

	return cs40l26_irq_update_mask(cs40l26, CS40L26_IRQ1_MASK_1,
			unmask_bits, CS40L26_IRQ_UNMASK);
}

static int cs40l26_gpio_config(struct cs40l26_private *cs40l26)
{
	u32 unmask_bits;

	unmask_bits = BIT(CS40L26_IRQ1_GPIO1_RISE)
			| BIT(CS40L26_IRQ1_GPIO1_FALL);

	if (cs40l26->devid == CS40L26_DEVID_B) /* 4 GPIO config */
		unmask_bits |= (u32) (GENMASK(CS40L26_IRQ1_GPIO4_FALL,
				CS40L26_IRQ1_GPIO2_RISE));

	return cs40l26_irq_update_mask(cs40l26, CS40L26_IRQ1_MASK_1,
			unmask_bits, CS40L26_IRQ_UNMASK);
}

static int cs40l26_brownout_prevention_init(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	struct regmap *regmap = cs40l26->regmap;
	u32 vbbr_thld = 0, vpbr_thld = 0;
	u32 vbbr_max_att = 0, vpbr_max_att = 0;
	u32 vpbr_atk_step = 0, vbbr_atk_step = 0;
	u32 vpbr_atk_rate = 0, vbbr_atk_rate = 0;
	u32 vpbr_wait = 0, vbbr_wait = 0;
	u32 vpbr_rel_rate = 0, vbbr_rel_rate = 0;
	u32 val;
	int ret;

	ret = regmap_read(regmap, CS40L26_BLOCK_ENABLES2, &val);
	if (ret) {
		dev_err(dev, "Failed to read block enables 2\n");
		return ret;
	}

	val |= ((cs40l26->pdata.vbbr_en << CS40L26_VBBR_EN_SHIFT)
			| (cs40l26->pdata.vpbr_en << CS40L26_VPBR_EN_SHIFT));

	ret = regmap_write(regmap, CS40L26_BLOCK_ENABLES2, val);
	if (ret) {
		dev_err(dev, "Failed to enable brownout prevention\n");
		return ret;
	}

	switch (cs40l26->revid) {
	case CS40L26_REVID_A0:
		ret = cs40l26_pseq_v1_add_pair(cs40l26,
			CS40L26_BLOCK_ENABLES2, val, CS40L26_PSEQ_V1_REPLACE);
		break;
	case CS40L26_REVID_A1:
		ret = cs40l26_pseq_v2_add_write_reg_full(cs40l26,
			CS40L26_BLOCK_ENABLES2, val, true);
		break;
	default:
		dev_err(cs40l26->dev, "Revid ID not supported: %02X\n",
			cs40l26->revid);
		return -EINVAL;
	}

	if (ret) {
		dev_err(dev, "Failed to sequence brownout prevention\n");
		return ret;
	}

	if (cs40l26->pdata.vbbr_en) {
		ret = cs40l26_irq_update_mask(cs40l26, CS40L26_IRQ1_MASK_2,
				BIT(CS40L26_IRQ2_VBBR_ATT_CLR) |
				BIT(CS40L26_IRQ2_VBBR_FLAG),
				CS40L26_IRQ_UNMASK);
		if (ret)
			return ret;

		ret = regmap_read(regmap, CS40L26_VBBR_CONFIG, &val);
		if (ret) {
			dev_err(dev, "Failed to get VBBR config.\n");
			return ret;
		}

		if (cs40l26->pdata.vbbr_thld) {
			if (cs40l26->pdata.vbbr_thld
					>= CS40L26_VBBR_THLD_MV_MAX)
				vbbr_thld = CS40L26_VBBR_THLD_MAX;
			else if (cs40l26->pdata.vbbr_thld
					<= CS40L26_VBBR_THLD_MV_MIN)
				vbbr_thld = CS40L26_VBBR_THLD_MIN;
			else
				vbbr_thld = cs40l26->pdata.vbbr_thld /
					CS40L26_VBBR_THLD_MV_STEP;

			val &= ~CS40L26_VBBR_THLD_MASK;
			val |= (vbbr_thld & CS40L26_VBBR_THLD_MASK);
		}

		if (cs40l26->pdata.vbbr_max_att != CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vbbr_max_att >=
					CS40L26_VXBR_MAX_ATT_MAX)
				vbbr_max_att = CS40L26_VXBR_MAX_ATT_MAX;
			else
				vbbr_max_att = cs40l26->pdata.vbbr_max_att;

			val &= ~CS40L26_VXBR_MAX_ATT_MASK;
			val |= ((vbbr_max_att << CS40L26_VXBR_MAX_ATT_SHIFT)
					& CS40L26_VXBR_MAX_ATT_MASK);
		}

		if (cs40l26->pdata.vbbr_atk_step) {
			if (cs40l26->pdata.vbbr_atk_step
					<= CS40L26_VXBR_ATK_STEP_MIN)
				vbbr_atk_step = CS40L26_VXBR_ATK_STEP_MIN;
			else if (cs40l26->pdata.vbbr_atk_step
					>= CS40L26_VXBR_ATK_STEP_MAX_DB)
				vbbr_atk_step = CS40L26_VXBR_ATK_STEP_MAX;
			else
				vbbr_atk_step = cs40l26->pdata.vbbr_atk_step;

			val &= ~CS40L26_VXBR_ATK_STEP_MASK;
			val |= ((vbbr_atk_step << CS40L26_VXBR_ATK_STEP_SHIFT)
					& CS40L26_VXBR_ATK_STEP_MASK);
		}

		if (cs40l26->pdata.vbbr_atk_rate !=
				CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vbbr_atk_rate
					> CS40L26_VXBR_ATK_RATE_MAX)
				vbbr_atk_rate = CS40L26_VXBR_ATK_RATE_MAX;
			else
				vbbr_atk_rate = cs40l26->pdata.vbbr_atk_rate;

			val &= ~CS40L26_VXBR_ATK_RATE_MASK;
			val |= ((vbbr_atk_rate << CS40L26_VXBR_ATK_RATE_SHIFT)
					& CS40L26_VXBR_ATK_RATE_MASK);
		}

		if (cs40l26->pdata.vbbr_wait != CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vbbr_wait > CS40L26_VXBR_WAIT_MAX)
				vbbr_wait = CS40L26_VXBR_WAIT_MAX;
			else
				vbbr_wait = cs40l26->pdata.vbbr_wait;

			val &= ~CS40L26_VXBR_WAIT_MASK;
			val |= ((vbbr_wait << CS40L26_VXBR_WAIT_SHIFT)
					& CS40L26_VXBR_WAIT_MASK);
		}

		if (cs40l26->pdata.vbbr_rel_rate != CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vbbr_rel_rate
					> CS40L26_VXBR_REL_RATE_MAX)
				vbbr_rel_rate = CS40L26_VXBR_REL_RATE_MAX;
			else
				vbbr_rel_rate = cs40l26->pdata.vbbr_rel_rate;

			val &= ~CS40L26_VXBR_REL_RATE_MASK;
			val |= ((vbbr_rel_rate << CS40L26_VXBR_REL_RATE_SHIFT)
					& CS40L26_VXBR_REL_RATE_MASK);
		}

		ret = regmap_write(regmap, CS40L26_VBBR_CONFIG, val);
		if (ret) {
			dev_err(dev, "Failed to write VBBR config.\n");
			return ret;
		}

		switch (cs40l26->revid) {
		case CS40L26_REVID_A0:
			ret = cs40l26_pseq_v1_add_pair(cs40l26,
				CS40L26_VBBR_CONFIG, val,
				CS40L26_PSEQ_V1_REPLACE);
			break;
		case CS40L26_REVID_A1:
			ret = cs40l26_pseq_v2_add_write_reg_full(cs40l26,
				CS40L26_VBBR_CONFIG, val,
				true);
			break;
		default:
			dev_err(cs40l26->dev, "Revid ID not supported: %02X\n",
				cs40l26->revid);
			return -EINVAL;
		}
		if (ret)
			return ret;
	}

	ret = regmap_read(regmap, CS40L26_VPBR_CONFIG, &val);
	if (ret) {
		dev_err(dev, "Failed to get VBBR config.\n");
		return ret;
	}

	if (cs40l26->pdata.vpbr_en) {
		ret = cs40l26_irq_update_mask(cs40l26, CS40L26_IRQ1_MASK_2,
				BIT(CS40L26_IRQ2_VPBR_ATT_CLR) |
				BIT(CS40L26_IRQ2_VPBR_FLAG),
				CS40L26_IRQ_UNMASK);
		if (ret)
			return ret;

		if (cs40l26->pdata.vpbr_thld) {
			if (cs40l26->pdata.vpbr_thld
					>= CS40L26_VPBR_THLD_MV_MAX)
				vpbr_thld = CS40L26_VPBR_THLD_MAX;
			else if (cs40l26->pdata.vpbr_thld
					<= CS40L26_VPBR_THLD_MV_MIN)
				vpbr_thld = CS40L26_VPBR_THLD_MIN;
			else
				vpbr_thld = (cs40l26->pdata.vpbr_thld /
						CS40L26_VPBR_THLD_MV_DIV)
						- CS40L26_VPBR_THLD_OFFSET;

			val &= ~CS40L26_VPBR_THLD_MASK;
			val |= (vpbr_thld & CS40L26_VPBR_THLD_MASK);

		}

		if (cs40l26->pdata.vpbr_max_att != CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vpbr_max_att >=
					CS40L26_VXBR_MAX_ATT_MAX)
				vpbr_max_att = CS40L26_VXBR_MAX_ATT_MAX;
			else
				vpbr_max_att = cs40l26->pdata.vpbr_max_att;

			val &= ~CS40L26_VXBR_MAX_ATT_MASK;
			val |= ((vpbr_max_att << CS40L26_VXBR_MAX_ATT_SHIFT)
					& CS40L26_VXBR_MAX_ATT_MASK);
		}

		if (cs40l26->pdata.vpbr_atk_step) {
			if (cs40l26->pdata.vpbr_atk_step
					<= CS40L26_VXBR_ATK_STEP_MIN)
				vpbr_atk_step = CS40L26_VXBR_ATK_STEP_MIN;
			else if (cs40l26->pdata.vpbr_atk_step
					>= CS40L26_VXBR_ATK_STEP_MAX_DB)
				vpbr_atk_step = CS40L26_VXBR_ATK_STEP_MAX;
			else
				vpbr_atk_step = cs40l26->pdata.vpbr_atk_step;

			val &= ~CS40L26_VXBR_ATK_STEP_MASK;
			val |= ((vpbr_atk_step << CS40L26_VXBR_ATK_STEP_SHIFT)
					& CS40L26_VXBR_ATK_STEP_MASK);
		}

		if (cs40l26->pdata.vpbr_atk_rate !=
				CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vpbr_atk_rate
					> CS40L26_VXBR_ATK_RATE_MAX)
				vpbr_atk_rate = CS40L26_VXBR_ATK_RATE_MAX;
			else
				vpbr_atk_rate = cs40l26->pdata.vpbr_atk_rate;

			val &= ~CS40L26_VXBR_ATK_RATE_MASK;
			val |= ((vpbr_atk_rate << CS40L26_VXBR_ATK_RATE_SHIFT)
					& CS40L26_VXBR_ATK_RATE_MASK);

		}

		if (cs40l26->pdata.vpbr_wait != CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vpbr_wait > CS40L26_VXBR_WAIT_MAX)
				vpbr_wait = CS40L26_VXBR_WAIT_MAX;
			else
				vpbr_wait = cs40l26->pdata.vpbr_wait;

			val &= ~CS40L26_VXBR_WAIT_MASK;
			val |= ((vpbr_wait << CS40L26_VXBR_WAIT_SHIFT)
					& CS40L26_VXBR_WAIT_MASK);
		}

		if (cs40l26->pdata.vpbr_rel_rate != CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vpbr_rel_rate
					> CS40L26_VXBR_REL_RATE_MAX)
				vpbr_rel_rate = CS40L26_VXBR_REL_RATE_MAX;
			else
				vpbr_rel_rate = cs40l26->pdata.vpbr_rel_rate;

			val &= ~CS40L26_VXBR_REL_RATE_MASK;
			val |= ((vpbr_rel_rate << CS40L26_VXBR_REL_RATE_SHIFT)
					& CS40L26_VXBR_REL_RATE_MASK);
		}

		ret = regmap_write(regmap, CS40L26_VPBR_CONFIG, val);
		if (ret) {
			dev_err(dev, "Failed to write VPBR config.\n");
			return ret;
		}

		switch (cs40l26->revid) {
		case CS40L26_REVID_A0:
			ret = cs40l26_pseq_v1_add_pair(cs40l26,
				CS40L26_VPBR_CONFIG, val,
				CS40L26_PSEQ_V1_REPLACE);
			break;
		case CS40L26_REVID_A1:
			ret = cs40l26_pseq_v2_add_write_reg_full(cs40l26,
				CS40L26_VPBR_CONFIG, val,
				true);
			break;
		default:
			dev_err(cs40l26->dev, "Revid ID not supported: %02X\n",
				cs40l26->revid);
			return -EINVAL;
		}
		if (ret)
			return ret;
	}

	return 0;
}

static int cs40l26_get_num_waves(struct cs40l26_private *cs40l26,
		u32 *num_waves)
{
	int ret;
	u32 reg;

	ret = cl_dsp_get_reg(cs40l26->dsp, "NUM_OF_WAVES",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		return ret;

	return regmap_read(cs40l26->regmap, reg, num_waves);
}

static int cs40l26_verify_fw(struct cs40l26_private *cs40l26)
{
	struct cs40l26_fw *fw = &cs40l26->fw;
	unsigned int val;
	int ret;

	ret = cl_dsp_fw_id_get(cs40l26->dsp, &val);
	if (ret)
		return ret;

	if (val != CS40L26_FW_ID) {
		dev_err(cs40l26->dev, "Invalid firmware ID: 0x%X\n", val);
		return -EINVAL;
	}

	ret = cl_dsp_fw_rev_get(cs40l26->dsp, &val);
	if (ret)
		return ret;

	if (val < fw->min_rev) {
		dev_err(cs40l26->dev, "Invalid firmware revision: %d.%d.%d\n",
			(int) CL_DSP_GET_MAJOR(val),
			(int) CL_DSP_GET_MINOR(val),
			(int) CL_DSP_GET_PATCH(val));
		return -EINVAL;
	}

	dev_info(cs40l26->dev, "Firmware revision %d.%d.%d\n",
			(int) CL_DSP_GET_MAJOR(val),
			(int) CL_DSP_GET_MINOR(val),
			(int) CL_DSP_GET_PATCH(val));

	return 0;
}

static int cs40l26_asp_config(struct cs40l26_private *cs40l26)
{
	struct reg_sequence *dsp1rx_config =
			kcalloc(2, sizeof(struct reg_sequence), GFP_KERNEL);
	int ret;

	if (!dsp1rx_config) {
		dev_err(cs40l26->dev, "Failed to allocate reg. sequence\n");
		return -ENOMEM;
	}

	dsp1rx_config[0].reg = CS40L26_DSP1RX1_INPUT;
	dsp1rx_config[0].def = CS40L26_DATA_SRC_ASPRX1;
	dsp1rx_config[1].reg = CS40L26_DSP1RX5_INPUT;
	dsp1rx_config[1].def = CS40L26_DATA_SRC_ASPRX2;

	ret = regmap_multi_reg_write(cs40l26->regmap, dsp1rx_config, 2);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to configure ASP\n");
		goto err_free;
	}

	switch (cs40l26->revid) {
	case CS40L26_REVID_A0:
		ret = cs40l26_pseq_v1_multi_add_pair(cs40l26, dsp1rx_config, 2,
				CS40L26_PSEQ_V1_REPLACE);
		break;
	case CS40L26_REVID_A1:
		ret = cs40l26_pseq_v2_multi_add_write_reg_full(cs40l26,
				dsp1rx_config, 2, true);
		break;
	default:
		dev_err(cs40l26->dev, "Revid ID not supported: %02X\n",
			cs40l26->revid);
		ret = -EINVAL;
		goto err_free;
	}

	if (ret)
		dev_err(cs40l26->dev, "Failed to add ASP config to pseq\n");

err_free:
	kfree(dsp1rx_config);

	return ret;
}

static int cs40l26_bst_dcm_config(struct cs40l26_private *cs40l26)
{
	int ret = 0;
	u32 val;

	if (cs40l26->pdata.bst_dcm_en != CS40L26_BST_DCM_EN_DEFAULT) {
		ret = regmap_read(cs40l26->regmap, CS40L26_BST_DCM_CTL, &val);
		if (ret)
			return ret;

		val &= ~CS40L26_BST_DCM_EN_MASK;
		val |= cs40l26->pdata.bst_dcm_en << CS40L26_BST_DCM_EN_SHIFT;

		ret = regmap_write(cs40l26->regmap, CS40L26_BST_DCM_CTL, val);
		if (ret)
			return ret;

		switch (cs40l26->revid) {
		case CS40L26_REVID_A0:
			ret = cs40l26_pseq_v1_add_pair(cs40l26,
						CS40L26_BST_DCM_CTL, val, true);
			break;
		case CS40L26_REVID_A1:
			ret = cs40l26_pseq_v2_add_write_reg_full(cs40l26,
						CS40L26_BST_DCM_CTL, val, true);
			break;
		default:
			dev_err(cs40l26->dev, "Revid ID not supported: %02X\n",
				cs40l26->revid);
			ret = -EINVAL;
		}
	}

	return ret;
}

static int cs40l26_bst_ipk_config(struct cs40l26_private *cs40l26)
{
	u32 val, bst_ipk_ma = cs40l26->pdata.bst_ipk / 1000;
	int ret;

	if (bst_ipk_ma < CS40L26_BST_IPK_MILLIAMP_MIN ||
			bst_ipk_ma > CS40L26_BST_IPK_MILLIAMP_MAX) {
		val = CS40L26_BST_IPK_DEFAULT;
		dev_dbg(cs40l26->dev, "Using default BST_IPK\n");
	} else {
		val = (bst_ipk_ma / 50) - 16;
	}

	ret = regmap_write(cs40l26->regmap, CS40L26_BST_IPK_CTL, val);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to update BST peak current\n");
		return ret;
	}

	if (cs40l26->revid == CS40L26_REVID_A0)
		ret = cs40l26_pseq_v1_add_pair(cs40l26,
				CS40L26_BST_IPK_CTL, val, true);
	else
		ret = cs40l26_pseq_v2_add_write_reg_full(cs40l26,
				CS40L26_BST_IPK_CTL, val, true);

	return ret;
}

static int cs40l26_dsp_config(struct cs40l26_private *cs40l26)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned int val;
	int ret;
	u32 reg;

	ret = cs40l26_verify_fw(cs40l26);
	if (ret)
		goto err_out;

	ret = regmap_update_bits(regmap, CS40L26_PWRMGT_CTL,
			CS40L26_MEM_RDY_MASK,
			CS40L26_ENABLE << CS40L26_MEM_RDY_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to set MEM_RDY to initialize RAM\n");
		goto err_out;
	}

	if (cs40l26->fw_mode == CS40L26_FW_MODE_RAM) {
		ret = cl_dsp_get_reg(cs40l26->dsp, "CALL_RAM_INIT",
				CL_DSP_XM_UNPACKED_TYPE,
				CS40L26_FW_ID, &reg);
		if (ret)
			goto err_out;

		ret = cs40l26_dsp_write(cs40l26, reg, CS40L26_ENABLE);
		if (ret)
			goto err_out;
	}

	cs40l26->fw_loaded = true;

	ret = cs40l26_dsp_start(cs40l26);
	if (ret)
		goto err_out;

	ret = cs40l26_pseq_init(cs40l26);
	if (ret)
		goto err_out;

	ret = cs40l26_iseq_init(cs40l26);
	if (ret)
		goto err_out;

	ret = cs40l26_irq_update_mask(cs40l26, CS40L26_IRQ1_MASK_1,
			BIT(CS40L26_IRQ1_VIRTUAL2_MBOX_WR), CS40L26_IRQ_UNMASK);
	if (ret)
		goto err_out;

	ret = cs40l26_wksrc_config(cs40l26);
	if (ret)
		goto err_out;

	ret = cs40l26_gpio_config(cs40l26);
	if (ret)
		goto err_out;

	ret = cs40l26_bst_dcm_config(cs40l26);
	if (ret)
		goto err_out;

	ret = cs40l26_bst_ipk_config(cs40l26);
	if (ret)
		goto err_out;

	ret = cs40l26_brownout_prevention_init(cs40l26);
	if (ret)
		goto err_out;

	/* ensure firmware running */
	ret = cl_dsp_get_reg(cs40l26->dsp, "HALO_STATE",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_FW_ID,
			&reg);
	if (ret)
		goto err_out;

	ret = regmap_read(regmap, reg, &val);
	if (ret) {
		dev_err(dev, "Failed to read HALO_STATE\n");
		goto err_out;
	}

	if (val != CS40L26_DSP_HALO_STATE_RUN) {
		dev_err(dev, "Firmware in unexpected state: 0x%X\n", val);
		goto err_out;
	}

	ret = cs40l26_pm_runtime_setup(cs40l26);
	if (ret)
		goto err_out;

	pm_runtime_get_sync(dev);

	ret = cl_dsp_get_reg(cs40l26->dsp, "TIMEOUT_MS",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		goto pm_err;

	ret = regmap_write(regmap, reg, 0);
	if (ret) {
		dev_err(dev, "Failed to set TIMEOUT_MS\n");
		goto pm_err;
	}

	ret = cs40l26_asp_config(cs40l26);
	if (ret)
		goto pm_err;

	ret = cs40l26_get_num_waves(cs40l26, &cs40l26->num_waves);
	if (!ret)
		dev_info(dev, "%s loaded with %u RAM waveforms\n",
				CS40L26_DEV_NAME, cs40l26->num_waves);

pm_err:
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
err_out:
	enable_irq(cs40l26->irq);
	return ret;
}

static void cs40l26_firmware_load(const struct firmware *fw, void *context)
{
	struct cs40l26_private *cs40l26 = (struct cs40l26_private *)context;
	struct device *dev = cs40l26->dev;
	const struct firmware *coeff_fw;
	int ret, i;

	if (!fw) {
		dev_err(dev, "Failed to request firmware file\n");
		return;
	}

	cs40l26->pm_ready = false;

	ret = cl_dsp_firmware_parse(cs40l26->dsp, fw,
			cs40l26->fw_mode == CS40L26_FW_MODE_RAM);
	release_firmware(fw);
	if (ret)
		return;

	for (i = 0; i < cs40l26->fw.num_coeff_files; i++) {
		request_firmware(&coeff_fw, cs40l26->fw.coeff_files[i],
				dev);
		if (!coeff_fw) {
			dev_warn(dev, "Continuing...\n");
			continue;
		}

		if (cl_dsp_coeff_file_parse(cs40l26->dsp, coeff_fw))
			dev_warn(dev, "Continuing...\n");
		else
			dev_dbg(dev, "%s Loaded Successfully\n",
				cs40l26->fw.coeff_files[i]);
	}

	cs40l26_dsp_config(cs40l26);
}

static int cs40l26_handle_platform_data(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	struct device_node *np = dev->of_node;
	u32 val;

	if (!np) {
		dev_err(dev, "No platform data found\n");
		return -ENOENT;
	}

	if (of_property_read_bool(np, "cirrus,basic-config"))
		cs40l26->fw_mode = CS40L26_FW_MODE_ROM;
	else
		cs40l26->fw_mode = CS40L26_FW_MODE_RAM;

	cs40l26->pdata.vbbr_en =
			of_property_read_bool(np, "cirrus,vbbr-enable");

	if (!of_property_read_u32(np, "cirrus,vbbr-thld-mv", &val))
		cs40l26->pdata.vbbr_thld = val;

	if (!of_property_read_u32(np, "cirrus,vbbr-max-att-db", &val))
		cs40l26->pdata.vbbr_max_att = val;
	else
		cs40l26->pdata.vbbr_max_att = CS40L26_VXBR_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,vbbr-atk-step", &val))
		cs40l26->pdata.vbbr_atk_step = val;

	if (!of_property_read_u32(np, "cirrus,vbbr-atk-rate", &val))
		cs40l26->pdata.vbbr_atk_rate = val;

	if (!of_property_read_u32(np, "cirrus,vbbr-wait", &val))
		cs40l26->pdata.vbbr_wait = val;
	else
		cs40l26->pdata.vbbr_wait = CS40L26_VXBR_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,vbbr-rel-rate", &val))
		cs40l26->pdata.vbbr_rel_rate = val;
	else
		cs40l26->pdata.vbbr_rel_rate = CS40L26_VXBR_DEFAULT;

	cs40l26->pdata.vpbr_en =
			of_property_read_bool(np, "cirrus,vpbr-enable");

	if (!of_property_read_u32(np, "cirrus,vpbr-thld-mv", &val))
		cs40l26->pdata.vpbr_thld = val;

	if (!of_property_read_u32(np, "cirrus,vpbr-max-att-db", &val))
		cs40l26->pdata.vpbr_max_att = val;
	else
		cs40l26->pdata.vpbr_max_att = CS40L26_VXBR_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,vpbr-atk-step", &val))
		cs40l26->pdata.vpbr_atk_step = val;

	if (!of_property_read_u32(np, "cirrus,vpbr-atk-rate", &val))
		cs40l26->pdata.vpbr_atk_rate = val;
	else
		cs40l26->pdata.vpbr_atk_rate = CS40L26_VXBR_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,vpbr-wait", &val))
		cs40l26->pdata.vpbr_wait = val;
	else
		cs40l26->pdata.vpbr_wait = CS40L26_VXBR_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,vpbr-rel-rate", &val))
		cs40l26->pdata.vpbr_rel_rate = val;
	else
		cs40l26->pdata.vpbr_rel_rate = CS40L26_VXBR_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,bst-dcm-en", &val))
		cs40l26->pdata.bst_dcm_en = val;
	else
		cs40l26->pdata.bst_dcm_en = CS40L26_BST_DCM_EN_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,bst-ipk-microamp", &val))
		cs40l26->pdata.bst_ipk = val;
	else
		cs40l26->pdata.bst_ipk = 0;

	return 0;
}

int cs40l26_probe(struct cs40l26_private *cs40l26,
		struct cs40l26_platform_data *pdata)
{
	struct device *dev = cs40l26->dev;
	struct regulator *vp_consumer, *va_consumer;
	int ret;

	mutex_init(&cs40l26->lock);

	cs40l26->vibe_workqueue = alloc_ordered_workqueue("vibe_workqueue",
			WQ_HIGHPRI);
	if (!cs40l26->vibe_workqueue) {
		ret = -ENOMEM;
		goto err;
	}

	INIT_WORK(&cs40l26->vibe_start_work, cs40l26_vibe_start_worker);
	INIT_WORK(&cs40l26->vibe_stop_work, cs40l26_vibe_stop_worker);
	INIT_WORK(&cs40l26->set_gain_work, cs40l26_set_gain_worker);

	ret = devm_regulator_bulk_get(dev, CS40L26_NUM_SUPPLIES,
			cs40l26_supplies);
	if (ret) {
		dev_err(dev, "Failed to request core supplies: %d\n", ret);
		goto err;
	}

	vp_consumer = cs40l26_supplies[CS40L26_VP_SUPPLY].consumer;
	va_consumer = cs40l26_supplies[CS40L26_VA_SUPPLY].consumer;

	if (pdata) {
		cs40l26->pdata = *pdata;
	} else if (cs40l26->dev->of_node) {
		ret = cs40l26_handle_platform_data(cs40l26);
		if (ret)
			goto err;
	} else {
		dev_err(dev, "No platform data found\n");
		ret = -ENOENT;
		goto err;
	}

	ret = regulator_bulk_enable(CS40L26_NUM_SUPPLIES, cs40l26_supplies);
	if  (ret) {
		dev_err(dev, "Failed to enable core supplies\n");
		goto err;
	}

	cs40l26->reset_gpio = devm_gpiod_get_optional(dev, "reset",
			GPIOD_OUT_LOW);
	if (IS_ERR(cs40l26->reset_gpio)) {
		dev_err(dev, "Failed to get reset GPIO\n");

		ret = PTR_ERR(cs40l26->reset_gpio);
		cs40l26->reset_gpio = NULL;
		goto err;
	}

	usleep_range(CS40L26_MIN_RESET_PULSE_WIDTH,
			CS40L26_MIN_RESET_PULSE_WIDTH + 100);

	gpiod_set_value_cansleep(cs40l26->reset_gpio, CS40L26_ENABLE);

	usleep_range(CS40L26_CONTROL_PORT_READY_DELAY,
			CS40L26_CONTROL_PORT_READY_DELAY + 100);

	ret = cs40l26_part_num_resolve(cs40l26);
	if (ret)
		goto err;

	ret = devm_request_threaded_irq(dev, cs40l26->irq, NULL, cs40l26_irq,
			IRQF_ONESHOT | IRQF_SHARED | IRQF_TRIGGER_LOW,
			"cs40l26", cs40l26);
	if (ret) {
		dev_err(dev, "Failed to request threaded IRQ\n");
		goto err;
	}
	/* the /ALERT pin may be asserted prior to firmware initialization.
	 * Disable the interrupt handler until firmware has downloaded
	 * so erroneous interrupt requests are ignored
	 */
	disable_irq(cs40l26->irq);

	ret = cs40l26_cl_dsp_init(cs40l26);
	if (ret)
		goto err;

	ret = cs40l26_dsp_pre_config(cs40l26);
	if (ret)
		goto err;

	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			CS40L26_FW_FILE_NAME, dev, GFP_KERNEL, cs40l26,
			cs40l26_firmware_load);

	ret = cs40l26_input_init(cs40l26);
	if (ret)
		goto err;

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, cs40l26_devs,
			CS40L26_NUM_MFD_DEVS, NULL, 0, NULL);
	if (ret) {
		dev_err(dev, "Failed to register codec component\n");
		goto err;
	}

	return 0;

err:
	cs40l26_remove(cs40l26);

	return ret;
}
EXPORT_SYMBOL(cs40l26_probe);

int cs40l26_remove(struct cs40l26_private *cs40l26)
{
	struct regulator *vp_consumer =
			cs40l26_supplies[CS40L26_VP_SUPPLY].consumer;
	struct regulator *va_consumer =
			cs40l26_supplies[CS40L26_VA_SUPPLY].consumer;

	disable_irq(cs40l26->irq);
	mutex_destroy(&cs40l26->lock);


	if (cs40l26->pm_ready)
		cs40l26_pm_runtime_teardown(cs40l26);

	if (cs40l26->vibe_workqueue) {
		destroy_workqueue(cs40l26->vibe_workqueue);
		cancel_work_sync(&cs40l26->vibe_start_work);
		cancel_work_sync(&cs40l26->vibe_stop_work);
		cancel_work_sync(&cs40l26->set_gain_work);
	}

	if (vp_consumer)
		regulator_disable(vp_consumer);

	if (va_consumer)
		regulator_disable(va_consumer);

	gpiod_set_value_cansleep(cs40l26->reset_gpio, CS40L26_DISABLE);

	if (cs40l26->vibe_timer.function)
		hrtimer_cancel(&cs40l26->vibe_timer);

	if (cs40l26->vibe_init_success) {
		sysfs_remove_group(&cs40l26->input->dev.kobj,
				&cs40l26_dev_attr_group);
		sysfs_remove_group(&cs40l26->input->dev.kobj,
				&cs40l26_dev_attr_cal_group);
	}

	if (cs40l26->input)
		input_unregister_device(cs40l26->input);

	return 0;
}
EXPORT_SYMBOL(cs40l26_remove);

int cs40l26_suspend(struct device *dev)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);

	if (!cs40l26->pm_ready) {
		dev_dbg(dev, "Suspend call ignored\n");
		return 0;
	}

	dev_dbg(cs40l26->dev, "%s: Enabling hibernation\n", __func__);

	return cs40l26_pm_state_transition(cs40l26,
			CS40L26_PM_STATE_ALLOW_HIBERNATE);
}
EXPORT_SYMBOL(cs40l26_suspend);

int cs40l26_sys_suspend(struct device *dev)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct i2c_client *i2c_client = to_i2c_client(dev);

	dev_dbg(cs40l26->dev, "System suspend, disabling IRQ\n");

	disable_irq(i2c_client->irq);

	return 0;
}
EXPORT_SYMBOL(cs40l26_sys_suspend);

int cs40l26_sys_suspend_noirq(struct device *dev)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct i2c_client *i2c_client = to_i2c_client(dev);

	dev_dbg(cs40l26->dev, "Late system suspend, re-enabling IRQ\n");
	enable_irq(i2c_client->irq);

	return 0;
}
EXPORT_SYMBOL(cs40l26_sys_suspend_noirq);

int cs40l26_resume(struct device *dev)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);

	if (!cs40l26->pm_ready) {
		dev_dbg(dev, "Resume call ignored\n");
		return 0;
	}

	dev_dbg(cs40l26->dev, "%s: Disabling hibernation\n", __func__);

	return cs40l26_pm_state_transition(cs40l26,
			CS40L26_PM_STATE_PREVENT_HIBERNATE);
}
EXPORT_SYMBOL(cs40l26_resume);

int cs40l26_sys_resume(struct device *dev)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct i2c_client *i2c_client = to_i2c_client(dev);

	dev_dbg(cs40l26->dev, "System resume, re-enabling IRQ\n");

	enable_irq(i2c_client->irq);

	return 0;
}
EXPORT_SYMBOL(cs40l26_sys_resume);

int cs40l26_sys_resume_noirq(struct device *dev)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct i2c_client *i2c_client = to_i2c_client(dev);

	dev_dbg(cs40l26->dev, "Early system resume, disabling IRQ\n");

	disable_irq(i2c_client->irq);

	return 0;
}
EXPORT_SYMBOL(cs40l26_sys_resume_noirq);

MODULE_DESCRIPTION("CS40L26 Boosted Mono Class D Amplifier for Haptics");
MODULE_AUTHOR("Fred Treven, Cirrus Logic Inc. <fred.treven@cirrus.com>");
MODULE_LICENSE("GPL");
