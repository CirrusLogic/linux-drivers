// SPDX-License-Identifier: GPL-2.0
//
// cs40l26.c -- CS40L26 Boosted Haptic Driver with Integrated DSP and
// Waveform Memory with Advanced Closed Loop Algorithms and LRA protection
//
// Copyright 2020 Cirrus Logic, Inc.
//
// Author: Fred Treven <fred.treven@cirrus.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation.

#include <linux/mfd/cs40l26.h>

static struct regulator_bulk_data cs40l26_supplies[CS40L26_NUM_SUPPLIES] = {
	{ .supply = CS40L26_VP_SUPPLY_NAME },
	{ .supply = CS40L26_VA_SUPPLY_NAME },
};

int cs40l26_dsp_read(struct cs40l26_private *cs40l26, u32 reg, u32 *val)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	int ret, i;
	u32 read_val;

	for (i = 0; i < CS40L26_DSP_ACK_TIMEOUT_COUNT; i++) {
		ret = regmap_read(regmap, reg, &read_val);
		if (ret)
			dev_warn(dev, "Failed to read 0x%X, attempt(s) = %d\n",
					reg, i + 1);
		else
			break;

		msleep(CS40L26_TIMEOUT_INTERVAL_MS);
	}

	if (i >= CS40L26_DSP_ACK_TIMEOUT_COUNT) {
		dev_err(dev, "Timed out attempting to read 0x%X\n", reg);
		return -ETIME;
	}

	*val = read_val;

	return 0;
}
EXPORT_SYMBOL(cs40l26_dsp_read);

int cs40l26_dsp_write(struct cs40l26_private *cs40l26, u32 reg, u32 val)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	int ret, i;

	for (i = 0; i < CS40L26_DSP_ACK_TIMEOUT_COUNT; i++) {
		ret = regmap_write(regmap, reg, val);
		if (ret)
			dev_warn(dev,
				"Failed to write to 0x%X, attempt(s) = %d\n",
				reg, i + 1);
		else
			break;

		msleep(CS40L26_TIMEOUT_INTERVAL_MS);
	}

	if (i >= CS40L26_DSP_ACK_TIMEOUT_COUNT) {
		dev_err(dev, "Timed out attempting to write to 0x%X\n", reg);
		return -ETIME;
	}

	return 0;
}
EXPORT_SYMBOL(cs40l26_dsp_write);

int cs40l26_ack_read(struct cs40l26_private *cs40l26, u32 reg, u32 ack_val)
{
	struct device *dev = cs40l26->dev;
	int ret;
	u32 val;

	ret = cs40l26_dsp_read(cs40l26, reg, &val);
	if (ret)
		return ret;

	if (val != ack_val) {
		dev_err(dev, "Read value 0x%X does not match expected 0x%X\n",
				val, ack_val);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(cs40l26_ack_read);

int cs40l26_ack_write(struct cs40l26_private *cs40l26,
		u32 reg, u32 write_val, u32 reset_val)
{
	int ret;

	ret = cs40l26_dsp_write(cs40l26, reg, write_val);
	if (ret)
		return ret;

	return cs40l26_ack_read(cs40l26, reg, reset_val);
}
EXPORT_SYMBOL(cs40l26_ack_write);

static int cs40l26_mbox_buffer_read(struct cs40l26_private *cs40l26, u32 *val)
{
	struct device *dev = cs40l26->dev;
	struct regmap *regmap = cs40l26->regmap;
	u32 base, last, len, write_ptr, read_ptr, mbox_response;
	u32 *buffer;
	int ret;

	buffer = kmalloc_array(CS40L26_DSP_MBOX_BUFFER_NUM_REGS,
			sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return false;

	ret = regmap_bulk_read(regmap, CS40L26_DSP_MBOX_BUFFER_BASE, buffer,
			CS40L26_DSP_MBOX_BUFFER_NUM_REGS);
	if (ret) {
		dev_err(dev, "Failed to read buffer contents\n");
		goto err_free;
	}

	base = buffer[0];
	len = buffer[1];
	write_ptr = buffer[2];
	read_ptr = buffer[3];
	last = base + ((len - 1) * CL_DSP_BYTES_PER_WORD);

	if ((read_ptr - CL_DSP_BYTES_PER_WORD) == write_ptr) {
		dev_err(dev, "Mailbox buffer is full, info missing\n");
		ret = -ENOSPC;
		goto err_free;
	}

	if (read_ptr == write_ptr) {
		dev_warn(dev, "No new message to read\n");
		ret = 0;
		goto err_free;
	}

	ret = regmap_read(regmap, read_ptr, &mbox_response);
	if (ret) {
		dev_err(dev, "Failed to read from mailbox buffer\n");
		goto err_free;
	}

	if (read_ptr == last)
		read_ptr = base;
	else
		read_ptr += CL_DSP_BYTES_PER_WORD;

	ret = regmap_write(regmap, CS40L26_DSP_MBOX_BUFFER_READ_PTR, read_ptr);
	if (ret)
		dev_err(dev, "Failed to update read pointer\n");

err_free:
	kfree(buffer);

	*val = mbox_response;

	return ret;
}

static int cs40l26_handle_mbox_buffer(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	u32 val = 0;
	int ret;

	ret = cs40l26_mbox_buffer_read(cs40l26, &val);
	if (ret)
		return ret;

	if ((val & CS40L26_DSP_MBOX_CMD_INDEX_MASK)
			== CS40L26_DSP_MBOX_PANIC) {
		dev_alert(dev, "DSP PANIC! Error condition: 0x%06X\n",
		(unsigned int) (val & CS40L26_DSP_MBOX_CMD_PAYLOAD_MASK));
		return -ENOTRECOVERABLE;
	}

	switch (val) {
	case CS40L26_DSP_MBOX_TRIGGER_COMPLETE:
		/* this will be needed soon */
		break;
	case CS40L26_DSP_MBOX_PM_AWAKE:
		dev_dbg(dev, "HALO Core is awake\n");
		break;
	case CS40L26_DSP_MBOX_F0_EST_START:
		/* intentionally fall through */
	case CS40L26_DSP_MBOX_F0_EST_DONE:
		/* intentionally fall through */
	case CS40L26_DSP_MBOX_REDC_EST_START:
		/* intentionally fall through */
	case CS40L26_DSP_MBOX_REDC_EST_DONE:
		/* intentionally fall through */
	case CS40L26_DSP_MBOX_SYS_ACK:
		dev_err(dev, "Mbox buffer value (0x%X) not supported\n", val);
		return -EPERM;
	default:
		dev_err(dev, "MBOX buffer value (0x%X) is invalid\n", val);
		return -EINVAL;
	}

	return 0;
}

int cs40l26_pm_state_transition(struct cs40l26_private *cs40l26,
		enum cs40l26_pm_state state)
{
	struct device *dev = cs40l26->dev;
	struct regmap *regmap = cs40l26->regmap;
	int ret;
	u32 val;

	if (cs40l26->pm_state == state)
		return 0;

	if (cs40l26->pm_state == CS40L26_PM_STATE_ALLOW_HIBERNATE
			&& state == CS40L26_PM_STATE_PREVENT_HIBERNATE) {
		ret = regmap_read(regmap, CL_DSP_HALO_XM_FW_ID_REG, &val);
		if (ret) {
			dev_err(dev, "Failed to read firmware ID\n");
			return ret;
		}

		if (val != cs40l26->dsp->fw_desc->id) {
			dev_err(dev, "Firmware ID corrupted upon wake\n");
			return -EINVAL;
		}
	}

	switch (state) {
	case CS40L26_PM_STATE_HIBERNATE:
		dev_err(dev, "Invalid PM state: %u\n", state);
		return -EINVAL;
	case CS40L26_PM_STATE_WAKEUP:
		dev_err(dev, "Invalid PM state: %u\n", state);
		return -EINVAL;
	case CS40L26_PM_STATE_PREVENT_HIBERNATE:
		ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				CS40L26_DSP_MBOX_CMD_PREVENT_HIBER,
				CS40L26_DSP_MBOX_RESET);
		if (ret) {
			dev_err(dev, "Failed to prevent hibernate\n");
			return ret;
		}
		break;
	case CS40L26_PM_STATE_ALLOW_HIBERNATE:
		ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				CS40L26_DSP_MBOX_CMD_ALLOW_HIBER,
				CS40L26_DSP_MBOX_RESET);
		if (ret) {
			dev_err(dev, "Failed to allow hibernate\n");
			return ret;
		}
		break;
	case CS40L26_PM_STATE_SHUTDOWN:
		ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				CS40L26_DSP_MBOX_CMD_SHUTDOWN,
				CS40L26_DSP_MBOX_RESET);
		if (ret) {
			dev_err(dev, "Failed to shut down HALO core\n");
			return ret;
		}
		break;
	default:
		dev_err(dev, "Unknown PM state: %u\n", state);
		return -EINVAL;
	}

	cs40l26->pm_state = state;

	return 0;
}
EXPORT_SYMBOL(cs40l26_pm_state_transition);

static int cs40l26_error_release(struct cs40l26_private *cs40l26,
		unsigned int err_rls)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	int ret;
	unsigned int err_sts, err_cfg;

	ret = regmap_read(regmap, CS40L26_ERROR_RELEASE, &err_sts);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to get error status\n");
		return ret;
	}

	/* To clear an error that causes the device to enter
	 * Actuator Safe Mode, the protection release sequence (0->1->0)
	 * must be applied to the respective error bit.
	 * If the condition that causes automatic protection becomes true
	 * again during the release sequence, the proteciton is not removed,
	 * and a new interrupt is generated.
	 */

	err_cfg = err_sts | (1 << err_rls);

	ret = regmap_write(regmap, CS40L26_ERROR_RELEASE, err_cfg);
	if (ret) {
		dev_err(dev, "Actuator Safe Mode release sequence failed\n");
		return ret;
	}

	err_cfg &= ~(1 << err_rls);

	ret = regmap_write(cs40l26->regmap, CS40L26_ERROR_RELEASE, err_cfg);
	if (ret) {
		dev_err(dev, "Actuator Safe Mode release sequence failed\n");
		return ret;
	}

	err_cfg |= (1 << err_rls);

	ret = regmap_write(cs40l26->regmap, CS40L26_ERROR_RELEASE, err_cfg);
	if (ret)
		dev_err(dev, "Actuator Safe Mode release sequence failed\n");

	return ret;
}

int cs40l26_iseq_update(struct cs40l26_private *cs40l26,
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
EXPORT_SYMBOL(cs40l26_iseq_update);

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

static int cs40l26_handle_irq1(struct cs40l26_private *cs40l26,
		enum cs40l26_irq1 irq1)
{
	struct device *dev = cs40l26->dev;
	unsigned int err_rls = 0;
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
		dev_info(dev, "GPIO%u %s event detected\n", (irq1 / 2) + 1,
				(irq1 % 2) ? "fall" : "rise");
		break;
	case CS40L26_IRQ1_WKSRC_STS_ANY:
		dev_info(dev, "Wakesource detected (ANY)\n");
		ret = cs40l26_iseq_populate(cs40l26);
		if (ret)
			return ret;

		ret = cs40l26_pm_state_transition(cs40l26,
				CS40L26_PM_STATE_PREVENT_HIBERNATE);
		if (ret)
			return ret;
		break;
	case CS40L26_IRQ1_WKSRC_STS_GPIO1:
	/* intentionally fall through */
	case CS40L26_IRQ1_WKSRC_STS_GPIO2:
	/* intentionally fall through */
	case CS40L26_IRQ1_WKSRC_STS_GPIO3:
	/* intentionally fall through */
	case CS40L26_IRQ1_WKSRC_STS_GPIO4:
		dev_info(dev, "GPIO%u event woke device from hibernate\n",
				irq1 - CS40L26_IRQ1_WKSRC_STS_GPIO1 + 1);
		break;
	case CS40L26_IRQ1_WKSRC_STS_SPI:
		dev_info(dev, "SPI event woke device from hibernate\n");
		break;
	case CS40L26_IRQ1_WKSRC_STS_I2C:
		dev_info(dev, "I2C event woke device from hibernate\n");
		break;
	case CS40L26_IRQ1_GLOBAL_EN_ASSERT:
		dev_info(dev, "Started power up seq. (GLOBAL_EN asserted)\n");
		break;
	case CS40L26_IRQ1_PDN_DONE:
		dev_info(dev,
			"Completed power down seq. (GLOBAL_EN cleared\n");
		break;
	case CS40L26_IRQ1_PUP_DONE:
		dev_info(dev,
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
		dev_dbg(dev, "Virtual 2 MBOX write occurred\n");

		ret = cs40l26_handle_mbox_buffer(cs40l26);
		if (ret)
			return ret;
		break;
	default:
		dev_err(dev, "Unrecognized IRQ1 EINT1 status\n");
		return -EINVAL;
	}

	if (err_rls) {
	/* boost related errors handled with global device enable turned off */
		if (bst_err) {
			ret = regmap_write(cs40l26->regmap,
					CS40L26_GLOBAL_ENABLES, CS40L26_DISABLE
					& CS40L26_GLOBAL_EN_MASK);
			if (ret) {
				dev_err(dev, "Failed to clear GLOBAL EN\n");
				return ret;
			}
		}

		ret = cs40l26_error_release(cs40l26, err_rls);
		if (ret)
			return ret;

		if (bst_err) {
			ret = regmap_write(cs40l26->regmap,
					CS40L26_GLOBAL_ENABLES, CS40L26_ENABLE);
			if (ret) {
				dev_err(dev, "Failed to set GLOBAL EN\n");
				return ret;
			}
		}
	}

	/* write 1 to clear the interrupt flag */
	ret = regmap_write(cs40l26->regmap, CS40L26_IRQ1_EINT_1, BIT(irq1));
	if (ret)
		dev_err(dev, "Failed to clear IRQ1 EINT1 %u\n", irq1);

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
		dev_info(dev, "PLL achieved lock\n");
		break;
	case CS40L26_IRQ2_PLL_PHASE_LOCK:
		dev_info(dev, "PLL achieved phase lock\n");
		break;
	case CS40L26_IRQ2_PLL_FREQ_LOCK:
		dev_info(dev, "PLL achieved frequency lock\n");
		break;
	case CS40L26_IRQ2_PLL_UNLOCK_RISE:
		dev_err(dev, "PLL has lost lock\n");
		break;
	case CS40L26_IRQ2_PLL_UNLOCK_FALL:
		dev_warn(dev, "PLL has regained lock\n");
		break;
	case CS40L26_IRQ2_PLL_READY:
		dev_info(dev, "PLL ready for use\n");
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
	struct device *dev = cs40l26->dev;
	struct regmap *regmap = cs40l26->regmap;
	unsigned int val, eint, mask, i;
	int ret;

	/* If waking from hibernate, this may take mutliple reads */
	for (i = 0; i < CS40L26_IRQ_TIMEOUT_COUNT; i++) {
		ret = regmap_read(regmap, CS40L26_IRQ1_STATUS, &val);
		if (ret) {
			usleep_range(CS40L26_IRQ_TIMEOUT_INTERVAL_US,
					CS40L26_IRQ_TIMEOUT_INTERVAL_US + 100);
			continue;
		}
		break;
	}

	if (i == CS40L26_IRQ_TIMEOUT_COUNT) {
		dev_err(dev, "Failed to get IRQ status\n");
		return IRQ_NONE;
	}

	if (!(val & CS40L26_IRQ_STATUS_ASSERT)) {
		dev_warn(dev, "IRQ1 asserted with no pending interrupts\n");
		return IRQ_NONE;
	}

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
		for (i = 0; i < CS40L26_IRQ1_NUM_IRQS; i++) {
			if (val & BIT(i)) {
				ret = cs40l26_handle_irq1(cs40l26, i);
				if (ret)
					goto err;
			}
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
		for (i = 0; i < CS40L26_IRQ2_NUM_IRQS; i++) {
			if (val & BIT(i)) {
				ret = cs40l26_handle_irq2(cs40l26, i);
				if (ret)
					goto err;
			}
		}
	}

err:
	/* if an error has occurred, the IRQ has not been successfully
	 * processed; however, IRQ_HANDLED is still returned since
	 * the interrupt was generated by the CS40L26 device
	 */
	if (ret)
		dev_err(dev, "Failed to process IRQ (%d): %u\n", irq, ret);

	return IRQ_HANDLED;
}

static void cs40l26_vibe_start_worker(struct work_struct *work)
{
	struct cs40l26_private *cs40l26 = container_of(work,
			struct cs40l26_private, vibe_start_work);
	struct device *dev = cs40l26->dev;
	u16 duration = cs40l26->effect->replay.length;
	int ret = 0;
	unsigned int reg, freq;
	u32 index;

	mutex_lock(&cs40l26->lock);

	hrtimer_start(&cs40l26->vibe_timer,
			ktime_set(CS40L26_MS_TO_SECS(duration),
			CS40L26_MS_TO_NS(duration % 1000)), HRTIMER_MODE_REL);

	cs40l26->vibe_state = CS40L26_VIBE_STATE_HAPTIC;

	switch (cs40l26->effect->u.periodic.waveform) {
	case FF_CUSTOM:
		index = cs40l26->trigger_indeces[cs40l26->effect->id];

		ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				index, CS40L26_DSP_MBOX_RESET);
		if (ret)
			goto err_mutex;
		break;
	case FF_SINE:
		ret = cl_dsp_get_reg(cs40l26->dsp, "BUZZ_EFFECTS1_BUZZ_FREQ",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_BUZZGEN_ALGO_ID,
			&reg);
		if (ret) {
			dev_err(dev, "Failed to find BUZZGEN control\n");
			goto err_mutex;
		}

		freq = CS40L26_MS_TO_HZ(cs40l26->effect->u.periodic.period);

		ret = regmap_write(cs40l26->regmap, reg, freq);
		if (ret) {
			dev_err(dev, "Failed to write BUZZGEN frequency\n");
			goto err_mutex;
		}

		ret = regmap_write(cs40l26->regmap,
				reg + CS40L26_BUZZGEN_DURATION_OFFSET,
				duration);
		if (ret) {
			dev_err(dev, "Failed to write BUZZGEN duration\n");
			goto err_mutex;
		}

		ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
					CS40L26_BUZZGEN_INDEX_START,
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
		cs40l26->vibe_state = CS40L26_VIBE_STATE_STOPPED;

	mutex_unlock(&cs40l26->lock);
}

static void cs40l26_vibe_stop_worker(struct work_struct *work)
{
	struct cs40l26_private *cs40l26 = container_of(work,
			struct cs40l26_private, vibe_stop_work);
	int ret;

	mutex_lock(&cs40l26->lock);

	if (cs40l26->vibe_state == CS40L26_VIBE_STATE_STOPPED)
		goto err_mutex;

	ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
		CS40L26_STOP_PLAYBACK, CS40L26_DSP_MBOX_RESET);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to stop playback\n");
		goto err_mutex;
	}

err_mutex:
	if (cs40l26->vibe_state != CS40L26_VIBE_STATE_STOPPED)
		cs40l26->vibe_state = CS40L26_VIBE_STATE_STOPPED;

	mutex_unlock(&cs40l26->lock);
}

static enum hrtimer_restart cs40l26_vibe_timer(struct hrtimer *timer)
{
	struct cs40l26_private *cs40l26 =
		container_of(timer, struct cs40l26_private, vibe_timer);

	queue_work(cs40l26->vibe_workqueue, &cs40l26->vibe_stop_work);

	return HRTIMER_NORESTART;
}

static int cs40l26_playback_effect(struct input_dev *dev,
		int effect_id, int val)
{
	struct cs40l26_private *cs40l26 = input_get_drvdata(dev);
	struct ff_effect *effect;

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

static int cs40l26_upload_effect(struct input_dev *dev,
		struct ff_effect *effect, struct ff_effect *old)
{
	struct cs40l26_private *cs40l26 = input_get_drvdata(dev);
	struct device *cdev = cs40l26->dev;
	s16 *raw_custom_data = NULL;
	int ret = 0;
	u16 index, bank;
	u32 bank_offset, trigger_val;

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
				kzalloc(sizeof(s16) * CS40L26_CUSTOM_DATA_SIZE,
				GFP_KERNEL);
		if (!raw_custom_data)
			return -ENOMEM;

		if (copy_from_user(raw_custom_data,
				effect->u.periodic.custom_data,
				sizeof(s16) * effect->u.periodic.custom_len)) {
			dev_err(cdev, "Failed to get user data\n");
			ret = -EFAULT;
			goto err_free;
		}

		bank = ((u16) raw_custom_data[0]);
		switch (bank) {
		case CS40L26_ROM_BANK0_ID:
			bank_offset = CS40L26_ROM_BANK0_START;
			break;
		case CS40L26_ROM_BANK1_ID:
			bank_offset = CS40L26_ROM_BANK1_START;
			break;
		case CS40L26_ROM_BANK2_ID:
			bank_offset = CS40L26_ROM_BANK2_START;
			break;
		case CS40L26_ROM_BANK3_ID:
			bank_offset = CS40L26_ROM_BANK3_START;
			break;
		case CS40L26_RAM_BANK_ID:
			bank_offset = CS40L26_RAM_INDEX_START;
			break;
		default:
			dev_err(cdev, "Bank ID (%u) out of bounds\n", bank);
			ret = -EINVAL;
			goto err_free;
		}

		index = ((u16) raw_custom_data[1]) & CS40L26_MAX_INDEX_MASK;
		trigger_val = index + bank_offset;
		if ((trigger_val >= CS40L26_RAM_INDEX_START
				&& trigger_val <= CS40L26_RAM_INDEX_END)
				|| (trigger_val >= CS40L26_ROM_INDEX_START
				&& trigger_val <= CS40L26_ROM_INDEX_END)) {
			cs40l26->trigger_indeces[effect->id] = trigger_val;
		} else {
			dev_err(cdev, "Trigger value (0x%X) out of bounds\n",
					trigger_val);
			ret = -EINVAL;
			goto err_free;
		}

		break;
	case FF_SINE:
		if (effect->u.periodic.period) {
			if (effect->u.periodic.period
					< CS40L26_BUZZGEN_PERIOD_MIN
					|| effect->u.periodic.period
					> CS40L26_BUZZGEN_PERIOD_MAX) {
				dev_err(cdev,
				"%u ms duration not within range (4-10 ms)\n",
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

err_free:
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

	ret = input_register_device(cs40l26->input);
	if (ret) {
		dev_err(dev, "Cannot register input device: %d\n", ret);
		return ret;
	}

	hrtimer_init(&cs40l26->vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cs40l26->vibe_timer.function = cs40l26_vibe_timer;

	ret = sysfs_create_group(&cs40l26->input->dev.kobj,
			&cs40l26_debug_dev_attr_group);
	if (ret) {
		dev_err(dev, "Failed to create sysfs group: %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&cs40l26->input->dev.kobj,
			&cs40l26_dev_attr_group);
	if (ret)
		dev_err(dev, "Failed to create sysfs group: %d\n", ret);
	else
		cs40l26->vibe_init_success = true;

	return ret;
}

static int cs40l26_part_num_resolve(struct cs40l26_private *cs40l26)
{
	int ret;
	unsigned int val;
	struct device *dev = cs40l26->dev;
	struct regmap *regmap = cs40l26->regmap;

	ret = regmap_read(regmap, CS40L26_DEVID, &val);
	if (ret) {
		dev_err(dev, "Failed to read device ID\n");
		return ret;
	}
	cs40l26->devid = val;

	ret = regmap_read(regmap, CS40L26_REVID, &val);
	if (ret) {
		dev_err(dev, "Failed to read revision ID\n");
		return ret;
	}
	cs40l26->revid = val;

	dev_info(dev, "Cirrus Logic %s ID: 0x%06X, Revision: 0x%02X\n",
			CS40L26_DEV_NAME, cs40l26->devid, cs40l26->revid);

	return 0;
}

static int cs40l26_cl_dsp_init(struct cs40l26_private *cs40l26)
{
	cs40l26->dsp = cl_dsp_create(cs40l26->dev, cs40l26->regmap);
	if (!cs40l26->dsp)
		return -ENOMEM;

	if (cs40l26->fw_mode == CS40L26_FW_MODE_ROM)
		cs40l26->dsp->fw_desc = &cs40l26_fw;
	else
		cs40l26->dsp->fw_desc = &cs40l26_ram_fw;

	return 0;
}

static bool cs40l26_pseq_addr_exists(struct cs40l26_private *cs40l26, u16 addr,
		int *index)
{
	int i;

	if (cs40l26->pseq_len == 0)
		return false;

	for (i = 0; i < cs40l26->pseq_len; i++) {
		if (cs40l26->pseq_table[i].addr == addr) {
			*index = i;
			return true;
		}
	}

	*index = -1;

	return false;
}

static int cs40l26_pseq_write(struct cs40l26_private *cs40l26,
		unsigned int pseq_offset)
{
	struct regmap *regmap = cs40l26->regmap;
	unsigned int len = cs40l26->pseq_len;
	struct device *dev = cs40l26->dev;
	u32 val;
	u16 addr;
	int ret;

	addr = cs40l26->pseq_table[pseq_offset].addr;
	val = cs40l26->pseq_table[pseq_offset].val;

	/* the "upper half" first 24-bit word of the sequence pair is written
	 * to the write sequencer as: [23-16] addr{15-0},
	 * [15-0] val{31-24} with bits [24-31] acting as a buffer
	 */
	ret = regmap_write(regmap, cs40l26->pseq_base +
			(pseq_offset * CS40L26_PSEQ_STRIDE),
			(addr << CS40L26_PSEQ_ADDR_SHIFT) |
			((val & ~CS40L26_PSEQ_VAL_MASK)
			>> CS40L26_PSEQ_VAL_SHIFT));
	if (ret) {
		dev_err(dev, "Failed to write power on seq. (upper half)\n");
		return ret;
	}

	/* the "lower half" of the address-value pair is written to the write
	 * sequencer as: [23-0] data{23-0} with bits [24-31] acting as a buffer
	 */
	ret = regmap_write(regmap, cs40l26->pseq_base + CL_DSP_BYTES_PER_WORD
			+ (pseq_offset * CS40L26_PSEQ_STRIDE),
			val & CS40L26_PSEQ_VAL_MASK);
	if (ret) {
		dev_err(dev, "Failed to write power on seq. (lower half)\n");
		return ret;
	}

	/* end of sequence must be marked by list terminator */
	ret = regmap_write(regmap, cs40l26->pseq_base +
			(len * CS40L26_PSEQ_STRIDE), CS40L26_PSEQ_LIST_TERM);
	if (ret)
		dev_err(dev, "Failed to write power on seq. terminator\n");

	return ret;
}

static int cs40l26_pseq_add_pair(struct cs40l26_private *cs40l26, u16 addr,
		u32 val)
{
	unsigned int len = cs40l26->pseq_len;
	struct device *dev = cs40l26->dev;
	unsigned int pseq_offset, prev_val;
	int ret, index;

	if (len >= CS40L26_PSEQ_MAX_ENTRIES) {
		dev_err(dev, "Power on seq. exceeded max number of entries\n");
		return -E2BIG;
	}

	if (cs40l26_pseq_addr_exists(cs40l26, addr, &index)) {
		prev_val = cs40l26->pseq_table[index].val;
		cs40l26->pseq_table[index].val = val;
		pseq_offset = index;
	} else {
		cs40l26->pseq_table[len].addr = addr;
		cs40l26->pseq_table[len].val = val;
		cs40l26->pseq_len++;
		pseq_offset = len;
	}

	ret = cs40l26_pseq_write(cs40l26, pseq_offset);
	if (ret) { /* If an error occurs during write, reset the sequence */
		if (index < 0) { /* No previous value for this address */
			cs40l26->pseq_table[len].addr = 0;
			cs40l26->pseq_table[len].val = 0;
			cs40l26->pseq_len--;
		} else {
			cs40l26->pseq_table[index].val = prev_val;
		}
	}

	return ret;
}

static int cs40l26_pseq_multi_add_pair(struct cs40l26_private *cs40l26,
		const struct reg_sequence *reg_seq, int num_regs)
{
	int ret, i;

	for (i = 0; i < num_regs; i++) {
		ret = cs40l26_pseq_add_pair(cs40l26, (u16) (reg_seq[i].reg
				& CS40L26_PSEQ_ADDR_MASK), reg_seq[i].def);
		if (ret)
			return ret;
	}

	return 0;
}

static int cs40l26_pseq_init(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	int ret, i, index = 0;
	u8 upper_val = 0;
	u16 addr = 0;
	u32 val, word;

	ret = cl_dsp_get_reg(cs40l26->dsp, "POWER_ON_SEQUENCE",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_PM_ALGO_ID,
			&cs40l26->pseq_base);
	if (ret)
		return ret;

	for (i = 0; i < CS40L26_PSEQ_MAX_ENTRIES; i++) {
		ret = regmap_read(cs40l26->regmap,
				cs40l26->pseq_base +
				(i * CL_DSP_BYTES_PER_WORD), &word);
		if (ret) {
			dev_err(dev, "Failed to read from power on seq.\n");
			return ret;
		}

		if ((word & CS40L26_PSEQ_LIST_TERM_MASK) ==
				CS40L26_PSEQ_LIST_TERM)
			break;

		if (i % CS40L26_PSEQ_PAIR_NUM_WORDS) { /* lower half */
			index = i / CS40L26_PSEQ_PAIR_NUM_WORDS;
			val = (upper_val << CS40L26_PSEQ_VAL_SHIFT) |
					(word & CS40L26_PSEQ_VAL_MASK);

			cs40l26->pseq_table[index].addr = addr;
			cs40l26->pseq_table[index].val = val;
		} else { /* upper half */
			addr = (word & CS40L26_PSEQ_ADDR_WORD_MASK) >>
					CS40L26_PSEQ_ADDR_SHIFT;

			upper_val = word & CS40L26_PSEQ_VAL_WORD_UPPER_MASK;
		}
	}

	if (i == CS40L26_PSEQ_MAX_ENTRIES) {
		dev_err(dev, "Original sequence exceeds max # of entries\n");
		return -E2BIG;
	}

	ret = regmap_write(cs40l26->regmap, cs40l26->pseq_base +
			(i * CL_DSP_BYTES_PER_WORD), CS40L26_PSEQ_LIST_TERM);
	if (ret) {
		dev_err(dev, "Failed to write power on seq. list terminator\n");
		return ret;
	}

	cs40l26->pseq_len = index + 1;
	return 0;
}

static int cs40l26_gpio_config(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	struct regmap *regmap = cs40l26->regmap;
	u32 otp_gpio_cfg;
	int ret;

	ret = regmap_read(regmap, CS40L26_OTP_MEM(28), &otp_gpio_cfg);
	if (ret) {
		dev_err(dev, "Failed to get GPIO config from OTP\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS40L26_IRQ1_MASK_1,
			BIT(CS40L26_IRQ1_GPIO1_RISE) |
			BIT(CS40L26_IRQ1_GPIO1_FALL), 0x0);
	if (ret) {
		dev_err(dev, "Failed to update IRQ mask for GPIO1\n");
		return ret;
	}

	if (otp_gpio_cfg & CS40L26_OTP_GPI_MASK) { /* 4 GPIO config */
		ret = regmap_multi_reg_write(regmap, cs40l26_gpio_setup_gpi,
				CS40L26_NUM_GPIO_SETUP_WRITES);
		if (ret) {
			dev_err(dev, "Failed to configure GPIOs as 4 GPIs\n");
			return ret;
		}

		ret = cs40l26_pseq_multi_add_pair(cs40l26,
				cs40l26_gpio_setup_gpi,
				CS40L26_NUM_GPIO_SETUP_WRITES);
		if (ret) {
			dev_err(dev, "Failed to sequence 4 GPI config\n");
			return ret;
		}

		ret = regmap_update_bits(regmap, CS40L26_IRQ1_MASK_1,
				GENMASK(CS40L26_IRQ1_GPIO4_FALL,
				CS40L26_IRQ1_GPIO2_RISE),
				0x00 << CS40L26_IRQ1_GPIO2_RISE);
		if (ret) {
			dev_err(dev, "Failed to update multi GPIO IRQs\n");
			return ret;
		}
	} else { /* ASP config */
		ret = regmap_multi_reg_write(regmap, cs40l26_gpio_setup_asp,
				CS40L26_NUM_GPIO_SETUP_WRITES);
		if (ret) {
			dev_err(dev, "Failed to configure GPIOs for ASP\n");
			return ret;
		}

		ret = cs40l26_pseq_multi_add_pair(cs40l26,
				cs40l26_gpio_setup_asp,
				CS40L26_NUM_GPIO_SETUP_WRITES);
		if (ret) {
			dev_err(dev, "Failed to sequence ASP config\n");
			return ret;
		}
	}

	return cs40l26_iseq_update(cs40l26, CS40L26_ISEQ_MASK1);
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

	ret = cs40l26_pseq_add_pair(cs40l26, (u16) (CS40L26_BLOCK_ENABLES2
			& CS40L26_PSEQ_ADDR_MASK), val);
	if (ret) {
		dev_err(dev, "Failed to sequence brownout prevention\n");
		return ret;
	}

	if (cs40l26->pdata.vbbr_en) {
		ret = regmap_update_bits(regmap, CS40L26_IRQ1_MASK_2,
				CS40L26_VBBR_ATT_CLR_MASK |
				CS40L26_VBBR_FLAG_MASK,
				(0 << CS40L26_VBBR_ATT_CLR_SHIFT) |
				(0 << CS40L26_VBBR_FLAG_SHIFT));
		if (ret) {
			dev_err(dev, "Failed to update VBBR IRQ mask\n");
			return ret;
		}

		ret = cs40l26_iseq_update(cs40l26, CS40L26_ISEQ_MASK2);
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

		ret = cs40l26_pseq_add_pair(cs40l26, (u16) (CS40L26_VBBR_CONFIG
				& CS40L26_PSEQ_ADDR_MASK), val);
		if (ret)
			return ret;
	}

	ret = regmap_read(regmap, CS40L26_VPBR_CONFIG, &val);
	if (ret) {
		dev_err(dev, "Failed to get VBBR config.\n");
		return ret;
	}

	if (cs40l26->pdata.vpbr_en) {
		ret = regmap_update_bits(regmap, CS40L26_IRQ1_MASK_2,
				CS40L26_VPBR_ATT_CLR_MASK |
				CS40L26_VPBR_FLAG_MASK,
				(0 << CS40L26_VPBR_ATT_CLR_SHIFT) |
				(0 << CS40L26_VPBR_FLAG_SHIFT));
		if (ret) {
			dev_err(dev, "Failed to update VPBR IRQ mask\n");
			return ret;
		}

		ret = cs40l26_iseq_update(cs40l26, CS40L26_ISEQ_MASK2);
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

		ret = cs40l26_pseq_add_pair(cs40l26, (u16) (CS40L26_VPBR_CONFIG
				& CS40L26_PSEQ_ADDR_MASK), val);
		if (ret)
			return ret;
	}

	return 0;
}

static int cs40l26_dsp_config(struct cs40l26_private *cs40l26)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	int ret;
	u32 reg;

	ret = cs40l26_pseq_init(cs40l26);
	if (ret)
		return ret;

	ret = regmap_multi_reg_write(regmap, cs40l26_output_default,
			CS40L26_NUM_OUTPUT_SETUP_WRITES);
	if (ret) {
		dev_err(dev, "Failed to configure output registers\n");
		return ret;
	}

	ret = cs40l26_pseq_multi_add_pair(cs40l26, cs40l26_output_default,
			CS40L26_NUM_OUTPUT_SETUP_WRITES);
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, CS40L26_IRQ1_MASK_1,
			BIT(CS40L26_IRQ1_VIRTUAL2_MBOX_WR),
			0 << CS40L26_IRQ1_VIRTUAL2_MBOX_WR);
	if (ret) {
		dev_err(dev, "Failed to unmask mailbox interrupt\n");
		return ret;
	}

	ret = cs40l26_iseq_init(cs40l26);
	if (ret)
		return ret;

	ret = cs40l26_gpio_config(cs40l26);
	if (ret)
		return ret;

	ret = cs40l26_brownout_prevention_init(cs40l26);
	if (ret)
		return ret;

	if (cs40l26->fw_mode == CS40L26_FW_MODE_RAM) {
		ret = cl_dsp_get_reg(cs40l26->dsp, "CALL_RAM_INIT",
				CL_DSP_XM_UNPACKED_TYPE,
				cs40l26->dsp->fw_desc->id, &reg);
		if (ret)
			return ret;

		ret = cs40l26_dsp_write(cs40l26, reg, CS40L26_ENABLE);
		if (ret)
			return ret;
	}

	/* ensure firmware running */
	ret = cl_dsp_get_reg(cs40l26->dsp, "HALO_STATE",
			CL_DSP_XM_UNPACKED_TYPE, cs40l26->dsp->fw_desc->id,
			&reg);
	if (ret)
		return ret;

	ret = cs40l26_ack_read(cs40l26, reg,
			cs40l26->dsp->fw_desc->halo_state_run);
	if (ret)
		return ret;

	/* OTP configures device to allow hibernate on boot */
	cs40l26->pm_state = CS40L26_PM_STATE_ALLOW_HIBERNATE;

	ret = cs40l26_pm_state_transition(cs40l26,
			CS40L26_PM_STATE_PREVENT_HIBERNATE);
	if (ret)
		return ret;

	enable_irq(cs40l26->irq);
	return 0;
}

static void cs40l26_firmware_load(const struct firmware *fw, void *context)
{
	struct cs40l26_private *cs40l26 = (struct cs40l26_private *)context;
	struct device *dev = cs40l26->dev;
	int ret;

	if (!fw) {
		dev_err(dev, "Failed to request firmware file\n");
		return;
	}

	ret = cl_dsp_firmware_parse(cs40l26->dsp, fw);
	release_firmware(fw);
	if (ret)
		return;

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

	return 0;
}

int cs40l26_probe(struct cs40l26_private *cs40l26,
		struct cs40l26_platform_data *pdata)
{
	struct device *dev = cs40l26->dev;
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

	ret = devm_regulator_bulk_get(dev, CS40L26_NUM_SUPPLIES,
			cs40l26_supplies);
	if (ret) {
		dev_err(dev, "Failed to request core supplies: %d\n", ret);
		goto err;
	}

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

	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			cs40l26->dsp->fw_desc->fw_file, dev, GFP_KERNEL,
			cs40l26, cs40l26_firmware_load);

	ret = cs40l26_input_init(cs40l26);
	if (ret)
		goto err;

	return 0;

err:
	regulator_bulk_disable(CS40L26_NUM_SUPPLIES, cs40l26_supplies);

	gpiod_set_value_cansleep(cs40l26->reset_gpio, CS40L26_DISABLE);

	return ret;
}
EXPORT_SYMBOL(cs40l26_probe);

int cs40l26_remove(struct cs40l26_private *cs40l26)
{
	mutex_destroy(&cs40l26->lock);

	if (cs40l26->vibe_workqueue) {
		destroy_workqueue(cs40l26->vibe_workqueue);
		cancel_work_sync(&cs40l26->vibe_start_work);
		cancel_work_sync(&cs40l26->vibe_stop_work);
	}

	regulator_bulk_disable(CS40L26_NUM_SUPPLIES, cs40l26_supplies);

	gpiod_set_value_cansleep(cs40l26->reset_gpio, CS40L26_DISABLE);

	if (cs40l26->vibe_timer.function)
		hrtimer_cancel(&cs40l26->vibe_timer);

	if (cs40l26->vibe_init_success) {
		sysfs_remove_group(&cs40l26->input->dev.kobj,
				&cs40l26_dev_attr_group);
		sysfs_remove_group(&cs40l26->input->dev.kobj,
				&cs40l26_debug_dev_attr_group);
	}

	if (cs40l26->input)
		input_unregister_device(cs40l26->input);

	return 0;
}
EXPORT_SYMBOL(cs40l26_remove);

MODULE_DESCRIPTION("CS40L26 Boosted Mono Class D Amplifier for Haptics");
MODULE_AUTHOR("Fred Treven, Cirrus Logic Inc. <fred.treven@cirrus.com>");
MODULE_LICENSE("GPL");
