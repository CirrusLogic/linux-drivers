// SPDX-License-Identifier: GPL-2.0
//
// ln8411_charger.c - LN8411 power supply charger driver
//
// Copyright 2022 Cirrus Logic, Inc.
//
// Author: Ricardo Rivera-Matos <rriveram@opensource.cirrus.com>

#include <linux/errno.h>
#include <linux/extcon-provider.h>
#include <linux/gpio/consumer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include "ln8411_charger.h"

static int ln8411_set_lion_ctrl(struct ln8411_device *ln8411, enum ln8411_keys key)
{
	int ret;

	ret = regmap_write(ln8411->regmap, LN8411_LION_CTRL, (unsigned int)key);
	if (ret)
		dev_err(ln8411->dev, "Failed to set LION_CTRL: %d\n", ret);

	return ret;
}

static int ln8411_a1_2to1_workaround(struct ln8411_device *ln8411, const bool enable)
{
	unsigned int val;
	int ret;

	if (enable)
		val = LN8411_SWAP_EN_0;
	else
		val = 0;

	ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_TEST_MODE);
	if (ret)
		return ret;

	ret = regmap_update_bits(ln8411->regmap, LN8411_SWAP_CTRL_3, LN8411_SWAP_EN_0, val);
	if (ret)
		return ret;

	return ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_LOCK);
}

static int __ln8411_get_adc__(struct ln8411_device *ln8411,
			      const enum ln8411_adc_chan chan, u16 *val)
{
	unsigned int lsb, msb, reg;
	int ret;

	ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_TEST_MODE);
	if (ret)
		return ret;

	ret = regmap_set_bits(ln8411->regmap, LN8411_ADC_CFG_2, LN8411_PAUSE_ADC_UPDATES);
	if (ret)
		return ret;

	reg = (2 * chan) + LN8411_IBUS_ADC1;

	ret = regmap_read(ln8411->regmap, reg, &msb);
	if (ret)
		return ret;

	reg++;

	ret = regmap_read(ln8411->regmap, reg, &lsb);
	if (ret)
		return ret;

	*val = (msb << LN8411_REG_BITS) | lsb;

	ret = regmap_clear_bits(ln8411->regmap, LN8411_ADC_CFG_2, LN8411_PAUSE_ADC_UPDATES);
	if (ret)
		return ret;

	return ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_LOCK);
}

static int ln8411_get_adc(struct ln8411_device *ln8411,
			  const enum ln8411_adc_chan chan, int *intval)
{
	int ret;
	u16 val;

	ret = __ln8411_get_adc__(ln8411, chan, &val);
	if (ret)
		return ret;

	switch (chan) {
	case LN8411_ADC_CHAN_IBUS:
		*intval = val * LN8411_IBUS_ADC_STEP_UA;
		break;
	case LN8411_ADC_CHAN_VBUS:
		*intval = val * LN8411_VBUS_ADC_STEP_UV;
		break;
	case LN8411_ADC_CHAN_VUSB:
		*intval = val * LN8411_VUSB_ADC_STEP_UV;
		break;
	case LN8411_ADC_CHAN_VWPC:
		*intval = val * LN8411_VWPC_ADC_STEP_UV;
		break;
	case LN8411_ADC_CHAN_VOUT:
		*intval = val * LN8411_VOUT_ADC_STEP_UV;
		break;
	case LN8411_ADC_CHAN_VBAT:
		*intval = val * LN8411_VBAT_ADC_STEP_UV;
		break;
	case LN8411_ADC_CHAN_IBAT:
		*intval = val * LN8411_IBAT_ADC_STEP_UA;
		break;
	case LN8411_ADC_CHAN_TSBAT:
		*intval = val;
		break;
	case LN8411_ADC_CHAN_TDIE:
		*intval = val * LN8411_TDIE_STEP_DECIC;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int ln8411_get_wpc_health(struct ln8411_device *ln8411, union power_supply_propval *val)
{
	unsigned int reg_code;
	int ret;

	val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;

	ret = regmap_read(ln8411->regmap, LN8411_FAULT2_STS, &reg_code);
	if (ret)
		return ret;

	if (reg_code & LN8411_VWPC_OV_STS) {
		val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		dev_dbg(ln8411->dev, "VWPC overvoltage condition detected!\n");
		return ret;
	}

	val->intval = POWER_SUPPLY_HEALTH_GOOD;

	return ret;
}

static int ln8411_get_usb_health(struct ln8411_device *ln8411, union power_supply_propval *val)
{
	unsigned int reg_code;
	int ret;

	val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;

	ret = regmap_read(ln8411->regmap, LN8411_FAULT2_STS, &reg_code);
	if (ret)
		return ret;

	if (reg_code & LN8411_VUSB_OV_STS) {
		val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		dev_dbg(ln8411->dev, "VUSB overvoltage condition detected!\n");
		return ret;
	}

	val->intval = POWER_SUPPLY_HEALTH_GOOD;

	return ret;
}

static int ln8411_get_batt_health(struct ln8411_device *ln8411, union power_supply_propval *val)
{
	unsigned int reg_code;
	int ret;

	val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;

	ret = regmap_read(ln8411->regmap, LN8411_FAULT3_STS, &reg_code);
	if (ret)
		return ret;

	if (reg_code & LN8411_IBAT_OC_DETECTED) {
		val->intval = POWER_SUPPLY_HEALTH_OVERCURRENT;
		dev_dbg(ln8411->dev, "IBAT overcurrent detected!\n");
		return ret;
	}

	if (reg_code & LN8411_VBAT_OV_STS) {
		val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		dev_dbg(ln8411->dev, "VBAT overvoltage detected!\n");
		return ret;
	}

	ret = regmap_read(ln8411->regmap, LN8411_SAFETY_STS, &reg_code);
	if (ret)
		return ret;

	if (reg_code & (LN8411_TSBAT_ALARM_STS | LN8411_TSBAT_SHUTDOWN_STS)) {
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		dev_dbg(ln8411->dev, "TSBAT condition detected!\n");
		return ret;
	}

	val->intval = POWER_SUPPLY_HEALTH_GOOD;

	return ret;
}

static int ln8411_get_charger_health(struct ln8411_device *ln8411, union power_supply_propval *val)
{
	u8 buf[3];
	int ret;

	val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;

	ret = regmap_bulk_read(ln8411->regmap, LN8411_FAULT1_STS, buf, 3);
	if (ret)
		return ret;

	if (buf[0] & LN8411_WATCHDOG_TIMER_STS) {
		val->intval = POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE;
	} else if (buf[2] & LN8411_IBUS_OC_DETECTED) {
		val->intval = POWER_SUPPLY_HEALTH_OVERCURRENT;
		dev_dbg(ln8411->dev, "IBUS overcurrent condition detected!\n");
	} else if (buf[0] & LN8411_PMID2OUT_OV_STS) {
		val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		dev_dbg(ln8411->dev, "PMID2OUT overvoltage condition detected!\n");
	} else {
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
	}

	return ret;
}

static int ln8411_set_present(struct ln8411_device *ln8411, const union power_supply_propval *val)
{
	int ret;

	if (!val->intval) {
		mutex_lock(&ln8411->lock);

		disable_irq(ln8411->irq);

		ret = ln8411_soft_reset(ln8411);
		if (ret)
			goto err_out;

		regcache_mark_dirty(ln8411->regmap);
		ret = regcache_sync(ln8411->regmap);
		if (ret)
			goto err_out;

		ret = ln8411_hw_init(ln8411);
		if (ret)
			goto err_out;

		enable_irq(ln8411->irq);

		mutex_unlock(&ln8411->lock);
	}

	return 0;

err_out:
	mutex_unlock(&ln8411->lock);
	return ret;
}

static int ln8411_set_status_not_charging(struct ln8411_device *ln8411)
{
	int ret;

	if ((ln8411->rev == LN8411_A1_DEV_REV_ID) && ln8411->state.mode == LN8411_FWD2TO1) {
		/* Disable A1 2:1 workaround */
		ret = ln8411_a1_2to1_workaround(ln8411, false);
		if (ret)
			return ret;
	}

	ret = regmap_clear_bits(ln8411->regmap, LN8411_CTRL1, LN8411_CP_EN);
	if (ret)
		return ret;

	ret = regmap_clear_bits(ln8411->regmap, LN8411_CTRL1, LN8411_QB_EN);
	if (ret)
		return ret;

	if (ln8411->en_gpio)
		gpiod_set_value(ln8411->en_gpio, false);

	return ret;
}

static void ln8411_charge_en_work(struct work_struct *work)
{
	struct ln8411_device *ln8411 =
		container_of(work, struct ln8411_device, charge_en_work.work);
	union power_supply_propval psy_val = {0};
	unsigned int reg_val;

	regmap_read(ln8411->regmap, LN8411_SYS_STS, &reg_val);

	if (reg_val & LN8411_ACTIVE_STS) {
		dev_dbg(ln8411->dev, "Successfully initiated charging\n");
	} else {
		dev_err(ln8411->dev, "Failed to initiate charging\n");

		if (!(reg_val & LN8411_PMID_SWITCH_OK_STS))
			dev_dbg(ln8411->dev, "Check PMID voltage\n");
		if (!(reg_val & LN8411_INFET_OK_STS))
			dev_dbg(ln8411->dev, "Power path blocked, check INFET\n");

		/* Converter cannot be re-initiated without clearing CP_EN*/
		psy_val.intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		power_supply_set_property(ln8411->charger, POWER_SUPPLY_PROP_STATUS, &psy_val);
	}

	power_supply_changed(ln8411->charger);
}

static int ln8411_set_status_charging(struct ln8411_device *ln8411)
{
	int ret;

	if ((ln8411->rev == LN8411_A1_DEV_REV_ID) && ln8411->state.mode == LN8411_FWD2TO1) {
		/* Enable A1 2:1 workaround */
		ret = ln8411_a1_2to1_workaround(ln8411, true);
		if (ret)
			return ret;
	}

	if (ln8411->en_gpio)
		gpiod_set_value(ln8411->en_gpio, true);

	ret = regmap_set_bits(ln8411->regmap, LN8411_CTRL1, LN8411_QB_EN);
	if (ret)
		return ret;

	usleep_range(21000, 22000);

	ret = regmap_set_bits(ln8411->regmap, LN8411_CTRL1, LN8411_CP_EN);
	if (ret)
		return ret;

	queue_delayed_work(system_wq, &ln8411->charge_en_work, msecs_to_jiffies(200));

	return ret;
}

static int ln8411_set_status(struct ln8411_device *ln8411, const int psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_STATUS_CHARGING:
		ret = ln8411_set_status_charging(ln8411);
		break;
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		ret = ln8411_set_status_not_charging(ln8411);
		break;
	default:
		return -EINVAL;
	}

	if (ret)
		return ret;

	ln8411->state.charging_status = psp;

	return ret;
}

static int ln8411_get_status(struct ln8411_device *ln8411)
{
	unsigned int reg_code;
	int ret;

	ln8411->state.charging_status = POWER_SUPPLY_STATUS_UNKNOWN;

	ret = regmap_read(ln8411->regmap, LN8411_SYS_STS, &reg_code);
	if (ret)
		return ret;

	if (reg_code & LN8411_ACTIVE_STS && (ln8411->state.mode < LN8411_REV1TO4))
		ln8411->state.charging_status = POWER_SUPPLY_STATUS_CHARGING;
	else if (reg_code & LN8411_SHUTDOWN_STS)
		ln8411->state.charging_status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (reg_code & LN8411_STANDBY_STS)
		ln8411->state.charging_status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return ret;
}

static int ln8411_set_mode(struct ln8411_device *ln8411, const int val)
{
	enum ln8411_modes mode;
	int ret;

	switch (val) {
	case POWER_SUPPLY_CHARGE_TYPE_STANDARD:
		mode = LN8411_FWD2TO1;
		break;
	case POWER_SUPPLY_CHARGE_TYPE_FAST:
		mode = LN8411_FWD4TO1;
		break;
	case POWER_SUPPLY_CHARGE_TYPE_BYPASS:
		mode = LN8411_FWD1TO1;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_update_bits(ln8411->regmap, LN8411_CTRL4, LN8411_MODE_MASK, mode);
	if (ret)
		return ret;

	ln8411->state.mode = mode;

	return ret;
}

static int ln8411_get_mode(struct ln8411_device *ln8411, int *type)
{
	unsigned int reg_code;
	int ret;

	ret = regmap_read(ln8411->regmap, LN8411_CTRL4, &reg_code);

	ln8411->state.mode = reg_code & LN8411_MODE_MASK;

	switch (ln8411->state.mode) {
	case LN8411_FWD4TO1:
		*type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case LN8411_FWD2TO1:
		*type = POWER_SUPPLY_CHARGE_TYPE_STANDARD;
		break;
	case LN8411_FWD1TO1:
		*type = POWER_SUPPLY_CHARGE_TYPE_BYPASS;
		break;
	case LN8411_REV1TO4...LN8411_REV1TO1:
		*type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	default:
		*type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	return ret;
}

static const int ln8411_wd_timeout_ms[LN8411_NUM_WD_MS_VAL] = {
	0, 200, 500, 1000, 5000, 30000, 100000, 255000
};

static int ln8411_set_wd_timeout(struct ln8411_device *ln8411)
{
	struct ln8411_init_data *init_data = &ln8411->init_data;
	unsigned int reg_code = LN8411_WD_TIMEOUT_DIS;
	int i;

	for (i = 0; i < LN8411_NUM_WD_MS_VAL; i++) {
		if (init_data->wd_timeout_ms == ln8411_wd_timeout_ms[i]) {
			reg_code = i;
			break;
		}
	}

	return regmap_update_bits(ln8411->regmap, LN8411_CTRL3,
				  LN8411_WD_TIMEOUT_CFG_MASK, reg_code);
}

static const int ln8411_fsw_freqs_hz[LN8411_NUM_FREQ_VAL] = {
	235000, 260000, 290000, 315000, 340000, 365000, 390000, 420000, 435000, 460000,
	480000, 505000, 530000, 550000, 570000, 595000, 470000, 530000, 580000, 635000,
	685000, 735000, 785000, 835000, 870000, 920000, 965000, 1015000, 1060000, 1105000,
	1150000, 1070000
};

static int ln8411_set_fsw(struct ln8411_device *ln8411)
{
	unsigned int reg_code = LN8411_FSW_DFLT << LN8411_FSW_SHIFT;
	struct ln8411_init_data *init_data = &ln8411->init_data;
	int i;

	for (i = 0; i < LN8411_NUM_FREQ_VAL; i++) {
		if (init_data->fsw_hz == ln8411_fsw_freqs_hz[i]) {
			reg_code = i << LN8411_FSW_SHIFT;
			break;
		}
	}

	return regmap_update_bits(ln8411->regmap, LN8411_CTRL2, LN8411_FSW_SET_MASK, reg_code);
}

static int ln8411_cfg_pmid2out_uvp(struct ln8411_device *ln8411)
{
	struct ln8411_init_data *init_data = &ln8411->init_data;
	unsigned int reg_code = 0;

	if (init_data->pmid2out_uvp_dis)
		reg_code |= LN8411_PMID2OUT_UVP_DIS;
	if (init_data->pmid2out_uvp_mask)
		reg_code |= LN8411_PMID2OUT_UVP_MASK;
	reg_code |= init_data->pmid2out_uvp;

	return regmap_update_bits(ln8411->regmap, LN8411_PMID2OUT_UVP,
				  LN8411_PMID2OUT_UVP_SET_MASK, reg_code);
}

static int ln8411_cfg_pmid2out_ovp(struct ln8411_device *ln8411)
{
	struct ln8411_init_data *init_data = &ln8411->init_data;
	unsigned int reg_code = 0;

	if (init_data->pmid2out_ovp_dis)
		reg_code |= LN8411_PMID2OUT_OVP_DIS;
	if (init_data->pmid2out_ovp_mask)
		reg_code |= LN8411_PMID2OUT_OVP_MASK;
	reg_code |= init_data->pmid2out_ovp;

	return regmap_update_bits(ln8411->regmap, LN8411_PMID2OUT_OVP,
				  LN8411_PMID2OUT_OVP_SET_MASK, reg_code);
}

static int ln8411_cfg_ibus_ucp(struct ln8411_device *ln8411)
{
	struct ln8411_init_data *init_data = &ln8411->init_data;
	unsigned int reg_code = 0;

	if (init_data->ibus_ucp_dis)
		reg_code |= LN8411_IBUS_UCP_DIS;
	if (init_data->ibus_ucp_rise_mask)
		reg_code |= LN8411_IBUS_UCP_RISE_MASK;
	if (init_data->ibus_ucp_fall_mask)
		reg_code |= LN8411_IBUS_UCP_FALL_MASK;

	return regmap_update_bits(ln8411->regmap, LN8411_IBUS_UCP,
				  LN8411_IBUS_UCP_SET_MASK, reg_code);
}

static int ln8411_cfg_ibus_ocp(struct ln8411_device *ln8411)
{
	struct ln8411_init_data *init_data = &ln8411->init_data;
	unsigned int reg_code = 0;

	if (init_data->ibus_ocp_dis)
		reg_code |= LN8411_IBUS_OCP_DIS;
	if (init_data->ibus_ocp_mask)
		reg_code |= LN8411_IBUS_OCP_MASK;

	return regmap_update_bits(ln8411->regmap, LN8411_IBUS_OCP,
				  LN8411_IBUS_OCP_SET_MASK, reg_code);
}

static inline int ln8411_set_ibus_oc_lvl(struct ln8411_device *ln8411, const bool ibus_oc_lvl)
{
	int ret;

	ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_TEST_MODE);
	if (ret)
		return ret;

	if (ibus_oc_lvl)
		ret = regmap_set_bits(ln8411->regmap, LN8411_CFG_10, LN8411_IBUS_OC_LVL_LOW_RANGE);
	else
		ret = regmap_clear_bits(ln8411->regmap, LN8411_CFG_10,
					LN8411_IBUS_OC_LVL_LOW_RANGE);
	if (ret)
		return ret;

	ln8411->state.ibus_oc_lvl = ibus_oc_lvl;

	return ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_LOCK);
}

static int ln8411_set_ibus_ocp(struct ln8411_device *ln8411, int val)
{
	unsigned int offset_ua, reg_code, step_ua;
	int ret;

	val = clamp(val, LN8411_IBUS_OCP_MIN_UA, LN8411_IBUS_OCP_MAX_UA);

	if (val < LN8411_IBUS_OCP_MIN_U_UA) {
		if (!ln8411->state.ibus_oc_lvl) {
			ret = ln8411_set_ibus_oc_lvl(ln8411, true);
			if (ret)
				return ret;
		}

		offset_ua = LN8411_IBUS_OCP_OFFSET_L_UA;
		step_ua = LN8411_IBUS_OCP_STEP_L_UA;
	} else {
		if (ln8411->state.ibus_oc_lvl) {
			ret = ln8411_set_ibus_oc_lvl(ln8411, false);
			if (ret)
				return ret;
		}

		offset_ua = LN8411_IBUS_OCP_OFFSET_U_UA;
		step_ua = LN8411_IBUS_OCP_STEP_U_UA;
	}

	reg_code = (val - offset_ua) / step_ua;

	ret = regmap_update_bits(ln8411->regmap,
				 LN8411_IBUS_OCP, LN8411_IBUS_OCP_CFG_MASK, reg_code);
	if (ret)
		return ret;

	ln8411->state.ibus_ocp_ua = val;

	return ret;
}

static int ln8411_get_ibus_ocp(struct ln8411_device *ln8411)
{
	unsigned int offset_ua, reg_code, step_ua;
	int ret;

	ret = regmap_read(ln8411->regmap, LN8411_CFG_10, &reg_code);
	if (ret)
		return ret;

	if (reg_code & LN8411_IBUS_OC_LVL_LOW_RANGE) {
		ln8411->state.ibus_oc_lvl = true;
		offset_ua = LN8411_IBUS_OCP_OFFSET_L_UA;
		step_ua = LN8411_IBUS_OCP_STEP_L_UA;
	} else {
		ln8411->state.ibus_oc_lvl = false;
		offset_ua = LN8411_IBUS_OCP_OFFSET_U_UA;
		step_ua = LN8411_IBUS_OCP_STEP_U_UA;
	}

	ret = regmap_read(ln8411->regmap, LN8411_IBUS_OCP, &reg_code);
	if (ret)
		return ret;

	ln8411->state.ibus_ocp_ua = ((reg_code & LN8411_IBUS_OCP_CFG_MASK) * step_ua) + offset_ua;

	return ret;
}

static int ln8411_cfg_vwpc_ovp(struct ln8411_device *ln8411)
{
	struct ln8411_init_data *init_data = &ln8411->init_data;
	unsigned int reg_code = 0;

	if (init_data->wpcgate_on_dg_set)
		reg_code |= LN8411_VWPC_OVP_DG_SET;
	if (init_data->vwpc_ovp_mask)
		reg_code |= LN8411_VWPC_OVP_MASK;

	return regmap_update_bits(ln8411->regmap, LN8411_VWPC_OVP,
				  LN8411_VWPC_OVP_SET_MASK, reg_code);
}

static int ln8411_set_vwpc_ovp(struct ln8411_device *ln8411, int val)
{
	unsigned int reg_code;
	int ret;

	if (val == LN8411_VWPC_OVP_DFLT_UV) {
		reg_code = LN8411_VWPC_OVP_DFLT;
	} else {
		val = clamp(val, LN8411_VWPC_OVP_MIN_UV, LN8411_VWPC_OVP_MAX_UV);
		reg_code = (val - LN8411_VWPC_OVP_OFFSET_UV) / LN8411_VWPC_OVP_STEP_UV;
	}

	ret = regmap_update_bits(ln8411->regmap,
				 LN8411_VWPC_OVP, LN8411_VWPC_OVP_CFG_MASK, reg_code);
	if (ret)
		return ret;

	ln8411->state.vwpc_ovp_uv = val;

	return ret;
}

static int ln8411_cfg_vusb_ovp(struct ln8411_device *ln8411)
{
	struct ln8411_init_data *init_data = &ln8411->init_data;
	unsigned int reg_code = 0;

	if (init_data->ovpgate_on_dg_set)
		reg_code |= LN8411_VUSB_OVP_DG_SET;
	if (init_data->vusb_ovp_mask)
		reg_code |= LN8411_VUSB_OVP_MASK;

	return regmap_update_bits(ln8411->regmap, LN8411_VUSB_OVP,
				  LN8411_VUSB_OVP_SET_MASK, reg_code);
}

static int ln8411_set_vusb_ovp(struct ln8411_device *ln8411, int val)
{
	unsigned int reg_code;
	int ret;

	if (val == LN8411_VUSB_OVP_DFLT_UV) {
		reg_code = LN8411_VUSB_OVP_DFLT;
	} else {
		val = clamp(val, LN8411_VUSB_OVP_MIN_UV, LN8411_VUSB_OVP_MAX_UV);
		reg_code = (val - LN8411_VUSB_OVP_OFFSET_UV) / LN8411_VUSB_OVP_STEP_UV;
	}

	ret = regmap_update_bits(ln8411->regmap,
				 LN8411_VUSB_OVP, LN8411_VUSB_OVP_CFG_MASK, reg_code);
	if (ret)
		return ret;

	ln8411->state.vusb_ovp_uv = val;

	return ret;
}

static int ln8411_cfg_ibat_ocp(struct ln8411_device *ln8411)
{
	struct ln8411_init_data *init_data = &ln8411->init_data;
	unsigned int reg_code = 0;

	if (init_data->ibat_ocp_dis)
		reg_code |= LN8411_IBAT_OCP_DIS;
	if (init_data->ibat_ocp_mask)
		reg_code |= LN8411_IBAT_OCP_MASK;

	return regmap_update_bits(ln8411->regmap, LN8411_IBAT_OCP,
				  LN8411_IBAT_OCP_SET_MASK, reg_code);
}

static int ln8411_get_ibat_ocp(struct ln8411_device *ln8411)
{
	unsigned int reg_code;
	int ret;

	ret = regmap_read(ln8411->regmap, LN8411_IBAT_OCP, &reg_code);
	if (ret)
		return ret;

	ln8411->state.ibat_ocp_ua = ((reg_code & LN8411_IBAT_OCP_CFG_MASK) *
			      LN8411_IBAT_OCP_STEP_UA) + LN8411_IBAT_OCP_OFFSET_UA;

	return ret;
}

static int ln8411_set_ibat_ocp(struct ln8411_device *ln8411, int val)
{
	unsigned int reg_code;
	int ret;

	val = clamp(val, LN8411_IBAT_OCP_MIN_UA, LN8411_IBAT_OCP_MAX_UA);

	reg_code = (val - LN8411_IBAT_OCP_OFFSET_UA) / LN8411_IBAT_OCP_STEP_UA;

	ret = regmap_update_bits(ln8411->regmap, LN8411_IBAT_OCP,
				 LN8411_IBAT_OCP_CFG_MASK, reg_code);
	if (ret)
		return ret;

	ln8411->state.ibat_ocp_ua = val;

	return ret;
}

static int ln8411_cfg_vbat_ovp(struct ln8411_device *ln8411)
{
	struct ln8411_init_data *init_data = &ln8411->init_data;
	unsigned int reg_code = 0;

	if (init_data->vbat_ovp_dis)
		reg_code |= LN8411_VBAT_OVP_DIS;
	if (init_data->vbat_ovp_dg_set)
		reg_code |= LN8411_VBAT_OVP_DG_SET;
	if (init_data->vbat_ovp_mask)
		reg_code |= LN8411_VBAT_OVP_MASK;

	return regmap_update_bits(ln8411->regmap, LN8411_VBAT_OVP,
				  LN8411_VBAT_OVP_SET_MASK, reg_code);
}

static int ln8411_get_vbat_ovp(struct ln8411_device *ln8411)
{
	unsigned int reg_code;
	int ret;

	ret = regmap_read(ln8411->regmap, LN8411_VBAT_OVP, &reg_code);
	if (ret)
		return ret;

	ln8411->state.vbat_ovp_uv = ((reg_code & LN8411_VBAT_OVP_CFG_MASK) *
				     LN8411_VBAT_OVP_STEP_UV) + LN8411_VBAT_OVP_OFFSET_UV;

	return ret;
}

static int ln8411_set_vbat_ovp(struct ln8411_device *ln8411, int val)
{
	unsigned int reg_code;
	int ret;

	val = clamp(val, LN8411_VBAT_OVP_MIN_UV, LN8411_VBAT_OVP_MAX_UV);

	reg_code = (val - LN8411_VBAT_OVP_OFFSET_UV) / LN8411_VBAT_OVP_STEP_UV;

	ret = regmap_update_bits(ln8411->regmap, LN8411_VBAT_OVP,
				 LN8411_VBAT_OVP_CFG_MASK, reg_code);
	if (ret)
		return ret;

	ln8411->state.vbat_ovp_uv = val;

	return ret;
}

static int ln8411_pre_irq_handler(void *irq_drv_data)
{
	struct ln8411_device *ln8411 = irq_drv_data;

	return regmap_set_bits(ln8411->regmap, LN8411_LION_INT_MASK_2, LN8411_PAUSE_INT_UPDATE);
}

static int ln8411_post_irq_handler(void *irq_drv_data)
{
	struct ln8411_device *ln8411 = irq_drv_data;
	union power_supply_propval val = {0};
	unsigned int reg;
	int ret;

	ret = regmap_read(ln8411->regmap, LN8411_COMP_FLAG0, &reg);
	if (ret)
		return ret;

	/* CP_EN bit does not auto clear during unplug, must be manually cleared*/
	if (reg & LN8411_IBUS_UCP_FALL_FLAG) {
		val.intval = POWER_SUPPLY_STATUS_NOT_CHARGING;

		ret = power_supply_set_property(ln8411->charger, POWER_SUPPLY_PROP_STATUS, &val);
		if (ret)
			return ret;

		dev_dbg(ln8411->dev, "Unplug detected, disabling converter\n");
	}

	dev_dbg(ln8411->dev, "COMP_FLAG0: 0x%x\n", reg);

	ret = regmap_read(ln8411->regmap, LN8411_COMP_FLAG1, &reg);
	if (ret)
		return ret;

	dev_dbg(ln8411->dev, "COMP_FLAG1: 0x%x\n", reg);

	ret = regmap_set_bits(ln8411->regmap, LN8411_LION_INT_MASK_2, LN8411_CLEAR_INT);
	if (ret)
		return ret;

	usleep_range(5000, 5100);

	ret = regmap_clear_bits(ln8411->regmap, LN8411_LION_INT_MASK_2, LN8411_CLEAR_INT);
	if (ret)
		return ret;

	return regmap_clear_bits(ln8411->regmap, LN8411_LION_INT_MASK_2, LN8411_PAUSE_INT_UPDATE);
}

static irqreturn_t ln8411_vout_ovp_handler(int irq, void *private)
{
	struct ln8411_device *ln8411 = private;

	dev_dbg(ln8411->dev, "VOUT OVP threshold has been exceeded\n");

	power_supply_changed(ln8411->battery);

	return IRQ_HANDLED;
}

static irqreturn_t ln8411_vbus_ovp_handler(int irq, void *private)
{
	struct ln8411_device *ln8411 = private;

	dev_dbg(ln8411->dev, "VBUS OVP threshold has been exceeded\n");

	power_supply_changed(ln8411->charger);

	return IRQ_HANDLED;
}

static irqreturn_t ln8411_conv_ocp_handler(int irq, void *private)
{
	struct ln8411_device *ln8411 = private;

	dev_dbg(ln8411->dev, "MOS OCP event has occurred\n");

	power_supply_changed(ln8411->charger);

	return IRQ_HANDLED;
}

static irqreturn_t ln8411_wd_timeout_handler(int irq, void *private)
{
	struct ln8411_device *ln8411 = private;
	int ret;

	dev_dbg(ln8411->dev, "Watchdog timeout has occurred\n");

	regcache_mark_dirty(ln8411->regmap);
	regcache_sync(ln8411->regmap);

	ret = ln8411_hw_init(ln8411);
	if (ret) {
		dev_err(ln8411->dev, "Hardware reinit has failed: %d", ret);
		return IRQ_NONE;
	}

	dev_dbg(ln8411->dev, "Hardware reinit was successful\n");

	power_supply_changed(ln8411->charger);

	return IRQ_HANDLED;
}

static irqreturn_t ln8411_tshut_handler(int irq, void *private)
{
	struct ln8411_device *ln8411 = private;

	dev_dbg(ln8411->dev, "Thermal shutdown condition");

	power_supply_changed(ln8411->charger);

	return IRQ_HANDLED;
}

static irqreturn_t ln8411_tsbat_flt_handler(int irq, void *private)
{
	struct ln8411_device *ln8411 = private;

	dev_dbg(ln8411->dev, "TSBAT fault condition\n");

	power_supply_changed(ln8411->charger);
	power_supply_changed(ln8411->battery);

	return IRQ_HANDLED;
}

static irqreturn_t ln8411_vusb_insert_handler(int irq, void *private)
{
	struct ln8411_device *ln8411 = private;

	dev_dbg(ln8411->dev, "VUSB UVLO threshold exceeded\n");

	if (ln8411->vusb) {
		power_supply_external_power_changed(ln8411->vusb);
		power_supply_changed(ln8411->vusb);
	}

	extcon_set_state_sync(ln8411->edev, EXTCON_USB, true);

	power_supply_external_power_changed(ln8411->charger);
	power_supply_changed(ln8411->charger);

	return IRQ_HANDLED;
}

static irqreturn_t ln8411_vwpc_insert_handler(int irq, void *private)
{
	struct ln8411_device *ln8411 = private;

	dev_dbg(ln8411->dev, "VWPC UVLO threshold exceeded\n");

	if (ln8411->vwpc) {
		extcon_set_state_sync(ln8411->edev, EXTCON_CHG_WPT, true);

		power_supply_external_power_changed(ln8411->vwpc);
		power_supply_changed(ln8411->vwpc);
	}

	power_supply_external_power_changed(ln8411->charger);
	power_supply_changed(ln8411->charger);

	return IRQ_HANDLED;
}

static irqreturn_t ln8411_vout_insert_handler(int irq, void *private)
{
	struct ln8411_device *ln8411 = private;

	dev_dbg(ln8411->dev, "VOUT UVLO threshold exceeded\n");

	power_supply_changed(ln8411->battery);

	return IRQ_HANDLED;
}

static const struct ln8411_irq ln8411_irqs[] = {
	LN8411_IRQ(VOUT_INSERT, "VOUT Insertion", ln8411_vout_insert_handler),
	LN8411_IRQ(VWPC_INSERT, "VWPC Insertion", ln8411_vwpc_insert_handler),
	LN8411_IRQ(VUSB_INSERT, "VUSB Insertion", ln8411_vusb_insert_handler),
	LN8411_IRQ(TSBAT_FLT, "TSBAT Fault", ln8411_tsbat_flt_handler),
	LN8411_IRQ(TSHUT, "Thermal Shutdown", ln8411_tshut_handler),
	LN8411_IRQ(WD_TIMEOUT, "Watchdog Timeout", ln8411_wd_timeout_handler),
	LN8411_IRQ(CONV_OCP, "Converter Overcurrent", ln8411_conv_ocp_handler),
	LN8411_IRQ(VBUS_OVP, "VBUS Overvoltage", ln8411_vbus_ovp_handler),
	LN8411_IRQ(VOUT_OVP, "VOUT Overvoltage", ln8411_vout_ovp_handler),
};

static const struct regmap_irq ln8411_reg_irqs[] = {
	LN8411_REG_IRQ(INT_FLAG, VOUT_INSERT),
	LN8411_REG_IRQ(INT_FLAG, VWPC_INSERT),
	LN8411_REG_IRQ(INT_FLAG, VUSB_INSERT),
	LN8411_REG_IRQ(FLT_FLAG, TSBAT_FLT),
	LN8411_REG_IRQ(FLT_FLAG, TSHUT),
	LN8411_REG_IRQ(FLT_FLAG, WD_TIMEOUT),
	LN8411_REG_IRQ(FLT_FLAG, CONV_OCP),
	LN8411_REG_IRQ(FLT_FLAG, VBUS_OVP),
	LN8411_REG_IRQ(FLT_FLAG, VOUT_OVP),
};

static struct regmap_irq_chip ln8411_regmap_irq_chip = {
	.name = "ln8411 IRQ Controller",
	.status_base = LN8411_INT_FLAG,
	.mask_base = LN8411_INT_MASK,
	.ack_base = LN8411_INT_FLAG,
	.num_regs = 3,
	.irqs = ln8411_reg_irqs,
	.num_irqs = ARRAY_SIZE(ln8411_reg_irqs),
	.handle_post_irq = ln8411_post_irq_handler,
	.handle_pre_irq = ln8411_pre_irq_handler,
};

static int ln8411_regmap_irq_init(struct ln8411_device *ln8411, struct device *dev)
{
	int i, irq, ret;

	ln8411_regmap_irq_chip.irq_drv_data = ln8411;

	ret = devm_regmap_add_irq_chip(dev, ln8411->regmap, ln8411->irq,
				       IRQF_ONESHOT, 0,
					       &ln8411_regmap_irq_chip,
					       &ln8411->irq_data);
	if (ret) {
		dev_err(dev, "Failed to register IRQ chip: %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(ln8411_irqs); i++) {
		irq = regmap_irq_get_virq(ln8411->irq_data, ln8411_irqs[i].irq);
		if (irq < 0) {
			dev_err(dev, "Failed to get %s\n", ln8411_irqs[i].name);
			return irq;
		}
		ret = devm_request_threaded_irq(dev, irq, NULL,
						ln8411_irqs[i].handler,
						IRQF_TRIGGER_LOW,
						ln8411_irqs[i].name,
						ln8411);
		if (ret) {
			dev_err(dev, "Failed to request irq %s: %d\n", ln8411_irqs[i].name, ret);
			return ret;
		}
	}

	return 0;
}

static int ln8411_otg_get_voltage(struct regulator_dev *rdev)
{
	struct ln8411_device *ln8411 = rdev_get_drvdata(rdev);
	union power_supply_propval val = {0};
	int ret;

	if (ln8411->state.mode < LN8411_REV1TO4)
		return LN8411_OTG_MIN_UV;

	ret = power_supply_get_property(ln8411->charger,
					POWER_SUPPLY_PROP_VOLTAGE_NOW,
					&val);
	if (ret)
		return ret;

	return val.intval;
}

static int ln8411_otg_set_voltage(struct regulator_dev *rdev,
				  int min_uV, int max_uV, unsigned int *selector)
{
	struct ln8411_device *ln8411 = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	if (min_uV <= LN8411_OTG_MIN_UV)
		val = LN8411_REV1TO1;
	else if (min_uV <= LN8411_OTG_MIN_1TO2_MIN_UV)
		val = LN8411_REV1TO2;
	else if (min_uV <= LN8411_OTG_MIN_1TO4_MIN_UV)
		val = LN8411_REV1TO4;
	else
		return -EPERM;

	ret = regmap_update_bits(ln8411->regmap, LN8411_CTRL4, LN8411_MODE_MASK, val);
	if (ret)
		return ret;

	ln8411->state.mode = val;

	return ret;

}

static int ln8411_set_current_limit_otg(struct regulator_dev *rdev, int min_uA, int max_uA)
{
	struct ln8411_device *ln8411 = rdev_get_drvdata(rdev);
	union power_supply_propval val = {0};

	val.intval = max_uA;

	return power_supply_set_property(ln8411->charger,
					 POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
					 &val);
}

static int ln8411_get_current_limit_otg(struct regulator_dev *rdev)
{
	struct ln8411_device *ln8411 = rdev_get_drvdata(rdev);
	union power_supply_propval val = {0};
	int ret;

	ret = power_supply_get_property(ln8411->charger,
					POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
					&val);
	if (ret)
		return ret;

	return val.intval;
}

static int ln8411_is_enabled_otg(struct ln8411_device *ln8411)
{
	unsigned int reg_code;
	int ret;

	ret = regmap_read(ln8411->regmap, LN8411_SYS_STS, &reg_code);
	if (ret)
		return ret;

	if (reg_code & LN8411_ACTIVE_STS && (ln8411->state.mode > LN8411_FWD1TO1))
		return true;
	else
		return false;
}

static int ln8411_is_enabled_vwpc_otg(struct regulator_dev *rdev)
{
	struct ln8411_device *ln8411 = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(ln8411->regmap, LN8411_CTRL5, &val);
	if (ret)
		return ret;

	if (val & LN8411_WPCGATE_STAT)
		return ln8411_is_enabled_otg(ln8411);
	else
		return false;
}

static int ln8411_enable_otg(struct ln8411_device *ln8411)
{
	int ret;

	dev_dbg(ln8411->dev, "Entering reverse mode\n");

	ret = regmap_set_bits(ln8411->regmap, LN8411_IBUS_UCP, LN8411_IBUS_UCP_DIS);
	if (ret)
		return ret;

	ret = regmap_set_bits(ln8411->regmap, LN8411_PMID2OUT_UVP, LN8411_PMID2OUT_UVP_DIS);
	if (ret)
		return ret;

	ret = regmap_clear_bits(ln8411->regmap, LN8411_LION_CFG_1, LN8411_DEVICE_MODE);
	if (ret)
		return ret;

	ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_TEST_MODE);
	if (ret)
		return ret;

	ret = regmap_set_bits(ln8411->regmap, LN8411_LION_COMP_CTRL_1, LN8411_PMID_SWITCH_OK_DIS);
	if (ret)
		return ret;

	ret = regmap_set_bits(ln8411->regmap, LN8411_LION_COMP_CTRL_2, LN8411_VBUS_UVP_DIS);
	if (ret)
		return ret;

	ret = regmap_set_bits(ln8411->regmap, LN8411_LION_COMP_CTRL_4, LN8411_INFET_OFF_DET_DIS);
	if (ret)
		return ret;

	ret = regmap_set_bits(ln8411->regmap, LN8411_TRIM_8, LN8411_IBUS_REV_ISNS_EN);
	if (ret)
		return ret;

	ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_LOCK);
	if (ret)
		return ret;

	if ((ln8411->rev == LN8411_A1_DEV_REV_ID) && (ln8411->state.mode == LN8411_REV1TO2)) {
		/* Enable A1 2:1 workaround */
		ret = ln8411_a1_2to1_workaround(ln8411, true);
		if (ret)
			return ret;
	}

	ret = regmap_set_bits(ln8411->regmap, LN8411_CTRL1, LN8411_QB_EN);
	if (ret)
		return ret;

	usleep_range(21000, 22000);

	return regmap_set_bits(ln8411->regmap, LN8411_CTRL1, LN8411_CP_EN);
}

static int ln8411_enable_vwpc_otg(struct regulator_dev *rdev)
{
	struct ln8411_device *ln8411 = rdev_get_drvdata(rdev);
	union power_supply_propval val = {0};
	int ret;

	ret = ln8411_enable_otg(ln8411);
	if (ret)
		return ret;

	if (ln8411->role == LN8411_SECONDARY)
		return ret;

	val.intval = true;

	usleep_range(21000, 22000);

	return power_supply_set_property(ln8411->vwpc, POWER_SUPPLY_PROP_ONLINE, &val);
}

static void ln8411_pulldown_res_work(struct work_struct *work)
{
	struct ln8411_device *ln8411 =
		container_of(work, struct ln8411_device, pulldown_res_work.work);

	regmap_clear_bits(ln8411->regmap, LN8411_CTRL1, LN8411_PD_EN_MASK);
}

static int ln8411_disable_otg(struct regulator_dev *rdev)
{
	struct ln8411_device *ln8411 = rdev_get_drvdata(rdev);
	union power_supply_propval val = {0};
	int ret;

	dev_dbg(ln8411->dev, "Exiting reverse mode\n");

	val.intval = POWER_SUPPLY_STATUS_NOT_CHARGING;

	ret = power_supply_set_property(ln8411->charger, POWER_SUPPLY_PROP_STATUS, &val);
	if (ret) {
		dev_err(ln8411->dev, "Failed to disable charge pump: %d\n", ret);
		return ret;
	}

	if (!ln8411->init_data.ibus_ocp_dis) {
		ret = regmap_clear_bits(ln8411->regmap, LN8411_IBUS_UCP, LN8411_IBUS_UCP_DIS);
		if (ret)
			return ret;
	}

	if (!ln8411->init_data.pmid2out_uvp_dis) {
		ret = regmap_clear_bits(ln8411->regmap,
					LN8411_PMID2OUT_UVP, LN8411_PMID2OUT_UVP_DIS);
		if (ret)
			return ret;
	}

	ret = regmap_set_bits(ln8411->regmap, LN8411_LION_CFG_1, LN8411_DEVICE_MODE);
	if (ret)
		return ret;

	if ((ln8411->rev == LN8411_A1_DEV_REV_ID) && (ln8411->state.mode == LN8411_REV1TO2)) {
		/* Disable A1 2:1 workaround */
		ret = ln8411_a1_2to1_workaround(ln8411, false);
		if (ret)
			return ret;
	}

	ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_TEST_MODE);
	if (ret)
		return ret;

	ret = regmap_clear_bits(ln8411->regmap, LN8411_LION_COMP_CTRL_1, LN8411_PMID_SWITCH_OK_DIS);
	if (ret)
		return ret;

	ret = regmap_clear_bits(ln8411->regmap, LN8411_LION_COMP_CTRL_2, LN8411_VBUS_UVP_DIS);
	if (ret)
		return ret;

	ret = regmap_clear_bits(ln8411->regmap, LN8411_LION_COMP_CTRL_4, LN8411_INFET_OFF_DET_DIS);
	if (ret)
		return ret;

	ret = regmap_clear_bits(ln8411->regmap, LN8411_TRIM_8, LN8411_IBUS_REV_ISNS_EN);
	if (ret)
		return ret;

	return ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_LOCK);
}

static const struct regulator_init_data ln8411_otg_init_data = {
	.constraints = {
		.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				   REGULATOR_CHANGE_CURRENT |
				   REGULATOR_CHANGE_STATUS),
		.max_uV = 20900000,
		.min_uV = 3500000,
	},
};

static const struct regulator_ops ln8411_chg_vwpc_otg_ops = {
	.disable = ln8411_disable_otg,
	.enable = ln8411_enable_vwpc_otg,
	.is_enabled = ln8411_is_enabled_vwpc_otg,
	.get_current_limit = ln8411_get_current_limit_otg,
	.set_current_limit = ln8411_set_current_limit_otg,
	.set_voltage = ln8411_otg_set_voltage,
	.get_voltage = ln8411_otg_get_voltage,
};

static const struct regulator_desc ln8411_vwpc_otg_desc = {
	.name = "ln8411-otg-vwpc",
	.ops = &ln8411_chg_vwpc_otg_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.of_match = "ln8411-otg-vwpc",
};

static int ln8411_wpc_otg_regulator_register(struct ln8411_device *ln8411)
{
	struct regulator_config reg_cfg = { };

	reg_cfg.dev = ln8411->dev;
	reg_cfg.driver_data = ln8411;
	reg_cfg.init_data = &ln8411_otg_init_data;
	reg_cfg.regmap = ln8411->regmap;

	ln8411->otg_wpc_reg = devm_regulator_register(ln8411->dev,
						      &ln8411_vwpc_otg_desc,
						      &reg_cfg);
	if (IS_ERR(ln8411->otg_wpc_reg)) {
		dev_err(ln8411->dev, "Failed to register VWPC OTG regulator\n");
		return PTR_ERR(ln8411->otg_wpc_reg);
	}

	return 0;
}

static int ln8411_is_enabled_vusb_otg(struct regulator_dev *rdev)
{
	struct ln8411_device *ln8411 = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(ln8411->regmap, LN8411_CTRL5, &val);
	if (ret)
		return ret;

	if (val & LN8411_OVPGATE_STAT)
		return ln8411_is_enabled_otg(ln8411);
	else
		return false;
}

static int ln8411_enable_vusb_otg(struct regulator_dev *rdev)
{
	struct ln8411_device *ln8411 = rdev_get_drvdata(rdev);
	union power_supply_propval val = {0};
	int ret;

	ret = ln8411_enable_otg(ln8411);
	if (ret)
		return ret;

	if (ln8411->role == LN8411_SECONDARY)
		return ret;

	val.intval = true;

	usleep_range(21000, 22000);

	return power_supply_set_property(ln8411->vusb, POWER_SUPPLY_PROP_ONLINE, &val);
}

static const struct regulator_ops ln8411_chg_vusb_otg_ops = {
	.disable = ln8411_disable_otg,
	.enable = ln8411_enable_vusb_otg,
	.is_enabled = ln8411_is_enabled_vusb_otg,
	.get_current_limit = ln8411_get_current_limit_otg,
	.set_current_limit = ln8411_set_current_limit_otg,
	.set_voltage = ln8411_otg_set_voltage,
	.get_voltage = ln8411_otg_get_voltage,
};

static const struct regulator_desc ln8411_vusb_otg_desc = {
	.name = "ln8411-otg-vusb",
	.ops = &ln8411_chg_vusb_otg_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.of_match = "ln8411-otg-vusb",
};

static int ln8411_usb_otg_regulator_register(struct ln8411_device *ln8411)
{
	struct regulator_config reg_cfg = { };

	reg_cfg.dev = ln8411->dev;
	reg_cfg.driver_data = ln8411;
	reg_cfg.init_data = &ln8411_otg_init_data;
	reg_cfg.regmap = ln8411->regmap;

	ln8411->otg_usb_reg = devm_regulator_register(ln8411->dev,
						      &ln8411_vusb_otg_desc,
						      &reg_cfg);
	if (IS_ERR(ln8411->otg_usb_reg)) {
		dev_err(ln8411->dev, "Failed to register VUSB OTG regulator\n");
		return PTR_ERR(ln8411->otg_usb_reg);
	}

	return 0;
}

static int ln8411_otg_regulator_register(struct ln8411_device *ln8411)
{
	int ret;

	ret = ln8411_usb_otg_regulator_register(ln8411);
	if (ret)
		return ret;

	if (ln8411->vwpc) {
		ret = ln8411_wpc_otg_regulator_register(ln8411);
		if (ret)
			return ret;
	}

	return ret;
}

static int ln8411_extcon_dev_init(struct ln8411_device *ln8411, struct device *dev)
{
	ln8411->edev = devm_extcon_dev_allocate(dev, ln8411_usb_extcon_cable);
	if (IS_ERR(ln8411->edev)) {
		dev_err(dev, "Failed to allocate extcon device\n");
		return -ENOMEM;
	}

	return devm_extcon_dev_register(dev, ln8411->edev);
}

static int ln8411_cfg_adc(struct ln8411_device *ln8411)
{
	unsigned int val;
	int ret;

	ret = regmap_write(ln8411->regmap, LN8411_ADC_FN_DISABLE1, 0);
	if (ret)
		return ret;

	ret = regmap_set_bits(ln8411->regmap, LN8411_ADC_CTRL, LN8411_ADC_DONE_MASK);
	if (ret)
		return ret;

	ret = regmap_read(ln8411->regmap, LN8411_SYS_STS, &val);
	if (ret)
		return ret;

	ret = regmap_set_bits(ln8411->regmap, LN8411_ADC_CTRL, LN8411_ADC_EN);
	if (ret)
		return ret;

	if (val & LN8411_SHUTDOWN_STS)
		msleep(375);
	else
		usleep_range(120, 130);

	return ret;
}

static int ln8411_property_is_writeable(struct power_supply *psy,
					const enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_PRESENT:
		return true;
	default:
		return false;
	}
}

static int ln8411_set_wpc_online(struct ln8411_device *ln8411,
				 const union power_supply_propval *val)
{
	int ret = 0;

	if (ln8411->role == LN8411_SECONDARY) {
		return ret;
	} else if (val->intval) {
		ret = regmap_set_bits(ln8411->regmap, LN8411_CTRL1, LN8411_WPCGATE_EN);
	} else {
		ret = regmap_clear_bits(ln8411->regmap, LN8411_CTRL1, LN8411_WPCGATE_EN);
		if (ret)
			return ret;

		ret = regmap_set_bits(ln8411->regmap, LN8411_CTRL1, LN8411_VBUS_PD_EN);
		if (ret)
			return ret;

		queue_delayed_work(system_wq, &ln8411->pulldown_res_work, msecs_to_jiffies(320));
	}

	return ret;
}

static int ln8411_set_wpc_property(struct power_supply *psy,
				   const enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct ln8411_device *ln8411 = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		return ln8411_set_vwpc_ovp(ln8411, val->intval);
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ln8411->state.iwpc_ocp_ua = val->intval;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		return ln8411_set_wpc_online(ln8411, val);
	default:
		return -EINVAL;
	}

	return 0;
}

static int ln8411_get_wpc_property(struct power_supply *psy,
				   const enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct ln8411_device *ln8411 = power_supply_get_drvdata(psy);
	unsigned int reg_code;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = ln8411->state.vwpc_ovp_uv;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = ln8411->state.iwpc_ocp_ua;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(ln8411->regmap, LN8411_CTRL5, &reg_code);
		if (ret)
			return ret;

		if (reg_code & LN8411_WPCGATE_STAT) {
			val->intval = true;
		} else {
			extcon_set_state_sync(ln8411->edev, EXTCON_CHG_WPT, false);
			val->intval = false;
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return ln8411_get_adc(ln8411, LN8411_ADC_CHAN_VWPC, &val->intval);
	case POWER_SUPPLY_PROP_HEALTH:
		return ln8411_get_wpc_health(ln8411, val);
	default:
		return -EINVAL;
	}

	return 0;
}

static int ln8411_set_usb_online(struct ln8411_device *ln8411,
				 const union power_supply_propval *val)
{
	int ret = 0;

	if (ln8411->role == LN8411_SECONDARY) {
		return ret;
	} else if (val->intval) {
		ret = regmap_set_bits(ln8411->regmap, LN8411_CTRL1, LN8411_OVPGATE_EN);
	} else {
		ret = regmap_clear_bits(ln8411->regmap, LN8411_CTRL1, LN8411_OVPGATE_EN);
		if (ret)
			return ret;

		ret = regmap_set_bits(ln8411->regmap, LN8411_CTRL1, LN8411_VBUS_PD_EN);
		if (ret)
			return ret;

		queue_delayed_work(system_wq, &ln8411->pulldown_res_work, msecs_to_jiffies(320));
	}

	return ret;
}

static int ln8411_set_usb_property(struct power_supply *psy,
				   const enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct ln8411_device *ln8411 = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		return ln8411_set_vusb_ovp(ln8411, val->intval);
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ln8411->state.iusb_ocp_ua = val->intval;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		return ln8411_set_usb_online(ln8411, val);
	default:
		return -EINVAL;
	}

	return 0;
}

static int ln8411_get_usb_property(struct power_supply *psy,
				   const enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct ln8411_device *ln8411 = power_supply_get_drvdata(psy);
	unsigned int reg_code;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = ln8411->state.vusb_ovp_uv;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = ln8411->state.iusb_ocp_ua;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(ln8411->regmap, LN8411_CTRL5, &reg_code);
		if (ret)
			return ret;

		if (reg_code & LN8411_OVPGATE_STAT) {
			val->intval = true;
		} else {
			extcon_set_state_sync(ln8411->edev, EXTCON_USB, false);
			val->intval = false;
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return ln8411_get_adc(ln8411, LN8411_ADC_CHAN_VUSB, &val->intval);
	case POWER_SUPPLY_PROP_HEALTH:
		return ln8411_get_usb_health(ln8411, val);
	default:
		return -EINVAL;
	}

	return 0;
}

static int ln8411_set_input_current_limit_from_supplier(struct power_supply *psy)
{
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_property_from_supplier(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
	if (ret)
		return ret;

	return power_supply_set_property(psy, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &val);
}

static void ln8411_usb_external_power_changed(struct power_supply *psy)
{
	struct ln8411_device *ln8411 = power_supply_get_drvdata(psy);
	int ret;

	ret = ln8411_set_input_current_limit_from_supplier(ln8411->vusb);
	if (ret)
		dev_dbg(ln8411->dev, "Failed to set USB current limit from supplier: %d!\n", ret);
}

static void ln8411_wpc_external_power_changed(struct power_supply *psy)
{
	struct ln8411_device *ln8411 = power_supply_get_drvdata(psy);
	int ret;

	ret = ln8411_set_input_current_limit_from_supplier(ln8411->vwpc);
	if (ret)
		dev_dbg(ln8411->dev, "Failed to set WPC current limit from supplier: %d!\n", ret);
}

static enum power_supply_property ln8411_input_props[] = {
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
};

static struct power_supply_desc ln8411_wpc_desc = {
	.name = "ln8411-wireless-input",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = ln8411_input_props,
	.num_properties = ARRAY_SIZE(ln8411_input_props),
	.property_is_writeable = ln8411_property_is_writeable,
	.get_property = ln8411_get_wpc_property,
	.set_property = ln8411_set_wpc_property,
	.external_power_changed = ln8411_wpc_external_power_changed,
};

static struct power_supply_desc ln8411_usb_desc = {
	.name = "ln8411-usb-input",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = ln8411_input_props,
	.num_properties = ARRAY_SIZE(ln8411_input_props),
	.property_is_writeable = ln8411_property_is_writeable,
	.get_property = ln8411_get_usb_property,
	.set_property = ln8411_set_usb_property,
	.external_power_changed = ln8411_usb_external_power_changed,
};

static int ln8411_get_battery_property(struct power_supply *psy,
				       const enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct ln8411_device *ln8411 = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return ln8411_get_adc(ln8411, LN8411_ADC_CHAN_VBAT, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return ln8411_get_adc(ln8411, LN8411_ADC_CHAN_IBAT, &val->intval);
	case POWER_SUPPLY_PROP_HEALTH:
		return ln8411_get_batt_health(ln8411, val);
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property ln8411_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_HEALTH,
};

static struct power_supply_desc ln8411_battery_desc = {
	.name = "ln8411-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = ln8411_battery_props,
	.num_properties = ARRAY_SIZE(ln8411_battery_props),
	.property_is_writeable = ln8411_property_is_writeable,
	.get_property = ln8411_get_battery_property,
};

static int ln8411_set_charger_property(struct power_supply *psy,
				       const enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct ln8411_device *ln8411 = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		return ln8411_set_vbat_ovp(ln8411, val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		return ln8411_set_ibat_ocp(ln8411, val->intval);
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return ln8411_set_ibus_ocp(ln8411, val->intval);
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		return ln8411_set_mode(ln8411, val->intval);
	case POWER_SUPPLY_PROP_STATUS:
		return ln8411_set_status(ln8411, val->intval);
	case POWER_SUPPLY_PROP_PRESENT:
		return ln8411_set_present(ln8411, val);
	default:
		return -EINVAL;
	}

	return 0;
}

static int ln8411_get_charger_property(struct power_supply *psy,
				       const enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct ln8411_device *ln8411 = power_supply_get_drvdata(psy);
	unsigned int reg_code;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(ln8411->regmap, LN8411_INT_STAT, &reg_code);
		if (ret)
			return ret;

		if (reg_code & (LN8411_VWPC_INSERT_STAT | LN8411_VUSB_INSERT_STAT)) {
			val->intval = true;
		} else {
			extcon_set_state_sync(ln8411->edev, EXTCON_USB, false);
			extcon_set_state_sync(ln8411->edev, EXTCON_CHG_WPT, false);
			val->intval = false;
		}
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = ln8411_get_vbat_ovp(ln8411);
		val->intval = ln8411->state.vbat_ovp_uv;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = ln8411_get_ibat_ocp(ln8411);
		val->intval = ln8411->state.ibat_ocp_ua;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = ln8411_get_ibus_ocp(ln8411);
		val->intval = ln8411->state.ibus_ocp_ua;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return ln8411_get_adc(ln8411, LN8411_ADC_CHAN_IBUS, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return ln8411_get_adc(ln8411, LN8411_ADC_CHAN_VBUS, &val->intval);
	case POWER_SUPPLY_PROP_TEMP:
		return ln8411_get_adc(ln8411, LN8411_ADC_CHAN_TDIE, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		return ln8411_get_mode(ln8411, &val->intval);
	case POWER_SUPPLY_PROP_STATUS:
		ret = ln8411_get_status(ln8411);
		val->intval = ln8411->state.charging_status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = true;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		return ln8411_get_charger_health(ln8411, val);
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = LN8411_MODEL_NAME;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = LN8411_MANUFACTURER;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static enum power_supply_property ln8411_2nd_charger_props[] = {
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
};

static enum power_supply_property ln8411_charger_props[] = {
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
};

static void ln8411_charger_external_power_changed(struct power_supply *psy)
{
	struct ln8411_device *ln8411 = power_supply_get_drvdata(psy);
	int ret;

	ret = ln8411_set_input_current_limit_from_supplier(ln8411->charger);
	if (ret)
		dev_dbg(ln8411->dev, "Failed to set bus current limit from supplier: %d!\n", ret);
}

static struct power_supply_desc ln8411_2nd_charger_desc = {
	.name = "ln8411-2nd-charger",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = ln8411_2nd_charger_props,
	.num_properties = ARRAY_SIZE(ln8411_2nd_charger_props),
	.property_is_writeable = ln8411_property_is_writeable,
	.get_property = ln8411_get_charger_property,
	.set_property = ln8411_set_charger_property,
	.external_power_changed = ln8411_charger_external_power_changed,
};

static struct power_supply_desc ln8411_charger_desc = {
	.name = "ln8411-charger",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = ln8411_charger_props,
	.num_properties = ARRAY_SIZE(ln8411_charger_props),
	.property_is_writeable = ln8411_property_is_writeable,
	.get_property = ln8411_get_charger_property,
	.set_property = ln8411_set_charger_property,
	.external_power_changed = ln8411_charger_external_power_changed,
};

static struct power_supply_desc ln8411_usb_charger_desc = {
	.name = "ln8411-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = ln8411_charger_props,
	.num_properties = ARRAY_SIZE(ln8411_charger_props),
	.property_is_writeable = ln8411_property_is_writeable,
	.get_property = ln8411_get_charger_property,
	.set_property = ln8411_set_charger_property,
	.external_power_changed = ln8411_charger_external_power_changed,
};

static char *ln8411_input_supplied_to_dual[] = {
	"ln8411-charger",
	"ln8411-2nd-charger",
};

static char *ln8411_input_supplied_to[] = {
	"ln8411-charger",
};

static char *ln8411_charger_supplied_to[] = {
	"ln8411-battery",
};

static int ln8411_power_supply_init_2nd(struct ln8411_device *ln8411,
					struct power_supply_config *psy_cfg,
					struct device *dev)
{
	ln8411->battery = devm_power_supply_register(dev, &ln8411_battery_desc, psy_cfg);
	if (IS_ERR(ln8411->battery)) {
		dev_err(dev, "Failed to register charger\n");
		return PTR_ERR(ln8411->battery);
	}

	psy_cfg->supplied_to = ln8411_charger_supplied_to;
	psy_cfg->num_supplicants = ARRAY_SIZE(ln8411_charger_supplied_to);

	ln8411->charger = devm_power_supply_register(dev,
						     &ln8411_2nd_charger_desc,
						     psy_cfg);
	if (IS_ERR(ln8411->charger)) {
		dev_err(dev, "Failed to register charger\n");
		return PTR_ERR(ln8411->charger);
	}

	return 0;
}

static int ln8411_power_supply_init_main(struct ln8411_device *ln8411,
					 struct power_supply_config *psy_cfg,
					 struct device *dev)
{
	ln8411->battery = devm_power_supply_register(dev, &ln8411_battery_desc, psy_cfg);
	if (IS_ERR(ln8411->battery)) {
		dev_err(dev, "Failed to register charger\n");
		return PTR_ERR(ln8411->battery);
	}

	psy_cfg->supplied_to = ln8411_charger_supplied_to;
	psy_cfg->num_supplicants = ARRAY_SIZE(ln8411_charger_supplied_to);

	ln8411->charger = devm_power_supply_register(dev, &ln8411_charger_desc, psy_cfg);
	if (IS_ERR(ln8411->charger)) {
		dev_err(dev, "Failed to register charger\n");
		return PTR_ERR(ln8411->charger);
	}

	psy_cfg->supplied_to = ln8411_input_supplied_to_dual;
	psy_cfg->num_supplicants = ARRAY_SIZE(ln8411_input_supplied_to_dual);

	ln8411->vusb = devm_power_supply_register(dev, &ln8411_usb_desc, psy_cfg);
	if (IS_ERR(ln8411->battery)) {
		dev_err(dev, "Failed to register charger\n");
		return PTR_ERR(ln8411->battery);
	}

	ln8411->vwpc = devm_power_supply_register(dev, &ln8411_wpc_desc, psy_cfg);
	if (IS_ERR(ln8411->battery)) {
		dev_err(dev, "Failed to register charger\n");
		return PTR_ERR(ln8411->battery);
	}

	return 0;
}

static int ln8411_power_supply_init_dual(struct ln8411_device *ln8411,
					 struct power_supply_config *psy_cfg,
					 struct device *dev)
{
	ln8411->battery = devm_power_supply_register(dev, &ln8411_battery_desc, psy_cfg);
	if (IS_ERR(ln8411->battery)) {
		dev_err(dev, "Failed to register charger\n");
		return PTR_ERR(ln8411->battery);
	}

	psy_cfg->supplied_to = ln8411_charger_supplied_to;
	psy_cfg->num_supplicants = ARRAY_SIZE(ln8411_charger_supplied_to);

	ln8411->charger = devm_power_supply_register(dev, &ln8411_charger_desc, psy_cfg);
	if (IS_ERR(ln8411->charger)) {
		dev_err(dev, "Failed to register charger\n");
		return PTR_ERR(ln8411->charger);
	}

	psy_cfg->supplied_to = ln8411_input_supplied_to;
	psy_cfg->num_supplicants = ARRAY_SIZE(ln8411_input_supplied_to);

	ln8411->vusb = devm_power_supply_register(dev, &ln8411_usb_desc, psy_cfg);
	if (IS_ERR(ln8411->battery)) {
		dev_err(dev, "Failed to register charger\n");
		return PTR_ERR(ln8411->battery);
	}

	ln8411->vwpc = devm_power_supply_register(dev, &ln8411_wpc_desc, psy_cfg);
	if (IS_ERR(ln8411->battery)) {
		dev_err(dev, "Failed to register charger\n");
		return PTR_ERR(ln8411->battery);
	}

	return 0;
}

static int ln8411_power_supply_init_usb(struct ln8411_device *ln8411,
					struct power_supply_config *psy_cfg,
					struct device *dev)
{
	ln8411->battery = devm_power_supply_register(dev, &ln8411_battery_desc, psy_cfg);
	if (IS_ERR(ln8411->battery)) {
		dev_err(dev, "Failed to register battery\n");
		return PTR_ERR(ln8411->battery);
	}

	psy_cfg->supplied_to = ln8411_charger_supplied_to;
	psy_cfg->num_supplicants = ARRAY_SIZE(ln8411_charger_supplied_to);

	ln8411->charger = devm_power_supply_register(dev,
						     &ln8411_usb_charger_desc,
						     psy_cfg);
	if (IS_ERR(ln8411->charger)) {
		dev_err(dev, "Failed to register charger\n");
		return PTR_ERR(ln8411->charger);
	}

	return 0;
}

static int ln8411_power_supply_init(struct ln8411_device *ln8411,
				    struct power_supply_config *psy_cfg,
				    struct device *dev)
{
	psy_cfg->drv_data = ln8411;
	if (dev->of_node)
		psy_cfg->of_node = dev->of_node;

	switch (ln8411->role) {
	case LN8411_USB:
		return ln8411_power_supply_init_usb(ln8411, psy_cfg, dev);
	case LN8411_DUAL:
		return ln8411_power_supply_init_dual(ln8411, psy_cfg, dev);
	case LN8411_MAIN:
		return ln8411_power_supply_init_main(ln8411, psy_cfg, dev);
	case LN8411_SECONDARY:
		return ln8411_power_supply_init_2nd(ln8411, psy_cfg, dev);
	default:
		return -EOPNOTSUPP;
	}
}

static int ln8411_cfg_sync(struct ln8411_device *ln8411)
{
	unsigned int reg_code = LN8411_SYNC_FUNCTION_EN;
	int ret;

	ret = regmap_clear_bits(ln8411->regmap, LN8411_CTRL4, LN8411_TSBAT_EN_PIN);
	if (ret)
		return ret;

	if (ln8411->role == LN8411_MAIN)
		reg_code |= LN8411_SYNC_MASTER_EN;

	return regmap_update_bits(ln8411->regmap, LN8411_CTRL4, LN8411_SYNC_MASK, reg_code);
}

static int ln8411_apply_conv_dt(struct ln8411_device *ln8411, struct ln8411_init_data *init_data)
{
	unsigned int reg_code = 0;
	int ret;

	ret = ln8411_cfg_pmid2out_ovp(ln8411);
	if (ret)
		return ret;

	ret = ln8411_cfg_pmid2out_uvp(ln8411);
	if (ret)
		return ret;

	ret = ln8411_set_fsw(ln8411);
	if (ret)
		return ret;

	ret = ln8411_set_wd_timeout(ln8411);
	if (ret)
		return ret;

	if (init_data->freq_shift)
		ret = regmap_set_bits(ln8411->regmap, LN8411_CTRL2, LN8411_FREQ_SHIFT);
	else
		ret = regmap_clear_bits(ln8411->regmap, LN8411_CTRL2, LN8411_FREQ_SHIFT);
	if (ret)
		return ret;

	if (init_data->ovpfetdr_v_cfg) {
		ret = regmap_set_bits(ln8411->regmap,
				      LN8411_OVPGATE_CTRL_0, LN8411_OVPFETDR_V_CFG);
		if (ret)
			return ret;
	}

	if (init_data->vbus_ovp_set)
		reg_code |= LN8411_VBUS_OVP_SET;
	if (init_data->set_ibat_sns_res)
		reg_code |= LN8411_SET_IBAT_SNS_RES;

	return regmap_update_bits(ln8411->regmap, LN8411_CTRL4, ~(LN8411_MODE_MASK), reg_code);
}

static int ln8411_apply_ibus_dt(struct ln8411_device *ln8411, struct ln8411_init_data *init_data)
{
	int ret;

	ret = ln8411_cfg_ibus_ocp(ln8411);
	if (ret)
		return ret;

	ret = ln8411_set_ibus_ocp(ln8411, init_data->ibus_ocp_ua);
	if (ret)
		return ret;

	return ln8411_cfg_ibus_ucp(ln8411);
}

static int ln8411_apply_vwpc_dt(struct ln8411_device *ln8411, struct ln8411_init_data *init_data)
{
	int ret;

	ret = ln8411_cfg_vwpc_ovp(ln8411);
	if (ret)
		return ret;

	return ln8411_set_vwpc_ovp(ln8411, init_data->vwpc_ovp_uv);
}

static int ln8411_apply_vusb_dt(struct ln8411_device *ln8411, struct ln8411_init_data *init_data)
{
	int ret;

	ret = ln8411_cfg_vusb_ovp(ln8411);
	if (ret)
		return ret;

	return ln8411_set_vusb_ovp(ln8411, init_data->vusb_ovp_uv);
}

static int ln8411_apply_ibat_dt(struct ln8411_device *ln8411, struct ln8411_init_data *init_data)
{
	int ret;

	ret = ln8411_cfg_ibat_ocp(ln8411);
	if (ret)
		return ret;

	return ln8411_set_ibat_ocp(ln8411, init_data->ibat_ocp_ua);
}

static int ln8411_apply_vbat_dt(struct ln8411_device *ln8411, struct ln8411_init_data *init_data)
{
	int ret;

	ret = ln8411_cfg_vbat_ovp(ln8411);
	if (ret)
		return ret;

	return ln8411_set_vbat_ovp(ln8411, init_data->vbat_ovp_uv);
}

static int ln8411_hw_init(struct ln8411_device *ln8411)
{
	struct ln8411_init_data *init_data = &ln8411->init_data;
	int ret;

	ret = ln8411_cfg_adc(ln8411);
	if (ret) {
		dev_err(ln8411->dev, "Failed to configure ADC: %d\n", ret);
		return ret;
	}

	ret = ln8411_apply_vbat_dt(ln8411, init_data);
	if (ret)
		return ret;

	ret = ln8411_apply_ibat_dt(ln8411, init_data);
	if (ret)
		return ret;

	ret = ln8411_apply_vusb_dt(ln8411, init_data);
	if (ret)
		return ret;

	ret = ln8411_apply_vwpc_dt(ln8411, init_data);
	if (ret)
		return ret;

	ret = ln8411_apply_ibus_dt(ln8411, init_data);
	if (ret)
		return ret;

	ret = ln8411_apply_conv_dt(ln8411, init_data);
	if (ret)
		return ret;

	if (ln8411->role > LN8411_DUAL)
		return ln8411_cfg_sync(ln8411);

	return ret;
}

static int ln8411_parse_dt_conv(struct device *dev, struct ln8411_init_data *init_data)
{
	unsigned int val;
	int ret;

	ret = device_property_read_u32(dev, "cirrus,switching-frequency-hz", &val);
	if (ret < 0)
		init_data->fsw_hz = LN8411_FSW_DFLT_HZ;
	else
		init_data->fsw_hz = val;

	init_data->freq_shift = device_property_read_bool(dev, "cirrus,freq-shift");

	ret = device_property_read_u32(dev, "cirrus,wd-timeout-ms", &val);
	if (ret < 0)
		init_data->wd_timeout_ms = LN8411_WD_TIMEOUT_DIS;
	else
		init_data->wd_timeout_ms = val;

	init_data->sync_func_en = device_property_read_bool(dev, "cirrus,sync-function-en");
	init_data->sync_main_en = device_property_read_bool(dev, "cirrus,sync-main-en");
	init_data->vbus_ovp_set = device_property_read_bool(dev, "cirrus,vbus-ovp-set");
	init_data->set_ibat_sns_res = device_property_read_bool(dev, "cirrus,set-ibat-sns-res");
	init_data->ovpfetdr_v_cfg = device_property_read_bool(dev, "cirrus,use-si-ovp-fets");

	return 0;
}

static int ln8411_parse_dt_pmid_uvp(struct device *dev, struct ln8411_init_data *init_data)
{
	unsigned int val;
	int ret;

	init_data->pmid2out_uvp_dis = device_property_read_bool(dev, "cirrus,pmid2out-uvp-dis");
	if (init_data->pmid2out_uvp_dis)
		return 0;

	init_data->pmid2out_uvp_mask = device_property_read_bool(dev, "cirrus,pmid2out-uvp-mask");

	ret = device_property_read_u32(dev, "cirrus,pmid2out-uvp", &val);
	if (ret < 0)
		init_data->pmid2out_uvp = LN8411_PMID2OUT_UVP_DFLT;
	else
		init_data->pmid2out_uvp = val;

	return 0;
}

static int ln8411_parse_dt_pmid_ovp(struct device *dev, struct ln8411_init_data *init_data)
{
	unsigned int val;
	int ret;

	init_data->pmid2out_ovp_dis = device_property_read_bool(dev, "cirrus,pmid2out-ovp-dis");
	if (init_data->pmid2out_ovp_dis)
		return 0;

	init_data->pmid2out_ovp_mask = device_property_read_bool(dev, "cirrus,pmid2out-ovp-mask");

	ret = device_property_read_u32(dev, "cirrus,pmid2out-ovp", &val);
	if (ret < 0)
		init_data->pmid2out_ovp = LN8411_PMID2OUT_OVP_DFLT;
	else
		init_data->pmid2out_ovp = val;

	return 0;
}

static int ln8411_parse_dt_ibus(struct device *dev, struct ln8411_init_data *init_data)
{
	unsigned int val;
	int ret;

	init_data->ibus_ucp_dis = device_property_read_bool(dev, "cirrus,ibus-ucp-dis");
	init_data->ibus_ucp_rise_mask = device_property_read_bool(dev, "cirrus,ibus-ucp-rise-mask");
	init_data->ibus_ucp_fall_mask = device_property_read_bool(dev, "cirrus,ibus-ucp-fall-mask");

	init_data->ibus_ocp_dis = device_property_read_bool(dev, "cirrus,ibus-ocp-dis");
	if (init_data->ibus_ocp_dis)
		return 0;

	init_data->ibus_ocp_mask = device_property_read_bool(dev, "cirrus,ibus-ocp-mask");

	ret = device_property_read_u32(dev, "cirrus,ibus-ocp-microamp", &val);
	if (ret < 0)
		init_data->ibus_ocp_ua = LN8411_IBUS_OCP_DLFT_UA;
	else
		init_data->ibus_ocp_ua = val;

	return 0;
}

static int ln8411_parse_dt_vwpc(struct device *dev, struct ln8411_init_data *init_data)
{
	unsigned int val;
	int ret;

	init_data->wpcgate_on_dg_set = device_property_read_bool(dev, "cirrus,wpcgate-on-dg-set");
	init_data->vwpc_ovp_mask = device_property_read_bool(dev, "cirrus,wpc-ovp-mask");

	ret = device_property_read_u32(dev, "cirrus,vwpc-ovp-microvolt", &val);
	if (ret < 0)
		init_data->vwpc_ovp_uv = LN8411_VWPC_OVP_DFLT_UV;
	else
		init_data->vwpc_ovp_uv = val;

	return 0;
}

static int ln8411_parse_dt_vusb(struct device *dev, struct ln8411_init_data *init_data)
{
	unsigned int val;
	int ret;

	init_data->ovpgate_on_dg_set = device_property_read_bool(dev, "cirrus,ovpgate-on-dg-set");
	init_data->vusb_ovp_mask = device_property_read_bool(dev, "cirrus,vusb-ovp-mask");

	ret = device_property_read_u32(dev, "cirrus,vusb-ovp-microvolt", &val);
	if (ret < 0)
		init_data->vusb_ovp_uv = LN8411_VUSB_OVP_DFLT_UV;
	else
		init_data->vusb_ovp_uv = val;

	return 0;
}

static void ln8411_parse_battery(struct device *dev, struct ln8411_device *ln8411)
{
	struct power_supply_battery_info *info;

	if (dev->of_node && !power_supply_get_battery_info(ln8411->charger, &info)) {
		ln8411->init_data.vbat_ovp_uv = info->overvoltage_limit_uv;
		ln8411->init_data.ibat_ocp_ua = info->constant_charge_current_max_ua;
		dev_dbg(dev, "Found battery info\n");
	} else {
		dev_dbg(dev, "Battery info not found!\n");
	}
}

static int ln8411_parse_dt_ibat(struct device *dev, struct ln8411_init_data *init_data)
{
	unsigned int val;
	int ret;

	init_data->ibat_ocp_dis = device_property_read_bool(dev, "cirrus,ibat-ocp-dis");
	if (init_data->ibat_ocp_dis)
		return 0;

	init_data->ibat_ocp_mask = device_property_read_bool(dev, "cirrus,ibat-ocp-mask");

	ret = device_property_read_u32(dev, "cirrus,ibat-ocp-microamp", &val);
	if (ret < 0)
		init_data->ibat_ocp_ua = LN8411_IBAT_OCP_DFLT_UA;
	else
		init_data->ibat_ocp_ua = val;

	return 0;
}

static int ln8411_parse_dt_vbat(struct device *dev, struct ln8411_init_data *init_data)
{
	unsigned int val;
	int ret;

	init_data->vbat_ovp_dis = device_property_read_bool(dev, "cirrus,vbat-ovp-dis");
	if (init_data->vbat_ovp_dis)
		return 0;

	init_data->vbat_ovp_dg_set = device_property_read_bool(dev, "cirrus,vbat-ovp-dg-set");
	init_data->vbat_ovp_mask = device_property_read_bool(dev, "cirrus,vbat-ovp-mask");

	ret = device_property_read_u32(dev, "cirrus,vbat-ovp-microvolt", &val);
	if (ret < 0)
		init_data->vbat_ovp_uv = LN8411_VBAT_OVP_DFLT_UV;
	else
		init_data->vbat_ovp_uv = val;

	return 0;
}

static int ln8411_parse_dt(struct device *dev, struct ln8411_device *ln8411)
{
	struct ln8411_init_data *init_data = &ln8411->init_data;
	int ret;

	ret = ln8411_parse_dt_vbat(dev, init_data);
	if (ret)
		return ret;

	ret = ln8411_parse_dt_ibat(dev, init_data);
	if (ret)
		return ret;

	ln8411_parse_battery(dev, ln8411);

	ret = ln8411_parse_dt_vusb(dev, init_data);
	if (ret)
		return ret;

	ret = ln8411_parse_dt_vwpc(dev, init_data);
	if (ret)
		return ret;

	ret = ln8411_parse_dt_ibus(dev, init_data);
	if (ret)
		return ret;

	ret = ln8411_parse_dt_pmid_ovp(dev, init_data);
	if (ret)
		return ret;

	ret = ln8411_parse_dt_pmid_uvp(dev, init_data);
	if (ret)
		return ret;

	return ln8411_parse_dt_conv(dev, init_data);
}

static int ln8411_soft_reset(struct ln8411_device *ln8411)
{
	const union power_supply_propval val = { .intval = POWER_SUPPLY_STATUS_NOT_CHARGING };
	int ret;

	/* Converter must be in standby mode before soft resetting to prevent damage*/
	if (ln8411->state.charging_status != POWER_SUPPLY_STATUS_NOT_CHARGING) {
		ret = power_supply_set_property(ln8411->charger, POWER_SUPPLY_PROP_STATUS, &val);
		if (ret)
			return ret;
	}

	ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_EN_RESET);
	if (ret)
		return ret;

	ret = regmap_set_bits(ln8411->regmap, LN8411_TEST_MODE_CTRL, LN8411_SOFT_RESET_REQ);
	if (ret)
		return ret;

	msleep(250);

	return ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_LOCK);
}

static int ln8411_gpio_cfg(struct ln8411_device *ln8411)
{
	int ret;

	ln8411->reset_gpio = devm_gpiod_get_optional(ln8411->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ln8411->reset_gpio)) {
		return dev_err_probe(ln8411->dev, PTR_ERR(ln8411->reset_gpio),
				     "Failed to get reset GPIO\n");
	}

	/* Enable GPIO cannot be used in sync mode */
	if (ln8411->role >= LN8411_MAIN)
		return 0;

	ret = regmap_set_bits(ln8411->regmap, LN8411_CTRL4, LN8411_TSBAT_EN_PIN);
	if (ret)
		return ret;

	ln8411->en_gpio = devm_gpiod_get_optional(ln8411->dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ln8411->en_gpio)) {
		return dev_err_probe(ln8411->dev, PTR_ERR(ln8411->en_gpio),
				     "Failed to get charge enable GPIO\n");
	}

	return 0;
}

static int ln8411_is_supported(struct ln8411_device *ln8411)
{
	unsigned int val;
	u16 dev_rev_id;
	int ret;

	ret = regmap_read(ln8411->regmap, LN8411_DEVICE_ID, &val);
	if (ret)
		return ret;

	dev_rev_id = val << LN8411_REG_BITS;

	ret = regmap_read(ln8411->regmap, LN8411_BC_STS_C, &val);
	if (ret)
		return ret;

	dev_rev_id |= (val & LN8411_CHIP_REV_MASK);

	switch (dev_rev_id) {
	case LN8411_A1_DEV_REV_ID:
		dev_info(ln8411->dev, "LN8411 A1 found: 0x%x\n", dev_rev_id);
		break;
	case LN8411_B0_DEV_REV_ID:
		dev_info(ln8411->dev, "LN8411 B0 found: 0x%x\n", dev_rev_id);
		break;
	default:
		dev_err(ln8411->dev, "Unsupported device found: 0x%x\n", dev_rev_id);
		return -EINVAL;
	}

	ln8411->rev = dev_rev_id;

	return ret;
}

static bool ln8411_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case LN8411_VBAT_OVP...LN8411_PMID2OUT_UVP:
	case LN8411_CTRL1...LN8411_CTRL4:
	case LN8411_ADC_CTRL...LN8411_ADC_FN_DISABLE1:
	case LN8411_ADC_CFG_2:
		return false;
	default:
		return true;
	}
}

static struct reg_default ln8411_reg_defs[] = {
	{LN8411_VBAT_OVP, 0x28},
	{LN8411_IBAT_OCP, 0xaf},
	{LN8411_VUSB_OVP, 0xcb},
	{LN8411_VWPC_OVP, 0xcb},
	{LN8411_IBUS_OCP, 0x28},
	{LN8411_IBUS_UCP, 0x1a},
	{LN8411_PMID2OUT_OVP, 0x14},
	{LN8411_PMID2OUT_UVP, 0x11},
	{LN8411_CTRL1, 0x18},
	{LN8411_CTRL2, 0xa0},
	{LN8411_CTRL3, 0x0},
	{LN8411_CTRL4, 0x0},
	{LN8411_INT_MASK, 0x3f},
	{LN8411_FLT_MASK, 0xff},
	{LN8411_ADC_CTRL, 0x08},
	{LN8411_ADC_FN_DISABLE1, 0x6},
	{LN8411_ADC_CFG_2, 0xc4},
};

static const struct regmap_config ln8411_regmap_config = {
	.reg_bits = LN8411_REG_BITS,
	.val_bits = LN8411_VAL_BITS,

	.max_register = LN8411_FAULT3_STS,
	.reg_defaults = ln8411_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(ln8411_reg_defs),
	.cache_type = REGCACHE_FLAT,
	.volatile_reg = ln8411_is_volatile_reg,
};

static int ln8411_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct power_supply_config psy_cfg = { };
	struct device *dev = &client->dev;
	struct ln8411_device *ln8411;
	int ret;

	ln8411 = devm_kzalloc(dev, sizeof(*ln8411), GFP_KERNEL);
	if (!ln8411)
		return -ENOMEM;

	ln8411->role = id->driver_data;
	ln8411->client = client;
	ln8411->dev = dev;

	mutex_init(&ln8411->lock);

	INIT_DELAYED_WORK(&ln8411->pulldown_res_work, ln8411_pulldown_res_work);
	INIT_DELAYED_WORK(&ln8411->charge_en_work, ln8411_charge_en_work);

	i2c_set_clientdata(client, ln8411);

	ln8411->regmap = devm_regmap_init_i2c(client, &ln8411_regmap_config);
	if (IS_ERR(ln8411->regmap)) {
		dev_err(dev, "Failed to allocate register map!\n");
		return PTR_ERR(ln8411->regmap);
	}

	ret = ln8411_is_supported(ln8411);
	if (ret)
		return ret;

	/* Only revision B0 and beyond support GPIOs */
	if (ln8411->rev >= LN8411_B0_DEV_REV_ID) {
		ret = ln8411_gpio_cfg(ln8411);
		if (ret)
			return ret;
	}

	ret = ln8411_power_supply_init(ln8411, &psy_cfg, dev);
	if (ret) {
		dev_err(dev, "Failed to register power supplies: %d\n", ret);
		return ret;
	}

	ret = ln8411_extcon_dev_init(ln8411, dev);
	if (ret < 0) {
		dev_err(dev, "Failed to register extcon device: %d\n", ret);
		return ret;
	}

	ret = ln8411_otg_regulator_register(ln8411);
	if (ret) {
		dev_err(dev, "Failed to register OTG regulator(s): %d\n", ret);
		return ret;
	}

	ret = ln8411_parse_dt(dev, ln8411);
	if (ret) {
		dev_err(dev, "Failed to read devicetree properties: %d\n", ret);
		return ret;
	}

	ret = ln8411_hw_init(ln8411);
	if (ret) {
		dev_err(dev, "Failed to apply devicetree properties: %d\n", ret);
		return ret;
	}

	ln8411->irq = client->irq;

	if (ln8411->irq) {
		ret = ln8411_regmap_irq_init(ln8411, dev);
		if (ret)
			return ret;
	}

	return ret;
}

static void ln8411_remove(struct i2c_client *client)
{
	struct ln8411_device *ln8411 = i2c_get_clientdata(client);
	int ret;

	ret = ln8411_soft_reset(ln8411);
	if (ret)
		dev_err(ln8411->dev, "Failed to reset: %d\n", ret);

	mutex_destroy(&ln8411->lock);
}

static const struct i2c_device_id ln8411_i2c_ids[] = {
	{ "ln8411-usb", LN8411_USB },
	{ "ln8411-dual", LN8411_DUAL },
	{ "ln8411-main", LN8411_MAIN },
	{ "ln8411-secondary", LN8411_SECONDARY },
	{},
};
MODULE_DEVICE_TABLE(i2c, ln8411_i2c_ids);

static const struct of_device_id ln8411_of_match[] = {
	{ .compatible = "cirrus,ln8411-usb",
	  .data = (void *)LN8411_USB,
	},
	{ .compatible = "cirrus,ln8411-dual",
	  .data = (void *)LN8411_DUAL,
	},
	{ .compatible = "cirrus,ln8411-main",
	  .data = (void *)LN8411_MAIN,
	},
	{ .compatible = "cirrus,ln8411-secondary",
	  .data = (void *)LN8411_SECONDARY,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, ln8411_of_match);

static struct i2c_driver ln8411_driver = {
	.driver = {
		.name = "ln8411",
		.of_match_table = ln8411_of_match,
	},
	.id_table = ln8411_i2c_ids,
	.probe = ln8411_probe,
	.remove = ln8411_remove,
};
module_i2c_driver(ln8411_driver);

MODULE_AUTHOR("Ricardo Rivera-Matos <rriveram@opensource.cirrus.com>");
MODULE_DESCRIPTION("ln8411 charger driver");
MODULE_LICENSE("GPL v2");
