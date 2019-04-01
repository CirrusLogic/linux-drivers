/*
 * Interrupt support for Cirrus Logic Tacna codecs
 *
 * Copyright 2017-2018 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/irqchip/irq-tacna.h>
#include <linux/irqchip/irq-tacna-pdata.h>
#include <linux/mfd/tacna/core.h>
#include <linux/mfd/tacna/pdata.h>
#include <linux/mfd/tacna/registers.h>

#define TACNA_AOD_VIRQ_INDEX  0
#define TACNA_MAIN_VIRQ_INDEX 1

struct tacna_irq_priv {
	struct device *dev;
	int irq;
	struct irq_domain *virq;
	struct regmap_irq_chip_data *aod_irq_chip;
	struct regmap_irq_chip_data *irq_chip;
	struct tacna *tacna;
};

static const struct regmap_irq tacna_main_irqs[TACNA_NUM_MAIN_IRQ] = {
	[TACNA_IRQ_OUT1R_SC] = {
		.reg_offset = 0x00, .mask = TACNA_OUT1R_SC_EINT1_MASK,
	},
	[TACNA_IRQ_OUT1L_SC] = {
		.reg_offset = 0x00, .mask = TACNA_OUT1L_SC_EINT1_MASK,
	},
	[TACNA_IRQ_OUT2R_SC] = {
		.reg_offset = 0x00, .mask = TACNA_OUT2R_SC_EINT1_MASK,
	},
	[TACNA_IRQ_OUT2L_SC] = {
		.reg_offset = 0x00, .mask = TACNA_OUT2L_SC_EINT1_MASK,
	},
	[TACNA_IRQ_DSP1_IRQ0] = {
		.reg_offset = 0x20, .mask = TACNA_DSP1_IRQ0_EINT1_MASK,
	},
	[TACNA_IRQ_DSP2_IRQ0] = {
		.reg_offset = 0x20, .mask = TACNA_DSP2_IRQ0_EINT1_MASK,
	},
	[TACNA_IRQ_DSP1_IRQ1] = {
		.reg_offset = 0x20, .mask = TACNA_DSP1_IRQ1_EINT1_MASK,
	},
	[TACNA_IRQ_DSP2_IRQ1] = {
		.reg_offset = 0x20, .mask = TACNA_DSP2_IRQ1_EINT1_MASK,
	},
	[TACNA_IRQ_DSP1_IRQ2] = {
		.reg_offset = 0x20, .mask = TACNA_DSP1_IRQ2_EINT1_MASK,
	},
	[TACNA_IRQ_DSP2_IRQ2] = {
		.reg_offset = 0x20, .mask = TACNA_DSP2_IRQ2_EINT1_MASK,
	},
	[TACNA_IRQ_DSP1_IRQ3] = {
		.reg_offset = 0x20, .mask = TACNA_DSP1_IRQ3_EINT1_MASK,
	},
	[TACNA_IRQ_DSP2_IRQ3] = {
		.reg_offset = 0x20, .mask = TACNA_DSP2_IRQ3_EINT1_MASK,
	},
	[TACNA_IRQ_US1_ACT_DET_RISE] = {
		.reg_offset = 0x10, .mask = TACNA_US1_ACT_DET_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_US1_ACT_DET_FALL] = {
		.reg_offset = 0x10, .mask = TACNA_US1_ACT_DET_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_US2_ACT_DET_RISE] = {
		.reg_offset = 0x10, .mask = TACNA_US2_ACT_DET_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_US2_ACT_DET_FALL] = {
		.reg_offset = 0x10, .mask = TACNA_US2_ACT_DET_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_US3_ACT_DET_RISE] = {
		.reg_offset = 0x10, .mask = TACNA_US3_ACT_DET_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_US3_ACT_DET_FALL] = {
		.reg_offset = 0x10, .mask = TACNA_US3_ACT_DET_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_INPUTS_SIG_DET_AO_RISE] = {
		.reg_offset = 0x10, .mask = TACNA_INPUTS_SIG_DET_AO_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_INPUTS_SIG_DET_AO_FALL] = {
		.reg_offset = 0x10, .mask = TACNA_INPUTS_SIG_DET_AO_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_INPUTS_SIG_DET_RISE] = {
		.reg_offset = 0x10, .mask = TACNA_INPUTS_SIG_DET_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_INPUTS_SIG_DET_FALL] = {
		.reg_offset = 0x10, .mask = TACNA_INPUTS_SIG_DET_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_HPDET] = {
		.reg_offset = 0x10, .mask = TACNA_HPDET_EINT1_MASK,
	},
	[TACNA_IRQ_MICDET1] = {
		.reg_offset = 0x10, .mask = TACNA_MICDET1_EINT1_MASK,
	},
	[TACNA_IRQ_MICDET2] = {
		.reg_offset = 0x10, .mask = TACNA_MICDET2_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO1_RISE] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO1_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO1_FALL] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO1_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO2_RISE] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO2_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO2_FALL] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO2_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO3_RISE] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO3_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO3_FALL] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO3_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO4_RISE] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO4_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO4_FALL] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO4_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO5_RISE] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO5_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO5_FALL] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO5_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO6_RISE] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO6_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO6_FALL] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO6_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO7_RISE] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO7_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO7_FALL] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO7_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO8_RISE] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO8_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_GPIO8_FALL] = {
		.reg_offset = 0x28, .mask = TACNA_GPIO8_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_OUT1R_ENABLE_DONE] = {
		.reg_offset = 0x08, .mask = TACNA_OUT1R_ENABLE_DONE_EINT1_MASK,
	},
	[TACNA_IRQ_OUT1L_ENABLE_DONE] = {
		.reg_offset = 0x08, .mask = TACNA_OUT1L_ENABLE_DONE_EINT1_MASK,
	},
	[TACNA_IRQ_OUT2R_ENABLE_DONE] = {
		.reg_offset = 0x08, .mask = TACNA_OUT2R_ENABLE_DONE_EINT1_MASK,
	},
	[TACNA_IRQ_OUT2L_ENABLE_DONE] = {
		.reg_offset = 0x08, .mask = TACNA_OUT2L_ENABLE_DONE_EINT1_MASK,
	},
	[TACNA_IRQ_OUT1R_DISABLE_DONE] = {
		.reg_offset = 0x08, .mask = TACNA_OUT1R_DISABLE_DONE_EINT1_MASK,
	},
	[TACNA_IRQ_OUT1L_DISABLE_DONE] = {
		.reg_offset = 0x08, .mask = TACNA_OUT1L_DISABLE_DONE_EINT1_MASK,
	},
	[TACNA_IRQ_OUT2R_DISABLE_DONE] = {
		.reg_offset = 0x08, .mask = TACNA_OUT2R_DISABLE_DONE_EINT1_MASK,
	},
	[TACNA_IRQ_OUT2L_DISABLE_DONE] = {
		.reg_offset = 0x08, .mask = TACNA_OUT2L_DISABLE_DONE_EINT1_MASK,
	},
	[TACNA_IRQ_OUTH_ENABLE_DONE] = {
		.reg_offset = 0x08, .mask = TACNA_OUTH_ENABLE_DONE_EINT1_MASK,
	},
	[TACNA_IRQ_OUTH_DISABLE_DONE] = {
		.reg_offset = 0x08, .mask = TACNA_OUTH_DISABLE_DONE_EINT1_MASK,
	},
	[TACNA_IRQ_DRC1_SIG_DET_RISE] = {
		.reg_offset = 0x10, .mask = TACNA_DRC1_SIG_DET_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_DRC1_SIG_DET_FALL] = {
		.reg_offset = 0x10, .mask = TACNA_DRC1_SIG_DET_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_DRC2_SIG_DET_RISE] = {
		.reg_offset = 0x10, .mask = TACNA_DRC2_SIG_DET_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_DRC2_SIG_DET_FALL] = {
		.reg_offset = 0x10, .mask = TACNA_DRC2_SIG_DET_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_JD2_HDRV_RISE] = {
		.reg_offset = 0x0c, .mask = TACNA_JD2_HDRV_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_JD2_HDRV_FALL] = {
		.reg_offset = 0x0c, .mask = TACNA_JD2_HDRV_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_FLL1_LOCK_RISE] = {
		.reg_offset = 0x14, .mask = TACNA_FLL1_LOCK_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_FLL1_LOCK_FALL] = {
		.reg_offset = 0x14, .mask = TACNA_FLL1_LOCK_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_FLL2_LOCK_RISE] = {
		.reg_offset = 0x14, .mask = TACNA_FLL2_LOCK_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_FLL2_LOCK_FALL] = {
		.reg_offset = 0x14, .mask = TACNA_FLL2_LOCK_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_FLL3_LOCK_RISE] = {
		.reg_offset = 0x14, .mask = TACNA_FLL3_LOCK_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_FLL3_LOCK_FALL] = {
		.reg_offset = 0x14, .mask = TACNA_FLL3_LOCK_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_FLL1_REF_LOST] = {
		.reg_offset = 0x14, .mask = TACNA_FLL1_REF_LOST_EINT1_MASK,
	},
	[TACNA_IRQ_FLL2_REF_LOST] = {
		.reg_offset = 0x14, .mask = TACNA_FLL2_REF_LOST_EINT1_MASK,
	},
	[TACNA_IRQ_FLL3_REF_LOST] = {
		.reg_offset = 0x14, .mask = TACNA_FLL3_REF_LOST_EINT1_MASK,
	},
	[TACNA_IRQ_FLL1AO_RELOCK_FAIL] = {
		.reg_offset = 0x0, .mask = TACNA_FLL1AO_RELOCK_FAIL_EINT1_MASK,
	},
	[TACNA_IRQ_SYSCLK_FAIL] = {
		.reg_offset = 0x00, .mask = TACNA_SYSCLK_FAIL_EINT1_MASK,
	},
	[TACNA_IRQ_CTRLIF_ERR] = {
		.reg_offset = 0x00, .mask = TACNA_CTRLIF_ERR_EINT1_MASK,
	},
	[TACNA_IRQ_SYSCLK_ERR] = {
		.reg_offset = 0x00, .mask = TACNA_SYSCLK_ERR_EINT1_MASK,
	},
	[TACNA_IRQ_ASYNCCLK_ERR] = {
		.reg_offset = 0x00, .mask = TACNA_ASYNCCLK_ERR_EINT1_MASK,
	},
	[TACNA_IRQ_DSPCLK_ERR] = {
		.reg_offset = 0x00, .mask = TACNA_DSPCLK_ERR_EINT1_MASK,
	},
	[TACNA_IRQ_AOCLK_ERR] = {
		.reg_offset = 0x00, .mask = TACNA_AOCLK_ERR_EINT1,
	},
	[TACNA_IRQ_DSP1_NMI] = {
		.reg_offset = 0x18, .mask = TACNA_DSP1_NMI_ERR_EINT1_MASK,
	},
	[TACNA_IRQ_DSP1_WDT_EXPIRE] = {
		.reg_offset = 0x18, .mask = TACNA_DSP1_WDT_EXPIRE_EINT1,
	},
	[TACNA_IRQ_DSP1_MPU_ERR] = {
		.reg_offset = 0x18, .mask = TACNA_DSP1_MPU_ERR_EINT1,
	},
	[TACNA_IRQ_DSP2_NMI] = {
		.reg_offset = 0x18, .mask = TACNA_DSP2_NMI_ERR_EINT1_MASK,
	},
	[TACNA_IRQ_DSP2_WDT_EXPIRE] = {
		.reg_offset = 0x18, .mask = TACNA_DSP2_WDT_EXPIRE_EINT1,
	},
	[TACNA_IRQ_DSP2_MPU_ERR] = {
		.reg_offset = 0x18, .mask = TACNA_DSP2_MPU_ERR_EINT1,
	},
	[TACNA_IRQ_MCU_HWERR] = {
		.reg_offset = 0x20, .mask = TACNA_MCU_HWERR_IRQ_OUT_EINT1_MASK,
	},
	[TACNA_IRQ_BOOT_DONE] = {
		.reg_offset = 0x04, .mask = TACNA_BOOT_DONE_EINT1_MASK,
	},
	[TACNA_IRQ_ASRC1_IN1_LOCK_RISE] = {
		.reg_offset = 0x24, .mask = TACNA_ASRC1_IN1_LOCK_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_ASRC1_IN1_LOCK_FALL] = {
		.reg_offset = 0x24, .mask = TACNA_ASRC1_IN1_LOCK_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_ASRC1_IN2_LOCK_RISE] = {
		.reg_offset = 0x24, .mask = TACNA_ASRC1_IN2_LOCK_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_ASRC1_IN2_LOCK_FALL] = {
		.reg_offset = 0x24, .mask = TACNA_ASRC1_IN2_LOCK_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_ASRC2_IN1_LOCK_RISE] = {
		.reg_offset = 0x24, .mask = TACNA_ASRC2_IN1_LOCK_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_ASRC2_IN1_LOCK_FALL] = {
		.reg_offset = 0x24, .mask = TACNA_ASRC2_IN1_LOCK_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_ASRC2_IN2_LOCK_RISE] = {
		.reg_offset = 0x24, .mask = TACNA_ASRC2_IN2_LOCK_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_ASRC2_IN2_LOCK_FALL] = {
		.reg_offset = 0x24, .mask = TACNA_ASRC2_IN2_LOCK_FALL_EINT1_MASK,
	},
	[TACNA_IRQ_SECURE_MODE_RISE] = {
		.reg_offset = 0x04, .mask = TACNA_SECURE_MODE_RISE_EINT1_MASK,
	},
	[TACNA_IRQ_SECURE_MODE_FALL] = {
		.reg_offset = 0x04, .mask = TACNA_SECURE_MODE_FALL_EINT1_MASK,
	},
};

/*
 * AOD IRQ numbers are offset by TACNA_NUM_MAIN_IRQ. So we don't waste memory
 * on unused table entries, they are indexed back to zero in this table
 */
#define TACNA_AOD_IRQ_OFFSET(__i)	((__i) - TACNA_NUM_MAIN_IRQ)

static const struct regmap_irq tacna_aod_irqs[TACNA_NUM_AOD_IRQ] = {
	/* AOD IRQs */
	[TACNA_AOD_IRQ_OFFSET(TACNA_IRQ_AOD_JD1_RISE)] = {
		.reg_offset =  0, .mask = TACNA_JD1_RISE_EINT1_MASK,
	},
	[TACNA_AOD_IRQ_OFFSET(TACNA_IRQ_AOD_JD1_FALL)] = {
		.reg_offset =  0, .mask = TACNA_JD1_FALL_EINT1_MASK,
	},
	[TACNA_AOD_IRQ_OFFSET(TACNA_IRQ_AOD_JD2_RISE)] = {
		.reg_offset =  0, .mask = TACNA_JD2_RISE_EINT1_MASK,
	},
	[TACNA_AOD_IRQ_OFFSET(TACNA_IRQ_AOD_JD2_FALL)] = {
		.reg_offset =  0, .mask = TACNA_JD2_FALL_EINT1_MASK,
	},
	[TACNA_AOD_IRQ_OFFSET(TACNA_IRQ_AOD_MICD_CLAMP1_RISE)] = {
		.reg_offset =  0, .mask = TACNA_MICD_CLAMP1_RISE_EINT1_MASK,
	},
	[TACNA_AOD_IRQ_OFFSET(TACNA_IRQ_AOD_MICD_CLAMP1_FALL)] = {
		.reg_offset =  0, .mask = TACNA_MICD_CLAMP1_FALL_EINT1_MASK,
	},
	[TACNA_AOD_IRQ_OFFSET(TACNA_IRQ_AOD_JD3_RISE)] = {
		.reg_offset =  0, .mask = TACNA_JD3_RISE_EINT1_MASK,
	},
	[TACNA_AOD_IRQ_OFFSET(TACNA_IRQ_AOD_JD3_FALL)] = {
		.reg_offset =  0, .mask = TACNA_JD3_FALL_EINT1_MASK,
	},
	[TACNA_AOD_IRQ_OFFSET(TACNA_IRQ_AOD_MICD_CLAMP2_RISE)] = {
		.reg_offset =  0, .mask = TACNA_MICD_CLAMP2_RISE_EINT1_MASK,
	},
	[TACNA_AOD_IRQ_OFFSET(TACNA_IRQ_AOD_MICD_CLAMP2_FALL)] = {
		.reg_offset =  0, .mask = TACNA_MICD_CLAMP2_FALL_EINT1_MASK,
	},
};

static const struct regmap_irq_chip tacna_main_irqchip = {
	.name = "tacna IRQ",
	.status_base = TACNA_IRQ1_EINT_1,
	.mask_base = TACNA_IRQ1_MASK_1,
	.ack_base = TACNA_IRQ1_EINT_1,
	.num_regs = 11,
	.irqs = tacna_main_irqs,
	.num_irqs = ARRAY_SIZE(tacna_main_irqs),
};

static const struct regmap_irq_chip tacna_aod_irqchip = {
	.name = "tacna AOD IRQ",
	.status_base = TACNA_IRQ1_EINT_AOD,
	.mask_base = TACNA_IRQ1_MASK_AOD,
	.ack_base = TACNA_IRQ1_EINT_AOD,
	.num_regs = 1,
	.irqs = tacna_aod_irqs,
	.num_irqs = ARRAY_SIZE(tacna_aod_irqs),
};

static int tacna_map_irq(struct tacna *tacna, int irq)
{
	struct tacna_irq_priv *priv = dev_get_drvdata(tacna->irq_dev);

	if (irq < 0)
		return irq;

	if (!tacna->irq_dev)
		return -ENOENT;

	if (irq < TACNA_NUM_MAIN_IRQ) {
		dev_dbg(priv->dev, "Mapping IRQ%d to main virq\n", irq);
		return regmap_irq_get_virq(priv->irq_chip, irq);
	} else if (priv->aod_irq_chip) {
		dev_dbg(priv->dev, "Mapping IRQ%d to AOD virq\n", irq);
		irq -= TACNA_NUM_MAIN_IRQ;
		return regmap_irq_get_virq(priv->aod_irq_chip, irq);
	}

	dev_err(priv->dev, "IRQ%d can't be mapped\n", irq);
	return -ENOENT;
}

int tacna_request_irq(struct tacna *tacna, int irq, const char *name,
		      irq_handler_t handler, void *data)
{
	irq = tacna_map_irq(tacna, irq);

	if (irq < 0)
		return irq;

	return request_threaded_irq(irq, NULL, handler, IRQF_ONESHOT, name,
				    data);

}
EXPORT_SYMBOL_GPL(tacna_request_irq);

void tacna_free_irq(struct tacna *tacna, int irq, void *data)
{
	irq = tacna_map_irq(tacna, irq);

	if (irq < 0)
		return;

	free_irq(irq, data);
}
EXPORT_SYMBOL_GPL(tacna_free_irq);

int tacna_set_irq_wake(struct tacna *tacna, int irq, int on)
{
	irq = tacna_map_irq(tacna, irq);

	if (irq < 0)
		return irq;

	return irq_set_irq_wake(irq, on);
}
EXPORT_SYMBOL_GPL(tacna_set_irq_wake);

static irqreturn_t tacna_irq_thread(int irq, void *data)
{
	struct tacna_irq_priv *priv = data;
	struct tacna *tacna = priv->tacna;
	unsigned int val, mask;
	irqreturn_t result = IRQ_NONE;
	int ret;

	/* Must power up parent MFD driver to access registers */
	ret = pm_runtime_get_sync(tacna->dev);
	if (ret < 0) {
		dev_err(priv->dev, "Failed to resume device: %d\n", ret);
		return IRQ_NONE;
	}

	if (priv->aod_irq_chip) {
		/* Check whether AOD interrupts are pending */
		ret = regmap_read(tacna->regmap, TACNA_IRQ1_EINT_AOD, &val);
		if (ret) {
			dev_warn(priv->dev, "Failed to read AOD IRQ1 %d\n",
				 ret);
		} else {
			ret = regmap_read(tacna->regmap, TACNA_IRQ1_MASK_AOD,
					  &mask);
			if (ret)
				dev_warn(priv->dev,
					 "Failed to read AOD IRQ1 MASK %d\n",
					 ret);
		}

		if ((ret == 0) && (val & ~mask)) {
			result = IRQ_HANDLED;
			handle_nested_irq(irq_find_mapping(priv->virq,
							TACNA_AOD_VIRQ_INDEX));
		}
	}

	/* Check whether main interrupts are pending */
	ret = regmap_read(tacna->regmap, TACNA_IRQ1_STATUS, &val);
	if (ret) {
		dev_warn(priv->dev, "Failed to read IRQ1_STATUS %d\n", ret);
	} else if (val & TACNA_IRQ1_STS_MASK) {
		result = IRQ_HANDLED;
		handle_nested_irq(irq_find_mapping(priv->virq,
						   TACNA_MAIN_VIRQ_INDEX));
	}

	pm_runtime_mark_last_busy(tacna->dev);
	pm_runtime_put_autosuspend(tacna->dev);

	return result;
}

static void tacna_root_irq_disable(struct irq_data *data)
{
}

static void tacna_root_irq_enable(struct irq_data *data)
{
}

static int tacna_root_irq_set_wake(struct irq_data *data, unsigned int on)
{
	struct tacna_irq_priv *priv = irq_data_get_irq_chip_data(data);

	return irq_set_irq_wake(priv->irq, on);
}

static struct irq_chip tacna_root_irqchip = {
	.name		= "tacna",
	.irq_disable	= &tacna_root_irq_disable,
	.irq_enable	= &tacna_root_irq_enable,
	.irq_set_wake	= &tacna_root_irq_set_wake,
};

static struct lock_class_key tacna_irq_lock_class;

static int tacna_irq_map(struct irq_domain *d, unsigned int virq,
			 irq_hw_number_t hw)
{
	struct tacna_irq_priv *priv = d->host_data;

	dev_dbg(priv->dev, "Mapping IRQ%lu", hw);

	irq_set_chip_data(virq, priv);
	irq_set_lockdep_class(virq, &tacna_irq_lock_class);
	irq_set_chip_and_handler(virq, &tacna_root_irqchip, handle_simple_irq);
	irq_set_nested_thread(virq, TACNA_MAIN_VIRQ_INDEX);

	return 0;
}

static struct irq_domain_ops tacna_irq_domain_ops = {
	.map	= &tacna_irq_map,
	.xlate	= &irq_domain_xlate_twocell,
};

#ifdef CONFIG_PM_SLEEP
static int tacna_suspend(struct device *dev)
{
	struct tacna_irq_priv *priv = dev_get_drvdata(dev);

	dev_dbg(priv->dev, "Suspend, disabling IRQ\n");

	/* Prevent IRQs while things are suspending */
	disable_irq(priv->irq);

	return 0;
}

static int tacna_suspend_noirq(struct device *dev)
{
	struct tacna_irq_priv *priv = dev_get_drvdata(dev);

	dev_dbg(priv->dev, "No IRQ suspend, reenabling IRQ\n");

	enable_irq(priv->irq);

	return 0;
}

static int tacna_resume_noirq(struct device *dev)
{
	struct tacna_irq_priv *priv = dev_get_drvdata(dev);

	dev_dbg(priv->dev, "No IRQ resume, disabling IRQ\n");

	disable_irq(priv->irq);

	return 0;
}

static int tacna_resume(struct device *dev)
{
	struct tacna_irq_priv *priv = dev_get_drvdata(dev);

	dev_dbg(priv->dev, "Resume, reenabling IRQ\n");

	enable_irq(priv->irq);

	return 0;
}
#endif

static const struct dev_pm_ops tacna_irq_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tacna_suspend, tacna_resume)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(tacna_suspend_noirq, tacna_resume_noirq)
};

static irqreturn_t tacna_sysclk_fail(int irq, void *data)
{
	struct tacna *tacna = data;

	dev_err(tacna->dev, "SYSCLK fail\n");

	return IRQ_HANDLED;
}

static irqreturn_t tacna_sysclk_error(int irq, void *data)
{
	struct tacna *tacna = data;

	dev_err(tacna->dev, "SYSCLK error\n");

	return IRQ_HANDLED;
}

static irqreturn_t tacna_ctrlif_error(int irq, void *data)
{
	struct tacna *tacna = data;

	dev_err(tacna->dev, "CTRLIF error\n");

	return IRQ_HANDLED;
}

static irqreturn_t tacna_boot_done(int irq, void *data)
{
	struct tacna *tacna = data;

	dev_dbg(tacna->dev, "BOOT_DONE\n");

	return IRQ_HANDLED;
}

static int tacna_irq_probe(struct platform_device *pdev)
{
	struct tacna *tacna = dev_get_drvdata(pdev->dev.parent);
	struct tacna_irq_priv *priv;
	struct irq_data *irq_data;
	unsigned int irq_flags = tacna->pdata.irqchip.irq_flags;
	unsigned int virq;
	int ret;

	dev_dbg(&pdev->dev, "probe\n");

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	priv->tacna = tacna;
	priv->irq = tacna->irq;
	dev_set_drvdata(priv->dev, priv);

	/* Read the flags from the interrupt controller if not specified */
	if (!irq_flags) {
		irq_data = irq_get_irq_data(priv->irq);
		if (!irq_data) {
			dev_err(priv->dev, "Invalid IRQ: %d\n", priv->irq);
			return -EINVAL;
		}

		irq_flags = irqd_get_trigger_type(irq_data);
		if (irq_flags == IRQ_TYPE_NONE)
			irq_flags = IRQF_TRIGGER_LOW; /* Device default */
	}

	if (irq_flags & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)) {
		dev_err(priv->dev, "Host interrupt not level-triggered\n");
		return -EINVAL;
	}

	if (irq_flags & IRQF_TRIGGER_HIGH)
		ret = regmap_update_bits(tacna->regmap, TACNA_IRQ1_CTRL_AOD,
					 TACNA_IRQ_POL_MASK, 0);
	else
		ret = regmap_update_bits(tacna->regmap, TACNA_IRQ1_CTRL_AOD,
					 TACNA_IRQ_POL_MASK, TACNA_IRQ_POL);
	if (ret) {
		dev_err(priv->dev, "Failed to set IRQ polarity: %d\n", ret);
		return ret;
	}

	/*
	 * Allocate a virtual IRQ domain to distribute to the regmap domains
	 * (without a OF node so that it's invisible to DT IRQ mappings)
	 */
	priv->virq = irq_domain_add_linear(NULL, 2,
					   &tacna_irq_domain_ops, priv);
	if (!priv->virq) {
		dev_err(priv->dev, "Failed to add core IRQ domain\n");
		ret = -EINVAL;
		goto err;
	}

	/*
	 * Create regmap interrupt handlers. Regmap registers its domains
	 * against the OF node of the owner of the regmap, which is the mfd
	 */
	switch (priv->tacna->type) {
	case CS48L31:
	case CS48L32:
	case CS48L33:
		break;
	default:
		virq = irq_create_mapping(priv->virq, TACNA_AOD_VIRQ_INDEX);
		if (!virq) {
			dev_err(priv->dev, "Failed to map AOD IRQs\n");
			ret = -EINVAL;
			goto err_domain;
		}

		ret = regmap_add_irq_chip(tacna->regmap, virq, IRQF_ONESHOT, 0,
					  &tacna_aod_irqchip,
					  &priv->aod_irq_chip);
		if (ret) {
			dev_err(priv->dev, "Failed to add AOD IRQs: %d\n", ret);
			goto err_map_aod;
		}
	}

	virq = irq_create_mapping(priv->virq, TACNA_MAIN_VIRQ_INDEX);
	if (!virq) {
		dev_err(priv->dev, "Failed to map main IRQs\n");
		ret = -EINVAL;
		goto err_aod;
	}

	ret = regmap_add_irq_chip(tacna->regmap, virq, IRQF_ONESHOT, 0,
				  &tacna_main_irqchip, &priv->irq_chip);
	if (ret) {
		dev_err(priv->dev, "Failed to add main IRQs: %d\n", ret);
		goto err_map_main_irq;
	}

	ret = request_threaded_irq(priv->irq, NULL, tacna_irq_thread,
				   irq_flags | IRQF_ONESHOT, "tacna", priv);

	if (ret) {
		dev_err(priv->dev, "Failed to request IRQ(%d) thread: %d\n",
			priv->irq, ret);
		goto err_main_irq;
	}

	tacna->irq_dev = priv->dev;

	tacna_request_irq(tacna, TACNA_IRQ_SYSCLK_FAIL, "SYSCLK fail",
			  tacna_sysclk_fail, tacna);
	tacna_request_irq(tacna, TACNA_IRQ_SYSCLK_ERR, "SYSCLK error",
			  tacna_sysclk_error, tacna);
	tacna_request_irq(tacna, TACNA_IRQ_CTRLIF_ERR, "CTRLIF error",
			  tacna_ctrlif_error, tacna);
	tacna_request_irq(tacna, TACNA_IRQ_BOOT_DONE, "BOOT_DONE",
			  tacna_boot_done, tacna);

	return 0;

err_main_irq:
	regmap_del_irq_chip(irq_find_mapping(priv->virq, TACNA_MAIN_VIRQ_INDEX),
			    priv->irq_chip);
err_map_main_irq:
	irq_dispose_mapping(irq_find_mapping(priv->virq,
					     TACNA_MAIN_VIRQ_INDEX));
err_aod:
	if (priv->aod_irq_chip)
		regmap_del_irq_chip(irq_find_mapping(priv->virq,
						     TACNA_AOD_VIRQ_INDEX),
						     priv->aod_irq_chip);
err_map_aod:
	irq_dispose_mapping(irq_find_mapping(priv->virq, TACNA_AOD_VIRQ_INDEX));
err_domain:
	irq_domain_remove(priv->virq);
err:
	return ret;
}

static int tacna_irq_remove(struct platform_device *pdev)
{
	struct tacna_irq_priv *priv = platform_get_drvdata(pdev);
	struct tacna *tacna = priv->tacna;
	unsigned int virq;

	disable_irq(priv->irq);

	tacna_free_irq(tacna, TACNA_IRQ_BOOT_DONE, tacna);
	tacna_free_irq(tacna, TACNA_IRQ_CTRLIF_ERR, tacna);
	tacna_free_irq(tacna, TACNA_IRQ_SYSCLK_ERR, tacna);
	tacna_free_irq(tacna, TACNA_IRQ_SYSCLK_FAIL, tacna);

	priv->tacna->irq_dev = NULL;

	virq = irq_find_mapping(priv->virq, TACNA_MAIN_VIRQ_INDEX);
	regmap_del_irq_chip(virq, priv->irq_chip);
	irq_dispose_mapping(virq);

	virq = irq_find_mapping(priv->virq, TACNA_AOD_VIRQ_INDEX);
	if (priv->aod_irq_chip)
		regmap_del_irq_chip(virq, priv->aod_irq_chip);
	irq_dispose_mapping(virq);

	irq_domain_remove(priv->virq);

	free_irq(priv->irq, priv);

	return 0;
}

static struct platform_driver tacna_irq_driver = {
	.probe = &tacna_irq_probe,
	.remove = &tacna_irq_remove,
	.driver = {
		.name	= "tacna-irq",
		.pm = &tacna_irq_pm_ops,
	}
};

module_platform_driver(tacna_irq_driver);

MODULE_DESCRIPTION("Tacna IRQ driver");
MODULE_AUTHOR("Richard Fitzgerald <rf@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL v2");
