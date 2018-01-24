/*
 * Interrupt support for Cirrus Logic Madera codecs
 *
 * Copyright 2016-2017 Cirrus Logic
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
#include <linux/irqchip/irq-madera.h>
#include <linux/irqchip/irq-madera-pdata.h>
#include <linux/mfd/madera/core.h>
#include <linux/mfd/madera/pdata.h>
#include <linux/mfd/madera/registers.h>

struct madera_irq_priv {
	struct device *dev;
	int irq;
	struct regmap_irq_chip_data *irq_data;
	struct madera *madera;
};

static const struct regmap_irq madera_irqs[MADERA_NUM_IRQ] = {
	[MADERA_IRQ_FLL1_LOCK] =  { .reg_offset = 0,
				    .mask = MADERA_FLL1_LOCK_EINT1 },
	[MADERA_IRQ_FLL2_LOCK] =  { .reg_offset = 0,
				    .mask = MADERA_FLL2_LOCK_EINT1 },
	[MADERA_IRQ_FLL3_LOCK] =  { .reg_offset = 0,
				    .mask = MADERA_FLL3_LOCK_EINT1 },
	[MADERA_IRQ_FLLAO_LOCK] = { .reg_offset = 0,
				    .mask = MADERA_FLLAO_LOCK_EINT1 },

	[MADERA_IRQ_MICDET1] = { .reg_offset = 4,
				 .mask = MADERA_MICDET1_EINT1 },
	[MADERA_IRQ_MICDET2] = { .reg_offset = 4,
				 .mask = MADERA_MICDET2_EINT1 },
	[MADERA_IRQ_HPDET] =   { .reg_offset = 4,
				 .mask = MADERA_HPDET_EINT1 },

	[MADERA_IRQ_MICD_CLAMP_RISE] = { .reg_offset = 5,
					 .mask = MADERA_MICD_CLAMP_RISE_EINT1 },
	[MADERA_IRQ_MICD_CLAMP_FALL] = { .reg_offset = 5,
					 .mask = MADERA_MICD_CLAMP_FALL_EINT1 },
	[MADERA_IRQ_JD1_FALL] =	       { .reg_offset = 5,
					 .mask = MADERA_JD1_FALL_EINT1 },
	[MADERA_IRQ_JD1_RISE] =	       { .reg_offset = 5,
					 .mask = MADERA_JD1_RISE_EINT1 },

	[MADERA_IRQ_ASRC2_IN1_LOCK] = { .reg_offset = 7,
					.mask = MADERA_ASRC2_IN1_LOCK_EINT1 },
	[MADERA_IRQ_ASRC2_IN2_LOCK] = { .reg_offset = 7,
					.mask = MADERA_ASRC2_IN2_LOCK_EINT1 },
	[MADERA_IRQ_ASRC1_IN1_LOCK] = { .reg_offset = 7,
					.mask = MADERA_ASRC1_IN1_LOCK_EINT1 },
	[MADERA_IRQ_ASRC1_IN2_LOCK] = { .reg_offset = 7,
					.mask = MADERA_ASRC1_IN2_LOCK_EINT1 },

	[MADERA_IRQ_DRC2_SIG_DET] = { .reg_offset = 7,
				      .mask = MADERA_DRC2_SIG_DET_EINT1 },
	[MADERA_IRQ_DRC1_SIG_DET] = { .reg_offset = 7,
				      .mask = MADERA_DRC1_SIG_DET_EINT1 },

	[MADERA_IRQ_DSP_IRQ1] = { .reg_offset = 9,
				  .mask = MADERA_DSP_IRQ1_EINT1 },
	[MADERA_IRQ_DSP_IRQ2] = { .reg_offset = 9,
				  .mask = MADERA_DSP_IRQ2_EINT1 },
	[MADERA_IRQ_DSP_IRQ3] = { .reg_offset = 9,
				  .mask = MADERA_DSP_IRQ3_EINT1 },
	[MADERA_IRQ_DSP_IRQ4] = { .reg_offset = 9,
				  .mask = MADERA_DSP_IRQ4_EINT1 },
	[MADERA_IRQ_DSP_IRQ5] = { .reg_offset = 9,
				  .mask = MADERA_DSP_IRQ5_EINT1 },
	[MADERA_IRQ_DSP_IRQ6] = { .reg_offset = 9,
				  .mask = MADERA_DSP_IRQ6_EINT1 },
	[MADERA_IRQ_DSP_IRQ7] = { .reg_offset = 9,
				  .mask = MADERA_DSP_IRQ7_EINT1 },
	[MADERA_IRQ_DSP_IRQ8] = { .reg_offset = 9,
				  .mask = MADERA_DSP_IRQ8_EINT1 },
	[MADERA_IRQ_DSP_IRQ9] = { .reg_offset = 9,
				  .mask = MADERA_DSP_IRQ9_EINT1 },
	[MADERA_IRQ_DSP_IRQ10] = { .reg_offset = 9,
				   .mask = MADERA_DSP_IRQ10_EINT1 },
	[MADERA_IRQ_DSP_IRQ11] = { .reg_offset = 9,
				   .mask = MADERA_DSP_IRQ11_EINT1 },
	[MADERA_IRQ_DSP_IRQ12] = { .reg_offset = 9,
				   .mask = MADERA_DSP_IRQ12_EINT1 },
	[MADERA_IRQ_DSP_IRQ13] = { .reg_offset = 9,
				   .mask = MADERA_DSP_IRQ13_EINT1 },
	[MADERA_IRQ_DSP_IRQ14] = { .reg_offset = 9,
				   .mask = MADERA_DSP_IRQ14_EINT1 },
	[MADERA_IRQ_DSP_IRQ15] = { .reg_offset = 9,
				   .mask = MADERA_DSP_IRQ15_EINT1 },
	[MADERA_IRQ_DSP_IRQ16] = { .reg_offset = 9,
				   .mask = MADERA_DSP_IRQ16_EINT1 },

	[MADERA_IRQ_HP3R_SC] = { .reg_offset = 10,
				.mask = MADERA_HP3R_SC_EINT1 },
	[MADERA_IRQ_HP3L_SC] = { .reg_offset = 10,
				.mask = MADERA_HP3L_SC_EINT1 },
	[MADERA_IRQ_HP2R_SC] = { .reg_offset = 10,
				.mask = MADERA_HP2R_SC_EINT1 },
	[MADERA_IRQ_HP2L_SC] = { .reg_offset = 10,
				.mask = MADERA_HP2L_SC_EINT1 },
	[MADERA_IRQ_HP1R_SC] = { .reg_offset = 10,
				.mask = MADERA_HP1R_SC_EINT1 },
	[MADERA_IRQ_HP1L_SC] = { .reg_offset = 10,
				.mask = MADERA_HP1L_SC_EINT1 },

	[MADERA_IRQ_SPK_OVERHEAT_WARN] = { .reg_offset = 13,
				.mask = MADERA_SPK_OVERHEAT_WARN_EINT1 },
	[MADERA_IRQ_SPK_OVERHEAT] = { .reg_offset = 13,
				.mask = MADERA_SPK_SHUTDOWN_EINT1 },

	[MADERA_IRQ_DSP1_BUS_ERROR] = { .reg_offset = 31,
					.mask = MADERA_ADSP_ERROR_STATUS_DSP1 },
	[MADERA_IRQ_DSP2_BUS_ERROR] = { .reg_offset = 31,
					.mask = MADERA_ADSP_ERROR_STATUS_DSP2 },
	[MADERA_IRQ_DSP3_BUS_ERROR] = { .reg_offset = 31,
					.mask = MADERA_ADSP_ERROR_STATUS_DSP3 },
	[MADERA_IRQ_DSP4_BUS_ERROR] = { .reg_offset = 31,
					.mask = MADERA_ADSP_ERROR_STATUS_DSP4 },
	[MADERA_IRQ_DSP5_BUS_ERROR] = { .reg_offset = 31,
					.mask = MADERA_ADSP_ERROR_STATUS_DSP5 },
	[MADERA_IRQ_DSP6_BUS_ERROR] = { .reg_offset = 31,
					.mask = MADERA_ADSP_ERROR_STATUS_DSP6 },
	[MADERA_IRQ_DSP7_BUS_ERROR] = { .reg_offset = 31,
					.mask = MADERA_ADSP_ERROR_STATUS_DSP7 },
};

static const struct regmap_irq_chip madera_irq = {
	.name = "madera IRQ",
	.status_base = MADERA_IRQ1_STATUS_2,
	.mask_base = MADERA_IRQ1_MASK_2,
	.ack_base = MADERA_IRQ1_STATUS_2,
	.runtime_pm = true, /* codec must be resumed to read IRQ status */
	.num_regs = 32,
	.irqs = madera_irqs,
	.num_irqs = ARRAY_SIZE(madera_irqs),
};

static int madera_map_irq(struct madera *madera, int irq)
{
	struct madera_irq_priv *priv = dev_get_drvdata(madera->irq_dev);

	if (irq < 0)
		return irq;

	if (!madera->irq_dev)
		return -ENOENT;

	return regmap_irq_get_virq(priv->irq_data, irq);
}

int madera_request_irq(struct madera *madera, int irq, const char *name,
			irq_handler_t handler, void *data)
{
	irq = madera_map_irq(madera, irq);

	if (irq < 0)
		return irq;

	return request_threaded_irq(irq, NULL, handler, IRQF_ONESHOT, name,
				    data);

}
EXPORT_SYMBOL_GPL(madera_request_irq);

void madera_free_irq(struct madera *madera, int irq, void *data)
{
	irq = madera_map_irq(madera, irq);

	if (irq < 0)
		return;

	free_irq(irq, data);
}
EXPORT_SYMBOL_GPL(madera_free_irq);

int madera_set_irq_wake(struct madera *madera, int irq, int on)
{
	irq = madera_map_irq(madera, irq);

	if (irq < 0)
		return irq;

	return irq_set_irq_wake(irq, on);
}
EXPORT_SYMBOL_GPL(madera_set_irq_wake);

#ifdef CONFIG_PM_SLEEP
static int madera_suspend_noirq(struct device *dev)
{
	struct madera_irq_priv *priv = dev_get_drvdata(dev);

	dev_dbg(priv->dev, "No IRQ suspend, reenabling IRQ\n");

	enable_irq(priv->irq);

	return 0;
}

static int madera_suspend(struct device *dev)
{
	struct madera_irq_priv *priv = dev_get_drvdata(dev);

	dev_dbg(priv->dev, "Suspend, disabling IRQ\n");

	disable_irq(priv->irq);

	return 0;
}

static int madera_resume_noirq(struct device *dev)
{
	struct madera_irq_priv *priv = dev_get_drvdata(dev);

	dev_dbg(priv->dev, "No IRQ resume, disabling IRQ\n");

	disable_irq(priv->irq);

	return 0;
}

static int madera_resume(struct device *dev)
{
	struct madera_irq_priv *priv = dev_get_drvdata(dev);

	dev_dbg(priv->dev, "Resume, reenabling IRQ\n");

	enable_irq(priv->irq);

	return 0;
}
#endif

static const struct dev_pm_ops madera_irq_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(madera_suspend, madera_resume)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(madera_suspend_noirq,
				      madera_resume_noirq)
};

static int madera_irq_probe(struct platform_device *pdev)
{
	struct madera *madera = dev_get_drvdata(pdev->dev.parent);
	struct madera_irq_priv *priv;
	struct irq_data *irq_data;
	unsigned int irq_flags = madera->pdata.irqchip.irq_flags;
	int ret;

	dev_dbg(&pdev->dev, "probe\n");

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	priv->madera = madera;
	priv->irq = madera->irq;

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
		dev_err(priv->dev,
			"Host interrupt not level-triggered\n");
		return -EINVAL;
	}

	if (irq_flags & IRQF_TRIGGER_HIGH) {
		ret = regmap_update_bits(madera->regmap, MADERA_IRQ1_CTRL,
					 MADERA_IRQ_POL_MASK, 0);
		if (ret) {
			dev_err(priv->dev,
				"Failed to set IRQ polarity: %d\n", ret);
			return ret;
		}
	}

	/*
	 * NOTE: regmap registers this against the OF node of the parent of
	 * the regmap - that is, against the mfd driver
	 */
	ret = regmap_add_irq_chip(madera->regmap, priv->irq, IRQF_ONESHOT, 0,
				  &madera_irq, &priv->irq_data);
	if (ret) {
		dev_err(priv->dev, "add_irq_chip failed: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, priv);
	madera->irq_dev = priv->dev;

	return 0;
}

static int madera_irq_remove(struct platform_device *pdev)
{
	struct madera_irq_priv *priv = platform_get_drvdata(pdev);

	/*
	 * The IRQ is disabled by the parent MFD driver before
	 * it starts cleaning up all child drivers
	 */

	priv->madera->irq_dev = NULL;

	regmap_del_irq_chip(priv->irq, priv->irq_data);
	free_irq(priv->irq, priv);

	return 0;
}

static struct platform_driver madera_irq_driver = {
	.probe = madera_irq_probe,
	.remove = madera_irq_remove,
	.driver = {
		.name	= "madera-irq",
		.pm = &madera_irq_pm_ops,
	}
};

module_platform_driver(madera_irq_driver);

MODULE_DESCRIPTION("Madera IRQ driver");
MODULE_AUTHOR("Richard Fitzgerald <rf@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL v2");
