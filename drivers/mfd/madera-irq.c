/*
 * madera-irq.c  --  interrupt support for Cirrus Logic Madera codecs
 *
 * Copyright 2015 Cirrus Logic
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <linux/mfd/madera/core.h>
#include <linux/mfd/madera/registers.h>

#include "madera.h"

int madera_request_irq(struct madera *madera, int irq, const char *name,
		     irq_handler_t handler, void *data)
{
	if (irq < 0)
		return irq;

	return request_threaded_irq(regmap_irq_get_virq(madera->irq_data, irq),
				    NULL, handler, IRQF_ONESHOT, name, data);

}
EXPORT_SYMBOL_GPL(madera_request_irq);

void madera_free_irq(struct madera *madera, int irq, void *data)
{
	if (irq < 0)
		return;

	free_irq(regmap_irq_get_virq(madera->irq_data, irq), data);
}
EXPORT_SYMBOL_GPL(madera_free_irq);

int madera_set_irq_wake(struct madera *madera, int irq, int on)
{
	if (irq < 0)
		return irq;

	return irq_set_irq_wake(regmap_irq_get_virq(madera->irq_data, irq), on);
}
EXPORT_SYMBOL_GPL(madera_set_irq_wake);

static irqreturn_t madera_edge_irq_thread(int irq, void *data)
{
	struct madera *madera = data;
	bool poll;
	int ret;

	dev_dbg(madera->dev, "edge_irq_thread handler\n");

	ret = pm_runtime_get_sync(madera->dev);
	if (ret < 0) {
		dev_err(madera->dev, "Failed to resume device: %d\n", ret);
		return IRQ_NONE;
	}

	do {
		poll = false;

		handle_nested_irq(irq_find_mapping(madera->edge_domain, 0));

		/*
		 * Poll the IRQ pin status to see if we're really done
		 * if the interrupt controller can't do it for us.
		 */
		if (!madera->pdata.irq_gpio) {
			break;
		} else if (madera->pdata.irq_flags & IRQF_TRIGGER_RISING &&
			   gpio_get_value_cansleep(madera->pdata.irq_gpio)) {
			poll = true;
		} else if (madera->pdata.irq_flags & IRQF_TRIGGER_FALLING &&
			   !gpio_get_value_cansleep(madera->pdata.irq_gpio)) {
			poll = true;
		}
	} while (poll);

	pm_runtime_mark_last_busy(madera->dev);
	pm_runtime_put_autosuspend(madera->dev);

	return IRQ_HANDLED;
}

static void madera_edge_irq_dummy(struct irq_data *data)
{
}

static int madera_edge_irq_set_wake(struct irq_data *data, unsigned int on)
{
	struct madera *madera = irq_data_get_irq_chip_data(data);

	return irq_set_irq_wake(madera->irq, on);
}
static struct irq_chip madera_edge_irq_chip = {
	.name		= "madera",
	.irq_disable	= madera_edge_irq_dummy,
	.irq_enable	= madera_edge_irq_dummy,
	.irq_ack	= madera_edge_irq_dummy,
	.irq_mask	= madera_edge_irq_dummy,
	.irq_unmask	= madera_edge_irq_dummy,
	.irq_set_wake	= madera_edge_irq_set_wake,
};

static int madera_edge_irq_map(struct irq_domain *h, unsigned int virq,
				irq_hw_number_t hw)
{
	struct madera *madera = h->host_data;

	irq_set_chip_data(virq, madera);
	irq_set_chip_and_handler(virq, &madera_edge_irq_chip, handle_edge_irq);
	irq_set_nested_thread(virq, true);

	/* ARM needs us to explicitly flag the IRQ as valid
	 * and will set them noprobe when we do so. */
#ifdef CONFIG_ARM
	set_irq_flags(virq, IRQF_VALID);
#else
	irq_set_noprobe(virq);
#endif

	return 0;
}

static const struct irq_domain_ops madera_edge_domain_ops = {
	.map	= madera_edge_irq_map,
	.xlate	= irq_domain_xlate_twocell,
};

int madera_irq_init(struct madera *madera)
{
	int flags = IRQF_ONESHOT;
	const struct regmap_irq_chip *irq = NULL;
	struct irq_data *irq_data;
	int ret;

	switch (madera->type) {
	case CS47L35:
		if (IS_ENABLED(CONFIG_MFD_CS47L35))
			irq = &cs47l35_irq;
		break;
	case CS47L85:
	case WM1840:
		if (IS_ENABLED(CONFIG_MFD_CS47L85))
			irq = &cs47l85_irq;
		break;
	case CS47L90:
	case CS47L91:
		if (IS_ENABLED(CONFIG_MFD_CS47L90))
			irq = &cs47l90_irq;
		break;
	default:
		dev_err(madera->dev, "Unknown Madera class device %d",
			madera->type);
		return -EINVAL;
	}

	if (!irq)
		return -EINVAL;

	/* Read the flags from the interrupt controller if not specified */
	if (!madera->pdata.irq_flags) {
		irq_data = irq_get_irq_data(madera->irq);
		if (!irq_data) {
			dev_err(madera->dev, "Invalid IRQ: %d\n", madera->irq);
			return -EINVAL;
		}

		madera->pdata.irq_flags = irqd_get_trigger_type(irq_data);
		switch (madera->pdata.irq_flags) {
		case IRQF_TRIGGER_LOW:
		case IRQF_TRIGGER_HIGH:
		case IRQF_TRIGGER_RISING:
		case IRQF_TRIGGER_FALLING:
			break;

		case IRQ_TYPE_NONE:
		default:
			/* Device default */
			madera->pdata.irq_flags = IRQF_TRIGGER_LOW;
			break;
		}
	}

	flags |= madera->pdata.irq_flags;

	if (flags & (IRQF_TRIGGER_HIGH | IRQF_TRIGGER_RISING)) {
		ret = regmap_update_bits(madera->regmap, MADERA_IRQ1_CTRL,
					 MADERA_IM_IRQ_POL, 0);
		if (ret) {
			dev_err(madera->dev,
				"Couldn't set IRQ polarity: %d\n", ret);
			return ret;
		}
	}


	if (flags & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)) {
		dev_dbg(madera->dev, "edge-trigger mode irq_gpio=%d\n",
			madera->pdata.irq_gpio);

		/* A GPIO is recommended to emulate edge trigger */
		if (madera->pdata.irq_gpio) {
			if (gpio_to_irq(madera->pdata.irq_gpio) !=
			    madera->irq) {
				dev_warn(madera->dev,
					 "IRQ %d is not GPIO %d (%d)\n",
					 madera->irq, madera->pdata.irq_gpio,
					 gpio_to_irq(madera->pdata.irq_gpio));

				madera->irq =
					gpio_to_irq(madera->pdata.irq_gpio);
			}

			ret = devm_gpio_request_one(madera->dev,
						    madera->pdata.irq_gpio,
						    GPIOF_IN, "madera IRQ");
			if (ret) {
				dev_err(madera->dev,
					"Failed to request IRQ GPIO %d: %d\n",
					madera->pdata.irq_gpio, ret);
				return ret;
			}
		}

		madera->edge_domain = irq_domain_add_linear(NULL, 1,
							&madera_edge_domain_ops,
							madera);

		ret = regmap_add_irq_chip(madera->regmap,
					irq_create_mapping(madera->edge_domain,
							   0),
					IRQF_ONESHOT, 0, irq,
					&madera->irq_data);
		if (ret) {
			dev_err(madera->dev, "add_irq_chip failed: %d\n", ret);
			return ret;
		}

		ret = request_threaded_irq(madera->irq, NULL,
					   madera_edge_irq_thread,
					   flags, "madera", madera);

		if (ret) {
			dev_err(madera->dev,
				"Failed to request threaded irq %d: %d\n",
				madera->irq, ret);
			regmap_del_irq_chip(madera->irq, madera->irq_data);
			return ret;
		}
	} else {
		dev_dbg(madera->dev, "level-trigger mode\n");

		ret = regmap_add_irq_chip(madera->regmap,
					  madera->irq,
					  IRQF_ONESHOT, 0, irq,
					  &madera->irq_data);
		if (ret) {
			dev_err(madera->dev, "add_irq_chip failed: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

int madera_irq_exit(struct madera *madera)
{
	regmap_del_irq_chip(madera->irq, madera->irq_data);

	free_irq(madera->irq, madera);

	return 0;
}
