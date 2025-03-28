/*
 * pinctrl/ingenic/pinctrl-ingenic.c
 *
 * Copyright 2015 Ingenic Semiconductor Co.,Ltd
 *
 * Author: cli <chen.li@ingenic.com>
 *
 * redistribute from drivers/pinctrl/pinctrl-samsung.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/syscore_ops.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/slab.h>
#include <linux/gpio/driver.h>
#include <dt-bindings/gpio/ingenic-gpio.h>

#include "pinctrl-ingenic.h"

#define PINCTL_NAME(x)   #x

static struct ingenic_pinctrl *gpctl = NULL;
static const struct ingenic_priv common_priv_data[];
static const struct of_device_id ingenic_pinctrl_dt_match[];

static void ingenic_gpio_set_func_normal(struct ingenic_gpio_chip *chip,
        enum gpio_function func, u32 pins)
{
	unsigned long flags, func_tmp = func;
	spin_lock_irqsave(&chip->lock, flags);
	if (chip->pctl->priv->dual_edge_interrupt) {
		if (func & 0x10) {
			ingenic_gpio_writel(chip, PxEDGS, pins);
		} else {
			ingenic_gpio_writel(chip, PxEDGC, pins);
		}
	} else {
		if (func_tmp & 0x10) {
			func = GPIO_INT_FE;
		}
	}
	if (func & 0x8) {
		ingenic_gpio_writel(chip, PxINTS, pins);
	} else {
		ingenic_gpio_writel(chip, PxINTC, pins);
	}
	if (func & 0x4) {
		ingenic_gpio_writel(chip, PxMSKS, pins);
	} else {
		ingenic_gpio_writel(chip, PxMSKC, pins);
	}

	if (func & 0x2) {
		ingenic_gpio_writel(chip, PxPAT1S, pins);
	} else {
		ingenic_gpio_writel(chip, PxPAT1C, pins);
	}

	if ((func_tmp & 0x10) && !chip->pctl->priv->dual_edge_interrupt) {
		/* if no hw dual edge interrupt support,
		   Dynamic change functions to simulate dual edge interrupts. */
		int old, new, timeout = 10;
		do {
			old = ingenic_gpio_readl(chip, PxPIN) & pins;
			if (old) {
				func = GPIO_INT_FE;
			} else {
				func = GPIO_INT_RE;
			}
			if (func & 0x1) {
				ingenic_gpio_writel(chip, PxPAT0S, pins);
			} else {
				ingenic_gpio_writel(chip, PxPAT0C, pins);
			}
			new = ingenic_gpio_readl(chip, PxPIN) & pins;
			timeout --;
		} while (old != new && timeout);
		if (!timeout) {
			pr_err("pins %x function %d failed\n", pins, func);
		}
	} else {

		/* Set PAT0 only if func is not output.*/
		if ((func != GPIO_OUTPUT0) && (func != GPIO_OUTPUT1)) {
			if (func & 0x1) {
				ingenic_gpio_writel(chip, PxPAT0S, pins);
			} else {
				ingenic_gpio_writel(chip, PxPAT0C, pins);
			}
		}
	}
	ingenic_gpio_writel(chip, PxFLGC, pins);
	spin_unlock_irqrestore(&chip->lock, flags);
}

static void ingenic_gpio_fill_func_shadow(struct ingenic_gpio_chip *chip,
        enum gpio_function func, u32 pins)
{
#if defined(CONFIG_SOC_X2600) || defined(CONFIG_SOC_AD100)
	if (func & 0x8) {
		ingenic_gpio_shadow_fill(chip, PxINTSS, pins);
	} else {
		ingenic_gpio_shadow_fill(chip, PxINTCS, pins);
	}
	if (func & 0x4) {
		ingenic_gpio_shadow_fill(chip, PxMSKSS, pins);
	} else {
		ingenic_gpio_shadow_fill(chip, PxMSKCS, pins);
	}
	if (func & 0x2) {
		ingenic_gpio_shadow_fill(chip, PxPAT1SS, pins);
	} else {
		ingenic_gpio_shadow_fill(chip, PxPAT1CS, pins);
	}
#else
	if (func & 0x8) {
		ingenic_gpio_shadow_fill(chip, PxINTS, pins);
	} else {
		ingenic_gpio_shadow_fill(chip, PxINTC, pins);
	}
	if (func & 0x4) {
		ingenic_gpio_shadow_fill(chip, PxMSKS, pins);
	} else {
		ingenic_gpio_shadow_fill(chip, PxMSKC, pins);
	}
	if (func & 0x2) {
		ingenic_gpio_shadow_fill(chip, PxPAT1S, pins);
	} else {
		ingenic_gpio_shadow_fill(chip, PxPAT1C, pins);
	}
#endif
	/* Set PAT0 only if the function is not output. */
	if ((func != GPIO_OUTPUT0) && (func != GPIO_OUTPUT1)) {
#if defined(CONFIG_SOC_X2600) || defined(CONFIG_SOC_AD100)
		if (func & 0x1) {
			ingenic_gpio_shadow_fill(chip, PxPAT0SS, pins);
		} else {
			ingenic_gpio_shadow_fill(chip, PxPAT0CS, pins);
		}
#else
		if (func & 0x1) {
			ingenic_gpio_shadow_fill(chip, PxPAT0S, pins);
		} else {
			ingenic_gpio_shadow_fill(chip, PxPAT0C, pins);
		}
#endif
	}
}

static void ingenic_gpio_set_func_shadow(struct ingenic_gpio_chip *chip,
        enum gpio_function func, u32 pins)
{
	struct ingenic_pinctrl *pctl = chip->pctl;
	unsigned long flags;
	unsigned long func_tmp = func;

	spin_lock_irqsave(&pctl->shadow_lock, flags);

	if (chip->pctl->priv->dual_edge_interrupt) {
		ingenic_gpio_fill_func_shadow(chip, func, pins);
		ingenic_gpio_shadow_writel(chip);

		/*No Shadows registers for EDG, set registers directly.*/
		if (func & 0x10) {
			ingenic_gpio_writel(chip, PxEDGS, pins);
		} else {
			ingenic_gpio_writel(chip, PxEDGC, pins);
		}
	} else {
		if (func_tmp & 0x10) {
			int old, new, timeout = 10;
			do {
				old = ingenic_gpio_readl(chip, PxPIN) & pins;
				if (old) {
					func = GPIO_INT_FE;
				} else {
					func = GPIO_INT_RE;
				}
				ingenic_gpio_fill_func_shadow(chip, func, pins);
				ingenic_gpio_shadow_writel(chip);
				new = ingenic_gpio_readl(chip, PxPIN) & pins;
				timeout --;
			} while (old != new && timeout);
			if (!timeout) {
				pr_err("pins %x function %d failed\n", pins, func);
			}
		} else {
			ingenic_gpio_fill_func_shadow(chip, func, pins);
			ingenic_gpio_shadow_writel(chip);
		}
	}

	/* clear possible irqs since not enabled by irq core.*/
	ingenic_gpio_writel(chip, PxFLGC, pins);

	spin_unlock_irqrestore(&pctl->shadow_lock, flags);
}

/**************************************************************
 * Set GPIO Function
 *  chip:       gpio chip
 *  have_shadow:    soc support shadow register ot not
 *  func:       function select
 *  pins:       pins bitmap
 **************************************************************/
static void ingenic_gpio_set_func(struct ingenic_gpio_chip *chip,
                                  bool have_shadow, enum gpio_function func,
                                  u32 pins)
{
	if (have_shadow) {
		ingenic_gpio_set_func_shadow(chip, func, pins);
	} else {
		ingenic_gpio_set_func_normal(chip, func, pins);
	}
}

static void ingenic_gpio_set_pull(struct ingenic_gpio_chip *chip,
                                  u32 pins, unsigned int state)
{
	const struct ingenic_priv *priv = chip->pctl->priv;

	/*1. try pull_up pull_down hiz settings first.*/
	if (priv->pull_tristate) {
		if (state == INGENIC_GPIO_PULLUP) {
			ingenic_gpio_writel(chip, PxPDC, pins);
			ingenic_gpio_writel(chip, PxPUS, pins);
		} else if (state == INGENIC_GPIO_PULLDOWN) {
			ingenic_gpio_writel(chip, PxPUC, pins);
			ingenic_gpio_writel(chip, PxPDS, pins);
		} else if (state == INGENIC_GPIO_HIZ) {
			ingenic_gpio_writel(chip, PxPUC, pins);
			ingenic_gpio_writel(chip, PxPDC, pins);
		}
		return;
	}

	/*2. For gpios pullup/pulldown state can not be changed.*/
	if (priv->pullen_set) {
		if (state == INGENIC_GPIO_PULLEN) {
			ingenic_gpio_writel(chip, PxPENS, pins);
		} else {
			ingenic_gpio_writel(chip, PxPENC, pins);
		}
	} else {
		if (state == INGENIC_GPIO_PULLEN) {
			ingenic_gpio_writel(chip, PxPENC, pins);
		} else {
			ingenic_gpio_writel(chip, PxPENS, pins);
		}
	}

}

static int ingenic_gpio_get_pull_state(struct ingenic_gpio_chip *chip, u32 pins)
{
	unsigned int pull_down = 0;
	unsigned int pull_up = 0;
	int state = 0;

	pull_down = (ingenic_gpio_readl(chip, PxPD) & BIT(pins)) >> pins;
	pull_up = (ingenic_gpio_readl(chip, PxPU) & BIT(pins)) >> pins;

	if (pull_down && pull_up) {
		dev_warn(chip->gc.parent, "Set both pull_up and pull_down on %s %d\n", chip->name, pins);
	}

	if (pull_down) {
		state = INGENIC_GPIO_PULLDOWN;
	} else if (pull_up) {
		state = INGENIC_GPIO_PULLUP;
	} else {
		state = INGENIC_GPIO_HIZ;
	}

	return state;

}
/*************************************************************
 * Set GPIO Input&Interrupt HW filter
 *  chip:   gpio chip
 *  pin:    pin num (one pin)
 *  pinflt: filter to set
 *************************************************************/
static void ingenic_gpio_set_filter(struct ingenic_gpio_chip *chip,
                                    u32 pin, u16 pinflt)
{
	const struct ingenic_priv *priv = chip->pctl->priv;
	if (chip->filter_bitmap & BIT(pin)) {
		priv->set_filter(chip, pin, pinflt);
	}
}

static void ingenic_gpio_set_schmitt_enable(struct ingenic_gpio_chip *chip,
        u32 pin, int enable)
{
	const struct ingenic_priv *priv = chip->pctl->priv;

	if (!priv->have_schmit) {
		/* dev_warn(chip->gc.dev, "Do not support schmit for this chip."); */
		return;
	}

	if (enable) {
		ingenic_gpio_writel(chip, PxSMTS, BIT(pin));
	} else {
		ingenic_gpio_writel(chip, PxSMTC, BIT(pin));
	}
}

static void ingenic_gpio_set_slew_rate(struct ingenic_gpio_chip *chip,
                                       u32 pin, int enable)
{
	const struct ingenic_priv *priv = chip->pctl->priv;

	if (!priv->have_slew) {
		/* dev_warn(chip->gc.dev, "Do not support slew_rate for this chip."); */
		return;
	}

	if (enable) {
		ingenic_gpio_writel(chip, PxSRS, BIT(pin));
	} else {
		ingenic_gpio_writel(chip, PxSRC, BIT(pin));
	}
}

static void ingenic_gpio_set_drive_strength(struct ingenic_gpio_chip *chip,
        u32 pin, u16 strength)
{
	const struct ingenic_priv *priv = chip->pctl->priv;

	if (!priv->have_ds) {
		/* dev_warn(chip->gc.dev, "Do not support drive strength settings for this chip."); */
		return;
	}

	if (strength > 7) {
		strength = 7;
	}

	if (strength & BIT(0)) {
		ingenic_gpio_writel(chip, PXDS0S, BIT(pin));
	} else {
		ingenic_gpio_writel(chip, PXDS0C, BIT(pin));
	}

	if (strength & BIT(1)) {
		ingenic_gpio_writel(chip, PXDS1S, BIT(pin));
	} else {
		ingenic_gpio_writel(chip, PXDS1C, BIT(pin));
	}
#ifndef CONFIG_SOC_X2600
	if (strength & BIT(2)) {
		ingenic_gpio_writel(chip, PXDS2S, BIT(pin));
	} else {
		ingenic_gpio_writel(chip, PXDS2C, BIT(pin));
	}
#endif
}

static unsigned short ingenic_gpio_get_drive_strength(struct ingenic_gpio_chip *chip,
        u32 pin)
{
	unsigned short strength = 0;

	const struct ingenic_priv *priv = chip->pctl->priv;

	if (!priv->have_ds) {
		dev_warn(chip->gc.parent, "Do not support drive strength settings for this chip.");
		return 0;
	}

	strength |= (ingenic_gpio_readl(chip, PXDS0) & BIT(pin)) ? (1 << 0) : 0;
	strength |= (ingenic_gpio_readl(chip, PXDS1) & BIT(pin)) ? (1 << 1) : 0;
	strength |= (ingenic_gpio_readl(chip, PXDS2) & BIT(pin)) ? (1 << 2) : 0;

	return strength;
}

static void ingenic_gpio_irq_mask(struct irq_data *irqd)
{
	struct ingenic_gpio_chip *jzgc = irq_data_get_irq_chip_data(irqd);
	struct ingenic_pinctrl  *pctl = jzgc->pctl;
	unsigned long flags;

	spin_lock_irqsave(&pctl->shadow_lock, flags);
	ingenic_gpio_writel(jzgc, PxMSKS, BIT(irqd->hwirq));
	spin_unlock_irqrestore(&pctl->shadow_lock, flags);
}

static void ingenic_gpio_irq_unmask(struct irq_data *irqd)
{
	struct ingenic_gpio_chip *jzgc = irq_data_get_irq_chip_data(irqd);
	struct ingenic_pinctrl  *pctl = jzgc->pctl;
	unsigned long flags;

	spin_lock_irqsave(&pctl->shadow_lock, flags);
	ingenic_gpio_writel(jzgc, PxMSKC, BIT(irqd->hwirq));
	spin_unlock_irqrestore(&pctl->shadow_lock, flags);
}

static void ingenic_gpio_irq_ack(struct irq_data *irqd)
{
	struct ingenic_gpio_chip *jzgc = irq_data_get_irq_chip_data(irqd);
	struct ingenic_pinctrl  *pctl = jzgc->pctl;
	unsigned long flags, pins, pins_before, pending;

	spin_lock_irqsave(&pctl->shadow_lock, flags);
	ingenic_gpio_writel(jzgc, PxFLGC, BIT(irqd->hwirq));
	spin_unlock_irqrestore(&pctl->shadow_lock, flags);

	if (jzgc->pctl->priv->dual_edge_interrupt) {
		return;
	}

	/*
	 * The controller not support BOTH EDGE trigger
	 * So we set the right trigger edge after irq ack
	 * Note: To make sure our edge interrupt will be tigger and handle at next time
	 *   0, clear pending
	 *   1, read the pin level
	 *   2, set edge according the pin level
	 *   3, read the pin level
	 *   4, read the pending
	 *   5, if pin level is change and irq not pending the irq may miss, goto 2 retry
	 */
	if (irqd_get_trigger_type(irqd) == IRQ_TYPE_EDGE_BOTH) {
		pins = ingenic_gpio_readl(jzgc, PxPIN);
		do {
			ingenic_gpio_writel(jzgc, pins & BIT(irqd->hwirq) ?
			                    PxPAT0C : PxPAT0S, BIT(irqd->hwirq));
			pins_before = pins;
			pins = ingenic_gpio_readl(jzgc, PxPIN);
			pending = ingenic_gpio_readl(jzgc, PxFLG);
		} while (((pins & BIT(irqd->hwirq)) !=
		          (pins_before & BIT(irqd->hwirq))) &&
		         !(pending & BIT(irqd->hwirq)));
	}
}

static int ingenic_gpio_irq_set_type(struct irq_data *irqd, unsigned int flow_type)
{
	struct ingenic_gpio_chip *jzgc = irq_data_get_irq_chip_data(irqd);
	const struct ingenic_priv *priv = jzgc->pctl->priv;
	enum gpio_function func;

	if (!(flow_type & IRQ_TYPE_SENSE_MASK)) {
		return 0;
	}

	if (flow_type & IRQ_TYPE_EDGE_BOTH) {
		irq_set_handler_locked(irqd, handle_edge_irq);
	} else {
		irq_set_handler_locked(irqd, handle_level_irq);
	}

	switch (flow_type & IRQD_TRIGGER_MASK) {
		case IRQ_TYPE_LEVEL_HIGH:       func = GPIO_INT_HI;     break;
		case IRQ_TYPE_LEVEL_LOW:        func = GPIO_INT_LO;     break;
		case IRQ_TYPE_EDGE_RISING:      func = GPIO_INT_RE;     break;
		case IRQ_TYPE_EDGE_FALLING:     func = GPIO_INT_FE;     break;
		case IRQ_TYPE_EDGE_BOTH:        func = GPIO_INT_RE_FE;  break;
		default:
			pr_err("unsupported external interrupt type\n");
			return -EINVAL;
	}

	ingenic_gpio_set_func(jzgc, priv->have_shadow, func, BIT(irqd->hwirq));
	return 0;
}

static int ingenic_gpio_irq_set_wake(struct irq_data *irqd, unsigned int on)
{
	struct ingenic_gpio_chip *jzgc = irq_data_get_irq_chip_data(irqd);

	if (on) {
		jzgc->wakeup_bitmap |= BIT(irqd->hwirq);
	} else {
		jzgc->wakeup_bitmap &= ~BIT(irqd->hwirq);
	}

	return irq_set_irq_wake(jzgc->irq, on);
}

static void ingenic_gpio_irq_suspend(struct irq_data *irqd)
{

	/* handle wakeup irq in pinctrl suspend procedure. */
#if 0
	struct ingenic_gpio_chip *jzgc = irq_data_get_irq_chip_data(irqd);

	if ((!(jzgc->wakeup_bitmap & BIT(irqd->hwirq))) &&
	    (!(ingenic_gpio_readl(jzgc, PxMSK) & BIT(irqd->hwirq)))) {
		jzgc->pm_irq_bitmap |= BIT(irqd->hwirq);
		ingenic_gpio_writel(jzgc, PxMSKS, BIT(irqd->hwirq));
	}
#endif
}

static void ingenic_gpio_irq_resume(struct irq_data *irqd)
{
#if 0
	struct ingenic_gpio_chip *jzgc = irq_data_get_irq_chip_data(irqd);

	if (jzgc->pm_irq_bitmap & BIT(irqd->hwirq)) {
		ingenic_gpio_writel(jzgc, PxMSKC, BIT(irqd->hwirq));
		jzgc->pm_irq_bitmap &= ~BIT(irqd->hwirq);
	}
#endif
}

static int ingenic_irq_request_resources(struct irq_data *irqd)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(irqd);
	struct ingenic_gpio_chip *jzgc = (void *) chip;

	if (!try_module_get(chip->owner)) {
		return -ENODEV;
	}

	if (gpiochip_lock_as_irq(chip, irqd->hwirq)) {
		pr_err("GPIO chip %s: unable to lock HW IRQ %lu for IRQ\n",
		       chip->label,
		       irqd->hwirq);
		module_put(chip->owner);
		return -EINVAL;
	}

	jzgc->irq_bitmap |= BIT(irqd->hwirq);

	return 0;
}

static void ingenic_irq_release_resources(struct irq_data *irqd)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(irqd);
	struct ingenic_gpio_chip *jzgc = (void *) chip;

	jzgc->irq_bitmap &= ~BIT(irqd->hwirq);
	gpiochip_unlock_as_irq(chip, irqd->hwirq);
	module_put(chip->owner);
}

static struct irq_chip ingenic_gpio_irq_chip = {
	.name           = "GPIO",
	.irq_unmask     = ingenic_gpio_irq_unmask,
	.irq_mask       = ingenic_gpio_irq_mask,
	.irq_ack        = ingenic_gpio_irq_ack,
	.irq_set_type   = ingenic_gpio_irq_set_type,
	.irq_set_wake   = ingenic_gpio_irq_set_wake,
	.irq_suspend    = ingenic_gpio_irq_suspend,
	.irq_resume = ingenic_gpio_irq_resume,
	.irq_request_resources = ingenic_irq_request_resources,
	.irq_release_resources = ingenic_irq_release_resources,
};

static void ingenic_gpio_irq_handler(struct irq_desc *desc)
{
	struct ingenic_gpio_chip *jzgc = irq_desc_get_handler_data(desc);
	unsigned long pend, mask;
	unsigned long i;

	if (IS_ERR_OR_NULL(jzgc->mcu_gpio_reg)) {
		mask = ingenic_gpio_readl(jzgc, PxMSK);
		pend = ingenic_gpio_readl(jzgc, PxFLG);
		pend = pend & ~mask;
	} else {
		pend = *(jzgc->mcu_gpio_reg);
	}

	for_each_set_bit(i, &pend, jzgc->gc.ngpio) {
		if (irq_find_mapping(jzgc->irq_domain, i)) {
			generic_handle_irq(irq_find_mapping(jzgc->irq_domain, i));
		}
	}

	if (!IS_ERR_OR_NULL(jzgc->mcu_gpio_reg)) {
		*(jzgc->mcu_gpio_reg) = 0;
	}
	return;
}

static void ingenic_gpio_pins_set(struct ingenic_gpio_chip *jzgc,
                                  unsigned pins, int value)
{
	unsigned long flags, chip_flags;
	struct ingenic_pinctrl *pctl = jzgc->pctl;

	spin_lock_irqsave(&pctl->shadow_lock, flags);
	spin_lock_irqsave(&jzgc->lock, chip_flags);

	if (value) {
		ingenic_gpio_writel(jzgc, PxPAT0S, pins);
	} else {
		ingenic_gpio_writel(jzgc, PxPAT0C, pins);
	}

	spin_unlock_irqrestore(&jzgc->lock, chip_flags);
	spin_unlock_irqrestore(&pctl->shadow_lock, flags);
}

static void ingenic_gpio_set(struct gpio_chip *chip,
                             unsigned pin, int value)
{
	struct ingenic_gpio_chip *jzgc = gc_to_ingenic_gc(chip);

	BUG_ON(pin > jzgc->gc.ngpio);
	ingenic_gpio_pins_set(jzgc, BIT(pin), value);
}

static int ingenic_gpio_get(struct gpio_chip *chip, unsigned pin)
{
	struct ingenic_gpio_chip *jzgc = gc_to_ingenic_gc(chip);

	BUG_ON(pin > chip->ngpio);
	if (jzgc->resume_pending & BIT(pin)) {
		jzgc->resume_pending &= ~BIT(pin);
		return !!(jzgc->sleep_level & BIT(pin));
	}
	return !!(ingenic_gpio_readl(jzgc, PxPIN) & BIT(pin));
}

static int ingenic_gpio_direction_input(struct gpio_chip *chip,
                                        unsigned pin)
{
	BUG_ON(pin > chip->ngpio);
	return pinctrl_gpio_direction_input(chip->base + pin);
}

static int ingenic_gpio_direction_output(struct gpio_chip *chip,
        unsigned pin, int value)
{
	ingenic_gpio_set(chip, pin, value);
	return pinctrl_gpio_direction_output(chip->base + pin);
}

static int ingenic_gpio_to_irq(struct gpio_chip *chip,
                               unsigned pin)
{
	struct ingenic_gpio_chip *jzgc = gc_to_ingenic_gc(chip);
	unsigned int virq;

	BUG_ON(pin > chip->ngpio);
	if (NULL == jzgc->irq_domain) {
		return -ENXIO;
	}
	virq = irq_create_mapping(jzgc->irq_domain, pin);

	return (virq) ? : -ENXIO;
}
static int ingenic_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct ingenic_gpio_chip *jzgc = gc_to_ingenic_gc(chip);
	unsigned gpio = chip->base + offset;

	if (jzgc->used_pins_bitmap & (1 << offset)) {
		printk("%s: GP:%s  used_pins_bitmap: 0X%08X\n", __func__, jzgc->name, jzgc->used_pins_bitmap);
		//printk("current gpio request pin: chip->name %s, gpio: 0X%08X\n", chip->of_node->name, 1 << offset);
		dump_stack();
		printk("%s:gpio functions has redefinition\n", __FILE__);
	}

	jzgc->used_pins_bitmap |= 1 << offset;

	return pinctrl_gpio_request(gpio);
}
static void ingenic_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct ingenic_gpio_chip *jzgc = gc_to_ingenic_gc(chip);
	unsigned gpio = chip->base + offset;
	pinctrl_gpio_free(gpio);
	jzgc->used_pins_bitmap &= ~(1 << offset);
}

static const struct gpio_chip ingenic_gpiolib_chip = {
	.owner = THIS_MODULE,
	.set = ingenic_gpio_set,
	.get = ingenic_gpio_get,
	.direction_input = ingenic_gpio_direction_input,
	.direction_output = ingenic_gpio_direction_output,
	.to_irq = ingenic_gpio_to_irq,
	.request = ingenic_gpio_request,
	.free = ingenic_gpio_free,
};

static int ingenic_of_gpio_xlate(struct gpio_chip *gc,
                                 const struct of_phandle_args *gpiospec, u32 *flags)
{
	struct ingenic_gpio_chip *jzgc = gc_to_ingenic_gc(gc);
	u32 pin;
	u32 pull;
#if defined(INGENIC_GPIO_FILTER)
	u32 filter;
#endif
	u32 drive_strength = 0;
	u32 schmit_enable = 0;
	u32 slewrate_enable = 0;
	pr_debug("%s %d gpiospec->args_count=%d gc->of_gpio_n_cells=%d \r\n", __func__, __LINE__, gpiospec->args_count, gc->of_gpio_n_cells);
	pr_debug("%s %d gpiospec->args[0]=%d gc->ngpio=%d \r\n", __func__, __LINE__, gpiospec->args[0], gc->ngpio);
	if (WARN_ON(gpiospec->args_count < gc->of_gpio_n_cells)) {
		return -EINVAL;
	}

	if (gpiospec->args[0] >= gc->ngpio) {
		return -EINVAL;
	}

	if (flags) {
		*flags = gpiospec->args[1];
	}

	pin = gpiospec->args[0];

	pull = INGENIC_GPIO_PULL(gpiospec->args[2]);
	ingenic_gpio_set_pull(jzgc, BIT(pin), pull);

	drive_strength = INGENIC_GPIO_DRIVE_STRENGTH(gpiospec->args[2]);
	schmit_enable = INGENIC_GPIO_SCHMITT_ENABLE(gpiospec->args[2]);
	slewrate_enable = INGENIC_GPIO_SLEW_RATE_ENABLE(gpiospec->args[2]);

	ingenic_gpio_set_drive_strength(jzgc, pin, drive_strength);
	ingenic_gpio_set_schmitt_enable(jzgc, pin, schmit_enable);
	ingenic_gpio_set_slew_rate(jzgc, pin, slewrate_enable);

#if defined(INGENIC_GPIO_FILTER)
	filter = INGENIC_GPIO_FILTER(gpiospec->args[2]);
	if (jzgc->filter_bitmap & BIT(pin)) {
		ingenic_gpio_set_filter(jzgc, pin, filter);
	}
#endif
	return pin;
}

static int ingenic_gpio_chip_add(struct ingenic_pinctrl *pctl,
                                 struct device_node *np, int base, int idx)
{
	struct ingenic_gpio_chip *jzgc = &pctl->gpio_chips[idx];
	struct gpio_chip *gc = &jzgc->gc;
	u32 ngpio;

	jzgc->gc = ingenic_gpiolib_chip;
	snprintf(jzgc->name, sizeof(jzgc->name), "GP%c", 'A' + idx);

	if (of_property_read_u32(np, "ingenic,ngpio", &ngpio)) {
		ngpio = MAX_GPIOS_ON_CHIP;
	}
	if (of_property_read_u32(np, "ingenic,filter-bitmap", &jzgc->filter_bitmap)) {
		jzgc->filter_bitmap = 0;
	}
	if (of_property_read_u32(np, "ingenic,pull-bitmap", &jzgc->pull_bitmap)) {
		jzgc->pull_bitmap = 0;
	}
	if (of_property_read_u32(np, "#gpio-cells", &gc->of_gpio_n_cells)) {
		gc->of_gpio_n_cells = 3;
	}
	jzgc->used_pins_bitmap = 0;
	pr_debug("%s (%d) config:\nfilter %08x\npull %08x\ncells=%d\nbase%d\n",
	         jzgc->name,
	         ngpio,
	         jzgc->filter_bitmap,
	         jzgc->pull_bitmap,
	         gc->of_gpio_n_cells,
	         base);
	jzgc->of_node = np;
	jzgc->idx = idx;
	jzgc->pctl = pctl;
	gc->ngpio = (u16)ngpio;
	//gc->of_node = np;
	gc->fwnode = of_fwnode_handle(np);
	gc->base = base;
	gc->parent = pctl->dev;
	gc->label = jzgc->name;
	gc->of_xlate = ingenic_of_gpio_xlate;
	return gpiochip_add(gc);
}

static void ingenic_gpio_sleep_init(struct device_node *np,
                                    const char *list_name, u32 *pm_pin)
{
	int size, i, array[32], pin_bitmap = 0;

	if (!of_find_property(np, list_name, &size)) {
		return;
	}

	if (of_property_read_u32_array(np, list_name,
	                               array, size / sizeof(u32))) {
		return;
	}

	for (i = 0; i < size / sizeof(u32); i++) {
		BUG_ON(array[i] > 31);
		pin_bitmap |= BIT(array[i]);
	}

	*pm_pin = pin_bitmap;
}

static void ingenic_gpio_pm_init(struct ingenic_pinctrl *pctl,
                                 struct device_node *np, int idx)
{
	struct ingenic_gpio_chip *jzgc = &pctl->gpio_chips[idx];

	ingenic_gpio_sleep_init(np, "ingenic,gpio-sleep-low",
	                        &jzgc->pm_bitmap[PM_SLEEP_LOW]);
	ingenic_gpio_sleep_init(np, "ingenic,gpio-sleep-high",
	                        &jzgc->pm_bitmap[PM_SLEEP_HIGH]);
	ingenic_gpio_sleep_init(np, "ingenic,gpio-sleep-pull",
	                        &jzgc->pm_bitmap[PM_SLEEP_PULL]);
	ingenic_gpio_sleep_init(np, "ingenic,gpio-sleep-npul",
	                        &jzgc->pm_bitmap[PM_SLEEP_NOPULL]);
	ingenic_gpio_sleep_init(np, "ingenic,gpio-sleep-pullup",
	                        &jzgc->pm_bitmap[PM_SLEEP_PULLUP]);
	ingenic_gpio_sleep_init(np, "ingenic,gpio-sleep-pulldown",
	                        &jzgc->pm_bitmap[PM_SLEEP_PULLDOWN]);
	ingenic_gpio_sleep_init(np, "ingenic,gpio-sleep-hiz",
	                        &jzgc->pm_bitmap[PM_SLEEP_HIZ]);
	ingenic_gpio_sleep_init(np, "ingenic,gpio-sleep-wakeup",
	                        &jzgc->pm_bitmap[PM_SLEEP_WAKEUP]);

	ingenic_gpio_sleep_init(np, "ingenic,gpio-sleep-func0",
	                        &jzgc->pm_bitmap[PM_SLEEP_FUNC0]);
	ingenic_gpio_sleep_init(np, "ingenic,gpio-sleep-func1",
	                        &jzgc->pm_bitmap[PM_SLEEP_FUNC1]);
	ingenic_gpio_sleep_init(np, "ingenic,gpio-sleep-func2",
	                        &jzgc->pm_bitmap[PM_SLEEP_FUNC2]);
	ingenic_gpio_sleep_init(np, "ingenic,gpio-sleep-func3",
	                        &jzgc->pm_bitmap[PM_SLEEP_FUNC3]);

	if (jzgc->pm_bitmap[PM_SLEEP_WAKEUP]) {
		irq_set_irq_wake(jzgc->irq, true);
	}

	jzgc->wakeup_bitmap = 0;

	/*pr_info("%s pm sleep:\nlow  %08x\nhigh %08x\npull %08x\nnpull %08x\n func0 %08x\nfunc1 %08x\nfunc2 %08x\nfunc3 %08x\n",
	        jzgc->name, jzgc->pm_bitmap[PM_SLEEP_LOW],
	        jzgc->pm_bitmap[PM_SLEEP_HIGH],
	        jzgc->pm_bitmap[PM_SLEEP_PULL],
	        jzgc->pm_bitmap[PM_SLEEP_NOPULL],
	        jzgc->pm_bitmap[PM_SLEEP_FUNC0],
	        jzgc->pm_bitmap[PM_SLEEP_FUNC1],
	        jzgc->pm_bitmap[PM_SLEEP_FUNC2],
	        jzgc->pm_bitmap[PM_SLEEP_FUNC3]);*/

}

static int ingenic_irq_domain_xlate(struct irq_domain *d,
                                    struct device_node *ctrlr,
                                    const u32 *intspec, unsigned int intsize,
                                    unsigned long *out_hwirq, unsigned int *out_type)
{
	struct ingenic_gpio_chip *jzgc = d->host_data;
	u32 pin;
#if defined(INGENIC_GPIO_PULL)
	u32 pull;
#endif
#if defined(INGENIC_GPIO_FILTER)
	u32 filter;
#endif
	if (WARN_ON(intsize < 2)) {
		return -EINVAL;
	}
	pin = *out_hwirq = intspec[0];
	*out_type = (intsize > 1) ? intspec[1] : IRQ_TYPE_NONE;

#if defined(INGENIC_GPIO_PULL)
	pull = INGENIC_GPIO_PULL(intspec[2]);
	ingenic_gpio_set_pull(jzgc, BIT(pin), pull);
#endif

#if defined(INGENIC_GPIO_FILTER)
	filter = INGENIC_GPIO_FILTER(intspec[2]);
	if (jzgc->filter_bitmap & BIT(pin)) {
		ingenic_gpio_set_filter(jzgc, pin, filter);
	}
#endif
	return 0;
}

static int ingenic_irq_domain_map(struct irq_domain *d, unsigned int virq, irq_hw_number_t hw)
{
	struct ingenic_gpio_chip *jzgc = d->host_data;
	irq_set_chip_data(virq, jzgc);
	irq_set_chip_and_handler(virq, &ingenic_gpio_irq_chip,
	                         handle_level_irq);
	return 0;
}

static const struct irq_domain_ops ingenic_irq_domain_ops = {
	.map = ingenic_irq_domain_map,
	.xlate = ingenic_irq_domain_xlate,
};

static int ingenic_gpio_irq_init(struct ingenic_pinctrl *pctl,
                                 struct device_node *np, int idx)
{
	struct ingenic_gpio_chip *jzgc = &pctl->gpio_chips[idx];

	jzgc->irq = irq_of_parse_and_map(np, 0);
	if (!jzgc->irq) {
		return -EINVAL;
	}

	jzgc->irq_domain = irq_domain_add_linear(np, jzgc->gc.ngpio,
	                   &ingenic_irq_domain_ops, (void *)jzgc);
	if (!jzgc->irq_domain) {
		return -ENOMEM;
	}
	irq_set_handler_data(jzgc->irq, jzgc);
	irq_set_chained_handler(jzgc->irq, ingenic_gpio_irq_handler);
	return 0;
}

static int ingenic_gpiolib_register(struct ingenic_pinctrl *pctl)
{
	struct ingenic_gpio_chip *gpio_chips;
	struct device_node *np;
	int idx = 0, ret;
	u32 offset;

	if (!of_property_read_u32(pctl->of_node, "ingenic,regs-offset", &offset)) {
		pctl->group_regs_offset = offset;
	}
	if (of_property_read_u32(pctl->of_node, "ingenic,num-chips", &pctl->num_chips)) {
		return -EINVAL;
	}

	pctl->bitmap_priv = devm_kzalloc(pctl->dev, sizeof(u32) * pctl->num_chips,
	                                 GFP_KERNEL);
	if (!pctl->bitmap_priv) {
		return -ENOMEM;
	}

	gpio_chips = devm_kzalloc(pctl->dev,
	                          pctl->num_chips * sizeof(struct ingenic_gpio_chip),
	                          GFP_KERNEL);
	pctl->gpio_chips = gpio_chips;
	pctl->total_pins = 0;
	spin_lock_init(&pctl->shadow_lock);

	for_each_child_of_node(pctl->of_node, np) {
		if (!of_find_property(np, "gpio-controller", NULL)) {
			continue;
		}

		if (WARN_ON(idx >= pctl->num_chips)) {
			break;
		}

		ret = ingenic_gpio_chip_add(pctl, np, pctl->total_pins, idx);
		if (ret) {
			dev_err(pctl->dev, "%s gpio add failed\n", gpio_chips[idx].name);
			return ret;
		}
		pctl->total_pins += gpio_chips[idx].gc.ngpio;
		spin_lock_init(&gpio_chips[idx].lock);

		if (!of_find_property(np, "interrupt-controller", NULL)) {
			goto next_chip;
		}

		ret = ingenic_gpio_irq_init(pctl, np, idx);
		if (ret)
			dev_err(pctl->dev, "%s irq init failed\n",
			        gpio_chips[idx].gc.label);

		ingenic_gpio_pm_init(pctl, np, idx);

next_chip:
		of_node_get(np);
		idx++;
	}

	dev_info(pctl->dev, "%d gpio chip add success, pins %d\n",
	         idx, pctl->total_pins);
	return 0;
}

static int ingenic_get_group_count(struct pinctrl_dev *pctldev)
{
	struct ingenic_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	return pctl->num_groups;
}

static const char *ingenic_get_group_name(struct pinctrl_dev *pctldev,
        unsigned selector)
{
	struct ingenic_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	BUG_ON(selector >= pctl->num_groups);
	return pctl->groups[selector].name;
}

static int ingenic_get_group_pins(struct pinctrl_dev *pctldev,
                                  unsigned selector,
                                  const unsigned **pins,
                                  unsigned *num_pins)
{
	struct ingenic_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	BUG_ON(selector >= pctl->num_groups);
	*pins = pctl->groups[selector].pins;
	*num_pins = pctl->groups[selector].num_pins;
	return 0;
}

static int ingenic_gc_match(struct gpio_chip *chip, void *data)
{
	return chip->fwnode == data;
}

static int ingenic_packed_config(int cfgval, int *config)
{
	int value;
	char *pinctrl_name = NULL;

	value = PINCFG_UNPACK_VALUE(cfgval);
	switch (cfgval & PINCTL_CFG_TYPE_MSK) {
		case PINCTL_CFG_FILTER:
			*config = pinconf_to_config_packed(PIN_CONFIG_INPUT_DEBOUNCE, value & 0xffff);
			pinctrl_name = PINCTL_NAME(PINCTL_CFG_FILTER);
			break;
		case PINCTL_CFG_DRIVE_STRENGTH:
			pinctrl_name = PINCTL_NAME(PINCTL_CFG_DRIVE_STRENGTH);
			if (gpctl->priv->have_ds) {
				*config = pinconf_to_config_packed(PIN_CONFIG_DRIVE_STRENGTH, value & 0xffff);
				break;
			}
			goto ERR;
		case PINCTL_CFG_BIAS_PULL_PIN_DEFAULT:
			pinctrl_name = PINCTL_NAME(PINCTL_CFG_BIAS_PULL_PIN_DEFAULT);
			if (gpctl->priv->pullen_set) {
				*config = pinconf_to_config_packed(PIN_CONFIG_BIAS_PULL_PIN_DEFAULT, 1);
				break;
			}
			goto ERR;
		case PINCTL_CFG_BIAS_DISABLE:
			pinctrl_name = PINCTL_NAME(PINCTL_CFG_BIAS_DISABLE);
			if (gpctl->priv->pullen_set) {
				*config = pinconf_to_config_packed(PIN_CONFIG_BIAS_DISABLE, 1);
				break;
			}
			goto ERR;
		case PINCTL_CFG_BIAS_HIGH_IMPEDANCE:
			pinctrl_name = PINCTL_NAME(PINCTL_CFG_BIAS_HIGH_IMPEDANCE);
			if (gpctl->priv->pull_tristate) {
				*config = pinconf_to_config_packed(PIN_CONFIG_BIAS_HIGH_IMPEDANCE, 1);
				break;
			}
			goto ERR;
		case PINCTL_CFG_INPUT_SCHMITT_ENABLE:
			pinctrl_name = PINCTL_NAME(PINCTL_CFG_INPUT_SCHMITT_ENABLE);
			if (gpctl->priv->have_schmit) {
				*config = pinconf_to_config_packed(PIN_CONFIG_INPUT_SCHMITT_ENABLE, 1);
				break;
			}
			goto ERR;
		case PINCTL_CFG_SLEW_RATE:
			pinctrl_name = PINCTL_NAME(PINCTL_CFG_SLEW_RATE);
			if (gpctl->priv->have_slew) {
				*config = pinconf_to_config_packed(PIN_CONFIG_SLEW_RATE, 1);
				break;
			}
			goto ERR;
		case PINCTL_CFG_BIAS_PULL_UP:
			pinctrl_name = PINCTL_NAME(PINCTL_CFG_BIAS_PULL_UP);
			if (gpctl->priv->pull_tristate) {
				*config = pinconf_to_config_packed(PIN_CONFIG_BIAS_PULL_UP, 1);
				break;
			}
			goto ERR;
		case PINCTL_CFG_BIAS_PULL_DOWN:
			pinctrl_name = PINCTL_NAME(PINCTL_CFG_BIAS_PULL_DOWN);
			if (gpctl->priv->pull_tristate) {
				*config = pinconf_to_config_packed(PIN_CONFIG_BIAS_PULL_DOWN, 1);
				break;
			}
			goto ERR;
		default:
			*config = 0;
			return -1;
			break;
	}

	/*  printk("%s %s packed ok!!!\n", __func__, pinctrl_name); */
	return 0;

	/* 该pinctrl控制器不支持该属性设置，报错返回 */
ERR:
	printk("%s packed error, this pinctrl does not support this attribute\n",  pinctrl_name);
	*config = 0;
	return -1;
}

#if defined(PINCTL_CFG_TYPES)
static int of_prase_pincfg_to_map(struct device_node *np,
                                  struct pinctrl_map *map, int num_maps)
{
	struct of_phandle_args out_args;
	struct ingenic_gpio_chip *jzgc;
	struct gpio_chip *gc;
	u32 idx = 0, config = 0, index = 0;
	unsigned long pin, bitmap;
	char *pin_names;

	while (!of_parse_phandle_with_args(np, "ingenic,pincfg",
	                                   "#ingenic,pincfg-cells", index++,
	                                   &out_args)) {
		gc = gpiochip_find(of_fwnode_handle(out_args.np), ingenic_gc_match);
		if (!gc) {
			return -EINVAL;
		}
		jzgc = gc_to_ingenic_gc(gc);

		bitmap = pin_bitmap(out_args.args[PIN_ARGS_FROM_INDEX],
		                    out_args.args[PIN_ARGS_TO_INDEX]);

		ingenic_packed_config(out_args.args[PIN_ARGS_CFG_INDEX], &config);
		for_each_set_bit(pin, &bitmap, jzgc->gc.ngpio) {
			pin_names = kzalloc(sizeof(char) * PIN_NAMES_LEN, GFP_KERNEL);
			if (!pin_names) {
				return -ENOMEM;
			}
			sprintf(pin_names, "%s-%d", jzgc->name, (unsigned)pin);
			for (idx = 0; idx < num_maps &&
			     map[idx].data.configs.group_or_pin &&
			     strcmp(pin_names, map[idx].data.configs.group_or_pin);
			     idx++);
			BUG_ON(num_maps == idx);
			if (map[idx].data.configs.num_configs == PINCTL_CFG_TYPES) {
#ifdef __WARN
				__WARN();
#endif
				continue;
			}
			kfree(map[idx].data.configs.group_or_pin);
			map[idx].data.configs.group_or_pin = pin_names;
			map[idx].data.configs.configs[map[idx].data.configs.num_configs] = config;
			map[idx].data.configs.num_configs++;
			map[idx].type = PIN_MAP_TYPE_CONFIGS_PIN;
		}
	}
	return 0;
}
#endif

static int ingenic_dt_node_to_map(struct pinctrl_dev *pctldev,
                                  struct device_node *np_config,
                                  struct pinctrl_map **maps, unsigned *num_maps)
{
	struct ingenic_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct gpio_chip *gc;
	struct ingenic_gpio_chip *jzgc;
	struct device_node *node;
	struct ingenic_pinctrl_group *grp;
	struct ingenic_pinctrl_func *func;
	struct pinctrl_map *new_map;
	unsigned long *cfgs;
	int index = 0, ret = 0;
	unsigned map_cnt = 0, cfg_map_cnt = 0, idx;
	struct of_phandle_args out_args;
	u32 *pincfg_bitmap = pctl->bitmap_priv;

	grp = find_group_by_of_node(pctl, np_config);
	if (!grp) {
		return -EINVAL;
	}

	if ((node = of_get_parent(np_config)) == pctl->of_node) {
		func = find_func_by_of_node(pctl, np_config);
	} else {
		func = find_func_by_of_node(pctl, node);
	}
	of_node_put(node);
	if (!func) {
		return -EINVAL;
	}

#if defined(PINCTL_CFG_TYPES)
	for (idx = 0; idx < pctl->num_chips; idx++) {
		pincfg_bitmap[idx] = 0;
	}
	while (!of_parse_phandle_with_args(np_config, "ingenic,pincfg",
	                                   "#ingenic,pincfg-cells", index++, &out_args)) {
		gc = gpiochip_find(of_fwnode_handle(out_args.np), ingenic_gc_match);
		if (!gc) {
			return -ENOMEM;
		}
		jzgc = gc_to_ingenic_gc(gc);
		pincfg_bitmap[jzgc->idx] |= pin_bitmap(out_args.args[PIN_ARGS_FROM_INDEX],
		                                       out_args.args[PIN_ARGS_TO_INDEX]);
	};
	for (idx = 0; idx < pctl->num_chips; idx++) {
		cfg_map_cnt += bit_count(pincfg_bitmap[idx]);
	}
	map_cnt = cfg_map_cnt;
#endif

	if (of_find_property(np_config, "ingenic,pinmux", NULL) &&
	    of_find_property(np_config, "ingenic,pinmux-funcsel", NULL)) {
		map_cnt++;
	}
	if (!map_cnt) {
		return -EINVAL;
	}

	*maps = new_map = kzalloc(sizeof(*new_map) * map_cnt, GFP_KERNEL);
	if (!new_map) {
		return -ENOMEM;
	}

#if defined(PINCTL_CFG_TYPES)
	if (!cfg_map_cnt) {
		goto skip_config;
	}

	cfgs = kzalloc(sizeof(*cfgs) * PINCTL_CFG_TYPES * cfg_map_cnt, GFP_KERNEL);
	if (!cfgs) {
		return -ENOMEM;
	}

	for (idx = 0; idx < cfg_map_cnt; idx++, cfgs += PINCTL_CFG_TYPES) {
		new_map[idx].data.configs.configs = cfgs;
	}

	ret = of_prase_pincfg_to_map(np_config, new_map, cfg_map_cnt);
	if (ret) {
		return ret;
	}
#endif
skip_config:
	*num_maps = cfg_map_cnt;
	if (!of_find_property(np_config, "ingenic,pinmux", NULL)) {
		return 0;
	}

	jzgc = gc_to_ingenic_gc(grp->gc);
	/* if(jzgc->used_pins_bitmap & grp->pinmux_bitmap) { */
	/*  printk("%s: GP:%s  used_pins_bitmap: 0X%08X\n", __func__, jzgc->name, jzgc->used_pins_bitmap); */
	/*  printk("current set function pin: chip->name %s, gpio: 0X%08X\n", grp->name, grp->pinmux_bitmap); */
	/*  dump_stack(); */
	/*  printk("%s:gpio functions has redefinition\n", __FILE__); */
	/* } */
	jzgc->used_pins_bitmap |= grp->pinmux_bitmap;
	new_map[*num_maps].type = PIN_MAP_TYPE_MUX_GROUP;
	new_map[*num_maps].data.mux.function = func->name;
	new_map[*num_maps].data.mux.group = grp->name;
	(*num_maps)++;
	return 0;
}

static void ingenic_dt_free_map(struct pinctrl_dev *pctldev,
                                struct pinctrl_map *maps, unsigned num_maps)
{
	unsigned long *configs = NULL;
	int idx;
	for (idx = 0; idx < num_maps; idx++) {
		if (maps[idx].type == PIN_MAP_TYPE_CONFIGS_GROUP) {
			if (NULL == configs) {
				configs = maps[idx].data.configs.configs;
			}
			kfree(maps[idx].data.configs.group_or_pin);
		}
	}
	kfree(configs);
	kfree(maps);
}

static const struct pinctrl_ops ingenic_pctl_ops = {
	.get_groups_count       = ingenic_get_group_count,
	.get_group_name         = ingenic_get_group_name,
	.get_group_pins         = ingenic_get_group_pins,
	.dt_node_to_map         = ingenic_dt_node_to_map,   /* TODO: use generic map*/
	.dt_free_map            = ingenic_dt_free_map,
};

static int ingenic_pinmux_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct ingenic_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	return pctl->num_funs;
}

static const char *ingenic_pinmux_get_function_name(struct pinctrl_dev *pctldev,
        unsigned selector)
{
	struct ingenic_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	return pctl->functions[selector].name;
}

static int ingenic_pinmux_get_groups(struct pinctrl_dev *pctldev,
                                     unsigned selector,
                                     const char *const **groups,
                                     unsigned *const num_groups)
{
	struct ingenic_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	BUG_ON(selector > pctl->num_funs);
	*groups = pctl->functions[selector].groups;
	*num_groups = pctl->functions[selector].num_groups;
	return 0;
}

static int ingenic_pinmux_enable(struct pinctrl_dev *pctldev,
                                 unsigned func_selector,
                                 unsigned group_selector)
{
	struct ingenic_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct ingenic_pinctrl_group *grp = NULL;
	const struct ingenic_priv *priv = pctl->priv;
	struct ingenic_gpio_chip *jzgc;

	if (func_selector > pctl->num_funs ||
	    group_selector > pctl->num_groups) {
		return -EINVAL;
	}

	grp = &pctl->groups[group_selector];
	jzgc = gc_to_ingenic_gc(grp->gc);
	if (grp->pinmux_func == GPIO_OUTPUT0) {
		ingenic_gpio_pins_set(jzgc, grp->pinmux_bitmap, 0);
	} else if (grp->pinmux_func == GPIO_OUTPUT1) {
		ingenic_gpio_pins_set(jzgc, grp->pinmux_bitmap, 1);
	}
	ingenic_gpio_set_func(jzgc, priv->have_shadow,
	                      grp->pinmux_func, grp->pinmux_bitmap);
	return 0;
}

static int ingenic_pinmux_gpio_set_dir(struct pinctrl_dev *pctldev,
                                       struct pinctrl_gpio_range *range, unsigned gpio,
                                       bool input)
{
	struct ingenic_gpio_chip *jzgc = gc_to_ingenic_gc(range->gc);
	const struct ingenic_priv *priv = jzgc->pctl->priv;
	unsigned pin = gpio - range->gc->base;
	enum gpio_function func = GPIO_INPUT;

	if (!input) {
		/*
		    这里的GPIO_OUTPUT0 并不一定是输出0，主要用于ingenic_gpio_set_func 识别出是输出Function，
		    实际输出的值由以下函数决定.
		    ingenic_gpio_pins_set(chip, grp->pinmux_bitmap, 0);
		    ingenic_gpio_set(chip, grp->pinmux_bitmap, 0);

		    一旦确定是输出function，在ingenic_gpio_set_func时，不会再设置PAT0寄存器。
		*/
		func = GPIO_OUTPUT0;
	}

	ingenic_gpio_set_func(jzgc, priv->have_shadow, func, BIT(pin));
	return 0;
}

static const struct pinmux_ops ingenic_pinmux_ops = {
	.get_functions_count    = ingenic_pinmux_get_functions_count,
	.get_function_name      = ingenic_pinmux_get_function_name,
	.get_function_groups    = ingenic_pinmux_get_groups,
	.set_mux        = ingenic_pinmux_enable,
	.gpio_set_direction = ingenic_pinmux_gpio_set_dir,
};

static int ingenic_pinconf_get(struct pinctrl_dev *pctldev,
                               unsigned gpio,
                               unsigned long *config)
{

	struct gpio_chip *gc = gpiod_to_chip(gpio_to_desc(gpio));
	struct ingenic_gpio_chip *jzgc = gc_to_ingenic_gc(gc);
	enum pin_config_param param;
	unsigned pin;
	unsigned arg;
	bool pull;

	if (!jzgc) {
		return -EINVAL;
	}

	pin = gpio - jzgc->gc.base;

	param = pinconf_to_config_param(*config);

	switch (param) {
		case PIN_CONFIG_BIAS_PULL_PIN_DEFAULT:
		case PIN_CONFIG_BIAS_DISABLE:
			pull = ingenic_gpio_readl(jzgc, PxPEN)&BIT(pin);
			if ((jzgc->pull_bitmap & BIT(pin)) && pull) {
				*config = pinconf_to_config_packed(PIN_CONFIG_BIAS_PULL_PIN_DEFAULT, 1);
			} else {
				*config = pinconf_to_config_packed(PIN_CONFIG_BIAS_DISABLE, 1);
			}
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			arg = ingenic_gpio_get_drive_strength(jzgc, pin);
			*config = pinconf_to_config_packed(param, arg);
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
		case PIN_CONFIG_BIAS_PULL_DOWN:
		case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
			arg = ingenic_gpio_get_pull_state(jzgc, pin);
			*config = pinconf_to_config_packed(param, arg);
			break;
		case PIN_CONFIG_SLEW_RATE:
			arg = !!(ingenic_gpio_readl(jzgc, PxSR) & BIT(pin));
			*config = pinconf_to_config_packed(param, arg);
			break;
		case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
			arg = !!(ingenic_gpio_readl(jzgc, PxSMT) & BIT(pin));
			*config = pinconf_to_config_packed(param, arg);
			break;
		default:
			return -ENOTSUPP;
			break;
	}
	return 0;
}

static int ingenic_pinconf_set(struct pinctrl_dev *pctldev,
                               unsigned gpio,
                               unsigned long *configs,
                               unsigned num_configs)
{
	struct gpio_chip *gc = gpiod_to_chip(gpio_to_desc(gpio));
	struct ingenic_gpio_chip *jzgc = gc_to_ingenic_gc(gc);
	enum pin_config_param param;
	u16 value;
	unsigned pin;
	int i;

	if (!jzgc) {
		return -EINVAL;
	}

	pin = gpio - jzgc->gc.base;

	dev_dbg(pctldev->dev, "%s %s %u %lx\n", __func__, jzgc->gc.label, pin, *configs);

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		switch (param) {
			case PIN_CONFIG_BIAS_PULL_PIN_DEFAULT:
				if (!(jzgc->pull_bitmap & BIT(pin))) {
					return -EINVAL;
				}
				ingenic_gpio_set_pull(jzgc, BIT(pin), INGENIC_GPIO_PULLEN);
				break;
			case PIN_CONFIG_BIAS_DISABLE:
				if ((!(jzgc->pull_bitmap & BIT(pin)))) {
					return -EINVAL;
				}
				ingenic_gpio_set_pull(jzgc, BIT(pin), !INGENIC_GPIO_PULLEN);
				break;
			case PIN_CONFIG_INPUT_DEBOUNCE:
				value = pinconf_to_config_argument(configs[i]);
				ingenic_gpio_set_filter(jzgc, pin, value);
				break;
			case PIN_CONFIG_DRIVE_STRENGTH:
				value = pinconf_to_config_argument(configs[i]);
				ingenic_gpio_set_drive_strength(jzgc, pin, value);
				break;
			case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
				ingenic_gpio_set_schmitt_enable(jzgc, pin, 1);
				break;
			case PIN_CONFIG_SLEW_RATE:
				ingenic_gpio_set_slew_rate(jzgc, pin, 1);
				break;
			case PIN_CONFIG_BIAS_PULL_UP:
				ingenic_gpio_set_pull(jzgc, BIT(pin), INGENIC_GPIO_PULLUP);
				break;
			case PIN_CONFIG_BIAS_PULL_DOWN:
				ingenic_gpio_set_pull(jzgc, BIT(pin), INGENIC_GPIO_PULLDOWN);
				break;
			case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
				ingenic_gpio_set_pull(jzgc, BIT(pin), INGENIC_GPIO_HIZ);
				break;
			default:
				return -ENOTSUPP;
		}
	}
	return 0;
}

/* set the pin config settings for a specified pin group */
static int ingenic_pinconf_group_set(struct pinctrl_dev *pctldev,
                                     unsigned group, unsigned long *configs,
                                     unsigned num_configs)
{
	struct ingenic_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	struct ingenic_pinctrl_group *grp = NULL;
	const unsigned int *pins;
	unsigned int cnt;

	grp = &pctl->groups[group];
	pins = grp->pins;

	for (cnt = 0; cnt < grp->num_pins; cnt++) {
		ingenic_pinconf_set(pctldev, pins[cnt], configs, num_configs);
	}

	return 0;
}

/* get the pin config settings for a specified pin group */
static int ingenic_pinconf_group_get(struct pinctrl_dev *pctldev,
                                     unsigned int group, unsigned long *config)
{
	struct ingenic_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	const unsigned int *pins;

	pins = pctl->groups[group].pins;
	ingenic_pinconf_get(pctldev, pins[0], config);
	return 0;
}

static const struct pinconf_ops ingenic_pinconf_ops = {
	.pin_config_get     = ingenic_pinconf_get,
	.pin_config_set     = ingenic_pinconf_set,
	.pin_config_group_get   = ingenic_pinconf_group_get,
	.pin_config_group_set   = ingenic_pinconf_group_set,
};

static int ingenic_init_group(struct device *dev, struct device_node *np,
                              struct ingenic_pinctrl_group *grp)
{
	struct device_node *gpio_np = NULL;
	struct of_phandle_args out_args;
	u32 func;
	int pin, idx = 0, index = 0;

	grp->name = np->name;
	grp->of_node = np;

	while (!of_parse_phandle_with_args(np, "ingenic,pinmux",
	                                   "#ingenic,pinmux-cells", index++, &out_args)) {
		if (gpio_np != NULL && out_args.np != gpio_np) {
			return -EINVAL;
		}
		if (of_property_read_u32(np, "ingenic,pinmux-funcsel", &func)) {
			return -EINVAL;
		}
		gpio_np = out_args.np;
		grp->pinmux_bitmap |= pin_bitmap(out_args.args[PIN_ARGS_FROM_INDEX],
		                                 out_args.args[PIN_ARGS_TO_INDEX]);
	}
	if (!gpio_np) {
		return 0;
	}
	grp->gc = gpiochip_find(of_fwnode_handle(gpio_np), ingenic_gc_match);
	if (!grp->gc) {
		return -EINVAL;
	}
	grp->num_pins = bit_count(grp->pinmux_bitmap);
	grp->pins = devm_kzalloc(dev, sizeof(unsigned) * grp->num_pins,
	                         GFP_KERNEL);
	if (!grp->pins) {
		return -ENOMEM;
	}

	switch (func) {
		case PINCTL_FUNCTION0:  grp->pinmux_func = GPIO_FUNC_0; break;
		case PINCTL_FUNCTION1:  grp->pinmux_func = GPIO_FUNC_1; break;
		case PINCTL_FUNCTION2:  grp->pinmux_func = GPIO_FUNC_2; break;
		case PINCTL_FUNCTION3:  grp->pinmux_func = GPIO_FUNC_3; break;
		case PINCTL_FUNCLOLVL:  grp->pinmux_func = GPIO_OUTPUT0; break;
		case PINCTL_FUNCHILVL:  grp->pinmux_func = GPIO_OUTPUT1; break;
		case PINCTL_FUNCINPUT:  grp->pinmux_func = GPIO_INPUT; break;
		case PINCTL_FUNCINPUT_LO: grp->pinmux_func = GPIO_INT_LO; break;
		case PINCTL_FUNCINPUT_HI: grp->pinmux_func = GPIO_INT_HI; break;
		case PINCTL_FUNCINPUT_FE: grp->pinmux_func = GPIO_INT_FE; break;
		case PINCTL_FUNCINPUT_RE: grp->pinmux_func = GPIO_INT_RE; break;
		case PINCTL_FUNCINPUT_RE_FE: grp->pinmux_func = GPIO_INT_RE_FE; break;
		default:
			return -EINVAL;
	}
	for_each_set_bit(pin, (unsigned long *)&grp->pinmux_bitmap, grp->gc->ngpio)
	grp->pins[idx++] = grp->gc->base + pin;
	return 0;
}

static struct ingenic_pinctrl_group *ingenic_pinctrl_create_groups(
    struct device *dev, unsigned *cnt)
{
	struct device_node *np, *subnp;
	struct ingenic_pinctrl_group *grp;
	unsigned int i = 0;

	*cnt = 0;
	for_each_child_of_node(dev->of_node, np) {
		unsigned count = 0;
		if (!!of_find_property(np, "gpio-controller", NULL)) {
			continue;
		}
		if (!!(count = of_get_child_count(np))) {
			*cnt += count;
		} else {
			(*cnt)++;
		}
	}

	grp = devm_kzalloc(dev, sizeof(*grp) * (*cnt), GFP_KERNEL);
	if (!grp) {
		return ERR_PTR(-ENOMEM);
	}

	for_each_child_of_node(dev->of_node, np) {
		if (!!of_find_property(np, "gpio-controller", NULL)) {
			continue;
		}

		if (!of_get_child_count(np)) {
			ingenic_init_group(dev, np, &grp[i++]);
			continue;
		}

		for_each_child_of_node(np, subnp) {
			ingenic_init_group(dev, subnp, &grp[i++]);
		}
	}
	return grp;
}

static struct ingenic_pinctrl_func *ingenic_pinctrl_create_functions(
    struct device *dev, unsigned int *cnt)
{
	struct device_node *np, *subnp;
	struct ingenic_pinctrl_func *func;
	int i = 0;

	*cnt = 0;
	for_each_child_of_node(dev->of_node, np) {
		if (!!of_find_property(np, "gpio-controller", NULL)) {
			continue;
		}
		(*cnt)++;
	}

	func = devm_kzalloc(dev, sizeof(*func) * (*cnt), GFP_KERNEL);
	if (!func) {
		return ERR_PTR(-ENOMEM);
	}

	for_each_child_of_node(dev->of_node, np) {
		u8 num_child, j = 0;
		if (!!of_find_property(np, "gpio-controller", NULL)) {
			continue;
		}
		func[i].name = np->name;
		func[i].of_node = np;
		num_child = of_get_child_count(np);
		func[i].num_groups = num_child ? : 1;
		func[i].groups = devm_kzalloc(dev, sizeof(char *) * func[i].num_groups,
		                              GFP_KERNEL);
		if (!func[i].groups) {
			return ERR_PTR(-ENOMEM);
		}
		if (num_child) {
			for_each_child_of_node(np, subnp)
			func[i].groups[j++] = subnp->name;
		} else {
			func[i].groups[0] = func[i].name;
		}
		i++;
	}
	return func;
}

static int ingenic_pinctrl_parse_dt(struct ingenic_pinctrl *pctl)
{
	struct ingenic_pinctrl_group *groups;
	struct ingenic_pinctrl_func *functions;
	unsigned int grp_cnt = 0, func_cnt = 0;

	groups = ingenic_pinctrl_create_groups(pctl->dev, &grp_cnt);
	if (IS_ERR(groups)) {
		dev_err(pctl->dev, "failed to parse pin groups\n");
		return PTR_ERR(groups);
	}

	functions = ingenic_pinctrl_create_functions(pctl->dev, &func_cnt);
	if (IS_ERR(functions)) {
		dev_err(pctl->dev, "failed to parse pin functions\n");
		return PTR_ERR(groups);
	}

	pctl->groups = groups;
	pctl->num_groups = grp_cnt;
	pctl->functions = functions;
	pctl->num_funs = func_cnt;
	return 0;
}

static int ingenic_pinctrl_register(struct ingenic_pinctrl *pctl)
{
	struct pinctrl_desc *pctl_desc = &pctl->pctl_desc;
	struct pinctrl_pin_desc *pindesc, *pdesc;
	char *pin_names;
	unsigned chip, pin;
	int ret;

	pctl_desc->name = "ingenic pinctrl";
	pctl_desc->owner = THIS_MODULE;
	pctl_desc->pctlops = &ingenic_pctl_ops;
	pctl_desc->pmxops = &ingenic_pinmux_ops;
	pctl_desc->confops = &ingenic_pinconf_ops;
	pctl_desc->npins = pctl->total_pins;
	pindesc = devm_kzalloc(pctl->dev,
	                       sizeof(*pindesc) * pctl_desc->npins, GFP_KERNEL);
	if (!pindesc) {
		return -ENOMEM;
	}
	pctl_desc->pins = pindesc;

	for (pin = 0, pdesc = pindesc; pin < pctl_desc->npins; pin++, pdesc++) {
		pdesc->number = pin;
	}

	pin_names = devm_kzalloc(pctl->dev,
	                         sizeof(char) * PIN_NAMES_LEN * pctl_desc->npins,
	                         GFP_KERNEL);
	if (!pin_names) {
		return -ENOMEM;
	}

	for (chip = 0; chip < pctl->num_chips; chip++) {
		struct ingenic_gpio_chip *jzgc = &pctl->gpio_chips[chip];
		for (pin = 0; pin < jzgc->gc.ngpio; pin++) {
			sprintf(pin_names, "%s-%d", jzgc->name, pin);
			pdesc = pindesc++;
			pdesc->name = pin_names;
			pin_names += PIN_NAMES_LEN;
		}
	}

	ret = ingenic_pinctrl_parse_dt(pctl);
	if (ret) {
		return ret;
	}

	pctl->pctl_dev = pinctrl_register(pctl_desc, pctl->dev, pctl);
	if (!pctl->pctl_dev) {
		return -EINVAL;
	}

	for (chip = 0; chip < pctl->num_chips; chip++) {
		struct ingenic_gpio_chip *jzgc = &pctl->gpio_chips[chip];

		jzgc->grange.name = jzgc->name;
		jzgc->grange.id = jzgc->idx;
		jzgc->grange.pin_base = jzgc->gc.base;
		jzgc->grange.base = jzgc->gc.base;
		jzgc->grange.npins = jzgc->gc.ngpio;
		jzgc->grange.gc = &jzgc->gc;
		pinctrl_add_gpio_range(pctl->pctl_dev, &jzgc->grange);
	}

	return 0;
}

#ifdef CONFIG_PINCTRL_DUMP
const unsigned char *func_map[] = {"Fun0", "Fun1", "Fun2", "Fun3", "Out0", "Out1", "Inpt", "Inpt", \
                                   "IntL", "IntH", "IntF", "IntR", "ItLM", "ItHM", "ItFM", "ItRM", "IntD", "ItDM"
                                  };

static ssize_t dump_gpio(struct device *dev,
                         struct device_attribute *attr,
                         char *buf)
{
	struct ingenic_pinctrl *pctl = dev_get_drvdata(dev);
	struct ingenic_gpio_chip *gpio_chips = pctl->gpio_chips;
	const struct ingenic_priv *priv = pctl->priv;
	int idx = 0;
	int i = 0;
	ssize_t len = 0;

	/*1. Table Header */

	len += snprintf(buf + len, PAGE_SIZE - len, "REG \t|");
	for (idx = 0;  idx < pctl->num_chips; idx++)
		len += snprintf(buf + len, PAGE_SIZE - len, "+++GP%c++|",
		                'A' + idx);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n\n");

	/*2. INL Level*/
	len += snprintf(buf + len, PAGE_SIZE - len, "INL \t|");
	for (idx = 0;  idx < pctl->num_chips; idx++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%08x|",
		                ingenic_gpio_readl(&gpio_chips[idx], PxPIN));
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	/*3. INT register*/
	len += snprintf(buf + len, PAGE_SIZE - len, "INT \t|");
	for (idx = 0;  idx < pctl->num_chips; idx++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%08x|",
		                ingenic_gpio_readl(&gpio_chips[idx], PxINT));
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	/*4. MSK register*/
	len += snprintf(buf + len, PAGE_SIZE - len, "MSK \t|");
	for (idx = 0;  idx < pctl->num_chips; idx++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%08x|",
		                ingenic_gpio_readl(&gpio_chips[idx], PxMSK));
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	/*5. PAT1 Register*/
	len += snprintf(buf + len, PAGE_SIZE - len, "PAT1 \t|");
	for (idx = 0;  idx < pctl->num_chips; idx++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%08x|",
		                ingenic_gpio_readl(&gpio_chips[idx], PxPAT1));
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	/*6. PAT0 Register*/
	len += snprintf(buf + len, PAGE_SIZE - len, "PAT0 \t|");
	for (idx = 0;  idx < pctl->num_chips; idx++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%08x|",
		                ingenic_gpio_readl(&gpio_chips[idx], PxPAT0));
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	/*7. EGE Register*/
	if (priv->dual_edge_interrupt) {
		len += snprintf(buf + len, PAGE_SIZE - len, "EDG \t|");
		for (idx = 0;  idx < pctl->num_chips; idx++)
			len += snprintf(buf + len, PAGE_SIZE - len, "%08x|",
			                ingenic_gpio_readl(&gpio_chips[idx], PxEDG));
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}

	/*7. PULL/PULLUP/PULLDOWN */
	if (!priv->pull_tristate) {
		len += snprintf(buf + len, PAGE_SIZE - len, "PULL \t|");
		for (idx = 0;  idx < pctl->num_chips; idx++)
			len += snprintf(buf + len, PAGE_SIZE - len, "%08x|",
			                ingenic_gpio_readl(&gpio_chips[idx], PxPEN));
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "PLUP \t|");
		for (idx = 0;  idx < pctl->num_chips; idx++)
			len += snprintf(buf + len, PAGE_SIZE - len, "%08x|",
			                ingenic_gpio_readl(&gpio_chips[idx], PxPU));
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");

		len += snprintf(buf + len, PAGE_SIZE - len, "PLDWN \t|");
		for (idx = 0;  idx < pctl->num_chips; idx++)
			len += snprintf(buf + len, PAGE_SIZE - len, "%08x|",
			                ingenic_gpio_readl(&gpio_chips[idx], PxPD));
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	}
	/*8. FLAG register */
	len += snprintf(buf + len, PAGE_SIZE - len, "FLAG \t|");
	for (idx = 0;  idx < pctl->num_chips; idx++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%08x|",
		                ingenic_gpio_readl(&gpio_chips[idx], PxFLG));
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	if (priv->dump_filter) {
		len += priv->dump_filter(pctl, buf + len, PAGE_SIZE - len);
	}

	// 0 ~ 32 pins
	for (i = 0; i < 32; i++) {
		// PA PB PC PD PE

		for (idx = 0; idx < pctl->num_chips; idx++) {
			unsigned int pxint = ingenic_gpio_readl(&gpio_chips[idx], PxINT);
			unsigned int pxmsk = ingenic_gpio_readl(&gpio_chips[idx], PxMSK);
			unsigned int pxpat1 = ingenic_gpio_readl(&gpio_chips[idx], PxPAT1);
			unsigned int pxpat0 = ingenic_gpio_readl(&gpio_chips[idx], PxPAT0);
			unsigned int level = ingenic_gpio_readl(&gpio_chips[idx], PxPIN);
			unsigned int edg = 0;
			unsigned int func = ((pxint >> i) & 1) << 3 |
			                    ((pxmsk >> i) & 1) << 2 |
			                    ((pxpat1 >> i) & 1) << 1 |
			                    ((pxpat0 >> i) & 1) << 0;
			unsigned pin = (level >> i) & 1;

			if (priv->dual_edge_interrupt) {
				edg = ingenic_gpio_readl(&gpio_chips[idx], PxEDG);
			}

			func = ((edg >> i) & 1) == 1 ? func + 1 : func;

			len += sprintf(buf + len, "P%c%02d:%4s,%-4s    ", 'A' + idx, i, func_map[func], pin ? "High" : "Low");
		}

		len += sprintf(buf + len, "\n");
	}

	return len;
}
static DEVICE_ATTR(dump_gpio, 0444, dump_gpio, NULL);

static struct attribute *ingenic_pinctrl_attrs[] = {
	&dev_attr_dump_gpio.attr,
	NULL,
};

static const struct attribute_group ingenic_pinctrl_attr_group = {
	.attrs = (struct attribute **)ingenic_pinctrl_attrs,
};
#endif

static int ingenic_pinctrl_probe(struct platform_device *pdev)
{
	struct ingenic_pinctrl *pctl;
	const struct of_device_id *match;
	struct resource *res;
	int ret;

	pctl = devm_kzalloc(&pdev->dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl) {
		return -ENOMEM;
	}
	match = of_match_node(ingenic_pinctrl_dt_match, pdev->dev.of_node);
	if (!match) {
		return -ENODEV;
	}

	pctl->priv = !match->data ? common_priv_data : (struct ingenic_priv *)match->data;
	pctl->dev = &pdev->dev;
	pctl->of_node = pdev->dev.of_node;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		return -ENOMEM;
	}

	pctl->io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pctl->io_base)) {
		return PTR_ERR(pctl->io_base);
	}

	platform_set_drvdata(pdev, pctl);

	ret = ingenic_gpiolib_register(pctl);
	if (ret) {
		return ret;
	}

	ret = ingenic_pinctrl_register(pctl);
	if (ret) {
		return ret;
	}

#ifdef CONFIG_PINCTRL_DUMP
	ret = sysfs_create_group(&pdev->dev.kobj, &ingenic_pinctrl_attr_group);
	if (ret) {
		dev_info(&pdev->dev, "ingenic pinctrl attr create failed\n");
	}
#endif

#define CPM_EXCLK_DS 0xb00000E0
	if (pctl->priv->voltage_switchable) {
		u32 voltage;
		ret = of_property_read_u32(pdev->dev.of_node, "ingenic,gpa_voltage", &voltage);
		if (!ret) {
#ifdef CONFIG_SOC_X2500
#define GPIO_PA_GPDCR 0xB0010100
			if (voltage == GPIO_VOLTAGE_1V8) {
				*(volatile unsigned int *)(GPIO_PA_GPDCR) |= (0x1);
			}
			if (voltage == GPIO_VOLTAGE_3V3) {
				*(volatile unsigned int *)(GPIO_PA_GPDCR) &= ~(1);
			}
#else
			if (voltage == GPIO_VOLTAGE_1V8) {
				*(volatile unsigned int *)CPM_EXCLK_DS |= (1 << 30);
			}
			if (voltage == GPIO_VOLTAGE_3V3) {
				*(volatile unsigned int *)CPM_EXCLK_DS &= ~(1 << 30);
			}
#endif
		}
		ret = of_property_read_u32(pdev->dev.of_node, "ingenic,gpe_msc_voltage", &voltage);
		if (!ret) {
			if (voltage == GPIO_VOLTAGE_1V8) {
				*(volatile unsigned int *)CPM_EXCLK_DS |= (1 << 31);
			}
			if (voltage == GPIO_VOLTAGE_3V3) {
				*(volatile unsigned int *)CPM_EXCLK_DS &= ~(1 << 31);
			}
		}
	}

	gpctl = pctl;

	dev_info(&pdev->dev, "ingenic pinctrl probe success\n");
	return 0;
}

unsigned long ingenic_pinctrl_lock(int port)
{
	unsigned long flags;
	struct ingenic_gpio_chip *chip;

	if (port < 0 || port >= gpctl->num_chips) {
		return 0;
	}

	chip = &gpctl->gpio_chips[port];

	spin_lock_irqsave(&chip->lock, flags);

	return flags;
}

unsigned long ingenic_pinctrl_unlock(int port, unsigned long flags)
{
	struct ingenic_gpio_chip *chip;

	if (port < 0 || port >= gpctl->num_chips) {
		return 0;
	}

	chip = &gpctl->gpio_chips[port];

	spin_unlock_irqrestore(&chip->lock, flags);

	return flags;
}

#ifdef CONFIG_PM_SLEEP
#ifdef CONFIG_PINCTRL_DUMP
static char dump_buf[PAGE_SIZE * 2];
void dump_pinctl(void)
{
	struct ingenic_pinctrl *pctl = gpctl;
	char *buf = dump_buf;
	int cnt = 0;
	int ret = 0;
	cnt = dump_gpio(pctl->dev, NULL, buf);

	do {
		ret = printk("%s", buf);
		if (ret == 0) {
			break;
		}

		buf += ret - 1;
		cnt -= ret;
	} while (cnt > 0);

	//ret1 = printk("%s\n", buf);
}
#else
static void dump_pinctl(void) {}
#endif

static int ingenic_gpio_suspend_set(struct ingenic_gpio_chip *jzgc)
{
	const struct ingenic_priv *priv = jzgc->pctl->priv;
	u32 pm_pull = 0, pm_nopull = 0, pm_pullup = 0, pm_pulldown = 0, pm_hiz = 0;
	u32 pm_high = 0, pm_low = 0, pm_wakeup = 0;

	u32 func0 = 0, func1 = 0, func2 = 0, func3 = 0;

	/* save status from registers. */
	jzgc->pm_bitmap[PM_RESUME_INT] = ingenic_gpio_readl(jzgc, PxINT);
	jzgc->pm_bitmap[PM_RESUME_MSK] = ingenic_gpio_readl(jzgc, PxMSK);
	jzgc->pm_bitmap[PM_RESUME_PAT0] = ingenic_gpio_readl(jzgc, PxPAT0);
	jzgc->pm_bitmap[PM_RESUME_PAT1] = ingenic_gpio_readl(jzgc, PxPAT1);

	/* 关闭唤醒源中断. */
	if (jzgc->irq_bitmap & ~jzgc->wakeup_bitmap) {
		ingenic_gpio_writel(jzgc, PxMSKS, jzgc->irq_bitmap & ~jzgc->wakeup_bitmap);
	}
	/* 打开唤醒源中断. */
	if (jzgc->wakeup_bitmap) {
		ingenic_gpio_writel(jzgc, PxMSKC, jzgc->wakeup_bitmap);
	}

	if (!priv->pull_tristate) {
		jzgc->pm_bitmap[PM_RESUME_PULL] = ingenic_gpio_readl(jzgc, PxPEN);
		pm_pull = jzgc->pm_bitmap[PM_SLEEP_PULL] & (~(jzgc->wakeup_bitmap));
		pm_nopull = jzgc->pm_bitmap[PM_SLEEP_NOPULL] & (~(jzgc->wakeup_bitmap));
	} else {
		jzgc->pm_bitmap[PM_RESUME_PULLUP] = ingenic_gpio_readl(jzgc, PxPU);
		jzgc->pm_bitmap[PM_RESUME_PULLDOWN] = ingenic_gpio_readl(jzgc, PxPD);

		pm_pullup = jzgc->pm_bitmap[PM_SLEEP_PULLUP] & (~(jzgc->wakeup_bitmap));
		pm_pulldown = jzgc->pm_bitmap[PM_SLEEP_PULLDOWN] & (~(jzgc->wakeup_bitmap));
	}

	pm_high = jzgc->pm_bitmap[PM_SLEEP_HIGH] & (~(jzgc->wakeup_bitmap));
	pm_low  = jzgc->pm_bitmap[PM_SLEEP_LOW] & (~(jzgc->wakeup_bitmap));
	pm_hiz  = jzgc->pm_bitmap[PM_SLEEP_HIZ] & (~(jzgc->wakeup_bitmap));
	pm_wakeup = jzgc->pm_bitmap[PM_SLEEP_WAKEUP] & (~(jzgc->wakeup_bitmap));

	func0   = jzgc->pm_bitmap[PM_SLEEP_FUNC0] & (~(jzgc->wakeup_bitmap));
	func1   = jzgc->pm_bitmap[PM_SLEEP_FUNC1] & (~(jzgc->wakeup_bitmap));
	func2   = jzgc->pm_bitmap[PM_SLEEP_FUNC2] & (~(jzgc->wakeup_bitmap));
	func3   = jzgc->pm_bitmap[PM_SLEEP_FUNC3] & (~(jzgc->wakeup_bitmap));

	if (pm_pull) {
		ingenic_gpio_set_func(jzgc, priv->have_shadow, GPIO_INPUT, pm_pull);
		ingenic_gpio_set_pull(jzgc, pm_pull, INGENIC_GPIO_PULLEN);
	}
	if (pm_nopull) {
		ingenic_gpio_set_func(jzgc, priv->have_shadow, GPIO_INPUT, pm_nopull);
		ingenic_gpio_set_pull(jzgc, pm_nopull, 0);
	}

	if (pm_pullup) {
		ingenic_gpio_set_func(jzgc, priv->have_shadow, GPIO_INPUT, pm_pullup);
		ingenic_gpio_set_pull(jzgc, pm_pullup, INGENIC_GPIO_PULLUP);
	}
	if (pm_pulldown) {
		ingenic_gpio_set_func(jzgc, priv->have_shadow, GPIO_INPUT, pm_pulldown);
		ingenic_gpio_set_pull(jzgc, pm_pulldown, INGENIC_GPIO_PULLDOWN);
	}
	if (pm_hiz) {
		ingenic_gpio_set_func(jzgc, priv->have_shadow, GPIO_INPUT, pm_hiz);
		ingenic_gpio_set_pull(jzgc, pm_hiz, INGENIC_GPIO_HIZ);
	}

	if (pm_high) {
		ingenic_gpio_pins_set(jzgc, pm_high, 1);
		ingenic_gpio_set_func(jzgc, priv->have_shadow, GPIO_OUTPUT1, pm_high);
	}
	if (pm_low) {
		ingenic_gpio_pins_set(jzgc, pm_high, 0);
		ingenic_gpio_set_func(jzgc, priv->have_shadow, GPIO_OUTPUT0, pm_low);
	}

	if (func0) {
		ingenic_gpio_set_func(jzgc, priv->have_shadow, GPIO_FUNC_0, func0);
	}
	if (func1) {
		ingenic_gpio_set_func(jzgc, priv->have_shadow, GPIO_FUNC_1, func1);
	}
	if (func2) {
		ingenic_gpio_set_func(jzgc, priv->have_shadow, GPIO_FUNC_2, func2);
	}
	if (func3) {
		ingenic_gpio_set_func(jzgc, priv->have_shadow, GPIO_FUNC_3, func3);
	}

	if (pm_wakeup) {
		ingenic_gpio_set_func(jzgc, priv->have_shadow, GPIO_INT_RE_FE, pm_wakeup);
	}

	return 0;
}

static int ingenic_pinctrl_suspend(void)
{
	struct ingenic_pinctrl *pctl = gpctl;
	int idx;

	if (!gpctl) {
		return 0;
	}
	dump_pinctl();
	for (idx = 0; idx < pctl->num_chips; idx++) {
		struct ingenic_gpio_chip *chip = &pctl->gpio_chips[idx];
		ingenic_gpio_suspend_set(chip);
		chip->resume_pending = 0;
	}

	dump_pinctl();
	return 0;
}

static void ingenic_check_wakeup_gpio(struct ingenic_gpio_chip *jzgc)
{
	u32 pend, mask;

	pend = ingenic_gpio_readl(jzgc, PxFLG);
	mask = ingenic_gpio_readl(jzgc, PxMSK);
	pend = pend & ~mask;
	pend = pend & jzgc->wakeup_bitmap;
	if (pend) {
		jzgc->resume_pending = pend;
		jzgc->sleep_level = ingenic_gpio_readl(jzgc, PxPAT0) & pend;
	}
}

static void ingenic_pinctrl_resume(void)
{
	struct ingenic_pinctrl *pctl = gpctl;
	int idx;

	if (!gpctl) {
		return;
	}
	for (idx = 0; idx < pctl->num_chips; idx++) {
		struct ingenic_gpio_chip *jzgc = &pctl->gpio_chips[idx];
		const struct ingenic_priv *priv = jzgc->pctl->priv;
		ingenic_check_wakeup_gpio(jzgc);
		ingenic_gpio_writel(jzgc, PxINT, jzgc->pm_bitmap[PM_RESUME_INT]);
		ingenic_gpio_writel(jzgc, PxMSK, jzgc->pm_bitmap[PM_RESUME_MSK]);
		ingenic_gpio_writel(jzgc, PxPAT0, jzgc->pm_bitmap[PM_RESUME_PAT0]);
		ingenic_gpio_writel(jzgc, PxPAT1, jzgc->pm_bitmap[PM_RESUME_PAT1]);
		if (!priv->pull_tristate) {
			ingenic_gpio_writel(jzgc, PxPEN, jzgc->pm_bitmap[PM_RESUME_PULL]);
		} else {
			ingenic_gpio_writel(jzgc, PxPU, jzgc->pm_bitmap[PM_RESUME_PULLUP]);
			ingenic_gpio_writel(jzgc, PxPD, jzgc->pm_bitmap[PM_RESUME_PULLDOWN]);
		}
	}
}
#else
#define ingenic_pinctrl_suspend NULL
#define ingenic_pinctrl_resume  NULL
#endif

static struct syscore_ops ingenic_pinctrl_syscore_ops = {
	.suspend = ingenic_pinctrl_suspend,
	.resume = ingenic_pinctrl_resume,
};

static const struct ingenic_priv common_priv_data[] = {
	{
		.have_shadow = true,
		.pullen_set = false,
	},
};

static const struct ingenic_priv x1800_priv_data[] = {
	{
		.have_shadow = true,
		.pullen_set = true,
	},
};

static const struct ingenic_priv x1021_priv_data[] = {
	{
		.have_shadow = true,
		.pullen_set = true,
	},
};

static const struct ingenic_priv x1520_priv_data[] = {
	{
		.have_shadow = true,
		.pullen_set = true,
	},
};

static const struct ingenic_priv x1600_priv_data[] = {
	{
		.have_shadow = true,
		.pullen_set = true,
		.dual_edge_interrupt = true,
	},
};

static const struct ingenic_priv x1630_priv_data[] = {
	{
		.have_shadow = true,
		.pullen_set = true,
	},
};

static const struct ingenic_priv x2000_priv_data[] = {
	{
		.have_shadow = true,
		.pull_tristate = true,
		.voltage_switchable = true,
		.dual_edge_interrupt = true,
		.have_ds = true,    // support driver strength settings.
		.have_slew = true,
		.have_schmit = true,
	},
};

static const struct ingenic_priv x2500_priv_data[] = {
	{
		.have_shadow = true,
		.pull_tristate = true,
		.voltage_switchable = true,
		.have_ds = true,    // support drive strength settings.
		.have_slew = true,
		.have_schmit = true,
	},
};

static const struct ingenic_priv m300_priv_data[] = {
	{
		.have_shadow = true,
		.pull_tristate = true,
		.voltage_switchable = true,
		.dual_edge_interrupt = true,
		.have_ds = true,
	},
};

static const struct ingenic_priv x2600_priv_data[] = {
	{
		.have_shadow = true,
		.pull_tristate = true,
		.voltage_switchable = true,
		.dual_edge_interrupt = true,
		.have_ds = true,    // support driver strength settings.
		.have_slew = true,
		.have_schmit = true,
	},
};

static const struct ingenic_priv ad100_priv_data[] = {
	{
		.have_shadow = true,
		.pull_tristate = true,
		.voltage_switchable = true,
		.dual_edge_interrupt = true,
		.have_ds = true,    // support driver strength settings.
		.have_slew = true,
		.have_schmit = true,
	},
};

static const struct of_device_id ingenic_pinctrl_dt_match[] = {
	{ .compatible = "ingenic,x1000-pinctrl", .data = (void *)common_priv_data},
	{ .compatible = "ingenic,x1800-pinctrl", .data = (void *)x1800_priv_data},
	{ .compatible = "ingenic,x1021-pinctrl", .data = (void *)x1021_priv_data},
	{ .compatible = "ingenic,x1520-pinctrl", .data = (void *)x1520_priv_data},
	{ .compatible = "ingenic,x1600-pinctrl", .data = (void *)x1600_priv_data},
	{ .compatible = "ingenic,x1630-pinctrl", .data = (void *)x1630_priv_data},
	{ .compatible = "ingenic,x2000-pinctrl", .data = (void *)x2000_priv_data},
	{ .compatible = "ingenic,m300-pinctrl", .data = (void *)m300_priv_data},
	{ .compatible = "ingenic,x2500-pinctrl", .data = (void *)x2500_priv_data},
	{ .compatible = "ingenic,x2600-pinctrl", .data = (void *)x2600_priv_data},
	{ .compatible = "ingenic,ad100-pinctrl", .data = (void *)ad100_priv_data},
	{},
};
MODULE_DEVICE_TABLE(of, ingenic_pinctrl_dt_match);

static struct platform_driver ingenic_pinctrl_drv = {
	.probe = ingenic_pinctrl_probe,
	.driver = {
		.name = "ingenic pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ingenic_pinctrl_dt_match),
	},
};

static int __init ingenic_pinctrl_drv_init(void)
{
	register_syscore_ops(&ingenic_pinctrl_syscore_ops);
	return platform_driver_register(&ingenic_pinctrl_drv);
}
postcore_initcall(ingenic_pinctrl_drv_init);

static void __exit ingenic_pinctrl_drv_exit(void)
{
	unregister_syscore_ops(&ingenic_pinctrl_syscore_ops);
	platform_driver_unregister(&ingenic_pinctrl_drv);
	return;
}
module_exit(ingenic_pinctrl_drv_exit);

int mcu_gpio_register(unsigned int ggpio, unsigned int reg)
{
	struct ingenic_pinctrl *pctl = gpctl;
	struct ingenic_gpio_chip *jzgc = NULL;
	if (!gpctl) {
		return -ENODEV;
	}
	jzgc = &pctl->gpio_chips[ggpio];
	jzgc->mcu_gpio_reg = (unsigned int *)reg;
	return 0;
}
EXPORT_SYMBOL_GPL(mcu_gpio_register);

void mcu_gpio_unregister(unsigned int ggpio)
{
	struct ingenic_pinctrl *pctl = gpctl;
	struct ingenic_gpio_chip *jzgc = NULL;

	if (!gpctl) {
		return;
	}
	jzgc = &pctl->gpio_chips[ggpio];
	jzgc->mcu_gpio_reg = NULL;
}
EXPORT_SYMBOL_GPL(mcu_gpio_unregister);

MODULE_AUTHOR("cli <chen.li@ingenic.com>");
MODULE_DESCRIPTION("Ingenic pinctrl driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pinctrl");
