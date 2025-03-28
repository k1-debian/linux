/*
 * Copyright (C) 2014 Ingenic Semiconductor Co., Ltd.
 *
 *  Author: dongsheng.qiu, dongsheng.qiu@ingenic.com bo.liu@ingenic.com
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>

#include <dt-bindings/interrupt-controller/mips-irq.h>

#include <asm/irq_cpu.h>
#include <asm/mipsregs.h>
#include <asm/mipsmtregs.h>
#include <asm/setup.h>

static inline void unmask_ingenic_irq(struct irq_data *d)
{
	set_c0_status(0x100 << (d->irq - MIPS_CPU_IRQ_BASE));
#ifdef CONFIG_SMP
	ingenic_irq_cpumask_idle(1);
#endif
}

static inline void mask_ingenic_irq(struct irq_data *d)
{
	clear_c0_status(0x100 << (d->irq - MIPS_CPU_IRQ_BASE));
#ifdef CONFIG_SMP
	ingenic_irq_cpumask_idle(0);
	if (d->irq != CORE_INTC_IRQ) {
		ingenic_irq_migration(1);
	}
#endif
}

static struct irq_chip ingenic_cpu_irq_controller = {
	.name       = "XBurst2",
	.irq_ack    = mask_ingenic_irq,
	.irq_mask   = mask_ingenic_irq,
	.irq_mask_ack   = mask_ingenic_irq,
	.irq_unmask = unmask_ingenic_irq,
	.irq_eoi    = unmask_ingenic_irq,
	.irq_disable    = mask_ingenic_irq,
	.irq_enable = unmask_ingenic_irq,
};

asmlinkage void __weak plat_irq_dispatch(void)
{
	unsigned int status = read_c0_status();
	unsigned int cause = read_c0_cause();

	unsigned long pending = ((status & cause) >> 8) & 0xff;

	if (!pending) {
		spurious_interrupt();
		return;
	}

	if (pending) {
		if (pending & 8) {          /* IPI */
			do_IRQ(MIPS_CPU_IRQ_BASE + 3);
		} else if (pending & 16) {  /* OST */
			do_IRQ(MIPS_CPU_IRQ_BASE + 4);
		} else if (pending & 4) {   /* INTC */
			do_IRQ(MIPS_CPU_IRQ_BASE + 2);
		} else {        /* Others */
			do_IRQ(MIPS_CPU_IRQ_BASE + __ffs(pending));
		}

	} else {
		//printk("IRQ Error, cpu: %d Cause:0x%08lx, Status:0x%08lx\n", smp_processor_id(), cause, status);
	}
}

static int ingenic_cpu_intc_map(struct irq_domain *d, unsigned int irq,
                                irq_hw_number_t hw)
{
	static struct irq_chip *chip;

	chip = &ingenic_cpu_irq_controller;

	irq_set_percpu_devid(irq);
	irq_set_chip_and_handler(irq, chip, handle_percpu_devid_irq);

	return 0;
}

static const struct irq_domain_ops ingenic_cpu_intc_irq_domain_ops = {
	.map = ingenic_cpu_intc_map,
	.xlate = irq_domain_xlate_onecell,
};

static void __init __ingenic_cpu_irq_init(struct device_node *of_node)
{
	struct irq_domain *domain;

	/* Mask interrupts. */
	clear_c0_status(ST0_IM);
	clear_c0_cause(CAUSEF_IP);

	domain = irq_domain_add_legacy(of_node, 8, MIPS_CPU_IRQ_BASE, 0,
	                               &ingenic_cpu_intc_irq_domain_ops, NULL);
	if (!domain) {
		panic("Failed to add irqdomain for MIPS CPU");
	}
}

void __init ingenic_cpu_irq_init(void)
{
	__ingenic_cpu_irq_init(NULL);
}

int __init ingenic_cpu_irq_of_init(struct device_node *of_node,
                                   struct device_node *parent)
{
	__ingenic_cpu_irq_init(of_node);
	return 0;
}
IRQCHIP_DECLARE(cpu_intc, "ingenic,cpu-interrupt-controller", ingenic_cpu_irq_of_init);
