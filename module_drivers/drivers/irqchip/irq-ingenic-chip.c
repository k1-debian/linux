/*
 * Copyright (C) 2017 Ingenic Semiconductor Co., Ltd.
 *
 *  Author: dongsheng.qiu, dongsheng.qiu@ingenic.com bo.liu@ingenic.com
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/ioport.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/timex.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/irqchip.h>
#include <linux/clk.h>

#include <dt-bindings/interrupt-controller/mips-irq.h>

#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/irq_cpu.h>
#include <asm/irq.h>

#define PART_OFF        0x20
#define ISR_OFF         (0x00)
#define IMR_OFF         (0x04)
#define IMSR_OFF        (0x08)
#define IMCR_OFF        (0x0c)
#define IPR_OFF         (0x10)

struct cpu_intc_map {
	unsigned int cpu_num;
	unsigned int dev_base;
};
static struct cpu_intc_map cpu_intc_map[NR_CPUS];

struct core_irq_chip {
	void __iomem *iobase;
	unsigned int next_irq_resp;
};
struct irq_chip_data {
	void __iomem *iobase[NR_CPUS];
	raw_spinlock_t  lock;
	struct cpumask irq_idle_mask;
	unsigned int wake_up_flag[(INTC_NR_IRQS + 31) / 32];    /*  Interrupts which can generate wakeup signal when cpu asleep. */
	unsigned char intc_num[INTC_NR_IRQS];                   /*  Which CPU/INTC does an interrupt current connect to, */
	struct cpumask affinity[INTC_NR_IRQS];                  /*  Which CPU does an interrupt src affinity to. */
};
struct core_irq_chips {
	struct core_irq_chip *__percpu *percpu_irq_chip;
	struct irq_chip_data chip_data;
	int irq;
	void __iomem *iobase; /* external intc iobase*/
	struct clk *intc_clk;
};

static struct core_irq_chip irq_chip_buf[NR_CPUS];
static struct core_irq_chips g_irq_chips;

static void percpu_irq_init(struct core_irq_chips *irq_chips, unsigned int cpu_num)
{
	unsigned int base = 0;
	struct core_irq_chip *irq_chip;
	int cpu = smp_processor_id();
	void __iomem *iobase;
	int i;

	for (i = 0; i < NR_CPUS; i++) {
		if (cpu_intc_map[i].cpu_num == cpu_num) {
			base = cpu_intc_map[i].dev_base;
			irq_chip = &irq_chip_buf[i];
			break;
		}
	}

	if (base == 0) {
		pr_err("Error: CPU[%d] don't finded intc base address\n", cpu_num);
		return;
	}

	iobase = (void *)base;

	irq_chip = &irq_chip_buf[cpu];

	irq_chip->iobase = iobase;
	irq_chip->next_irq_resp = 0;
	irq_chips->chip_data.iobase[cpu] = iobase;

	*this_cpu_ptr(irq_chips->percpu_irq_chip) = irq_chip;
	enable_percpu_irq(irq_chips->irq, IRQ_TYPE_NONE);

	pr_info("percpu irq inited.\n");
}
int ingenic_percpu_irq_init(int cpu_num)
{
	percpu_irq_init(&g_irq_chips, cpu_num);
	return 0;
}
EXPORT_SYMBOL_GPL(ingenic_percpu_irq_init);

static void percpu_irq_deinit(struct core_irq_chips *irq_chips)
{
	disable_percpu_irq(irq_chips->irq);
}

void ingenic_percpu_irq_deinit(void)
{
	percpu_irq_deinit(&g_irq_chips);
}
EXPORT_SYMBOL_GPL(ingenic_percpu_irq_deinit);

#ifdef CONFIG_SMP
static inline void irq_lock(struct irq_chip_data *chip)
{
	raw_spin_lock(&chip->lock);
}

static inline void irq_unlock(struct irq_chip_data *chip)
{
	raw_spin_unlock(&chip->lock);
}
#else
static inline void irq_lock(struct irq_chip_data *chip) { }
static inline void irq_unlock(struct irq_chip_data *chip) { }
#endif

static void xburst2_irq_unmask(struct irq_data *data)
{
	struct irq_chip_data *chip = (struct irq_chip_data *)irq_data_get_irq_chip_data(data);
	void __iomem *iobase;
	int hwirq = data->hwirq;
	unsigned int group;
	unsigned int bit;
	int intc_num;

	group = hwirq / 32;
	bit = 1 << ((hwirq) & 31);
	irq_lock(chip);
	intc_num = chip->intc_num[hwirq];
	iobase = chip->iobase[intc_num];
	writel(bit, iobase + group * PART_OFF + IMCR_OFF);
	irq_unlock(chip);
}

static void xburst2_irq_mask(struct irq_data *data)
{
	struct irq_chip_data *chip = (struct irq_chip_data *)irq_data_get_irq_chip_data(data);
	void __iomem *iobase;
	int hwirq = data->hwirq;
	unsigned int group;
	unsigned int bit;
	int intc_num;

	group = hwirq / 32;
	bit = 1 << ((hwirq) & 31);
	irq_lock(chip);
	intc_num = chip->intc_num[hwirq];
	iobase = chip->iobase[intc_num];
	writel(bit, iobase + group * PART_OFF + IMSR_OFF);

#ifdef CONFIG_SMP
	ingenic_irq_migration(0);
#endif
	irq_unlock(chip);
}

static int xburst2_irq_set_wake(struct irq_data *data, unsigned int on)
{
	struct irq_chip_data *chip = (struct irq_chip_data *)irq_data_get_irq_chip_data(data);
	unsigned int group = (data->irq - IRQ_INTC_BASE) / 32;
	unsigned int bit = 1 << ((data->irq - IRQ_INTC_BASE) & 31);

	irq_lock(chip);
	if (on) {
		chip->wake_up_flag[group] |= bit;
	} else {
		chip->wake_up_flag[group] &= ~bit;
	}
	irq_unlock(chip);
	return 0;
}

#ifdef CONFIG_SMP
static int xburst2_set_affinity(struct irq_data *data, const struct cpumask *dest, bool force)
{
	struct irq_chip_data *chip = (struct irq_chip_data *)irq_data_get_irq_chip_data(data);
	int hwirq = data->hwirq;
	unsigned int cpu = cpumask_any_and(dest, cpu_online_mask);
	void __iomem *prev_iobase;
	void __iomem *iobase;
	int prev_intc_num;
	int group;
	unsigned int bit;

	//      printk("########%s, %d, dest->bits %lx, cpu_online_mask->bits: %lx\n", __func__, __LINE__, dest->bits[0], cpu_online_mask->bits[0]);

	irq_lock(chip);

	cpumask_and(&chip->affinity[hwirq], dest, cpu_online_mask);

	prev_intc_num = chip->intc_num[hwirq];
	prev_iobase = chip->iobase[prev_intc_num];
	iobase = chip->iobase[cpu];
	group = hwirq / 32;
	bit = 1 << ((hwirq) & 31);

	if (!(readl(prev_iobase + PART_OFF * group + IMR_OFF) & bit)) {
		writel(bit, prev_iobase + PART_OFF * group + IMSR_OFF);
		writel(bit, iobase + PART_OFF * group + IMCR_OFF);
	} else {
		writel(bit, prev_iobase + PART_OFF * group + IMSR_OFF);
	}
	chip->intc_num[hwirq] = cpu;

	irq_unlock(chip);

	return IRQ_SET_MASK_OK;
}
#endif

static struct irq_chip xburst2_irq_chip = {
	.name           = "XBurst2-irqchip",
	.irq_mask       = xburst2_irq_mask,
	.irq_mask_ack   = xburst2_irq_mask,
	.irq_unmask     = xburst2_irq_unmask,
	.irq_ack        = xburst2_irq_mask,
	.irq_eoi        = xburst2_irq_unmask,
	.irq_set_wake   = xburst2_irq_set_wake,
#ifdef CONFIG_SMP
	.irq_set_affinity = xburst2_set_affinity,
#endif
};

static irqreturn_t xburst2_irq_handler(int irq, void *data)
{
	struct core_irq_chip *irq_chip = *(struct core_irq_chip **)data;
	void __iomem *iobase = irq_chip->iobase;
	unsigned int irq_num = 0xffffffff;
	int n = 0, i;
	unsigned int next_irq_resp = irq_chip->next_irq_resp;
	unsigned int ipr = readl(iobase + (next_irq_resp / 32) * PART_OFF + IPR_OFF);

	do {
		if (irq) {
			for (i = next_irq_resp & 31; i < 32; i++) {
				if (ipr & (1 << i)) {
					irq_num = (next_irq_resp & (~31)) + i;
					break;
				}
			}
		}
		if (irq_num != 0xffffffff) {
			break;
		}
		next_irq_resp = (next_irq_resp & (~31)) + 32;
		if (next_irq_resp >= INTC_NR_IRQS) {
			next_irq_resp = 0;
		}
		ipr = readl(iobase + (next_irq_resp / 32) * PART_OFF + IPR_OFF);
		n++;
	} while (n < ((INTC_NR_IRQS + 31) / 32 + 1));

	if (irq_num != 0xffffffff) {
#ifdef CONFIG_SMP
		irq_chip->next_irq_resp = irq_num;
#else
		irq_chip->next_irq_resp = irq_num + 1;
		if (irq_chip->next_irq_resp >= INTC_NR_IRQS) {
			irq_chip->next_irq_resp = 0;
		}
#endif
		do_IRQ(irq_num + IRQ_INTC_BASE);
	} else {
		//pr_err("Error: Not Find any irq,check me %s %d.\n",__FILE__,__LINE__);
	}
	return IRQ_HANDLED;
}

static void __init core_irq_setup(struct core_irq_chips *irq_chips)
{
	struct irq_chip_data *chip = &irq_chips->chip_data;
	int ret;
	int i;

	irq_chips->percpu_irq_chip = alloc_percpu(struct core_irq_chip *);
	if (!irq_chips->percpu_irq_chip) {
		pr_err("ERROR:alloc cpu intc dev percpu fail!\n");
		return;
	}

	ret = request_percpu_irq(irq_chips->irq, xburst2_irq_handler, "xburst2-intc", irq_chips->percpu_irq_chip);
	if (ret) {
		pr_err("ERROR:intc request error,ret: %d check %s %d\n", ret, __FILE__, __LINE__);
		return;
	}

	cpumask_clear(&chip->irq_idle_mask);
	raw_spin_lock_init(&chip->lock);

	for (i = 0; i < ARRAY_SIZE(chip->wake_up_flag); i++) {
		chip->wake_up_flag[i] = 0;
	}

	for (i = 0; i < INTC_NR_IRQS; i++) {
		cpumask_clear(&chip->affinity[i]);
		chip->intc_num[i] = 0;
	}

	pr_info("core irq setup finished\n");
}

static int ingenic_irq_domain_map(struct irq_domain *d, unsigned int irq,
                                  irq_hw_number_t hw)
{

	irq_set_chip_data(irq, (void *)d->host_data);
	irq_set_chip_and_handler(irq, &xburst2_irq_chip, handle_level_irq);

	return 0;
}

static const struct irq_domain_ops ingenic_irq_domain_ops = {
	.map = ingenic_irq_domain_map,
	.xlate = irq_domain_xlate_onetwocell,
};

static int __init ingenic_intc_of_init(struct device_node *node)
{
	struct core_irq_chips *irq_chips = &g_irq_chips;
	void __iomem *iobase;
	void __iomem *iobase_ex;
	struct irq_domain *domain;
	struct irq_chip_data *chip = &irq_chips->chip_data;
	int irq;
	struct property *prop;
	const unsigned int *vp;
	unsigned int pv;
	int index = 0;

	if (WARN_ON(!node)) {
		return -ENODEV;
	}

	iobase = of_iomap(node, 0);
	WARN(!iobase, "Unable to map core intc iobase registers!\n");

	iobase_ex = of_iomap(node, 1);
	WARN(!iobase_ex, "Unable to map external intc iobase registers!\n");
	irq_chips->iobase = iobase_ex;

	irq = irq_of_parse_and_map(node, 0);
	WARN(irq < 0, "Failed to get intc irq from DT!\n");

	irq_chips->irq = irq;

	of_property_for_each_u32(node, "cpu-intc-map", prop, vp, pv) {
		if (index % 2) {
			cpu_intc_map[index / 2].dev_base = (unsigned int)iobase + pv;
		} else {
			cpu_intc_map[index / 2].cpu_num = pv;
		}

		index ++;
		if (index > NR_CPUS * 2 - 1) {
			printk("parse cpu-intc-iomap, intc define in dt is too large!\n");
			break;
		}
	}

	core_irq_setup(irq_chips);

	irq_chips->intc_clk  = clk_get(NULL, "gate_intc");

	domain = irq_domain_add_legacy(node, INTC_NR_IRQS, 8, 0, &ingenic_irq_domain_ops, chip);
	WARN(!iobase, "Unable to register IRQ domain!\n");

	percpu_irq_init(irq_chips, 0);

	return 0;
}

static int __init ingenic_intc_init(struct device_node *node,
                                    struct device_node *parent)
{
	return ingenic_intc_of_init(node);
}
IRQCHIP_DECLARE(ingenic_intc, "ingenic,core-intc", ingenic_intc_init);

asmlinkage void plat_irq_dispatch(void)
{
	unsigned long status = read_c0_status();
	unsigned long cause = read_c0_cause();
	volatile unsigned long r = ((status & cause) >> 8) & 0xff;

	if (r) {
		if (r & 8) {        /* IPI */
			do_IRQ(MIPS_CPU_IRQ_BASE + 3);
		} else if (r & 16) { /* OST */
			do_IRQ(MIPS_CPU_IRQ_BASE + 4);
		} else if (r & 4) { /* INTC */
			do_IRQ(MIPS_CPU_IRQ_BASE + 2);
		} else {        /* Others */
			do_IRQ(MIPS_CPU_IRQ_BASE + __ffs(r));
		}

	} else {
		//printk("IRQ Error, cpu: %d Cause:0x%08lx, Status:0x%08lx\n", smp_processor_id(), cause, status);
	}
}

void ingenic_irq_migration(int lock)
{
	struct core_irq_chips *irq_chips = &g_irq_chips;
	struct irq_chip_data *chip = &irq_chips->chip_data;
	unsigned int cpu = smp_processor_id();
	void __iomem *iobase = NULL;
	void __iomem *prev_iobase = NULL;
	int i;
	struct cpumask resp_mask;
	unsigned int ipr = 0;

	if (lock) {
		irq_lock(chip);
	}

	prev_iobase = chip->iobase[cpu];
	for (i = 0; i < INTC_NR_IRQS; i++) {
		int group = i / 32;
		int bitcount = i & 31;
		unsigned int bit = 1 << bitcount;
		int resp_cpu;

		if (bitcount == 0) {
			ipr = readl(prev_iobase + group * PART_OFF + IPR_OFF);
		}

		if (ipr == 0) {
			i = (i & (~31)) + 31;
			continue;
		}

		if (ipr & bit) {
			cpumask_and(&resp_mask, &chip->affinity[i], &chip->irq_idle_mask);
			for_each_cpu(resp_cpu, &resp_mask) {
				if (resp_cpu != cpu) {
					//printk("migrate irq from cpu[%d] to cpu[%d], i: %d\n", cpu, resp_cpu, i);
					iobase = chip->iobase[resp_cpu];
					if (!(readl(prev_iobase + PART_OFF * group + IMR_OFF) & bit)) {
						writel(bit, prev_iobase + PART_OFF * group + IMSR_OFF);
						writel(bit, iobase + PART_OFF * group + IMCR_OFF);
					} else {
						writel(bit, iobase + PART_OFF * group + IMSR_OFF);
					}
					cpumask_clear_cpu(resp_cpu, &chip->irq_idle_mask);
					chip->intc_num[i] = resp_cpu;
					break;
				}
			}
		}
	}
	if (lock) {
		irq_unlock(chip);
	}
}
EXPORT_SYMBOL_GPL(ingenic_irq_migration);

void ingenic_irq_cpumask_idle(int idle)
{
	struct core_irq_chips *irq_chips = &g_irq_chips;
	struct irq_chip_data *chip = &irq_chips->chip_data;
	int cpu = smp_processor_id();
	irq_lock(chip);
	if (idle) {
		cpumask_set_cpu(cpu, &chip->irq_idle_mask);
	} else {
		cpumask_clear_cpu(cpu, &chip->irq_idle_mask);
	}
	irq_unlock(chip);
}
EXPORT_SYMBOL_GPL(ingenic_irq_cpumask_idle);

static unsigned long intc_saved[4];
extern void __enable_irq(struct irq_desc *desc, unsigned int irq, bool resume);

void arch_suspend_disable_irqs(void)
{
	struct irq_chip_data *chip_data = &g_irq_chips.chip_data;
	struct core_irq_chip *irq_chip = *this_cpu_ptr(g_irq_chips.percpu_irq_chip);
	struct core_irq_chips *irq_chips = &g_irq_chips;

	unsigned int *intc_wakeup = chip_data->wake_up_flag;
	void __iomem *intc_base = irq_chip->iobase;

	local_irq_disable();

	intc_saved[0] = readl(intc_base + IMR_OFF);
	intc_saved[1] = readl(intc_base + PART_OFF + IMR_OFF);

	/* Mask interrupts which are not wakeup src. */
	writel(0xffffffff & ~intc_wakeup[0], intc_base + IMSR_OFF);
	writel(0xffffffff & ~intc_wakeup[1], intc_base + PART_OFF + IMSR_OFF);
	if (IS_ERR(irq_chips->intc_clk)) {
		irq_chips->intc_clk  = clk_get(NULL, "gate_intc");
	}
	if (!IS_ERR(irq_chips->intc_clk)) {
		clk_prepare_enable(irq_chips->intc_clk);
	}

	intc_saved[2] = readl(irq_chips->iobase + IMR_OFF);
	intc_saved[3] = readl(irq_chips->iobase + PART_OFF + IMR_OFF);

	writel(0xffffffff & ~intc_wakeup[0], irq_chips->iobase + IMR_OFF);
	writel(0xffffffff & ~intc_wakeup[1], irq_chips->iobase + PART_OFF + IMR_OFF);

}

void arch_suspend_enable_irqs(void)
{
	struct core_irq_chip *irq_chip = *this_cpu_ptr(g_irq_chips.percpu_irq_chip);
	void __iomem *intc_base = irq_chip->iobase;
	struct core_irq_chips *irq_chips = &g_irq_chips;
	struct clk *intc_clk = irq_chips->intc_clk;

	writel(intc_saved[2], irq_chips->iobase + IMR_OFF);
	writel(intc_saved[3], irq_chips->iobase + PART_OFF + IMR_OFF);
	if (!IS_ERR(intc_clk)) {
		clk_disable_unprepare(intc_clk);
	}

	writel(0xffffffff & ~intc_saved[0], intc_base + IMCR_OFF);
	writel(0xffffffff & ~intc_saved[1], intc_base + PART_OFF + IMCR_OFF);

	local_irq_enable();
}
