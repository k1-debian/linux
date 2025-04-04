/*
 * Setting up the xburst2 system ost, including:
 *      ---->   core system ost;
 *      ---->   global system ost.
 *
 * Copyright (C) 2016 Ingenic Semiconductor Inc.
 *
 *  Author: dongsheng.qiu, dongsheng.qiu@ingenic.com
 *      Modified by qipengzhen, aric.pzqi@ingenic.com
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/clockchips.h>
#include <linux/sched_clock.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/cpu.h>
#include <linux/delay.h>

#include <dt-bindings/interrupt-controller/mips-irq.h>

#define OSTCCR 0x00
#define OSTER  0x04
#define OSTCR  0x08
#define OSTFR  0x0C
#define OSTMR  0x10
#define OSTDFR 0x14
#define OSTCNT 0x18

#define G_OSTCCR   0x00
#define G_OSTER    0x04
#define G_OSTCR    0x08
#define G_OSTCNTH  0x0C
#define G_OSTCNTL  0x10
#define G_OSTCNTB  0x14
/*-----------------------------------------------------------------------------
 *  -- global ost, clocksource, the whole system shares only one.
 *  -- core ost, each cpu owns one.
 *
 *     -----------------------------------------------
 *        |       |         |         |
 *    |       --------      --------           --------
 *    |      |  cpu0  |    |  cpu1  |         |  cpuN  |
 *    |       --------      --------           --------
 *    |       ^     ^         ^
 *    V       |     |         |
 *   ------------    ------------   -----------       -----------
 *  | global OST |   | core ost0 | | core ost1 | ... | core ostN |
 *   ------------     -----------   -----------       -----------
 *
 *-----------------------------------------------------------------------------*/

struct cpu_ost_map {
	unsigned int cpu_num;
	unsigned int dev_base;
};
static struct cpu_ost_map cpu_ost_map[NR_CPUS];

struct core_timerevent {
	struct clock_event_device clkevt;
	struct irqaction evt_action;
	void __iomem *iobase;
	unsigned int rate;
	raw_spinlock_t  lock;
	unsigned int prev_set;
};

static struct core_timerevent timerevent_buf[NR_CPUS];
#define CLK_DIV  1
#define CSRDIV(x)      ({int n = 0;int d = x; while(d){ d >>= 2;n++;};(n-1);})

#define ost_readl(reg)          readl(reg)
#define ost_writel(value,reg)   writel(value, reg)

struct tmr_src {
	struct clocksource cs;
	struct core_global_ost_resource *resource;
	struct clk *clk_gate;
	void __iomem *iobase;
	raw_spinlock_t  lock;
};
#ifdef CONFIG_SMP
	#define ost_lock(lock,flags) raw_spin_lock_irqsave(&(lock),flags)
	#define ost_unlock(lock,flags) raw_spin_unlock_irqrestore(&(lock),flags)
#else
	#define ost_lock(lock,flags) (flags = 0)
	#define ost_unlock(lock,flags) (flags = 0)
#endif

struct core_ost_chip {
	struct tmr_src *tmrsrc;

	struct core_timerevent *__percpu *percpu_timerevent;
	struct clk *core_gate;
	void __iomem *iobase;
	unsigned int rate;
	int irq;
} g_core_ost;

static raw_spinlock_t clocksource_lock;

static u64 core_clocksource_get_cycles(struct clocksource *cs)
{
	union clycle_type {
		u64 cycle64;
		unsigned int cycle32[2];
	} cycle;
	struct tmr_src *tmr = container_of(cs, struct tmr_src, cs);
	unsigned long flags;
	raw_spin_lock_irqsave(&clocksource_lock, flags);
	void __iomem *iobase = tmr->iobase;

	do {
		cycle.cycle32[1] = ost_readl(iobase + G_OSTCNTH);
		cycle.cycle32[0] = ost_readl(iobase + G_OSTCNTL);
	} while (cycle.cycle32[1] != ost_readl(iobase + G_OSTCNTH));
	raw_spin_unlock_irqrestore(&clocksource_lock, flags);

	return cycle.cycle64;
}

static int tmr_src_enable(struct clocksource *cs)
{
	struct tmr_src *tmr = container_of(cs, struct tmr_src, cs);
	void __iomem *iobase = tmr->iobase;
	unsigned long flags;
	ost_lock(tmr->lock, flags);
	ost_writel(1, iobase + G_OSTER);
	ost_unlock(tmr->lock, flags);
	return 0;
}

static  void tmr_src_disable(struct clocksource *cs)
{
	struct tmr_src *tmr = container_of(cs, struct tmr_src, cs);
	void __iomem *iobase = tmr->iobase;
	unsigned long flags;
	ost_lock(tmr->lock, flags);
	ost_writel(0, iobase + G_OSTER);
	ost_unlock(tmr->lock, flags);
}
static  void tmr_src_suspend(struct clocksource *cs)
{
	struct tmr_src *tmr = container_of(cs, struct tmr_src, cs);
	struct core_ost_chip *core_ost = &g_core_ost;

	if (core_ost->core_gate) {
		clk_disable_unprepare(core_ost->core_gate);
	}
	if (tmr->clk_gate) {
		clk_disable_unprepare(tmr->clk_gate);
	}
}
static void tmr_src_resume(struct clocksource *cs)
{
	struct tmr_src *tmr = container_of(cs, struct tmr_src, cs);
	struct core_ost_chip *core_ost = &g_core_ost;

	if (tmr->clk_gate) {
		clk_prepare_enable(tmr->clk_gate);
	}
	if (core_ost->core_gate) {
		clk_prepare_enable(core_ost->core_gate);
	}

}
static struct tmr_src g_tmr_src = {
	.cs = {
		.name       = "jz_clocksource",
		.rating     = 400,
		.read       = core_clocksource_get_cycles,
		.mask       = 0x7FFFFFFFFFFFFFFFULL,
		.shift      = 10,
		.flags      = CLOCK_SOURCE_WATCHDOG | CLOCK_SOURCE_IS_CONTINUOUS,
		.enable         = tmr_src_enable,
		.disable        = tmr_src_disable,
		.suspend        = tmr_src_suspend,
		.resume         = tmr_src_resume,
	}
};

static unsigned long long sched_clock(void)
{
	struct tmr_src *tmr = &g_tmr_src;
	if (!tmr->iobase) {
		return 0LL;
	}
	if (!ost_readl(tmr->iobase + G_OSTER)) {
		return 0LL;
	}
	return ((u64)core_clocksource_get_cycles(&tmr->cs)); // * tmr_src.cs.mult) >> tmr_src.cs.shift;
}

void __init core_clocksource_init(struct core_ost_chip *core_ost, struct tmr_src *tmr)
{
	unsigned long hz;
	void __iomem *iobase;

	core_ost->tmrsrc = tmr;
	tmr->cs.mult = clocksource_hz2mult(core_ost->rate, tmr->cs.shift);
	iobase = tmr->iobase;

	if (tmr->clk_gate) {
		clk_prepare_enable(tmr->clk_gate);
	}

	hz = core_ost->rate;

	raw_spin_lock_init(&clocksource_lock);

	raw_spin_lock_init(&tmr->lock);
	ost_writel(0, iobase + G_OSTER);
	ost_writel(CSRDIV(CLK_DIV), iobase + OSTCCR);
	ost_writel(1, iobase + G_OSTCR);

	clocksource_register_hz(&tmr->cs, hz);
	sched_clock_register(sched_clock, 64, hz);
}

static int core_timerevent_set_next_event(unsigned long evt,
        struct clock_event_device *clk_evt_dev)
{
	struct core_timerevent *evt_dev = container_of(clk_evt_dev, struct core_timerevent, clkevt);
	void __iomem *iobase = evt_dev->iobase;
	unsigned long flags;
	ost_lock(evt_dev->lock, flags);
	if (evt <= 1) {
		WARN_ON(1);
		evt = 2;
	}
	ost_writel(0, iobase + OSTER);
	if (!ost_readl(iobase + OSTFR)) {
		ost_writel(evt, iobase + OSTDFR);
		ost_writel(1, iobase + OSTCR);
		ost_writel(1, iobase + OSTER);
	} else {
		//pr_err("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO evt = %ld\n",evt);
		evt_dev->prev_set = evt;
	}

	ost_unlock(evt_dev->lock, flags);
	return 0;
}
extern void sysrq_handle_showallcpus(int key);
extern void sysrq_handle_showlocks(int key);
static irqreturn_t core_timerevent_interrupt(int irq, void *dev_id)
{
	struct core_timerevent *evt_dev =  *(struct core_timerevent **)dev_id;
	void __iomem *iobase = evt_dev->iobase;
	int cpu = smp_processor_id();
	unsigned long flags;
	if (ost_readl(iobase + OSTFR)) {
		ost_lock(evt_dev->lock, flags);
		ost_writel(0, iobase + OSTFR);

		if (clockevent_state_oneshot(&evt_dev->clkevt) ||
		    clockevent_state_oneshot_stopped(&evt_dev->clkevt)) {
			if (evt_dev->prev_set == 0) {
				ost_writel(0, iobase + OSTER);
			} else {
				ost_writel(evt_dev->prev_set, iobase + OSTDFR);
				ost_writel(1, iobase + OSTCR);
				ost_writel(1, iobase + OSTER);
				evt_dev->prev_set = 0;
			}

		}

		ost_unlock(evt_dev->lock, flags);
		evt_dev->clkevt.event_handler(&evt_dev->clkevt);
	} else {
		pr_err("\nERROR: cpu: %d c0_status:%x c0_cause:%x\n", cpu, read_c0_status(), read_c0_cause());
		pr_err("CPU[%d]: ost couldn't find ostfr.\n", cpu);
	}
	return IRQ_HANDLED;
}

static int set_state_periodic(struct clock_event_device *clkevt)
{
	struct core_timerevent *evt_dev = container_of(clkevt, struct core_timerevent, clkevt);
	void __iomem *iobase = evt_dev->iobase;
	unsigned int latch = (evt_dev->rate + (HZ >> 1)) / HZ;
	unsigned long flags;

	ost_lock(evt_dev->lock, flags);

	ost_writel(latch, iobase + OSTDFR);
	ost_writel(1, iobase + OSTER);
	ost_unlock(evt_dev->lock, flags);
	return 0;
}
#if 0
static int set_state_oneshot(struct clock_event_device *clkevt)
{
	struct core_timerevent *evt_dev = container_of(clkevt, struct core_timerevent, clkevt);
	void __iomem *iobase = evt_dev->iobase;
	unsigned int latch = (evt_dev->rate + (HZ >> 1)) / HZ;
	unsigned long flags;

	ost_lock(evt_dev->lock, flags);
	ost_writel(latch, iobase + OSTDFR);
	ost_writel(1, iobase + OSTER);
	ost_unlock(evt_dev->lock, flags);
	return 0;

}
static int set_state_oneshot_stopped(struct clock_event_device *clkevt)
{
	struct core_timerevent *evt_dev = container_of(clkevt, struct core_timerevent, clkevt);
	void __iomem *iobase = evt_dev->iobase;
	unsigned long flags;

	ost_lock(evt_dev->lock, flags);
	ost_writel(1, iobase + OSTER);
	ost_unlock(evt_dev->lock, flags);
	return 0;
}
#endif

static int set_state_shutdown(struct clock_event_device *clkevt)
{
	struct core_timerevent *evt_dev = container_of(clkevt, struct core_timerevent, clkevt);
	void __iomem *iobase = evt_dev->iobase;
	unsigned long flags;

	ost_lock(evt_dev->lock, flags);
	ost_writel(0, iobase + OSTER);
	ost_unlock(evt_dev->lock, flags);
	return 0;
}
#if 0
static int tick_resume(struct clock_event_device *clkevt)
{
	struct core_timerevent *evt_dev = container_of(clkevt, struct core_timerevent, clkevt);
	void __iomem *iobase = evt_dev->iobase;
	unsigned int latch = (evt_dev->rate + (HZ >> 1)) / HZ;
	unsigned long flags;
	unsigned int val;

	ost_lock(evt_dev->lock, flags);
	val = ost_readl(iobase + OSTCNT);
	if (val >= latch) {
		ost_writel(latch, iobase + OSTDFR);
	}
	ost_writel(1, iobase + OSTER);

	ost_unlock(evt_dev->lock, flags);

	return 0;
}
#endif

static void percpu_timerevent_init(struct core_ost_chip *core_ost, int cpu_num)
{
	int i;
	void __iomem *iobase;
	unsigned int base = 0;
	struct core_timerevent *timerevent = NULL;
	struct clock_event_device *cd = NULL;
	pr_info("percpu cpu_num:%d timerevent init\n", cpu_num);
	for (i = 0; i < NR_CPUS; i++) {
		if (cpu_ost_map[i].cpu_num == cpu_num) {
			base = cpu_ost_map[i].dev_base;
			timerevent = &timerevent_buf[i];
			break;
		}
	}

	if (base == 0) {
		pr_err("Error: CPU[%d] don't finded ost base address\n", cpu_num);
		return;
	}

	iobase = (void *)base;
	if (timerevent->iobase == NULL) {
		timerevent->iobase = iobase;
		timerevent->rate = core_ost->rate;

		raw_spin_lock_init(&timerevent->lock);
		cd = &timerevent->clkevt;
		memset(cd, 0, sizeof(struct clock_event_device));
		cd->name = "clockenvent";
		cd->features = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC;
		cd->shift = 10;
		cd->rating = 400;
		cd->set_state_periodic = set_state_periodic;
		cd->set_state_oneshot = set_state_shutdown;
		cd->set_state_oneshot_stopped = set_state_shutdown;
		cd->set_state_shutdown = set_state_shutdown;
		cd->tick_resume = set_state_shutdown;
		cd->set_next_event = core_timerevent_set_next_event;
		cd->irq = core_ost->irq;
		cd->cpumask = cpumask_of(smp_processor_id());
	} else {
		iobase = timerevent->iobase;
		cd = &timerevent->clkevt;
	}
	ost_writel(0, iobase + OSTER); //disable ost
	ost_writel(CSRDIV(CLK_DIV), iobase + OSTCCR);
	ost_writel(1, iobase + OSTCR);
	ost_writel(0, iobase + OSTMR);

	clockevents_config_and_register(cd, timerevent->rate, 0xf, 0xffffffff);

	*this_cpu_ptr(core_ost->percpu_timerevent) = timerevent;

	enable_percpu_irq(core_ost->irq, IRQ_TYPE_NONE);
	printk("clockevents_config_and_register success.\n");
}
void ingenic_percpu_timerevent_init(unsigned int cpu_num)
{
	percpu_timerevent_init(&g_core_ost, cpu_num);
}
EXPORT_SYMBOL_GPL(ingenic_percpu_timerevent_init);

static void percpu_timerevent_deinit(struct core_ost_chip *core_ost)
{
	struct core_timerevent *timerevent = *this_cpu_ptr(core_ost->percpu_timerevent);
	disable_percpu_irq(core_ost->irq);
	ost_writel(0, timerevent->iobase + OSTER); //disable ost
	ost_writel(1, timerevent->iobase + OSTMR);
}
void ingenic_percpu_timerevent_deinit(void)
{
	percpu_timerevent_deinit(&g_core_ost);
}
EXPORT_SYMBOL_GPL(ingenic_percpu_timerevent_deinit);

static void __init core_ost_setup(struct core_ost_chip *core_ost, unsigned long ext_rate)
{
	int ret;

	if (core_ost->core_gate) {
		clk_prepare_enable(core_ost->core_gate);
	}

	core_ost->rate = (ext_rate / CLK_DIV);   /* TODO: should be clk_get_rate(extclk) / CLK_DIV in real chip */

	core_ost->percpu_timerevent = alloc_percpu(struct core_timerevent *);
	if (!core_ost->percpu_timerevent) {
		pr_err("ost percpu alloc fail!\n");
		return;
	}
	ret = request_percpu_irq(core_ost->irq, core_timerevent_interrupt, "core_timerevent", core_ost->percpu_timerevent);
	if (ret < 0) {
		pr_err("dddd timer request ost error %d \n", ret);
	}
	memset(timerevent_buf, 0, sizeof(timerevent_buf));

}

static int __init ingenic_ost_init(struct device_node *np)
{
	struct core_ost_chip *core_ost = &g_core_ost;
	struct tmr_src *tmr = &g_tmr_src ;
	struct clk *ext_clk = NULL;
	unsigned long ext_rate;
	void __iomem *g_iobase = NULL;
	void __iomem *core_iobase = NULL;
	int irq = -1;
	struct property *prop;
	const unsigned int *vp;
	unsigned int pv;
	int index = 0;

	g_iobase = of_io_request_and_map(np, 0, "ost");
	if (g_iobase == NULL) {
		pr_err("Failed to map global clocksource iobase!\n");
		return -EINVAL;
	}
	core_iobase = of_io_request_and_map(np, 1, "core-ost");
	if (core_iobase == NULL) {
		pr_err("Failed to map core clockevent iobase!\n");
		return -EINVAL;
	}

	irq = of_irq_get_byname(np, "sys_ost");
	if (irq < 0) {
		pr_err("ftm: unable to get IRQ from DT, %d\n", irq);
		return -EINVAL;
	}

	of_property_for_each_u32(np, "cpu-ost-map", prop, vp, pv) {
		if (index % 2) {
			cpu_ost_map[index / 2].dev_base = (unsigned int)core_iobase + pv;
		} else {
			cpu_ost_map[index / 2].cpu_num = pv;
		}
		index ++;
		if (index > NR_CPUS * 2 - 1) {
			printk("parse cpu-ost-iomap, ost number define in dt is too large!\n");
			break;
		}
	}

	ext_clk = clk_get(NULL, "ext");
	if (IS_ERR_OR_NULL(ext_clk)) {
		pr_err("Warnning ... Ingenic Ost: Failed to get ext clk, Using default clk rate(24MHz),\n");
		ext_rate = 24000000;
	} else {
		ext_rate = clk_get_rate(ext_clk);
		clk_put(ext_clk);
	}

	tmr->iobase = g_iobase;
	core_ost->iobase = core_iobase;
	core_ost->irq = irq;

	tmr->clk_gate = clk_get(NULL, "gate_ost");
	if (IS_ERR_OR_NULL(tmr->clk_gate)) {
		pr_err("Failed to get global ost clk gate!\n");
		tmr->clk_gate = NULL;
	}

	core_ost->core_gate = clk_get(NULL, "gate_ost");
	if (IS_ERR_OR_NULL(core_ost->core_gate)) {
		pr_err("Failed to get core ost clk gate!\n");
		core_ost->core_gate = NULL;
	}

	core_ost_setup(core_ost, ext_rate);
	percpu_timerevent_init(core_ost, 0);
	core_clocksource_init(core_ost, tmr);

	return 0;
}

TIMER_OF_DECLARE(ingenic_ost_init, "ingenic,core-ost", ingenic_ost_init);
