/*
 * Copyright (C) 2015 Ingenic Semiconductor Co., Ltd.
 * Author:  xyfu <xiaoyang.fu@ingenic.com> (kernel.3.10.14)
 * Modified:    cli <chen.li@ingenic.com>
 *
 * Operating System Timer interface for Ingenic's SOC, such as X1000,
 * and so on. (kernel.4.4)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

/* #define DEBUG */
/* #define VERBOSE_DEBUG */
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/clockchips.h>
#include <linux/clk.h>
#include <linux/sched_clock.h>
#include <linux/clk-provider.h>
#include <linux/notifier.h>
#include <linux/cpu.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <soc/ost.h>

#define CLKSOURCE_DIV   (16)
#define CLKEVENT_DIV    (16)

/*-----------------------------------------------------------------------------
 *  C L O C K S O U R C E
 *-----------------------------------------------------------------------------*/
struct tmr_src {
	struct clocksource cs;
	struct clk *clk_gate;
	void __iomem *iobase;
};

/* clocksource cycle base type */
typedef u64 cycle_t;

static cycle_t ingenic_read_cycles(struct clocksource *cs)
{
	struct tmr_src *tmr = container_of(cs, struct tmr_src, cs);
	union clycle_type {
		cycle_t cycle64;
		unsigned int cycle32[2];
	} cycle;

	cycle.cycle32[0] = ost_readl(tmr->iobase + OST_T2CNTL);
	cycle.cycle32[1] = ost_readl(tmr->iobase + OST_TCNT2HBUF);

	return cycle.cycle64;
}

static int tmr_src_enable(struct clocksource *cs)
{
	struct tmr_src *tmr = container_of(cs, struct tmr_src, cs);

	ost_writel(tmr->iobase + OST_TESR, TESR_OSTEN2);
	return 0;
}
static  void tmr_src_disable(struct clocksource *cs)
{
	struct tmr_src *tmr = container_of(cs, struct tmr_src, cs);

	ost_writel(tmr->iobase + OST_TCR, TCR_OSTCLR2);
	ost_writel(tmr->iobase + OST_TECR, TESR_OSTEN2);
}
static void tmr_src_suspend(struct clocksource *cs)
{
	struct tmr_src *tmr = container_of(cs, struct tmr_src, cs);

	ost_writel(tmr->iobase + OST_TFR, ~TFR_OSTM);
	if (tmr->clk_gate) {
		clk_disable_unprepare(tmr->clk_gate);

	}
}
static void tmr_src_resume(struct clocksource *cs)
{
	struct tmr_src *tmr = container_of(cs, struct tmr_src, cs);
	if (tmr->clk_gate) {
		clk_prepare_enable(tmr->clk_gate);
	}
}

static struct tmr_src tmr_src = {
	.cs = {
		.name       = "ingenic_clocksource",
		.rating     = 400,
		.read       = ingenic_read_cycles,
		.mask       = CLOCKSOURCE_MASK(64),
		.flags      = CLOCK_SOURCE_WATCHDOG | CLOCK_SOURCE_IS_CONTINUOUS,
		.enable         = tmr_src_enable,
		.disable        = tmr_src_disable,
		.suspend        = tmr_src_suspend,
		.resume         = tmr_src_resume,
	}
};

static u64 notrace ingenic_read_sched_clock(void)
{
	union clycle_type {
		uint64_t cycle64;
		uint32_t cycle32[2];
	} cycle;
	cycle.cycle32[0] = ost_readl(tmr_src.iobase + OST_T2CNTL);
	cycle.cycle32[1] = ost_readl(tmr_src.iobase + OST_TCNT2HBUF);

	return cycle.cycle64;
}

static void __init ingenic_clocksource_init(struct tmr_src *tmr, unsigned long ext_rate)
{
	unsigned long hz = ext_rate / CLKSOURCE_DIV;
	unsigned int val;

	tmr->clk_gate = clk_get(NULL, "gate_ost");
	if (IS_ERR_OR_NULL(tmr->clk_gate)) {
		pr_err("ERROR: Failed to get clk, Please check clk driver.\n");
		pr_err("%s, incase of Debug at the very beginning, we ignore the clk_gate. \n \
			You must implemented clk driver immediately\n\n\t\n", __func__);
	} else {
		clk_prepare_enable(tmr->clk_gate);
	}

	val =  ost_readl(tmr->iobase + OST_TCCR);
	if (ost_readl(tmr->iobase + OST_TER) & TESR_OSTEN2) {
		u32 div = 0;
		/*
		 * get previous boot time
		 */
		switch ((val & TCCRDIV_MSK2) >> TCCRDIV_SFT2) {
		case 0: div = 1; break;
		case 1: div = 4; break;
		case 2: div = 16; break;
		}
		if (likely(div)) {
			u64 cycles = ingenic_read_sched_clock();
			u32 pre_hz = ext_rate / div;
			cycles = cycles * USEC_PER_SEC;
			do_div(cycles, pre_hz);
			pr_info("Previous Boot Time is %u us\n", (u32)cycles);
		}
		ost_writel(tmr->iobase + OST_TECR, TESR_OSTEN2);
	}
	val &= ~TCCRDIV_MSK2;
	val |= TCCRDIV2(CLKSOURCE_DIV);
	ost_writel(tmr->iobase + OST_TCCR, val);
	ost_writel(tmr->iobase + OST_TCR, TCR_OSTCLR2);
	ost_writel(tmr->iobase + OST_TESR, TESR_OSTEN2);

	clocksource_register_hz(&tmr->cs, hz);
	sched_clock_register(ingenic_read_sched_clock, 64, hz);
}

/*-----------------------------------------------------------------------------
 *  C L O C K E V E N T
 *-----------------------------------------------------------------------------*/
struct ingenic_timerevent {
	struct clock_event_device clkevt;
	void __iomem *iobase;
	unsigned int periodic_ticks;
	struct clk *clk_gate;
	unsigned int rate;
	int irq;
	struct irqaction evt_action;
} ingenic_clockevent;

static int ingenic_set_state_periodic(struct clock_event_device *cd)
{
	struct ingenic_timerevent *evt_dev = container_of(cd,
	                                     struct ingenic_timerevent, clkevt);

	ost_writel(evt_dev->iobase + OST_TMR, TMR_OSTM);
	ost_writel(evt_dev->iobase + OST_TECR, TESR_OSTEN1);
	ost_writel(evt_dev->iobase + OST_TFR, ~TFR_OSTM);
	ost_writel(evt_dev->iobase + OST_T1DFR,  evt_dev->periodic_ticks);
	ost_writel(evt_dev->iobase + OST_TCR, TCR_OSTCLR1);
	ost_writel(evt_dev->iobase + OST_TESR, TESR_OSTEN1);
	ost_writel(evt_dev->iobase + OST_TMR, ~TMR_OSTM);
	return 0;
}

static int ingenic_set_state_shutdown(struct clock_event_device *cd)
{
	struct ingenic_timerevent *evt_dev = container_of(cd,
	                                     struct ingenic_timerevent, clkevt);
	ost_writel(evt_dev->iobase + OST_TECR, TESR_OSTEN1);
	return 0;

}
static int ingenic_set_next_event(unsigned long evt, struct clock_event_device *cd)
{
	struct ingenic_timerevent *evt_dev = container_of(cd,
	                                     struct ingenic_timerevent, clkevt);

	ost_writel(evt_dev->iobase + OST_TMR, TMR_OSTM);
	ost_writel(evt_dev->iobase + OST_TECR, TESR_OSTEN1);
	ost_writel(evt_dev->iobase + OST_TFR, ~TFR_OSTM);
	ost_writel(evt_dev->iobase + OST_T1DFR, evt);
	ost_writel(evt_dev->iobase + OST_TCR, TCR_OSTCLR1);
	ost_writel(evt_dev->iobase + OST_TESR, TESR_OSTEN1);
	ost_writel(evt_dev->iobase + OST_TMR, ~TMR_OSTM);
	return 0;
}

static irqreturn_t ingenic_timer_interrupt(int irq, void *dev_id)
{
	struct ingenic_timerevent *evt_dev = dev_id;
	struct clock_event_device *cd = &evt_dev->clkevt;

	ost_writel(evt_dev->iobase + OST_TFR, ~TFR_OSTM);

	if (clockevent_state_oneshot(cd)) {
		ost_writel(evt_dev->iobase + OST_TECR, TESR_OSTEN1);
	}

	evt_dev->clkevt.event_handler(&evt_dev->clkevt);
	return IRQ_HANDLED;
}

static void __init ingenic_clockevent_init(struct ingenic_timerevent *evt_dev, unsigned long ext_rate)
{
	struct clock_event_device *cd = &evt_dev->clkevt;
	unsigned int val;
	int ret;

	ost_writel(evt_dev->iobase + OST_TMR, TMR_OSTM);
	ost_writel(evt_dev->iobase + OST_TECR, TESR_OSTEN1);
	ost_writel(evt_dev->iobase + OST_TFR, ~TFR_OSTM);
	val = ost_readl(evt_dev->iobase + OST_TCCR);
	val &= ~TCCRDIV_MSK1;
	val |= TCCRDIV1(CLKEVENT_DIV);
	ost_writel(evt_dev->iobase + OST_TCCR, val);
	ost_writel(evt_dev->iobase + OST_TCR, TCR_OSTCLR1);

	evt_dev->evt_action.handler = ingenic_timer_interrupt;
	evt_dev->evt_action.flags = IRQF_TIMER;
	evt_dev->evt_action.name = "ingenic-timerost";
	evt_dev->evt_action.dev_id = (void *)evt_dev;
	evt_dev->rate = ext_rate / CLKEVENT_DIV;
	evt_dev->periodic_ticks = ((evt_dev->rate + (HZ >> 1)) / HZ) - 1;   /* - 1 for scroll back */

	memset(cd, 0, sizeof(struct clock_event_device));
	ret = request_irq(evt_dev->irq, evt_dev->evt_action.handler, evt_dev->evt_action.flags, evt_dev->evt_action.name, (void *)cd);
	if (ret < 0) {
		pr_err("request system ost error %d \n", ret);
	}

	cd->name = "ingenic-clockenvent";
	cd->features = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC;
	cd->rating = 400;
	cd->set_state_periodic = ingenic_set_state_periodic;
	cd->set_state_oneshot = ingenic_set_state_shutdown;
	cd->set_state_shutdown = ingenic_set_state_shutdown;
	cd->set_next_event = ingenic_set_next_event;
	cd->irq = evt_dev->irq;
	cd->cpumask = cpumask_of(0);

	clockevents_config_and_register(cd, evt_dev->rate, 4, 0xffffffff);
}

static int __init ingenic_ost_init(struct device_node *np)
{
	struct ingenic_timerevent *evt = &ingenic_clockevent;
	struct tmr_src *tmr = &tmr_src ;
	struct clk *ext_clk = NULL;
	unsigned long ext_rate;
	void __iomem *iobase = NULL;
	int irq_ost = -1;

	iobase = of_io_request_and_map(np, 0, "ost");
	if (iobase == NULL) {
		pr_err("Failed to map clocksource iobase!\n");
		return -EINVAL;
	}

	irq_ost = of_irq_get_byname(np, "sys_ost");
	if (irq_ost < 0) {
		pr_err("ftm: unable to get IRQ from DT, %d\n", irq_ost);
		return -EINVAL;
	}

	ext_clk = clk_get(NULL, "ext");
	if (IS_ERR_OR_NULL(ext_clk)) {
		pr_warn("Warning Ingenic Ost: Can not get extern clock, Please check clk driver !!\n\n\t\n");
		ext_rate = 24000000;
	} else {

		ext_rate = clk_get_rate(ext_clk);
		clk_put(ext_clk);
	}

	tmr->iobase = iobase;
	evt->iobase = iobase;
	evt->irq = irq_ost;

	ingenic_clocksource_init(tmr, ext_rate);
	ingenic_clockevent_init(evt, ext_rate);

	return 0;
}

TIMER_OF_DECLARE(x1000_ost_init, "ingenic,x1000-ost", ingenic_ost_init);
TIMER_OF_DECLARE(x1800_ost_init, "ingenic,x1800-ost", ingenic_ost_init);
TIMER_OF_DECLARE(x1021_ost_init, "ingenic,x1021-ost", ingenic_ost_init);
TIMER_OF_DECLARE(x1520_ost_init, "ingenic,x1520-ost", ingenic_ost_init);
TIMER_OF_DECLARE(x1600_ost_init, "ingenic,x1600-ost", ingenic_ost_init);
TIMER_OF_DECLARE(x1630_ost_init, "ingenic,x1630-ost", ingenic_ost_init);
