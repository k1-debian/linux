/*
 * ingenic-tcu.c - Inegnic Soc TCU MFD driver.
 *
 * Copyright (C) 2015 Ingenic Semiconductor Co., Ltd.
 * Written by bo.liu <bo.liu@ingenic.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/syscore_ops.h>
//#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <mfd/ingenic-tcu_v1.h>
#include <linux/clk.h>
#include <linux/ctype.h>

#define WDT_TDR         (0x00)  /* rw, 32, 0x???????? */
#define WDT_TCER        (0x04)  /* rw, 32, 0x???????? */
#define WDT_TCNT        (0x08)  /* rw, 32, 0x???????? */
#define WDT_TCSR        (0x0c)  /* rw, 32, 0x???????? */

#define TCU_TSTR        (0xF0)  /* Timer Status Register,Only Used In Tcu2 Mode */
#define TCU_TSTSR       (0xF4)  /* Timer Status Set Register */
#define TCU_TSTCR       (0xF8)  /* Timer Status Clear Register */
#define TCU_TSR         (0x1C)  /* Timer Stop Register */
#define TCU_TSSR        (0x2C)  /* Timer Stop Set Register */
#define TCU_TSCR        (0x3C)  /* Timer Stop Clear Register */
#define TCU_TER         (0x10)  /* Timer Counter Enable Register */
#define TCU_TESR        (0x14)  /* Timer Counter Enable Set Register */
#define TCU_TECR        (0x18)  /* Timer Counter Enable Clear Register */
#define TCU_TFR         (0x20)  /* Timer Flag Register */
#define TCU_TFSR        (0x24)  /* Timer Flag Set Register */
#define TCU_TFCR        (0x28)  /* Timer Flag Clear Register */
#define TCU_TMR         (0x30)  /* Timer Mask Register */
#define TCU_TMSR        (0x34)  /* Timer Mask Set Register */
#define TCU_TMCR        (0x38)  /* Timer Mask Clear Register */

#define CH_TDFR(n)      (0x40 + (n)*0x10)       /* Timer Data Full Reg */
#define CH_TDHR(n)      (0x44 + (n)*0x10)       /* Timer Data Half Reg */
#define CH_TCNT(n)      (0x48 + (n)*0x10)       /* Timer Counter Reg */
#define CH_TCSR(n)      (0x4C + (n)*0x10)       /* Timer Control Reg */

#define TCSR_PWM_BYPASS (1 << 11)       /* clear counter to 0, only used in TCU2 mode */
#define TCSR_CNT_CLRZ   (1 << 10)       /* clear counter to 0, only used in TCU2 mode */
#define TCSR_PWM_SD     (1 << 9)        /* shut down the pwm output only used in TCU1 mode */
#define TCSR_PWM_HIGH   (1 << 8)        /* selects an initial output level for pwm output */
#define TCSR_PWM_EN     (1 << 7)        /* pwm pin output enable */
#define TCSR_PWM_IN     (1 << 6)        /* pwm pin output enable */

struct ingenic_tcu {
	void __iomem *base;
	struct device *dev;
	struct clk *clk;
	struct device_node *np;
	struct irq_domain *irq_domain;
	struct mfd_cell *tcu_cells;
	struct ingenic_tcu_chn *tcu_chn;

	char idmap[32];
	int channel_mask;
	int irq_tcu0;
	int irq_tcu1;
	int irq_tcu2;
	int channel_num;
	int channel_irq_num;
	spinlock_t lock;
};

#define TCU_DEBUG 0
//#define TCU_FUNC_TEST

static struct ingenic_tcu *tcu;

static inline int tcu_readl(struct ingenic_tcu *tcu, unsigned int reg_addr)
{
	return readl(tcu->base + reg_addr);
}
static inline void tcu_writel(struct ingenic_tcu *tcu, unsigned int reg_addr,
                              unsigned int val)
{
	writel(val, tcu->base + reg_addr);
}
#if TCU_DEBUG
static void tcu_dump(int id)
{
	printk("====================================channel %d===================================\n", id);

	if (id == 16) {
		printk("tcu_readl(WDT_TDR %x) = %x\n", (unsigned int)(tcu->base + WDT_TDR), tcu_readl(tcu, WDT_TDR));
		printk("tcu_readl(WDT_TCER %x) = %x\n", (unsigned int)(tcu->base + WDT_TCER), tcu_readl(tcu, WDT_TCER));
		printk("tcu_readl(WDT_TCNT %x) = %x\n", (unsigned int)(tcu->base + WDT_TCNT), tcu_readl(tcu, WDT_TCNT));
		printk("tcu_readl(WDT_TCSR %x) = %x\n", (unsigned int)(tcu->base + WDT_TCSR), tcu_readl(tcu, WDT_TCSR));
		printk("tcu_readl(TCU_TSR %x) = %x\n", (unsigned int)(tcu->base + TCU_TSR), tcu_readl(tcu, TCU_TSR));
		return;
	}

	printk("tcu_readl(CH_TDFR(id) %x) = %x\n", (unsigned int)(tcu->base + CH_TDFR(id)), tcu_readl(tcu, CH_TDFR(id)));
	printk("tcu_readl(CH_TDHR(id) %x) = %x\n", (unsigned int)(tcu->base + CH_TDHR(id)), tcu_readl(tcu, CH_TDHR(id)));
	printk("tcu_readl(CH_TCNT(id) %x) = %x\n", (unsigned int)(tcu->base + CH_TCNT(id)), tcu_readl(tcu, CH_TCNT(id)));
	printk("tcu_readl(CH_TCSR(id) %x) = %x\n", (unsigned int)(tcu->base + CH_TCSR(id)), tcu_readl(tcu, CH_TCSR(id)));

	printk("tcu_readl(TCU_TSTR %x) = %x\n", (unsigned int)(tcu->base + TCU_TSTR), tcu_readl(tcu, TCU_TSTR));
	printk("tcu_readl(TCU_TSR %x) = %x\n", (unsigned int)(tcu->base + TCU_TSR), tcu_readl(tcu, TCU_TSR));
	printk("tcu_readl(TCU_TER %x) = %x\n", (unsigned int)(tcu->base + TCU_TER), tcu_readl(tcu, TCU_TER));
	printk("tcu_readl(TCU_TFR %x) = %x\n", (unsigned int)(tcu->base + TCU_TFR), tcu_readl(tcu, TCU_TFR));
	printk("tcu_readl(TCU_TMR %x) = %x\n", (unsigned int)(tcu->base + TCU_TMR), tcu_readl(tcu, TCU_TMR));
	printk("=======================================================================\n");
}
#endif

static inline void tcu_mask_full_irq(int id)
{
	tcu_writel(tcu, TCU_TMSR, 1 << id);
}
static inline void tcu_unmask_full_irq(int id)
{
	tcu_writel(tcu, TCU_TMCR, 1 << id);
}
static inline void tcu_clear_full_irq(int id)
{
	tcu_writel(tcu, TCU_TFCR, 1 << id);
}
static inline void tcu_mask_half_irq(int id)
{
	tcu_writel(tcu, TCU_TMSR, 1 << (id + 16));
}
static inline void tcu_unmask_half_irq(int id)
{
	tcu_writel(tcu, TCU_TMCR, 1 << (id + 16));
}
static inline void tcu_clear_half_irq(int id)
{
	tcu_writel(tcu, TCU_TFCR, 1 << (id + 16));
}

void tcu_enable_counter(int id)
{
	tcu_writel(tcu, TCU_TESR, BIT(id));
}
EXPORT_SYMBOL_GPL(tcu_enable_counter);

void tcu_disable_counter(int id)
{
	int index = tcu->idmap[id];
	struct ingenic_tcu_chn *chn = &tcu->tcu_chn[index];
	tcu_writel(tcu, TCU_TECR, BIT(id));
	if (chn->cib.mode == TCU_MODE2) {
		int timeout = 5000;
		while ((tcu_readl(tcu, TCU_TSTR) & (1 << id)) || (timeout--));
		if (!timeout) {
			dev_err(tcu->dev, "channel %d:the reset of counter is not finished now\n", id);
		}
	}
}
EXPORT_SYMBOL_GPL(tcu_disable_counter);

void tcu_start_counter(int id)
{
	tcu_writel(tcu, TCU_TSCR, BIT(id));
}
EXPORT_SYMBOL_GPL(tcu_start_counter);

void tcu_stop_counter(int id)
{
	tcu_writel(tcu, TCU_TSSR, BIT(id));
}
EXPORT_SYMBOL_GPL(tcu_stop_counter);

void tcu_set_counter(int id, unsigned int val)
{
	tcu_writel(tcu, CH_TCNT(id), val);
}
EXPORT_SYMBOL_GPL(tcu_set_counter);

int tcu_get_counter(int id)
{
	return tcu_readl(tcu, CH_TCNT(id));
}
EXPORT_SYMBOL_GPL(tcu_get_counter);

static inline void tcu_disable_all_channel(void)
{
	/* Timer irqs are unmasked by default, mask them */
	tcu_writel(tcu, TCU_TMSR, 0x00ff00ff);
	/* Stop all timer counter */
	tcu_writel(tcu, TCU_TECR, 0x000080ff);
	/* Disable all timer clocks except for those used as system timers */
	tcu_writel(tcu, TCU_TSSR, 0x000000ff);
}

static int tcu_pwm_output_enable(int id)
{
	unsigned int val;

	val = tcu_readl(tcu, CH_TCSR(id));
	val |= TCSR_PWM_EN;
	tcu_writel(tcu, CH_TCSR(id), val);

	return 0;
}

static void tcu_pwm_output_disable(int id)
{
	unsigned int val;
	val = tcu_readl(tcu, CH_TCSR(id));
	val &= ~TCSR_PWM_EN;
	tcu_writel(tcu, CH_TCSR(id), val);
}

static int tcu_pwm_bypass_enable(int id)
{
	unsigned int val;

	val = tcu_readl(tcu, CH_TCSR(id));
	if (val & TCU_CLKSRC_PCK) {
		dev_err(tcu->dev, "TCU %d pwm bypass is not support clk source pck\n", id);
		return -ENXIO;
	}
	val |= TCSR_PWM_BYPASS;
	tcu_writel(tcu, CH_TCSR(id), val);

	return 0;
}

static int tcu_pwm_bypass_disable(int id)
{
	unsigned int val;

	val = tcu_readl(tcu, CH_TCSR(id));
	val &= ~TCSR_PWM_BYPASS;
	tcu_writel(tcu, CH_TCSR(id), val);

	return 0;
}

static void tcu_clear_counter_to_zero(int id)
{
	int index = tcu->idmap[id];
	struct ingenic_tcu_chn *chn = &tcu->tcu_chn[index];
	if (chn->cib.mode == TCU_MODE2) {
		unsigned int val = tcu_readl(tcu, CH_TCSR(id));
		tcu_writel(tcu, CH_TCSR(id), (val | TCSR_CNT_CLRZ));
	}

	tcu_writel(tcu, CH_TCNT(id), 0);
}

static void set_tcu_full_half_value(int id, unsigned int full_num,
                                    unsigned int half_num)
{
	tcu_writel(tcu, CH_TDFR(id), full_num);
	tcu_writel(tcu, CH_TDHR(id), half_num);
}

static void tcu_set_pwm_shutdown(int id, unsigned int shutdown)
{
	unsigned int val;

	val = tcu_readl(tcu, CH_TCSR(id));
	if (shutdown) {
		val |= TCSR_PWM_SD;
	} else {
		val &= ~TCSR_PWM_SD;
	}
	tcu_writel(tcu, CH_TCSR(id), val);
}

static void tcu_set_start_state(int id)
{
	/*fix this*/
	/*tcu_disable_counter(id);*/
	tcu_start_counter(id);

	/* TCSR just keep, which will be update follow process
	   in function ingenic_tcu_config
	   tcu_writel(tcu, CH_TCSR(id), 0);
	 */
}

static void tcu_pwm_input_enable(int id)
{
	unsigned int val;

	val = tcu_readl(tcu, CH_TCSR(id));
	val |= TCSR_PWM_IN;
	tcu_writel(tcu, CH_TCSR(id), val);
}

static void tcu_pwm_input_disable(int id)
{
	unsigned int val;

	val = tcu_readl(tcu, CH_TCSR(id));
	val &= ~TCSR_PWM_IN;
	tcu_writel(tcu, CH_TCSR(id), val);
}

int ingenic_tcu_counter_begin(struct ingenic_tcu_chn *chn)
{
	int ret = 0;

	if (chn->is_pwm) {
		unsigned long flags;

		spin_lock_irqsave(&tcu->lock, flags);
		tcu_pwm_output_enable(chn->cib.id);
		spin_unlock_irqrestore(&tcu->lock, flags);
	}
	tcu_enable_counter(chn->cib.id);
	return ret;
}
EXPORT_SYMBOL_GPL(ingenic_tcu_counter_begin);

void ingenic_tcu_counter_stop(struct ingenic_tcu_chn *chn)
{
	if (chn->is_pwm) {
		unsigned long flags;

		spin_lock_irqsave(&tcu->lock, flags);
		tcu_pwm_output_disable(chn->cib.id);
		spin_unlock_irqrestore(&tcu->lock, flags);
	}

	tcu_disable_counter(chn->cib.id);
}
EXPORT_SYMBOL_GPL(ingenic_tcu_counter_stop);

void ingenic_tcu_set_period(int id, uint16_t period)
{
	tcu_writel(tcu, CH_TDFR(id), period);
}
EXPORT_SYMBOL_GPL(ingenic_tcu_set_period);

void ingenic_tcu_set_duty(int id, uint16_t duty)
{
	tcu_writel(tcu, CH_TDHR(id), duty);
}
EXPORT_SYMBOL_GPL(ingenic_tcu_set_duty);

void ingenic_tcu_set_prescale(int id, enum tcu_prescale prescale)
{
	unsigned int val;
	/* unsigned long flags; */

	//spin_lock_irqsave(&tcu->lock, flags);
	val = tcu_readl(tcu, CH_TCSR(id));
	val &= ~(0x7 << 3);
	val |= (prescale << 3);
	tcu_writel(tcu, CH_TCSR(id), val);
	//spin_unlock_irqrestore(&tcu->lock, flags);
}
EXPORT_SYMBOL_GPL(ingenic_tcu_set_prescale);

void ingenic_tcu_set_pwm_output_init_level(int id, int level)
{
	unsigned int val;
	/* unsigned long flags; */

	//spin_lock_irqsave(&tcu->lock, flags);
	val = tcu_readl(tcu, CH_TCSR(id));
	if (level) {
		val |= (TCSR_PWM_HIGH);
	} else {
		val &= ~(TCSR_PWM_HIGH);
	}
	tcu_writel(tcu, CH_TCSR(id), val);
	//spin_unlock_irqrestore(&tcu->lock, flags);
}
EXPORT_SYMBOL_GPL(ingenic_tcu_set_pwm_output_init_level);

void ingenic_tcu_set_clksrc(int id, enum tcu_clksrc src)
{
	unsigned int val;
	/* unsigned long flags; */

	//spin_lock_irqsave(&tcu->lock, flags);
	val = tcu_readl(tcu, CH_TCSR(id));
	val &= ~0x7;
	val |= src;
	tcu_writel(tcu, CH_TCSR(id), val);

	//spin_unlock_irqrestore(&tcu->lock, flags);
}
EXPORT_SYMBOL_GPL(ingenic_tcu_set_clksrc);

int ingenic_tcu_get_count(int id)
{
	if (id == 1 || id == 2) {
		int i = 0;
		int tmp = 0;

		while ((tmp == 0) && (i < 5)) {
			tmp = tcu_readl(tcu, TCU_TSTR) & (1 << (id + 16));
			i++;
		}
		if (tmp == 0) {
			return -EINVAL;
		}
	}
	return tcu_get_counter(id);
}
EXPORT_SYMBOL_GPL(ingenic_tcu_get_count);

void ingenic_tcu_channel_to_virq(struct ingenic_tcu_chn *chn)
{
	int index;

	if (chn->cib.id == 5) {
		chn->virq[0] = tcu->irq_tcu1;
		return;
	}
	if (chn->cib.id == 15) {
		chn->virq[0] = tcu->irq_tcu0;
		return;
	}
	index = chn->cib.id * 2;

	switch (chn->irq_type) {
		case FULL_IRQ_MODE :
			chn->virq[0] = irq_create_mapping(tcu->irq_domain, index);
			break;
		case HALF_IRQ_MODE :
			chn->virq[1] = irq_create_mapping(tcu->irq_domain, index + 1);
			break;
		case FULL_HALF_IRQ_MODE :
			chn->virq[0] = irq_create_mapping(tcu->irq_domain, index);
			chn->virq[1] = irq_create_mapping(tcu->irq_domain, index + 1);
			break;
		default:
			break;
	}

	if (chn->virq[0] < 0 || chn->virq[1] < 0) {
		return;
	}

	if (chn->virq[0]) {
		irq_set_chip_data(chn->virq[0], chn);
	}

	if (chn->virq[1]) {
		irq_set_chip_data(chn->virq[1], chn);
	}
}
EXPORT_SYMBOL_GPL(ingenic_tcu_channel_to_virq);

void ingenic_watchdog_set_count(unsigned int value)
{
	tcu_writel(tcu, WDT_TCNT, value);
}
EXPORT_SYMBOL_GPL(ingenic_watchdog_set_count);

void ingenic_watchdog_config(unsigned int tcsr_val, unsigned int timeout_value)
{
	tcu_writel(tcu, WDT_TCER, 0);
	tcu_writel(tcu, WDT_TCSR, tcsr_val);
	tcu_writel(tcu, WDT_TDR, timeout_value);
	tcu_writel(tcu, WDT_TCNT, 0);
	tcu_writel(tcu, WDT_TCER, 1);
}
EXPORT_SYMBOL_GPL(ingenic_watchdog_config);

static inline void tcu_disable_current_channel(int id)
{
	/* Stop current timer counter */
	tcu_disable_counter(id);
	/* Disable current timer clocks except for those used as system timers */
	tcu_stop_counter(id);
}

struct mfd_cell *request_cell(int id)
{
	int i;
	if (!(tcu->channel_mask & BIT(id))) {
		dev_err(tcu->dev,
		        "current tcu channel %d busy\n", id);
		return NULL;
	}

	tcu_disable_current_channel(id);
	i = tcu->idmap[id];
	tcu->channel_mask &= ~(BIT(id));

	return &(tcu->tcu_cells[i]);
}
EXPORT_SYMBOL_GPL(request_cell);

void free_cell(int id)
{
	tcu->channel_mask |= BIT(id);
}
EXPORT_SYMBOL_GPL(free_cell);

static int check_pwm_is_availabl(struct ingenic_tcu_chn *chn)
{
	int id = chn->cib.id;
	if (id != 15) {
		if (chn->is_pwm && !(chn->cib.func & PWM_FUNC)) {
			dev_err(tcu->dev, "channel %d not support pwm function\n", id);
			return -1;
		} else if (!chn->is_pwm && !(chn->cib.func & TRACKBALL_FUNC)) {
			dev_err(tcu->dev, "channel %d not support trackball function\n", id);
			return -1;
		}
		if (chn->pwm_in_en && !chn->cib.pwmin) {
			dev_err(tcu->dev, "channel %d not support pwm in function\n", id);
			return -1;
		}
	}
	return 0;
}

int ingenic_tcu_config(struct ingenic_tcu_chn *chn)
{
	unsigned long flags;
	int id = chn->cib.id;

	if (check_pwm_is_availabl(chn) < 0) {
		return -1;
	}
	/*
	  tcu lock should be used to lock all tcu register update.
	  example: led-pwm work thread and pwm suspend maybe update TSR
	  at the same time.
	 */
	spin_lock_irqsave(&tcu->lock, flags);
	tcu_set_start_state(id);

	tcu_clear_full_irq(id);
	tcu_clear_half_irq(id);

	switch (chn->irq_type) {
		case NULL_IRQ_MODE :
			tcu_mask_full_irq(id);
			tcu_mask_half_irq(id);
			tcu_clear_full_irq(id);
			tcu_clear_half_irq(id);
			break;
		case FULL_IRQ_MODE :
			tcu_unmask_full_irq(id);
			tcu_mask_half_irq(id);
			tcu_clear_full_irq(id);
			break;
		case HALF_IRQ_MODE :
			tcu_mask_full_irq(id);
			tcu_unmask_half_irq(id);
			tcu_clear_half_irq(id);
			break;
		case FULL_HALF_IRQ_MODE :
			tcu_unmask_full_irq(id);
			tcu_unmask_half_irq(id);
			tcu_clear_full_irq(id);
			tcu_clear_half_irq(id);
			break;
		default :
			break;
	}

	ingenic_tcu_set_pwm_output_init_level(id, chn->init_level);
	if (chn->cib.mode == TCU_MODE1) {
		tcu_set_pwm_shutdown(id, chn->shutdown_mode);
	}

	if (chn->is_pwm) {
		/*
		   configure just do configuer, pwm should
		   be enable through pwm_enable interface.
		   This is standard usage of pwm interface.
		   */
		if (chn->pwm_bapass_mode) {
			if (chn->clk_src == TCU_CLKSRC_PCK) {
				dev_err(tcu->dev, "the current version can not bypass pclk\n");
			}
			tcu_pwm_bypass_enable(id);
		} else {
			tcu_pwm_bypass_disable(id);
		}
	} else {
		tcu_pwm_output_disable(id);
	}

	ingenic_tcu_set_prescale(id, chn->clk_div);
	set_tcu_full_half_value(id, chn->full_num, chn->half_num);
	if (chn->pwm_in_en) {
		tcu_pwm_input_enable(id);
	} else {
		tcu_pwm_input_disable(id);
	}
	ingenic_tcu_set_clksrc(id, chn->clk_src);
	tcu_clear_counter_to_zero(id);

	spin_unlock_irqrestore(&tcu->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(ingenic_tcu_config);

static void tcu_irq_mask(struct irq_data *data)
{
	struct ingenic_tcu_chn *chn = irq_data_get_irq_chip_data(data);
	int id = chn->cib.id;

	switch (chn->irq_type) {
		case FULL_IRQ_MODE :
			tcu_mask_full_irq(id);
			break;
		case HALF_IRQ_MODE :
			tcu_mask_half_irq(id);
			break;
		case FULL_HALF_IRQ_MODE :
			tcu_mask_full_irq(id);
			tcu_mask_half_irq(id);
			break;
		default:
			break;
	}
}

static void tcu_irq_unmask(struct irq_data *data)
{
	struct ingenic_tcu_chn *chn = irq_data_get_irq_chip_data(data);
	int id = chn->cib.id;

	switch (chn->irq_type) {
		case FULL_IRQ_MODE :
			tcu_unmask_full_irq(id);
			break;
		case HALF_IRQ_MODE :
			tcu_unmask_half_irq(id);
			break;
		case FULL_HALF_IRQ_MODE :
			tcu_unmask_full_irq(id);
			tcu_unmask_half_irq(id);
			break;
		default:
			break;
	}
}

static void tcu_mask_and_ack_irq(struct irq_data *data)
{
	struct ingenic_tcu_chn *chn = irq_data_get_irq_chip_data(data);
	int id = chn->cib.id;

	switch (chn->irq_type) {
		case FULL_IRQ_MODE :
			tcu_mask_full_irq(id);
			tcu_clear_full_irq(id);
			break;
		case HALF_IRQ_MODE :
			tcu_mask_half_irq(id);
			tcu_clear_half_irq(id);
			break;
		case FULL_HALF_IRQ_MODE :
			tcu_mask_full_irq(id);
			tcu_mask_half_irq(id);
			tcu_clear_full_irq(id);
			tcu_clear_half_irq(id);
			break;
		default :
			break;
	}
}

#ifdef TCU_FUNC_TEST
int tcu_channel = 0;
static ssize_t tcu_store_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_tcu *tcu = dev_get_drvdata(dev);
	struct ingenic_tcu_chn *chn;
	int channel_id;
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			return count;
		}
	}
	channel_id = simple_strtoul(str, (char **)&str, 10);
	tcu_channel = channel_id;

	chn = &tcu->tcu_chn[channel_id];

	chn->is_pwm = 0;
	chn->half_num = 0x00ff;
	chn->full_num = 0xffff;
	chn->irq_type = NULL_IRQ_MODE;
	chn->clk_src = TCU_CLKSRC_EXT;
	chn->clk_div = TCU_PRESCALE_256;
	chn->cib.func = TRACKBALL_FUNC;

	if (channel_id >= 0 && channel_id < 8) {

		ingenic_tcu_config(chn);
		ingenic_tcu_counter_begin(chn);
	} else {
		printk("Please select the correct channel 0 ~ 7 range.\n");
	}
	return count;
}

static ssize_t tcu_store_disable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_tcu *tcu = dev_get_drvdata(dev);
	struct ingenic_tcu_chn *chn;
	int id;
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			return count;
		}
	}

	id = simple_strtoul(str, (char **)&str, 10);

	if (id >= 0 && id < 8) {
		chn = &tcu->tcu_chn[id];
		ingenic_tcu_counter_stop(chn);
	} else {
		printk("Please select the correct channel 0 ~ 7 range.\n");
	}
	return count;
}
static ssize_t tcu_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "channel %d count is %d\n", tcu_channel, ingenic_tcu_get_count(tcu_channel));
}

static struct device_attribute tcu_device_attributes[] = {
	__ATTR(enable, S_IWUSR, NULL, tcu_store_enable),
	__ATTR(disable, S_IWUSR, NULL, tcu_store_disable),
	__ATTR(count, S_IRUGO, tcu_count_show, NULL),
};
#endif

static struct irq_chip tcu_irq_chip = {
	.name = "ingenic-tcu_v1",
	.irq_disable = tcu_mask_and_ack_irq,
	.irq_enable = tcu_irq_unmask,
	.irq_unmask = tcu_irq_unmask,
	.irq_mask   = tcu_irq_mask,
	.irq_mask_ack = tcu_mask_and_ack_irq,
};

static int tcu2_irq_domain_map(struct irq_domain *d, unsigned int virq, irq_hw_number_t hw)
{
	struct ingenic_tcu *tcu = d->host_data;

	irq_set_chip_data(virq, tcu);
	irq_set_chip_and_handler(virq, &tcu_irq_chip, handle_level_irq);
	return 0;
}

static const struct irq_domain_ops tcu2_irq_domain_ops = {
	.map = tcu2_irq_domain_map,
	.xlate = irq_domain_xlate_onetwocell,
};

static void tcu_irq_demux(struct irq_desc *desc)
{
	struct ingenic_tcu *tcu = irq_desc_get_handler_data(desc);
	unsigned long pend, mask, tcu2_pend = 0;

	pend = readl(tcu->base + TCU_TFR);
	mask = readl(tcu->base + TCU_TMR) | (1 << 5 | 1 << (5 + 16));

	pend = pend & ~(mask);
	while (pend) {
		struct ingenic_tcu_chn *chn;
		int index;

		tcu2_pend = ffs(pend) - 1;
		index = tcu->idmap[tcu2_pend];
		chn = &tcu->tcu_chn[index];
		if ((pend & 0xffff)) {
			generic_handle_irq(chn->virq[FULL_BIT]);
		} else {
			generic_handle_irq(chn->virq[HALF_BIT]);
		}
		pend &= ~(1 << tcu2_pend);
	}
}

static int __init setup_tcu_irq(struct ingenic_tcu *tcu)
{
	irq_set_chip_and_handler(tcu->irq_tcu0, &tcu_irq_chip, handle_level_irq);
	irq_set_chip_and_handler(tcu->irq_tcu1, &tcu_irq_chip, handle_level_irq);

	tcu->irq_domain = irq_domain_add_linear(tcu->np, tcu->channel_irq_num, &tcu2_irq_domain_ops, (void *)tcu);
	if (!tcu->irq_domain) {
		pr_err("Failed to add tcu irq into irq domain\n");
		return -ENOMEM;
	}
	irq_set_chained_handler_and_data(tcu->irq_tcu2, tcu_irq_demux, tcu);

	return 0;
}

static int init_mfd_cells(struct ingenic_tcu *tcu, struct mfd_cell *cells)
{
	struct device_node *child;
	int i = 0, ret;

	for_each_child_of_node(tcu->np, child) {
		const char *tmp = NULL;
		int id;
		struct ingenic_tcu_chn *chn = &tcu->tcu_chn[i];

		ret = of_property_read_u32(child, "ingenic,channel-info", &chn->chn_info);
		if (ret < 0) {
			dev_err(tcu->dev, "Cannot get ingenic,channel-info\n");
			return -ENOENT;
		}
		ret = of_property_read_string(child, "compatible", &tmp);
		if (ret < 0) {
			dev_err(tcu->dev, "Cannot get compatible string\n");
			return -ENOENT;
		}

		id = chn->cib.id;
		tcu->idmap[id] = i;
		tcu->channel_mask |= BIT(id);
		chn->enable = tcu_start_counter;
		chn->disable = tcu_stop_counter;
		cells[i].id = id;
		cells[i].name = tmp;
		cells[i].pdata_size = sizeof(struct ingenic_tcu_chn);
		cells[i].of_compatible = tmp;
		cells[i].platform_data = chn;
		i ++;
	}
	/* for(i = 0; i < tcu->channel_num; i++) { */
	/*  printk("id = %d, name = %s, of_com = %s\n", cells[i].id, cells[i].name, cells[i].of_compatible); */
	/* } */
	return 0;
}
static int ingenic_tcu_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;

	tcu = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_tcu), GFP_KERNEL);
	if (!tcu) {
		dev_err(&pdev->dev, "Failed to allocate driver structure\n");
		return -ENOMEM;
	}

	tcu->np = pdev->dev.of_node;
	tcu->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		return -ENOENT;
	}
	tcu->base = ioremap(res->start, resource_size(res));
	if (tcu->base == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		return -ENXIO;
	}
	tcu->irq_tcu0 = of_irq_get_byname(tcu->np, "tcu_int0");
	if (tcu->irq_tcu0 < 0) {
		dev_err(tcu->dev, "int0:unable to get IRQ from DT, %d\n", tcu->irq_tcu0);
		return -ENXIO;
	}
	tcu->irq_tcu1 = of_irq_get_byname(tcu->np, "tcu_int1");
	if (tcu->irq_tcu1 < 0) {
		dev_err(tcu->dev, "int1:unable to get IRQ from DT, %d\n", tcu->irq_tcu1);
		return -ENXIO;
	}
	tcu->irq_tcu2 = of_irq_get_byname(tcu->np, "tcu_int2");
	if (tcu->irq_tcu2 < 0) {
		dev_err(tcu->dev, "int2:unable to get IRQ from DT, %d\n", tcu->irq_tcu2);
		return -ENXIO;
	}

	tcu->clk = devm_clk_get(tcu->dev, "gate_tcu");
	clk_prepare_enable(tcu->clk);

	tcu->channel_num = of_get_child_count(tcu->np);
	tcu->tcu_chn = devm_kzalloc(tcu->dev, sizeof(struct ingenic_tcu_chn) * tcu->channel_num,
	                            GFP_KERNEL);
	if (!tcu->tcu_chn) {
		dev_err(tcu->dev, "Failed to allocate driver structure tcu_channel\n");
		return -ENOMEM;
	}
	tcu->channel_irq_num = (tcu->channel_num - 1) * 2;

	setup_tcu_irq(tcu);

	spin_lock_init(&tcu->lock);
	platform_set_drvdata(pdev, tcu);

#ifdef TCU_FUNC_TEST
	for (i = 0; i < ARRAY_SIZE(tcu_device_attributes); i++) {
		ret = device_create_file(&pdev->dev, &tcu_device_attributes[i]);
		if (ret) {
			dev_warn(&pdev->dev, "attribute %d create failed", i);
		}
	}
#endif
	tcu->tcu_cells = devm_kzalloc(tcu->dev, sizeof(struct mfd_cell) * tcu->channel_num,
	                              GFP_KERNEL);
	if (!tcu->tcu_cells) {
		dev_err(tcu->dev, "Failed to allocate driver structure cells\n");
		return -ENOMEM;
	}

	init_mfd_cells(tcu, tcu->tcu_cells);

	ret = mfd_add_devices(&pdev->dev, 0, tcu->tcu_cells,
	                      tcu->channel_num, res, tcu->irq_tcu0, NULL);

	dev_info(&pdev->dev, "Ingenic TCU driver register completed ret = %d\n", ret);

	return 0;
}

static int ingenic_tcu_remove(struct platform_device *pdev)
{
	struct ingenic_tcu *tcu = platform_get_drvdata(pdev);

	clk_disable_unprepare(tcu->clk);

	mfd_remove_devices(&pdev->dev);

	irq_set_handler_data(tcu->irq_tcu0, NULL);
	irq_set_chained_handler(tcu->irq_tcu0, NULL);

	irq_set_handler_data(tcu->irq_tcu1, NULL);
	irq_set_chained_handler(tcu->irq_tcu1, NULL);

	platform_set_drvdata(pdev, NULL);

	devm_kfree(&pdev->dev, tcu);

	return 0;
}

static const struct of_device_id tcu_match[] = {
	{ .compatible = "ingenic,x2500-tcu", },
	{},
};
MODULE_DEVICE_TABLE(of, tcu_match);

static struct platform_driver ingenic_tcu_driver = {
	.probe  = ingenic_tcu_probe,
	.remove = ingenic_tcu_remove,
	.driver = {
		.name = "ingenic-tcu_v1",
		.of_match_table = tcu_match,
	},
};

module_platform_driver(ingenic_tcu_driver);

MODULE_DESCRIPTION("ingenic SoC TCU driver");
MODULE_AUTHOR("bo.liu <bo.liu@ingenic.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-tcu");
