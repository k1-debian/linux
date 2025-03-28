/*
 * ingenic_tcu.c - ingenic Soc TCU MFD driver.
 *
 * Copyright (C) 2020 Ingenic Semiconductor Co., Ltd.
 * Written by wssong <wenshuo.song@ingenic.com>.
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
#include <irq.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ctype.h>
#include <linux/syscore_ops.h>
#include <linux/mutex.h>
#include <asm/div64.h>
#include <linux/mfd/core.h>
#include <mfd/ingenic-tcu_v2.h>
#include <linux/clk.h>
#include <linux/of.h>

#define NR_TCU_CHNS TCU_NR_IRQS

#define DE_WARNING  0

static inline void ingenic_tcu_full_mask(struct ingenic_tcu_chn *tcu_chn);
static inline void ingenic_tcu_half_mask(struct ingenic_tcu_chn *tcu_chn);

/*Timer Counter Enable/Disable Register*/
static inline void ingenic_tcu_set_enable(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_TESR, 1 << tcu_chn->index);
}

static inline void ingenic_tcu_set_disable(struct ingenic_tcu_chn *tcu_chn)
{
	u32 tcsr;

	spin_lock(&tcu_chn->tcu->lock);
	ingenic_tcu_full_mask(tcu_chn);
	ingenic_tcu_half_mask(tcu_chn);
	tcu_writel(tcu_chn->tcu, TCU_TECR, 1 << tcu_chn->index);
	if ((tcu_chn->capture_num & CAPTURE_LOOP_FLAGS) != 0) {
		tcsr = tcu_readl(tcu_chn->tcu, CHN_CAP(tcu_chn->index)) & ~CAP_NUM_MSK;
		tcu_writel(tcu_chn->tcu, CHN_CAP(tcu_chn->index), tcsr);
	}

	tcu_chn->en_flag = 0;
	spin_unlock(&tcu_chn->tcu->lock);
}

/*Timer Stop Set/Clr Register*/
static inline void ingenic_tcu_set_stop(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_TSSR, 1 << tcu_chn->index);
}

static inline void ingenic_tcu_clr_stop(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_TSCR, 1 << tcu_chn->index);
}

/*Timer Mast Register set/clr operation and flag clr operation (full)*/
static inline void ingenic_tcu_full_mask(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_TMSR, 1 << tcu_chn->index);
}

static inline void ingenic_tcu_full_unmask(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_TMCR, 1 << tcu_chn->index);
}

static inline void ingenic_tcu_clear_full_flag(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_TFCR, 1 << tcu_chn->index);
}

/*Timer Mast Register set/clr operation and flag clr operation (half)*/
static inline void ingenic_tcu_half_mask(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_TMSR, 1 << (tcu_chn->index + 16));
}

static inline void ingenic_tcu_half_unmask(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_TMCR, 1 << (tcu_chn->index + 16));
}

static inline void ingenic_tcu_clear_half_flag(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_writel(tcu_chn->tcu, TCU_TFCR, 1 << (tcu_chn->index + 16));
}

/*Timer Control Register select the TCNT count clock frequency*/
static inline void ingenic_tcu_set_prescale(struct ingenic_tcu_chn *tcu_chn, enum tcu_prescale prescale)
{
	u32 tcsr = tcu_chn_readl(tcu_chn, CHN_TCSR) & ~CSR_DIV_MSK;
	tcu_chn_writel(tcu_chn, CHN_TCSR, tcsr | (prescale << 3));
}

/*Timer Counter clear to zero*/
static inline void ingenic_tcu_clear_tcnt(struct ingenic_tcu_chn *tcu_chn)
{
	tcu_chn_writel(tcu_chn, CHN_TCNT, 0);
}

/*Timer Data FULL/HALF Register*/
static inline void ingenic_tcu_set_chn_full(struct ingenic_tcu_chn *tcu_chn, unsigned int value)
{
	tcu_chn_writel(tcu_chn, CHN_TDFR, value);
}

static inline void ingenic_tcu_set_chn_half(struct ingenic_tcu_chn *tcu_chn, unsigned int value)
{
	tcu_chn_writel(tcu_chn, CHN_TDHR, value);
}

/*Timer Control Register set count mode */
static inline void ingenic_tcu_set_count_mode(struct ingenic_tcu_chn *tcu_chn, enum tcu_count_mode count_mode)
{
	u32 tcsr = tcu_chn_readl(tcu_chn, CHN_TCSR) & ~(CSR_CM_MSK);
	tcu_chn_writel(tcu_chn, CHN_TCSR, tcsr | count_mode << 22);
}

/*Timer control set  pos and neg*/
static inline void ingenic_tcu_set_pos(struct ingenic_tcu_chn *tcu_chn, unsigned int offset)
{
	u32 tcsr;
	if (offset > 15 && offset < 22) {
		tcsr = tcu_chn_readl(tcu_chn, CHN_TCSR) | TCU_CONTROL_BIT(offset);
		tcu_chn_writel(tcu_chn, CHN_TCSR, tcsr);
	}
}

static inline void ingenic_tcu_set_neg(struct ingenic_tcu_chn *tcu_chn, unsigned int offset)
{
	u32 tcsr;
	if (offset > 15 && offset < 22) {
		tcsr = tcu_chn_readl(tcu_chn, CHN_TCSR) | TCU_CONTROL_BIT(offset);
		tcu_chn_writel(tcu_chn, CHN_TCSR, tcsr);
	}

}

static inline void ingenic_tcu_set_pos_neg(struct ingenic_tcu_chn *tcu_chn, unsigned int offset, unsigned int offset1)
{
	ingenic_tcu_set_pos(tcu_chn, offset);
	ingenic_tcu_set_neg(tcu_chn, offset1);
}

static inline void ingenic_tcu_clr_pos_neg(struct ingenic_tcu_chn *tcu_chn, unsigned int bit)
{
	u32 tcsr = tcu_chn_readl(tcu_chn, CHN_TCSR) & ~(TCU_CONTROL_BIT(bit) | TCU_CONTROL_BIT((bit + 1)));
	tcu_chn_writel(tcu_chn, CHN_TCSR, tcsr);
}

/*Timer control config signal pos and neg*/
static inline void ingenic_tcu_config_sig_pos_neg(struct ingenic_tcu_chn *tcu_chn, enum tcu_clksrc clksrc)
{
	switch (clksrc) {
		case TCU_CLKSRC_EXT :
			ingenic_tcu_clr_pos_neg(tcu_chn, CLK_POS);
			switch (tcu_chn->sig_ext) {
				case SIG_NEG_EN :
					ingenic_tcu_set_neg(tcu_chn, CLK_NEG);
					break;
				case SIG_POS_EN :
					ingenic_tcu_set_pos(tcu_chn, CLK_POS);
					break;
				case SIG_POS_NEG_EN :
					ingenic_tcu_set_pos_neg(tcu_chn, CLK_POS, CLK_NEG);
					break;
				default :
					break;
			}
			break;
		case TCU_CLKSRC_GPIO0 :
			ingenic_tcu_clr_pos_neg(tcu_chn, GPIO0_POS);
			switch (tcu_chn->sig_gpio0) {
				case SIG_NEG_EN :
					ingenic_tcu_set_neg(tcu_chn, GPIO0_NEG);
					break;
				case SIG_POS_EN :
					ingenic_tcu_set_pos(tcu_chn, GPIO0_POS);
					break;
				case SIG_POS_NEG_EN :
					ingenic_tcu_set_pos_neg(tcu_chn, GPIO0_NEG, GPIO0_POS);
					break;
				default :
					break;
			}
			break;
		case TCU_CLKSRC_GPIO1 :
			ingenic_tcu_clr_pos_neg(tcu_chn, GPIO1_POS);
			switch (tcu_chn->sig_gpio1) {
				case SIG_NEG_EN :
					ingenic_tcu_set_neg(tcu_chn, GPIO1_NEG);
					break;
				case SIG_POS_EN :
					ingenic_tcu_set_pos(tcu_chn, GPIO1_POS);
					break;
				case SIG_POS_NEG_EN :
					ingenic_tcu_set_pos_neg(tcu_chn, GPIO1_POS, GPIO1_NEG);
					break;
				default :
					break;
			}
			break;
		default :
			break;
	}
}

/*Timer Control Register select timer clock input and counting mode pos or neg*/
static inline void ingenic_tcu_set_clksrc(struct ingenic_tcu_chn *tcu_chn, enum tcu_clksrc ext, enum tcu_clksrc gpio0, enum tcu_clksrc gpio1)
{
	u32 tcsr;
	ingenic_tcu_config_sig_pos_neg(tcu_chn, ext);
	ingenic_tcu_config_sig_pos_neg(tcu_chn, gpio0);
	ingenic_tcu_config_sig_pos_neg(tcu_chn, gpio1);
	tcsr = tcu_chn_readl(tcu_chn, CHN_TCSR) & ~(ONE_BIT_OFFSET(2) | ONE_BIT_OFFSET(6) | ONE_BIT_OFFSET(7));
	tcu_chn_writel(tcu_chn, CHN_TCSR, tcsr | ext | gpio0 | gpio1);
}

static inline void ingenic_tcu_set_shutdown(struct ingenic_tcu_chn *tcu_chn)
{
	if (tcu_chn->shutdown_mode) {
		tcu_chn_writel(tcu_chn, CHN_TCSR, tcu_chn_readl(tcu_chn, CHN_TCSR) | (TCU_CONTROL_BIT(15)));
	} else {
		tcu_chn_writel(tcu_chn, CHN_TCSR, tcu_chn_readl(tcu_chn, CHN_TCSR) & ~(TCU_CONTROL_BIT(15)));
	}
}

static inline void ingenic_tcu_irq_mode(struct ingenic_tcu_chn *tcu_chn)
{
	switch (tcu_chn->irq_type) {
		case NULL_IRQ_MODE :
			ingenic_tcu_full_mask(tcu_chn);
			ingenic_tcu_half_mask(tcu_chn);
			break;
		case FULL_IRQ_MODE :
			ingenic_tcu_full_unmask(tcu_chn);
			ingenic_tcu_half_mask(tcu_chn);
			break;
		case HALF_IRQ_MODE :
			ingenic_tcu_full_mask(tcu_chn);
			ingenic_tcu_half_unmask(tcu_chn);
			break;
		case FULL_HALF_IRQ_MODE :
			ingenic_tcu_full_unmask(tcu_chn);
			ingenic_tcu_half_unmask(tcu_chn);
			break;
		default:
			break;
	}
}

/*config gate work mode */
static inline void ingenic_tcu_config_gate_mode(struct ingenic_tcu_chn *tcu_chn)
{
	u32 tcsr;
	if (tcu_chn->gate_pola) {
		tcu_chn_writel(tcu_chn, CHN_TCSR, tcu_chn_readl(tcu_chn, CHN_TCSR) | (TCU_CONTROL_BIT(14)));
	} else {
		tcu_chn_writel(tcu_chn, CHN_TCSR, tcu_chn_readl(tcu_chn, CHN_TCSR) & ~(TCU_CONTROL_BIT(14)));
	}
	tcsr = tcu_chn_readl(tcu_chn, CHN_TCSR) & ~(CSR_GATE_MSK);
	tcu_chn_writel(tcu_chn, CHN_TCSR, tcsr | tcu_chn->gate_sel << 11);
}

/*config direction work mode*/
static inline void ingenic_tcu_config_direction_mode(struct ingenic_tcu_chn *tcu_chn)
{
	u32 tcsr;
	if (tcu_chn->dir_pola) {
		tcu_chn_writel(tcu_chn, CHN_TCSR, tcu_chn_readl(tcu_chn, CHN_TCSR) | (TCU_CONTROL_BIT(13)));
	} else {
		tcu_chn_writel(tcu_chn, CHN_TCSR, tcu_chn_readl(tcu_chn, CHN_TCSR) & ~(TCU_CONTROL_BIT(13)));
	}
	tcsr = tcu_chn_readl(tcu_chn, CHN_TCSR) & ~(CSR_DIR_MSK);
	tcu_chn_writel(tcu_chn, CHN_TCSR, tcsr | tcu_chn->dir_sel << 8);
}

/*config quadrature work mode*/
static inline void ingenic_tcu_config_quadrature_mode(struct ingenic_tcu_chn *tcu_chn)
{
	u32 tcsr;
	if (tcu_chn->dir_pola) {
		tcu_chn_writel(tcu_chn, CHN_TCSR, tcu_chn_readl(tcu_chn, CHN_TCSR) | (TCU_CONTROL_BIT(13)));
	} else {
		tcu_chn_writel(tcu_chn, CHN_TCSR, tcu_chn_readl(tcu_chn, CHN_TCSR) & ~(TCU_CONTROL_BIT(13)));
	}
	tcsr = tcu_chn_readl(tcu_chn, CHN_TCSR) & ~(CSR_DIR_MSK);
	tcu_chn_writel(tcu_chn, CHN_TCSR, tcsr | tcu_chn->dir_sel << 8);

}

/*config pos work mode*/
static inline void ingenic_tcu_config_pos_mode(struct ingenic_tcu_chn *tcu_chn)
{
	u32 tcsr;
	if (tcu_chn->pos_sel > 0 && tcu_chn->pos_sel < 4) {
		tcsr = tcu_readl(tcu_chn->tcu, CHN_CAP(tcu_chn->index)) & ~(CAP_SEL_MSK);
		tcu_writel(tcu_chn->tcu, CHN_CAP(tcu_chn->index), tcsr | tcu_chn->pos_sel << 16);
	}
}

/*config capture work mode*/
static inline void ingenic_tcu_config_capture_mode(struct ingenic_tcu_chn *tcu_chn)
{
	u32 tcsr;
	if (tcu_chn->capture_sel >= 0 && tcu_chn->capture_sel < 3) {
		tcsr = tcu_readl(tcu_chn->tcu, CHN_CAP(tcu_chn->index)) & ~(CAP_SEL_MSK | CAP_NUM_MSK);
		tcu_writel(tcu_chn->tcu, CHN_CAP(tcu_chn->index), tcsr | tcu_chn->capture_sel << 16 | tcu_chn->capture_num);
	}
}

/*config filter work mode*/
static inline void ingenic_tcu_config_filter_mode(struct ingenic_tcu_chn *tcu_chn)
{
	u32 tcsr;
	tcsr = tcu_readl(tcu_chn->tcu, CHN_FIL_VAL(tcu_chn->index)) & ~(FIL_VAL_GPIO1_MSK | FIL_VAL_GPIO0_MSK);
	tcu_writel(tcu_chn->tcu, CHN_FIL_VAL(tcu_chn->index), tcsr | tcu_chn->fil_a_num | tcu_chn->fil_b_num << 16);
}

/*Choose a working mode*/
static inline void ingenic_tcu_sel_work_mode(struct ingenic_tcu_chn *tcu_chn)
{
	switch (tcu_chn->mode_sel) {
		case GENERAL_MODE :
			break;
		case GATE_MODE :
			ingenic_tcu_config_gate_mode(tcu_chn);
			break;
		case DIRECTION_MODE :
			ingenic_tcu_config_direction_mode(tcu_chn);
			break;
		case QUADRATURE_MODE :
			ingenic_tcu_config_quadrature_mode(tcu_chn);
			break;
		case POS_MODE :
			ingenic_tcu_config_pos_mode(tcu_chn);
			break;
		case CAPTURE_MODE :
			ingenic_tcu_config_capture_mode(tcu_chn);
			break;
		case FILTER_MODE :
			ingenic_tcu_config_filter_mode(tcu_chn);
			break;
		default :
			break;
	}

}

void sws_pr_debug(struct ingenic_tcu_chn *tcu_chn, int count)
{
	printk("\n\n----------------------------------------count N0.%d----------------------------------------------------\n\n", count);
	printk("-stop-----addr-%08x-value-%08x-----------\n", (unsigned int)(tcu_chn->tcu->iomem + TCU_TSR), tcu_readl(tcu_chn->tcu, TCU_TSR));
	printk("-mask-----addr-%08x-value-%08x-----------\n", (unsigned int)(tcu_chn->tcu->iomem + TCU_TMR), tcu_readl(tcu_chn->tcu, TCU_TMR));
	printk("-enable---addr-%08x-value-%08x-----------\n", (unsigned int)(tcu_chn->tcu->iomem + TCU_TER), tcu_readl(tcu_chn->tcu, TCU_TER));
	printk("-flag-----addr-%08x-value-%08x-----------\n", (unsigned int)(tcu_chn->tcu->iomem + TCU_TFR), tcu_readl(tcu_chn->tcu, TCU_TFR));
	printk("-Control--addr-%08x-value-%08x-----------\n", (unsigned int)(tcu_chn->tcu->iomem + tcu_chn->reg_base + CHN_TCSR), tcu_chn_readl(tcu_chn, CHN_TCSR));
	printk("-full-----addr-%08x-value-%08x-----------\n", (unsigned int)(tcu_chn->tcu->iomem + tcu_chn->reg_base + CHN_TDFR), tcu_chn_readl(tcu_chn, CHN_TDFR));
	printk("-half-   -addr-%08x-value-%08x-----------\n", (unsigned int)(tcu_chn->tcu->iomem + tcu_chn->reg_base + CHN_TDHR), tcu_chn_readl(tcu_chn, CHN_TDHR));
	printk("-TCNT-----addr-%08x-value-%08x-----------\n", (unsigned int)(tcu_chn->tcu->iomem + tcu_chn->reg_base + CHN_TCNT), tcu_chn_readl(tcu_chn, CHN_TCNT));
	printk("-CAP reg_base-------value-%08x-----------\n", tcu_readl(tcu_chn->tcu, CHN_CAP(tcu_chn->index)));
	printk("-CAP_VAL register---value-%08x-----------\n", tcu_readl(tcu_chn->tcu, CHN_CAP_VAL(tcu_chn->index)));
}
EXPORT_SYMBOL_GPL(sws_pr_debug);

void ingenic_tcu_clear_irq_flag(struct ingenic_tcu_chn *tcu_chn)
{
	ingenic_tcu_clear_full_flag(tcu_chn);
	ingenic_tcu_clear_half_flag(tcu_chn);
}

EXPORT_SYMBOL_GPL(ingenic_tcu_clear_irq_flag);

void ingenic_tcu_config_chn(struct ingenic_tcu_chn *tcu_chn)
{

	spin_lock(&tcu_chn->tcu->lock);
	/* Clear IRQ flag */
	ingenic_tcu_clear_irq_flag(tcu_chn);

	/* Config IRQ */
	ingenic_tcu_irq_mode(tcu_chn);

	/*select work mode*/
	ingenic_tcu_sel_work_mode(tcu_chn);

	/*full num and half num*/
	ingenic_tcu_set_chn_full(tcu_chn, tcu_chn->full_num);
	ingenic_tcu_set_chn_half(tcu_chn, tcu_chn->half_num);

	/*TCNT clear to 0*/
	ingenic_tcu_clear_tcnt(tcu_chn);

	/* shutdown mode */
	ingenic_tcu_set_shutdown(tcu_chn);

	/* prescale */
	if (!(tcu_readl(tcu_chn->tcu, TCU_TER) & (1 << tcu_chn->index))) {
		ingenic_tcu_set_prescale(tcu_chn, tcu_chn->prescale);
	}

	/* clk source */
	ingenic_tcu_set_clksrc(tcu_chn, tcu_chn->clk_ext, tcu_chn->clk_gpio0, tcu_chn->clk_gpio1);

	/*select counter mode*/
	ingenic_tcu_set_count_mode(tcu_chn, tcu_chn->count_mode);
	spin_unlock(&tcu_chn->tcu->lock);
}
EXPORT_SYMBOL_GPL(ingenic_tcu_config_chn);

#if DE_WARNING
static void ingenic_tcu_irq_mask(struct ingenic_tcu *tcu)
{
	unsigned long flags;
	struct ingenic_tcu_chn *channel = tcu->channel;
	int id ;

	for (id = 0; id < NR_TCU_CHNS; id++) {
		if (channel[id].en_flag) {
			spin_lock_irqsave(&channel[id].tcu->lock, flags);
			switch (channel[id].irq_type) {
				case FULL_IRQ_MODE :
					ingenic_tcu_full_mask(&channel[id]);
					break;
				case HALF_IRQ_MODE :
					ingenic_tcu_half_mask(&channel[id]);
					break;
				case FULL_HALF_IRQ_MODE :
					ingenic_tcu_full_mask(&channel[id]);
					ingenic_tcu_half_mask(&channel[id]);
					break;
				default:
					break;
			}
			spin_unlock_irqrestore(&channel[id].tcu->lock, flags);
		}
	}
}
#endif

#if DE_WARNING
static void ingenic_tcu_irq_unmask(struct ingenic_tcu *tcu)
{
	unsigned long flags;
	int id ;
	struct ingenic_tcu_chn *channel = tcu->channel;

	for (id = 0; id < NR_TCU_CHNS; id++) {
		if (channel[id].en_flag) {
			spin_lock_irqsave(&channel[id].tcu->lock, flags);
			switch (channel[id].irq_type) {
				case FULL_IRQ_MODE :
					ingenic_tcu_full_unmask(&channel[id]);
					break;
				case HALF_IRQ_MODE :
					ingenic_tcu_half_unmask(&channel[id]);
					break;
				case FULL_HALF_IRQ_MODE :
					ingenic_tcu_full_unmask(&channel[id]);
					ingenic_tcu_half_unmask(&channel[id]);
					break;
				default:
					break;
			}
			spin_unlock_irqrestore(&channel[id].tcu->lock, flags);
		}
	}
}
#endif

static void ingenic_tcu_irq_ack(struct ingenic_tcu *tcu)
{
	unsigned long flags;
	unsigned long tmp;
	int id ;
	int i;
	int times = timeout;
	struct ingenic_tcu_chn *channel = tcu->channel;

	for (id = 0; id < NR_TCU_CHNS; id++) {
		if (channel[id].en_flag) {
			spin_lock_irqsave(&channel[id].tcu->lock, flags);
			switch (channel[id].irq_type) {
				case FULL_IRQ_MODE :
					if ((tcu_readl(tcu, TCU_TFR) & (1 << id)) && !(tcu_readl(tcu, TCU_TMR) & (1 << id))) {
						ingenic_tcu_clear_full_flag(&channel[id]);
					}
					break;
				case HALF_IRQ_MODE :
					if ((tcu_readl(tcu, TCU_TFR) & (1 << (16 + id))) && !(tcu_readl(tcu, TCU_TMR) & (1 << (16 + id)))) {
						ingenic_tcu_clear_half_flag(&channel[id]);
					}
					break;
				case FULL_HALF_IRQ_MODE :
					if ((tcu_readl(tcu, TCU_TFR) & (1 << (16 + id))) && !(tcu_readl(tcu, TCU_TMR) & (1 << (16 + id)))) {
						ingenic_tcu_clear_half_flag(&channel[id]);
					}
					if ((tcu_readl(tcu, TCU_TFR) & (1 << id)) && !(tcu_readl(tcu, TCU_TMR) & (1 << id))) {
						ingenic_tcu_clear_full_flag(&channel[id]);
					}
					break;
				default:
					break;
			}

			if (channel[id].gpio_trigger) {
				tmp = ingenic_tcu_store_flag_st(tcu);
				i = 0;
				while (tmp) {
					if (tmp & (1 << i)) {
						tmp &= ~(1 << i);
						ingenic_tcu_store_flag_clr(&channel[i]);
						printk("%d %d %d\n", __LINE__, i, ingenic_tcu_store_val(&channel[i]));
					}

					if (times-- == 0) {
						break;
					}
					i++;
				}
			}
			spin_unlock_irqrestore(&channel[id].tcu->lock, flags);
		}
	}
}

/*This is a simple configuration test demo*/
static void ingenic_tcu_config_attr(int id, enum tcu_mode_sel mode_sel, struct ingenic_tcu_chn *channel)
{
	int i = 0;
	channel[id].mode_sel = mode_sel;
	channel[id].irq_type = FULL_IRQ_MODE;
	channel[id].full_num = 0xffff;
	channel[id].half_num = 0x5000;
	channel[id].prescale = TCU_PRESCALE_1024;

	channel[id].count_value = 0;
	channel[id].shutdown_mode = 0;
	channel[id].en_flag = 1;

	/*In order to facilitate testing,
	 * it has no practical significance.*/
	switch (channel[id].mode_sel) {
		case GENERAL_MODE:
			/*Enable external clock to use rising edge counting , result TCNT != 0*/
			channel[id].clk_ext = TCU_CLKSRC_EXT;
			channel[id].sig_ext = SIG_POS_EN;
			break;
		case GATE_MODE:
			/*gate signal hold on 0,counter start when control signal is 1,result TCNT == 0*/
			channel[id].clk_gpio0    = TCU_CLKSRC_GPIO0;
			channel[id].gate_sel     = GATE_SEL_GPIO0;
			channel[id].gate_pola    = GATE_POLA_HIGH;
			channel[id].clk_ext      = TCU_CLKSRC_EXT;
			channel[id].sig_ext      = SIG_POS_EN;
			break;
		case DIRECTION_MODE:
			/*use gpio0 with direction signa. counter sub when control signal is 1.result TCNT add and sub */
			channel[id].clk_gpio0    = TCU_CLKSRC_GPIO0;
			channel[id].dir_sel  = DIR_SEL_GPIO0;
			channel[id].dir_pola     = DIR_POLA_HIGH;
			channel[id].clk_ext  = TCU_CLKSRC_EXT;
			channel[id].sig_ext  = SIG_POS_EN;
			break;
		case QUADRATURE_MODE:
			channel[id].clk_ext      = TCU_CLKSRC_EXT;
			channel[id].clk_gpio0    = TCU_CLKSRC_GPIO0;
			channel[id].clk_gpio1    = TCU_CLKSRC_GPIO1;
			channel[id].dir_sel      = DIR_SEL_GPIO_QUA;
			channel[id].sig_gpio0    = SIG_POS_NEG_EN;
			channel[id].sig_gpio1    = SIG_POS_NEG_EN;
			break;
		case POS_MODE:
			/*TCNT is cleared on the rising edge of gpio0.*/
			channel[id].clk_ext      = TCU_CLKSRC_EXT;
			channel[id].sig_ext      = SIG_POS_EN;
			/*gpio0 pos clear count*/
			channel[id].clk_gpio0    = TCU_CLKSRC_GPIO0;
			channel[id].sig_gpio0    = SIG_POS_EN;
			channel[id].pos_sel      = GPIO0_POS_CLR;
			break;
		case CAPTURE_MODE:
			/*Use ext_clk to capture the duty cycle of gpio0, result CAP_VAL register have value*/
			channel[id].clk_ext = TCU_CLKSRC_EXT;
			channel[id].sig_ext = SIG_POS_EN;

			channel[id].clk_gpio0 = TCU_CLKSRC_GPIO0;
			channel[id].capture_sel = CAPTURE_GPIO0;
			channel[id].capture_num = 0xa0;

			break;
		case FILTER_MODE:
			/*for easy test set max*/
			channel[id].clk_gpio0 = TCU_CLKSRC_GPIO0;
			channel[id].sig_gpio0 = SIG_POS_EN;
			channel[id].fil_a_num = 0x3ff;
			channel[id].fil_b_num = 0x3ff;
			break;
		default:
			break;
	}

	if (channel[id].gpio_trigger) {
		ingenic_tcu_store_mask_set(&channel[id]);
		ingenic_tcu_store_neg_enable(&channel[id]);
		ingenic_tcu_store_enable(&channel[id]);
		ingenic_tcu_store_mask_clr(&channel[id]);
	}

	ingenic_tcu_config_chn(&channel[id]);
	ingenic_tcu_enable_counter(&channel[id]);
	ingenic_tcu_start_counter(&channel[id]);

	if (!channel[id].gpio_trigger) {
		for (i = 0; i < 20 ; i++) {
			sws_pr_debug(&channel[id], i);
		}
	}
}

static irqreturn_t ingenic_tcu_interrupt(int irq, void *dev_id)
{
	struct ingenic_tcu *tcu = (struct ingenic_tcu *)(dev_id);
	if ((tcu_readl(tcu, TCU_TFR) & TCU_FLAG_RD) && !(tcu_readl(tcu, TCU_TMR) & TCU_MASK_RD)) {
		//Watchdog interrupts are not handled here
	} else {
		ingenic_tcu_irq_ack(tcu);
	}
	return IRQ_HANDLED;
}

#if (!(defined CONFIG_SOC_X2600) && !(defined CONFIG_SOC_AD100))
static irqreturn_t ingenic_tcu_trigger_interrupt(int irq, void *dev_id)
{
	struct ingenic_tcu *tcu = (struct ingenic_tcu *)(dev_id);
	struct ingenic_tcu_chn *channel = tcu->channel;
	unsigned long flags;
	unsigned long tmp;
	int i;
	int times = timeout;

	spin_lock_irqsave(&tcu->lock, flags);
	tmp = ingenic_tcu_store_flag_st(tcu);
	i = 0;
	while (tmp) {
		if (tmp & (1 << i)) {
			tmp &= ~(1 << i);
			ingenic_tcu_store_flag_clr(&channel[i]);
			printk("%d %d %d\n", __LINE__, i, ingenic_tcu_store_val(&channel[i]));
		}

		if (times-- == 0) {
			break;
		}

		i++;
	}
	spin_unlock_irqrestore(&tcu->lock, flags);

	return IRQ_HANDLED;
}
#endif

static ssize_t tcu_show_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i;
	int ret = 0;
	struct ingenic_tcu *tcu = dev_get_drvdata(dev);
	struct ingenic_tcu_chn *channel = tcu->channel;

	ret += sprintf(buf + ret, "\nmode_sel :\n");
	ret += sprintf(buf + ret, "0:GENERAL_MODE 1:GATE_MODE 2:DIRECTION_MODE 3:QUADRATURE_MODE\n");
	ret += sprintf(buf + ret, "4:POS_MODE 5:CAPTURE_MODE 6:FILTER_MODE\n\n");
	ret += sprintf(buf + ret, "####################example####################\n");
	ret += sprintf(buf + ret, "## echo channel_id mode_sel gpio_trigger > enable \n");
	ret += sprintf(buf + ret, "## echo channel_id > disable \n\n");
	for (i = 0; i < NR_TCU_CHNS ; i++) {
		if (channel[i].en_flag) {
			ret += sprintf(buf + ret, "channel: %02d enable\n", i);
		} else {
			ret += sprintf(buf + ret, "channel: %02d disable\n", i);
		}
	}
	return ret;
}

static void ingenic_tcu_reset_reg(int id, struct ingenic_tcu_chn *channel)
{
	/* Restore Register Status  */
	tcu_writel(channel->tcu, CHN_CAP(id), 0);
	tcu_writel(channel->tcu, CH_TDFR(id), 0);
	tcu_writel(channel->tcu, CH_TDHR(id), 0);
	tcu_writel(channel->tcu, CH_TCNT(id), 0);
	tcu_writel(channel->tcu, CH_TCSR(id), 0);
	tcu_writel(channel->tcu, CHN_FIL_VAL(id), 0);
}

static void ingenic_tcu_reset_channel(int id, struct ingenic_tcu_chn *channel)
{
	/* reset channel configs*/
	channel[id].capture_num  = 0;
	channel[id].fil_a_num    = 0;
	channel[id].fil_b_num    = 0;
	channel[id].clk_ext          = TCU_CLKSRC_NULL;
	channel[id].clk_gpio0    = TCU_CLKSRC_NULL;
	channel[id].clk_gpio1    = TCU_CLKSRC_NULL;
	channel[id].prescale     = TCU_PRESCALE_64;
	channel[id].count_mode   = COUNT_MODE_FCZ;
	channel[id].gate_pola    = GATE_POLA_LOW;
	channel[id].gate_sel     = GATE_SEL_HZ;
	channel[id].dir_sel          = DIR_SEL_HH;
	channel[id].dir_pola     = DIR_POLA_LOW;
	channel[id].sig_ext      = SIG_INIT;
}

static ssize_t tcu_store_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int channel_id;
	int mode_id;
	enum tcu_mode_sel mode_sel;
	const char *str = buf;
	int ret_count = 0;
	struct ingenic_tcu *tcu = dev_get_drvdata(dev);
	struct ingenic_tcu_chn *channel = tcu->channel;

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			return count;
		}
	}
	channel_id = simple_strtoul(str, (char **)&str, 10);

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			return count;
		}
	}
	mode_id = simple_strtoul(str, (char **)&str, 10);

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			channel[channel_id].gpio_trigger = 0;
		}
	}
	channel[channel_id].gpio_trigger = simple_strtoul(str, (char **)&str, 10);

	switch (mode_id) {
		case 0: mode_sel = GENERAL_MODE;
			break;
		case 1: mode_sel = GATE_MODE;
			break;
		case 2: mode_sel = DIRECTION_MODE;
			break;
		case 3: mode_sel = QUADRATURE_MODE;
			break;
		case 4: mode_sel = POS_MODE;
			break;
		case 5: mode_sel = CAPTURE_MODE;
			break;
		case 6: mode_sel = FILTER_MODE;
			break;
		default: mode_sel = GENERAL_MODE;
			break;
	}

	if (channel_id >= 0 && channel_id < 8) {
		if (channel[channel_id].en_flag) {
			printk("channel %d already enable \n", channel_id);
			return -1;
		}
		ingenic_tcu_reset_reg(channel_id, channel);
		ingenic_tcu_reset_channel(channel_id, channel);
		ingenic_tcu_config_attr(channel_id, mode_sel, channel);
	} else {
		printk("Please select the correct channel 0 ~ 7 range.\n");
	}
	return count;
}

static ssize_t tcu_store_disable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int id;
	const char *str = buf;
	int ret_count = 0;
	struct ingenic_tcu *tcu = dev_get_drvdata(dev);
	struct ingenic_tcu_chn *channel = tcu->channel;

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			return count;
		}
	}

	id = simple_strtoul(str, (char **)&str, 10);

	if (id >= 0 && id < 8) {
		if (!channel[id].en_flag) {
			printk("channel %d already disable \n", id);
			return -1;
		}

		if (channel[id].gpio_trigger) {
			ingenic_tcu_store_mask_clr(&channel[id]);
			ingenic_tcu_store_neg_disable(&channel[id]);
			ingenic_tcu_store_disable(&channel[id]);
		}
		ingenic_tcu_set_disable(&channel[id]);
	} else {
		printk("Please select the correct channel 0 ~ 7 range.\n");
	}
	return count;
}

static struct device_attribute tcu_device_attributes[] = {
	__ATTR(enable, S_IRUGO | S_IWUSR, tcu_show_enable, tcu_store_enable),
	__ATTR(disable, S_IRUGO | S_IWUSR, tcu_show_enable, tcu_store_disable),
};

static int ingenic_tcu_probe(struct platform_device *pdev)
{
	struct ingenic_tcu *tcu;
	int  i, ret = 0;
	struct ingenic_tcu_chn *channel = NULL;

	tcu = kmalloc(sizeof(struct ingenic_tcu), GFP_KERNEL);
	if (!tcu) {
		dev_err(&pdev->dev, "Failed to allocate driver struct\n");
		return -ENOMEM;
	}

	tcu->dev = &pdev->dev;

	channel = kmalloc(sizeof(struct ingenic_tcu_chn) * NR_TCU_CHNS, GFP_KERNEL);
	if (!channel) {
		dev_err(&pdev->dev, "Failed to allocate channel struct\n");
		return -ENOMEM;
	}
	tcu->channel = channel;

	tcu->irq = platform_get_irq(pdev, 0);
	if (tcu->irq < 0) {
		dev_err(&pdev->dev, "Failed to get platform irq\n");
		ret = tcu->irq;
		goto err_free;
	}

#if (!(defined CONFIG_SOC_X2600) && !(defined CONFIG_SOC_AD100))
	tcu->irq_trigger = platform_get_irq(pdev, 1);
	if (tcu->irq_trigger < 0) {
		dev_err(&pdev->dev, "not support irq trigger function\n");
		ret = tcu->irq_trigger;
	} else {
		ret = request_irq(tcu->irq_trigger, ingenic_tcu_trigger_interrupt,
		                  IRQF_SHARED | IRQF_TRIGGER_LOW, "ingenic-tcu-tri-interrupt", tcu);

		if (ret) {
			dev_err(&pdev->dev, "request_irq failed !! %d-\n", tcu->irq_trigger);
		}
	}
#endif

	tcu->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!tcu->res) {
		dev_err(&pdev->dev, "No iomem resource\n");
		ret = -ENXIO;
		goto err_ioremap;
	}

	tcu->clk = of_clk_get(pdev->dev.of_node, 0);
	if (!tcu->clk) {
		dev_err(&pdev->dev, "get clk from dts err\n");
		goto err_mfd_add;
	}
	clk_prepare_enable(tcu->clk);

	tcu->iomem = ioremap(tcu->res->start, resource_size(tcu->res));
	if (!tcu->iomem) {
		goto err_mfd_add;
	}

	spin_lock_init(&tcu->lock);

	ret = request_irq(tcu->irq, ingenic_tcu_interrupt,
	                  IRQF_SHARED | IRQF_TRIGGER_LOW, "ingenic-tcu-interrupt", tcu);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed !! %d-\n", tcu->irq);
		goto err_mfd_add;
	}

	platform_set_drvdata(pdev, tcu);

	for (i = 0; i < NR_TCU_CHNS; i++) {
		channel[i].index = i;
		channel[i].capture_num = 0;
		channel[i].count_value = 0;
		channel[i].fil_a_num     = 0;
		channel[i].fil_b_num     = 0;
		channel[i].reg_base  = TCU_FULL0 + i * TCU_CHN_OFFSET;
		channel[i].irq_type  = NULL_IRQ_MODE;
		channel[i].clk_ext   = TCU_CLKSRC_NULL;
		channel[i].clk_gpio0     = TCU_CLKSRC_NULL;
		channel[i].clk_gpio1     = TCU_CLKSRC_NULL;
		channel[i].prescale  = TCU_PRESCALE_64;
		channel[i].shutdown_mode = 0;
		channel[i].count_mode    = COUNT_MODE_FCZ;
		channel[i].mode_sel  = GENERAL_MODE;
		channel[i].gate_pola     = GATE_POLA_LOW;
		channel[i].gate_sel  = GATE_SEL_HZ;
		channel[i].dir_sel   = DIR_SEL_HH;
		channel[i].dir_pola  = DIR_POLA_LOW;
		channel[i].en_flag   = 0;
		channel[i].tcu   = tcu;
		channel[i].gpio_trigger = 0;
	}

	for (i = 0; i < ARRAY_SIZE(tcu_device_attributes); i++) {
		ret = device_create_file(&pdev->dev, &tcu_device_attributes[i]);
		if (ret) {
			dev_warn(&pdev->dev, "attribute %d create failed", i);
		}
	}

	dev_info(&pdev->dev, "ingenic TCU driver register completed\n");
	return 0;

err_mfd_add:
	iounmap(tcu->iomem);
err_ioremap:
	release_resource(tcu->res);
err_free:
	kfree(tcu);
	return ret;
}

static int ingenic_tcu_remove(struct platform_device *pdev)
{
	struct ingenic_tcu *tcu = platform_get_drvdata(pdev);

	clk_disable_unprepare(tcu->clk);
	iounmap(tcu->iomem);
	release_resource(tcu->res);
	platform_set_drvdata(pdev, NULL);
	free_irq(tcu->irq, tcu);
	kfree(tcu);

	return 0;
}
static const struct of_device_id tcu_match[] = {
	{ .compatible = "ingenic,tcu", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, tcu_match);

static struct platform_driver ingenic_tcu_driver = {
	.probe  = ingenic_tcu_probe,
	.remove = ingenic_tcu_remove,
	.driver = {
		.name = "ingenic-tcu",
		.of_match_table = tcu_match,
	},
};

module_platform_driver(ingenic_tcu_driver);

MODULE_AUTHOR("wssong <wenshuo.song@ingenic.com>");
MODULE_DESCRIPTION("Ingenic TCU driver");
MODULE_ALIAS("platform:x2000_v2-tcu");
MODULE_LICENSE("GPL v2");
