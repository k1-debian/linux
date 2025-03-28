/* drivers/pwm/pwm-ingenic-x1600.c
 * PWM driver of Ingenic's SoC X1600
 *
 * Copyright (C) 2015 Ingenic Semiconductor Co., Ltd.
 *  http://www.ingenic.com
 * Author:  sihui.liu <sihui.liu@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/clk.h>
#include <linux/ctype.h>
#include <linux/stat.h>
#include <linux/pwm.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm-generic/div64.h>
#include <linux/sysfs.h>
#include <irq.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <dt-bindings/dma/ingenic-pdma.h>

#include <pwm-ingenic-v3.h>

/**
 **     pwm basic operation configuration interface
 **/
static void pwm_writel(struct ingenic_pwm_chip *ingenic_pwm, unsigned int value, unsigned int offset)
{
	writel(value, ingenic_pwm->iomem + offset);
}

static unsigned int pwm_readl(struct ingenic_pwm_chip *ingenic_pwm, unsigned int offset)
{
	return readl(ingenic_pwm->iomem + offset);
}

static unsigned int pwm_store_status(struct ingenic_pwm_chip *ingenic_pwm)
{
	return pwm_readl(ingenic_pwm, PWM_SS);
}

static void pwm_store_clear_status(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	pwm_writel(ingenic_pwm, 1 << channel, PWM_SS);
}

static void pwm_trigger_enable_interrupt(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SIE);
	pwm_writel(ingenic_pwm, val | (1 << channel), PWM_SIE);
}

static void pwm_store_disable_interrupt(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SIE);
	pwm_writel(ingenic_pwm, val & ~(1 << channel), PWM_SIE);
}

static void pwm_trigger_enable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SC(channel));
	pwm_writel(ingenic_pwm, val | PWM_STORE_SE, PWM_SC(channel));
}

static void pwm_store_disable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SC(channel));
	pwm_writel(ingenic_pwm, val & ~PWM_STORE_SE, PWM_SC(channel));
}

static void pwm_trigger_posedge_enable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SC(channel));
	pwm_writel(ingenic_pwm, val | PWM_STORE_SPE, PWM_SC(channel));
}

static void pwm_trigger_posedge_disable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SC(channel));
	pwm_writel(ingenic_pwm, val & ~PWM_STORE_SPE, PWM_SC(channel));
}
static void pwm_trigger_negedge_enable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SC(channel));
	pwm_writel(ingenic_pwm, val | PWM_STORE_SNE, PWM_SC(channel));
}

static void pwm_trigger_negedge_disable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SC(channel));
	pwm_writel(ingenic_pwm, val & ~PWM_STORE_SNE, PWM_SC(channel));
}

static void pwm_trigger_filter_set(struct ingenic_pwm_chip *ingenic_pwm,
                                   unsigned int channel,
                                   unsigned int number)
{
	unsigned int val;
	val = pwm_readl(ingenic_pwm, PWM_SC(channel));
	pwm_writel(ingenic_pwm, (val & ~PWM_STORE_SFN_MASK) | number, PWM_SC(channel));
}

static unsigned int pwm_get_store_num(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	return pwm_readl(ingenic_pwm, PWM_SN(channel));
}

static unsigned int pwm_get_output_num(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	return pwm_readl(ingenic_pwm, PWM_ON(channel));
}

static void pwm_clk_config(struct ingenic_pwm_chip *ingenic_pwm,
                           unsigned int channel, unsigned int prescale)
{
	pwm_writel(ingenic_pwm, prescale, PWM_CCFG(channel));
}
/*
static int pwm_get_prescale(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
    return pwm_readl(ingenic_pwm, PWM_CCFG(channel));
}
*/
static void pwm_enable_hw(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	pwm_writel(ingenic_pwm, 1 << channel, PWM_ENS);
}

static void pwm_disable_hw(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	pwm_writel(ingenic_pwm, 1 << channel, PWM_ENC);
}

static int pwm_enable_status(struct ingenic_pwm_chip *ingenic_pwm)
{
	return pwm_readl(ingenic_pwm, PWM_EN);
}
static void pwm_update(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	pwm_writel(ingenic_pwm, 1 << channel, PWM_UPT);
}

static int pwm_busy(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp = 0;
	tmp = pwm_readl(ingenic_pwm, PWM_BUSY);
	return tmp & (1 << channel);
}

static void pwm_set_init_level(struct ingenic_pwm_chip *ingenic_pwm,
                               unsigned int channel, unsigned int level)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm, PWM_INL);

	if (level) {
		tmp |= 1 << channel;
	} else {
		tmp &= ~(1 << channel);
	}
	pwm_writel(ingenic_pwm, tmp, PWM_INL);
}

static void pwm_set_finish_level(struct ingenic_pwm_chip *ingenic_pwm,
                                 unsigned int channel, unsigned int level)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm, PWM_IDL);
	if (level) {
		tmp |= 1 << channel;
	} else {
		tmp &= ~(1 << channel);
	}
	pwm_writel(ingenic_pwm, tmp, PWM_IDL);
}

static void pwm_waveform_high(struct ingenic_pwm_chip *ingenic_pwm,
                              unsigned int channel, unsigned int high_num)
{
	int tmp = 0;
	tmp = pwm_readl(ingenic_pwm, PWM_WCFG(channel));
	tmp &= ~(0xffff << PWM_WCFG_HIGH);
	tmp |= high_num << PWM_WCFG_HIGH;
	pwm_writel(ingenic_pwm, tmp, PWM_WCFG(channel));
}

static void pwm_waveform_low(struct ingenic_pwm_chip *ingenic_pwm,
                             unsigned int channel, unsigned int low_num)
{
	int tmp = 0;
	tmp = pwm_readl(ingenic_pwm, PWM_WCFG(channel));
	tmp &= ~(0xffff);
	tmp |= low_num << PWM_WCFG_LOW;
	pwm_writel(ingenic_pwm, tmp, PWM_WCFG(channel));
}

/*pwm DMA mode control register operation*/
static void pwm_dma_enable_hw(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp = 0;
	tmp = pwm_readl(ingenic_pwm, PWM_DRE);
	tmp |= (1 << channel);
	pwm_writel(ingenic_pwm, tmp, PWM_DRE);
}

static void pwm_dma_disable_hw(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp = 0;
	tmp = pwm_readl(ingenic_pwm, PWM_DRE);
	tmp &= ~(1 << channel);
	pwm_writel(ingenic_pwm, tmp, PWM_DRE);
}

static void pwm_set_update_mode(struct ingenic_pwm_chip *ingenic_pwm,
                                unsigned int channel, int mode_sel)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm, PWM_MS);
	if (mode_sel == DMA_MODE_SG || mode_sel == DMA_MODE_CYCLIC || mode_sel == DMA_MODE_SMC) {
		tmp |= 1 << channel;
	}
	if (mode_sel == COMMON_MODE) {
		tmp &= ~(1 << channel);
	}
	pwm_writel(ingenic_pwm, tmp, PWM_MS);
}
/*
static void pwm_dma_under_irq_enable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
    int tmp;
    tmp = pwm_readl(ingenic_pwm, PWM_DFIE);
    tmp |= (1 << channel);
    pwm_writel(ingenic_pwm, tmp, PWM_DFIE);
}

static void pwm_dma_under_irq_disable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
    int tmp;
    tmp = pwm_readl(ingenic_pwm, PWM_DFIE);
    tmp &= ~(1 << channel);
    pwm_writel(ingenic_pwm, tmp, PWM_DFIE);
}
*/
static unsigned int pwm_dma_under_status(struct ingenic_pwm_chip *ingenic_pwm)
{
	return pwm_readl(ingenic_pwm, PWM_DFS);
}

static void pwm_dma_clear_under(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm, PWM_DFS);
	tmp |= (1 << channel);
	pwm_writel(ingenic_pwm, tmp, PWM_DFS);
}

static void pwm_set_fifo_throshold(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel, int mode)
{
	if (DMA_MODE_SMC == mode) {
		pwm_writel(ingenic_pwm, 64, PWM_DRTN(channel));
	} else {
		pwm_writel(ingenic_pwm, 32, PWM_DRTN(channel));
	}
}

static void pwm_dma_fifo_flush(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	pwm_writel(ingenic_pwm, 1 << channel, PWM_DCFF);

	while (!!(pwm_readl(ingenic_pwm, PWM_DCFF) & (1 << channel)));
}

#ifdef CONFIG_PWM_DUMP
__attribute__((__unused__)) static void dump_pwm_reg(struct ingenic_pwm_chip *ingenic_pwm, int channel)
{
	int i = channel;
	printk("PWM_EN(%08x)	= %08x\n", PWM_EN, pwm_readl(ingenic_pwm, PWM_EN));
	printk("PWM_UPT(%08x)	= %08x\n", PWM_UPT, pwm_readl(ingenic_pwm, PWM_UPT));
	printk("PWM_BUSY(%08x)  = %08x\n", PWM_BUSY, pwm_readl(ingenic_pwm, PWM_BUSY));
	printk("PWM_MS(%08x)	= %08x\n", PWM_MS, pwm_readl(ingenic_pwm, PWM_MS));
	printk("PWM_INL(%08x)	= %08x\n", PWM_INL, pwm_readl(ingenic_pwm, PWM_INL));
	printk("PWM_IDL(%08x)  = %08x\n", PWM_IDL, pwm_readl(ingenic_pwm, PWM_IDL));
	printk("PWM_DRS(%08x)  = %08x\n", PWM_DRS, pwm_readl(ingenic_pwm, PWM_DRS));
	printk("PWM_DFIE(%08x)  = %08x\n", PWM_DFIE, pwm_readl(ingenic_pwm, PWM_DFIE));
	printk("PWM_DFS(%08x)  = %08x\n", PWM_DFS, pwm_readl(ingenic_pwm, PWM_DFS));
	printk("PWM_DRE(%08x)  = %08x\n", PWM_DRE, pwm_readl(ingenic_pwm, PWM_DRE));
	printk("PWM_SS(%08x)  = %08x\n", PWM_SS, pwm_readl(ingenic_pwm, PWM_SS));
	printk("PWM_SIE(%08x)  = %08x\n", PWM_SIE, pwm_readl(ingenic_pwm, PWM_SIE));

	printk("\n>>>Channel-%d store_num = (0x%x)%d:\n", i, ingenic_pwm->chan[i].store_irq_num, ingenic_pwm->chan[i].store_irq_num);
	printk("PWM_CCFG(%08x)  = %08x\n", PWM_CCFG(i), pwm_readl(ingenic_pwm, PWM_CCFG(i)));
	printk("PWM_WCFG(%08x)  = %08x\n", PWM_WCFG(i), pwm_readl(ingenic_pwm, PWM_WCFG(i)));
	printk("PWM_DR(%08x)  = %08x\n", PWM_DR(i), pwm_readl(ingenic_pwm, PWM_DR(i)));
	printk("PWM_DFN(%08x)  = %08x\n", PWM_DFN(i), pwm_readl(ingenic_pwm, PWM_DFN(i)));
	printk("PWM_DRTN(%08x)  = %08x\n\n", PWM_DRTN(i), pwm_readl(ingenic_pwm, PWM_DRTN(i)));
	printk("PWM_SC(%08x)  = %08x\n\n", PWM_SC(i), pwm_readl(ingenic_pwm, PWM_SC(i)));
	printk("PWM_SN(%08x)  = %08x\n\n", PWM_SN(i), pwm_readl(ingenic_pwm, PWM_SN(i)));
	printk("PWM_ON(%08x)  = %08x\n\n", PWM_ON(i), pwm_readl(ingenic_pwm, PWM_ON(i)));
}
#endif

static void dma_tx_callback(void *data)
{
	struct ingenic_pwm_chan *ingenic_chan = data;
	/* print_dbg(">>%s %d\n",__func__,__LINE__); */

	if (ingenic_chan->callback != NULL) {
		ingenic_chan->callback(ingenic_chan->callback_param);
	}
}

static int ingenic_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ingenic_pwm_chip *ingenic_pwm = to_ingenic_chip(chip);
	struct ingenic_pwm_chan *ingenic_chan = NULL;
	int channel = pwm->hwpwm;
	ingenic_pwm->debug_current_id = channel;
	ingenic_pwm->debug_pwm[channel] = pwm;
	ingenic_chan = &ingenic_pwm->chan[channel];
	ingenic_chan->clk_in = clk_get_rate(ingenic_pwm->clk_pwm);
	pwm_set_chip_data(pwm, ingenic_chan);
	return 0;
}

static int ingenic_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
                              int duty_ns, int period_ns)
{
	struct ingenic_pwm_chip *ingenic_pwm = to_ingenic_chip(chip);
	int channel = pwm->hwpwm;
	struct ingenic_pwm_chan *ingenic_chan = &ingenic_pwm->chan[channel];
	unsigned int period = 0;
	unsigned int duty = 0;
	unsigned int clk_in = 0;
	unsigned int prescale = 0;
	unsigned long long tmp;
	unsigned int period_tmp = 0;
	int mode = 0;

	mutex_lock(&ingenic_pwm->mutex);
	if (duty_ns < 0 || duty_ns > period_ns) {
		pr_err("%s, duty_ns(%d)< 0 or duty_ns > period_ns(%d)\n", __func__, duty_ns, period_ns);
		mutex_unlock(&ingenic_pwm->mutex);
		return -EINVAL;
	}

	if (duty_ns < 0 || duty_ns > period_ns) {
		mutex_unlock(&ingenic_pwm->mutex);
		return -ERANGE;
	}

	/*select mode */
	mode = ingenic_chan->mode;
	ingenic_chan->duty_ns = duty_ns;
	ingenic_chan->period_ns = period_ns;

	/*set prescale*/
	clk_in = clk_get_rate(ingenic_pwm->clk_pwm);
	if (ingenic_chan->sys_prescale) {
		clk_in /= ingenic_chan->sys_prescale;
	}
	/* period = period_ns / (1000000000 / clk_in) */
	tmp = (unsigned long long)clk_in * period_ns;
	do_div(tmp, 1000000000);
	period = tmp;
	period_tmp = tmp;

	if (!ingenic_chan->sys_prescale) {
		while (period > 0xffff && prescale < ingenic_pwm->chip_priv->prescale) {
			if (ingenic_pwm->chip_priv->version_num == 1) {
				period >>= 1;
				++prescale;
			} else {
				prescale += 1;
				period = period_tmp / prescale;
			}

		}
	}
	if ((ingenic_pwm->chip_priv->version_num == 1 && prescale == ingenic_pwm->chip_priv->prescale) || (ingenic_pwm->chip_priv->version_num == 2 && prescale > ingenic_pwm->chip_priv->prescale)) {
		mutex_unlock(&ingenic_pwm->mutex);
		return -EINVAL;
	}

	if (ingenic_pwm->chip_priv->version_num == 2 && prescale >= 1) {
		prescale -= 1;    //feq = inclk/(prescale+1)
	}
	/* duty = period / (period_ns / duty_ns) */
	tmp = (unsigned long long)period * duty_ns;
	do_div(tmp, period_ns);
	duty = tmp;

	print_dbg("period_ns=%d duty_ns = %d high_ns = %d clk_in = %u\n",
	          period_ns, duty_ns,
	          period_ns - duty_ns,
	          clk_in);
	print_dbg("period = %d LOW=%d HIGH=%d init_level=%d finsh_level = %d div = %d\n",
	          period, duty, period - duty,
	          ingenic_chan->init_level,
	          ingenic_chan->finish_level,
	          prescale);
	print_dbg("trigger_en = %d negedge = %d posedge  =%d filter = %d\n",
	          ingenic_chan->trigger_en,
	          ingenic_chan->trigger_posedge,
	          ingenic_chan->trigger_negedge,
	          ingenic_chan->trigger_filter);
	if (duty_ns == period_ns) {
		ingenic_chan->finish_level = !ingenic_chan->init_level;
		ingenic_chan->full_duty_status = 1;
		ingenic_chan->en_status = 1;
	} else if (duty_ns == 0) {
		ingenic_chan->finish_level = ingenic_chan->init_level;
		ingenic_chan->full_duty_status = 1;
		ingenic_chan->en_status = 1;
	} else {
		ingenic_chan->full_duty_status = 0;
		ingenic_chan->en_status = 0;
	}

	/*set init level*/
	if (!ingenic_chan->sys_prescale) {
		pwm_clk_config(ingenic_pwm, channel, prescale);
	}
	pwm_set_init_level(ingenic_pwm, channel, ingenic_chan->init_level);
	pwm_set_finish_level(ingenic_pwm, channel, ingenic_chan->finish_level);
	pwm_set_update_mode(ingenic_pwm, channel, mode);
	if (ingenic_chan->trigger_en) {
		pwm_trigger_enable(ingenic_pwm, channel);
		if (ingenic_chan->trigger_negedge) {
			pwm_trigger_negedge_enable(ingenic_pwm, channel);
		} else {
			pwm_trigger_negedge_disable(ingenic_pwm, channel);
		}
		if (ingenic_chan->trigger_posedge) {
			pwm_trigger_posedge_enable(ingenic_pwm, channel);
		} else {
			pwm_trigger_posedge_disable(ingenic_pwm, channel);
		}
		pwm_trigger_enable_interrupt(ingenic_pwm, channel);
		pwm_trigger_filter_set(ingenic_pwm, channel, ingenic_chan->trigger_filter);
	}

	/*set waveform high_num and low_num*/
	if (COMMON_MODE == mode) {
		pwm_waveform_high(ingenic_pwm, channel, duty);
		pwm_waveform_low(ingenic_pwm, channel, period - duty);

		if (pwm_enable_status(ingenic_pwm) & 1 << channel) {
			if (ingenic_chan->full_duty_status && ingenic_chan->en_status) {
				pwm_disable_hw(ingenic_pwm, channel);
			} else {
				pwm_update(ingenic_pwm, channel);
				while (pwm_busy(ingenic_pwm, channel));
			}
		}

		if ((ingenic_chan->flags & PWM_ENABLE_FLAG) && (ingenic_chan->full_duty_status | ingenic_chan->en_status) == 0) {
			pwm_enable_hw(ingenic_pwm, channel);
		}
	}

	if (DMA_MODE_SMC == mode) {
		pwm_set_fifo_throshold(ingenic_pwm, channel, mode);
		pwm_dma_fifo_flush(ingenic_pwm, channel);
	}

	if (DMA_MODE_SG == mode || DMA_MODE_CYCLIC == mode) {
		struct dma_slave_config tx_config;
		struct dma_async_tx_descriptor *txdesc;
		struct dma_chan *txchan = ingenic_chan->dma_chan;

		tx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		tx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		tx_config.dst_maxburst = 4;
		tx_config.src_maxburst = 4;
		tx_config.dst_addr = (dma_addr_t)(ingenic_pwm->phys + PWM_DR(channel));
		tx_config.direction = DMA_MEM_TO_DEV;
		dmaengine_slave_config(txchan, &tx_config);

#if 0
		memset(ingenic_chan->buffer, 0, BUFFER_SIZE);
		wave = ingenic_chan->buffer;
		val = duty | ((period - duty) << 16);
		for (i = 0; i < BUFFER_SIZE / 4; i++) {
			wave[i] = val;
		}
#endif
		if (mode == DMA_MODE_CYCLIC) {
			txdesc = txchan->device->device_prep_dma_cyclic(txchan,
			         ingenic_chan->buffer_dma,
			         ingenic_chan->total_buffer_size,
			         ingenic_chan->frame_size,
			         DMA_MEM_TO_DEV,
			         DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		} else {
			sg_init_one(ingenic_chan->sg, ingenic_chan->buffer, ingenic_chan->sg_pwm_num * 4);
			if (dma_map_sg(ingenic_pwm->chip.dev,
			               ingenic_chan->sg, 1, DMA_TO_DEVICE) != 1) {
				dev_err(ingenic_pwm->chip.dev, "dma_map_sg tx error\n");
			}

			txdesc = dmaengine_prep_slave_sg(txchan,
			                                 ingenic_chan->sg,
			                                 1,
			                                 DMA_MEM_TO_DEV,
			                                 DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		}
		if (!txdesc) {
			dev_err(ingenic_pwm->chip.dev, "PWM request dma desc failed");
		}

		txdesc->callback = dma_tx_callback;
		txdesc->callback_param = ingenic_chan;
		dmaengine_submit(txdesc);

		pwm_set_fifo_throshold(ingenic_pwm, channel, mode);
		pwm_dma_fifo_flush(ingenic_pwm, channel);

	}
	mutex_unlock(&ingenic_pwm->mutex);
	return 0;
}

static int ingenic_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ingenic_pwm_chip *ingenic_pwm = to_ingenic_chip(chip);
	int channel = pwm->hwpwm;
	struct ingenic_pwm_chan *ingenic_chan = &ingenic_pwm->chan[channel];

	ingenic_chan->flags &= ~(PWM_ENABLE_FLAG);
	ingenic_chan->flags |= PWM_ENABLE_FLAG;

	if (ingenic_chan->full_duty_status) {
		return 0;
	}

	mutex_lock(&ingenic_pwm->mutex);
	ingenic_chan->store_irq_num = 0;
	if (DMA_MODE_SG == ingenic_chan->mode ||
	    DMA_MODE_CYCLIC == ingenic_chan->mode || DMA_MODE_SMC == ingenic_chan->mode) {
		pwm_dma_enable_hw(ingenic_pwm, channel);
		pwm_dma_fifo_flush(ingenic_pwm, channel);
	}

	pwm_enable_hw(ingenic_pwm, channel);

	if (ingenic_chan->mode == DMA_MODE_SG ||
	    ingenic_chan->mode == DMA_MODE_CYCLIC) {
		dma_async_issue_pending(ingenic_chan->dma_chan);
		//pwm_dma_under_irq_enable(ingenic_pwm, channel);
	}

	mutex_unlock(&ingenic_pwm->mutex);

	return 0;
}

static void ingenic_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ingenic_pwm_chip *ingenic_pwm = to_ingenic_chip(chip);
	int channel = pwm->hwpwm;
	struct ingenic_pwm_chan *ingenic_chan = &ingenic_pwm->chan[channel];

	mutex_lock(&ingenic_pwm->mutex);

	if (ingenic_chan->mode == DMA_MODE_SG || ingenic_chan->mode == DMA_MODE_CYCLIC) {
		dmaengine_terminate_all(ingenic_chan->dma_chan);

		if (ingenic_chan->mode == DMA_MODE_SG) {
			dma_unmap_sg(ingenic_pwm->chip.dev, ingenic_chan->sg, 1, DMA_TO_DEVICE);
		}

		//pwm_dma_under_irq_disable(ingenic_pwm,channel);

		pwm_dma_disable_hw(ingenic_pwm, channel);
	}

	if (ingenic_chan->mode == DMA_MODE_SMC) {
		pwm_dma_disable_hw(ingenic_pwm, channel);
	}

	if (ingenic_chan->trigger_en) {
		pwm_store_disable(ingenic_pwm, channel);
		pwm_trigger_negedge_disable(ingenic_pwm, channel);
		pwm_trigger_posedge_disable(ingenic_pwm, channel);
		pwm_store_disable_interrupt(ingenic_pwm, channel);
		pwm_trigger_filter_set(ingenic_pwm, channel, 0);
	}
	pwm_disable_hw(ingenic_pwm, channel);
	ingenic_chan->flags &= ~(PWM_ENABLE_FLAG);
	mutex_unlock(&ingenic_pwm->mutex);
}

void ingenic_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ingenic_pwm_chip *ingenic_pwm = to_ingenic_chip(chip);
	int channel = pwm->hwpwm;

	ingenic_pwm->debug_pwm[channel] = NULL;
	pwm->state.period = 0;
	pwm->state.duty_cycle = 0;
}

static int ingenic_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm, enum pwm_polarity polarity)
{
	struct ingenic_pwm_chip *ingenic_pwm = to_ingenic_chip(chip);
	mutex_lock(&ingenic_pwm->mutex);
	ingenic_pwm->chan->init_level = polarity;
	mutex_unlock(&ingenic_pwm->mutex);
	return 0;
}

static int ingenic_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
                             const struct pwm_state *state)
{
	int err;
	bool enabled = pwm->state.enabled;

	if (state->polarity != pwm->state.polarity) {
		if (enabled) {
			ingenic_pwm_disable(chip, pwm);
			enabled = false;
		}
		err = ingenic_pwm_set_polarity(chip, pwm, state->polarity);
		if (err) {
			return err;
		}
	}

	if (!state->enabled) {
		if (enabled) {
			ingenic_pwm_disable(chip, pwm);
		}
		return 0;
	}

	err = ingenic_pwm_config(chip, pwm, state->duty_cycle, state->period);
	if (err) {
		return err;
	}

	if (!enabled) {
		return ingenic_pwm_enable(chip, pwm);
	}
	return 0;
}

static const struct pwm_ops ingenic_pwm_ops = {
	.request = ingenic_pwm_request,
	.free = ingenic_pwm_free,
	.apply = ingenic_pwm_apply,
	.owner = THIS_MODULE,
};

static ssize_t pwm_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	return sprintf(buf, "0x%x", pwm_enable_status(ingenic_pwm));
}

static ssize_t pwm_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan = &ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	int enable;
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			return count;
		}
	}
	enable = simple_strtoul(str, (char **)&str, 10);

	print_dbg("input enable value %d,current_id %d\n", enable, ingenic_pwm->debug_current_id);
	if (enable) {
		pwm_config(ingenic_pwm->debug_pwm[ingenic_pwm->debug_current_id],
		           ingenic_chan->duty_ns, ingenic_chan->period_ns);
		pwm_enable(ingenic_pwm->debug_pwm[ingenic_pwm->debug_current_id]);
	} else {
		pwm_disable(ingenic_pwm->debug_pwm[ingenic_pwm->debug_current_id]);
	}
	return count;
}

static ssize_t pwm_prescale_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan = &ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	int prescale;
	int channel = ingenic_pwm->debug_current_id;
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			return count;
		}
	}
	prescale = simple_strtoul(str, (char **)&str, 10);
	if (prescale < 1 || prescale > 65536) {
		printk("The allowed range for prescale is 1-65536 !!! The current prescale is %d\n", prescale);
		return -1;
	} else {
		pwm_clk_config(ingenic_pwm, channel, prescale - 1);
	}
	ingenic_chan->sys_prescale = prescale;

	return count;
}

static ssize_t pwm_channel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);

	return sprintf(buf, "channel = %x", ingenic_pwm->debug_current_id);

}

static ssize_t pwm_channel_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	const char *str = buf;
	int ret_count = 0;
	int pwm_id = 0;
	struct pwm_device *pwm = NULL;

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			return count;
		}
	}
	pwm_id = simple_strtoul(str, (char **)&str, 10);
	//find the correct PWM device from the PWM device array
	struct pwm_chip *chip = dev_get_drvdata(dev);
	pwm = &chip->pwms[pwm_id];

	if (IS_ERR(pwm)) {
		printk("unable to request pwm\n");
		return -1;
	}
	ingenic_pwm->debug_pwm[pwm_id] = pwm;
	ingenic_pwm->debug_current_id = pwm_id;
	print_dbg("channel = %d\n", ingenic_pwm->debug_current_id);
	return count;
}

static ssize_t pwm_free_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	const char *str = buf;
	int ret_count = 0;
	int pwm_id = 0;

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			return count;
		}
	}
	pwm_id = simple_strtoul(str, (char **)&str, 10);

	pwm_disable(ingenic_pwm->debug_pwm[pwm_id]);
	ingenic_pwm->debug_pwm[pwm_id] = NULL;
	return count;
}

static ssize_t pwm_show_requested_channel(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	int i = 0;
	int ret = 0;

	for (i = 0; i < ingenic_pwm->chip_priv->npwm ; i++) {
		if (ingenic_pwm->debug_pwm[i] == &(ingenic_pwm->chip.pwms[i])) {
			ret += sprintf(buf + ret, "ch: %02d requested\n", i);
		} else {
			ret += sprintf(buf + ret, "ch: %02d unrequested\n", i);
		}
	}
	return ret;
}

static ssize_t pwm_duty_ns_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		    &ingenic_pwm->chan[ingenic_pwm->debug_current_id];

	return sprintf(buf, "%d\n", ingenic_chan->duty_ns);
}

static ssize_t pwm_duty_ns_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		    &ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			dev_err(dev, "NO duty_ns config!\n");
			return count;
		}
	}
	ingenic_chan->duty_ns = simple_strtoul(str, (char **)&str, 10);

	return count;
}
/*
static ssize_t pwm_sg_pwm_num_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
    struct ingenic_pwm_chan *ingenic_chan =
            &ingenic_pwm->chan[ingenic_pwm->debug_current_id];

    return sprintf(buf, "%d\n", ingenic_chan->sg_pwm_num);
}

static ssize_t pwm_sg_pwm_num_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
    struct ingenic_pwm_chan *ingenic_chan =
            &ingenic_pwm->chan[ingenic_pwm->debug_current_id];
    const char *str = buf;
    int ret_count = 0;

    while (!isxdigit(*str)) {
        str++;
        if (++ret_count >= count) {
            dev_err(dev, "NO sg_pwm_num config!\n");
            return count;
        }
    }
    ingenic_chan->sg_pwm_num = simple_strtoul(str, (char **)&str, 10);

    return count;
}
*/
static ssize_t pwm_period_ns_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		    &ingenic_pwm->chan[ingenic_pwm->debug_current_id];

	return sprintf(buf, "%d\n", ingenic_chan->period_ns);
}

static ssize_t pwm_period_ns_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		    &ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			dev_err(dev, "NO period_ns config!\n");
			return count;
		}
	}
	ingenic_chan->period_ns = simple_strtoul(str, (char **)&str, 10);

	return count;
}
/*
static ssize_t pwm_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
    struct ingenic_pwm_chan *ingenic_chan =
            &ingenic_pwm->chan[ingenic_pwm->debug_current_id];
    int ret;

    ret = sprintf(buf, "0 --> CPU mode, Continuous waveform.\n"
                  "1 --> DMA mode,Specified number of waveforms.\n"
                  "2 --> DMA mode,Continuous waveform.\n");
    ret += sprintf(buf + ret, "current mode: %d\n", ingenic_chan->mode);
    return ret;
}

static ssize_t pwm_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
    struct ingenic_pwm_chan *ingenic_chan =
            &ingenic_pwm->chan[ingenic_pwm->debug_current_id];
    const char *str = buf;
    int ret_count = 0;

    while (!isxdigit(*str)) {
        str++;
        if (++ret_count >= count) {
            dev_err(dev, "NO mode config!\n");
            return count;
        }
    }
    ingenic_chan->mode = simple_strtoul(str, (char **)&str, 10);

    return count;
}
*/
static ssize_t pwm_init_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		    &ingenic_pwm->chan[ingenic_pwm->debug_current_id];

	return sprintf(buf, "%d\n", ingenic_chan->init_level);
}

static ssize_t pwm_init_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		    &ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			dev_err(dev, "NO init_level config!\n");
			return count;
		}
	}
	ingenic_chan->init_level = simple_strtoul(str, (char **)&str, 10);

	return count;
}

static ssize_t pwm_finish_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		    &ingenic_pwm->chan[ingenic_pwm->debug_current_id];

	return sprintf(buf, "%d\n", ingenic_chan->finish_level);
}

static ssize_t pwm_finish_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		    &ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			dev_err(dev, "NO finish_level config!\n");
			return count;
		}
	}
	ingenic_chan->finish_level = simple_strtoul(str, (char **)&str, 10);

	return count;
}

static ssize_t pwm_trigger_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan =
		    &ingenic_pwm->chan[ingenic_pwm->debug_current_id];

	return sprintf(buf, "trigger_en:%d trigger_negedge:%d trigger_posedge:%d trigger_filter:%d\n",
	               ingenic_chan->trigger_en, ingenic_chan->trigger_negedge,
	               ingenic_chan->trigger_posedge, ingenic_chan->trigger_filter);
}

static ssize_t pwm_trigger_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	struct ingenic_pwm_chan *ingenic_chan = &ingenic_pwm->chan[ingenic_pwm->debug_current_id];
	const char *str = buf;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			dev_err(dev, "No trigger_en config!\n");
			ingenic_chan->trigger_en = 0;
			return count;
		}
	}
	ingenic_chan->trigger_en = simple_strtoul(str, (char **)&str, 10);

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			dev_err(dev, "No trigger_negedge config!\n");
			return count;
		}
	}
	ingenic_chan->trigger_negedge = simple_strtoul(str, (char **)&str, 10);

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			dev_err(dev, "No trigger_posedge config!\n");
			return count;
		}
	}
	ingenic_chan->trigger_posedge = simple_strtoul(str, (char **)&str, 10);

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			dev_err(dev, "No trigger_filter config!\n");
			return count;
		}
	}
	ingenic_chan->trigger_filter = simple_strtoul(str, (char **)&str, 10);

	return count;
}

static struct device_attribute pwm_device_attributes[] = {
	__ATTR(request, S_IRUGO | S_IWUSR, pwm_channel_show, pwm_channel_store),
	__ATTR(free, S_IWUSR, NULL, pwm_free_store),

	__ATTR(duty_ns, S_IRUGO | S_IWUSR, pwm_duty_ns_show, pwm_duty_ns_store),
	__ATTR(period_ns, S_IRUGO | S_IWUSR, pwm_period_ns_show, pwm_period_ns_store),
	//__ATTR(sg_pwm_num, S_IRUGO|S_IWUSR, pwm_sg_pwm_num_show, pwm_sg_pwm_num_store),
	//__ATTR(mode, S_IRUGO|S_IWUSR, pwm_mode_show, pwm_mode_store),
	__ATTR(init_level, S_IRUGO | S_IWUSR, pwm_init_level_show, pwm_init_level_store),
	__ATTR(finish_level, S_IRUGO | S_IWUSR, pwm_finish_level_show, pwm_finish_level_store),
	__ATTR(trigger, S_IRUGO | S_IWUSR, pwm_trigger_show, pwm_trigger_store),
	__ATTR(enable, S_IRUGO | S_IWUSR, pwm_enable_show, pwm_enable_store),
	__ATTR(prescale, S_IWUSR, NULL, pwm_prescale_store),

	__ATTR(channels, S_IRUGO, pwm_show_requested_channel, NULL),
};
/*
static bool pwm_dma_chan_filter(struct dma_chan *chan, void *param)
{
    struct ingenic_pwm_chan *pwm_chan = param;

    return (INGENIC_DMA_REQ_PWM0_TX + pwm_chan->id) == (int)chan->private;
}
*/
static irqreturn_t ingenic_pwm_interrupt(int irq, void *dev_id)
{
	unsigned int under_st, store_st;
	struct ingenic_pwm_chip *ingenic_pwm = (struct ingenic_pwm_chip *)(dev_id);
	int n;

	under_st = pwm_dma_under_status(ingenic_pwm) & pwm_readl(ingenic_pwm, PWM_DFIE);

	store_st = pwm_store_status(ingenic_pwm) & pwm_readl(ingenic_pwm, PWM_SIE);

	if (store_st) {
		n = ffs(store_st) - 1;
		pwm_store_clear_status(ingenic_pwm, n);
		ingenic_pwm->chan[n].store_irq_num++;
		printk("%d %d %d %d\n", n, pwm_get_store_num(ingenic_pwm, n),
		       pwm_get_output_num(ingenic_pwm, n), ingenic_pwm->chan[n].store_irq_num);
		goto end;
	}

	if (under_st) {
		n = ffs(under_st) - 1;
		pwm_dma_clear_under(ingenic_pwm, n);
		//printk("pwm%d under come!\n", n);
		goto end;
	}

	printk("pwm irq entry, but not pwm valid irq!\n");
end:
	return IRQ_HANDLED;
}

static struct pwm_chip_priv version1_pri = {
	.prescale = 8,
	.npwm = 8,
	.version_num = 1,

};

static struct pwm_chip_priv version2_pri = {
	.prescale = 0xffff,
	.npwm = 16,
	.version_num = 2,

};

static const struct of_device_id ingenic_pwm_matches[] = {
	{ .compatible = "ingenic,x1600-pwm", .data = (struct pwm_chip_priv *) &version1_pri },
	{ .compatible = "ingenic,x2600-pwm", .data = (struct pwm_chip_priv *) &version2_pri },
	{ .compatible = "ingenic,ad100-pwm", .data = (struct pwm_chip_priv *) &version2_pri },
	{},
};
MODULE_DEVICE_TABLE(of, ingenic_pwm_matches);

static int ingenic_pwm_probe(struct platform_device *pdev)
{
	struct ingenic_pwm_chip *chip;
	struct resource *res;
	int err = 0;
	int ret = 0;
	int i = 0;
	char str[2] = "0";
	chip = devm_kzalloc(&pdev->dev,
	                    sizeof(struct ingenic_pwm_chip), GFP_KERNEL);
	if (!chip) {
		pr_err("%s %d,malloc ingenic_pwm_chip error\n",
		       __func__, __LINE__);
		return -ENOMEM;
	}

	chip->priv = of_match_node(ingenic_pwm_matches, pdev->dev.of_node);
	chip->chip_priv = (struct pwm_chip_priv *)chip->priv->data;

	chip->debug_pwm = devm_kzalloc(&pdev->dev, sizeof(struct pwm_device *) * chip->chip_priv->npwm, GFP_KERNEL);
	if (IS_ERR_OR_NULL(chip->debug_pwm)) {
		dev_err(&pdev->dev, "faile to allocate debug_pwm!\n");
		ret = -ENOMEM;
		goto err_1;
	}
	chip->chan = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_pwm_chan) * chip->chip_priv->npwm, GFP_KERNEL);
	if (IS_ERR_OR_NULL(chip->chan)) {
		dev_err(&pdev->dev, "Failed to allcoate ingenic_pwm_chan!\n");
		ret = -ENOMEM;

		goto err_2;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		return  -ENOENT;
	}

	chip->irq = platform_get_irq(pdev, 0);
	if (chip->irq < 0) {
		dev_err(&pdev->dev, "Cannot get %d  IORESOURCE_IRQ\n", chip->irq);
		return  -ENOENT;
	}

	chip->iomem = ioremap(res->start, (res->end - res->start) + 1);
	if (chip->iomem == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}

	chip->clk_gate = devm_clk_get(&pdev->dev, "gate_pwm");
	if (IS_ERR(chip->clk_gate)) {
		dev_err(&pdev->dev, "get pwm clk gate failed %ld\n", PTR_ERR(chip->clk_gate));
		return PTR_ERR(chip->clk_gate);
	}

	chip->clk_pwm = devm_clk_get(&pdev->dev, "div_pwm");
	if (IS_ERR(chip->clk_pwm)) {
		dev_err(&pdev->dev, "get pwm clk failed %ld\n", PTR_ERR(chip->clk_pwm));
		return PTR_ERR(chip->clk_pwm);
	}

	if (chip->clk_gate) {
		ret = clk_prepare_enable(chip->clk_gate);
		if (ret) {
			dev_err(&pdev->dev, "enable pwm clock gate failed!\n");
		}
	}

	if (chip->clk_pwm) {
		ret = clk_set_rate(chip->clk_pwm, DEFAULT_PWM_CLK_RATE);
		if (ret) {
			dev_err(&pdev->dev, "set pwm clock rate failed!\n");
		}
		ret = clk_prepare_enable(chip->clk_pwm);
		if (ret) {
			dev_err(&pdev->dev, "enable pwm clock failed!\n");
		}
	}

	chip->chip.dev = &pdev->dev;
	chip->chip.ops = &ingenic_pwm_ops;
	chip->chip.base = 0;
	chip->chip.npwm = chip->chip_priv->npwm;
	chip->phys = res->start;

	ret = pwmchip_add(&chip->chip);
	if (ret < 0) {
		devm_kfree(&pdev->dev, chip);
		return ret;
	}

	{
		dma_cap_mask_t mask;
		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);

		for (i = 0; i < chip->chip.npwm; i++) {
			chip->chan[i].id = i;
			chip->chan[i].chip = chip;
			sprintf(str, "%d", i);
			chip->chan[i].dma_chan = dma_request_chan(&pdev->dev, str);
			if (!chip->chan[i].dma_chan) {
				dev_err(&pdev->dev, "PWM request dma tx channel failed");
			}
			chip->chan[i].sg = kmalloc(sizeof(struct scatterlist), GFP_KERNEL);
			if (!chip->chan[i].sg) {
				dev_err(&pdev->dev, "Failed to alloc tx scatterlist\n");
			}
			chip->chan[i].init_level = 0;
			chip->chan[i].finish_level = 0;
			chip->chan[i].trigger_en = 0;
			chip->chan[i].sg_pwm_num = SG_PWM_NUM;
		}
	}

	ret = request_irq(chip->irq, ingenic_pwm_interrupt,
	                  IRQF_SHARED | IRQF_TRIGGER_LOW, "pwm-interrupt", chip);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed !! %d-\n", chip->irq);
		goto err_no_iomap;
	}

	platform_set_drvdata(pdev, chip);
	mutex_init(&chip->mutex);

	for (i = 0; i < ARRAY_SIZE(pwm_device_attributes); i++) {
		ret = device_create_file(&pdev->dev, &pwm_device_attributes[i]);
		if (ret) {
			dev_warn(&pdev->dev, "attribute %d create failed", i);
		}
	}

	dev_info(&pdev->dev, "pwm driver success!\n");
	return 0;

err_no_iomap:
	iounmap(chip->iomem);

	devm_kfree(&pdev->dev, chip->chan);
err_2:
	devm_kfree(&pdev->dev, chip->debug_pwm);
err_1:
	devm_kfree(&pdev->dev, chip);
	return err;
}

static int ingenic_pwm_remove(struct platform_device *pdev)
{
	struct ingenic_pwm_chip *chip;
	chip = platform_get_drvdata(pdev);
	if (!chip) {
		return -ENODEV;
	}

	pwmchip_remove(&chip->chip);
	devm_clk_put(&pdev->dev, chip->clk_pwm);
	devm_clk_put(&pdev->dev, chip->clk_gate);
	devm_kfree(&pdev->dev, chip->debug_pwm);
	devm_kfree(&pdev->dev, chip->chan);
	devm_kfree(&pdev->dev, chip);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ingenic_pwm_suspend(struct device *dev)
{
	struct ingenic_pwm_chip *chip = dev_get_drvdata(dev);
	unsigned int i;

	/*
	 * No one preserves these values during suspend so reset them.
	 * Otherwise driver leaves PWM unconfigured if same values are
	 * passed to pwm_config() next time.
	 */
	for (i = 0; i < chip->chip_priv->npwm; ++i) {
		struct pwm_device *pwm = &chip->chip.pwms[i];
		struct ingenic_pwm_channel *chan = pwm_get_chip_data(pwm);

		if (!chan) {
			continue;
		}
		chan->period_ns = 0;
		chan->duty_ns = 0;
	}

	return 0;
}

static int ingenic_pwm_resume(struct device *dev)
{
	struct ingenic_pwm_chip *chip = dev_get_drvdata(dev);
	unsigned int chan;

	/*
	 * Inverter setting must be preserved across suspend/resume
	 * as nobody really seems to configure it more than once.
	 */
	for (chan = 0; chan < chip->chip_priv->npwm; ++chan) {
		if (chip->output_mask & BIT(chan)) {
			/*TODO: ??*/
		}
	}
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ingenic_pwm_pm_ops, ingenic_pwm_suspend, ingenic_pwm_resume);

static struct platform_driver ingenic_pwm_driver = {
	.driver = {
		.name = "ingenic-pwm",
		.pm = &ingenic_pwm_pm_ops,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ingenic_pwm_matches),
	},
	.probe = ingenic_pwm_probe,
	.remove = ingenic_pwm_remove,
};
module_platform_driver(ingenic_pwm_driver);

MODULE_DESCRIPTION("Ingenic SoC PWM driver");
MODULE_ALIAS("platform:ingenic-pwm");
MODULE_LICENSE("GPL");
