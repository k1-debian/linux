/* drivers/pwm/pwm-ingenic-x2000.c
 * PWM driver of Ingenic's SoC X2000
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
#include <linux/dma-mapping.h>

#define INGENIC_PWM_NUM     (16)
#define NS_IN_HZ        (1000000000UL)
#define DEFAULT_PWM_CLK_RATE    (50000000)
#define PWM_INIT_HIGH       (1)
#define PWM_INIT_LOW        (0)
#define PWM_WCFG_HIGH       (16)
#define PWM_WCFG_LOW        (0)

#define DE_WARNING  0

struct ingenic_pwm_channel {
	u32 period_ns;
	u32 duty_ns;
};

enum pwm_mode_sel {
	COMMON_MODE,
	DMA_MODE
};

struct ingenic_pwm_chip {
	struct pwm_chip chip;
	struct clk *clk_pwm;
	struct clk *clk_gate;
	void __iomem    *iomem;
	int init_level;
	struct mutex mutex;
	unsigned int output_mask;

	struct pwm_device *debug_pwm[INGENIC_PWM_NUM];
	void *vir_addr[INGENIC_PWM_NUM];
	unsigned int phy_addr[INGENIC_PWM_NUM];
	int debug_current_id;
	int mode_sel[INGENIC_PWM_NUM];
	int irq;
};

/**-----------------------------------------------------------------------------
 **                       reg offset
 **-----------------------------------------------------------------------------*/

#define PWM_CCFG0   (0x0)
#define PWM_CCFG1   (0x4)
#define PWM_ENS     (0x10)
#define PWM_ENC     (0x14)
#define PWM_EN      (0x18)
#define PWM_UP      (0x20)
#define PWM_BUSY    (0x24)
#define PWM_INITR   (0x30)
#define PWM_WCFG    (0xb0)
#define PWM_DES     (0x100)
#define PWM_DEC     (0x104)
#define PWM_DE      (0x108)
#define PWM_DCR0    (0x110)
#define PWM_DCR1    (0x114)
#define PWM_DTRIG   (0x120)
#define PWM_DFER    (0x124)
#define PWM_DFSM    (0x128)
#define PWM_DSR     (0x130)
#define PWM_DSCR    (0x134)
#define PWM_DINTC   (0x138)
#define PWM_DMADDR  (0x140)
#define PWM_DTLR    (0x190)
#define PWM_OEN     (0x300)

#define PRESCALE_MSK    (0xf)
#define PRESCALE    (0x2)
#define TRANS_LEN   (32)
#define PWM_PER     (0x30)

/**-----------------------------------------------------------------------------
 **                       reg bit field
 **-----------------------------------------------------------------------------*/

static void pwm_writel(struct ingenic_pwm_chip *chip, unsigned int value, unsigned int offset)
{
	writel(value, chip->iomem + offset);
}

static unsigned int pwm_readl(struct ingenic_pwm_chip *chip, unsigned int offset)
{
	return readl(chip->iomem + offset);
}

static void pwm_clk_config(struct ingenic_pwm_chip *chip, unsigned int channel, unsigned int prescale)
{
	int tmp = 0;
	if (channel < 8) {
		tmp = pwm_readl(chip, PWM_CCFG0) & ~(PRESCALE_MSK << (channel * 4));
		tmp |= prescale << (channel * 4);
		pwm_writel(chip, tmp, PWM_CCFG0);
	} else {
		tmp = pwm_readl(chip, PWM_CCFG1) & ~(PRESCALE_MSK << ((channel - 8) * 4));
		tmp |= prescale << ((channel - 8) * 4);
		pwm_writel(chip, tmp, PWM_CCFG1);
	}
}

static unsigned int pwm_get_prescale(struct ingenic_pwm_chip *chip, unsigned int channel)
{
	int tmp = 0;
	int prescale = 0;
	if (channel < 8) {
		tmp = pwm_readl(chip, PWM_CCFG0);
		prescale = tmp & (0xf << channel * 4);
	} else {
		tmp = pwm_readl(chip, PWM_CCFG1);
		prescale = tmp & (0xf << (channel - 8) * 4);
	}
	return prescale;
}

static void pwm_enable_hw(struct ingenic_pwm_chip *chip, unsigned int channel)
{
	int tmp = 0;
	tmp = pwm_readl(chip, PWM_EN);
	tmp |= (1 << channel);
	pwm_writel(chip, tmp, PWM_ENS);
}

static void pwm_disable_hw(struct ingenic_pwm_chip *chip, unsigned int channel)
{
	pwm_writel(chip, 1 << channel, PWM_ENC);
}

static int pwm_enable_status(struct ingenic_pwm_chip *chip)
{
	return pwm_readl(chip, PWM_EN);
}

static void pwm_update(struct ingenic_pwm_chip *chip, unsigned int channel)
{
	pwm_writel(chip, 1 << channel, PWM_UP);
}

static int pwm_busy(struct ingenic_pwm_chip *chip, unsigned int channel)
{
	int tmp = 0;
	tmp = pwm_readl(chip, PWM_BUSY);
	return tmp & (1 << channel);
}

#if DE_WARNING
static void pwm_set_period_hw(struct ingenic_pwm_chip *chip, unsigned int channel, unsigned int period)
{
	pwm_writel(chip, period, PWM_PER + channel * 4);
}
#endif

static void pwm_set_init_level(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel, unsigned int level)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm, PWM_INITR);
	if (level) {
		tmp |= 1 << channel;
	} else {
		tmp &= ~(1 << channel);
	}
	pwm_writel(ingenic_pwm, tmp, PWM_INITR);
}

static void pwm_set_finish_level(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel, unsigned int level)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm, PWM_INITR);
	if (level) {
		tmp |= 1 << (channel + 16);
	} else {
		tmp &= ~(1 << (channel + 16));
	}
	pwm_writel(ingenic_pwm, tmp, PWM_INITR);
}

static void pwm_waveform_high(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel, unsigned int high_num)
{
	int tmp = 0;
	tmp = pwm_readl(ingenic_pwm, PWM_WCFG + channel * 4);
	tmp &= ~(0xffff << PWM_WCFG_HIGH);
	tmp |= high_num << PWM_WCFG_HIGH;
	pwm_writel(ingenic_pwm, tmp, PWM_WCFG + channel * 4);
}

static void pwm_waveform_low(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel, unsigned int low_num)
{
	int tmp = 0;
	tmp = pwm_readl(ingenic_pwm, PWM_WCFG + channel * 4);
	tmp &= ~(0xffff);
	tmp |= low_num << PWM_WCFG_LOW;
	pwm_writel(ingenic_pwm, tmp, PWM_WCFG + channel * 4);
}

/*pwm DMA mode control register operation*/
static void pwm_dma_enable_hw(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp = 0;
	tmp = pwm_readl(ingenic_pwm, PWM_DES);
	tmp |= (1 << channel);
	pwm_writel(ingenic_pwm, tmp, PWM_DES);
}

static void pwm_dma_disable_hw(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	pwm_writel(ingenic_pwm, 1 << channel, PWM_DEC);
}

#if DE_WARNING
static int pwm_dma_enable_status(struct ingenic_pwm_chip *ingenic_pwm)
{
	return pwm_readl(ingenic_pwm, PWM_DE);
}
#endif

static void pwm_set_update_mode(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel, int mode_sel)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm, PWM_DCR0);
	if (mode_sel == DMA_MODE) {
		tmp |= 1 << channel;
	}
	if (mode_sel == COMMON_MODE) {
		tmp &= ~(1 << channel);
	}
	pwm_writel(ingenic_pwm, tmp, PWM_DCR0);
}

#if DE_WARNING
static void pwm_dma_set_loop(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm, PWM_DCR1);
	tmp |= 1 << channel;
	pwm_writel(ingenic_pwm, tmp, PWM_DCR1);
}
#endif

static void pwm_dma_clear_loop(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm, PWM_DCR1);
	tmp &= ~(1 << channel);
	pwm_writel(ingenic_pwm, tmp, PWM_DCR1);
}

static unsigned int pwm_dma_get_loop_status(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	return pwm_readl(ingenic_pwm, PWM_DCR1) & (1 << channel);
}

static void pwm_dma_trans_trigger(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm, PWM_DTRIG);
	tmp |= 1 << channel;
	pwm_writel(ingenic_pwm, tmp, PWM_DTRIG);
}

static unsigned int pwm_dma_fifo_empty(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	return pwm_readl(ingenic_pwm, PWM_DFER) & (1 << channel);
}

static unsigned int pwm_dma_get_dfsm(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm, PWM_DFSM);
	tmp &= 1 << channel;
	return tmp;
}

static int pwm_dma_get_status(struct ingenic_pwm_chip *ingenic_pwm)
{
	return pwm_readl(ingenic_pwm, PWM_DSR);
}

#if DE_WARNING
static void pwm_dma_clear_status(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm, PWM_DSCR);
	tmp |= 1 << channel;
	pwm_writel(ingenic_pwm, tmp, PWM_DSCR);
}
#endif

#if DE_WARNING
static void pwm_dma_interrupt_mask(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm, PWM_DINTC);
	tmp |= 1 << channel;
	pwm_writel(ingenic_pwm, tmp, PWM_DINTC);
}
#endif

static void pwm_dma_interrupt_unmask(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm, PWM_DINTC);
	tmp &=  ~(1 << channel);
	pwm_writel(ingenic_pwm, tmp, PWM_DINTC);
}

static void pwm_dma_set_mem_addr(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel, unsigned int mem_addr)
{
	pwm_writel(ingenic_pwm, mem_addr, PWM_DMADDR + channel * 4);
}

/*DMA transfer length must 4 word align*/
static void pwm_dma_set_trans_length(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel, int tran_len)
{
	pwm_writel(ingenic_pwm, tran_len, PWM_DTLR + channel * 4);
}

static void pwm_set_io_output_enable(struct ingenic_pwm_chip *ingenic_pwm, unsigned int channel, int enable)
{
	int tmp;
	tmp = pwm_readl(ingenic_pwm, PWM_OEN);
	if (enable) {
		tmp |= 1 << channel;
	} else {
		tmp &= ~(1 << channel);
	}
	pwm_writel(ingenic_pwm, tmp, PWM_OEN);
}

#ifdef CONFIG_PWM_DUMP
static void dump_pwm_reg(struct ingenic_pwm_chip *ingenic_pwm)
{
	int i;
	printk("PWM_CCFG0 addr %08x  = %08x\n", (unsigned int)(ingenic_pwm->iomem + PWM_CCFG0), pwm_readl(ingenic_pwm, PWM_CCFG0));
	printk("PWM_CCFG1 addr %08x  = %08x\n", (unsigned int)(ingenic_pwm->iomem + PWM_CCFG1), pwm_readl(ingenic_pwm, PWM_CCFG1));
	printk("PWM_ENS   addr %08x  = %08x\n", (unsigned int)(ingenic_pwm->iomem + PWM_ENS), pwm_readl(ingenic_pwm, PWM_ENS));
	printk("PWM_ENC   addr %08x  = %08x\n", (unsigned int)(ingenic_pwm->iomem + PWM_ENC), pwm_readl(ingenic_pwm, PWM_ENC));
	printk("PWM_EN    addr %08x  = %08x\n", (unsigned int)(ingenic_pwm->iomem + PWM_EN), pwm_readl(ingenic_pwm, PWM_EN));
	printk("PWM_UP    addr %08x  = %08x\n", (unsigned int)(ingenic_pwm->iomem + PWM_UP), pwm_readl(ingenic_pwm, PWM_UP));
	printk("PWM_LOOP  addr %08x  = %08x\n", (unsigned int)(ingenic_pwm->iomem + PWM_DCR1), pwm_readl(ingenic_pwm, PWM_DCR1));
	printk("PWM_MASK  addr %08x  = %08x\n", (unsigned int)(ingenic_pwm->iomem + PWM_DINTC), pwm_readl(ingenic_pwm, PWM_DINTC));
	printk("PWM_BUSY  addr %08x  = %08x\n", (unsigned int)(ingenic_pwm->iomem + PWM_BUSY), pwm_readl(ingenic_pwm, PWM_BUSY));
	for (i = 0; i < 16; i++) {
		printk("PWM_DAR %d addr %08x = %08x\n", i, (unsigned int)(ingenic_pwm->iomem + i * 4), pwm_readl(ingenic_pwm, PWM_DMADDR + i * 4));
	}
	for (i = 0; i < 16; i++) {
		printk("PWM_WCFG%d addr %08x = %08x\n", i, (unsigned int)(ingenic_pwm->iomem + i * 4), pwm_readl(ingenic_pwm, PWM_WCFG + i * 4));
	}
	for (i = 0; i < 16; i++) {
		printk("PWM_DTLR%d addr %08x = %08x\n", i, (unsigned int)(ingenic_pwm->iomem + i * 4), pwm_readl(ingenic_pwm, PWM_DTLR + i * 4));
	}
}
#endif

static irqreturn_t ingenic_pwm_interrupt(int irq, void *dev_id)
{
	int tmp;
	struct ingenic_pwm_chip *ingenic_pwm = (struct ingenic_pwm_chip *)(dev_id);
	tmp = pwm_dma_get_status(ingenic_pwm);
	/*clear end flag*/
	pwm_writel(ingenic_pwm, tmp, PWM_DSCR);

	/*restart DMA transfer*/
	pwm_writel(ingenic_pwm, tmp, PWM_DTRIG);

	return IRQ_HANDLED;
}
//--------------------------add end----------------------

static inline struct ingenic_pwm_chip *to_ingenic_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct ingenic_pwm_chip, chip);
}

static int ingenic_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
                              int duty_ns, int period_ns)
{
	struct ingenic_pwm_chip *ingenic_pwm = to_ingenic_chip(chip);
	int channel = pwm->hwpwm;
	unsigned int period = 0;
	unsigned int duty = 0;
	unsigned int pwm_freq = 0;
	unsigned int clk_in = 0;
	unsigned int prescale = 0;
	unsigned int tmp = 1;
	int i = 0;
	int update_flag = 1;
	int mode = 0;
	void *dma_coherent;
	dma_addr_t dma_coherent_handle;
	unsigned long long clk_ns = 1000000000ULL;

	mutex_lock(&ingenic_pwm->mutex);
	if (duty_ns < 0 || duty_ns > period_ns) {
		pr_err("%s, duty_ns(%d)< 0 or duty_ns > period_ns(%d)\n", __func__, duty_ns, period_ns);
		mutex_unlock(&ingenic_pwm->mutex);
		return -EINVAL;
	}

	if (period_ns > NS_IN_HZ || duty_ns > NS_IN_HZ) {
		mutex_unlock(&ingenic_pwm->mutex);
		return -ERANGE;
	}

	/*select mode */
	mode = ingenic_pwm->mode_sel[channel];

	/*set prescale*/
	pwm_clk_config(ingenic_pwm, channel, PRESCALE);
	clk_in = clk_get_rate(ingenic_pwm->clk_pwm);
	prescale = pwm_get_prescale(ingenic_pwm, channel);

	for (i = 0; i < PRESCALE; i++) {
		tmp = 2 * tmp;
	}

	pwm_freq = clk_in / tmp;
	do_div(clk_ns, pwm_freq);

	period = (period_ns / (unsigned int)clk_ns);
	printk("period=%d\n", period);
	//pwm_set_period_hw(ingenic_pwm, channel, period);
	duty = duty_ns / (unsigned int)clk_ns;

	/*set init level*/
	if (ingenic_pwm->init_level == PWM_INIT_HIGH) {
		pwm_set_init_level(ingenic_pwm, channel, 1);
	} else if (ingenic_pwm->init_level == PWM_INIT_LOW) {
		pwm_set_init_level(ingenic_pwm, channel, 0);
	} else {
		pr_err("%s, pwm init level error\n", __func__);
	}

	printk("duty=%d level_init=%d low=%d\n", duty, ingenic_pwm->init_level, period - duty);

	/*set waveform high_num and low_num*/
	if (COMMON_MODE == mode) {
		//*****************************test**(duty**period)**********
		pwm_waveform_high(ingenic_pwm, channel, duty);
		pwm_waveform_low(ingenic_pwm, channel, period - duty);
	}

	/*Set the level state at the end of the waveform*/
	pwm_set_finish_level(ingenic_pwm, channel, 0);

	/*pwm output enable*/
	pwm_set_io_output_enable(ingenic_pwm, channel, 1);

	/*Update bit according to mode setting*/
	pwm_set_update_mode(ingenic_pwm, channel, mode);

	/*Whether to update*/
	if (pwm->state.enabled) {
		if (COMMON_MODE == mode) {
			pwm_update(ingenic_pwm, channel);
			while (pwm_busy(ingenic_pwm, channel));
		}
		if (DMA_MODE == mode) {
			update_flag = 0;
			if (!pwm_dma_get_loop_status(ingenic_pwm, channel)) {
				while (!pwm_dma_fifo_empty(ingenic_pwm, channel));
				if (pwm_dma_fifo_empty(ingenic_pwm, channel)) {
					update_flag = 1;
				}
			}
		}
	}
	/*DMA mode related configuration*/
	if (DMA_MODE == mode && update_flag) {

		dma_coherent = dma_alloc_coherent(ingenic_pwm->chip.dev, sizeof(int) * TRANS_LEN, &dma_coherent_handle, GFP_KERNEL);
		//printk("--%s-------%d-dma_coherent = %x--dma_coherent_handle = %x---write value = %x----\n",__func__,__LINE__,(unsigned int)dma_coherent,dma_coherent_handle,(0x1111ffff));
		dma_cache_wback_inv((unsigned int)dma_coherent, sizeof(int) * TRANS_LEN);
		for (i = 0; i < TRANS_LEN; i++) {
			//printk("-----------befor-addr = %x--value ----%x --- \n",(unsigned int)dma_coherent + i * 4,*(volatile unsigned int *)(dma_coherent + i * 4));
			/* configuration duty ratio 1/2 */
			*(volatile unsigned int *)(dma_coherent + i * 4) = (0xffffffff);
			//printk("-----------after-addr = %x--value ----%x --- \n",(unsigned int)dma_coherent + i * 4,*(volatile unsigned int *)(dma_coherent + i * 4));
		}
		ingenic_pwm->vir_addr[channel] = dma_coherent;
		ingenic_pwm->phy_addr[channel] = dma_coherent_handle;

		/*set addr and trans_len*/
		pwm_dma_set_mem_addr(ingenic_pwm, channel, dma_coherent_handle);
		pwm_dma_set_trans_length(ingenic_pwm, channel, TRANS_LEN);

		/*set interrupt */
		pwm_dma_clear_loop(ingenic_pwm, channel);
		pwm_dma_interrupt_unmask(ingenic_pwm, channel);
	}
	mutex_unlock(&ingenic_pwm->mutex);
	return 0;
}

static int ingenic_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
                                    enum pwm_polarity polarity)
{
	struct ingenic_pwm_chip *ingenic_pwm = to_ingenic_chip(chip);
	int channel = pwm->hwpwm;

	printk("%s: %d, polarity = %d\n", __func__, __LINE__, polarity);
	if (polarity == PWM_POLARITY_NORMAL) {
		pwm_set_init_level(ingenic_pwm, channel, PWM_INIT_HIGH);
	} else if (polarity == PWM_POLARITY_INVERSED) {
		pwm_set_init_level(ingenic_pwm, channel, PWM_INIT_LOW);
	} else {
		pr_err("%s, pwm init level error\n", __func__);
	}
	return 0;
}

static int ingenic_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ingenic_pwm_chip *ingenic_pwm = to_ingenic_chip(chip);
	int channel = pwm->hwpwm;
	int count = 0;
	mutex_lock(&ingenic_pwm->mutex);

	if (ingenic_pwm->mode_sel[channel] == DMA_MODE) {
		/*enable DMA mode*/
		while (pwm_dma_get_dfsm(ingenic_pwm, channel)) {
			count ++;
			if (count == 0x10000000) {
				printk("pwm %d channel is working\n", channel);
				return 0;
			}
		}
		pwm_dma_enable_hw(ingenic_pwm, channel);
		pwm_dma_trans_trigger(ingenic_pwm, channel);
	}
	pwm_enable_hw(ingenic_pwm, channel);
	mutex_unlock(&ingenic_pwm->mutex);

	return 0;
}

static void ingenic_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ingenic_pwm_chip *ingenic_pwm = to_ingenic_chip(chip);
	int channel = pwm->hwpwm;

	mutex_lock(&ingenic_pwm->mutex);
	if (ingenic_pwm->mode_sel[channel] == DMA_MODE) {
		pwm_dma_disable_hw(ingenic_pwm, channel);
		dma_free_coherent(ingenic_pwm->chip.dev, sizeof(int) * TRANS_LEN, ingenic_pwm->vir_addr[channel], ingenic_pwm->phy_addr[channel]);
	}

	pwm_disable_hw(ingenic_pwm, channel);
	mutex_unlock(&ingenic_pwm->mutex);
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
	.apply = ingenic_pwm_apply,
	.owner = THIS_MODULE,
};

static ssize_t pwm_show_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	return sprintf(buf, "%x", pwm_enable_status(ingenic_pwm));
}

static ssize_t pwm_store_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
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

	printk("input enable value %d,current_id %d\n", enable, ingenic_pwm->debug_current_id);
	if (enable) {
		pwm_enable(ingenic_pwm->debug_pwm[ingenic_pwm->debug_current_id]);
	} else {
		pwm_disable(ingenic_pwm->debug_pwm[ingenic_pwm->debug_current_id]);
	}
	return count;
}

static ssize_t pwm_show_config(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_PWM_DUMP
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	dump_pwm_reg(ingenic_pwm);
#endif
	printk("\n");
	return 0;
}

static ssize_t pwm_store_config(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);
	const char *str = buf;
	unsigned int period_ns, duty_ns;
	int mode = 0;
	int ret_count = 0;

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			return count;
		}
	}
	mode = simple_strtoul(str, (char **)&str, 10);
	if (DMA_MODE == mode) {
		ingenic_pwm->mode_sel[ingenic_pwm->debug_current_id] = DMA_MODE;
	}

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			return count;
		}
	}
	duty_ns = simple_strtoul(str, (char **)&str, 10);

	while (!isxdigit(*str)) {
		str++;
		if (++ret_count >= count) {
			return count;
		}
	}
	period_ns = simple_strtoul(str, (char **)&str, 10);

	printk("current_id = %d, period_ns = %d , duty_ns = %d mode = %s\n", ingenic_pwm->debug_current_id, period_ns, duty_ns, mode ? "DMA_MODE" : "COMMON_MODE");
	pwm_config(ingenic_pwm->debug_pwm[ingenic_pwm->debug_current_id], duty_ns, period_ns);
	printk("pwm_config finish\n");

	return count;
}

static ssize_t pwm_show_channel(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ingenic_pwm_chip *ingenic_pwm = dev_get_drvdata(dev);

	return sprintf(buf, "channel = %x", ingenic_pwm->debug_current_id);

}

static ssize_t pwm_store_channel(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
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
	printk("channel = %d\n", ingenic_pwm->debug_current_id);
	return count;
}

static ssize_t pwm_store_free(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
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

	for (i = 0; i < INGENIC_PWM_NUM ; i++) {
		if (ingenic_pwm->debug_pwm[i] == &(ingenic_pwm->chip.pwms[i])) {
			ret += sprintf(buf + ret, "ch: %02d requested\n", i);
		} else {
			ret += sprintf(buf + ret, "ch: %02d unrequested\n", i);
		}
	}
	return ret;
}

static struct device_attribute pwm_device_attributes[] = {
	__ATTR(enable, S_IRUGO | S_IWUSR, pwm_show_enable, pwm_store_enable),
	__ATTR(config, S_IRUGO | S_IWUSR, pwm_show_config, pwm_store_config),
	__ATTR(request, S_IRUGO | S_IWUSR, pwm_show_channel, pwm_store_channel),
	__ATTR(free, S_IWUSR, NULL, pwm_store_free),
	__ATTR(channels, S_IRUGO, pwm_show_requested_channel, NULL),
};

static int ingenic_pwm_probe(struct platform_device *pdev)
{
	struct ingenic_pwm_chip *chip;
	struct resource *res;
	int err = 0;
	int ret = 0;
	int i = 0;

	chip = devm_kzalloc(&pdev->dev,
	                    sizeof(struct ingenic_pwm_chip), GFP_KERNEL);
	if (!chip) {
		pr_err("%s %d,malloc ingenic_pwm_chip error\n",
		       __func__, __LINE__);
		return -ENOMEM;
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
	chip->chip.npwm = INGENIC_PWM_NUM;
	chip->init_level = PWM_INIT_HIGH;

	ret = pwmchip_add(&chip->chip);
	if (ret < 0) {
		devm_kfree(&pdev->dev, chip);
		return ret;
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

	dev_info(&pdev->dev, "ingenic-x2000 Probe of pwm success!\n");
	return 0;

err_no_iomap:
	iounmap(chip->iomem);

	return err;
}

static int ingenic_pwm_remove(struct platform_device *pdev)
{
	struct ingenic_pwm_chip *chip;
	chip = platform_get_drvdata(pdev);
	if (!chip) {
		return -ENODEV;
	}

	free_irq(chip->irq, chip);
	pwmchip_remove(&chip->chip);
	devm_clk_put(&pdev->dev, chip->clk_pwm);
	devm_clk_put(&pdev->dev, chip->clk_gate);
	devm_kfree(&pdev->dev, chip);
	return 0;
}

static const struct of_device_id ingenic_pwm_matches[] = {
	{ .compatible = "ingenic,x2000-pwm", .data = NULL },
	{ .compatible = "ingenic,m300-pwm", .data = NULL },
	{},
};

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
	for (i = 0; i < INGENIC_PWM_NUM; ++i) {
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
	for (chan = 0; chan < INGENIC_PWM_NUM; ++chan) {
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
