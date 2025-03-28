/*
 * sound/soc/ingenic/icodec/icdc_inno_v3.c
 * ALSA SoC Audio driver -- ingenic internal codec (icodec) driver

 * Copyright 2019 Ingenic Semiconductor Co.,Ltd
 *
 *
 * Note: icodec is an internal codec for ingenic SOC
 *   used for x2600
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/soc-dai.h>
#include <linux/of_address.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <sound/tlv.h>

#define ICODEC_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_LE | \
                        SNDRV_PCM_FMTBIT_S24_LE)

#define RSTR        0x0
#define DACCR1      0x04
#define DACCR2      0x08
#define DACCR3      0x0c
#define DACCR4      0x10
#define DACCR5      0x14
#define DACDGR      0x18
#define ADCCR1      0x24
#define ADCCR2      0x28
#define ADCCR3      0x2c
#define ADCCR4      0x30
#define ADCDGR      0x34
#define BIASCR1     0x80
#define BIASCR2     0x84
#define BIASCR3     0x88
#define DACLCR      0xa0
#define HPCR        0xa4
#define HPDSR       0xa8
#define HPGR        0xac
#define ADCLCR      0xc0
#define MICCR       0xc4
#define ALCGR       0xc8
#define MICGR       0xcc
#define AGCCR1      0x100
#define AGCCR2      0x104
#define AGCCR3      0x108
#define PGAGR       0x10c
#define AGCSRR      0x110
#define AGCMAXLR    0x114
#define AGCMAXHR    0x118
#define AGCMINLR    0x11c
#define AGCMINHR    0x120
#define AGCGR       0x124
#define ALCOGR      0x138

struct icodec {
	struct device       *dev;
	struct snd_soc_component    *component;
	spinlock_t      io_lock;
	void *__iomem io_base;
	bool powered;
	long dac_dgain;
	long adc_dgain;
	int dac2adc_enable;
	u32 micbias_level;
};

typedef void (*p_icodec_playback_power)(bool);
static p_icodec_playback_power g_icodec_playback_pwr;// playback power control funtion pointer assignment in boards
void set_playback_pwr_callback(p_icodec_playback_power func)
{
	g_icodec_playback_pwr = func;
}
EXPORT_SYMBOL(set_playback_pwr_callback);

#if 0
#ifdef CONFIG_AUDIO_DUMP
static void dump_registers(struct icodec *icodec)
{
	printk("---------- RSTR     : = %#x\r\n", readl(icodec->io_base + RSTR));
	printk("---------- DACCR1   : = %#x\r\n", readl(icodec->io_base + DACCR1));
	printk("---------- DACCR2   : = %#x\r\n", readl(icodec->io_base + DACCR2));
	printk("---------- DACCR3   : = %#x\r\n", readl(icodec->io_base + DACCR3));
	printk("---------- DACCR4   : = %#x\r\n", readl(icodec->io_base + DACCR4));
	printk("---------- DACCR5   : = %#x\r\n", readl(icodec->io_base + DACCR5));
	printk("---------- DACDGR   : = %#x\r\n", readl(icodec->io_base + DACDGR));
	printk("---------- ADCCR1   : = %#x\r\n", readl(icodec->io_base + ADCCR1));
	printk("---------- ADCCR2   : = %#x\r\n", readl(icodec->io_base + ADCCR2));
	printk("---------- ADCCR3   : = %#x\r\n", readl(icodec->io_base + ADCCR3));
	printk("---------- ADCCR4   : = %#x\r\n", readl(icodec->io_base + ADCCR4));
	printk("---------- ADCDGR   : = %#x\r\n", readl(icodec->io_base + ADCDGR));
	printk("---------- BIASCR1  : = %#x\r\n", readl(icodec->io_base + BIASCR1));
	printk("---------- BIASCR2  : = %#x\r\n", readl(icodec->io_base + BIASCR2));
	printk("---------- BIASCR3  : = %#x\r\n", readl(icodec->io_base + BIASCR3));
	printk("---------- DACLCR   : = %#x\r\n", readl(icodec->io_base + DACLCR));
	printk("---------- HPCR     : = %#x\r\n", readl(icodec->io_base + HPCR));
	printk("---------- HPDSR    : = %#x\r\n", readl(icodec->io_base + HPDSR));
	printk("---------- HPGR     : = %#x\r\n", readl(icodec->io_base + HPGR));
	printk("---------- ADCLCR   : = %#x\r\n", readl(icodec->io_base + ADCLCR));
	printk("---------- MICCR    : = %#x\r\n", readl(icodec->io_base + MICCR));
	printk("---------- ALCGR    : = %#x\r\n", readl(icodec->io_base + ALCGR));
	printk("---------- MICGR    : = %#x\r\n", readl(icodec->io_base + MICGR));
	printk("---------- AGCCR1   : = %#x\r\n", readl(icodec->io_base + AGCCR1));
	printk("---------- AGCCR2   : = %#x\r\n", readl(icodec->io_base + AGCCR2));
	printk("---------- AGCCR3   : = %#x\r\n", readl(icodec->io_base + AGCCR3));
	printk("---------- PGAGR    : = %#x\r\n", readl(icodec->io_base + PGAGR));
	printk("---------- AGCSRR   : = %#x\r\n", readl(icodec->io_base + AGCSRR));
	printk("---------- AGCMAXLR : = %#x\r\n", readl(icodec->io_base + AGCMAXLR));
	printk("---------- AGCMAXHR : = %#x\r\n", readl(icodec->io_base + AGCMAXHR));
	printk("---------- AGCMINLR : = %#x\r\n", readl(icodec->io_base + AGCMINLR));
	printk("---------- AGCMINHR : = %#x\r\n", readl(icodec->io_base + AGCMINHR));
	printk("---------- AGCGR    : = %#x\r\n", readl(icodec->io_base + AGCGR));
	printk("---------- ALCOGR   : = %#x\r\n", readl(icodec->io_base + ALCOGR));
}
#endif
#endif

static unsigned int icodec_read(struct snd_soc_component *component, unsigned int reg)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);
	int value = readl(icodec->io_base + reg);

	return value;
}

static int icodec_write(struct snd_soc_component *component, unsigned int reg, unsigned value)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);
	writel(value, icodec->io_base + reg);

	return 0;
}

static inline unsigned int icodec_reg_read(struct icodec *icodec, int offset)
{
	return readl(icodec->io_base + offset);
}

static inline void icodec_reg_write(struct icodec *icodec, int offset, int data)
{
	writel(data, icodec->io_base + offset);
}

int icodec_reg_set(struct icodec *icodec, unsigned int reg, int start, int end, int val)
{
	int ret = 0;
	int i = 0, mask = 0;
	unsigned int oldv = 0;
	unsigned int new = 0;

	for (i = 0;  i < (end - start + 1); i++) {
		mask += (1 << (start + i));
	}

	oldv = icodec_reg_read(icodec, reg);
	new = oldv & (~mask);
	new |= val << start;
	icodec_reg_write(icodec, reg, new);

	if ((new & 0x000000FF) != icodec_reg_read(icodec, reg)) {
		printk("%s(%d):codec write  0x%08x error!!\n", __func__, __LINE__, reg);
		printk("new = 0x%08x, read val = 0x%08x\n", new, icodec_reg_read(icodec, reg));

		return -1;
	}

	return ret;
}

static void icodec_poweron(struct snd_soc_component *component)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	/* step1: Supply the power of the digital part and reset the audio codec. */
	icodec_reg_set(icodec, RSTR, 0, 0, 0x0);        // reset codec system
	icodec_reg_set(icodec, RSTR, 4, 5, 0x0);        // reset Codec DAC digital core and Codec ADC digital core.
	mdelay(10);
	icodec_reg_set(icodec, RSTR, 0, 0, 0x1);
	icodec_reg_set(icodec, RSTR, 4, 5, 0x3);

	/* step2: setup the output DC voltage of DAC left channel*/
	icodec_reg_set(icodec, DACLCR, 5, 6, 0x1);

	/* step3: Configure the register BIASCR2.SELVREF to 0x01. */
	icodec_reg_set(icodec, BIASCR2, 0, 7, 0x1);

	/* step4: Supply the power of the analog part. */
	icodec_reg_set(icodec, RSTR, 0, 0, 0x1);
	icodec_reg_set(icodec, RSTR, 4, 5, 0x3);

	/* step5: setup reference voltage. */
	icodec_reg_set(icodec, BIASCR1, 0, 0, 0x1);

	/* step6: Increase the register BIASCR2.SELVREF from 0x01 to 0xff step by step, or configure this register to 0xff directly.*/
	icodec_reg_set(icodec, BIASCR2, 0, 7, 0xff);
	mdelay(20);

	/* step7: Wait until the voltage of VCM keeps stable at the AVDD/2. */
	mdelay(20);

	/* step8: Configure the register BIASCR2.SELVREF to the appropriate value (except 0x00) for reducing power. */
	icodec_reg_set(icodec, BIASCR2, 0, 7, 0x2);
}

static void icodec_poweroff(struct snd_soc_component *component)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	/* step1: Keep the power on and disable the DAC and ADC path. */

	/* step2: Configure the register BIASCR2.SELVREF to 0x01. */
	icodec_reg_set(icodec, BIASCR2, 0, 7, 0x01);

	/* step3: Configure the register BIASCR1.VREFEN to 0 for disable reference voltage. */
	icodec_reg_set(icodec, BIASCR1, 0, 0, 0x0);

	/* step4: Increase the register BIASCR2.SELVREF from 0x01 to 0xff step by step, or configure this register to 0xff directly. The suggestion slot time of the step is 20ms.*/
	icodec_reg_set(icodec, BIASCR2, 0, 7, 0xff);
	mdelay(20);

	/* step5: Wait until the voltage of VCM keep stable at AGND */
	mdelay(20);

	/* step6: Power off the analog power supply. */

	/* step7: Power off the digital power supply. */

}

static void icodec_dac_enable(struct snd_soc_component *component)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	/* step1: Power up the CODEC and input the mute signal. */

	/* step2: enable the current source of DAC. */
	icodec_reg_set(icodec, BIASCR1, 1, 1, 0x1);

	/* step3: enable the reference voltage buffer of the DAC left channel.*/
	icodec_reg_set(icodec, DACLCR, 2, 2, 0x1);

	/* step4: enable POP sound in the DAC left channel.*/
	icodec_reg_set(icodec, DACLCR, 5, 6, 0x2);

	/* step5: enable the DRVL module in the DAC left channel. */
	icodec_reg_set(icodec, HPCR, 0, 0, 1);

	/* step6: end the initialization of the DRVL module in the DAC left channel. */
	icodec_reg_set(icodec, HPCR, 4, 4, 1);

	/* step7: enable the reference voltage of DACL module. */
	icodec_reg_set(icodec, DACLCR, 3, 3, 1);

	/* step8: enable the clock module of DACL module. */
	icodec_reg_set(icodec, DACLCR, 1, 1, 1);

	/* step9: enable the DACL module. */
	icodec_reg_set(icodec, DACLCR, 0, 0, 1);

	/* step10: end the initialization of the DACL module. */
	icodec_reg_set(icodec, DACLCR, 4, 4, 1);

	/* step11: end the mute station of the DRVL module in the DAC left channel. */
	icodec_reg_set(icodec, HPCR, 5, 5, 1);

	/* step12: select the gain of DRVL module in the DAC left channel. */
	icodec_reg_set(icodec, HPGR, 0, 4, 0x1a);

	/* step13: Configure the DAC digital gain. */
	icodec_reg_set(icodec, DACDGR, 0, 7, icodec->dac_dgain);

	/* step14: Play the music. */
}

static int icodec_dac_configure(struct snd_soc_component *component,
                                struct snd_pcm_hw_params *params)
{
	int fmt_width = snd_pcm_format_width(params_format(params));
	unsigned int samplerate = params_rate(params);
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	/* dac master mode */
	icodec_reg_set(icodec, DACCR2, 4, 5, 0x3);

	icodec_reg_set(icodec, DACCR2, 2, 2, 0x0);

	/* configure audio sample depth */
	switch (fmt_width) {
		case 24:
			icodec_reg_set(icodec, DACCR1, 4, 5, 0x2);
			break;
		case 20:
			icodec_reg_set(icodec, DACCR1, 4, 5, 0x1);
			break;
		case 16:
			icodec_reg_set(icodec, DACCR1, 4, 5, 0x0);
			break;
		default:
			dev_err(component->dev, "icodec not support format width\n");
			return -EINVAL;
	}

	/* configure mute sample rate. */
	switch (samplerate) {
		case 96000:
			icodec_reg_set(icodec, DACCR3, 4, 6, 0x7);
			break;
		case 88200:
			icodec_reg_set(icodec, DACCR3, 4, 6, 0x6);
			break;
		case 48000:
			icodec_reg_set(icodec, DACCR3, 4, 6, 0x5);
			break;
		case 44100:
			icodec_reg_set(icodec, DACCR3, 4, 6, 0x4);
			break;
		case 32000:
			icodec_reg_set(icodec, DACCR3, 4, 6, 0x3);
			break;
		case 24000:
			icodec_reg_set(icodec, DACCR3, 4, 6, 0x2);
			break;
		case 16000:
			icodec_reg_set(icodec, DACCR3, 4, 6, 0x1);
			break;
		case 8000:
			icodec_reg_set(icodec, DACCR3, 4, 6, 0x0);
			break;
		default:
			dev_err(component->dev, "icodec not support sample rate.");
			return -EINVAL;
	}

	return 0;
}

static void icodec_dac_disable(struct snd_soc_component *component)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	/* step1: Keep the DAC channel work and input the mute signal. */

	/* step2: select the gain of the DRVL in the DAC left channel.*/
	icodec_reg_set(icodec, HPGR, 0, 4, 0x0);

	/* step3: mute the DRVL module in the DAC left channel. */
	icodec_reg_set(icodec, HPCR, 5, 5, 0x0);

	/* step4: begin the initialization of the DRVL module in the DAC left channel. */
	icodec_reg_set(icodec, HPCR, 4, 4, 0);

	/* step5: disable the DRVL module in the DAC left channel. */
	icodec_reg_set(icodec, HPCR, 0, 0, 0);

	/* step6: disable the DACL module. */
	icodec_reg_set(icodec, DACLCR, 0, 0, 0);

	/* step7: disable the clock module of DACL module. */
	icodec_reg_set(icodec, DACLCR, 1, 1, 0);

	/* step8: disable the reference voltage of DACL module. */
	icodec_reg_set(icodec, DACLCR, 3, 3, 0);

	/* step9: initialize the POP sound in the DAC left channel. */
	icodec_reg_set(icodec, DACLCR, 5, 6, 0x1);

	/* step10: disable the reference voltage buffer of the DAC left channel. */
	icodec_reg_set(icodec, DACLCR, 2, 2, 0);

	/* step11: disable the current source of DAC. */
	icodec_reg_set(icodec, BIASCR1, 1, 1, 0);

	/* step12:  begin the initialization of the DACL module. */
	icodec_reg_set(icodec, DACLCR, 4, 4, 0);
}

static void icodec_adc_enable(struct snd_soc_component *component)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	/* step1: Power up the CODEC according to the section Power Up. */

	/* step2: Set the level range control signal of MIC bias voltage (MICBIAS) and enable micbias. */
	if (1 <= icodec->micbias_level && icodec->micbias_level <= 8) {
		icodec_reg_set(icodec, BIASCR1, 5, 7, icodec->micbias_level - 1);
		icodec_reg_set(icodec, BIASCR1, 3, 3, 0x1);
	}

	/* step3: enable the current source of ADC. */
	icodec_reg_set(icodec, BIASCR1, 2, 2, 0x1);

	/* step4: enable the reference voltage buffer in ADC left channel. */
	icodec_reg_set(icodec, ADCLCR, 2, 2, 0x1);

	/* step5: enable the reference voltage buffer in ADC left channel. */
	icodec_reg_set(icodec, MICCR, 1, 1, 1);

	/* step6: enable the ALC module in ADC left channel. */
	icodec_reg_set(icodec, MICCR, 0, 0, 1);

	/* step7: enable the clock module in ADC left channel. */
	icodec_reg_set(icodec, ADCLCR, 1, 1, 0x1);

	/* step8: enable the ADC module in ADC left channel. */
	icodec_reg_set(icodec, ADCLCR, 0, 0, 0x1);

	/* step9: end the initialization of the ADCL module. */
	icodec_reg_set(icodec, ADCLCR, 4, 4, 0x1);

	/* step10: end the initialization of the left ALC module. */
	icodec_reg_set(icodec, MICCR, 2, 2, 0x1);

	/* step11: end the initialization of the left MIC module. */
	icodec_reg_set(icodec, MICCR, 3, 3, 0x1);
	mdelay(10);

	/* step12: end the mute station of the ADC left channel. */
	icodec_reg_set(icodec, MICCR, 4, 4, 0x1);

	/* step13: select the gain of the left MIC module. */
	icodec_reg_set(icodec, MICGR, 0, 1, 0x0);

	/* step14: select the gain of the left ALC module. */
	icodec_reg_set(icodec, ALCGR, 0, 4, 0x6);

	/* step15: enable the zero-crossing detection function in ADC left channel. */
	icodec_reg_set(icodec, ADCLCR, 3, 3, 0x1);

	/* step16: Configure the ADC digital gain. */
	icodec_reg_set(icodec, ADCDGR, 0, 7, icodec->adc_dgain);

	/* step17: Begin recording. */
}

static int icodec_adc_configure(struct snd_soc_component *component,
                                struct snd_pcm_hw_params *params)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);
	unsigned int fmt_width = snd_pcm_format_width(params_format(params));
	unsigned int channel = params_channels(params);

	if (1 == channel) {
#ifdef CONFIG_SND_ASOC_INGENIC_MONO_LEFT
		icodec_reg_set(icodec, ADCCR1, 6, 7, 0x0);
#endif
#ifdef CONFIG_SND_ASOC_INGENIC_MONO_RIGHT
		icodec_reg_set(icodec, ADCCR1, 6, 7, 0x3);
#endif
	} else {
		icodec_reg_set(icodec, ADCCR1, 6, 7, 0x1);
	}

	/* Configure Audio Sample Depth. */
	switch (fmt_width) {
		case 24:
			icodec_reg_set(icodec, ADCCR1, 4, 5, 0x2);
			break;
		case 20:
			icodec_reg_set(icodec, ADCCR1, 4, 5, 0x1);
			break;
		case 16:
			icodec_reg_set(icodec, ADCCR1, 4, 5, 0x0);
			break;
		default:
			dev_err(component->dev, "icodec not support format width\n");
			return -EINVAL;
	}

	icodec_reg_set(icodec, ADCCR2, 4, 5, 0x3);

	icodec_reg_set(icodec, ADCCR3, 2, 3, 0x3);

	return 0;
}

static void icodec_adc_disable(struct snd_soc_component *component)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	/* step1: Keep ADC channel work and stop recording. */

	/* step2: disable the zero-crossing detection function in ADC left channel. */
	icodec_reg_set(icodec, ADCLCR, 3, 3, 0x0);

	/* step3: disable the ADC module in ADC left channel. */
	icodec_reg_set(icodec, ADCLCR, 0, 0, 0x0);

	/* step4: disable the clock module in ADC left channel. */
	icodec_reg_set(icodec, ADCLCR, 1, 1, 0x0);

	/* step5: disable the ALC module in ADC left channel. */
	icodec_reg_set(icodec, MICCR, 0, 0, 0x0);

	/* step6: disable the MIC module in ADC left channel. */
	icodec_reg_set(icodec, MICCR, 1, 1, 0x0);

	/* step7: disable the reference voltage buffer in ADC left channel. */
	icodec_reg_set(icodec, ADCLCR, 2, 2, 0x0);

	/* step8: disable the current source of ADC. */
	icodec_reg_set(icodec, BIASCR1, 2, 2, 0x0);

	/* step9: disable micbias. */
	if (1 <= icodec->micbias_level && icodec->micbias_level <= 8) {
		icodec_reg_set(icodec, BIASCR1, 3, 3, 0x0);
	}

	/* step10: begin the initialization of the ADCL module. */
	icodec_reg_set(icodec, ADCLCR, 4, 4, 0x0);

	/* step11: begin the initialization of the left ALC module. */
	icodec_reg_set(icodec, MICCR, 2, 2, 0x0);

	/* step12: begin the initialization of the left MIC module. */
	icodec_reg_set(icodec, MICCR, 3, 3, 0x0);
}

static int icodec_probe(struct snd_soc_component *component)
{
	struct icodec *icodec = snd_soc_component_get_drvdata(component);
	dev_info(component->dev, "icodec probe enter\n");

	icodec->component = component;
	return 0;
}

static void icodec_remove(struct snd_soc_component *component)
{
	dev_info(component->dev, "codec icodec remove enter\n");
}

#ifdef CONFIG_PM
static int icodec_suspend(struct snd_soc_component *component)
{
	return 0;
}

static int icodec_resume(struct snd_soc_component *component)
{
	return 0;
}
#endif

static int icodec_startup(struct snd_pcm_substream *substream,
                          struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	if (!icodec->powered) {
		icodec->powered = true;
		icodec_poweron(component);
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		icodec_dac_enable(component);

		if (g_icodec_playback_pwr != NULL) {
			(*g_icodec_playback_pwr)(1);
		}
	}

	return 0;
}

static int icodec_hw_params(struct snd_pcm_substream *substream,
                            struct snd_pcm_hw_params *params,
                            struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;

	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ret = icodec_dac_configure(component, params);
	} else {
		ret = icodec_adc_configure(component, params);
	}

	return 0;
}

static int icodec_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:                    /* i2s format */
			icodec_reg_set(icodec, DACCR1, 2, 3, 0x2);
			icodec_reg_set(icodec, ADCCR1, 2, 3, 0x2);
			break;
		case SND_SOC_DAIFMT_LEFT_J:                 /* msb format */
			icodec_reg_set(icodec, DACCR1, 2, 3, 0x1);
			icodec_reg_set(icodec, ADCCR1, 2, 3, 0x1);
			break;
		default:
			dev_err(component->dev, "icodec not support interface format.\n");
			return -EINVAL;
	}

	return 0;
}

static int icodec_trigger(struct snd_pcm_substream *substream, int cmd,
                          struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			} else {
				icodec_adc_enable(component);
			}
			if (icodec->dac2adc_enable) {
				icodec_reg_set(icodec, ADCLCR, 5, 6, 0x2);
			}
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			} else {
				icodec_adc_disable(component);
			}
			break;
	}

	return 0;
}

static void icodec_shutdown(struct snd_pcm_substream *substream,
                            struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (g_icodec_playback_pwr != NULL) {
			(*g_icodec_playback_pwr)(0);
		}

		icodec_dac_disable(component);
	}

	if (!icodec->powered || (snd_soc_component_active(component) != 0)) {
		return;
	}
	icodec->powered = false;

	icodec_poweroff(component);
}

static int icodec_spk_power(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol, int event)
{
	return 0;
}

static int icodec_controls_get(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	if (!strcmp(kcontrol->id.name, "Speaker Playback Volume")) {
		ucontrol->value.integer.value[0] = icodec->dac_dgain;
	}

	if (!strcmp(kcontrol->id.name, "Capture Volume")) {
		ucontrol->value.integer.value[0] = icodec->adc_dgain;
	}

	return 0;
}

static int icodec_controls_put(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct icodec *icodec = snd_soc_component_get_drvdata(component);

	if (!strcmp(kcontrol->id.name, "Speaker Playback Volume")) {
		icodec->dac_dgain = ucontrol->value.integer.value[0];
		icodec_reg_set(icodec, DACDGR, 0, 7, icodec->dac_dgain);
	}

	if (!strcmp(kcontrol->id.name, "Capture Volume")) {
		icodec->adc_dgain = ucontrol->value.integer.value[0];
		icodec_reg_set(icodec, ADCDGR, 0, 7, icodec->adc_dgain);
	}

	return 0;
}

static struct snd_soc_dai_ops icodec_dai_ops = {
	.hw_params  = icodec_hw_params,
	.set_fmt    = icodec_set_dai_fmt,
	.trigger    = icodec_trigger,
	.shutdown   = icodec_shutdown,
	.startup    = icodec_startup,
};

static struct snd_soc_dai_driver  icodec_codec_dai = {
	.name = "icodec",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = ICODEC_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = ICODEC_FORMATS,
	},
	.ops = &icodec_dai_ops,
};

/**
 * DAC DGAIN volume control:
 *  from -121dB to 6dB in 0.5dB steps
 */
static const DECLARE_TLV_DB_SCALE(dac_dgain_tlv, -12100, 50, 0);

/**
 * ADC DGAIN volume control:
 *  from -95.5dB to 31.5dB in 0.5dB steps
 */
static const DECLARE_TLV_DB_SCALE(adc_dgain_tlv, -9550, 50, 0);

static const struct snd_kcontrol_new icodec_controls[] = {
#if 1
	SOC_SINGLE_EXT_TLV("Speaker Playback Volume", DACDGR, 0, 0xff, 0, icodec_controls_get, icodec_controls_put, dac_dgain_tlv),
	SOC_SINGLE_EXT_TLV("Capture Volume", ADCDGR, 0, 0xff, 0, icodec_controls_get, icodec_controls_put, adc_dgain_tlv),
#else
	SOC_SINGLE_TLV("Speaker Playback Volume", DACDGR, 0, 0xff, 0, dac_dgain_tlv),
	SOC_SINGLE_TLV("Capture Volume", ADCDGR, 0, 0xff, 0, adc_dgain_tlv),
#endif
};

static const struct snd_soc_dapm_widget icodec_widgets[] = {
	SND_SOC_DAPM_AIF_OUT("SDTO", "Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("SDTI", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DACL", NULL, SND_SOC_NOPM, 0, 0),/*pw*/
	SND_SOC_DAPM_ADC("ADCL", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADCR", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ALCL", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ALCR", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SPK("Speaker", icodec_spk_power),
	SND_SOC_DAPM_OUTPUT("HPOUTL"),
	SND_SOC_DAPM_INPUT("MICL"),
	SND_SOC_DAPM_INPUT("MICR"),
};

static const struct snd_soc_dapm_route icodec_route[] = {
	{"DACL",        NULL,   "SDTI"},
	{"HPOUTL",      NULL,   "DACL"},
	{"Speaker",     NULL,   "HPOUTL"},
	{"SDTO",        NULL,   "ADCL"},
	{"ADCL",        NULL,   "ALCL"},
	{"ALCL",        NULL,   "MICL"},
	{"SDTO",        NULL,   "ADCR"},
	{"ADCR",        NULL,   "ALCR"},
	{"ALCR",        NULL,   "MICR"},
};

static struct snd_soc_component_driver soc_codec_dev_icodec_codec = {
	.probe =    icodec_probe,
	.remove =   icodec_remove,
#ifdef CONFIG_PM
	.suspend =  icodec_suspend,
	.resume =   icodec_resume,
#endif

	.read =     icodec_read,
	.write =    icodec_write,

	.controls =     icodec_controls,
	.num_controls = ARRAY_SIZE(icodec_controls),
	.dapm_widgets = icodec_widgets,
	.num_dapm_widgets = ARRAY_SIZE(icodec_widgets),
	.dapm_routes = icodec_route,
	.num_dapm_routes = ARRAY_SIZE(icodec_route),
};

static int icodec_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct icodec *icodec = NULL;
	struct resource *res = NULL;
	int ret = 0, value;

	icodec = (struct icodec *)devm_kzalloc(&pdev->dev,
	                                       sizeof(struct icodec), GFP_KERNEL);
	if (!icodec) {
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	icodec->io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(icodec->io_base)) {
		dev_err(&pdev->dev, "Failed to ioremap mmio memory\n");
		return PTR_ERR(icodec->io_base);
	}

	icodec->dev = &pdev->dev;
	icodec->dac_dgain = 0xe2;
	icodec->adc_dgain = 0xc4;
	icodec->dac2adc_enable = of_property_read_u32(dev->of_node, "dac2adc_enable", &value) ? 0 : value;
	of_property_read_u32(dev->of_node, "ingenic,micbias_level", &icodec->micbias_level);

	spin_lock_init(&icodec->io_lock);
	platform_set_drvdata(pdev, (void *)icodec);

	ret = snd_soc_register_component(&pdev->dev,
	                                 &soc_codec_dev_icodec_codec, &icodec_codec_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Faild to register codec\n");
		platform_set_drvdata(pdev, NULL);
		return ret;
	}

	dev_info(&pdev->dev, "codec icodec platfrom probe success\n");

	return 0;
}

static int icodec_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	platform_set_drvdata(pdev, NULL);

	dev_info(&pdev->dev, "codec icodec platform remove\n");

	return 0;
}

static const struct of_device_id icodec_dt_match[] = {
	{ .compatible = "ingenic,x2600-icodec", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, icodec_dt_match);

static struct platform_driver icodec_inno_v3_driver = {
	.driver = {
		.name = "icodec",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(icodec_dt_match),
	},
	.probe = icodec_platform_probe,
	.remove = icodec_platform_remove,
};
module_platform_driver(icodec_inno_v3_driver);

MODULE_DESCRIPTION("Icodec Codec Driver");
MODULE_AUTHOR("");
MODULE_LICENSE("GPL");
