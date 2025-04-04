/*
 *  sound/soc/ingenic/asoc-spdif.c
 *  ALSA Soc Audio Layer -- ingenic spdif (part of aic controller) driver
 *
 *  Copyright 2014 Ingenic Semiconductor Co.,Ltd
 *  cscheng <shicheng.cheng@ingenic.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <sound/dmaengine_pcm.h>
#include "asoc-aic.h"
#include "asoc-dma.h"

#define I2S_CPM_VALID 0xb0000070
static int ingenic_spdif_debug = 0;
module_param(ingenic_spdif_debug, int, 0644);
#define spdif_DEBUG_MSG(msg...)         \
	do {                    \
		if (ingenic_spdif_debug)        \
			printk("SPDIF: " msg);  \
	} while(0)

struct ingenic_spdif {
	struct device *aic;
	struct snd_dmaengine_dai_dma_data playback_dma_data;
};

struct clk {
	const char *name;
	unsigned long rate;
	struct clk *parent;
	unsigned long flags;
	struct clk_ops *ops;
	int count;
	struct clk *source;
};

#define SPDIF_TFIFO_DEPTH 32
#define INGENIC_SPDIF_FORMATS (SNDRV_PCM_FMTBIT_S24_LE|SNDRV_PCM_FMTBIT_S16_LE)
#define INGENIC_SPDIF_RATE (SNDRV_PCM_RATE_32000|SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000|SNDRV_PCM_RATE_96000|SNDRV_PCM_RATE_192000)

#ifdef CONFIG_AUDIO_DUMP
static void dump_registers(struct device *aic)
{
	struct ingenic_aic *ingenic_aic = dev_get_drvdata(aic);

	pr_info("AIC_FR\t\t%p : 0x%08x\n", (ingenic_aic->vaddr_base + AICFR), ingenic_aic_read_reg(aic, AICFR));
	pr_info("AIC_CR\t\t%p : 0x%08x\n", (ingenic_aic->vaddr_base + AICCR), ingenic_aic_read_reg(aic, AICCR));
	pr_info("AIC_SPCTRL\t%p : 0x%08x\n", (ingenic_aic->vaddr_base + SPCTRL), ingenic_aic_read_reg(aic, SPCTRL));
	pr_info("AIC_SR\t\t%p : 0x%08x\n", (ingenic_aic->vaddr_base + AICSR), ingenic_aic_read_reg(aic, AICSR));
	pr_info("AIC_SPSTATE\t%p : 0x%08x\n", (ingenic_aic->vaddr_base + SPSTATE), ingenic_aic_read_reg(aic, SPSTATE));
	pr_info("AIC_SPCFG1\t%p : 0x%08x\n", (ingenic_aic->vaddr_base + SPCFG1), ingenic_aic_read_reg(aic, SPCFG1));
	pr_info("AIC_SPCFG2\t%p : 0x%08x\n", (ingenic_aic->vaddr_base + SPCFG2), ingenic_aic_read_reg(aic, SPCFG2));
	return;
}
#endif

static int ingenic_spdif_startup(struct snd_pcm_substream *substream,
                                 struct snd_soc_dai *dai)
{
	struct ingenic_spdif *ingenic_spdif = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_spdif->aic;
	enum aic_mode work_mode = AIC_SPDIF_MODE;

	spdif_DEBUG_MSG("enter %s, substream = %s\n",
	                __func__,
	                (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");
	work_mode = aic_set_work_mode(aic, work_mode, true);
	if (work_mode != AIC_SPDIF_MODE) {
		dev_warn(ingenic_spdif->aic, "Aic now is working on %s mode, open spdif mode failed\n",
		         aic_work_mode_str(work_mode));
		return -EPERM;
	}
	printk("start set AIC register....\n");
	return 0;
}

static unsigned long  spdif_select_ori_sample_freq(struct device *aic, unsigned long sync)
{
	int div = 0;
	switch (sync) {
	case 8000: div = 0x6;
		break;
	case 11025: div = 0xa;
		break;
	case 16000: div = 0x8;
		break;
	case 22050: div = 0xb;
		break;
	case 24000: div = 0x9;
		break;
	case 32000: div = 0xc;
		break;
	case 44100: div = 0xf;
		break;
	case 48000: div = 0xd;
		break;
	case 96000: div = 0x5;
		break;
	case 192000: div = 0x1;
		break;
	default :
		div = 0xf;
		break;
	}
	__spdif_set_ori_sample_freq(aic, div);
	return div;
}

static unsigned long  spdif_select_sample_freq(struct device *aic, unsigned long sync)
{
	int div = 0;
	switch (sync) {
	case 8000: div = 0x9;
		break;
	case 11025: div = 0x5;
		break;
	case 16000: div = 0x7;
		break;
	case 22050: div = 0x4;
		break;
	case 24000: div = 0x6;
		break;
	case 32000: div = 0x3;
		break;
	case 44100: div = 0x0;
		break;
	case 48000: div = 0x2;
		break;
	case 96000: div = 0xa;
		break;
	case 192000: div = 0xe;
		break;
	default :
		div = 0;
		break;
	}
	__spdif_set_sample_freq(aic, div);
	return div;
}
static int ingenic_spdif_set_rate(struct device *aic, struct ingenic_aic *ingenic_aic, unsigned long sample_rate)
{
	struct clk *cgu_aic_clk = ingenic_aic->clk;

	__i2s_stop_bitclk(aic);
	clk_set_rate(cgu_aic_clk, sample_rate);
	writel(0xa, (volatile void *)I2S_CPM_VALID);
	__i2s_start_bitclk(aic);
	spdif_select_ori_sample_freq(aic, sample_rate);
	spdif_select_sample_freq(aic, sample_rate);

	return sample_rate;
}

static int ingenic_spdif_hw_params(struct snd_pcm_substream *substream,
                                   struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	int channels = params_channels(params);
	int fmt_width = snd_pcm_format_width(params_format(params));
	struct ingenic_spdif *ingenic_spdif = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_spdif->aic;
	struct ingenic_aic *ingenic_aic = dev_get_drvdata(aic);
	enum dma_slave_buswidth buswidth;
	int trigger;
	unsigned long sample_rate = params_rate(params);
	unsigned long tmp_rate = 0;

	spdif_DEBUG_MSG("enter %s, substream = %s\n", __func__,
	                (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	if (!((1 << params_format(params)) & INGENIC_SPDIF_FORMATS) || channels > 2) {
		dev_err(dai->dev, "hw params not inval channel %d params %x\n",
		        channels, params_format(params));
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		__i2s_channel(aic, channels);
		/* format */
		if (fmt_width == 16) {
			buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
			__spdif_set_max_wl(aic, 0);
			__spdif_set_sample_size(aic, 1);
		} else if (fmt_width == 24) {
			buswidth = DMA_SLAVE_BUSWIDTH_4_BYTES;
			__spdif_set_max_wl(aic, 1);
			__spdif_set_sample_size(aic, 5);
		} else {
			return -EINVAL;
		}

		ingenic_spdif->playback_dma_data.addr_width = buswidth;
		ingenic_spdif->playback_dma_data.maxburst = (SPDIF_TFIFO_DEPTH * buswidth) / 2;
		trigger = SPDIF_TFIFO_DEPTH - (ingenic_spdif->playback_dma_data.maxburst / (int)buswidth);
		//      __i2s_set_transmit_trigger(aic,trigger/2);
		__spdif_set_trigger(aic, 3);
		//      snd_soc_dai_set_dma_data(dai, substream, (void *)&ingenic_spdif->playback_dma_data);

	} else {
		printk("spdif is not a capture device!\n");
		return -EINVAL;
	}

	/* sample rate */
	tmp_rate = ingenic_spdif_set_rate(aic, ingenic_aic, sample_rate);
	if (tmp_rate < 0) {
		printk("set spdif clk failed!!\n");
	}
	/* signed transfer */
	if (snd_pcm_format_signed(params_format(params))) {
		__spdif_clear_signn(aic);
	} else {
		__spdif_set_signn(aic);
	}
	return 0;
}

static int ingenic_spdif_start_substream(struct snd_pcm_substream *substream,
        struct snd_soc_dai *dai)
{
	struct ingenic_spdif *ingenic_spdif = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_spdif->aic;
	int rst_test = 0xfff;
	spdif_DEBUG_MSG("enter %s, substream = %s\n",
	                __func__,
	                (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		__spdif_reset(aic);
		while (__spdif_get_reset(aic)) {
			if (rst_test-- <= 0) {
				printk("spdif rst err\n");
				return -EINVAL;
			}
		}
		__spdif_enable_transmit_dma(aic);
		__spdif_clear_underrun(aic);
		__spdif_enable(aic);
		__aic_enable(aic);
	} else {
		printk("spdif is not a capture device!\n");
		return -EINVAL;
	}
	return 0;
}

static int ingenic_spdif_stop_substream(struct snd_pcm_substream *substream,
                                        struct snd_soc_dai *dai)
{
	struct ingenic_spdif *ingenic_spdif = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_spdif->aic;
	spdif_DEBUG_MSG("enter %s, substream = %s\n",
	                __func__,
	                (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (ingenic_spdif_debug) {
			__aic_dis_tur_int(aic);
		}
		if (__spdif_is_enable_transmit_dma(aic)) {
			__spdif_disable_transmit_dma(aic);
			__spdif_clear_underrun(aic);
			/*hrtime mode: stop will be happen in any where, make sure there is
			 *  no data transfer on ahb bus before stop dma
			 */
			while (!__spdif_test_underrun(aic));
		}
		__spdif_disable(aic);
		__spdif_clear_underrun(aic);
	} else {
		printk("spdif is not a capture device!\n");
		return -EINVAL;
	}

	return 0;
}

static int ingenic_spdif_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	struct ingenic_pcm_runtime_data *prtd = substream->runtime->private_data;
	spdif_DEBUG_MSG("enter %s, substream = %s cmd = %d\n",
	                __func__,
	                (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture",
	                cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (atomic_read(&prtd->wait_stopdma)) {
			return -EPIPE;
		}
		if (ingenic_spdif_start_substream(substream, dai)) {
			return -EINVAL;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (atomic_read(&prtd->wait_stopdma)) {
			return 0;
		}
		ingenic_spdif_stop_substream(substream, dai);
		break;
	}
	/*dump_registers(aic);*/
	return 0;
}

static void ingenic_spdif_shutdown(struct snd_pcm_substream *substream,
                                   struct snd_soc_dai *dai)
{
	struct ingenic_spdif *ingenic_spdif = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_spdif->aic;
	enum aic_mode work_mode = AIC_SPDIF_MODE;

	spdif_DEBUG_MSG("enter %s, substream = %s\n",
	                __func__, (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");
	work_mode = aic_set_work_mode(ingenic_spdif->aic, work_mode, false);
	BUG_ON((work_mode != AIC_NO_MODE));
	ingenic_spdif_stop_substream(substream, dai);
	__aic_disable(aic);
	return;
}

static int ingenic_spdif_probe(struct snd_soc_dai *dai)
{
	struct ingenic_spdif *ingenic_spdif = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_spdif->aic;
	unsigned int reg_tmp;
	snd_soc_dai_init_dma_data(dai, &ingenic_spdif->playback_dma_data, NULL);

	spdif_DEBUG_MSG("enter %s\n", __func__);

	__i2s_disable_transmit_dma(aic);
	__i2s_disable_receive_dma(aic);
	__i2s_disable_replay(aic);
	__i2s_disable_record(aic);
	__i2s_disable_loopback(aic);
	__aic_disable(aic);

	reg_tmp = ingenic_aic_read_reg(aic, SPCTRL);
	reg_tmp |= (SPCTRL_SPDIF_I2S_MASK | SPCTRL_M_TRIG_MASK |
	            SPCTRL_M_FFUR_MASK | SPCTRL_INVALID_MASK);
	ingenic_aic_write_reg(aic, SPCTRL, reg_tmp);

	__i2s_stop_bitclk(aic);
	__aic_select_external_codec(aic);
	__i2s_bclk_output(aic);
	__i2s_sync_output(aic);
	__aic_select_i2s(aic);
	__i2s_send_rfirst(aic);

	__spdif_set_dtype(aic, 0);
	__spdif_set_ch1num(aic, 0);
	__spdif_set_ch2num(aic, 1);
	__spdif_set_srcnum(aic, 0);

	__interface_select_spdif(aic);
	__spdif_play_lastsample(aic);
	__spdif_init_set_low(aic);
	__spdif_choose_consumer(aic);
	__spdif_clear_audion(aic);
	__spdif_set_copyn(aic);
	__spdif_clear_pre(aic);
	__spdif_choose_chmd(aic);
	__spdif_set_category_code_normal(aic);
	__spdif_set_clkacu(aic, 0);
	__spdif_set_sample_size(aic, 1);
	__spdif_set_valid(aic);
	__spdif_mask_trig(aic);
	__spdif_disable_underrun_intr(aic);
	/*select spdif trans*/
	printk("spdif cpu dai prob ok\n");

	return 0;
}

static struct snd_soc_dai_ops ingenic_spdif_dai_ops = {
	.startup = ingenic_spdif_startup,
	.trigger    = ingenic_spdif_trigger,
	.hw_params  = ingenic_spdif_hw_params,
	.shutdown   = ingenic_spdif_shutdown,
};

#define ingenic_spdif_suspend NULL
#define ingenic_spdif_resume    NULL
static struct snd_soc_dai_driver ingenic_spdif_dai = {
	.probe   = ingenic_spdif_probe,
	.suspend = ingenic_spdif_suspend,
	.resume  = ingenic_spdif_resume,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = INGENIC_SPDIF_RATE,
		.formats = INGENIC_SPDIF_FORMATS,
	},
	.ops = &ingenic_spdif_dai_ops,
};

#ifdef CONFIG_AUDIO_DUMP
static ssize_t ingenic_spdif_regs_show(struct device *dev,
                                       struct device_attribute *attr, char *buf)
{
	struct ingenic_spdif *ingenic_spdif = dev_get_drvdata(dev);
	dump_registers(ingenic_spdif->aic);
	return 0;
}

static struct device_attribute ingenic_spdif_sysfs_attrs[] = {
	__ATTR(spdif_regs, S_IRUGO, ingenic_spdif_regs_show, NULL),
};
#endif

static const struct snd_soc_component_driver ingenic_spdif_component = {
	.name       = "aic-spdif",
};

static int ingenic_spdif_platfrom_probe(struct platform_device *pdev)
{
	//  struct ingenic_aic_subdev_pdata *pdata = dev_get_platdata(&pdev->dev);
	struct device_node *parent = of_get_parent(pdev->dev.of_node);
	struct resource res;
	struct ingenic_spdif *ingenic_spdif;
	int i = 0, ret;

	if (!parent) {
		return -ENOMEM;
	}
	ret = of_address_to_resource(parent, 0, &res);
	if (ret) {
		return ret;
	}

	ingenic_spdif = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_spdif), GFP_KERNEL);
	if (!ingenic_spdif) {
		return -ENOMEM;
	}

	ingenic_spdif->aic = pdev->dev.parent;

	ingenic_spdif->playback_dma_data.addr = res.start + SPFIFO;
	platform_set_drvdata(pdev, (void *)ingenic_spdif);

#ifdef CONFIG_AUDIO_DUMP
	for (; i < ARRAY_SIZE(ingenic_spdif_sysfs_attrs); i++) {
		ret = device_create_file(&pdev->dev, &ingenic_spdif_sysfs_attrs[i]);
		if (ret)
			dev_warn(&pdev->dev, "attribute %s create failed %x",
			         ingenic_spdif_sysfs_attrs[i].attr.name, ret);
	}
#endif

	ret = snd_soc_register_component(&pdev->dev, &ingenic_spdif_component,
	                                 &ingenic_spdif_dai, 1);
	if (!ret) {
		dev_dbg(&pdev->dev, "spdif platform probe success\n");
	}
	return ret;
}

static int ingenic_spdif_platfom_remove(struct platform_device *pdev)
{
	int i;
#ifdef CONFIG_AUDIO_DUMP
	for (i = 0; i < ARRAY_SIZE(ingenic_spdif_sysfs_attrs); i++) {
		device_remove_file(&pdev->dev, &ingenic_spdif_sysfs_attrs[i]);
	}
#endif
	platform_set_drvdata(pdev, NULL);
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}
static const struct of_device_id spdif_dt_match[] = {
	{ .compatible = "ingenic,spdif", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, spdif_dt_match);
static struct platform_driver ingenic_spdif_plat_driver = {
	.probe  = ingenic_spdif_platfrom_probe,
	.remove = ingenic_spdif_platfom_remove,
	.driver = {
		.name = "ingenic-asoc-aic-spdif",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(spdif_dt_match),
	},
};

module_platform_driver(ingenic_spdif_plat_driver);
MODULE_AUTHOR("shicheng.cheng <shicheng.cheng@ingenic.com>");
MODULE_DESCRIPTION("INGENIC AIC SPDIF SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-aic-spdif");
