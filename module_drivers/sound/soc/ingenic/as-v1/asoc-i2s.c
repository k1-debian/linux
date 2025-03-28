/*
 *  sound/soc/ingenic/asoc-i2s.c
 *  ALSA Soc Audio Layer -- ingenic i2s (part of aic controller) driver
 *
 *  Copyright 2014 Ingenic Semiconductor Co.,Ltd
 *  cli <chen.li@ingenic.com>
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

static int ingenic_i2s_debug = 0;
module_param(ingenic_i2s_debug, int, 0644);
#define I2S_DEBUG_MSG(msg...)                   \
	do {                                        \
		if (ingenic_i2s_debug)                  \
			pr_debug("I2S: " msg);              \
	} while(0)

struct ingenic_i2s_priv {
	bool ctrl_sysclk_output;
	bool have_internal_codec;
	bool ctrl_stop_bitclk;
	bool have_share_clk;
	bool init_clk_direction;
	bool ctrl_record_channel;
};

struct ingenic_i2s {
	struct device *aic; /*register access device*/
#define I2S_WRITE 0x1
#define I2S_READ  0x2
	int i2s_mode;
	struct snd_dmaengine_dai_dma_data playback_dma_data;
	struct snd_dmaengine_dai_dma_data capture_dma_data;
	struct snd_soc_dai_driver dai_driver;
	dma_addr_t dma_base;
	struct clk *cgu_clk;
	int playback_channels;
	const struct ingenic_i2s_priv *priv;
};

#define I2S_RFIFO_DEPTH 32
#define I2S_TFIFO_DEPTH 64
#define TX_FIFO_LEVEL 16

static const struct ingenic_i2s_priv common_priv_data[];
static const struct of_device_id i2s_dt_match[];

#ifdef CONFIG_AUDIO_DUMP
static void dump_registers(struct device *aic)
{
	struct ingenic_aic *ingenic_aic = dev_get_drvdata(aic);

	pr_info("AIC_FR\t\t%p : 0x%08x\n", (ingenic_aic->vaddr_base + AICFR), ingenic_aic_read_reg(aic, AICFR));
	pr_info("AIC_CR\t\t%p : 0x%08x\n", (ingenic_aic->vaddr_base + AICCR), ingenic_aic_read_reg(aic, AICCR));
	pr_info("AIC_I2SCR\t%p : 0x%08x\n", (ingenic_aic->vaddr_base + I2SCR), ingenic_aic_read_reg(aic, I2SCR));
	pr_info("AIC_SR\t\t%p : 0x%08x\n", (ingenic_aic->vaddr_base + AICSR), ingenic_aic_read_reg(aic, AICSR));
	pr_info("AIC_I2SSR\t%p : 0x%08x\n", (ingenic_aic->vaddr_base + I2SSR), ingenic_aic_read_reg(aic, I2SSR));
	pr_info("AIC_I2SDIV\t%p : 0x%08x\n", (ingenic_aic->vaddr_base + I2SDIV), ingenic_aic_read_reg(aic, I2SDIV));
	pr_info("AIC_DR\t\t%p\n", (ingenic_aic->vaddr_base + AICDR));
	return;
}
#endif

static int ingenic_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_i2s->aic;

	I2S_DEBUG_MSG("enter %s dai fmt %x\n", __func__, fmt);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:        /*i2s format*/
			__i2s_select_i2s_fmt(aic);
			break;
		case SND_SOC_DAIFMT_MSB:
			__i2s_select_msb_fmt(aic);  /*msb format*/
			break;
		default:
			return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			__i2s_send_lfirst(aic);
			break;
		case SND_SOC_DAIFMT_NB_IF:
			__i2s_send_rfirst(aic);
			break;
		default:
			return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBM_CFM:    /*sync and bit clk (codec master : i2s slave)*/
			__i2s_bclk_input(aic);
			__i2s_sync_input(aic);
			break;
		case SND_SOC_DAIFMT_CBS_CFM:
			__i2s_bclk_output(aic);
			__i2s_sync_output(aic);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int ingenic_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_i2s->aic;

	I2S_DEBUG_MSG("enter %s div_id %d div %d\n", __func__, div_id, div);

	if (ingenic_i2s->priv->have_share_clk) {
		if (div_id & INGENIC_I2S_PLAYBACK) {
			__i2s_set_dv(aic, div);
		} else {
#ifdef CONFIG_SND_ASOC_INGENIC_SHARE_CLK
			__i2s_set_dv(aic, div);
#else
			__i2s_set_idv(aic, div);
#endif
		}
	}

	if (ingenic_i2s->priv->have_internal_codec) {
		if (div_id & INGENIC_I2S_INNER_CODEC) {
			__i2s_set_mdiv(aic, div);
		}
	}

	return 0;
}

static int ingenic_set_sysclk(struct snd_soc_dai *dai, int clk_id,
                              unsigned int freq, int dir)
{
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_i2s->aic;

	I2S_DEBUG_MSG("enter %s clk_id %d req %d clk dir %d\n", __func__,
	              clk_id, freq, dir);

	if (ingenic_i2s->priv->have_internal_codec) {
		if (clk_id & INGENIC_I2S_INNER_CODEC) {
			__aic_select_internal_codec(aic);
			__i2s_codec_slave(aic);
		} else {
			__aic_select_external_codec(aic);
			__i2s_codec_master(aic);
		}
	}

	if (clk_id & INGENIC_I2S_PLAYBACK) {
		aic_set_rate(aic, freq, STREAM_PLAY);
	} else if (clk_id & INGENIC_I2S_CAPTURE) {
		if (ingenic_i2s->priv->have_share_clk) {
#ifdef CONFIG_SND_ASOC_INGENIC_SHARE_CLK
			aic_set_rate(aic, freq, STREAM_PLAY);
#else
			aic_set_rate(aic, freq, STREAM_RECORD);
#endif
		} else {
			aic_set_rate(aic, freq, STREAM_RECORD);
		}
	} else {
		aic_set_rate(aic, freq, STREAM_COMMON);
	}

	if (ingenic_i2s->priv->ctrl_sysclk_output) {
		if (dir  == SND_SOC_CLOCK_OUT) {
			__i2s_select_sysclk_output(aic);
		} else {
			__i2s_select_sysclk_input(aic);
		}
	}

	return 0;
}

static int ingenic_i2s_startup(struct snd_pcm_substream *substream,
                               struct snd_soc_dai *dai)
{
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_i2s->aic;
	enum aic_mode work_mode = AIC_I2S_MODE;

	I2S_DEBUG_MSG("enter %s, substream = %s\n", __func__,
	              (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	work_mode = aic_set_work_mode(aic, work_mode, true);
	if (work_mode != AIC_I2S_MODE) {
		dev_warn(ingenic_i2s->aic, "Aic now is working on %s mode, open i2s mode failed\n",
		         aic_work_mode_str(work_mode));
		return -EBUSY;
	}

	if (!ingenic_i2s->i2s_mode) {
		__aic_select_i2s(aic);
		__i2s_play_lastsample(aic);

		if (ingenic_i2s->priv->have_share_clk) {
#ifdef CONFIG_SND_ASOC_INGENIC_SHARE_CLK
			__i2s_share_clk(aic);
#else
			__i2s_independent_clk(aic);
#endif
		}

		__i2s_aic_unpacket16(aic);
		__aic_enable(aic);
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ingenic_i2s->i2s_mode |= I2S_WRITE;
	} else {
		ingenic_i2s->i2s_mode |= I2S_READ;
	}

	aic_clk_ctrl(aic, true);
	return 0;
}

static int ingenic_i2s_hw_params(struct snd_pcm_substream *substream,
                                 struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	int channels = params_channels(params);
	int fmt_width = snd_pcm_format_width(params_format(params));
	int phys_width = snd_pcm_format_physical_width(params_format(params));
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_i2s->aic;
	enum dma_slave_buswidth buswidth;
	int trigger;

	I2S_DEBUG_MSG("enter %s, substream = %s\n", __func__,
	              (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	/* format */
	if (phys_width == 8) {
		buswidth = DMA_SLAVE_BUSWIDTH_1_BYTE;
	} else if (phys_width == 16) {
		buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
	} else if (phys_width == 24 || phys_width == 20) {
		buswidth = DMA_SLAVE_BUSWIDTH_3_BYTES;
	} else if (phys_width == 32) {
		buswidth = DMA_SLAVE_BUSWIDTH_4_BYTES;
	} else {
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* channel */
		ingenic_i2s->playback_channels = channels;
		__i2s_channel(aic, ingenic_i2s->playback_channels);

		if (channels == 1) {
			__i2s_m2s_enable(aic);    /*unavailed ??*/
		} else {
			__i2s_m2s_disable(aic);
		}

		ingenic_i2s->playback_dma_data.addr_width = buswidth;
		ingenic_i2s->playback_dma_data.maxburst = I2S_TFIFO_DEPTH / 2;

		trigger = I2S_TFIFO_DEPTH - (ingenic_i2s->playback_dma_data.maxburst);
		__i2s_set_oss(aic, fmt_width);
		__i2s_set_transmit_trigger(aic, (trigger / 2));
	} else {
		ingenic_i2s->capture_dma_data.addr_width = buswidth;
		ingenic_i2s->capture_dma_data.maxburst = I2S_TFIFO_DEPTH / 2;

		trigger = ingenic_i2s->capture_dma_data.maxburst;
		__i2s_set_iss(aic, fmt_width);
		__i2s_set_receive_trigger(aic, (trigger / 2 - 1));

		if (ingenic_i2s->priv->ctrl_record_channel) {
			if (channels == 2) {
				__i2s_enable_stereo(aic);
			} else if (channels == 1) {
#ifdef CONFIG_SND_ASOC_INGENIC_MONO_LEFT
				__i2s_enable_monoctr_left(aic);
#endif
#ifdef CONFIG_SND_ASOC_INGENIC_MONO_RIGHT
				__i2s_enable_monoctr_right(aic);
#endif
			}
		}
	}

	return 0;
}

static void ingenic_i2s_start_substream(struct snd_pcm_substream *substream,
                                        struct snd_soc_dai *dai)
{
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_i2s->aic;
	int level, i, level_d, discard;
	I2S_DEBUG_MSG("enter %s, substream = %s\n", __func__,
	              (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		int timeout = 1000;
		__aic_clear_tur(aic);
		__i2s_enable_replay(aic);
		__aic_flush_txfifo(aic);

		/* Wait flush fifo complete */
		while (__aic_read_tfl(aic)) {
			if (--timeout < 0) {
				dev_printk(KERN_DEBUG, dai->dev, "Flush transmit FIFO timeout !\n");
				break;
			}
		}

		__i2s_enable_transmit_dma(aic);

		level = __aic_read_tfl(aic);
		I2S_DEBUG_MSG("aic dirty fifo size %d\n", level);
		if (ingenic_i2s_debug) {
			__aic_en_tur_int(aic);
		}
	} else {
		level_d = level = __aic_read_rfl(aic);
		__aic_clear_ror(aic);

		__i2s_enable_record(aic);

		/*Left and right channel will inversion, no sure, just makesure*/
		for (i = 0; i < level; i++) {
			discard = __aic_read_rxfifo(aic);
		}

		__i2s_enable_receive_dma(aic);

		I2S_DEBUG_MSG("aic dirty fifo size %d\n", level_d);

		if (ingenic_i2s_debug) {
			__aic_en_ror_int(aic);
		}
	}
	return;
}

static void ingenic_i2s_stop_substream(struct snd_pcm_substream *substream,
                                       struct snd_soc_dai *dai)
{
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_i2s->aic;
	I2S_DEBUG_MSG("enter %s, substream = %s\n", __func__,
	              (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (ingenic_i2s_debug) {
			__aic_dis_tur_int(aic);
		}
		if (__i2s_transmit_dma_is_enable(aic)) {
			__i2s_disable_transmit_dma(aic);
		}
		__i2s_disable_replay(aic);
	} else {
		if (ingenic_i2s_debug) {
			__aic_dis_ror_int(aic);
		}
		if (__i2s_receive_dma_is_enable(aic)) {
			__i2s_disable_receive_dma(aic);
		}
		__i2s_disable_record(aic);
	}
	return;
}

static int ingenic_i2s_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	struct ingenic_pcm_runtime_data *prtd = substream_to_prtd(substream);

	I2S_DEBUG_MSG("enter %s, substream = %s cmd = %d\n",  __func__,
	              (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture",
	              cmd);

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			if (atomic_read(&prtd->wait_stopdma)) {
				return -EPIPE;
			}
			I2S_DEBUG_MSG("i2s start\n");

			ingenic_i2s_start_substream(substream, dai);
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			if (atomic_read(&prtd->wait_stopdma)) {
				return 0;
			}
			I2S_DEBUG_MSG("i2s stop\n");

			ingenic_i2s_stop_substream(substream, dai);

			break;
	}
	return 0;
}

static void ingenic_i2s_shutdown(struct snd_pcm_substream *substream,
                                 struct snd_soc_dai *dai)
{
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_i2s->aic;
	enum aic_mode work_mode = AIC_I2S_MODE;

	I2S_DEBUG_MSG("enter %s, substream = %s\n", __func__,
	              (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	work_mode = aic_set_work_mode(ingenic_i2s->aic, work_mode, false);
	BUG_ON((work_mode != AIC_NO_MODE));

	ingenic_i2s_stop_substream(substream, dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ingenic_i2s->i2s_mode &= ~I2S_WRITE;
	} else {
		ingenic_i2s->i2s_mode &= ~I2S_READ;
	}

	if (!ingenic_i2s->i2s_mode) {
		__aic_disable(aic);
	}

	aic_clk_ctrl(aic, false);
	return;
}

static int ingenic_i2s_probe(struct snd_soc_dai *dai)
{
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = ingenic_i2s->aic;

	ingenic_i2s->playback_dma_data.addr = ingenic_i2s->dma_base + AICDR;
	ingenic_i2s->playback_dma_data.fifo_size = I2S_TFIFO_DEPTH;
	ingenic_i2s->capture_dma_data.addr = ingenic_i2s->dma_base + AICDR;
	ingenic_i2s->capture_dma_data.fifo_size = I2S_RFIFO_DEPTH;

	snd_soc_dai_init_dma_data(dai, &ingenic_i2s->playback_dma_data,
	                          &ingenic_i2s->capture_dma_data);

	aic_clk_ctrl(aic, true);
	__aic_reset(aic);
	udelay(1);

	/* inner codec(icdc) probe must have mclk*/
	__aic_disable(aic);

	if (ingenic_i2s->priv->ctrl_stop_bitclk) {
		__i2s_stop_bitclk(aic);
		__aic_select_i2s(aic);

		if (ingenic_i2s->priv->init_clk_direction) {
			__i2s_codec_slave(aic);
			__i2s_slave_clkset(aic);
		}

		__aic_select_internal_codec(aic);
		__i2s_enable_sysclk_output(aic);
		__i2s_select_sysclk_output(aic);
		__i2s_start_bitclk(aic);
	} else {
		__aic_select_i2s(aic);
		if (ingenic_i2s->priv->init_clk_direction) {
			__i2s_bclk_output(aic);
			__i2s_sync_output(aic);
		}
	}
#if (defined(CONFIG_SOC_X2600) && defined(CONFIG_DMIC_AND_AMIC_SYNC))
	__aic_enable_rec_exen(aic);
#endif

	__aic_enable(aic);
	aic_clk_ctrl(aic, false);
	return 0;
}

static struct snd_soc_dai_ops ingenic_i2s_dai_ops = {
	.probe      = ingenic_i2s_probe,
	.startup    = ingenic_i2s_startup,
	.trigger    = ingenic_i2s_trigger,
	.hw_params  = ingenic_i2s_hw_params,
	.shutdown   = ingenic_i2s_shutdown,
	.set_fmt    = ingenic_set_dai_fmt,
	.set_sysclk = ingenic_set_sysclk,
	.set_clkdiv = ingenic_set_clkdiv,
};

// #define ingenic_i2s_suspend  NULL
// #define ingenic_i2s_resume   NULL

#ifdef CONFIG_PM
static int ingenic_i2s_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ingenic_i2s *ingenic_i2s = platform_get_drvdata(pdev);
	struct device *aic = ingenic_i2s->aic;

	if (__i2s_record_is_enable(aic) || __i2s_replay_is_enable(aic)) {
		return -1;
	}

	return 0;
}

static int ingenic_i2s_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

#ifdef CONFIG_AUDIO_DUMP
static ssize_t ingenic_i2s_regs_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	struct ingenic_i2s *ingenic_i2s = dev_get_drvdata(dev);
	dump_registers(ingenic_i2s->aic);
	return 0;
}

static DEVICE_ATTR(i2s_regs, S_IRUGO, ingenic_i2s_regs_show, NULL);
static const struct attribute *dump_attrs[] = {
	&dev_attr_i2s_regs.attr,
	NULL,
};

static const struct attribute_group dump_attr_group = {
	.attrs = (struct attribute **)dump_attrs,
};
#endif

static const struct snd_soc_component_driver ingenic_i2s_component = {
	.name       = "aic-i2s",
};

#define INGENIC_I2S_FORMATS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_U8 | \
                             SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE | \
                             SNDRV_PCM_FMTBIT_U16_LE | SNDRV_PCM_FMTBIT_U16_BE | \
                             SNDRV_PCM_FMTBIT_S20_LE | SNDRV_PCM_FMTBIT_S20_BE | \
                             SNDRV_PCM_FMTBIT_U20_LE | SNDRV_PCM_FMTBIT_U20_BE | \
                             SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE | \
                             SNDRV_PCM_FMTBIT_U24_LE | SNDRV_PCM_FMTBIT_U24_BE | \
                             SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_U24_3LE | \
                             SNDRV_PCM_FMTBIT_S24_3BE | SNDRV_PCM_FMTBIT_U24_3BE | \
                             SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_U20_3LE | \
                             SNDRV_PCM_FMTBIT_S20_3BE | SNDRV_PCM_FMTBIT_U20_3BE | \
                             SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_U18_3LE | \
                             SNDRV_PCM_FMTBIT_S18_3BE | SNDRV_PCM_FMTBIT_U18_3BE )

static int ingenic_i2s_platfrom_probe(struct platform_device *pdev)
{
	struct device_node *parent = of_get_parent(pdev->dev.of_node);
	struct resource res;
	struct ingenic_i2s *ingenic_i2s;
	const struct of_device_id *match;
	int ret;

	if (!parent) {
		return -ENOMEM;
	}

	ingenic_i2s = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_i2s), GFP_KERNEL);
	if (!ingenic_i2s) {
		return -ENOMEM;
	}

	match = of_match_node(i2s_dt_match, pdev->dev.of_node);
	if (!match) {
		return -ENODEV;
	}

	ingenic_i2s->priv = !match->data ? common_priv_data : (struct ingenic_i2s_priv *)match->data;
	ret = of_address_to_resource(parent, 0, &res);
	if (ret) {
		return ret;
	}

	ingenic_i2s->dma_base = res.start;
	ingenic_i2s->aic = pdev->dev.parent;
	ingenic_i2s->i2s_mode = 0;
	ingenic_i2s->dai_driver.name = "i2s";
	//ingenic_i2s->dai_driver.suspend = ingenic_i2s_suspend;
	//ingenic_i2s->dai_driver.resume = ingenic_i2s_resume;
	ingenic_i2s->dai_driver.ops = &ingenic_i2s_dai_ops;
	ingenic_i2s->dai_driver.playback.channels_min = 1;
	ingenic_i2s->dai_driver.playback.channels_max = 2;
	ingenic_i2s->dai_driver.playback.rates = SNDRV_PCM_RATE_8000_192000;
	ingenic_i2s->dai_driver.playback.formats = INGENIC_I2S_FORMATS;
	ingenic_i2s->dai_driver.capture.channels_min = 1;
	ingenic_i2s->dai_driver.capture.channels_max = 2;
	ingenic_i2s->dai_driver.capture.rates = SNDRV_PCM_RATE_8000_192000;
	ingenic_i2s->dai_driver.capture.formats = INGENIC_I2S_FORMATS;
	ingenic_i2s->playback_channels = 2; /*default value*/

	platform_set_drvdata(pdev, (void *)ingenic_i2s);

	ret = devm_snd_soc_register_component(&pdev->dev, &ingenic_i2s_component,
	                                      &ingenic_i2s->dai_driver, 1);
	if (!ret) {
		dev_info(&pdev->dev, "i2s platform probe success\n");
	}

#ifdef CONFIG_AUDIO_DUMP
	ret = sysfs_create_group(&pdev->dev.kobj, &dump_attr_group);
	if (ret) {
		dev_info(&pdev->dev, "i2s attr create failed\n");
	}
#endif

	return ret;
}

static const struct ingenic_i2s_priv common_priv_data[] = {
	{
		.ctrl_sysclk_output = true,
		.have_internal_codec = true,
		.ctrl_stop_bitclk = true,
		.have_share_clk = false,
		.init_clk_direction = false,
		.ctrl_record_channel = false,
	},
};

static const struct ingenic_i2s_priv x1600_priv_data[] = {
	{
		.ctrl_sysclk_output = false,
		.have_internal_codec = false,
		.ctrl_stop_bitclk = false,
		.have_share_clk = true,
		.init_clk_direction = true,
		.ctrl_record_channel = true,
	},
};

static const struct ingenic_i2s_priv x2500_priv_data[] = {
	{
		.ctrl_sysclk_output = true,
		.have_internal_codec = true,
		.ctrl_stop_bitclk = true,
		.have_share_clk = false,
		.init_clk_direction = true,
		.ctrl_record_channel = true,
	},
};

static const struct ingenic_i2s_priv x2600_priv_data[] = {
	{
		.ctrl_sysclk_output = false,
		.have_internal_codec = true,
		.ctrl_stop_bitclk = false,
		.have_share_clk = true,
		.init_clk_direction = true,
		.ctrl_record_channel = true,
	},
};

static const struct of_device_id i2s_dt_match[] = {
	{ .compatible = "ingenic,i2s", .data = (void *)common_priv_data },
	{ .compatible = "ingenic,x1600-i2s", .data = (void *)x1600_priv_data },
	{ .compatible = "ingenic,x2500-i2s", .data = (void *)x2500_priv_data },
	{ .compatible = "ingenic,x2600-i2s", .data = (void *)x2600_priv_data },
	{},
};
MODULE_DEVICE_TABLE(of, i2s_dt_match);

static struct platform_driver ingenic_i2s_plat_driver = {
	.probe  = ingenic_i2s_platfrom_probe,
	.driver = {
		.name = "asoc-aic-i2s",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(i2s_dt_match),
	},
#ifdef CONFIG_PM
	.suspend = ingenic_i2s_suspend,
	.resume = ingenic_i2s_resume,
#endif
};
module_platform_driver(ingenic_i2s_plat_driver);

MODULE_AUTHOR("cli <chen.li@ingenic.com>");
MODULE_DESCRIPTION("INGENIC AIC I2S SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-aic-i2s");
