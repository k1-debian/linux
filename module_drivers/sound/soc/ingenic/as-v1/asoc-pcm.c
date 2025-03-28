/*
 *  sound/soc/ingenic/asoc-pcm.c
 *  ALSA Soc Audio Layer -- ingenic pcm (part of aic controller) driver
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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <linux/slab.h>
#include "asoc-pcm.h"

static int ingenic_pcm_debug = 0;
module_param(ingenic_pcm_debug, int, 0644);
#define PCM_DEBUG_MSG(msg...)           \
	do {                    \
		if (ingenic_pcm_debug)      \
			printk(KERN_DEBUG"PCM: " msg);  \
	} while(0)

#define PCM_RFIFO_DEPTH 16
#define PCM_TFIFO_DEPTH 16
#define INGENIC_PCM_FORMATS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE)


static const struct of_device_id pcm_dt_match[];

#ifdef CONFIG_AUDIO_DUMP
static void dump_registers(struct device *dev)
{
	struct ingenic_pcm *ingenic_pcm = dev_get_drvdata(dev);
	pr_info("PCMCTL  %p : 0x%08x\n", (ingenic_pcm->vaddr_base + PCMCTL), pcm_read_reg(dev, PCMCTL));
	pr_info("PCMCFG  %p : 0x%08x\n", (ingenic_pcm->vaddr_base + PCMCFG), pcm_read_reg(dev, PCMCFG));
	pr_info("PCMINTC %p : 0x%08x\n", (ingenic_pcm->vaddr_base + PCMINTC), pcm_read_reg(dev, PCMINTC));
	pr_info("PCMINTS %p : 0x%08x\n", (ingenic_pcm->vaddr_base + PCMINTS), pcm_read_reg(dev, PCMINTS));
	pr_info("PCMDIV  %p : 0x%08x\n", (ingenic_pcm->vaddr_base + PCMDIV), pcm_read_reg(dev, PCMDIV));
	return;
}
#endif

static int ingenic_set_sysclk(struct snd_soc_dai *dai, int clk_id,
                              unsigned int freq, int dir)
{
	struct device *dev = dai->dev;
	struct ingenic_pcm *ingenic_pcm = dev_get_drvdata(dai->dev);
	int ret = 0;

	PCM_DEBUG_MSG("enter %s clk_id %d req %d clk dir %d\n", __func__,
	              clk_id, freq, dir);

	if (ingenic_pcm->master_mode) {
		if (freq != ingenic_pcm->rate) {
			ret = clk_set_rate(ingenic_pcm->clk, freq);
			if (ret == -EBUSY) {
				clk_disable_unprepare(ingenic_pcm->clk);
				ret = clk_set_rate(ingenic_pcm->clk, freq);
				clk_prepare_enable(ingenic_pcm->clk);
			}
			ingenic_pcm->rate = clk_get_rate(ingenic_pcm->clk);
		}
	} else {
		dev_warn(dev, "You do not need to configure sysclk in pcm slave mode.\r\n");
	}

	return 0;
}

static int ingenic_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{
	struct device *dev = dai->dev;
	struct ingenic_pcm *ingenic_pcm = dev_get_drvdata(dai->dev);

	PCM_DEBUG_MSG("enter %s div_id %d div %d\n", __func__, div_id, div);

	if (ingenic_pcm->master_mode) {
		if (div_id & INGENIC_PCM_CLKDIV) {
			__pcm_set_clkdiv(dev, div);
		}

		if (div_id & INGENIC_PCM_SYNDIV) {
			__pcm_set_syndiv(dev, div);
		}
	} else {
		dev_warn(dev, "CLKDIV and SYNDIV do not need to be configured in pcm slave mode.\r\n");
	}

	return 0;
}

static int ingenic_pcm_startup(struct snd_pcm_substream *substream,
                               struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct ingenic_pcm *ingenic_pcm = dev_get_drvdata(dai->dev);

	PCM_DEBUG_MSG("enter %s, substream = %s\n",
	              __func__,
	              (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	if (!ingenic_pcm->pcm_mode) {
		if (ingenic_pcm->master_mode) {
			__pcm_as_master(dev);
		} else {
			__pcm_as_slaver(dev);
		}

		__pcm_set_msb_one_shift_in(dev);
		__pcm_set_msb_one_shift_out(dev);
		__pcm_play_lastsample(dev);
		__pcm_enable(dev);
		__pcm_clock_enable(dev);
	}

	if (substream->stream ==
	    SNDRV_PCM_STREAM_PLAYBACK) {
		__pcm_disable_transmit_dma(dev);
		__pcm_disable_replay(dev);
		__pcm_clear_tur(dev);
		ingenic_pcm->pcm_mode |= PCM_WRITE;
	} else {
		__pcm_disable_receive_dma(dev);
		__pcm_disable_record(dev);
		__pcm_clear_ror(dev);
		ingenic_pcm->pcm_mode |= PCM_READ;
	}
	printk("start set PCM register....\n");
	return 0;
}

static int ingenic_pcm_hw_params(struct snd_pcm_substream *substream,
                                 struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct ingenic_pcm *ingenic_pcm = dev_get_drvdata(dai->dev);
	int fmt_width = snd_pcm_format_width(params_format(params));
	enum dma_slave_buswidth buswidth;
	int trigger;

	PCM_DEBUG_MSG("enter %s, substream = %s\n",
	              __func__,
	              (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	if (!((1 << params_format(params)) & INGENIC_PCM_FORMATS) ||
	    params_channels(params) != 1) {
		dev_err(dai->dev, "hw params not inval channel %d params %x\n",
		        params_channels(params), params_format(params));
		return -EINVAL;
	}

	/* format 8 bit or 16 bit*/
	if (fmt_width == 8) {
		buswidth = DMA_SLAVE_BUSWIDTH_1_BYTE;
	} else {
		buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ingenic_pcm->tx_dma_data.addr_width = buswidth;
		ingenic_pcm->tx_dma_data.maxburst = (PCM_TFIFO_DEPTH * buswidth) / 2;
		trigger = PCM_TFIFO_DEPTH - (ingenic_pcm->tx_dma_data.maxburst / (int)buswidth);
		__pcm_set_oss_sample_size(dev, fmt_width == 8 ? 0 : 1);
		__pcm_set_transmit_trigger(dev, trigger);
		snd_soc_dai_set_dma_data(dai, substream, (void *)&ingenic_pcm->tx_dma_data);
	} else {
		ingenic_pcm->rx_dma_data.addr_width = buswidth;
		ingenic_pcm->rx_dma_data.maxburst = (PCM_RFIFO_DEPTH * buswidth) / 2;
		trigger = ingenic_pcm->rx_dma_data.maxburst / (int)buswidth;
		__pcm_set_iss_sample_size(dev, fmt_width == 8 ? 0 : 1);
		__pcm_set_receive_trigger(dev, trigger);
		snd_soc_dai_set_dma_data(dai, substream, (void *)&ingenic_pcm->rx_dma_data);
	}
	return 0;
}

static void ingenic_pcm_start_substream(struct snd_pcm_substream *substream,
                                        struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	PCM_DEBUG_MSG("enter %s, substream = %s\n",
	              __func__,
	              (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		__pcm_enable_transmit_dma(dev);
		__pcm_enable_replay(dev);
	} else {
		__pcm_enable_record(dev);
		__pcm_enable_receive_dma(dev);
	}
	return;
}

static void ingenic_pcm_stop_substream(struct snd_pcm_substream *substream,
                                       struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	int timeout = 150000;
	PCM_DEBUG_MSG("enter %s, substream = %s\n",
	              __func__,
	              (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (__pcm_transmit_dma_is_enable(dev)) {
			__pcm_disable_transmit_dma(dev);
			__pcm_clear_tur(dev);
			/* Hrtimer mode: stop will be happen in any where, make sure there is
			 *  no data transfer on ahb bus before stop dma
			 * Harzard:
			 *  In pcm slave mode, the clk maybe stop before here, we will dead here
			 */
			while ((!__pcm_test_tur(dev)) && (timeout--)) {
				if (timeout == 0) {
					printk("wait tansmit fifo under run error\n");
					return;
				}
			}
		}
		__pcm_disable_replay(dev);
		__pcm_clear_tur(dev);
	} else {
		if (__pcm_receive_dma_is_enable(dev)) {
			__pcm_disable_receive_dma(dev);
			__pcm_clear_ror(dev);
			while (!__pcm_test_ror(dev) && timeout--) {
				if (timeout == 0) {
					printk("wait tansmit fifo under run error\n");
					return;
				}
			}
		}
		__pcm_disable_record(dev);
		__pcm_clear_ror(dev);
	}
	return;
}

static int ingenic_pcm_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	struct ingenic_pcm_runtime_data *prtd = substream->runtime->private_data;
	PCM_DEBUG_MSG("enter %s, substream = %s cmd = %d\n",
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
			printk(KERN_DEBUG"pcm start\n");
			ingenic_pcm_start_substream(substream, dai);
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			if (atomic_read(&prtd->wait_stopdma)) {
				return 0;
			}
			printk(KERN_DEBUG"pcm stop\n");
			ingenic_pcm_stop_substream(substream, dai);
			break;
	}
	return 0;
}

static void ingenic_pcm_shutdown(struct snd_pcm_substream *substream,
                                 struct snd_soc_dai *dai)
{
	struct ingenic_pcm *ingenic_pcm = dev_get_drvdata(dai->dev);
	struct device *dev = dai->dev;

	PCM_DEBUG_MSG("enter %s, substream = %s\n",
	              __func__,
	              (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");
	ingenic_pcm_stop_substream(substream, dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ingenic_pcm->pcm_mode &= ~PCM_WRITE;
	} else {
		ingenic_pcm->pcm_mode &= ~PCM_READ;
	}

	if (!ingenic_pcm->pcm_mode) {
		__pcm_disable(dev);
		__pcm_clock_disable(dev);
	}
	return;
}

static int ingenic_pcm_probe(struct snd_soc_dai *dai)
{
	struct ingenic_pcm *ingenic_pcm = dev_get_drvdata(dai->dev);
	ingenic_pcm->rx_dma_data.fifo_size = PCM_RFIFO_DEPTH;
	ingenic_pcm->tx_dma_data.fifo_size = PCM_TFIFO_DEPTH;
	snd_soc_dai_init_dma_data(dai, &ingenic_pcm->tx_dma_data, &ingenic_pcm->rx_dma_data);
	return 0;
}

static struct snd_soc_dai_ops ingenic_pcm_dai_ops = {
	.probe      = ingenic_pcm_probe,
	.startup    = ingenic_pcm_startup,
	.trigger    = ingenic_pcm_trigger,
	.hw_params  = ingenic_pcm_hw_params,
	.shutdown   = ingenic_pcm_shutdown,
	.set_sysclk = ingenic_set_sysclk,
	.set_clkdiv = ingenic_set_clkdiv,
};

#define ingenic_pcm_suspend NULL
#define ingenic_pcm_resume  NULL

#ifdef CONFIG_AUDIO_DUMP
static ssize_t ingenic_pcm_regs_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	struct ingenic_pcm *ingenic_pcm = dev_get_drvdata(dev);
	dump_registers(ingenic_pcm->dev);
	return 0;
}

static struct device_attribute ingenic_pcm_sysfs_attrs[] = {
	__ATTR(pcm_regs, S_IRUGO, ingenic_pcm_regs_show, NULL),
};
#endif

static struct snd_soc_dai_driver ingenic_pcm_dai = {
	.name     = "ingenic-pcm",
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = INGENIC_PCM_FORMATS,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = INGENIC_PCM_FORMATS,
	},
	.ops = &ingenic_pcm_dai_ops,
};
static const struct snd_soc_component_driver ingenic_pcm_component = {
	.name       = "ingenic-pcm",
};

extern int ingenic_dma_pcm_register(struct device *dev,  const struct snd_dmaengine_pcm_config *config, struct ingenic_dma_pcm *ingenic_pcm);
extern void ingenic_dma_pcm_unregister(struct ingenic_dma_pcm *ingenic_pcm);
static int ingenic_pcm_platfrom_probe(struct platform_device *pdev)
{
	struct ingenic_pcm *ingenic_pcm;
	struct resource *res = NULL;
	const struct of_device_id *match = NULL;
	struct clk *clk = NULL;
	int i = 0, ret = 0;

	ingenic_pcm = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_pcm), GFP_KERNEL);
	if (!ingenic_pcm) {
		return -ENOMEM;
	}

	ingenic_pcm->dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		return -ENOENT;
	}
	if (!devm_request_mem_region(&pdev->dev,
	                             res->start, resource_size(res),
	                             pdev->name)) {
		return -EBUSY;
	}

	ingenic_pcm->res_start = res->start;
	ingenic_pcm->res_size = resource_size(res);
	ingenic_pcm->vaddr_base = devm_ioremap_wc(&pdev->dev,
	                          ingenic_pcm->res_start, ingenic_pcm->res_size);
	if (!ingenic_pcm->vaddr_base) {
		dev_err(&pdev->dev, "Failed to ioremap mmio memory\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "ingenic,pcm-master-mode", &ingenic_pcm->master_mode);
	if (ret) {
		return ret;
	}

	match = of_match_node(pcm_dt_match, pdev->dev.of_node);
	if (!match) {
		return -ENODEV;
	}

	if (!match->data) {
		dev_err(&pdev->dev, "match->data is null!\r\n");
		return -ENODEV;
	};

	ingenic_pcm->priv = (struct ingenic_pcm_priv *)match->data;

	for (i = 0; ingenic_pcm->priv[i].clk_name != NULL; i++) {
		if (ingenic_pcm->priv[i].clk_mode == PCM_CLOCK_GATE) {
			if (ingenic_pcm->priv[i].is_bus_clk) {
				clk = devm_clk_get(ingenic_pcm->dev, ingenic_pcm->priv[i].clk_name);
				if (IS_ERR(clk)) {
					return PTR_ERR(clk);
				}
				clk_prepare_enable(clk);
			}
		}
#ifdef CONFIG_SOC_X1000
		if (ingenic_pcm->priv[i].clk_mode == PCM_CLOCK_MUX) {
			ingenic_pcm->clk = devm_clk_get(&pdev->dev, ingenic_pcm->priv[i].clk_name);
			if (IS_ERR(ingenic_pcm->clk)) {
				return PTR_ERR(ingenic_pcm->clk);
			}
		}
#else
		if (ingenic_pcm->master_mode) {
			if (ingenic_pcm->priv[i].clk_mode == PCM_CLOCK_CE) {
				clk = devm_clk_get(ingenic_pcm->dev, ingenic_pcm->priv[i].clk_name);
				if (IS_ERR(clk)) {
					return PTR_ERR(clk);
				}
				clk_prepare_enable(clk);
			}

			if (ingenic_pcm->priv[i].clk_mode == PCM_CLOCK_MUX) {
				clk = devm_clk_get(&pdev->dev, ingenic_pcm->priv[i].clk_name);
				if (IS_ERR(clk)) {
					return PTR_ERR(clk);
				}
				clk_set_parent(clk_get(NULL, ingenic_pcm->priv[i].clk_name), clk_get(NULL, ingenic_pcm->priv[i].mux_select));
			}

			if (ingenic_pcm->priv[i].clk_mode == PCM_CLOCK_DIV) {
				ingenic_pcm->clk = devm_clk_get(&pdev->dev, ingenic_pcm->priv[i].clk_name);
				if (IS_ERR(ingenic_pcm->clk)) {
					return PTR_ERR(ingenic_pcm->clk);
				}
			}
		}
#endif
	}

	ingenic_pcm->pcm_mode = 0;
	ingenic_pcm->tx_dma_data.addr = (dma_addr_t)ingenic_pcm->res_start + PCMDP;
	ingenic_pcm->rx_dma_data.addr = (dma_addr_t)ingenic_pcm->res_start + PCMDP;
	platform_set_drvdata(pdev, (void *)ingenic_pcm);

#ifdef CONFIG_AUDIO_DUMP
	for (i = 0; i < ARRAY_SIZE(ingenic_pcm_sysfs_attrs); i++) {
		ret = device_create_file(&pdev->dev, &ingenic_pcm_sysfs_attrs[i]);
		if (ret) {
			dev_warn(&pdev->dev, "attribute create failed\n");
		}
	}
#endif

	ret = snd_soc_register_component(&pdev->dev, &ingenic_pcm_component,
	                                 &ingenic_pcm_dai, 1);
	if (ret) {
		goto err_register_cpu_dai;
	}

	ret = ingenic_dma_pcm_register(&pdev->dev, NULL, ingenic_pcm->ingenic_pcm);
	if (ret) {
		goto err_register_platform;
	}

	dev_info(&pdev->dev, "pcm platform probe success\n");

	return 0;

err_register_platform:
	snd_soc_unregister_component(&pdev->dev);

err_register_cpu_dai:
	for (i = 0; ingenic_pcm->priv[i].clk_name != NULL; i++) {
		if (ingenic_pcm->priv[i].clk_mode == PCM_CLOCK_GATE || ingenic_pcm->priv[i].clk_mode == PCM_CLOCK_CE) {
			clk = devm_clk_get(&pdev->dev, ingenic_pcm->priv[i].clk_name);
			if (IS_ERR(clk)) {
				return PTR_ERR(clk);
			}
			clk_disable_unprepare(clk);
		}
	}
	platform_set_drvdata(pdev, NULL);

	return ret;
}

static int ingenic_pcm_platfom_remove(struct platform_device *pdev)
{
	struct ingenic_pcm *ingenic_pcm = platform_get_drvdata(pdev);
	struct clk *clk = NULL;
	int i;
#ifdef CONFIG_AUDIO_DUMP
	for (i = 0; i < ARRAY_SIZE(ingenic_pcm_sysfs_attrs); i++) {
		device_remove_file(&pdev->dev, &ingenic_pcm_sysfs_attrs[i]);
	}
#endif
	snd_soc_unregister_component(&pdev->dev);
	ingenic_dma_pcm_unregister(ingenic_pcm->ingenic_pcm);

	for (i = 0; ingenic_pcm->priv[i].clk_name != NULL; i++) {
		if (ingenic_pcm->priv[i].clk_mode == PCM_CLOCK_GATE || ingenic_pcm->priv[i].clk_mode == PCM_CLOCK_CE) {
			clk = devm_clk_get(&pdev->dev, ingenic_pcm->priv[i].clk_name);
			if (IS_ERR(clk)) {
				return PTR_ERR(clk);
			}
			clk_disable_unprepare(clk);
		}
	}

	platform_set_drvdata(pdev, NULL);

	dev_info(&pdev->dev, "pcm platform remove success\n");

	return 0;
}

static const struct ingenic_pcm_priv x1000_priv_data[] = {
	{ .clk_mode = PCM_CLOCK_GATE, .clk_name = "gate_pcm", .is_bus_clk = true },
	{ .clk_mode = PCM_CLOCK_MUX,  .clk_name = "cgu_pcm" },
};

static const struct ingenic_pcm_priv x2600_priv_data[] = {
	{ .clk_mode = PCM_CLOCK_GATE, .clk_name = "gate_pcm",    .is_bus_clk = true },    // PCM0是总线时钟，访问寄存器的
	{ .clk_mode = PCM_CLOCK_GATE, .clk_name = "gate_pcm1",   .is_bus_clk = true },   // pcm1 是设备时钟
	{ .clk_mode = PCM_CLOCK_CE,   .clk_name = "ce_pcm" },
	{ .clk_mode = PCM_CLOCK_DIV,  .clk_name = "div_pcm_mn" },
	{ .clk_mode = PCM_CLOCK_MUX,  .clk_name = "mux_pcmcs", .mux_select = "epll" },
	{ /* end */ },
};

static const struct of_device_id pcm_dt_match[] = {
	{ .compatible = "ingenic,pcm",       .data = (void *)x1000_priv_data },
	{ .compatible = "ingenic,x2600-pcm", .data = (void *)x2600_priv_data },
	{},
};
MODULE_DEVICE_TABLE(of, pcm_dt_match);
static struct platform_driver ingenic_pcm_plat_driver = {
	.probe  = ingenic_pcm_platfrom_probe,
	.remove = ingenic_pcm_platfom_remove,
	.driver = {
		.name = "ingenic-asoc-pcm",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pcm_dt_match),
	},
};

module_platform_driver(ingenic_pcm_plat_driver);
MODULE_AUTHOR("shicheng.cheng@ingenic.com");
MODULE_DESCRIPTION("INGENIC AIC PCM SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-pcm");
