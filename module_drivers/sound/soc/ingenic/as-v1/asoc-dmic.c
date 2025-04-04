/*
 *  sound/soc/ingenic/asoc-dmic.c
 *  ALSA Soc Audio Layer -- ingenic dmic (part of aic controller) driver
 *
 *  Copyright 2014 Ingenic Semiconductor Co.,Ltd
 *  syzhang <siyu.zhang@ingenic.com>
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
#include <linux/ctype.h>
#include <linux/io.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <sound/tlv.h>
#include <linux/version.h>
#include "asoc-dmic.h"
#include "asoc-aic.h"

static int ingenic_dmic_debug = 0;
module_param(ingenic_dmic_debug, int, 0644);
#define DMIC_DEBUG_MSG(msg...)          \
	do {                    \
		if (ingenic_dmic_debug)     \
			printk(KERN_DEBUG"dmic: " msg); \
	} while(0)

#define DMIC_FIFO_DEPTH 64
#define INGENIC_DMIC_FORMATS (SNDRV_PCM_FMTBIT_S16_LE)
#define INGENIC_DMIC_RATE (SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000|SNDRV_PCM_RATE_48000)

#ifdef CONFIG_AUDIO_DUMP
static void dump_registers(struct device *dev)
{
	struct ingenic_dmic *ingenic_dmic = dev_get_drvdata(dev);
	pr_info("DMICCR0  %p : 0x%08x\n", (ingenic_dmic->vaddr_base + DMICCR0), dmic_read_reg(dev, DMICCR0));
	pr_info("DMICGCR  %p : 0x%08x\n", (ingenic_dmic->vaddr_base + DMICGCR), dmic_read_reg(dev, DMICGCR));
	pr_info("DMICIMR  %p : 0x%08x\n", (ingenic_dmic->vaddr_base + DMICIMR), dmic_read_reg(dev, DMICIMR));
	pr_info("DMICINTCR  %p : 0x%08x\n", (ingenic_dmic->vaddr_base + DMICINTCR), dmic_read_reg(dev, DMICINTCR));
	pr_info("DMICTRICR  %p : 0x%08x\n", (ingenic_dmic->vaddr_base + DMICTRICR), dmic_read_reg(dev, DMICTRICR));
	pr_info("DMICTHRH  %p : 0x%08x\n", (ingenic_dmic->vaddr_base + DMICTHRH), dmic_read_reg(dev, DMICTHRH));
	pr_info("DMICTHRL  %p : 0x%08x\n", (ingenic_dmic->vaddr_base + DMICTHRL), dmic_read_reg(dev, DMICTHRL));
	pr_info("DMICTRIMMAX  %p : 0x%08x\n", (ingenic_dmic->vaddr_base + DMICTRIMMAX), dmic_read_reg(dev, DMICTRIMMAX));
	pr_info("DMICTRINMAX  %p : 0x%08x\n", (ingenic_dmic->vaddr_base + DMICTRINMAX), dmic_read_reg(dev, DMICTRINMAX));
	pr_info("DMICDR  %p : 0x%08x\n", (ingenic_dmic->vaddr_base + DMICDR), dmic_read_reg(dev, DMICDR));
	pr_info("DMICFTHR  %p : 0x%08x\n", (ingenic_dmic->vaddr_base + DMICFTHR), dmic_read_reg(dev, DMICFTHR));
	pr_info("DMICFSR  %p : 0x%08x\n", (ingenic_dmic->vaddr_base + DMICFSR), dmic_read_reg(dev, DMICFSR));
	pr_info("DMICCGDIS  %p : 0x%08x\n", (ingenic_dmic->vaddr_base + DMICCGDIS), dmic_read_reg(dev, DMICCGDIS));
	return;
}
#endif

static int ingenic_dmic_contoller_init(struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	DMIC_DEBUG_MSG("enter %s\n", __func__);

	dmic_write_reg(dev, DMICCR0, 0xC8);
	dmic_write_reg(dev, DMICTRICR, 0x00030000);
	/*gain: 0, ..., e*/
	__dmic_reset(dev);
	while (__dmic_get_reset(dev));
	__dmic_set_sr_8k(dev);
	__dmic_enable_hpf1(dev);
	__dmic_set_gcr(dev, 8);
	__dmic_mask_all_int(dev);
	__dmic_enable_pack(dev);
	__dmic_disable_sw_lr(dev);
	__dmic_enable_lp(dev);
	__dmic_set_request(dev, 48);
	__dmic_set_thr_high(dev, 32);
	__dmic_set_thr_low(dev, 16);

	return 0;
}

static int ingenic_dmic_startup(struct snd_pcm_substream *substream,
                                struct snd_soc_dai *dai)
{
	struct ingenic_dmic *ingenic_dmic = dev_get_drvdata(dai->dev);
	int ret;

	DMIC_DEBUG_MSG("enter %s, substream capture\n", __func__);

	if (!ingenic_dmic->dmic_mode) {
		__dmic_enable(ingenic_dmic->dev);
	}
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		ingenic_dmic->dmic_mode |= DMIC_READ;
	} else {
		dev_err(dai->dev, "dmic is a capture device\n");
		return -EINVAL;
	}

	clk_prepare_enable(ingenic_dmic->clk_gate_dmic);
	if (!IS_ERR(ingenic_dmic->vcc_dmic)) {
		ret = regulator_enable(ingenic_dmic->vcc_dmic);
	}

	ingenic_dmic->en = 1;
	printk("start set dmic register....\n");
	return 0;
}

static int dmic_set_rate(struct device *dev, int rate)
{
	DMIC_DEBUG_MSG("enter %s, rate = %d\n", __func__, rate);
	switch (rate) {
		case 8000:
			__dmic_set_sr_8k(dev);
			break;
		case 16000:
			__dmic_set_sr_16k(dev);
			break;
		case 48000:
			__dmic_set_sr_48k(dev);
			break;
		default:
			dev_err(dev, "dmic unsupport rate %d\n", rate);
	}
	return 0;
}

static int ingenic_dmic_hw_params(struct snd_pcm_substream *substream,
                                  struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	int channels = params_channels(params);
	int rate = params_rate(params);
	struct ingenic_dmic *ingenic_dmic = dev_get_drvdata(dai->dev);
	int fmt_width = snd_pcm_format_width(params_format(params));
	enum dma_slave_buswidth buswidth;
	int trigger;

	DMIC_DEBUG_MSG("enter %s, substream = %s\n", __func__, "capture");

	if (!((1 << params_format(params)) & INGENIC_DMIC_FORMATS)
	    || channels < 1 || channels > 4 || rate > 48000 || rate < 8000
	    || fmt_width != 16) {
		dev_err(dai->dev, "hw params not inval channel %d params %x rate %d fmt_width %d\n",
		        channels, params_format(params), rate, fmt_width);
		return -EINVAL;
	}

	if (ingenic_dmic->unpack_enable) {
		buswidth = DMA_SLAVE_BUSWIDTH_4_BYTES;
	} else {
		buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		ingenic_dmic->capture_dma_data.addr_width = buswidth;
		ingenic_dmic->capture_dma_data.maxburst = (DMIC_FIFO_DEPTH * buswidth) / 2;
		trigger = ingenic_dmic->capture_dma_data.maxburst / (int)buswidth;
		__dmic_set_request(dev, trigger);
		__dmic_set_chnum(dev, channels - 1);
		dmic_set_rate(dev, rate);
		//      snd_soc_dai_set_dma_data(dai, substream, (void *)&ingenic_dmic->capture_dma_data);
	} else {
		dev_err(dai->dev, "DMIC is a capture device\n");
	}
	return 0;
}

static void ingenic_dmic_start_substream(struct snd_pcm_substream *substream,
        struct snd_soc_dai *dai)
{
	struct ingenic_dmic *ingenic_dmic = dev_get_drvdata(dai->dev);
	DMIC_DEBUG_MSG("enter %s, substream start capture\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		//      clk_enable(ingenic_dmic->dmic_enable);
		clk_prepare_enable(ingenic_dmic->clk_gate_dmic);
		__dmic_enable(ingenic_dmic->dev);
		__dmic_enable_rdms(ingenic_dmic->dev);
	} else {
		dev_err(dai->dev, "DMIC is a capture device\n");
	}
	return;
}

static void ingenic_dmic_stop_substream(struct snd_pcm_substream *substream,
                                        struct snd_soc_dai *dai)
{
	struct ingenic_dmic *ingenic_dmic = dev_get_drvdata(dai->dev);
	struct device *dev = dai->dev;
	DMIC_DEBUG_MSG("enter %s, substream stop capture\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (__dmic_is_enable_rdms(dev)) {
			__dmic_disable_rdms(dev);
		}
		//      clk_disable(ingenic_dmic->dmic_enable);
		clk_disable_unprepare(ingenic_dmic->clk_gate_dmic);
		__dmic_disable(ingenic_dmic->dev);
	} else {
		dev_err(dai->dev, "DMIC is a capture device\n");
	}
	return;
}

static int ingenic_dmic_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	struct ingenic_pcm_runtime_data *prtd = substream->runtime->private_data;
	DMIC_DEBUG_MSG("enter %s, substream capture cmd = %d\n", __func__, cmd);
	int ret = 0;
	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			if (atomic_read(&prtd->wait_stopdma)) {
				return -EPIPE;
			}
			printk(KERN_DEBUG"dmic start\n");
			ingenic_dmic_start_substream(substream, dai);
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			ret = dmaengine_terminate_all(prtd->dma_chan);
			if (ret == -EBUSY) {
				atomic_set(&prtd->wait_stopdma, 1);
				return 0;
			}
			printk(KERN_DEBUG"dmic stop\n");
			ingenic_dmic_stop_substream(substream, dai);
			break;
	}
	return 0;
}

static void ingenic_dmic_shutdown(struct snd_pcm_substream *substream,
                                  struct snd_soc_dai *dai)
{
	struct ingenic_dmic *ingenic_dmic = dev_get_drvdata(dai->dev);
	struct device *dev = dai->dev;

	DMIC_DEBUG_MSG("enter %s, substream = capture\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		ingenic_dmic->dmic_mode &= ~DMIC_READ;
	}

	if (!ingenic_dmic->dmic_mode) {
		__dmic_disable(dev);
	}
	if (!IS_ERR(ingenic_dmic->vcc_dmic)) {
		regulator_disable(ingenic_dmic->vcc_dmic);
	}
	clk_disable_unprepare(ingenic_dmic->clk_gate_dmic);
	ingenic_dmic->en = 0;
	return;
}

static int ingenic_dmic_probe(struct snd_soc_dai *dai)
{
	struct ingenic_dmic *ingenic_dmic = dev_get_drvdata(dai->dev);
	snd_soc_dai_init_dma_data(dai, NULL,
	                          &ingenic_dmic->capture_dma_data);

	clk_prepare_enable(ingenic_dmic->clk_gate_dmic);
	ingenic_dmic_contoller_init(dai);
	clk_disable_unprepare(ingenic_dmic->clk_gate_dmic);
	return 0;
}

static struct snd_soc_dai_ops ingenic_dmic_dai_ops = {
	.probe      = ingenic_dmic_probe,
	.startup    = ingenic_dmic_startup,
	.trigger    = ingenic_dmic_trigger,
	.hw_params  = ingenic_dmic_hw_params,
	.shutdown   = ingenic_dmic_shutdown,
};

#define ingenic_dmic_suspend    NULL
#define ingenic_dmic_resume NULL

#ifdef CONFIG_AUDIO_DUMP
static ssize_t ingenic_dmic_regs_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
	struct ingenic_dmic *ingenic_dmic = dev_get_drvdata(dev);
	dump_registers(ingenic_dmic->dev);
	return 0;
}

static ssize_t ingenic_dmic_regs_store(struct device *dev,
                                       struct device_attribute *attr, const char *buf,
                                       size_t count)
{
	struct ingenic_dmic *ingenic_dmic = dev_get_drvdata(dev);
	const char *start = buf;
	unsigned int reg, val;
	int ret_count = 0;
	int ret;

	while (!isxdigit(*start)) {
		start++;
		if (++ret_count >= count) {
			return count;
		}
	}
	reg = simple_strtoul(start, (char **)&start, 16);

	while (!isxdigit(*start)) {
		start++;
		if (++ret_count >= count) {
			return count;
		}
	}
	val = simple_strtoul(start, (char **)&start, 16);

	clk_prepare_enable(ingenic_dmic->clk_gate_dmic);

	if (!IS_ERR(ingenic_dmic->vcc_dmic)) {
		ret = regulator_enable(ingenic_dmic->vcc_dmic);
	}
	dmic_write_reg(ingenic_dmic->dev, reg, val);

	if (!IS_ERR(ingenic_dmic->vcc_dmic)) {
		regulator_disable(ingenic_dmic->vcc_dmic);
	}
	clk_disable_unprepare(ingenic_dmic->clk_gate_dmic);

	return count;

}

static DEVICE_ATTR(dmic_regs, S_IRUGO | S_IWUSR, ingenic_dmic_regs_show, ingenic_dmic_regs_store);
static const struct attribute *dump_attrs[] = {
	&dev_attr_dmic_regs.attr,
	NULL,
};
static const struct attribute_group dump_attr_group = {
	.attrs = (struct attribute **)dump_attrs,
};
#endif

static struct snd_soc_dai_driver ingenic_dmic_dai = {
	.name    = "dmic",
	.capture = {
		.channels_min = 1,
		.channels_max = 4,
		.rates = INGENIC_DMIC_RATE,
		.formats = INGENIC_DMIC_FORMATS,
	},
	.ops = &ingenic_dmic_dai_ops,
};

static const struct snd_soc_component_driver ingenic_dmic_component = {
	.name       = "ingenic-dmic",
};

static int dmic_gain_get(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct ingenic_dmic *ingenic_dmic = snd_soc_component_get_drvdata(codec);
	unsigned int value;
	clk_prepare_enable(ingenic_dmic->clk_gate_dmic);
	value = dmic_read_reg(ingenic_dmic->dev, DMICGCR);
	clk_disable_unprepare(ingenic_dmic->clk_gate_dmic);
	ucontrol->value.integer.value[0] = value;

	return 0;
}
static int dmic_gain_put(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct ingenic_dmic *ingenic_dmic = snd_soc_component_get_drvdata(codec);
	unsigned int value = ucontrol->value.integer.value[0];

	clk_prepare_enable(ingenic_dmic->clk_gate_dmic);
	dmic_write_reg(ingenic_dmic->dev, DMICGCR, value);
	clk_disable_unprepare(ingenic_dmic->clk_gate_dmic);

	return 0;
}

/*
 * DMIC GAIN volume control:
 * from 0dB to 93dB in 3 dB steps
 * */
static const DECLARE_TLV_DB_SCALE(dmic_gain_tlv, 0, 300, 0);
static const struct snd_kcontrol_new  dmic_codec_controls[] = {
	SOC_SINGLE_EXT_TLV("DMIC GAIN", 0, 0, 0x1f, 0, dmic_gain_get, dmic_gain_put, dmic_gain_tlv),
	SOC_SINGLE_EXT_TLV("DMIC Capture Volume", 0, 0, 0x1f, 0, dmic_gain_get, dmic_gain_put, dmic_gain_tlv),
};

static struct snd_soc_dai_driver  dmic_codec_dai = {
	.name = "dmic-codec-hifi",
	.capture = {
		.channels_min = 1,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static struct snd_soc_component_driver soc_codec_dev_dmic_component = {
	.controls =     dmic_codec_controls,
	.num_controls = ARRAY_SIZE(dmic_codec_controls),
};

extern int ingenic_dma_pcm_register(struct device *dev,  const struct snd_dmaengine_pcm_config *config, struct ingenic_dma_pcm *ingenic_pcm);
extern void ingenic_dma_pcm_unregister(struct ingenic_dma_pcm *ingenic_pcm);
static int ingenic_dmic_platfrom_probe(struct platform_device *pdev)
{
	struct ingenic_dmic *ingenic_dmic;
	struct resource *res = NULL;
	int ret;

	ingenic_dmic = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_dmic), GFP_KERNEL);
	if (!ingenic_dmic) {
		return -ENOMEM;
	}

	ingenic_dmic->dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		return -ENOENT;
	}
	if (!devm_request_mem_region(&pdev->dev,
	                             res->start, resource_size(res), pdev->name)) {
		return -EBUSY;
	}

	ingenic_dmic->res_start = res->start;
	ingenic_dmic->res_size = resource_size(res);
	ingenic_dmic->vaddr_base = devm_ioremap(&pdev->dev,
	                                        ingenic_dmic->res_start, ingenic_dmic->res_size);
	if (!ingenic_dmic->vaddr_base) {
		dev_err(&pdev->dev, "Failed to ioremap mmio memory\n");
		return -ENOMEM;
	}

	ingenic_dmic->dmic_mode = 0;
	ingenic_dmic->capture_dma_data.addr = (dma_addr_t)ingenic_dmic->res_start + DMICDR;

	ingenic_dmic->vcc_dmic = regulator_get(&pdev->dev, "vcc_dmic");

	platform_set_drvdata(pdev, (void *)ingenic_dmic);

#ifdef CONFIG_AUDIO_DUMP
	ret = sysfs_create_group(&pdev->dev.kobj, &dump_attr_group);
	if (ret) {
		dev_info(&pdev->dev, "dmic attr create failed\n");
	}
#endif

	ingenic_dmic->clk_gate_dmic = clk_get(&pdev->dev, "gate_dmic");
	if (IS_ERR_OR_NULL(ingenic_dmic->clk_gate_dmic)) {
		ret = PTR_ERR(ingenic_dmic->clk_gate_dmic);
		ingenic_dmic->clk_gate_dmic = NULL;
		dev_err(&pdev->dev, "Failed to get clock: %d\n", ret);
		return ret;
	}
	//  clk_prepare_enable(ingenic_dmic->clk_gate_dmic);
	//  __dmic_enable(&pdev->dev);
	//  printk("XXXXXXXXXXXXXXXXXXXXcgu-i2s%x\n",*(volatile unsigned int*)0xb0000060);
	//  printk("XXXXXXXXXXXXXXXXXXXXgate-dmic%x\n",*(volatile unsigned int*)0xb0000020);
	//  ingenic_dmic->dmic_enable = clk_get(&pdev->dev, "dmic_enable");
	//  if (IS_ERR_OR_NULL(ingenic_dmic->dmic_enable)) {
	//      ret = PTR_ERR(ingenic_dmic->dmic_enable);
	//      ingenic_dmic->dmic_enable = NULL;
	//      dev_err(&pdev->dev, "Failed to get clock: %d\n", ret);
	//      return ret;
	//  }

	ret = snd_soc_register_component(&pdev->dev,
	                                 &soc_codec_dev_dmic_component, &dmic_codec_dai, 1);
	ret = snd_soc_register_component(&pdev->dev, &ingenic_dmic_component,
	                                 &ingenic_dmic_dai, 1);
	if (ret) {
		goto err_register_cpu_dai;
	}
	dev_dbg(&pdev->dev, "dmic platform probe success\n");

	ret = ingenic_dma_pcm_register(&pdev->dev, NULL, ingenic_dmic->ingenic_pcm);

	return ret;

err_register_cpu_dai:
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int ingenic_dmic_platfom_remove(struct platform_device *pdev)
{
	struct ingenic_dmic *ingenic_dmic = platform_get_drvdata(pdev);
#ifdef CONFIG_AUDIO_DUMP
	sysfs_remove_group(&pdev->dev.kobj, &dump_attr_group);
#endif
	snd_soc_unregister_component(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	ingenic_dma_pcm_unregister(ingenic_dmic->ingenic_pcm);
	return 0;
}

#ifdef CONFIG_PM
static int ingenic_dmic_platfom_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ingenic_dmic *ingenic_dmic = platform_get_drvdata(pdev);
	if (ingenic_dmic->en) {
		if (!IS_ERR(ingenic_dmic->vcc_dmic)) {
			regulator_disable(ingenic_dmic->vcc_dmic);
		}

		clk_disable_unprepare(ingenic_dmic->clk_gate_dmic);
	}
	return 0;
}

static int ingenic_dmic_platfom_resume(struct platform_device *pdev)
{
	struct ingenic_dmic *ingenic_dmic = platform_get_drvdata(pdev);
	int ret;
	if (ingenic_dmic->en) {
		if (!IS_ERR(ingenic_dmic->vcc_dmic)) {
			ret = regulator_enable(ingenic_dmic->vcc_dmic);
		}

		clk_prepare_enable(ingenic_dmic->clk_gate_dmic);
	}
	return 0;
}
#endif

static const struct of_device_id dmic_dt_match[] = {
	{ .compatible = "ingenic,dmic", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, dmic_dt_match);
static struct platform_driver ingenic_dmic_plat_driver = {
	.probe  = ingenic_dmic_platfrom_probe,
	.remove = ingenic_dmic_platfom_remove,
#ifdef CONFIG_PM
	.suspend = ingenic_dmic_platfom_suspend,
	.resume = ingenic_dmic_platfom_resume,
#endif
	.driver = {
		.name = "ingenic-asoc-dmic",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(dmic_dt_match),
	},
};

module_platform_driver(ingenic_dmic_plat_driver);
MODULE_AUTHOR("siyu.zhang@ingenic.com");
MODULE_DESCRIPTION("INGENIC AIC dmic SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-dmic");
