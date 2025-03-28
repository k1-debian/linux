/*
 * ALSA Soc Audio Layer -- ingenic as(audio system) dmic(Basic Audio Inter-
 * face Controller) driver
 *
 * Copyright 2017 - 2022 Ingenic Semiconductor Co.,Ltd
 *   wqshao <wangquan.shao@ingenic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
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
#include <linux/types.h>
#include <linux/clk.h>
#include <sound/tlv.h>
#include <sound/control.h>
#include "as-dmic.h"

#ifdef DEBUG
	static int ingenic_dmic_debug = 1;
#else
	static int ingenic_dmic_debug = 0;
#endif
module_param(ingenic_dmic_debug, int, 0644);
#define DMIC_DEBUG_MSG(msg...)                  \
	do {                                        \
		if (ingenic_dmic_debug)                 \
			printk(KERN_DEBUG"DMIC: " msg);     \
	} while(0)

static int ingenic_dmic_reset(struct ingenic_dmic *dmic)
{
	unsigned int val, time_out = 0xfff;

	regmap_update_bits(dmic->regmap, DMIC_CR0, DMIC_RESET, DMIC_RESET);
	do {
		regmap_read(dmic->regmap, DMIC_CR0, &val);
	} while ((val & DMIC_RESET) && (--time_out));
	if (!time_out) {
		dev_err(dmic->dev, "DMIC reset fail ...\n");
		return -EIO;
	}
	return 0;
}

static int ingenic_dmic_probe(struct snd_soc_dai *dai)
{
	struct ingenic_dmic *dmic = dev_get_drvdata(dai->dev);

	regmap_update_bits(dmic->regmap, DMIC_CR0, HPF2_EN | LPF_EN | HPF1_EN,
	                   HPF2_EN | LPF_EN | HPF1_EN);
	/*gain: 0, ..., 1F*/
	regmap_write(dmic->regmap, DMIC_GCR, DMIC_SET_GAIN(4));

	return 0;
}

static int ingenic_dmic_startup(struct snd_pcm_substream *substream,
                                struct snd_soc_dai *dai)
{
	struct ingenic_dmic *dmic = dev_get_drvdata(dai->dev);

	DMIC_DEBUG_MSG("enter %s\n", __func__);

	return ingenic_dmic_reset(dmic);
}

static int ingenic_dmic_trigger(struct snd_pcm_substream *substream,
                                int cmd, struct snd_soc_dai *dai)
{
	struct ingenic_dmic *dmic = dev_get_drvdata(dai->dev);

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			DMIC_DEBUG_MSG("start\n");
			regmap_update_bits(dmic->regmap, DMIC_CR0, DMIC_EN, DMIC_EN);
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			DMIC_DEBUG_MSG("stop\n");
			regmap_update_bits(dmic->regmap, DMIC_CR0, DMIC_EN, 0);
			return ingenic_dmic_reset(dmic);
			break;
	}
	return 0;
}

static int ingenic_dmic_params_check(int channels, int rate, int fmt_width)
{
	if (channels < 1 && channels > 12) {
		goto error;
	}

	if ((rate != 8000) && (rate != 16000)
	    && (rate != 48000) && (rate != 96000)) {
		goto error;
	}

	if ((fmt_width != 16) && (fmt_width != 24)) {
		goto error;
	}

	return true;
error:
	pr_err("%s %d, parameter is illegal!\n", __func__, __LINE__);
	return false;
}

static int ingenic_dmic_hw_params(struct snd_pcm_substream *substream,
                                  struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	int channels = params_channels(params);
	int rate = params_rate(params);
	int fmt_width = snd_pcm_format_width(params_format(params));
	struct ingenic_dmic *dmic = dev_get_drvdata(dai->dev);
	u32 dmic_cr = 0;

	DMIC_DEBUG_MSG("enter %s, substream = %s chl=%d, rate=%d, fmt_width=%d\n", __func__,
	               "capture", channels, rate, fmt_width);

	if (!ingenic_dmic_params_check(channels, rate, fmt_width)) {
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		dmic_cr = DMIC_SET_CHL(channels);
		dmic_cr |= DMIC_SET_SR(rate);
		dmic_cr |= DMIC_SET_OSS(fmt_width);
		regmap_update_bits(dmic->regmap, DMIC_CR0, CHNUM_MASK | SR_MASK | OSS_MASK, dmic_cr);
	} else {
		dev_err(dmic->dev, "DMIC is a capture device\n");
		return -ENXIO;
	}

	return 0;
}

static struct snd_soc_dai_ops ingenic_dmic_dai_ops = {
	.probe      = ingenic_dmic_probe,
	.startup    = ingenic_dmic_startup,
	.trigger    = ingenic_dmic_trigger,
	.hw_params  = ingenic_dmic_hw_params,
};

/*
 * GAIN volume control:
 * from 0 to 93 dB in 3 dB steps (0dB means no gain is used)
 * */
static const DECLARE_TLV_DB_SCALE(dmic_tlv, 0, 300, 0);

static const struct snd_kcontrol_new ingenic_dmic_controls[] = {
	SOC_SINGLE_TLV("DMIC Capture Volume", DMIC_GCR, DGAIN_SFT, 31, 0, dmic_tlv),

	SOC_SINGLE("DMIC High Pass Filter1 Switch", DMIC_CR0, HPF1_EN_SFT, 1, 0),
	SOC_SINGLE("DMIC High Pass Filter2 Switch", DMIC_CR0, HPF2_EN_SFT, 1, 0),
	SOC_SINGLE("DMIC Low Pass Filter Switch", DMIC_CR0, LPF_EN_SFT, 1, 0),

	SOC_SINGLE("DMIC SW_LR Switch", DMIC_CR0, SW_LR_SFT, 1, 0),
};

struct snd_soc_component_driver ingenic_dmic_component = {
	.name           = "as_dmic",
	.controls       = ingenic_dmic_controls,
	.num_controls   = ARRAY_SIZE(ingenic_dmic_controls),
};

static int ingenic_dmic_clk_init(struct platform_device *pdev,
                                 struct ingenic_dmic *dmic)
{
	const char *name;

	of_property_read_string(dmic->dev->of_node, "ingenic,clk-name", &name);
	dmic->clk = devm_clk_get(dmic->dev, name);
	if (IS_ERR_OR_NULL(dmic->clk)) {
		dev_warn(dmic->dev, "Warning ... Failed to get %s!\n", name);
	}

	/* Setup dmic's Parent clk, fix to i2s2 */
	if (dmic->clk) {
		struct clk *parent;
		of_property_read_string(dmic->dev->of_node, "ingenic,clk-parent", &name);
		parent =  devm_clk_get(dmic->dev, name);
		if (IS_ERR_OR_NULL(parent)) {
			dev_warn(dmic->dev, "Warning ... Failed to get %s!\n", name);
		}
		clk_set_parent(dmic->clk, parent);
		devm_clk_put(dmic->dev, parent);
	}

	dmic->clk_gate = devm_clk_get(dmic->dev, "gate_dmic");
	if (IS_ERR_OR_NULL(dmic->clk_gate)) {
		dev_warn(dmic->dev, "Warning ... Failed to get gate_dmic!\n");
	}

	if (dmic->clk) {
		clk_set_rate(dmic->clk, 24000000);
		clk_prepare_enable(dmic->clk);
	}
	if (dmic->clk_gate) {
		clk_prepare_enable(dmic->clk_gate);
	}

	return 0;
}

static int ingenic_dmic_platform_probe(struct platform_device *pdev)
{
	struct ingenic_dmic *dmic;
	struct resource *res;
	int ret;
	struct regmap_config regmap_config = {
		.reg_bits = 32,
		.reg_stride = 4,
		.val_bits = 32,
		.cache_type = REGCACHE_NONE,
	};

	dmic = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_dmic) +
	                    sizeof(struct snd_soc_dai_driver), GFP_KERNEL);
	if (!dmic) {
		return -ENOMEM;
	}

	dmic->dai_driver = (struct snd_soc_dai_driver *)(dmic + 1);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dmic->io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dmic->io_base)) {
		return PTR_ERR(dmic->io_base);
	}
	regmap_config.max_register = resource_size(res) - 0x4;
	dmic->regmap = devm_regmap_init_mmio(&pdev->dev,
	                                     dmic->io_base,
	                                     &regmap_config);
	if (IS_ERR(dmic->regmap)) {
		return PTR_ERR(dmic->regmap);
	}
	dmic->dev = &pdev->dev;

	dmic->dai_driver->id = 1;
	dmic->dai_driver->name = "DMIC";
	dmic->dai_driver->ops = &ingenic_dmic_dai_ops;
	dmic->dai_driver->capture.stream_name = "DMIC capture";
	dmic->dai_driver->capture.channels_min = 1;
	dmic->dai_driver->capture.channels_max = 8;

	ingenic_dmic_clk_init(pdev, dmic);
	platform_set_drvdata(pdev, dmic);

	ret = devm_snd_soc_register_component(&pdev->dev, &ingenic_dmic_component,
	                                      dmic->dai_driver, 1);
	if (!ret) {
		dev_info(&pdev->dev, "dmic platform probe success\n");
	}

	return ret;
}

static int ingenic_dmic_platform_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id ingenic_dmic_match_table[] = {
	{ .compatible = "ingenic,as-dmic", },
	{ }
};
MODULE_DEVICE_TABLE(of, ingenic_dmic_match_table);

#ifdef CONFIG_PM
static int ingenic_dmic_runtime_suspend(struct device *dev)
{
	return 0;
}

static int ingenic_dmic_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops ingenic_dmic_pm_ops = {
	SET_RUNTIME_PM_OPS(ingenic_dmic_runtime_suspend,
	                   ingenic_dmic_runtime_resume, NULL)
};

static struct platform_driver ingenic_dmic_platform_driver = {
	.driver = {
		.name = "as-dmic",
		.of_match_table = ingenic_dmic_match_table,
		.pm = &ingenic_dmic_pm_ops,
	},
	.probe = ingenic_dmic_platform_probe,
	.remove = ingenic_dmic_platform_remove,
};
module_platform_driver(ingenic_dmic_platform_driver);

MODULE_AUTHOR("wqshao <wangquan.shao@ingenic.com>");
MODULE_DESCRIPTION("Ingenic AS dmic SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-as-dmic");
