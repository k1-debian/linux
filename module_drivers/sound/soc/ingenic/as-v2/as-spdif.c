/*
 * ALSA Soc Audio Layer -- ingenic as(audio system) spdif(Basic Audio Inter-
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

#include "as-spdif.h"

#ifdef DEBUG
	static int ingenic_spdif_debug = 1;
#else
	static int ingenic_spdif_debug = 0;
#endif
module_param(ingenic_spdif_debug, int, 0644);
#define SPDIF_DEBUG_MSG(msg...)                 \
	do {                                        \
		if (ingenic_spdif_debug)                \
			printk("SPDIF: " msg);              \
	} while(0)

int __attribute__((weak)) ingenic_spdif_fmtcov_be_fix(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params)
{
	return 0;
}

static int ingenic_spdif_startup(struct snd_pcm_substream *substream,
                                 struct snd_soc_dai *dai)
{
	struct ingenic_spdif *spdif = dev_get_drvdata(dai->dev);
	u32 ch_cfg;

	SPDIF_DEBUG_MSG("enter %s, substream = %s\n", __func__,
	                (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
	                "playback" : "capture");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_update_bits(spdif->spdif_out_regmap, SPCFG2,
		                   SPO_AUDIO_N | SPO_CON_PRO, spdif->non_pcm ? SPO_AUDIO_N : 0);
		if (!spdif->non_pcm) {
			ch_cfg = SPO_SET_SCR_NUM(1) | SPO_SET_CH1_NUM(1) | SPO_SET_CH2_NUM(1);
			regmap_update_bits(spdif->spdif_out_regmap, SPCFG1,
			                   SPO_SCR_NUM_MASK | SPO_CH1_NUM_MASK | SPO_CH2_NUM_MASK, ch_cfg);
			regmap_update_bits(spdif->spdif_out_regmap, SPCFG2,
			                   SPO_CAT_CODE_MASK | SPO_CH_MD_MASK, SPO_CAT_CODE_GEN | SPO_CH_MD_0);
		}
		regmap_update_bits(spdif->spdif_out_regmap, SPCTRL, SPO_INVALID, 0);
	}

	return 0;
}

static int ingenic_spdif_start_substream(struct snd_pcm_substream *substream,
        struct ingenic_spdif *spdif)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_write(spdif->spdif_out_regmap, SPENA, SPO_SPEN);
	} else {
		regmap_update_bits(spdif->spdif_in_regmap, SPIENA, SPI_SPIEN, SPI_SPIEN);
		ndelay(200);
		regmap_update_bits(spdif->spdif_in_regmap, SPIENA, SPI_RESET, SPI_RESET);
	}

	return 0;
}

static int ingenic_spdif_stop_substream(struct snd_pcm_substream *substream,
                                        struct ingenic_spdif *spdif)
{
	u32 val, timeout = 0xfff;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_write(spdif->spdif_out_regmap, SPENA, 0);
		do {
			regmap_read(spdif->spdif_out_regmap, SPENA, &val);
		} while ((val & SPO_SPEN) && --timeout);
		if (!timeout) {
			goto error;
		}
	} else {
		regmap_update_bits(spdif->spdif_in_regmap, SPIENA, SPI_RESET, SPI_RESET);
		ndelay(200);
		regmap_update_bits(spdif->spdif_in_regmap, SPIENA, SPI_SPIEN, 0);
	}

	return 0;
error:
	pr_err("[ERROR]: %s,%d disable fail ...\n", __func__, __LINE__);
	return -EIO;
}

static int ingenic_spdif_trigger(struct snd_pcm_substream *substream,
                                 int cmd, struct snd_soc_dai *dai)
{
	struct ingenic_spdif *spdif = dev_get_drvdata(dai->dev);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		SPDIF_DEBUG_MSG("start\n");
		ingenic_spdif_start_substream(substream, spdif);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		SPDIF_DEBUG_MSG("stop\n");
		ingenic_spdif_stop_substream(substream, spdif);
		break;
	}

	return 0;
}

static int ingenic_spdif_div(struct ingenic_spdif *spdif, int stream, unsigned long rate, int channels)
{
	int clk_ratio = stream ? 1280 : 32 * 2 * channels;  //fixed 2channels
	int clk, div, sysclk = spdif->sysclk;

	clk = rate * clk_ratio;
	div = ((sysclk + clk - 1) / clk) & (~0x1UL);

	return div;
}

struct sampl_freq {
	u32 freq;
#define SPDIF_OUT_FS        0
#define SPDIF_OUT_ORG_FRQ   1
	int flag;
	u32 val;
} sample[] = {
	/* References : IEC 60958-3 configure */
	{22050,  SPDIF_OUT_FS, SPO_SET_FS(0x2)},
	{24000,  SPDIF_OUT_FS, SPO_SET_FS(0x6)},
	{32000,  SPDIF_OUT_FS, SPO_SET_FS(0xc)},
	{44100,  SPDIF_OUT_FS, SPO_SET_FS(0x0)},
	{48000,  SPDIF_OUT_FS, SPO_SET_FS(0x4)},
	{88200,  SPDIF_OUT_FS, SPO_SET_FS(0x1)},
	{96000,  SPDIF_OUT_FS, SPO_SET_FS(0x5)},
	{192000, SPDIF_OUT_FS, SPO_SET_FS(0x7)},
	{176400, SPDIF_OUT_FS, SPO_SET_FS(0x3)},
	{768000, SPDIF_OUT_FS, SPO_SET_FS(0x9)},

	{8000,   SPDIF_OUT_ORG_FRQ, SPO_SET_ORG_FRQ(0x6)},
	{11025,  SPDIF_OUT_ORG_FRQ, SPO_SET_ORG_FRQ(0x5)},
	{12000,  SPDIF_OUT_ORG_FRQ, SPO_SET_ORG_FRQ(0x4)},
	{16000,  SPDIF_OUT_ORG_FRQ, SPO_SET_ORG_FRQ(0x1)},
	{22050,  SPDIF_OUT_ORG_FRQ, SPO_SET_ORG_FRQ(0xb)},
	{24000,  SPDIF_OUT_ORG_FRQ, SPO_SET_ORG_FRQ(0x9)},
	{32000,  SPDIF_OUT_ORG_FRQ, SPO_SET_ORG_FRQ(0x3)},
	{44100,  SPDIF_OUT_ORG_FRQ, SPO_SET_ORG_FRQ(0xf)},
	{48000,  SPDIF_OUT_ORG_FRQ, SPO_SET_ORG_FRQ(0xd)},
	{88200,  SPDIF_OUT_ORG_FRQ, SPO_SET_ORG_FRQ(0xe)},
	{96000,  SPDIF_OUT_ORG_FRQ, SPO_SET_ORG_FRQ(0xa)},
	{176400, SPDIF_OUT_ORG_FRQ, SPO_SET_ORG_FRQ(0xc)},
	{192000, SPDIF_OUT_ORG_FRQ, SPO_SET_ORG_FRQ(0x8)},
};

static int ingenic_spdif_out_set_rate(struct ingenic_spdif *spdif, unsigned long sample_rate)
{
	int i;
	u32 fs = SPO_SET_FS(0x8), orq_frq = 0;

	for (i = 0; i < ARRAY_SIZE(sample); i++) {
		if (sample[i].freq == sample_rate) {
			if (sample[i].flag == SPDIF_OUT_FS) {
				fs = sample[i].val;
			} else {
				orq_frq = sample[i].val;
			}
		}
	}
	return fs | orq_frq;
}

static bool ingenic_spdif_params_check(int channels, int fmt_width)
{
	if (channels != 2) {
		goto error;
	}
	if (fmt_width < 16 || fmt_width > 24) {
		goto error;
	}

	return true;
error:
	pr_err("[ERROR]: %s %d\n", __func__, __LINE__);
	return false;
}

static int ingenic_spdif_hw_params(struct snd_pcm_substream *substream,
                                   struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	int channels = params_channels(params);
	int rate = params_rate(params);
	int fmt_width = snd_pcm_format_width(params_format(params));
	struct ingenic_spdif *spdif = dev_get_drvdata(dai->dev);
	u32 max_wl = 0, div = 0, sampl_wl = 0, rate_reg = 0;

	SPDIF_DEBUG_MSG("enter %s, substream = %s, channels %d, rate %d, fmt_width %d\n", __func__,
	                (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture",
	                channels, rate, fmt_width);

	if (!ingenic_spdif_params_check(channels, fmt_width)) {
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (!spdif->non_pcm) {
			max_wl = (1 == fmt_width / 10) ? 0 : 1;
			sampl_wl = SPO_SET_SAMPL_WL(fmt_width, max_wl);
			rate_reg = ingenic_spdif_out_set_rate(spdif, rate);
			regmap_update_bits(spdif->spdif_out_regmap, SPCFG2,
			                   SPO_FS_MASK | SPO_ORG_FRQ_MASK | SPO_SAMPL_WL_MASK | SPO_MAX_WL,
			                   rate_reg | sampl_wl | (max_wl ? SPO_MAX_WL : 0));
		}
		regmap_update_bits(spdif->spdif_out_regmap, SPCTRL, SPO_SIGN_N,
		                   snd_pcm_format_signed(params_format(params)) ? 0 : 1);
		div = ingenic_spdif_div(spdif, substream->stream, rate, channels);
		if (spdif->out_clk_div) {
			div = spdif->out_clk_div;
		}
		SPDIF_DEBUG_MSG("SPDIF OUT's clk div %d\n", div);
		regmap_write(spdif->spdif_out_regmap, SPDIV, SPO_SET_DV(div));
	} else {
		div = ingenic_spdif_div(spdif, substream->stream, rate, channels);
		if (spdif->in_clk_div) {
			div = spdif->in_clk_div;
		}
		SPDIF_DEBUG_MSG("SPDIF IN's clk div %d\n", div);
		regmap_write(spdif->spdif_in_regmap, SPIDIV, SPI_SET_DV(div));
	}

	return ingenic_spdif_fmtcov_be_fix(substream, params);
}

static int ingenic_spdif_set_sysclk(struct snd_soc_dai *dai, int clk_id,
                                    unsigned int freq, int dir)
{
	struct ingenic_spdif *spdif = dev_get_drvdata(dai->dev);

	SPDIF_DEBUG_MSG("enter %s, sysclk freq=%d\n", __func__, freq);
	clk_set_rate(spdif->clk, freq);
	spdif->sysclk = freq;

	return 0;
}

static int ingenic_spdif_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{
	struct ingenic_spdif *spdif = dev_get_drvdata(dai->dev);

	switch (div_id) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		spdif->out_clk_div = div;
		break;
	case SNDRV_PCM_STREAM_CAPTURE:
		spdif->in_clk_div = div;
		break;
	default:
		pr_err("%s:%d\n", __func__, __LINE__);
		return -EINVAL;
	}

	return 0;
}

static struct snd_soc_dai_ops ingenic_spdif_dai_ops = {
	.startup    = ingenic_spdif_startup,
	.trigger    = ingenic_spdif_trigger,
	.hw_params  = ingenic_spdif_hw_params,
	.set_sysclk = ingenic_spdif_set_sysclk,
	.set_clkdiv = ingenic_spdif_set_clkdiv,
};

struct snd_soc_component_driver ingenic_spdif_component = {
	.name = "spdif",
};

/* TODO: replace by dts. */
static int ingenic_spdif_clk_init(struct platform_device *pdev, struct ingenic_spdif *spdif)
{
	const char *name;

	of_property_read_string(spdif->dev->of_node, "ingenic,clk-name", &name);
	spdif->clk = devm_clk_get(spdif->dev, name);
	if (IS_ERR_OR_NULL(spdif->clk)) {
		dev_warn(spdif->dev, "Warning ... Failed to get %s\n", name);
	}

	if (spdif->clk) {
		struct clk *parent;
		of_property_read_string(spdif->dev->of_node, "ingenic,clk-parent", &name);
		parent = devm_clk_get(spdif->dev, name);
		if (IS_ERR_OR_NULL(parent)) {
			dev_warn(spdif->dev, "Warning ... Failed to get %s\n", name);
		}
		clk_set_parent(spdif->clk, parent);
		devm_clk_put(spdif->dev, parent);
	}

	spdif->clk_gate = devm_clk_get(spdif->dev, "gate_spdif");
	if (IS_ERR_OR_NULL(spdif->clk_gate)) {
		dev_warn(spdif->dev, "Warning ... Failed to get gate_spdif\n");
	}

	if (spdif->clk) {
		clk_set_rate(spdif->clk, 24000000);
		spdif->sysclk = 24000000;
		clk_prepare_enable(spdif->clk);
	}
	if (spdif->clk_gate) {
		clk_prepare_enable(spdif->clk_gate);
	}

	return 0;
}

static int ingenic_spdif_platform_probe(struct platform_device *pdev)
{
	struct ingenic_spdif *spdif;
	struct resource *res;
	int ret;
	struct regmap_config regmap_config = {
		.reg_bits = 32,
		.reg_stride = 4,
		.val_bits = 32,
		.cache_type = REGCACHE_NONE,
	};

	spdif = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_spdif) +
	                     sizeof(struct snd_soc_dai_driver), GFP_KERNEL);
	if (!spdif) {
		return -ENOMEM;
	}

	spdif->dai_driver = (struct snd_soc_dai_driver *)(spdif + 1);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "out");
	spdif->spdif_out_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(spdif->spdif_out_base)) {
		return PTR_ERR(spdif->spdif_out_base);
	}
	regmap_config.max_register = resource_size(res) - 0x4;
	regmap_config.name = res->name;
	spdif->spdif_out_regmap = devm_regmap_init_mmio(&pdev->dev,
	                          spdif->spdif_out_base,
	                          &regmap_config);
	if (IS_ERR(spdif->spdif_out_regmap)) {
		return PTR_ERR(spdif->spdif_out_regmap);
	}
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "in");
	spdif->spdif_in_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(spdif->spdif_in_base)) {
		return PTR_ERR(spdif->spdif_in_base);
	}
	regmap_config.max_register = resource_size(res) - 0x4;
	regmap_config.name = res->name;
	spdif->spdif_in_regmap = devm_regmap_init_mmio(&pdev->dev,
	                         spdif->spdif_in_base,
	                         &regmap_config);
	if (IS_ERR(spdif->spdif_in_regmap)) {
		return PTR_ERR(spdif->spdif_in_regmap);
	}

	spdif->non_pcm = of_property_read_bool(pdev->dev.of_node, "ingenic,non-pcm");

	spdif->dev = &pdev->dev;

	spdif->dai_driver->id = 1;
	spdif->dai_driver->name = "SPDIF";
	spdif->dai_driver->ops = &ingenic_spdif_dai_ops;
	spdif->dai_driver->playback.stream_name = "SPDIF playback";
	spdif->dai_driver->playback.channels_min = 1;
	spdif->dai_driver->capture.stream_name = "SPDIF capture";
	spdif->dai_driver->capture.channels_min = 1;

	ingenic_spdif_clk_init(pdev, spdif);

	platform_set_drvdata(pdev, spdif);

	ret = devm_snd_soc_register_component(&pdev->dev, &ingenic_spdif_component,
	                                      spdif->dai_driver, 1);
	if (!ret) {
		dev_info(&pdev->dev, "spdif platform probe success\n");
	}

	return ret;
}

static int ingenic_spdif_platform_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id ingenic_spdif_match_table[] = {
	{ .compatible = "ingenic,as-spdif", },
	{ }
};
MODULE_DEVICE_TABLE(of, ingenic_spdif_match_table);

#ifdef CONFIG_PM
static int ingenic_spdif_runtime_suspend(struct device *dev)
{
	return 0;
}

static int ingenic_spdif_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops ingenic_spdif_pm_ops = {
	SET_RUNTIME_PM_OPS(ingenic_spdif_runtime_suspend,
	                   ingenic_spdif_runtime_resume, NULL)
};

static struct platform_driver ingenic_spdif_platform_driver = {
	.driver = {
		.name = "as-spdif",
		.of_match_table = ingenic_spdif_match_table,
		.pm = &ingenic_spdif_pm_ops,
	},
	.probe = ingenic_spdif_platform_probe,
	.remove = ingenic_spdif_platform_remove,
};
module_platform_driver(ingenic_spdif_platform_driver);

MODULE_AUTHOR("wqshao <wangquan.shao@ingenic.com>");
MODULE_DESCRIPTION("Ingenic AS spdif SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-as-spdif");
