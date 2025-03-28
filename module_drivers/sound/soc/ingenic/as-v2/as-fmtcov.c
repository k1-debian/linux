/*
 * ALSA Soc Audio Layer -- Ingenic AS fmtcov driver
 *
 * Copyright 2017 - 2022 Ingenic Semiconductor Co.,Ltd
 *  wqshao <wangquan.shao@ingenic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/list.h>
#include <sound/soc.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <sound/pcm_params.h>
#include "as-fmtcov.h"
#include "as-dma.h"

#ifdef DEBUG
	static int ingenic_fmtcov_debug = 1;
#else
	static int ingenic_fmtcov_debug = 0;
#endif
module_param(ingenic_fmtcov_debug, int, 0644);
#define FMTCOV_DEBUG_MSG(msg...)                \
	do {                                        \
		if (ingenic_fmtcov_debug)               \
			printk(KERN_DEBUG"FMTCOV: " msg);   \
	} while(0)

static struct ingenic_as_fmtcov *as_fmtcov;

static bool ingenic_fmt_need_packen(snd_pcm_format_t format)
{
	switch (format) {
	case SNDRV_PCM_FORMAT_S24_3LE:
	case SNDRV_PCM_FORMAT_U24_3LE:
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_U20_3LE:
	case SNDRV_PCM_FORMAT_S18_3LE:
	case SNDRV_PCM_FORMAT_U18_3LE:
	case SNDRV_PCM_FORMAT_S24_3BE:
	case SNDRV_PCM_FORMAT_U24_3BE:
	case SNDRV_PCM_FORMAT_S20_3BE:
	case SNDRV_PCM_FORMAT_U20_3BE:
	case SNDRV_PCM_FORMAT_S18_3BE:
	case SNDRV_PCM_FORMAT_U18_3BE:
		return true;
	default:
		break;
	}

	return false;
}

static void ingenic_as_fmtcov_fix_match_mod(u8 dai_id, u32 fmtcfg)
{
	as_fmtcov->fmtcfg[dai_id] = fmtcfg;
}

int ingenic_spdif_fmtcov_be_fix(struct snd_pcm_substream *substream,
                                struct snd_pcm_hw_params *params)
{
	struct ingenic_as_dma_runtime *prtd = substream_to_prtd(substream);
	u32 fmtcfg = 0;

	if (WARN(prtd->dai_id > FMT_CHL_NUM, "[as-fmtcov]: %s dai_id(%d) out of range\n",
	         __func__, prtd->dai_id)) {
		return -ECHRNG;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		fmtcfg = DFCR_SS(params_width(params)) | DFCR_CHNUM(1);
	} else {
		fmtcfg = DFCR_SS(32) | DFCR_CHNUM(1);
	}
	if (ingenic_fmt_need_packen(params_format(params))) {
		fmtcfg |= DFCR_PACKEN;
	}

	ingenic_as_fmtcov_fix_match_mod(prtd->dai_id, fmtcfg);
	return 0;
}
EXPORT_SYMBOL_GPL(ingenic_spdif_fmtcov_be_fix);

int ingenic_as_fmtcov_cfg(struct snd_pcm_substream *substream,
                          struct snd_pcm_hw_params *params)
{
	struct ingenic_as_dma_runtime *prtd = substream_to_prtd(substream);
	u32 fmtcfg;

	if (WARN(prtd->dai_id > FMT_CHL_NUM, "[as-fmtcov]: %s dai_id(%d) out of range\n",
	         __func__, prtd->dai_id)) {
		return -ECHRNG;
	}

	fmtcfg = DFCR_SS(params_width(params)) | DFCR_CHNUM(params_channels(params));
	if (ingenic_fmt_need_packen(params_format(params))) {
		fmtcfg |= DFCR_PACKEN;
	}

	if (as_fmtcov->fmtcfg[prtd->dai_id]) {
		fmtcfg = as_fmtcov->fmtcfg[prtd->dai_id];
		as_fmtcov->fmtcfg[prtd->dai_id] = 0;
	}

	FMTCOV_DEBUG_MSG("enter %s, fmtcov DFCR%d = 0x%08x\n", __func__, prtd->dai_id, fmtcfg);
	writel_relaxed(fmtcfg, as_fmtcov->io_base + DFCR(prtd->dai_id));

	return 0;
}
EXPORT_SYMBOL_GPL(ingenic_as_fmtcov_cfg);

void ingenic_as_fmtcov_enable(u8 dai_id, bool enable)
{
	regmap_update_bits(as_fmtcov->regmap, DFCR(dai_id), DFCR_ENABLE, enable ? DFCR_ENABLE : 0);
}
EXPORT_SYMBOL_GPL(ingenic_as_fmtcov_enable);

static int ingenic_as_fmtcov_platform_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct regmap_config regmap_config = {
		.reg_bits = 32,
		.reg_stride = 4,
		.val_bits = 32,
		.cache_type = REGCACHE_NONE,
	};

	as_fmtcov = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_as_fmtcov), GFP_KERNEL);
	if (!as_fmtcov) {
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	as_fmtcov->io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(as_fmtcov->io_base)) {
		return PTR_ERR(as_fmtcov->io_base);
	}
	regmap_config.max_register = resource_size(res) - 0x4;
	as_fmtcov->regmap = devm_regmap_init_mmio(&pdev->dev, as_fmtcov->io_base,
	                    &regmap_config);
	if (IS_ERR(as_fmtcov->regmap)) {
		return PTR_ERR(as_fmtcov->regmap);
	}
	as_fmtcov->dev = &pdev->dev;

	platform_set_drvdata(pdev, as_fmtcov);
	return 0;
}

static int ingenic_as_fmtcov_platform_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id ingenic_as_fmtcov_match_table[] = {
	{ .compatible = "ingenic,as-fmtcov", },
	{ }
};
MODULE_DEVICE_TABLE(of, ingenic_as_fmtcov_match_table);

static struct platform_driver ingenic_as_fmtcov_platform_driver = {
	.driver = {
		.name = "as-fmtcov",
		.of_match_table = ingenic_as_fmtcov_match_table,
	},
	.probe = ingenic_as_fmtcov_platform_probe,
	.remove = ingenic_as_fmtcov_platform_remove,
};

int ingenic_as_fmtcov_platform_driver_modinit(void)
{
	return platform_driver_register(&ingenic_as_fmtcov_platform_driver);
}
EXPORT_SYMBOL(ingenic_as_fmtcov_platform_driver_modinit);

void ingenic_as_fmtcov_platform_driver_exit(void)
{
	platform_driver_unregister(&ingenic_as_fmtcov_platform_driver);
}
EXPORT_SYMBOL(ingenic_as_fmtcov_platform_driver_exit);

/* module_platform_driver(ingenic_as_fmtcov_platform_driver); */

MODULE_AUTHOR("wqshao <wangquan.shao@ingenic.com>");
MODULE_DESCRIPTION("Ingenic AS fmtcov SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-as-fmtcov");
