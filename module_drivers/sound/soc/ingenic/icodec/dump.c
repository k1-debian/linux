/*
 * sound/soc/ingenic/icodec/dump.c
 * ALSA SoC Audio driver -- dump codec driver used for devices like bt, hdmi

 * Copyright 2014 Ingenic Semiconductor Co.,Ltd
 *  cli <chen.li@ingenic.com>
 *
 * Note: dlv4780 is an internal codec for ingenic SOC
 *   used for dlv4780 m200 and so on
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

static const struct snd_soc_dai_driver pcm_dump_dai __initdata = {
	.name = "pcm-dump",
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_S20_3LE |
		           SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_U8,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_S20_3LE |
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_U8,
	},
};

static const struct snd_soc_dai_driver spdif_dump_dai __initdata = {
	.name = "spdif-dump",
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		         SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static const struct snd_soc_dai_driver dmic_dump_dai __initdata = {
	.name = "dmic-dump",
	.capture = {
		.channels_min = 1,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static const struct of_device_id dump_dt_match[];
static int dump_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_component_driver *dump_codec;
	struct snd_soc_dai_driver *dump_dai;
	const struct of_device_id *match;
	int ret = 0;

	dump_dai = (struct snd_soc_dai_driver *)devm_kzalloc(&pdev->dev,
	           sizeof(struct snd_soc_dai_driver), GFP_KERNEL);
	if (!dump_dai) {
		return -ENOMEM;
	}
	match = of_match_node(dump_dt_match, pdev->dev.of_node);
	if (!match) {
		return -ENODEV;
	}
	memcpy(dump_dai, match->data, sizeof(struct snd_soc_dai_driver));

	dump_codec = (struct snd_soc_component_driver *)devm_kzalloc(&pdev->dev,
	             sizeof(struct snd_soc_component_driver), GFP_KERNEL);
	if (!dump_codec) {
		return -ENOMEM;
	}

	ret = snd_soc_register_component(&pdev->dev, dump_codec, dump_dai, 1);
	dev_dbg(&pdev->dev, "codec dump codec platfrom probe success\n");
	return ret;
}

static int dump_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id dump_dt_match[] = {
	{ .compatible = "ingenic,pcm-dump-codec", .data = &pcm_dump_dai},
	{ .compatible = "ingenic,spdif-dump-codec", .data = &spdif_dump_dai},
	{ .compatible = "ingenic,dmic-dump-codec", .data = &dmic_dump_dai},
	{},
};
MODULE_DEVICE_TABLE(of, dump_dt_match);

static struct platform_driver dump_driver = {
	.driver = {
		.name = "dump codec",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(dump_dt_match),
	},
	.probe = dump_platform_probe,
	.remove = dump_platform_remove,
};
module_platform_driver(dump_driver);

MODULE_DESCRIPTION("Dump Codec Driver");
MODULE_AUTHOR("cli<chen.li@ingenic.com>");
MODULE_LICENSE("GPL");
