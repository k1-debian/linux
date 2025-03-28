/*
 * ALSA Soc Audio Layer -- ingenic as(audio system) Virtual FE (Basic Audio
 * Interface Controller) driver
 *
 * Copyright 2017 - 2022 Ingenic Semiconductor Co.,Ltd
 *   wqshao <wangquan.shao@ingenic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

static const struct snd_pcm_hardware ingenic_vir_fe_hardware = {
	/* Random values to keep userspace happy when checking constraints */
	.info           = SNDRV_PCM_INFO_INTERLEAVED |
	                  SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.buffer_bytes_max   = 128 * 1024,
	.period_bytes_min   = PAGE_SIZE,
	.period_bytes_max   = PAGE_SIZE * 2,
	.periods_min        = 2,
	.periods_max        = 128,
};

static int ingenic_vir_fe_open(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	/* BE's dont need virtual params */
	if (!rtd->dai_link->no_pcm) {
		snd_soc_set_runtime_hwparams(substream, &ingenic_vir_fe_hardware);
	}

	return 0;
}

static int ingenic_vir_fe_ioctl(struct snd_soc_component *component, struct snd_pcm_substream *substream, unsigned int cmd, void *arg)
{
	return snd_pcm_lib_ioctl(substream, cmd, arg);
}

static int ingenic_vir_fe_prepare(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	/* The analog play buffer has data on play music */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->stop_threshold = runtime->boundary;
	}

	return 0;
}

static struct snd_soc_component_driver ingenic_vir_platform = {
	.name   = "as-vir-fe",
	.open = ingenic_vir_fe_open,
	.ioctl = ingenic_vir_fe_ioctl,
	.prepare = ingenic_vir_fe_prepare,
};

#define INGENIC_STUB_RATES  SNDRV_PCM_RATE_8000_192000
#define INGENIC_STUB_FORMATS    (SNDRV_PCM_FMTBIT_S8 | \
                                 SNDRV_PCM_FMTBIT_U8 | \
                                 SNDRV_PCM_FMTBIT_S16_LE | \
                                 SNDRV_PCM_FMTBIT_U16_LE | \
                                 SNDRV_PCM_FMTBIT_S24_LE | \
                                 SNDRV_PCM_FMTBIT_U24_LE | \
                                 SNDRV_PCM_FMTBIT_S32_LE | \
                                 SNDRV_PCM_FMTBIT_U32_LE | \
                                 SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE)

#define INGENIC_VIR_FE_MAX  10
static int ingenic_as_vir_fe_probe(struct platform_device *pdev)
{
	struct snd_soc_dai_driver *vir_fe_dais;
	static char dai_name[INGENIC_VIR_FE_MAX][20];
	int i, ret, num_dais, cap_dai_bitmsk;

	if (of_property_read_u32(pdev->dev.of_node, "ingenic,num-dais", &num_dais)) {
		return -ENODEV;
	}

	if (of_property_read_u32(pdev->dev.of_node, "ingenic,cap-dai-bm", &cap_dai_bitmsk)) {
		return -ENODEV;
	}

	vir_fe_dais =  devm_kzalloc(&pdev->dev, sizeof(*vir_fe_dais) * num_dais, GFP_KERNEL);
	if (!vir_fe_dais) {
		return -ENOMEM;
	}

	for (i = 0; i < num_dais; i++) {
		sprintf(dai_name[i], "Virtual-FE%d", i);
		vir_fe_dais[i].name = dai_name[i];
		vir_fe_dais[i].id = i;
		if ((BIT(i) & cap_dai_bitmsk)) {
			vir_fe_dais[i].capture.stream_name = dai_name[i];
			vir_fe_dais[i].capture.channels_min = 1;
			vir_fe_dais[i].capture.channels_max = 384;
			vir_fe_dais[i].capture.rates = INGENIC_STUB_RATES;
			vir_fe_dais[i].capture.formats = INGENIC_STUB_FORMATS;
		} else {
			vir_fe_dais[i].playback.stream_name = dai_name[i];
			vir_fe_dais[i].playback.channels_min = 1;
			vir_fe_dais[i].playback.channels_max = 384;
			vir_fe_dais[i].playback.rates = INGENIC_STUB_RATES;
			vir_fe_dais[i].playback.formats = INGENIC_STUB_FORMATS;
		}
	}

	ret = snd_soc_register_component(&pdev->dev, &ingenic_vir_platform, vir_fe_dais, num_dais);

	if (ret < 0) {
		snd_soc_unregister_component(&pdev->dev);
		return ret;
	}

	dev_info(&pdev->dev, "probe success!!!\n");
	return ret;
}

static int ingenic_as_vir_fe_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id ingenic_as_vir_fe_match_table[] = {
	{ .compatible = "ingenic,as-vir-fe", },
	{ }
};
MODULE_DEVICE_TABLE(of, ingenic_as_vir_fe_match_table);

static struct platform_driver ingenic_as_vir_fe_platform_driver = {
	.driver = {
		.name = "as-virtual-fe",
		.of_match_table = ingenic_as_vir_fe_match_table,
	},
	.probe = ingenic_as_vir_fe_probe,
	.remove = ingenic_as_vir_fe_remove,
};
module_platform_driver(ingenic_as_vir_fe_platform_driver);

MODULE_AUTHOR("wqshao <wangquan.shao@ingenic.com>");
MODULE_DESCRIPTION("Ingenic AS Virtual FE SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-as-vir-fe");
