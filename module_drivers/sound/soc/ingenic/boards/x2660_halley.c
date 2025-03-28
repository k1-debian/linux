/**
 * Copyright (C) 2014 Ingenic Semiconductor Co., Ltd.
 *   http://www.ingenic.com
 * Author:
 *  cwang <chang.wang@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "../as-v1/asoc-aic.h"
#include "../as-v1/asoc-pcm.h"

struct icodec_spk_power {
	int spk_gpio;
	unsigned char spk_en_level;
};
static struct icodec_spk_power *icodec_spk_power;
typedef void (*p_icodec_playback_power)(bool);
extern void set_playback_pwr_callback(p_icodec_playback_power func);

static int x2660_i2s_icdc_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	int ret = 0;
	int id = 0;
	int sysclk = 24576000;
	int mdiv = sysclk / 64 / params_rate(params) / 4;
	int aic_mode = 0, codec_mode = 0;

	/**
	 * The AIC master/slave mode is invalid when internal codec is enabled on the x2600.
	 * This is reserved because the ingenic_set_dai_fmt() function is called back to
	 * determine the master-slave mode of its configuration.
	 * For now, make modifications like this before further considering how to modify it.
	 */
#ifdef CONFIG_SND_ASOC_INGENIC_MASTER_MODE
	aic_mode |= SND_SOC_DAIFMT_CBS_CFM;
	codec_mode |= SND_SOC_DAIFMT_CBS_CFM;
#endif

#ifdef CONFIG_SND_ASOC_INGENIC_SLAVE_MODE
	aic_mode |= SND_SOC_DAIFMT_CBM_CFM;
	codec_mode |= SND_SOC_DAIFMT_CBM_CFM;
#endif

#ifdef CONFIG_SND_ASOC_INGENIC_I2S_MODE
	aic_mode |= SND_SOC_DAIFMT_I2S;
	codec_mode |= SND_SOC_DAIFMT_I2S;
#endif

#ifdef CONFIG_SND_ASOC_INGENIC_LEFT_J_MODE
	aic_mode |= SND_SOC_DAIFMT_MSB;
	codec_mode |= SND_SOC_DAIFMT_LEFT_J;
#endif

	ret = snd_soc_dai_set_fmt(cpu_dai, aic_mode);
	if (ret) {
		return ret;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, codec_mode);
	if (ret) {
		return ret;
	}

	id |= INGENIC_I2S_INNER_CODEC;

	ret = snd_soc_dai_set_sysclk(cpu_dai, id, sysclk, SND_SOC_CLOCK_OUT);
	if (ret) {
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, id, mdiv);
	if (ret) {
		return ret;
	}

	return 0;
}

static int x2660_pcm_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	int ret = 0;
	int id = 0;
	int clkdiv = 30;
	int syndiv = 0x1f;
	int sysclk = 2048000 * (clkdiv - clkdiv % 2);

	ret = snd_soc_dai_set_sysclk(cpu_dai, id, sysclk, SND_SOC_CLOCK_OUT);
	if (ret) {
		return ret;
	}

	id = INGENIC_PCM_CLKDIV;
	ret = snd_soc_dai_set_clkdiv(cpu_dai, id, clkdiv);
	if (ret) {
		return ret;
	}

	id = INGENIC_PCM_SYNDIV;
	ret = snd_soc_dai_set_clkdiv(cpu_dai, id, syndiv);
	if (ret) {
		return ret;
	}

	return 0;
}

static int x2660_i2s_hw_free(struct snd_pcm_substream *substream)
{
	/*notify release pll*/

	return 0;
};

static int x2660_pcm_hw_free(struct snd_pcm_substream *substream)
{
	/*notify release pll*/

	return 0;
}

static const struct snd_soc_dapm_widget x2660_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("MICBIAS", NULL),
	SND_SOC_DAPM_MIC("MICL", NULL),
	SND_SOC_DAPM_MIC("MICR", NULL),
};

static int x2660_i2s_icdc_dai_link_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dapm_context *dapm = &card->dapm;

	snd_soc_dapm_enable_pin(dapm, "Speaker");
	snd_soc_dapm_enable_pin(dapm, "MICBIAS");
	snd_soc_dapm_enable_pin(dapm, "MICL");
	snd_soc_dapm_enable_pin(dapm, "MICR");

	return 0;
}

static struct snd_soc_ops x2660_i2s_icdc_ops = {
	.hw_params = x2660_i2s_icdc_hw_params,
	.hw_free = x2660_i2s_hw_free,
};

static struct snd_soc_ops x2660_pcm_ops = {
	.hw_params = x2660_pcm_hw_params,
	.hw_free = x2660_pcm_hw_free,
};


void icodec_playback_power_ctr(bool enable)
{
	if (!gpio_is_valid(icodec_spk_power->spk_gpio)) {
		return;
	}

	if (enable) {
		gpio_direction_output(icodec_spk_power->spk_gpio, icodec_spk_power->spk_en_level);

		/* Power on time of power amplifier */
		mdelay(40);
	} else {
		gpio_direction_output(icodec_spk_power->spk_gpio, !icodec_spk_power->spk_en_level);
	}
}

static int snd_x2660_probe(struct platform_device *pdev)
{
	int i, ret = 0;
	struct snd_soc_card *card = NULL;
	struct snd_soc_dai_link *dai_link = NULL;
	struct device_node *snd_node = pdev->dev.of_node;
	int num_links;

	const char *cpu_status = NULL;
	const char *platform_status = NULL;
	const char *codec_status = NULL;
	int count = 0;

	num_links = of_property_count_strings(snd_node, "ingenic,dai-link");
	if (num_links < 0) {
		return num_links;
	}
	BUG_ON(!num_links);

	card = (struct snd_soc_card *)devm_kzalloc(&pdev->dev,
	        sizeof(struct snd_soc_card), GFP_KERNEL);
	if (!card) {
		return -ENOMEM;
	}

	dai_link = (struct snd_soc_dai_link *)devm_kzalloc(&pdev->dev,
	           sizeof(struct snd_soc_dai_link) * num_links, GFP_KERNEL);
	if (!dai_link) {
		return -ENOMEM;
	}

	card->owner = THIS_MODULE;
	card->dev = &pdev->dev;
	card->num_links = num_links;
	card->dai_link = dai_link;
	card->dapm_widgets = x2660_dapm_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(x2660_dapm_widgets);

	ret = snd_soc_of_parse_card_name(card, "ingenic,model");
	if (ret) {
		return ret;
	}

	ret = snd_soc_of_parse_audio_routing(card, "ingenic,audio-routing");
	if (ret) {
		return ret;
	}

	icodec_spk_power = kmalloc(sizeof(*icodec_spk_power), GFP_KERNEL);
	if (!icodec_spk_power) {
		return -ENOMEM;
	}
	icodec_spk_power->spk_gpio = of_get_named_gpio(pdev->dev.of_node, "ingenic,spken-gpio", 0);
	if (gpio_is_valid(icodec_spk_power->spk_gpio)) {
		unsigned long init_flags;
		icodec_spk_power->spk_en_level = 1;
		init_flags = GPIOF_OUT_INIT_LOW;
		ret = devm_gpio_request_one(&pdev->dev, icodec_spk_power->spk_gpio, init_flags,          "Speaker_en");
		if (ret) {
			pr_warn("Speaker enable pin(%d) request failed\n", icodec_spk_power->spk_gpio);
		} else {
			pr_info("Speaker enable pin(%d) request ok\n", icodec_spk_power->spk_gpio);
		}
		/* icodec_playback_power = icodec_playback_power_ctr; */
		set_playback_pwr_callback(icodec_playback_power_ctr);
	}

	for (i = 0; i < card->num_links; i++) {
		dai_link[i - count].cpus = devm_kzalloc(&pdev->dev, sizeof(struct snd_soc_dai_link_component), GFP_KERNEL);
		dai_link[i - count].codecs = devm_kzalloc(&pdev->dev, sizeof(struct snd_soc_dai_link_component), GFP_KERNEL);
		dai_link[i - count].platforms = devm_kzalloc(&pdev->dev, sizeof(struct snd_soc_dai_link_component), GFP_KERNEL);

		dai_link[i - count].num_cpus = 1;
		dai_link[i - count].num_platforms = 1;
		dai_link[i - count].num_codecs = 1;

		dai_link[i - count].cpus->of_node = of_parse_phandle(snd_node, "ingenic,cpu-dai", i);
		dai_link[i - count].platforms->of_node = of_parse_phandle(snd_node, "ingenic,platform", i);
		dai_link[i - count].codecs->of_node = of_parse_phandle(snd_node, "ingenic,codec", i);

		of_property_read_string(dai_link[i - count].cpus->of_node, "status", &cpu_status);
		of_property_read_string(dai_link[i - count].platforms->of_node, "status", &platform_status);
		of_property_read_string(dai_link[i - count].codecs->of_node, "status", &codec_status);

		if (strcmp(cpu_status, "okay") || strcmp(platform_status, "okay") || strcmp(codec_status, "okay")) {
			dai_link[i - count].cpus->of_node = NULL;
			dai_link[i - count].platforms->of_node = NULL;
			dai_link[i - count].codecs->of_node = NULL;
			count++;
			continue;
		}

		ret = of_property_read_string_index(snd_node, "ingenic,codec-dai", i,
		                                    &(dai_link[i - count].codecs->dai_name));
		if (ret || !dai_link[i - count].cpus->of_node ||
		    !dai_link[i - count].codecs->of_node ||
		    !dai_link[i - count].platforms->of_node) {
			return -ENODEV;
		}

		ret = of_property_read_string_index(snd_node, "ingenic,dai-link", i,
		                                    &(dai_link[i - count].name));
		if (ret) {
			return -ENODEV;
		}

		ret = of_property_read_string_index(snd_node, "ingenic,stream", i,
		                                    &(dai_link[i - count].stream_name));
		if (ret) {
			dai_link[i - count].stream_name = dai_link[i - count].name;
		}

		dev_dbg(&pdev->dev, "dai_link %s\n", dai_link[i - count].name);
		dev_dbg(&pdev->dev, "stream_name %s\n", dai_link[i - count].stream_name);
		dev_dbg(&pdev->dev, "cpu %s(%s)\n", dai_link[i - count].cpus->of_node->name,
		        dai_link[i - count].cpus->of_node->full_name);
		dev_dbg(&pdev->dev, "platform %s(%s)\n", dai_link[i - count].platforms->of_node->name,
		        dai_link[i - count].platforms->of_node->full_name);
		dev_dbg(&pdev->dev, "codec dai %s\n", dai_link[i - count].codecs->dai_name);
		dev_dbg(&pdev->dev, "codec %s(%s)\n", dai_link[i - count].codecs->of_node->name,
		        dai_link[i - count].codecs->of_node->full_name);

		if (!strcmp(dai_link[i - count].name, "i2s-icodec")) {
			dai_link[i - count].ops = &x2660_i2s_icdc_ops;
			dai_link[i - count].init = x2660_i2s_icdc_dai_link_init;
		}

		if (!strcmp(dai_link[i - count].name, "pcm")) {
			dai_link[i - count].ops = &x2660_pcm_ops;
		}

		if (!strcmp(dai_link[i - count].name, "dmic")) {
			dai_link[i - count].capture_only = 1;
		}
	}
	card->num_links -= count;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, card);

	dev_info(&pdev->dev, "Sound card registration succeeded.");

	return 0;
}

static int snd_x2660_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	platform_set_drvdata(pdev, NULL);

	dev_info(&pdev->dev, "Sound card released successfully.");

	return 0;
}

static const struct of_device_id sound_dt_match[] = {
	{ .compatible = "ingenic,x2660-halley-sound", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, sound_dt_match);

static struct platform_driver snd_x2660_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ingenic-x2660-halley",
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(sound_dt_match),
	},
	.probe = snd_x2660_probe,
	.remove = snd_x2660_remove,
};
module_platform_driver(snd_x2660_driver);

MODULE_AUTHOR("cwang<chang.wang@ingenic.com>");
MODULE_DESCRIPTION("ALSA SoC X2660 Halley Snd Card");
MODULE_LICENSE("GPL");
