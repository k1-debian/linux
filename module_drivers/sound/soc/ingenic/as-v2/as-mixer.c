/*
 * ALSA Soc Audio Layer -- Ingenic AS MIXER driver
 *
 * Copyright 2017 - 2022 Ingenic Semiconductor Co.,Ltd
 *  cli <chen.li@ingenic.com>
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
#include "as-mixer.h"

static int ingenic_as_mixer_widget_event(struct snd_soc_dapm_widget *widget,
        struct snd_kcontrol *kcontrol, int event)
{
	if (event == SND_SOC_DAPM_PRE_PMU) {
		struct snd_soc_dapm_context *dapm = widget->dapm;
		struct snd_soc_component *component = snd_soc_dapm_to_component(dapm);
		struct ingenic_as_mixer *as_mixer = (struct ingenic_as_mixer *)
		                                    snd_soc_component_get_drvdata(component);
		int id = (int)widget->priv;
		dev_dbg(as_mixer->dev, "reset mixer[%d]\n", id);
		regmap_update_bits(as_mixer->regmap, MIX_CTL(id), MIX_RESET, MIX_RESET);
	}
	return 0;
}

static int ingenic_as_mixer_probe(struct snd_soc_component *cmpnt)
{
	struct ingenic_as_mixer *as_mixer = dev_get_drvdata(cmpnt->dev);
	int i;

	for (i = 0; i < as_mixer->num_mixers; i++) {
		snd_soc_component_update_bits(cmpnt, MIX_CFG(i),
		                              MIX_CH_MSK | MIX_MODE_MSK, MIX_CH(2) | MIX_MODE_AVG);
	}
	return 0;
}

static const char *const ingenic_mixer0_mode_src[] = {
	"Linear weighted plus",
	"Average",
	"Clamping",
	"Nonlinear Distort",
};

static int ingenic_mixer0_mode_values[] = {
	0, 1, 2, 3,
};

static SOC_VALUE_ENUM_SINGLE_DECL(ingenic_mixer0_mode,
                                  MIX_CFG(0), MIX_MODE_SFT,
                                  0x3, ingenic_mixer0_mode_src,
                                  ingenic_mixer0_mode_values);

static const struct snd_kcontrol_new ingenic_mixer_controls[] = {
	/* Mixer0 */
	SOC_SINGLE("Mixer0 LR_MIX Switch", MIX_CFG(0), MIX_LR_MIX_SFT, 1, 0),
	SOC_ENUM("Mixer0 MIX_MODE Option", ingenic_mixer0_mode),
};

static int ingenic_as_mixer_platform_probe(struct platform_device *pdev)
{
	struct ingenic_as_mixer *as_mixer;
	struct resource *res;
	struct snd_soc_dapm_widget *dapm_widgets;
	char *dev_name;
	int i, ret;
	struct regmap_config regmap_config = {
		.reg_bits = 32,
		.reg_stride = 4,
		.val_bits = 32,
		.cache_type = REGCACHE_NONE,
	};

	as_mixer = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_as_mixer), GFP_KERNEL);
	if (!as_mixer) {
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	as_mixer->io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(as_mixer->io_base)) {
		return PTR_ERR(as_mixer->io_base);
	}
	regmap_config.max_register = resource_size(res) - 0x4;
	as_mixer->regmap = devm_regmap_init_mmio(&pdev->dev, as_mixer->io_base,
	                   &regmap_config);
	if (IS_ERR(as_mixer->regmap)) {
		return PTR_ERR(as_mixer->regmap);
	}
	as_mixer->dev = &pdev->dev;

	if (of_property_read_u32(pdev->dev.of_node, "ingenic,num-mixers",
	                         &as_mixer->num_mixers)) {
		return -ENODEV;
	}

	dapm_widgets = devm_kzalloc(&pdev->dev,
	                            sizeof(struct snd_soc_dapm_widget) * as_mixer->num_mixers,
	                            GFP_KERNEL);
	if (!dapm_widgets) {
		return -ENOMEM;
	}

#define NAME_LEN 10
	dev_name = devm_kzalloc(&pdev->dev,
	                        sizeof(char) * as_mixer->num_mixers * NAME_LEN,
	                        GFP_KERNEL);
	if (!dev_name) {
		return -ENOMEM;
	}

	for (i = 0; i < as_mixer->num_mixers; i++) {
		dapm_widgets[i].id = snd_soc_dapm_mixer;
		snprintf(&dev_name[i * NAME_LEN], NAME_LEN, "MIX%d", i);
		dapm_widgets[i].name = &dev_name[i * NAME_LEN];
		dapm_widgets[i].reg = MIX_CTL(i);
		dapm_widgets[i].mask = 1;
		dapm_widgets[i].shift = MIX_EN_SFT;
		dapm_widgets[i].on_val = 1;
		dapm_widgets[i].event = ingenic_as_mixer_widget_event;
		dapm_widgets[i].priv = (void *)i;
		dapm_widgets[i++].event_flags = SND_SOC_DAPM_PRE_PMU;
	}
#undef  NAME_LEN

	as_mixer->cmpnt_drv.name = "ingenic-as-mixer";
	as_mixer->cmpnt_drv.controls = ingenic_mixer_controls;
	as_mixer->cmpnt_drv.num_controls = ARRAY_SIZE(ingenic_mixer_controls);
	as_mixer->cmpnt_drv.dapm_widgets = dapm_widgets;
	as_mixer->cmpnt_drv.num_dapm_widgets = as_mixer->num_mixers;
	as_mixer->cmpnt_drv.probe = ingenic_as_mixer_probe;

	platform_set_drvdata(pdev, as_mixer);

	ret = snd_soc_register_component(&pdev->dev, &as_mixer->cmpnt_drv,
	                                 NULL, 0);
	if (!ret) {
		dev_info(&pdev->dev, "amixer platform probe success\n");
	}

	return ret;
}

static int ingenic_as_mixer_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id ingenic_as_mixer_match_table[] = {
	{ .compatible = "ingenic,as-mixer", },
	{ }
};
MODULE_DEVICE_TABLE(of, ingenic_as_mixer_match_table);

static struct platform_driver ingenic_as_mixer_platform_driver = {
	.driver = {
		.name = "as-mixer",
		.of_match_table = ingenic_as_mixer_match_table,
	},
	.probe = ingenic_as_mixer_platform_probe,
	.remove = ingenic_as_mixer_platform_remove,
};

int ingenic_as_mixer_platform_driver_modinit(void)
{
	return platform_driver_register(&ingenic_as_mixer_platform_driver);
}
EXPORT_SYMBOL(ingenic_as_mixer_platform_driver_modinit);

void ingenic_as_mixer_platform_driver_exit(void)
{
	platform_driver_unregister(&ingenic_as_mixer_platform_driver);
}
EXPORT_SYMBOL(ingenic_as_mixer_platform_driver_exit);

/* module_platform_driver(ingenic_as_mixer_platform_driver); */

MODULE_AUTHOR("cli <chen.li@ingenic.com>");
MODULE_DESCRIPTION("Ingenic AS Mixer SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-as-mixer");
