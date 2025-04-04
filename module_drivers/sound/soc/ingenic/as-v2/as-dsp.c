/*
 * ALSA Soc Audio Layer -- ingenic as(audio system) dsp (dbus) driver
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
#include <linux/regmap.h>
#include <linux/thermal.h>

#include "as-dsp.h"

/* Global variables */
struct ingenic_as_dsp *as_dsp_t;

#ifdef DEBUG
	static int ingenic_dsp_debug = 1;
#else
	static int ingenic_dsp_debug = 0;
#endif

module_param(ingenic_dsp_debug, int, 0644);

#define DSP_DEBUG_MSG(msg...)   \
	do { \
		if (ingenic_dsp_debug)  \
			printk(KERN_DEBUG"DSP: " msg);  \
	} while(0)

static const char *const lo_names[LI_PORT_MAX_NUM] = {
	"LO0_MUX", "LO1_MUX", "LO2_MUX", "LO3_MUX", "LO4_MUX",
	"LO5_MUX", "LO6_MUX", "LO7_MUX", "LO8_MUX", "LO9_MUX",
	"LO10_MUX", "LO11_MUX", "LO12_MUX", "LO13_MUX", "LO14_MUX",
	"LO15_MUX"
};

#define LO_MUX_ITEMS_CNT    (LI_PORT_MAX_NUM + 1)
static const char *const mux_item_names[LO_MUX_ITEMS_CNT] = {
	"UNUSED", "LI0", "LI1", "LI2", "LI3", "LI4",
	"LI5", "LI6", "LI7", "LI8", "LI9",
	"LI10", "LI11", "LI12", "LI13", "LI14",
	"LI15"
};
#define LO_PORT_NAME(num)   lo_names[num]
#define LI_PORT_NAME(num)   mux_item_names[(num) + 1]
#define ITEMS_TO_LI_DEVID(item) (u8)(item != 0 ? (item) - 1 : LI_PORT_MAX_NUM)

static struct ingenic_dsp_port *ingenic_dsp_get_port(struct ingenic_as_dsp *as_dsp,
        u8 dev_id, bool is_out)
{
	int idx;

	if (!as_dsp) {
		return NULL;
	}

	if (!is_out) {
		if (unlikely(dev_id >= LI_PORT_MAX_NUM)) {
			return NULL;
		}
		idx = LI_DEVID_TO_IDX(dev_id);
	} else {
		if (unlikely(dev_id >= LO_PORT_MAX_NUM)) {
			return NULL;
		}
		idx = LO_DEVID_TO_IDX(dev_id);
	}

	if (as_dsp->port[idx].vaild) {
		return &as_dsp->port[idx];
	} else {
		return NULL;
	}
}

static inline u8 ingenic_timeslot_alloc(struct ingenic_as_dsp *as_dsp)
{
	int id;
	id = find_first_zero_bit(as_dsp->slot_bitmap, SLOT_NUM);
	bitmap_set(as_dsp->slot_bitmap, id, 1);
	return id + 1;  /*skip unused slot id 0*/
}

static inline void ingenic_timeslot_free(struct ingenic_as_dsp *as_dsp, u8 id)
{
	if (!id) { /*skip unused slot id 0*/
		return;
	}
	bitmap_clear(as_dsp->slot_bitmap, (id - 1), 1);
}

/*Must under as_dsp mutex*/
static void ingenic_dsp_free_timesolt(struct ingenic_as_dsp *as_dsp,
                                      struct ingenic_as_dsp_enum *e)
{
	struct ingenic_dsp_port *lo, *li;
	unsigned long reg, sft;

	lo = ingenic_dsp_get_port(as_dsp, e->dev_id, true);
	if (!lo || !lo->timeslot) {
		return;
	}

	/*clear target device timeslot*/
	reg = ingenic_get_dsp_port_register(e->dev_id, &sft, true);
	regmap_update_bits(as_dsp->regmap, reg, BSORTT_MSK(sft), 0);
	lo->timeslot = 0;
	list_del(&lo->node);

	li = (struct ingenic_dsp_port *)lo->private;
	if (li && list_empty(&li->head)) {
		/*clear source device timeslot*/
		reg = ingenic_get_dsp_port_register(li->dev_id, &sft, false);
		regmap_update_bits(as_dsp->regmap, reg, BSORTT_MSK(sft), 0);

		/*disable and free timeslot*/
		regmap_update_bits(as_dsp->regmap, BTCLR, BT_TSLOT(li->timeslot), BT_TSLOT(li->timeslot));

		ingenic_timeslot_free(as_dsp, li->timeslot);

		li->timeslot = 0;
	}
	lo->private = NULL;
}

#if 0
#ifdef CONFIG_AUDIO_DUMP
static void dump_dsp_register(struct ingenic_as_dsp *as_dsp)
{
	unsigned int val;

	printk("==>DSP\n");
	regmap_read(as_dsp->regmap, BTSR, &val);
	printk("BTSR(0x%x):	0x%x\n", BTSR, val);
	regmap_read(as_dsp->regmap, BFSR, &val);
	printk("BFSR(0x%x):	0x%x\n", BFSR, val);
	regmap_read(as_dsp->regmap, BFCR0, &val);
	printk("BFCR0(0x%x):	0x%x\n", BFCR0, val);
	regmap_read(as_dsp->regmap, BFCR1, &val);
	printk("BFCR1(0x%x):	0x%x\n", BFCR1, val);
	regmap_read(as_dsp->regmap, BFCR2, &val);
	printk("BFCR2(0x%x):	0x%x\n", BFCR2, val);
	regmap_read(as_dsp->regmap, BST0, &val);
	printk("BST0(0x%x):	0x%x\n", BST0, val);
	regmap_read(as_dsp->regmap, BST1, &val);
	printk("BST1(0x%x):	0x%x\n", BST1, val);
	regmap_read(as_dsp->regmap, BST2, &val);
	printk("BST2(0x%x):	0x%x\n", BST2, val);
	regmap_read(as_dsp->regmap, BTT0, &val);
	printk("BTT0(0x%x):	0x%x\n", BTT0, val);
	regmap_read(as_dsp->regmap, BTT1, &val);
	printk("BTT1(0x%x):	0x%x\n", BTT1, val);
}
#endif
#endif

int ingenic_dsp_is_spdif_out(uint32_t dma_device_id)
{
	unsigned long reg, sft;
	unsigned int val, dma_timeslot, spdif_out_timeslot;
	unsigned int dev_id;
	unsigned int spdif_out_id;
	struct ingenic_as_dsp *as_dsp = as_dsp_t;

	if (dma_device_id < 5) {
		/* DMA0 ~ DMA4 */
		dev_id = dma_device_id + 8;

		/* spdif out*/
		spdif_out_id = 2;

		/* get DMA device timeslot */
		reg = ingenic_get_dsp_port_register(dev_id, &sft, false);

		regmap_read(as_dsp->regmap, reg, &val);
		val &= BSORTT_MSK(sft);
		val >>= sft;
		dma_timeslot = val;

		/* get spdif out timeslot */
		reg = ingenic_get_dsp_port_register(spdif_out_id, &sft, true);

		regmap_read(as_dsp->regmap, reg, &val);
		val &= BSORTT_MSK(sft);
		val >>= sft;
		spdif_out_timeslot = val;

		if ((dma_timeslot == spdif_out_timeslot) && dma_timeslot) {
			return 1;
		}
	}

	return 0;
}

static int ingenic_dsp_request_timesolt(struct ingenic_as_dsp *as_dsp,
                                        struct ingenic_as_dsp_enum *e, int item)
{
	struct ingenic_dsp_port *lo, *li;
	unsigned long reg, sft;

	/*zero is unused*/
	if (item == 0) {
		return 0;
	}

	li = ingenic_dsp_get_port(as_dsp, ITEMS_TO_LI_DEVID(item), false);
	lo = ingenic_dsp_get_port(as_dsp, e->dev_id, true);
	if (!li || !lo) {
		dev_err(as_dsp->dev, "request timeslot: get %s port(%u) failed\n",
		        li ? "lo" : "li", li ? e->dev_id : ITEMS_TO_LI_DEVID(item));
		return -ENODEV;
	}

	if (WARN(lo->timeslot, "[as-dsp]request timeslot: lo port (%u)has been request\n",
	         e->dev_id)) {
		return -EBUSY;
	}

	if (li->timeslot == 0) {
		/*config bus source timeslot*/
		li->timeslot = ingenic_timeslot_alloc(as_dsp);
		reg = ingenic_get_dsp_port_register(li->dev_id, &sft, false);
		regmap_update_bits(as_dsp->regmap, reg, BSORTT_MSK(sft), BSORTT(li->timeslot, sft));
	}

	/*config bus target timeslot*/
	lo->timeslot = li->timeslot;
	reg = ingenic_get_dsp_port_register(lo->dev_id, &sft, true);
	regmap_update_bits(as_dsp->regmap, reg, BSORTT_MSK(sft), BSORTT(lo->timeslot, sft));

	/*enable timeslot*/
	regmap_update_bits(as_dsp->regmap, BTSET, BT_TSLOT(li->timeslot), BT_TSLOT(li->timeslot));

	list_add_tail(&lo->node, &li->head);
	lo->private = (void *)li;
	return 0;
}

static int ingenic_lomux_dapm_get_enum(struct snd_kcontrol *kcontrol,
                                       struct snd_ctl_elem_value *ucontrol)
{
	struct ingenic_as_dsp_enum *e = (struct ingenic_as_dsp_enum *)kcontrol->private_value;
	ucontrol->value.enumerated.item[0] = e->value;
	return 0;
}

static int ingenic_lomux_dapm_put_enum(struct snd_kcontrol *kcontrol,
                                       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_component *component = snd_soc_dapm_to_component(dapm);
	struct ingenic_as_dsp *as_dsp = (struct ingenic_as_dsp *)snd_soc_component_get_drvdata(component);
	struct ingenic_as_dsp_enum *e = (struct ingenic_as_dsp_enum *)kcontrol->private_value;
	unsigned int item = ucontrol->value.enumerated.item[0];
	unsigned int change = 0;

	if (item >= e->e.items) {
		return -EINVAL;
	}

	mutex_lock(&as_dsp->dapm_mutex);
	if (e->value != item) {
		change = 1;
		e->value = item;
	}
	if (change) {
		/*free old timeslot*/
		ingenic_dsp_free_timesolt(as_dsp, e);

		/*alloc new timeslot*/
		ingenic_dsp_request_timesolt(as_dsp, e, item);

		snd_soc_dapm_mux_update_power(dapm, kcontrol, item, (struct soc_enum *)e, NULL);
	}
	mutex_unlock(&as_dsp->dapm_mutex);

	return change;
}

static struct ingenic_as_dsp_enum lomux_enum[] = {  /*not const*/
#define LOMUX_SOC_ENUM(_dev_id) \
	{ \
		.e = SOC_ENUM_SINGLE_EXT(LO_MUX_ITEMS_CNT, mux_item_names), \
		.dev_id = _dev_id,  \
		.value = 0, \
	}
	LOMUX_SOC_ENUM(0),
	LOMUX_SOC_ENUM(1),
	LOMUX_SOC_ENUM(2),
	LOMUX_SOC_ENUM(3),
	LOMUX_SOC_ENUM(4),
	LOMUX_SOC_ENUM(5),
	LOMUX_SOC_ENUM(6),
	LOMUX_SOC_ENUM(7),
	LOMUX_SOC_ENUM(8),
	LOMUX_SOC_ENUM(9),
	LOMUX_SOC_ENUM(10),
	LOMUX_SOC_ENUM(11),
	LOMUX_SOC_ENUM(12),
	LOMUX_SOC_ENUM(13),
	LOMUX_SOC_ENUM(14),
	LOMUX_SOC_ENUM(15),
#undef LOMUX_SOC_ENUM
};

static const struct snd_kcontrol_new lomux_controls[] = {
#define LOMUX_SOC_DAPM_ENUM_EXT(dev_id) \
	SOC_DAPM_ENUM_EXT("route", lomux_enum[(dev_id)], \
	                  ingenic_lomux_dapm_get_enum, \
	                  ingenic_lomux_dapm_put_enum)
	LOMUX_SOC_DAPM_ENUM_EXT(0),
	LOMUX_SOC_DAPM_ENUM_EXT(1),
	LOMUX_SOC_DAPM_ENUM_EXT(2),
	LOMUX_SOC_DAPM_ENUM_EXT(3),
	LOMUX_SOC_DAPM_ENUM_EXT(4),
	LOMUX_SOC_DAPM_ENUM_EXT(5),
	LOMUX_SOC_DAPM_ENUM_EXT(6),
	LOMUX_SOC_DAPM_ENUM_EXT(7),
	LOMUX_SOC_DAPM_ENUM_EXT(8),
	LOMUX_SOC_DAPM_ENUM_EXT(9),
	LOMUX_SOC_DAPM_ENUM_EXT(10),
	LOMUX_SOC_DAPM_ENUM_EXT(11),
	LOMUX_SOC_DAPM_ENUM_EXT(12),
	LOMUX_SOC_DAPM_ENUM_EXT(13),
	LOMUX_SOC_DAPM_ENUM_EXT(14),
	LOMUX_SOC_DAPM_ENUM_EXT(15),
#undef LOMUX_SOC_DAPM_ENUM_EXT
};

static int ingenic_as_dsp_widget_event(struct snd_soc_dapm_widget *widget,
                                       struct snd_kcontrol *kcontrol, int event)
{
	if (event == SND_SOC_DAPM_PRE_PMU) {
		struct snd_soc_dapm_context *dapm = widget->dapm;
		struct snd_soc_component *component = snd_soc_dapm_to_component(dapm);
		struct ingenic_as_dsp *as_dsp = (struct ingenic_as_dsp *)snd_soc_component_get_drvdata(component);
		int id, is_out = true;

		struct snd_soc_dapm_path *p;
		struct ingenic_as_dsp_enum *e;

		if (widget->id == snd_soc_dapm_aif_in) {
			is_out = false;
		}
		id = (int)widget->priv;

		snd_soc_dapm_widget_for_each_sink_path(widget, p) {

			if (!p->connect) {
				continue;
			}

			if (p->sink->id != snd_soc_dapm_mux) {
				continue;
			}

			e = &lomux_enum[(int)p->sink->priv];

			if (e->value != id + 1) {
				e->value = id + 1;

				ingenic_dsp_free_timesolt(as_dsp, e);
				ingenic_dsp_request_timesolt(as_dsp, e, id + 1);
			}
		}

		DSP_DEBUG_MSG("flush %s fifo %d \n", is_out ? "tx" : "rx", id);
		regmap_update_bits(as_dsp->regmap, BFCR0, BFCR0_DEV_TF_MSK | BFCR0_DEV_RF_MSK,
		                   is_out ? BFCR0_DEV_TF(id) : BFCR0_DEV_RF(id));
	}
	return 0;
}

static const char *const dai_name[] = {
	"DMA0", "DMA1", "DMA2", "DMA3", "DMA4",
	"DMA5", "DMA6", "DMA7", "DMA8", "DMA9"
};

static int ingenic_as_dsp_probe(struct platform_device *pdev)
{
	int ret, i, num_widgets, num_dais, id;
	struct ingenic_as_dsp *as_dsp;
	struct resource *res;
	struct property *prop;
	const __be32 *p;
	struct snd_soc_dapm_widget *dapm_widgets;
	struct snd_soc_dai_driver *fe_dais;
	struct regmap_config regmap_config = {
		.reg_bits = 32,
		.reg_stride = 4,
		.val_bits = 32,
		.cache_type = REGCACHE_NONE,
	};

	as_dsp = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_as_dsp), GFP_KERNEL);
	if (!as_dsp) {
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	as_dsp->base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(as_dsp->base_addr)) {
		return PTR_ERR(as_dsp->base_addr);
	}
	regmap_config.max_register = resource_size(res) - 0x4;
	as_dsp->regmap = devm_regmap_init_mmio(&pdev->dev, as_dsp->base_addr,
	                                       &regmap_config);
	if (IS_ERR(as_dsp->regmap)) {
		return PTR_ERR(as_dsp->regmap);
	}

	as_dsp->dev = &pdev->dev;
	mutex_init(&as_dsp->dapm_mutex);
	bitmap_zero(as_dsp->slot_bitmap, SLOT_NUM);

	/*init dsp port*/
	for (i = 0; i < LI_PORT_MAX_NUM; i++) {
		as_dsp->port[LI_DEVID_TO_IDX(i)].dev_id = i;
		INIT_LIST_HEAD(&as_dsp->port[LI_DEVID_TO_IDX(i)].head);
	}

	for (i = 0; i < LO_PORT_MAX_NUM; i++) {
		as_dsp->port[LO_DEVID_TO_IDX(i)].dev_id = i;
		as_dsp->port[LO_DEVID_TO_IDX(i)].is_out = true;
		INIT_LIST_HEAD(&as_dsp->port[LO_DEVID_TO_IDX(i)].node);
	}

	as_dsp->num_li_ports = of_property_count_u32_elems(pdev->dev.of_node, "ingenic,li-port");
	if (as_dsp->num_li_ports <= 0 || as_dsp->num_li_ports > LI_PORT_MAX_NUM) {
		return -EINVAL;
	}
	as_dsp->num_lo_ports = of_property_count_u32_elems(pdev->dev.of_node, "ingenic,lo-port");
	if (as_dsp->num_lo_ports <= 0 || as_dsp->num_lo_ports > LO_PORT_MAX_NUM) {
		return -EINVAL;
	}

	/*init widgets*/
	num_widgets = as_dsp->num_li_ports + as_dsp->num_lo_ports;
	dapm_widgets = devm_kzalloc(&pdev->dev,
	                            sizeof(struct snd_soc_dapm_widget) * num_widgets,
	                            GFP_KERNEL);
	i = 0;
	of_property_for_each_u32(pdev->dev.of_node, "ingenic,li-port", prop, p, id) {
		dapm_widgets[i].id = snd_soc_dapm_aif_in;
		dapm_widgets[i].name = LI_PORT_NAME(id);
		dapm_widgets[i].reg = SND_SOC_NOPM;
		dapm_widgets[i].event = ingenic_as_dsp_widget_event;
		dapm_widgets[i].priv = (void *)id;
		dapm_widgets[i++].event_flags = SND_SOC_DAPM_PRE_PMU;
		as_dsp->port[LI_DEVID_TO_IDX(id)].vaild = true;
	}
	of_property_for_each_u32(pdev->dev.of_node, "ingenic,lo-port", prop, p, id) {
		dapm_widgets[i].id = snd_soc_dapm_mux;
		dapm_widgets[i].name = LO_PORT_NAME(id);
		dapm_widgets[i].reg = SND_SOC_NOPM;
#if 0
		dapm_widgets[i].kcontrol_news = &lomux_controls[id];
		dapm_widgets[i].num_kcontrols = 1;
#endif
		dapm_widgets[i].event = ingenic_as_dsp_widget_event;
		dapm_widgets[i].priv = (void *)id;
		dapm_widgets[i++].event_flags = SND_SOC_DAPM_PRE_PMU;
		as_dsp->port[LO_DEVID_TO_IDX(id)].vaild = true;
	}

#if 0

	/*init routes*/
	num_routes = of_property_count_strings(pdev->dev.of_node, "ingenic,routes");
	num_routes /= 2;
	if (num_routes <= 0 || num_routes & 0x1) {
		full_path = true;
		num_routes = as_dsp->num_li_ports * as_dsp->num_lo_ports;
	}

	dapm_routes = devm_kzalloc(&pdev->dev,
	                           sizeof(struct snd_soc_dapm_route) * num_routes,
	                           GFP_KERNEL);

	if (!dapm_routes) {
		return -ENOMEM;
	}

	if (full_path) {
		struct property *prop1;
		const __be32 *p1;
		int id1;
		i = 0;
		of_property_for_each_u32(pdev->dev.of_node, "ingenic,lo-port", prop, p, id) {
			of_property_for_each_u32(pdev->dev.of_node, "ingenic,li-port", prop1, p1, id1) {
				dapm_routes[i].source = dapm_routes[i].control = LI_PORT_NAME(id1);
				dapm_routes[i++].sink = LO_PORT_NAME(id);
			}
		}
	} else {
		for (i = 0; i < num_routes; i++) {
			of_property_read_string_index(pdev->dev.of_node, "ingenic,routes",
			                              2 * i, &dapm_routes[i].sink);
			of_property_read_string_index(pdev->dev.of_node, "ingenic,routes",
			                              (2 * i) + 1, &dapm_routes[i].source);
			dapm_routes[i].control = dapm_routes[i].source;
		}
	}
#endif

	/*init component driver*/
	as_dsp->cmpnt_drv.name = "ingenic-as-dsp";
	as_dsp->cmpnt_drv.dapm_widgets = dapm_widgets;
	as_dsp->cmpnt_drv.num_dapm_widgets = num_widgets;
	//  as_dsp->cmpnt_drv.dapm_routes = dapm_routes;
	//  as_dsp->cmpnt_drv.num_dapm_routes = num_routes;
	platform_set_drvdata(pdev, as_dsp);

	/* init global variables */
	as_dsp_t = as_dsp;

	/*init dais*/
	if (of_property_read_u32(pdev->dev.of_node, "ingenic,num-dais", &num_dais)) {
		return -ENODEV;
	}

	if (of_property_read_u32(pdev->dev.of_node, "ingenic,cap-dai-bm", &as_dsp->cap_dai_bitmsk)) {
		return -ENODEV;
	}

	fe_dais =  devm_kzalloc(&pdev->dev, sizeof(*fe_dais) * num_dais, GFP_KERNEL);
	if (!fe_dais) {
		return -ENOMEM;
	}

	for (i = 0; i < num_dais; i++) {
		fe_dais[i].name = dai_name[i];
		fe_dais[i].id = i;
		if ((BIT(i) & as_dsp->cap_dai_bitmsk)) {
			fe_dais[i].capture.stream_name = dai_name[i];
			fe_dais[i].capture.channels_min = 1;
			fe_dais[i].capture.channels_max = 12;
			fe_dais[i].capture.rates = SNDRV_PCM_RATE_8000_192000;
			fe_dais[i].capture.formats = ~0ULL;
		} else {
			fe_dais[i].playback.stream_name = dai_name[i];
			fe_dais[i].playback.channels_min = 1;
			fe_dais[i].playback.channels_max = 8;
			fe_dais[i].playback.rates = SNDRV_PCM_RATE_8000_192000;
			fe_dais[i].playback.formats = ~0ULL;
		}
	}

	regmap_write(as_dsp->regmap, BFCR1, BFCR1_DEV_DBE_MSK);
	regmap_write(as_dsp->regmap, BFCR2, BFCR2_DEV_DBE_MSK);

	ret = snd_soc_register_component(&pdev->dev,
	                                 &as_dsp->cmpnt_drv,
	                                 fe_dais, num_dais);
	if (ret) {
		return ret;
	}

	dev_info(&pdev->dev, "probe success!!\n");
	return 0;
}

static int ingenic_as_dsp_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id ingenic_as_dsp_match_table[] = {
	{ .compatible = "ingenic,as-dsp", },
	{ }
};
MODULE_DEVICE_TABLE(of, ingenic_as_dsp_match_table);

#ifdef CONFIG_PM
static int ingenic_as_dsp_runtime_suspend(struct device *dev)
{
	return 0;
}

static int ingenic_as_dsp_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops ingenic_as_dsp_pm_ops = {
	SET_RUNTIME_PM_OPS(ingenic_as_dsp_runtime_suspend,
	                   ingenic_as_dsp_runtime_resume, NULL)
};

static struct platform_driver ingenic_as_dsp_platform_driver = {
	.driver = {
		.name = "as-dsp",
		.of_match_table = ingenic_as_dsp_match_table,
		.pm = &ingenic_as_dsp_pm_ops,
	},
	.probe = ingenic_as_dsp_probe,
	.remove = ingenic_as_dsp_remove,
};

int ingenic_as_dsp_platform_driver_modinit(void)
{
	return platform_driver_register(&ingenic_as_dsp_platform_driver);
}
EXPORT_SYMBOL(ingenic_as_dsp_platform_driver_modinit);

void ingenic_as_dsp_platform_driver_exit(void)
{
	platform_driver_unregister(&ingenic_as_dsp_platform_driver);
}
EXPORT_SYMBOL(ingenic_as_dsp_platform_driver_exit);

/* module_platform_driver(ingenic_as_dsp_platform_driver); */

MODULE_AUTHOR("cli <chen.li@ingenic.com>");
MODULE_DESCRIPTION("Ingenic AS DSP SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-as-dsp");
