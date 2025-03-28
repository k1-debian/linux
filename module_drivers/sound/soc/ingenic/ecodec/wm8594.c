/*
 *  sound/soc/ingenic/ecodec/wm8594.c
 *  ALSA SoC driver for wm8594 ADC
 *
 *  Copyright 2017 Ingenic Semiconductor Co.,Ltd
 *  wqshao <wangquan.shao@ingenic.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Only for x2000 FPGA Audio test.
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include "wm8594.h"

#define CODEC_NAME  "wm8594"

#define wm8594_reset(c) wm8594_write(c, WM8594_RESET, 0)

struct wm8594_priv {
	struct regmap *regmap;
	bool powered;
	struct gpio_desc *reset_gpiod;
	struct gpio_desc *mute_gpiod;
	struct gpio_desc *pwdn_gpiod;
};

#define SND_SOC_WM8594_SHARE_CLOCK_MASK     0xf0000
#define SND_SOC_WM8594_SHARE_CLOCK_PLAYBACK (1 << 16)
#define SND_SOC_WM8594_SHARE_CLOCK_RECORD   (2 << 16)

static const struct reg_default wm8594_reg_defaults[] = {
	{ 0x00, 0x8594 },
	{ 0x01, 0x0004 },
	{ 0x02, 0x008a },
	{ 0x03, 0x0000 },
	{ 0x04, 0x0000 },
	{ 0x05, 0x00c8 },
	{ 0x06, 0x00c8 },
	{ 0x07, 0x008a },
	{ 0x08, 0x0000 },
	{ 0x09, 0x0000 },
	{ 0x0a, 0x00c8 },
	{ 0x0b, 0x00c8 },
	{ 0x0c, 0x0000 },
	{ 0x0d, 0x200a },
	{ 0x0e, 0x0000 },
	{ 0x0f, 0x0000 },
	{ 0x10, 0x00c3 },
	{ 0x11, 0x00c3 },
	{ 0x13, 0x000c },
	{ 0x14, 0x000c },
	{ 0x15, 0x000c },
	{ 0x16, 0x000c },
	{ 0x17, 0x000c },
	{ 0x18, 0x000c },
	{ 0x19, 0x0003 },
	{ 0x1a, 0x007e },
	{ 0x1b, 0x0048 },
	{ 0x1c, 0x0000 },
	{ 0x1d, 0x0000 },
	{ 0x1e, 0x0008 },
	{ 0x1f, 0x0000 },
	{ 0x20, 0x0088 },
	{ 0x21, 0x0163 },
	{ 0x22, 0x0040 },
	{ 0x23, 0x0010 },
	{ 0x24, 0x0002 },
};

static const struct reg_default wm8594_reg_init[] = {
	{WM8594_DAC1_CTRL1, 0x008e}, /*
                      * DAC1 cross enable
                      * 32 bits i2s format
                      */
	{WM8594_DAC1_CTRL2, 0x001b}, /*
                      * MCLK = 256 * Fs
                      * BLCK = 64  * Fs
                      */
	{WM8594_ADC_CTRL1,  0x200a}, /*
                      * ADC cross enable
                      * 32 bits i2s format
                      */
	{WM8594_DAC2_CTRL1, 0x008e}, /*
                      * DAC2 cross enable
                      * 32 bits i2s format
                      */
	{WM8594_DAC2_CTRL2, 0x001b}, /*
                      * MCLK = 256 * Fs
                      * BLCK = 64  * Fs
                      */
	{WM8594_INPUT_CTRL1,    0x09a9},
	{WM8594_INPUT_CTRL2,    0x0a9a}, /*
                      * connect PGA1 and PGA2
                      * to DAC1
                      */
	{WM8594_INPUT_CTRL3,    0x0480}, /* connect ADC to VIN1 */
};

static unsigned int wm8594_read(struct snd_soc_component *component,
                                unsigned int reg)
{
	return snd_soc_component_read(component, reg);
}

static int wm8594_write(struct snd_soc_component *component,
                        unsigned int reg,
                        unsigned int value)
{
	int ret = 0;
	ret = snd_soc_component_update_bits(component, reg, 0xffff, value);
	if (ret >= 0) {
		dev_dbg(component->dev, "%s: reg = 0x%x ; value = 0x%x (change %x)success\n",
		        __func__, reg, value, ret);
		ret = 0;
	} else {
		dev_err(component->dev, "%s: reg = 0x%x ; value = 0x%x error\n",
		        __func__, reg, value);
		ret = -EIO;
	}

	return ret;
}

static int wm8594_vol_update(struct snd_kcontrol *kcontrol,
                             struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	/*  struct snd_soc_codec *codec = component->codec;*/
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	int err;
	bool type_2r = 0;
	unsigned int val2 = 0;
	unsigned int val, val_mask;

	val = (ucontrol->value.integer.value[0] & mask);
	if (invert) {
		val = max - val;
	}
	val_mask = mask << shift;
	val = val << shift;
	if (snd_soc_volsw_is_stereo(mc)) {
		val2 = (ucontrol->value.integer.value[1] & mask);
		if (invert) {
			val2 = max - val2;
		}
		if (reg == reg2) {
			val_mask |= mask << rshift;
			val |= val2 << rshift;
		} else {
			val2 = val2 << shift;
			type_2r = 1;
		}
	}

	err = snd_soc_component_update_bits(component,
	                                    reg,
	                                    val_mask | 0x100,
	                                    val | 0x100);
	if (err < 0) {
		return err;
	}

	if (type_2r)
		err = snd_soc_component_update_bits(component,
		                                    reg2,
		                                    val_mask | 0x100,
		                                    val2 | 0x100);

	return err;
}

static int wm8594_adc_input_get_switch(struct snd_kcontrol *kcontrol,
                                       struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_component *component = snd_soc_dapm_to_component(dapm);
	int ret = 0;
	u16 sel = 0;
	sel = snd_soc_component_read(component, WM8594_INPUT_CTRL3);
	if (sel < 0) {
		return -EIO;
	}
	switch (sel & 0xf) {
		case 0x0:
			/* VIN1L */
			ucontrol->value.enumerated.item[0] = 0;
			break;
		case 0x1:
			/* VIN2L */
			ucontrol->value.enumerated.item[0] = 1;
			break;
		case 0x2:
			/* VIN3L */
			ucontrol->value.enumerated.item[0] = 2;
			break;
		case 0x3:
			/* VIN4L */
			ucontrol->value.enumerated.item[0] = 3;
			break;
		case 0x4:
			/* VIN5L */
			ucontrol->value.enumerated.item[0] = 4;
			break;
		default:
			ret = -EINVAL;
			dev_err(component->dev, "%s: unexpected control value\n", __func__);
			break;
	};
	return ret;
}

static int wm8594_adc_input_set_switch(struct snd_kcontrol *kcontrol,
                                       struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_component *component = snd_soc_dapm_to_component(dapm);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	int ret = 0;
	u16 sel = 0;

	switch (item[0]) {
		case 0:
			/* VIN1 */
			sel = 0x80;
			break;
		case 1:
			/* VIN2 */
			sel = 0x91;
			break;
		case 2:
			/* VIN3 */
			sel = 0xa2;
			break;
		case 3:
			/* VIN4 */
			sel = 0xb3;
			break;
		case 4:
			/* VIN5 */
			sel = 0xc4;
			break;
		default:
			ret = -EINVAL;
			dev_err(component->dev, "%s: unexpected control value\n", __func__);
			goto error;
			break;
	}

	ret = snd_soc_component_update_bits(component, WM8594_INPUT_CTRL3, 0xff, sel);
	if (ret < 0) {
		goto error;
	}
	if (ret) {
		snd_soc_dapm_mux_update_power(dapm, kcontrol, item[0], e, NULL);
	}
	ret = 0;
error:
	return ret;
}

static int wm8594_pga_sel_get_switch(struct snd_kcontrol *kcontrol,
                                     struct snd_ctl_elem_value *ucontrol,
                                     u16 reg_left, u16 shift_left,
                                     u16 reg_right, u16 shift_right)
{
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_component *component = snd_soc_dapm_to_component(dapm);
	int ret = 0;
	u16 sel = 0;
	sel = snd_soc_component_read(component, reg_left);
	if (sel < 0) {
		sel = 0;
	}
	sel >>= shift_left;
	sel &= 0xf;

	switch (sel) {
		case 0x0 ... 0x5:
			ucontrol->value.enumerated.item[0] = sel;
			break;
		case 0x9:
			ucontrol->value.enumerated.item[0] = 6;
			break;
		case 0xb:
			ucontrol->value.enumerated.item[0] = 7;
			break;
		default:
			ret = -EINVAL;
			dev_err(component->dev, "%s: unexpected control value\n", __func__);
			break;
	};

	return ret;
}

static int wm8594_pga_sel_set_switch(struct snd_kcontrol *kcontrol,
                                     struct snd_ctl_elem_value *ucontrol,
                                     u16 reg_left, u16 shift_left,
                                     u16 reg_right, u16 shift_right)
{
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_component *component = snd_soc_dapm_to_component(dapm);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	int ret = 0, change = 0;
	u16 lsel = 0, rsel = 0, lmsk = 0xf << shift_left,
	    rmsk = 0xf << shift_right;

	switch (item[0]) {
		case 0: /* no input */
			break;
		case 1 ... 5: /* VIN1 - VIN5 */
			lsel = item[0] << shift_left;
			rsel = item[0] << shift_right;
			break;
		case 6: /* DAC1 */
			lsel = 0x9 << shift_left;
			rsel = 0xa << shift_right;
			break;
		case 7:
			lsel = 0xb << shift_left;
			rsel = 0xc << shift_right;
			break;
		default:
			ret = -EINVAL;
			dev_err(component->dev, "%s: unexpected control value\n", __func__);
			goto error;
	}

	if (reg_left == reg_right) {
		lsel |= rsel;
		lmsk |= rmsk;
		ret = snd_soc_component_update_bits(component, reg_left, lmsk, lsel);
		if (ret < 0) {
			goto error;
		}
		change = ret;
	} else {
		ret = snd_soc_component_update_bits(component, reg_left, lmsk, lsel);
		if (ret < 0) {
			goto error;
		}
		change = ret;
		ret = snd_soc_component_update_bits(component, reg_right, rmsk, rsel);
		if (ret < 0) {
			goto error;
		}
		change |= ret;
	}

	if (change) {
		snd_soc_dapm_mux_update_power(dapm, kcontrol, item[0], e, NULL);
	}
error:
	return ret;
}

static const DECLARE_TLV_DB_SCALE(dac_tlv, -10000, 50, 0);
static const DECLARE_TLV_DB_SCALE(adc_tlv, -9750, 50, 0);
static const DECLARE_TLV_DB_SCALE(pga_tlv, -7400, 50, 0xa0);

static const struct snd_kcontrol_new wm8594_snd_controls[] = {
	SOC_DOUBLE_R_EXT_TLV("WM8594 DAC1 Playback Volume",
	                     WM8594_DAC1L_VOL, WM8594_DAC1R_VOL, 0, 224, 0,
	                     snd_soc_get_volsw, wm8594_vol_update, dac_tlv),
	SOC_SINGLE("WM8594 DAC1 Mute Switch", WM8594_DAC1_CTRL1, 9, 1, 1),

	SOC_DOUBLE_R_EXT_TLV("WM8594 DAC2 Playback Volume",
	                     WM8594_DAC2L_VOL, WM8594_DAC2R_VOL, 0, 224, 0,
	                     snd_soc_get_volsw, wm8594_vol_update, dac_tlv),
	SOC_SINGLE("WM8594 DAC2 Mute Switch", WM8594_DAC2_CTRL1, 9, 1, 1),

	SOC_DOUBLE_R_EXT_TLV("WM8594 ADC Capture Volume",
	                     WM8594_ADCL_VOL, WM8594_ADCR_VOL, 0, 255, 0,
	                     snd_soc_get_volsw, wm8594_vol_update, adc_tlv),

	SOC_DOUBLE_R_EXT_TLV("WM8594 PGA1 Volume",
	                     WM8594_PGA1L_VOL, WM8594_PGA1R_VOL, 0, 160, 1,
	                     snd_soc_get_volsw, wm8594_vol_update, pga_tlv),
	SOC_DOUBLE("WM8594 PGA1 Mute Switch", WM8594_PGA_CTRL2, 1, 2, 1, 1),

	SOC_DOUBLE_R_EXT_TLV("WM8594 PGA2 Volume",
	                     WM8594_PGA2L_VOL, WM8594_PGA2R_VOL, 0, 160, 1,
	                     snd_soc_get_volsw, wm8594_vol_update, pga_tlv),
	SOC_DOUBLE("WM8594 PGA2 Mute Switch", WM8594_PGA_CTRL2, 3, 4, 1, 1),

	SOC_DOUBLE_R_EXT_TLV("WM8594 PGA3 Volume",
	                     WM8594_PGA3L_VOL, WM8594_PGA3R_VOL, 0, 160, 1,
	                     snd_soc_get_volsw, wm8594_vol_update, pga_tlv),
	SOC_DOUBLE("WM8594 PGA3 Mute Switch", WM8594_PGA_CTRL2, 5, 4, 1, 1),

	SOC_SINGLE("WM8594 PGA Mute All Switch", WM8594_PGA_CTRL2, 0, 1, 1),

	SOC_SINGLE("WM8594 ADC Gain Control", WM8594_INPUT_CTRL3, 8, 3, 0),
};

static int wm8594_pcm_hw_params(struct snd_pcm_substream *substream,
                                struct snd_pcm_hw_params *params,
                                struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;

	u16 dac1_ctrl1 = wm8594_read(component, WM8594_DAC1_CTRL1) & ~0xc;
	u16 dac1_ctrl2 = wm8594_read(component, WM8594_DAC1_CTRL2) & ~0x3f;
	u16 dac2_ctrl1 = wm8594_read(component, WM8594_DAC2_CTRL1) & ~0xc;
	u16 dac2_ctrl2 = wm8594_read(component, WM8594_DAC2_CTRL2) & ~0x3f;

	u16 adc_ctrl1 = wm8594_read(component, WM8594_ADC_CTRL1) & ~0xc;
	u16 adc_ctrl2 = wm8594_read(component, WM8594_ADC_CTRL2) & ~0x3f;

	u16 add = wm8594_read(component, WM8594_ADD_CTRL1) & ~0x70;

	switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			break;
		case SNDRV_PCM_FORMAT_S20_3LE:
			dac1_ctrl1 |= 0x0004;
			dac2_ctrl1 |= 0x0004;

			adc_ctrl1 |= 0x0004;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			dac1_ctrl1 |= 0x0008;
			dac2_ctrl1 |= 0x0008;

			adc_ctrl1 |= 0x0008;
			break;
		case SNDRV_PCM_FORMAT_S32_LE:
			dac1_ctrl1 |= 0x000c;
			dac2_ctrl1 |= 0x000c;

			adc_ctrl1 |= 0x000c;
			break;
	}

	/* set sample rate */
	switch (params_rate(params)) {
		case SNDRV_PCM_RATE_32000:
			break;
		case SNDRV_PCM_RATE_44100:
			add |= 0x10;
			break;
		case SNDRV_PCM_RATE_88200:
			add |= 0x30;
			break;
		case SNDRV_PCM_RATE_96000:
			add |= 0x40;
			break;
		case SNDRV_PCM_RATE_176400:
			add |= 0x50;
			break;
		case SNDRV_PCM_RATE_192000:
			add |= 0x60;
			break;
		/* 48000 is the default value */
		case SNDRV_PCM_RATE_48000:
		default:
			add |= 0x20;
			break;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		switch (params_rate(params)) {
			case 32000:
				dac1_ctrl2 |= (6 << CTRL2_SR) | (3 << CTRL2_BCLKDIV); /*768*/
				dac2_ctrl2 |= (6 << CTRL2_SR) | (3 << CTRL2_BCLKDIV);
				break;
			case 44100:
				dac1_ctrl2 |= (0 << CTRL2_SR) | (3 << CTRL2_BCLKDIV); /*512 not sure*/
				dac2_ctrl2 |= (0 << CTRL2_SR) | (3 << CTRL2_BCLKDIV);
				break;
				break;
			case 96000:
				dac1_ctrl2 |= (3 << CTRL2_SR) | (3 << CTRL2_BCLKDIV); /*256*/
				dac2_ctrl2 |= (3 << CTRL2_SR) | (3 << CTRL2_BCLKDIV);
				break;
			case 48000:
				dac1_ctrl2 |= (5 << CTRL2_SR) | (3 << CTRL2_BCLKDIV); /*512*/
				dac2_ctrl2 |= (5 << CTRL2_SR) | (3 << CTRL2_BCLKDIV);
				dac1_ctrl2 |= (0 << CTRL2_SR) | (3 << CTRL2_BCLKDIV);
				dac2_ctrl2 |= (0 << CTRL2_SR) | (3 << CTRL2_BCLKDIV);
				break;
			default:
				dac1_ctrl2 |= (0 << CTRL2_SR) | (3 << CTRL2_BCLKDIV);
				dac2_ctrl2 |= (0 << CTRL2_SR) | (3 << CTRL2_BCLKDIV);
				break;
		}
	} else {
		switch (params_rate(params)) {
			case 32000:
				adc_ctrl2 |= (6 << CTRL2_SR) | (3 << CTRL2_BCLKDIV);
				break;
			case 44100:
				adc_ctrl2 = (5 << CTRL2_SR) | (3 << CTRL2_BCLKDIV);
				break;
				break;
			case 96000:
				adc_ctrl2 = (3 << CTRL2_SR) | (3 << CTRL2_BCLKDIV);
				break;
			case 48000:
			default:
				adc_ctrl2 = (5 << CTRL2_SR) | (3 << CTRL2_BCLKDIV);
				break;
		}
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		wm8594_write(component, WM8594_DAC1_CTRL1, dac1_ctrl1);
		wm8594_write(component, WM8594_DAC1_CTRL2, dac1_ctrl2);
		wm8594_write(component, WM8594_DAC2_CTRL1, dac2_ctrl1);
		wm8594_write(component, WM8594_DAC2_CTRL2, dac2_ctrl2);
	} else {
		wm8594_write(component, WM8594_ADC_CTRL1, adc_ctrl1);
		wm8594_write(component, WM8594_ADC_CTRL2, adc_ctrl2);
	}
	wm8594_write(component, WM8594_ADD_CTRL1, add);
	return 0;
}

static int wm8594_set_dai_fmt(struct snd_soc_dai *component_dai,
                              unsigned int fmt)
{
	/*  struct snd_soc_component *component = component_dai->component;*/
	struct snd_soc_component *component = component_dai->component;

	u16 dac1_ctrl1 = wm8594_read(component, WM8594_DAC1_CTRL1)
	                 & ~0x33;
	u16 dac1_ctrl3 = wm8594_read(component, WM8594_DAC1_CTRL3) & ~0x1;

	u16 dac2_ctrl1 = wm8594_read(component, WM8594_DAC2_CTRL1)
	                 & ~0x33;
	u16 dac2_ctrl3 = wm8594_read(component, WM8594_DAC2_CTRL3) & ~0x1;

	u16 adc_ctrl1 = wm8594_read(component, WM8594_ADC_CTRL1) & ~0x33;
	u16 adc_ctrl3 = wm8594_read(component, WM8594_ADC_CTRL3) & ~0x1;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBM_CFM:
#ifndef CONFIG_SND_ASOC_INGENIC_SHARE_CLK
			dac1_ctrl3 |= 0x0001;
			dac2_ctrl3 |= 0x0001;
			adc_ctrl3 |= 0x0001;
#else
			switch (fmt & SND_SOC_WM8594_SHARE_CLOCK_MASK) {
				case SND_SOC_WM8594_SHARE_CLOCK_PLAYBACK:
					dac1_ctrl3 |= 0x0001;
					dac2_ctrl3 |= 0x0001;
					break;
				case SND_SOC_WM8594_SHARE_CLOCK_RECORD:
					adc_ctrl3 |= 0x0001;
					break;
				default:
					dac1_ctrl3 |= 0x0001;
					dac2_ctrl3 |= 0x0001;
					break;
			}
#endif
			break;

		case SND_SOC_DAIFMT_CBS_CFM:
			break;

		case SND_SOC_DAIFMT_CBS_CFS:
			break;
		default:
			return -EINVAL;
	}
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			dac1_ctrl1 |= 0x0002;
			dac2_ctrl1 |= 0x0002;
			adc_ctrl1 |= 0x0002;

			break;

		case SND_SOC_DAIFMT_RIGHT_J:

			break;

		case SND_SOC_DAIFMT_LEFT_J:
			dac1_ctrl1 |= 0x0001;
			dac2_ctrl1 |= 0x0001;
			adc_ctrl1 |= 0x0001;

			break;

		case SND_SOC_DAIFMT_DSP_A:
			dac1_ctrl1 |= 0x0003;
			dac2_ctrl1 |= 0x0003;
			adc_ctrl1 |= 0x0003;

			break;

		default:

			return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			break;

		case SND_SOC_DAIFMT_IB_IF:
			dac1_ctrl1 |= 0x0030;
			dac2_ctrl1 |= 0x0030;
			adc_ctrl1 |= 0x0030;
			break;

		case SND_SOC_DAIFMT_IB_NF:
			dac1_ctrl1 |= 0x0010;
			dac2_ctrl1 |= 0x0010;
			adc_ctrl1 |= 0x0010;
			break;

		case SND_SOC_DAIFMT_NB_IF:
			dac1_ctrl1 |= 0x0020;
			dac2_ctrl1 |= 0x0020;
			adc_ctrl1 |= 0x0020;
			break;

		default:
			return -EINVAL;
	}
	wm8594_write(component, WM8594_DAC1_CTRL1, dac1_ctrl1);
	wm8594_write(component, WM8594_DAC1_CTRL3, dac1_ctrl3);

	wm8594_write(component, WM8594_DAC2_CTRL1, dac2_ctrl1);
	wm8594_write(component, WM8594_DAC2_CTRL3, dac2_ctrl3);

	wm8594_write(component, WM8594_ADC_CTRL1, adc_ctrl1);
	wm8594_write(component, WM8594_ADC_CTRL3, adc_ctrl3);
	return 0;
}

#if 0
static int wm8594_mute(struct snd_soc_dai *component_dai, int mute)
{
	return 0;
}
#endif

static struct snd_soc_dai_ops wm8594_dai_ops = {
	.hw_params    = wm8594_pcm_hw_params,
	.set_fmt      = wm8594_set_dai_fmt,
};

static int wm8594_power_up(struct snd_soc_component *component)
{
	struct wm8594_priv *wm8594 = snd_soc_component_get_drvdata(component);
	u16 bias = 0;
	u16 dac1 = wm8594_read(component, WM8594_DAC1_CTRL1) & ~0x100;
	u16 dac2 = wm8594_read(component, WM8594_DAC2_CTRL1) & ~0x100;
	u16 adc = wm8594_read(component, WM8594_ADC_CTRL1) & ~0x40;

	if (wm8594->powered) {
		return 0;
	}
	wm8594->powered = true;

	/*
	 * set up initial biases
	 * SOFT_ST | FAST_EN | POBCTRL | BUFIO_EN
	 */
	bias |= 0x1d;
	wm8594_write(component, WM8594_BIAS, bias);

	/*
	 * enable output drivers to allow the AC coupling capacitors at the
	 * output charge to be precharged to DACVMID
	 * VOUTxL_EN | VOUTxR_EN
	 */
	wm8594_write(component, WM8594_OUTPUT_CTRL3, 0x1fc0);

	/*
	 * enable DACVMID, 750k selected for pop reduction
	 * VMID_SEL = 10
	 */
	bias |= 0x80;
	wm8594_write(component, WM8594_BIAS, bias);

	/*
	 * wait until DACVMID has fully charged
	 */

	mdelay(1500);

	/*
	 * enable master bias
	 * BIAS_EN
	 */
	bias |= 0x20;
	wm8594_write(component, WM8594_BIAS, bias);

	/*
	 * switch output drivers to use the master bias instead of the
	 * power-up (fast) bias
	 * POBCTRL = 0
	 */
	bias &= ~0x1;
	wm8594_write(component, WM8594_BIAS, bias);

	/*
	 * enable all function required for use:
	 * - enable all inputs
	 * - enable component
	 * - enable dac1, dac2 and adc
	 */
	wm8594_write(component, WM8594_INPUT_CTRL4, 0xff);
	wm8594_write(component, WM8594_ENABLE, 0x1);
	dac1 |= 0x100;
	wm8594_write(component, WM8594_DAC1_CTRL1, dac1);
	dac2 |= 0x100;
	wm8594_write(component, WM8594_DAC2_CTRL1, dac2);
	adc |= 0x40;
	wm8594_write(component, WM8594_ADC_CTRL1, adc);

	/*
	 * unmute PGAs and switch DACVMID to 75k for normal operation
	 */
	bias &= ~0xc0;
	bias |= 0x40;
	wm8594_write(component, WM8594_PGA_CTRL2, 0x0);

	return 0;
}

static int wm8594_power_down(struct snd_soc_component *component)
{
	struct wm8594_priv *wm8594 = snd_soc_component_get_drvdata(component);
	u16 bias;
	u16 dac1 = wm8594_read(component, WM8594_DAC1_CTRL1) & ~0x100;
	u16 dac2 = wm8594_read(component, WM8594_DAC2_CTRL1) & ~0x100;
	u16 adc = wm8594_read(component, WM8594_ADC_CTRL1) & ~0x40;
	u16 output = wm8594_read(component, WM8594_OUTPUT_CTRL3) & ~0x40;

	if (!wm8594->powered) {
		return 0;
	}
	wm8594->powered = false;
	/*
	 * mute all PGAs
	 */
	wm8594_write(component, WM8594_PGA_CTRL2, 0x7f);

	/*
	 * set biases for power down mode
	 * FAST_EN = 1
	 * VMID_SEL = 01
	 * BIAS_EN = 1
	 * BUFIO_EN = 1
	 * VMIDTOG = 0
	 * SOFT_ST = 1
	 */
	bias = 0x7c;
	wm8594_write(component, WM8594_BIAS, bias);

	/*
	 * switch outputs to fast bias
	 * POBCTRL = 1
	 */
	bias |= 0x2;
	wm8594_write(component, WM8594_BIAS, bias);

	/*
	 * power down
	 */
	wm8594_write(component, WM8594_DAC1_CTRL1, dac1);
	wm8594_write(component, WM8594_DAC2_CTRL1, dac2);
	wm8594_write(component, WM8594_ADC_CTRL1, adc);
	wm8594_write(component, WM8594_INPUT_CTRL4, 0x00);
	wm8594_write(component, WM8594_ENABLE, 0x0);

	/*
	 * power down VMID
	 * VMIDSEL = 00
	 */
	bias &= ~0xc0;
	wm8594_write(component, WM8594_BIAS, bias);

	/*
	 * wait for DACVMID to be discharged
	 */
	mdelay(1500);

	/*
	 * clamp outputs to ground
	 * power down outputs
	 */
	wm8594_write(component, WM8594_OUTPUT_CTRL3, output);
	wm8594_write(component, WM8594_OUTPUT_CTRL3, 0);

	/*
	 * disable remaining bias controls
	 */
	bias = 0;
	wm8594_write(component, WM8594_BIAS, bias);

	return 0;
}

static int wm8594_set_bias_level(struct snd_soc_component *component,
                                 enum snd_soc_bias_level level)
{
	switch (level) {
		case SND_SOC_BIAS_PREPARE:
			break;
		case SND_SOC_BIAS_STANDBY:
			wm8594_power_up(component);
			break;
		case SND_SOC_BIAS_ON:
			break;
		case SND_SOC_BIAS_OFF:
			wm8594_power_down(component);
			break;
	}
	return 0;
}

static int wm8594_probe(struct snd_soc_component *component)
{
	int i;
	int ret = 0;
	dev_dbg(component->dev, "WM8594 Audio component");

	ret = wm8594_reset(component);
	if (ret != 0) {
		dev_err(component->dev, "Failed to reset component: %d\n", ret);
		return ret;
	}

	/* initialize component register */
	for (i = 0; i < ARRAY_SIZE(wm8594_reg_init); i++) {
		wm8594_write(component, wm8594_reg_init[i].reg,
		             wm8594_reg_init[i].def);
	}

	return ret;
}

static int wm8594_resume(struct snd_soc_component *component)
{
	int i;
	unsigned int value;
	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(wm8594_reg_defaults); i++) {
		if (wm8594_reg_defaults[i].reg == WM8594_RESET) {
			continue;
		}
		value = snd_soc_component_read(component, wm8594_reg_defaults[i].reg);
		snd_soc_component_write(component, wm8594_reg_defaults[i].reg, value);
	}
	return 0;
}

#if 0
static struct regmap *wm8594_get_regmap(struct device *dev)
{
	struct wm8594_priv *wm8594 = dev_get_drvdata(dev);
	return wm8594->regmap;
}
#endif

static int wm8594_pga1_sel_get_switch(struct snd_kcontrol *kcontrol,
                                      struct snd_ctl_elem_value *ucontrol)
{
	return wm8594_pga_sel_get_switch(kcontrol, ucontrol,
	                                 WM8594_INPUT_CTRL1, 0,
	                                 WM8594_INPUT_CTRL1, 4);
}

static int wm8594_pga1_sel_set_switch(struct snd_kcontrol *kcontrol,
                                      struct snd_ctl_elem_value *ucontrol)
{
	return wm8594_pga_sel_set_switch(kcontrol, ucontrol,
	                                 WM8594_INPUT_CTRL1, 0,
	                                 WM8594_INPUT_CTRL1, 4);
}

static int wm8594_pga2_sel_get_switch(struct snd_kcontrol *kcontrol,
                                      struct snd_ctl_elem_value *ucontrol)
{
	return wm8594_pga_sel_get_switch(kcontrol, ucontrol,
	                                 WM8594_INPUT_CTRL1, 8,
	                                 WM8594_INPUT_CTRL2, 0);
}

static int wm8594_pga2_sel_set_switch(struct snd_kcontrol *kcontrol,
                                      struct snd_ctl_elem_value *ucontrol)
{
	return wm8594_pga_sel_set_switch(kcontrol, ucontrol,
	                                 WM8594_INPUT_CTRL1, 8,
	                                 WM8594_INPUT_CTRL2, 0);
}

static int wm8594_pga3_sel_get_switch(struct snd_kcontrol *kcontrol,
                                      struct snd_ctl_elem_value *ucontrol)
{
	return wm8594_pga_sel_get_switch(kcontrol, ucontrol,
	                                 WM8594_INPUT_CTRL2, 4,
	                                 WM8594_INPUT_CTRL2, 8);
}

static int wm8594_pga3_sel_set_switch(struct snd_kcontrol *kcontrol,
                                      struct snd_ctl_elem_value *ucontrol)
{
	return wm8594_pga_sel_set_switch(kcontrol, ucontrol,
	                                 WM8594_INPUT_CTRL2, 4,
	                                 WM8594_INPUT_CTRL2, 8);
}

static const char *const wm8594_adc_input_sel_text[] = {
	"VIN1", "VIN2", "VIN3", "VIN4", "VIN5"
};

static const char *const wm8594_pga_sel_text[] = {
	"No Input", "VIN1", "VIN2", "VIN3", "VIN4", "VIN5", "DAC1", "DAC2"
};

static const struct soc_enum wm8594_adc_input_sel_enum =
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(wm8594_adc_input_sel_text),
                        wm8594_adc_input_sel_text);

static const struct soc_enum wm8594_pga_sel_enum =
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(wm8594_pga_sel_text),
                        wm8594_pga_sel_text);

static const struct snd_kcontrol_new wm8594_pga1_controls =
    SOC_ENUM_EXT("PGA1 Input Selection", wm8594_pga_sel_enum,
                 wm8594_pga1_sel_get_switch, wm8594_pga1_sel_set_switch);

static const struct snd_kcontrol_new wm8594_pga2_controls =
    SOC_ENUM_EXT("PGA2 Input Selection", wm8594_pga_sel_enum,
                 wm8594_pga2_sel_get_switch, wm8594_pga2_sel_set_switch);

static const struct snd_kcontrol_new wm8594_pga3_controls =
    SOC_ENUM_EXT("PGA3 Input Selection", wm8594_pga_sel_enum,
                 wm8594_pga3_sel_get_switch, wm8594_pga3_sel_set_switch);

static const struct snd_kcontrol_new wm8594_adc_controls =
    SOC_ENUM_EXT("ADC Input Selection", wm8594_adc_input_sel_enum,
                 wm8594_adc_input_get_switch, wm8594_adc_input_set_switch);

static const struct snd_soc_dapm_widget wm8594_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC1", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC2", "Playback", SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MUX("VOUT1_MUX", SND_SOC_NOPM, 0, 0, &wm8594_pga1_controls),
	SND_SOC_DAPM_MUX("VOUT2_MUX", SND_SOC_NOPM, 0, 0, &wm8594_pga2_controls),
	SND_SOC_DAPM_MUX("VOUT3_MUX", SND_SOC_NOPM, 0, 0, &wm8594_pga3_controls),

	SND_SOC_DAPM_OUTPUT("VOUT1"),
	SND_SOC_DAPM_OUTPUT("VOUT2"),
	SND_SOC_DAPM_OUTPUT("VOUT3"),

	SND_SOC_DAPM_ADC("ADC", "Capture", SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MUX("VIN_MUX", SND_SOC_NOPM, 0, 0, &wm8594_adc_controls),

	SND_SOC_DAPM_INPUT("AIN1"),
	SND_SOC_DAPM_INPUT("AIN2"),
	SND_SOC_DAPM_INPUT("AIN3"),
	SND_SOC_DAPM_INPUT("AIN4"),
	SND_SOC_DAPM_INPUT("AIN5"),
};

static const struct snd_soc_dapm_route wm8594_dapm_routes[] = {
	{ "VOUT1", NULL, "VOUT1_MUX" },
	{ "VOUT2", NULL, "VOUT2_MUX" },
	{ "VOUT3", NULL, "VOUT3_MUX" },

	{ "VOUT1_MUX", NULL, "DAC1"},
	{ "VOUT2_MUX", NULL, "DAC1"},
	{ "VOUT3_MUX", NULL, "DAC1"},
	{ "VOUT1_MUX", NULL, "DAC2"},
	{ "VOUT2_MUX", NULL, "DAC2"},
	{ "VOUT3_MUX", NULL, "DAC2"},

	{ "VOUT1_MUX", "VIN1", "AIN1"},
	{ "VOUT1_MUX", "VIN2", "AIN2"},
	{ "VOUT1_MUX", "VIN3", "AIN3"},
	{ "VOUT1_MUX", "VIN4", "AIN4"},
	{ "VOUT1_MUX", "VIN5", "AIN5"},

	{ "VOUT2_MUX", "VIN1", "AIN1"},
	{ "VOUT2_MUX", "VIN2", "AIN2"},
	{ "VOUT2_MUX", "VIN3", "AIN3"},
	{ "VOUT2_MUX", "VIN4", "AIN4"},
	{ "VOUT2_MUX", "VIN5", "AIN5"},

	{ "VOUT3_MUX", "VIN1", "AIN1"},
	{ "VOUT3_MUX", "VIN2", "AIN2"},
	{ "VOUT3_MUX", "VIN3", "AIN3"},
	{ "VOUT3_MUX", "VIN4", "AIN4"},
	{ "VOUT3_MUX", "VIN5", "AIN5"},

	{ "VIN_MUX", "VIN1", "AIN1"},
	{ "VIN_MUX", "VIN2", "AIN2"},
	{ "VIN_MUX", "VIN3", "AIN3"},
	{ "VIN_MUX", "VIN4", "AIN4"},
	{ "VIN_MUX", "VIN5", "AIN5"},

	{ "ADC", NULL, "VIN_MUX" },
};

struct snd_soc_component_driver soc_component_dev_wm8594 = {
	.probe             = wm8594_probe,
	.resume            = wm8594_resume,
	.set_bias_level    = wm8594_set_bias_level,
	.controls          = wm8594_snd_controls,
	.num_controls      = ARRAY_SIZE(wm8594_snd_controls),
	.dapm_widgets      = wm8594_dapm_widgets,
	.num_dapm_widgets  = ARRAY_SIZE(wm8594_dapm_widgets),
	.dapm_routes       = wm8594_dapm_routes,
	.num_dapm_routes   = ARRAY_SIZE(wm8594_dapm_routes),
	.suspend_bias_off  = false,
};

#define WM8594_RATES (SNDRV_PCM_RATE_8000  |    \
                      SNDRV_PCM_RATE_11025 |    \
                      SNDRV_PCM_RATE_16000 |    \
                      SNDRV_PCM_RATE_22050 |    \
                      SNDRV_PCM_RATE_32000 |    \
                      SNDRV_PCM_RATE_44100 |    \
                      SNDRV_PCM_RATE_48000 |    \
                      SNDRV_PCM_RATE_96000 |    \
                      SNDRV_PCM_RATE_192000)

#define WM8594_FORMATS (SNDRV_PCM_FMTBIT_S16_LE  |  \
                        SNDRV_PCM_FMTBIT_S20_3LE |  \
                        SNDRV_PCM_FMTBIT_S24_LE  |  \
                        SNDRV_PCM_FMTBIT_S24_3LE |  \
                        SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver wm8594_dai[] = {
	{
		.name     = "wm8594-hifi",
		.id       = 0,
		.playback = {
			.stream_name  = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates        = WM8594_RATES,
			.formats      = WM8594_FORMATS,
		},
		.capture = {
			.stream_name  = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates        = WM8594_RATES,
			.formats      = WM8594_FORMATS,
		},
		.ops = &wm8594_dai_ops,
	},
};

static const struct regmap_config wm8903_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = WM8594_CACHEREG_NUM,
	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = wm8594_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(wm8594_reg_defaults),
};

static int wm8594_i2c_probe(struct i2c_client *i2c)
{
	struct wm8594_priv *wm8594;
	int ret;
	unsigned int devid = 0, version = 0;

	wm8594 = devm_kzalloc(&i2c->dev,
	                      sizeof(struct wm8594_priv),
	                      GFP_KERNEL);
	if (!wm8594) {
		return -ENOMEM;
	}

	wm8594->regmap = devm_regmap_init_i2c(i2c, &wm8903_regmap);
	if (IS_ERR(wm8594->regmap)) {
		ret = PTR_ERR(wm8594->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
		        ret);
		return ret;
	}
	wm8594->powered = false;

	wm8594->reset_gpiod = devm_gpiod_get_optional(&i2c->dev, "reset",
	                      GPIOD_OUT_LOW);
	if (IS_ERR(wm8594->reset_gpiod)) {
		return PTR_ERR(wm8594->reset_gpiod);
	}
	gpiod_set_value_cansleep(wm8594->reset_gpiod, 1);

	wm8594->pwdn_gpiod = devm_gpiod_get_optional(&i2c->dev, "pwdn",
	                     GPIOD_OUT_LOW);
	if (IS_ERR(wm8594->pwdn_gpiod)) {
		return PTR_ERR(wm8594->pwdn_gpiod);
	}
	gpiod_set_value_cansleep(wm8594->pwdn_gpiod, 1);

	wm8594->mute_gpiod = devm_gpiod_get_optional(&i2c->dev, "mute",
	                     GPIOD_OUT_LOW);
	if (IS_ERR(wm8594->mute_gpiod)) {
		return PTR_ERR(wm8594->mute_gpiod);
	}
	gpiod_set_value_cansleep(wm8594->mute_gpiod, 0);

	regcache_cache_bypass(wm8594->regmap, true);
	regmap_read(wm8594->regmap, WM8594_DEVICE_ID, &devid);
	regmap_read(wm8594->regmap, WM8594_REVISION, &version);
	regcache_cache_bypass(wm8594->regmap, false);
	dev_info(&i2c->dev, "DEVID %x, VERSION %x\n", devid & 0xffff, version & 0xff);

	i2c_set_clientdata(i2c, wm8594);

	ret = snd_soc_register_component(&i2c->dev, &soc_component_dev_wm8594,
	                                 wm8594_dai, ARRAY_SIZE(wm8594_dai));

	if (ret < 0) {
		pr_err(CODEC_NAME  ": %s failed: ret = %d\n", __func__, ret);
		return ret;
	}

	dev_info(&i2c->dev, "probe success!!!\n");

	return ret;
}

static void wm8594_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_component(&client->dev);
}

/*
 * WM8594 2 wires address is determined by pin 45
 * state during powerup.
 *    low  = 0x1a
 *    high = 0x1c
 */
static const struct i2c_device_id wm8594_i2c_table[] = {
	{"wm8594", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, wm8594_i2c_table);

static const struct of_device_id wm8594_of_match[] = {
	{.compatible = "wlf,wm8594", },
	{},
};
MODULE_DEVICE_TABLE(of, wm8594_of_match);

static struct i2c_driver wm8594_i2c_driver = {
	.driver = {
		.name = "wm8594",
		.of_match_table = of_match_ptr(wm8594_of_match),
	},
	.probe    = wm8594_i2c_probe,
	.remove   = wm8594_i2c_remove,
	.id_table = wm8594_i2c_table,
};
module_i2c_driver(wm8594_i2c_driver);

MODULE_DESCRIPTION("ASoC wm8594 driver");
MODULE_LICENSE("GPL");
