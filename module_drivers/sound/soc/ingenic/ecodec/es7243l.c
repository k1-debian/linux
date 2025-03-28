/*
 * ALSA SoC ES7243l adc driver
 *
 * Author:      David Yang, <yangxiaohua@everest-semi.com>
 * Copyright:   (C) 2017 Everest Semiconductor Co Ltd.,
 *
 * Based on sound/soc/codecs/es7243.c by DavidYang
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Notes:
 * ES7243l is a stereo Audio ADC for Microphone Array
 *
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/initval.h>
#include <linux/regmap.h>

#include "es7243l.h"

#define AVDD_3V3 1
struct i2c_client *i2c_clt[(ES7243l_CHANNELS_MAX) / 2];

/* codec private data */
struct es7243l_priv {
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct clk *mclk;
	unsigned int sysclk;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;
	bool dmic;
	u8 mclksrc;
	bool mclkinv;
	bool bclkinv;
	u8 tdm;
	u8 vdda;
};
struct snd_soc_component *tron_codec[8];
static int es7243l_codec_num = 0;

static bool es7243l_reg_is_volatile(struct device *dev, unsigned int reg)
{
	return (reg == ES7243l_CHIPID1 || reg == ES7243l_CHIPID2);
}

static const struct regmap_config es7243l_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.volatile_reg = es7243l_reg_is_volatile,
};

static int es7243l_read(u8 reg, u8 *rt_value, struct i2c_client *client)
{
	int ret;
	u8 read_cmd[3] = { 0 };
	u8 cmd_len = 0;

	read_cmd[0] = reg;
	cmd_len = 1;

	if (client->adapter == NULL) {
		pr_err("es7243_read client->adapter==NULL\n");
	}

	ret = i2c_master_send(client, read_cmd, cmd_len);
	if (ret != cmd_len) {
		pr_err("es7243_read error1\n");
		return -1;
	}

	ret = i2c_master_recv(client, rt_value, 1);
	if (ret != 1) {
		pr_err("es7243_read error2, ret = %d.\n", ret);
		return -1;
	}

	return 0;
}

static int
es7243l_write(u8 reg, unsigned char value, struct i2c_client *client)
{
	int ret = 0;
	u8 write_cmd[2] = { 0 };

	write_cmd[0] = reg;
	write_cmd[1] = value;

	ret = i2c_master_send(client, write_cmd, 2);
	if (ret != 2) {
		pr_err("es7243_write error->[REG-0x%02x,val-0x%02x]\n",
		       reg, value);
		return -1;
	}

	return 0;
}

static int
es7243l_update_bits(u8 reg, u8 mask, u8 value, struct i2c_client *client)
{
	u8 val_old, val_new;

	es7243l_read(reg, &val_old, client);
	val_new = (val_old & ~mask) | (value & mask);
	if (val_new != val_old) {
		es7243l_write(reg, val_new, client);
	}

	return 0;
}

struct _coeff_div {
	u32 mclk;       //mclk frequency
	u32 sr_rate;        //sample rate
	u8 osr;         //adc over sample rate
	u8 prediv_premulti; //adcclk and dacclk divider
	u8 cf_dsp_div;      //adclrck divider and daclrck divider
	u8 scale;
	u8 lrckdiv_h;
	u8 lrckdiv_l;
	u8 bclkdiv;     //sclk divider
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	//mclk     lrck,  osr, pre, div  ,scale, lrckdiv_h, lrckdiv_l, bclkdiv
	/* 24.576MHZ */
	{24576000, 8000, 0x20, 0x50, 0x00, 0x00, 0x0b, 0xff, 0x2f},
	{24576000, 12000, 0x20, 0x30, 0x00, 0x00, 0x07, 0xff, 0x1f},
	{24576000, 16000, 0x20, 0x20, 0x00, 0x00, 0x05, 0xff, 0x17},
	{24576000, 24000, 0x20, 0x10, 0x00, 0x00, 0x03, 0xff, 0x0f},
	{24576000, 32000, 0x20, 0x21, 0x00, 0x00, 0x02, 0xff, 0x0b},
	{24576000, 48000, 0x20, 0x00, 0x00, 0x00, 0x01, 0xff, 0x07},
	{24576000, 96000, 0x20, 0x01, 0x00, 0x00, 0x01, 0xff, 0x07},
	/* 12.288MHZ */
	{12288000, 8000, 0x20, 0x20, 0x00, 0x00, 0x05, 0xff, 0x17},
	{12288000, 12000, 0x20, 0x10, 0x00, 0x00, 0x03, 0xff, 0x0f},
	{12288000, 16000, 0x20, 0x21, 0x00, 0x00, 0x02, 0xff, 0x0b},
	{12288000, 24000, 0x20, 0x00, 0x00, 0x00, 0x01, 0xff, 0x07},
	{12288000, 32000, 0x20, 0x22, 0x00, 0x00, 0x01, 0x7f, 0x05},
	{12288000, 48000, 0x20, 0x01, 0x00, 0x00, 0x00, 0xff, 0x03},
	{12288000, 96000, 0x20, 0x13, 0x00, 0x00, 0x00, 0x7f, 0x01},
	/* 6.144MHZ */
	{6144000, 8000, 0x20, 0x21, 0x00, 0x00, 0x02, 0xff, 0x0b},
	{6144000, 12000, 0x20, 0x00, 0x00, 0x00, 0x01, 0xff, 0x07},
	{6144000, 16000, 0x20, 0x22, 0x00, 0x00, 0x01, 0x7f, 0x05},
	{6144000, 24000, 0x20, 0x01, 0x00, 0x00, 0x00, 0xff, 0x03},
	{6144000, 32000, 0x20, 0x23, 0x00, 0x00, 0x00, 0xbf, 0x02},
	{6144000, 48000, 0x20, 0x02, 0x00, 0x00, 0x00, 0x7f, 0x01},
	{6144000, 96000, 0x20, 0x03, 0x00, 0x00, 0x00, 0x7f, 0x01},
	/* 3.072MHZ */
	{3072000, 8000, 0x20, 0x22, 0x00, 0x00, 0x01, 0x7f, 0x05},
	{3072000, 12000, 0x20, 0x01, 0x00, 0x00, 0x00, 0xff, 0x03},
	{3072000, 16000, 0x20, 0x23, 0x00, 0x00, 0x00, 0xbf, 0x02},
	{3072000, 24000, 0x20, 0x02, 0x00, 0x00, 0x00, 0x7f, 0x01},
	{3072000, 32000, 0x10, 0x03, 0x20, 0x04, 0x00, 0x5f, 0x02},
	{3072000, 48000, 0x20, 0x03, 0x00, 0x00, 0x00, 0x3f, 0x00},
	/* 1.536MHZ */
	{1536000, 8000, 0x20, 0x23, 0x00, 0x00, 0x00, 0xbf, 0x02},
	{1536000, 12000, 0x20, 0x02, 0x00, 0x00, 0x00, 0x7f, 0x01},
	{1536000, 16000, 0x10, 0x03, 0x20, 0x04, 0x00, 0x5f, 0x00},
	{1536000, 24000, 0x20, 0x03, 0x00, 0x00, 0x00, 0x3f, 0x00},
	/* 32.768MHZ */
	{32768000, 8000, 0x20, 0x70, 0x00, 0x00, 0x0f, 0xff, 0x3f},
	{32768000, 16000, 0x20, 0x30, 0x00, 0x00, 0x07, 0xff, 0x1f},
	{32768000, 32000, 0x20, 0x10, 0x00, 0x00, 0x03, 0xff, 0x0f},
	/* 16.384MHZ */
	{16384000, 8000, 0x20, 0x30, 0x00, 0x00, 0x07, 0xff, 0x1f},
	{16384000, 16000, 0x20, 0x10, 0x00, 0x00, 0x03, 0xff, 0x0f},
	{16384000, 32000, 0x20, 0x00, 0x00, 0x00, 0x01, 0xff, 0x07},
	/* 8.192MHZ */
	{8192000, 8000, 0x20, 0x10, 0x00, 0x00, 0x03, 0xff, 0x0f},
	{8192000, 16000, 0x20, 0x00, 0x00, 0x00, 0x01, 0xff, 0x07},
	{8192000, 32000, 0x20, 0x01, 0x00, 0x00, 0x00, 0xff, 0x03},
	/* 4.096MHZ */
	{4096000, 8000, 0x20, 0x00, 0x00, 0x00, 0x01, 0xff, 0x07},
	{4096000, 16000, 0x20, 0x01, 0x00, 0x00, 0x00, 0xff, 0x03},
	{4096000, 32000, 0x20, 0x02, 0x00, 0x00, 0x00, 0x7f, 0x01},
	/* 2.048MHZ */
	{2048000, 8000, 0x20, 0x01, 0x00, 0x00, 0x00, 0xff, 0x03},
	{2048000, 16000, 0x20, 0x02, 0x00, 0x00, 0x00, 0x7f, 0x01},
	{2048000, 32000, 0x20, 0x03, 0x00, 0x00, 0x00, 0x3f, 0x00},
	/* 1.024MHZ */
	{1024000, 8000, 0x20, 0x02, 0x00, 0x00, 0x00, 0x7f, 0x01},
	{1024000, 16000, 0x20, 0x03, 0x00, 0x00, 0x00, 0x3f, 0x00},
	/* 22.5792MHz */
	{22579200, 11025, 0x20, 0x30, 0x00, 0x00, 0x07, 0xff, 0x1f},
	{22579200, 22050, 0x20, 0x10, 0x00, 0x00, 0x03, 0xff, 0x0f},
	{22579200, 44100, 0x20, 0x00, 0x00, 0x00, 0x01, 0xff, 0x07},
	/* 11.2896MHz */
	{11289600, 11025, 0x20, 0x10, 0x00, 0x00, 0x03, 0xff, 0x0f},
	{11289600, 22050, 0x20, 0x00, 0x00, 0x00, 0x01, 0xff, 0x07},
	{11289600, 44100, 0x20, 0x01, 0x00, 0x00, 0x00, 0xff, 0x03},
	/* 5.6448MHz */
	{56448000, 11025, 0x20, 0x00, 0x00, 0x00, 0x01, 0xff, 0x07},
	{56448000, 22050, 0x20, 0x01, 0x00, 0x00, 0x00, 0xff, 0x03},
	{56448000, 44100, 0x20, 0x02, 0x00, 0x00, 0x00, 0x7f, 0x01},
	/* 2.8224MHz */
	{28224000, 11025, 0x20, 0x01, 0x00, 0x00, 0x00, 0xff, 0x03},
	{28224000, 22050, 0x20, 0x02, 0x00, 0x00, 0x00, 0x7f, 0x01},
	{28224000, 44100, 0x20, 0x03, 0x00, 0x00, 0x00, 0x3f, 0x00},
	/* 1.4112MHz */
	{14112000, 11025, 0x20, 0x02, 0x00, 0x00, 0x00, 0x7f, 0x01},
	{14112000, 22050, 0x20, 0x03, 0x00, 0x00, 0x00, 0x3f, 0x00},
};

static inline int get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].sr_rate == rate && coeff_div[i].mclk == mclk) {
			return i;
		}
	}

	return -EINVAL;
}

/* The set of rates we can generate from the above for each SYSCLK */

static unsigned int rates_12288[] = {
	8000, 12000, 16000, 24000, 32000, 48000, 64000, 96000, 128000, 192000,
};

static unsigned int rates_8192[] = {
	8000, 16000, 32000, 64000, 128000,
};

static struct snd_pcm_hw_constraint_list constraints_12288 = {
	.count = ARRAY_SIZE(rates_12288),
	.list = rates_12288,
};

static struct snd_pcm_hw_constraint_list constraints_8192 = {
	.count = ARRAY_SIZE(rates_8192),
	.list = rates_8192,
};

static unsigned int rates_112896[] = {
	8000, 11025, 22050, 44100,
};

static struct snd_pcm_hw_constraint_list constraints_112896 = {
	.count = ARRAY_SIZE(rates_112896),
	.list = rates_112896,
};

/*
 * Note that this should be called from init rather than from hw_params.
 */
static int
es7243l_set_dai_sysclk(struct snd_soc_dai *codec_dai,
                       int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = codec_dai->component;
	struct es7243l_priv *es7243l = snd_soc_component_get_drvdata(component);

	printk("Enter into %s(), freq = %d\n", __func__, freq);
	switch (freq) {
		case 11289600:
		case 22579200:
			es7243l->sysclk_constraints = &constraints_112896;
			es7243l->sysclk = freq;
			return 0;
		case 12288000:
		case 24576000:
			es7243l->sysclk_constraints = &constraints_12288;
			es7243l->sysclk = freq;
			return 0;
		case 4096000:
		case 8192000:
			es7243l->sysclk_constraints = &constraints_8192;
			es7243l->sysclk = freq;
			return 0;
	}
	return 0;
}

static int es7243l_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{

	u8 iface = 0;
	u8 adciface = 0;
	u8 clksel = 0;
	u8 i;
	for (i = 0; i < (ES7243l_CHANNELS_MAX) / 2; i++) {
		es7243l_read(ES7243l_SDP_FORMAT, &adciface, i2c_clt[i]);//get interface format
		es7243l_read(ES7243l_RESET, &iface, i2c_clt[i]);    //get spd interface
		es7243l_read(ES7243l_CLK2, &clksel, i2c_clt[i]);    //get spd interface

		/* set master/slave audio interface */
		switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
			case SND_SOC_DAIFMT_CBM_CFM:    // MASTER MODE
				iface |= 0x40;
				break;
			case SND_SOC_DAIFMT_CBS_CFS:    // SLAVE MODE
				iface &= 0xbf;
				break;
			default:
				return -EINVAL;
		}
		/* interface format */
		switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
			case SND_SOC_DAIFMT_I2S:
				adciface &= 0xFC;
				break;
			case SND_SOC_DAIFMT_RIGHT_J:
				return -EINVAL;
			case SND_SOC_DAIFMT_LEFT_J:
				adciface &= 0xFC;
				adciface |= 0x01;
				break;
			case SND_SOC_DAIFMT_DSP_A:
				adciface &= 0xDC;
				adciface |= 0x03;
				break;
			case SND_SOC_DAIFMT_DSP_B:
				adciface &= 0xDC;
				adciface |= 0x23;
				break;
			default:
				return -EINVAL;
		}

		/* clock inversion */
		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
			case SND_SOC_DAIFMT_NB_NF:
				adciface &= 0xdf;
				clksel &= 0xfe;
				break;
			case SND_SOC_DAIFMT_IB_IF:
				adciface |= 0x20;
				clksel |= 0x01;
				break;
			case SND_SOC_DAIFMT_IB_NF:
				adciface &= 0xdf;
				clksel |= 0x01;
				break;
			case SND_SOC_DAIFMT_NB_IF:
				adciface |= 0x20;
				clksel &= 0xfe;
				break;
			default:
				return -EINVAL;
		}

		es7243l_write(ES7243l_RESET, iface, i2c_clt[i]);
		es7243l_write(ES7243l_CLK2, clksel, i2c_clt[i]);
		es7243l_write(ES7243l_SDP_FORMAT, adciface, i2c_clt[i]);
	}
	return 0;
}

static int
es7243l_pcm_startup(struct snd_pcm_substream *substream,
                    struct snd_soc_dai *dai)
{
#if 0
	struct snd_soc_component *component = dai->component;
	struct es7243l_priv *es7243l = snd_soc_component_get_drvdata(component);

	//  printk("Enter into %s()%d\n", __func__,__LINE__);
	/* es7243l needs mclk/es7243l->sysclk value to configure registers.
	 * remove the following code and change the code in es7243l_pcm_hw_params
	 * according to the mclk configuration.
	 */
	if (!es7243l->sysclk && (ES7243l_MCLK_SOURCE == FROM_MCLK_PIN)) {
		dev_err(component->dev,
		        "No MCLK configured, set es7243l->sysclk in es7243l_pcm_hw_params\n");
		return -EINVAL;
	}

	snd_pcm_hw_constraint_list(substream->runtime, 0,
	                           SNDRV_PCM_HW_PARAM_RATE,
	                           es7243l->sysclk_constraints);
#endif
	return 0;
}

static int
es7243l_pcm_hw_params(struct snd_pcm_substream *substream,
                      struct snd_pcm_hw_params *params,
                      struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es7243l_priv *es7243l = snd_soc_component_get_drvdata(component);
	u8 index, regv;
	int coeff;

	/* If the es7243l->sysclk is a fixed value, for example 12.288M,
	 * set es7243l->sysclk = 12288000;
	 * else if es7243l->sysclk is some value times lrck, for example 128Fs,
	 * set es7243l->sysclk = 128 * params_rate(params);
	 */
	printk("ES7243l_MCLK_SOURCE=%d\n", ES7243l_MCLK_SOURCE);

	if (ES7243l_MCLK_SOURCE == FROM_MCLK_PIN) {
		coeff = get_coeff(es7243l->sysclk, params_rate(params));
	} else if (ES7243l_MCLK_SOURCE == FROM_INTERNAL_BCLK)
		coeff = get_coeff(params_rate(params) * ES7243l_MCLK_LRCK_RATIO,
		                  params_rate(params));
	if (coeff < 0) {
		printk("Unable to configure sample rate %dHz with %dHz MCLK",
		       params_rate(params), (ES7243l_MCLK_SOURCE == FROM_MCLK_PIN) ?
		       es7243l->sysclk : (params_rate(params) * ES7243l_MCLK_LRCK_RATIO));
		return coeff;
	}

	/*
	 * set clock parameters
	 */

	if (coeff >= 0) {
		for (index = 0; index < (ES7243l_CHANNELS_MAX) / 2; index++) {
			es7243l_write(ES7243l_ADC_OSR, coeff_div[coeff].osr,
			              i2c_clt[index]);
			es7243l_write(ES7243l_PREDIV,
			              coeff_div[coeff].prediv_premulti,
			              i2c_clt[index]);
			es7243l_write(ES7243l_CLK_DIV, coeff_div[coeff].cf_dsp_div,
			              i2c_clt[index]);
			es7243l_write(ES7243l_ADCCTL1, coeff_div[coeff].scale,
			              i2c_clt[index]);
			es7243l_write(ES7243l_ADC_OSR, coeff_div[coeff].osr,
			              i2c_clt[index]);

			es7243l_read(ES7243l_CLK_TRI, &regv, i2c_clt[index]);
			regv &= 0xf0;
			regv |= (coeff_div[coeff].lrckdiv_h & 0x0f);
			es7243l_write(ES7243l_CLK_TRI, regv, i2c_clt[index]);
			es7243l_write(ES7243l_BCLK_DIV, coeff_div[coeff].bclkdiv,
			              i2c_clt[index]);

		}
	}

	/*
	 * set data length
	 */
	for (index = 0; index < (ES7243l_CHANNELS_MAX) / 2; index++) {
		es7243l_read(ES7243l_SDP_FORMAT, &regv, i2c_clt[index]);
		regv &= 0xe3;
		switch (params_format(params)) {
			case SNDRV_PCM_FORMAT_S16_LE:
				regv |= 0x0c;
				break;
			case SNDRV_PCM_FORMAT_S20_3LE:
				regv |= 0x04;
				break;
			case SNDRV_PCM_FORMAT_S24_LE:
				break;
			case SNDRV_PCM_FORMAT_S32_LE:
				regv |= 0x10;
				break;
			default:
				regv |= 0x0c;
				break;
		}
		es7243l_write(ES7243l_SDP_FORMAT, regv, i2c_clt[index]);
	}
	msleep(50);
	for (index = 0; index < (ES7243l_CHANNELS_MAX) / 2; index++) {
		es7243l_read(ES7243l_SDP_FORMAT, &regv, i2c_clt[index]);
		regv &= 0x3f;
		es7243l_write(ES7243l_SDP_FORMAT, regv, i2c_clt[index]);

		/* enable internal clk */
		es7243l_write(ES7243l_CLK1, 0x3a, i2c_clt[index]);
	}

	return 0;
}

//static int es7243l_mute(struct snd_soc_dai *dai, int mute)
static int es7243l_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	u8 i;

	for (i = 0; i < (ES7243l_CHANNELS_MAX) / 2; i++) {
		if (mute) {
			es7243l_update_bits(ES7243l_SDP_FORMAT, 0xc0, 0xc0, i2c_clt[i]);
		} else {
			es7243l_update_bits(ES7243l_SDP_FORMAT, 0xc0, 0x00, i2c_clt[i]);
		}
	}
	return 0;
}

static int es7243l_set_bias_level(struct snd_soc_component *component,
                                  enum snd_soc_bias_level level)
{
	u8 i, regv;
	switch (level) {
		case SND_SOC_BIAS_ON:
			for (i = 0; i < (ES7243l_CHANNELS_MAX) / 2; i++) {
				es7243l_update_bits(ES7243l_SDP_FORMAT, 0xc0, 0xc0, i2c_clt[i]); //MUTE I2S SDP
				es7243l_write(ES7243l_RESET, 0x9E, i2c_clt[i]); //Digital reset
				es7243l_write(ES7243l_RESET, 0x80, i2c_clt[i]); //Digital Unreset
				es7243l_update_bits(ES7243l_SDP_FORMAT, 0xc0, 0x00, i2c_clt[i]); //Unmute I2S SDP
			}
			msleep(175);
			break;
		case SND_SOC_BIAS_PREPARE:
			break;
		case SND_SOC_BIAS_STANDBY:
			for (i = 0; i < (ES7243l_CHANNELS_MAX) / 2; i++) {
				/*
				 * enable clock
				 */
				es7243l_read(ES7243l_CLK1, &regv, i2c_clt[i]);
				regv |= 0x0A;
				es7243l_write(ES7243l_CLK1, regv, i2c_clt[i]);
				es7243l_write(ES7243l_ANALOG_PDN, 0x00, i2c_clt[i]);    //power up analog
				/*
				 * enable mic input 1
				 */
				es7243l_read(ES7243l_PGA1, &regv, i2c_clt[i]);
				regv |= 0x10;
				es7243l_write(ES7243l_PGA1, regv, i2c_clt[i]);
				/*
				 * enable mic input 2
				 */
				es7243l_read(ES7243l_PGA2, &regv, i2c_clt[i]);
				regv |= 0x10;
				es7243l_write(ES7243l_PGA2, regv, i2c_clt[i]);
				msleep(100);
				es7243l_update_bits(ES7243l_SDP_FORMAT, 0xc0, 0x00, i2c_clt[i]);//mute SDP
			}
			break;
		case SND_SOC_BIAS_OFF:
			for (i = 0; i < (ES7243l_CHANNELS_MAX) / 2; i++) {
				es7243l_update_bits(ES7243l_SDP_FORMAT, 0xc0, 0xc0, i2c_clt[i]);//mute SDP
				/*
				 * disable mic input 1
				 */
				es7243l_read(ES7243l_PGA1, &regv, i2c_clt[i]);
				regv &= 0xef;
				es7243l_write(ES7243l_PGA1, regv, i2c_clt[i]);
				/*
				 * disable mic input 2
				 */
				es7243l_read(ES7243l_PGA2, &regv, i2c_clt[i]);
				regv &= 0xef;
				es7243l_write(ES7243l_PGA2, regv, i2c_clt[i]);

				es7243l_write(ES7243l_ANALOG_PDN, 0xff, i2c_clt[i]);    //power down analog

				/*
				 * disable clock
				 */
				es7243l_read(ES7243l_CLK1, &regv, i2c_clt[i]);
				regv &= 0xf5;
				es7243l_write(ES7243l_CLK1, regv, i2c_clt[i]);
				es7243l_write(ES7243l_RESET, 0x9E, i2c_clt[i]); //Digital reset
			}
			break;
	}
	return 0;
}

/*
 * snd_controls for PGA gain, Mute, suspend and resume
 */
static const DECLARE_TLV_DB_SCALE(mic_boost_tlv, 0, 300, 0);

#if ES7243l_CHANNELS_MAX > 0
static int
es7243l_micboost1_setting_set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA1, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[0]);
	return 0;
}

static int
es7243l_micboost1_setting_get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7243l_read(ES7243l_PGA1, &val, i2c_clt[0]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_micboost2_setting_set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA2, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[0]);
	return 0;
}

static int
es7243l_micboost2_setting_get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7243l_read(ES7243l_PGA2, &val, i2c_clt[0]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_adc1_mute_set(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x40,
	                    (ucontrol->value.integer.value[0] & 0x01) << 6,
	                    i2c_clt[0]);
	return 0;
}

static int
es7243l_adc1_mute_get(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[0]);
	ucontrol->value.integer.value[0] = (val & 0x40) >> 6;
	return 0;
}

static int
es7243l_adc2_mute_set(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x80,
	                    (ucontrol->value.integer.value[0] & 0x01) << 7,
	                    i2c_clt[0]);
	return 0;
}

static int
es7243l_adc2_mute_get(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[0]);
	ucontrol->value.integer.value[0] = (val & 0x80) >> 7;
	return 0;
}

static int
es7243l_adc1adc2_suspend_get(struct snd_kcontrol *kcontrol,
                             struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7243l_read(ES7243l_VMIDSEL, &val, i2c_clt[0]);
	ucontrol->value.integer.value[0] = ((val & 0x02) >> 1) == 1 ? 0 : 1;
	return 0;
}

static int
es7243l_adc1adc2_suspend_set(struct snd_kcontrol *kcontrol,
                             struct snd_ctl_elem_value *ucontrol)
{
	//u8 val;
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {   //suspend
		es7243l_write(ES7243l_PREDIV, 0x02, i2c_clt[0]);
		es7243l_write(ES7243l_PREDIV, 0x01, i2c_clt[0]);
		es7243l_write(ES7243l_TESTMOD_0xF7, 0x30, i2c_clt[0]);
		es7243l_write(ES7243l_DLL_PWN, 0x01, i2c_clt[0]);
		es7243l_write(ES7243l_ANALOG_PDN, 0xff, i2c_clt[0]);
		es7243l_write(ES7243l_VMIDSEL, 0x00, i2c_clt[0]);
		es7243l_write(ES7243l_CLK1, 0x38, i2c_clt[0]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x00, i2c_clt[0]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x00, i2c_clt[0]);
		es7243l_write(ES7243l_RESET, 0x8e, i2c_clt[0]);
		es7243l_write(ES7243l_CLK1, 0x30, i2c_clt[0]);
	} else {        //resume
		es7243l_write(ES7243l_CLK1, 0x3a, i2c_clt[0]);
		es7243l_write(ES7243l_RESET, 0x80, i2c_clt[0]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x10, i2c_clt[0]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x10, i2c_clt[0]);
		es7243l_write(ES7243l_ANALOG_PDN, 0x00, i2c_clt[0]);
		es7243l_write(ES7243l_VMIDSEL, 0x02, i2c_clt[0]);
	}
	return 0;
}
#endif
#if ES7243l_CHANNELS_MAX > 2
static int
es7243l_micboost3_setting_set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA1, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[1]);
	return 0;
}

static int
es7243l_micboost3_setting_get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7243l_read(ES7243l_PGA1, &val, i2c_clt[1]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_micboost4_setting_set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA2, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[1]);
	return 0;
}

static int
es7243l_micboost4_setting_get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	u8 val;
	es7243l_read(ES7243l_PGA2, &val, i2c_clt[1]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_adc3_mute_set(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x40,
	                    (ucontrol->value.integer.value[0] & 0x01) << 6,
	                    i2c_clt[1]);
	return 0;
}

static int
es7243l_adc3_mute_get(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[1]);
	ucontrol->value.integer.value[0] = (val & 0X40) >> 6;
	return 0;
}

static int
es7243l_adc4_mute_set(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x80,
	                    (ucontrol->value.integer.value[0] & 0x01) << 7,
	                    i2c_clt[1]);
	return 0;
}

static int
es7243l_adc4_mute_get(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[1]);
	ucontrol->value.integer.value[0] = (val & 0x80) >> 7;
	return 0;
}

static int
es7243l_adc3adc4_suspend_get(struct snd_kcontrol *kcontrol,
                             struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_VMIDSEL, &val, i2c_clt[1]);
	ucontrol->value.integer.value[0] = ((val & 0x02) >> 1) == 1 ? 0 : 1;
	return 0;
}

static int
es7243l_adc3adc4_suspend_set(struct snd_kcontrol *kcontrol,
                             struct snd_ctl_elem_value *ucontrol)
{

	//u8 val;
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {   //suspend
		es7243l_write(ES7243l_PREDIV, 0x02, i2c_clt[1]);
		es7243l_write(ES7243l_PREDIV, 0x01, i2c_clt[1]);
		es7243l_write(ES7243l_TESTMOD_0xF7, 0x30, i2c_clt[1]);
		es7243l_write(ES7243l_DLL_PWN, 0x01, i2c_clt[1]);
		es7243l_write(ES7243l_ANALOG_PDN, 0xff, i2c_clt[1]);
		es7243l_write(ES7243l_VMIDSEL, 0x00, i2c_clt[1]);
		es7243l_write(ES7243l_CLK1, 0x38, i2c_clt[1]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x00, i2c_clt[1]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x00, i2c_clt[1]);
		es7243l_write(ES7243l_RESET, 0x8e, i2c_clt[1]);
		es7243l_write(ES7243l_CLK1, 0x30, i2c_clt[1]);
	} else {        //resume
		es7243l_write(ES7243l_CLK1, 0x3a, i2c_clt[1]);
		es7243l_write(ES7243l_RESET, 0x80, i2c_clt[1]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x10, i2c_clt[1]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x10, i2c_clt[1]);
		es7243l_write(ES7243l_ANALOG_PDN, 0x00, i2c_clt[1]);
		es7243l_write(ES7243l_VMIDSEL, 0x02, i2c_clt[1]);
	}
	return 0;
}
#endif
#if ES7243l_CHANNELS_MAX > 4
static int
es7243l_micboost5_setting_set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA1, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[2]);
	return 0;
}

static int
es7243l_micboost5_setting_get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_PGA1, &val, i2c_clt[2]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_micboost6_setting_set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA2, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[2]);
	return 0;
}

static int
es7243l_micboost6_setting_get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_PGA2, &val, i2c_clt[2]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_adc5_mute_set(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x40,
	                    (ucontrol->value.integer.value[0] & 0x01) << 6,
	                    i2c_clt[2]);
	return 0;
}

static int
es7243l_adc5_mute_get(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[2]);
	ucontrol->value.integer.value[0] = (val & 0x40) >> 6;
	return 0;
}

static int
es7243l_adc6_mute_set(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x80,
	                    (ucontrol->value.integer.value[0] & 0x01) << 7,
	                    i2c_clt[2]);
	return 0;
}

static int
es7243l_adc6_mute_get(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[2]);
	ucontrol->value.integer.value[0] = (val & 0x80) >> 7;
	return 0;
}

static int
es7243l_adc5adc6_suspend_get(struct snd_kcontrol *kcontrol,
                             struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_VMIDSEL, &val, i2c_clt[2]);
	ucontrol->value.integer.value[0] = ((val & 0x02) >> 1) == 1 ? 0 : 1;
	return 0;
}

static int
es7243l_adc5adc6_suspend_set(struct snd_kcontrol *kcontrol,
                             struct snd_ctl_elem_value *ucontrol)
{
	//u8 val;
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {   //suspend
		es7243l_write(ES7243l_PREDIV, 0x02, i2c_clt[2]);
		es7243l_write(ES7243l_PREDIV, 0x01, i2c_clt[2]);
		es7243l_write(ES7243l_TESTMOD_0xF7, 0x30, i2c_clt[2]);
		es7243l_write(ES7243l_DLL_PWN, 0x01, i2c_clt[2]);
		es7243l_write(ES7243l_ANALOG_PDN, 0xff, i2c_clt[2]);
		es7243l_write(ES7243l_VMIDSEL, 0x00, i2c_clt[2]);
		es7243l_write(ES7243l_CLK1, 0x38, i2c_clt[2]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x00, i2c_clt[2]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x00, i2c_clt[2]);
		es7243l_write(ES7243l_RESET, 0x8e, i2c_clt[2]);
		es7243l_write(ES7243l_CLK1, 0x30, i2c_clt[2]);
	} else {        //resume
		es7243l_write(ES7243l_CLK1, 0x3a, i2c_clt[2]);
		es7243l_write(ES7243l_RESET, 0x80, i2c_clt[2]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x10, i2c_clt[2]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x10, i2c_clt[2]);
		es7243l_write(ES7243l_ANALOG_PDN, 0x00, i2c_clt[2]);
		es7243l_write(ES7243l_VMIDSEL, 0x02, i2c_clt[2]);
	}
	return 0;
}
#endif
#if ES7243l_CHANNELS_MAX > 6
static int
es7243l_micboost7_setting_set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA1, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[3]);
	return 0;
}

static int
es7243l_micboost7_setting_get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_PGA1, &val, i2c_clt[3]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_micboost8_setting_set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA2, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[3]);
	return 0;
}

static int
es7243l_micboost8_setting_get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_PGA2, &val, i2c_clt[3]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_adc7_mute_set(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x40,
	                    (ucontrol->value.integer.value[0] & 0x01) << 6,
	                    i2c_clt[3]);
	return 0;
}

static int
es7243l_adc7_mute_get(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[3]);
	ucontrol->value.integer.value[0] = (val & 0x40) >> 6;
	return 0;
}

static int
es7243l_adc8_mute_set(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x80,
	                    (ucontrol->value.integer.value[0] & 0x01) << 7,
	                    i2c_clt[3]);
	return 0;
}

static int
es7243l_adc8_mute_get(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[3]);
	ucontrol->value.integer.value[0] = (val & 0x80) >> 7;
	return 0;
}

static int
es7243l_adc7adc8_suspend_get(struct snd_kcontrol *kcontrol,
                             struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_VMIDSEL, &val, i2c_clt[3]);
	ucontrol->value.integer.value[0] = ((val & 0x02) >> 1) == 1 ? 0 : 1;
	return 0;
}

static int
es7243l_adc7adc8_suspend_set(struct snd_kcontrol *kcontrol,
                             struct snd_ctl_elem_value *ucontrol)
{
	//      u8 val;
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {   //suspend
		es7243l_write(ES7243l_PREDIV, 0x02, i2c_clt[3]);
		es7243l_write(ES7243l_PREDIV, 0x01, i2c_clt[3]);
		es7243l_write(ES7243l_TESTMOD_0xF7, 0x30, i2c_clt[3]);
		es7243l_write(ES7243l_DLL_PWN, 0x01, i2c_clt[3]);
		es7243l_write(ES7243l_ANALOG_PDN, 0xff, i2c_clt[3]);
		es7243l_write(ES7243l_VMIDSEL, 0x00, i2c_clt[3]);
		es7243l_write(ES7243l_CLK1, 0x38, i2c_clt[3]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x00, i2c_clt[3]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x00, i2c_clt[3]);
		es7243l_write(ES7243l_RESET, 0x8e, i2c_clt[3]);
		es7243l_write(ES7243l_CLK1, 0x30, i2c_clt[3]);
	} else {        //resume
		es7243l_write(ES7243l_CLK1, 0x3a, i2c_clt[3]);
		es7243l_write(ES7243l_RESET, 0x80, i2c_clt[3]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x10, i2c_clt[3]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x10, i2c_clt[3]);
		es7243l_write(ES7243l_ANALOG_PDN, 0x00, i2c_clt[3]);
		es7243l_write(ES7243l_VMIDSEL, 0x02, i2c_clt[3]);
	}
	return 0;
}
#endif
#if ES7243l_CHANNELS_MAX > 8
static int
es7243l_micboost9_setting_set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA1, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[4]);
	return 0;
}

static int
es7243l_micboost9_setting_get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_PGA1, &val, i2c_clt[4]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_micboost10_setting_set(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA2, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[4]);
	return 0;
}

static int
es7243l_micboost10_setting_get(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_PGA2, &val, i2c_clt[4]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_adc9_mute_set(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x40,
	                    (ucontrol->value.integer.value[0] & 0x01) << 6,
	                    i2c_clt[4]);
	return 0;
}

static int
es7243l_adc9_mute_get(struct snd_kcontrol *kcontrol,
                      struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[4]);
	ucontrol->value.integer.value[0] = (val & 0x40) >> 6;
	return 0;
}

static int
es7243l_adc10_mute_set(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x80,
	                    (ucontrol->value.integer.value[0] & 0x01) << 7,
	                    i2c_clt[4]);
	return 0;
}

static int
es7243l_adc10_mute_get(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[4]);
	ucontrol->value.integer.value[0] = (val & 0x80) >> 7;
	return 0;
}

static int
es7243l_adc9adc10_suspend_get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_VMIDSEL, &val, i2c_clt[4]);
	ucontrol->value.integer.value[0] = ((val & 0x02) >> 1) == 1 ? 0 : 1;
	return 0;
}

static int
es7243l_adc9adc10_suspend_set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	//      u8 val;
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {   //suspend
		es7243l_write(ES7243l_PREDIV, 0x02, i2c_clt[4]);
		es7243l_write(ES7243l_PREDIV, 0x01, i2c_clt[4]);
		es7243l_write(ES7243l_TESTMOD_0xF7, 0x30, i2c_clt[4]);
		es7243l_write(ES7243l_DLL_PWN, 0x01, i2c_clt[4]);
		es7243l_write(ES7243l_ANALOG_PDN, 0xff, i2c_clt[4]);
		es7243l_write(ES7243l_VMIDSEL, 0x00, i2c_clt[4]);
		es7243l_write(ES7243l_CLK1, 0x38, i2c_clt[4]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x00, i2c_clt[4]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x00, i2c_clt[4]);
		es7243l_write(ES7243l_RESET, 0x8e, i2c_clt[4]);
		es7243l_write(ES7243l_CLK1, 0x30, i2c_clt[4]);
	} else {        //resume
		es7243l_write(ES7243l_CLK1, 0x3a, i2c_clt[4]);
		es7243l_write(ES7243l_RESET, 0x80, i2c_clt[4]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x10, i2c_clt[4]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x10, i2c_clt[4]);
		es7243l_write(ES7243l_ANALOG_PDN, 0x00, i2c_clt[4]);
		es7243l_write(ES7243l_VMIDSEL, 0x02, i2c_clt[4]);
	}
	return 0;
}
#endif
#if ES7243l_CHANNELS_MAX > 10
static int
es7243l_micboost11_setting_set(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA1, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[5]);
	return 0;
}

static int
es7243l_micboost11_setting_get(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_PGA1, &val, i2c_clt[5]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_micboost12_setting_set(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA2, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[5]);
	return 0;
}

static int
es7243l_micboost12_setting_get(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_PGA2, &val, i2c_clt[5]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_adc11_mute_set(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x40,
	                    (ucontrol->value.integer.value[0] & 0x01) << 6,
	                    i2c_clt[5]);
	return 0;
}

static int
es7243l_adc11_mute_get(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[5]);
	ucontrol->value.integer.value[0] = (val & 0x40) >> 6;
	return 0;
}

static int
es7243l_adc12_mute_set(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x80,
	                    (ucontrol->value.integer.value[0] & 0x01) << 7,
	                    i2c_clt[5]);
	return 0;
}

static int
es7243l_adc12_mute_get(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[5]);
	ucontrol->value.integer.value[0] = (val & 0x80) >> 7;
	return 0;
}

static int
es7243l_adc11adc12_suspend_get(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_VMIDSEL, &val, i2c_clt[5]);
	ucontrol->value.integer.value[0] = ((val & 0x02) >> 1) == 1 ? 0 : 1;
	return 0;
}

static int
es7243l_adc11adc12_suspend_set(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {   //suspend
		es7243l_write(ES7243l_PREDIV, 0x02, i2c_clt[5]);
		es7243l_write(ES7243l_PREDIV, 0x01, i2c_clt[5]);
		es7243l_write(ES7243l_TESTMOD_0xF7, 0x30, i2c_clt[5]);
		es7243l_write(ES7243l_DLL_PWN, 0x01, i2c_clt[5]);
		es7243l_write(ES7243l_ANALOG_PDN, 0xff, i2c_clt[5]);
		es7243l_write(ES7243l_VMIDSEL, 0x00, i2c_clt[5]);
		es7243l_write(ES7243l_CLK1, 0x38, i2c_clt[5]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x00, i2c_clt[5]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x00, i2c_clt[5]);
		es7243l_write(ES7243l_RESET, 0x8e, i2c_clt[5]);
		es7243l_write(ES7243l_CLK1, 0x30, i2c_clt[5]);
	} else {        //resume
		es7243l_write(ES7243l_CLK1, 0x3a, i2c_clt[5]);
		es7243l_write(ES7243l_RESET, 0x80, i2c_clt[5]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x10, i2c_clt[5]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x10, i2c_clt[5]);
		es7243l_write(ES7243l_ANALOG_PDN, 0x00, i2c_clt[5]);
		es7243l_write(ES7243l_VMIDSEL, 0x02, i2c_clt[5]);
	}
	return 0;
}
#endif
#if ES7243l_CHANNELS_MAX > 12
static int
es7243l_micboost13_setting_set(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA1, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[6]);
	return 0;
}

static int
es7243l_micboost13_setting_get(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_PGA1, &val, i2c_clt[6]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_micboost14_setting_set(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA2, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[6]);
	return 0;
}

static int
es7243l_micboost14_setting_get(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_PGA2, &val, i2c_clt[6]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_adc13_mute_set(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x40,
	                    (ucontrol->value.integer.value[0] & 0x01) << 6,
	                    i2c_clt[6]);
	return 0;
}

static int
es7243l_adc13_mute_get(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[6]);
	ucontrol->value.integer.value[0] = (val & 0x40) >> 6;
	return 0;
}

static int
es7243l_adc14_mute_set(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x80,
	                    (ucontrol->value.integer.value[0] & 0x01) << 7,
	                    i2c_clt[6]);
	return 0;
}

static int
es7243l_adc14_mute_get(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[6]);
	ucontrol->value.integer.value[0] = (val & 0x80) >> 7;
	return 0;
}

static int
es7243l_adc13adc14_suspend_get(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_VMIDSEL, &val, i2c_clt[6]);
	ucontrol->value.integer.value[0] = ((val & 0x02) >> 1) == 1 ? 0 : 1;
	return 0;
}

static int
es7243l_adc13adc14_suspend_set(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {   //suspend
		es7243l_write(ES7243l_PREDIV, 0x02, i2c_clt[6]);
		es7243l_write(ES7243l_PREDIV, 0x01, i2c_clt[6]);
		es7243l_write(ES7243l_TESTMOD_0xF7, 0x30, i2c_clt[6]);
		es7243l_write(ES7243l_DLL_PWN, 0x01, i2c_clt[6]);
		es7243l_write(ES7243l_ANALOG_PDN, 0xff, i2c_clt[6]);
		es7243l_write(ES7243l_VMIDSEL, 0x00, i2c_clt[6]);
		es7243l_write(ES7243l_CLK1, 0x38, i2c_clt[6]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x00, i2c_clt[6]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x00, i2c_clt[6]);
		es7243l_write(ES7243l_RESET, 0x8e, i2c_clt[6]);
		es7243l_write(ES7243l_CLK1, 0x30, i2c_clt[6]);
	} else {        //resume

		es7243l_write(ES7243l_CLK1, 0x3a, i2c_clt[6]);
		es7243l_write(ES7243l_RESET, 0x80, i2c_clt[6]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x10, i2c_clt[6]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x10, i2c_clt[6]);
		es7243l_write(ES7243l_ANALOG_PDN, 0x00, i2c_clt[6]);
		es7243l_write(ES7243l_VMIDSEL, 0x02, i2c_clt[6]);
	}
	return 0;
}
#endif
#if ES7243l_CHANNELS_MAX > 14
static int
es7243l_micboost15_setting_set(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA1, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[7]);
	return 0;
}

static int
es7243l_micboost15_setting_get(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_PGA1, &val, i2c_clt[7]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_micboost16_setting_set(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_PGA2, 0x0F,
	                    ucontrol->value.integer.value[0] & 0x0f,
	                    i2c_clt[7]);
	return 0;
}

static int
es7243l_micboost16_setting_get(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_PGA2, &val, i2c_clt[7]);
	ucontrol->value.integer.value[0] = val & 0x0f;
	return 0;
}

static int
es7243l_adc15_mute_set(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x40,
	                    (ucontrol->value.integer.value[0] & 0x01) << 6,
	                    i2c_clt[7]);
	return 0;
}

static int
es7243l_adc15_mute_get(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[7]);
	ucontrol->value.integer.value[0] = (val 0x40) >> 6;
	return 0;
}

static int
es7243l_adc16_mute_set(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
	es7243l_update_bits(ES7243l_SDP_FORMAT, 0x80,
	                    (ucontrol->value.integer.value[0] & 0x01) << 7,
	                    i2c_clt[7]);
	return 0;
}

static int
es7243l_adc16_mute_get(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_SDP_FORMAT, &val, i2c_clt[7]);
	ucontrol->value.integer.value[0] = (val & 0x80) >> 7;
	return 0;
}

static int
es7243l_adc15adc16_suspend_get(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	u8 val;

	es7243l_read(ES7243l_VMIDSEL, &val, i2c_clt[7]);
	ucontrol->value.integer.value[0] = ((val & 0x02) >> 1) == 1 ? 0 : 1;
	return 0;
}

static int
es7243l_adc15adc16_suspend_set(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	if ((ucontrol->value.integer.value[0] & 0x01) == 1) {   //suspend
		es7243l_write(ES7243l_PREDIV, 0x02, i2c_clt[7]);
		es7243l_write(ES7243l_PREDIV, 0x01, i2c_clt[7]);
		es7243l_write(ES7243l_TESTMOD_0xF7, 0x30, i2c_clt[7]);
		es7243l_write(ES7243l_DLL_PWN, 0x01, i2c_clt[7]);
		es7243l_write(ES7243l_ANALOG_PDN, 0xff, i2c_clt[7]);
		es7243l_write(ES7243l_VMIDSEL, 0x00, i2c_clt[7]);
		es7243l_write(ES7243l_CLK1, 0x38, i2c_clt[7]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x00, i2c_clt[7]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x00, i2c_clt[7]);
		es7243l_write(ES7243l_RESET, 0x8e, i2c_clt[7]);
		es7243l_write(ES7243l_CLK1, 0x30, i2c_clt[7]);
	} else {        //resume
		es7243l_write(ES7243l_CLK1, 0x3a, i2c_clt[7]);
		es7243l_write(ES7243l_RESET, 0x80, i2c_clt[7]);
		es7243l_update_bits(ES7243l_PGA1, 0x10, 0x10, i2c_clt[7]);
		es7243l_update_bits(ES7243l_PGA2, 0x10, 0x10, i2c_clt[7]);
		es7243l_write(ES7243l_ANALOG_PDN, 0x00, i2c_clt[7]);
		es7243l_write(ES7243l_VMIDSEL, 0x02, i2c_clt[7]);
	}
	return 0;
}
#endif
static const struct snd_kcontrol_new es7243l_snd_controls[] = {
#if ES7243l_CHANNELS_MAX > 0
	SOC_SINGLE_EXT_TLV("PGA1_setting", 0x20, 0, 0x0F, 0,
	                   es7243l_micboost1_setting_get,
	                   es7243l_micboost1_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA2_setting", 0x21, 0, 0x0F, 0,
	                   es7243l_micboost2_setting_get,
	                   es7243l_micboost2_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT("ADC1_MUTE", 0x0B, 1, 0x40, 0,
	               es7243l_adc1_mute_get, es7243l_adc1_mute_set),
	SOC_SINGLE_EXT("ADC2_MUTE", 0x0B, 1, 0x80, 0,
	               es7243l_adc2_mute_get, es7243l_adc2_mute_set),
	SOC_SINGLE_EXT("ADC1ADC2_SUSPEND", 0x17, 1, 1, 0,
	               es7243l_adc1adc2_suspend_get,
	               es7243l_adc1adc2_suspend_set),
#endif
#if ES7243l_CHANNELS_MAX > 2
	SOC_SINGLE_EXT_TLV("PGA3_setting", 0x20, 0, 0x0F, 0,
	                   es7243l_micboost3_setting_get,
	                   es7243l_micboost3_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA4_setting", 0x21, 0, 0x0F, 0,
	                   es7243l_micboost4_setting_get,
	                   es7243l_micboost4_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT("ADC3_MUTE", 0x0B, 1, 0x40, 0,
	               es7243l_adc3_mute_get, es7243l_adc3_mute_set),
	SOC_SINGLE_EXT("ADC4_MUTE", 0x0B, 1, 0x80, 0,
	               es7243l_adc4_mute_get,
	               es7243l_adc4_mute_set),
	SOC_SINGLE_EXT("ADC3ADC4_SUSPEND", 0x17, 1, 1, 0,
	               es7243l_adc3adc4_suspend_get,
	               es7243l_adc3adc4_suspend_set),
#endif
#if ES7243l_CHANNELS_MAX > 4
	SOC_SINGLE_EXT_TLV("PGA5_setting", 0x20, 0, 0x0F, 0,
	                   es7243l_micboost5_setting_get,
	                   es7243l_micboost5_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA6_setting", 0x21, 0, 0x0F, 0,
	                   es7243l_micboost6_setting_get,
	                   es7243l_micboost6_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT("ADC5_MUTE", 0x0B, 1, 0x40, 0,
	               es7243l_adc5_mute_get, es7243l_adc5_mute_set),
	SOC_SINGLE_EXT("ADC6_MUTE", 0x0B, 1, 0x80, 0,
	               es7243l_adc6_mute_get, es7243l_adc6_mute_set),
	SOC_SINGLE_EXT("ADC5ADC6_SUSPEND", 0x17, 1, 1, 0,
	               es7243l_adc5adc6_suspend_get,
	               es7243l_adc5adc6_suspend_set),
#endif
#if ES7243l_CHANNELS_MAX > 6
	SOC_SINGLE_EXT_TLV("PGA7_setting", 0x20, 0, 0x0F, 0,
	                   es7243l_micboost7_setting_get,
	                   es7243l_micboost7_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA8_setting", 0x21, 0, 0x0F, 0,
	                   es7243l_micboost8_setting_get,
	                   es7243l_micboost8_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT("ADC7_MUTE", 0x0B, 1, 0x40, 0,
	               es7243l_adc7_mute_get, es7243l_adc7_mute_set),
	SOC_SINGLE_EXT("ADC8_MUTE", 0x0B, 1, 0x80, 0,
	               es7243l_adc8_mute_get, es7243l_adc8_mute_set),
	SOC_SINGLE_EXT("ADC7ADC8_SUSPEND", 0x17, 1, 1, 0,
	               es7243l_adc7adc8_suspend_get,
	               es7243l_adc7adc8_suspend_set),
#endif
#if ES7243l_CHANNELS_MAX > 8
	SOC_SINGLE_EXT_TLV("PGA9_setting", 0x20, 0, 0x0F, 0,
	                   es7243l_micboost9_setting_get,
	                   es7243l_micboost9_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA10_setting", 0x21, 0, 0x0F, 0,
	                   es7243l_micboost10_setting_get,
	                   es7243l_micboost10_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT("ADC9_MUTE", 0x0B, 1, 0x40, 0,
	               es7243l_adc9_mute_get, es7243l_adc9_mute_set),
	SOC_SINGLE_EXT("ADC10_MUTE", 0x0B, 1, 0x80, 0,
	               es7243l_adc10_mute_get, es7243l_adc10_mute_set),
	SOC_SINGLE_EXT("ADC9ADC10_SUSPEND", 0x17, 1, 1, 0,
	               es7243l_adc9adc10_suspend_get,
	               es7243l_adc9adc10_suspend_set),
#endif
#if ES7243l_CHANNELS_MAX > 10
	SOC_SINGLE_EXT_TLV("PGA11_setting", 0x20, 0, 0x0F, 0,
	                   es7243l_micboost11_setting_get,
	                   es7243l_micboost11_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA12_setting", 0x21, 0, 0x0F, 0,
	                   es7243l_micboost12_setting_get,
	                   es7243l_micboost12_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT("ADC11_MUTE", 0x0B, 1, 0x40, 0,
	               es7243l_adc11_mute_get, es7243l_adc11_mute_set),
	SOC_SINGLE_EXT("ADC12_MUTE", 0x0B, 1, 0x80, 0,
	               es7243l_adc12_mute_get, es7243l_adc12_mute_set),
	SOC_SINGLE_EXT("ADC11ADC12_SUSPEND", 0x17, 1, 1, 0,
	               es7243l_adc11adc12_suspend_get,
	               es7243l_adc11adc12_suspend_set),
#endif
#if ES7243l_CHANNELS_MAX > 12
	SOC_SINGLE_EXT_TLV("PGA13_setting", 0x20, 0, 0x0F, 0,
	                   es7243l_micboost13_setting_get,
	                   es7243l_micboost13_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA14_setting", 0x21, 0, 0x0F, 0,
	                   es7243l_micboost14_setting_get,
	                   es7243l_micboost14_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT("ADC13_MUTE", 0x0B, 1, 0x40, 0,
	               es7243l_adc13_mute_get, es7243l_adc13_mute_set),
	SOC_SINGLE_EXT("ADC14_MUTE", 0x0B, 1, 0x80, 0,
	               es7243l_adc14_mute_get, es7243l_adc14_mute_set),
	SOC_SINGLE_EXT("ADC13ADC14_SUSPEND", 0x17, 1, 1, 0,
	               es7243l_adc13adc14_suspend_get,
	               es7243l_adc13adc14_suspend_set),
#endif
#if ES7243l_CHANNELS_MAX > 14
	SOC_SINGLE_EXT_TLV("PGA15_setting", 0x20, 0, 0x0F, 0,
	                   es7243l_micboost15_setting_get,
	                   es7243l_micboost15_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT_TLV("PGA16_setting", 0x21, 0, 0x0F, 0,
	                   es7243l_micboost16_setting_get,
	                   es7243l_micboost16_setting_set,
	                   mic_boost_tlv),
	SOC_SINGLE_EXT("ADC15_MUTE", 0x0B, 1, 0x40, 0,
	               es7243l_adc15_mute_get, es7243l_adc15_mute_set),
	SOC_SINGLE_EXT("ADC16_MUTE", 0x0B, 1, 0x80, 0,
	               es7243l_adc16_mute_get, es7243l_adc16_mute_set),
	SOC_SINGLE_EXT("ADC15ADC16_SUSPEND", 0x17, 1, 1, 0,
	               es7243l_adc15adc16_suspend_get,
	               es7243l_adc15adc16_suspend_set),
#endif
};

static const struct snd_soc_dapm_widget es7243l_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("INPUT"),
	SND_SOC_DAPM_ADC("ADC", NULL,
	                 SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SDOUT", "I2S Capture", NULL,
	                     SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route es7243l_dapm_routes[] = {
	{"ADC", NULL, "INPUT"},
	{"SDOUT", NULL, "ADC"},
};

#define es7243l_RATES (SNDRV_PCM_RATE_8000_48000 | SNDRV_PCM_RATE_96000)

#define es7243l_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
                         SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops es7243l_ops = {
	.startup = es7243l_pcm_startup,
	.hw_params = es7243l_pcm_hw_params,
	.set_fmt = es7243l_set_dai_fmt,
	.set_sysclk = es7243l_set_dai_sysclk,
	//.digital_mute = es7243l_mute,
	.mute_stream = es7243l_mute,
};

#if ES7243l_CHANNELS_MAX > 0
static struct snd_soc_dai_driver es7243l_dai0 = {
	.name = "ES7243l HiFi 0",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = ES7243l_CHANNELS_MAX,
		.rates = es7243l_RATES,
		.formats = es7243l_FORMATS,
	},
	.ops = &es7243l_ops,
	.symmetric_rate = 1,
};
#endif
#if ES7243l_CHANNELS_MAX > 2
static struct snd_soc_dai_driver es7243l_dai1 = {
	.name = "ES7243l HiFi 1",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es7243l_RATES,
		.formats = es7243l_FORMATS,
	},
	.ops = &es7243l_ops,
	.symmetric_rate = 1,
};
#endif
#if ES7243l_CHANNELS_MAX > 4
static struct snd_soc_dai_driver es7243l_dai2 = {
	.name = "ES7243l HiFi 2",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es7243l_RATES,
		.formats = es7243l_FORMATS,
	},
	.ops = &es7243l_ops,
	.symmetric_rate = 1,
};
#endif
#if ES7243l_CHANNELS_MAX > 6
static struct snd_soc_dai_driver es7243l_dai3 = {
	.name = "ES7243l HiFi 3",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es7243l_RATES,
		.formats = es7243l_FORMATS,
	},
	.ops = &es7243l_ops,
	.symmetric_rate = 1,
};
#endif
#if ES7243l_CHANNELS_MAX > 8
static struct snd_soc_dai_driver es7243l_dai5 = {
	.name = "ES7243l HiFi 4",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es7243l_RATES,
		.formats = es7243l_FORMATS,
	},
	.ops = &es7243l_ops,
	.symmetric_rate = 1,
};
#endif
#if ES7243l_CHANNELS_MAX > 10
static struct snd_soc_dai_driver es7243l_dai6 = {
	.name = "ES7243l HiFi 5",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es7243l_RATES,
		.formats = es7243l_FORMATS,
	},
	.ops = &es7243l_ops,
	.symmetric_rate = 1,
};
#endif
#if ES7243l_CHANNELS_MAX > 12
static struct snd_soc_dai_driver es7243l_dai7 = {
	.name = "ES7243l HiFi 6",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es7243l_RATES,
		.formats = es7243l_FORMATS,
	},
	.ops = &es7243l_ops,
	.symmetric_rate = 1,
};
#endif
#if ES7243l_CHANNELS_MAX > 14
static struct snd_soc_dai_driver es7243l_dai8 = {
	.name = "ES7243l HiFi 7",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es7243l_RATES,
		.formats = es7243l_FORMATS,
	},
	.ops = &es7243l_ops,
	.symmetric_rate = 1,
};
#endif
static struct snd_soc_dai_driver *es7243l_dai[] = {
#if ES7243l_CHANNELS_MAX > 0
	&es7243l_dai0,
#endif
#if ES7243l_CHANNELS_MAX > 2
	&es7243l_dai1,
#endif
#if ES7243l_CHANNELS_MAX > 4
	&es7243l_dai2,
#endif
#if ES7243l_CHANNELS_MAX > 6
	&es7243l_dai3,
#endif
#if ES7243l_CHANNELS_MAX > 8
	&es7243l_dai4,
#endif
#if ES7243l_CHANNELS_MAX > 10
	&es7243l_dai5,
#endif
#if ES7243l_CHANNELS_MAX > 12
	&es7243l_dai6,
#endif
#if ES7243l_CHANNELS_MAX > 14
	&es7243l_dai7,
#endif
};

static int es7243l_suspend(struct snd_soc_component *component)
{
	es7243l_set_bias_level(component, SND_SOC_BIAS_OFF);
	return 0;
}

static int es7243l_resume(struct snd_soc_component *component)
{
	es7243l_set_bias_level(component, SND_SOC_BIAS_STANDBY);
	return 0;
}

struct _mclk_lrck_ratio {
	u16 ratio;      //ratio between mclk and lrck
	u8 nfs;         //nfs mode, =0, nfs mode disabled
	u8 osr;         //adc over sample rate
	u8 prediv_premulti; //adcclk and dacclk divider
	u8 cf_dsp_div;      //adclrck divider and daclrck divider
	u8 scale;
};

/* codec hifi mclk clock divider coefficients */
static const struct _mclk_lrck_ratio ratio_div[] = {
	//ratio     nfs,  osr,  pre,  div  ,scale
	{3072, 0, 0x20, 0x50, 0x00, 0x00},
	{3072, 2, 0x20, 0xb0, 0x00, 0x00},

	{2048, 0, 0x20, 0x30, 0x00, 0x00},
	{2048, 2, 0x20, 0x70, 0x00, 0x00},
	{2048, 3, 0x20, 0xb0, 0x00, 0x00},
	{2048, 4, 0x20, 0xf0, 0x00, 0x00},

	{1536, 0, 0x20, 0x20, 0x00, 0x00},
	{1536, 2, 0x20, 0x50, 0x00, 0x00},
	{1536, 3, 0x20, 0x80, 0x00, 0x00},
	{1536, 4, 0x20, 0xb0, 0x00, 0x00},

	{1024, 0, 0x20, 0x10, 0x00, 0x00},
	{1024, 2, 0x20, 0x30, 0x00, 0x00},
	{1024, 3, 0x20, 0x50, 0x00, 0x00},
	{1024, 4, 0x20, 0x70, 0x00, 0x00},
	{1024, 5, 0x20, 0x90, 0x00, 0x00},
	{1024, 6, 0x20, 0xb0, 0x00, 0x00},
	{1024, 7, 0x20, 0xd0, 0x00, 0x00},
	{1024, 8, 0x20, 0xf0, 0x00, 0x00},

	{768, 0, 0x20, 0x21, 0x00, 0x00},
	{768, 2, 0x20, 0x20, 0x00, 0x00},
	{768, 3, 0x20, 0x81, 0x00, 0x00},
	{768, 4, 0x20, 0x50, 0x00, 0x00},
	{768, 5, 0x20, 0xe1, 0x00, 0x00},
	{768, 6, 0x20, 0x80, 0x00, 0x00},
	{768, 8, 0x20, 0xb0, 0x00, 0x00},

	{512, 0, 0x20, 0x00, 0x00, 0x00},
	{512, 2, 0x20, 0x10, 0x00, 0x00},
	{512, 3, 0x20, 0x20, 0x00, 0x00},
	{512, 4, 0x20, 0x30, 0x00, 0x00},
	{512, 5, 0x20, 0x40, 0x00, 0x00},
	{512, 6, 0x20, 0x50, 0x00, 0x00},
	{512, 7, 0x20, 0x60, 0x00, 0x00},
	{512, 8, 0x20, 0x70, 0x00, 0x00},

	{384, 0, 0x20, 0x22, 0x00, 0x00},
	{384, 2, 0x20, 0x21, 0x00, 0x00},
	{384, 3, 0x20, 0x82, 0x00, 0x00},
	{384, 4, 0x20, 0x20, 0x00, 0x00},
	{384, 5, 0x20, 0xe2, 0x00, 0x00},
	{384, 6, 0x20, 0x81, 0x00, 0x00},
	{384, 8, 0x20, 0x50, 0x00, 0x00},

	{256, 0, 0x20, 0x01, 0x00, 0x00},
	{256, 2, 0x20, 0x00, 0x00, 0x00},
	{256, 3, 0x20, 0x21, 0x00, 0x00},
	{256, 4, 0x20, 0x10, 0x00, 0x00},
	{256, 5, 0x20, 0x41, 0x00, 0x00},
	{256, 6, 0x20, 0x20, 0x00, 0x00},
	{256, 7, 0x20, 0x61, 0x00, 0x00},
	{256, 8, 0x20, 0x30, 0x00, 0x00},

	{192, 0, 0x20, 0x23, 0x00, 0x00},
	{192, 2, 0x20, 0x22, 0x00, 0x00},
	{192, 3, 0x20, 0x83, 0x00, 0x00},
	{192, 4, 0x20, 0x21, 0x00, 0x00},
	{192, 5, 0x20, 0xe3, 0x00, 0x00},
	{192, 6, 0x20, 0x82, 0x00, 0x00},
	{192, 8, 0x20, 0x20, 0x00, 0x00},

	{128, 0, 0x20, 0x02, 0x00, 0x00},
	{128, 2, 0x20, 0x01, 0x00, 0x00},
	{128, 3, 0x20, 0x22, 0x00, 0x00},
	{128, 4, 0x20, 0x00, 0x00, 0x00},
	{128, 5, 0x20, 0x42, 0x00, 0x00},
	{128, 6, 0x20, 0x21, 0x00, 0x00},
	{128, 7, 0x20, 0x62, 0x00, 0x00},
	{128, 8, 0x20, 0x10, 0x00, 0x00},

	{64, 0, 0x20, 0x03, 0x00, 0x00},
	{64, 2, 0x20, 0x02, 0x00, 0x00},
	{64, 3, 0x20, 0x23, 0x00, 0x00},
	{64, 4, 0x20, 0x01, 0x00, 0x00},
	{64, 5, 0x20, 0x43, 0x00, 0x00},
	{64, 6, 0x20, 0x22, 0x00, 0x00},
	{64, 7, 0x20, 0x63, 0x00, 0x00},
	{64, 8, 0x20, 0x00, 0x00, 0x00},
};

static inline int get_mclk_lrck_ratio(int clk_ratio, int n_fs)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(ratio_div); i++) {
		if (ratio_div[i].ratio == clk_ratio
		    && ratio_div[i].nfs == n_fs)

		{
			return i;
		}
	}
	return -EINVAL;
}

static int es7243l_probe(struct snd_soc_component *component)
{
	struct es7243l_priv *es7243l = snd_soc_component_get_drvdata(component);
	u8 index, regv, chn, work_mode, ratio_index, datbits;
	u16 ratio;
	u8 digital_vol[16], pga_gain[16];

	//  printk("begin->>>>>>>>>>%s!\n", __func__);
	/* Enable the following code if there is no mclk.
	 * a clock named "mclk" need to be defined in the dts (see sample dts)
	 *
	 * No need to enable the following code to get mclk if:
	 * 1. sclk/bclk is used as mclk
	 * 2. mclk is controled by soc I2S
	 */
#if 0
	es7243l->mclk = devm_clk_get(codec->dev, "mclk");
	if (IS_ERR(es7243l->mclk)) {
		dev_err(codec->dev, "%s,unable to get mclk\n", __func__);
		return PTR_ERR(es7243l->mclk);
	}
	if (!es7243l->mclk) {
		dev_err(codec->dev, "%s, assuming static mclk\n", __func__);
	}

	ret = clk_prepare_enable(es7243l->mclk);
	if (ret) {
		dev_err(codec->dev, "%s, unable to enable mclk\n", __func__);
		return ret;
	}
#endif
	tron_codec[es7243l_codec_num++] = component;
	index = 0;
#if ES7243l_CHANNELS_MAX > 0
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_1;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN1_PGA;
	index++;
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_2;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN2_PGA;
	index++;
#endif
#if ES7243l_CHANNELS_MAX > 2
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_3;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN3_PGA;
	index++;
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_4;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN4_PGA;
	index++;
#endif
#if ES7243l_CHANNELS_MAX > 4
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_5;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN5_PGA;
	index++;
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_6;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN6_PGA;
	index++;
#endif
#if ES7243l_CHANNELS_MAX > 6
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_7;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN7_PGA;
	index++;
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_8;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN8_PGA;
	index++;
#endif
#if ES7243l_CHANNELS_MAX > 8
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_9;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN9_PGA;
	index++;
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_10;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN10_PGA;
	index++;
#endif
#if ES7243l_CHANNELS_MAX > 10
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_11;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN11_PGA;
	index++;
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_12;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN12_PGA;
	index++;
#endif
#if ES7243l_CHANNELS_MAX > 12
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_13;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN13_PGA;
	index++;
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_14;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN14_PGA;
	index++;
#endif
#if ES7243l_CHANNELS_MAX > 14
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_15;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN15_PGA;
	index++;
	digital_vol[index] = ES7243l_DIGITAL_VOLUME_16;
	pga_gain[index] = ES7243l_MIC_ARRAY_AIN16_PGA;
	index++;
#endif

	for (index = 0; index < (ES7243l_CHANNELS_MAX) / 2; index++) {
		es7243l_read(ES7243l_CLK2, &regv, i2c_clt[index]);
		if (es7243l->mclksrc == FROM_MCLK_PIN) {
			regv &= 0x7f;
		} else {
			regv |= 0x80;
		}
		regv &= 0xfd;
		if (es7243l->mclkinv == true) {
			regv |= 0x20;
		}
		regv &= 0xfe;
		if (es7243l->bclkinv == true) {
			regv |= 0x01;
		}
		es7243l_write(ES7243l_CLK2, regv, i2c_clt[index]);
		/*
		 * set data bits
		 */
		es7243l_read(ES7243l_SDP_FORMAT, &regv, i2c_clt[index]);
		regv &= 0xe3;
		datbits = ES7243l_DATA_LENGTH;
		switch (datbits) {
			case DATA_16BITS:
				regv |= 0x0c;
				break;
			case DATA_24BITS:
				break;
			case DATA_32BITS:
				regv |= 0x10;
				break;
			default:
				regv |= 0x0c;
				break;
		}
		es7243l_write(ES7243l_SDP_FORMAT, regv, i2c_clt[index]);
		/*
		 * set sdp format and tdm mode
		 */
		chn = ES7243l_CHANNELS_MAX / 2;
		switch (es7243l->tdm) {
			case ES7243l_NORMAL_I2S:
				es7243l_read(ES7243l_SDP_FORMAT, &regv, i2c_clt[index]);
				regv &= 0xfc;
				es7243l_write(ES7243l_SDP_FORMAT, regv, i2c_clt[index]);
				es7243l_read(ES7243l_TDM, &regv, i2c_clt[index]);
				regv &= 0xc0;
				es7243l_write(ES7243l_TDM, regv, i2c_clt[index]);
				work_mode = 0;
				break;
			case ES7243l_NORMAL_LJ:
				es7243l_read(ES7243l_SDP_FORMAT, &regv, i2c_clt[index]);
				regv &= 0xfc;
				regv |= 0x01;
				es7243l_write(ES7243l_SDP_FORMAT, regv, i2c_clt[index]);
				es7243l_read(ES7243l_TDM, &regv, i2c_clt[index]);
				regv &= 0xc0;
				es7243l_write(ES7243l_TDM, regv, i2c_clt[index]);
				work_mode = 0;
				break;
			case ES7243l_NORMAL_DSPA:
				es7243l_read(ES7243l_SDP_FORMAT, &regv, i2c_clt[index]);
				regv &= 0xdc;
				regv |= 0x03;
				es7243l_write(ES7243l_SDP_FORMAT, regv, i2c_clt[index]);
				es7243l_read(ES7243l_TDM, &regv, i2c_clt[index]);
				regv &= 0xc0;
				es7243l_write(ES7243l_TDM, regv, i2c_clt[index]);
				work_mode = 0;
				break;
			case ES7243l_NORMAL_DSPB:
				es7243l_read(ES7243l_SDP_FORMAT, &regv, i2c_clt[index]);
				regv &= 0xdc;
				regv |= 0x23;
				es7243l_write(ES7243l_SDP_FORMAT, regv, i2c_clt[index]);
				es7243l_read(ES7243l_TDM, &regv, i2c_clt[index]);
				regv &= 0xc0;
				es7243l_write(ES7243l_TDM, regv, i2c_clt[index]);
				work_mode = 0;
				break;
			case ES7243l_TDM_A:
				es7243l_read(ES7243l_SDP_FORMAT, &regv, i2c_clt[index]);
				regv &= 0xdc;
				regv |= 0x03;
				es7243l_write(ES7243l_SDP_FORMAT, regv, i2c_clt[index]);
				es7243l_read(ES7243l_TDM, &regv, i2c_clt[index]);
				regv &= 0xc0;
				regv |= 0x08;
				es7243l_write(ES7243l_TDM, regv, i2c_clt[index]);
				work_mode = 0;
				break;
			case ES7243l_NFS:
				es7243l_read(ES7243l_SDP_FORMAT, &regv, i2c_clt[index]);
				regv &= 0xfc;
				es7243l_write(ES7243l_SDP_FORMAT, regv, i2c_clt[index]);
				es7243l_read(ES7243l_TDM, &regv, i2c_clt[index]);
				regv &= 0xc0;
				switch (chn) {
					case 2:
						regv |= 0x01;
						work_mode = 2;
						break;
					case 3:
						regv |= 0x02;
						work_mode = 3;
						break;
					case 4:
						regv |= 0x03;
						work_mode = 4;
						break;
					case 5:
						regv |= 0x04;
						work_mode = 5;
						break;
					case 6:
						regv |= 0x05;
						work_mode = 6;
						break;
					case 7:
						regv |= 0x06;
						work_mode = 7;
						break;
					case 8:
						regv |= 0x07;
						work_mode = 8;
						break;
					default:
						work_mode = 0;
						break;
				}
				/*
				 * the last chip generate flag bits, others chip in sync mode
				 */
				if (index == ((ES7243l_CHANNELS_MAX / 2) - 1)) {
					regv |= 0x10;
				} else {
					regv |= 0x20;
				}
				es7243l_write(ES7243l_TDM, regv, i2c_clt[index]);
				break;
			default:
				work_mode = 0;
				break;
		}

		/*
		 * set clock divider and multiplexer according clock ratio and nfs mode
		 */
		ratio = ES7243l_MCLK_LRCK_RATIO;
		ratio_index = get_mclk_lrck_ratio(ratio, work_mode);
		if (ratio_index < 0) {
			//          printk
			//              ("can't get configuration for %d ratio with %d es7243l",
			//               ratio, work_mode);
			es7243l_write(ES7243l_ADC_OSR, 0x20, i2c_clt[index]);
			es7243l_write(ES7243l_ADCCTL1, 0x00, i2c_clt[index]);
			es7243l_write(ES7243l_PREDIV, 0x00, i2c_clt[index]);
			es7243l_write(ES7243l_CLK_DIV, 0x00, i2c_clt[index]);
		} else {
			es7243l_write(ES7243l_ADC_OSR, ratio_div[ratio_index].osr,
			              i2c_clt[index]);
			es7243l_write(ES7243l_ADCCTL1, ratio_div[ratio_index].scale,
			              i2c_clt[index]);
			es7243l_write(ES7243l_PREDIV,
			              ratio_div
			              [ratio_index].prediv_premulti,
			              i2c_clt[index]);
			es7243l_write(ES7243l_CLK_DIV,
			              ratio_div[ratio_index].cf_dsp_div,
			              i2c_clt[index]);
		}

		es7243l_write(ES7243l_S1_SEL, 0xe0, i2c_clt[index]);
		es7243l_write(ES7243l_S3_SEL, 0xa0, i2c_clt[index]);

		es7243l_write(ES7243l_ADC_VOL, digital_vol[index * 2], i2c_clt[index]);
		//  es7243l_write(ES7243l_ADC_VOL, 0xcf, i2c_clt[index]);
		es7243l_write(ES7243l_ADC_HPF1, 0x0c, i2c_clt[index]);
		es7243l_write(ES7243l_ADC_HPF2, 0x0c, i2c_clt[index]);

		es7243l_write(ES7243l_ADCCTL3, 0x16, i2c_clt[index]);
		es7243l_write(ES7243l_ADCCTL4, 0x16, i2c_clt[index]);

		es7243l_read(0xfd, &regv, i2c_clt[index]);
		if (0x72 == regv) {
			//7243l
			es7243l_write(ES7243l_ADCCTL2, 0x80, i2c_clt[index]);
			es7243l_write(ES7243l_ANALOG_PDN, 0x00, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_BIAS_0x18, 0x26, i2c_clt[index]);
			es7243l_write(ES7243l_VMIDSEL, 0x02, i2c_clt[index]);
			es7243l_write(ES7243l_PGA_BIAS, 0xAA, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_BIAS_0x1A, 0xf4, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_MICBIAS, 0x66, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_VRPBIAS, 0x44, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_LP, 0x00, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_PGA_LP, 0x00, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_VMID, 0x00, i2c_clt[index]);
		} else if (0x7a == regv) {
			//7243e
#if AVDD_3V3
			es7243l_write(ES7243l_ADCCTL2, 0x80, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_BIAS_0x18, 0x26, i2c_clt[index]);
			es7243l_write(ES7243l_VMIDSEL, 0x02, i2c_clt[index]);
			es7243l_write(ES7243l_PGA_BIAS, 0x77, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_BIAS_0x1A, 0xf4, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_MICBIAS, 0x66, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_VRPBIAS, 0x44, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_LP, 0x3c, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_PGA_LP, 0x00, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_VMID, 0x0c, i2c_clt[index]);
#else
			es7243l_write(ES7243l_ADCCTL2, 0x80, i2c_clt[index]);
			es7243l_write(ES7243l_ANALOG_PDN, 0x00, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_BIAS_0x18, 0x26, i2c_clt[index]);
			es7243l_write(ES7243l_VMIDSEL, 0x02, i2c_clt[index]);
			es7243l_write(ES7243l_PGA_BIAS, 0x66, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_BIAS_0x1A, 0x44, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_MICBIAS, 0x44, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_VRPBIAS, 0x44, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_LP, 0x3c, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_PGA_LP, 0x0f, i2c_clt[index]);
			es7243l_write(ES7243l_ADC_VMID, 0x07, i2c_clt[index]);
#endif
		}
		//es7243l_write(ES7243l_RESET, 0x80, i2c_clt[index]);
		es7243l_write(ES7243l_CLK1, 0x3a, i2c_clt[index]);
		es7243l_write(ES7243l_ANALOG_PDN, 0x00, i2c_clt[index]);

		es7243l_write(ES7243l_PGA1, (0x10 | pga_gain[index * 2]),
		              i2c_clt[index]);
		es7243l_write(ES7243l_PGA2, (0x10 | pga_gain[index * 2 + 1]),
		              i2c_clt[index]);

		/*
		 * reset PGA
		 */
		msleep(100);
		es7243l_write(ES7243l_ANALOG_PDN, 0x03, i2c_clt[index]);
		msleep(100);
		es7243l_write(ES7243l_ANALOG_PDN, 0x00, i2c_clt[index]);
	}
	return 0;
}

static void es7243l_remove(struct snd_soc_component *component)
{
	es7243l_set_bias_level(component, SND_SOC_BIAS_OFF);
}

static const struct snd_soc_component_driver soc_codec_dev_es7243l = {
	.probe = es7243l_probe,
	.remove = es7243l_remove,
	.suspend = es7243l_suspend,
	.resume = es7243l_resume,
	.set_bias_level = es7243l_set_bias_level,

	.controls = es7243l_snd_controls,
	.num_controls = ARRAY_SIZE(es7243l_snd_controls),
	.dapm_widgets = es7243l_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(es7243l_dapm_widgets),
	.dapm_routes = es7243l_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(es7243l_dapm_routes),
};

static ssize_t
es7243l_store(struct device *dev,
              struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0, flag = 0;
	u8 i = 0, reg, num, value_w, value_r;
	struct es7243l_priv *es7243l = dev_get_drvdata(dev);

	val = simple_strtol(buf, NULL, 16);
	flag = (val >> 16) & 0xFF;

	if (flag) {
		reg = (val >> 8) & 0xFF;
		value_w = val & 0xFF;
		printk("\nWrite: start REG:0x%02x,val:0x%02x,count:0x%02x\n",
		       reg, value_w, flag);
		while (flag--) {
			es7243l_write(reg, value_w, es7243l->i2c);
			printk("Write 0x%02x to REG:0x%02x\n", value_w, reg);
			reg++;
		}
	} else {
		reg = (val >> 8) & 0xFF;
		num = val & 0xff;
		printk("\nRead: start REG:0x%02x,count:0x%02x\n", reg, num);
		do {
			value_r = 0;
			es7243l_read(reg, &value_r, es7243l->i2c);
			printk("REG[0x%02x]: 0x%02x;  \n", reg, value_r);
			reg++;
			i++;
			if ((i == num) || (i % 4 == 0)) {
				printk("\n");
			}
		} while (i < num);
	}

	return count;
}

static ssize_t
es7243l_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("echo flag|reg|val > es7243l\n");
	printk("eg read star address=0x06,count 0x10:echo 0610 >es7243l\n");
	printk("eg write star address=0x90,value=0x3c,count=4:echo 4903c >es7243\n");
	printk("eg write value:0xfe to address:0x06 :echo 106fe > es7243\n");
	return 0;
}

static DEVICE_ATTR(es7243l, 0644, es7243l_show, es7243l_store);

static struct attribute *es7243l_debug_attrs[] = {
	&dev_attr_es7243l.attr,
	NULL,
};

static struct attribute_group es7243l_debug_attr_group = {
	.name = "es7243l_debug",
	.attrs = es7243l_debug_attrs,
};

/*
 * If the i2c layer weren't so broken, we could pass this kind of data
 * around
 */
static int
es7243l_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *i2c_id)
{
	struct es7243l_priv *es7243l;
	int ret;

	unsigned int val;
	printk("begin->>>>>>>>>>%s!\n", __func__);

	es7243l = devm_kzalloc(&i2c->dev, sizeof(struct es7243l_priv),
	                       GFP_KERNEL);
	if (es7243l == NULL) {
		return -ENOMEM;
	}
	es7243l->i2c = i2c;
	es7243l->tdm = ES7243l_WORK_MODE;   //to initialize tdm mode
	es7243l->mclksrc = ES7243l_MCLK_SOURCE;
	es7243l->dmic = DMIC_INTERFACE;
	es7243l->mclkinv = MCLK_INVERTED_OR_NOT;
	es7243l->bclkinv = BCLK_INVERTED_OR_NOT;
	es7243l->vdda = VDDA_VOLTAGE;
	es7243l->regmap = devm_regmap_init_i2c(i2c, &es7243l_regmap_config);
	if (IS_ERR(es7243l->regmap)) {
		ret = PTR_ERR(es7243l->regmap);
		dev_err(&i2c->dev, "regmap_init() failed: %d\n", ret);
		return ret;
	}
	/* verify that we have an es7243l */
	ret = regmap_read(es7243l->regmap, ES7243l_CHIPID1, &val);
	if (ret < 0) {
		dev_err(&i2c->dev, "failed to read i2c at addr %X\n",
		        i2c->addr);
		return ret;
	}

	/* The first ID should be 0x72 or 7a. */
	if (!(val == 0x7a || val == 0x72)) {
		dev_err(&i2c->dev, "device at addr %X is not an es7243l\n",
		        i2c->addr);
		return -ENODEV;
	}
	ret = regmap_read(es7243l->regmap, ES7243l_CHIPID2, &val);
	/* The NEXT ID should be 0x43. */
	if (val != 0x43) {
		dev_err(&i2c->dev, "device at addr %X is not an es7243l\n",
		        i2c->addr);
		return -ENODEV;
	}
	dev_set_drvdata(&i2c->dev, es7243l);

	if (i2c_id->driver_data < (ES7243l_CHANNELS_MAX) / 2) {
		i2c_clt[i2c_id->driver_data] = i2c;
		ret = snd_soc_register_component(&i2c->dev,
		                                 &soc_codec_dev_es7243l,
		                                 es7243l_dai
		                                 [i2c_id->driver_data], 1);
		if (ret < 0) {
			kfree(es7243l);
			printk("%s(), failed to register codec device\n", __func__);
			return ret;
		}
	}
	ret = sysfs_create_group(&i2c->dev.kobj, &es7243l_debug_attr_group);
	if (ret) {
		pr_err("failed to create attr group\n");
	}
	return ret;
}

static void __exit es7243l_i2c_remove(struct i2c_client *i2c)
{
	//snd_soc_unregister_codec(&i2c->dev);
	kfree(i2c_get_clientdata(i2c));
}

#if !ES7243l_MATCH_DTS_EN
static int
es7243l_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	if (adapter->nr == ES7243l_I2C_BUS_NUM) {
#if ES7243l_CHANNELS_MAX > 0
		if (client->addr == ES7243l_I2C_CHIP_ADDRESS_0) {
			strlcpy(info->type, "MicArray_0", I2C_NAME_SIZE);
			return 0;
		}
#endif
#if ES7243l_CHANNELS_MAX > 2
		else if (client->addr == ES7243l_I2C_CHIP_ADDRESS_1) {
			strlcpy(info->type, "MicArray_1", I2C_NAME_SIZE);
			return 0;
		}
#endif
#if ES7243l_CHANNELS_MAX > 4
		else if (client->addr == ES7243l_I2C_CHIP_ADDRESS_2) {
			strlcpy(info->type, "MicArray_2", I2C_NAME_SIZE);
			return 0;
		}
#endif
#if ES7243l_CHANNELS_MAX > 6
		else if (client->addr == ES7243l_I2C_CHIP_ADDRESS_3) {
			strlcpy(info->type, "MicArray_3", I2C_NAME_SIZE);
			return 0;
		}
#endif
#if ES7243l_CHANNELS_MAX > 8
		else if (client->addr == ES7243l_I2C_CHIP_ADDRESS_4) {
			strlcpy(info->type, "MicArray_4", I2C_NAME_SIZE);
			return 0;
		}
#endif
#if ES7243l_CHANNELS_MAX > 10
		else if (client->addr == ES7243l_I2C_CHIP_ADDRESS_5) {
			strlcpy(info->type, "MicArray_5", I2C_NAME_SIZE);
			return 0;
		}
#endif
#if ES7243l_CHANNELS_MAX > 12
		else if (client->addr == ES7243l_I2C_CHIP_ADDRESS_6) {
			strlcpy(info->type, "MicArray_6", I2C_NAME_SIZE);
			return 0;
		}
#endif
#if ES7243l_CHANNELS_MAX > 14
		else if (client->addr == ES7243l_I2C_CHIP_ADDRESS_7) {
			strlcpy(info->type, "MicArray_7", I2C_NAME_SIZE);
			return 0;
		}
#endif
	}
	return -ENODEV;
}
#endif

static const unsigned short es7243l_i2c_addr[] = {
#if ES7243l_CHANNELS_MAX > 0
	ES7243l_I2C_CHIP_ADDRESS_0,
#endif

#if ES7243l_CHANNELS_MAX > 2
	ES7243l_I2C_CHIP_ADDRESS_1,
#endif

#if ES7243l_CHANNELS_MAX > 4
	ES7243l_I2C_CHIP_ADDRESS_2,
#endif

#if ES7243l_CHANNELS_MAX > 6
	ES7243l_I2C_CHIP_ADDRESS_3,
#endif

#if ES7243l_CHANNELS_MAX > 8
	ES7243l_I2C_CHIP_ADDRESS_4,
#endif

#if ES7243l_CHANNELS_MAX > 10
	ES7243l_I2C_CHIP_ADDRESS_5,
#endif

#if ES7243l_CHANNELS_MAX > 12
	ES7243l_I2C_CHIP_ADDRESS_6,
#endif

#if ES7243l_CHANNELS_MAX > 14
	ES7243l_I2C_CHIP_ADDRESS_7,
#endif

	I2C_CLIENT_END,
};

/*
* device tree source or i2c_board_info both use to transfer hardware information to linux kernel,
* use one of them wil be OK
*/
#if !ES7243l_MATCH_DTS_EN
static struct i2c_board_info es7243l_i2c_board_info[] = {
#if ES7243l_CHANNELS_MAX > 0
	{I2C_BOARD_INFO("MicArray_0", ES7243l_I2C_CHIP_ADDRESS_0),},    //es7243l_0
#endif

#if ES7243l_CHANNELS_MAX > 2
	{I2C_BOARD_INFO("MicArray_1", ES7243l_I2C_CHIP_ADDRESS_1),},    //es7243l_1
#endif

#if ES7243l_CHANNELS_MAX > 4
	{I2C_BOARD_INFO("MicArray_2", ES7243l_I2C_CHIP_ADDRESS_2),},    //es7243l_2
#endif

#if ES7243l_CHANNELS_MAX > 6
	{I2C_BOARD_INFO("MicArray_3", ES7243l_I2C_CHIP_ADDRESS_3),},    //es7243l_3
#endif
#if ES7243l_CHANNELS_MAX > 8
	{I2C_BOARD_INFO("MicArray_4", ES7243l_I2C_CHIP_ADDRESS_4),},    //es7243l_4
#endif

#if ES7243l_CHANNELS_MAX > 10
	{I2C_BOARD_INFO("MicArray_5", ES7243l_I2C_CHIP_ADDRESS_5),},    //es7243l_5
#endif

#if ES7243l_CHANNELS_MAX > 12
	{I2C_BOARD_INFO("MicArray_6", ES7243l_I2C_CHIP_ADDRESS_6),},    //es7243l_6
#endif

#if ES7243l_CHANNELS_MAX > 14
	{I2C_BOARD_INFO("MicArray_7", ES7243l_I2C_CHIP_ADDRESS_7),},    //es7243l_7
#endif
};
#endif
static const struct i2c_device_id es7243l_i2c_id[] = {
#if ES7243l_CHANNELS_MAX > 0
	{"MicArray_0", 0},  //es7243l_0
#endif

#if ES7243l_CHANNELS_MAX > 2
	{"MicArray_1", 1},  //es7243l_1
#endif

#if ES7243l_CHANNELS_MAX > 4
	{"MicArray_2", 2},  //es7243l_2
#endif

#if ES7243l_CHANNELS_MAX > 6
	{"MicArray_3", 3},  //es7243l_3
#endif

#if ES7243l_CHANNELS_MAX > 8
	{"MicArray_4", 4},  //es7243l_4
#endif

#if ES7243l_CHANNELS_MAX > 10
	{"MicArray_5", 5},  //es7243l_5
#endif

#if ES7243l_CHANNELS_MAX > 12
	{"MicArray_6", 6},  //es7243l_6
#endif

#if ES7243l_CHANNELS_MAX > 14
	{"MicArray_7", 7},  //es7243l_7
#endif
	{}
};

MODULE_DEVICE_TABLE(i2c, es7243l_i2c_id);

static const struct of_device_id es7243l_dt_ids[] = {
#if ES7243l_CHANNELS_MAX > 0
	{.compatible = "MicArray_0",},  //es7243l_0
#endif

#if ES7243l_CHANNELS_MAX > 2
	{.compatible = "MicArray_1",},  //es7243l_1
#endif

#if ES7243l_CHANNELS_MAX > 4
	{.compatible = "MicArray_2",},  //es7243l_2
#endif

#if ES7243l_CHANNELS_MAX > 6
	{.compatible = "MicArray_3",},  //es7243l_3
#endif

#if ES7243l_CHANNELS_MAX > 8
	{.compatible = "MicArray_4",},  //es7243l_4
#endif

#if ES7243l_CHANNELS_MAX > 10
	{.compatible = "MicArray_5",},  //es7243l_5
#endif

#if ES7243l_CHANNELS_MAX > 12
	{.compatible = "MicArray_6",},  //es7243l_6
#endif

#if ES7243l_CHANNELS_MAX > 14
	{.compatible = "MicArray_7",},  //es7243l_7
#endif
	{},
};

MODULE_DEVICE_TABLE(of, es7243l_dt_ids);

static struct i2c_driver es7243l_i2c_driver = {
	.driver = {
		.name = "es7243l",
		.owner = THIS_MODULE,
#if ES7243l_MATCH_DTS_EN
		.of_match_table = es7243l_dt_ids,
#endif
	},
	.probe = es7243l_i2c_probe,
	.remove = __exit_p(es7243l_i2c_remove),
	.class = I2C_CLASS_HWMON,
	.id_table = es7243l_i2c_id,

#if !ES7243l_MATCH_DTS_EN
	.address_list = es7243l_i2c_addr,
	.detect = es7243l_i2c_detect,
#endif

};

static int __init es7243l_modinit(void)
{
	int ret;
#if 0
	int i;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	adapter = i2c_get_adapter(ES7243l_I2C_BUS_NUM);
	if (!adapter) {
		printk("i2c_get_adapter() fail!\n");
		return -ENODEV;
	}
	printk("%s() begin0000\n", __func__);

	for (i = 0; i < ES7243l_CHANNELS_MAX / 2; i++) {
		client = i2c_new_device(adapter, &es7243l_i2c_board_info[i]);
		printk("%s() i2c_new_device\n", __func__);
		if (!client) {
			return -ENODEV;
		}
	}
	i2c_put_adapter(adapter);
#endif
	ret = i2c_add_driver(&es7243l_i2c_driver);
	if (ret != 0) {
		printk("Failed to register es7243 i2c driver : %d \n", ret);
	}
	return ret;
}

//late_initcall(es7243l_modinit);
module_init(es7243l_modinit);
static void __exit es7243l_exit(void)
{
	i2c_del_driver(&es7243l_i2c_driver);
}

module_exit(es7243l_exit);
MODULE_DESCRIPTION("ASoC ES7243l audio adc driver");
MODULE_AUTHOR("David Yang <yangxiaohua@everest-semi.com>");
MODULE_LICENSE("GPL v2");
