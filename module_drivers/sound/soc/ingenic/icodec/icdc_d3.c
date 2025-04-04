/*
 * sound/soc/ingenic/icodec/icdc_d3.c
 * ALSA SoC Audio driver -- ingenic internal codec (icdc_d3) driver

 * Copyright 2014 Ingenic Semiconductor Co.,Ltd
 *  cscheng <shicheng.cheng@ingenic.com>
 *
 * Note: icdc_d3 is an internal codec for ingenic SOC
 *   used for x1000 and so on
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>
#include <sound/soc-dai.h>
#include <linux/of_address.h>
#include "icdc_d3.h"

static int icdc_d3_debug = 0;
module_param(icdc_d3_debug, int, 0644);
#define DEBUG_MSG(msg...)           \
	do {                    \
		if (icdc_d3_debug)      \
			printk("ICDC: " msg);   \
	} while(0)

static u8 icdc_d3_reg_defcache[SCODA_MAX_REG_NUM] = {
	/* reg 0x0 ... 0x9 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd3, 0xd3,
	/* reg 0xa ... 0x13 */
	0x00, 0x30, 0x30, 0xb0, 0xb1, 0xb0, 0x00, 0x00, 0x0f, 0x40,
	/* reg 0x14 ... 0x1d */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff,
	/* reg 0x1e ... 0x27 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* reg 0x28 ... 0x31 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* reg 0x32 ... 0x39 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* extern reg */
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x34, 0x07, 0x44, 0x1f, 0x00,
};

static int icdc_d3_volatile(struct snd_soc_codec *codec, unsigned int reg)
{
	if (reg > SCODA_MAX_REG_NUM) {
		return 1;
	}

	switch (reg) {
	case SCODA_REG_SR:
	case SCODA_REG_SR2:
	case SCODA_REG_SIGR:
	case SCODA_REG_SIGR3:
	case SCODA_REG_SIGR5:
	case SCODA_REG_SIGR7:
	case SCODA_REG_MR:
	case SCODA_REG_IFR:
	case SCODA_REG_IFR2:
	case SCODA_REG_SR_ADC_AGCDGL:
	case SCODA_REG_SR_ADC_AGCDGR:
	case SCODA_REG_SR_ADC_AGCAGL:
	case SCODA_REG_SR_ADC_AGCAGR:
	case SCODA_REG_SR_TR1:
	case SCODA_REG_SR_TR2:
	case SCODA_REG_SR_TR_SRCDAC:
		return 1;
	default:
		return 0;
	}
}

static int icdc_d3_writable(struct snd_soc_codec *codec, unsigned int reg)
{
	if (reg > SCODA_MAX_REG_NUM) {
		return 0;
	}

	switch (reg) {
	case SCODA_REG_SR:
	case SCODA_REG_SR2:
	case SCODA_REG_SIGR:
	case SCODA_REG_SIGR3:
	case SCODA_REG_SIGR5:
	case SCODA_REG_SIGR7:
	case SCODA_REG_MR:
	case SCODA_REG_SR_ADC_AGCDGL:
	case SCODA_REG_SR_ADC_AGCDGR:
	case SCODA_REG_SR_ADC_AGCAGL:
	case SCODA_REG_SR_ADC_AGCAGR:
	case SCODA_REG_SR_TR1:
	case SCODA_REG_SR_TR2:
	case SCODA_REG_SR_TR_SRCDAC:
		return 0;
	default:
		return 1;
	}
}

static int icdc_d3_readable(struct snd_soc_codec *codec, unsigned int reg)
{
	if (reg > SCODA_MAX_REG_NUM) {
		return 0;
	} else {
		return 1;
	}
}

static void dump_registers_hazard(struct icdc_d3 *icdc_d3)
{
	int reg = 0;
	dev_info(icdc_d3->dev, "-------------------register:");
	for (; reg < SCODA_MAX_REG_NUM; reg++) {
		if (reg % 8 == 0) {
			printk("\n");
		}
		if (icdc_d3_readable(icdc_d3->codec, reg)) {
			printk(" 0x%02x:0x%02x,", reg, icdc_d3_hw_read(icdc_d3, reg));
		} else {
			printk(" 0x%02x:0x%02x,", reg, 0x0);
		}
	}
	printk("\n");
	printk("mix_0=%02x\n", icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_0));
	printk("mix_1=%02x\n", icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_1));
	printk("mix_2=%02x\n", icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_2));
	printk("mix_3=%02x\n", icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_3));
	printk("mix_4=%02x\n", icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_4));

	printk("\n");
	dev_info(icdc_d3->dev, "----------------------------\n");
	return;
}

static int icdc_d3_write(struct snd_soc_codec *codec, unsigned int reg,
                         unsigned int value)
{
	struct icdc_d3 *icdc_d3 = snd_soc_codec_get_drvdata(codec);
	int val = value;
	BUG_ON(reg > SCODA_MAX_REG_NUM);
	dev_dbg(icdc_d3->dev, "%s reg = %x value = %x \n", __func__, reg, val);

	if (icdc_d3_writable(codec, reg)) {
		if (!icdc_d3_volatile(codec, reg)) {
			u8 *cache = codec->reg_cache;
			if ((reg == SCODA_REG_GCR_DACL) || (reg == SCODA_REG_GCR_DACR)) {
				if (val < 32) {
					val = 31 - val;
				} else {
					val = 95 - val;
				}
			}
			cache[reg] = val;
		}
		return icdc_d3_hw_write(icdc_d3, reg, val);
	}
	return 0;
}

static unsigned int icdc_d3_read(struct snd_soc_codec *codec, unsigned int reg)
{

	struct icdc_d3 *icdc_d3 = snd_soc_codec_get_drvdata(codec);
	int val = 0;
	BUG_ON(reg > SCODA_MAX_REG_NUM);

	if (!icdc_d3_volatile(codec, reg)) {
		u8 *cache = codec->reg_cache;
		val = cache[reg];
		if ((reg == SCODA_REG_GCR_DACL) || (reg == SCODA_REG_GCR_DACR)) {
			if (val < 32) {
				val = 31 - val;
			} else {
				val = 95 - val;
			}
		}
		return val;
	}

	if (icdc_d3_readable(codec, reg)) {
		return icdc_d3_hw_read(icdc_d3, reg);
	}

	return 0;
}

static int icdc_d3_hw_params(struct snd_pcm_substream *substream,
                             struct snd_pcm_hw_params *params,
                             struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	int playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);
	int bit_width_sel = 3;
	int speed_sel = 0;
	int aicr_reg = playback ? SCODA_REG_AICR_DAC : SCODA_REG_AICR_ADC;
	int fcr_reg = playback ? SCODA_REG_FCR_DAC : SCODA_REG_FCR_ADC;
	int sample_attr[] = {   8000, 11025, 12000, 16000, 22050,  24000, 32000, 44100,
	                        48000, 88200,  96000, 176400, 192000,
	                    };
	int speed = params_rate(params);
	int bit_width = params_format(params);
	DEBUG_MSG("%s enter  set bus width %d , sample rate %d\n",
	          __func__, bit_width, speed);
	/* bit width */
	switch (bit_width) {
	case SNDRV_PCM_FORMAT_S16_LE:
		bit_width_sel = 0;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		bit_width_sel = 1;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		bit_width_sel = 2;
		break;
	default:
	case SNDRV_PCM_FORMAT_S24_3LE:
		bit_width_sel = 3;
		break;
	}

	/*sample rate*/
	for (speed_sel = 0; speed > sample_attr[speed_sel]; speed_sel++)
		;
	snd_soc_update_bits(codec, aicr_reg, SCODA_AICR_DAC_ADWL_MASK,
	                    (bit_width_sel << SCODA_AICR_DAC_ADWL_SHIFT));
	snd_soc_update_bits(codec, fcr_reg, SCODA_FCR_FREQ_MASK,
	                    (speed_sel << SCODA_FCR_FREQ_SHIFT));
	return 0;
}

static int icdc_d3_trigger(struct snd_pcm_substream *stream, int cmd,
                           struct snd_soc_dai *dai)
{
#ifdef DEBUG
	struct snd_soc_codec *codec = dai->codec;
	struct icdc_d3 *icdc_d3 = snd_soc_codec_get_drvdata(codec);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dump_registers_hazard(icdc_d3);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dump_registers_hazard(icdc_d3);
		break;
	}
#endif
	return 0;
}

#define DLV4780_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S18_3LE | \
                         SNDRV_PCM_FMTBIT_S20_3LE |SNDRV_PCM_FMTBIT_S24_LE)

static int ingenic_icdc_startup(struct snd_pcm_substream *substream,
                                struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	/*power on codec*/
	if (snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 0)) {
		msleep(250);
	}
	if (snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 0)) {
		msleep(400);
	}

	return 0;
}

static void ingenic_icdc_shutdown(struct snd_pcm_substream *substream,
                                  struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	/*power off codec*/
	snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 1);
	snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 1);
	return;
}

int icdc_d3_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_codec *codec = dai->codec;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		snd_soc_update_bits(codec, SCODA_REG_CR_DAC, SCODA_CR_DAC_SMUTE_MASK, mute << SCODA_CR_DAC_SMUTE_SHIFT);
	} else if (stream == SNDRV_PCM_STREAM_CAPTURE) {
		snd_soc_update_bits(codec, SCODA_REG_CR_ADC, SCODA_CR_ADC_SMUTE_MASK, mute << SCODA_CR_ADC_SMUTE_SHIFT);
	}
	return 0;
}

static struct snd_soc_dai_ops icdc_d3_dai_ops = {
	.hw_params  = icdc_d3_hw_params,
	.mute_stream    = icdc_d3_mute_stream,
	.trigger    = icdc_d3_trigger,
	.shutdown   = ingenic_icdc_shutdown,
	.startup    = ingenic_icdc_startup,
};

static struct snd_soc_dai_driver  icdc_d3_codec_dai = {
	.name = "icdc-d3-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = DLV4780_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = DLV4780_FORMATS,
	},
	.ops = &icdc_d3_dai_ops,
};

/* unit: 0.01dB */
static const DECLARE_TLV_DB_SCALE(dac_tlv, -3100, 100, 0);
static const DECLARE_TLV_DB_SCALE(mix_tlv, -3100, 100, 0);
static const DECLARE_TLV_DB_SCALE(adc_tlv, 0, 100, 0);
static const DECLARE_TLV_DB_SCALE(mic_tlv, 0, 100, 0);

static const unsigned int icdc_d3_adc_mic_sel_value[] = {0x0, 0x1,};
static const unsigned int icdc_d3_mixer_input_sel_value[] = {0x0, 0x5, 0xa, 0xf,};
static const unsigned int icdc_d3_mixer_input_sel_value_double[] = {0x0, 0x1, 0x2, 0x3,};
static const unsigned int icdc_d3_mixer_mode_sel_value[] = {0x0, 0x1,};

static const char *icdc_d3_mixer_input_sel[] = { "Normal Inputs", "Cross Inputs", "Mixed Inputs", "Zero Inputs"};
static const char *icdc_d3_adc_mic_sel[] = { "AMIC ON", "DMIC ON"};
static const char *icdc_d3_mercury_vir_sel[] = { "MERCURY ON", "MERCURY OFF"};
static const char *icdc_d3_titanium_vir_sel[] = { "TITANIUM ON", "TITANIUM OFF"};
static const char *icdc_d3_dac_mixer_mode_sel[] = { "PLAYBACK DAC", "PLAYBACK DAC + ADC"};
static const char *icdc_d3_adc_mixer_mode_sel[] = { "RECORD INPUT", "RECORD INPUT + DAC"};

static const struct soc_enum icdc_d3_enum[] = {
	SOC_VALUE_ENUM_SINGLE(SCODA_REG_CR_ADC, 6, 0x1,  ARRAY_SIZE(icdc_d3_adc_mic_sel), icdc_d3_adc_mic_sel, icdc_d3_adc_mic_sel_value), /*0*/
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(icdc_d3_mercury_vir_sel), icdc_d3_mercury_vir_sel), /*1*/
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(icdc_d3_titanium_vir_sel), icdc_d3_titanium_vir_sel), /*2*/

	/*select input method*/
	SOC_VALUE_ENUM_DOUBLE(SCODA_MIX_0, 6, 4, 0x3,  ARRAY_SIZE(icdc_d3_mixer_input_sel), icdc_d3_mixer_input_sel, icdc_d3_mixer_input_sel_value_double), /*3*/
	SOC_VALUE_ENUM_DOUBLE(SCODA_MIX_1, 6, 4, 0x3,  ARRAY_SIZE(icdc_d3_mixer_input_sel), icdc_d3_mixer_input_sel, icdc_d3_mixer_input_sel_value_double), /*4*/
	SOC_VALUE_ENUM_DOUBLE(SCODA_MIX_2, 6, 4, 0x3,  ARRAY_SIZE(icdc_d3_mixer_input_sel), icdc_d3_mixer_input_sel, icdc_d3_mixer_input_sel_value_double), /*5*/
	SOC_VALUE_ENUM_DOUBLE(SCODA_MIX_3, 6, 4, 0x3,  ARRAY_SIZE(icdc_d3_mixer_input_sel), icdc_d3_mixer_input_sel, icdc_d3_mixer_input_sel_value_double), /*6*/
	SOC_VALUE_ENUM_SINGLE(SCODA_MIX_4, 4, 0xf,  ARRAY_SIZE(icdc_d3_mixer_input_sel), icdc_d3_mixer_input_sel, icdc_d3_mixer_input_sel_value), /*7*/

	/*select mix mode*/
	SOC_VALUE_ENUM_SINGLE(SCODA_MIX_0, 0, 1,  ARRAY_SIZE(icdc_d3_dac_mixer_mode_sel), icdc_d3_dac_mixer_mode_sel, icdc_d3_mixer_mode_sel_value), /*8*/
	SOC_VALUE_ENUM_SINGLE(SCODA_MIX_4, 0, 1,  ARRAY_SIZE(icdc_d3_dac_mixer_mode_sel), icdc_d3_dac_mixer_mode_sel, icdc_d3_mixer_mode_sel_value), /*9*/
	SOC_VALUE_ENUM_SINGLE(SCODA_MIX_2, 0, 1,  ARRAY_SIZE(icdc_d3_adc_mixer_mode_sel), icdc_d3_adc_mixer_mode_sel, icdc_d3_mixer_mode_sel_value), /*10*/

	SOC_VALUE_ENUM_SINGLE(SCODA_MIX_2, 6, 0x3,  ARRAY_SIZE(icdc_d3_mixer_input_sel), icdc_d3_mixer_input_sel, icdc_d3_mixer_input_sel_value), /*11*/
	SOC_VALUE_ENUM_SINGLE(SCODA_MIX_2, 4, 0x3,  ARRAY_SIZE(icdc_d3_mixer_input_sel), icdc_d3_mixer_input_sel, icdc_d3_mixer_input_sel_value), /*12*/
};

static const struct snd_kcontrol_new icdc_d3_adc_controls =
    SOC_DAPM_ENUM("Route",  icdc_d3_enum[0]);

static const struct snd_kcontrol_new icdc_d3_mercury_vmux_controls =
    SOC_DAPM_ENUM("Route", icdc_d3_enum[1]);

static const struct snd_kcontrol_new icdc_d3_titanium_vmux_controls =
    SOC_DAPM_ENUM("Route", icdc_d3_enum[2]);

static const struct snd_kcontrol_new icdc_d3_mercury_aidac_input_sel_controls =
    SOC_DAPM_ENUM("Route", icdc_d3_enum[3]);

static const struct snd_kcontrol_new icdc_d3_dac_input_sel_controls =
    SOC_DAPM_ENUM("Route", icdc_d3_enum[4]);

static const struct snd_kcontrol_new icdc_d3_aiadc_input_sel_controls =
    SOC_DAPM_ENUM("Route", icdc_d3_enum[5]);

static const struct snd_kcontrol_new icdc_d3_adc_input_sel_controls =
    SOC_DAPM_ENUM("Route", icdc_d3_enum[6]);

static const struct snd_kcontrol_new icdc_d3_titanium_aidac_input_sel_controls =
    SOC_DAPM_ENUM("Route", icdc_d3_enum[7]);

static const struct snd_kcontrol_new icdc_d3_mercury_mixer_mode_sel_controls =
    SOC_DAPM_ENUM("Route", icdc_d3_enum[8]);

static const struct snd_kcontrol_new icdc_d3_titanium_mixer_mode_sel_controls =
    SOC_DAPM_ENUM("Route", icdc_d3_enum[9]);

static const struct snd_kcontrol_new icdc_d3_aiadc_mixer_mode_sel_controls =
    SOC_DAPM_ENUM("Route", icdc_d3_enum[10]);

static const struct snd_kcontrol_new icdc_d3_aiadc_input_sel_controls_l =
    SOC_DAPM_ENUM("Route", icdc_d3_enum[11]);

static const struct snd_kcontrol_new icdc_d3_aiadc_input_sel_controls_r =
    SOC_DAPM_ENUM("Route", icdc_d3_enum[12]);

static const struct snd_kcontrol_new icdc_d3_snd_controls[] = {
	/* Volume controls */
	SOC_DOUBLE_R_TLV("MERCURY Playback Volume", SCODA_REG_GCR_DACL, SCODA_REG_GCR_DACR, 0, 63, 0, dac_tlv),
	SOC_DOUBLE_R_TLV("TITANIUM Playback Volume", SCODA_REG_GCR_DACL, SCODA_REG_GCR_DACR, 0, 63, 0, dac_tlv),
	SOC_DOUBLE_R_TLV("Playback Mixer Volume", SCODA_REG_GCR_MIXDACL, SCODA_REG_GCR_MIXDACR, 0, 31, 1, mix_tlv),
	SOC_DOUBLE_R_TLV("Digital Capture Volume", SCODA_REG_GCR_ADCL, SCODA_REG_GCR_ADCR, 0, 43, 0, adc_tlv),
	SOC_DOUBLE_R_TLV("Digital Capture Mixer Volume", SCODA_REG_GCR_MIXADCL, SCODA_REG_GCR_MIXADCR, 0, 31, 1, mix_tlv),
	SOC_SINGLE_TLV("Mic Volume", SCODA_REG_GCR_MIC1, 0, 4, 0, mic_tlv),

	/* ADC private controls */
	SOC_SINGLE("ADC High Pass Filter Switch", SCODA_REG_FCR_ADC, 6, 1, 0),

	/* mic private controls */
	SOC_SINGLE("Digital Playback mute", SCODA_REG_CR_DAC, 7, 1, 0),
	/* mixer enable controls */
	SOC_SINGLE("mixer Enable", SCODA_REG_CR_MIX, 7, 1, 0),
};

static const struct snd_soc_dapm_widget icdc_d3_dapm_widgets[] = {
	/* ADC */
	SND_SOC_DAPM_ADC("ADC", "Capture", SCODA_REG_AICR_ADC, 4, 1),
	SND_SOC_DAPM_MUX("ADC Mux", SCODA_REG_CR_ADC, 4, 1, &icdc_d3_adc_controls),
	SND_SOC_DAPM_MICBIAS("MICBIAS", SCODA_REG_CR_MIC1, 5, 1),
	SND_SOC_DAPM_PGA("AMIC", SCODA_REG_CR_MIC1, 4, 1, NULL, 0),
	SND_SOC_DAPM_PGA("DMIC", SCODA_REG_CR_DMIC, 7, 0, NULL, 0),

	/* DAC */
	SND_SOC_DAPM_DAC("DAC", "Playback", SCODA_REG_AICR_DAC, 4, 1),

	SND_SOC_DAPM_MUX("DAC_MERCURY VMux", SND_SOC_NOPM, 0, 0, &icdc_d3_mercury_vmux_controls),
	SND_SOC_DAPM_PGA("DAC_MERCURY", SCODA_REG_CR_DAC, 4, 1, NULL, 0),

	/*  SND_SOC_DAPM_MUX("DAC_TITANIUM VMux", SND_SOC_NOPM, 0, 0, &icdc_d3_titanium_vmux_controls),*/
	/*  SND_SOC_DAPM_PGA("DAC_TITANIUM", SCODA_REG_CR_DAC2, 4, 1, NULL, 0),*/

	/* MIXER */
	SND_SOC_DAPM_MUX("MERCURY AIDAC MIXER Mux", SND_SOC_NOPM, 0, 0, &icdc_d3_dac_input_sel_controls),
	SND_SOC_DAPM_MUX("DAC Mode Mux", SND_SOC_NOPM, 0, 0, &icdc_d3_mercury_mixer_mode_sel_controls),
	SND_SOC_DAPM_MUX("MERCURY AIDAC Mux", SND_SOC_NOPM, 0, 0, &icdc_d3_mercury_aidac_input_sel_controls),

	/* ADC */
	SND_SOC_DAPM_MUX("ADC Mode Mux", SND_SOC_NOPM, 0, 0, &icdc_d3_aiadc_mixer_mode_sel_controls),
	SND_SOC_DAPM_MUX("AIADC Mux", SND_SOC_NOPM, 0, 0, &icdc_d3_aiadc_input_sel_controls),
	SND_SOC_DAPM_MUX("AIADC Mux L", SND_SOC_NOPM, 0, 0, &icdc_d3_aiadc_input_sel_controls_l),
	SND_SOC_DAPM_MUX("AIADC Mux R", SND_SOC_NOPM, 0, 0, &icdc_d3_aiadc_input_sel_controls_r),
	SND_SOC_DAPM_MUX("ADC MIXER Mux", SND_SOC_NOPM, 0, 0, &icdc_d3_adc_input_sel_controls),

	/* PINS */
	SND_SOC_DAPM_INPUT("AIP"),
	SND_SOC_DAPM_INPUT("AIN"),
	SND_SOC_DAPM_INPUT("DMIC IN"),
	SND_SOC_DAPM_OUTPUT("DO_LO_PWM"),
	SND_SOC_DAPM_OUTPUT("DO_BO_PWM"),
};

static const struct snd_soc_dapm_route intercon[] = {

	{ "MICBIAS",  NULL,  "AIP" },
	{ "MICBIAS",  NULL,  "AIN" },
	{ "AMIC",  NULL,  "MICBIAS" },
	{ "AMIC",  NULL,  "MICBIAS" },

	/*input*/
	{ "ADC Mux", "AMIC ON", "AMIC" },
	{ "ADC Mux", "DMIC ON", "DMIC IN" },

	{ "ADC Mode Mux", "RECORD INPUT", "ADC Mux"},
	{ "ADC Mode Mux", "RECORD INPUT + DAC", "ADC Mux"},

	{ "AIADC Mux", "Normal Inputs", "ADC Mode Mux"},
	{ "AIADC Mux", "Cross Inputs", "ADC Mode Mux"},
	{ "AIADC Mux", "Mixed Inputs", "ADC Mode Mux"},
	{ "AIADC Mux", "Zero Inputs", "ADC Mode Mux"},
	{ "AIADC Mux L", "Normal Inputs", "ADC Mode Mux"},
	{ "AIADC Mux L", "Cross Inputs", "ADC Mode Mux"},
	{ "AIADC Mux L", "Mixed Inputs", "ADC Mode Mux"},
	{ "AIADC Mux L", "Zero Inputs", "ADC Mode Mux"},
	{ "AIADC Mux R", "Normal Inputs", "ADC Mode Mux"},
	{ "AIADC Mux R", "Cross Inputs", "ADC Mode Mux"},
	{ "AIADC Mux R", "Mixed Inputs", "ADC Mode Mux"},
	{ "AIADC Mux R", "Zero Inputs", "ADC Mode Mux"},

	{ "ADC", NULL, "AIADC Mux" },
	{ "ADC", NULL, "AIADC Mux L" },
	{ "ADC", NULL, "AIADC Mux R" },

	{ "ADC MIXER Mux", "Normal Inputs", "ADC Mux"},
	{ "ADC MIXER Mux", "Cross Inputs", "ADC Mux"},
	{ "ADC MIXER Mux", "Mixed Inputs", "ADC Mux"},
	{ "ADC MIXER Mux", "Zero Inputs", "ADC Mux"},

	{"DAC Mode Mux", NULL, "ADC MIXER Mux"},
	/*output*/
	{ "DAC_MERCURY", NULL, "DAC" },
	{ "DAC_MERCURY VMux", "MERCURY ON", "DAC_MERCURY" },

	/* select mixer inputs*/
	{"MERCURY AIDAC MIXER Mux", "Normal Inputs", "DAC_MERCURY VMux"},
	{"MERCURY AIDAC MIXER Mux", "Cross Inputs", "DAC_MERCURY VMux"},
	{"MERCURY AIDAC MIXER Mux", "Mixed Inputs", "DAC_MERCURY VMux"},
	{"MERCURY AIDAC MIXER Mux", "Zero Inputs", "DAC_MERCURY VMux"},

	/*select mixer mode*/
	{"DAC Mode Mux", "PLAYBACK DAC", "DAC_MERCURY VMux"},
	{"DAC Mode Mux", "PLAYBACK DAC + ADC", "MERCURY AIDAC MIXER Mux"},

	/*  DAC_MERCURY Vmux->DAC Mux*/
	/*select mixer output channels*/
	{ "MERCURY AIDAC Mux", "Normal Inputs", "DAC Mode Mux" },
	{ "MERCURY AIDAC Mux", "Cross Inputs", "DAC Mode Mux" },
	{ "MERCURY AIDAC Mux", "Mixed Inputs", "DAC Mode Mux" },
	{ "MERCURY AIDAC Mux", "Zero Inputs", "DAC Mode Mux" },

	{ "DO_LO_PWM", NULL, "MERCURY AIDAC Mux" },
};

#ifdef CONFIG_PM
static int icdc_d3_suspend(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

	snd_soc_update_bits(codec, SCODA_REG_AICR_ADC, SCODA_AICR_ADC_SB_MASK, 1);
	snd_soc_update_bits(codec, SCODA_REG_AICR_DAC, SCODA_AICR_DAC_SB_MASK, 1);
	snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 1);
	snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 1);
	snd_soc_update_bits(codec, SCODA_REG_CR_CK, SCODA_CR_CK_SDCLK_MASK, 1);
	snd_soc_dapm_force_bias_level(dapm, SND_SOC_BIAS_OFF);
	return 0;
}

static int icdc_d3_resume(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

	snd_soc_update_bits(codec, SCODA_REG_CR_CK, SCODA_CR_CK_SDCLK_MASK, 0);

	if (snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 0)) {
		msleep(250);
	}
	if (snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 0)) {
		msleep(10);
	}
	snd_soc_update_bits(codec, SCODA_REG_AICR_ADC, SCODA_AICR_ADC_SB_MASK, 0);
	snd_soc_update_bits(codec, SCODA_REG_AICR_DAC, SCODA_AICR_DAC_SB_MASK, 0);
	snd_soc_dapm_force_bias_level(dapm, SND_SOC_BIAS_STANDBY);
	return 0;
}
#endif

static int icdc_d3_probe(struct snd_soc_codec *codec)
{
	struct icdc_d3 *icdc_d3 = snd_soc_codec_get_drvdata(codec);
	dev_info(codec->dev, "codec icdc-d3 probe enter\n");

	/* power off codec */
	snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 1);
	snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 1);

#ifdef DEBUG
	/*dump for debug*/
	dump_registers_hazard(icdc_d3);
#endif
	/* codec select enable 24M clock*/
	snd_soc_update_bits(codec, SCODA_REG_CR_CK, SCODA_CR_CK_MCLK_DIV_MASK, 1 << SCODA_CR_CK_MCLK_DIV_SHIFT);
	snd_soc_update_bits(codec, SCODA_REG_CR_CK, SCODA_CR_CK_SDCLK_MASK, 0 << SCODA_CR_CK_SDCLK_SHIFT);
	snd_soc_update_bits(codec, SCODA_REG_CR_CK, SCODA_CR_CRYSTAL_MASK, 0 << SCODA_CR_CRYSTAL_SHIFT);

	/*codec select Dac/Adc i2s interface*/
	snd_soc_update_bits(codec, SCODA_REG_AICR_DAC, SCODA_AICR_DAC_SLAVE_MASK, 0);
	snd_soc_update_bits(codec, SCODA_REG_AICR_DAC, SCODA_AICR_DAC_AUDIO_MASK, SCODA_AICR_DAC_AUDIOIF_I2S);

	/*codec generated IRQ is a high level */
	snd_soc_update_bits(codec, SCODA_REG_ICR, SCODA_ICR_INT_FORM_MASK, SCODA_ICR_INT_FORM_LOW);

	/*codec irq mask*/
	snd_soc_write(codec, SCODA_REG_IMR, SCODA_IMR_COMMON_MASK);
	snd_soc_write(codec, SCODA_REG_IMR2, SCODA_IMR2_COMMON_MASK);

	/*codec clear all irq*/
	snd_soc_write(codec, SCODA_REG_IFR, SCODA_IMR_COMMON_MASK);
	snd_soc_write(codec, SCODA_REG_IFR2, SCODA_IMR2_COMMON_MASK);

	icdc_d3_write(codec, SCODA_MIX_3, 0x5 << 4);
	icdc_d3_write(codec, SCODA_MIX_2, 0x1 << 4);

	icdc_d3->codec = codec;
	return 0;
}

static int icdc_d3_remove(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	dev_info(codec->dev, "codec icdc_d3 remove enter\n");
	snd_soc_dapm_force_bias_level(dapm, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_icdc_d3_codec = {
	.probe =    icdc_d3_probe,
	.remove =   icdc_d3_remove,
#ifdef CONFIG_PM
	.suspend =  icdc_d3_suspend,
	.resume =   icdc_d3_resume,
#endif

	.read =     icdc_d3_read,
	.write =    icdc_d3_write,
	.reg_cache_default = icdc_d3_reg_defcache,
	.reg_word_size = sizeof(u8),
	.reg_cache_step = 1,
	.reg_cache_size = SCODA_MAX_REG_NUM,

	.controls =     icdc_d3_snd_controls,
	.num_controls = ARRAY_SIZE(icdc_d3_snd_controls),
	.dapm_widgets = icdc_d3_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(icdc_d3_dapm_widgets),
	.dapm_routes = intercon,
	.num_dapm_routes = ARRAY_SIZE(intercon),
};

#ifdef CONFIG_AUDIO_DUMP
/*Just for debug*/
static ssize_t hw_regs_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	struct icdc_d3 *icdc_d3 = dev_get_drvdata(dev);
	struct snd_soc_codec *codec = icdc_d3->codec;
	if (!codec) {
		dev_info(dev, "icdc_d3 is not probe, can not use %s function\n", __func__);
		return 0;
	}
	mutex_lock(&codec->component.io_mutex);
	dump_registers_hazard(icdc_d3);
	mutex_unlock(&codec->component.io_mutex);
	return 0;
}

static ssize_t hw_regs_store(struct device *dev,
                             struct device_attribute *attr, const char *buf,
                             size_t count)
{
	struct icdc_d3 *icdc_d3 = dev_get_drvdata(dev);
	struct snd_soc_codec *codec = icdc_d3->codec;
	const char *start = buf;
	unsigned int reg, val;
	int ret_count = 0;

	if (!codec) {
		dev_info(dev, "icdc_d3 is not probe, can not use %s function\n", __func__);
		return count;
	}

	while (!isxdigit(*start)) {
		start++;
		if (++ret_count >= count) {
			return count;
		}
	}
	reg = simple_strtoul(start, (char **)&start, 16);
	while (!isxdigit(*start)) {
		start++;
		if (++ret_count >= count) {
			return count;
		}
	}
	val = simple_strtoul(start, (char **)&start, 16);
	mutex_lock(&codec->component.io_mutex);
	icdc_d3_write(codec, reg, val);
	mutex_unlock(&codec->component.io_mutex);
	return count;
}

static struct device_attribute icdc_d3_sysfs_attrs =
    __ATTR_RW(hw_regs);
#endif

static int icdc_d3_platform_probe(struct platform_device *pdev)
{
	struct device_node *parent = of_get_parent(pdev->dev.of_node);
	struct icdc_d3 *icdc_d3 = NULL;
	int ret = 0;

	icdc_d3 = (struct icdc_d3 *)devm_kzalloc(&pdev->dev,
	          sizeof(struct icdc_d3), GFP_KERNEL);
	if (!icdc_d3) {
		return -ENOMEM;
	}

	icdc_d3->mapped_base = of_iomap(parent, 0);
	if (IS_ERR(icdc_d3->mapped_base)) {
		dev_err(&pdev->dev, "Failed to ioremap mmio memory\n");
		return PTR_ERR(icdc_d3->mapped_base);
	}

	icdc_d3->dev = &pdev->dev;
	icdc_d3->dac_user_mute = 1;
	icdc_d3->aohp_in_pwsq = 0;
	spin_lock_init(&icdc_d3->io_lock);
	platform_set_drvdata(pdev, (void *)icdc_d3);

	ret = snd_soc_register_codec(&pdev->dev,
	                             &soc_codec_dev_icdc_d3_codec, &icdc_d3_codec_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Faild to register codec\n");
		platform_set_drvdata(pdev, NULL);
		return ret;
	}

#ifdef CONFIG_AUDIO_DUMP
	ret = device_create_file(&pdev->dev, &icdc_d3_sysfs_attrs);
#endif
	dev_info(&pdev->dev, "codec icdc-d3 platfrom probe success\n");
	return 0;
}

static int icdc_d3_platform_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "codec icdc-d3 platform remove\n");
	snd_soc_unregister_codec(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id codec_dt_match[] = {
	{ .compatible = "ingenic,icdc3", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, codec_dt_match);

static struct platform_driver icdc_d3_codec_driver = {
	.driver = {
		.name = "icdc-d3",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(codec_dt_match),
	},
	.probe = icdc_d3_platform_probe,
	.remove = icdc_d3_platform_remove,
};

static int icdc_d3_modinit(void)
{
	return platform_driver_register(&icdc_d3_codec_driver);
}
module_init(icdc_d3_modinit);

static void icdc_d3_exit(void)
{
	platform_driver_unregister(&icdc_d3_codec_driver);
}
module_exit(icdc_d3_exit);

MODULE_DESCRIPTION("iCdc d3 Codec Driver");
MODULE_AUTHOR("sccheng<shicheng.cheng@ingenic.com>");
MODULE_LICENSE("GPL");
