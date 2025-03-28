/*
* Copyright (C) 2017 Ingenic Semiconductor Co., Ltd.
*  http://www.ingenic.com
* Author: cli <chen.li@ingenic.com>
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
*
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <linux/clk.h>
#include "../as-v2/as-baic.h"

static const struct snd_soc_dapm_route audio_map[] = {
	/*  Target    control   Source*/
	{ "BAIC0 playback", NULL, "LO0_MUX" },
	{ "BAIC1 playback", NULL, "LO1_MUX" },
	{ "SPDIF playback", NULL, "LO2_MUX" },
	{ "BAIC3 playback", NULL, "LO3_MUX" },
	{ "BAIC4 playback", NULL, "LO4_MUX" },

	{ "LI0", NULL, "DMIC capture" },
	{ "LI1", NULL, "SPDIF capture" },
	{ "LI2", NULL, "BAIC0 capture" },
	{ "LI3", NULL, "BAIC1 capture" },
	{ "LI4", NULL, "BAIC2 capture" },
	{ "LI6", NULL, "BAIC4 capture" },
#if 1
	{"LO7_MUX", NULL, "LI3"},   /*I2S1 Capture: BAIC1 capture->LI3->LO7_MUX->DMA7*/
	{"LO1_MUX", NULL, "LI9"},   /*I2S1 Playback: DMA1->LI9->LO1_MUX->BAIC1 playback*/
	{"LO8_MUX", NULL, "LI4"},   /*I2S2 Capture: BAIC2 capture->LI4->LO8_MUX->DMA8*/
	{"LO3_MUX", NULL, "LI10"},  /*I2S3 Playback: DMA2->LI10->LO3_MUX->BAIC3 playback*/
#endif

	{ "LI8", NULL, "DMA0" },
	{ "LI9", NULL, "DMA1" },
	{ "LI10", NULL, "DMA2" },
	{ "LI11", NULL, "DMA3" },
	{ "LI12", NULL, "DMA4" },

	{ "DMA5", NULL, "LO5_MUX"},
	{ "DMA6", NULL, "LO6_MUX"},
	{ "DMA7", NULL, "LO7_MUX"},
	{ "DMA8", NULL, "LO8_MUX"},
	{ "DMA9", NULL, "LO9_MUX"},

	{ "MIX0", NULL, "LO10_MUX"},
	{ "MIX0", NULL, "LO11_MUX"},
	{ "LI7",  NULL, "MIX0"},

	{"SPDIF_OUT", NULL, "SPDIF playback"},
	{"SPDIF capture", NULL, "SPDIF_IN"},
	{"DMIC capture", NULL, "DMIC"},
};

static const struct snd_soc_dapm_widget dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("SPDIF_OUT"),
	SND_SOC_DAPM_INPUT("SPDIF_IN"),
	SND_SOC_DAPM_INPUT("DMIC"),
};

struct codec_format_enum {
	struct soc_enum e;
	int baic_id;
	unsigned int *format;
};

unsigned int codec_format[MAX_BAIC_NUM] = {
	/*baic0 icodec format*/
	SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
	/*baic1 format*/
	SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
	/*baic2 format*/
	SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS | BAIC_DAIFMT_SEL_TDM1,
	/*baic3 format*/
	SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
	/*baic4 pcm format*/
	SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS | BAIC_DAIFMT_SEL_PCM
};
static const char *const baic_format_item_name[BAIC_SUPPORT_FORMAT_MAX] = {
	"PCMA", "PCMB", "DSPA", "DSPB", "TDM1A", "TDM1B", "TDM2A", "TDM2B", "I2S", "LEFT", "RIGHT"
};
static const unsigned int baic_format_item_value[BAIC_SUPPORT_FORMAT_MAX] = {
	SND_SOC_DAIFMT_DSP_A | BAIC_DAIFMT_SEL_PCM,
	SND_SOC_DAIFMT_DSP_B | BAIC_DAIFMT_SEL_PCM,
	SND_SOC_DAIFMT_DSP_A | BAIC_DAIFMT_SEL_DSP,
	SND_SOC_DAIFMT_DSP_B | BAIC_DAIFMT_SEL_DSP,
	SND_SOC_DAIFMT_DSP_A | BAIC_DAIFMT_SEL_TDM1,
	SND_SOC_DAIFMT_DSP_B | BAIC_DAIFMT_SEL_TDM1,
	SND_SOC_DAIFMT_DSP_A | BAIC_DAIFMT_SEL_TDM2,
	SND_SOC_DAIFMT_DSP_B | BAIC_DAIFMT_SEL_TDM2,
	SND_SOC_DAIFMT_I2S,
	SND_SOC_DAIFMT_LEFT_J,
	SND_SOC_DAIFMT_RIGHT_J,
};
static int baic_format_get_enum(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol)
{
	struct codec_format_enum *e = (struct codec_format_enum *)kcontrol->private_value;
	unsigned int format = (*e->format) & (SND_SOC_DAIFMT_FORMAT_MASK | BAIC_DAIFMT_SEL_MASK);
	int i;
	for (i = 0; i < BAIC_SUPPORT_FORMAT_MAX; i++) {
		if (format == baic_format_item_value[i]) {
			ucontrol->value.enumerated.item[0] = i;
			return 0;
		}
	}
	return -EINVAL;
}

static int baic_format_put_enum(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol)
{
	struct codec_format_enum *e = (struct codec_format_enum *)kcontrol->private_value;
	unsigned int item = ucontrol->value.enumerated.item[0];

	(*e->format) &= ~(SND_SOC_DAIFMT_FORMAT_MASK | BAIC_DAIFMT_SEL_MASK);
	(*e->format) |= baic_format_item_value[item];

	return 0;
}

static struct codec_format_enum codec_format_enum[] = { /*not const*/
#define BAIC_FORMAT_ENUM(_baic_id)  \
	{ \
		.e = SOC_ENUM_SINGLE_EXT(BAIC_SUPPORT_FORMAT_MAX, baic_format_item_name), \
		.baic_id = _baic_id,    \
		.format = &codec_format[(_baic_id)],    \
	}
	BAIC_FORMAT_ENUM(0),
	BAIC_FORMAT_ENUM(1),
	BAIC_FORMAT_ENUM(2),
	BAIC_FORMAT_ENUM(3),
	BAIC_FORMAT_ENUM(4),
#undef BAIC_FORMAT_ENUM
};

static const struct snd_kcontrol_new card_snd_controls[] = {
#define FORMAT0 "baic0_fmt"
#define FORMAT1 "baic1_fmt"
#define FORMAT2 "baic2_fmt"
#define FORMAT3 "baic3_fmt"
#define FORMAT4 "baic4_fmt"
#define CODEC_FORMAT_ENUM_EXT(N) \
	SOC_ENUM_EXT(FORMAT##N,     \
	             codec_format_enum[(N)], \
	             baic_format_get_enum,   \
	             baic_format_put_enum)
	CODEC_FORMAT_ENUM_EXT(0),
	CODEC_FORMAT_ENUM_EXT(1),
	CODEC_FORMAT_ENUM_EXT(2),
	CODEC_FORMAT_ENUM_EXT(3),
	CODEC_FORMAT_ENUM_EXT(4),
#undef FORMAT0
#undef FORMAT1
#undef FORMAT2
#undef FORMAT3
#undef FORMAT4
#undef CODEC_FORMAT_ENUM_EXT
};

typedef enum slot_width {
	slot_width_auto = 0,
	slot_width_16 = 16,
	slot_width_32 = 32,
	slot_width_64 = 64,
} slot_width_t;

slot_width_t codec_slot_width[MAX_BAIC_NUM] = {
	slot_width_auto,
	slot_width_auto,
	slot_width_32,
	slot_width_32,
	slot_width_auto
};

static int  get_baic_id(struct snd_soc_dai *dai)
{
	int id = -1;
	const char *name = dai->driver->name;

	sscanf(&name[strlen("BAIC")], "%d", &id);

	return id;
}

static int baic_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	int bclk_div = 0, sysclk = 24000000, bclk = 0, codec_slots = 0,
	    sync_div = 0, sync_w_div = 1, tx_msk = 0, rx_msk = 0;
	bool is_playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? true : false;
	int divid_dir = is_playback ? DIVID_DIR_T : 0;
	int clkid = is_playback ? CLKID_SYSCLK_T : CLKID_SYSCLK_R;
	int channels = params_channels(params);
	int fmt_width = snd_pcm_format_width(params_format(params));
	int id = get_baic_id(cpu_dai);
	int slot_width = codec_slot_width[id];
	unsigned int format = codec_format[id];
	int ret;

	format |= is_playback ? BAIC_DAIFMT_T : BAIC_DAIFMT_R;
	ret = snd_soc_runtime_set_dai_fmt(rtd, format);
	if (ret) {
		return ret;
	}

	switch (format & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_LEFT_J:
		case SND_SOC_DAIFMT_I2S:
		case SND_SOC_DAIFMT_RIGHT_J:
			if (fmt_width > 16) {
				sync_div = 64;
			} else if (fmt_width > 8) {
				sync_div = 32;
			} else {
				sync_div = 16;
			}
			break;
		case SND_SOC_DAIFMT_DSP_B:
		case SND_SOC_DAIFMT_DSP_A:
			if (!slot_width) {
				slot_width = (fmt_width > 16) ? 32 : 16;
			}
			if (is_playback) {
				tx_msk = (1 << channels) - 1;
			} else {
				rx_msk = (1 << channels) - 1;
			}
			ret = snd_soc_dai_set_clkdiv(cpu_dai, DIVID_SYNC_W | divid_dir, sync_w_div);
			if (ret) {
				return ret;
			}
			ret = snd_soc_dai_set_tdm_slot(cpu_dai, tx_msk, rx_msk, channels, slot_width);
			if (ret) {
				return ret;
			}

			if ((channels == 8) && ((format & BAIC_DAIFMT_SEL_TDM2) == BAIC_DAIFMT_SEL_TDM2)) {
				sync_div = slot_width * 4;
				/*for ak4458/ak5558*/
				codec_slots = 4;
			} else {
				sync_div = slot_width * channels;
				codec_slots = channels;
			}
			ret = snd_soc_dai_set_tdm_slot(codec_dai, tx_msk, rx_msk, codec_slots, slot_width);
			if (ret != 0 && ret != -ENOTSUPP) {
				dev_warn(codec_dai->dev,
				         "ASoC: Failed to set tdm slot: %d\n", ret);
				return ret;
			}
			break;
		default:
			printk("%s codec config err!\n", __func__);
			return -EPERM;

	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, DIVID_SYNC | divid_dir, sync_div);
	if (ret) {
		return ret;
	}

	bclk = params_rate(params) * sync_div;
	if (id == 0 || id == 1 || id == 2 || id == 3) {
		/*for baic0: sysclk = 256 * sample_rate */
		sysclk = 256 * params_rate(params);
	}

	bclk_div = (sysclk + bclk - 1) / bclk;
	/* bclk div value must be even */
	if (bclk_div % 2) {
		sysclk = (bclk_div + 1) * bclk;
		bclk_div += 1;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, clkid, sysclk, SND_SOC_CLOCK_OUT);
	if (ret) {
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, DIVID_BCLK | divid_dir, bclk_div);
	if (ret) {
		return ret;
	}

	return 0;
};

static struct snd_soc_ops baic_ops = {
	.hw_params = baic_hw_params,
};

static int spdif_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	u32 sysclk = 0, spdif_out_clk_div = 0, spdif_in_clk_div = 0;
	int clk_ratio = substream->stream ? 1280 : 32 * 2;  //fixed 2channels
	int ret;

	// sysclk = 120000000;
	sysclk = params_rate(params) * 640;

	spdif_out_clk_div = (sysclk + ((params_rate(params) * clk_ratio) - 1))
	                    / (params_rate(params) * clk_ratio);
	spdif_out_clk_div &= ~0x1;

	spdif_in_clk_div = (sysclk + ((params_rate(params) * clk_ratio) - 1))
	                   / (params_rate(params) * clk_ratio);
	spdif_in_clk_div &= ~0x1;

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, sysclk, 0);
	if (ret) {
		return ret;
	}
	ret = snd_soc_dai_set_clkdiv(cpu_dai, substream->stream,
	                             !(substream->stream) ? spdif_out_clk_div : spdif_in_clk_div);
	if (ret) {
		return ret;
	}

	return 0;
};

static struct snd_soc_ops spdif_ops = {
	.hw_params  = spdif_hw_params,
};

SND_SOC_DAILINK_DEFS(dma0_playback,
                     DAILINK_COMP_ARRAY(COMP_CPU("DMA0")),
                     DAILINK_COMP_ARRAY(COMP_DUMMY()),
                     DAILINK_COMP_ARRAY(COMP_PLATFORM("134d0000.as-platform")));

SND_SOC_DAILINK_DEFS(dma1_playback,
                     DAILINK_COMP_ARRAY(COMP_CPU("DMA1")),
                     DAILINK_COMP_ARRAY(COMP_DUMMY()),
                     DAILINK_COMP_ARRAY(COMP_PLATFORM("134d0000.as-platform")));

SND_SOC_DAILINK_DEFS(dma2_playback,
                     DAILINK_COMP_ARRAY(COMP_CPU("DMA2")),
                     DAILINK_COMP_ARRAY(COMP_DUMMY()),
                     DAILINK_COMP_ARRAY(COMP_PLATFORM("134d0000.as-platform")));

SND_SOC_DAILINK_DEFS(dma3_playback,
                     DAILINK_COMP_ARRAY(COMP_CPU("DMA3")),
                     DAILINK_COMP_ARRAY(COMP_DUMMY()),
                     DAILINK_COMP_ARRAY(COMP_PLATFORM("134d0000.as-platform")));

SND_SOC_DAILINK_DEFS(dma4_playback,
                     DAILINK_COMP_ARRAY(COMP_CPU("DMA4")),
                     DAILINK_COMP_ARRAY(COMP_DUMMY()),
                     DAILINK_COMP_ARRAY(COMP_PLATFORM("134d0000.as-platform")));

SND_SOC_DAILINK_DEFS(dma5_capture,
                     DAILINK_COMP_ARRAY(COMP_CPU("DMA5")),
                     DAILINK_COMP_ARRAY(COMP_DUMMY()),
                     DAILINK_COMP_ARRAY(COMP_PLATFORM("134d0000.as-platform")));

SND_SOC_DAILINK_DEFS(dma6_capture,
                     DAILINK_COMP_ARRAY(COMP_CPU("DMA6")),
                     DAILINK_COMP_ARRAY(COMP_DUMMY()),
                     DAILINK_COMP_ARRAY(COMP_PLATFORM("134d0000.as-platform")));

SND_SOC_DAILINK_DEFS(dma7_capture,
                     DAILINK_COMP_ARRAY(COMP_CPU("DMA7")),
                     DAILINK_COMP_ARRAY(COMP_DUMMY()),
                     DAILINK_COMP_ARRAY(COMP_PLATFORM("134d0000.as-platform")));

SND_SOC_DAILINK_DEFS(dma8_capture,
                     DAILINK_COMP_ARRAY(COMP_CPU("DMA8")),
                     DAILINK_COMP_ARRAY(COMP_DUMMY()),
                     DAILINK_COMP_ARRAY(COMP_PLATFORM("134d0000.as-platform")));

SND_SOC_DAILINK_DEFS(dma9_capture,
                     DAILINK_COMP_ARRAY(COMP_CPU("DMA9")),
                     DAILINK_COMP_ARRAY(COMP_DUMMY()),
                     DAILINK_COMP_ARRAY(COMP_PLATFORM("134d0000.as-platform")));

SND_SOC_DAILINK_DEFS(inno_icodec,
                     DAILINK_COMP_ARRAY(COMP_CPU("BAIC0")),
                     DAILINK_COMP_ARRAY(COMP_CODEC("10020000.icodec", "icodec")));

SND_SOC_DAILINK_DEFS(dmic,
                     DAILINK_COMP_ARRAY(COMP_CPU("DMIC")),
                     DAILINK_COMP_ARRAY(COMP_DUMMY()));

SND_SOC_DAILINK_DEFS(baic1_bt,
                     DAILINK_COMP_ARRAY(COMP_CPU("BAIC1")),
                     DAILINK_COMP_ARRAY(COMP_CODEC("wm8594.2-001a", "wm8594-hifi")));

SND_SOC_DAILINK_DEFS(baic3_bt,
                     DAILINK_COMP_ARRAY(COMP_CPU("BAIC3")),
                     DAILINK_COMP_ARRAY(COMP_CODEC("ak4458.2-0010", "ak4458.2-0010")));

SND_SOC_DAILINK_DEFS(baic4_bt,
                     DAILINK_COMP_ARRAY(COMP_CPU("BAIC4")),
                     DAILINK_COMP_ARRAY(COMP_DUMMY()));

SND_SOC_DAILINK_DEFS(spdif,
                     DAILINK_COMP_ARRAY(COMP_CPU("SPDIF")),
                     DAILINK_COMP_ARRAY(COMP_DUMMY()));

static struct snd_soc_dai_link x2000_dais[] = {
	/*FE DAIS*/
	[0] = {
		.name = "DMA0 playback",
		.stream_name = "DMA0 playback",
		.trigger =  {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(dma0_playback),
	},
	[1] = {
		.name = "DMA1 playback",
		.stream_name = "DMA1 playback",
		.trigger =  {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(dma1_playback),
	},
	[2] = {
		.name = "DMA2 playback",
		.stream_name = "DMA2 playback",
		.trigger =  {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(dma2_playback),
	},
	[3] = {
		.name = "DMA3 playback",
		.stream_name = "DMA3 playback",
		.trigger =  {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(dma3_playback),
	},

	[4] = {
		.name = "DMA4 playback",
		.stream_name = "DMA4 playback",
		.trigger =  {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(dma4_playback),
	},

	[5] = {
		.name = "DMA5 capture",
		.stream_name = "DMA5 capture",
		.trigger =  {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(dma5_capture),
	},

	[6] = {
		.name = "DMA6 capture",
		.stream_name = "DMA6 capture",
		.trigger =  {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(dma6_capture),
	},

	[7] = {
		.name = "DMA7 capture",
		.stream_name = "DMA7 capture",
		.trigger =  {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(dma7_capture),
	},

	[8] = {
		.name = "DMA8 capture",
		.stream_name = "DMA8 capture",
		.trigger =  {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(dma8_capture),
	},

	[9] = {
		.name = "DMA9 capture",
		.stream_name = "DMA9 capture",
		.trigger =  {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(dma9_capture),
	},

	/*BE DAIS*/
	[10] = {
		.name = "BAIC0",
		.stream_name = "BAIC0",
		.ops = &baic_ops,
		.be_hw_params_fixup = NULL,
		.no_pcm = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(inno_icodec),
	},
	[11] = {
		.name = "BAIC1",
		.stream_name = "BAIC1",
		.ops = &baic_ops,
		.be_hw_params_fixup = NULL,
		.no_pcm = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(baic1_bt),
	},
	[12] = {
		.name = "BAIC3",
		.stream_name = "BAIC3",
		.ops = &baic_ops,
		.be_hw_params_fixup = NULL,
		.dpcm_playback = 1,
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(baic3_bt),
	},
	[13] = {
		.name = "BAIC4",
		.stream_name = "BAIC4",
		.ops = &baic_ops,
		.be_hw_params_fixup = NULL,
		.no_pcm = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(baic4_bt),
	},
	[14] = {
		.name = "DMIC",
		.stream_name = "DMIC",
		.no_pcm = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(dmic),
	},
	[15] = {
		.name = "SPDIF",
		.stream_name = "SPDIF",
		.ops = &spdif_ops,
		.no_pcm = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(spdif),
	},
};

static struct snd_soc_aux_dev x2000_aux_dev = {
	.dlc = COMP_AUX("134dc000.as-mixer"),
};

static int audio_clk_init(struct device *dev)
{
	struct clk *clk, *parent;
	char clk_name[16];
	unsigned long rate;
	int i;

	parent = clk_get(dev, "epll");
	if (IS_ERR_OR_NULL(parent)) {
		dev_err(dev, "Get epll clk failed!\n");
		return -EINVAL;
	}
	for (i = 0; i < 4; i++) {
		sprintf(clk_name, "div_i2s%d", i);
		clk = clk_get(dev, clk_name);
		if (IS_ERR_OR_NULL(clk)) {
			dev_err(dev, "Get %s clk failed!\n", clk_name);
			return -EINVAL;
		}
		rate = clk_get_rate(clk);
		clk_set_parent(clk, parent);
		clk_set_rate(clk, rate);
		clk_put(clk);
	}
	clk_put(parent);

	return 0;
}

static int snd_x2000_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	int ret;

	card = (struct snd_soc_card *)devm_kzalloc(&pdev->dev,
	        sizeof(struct snd_soc_card), GFP_KERNEL);
	if (!card) {
		return -ENOMEM;
	}

	card->dapm_widgets = dapm_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(dapm_widgets);
	card->dapm_routes = audio_map;
	card->num_dapm_routes = ARRAY_SIZE(audio_map);
	card->controls = card_snd_controls;
	card->num_controls = ARRAY_SIZE(card_snd_controls);
	card->dai_link = x2000_dais;
	card->num_links = ARRAY_SIZE(x2000_dais);
	card->owner = THIS_MODULE;
	card->dev = &pdev->dev;
	card->aux_dev = &x2000_aux_dev;
	card->num_aux_devs = 1;

	ret = snd_soc_of_parse_card_name(card, "ingenic,model");
	if (ret) {
		return ret;
	}

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed %d\n", ret);
		return ret;
	}

	ret = audio_clk_init(card->dev);
	if (ret) {
		return ret;
	}

	platform_set_drvdata(pdev, card);
	dev_info(&pdev->dev, "Sound Card successed\n");

	return ret;
}

static int snd_x2000_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	snd_soc_unregister_card(card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id x2000_match_table[] = {
	{ .compatible = "ingenic,x2000-sound", },
	{ }
};
MODULE_DEVICE_TABLE(of, x2000_match_table);

static struct platform_driver snd_x2000_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "x2000-sound",
		.of_match_table = x2000_match_table
	},
	.probe = snd_x2000_probe,
	.remove = snd_x2000_remove,
};
module_platform_driver(snd_x2000_driver);

MODULE_AUTHOR("cli<chen.li@ingenic.com>");
MODULE_DESCRIPTION("ALSA SoC X2000 Snd Card");
MODULE_LICENSE("GPL");
