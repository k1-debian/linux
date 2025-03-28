/*
 * ALSA Soc Audio Layer -- ingenic as(audio system) dmic(Basic Audio Inter-
 * face Controller) driver
 *
 * Copyright 2017 - 2022 Ingenic Semiconductor Co.,Ltd
 *   wqshao <wangquan.shao@ingenic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef __AS_DMIC_H__
#define __AS_DMIC_H__

#include <linux/bitops.h>

struct ingenic_dmic {
	struct device   *dev;
	struct regmap *regmap;
	void __iomem    *io_base;
	struct snd_soc_dai_driver *dai_driver;
	struct snd_soc_dai_ops *dai_ops;
	struct clk *clk;
	struct clk *clk_gate;
};

#define DMIC_CR0    (0x0)
#define DMIC_GCR    (0x4)
#define DMIC_IER    (0x8)
#define DMIC_ICR    (0xc)
#define DMIC_TCR    (0x10)

/* DMIC_CR0 */
#define DMIC_RESET      BIT(31)
#define HPF2_EN_SFT     (22)
#define HPF2_EN         BIT(22)
#define LPF_EN_SFT      (21)
#define LPF_EN          BIT(21)
#define HPF1_EN_SFT     (20)
#define HPF1_EN         BIT(20)
#define CHNUM_SFT       (16)
#define CHNUM_MASK      GENMASK(19, CHNUM_SFT)
#define DMIC_SET_CHL(chl)   (((chl) - 1) << CHNUM_SFT)
#define OSS_SFT         (12)
#define OSS_MASK        GENMASK(13, OSS_SFT)
#define OSS_16BIT       (0x0 << OSS_SFT)
#define OSS_24BIT       (0x1 << OSS_SFT)
#define DMIC_SET_OSS(x) ((x == 16) ? OSS_16BIT : (x == 24) ? OSS_24BIT : OSS_16BIT)
#define SW_LR_SFT       (11)
#define SW_LR           BIT(11)
#define SR_SFT          (6)
#define SR_MASK         GENMASK(8, SR_SFT)
#define SR_8k           (0x0 << SR_SFT)
#define SR_16k          (0x1 << SR_SFT)
#define SR_48k          (0x2 << SR_SFT)
#define SR_96k          (0x3 << SR_SFT)
#define DMIC_SET_SR(r)  ((r) == 8000 ? SR_8k : (r == 16000) ? SR_16k : (r == 48000) ? SR_48k : (r == 96000) ? SR_96k : SR_48k)
#define DMIC_EN         BIT(0)

/* DMIC_GCR */
#define DGAIN_SFT       (0)
#define DMIC_SET_GAIN(x)    ((x) << DGAIN_SFT)

#endif //__AS_DMIC_H__
