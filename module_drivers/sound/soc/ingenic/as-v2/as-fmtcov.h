/*
 * ALSA Soc Audio Layer -- Ingenic AS fmtcov driver
 *
 * Copyright 2017 - 2022 Ingenic Semiconductor Co.,Ltd
 *  wqshao <wangquan.shao@ingenic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef __AS_fmtcov__
#define __AS_fmtcov__

#include <linux/bitops.h>

int ingenic_spdif_fmtcov_be_fix(struct snd_pcm_substream *substream,
                                struct snd_pcm_hw_params *params);

int ingenic_as_fmtcov_cfg(struct snd_pcm_substream *substream,
                          struct snd_pcm_hw_params *params);
void ingenic_as_fmtcov_enable(u8 dai_id, bool enable);

struct ingenic_as_fmtcov {
	struct regmap *regmap;
	struct device *dev;
	void *__iomem io_base;
	u8 dai_id;
#define FMT_CHL_NUM 10
	u32 fmtcfg[FMT_CHL_NUM];
	uint32_t fmtcov_bitmask;
};

/* FORMAT */
#define DFCR0   (0x0)

#define FMTCOV_OFFSET   (0x4)
#define DFCR(n) ((n) * FMTCOV_OFFSET + DFCR0)

#define DFCR_CHNUM_SFT  (12)
#define DFCR_CHNUM_MSK  GENMASK(15, DFCR_CHNUM_SFT)
#define DFCR_CHNUM(ch)  ((((!((ch) % 2) || ((ch) == 1)) ? ((ch) - 1) : (ch)) << DFCR_CHNUM_SFT) & DFCR_CHNUM_MSK)
#define DFCR_SS_SFT (8)
#define DFCR_SS_MSK GENMASK(10, DFCR_SS_SFT)
#define DFCR_SS_8   (0 << DFCR_SS_SFT)
#define DFCR_SS_12  (1 << DFCR_SS_SFT)
#define DFCR_SS_13  (2 << DFCR_SS_SFT)
#define DFCR_SS_16  (3 << DFCR_SS_SFT)
#define DFCR_SS_18  (4 << DFCR_SS_SFT)
#define DFCR_SS_20  (5 << DFCR_SS_SFT)
#define DFCR_SS_24  (6 << DFCR_SS_SFT)
#define DFCR_SS_32  (7 << DFCR_SS_SFT)
#define DFCR_SS(ss) ((ss <= 8) ? DFCR_SS_8 : (ss <= 12) ? DFCR_SS_12 : (ss == 13) ? DFCR_SS_13 : (ss <= 16) ? DFCR_SS_16 : (ss <= 18) ? DFCR_SS_18 : (ss <= 20) ? DFCR_SS_20 : (ss <= 24) ? DFCR_SS_24: DFCR_SS_32)
#define DFCR_PACKEN BIT(2)
#define DFCR_ENABLE BIT(0)

#endif /*__AS_fmtcov__*/
