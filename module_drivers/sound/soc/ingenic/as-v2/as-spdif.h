/*
 * ALSA Soc Audio Layer -- Ingenic AS BAIC Controller
 *
 * Copyright 2017 - 2022 Ingenic Semiconductor Co.,Ltd
 *     wqshao <wangquan.shao@ingenic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef __AS_SPDIF_H__
#define __AS_SPDIF_H__

#include <linux/bitops.h>

struct ingenic_spdif {
	struct device   *dev;
	struct regmap *spdif_out_regmap;
	struct regmap *spdif_in_regmap;
	void __iomem    *spdif_out_base;
	void __iomem    *spdif_in_base;
	bool non_pcm;
	u32 sysclk;
	u32 out_clk_div;
	u32 in_clk_div;
	struct snd_soc_dai_driver *dai_driver;

	struct clk *clk;
	struct clk *clk_gate;
};

/* SPDIF OUT */
#define SPENA   (0x0)
#define SPO_SPEN            BIT(0)

#define SPCTRL  (0x4)
#define SPO_D_TYPE          BIT(14)
#define SPO_SIGN_N          BIT(13)
#define SPO_INVALID         BIT(12)
#define SPO_SFT_RST         BIT(11)

#define SPCFG1  (0x8)
#define SPO_SCR_NUM_SFT     (8)
#define SPO_SCR_NUM_MASK    GENMASK(11, SPO_SCR_NUM_SFT)
#define SPO_SET_SCR_NUM(x)  ((x) << SPO_SCR_NUM_SFT)
#define SPO_CH1_NUM_SFT     (4)
#define SPO_CH1_NUM_MASK    GENMASK(7, SPO_CH1_NUM_SFT)
#define SPO_SET_CH1_NUM(x)  ((x) << SPO_CH1_NUM_SFT)
#define SPO_CH2_NUM_SFT     (0)
#define SPO_CH2_NUM_MASK    GENMASK(3, SPO_CH2_NUM_SFT)
#define SPO_SET_CH2_NUM(x)  ((x) << SPO_CH2_NUM_SFT)

#define SPCFG2  (0xc)
#define SPO_FS_SFT          (26)
#define SPO_FS_MASK         GENMASK(29, SPO_FS_SFT)
#define SPO_SET_FS(x)       ((x) << SPO_FS_SFT)
#define SPO_ORG_FRQ_SFT     (22)
#define SPO_ORG_FRQ_MASK    GENMASK(25, SPO_ORG_FRQ_SFT)
#define SPO_SET_ORG_FRQ(x)  ((x) << SPO_ORG_FRQ_SFT)

#define SPO_SAMPL_WL_SFT    (19)
#define SPO_SAMPL_WL_MASK   GENMASK(21, SPO_SAMPL_WL_SFT)
#define SAMPL_WL_16         (0x1 << SPO_SAMPL_WL_SFT)
#define SAMPL_WL_17_21      (0x6 << SPO_SAMPL_WL_SFT)
#define SAMPL_WL_18_22      (0x2 << SPO_SAMPL_WL_SFT)
#define SAMPL_WL_19_23      (0x4 << SPO_SAMPL_WL_SFT)
#define SAMPL_WL_24         (0x6 << SPO_SAMPL_WL_SFT)
#define SAMPL_WL0_20        (0x5 << SPO_SAMPL_WL_SFT)
#define SAMPL_WL1_20        (0x1 << SPO_SAMPL_WL_SFT)
#define SPO_SET_SAMPL_WL(wl, max_wl) (((wl == 16) && !max_wl) ? SAMPL_WL_16 : ((wl == 17) || ( wl == 21)) ? SAMPL_WL_17_21 : ((wl == 18) || (wl == 22)) ? SAMPL_WL_18_22 : ((wl == 19) || (wl == 23)) ? SAMPL_WL_19_23 : (wl == 24) ? SAMPL_WL_24 : (wl == 20) && max_wl ? SAMPL_WL1_20 : SAMPL_WL0_20)
#define SPO_MAX_WL          BIT(18)
#define SPO_CAT_CODE_SFT    (8)
#define SPO_CAT_CODE_MASK   GENMASK(15, SPO_CAT_CODE_SFT)
#define SPO_CAT_CODE_GEN    (0x0 << SPO_CAT_CODE_SFT)
#define SPO_CAT_CODE_DVD    (0x4c << SPO_CAT_CODE_SFT)
#define SPO_CH_MD_SFT       (6)
#define SPO_CH_MD_MASK      GENMASK(7, SPO_CH_MD_SFT)
#define SPO_CH_MD_0         (0x0 << SPO_CH_MD_SFT)
#define SPO_AUDIO_N         BIT(1)
#define SPO_CON_PRO         BIT(0)

#define SPDIV   (0x10)
#define SPO_DV_SFT          (0)
#define SPO_SET_DV(x) (((x) > ((1 << 7) - 1) ? ((1 << 7) - 1) : (x)) << SPO_DV_SFT)

/* SPDIF IN */
#define SPIENA  (0x0)
#define SPI_RESET           BIT(1)
#define SPI_SPIEN           BIT(0)

#define SPICFG1 (0x4)
#define SPICFG2 (0x8)
#define SPICFG3 (0xc)
#define SPIDIV  (0x10)
#define SPI_DV_SFT          (0)
#define SPI_SET_DV(x)       SPO_SET_DV(x)

#endif //__AS_SPDIF_H__
