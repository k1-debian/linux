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
#ifndef __AS_MIXER__
#define __AS_MIXER__

#include <linux/bitops.h>

#define MIX_OFF     (0x4)
#define MIX_CTL0    (0x0)
#define MIX_CFG0    (0x4)

#define MIX_CTL(n)  (MIX_CTL0 + (n) * MIX_OFF)
#define MIX_CFG(n)  (MIX_CFG0 + (n) * MIX_OFF)

#define MIX_RESET   BIT(1)
#define MIX_EN_SFT  (0)
#define MIX_EN      BIT(MIX_EN_SFT)

#define MIX_CH_SFT  (5)
#define MIX_CH_MSK  GENMASK(6, MIX_CH_SFT)
#define MIX_CH(n)   ((((n) - 1) << MIX_CH_SFT) & MIX_CH_MSK)

#define MIX_MODE_SFT    (3)
#define MIX_MODE_MSK    GENMASK(4, MIX_MODE_SFT)
#define MIX_MODE_LWP    (0 << MIX_MODE_SFT)
#define MIX_MODE_AVG    (1 << MIX_MODE_SFT)
#define MIX_MODE_CLMP   (2 << MIX_MODE_SFT)
#define MIX_MODE_NOL    (3 << MIX_MODE_SFT)

#define MIX_LR_MIX_SFT  (1)
#define MIX_LR_MIX      BIT(MIX_LR_MIX_SFT)
#define MIX_LAST_SMP    BIT(0)

struct ingenic_as_mixer {
	struct regmap *regmap;
	struct device *dev;
	void *__iomem io_base;
	u32 num_mixers;
	struct snd_soc_component_driver cmpnt_drv;
};
#endif /*__AS_MIXER__*/
