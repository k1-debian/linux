/*
 * ALSA Soc Audio Layer -- Ingenic AS BAIC Controller
 *
 * Copyright 2017 - 2022 Ingenic Semiconductor Co.,Ltd
 *     cli <chen.li@ingenic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef __AS_BAIC_H__
#define __AS_BAIC_H__

#include <linux/bitops.h>
#include <dt-bindings/sound/ingenic-baic.h>
#define MAX_BAIC_NUM        (5)
#define BAIC_SUPPORT_FORMAT_MAX (11)
/*CLKID*/
#define CLKID_SYSCLK        0x1
#define CLKID_SYSCLK_T      0x2
#define CLKID_SYSCLK_R      0x3

#define DIVID_DIR_T BIT(0)
#define DIVID_SFT   (1)
#define DIVID_MSK   GENMASK(2, DIVID_SFT)
#define DIVID_SYNC_W    (0x0 << DIVID_SFT)  /*divided by bclk*/
#define DIVID_SYNC  (0x1 << DIVID_SFT)  /*divided by bclk*/
#define DIVID_BCLK  (0x2 << DIVID_SFT)  /*divided by sysclk*/

#define BAIC_DAIFMT_T       (0x1 << 16)
#define BAIC_DAIFMT_R       (0x2 << 16)
#define BAIC_DAIFMT_DIR_MSK (0x3 << 16)

#define BAIC_DAIFMT_SEL_PCM  (0x1 << 20)
#define BAIC_DAIFMT_SEL_DSP  (0x2 << 20)
#define BAIC_DAIFMT_SEL_TDM1 (0x3 << 20)
#define BAIC_DAIFMT_SEL_TDM2 (0x4 << 20)
#define BAIC_DAIFMT_SEL_MASK (0xf << 20)

/*REGISTERS*/
#define BAICRCTL0   (0x0)
#define BAICRCFG0   (0x4)
#define BAICRDIV0   (0x8)
#define BAICRCGR0   (0xc)
#define BAICTCTL0   (0x10)
#define BAICTCFG0   (0x14)
#define BAICTDIV0   (0x18)
#define BAICTCGR0   (0x1c)
#define BAICTLCR0   (0x20)
#define BAICCCR0    (0x24)
#define BAICMAX     BAICCCR0

#define BAICOFF     (0x1000)
#define BAICRCTL(n) (BAICRCTL0 + BAICOFF * (n)) /*BAIC Receiver Control Register (BAICRCTL) description*/
#define BAICRCFG(n) (BAICRCFG0 + BAICOFF * (n)) /*BAIC Receiver Configuration Register (BAICRCFG) description*/
#define BAICRDIV(n) (BAICRDIV0 + BAICOFF * (n)) /*BAIC Receiver CLK divide register (BAICRDIV) description*/
#define BAICRCGR(n) (BAICRCGR0 + BAICOFF * (n)) /*BAIC Receiver CLK Gate Register (BAICRCGR) description(Unused)*/

#define BAICTCTL(n) (BAICTCTL0 + BAICOFF * (n)) /*BAIC Transmitter Control Register (BAICTCTL) description*/
#define BAICTCFG(n) (BAICTCFG0 + BAICOFF * (n)) /*BAIC Transmitter Configuration Register (BAICTCFG) description*/
#define BAICTDIV(n) (BAICTDIV0 + BAICOFF * (n)) /*BAIC Transmitter CLK divide register (BAICTDIV） description*/
#define BAICTCGR(n) (BAICTCGR0 + BAICOFF * (n)) /*BAIC Transmitter CLK Gate Register (BAICTCGR) description*/

#define BAICTLCR(n) (BAICTLCR0 + BAICOFF * (n)) /*BAIC Top Level Configure Register (BAICTLCR) description*/
#define BAICCCR(n)  (BAICCCR0 + BAICOFF * (n))  /*BAIC Common Control Register (BAICCCR) description*/

#define BAICCTR_RST BIT(0)

/*#define BAICCFG_T_PAUSE       BIT(31)*/
#define BAICCFG_SLOT_SFT    (24)
#define BAICCFG_SLOT_MSK    GENMASK(27, BAICCFG_SLOT_SFT)
#define BAICCFG_SLOT(slot)  ((((slot) - 1) << BAICCFG_SLOT_SFT) & BAICCFG_SLOT_MSK)
#define BAICCFG_SWLR        BIT(20)
#define BAICCFG_CHANNEL_SFT (17)
#define BAICCFG_CHANNEL_MSK GENMASK(19, BAICCFG_CHANNEL_SFT)
#define BAICCFG_CHANNEL(ch) ((((ch) >> 1) << BAICCFG_CHANNEL_SFT) & BAICCFG_CHANNEL_MSK)
#define BAICCFG_SLS_SFT (13)
#define BAICCFG_SLS_MSK     GENMASK(14, BAICCFG_SLS_SFT)
#define BAICCFG_SLS(sls)    (((sls%16) ? 0 : (((sls) >> 5) + 1) << BAICCFG_SLS_SFT) & BAICCFG_SLS_MSK)
#define BAICCFG_ISYNC       BIT(12)
#define BAICCFG_ASVTSU      BIT(11)
#define BAICCFG_SS_SFT      (8)
#define BAICCFG_SS_MSK      GENMASK(10, BAICCFG_SS_SFT)
#define BAICCFG_SS_8        (0 << BAICCFG_SS_SFT)
#define BAICCFG_SS_12       (1 << BAICCFG_SS_SFT)
#define BAICCFG_SS_13       (2 << BAICCFG_SS_SFT)
#define BAICCFG_SS_16       (3 << BAICCFG_SS_SFT)
#define BAICCFG_SS_18       (4 << BAICCFG_SS_SFT)
#define BAICCFG_SS_20       (5 << BAICCFG_SS_SFT)
#define BAICCFG_SS_24       (6 << BAICCFG_SS_SFT)
#define BAICCFG_SS_32       (7 << BAICCFG_SS_SFT)
#define BAICCFG_SS(ss)      ((ss <= 8) ? BAICCFG_SS_8 : (ss <= 12) ? BAICCFG_SS_12 : (ss == 13) ? BAICCFG_SS_13 : (ss <= 16) ? BAICCFG_SS_16 : (ss <= 18) ? BAICCFG_SS_18 : (ss <= 20) ? BAICCFG_SS_20 : (ss <= 24) ? BAICCFG_SS_24: BAICCFG_SS_32)
#define BAICCFG_MODE_SFT    (2)
#define BAICCFG_MODE_MSK    GENMASK(5, BAICCFG_MODE_SFT)
#define BAICCFG_MODE_PCMA   (0 << BAICCFG_MODE_SFT)
#define BAICCFG_MODE_PCMB   (1 << BAICCFG_MODE_SFT)
#define BAICCFG_MODE_DSPA   (2 << BAICCFG_MODE_SFT)
#define BAICCFG_MODE_DSPB   (3 << BAICCFG_MODE_SFT)
#define BAICCFG_MODE_I2S    (4 << BAICCFG_MODE_SFT)
#define BAICCFG_MODE_LEFTJ  (5 << BAICCFG_MODE_SFT)
#define BAICCFG_MODE_RIGHTJ (6 << BAICCFG_MODE_SFT)
#define BAICCFG_MODE_TDM1A  (8 << BAICCFG_MODE_SFT)     /*one data pin*/
#define BAICCFG_MODE_TDM1B  (9 << BAICCFG_MODE_SFT)
#define BAICCFG_MODE_TDM2A  (10 << BAICCFG_MODE_SFT)        /*At most four data pin*/
#define BAICCFG_MODE_TDM2B  (11 << BAICCFG_MODE_SFT)
#define BAICCFG_NEG     BIT(1)      /*Use BCLK falling edge receive serial data Master mode*/
#define BAICCFG_MASTER      BIT(0)

#define BAICDIV_BCLKDIV_SFT (16)        /*BCLK = SYS_CLK / BCLKDIV*/
#define BAICDIV_BCLKDIV_MSK GENMASK(23, BAICDIV_BCLKDIV_SFT)
#define BAICDIV_BCLKDIV(div)    (((div) << BAICDIV_BCLKDIV_SFT) & BAICDIV_BCLKDIV_MSK)
#define BAICDIV_SYNL_SFT    (8)     /*BAICSYN = ( SYNL + 1 ) * BCLK*/
#define BAICDIV_SYNL_MSK    GENMASK(15, BAICDIV_SYNL_SFT)
#define BAICDIV_SYNL(width) ((((width) - 1) << BAICDIV_SYNL_SFT) & BAICDIV_SYNL_MSK)
#define BAICDIV_SYNLDIV_SFT (0)     /*SYNC = BCLK/(8*(SYNCDIV+1))*/
#define BAICDIV_SYNLDIV_MSK GENMASK(7, BAICDIV_SYNLDIV_SFT)
#define BAICDIV_SYNLDIV_I2S(div)   (((((div)/16) - 1) << BAICDIV_SYNLDIV_SFT) & BAICDIV_SYNLDIV_MSK)
#define BAICDIV_SYNLDIV_DSP(div)   (((((div)/8) - 1) << BAICDIV_SYNLDIV_SFT) & BAICDIV_SYNLDIV_MSK)

#define BAICTLCR_ICDC       BIT(1)
#define BAICTLCR_CLK_SPLIT_EN   BIT(0)

#define BAICCCR_TEN     BIT(3)
#define BAICCCR_REN     BIT(2)

/*struct*/
struct ingenic_baic_div {
	uint32_t sync_div: 8;
	uint32_t synl: 8;
	uint32_t bclkdiv: 8;
	uint32_t reserved24_31: 8;
} __attribute__((packed));

struct ingenic_baic_dai {
	u8 id;
	u16 support_func;
	u16 support_mode;
	u8 data_pin_num;
	bool clk_split;
	bool frame_tmaster;
	bool frame_rmaster;
	u32 tsysclk;
	u32 rsysclk;
	u16 bclk_tratio;
	u16 bclk_rratio;
	u16 select_tmode;
	u16 select_rmode;
	u32 baic_tcfg;
	u32 baic_rcfg;
	union {
		struct ingenic_baic_div v;
		uint32_t r;
	} rdiv, tdiv;

	struct clk *clk_r;
	const char *clk_rname;
	struct clk *clk_t;
	const char *clk_tname;
	struct clk *clk_gate;
	spinlock_t lock;
	struct mutex clk_lock;
	bool clk_gate_en;
};

struct ingenic_baic {
	struct device *dev; /*register access device*/
	struct regmap *regmap;
	void *__iomem io_base;
	int num_dais;
	struct snd_soc_dai_driver *dai_driver;
	struct ingenic_baic_dai *dai;
};

#endif /*__AS_BAIC_H__*/
