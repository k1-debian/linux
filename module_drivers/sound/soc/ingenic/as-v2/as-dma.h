/*
 * ALSA Soc Audio Layer -- Ingenic AS DMA Controller
 *
 * Copyright 2017 - 2022 Ingenic Semiconductor Co.,Ltd
 *     cli <chen.li@ingenic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef __AS_DMA_H__
#define __AS_DMA_H__

#include <linux/bitops.h>

/* DMA */
#define DMA_CH_OFF  (0x18)
#define DBA(ch)     (0x0 + (ch) * DMA_CH_OFF)
#define DTC(ch)     (0x4 + (ch) * DMA_CH_OFF)
#define DCR(ch)     (0x8 + (ch) * DMA_CH_OFF)
#define DSR(ch)     (0xc + (ch) * DMA_CH_OFF)
#define DCM(ch)     (0x10 + (ch) * DMA_CH_OFF)
#define DDA(ch)     (0x14 + (ch) * DMA_CH_OFF)
#define DGRR        (0x100)
#define DGER        (0x104)
#define AEER        (0x108)
#define AESR        (0x10c)
#define AIPR        (0x110)

#define DTC_TC_SFT      (0)
#define DTC_TC          GENMASK(23, DTC_TC_SFT)

#define DCR_RESET       BIT(1)
#define DCR_CTE         BIT(0)

#define DSR_CDOA_SFT        (8)
#define DSR_CDOA        GENMASK(15, DSR_CDOA_SFT)
#define DSR_TT_INT      BIT(5)
#define DSR_LTT_INT     BIT(4)
#define DSR_TT          BIT(3)
#define DSR_LTT         BIT(2)
#define DSR_RST_EN      BIT(1)
#define DSR_LINK        BIT(0)

#define DCM_NDES        BIT(31)
#define DCM_BAI         BIT(8)
#define DCM_TSZ_SFT     (4)
#define SET_DMA_BURST_SZ(burst) (0x1 << (burst + 1))
#define DMA_BURST_4_WORD        (0x1)
#define DMA_BURST_8_WORD        (0x2)
#define DMA_BURST_16_WORD       (0x3)
#define DMA_BURST_32_WORD       (0x4)
#define DCM_TSZ_MAX     DMA_BURST_32_WORD
#define DCM_TSZ_MAX_WORD    SET_DMA_BURST_SZ(DCM_TSZ_MAX)
#define DCM_TSZ_MSK     GENMASK(6, DCM_TSZ_SFT)
#define DCM_TIE         BIT(1)
#define DCM_LTIE        BIT(0)

#define DGRR_RESET      BIT(0)

#define DGER_DMA_EN     BIT(0)

#define AEER_EXP_EN     BIT(0)

#define AESR_EXP        BIT(0)

#define AIPR_MSK        GENMASK(11, 0)
#define AIPR_EXP_INT        BIT(11)
#define AIPR_DIMC_INT       BIT(10)
#define AIPR_DMA_INT_MSK    GENMASK(9, 0)
#define AIPR_DMA_INT(ch)    BIT((ch))

/* FIFO */
#define FIFO_CH_OFF (0x10)
#define FAS(ch)     (0x0 + (ch) * FIFO_CH_OFF)
#define FCR(ch)     (0x4 + (ch) * FIFO_CH_OFF)
#define FFR(ch)     (0x8 + (ch) * FIFO_CH_OFF)
#define FSR(ch)     (0xc + (ch) * FIFO_CH_OFF)

#define FAS_FAD_SFT (0)
#define FAS_FAD     GENMASK(12, FAS_FAD_SFT)

#define FCR_FLUSH   BIT(2)
#define FCR_FULL_EN BIT(1)
#define FCR_FIFO_EN BIT(0)

#define FFR_FTH_SFT (16)
#define FFR_FTH_MSK GENMASK(28, FFR_FTH_SFT)
#define FFR_FTH(n)  (((n) << FFR_FTH_SFT) & FFR_FTH_MSK)
#define FFR_TURRORE BIT(9)
#define FFR_FSE     BIT(8)
#define FFR_FIFO_TD BIT(7)

#define FSR_FLEVEL_SFT  (16)
#define FSR_FLEVEL  GENMASK(28, FSR_FLEVEL_SFT)
#define FSR_TURROR_INT  BIT(5)
#define FSR_TFSRFS_INT  BIT(4)
#define FSR_TURROR  BIT(2)
#define FSR_TFSRFS  BIT(0)

struct ingenic_as_dma_desc {
	/*desc0*/
	uint32_t ltie: 1;   /* Last Transfer Interrupt Enable */
	uint32_t tie: 1;    /* Transfer Interrupt Enable */
	uint32_t link: 1;   /* Enable Descriptor Link Enable */
	uint32_t lte: 1;    /* Last Descriptor Transfer end Status */
	uint32_t tsz: 3;    /* Transfer Data Size (burst)*/
	uint32_t reserved0_7: 1;
	uint32_t bai: 1;    /* Target Address Increment */
	uint32_t reserved0_9_31: 23;
	/*desc 1*/
	uint32_t dba;       /* Bus Address (memory address)*/
	/*desc 2*/
	uint32_t ndoa;      /* Descriptor Next Offset address*/
	/*desc 3*/
	uint32_t dtc: 24;   /*Transfer Counter(burst count)*/
	uint32_t reserved3_24_31: 8;
} __attribute__((packed));

#define INGENIC_DEF_FIFO_DEPTH (4096)
#define INGENIC_PERIODS_MIN (4)
#define INGENIC_PERIODS_BYTES_MIN (4096)
#define INGENIC_DMA_BUFFERSIZE_MAX (CONFIG_SND_ASOC_INGENIC_DMA_PREALLOC_PAGES * 4096)
#define INGENIC_DMA_BUFFERSIZE_PREALLOC INGENIC_DMA_BUFFERSIZE_MAX

struct ingenic_as_dma_runtime {
	struct snd_pcm_substream *substream;
	u8 dai_id;
	struct ingenic_as_dma_desc *descs[INGENIC_DMA_BUFFERSIZE_MAX / INGENIC_PERIODS_BYTES_MIN];
	dma_addr_t descs_phy[INGENIC_DMA_BUFFERSIZE_MAX / INGENIC_PERIODS_BYTES_MIN];
	unsigned int hw_ptr;

	/* dumm desc */
	struct ingenic_as_dma_desc *dummy_descs;
	dma_addr_t dummy_descs_phy;
	bool dummy_data_flag;
	bool dummy_data_irq;

	uint8_t desc_cnts;
	uint32_t fifo_depth;
	uint32_t tsz_words;
	struct ingenic_as_dma *as_dma;
};

struct ingenic_as_dma {
	struct device *dev;
	struct regmap *dma_regmap;
	struct regmap *fifo_regmap;
	void __iomem *dma_base;
	void __iomem *fifo_base;
	int irq;
	int chan_cnts;
	struct dma_pool *desc_pool;

	/* dummy data */
	uint8_t *dummy_data;
	dma_addr_t dummy_data_pyaddr;

	uint32_t fifo_total_depth;
	bool dma_fth_quirk;

	struct clk *audio_clk;

	spinlock_t lock;
	struct ingenic_as_dma_runtime **rtd;
	uint32_t *fifo_depth;
	int refcnt;
};

static inline struct ingenic_as_dma_runtime *substream_to_prtd(
    const struct snd_pcm_substream *substream)
{
	return substream->runtime->private_data;
}

static inline int ingenic_as_dma_get_tsz(int fifo_depth,
        int period_sz, int *ts_byte_sz)
{
	int tsz_reg = 0, ts_word_sz = 1;

	if (period_sz & ((4 << 2) - 1)) {
		*ts_byte_sz = ts_word_sz << 2;
		return tsz_reg;
	}
	period_sz >>= 2;    /*word size*/

	period_sz >>= 2;
	ts_word_sz <<= 2;
	tsz_reg++;

	while ((!(period_sz & 0x1)) &&
	       (ts_word_sz << 1) <= fifo_depth) {
		period_sz >>= 1;
		ts_word_sz <<= 1;
		tsz_reg++;
		if (ts_word_sz >= DCM_TSZ_MAX_WORD) {
			break;
		}
	}

	if (ts_byte_sz) {
		*ts_byte_sz = ts_word_sz << 2;
	}
	return tsz_reg;
}
#endif /*__AS_DMA_H__*/
