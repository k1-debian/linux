#ifndef __ASOC_DMIC_V2_H__
#define __ASOC_DMIC_V2_H__

#include <linux/bitops.h>

/* DMIC Controller Registers */
#define DBA                     (0x78)                          /* DMA Bus Address */
#define DTC                     (0x7c)                          /* DMA Transfer Count */
#define DCR                     (0x80)                          /* DMA Channel Control Register */
#define DSR                     (0x84)                          /* DMA Channel Status Register */
#define DCM                     (0x88)                          /* DMA Channel Command */
#define DDA                     (0x8c)                          /* DMA Descriptor Address */
#define DGRR                    (0x100)                         /* DMA Global Reset Register */
#define DGER                    (0x104)                         /* DMA Global Enable Register */
#define DEIER                   (0x108)                         /* DMIC Exception Interrupt Enable Register */
#define DEISR                   (0x10c)                         /* DMIC Exception Interrupt Status Register */
#define DIPR                    (0x110)                         /* DMIC Interrupt Pending Register */
#define FAS                     (0x1050)                        /* FIFO Address Space */
#define FCR                     (0x1054)                        /* FIFO Control Register */
#define FFR                     (0x1058)                        /* FIFO Configuration Register */
#define FSR                     (0x105c)                        /* FIFO Status Register */
#define DFCR                    (0x2014)                        /* Data Format Control Register */
#define FDR                     (0x3000)                        /* FIFO Data Register */
#define BTSET                   (0x4000)                        /* BUS TimeSlot Set Register */
#define BTCLR                   (0x4004)                        /* BUS TimeSlot Clear Register */
#define BTSR                    (0x4008)                        /* BUS TimeSlot Status Register */
#define BFSR                    (0x400c)                        /* BUS FIFO Status Register */
#define BFCR0                   (0x4010)                        /* BUS FIFO Control Register 0 */
#define BFCR1                   (0x4014)                        /* BUS FIFO Control Register 1 */
#define BFCR2                   (0x4018)                        /* BUS FIFO Control Register 2 */
#define BSTR0                   (0x401c)                        /* BUS Source Timeslot Register 0 */
#define BTTR0                   (0x4028)                        /* BUS Target Timeslot Register 0 */
#define DMIC_CR0                (0xa000)                        /* DMIC Control register 0 */
#define DMIC_GCR                (0xa004)                        /* DMIC Gain Control Register */
#define DMIC_TRI_IER            (0xa008)                        /* DMIC Trigger Interrupt Enable Register */
#define DMIC_TRI_ICR            (0xa00c)                        /* DMIC Trigger Interrupt Control Register */
#define DMIC_TCR                (0xa010)                        /* DMIC Trigger Control register */
#if 0
	#define DMIC_THR_L1             (0xa014)
	#define DMIC_THR_H2             (0xa018)
	#define DMIC_THR_L2             (0xa01c)
	#define DMIC_TRI_MMAX           (0xa020)
	#define DMIC_TRI_NMAX           (0xa024)
	#define DMIC_THR_L3             (0xa028)
	#define DMIC_TRI_TM             (0xa02c)
	#define DMIC_FGDIS              (0xa030)
	#define DMIC_CGDIS              (0xa03c)
	#define DMIC_VTSR               (0xa040)
#endif

/* DTC */
#define DTC_TC_SFT              (0)
#define DTC_TC                  GENMASK(23, DTC_TC_SFT)

/* DCR */
#define DCR_DFS                 BIT(1)
#define DCR_CTE                 BIT(0)

/* DSR */
#define DSR_CDOA_SFT            (8)
#define DSR_CDOA                GENMASK(15, DSR_CDOA_SFT)
#define DSR_TT_INT              BIT(5)
#define DSR_LTT_INT             BIT(4)
#define DSR_TT                  BIT(3)
#define DSR_LTT                 BIT(2)
#define DSR_LINK                BIT(0)

/* DCM */
#define DCM_NDES                BIT(31)
#define DCM_BAI                 BIT(8)
#define DCM_TSZ_SFT             (4)
#define SET_DMA_BURST_SZ(burst) (0x1 << (burst + 1))
#define DMA_BURST_1_WORD        (0x0)
#define DMA_BURST_4_WORD        (0x1)
#define DMA_BURST_8_WORD        (0x2)
#define DMA_BURST_16_WORD       (0x3)
#define DMA_BURST_32_WORD       (0x4)
#define DCM_TSZ_MAX             DMA_BURST_32_WORD
#define DCM_TSZ_MAX_WORD        SET_DMA_BURST_SZ(DCM_TSZ_MAX)
#define DCM_TSZ_MSK             GENMASK(6, DCM_TSZ_SFT)
#define DCM_TIE                 BIT(1)
#define DCM_LTIE                BIT(0)

/* DGRR */
#define DGRR_RESET              BIT(0)

/* DGER */
#define DGER_DMA_EN             BIT(0)

/* DEIER */
#define DEIER_EXP_EN            BIT(0)

/* DEISR */
#define DEISR_EXP               BIT(0)

/* DIPR */
#define DIPR_EXP_INT            BIT(11)
#define DIPR_DMIC_INT           BIT(10)
#define DIPR_DMA_INT            BIT(5)

/* FAS */
#define FAS_FAD_SFT             (0)
#define FAS_FAD                 GENMASK(9, FAS_FAD_SFT)

/* FCR */
#define FCR_FLUSH               BIT(2)
#define FCR_FULL_EN             BIT(1)
#define FCR_FIFO_EN             BIT(0)

/* FFR */
#define FFR_FTH_SFT             (16)
#define FFR_FTH_MSK             GENMASK(25, FFR_FTH_SFT)
#define FFR_FTH(n)              (((n) << FFR_FTH_SFT) & FFR_FTH_MSK)
#define FFR_TURROREN            BIT(9)
#define FFR_FSEN                BIT(8)
#define FFR_FIFO_TD             BIT(7)

/* FSR */
#define FSR_FLEVEL_SFT          (16)
#define FSR_FLEVEL              GENMASK(28, FSR_FLEVEL_SFT)
#define FSR_TURROR_CC           BIT(5)
#define FSR_TFSRFS_CC           BIT(4)
#define FSR_TURROR              BIT(2)
#define FSR_TFSRFS              BIT(0)

/* DFCR */
#define DFCR_CHNUM_SFT          (12)
#define DFCR_CHNUM_MSK          GENMASK(15, DFCR_CHNUM_SFT)
#define DFCR_CHNUM(ch)          ((((!((ch) % 2) || ((ch) == 1)) ? ((ch) - 1) : (ch)) << DFCR_CHNUM_SFT) & DFCR_CHNUM_MSK)
#define DFCR_SS_SFT             (8)
#define DFCR_SS_MSK             GENMASK(10, DFCR_SS_SFT)
#define DFCR_SS_8               (0 << DFCR_SS_SFT)
#define DFCR_SS_12              (1 << DFCR_SS_SFT)
#define DFCR_SS_13              (2 << DFCR_SS_SFT)
#define DFCR_SS_16              (3 << DFCR_SS_SFT)
#define DFCR_SS_18              (4 << DFCR_SS_SFT)
#define DFCR_SS_20              (5 << DFCR_SS_SFT)
#define DFCR_SS_24              (6 << DFCR_SS_SFT)
#define DFCR_SS_32              (7 << DFCR_SS_SFT)
#define DFCR_SS(ss)             ((ss <= 8) ? DFCR_SS_8 : (ss <= 12) ? DFCR_SS_12 : (ss == 13) ? DFCR_SS_13 : (ss <= 16) ? DFCR_SS_16 : (ss <= 18) ? DFCR_SS_18 : (ss <= 20) ? DFCR_SS_20 : (ss <= 24) ? DFCR_SS_24: DFCR_SS_32)
#define DFCR_PACK_EN            BIT(2)
#define DFCR_RECEN              BIT(0)

/* BTSET */
#define BTSET_TSLOT_SET         BIT(1)

/* BFSR*/
#define BFSR_DEV5_TUR           BIT(21)
#define BFSR_DEV0_ROR           BIT(0)

/* BFCR0 */
#define BFCR0_DEV5_TFIFO        BIT(21)
#define BFCR0_DEV0_RFIFO        BIT(0)

/* BFCR1 */
#define BFCR1_DEV5_LSMP         BIT(5)

/* BFCR2 */
#define BFCR2_DEV0_DBE          BIT(0)

/* BSTR0 */
#define BSTR0_DEV0_SUR          BIT(0)

/* BTTR0 */
#define BTTR0_DEV5_TAR          BIT(26)

/* DMIC_CR0 */
#define DMIC_CR0_SOFT_RST       BIT(31)
#define DMIC_CR0_HPF2_EN_SFT    (22)
#define DMIC_CR0_HPF2_EN        BIT(22)
#define DMIC_CR0_LPF_EN_SFT     (21)
#define DMIC_CR0_LPF_EN         BIT(21)
#define DMIC_CR0_HPF1_EN_SFT    (20)
#define DMIC_CR0_HPF1_EN        BIT(20)
#define DMIC_CR0_CHNUM_SFT      (16)
#define DMIC_CR0_CHNUM_MASK     GENMASK(19, DMIC_CR0_CHNUM_SFT)
#define DMIC_CR0_SET_CHNUM(chl) (((chl) - 1) << DMIC_CR0_CHNUM_SFT)
#define DMIC_CR0_OSS_SFT        (12)
#define DMIC_CR0_OSS_MASK       GENMASK(13, DMIC_CR0_OSS_SFT)
#define OSS_16BIT               (0x0 << DMIC_CR0_OSS_SFT)
#define OSS_24BIT               (0x1 << DMIC_CR0_OSS_SFT)
#define DMIC_CR0_SET_OSS(x)     ((x == 16) ? OSS_16BIT : (x == 24) ? OSS_24BIT : OSS_16BIT)
#define DMIC_CR0_SW_LR_SFT      (11)
#define DMIC_CR0_SW_LR          BIT(11)
#define DMIC_CR0_SR_SFT         (6)
#define DMIC_CR0_SR_MASK        GENMASK(8, DMIC_CR0_SR_SFT)
#define SR_8k                   (0x0 << DMIC_CR0_SR_SFT)
#define SR_16k                  (0x1 << DMIC_CR0_SR_SFT)
#define SR_48k                  (0x2 << DMIC_CR0_SR_SFT)
#define SR_96k                  (0x3 << DMIC_CR0_SR_SFT)
#define DMIC_CR0_SET_SR(r)      ((r) == 8000 ? SR_8k : (r == 16000) ? SR_16k : (r == 48000) ? SR_48k : (r == 96000) ? SR_96k : SR_48k)
#define DMIC_CR0_FREQ           BIT(2)
#define DMIC_CR0_EXEN           BIT(1)
#define DMIC_CR0_DMIC_EN        BIT(0)

/* DMIC_GCR */
#define DGAIN_SFT               (0)
#define DMIC_GCR_SET_DGAIN(x)   ((x) << DGAIN_SFT)

/* DMIC_TRI_IER */
#define DMIC_TRI_IER_FTRIINTEN  BIT(4)
#define DMIC_TRI_IER_WUINTEN    BIT(0)

/* DMIC_TRI_ICR */
#define DMIC_TRI_ICR_FTRIFLAG   BIT(4)
#define DMIC_TRI_ICR_WUFLAG     BIT(0)

/* DMIC_TCR */

#define INGENIC_DEF_FIFO_DEPTH (4096)
#define INGENIC_PERIODS_MIN (8)
#define INGENIC_PERIODS_BYTES_MIN (4096)
#define INGENIC_DMA_BUFFERSIZE_MAX (CONFIG_SND_ASOC_INGENIC_DMA_PREALLOC_PAGES * 4096)
#define INGENIC_DMA_BUFFERSIZE_PREALLOC INGENIC_DMA_BUFFERSIZE_MAX

enum dmic_clk_mode {
	DMIC_CLOCK_GATE,
	DMIC_CLOCK_CE,
	DMIC_CLOCK_DIV,
	DMIC_CLOCK_MUX,
};

struct ingenic_dmic_priv {
	enum dmic_clk_mode clk_mode;
	const char *clk_name;
	const char *mux_select;
	bool is_bus_clk;
};

struct ingenic_dmic_dma_desc {
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

struct ingenic_dmic_dma_runtime {
	struct snd_pcm_substream *substream;
	u8 dai_id;
	struct ingenic_dmic_dma_desc *descs[INGENIC_DMA_BUFFERSIZE_MAX / INGENIC_PERIODS_BYTES_MIN];
	dma_addr_t descs_phy[INGENIC_DMA_BUFFERSIZE_MAX / INGENIC_PERIODS_BYTES_MIN];
	unsigned int hw_ptr;

	/* dumm desc */
	struct ingenic_dmic_dma_desc *dummy_descs;
	dma_addr_t dummy_descs_phy;
	bool dummy_data_flag;
	bool dummy_data_irq;

	uint8_t desc_cnts;
	uint32_t fifo_depth;
	uint32_t tsz_words;
	struct ingenic_dmic_dma *dmic_dma;
};

struct ingenic_dmic_dma {
	int irq;
	struct dma_pool *desc_pool;

	/* dummy data */
	uint8_t *dummy_data;
	dma_addr_t dummy_data_pyaddr;

	uint32_t fifo_total_depth;
	bool dma_fth_quirk;

	struct clk *audio_clk;

	spinlock_t lock;
	struct ingenic_dmic_dma_runtime *rtd;
	uint32_t fifo_depth;
	int refcnt;
};

struct ingenic_dmic {
	struct device *dev;
	struct regmap *regmap;
	void __iomem    *io_base;
	struct snd_soc_dai_driver *dai_driver;
	struct ingenic_dmic_dma *dma;
	struct clk *clk;
	const struct ingenic_dmic_priv *priv;
};

#endif          /* __ASOC_DMIC_V2_H__ */
