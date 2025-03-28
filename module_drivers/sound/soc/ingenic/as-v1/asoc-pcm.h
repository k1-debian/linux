
#ifndef __ASOC_PCM_H__
#define __ASOC_PCM_H__

#include <sound/dmaengine_pcm.h>
#include "asoc-dma.h"

#define PCMCTL      0x00
#define PCMCFG      0x04
#define PCMDP       0x08
#define PCMINTC     0x0C
#define PCMINTS     0x10
#define PCMDIV      0x14

#define INGENIC_PCM_CLKDIV (0x1 << 0)
#define INGENIC_PCM_SYNDIV (0x1 << 1)

enum pcm_clk_mode {
	PCM_CLOCK_GATE,
	PCM_CLOCK_CE,
	PCM_CLOCK_DIV,
	PCM_CLOCK_MUX,
};

struct ingenic_pcm_priv {
	enum pcm_clk_mode clk_mode;
	char *clk_name;
	char *mux_select;
	bool is_bus_clk;
};

struct ingenic_pcm {
	struct device   *dev;
	resource_size_t res_start;
	resource_size_t res_size;
	void __iomem    *vaddr_base;
	u32 master_mode;
	/* for clock */
	struct ingenic_pcm_priv *priv;
	struct clk  *clk;
	int rate;
#define PCM_READ    0x1
#define PCM_WRITE   0X2
	int pcm_mode;
	/*for dma*/
	struct snd_dmaengine_dai_dma_data tx_dma_data;
	struct snd_dmaengine_dai_dma_data rx_dma_data;
	struct ingenic_dma_pcm *ingenic_pcm;
};

static void inline pcm_write_reg(struct device *dev, unsigned int reg,
                                 unsigned int val)
{
	struct ingenic_pcm *ingenic_pcm = dev_get_drvdata(dev);
	writel(val, ingenic_pcm->vaddr_base + reg);
}

static unsigned int inline pcm_read_reg(struct device *dev, unsigned int reg)
{
	struct ingenic_pcm *ingenic_pcm = dev_get_drvdata(dev);
	return readl(ingenic_pcm->vaddr_base + reg);
}

#define pcm_set_reg(dev, addr, val, mask, offset)\
	do {    \
		int tmp_val = val;                          \
		int read_val = pcm_read_reg(dev, addr);         \
		read_val &= (~mask);                    \
		tmp_val = ((tmp_val << offset) & mask); \
		tmp_val |= read_val;                    \
		pcm_write_reg(dev, addr, tmp_val);           \
	}while(0)

#define pcm_get_reg(dev, addr, mask, offset)  \
	((pcm_read_reg(dev, addr) & mask) >> offset)

/*PCMCTL*/
#define PCM_ERDMA_OFFSET    (9)
#define PCM_ERDMA_MASK          (0x1 << PCM_ERDMA_OFFSET)
#define PCM_ETDMA_OFFSET    (8)
#define PCM_ETDMA_MASK          (0x1 << PCM_ETDMA_OFFSET)
#define PCM_LSMP_OFFSET     (7)
#define PCM_LSMP_MASK           (0x1 << PCM_LSMP_OFFSET)
#define PCM_ERPL_OFFSET     (6)
#define PCM_ERPL_MASK           (0x1 << PCM_ERPL_OFFSET)
#define PCM_EREC_OFFSET     (5)
#define PCM_EREC_MASK           (0x1 << PCM_EREC_OFFSET)
#define PCM_FLUSH_OFFSET    (4)
#define PCM_FLUSH_MASK          (0x1 << PCM_FLUSH_OFFSET)
#define PCM_RST_OFFSET      (3)
#define PCM_RST_MASK            (0x1 << PCM_RST_OFFSET)
#define PCM_CLKEN_OFFSET    (1)
#define PCM_CLKEN_MASK          (0x1 << PCM_CLKEN_OFFSET)
#define PCM_PCMEN_OFFSET    (0)
#define PCM_PCMEN_MASK          (0x1 << PCM_PCMEN_OFFSET)

#define __pcm_transmit_dma_is_enable(dev) \
	pcm_get_reg(dev,PCMCTL,PCM_ETDMA_MASK,PCM_ETDMA_OFFSET)
#define __pcm_enable_transmit_dma(dev)    \
	pcm_set_reg(dev,PCMCTL,1,PCM_ETDMA_MASK,PCM_ETDMA_OFFSET)
#define __pcm_disable_transmit_dma(dev)   \
	pcm_set_reg(dev,PCMCTL,0,PCM_ETDMA_MASK,PCM_ETDMA_OFFSET)

#define __pcm_receive_dma_is_enable(dev)  \
	pcm_get_reg(dev,PCMCTL,PCM_ERDMA_MASK,PCM_ERDMA_OFFSET)
#define __pcm_enable_receive_dma(dev)     \
	pcm_set_reg(dev,PCMCTL,1,PCM_ERDMA_MASK,PCM_ERDMA_OFFSET)
#define __pcm_disable_receive_dma(dev)    \
	pcm_set_reg(dev,PCMCTL,0,PCM_ERDMA_MASK,PCM_ERDMA_OFFSET)

#define __pcm_play_zero(dev)              \
	pcm_set_reg(dev,PCMCTL,0,PCM_LSMP_MASK,PCM_LSMP_OFFSET)
#define __pcm_play_lastsample(dev)        \
	pcm_set_reg(dev,PCMCTL,1,PCM_LSMP_MASK,PCM_LSMP_OFFSET)

#define __pcm_enable_replay(dev)          \
	pcm_set_reg(dev,PCMCTL,1,PCM_ERPL_MASK,PCM_ERPL_OFFSET)
#define __pcm_disable_replay(dev)         \
	pcm_set_reg(dev,PCMCTL,0,PCM_ERPL_MASK,PCM_ERPL_OFFSET)
#define __pcm_enable_record(dev)          \
	pcm_set_reg(dev,PCMCTL,1,PCM_EREC_MASK,PCM_EREC_OFFSET)
#define __pcm_disable_record(dev)         \
	pcm_set_reg(dev,PCMCTL,0,PCM_EREC_MASK,PCM_EREC_OFFSET)

#define __pcm_flush_fifo(dev)            \
	pcm_set_reg(dev, PCMCTL,1,PCM_FLUSH_MASK,PCM_FLUSH_OFFSET)

#define __pcm_reset(dev)                  \
	pcm_set_reg(dev,PCMCTL,1,PCM_RST_MASK,PCM_RST_OFFSET)

#define __pcm_clock_enable(dev)         \
	pcm_set_reg(dev,PCMCTL,1,PCM_CLKEN_MASK,PCM_CLKEN_OFFSET)
#define __pcm_clock_disable(dev)            \
	pcm_set_reg(dev,PCMCTL,0,PCM_CLKEN_MASK,PCM_CLKEN_OFFSET)

#define __pcm_enable(dev)                 \
	pcm_set_reg(dev,PCMCTL,1,PCM_PCMEN_MASK,PCM_PCMEN_OFFSET)
#define __pcm_disable(dev)                \
	pcm_set_reg(dev,PCMCTL,0,PCM_PCMEN_MASK,PCM_PCMEN_OFFSET)

/*PCMCFG*/
#define PCM_SLOT_OFFSET     (13)
#define PCM_SLOT_MASK           (0x3 << PCM_SLOT_OFFSET)
#define PCM_ISS_OFFSET      (12)
#define PCM_ISS_MASK            (0x1 << PCM_ISS_OFFSET)
#define PCM_OSS_OFFSET      (11)
#define PCM_OSS_MASK            (0x1 << PCM_OSS_OFFSET)
#define PCM_IMSBPOS_OFFSET  (10)
#define PCM_IMSBPOS_MASK        (0x1 << PCM_IMSBPOS_OFFSET)
#define PCM_OMSBPOS_OFFSET  (9)
#define PCM_OMSBPOS_MASK        (0x1 << PCM_OMSBPOS_OFFSET)
#define PCM_RFTH_OFFSET     (5)
#define PCM_RFTH_MASK           (0xf << PCM_RFTH_OFFSET)
#define PCM_TFTH_OFFSET     (1)
#define PCM_TFTH_MASK           (0xf << PCM_TFTH_OFFSET)
#define PCM_PCMMOD_OFFSET   (0)
#define PCM_PCMMOD_MASK         (0x1 << PCM_PCMMOD_OFFSET)

#define __pcm_set_slot(dev,n)   \
	pcm_set_reg(dev,PCMCFG,n,PCM_SLOT_MASK,PCM_SLOT_OFFSET)

#define __pcm_set_oss_sample_size(dev,n)   \
	pcm_set_reg(dev,PCMCFG,n,PCM_OSS_MASK,PCM_OSS_OFFSET)
#define __pcm_set_iss_sample_size(dev,n)   \
	pcm_set_reg(dev,PCMCFG,n,PCM_ISS_MASK,PCM_ISS_OFFSET)

#define __pcm_set_msb_normal_in(dev)   \
	pcm_set_reg(dev,PCMCFG,0,PCM_IMSBPOS_MASK,PCM_IMSBPOS_OFFSET)
#define __pcm_set_msb_one_shift_in(dev)   \
	pcm_set_reg(dev,PCMCFG,1,PCM_IMSBPOS_MASK,PCM_IMSBPOS_OFFSET)

#define __pcm_set_msb_normal_out(dev)   \
	pcm_set_reg(dev,PCMCFG,0,PCM_OMSBPOS_MASK,PCM_OMSBPOS_OFFSET)
#define __pcm_set_msb_one_shift_out(dev)   \
	pcm_set_reg(dev,PCMCFG,1,PCM_OMSBPOS_MASK,PCM_OMSBPOS_OFFSET)

#define __pcm_set_transmit_trigger(dev,n)  \
	pcm_set_reg(dev,PCMCFG,n,PCM_TFTH_MASK,PCM_TFTH_OFFSET)
#define __pcm_set_receive_trigger(dev,n)   \
	pcm_set_reg(dev,PCMCFG,n,PCM_RFTH_MASK,PCM_RFTH_OFFSET)

#define __pcm_as_master(dev)             \
	pcm_set_reg(dev,PCMCFG,0,PCM_PCMMOD_MASK,PCM_PCMMOD_OFFSET)
#define __pcm_as_slaver(dev)             \
	pcm_set_reg(dev,PCMCFG,1,PCM_PCMMOD_MASK,PCM_PCMMOD_OFFSET)

/*PCMDP*/
#define PCM_PCMDP_OFFSET        (0)
#define PCM_PCMDP_MASK          (~0)

#define __pcm_read_fifo(dev)        \
	pcm_get_reg(dev,PCMDP,PCM_PCMDP_MASK,PCM_PCMDP_OFFSET);

/*PCMINTC*/
#define PCM_ETFS_OFFSET     (3)
#define PCM_ETFS_MASK           (0X1 << PCM_ETFS_OFFSET)
#define PCM_ETUR_OFFSET     (2)
#define PCM_ETUR_MASK           (0x1 << PCM_ETUR_OFFSET)
#define PCM_ERFS_OFFSET     (1)
#define PCM_ERFS_MASK           (0x1 << PCM_ERFS_OFFSET)
#define PCM_EROR_OFFSET     (0)
#define PCM_EROR_MASK           (0x1 << PCM_EROR_OFFSET)

#define __pcm_enable_receive_intr(dev)    \
	pcm_set_reg(dev,PCMINTC,1,PCM_ERFS_MASK,PCM_ERFS_OFFSET)
#define __pcm_disable_receive_intr(dev)   \
	pcm_set_reg(dev,PCMINTC,0,PCM_ERFS_MASK,PCM_ERFS_OFFSET)

#define __pcm_enable_underrun_intr(dev)   \
	pcm_set_reg(dev,PCMINTC,1,PCM_ETUR_MASK,PCM_ETUR_OFFSET)
#define __pcm_disable_underrun_intr(dev)  \
	pcm_set_reg(dev,PCMINTC,0,PCM_ETUR_MASK,PCM_ETUR_OFFSET)

#define __pcm_enable_transmit_intr(dev)   \
	pcm_set_reg(dev,PCMINTC,1,PCM_ETFS_MASK,PCM_ETFS_OFFSET)
#define __pcm_disable_transmit_intr(dev)  \
	pcm_set_reg(dev,PCMINTC,0,PCM_ETFS_MASK,PCM_ETFS_OFFSET)

#define __pcm_enable_overrun_intr(dev)    \
	pcm_set_reg(dev,PCMINTC,1,PCM_EROR_MASK,PCM_EROR_OFFSET)
#define __pcm_disable_overrun_intr(dev)   \
	pcm_set_reg(dev,PCMINTC,0,PCM_EROR_MASK,PCM_EROR_OFFSET)

/*PCMINTS*/
#define PCM_RSTS_OFFSET     (14)
#define PCM_RSTS_MASK           (0x1 << PCM_RSTS_OFFSET)
#define PCM_TFL_OFFSET      (9)
#define PCM_TFL_MASK            (0x1f << PCM_TFL_OFFSET)
#define PCM_TFS_OFFSET      (8)
#define PCM_TFS_MASK            (0x1 << PCM_TFS_OFFSET)
#define PCM_TUR_OFFSET      (7)
#define PCM_TUR_MASK            (0x1 << PCM_TUR_OFFSET)
#define PCM_RFL_OFFSET      (2)
#define PCM_RFL_MASK            (0x1f << PCM_RFL_OFFSET)
#define PCM_RFS_OFFSET      (1)
#define PCM_RFS_MASK            (0x1 << PCM_RFS_OFFSET)
#define PCM_ROR_OFFSET      (0)
#define PCM_ROR_MASK            (0X1 << PCM_ROR_OFFSET)

#define __pcm_test_rst_complie(dev) \
	!pcm_get_reg(dev,PCMINTS,PCM_RSTS_MASK,PCM_RSTS_OFFSET)
#define __pcm_test_tfl(dev)               \
	pcm_get_reg(dev,PCMINTS,PCM_TFL_MASK,PCM_TFL_OFFSET)
#define __pcm_test_tfs(dev)               \
	pcm_get_reg(dev,PCMINTS,PCM_TFS_MASK,PCM_TFS_OFFSET)
#define __pcm_test_tur(dev)               \
	pcm_get_reg(dev,PCMINTS,PCM_TUR_MASK,PCM_TUR_OFFSET)
#define __pcm_clear_tur(dev)    \
	pcm_set_reg(dev,PCMINTS,0,PCM_TUR_MASK,PCM_TUR_OFFSET)
#define __pcm_test_rfl(dev)               \
	pcm_get_reg(dev,PCMINTS,PCM_RFL_MASK,PCM_RFL_OFFSET)
#define __pcm_test_rfs(dev)               \
	pcm_get_reg(dev,PCMINTS,PCM_RFS_MASK,PCM_RFS_OFFSET)
#define __pcm_clear_ror(dev)    \
	pcm_set_reg(dev,PCMINTS,0,PCM_ROR_MASK,PCM_ROR_OFFSET)
#define __pcm_test_ror(dev)               \
	pcm_get_reg(dev,PCMINTS,PCM_ROR_MASK,PCM_ROR_OFFSET)
/* PCMDIV */
#define PCM_SYNC_OFFSET     (11)
#define PCM_SYNC_MASK           (0x3f << PCM_SYNC_OFFSET)
#define PCM_SYNDIV_OFFSET   (6)
#define PCM_SYNDIV_MASK         (0x1f << PCM_SYNDIV_OFFSET)
#define PCM_CLKDIV_OFFSET   (0)
#define PCM_CLKDIV_MASK         (0x3f << PCM_CLKDIV_OFFSET)

#define __pcm_set_sync(dev,n)   \
	pcm_set_reg(dev,PCMDIV,n,PCM_SYNC_MASK,PCM_SYNC_OFFSET)
#define __pcm_set_syndiv(dev,n) \
	pcm_set_reg(dev,PCMDIV,n,PCM_SYNDIV_MASK,PCM_SYNDIV_OFFSET)
#define __pcm_set_clkdiv(dev,n) \
	pcm_set_reg(dev,PCMDIV,n,PCM_CLKDIV_MASK,PCM_CLKDIV_OFFSET)

#endif /* __ASOC_PCM_H__ */
