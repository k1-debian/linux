/*
 * Copyright (C) 2016 Ingenic Semiconductor Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __INGENIC_DMA_H__
#define __INGENIC_DMA_H__
#include <dt-bindings/dma/ingenic-pdma.h>
#include "virt-dma.h"
#include "dmaengine.h"
#define CH_DSA  0x00
#define CH_DTA  0x04
#define CH_DTC  0x08
#define CH_DRT  0x0C
#define CH_DCS  0x10
#define CH_DCM  0x14
#define CH_DDA  0x18
#define CH_DSD  0x1C

#define TCSM    0x2000

#define DMAC    0x1000
#define DIRQP   0x1004
#define DDR 0x1008
#define DDRS    0x100C
#define DIP 0x1010
#define DIC 0x1014
#define DMACP   0x101C
#define DSIRQP  0x1020
#define DSIRQM  0x1024
#define DCIRQP  0x1028
#define DCIRQM  0x102C
#define DSN0    0x1060
#define DSN1    0x1064
#define DSCIDX0 0x1068
#define DSCIDX1 0x106C

#define DMACH_OFF   0x20
/* DCS */
#define DCS_NDES    BIT(31)
#define DCS_DES8    BIT(30)
#define DCS_CDOA_SFT    8
#define DCS_CDOA_MSK    (0xff << DCS_CDOA_SFT)
#define DCS_AR      BIT(4)
#define DCS_TT      BIT(3)
#define DCS_HLT     BIT(2)
#define DCS_CTE     BIT(0)

/* DTC */
#define DTC_TC_SFT  0
#define DTC_TC_MSK  0xffffff

/* DCM */
#define DCM_SAI     BIT(23)
#define DCM_DAI     BIT(22)
#define DCM_SAIW        BIT(21)
#define DCM_DAIW        BIT(20)
#define DCM_PORT_MSK    (0xf << 12)
#define DCM_PORT_8  (0x1 << 14 | 0x1 <<12)
#define DCM_PORT_16 (0x2 << 14 | 0x2 <<12)
#define DCM_PORT_32 (0x0 << 14 | 0x0 <<12)
#define DCM_RDIL_SFT    16
#define DCM_RDIL_MAX    0x9
#define DCM_RDIL_MSK    (0xf << DCM_RDIL_SFT)
#define DCM_TSZ_SFT 8
#define DCM_TSZ_AUTO    0x7
#define DCM_TSZ_MSK (0x7 << DCM_TSZ_SFT)
#define DCM_TSZ_EXTR    (0x8 << DCM_TSZ_SFT)
#define DCM_STDE    BIT(2)
#define DCM_TIE     BIT(1)
#define DCM_LINK    BIT(0)

/* DDA */
#define DDA_DBA_SFT 12
#define DDA_DBA_MSK (0xfffff << DDA_DBA_SFT)
#define DDA_DOA_SFT 4
#define DDA_DOA_MSK (0xff << DDA_DOA_SFT)
#define PHY_TO_DESC_DOA(dma)     ((((dma) & DDA_DOA_MSK) >> DDA_DOA_SFT) << 24)

/* DSD */
#define DSD_TSD_SFT 16
#define DSD_TSD_MSK (0xffff << DSD_TSD_SFT)
#define DSD_SSD_SFT 0
#define DSD_SSD_MSK (0xffff << DSD_SSD_SFT)

/* DMAC */
#define DMAC_FMSC   BIT(31)
#define DMAC_FSSI   BIT(30)
#define DMAC_FTSSI  BIT(29)
#define DMAC_FUART  BIT(28)
#define DMAC_FAIC   BIT(27)
#define DMAC_INTCC_SFT  17
#define DMAC_INTCC_MSK  (0x1f << 17)
#define DMAC_INTCE  BIT(16)
#define DMAC_HLT    BIT(3)
#define DMAC_AR     BIT(2)
#define DMAC_CH01   BIT(1)
#define DMAC_DMAE   BIT(0)

/* MCU of PDMA */
#define DMCS    0x1030
#define DMNMB   0x1034
#define DMSMB   0x1038
#define DMINT   0x103C

/* MCU of PDMA */
#define DMINT_S_IP      BIT(17)
#define DMINT_N_IP      BIT(16)

#define DMA_SPECAIL_CHS 0x3     /*Channel 0 & 1*/

/*8-word hardware dma descriptor*/
struct hdma_desc {
	unsigned long dcm;
	dma_addr_t dsa;
	dma_addr_t dta;
	unsigned long dtc;
	unsigned long sd;
	unsigned long drt;
	unsigned long reserved[2];
};

enum sdesc_status {
	STAT_STOPPED = 0, STAT_RUNNING, STAT_ERROR
};

struct ingenic_dma_chan;
struct ingenic_dma_sdesc {
	struct virt_dma_desc    vd;     /* Virtual descriptor */
	int         nb_desc;    /* Number of hw. descriptors */
	size_t          len;        /* Number of bytes xfered */
	bool            cyclic;
	struct hdma_desc    **hw_desc;  /* DMA coherent descriptors */
	dma_addr_t      *hw_desc_dma;   /* DMA address of the Descriptors*/
	struct ingenic_dma_chan *dmac;      /* for free*/
	int         dcs;        /* The DCS initial value */
	int         curr_desc;
	enum sdesc_status   status;
};

struct ingenic_dma_chan {
	struct virt_dma_chan    vc;     /* Virtual channel */
	int             id;     /* Channel id*/
	void __iomem        *iomem;
	struct ingenic_dma_engine   *engine;
	bool            fake_cyclic;

	/*dma slave channel config*/
	unsigned int        slave_id;   /* Request type of the channel */
	enum dma_transfer_direction direction;
	dma_addr_t      slave_addr;
	unsigned int        maxburst;
	unsigned int        transfer_width;
	unsigned int        fast_mode;  /* The fast mode bit of dmac*/
	unsigned int        dcm;        /* The DCM of HW Descriptor initial value*/

	/*Descriptors*/
	struct dma_pool     *hdesc_pool;    /*HW Descriptors pool */
	spinlock_t      hdesc_lock; /*HW Descriptor assign lock*/
	int         hdesc_num;  /*HW Descriptors assigned num*/
	int         hdesc_max;  /*HW Descriptors maxnum capacity*/

	struct ingenic_dma_sdesc    *sdesc;     /*Current Running Async Tx Desc*/

	/*channel terminated completion*/
	struct completion completion;
};

struct ingenic_dma_engine {
	struct device       *dev;
	void __iomem        *iomem;
	struct clk      *gate_clk;
	struct dma_device       dma_device;
	struct device_dma_parameters    dma_parms;

	uint32_t        chan_reserved;
	uint32_t        chan_programed;
	int         intc_ch;
	bool            special_ch;

	/*hardware interrupt*/
	int             irq_pdma;   /* pdma interrupt*/
	int                     irq_pdmam;  /* pdma mcu interrupt */
	int         dma_type;   // ahb2_pdma or ahb_mcu_pdma
	int         irq_pdmad;  /* pdma per-descriptor interrupt*/

	/*hardware params*/
#define HWATTR_INTC_IRQ     (1 << 0)
#define HWATTR_SPECIAL_CH01 (1 << 1)
#define HWATTR_DESC_INTER   (1 << 2)
#define HWATTR_INTC_IRQ_SUP(x)      (HWATTR_INTC_IRQ & (x))
#define HWATTR_SPECIAL_CH01_SUP(x)  (HWATTR_SPECIAL_CH01 & (x))
#define HWATTR_DESC_INTER_SUP(x)    (HWATTR_DESC_INTER & (x))
	unsigned int        hwattr;
	/*channels*/
	int         nr_chs;
	struct ingenic_dma_chan    *chan[];
};

static inline struct device *chan2dev(struct dma_chan *chan)
{
	return &chan->dev->device;
}

static inline struct ingenic_dma_chan *to_ingenic_dma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct ingenic_dma_chan, vc.chan);
}

static inline struct ingenic_dma_sdesc *to_ingenic_dma_sdesc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct ingenic_dma_sdesc, vd);
}

u32 ingenic_dma_read_dtc(struct dma_chan *chan);

u32 ingenic_dma_read_dda(struct dma_chan *chan);

int ingenic_dma_get_desc_index(struct dma_chan *chan, unsigned int doa, int desc_num);

#endif /*__INGENIC_DMA_H__*/
