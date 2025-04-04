/*
 * Copyright (C) 2016 Ingenic Semiconductor Co., Ltd.
 * Author: cli <chen.li@ingenic.com>
 *
 * Programmable DMA Controller Driver For Ingenic's SOC,
 * such as X1000, and so on. (kernel.4.4)
 *
 *  Author: cli <chen.li@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#undef DEBUG
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/dmapool.h>
#include "ingenic_dma.h"

#if defined(CONFIG_SOC_X2500) || defined(CONFIG_INGENIC_AIC_USES_PDMA)
	#define AIC_USES_PDMA
#endif

#ifdef CONFIG_SOC_X2500
	#include <soc/pdma.h>
#endif

#define JZCAN_RX_DATA_OFFSET    0x90

#define AHB2_PDMA   0
#define AHB_MCU_PDMA    1

/* tsz for 1,2,4,8,16,32,64 128bytes */
const static char dcm_tsz[8] = { 1, 2, 0, 0, 3, 4, 5, 6};

u32 ingenic_dma_read_dtc(struct dma_chan *chan)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
	return readl(dmac->iomem + CH_DTC);
}
EXPORT_SYMBOL_GPL(ingenic_dma_read_dtc);

u32 ingenic_dma_read_dda(struct dma_chan *chan)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
	return readl(dmac->iomem + CH_DDA);
}
EXPORT_SYMBOL_GPL(ingenic_dma_read_dda);

int ingenic_dma_get_desc_index(struct dma_chan *chan, unsigned int doa, int desc_num)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
	struct ingenic_dma_sdesc *sdesc = dmac->sdesc;
	int i;
	for (i = 0; i < desc_num; i++) {
		if ((doa << 24) == PHY_TO_DESC_DOA(sdesc->hw_desc_dma[i])) {
			if (i == 0) {
				return desc_num - 1;
			} else {
				return i - 1;
			}
		}
	}

	return -1;
}
EXPORT_SYMBOL_GPL(ingenic_dma_get_desc_index);

static inline unsigned int get_current_tsz(unsigned long dcmp)
{
	int i;
	int val = (dcmp & DCM_TSZ_MSK) >> DCM_TSZ_SFT;

	if (DCM_TSZ_AUTO == val) {
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(dcm_tsz); i++) {
		if (val == dcm_tsz[i]) {
			break;
		}
	}

	return i;
}

static inline unsigned get_max_tsz(unsigned long val, unsigned int *shift)
{
	int ord = ffs(val) - 1;

	/*
	 * 8 byte transfer sizes unsupported so fall back on 4. If it's larger
	 * than the maximum, just limit it. It is perfectly safe to fall back
	 * in this way since we won't exceed the maximum burst size supported
	 * by the device, the only effect is reduced efficiency. This is better
	 * than refusing to perform the request at all.
	 */
	if (ord == 3) {
		ord = 2;
	} else if (ord > 7) {
		ord = 7;
	}

	if (shift) {
		*shift = ord;
	}

	return dcm_tsz[ord];
}

static const struct of_device_id ingenic_dma_dt_match[];
static struct ingenic_dma_engine *ingenic_dma_parse_dt(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct ingenic_dma_engine *ingenic_dma;
	u32 nr_chs;

	if (!pdev->dev.of_node) {
		return ERR_PTR(-ENODEV);
	}

	match = of_match_node(ingenic_dma_dt_match, pdev->dev.of_node);
	if (!match) {
		return ERR_PTR(-ENODEV);
	}

	if (of_property_read_u32(pdev->dev.of_node, "#dma-channels", &nr_chs)) {
		nr_chs = 32;
	}

	ingenic_dma = devm_kzalloc(&pdev->dev, sizeof(*ingenic_dma) +
	                           sizeof(struct ingenic_dma_chan *) * nr_chs, GFP_KERNEL);
	if (!ingenic_dma) {
		return ERR_PTR(-ENOMEM);
	}

	ingenic_dma->dev = &pdev->dev;
	ingenic_dma->nr_chs = nr_chs;
	ingenic_dma->hwattr = (unsigned int)match->data;

	/* Property is optional, if it doesn't exist the value will remain 0. */
	of_property_read_u32(pdev->dev.of_node, "ingenic,reserved-chs",
	                     &ingenic_dma->chan_reserved);

	ingenic_dma->dma_type = 0;
	of_property_read_u32(pdev->dev.of_node, "ingenic,bus-ctrl", &ingenic_dma->dma_type);

	if (!of_property_read_u32(pdev->dev.of_node, "ingenic,programed-chs",
	                          &ingenic_dma->chan_programed)) {
		ingenic_dma->chan_reserved |= ingenic_dma->chan_programed;
	}

	if (HWATTR_SPECIAL_CH01_SUP(ingenic_dma->hwattr) &&
	    of_property_read_bool(pdev->dev.of_node, "ingenic,special-chs")) {
		ingenic_dma->chan_reserved |= DMA_SPECAIL_CHS;
		ingenic_dma->chan_programed |= DMA_SPECAIL_CHS;
		ingenic_dma->special_ch = true;
	}

	ingenic_dma->intc_ch = -1;
	if (HWATTR_INTC_IRQ_SUP(ingenic_dma->hwattr) &&
	    !of_property_read_u32(pdev->dev.of_node, "ingenic,intc-ch",
	                          (u32 *)&ingenic_dma->intc_ch)) {

		if (ingenic_dma->intc_ch >= ingenic_dma->nr_chs) {
			ingenic_dma->intc_ch = (ingenic_dma->nr_chs - 1);
		}

		if (BIT(ingenic_dma->intc_ch) & ingenic_dma->chan_reserved)
			dev_warn(ingenic_dma->dev, "WARN: intc irq channel %d is already reserved\n",
			         ingenic_dma->intc_ch);

		ingenic_dma->chan_reserved |= BIT(ingenic_dma->intc_ch);
	}
	return ingenic_dma;
}

#ifdef DUMP_DMA_DESC
static int dump_dma_hdesc(struct hdma_desc *desc, const char *d)
{
	int i;
	unsigned long *p;
	printk(KERN_DEBUG "%s(): %s\n", __func__, d);
	p = (unsigned long *)desc;
	for (i = 0; i < 8; i++) {
		printk(KERN_DEBUG "\t%08lx\n", (unsigned long)*p);
		p++;
	}

	return 0;
}
#endif

static unsigned int build_one_desc(dma_addr_t saddr, dma_addr_t daddr,
                                   unsigned int length, struct hdma_desc *desc)
{
	unsigned len = length;

	if (length < DTC_TC_MSK) {
		desc->dcm = DCM_DAI | DCM_SAI | \
		            (DCM_TSZ_AUTO << DCM_TSZ_SFT) | (DCM_RDIL_MAX << DCM_RDIL_SFT);
		desc->dtc = len; /* DCM_TSZ_AUTO in bytes */
	} else {
		unsigned int tsz, transfer_shift;
		len = ALIGN_DOWN(len, sizeof(uint32_t));
		tsz = get_max_tsz(len, &transfer_shift);
		desc->dcm = DCM_DAI | DCM_SAI | tsz << DCM_TSZ_SFT;
		desc->dtc = len >> transfer_shift; /* in burst unit */
		barrier();
		len = desc->dtc << transfer_shift;
	}

	desc->dsa = saddr;
	desc->dta = daddr;
	desc->sd = 0;
	desc->drt = INGENIC_DMA_REQ_AUTO_TX;

	return len;
}

static unsigned int build_one_slave_desc(struct ingenic_dma_chan *dmac, dma_addr_t addr,
        unsigned int length,
        enum dma_transfer_direction direction,
        struct hdma_desc *desc)
{
	enum dma_transfer_direction dir;
	unsigned int rdil;

	desc->dcm = dmac->dcm;
	desc->drt = dmac->slave_id;
	desc->sd = 0;

	if ((direction != DMA_DEV_TO_MEM) && (direction != DMA_MEM_TO_DEV)) {
		dir = dmac->direction;
	} else {
		dir = direction;
	}

	if (dir == DMA_DEV_TO_MEM) {
		desc->dta = addr;
		desc->dsa = dmac->slave_addr;
		desc->dcm |= DCM_DAI;
	} else {
		desc->dsa = addr;
		desc->dta = dmac->slave_addr;
		desc->dcm |= DCM_SAI;
	}

	rdil = dmac->maxburst * dmac->transfer_width;
	if (rdil > 4) {
		rdil = min((int)fls(rdil) + 1, (int)DCM_RDIL_MAX);
	}

	WARN_ON(length & (~DTC_TC_MSK));
	if (WARN_ON(!IS_ALIGNED(length, dmac->transfer_width))) {
		desc->dtc = ALIGN_DOWN((length & DTC_TC_MSK), dmac->transfer_width);
	} else {
		desc->dtc = (length & DTC_TC_MSK);
	}
	desc->dcm |= DCM_TSZ_MSK | (rdil << DCM_RDIL_SFT);

	return desc->dtc;
}

static void ingenic_dma_free_swdesc(struct virt_dma_desc *vd)
{
	struct ingenic_dma_sdesc *sdesc = to_ingenic_dma_sdesc(vd);
	struct ingenic_dma_chan *dmac = sdesc->dmac;
	unsigned long flags;
	int i;

	WARN_ON(!sdesc->dmac);

	spin_lock_irqsave(&dmac->hdesc_lock, flags);
	if (dmac->hdesc_pool) {
		for (i = 0; i < sdesc->nb_desc; i++) {
			dma_pool_free(dmac->hdesc_pool, sdesc->hw_desc[i], sdesc->hw_desc_dma[i]);
		}
		dmac->hdesc_num -= sdesc->nb_desc;
	}
	spin_unlock_irqrestore(&dmac->hdesc_lock, flags);

	kfree(sdesc);
	return;
}

static struct ingenic_dma_sdesc *ingenic_dma_alloc_swdesc(struct ingenic_dma_chan *dmac, int num_hdesc)
{
	struct ingenic_dma_sdesc *sdesc;
	int i;
	unsigned long flags;

	spin_lock_irqsave(&dmac->hdesc_lock, flags);
	if (num_hdesc > (dmac->hdesc_max - dmac->hdesc_num) || !dmac->hdesc_pool) {
		spin_unlock_irqrestore(&dmac->hdesc_lock, flags);
		return NULL;
	}

	sdesc = (struct ingenic_dma_sdesc *)kzalloc(sizeof(struct ingenic_dma_sdesc) +
	        num_hdesc * (sizeof(void **) + sizeof(dma_addr_t)),
	        GFP_NOWAIT);
	if (!sdesc) {
		spin_unlock_irqrestore(&dmac->hdesc_lock, flags);
		return NULL;
	}

	sdesc->hw_desc = (struct hdma_desc **)(sdesc + 1);
	sdesc->hw_desc_dma = (dma_addr_t *)(sdesc->hw_desc + num_hdesc);
	sdesc->dmac = dmac;

	for (i = 0; i < num_hdesc; i++) {
		sdesc->hw_desc[i] = dma_pool_alloc(dmac->hdesc_pool, GFP_NOWAIT, &sdesc->hw_desc_dma[i]);
		pr_debug("sdesc->hw_desc[%d] = %p, sdesc->hw_desc_dma[%d] = 0x%08x\n",
		         i, sdesc->hw_desc[i],
		         i, sdesc->hw_desc_dma[i]);
		if (!sdesc->hw_desc[i]) {
			dev_err(&dmac->vc.chan.dev->device,
			        "%s(): Couldn't allocate the hw_desc from dma_pool %p\n",
			        __func__, dmac->hdesc_pool);
			spin_unlock_irqrestore(&dmac->hdesc_lock, flags);
			goto err;
		}
		sdesc->nb_desc++;
	}
	dmac->hdesc_num += sdesc->nb_desc;
	spin_unlock_irqrestore(&dmac->hdesc_lock, flags);
	return sdesc;
err:
	ingenic_dma_free_swdesc(&sdesc->vd);
	return NULL;
}

#ifdef CONFIG_INGENIC_DMA_RECEIVE_DXTERNAL_DECIDE
static void ingenic_dma_use_external_trigger(struct dma_chan *chan,
        struct ingenic_dma_sdesc *sdesc, unsigned int desc_len)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
	int i;
	unsigned long des_offset = 0;

	if (dmac->engine->dma_type == AHB2_PDMA) {
		for (i = 0; i < desc_len; i++) {
			if (sdesc->hw_desc[i]->drt == INGENIC_DMA_REQ_CAN0_TX ||
			    sdesc->hw_desc[i]->drt == INGENIC_DMA_REQ_CAN1_TX) {
				sdesc->hw_desc[i]->dcm = DCM_SAI | DCM_DAI;
				des_offset = sdesc->hw_desc[i]->dtc & (~DTC_TC_MSK);
				sdesc->hw_desc[i]->dtc = sdesc->hw_desc[i]->dtc - des_offset;
				sdesc->hw_desc[i]->dtc /= 4;
				sdesc->hw_desc[i]->dtc += des_offset;
			} else if (sdesc->hw_desc[i]->drt == INGENIC_DMA_REQ_CAN0_RX ||
			           sdesc->hw_desc[i]->drt == INGENIC_DMA_REQ_CAN1_RX) {
				des_offset = sdesc->hw_desc[i]->dtc & (~DTC_TC_MSK);
				sdesc->hw_desc[i]->dtc = sdesc->hw_desc[i]->dtc - des_offset;
				sdesc->hw_desc[i]->dtc /= 16;
				sdesc->hw_desc[i]->dtc += des_offset;
				sdesc->hw_desc[i]->dcm = DCM_DAI | DCM_SAIW | DCM_TSZ_EXTR;
				writel(JZCAN_RX_DATA_OFFSET, dmac->engine->iomem + DSN0);
			} else if ((sdesc->hw_desc[i]->drt == INGENIC_DMA_REQ_UART0_RX) ||
			           (sdesc->hw_desc[i]->drt == INGENIC_DMA_REQ_UART1_RX) ||
			           (sdesc->hw_desc[i]->drt == INGENIC_DMA_REQ_UART2_RX) ||
			           (sdesc->hw_desc[i]->drt == INGENIC_DMA_REQ_UART3_RX)) {
				sdesc->hw_desc[i]->dcm &= ~DCM_RDIL_MSK;
				sdesc->hw_desc[i]->dcm &= ~DCM_TSZ_MSK;
				sdesc->hw_desc[i]->dcm |= DCM_DAI | DCM_TSZ_EXTR;
			} else {
				/* add other need external trigger controller*/
			}

			if (i) {
				sdesc->hw_desc[i - 1]->dcm |= DCM_LINK;
			}
		}
		sdesc->hw_desc[i - 1]->dcm |= DCM_TIE;
	} else if (dmac->engine->dma_type == AHB_MCU_PDMA) {
#if defined(CONFIG_SOC_X2600) || defined(CONFIG_SOC_AD100)
		for (i = 0; i < desc_len; i++) {
			if ((sdesc->hw_desc[i]->drt == INGENIC_DMA_REQ_SADC_SEQ1_RX) || (sdesc->hw_desc[i]->drt == INGENIC_DMA_REQ_SADC_SEQ2_RX)) {
				sdesc->hw_desc[i]->dcm &= ~DCM_TSZ_MSK;
				sdesc->hw_desc[i]->dcm &= ~DCM_DAI;
				sdesc->hw_desc[i]->dcm &= ~DCM_PORT_MSK;

				sdesc->hw_desc[i]->dcm |= DCM_TSZ_EXTR;
				sdesc->hw_desc[i]->dcm |= DCM_DAI;
				sdesc->hw_desc[i]->dcm |= DCM_PORT_32;

			}

			else {
				/* add other need external trigger controller*/
			}

			if (i) {
				sdesc->hw_desc[i - 1]->dcm |= DCM_LINK;
			}
		}
		sdesc->hw_desc[i - 1]->dcm |= DCM_TIE;
#endif
	}
}
#endif

static struct dma_async_tx_descriptor *ingenic_dma_prep_slave_sg(
    struct dma_chan *chan, struct scatterlist *sgl,
    unsigned int sg_len, enum dma_transfer_direction direction,
    unsigned long flags, void *context)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
	struct ingenic_dma_sdesc *sdesc;
	int i;

	sdesc = ingenic_dma_alloc_swdesc(dmac, sg_len);
	if (!sdesc) {
		return NULL;
	}

	for (i = 0; i < sg_len; i++) {
		sdesc->len += build_one_slave_desc(dmac, sg_dma_address(&sgl[i]),
		                                   sg_dma_len(&sgl[i]), direction, sdesc->hw_desc[i]);
		if (i) {
			sdesc->hw_desc[i - 1]->dcm |= DCM_LINK;
			sdesc->hw_desc[i - 1]->dtc |= PHY_TO_DESC_DOA(sdesc->hw_desc_dma[i]);
		}
	}
	sdesc->hw_desc[i - 1]->dcm |= DCM_TIE;

#ifdef CONFIG_INGENIC_DMA_RECEIVE_DXTERNAL_DECIDE
	ingenic_dma_use_external_trigger(chan, sdesc, sg_len);
#endif
	/* use 8-word descriptors */
	sdesc->dcs = DCS_DES8;

	return vchan_tx_prep(&dmac->vc, &sdesc->vd, flags);
}

static struct dma_async_tx_descriptor *ingenic_dma_prep_dma_cyclic(
    struct dma_chan *chan, dma_addr_t buf_addr, size_t buf_len,
    size_t period_len, enum dma_transfer_direction direction,
    unsigned long flags)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
	unsigned int periods = buf_len / period_len;
	struct ingenic_dma_sdesc *sdesc;
	int i;

	sdesc = ingenic_dma_alloc_swdesc(dmac, periods);
	if (!sdesc) {
		return NULL;
	}

	for (i = 0; i < periods; i++) {
		sdesc->len += build_one_slave_desc(dmac, buf_addr + (i * period_len),
		                                   period_len, direction, sdesc->hw_desc[i]);
		if (i) {
			sdesc->hw_desc[i - 1]->dcm |= DCM_LINK | DCM_TIE;
			sdesc->hw_desc[i - 1]->dtc |= PHY_TO_DESC_DOA(sdesc->hw_desc_dma[i]);
		}
	}

#ifdef CONFIG_INGENIC_DMA_RECEIVE_DXTERNAL_DECIDE
	ingenic_dma_use_external_trigger(chan, sdesc, periods);
#endif
	/*make it cyclic*/
	sdesc->hw_desc[i - 1]->dcm |= DCM_LINK | DCM_TIE;
	sdesc->hw_desc[i - 1]->dtc |= PHY_TO_DESC_DOA(sdesc->hw_desc_dma[0]);
	sdesc->cyclic = true;

	/* use 8-word descriptors */
	sdesc->dcs = DCS_DES8;

	return vchan_tx_prep(&dmac->vc, &sdesc->vd, flags);
}

static struct dma_async_tx_descriptor *ingenic_dma_prep_dma_memcpy(struct dma_chan *chan,
        dma_addr_t dma_dest,
        dma_addr_t dma_src,
        size_t len,
        unsigned long flags)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
	struct ingenic_dma_sdesc *sdesc;

	sdesc = ingenic_dma_alloc_swdesc(dmac, 1);
	if (!sdesc) {
		return NULL;
	}

	sdesc->len = build_one_desc(dma_src, dma_dest, len, sdesc->hw_desc[0]);
	sdesc->hw_desc[0]->dcm |= DCM_TIE;

	/* use 8-word descriptors */
	sdesc->dcs = DCS_DES8;

	return vchan_tx_prep(&dmac->vc, &sdesc->vd, flags);
}

#if 0
static int build_dma_sg_desc(struct ingenic_dma_sdesc *sdesc,
                             struct scatterlist *dst_sg, unsigned int dst_nents,
                             struct scatterlist *src_sg, unsigned int src_nents)
{
	int src_sg_avail = 0;
	int dst_sg_avail = 0;
	int size;
	dma_addr_t src_addr;
	dma_addr_t dst_addr;
	int i = 0;

	src_nents--;
	dst_nents--;
	dst_sg_avail = sg_dma_len(src_sg);
	src_sg_avail = sg_dma_len(dst_sg);
	while (true) {
		src_addr = sg_dma_address(src_sg) + sg_dma_len(src_sg) - src_sg_avail;
		dst_addr = sg_dma_address(dst_sg) + sg_dma_len(dst_sg) - dst_sg_avail;

		size = min(src_sg_avail, dst_sg_avail);
		size = min(size, (DTC_TC_MSK & (~0x3)));

		if (NULL != sdesc) {
			sdesc->len += build_one_desc(src_addr, dst_addr, size, sdesc->hw_desc[i]);
			if (i) {
				sdesc->hw_desc[i - 1]->dcm |= DCM_LINK;
				sdesc->hw_desc[i - 1]->dtc |= PHY_TO_DESC_DOA(sdesc->hw_desc_dma[i]);
			}
		}

		dst_sg_avail -= size;
		src_sg_avail -= size;

		if (0 == src_sg_avail) {
			if (0 == src_nents) {
				break;
			}
			src_sg = sg_next(src_sg);
			src_sg_avail = sg_dma_len(src_sg);
			src_nents--;
		}

		if (0 == dst_sg_avail) {
			if (0 == dst_nents) {
				break;
			}
			dst_sg = sg_next(dst_sg);
			dst_sg_avail = sg_dma_len(dst_sg);
			dst_nents--;
		}
		i++;
	}
	if (sdesc) {
		sdesc->hw_desc[i]->dcm |= DCM_TIE;
	}
	return ++i;
}

static struct dma_async_tx_descriptor *ingenic_dma_prep_dma_sg(
    struct dma_chan *chan,
    struct scatterlist *dst_sg, unsigned int dst_nents,
    struct scatterlist *src_sg, unsigned int src_nents,
    unsigned long flags)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
	struct ingenic_dma_sdesc *sdesc;
	int hw_nums;

	if (0 == dst_nents || 0 == src_nents) {
		return NULL;
	}

	if (NULL == dst_sg || NULL == src_sg) {
		return NULL;
	}

	hw_nums = build_dma_sg_desc(NULL, dst_sg, dst_nents, src_sg, src_nents);

	sdesc = ingenic_dma_alloc_swdesc(dmac, hw_nums);
	if (!sdesc) {
		return NULL;
	}

	build_dma_sg_desc(sdesc, dst_sg, dst_nents, src_sg, src_nents);

	/* use 8-word descriptors */
	sdesc->dcs = DCS_DES8;

	return vchan_tx_prep(&dmac->vc, &sdesc->vd, flags);
}
#endif

static size_t ingenic_dma_desc_residue(struct ingenic_dma_chan *dmac, struct ingenic_dma_sdesc *sdesc)
{
	unsigned int residue = 0, shift, pass = 0;
	unsigned int i;
	bool dsa = false;
	dma_addr_t start, end, compare;

	if (sdesc->hw_desc[0]->dcm & DCM_SAI) {
		compare = readl(dmac->iomem + CH_DSA);
		dsa = true;
	} else {
		compare = readl(dmac->iomem + CH_DTA);
	}

	for (i = 0; i < sdesc->nb_desc; i++) {
		start = dsa ? sdesc->hw_desc[i]->dsa : sdesc->hw_desc[i]->dta;
		shift = get_current_tsz(sdesc->hw_desc[i]->dcm);
		end = start + (sdesc->hw_desc[i]->dtc << shift);
		if (start <= compare && end > compare) {
			pass += (compare - start);
			break;
		} else {
			pass += sdesc->hw_desc[i]->dtc << shift;
		}
	}

	residue = sdesc->len - pass;
	return residue;
}

static enum dma_status ingenic_dma_tx_status(struct dma_chan *chan,
        dma_cookie_t cookie,
        struct dma_tx_state *txstate)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
	struct virt_dma_desc *vd;
	enum dma_status status;
	unsigned long flags;

	status = dma_cookie_status(chan, cookie, txstate);
	if ((status == DMA_COMPLETE) || (txstate == NULL)) {
		return status;
	}

	spin_lock_irqsave(&dmac->vc.lock, flags);

	vd = vchan_find_desc(&dmac->vc, cookie);
	if (vd) {
		dma_set_residue(txstate, to_ingenic_dma_sdesc(vd)->len);
	} else if (dmac->sdesc && cookie == dmac->sdesc->vd.tx.cookie) {
		dma_set_residue(txstate, ingenic_dma_desc_residue(dmac, dmac->sdesc));
	} else {
		dma_set_residue(txstate, 0);
	}

	spin_unlock_irqrestore(&dmac->vc.lock, flags);
	return status;
}

static void ingenic_dma_start_trans(struct ingenic_dma_chan *dmac)
{
	static struct virt_dma_desc *vd = NULL;
	struct ingenic_dma_sdesc *sdesc;
	int i;

	if (!dmac->sdesc) {
		vd = vchan_next_desc(&dmac->vc);
		if (!vd) {
			return;
		}

		list_del(&vd->node);

		sdesc = dmac->sdesc = to_ingenic_dma_sdesc(vd);

		if (dmac->fake_cyclic && sdesc->cyclic && vd->tx.callback) {
			/*
			 * The DMA controller doesn't support triggering an interrupt
			 * after processing each descriptor, only after processing an
			 * entire terminated list of descriptors.For a cyclic DMA
			 * setup the list of descriptors is not terminated so we can
			 * never get an interrupt.
			 *
			 * If the user requested a callback for a cyclic DMA setup then
			 * we workaround this hardware limitation here by degrading to
			 * a set of unlinked descriptors which we will submit in
			 * sequence in response to the completion of processing the
			 * previous descriptor
			 */
			for (i = 0; i < sdesc->nb_desc; i++) {
				sdesc->hw_desc[i]->dcm &= ~DCM_LINK;
			}
		}

		for (i = 0; i < sdesc->nb_desc; i++) {
			int j;
			uint32_t *vaddr = (void *)sdesc->hw_desc[i];
			for (j = 0; j < 8; j++) {
				pr_debug("<%d>: &vaddr[0] %p vaddr[0] %x\n", i, &vaddr[j], vaddr[j]);
			}
			pr_debug("sdesc->hw_desc_dma[0] 0x%08x dmac->iomem %p\n", sdesc->hw_desc_dma[i], dmac->iomem);
		}

		sdesc->curr_desc = 0;
		sdesc->status = STAT_RUNNING;
	} else {
		sdesc = dmac->sdesc;
		WARN_ON_ONCE((!sdesc->cyclic));
		WARN_ON_ONCE((!vd->tx.callback));
		WARN_ON_ONCE((!dmac->fake_cyclic));
		sdesc->status = STAT_RUNNING;
		sdesc->curr_desc++;
		sdesc->curr_desc = sdesc->curr_desc % sdesc->nb_desc;
	}

#ifdef DUMP_DMA_DESC
	dump_dma_hdesc(sdesc->hw_desc[sdesc->curr_desc], __func__);
#endif
	/* dma descriptor address */
	writel(sdesc->hw_desc_dma[sdesc->curr_desc], dmac->iomem + CH_DDA);
	/* initiate descriptor fetch */
	writel(BIT(dmac->id), dmac->engine->iomem + DDRS);
	/* transfer start */
	dev_dbg(chan2dev(&dmac->vc.chan), "dcs:%x start transfer\n",
	        readl(dmac->iomem + CH_DCS));

	writel(sdesc->dcs | DCS_CTE, dmac->iomem + CH_DCS);
	return;
}

static void ingenic_dma_issue_pending(struct dma_chan *chan)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&dmac->vc.lock, flags);

	if (vchan_issue_pending(&dmac->vc) && !dmac->sdesc) {
		ingenic_dma_start_trans(dmac);
	}

	spin_unlock_irqrestore(&dmac->vc.lock, flags);
}

#if 0
/*
 *  get dma current transfer address
 */
static dma_addr_t jzdma_get_current_trans_addr(struct dma_chan *chan,
        dma_addr_t *dst_addr,
        dma_addr_t *src_addr,
        enum dma_transfer_direction
        direction) {
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
	dma_addr_t ret_val = 0;

	if (!dmac->sdesc)
	{
		return 0;
	}
	if (dmac->sdesc->status == STAT_STOPPED)
	{
		return 0;
	}

	if (direction ==  DMA_MEM_TO_DEV)
	{
		ret_val = readl(dmac->iomem + CH_DSA);
		if (src_addr) {
			*src_addr = ret_val;
		}
		if (dst_addr) {
			*dst_addr = readl(dmac->iomem + CH_DTA);
		}
	} else if (direction == DMA_DEV_TO_MEM)
	{
		ret_val = readl(dmac->iomem + CH_DTA);
		if (dst_addr) {
			*dst_addr = ret_val;
		}
		if (src_addr) {
			*src_addr = readl(dmac->iomem + CH_DSA);
		}
	} else if (direction == DMA_MEM_TO_MEM)
	{
		if (dst_addr) {
			*dst_addr = readl(dmac->iomem + CH_DTA);
		}
		if (src_addr) {
			*src_addr = readl(dmac->iomem + CH_DSA);
		}
	}

	return ret_val;
}
#endif

static int ingenic_dma_terminate_all(struct dma_chan *chan)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
	unsigned long flags;
	int ret = 0;
	LIST_HEAD(head);

	spin_lock_irqsave(&dmac->vc.lock, flags);
#ifdef AIC_USES_PDMA
	/* the part handler audio driver */
#ifdef CONFIG_SOC_X1000
	if (dmac->slave_id == INGENIC_DMA_REQ_AIC_TX || dmac->slave_id == INGENIC_DMA_REQ_AIC_RX
#else
	if (dmac->slave_id == INGENIC_DMA_REQ_AIC_LOOP_RX || dmac->slave_id == INGENIC_DMA_REQ_AIC_TX || dmac->slave_id == INGENIC_DMA_REQ_AIC_RX
#endif
#ifdef CONFIG_SND_ASOC_INGENIC_DMIC
	    || dmac->slave_id == INGENIC_DMA_REQ_DMIC_RX
#endif
	    || dmac->slave_id == INGENIC_DMA_REQ_PCM_TX || dmac->slave_id == INGENIC_DMA_REQ_PCM_RX) {
		if (dmac->sdesc) {  /*DMA transfer is running*/
			if (dmac->sdesc->status != STAT_STOPPED) {
				dmac->sdesc->status = STAT_STOPPED;
				writel(0, dmac->iomem + CH_DCS);
			}
			ingenic_dma_free_swdesc(&dmac->sdesc->vd);
			dmac->sdesc = NULL;
		}

		//      printk("%s[%d]: pdma%d\n",__func__,__LINE__,dmac->slave_id);
		vchan_get_all_descriptors(&dmac->vc, &head);

		spin_unlock_irqrestore(&dmac->vc.lock, flags);

		vchan_dma_desc_free_list(&dmac->vc, &head);

		return ret;
	}
#endif
	if (dmac->sdesc) {  /*DMA transfer is running*/
		int i;
		ret = -EBUSY;
		if (dmac->sdesc->status != STAT_STOPPED) {
			dmac->sdesc->status = STAT_STOPPED;
			reinit_completion(&dmac->completion);

			for (i = 0; i < dmac->sdesc->nb_desc; i++) {
				dmac->sdesc->hw_desc[i]->dcm |= DCM_TIE;
				dmac->sdesc->hw_desc[i]->dcm &= ~DCM_LINK;
			}

			if (HWATTR_DESC_INTER_SUP(dmac->engine->hwattr)) {
				/*
				 * The version of controller support descriptor interrupt
				 * can clear LINK on runtime
				 */
				unsigned int dcm = readl(dmac->iomem + CH_DCM);
				if (dcm & DCM_LINK) {
					dcm &= ~DCM_LINK;
					writel(dcm, dmac->iomem + CH_DCM);
				}

				if (dmac->sdesc->cyclic) {
					/* direct disable dma transfer. */
					writel(0, dmac->iomem + CH_DCS);
					ingenic_dma_free_swdesc(&dmac->sdesc->vd);
					dmac->sdesc = NULL;
				}
			}
		} else if (readl(dmac->iomem + CH_DRT) != INGENIC_DMA_REQ_AUTO_TX) {
			writel(0, dmac->iomem + CH_DCS);
			ingenic_dma_free_swdesc(&dmac->sdesc->vd);
			dmac->sdesc->status = STAT_STOPPED;
			dmac->sdesc = NULL;
			complete(&dmac->completion);
			ret = 0; /*DMA transfer force stop !!!!!*/
		}
	} else {    /*DMA transfer already stoped*/
		writel(0, dmac->iomem + CH_DCS);
	}

	vchan_get_all_descriptors(&dmac->vc, &head);

	spin_unlock_irqrestore(&dmac->vc.lock, flags);

	vchan_dma_desc_free_list(&dmac->vc, &head);

	return ret;
}

static int ingenic_dma_wait_terminate_complete(struct dma_chan *chan)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);

	if (dmac->sdesc) {
		wait_for_completion(&dmac->completion);
	}
	return 0;
}

static int ingenic_dma_config(struct dma_chan *chan, struct dma_slave_config *config)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
	enum dma_slave_buswidth transfer_width;

	if (!config) {
		return -EINVAL;
	}

	switch (config->direction) {
	case DMA_MEM_TO_DEV:
		if (!config->dst_addr_width || !config->dst_addr) {
			return -EINVAL;
		}
		if (!config->dst_maxburst) {
			config->dst_maxburst = 1;
		}
		transfer_width = config->dst_addr_width;
		dmac->slave_addr = config->dst_addr;
		dmac->maxburst = config->dst_maxburst;
		break;
	case DMA_DEV_TO_MEM:
		if (!config->src_addr_width || !config->src_addr) {
			return -EINVAL;
		}
		if (!config->src_maxburst) {
			config->src_maxburst = 1;
		}
		transfer_width = config->src_addr_width;
		dmac->slave_addr = config->src_addr;
		dmac->maxburst = config->src_maxburst;
		break;
	default:
		return -EINVAL;
	}

	switch (transfer_width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		dmac->dcm = DCM_PORT_8;
		dmac->transfer_width = 1;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		dmac->dcm = DCM_PORT_16;
		dmac->transfer_width = 2;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		dmac->dcm = DCM_PORT_32;
		dmac->transfer_width = 4;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void pdma_handle_chan_irq(struct ingenic_dma_engine *ingenic_dma, int ch_id)
{
	struct ingenic_dma_chan *dmac = ingenic_dma->chan[ch_id];
	struct ingenic_dma_sdesc *sdesc;
	unsigned int dcs;

	spin_lock(&dmac->vc.lock);

	dcs = readl(dmac->iomem + CH_DCS);
	writel(0, dmac->iomem + CH_DCS);

	if (dcs & DCS_AR)
		dev_warn(&dmac->vc.chan.dev->device,
		         "address error (DCS=0x%x)\n", dcs);

	if (dcs & DCS_HLT)
		dev_warn(&dmac->vc.chan.dev->device,
		         "channel halt (DCS=0x%x)\n", dcs);
	sdesc = dmac->sdesc;
	if (sdesc) {
		if (sdesc->status == STAT_STOPPED) {
			dma_cookie_complete(&sdesc->vd.tx);
			dmac->sdesc = NULL;
			ingenic_dma_free_swdesc(&sdesc->vd);
			complete(&dmac->completion);
		} else if (dmac->fake_cyclic && sdesc->cyclic) {
			vchan_cyclic_callback(&sdesc->vd);
		} else {
			vchan_cookie_complete(&sdesc->vd);
			dmac->sdesc = NULL;
		}
		ingenic_dma_start_trans(dmac);
	} else {
		dev_warn(&dmac->vc.chan.dev->device,
		         "channel irq with no active transfer, channel stop\n");
	}

	spin_unlock(&dmac->vc.lock);
}

static irqreturn_t pdma_int_handler(int irq, void *dev)
{
	struct ingenic_dma_engine *ingenic_dma = (struct ingenic_dma_engine *)dev;
	unsigned long pending, dmac;
	int i;

	pending = readl(ingenic_dma->iomem + DIRQP);
	writel(~pending, ingenic_dma->iomem + DIRQP);

	for (i = 0; i < ingenic_dma->nr_chs ; i++) {
		if (!(pending & (1 << i))) {
			continue;
		}
		pdma_handle_chan_irq(ingenic_dma, i);
	}

	dmac = readl(ingenic_dma->iomem + DMAC);
	dmac &= ~(DMAC_HLT | DMAC_AR);
	writel(dmac, ingenic_dma->iomem + DMAC);
	return IRQ_HANDLED;
}

static irqreturn_t pdmam_int_handler(int irq, void *dev)
{
	/*TODO*/
	return IRQ_HANDLED;
}

static irqreturn_t pdmad_int_handler(int irq, void *dev)
{
	struct ingenic_dma_engine *ingenic_dma = (struct ingenic_dma_engine *)dev;
	unsigned long pending;
	int i;

	pending = readl(ingenic_dma->iomem + DIP);
	writel(readl(ingenic_dma->iomem + DIP) & (~pending), ingenic_dma->iomem + DIC);

	for (i = 0; i < ingenic_dma->nr_chs; i++) {
		struct ingenic_dma_chan *dmac = ingenic_dma->chan[i];
		struct ingenic_dma_sdesc *sdesc;

		if (!(pending & (1 << i))) {
			continue;
		}
		sdesc = dmac->sdesc;
		if (sdesc && sdesc->cyclic) {
			spin_lock(&dmac->vc.lock);
			vchan_cyclic_callback(&sdesc->vd);
			spin_unlock(&dmac->vc.lock);
		}
	}
	return IRQ_HANDLED;

}

static int ingenic_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);

	dmac->hdesc_pool = dma_pool_create(dev_name(&chan->dev->device),
	                                   chan->device->dev,
	                                   sizeof(struct hdma_desc),
	                                   0, PAGE_SIZE);
	if (!dmac->hdesc_pool) {
		dev_err(&chan->dev->device,
		        "failed to allocate descriptor pool\n");
		return -ENOMEM;
	}
	dmac->hdesc_max = PAGE_SIZE / sizeof(struct hdma_desc);
	dmac->hdesc_num = 0;
	return 0;
}

static void ingenic_dma_free_chan_resources(struct dma_chan *chan)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
	unsigned long flags;

	ingenic_dma_terminate_all(chan);

	ingenic_dma_wait_terminate_complete(chan);
	dma_pool_destroy(dmac->hdesc_pool);

	spin_lock_irqsave(&dmac->hdesc_lock, flags);
	dmac->hdesc_pool = NULL;
	dmac->hdesc_max = 0;
	dmac->hdesc_num = 0;
	spin_unlock_irqrestore(&dmac->hdesc_lock, flags);
}

static bool ingenic_dma_filter_fn(struct dma_chan *chan, void *param)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
	unsigned int private  = *(unsigned int *)param;
	int channel = -1;
	int bus = 0;

	bus = private & INGENIC_DMA_CTRL_BUS_MSK;
	if (INGENIC_GET_CTRL_BUS(bus) != dmac->engine->dma_type) {
		return false;
	}

	if (private & INGENIC_DMA_TYPE_CH_EN) {
		channel = (private & INGENIC_DMA_TYPE_CH_MSK) >> INGENIC_DMA_TYPE_CH_SFT;
		if (dmac->id == channel) {
			return true;
		}
		return false;
	}

	if (dmac->engine->chan_reserved & BIT(dmac->id)) {
		return false;
	}

	dmac->slave_id = private & INGENIC_DMA_TYPE_REQ_MSK;
	return true;
}

static struct of_dma_filter_info of_ingenic_dma_info = {
	.filter_fn = ingenic_dma_filter_fn,
};

static int ingenic_dma_chan_init(struct ingenic_dma_engine *dma, int id)
{
	struct ingenic_dma_chan *dmac = NULL;

	dmac = devm_kzalloc(dma->dev, sizeof(*dmac), GFP_KERNEL);
	if (!dmac) {
		return -ENOMEM;
	}
	dmac->id = id;
	dmac->iomem = dma->iomem + dmac->id * DMACH_OFF;
	dmac->engine = dma;
	dmac->fake_cyclic = HWATTR_DESC_INTER_SUP(dma->hwattr) ? false : true;
	spin_lock_init(&dmac->hdesc_lock);
	init_completion(&dmac->completion);

	vchan_init(&dmac->vc, &dma->dma_device);

	dmac->vc.desc_free = ingenic_dma_free_swdesc;
#ifdef CONFIG_SOC_X2500
	dmac->vc.chan.private = (void *)pdma_maps[id];
	dmac->slave_id = pdma_maps[id] & INGENIC_DMA_TYPE_REQ_MSK;
#endif
	dma->chan[id] = dmac;
	return 0;
}

static int __init ingenic_dma_probe(struct platform_device *pdev)
{
	struct ingenic_dma_engine *dma = NULL;
	struct resource *iores;
	u32 reg_dmac = DMAC_DMAE;
	int i, ret = 0;

	/* check of first. if of failed, use platform */
	dma = ingenic_dma_parse_dt(pdev);
	if (IS_ERR(dma)) {
		return PTR_ERR(dma);
	}

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dma->iomem = devm_ioremap_resource(&pdev->dev, iores);
	if (IS_ERR(dma->iomem)) {
		return PTR_ERR(dma->iomem);
	}

	/* PDMA interrupt*/
	dma->irq_pdma = platform_get_irq_byname(pdev, "pdma");
	if (dma->irq_pdma < 0) {
		return dma->irq_pdma;
	}
	ret = devm_request_irq(&pdev->dev, dma->irq_pdma, pdma_int_handler,
	                       0, "pdma", dma);
	if (ret) {
		return ret;
	}

	/* PDMA mcu interrupt*/
	dma->irq_pdmam = platform_get_irq_byname(pdev, "pdmam");
	if (dma->irq_pdmam >= 0) {
		ret = devm_request_irq(&pdev->dev, dma->irq_pdmam, pdmam_int_handler,
		                       0, "pdmam", dma);
		if (ret) {
			return ret;
		}
	}

	/* PDMA descriptor interrupt */
	if (HWATTR_DESC_INTER_SUP(dma->hwattr)) {
		dma->irq_pdmad = platform_get_irq_byname(pdev, "pdmad");
		if (dma->irq_pdmad < 0) {
			return dma->irq_pdmad;
		}
		ret = devm_request_irq(&pdev->dev, dma->irq_pdmad, pdmad_int_handler,
		                       0, "pdmad", dma);
		if (ret) {
			return ret;
		}
		irq_set_irq_wake(dma->irq_pdmad, 1);
	}

	/* Initialize dma engine */
	INIT_LIST_HEAD(&dma->dma_device.channels);
	for (i = 0; i < dma->nr_chs; i++) {
		/*reserved one channel for intc interrupt*/
		if (dma->intc_ch == i) {
			continue;
		}
		ingenic_dma_chan_init(dma, i);
	}
	dma_cap_set(DMA_MEMCPY, dma->dma_device.cap_mask);
	dma_cap_set(DMA_SLAVE, dma->dma_device.cap_mask);
	dma_cap_set(DMA_CYCLIC, dma->dma_device.cap_mask);

	dma->dma_device.dev = &pdev->dev;
	dma->dma_device.device_alloc_chan_resources = ingenic_dma_alloc_chan_resources;
	dma->dma_device.device_free_chan_resources = ingenic_dma_free_chan_resources;
	dma->dma_device.device_tx_status = ingenic_dma_tx_status;
	dma->dma_device.device_prep_slave_sg = ingenic_dma_prep_slave_sg;
	dma->dma_device.device_prep_dma_cyclic = ingenic_dma_prep_dma_cyclic;
	dma->dma_device.device_prep_dma_memcpy = ingenic_dma_prep_dma_memcpy;
	dma->dma_device.device_config = ingenic_dma_config;
	dma->dma_device.device_terminate_all = ingenic_dma_terminate_all;
	dma->dma_device.device_issue_pending = ingenic_dma_issue_pending;
	dma->dma_device.copy_align = DMAENGINE_ALIGN_4_BYTES;
	dma->dma_device.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) | BIT(DMA_SLAVE_BUSWIDTH_2_BYTES)
	                                  | BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dma->dma_device.dst_addr_widths = dma->dma_device.src_addr_widths;
	dma->dma_device.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV) | BIT(DMA_MEM_TO_MEM);
	dma->dma_device.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	dma->dma_device.dev->dma_parms = &dma->dma_parms;
	dma_set_max_seg_size(dma->dma_device.dev, DTC_TC_MSK);  /*At least*/
	if (dma->dma_type == AHB_MCU_PDMA) {
		dma->gate_clk = devm_clk_get(&pdev->dev, "gate_pdma1");
	} else {
		dma->gate_clk = devm_clk_get(&pdev->dev, "gate_pdma");
	}
	if (IS_ERR(dma->gate_clk)) {
		return PTR_ERR(dma->gate_clk);
	}

	ret = dma_async_device_register(&dma->dma_device);
	if (ret) {
		dev_err(&pdev->dev, "unable to register\n");
		clk_disable(dma->gate_clk);
		return ret;
	}

	of_ingenic_dma_info.dma_cap = dma->dma_device.cap_mask;
	ret = of_dma_controller_register(pdev->dev.of_node,
	                                 of_dma_simple_xlate, &of_ingenic_dma_info);
	if (ret) {
		dev_err(&pdev->dev, "unable to register dma to device tree\n");
		dma_async_device_unregister(&dma->dma_device);
		clk_disable(dma->gate_clk);
		return ret;
	}
	platform_set_drvdata(pdev, dma);

	/*enable pdma controller*/
	clk_prepare_enable(dma->gate_clk);

	if (dma->chan_programed) {
		writel(dma->chan_programed, dma->iomem + DMACP);
	}
	if (dma->intc_ch >= 0) {
		reg_dmac |= DMAC_INTCE | ((dma->intc_ch << DMAC_INTCC_SFT) & DMAC_INTCC_MSK);
	}
	if (dma->special_ch) {
		reg_dmac |= DMAC_CH01;
	}
	writel(reg_dmac, dma->iomem + DMAC);
	dev_info(dma->dev, "INGENIC SoC DMA initialized\n");
	return 0;
}

static int ingenic_dma_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ingenic_dma_engine *ingenic_dma = platform_get_drvdata(pdev);
	struct dma_chan *chan;

	list_for_each_entry(chan, &ingenic_dma->dma_device.channels, device_node) {
		struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(chan);
		struct ingenic_dma_sdesc *sdesc = dmac->sdesc;

		if (sdesc) {
			switch (sdesc->hw_desc[0]->drt) {
			case INGENIC_DMA_REQ_UART0_RX:
			case INGENIC_DMA_REQ_UART1_RX:
			case INGENIC_DMA_REQ_UART2_RX:
#ifndef CONFIG_SOC_X1000
			case INGENIC_DMA_REQ_UART3_RX:
#endif
#ifdef CONFIG_INGENIC_CAN
			case INGENIC_DMA_REQ_CAN0_RX:
			case INGENIC_DMA_REQ_CAN1_RX:
#endif
				break;
			default:
				return -EBUSY;
			}
		}
	}
	clk_disable_unprepare(ingenic_dma->gate_clk);
	return 0;
}

static int ingenic_dma_resume(struct platform_device *pdev)
{
	struct ingenic_dma_engine *ingenic_dma = platform_get_drvdata(pdev);
	clk_prepare_enable(ingenic_dma->gate_clk);
	return 0;
}

static const struct of_device_id ingenic_dma_dt_match[] = {
	{ .compatible = "ingenic,m200-pdma", .data = (void *)(HWATTR_SPECIAL_CH01 | HWATTR_INTC_IRQ)},
	{ .compatible = "ingenic,x1000-pdma", .data = (void *)HWATTR_DESC_INTER},
	{ .compatible = "ingenic,x1021-pdma", .data = (void *)HWATTR_DESC_INTER},
	{ .compatible = "ingenic,x1520-pdma", .data = (void *)HWATTR_DESC_INTER},
	{ .compatible = "ingenic,x1630-pdma", .data = (void *)HWATTR_DESC_INTER},
	{ .compatible = "ingenic,x2000-pdma", .data = (void *)HWATTR_DESC_INTER},
	{ .compatible = "ingenic,x2500-pdma", .data = (void *)HWATTR_INTC_IRQ},
	{ .compatible = "ingenic,m300-pdma", .data = (void *)HWATTR_DESC_INTER},
	{ .compatible = "ingenic,x1600-pdma", .data = (void *)HWATTR_DESC_INTER},
	{ .compatible = "ingenic,x2600-pdma", .data = (void *)HWATTR_DESC_INTER},
	{ .compatible = "ingenic,ad100-pdma", .data = (void *)HWATTR_DESC_INTER},
	{},
};
MODULE_DEVICE_TABLE(of, ingenic_dma_dt_match);

static struct platform_driver ingenic_dma_driver = {
	.driver = {
		.name = "ingenic-dma",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ingenic_dma_dt_match),
	},
	.suspend = ingenic_dma_suspend,
	.resume = ingenic_dma_resume,
};

static int __init ingenic_dma_module_init(void)
{
	return platform_driver_probe(&ingenic_dma_driver, ingenic_dma_probe);
}
subsys_initcall(ingenic_dma_module_init);
MODULE_AUTHOR("Chen.li <chen.li@ingenic.cn>");
MODULE_DESCRIPTION("Ingenic dma driver");
MODULE_LICENSE("GPL");
