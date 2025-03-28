/**
 *  sound/soc/ingenic/asoc-dmic-v2.c
 *  ALSA Soc Audio Layer -- ingenic dmic (part of aic controller) driver
 *
 *  Copyright 2014 Ingenic Semiconductor Co.,Ltd
 *  cwang <chang.wang@ingenic.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <sound/tlv.h>
#include <sound/control.h>
#include <linux/of_irq.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/vmalloc.h>
#include <linux/lcm.h>
#include "asoc-dmic-v2.h"

#ifdef DEBUG
	static int ingenic_dma_debug = 1;
#else
	static int ingenic_dma_debug = 0;
#endif

#if 0
#ifdef CONFIG_AUDIO_DUMP
static void dump_register(struct ingenic_dmic *dmic)
{
	printk("----- DBA          : %#x\r\n", readl(dmic->io_base + DBA));
	printk("----- DTC          : %#x\r\n", readl(dmic->io_base + DTC));
	printk("----- DCR          : %#x\r\n", readl(dmic->io_base + DCR));
	printk("----- DSR          : %#x\r\n", readl(dmic->io_base + DSR));
	printk("----- DCM          : %#x\r\n", readl(dmic->io_base + DCM));
	printk("----- DDA          : %#x\r\n", readl(dmic->io_base + DDA));
	printk("----- DGRR         : %#x\r\n", readl(dmic->io_base + DGRR));
	printk("----- DGER         : %#x\r\n", readl(dmic->io_base + DGER));
	printk("----- DEIER        : %#x\r\n", readl(dmic->io_base + DEIER));
	printk("----- DEISR        : %#x\r\n", readl(dmic->io_base + DEISR));
	printk("----- DIPR         : %#x\r\n", readl(dmic->io_base + DIPR));
	printk("----- FAS          : %#x\r\n", readl(dmic->io_base + FAS));
	printk("----- FCR          : %#x\r\n", readl(dmic->io_base + FCR));
	printk("----- FFR          : %#x\r\n", readl(dmic->io_base + FFR));
	printk("----- FSR          : %#x\r\n", readl(dmic->io_base + FSR));
	printk("----- DFCR         : %#x\r\n", readl(dmic->io_base + DFCR));
	printk("----- FDR          : %#x\r\n", readl(dmic->io_base + FDR));
	printk("----- BTSET        : %#x\r\n", readl(dmic->io_base + BTSET));
	printk("----- BTCLR        : %#x\r\n", readl(dmic->io_base + BTCLR));
	printk("----- BTSR         : %#x\r\n", readl(dmic->io_base + BTSR));
	printk("----- BFSR         : %#x\r\n", readl(dmic->io_base + BFSR));
	printk("----- BFCR0        : %#x\r\n", readl(dmic->io_base + BFCR0));
	printk("----- BFCR1        : %#x\r\n", readl(dmic->io_base + BFCR1));
	printk("----- BFCR2        : %#x\r\n", readl(dmic->io_base + BFCR2));
	printk("----- BSTR0        : %#x\r\n", readl(dmic->io_base + BSTR0));
	printk("----- BTTR0        : %#x\r\n", readl(dmic->io_base + BTTR0));
	printk("----- DMIC_CR0     : %#x\r\n", readl(dmic->io_base + DMIC_CR0));
	printk("----- DMIC_GCR     : %#x\r\n", readl(dmic->io_base + DMIC_GCR));
	printk("----- DMIC_TRI_IER : %#x\r\n", readl(dmic->io_base + DMIC_TRI_IER));
	printk("----- DMIC_TRI_ICR : %#x\r\n", readl(dmic->io_base + DMIC_TRI_ICR));
	printk("----- DMIC_TCR     : %#x\r\n", readl(dmic->io_base + DMIC_TCR));
}
#endif
#endif

static const struct snd_pcm_hardware ingenic_dmic_dma_pcm_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
	         SNDRV_PCM_INFO_MMAP_VALID),
	.buffer_bytes_max = INGENIC_DMA_BUFFERSIZE_MAX,
	.period_bytes_min = INGENIC_PERIODS_BYTES_MIN,
	.period_bytes_max = (INGENIC_DMA_BUFFERSIZE_MAX / INGENIC_PERIODS_MIN),
	.periods_min = INGENIC_PERIODS_MIN,
	.periods_max = (INGENIC_DMA_BUFFERSIZE_MAX / INGENIC_PERIODS_BYTES_MIN),
	.fifo_size = 0,
	.formats = ~0ULL,
};
static const struct of_device_id ingenic_dmic_match_table[];

static irqreturn_t ingenic_dmic_dma_irq_handler(int irq, void *dev_id)
{
	struct ingenic_dmic *dmic = (void *)dev_id;
	struct ingenic_dmic_dma_runtime *prtd = NULL;
	struct snd_pcm_substream *substream = NULL;
	unsigned int dma_status, fifo_status, pending;

	regmap_read(dmic->regmap, DIPR, &pending);
	dev_dbg(dmic->dev, "interrupt (irq%d) pending %#x\n", irq, pending);

	if (pending & DIPR_DMA_INT) {
		regmap_read(dmic->regmap, DSR, &dma_status);
		regmap_write(dmic->regmap, DSR, dma_status);

		if (dma_status & DSR_TT_INT) {
			dev_dbg(dmic->dev, "dma channel pending\n");

			spin_lock(&dmic->dma->lock);
			substream = NULL;
			prtd = dmic->dma->rtd;
			if (likely(prtd)) {
				substream = prtd->substream;
				spin_unlock(&dmic->dma->lock);
				if (prtd->dummy_data_flag && prtd->dummy_data_irq) {
					prtd->dummy_data_irq = 0;
				} else {
					prtd->hw_ptr = readl_relaxed(dmic->io_base + DBA);
					snd_pcm_period_elapsed(substream);
				}
			} else {
				spin_unlock(&dmic->dma->lock);
				dev_warn(dmic->dev, "dma channel stream has been stopped\n");
			}
		}
	}

	if (IS_BUILTIN(CONFIG_FPGA_TEST) || ingenic_dma_debug) {
		if (pending & DIPR_DMA_INT) {           /*handle ror/tur debug*/
			regmap_read(dmic->regmap, FSR, &fifo_status);
			regmap_write(dmic->regmap, FSR, fifo_status);
			if (fifo_status & FSR_TURROR_CC) {
				dev_warn(dmic->dev, "Fifo xrun\n");
			}
		}
	}

	return IRQ_HANDLED;
}

static inline int ingenic_dmic_dma_get_tsz(int fifo_depth,
        int period_sz, int *ts_byte_sz)
{
	int tsz_reg = 0, ts_word_sz = 1;

	if (period_sz & ((4 << 2) - 1)) {
		*ts_byte_sz = ts_word_sz << 2;
		return tsz_reg;
	}

	period_sz >>= 2;        /* word size */

	period_sz >>= 2;
	ts_word_sz <<= 2;
	tsz_reg++;

	while ((!(period_sz & 0x1)) && (ts_word_sz << 1) <= fifo_depth) {
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

static bool ingenic_fmt_need_packen(snd_pcm_format_t format)
{
	switch (format) {
		case SNDRV_PCM_FORMAT_S24_3LE:
		case SNDRV_PCM_FORMAT_U24_3LE:
		case SNDRV_PCM_FORMAT_S20_3LE:
		case SNDRV_PCM_FORMAT_U20_3LE:
		case SNDRV_PCM_FORMAT_S18_3LE:
		case SNDRV_PCM_FORMAT_U18_3LE:
		case SNDRV_PCM_FORMAT_S24_3BE:
		case SNDRV_PCM_FORMAT_U24_3BE:
		case SNDRV_PCM_FORMAT_S20_3BE:
		case SNDRV_PCM_FORMAT_U20_3BE:
		case SNDRV_PCM_FORMAT_S18_3BE:
		case SNDRV_PCM_FORMAT_U18_3BE:
			return true;
		default:
			break;
	}

	return false;
}

static inline struct ingenic_dmic_dma_runtime *substream_to_prtd(
    const struct snd_pcm_substream *substream)
{
	return substream->runtime->private_data;
}

static bool ingenic_dmic_params_check(int channels, int rate, int fmt_width)
{
	if ((1 != channels) && (2 != channels) && (4 != channels)) {
		goto error;
	}

	if ((rate != 8000) && (rate != 16000)
	    && (rate != 48000) && (rate != 96000)) {
		goto error;
	}

	if ((fmt_width != 16) && (fmt_width != 24)) {
		goto error;
	}

	return true;

error:
	pr_err("%s %d, parameter is illegal!\n", __func__, __LINE__);
	return false;
}

#ifdef CONFIG_PM
static int ingenic_dmic_runtime_suspend(struct device *dev)
{
	return 0;
}

static int ingenic_dmic_runtime_resume(struct device *dev)
{
	return 0;
}

#endif

static snd_pcm_uframes_t ingenic_dmic_dma_pointer(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct ingenic_dmic_dma_runtime *prtd = substream_to_prtd(substream);
	dma_addr_t now;
	ssize_t pos;

	if (likely(substream->runtime && substream->runtime->dma_addr)) {
		// now = readl_relaxed(as_dma->dma_base + DBA(prtd->dai_id));
		now = prtd->hw_ptr;
		if (now) {
			pos = (ssize_t)(now - substream->runtime->dma_addr);
		} else {
			pos = 0;
		}
	} else {
		WARN("%s: %s has no runtime\n", substream->name);
		pos = 0;
	}

	if (pos >= snd_pcm_lib_buffer_bytes(substream)) {
		pos = 0;
	}

	return bytes_to_frames(substream->runtime, pos);
}

static int ingenic_dmic_dma_period_bytes_quirk(struct snd_pcm_hw_params *params,
        struct snd_pcm_hw_rule *rule)
{
	struct snd_interval *iperiod_bytes = hw_param_interval(params,
	                                     SNDRV_PCM_HW_PARAM_PERIOD_BYTES);
	struct snd_interval *iframe_bits = hw_param_interval(params,
	                                   SNDRV_PCM_HW_PARAM_FRAME_BITS);
	int align_bytes = DCM_TSZ_MAX_WORD << 2;
	int min_frame_bytes = iframe_bits->min >> 3;
	int max_frame_bytes = iframe_bits->max >> 3;
	int min_period_bytes = iperiod_bytes->min;
	int max_period_bytes = iperiod_bytes->max;
	int min_align_bytes, max_align_bytes;
	struct snd_interval nperiod_bytes;

	snd_interval_any(&nperiod_bytes);
	min_align_bytes = lcm(align_bytes, min_frame_bytes);
	min_period_bytes = (min_period_bytes + min_align_bytes - 1) / min_align_bytes;
	nperiod_bytes.min = min_period_bytes * min_align_bytes;

	max_align_bytes = lcm(align_bytes, max_frame_bytes);
	max_period_bytes = max_period_bytes / max_align_bytes;
	nperiod_bytes.max = max_period_bytes * max_align_bytes;

	return snd_interval_refine(iperiod_bytes, &nperiod_bytes);
}

static int ingenic_dmic_dma_open(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct ingenic_dmic *dmic = snd_soc_component_get_drvdata(component);
	struct ingenic_dmic_dma_runtime *prtd = NULL;
	struct snd_pcm_hardware pcm_hardware = ingenic_dmic_dma_pcm_hardware;
	int ret;

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (!prtd) {
		return -ENOMEM;
	}

	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		return ret;
	}

	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIOD_BYTES);
	if (ret < 0) {
		return ret;
	}

	if (dmic->dma->dma_fth_quirk) {
		snd_pcm_hw_rule_add(substream->runtime, 0,
		                    SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
		                    ingenic_dmic_dma_period_bytes_quirk,
		                    NULL,
		                    SNDRV_PCM_HW_PARAM_FRAME_BITS,
		                    SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
		                    -1);
	}

	prtd->dmic_dma = dmic->dma;
	prtd->substream = substream;
	runtime->private_data = prtd;

	/* dummy desc */
	prtd->dummy_descs = (struct ingenic_dmic_dma_desc *)dma_alloc_coherent(dmic->dev,
	                    sizeof(struct ingenic_dmic_dma_desc), &prtd->dummy_descs_phy, GFP_KERNEL);
	if (IS_ERR_OR_NULL(prtd->dummy_descs)) {
		return -ENOMEM;
	}

	spin_lock(&dmic->dma->lock);
	dmic->dma->rtd = prtd;
	dmic->dma->refcnt++;
	prtd->fifo_depth = dmic->dma->fifo_depth;
	spin_unlock(&dmic->dma->lock);

	pcm_hardware.fifo_size = prtd->fifo_depth << 2;
	snd_soc_set_runtime_hwparams(substream, &pcm_hardware);

	return 0;
}

static int ingenic_dmic_dma_close(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct ingenic_dmic_dma_runtime *prtd = substream_to_prtd(substream);
	struct ingenic_dmic *dmic = snd_soc_component_get_drvdata(component);

	spin_lock(&dmic->dma->lock);
	dmic->dma->refcnt--;
	dmic->dma->rtd = NULL;
	spin_unlock(&dmic->dma->lock);
	dma_free_coherent(dmic->dev, sizeof(struct ingenic_dmic_dma_desc), prtd->dummy_descs, prtd->dummy_descs_phy);
	kfree(prtd);
	substream->runtime->private_data = NULL;

	return 0;
}

static int ingenic_dmic_dma_ioctl(struct snd_soc_component *component, struct                  snd_pcm_substream *substream, unsigned int cmd, void *arg)
{
	return snd_pcm_lib_ioctl(substream, cmd, arg);
}

/* RETURN 0 not change, 1 change, < 0 failed */
static int ingenic_dmic_dma_alloc_descs(struct ingenic_dmic_dma_runtime *prtd, int cnt)
{
	struct ingenic_dmic_dma *dmic_dma = prtd->dmic_dma;
	int i, old_cnts = prtd->desc_cnts;

	if (cnt == prtd->desc_cnts) {
		return 0;
	}

	for (i = cnt; i < old_cnts; i++) {
		dma_pool_free(dmic_dma->desc_pool, prtd->descs[i], prtd->descs_phy[i]);
		prtd->descs[i] = NULL;
		prtd->descs_phy[i] = 0;
		prtd->desc_cnts--;
	}

	for (i = old_cnts; i < cnt; i++) {
		prtd->descs[i] = dma_pool_alloc(dmic_dma->desc_pool,
		                                GFP_KERNEL, &prtd->descs_phy[i]);
		if (!prtd->descs[i]) {
			return -ENOMEM;
		}

		prtd->desc_cnts++;
	}

	return 1;

}

static void ingenic_dmic_dma_free_descs(struct ingenic_dmic_dma_runtime *prtd)
{
	struct ingenic_dmic_dma *dmic_dma = prtd->dmic_dma;
	int i = 0;

	for (i = 0; i < prtd->desc_cnts; i++) {
		dma_pool_free(dmic_dma->desc_pool, prtd->descs[i], prtd->descs_phy[i]);
		prtd->descs[i] = NULL;
		prtd->descs_phy[i] = 0;
	}
	prtd->desc_cnts = 0;
}

static void ingenic_dmic_dma_cyclic_descs_init(struct ingenic_dmic_dma_runtime *prtd,
        dma_addr_t dma_addr, struct snd_pcm_hw_params *params)
{
	struct ingenic_dmic_dma *dmic_dma = prtd->dmic_dma;
	int i, tsz, ts_size;
	int period_bytes = params_period_bytes(params);

	if (unlikely(!dmic_dma->dma_fth_quirk)) {
		tsz = ingenic_dmic_dma_get_tsz(prtd->fifo_depth, period_bytes, &ts_size);
	} else {
		ts_size = DCM_TSZ_MAX_WORD << 2;
		tsz = DCM_TSZ_MAX;
	}

	prtd->tsz_words = ts_size >> 2;

	if (prtd->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		printk("----- DMIC supports only recording.\r\n");
	}

	for (i = 0; i < prtd->desc_cnts; i++) {
		memset(prtd->descs[i], 0, sizeof(prtd->descs[i]));
		prtd->descs[i]->link = prtd->descs[i]->tie = prtd->descs[i]->bai = 1;
		prtd->descs[i]->tsz = tsz;
		prtd->descs[i]->dba = dma_addr + i * period_bytes;
		prtd->descs[i]->ndoa = *(prtd->descs_phy + ((i + 1) % prtd->desc_cnts));
		prtd->descs[i]->dtc = (period_bytes / ts_size);
	}

	iob();
}

static int ingenic_dmic_dma_hw_params(struct snd_soc_component *component, struct snd_pcm_substream *substream,
                                      struct snd_pcm_hw_params *params)
{
	struct ingenic_dmic *dmic = snd_soc_component_get_drvdata(component);
	struct ingenic_dmic_dma_runtime *prtd = substream_to_prtd(substream);

	int *x = NULL;
	int ret = 0;
	u32 fmtcfg = 0;

	ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (ret < 0) {
		return ret;
	}

	ret = ingenic_dmic_dma_alloc_descs(prtd, params_periods(params));
	if (ret < 0) {
		return ret;
	}

	ingenic_dmic_dma_cyclic_descs_init(prtd,
	                                   substream->runtime->dma_addr,
	                                   params);

	x = (int *)prtd->descs[0];

	fmtcfg = DFCR_SS(params_width(params)) | DFCR_CHNUM(params_channels(params));

	if (ingenic_fmt_need_packen(params_format(params))) {
		fmtcfg |= DFCR_PACK_EN;
	}

	writel_relaxed(fmtcfg, dmic->io_base + DFCR);

	regmap_write(dmic->regmap, FFR, FFR_FTH(prtd->tsz_words) | FFR_FIFO_TD);

	return 0;
}

static int ingenic_dmic_dma_hw_free(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct ingenic_dmic_dma_runtime *prtd = substream_to_prtd(substream);

	ingenic_dmic_dma_free_descs(prtd);

	return snd_pcm_lib_free_pages(substream);
}

static int ingenic_dmic_dma_start(struct ingenic_dmic *dmic)
{
	struct ingenic_dmic_dma_runtime *prtd = dmic->dma->rtd;

	regmap_write(dmic->regmap, DSR, DSR_TT_INT | DSR_LTT_INT | DSR_LTT | DSR_TT);
	regmap_update_bits(dmic->regmap, DCM, DCM_NDES | DCM_TIE, DCM_TIE);

	/**
	 * Note: DMA enable first FIFO enable, read operation
	 * to ensure that the DMA module work
	 */
	if (prtd->dummy_data_flag) {
		prtd->dummy_data_irq = 1;
		regmap_write(dmic->regmap, DDA, prtd->dummy_descs_phy);
	} else {
		regmap_write(dmic->regmap, DDA, prtd->descs_phy[0]);
	}
	readl_relaxed(dmic->io_base + DDA);

	regmap_update_bits(dmic->regmap, DFCR, DFCR_RECEN, DFCR_RECEN);

	regmap_write(dmic->regmap, DCR, DCR_CTE);

	regmap_update_bits(dmic->regmap, FCR, FCR_FLUSH, FCR_FLUSH);

	regmap_write(dmic->regmap, FCR, FCR_FIFO_EN);

	if (IS_BUILTIN(CONFIG_FPGA_TEST) || ingenic_dma_debug) {
		regmap_write(dmic->regmap, FSR, FSR_TURROR);
		regmap_update_bits(dmic->regmap, FFR, FFR_TURROREN, FFR_TURROREN);
	}

	return 0;
}

static int ingenic_dmic_dma_stop(struct ingenic_dmic *dmic)
{
	if (IS_BUILTIN(CONFIG_FPGA_TEST) || ingenic_dma_debug) {
		regmap_update_bits(dmic->regmap, FFR, FFR_TURROREN, 0);
		regmap_write(dmic->regmap, FSR, FSR_TURROR);
	}

	regmap_update_bits(dmic->regmap, DCM, DCM_TIE, 0);

	regmap_write(dmic->regmap, DSR, DSR_TT_INT | DSR_LTT_INT | DSR_LTT | DSR_TT);

	regmap_write(dmic->regmap, FCR, 0);

	regmap_write(dmic->regmap, DCR, 0);
	regmap_write(dmic->regmap, DCR, DCR_DFS);

	udelay(10);

	regmap_update_bits(dmic->regmap, DFCR, DFCR_RECEN, 0);

	return 0;
}

static int ingenic_dmic_dma_trigger(struct snd_soc_component *component, struct snd_pcm_substream *substream, int cmd)
{
	struct ingenic_dmic *dmic = snd_soc_component_get_drvdata(component);

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:      /*FIXME*/
			regmap_write(dmic->regmap, BSTR0, BSTR0_DEV0_SUR);
			regmap_write(dmic->regmap, BTTR0, BTTR0_DEV5_TAR);
			regmap_write(dmic->regmap, BTSET, BTSET_TSLOT_SET);
			regmap_write(dmic->regmap, BFCR0, BFCR0_DEV5_TFIFO | BFCR0_DEV0_RFIFO);
			return ingenic_dmic_dma_start(dmic);
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
			return ingenic_dmic_dma_stop(dmic);
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			/*JUST stop or resume the be dai*/
			return 0;
		default:
			return -EINVAL;
	}
}

static int ingenic_dmic_dma_pcm_new(struct snd_soc_component *component, struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;

	/**
	 * nonatomic条件，中断是使用threaded_irq，线程安全的。
	 *
	 * snd_pcm_period_elapsed(substream); 的调用在线程上下文.
	 * 1. 使用nonatomic的好处，在alsa pcm_lib里面使用了大量的
	 * snd_pcm_stream_lock_irqsave(substream, flags);
	 * snd_pcm_stream_lock_irq();
	 * 关闭本地中断，并且处理过于复杂，占用较多时间，导致其他硬件响应出问题.
	 */
	pcm->nonatomic = true;

	/* SNDRV_DMA_TYPE_DEV: 使用 dma_alloc_coherent API 申请一致性内存. uncache访问, 读写效率低，实际copy时应该使用cache地址. */
	/* SNDRV_DMA_TYPE_CONTINUOUS: 通用的使用get_free_pages申请连续的内存，cache 访问，读写效率高，需要刷cache. */
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV, card->dev,
	                                      INGENIC_DMA_BUFFERSIZE_PREALLOC, INGENIC_DMA_BUFFERSIZE_MAX);
	return 0;
}

static void ingenic_dmic_dma_pcm_free(struct snd_soc_component *component, struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

static const struct snd_soc_component_driver ingenic_dmic_dma_platform = {
	.name       = "as-dmic-dma",
	.open       = ingenic_dmic_dma_open,
	.close      = ingenic_dmic_dma_close,
	.ioctl      = ingenic_dmic_dma_ioctl,
	.hw_params  = ingenic_dmic_dma_hw_params,
	.hw_free    = ingenic_dmic_dma_hw_free,
	.trigger    = ingenic_dmic_dma_trigger,
	.pointer    = ingenic_dmic_dma_pointer,
	.pcm_construct  = ingenic_dmic_dma_pcm_new,
	.pcm_destruct   = ingenic_dmic_dma_pcm_free,
};

static int ingenic_dmic_set_work_freq(struct ingenic_dmic *dmic)
{
	int ret = 0;

#ifdef CONFIG_SND_ASOC_DMIC_CLK_2400KHz
	ret = clk_set_rate(dmic->clk, 24000000);
	if (ret == -EBUSY) {
		clk_disable_unprepare(dmic->clk);
		ret = clk_set_rate(dmic->clk, 24000000);
	}
	clk_prepare_enable(dmic->clk);

	/* extclk : 24MHz ----> 2.4MHz DMIC_CLK @ 24MHz */
	regmap_update_bits(dmic->regmap, DMIC_CR0, DMIC_CR0_FREQ, 0);
#endif

#ifdef CONFIG_SND_ASOC_DMIC_CLK_3072KHz
	ret = clk_set_rate(dmic->clk, 24576000);
	if (ret == -EBUSY) {
		clk_disable_unprepare(dmic->clk);
		ret = clk_set_rate(dmic->clk, 24576000);
	}
	clk_prepare_enable(dmic->clk);

	/* 3.072MHz DMIC_CLK @ 24.576MHz */
	regmap_update_bits(dmic->regmap, DMIC_CR0, DMIC_CR0_FREQ, DMIC_CR0_FREQ);
#endif

	return 0;
}

static int ingenic_dmic_clk_init(struct platform_device *pdev,
                                 struct ingenic_dmic *dmic)
{
	struct clk *clk = NULL;
	int i = 0;

	for (i = 0; dmic->priv[i].clk_name != NULL; i++) {
		if (dmic->priv[i].clk_mode == DMIC_CLOCK_GATE) {
			if (dmic->priv[i].is_bus_clk) {
				clk = devm_clk_get(dmic->dev, dmic->priv[i].clk_name);
				if (IS_ERR(clk)) {
					return PTR_ERR(clk);
				}
				clk_prepare_enable(clk);
			}
		}

		if (dmic->priv[i].clk_mode == DMIC_CLOCK_CE) {
			clk = devm_clk_get(dmic->dev, dmic->priv[i].clk_name);
			if (IS_ERR(clk)) {
				return PTR_ERR(clk);
			}
			clk_prepare_enable(clk);
		}

		if (dmic->priv[i].clk_mode == DMIC_CLOCK_MUX) {
#ifdef CONFIG_SND_ASOC_DMIC_CLK_2400KHz
			dmic->clk = devm_clk_get(&pdev->dev, dmic->priv[i].clk_name);
			if (IS_ERR(dmic->clk)) {
				return PTR_ERR(dmic->clk);
			}
#endif

#ifdef CONFIG_SND_ASOC_DMIC_CLK_3072KHz
			clk = devm_clk_get(&pdev->dev, dmic->priv[i].clk_name);
			if (IS_ERR(clk)) {
				return PTR_ERR(clk);
			}
#endif
			clk_set_parent(clk_get(NULL, dmic->priv[i].clk_name), clk_get(NULL, dmic->priv[i].mux_select));
		}

		if (dmic->priv[i].clk_mode == DMIC_CLOCK_DIV) {
#ifdef CONFIG_SND_ASOC_DMIC_CLK_3072KHz
			dmic->clk = devm_clk_get(&pdev->dev, dmic->priv[i].clk_name);
			if (IS_ERR(dmic->clk)) {
				return PTR_ERR(dmic->clk);
			}
#endif
		}
	}

	return 0;
}

static int ingenic_dmic_clk_deinit(struct platform_device *pdev,
                                   struct ingenic_dmic *dmic)
{
	struct clk *clk =  NULL;
	int i = 0;

	for (i = 0; dmic->priv[i].clk_name != NULL; i++) {
		if (dmic->priv[i].clk_mode == DMIC_CLOCK_GATE || dmic->priv[i].clk_mode == DMIC_CLOCK_CE) {
			clk = devm_clk_get(&pdev->dev, dmic->priv[i].clk_name);
			if (IS_ERR(clk)) {
				return PTR_ERR(clk);
			}
			clk_disable_unprepare(clk);
		}
	}

	return 0;
}

static int ingenic_dmic_probe(struct snd_soc_dai *dai)
{
	struct ingenic_dmic *dmic = dev_get_drvdata(dai->dev);

	regmap_update_bits(dmic->regmap, DMIC_CR0,
	                   DMIC_CR0_HPF2_EN | DMIC_CR0_LPF_EN | DMIC_CR0_HPF1_EN,
	                   DMIC_CR0_HPF2_EN | DMIC_CR0_LPF_EN | DMIC_CR0_HPF1_EN);

	/* gain: 0, ..., 1F */
	regmap_write(dmic->regmap, DMIC_GCR, DMIC_GCR_SET_DGAIN(4));

#ifdef CONFIG_DMIC_AND_AMIC_SYNC
	regmap_update_bits(dmic->regmap, DMIC_CR0, DMIC_CR0_EXEN, DMIC_CR0_EXEN);
#endif

	return 0;
}

#define ingenic_dmic_suspend    NULL
#define ingenic_dmic_resume     NULL

static int ingenic_dmic_reset(struct ingenic_dmic *dmic)
{
	regmap_update_bits(dmic->regmap, DMIC_CR0, DMIC_CR0_SOFT_RST, DMIC_CR0_SOFT_RST);

	ndelay(350);

	return 0;
}

static int ingenic_dmic_startup(struct snd_pcm_substream *substream,
                                struct snd_soc_dai *dai)
{
	struct ingenic_dmic *dmic = dev_get_drvdata(dai->dev);

	return ingenic_dmic_reset(dmic);
}

static int ingenic_dmic_trigger(struct snd_pcm_substream *substream,
                                int cmd, struct snd_soc_dai *dai)
{
	struct ingenic_dmic *dmic = dev_get_drvdata(dai->dev);

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			regmap_update_bits(dmic->regmap, DMIC_CR0, DMIC_CR0_DMIC_EN, DMIC_CR0_DMIC_EN);
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			regmap_update_bits(dmic->regmap, DMIC_CR0, DMIC_CR0_DMIC_EN, 0);
			return ingenic_dmic_reset(dmic);
		default:
			return -EINVAL;
	}

	return 0;
}

static int ingenic_dmic_hw_params(struct snd_pcm_substream *substream,
                                  struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	int channels = params_channels(params);
	int rate = params_rate(params);
	int fmt_width = snd_pcm_format_width(params_format(params));
	struct ingenic_dmic *dmic = dev_get_drvdata(dai->dev);
	u32 dmic_cr = 0;

	ingenic_dmic_set_work_freq(dmic);

	if (!ingenic_dmic_params_check(channels, rate, fmt_width)) {
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		dmic_cr = DMIC_CR0_SET_CHNUM(channels);
		dmic_cr |= DMIC_CR0_SET_SR(rate);
		dmic_cr |= DMIC_CR0_SET_OSS(fmt_width);

		regmap_update_bits(dmic->regmap, DMIC_CR0, DMIC_CR0_CHNUM_MASK |
		                   DMIC_CR0_SR_MASK | DMIC_CR0_OSS_MASK, dmic_cr);
	} else {
		dev_err(dmic->dev, "DMIC is a capture device\n");
		return -ENXIO;
	}

	return 0;
}

static struct snd_soc_dai_ops ingenic_dmic_dai_ops = {
	.probe      = ingenic_dmic_probe,
	.startup    = ingenic_dmic_startup,
	.trigger    = ingenic_dmic_trigger,
	.hw_params  = ingenic_dmic_hw_params,
};

/**
 * GAIN volume control:
 *      from 0 to 93 dB in 3 dB steps (0dB means no gain is used)
 */
static const DECLARE_TLV_DB_SCALE(dmic_tlv, 0, 300, 0);

static const struct snd_kcontrol_new ingenic_dmic_controls[] = {
	SOC_SINGLE_TLV("DMIC Capture Volume", DMIC_GCR, DGAIN_SFT, 31, 0, dmic_tlv),

	SOC_SINGLE("DMIC High Pass Filter1 Switch", DMIC_CR0, DMIC_CR0_HPF1_EN_SFT, 1, 0),
	SOC_SINGLE("DMIC High Pass Filter2 Switch", DMIC_CR0, DMIC_CR0_HPF2_EN_SFT, 1, 0),
	SOC_SINGLE("DMIC Low Pass Filter Switch", DMIC_CR0, DMIC_CR0_LPF_EN_SFT, 1, 0),

	SOC_SINGLE("DMIC SW_LR Switch", DMIC_CR0, DMIC_CR0_SW_LR_SFT, 1, 0),
};

struct snd_soc_component_driver ingenic_dmic_component = {
	.name = "asoc-dmic-v2",
	.controls       = ingenic_dmic_controls,
	.num_controls   = ARRAY_SIZE(ingenic_dmic_controls),
};

static struct snd_soc_dai_driver dmic_codec_dai = {
	.name = "dmic-dump-codec",
	.capture = {
		.channels_min = 1,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_48000 | \
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static int ingenic_dmic_platform_probe(struct platform_device *pdev)
{
	struct ingenic_dmic *dmic = NULL;
	const struct of_device_id *match = NULL;
	struct snd_soc_component_driver *dmic_dump_codec = NULL;
	struct resource *res = NULL;
	u32 fifo_depth = 0;
	int ret = 0;

	struct regmap_config regmap_config = {
		.reg_bits = 32,
		.reg_stride = 4,
		.val_bits = 32,
		.cache_type = REGCACHE_NONE,
	};

	dmic = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_dmic) +
	                    sizeof(struct snd_soc_dai_driver) + sizeof(struct ingenic_dmic_dma),
	                    GFP_KERNEL);
	if (!dmic) {
		return -ENOMEM;
	}

	dmic_dump_codec = (struct snd_soc_component_driver *)devm_kzalloc(&pdev->dev,
	                  sizeof(struct snd_soc_component_driver), GFP_KERNEL);
	if (!dmic_dump_codec) {
		return -ENOMEM;
	}

	dmic->dai_driver = (struct snd_soc_dai_driver *)(dmic + 1);
	dmic->dma = (struct ingenic_dmic_dma *)(dmic->dai_driver + 1);

	dmic->dma->rtd = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_dmic_dma_runtime), GFP_KERNEL);
	if (!dmic->dma->rtd) {
		return -ENOMEM;
	}

	match = of_match_node(ingenic_dmic_match_table, pdev->dev.of_node);
	if (!match) {
		return -ENODEV;
	}

	if (!match->data) {
		dev_err(&pdev->dev, "match->data is null!\r\n");
		return -ENODEV;
	}

	dmic->priv = (struct ingenic_dmic_priv *)match->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		return -EINVAL;
	}

	dmic->io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dmic->io_base)) {
		return PTR_ERR(dmic->io_base);
	}

	regmap_config.max_register = resource_size(res) - 0x4;
	dmic->regmap = devm_regmap_init_mmio(&pdev->dev, dmic->io_base, &regmap_config);
	if (IS_ERR(dmic->regmap)) {
		return PTR_ERR(dmic->regmap);
	}

	dmic->dma->dummy_data = (uint8_t *)dma_alloc_coherent(&pdev->dev,
	                        sizeof(uint8_t) * 4 * 32, &dmic->dma->dummy_data_pyaddr, GFP_KERNEL);
	if (IS_ERR_OR_NULL(dmic->dma->dummy_data)) {
		return -ENOMEM;
	}

	dmic->dma->irq = of_irq_get(pdev->dev.of_node, 0);
	if (dmic->dma->irq < 0) {
		dma_free_coherent(&pdev->dev, sizeof(uint8_t) * 4 * 32, dmic->dma->dummy_data,
		                  dmic->dma->dummy_data_pyaddr);
		return dmic->dma->irq;
	}

	spin_lock_init(&dmic->dma->lock);


	if (of_property_read_u32(pdev->dev.of_node, "ingenic,fifo_size", &dmic->dma->fifo_total_depth)) {
		dmic->dma->fifo_total_depth = INGENIC_DEF_FIFO_DEPTH;
	}

	dmic->dma->dma_fth_quirk = of_property_read_bool(pdev->dev.of_node, "ingenic,fth_quirk");

	fifo_depth = 384;
	dmic->dma->fifo_depth = fifo_depth;

	ret = devm_request_threaded_irq(&pdev->dev, dmic->dma->irq, NULL,
	                                ingenic_dmic_dma_irq_handler, IRQF_SHARED | IRQF_ONESHOT,
	                                "as-dmic-dma", dmic);
	if (ret) {
		dma_free_coherent(&pdev->dev, sizeof(uint8_t) * 4 * 32, dmic->dma->dummy_data,
		                  dmic->dma->dummy_data_pyaddr);
		return ret;
	}

	dmic->dma->desc_pool = dma_pool_create("dmic_dma_desc_pool", &pdev->dev,
	                                       sizeof(struct ingenic_dmic_dma_desc),
	                                       __alignof__(struct ingenic_dmic_dma_desc), 0);
	if (!dmic->dma->desc_pool) {
		dma_free_coherent(&pdev->dev, sizeof(uint8_t) * 4 * 32, dmic->dma->dummy_data,
		                  dmic->dma->dummy_data_pyaddr);
		return -ENOMEM;
	}

	dmic->dev = &pdev->dev;
	dmic->dai_driver->name = "dmic-v2";
	dmic->dai_driver->ops = &ingenic_dmic_dai_ops;
	dmic->dai_driver->capture.channels_min = 1;
	dmic->dai_driver->capture.channels_max = 4;
	dmic->dai_driver->capture.rates = SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_48000 | \
	                                  SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_8000;
	dmic->dai_driver->capture.formats = SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S16_LE;

	platform_set_drvdata(pdev, dmic);

	ret = snd_soc_register_component(&pdev->dev,
	                                 dmic_dump_codec, &dmic_codec_dai, 1);
	if (ret) {
		dma_pool_destroy(dmic->dma->desc_pool);
		dma_free_coherent(&pdev->dev, sizeof(uint8_t) * 4 * 32, dmic->dma->dummy_data,
		                  dmic->dma->dummy_data_pyaddr);
		return ret;
	}

	ret = devm_snd_soc_register_component(&pdev->dev, &ingenic_dmic_dma_platform, NULL, 0);
	if (ret) {
		snd_soc_unregister_component(&pdev->dev);
		dma_pool_destroy(dmic->dma->desc_pool);
		dma_free_coherent(&pdev->dev, sizeof(uint8_t) * 4 * 32, dmic->dma->dummy_data,
		                  dmic->dma->dummy_data_pyaddr);
		return ret;
	}

	ingenic_dmic_clk_init(pdev, dmic);

	regmap_write(dmic->regmap, DGRR, DGRR_RESET);

	regmap_write(dmic->regmap, FAS, fifo_depth);

	regmap_write(dmic->regmap, BFCR1, BFCR1_DEV5_LSMP);

	regmap_write(dmic->regmap, BFCR2, BFCR2_DEV0_DBE);

	ret = devm_snd_soc_register_component(&pdev->dev, &ingenic_dmic_component,
	                                      dmic->dai_driver, 1);
	if (ret) {
		snd_soc_unregister_component(&pdev->dev);
		dma_pool_destroy(dmic->dma->desc_pool);
		dma_free_coherent(&pdev->dev, sizeof(uint8_t) * 4 * 32, dmic->dma->dummy_data,
		                  dmic->dma->dummy_data_pyaddr);
		return ret;
	}

	dev_info(&pdev->dev, "dmic platform probe success\n");

	return ret;
}

static int ingenic_dmic_platform_remove(struct platform_device *pdev)
{
	struct ingenic_dmic *dmic = platform_get_drvdata(pdev);

	ingenic_dmic_clk_deinit(pdev, dmic);

	snd_soc_unregister_component(&pdev->dev);
	dma_pool_destroy(dmic->dma->desc_pool);
	dma_free_coherent(&pdev->dev, sizeof(uint8_t) * 4 * 32, dmic->dma->dummy_data,
	                  dmic->dma->dummy_data_pyaddr);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct ingenic_dmic_priv x2600_priv_data[] = {
#ifdef CONFIG_SND_ASOC_DMIC_CLK_2400KHz
	{ .clk_mode = DMIC_CLOCK_GATE, .clk_name = "gate_dmic", .is_bus_clk = true },
	{ .clk_mode = DMIC_CLOCK_CE,   .clk_name = "ce_dmic" },
	{ .clk_mode = DMIC_CLOCK_MUX,  .clk_name = "mux_dmic", .mux_select = "ext" },
#endif
#ifdef CONFIG_SND_ASOC_DMIC_CLK_3072KHz
	{ .clk_mode = DMIC_CLOCK_GATE, .clk_name = "gate_dmic", .is_bus_clk = true },
	{ .clk_mode = DMIC_CLOCK_GATE, .clk_name = "gate_i2s",  .is_bus_clk = false },
	{ .clk_mode = DMIC_CLOCK_CE,   .clk_name = "ce_i2st" },
	{ .clk_mode = DMIC_CLOCK_DIV,  .clk_name = "div_i2s_mn" },
	{ .clk_mode = DMIC_CLOCK_MUX,  .clk_name = "mux_i2scs", .mux_select = "epll" },
#endif
	{ /* end */},
};

static const struct of_device_id ingenic_dmic_match_table[] = {
	{ .compatible = "ingenic,x2600-dmic", .data = (void *)x2600_priv_data },
	{ },
};
MODULE_DEVICE_TABLE(of, ingenic_dmic_match_table);

static const struct dev_pm_ops ingenic_dmic_pm_ops = {
	SET_RUNTIME_PM_OPS(ingenic_dmic_runtime_suspend,
	                   ingenic_dmic_runtime_resume, NULL)
};

static struct platform_driver ingenic_dmic_platform_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "asoc-dmic-v2",
		.of_match_table = of_match_ptr(ingenic_dmic_match_table),
		// .pm = &ingenic_dmic_pm_ops,
	},
	.probe = ingenic_dmic_platform_probe,
	.remove = ingenic_dmic_platform_remove,
};

module_platform_driver(ingenic_dmic_platform_driver);

MODULE_AUTHOR("chang.wang@ingenic.com");
MODULE_DESCRIPTION("Ingenic AS dmic SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-asoc-dmic-v2");
