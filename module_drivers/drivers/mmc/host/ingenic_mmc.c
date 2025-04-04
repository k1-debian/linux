/*
 * Ingenic MMC/SD Controller driver
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 * Written by Large Dipper <ykli@ingenic.com>.
 *
 * Modified by qipengzhen <aric.pzqi@ingenic.com> 2016-04-21
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <asm-generic/delay.h>

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/clk-provider.h>

#include "ingenic_mmc.h"

/**
 * MMC driver parameters
 */
#define TIMEOUT_PERIOD      3000    /* msc operation timeout detect period */
#define PIO_THRESHOLD       64  /* use pio mode if data length < PIO_THRESHOLD */
#define CLK_RATE (CONFIG_EXTAL_CLOCK * 1000000)
#define CLK_CTRL

#define ERROR_IFLG (                            \
        IFLG_CRC_RES_ERR    |                   \
        IFLG_CRC_READ_ERR   |                   \
        IFLG_CRC_WRITE_ERR  |                   \
        IFLG_TIMEOUT_RES    |                   \
        IFLG_TIMEOUT_READ)

/*
 * Error status including CRC_READ_ERROR, CRC_WRITE_ERROR,
 * CRC_RES_ERR, TIME_OUT_RES, TIME_OUT_READ
 */
#define ERROR_STAT      0x3f

#define ingenic_mmc_check_pending(host, event)      \
	test_and_clear_bit(event, &host->pending_events)
#define ingenic_mmc_set_pending(host, event)            \
	set_bit(event, &host->pending_events)
#define is_pio_mode(host)                       \
	(host->flags & (1 << INGENIC_MMC_USE_PIO))
#define enable_pio_mode(host)                   \
	(host->flags |= (1 << INGENIC_MMC_USE_PIO))
#define disable_pio_mode(host)                  \
	(host->flags &= ~(1 << INGENIC_MMC_USE_PIO))

static LIST_HEAD(manual_list);

/*-------------------End structure and macro define------------------------*/

#ifndef IT_IS_USED_FOR_DEBUG
static void ingenic_mmc_dump_reg(struct ingenic_mmc_host *host)
{
	dev_info(host->dev, "\nREG dump:\n"
	         "\tCTRL2\t= 0x%08X\n"
	         "\tSTAT\t= 0x%08X\n"
	         "\tCLKRT\t= 0x%08X\n"
	         "\tCMDAT\t= 0x%08X\n"
	         "\tRESTO\t= 0x%08X\n"
	         "\tRDTO\t= 0x%08X\n"
	         "\tBLKLEN\t= 0x%08X\n"
	         "\tNOB\t= 0x%08X\n"
	         "\tSNOB\t= 0x%08X\n"
	         "\tIMASK\t= 0x%08X\n"
	         "\tIFLG\t= 0x%08X\n"
	         "\tCMD\t= 0x%08X\n"
	         "\tARG\t= 0x%08X\n"
	         "\tRES\t= 0x%08X\n"
	         "\tLPM\t= 0x%08X\n"
	         "\tDMAC\t= 0x%08X\n"
	         "\tDMANDA\t= 0x%08X\n"
	         "\tDMADA\t= 0x%08X\n"
	         "\tDMALEN\t= 0x%08X\n"
	         "\tDMACMD\t= 0x%08X\n"
	         "\tRTCNT\t= 0x%08X\n"
	         "\tDEBUG\t= 0x%08X\n",

	         msc_readl(host, CTRL2),
	         msc_readl(host, STAT),
	         msc_readl(host, CLKRT),
	         msc_readl(host, CMDAT),
	         msc_readl(host, RESTO),
	         msc_readl(host, RDTO),
	         msc_readl(host, BLKLEN),
	         msc_readl(host, NOB),
	         msc_readl(host, SNOB),
	         msc_readl(host, IMASK),
	         msc_readl(host, IFLG),
	         msc_readl(host, CMD),
	         msc_readl(host, ARG),
	         msc_readl(host, RES),
	         msc_readl(host, LPM),
	         msc_readl(host, DMAC),
	         msc_readl(host, DMANDA),
	         msc_readl(host, DMADA),
	         msc_readl(host, DMALEN),
	         msc_readl(host, DMACMD),
	         msc_readl(host, RTCNT),
	         msc_readl(host, DEBUG));
}
#endif

/*
 * Functional functions.
 *
 * These small function will be called frequently.
 */
static inline void enable_msc_irq(struct ingenic_mmc_host *host, unsigned long bits)
{
	unsigned long imsk;

	spin_lock_bh(&host->lock);
	imsk = msc_readl(host, IMASK);
	imsk &= ~bits;
	msc_writel(host, IMASK, imsk);
	spin_unlock_bh(&host->lock);
}

static inline void clear_msc_irq(struct ingenic_mmc_host *host, unsigned long bits)
{
	msc_writel(host, IFLG, bits);
}

static inline void disable_msc_irq(struct ingenic_mmc_host *host, unsigned long bits)
{
	unsigned long imsk;

	spin_lock_bh(&host->lock);
	imsk = msc_readl(host, IMASK);
	imsk |= bits;
	msc_writel(host, IMASK, imsk);
	spin_unlock_bh(&host->lock);
}

static inline void ingenic_mmc_reset(struct ingenic_mmc_host *host)
{
	unsigned int clkrt = msc_readl(host, CLKRT);
	unsigned int cnt = 100 * 1000 * 1000;
	int vl;

	msc_writel(host, CTRL, CTRL_RESET);
	vl = msc_readl(host, CTRL);
	vl &= ~CTRL_RESET;
	msc_writel(host, CTRL, vl);

	while ((msc_readl(host, STAT) & STAT_IS_RESETTING) && (--cnt));
	WARN_ON(!cnt);

	if (host->pdata->sdio_clk) {
		msc_writel(host, CTRL, CTRL_CLOCK_START);
	} else {
		msc_writel(host, LPM, LPM_LPM);
	}

	msc_writel(host, IMASK, 0xffffffff);
	msc_writel(host, IFLG, 0xffffffff);

	msc_writel(host, CLKRT, clkrt);
}

static inline void ingenic_mmc_stop_dma(struct ingenic_mmc_host *host)
{
	dev_warn(host->dev, "%s\n", __func__);

	/*
	 * Theoretically, DMA can't be stopped when transfering, so we can only
	 * diable it when it is out of DMA request.
	 */
	msc_writel(host, DMAC, 0);
}

static inline int request_need_stop(struct mmc_request *mrq)
{
	return mrq->stop ? 1 : 0;
}
static inline void ingenic_mmc_clk_onoff(struct ingenic_mmc_host *host, unsigned int on)
{
	if (on) {
		clk_prepare_enable(host->clk_cgu);
		clk_prepare_enable(host->clk_gate);
	} else {
		clk_disable_unprepare(host->clk_cgu);
		clk_disable_unprepare(host->clk_gate);
	}
}
static inline int check_error_status(struct ingenic_mmc_host *host, unsigned int status)
{
	if (status & ERROR_STAT) {
		dev_err(host->dev, "Error status->0x%08X: cmd=%d, state=%d\n",
		        status, host->cmd->opcode, host->state);
		return -1;
	}
	return 0;
}

static int ingenic_mmc_polling_status(struct ingenic_mmc_host *host, unsigned int status)
{
	unsigned int cnt = 100 * 1000 * 1000;
	while (!(msc_readl(host, STAT) & (status | ERROR_STAT))              \
	       && test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags) && (--cnt));

	if (unlikely(!cnt)) {
		dev_err(host->dev, "polling status(0x%08X) time out, "
		        "op=%d, status=0x%08X\n", status,
		        host->cmd->opcode, msc_readl(host, STAT));
		return -1;
	}
	if (unlikely(!test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags))) {
		dev_err(host->dev, "card remove while polling"
		        "status(0x%08X), op=%d\n", status, host->cmd->opcode);
		return -1;
	}
	if (msc_readl(host, STAT) & ERROR_STAT) {
		dev_err(host->dev, "polling status(0x%08X) error, "
		        "op=%d, status=0x%08X\n", status,
		        host->cmd->opcode, msc_readl(host, STAT));
		return -1;
	}

	return 0;
}

static void send_stop_command(struct ingenic_mmc_host *host)
{
	struct mmc_command *stop_cmd = host->mrq->stop;

	msc_writel(host, CMD, stop_cmd->opcode);
	msc_writel(host, ARG, stop_cmd->arg);
	msc_writel(host, CMDAT, CMDAT_BUSY | CMDAT_RESPONSE_R1);
	msc_writel(host, RESTO, 0xff);
	msc_writel(host, CTRL, CTRL_START_OP);

	if (ingenic_mmc_polling_status(host, STAT_END_CMD_RES)) {
		stop_cmd->error = -EIO;
	}
}
static void ingenic_mmc_command_done(struct ingenic_mmc_host *host, struct mmc_command *cmd)
{
	unsigned long res;

	if ((host->cmdat & CMDAT_RESPONSE_MASK) == CMDAT_RESPONSE_R2) {
		int i;
		res = msc_readl(host, RES);
		for (i = 0 ; i < 4 ; i++) {
			cmd->resp[i] = res << 24;
			res = msc_readl(host, RES);
			cmd->resp[i] |= res << 8;
			res = msc_readl(host, RES);
			cmd->resp[i] |= res >> 8;
		}
	} else {
		res = msc_readl(host, RES);
		cmd->resp[0] = res << 24;
		res = msc_readl(host, RES);
		cmd->resp[0] |= res << 8;
		res = msc_readl(host, RES);
		cmd->resp[0] |= res & 0xff;
	}

	clear_msc_irq(host, IFLG_END_CMD_RES);
}

static void ingenic_mmc_data_done(struct ingenic_mmc_host *host)
{
	struct mmc_data *data = host->data;

	if (data->error == 0) {
		data->bytes_xfered = (data->blocks * data->blksz);
	} else {
		ingenic_mmc_stop_dma(host);
		data->bytes_xfered = 0;
		dev_err(host->dev, "error when request done\n");
	}

	del_timer_sync(&host->request_timer);
	mmc_request_done(host->mmc, host->mrq);
}

/*------------------------End functional functions-------------------------*/

/*
 * State machine.
 *
 * The state machine is the manager of the mmc_request. It's triggered by
 * MSC interrupt and work in interrupt context.
 */
static void ingenic_mmc_state_machine(struct ingenic_mmc_host *host, unsigned int status)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_data *data = host->data;

	WARN_ON(host->double_enter++);
start:
	dev_dbg(host->dev, "enter state: %d\n", host->state);

	switch (host->state) {
	case STATE_IDLE:
		dev_warn(host->dev, "WARN: enter state machine with IDLE\n");
		break;

	case STATE_WAITING_RESP:
		if (!ingenic_mmc_check_pending(host, EVENT_CMD_COMPLETE)) {
			break;
		}
		if (unlikely(check_error_status(host, status) != 0)) {
			host->state = STATE_ERROR;
			clear_msc_irq(host, IFLG_CRC_RES_ERR
			              | IFLG_TIMEOUT_RES
			              | IFLG_END_CMD_RES);
			goto start;
		}
		ingenic_mmc_command_done(host, mrq->cmd);
		if (!data) {
			host->state = STATE_IDLE;
			del_timer_sync(&host->request_timer);
			mmc_request_done(host->mmc, host->mrq);
			break;
		}
		host->state = STATE_WAITING_DATA;
		break;

	case STATE_WAITING_DATA:
		if (!ingenic_mmc_check_pending(host, EVENT_DATA_COMPLETE)) {
			break;
		}
		if (unlikely(check_error_status(host, status) != 0)) {
			clear_msc_irq(host, IFLG_DATA_TRAN_DONE
			              | IFLG_CRC_READ_ERR
			              | IFLG_CRC_WRITE_ERR
			              | IFLG_TIMEOUT_READ);
			if (request_need_stop(host->mrq)) {
				send_stop_command(host);
			}
			host->state = STATE_ERROR;
			goto start;
		}

		if (request_need_stop(host->mrq)) {
			if (likely(msc_readl(host, STAT) & STAT_AUTO_CMD12_DONE)) {
				disable_msc_irq(host, IMASK_AUTO_CMD12_DONE);
				clear_msc_irq(host, IFLG_AUTO_CMD12_DONE);
				host->state = STATE_IDLE;
				ingenic_mmc_data_done(host);
			} else {
				enable_msc_irq(host, IMASK_AUTO_CMD12_DONE);
				if (msc_readl(host, STAT) & STAT_AUTO_CMD12_DONE) {
					disable_msc_irq(host, IMASK_AUTO_CMD12_DONE);
					clear_msc_irq(host, IFLG_AUTO_CMD12_DONE);
					host->state = STATE_IDLE;
					ingenic_mmc_data_done(host);
				} else {
					host->state = STATE_SENDING_STOP;
				}
			}
		} else {
			host->state = STATE_IDLE;
			ingenic_mmc_data_done(host);
		}
		break;

	case STATE_SENDING_STOP:
		if (!ingenic_mmc_check_pending(host, EVENT_STOP_COMPLETE)) {
			break;
		}
		host->state = STATE_IDLE;
		ingenic_mmc_data_done(host);
		break;

	case STATE_ERROR:
		if (host->state == STATE_WAITING_DATA) {
			host->data->error = -1;
		}
		host->cmd->error = -1;

		if (data) {
			data->bytes_xfered = 0;
			/* Whether should we stop DMA here? */
		}
		del_timer_sync(&host->request_timer);
		host->state = STATE_IDLE;
		mmc_request_done(host->mmc, host->mrq);
		break;
	}

	dev_dbg(host->dev, "exit state: %d\n", host->state);
	host->double_enter--;
}

static irqreturn_t ingenic_mmc_thread_handle(int irq, void *dev_id)
{
	struct ingenic_mmc_host *host = (struct ingenic_mmc_host *)dev_id;
	unsigned int iflg, imask, pending, status;

start:
	iflg = msc_readl(host, IFLG);
	imask = msc_readl(host, IMASK);
	pending = iflg & ~imask;
	status = msc_readl(host, STAT);
	dev_dbg(host->dev, "%s: iflg-0x%08X imask-0x%08X status-0x%08X\n",
	        __func__, iflg, imask, status);

	if (!pending) {
		goto out;
	} else if (pending & IFLG_SDIO) {
		mmc_signal_sdio_irq(host->mmc);
		goto out;
	} else if (pending & ERROR_IFLG) {
		unsigned int mask = ERROR_IFLG;

		dev_dbg(host->dev, "%s: iflg-0x%08X imask-0x%08X status-0x%08X\n",
		        __func__, iflg, imask, status);

		dev_dbg(host->dev, "err%d cmd%d iflg%08X status%08X\n",
		        host->state, host->cmd ? host->cmd->opcode : -1, iflg, status);

		if (host->state == STATE_WAITING_RESP) {
			mask |= IMASK_END_CMD_RES;
		} else if (host->state == STATE_WAITING_DATA) {
			mask |= IMASK_WR_ALL_DONE | IMASK_DMA_DATA_DONE;
		}

		clear_msc_irq(host, mask);
		disable_msc_irq(host, mask);

		/*
		 * It seems that cmd53 CRC error occurs frequently
		 * at 50mHz clk, but it disappear at 40mHz. In case of
		 * it happens, we add retry here to try to fix the error.
		 */
		if ((host->cmd->opcode == 53)
		    && (status & STAT_CRC_READ_ERROR)) {
			dev_err(host->dev, "cmd53 crc error, retry.\n");
			host->cmd->error = -1;
			host->cmd->retries = 1;
			host->data->bytes_xfered = 0;
			del_timer_sync(&host->request_timer);
			host->state = STATE_IDLE;
			mmc_request_done(host->mmc, host->mrq);
			goto out;
		}
		host->state = STATE_ERROR;
		ingenic_mmc_state_machine(host, status);
		goto out;

	} else if (pending & IFLG_END_CMD_RES) {
		ingenic_mmc_set_pending(host, EVENT_CMD_COMPLETE);
		disable_msc_irq(host, IMASK_END_CMD_RES |       \
		                IMASK_CRC_RES_ERR | IMASK_TIME_OUT_RES);
		ingenic_mmc_state_machine(host, status);
	} else if (pending & IFLG_WR_ALL_DONE) {
		ingenic_mmc_set_pending(host, EVENT_DATA_COMPLETE);
		clear_msc_irq(host, IFLG_WR_ALL_DONE
		              | IFLG_DMAEND
		              | IFLG_DATA_TRAN_DONE
		              | IFLG_PRG_DONE);
		disable_msc_irq(host, IMASK_WR_ALL_DONE | IMASK_CRC_WRITE_ERR);
		ingenic_mmc_state_machine(host, status);

	} else if (pending & IFLG_DMA_DATA_DONE) {
		ingenic_mmc_set_pending(host, EVENT_DATA_COMPLETE);
		clear_msc_irq(host, IFLG_DATA_TRAN_DONE | IFLG_DMAEND |
		              IFLG_DMA_DATA_DONE);
		disable_msc_irq(host, IMASK_DMA_DATA_DONE | IMASK_CRC_READ_ERR);
		ingenic_mmc_state_machine(host, status);
	} else if (pending & IFLG_AUTO_CMD12_DONE) {
		ingenic_mmc_set_pending(host, EVENT_STOP_COMPLETE);
		clear_msc_irq(host, IFLG_AUTO_CMD12_DONE);
		disable_msc_irq(host, IMASK_AUTO_CMD12_DONE);
		ingenic_mmc_state_machine(host, status);

	} else {
		dev_warn(host->dev, "state-%d: Nothing happens?!\n", host->state);
	}

	/*
	 * Check if the status has already changed. If so, goto start so that
	 * we can avoid an interrupt.
	 */
	if (status != msc_readl(host, STAT)) {
		goto start;
	}

out:
	return IRQ_HANDLED;
}
/*--------------------------End state machine------------------------------*/

/*
 * DMA handler.
 *
 * Descriptor DMA transfer that can handle scatter gather list directly
 * without bounce buffer which may cause a big deal of memcpy.
 */
static inline void sg_to_desc(struct scatterlist *sgentry, struct desc_hd *dhd)
{
	dhd->dma_desc->da = sg_phys(sgentry);
	dhd->dma_desc->len = sg_dma_len(sgentry);
	dhd->dma_desc->dcmd = DMACMD_LINK;
}

static void ingenic_mmc_submit_dma(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	int i = 0;
	struct scatterlist *sgentry;
	struct desc_hd *dhd = &(host->decshds[0]);

	dma_map_sg(host->dev, data->sg, data->sg_len,
	           data->flags & MMC_DATA_WRITE
	           ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

	for_each_sg(data->sg, sgentry, data->sg_len, i) {
		sg_to_desc(sgentry, dhd);
		if ((data->sg_len - i) > 1) {
			if (unlikely(dhd->next == NULL)) {
				dev_err(host->dev, "dhd->next == NULL\n");
			} else {
				dhd->dma_desc->nda = dhd->next->dma_desc_phys_addr;
				dhd = dhd->next;
			}
		}
	}

	dma_unmap_sg(host->dev, data->sg, data->sg_len,
	             data->flags & MMC_DATA_WRITE
	             ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

	dhd->dma_desc->dcmd |= DMACMD_ENDI;
	dhd->dma_desc->dcmd &= ~DMACMD_LINK;
}

static inline unsigned int get_incr(unsigned int dma_len)
{
	unsigned int incr = 0;

	BUG_ON(!dma_len);
#if 0
	/*
	 * BUG here!
	 */
	switch (dma_len) {
#define _CASE(S,D) case S: incr = D; break
		_CASE(1 ... 31, 0);
		_CASE(32 ... 63, 1);
	default:
		incr = 2;
		break;
#undef _CASE
	}
#else
	incr = 2;
#endif
	return incr;
}

/* #define PERFORMANCE_DMA */
static inline void ingenic_mmc_dma_start(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	dma_addr_t dma_addr = sg_phys(data->sg);
	unsigned int dma_len = sg_dma_len(data->sg);
	unsigned int dmac;

#ifdef PERFORMANCE_DMA
	dmac = (get_incr(dma_len) << DMAC_INCR_SHF) | DMAC_DMAEN | DMAC_MODE_SEL;
#else
	dmac = (get_incr(dma_len) << DMAC_INCR_SHF) | DMAC_DMAEN;
#endif

	if ((dma_addr & 0x3) || (dma_len & 0x3)) {
		dmac |= DMAC_ALIGNEN;
		if (dma_addr & 0x3) {
			dmac |= (dma_addr % 4) << DMAC_AOFST_SHF;
		}
	}
	msc_writel(host, DMANDA, host->decshds[0].dma_desc_phys_addr);
	msc_writel(host, DMAC, dmac);
}

/*----------------------------End DMA handler------------------------------*/

/*
 * PIO transfer mode.
 *
 * Functions of PIO read/write mode that can handle 1, 2 or 3 bytes transfer
 * even though the FIFO register is 32-bits width.
 * It's better just used for test.
 */
static int wait_cmd_response(struct ingenic_mmc_host *host)
{
	if (ingenic_mmc_polling_status(host, STAT_END_CMD_RES) < 0) {
		dev_err(host->dev, "PIO mode: command response error\n");
		return -1;
	}
	msc_writel(host, IFLG, IFLG_END_CMD_RES);
	return 0;
}

static void do_pio_read(struct ingenic_mmc_host *host,
                        unsigned int *addr, unsigned int cnt)
{
	int i = 0;
	unsigned int status = 0;

	for (i = 0; i < cnt / 4; i++) {
		while (((status = msc_readl(host, STAT))
		        & STAT_DATA_FIFO_EMPTY)
		       && test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags));

		if (!test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags)) {
			host->data->error = -ENOMEDIUM;
			dev_err(host->dev,
			        "PIO mode: card remove while reading\n");
			return;
		}
		if (check_error_status(host, status)) {
			host->data->error = -1;
			return;
		}
		*addr++ = msc_readl(host, RXFIFO);
	}

	/*
	 * These codes handle the last 1, 2 or 3 bytes transfer.
	 */
	if (cnt & 3) {
		u32 n = cnt & 3;
		u32 data = msc_readl(host, RXFIFO);
		u8 *p = (u8 *)addr;

		while (n--) {
			*p++ = data;
			data >>= 8;
		}
	}
}

static void do_pio_write(struct ingenic_mmc_host *host,
                         unsigned int *addr, unsigned int cnt)
{
	int i = 0;
	unsigned int status = 0;

	for (i = 0; i < (cnt / 4); i++) {
		while (((status = msc_readl(host, STAT))
		        & STAT_DATA_FIFO_FULL)
		       && test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags));

		if (!test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags)) {
			host->data->error = -ENOMEDIUM;
			dev_err(host->dev,
			        "PIO mode: card remove while writing\n");
			break;
		}
		if (check_error_status(host, status)) {
			host->data->error = -1;
			return;
		}
		msc_writel(host, TXFIFO, *addr++);
	}

	/*
	 * These codes handle the last 1, 2 or 3 bytes transfer.
	 */
	if (cnt & 3) {
		u32 data = 0;
		u8 *p = (u8 *)addr;

		for (i = 0; i < (cnt & 3); i++) {
			data |= *p++ << (8 * i);
		}

		msc_writel(host, TXFIFO, data);
	}
}

static inline void pio_trans_start(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	unsigned int *addr = sg_virt(data->sg);
	unsigned int cnt = sg_dma_len(data->sg);

	if (data->flags & MMC_DATA_WRITE) {
		do_pio_write(host, addr, cnt);
	} else {
		do_pio_read(host, addr, cnt);
	}
}

static void pio_trans_done(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	if (data->error == 0) {
		data->bytes_xfered = data->blocks * data->blksz;
	} else {
		data->bytes_xfered = 0;
	}

	if (host->mrq->stop) {
		if (ingenic_mmc_polling_status(host, STAT_AUTO_CMD12_DONE) < 0) {
			data->error = -EIO;
		}
	}

	if (data->flags & MMC_DATA_WRITE) {
		if (ingenic_mmc_polling_status(host, STAT_PRG_DONE) < 0) {
			data->error = -EIO;
		}
		clear_msc_irq(host, IFLG_PRG_DONE);
	} else {
		if (ingenic_mmc_polling_status(host, STAT_DATA_TRAN_DONE) < 0) {
			data->error = -EIO;
		}
		clear_msc_irq(host, IFLG_DATA_TRAN_DONE);
	}
}

/*-------------------------End PIO transfer mode---------------------------*/

/*
 * Achieve mmc_request here.
 */
static void ingenic_mmc_data_pre(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	unsigned int nob = data->blocks;
	unsigned long cmdat, imsk;

	msc_writel(host, RDTO, 0xffffff);
	msc_writel(host, NOB, nob);
	msc_writel(host, BLKLEN, data->blksz);
	cmdat = CMDAT_DATA_EN;

	msc_writel(host, CMDAT, CMDAT_DATA_EN);

	if (data->flags & MMC_DATA_WRITE) {
		cmdat |= CMDAT_WRITE_READ;
		imsk = IMASK_WR_ALL_DONE | IMASK_CRC_WRITE_ERR;
	} else if (data->flags & MMC_DATA_READ) {
		cmdat &= ~CMDAT_WRITE_READ;
		imsk = IMASK_DMA_DATA_DONE
		       | IMASK_TIME_OUT_READ
		       | IMASK_CRC_READ_ERR;
	} else {
		dev_err(host->dev, "data direction confused\n");
		BUG_ON(1);
	}
	host->cmdat |= cmdat;

	if (!is_pio_mode(host)) {
		ingenic_mmc_submit_dma(host, data);
		clear_msc_irq(host, IFLG_PRG_DONE);
		enable_msc_irq(host, imsk);
	}
}

static void ingenic_mmc_data_start(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	if (is_pio_mode(host)) {
		pio_trans_start(host, data);
		pio_trans_done(host, data);
		del_timer_sync(&host->request_timer);
		if (!(host->pdata->pio_mode)) {
			disable_pio_mode(host);
		}
		mmc_request_done(host->mmc, host->mrq);
	} else {
		ingenic_mmc_dma_start(host, data);
	}
}

static void ingenic_mmc_command_start(struct ingenic_mmc_host *host, struct mmc_command *cmd)
{
	unsigned long cmdat = 0;
	unsigned long imsk;

	if (cmd->flags & MMC_RSP_BUSY) {
		cmdat |= CMDAT_BUSY;
	}
	if (request_need_stop(host->mrq)) {
		cmdat |= CMDAT_AUTO_CMD12;
	}

	switch (mmc_resp_type(cmd)) {
#define _CASE(S,D) case MMC_RSP_##S: cmdat |= CMDAT_RESPONSE_##D; break
		_CASE(R1, R1);  /* r1 = r5,r6,r7 */
		_CASE(R1B, R1);
		_CASE(R2, R2);
		_CASE(R3, R3);  /* r3 = r4 */
	default:
		break;
#undef _CASE
	}
	host->cmdat |= cmdat;
	if (!is_pio_mode(host)) {
		imsk = IMASK_TIME_OUT_RES | IMASK_END_CMD_RES;
		enable_msc_irq(host, imsk);
		host->state = STATE_WAITING_RESP;
	}
	msc_writel(host, CMD, cmd->opcode);
	msc_writel(host, ARG, cmd->arg);
	msc_writel(host, CMDAT, host->cmdat);
	msc_writel(host, CTRL, CTRL_START_OP);
	if (is_pio_mode(host)) {
		if (wait_cmd_response(host) < 0) {
			cmd->error = -ETIMEDOUT;
			del_timer_sync(&host->request_timer);
			mmc_request_done(host->mmc, host->mrq);
			return;
		}
		ingenic_mmc_command_done(host, host->cmd);
		if (!host->data) {
			del_timer_sync(&host->request_timer);
			mmc_request_done(host->mmc, host->mrq);
		}
	}
}

static void ingenic_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct ingenic_mmc_host *host = mmc_priv(mmc);
	if (!test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags)) {
		dev_vdbg(host->dev, "No card present\n");
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	/*
	 * It means that this request may flush cache in interrupt context.
	 * It never happens in design, but we add BUG_ON here to prevent it.
	 */
	if ((host->state != STATE_IDLE) && (mrq->data != NULL)) {
		dev_warn(host->dev, "operate in non-idle state\n");
		WARN_ON(1);
	}

	host->mrq = mrq;
	host->data = mrq->data;
	host->cmd = mrq->cmd;
	if (host->data)
		dev_dbg(host->dev, "op:%d arg:0x%08X sz:%uk\n",
		        host->cmd->opcode, host->cmd->arg,
		        host->data->blocks >> 1);
	else {
		dev_dbg(host->dev, "op:%d\n", host->cmd->opcode);
	}

	host->cmdat = host->cmdat_def;
	if (host->data) {
		if ((host->data->sg_len == 1)
		    && (sg_dma_len(host->data->sg)) < PIO_THRESHOLD) {
			enable_pio_mode(host);
		}

		ingenic_mmc_data_pre(host, host->data);
	}
	/*
	 * We would get mmc_request_done at last, unless some terrible error
	 * occurs such as intensity rebounding of VDD, that maybe result in
	 * no action to complete the request.
	 */
	host->timeout_cnt = 0;
	mod_timer(&host->request_timer, jiffies +
	          msecs_to_jiffies(TIMEOUT_PERIOD));
	ingenic_mmc_command_start(host, host->cmd);
	if (host->data) {
		ingenic_mmc_data_start(host, host->data);

	}
	if (unlikely(test_and_clear_bit(INGENIC_MMC_CARD_NEED_INIT, &host->flags))) {
		host->cmdat_def &= ~CMDAT_INIT;
	}
}

static void ingenic_mmc_request_timeout(struct timer_list *t)
{
	struct ingenic_mmc_host *host = from_timer(host, t, request_timer);
	unsigned int status = msc_readl(host, STAT);
	if (host->timeout_cnt++ < (3000 / TIMEOUT_PERIOD)) {
		dev_warn(host->dev, "timeout %dms op:%d %s sz:%d state:%d "
		         "STAT:0x%08X DMALEN:0x%08X blks:%d/%d clk:%s\n",
		         host->timeout_cnt * TIMEOUT_PERIOD,
		         host->cmd->opcode,
		         host->data
		         ? (host->data->flags & MMC_DATA_WRITE ? "w" : "r")
		         : "",
		         host->data ? host->data->blocks << 9 : 0,
		         host->state,
		         status,
		         msc_readl(host, DMALEN),
		         msc_readl(host, SNOB),
		         msc_readl(host, NOB),
		         __clk_is_enabled(host->clk_cgu) ? "enable" : "disable");
		mod_timer(&host->request_timer, jiffies +
		          msecs_to_jiffies(TIMEOUT_PERIOD));
		return;

	} else if (host->timeout_cnt++ < (60000 / TIMEOUT_PERIOD)) {
		mod_timer(&host->request_timer, jiffies +
		          msecs_to_jiffies(TIMEOUT_PERIOD));
		return;
	}

	dev_err(host->dev, "request time out, op=%d arg=0x%08X, "
	        "sz:%dB state=%d, status=0x%08X, pending=0x%08X, nr_desc=%d\n",
	        host->cmd->opcode, host->cmd->arg,
	        host->data ? host->data->blocks << 9 : -1,
	        host->state, status, (u32)host->pending_events,
	        host->data ? host->data->sg_len : 0);
	ingenic_mmc_dump_reg(host);

	if (host->data) {
		int i;
		dev_err(host->dev, "Descriptor dump:\n");
		for (i = 0; i < MAX_SEGS; i++) {
			unsigned int *desc = (unsigned int *)host->decshds[i].dma_desc;
			dev_err(host->dev, "\t%03d\t nda=%08X da=%08X len=%08X dcmd=%08X\n",
			        i, *desc, *(desc + 1), *(desc + 2), *(desc + 3));
		}
		dev_err(host->dev, "\n");
	}

	if (host->mrq) {
		if (request_need_stop(host->mrq)) {
			send_stop_command(host);
		}
		host->cmd->error = -ENOMEDIUM;
		host->state = STATE_IDLE;
		mmc_request_done(host->mmc, host->mrq);
	}
}

/*---------------------------End mmc_request-------------------------------*/

/*
 * Card insert and remove handler.
 */
static irqreturn_t ingenic_mmc_detect_handler(int irq, void *dev_id)
{
	struct ingenic_mmc_host *host = (struct ingenic_mmc_host *)dev_id;

	disable_irq_nosync(irq);
	mod_timer(&host->detect_timer, jiffies + msecs_to_jiffies(200));

	return IRQ_HANDLED;
}

static int get_pin_status(struct gpio_desc *gd)
{
	if (IS_ERR(gd)) {
		return PTR_ERR(gd);
	}

	return gpiod_get_raw_value(gd);
}

static void set_pin_status(struct gpio_desc *gd, int enable)
{
	if (IS_ERR(gd)) {
		return;
	}

	gpiod_set_raw_value(gd, enable);
}

static void ingenic_mmc_detect(struct timer_list *t)
{
	struct ingenic_mmc_host *host = from_timer(host, t, detect_timer);
	bool present;
	bool present_old;
	static int irq_disable_count;

	present = get_pin_status(host->pdata->gpio->cd);
	present_old = test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);

	if ((present != present_old) || (present_old && host->mmc->card)) {
		if (present && present_old) {
			dev_warn(host->dev, "rapidly remove\n");
		} else
			dev_notice(host->dev, "card %s, state=%d\n",
			           present ? "inserted" : "removed", host->state);

		if (!present || present_old) {
			clear_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
			if (!irq_disable_count) {
				disable_irq_nosync(host->irq);
				irq_disable_count = 1;
			}
			ingenic_mmc_reset(host);

			if (host->mrq && (host->state > STATE_IDLE)) {
				host->cmd->error = -ENOMEDIUM;
				if (host->data) {
					host->data->bytes_xfered = 0;
					ingenic_mmc_stop_dma(host);
				}
				del_timer_sync(&host->request_timer);
				mmc_request_done(host->mmc, host->mrq);
				host->state = STATE_IDLE;
			}
			mmc_detect_change(host->mmc, 0);
		} else {
			if (irq_disable_count) {
				enable_irq(host->irq);
				irq_disable_count = 0;
			}
			set_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
			/*
			 * spin_lock() here may case recursion,
			 * so discard the clk operation.
			 */
			ingenic_mmc_clk_onoff(host, 1);
			mmc_detect_change(host->mmc, msecs_to_jiffies(1000));
		}

		if (!test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags)) {
			ingenic_mmc_clk_onoff(host, 0);
		}
	}

	enable_irq(gpiod_to_irq(host->pdata->gpio->cd));
}

/**
 *  ingenic_mmc_manual_detect - insert or remove card manually
 *  @index: host->index, namely the index of the controller.
 *  @on: 1 means insert card, 0 means remove card.
 *
 *  This functions will be called by manually card-detect driver such as
 *  wifi. To enable this mode you can set value pdata.removal = MANUAL.
 */
int ingenic_mmc_manual_detect(int index, int on)
{
	struct ingenic_mmc_host *host;
	struct list_head *pos;

	list_for_each(pos, &manual_list) {
		host = list_entry(pos, struct ingenic_mmc_host, list);
		if (host->index == index) {
			break;
		} else {
			host = NULL;
		}
	}

	if (!host) {
		printk("no manual card detect\n");
		return -1;
	}

	if (on) {
		dev_vdbg(host->dev, "card insert manually\n");
		set_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
#ifdef CLK_CTRL
		ingenic_mmc_clk_onoff(host, 1);
#endif
		mmc_detect_change(host->mmc, 0);
	} else {
		dev_vdbg(host->dev, "card remove manually\n");
		clear_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
		mmc_detect_change(host->mmc, 0);
#ifdef CLK_CTRL
		ingenic_mmc_clk_onoff(host, 0);
#endif
	}

	return 0;
}
EXPORT_SYMBOL(ingenic_mmc_manual_detect);

/**
 *  ingenic_mmc_clk_ctrl - enable or disable msc clock gate
 *  @index: host->index, namely the index of the controller.
 *  @on: 1-enable msc clock gate, 0-disable msc clock gate.
 */
int ingenic_mmc_clk_ctrl(int index, int on)
{
	struct ingenic_mmc_host *host;
	struct list_head *pos;

#ifdef CLK_CTRL
	list_for_each(pos, &manual_list) {
		host = list_entry(pos, struct ingenic_mmc_host, list);
		if (host->index == index) {
			break;
		} else {
			host = NULL;
		}
	}

	if (!host) {
		printk("no manual card detect\n");
		return -1;
	}
	ingenic_mmc_clk_onoff(host, on);
#endif
	return 0;
}
EXPORT_SYMBOL(ingenic_mmc_clk_ctrl);

/*
 * for module driver
 */
int jzmmc_manual_detect(int index, int on)
{
	return ingenic_mmc_manual_detect(index, on);
}
EXPORT_SYMBOL(jzmmc_manual_detect);

int jzmmc_clk_ctrl(int index, int on)
{
	return ingenic_mmc_clk_ctrl(index, on);
}
EXPORT_SYMBOL(jzmmc_clk_ctrl);
/*-------------------End card insert and remove handler--------------------*/

/*
 * Other mmc_ops except request.
 */
static inline void ingenic_mmc_power_on(struct ingenic_mmc_host *host)
{
	dev_vdbg(host->dev, "power_on\n");

	if (!IS_ERR(host->power)) {
		/* if(!regulator_is_enabled(host->power)) */
		/*  regulator_enable(host->power); */

	} else if (host->pdata->gpio) {
		set_pin_status(host->pdata->gpio->pwr, 1);
	}
}

static inline void ingenic_mmc_power_off(struct ingenic_mmc_host *host)
{
	dev_vdbg(host->dev, "power_off\n");

	if (!IS_ERR(host->power)) {
		/* if(regulator_is_enabled(host->power)) */
		/*  regulator_disable(host->power); */

	} else if (host->pdata->gpio) {
		set_pin_status(host->pdata->gpio->pwr, 0);
	}
}

static int ingenic_mmc_get_read_only(struct mmc_host *mmc)
{
	struct ingenic_mmc_host *host = mmc_priv(mmc);
	int ret = 0;

	dev_vdbg(host->dev, "get card ro\n");
	if (host->pdata->gpio != NULL) {
		ret = get_pin_status(host->pdata->gpio->wp);
	}

	return ret < 0 ? 0 : ret;
}

static int ingenic_mmc_get_card_detect(struct mmc_host *mmc)
{
	struct ingenic_mmc_host *host = mmc_priv(mmc);
	int ret = -1;

	dev_vdbg(host->dev, "get card present\n");
	if ((host->pdata->removal == NONREMOVABLE)
	    || (host->pdata->removal == MANUAL)) {
		return test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
	}

	if (host->pdata->gpio != NULL) {
		ret = get_pin_status(host->pdata->gpio->cd);
	}

	return ret < 0 ? 1 : ret;
}

static void ingenic_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ingenic_mmc_host *host = mmc_priv(mmc);

	/*
	 * The max bus width is set in the platformdata->capacity,
	 * MMC_CAP_4_BIT_DATA: Can the host do 4 bit transfers
	 * MMC_CAP_8_BIT_DATA: Can the host do 8 bit transfers
	 */

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		host->cmdat_def &= ~CMDAT_BUS_WIDTH_MASK;
		host->cmdat_def |= CMDAT_BUS_WIDTH_1BIT;
		break;
	case MMC_BUS_WIDTH_4:
		host->cmdat_def &= ~CMDAT_BUS_WIDTH_MASK;
		host->cmdat_def |= CMDAT_BUS_WIDTH_4BIT;
		break;
	case MMC_BUS_WIDTH_8:
		host->cmdat_def &= ~CMDAT_BUS_WIDTH_MASK;
		host->cmdat_def |= CMDAT_BUS_WIDTH_8BIT;
		break;
	}

	if (ios->clock) {
		unsigned int clk_set = 0, clkrt = 0;
		unsigned int clk_want = ios->clock;
		unsigned int lpm = 0;

		ingenic_mmc_clk_onoff(host, 0);
		if (clk_want > 3000000) {
			clk_set_rate(host->clk_cgu, ios->clock);
		} else {
			clk_set_rate(host->clk_cgu, CLK_RATE);
		}
		/*clk_get_rate is permanently 24000000 on board_4785_fpga*/
		clk_set = clk_get_rate(host->clk_cgu);

		while (clk_want < clk_set) {
			clkrt++;
			clk_set >>= 1;
		}
		/* discard this warning on board 4785 fpga */
		if ((clk_want > 3000000) && clkrt) {
			dev_err(host->dev, "CLKRT must be set to 0 "
			        "when MSC works during normal r/w: "
			        "ios->clock=%d clk_want=%d "
			        "clk_set=%d clkrt=%X,\n",
			        ios->clock, clk_want, clk_set, clkrt);
			WARN_ON(1);
		}

		if (clkrt > 7) {
			dev_err(host->dev, "invalid value of CLKRT: "
			        "ios->clock=%d clk_want=%d "
			        "clk_set=%d clkrt=%X,\n",
			        ios->clock, clk_want, clk_set, clkrt);
			WARN_ON(1);
			return;
		}
		if (!clkrt)
			dev_vdbg(host->dev, "clk_want: %u, clk_set: %luHz\n",
			         ios->clock, clk_get_rate(host->clk_cgu));

		ingenic_mmc_clk_onoff(host, 1);
		msc_writel(host, CLKRT, clkrt);

		/* sample immediately at clk rising edge */
		if (clk_set > 25000000) {
			lpm = (0x2 << LPM_DRV_SEL_SHF);
		}

		if (host->pdata->sdio_clk) {
			msc_writel(host, LPM, lpm);
			msc_writel(host, CTRL, CTRL_CLOCK_START);
		} else {
			lpm |= LPM_LPM;
			msc_writel(host, LPM, lpm);
		}
	}

	switch (ios->power_mode) {
	case MMC_POWER_ON:
	case MMC_POWER_UP:
		host->cmdat_def |= CMDAT_INIT;
		set_bit(INGENIC_MMC_CARD_NEED_INIT, &host->flags);
		ingenic_mmc_power_on(host);
		break;
	case MMC_POWER_OFF:
		ingenic_mmc_power_off(host);
		break;
	default:
		break;
	}
}

static void ingenic_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct ingenic_mmc_host *host = mmc_priv(mmc);

	if (enable) {
		enable_msc_irq(host, IMASK_SDIO);
	} else {
		clear_msc_irq(host, IFLG_SDIO);
		disable_msc_irq(host, IMASK_SDIO);
	}
}

static const struct mmc_host_ops ingenic_mmc_ops = {
	.request        = ingenic_mmc_request,
	.set_ios        = ingenic_mmc_set_ios,
	.get_ro         = ingenic_mmc_get_read_only,
	.get_cd         = ingenic_mmc_get_card_detect,
	.enable_sdio_irq    = ingenic_mmc_enable_sdio_irq,
};

/*--------------------------End other mmc_ops------------------------------*/
static ssize_t ingenic_mmc_present_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
	struct ingenic_mmc_host *host = dev_get_drvdata(dev);
	ssize_t count = 0;

	if (test_bit(INGENIC_MMC_CARD_PRESENT, &host->flags)) {
		count = sprintf(buf, "Y\n");
	} else {
		count = sprintf(buf, "N\n");
	}

	return count;
}

static ssize_t ingenic_mmc_present_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
	struct ingenic_mmc_pdata *pdata = dev->platform_data;
	struct ingenic_mmc_host *host = dev_get_drvdata(dev);

	if ((buf == NULL) || (pdata->removal != NONREMOVABLE)) {
		dev_err(host->dev, "can't set present\n");
		return count;
	}

	if (strncmp(buf, "INSERT", 6) == 0) {
		dev_info(host->dev, "card insert via sysfs\n");
		set_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
		mmc_detect_change(host->mmc, 0);

	} else if (strncmp(buf, "REMOVE", 6) == 0) {
		dev_info(host->dev, "card remove via sysfs\n");
		clear_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
		mmc_detect_change(host->mmc, 0);
		ingenic_mmc_reset(host);

	} else {
		dev_err(host->dev, "set present error, "
		        "the argument can't be recognised\n");
	}

	return count;
}

static DEVICE_ATTR(present, S_IWUSR | S_IRUSR,
                   ingenic_mmc_present_show, ingenic_mmc_present_store);

static struct attribute *ingenic_mmc_attributes[] = {
	&dev_attr_present.attr,
	NULL
};

static const struct attribute_group ingenic_mmc_attr_group = {
	.attrs = ingenic_mmc_attributes,
};

/*-------------------------End Sysfs interface-----------------------------*/

/*
 * Platform driver and initialization.
 */
static void __init ingenic_mmc_host_init(struct ingenic_mmc_host *host, struct mmc_host *mmc)
{
	struct ingenic_mmc_pdata *pdata = host->pdata;

	mmc->ops = &ingenic_mmc_ops;
	mmc->f_min = 200000;
	mmc->ocr_avail = pdata->ocr_avail;
	mmc->pm_flags |= pdata->pm_flags;
#ifdef CONFIG_MMC_BLOCK_BOUNCE
	mmc->max_blk_count = 65535;
	mmc->max_req_size = PAGE_SIZE * 16;
#else
	mmc->max_segs = MAX_SEGS;
	mmc->max_blk_count = 4096;
	mmc->max_req_size = 4096 * 512;
#endif
	mmc->max_blk_size = 512;
	mmc->max_seg_size = mmc->max_req_size;

	host->mmc = mmc;
	timer_setup(&host->request_timer, ingenic_mmc_request_timeout, 0);

	mmc_of_parse(mmc);

	mmc_add_host(mmc);
}

static int __init ingenic_mmc_dma_init(struct ingenic_mmc_host *host)
{
	struct sdma_desc *next_desc;
	unsigned char i = 0;
	void *desc_mem;
	void __iomem *desc;

	desc_mem = (struct sdma_desc *)get_zeroed_page(GFP_KERNEL);
	if (desc_mem == NULL) {
		dev_err(host->dev, "get DMA descriptor memory error\n");
		return -ENODEV;
	}

	desc = devm_ioremap(host->dev, virt_to_phys(desc_mem), PAGE_SIZE);
	if (desc == NULL) {
		dev_err(host->dev, "remap descriptor memory error\n");
		kfree(desc_mem);
		return -ENODEV;
	}

	host->decshds[0].dma_desc = (struct sdma_desc *)desc;
	next_desc = host->decshds[0].dma_desc;

	for (i = 0; i < MAX_SEGS; ++i) {
		struct desc_hd *dhd = &host->decshds[i];
		dhd->dma_desc = next_desc;
		dhd->dma_desc_phys_addr = CPHYSADDR((unsigned long)dhd->dma_desc);
		next_desc += 1;
		dhd->next = dhd + 1;
	}
	host->decshds[MAX_SEGS - 1].next = NULL;
	return 0;
}

static int __init ingenic_mmc_msc_init(struct ingenic_mmc_host *host)
{
	ingenic_mmc_reset(host);
	host->cmdat_def = CMDAT_RTRG_EQUALT_16 | CMDAT_TTRG_LESS_16 |   \
	                  CMDAT_BUS_WIDTH_1BIT;

	return devm_request_threaded_irq(host->dev, host->irq, NULL,
	                                 ingenic_mmc_thread_handle, IRQF_ONESHOT, dev_name(host->dev), host);
}

static int __init ingenic_mmc_gpio_init(struct ingenic_mmc_host *host)
{
	struct card_gpio *card_gpio = host->pdata->gpio;
	int ret = 0;

	card_gpio->cd = devm_gpiod_get_optional(host->dev, "ingenic,cd", GPIOD_IN);
	if (IS_ERR(card_gpio->cd)) {
		dev_err(host->dev, "get ingenic,cd failed with error %ld\n",
		        PTR_ERR(card_gpio->cd));
	}

	card_gpio->wp = devm_gpiod_get_optional(host->dev, "ingenic,wp", GPIOD_IN);
	if (IS_ERR(card_gpio->wp)) {
		dev_err(host->dev, "get ingenic,wp failed with error %ld\n",
		        PTR_ERR(card_gpio->wp));
	}

	card_gpio->rst = devm_gpiod_get_optional(host->dev, "ingenic,rst", GPIOD_OUT_HIGH);
	if (IS_ERR(card_gpio->rst)) {
		dev_err(host->dev, "get ingenic,rst failed with error %ld\n",
		        PTR_ERR(card_gpio->rst));
	}

	card_gpio->pwr = devm_gpiod_get_optional(host->dev, "ingenic,pwr", GPIOD_OUT_LOW);
	if (IS_ERR(card_gpio->pwr)) {
		dev_err(host->dev, "get ingenic,pwr failed with error %ld\n",
		        PTR_ERR(card_gpio->pwr));
	}

	switch (host->pdata->removal) {
	case NONREMOVABLE:
		break;
	case REMOVABLE:
		if (card_gpio->cd) {
			timer_setup(&host->detect_timer, ingenic_mmc_detect, 0);
			ret = devm_request_irq(host->dev, gpiod_to_irq(card_gpio->cd),
			                       ingenic_mmc_detect_handler,
			                       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			                       "mmc-insert-detect", host);
			if (ret) {
				dev_err(host->dev, "request detect irq-%d fail\n",
				        gpiod_to_irq(card_gpio->cd));
				break;
			}

			if (!timer_pending(&host->detect_timer)) {
				disable_irq_nosync(gpiod_to_irq(card_gpio->cd));
				mod_timer(&host->detect_timer, jiffies);
			}
		} else {
			dev_err(host->dev, "card-detect pin must be valid "
			        "when host->pdata->removal = 1, errno=%ld\n",
			        PTR_ERR(card_gpio->cd));
		}

		break;
	case MANUAL:
		list_add(&(host->list), &manual_list);
		break;
	default:
		set_bit(INGENIC_MMC_CARD_PRESENT, &host->flags);
		break;
	}

	return ret;
}

static void ingenic_mmc_gpio_deinit(struct ingenic_mmc_host *host)
{
	struct card_gpio *card_gpio = host->pdata->gpio;

	if (card_gpio) {
		if (card_gpio->cd) {
			devm_gpiod_put(host->dev, card_gpio->cd);
			card_gpio->cd = NULL;
		}
		if (card_gpio->wp) {
			devm_gpiod_put(host->dev, card_gpio->wp);
			card_gpio->wp = NULL;
		}
		if (card_gpio->pwr) {
			devm_gpiod_put(host->dev, card_gpio->pwr);
			card_gpio->pwr = NULL;
		}
		if (card_gpio->rst) {
			devm_gpiod_put(host->dev, card_gpio->rst);
			card_gpio->rst = NULL;
		}
	}
}

static void mmc_get_clk_name_v1(int id, char *cgu_name, char *gate_name)
{
	sprintf(cgu_name, "cgu_msc%d", id);
	sprintf(gate_name, "gate_msc%d", id);
}

static void mmc_get_clk_name_v2(int id, char *cgu_name, char *gate_name)
{
	sprintf(cgu_name, "div_msc%d", id);
	sprintf(gate_name, "gate_msc%d", id);
}

static struct ingenic_mmc_priv ingenic_mmc_priv = {
	.get_clk_name = mmc_get_clk_name_v1,
};

static struct ingenic_mmc_priv x1600_mmc_priv = {
	.get_clk_name = mmc_get_clk_name_v2,
};

static const struct of_device_id mmc_ingenic_of_match[] = {
	{.compatible = "ingenic,mmc", .data = &ingenic_mmc_priv},
	{.compatible = "ingenic,x1600-mmc", .data = &x1600_mmc_priv},
	{},
};
MODULE_DEVICE_TABLE(of, mmc_ingenic_of_match);

static struct ingenic_mmc_pdata *of_get_mmc_ingenic_pdata(struct device *dev)
{
	struct ingenic_mmc_pdata *pdata;
	struct device_node *np = dev->of_node;
	struct card_gpio *card_gpio;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		return NULL;    /* out of memory */
	}

	card_gpio = devm_kzalloc(dev, sizeof(struct card_gpio), GFP_KERNEL);
	if (!card_gpio) {
		return NULL;
	}

	pdata->gpio = card_gpio;

	if (of_property_read_bool(np, "pio-mode")) {
		pdata->pio_mode = 1;
	}

	{
		unsigned int val;
		if (!(of_property_read_u32(np, "ingenic,sdio_clk", &val))) {
			pdata->sdio_clk = val;
		}
	}

	if (of_property_read_bool(np, "ingenic,removal-dontcare")) {
		pdata->removal = DONTCARE;
	} else if (of_property_read_bool(np, "ingenic,removal-nonremovable")) {
		pdata->removal = NONREMOVABLE;
	} else if (of_property_read_bool(np, "ingenic,removal-removable")) {
		pdata->removal = REMOVABLE;
	} else if (of_property_read_bool(np, "ingenic,removal-manual")) {
		pdata->removal = MANUAL;
	};

	return pdata;
}

static int mmc_ingenic_probe(struct platform_device *pdev)
{
	struct ingenic_mmc_pdata *pdata;
	struct resource *regs;
	struct ingenic_mmc_host *host = NULL;
	struct mmc_host *mmc;
	const struct ingenic_mmc_priv *priv;
	const struct of_device_id *of_device_id;
	int ret = 0;
	char clk_cgu_name[16];
	char clk_gate_name[16];

	pdata = of_get_mmc_ingenic_pdata(&pdev->dev);
	if (IS_ERR(pdata)) {
		dev_err(&pdev->dev, "Platform Data is missing!\n");
		return PTR_ERR(pdata);
	}
	mmc = mmc_alloc_host(sizeof(struct ingenic_mmc_host), &pdev->dev);
	if (!mmc) {
		return -ENOMEM;
	}

	host = mmc_priv(mmc);

	of_device_id = of_match_node(mmc_ingenic_of_match, pdev->dev.of_node);
	priv = of_device_id->data;

	pdev->id = of_alias_get_id(pdev->dev.of_node, "mmc");
	host->index = pdev->id;
	priv->get_clk_name(host->index, clk_cgu_name, clk_gate_name);
	host->clk_cgu = devm_clk_get(&pdev->dev, clk_cgu_name);
	if (!host->clk_cgu) {
		dev_err(&pdev->dev, "Failed to Get MSC clk!\n");
		return PTR_ERR(host->clk_cgu);
	}
	host->clk_gate = devm_clk_get(&pdev->dev, clk_gate_name);
	if (!host->clk_gate) {
		dev_err(&pdev->dev, "Failed to Get PWC MSC clk!\n");
		return PTR_ERR(host->clk_gate);
	}

	clk_set_rate(host->clk_cgu, CLK_RATE);
	if (clk_get_rate(host->clk_cgu) > CLK_RATE) {
		dev_err(&pdev->dev, "Failed to Set MSC clk %ld!\n", clk_get_rate(host->clk_cgu));
		goto err_clk_get_rate;
	}
	ingenic_mmc_clk_onoff(host, 1);

	host->dev = &pdev->dev;
	host->pdata = pdata;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "No iomem resource\n");
		return -ENXIO;
	}
	host->iomem = devm_ioremap_resource(&pdev->dev, regs);
	if (!host->iomem) {
		goto err_ioremap;
	}

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		dev_err(&pdev->dev, "No irq resource\n");
		return host->irq;
	}

	/* host->power = regulator_get(host->dev, "cpu_mem12"); */
	/* if (IS_ERR(host->power)) { */
	/*  dev_warn(host->dev, "vmmc regulator missing\n"); */
	/* } */
	if (host->pdata->pio_mode) {
		set_bit(INGENIC_MMC_USE_PIO, &host->flags);
	}

	if (!test_bit(INGENIC_MMC_USE_PIO, &host->flags)) {
		ret = ingenic_mmc_dma_init(host);
		if (ret < 0) {
			goto err_dma_init;
		}
	}

	spin_lock_init(&host->lock);
	if (pdata->sdio_clk) {
		ingenic_sdio_wlan_init(&pdev->dev, host->index);
	}

	ret = ingenic_mmc_msc_init(host);
	if (ret < 0) {
		goto err_msc_init;
	}

	mmc_of_parse_voltage(mmc, &pdata->ocr_avail);

	ingenic_mmc_host_init(host, mmc);

	ret = ingenic_mmc_gpio_init(host);
	if (ret < 0) {
		goto err_gpio_init;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &ingenic_mmc_attr_group);
	if (ret < 0) {
		goto err_sysfs_create;
	}

	platform_set_drvdata(pdev, host);

	dev_info(host->dev, "register success!\n");
	return 0;

err_sysfs_create:
	ingenic_mmc_gpio_deinit(host);
err_gpio_init:
	free_irq(host->irq, host);
err_msc_init:
	devm_iounmap(&pdev->dev, (void __iomem *)host->decshds[0].dma_desc);
err_dma_init:
	devm_iounmap(&pdev->dev, host->iomem);
err_ioremap:
	mmc_free_host(mmc);
	clk_disable_unprepare(host->clk_cgu);
err_clk_get_rate:
	clk_put(host->clk_cgu);
	clk_put(host->clk_gate);
	dev_err(host->dev, "mmc probe error\n");
	return ret;
}

static int __exit mmc_ingenic_remove(struct platform_device *pdev)
{
	struct ingenic_mmc_host *host = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	mmc_remove_host(host->mmc);
	mmc_free_host(host->mmc);
	sysfs_remove_group(&pdev->dev.kobj, &ingenic_mmc_attr_group);

	ingenic_mmc_power_off(host);
	if (host->pdata->removal == REMOVABLE) {
		free_irq(gpiod_to_irq(host->pdata->gpio->cd), host);
	}

	free_irq(host->irq, host);
	ingenic_mmc_gpio_deinit(host);
	devm_iounmap(&pdev->dev, (void __iomem *)host->decshds[0].dma_desc);
	/* regulator_put(host->power); */
	ingenic_mmc_clk_onoff(host, 0);

	clk_put(host->clk_cgu);
	clk_put(host->clk_gate);
	iounmap(host->iomem);
	kfree(host);

	return 0;
}

static void mmc_ingenic_shutdown(struct platform_device *pdev)
{
	struct ingenic_mmc_host *host = platform_get_drvdata(pdev);
	struct card_gpio *card_gpio = host->pdata->gpio;

	/*
	 * Remove host when shutdown to avoid illegal request,
	 * but don't remove sdio_host in case of the SDIO device driver
	 * can't handle bus remove correctly.
	 */
	dev_vdbg(host->dev, "shutdown\n");
	if (host->mmc->card && !mmc_card_sdio(host->mmc->card)) {
		if (card_gpio->rst) {
			gpiod_set_value(card_gpio->rst, 0);
		} else {
			mmc_remove_host(host->mmc);
		}
	}

}

#ifdef CONFIG_PM_SLEEP
static int mmc_ingenic_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ingenic_mmc_host *host = platform_get_drvdata(pdev);
	int ret = 0;

	ingenic_mmc_clk_onoff(host, 0);
	/* if (host->mmc->card && host->mmc->card->type != MMC_TYPE_SDIO) { */
	/*  ret = mmc_suspend_host(host->mmc); */
	/* } */
	return ret;
}

static int mmc_ingenic_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ingenic_mmc_host *host = platform_get_drvdata(pdev);
	int ret = 0;
	ingenic_mmc_clk_onoff(host, 1);
	/* if (host->mmc->card && host->mmc->card->type != MMC_TYPE_SDIO) { */
	/*  ret = mmc_resume_host(host->mmc); */
	/* } */
	return ret;
}
#else
#define mmc_ingenic_suspend NULL
#define mmc_ingenic_resume NULL
#endif
static SIMPLE_DEV_PM_OPS(mmc_ingenic_pm_ops, mmc_ingenic_suspend,
                         mmc_ingenic_resume);

static struct platform_driver mmc_ingenic_driver = {
	.driver = {
		.name   = "ingenic,mmc",
		.owner  = THIS_MODULE,
		.pm = &mmc_ingenic_pm_ops,
		.of_match_table = of_match_ptr(mmc_ingenic_of_match),
	},
	.probe = mmc_ingenic_probe,
	.remove = mmc_ingenic_remove,
	.shutdown = mmc_ingenic_shutdown,
};

module_platform_driver(mmc_ingenic_driver);

MODULE_DESCRIPTION("Multimedia Card Interface driver, MMC version 1.2");
MODULE_AUTHOR("bo.liu <bo.liu@ingenic.cn>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("20170222");
