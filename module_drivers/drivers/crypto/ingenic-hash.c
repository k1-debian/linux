/*
 * Cryptographic API.
 *
 * Support for Ingenic HASH HW acceleration.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/crypto.h>
#include <linux/interrupt.h>
#include <crypto/scatterwalk.h>
#include <crypto/hash.h>
#include <linux/delay.h>
#include <crypto/internal/hash.h>
#include "ingenic-hash.h"
#include <crypto/sha1.h>
#include <crypto/sha2.h>

/* keep registered devices data here */
static LIST_HEAD(dev_list);
static DEFINE_SPINLOCK(list_lock);

static inline unsigned long hash_read(struct ingenic_hash_dev *hash, unsigned long offset)
{
	return readl(hash->io_base + offset);
}

static inline void hash_write(struct ingenic_hash_dev *hash, unsigned long offset,
                              unsigned long value)
{
	writel(value, hash->io_base + offset);
}

static inline void dump_reg(struct ingenic_hash_dev *hash)
{
	dev_info(hash->dev, "HASH_HSCR: 0x%08lx\n", hash_read(hash, HASH_HSCR));
	dev_info(hash->dev, "HASH_HSSR: 0x%08lx\n", hash_read(hash, HASH_HSSR));
	dev_info(hash->dev, "hash_ASINTM: 0x%08lx\n", hash_read(hash, HASH_HSINTM));
	dev_info(hash->dev, "HASH_HSSA: 0x%08lx\n", hash_read(hash, HASH_HSSA));
	dev_info(hash->dev, "HASH_HSTC: 0x%08lx\n", hash_read(hash, HASH_HSTC));
}
static inline int ingenic_hash_hw_init(struct ingenic_hash_dev *hash, struct ingenic_hash_reqctx *ctx)
{
	/*
	 * clocks are enabled when request starts and disabled when finished.
	 * It may be long delays between requests.
	 * Device might go to off mode to save power.
	 */
	unsigned int alg = -1;
	unsigned int init_data = 0;
	pm_runtime_get_sync(hash->dev);

	if (ctx->flags & HASH_FLAGS_MD5) {
		alg = 0;
		init_data = 0;
	} else if (ctx->flags & HASH_FLAGS_SHA1) {
		init_data = 3 << 7;
		alg = 1;
	} else if (ctx->flags & HASH_FLAGS_SHA224) {
		init_data = 3 << 7;
		alg = 2;
	} else if (ctx->flags & HASH_FLAGS_SHA256) {
		init_data = 3 << 7;
		alg = 3;
	} else if (ctx->flags & HASH_FLAGS_SHA384) {
		init_data = 3 << 7;
		alg = 4;
	} else if (ctx->flags & HASH_FLAGS_SHA512) {
		init_data = 3 << 7;
		alg = 5;
	}
	init_data |= 3 << 28 | 1 << 4 | alg << 1 | 1;
	hash_write(hash, HASH_HSCR, init_data);
	hash_write(hash, HASH_HSINTM, 0);
	hash->flags |= HASH_FLAGS_INIT;
	return 0;
}
static int ingenic_hash_update_dma_stop(struct ingenic_hash_dev *hash)
{
	struct ingenic_hash_reqctx *ctx = ahash_request_ctx(hash->req);

	if (ctx->flags & HASH_FLAGS_SG) {
		dma_unmap_sg(hash->dev, ctx->sg, 1, DMA_TO_DEVICE);
		if (ctx->sg->length == ctx->offset) {
			ctx->sg = sg_next(ctx->sg);
			if (ctx->sg) {
				ctx->offset = 0;
			}
		}
		if (ctx->flags & HASH_FLAGS_PAD) {
			dma_unmap_single(hash->dev, ctx->dma_addr,
			                 ctx->buflen + ctx->block_size, DMA_TO_DEVICE);
		}
	} else {
		dma_unmap_single(hash->dev, ctx->dma_addr, ctx->buflen +
		                 ctx->block_size, DMA_TO_DEVICE);
	}

	return 0;
}
static void ingenic_hash_copy_hash(struct ahash_request *req)
{
	struct ingenic_hash_reqctx *ctx = ahash_request_ctx(req);
	u32 *hash = (u32 *)ctx->digest;
	int i;
	if (ctx->flags & HASH_FLAGS_MD5) {
		for (i = 0; i < MD5_DIGEST_SIZE / sizeof(u32); i++) {
			hash[i] = hash_read(ctx->hash, HASH_HSDO);
		}
	} else if (ctx->flags & HASH_FLAGS_SHA1)
		for (i = 0; i < SHA1_DIGEST_SIZE / sizeof(u32); i++) {
			hash[i] = hash_read(ctx->hash, HASH_HSDO);
		} else if (ctx->flags & HASH_FLAGS_SHA224)
		for (i = 0; i < SHA224_DIGEST_SIZE / sizeof(u32); i++) {
			hash[i] = hash_read(ctx->hash, HASH_HSDO);
		} else if (ctx->flags & HASH_FLAGS_SHA256)
		for (i = 0; i < SHA256_DIGEST_SIZE / sizeof(u32); i++) {
			hash[i] = hash_read(ctx->hash, HASH_HSDO);
		} else if (ctx->flags & HASH_FLAGS_SHA384)
		for (i = 0; i < SHA384_DIGEST_SIZE / sizeof(u32); i++) {
			hash[i] = hash_read(ctx->hash, HASH_HSDO);
		} else
		for (i = 0; i < SHA512_DIGEST_SIZE / sizeof(u32); i++) {
			hash[i] = hash_read(ctx->hash, HASH_HSDO);
		}
}

static void ingenic_hash_copy_ready_hash(struct ahash_request *req)
{
	struct ingenic_hash_reqctx *ctx = ahash_request_ctx(req);
	if (!req->result) {
		return;
	}
	if (ctx->flags & HASH_FLAGS_MD5) {
		memcpy(req->result, ctx->digest, MD5_DIGEST_SIZE);
	}
	if (ctx->flags & HASH_FLAGS_SHA1) {
		memcpy(req->result, ctx->digest, SHA1_DIGEST_SIZE);
	} else if (ctx->flags & HASH_FLAGS_SHA224) {
		memcpy(req->result, ctx->digest, SHA224_DIGEST_SIZE);
	} else if (ctx->flags & HASH_FLAGS_SHA256) {
		memcpy(req->result, ctx->digest, SHA256_DIGEST_SIZE);
	} else if (ctx->flags & HASH_FLAGS_SHA384) {
		memcpy(req->result, ctx->digest, SHA384_DIGEST_SIZE);
	} else {
		memcpy(req->result, ctx->digest, SHA512_DIGEST_SIZE);
	}
}

static int ingenic_hash_finish(struct ahash_request *req)
{
	struct ingenic_hash_reqctx *ctx = ahash_request_ctx(req);
	struct ingenic_hash_dev *hash = ctx->hash;
	int err = 0;

	if (ctx->digcnt[0] || ctx->digcnt[1]) {
		ingenic_hash_copy_ready_hash(req);
	}

	dev_dbg(hash->dev, "digcnt: 0x%llx 0x%llx, bufcnt: %d\n", ctx->digcnt[1],
	        ctx->digcnt[0], ctx->bufcnt);

	return err;
}
static void ingenic_hash_finish_req(struct ahash_request *req, int err)
{
	struct ingenic_hash_reqctx *ctx = ahash_request_ctx(req);
	struct ingenic_hash_dev *hash = ctx->hash;
	if (ctx->bufcnt == 0) {
		if (!err) {
			ingenic_hash_copy_hash(req);
			if (HASH_FLAGS_FINAL & hash->flags) {
				err = ingenic_hash_finish(req);
			}
		} else {
			ctx->flags |= HASH_FLAGS_ERROR;
		}

		/* atomic operation is not needed here */
		hash->flags &= ~(HASH_FLAGS_BUSY | HASH_FLAGS_FINAL | HASH_FLAGS_INIT);
		pm_runtime_put_sync(hash->dev);
	}
	/* clk_disable_unprepare(hash->iclk); */
	if (req->base.complete) {
		ahash_request_complete(req, err);
	}
}

static struct ingenic_hash_dev *ingenic_hash_find_dev(struct ingenic_hash_ctx *ctx)
{
	struct ingenic_hash_dev *hash = NULL, *tmp;

	spin_lock(&list_lock);
	hash = ctx->hash;
	if (!hash) {
		list_for_each_entry(tmp, &dev_list, list) {
			/* FIXME: take fist available hash core */
			hash = tmp;
			ctx->hash = tmp;
			break;
		}
	}
	spin_unlock(&list_lock);
	return hash;
}
/*
 * The purpose of this padding is to ensure that the padded message is a
 * multiple of 512 bits (SHA1/SHA224/SHA256) or 1024 bits (SHA384/SHA512).
 * The bit "1" is appended at the end of the message followed by
 * "padlen-1" zero bits. Then a 64 bits block (SHA1/SHA224/SHA256) or
 * 128 bits block (SHA384/SHA512) equals to the message length in bits
 * is appended.
 *
 * For SHA1/SHA224/SHA256, padlen is calculated as followed:
 *  - if message length < 56 bytes then padlen = 56 - message length
 *  - else padlen = 64 + 56 - message length
 *
 * For SHA384/SHA512, padlen is calculated as followed:
 *  - if message length < 112 bytes then padlen = 112 - message length
 *  - else padlen = 128 + 112 - message length
 */
static void ingenic_hash_fill_padding(struct ingenic_hash_reqctx *ctx, int length)
{

	unsigned int index, padlen;
	u64 bits[2];
	u64 size[2];

	size[0] = ctx->digcnt[0];
	size[1] = ctx->digcnt[1];

	size[0] += ctx->bufcnt;
	if (size[0] < ctx->bufcnt) {
		size[1]++;
	}

	size[0] += length;
	if (size[0]  < length) {
		size[1]++;
	}

	bits[1] = cpu_to_be64(size[0] << 3);
	bits[0] = cpu_to_be64(size[1] << 3 | size[0] >> 61);
	if (ctx->flags & (HASH_FLAGS_SHA384 | HASH_FLAGS_SHA512)) {
		index = ctx->bufcnt & 0x7f;
		padlen = (index < 112) ? (112 - index) : ((128 + 112) - index);
		*(ctx->buffer + ctx->bufcnt) = 0x80;
		memset(ctx->buffer + ctx->bufcnt + 1, 0, padlen - 1);
		memcpy(ctx->buffer + ctx->bufcnt + padlen, bits, 16);
		ctx->bufcnt += padlen + 16;
		ctx->flags |= HASH_FLAGS_PAD;
	} else if (ctx->flags & HASH_FLAGS_MD5) {
		unsigned int *d;
		index = ctx->bufcnt & 0x3f;
		padlen = (index < 56) ? (56 - index) : ((64 + 56) - index);
		*(ctx->buffer + ctx->bufcnt) = 0x80;
		memset(ctx->buffer + ctx->bufcnt + 1, 0, padlen - 1);
		d = (unsigned int *)((unsigned char *)ctx->buffer + ctx->bufcnt + padlen);
		d[0] = cpu_to_le32(size[0] << 3);
		d[1] = cpu_to_le32(size[0] >> 29);
		//memcpy(ctx->buffer + ctx->bufcnt + padlen + 1, &bits1[1], 4);
		ctx->bufcnt += padlen + 8;
		ctx->flags |= HASH_FLAGS_PAD;
	} else {
		index = ctx->bufcnt & 0x3f;
		padlen = (index < 56) ? (56 - index) : ((64 + 56) - index);
		*(ctx->buffer + ctx->bufcnt) = 0x80;
		memset(ctx->buffer + ctx->bufcnt + 1, 0, padlen - 1);
		memcpy(ctx->buffer + ctx->bufcnt + padlen, &bits[1], 8);
		ctx->bufcnt += padlen + 8;
		ctx->flags |= HASH_FLAGS_PAD;
	}
	/* for(int i = 0;i < 64/4;i++){ */
	/*  unsigned int *d = ctx->buffer; */
	/*  d +=(ctx->bufcnt - 64) / sizeof(unsigned int); */
	/*  printk("buffer[%p]%03d: 0x%08x\n",d+i,i,d[i]); */
	/* } */
}

static size_t ingenic_hash_append_sg(struct ingenic_hash_reqctx *ctx)
{
	size_t count;

	while ((ctx->bufcnt < ctx->buflen) && ctx->total) {
		count = min(ctx->sg->length - ctx->offset, ctx->total);
		count = min(count, ctx->buflen - ctx->bufcnt);

		if (count <= 0) {
			break;
		}

		scatterwalk_map_and_copy(ctx->buffer + ctx->bufcnt, ctx->sg,
		                         ctx->offset, count, 0);

		ctx->bufcnt += count;
		ctx->offset += count;
		ctx->total -= count;

		if (ctx->offset == ctx->sg->length) {
			ctx->sg = sg_next(ctx->sg);
			if (ctx->sg) {
				ctx->offset = 0;
			} else {
				ctx->total = 0;
			}
		}
	}
	return 0;
}
static int ingenic_hash_xmit_start(struct ingenic_hash_dev *hash, dma_addr_t dma_addr,
                                   size_t length, int final)
{
	struct ingenic_hash_reqctx *ctx = ahash_request_ctx(hash->req);
	ctx->digcnt[0] += length;
	if (ctx->digcnt[0] < length) {
		ctx->digcnt[1]++;
	}

	if (final) {
		hash->flags |= HASH_FLAGS_FINAL;
	}
	hash->flags |= HASH_FLAGS_DMA_ACTIVE;
	hash_write(hash, HASH_HSSA, dma_addr);
	hash_write(hash, HASH_HSTC, length / (512 / 8));

	hash_write(hash, HASH_HSINTM, 2);
	//  dump_reg(hash);
	{
		unsigned int val;
		val = hash_read(hash, HASH_HSCR);
		hash_write(hash, HASH_HSCR, val | (3 << 5));
	}
	return -EINPROGRESS;
}

static int ingenic_hash_xmit_dma_map(struct ingenic_hash_dev *hash,
                                     struct ingenic_hash_reqctx *ctx,
                                     size_t length, int final)
{
	ctx->dma_addr = dma_map_single(hash->dev, ctx->buffer,
	                               ctx->buflen + ctx->block_size, DMA_TO_DEVICE);

	if (dma_mapping_error(hash->dev, ctx->dma_addr)) {
		dev_err(hash->dev, "dma %u bytes error\n", ctx->buflen +
		        ctx->block_size);
		return -EINVAL;
	}

	ctx->flags &= ~HASH_FLAGS_SG;

	/* next call does not fail... so no unmap in the case of error */
	return ingenic_hash_xmit_start(hash, ctx->dma_addr, length, final);
}
static int ingenic_hash_final_req(struct ingenic_hash_dev *hash)
{
	struct ahash_request *req = hash->req;
	struct ingenic_hash_reqctx *ctx = ahash_request_ctx(req);
	int err = 0;
	int count;

	ingenic_hash_fill_padding(ctx, 0);
	count = ctx->bufcnt;
	ctx->bufcnt = 0;
	err = ingenic_hash_xmit_dma_map(hash, ctx, count, 1);
	dev_dbg(hash->dev, "final_req: err: %d\n", err);

	return err;
}

static int ingenic_hash_update_dma_slow(struct ingenic_hash_dev *hash)
{
	struct ingenic_hash_reqctx *ctx = ahash_request_ctx(hash->req);
	unsigned int final;
	size_t count;

	ingenic_hash_append_sg(ctx);

	final = (ctx->flags & HASH_FLAGS_FINUP) && !ctx->total;

	dev_dbg(hash->dev, "slow: bufcnt: %u, digcnt: 0x%llx 0x%llx, final: %d\n",
	        ctx->bufcnt, ctx->digcnt[1], ctx->digcnt[0], final);

	if (final) {
		ingenic_hash_fill_padding(ctx, 0);
	}

	if (final || (ctx->bufcnt == ctx->buflen && ctx->total)) {
		count = ctx->bufcnt;
		ctx->bufcnt = 0;
		return ingenic_hash_xmit_dma_map(hash, ctx, count, final);
	}

	return 0;
}

static int ingenic_hash_update_dma_start(struct ingenic_hash_dev *hash)
{
	struct ingenic_hash_reqctx *ctx = ahash_request_ctx(hash->req);
	unsigned int length, final, tail;
	struct scatterlist *sg;
	unsigned int count;
	sg = ctx->sg;
	if (!ctx->total && !ctx->bufcnt) {
		return 0;
	}
	if (ctx->flags & HASH_FLAGS_FINUP) {
		if (ctx->bufcnt || ctx->offset) {
			return ingenic_hash_update_dma_slow(hash);
		}
		dev_dbg(hash->dev, "fast: digcnt: 0x%llx 0x%llx, bufcnt: %u, total: %u\n",
		        ctx->digcnt[1], ctx->digcnt[0], ctx->bufcnt, ctx->total);

		if (!IS_ALIGNED(sg->offset, sizeof(u32))) {
			return ingenic_hash_update_dma_slow(hash);
		}

		if (!sg_is_last(sg) && !IS_ALIGNED(sg->length, ctx->block_size)) {
			/* size is not ctx->block_size aligned */
			return ingenic_hash_update_dma_slow(hash);
		}
	}
	length = min(ctx->total, sg->length);
	if (sg_is_last(sg)) {
		if (!(ctx->flags & HASH_FLAGS_FINUP)) {
			/* not last sg must be ctx->block_size aligned */
			tail = length & (ctx->block_size - 1);
			length -= tail;
			if (length == 0) {
				ingenic_hash_append_sg(ctx);
				hash->flags &= ~HASH_FLAGS_BUSY;
				return 0;
			}
		} else if (length + ctx->bufcnt > ctx->buflen) {
			length = ctx->buflen - ctx->bufcnt;
		}
	}
	final = (ctx->flags & HASH_FLAGS_FINUP) && !(ctx->total - length);
	/* Add padding */
	if (final) {

		sg = ctx->sg;
		ingenic_hash_append_sg(ctx);

		ingenic_hash_fill_padding(ctx, 0);

		ctx->dma_addr = dma_map_single(hash->dev, ctx->buffer,
		                               ctx->buflen + ctx->block_size, DMA_TO_DEVICE);

		if (dma_mapping_error(hash->dev, ctx->dma_addr)) {
			dev_err(hash->dev, "dma %u bytes error\n",
			        ctx->buflen + ctx->block_size);
			return -EINVAL;
		}

		if (ctx->bufcnt != 0) {
			ctx->flags &= ~HASH_FLAGS_SG;
			count = ctx->bufcnt;
			ctx->bufcnt = 0;
			return ingenic_hash_xmit_start(hash, ctx->dma_addr, count, final);
		}
	} else if (ctx->bufcnt && (ctx->flags & HASH_FLAGS_FINUP)) {
		return ingenic_hash_update_dma_slow(hash);
	} else {
		ctx->total -= length;
		ctx->offset = length; /* offset where to start slow */
	}

	if (!dma_map_sg(hash->dev, ctx->sg, 1, DMA_TO_DEVICE)) {
		dev_err(hash->dev, "dma_map_sg  error\n");
		return -EINVAL;
	}
	ctx->flags |= HASH_FLAGS_SG;

	/* next call does not fail... so no unmap in the case of error */
	return ingenic_hash_xmit_start(hash, sg_dma_address(ctx->sg), length, final);
}

static int ingenic_hash_update_req(struct ingenic_hash_dev *hash)
{
	struct ahash_request *req = hash->req;
	struct ingenic_hash_reqctx *ctx = ahash_request_ctx(req);
	int err;

	dev_dbg(hash->dev, "update_req: total: %u, digcnt: 0x%llx 0x%llx\n",
	        ctx->total, ctx->digcnt[1], ctx->digcnt[0]);

	err = ingenic_hash_update_dma_start(hash);

	/* wait for dma completion before can take more data */
	dev_dbg(hash->dev, "update: err: %d, digcnt: 0x%llx 0%llx\n",
	        err, ctx->digcnt[1], ctx->digcnt[0]);

	return err;
}
static int ingenic_hash_handle_queue(struct ingenic_hash_dev *hash,
                                     struct ahash_request *req)
{
	struct crypto_async_request *async_req, *backlog;
	struct ingenic_hash_reqctx *ctx;
	unsigned long flags;
	int err = 0, ret = 0;
	spin_lock_irqsave(&hash->lock, flags);
	if (req) {
		ret = ahash_enqueue_request(&hash->queue, req);
	}

	if (HASH_FLAGS_BUSY & hash->flags) {
		spin_unlock_irqrestore(&hash->lock, flags);
		return ret;
	}

	backlog = crypto_get_backlog(&hash->queue);
	async_req = crypto_dequeue_request(&hash->queue);
	if (async_req) {
		hash->flags |= HASH_FLAGS_BUSY;
	}

	spin_unlock_irqrestore(&hash->lock, flags);

	if (!async_req) {
		return ret;
	}

	if (backlog) {
		backlog->complete(backlog, -EINPROGRESS);
	}

	req = ahash_request_cast(async_req);
	hash->req = req;
	ctx = ahash_request_ctx(req);

	dev_dbg(hash->dev, "handling new req, op: %lu, nbytes: %d\n",
	        ctx->op, req->nbytes);

	if (!(hash->flags & HASH_FLAGS_INIT)) {
		err = ingenic_hash_hw_init(hash, ctx);
		if (err) {
			goto err1;
		}
	}
	/* if (ctx->op == HASH_OP_UPDATE) { */
	err = ingenic_hash_update_req(hash);
	/*  if (err != -EINPROGRESS && (ctx->flags & HASH_FLAGS_FINUP)) */
	/*      /\* no final() after finup() *\/ */
	/*      err = ingenic_hash_final_req(hash); */
	/* } else if (ctx->op == HASH_OP_FINAL) { */
	/*  err = ingenic_hash_final_req(hash); */
	/* } */
err1:
	return err;
#if 0
err1:
	if (err != -EINPROGRESS)
		/* done_task will not finish it, so do it here */
	{
		ingenic_hash_finish_req(req, err);
	}

	dev_dbg(hash->dev, "exit, err: %d\n", err);

	return err;
#endif
}

static int ingenic_hash_enqueue(struct ahash_request *req, unsigned int op)
{
	struct ingenic_hash_reqctx *ctx = ahash_request_ctx(req);
	struct ingenic_hash_ctx *tctx = crypto_tfm_ctx(req->base.tfm);
	struct ingenic_hash_dev *hash = tctx->hash;

	ctx->op = op;

	return ingenic_hash_handle_queue(hash, req);
}
static void hash_dma_done(struct ingenic_hash_dev *hash)
{
	int err = 0;
	if (HASH_FLAGS_DMA_ACTIVE & hash->flags) {
		hash->flags &= ~HASH_FLAGS_DMA_ACTIVE;
		ingenic_hash_update_dma_stop(hash);
	}
	err = ingenic_hash_update_dma_start(hash);
	if (err != -EINPROGRESS) {
		ingenic_hash_finish_req(hash->req, err);
	} else if (!(HASH_FLAGS_BUSY & hash->flags)) {
		ingenic_hash_handle_queue(hash, NULL);
	}
}
static irqreturn_t ingenic_hash_irqthread(int irq, void *data)
{
	struct ingenic_hash_dev *hash = (struct ingenic_hash_dev *)data;
	unsigned int val;
	val = hash_read(hash, HASH_HSSR);
	hash_write(hash, HASH_HSSR, val);
	if (val & 2) {
		hash_dma_done(hash);
	} else {
		printk("Don't support the irq");
	}
	return IRQ_HANDLED;
}

/* ********************** ALG API ************************************ */

static int ingenic_hash_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct ingenic_hash_ctx *ctx = crypto_ahash_ctx(tfm);
	struct ingenic_hash_reqctx *rctx = ahash_request_ctx(req);
	struct ingenic_hash_dev *hash = NULL;
	/* struct ingenic_hash_dev *tmp; */
	ENTER();
	hash = ingenic_hash_find_dev(ctx);

	rctx->hash = hash;
	rctx->flags = 0;
	dev_dbg(hash->dev, "init: digest size: %d\n",
	        crypto_ahash_digestsize(tfm));
	switch (crypto_ahash_digestsize(tfm)) {
	case MD5_DIGEST_SIZE:
		rctx->flags |= HASH_FLAGS_MD5;
		rctx->block_size = MD5_BLOCK_SIZE;
		break;
	case SHA1_DIGEST_SIZE:
		rctx->flags |= HASH_FLAGS_SHA1;
		rctx->block_size = SHA1_BLOCK_SIZE;
		break;
	case SHA224_DIGEST_SIZE:
		rctx->flags |= HASH_FLAGS_SHA224;
		rctx->block_size = SHA224_BLOCK_SIZE;
		break;
	case SHA256_DIGEST_SIZE:
		rctx->flags |= HASH_FLAGS_SHA256;
		rctx->block_size = SHA256_BLOCK_SIZE;
		break;
	case SHA384_DIGEST_SIZE:
		rctx->flags |= HASH_FLAGS_SHA384;
		rctx->block_size = SHA384_BLOCK_SIZE;
		break;
	case SHA512_DIGEST_SIZE:
		rctx->flags |= HASH_FLAGS_SHA512;
		rctx->block_size = SHA512_BLOCK_SIZE;
		break;
	default:
		return -EINVAL;
		break;
	}
	rctx->bufcnt = 0;
	rctx->total = 0;
	rctx->digcnt[0] = 0;
	rctx->digcnt[1] = 0;
	rctx->buffer = ctx->buf;
	rctx->buflen = HASH_BUFFER_LEN;
	EXIT();

	return 0;
}
static int ingenic_hash_final(struct ahash_request *req)
{
	struct ingenic_hash_reqctx *ctx = ahash_request_ctx(req);
	struct ingenic_hash_ctx *tctx = crypto_tfm_ctx(req->base.tfm);
	struct ingenic_hash_dev *hash = tctx->hash;

	int err = 0;
	ENTER();
	ctx->flags |= HASH_FLAGS_FINUP;
	if (ctx->flags & HASH_FLAGS_ERROR) {
		EXIT();
		return 0; /* uncompleted hash is not needed */
	}
	if (ctx->bufcnt) {
		err = ingenic_hash_enqueue(req, HASH_OP_FINAL);
		EXIT();
		return err;
	} else if (!(ctx->flags & HASH_FLAGS_PAD)) { /* add padding */
		err = ingenic_hash_hw_init(hash, ctx);
		if (err) {
			goto err1;
		}
		hash->req = req;
		hash->flags |= HASH_FLAGS_BUSY;
		err = ingenic_hash_final_req(hash);
	} else {
		/* copy ready hash (+ finalize hmac) */
		return ingenic_hash_finish(req);
	}
err1:
	if (err != -EINPROGRESS)
		/* done_task will not finish it, so do it here */
	{
		ingenic_hash_finish_req(req, err);
	}
	EXIT();
	return err;
}

static int ingenic_hash_update(struct ahash_request *req)
{
	struct ingenic_hash_reqctx *ctx = ahash_request_ctx(req);
	int ret;
	ENTER();
	if (!req->nbytes) {
		EXIT();
		return 0;
	}

	ctx->total = req->nbytes;
	ctx->sg = req->src;

	ctx->offset = 0;
	if (ctx->bufcnt + ctx->total < ctx->buflen) {
		ingenic_hash_append_sg(ctx);
		return 0;
	}
	ret = ingenic_hash_enqueue(req, HASH_OP_UPDATE);
	EXIT();
	return ret;
}

static int ingenic_hash_finup(struct ahash_request *req)
{
	struct ingenic_hash_reqctx *ctx = ahash_request_ctx(req);
	int err1, err2;
	ENTER();
	ctx->flags |= HASH_FLAGS_FINUP;

	err1 = ingenic_hash_update(req);
	if (err1 == -EINPROGRESS || err1 == -EBUSY) {
		return err1;
	}

	/*
	 * final() has to be always called to cleanup resources
	 * even if udpate() failed, except EINPROGRESS
	 */
	err2 = ingenic_hash_final(req);
	EXIT();
	return err1 ? : err2;
}

static int ingenic_hash_digest(struct ahash_request *req)
{
	return ingenic_hash_init(req) ? : ingenic_hash_finup(req);
}

static int ingenic_hash_export(struct ahash_request *req, void *out)
{
	struct ingenic_hash_reqctx *rctx = ahash_request_ctx(req);

	memcpy(out, rctx, sizeof(*rctx));

	return 0;
}

static int ingenic_hash_import(struct ahash_request *req, const void *in)
{
	struct ingenic_hash_reqctx *rctx = ahash_request_ctx(req);

	memcpy(rctx, in, sizeof(*rctx));

	return 0;
}

static int ingenic_hash_cra_init_alg(struct crypto_tfm *tfm, const char *alg_base)
{
	//  struct ingenic_hash_ctx *tctx = crypto_tfm_ctx(tfm);
	//  const char *alg_name = crypto_tfm_alg_name(tfm);
	//  ENTER();
	//  /* Allocate a fallback and abort if it failed. */
	//  tctx->fallback = crypto_alloc_shash(alg_name, 0,
	//      CRYPTO_ALG_NEED_FALLBACK);
	//  if (IS_ERR(tctx->fallback)) {
	//      pr_err("ingenic-hash: fallback driver '%s' could not be loaded.\n",
	//          alg_name);
	//      EXIT();
	//      return PTR_ERR(tctx->fallback);
	//  }
	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
	                         sizeof(struct ingenic_hash_reqctx));
	//  EXIT();
	return 0;
}

static int ingenic_hash_cra_init(struct crypto_tfm *tfm)
{
	return ingenic_hash_cra_init_alg(tfm, NULL);
}

static void ingenic_hash_cra_exit(struct crypto_tfm *tfm)
{
	struct ingenic_hash_ctx *tctx = crypto_tfm_ctx(tfm);
	ENTER();
	tctx->fallback = NULL;
	EXIT();
}

/* ********************** ALGS ************************************ */
static struct ahash_alg hash_md5_algs[] = {
#if 0
	{
		.init       = ingenic_hash_init,
		.update     = ingenic_hash_update,
		.final      = ingenic_hash_final,
		.finup      = ingenic_hash_finup,
		.digest     = ingenic_hash_digest,
		.halg = {
			.digestsize = MD5_DIGEST_SIZE,
			.statesize  = 1,
			.base   = {
				.cra_name       = "md5",
				.cra_driver_name = "ingenic-sha1",
				.cra_priority   = 100,
				.cra_flags      = CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK,
				.cra_blocksize  = MD5_BLOCK_SIZE,
				.cra_ctxsize    = sizeof(struct ingenic_hash_ctx),
				.cra_alignmask  = 0,
				.cra_module     = THIS_MODULE,
				.cra_init       = ingenic_hash_cra_init,
				.cra_exit       = ingenic_hash_cra_exit,
			}
		}
	},
#endif
	{
		.init       = ingenic_hash_init,
		.update     = ingenic_hash_update,
		.final      = ingenic_hash_final,
		.finup      = ingenic_hash_finup,
		.export     = ingenic_hash_export,
		.import     = ingenic_hash_import,
		.digest     = ingenic_hash_digest,
		.halg = {
			.digestsize = SHA1_DIGEST_SIZE,
			.statesize = sizeof(struct ingenic_hash_reqctx),
			.base   = {
				.cra_name       = "sha1",
				.cra_driver_name    = "ingenic-sha1",
				.cra_priority       = 100,
				.cra_flags      = CRYPTO_ALG_ASYNC,
				.cra_blocksize      = SHA1_BLOCK_SIZE,
				.cra_ctxsize        = sizeof(struct ingenic_hash_ctx),
				.cra_alignmask      = 0,
				.cra_module     = THIS_MODULE,
				.cra_init       = ingenic_hash_cra_init,
				.cra_exit       = ingenic_hash_cra_exit,
			}
		}
	},
	{
		.init       = ingenic_hash_init,
		.update     = ingenic_hash_update,
		.final      = ingenic_hash_final,
		.finup      = ingenic_hash_finup,
		.export     = ingenic_hash_export,
		.import     = ingenic_hash_import,
		.digest     = ingenic_hash_digest,
		.halg = {
			.digestsize = SHA224_DIGEST_SIZE,
			.statesize = sizeof(struct ingenic_hash_reqctx),
			.base   = {
				.cra_name       = "sha224",
				.cra_driver_name    = "ingenic-sha224",
				.cra_priority       = 100,
				.cra_flags      = CRYPTO_ALG_ASYNC,
				.cra_blocksize      = SHA224_BLOCK_SIZE,
				.cra_ctxsize        = sizeof(struct ingenic_hash_ctx),
				.cra_alignmask      = 0,
				.cra_module     = THIS_MODULE,
				.cra_init       = ingenic_hash_cra_init,
				.cra_exit       = ingenic_hash_cra_exit,
			}
		}
	},
	{
		.init       = ingenic_hash_init,
		.update     = ingenic_hash_update,
		.final      = ingenic_hash_final,
		.finup      = ingenic_hash_finup,
		.export     = ingenic_hash_export,
		.import     = ingenic_hash_import,
		.digest     = ingenic_hash_digest,
		.halg = {
			.digestsize = SHA256_DIGEST_SIZE,
			.statesize = sizeof(struct ingenic_hash_reqctx),
			.base   = {
				.cra_name       = "sha256",
				.cra_driver_name    = "ingenic-sha256",
				.cra_priority       = 100,
				.cra_flags      = CRYPTO_ALG_ASYNC,
				.cra_blocksize      = SHA256_BLOCK_SIZE,
				.cra_ctxsize        = sizeof(struct ingenic_hash_ctx),
				.cra_alignmask      = 0,
				.cra_module     = THIS_MODULE,
				.cra_init       = ingenic_hash_cra_init,
				.cra_exit       = ingenic_hash_cra_exit,
			}
		}
	},
	{
		.init       = ingenic_hash_init,
		.update     = ingenic_hash_update,
		.final      = ingenic_hash_final,
		.finup      = ingenic_hash_finup,
		.digest     = ingenic_hash_digest,
		.export     = ingenic_hash_export,
		.import     = ingenic_hash_import,
		.halg = {
			.digestsize = SHA384_DIGEST_SIZE,
			.statesize = sizeof(struct ingenic_hash_reqctx),
			.base   = {
				.cra_name       = "sha384",
				.cra_driver_name    = "ingenic-sha384",
				.cra_priority       = 100,
				.cra_flags      = CRYPTO_ALG_ASYNC,
				.cra_blocksize      = SHA384_BLOCK_SIZE,
				.cra_ctxsize        = sizeof(struct ingenic_hash_ctx),
				.cra_alignmask      = 0x3,
				.cra_module     = THIS_MODULE,
				.cra_init       = ingenic_hash_cra_init,
				.cra_exit       = ingenic_hash_cra_exit,
			}
		}
	},
	{
		.init       = ingenic_hash_init,
		.update     = ingenic_hash_update,
		.final      = ingenic_hash_final,
		.finup      = ingenic_hash_finup,
		.digest     = ingenic_hash_digest,
		.export     = ingenic_hash_export,
		.import     = ingenic_hash_import,
		.halg = {
			.digestsize = SHA512_DIGEST_SIZE,
			.statesize = sizeof(struct ingenic_hash_reqctx),
			.base   = {
				.cra_name       = "sha512",
				.cra_driver_name    = "ingenic-sha512",
				.cra_priority       = 100,
				.cra_flags      = CRYPTO_ALG_ASYNC,
				.cra_blocksize      = SHA512_BLOCK_SIZE,
				.cra_ctxsize        = sizeof(struct ingenic_hash_ctx),
				.cra_alignmask      = 0x3,
				.cra_module     = THIS_MODULE,
				.cra_init       = ingenic_hash_cra_init,
				.cra_exit       = ingenic_hash_cra_exit,
			}
		}
	},
};

static struct ingenic_hash_pdata ingenic_hash_pdata = {
	.algs_list  = hash_md5_algs,
	.size       = ARRAY_SIZE(hash_md5_algs),
};
static int ingenic_hash_get_res_pdev(struct ingenic_hash_dev *hash,
                                     struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *r;
	int err;
	int ret = 0;
	/* Get the base address */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(dev, "no MEM resource info\n");
		err = -ENODEV;
		goto err;
	}

	hash->io_base = devm_ioremap(hash->dev, r->start, resource_size(r));
	if (IS_ERR_OR_NULL(hash->io_base)) {
		dev_err(&pdev->dev, "Failed to remap io resources!\n");
		ret = -ENOMEM;
		goto err;
	}

	hash->irq = platform_get_irq(pdev, 0);
	if (hash->irq < 0) {
		dev_err(dev, "Failed to request irq resource info\n");
		ret = -EBUSY;
		goto err;
	}

	err = request_threaded_irq(hash->irq, NULL, ingenic_hash_irqthread,
	                           IRQF_ONESHOT, dev_name(dev), hash);

	hash->pdata = &ingenic_hash_pdata;

err:
	return ret;
}

static int ingenic_hash_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ingenic_hash_dev *hash;
	struct ahash_alg *algp;
	int err = -ENOMEM, i;

	hash = devm_kzalloc(dev, sizeof(struct ingenic_hash_dev), GFP_KERNEL);
	if (hash == NULL) {
		dev_err(dev, "unable to alloc data struct.\n");
		goto err_data;
	}
	hash->dev = dev;

	hash->clk_gate = devm_clk_get(&pdev->dev, "gate_hash");
	if (IS_ERR(hash->clk_gate)) {
		dev_err(&pdev->dev, "cannot find hash clock\n");
		err = PTR_ERR(hash->clk_gate);
		goto err_data;
	}

	clk_prepare_enable(hash->clk_gate);

	platform_set_drvdata(pdev, hash);

	crypto_init_queue(&hash->queue, INGENIC_HASH_QUEUE_LENGTH);
	spin_lock_init(&hash->lock);
	err = ingenic_hash_get_res_pdev(hash, pdev);
	if (err) {
		goto err_data;
	}

	pm_runtime_enable(dev);

	INIT_LIST_HEAD(&hash->list);
	spin_lock(&list_lock);
	list_add_tail(&hash->list, &dev_list);

	spin_unlock(&list_lock);
	for (i = 0; i < hash->pdata->size; i++) {
		algp = &hash->pdata->algs_list[i];
		pr_info("reg alg:  %s\n", algp->halg.base.cra_name);
		//          INIT_LIST_HEAD(&algp->cra_list);
		err = crypto_register_ahash(algp);
		if (err) {
			goto err_algs;
		}
		hash->pdata->registered++;
	}
	printk("ingenic hash[%d] register ok.", hash->pdata->registered);
	return 0;
err_algs:
	for (i = hash->pdata->registered - 1; i >= 0; i--)
		crypto_unregister_ahash(
		    &hash->pdata->algs_list[i]);
	pm_runtime_disable(dev);
err_data:
	dev_err(dev, "initialization failed.\n");
	return err;
}

static int ingenic_hash_remove(struct platform_device *pdev)
{
	struct ingenic_hash_dev *hash = platform_get_drvdata(pdev);
	int i;

	if (!hash) {
		return -ENODEV;
	}

	spin_lock(&list_lock);
	list_del(&hash->list);
	spin_unlock(&list_lock);
	for (i = hash->pdata->registered - 1; i >= 0; i--)
		crypto_unregister_ahash(
		    &hash->pdata->algs_list[i]);
	hash->pdata->registered = 0;
	pm_runtime_disable(hash->dev);
	kfree(hash);
	hash = NULL;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ingenic_hash_suspend(struct device *dev)
{
	pm_runtime_put_sync(dev);
	return 0;
}

static int ingenic_hash_resume(struct device *dev)
{
	pm_runtime_get_sync(dev);
	return 0;
}
#endif

static const struct dev_pm_ops ingenic_hash_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ingenic_hash_suspend, ingenic_hash_resume)
};

static const struct of_device_id ingenic_hash_dt_match[] = {
	{ .compatible = "ingenic,hash", .data = NULL },
	{},
};

MODULE_DEVICE_TABLE(of, ingenic_hash_dt_match);
static struct platform_driver ingenic_hash_driver = {
	.probe  = ingenic_hash_probe,
	.remove = ingenic_hash_remove,
	.driver = {
		.name   = "hash",
		.owner  = THIS_MODULE,
		.pm = &ingenic_hash_pm_ops,
		.of_match_table = of_match_ptr(ingenic_hash_dt_match),
	},
};

module_platform_driver(ingenic_hash_driver);

MODULE_DESCRIPTION("Ingenic MD5,SHA(160,224,256,384,512) hw acceleration support.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Stephon.Qiu");
