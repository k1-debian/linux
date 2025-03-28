/*
* Copyright© 2014 Ingenic Semiconductor Co.,Ltd
*
* Author: qipengzhen <aric.pzqi@ingenic.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#include <linux/slab.h>
#include <media/v4l2-mem2mem.h>

#include "helix_buf.h"
#include "helix_drv.h"

#include "jpge.h"

#include "jpge/ht.h"
#include "jpge/qt.h"
#include "jpge/head.h"

static void __maybe_unused dump_slice_info(_JPEGE_SliceInfo *s)
{
	printk("s->des_va:	%08x\n", (unsigned int)s->des_va);
	printk("s->des_pa:	%08x\n", s->des_pa);
	printk("s->ncol:	%d\n", s->ncol);
	printk("s->rsm:		%d\n", s->rsm);
	printk("s->bsa:		%08x\n", s->bsa);
	printk("s->p0a:		%08x\n", s->p0a);
	printk("s->p1a:		%08x\n", s->p1a);
	printk("s->nrsm:	%d\n", s->nrsm);
	printk("s->raw[0]:	%08x\n", s->raw[0]);
	printk("s->raw[1]:	%08x\n", s->raw[1]);
	printk("s->raw[2]:	%08x\n", s->raw[2]);
	printk("s->stride[0]:	%d\n", s->stride[0]);
	printk("s->stride[1]:	%d\n", s->stride[1]);
	printk("s->mb_height:	%d\n", s->mb_height);
	printk("s->mb_width:	%d\n", s->mb_width);
	printk("s->nmcu:	%d\n", s->nmcu);
	printk("s->raw_format:	%d(%s)\n", s->raw_format,
	       s->raw_format == 8 ? "NV12" :
	       s->raw_format == 12 ? "NV21" :
	       s->raw_format == 0 ? "TILE(unsupported)" : "invalid");
	printk("s->ql_sel:	%d(%s)\n", s->ql_sel,
	       s->ql_sel == LOW_QUALITY ? "low_quality" :
	       s->ql_sel == MEDIUMS_QUALITY ? "mediums_quality" :
	       s->ql_sel == HIGH_QUALITY ? "high_quality" : "invld");
	printk("s->huffenc_sel:	%d\n", s->huffenc_sel);
}

static int jpge_fill_slice_info(struct jpge_ctx *ctx)
{
	struct jpge_params *p = &ctx->p;
	_JPEGE_SliceInfo *s = ctx->s;
	int ret = 0;

	s->des_va = ctx->desc;
	s->des_pa = ctx->desc_pa;

	s->ncol = 2;    /* unused? */
	s->rsm = 0;
	s->bsa = ctx->bs->pa + ctx->header_size;
	s->p0a = 0;
	s->p1a = 0;
	s->nrsm = 0;

	s->raw[0] = ctx->frame->fb_addr[0].pa;  /*Y*/
	s->raw[1] = ctx->frame->fb_addr[1].pa;  /*U for 420p or UV for nv12*/
	s->raw[2] = ctx->frame->fb_addr[2].pa;  /*V for 420p*/

	s->stride[0] = s->stride[1] = p->width;

	s->mb_height = (p->height + 15) / 16;
	s->mb_width = p->width / 16;
	s->nmcu = s->mb_height * s->mb_width - 1;
	s->raw_format = p->format;
	s->ql_sel = p->compr_quality;
	s->huffenc_sel = 0; /*only one huffenc table?*/

	return ret;
}

static int jpge_gen_header(struct jpge_ctx *ctx)
{
	struct ingenic_vcodec_mem *bs = ctx->bs;
	struct jpge_params *p = &ctx->p;
	int ql_sel = p->compr_quality;
	char *pbuf = bs->va;
	char *ptr = NULL;
	int i, j;
	int header_size;
	int padsize;

	/* SOI 文件开始 */
	*pbuf++ = 0xff;
	*pbuf++ = M_SOI;

	/* DQT -- 0 */
	*pbuf++ = 0xff;
	*pbuf++ = M_DQT;
	*pbuf++ = 0x0;
	*pbuf++ = 0x43;
	*pbuf++ = 0x0;

	ptr = (char *)&qt[ql_sel][0];
	for (i = 0; i < 64; i++) {
		*pbuf++ = *ptr++;
	}

	/* DQT -- 1*/
	*pbuf++ = 0xff;
	*pbuf++ = M_DQT;
	*pbuf++ = 0;
	*pbuf++ = 0x43;
	*pbuf++ = 0x01;
	for (i = 0; i < 64; i++) {
		*pbuf++ = *ptr++;
	}

	/* SOF */
	*pbuf++ = 0xff;
	*pbuf++ = M_SOF0;
	*pbuf++ = 0x0;
	*pbuf++ = 0x11;             //Lf = 17
	*pbuf++ = 8;                //8bit sample
	*pbuf++ = (p->height & 0xff00) >> 8;    //Y=height
	*pbuf++ = p->height & 0xff;
	*pbuf++ = (p->width & 0xff00) >> 8; //X=width
	*pbuf++ = p->width & 0xff;
	*pbuf++ = 3;                //Nf=3, number of component, Y U V
	*pbuf++ = 1;                //采样系数, Component Y 设置.
	*pbuf++ = 0x22;             //Hori:2 Vertical:2 水平采样系数和垂直采样系数.
	*pbuf++ = 0x00;             //使用量化表0.
	*pbuf++ = 2;                //Component Cb.
	*pbuf++ = 0x11;             //H:1 V:1
	*pbuf++ = 0x1;              //使用量化表1.
	*pbuf++ = 3;                //Component Cr.
	*pbuf++ = 0x11;             //H:1 V:1
	*pbuf++ = 0x1;              //使用量化表1.

	/* DHT -- lumia DC/AC, Chromia DC/AC */
	for (j = 0; j < 4; j++) {
		*pbuf++ = 0xff;
		*pbuf++ = M_DHT;
		*pbuf++ = (ht_size[j] & 0xff00) >> 8;   //ht_size
		*pbuf++ = ht_size[j] & 0xff;
		*pbuf++ = dht_sel[j];           //dht_sel; ?
		for (i = 0; i < 16; i++) {
			*pbuf++ = ht_len[j][i];
		}

		for (i = 0; i < 16; i++) {
			int m;
			for (m = 0; m < ht_len[j][i]; m++) {
				*pbuf++ = ht_val[j][i][m];
			}
		}
	}

	/*添加0xff,填充header到256字节对齐. 已知SOS 段14字节*/
	header_size = pbuf - (char *)bs->va + 14;
	padsize = 256 - header_size % 256;

	memset(pbuf, 0xff, padsize);
	pbuf += padsize;

	//header_size = (header_size + 255) / 256 * 256; // align to 256;

	/* SOS */
	*pbuf++ = 0xff;
	*pbuf++ = M_SOS;
	//Ls = 12
	*pbuf++ = 0x0;
	*pbuf++ = 0xC;
	//Ns
	*pbuf++ = 0x3;
	//Cs1 - Y
	*pbuf++ = 0x1;
	//Td, Ta
	*pbuf++ = 0x00;
	//Cs2 - U
	*pbuf++ = 0x2;
	//Td, Ta
	*pbuf++ = 0x11;
	//Cs3 - V
	*pbuf++ = 0x3;
	//Td, Ta
	*pbuf++ = 0x11;
	//Ss
	*pbuf++ = 0x00;
	//Se
	*pbuf++ = 0x3f;
	//Ah, Al
	*pbuf++ = 0x0;

	ctx->header_size = pbuf - (char *)bs->va;
	return 0;
}

int jpeg_encoder_encode(struct jpge_ctx *ctx, struct video_frame_buffer *frame, struct ingenic_vcodec_mem *bs)
{
	int ret = 0;
	char *pbuf = NULL;

	ctx->frame = frame;
	bs->va = (void *)((unsigned long)bs->va | 0xa0000000);  //使用uncache地址生成头信息.影响效率.
	ctx->bs = bs;

	ret = jpge_gen_header(ctx);

	jpge_fill_slice_info(ctx);
	//dump_slice_info(ctx->s);

	JPEGE_SliceInit(ctx->s);

	ret = ingenic_vpu_start(ctx->priv);
	if (ret < 0) {
		return ret;
	}

	/* Modify bslen.
	    total bslen = header_size + vpu encoded bslen.
	*/
	ctx->bslen = ctx->header_size + ctx->bslen;

	pbuf = bs->va + ctx->bslen;
	/* EOI */
	*pbuf++ = 0xff;
	*pbuf++ = 0xd9;

	ctx->bslen += 2;

	return ret;
}

int jpeg_encoder_set_fmt(struct jpge_ctx *ctx, int width, int height, int format)
{
	struct jpge_params *p = &ctx->p;
	int ret = 0;

	p->width = width;
	p->height = height;

	switch (format) {
	case HELIX_NV12_MODE:
	case HELIX_NV21_MODE:
		p->format = format;
		break;
	default:
		pr_err("Unsupported pix fmt: %x\n", format);
		ret = -EINVAL;
		break;
	}

	return ret;
}

int jpeg_encoder_alloc_workbuf(struct jpge_ctx *ctx)
{
	struct ingenic_venc_ctx *venc_ctx = ctx->priv;
	int ret = 0;

	ctx->desc = dma_alloc_coherent(venc_ctx->dev->dev, ctx->vdma_chain_len, &ctx->desc_pa, GFP_KERNEL);
	if (!ctx->desc) {
		pr_err("Failed to alloc desc memory!\n");
		ret = -ENOMEM;
		goto err_desc;
	}

	return ret;
err_desc:
	return ret;
}

int jpeg_encoder_free_workbuf(struct jpge_ctx *ctx)
{
	struct ingenic_venc_ctx *venc_ctx = ctx->priv;
	dma_free_coherent(venc_ctx->dev->dev, ctx->vdma_chain_len, ctx->desc, ctx->desc_pa);

	return 0;
}

int jpeg_encoder_init(struct jpge_ctx *ctx)
{
	struct jpge_params *p = NULL;
	_JPEGE_SliceInfo *s = NULL;

	s = kzalloc(sizeof(_JPEGE_SliceInfo), GFP_KERNEL);
	if (!s) {
		return -ENOMEM;
	}

	p = &ctx->p;
	ctx->s = s;
	ctx->vdma_chain_len = 40960 + 256;

	return 0;
}

int jpeg_encoder_deinit(struct jpge_ctx *ctx)
{
	if (!ctx) {
		return 0;
	}

	if (ctx->s) {
		kfree(ctx->s);
	}

	return 0;
}

void jpeg_encoder_set_priv(struct jpge_ctx *ctx, void *data)
{
	ctx->priv = data;
}
