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
#include "jpgd.h"

static void __maybe_unused dump_slice_info(_JPEGD_SliceInfo *s)
{
	printk("s->des_va:	%08x\n", (unsigned int)s->des_va);
	printk("s->des_pa:	%08x\n", s->des_pa);
	printk("s->bsa:		%08x\n", s->bsa);
	printk("s->p0a:		%08x\n", s->p0a);
	printk("s->p1a:		%08x\n", s->p1a);
	printk("s->nrsm:	%d\n", s->nrsm);
	printk("s->nmcu:	%d\n", s->nmcu);
	printk("s->pxc[0]:	%x\n", s->pxc[0]);
	printk("s->pxc[1]:	%x\n", s->pxc[1]);
	printk("s->pxc[2]:	%x\n", s->pxc[2]);
	printk("s->pxc[3]:	%x\n", s->pxc[3]);
}

static int jpgd_fill_slice_info(struct jpgd_ctx *ctx)
{
	struct jpgd_params *p = &ctx->p;
	_JPEGD_SliceInfo *s = ctx->s;
	int ret = 0;

	s->des_va   = ctx->desc;
	s->des_pa   = ctx->desc_pa;
	s->bsa      = ctx->bs->pa;
	s->p0a      = ctx->frame->fb_addr[0].pa;
	s->p1a      = ctx->frame->fb_addr[1].pa;
	s->nrsm     = 0;
	s->nmcu     = ((p->height + 15) / 16) * ((p->width + 15) / 16) - 1;
	s->huffmin  = p->huffmin;
	s->huffbase     = p->hea;
	s->huffsymb     = p->heb;
	s->qmem         = (int *)p->qt;

	if (p->format == HELIX_NV12_MODE) {
		s->width = 1 << 15 | ((p->width + 15) / 16 - 1);
	}
	s->pxc[0]   = p->pxc[0];
	s->pxc[1]   = p->pxc[1];
	s->pxc[2]   = p->pxc[2];
	s->pxc[3]   = p->pxc[3];

	return ret;
}

/* decode header info to params ??*/
static int jpgd_decode_header(struct jpgd_ctx *ctx)
{
	struct ingenic_venc_ctx *venc_ctx = ctx->priv;
	struct ingenic_vcodec_mem *bs = ctx->bs;
	struct jpgd_params *p = &ctx->p;
	unsigned char *pbuf = bs->va;
	int bslen = bs->size;
	unsigned char *bsend = bs->va + bs->size;

	int hbase, hb, abase, bbase, dc, ht;
	int i, j, n, v, c, nrst, nm, code, hid;

	int ncol;
	int l[16];
	int nblk[4] = {0};
	int qt_sel[4] = {0};
	int ha_sel[4] = {0};
	int hd_sel[4] = {0};
	int is_sos = 0;

	unsigned char *dst_bs = bs->va;
	unsigned char *dst_bslen = 0;
	unsigned int min0, min1, min2, min3;
	int tmp = 0;

	nrst = nm = ncol = 0;
	/* init default table ?*/
	for (i = 0; i < 384; i++) {
		p->huffenc[i] = 0xfff;
	}
	for (i = 168, j = 0xfd0; i < 176; i++, j++) {
		p->huffenc[i] = j;
	}
	for (i = 344, j = 0xfd0; i < 352; i++, j++) {
		p->huffenc[i] = j;
	}

	for (i = 162; i < 174; i++) {
		p->heb[i] = 0;
	}

	for (i = 0; i < 4; i++)
		for (j = 0; j < 64; j++) {
			p->qt[i][j] = 0;
		}

	while (1) {

		if (is_sos || pbuf >= bsend) {
			break;
		}

		c = *pbuf++;
		if (c != 0xff) {
			continue;
		}

		while ((c = *pbuf++) == 0xff);
		if (c == 0) {
			/* not a marker */
			continue;
		}

		switch (c) {
		case 0xc0 : // Baseline (0, 0)
		case 0xc1 : // Ext. Sequential, Huffman (0, 0)
		case 0xc2 : // Progressive, Huffman (1, 0)
		case 0xc3 : // Lossless, Huffman
		case 0xc5 : // Differential Sequential, Huffman
		case 0xc6 : // Differential Progressive, Huffman
		case 0xc7 : // Differential Lossless, Huffman
		case 0xc9 : // Extended Sequential, Arithmetic (0, 1)
		case 0xca : // Progressive, Arithmetic (1, 1)
		case 0xcb : // Lossless, Huffman
		case 0xcd : // Differential Sequential, Arithmetic
		case 0xce : // Differential Progressive, Arithmetic
		case 0xcf : // Differential Lossless, Arithmetic

			pbuf += 3;
			tmp = *pbuf++ << 8;
			p->height = tmp | *pbuf++;
			tmp = *pbuf++ << 8;
			p->width  = tmp | *pbuf++;
			ncol      = *pbuf++;

			for (i = 0; i < ncol; i++) {
				pbuf++;
				nblk[i] = *pbuf++;
				qt_sel[i] = *pbuf++;
			}
			break;
		case 0xc4 :
			/* DHT marker detected */
			/* Get the lenght of the marker segment */
			// Lh : HT length (16b)
			tmp = *pbuf++ << 8;
			n = tmp | *pbuf++;

			/* reduce 2 ?*/
			n -= 2;

			while (n) {
				/* Get the type of table */
				v = *pbuf++; // Tc & Th
				// Tc : Table class (4b)
				//      0 = DC or lossless table
				//      1 = AC table
				// Th : HT destination identifier
				// - specifies 1 of 4 possible destinations at the decoder into
				// which HT shall be installed.

				/* Reduce marker segment byte count */
				n--;

				hid = v >> 4 ? 2 : 0;
				hid |= v & 15 ? 1 : 0;
				switch (hid) {
				case 1:
					hbase = 368;
					break;
				case 2:
					hbase = 0;
					break;
				case 3:
					hbase = 176;
					break;
				default :
					hbase = 352;
					break;
				}
				if ((v >> 4)) {
					abase = 0;
				} else {
					abase = 1;
				}

				dc = abase;
				ht = v & 15;
				abase |= ht << 1;
				switch (abase) {
				case 1 :
				case 3 :
					bbase = 162;
					break;
				case 2 :
					bbase = 174;
					break;
				default :
					bbase = 0;
					break;
				}
				abase <<= 4;

				/* Memory initialization */
				for (i = abase; i < abase + 16; i++) {
					p->hea[i] = 255;
				}
				/* Get the number of codes for each length */
				// Lj : # of Huffman codes of length i
				// - specifies the # of Huffman codes for each of 16 possible lengths
				// allowed by spec. BITS
				for (i = 0; i < 16; i++) {
					l[i] = *pbuf++;
				}
				/* Reduce marker segment byte count */
				n -= 16;
				code = 0;
				for (i = 0; i < 16; i++, abase++) {
					p->min[abase] = code;
					p->hea[abase] = bbase - code;
					if (l[i]) {
						// Vi,j : associated with each Huffman code
						// - specifies, for each i the value associated with each Huffman code
						// of length i.  HUFFVAL
						for (j = 0; j < l[i]; j++, bbase++) {
							v = *pbuf++;
							/* Reduce marker segment byte count */
							n--;
							if (dc) {
								p->huffenc[hbase + v] = (i << 8) | (code & 0xff);
								v &= 15;
								if (ht) {
									v <<= 4;
								}
								p->heb[bbase] |= v;
							} else {
								if (v == 0) {
									hb = 160;
								} else if (v == 0xf0) {
									hb = 161;
								} else {
									hb = (v >> 4) * 10 + (v & 0xf) - 1;
								}
								p->huffenc[hbase + hb] = (i << 8) | (code & 0xff);
								p->heb[bbase] = v;
							}
							code++;
						}

					}
					code <<= 1;
				}
			}
			break;
		case 0xc8 :
			break;

		case 0xcc :
			break;
		/* M_RST0 ~ M_RST7, ignore? */
		case 0xd0 :
		case 0xd1 :
		case 0xd2 :
		case 0xd3 :
		case 0xd4 :
		case 0xd5 :
		case 0xd6 :
		case 0xd7 :
			break;
		/* M_SOI */
		case 0xd8 :
			break;
		/* M_EOI */
		case 0xd9 :
			*dst_bs++ = 0xff;
			dst_bslen++;
			*dst_bs++ = 0xd9;
			dst_bslen++;
			break;
		/* M_SOS */
		case 0xda :
			pbuf += 2;
			n = *pbuf++; //Ns (# of image components)

			for (i = 0; i < n; i++) {
				// Cs, Td&Ta
				pbuf++;

				ha_sel[i] = hd_sel[i] = *pbuf++;
				ha_sel[i] = (ha_sel[i] & 0x30) >> 4;
				hd_sel[i] = hd_sel[i] & 0x3;
			}
			pbuf++; //Ss
			pbuf++; //Se
			pbuf++; //Ah&Al
			/* TODO... */
			/* BS left can be decoded by vpu.*/
			is_sos = 1;

			break;
		case 0xdb :
			// Lq : QT Length (16b)
			tmp = *pbuf++ << 8;
			v = tmp | *pbuf++;
			int len = v - 2;
			while (len > 0) {
				int prec;
				v = *pbuf++;
				// Pq : QT element precision (4b)
				// - specifies the precision of the Qk values.
				//   0 indicates 8-bits Qk values.
				//   1 indicates 16-bits Qk values
				prec = v >> 4;
				// Tq : QT destination identifier (4b)
				// - specifies one of 4 possible destnations at the decoder into
				// which the QT shall be installed.
				n = v & 15;
				if (n > 3) {
					/*ijpegd_log(h, C_LOG_DEBUG, "error QT\n");*/
				}
				for (i = 0; i < 64; i++) {
					// Qk: Quantization table element
					// k is the index in the zigzag ordering of the DCT coeff
					// JPC only do 8-bit Qk! (ie, Pq shall be 0)
					if (prec) {
						tmp = *pbuf++ << 8;
						p->qt[n][i] = tmp | *pbuf++;
					} else {
						p->qt[n][i] = (*pbuf++);
					}
				}
				len -= 64 + 1;
				if (prec) {
					len -= 64;
				}
			}
			break;
		case 0xdd :
			// Lr : restart interval segment length (16b)
			// - specifies the length of the paramenters in the DRI segment
			pbuf += 2;
			// Ri : restart interval (16b)
			// - specifies the number of MCU in the restart interval.
			tmp = *pbuf++ << 8;
			nrst = tmp | *pbuf++;
			break;
		case 0xe0 :/* All these markers are ignored */
		case 0xe1 :
		case 0xe2 :
		case 0xe3 :
		case 0xe4 :
		case 0xe5 :
		case 0xe6 :
		case 0xe7 :
		case 0xe8 :
		case 0xe9 :
		case 0xea :
		case 0xeb :
		case 0xec :
		case 0xed :
		case 0xee :
		case 0xef :
		case 0xf0 :
		case 0xf1 :
		case 0xf2 :
		case 0xf3 :
		case 0xf4 :
		case 0xf5 :
		case 0xf6 :
		case 0xf7 :
		case 0xf8 :
		case 0xf9 :
		case 0xfa :
		case 0xfb :
		case 0xfc :
		case 0xfd :
		case 0xfe :
			tmp = *pbuf++ << 8;
			v = tmp | *pbuf++;
			v -= 2;
			pbuf += v;
			break;
		default :
			break;
		}

	}

	for (i = 0; i < 64;) {
		v = p->min[i++] & 1;
		v <<= 2;
		v |= p->min[i++] & 3;
		v <<= 3;
		v |= p->min[i++] & 7;
		min3 = (v >> 2) & 0xF;
		//ijpegd_log(h, C_LOG_DEBUG, fpo,"%x",v>>2);
		v <<= 4;
		v |= p->min[i++] & 15;
		v <<= 5;
		v |= p->min[i++] & 31;
		v <<= 6;
		v |= p->min[i++] & 63;
		v <<= 7;
		v |= p->min[i++] & 127;
		v <<= 8;
		v |= p->min[i++] & 255;
		min2 = v & 0xFFFFFFFF;
		//ijpegd_log(h, C_LOG_DEBUG, fpo,"%08x",v);
		v = p->min[i++] & 255;
		v <<= 8;
		v |= p->min[i++] & 255;
		v <<= 8;
		v |= p->min[i++] & 255;
		v <<= 8;
		v |= p->min[i++] & 255;
		//ijpegd_log(h, C_LOG_DEBUG, fpo,"%08x",v);
		min1 = v & 0xFFFFFFFF;
		v = p->min[i++] & 255;
		v <<= 8;
		v |= p->min[i++] & 255;
		v <<= 8;
		v |= p->min[i++] & 255;
		v <<= 8;
		v |= p->min[i++] & 255;
		min0 = v & 0xFFFFFFFF;

		p->huffmin[i / 4 - 4] = min0;
		p->huffmin[i / 4 - 3] = min1;
		p->huffmin[i / 4 - 2] = min2;
		p->huffmin[i / 4 - 1] = min3;
	}

	/* huffbase */
	for (i = 0; i < 64; i++) { // hea
		p->hea[i] = p->hea[i] & 0x1FF;
	}
	/* huffsymb */
	for (i = 0; i < 336; i++) {
		p->heb[i] = p->heb[i] & 0xFF;
	}

	for (i = 0; i < 4; i++) {
		p->pxc[i] = (nblk[i] & 0x3) * ((nblk[i] & 0x30) >> 4) - 1;
		p->pxc[i] = (p->pxc[i] & 0xf) << 4;
		p->pxc[i] |= qt_sel[i] << 2 |
		             ((ha_sel[i] & 0x1) << 1) |
		             ((hd_sel[i] & 0x1) << 0);
	}

	ctx->header_size = pbuf - (unsigned char *)bs->va;

	memcpy(bs->va, pbuf, bslen - ctx->header_size);
	dma_sync_single_for_device(venc_ctx->dev->dev, bs->pa, bslen - ctx->header_size, DMA_TO_DEVICE);

	return 0;
}

int jpeg_decoder_decode(struct jpgd_ctx *ctx, struct ingenic_vcodec_mem *bs, struct video_frame_buffer *frame)
{
	int ret = 0;

	ctx->frame = frame;
	ctx->bs = bs;

	ret = jpgd_decode_header(ctx);

	jpgd_fill_slice_info(ctx);
	//dump_slice_info(ctx->s);

	JPEGD_SliceInit(ctx->s);

	ret = ingenic_vpu_start(ctx->priv);
	if (ret < 0) {
		return ret;
	}

	return ret;
}

int jpeg_decoder_set_fmt(struct jpgd_ctx *ctx, int width, int height, int format)
{
	struct jpgd_params *p = &ctx->p;
	int ret = 0;

	switch (format) {
	case HELIX_NV12_MODE:
	case HELIX_TILE_MODE:
		p->format = format;
		break;
	default:
		pr_err("Unsupported pix fmt: %x\n", format);
		ret = -EINVAL;
		break;
	}

	return ret;
}

int jpeg_decoder_alloc_workbuf(struct jpgd_ctx *ctx)
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
int jpeg_decoder_free_workbuf(struct jpgd_ctx *ctx)
{
	struct ingenic_venc_ctx *venc_ctx = ctx->priv;
	dma_free_coherent(venc_ctx->dev->dev, ctx->vdma_chain_len, ctx->desc, ctx->desc_pa);

	return 0;
}

int jpeg_decoder_init(struct jpgd_ctx *ctx)
{
	struct jpgd_params *p = NULL;
	_JPEGD_SliceInfo *s = NULL;

	s = kzalloc(sizeof(_JPEGD_SliceInfo), GFP_KERNEL);
	if (!s) {
		return -ENOMEM;
	}

	p = &ctx->p;
	ctx->s = s;
	ctx->vdma_chain_len = 40960 + 256;

	return 0;
}

int jpeg_decoder_deinit(struct jpgd_ctx *ctx)
{
	if (!ctx) {
		return 0;
	}

	if (ctx->s) {
		kfree(ctx->s);
	}

	return 0;
}

void jpeg_decoder_set_priv(struct jpgd_ctx *ctx, void *data)
{
	ctx->priv = data;
}
