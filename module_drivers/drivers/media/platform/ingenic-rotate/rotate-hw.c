/* * drivers/media/platform/ingenic_rotate/rotate-hw.c
 *
 * Copyright (C) 2016 Ingenic Semiconductor Inc.
 *
 * Author:clwang<chunlei.wang@ingenic.com>
 *
 * This program is free software, you can redistribute it and/or modify it
 *
 * under the terms of the GNU General Public License version 2 as published by
 *
 * the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/delay.h>

#include <linux/kfifo.h>
#include "rotate.h"
#include "rotate-regs.h"

void dump_rot_reg(struct ingenic_rot_dev *dev)
{
	printk("-----------------rot_reg------------------\n");
	printk("ROT_FRM_CFG_ADDR:    0x%lx\n",reg_read(dev, ROT_FRM_CFG_ADDR));
	printk("ROT_FRM_SIZE:	     0x%lx\n",reg_read(dev, ROT_FRM_SIZE));
	printk("ROT_GLB_CFG:         0x%lx\n",reg_read(dev, ROT_GLB_CFG));
	printk("ROT_CTRL:            0x%lx\n",reg_read(dev, ROT_CTRL));
	printk("ROT_ST:              0x%lx\n",reg_read(dev, ROT_ST));
	printk("ROT_CLR_ST:          0x%lx\n",reg_read(dev, ROT_CLR_ST));
	printk("ROT_INT_MASK:        0x%lx\n",reg_read(dev, ROT_INT_MASK));
	printk("ROT_RDMA_SITE:       0x%lx\n",reg_read(dev, ROT_RDMA_SITE));
	printk("ROT_WDMA_SITE:       0x%lx\n",reg_read(dev, ROT_WDMA_SITE));
	printk("ROT_QOS_CTRL:        0x%lx\n",reg_read(dev, ROT_QOS_CTRL));
	printk("ROT_QOS_CFG:         0x%lx\n",reg_read(dev, ROT_QOS_CFG));
	printk("-----------------rot_reg------------------\n");
}

void dump_rot_desc(struct ingenic_rot_desc *desc)
{
	printk("-----------------rot_desc------------------\n");
	printk("rot_des		     0x%x\n",(uint32_t)desc);
	printk("NextCfgAddr:         0x%x\n",desc->NextCfgAddr);
	printk("SrcBufferAddr:       0x%x\n",desc->SrcBufferAddr);
	printk("SrcStride:           0x%x\n",desc->SrcStride);
	printk("FrameStop:           0x%x\n",desc->FrameStop.d32);
	printk("TargetBufferAddr:    0x%x\n",desc->TargetBufferAddr);
	printk("TargetStride:        0x%x\n",desc->TargetStride);
	printk("irq_ctrl:            0x%x\n",desc->irq_ctrl.d32);
	printk("-----------------rot_desc------------------\n");
}

void dump_rot_desc_reg(struct ingenic_rot_dev *dev)
{
	reg_write(dev, ROT_CTRL, ROT_DES_CNT_RST);
	printk("-----------------rot_desc_reg------------------\n");
	printk("NextCfgAddr:         0x%lx\n",reg_read(dev, ROT_DS_FRM_DES));
	printk("SrcBufferAddr:       0x%lx\n",reg_read(dev, ROT_DS_FRM_DES));
	printk("SrcStride:           0x%lx\n",reg_read(dev, ROT_DS_FRM_DES));
	printk("FrameStop:           0x%lx\n",reg_read(dev, ROT_DS_FRM_DES));
	printk("TargetBufferAddr:    0x%lx\n",reg_read(dev, ROT_DS_FRM_DES));
	printk("TargetStride:        0x%lx\n",reg_read(dev, ROT_DS_FRM_DES));
	printk("irq_ctrl:            0x%lx\n",reg_read(dev, ROT_DS_FRM_DES));
	printk("-----------------rot_desc_reg------------------\n");
}

void dump_all(struct ingenic_rot_dev *dev)
{
	struct ingenic_rot_ctx *ctx;

	ctx = dev->rot_curr;

	dump_rot_reg(dev);
	dump_rot_desc_reg(dev);
	dump_rot_desc(ctx->desc[0]);
}
void rot_clr_irq(struct ingenic_rot_dev *dev)
{
	uint32_t flag;
	struct ingenic_rot_ctx *ctx;
	ctx = dev->rot_curr;
	flag = reg_read(dev, ROT_INT_MASK) & reg_read(dev, ROT_ST);
	if(flag & ROT_EOF) {
		reg_write(dev, ROT_CLR_ST, ROT_CLR_EOF);
		ctx->rot_end = 1;
	}
	if(flag & ROT_SOF) {
		reg_write(dev, ROT_CLR_ST, ROT_CLR_SOF);
	}
	if(flag & ROT_GEN_STOP_ACK) {
		reg_write(dev, ROT_CLR_ST, ROT_GEN_STOP_ACK);
	}
}

int wait_rot_state(struct ingenic_rot_dev *dev, int32_t state, uint32_t flag)
{
	unsigned long timeout = 100000;
	while((	(!(reg_read(dev, ROT_ST) & state)) == flag) && timeout) {
		timeout--;
		udelay(10);
	}
	if(timeout <= 0) {
		printk("wait state timeout! state = %d, ROT_ST = 0x%lx\n", state, reg_read(dev, ROT_ST));
		return -1;
	}
	return 0;
}

void rot_set_src_desc(struct ingenic_rot_dev *dev, dma_addr_t addr)
{
	struct ingenic_rot_ctx *ctx;
	struct ingenic_rot_desc *desc;
	struct rot_frm_cfg *frm_cfg;

	ctx = dev->rot_curr;
	desc = ctx->desc[0];
	frm_cfg = &ctx->in;

	desc->NextCfgAddr = ctx->desc_phys[0];
	desc->SrcBufferAddr = addr;
	desc->SrcStride = frm_cfg->width;
}

void rot_set_dst_desc(struct ingenic_rot_dev *dev, dma_addr_t addr)
{
	struct ingenic_rot_ctx *ctx;
	struct ingenic_rot_desc *desc;
	struct rot_frm_cfg *frm_cfg;

	ctx = dev->rot_curr;
	desc = ctx->desc[0];
	frm_cfg = &ctx->out;

	desc->TargetBufferAddr = addr;
	if((ctx->angle == 90) || (ctx->angle == 270))
		desc->TargetStride = frm_cfg->height;
	else
		desc->TargetStride = frm_cfg->width;
}

void rot_set_hflip(struct ingenic_rot_dev *dev, uint32_t hflip)
{
	uint32_t cfg;

	cfg = reg_read(dev, ROT_GLB_CFG);
	if(hflip)
		cfg |= ROT_H_MIRROR;
	else
		cfg &= ~ROT_H_MIRROR;

	reg_write(dev, ROT_GLB_CFG, cfg);
}

void rot_set_vflip(struct ingenic_rot_dev *dev, uint32_t vflip)
{
	uint32_t cfg;

	cfg = reg_read(dev, ROT_GLB_CFG);
	if(vflip)
		cfg |= ROT_V_MIRROR;
	else
		cfg &= ~ROT_V_MIRROR;

	reg_write(dev, ROT_GLB_CFG, cfg);
}

void rot_set_angle(struct ingenic_rot_dev *dev, uint32_t angle)
{
	uint32_t cfg;

	cfg = reg_read(dev, ROT_GLB_CFG);
	switch(angle) {
	case 0:
		cfg |= ROT_ANGLE_0;
		break;
	case 90:
		cfg |= ROT_ANGLE_90;
		break;
	case 180:
		cfg |= ROT_ANGLE_180;
		break;
	case 270:
		cfg |= ROT_ANGLE_270;
		break;
	default:
		printk("!!!!!Not support angle!\n");
		break;
	}

	reg_write(dev, ROT_GLB_CFG, cfg);
}

void rot_set_dst_cfg(struct ingenic_rot_dev *dev, struct rot_frm_cfg *out)
{
	uint32_t cfg;

	cfg = reg_read(dev, ROT_GLB_CFG);
	cfg |= out->rot_format;
	reg_write(dev, ROT_GLB_CFG, cfg);
}

void rot_set_src_cfg(struct ingenic_rot_dev *dev, struct rot_frm_cfg *in)
{
	uint32_t cfg;
	uint32_t frm_size = 0;

	frm_size |= in->width << ROT_FRM_WIDTH_LBIT;
	frm_size  |= in->height << ROT_FRM_HEIGHT_LBIT;
	reg_write(dev, ROT_FRM_SIZE, frm_size);

	cfg = reg_read(dev, ROT_GLB_CFG);
	cfg |= in->rot_format;
	cfg |= in->rot_color;
	reg_write(dev, ROT_GLB_CFG, cfg);
}

#define CPM_ROT_SOFT_RESET	(0x1 << 17)
#define CPM_SOFT_RESET		(0xb00000c4)
void rot_cpm_reset(void)
{
	outl(CPM_ROT_SOFT_RESET, CPM_SOFT_RESET);
	udelay(2);
	outl(0, CPM_SOFT_RESET);
}

void rot_reset(struct ingenic_rot_dev *dev)
{
	struct ingenic_rot_ctx *ctx;
	uint32_t cfg = 0;
	int i;

	ctx = dev->rot_curr;

	for(i = 0; i < ROT_DESC_NUM; i++) {
		memset(ctx->desc[i], 0, sizeof(struct ingenic_rot_desc));
	}
#if defined(ROT_GEN_STOP) || defined(ROT_QCK_STOP)
	ctx->desc[0]->FrameStop.b.stop = 0;
#else
	ctx->desc[0]->FrameStop.b.stop = 1;
#endif
	ctx->desc[0]->irq_ctrl.d32 = ROT_EOF_MASK | ROT_SOF_MASK;
	cfg = ROT_WDMA_BURST_32 | ROT_RDMA_BURST_32;
	cfg |= ROT_RDMA_ORDER_RGB;

	/* ahb0 = 300M 720p 60fps */
	reg_write(dev, ROT_QOS_CFG, (reg_read(dev, ROT_QOS_CFG) & (~0xff)) | 13);
	/* Set rot qos value */
	reg_write(dev, ROT_QOS_CTRL, 0x1);
	reg_write(dev, ROT_GLB_CFG, cfg);
	reg_write(dev, ROT_FRM_CFG_ADDR, ctx->desc_phys[0]);
}

void rot_start(struct ingenic_rot_dev *dev)
{
	reg_write(dev, ROT_INT_MASK, ROT_EOF_MASK);
	reg_write(dev, ROT_CTRL, ROT_START);
}

void rot_qck_stop(struct ingenic_rot_dev *dev)
{
	reg_write(dev, ROT_CTRL, ROT_QCK_STP);
	return;
}

void rot_gen_stop(struct ingenic_rot_dev *dev)
{
	reg_write(dev, ROT_CTRL, ROT_GEN_STP);
	return;
}
