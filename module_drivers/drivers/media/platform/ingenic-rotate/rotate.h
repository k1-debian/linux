/* * drivers/media/platform/ingenic_rotate/rotate.h
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

#ifndef __ROTATER_H__
#define __ROTATER_H__

#include <linux/platform_device.h>
#include "v4l2_rotate.h"
#define JZ_ROT_NAME "ingenic-rot"
#define ROT_DESC_NUM 1

#define DEFAULT_WIDTH		(100)
#define DEFAULT_HEIGHT		(100)

#define MIN_WIDTH		(4)
#define MIN_HEIGHT		(4)
#define MAX_WIDTH		(2047)
#define MAX_HEIGHT		(2047)


//#define ROT_QCK_STOP
//#define ROT_GEN_STOP

typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;

typedef union frame_stop {
	uint32_t d32;
	struct {
		uint32_t stop:1;
		uint32_t reserve31_1:31;
	} b;
} frame_stop_t;

typedef union irq_ctrl {
	uint32_t d32;
	struct {
		uint32_t reserve1_0:2;
		uint32_t eof_mask:1;
		uint32_t sof_mask:1;
		uint32_t reserve31_4:28;
	} b;
} irq_ctrl_t;

struct ingenic_rot_desc {
	uint32_t	NextCfgAddr;
	uint32_t	SrcBufferAddr;
	uint32_t	SrcStride;
	frame_stop_t	FrameStop;
	uint32_t	TargetBufferAddr;
	uint32_t	TargetStride;
	irq_ctrl_t	irq_ctrl;
} __attribute__ ((aligned(8)));

struct rot_frm_cfg {
	uint32_t rot_format;
	uint32_t rot_color;
	uint32_t width;
	uint32_t height;
};

struct ingenic_rot_ctx {
	struct ingenic_rot_dev	*dev;
	struct rot_frm_cfg	in;
	struct rot_frm_cfg	out;
	uint32_t		vflip;
	uint32_t		hflip;
	uint32_t		angle;
	struct ingenic_rot_desc		*desc[ROT_DESC_NUM];
	dma_addr_t		desc_phys[ROT_DESC_NUM];
	struct work_struct ingenic_rot_work;
	dma_addr_t src_addr;
	dma_addr_t dst_addr;
	int rot_auto;
//	struct kfifo *fifo;
//	dma_addr_t wb_buf[3];
	int rot_end;
};

struct ingenic_rot_dev {
	const char *name;
	struct device		*dev;
	struct mutex		rot_mutex;
	atomic_t		num_inst;
	void __iomem		*regs;
	struct clk		*clk;
	struct ingenic_rot_ctx	*rot_curr;
	int irq;
	int irq_is_request;
	wait_queue_head_t	irq_queue;
	struct completion   ctx_done;
	struct workqueue_struct *dec_workqueue;
	int flag;
	void *rot_v4l2;
};

static inline unsigned long reg_read(struct ingenic_rot_dev *dev, int offset)
{
	return readl(dev->regs + offset);
}

static inline void reg_write(struct ingenic_rot_dev *dev, int offset, unsigned long val)
{
	writel(val, dev->regs + offset);
}
void rot_manual_auto_mode(struct ingenic_rot_dev *dev);
void rot_set_src_desc(struct ingenic_rot_dev *dev, dma_addr_t addr);
void rot_set_dst_desc(struct ingenic_rot_dev *dev, dma_addr_t addr);
void rot_set_hflip(struct ingenic_rot_dev *dev, uint32_t hflip);
void rot_set_vflip(struct ingenic_rot_dev *dev, uint32_t vflip);
void rot_set_angle(struct ingenic_rot_dev *dev, uint32_t angle);
void rot_set_dst_cfg(struct ingenic_rot_dev *dev, struct rot_frm_cfg *out);
void rot_set_src_cfg(struct ingenic_rot_dev *dev, struct rot_frm_cfg *in);
int wait_rot_state(struct ingenic_rot_dev *dev, int32_t state, uint32_t flag);
void dump_all(struct ingenic_rot_dev *dev);
void dump_rot_desc_reg(struct ingenic_rot_dev *dev);
void dump_rot_desc(struct ingenic_rot_desc *desc);
void rot_clr_irq(struct ingenic_rot_dev *dev);
void rot_gen_stop(struct ingenic_rot_dev *dev);
void rot_qck_stop(struct ingenic_rot_dev *dev);
void rot_start(struct ingenic_rot_dev *dev);
void rot_reset(struct ingenic_rot_dev *dev);
void rot_cpm_reset(void);
struct ingenic_rot_ctx *rot_create_ctx(void);
int rot_destory_ctx(struct ingenic_rot_ctx *ctx);
void rot_do_rotate(struct ingenic_rot_ctx *ctx);
#endif
