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

#ifndef __V4L2_ROTATER_H__
#define __V4L2_ROTATER_H__

#include <linux/platform_device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include "rotate.h"
#include "rotate-regs.h"
#define JZ_ROT_V4L2 "jz-rot"
#define MEM2MEM_CAPTURE	(1 << 0)
#define MEM2MEM_OUTPUT	(1 << 1)
struct v4l2_rot_fmt {
	char	*name;
	u32	fourcc;
	int	depth;
	u32	types;
	u32	src_rot_format;
	u32	src_rot_color;
	u32	dst_rot_format;
};

struct rot_frm_info {
	uint32_t	width;
	uint32_t	height;
	struct v4l2_rot_fmt	*fmt;
	uint32_t	bytesperline;
	uint32_t	size;
};

struct rot_v4l2_ctx {
	struct v4l2_fh		fh;
	struct v4l2_m2m_ctx	*m2m_ctx;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*ctrl_hflip;
	struct v4l2_ctrl	*ctrl_vflip;
	struct v4l2_ctrl	*ctrl_rot;
	struct ingenic_rot_ctx  *rot_ctx;
	struct rot_v4l2_dev *dev;
	struct rot_frm_info	in;
	struct rot_frm_info	out;
};

struct rot_v4l2_dev {
	struct device		*dev;
	struct mutex		mutex;
	spinlock_t		ctrl_lock;
	atomic_t		num_inst;
	struct video_device	*vfd;
	struct v4l2_device	v4l2_dev;
	struct v4l2_m2m_dev	*m2m_dev;
	struct vb2_alloc_ctx	*alloc_ctx;
	struct rot_v4l2_ctx	*curr;
	int v4l2_flag;
};

void *rot_v4l2_init(struct device *dev);
int rot_v4l2_remove(void *priv);
#endif
