/* * drivers/media/platform/ingenic_rotate/rotate.c
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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>

#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
//#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-dma-contig-ingenic.h>
#include <media/ingenic_video_nr.h>

#include "v4l2_rotate.h"
#include "rotate-regs.h"

#define fh2ctx(__fh) container_of(__fh, struct rot_v4l2_ctx, fh)

struct v4l2_rot_fmt v4l2_rot_formats[] = {
	{
		.name	= "ARGB_8888",
		.fourcc	= V4L2_PIX_FMT_ARGB32,
		.depth	= 32,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
		.src_rot_format = ROT_RDMA_FMT_ARGB8888,
		.src_rot_color = ROT_RDMA_ORDER_RGB,
		.dst_rot_format = ROT_WDMA_FMT_ARGB8888,
	},
	{
		.name	= "RGB_8888",
		.fourcc	= V4L2_PIX_FMT_RGB32,
		.depth	= 32,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
		.src_rot_format = ROT_RDMA_FMT_ARGB8888,
		.src_rot_color = ROT_RDMA_ORDER_RGB,
		.dst_rot_format = ROT_WDMA_FMT_ARGB8888,
	},
	{
		.name	= "RGB_8888",
		.fourcc	= V4L2_PIX_FMT_XRGB32,
		.depth	= 32,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
		.src_rot_format = ROT_RDMA_FMT_RGB888,
		.src_rot_color = ROT_RDMA_ORDER_RGB,
		.dst_rot_format = ROT_WDMA_FMT_ARGB8888,
	},
	{
		.name	= "ABGR_8888",
		.fourcc	= V4L2_PIX_FMT_ABGR32,
		.depth	= 32,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
		.src_rot_format = ROT_RDMA_FMT_ARGB8888,
		.src_rot_color = ROT_RDMA_ORDER_BGR,
	},
	{
		.name	= "BGR_8888",
		.fourcc	= V4L2_PIX_FMT_BGR32,
		.depth	= 32,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
		.src_rot_format = ROT_RDMA_FMT_ARGB8888,
		.src_rot_color = ROT_RDMA_ORDER_BGR,
	},
	{
		.name	= "BGR_8888",
		.fourcc	= V4L2_PIX_FMT_XBGR32,
		.depth	= 32,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
		.src_rot_format = ROT_RDMA_FMT_RGB888,
		.src_rot_color = ROT_RDMA_ORDER_BGR,
	},
	{
		.name	= "RGB_565",
		.fourcc	= V4L2_PIX_FMT_RGB565,
		.depth	= 16,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
		.src_rot_format = ROT_RDMA_FMT_RGB565,
		.src_rot_color = ROT_RDMA_ORDER_RGB,
		.dst_rot_format = ROT_WDMA_FMT_RGB565,
	},
	{
		.name	= "RGB_555",
		.fourcc	= V4L2_PIX_FMT_RGB555,
		.depth	= 16,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
		.src_rot_format = ROT_RDMA_FMT_RGB555,
		.src_rot_color = ROT_RDMA_ORDER_RGB,
		.dst_rot_format = ROT_WDMA_FMT_RGB555,
	},
	{
		.name	= "ARGB_1555",
		.fourcc	= V4L2_PIX_FMT_ARGB555,
		.depth	= 16,
		.types	= MEM2MEM_OUTPUT,
		.src_rot_format = ROT_RDMA_FMT_RGB1555,
		.src_rot_color = ROT_RDMA_ORDER_RGB,
	},
	{
		.name	= "YUV 422P",
		.fourcc	= V4L2_PIX_FMT_YUYV,
		.depth	= 16,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
		.src_rot_format = ROT_RDMA_FMT_YUV422,
		.src_rot_color = ROT_RDMA_ORDER_RGB,
		.dst_rot_format = ROT_WDMA_FMT_YUV422,
	},
};
#define NUM_FORMATS ARRAY_SIZE(v4l2_rot_formats)

static struct v4l2_rot_fmt *find_fmt(struct v4l2_format *f)
{
	unsigned int i;
	for (i = 0; i < NUM_FORMATS; i++) {
		if (v4l2_rot_formats[i].fourcc == f->fmt.pix.pixelformat)
			return &v4l2_rot_formats[i];
	}
	return NULL;
}

static int _v4l2_vidioc_expbuf(struct file *file, void* prv,
		  struct v4l2_exportbuffer *eb)
{
	struct rot_v4l2_ctx *ctx = fh2ctx(prv);
	if(ctx->m2m_ctx){
		return v4l2_m2m_expbuf(file,ctx->m2m_ctx,eb);
	}
	return -EINVAL;
}


static struct rot_frm_info *get_frame(struct rot_v4l2_ctx *ctx,
		enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return &ctx->in;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return &ctx->out;
	default:
		return ERR_PTR(-EINVAL);
	}
}

static int v4l2_queue_setup(struct vb2_queue *vq,
		unsigned int *nbuffers, unsigned int *nplanes,
		unsigned int sizes[], struct device *alloc_ctxs[])

{
	struct rot_v4l2_ctx *ctx = vb2_get_drv_priv(vq);
	struct rot_frm_info *f = get_frame(ctx, vq->type);

	if (IS_ERR(f))
		return PTR_ERR(f);
	*nplanes = 1;
	sizes[0] = f->size;
	alloc_ctxs[0] = ctx->dev->dev;

	if (*nbuffers == 0)
		*nbuffers = 1;

	return 0;
}

static int v4l2_buf_prepare(struct vb2_buffer *vb)
{
	struct rot_v4l2_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct rot_frm_info *f = get_frame(ctx, vb->vb2_queue->type);

	if (IS_ERR(f))
		return PTR_ERR(f);
	vb2_set_plane_payload(vb, 0, f->size);
	return 0;
}

static void v4l2_buf_queue(struct vb2_buffer *vb)
{
	struct rot_v4l2_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (ctx->m2m_ctx)
		v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}


static struct vb2_ops v4l2_qops = {
	.queue_setup	= v4l2_queue_setup,
	.buf_prepare	= v4l2_buf_prepare,
	.buf_queue	= v4l2_buf_queue,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
						struct vb2_queue *dst_vq)
{
	struct rot_v4l2_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &v4l2_qops;
	src_vq->mem_ops = &ingenic_vb2_dma_contig_memops;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->dev->mutex;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &v4l2_qops;
	dst_vq->mem_ops = &ingenic_vb2_dma_contig_memops;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->dev->mutex;

	return vb2_queue_init(dst_vq);
}

static int rot_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct rot_v4l2_ctx *ctx = container_of(ctrl->handler, struct rot_v4l2_ctx,
								ctrl_handler);
	unsigned long flags;

	spin_lock_irqsave(&ctx->dev->ctrl_lock, flags);
	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		ctx->rot_ctx->hflip = ctx->ctrl_hflip->val;
		break;
	case V4L2_CID_VFLIP:
		ctx->rot_ctx->vflip = ctx->ctrl_vflip->val;
		break;
	case V4L2_CID_ROTATE:
		ctx->rot_ctx->angle = ctx->ctrl_rot->val;
		break;
	}
	spin_unlock_irqrestore(&ctx->dev->ctrl_lock, flags);
	return 0;
}

static const struct v4l2_ctrl_ops rot_ctrl_ops = {
	.s_ctrl		= rot_s_ctrl,
};

static int v4l2_setup_ctrls(struct rot_v4l2_ctx *ctx)
{
	struct rot_v4l2_dev *dev = ctx->dev;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 3);

	ctx->ctrl_hflip = v4l2_ctrl_new_std(&ctx->ctrl_handler, &rot_ctrl_ops,
						V4L2_CID_HFLIP, 0, 1, 1, 0);

	ctx->ctrl_vflip = v4l2_ctrl_new_std(&ctx->ctrl_handler, &rot_ctrl_ops,
						V4L2_CID_VFLIP, 0, 1, 1, 0);
	ctx->ctrl_rot = v4l2_ctrl_new_std(&ctx->ctrl_handler, &rot_ctrl_ops,
						V4L2_CID_ROTATE, 0, 270, 90, 0);

	if (ctx->ctrl_handler.error) {
		int err = ctx->ctrl_handler.error;
		v4l2_err(&dev->v4l2_dev, "v4l2_setup_ctrls failed\n");
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		return err;
	}

	return 0;
}

static int v4l2_open(struct file *file)
{
	struct rot_v4l2_dev *dev = video_drvdata(file);
	struct rot_v4l2_ctx *ctx = NULL;
	int ret = 0;
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	if (mutex_lock_interruptible(&dev->mutex)) {
		kfree(ctx);
		return -ERESTARTSYS;
	}

	ctx->rot_ctx = rot_create_ctx();
	if(ctx->rot_ctx == NULL)
		goto free;
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	ctx->dev = dev;
	ctx->m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, &queue_init);
	if (IS_ERR(ctx->m2m_ctx)) {
		ret = PTR_ERR(ctx->m2m_ctx);
		goto err;
	}
	ctx->fh.m2m_ctx = ctx->m2m_ctx;

	ret = v4l2_setup_ctrls(ctx);
	if(ret)
		goto err;
	/* Write the default values to the ctx struct */
	v4l2_ctrl_handler_setup(&ctx->ctrl_handler);
	mutex_unlock(&dev->mutex);
	return 0;
err:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
free:
	mutex_unlock(&dev->mutex);
	kfree(ctx);
	return ret;
}

static int v4l2_release(struct file *file)
{
	struct rot_v4l2_dev *dev = video_drvdata(file);
	struct rot_v4l2_ctx *ctx = fh2ctx(file->private_data);
	mutex_lock(&dev->mutex);
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	mutex_unlock(&dev->mutex);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	rot_destory_ctx(ctx->rot_ctx);
	kfree(ctx);
	return 0;
}

static int vidioc_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	strncpy(cap->driver, JZ_ROT_V4L2, sizeof(cap->driver) - 1);
	strncpy(cap->card, JZ_ROT_V4L2, sizeof(cap->card) - 1);
	cap->bus_info[0] = 0;
	cap->version = KERNEL_VERSION(1, 0, 0);
	/*
	 * This is only a mem-to-mem video device. The capture and output
	 * device capability flags are left only for backward compatibility
	 * and are scheduled for removal.
	 */
	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M;
	cap->capabilities =  cap->device_caps |
		V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OUTPUT |
		V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int enum_fmt(struct v4l2_fmtdesc *f, u32 type)
{
	int i, num;
	struct v4l2_rot_fmt *fmt;

	num = 0;
	for (i = 0; i < NUM_FORMATS; ++i) {
		if (v4l2_rot_formats[i].types & type) {
			/* index-th format of type type found ? */
			if (num == f->index)
				break;
			/* Correct type but haven't reached our index yet,
			 * just increment per-type index */
			++num;
		}
	}

	if (i < NUM_FORMATS) {
		/* Format found */
		fmt = &v4l2_rot_formats[i];
		strncpy(f->description, fmt->name, sizeof(f->description) - 1);
		f->pixelformat = fmt->fourcc;
		return 0;
	}

	/* Format not found */
	return -EINVAL;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, MEM2MEM_CAPTURE);
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, MEM2MEM_OUTPUT);
}

static inline struct rot_v4l2_ctx *fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct rot_v4l2_ctx, fh);
}

static int vidioc_g_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
	struct rot_v4l2_ctx *ctx = fh_to_ctx(prv);
	struct vb2_queue *vq;
	struct rot_frm_info *frm;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;
	frm = get_frame(ctx, f->type);
	if (IS_ERR(frm))
		return PTR_ERR(frm);

	f->fmt.pix.width		= frm->width;
	f->fmt.pix.height		= frm->height;
	f->fmt.pix.field		= V4L2_FIELD_NONE;
	f->fmt.pix.pixelformat		= frm->fmt->fourcc;
	f->fmt.pix.bytesperline		= (frm->width * frm->fmt->depth) >> 3;
	f->fmt.pix.sizeimage		= frm->size;
	return 0;
}

static int vidioc_try_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
	struct v4l2_rot_fmt *fmt;
	enum v4l2_field *field;

	fmt = find_fmt(f);
	if (!fmt)
		return -EINVAL;

	field = &f->fmt.pix.field;
	if (*field == V4L2_FIELD_ANY)
		*field = V4L2_FIELD_NONE;
	else if (*field != V4L2_FIELD_NONE)
		return -EINVAL;

	if (f->fmt.pix.width > MAX_WIDTH
		|| f->fmt.pix.height > MAX_HEIGHT
		|| f->fmt.pix.width < MIN_WIDTH
		|| f->fmt.pix.height < MIN_HEIGHT) {
		return -EINVAL;
	}

	f->fmt.pix.bytesperline = (f->fmt.pix.width * fmt->depth) >> 3;
	f->fmt.pix.sizeimage = f->fmt.pix.height * f->fmt.pix.bytesperline;
	return 0;
}

static int vidioc_s_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
	struct rot_v4l2_ctx *ctx = fh_to_ctx(prv);
	struct rot_v4l2_dev *dev = ctx->dev;
	struct vb2_queue *vq;
	struct rot_frm_info *frm;
	struct v4l2_rot_fmt *fmt;
	int ret = 0;

	/* Adjust all values accordingly to the hardware capabilities
	 * and chosen format. */
	ret = vidioc_try_fmt(file, prv, f);
	if (ret)
		return ret;
	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (vb2_is_busy(vq)) {
		v4l2_err(&dev->v4l2_dev, "queue (%d) bust\n", f->type);
		return -EBUSY;
	}
	frm = get_frame(ctx, f->type);
	if (IS_ERR(frm))
		return PTR_ERR(frm);
	fmt = find_fmt(f);
	if (!fmt)
		return -EINVAL;
	frm->width	= f->fmt.pix.width;
	frm->height	= f->fmt.pix.height;
	frm->size	= f->fmt.pix.sizeimage;
	frm->fmt	= fmt;
	frm->bytesperline	= f->fmt.pix.bytesperline;
	return 0;
}

static int vidioc_streamon(struct file *file, void *priv,
					enum v4l2_buf_type type)
{
	struct rot_v4l2_ctx *ctx = priv;

	return v4l2_m2m_streamon(file, ctx->m2m_ctx, type);
}

static int vidioc_streamoff(struct file *file, void *priv,
					enum v4l2_buf_type type)
{
	struct rot_v4l2_ctx *ctx = priv;

	return v4l2_m2m_streamoff(file, ctx->m2m_ctx, type);
}

static void job_abort(void *prv)
{
	printk("-----rot job_abort\n");
}


static void device_run(void *prv)
{
	struct rot_v4l2_ctx *ctx = prv;
	struct rot_v4l2_dev *dev = ctx->dev;
	struct vb2_v4l2_buffer *src, *dst;

	src = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);

	ctx->rot_ctx->src_addr = ingenic_vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
	ctx->rot_ctx->dst_addr = ingenic_vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	ctx->rot_ctx->in.rot_format = ctx->in.fmt->src_rot_format;
	ctx->rot_ctx->in.rot_color = ctx->in.fmt->src_rot_color;
	ctx->rot_ctx->in.width = ctx->in.width;
	ctx->rot_ctx->in.height = ctx->in.height;
	ctx->rot_ctx->out.rot_format = ctx->out.fmt->dst_rot_format;
	ctx->rot_ctx->out.width = ctx->out.width;
	ctx->rot_ctx->out.height = ctx->out.height;

	rot_do_rotate(ctx->rot_ctx);

	src = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	if(unlikely(src == NULL)) {
		printk("Rotater:src == NULL\n");
	}
	dst = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
	if(unlikely(dst == NULL)) {
		printk("Rotater:dst == NULL\n");
	}

	v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
	v4l2_m2m_job_finish(dev->m2m_dev, ctx->m2m_ctx);
}


static const struct v4l2_file_operations v4l2_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_open,
	.release	= v4l2_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static const struct v4l2_ioctl_ops v4l2_ioctl_ops = {
	.vidioc_querycap		= vidioc_querycap,

	.vidioc_enum_fmt_vid_cap	= vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= vidioc_g_fmt,
	.vidioc_try_fmt_vid_cap		= vidioc_try_fmt,
	.vidioc_s_fmt_vid_cap		= vidioc_s_fmt,

	.vidioc_enum_fmt_vid_out	= vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out		= vidioc_g_fmt,
	.vidioc_try_fmt_vid_out		= vidioc_try_fmt,
	.vidioc_s_fmt_vid_out		= vidioc_s_fmt,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf          = _v4l2_vidioc_expbuf,

	.vidioc_streamon		= vidioc_streamon,
	.vidioc_streamoff		= vidioc_streamoff,
};

static struct video_device v4l2_videodev = {
	.name		= "ingenic-rot-v4l2",
	.fops		= &v4l2_fops,
	.ioctl_ops	= &v4l2_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,
	.vfl_dir	= VFL_DIR_M2M,
};

static struct v4l2_m2m_ops v4l2_m2m_ops = {
	.device_run	= device_run,
	.job_abort	= job_abort,
};

void * rot_v4l2_init(struct device *dev)
{
	struct rot_v4l2_dev *rot_dev;
	struct video_device *vfd;
	int ret = 0;

	rot_dev = devm_kzalloc(dev, sizeof(*rot_dev), GFP_KERNEL);
	if (!rot_dev)
		return NULL;

	rot_dev->dev = dev;
	spin_lock_init(&rot_dev->ctrl_lock);
	mutex_init(&rot_dev->mutex);
//	rot_dev->alloc_ctx = ingenic_vb2_dma_contig_init_ctx(rot_dev->dev);
//	if (IS_ERR(rot_dev->alloc_ctx)) {
//		ret = PTR_ERR(rot_dev->alloc_ctx);
//	}

	ret = v4l2_device_register(rot_dev->dev, &rot_dev->v4l2_dev);
	if (ret)
		goto alloc_ctx_cleanup;

	vfd = &v4l2_videodev;
	vfd->lock = &rot_dev->mutex;
	vfd->v4l2_dev = &rot_dev->v4l2_dev;
	vfd->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M;
	ret = video_register_device(vfd, VFL_TYPE_VIDEO, INGENIC_ROTATE_VIDEO_NR);
	if (ret) {
		v4l2_err(&rot_dev->v4l2_dev, "Failed to register video device\n");
		goto free_video_device;
	}

	video_set_drvdata(vfd, rot_dev);
	snprintf(vfd->name, sizeof(vfd->name), "%s", v4l2_videodev.name);
	rot_dev->vfd = vfd;
	v4l2_info(&rot_dev->v4l2_dev, "device registered as /dev/video%d\n",vfd->num);

	rot_dev->m2m_dev = v4l2_m2m_init(&v4l2_m2m_ops);

	if (IS_ERR(rot_dev->m2m_dev)) {
		v4l2_err(&rot_dev->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(rot_dev->m2m_dev);
		goto unreg_video_dev;
	}

	return rot_dev;

unreg_video_dev:
	video_unregister_device(vfd);
free_video_device:
	v4l2_device_unregister(&rot_dev->v4l2_dev);
alloc_ctx_cleanup:
//	ingenic_vb2_dma_contig_cleanup_ctx(rot_dev->alloc_ctx);
	return NULL;
}

int rot_v4l2_remove(void *priv)
{
	struct rot_v4l2_dev *dev = (struct rot_v4l2_dev *)priv;

	v4l2_m2m_release(dev->m2m_dev);
	video_unregister_device(dev->vfd);
	video_device_release(dev->vfd);
	v4l2_device_unregister(&dev->v4l2_dev);
//	ingenic_vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
	return 0;
}

