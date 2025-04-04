#include <linux/clk.h>
#include <linux/media-bus-format.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/component.h>
#include <linux/mutex.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-dma-contig-ingenic.h>

#include "isp-drv.h"
#define ISP_VIDEO_DRIVER_NAME   "ispvideo"

int isp_max_buffer_num = 3;
module_param(isp_max_buffer_num, int, 0644);
MODULE_PARM_DESC(isp_max_buffer_num, "isp max buffer numer");

int isp_force_img_depth = 0;
module_param(isp_force_img_depth, int, 0644);
MODULE_PARM_DESC(isp_force_img_depth, "force fmt depth, usefull when need mem more than real fmt.");

static struct v4l2_subdev *
isp_video_remote_subdev(struct isp_video_device *ispvideo, u32 *pad)
{
	struct media_pad *remote;

	remote = media_pad_remote_pad_unique(&ispvideo->pad);

	if (remote == NULL || !is_media_entity_v4l2_subdev(remote->entity)) {
		return NULL;
	}

	if (pad) {
		*pad = remote->index;
	}

	return media_entity_to_v4l2_subdev(remote->entity);
}

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int
isp_video_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct isp_video_device *ispvideo = video_drvdata(file);

	strlcpy(cap->driver, ISP_VIDEO_DRIVER_NAME, sizeof(cap->driver));
	strlcpy(cap->card, ispvideo->video.name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s", dev_name(ispvideo->dev));

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_DEVICE_CAPS;

	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static int isp_video_enum_fmt_vid_cap(struct file *file, void *fh,
                                      struct v4l2_fmtdesc *f)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	const struct isp_video_format *fmt = NULL;

	fmt = ispvideo->ops->find_format(NULL, NULL, f->index);
	if (fmt == NULL) {
		return -EINVAL;
	}
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;

	return 0;
}
static int
isp_video_g_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *format)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	struct isp_video_ctx *ctx = ispvideo->ctx;

	//mutex_lock(&ispvideo->mutex);
	if (ctx->format.type == 0) {
		dev_warn(ispvideo->dev, "Format not setted before calling g_fmt\n");
	} else {
		*format = ctx->format;
	}
	//mutex_unlock(&ispvideo->mutex);

	return 0;
}

static int
__isp_video_try_format(struct isp_video_ctx *ctx, struct v4l2_pix_format *pix,
                       const struct isp_video_format **ofmt)
{
	struct isp_video_format *fmt = NULL;
	struct isp_video_device *ispvideo = ctx->ispvideo;
	unsigned int depth = 0;
	unsigned int use_depth = 0;
	int i;

	fmt = ispvideo->ops->find_format(&pix->pixelformat, NULL, -1);
	if (fmt == NULL) {
		dev_err(ispvideo->dev, "Cannot find appropriate format for pix->pixelformat:[%x]\n", pix->pixelformat);
		return -EINVAL;
	}

	/*TODO: handle unsupported fmt using init user fmt*/
#if 0
	pix->width = ((int)pix->width <= 0) ? 1920 : pix->width;
	pix->height = ((int)pix->height <= 0) ? 1080 : pix->height;
#endif

	ispvideo->current_format = fmt;

	for (i = 0; i < fmt->num_planes; i++) {
		depth += fmt->depth[i];
	}

	pix->colorspace = fmt->colorspace;
	pix->field = V4L2_FIELD_NONE;
	pix->pixelformat = fmt->fourcc;
	pix->bytesperline = pix->width * depth / 8;

	use_depth = isp_force_img_depth > depth ? isp_force_img_depth : depth;
	pix->sizeimage = pix->width * use_depth / 8 * pix->height;

	ctx->uv_offset = pix->width * fmt->depth[0] * pix->height / 8;
	ctx->payload_size = pix->bytesperline * pix->height;

	if (ofmt) {
		*ofmt = fmt;
	}

	/*TODO, bound width and height!*/
	//printk("=======%s,%d, pixelformat: %x, widht: %d, height: %d, num_planes: %d, uv_offset: %d\n", __func__, __LINE__, pix->pixelformat, pix->width, pix->height, fmt->num_planes, ctx->uv_offset);

	return 0;
}

static void to_v4l2_mbus_framefmt(struct isp_video_device *ispvideo, struct v4l2_format *format, struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_pix_format *pix = &format->fmt.pix;
	struct isp_video_format *ispfmt = NULL;

	ispfmt = ispvideo->ops->find_format(&pix->pixelformat, NULL, -1);

	fmt->width = pix->width;
	fmt->height = pix->height;
	fmt->code = ispfmt->mbus_code;
	fmt->field = pix->field;
	fmt->colorspace = pix->colorspace;
	fmt->ycbcr_enc = pix->ycbcr_enc;
	fmt->quantization = pix->quantization;
	fmt->xfer_func = pix->xfer_func;
}

static int
isp_video_s_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *format)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	struct isp_video_ctx *ctx = ispvideo->ctx;
	struct v4l2_subdev *sd = NULL;
	struct v4l2_subdev_format subdev_fmt = {0};
	unsigned int pad = 0;
	int ret = 0;

	if (vb2_is_busy(&ctx->queue)) {
		return -EBUSY;
	}

	ret = __isp_video_try_format(ctx, &format->fmt.pix, NULL);
	if (ret < 0) {
		dev_err(ispvideo->dev, "Failed to set format cap!\n");
		return -EINVAL;
	}

	sd = isp_video_remote_subdev(ispvideo, &pad);
	if (sd == NULL) {
		return -EINVAL;
	}
	v4l2_set_subdev_hostdata(sd, ispvideo);

	ctx->format = *format;

	to_v4l2_mbus_framefmt(ispvideo, format, &subdev_fmt.format);

	subdev_fmt.pad = pad;
	subdev_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;

	ret = v4l2_subdev_call(sd, pad, set_fmt, NULL, &subdev_fmt);
	if (ret < 0) {
		dev_err(ispvideo->dev, "Failed to set subdev format\n");
		return -EINVAL;
	}

	return 0;
}

static int
isp_video_try_format_cap(struct file *file, void *fh, struct v4l2_format *format)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	struct isp_video_ctx *ctx = ispvideo->ctx;
	int ret = 0;

	ret = __isp_video_try_format(ctx, &format->fmt.pix, NULL);
	if (ret < 0) {
		dev_err(ispvideo->dev, "Try format cap error!\n");
		return -EINVAL;
	}
	return 0;
}

static int isp_video_pixelaspect(struct file *file, void *fh, int buf_type, struct v4l2_fract *aspect)
{
	printk("-----------%s, %d------------\n", __func__, __LINE__);
	return 0;
}

#if 0
static int
isp_video_cropcap(struct file *file, void *fh, struct v4l2_cropcap *cropcap)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	unsigned int pad;
	struct v4l2_subdev *sd = isp_video_remote_subdev(ispvideo, &pad);

	if (cropcap->type == V4L2_BUF_TYPE_VIDEO_CAPTURE || cropcap->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		return  v4l2_subdev_call(sd, video, cropcap, cropcap);
	} else {
		dev_warn(ispvideo->dev, "unsupported cropcap type!\n");
		return -EINVAL;
	}
}

static int
isp_video_get_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	unsigned int pad;
	struct v4l2_subdev *sd = isp_video_remote_subdev(ispvideo, &pad);

	if (crop->type == V4L2_BUF_TYPE_VIDEO_CAPTURE || crop->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		return v4l2_subdev_call(sd, video, g_crop, crop);
	} else {
		dev_warn(ispvideo->dev, "unsupported crop type!\n");
		return -EINVAL;
	}
}

static int
isp_video_set_crop(struct file *file, void *fh, const struct v4l2_crop *crop)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	unsigned int pad;
	struct v4l2_subdev *sd = isp_video_remote_subdev(ispvideo, &pad);

	if (crop->type == V4L2_BUF_TYPE_VIDEO_CAPTURE || crop->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		return v4l2_subdev_call(sd, video, s_crop, crop);
	} else {
		dev_warn(ispvideo->dev, "unsupported crop type!\n");
		return -EINVAL;
	}
}
#endif

static int isp_video_get_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	unsigned int pad = 0;
	struct v4l2_subdev *sd = isp_video_remote_subdev(ispvideo, &pad);
	struct v4l2_subdev_selection sel = {0};
	int ret = 0;

	if (s->type == V4L2_BUF_TYPE_VIDEO_CAPTURE || s->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		sel.target = s->target;
		sel.pad = pad;
		sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		ret = v4l2_subdev_call(sd, pad, get_selection, NULL, &sel);
		memcpy(&s->r, &sel.r, sizeof(struct v4l2_rect));
		return ret;
	} else {
		dev_warn(ispvideo->dev, "unsupported selection type!\n");
		return -EINVAL;
	}
}

static int isp_video_set_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	unsigned int pad;
	struct v4l2_subdev *sd = isp_video_remote_subdev(ispvideo, &pad);
	struct isp_video_ctx *ctx = ispvideo->ctx;
	struct v4l2_format *format = &ctx->format;
	struct v4l2_subdev_selection sel;
	int ret = 0;

	if (s->type == V4L2_BUF_TYPE_VIDEO_CAPTURE || s->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		sel.pad = pad;
		sel.flags = s->flags;
		sel.target = s->target;
		memcpy(&sel.r, &s->r, sizeof(struct v4l2_rect));
		ret = v4l2_subdev_call(sd, pad, set_selection, NULL, &sel);
		if (ret) {
			return ret;
		}

		format->fmt.pix.width = sel.r.width;
		format->fmt.pix.height = sel.r.height;
		ret = __isp_video_try_format(ctx, &format->fmt.pix, NULL);
		return ret;
	} else {
		dev_warn(ispvideo->dev, "unsupported selection type!\n");
		return -EINVAL;
	}
}

static int isp_video_enum_input(struct file *file, void *priv,
                                struct v4l2_input *inp)
{
	struct isp_video_device *ispvideo = video_drvdata(file);

	if (inp->index != 0) {
		return -EINVAL;
	}

	/* default is camera */
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->std = ispvideo->video.tvnorms;
	strcpy(inp->name, "Camera");

	return 0;
}

static int isp_video_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;

	return 0;
}

static int isp_video_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i > 0) {
		return -EINVAL;
	}

	return 0;
}

int isp_video_enum_framesizes(struct file *file, void *fh, struct v4l2_frmsizeenum *fsize)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	struct ispcam_device    *ispcam = ispvideo->ispcam;
	struct mscaler_device   *mscaler = ispcam->mscaler;
	struct v4l2_subdev_frame_size_enum fse;
	int ret = 0;

	fse.code = fsize->pixel_format;
	fse.index = fsize->index;

	ret = v4l2_subdev_call(&mscaler->sd, pad, enum_frame_size, NULL, &fse);
	if (ret < 0) {
		return ret;
	}

	if (!ispvideo->bypass) {
		fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		fsize->stepwise.min_width = fse.min_width;
		fsize->stepwise.max_width = fse.max_width;
		fsize->stepwise.min_height = fse.min_height;
		fsize->stepwise.max_height = fse.max_height;
		fsize->stepwise.step_width = 2;
		fsize->stepwise.step_height = 2;
	} else {
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = fse.min_width;
		fsize->discrete.height = fse.min_height;
	}
	return 0;
}

int isp_video_g_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	struct ispcam_device    *ispcam = ispvideo->ispcam;
	struct isp_async_device *isd    = ispcam->isd[0];
	int ret = 0;
	struct v4l2_subdev_frame_interval interval;

	interval.pad = 0;
	ret = v4l2_subdev_call(isd->sd, video, g_frame_interval, &interval);
	if (ret < 0) {
		return ret;
	}
	a->parm.capture.timeperframe.numerator = interval.interval.numerator;
	a->parm.capture.timeperframe.denominator = interval.interval.denominator;
	return 0;
}

extern int isp_video_s_ctrl(struct file *file, void *fh, struct v4l2_control *a);
extern int isp_video_g_ctrl(struct file *file, void *fh, struct v4l2_control *a);

extern int bypass_video_s_ctrl(struct file *file, void *fh, struct v4l2_control *a);
extern int bypass_video_g_ctrl(struct file *file, void *fh, struct v4l2_control *a);

static int isp_video_ioctl_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct video_device *vdev = video_devdata(file);
	int ret = 0;
	int plane = 0;
	struct vb2_buffer *vb = NULL;
	dma_addr_t *dma_addr = NULL;
	struct vb2_queue *vb2_q = vdev->queue;
	/* No need to call vb2_queue_is_busy(), anyone can query buffers. */
	ret = vb2_querybuf(vb2_q, p);
	if (ret) {
		return ret;
	}
	// add paddr store in v4l2_buffer reserved
	vb = vb2_q->bufs[p->index];
	if (vb2_q->memory == VB2_MEMORY_MMAP) {
		vb = vb2_q->bufs[p->index];
		if (vb2_q->is_multiplanar) {
			for (plane = 0; plane < vb->num_planes; ++plane) {
				struct v4l2_plane *pdst = &p->m.planes[plane];
				dma_addr = (dma_addr_t *)vb2_plane_cookie(vb, plane);
				if (!dma_addr) {
					//printk("failed get  planes paddr \n");
					ret = -1;
				} else {
					pdst->reserved[0] = *dma_addr;
				}
			}
		} else {
			dma_addr = (dma_addr_t *)vb2_plane_cookie(vb, 0);
			if (!dma_addr) {
				//printk("failed get  planes paddr \n");
				ret = -1;
			} else {
				p->reserved = *dma_addr;
			}
		}
	}

	return ret;
}

static const struct v4l2_ioctl_ops isp_video_ioctl_ops = {
	.vidioc_querycap                = isp_video_querycap,
	.vidioc_enum_fmt_vid_cap    = isp_video_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap       = isp_video_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap       = isp_video_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap     = isp_video_try_format_cap,
	//  .vidioc_cropcap                 = isp_video_cropcap,
	//  .vidioc_g_crop                  = isp_video_get_crop,
	//  .vidioc_s_crop                  = isp_video_set_crop,
	.vidioc_g_selection             = isp_video_get_selection,
	.vidioc_s_selection             = isp_video_set_selection,
	.vidioc_g_pixelaspect       = isp_video_pixelaspect,
	.vidioc_reqbufs                 = vb2_ioctl_reqbufs,
	.vidioc_querybuf                = isp_video_ioctl_querybuf,
	.vidioc_qbuf                    = vb2_ioctl_qbuf,
	.vidioc_dqbuf                   = vb2_ioctl_dqbuf,
	.vidioc_expbuf          = vb2_ioctl_expbuf,
	.vidioc_streamon                = vb2_ioctl_streamon,
	.vidioc_streamoff               = vb2_ioctl_streamoff,
	.vidioc_enum_input      = isp_video_enum_input,
	.vidioc_g_input         = isp_video_g_input,
	.vidioc_s_input         = isp_video_s_input,
	.vidioc_enum_framesizes     = isp_video_enum_framesizes,
	.vidioc_g_parm          = isp_video_g_parm,
	.vidioc_s_ctrl          = isp_video_s_ctrl,
	.vidioc_g_ctrl          = isp_video_g_ctrl,
};

static const struct v4l2_ioctl_ops bypass_video_ioctl_ops = {
	.vidioc_querycap                = isp_video_querycap,   /**/
	.vidioc_enum_fmt_vid_cap    = isp_video_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap       = isp_video_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap       = isp_video_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap     = isp_video_try_format_cap,
	.vidioc_g_pixelaspect       = isp_video_pixelaspect,
	.vidioc_reqbufs                 = vb2_ioctl_reqbufs,
	.vidioc_querybuf                = vb2_ioctl_querybuf,
	.vidioc_qbuf                    = vb2_ioctl_qbuf,
	.vidioc_dqbuf                   = vb2_ioctl_dqbuf,
	.vidioc_expbuf          = vb2_ioctl_expbuf,
	.vidioc_streamon                = vb2_ioctl_streamon,
	.vidioc_streamoff               = vb2_ioctl_streamoff,
	.vidioc_enum_input      = isp_video_enum_input, /**/
	.vidioc_g_input         = isp_video_g_input,
	.vidioc_s_input         = isp_video_s_input,
	.vidioc_enum_framesizes     = isp_video_enum_framesizes,
	.vidioc_g_parm          = isp_video_g_parm,  /**/
	//  .vidioc_s_ctrl          = bypass_video_s_ctrl,
	//  .vidioc_g_ctrl          = bypass_video_g_ctrl,
};

/* -----------------------------------------------------------------------------
 * Video queue operations
 */

static int isp_video_queue_setup(struct vb2_queue *queue,
                                 unsigned int *count, unsigned int *num_planes,
                                 unsigned int sizes[], struct device *alloc_ctxs[])
{
	struct isp_video_ctx *ctx = vb2_get_drv_priv(queue);
	struct isp_video_device *ispvideo = ctx->ispvideo;
	struct v4l2_format *format = &ctx->format;

	*num_planes = 1;

	sizes[0] = format->fmt.pix.sizeimage;
	if (sizes[0] == 0) {
		dev_err(ispvideo->dev, "queue setup 0 sizeimage\n");
		return -EINVAL;
	}

	alloc_ctxs[0] = ispvideo->ispcam->dev;

	/*MAX_BUFFER_NUMS:*/
	if (ispvideo->bypass) {
		isp_max_buffer_num = 5;
	}
	if (*count >= isp_max_buffer_num) {
		*count = isp_max_buffer_num;
	}

	ispvideo->max_buffer_num = *count;

	return 0;
}

static int isp_video_buffer_prepare(struct vb2_buffer *buf)
{
	struct isp_video_ctx *ctx = vb2_get_drv_priv(buf->vb2_queue);

	//vb2_set_plane_payload(buf, 0, format->fmt.pix.sizeimage);
	vb2_set_plane_payload(buf, 0, ctx->payload_size);
	return 0;
}

static void isp_video_buffer_queue(struct vb2_buffer *buf)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(buf);
	struct isp_video_ctx *ctx = vb2_get_drv_priv(buf->vb2_queue);
	struct isp_video_buffer *isp_buffer = to_isp_buffer(vbuf);
	struct isp_video_device *ispvideo = ctx->ispvideo;
	int ret = 0;

	isp_buffer->uv_offset = ctx->uv_offset;

	ret = ispvideo->ops->qbuf(ispvideo, isp_buffer);
	if (ret < 0) {
		dev_err(ispvideo->dev, "failed to queue buf!\n");
	}
}

static int isp_video_subdev_init(struct isp_video_device *ispvideo)
{
	struct media_entity *entity = &ispvideo->video.entity;
	struct v4l2_subdev *sd = NULL;
	struct media_pad *pad = NULL;
	int ret = 0;

	/*subdevs reset.*/
	while (1) {
		pad = &entity->pads[0];
		/*直到找到该pipeline上的仅有一个FL_SOURCE pad设备.*/
		if (!(pad->flags & MEDIA_PAD_FL_SINK)) {
			break;
		}

		pad = media_pad_remote_pad_unique(pad);
		if (pad == NULL || !is_media_entity_v4l2_subdev(pad->entity)) {
			break;
		}
		entity = pad->entity;
		sd = media_entity_to_v4l2_subdev(entity);

		/*stream on*/
		ret = v4l2_subdev_call(sd, core, init, 1);
		if (ret < 0 && ret != -ENOIOCTLCMD) {
			dev_err(ispvideo->dev, "failed to init subdev[%s]\n", entity->name);
			break;
		} else {
			ret = 0;
		}
	}

	return ret;
}

static int isp_video_subdev_reset(struct isp_video_device *ispvideo)
{
	struct media_entity *entity = &ispvideo->video.entity;
	struct v4l2_subdev *sd = NULL;
	struct media_pad *pad = NULL;
	int ret = 0;

	/*subdevs reset.*/
	while (1) {
		pad = &entity->pads[0];
		/*直到找到该pipeline上的仅有一个FL_SOURCE pad设备.*/
		if (!(pad->flags & MEDIA_PAD_FL_SINK)) {
			break;
		}

		pad = media_pad_remote_pad_unique(pad);
		if (pad == NULL || !is_media_entity_v4l2_subdev(pad->entity)) {
			break;
		}
		entity = pad->entity;
		sd = media_entity_to_v4l2_subdev(entity);

		/*stream on*/
		ret = v4l2_subdev_call(sd, core, reset, 1);
		if (ret < 0 && ret != -ENOIOCTLCMD) {
			dev_err(ispvideo->dev, "failed to reset subdev[%s]\n", entity->name);
		} else {
			ret = 0;
		}
	}

	return ret;

}

static int isp_video_subdev_streamon(struct isp_video_device *ispvideo)
{
	struct ispcam_device    *ispcam = ispvideo->ispcam;
	struct mscaler_device   *mscaler = ispcam->mscaler;
	struct isp_device   *isp    = ispcam->isp;
	struct vic_device   *vic    = ispcam->vic;
	struct csi_device   *csi    = ispcam->csi;
	struct isp_async_device *isd = ispcam->isd[0];
	int ret = 0;

	if (!ispvideo->bypass) {
		ret = v4l2_subdev_call(&isp->sd, video, s_stream, 1);
		if (ret < 0) {
			goto isp_streamon_err;
		}

		v4l2_set_subdev_hostdata(&mscaler->sd, ispvideo);
		ret = v4l2_subdev_call(&mscaler->sd, video, s_stream, 1);
		if (ret < 0) {
			goto mscaler_streamon_err;
		}
	}

	if (csi && (isd->bus_type == V4L2_MBUS_CSI2_DPHY)) {
		ret = v4l2_subdev_call(&csi->sd, video, s_stream, 1);
		if (ret < 0) {
			goto csi_streamon_err;
		}
	}

	if (!isd->enabled) {
		ret = v4l2_subdev_call(isd->sd, video, s_stream, 1);
		if (ret < 0) {
			goto isd_streamon_err;
		}
	}
	isd->enabled++;

	v4l2_set_subdev_hostdata(&vic->sd, ispvideo);
	ret = v4l2_subdev_call(&vic->sd, video, s_stream, 1);
	if (ret < 0) {
		goto vic_streamon_err;
	}

	return ret;
vic_streamon_err:
	isd->enabled--;
	if (!isd->enabled) {
		ret = v4l2_subdev_call(isd->sd, video, s_stream, 0);
	}
isd_streamon_err:
	if (csi && (isd->bus_type == V4L2_MBUS_CSI2_DPHY)) {
		v4l2_subdev_call(&csi->sd, video, s_stream, 0);
	}
csi_streamon_err:
	if (!ispvideo->bypass) {
		v4l2_subdev_call(&isp->sd, video, s_stream, 0);
	}
isp_streamon_err:
	if (!ispvideo->bypass) {
		v4l2_subdev_call(&mscaler->sd, video, s_stream, 0);
	}
mscaler_streamon_err:
	dev_err(ispvideo->dev, "start stream error\n");
	return ret;
}

static int isp_video_subdev_streamoff(struct isp_video_device *ispvideo)
{
	struct ispcam_device    *ispcam = ispvideo->ispcam;
	struct mscaler_device   *mscaler = ispcam->mscaler;
	struct isp_device   *isp    = ispcam->isp;
	struct vic_device   *vic    = ispcam->vic;
	struct csi_device   *csi    = ispcam->csi;
	struct isp_async_device *isd = ispcam->isd[0];
	int ret = 0;

	//global ispcam device mutex.
	if (!ispvideo->bypass) {
		v4l2_set_subdev_hostdata(&mscaler->sd, ispvideo);
		ret = v4l2_subdev_call(&mscaler->sd, video, s_stream, 0);
		if (ret < 0) {
			goto err;
		}

		ret = v4l2_subdev_call(&isp->sd, video, s_stream, 0);
		if (ret < 0) {
			goto err;
		}
	}

	if (csi && (isd->bus_type == V4L2_MBUS_CSI2_DPHY)) {
		ret = v4l2_subdev_call(&csi->sd, video, s_stream, 0);
		if (ret < 0) {
			goto err;
		}
	}

	isd->enabled--;
	if (!isd->enabled) {
		ret = v4l2_subdev_call(isd->sd, video, s_stream, 0);
		if (ret < 0) {
			goto err;
		}
	}

	v4l2_set_subdev_hostdata(&vic->sd, ispvideo);
	ret = v4l2_subdev_call(&vic->sd, video, s_stream, 0);
	if (ret < 0) {
		goto err;
	}

	return ret;
err:
	dev_err(ispvideo->dev, "start stream error\n");
	return ret;

}

static int isp_video_subdev_stream(struct isp_video_device *ispvideo, int enable)
{
	int ret = 0;

	//mutex lock.

	if (enable) {
		ret = isp_video_subdev_streamon(ispvideo);
	} else {
		ret = isp_video_subdev_streamoff(ispvideo);
	}

	return ret;
}

DEFINE_MUTEX(global_mutex);
static int isp_video_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct isp_video_ctx *ctx = vb2_get_drv_priv(q);
	struct isp_video_device *ispvideo = ctx->ispvideo;
	struct media_entity *entity = &ispvideo->video.entity;
	struct isp_pipeline *pipe = &ispvideo->pipe;
	struct ispcam_device    *ispcam = ispvideo->ispcam;
	int ret = 0;

	mutex_lock(&global_mutex);

	mutex_lock(&ispvideo->stream_lock);

	if (ispcam->enabled++ == 0) {
		ret = video_device_pipeline_start(&ispvideo->video, &pipe->pipeline);
		if (ret < 0) {
			dev_err(ispvideo->dev, "Failed to start pipeline\n");
			goto err_pipeline_start;
		}

		ret = isp_video_subdev_reset(ispvideo);
		ret = isp_video_subdev_init(ispvideo);
		if (ret) {
			dev_err(ispvideo->dev, "Failed to init subdev\n");
			goto err_subdev_init;
		}
	}

	ret = isp_video_subdev_stream(ispvideo, 1);

	if (ret < 0) {
		goto err_start_stream;
	}

	mutex_unlock(&ispvideo->stream_lock);

	mutex_unlock(&global_mutex);
	return 0;

err_start_stream:
err_subdev_init:
	media_pipeline_stop(entity->pads);
err_pipeline_start:
	ispcam->enabled--;
	mutex_unlock(&ispvideo->stream_lock);
	mutex_unlock(&global_mutex);
	return ret;
}

extern int tiziano_stop(struct v4l2_subdev *sd);
static void isp_video_stop_streaming(struct vb2_queue *q)
{
	struct isp_video_ctx *ctx = vb2_get_drv_priv(q);
	struct isp_video_device *ispvideo = ctx->ispvideo;
	struct ispcam_device    *ispcam = ispvideo->ispcam;
	unsigned int is_streaming = 0;
	int ret = 0;

	mutex_lock(&global_mutex);

	mutex_lock(&ispvideo->stream_lock);

	is_streaming = vb2_is_streaming(q);

	if (!is_streaming) {
		goto done;
	}

	ret = isp_video_subdev_stream(ispvideo, 0);

	if (--ispcam->enabled > 0) {
		goto finish;
	}

	media_pipeline_stop(ispvideo->video.entity.pads);

finish:
done:
	mutex_unlock(&ispvideo->stream_lock);
	mutex_unlock(&global_mutex);
}

static const struct vb2_ops isp_video_queue_ops = {
	.queue_setup = isp_video_queue_setup,
	.buf_prepare = isp_video_buffer_prepare,
	.buf_queue = isp_video_buffer_queue,
	.start_streaming = isp_video_start_streaming,
	.stop_streaming = isp_video_stop_streaming,
};

static int isp_video_open(struct file *file)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	struct video_device *vdev = &ispvideo->video;
	struct isp_video_ctx *ctx = NULL;
	struct vb2_queue *queue = NULL;
	int ret = 0;

	mutex_lock(&ispvideo->mutex);
	v4l2_fh_open(file);
	if (v4l2_fh_is_singular(file->private_data)) {
		ctx = kzalloc(sizeof(struct isp_video_ctx), GFP_KERNEL);
		if (IS_ERR_OR_NULL(ctx)) {
			dev_err(ispvideo->ispcam->dev, "Failed to alloc ctx for isp_video_ctx\n");
			ret = -ENOMEM;
			goto err_ctx_alloc;
		}

		queue = &ctx->queue;
		queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		queue->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
		queue->drv_priv = ctx;
		queue->ops = &isp_video_queue_ops;
		queue->mem_ops = &ingenic_vb2_dma_contig_memops;
		queue->buf_struct_size = sizeof(struct isp_video_buffer);
		queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
		queue->lock = &ispvideo->queue_lock;

		ret = vb2_queue_init(&ctx->queue);
		if (ret < 0) {
			goto err_vb2_queue_init;
		}

		ctx->ispvideo = ispvideo;
		ispvideo->ctx = ctx;
		vdev->queue = ispvideo->queue = queue;
		ctx->fh = file->private_data;

		/*TODO:init user fmt*/
#if 0
		ctx->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		ctx->format.fmt.pix.width = 1920;
		ctx->format.fmt.pix.height = 1080;

		if (!ispvideo->bypass) {
			ctx->format.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
			ctx->format.fmt.pix.bytesperline = 1920 * 12 / 8;
			ctx->format.fmt.pix.sizeimage = 1920 * 1080 * 1.5;
		} else {
			ctx->format.fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR10;
			ctx->format.fmt.pix.bytesperline = 1920 * 16 / 8;
			ctx->format.fmt.pix.sizeimage = 1920 * 1080 * 2;
		}

		ctx->format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
		ctx->format.fmt.pix.field = V4L2_FIELD_NONE;
#endif

	} else {
		v4l2_fh_release(file);
		mutex_unlock(&ispvideo->mutex);
		return -EBUSY;
	}
	mutex_unlock(&ispvideo->mutex);

	return 0;
err_vb2_queue_init:
	kfree(ctx);
err_ctx_alloc:
	v4l2_fh_release(file);
	mutex_unlock(&ispvideo->mutex);
	return ret;
}
static int isp_video_release(struct file *file)
{
	struct isp_video_device *ispvideo = video_drvdata(file);
	struct video_device *vdev = &ispvideo->video;
	struct isp_video_ctx *ctx = ispvideo->ctx;

	mutex_lock(&ispvideo->mutex);
	if (v4l2_fh_is_singular(file->private_data)) {

		vb2_queue_release(&ctx->queue);
		kfree(ctx);
		vdev->queue = ispvideo->queue = NULL;

	}
	v4l2_fh_release(file);
	file->private_data = NULL;
	mutex_unlock(&ispvideo->mutex);

	return 0;
}

const static struct v4l2_file_operations isp_video_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = isp_video_open,
	.release = isp_video_release,
	.poll = vb2_fop_poll, /*isp_video_poll,*/
	.mmap = vb2_fop_mmap, /*isp_video_mmap,*/
};

int isp_video_init(struct isp_video_device *ispvideo, char *name, const struct isp_video_ops *video_ops)
{
	int ret = 0;
	struct video_device *video = &ispvideo->video;
	struct media_entity *entity = &video->entity;
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);


	/*初始化一个sink pad*/
	ispvideo->pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_pads_init(&video->entity, 1, &ispvideo->pad);
	if (ret < 0) {
		/*TODO: error case*/
	}
	sd->owner = THIS_MODULE;
	sd->flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_IO_V4L;

	mutex_init(&ispvideo->queue_lock);
	mutex_init(&ispvideo->mutex);
	mutex_init(&ispvideo->stream_lock);

	snprintf(video->name, sizeof(video->name), "isp-%s", name);
	video->fops = &isp_video_fops;

	if (ispvideo->bypass) {
		video->ioctl_ops = &bypass_video_ioctl_ops;
	} else {
		video->ioctl_ops = &isp_video_ioctl_ops;
	}

	video->release = video_device_release_empty;
	video->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	ispvideo->name = video->name;
	ispvideo->ops = video_ops;
	ispvideo->dev = ispvideo->ispcam->dev;

	video_set_drvdata(&ispvideo->video, ispvideo);

	return 0;

}

int isp_video_cleanup(struct isp_video_device *ispvideo)
{

	return 0;
}

int isp_video_register(struct isp_video_device *ispvideo, struct v4l2_device *v4l2_dev, int nr)
{
	struct video_device *video = &ispvideo->video;
	int ret = 0;

	video->v4l2_dev = v4l2_dev;

	ret = video_register_device(video, VFL_TYPE_VIDEO, nr);
	if (ret < 0) {
		/*TODO: error case*/
	}

	dev_info(ispvideo->dev, "register video device %s @ /dev/video%d ok\n", video->name, video->num);

	return ret;
}
int isp_video_unregister(struct isp_video_device *ispvideo)
{

	return 0;
}
