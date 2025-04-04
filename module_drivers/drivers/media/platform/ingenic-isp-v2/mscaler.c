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
#include <linux/time.h>
#include <linux/delay.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig-ingenic.h>
#include <media/videobuf2-v4l2.h>
#include <media/ingenic_video_nr.h>

#include "isp-drv.h"
#include "mscaler-regs.h"

static void dump_mscaler_regs(struct mscaler_device *mscaler);

static struct isp_video_format mscaler_output_formats[] = {
	{
		.name     = "NV12, Y/CbCr 4:2:0",
		.fourcc   = V4L2_PIX_FMT_NV12,
		.depth    = {8, 4},
		.num_planes = 2,
		.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "NV21, Y/CrCb 4:2:0",
		.fourcc   = V4L2_PIX_FMT_NV21,
		.depth    = {8, 4},
		.num_planes = 2,
		.mbus_code = MEDIA_BUS_FMT_YVYU8_2X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.name     = "GREY, Greyscale",
		.fourcc   = V4L2_PIX_FMT_GREY,
		.depth    = {8},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
#ifdef CONFIG_MSCA_BDEV
	{
		.name     = "JFIF JPEG",
		.fourcc   = V4L2_PIX_FMT_JPEG,
		.depth    = {8},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_JPEG_1X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
	},
	{
		.name     = "H264 with start codes",
		.fourcc   = V4L2_PIX_FMT_H264,
		.depth    = {8},
		.num_planes = 1,
		.mbus_code = MEDIA_BUS_FMT_JPEG_1X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
	},
#endif
	/*
	{
	    .name     = "RGB565, RGB-5-6-5",
	    .fourcc   = V4L2_PIX_FMT_RGB565,
	    .depth    = {16},
	    .num_planes = 1,
	    .mbus_code = MEDIA_BUS_FMT_RGB565_1X16,
	    .colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
	    .name     = "ARGB32, ARGB-8-8-8-8",
	    .fourcc   = V4L2_PIX_FMT_ARGB32,
	    .depth    = {32},
	    .num_planes = 1,
	    .mbus_code = MEDIA_BUS_FMT_ARGB8888_1X32,
	    .colorspace = V4L2_COLORSPACE_SRGB,
	},
	*/
};

struct mask_kobj_attr {
	struct kobj_attribute attr;
	unsigned int id;
};

#define to_mask_attr(attr)     \
	container_of(attr, struct mask_kobj_attr, attr)

static inline void mscaler_reg_writel(struct mscaler_device *mscaler, unsigned int reg, unsigned int val)
{
	writel(val, mscaler->iobase + reg);
}

static inline unsigned int mscaler_reg_readl(struct mscaler_device *mscaler, unsigned int reg)
{
	return readl(mscaler->iobase + reg);
}

/* isp video 回调函数，用来寻找当前支持的输出视频格式，由于mscaler可以输出，vic也可以输出，所以这里设计成回调的方式.*/

/**
 * @brief
 *
 * @param pixelformat   如果pixelformat不为空，则使用pixelformat匹配.
 * @param mbus_code
 * @param index
 *
 * @return
 */
struct isp_video_format *mscaler_find_format(const u32 *pixelformat, const u32 *mbus_code, int index)
{
	int i;
	struct isp_video_format *fmt = NULL;

	for (i = 0; i < ARRAY_SIZE(mscaler_output_formats); i++) {
		fmt = &mscaler_output_formats[i];

		if (pixelformat && fmt->fourcc == *pixelformat) {
			return fmt;
		}
		if (mbus_code && fmt->mbus_code == *mbus_code) {
			return fmt;
		}

		if (index == i) {
			return fmt;
		}
	}

	return NULL;
}

/* In irq context. */
static int mscaler_irq_notify_ch_done(struct mscaler_device *mscaler, int ch)
{
	struct isp_video_buffer *isp_buffer = NULL;
	//unsigned int y_last_addr = 0;
	//unsigned int uv_last_addr = 0;
	unsigned int fifo_empty = 0;
	unsigned int fifo_full = 0;
	unsigned int y_fifo_st = 0, uv_fifo_st = 0;
	dma_addr_t y_addr = 0;
	dma_addr_t uv_addr = 0;
	struct vb2_buffer *vb2_buf = NULL;

	spin_lock(&mscaler->lock);
	/*由于mscaler stop stream时, ISP 可能还会产生中断,这个时候不用处理*/
	if (mscaler->state[ch] == 0) {
		spin_unlock(&mscaler->lock);
		return 0;
	}

	isp_buffer = list_first_entry_or_null(&mscaler->dma_queued_list[ch], struct isp_video_buffer, list_entry);
	if (!isp_buffer) {
		dev_err(mscaler->dev, "[warning] no isp_buffer found in dma_queued_list when interrupt happend!\n");
		spin_unlock(&mscaler->lock);
		return 0;
	}

	list_del(&isp_buffer->list_entry);

	//  y_last_addr = mscaler_reg_readl(mscaler, CHx_DMAOUT_Y_LAST_ADDR(ch));

	//  uv_last_addr = mscaler_reg_readl(mscaler, CHx_DMAOUT_UV_LAST_ADDR(ch));

	/*TODO Sequence.*/
	//if(isp_buffer->vb2.vb2_buf.state == VB2_BUF_STATE_ACTIVE)
	isp_buffer->vb2.vb2_buf.timestamp = ktime_get_ns();
	isp_buffer->vb2.sequence = mscaler->framenum[ch]++;

#ifdef CONFIG_MSCA_BDEV
	/*如果后端设备存在，则调用ms_bdev_qbuf,交给后端处理.*/
	if (mscaler->bdev[ch].activated) {
		ms_bdev_qbuf(&mscaler->bdev[ch], isp_buffer);
	} else {
		vb2_buffer_done(&isp_buffer->vb2.vb2_buf, VB2_BUF_STATE_DONE);
	}
#else
	vb2_buffer_done(&isp_buffer->vb2.vb2_buf, VB2_BUF_STATE_DONE);
#endif

	y_fifo_st = mscaler_reg_readl(mscaler, CHx_Y_ADDR_FIFO_STA(ch));
	uv_fifo_st = mscaler_reg_readl(mscaler, CHx_UV_ADDR_FIFO_STA(ch));

	if ((y_fifo_st & (1 << 0)) && (uv_fifo_st & (1 << 0))) {
		fifo_empty = 1;
	} else if ((y_fifo_st & (1 << 4)) || (uv_fifo_st & (1 << 4))) {
		fifo_full = 1;
	}

	/* 如果stream on了，就写入硬件.*/
	if (mscaler->state[ch] == 1 && !fifo_full) {
		isp_buffer = list_first_entry_or_null(&mscaler->dma_pending_list[ch], struct isp_video_buffer, list_entry);
		if (isp_buffer) {
			vb2_buf = &isp_buffer->vb2.vb2_buf;
			/*Y*/
			y_addr = ingenic_vb2_dma_contig_plane_dma_addr(vb2_buf, 0);
			mscaler_reg_writel(mscaler, CHx_DMAOUT_Y_ADDR(ch), y_addr);

			uv_addr = y_addr + isp_buffer->uv_offset;
			/*UV*/
			if (vb2_buf->num_planes == 2) {
				uv_addr = ingenic_vb2_dma_contig_plane_dma_addr(vb2_buf, 1);
			}
			mscaler_reg_writel(mscaler, CHx_DMAOUT_UV_ADDR(ch), uv_addr);
			list_del(&isp_buffer->list_entry);
			list_add_tail(&isp_buffer->list_entry, &mscaler->dma_queued_list[ch]);
		}
	}

	spin_unlock(&mscaler->lock);

	return 0;
}

static int mscaler_interrupt_service_routine(struct v4l2_subdev *sd,
        u32 status, bool *handled)
{
	struct mscaler_device *mscaler = v4l2_get_subdevdata(sd);
	int ret = 0;

	/*只管心 mscaler的三个ch的done中断和crop出错的中断.*/
	if (status & (1 << MSCA_CH0_FRM_DONE_INT)) {
		ret = mscaler_irq_notify_ch_done(mscaler, 0);
	}
	if (status & (1 << MSCA_CH1_FRM_DONE_INT)) {
		ret = mscaler_irq_notify_ch_done(mscaler, 1);
	}

	if (status & (1 << MSCA_CH2_FRM_DONE_INT)) {
		ret = mscaler_irq_notify_ch_done(mscaler, 2);
	}

	/*TODO: add handler.*/

	return ret;
}

static int mscaler_subdev_init(struct v4l2_subdev *sd, u32 val)
{
	return 0;
}

static int mscaler_subdev_reset(struct v4l2_subdev *sd, u32 val)
{
	return 0;
}

static const struct v4l2_subdev_core_ops mscaler_subdev_core_ops = {
	.init   = mscaler_subdev_init,
	.reset = mscaler_subdev_reset,
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.interrupt_service_routine = mscaler_interrupt_service_routine,
};

static unsigned int ch_to_pad[] = {
	MSCALER_PAD_SOURCE_CH0,
	MSCALER_PAD_SOURCE_CH1,
	MSCALER_PAD_SOURCE_CH2
};

static int pad_to_ch(int pad)
{
	switch (pad) {
		case MSCALER_PAD_SOURCE_CH0:
			return 0;
		case MSCALER_PAD_SOURCE_CH1:
			return 1;
		case MSCALER_PAD_SOURCE_CH2:
			return 2;
		default:
			return -EINVAL;
	}
}

static int mscaler_enum_mbus_code(struct v4l2_subdev *sd,
                                  struct v4l2_subdev_state *state,
                                  struct v4l2_subdev_mbus_code_enum *code)
{
	//printk("------%s, %d, sd->name: %s\n", __func__, __LINE__, sd->name);

	if (code->index >= ARRAY_SIZE(mscaler_output_formats)) {
		dev_err(sd->dev, "too many mbus formats!\n");
		return -EINVAL;
	}

	code->code = mscaler_output_formats[code->index].mbus_code;

	return 0;
}
static int mscaler_enum_frame_size(struct v4l2_subdev *sd,
                                   struct v4l2_subdev_state *state,
                                   struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;
	const struct isp_video_format *fmt = NULL;
	struct mscaler_device *mscaler = v4l2_get_subdevdata(sd);
	struct ispcam_device    *ispcam = mscaler->ispcam;
	struct isp_async_device *isd    = ispcam->isd[0];
	struct v4l2_subdev_format isd_subdev_fmt;
	int ret = 0;

	/*step_width*/
	if (index > 0) {
		return -EINVAL;
	}

	fmt = mscaler_find_format(&fse->code, NULL, -1);
	if (fmt == NULL) {
		return -EINVAL;
	}

	ret = v4l2_subdev_call(isd->sd, pad, get_fmt, NULL, &isd_subdev_fmt);
	if (ret) {
		return ret;
	}

	fse->min_width = 100;
	fse->min_height = 100;
	/*TODO init_user_fmt*/
	fse->max_width = isd_subdev_fmt.format.width;
	fse->max_height = isd_subdev_fmt.format.height;
	return 0;
}

static int mscaler_get_fmt(struct v4l2_subdev *sd,
                           struct v4l2_subdev_state *state,
                           struct v4l2_subdev_format *format)
{
	struct mscaler_device *mscaler = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *framefmt = NULL;
	//printk("------%s, %d, sd->name: %s\n", __func__, __LINE__, sd->name);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, state, format->pad);
	} else {
		framefmt = &mscaler->formats[format->pad].format;
	}

	format->format = *framefmt;

	return 0;
}

static int mscaler_set_fmt(struct v4l2_subdev *sd,
                           struct v4l2_subdev_state *state,
                           struct v4l2_subdev_format *format)
{
	struct mscaler_device *mscaler = v4l2_get_subdevdata(sd);
	struct isp_video_device *ispvideo = v4l2_get_subdev_hostdata(sd);
	struct media_pad *remote = NULL;
	struct v4l2_subdev *remote_sd = NULL;
	struct v4l2_subdev_format remote_subdev_fmt = {0};
	int ch = 0;
	int i = 0, ret = 0;

	for (i = 0; i < MSCALER_MAX_CH; i++) {
		if (ispvideo == &mscaler->ispvideo[i]) {
			ch = i;
			break;
		}
	}

	//printk("%s, %d, width: %d, height: %d, mbus_code: %x\n", __func__, __LINE__, format->format.width, format->format.height, format->format.code);

	mscaler->formats[format->pad] = *format;

	remote = media_pad_remote_pad_unique(&mscaler->pads[MSCALER_PAD_SINK]);
	remote_sd = media_entity_to_v4l2_subdev(remote->entity);

	memcpy(&remote_subdev_fmt, format, sizeof(struct v4l2_subdev_format));
	remote_subdev_fmt.pad = remote->index;
	remote_subdev_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(remote_sd, pad, set_fmt, NULL, &remote_subdev_fmt);
	if (ret < 0) {
		dev_dbg(mscaler->dev, "Failed to set_fmt from remote pad\n");
	}

	memset(&remote_subdev_fmt, 0, sizeof(struct v4l2_subdev_format));
	remote_subdev_fmt.pad = remote->index;
	remote_subdev_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(remote_sd, pad, get_fmt, NULL, &remote_subdev_fmt);
	if (ret < 0) {
		dev_err(mscaler->dev, "Failed to get_fmt from remote pad\n");
		return -EINVAL;
	}

	memcpy(&mscaler->formats[MSCALER_PAD_SINK], &remote_subdev_fmt, sizeof(struct v4l2_subdev_format));

	/*def crop info.*/
	mscaler->attr[ch].fcrop_en = 0;
	mscaler->attr[ch].fcrop_x = 0;
	mscaler->attr[ch].fcrop_y = 0;
	mscaler->attr[ch].fcrop_width = mscaler->formats[MSCALER_PAD_SINK].format.width;
	mscaler->attr[ch].fcrop_height = mscaler->formats[MSCALER_PAD_SINK].format.height;
	mscaler->attr[ch].scaler_en = 0;
	mscaler->attr[ch].scaler_width = mscaler->formats[format->pad].format.width;
	mscaler->attr[ch].scaler_height = mscaler->formats[format->pad].format.height;
	mscaler->attr[ch].crop_en = 0;
	mscaler->attr[ch].crop_x = 0;
	mscaler->attr[ch].crop_y = 0;
	mscaler->attr[ch].crop_width = mscaler->formats[format->pad].format.width;
	mscaler->attr[ch].crop_height = mscaler->formats[format->pad].format.height;

	return 0;
}

static int mscaler_get_selection(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_selection *sel)
{
	struct mscaler_device *mscaler = v4l2_get_subdevdata(sd);
	struct v4l2_subdev_format *output_fmt = NULL;
	int ch = pad_to_ch(sel->pad);

	if (ch < 0) {
		return -EINVAL;
	}

	output_fmt = &mscaler->formats[sel->pad];
	if (sel->target == V4L2_SEL_TGT_CROP || sel->target == V4L2_SEL_TGT_CROP_DEFAULT) {
		sel->r.left = mscaler->attr[ch].crop_x;
		sel->r.top = mscaler->attr[ch].crop_y;
		sel->r.width = mscaler->attr[ch].crop_width;
		sel->r.height = mscaler->attr[ch].crop_height;
	} else if (sel->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = output_fmt->format.width;
		sel->r.height = output_fmt->format.height;
	} else {
		dev_warn(mscaler->dev, "unsupported selection target!\n");
		return -EINVAL;
	}

	return 0;
}

static int mscaler_set_selection(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_selection *sel)
{
	struct mscaler_device *mscaler = v4l2_get_subdevdata(sd);
	int ch = pad_to_ch(sel->pad);

	if (ch < 0) {
		return -EINVAL;
	}

	if (sel->flags == V4L2_SEL_FLAG_GE) {
		dev_warn(mscaler->dev, "unsupported selection flags!\n");
		return -EINVAL;
	}

	if (sel->target == V4L2_SEL_TGT_CROP) {
		mscaler->attr[ch].crop_en = 1;
		mscaler->attr[ch].crop_x = sel->r.left;
		mscaler->attr[ch].crop_y = sel->r.top;
		mscaler->attr[ch].crop_width = sel->r.width;
		mscaler->attr[ch].crop_height = sel->r.height;
	} else {
		dev_warn(mscaler->dev, "unsupported selection target!\n");
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_subdev_pad_ops mscaler_subdev_pad_ops = {
	.enum_mbus_code         = mscaler_enum_mbus_code,
	.enum_frame_size        = mscaler_enum_frame_size,
	.get_fmt                = mscaler_get_fmt,
	.set_fmt                = mscaler_set_fmt,
	.get_selection          = mscaler_get_selection,
	.set_selection          = mscaler_set_selection,
};

static int mscaler_stream_enable(struct mscaler_device *mscaler, int ch)
{
	struct v4l2_subdev_format *input_fmt = &mscaler->formats[MSCALER_PAD_SINK];
	struct v4l2_subdev_format *output_fmt = &mscaler->formats[ch_to_pad[ch]];
	unsigned long flags = 0;
	int ret = 0;
	struct isp_video_buffer *isp_buffer = NULL;
	struct isp_video_buffer *tmp = NULL;
	dma_addr_t y_addr = 0;
	dma_addr_t uv_addr = 0;
	unsigned int fifo_empty = 0;
	unsigned int fifo_full = 0;
	unsigned int y_fifo_st = 0, uv_fifo_st = 0;
	struct vb2_buffer *vb2_buf = NULL;
	tisp_channel_attr_t attr;

	dev_dbg(mscaler->dev, "-----input_fmt->format.width: %d, input_fmt->format.height: %d\n", input_fmt->format.width, input_fmt->format.height);
	dev_dbg(mscaler->dev, "-----output_fmt->format.width: %d, output_fmt->format.height: %d\n", output_fmt->format.width, output_fmt->format.height);

	spin_lock_irqsave(&mscaler->lock, flags);

	memset(&attr, 0, sizeof(tisp_channel_attr_t));

	/*TODO fcrop*/
	/*TODO output_fmt*/
	switch (output_fmt->format.code) {
		case MEDIA_BUS_FMT_YUYV8_2X8:
			mscaler->attr[ch].output_format = CHx_OUT_FMT_NV12;
			break;
		case MEDIA_BUS_FMT_YVYU8_2X8:
			mscaler->attr[ch].output_format = CHx_OUT_FMT_NV21;
			break;
	}

	if (mscaler->index == 0) {
		tisp_channel_main_attr_set(ch, &mscaler->attr[ch]);
	} else if (mscaler->index == 1) {
		tisp_channel_sec_attr_set(ch, &mscaler->attr[ch]);
	}
	mscaler->state[ch] = 1;
	mscaler->framenum[ch] = 0;

	/*因为ISP复位的执行时间时在stream_on的时候，这个时候上层qbuf已经调用了，
	 * 如果部将这些buffer重新加入硬件队列，则会出现问题.*/
	list_for_each_entry_safe(isp_buffer, tmp, &mscaler->dma_pending_list[ch], list_entry) {
		y_fifo_st = mscaler_reg_readl(mscaler, CHx_Y_ADDR_FIFO_STA(ch));
		uv_fifo_st = mscaler_reg_readl(mscaler, CHx_UV_ADDR_FIFO_STA(ch));
		if ((y_fifo_st & (1 << 0)) && (uv_fifo_st & (1 << 0))) {
			fifo_empty = 1;
		} else if ((y_fifo_st & (1 << 4)) || (uv_fifo_st & (1 << 4))) {
			fifo_full = 1;
		}
		if (fifo_full) {
			break;
		}

		vb2_buf = &isp_buffer->vb2.vb2_buf;
		isp_buffer->vb2.field = input_fmt->format.field;
		/*Y*/
		y_addr = ingenic_vb2_dma_contig_plane_dma_addr(vb2_buf, 0);
		mscaler_reg_writel(mscaler, CHx_DMAOUT_Y_ADDR(ch), y_addr);

		uv_addr = y_addr + isp_buffer->uv_offset;
		if (vb2_buf->num_planes == 2) {
			uv_addr = ingenic_vb2_dma_contig_plane_dma_addr(vb2_buf, 1);
		}
		mscaler_reg_writel(mscaler, CHx_DMAOUT_UV_ADDR(ch), uv_addr);
		list_del(&isp_buffer->list_entry);
		list_add_tail(&isp_buffer->list_entry, &mscaler->dma_queued_list[ch]);
	}
	if (mscaler->index == 0) {
		tisp_channel_main_start(ch);
	} else if (mscaler->index == 1) {
		tisp_channel_sec_start(ch);
	}

	spin_unlock_irqrestore(&mscaler->lock, flags);

#ifdef CONFIG_MSCA_BDEV
	if (mscaler->bdev[ch].activated == 1) {
		ret = ms_bdev_init(&mscaler->bdev[ch]);
		if (ret < 0) {
			return -EINVAL;
		}
	}
#endif

	return ret;
}

static int mscaler_stream_disable(struct mscaler_device *mscaler, int ch)
{
	unsigned long flags = 0;
	int ret = 0;
	unsigned int timeout = 0xffffff;
	struct isp_video_buffer *isp_buffer = NULL;
	struct isp_video_buffer *tmp = NULL;
	unsigned int val = 0;

	spin_lock_irqsave(&mscaler->lock, flags);
	if (mscaler->index == 0) {
		tisp_channel_main_stop(ch);
	} else {
		tisp_channel_sec_stop(ch);
	}

	spin_unlock_irqrestore(&mscaler->lock, flags);

	/* polling status to make sure channel stopped.*/
	do {
		val = mscaler_reg_readl(mscaler, MSCA_WORK_STAT);

	} while (!!((val & (1 << ch)) << 0) && --timeout);

	if (!timeout) {
		dev_err(mscaler->dev, "[Warning] mscaler disable timeout!\n");
	}

	spin_lock_irqsave(&mscaler->lock, flags);

	/* clear fifo*/
	mscaler_reg_writel(mscaler, CHx_DMAOUT_Y_ADDR_CLR(ch), 1);
	mscaler_reg_writel(mscaler, CHx_DMAOUT_UV_ADDR_CLR(ch), 1);
	mscaler_reg_writel(mscaler, CHx_DMAOUT_Y_LAST_ADDR_CLR(ch), 1);
	mscaler_reg_writel(mscaler, CHx_DMAOUT_UV_LAST_ADDR_CLR(ch), 1);

	list_for_each_entry_safe(isp_buffer, tmp, &mscaler->dma_queued_list[ch], list_entry) {
		struct vb2_buffer *vb2_buf = &isp_buffer->vb2.vb2_buf;
		vb2_buffer_done(vb2_buf, VB2_BUF_STATE_QUEUED);
		list_del(&isp_buffer->list_entry);
	}
	list_for_each_entry_safe(isp_buffer, tmp, &mscaler->dma_pending_list[ch], list_entry) {
		struct vb2_buffer *vb2_buf = &isp_buffer->vb2.vb2_buf;
		vb2_buffer_done(vb2_buf, VB2_BUF_STATE_QUEUED);
		list_del(&isp_buffer->list_entry);
	}

	mscaler->state[ch] = 0;
	spin_unlock_irqrestore(&mscaler->lock, flags);

#ifdef CONFIG_MSCA_BDEV
	if (mscaler->bdev[ch].activated) {
		ms_bdev_deinit(&mscaler->bdev[ch]);
		mscaler->bdev[ch].activated = 0;
	}
#endif

	return ret;
}

static int mscaler_subdev_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mscaler_device *mscaler = v4l2_get_subdevdata(sd);
	struct isp_video_device *ispvideo = v4l2_get_subdev_hostdata(sd);
	int ch = 0;
	int i = 0, ret = 0;

	for (i = 0; i < MSCALER_MAX_CH; i++) {
		if (ispvideo == &mscaler->ispvideo[i]) {
			ch = i;
			break;
		}
	}

	//  printk("%s,%d, sd->video_device->name:%s, ispvideo: %p, ch = %d, enable = %d\n", __func__, __LINE__, sd->devnode->name, ispvideo, ch, enable);
	/*获取当前channel.*/
	if (enable) {
		ret = mscaler_stream_enable(mscaler, ch);
		//dump_mscaler_regs(mscaler);
	} else {
		ret = mscaler_stream_disable(mscaler, ch);
	}

	if (ret < 0) {
		dev_err(mscaler->dev, "enable stream error!\n");
		return -EINVAL;
	}

	return 0;
}

#if 0
static int mscaler_subdev_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *cc)
{
	struct mscaler_device *mscaler = v4l2_get_subdevdata(sd);
	struct isp_video_device *ispvideo = v4l2_get_subdev_hostdata(sd);
	struct v4l2_subdev_format *input_fmt = NULL;
	int ch = 0;
	int i = 0, ret = 0;

	for (i = 0; i < MSCALER_MAX_CH; i++) {
		if (ispvideo == &mscaler->ispvideo[i]) {
			ch = i;
			break;
		}
	}

	input_fmt = &mscaler->formats[MSCALER_PAD_SINK];

	cc->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cc->bounds.left = 0;
	cc->bounds.top = 0;
	cc->bounds.width = input_fmt->format.width;
	cc->bounds.height = input_fmt->format.height;
	cc->defrect.left = 0;
	cc->defrect.top = 0;
	cc->defrect.width = input_fmt->format.width;
	cc->defrect.height = input_fmt->format.height;
	cc->pixelaspect.numerator = input_fmt->format.height;
	cc->pixelaspect.denominator = input_fmt->format.width;

	return ret;
}

static int mscaler_subdev_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
{
	struct mscaler_device *mscaler = v4l2_get_subdevdata(sd);
	struct isp_video_device *ispvideo = v4l2_get_subdev_hostdata(sd);
	int ch = 0;
	int i = 0, ret = 0;

	for (i = 0; i < MSCALER_MAX_CH; i++) {
		if (ispvideo == &mscaler->ispvideo[i]) {
			ch = i;
			break;
		}
	}

	crop->c.top = mscaler->attr[ch].fcrop_y;
	crop->c.left = mscaler->attr[ch].fcrop_x;
	crop->c.width = mscaler->attr[ch].fcrop_width;
	crop->c.height = mscaler->attr[ch].fcrop_height;

	return ret;
}

static int mscaler_subdev_s_crop(struct v4l2_subdev *sd, const struct v4l2_crop *crop)
{
	struct mscaler_device *mscaler = v4l2_get_subdevdata(sd);
	struct isp_video_device *ispvideo = v4l2_get_subdev_hostdata(sd);
	int ch = 0;
	int i = 0, ret = 0;

	for (i = 0; i < MSCALER_MAX_CH; i++) {
		if (ispvideo == &mscaler->ispvideo[i]) {
			ch = i;
			break;
		}
	}

	mscaler->attr[ch].fcrop_en = 1;
	mscaler->attr[ch].fcrop_y = crop->c.top;
	mscaler->attr[ch].fcrop_x = crop->c.left;
	mscaler->attr[ch].fcrop_width = crop->c.width;
	mscaler->attr[ch].fcrop_height = crop->c.height;

	return ret;
}
#endif

static const struct v4l2_subdev_video_ops mscaler_subdev_video_ops = {
	.s_stream = mscaler_subdev_s_stream,
	//  .cropcap = mscaler_subdev_cropcap,
	//  .g_crop = mscaler_subdev_g_crop,
	//  .s_crop = mscaler_subdev_s_crop,
};

static const struct v4l2_subdev_ops mscaler_subdev_ops = {
	.core = &mscaler_subdev_core_ops,
	.pad = &mscaler_subdev_pad_ops,
	.video = &mscaler_subdev_video_ops,
};

static int mscaler_video_qbuf(struct isp_video_device *ispvideo, struct isp_video_buffer *isp_buffer)
{
	struct mscaler_device *mscaler = ispvideo->ispcam->mscaler;
	struct v4l2_subdev_format *input_fmt = &mscaler->formats[MSCALER_PAD_SINK];
	struct vb2_buffer *vb2_buf = NULL;
	int ch = 0;
	unsigned long flags = 0;
	unsigned int fifo_empty = 0;
	unsigned int fifo_full = 0;
	unsigned int y_fifo_st = 0, uv_fifo_st = 0;
	dma_addr_t y_addr = 0;
	dma_addr_t uv_addr = 0;

	int i = 0;
	for (i = 0; i < MSCALER_MAX_CH; i++) {
		if (ispvideo == &mscaler->ispvideo[i]) {
			ch = i;
			break;
		}
	}

	spin_lock_irqsave(&mscaler->lock, flags);

	list_add_tail(&isp_buffer->list_entry, &mscaler->dma_pending_list[ch]);

	if (mscaler->iobase) {
		y_fifo_st = mscaler_reg_readl(mscaler, CHx_Y_ADDR_FIFO_STA(ch));
		uv_fifo_st = mscaler_reg_readl(mscaler, CHx_UV_ADDR_FIFO_STA(ch));

		if ((y_fifo_st & (1 << 0)) && (uv_fifo_st & (1 << 0))) {
			fifo_empty = 1;
		} else if ((y_fifo_st & (1 << 4)) || (uv_fifo_st & (1 << 4))) {
			fifo_full = 1;
		}

		if (mscaler->state[ch] == 1 && !fifo_full) {
			isp_buffer = list_first_entry_or_null(&mscaler->dma_pending_list[ch], struct isp_video_buffer, list_entry);
			if (isp_buffer) {
				vb2_buf = &isp_buffer->vb2.vb2_buf;
				isp_buffer->vb2.field = input_fmt->format.field;
				/*Y*/
				y_addr = ingenic_vb2_dma_contig_plane_dma_addr(vb2_buf, 0);
				mscaler_reg_writel(mscaler, CHx_DMAOUT_Y_ADDR(ch), y_addr);

				uv_addr = y_addr + isp_buffer->uv_offset;
				/*UV*/
				if (vb2_buf->num_planes == 2) {
					uv_addr = ingenic_vb2_dma_contig_plane_dma_addr(vb2_buf, 1);
				}
				mscaler_reg_writel(mscaler, CHx_DMAOUT_UV_ADDR(ch), uv_addr);
				list_del(&isp_buffer->list_entry);
				list_add_tail(&isp_buffer->list_entry, &mscaler->dma_queued_list[ch]);
			}
		}
	}

	spin_unlock_irqrestore(&mscaler->lock, flags);

	//printk("-----%s,%d queued buffer y_addr: 0x%08x, uv: 0x%08x on ch: %d\n", __func__, __LINE__, y_addr, uv_addr, ch);

	return 0;
}

static const struct isp_video_ops mscaler_video_ops = {
	.find_format    = mscaler_find_format,
	.qbuf       = mscaler_video_qbuf,
};

static ssize_t
dump_mscaler(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mscaler_device *mscaler = dev_get_drvdata(dev);
	char *p = buf;
	int i = 0;

	p += sprintf(p, "MSCA_CH_EN     :0x%08x\n", mscaler_reg_readl(mscaler, MSCA_CH_EN));
	p += sprintf(p, "MSCA_INT_STA   :0x%08x\n", mscaler_reg_readl(mscaler, MSCA_INT_STA));
	p += sprintf(p, "MSCA_INT_MSK   :0x%08x\n", mscaler_reg_readl(mscaler, MSCA_INT_MSK));
	p += sprintf(p, "MSCA_MASK_EN   :0x%08x\n", mscaler_reg_readl(mscaler, MSCA_MASK_EN));
	p += sprintf(p, "MSCA_DMAOUT_ARB:0x%08x\n", mscaler_reg_readl(mscaler, MSCA_DMAOUT_ARB));
	p += sprintf(p, "MSCA_CLK_GATE_EN:0x%08x\n", mscaler_reg_readl(mscaler, MSCA_CLK_GATE_EN));
	p += sprintf(p, "MSCA_CLK_DIS   :0x%08x\n", mscaler_reg_readl(mscaler, MSCA_CLK_DIS));
	p += sprintf(p, "MSCA_SRC_IN    :0x%08x\n", mscaler_reg_readl(mscaler, MSCA_SRC_IN));
	p += sprintf(p, "MSCA_SRC_SIZE  :0x%08x\n", mscaler_reg_readl(mscaler, MSCA_SRC_SIZE));
	p += sprintf(p, "MSCA_GLO_RSZ_COEF_WR:0x%08x\n", mscaler_reg_readl(mscaler, MSCA_GLO_RSZ_COEF_WR));
	p += sprintf(p, "MSCA_SYS_PRO_CLK_EN :0x%08x\n", mscaler_reg_readl(mscaler, MSCA_SYS_PRO_CLK_EN));
	p += sprintf(p, "MSCA_DS0_CLK_NUM    :0x%08x\n", mscaler_reg_readl(mscaler, MSCA_DS0_CLK_NUM));
	p += sprintf(p, "MSCA_DS1_CLK_NUM    :0x%08x\n", mscaler_reg_readl(mscaler, MSCA_DS1_CLK_NUM));
	p += sprintf(p, "MSCA_DS2_CLK_NUM    :0x%08x\n", mscaler_reg_readl(mscaler, MSCA_DS2_CLK_NUM));

	for (i = 0; i < MSCALER_MAX_CH; i++) {

		p += sprintf(p, "CHx_RSZ_OSIZE(%d)	:0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_RSZ_OSIZE(i)));
		p += sprintf(p, "CHx_RSZ_STEP(%d)       :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_RSZ_STEP(i)));
		p += sprintf(p, "CHx_CROP_OPOS(%d)      :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_CROP_OPOS(i)));
		p += sprintf(p, "CHx_CROP_OSIZE(%d)     :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_CROP_OSIZE(i)));
		p += sprintf(p, "CHx_FRA_CTRL_LOOP(%d)  :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_FRA_CTRL_LOOP(i)));
		p += sprintf(p, "CHx_FRA_CTRL_MASK(%d)  :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_FRA_CTRL_MASK(i)));
		p += sprintf(p, "CHx_MS0_POS(%d)        :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_MS0_POS(i)));
		p += sprintf(p, "CHx_MS0_SIZE(%d)       :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_MS0_SIZE(i)));
		p += sprintf(p, "CHx_MS0_VALUE(%d)      :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_MS0_VALUE(i)));
		p += sprintf(p, "CHx_MS1_POS(%d)        :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_MS1_POS(i)));
		p += sprintf(p, "CHx_MS1_SIZE(%d)       :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_MS1_SIZE(i)));
		p += sprintf(p, "CHx_MS1_VALUE(%d)      :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_MS1_VALUE(i)));
		p += sprintf(p, "CHx_MS2_POS(%d)        :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_MS2_POS(i)));
		p += sprintf(p, "CHx_MS2_SIZE(%d)       :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_MS2_SIZE(i)));
		p += sprintf(p, "CHx_MS2_VALUE(%d)      :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_MS2_VALUE(i)));
		p += sprintf(p, "CHx_MS3_POS(%d)        :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_MS3_POS(i)));
		p += sprintf(p, "CHx_MS3_SIZE(%d)       :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_MS3_SIZE(i)));
		p += sprintf(p, "CHx_MS3_VALUE(%d)      :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_MS3_VALUE(i)));
		p += sprintf(p, "CHx_OUT_FMT(%d)        :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_OUT_FMT(i)));
		p += sprintf(p, "CHx_DMAOUT_Y_ADDR(%d)  :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_DMAOUT_Y_ADDR(i)));
		p += sprintf(p, "CHx_Y_ADDR_FIFO_STA(%d):0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_Y_ADDR_FIFO_STA(i)));
		//  p += sprintf(p, "CHx_DMAOUT_Y_LAST_ADDR(%d)              :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_DMAOUT_Y_LAST_ADDR(i)));
		//  p += sprintf(p, "CHx_DMAOUT_Y_LAST_STATS_NUM(%d)         :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_DMAOUT_Y_LAST_STATS_NUM(i)));
		p += sprintf(p, "CHx_Y_LAST_ADDR_FIFO_STA(%d):0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_Y_LAST_ADDR_FIFO_STA(i)));
		p += sprintf(p, "CHx_DMAOUT_Y_STRI(%d)       :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_DMAOUT_Y_STRI(i)));
		p += sprintf(p, "CHx_DMAOUT_UV_ADDR(%d)      :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_DMAOUT_UV_ADDR(i)));
		p += sprintf(p, "CHx_UV_ADDR_FIFO_STA(%d)    :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_UV_ADDR_FIFO_STA(i)));
		//  p += sprintf(p, "CHx_DMAOUT_UV_LAST_ADDR(%d) :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_DMAOUT_UV_LAST_ADDR(i)));
		//  p += sprintf(p, "CHx_DMAOUT_UV_LAST_STATS_NUM(%d)        :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_DMAOUT_UV_LAST_STATS_NUM(i)));
		p += sprintf(p, "CHx_UV_LAST_ADDR_FIFO_STA(%d):0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_UV_LAST_ADDR_FIFO_STA(i)));
		p += sprintf(p, "CHx_DMAOUT_UV_STRI(%d)       :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_DMAOUT_UV_STRI(i)));
		p += sprintf(p, "CHx_DMAOUT_Y_ADDR_CLR(%d)    :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_DMAOUT_Y_ADDR_CLR(i)));
		p += sprintf(p, "CHx_DMAOUT_UV_ADDR_CLR(%d)   :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_DMAOUT_UV_ADDR_CLR(i)));
		p += sprintf(p, "CHx_DMAOUT_Y_LAST_ADDR_CLR(%d):0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_DMAOUT_Y_LAST_ADDR_CLR(i)));
		p += sprintf(p, "CHx_DMAOUT_UV_LAST_ADDR_CLR(%d):0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_DMAOUT_UV_LAST_ADDR_CLR(i)));
		p += sprintf(p, "CHx_DMAOUT_Y_ADDR_SEL(%d)    :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_DMAOUT_Y_ADDR_SEL(i)));
		p += sprintf(p, "CHx_DMAOUT_UV_ADDR_SEL(%d)   :0x%08x\n", i, mscaler_reg_readl(mscaler, CHx_DMAOUT_UV_ADDR_SEL(i)));
	}

	return p - buf;
}

#ifdef CONFIG_MSCA_DUMP
static DEVICE_ATTR(dump_mscaler, S_IRUGO | S_IWUSR, dump_mscaler, NULL);

static struct attribute *mscaler_debug_attrs[] = {
	&dev_attr_dump_mscaler.attr,
	NULL,
};

static struct attribute_group mscaler_debug_attr_group = {
	.name   = "debug",
	.attrs  = mscaler_debug_attrs,
};
#endif

#define to_mscaler_device(kobj) dev_get_drvdata(container_of(kobj->parent->parent, struct device, kobj))

static ssize_t mask_enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct mscaler_device *mscaler = to_mscaler_device(kobj);
	int ch = kobj->name[2] - '0';
	char *p = buf;
	unsigned int tmp = 0;

	tmp = mscaler_reg_readl(mscaler, MSCA_MASK_EN);
	p += sprintf(p, "%d\n", (tmp >> ch) & 1);

	return p - buf;
}

static ssize_t mask_enable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct mscaler_device *mscaler = to_mscaler_device(kobj);
	int ch = kobj->name[2] - '0';
	unsigned int tmp = 0;
	int enable = simple_strtol(buf, NULL, 10);

	tmp = mscaler_reg_readl(mscaler, MSCA_MASK_EN);
	if (enable == 1) {
		tmp |= 1 << ch;
	} else if (enable == 0) {
		tmp &= ~(1 << ch);
	} else {
		return -EINVAL;
	}

	mscaler_reg_writel(mscaler, MSCA_MASK_EN, tmp);
	return count;
}

static struct kobj_attribute enable_attribute =
    __ATTR(enable, S_IRUGO | S_IWUSR, mask_enable_show, mask_enable_store);

static ssize_t mask_pos_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct mscaler_device *mscaler = to_mscaler_device(kobj);
	int ch = kobj->name[2] - '0';
	int mask = to_mask_attr(attr)->id;
	char *p = buf;
	unsigned int tmp = 0;

	tmp = mscaler_reg_readl(mscaler, CHx_MSx_POS(ch, mask));
	p += sprintf(p, "x:%d\ty:%d\n", tmp >> 16, tmp & 0xffff);

	return p - buf;
}

static ssize_t mask_pos_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct mscaler_device *mscaler = to_mscaler_device(kobj);
	int ch = kobj->name[2] - '0';
	int mask = to_mask_attr(attr)->id;
	char *p;
	char *s = (char *)buf;
	int pos_x, pos_y;

	p = strsep(&s, "*");
	if (!s) {
		return -EINVAL;
	}

	pos_x = simple_strtoul(p, NULL, 10);
	pos_y = simple_strtoul(s, NULL, 10);

	mscaler_reg_writel(mscaler, CHx_MSx_POS(ch, mask), (pos_x << 16) | pos_y);
	return count;
}

static ssize_t mask_size_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct mscaler_device *mscaler = to_mscaler_device(kobj);
	int ch = kobj->name[2] - '0';
	int mask = to_mask_attr(attr)->id;
	char *p = buf;
	unsigned int tmp = 0;

	tmp = mscaler_reg_readl(mscaler, CHx_MSx_SIZE(ch, mask));
	p += sprintf(p, "width:%d\theight:%d\n", tmp >> 16, tmp & 0xffff);

	return p - buf;
}

static ssize_t mask_size_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct mscaler_device *mscaler = to_mscaler_device(kobj);
	int ch = kobj->name[2] - '0';
	int mask = to_mask_attr(attr)->id;
	char *p;
	char *s = (char *)buf;
	int width, height;

	p = strsep(&s, "*");
	if (!s) {
		return -EINVAL;
	}

	width = simple_strtoul(p, NULL, 10);
	height = simple_strtoul(s, NULL, 10);

	mscaler_reg_writel(mscaler, CHx_MSx_SIZE(ch, mask), (width << 16) | height);
	return count;
}
static ssize_t mask_color_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct mscaler_device *mscaler = to_mscaler_device(kobj);
	int ch = kobj->name[2] - '0';
	int mask = to_mask_attr(attr)->id;
	char *p = buf;
	unsigned int tmp = 0;

	tmp = mscaler_reg_readl(mscaler, CHx_MSx_VALUE(ch, mask));
	p += sprintf(p, "0x%08x\n", tmp);

	return p - buf;
}

static ssize_t mask_color_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct mscaler_device *mscaler = to_mscaler_device(kobj);
	int ch = kobj->name[2] - '0';
	int mask = to_mask_attr(attr)->id;
	char *p;
	char *s = (char *)buf;
	int value = 0;

	p = strsep(&s, "x");
	if (!s) {
		return -EINVAL;
	}

	value = simple_strtoul(s, NULL, 16);

	mscaler_reg_writel(mscaler, CHx_MSx_VALUE(ch, mask), value);
	return count;
}

#define MASK_ATTR(mask, _name, _mode, _show, _store)          \
	{                                                       \
		.attr   = __ATTR(_name, _mode, _show, _store),  \
		          .id     = mask,                                                \
	}

#define MASK_ATTRIBUTE(_name, _mode, _show, _store)                                                    \
	static struct mask_kobj_attr kobj_attr_##_name##mask0 = MASK_ATTR(0, _name, _mode, _show, _store); \
	static struct mask_kobj_attr kobj_attr_##_name##mask1 = MASK_ATTR(1, _name, _mode, _show, _store); \
	static struct mask_kobj_attr kobj_attr_##_name##mask2 = MASK_ATTR(2, _name, _mode, _show, _store); \
	static struct mask_kobj_attr kobj_attr_##_name##mask3 = MASK_ATTR(3, _name, _mode, _show, _store)

MASK_ATTRIBUTE(pos, S_IRUGO | S_IWUSR, mask_pos_show, mask_pos_store);
MASK_ATTRIBUTE(size, S_IRUGO | S_IWUSR, mask_size_show, mask_size_store);
MASK_ATTRIBUTE(color, S_IRUGO | S_IWUSR, mask_color_show, mask_color_store);

#define MASK_ATTRIBUTE_GROUP(name)                                     \
	static struct attribute *mscaler_##name##_attrs[] = {               \
		&kobj_attr_pos##name.attr.attr,                     \
		&kobj_attr_size##name.attr.attr,                      \
		&kobj_attr_color##name.attr.attr,                  \
		NULL,                                                   \
	};

MASK_ATTRIBUTE_GROUP(mask0);
MASK_ATTRIBUTE_GROUP(mask1);
MASK_ATTRIBUTE_GROUP(mask2);
MASK_ATTRIBUTE_GROUP(mask3);

static struct attribute_group mscaler_mask0_group = {
	.name = "mask0",
	.attrs = mscaler_mask0_attrs,
};
static struct attribute_group mscaler_mask1_group = {
	.name = "mask1",
	.attrs = mscaler_mask1_attrs,
};
static struct attribute_group mscaler_mask2_group = {
	.name = "mask2",
	.attrs = mscaler_mask2_attrs,
};
static struct attribute_group mscaler_mask3_group = {
	.name = "mask3",
	.attrs = mscaler_mask3_attrs,
};

static const struct attribute_group *mscaler_maskx_groups[] = {
	&mscaler_mask0_group,
	&mscaler_mask1_group,
	&mscaler_mask2_group,
	&mscaler_mask3_group,
	NULL,
};

static void __maybe_unused dump_mscaler_regs(struct mscaler_device *mscaler)
{
	char *buf = kzalloc(4 * 1024, GFP_KERNEL);
	int ret = 0;

	ret = dump_mscaler(mscaler->dev, NULL, buf);

	printk("%s", buf);

	kfree(buf);
}

int mscaler_video_nr_map[2][3] = {
	{INGENIC_MSCA0_CH0_VIDEO_NR, INGENIC_MSCA0_CH1_VIDEO_NR, INGENIC_MSCA0_CH2_VIDEO_NR},
	{INGENIC_MSCA1_CH0_VIDEO_NR, INGENIC_MSCA1_CH1_VIDEO_NR, INGENIC_MSCA1_CH2_VIDEO_NR},

};

static int mscaler_comp_bind(struct device *comp, struct device *master,
                             void *master_data)
{
	struct mscaler_device *mscaler = dev_get_drvdata(comp);
	struct ispcam_device *ispcam = (struct ispcam_device *)master_data;
	struct v4l2_device *v4l2_dev = &ispcam->v4l2_dev;
	struct v4l2_subdev *sd = &mscaler->sd;
	int i = 0;
	int ret = 0;
	int nr = -1;

	//dev_info(comp, "----dev_name(comp): %s----%s, %d \n", dev_name(comp), __func__, __LINE__);

	/* link subdev to master.*/
	mscaler->ispcam = (void *)ispcam;
	ispcam->mscaler = mscaler;
	mscaler->index = ispcam->dev_nr;

	/*1. register supported subdev ctrls.*/

	/*2. init v4l2_subdev*/

	v4l2_subdev_init(sd, &mscaler_subdev_ops);

	sd->owner = THIS_MODULE;
	sd->flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	strscpy(sd->name, dev_name(comp), sizeof(sd->name));
	v4l2_set_subdevdata(sd, mscaler);

	/* init mscaler pads. */
	mscaler->pads = kzalloc(sizeof(struct media_pad) * MSCALER_NUM_PADS, GFP_KERNEL);
	if (!mscaler->pads) {
		ret = -ENOMEM;
		goto err_alloc_pads;
	}
	mscaler->pads[0].index = MSCALER_PAD_SINK;
	mscaler->pads[0].flags = MEDIA_PAD_FL_SINK;
	mscaler->pads[1].index = MSCALER_PAD_SOURCE_CH0;
	mscaler->pads[1].flags = MEDIA_PAD_FL_SOURCE;
	mscaler->pads[2].index = MSCALER_PAD_SOURCE_CH1;
	mscaler->pads[2].flags = MEDIA_PAD_FL_SOURCE;
	mscaler->pads[3].index = MSCALER_PAD_SOURCE_CH2;
	mscaler->pads[3].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, MSCALER_NUM_PADS, mscaler->pads);

	/*3. register v4l2_subdev*/
	sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_SCALER;
	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0) {
		dev_err(comp, "Failed to register v4l2_subdev for mscaler\n");
		goto err_subdev_register;
	}

	/*create mask dir*/
	mscaler->mask_kobj = kobject_create_and_add("mask", &mscaler->dev->kobj);
	if (!mscaler->mask_kobj) {
		dev_err(mscaler->dev, "device create kobject failed\n");
		ret = -EINVAL;
		goto err_mask_kobj;
	}

	for (i = 0; i < MSCALER_MAX_CH; i++) {
		struct isp_video_device *ispvideo = &mscaler->ispvideo[i];
		char name[32];
		sprintf(name, "%s-ch%d", dev_name(mscaler->dev), i);

		ispvideo->ispcam = ispcam;
		ret = isp_video_init(ispvideo, name, &mscaler_video_ops);
		if (ret < 0) {
			/*TODO*/
		}

		nr = mscaler_video_nr_map[ispcam->dev_nr][i];
		ret = isp_video_register(ispvideo, &ispcam->v4l2_dev, nr);
		if (ret < 0) {
			/*TODO*/
		}

		/*create mask/chx dir*/
		sprintf(name, "ch%d", i);
		mscaler->mask_ch_kobj[i] = kobject_create_and_add(name, mscaler->mask_kobj);
		if (!mscaler->mask_ch_kobj[i]) {
			dev_err(mscaler->dev, "device create kobject failed\n");
			ret = -EINVAL;
			goto err_mask_ch_kobj;
		}

		/*create enable attr*/
		ret = sysfs_create_file(mscaler->mask_ch_kobj[i], &enable_attribute.attr);
		if (ret) {
			dev_err(mscaler->dev, "device create sys file failed\n");
			goto err_mask_create_file;
		}

		/*create mask/chx/maskx dir*/
		ret = sysfs_create_groups(mscaler->mask_ch_kobj[i], mscaler_maskx_groups);
		if (ret) {
			dev_err(mscaler->dev, "device create sys groups failed\n");
			goto err_mask_create_groups;
		}

	}
	return 0;
err_mask_create_groups:
err_mask_create_file:
err_mask_ch_kobj:
err_mask_kobj:
err_subdev_register:
err_alloc_pads:
	return ret;
}

static void mscaler_comp_unbind(struct device *comp, struct device *master,
                                void *master_data)
{
	struct mscaler_device *mscaler = dev_get_drvdata(comp);

	dev_info(comp, "----TODO: %p----%s, %d \n", mscaler, __func__, __LINE__);

}

static const struct component_ops mscaler_comp_ops = {
	.bind = mscaler_comp_bind,
	.unbind = mscaler_comp_unbind,
};

static int ingenic_mscaler_probe(struct platform_device *pdev)
{

	struct mscaler_device *mscaler = NULL;
	struct resource *regs = NULL;
	int ret = 0;
	int i = 0;

	mscaler = kzalloc(sizeof(struct mscaler_device), GFP_KERNEL);
	if (!mscaler) {
		pr_err("Failed to alloc mscaler dev [%s]\n", pdev->name);
		return -ENOMEM;
	}

	mscaler->dev = &pdev->dev;
	platform_set_drvdata(pdev, mscaler);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "No iomem resource!\n");
		goto err_get_resource;
	}

	mscaler->iobase = devm_ioremap_resource(&pdev->dev, regs);
	if (!mscaler->iobase) {
		goto err_ioremap;
	}

	spin_lock_init(&mscaler->lock);

	for (i = 0; i < MSCALER_MAX_CH; i++) {
		INIT_LIST_HEAD(&mscaler->dma_queued_list[i]);
		INIT_LIST_HEAD(&mscaler->dma_pending_list[i]);
	}

#ifdef CONFIG_MSCA_DUMP
	ret = sysfs_create_group(&mscaler->dev->kobj, &mscaler_debug_attr_group);
	if (ret) {
		dev_err(mscaler->dev, "device create sysfs group failed\n");

		ret = -EINVAL;
		goto err_sys_group;
	}
#endif

	ret = component_add(mscaler->dev, &mscaler_comp_ops);
	if (ret < 0) {
		dev_err(mscaler->dev, "Failed to add component mscaler!\n");
	}

	return 0;
err_sys_group:
err_ioremap:
err_get_resource:
	return ret;
}

static int ingenic_mscaler_remove(struct platform_device *pdev)
{

	return 0;
}

static const struct of_device_id ingenic_mscaler_dt_match[] = {
	{ .compatible = "ingenic,x2500-mscaler" },
	{ }
};

MODULE_DEVICE_TABLE(of, ingenic_mscaler_dt_match);

static int __maybe_unused ingenic_mscaler_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int __maybe_unused ingenic_mscaler_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver ingenic_mscaler_driver = {
	.probe = ingenic_mscaler_probe,
	.remove = ingenic_mscaler_remove,
	.suspend = ingenic_mscaler_suspend,
	.resume = ingenic_mscaler_resume,
	.driver = {
		.name = "ingenic-mscaler",
		.of_match_table = ingenic_mscaler_dt_match,
	},
};

module_platform_driver(ingenic_mscaler_driver);

MODULE_ALIAS("platform:ingenic-mscaler");
MODULE_DESCRIPTION("ingenic mscaler subsystem");
MODULE_AUTHOR("qipengzhen <aric.pzqi@ingenic.com>");
MODULE_LICENSE("GPL v2");
