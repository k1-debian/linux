#include <linux/clk.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/kthread.h>
#include <asm/dma.h>
#include <asm/page.h>
#include <linux/videodev2.h>
#include <linux/of_reserved_mem.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <asm/mach-generic/spaces.h>
#include <media/v4l2-async.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig-ingenic.h>
#include <media/ingenic_video_nr.h>
#include <linux/delay.h>

#include "hal/vic_regs.h"
#include "hal/vic_sensor_info.h"
#include "hal/csi.h"

struct vic_dev;

struct vic_file_ctx {
	struct v4l2_fh fh;
	struct vic_dev *vic;
};

struct vic_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head link;
};

struct vic_dev {
	void __iomem *iobase;
	int irq;
	struct clk *isp_div_clk;
	struct clk *isp_power_clk;
	struct clk *isp_gate_clk;
	struct clk *mipi_clk0;
	struct clk *mipi_clk1;
	struct resource *res;

	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct v4l2_fwnode_endpoint endpoint;
	struct v4l2_async_subdev async_subdev;
	struct video_device video;
	struct v4l2_subdev *subdev;
	int sequence;
	int video_device_num;

	struct mutex mutex;
	struct mutex queue_mutex;
	struct vb2_queue queue;
	struct vic_file_ctx *cur_ctx;
	struct v4l2_pix_format fmt;

	struct list_head list;
	spinlock_t lock;

	struct vic_hal_info info;

	int open_cnt;
	int is_streaming;
	struct vic_buffer *t[2];
	int frame_index;
};

struct vic_format {
	unsigned int fourcc;
	unsigned int mbus_code;
	mipi_data_fmt mipi_fmt;
	dvp_data_fmt dvp_fmt;
	unsigned int depth;
	const char *name;
	enum v4l2_colorspace colorspace;
};

static struct vic_format vic_formats[] = {
	/*MONO */
	{V4L2_PIX_FMT_GREY, MEDIA_BUS_FMT_Y8_1X8, MIPI_RAW8, DVP_RAW8, 8, "Y8, GREY", V4L2_COLORSPACE_RAW},
	{V4L2_PIX_FMT_Y10, MEDIA_BUS_FMT_Y10_1X10, MIPI_RAW10, DVP_RAW10, 16, "Y10, Y10", V4L2_COLORSPACE_RAW},
	{V4L2_PIX_FMT_Y12, MEDIA_BUS_FMT_Y12_1X12, MIPI_RAW12, DVP_RAW12, 16, "Y12, Y12", V4L2_COLORSPACE_RAW},
	/*yuv422 */
	{V4L2_PIX_FMT_YUYV, MEDIA_BUS_FMT_YUYV8_2X8, MIPI_YUV422, DVP_YUV422, 16, "YUV422, YUYV", V4L2_COLORSPACE_DEFAULT},
	{V4L2_PIX_FMT_YVYU, MEDIA_BUS_FMT_YVYU8_2X8, MIPI_YUV422, DVP_YUV422, 16, "YUV422, YVYU", V4L2_COLORSPACE_DEFAULT},
	{V4L2_PIX_FMT_UYVY, MEDIA_BUS_FMT_UYVY8_2X8, MIPI_YUV422, DVP_YUV422, 16, "YUV422, UYVY", V4L2_COLORSPACE_DEFAULT},
	{V4L2_PIX_FMT_VYUY, MEDIA_BUS_FMT_VYUY8_2X8, MIPI_YUV422, DVP_YUV422, 16, "YUV422, VYUY", V4L2_COLORSPACE_DEFAULT},
	/* RAW8 */
	{V4L2_PIX_FMT_SBGGR8, MEDIA_BUS_FMT_SBGGR8_1X8, MIPI_RAW8, DVP_RAW8, 8, "RAW8, BGBG GRGR", V4L2_COLORSPACE_SRGB},
	{V4L2_PIX_FMT_SGBRG8, MEDIA_BUS_FMT_SGBRG8_1X8, MIPI_RAW8, DVP_RAW8, 8, "RAW8, GBGB RGRG", V4L2_COLORSPACE_SRGB},
	{V4L2_PIX_FMT_SGRBG8, MEDIA_BUS_FMT_SGRBG8_1X8, MIPI_RAW8, DVP_RAW8, 8, "RAW8, GRGR BGBG", V4L2_COLORSPACE_SRGB},
	{V4L2_PIX_FMT_SRGGB8, MEDIA_BUS_FMT_SRGGB8_1X8, MIPI_RAW8, DVP_RAW8, 8, "RAW8, RGRG GBGB", V4L2_COLORSPACE_SRGB},
	/* sensor RAW10 to vic RAW16 */
	{V4L2_PIX_FMT_SBGGR10, MEDIA_BUS_FMT_SBGGR10_1X10, MIPI_RAW10, DVP_RAW10, 16, "RAW10, BGBG GRGR", V4L2_COLORSPACE_SRGB},
	{V4L2_PIX_FMT_SGBRG10, MEDIA_BUS_FMT_SGBRG10_1X10, MIPI_RAW10, DVP_RAW10, 16, "RAW10, GBGB RGRG", V4L2_COLORSPACE_SRGB},
	{V4L2_PIX_FMT_SGRBG10, MEDIA_BUS_FMT_SGRBG10_1X10, MIPI_RAW10, DVP_RAW10, 16, "RAW10, GRGR BGBG", V4L2_COLORSPACE_SRGB},
	{V4L2_PIX_FMT_SRGGB10, MEDIA_BUS_FMT_SRGGB10_1X10, MIPI_RAW10, DVP_RAW10, 16, "RAW10, RGRG GBGB", V4L2_COLORSPACE_SRGB},
	/* sensor RAW12 to vic RAW16 */
	{V4L2_PIX_FMT_SBGGR12, MEDIA_BUS_FMT_SBGGR12_1X12, MIPI_RAW12, DVP_RAW12, 16, "RAW12, BGBG GRGR", V4L2_COLORSPACE_SRGB},
	{V4L2_PIX_FMT_SGBRG12, MEDIA_BUS_FMT_SGBRG12_1X12, MIPI_RAW12, DVP_RAW12, 16, "RAW12, GBGB RGRG", V4L2_COLORSPACE_SRGB},
	{V4L2_PIX_FMT_SGRBG12, MEDIA_BUS_FMT_SGRBG12_1X12, MIPI_RAW12, DVP_RAW12, 16, "RAW12, GRGR BGBG", V4L2_COLORSPACE_SRGB},
	{V4L2_PIX_FMT_SRGGB12, MEDIA_BUS_FMT_SRGGB12_1X12, MIPI_RAW12, DVP_RAW12, 16, "RAW12, RGRG GBGB", V4L2_COLORSPACE_SRGB},
};

struct vic_format *fourcc_to_vic_format(unsigned int fourcc)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(vic_formats); i++) {
		struct vic_format *fmt = &vic_formats[i];
		if (fourcc == fmt->fourcc) {
			return fmt;
		}
	}
	return NULL;
}

struct vic_format *mbus_code_to_vic_format(unsigned int code)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(vic_formats); i++) {
		struct vic_format *fmt = &vic_formats[i];
		if (code == fmt->mbus_code) {
			return fmt;
		}
	}
	return NULL;
}

static unsigned int fourcc_to_mbus_code(unsigned int fourcc)
{
	struct vic_format *fmt = fourcc_to_vic_format(fourcc);
	return fmt ? fmt->mbus_code : 0;
}

static unsigned int fourcc_to_mbus_depth(unsigned int fourcc)
{
	struct vic_format *fmt = fourcc_to_vic_format(fourcc);
	return fmt ? fmt->depth : 0;
}

static unsigned int fourcc_to_mipi_fmt(unsigned int fourcc)
{
	struct vic_format *fmt = fourcc_to_vic_format(fourcc);
	return fmt ? fmt->mipi_fmt : 0;
}

static unsigned int fourcc_to_dvp_fmt(unsigned int fourcc)
{
	struct vic_format *fmt = fourcc_to_vic_format(fourcc);
	return fmt ? fmt->dvp_fmt : 0;
}

static int vic_debug = 0;
module_param(vic_debug, int, 0644);

#define vic_dbg(x...) \
	do { \
		if (vic_debug) \
			printk(KERN_ERR x); \
	} while (0)

static int vic_querycap(struct file *file, void *fh,
                        struct v4l2_capability *cap)
{
	// struct vic_file_ctx *ctx = video_drvdata(file);

	strlcpy(cap->driver, "ingenic-camera", sizeof(cap->driver));
	cap->version = 0;
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int vic_enum_fmt_vid_cap(struct file *file, void *fh,
                                struct v4l2_fmtdesc *f)
{
	struct vic_dev *vic = video_drvdata(file);
	struct v4l2_subdev_mbus_code_enum code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	code.index = f->index;
	ret = v4l2_subdev_call(vic->subdev, pad, enum_mbus_code, NULL, &code);
	if (ret < 0) {
		dev_err(vic->dev, "subdev failed to enum_mbus_code: %d\n", ret);
		return -EINVAL;
	}

	struct vic_format *fmt = mbus_code_to_vic_format(code.code);
	if (fmt) {
		strlcpy(f->description, fmt->name, sizeof(f->description));
		f->pixelformat = fmt->fourcc;
		return 0;
	}

	dev_err(vic->dev, "subdev code not support: %x\n", code.code);

	return -ENODEV;
}

static int check_range(int x, int min, int max)
{
	if (x < min) {
		return min;
	}
	if (x > max) {
		return max;
	}
	return x;
}

static int vic_try_fmt(struct file *file, struct v4l2_format *f)
{
	struct vic_dev *vic = video_drvdata(file);
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
	};
	struct v4l2_subdev_pad_config cfg;
	struct v4l2_mbus_framefmt *mf = &format.format;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int depth, ret;

	mf->width = check_range(pix->width, 128, 4096);
	mf->height = check_range(pix->height, 128, 4096);
	mf->field = pix->field;
	mf->colorspace = pix->colorspace;
	mf->code = fourcc_to_mbus_code(pix->pixelformat);

	ret = v4l2_subdev_call(vic->subdev, pad, set_fmt, &cfg, &format);
	if (ret < 0) {
		dev_err(vic->dev, "subdev failed to try fmt: %dx%d@%x\n",
		        mf->width, mf->height, mf->code);
		return ret;
	}

	pix->width = mf->width;
	pix->height = mf->height;
	pix->field = mf->field;
	pix->colorspace = mf->colorspace;
	depth = fourcc_to_mbus_depth(pix->pixelformat);
	pix->bytesperline = pix->width * depth / 8;
	pix->sizeimage = pix->bytesperline * pix->height;

	return 0;
}

static int vic_s_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vic_dev *vic = video_drvdata(file);
	int ret;

	ret = vic_try_fmt(file, f);
	if (ret) {
		return ret;
	}

	vic->fmt = f->fmt.pix;
	vic->info.width = vic->fmt.width;
	vic->info.height = vic->fmt.height;

	if (vic->info.bus_type == SENSOR_DATA_BUS_MIPI) {
		vic->info.mipi.data_fmt = fourcc_to_mipi_fmt(vic->fmt.pixelformat);
		vic->info.is_y8 = vic->info.mipi.data_fmt == MIPI_RAW8;
	}

	if (vic->info.bus_type == SENSOR_DATA_BUS_DVP) {
		vic->info.dvp.data_fmt = fourcc_to_dvp_fmt(vic->fmt.pixelformat);
		vic->info.is_y8 = vic->info.dvp.data_fmt == DVP_RAW8;
	}

	printk(KERN_INFO "vic: s_fmt_vid_cap: %dx%d\n", vic->info.width, vic->info.height);

	return 0;
}

static int vic_g_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vic_dev *vic = video_drvdata(file);

	vic_dbg("vic: g_fmt_vid_cap\n");

	f->fmt.pix = vic->fmt;

	return 0;
}

static int vic_try_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	// struct vic_dev *vic = video_drvdata(file);

	vic_dbg("vic: g_fmt_vid_cap\n");

	return vic_try_fmt(file, f);
}

static int vic_enum_framesizes(struct file *file, void *fh, struct v4l2_frmsizeenum *fsize)
{
	struct vic_dev *vic = video_drvdata(file);

	struct v4l2_subdev_frame_size_enum fse = {
		.index = fsize->index,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	vic_dbg("vic: enum_framesizes\n");

	ret = v4l2_subdev_call(vic->subdev, pad, enum_frame_size, NULL, &fse);
	if (ret < 0) {
		dev_err(vic->dev, "subdev failed to enum_frame_size: %d\n", ret);
		return ret;
	}

	if (fse.min_width == fse.max_width &&
	    fse.min_height == fse.max_height) {
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = fse.min_width;
		fsize->discrete.height = fse.min_height;
		return 0;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = fse.min_width;
	fsize->stepwise.max_width = fse.max_width;
	fsize->stepwise.min_height = fse.min_height;
	fsize->stepwise.max_height = fse.max_height;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int vic_s_input(struct file *file, void *fh, unsigned int i)
{
	// struct vic_dev *vic = video_drvdata(file);

	vic_dbg("vic: s_input\n");

	return 0;
}

static const struct v4l2_ioctl_ops vic_video_ioctl_ops = {
	.vidioc_querycap                = vic_querycap,
	.vidioc_enum_fmt_vid_cap        = vic_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap           = vic_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap           = vic_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap         = vic_try_fmt_vid_cap,
	.vidioc_create_bufs             = vb2_ioctl_create_bufs,
	.vidioc_reqbufs                 = vb2_ioctl_reqbufs,
	.vidioc_querybuf                = vb2_ioctl_querybuf,
	.vidioc_qbuf                    = vb2_ioctl_qbuf,
	.vidioc_dqbuf                   = vb2_ioctl_dqbuf,
	.vidioc_streamon                = vb2_ioctl_streamon,
	.vidioc_streamoff               = vb2_ioctl_streamoff,
	.vidioc_enum_framesizes         = vic_enum_framesizes,
	.vidioc_s_input                 = vic_s_input,
};

static int vic_queue_setup(struct vb2_queue *vq,
                           unsigned int *nbuffers, unsigned int *nplanes,
                           unsigned int sizes[], struct device *alloc_devs[])
{
	struct vic_dev *vic = vb2_get_drv_priv(vq);

	if (*nplanes && *nplanes != 1) {
		dev_err(vic->dev, "nplanes not valid: %d\n", *nplanes);
		return -EINVAL;
	}

	*nplanes = 1;
	sizes[0] = vic->fmt.sizeimage;
	alloc_devs[0] = vic->dev;

	vic_dbg("vic: queue setup\n");

	return 0;
}

static int vic_buf_prepare(struct vb2_buffer *vb)
{
	struct vic_dev *vic = vb2_get_drv_priv(vb->vb2_queue);

	vic_dbg("vic: buf prepare: %p\n", vb);

	vb2_set_plane_payload(vb, 0, vic->fmt.sizeimage);
	if (vb2_plane_vaddr(vb, 0) &&
	    vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0)) {
		dev_err(vic->dev, "vb2 size not valid: %ld imagesize is: %d\n",
		        vb2_plane_size(vb, 0), vic->fmt.sizeimage);
		return -EINVAL;
	}

	return 0;
}

static void vic_buf_queue(struct vb2_buffer *vb)
{
	struct vic_dev *vic = vb2_get_drv_priv(vb->vb2_queue);
	struct vic_buffer *buf = (void *)vb;
	unsigned long flags;

	vic_dbg("vic: buf queue: %p\n", vb);

	spin_lock_irqsave(&vic->lock, flags);

	if (vic->is_streaming && vic->t[0] == vic->t[1]) {
		unsigned long addr = ingenic_vb2_dma_contig_plane_dma_addr(vb, 0);
		vic_dbg("buf phys: %lx\n", addr);

		int i = !vic->frame_index;
		vic_set_dma_addr(vic->info.index, addr, addr, i);
		vic->t[i] = buf;
	} else {
		list_add_tail(&buf->link, &vic->list);
	}

	spin_unlock_irqrestore(&vic->lock, flags);
}

static void vic_buf_cleanup(struct vb2_buffer *vb)
{
	// struct vic_dev *vic = vb2_get_drv_priv(vb->vb2_queue);
	// struct vic_buffer *buf = (void *)vb;

	vic_dbg("vic: buf queue clean: %p\n", vb2_plane_vaddr(vb, 0));
}

static int vic_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	unsigned long flags;
	struct vic_dev *vic = vb2_get_drv_priv(vq);

	vic_dbg("vic: start streaming\n");

	spin_lock_irqsave(&vic->lock, flags);

	vic->is_streaming = 1;
	vic->frame_index = 0;
	vic->sequence = 0;

	struct vic_buffer *buf = NULL;

	int i = 0;
	while (!list_empty(&vic->list)) {
		buf = list_first_entry(&vic->list, struct vic_buffer, link);
		list_del_init(&buf->link);

		unsigned long addr = ingenic_vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
		vic_dbg("vic: buf phys: %lx\n", addr);

		vic_set_dma_addr(vic->info.index, addr, addr, i);
		vic->t[i] = buf;
		if (++i == 2) {
			break;
		}
	}

	vic_set_bit(vic->info.index, VIC_DMA_CONFIGURE, Dma_en, 1);

	spin_unlock_irqrestore(&vic->lock, flags);

	if (i == 0) {
		printk(KERN_ERR "vic: failed to start streaming: no buf queued\n");
		return -EINVAL;
	}

	vic_hal_stream_on(&vic->info);

	v4l2_subdev_call(vic->subdev, video, s_stream, 1);

	return 0;
}

static void vic_buf_done(struct vic_dev *vic, struct vic_buffer *buf, enum vb2_buffer_state state)
{
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = vic->sequence++;
	vb2_buffer_done(&buf->vb.vb2_buf, state);
	vic_dbg("buf done: %p %d\n", &buf->vb.vb2_buf, state);
}

static void check_free_vb2_buffer(struct vic_dev *vic)
{
	struct vic_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&vic->lock, flags);

	if (vic->t[0]) {
		vic_buf_done(vic, vic->t[0], VB2_BUF_STATE_ERROR);
	}
	if (vic->t[0] != vic->t[1] && vic->t[1]) {
		vic_buf_done(vic, vic->t[1], VB2_BUF_STATE_ERROR);
	}
	vic->t[0] = NULL;
	vic->t[1] = NULL;

	while (!list_empty(&vic->list)) {
		buf = list_first_entry(&vic->list, struct vic_buffer, link);
		list_del_init(&buf->link);
		vic_buf_done(vic, buf, VB2_BUF_STATE_ERROR);
	}

	spin_unlock_irqrestore(&vic->lock, flags);
}

static void check_stop_streaming(struct vic_dev *vic)
{
	if (!vic->is_streaming) {
		return;
	}

	vic_hal_stream_off(&vic->info);

	v4l2_subdev_call(vic->subdev, video, s_stream, 0);
	vic->is_streaming = 0;
}

static void vic_stop_streaming(struct vb2_queue *vq)
{
	struct vic_dev *vic = vb2_get_drv_priv(vq);

	vic_dbg("vic: stop streaming\n");

	check_stop_streaming(vic);
	check_free_vb2_buffer(vic);
}

static irqreturn_t vic_irq_dma_handler(int irq, void *data)
{
	unsigned long flags;
	struct vic_dev *vic = data;
	volatile unsigned long state, pending, mask;

	mask = vic_read_reg(vic->info.index, VIC_INT_MASK);
	state = vic_read_reg(vic->info.index, VIC_INT_STA);
	pending = state & (~mask);
	vic_write_reg(vic->info.index, VIC_INT_CLR, pending);

	vic_dbg("vic: status = 0x%08lx\n", pending);

	// while (1) {
	//     printk(KERN_ERR "vic: %d %d\n",
	//     vic_read_reg(vic->info.index, 0x90), vic_read_reg(vic->info.index, 0xa0));
	// }

	spin_lock_irqsave(&vic->lock, flags);

	if (get_bit_field(pending, DMA_FRD)) {
		struct vic_buffer *buf = NULL, *usable_buf = NULL;
		int i = vic->frame_index;
		vic->frame_index = !i;

		/*
		 * 当传输列表只有一帧的时候，不能用这一帧
		 */
		if (vic->t[0] != vic->t[1]) {
			usable_buf = vic->t[i];
		}

		/*
		 * 1 优先从空闲列表中获取新的帧加入传输
		 * 2 如果空闲列表也没有，那么用下一帧做保底
		 */
		if (!list_empty(&vic->list)) {
			buf = list_first_entry(&vic->list, struct vic_buffer, link);
			list_del_init(&buf->link);
		}

		if (!buf) {
			buf = vic->t[!i];
		}
		vic->t[i] = buf;
		unsigned long addr = ingenic_vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
		vic_set_dma_addr(vic->info.index, addr, addr, i);

		if (usable_buf) {
			vic_buf_done(vic, usable_buf, VB2_BUF_STATE_DONE);
		}
	}

	if (get_bit_field(pending, VIC_HVRES_ERR)) {
		/* 复位vic后，下帧重头开始取（防裂屏），dma地址会从第一个地址开始往后写 */
		vic_reset(vic->info.index);
		vic->frame_index = 0;
		printk(KERN_ERR "## VIC WARN status = 0x%08lx\n", pending);
	}
	if (get_bit_field(pending, VIC_FIFO_OVF)) {

	}

	if (get_bit_field(pending, VIC_FRM_START)) {

	}

	if (get_bit_field(pending, VIC_FRD)) {

	}

	spin_unlock_irqrestore(&vic->lock, flags);

	return IRQ_HANDLED;
}

static struct vb2_ops vic_video_queue_ops = {
	.queue_setup      = vic_queue_setup,
	.buf_prepare      = vic_buf_prepare,
	.buf_queue        = vic_buf_queue,
	.buf_cleanup      = vic_buf_cleanup,
	.start_streaming  = vic_start_streaming,
	.stop_streaming   = vic_stop_streaming,
};

static void enable_vic_clk(struct vic_dev *vic)
{
	clk_prepare_enable(vic->isp_div_clk);
	clk_set_rate(vic->isp_div_clk, vic->info.isp_clk_rate);
	clk_prepare_enable(vic->isp_gate_clk);
	clk_prepare_enable(vic->isp_power_clk);
}

static void disable_vic_clk(struct vic_dev *vic)
{
	clk_disable_unprepare(vic->isp_div_clk);
	clk_disable_unprepare(vic->isp_gate_clk);
	clk_disable_unprepare(vic->isp_power_clk);
}

static int vic_video_open(struct file *file)
{
	int ret = 0;
	struct vic_dev *vic = video_drvdata(file);
	struct vic_file_ctx *ctx;

	mutex_lock(&vic->mutex);

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		dev_err(vic->dev, "failed to alloc file ctx\n");
		ret = -ENOMEM;
		goto unlock;
	}
	ctx->vic = vic;

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	v4l2_fh_add(&ctx->fh);
	file->private_data = &ctx->fh;

	if (vic->open_cnt++ == 0) {
		enable_vic_clk(vic);
	}

unlock:
	mutex_unlock(&vic->mutex);
	return ret;
}

static int vic_video_release(struct file *file)
{
	struct vic_dev *vic = video_drvdata(file);
	// struct vic_file_ctx *ctx = file->private_data;
	// struct vb2_queue *queue = &ctx->queue;

	mutex_lock(&vic->mutex);

	vic_dbg("vic: video release\n");

	if (file->private_data == vic->queue.owner) {
		check_stop_streaming(vic);
	}

	check_free_vb2_buffer(vic);

	vb2_fop_release(file);
	// 这三句话会被 vb2_fop_release 调用
	// v4l2_fh_del(&ctx->fh);
	// v4l2_fh_exit(&ctx->fh);
	// kfree(ctx);

	if (--vic->open_cnt == 0) {
		disable_vic_clk(vic);
	}

	mutex_unlock(&vic->mutex);

	return 0;
}

static struct v4l2_file_operations vic_video_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = vic_video_open,
	.release = vic_video_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

int vic_async_notifier_bound(struct v4l2_async_notifier *notifier,
                             struct v4l2_subdev *subdev,
                             struct v4l2_async_subdev *asd)
{
	struct v4l2_device *v4l2_dev = notifier->v4l2_dev;
	struct vic_dev *vic = container_of(notifier, struct vic_dev, notifier);
	struct video_device *video = &vic->video;
	struct vb2_queue *queue = &vic->queue;
	queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	queue->io_modes = VB2_MMAP | VB2_USERPTR;
	queue->drv_priv = vic;
	queue->ops = &vic_video_queue_ops;
	queue->mem_ops = &ingenic_vb2_dma_contig_memops;
	queue->buf_struct_size = sizeof(struct vic_buffer);
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	queue->lock = &vic->queue_mutex;

	int ret = vb2_queue_init(queue);
	if (ret < 0) {
		dev_err(vic->dev, "failed to init queue: %d\n", ret);
		return ret;
	}

	strcpy(video->name, v4l2_dev->name);
	video->fops = &vic_video_fops;
	video->ioctl_ops = &vic_video_ioctl_ops;
	video->release = video_device_release_empty;
	video->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	video->v4l2_dev = v4l2_dev;
	video->ctrl_handler = NULL;
	video_set_drvdata(video, vic);
	video->queue = queue;

	ret = video_register_device(video, VFL_TYPE_VIDEO, vic->video_device_num);
	if (ret < 0) {
		dev_err(vic->dev, "failed to register video device: %d\n", ret);
		vb2_queue_release(queue);
		return ret;
	}

	vic->subdev = subdev;

	dev_info(vic->dev, "register video device %s @ /dev/video%d ok\n", video->name, video->num);

	return 0;
}

int vic_async_notifier_complete(struct v4l2_async_notifier *notifier)
{

	return 0;
}

void vic_async_notifier_unbind(struct v4l2_async_notifier *notifier,
                               struct v4l2_subdev *subdev,
                               struct v4l2_async_subdev *asd)
{
	struct vic_dev *vic = container_of(notifier, struct vic_dev, notifier);

	video_unregister_device(&vic->video);
	vb2_queue_release(&vic->queue);
}

struct v4l2_async_notifier_operations vic_async_ops = {
	.bound = vic_async_notifier_bound,
	.complete = vic_async_notifier_complete,
	.unbind = vic_async_notifier_unbind,
};

static int vic_get_remote_endpoint(struct vic_dev *vic)
{
	struct device_node *node = NULL;
	struct v4l2_async_subdev *asd = NULL;
	int ret = 0;

	v4l2_async_notifier_init(&vic->notifier);
	node = of_graph_get_next_endpoint(vic->dev->of_node, node);
	if (!node) {
		dev_err(vic->dev, "failed to get endpoint\n");
		return -ENODEV;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(node), &vic->endpoint);
	if (ret) {
		of_node_put(node);
		return ret;
	}

	asd = &vic->async_subdev;
	asd->match.fwnode = fwnode_graph_get_remote_port_parent(of_fwnode_handle(node));
	asd->match_type = V4L2_ASYNC_MATCH_FWNODE;

	of_node_put(node);

	ret = v4l2_async_notifier_add_subdev(&vic->notifier, asd);
	if (ret) {
		dev_err(vic->dev, "failed to add aync subdev\n");
	}

	fwnode_handle_put(asd->match.fwnode);

	return ret;
}

#include <stdarg.h>

static int str_to_enum(
    struct device_node *np, const char *propname, void *index, ...)
{
	const char *str;
	int i;
	int ret = of_property_read_string(np, propname, &str);
	if (ret) {
		return ret;
	}

	va_list ap;

	va_start(ap, index);
	while (1) {
		const char *value = va_arg(ap, const char *);
		if (!value) {
			ret = -ENODEV;
			break;
		}
		i = va_arg(ap, int);
		if (!strcmp(value, str)) {
			if (index) {
				*(unsigned int *)index = i;
				vic_dbg("--> %s %s %x\n", propname, value, i);
			}
			break;
		}
	}
	va_end(ap);

	return ret;
}

static int get_of_properties(struct vic_dev *vic, struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct vic_hal_info *info = &vic->info;
	int ret;

	vic->isp_div_clk = of_clk_get(np, 0);
	if (IS_ERR_OR_NULL(vic->isp_div_clk)) {
		dev_err(&pdev->dev, "failed to get isp div clk\n");
		return PTR_ERR(vic->isp_div_clk);
	}

	vic->isp_gate_clk = of_clk_get(np, 1);
	if (IS_ERR_OR_NULL(vic->isp_gate_clk)) {
		dev_err(&pdev->dev, "failed to get isp gate clk\n");
		return PTR_ERR(vic->isp_gate_clk);
	}

	vic->isp_power_clk = of_clk_get(np, 2);
	if (IS_ERR_OR_NULL(vic->isp_power_clk)) {
		dev_err(&pdev->dev, "failed to get isp power clk\n");
		return PTR_ERR(vic->isp_power_clk);
	}

	vic->mipi_clk0 = of_clk_get(np, 3);
	if (IS_ERR_OR_NULL(vic->mipi_clk0)) {
		dev_err(&pdev->dev, "failed to get mipi clk 0\n");
		return PTR_ERR(vic->mipi_clk0);
	}

	vic->mipi_clk1 = of_clk_get(np, 3);
	if (IS_ERR_OR_NULL(vic->mipi_clk1)) {
		dev_err(&pdev->dev, "failed to get mipi clk 1\n");
		return PTR_ERR(vic->mipi_clk1);
	}

	info->isp_clk_rate = 150 * 1000 * 1000;
	of_property_read_u32(np, "vic,isp_clk_rate", &info->isp_clk_rate);

	ret = of_property_read_u32(np, "vic_index", &info->index);
	if (ret) {
		dev_err(&pdev->dev, "failed to get vic_index\n");
		return ret;
	}
	vic_hal_init_iodata(info->index, (unsigned long)vic->iobase);

	unsigned int iobase[3];
	ret = of_property_read_u32_array(np, "mipi_iobase", iobase, 3);
	if (ret) {
		dev_err(&pdev->dev, "failed to get mipi iobase\n");
		return ret;
	}

	iobase[0] = (unsigned int) ioremap(iobase[0], 4096);
	iobase[1] = (unsigned int) ioremap(iobase[1], 4096);
	iobase[2] = (unsigned int) ioremap(iobase[2], 4096);

	mipi_csi_init_iodata(iobase[0], iobase[1], iobase[2],
	                     vic->mipi_clk0, vic->mipi_clk1);

	ret = str_to_enum(np, "vic,bus_type", &info->bus_type,
	                  "mipi", SENSOR_DATA_BUS_MIPI,
	                  "dvp", SENSOR_DATA_BUS_DVP, NULL);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get vic,bus-type, it must set\n");
		return ret;
	}

	str_to_enum(np, "vic,dvp_timing_mode", &info->dvp.timing_mode,
	            "href_mode", DVP_HREF_MODE,
	            "hsync_mode", DVP_HSYNC_MODE,
	            "sony_mode", DVP_SONY_MODE, NULL);

	info->dvp.yuv_data_order = order_1_2_3_4;
	str_to_enum(np, "vic,dvp_yuv_data_order", &info->dvp.yuv_data_order,
	            "order_2_1_4_3", order_2_1_4_3,
	            "order_2_3_4_1", order_2_3_4_1,
	            "order_1_2_3_4", order_1_2_3_4,
	            "order_1_4_3_2", order_1_4_3_2, NULL);

	str_to_enum(np, "vic,dvp_hsync_polarity", &info->dvp.hsync_polarity,
	            "high_active", POLARITY_HIGH_ACTIVE,
	            "low_active", POLARITY_LOW_ACTIVE, NULL);

	str_to_enum(np, "vic,dvp_vsync_polarity", &info->dvp.vsync_polarity,
	            "high_active", POLARITY_HIGH_ACTIVE,
	            "low_active", POLARITY_LOW_ACTIVE, NULL);

	str_to_enum(np, "vic,dvp_pclk_polarity", &info->dvp.pclk_polarity,
	            "sample_rising", POLARITY_SAMPLE_RISING,
	            "sample_falling", POLARITY_SAMPLE_FALLING, NULL);

	str_to_enum(np, "vic,dvp_img_scan_mode", &info->dvp.img_scan_mode,
	            "progress", DVP_IMG_SCAN_PROGRESS,
	            "interlace", DVP_IMG_SCAN_INTERLACE, NULL);

	str_to_enum(np, "vic,dvp_gpio_mode", &info->dvp.gpio_mode,
	            "low_10bit", DVP_PA_LOW_10BIT,
	            "high_10bit", DVP_PA_HIGH_10BIT,
	            "full_12bit", DVP_PA_12BIT,
	            "low_8bit", DVP_PA_LOW_8BIT,
	            "high_8bit", DVP_PA_HIGH_8BIT, NULL);

	info->mipi.data_lanes = 1;
	info->mipi.clk_settle_time = 120;
	info->mipi.data_settle_time = 130;
	of_property_read_u32(np, "vic,mipi_data_lanes", &info->mipi.data_lanes);
	of_property_read_u16(np, "vic,mipi_clk_settle_time", &info->mipi.clk_settle_time);
	of_property_read_u16(np, "vic,mipi_data_settle_time", &info->mipi.data_settle_time);

	of_property_read_u32(np, "vic,video_device_num", &vic->video_device_num);

	return 0;
}

static int vic_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct vic_dev *vic;

	dev_err(&pdev->dev, "start probe vic driver\n");

	vic = kzalloc(sizeof(*vic), GFP_KERNEL);
	if (!vic) {
		dev_err(&pdev->dev, "failed to alloc vic data\n");
		return -ENOMEM;
	}
	vic->dev = &pdev->dev;

	vic->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!vic->res) {
		dev_err(&pdev->dev, "failed to get resource!\n");
		ret = -ENODEV;
		goto free_vic;
	}

	vic->iobase = devm_ioremap_resource(&pdev->dev, vic->res);
	if (!vic->iobase) {
		dev_err(&pdev->dev, "failed to map resource!\n");
		ret = -EINVAL;
		goto free_vic;
	}

	vic->irq = platform_get_irq(pdev, 0);
	if (vic->irq <= 0) {
		dev_err(&pdev->dev, "failed to get irq!\n");
		ret = -ENODEV;
		goto free_vic;
	}

	ret = get_of_properties(vic, pdev);
	if (ret) {
		goto free_vic;
	}

	ret = devm_request_irq(&pdev->dev, vic->irq, vic_irq_dma_handler, 0,
	                       dev_name(&pdev->dev), vic);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq!\n");
		goto free_vic;
	}

	ret = v4l2_device_register(&pdev->dev, &vic->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register v4l2 device!\n");
		goto free_vic;
	}

	ret = vic_get_remote_endpoint(vic);
	if (ret) {
		goto free_vic;
	}

	ret = of_reserved_mem_device_init(vic->dev);
	if (ret) {
		dev_warn(vic->dev, "failed to init reserved mem: %d\n", ret);
	}

	platform_set_drvdata(pdev, vic);

	vic->notifier.ops = &vic_async_ops;
	ret = v4l2_async_notifier_register(&vic->v4l2_dev, &vic->notifier);
	if (ret) {
		dev_err(&pdev->dev, "failed to register aysnc notifier: %d!\n", ret);
		goto free_vic;
	}

	mutex_init(&vic->mutex);
	mutex_init(&vic->queue_mutex);
	INIT_LIST_HEAD(&vic->list);
	spin_lock_init(&vic->lock);

	dev_err(&pdev->dev, "end probe vic driver\n");
	return 0;

free_vic:
	kfree(vic);
	return ret;
}

static int vic_remove(struct platform_device *pdev)
{
	struct vic_dev *vic = platform_get_drvdata(pdev);

	v4l2_async_notifier_unregister(&vic->notifier);

	v4l2_device_unregister(&vic->v4l2_dev);

	of_reserved_mem_device_release(&pdev->dev);

	clk_put(vic->isp_div_clk);
	clk_put(vic->isp_gate_clk);
	clk_put(vic->isp_power_clk);
	clk_put(vic->mipi_clk0);
	clk_put(vic->mipi_clk1);

	kfree(vic);

	return 0;
}

static const struct of_device_id vic_of_match[] = {
	{ .compatible = "ingenic,x2000-simple-vic" },
	{ },
};
MODULE_DEVICE_TABLE(of, vic_of_match);

static struct platform_driver vic_driver = {
	.probe        = vic_probe,
	.remove        = vic_remove,
	.driver        = {
		.name    = "ingenic-simple-vic",
		.owner    = THIS_MODULE,
		.of_match_table = vic_of_match,
	},
};

module_platform_driver(vic_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ingenic vic Host Driver");
