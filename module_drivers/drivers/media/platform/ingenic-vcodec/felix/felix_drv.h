#ifndef __INGENIC_FELIX_DRV_H__
#define __INGENIC_FELIX_DRV_H__

#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>

#include "avcodec.h"
#include "frame.h"
#include "h264dec.h"


#define INGENIC_VCODEC_DEC_NAME "felix-vdec"
#define INGENIC_VCODEC_MAX_PLANES   3

enum ingenic_fmt_type {
	INGENIC_FMT_FRAME = 0,
	INGENIC_FMT_DEC = 1,
};

enum felix_raw_format {
	FELIX_TILE_MODE = 0,
	FELIX_420P_MODE = 4,    /* not support */
	FELIX_NV12_MODE = 8,
	FELIX_NV21_MODE = 12,   /* not support */
	FELIX_FORMAT_NONE,
};

struct ingenic_video_fmt {
	u32 fourcc;
	enum ingenic_fmt_type type;
	u32 num_planes;
	enum felix_raw_format format;
};

struct ingenic_vcodec_framesizes {
	u32 fourcc;
	struct v4l2_frmsize_stepwise stepwise;
};

enum ingenic_vcodec_state {
	INGENIC_STATE_IDLE = 0,
	INGENIC_STATE_HEADER,
	INGENIC_STATE_RUNNING,
	INGENIC_STATE_ABORT,
};


/* private vcode buf related to each vb2, TODO: will implement in future. */
/*
 *  这里的大小必须是struct v4l2_m2m_buffer.
 * */
struct ingenic_vcodec_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;  //struct v4l2_m2m_buffer

	/*下面才是自己私有的数据结构.*/
	AVFrame frame;
	int is_last_frame;  /*End of Stream.*/
	int used;
};

enum ingenic_q_type {
	INGENIC_Q_DATA_SRC = 0,
	INGENIC_Q_DATA_DST = 1,
};

struct ingenic_vcodec_q_data {
	unsigned int    visible_width;
	unsigned int    visible_height;
	unsigned int    coded_width;
	unsigned int    coded_height;
	enum v4l2_field field;
	unsigned int    bytesperline[INGENIC_VCODEC_MAX_PLANES];
	unsigned int    sizeimage[INGENIC_VCODEC_MAX_PLANES];
	unsigned int    real_sizeimage[INGENIC_VCODEC_MAX_PLANES];
	struct ingenic_video_fmt    *fmt;
};

/*  vpu hw ctx ?? */




/* Each open creates a ctx ? */
struct ingenic_vdec_ctx {

	struct ingenic_vdec_dev *dev;
	struct v4l2_fh fh;
	struct v4l2_m2m_ctx *m2m_ctx;

	struct v4l2_ctrl_handler ctrl_hdl;

	int id; /* used for debug ?*/

	struct ingenic_vcodec_q_data q_data[2];

	wait_queue_head_t queue;
	int output_stopped;
	int capture_stopped;

	enum ingenic_vcodec_state state;

	enum v4l2_colorspace colorspace;
	struct work_struct decode_work;

	struct ingenic_vcodec_buf *empty_flush_buf;

	int int_cond;
	struct mutex lock;
	struct mutex buffer_mutex;

	AVCodecContext *avctx;
	H264Context *h;
	AVBuffer *h_avbuf;  /* handle AVBuffer for "h" */
};


struct ingenic_vdec_dev {
	struct v4l2_device v4l2_dev;
	struct video_device *vfd;
	struct device *dev;

	struct v4l2_m2m_dev *m2m_dev;
	struct platform_device *plat_dev;

	struct mutex dev_mutex;

	/* TODO: add workqueue*/
	struct workqueue_struct *dec_workqueue;

	spinlock_t spinlock;

	struct ingenic_vdec_ctx *curr_ctx;
	struct vb2_dc_conf *alloc_ctx;

	int id_counter;

	void __iomem *reg_base;
	int irq;
	struct clk *clk_gate;
	struct clk *clk;
};


/* -----------------------------------helpler-------------------------------------*/
static inline struct ingenic_vdec_ctx *fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct ingenic_vdec_ctx, fh);
}

static inline struct ingenic_vdec_ctx *ctrl_to_ctx(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct ingenic_vdec_ctx, ctrl_hdl);
}


#endif
