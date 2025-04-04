#ifndef __INGENIC_HELIX_DRV_H__
#define __INGENIC_HELIX_DRV_H__

#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>

//#include "api/helix_x264_enc.h"
//#include "api/helix_jpeg_enc.h"
#include "api/helix.h"
#include "helix_buf.h"

#include "h264e_rc.h"
#include "jpge.h"
#include "jpgd.h"

#define INGENIC_VCODEC_ENC_NAME "helix-venc"
#define INGENIC_VCODEC_MAX_PLANES   3

enum ingenic_fmt_type {
	INGENIC_FMT_FRAME   = 0,
	INGENIC_FMT_ENC     = 1,
	INGENIC_FMT_DEC     = 2,
};

struct ingenic_video_fmt {
	u32 fourcc;
	enum ingenic_fmt_type type;
	u32 num_planes;
	enum helix_raw_format format;
};

struct ingenic_codec_framesizes {
	u32 fourcc;
	struct v4l2_frmsize_stepwise stepwise;
};

struct ingenic_video_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;

	struct video_frame_buffer buf;
};

enum ingenic_q_type {
	INGENIC_Q_DATA_SRC = 0,
	INGENIC_Q_DATA_DST = 1,
};

/* Queue Data. */
struct ingenic_venc_q_data {
	unsigned int    visible_width;
	unsigned int    visible_height;
	unsigned int    coded_width;
	unsigned int    coded_height;
	enum v4l2_field field;
	unsigned int    bytesperline[INGENIC_VCODEC_MAX_PLANES];
	unsigned int    sizeimage[INGENIC_VCODEC_MAX_PLANES];
	struct ingenic_video_fmt    *fmt;

};

/*
  when ctx is created, IDLE
  when start_streaming, START,
  when abort, ABORT,
*/

enum ingenic_venc_state {
	INGENIC_STATE_IDLE = 0,
	INGENIC_STATE_HEADER,
	INGENIC_STATE_RUNNING,
	INGENIC_STATE_ABORT,
};

/* v4l2 set parm to sw_parm, */
/* sw_parm api to sliceinfo. */
/* Sliceinfo to dma desc. */

/* helix_ctrl_if_start. */

struct ingenic_venc_ctx {
	struct ingenic_venc_dev *dev;

	struct v4l2_fh fh;
	struct v4l2_m2m_ctx *m2m_ctx;

	struct v4l2_ctrl_handler ctrl_hdl;

	int id;

	int capture_stopped;
	int output_stopped;

	struct ingenic_venc_q_data q_data[2];
	enum ingenic_venc_state state;

	wait_queue_head_t queue;

	int int_cond;
	int int_status;

	struct work_struct encode_work;
	enum v4l2_colorspace colorspace;
	//enum v4l2_ycbcr_encoding ycbcr_enc;
	//enum v4l2_quantization quantization;
	//enum v4l2_xfer_func xfer_func;

	int codec_id;
#define CODEC_ID_H264E  1
#define CODEC_ID_JPGE   2
#define CODEC_ID_JPGD   3
	union {
		struct h264e_ctx h264e_ctx;
		struct jpge_ctx jpge_ctx;
		struct jpgd_ctx jpgd_ctx;
	};

};

struct ingenic_venc_dev {

	struct v4l2_device v4l2_dev;
	struct video_device *vfd_enc;
	struct device *dev;

	struct v4l2_m2m_dev *m2m_dev_enc;
	struct platform_device *plat_dev;

	struct mutex dev_mutex;
	struct workqueue_struct *encode_workqueue;

	spinlock_t spinlock;

	struct ingenic_venc_ctx *curr_ctx;
	struct vb2_dc_conf *alloc_ctx;

	int id_counter;

	void __iomem *reg_base;
	int irq;
	struct clk *clk_gate;
	struct clk *clk;
};

static inline struct ingenic_venc_ctx *fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct ingenic_venc_ctx, fh);
}

static inline struct ingenic_venc_ctx *ctrl_to_ctx(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct ingenic_venc_ctx, ctrl_hdl);
}

/* 内核全局g_dev.*/
extern struct ingenic_venc_dev *g_dev;
static inline void ingenic_venc_dev_set(struct ingenic_venc_dev *dev)
{
	g_dev = dev;
}
static inline struct ingenic_venc_dev *ingenic_venc_dev_get(void)
{
	return g_dev;
}

#endif
