#ifndef __INGENIC_ISP_DRV_H__
#define __INGENIC_ISP_DRV_H__

#include <linux/clk.h>

#include <media/media-device.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>

#include <system_sensor_drv.h>
#include <tiziano-isp.h>
#include <tiziano-core-ctrl.h>
#include "isp-sensor.h"

#define     MAX_SUB_DEVS        5
#define     MAX_ASYNC_SUBDEVS   1   /*每个ISP同时最大支持1个sensor.*/

extern unsigned int MAIN_ISP_INDEX;
extern unsigned int SEC_ISP_INDEX;

struct isp_video_format {
	const char *name;
	unsigned int fourcc;
	unsigned int depth[3];  /*max 3 plane*/
	unsigned int mbus_code;
	unsigned int num_planes;
	enum v4l2_colorspace colorspace;
};

struct isp_pipeline {
	struct media_pipeline pipeline;
};

#define to_isp_buffer(buf)      container_of(buf, struct isp_video_buffer, vb2)
struct isp_video_buffer {
	struct vb2_v4l2_buffer vb2;
	struct list_head list_entry;
	unsigned int uv_offset; /* 当使用single plane的buffer时,表明uv的偏移大小.*/
};

/*每个mscaler_video_device代表一个数据节点示例, mscaler有三个channel，所有三个video_device.*/
/*通用video_device， mscaler 的三个channel可以注册, vic也可以注册，所以通称为isp_video_device.*/
#define ISP_VIDEO_PAD_SINK  0
#define ISP_VIDEO_NUM_PADS  1
struct isp_video_device {
	char *name;

	struct video_device video;
	struct device *dev;
	struct isp_video_ctx *ctx;

	struct media_pad pad;

	struct ispcam_device *ispcam;

	void *alloc_ctx;
	struct vb2_queue *queue;
	struct mutex queue_lock;

	struct mutex mutex;

	struct mutex stream_lock;
	struct isp_pipeline pipe;

	struct v4l2_format format;
	struct isp_video_format *(*find_format)(const u32 *pixelformat, const u32 *mbus_code, int index);
	struct isp_video_format *current_format;
	const struct isp_video_ops *ops;

	int bypass; /*bypass isp.*/
	int prestart;
	unsigned int max_buffer_num;    /*max vb2 num*/
};

struct isp_video_ops {
	/**/
	struct isp_video_format *(*find_format)(const u32 *pixelformat, const u32 *mbus_code, int index);

	/*qbuf*/
	/*called by VIDIOC_QBUF in isp-video.c*/
	int (*qbuf)(struct isp_video_device *ispvideo, struct isp_video_buffer *isp_buffer);
};

/* isp-core-tuning*/
struct isp_core_tuning_ctrl {
	unsigned int vflip;
	unsigned int hflip;
	TISP_MODE_DN_E daynight;
	int saturation;
	int brightness;
	int contrast;
	int sharpness;
	int hue;
	tisp_defog_t attr;
	tisp_wb_attr_t wb_attr;
};

enum isp_image_tuning_private_cmd_id {
	IMAGE_TUNING_CID_MODULE_CONTROL = V4L2_CID_PRIVATE_BASE,
	IMAGE_TUNING_CID_DAY_OR_NIGHT,
	IMAGE_TUNING_CID_AE_LUMA,
	IMAGE_TUNING_CID_FACE_AE_CONTROL,
	IMAGE_TUNING_CID_FACE_AWB_CONTROL,
	IMAGE_TUNING_CID_SWITCH_BIN,
	IMAGE_TUNING_CID_SET_DEFAULT_BIN_PATH,
	IMAGE_TUNING_CID_DEFOG_CONTROL,
	IMAGE_TUNING_CID_CSC_UV_THRESHOLD,
	IMAGE_TUNING_CID_HILIGHTDEPRESS_CONTROL,
	IMAGE_TUNING_CID_AF_METRIC,
};

#define CSI_PAD_SINK    0
#define CSI_PAD_SOURCE  1
#define CSI_NUM_PADS    2
struct csi_device {
	struct device *dev;
	int irq;
	void __iomem *iobase;
	unsigned int phy_base;
	struct ispcam_device *ispcam;   /*struct ispcam_device*/

	struct v4l2_subdev sd;

	struct media_pad *pads;
	struct v4l2_subdev_format formats[CSI_NUM_PADS];

	struct clk *gate_clk;

	int reset_index;
	int enabled;

	int settle_time;
	struct sensor_info *sensor_info;
};

#define VIC_PAD_SINK        0
#define VIC_PAD_SOURCE      1
#define VIC_NUM_PADS        2
struct vic_device {
	struct device *dev;

	int irq;
	void __iomem *iobase;
	struct ispcam_device *ispcam;   /*struct ispcam_device*/

	struct v4l2_subdev sd;

	struct media_pad *pads;
	struct isp_video_device ispvideo;   /*VIC support video out*/
	struct v4l2_subdev_format formats[VIC_NUM_PADS];

	unsigned int active_sink_pad;

	struct list_head dma_queued_list;
	spinlock_t lock;
	int enabled;

	unsigned int buffer_index;
	int buffer_count;
	int stop_flag;
	int global_reset;
	int frame_capture_counter;

	unsigned int framenum;

	unsigned int snap_paddr;
	void *snap_vaddr;
	struct completion snap_comp;

	unsigned int dma_complete;
	struct sensor_info *sensor_info;
};

#define ISP_PAD_SINK    0
#define ISP_PAD_SOURCE  1
#define ISP_NUM_PADS    2

#define MDNS_BUF 0
#define DUALSENSOR_BUF 1
struct buf_info {
	dma_addr_t paddr;
	void *vaddr;
	unsigned int size;
};

#define ISP_IRQ_BCSH_FRM_DONE 34

struct isp_device {
	struct device *dev;
	int irq;
	void __iomem *iobase;
	struct ispcam_device *ispcam;
	unsigned int index;
	spinlock_t lock;

	void __iomem *cpm_reset;
	unsigned int bit_sr;
	unsigned int bit_stp;
	unsigned int bit_ack;

	struct v4l2_subdev sd;

	struct media_pad *pads;
	struct v4l2_subdev_format formats[ISP_NUM_PADS];

	struct clk *div_clk;
	struct clk *gate_clk;
	struct clk *power_clk;

	multisensor_mode_t multi_mode;

	struct task_struct *process_thread;
	//  tisp_core_t core;
	int enabled;

	/* IRQ callbacks.*/
	int (*irq_func_cb[42])(void);

	struct buf_info buf_info[3];
	struct isp_core_tuning_ctrl ctrls;
	struct sensor_info *sensor_info;

	tx_isp_bin_path bpath;
};

#ifdef CONFIG_MSCA_BDEV
struct mscaler_backend_device {
	struct device *dev;
	struct list_head processing_list;
	spinlock_t lock;
	wait_queue_head_t wq;

	struct task_struct *process_thread;

	/* mscaler output fmt. */
	unsigned int width;
	unsigned int height;
	unsigned int fmt;
	unsigned int ch;    /*which channel this backend device belongs to.*/

	int activated;  /* is bdev activated??*/

	struct ingenic_venc_ctx *ctx;
};
#endif

/*MSCALER最大支持3个ch输出，每个ch支持不同的缩放等级. 这里只使用1个channel.*/
#define MSCALER_MAX_CH          3
#define MSCALER_PAD_SINK        0
#define MSCALER_PAD_SOURCE_CH0      1
#define MSCALER_PAD_SOURCE_CH1      2
#define MSCALER_PAD_SOURCE_CH2      3
#define MSCALER_NUM_PADS        4
struct mscaler_device {
	struct device *dev;
	int irq;
	void __iomem *iobase;
	struct ispcam_device *ispcam;
	unsigned int index;

	struct v4l2_subdev sd;
	struct media_pad *pads;

	struct list_head dma_queued_list[MSCALER_MAX_CH];
	struct list_head dma_pending_list[MSCALER_MAX_CH];

	struct isp_video_device ispvideo[MSCALER_MAX_CH];
	/*包含了输入pad和输出pad的format*/
	struct v4l2_subdev_format formats[MSCALER_NUM_PADS];
	tisp_channel_attr_t attr[MSCALER_MAX_CH];

	spinlock_t lock;

	unsigned int state[MSCALER_MAX_CH]; /*0: stopped, 1: streaming*/
	unsigned int framenum[MSCALER_MAX_CH];

	struct kobject *mask_kobj;
	struct kobject *mask_ch_kobj[MSCALER_MAX_CH];

#ifdef CONFIG_MSCA_BDEV
	struct mscaler_backend_device bdev[MSCALER_MAX_CH];
#endif
};

/* 每个 isp_async_device 代表一个异步注册的 sensor设备. */
struct isp_async_device {
	struct v4l2_async_connection asd;
	struct v4l2_subdev *sd;
	int index;

	enum v4l2_mbus_type bus_type;
	struct v4l2_mbus_config_parallel parallel;
	struct v4l2_mbus_config_mipi_csi2 mipi_csi2;

	int enabled;

	unsigned int    source_pad;
};

struct sensor_device {
	struct isp_async_device *isd;

	struct ispcam_device *ispcam;
	struct isp_device *isp;

};

#define to_isp_video_ctx(fh)  container_of(fh, struct isp_video_ctx, fh)

/*every open create one ctx, should only one ctx is running at a time*/
struct isp_video_ctx {
	struct isp_video_device *ispvideo;

	struct v4l2_fh *fh; /*handle*/
	struct vb2_queue queue;
	struct v4l2_format format;
	unsigned int uv_offset;
	unsigned int payload_size;
};

#define SUBDEV_PAD_SINK     0
#define SUBDEV_PAD_SOURCE   1
#define SUBDEV_NUM_PADS     2

struct ispcam_data {
	char chip_name[32];
};

struct ispcam_device {
	struct device *dev;
	struct platform_device *subdevs[MAX_SUB_DEVS];
	unsigned int subdev_num;
	char chip_name[32];
	int dev_nr;

	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;    /*异步通知，用来接收sensor注册完成信息.*/

	/*TODO: 应该只会支持一个sensor, MAX_ASYNC_SUBDEVS 应该为1*/
	struct isp_async_device *isd[MAX_ASYNC_SUBDEVS];
	//struct v4l2_async_subdev asd[MAX_ASYNC_SUBDEVS];
	unsigned int asd_num;       /*实际的async device个数，从endpoint中解析出来的.*/
	struct v4l2_fwnode_endpoint vep[MAX_ASYNC_SUBDEVS]; /*每个异步设备对应到一个端点上.*/

	int enabled;

	struct mutex    mutex;
	struct isp_device   *isp;
	struct csi_device   *csi;
	struct vic_device   *vic;
	struct mscaler_device   *mscaler;
	struct sensor_device    sensor;
};

/* isp-video*/
extern int isp_video_init(struct isp_video_device *ispvideo, char *name, const struct isp_video_ops *ops);

extern int isp_video_cleanup(struct isp_video_device *ispvideo);

extern int isp_video_register(struct isp_video_device *ispvideo, struct v4l2_device *v4l2_dev, int nr);

extern int isp_video_unregister(struct isp_video_device *ispvideo);

static inline struct v4l2_mbus_config_parallel *ispcam_get_bus_parallel(struct ispcam_device *ispcam)
{
	return &ispcam->isd[0]->parallel;
}

static inline struct v4l2_mbus_config_mipi_csi2 *ispcam_get_bus_mipi_csi2(struct ispcam_device *ispcam)
{
	return &ispcam->isd[0]->mipi_csi2;
}

/* sensor */

int sensor_device_probe(struct sensor_device *sensor, struct ispcam_device *ispcam);

#ifdef CONFIG_MSCA_BDEV
	/* mscaler backend */
	int ms_bdev_init(struct mscaler_backend_device *ms_bdev);

	int ms_bdev_deinit(struct mscaler_backend_device *ms_bdev);

	int ms_bdev_qbuf(struct mscaler_backend_device *ms_bdev, struct isp_video_buffer *isp_buffer);
#endif

#endif
