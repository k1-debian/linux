#ifndef __JPEGE_DRV_H__
#define __JPEGE_DRV_H__

#define JPEGE_PIX_FMT_RGB       0
#define JPEGE_PIX_FMT_YUV444    1

// 设置对yuv图像的抽样方式，如果输入的是rgb888那么内部会转换为yuv
#define JPEGE_SUBSAMP_TYPE_NONE  0x1
#define JPEGE_SUBSAMP_TYPE_422H  0x2
#define JPEGE_SUBSAMP_TYPE_420  0x3

typedef enum {
	IMPP_PIX_FMT_NV12,
	IMPP_PIX_FMT_NV21,
	IMPP_PIX_FMT_NV16,
	IMPP_PIX_FMT_YUV422,
	IMPP_PIX_FMT_YUV422P,
	IMPP_PIX_FMT_I420,
	IMPP_PIX_FMT_I422,
	IMPP_PIX_FMT_YUV420p,
	IMPP_PIX_FMT_RAW8,
	IMPP_PIX_FMT_RAW10,
	IMPP_PIX_FMT_RGBA_8888,
	IMPP_PIX_FMT_RGBX_8888,
	IMPP_PIX_FMT_BGRA_8888,
	IMPP_PIX_FMT_BGRX_8888,
	IMPP_PIX_FMT_ABGR_8888,
	IMPP_PIX_FMT_ARGB_8888,
	IMPP_PIX_FMT_RGB_888,
	IMPP_PIX_FMT_BGR_888,
	IMPP_PIX_FMT_RGB_565,
	IMPP_PIX_FMT_RGB_555,
	IMPP_PIX_FMT_RGBA_5551,
	IMPP_PIX_FMT_BGRA_5551,
	IMPP_PIX_FMT_ARGB_1555,
	IMPP_PIX_FMT_HSV,

	/* Camera Raw8 */
	IMPP_PIX_FMT_SBGGR8,
	IMPP_PIX_FMT_SGBRG8,
	IMPP_PIX_FMT_SGRBG8,
	IMPP_PIX_FMT_SRGGB8,

	/* Camera Raw10 */
	IMPP_PIX_FMT_SBGGR10,
	IMPP_PIX_FMT_SGBRG10,
	IMPP_PIX_FMT_SGRBG10,
	IMPP_PIX_FMT_SRGGB10,

	/* Camera Raw12 */
	IMPP_PIX_FMT_SBGGR12,
	IMPP_PIX_FMT_SGBRG12,
	IMPP_PIX_FMT_SGRBG12,
	IMPP_PIX_FMT_SRGGB12,

	/* Y only.*/
	IMPP_PIX_FMT_GREY,
	IMPP_PIX_FMT_Y4,
	IMPP_PIX_FMT_Y6,
	IMPP_PIX_FMT_Y12,
	IMPP_PIX_FMT_Y16,

	/* YUV422. */
	IMPP_PIX_FMT_YUYV,
	IMPP_PIX_FMT_YYUV,
	IMPP_PIX_FMT_YVYU,
	IMPP_PIX_FMT_VYUY,
	/* YUV422. */
	IMPP_PIX_FMT_YUV444,
} IMPP_PIX_FMT;

struct jz_jpeg_enc {
	int irq;
	char name[16];
	struct clk *clk;
	void __iomem *iomem;
	struct device *dev;
	struct resource *res;
	struct miscdevice misc_dev;

	struct mutex mutex;
	struct completion done_encoder;
	struct completion done_buf;
};

struct jpege_in_info {
	unsigned int src_paddr;
	unsigned int dst_paddr;
	unsigned int dst_bufsize;
	unsigned short frame_width;
	unsigned short frame_height;
	short qa;
	IMPP_PIX_FMT pix_fmt;
	char subsamp;
	unsigned short rstm_request;
};

struct jpege_out_info {
	unsigned int imagesize;
};

struct jpege_info {
	struct jpege_in_info in;
	struct jpege_out_info out;
};

struct jpege_cache_info {
	unsigned int kaddr;
	unsigned int paddr;
	unsigned int len;
	int dir;
};

#define JZ_JPEGE_IO_MAGIC   'J'

#define JZ_JPEGE_START          _IO(JZ_JPEGE_IO_MAGIC,100)
#define JZ_JPEGE_ALLOC_DMABUF   _IO(JZ_JPEGE_IO_MAGIC,101)
#define JZ_JPEGE_ALLOC_BUFFER   _IO(JZ_JPEGE_IO_MAGIC,102)
#define JZ_JPEGE_FREE_BUFFER    _IO(JZ_JPEGE_IO_MAGIC,103)
#define JZ_JPEGE_FLUSH_CACHE    _IO(JZ_JPEGE_IO_MAGIC,104)
#define JZ_JPEGE_FLUSH_DMACACHE _IO(JZ_JPEGE_IO_MAGIC,105)
#define JZ_JPEGE_GET_DMABUF_PADDR _IO(JZ_JPEGE_IO_MAGIC,106)

#endif
