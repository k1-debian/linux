#ifndef __JPEGD_DRV_H__
#define __JPEGD_DRV_H__

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

	/* YUV444. */
	IMPP_PIX_FMT_YUV444,
} IMPP_PIX_FMT;

struct jz_jpeg_dec {
	int irq;
	char name[16];
	struct clk *clk;
	void __iomem *iomem;
	struct device *dev;
	struct resource *res;
	struct miscdevice misc_dev;

	struct mutex mutex;
	struct completion done_decoder;
	struct completion done_buf;
};

struct jpegd_param {
	unsigned int file_size;
	unsigned int src_paddr;
	unsigned int dst_paddr;
	IMPP_PIX_FMT out_fmt;
};

struct jpegd_out_info {
	unsigned int yuv_type;
	unsigned short width;
	unsigned short height;
	unsigned int size;
};

struct jpegd_info {
	struct jpegd_param in_param;
	struct jpegd_out_info out_info;
};

struct jpegd_cache_info {
	unsigned int kaddr;
	unsigned int paddr;
	unsigned int len;
	int dir;
};

#define JZ_JPEGD_IO_MAGIC   'J'

#define JZ_JPEGD_START          _IO(JZ_JPEGD_IO_MAGIC,100)
#define JZ_JPEGD_ALLOC_DMABUF   _IO(JZ_JPEGD_IO_MAGIC,101)
#define JZ_JPEGD_ALLOC_BUFFER   _IO(JZ_JPEGD_IO_MAGIC,102)
#define JZ_JPEGD_FREE_BUFFER    _IO(JZ_JPEGD_IO_MAGIC,103)
#define JZ_JPEGD_FLUSH_CACHE    _IO(JZ_JPEGD_IO_MAGIC,104)
#define JZ_JPEGD_FLUSH_DMACACHE _IO(JZ_JPEGD_IO_MAGIC,105)
#define JZ_JPEGD_GET_DMABUF_PADDR   _IO(JZ_JPEGD_IO_MAGIC,106)
#define JZ_JPEGD_GET_RESERVED_STA   _IO(JZ_JPEGD_IO_MAGIC,107)

#endif
