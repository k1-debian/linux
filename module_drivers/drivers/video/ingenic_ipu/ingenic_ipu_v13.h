#ifndef _JZ_IPU_H_
#define _JZ_IPU_H_

#define IPU_LUT_LEN      32

/* ipu output mode */
#define IPU_OUTPUT_TO_LCD_FG1           0x00000002
#define IPU_OUTPUT_TO_LCD_FB0           0x00000004
#define IPU_OUTPUT_TO_LCD_FB1           0x00000008
#define IPU_OUTPUT_TO_FRAMEBUFFER       0x00000010 /* output to user defined buffer */
#define IPU_OUTPUT_MODE_MASK            0x000000FF
#define IPU_DST_USE_COLOR_KEY           0x00000100
#define IPU_DST_USE_ALPHA               0x00000200
#define IPU_OUTPUT_BLOCK_MODE           0x00000400
#define IPU_OUTPUT_MODE_FG0_OFF         0x00000800
#define IPU_OUTPUT_MODE_FG1_TVOUT       0x00001000
#define IPU_DST_USE_PERPIXEL_ALPHA      0x00010000

#ifdef __KERNEL__

/* match HAL_PIXEL_FORMAT_ in system/core/include/system/graphics.h */
enum {
	HAL_PIXEL_FORMAT_RGBA_8888    = 1,
	HAL_PIXEL_FORMAT_RGBX_8888    = 2,
	HAL_PIXEL_FORMAT_RGB_888      = 3,
	HAL_PIXEL_FORMAT_RGB_565      = 4,
	HAL_PIXEL_FORMAT_BGRA_8888    = 5,
	//HAL_PIXEL_FORMAT_BGRX_8888    = 0x8000, /* Add BGRX_8888, Wolfgang, 2010-07-24 */
	HAL_PIXEL_FORMAT_BGRX_8888      = 0x1ff, /* 2012-10-23 */
	HAL_PIXEL_FORMAT_RGBA_5551    = 6,
	HAL_PIXEL_FORMAT_RGBA_4444    = 7,
	HAL_PIXEL_FORMAT_ABGR_8888    = 8,
	HAL_PIXEL_FORMAT_ARGB_8888    = 9,
	HAL_PIXEL_FORMAT_YCbCr_422_SP = 0x10,
	HAL_PIXEL_FORMAT_YCbCr_420_SP = 0x11,
	HAL_PIXEL_FORMAT_YCbCr_422_P  = 0x12,
	HAL_PIXEL_FORMAT_YCbCr_420_P  = 0x13,
	HAL_PIXEL_FORMAT_YCbCr_420_B  = 0x24,
	HAL_PIXEL_FORMAT_YCbCr_422_I  = 0x14,
	HAL_PIXEL_FORMAT_YCbCr_420_I  = 0x15,
	HAL_PIXEL_FORMAT_CbYCrY_422_I = 0x16,
	HAL_PIXEL_FORMAT_CbYCrY_420_I = 0x17,
	HAL_PIXEL_FORMAT_NV12         = 0x18,
	HAL_PIXEL_FORMAT_NV21         = 0x19,
	HAL_PIXEL_FORMAT_BGRA_5551    = 0x1a,
	HAL_PIXEL_FORMAT_HSV          = 0x1b,/*Add HSV*/

	/* suport for YUV420 */
	HAL_PIXEL_FORMAT_JZ_YUV_420_P       = 0x47700001, // YUV_420_P
	HAL_PIXEL_FORMAT_JZ_YUV_420_B       = 0x47700002, // YUV_420_P BLOCK MODE
};

typedef struct {
	unsigned int coef;
	unsigned short int in_n;
	unsigned short int out_n;
} rsz_lut;

#endif /* ifdef __KERNEL__ */

enum {
	ZOOM_MODE_BILINEAR = 0,
	ZOOM_MODE_BICUBE,
	ZOOM_MODE_BILINEAR_ENHANCE,
};

struct YuvCsc {
	// YUV(default) or  YCbCr
	unsigned int csc0;              //  0x400           0x4A8
	unsigned int csc1;              //  0x59C           0x662
	unsigned int csc2;              //  0x161           0x191
	unsigned int csc3;              //  0x2DC           0x341
	unsigned int csc4;              //  0x718           0x811
	unsigned int chrom;             //  128             128
	unsigned int luma;              //  0               16
};

struct YuvStride {
	unsigned int y;
	unsigned int u;
	unsigned int v;
	unsigned int out;
	unsigned int out_uv;
};

struct ipu_param {
	unsigned int        cmd;            /* IPU command */

	unsigned int        bg_w;           /* background weight */
	unsigned int        bg_h;           /* background hight */
	unsigned int        bg_fmt;         /* background format */
	unsigned int        bg_buf_p;       /* background buffer physical addr */

	unsigned int        out_fmt;        /* out format */

	unsigned int        osd_ch0_fmt;
	unsigned int        osd_ch0_para;
	unsigned int        osd_ch0_bak_argb;
	unsigned int        osd_ch0_pos_x;
	unsigned int        osd_ch0_pos_y;
	unsigned int        osd_ch0_src_w;
	unsigned int        osd_ch0_src_h;
	unsigned int        osd_ch0_buf_p;

	unsigned int        osd_ch1_fmt;
	unsigned int        osd_ch1_para;
	unsigned int        osd_ch1_bak_argb;
	unsigned int        osd_ch1_pos_x;
	unsigned int        osd_ch1_pos_y;
	unsigned int        osd_ch1_src_w;
	unsigned int        osd_ch1_src_h;
	unsigned int        osd_ch1_buf_p;

	unsigned int        osd_ch2_fmt;
	unsigned int        osd_ch2_para;
	unsigned int        osd_ch2_bak_argb;
	unsigned int        osd_ch2_pos_x;
	unsigned int        osd_ch2_pos_y;
	unsigned int        osd_ch2_src_w;
	unsigned int        osd_ch2_src_h;
	unsigned int        osd_ch2_buf_p;

	unsigned int        osd_ch3_fmt;
	unsigned int        osd_ch3_para;
	unsigned int        osd_ch3_bak_argb;
	unsigned int        osd_ch3_pos_x;
	unsigned int        osd_ch3_pos_y;
	unsigned int        osd_ch3_src_w;
	unsigned int        osd_ch3_src_h;
	unsigned int        osd_ch3_buf_p;

};

#define IPU_CMD_OSD0        (1<<0)
#define IPU_CMD_OSD1        (1<<1)
#define IPU_CMD_OSD2        (1<<2)
#define IPU_CMD_OSD3        (1<<3)
#define IPU_CMD_OSD         (0xF<<0)
#define IPU_CMD_CSC         (1<<4)

#ifdef __KERNEL__

/*
 * IPU driver's native data
 */

struct ipu_reg_struct {
	char *name;
	unsigned int addr;
};

struct ipu_buf_info {
	unsigned int vaddr_alloc;
	unsigned int paddr;
	unsigned int paddr_align;
	unsigned int size;
};

struct jz_ipu {

	int irq;
	char name[16];

	struct clk *clk;
	struct clk *ahb0_gate; /* T31 IPU mount at AHB0*/
	void __iomem *iomem;
	struct device *dev;
	struct resource *res;
	struct miscdevice misc_dev;

	struct mutex mutex;
	struct completion done_ipu;
	struct completion done_buf;
	struct ipu_buf_info pbuf;
};

struct ipu_flush_cache_para {
	void *addr;
	unsigned int size;
};

#define JZIPU_IOC_MAGIC  'I'
#define IOCTL_IPU_START         _IO(JZIPU_IOC_MAGIC, 106)
#define IOCTL_IPU_RES_PBUFF     _IO(JZIPU_IOC_MAGIC, 114)
#define IOCTL_IPU_GET_PBUFF     _IO(JZIPU_IOC_MAGIC, 115)
#define IOCTL_IPU_BUF_LOCK      _IO(JZIPU_IOC_MAGIC, 116)
#define IOCTL_IPU_BUF_UNLOCK    _IO(JZIPU_IOC_MAGIC, 117)
#define IOCTL_IPU_BUF_FLUSH_CACHE   _IO(JZIPU_IOC_MAGIC, 118)
#define IOCTL_IPU_GET_DMA_FD        _IO(JZIPU_IOC_MAGIC, 119)
#define IOCTL_IPU_GET_DMA_PHY       _IO(JZIPU_IOC_MAGIC, 120)
#define IOCTL_IPU_FLUSH_DMA_FD       _IO(JZIPU_IOC_MAGIC, 121)

static inline unsigned int ipu_reg_read(struct jz_ipu *jzipu, int offset)
{
	return readl(jzipu->iomem + offset);
}

static inline void ipu_reg_write(struct jz_ipu *jzipu, int offset, unsigned int val)
{
	writel(val, jzipu->iomem + offset);
}

#endif  /* #ifdef __KERNEL__ */

#endif // _JZ_IPU_H_
