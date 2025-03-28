#ifndef __UAPI_INGENICFB_H__
#define __UAPI_INGENICFB_H__

/* structs export to userspace */

#define PIXEL_ALIGN 4
#define DESC_ALIGN 8
#define MAX_BITS_PER_PIX (32)
#define DPU_MAX_SIZE (2047)
#define DPU_MIN_SIZE (4)
#define DPU_STRIDE_SIZE (4096)
#define DPU_SCALE_MIN_SIZE (20)

#define MAX_STRIDE_VALUE (4096)

/* Maximum layers supported by DPU hardware */
#define DPU_SUPPORT_MAX_LAYERS 4

/* Maximum frames supported by DPU drivers */
#define DPU_SUPPORT_MAX_FRAMES 3

enum {
	DC_WB_FORMAT_8888 = 0,
	DC_WB_FORMAT_565 = 1,
	DC_WB_FORMAT_555 = 2,
	DC_WB_FORMAT_YUV422 = 3,    /*Not support*/
	DC_WB_FORMAT_MONO = 4,      /*Not support*/
	DC_WB_FORMAT_888 = 6,
};

enum {
	LAYER_CFG_FORMAT_RGB555 = 0,
	LAYER_CFG_FORMAT_ARGB1555 = 1,
	LAYER_CFG_FORMAT_RGB565 = 2,
	LAYER_CFG_FORMAT_RGB888 = 4,
	LAYER_CFG_FORMAT_ARGB8888 = 5,
	LAYER_CFG_FORMAT_MONO8 = 6,
	LAYER_CFG_FORMAT_MONO16 = 7,
	LAYER_CFG_FORMAT_NV12 = 8,
	LAYER_CFG_FORMAT_NV21 = 9,
	LAYER_CFG_FORMAT_YUV422 = 10,
	LAYER_CFG_FORMAT_TILE_H264 = 12,
};

enum {
	LAYER_CFG_COLOR_RGB = 0,
	LAYER_CFG_COLOR_RBG = 1,
	LAYER_CFG_COLOR_GRB = 2,
	LAYER_CFG_COLOR_GBR = 3,
	LAYER_CFG_COLOR_BRG = 4,
	LAYER_CFG_COLOR_BGR = 5,
};

enum {
	LAYER_Z_ORDER_0 = 0, /* bottom */
	LAYER_Z_ORDER_1 = 1,
	LAYER_Z_ORDER_2 = 2,
	LAYER_Z_ORDER_3 = 3, /* top */
};

struct ingenicfb_lay_cfg {
	unsigned int lay_en: 1;
	unsigned int tlb_en: 1;
	unsigned int lay_scale_en: 1;
	unsigned int lay_z_order: 3;
	unsigned int format: 4;
	unsigned int color: 3;
	unsigned int g_alpha_en: 1;
	unsigned int g_alpha_val: 8;
	unsigned int source_w;
	unsigned int source_h;
	unsigned int stride;
	unsigned int uv_stride; /*NV12 only.*/
	unsigned int scale_w;
	unsigned int scale_h;
	unsigned int disp_pos_x;
	unsigned int disp_pos_y;
	unsigned int addr[3];
	unsigned int uv_addr[3]; /*uv_addr for NV12 format.*/
	unsigned int domain_multi;
	unsigned int version;
	unsigned int reserve[2];
	/* source image crop settings, according to source_w, source_h */
	unsigned int src_crop_x;    /* start pos x in memory of source image.*/
	unsigned int src_crop_y;    /* start pos y in memory of source image.*/
	unsigned int src_crop_w;    /* crop w for source image. */
	unsigned int src_crop_h;    /* crop h for source image. */
};

struct wback_cfg {
	int en;
	unsigned int fmt;
	unsigned int addr;
	unsigned int stride;
	int dither_en;
};

struct ingenicfb_frm_cfg {
	struct ingenicfb_lay_cfg lay_cfg[DPU_SUPPORT_MAX_LAYERS];
	struct wback_cfg wback_info;
	unsigned int width;
	unsigned int height;
};

/*
    var_screeninfo 的私有扩展，实现更新buffer内容的同时，更新buffer属性.
    扩展 pandisplay 的功能.

    MAGIC: Ingenic Framebuffer Var Extended
*/
#define VAR_INFO_EXT_MAGIC      ('I'<<24 | 'F' << 16 | 'V' << 8 | 'E' << 0)
struct ingenicfb_var_screeninfo_extended {
	unsigned int magic;
	unsigned int use_extended;
	struct ingenicfb_lay_cfg lay_cfg;
};

enum cache_op_direction {
	CACHE_OP_BIDIRECTIONAL = 0,
	CACHE_OP_TO_DEVICE = 1,
	CACHE_OP_FROM_DEVICE = 2,
	CACHE_OP_NONE = 3,
};

struct ingenicfb_cache_op {
	unsigned int offset;
	unsigned int size;
	enum cache_op_direction dir;
};

#define JZFB_PUT_FRM_CFG        _IOWR('F', 0x101, struct ingenicfb_frm_cfg *)
#define JZFB_GET_FRM_CFG        _IOWR('F', 0x102, struct ingenicfb_frm_cfg *)

#define JZFB_SET_CSC_MODE       _IOW('F', 0x120, csc_mode_t)
#define JZFB_SET_ROT_ANGLE      _IOW('F', 0x121, int)
#define JZFB_GET_ROT_ANGLE      _IOW('F', 0x122, int)
#define JZFB_USE_TLB            _IOW('F', 0x124, unsigned int)
#define JZFB_DMMU_MAP           _IOWR('F', 0x130, struct dpu_dmmu_map_info)
#define JZFB_DMMU_UNMAP         _IOWR('F', 0x131, struct dpu_dmmu_map_info)
#define JZFB_DMMU_UNMAP_ALL     _IOWR('F', 0x132, struct dpu_dmmu_map_info)
#define JZFB_DMMU_MEM_SMASH     _IOWR('F', 0x133, struct smash_mode)
#define JZFB_DMMU_DUMP_MAP      _IOWR('F', 0x134, unsigned long)
#define JZFB_DMMU_FLUSH_CACHE       _IOWR('F', 0x135, struct dpu_dmmu_map_info)
#define JZFB_DUMP_LCDC_REG      _IOW('F', 0x150, int)
#define JZFB_GET_DMA_PHY        _IOW('F', 0x151, unsigned int)

#define JZFB_SET_VSYNCINT       _IOW('F', 0x210, int)
#define JZFB_GET_LAYERS_NUM     _IOWR('F', 0x211, unsigned int)
#define JZFB_CACHE_OP            _IOW('F', 0x212, int)

#endif
