#ifndef _JZM_H264_API_H_
#define _JZM_H264_API_H_

#include "jzm_vpu.h"

/*-------------- vmem def --------------*/
#define VRAM_TCSM1_BASE (VPU_BASE + 0xC0000)
#define VRAM_SRAM_BASE  (VPU_BASE + 0xF0000)
#define DESP_FIFO_WIDTH 4
#define DESP_FIFO_DEPTH (1 << DESP_FIFO_WIDTH)
#define VRAM_SCH_FIFO_DEPTH DESP_FIFO_DEPTH
#define TOP_NEI_ADDR (VRAM_TCSM1_BASE)
#define TOP_NEI_SIZE (144*16*4)
#define MC_DESP_ADDR (TOP_NEI_ADDR + TOP_NEI_SIZE)
#define MC_DESP_ONE_SIZE (128*4)
#define MC_DESP_SIZE (MC_DESP_ONE_SIZE*DESP_FIFO_DEPTH)
#define DBLK_MV_ADDR (MC_DESP_ADDR + MC_DESP_SIZE)
#define DBLK_MV_ONE_SIZE (64*4)
#define DBLK_MV_SIZE (DBLK_MV_ONE_SIZE*DESP_FIFO_DEPTH)
#define VMAU_DESP_ADDR (DBLK_MV_ADDR + DBLK_MV_SIZE)
#define VMAU_DESP_ONE_SIZE (16*4)
#define VMAU_DESP_SIZE (VMAU_DESP_ONE_SIZE*DESP_FIFO_DEPTH)
#define DBLK_DESP_ADDR (VMAU_DESP_ADDR + VMAU_DESP_SIZE)
#define DBLK_DESP_ONE_SIZE (8*4)
#define DBLK_DESP_SIZE (DBLK_DESP_ONE_SIZE*DESP_FIFO_DEPTH)
#define MOTION_IWTA_BASE ((DBLK_DESP_ADDR+DBLK_DESP_SIZE + 1024) & 0xFFFFFC00)
#define MOTION_IWTA_SIZE (2048)
#define MOTION_DSA_BASE (MOTION_IWTA_BASE + MOTION_IWTA_SIZE)
#define MAU_ENDF_BASE (MOTION_DSA_BASE+4)//(MAU_SRC_BASE + MAU_SRC_SIZE)
#define DBLK_ENDF_BASE ( MAU_ENDF_BASE + 4 )
#define DBLK_GP_ENDF_BASE ( DBLK_ENDF_BASE + 4 )
#define TCSM1_END (MAU_SRC_BASE+MAU_SRC_SIZE)
#define RESIDUAL_DOUT_ADDR (VRAM_SRAM_BASE)
#define RESIDUAL_DOUT_ONE_SIZE (256*4)
#define RESIDUAL_DOUT_SIZE (RESIDUAL_DOUT_ONE_SIZE*DESP_FIFO_DEPTH)
#define SRAM_END (RESIDUAL_DOUT_ADDR+RESIDUAL_DOUT_SIZE)
#define PMON_BUF (SRAM_END)

/*----------------------------*/
#define JZM_H264_I_TYPE 1
#define JZM_H264_P_TYPE 2
#define JZM_H264_B_TYPE 4

#define ROA_ALN     256
#define DOUT_Y_STRD 16
#define DOUT_C_STRD 8

typedef struct JZM_H264 {
	unsigned short start_mb_x;
	unsigned short start_mb_y;
	unsigned short mb_width;
	unsigned short mb_height;
	unsigned char slice_num;
	unsigned char slice_type;
	unsigned char qscale;                       /* s->qscale */
	unsigned char field_picture;
	unsigned char cabac;                        /* h->pps.cabac */
	unsigned char transform_8x8_mode;           /* !!h->pps.transform_8x8_mode */
	unsigned char constrained_intra_pred;       /* !!h->pps.constrained_intra_pred */
	unsigned char direct_8x8_inference_flag;    /* !!h->pps.direct_8x8_inference_flag */
	unsigned char direct_spatial_mv_pred;       /* !!h->direct_spatial_mv_pred */
	unsigned char ref_count_0;                  /* h->ref_count[0] */
	unsigned char ref_count_1;                  /* h->ref_count[1] */
	unsigned char deblocking_filter;            /* !!h->deblocking_filter */
	int dblk_left_en;
	int dblk_top_en;
	int x264_build;                   /* h->x264_build */
	int slice_alpha_c0_offset;
	int slice_beta_offset;
	unsigned long bs_buffer;           /* s->gb.buffer */
	unsigned int bs_index;            /* s->gb.index */
	unsigned int bs_size_in_bits;            /* s->gb.index */
	int cabac_init_idc;
	/*   int * curr_frm_slice_start_mb; */
	/*   int * ref_frm_slice_start_mb; */
	unsigned int ref_frm_ctrl;
	unsigned int ref_frm_mv;
	unsigned int curr_frm_ctrl;
	unsigned int curr_frm_mv;
	unsigned int dir_scale_table[16];
	unsigned int chroma_qp_table[128];
	unsigned char scaling_matrix8[6][64]; //2x64
	unsigned char scaling_matrix4[6][16];
	unsigned int tlb_phy_addr;
	unsigned int dec_result_y;
	unsigned int dec_result_uv;
	unsigned int mc_ref_y[2][16];
	unsigned int mc_ref_c[2][16];
	unsigned int luma_weight[2][16];
	unsigned int luma_offset[2][16];
	unsigned int chroma_weight[2][16][2];
	unsigned int chroma_offset[2][16][2];
	unsigned char implicit_weight[16][16];
	unsigned int use_weight;
	unsigned int use_weight_chroma;
	unsigned int luma_log2_weight_denom;
	unsigned int chroma_log2_weight_denom;
	int *des_va, * des_pa;
	unsigned char new_odma_flag;
	unsigned char new_odma_format; //0: nv12, 1: nv21.[m
	unsigned int dec_result1_y;
	unsigned int dec_result1_uv;
	unsigned int frm_y_stride;
	unsigned int frm_c_stride;
	unsigned int bs_rbsp_en;
} jzm_h264;

struct SDE_VLC_STA {
	int ram_ofst;
	int size;
	int lvl0_len;
};

extern void jzm_h264_slice_init_vdma(struct JZM_H264 *st_h264);

#endif // _JZM_H264_API_H_
