/****************************************************************
*****************************************************************/

#ifndef __JZM_X264_ENC_H__
#define __JZM_X264_ENC_H__
#include "helix.h"

#ifdef __KERNEL__
	#include <linux/types.h>
	#include <linux/printk.h>
	#include <asm-generic/div64.h>

#else
	#include <stdint.h>
	#include <stdio.h>

#endif

#define SCH_FIFO_DEPTH      16

/************************************************************
 CHN Space Allocation
 ************************************************************/
#define VRAM_DUMMY          (VPU_BASE | 0xFFFFC)

#define VRAM_MAU_RESA       (VPU_BASE | 0xC0000)  //residual address
#define VRAM_MAU_RES_SIZE   (SCH_FIFO_DEPTH*256*4)

#define VRAM_RAWY_BA        (VPU_BASE | 0xF0000)
#define VRAM_RAWC_BA        (VRAM_RAWY_BA + 256)
#define VRAM_RAW_SIZE       (SCH_FIFO_DEPTH*128*4)

#define VRAM_TOPPA_BA       (VRAM_MAU_RESA+VRAM_MAU_RES_SIZE)  //recover address
#define VRAM_TOPMV_BA       (VRAM_TOPPA_BA+VPU_MAX_MB_WIDTH*8)

#define VRAM_MAU_CHN_BASE   (VRAM_TOPMV_BA+VPU_MAX_MB_WIDTH*4)
#define VRAM_MAU_CHN_SIZE   (SCH_FIFO_DEPTH*16*4)
#define VRAM_DBLK_CHN_BASE  (VRAM_MAU_CHN_BASE + VRAM_MAU_CHN_SIZE)
#define VRAM_DBLK_CHN_SIZE  (SCH_FIFO_DEPTH*16*4)
#define VRAM_ME_CHN_BASE    (VRAM_DBLK_CHN_BASE + VRAM_DBLK_CHN_SIZE)
#define VRAM_ME_CHN_SIZE    (SCH_FIFO_DEPTH*8*4)
#define VRAM_SDE_CHN_BASE   (VRAM_ME_CHN_BASE + VRAM_ME_CHN_SIZE)
#define VRAM_SDE_CHN_SIZE   (SCH_FIFO_DEPTH*8*4)
#define VRAM_QP_TAB_BA      (VRAM_SDE_CHN_BASE + VRAM_SDE_CHN_SIZE)

#define VRAM_ME_DSA         DSA_SCH_CH1
#define VRAM_MAU_DEC_SYNA   DSA_SCH_CH2
#define VRAM_DBLK_CHN_SYNA  DSA_SCH_CH3
#define VRAM_SDE_SYNA       DSA_SCH_CH4

#define VRAM_MAU_ENC_SYNA   VRAM_DUMMY
#define VRAM_DBLK_DOUT_SYNA VRAM_DUMMY

#define VRAM_ME_MVPA        (VPU_BASE | REG_EFE_MVRP)

#define EMC_SIZE        (1 << 21)
#define DBLK_SIZE       (1 << 20)
#define RECON_SIZE      (1 << 18)
#define MV_SIZE         (1 << 11)
#define SE_SIZE         (1 << 11)
#define QPT_SIZE        (1<<14)
#define RC_SIZE         (1<<15)
#define CPX_SIZE        (1<<17)
#define MOD_SIZE        (1<<15)
#define SAD_SIZE        (1<<17)
#define ENCU_SIZE        (1<<13)
#define BEYOND_MAX_SIZE 64*3

typedef unsigned int paddr_t;

#define __ALN32__ __attribute__ ((aligned(4)))

typedef struct ROI_info {
	uint8_t roi_en;
	uint8_t roi_md;
	int8_t  roi_qp;
	uint8_t roi_lmbx;
	uint8_t roi_rmbx;
	uint8_t roi_umby;
	uint8_t roi_bmby;
} roi_info_t;

/*
  _H264E_SliceInfo:
  H264 Encoder Slice Level Information
 */
typedef struct _H264E_SliceInfo {
	/*basic*/
	uint8_t frame_type;
	uint8_t mb_width;
	uint8_t mb_height;
	uint8_t first_mby;
	uint8_t last_mby;  //for multi-slice

	/* motion */
	int frame_width;
	int frame_height;
	uint8_t frm_re[4];
	uint8_t pskip_en;
	uint8_t force_mv0_en;
	uint8_t mref_en;
	uint8_t dct8x8_en;
	uint8_t scl;
	uint8_t hpel_en;
	uint8_t qpel_en;
	uint8_t ref_mode;
	uint32_t max_sech_step_i;
	uint32_t max_mvrx_i;
	uint32_t max_mvry_i;
	uint8_t lambda_scale_parameter;
	uint8_t fs_en; //fs function enable
	uint32_t fs_md; //fs step mode, 0: 1, 1: 3
	uint8_t fs_px; //fs period x
	uint8_t fs_py; //fs period y
	uint8_t fs_rx; //fs range x, must be multiples of 3
	uint8_t fs_ry; //fs range y, must be multiples of 3
	uint8_t frm_mv_en; //add a frame level mv
	uint8_t frm_mv_size; //mv enable after x mb, x=2^(size+8)
	uint8_t glb_mv_en;  //global mv enable
	int glb_mvx;  //global mvx value
	int glb_mvy;  //global mvy value
	uint8_t me_step_en; //auto-modify max step number
	uint8_t me_step_0; //step number threshold 0
	uint8_t me_step_1; //step number threshold 1
	/*vmau scaling list*/
	uint8_t __ALN32__ scaling_list[4][16];
	uint8_t __ALN32__ scaling_list8[2][64];
	int deadzone[9];
	uint32_t acmask_mode;

	uint32_t intra_mode_msk;

	uint8_t i_4x4_dis;
	uint8_t i_8x8_dis;
	uint8_t i_16x16_dis;
	uint8_t p_l0_dis;
	uint8_t p_t8_dis;
	uint8_t p_skip_dis;
	uint8_t p_skip_pl0f_dis;
	uint8_t p_skip_pt8f_dis;

	uint8_t cost_bias_en;
	uint8_t cost_bias_i_4x4;
	uint8_t cost_bias_i_8x8;
	uint8_t cost_bias_i_16x16;
	uint8_t cost_bias_p_l0;
	uint8_t cost_bias_p_t8;
	uint8_t cost_bias_p_skip;

	uint8_t intra_lambda_y_bias_en;
	uint8_t intra_lambda_c_bias_en;
	uint8_t intra_lambda_bias_qp0;
	uint8_t intra_lambda_bias_qp1;
	uint8_t intra_lambda_bias_0;
	uint8_t intra_lambda_bias_1;
	uint8_t intra_lambda_bias_2;

	uint8_t chroma_sse_bias_en;
	uint8_t chroma_sse_bias_qp0;
	uint8_t chroma_sse_bias_qp1;
	uint8_t chroma_sse_bias_0;
	uint8_t chroma_sse_bias_1;
	uint8_t chroma_sse_bias_2;

	uint8_t sse_lambda_bias_en;
	uint8_t sse_lambda_bias;

	uint8_t fbc_ep;
	uint8_t jm_lambda2_en;
	uint8_t inter_nei_en;
	uint8_t skip_bias_en;

	uint8_t info_en;
	uint8_t mvd_sum_all;
	uint8_t mvd_sum_abs;
	uint8_t mv_sum_all;
	uint8_t mv_sum_abs;

	uint32_t ysse_thr;
	uint32_t csse_thr;

	uint8_t  cfg_size_x;
	uint8_t  cfg_size_y;
	uint16_t cfg_iw_thr;

	uint16_t cfg_mvr_thr1;
	uint32_t cfg_mvr_thr2;
	uint32_t cfg_mvr_thr3;

	uint8_t  dcm_en;
	uint32_t dcm_param;

	uint8_t  sde_prior;
	uint8_t  db_prior;
	/*ipred bit&lambda ctrl*/
	uint8_t  mb_mode_val;
	uint8_t  bit_16_en;
	uint8_t  bit_8_en;
	uint8_t  bit_4_en;
	uint8_t  bit_uv_en;
	uint8_t  lamb_16_en;
	uint8_t  lamb_8_en;
	uint8_t  lamb_4_en;
	uint8_t  lamb_uv_en;
	uint8_t  c_16_en;
	uint8_t  c_8_en;
	uint8_t  c_4_en;
	uint8_t  c_uv_en;
	uint8_t  pri_16;
	uint8_t  pri_8;
	uint8_t  pri_4;
	uint8_t  pri_uv;
	uint8_t  ref_neb_4;
	uint8_t  ref_neb_8;
	uint8_t  bit_16[4];
	uint8_t  bit_uv[4];
	uint8_t  bit_4[4];
	uint8_t  bit_8[4];
	uint8_t  lambda_info16;
	uint8_t  lambda_info8;
	uint8_t  lambda_info4;
	uint8_t  lambda_infouv;
	uint8_t  ref_4;
	uint8_t  ref_8;
	uint8_t  const_16[4];
	uint8_t  const_uv[4];
	uint8_t  const_4[4];
	uint8_t  const_8[4];

	/*loop filter*/
	uint8_t deblock;
	uint8_t rotate;
	int8_t alpha_c0_offset;
	int8_t beta_offset;

	/*cabac*/
	uint8_t *state;
	paddr_t bs;             /*BS output address*/
	uint8_t bs_rbsp_en;     /*BS rbsp enable*/
	uint8_t bs_head_en;     /*read BS header enable*/
	uint8_t bs_head_len;    /*read BS header lenth*/
	paddr_t *bs_head_va;     /*read BS header data addr*/
	paddr_t bs_head_pa;     /*read BS header data addr*/
	uint8_t qp;
	uint8_t  bs_size_en;
	uint32_t bs_size;

	uint8_t skip_en;
	int8_t cqp_offset;
	/*mode decision*/
	uint8_t use_intra_in_pframe;
	uint8_t use_fast_mvp;

	uint8_t mode_ctrl;
	uint32_t mode_ctrl_param[40];

	/*frame buffer address: all of the buffers should be 256byte aligned!*/
	paddr_t fb[3][2];       /*{curr, ref}{tile_y, tile_c}*/
	paddr_t raw[3];         /*{rawy, rawu, rawv} or {rawy, rawc, N/C}*/
	int stride[2];          /*{stride_y, stride_c}, only used in raster raw*/

	/* RAW plane format */
	uint8_t raw_format;

	uint8_t size_mode;
	uint8_t step_mode;
	/*descriptor address*/
	paddr_t *des_va, des_pa;
	paddr_t emc_bs_pa;
	paddr_t *emc_dblk_va, emc_dblk_pa;
	paddr_t *emc_recon_va, emc_recon_pa;
	paddr_t *emc_mv_va, emc_mv_pa;
	paddr_t *emc_se_va, emc_se_pa;
	paddr_t *emc_qpt_va, emc_qpt_pa;
	paddr_t *emc_rc_va, emc_rc_pa;
	paddr_t *emc_cpx_va, emc_cpx_pa;
	paddr_t *emc_mod_va, emc_mod_pa;
	paddr_t *emc_ncu_va, emc_ncu_pa;
	paddr_t *emc_sad_va, emc_sad_pa;

	/*TLB address*/
	paddr_t tlba;

	/* ROI info */
	roi_info_t roi_info[8];
	uint8_t base_qp;
	uint8_t max_qp;
	uint8_t min_qp;

	uint8_t qp_tab_mode;
	uint8_t qp_tab_en;
	uint32_t qp_tab_len;
	uint32_t *qp_tab;
	uint8_t sas_en;
	uint8_t crp_en;
	uint8_t sas_mthd;
	uint16_t qpg_mb_thd[7];
	int8_t qpg_mbqp_ofst[8];
	uint8_t qpg_flt_thd[5];
	uint8_t mbrc_qpg_sel; //whether use crp/sas qp offset or not, when MB rate control enable.

	/* rate ctrl */
	uint8_t rc_mb_en;
	uint8_t rc_bu_wait_en;// efe chn wait rc qp offset en
	uint8_t rc_mb_wait_en;// efe chn wait rc qp offset en
	uint8_t rc_bu_num;// total basic unit number
	uint16_t rc_bu_size;// mb number in a basic unit
	uint8_t rc_bu_level;// 1:1line 2:2line 3:4line 4:8line 5:16line 6:32line 7:64line
	uint8_t rc_mb_level;// 0:skip 1mb, 1:skip 2mb, 2:skip 4mb, 3:skip 8mb
	int32_t *mb_ref_info;
	int32_t *bu_ref_info;
	uint32_t rc_frm_tbs;
	uint32_t avg_bu_bs;
	uint16_t tar_bs_thd[6];
	int8_t bu_alg0_qpo[6];
	int8_t bu_alg1_qpo[6];
	int8_t mb_cs_qpo[7];
	int8_t mb_top_bs_qpo[2];
	int8_t mb_rinfo_qpo[2];
	uint16_t mb_target_avg_bs[2];
	uint16_t mb_gp_num;
	uint16_t last_bu_size;
	uint8_t rc_bcfg_mode;

	//mosaic
	uint8_t mosaic_en;
	uint8_t mos_gthd[2];
	uint8_t mos_sthd[2];
	/* VPU Daisy Chain setting */
	uint8_t daisy_chain_en;
	uint8_t curr_thread_id;

	//odma,jrfc,jrfd
	uint8_t jrfcd_flag;
	uint8_t jrfc_enable;
	uint8_t jrfd_enable;
	uint32_t lm_head_total;
	uint32_t cm_head_total;
	paddr_t jh[3][2];       /*head addr {curr, ref0, ref1}{y, c}*/
	paddr_t spe_y_addr;
	paddr_t spe_c_addr;

	//eigen cfg
	uint8_t     mb_mode_use;
	uint32_t    *mb_mode_info;
	uint8_t     force_i16dc;//ipred
	uint8_t     force_i16;
	uint8_t     refresh_en;
	uint8_t     refresh_mode;
	uint8_t     refresh_bias;
	uint8_t     refresh_cplx_thd;
	uint8_t     cplx_thd_sel;
	uint8_t     diff_cplx_sel;
	uint8_t     diff_thd_sel;
	uint8_t     i16dc_cplx_thd;
	uint8_t     i16dc_qp_base;
	uint8_t     i16dc_qp_sel;
	uint8_t     i16_qp_base;
	uint8_t     i16_qp_sel;
	uint8_t     diff_cplx_thd;
	uint8_t     diff_qp_base[2];
	uint8_t     diff_qp_sel[2];
	uint8_t     cplx_thd_idx[24];
	uint8_t     cplx_thd[8];
	uint16_t    diff_thd_base[3];
	uint16_t    diff_thd_ofst[3];
	uint8_t     sas_eigen_en;
	uint8_t     crp_eigen_en;
	uint8_t     sas_eigen_dump;
	uint8_t     crp_eigen_dump;
	//ifa
	uint8_t     rrs_en;
	uint8_t     rrs_dump_en;
	uint8_t     rrs_uv_en;
	uint8_t     rrs_size_y; //0: 4, 1: 8, 2: 12, 3: 16
	uint8_t     rrs_size_c; //0: 4, 1: 8
	uint16_t    rrs_thrd_y; //threshold
	uint16_t    rrs_thrd_u;
	uint16_t    rrs_thrd_v;

	//skin
	uint8_t     skin_dt_en;
	uint8_t     skin_lvl;
	uint8_t     skin_cnt_thd;
	uint8_t     skin_pxlu_thd[3][2];
	uint8_t     skin_pxlv_thd[3][2];
	int8_t      skin_qp_ofst[4];
	uint8_t     mult_factor[3];
	uint8_t     shift_factor[3][3];//[0][]:0.4, [1][]:0.6, [2][]: 5.1
	uint16_t    skin_ofst[4];//[0]: 1.5, [1]:0.4, [2]:0.6, [3]: 5.1
	uint8_t     ncu_mov_en;
	uint32_t    ncu_move_len;
	uint32_t    *ncu_move_info;
	//buf-share
	uint8_t     buf_share_en;
	uint8_t     buf_share_size;
	uint32_t    frame_idx;
	uint8_t     is_first_Pframe;
} _H264E_SliceInfo;

typedef struct _H264E_SliceInfo H264E_SliceInfo_t;

/*
  H264E_SliceInit(_H264E_SliceInfo *s)
  @param s: slice information structure
 */
void H264E_SliceInit(_H264E_SliceInfo *s);
void H264E_DumpInfo(_H264E_SliceInfo *s);

/*
    default _H264_SliceInfo.
*/
//extern _H264E_SliceInfo default_H264E_Sliceinfo;

#endif /*__JZM_H264E_H__*/
