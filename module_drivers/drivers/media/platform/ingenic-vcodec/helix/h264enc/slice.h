#ifndef __H264_SLICE_H__
#define __H264_SLICE_H__

#include "common.h"

enum slice_type_e {
	M_SLICE_TYPE_P  = 0,
	M_SLICE_TYPE_B  = 1,
	M_SLICE_TYPE_I  = 2,
};

#define MAX(X, Y) ((X) >= (Y) ? (X) : (Y))
#define MIN(X, Y) ((X) <= (Y) ? (X) : (Y))

#define QP_MAX_SPEC             (51)

#define X264_REF_MAX    1
#define SPEC_QP(x)              MIN((x), QP_MAX_SPEC)

typedef struct {
	h264_sps_t *sps;
	h264_pps_t *pps;

	int i_type;
	int i_first_mb;
	int i_last_mb;

	int i_pps_id;

	int i_frame_num;

	int b_mbaff;
	int b_field_pic;
	int b_bottom_field;

	int i_idr_pic_id;   /* -1 if nal_type != 5 */

	int i_poc;
	int i_delta_poc_bottom;

	int i_delta_poc[2];
	int i_redundant_pic_cnt;

	int b_direct_spatial_mv_pred;

	int b_num_ref_idx_override;
	int i_num_ref_idx_l0_active;
	int i_num_ref_idx_l1_active;

	int b_ref_pic_list_reordering[2];
	struct {
		int idc;
		int arg;
	} ref_pic_list_order[2][X264_REF_MAX];

#if 0
	/* 使用默认的加权预测 */
	/* P-frame weighting */
	int b_weighted_pred;
	x264_weight_t weight[X264_REF_MAX * 2][3];
#endif
	int i_mmco_remove_from_end;
	int i_mmco_command_count;
	struct { /* struct for future expansion */
		int i_difference_of_pic_nums;
		int i_poc;
	} mmco[X264_REF_MAX];

	int i_cabac_init_idc;

	int i_qp;
	int i_qp_delta;
	int b_sp_for_swidth;
	int i_qs_delta;

	/* deblocking filter */
	int i_disable_deblocking_filter_idc;
	int i_alpha_c0_offset;
	int i_beta_offset;
} h264_slice_header_t;

extern void h264e_slice_header_init(h264_slice_header_t *sh,
                                    h264_sps_t *sps, h264_pps_t *pps,
                                    int i_idr_pic_id, int i_frame, int i_qp);

extern void h264e_slice_header_write(bs_t *s, h264_slice_header_t *sh, int i_nal_ref_idc);

#endif
