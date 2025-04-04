#ifndef H264_SET_H
#define H264_SET_H

#include "bitstream.h"

typedef struct {
	int i_id;

	int i_profile_idc;
	int i_level_idc;

	int b_constraint_set0;
	int b_constraint_set1;
	int b_constraint_set2;
	int b_constraint_set3;

	int i_log2_max_frame_num;

	int i_poc_type;
	/* poc 0 */
	int i_log2_max_poc_lsb;

	int i_num_ref_frames;
	int b_gaps_in_frame_num_value_allowed;
	int i_mb_width;
	int i_mb_height;
	int b_frame_mbs_only;
	int b_mb_adaptive_frame_field;
	int b_direct8x8_inference;

	int b_crop;
	struct {
		int i_left;
		int i_right;
		int i_top;
		int i_bottom;
	} crop;

	int b_vui;
	struct {
		int b_aspect_ratio_info_present;
		int i_sar_width;
		int i_sar_height;

		int b_overscan_info_present;
		int b_overscan_info;

		int b_signal_type_present;
		int i_vidformat;
		int b_fullrange;
		int b_color_description_present;
		int i_colorprim;
		int i_transfer;
		int i_colmatrix;

		int b_chroma_loc_info_present;
		int i_chroma_loc_top;
		int i_chroma_loc_bottom;

		int b_timing_info_present;
		uint32_t i_num_units_in_tick;
		uint32_t i_time_scale;
		int b_fixed_frame_rate;

		int b_nal_hrd_parameters_present;
		int b_vcl_hrd_parameters_present;

		struct {
			int i_cpb_cnt;
			int i_bit_rate_scale;
			int i_cpb_size_scale;
			int i_bit_rate_value;
			int i_cpb_size_value;
			int i_bit_rate_unscaled;
			int i_cpb_size_unscaled;
			int b_cbr_hrd;

			int i_initial_cpb_removal_delay_length;
			int i_cpb_removal_delay_length;
			int i_dpb_output_delay_length;
			int i_time_offset_length;
		} hrd;

		int b_pic_struct_present;
		int b_bitstream_restriction;
		int b_motion_vectors_over_pic_boundaries;
		int i_max_bytes_per_pic_denom;
		int i_max_bits_per_mb_denom;
		int i_log2_max_mv_length_horizontal;
		int i_log2_max_mv_length_vertical;
		int i_num_reorder_frames;
		int i_max_dec_frame_buffering;

		/* FIXME to complete */
	} vui;

	int b_qpprime_y_zero_transform_bypass;
	int i_chroma_format_idc;

	int b_avcintra;
	int i_cqm_preset;
	const uint8_t *scaling_list[8]; /* could be 12, but we don't allow separate Cb/Cr lists */

} h264_sps_t;

typedef struct {
	int i_id;
	int i_sps_id;

	int b_cabac;

	int b_pic_order;
	int i_num_slice_groups;

	int i_num_ref_idx_l0_default_active;
	int i_num_ref_idx_l1_default_active;

	int b_weighted_pred;
	int b_weighted_bipred;

	int i_pic_init_qp;
	int i_pic_init_qs;

	int i_chroma_qp_index_offset;

	int b_deblocking_filter_control;
	int b_constrained_intra_pred;
	int b_redundant_pic_cnt;

	int b_transform_8x8_mode;

} h264_pps_t;

extern void h264e_sps_write(bs_t *s, h264_sps_t *sps);

extern void h264e_pps_write(bs_t *s, h264_sps_t *sps, h264_pps_t *pps);

extern void h264e_sei_write(bs_t *s, uint8_t *payload, int payload_size, int payload_type);

extern int h264e_sei_version_write(bs_t *s);

#endif
