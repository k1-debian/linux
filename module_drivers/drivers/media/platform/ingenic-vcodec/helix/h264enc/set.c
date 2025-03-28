#ifdef __KERNEL__
	#include <linux/printk.h>
#else

	#define printk printf
#endif
#include "common.h"

void dump_sps(h264_sps_t *sps)
{
	printk("---sps->i_profile_idc %d\n", sps->i_profile_idc);
	printk("---sps->b_constraint_set0 %d\n", sps->b_constraint_set0);
	printk("---sps->b_constraint_set1 %d\n", sps->b_constraint_set1);
	printk("---sps->b_constraint_set2 %d\n", sps->b_constraint_set2);
	printk("---sps->b_constraint_set3 %d\n", sps->b_constraint_set3);

	printk("---sps->i_level_idc %d\n", sps->i_level_idc);
	printk("---sps->i_id %d\n", sps->i_id);
	printk("---sps->i_num_ref_frames %d\n", sps->i_num_ref_frames);
	printk("---sps->i_mb_width %d\n", sps->i_mb_width);
	printk("---sps->i_mb_height %d\n", sps->i_mb_height);
}

void dump_pps(h264_pps_t *pps)
{
	printk("---pps->i_sps_id %d\n", pps->i_sps_id);
	printk("---pps->b_cabac %d\n", pps->b_cabac);
	printk("---pps->i_num_slice_groups %d\n", pps->i_num_slice_groups);
}

void h264e_sps_write(bs_t *s, h264_sps_t *sps)
{
	bs_realign(s);
	bs_write(s, 8, sps->i_profile_idc);

	bs_write1(s, sps->b_constraint_set0);
	bs_write1(s, sps->b_constraint_set1);
	bs_write1(s, sps->b_constraint_set2);
	bs_write1(s, sps->b_constraint_set3);

	bs_write(s, 4, 0); //reserved_zero_4bits; /* equal to 0*/
	bs_write(s, 8, sps->i_level_idc);
	bs_write_ue(s, sps->i_id);

	bs_write_ue(s, sps->i_log2_max_frame_num - 4); // log2_max_frame_num_minus4;
	bs_write_ue(s, sps->i_poc_type); // pic_order_cnt_type;
	if (sps->i_poc_type == 0) {
		bs_write_ue(s, sps->i_log2_max_poc_lsb - 4);
	}

	bs_write_ue(s, sps->i_num_ref_frames);
	bs_write1(s, sps->b_gaps_in_frame_num_value_allowed);
	bs_write_ue(s, sps->i_mb_width - 1);   /*pic_width_in_mbs_minus1*/
	bs_write_ue(s, (sps->i_mb_height >> !sps->b_frame_mbs_only) - 1);
	bs_write1(s, sps->b_frame_mbs_only);

	if (!sps->b_frame_mbs_only) {
		bs_write1(s, sps->b_mb_adaptive_frame_field);
	}
	bs_write1(s, sps->b_direct8x8_inference);

	bs_write1(s, sps->b_crop);
	if (sps->b_crop) {
		int h_shift = sps->i_chroma_format_idc == CHROMA_420 || sps->i_chroma_format_idc == CHROMA_422;
		int v_shift = sps->i_chroma_format_idc == CHROMA_420;
		bs_write_ue(s, sps->crop.i_left   >> h_shift);
		bs_write_ue(s, sps->crop.i_right  >> h_shift);
		bs_write_ue(s, sps->crop.i_top    >> v_shift);
		bs_write_ue(s, sps->crop.i_bottom >> v_shift);
	}

	bs_write1(s, sps->b_vui);
	if (sps->b_vui) {
	}

	bs_rbsp_trailing(s);
	bs_flush(s);
}

void h264e_pps_write(bs_t *s, h264_sps_t *sps, h264_pps_t *pps)
{
	bs_realign(s);
	bs_write_ue(s, pps->i_id);
	bs_write_ue(s, pps->i_sps_id);

	bs_write1(s, pps->b_cabac);
	bs_write1(s, pps->b_pic_order);
	bs_write_ue(s, pps->i_num_slice_groups - 1);

	bs_write_ue(s, pps->i_num_ref_idx_l0_default_active - 1);
	bs_write_ue(s, pps->i_num_ref_idx_l1_default_active - 1);
	bs_write1(s, pps->b_weighted_pred);
	bs_write(s, 2, pps->b_weighted_bipred);

	bs_write_se(s, pps->i_pic_init_qp - 26);
	bs_write_se(s, pps->i_pic_init_qs - 26);
	bs_write_se(s, pps->i_chroma_qp_index_offset);

	bs_write1(s, pps->b_deblocking_filter_control);
	bs_write1(s, pps->b_constrained_intra_pred);
	bs_write1(s, pps->b_redundant_pic_cnt);

	if (pps->b_transform_8x8_mode) {

		bs_write1(s, pps->b_transform_8x8_mode);
		bs_write1(s, 0);

		bs_write_se(s, pps->i_chroma_qp_index_offset);

	}

	bs_rbsp_trailing(s);
	bs_flush(s);
}

void h264e_sei_write(bs_t *s, uint8_t *payload, int payload_size, int payload_type)
{
	int i;
	bs_realign(s);
	for (i = 0; i <= payload_type - 255; i += 255) {
		bs_write(s, 8, 255);
	}
	bs_write(s, 8, payload_type - i);

	for (i = 0; i <= payload_size - 255; i += 255) {
		bs_write(s, 8, 255);
	}
	bs_write(s, 8, payload_size - i);

	for (i = 0; i < payload_size; i++) {
		bs_write(s, 8, payload[i]);
	}

	bs_rbsp_trailing(s);
	bs_flush(s);
}

int h264e_sei_version_write(bs_t *s)
{

	return 0;
}
