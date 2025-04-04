#ifndef AVCODEC_H264_PS_H
#define AVCODEC_H264_PS_H

//#include "libavutil/buffer.h"
//#include "libavutil/pixfmt.h"
//#include "libavutil/rational.h"

#include "avcodec.h"
#include "get_bits.h"
#include "h264.h"

#if 1
	#define MAX_SPS_COUNT          32
	#define MAX_PPS_COUNT         256
#else
	#define MAX_SPS_COUNT          3
	#define MAX_PPS_COUNT         4
#endif

#define MAX_LOG2_MAX_FRAME_NUM    (12 + 4)

typedef struct AVRational {
	int num; ///< Numerator
	int den; ///< Denominator
} AVRational;

static const AVRational ff_h264_pixel_aspect[17] = {
	{   0,  1 },
	{   1,  1 },
	{  12, 11 },
	{  10, 11 },
	{  16, 11 },
	{  40, 33 },
	{  24, 11 },
	{  20, 11 },
	{  32, 11 },
	{  80, 33 },
	{  18, 11 },
	{  15, 11 },
	{  64, 33 },
	{ 160, 99 },
	{   4,  3 },
	{   3,  2 },
	{   2,  1 },
};
/**
 * Sequence parameter set
 */
typedef struct SPS {
	unsigned int sps_id;
	int profile_idc;
	int level_idc;
	int chroma_format_idc;
	int transform_bypass;              ///< qpprime_y_zero_transform_bypass_flag
	int log2_max_frame_num;            ///< log2_max_frame_num_minus4 + 4
	int poc_type;                      ///< pic_order_cnt_type
	int log2_max_poc_lsb;              ///< log2_max_pic_order_cnt_lsb_minus4
	int delta_pic_order_always_zero_flag;
	int offset_for_non_ref_pic;
	int offset_for_top_to_bottom_field;
	int poc_cycle_length;              ///< num_ref_frames_in_pic_order_cnt_cycle
	int ref_frame_count;               ///< num_ref_frames
	int gaps_in_frame_num_allowed_flag;
	int mb_width;                      ///< pic_width_in_mbs_minus1 + 1
	///< (pic_height_in_map_units_minus1 + 1) * (2 - frame_mbs_only_flag)
	int mb_height;
	int frame_mbs_only_flag;
	int mb_aff;                        ///< mb_adaptive_frame_field_flag
	int direct_8x8_inference_flag;
	int crop;                          ///< frame_cropping_flag

	/* those 4 are already in luma samples */
	unsigned int crop_left;            ///< frame_cropping_rect_left_offset
	unsigned int crop_right;           ///< frame_cropping_rect_right_offset
	unsigned int crop_top;             ///< frame_cropping_rect_top_offset
	unsigned int crop_bottom;          ///< frame_cropping_rect_bottom_offset
	int vui_parameters_present_flag;
	AVRational sar;
	int video_signal_type_present_flag;
	int full_range;
	int colour_description_present_flag;
#if 0
	enum AVColorPrimaries color_primaries;
	enum AVColorTransferCharacteristic color_trc;
	enum AVColorSpace colorspace;
#endif
	int timing_info_present_flag;
	uint32_t num_units_in_tick;
	uint32_t time_scale;
	int fixed_frame_rate_flag;
	//short offset_for_ref_frame[256]; // FIXME dyn aloc?
	short offset_for_ref_frame[128]; // FIXME dyn aloc?
	int bitstream_restriction_flag;
	int num_reorder_frames;
	int scaling_matrix_present;
	uint8_t scaling_matrix4[6][16];
	uint8_t scaling_matrix8[6][64];
	int nal_hrd_parameters_present_flag;
	int vcl_hrd_parameters_present_flag;
	int pic_struct_present_flag;
	int time_offset_length;
	int cpb_cnt;                          ///< See H.264 E.1.2
	int initial_cpb_removal_delay_length; ///< initial_cpb_removal_delay_length_minus1 + 1
	int cpb_removal_delay_length;         ///< cpb_removal_delay_length_minus1 + 1
	int dpb_output_delay_length;          ///< dpb_output_delay_length_minus1 + 1
	int bit_depth_luma;                   ///< bit_depth_luma_minus8 + 8
	int bit_depth_chroma;                 ///< bit_depth_chroma_minus8 + 8
	int residual_color_transform_flag;    ///< residual_colour_transform_flag
	int constraint_set_flags;             ///< constraint_set[0-3]_flag
#if 0
	uint8_t data[4096];
	size_t data_size;
#endif
} SPS;

/**
 * Picture parameter set
 */
typedef struct PPS {
	unsigned int sps_id;
	int cabac;                  ///< entropy_coding_mode_flag
	int pic_order_present;      ///< pic_order_present_flag
	int slice_group_count;      ///< num_slice_groups_minus1 + 1
	int mb_slice_group_map_type;
	unsigned int ref_count[2];  ///< num_ref_idx_l0/1_active_minus1 + 1
	int weighted_pred;          ///< weighted_pred_flag
	int weighted_bipred_idc;
	int init_qp;                ///< pic_init_qp_minus26 + 26
	int init_qs;                ///< pic_init_qs_minus26 + 26
	int chroma_qp_index_offset[2];
	int deblocking_filter_parameters_present; ///< deblocking_filter_parameters_present_flag
	int constrained_intra_pred;     ///< constrained_intra_pred_flag
	int redundant_pic_cnt_present;  ///< redundant_pic_cnt_present_flag
	int transform_8x8_mode;         ///< transform_8x8_mode_flag
	uint8_t scaling_matrix4[6][16];
	uint8_t scaling_matrix8[6][64];
	uint8_t chroma_qp_table[2][QP_MAX_NUM + 1]; ///< pre-scaled (with chroma_qp_index_offset) version of qp_table
	int chroma_qp_diff;
#if 0
	uint8_t data[4096];
	size_t data_size;
#endif

#if 0
	uint32_t dequant4_buffer[6][QP_MAX_NUM + 1][16];
	uint32_t dequant8_buffer[6][QP_MAX_NUM + 1][64];
	uint32_t(*dequant4_coeff[6])[16];
	uint32_t(*dequant8_coeff[6])[64];
#endif
} PPS;

typedef struct H264ParamSets {
#if 0
	AVBufferRef *sps_list[MAX_SPS_COUNT];
	AVBufferRef *pps_list[MAX_PPS_COUNT];

	AVBufferRef *pps_ref;
	AVBufferRef *sps_ref;
#endif

	SPS sps_list[MAX_SPS_COUNT];
	PPS pps_list[MAX_PPS_COUNT];
	/* currently active parameters sets */
	const PPS *pps;
	const SPS *sps;
} H264ParamSets;

/**
 * Decode SPS
 */
int ff_h264_decode_seq_parameter_set(GetBitContext *gb, AVCodecContext *avctx,
                                     H264ParamSets *ps, int ignore_truncation);

/**
 * Decode PPS
 */
int ff_h264_decode_picture_parameter_set(GetBitContext *gb, AVCodecContext *avctx,
        H264ParamSets *ps, int bit_length);

/**
 * Uninit H264 param sets structure.
 */
void ff_h264_ps_uninit(H264ParamSets *ps);

#endif /* AVCODEC_H264_PS_H */
