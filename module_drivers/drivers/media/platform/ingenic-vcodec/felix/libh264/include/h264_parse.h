#ifndef AVCODEC_H264_PARSE_H
#define AVCODEC_H264_PARSE_H

#include "get_bits.h"
#include "h264_ps.h"

typedef struct H264PredWeightTable {
	int use_weight;
	int use_weight_chroma;
	int luma_log2_weight_denom;
	int chroma_log2_weight_denom;
	int luma_weight_flag[2];    ///< 7.4.3.2 luma_weight_lX_flag
	int chroma_weight_flag[2];  ///< 7.4.3.2 chroma_weight_lX_flag
	// The following 2 can be changed to int8_t but that causes a 10 CPU cycles speed loss
	int luma_weight[48][2][2];
	int chroma_weight[48][2][2][2];
	int implicit_weight[48][48][2];
} H264PredWeightTable;

typedef struct H264POCContext {
	int poc_lsb;
	int poc_msb;
	int delta_poc_bottom;
	int delta_poc[2];
	int frame_num;
	int prev_poc_msb;           ///< poc_msb of the last reference pic for POC type 0
	int prev_poc_lsb;           ///< poc_lsb of the last reference pic for POC type 0
	int frame_num_offset;       ///< for POC type 2
	int prev_frame_num_offset;  ///< for POC type 2
	int prev_frame_num;         ///< frame_num of the last pic for POC type 1/2
} H264POCContext;

int ff_h264_pred_weight_table(GetBitContext *gb, const SPS *sps,
                              const int *ref_count, int slice_type_nos,
                              H264PredWeightTable *pwt,
                              int picture_structure, void *logctx);

/**
 * Check if the top & left blocks are available if needed & change the
 * dc mode so it only uses the available blocks.
 */
int ff_h264_check_intra4x4_pred_mode(int8_t *pred_mode_cache, void *logctx,
                                     int top_samples_available, int left_samples_available);

/**
 * Check if the top & left blocks are available if needed & change the
 * dc mode so it only uses the available blocks.
 */
int ff_h264_check_intra_pred_mode(void *logctx, int top_samples_available,
                                  int left_samples_available,
                                  int mode, int is_chroma);

int ff_h264_parse_ref_count(int *plist_count, int ref_count[2],
                            GetBitContext *gb, const PPS *pps,
                            int slice_type_nos, int picture_structure, void *logctx);

int ff_h264_init_poc(int pic_field_poc[2], int *pic_poc,
                     const SPS *sps, H264POCContext *poc,
                     int picture_structure, int nal_ref_idc);

int ff_h264_decode_extradata(const uint8_t *data, int size, H264ParamSets *ps,
                             int *is_avc, int *nal_length_size,
                             int err_recognition, void *logctx);

/**
 * compute profile from sps
 */
int ff_h264_get_profile(const SPS *sps);

#endif /* AVCODEC_H264_PARSE_H */
