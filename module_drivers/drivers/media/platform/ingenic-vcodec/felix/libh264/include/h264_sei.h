#ifndef AVCODEC_H264_SEI_H
#define AVCODEC_H264_SEI_H

#include "get_bits.h"

/**
 * SEI message types
 */
typedef enum {
	H264_SEI_TYPE_BUFFERING_PERIOD       = 0,   ///< buffering period (H.264, D.1.1)
	H264_SEI_TYPE_PIC_TIMING             = 1,   ///< picture timing
	H264_SEI_TYPE_PAN_SCAN_RECT          = 2,   ///< pan-scan rectangle
	H264_SEI_TYPE_FILLER_PAYLOAD         = 3,   ///< filler data
	H264_SEI_TYPE_USER_DATA_REGISTERED   = 4,   ///< registered user data as specified by Rec. ITU-T T.35
	H264_SEI_TYPE_USER_DATA_UNREGISTERED = 5,   ///< unregistered user data
	H264_SEI_TYPE_RECOVERY_POINT         = 6,   ///< recovery point (frame # to decoder sync)
	H264_SEI_TYPE_FRAME_PACKING          = 45,  ///< frame packing arrangement
	H264_SEI_TYPE_DISPLAY_ORIENTATION    = 47,  ///< display orientation
	H264_SEI_TYPE_GREEN_METADATA         = 56,  ///< GreenMPEG information
	H264_SEI_TYPE_MASTERING_DISPLAY_COLOUR_VOLUME = 137,  ///< mastering display properties
	H264_SEI_TYPE_ALTERNATIVE_TRANSFER   = 147, ///< alternative transfer
} H264_SEI_Type;

/**
 * pic_struct in picture timing SEI message
 */
typedef enum {
	H264_SEI_PIC_STRUCT_FRAME             = 0, ///<  0: %frame
	H264_SEI_PIC_STRUCT_TOP_FIELD         = 1, ///<  1: top field
	H264_SEI_PIC_STRUCT_BOTTOM_FIELD      = 2, ///<  2: bottom field
	H264_SEI_PIC_STRUCT_TOP_BOTTOM        = 3, ///<  3: top field, bottom field, in that order
	H264_SEI_PIC_STRUCT_BOTTOM_TOP        = 4, ///<  4: bottom field, top field, in that order
	H264_SEI_PIC_STRUCT_TOP_BOTTOM_TOP    = 5, ///<  5: top field, bottom field, top field repeated, in that order
	H264_SEI_PIC_STRUCT_BOTTOM_TOP_BOTTOM = 6, ///<  6: bottom field, top field, bottom field repeated, in that order
	H264_SEI_PIC_STRUCT_FRAME_DOUBLING    = 7, ///<  7: %frame doubling
	H264_SEI_PIC_STRUCT_FRAME_TRIPLING    = 8  ///<  8: %frame tripling
} H264_SEI_PicStructType;

/**
 * frame_packing_arrangement types
 */
typedef enum {
	H264_SEI_FPA_TYPE_CHECKERBOARD        = 0,
	H264_SEI_FPA_TYPE_INTERLEAVE_COLUMN   = 1,
	H264_SEI_FPA_TYPE_INTERLEAVE_ROW      = 2,
	H264_SEI_FPA_TYPE_SIDE_BY_SIDE        = 3,
	H264_SEI_FPA_TYPE_TOP_BOTTOM          = 4,
	H264_SEI_FPA_TYPE_INTERLEAVE_TEMPORAL = 5,
	H264_SEI_FPA_TYPE_2D                  = 6,
} H264_SEI_FpaType;

typedef struct H264SEITimeCode {
	/* When not continuously receiving full timecodes, we have to reference
	   the previous timecode received */
	int full;
	int frame;
	int seconds;
	int minutes;
	int hours;
	int dropframe;
} H264SEITimeCode;

typedef struct H264SEIPictureTiming {
	int present;
	H264_SEI_PicStructType pic_struct;

	/**
	 * Bit set of clock types for fields/frames in picture timing SEI message.
	 * For each found ct_type, appropriate bit is set (e.g., bit 1 for
	 * interlaced).
	 */
	int ct_type;

	/**
	 * dpb_output_delay in picture timing SEI message, see H.264 C.2.2
	 */
	int dpb_output_delay;

	/**
	 * cpb_removal_delay in picture timing SEI message, see H.264 C.1.2
	 */
	int cpb_removal_delay;

	/**
	 * Maximum three timecodes in a pic_timing SEI.
	 */
	H264SEITimeCode timecode[3];

	/**
	 * Number of timecode in use
	 */
	int timecode_cnt;
} H264SEIPictureTiming;

typedef struct H264SEIAFD {
	int present;
	uint8_t active_format_description;
} H264SEIAFD;

typedef struct H264SEIA53Caption {
#if 0
	AVBufferRef *buf_ref;
#endif
} H264SEIA53Caption;

typedef struct H264SEIUnregistered {
	int x264_build;
} H264SEIUnregistered;

typedef struct H264SEIRecoveryPoint {
	/**
	 * recovery_frame_cnt
	 *
	 * Set to -1 if no recovery point SEI message found or to number of frames
	 * before playback synchronizes. Frames having recovery point are key
	 * frames.
	 */
	int recovery_frame_cnt;
} H264SEIRecoveryPoint;

typedef struct H264SEIBufferingPeriod {
	int present;   ///< Buffering period SEI flag
	int initial_cpb_removal_delay[32];  ///< Initial timestamps for CPBs
} H264SEIBufferingPeriod;

typedef struct H264SEIFramePacking {
	int present;
	int arrangement_id;
	int arrangement_cancel_flag;  ///< is previous arrangement canceled, -1 if never received
	H264_SEI_FpaType arrangement_type;
	int arrangement_repetition_period;
	int content_interpretation_type;
	int quincunx_sampling_flag;
	int current_frame_is_frame0_flag;
} H264SEIFramePacking;

typedef struct H264SEIDisplayOrientation {
	int present;
	int anticlockwise_rotation;
	int hflip, vflip;
} H264SEIDisplayOrientation;

typedef struct H264SEIGreenMetaData {
	uint8_t green_metadata_type;
	uint8_t period_type;
	uint16_t num_seconds;
	uint16_t num_pictures;
	uint8_t percent_non_zero_macroblocks;
	uint8_t percent_intra_coded_macroblocks;
	uint8_t percent_six_tap_filtering;
	uint8_t percent_alpha_point_deblocking_instance;
	uint8_t xsd_metric_type;
	uint16_t xsd_metric_value;
} H264SEIGreenMetaData;

typedef struct H264SEIAlternativeTransfer {
	int present;
	int preferred_transfer_characteristics;
} H264SEIAlternativeTransfer;

typedef struct H264SEIContext {
	H264SEIPictureTiming picture_timing;
	H264SEIAFD afd;
	H264SEIA53Caption a53_caption;
	H264SEIUnregistered unregistered;
	H264SEIRecoveryPoint recovery_point;
	H264SEIBufferingPeriod buffering_period;
	H264SEIFramePacking frame_packing;
	H264SEIDisplayOrientation display_orientation;
	H264SEIGreenMetaData green_metadata;
	H264SEIAlternativeTransfer alternative_transfer;
} H264SEIContext;

struct H264ParamSets;

int ff_h264_sei_decode(H264SEIContext *h, GetBitContext *gb,
                       const struct H264ParamSets *ps, void *logctx);

/**
 * Reset SEI values at the beginning of the frame.
 */
void ff_h264_sei_uninit(H264SEIContext *h);

/**
 * Get stereo_mode string from the h264 frame_packing_arrangement
 */
const char *ff_h264_sei_stereo_mode(const H264SEIFramePacking *h);

#endif /* AVCODEC_H264_SEI_H */
