#ifndef __H264_NAL_H__
#define __H264_NAL_H__

/****************************************************************************
 * NAL structure and functions
 ****************************************************************************/

enum nal_unit_type_e {
	NAL_UNKNOWN     = 0,
	NAL_SLICE       = 1,
	NAL_SLICE_DPA   = 2,
	NAL_SLICE_DPB   = 3,
	NAL_SLICE_DPC   = 4,
	NAL_SLICE_IDR   = 5,    /* ref_idc != 0 */
	NAL_SEI         = 6,    /* ref_idc == 0 */
	NAL_SPS         = 7,
	NAL_PPS         = 8,
	NAL_AUD         = 9,
	NAL_FILLER      = 12,
	/* ref_idc == 0 for 6,9,10,11,12 */
};

enum nal_priority_e {
	NAL_PRIORITY_DISPOSABLE = 0,
	NAL_PRIORITY_LOW        = 1,
	NAL_PRIORITY_HIGH       = 2,
	NAL_PRIORITY_HIGHEST    = 3,
};

/* The data within the payload is already NAL-encapsulated; the ref_idc and type
 * are merely in the struct for easy access by the calling application.
 * All data returned in an h264_nal_t, including the data in p_payload, is no longer
 * valid after the next call to h264_encoder_encode.  Thus it must be used or copied
 * before calling h264_encoder_encode or h264_encoder_headers again. */
typedef struct h264_nal_t {
	int i_ref_idc;  /* nal_priority_e */
	int i_type;     /* nal_unit_type_e */
	int b_long_startcode;
	int i_first_mb; /* If this NAL is a slice, the index of the first MB in the slice. */
	int i_last_mb;  /* If this NAL is a slice, the index of the last MB in the slice. */

	/* Size of payload (including any padding) in bytes. */
	int     i_payload;
	/* If param->b_annexb is set, Annex-B bytestream with startcode.
	 * Otherwise, startcode is replaced with a 4-byte size.
	 * This size is the size used in mp4/similar muxing; it is equal to i_payload-4 */
	uint8_t *p_payload;

	/* Size of padding in bytes. */
	int i_padding;
} h264_nal_t;

#endif
