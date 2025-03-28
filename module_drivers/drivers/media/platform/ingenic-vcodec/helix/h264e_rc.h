#ifndef __H264E_H__
#define __H264E_H__

#include "h264enc/common.h"
#include "h264enc/set.h"

#ifdef CONFIG_INGENIC_HELIX_H264E_RC_NETLINK
	#include "api/helix_x264_enc.h"
#endif

#ifdef CONFIG_INGENIC_HELIX_H264E_RC_IN_KERNEL
	#include "h264e-rc/librc/jzm_enc_api.h"
#endif

#include "helix_buf.h"
#include "h264e_rc_proto.h"

#define MAX_BS_HEADER_SIZE  256 /* maybe enough for header only.*/
#define MAX_SLICE_HEADER_SIZE   128 /* 128bytes of slice header bs.*/

struct h264e_ctx {
	unsigned char bs_header[MAX_BS_HEADER_SIZE]; /* buffer to store sps/pps/sei headers. */
	unsigned int bs_header_size;    /* buffer size. */

	unsigned char slice_header[128]; /*store slice buffer.*/

	int vdma_chain_len;
	unsigned int framesize;
	/*根据height,width分配的,*/
	struct  h264_ref_tile fb[3]; /*参考body buffer.按frame申请*/
	struct  h264_ref_tile jh[3]; /*参考header buffer.按frame申请*/

	unsigned int *emc_buf;      /* 128KBytes固定空间*/
	dma_addr_t emc_buf_pa;

	unsigned int *desc;     /* descriptor, 40960+256, 固定大小*/
	dma_addr_t desc_pa;

	struct video_frame_buffer *frame; /* input picture frame. */
	struct ingenic_vcodec_mem *bs; /* output slice data. exclude slice header? */
	struct ingenic_vcodec_mem bs_tmp_buf; /* temperary buf for now., used to store vpu output buffer. then memory cp to userspace. */

	unsigned int src_crc;   /*Debug*/

	h264_sps_t sps;
	h264_pps_t pps;
	h264_slice_header_t sh;
	h264_cabac_t cb;    /*cabac state*/

	int i_frame;    /* (i_frame + 1) % gop_size */

	/* modify after encode frame*/
	unsigned long frameindex;
	unsigned int status;
	unsigned int r_bs_len;      /*result bs len.*/
	unsigned int r_rbsp_len;    /*0x03 的个数.*/
	unsigned int encoded_bs_len;    /* encoded bs_len */
#ifdef CONFIG_INGENIC_HELIX_H264E_RC_IN_KERNEL
	VPU_RC_S *rc;   // rc Instance.
#endif
	/* 准备删除. */
	struct h264e_params p;

#ifdef CONFIG_INGENIC_HELIX_H264E_RC_NETLINK
	unsigned int pid;   /*netlink port*/
#define H264E_MAX_NL_MSG    1024
	unsigned char recved_msg[H264E_MAX_NL_MSG]; /* netlink received msg.*/
	struct completion msg_complete;
#endif

	void *priv;
};

extern int h264e_generate_headers(struct h264e_ctx *ctx, int is_stream_on);

extern int h264e_encode_headers(struct h264e_ctx *ctx, struct ingenic_vcodec_mem *bs);

extern int h264e_encode(struct h264e_ctx *ctx, struct video_frame_buffer *frame, struct ingenic_vcodec_mem *bs, int force_idr, int *keyframe);

extern int h264e_set_fmt(struct h264e_ctx *ctx, int width, int height, int format);
extern int h264e_alloc_workbuf(struct h264e_ctx *ctx);

extern int h264e_free_workbuf(struct h264e_ctx *ctx);

extern int h264e_encoder_init(struct h264e_ctx *ctx);

extern int h264e_encoder_deinit(struct h264e_ctx *ctx);

extern void h264e_set_priv(struct h264e_ctx *ctx, void *data);

#ifdef CONFIG_INGENIC_HELIX_H264E_RC_NETLINK

	extern int h264e_netlink_test(void);

	extern int h264e_nl_init_ctx(struct h264e_ctx *ctx, struct nl_ctx_info *info);
	extern int h264e_nl_free_ctx(struct h264e_ctx *ctx);
	extern int h264e_nl_setup_headers(struct h264e_ctx *ctx, struct nl_header_info *info);
	extern int h264e_nl_rc_video_cfg(struct h264e_ctx *ctx, struct h264e_params *p);
	extern int h264e_nl_rc_frame_start(struct h264e_ctx *ctx, struct h264e_frame_start_params *p);
	extern int h264e_nl_rc_frame_end(struct h264e_ctx *ctx, struct h264e_frame_end_params *p);

#endif

#ifdef CONFIG_INGENIC_HELIX_H264E_RC_IN_KERNEL
	/* Rate Control API. */
	extern int h264e_rc_ctx_init(struct h264e_ctx *ctx);

	extern int h264e_rc_ctx_free(struct h264e_ctx *ctx);

	extern int h264e_rc_video_cfg(struct h264e_ctx *ctx, struct h264e_params *p);

	extern int h264e_rc_frame_start(struct h264e_ctx *ctx, struct h264e_frame_start_params *p);

	extern int h264e_rc_frame_end(struct h264e_ctx *ctx, struct h264e_frame_end_params *p);
#endif

#endif
