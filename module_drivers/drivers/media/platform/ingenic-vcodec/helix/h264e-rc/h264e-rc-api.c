#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/v4l2-controls.h>

#include "../api/helix_x264_enc.h"
#include "librc/jzm_enc_api.h"
#include "../h264enc/common.h"
#include "../h264enc/set.h"
#include "../h264e_rc.h"
#include "../h264e_rc_proto.h"

#if 0
static void hexdump(unsigned char *buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if ((i % 16) == 0)
			printf("%s%08x: ", i ? "\n" : "",
			       (unsigned int)&buf[i]);
		printf("%02x ", buf[i]);
	}
	printf("\n");
}
#endif

int h264e_rc_ctx_init(struct h264e_ctx *ctx)
{
	ctx->rc = kmalloc(sizeof(VPU_RC_S), GFP_KERNEL);
	if (!ctx->rc) {
		printk("Failed to malloc ctx->rc\n");
		goto err;
	}
	memset(ctx->rc, 0, sizeof(VPU_RC_S));
	VPU_RC_S *rc = ctx->rc;

	rc->h264SliceInfo = kmalloc(sizeof(H264E_SliceInfo_t), GFP_KERNEL);
	if (!rc->h264SliceInfo) {
		printk("Failed to malloc rc->Sliceinfo\n");
		goto err;
	}
	memset(rc->h264SliceInfo, 0, sizeof(H264E_SliceInfo_t));

	rc->vpuPri = ctx->emc_buf;;
	rc->h264SliceInfo->des_va = ctx->desc;

	/*由内核将编码好的sps和pps copy到用户空间保留.*/
	//  ctx->sps = malloc(sizeof(h264_sps_t));
	//  ctx->pps = malloc(sizeof(h264_pps_t));

	//  printf("info->emc_buf_paddr: %x map@%x\n", info->emc_buf_paddr, rc->vpuPri);
	//  printf("info->desc_paddr: %x map@%x\n", info->desc_paddr, rc->h264SliceInfo->des_va);

	/*先考虑单例的.*/
	/*create a handle , with nlmsg_hdr->type*/

	ctx->p.i_idr_pic_id = 0;

	h264_cabac_init();

	RC_CFG_S *cfg = NULL;
	cfg = &rc->cfg;

	cfg->u16FrmWidth    = 1280;
	cfg->u16FrmHeight   = 720;
	cfg->encProtocol    = ENC_H264;
	cfg->gopMode        = NORMALP;

	if (ctx->p.rc_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR) {
		cfg->rcMode       = VBR;
	} else if (ctx->p.rc_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_CBR) {
		cfg->rcMode     = CBR;
	} else if (ctx->p.rc_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_CQ) {
		cfg->rcMode     = CQP;
	} else {
		cfg->rcMode     = CBR;  // default.
	}

	char *rc_mode [] = {"vbr", "cbr", "cqp"};

	printk("rc_mode: %s\n", rc_mode[ctx->p.rc_mode]);

	eprc_default_set_T21(rc);

err:
	return 0;
}

int h264e_rc_ctx_free(struct h264e_ctx *ctx)
{

	kfree(ctx->rc->h264SliceInfo);
	JZ_VPU_RC_FREE_T21(ctx->rc);
	kfree(ctx->rc);
	return 0;
}

int h264e_rc_video_cfg(struct h264e_ctx *ctx, struct h264e_params *p)
{
	RC_ATTR_H264_CBR_S *acbr = NULL;
	VPU_RC_S *rc = ctx->rc;
	RC_CFG_S *cfg = &rc->cfg;
	acbr = &cfg->attrH264Cbr;

	RC_ATTR_H264_CQP_S  *cqp = &rc->cfg.attrH264Cqp;
	RC_PARAM_H264_CBR_S *pcbr = &rc->cfg.paramH264Cbr;
	RC_ATTR_H264_VBR_S  *avbr = &rc->cfg.attrH264Vbr;
	//        RC_PARAM_H264_VBR_S *pvbr = &rc->cfg.paramH264Vbr;

	cfg->u16FrmWidth = p->width;
	cfg->u16FrmHeight = p->height;
	cfg->u16MbWidth          = ((cfg->u16FrmWidth + 15) & ~15) / 16;
	cfg->u16MbHeight         = ((cfg->u16FrmHeight + 15) & ~15) / 16;

	if (p->i_qp > 0 && p->i_qp < 51) {
		cfg->s8StartQp      = p->i_qp;
	} else {
		cfg->s8StartQp      = 30;
	}

	/* CQP attributes */

	if (cfg->rcMode == CQP) {
		cqp->u32Gop     = p->gop_size;
		cqp->u8IQp      = p->i_qp;
		cqp->u8PQp      = p->p_qp;
	} else if (cfg->rcMode == CBR) {
		/* CBR attributes. */
		acbr->u32Gop        = p->gop_size;
		acbr->u32BitRate    = p->bitrate / 1000;
		pcbr->u8MaxQp       = p->h264_max_qp;
		pcbr->u8MinQp       = p->h264_min_qp;
	} else if (cfg->rcMode == VBR) {
		avbr->u32Gop        = p->gop_size;
		avbr->u8MaxQp       = p->h264_max_qp;
		avbr->u8MinQp       = p->h264_min_qp;
		avbr->u32MaxBitRate = p->bitrate / 1000;
	}

	JZ_VPU_RC_VIDEO_CFG_T21(rc);

	/* send back to kernel ?*/
	return 0;
}

static int h264e_slice_init(struct h264e_ctx *ctx, int i_nal_type)
{
	VPU_RC_S *rc = ctx->rc;

	if (i_nal_type == NAL_SLICE_IDR) {

		h264e_slice_header_init(&ctx->sh, &ctx->sps, &ctx->pps, ctx->p.i_idr_pic_id, rc->u32FrmCnt, rc->u8FrmQp);
		ctx->p.i_idr_pic_id ^= 1;
	} else {
		h264e_slice_header_init(&ctx->sh, &ctx->sps, &ctx->pps, -1, rc->u32FrmCnt, rc->u8FrmQp);

		ctx->sh.i_num_ref_idx_l0_active = 1;
		ctx->sh.i_num_ref_idx_l1_active = 1;
	}

	return 0;
}

int h264e_rc_frame_start(struct h264e_ctx *ctx, struct h264e_frame_start_params *p)
{
	VPU_RC_S *rc = ctx->rc;
	H264E_SliceInfo_t *s = rc->h264SliceInfo;
	int i_nal_type = 0;
	int i_nal_ref_idc = 0;
	bs_t bs;
	h264_pps_t *pps = &ctx->pps;
	unsigned int slice_header_size = 0;
	unsigned int slice_header_len = 0;
	int i = 0;

#if 0
	printf("---%s, %d, msg->ctx_id: %d\n", __func__, __LINE__, msg->ctx_id);

	printf("---rc->u32FrmCnt: %x\n", rc->u32FrmCnt);

	printf("p->bIFrmReq: %d\n", p->bIFrmReq);
	printf("p->frmSkipType: %d\n", p->frmSkipType);
	printf("p->raw[0]: 0x%x\n", p->raw[0]);
	printf("p->raw[1]: 0x%x\n", p->raw[1]);
	printf("p->raw[2]: 0x%x\n", p->raw[2]);
	for (i = 0; i < 3; i++) {
		printf("p->fb[%d][0]: 0x%x\n", i, p->fb[i][0]);
		printf("p->fb[%d][1]: 0x%x\n", i, p->fb[i][1]);
	}
	printf("p->stride[0]: %d\n", p->stride[0]);
	printf("p->stride[1]: %d\n", p->stride[1]);

#endif
	rc->bIFrmReq        = p->bIFrmReq;
	rc->frmSkipType     = p->frmSkipType;

	s->raw[0]       = p->raw[0];
	s->raw[1]       = p->raw[1];
	s->raw[2]       = p->raw[2];

	for (i = 0; i < 3; i++) {
		s->fb[i][0] = p->fb[i][0];
		s->fb[i][1] = p->fb[i][1];
	}

	s->stride[0]        = p->stride[0];
	s->stride[1]        = p->stride[1];

	s->raw_format   = p->raw_format;

	s->emc_dblk_pa  = p->emc_dblk_pa;
	s->emc_recon_pa = s->emc_dblk_pa + DBLK_SIZE;
	s->emc_mv_pa    = s->emc_recon_pa + RECON_SIZE;
	s->emc_se_pa    = s->emc_mv_pa + MV_SIZE;
	s->emc_qpt_pa   = s->emc_se_pa + SE_SIZE;
	s->emc_rc_pa    = s->emc_qpt_pa + QPT_SIZE;
	s->emc_cpx_pa   = s->emc_rc_pa + RC_SIZE;
	s->emc_mod_pa   = s->emc_cpx_pa + CPX_SIZE;
	s->emc_sad_pa   = s->emc_mod_pa + MOD_SIZE;
	s->emc_ncu_pa   = s->emc_sad_pa + SAD_SIZE;

	s->emc_bs_pa    = p->emc_bs_pa;

	JZ_VPU_RC_FRAME_START_T21(rc);

	if (rc->frmType == SLICE_TYPE_IDR) {
		i_nal_type  = NAL_SLICE_IDR;
		i_nal_ref_idc   = NAL_PRIORITY_HIGHEST;
		ctx->sh.i_type = M_SLICE_TYPE_I;
	} else {
		i_nal_type = NAL_SLICE;
		i_nal_ref_idc = NAL_PRIORITY_HIGH;
		ctx->sh.i_type = M_SLICE_TYPE_P;
	}

	bs_init(&bs, ctx->slice_header, MAX_SLICE_HEADER_SIZE);
	h264e_slice_init(ctx, i_nal_type);

	/*1. nal_header*/
	bs_write(&bs, 8, 0x00);
	bs_write(&bs, 8, 0xFF);
	bs_write(&bs, 8, 0x00);
	bs_write(&bs, 8, 0x01);
	/* nal header */
	bs_write(&bs, 8, (0 << 7 | (i_nal_ref_idc << 5) | (i_nal_type)));

	/*2. slice_header, */
	h264e_slice_header_write(&bs, &ctx->sh, i_nal_ref_idc);

	//hexdump(ctx->slice_header, 64);

	//print_hex_dump(KERN_INFO, "sl_header@", DUMP_PREFIX_ADDRESS, 16, 1, ctx->slice_header, 64, 1);
	if (pps->b_cabac) {
		bs_align_1(&bs);
		h264_cabac_context_init(&ctx->cb, rc->frmType, ctx->sh.i_qp, ctx->sh.i_cabac_init_idc);

	}

	s->state = ctx->cb.state;

	slice_header_size = bs_pos(&bs);
	slice_header_len = (slice_header_size + 7) / 8;

	s->bs_head_en           = 1;    /*X2000 new add, fix value*/
	s->bs_head_va           = (paddr_t *)ctx->slice_header;
	s->bs_head_len          = (slice_header_size + 7) / 8;
	s->bs_rbsp_en           = 1;

	p->slice_header_len = slice_header_len;

	/*TODO: tuning???*/
	s->dct8x8_en = 0;
	s->force_i16dc = 0;
	s->force_i16 = 0;

	//H264E_DumpInfo(s);
	H264E_SliceInit(s);
	return 0;
}

int h264e_rc_frame_end(struct h264e_ctx *ctx, struct h264e_frame_end_params *p)
{

	VPU_RC_S *rc = ctx->rc;

	rc->u32FrmActBs = p->u32FrmActBs;

	JZ_VPU_RC_FRAME_END_T21(rc);

	return 0;
}
