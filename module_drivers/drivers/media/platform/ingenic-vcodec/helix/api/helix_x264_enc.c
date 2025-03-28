#include "helix_x264_enc.h"
#define MB_WID ((s->frame_width + 15) / 16)
#define MB_HEI ((s->frame_height + 15) / 16)
__place_k0_data__
static uint32_t lps_range[64] = {
	0xeeceaefc,  0xe1c3a5fc,  0xd6b99cfc,  0xcbb094f2,
	0xc1a78ce4,  0xb79e85da,  0xad967ece,  0xa48e78c4,
	0x9c8772ba,  0x94806cb0,  0x8c7966a6,  0x8573619e,
	0x7e6d5c96,  0x7867578e,  0x72625386,  0x6c5d4e80,
	0x66584a78,  0x61544672,  0x5c4f436c,  0x574b3f66,
	0x53473c62,  0x4e43395c,  0x4a403658,  0x463d3352,
	0x4339304e,  0x3f362e4a,  0x3c342b46,  0x39312942,
	0x362e273e,  0x332c253c,  0x30292338,  0x2e272136,
	0x2b251f32,  0x29231d30,  0x27211c2c,  0x251f1a2a,
	0x231e1928,  0x211c1826,  0x1f1b1624,  0x1d191522,
	0x1c181420,  0x1a17131e,  0x1915121c,  0x1714111a,
	0x16131018,  0x15120f18,  0x14110e16,  0x13100d14,
	0x120f0c14,  0x110e0c12,  0x100d0b12,  0x0f0d0a10,
	0x0e0c0a10,  0x0d0b090e,  0x0c0a090e,  0x0c0a080c,
	0x0b09070c,  0x0a09070a,  0x0a08070a,  0x0908060a,
	0x09070608,  0x08070508,  0x07060508,  0x00000000,
};

#define C_SPE_ADD_MB_NUM 3
void BUF_SHARE_CFG(_H264E_SliceInfo *s, uint32_t *buf_addr_group, uint8_t *buf_ref_mby_size, uint8_t *buf_odma_spe_flag, uint8_t *buf_odma_alg_flag)
{
	int beyond_size = (s->buf_share_size + 1) * 64;

	int y_space = (s->mb_width * 16) * (s->mb_height * 16 + beyond_size);
	int c_space = (s->mb_width * 16) * (s->mb_height * 8 + (beyond_size / 2));
	int y_every_space = (s->mb_width * 16) * beyond_size;
	int c_every_space = (s->mb_width * 16) * (beyond_size / 2);

	int mb_total = s->mb_width * s->mb_height;

	int spe_frm = 0;
	int spe_ad_flag = 0;
	int spe_mi_flag = 0;
	int last_spe_ad_flag = 0;
	int last_spe_mi_flag = 0;
	int c_ofst_addr_n = ((s->frame_idx) * c_every_space) / c_space;
	int last_c_ofst_addr_n = ((s->frame_idx - 1) * c_every_space) / c_space;

	if (mb_total % 2) {
		//spe_frm = 1;
	}

	//buf_beyond_yaddr
	buf_addr_group[0] = s->fb[0][0] + y_space;
	//buf_beyond_caddr
	buf_addr_group[1] = spe_frm ? s->fb[0][1] + c_space + (128 * C_SPE_ADD_MB_NUM) : s->fb[0][1] + c_space;
	int tmp_beyond_caddr = spe_frm ? s->fb[0][1] + c_space + 128 : s->fb[0][1] + c_space;

	//calc last frame addr
	int last_y_ofst_addr = s->frame_idx == 0 ? 0 : ((s->frame_idx - 1) * y_every_space) % y_space;
	int last_y_tmp_addr  = last_y_ofst_addr ? (buf_addr_group[0] - last_y_ofst_addr) : s->fb[0][0];

	int last_c_ofst_addr = s->frame_idx == 0 ? 0 : ((s->frame_idx - 1) * c_every_space) % c_space;
	int last_c_tmp_addr  = last_c_ofst_addr ? (tmp_beyond_caddr - last_c_ofst_addr) : s->fb[0][1];

	if ((spe_frm & last_c_tmp_addr) != s->fb[0][1]) {
		if (last_c_ofst_addr_n % 2) {
			last_c_tmp_addr -= 256;
			last_spe_mi_flag = 1;
		}

		if (last_c_tmp_addr % 256) {
			last_c_tmp_addr += 128;
			last_spe_ad_flag = 1;
		}

		if ((last_c_tmp_addr + c_every_space > buf_addr_group[1]) & last_spe_mi_flag) {
			last_c_tmp_addr += 256;
		}
	}

	//refy_addr_ba0
	buf_addr_group[4]  = s->frame_idx == 0 ? s->fb[0][0] : last_y_tmp_addr;
	//refy_addr_ba1
	buf_addr_group[5]  = s->fb[0][0];
	//refc_addr_ba0
	buf_addr_group[6]  = s->frame_idx == 0 ? s->fb[0][1] : last_c_tmp_addr;
	//refc_addr_ba1
	buf_addr_group[7]  = s->fb[0][1];
	//ref_mby_size
	*buf_ref_mby_size  = !s->buf_share_en ? 0xff : ((buf_addr_group[0] - buf_addr_group[4]) / (s->mb_width * 16)) / 16 - 1;

	int y_ofst_addr   = (s->frame_idx * y_every_space) % y_space;
	int y_tmp_addr    = y_ofst_addr ? (buf_addr_group[0] - y_ofst_addr) : s->fb[0][0];
	//buf_start_yaddr
	buf_addr_group[2]   = s->frame_idx == 0 ? s->fb[0][0] : y_tmp_addr;

	int c_ofst_addr   = (s->frame_idx * c_every_space) % c_space;
	int c_tmp_addr    = c_ofst_addr ? (tmp_beyond_caddr - c_ofst_addr) : s->fb[0][1];
	//buf_start_caddr
	buf_addr_group[3]   = s->frame_idx == 0 ? s->fb[0][1] : c_tmp_addr;

	int spe_flag = 1;
	if ((spe_frm & buf_addr_group[3]) != s->fb[0][1]) {
		if (c_ofst_addr_n % 2) {
			buf_addr_group[3] -= 256;
			spe_mi_flag = 1;
		}

		if (buf_addr_group[3] % 256) {
			buf_addr_group[3] += 128;
			spe_ad_flag = 1;
		}

		if ((buf_addr_group[3] + c_every_space > buf_addr_group[1]) & spe_mi_flag) {
			buf_addr_group[3] += 256;
			spe_flag = 0;
		}

	}

	*buf_odma_spe_flag = spe_flag;

	int alg_flag = 0;
	int c_pxl_space_row = s->mb_width * 16 * 8;
	if (((buf_addr_group[1] - (128 * C_SPE_ADD_MB_NUM)) - buf_addr_group[3]) % c_pxl_space_row == 0) {
		alg_flag = 1;
	}

	*buf_odma_alg_flag = alg_flag;
}

void H264E_SliceInit(_H264E_SliceInfo *s)
{
	unsigned int i, j/*, tmp = 0*/;
	volatile unsigned int *chn = (volatile unsigned int *)s->des_va;

	GEN_VDMA_ACFG(chn, TCSM_FLUSH, 0, 0);

	/**************************************************
	 buf share cfg
	 *************************************************/

	uint32_t buf_addr_group[8];
	uint8_t buf_ref_mby_size;
	uint8_t buf_odma_spe_flag;
	uint8_t buf_odma_alg_flag;

	BUF_SHARE_CFG(s, buf_addr_group, &buf_ref_mby_size, &buf_odma_spe_flag, &buf_odma_alg_flag);

	uint32_t buf_beyond_yaddr = buf_addr_group[0];
	uint32_t buf_beyond_caddr = buf_addr_group[1];
	uint32_t buf_start_yaddr  = buf_addr_group[2];
	uint32_t buf_start_caddr  = buf_addr_group[3];
	uint32_t refy_addr_ba0 = buf_addr_group[4];
	uint32_t refy_addr_ba1 = buf_addr_group[5];
	uint32_t refc_addr_ba0 = buf_addr_group[6];
	uint32_t refc_addr_ba1 = buf_addr_group[7];
	uint8_t  ref_mby_size = buf_ref_mby_size;
	uint8_t  odma_spe_flag = buf_odma_spe_flag;
	uint8_t  odma_alg_flag = buf_odma_alg_flag;

	if (0) {
		printk("[BUF_SHARE] frame_idx: %d,  y_start: 0x%x, y_base: 0x%x, y_beyond: 0x%x, spe_flag: %d, alg_flag: %d\n",
		       s->frame_idx, buf_addr_group[2], s->fb[0][0], buf_addr_group[0], odma_spe_flag, odma_alg_flag);
		printk("                            c_start: 0x%x, c_base: 0x%x, c_beyond: 0x%x\n",
		       buf_addr_group[3], s->fb[0][1], buf_addr_group[1]);
		printk("                            ifa_y0: 0x%x, ifa_y1: 0x%x, ifa_c0: 0x%x, ifa_c1: 0x%x, mby: %d\n",
		       buf_addr_group[4], buf_addr_group[5], buf_addr_group[6], buf_addr_group[7], buf_ref_mby_size);
	}

	/**************************************************
	 EFE configuration
	 *************************************************/
	GEN_VDMA_ACFG(chn, REG_EFE_GEOM, 0, (EFE_FST_MBY(s->first_mby) |
	                                     EFE_LST_MBY(s->last_mby) |
	                                     EFE_LST_MBX(MB_WID - 1)));

	GEN_VDMA_ACFG(chn, REG_EFE_COEF_BA, 0, VRAM_MAU_RESA);
	GEN_VDMA_ACFG(chn, REG_EFE_RAWY_SBA, 0, s->raw[0]);
	GEN_VDMA_ACFG(chn, REG_EFE_RAWU_SBA, 0, s->raw[1]);
	GEN_VDMA_ACFG(chn, REG_EFE_RAWV_SBA, 0, s->raw[2]);
	GEN_VDMA_ACFG(chn, REG_EFE_RAW_STRD, 0, (EFE_RAW_STRDY(s->stride[0]) |
	              EFE_RAW_STRDC(s->stride[1])));
	GEN_VDMA_ACFG(chn, REG_EFE_TOPMV_BA, 0, VRAM_TOPMV_BA);
	GEN_VDMA_ACFG(chn, REG_EFE_TOPPA_BA, 0, VRAM_TOPPA_BA);
	GEN_VDMA_ACFG(chn, REG_EFE_MECHN_BA, 0, VRAM_ME_CHN_BASE);
	GEN_VDMA_ACFG(chn, REG_EFE_MAUCHN_BA, 0, VRAM_MAU_CHN_BASE);
	GEN_VDMA_ACFG(chn, REG_EFE_DBLKCHN_BA, 0, VRAM_DBLK_CHN_BASE);
	GEN_VDMA_ACFG(chn, REG_EFE_SDECHN_BA, 0, VRAM_SDE_CHN_BASE);
	GEN_VDMA_ACFG(chn, REG_EFE_RAW_DBA, 0, VRAM_RAWY_BA);

	GEN_VDMA_ACFG(chn, REG_EFE_ROI_MAX_QP, 0, s->max_qp);

	GEN_VDMA_ACFG(chn, REG_EFE_ROI_BASE_INFO0, 0,
	              (((s->roi_info[3].roi_qp & 0x3F) << 2 |
	                (s->roi_info[3].roi_md & 0x1)  << 1 |
	                (s->roi_info[3].roi_en & 0x1)  << 0) << 24) |
	              (((s->roi_info[2].roi_qp & 0x3F) << 2 |
	                (s->roi_info[2].roi_md & 0x1)  << 1 |
	                (s->roi_info[2].roi_en & 0x1)  << 0) << 16) |
	              (((s->roi_info[1].roi_qp & 0x3F) << 2 |
	                (s->roi_info[1].roi_md & 0x1)  << 1 |
	                (s->roi_info[1].roi_en & 0x1)  << 0) << 8) |
	              (((s->roi_info[0].roi_qp & 0x3F) << 2 |
	                (s->roi_info[0].roi_md & 0x1)  << 1 |
	                (s->roi_info[0].roi_en & 0x1)  << 0) << 0)
	             );

	GEN_VDMA_ACFG(chn, REG_EFE_ROI_BASE_INFO1, 0,
	              (((s->roi_info[7].roi_qp & 0x3F) << 2 |
	                (s->roi_info[7].roi_md & 0x1)  << 1 |
	                (s->roi_info[7].roi_en & 0x1)  << 0) << 24) |
	              (((s->roi_info[6].roi_qp & 0x3F) << 2 |
	                (s->roi_info[6].roi_md & 0x1)  << 1 |
	                (s->roi_info[6].roi_en & 0x1)  << 0) << 16) |
	              (((s->roi_info[5].roi_qp & 0x3F) << 2 |
	                (s->roi_info[5].roi_md & 0x1)  << 1 |
	                (s->roi_info[5].roi_en & 0x1)  << 0) << 8) |
	              (((s->roi_info[4].roi_qp & 0x3F) << 2 |
	                (s->roi_info[4].roi_md & 0x1)  << 1 |
	                (s->roi_info[4].roi_en & 0x1)  << 0) << 0)
	             );

	GEN_VDMA_ACFG(chn, REG_EFE_ROI_POS_INFO0, 0,
	              ((s->roi_info[0].roi_bmby & 0xFF) << 24 |
	               (s->roi_info[0].roi_umby & 0xFF) << 16 |
	               (s->roi_info[0].roi_rmbx & 0xFF) << 8  |
	               (s->roi_info[0].roi_lmbx & 0xFF) << 0)
	             );

	GEN_VDMA_ACFG(chn, REG_EFE_ROI_POS_INFO1, 0,
	              ((s->roi_info[1].roi_bmby & 0xFF) << 24 |
	               (s->roi_info[1].roi_umby & 0xFF) << 16 |
	               (s->roi_info[1].roi_rmbx & 0xFF) << 8  |
	               (s->roi_info[1].roi_lmbx & 0xFF) << 0)
	             );

	GEN_VDMA_ACFG(chn, REG_EFE_ROI_POS_INFO2, 0,
	              ((s->roi_info[2].roi_bmby & 0xFF) << 24 |
	               (s->roi_info[2].roi_umby & 0xFF) << 16 |
	               (s->roi_info[2].roi_rmbx & 0xFF) << 8  |
	               (s->roi_info[2].roi_lmbx & 0xFF) << 0)
	             );

	GEN_VDMA_ACFG(chn, REG_EFE_ROI_POS_INFO3, 0,
	              ((s->roi_info[3].roi_bmby & 0xFF) << 24 |
	               (s->roi_info[3].roi_umby & 0xFF) << 16 |
	               (s->roi_info[3].roi_rmbx & 0xFF) << 8  |
	               (s->roi_info[3].roi_lmbx & 0xFF) << 0)
	             );

	GEN_VDMA_ACFG(chn, REG_EFE_ROI_POS_INFO4, 0,
	              ((s->roi_info[4].roi_bmby & 0xFF) << 24 |
	               (s->roi_info[4].roi_umby & 0xFF) << 16 |
	               (s->roi_info[4].roi_rmbx & 0xFF) << 8  |
	               (s->roi_info[4].roi_lmbx & 0xFF) << 0)
	             );

	GEN_VDMA_ACFG(chn, REG_EFE_ROI_POS_INFO5, 0,
	              ((s->roi_info[5].roi_bmby & 0xFF) << 24 |
	               (s->roi_info[5].roi_umby & 0xFF) << 16 |
	               (s->roi_info[5].roi_rmbx & 0xFF) << 8  |
	               (s->roi_info[5].roi_lmbx & 0xFF) << 0)
	             );

	GEN_VDMA_ACFG(chn, REG_EFE_ROI_POS_INFO6, 0,
	              ((s->roi_info[6].roi_bmby & 0xFF) << 24 |
	               (s->roi_info[6].roi_umby & 0xFF) << 16 |
	               (s->roi_info[6].roi_rmbx & 0xFF) << 8  |
	               (s->roi_info[6].roi_lmbx & 0xFF) << 0)
	             );

	GEN_VDMA_ACFG(chn, REG_EFE_ROI_POS_INFO7, 0,
	              ((s->roi_info[7].roi_bmby & 0xFF) << 24 |
	               (s->roi_info[7].roi_umby & 0xFF) << 16 |
	               (s->roi_info[7].roi_rmbx & 0xFF) << 8  |
	               (s->roi_info[7].roi_lmbx & 0xFF) << 0)
	             );

	GEN_VDMA_ACFG(chn, REG_EFE_QP_GEN_TAB, 0, ((VRAM_QP_TAB_BA & 0xFFFFF) |
	              ((s->qp_tab_len & 0x7FF) << 20) |
	              (s->qp_tab_mode & 0x1) << 31));
	/*
	for(i=0; i<s->qp_tab_len;i++){
	  GEN_VDMA_ACFG(chn, VRAM_QP_TAB_BA+i*4, 0, *(s->qp_tab+i));
	}
	*/
	GEN_VDMA_ACFG(chn, REG_EFE_CQP_OFST, 0, (s->cqp_offset & 0x1F));

	GEN_VDMA_ACFG(chn, REG_EFE_SSAD, 0, 0);

	GEN_VDMA_ACFG(chn, REG_EFE_DCS, 0,
	              (EFE_DCS_EN(s->daisy_chain_en) |
	               EFE_DCS_OTH(s->curr_thread_id) |
	               EFE_DCS_RT(4))
	             );
	/*
	GEN_VDMA_ACFG(chn, REG_EFE_QPG_CTRL , 0, (s->sas_en       << 0 |
	                  s->crp_en       << 1 |
	                  s->sas_mthd     << 2 |
	                  0               << 3 |
	                  s->rc_bu_level  << 4 |
	                  s->rc_wait_en   << 7 |
	                  s->base_qp      << 8 |
	                  s->rc_mb_level  << 14 |
	                  s->min_qp       << 16 |
	                  s->qp_tab_en    << 22 |
	                  s->mbrc_qpg_sel << 23 | // 0 : only mbrc_ofst valid;  1: both mbrc_ofst and qpg_ofst valid
	                  s->max_qp       << 24 |
	                  s->mbrc_enable  << 30 |
	                  s->mbrc_wait    << 31 ));
	*/
	GEN_VDMA_ACFG(chn, REG_EFE_QPG_CTRL, 0, (s->sas_en       << 0 |
	              s->crp_en       << 1 |
	              s->sas_mthd     << 2 |
	              0               << 3 |
	              s->rc_bu_level  << 4 |
	              s->rc_bu_wait_en   << 7 |
	              s->base_qp      << 8 |
	              s->rc_mb_level  << 14 |
	              s->min_qp       << 16 |
	              s->qp_tab_en    << 22 |
	              s->mbrc_qpg_sel << 23 |//0:mb_rc effect, 1:mb_rc + qpg effect
	              s->max_qp       << 24 |
	              s->rc_mb_wait_en  << 30 |
	              (s->rc_mb_en & 0x1) << 31)); //mb_rc_en

	GEN_VDMA_ACFG(chn, REG_EFE_QPG_CFG0, 0, ((s->qpg_flt_thd[0] & 0x7) << 0 |
	              (s->qpg_flt_thd[1] & 0x7) << 4 |
	              (s->qpg_flt_thd[2] & 0xF) << 8 |
	              (s->qpg_flt_thd[3] & 0x7) << 12 |
	              (s->qpg_flt_thd[4] & 0x7) << 16));

	GEN_VDMA_ACFG(chn, REG_EFE_QPG_CFG1, 0, (s->qpg_mb_thd[0] << 0 |
	              s->qpg_mb_thd[1] << 16));

	GEN_VDMA_ACFG(chn, REG_EFE_QPG_CFG2, 0, (s->qpg_mb_thd[2] << 0 |
	              s->qpg_mb_thd[3] << 16));

	GEN_VDMA_ACFG(chn, REG_EFE_QPG_CFG3, 0, (s->qpg_mb_thd[4] << 0 |
	              s->qpg_mb_thd[5] << 16));

	GEN_VDMA_ACFG(chn, REG_EFE_QPG_CFG4, 0, (s->qpg_mb_thd[6] << 0));

	GEN_VDMA_ACFG(chn, REG_EFE_QPG_CFG5, 0, ((s->qpg_mbqp_ofst[0] & 0x3F) << 0 |
	              (s->qpg_mbqp_ofst[1] & 0x3F) << 8 |
	              (s->qpg_mbqp_ofst[2] & 0x3F) << 16 |
	              (s->qpg_mbqp_ofst[3] & 0x3F) << 24));

	GEN_VDMA_ACFG(chn, REG_EFE_QPG_CFG6, 0, ((s->qpg_mbqp_ofst[4] & 0x3F) << 0 |
	              (s->qpg_mbqp_ofst[5] & 0x3F) << 8 |
	              (s->qpg_mbqp_ofst[6] & 0x3F) << 16 |
	              (s->qpg_mbqp_ofst[7] & 0x3F) << 24));

	GEN_VDMA_ACFG(chn, REG_EFE_EIGEN_CFG0, 0, ((s->force_i16 & 0x1) << 0 |
	              (s->refresh_en & 0x1) << 1 |
	              (s->refresh_mode & 0x1) << 2 |
	              (s->refresh_bias & 0x7) << 3 |
	              (s->sas_eigen_en & 0x1)    << 6 |
	              (s->crp_eigen_en & 0x1)    << 7 |
	              (s->sas_eigen_dump & 0x1)  << 8 |
	              (s->crp_eigen_dump & 0x1)  << 9 |
	              (s->cplx_thd_sel & 0x1) << 10 |
	              (s->diff_cplx_sel & 0x1) << 11 |
	              (s->diff_thd_sel & 0x3) << 12 |
	              (s->mb_mode_use & 0x1)  << 14 |
	              (s->rrs_en & 0x1) << 15 |
	              (s->refresh_cplx_thd & 0xFF) << 16 |
	              (s->i16dc_cplx_thd & 0xFF) << 24));

	GEN_VDMA_ACFG(chn, REG_EFE_EIGEN_CFG1, 0, ((s->i16dc_qp_base & 0x3F) << 0 |
	              (s->i16dc_qp_sel & 0x1) << 6 |
	              (s->i16_qp_base & 0x3F) << 8 |
	              (s->i16_qp_sel & 0x1) << 14 |
	              (s->diff_qp_base[0] & 0x3F) << 16 |
	              (s->diff_qp_sel[0] & 0x1) << 22 |
	              (s->diff_qp_base[1] & 0x3F) << 24 |
	              (s->diff_qp_sel[1] & 0x1) << 30));

	GEN_VDMA_ACFG(chn, REG_EFE_EIGEN_CFG2, 0, ((s->cplx_thd_idx[0] & 0x7) << 0 |
	              (s->cplx_thd_idx[1] & 0x7) << 3 |
	              (s->cplx_thd_idx[2] & 0x7) << 6 |
	              (s->cplx_thd_idx[3] & 0x7) << 9 |
	              (s->cplx_thd_idx[4] & 0x7) << 12 |
	              (s->cplx_thd_idx[5] & 0x7) << 15 |
	              (s->cplx_thd_idx[6] & 0x7) << 18 |
	              (s->cplx_thd_idx[7] & 0x7) << 21 |
	              (s->cplx_thd_idx[8] & 0x7) << 24 |
	              (s->cplx_thd_idx[9] & 0x7) << 27));

	GEN_VDMA_ACFG(chn, REG_EFE_EIGEN_CFG3, 0, ((s->cplx_thd_idx[10] & 0x7) << 0 |
	              (s->cplx_thd_idx[11] & 0x7) << 3 |
	              (s->cplx_thd_idx[12] & 0x7) << 6 |
	              (s->cplx_thd_idx[13] & 0x7) << 9 |
	              (s->cplx_thd_idx[14] & 0x7) << 12 |
	              (s->cplx_thd_idx[15] & 0x7) << 15 |
	              (s->cplx_thd_idx[16] & 0x7) << 18 |
	              (s->cplx_thd_idx[17] & 0x7) << 21 |
	              (s->cplx_thd_idx[18] & 0x7) << 24 |
	              (s->cplx_thd_idx[19] & 0x7) << 27));

	GEN_VDMA_ACFG(chn, REG_EFE_EIGEN_CFG4, 0, ((s->cplx_thd_idx[20] & 0x7) << 0 |
	              (s->cplx_thd_idx[21] & 0x7) << 3 |
	              (s->cplx_thd_idx[22] & 0x7) << 6 |
	              (s->cplx_thd_idx[23] & 0x7) << 9 |
	              (s->diff_cplx_thd & 0xFF) << 24));

	GEN_VDMA_ACFG(chn, REG_EFE_EIGEN_CFG5, 0, ((s->cplx_thd[0] & 0xFF) << 0 |
	              (s->cplx_thd[1] & 0xFF) << 8 |
	              (s->cplx_thd[2] & 0xFF) << 16 |
	              (s->cplx_thd[3] & 0xFF) << 24));

	GEN_VDMA_ACFG(chn, REG_EFE_EIGEN_CFG6, 0, ((s->cplx_thd[4] & 0xFF) << 0 |
	              (s->cplx_thd[5] & 0xFF) << 8 |
	              (s->cplx_thd[6] & 0xFF) << 16 |
	              (s->cplx_thd[7] & 0xFF) << 24));

	GEN_VDMA_ACFG(chn, REG_EFE_DIFFY_CFG, 0, ((s->diff_thd_base[0] & 0xFFFF) << 0 |
	              (s->diff_thd_ofst[0] & 0xFFFF) << 16));

	GEN_VDMA_ACFG(chn, REG_EFE_DIFFU_CFG, 0, ((s->diff_thd_base[1] & 0xFFFF) << 0 |
	              (s->diff_thd_ofst[1] & 0xFFFF) << 16));

	GEN_VDMA_ACFG(chn, REG_EFE_DIFFV_CFG, 0, ((s->diff_thd_base[2] & 0xFFFF) << 0 |
	              (s->diff_thd_ofst[2] & 0xFFFF) << 16));

	GEN_VDMA_ACFG(chn, REG_EFE_SKIN_CTRL, 0, ((s->skin_dt_en & 0x1) << 0 |
	              (s->skin_lvl & 0x3) << 1 |
	              (s->ncu_mov_en & 0x1) << 3 |
	              (s->skin_cnt_thd & 0x7F) << 8));

	GEN_VDMA_ACFG(chn, REG_EFE_SKIN_PTHD0, 0, ((s->skin_pxlu_thd[0][0] & 0xFF) << 0 |
	              (s->skin_pxlu_thd[0][1] & 0xFF) << 8 |
	              (s->skin_pxlv_thd[0][0] & 0xFF) << 16 |
	              (s->skin_pxlv_thd[0][1] & 0xFF) << 24));

	GEN_VDMA_ACFG(chn, REG_EFE_SKIN_PTHD1, 0, ((s->skin_pxlu_thd[1][0] & 0xFF) << 0 |
	              (s->skin_pxlu_thd[1][1] & 0xFF) << 8 |
	              (s->skin_pxlv_thd[1][0] & 0xFF) << 16 |
	              (s->skin_pxlv_thd[1][1] & 0xFF) << 24));

	GEN_VDMA_ACFG(chn, REG_EFE_SKIN_PTHD2, 0, ((s->skin_pxlu_thd[2][0] & 0xFF) << 0 |
	              (s->skin_pxlu_thd[2][1] & 0xFF) << 8 |
	              (s->skin_pxlv_thd[2][0] & 0xFF) << 16 |
	              (s->skin_pxlv_thd[2][1] & 0xFF) << 24));

	GEN_VDMA_ACFG(chn, REG_EFE_SKIN_QP_OFST, 0, ((s->skin_qp_ofst[0] & 0x3F) << 0 |
	              (s->skin_qp_ofst[1] & 0x3F) << 8 |
	              (s->skin_qp_ofst[2] & 0x3F) << 16 |
	              (s->skin_qp_ofst[3] & 0x3F) << 24));

	GEN_VDMA_ACFG(chn, REG_EFE_SKIN_PARAM0, 0, ((s->mult_factor[0] & 0xF) << 0 |
	              (s->shift_factor[0][0] & 0x7) << 4 |
	              (s->shift_factor[0][1] & 0x7) << 8 |
	              (s->skin_ofst[1] & 0x3FF) << 11 |
	              (s->skin_ofst[0] & 0x3FF) << 21));

	GEN_VDMA_ACFG(chn, REG_EFE_SKIN_PARAM1, 0, ((s->mult_factor[1] & 0xF) << 0 |
	              (s->shift_factor[1][0] & 0x7) << 8 |
	              (s->shift_factor[1][1] & 0x7) << 12 |
	              (s->shift_factor[1][2] & 0x7) << 16 |
	              (s->skin_ofst[2] & 0x3FF) << 20));

	GEN_VDMA_ACFG(chn, REG_EFE_SKIN_PARAM2, 0, ((s->mult_factor[2] & 0xF) << 0 |
	              (s->shift_factor[2][0] & 0x7) << 8 |
	              (s->shift_factor[2][1] & 0x7) << 12 |
	              (s->shift_factor[2][2] & 0x7) << 16 |
	              (s->skin_ofst[3] & 0x3FF) << 20));

	/**************************************************
	 RC configuration
	 *************************************************/
	if (s->rc_mb_en) { //rate ctrl
		//fprintf(stderr,"s->rc_bu_wait_en:%d\n",s->rc_bu_wait_en);
		//fprintf(stderr,"s->last_bu_size:%d, s->rc_bu_size:%d\n",s->last_bu_size, s->rc_bu_size);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_BINFO1, 0, EFE_RC_BU_LSIZE(s->last_bu_size) | EFE_RC_BU_SIZE(s->rc_bu_size));
		//fprintf(stderr,"s->tar_bs_thd[0]:%d, s->tar_bs_thd[1]:%d\n",s->tar_bs_thd[0],s->tar_bs_thd[1]);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_GP0THD, 0, ((s->tar_bs_thd[0] & 0xFFFF) << 0 |
		              (s->tar_bs_thd[1] & 0xFFFF) << 16));
		//fprintf(stderr,"s->tar_bs_thd[2]:%d, s->tar_bs_thd[3]:%d\n",s->tar_bs_thd[2],s->tar_bs_thd[3]);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_GP1THD, 0, ((s->tar_bs_thd[2] & 0xFFFF) << 0 |
		              (s->tar_bs_thd[3] & 0xFFFF) << 16));
		//fprintf(stderr,"s->tar_bs_thd[4]:%d, s->tar_bs_thd[5]:%d\n",s->tar_bs_thd[4],s->tar_bs_thd[5]);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_GP2THD, 0, ((s->tar_bs_thd[4] & 0xFFFF) << 0 |
		              (s->tar_bs_thd[5] & 0xFFFF) << 16));
		//fprintf(stderr,"s->rc_frm_tbs:%d\n",s->rc_frm_tbs & 0x7FFFFFFF);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_TBS,    0, s->rc_frm_tbs & 0x7FFFFFFF);
		//fprintf(stderr,"s->bu_alg0_qpo[2]:%d, s->bu_alg0_qpo[1]:%d, s->bu_alg0_qpo[0]:%d\n",s->bu_alg0_qpo[2], s->bu_alg0_qpo[1], s->bu_alg0_qpo[0]);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_BNQA0, 0, EFE_RC_QPO_CFG(0, s->bu_alg0_qpo[2], s->bu_alg0_qpo[1], s->bu_alg0_qpo[0]));
		//fprintf(stderr,"s->bu_alg0_qpo[5]:%d, s->bu_alg0_qpo[4]:%d, s->bu_alg0_qpo[3]:%d\n",s->bu_alg0_qpo[5], s->bu_alg0_qpo[4], s->bu_alg0_qpo[3]);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_BPQA0, 0, EFE_RC_QPO_CFG(0, s->bu_alg0_qpo[5], s->bu_alg0_qpo[4], s->bu_alg0_qpo[3]));
		//fprintf(stderr,"s->bu_alg1_qpo[2]:%d, s->bu_alg1_qpo[1]:%d, s->bu_alg1_qpo[0]:%d\n",s->bu_alg1_qpo[2], s->bu_alg1_qpo[1], s->bu_alg1_qpo[0]);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_BNQA1, 0, EFE_RC_QPO_CFG(0, s->bu_alg1_qpo[2], s->bu_alg1_qpo[1], s->bu_alg1_qpo[0]));
		//fprintf(stderr,"s->bu_alg1_qpo[5]:%d, s->bu_alg1_qpo[4]:%d, s->bu_alg1_qpo[3]:%d\n",s->bu_alg1_qpo[5], s->bu_alg1_qpo[4], s->bu_alg1_qpo[3]);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_BPQA1, 0, EFE_RC_QPO_CFG(0, s->bu_alg1_qpo[5], s->bu_alg1_qpo[4], s->bu_alg1_qpo[3]));
		//fprintf(stderr,"s->mb_cs_qpo[2]:%d, s->mb_cs_qpo[1]:%d, s->mb_cs_qpo[0]:%d\n",s->mb_cs_qpo[2], s->mb_cs_qpo[1], s->mb_cs_qpo[0]);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_MNQCS, 0, EFE_RC_QPO_CFG(0, s->mb_cs_qpo[2], s->mb_cs_qpo[1], s->mb_cs_qpo[0]));
		//fprintf(stderr,"s->mb_cs_qpo[6]:%d, s->mb_cs_qpo[5]:%d, s->mb_cs_qpo[4]:%d, s->mb_cs_qpo[3]:%d\n",
		//s->mb_cs_qpo[6], s->mb_cs_qpo[5], s->mb_cs_qpo[4], s->mb_cs_qpo[3]);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_MPQCS, 0, EFE_RC_QPO_CFG(s->mb_cs_qpo[6], s->mb_cs_qpo[5], s->mb_cs_qpo[4], s->mb_cs_qpo[3]));
		//fprintf(stderr,"s->mb_top_bs_qpo[1]:%d, s->mb_top_bs_qpo[0]:%d\n",s->mb_top_bs_qpo[1], s->mb_top_bs_qpo[0]);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_MTBQ, 0, EFE_RC_QPO_CFG(0, 0, s->mb_top_bs_qpo[1], s->mb_top_bs_qpo[0]));
		//fprintf(stderr,"s->mb_rinfo_qpo[1]:%d, s->mb_rinfo_qpo[0]:%d\n",s->mb_rinfo_qpo[1], s->mb_rinfo_qpo[0]);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_MRFQ, 0, EFE_RC_QPO_CFG(0, 0, s->mb_rinfo_qpo[1], s->mb_rinfo_qpo[0]));
		//fprintf(stderr,"s->mb_target_avg_bs[1]:%d, s->mb_target_avg_bs[0]:%d\n",s->mb_target_avg_bs[1], s->mb_target_avg_bs[0]);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_MAMIN, 0, s->mb_target_avg_bs[0] & 0x3FFFF);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_MAMAX, 0, s->mb_target_avg_bs[1] & 0x3FFFF);
		//fprintf(stderr,"s->rc_bu_num:%d, s->mb_gp_num:%d\n",s->rc_bu_num, s->mb_gp_num);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_BBS, 0, s->avg_bu_bs & 0x7FFFFF);
		GEN_VDMA_ACFG(chn, REG_EFE_RC_MINFO, 0, (//EFE_RC_MAD_CFG_RDY |
		                  EFE_RC_MBGP_NUM(s->mb_gp_num) |
		                  (s->rc_mb_en & 0x1)));
		char rc_slice_type = s->rc_frm_tbs ? s->frame_type : 0;
		GEN_VDMA_ACFG(chn, REG_EFE_RC_BINFO0, 0, (EFE_RC_SLICE_TP(rc_slice_type) |
		              EFE_RC_BU_CFG(s->rc_bcfg_mode) |
		              EFE_RC_BU_NUM(s->rc_bu_num) |
		              ((s->rc_mb_en >> 1) & 0x1)));
	}

	/**************************************************
	 JRFD configuration
	 *************************************************/
	GEN_VDMA_ACFG(chn, REG_JRFD_CTRL, 0, (((1 == 2) << 31) | /* 2:NV21 ,1:NV12*/
	                                      (2 << 28) |//sim_lvl
	                                      (s->jrfd_enable << 27) |
	                                      (s->frame_height << 14) |
	                                      s->frame_width));
	GEN_VDMA_ACFG(chn, REG_JRFD_HDYA, 0, s->jh[1][0]);
	GEN_VDMA_ACFG(chn, REG_JRFD_HDCA, 0, s->jh[1][1]);

	GEN_VDMA_ACFG(chn, REG_JRFD_HSTR, 0, (((s->cm_head_total & 0xffff) << 16) | (s->lm_head_total & 0xffff)));

	GEN_VDMA_ACFG(chn, REG_JRFD_BDYA, 0, s->fb[1][0]);//tile Y address
	GEN_VDMA_ACFG(chn, REG_JRFD_BDCA, 0, s->fb[1][1]); //tile C address

	GEN_VDMA_ACFG(chn, REG_JRFD_BSTR, 0,
	              JRFD_BODY_STRDY(((s->frame_width + 15) / 16) * 16) |
	              JRFD_BODY_STRDC(((s->frame_width + 15) / 16) * 8));

	GEN_VDMA_ACFG(chn, REG_JRFD_MHDY, 0, s->jh[2][0]);
	GEN_VDMA_ACFG(chn, REG_JRFD_MHDC, 0, s->jh[2][1]);
	GEN_VDMA_ACFG(chn, REG_JRFD_MBDY, 0, s->fb[2][0]);
	GEN_VDMA_ACFG(chn, REG_JRFD_MBDC, 0, s->fb[2][1]);

	GEN_VDMA_ACFG(chn, REG_JRFD_TRIG, 0, ((1 << 5) |//init
	                                      (0 << 4)  //ckg low means do clock-gating
	                                     ));//enable_tile

	/**************************************************
	 Motion configuration
	*************************************************/
	if (s->frame_type) {
		//set CHAIN/TMV/SYNC ADDR
		GEN_VDMA_ACFG(chn, REG_MCE_CHN_BA, 0, VRAM_ME_CHN_BASE);//chn
		GEN_VDMA_ACFG(chn, REG_MCE_TMV_BA, 0, VRAM_TOPMV_BA);//tmv
		GEN_VDMA_ACFG(chn, REG_MCE_CHN_SYNC, 0, VRAM_ME_DSA);//sync

		//set frame size
		GEN_VDMA_ACFG(chn, REG_MCE_FRM_SIZE, 0,
		              MCE_FRM_SIZE_FH(s->frame_height - 1) |
		              MCE_FRM_SIZE_FW(s->frame_width - 1) |
		              MCE_FRM_SIZE_LRE(s->frm_re[0]) |
		              MCE_FRM_SIZE_RRE(s->frm_re[1]) |
		              MCE_FRM_SIZE_TRE(s->frm_re[2]) |
		              MCE_FRM_SIZE_BRE(s->frm_re[3]));

		//set frame stride
		GEN_VDMA_ACFG(chn, REG_MCE_FRM_STRD, 0,
		              MCE_FRM_STRD_STRDC(((s->frame_width + 15) / 16) * 16) |
		              MCE_FRM_STRD_STRDY(((s->frame_width + 15) / 16) * 16));

		//set COMP_CTRL
		GEN_VDMA_ACFG(chn, REG_MCE_COMP_CTRL, 0,
		              MCE_COMP_CTRL_CCE |
		              MCE_COMP_CTRL_CTAP(MCE_TAP_TAP2) |
		              MCE_COMP_CTRL_CSPT(MCE_SPT_BILI) |
		              MCE_COMP_CTRL_CSPP(MCE_SPP_EPEL) |
		              MCE_COMP_CTRL_YCE |
		              MCE_COMP_CTRL_YTAP(MCE_TAP_TAP6) |
		              MCE_COMP_CTRL_YSPT(MCE_SPT_AUTO) |
		              MCE_COMP_CTRL_YSPP(MCE_SPP_QPEL));

		//set ESTI_CTRL
		unsigned int esti_ctrl = 0;
		esti_ctrl |= (MCE_ESTI_CTRL_LSP(s->lambda_scale_parameter) |
		              MCE_ESTI_CTRL_SCL(s->scl)/*trbl mvp*/ |
		              MCE_ESTI_CTRL_FBG(0) |
		              MCE_ESTI_CTRL_CLMV |
		              MCE_ESTI_CTRL_MSS(s->max_sech_step_i) |
		              MCE_ESTI_CTRL_QRL(s->qpel_en) |
		              MCE_ESTI_CTRL_HRL(s->hpel_en));

		esti_ctrl |= MCE_ESTI_CTRL_PUE_16X16;

		GEN_VDMA_ACFG(chn, REG_MCE_ESTI_CTRL, 0, esti_ctrl);

		//set MRGI
		unsigned int merg_info = 0;
		GEN_VDMA_ACFG(chn, REG_MCE_MRGI, 0, merg_info);

		//set MVR
		GEN_VDMA_ACFG(chn, REG_MCE_MVR, 0,
		              MCE_MVR_MVRY(s->max_mvry_i * 4) |
		              MCE_MVR_MVRX(s->max_mvrx_i * 4));
		if (s->buf_share_en) {
			GEN_VDMA_ACFG(chn, REG_MCE_REFY0, 0, refy_addr_ba0);
			GEN_VDMA_ACFG(chn, REG_MCE_REFC0, 0, refc_addr_ba0);
			GEN_VDMA_ACFG(chn, REG_MCE_REFY0_1, 0, refy_addr_ba1);
			GEN_VDMA_ACFG(chn, REG_MCE_REFC0_1, 0, refc_addr_ba1);
		} else {
			GEN_VDMA_ACFG(chn, REG_MCE_REFY0, 0, s->fb[1][0]);
			GEN_VDMA_ACFG(chn, REG_MCE_REFC0, 0, s->fb[1][1]);
			GEN_VDMA_ACFG(chn, REG_MCE_REFY0_1, 0, s->fb[2][0]);
			GEN_VDMA_ACFG(chn, REG_MCE_REFC0_1, 0, s->fb[2][1]);
		}
		GEN_VDMA_ACFG(chn, REG_MCE_REFY1, 0, s->fb[2][0]);
		GEN_VDMA_ACFG(chn, REG_MCE_REFC1, 0, s->fb[2][1]);

		GEN_VDMA_ACFG(chn, REG_MCE_SLC_SPOS, 0, 0);
		GEN_VDMA_ACFG(chn, REG_MCE_FSC, 0,
		              MCE_FSC_FSE(s->fs_en) |
		              MCE_FSC_FSMD(s->fs_md) |
		              MCE_FSC_PERX(s->fs_px) |
		              MCE_FSC_PERY(s->fs_py) |
		              MCE_FSC_RECX(s->fs_rx) |
		              MCE_FSC_RECY(s->fs_ry)
		             );
		GEN_VDMA_ACFG(chn, REG_MCE_GLB_MV, 0, MCE_GLB_MVX(s->glb_mvx) | MCE_GLB_MVY(s->glb_mvy));
		/**/
		GEN_VDMA_ACFG(chn, REG_MCE_GLB_CTRL, 0, (MCE_GLB_CTRL_FMV0(s->force_mv0_en) |
		              MCE_GLB_CTRL_MSTEP(s->me_step_en) |
		              MCE_GLB_CTRL_STEP1(s->me_step_1) |
		              MCE_GLB_CTRL_STEP0(s->me_step_0) |
		              MCE_GLB_CTRL_RBS(ref_mby_size) |
		              MCE_GLB_CTRL_FMS(s->frm_mv_size) |
		              MCE_GLB_CTRL_FRMMV(s->frm_mv_en) |
		              MCE_GLB_CTRL_GLBMV(s->glb_mv_en) |
		              MCE_GLB_CTRL_DCT8(s->dct8x8_en) |
		              MCE_GLB_CTRL_PSKIP(s->pskip_en) | MCE_GLB_CTRL_INIT |
		              MCE_GLB_CTRL_MREF(s->mref_en) | MCE_GLB_CTRL_RM(s->ref_mode) |
		              MCE_GLB_CTRL_JRFD(!s->jrfd_enable)));
	}
	/**************************************************
	 VMAU configuration
	 *************************************************/
	GEN_VDMA_ACFG(chn, REG_VMAU_GBL_RUN, 0, VMAU_RESET);
	GEN_VDMA_ACFG(chn, REG_VMAU_VIDEO_TYPE, 0, (VMAU_MODE_ENC | 6 << 7
	              | VMAU_I4_MSK(0)
	              | VMAU_I16_MSK(0)
	              | VMAU_I8_MSK(0)
	              | VMAU_PT8_MSK(0)
	              | VMAU_IS_ISLICE(!s->frame_type)
	              | VMAU_PREDM_MSK(s->intra_mode_msk & 0x1ffff)
	              | VMAU_TSE(s->use_intra_in_pframe)));
	GEN_VDMA_ACFG(chn, REG_VMAU_NCCHN_ADDR, 0, VRAM_MAU_CHN_BASE);
	GEN_VDMA_ACFG(chn, REG_VMAU_ENC_DONE, 0, VRAM_MAU_ENC_SYNA);
	GEN_VDMA_ACFG(chn, REG_VMAU_DEC_DONE, 0, VRAM_MAU_DEC_SYNA);
	GEN_VDMA_ACFG(chn, REG_VMAU_Y_GS, 0, (VMAU_FRM_WID(MB_WID * 16)
	                                      | VMAU_FRM_HEI(MB_HEI * 16)));
	GEN_VDMA_ACFG(chn, REG_VMAU_GBL_CTR, 0, (VMAU_CTRL_TO_DBLK
	              | VMAU_CTRL_FIFO_M
	              | VMAU_LAMBDA_THRETH((s->intra_mode_msk >> 17) & 0x3)));
	GEN_VDMA_ACFG(chn, REG_VMAU_DEADZONE, 0, (VMAU_DEADZONE0_IY(s->deadzone[0])
	              | VMAU_DEADZONE1_PY(s->deadzone[1])
	              | VMAU_DEADZONE2_IC(s->deadzone[2])
	              | VMAU_DEADZONE3_PC(s->deadzone[3])));
	GEN_VDMA_ACFG(chn, REG_VMAU_DEADZONE1, 0, ((s->deadzone[4] & 0x3f) << 0 |
	              (s->deadzone[5] & 0x3f) << 6 |
	              (s->deadzone[6] & 0x3f) << 12 |
	              (s->deadzone[7] & 0x3f) << 18 |
	              (s->deadzone[8] & 0x3f) << 24));
	GEN_VDMA_ACFG(chn, REG_VMAU_TOP_BASE, 0, VRAM_TOPPA_BA);
	GEN_VDMA_ACFG(chn, REG_VMAU_MD_CFG0, 0, (VMAU_MD_SLICE_I(!s->frame_type)
	              | VMAU_MD_SLICE_P(s->frame_type)
	              | VMAU_MD_I4_DIS(0)
	              | VMAU_MD_I16_DIS(0)
	              | VMAU_MD_PSKIP_DIS(0)
	              | VMAU_MD_P_L0_DIS(0)
	              | VMAU_MD_I8_DIS(0)
	              | VMAU_MD_PT8_DIS(0)
	              | VMAU_MD_DREF_EN(s->mref_en)
	              | VMAU_MD_DCT8_EN(s->dct8x8_en)
	              | VMAU_MD_FRM_REDGE(MB_WID - 1)
	              | VMAU_MD_FRM_BEDGE(MB_HEI - 1)));
	GEN_VDMA_ACFG(chn, REG_VMAU_MD_CFG1, 0, (VMAU_IPMY_BIAS_EN(s->intra_lambda_y_bias_en)
	              | VMAU_IPMC_BIAS_EN(s->intra_lambda_c_bias_en)
	              | VMAU_COST_BIAS_EN(s->cost_bias_en)
	              | VMAU_CSSE_BIAS_EN(s->chroma_sse_bias_en)
	              | VMAU_JMLAMBDA2_EN(s->jm_lambda2_en)
	              | VMAU_INTER_NEI_EN(s->inter_nei_en)
	              | VMAU_SKIP_BIAS_EN(s->skip_bias_en)
	              | VMAU_LMD_BIAS_EN(s->sse_lambda_bias_en)
	              | VMAU_INFO_EN(s->info_en)
	              | VMAU_DCM_EN(s->dcm_en)
	              | VMAU_MVDS_ALL(s->mvd_sum_all)
	              | VMAU_MVDS_ABS(s->mvd_sum_abs)
	              | VMAU_MVS_ALL(s->mv_sum_all)
	              | VMAU_MVS_ABS(s->mv_sum_abs)
	              | VMAU_P_L0_BIAS(s->cost_bias_p_l0)
	              | VMAU_PSKIP_BIAS(s->cost_bias_p_skip)
	              | VMAU_I4_BIAS(s->cost_bias_i_4x4)
	              | VMAU_I16_BIAS(s->cost_bias_i_16x16)));
	GEN_VDMA_ACFG(chn, REG_VMAU_MD_CFG2, 0, (VMAU_IPM_BIAS_0(s->intra_lambda_bias_0)
	              | VMAU_IPM_BIAS_1(s->intra_lambda_bias_1)
	              | VMAU_IPM_BIAS_2(s->intra_lambda_bias_2)
	              | VMAU_IPM_BIAS_QP0(s->intra_lambda_bias_qp0)
	              | VMAU_IPM_BIAS_QP1(s->intra_lambda_bias_qp1)
	              | VMAU_MD_FBC_EP(s->fbc_ep)));
	GEN_VDMA_ACFG(chn, REG_VMAU_MD_CFG3, 0, (VMAU_CSSE_BIAS_0(s->chroma_sse_bias_0)
	              | VMAU_CSSE_BIAS_1(s->chroma_sse_bias_1)
	              | VMAU_CSSE_BIAS_2(s->chroma_sse_bias_2)
	              | VMAU_CSSE_BIAS_QP0(s->chroma_sse_bias_qp0)
	              | VMAU_CSSE_BIAS_QP1(s->chroma_sse_bias_qp1)
	              | VMAU_LMD_BIAS(s->sse_lambda_bias)
	              | VMAU_PL0_FS_DIS(s->p_skip_pl0f_dis)
	              | VMAU_PT8_FS_DIS(s->p_skip_pt8f_dis)
	              | VMAU_PL0_COST_MAX(s->p_l0_dis)
	              | VMAU_PT8_COST_MAX(s->p_t8_dis)
	                                        ));
	GEN_VDMA_ACFG(chn, REG_VMAU_MD_CFG4, 0, (VMAU_YSSE_THR(s->ysse_thr)
	              | VMAU_I8_BIAS(s->cost_bias_i_8x8)
	              | VMAU_PT8_BIAS(s->cost_bias_p_t8)
	                                        ));
	GEN_VDMA_ACFG(chn, REG_VMAU_MD_CFG5, 0, (VMAU_CSSE_THR(s->csse_thr)
	              | VMAU_CQP_OFFSET(s->cqp_offset)
	              | VMAU_I4_COST_MAX(s->i_4x4_dis)
	              | VMAU_I8_COST_MAX(s->i_8x8_dis)
	              | VMAU_I16_COST_MAX(s->i_16x16_dis)
	                                        ));
	GEN_VDMA_ACFG(chn, REG_VMAU_MD_CFG6, 0, (VMAU_DCM_PARAM(s->dcm_param)
	              | VMAU_SDE_PRIOR(s->sde_prior)
	              | VMAU_DB_PRIOR(s->db_prior)
	                                        ));
	GEN_VDMA_ACFG(chn, REG_VMAU_MD_CFG7, 0, (VMAU_CFG_SIZE_X(s->cfg_size_x)
	              | VMAU_CFG_SIZE_Y(s->cfg_size_y)
	              | VMAU_CFG_IW_THR(s->cfg_iw_thr)
	              | VMAU_CFG_BASEQP(s->base_qp)
	              | VMAU_CFG_ALPHA(s->alpha_c0_offset)
	              | VMAU_PS_COST_MAX(s->p_skip_dis)
	                                        ));
	GEN_VDMA_ACFG(chn, REG_VMAU_MD_CFG8, 0, (VMAU_CFG_MVR_THR1(s->cfg_mvr_thr1)
	              | VMAU_CFG_MVR_THR2(s->cfg_mvr_thr2)));
	GEN_VDMA_ACFG(chn, REG_VMAU_MD_CFG9, 0, (VMAU_CFG_MVR_THR3(s->cfg_mvr_thr3)
	              | VMAU_CFG_BETA(s->beta_offset)
	                                        ));

	/* eigen leverages mode decision */
	int set_modectrl_bit0 = s->mb_mode_use & 0x1;
	int set_modectrl_bit2 = s->rrs_en || s->force_i16 || s->refresh_en;
	GEN_VDMA_ACFG(chn, REG_VMAU_MD_CFG11, 0, (VMAU_CFG_MD_RBIAS(s->refresh_bias)
	              | VMAU_CFG_MD_RBIAS_EN(1)
	              | VMAU_CFG_MD_PCDC_N0(s->mode_ctrl >> 3 & 7)
	              | VMAU_CFG_MD_IFA_VLD(set_modectrl_bit2)
	              | VMAU_CFG_MD_SLV_VLD(0)
	              | VMAU_CFG_MD_SET_VLD((s->mode_ctrl & 0x2) || set_modectrl_bit0)
	                                         ));

	GEN_VDMA_ACFG(chn, REG_VMAU_ACMASK, 0, (s->acmask_mode << 14) + 2); /* 2 for TEST: rd file */

	for (j = 0; j < 40; ++j) {
		GEN_VDMA_ACFG(chn, REG_VMAU_MD_MODE, 0, s->mode_ctrl_param[j]);
	}

	for (j = 0; j < 42; j++)
		GEN_VDMA_ACFG(chn, REG_VMAU_CTX, 0, s->state[ j < 10 ? (!s->frame_type ? j + 3 : j + 14)
		              : j < 22 ? j - 10 + 73
		              : j < 28 ? j - 22 + 64 : j - 28 + 40 ]);

	for (j = 0; j < 225; j++)
		GEN_VDMA_ACFG(chn, REG_VMAU_CTX_CFBC, 0, s->state[ j < 4  ? j + 85 :   //LUMA_DC_SIGNCG
		              j < 19 ? j - 4 + 105 : //LUMA_DC_SIGN
		              j < 34 ? j - 19 + 166 : //LUMA_DC_LAST
		              j < 44 ? j - 34 + 227 : //LUMA_DC_LEVEL
		              j < 48 ? j - 44 + 89  : //LUMA_AC_SIGNCG
		              j < 62 ? j - 48 + 120 : //LUMA_AC_SIGN
		              j < 76 ? j - 62 + 181 : //LUMA_AC_LAST
		              j < 86 ? j - 76 + 237 : //LUMA_AC_LEVEL
		              j < 90 ? j - 86 + 93  : //LUMA_44_SIGNCG
		              j < 105 ? j - 90 + 134  : //LUMA_44_SIGN
		              j < 120 ? j - 105 + 195  : //LUMA_44_LAST
		              j < 130 ? j - 120 + 247  : //LUMA_44_LEVEL
		              j < 134 ? j - 130 + 97   : //CHROMA_DC_SIGNCG
		              j < 137 ? j - 134 + 149  : //CHROMA_DC_SIGN
		              j < 140 ? j - 137 + 210  : //CHROMA_DC_LAST
		              j < 149 ? j - 140 + 257  : //CHROMA_DC_LEVEL
		              j < 153 ? j - 149 + 101  : //CHROMA_AC_SIGNCG
		              j < 167 ? j - 153 + 152  : //CHROMA_AC_SIGN
		              j < 181 ? j - 167 + 213  : //CHROMA_AC_LAST
		              j < 191 ? j - 181 + 266  : //CHROMA_AC_LEVEL
		              j < 206 ? j - 191 + 402  : //LUMA_88_SIGN
		              j < 215 ? j - 206 + 417  : //LUMA_88_LAST
		              j - 215 + 426        //LUMA_88_LEVEL
		                                                 ]);

#if 0
	for (j = 0; j < 4; j++) {
		for (i = 0; i < 4; i++) {
			GEN_VDMA_ACFG(chn, REG_VMAU_QT + tmp, 0, *(int *)(&s->scaling_list[j][i * 4]));
			tmp += 4;
		}
	}
	for (j = 0; j < 4; j++) {
		for (i = 0; i < 8; i++) {
			int tmp0 = (1ULL << 8) / s->scaling_list[j][2 * i];
			int tmp1 = (1ULL << 8) / s->scaling_list[j][2 * i + 1];
			GEN_VDMA_ACFG(chn, REG_VMAU_QT + tmp, 0, ((tmp0 & 0xFFFF) | (tmp1 << 16)));
			tmp += 4;
		}
	}
#endif
	GEN_VDMA_ACFG(chn, REG_VMAU_IPRED_CFG0, 0, (VMAU_IP_MD_VAL(s->force_i16dc) |
	              VMAU_IP_REF_NEB_4(s->ref_neb_4) |
	              VMAU_IP_REF_NEB_8(s->ref_neb_8) |
	              VMAU_IP_REF_PRD_C(s->pri_uv) |
	              VMAU_IP_REF_PRD_16(s->pri_16) |
	              VMAU_IP_REF_PRD_8(s->pri_8) |
	              VMAU_IP_REF_PRD_4(s->pri_4) |
	              VMAU_IP_REF_C4_EN(s->c_4_en) |
	              VMAU_IP_REF_C8_EN(s->c_8_en) |
	              VMAU_IP_REF_C16_EN(s->c_16_en) |
	              VMAU_IP_REF_CUV_EN(s->c_uv_en) |
	              VMAU_IP_REF_LMD4_EN(s->lamb_4_en) |
	              VMAU_IP_REF_LMD8_EN(s->lamb_8_en) |
	              VMAU_IP_REF_LMD16_EN(s->lamb_16_en) |
	              VMAU_IP_REF_LMDUV_EN(s->lamb_uv_en) |
	              VMAU_IP_REF_BIT4_EN(s->bit_4_en) |
	              VMAU_IP_REF_BIT8_EN(s->bit_8_en) |
	              VMAU_IP_REF_BIT16_EN(s->bit_16_en) |
	              VMAU_IP_REF_BITUV_EN(s->bit_uv_en)));
	GEN_VDMA_ACFG(chn, REG_VMAU_IPRED_CFG1, 0, (VMAU_IP_REF_4_BIT0(s->bit_4[0]) |
	              VMAU_IP_REF_4_BIT1(s->bit_4[1]) |
	              VMAU_IP_REF_4_BIT2(s->bit_4[2]) |
	              VMAU_IP_REF_4_BIT3(s->bit_4[3]) |
	              VMAU_IP_REF_8_BIT0(s->bit_8[0]) |
	              VMAU_IP_REF_8_BIT1(s->bit_8[1]) |
	              VMAU_IP_REF_8_BIT2(s->bit_8[2]) |
	              VMAU_IP_REF_8_BIT3(s->bit_8[3])));
	GEN_VDMA_ACFG(chn, REG_VMAU_IPRED_CFG2, 0, (VMAU_IP_REF_C_BIT0(s->bit_uv[0]) |
	              VMAU_IP_REF_C_BIT1(s->bit_uv[1]) |
	              VMAU_IP_REF_C_BIT2(s->bit_uv[2]) |
	              VMAU_IP_REF_C_BIT3(s->bit_uv[3]) |
	              VMAU_IP_REF_16_BIT0(s->bit_16[0]) |
	              VMAU_IP_REF_16_BIT1(s->bit_16[1]) |
	              VMAU_IP_REF_16_BIT2(s->bit_16[2]) |
	              VMAU_IP_REF_16_BIT3(s->bit_16[3])));
	GEN_VDMA_ACFG(chn, REG_VMAU_IPRED_CFG3, 0, (VMAU_IP_REF_LMDUV_IFO(s->lambda_infouv) |
	              VMAU_IP_REF_LMD16_IFO(s->lambda_info16) |
	              VMAU_IP_REF_LMD8_IFO(s->lambda_info8) |
	              VMAU_IP_REF_LMD4_IFO(s->lambda_info4) |
	              VMAU_IP_REF_NEB_4REF(s->ref_4) |
	              VMAU_IP_REF_NEB_8REF(s->ref_8)));
	GEN_VDMA_ACFG(chn, REG_VMAU_IPRED_CFG4, 0, (VMAU_IP_REF_C4_IFO((s->const_4[0] << 0) |
	              (s->const_4[1] << 4) |
	              (s->const_4[2] << 8) |
	              (s->const_4[3] << 12)) |
	              VMAU_IP_REF_C8_IFO((s->const_8[0] << 0) |
	                                 (s->const_8[1] << 4) |
	                                 (s->const_8[2] << 8) |
	                                 (s->const_8[3] << 12))));
	GEN_VDMA_ACFG(chn, REG_VMAU_IPRED_CFG5, 0, (VMAU_IP_REF_C16_IFO((s->const_16[0] << 0) |
	              (s->const_16[1] << 4) |
	              (s->const_16[2] << 8) |
	              (s->const_16[3] << 12)) |
	              VMAU_IP_REF_CUV_IFO((s->const_uv[0] << 0) |
	                                  (s->const_uv[1] << 4) |
	                                  (s->const_uv[2] << 8) |
	                                  (s->const_uv[3] << 12))));

	if (1) { //configure scalinglist8
		uint8_t scaling_list_z[2][64];
		int kk, jj;
		for (kk = 0; kk < 2; kk++) {
			for (jj = 0; jj < 64; jj++) {
				int jj_x = jj % 8; //0 ~ 7
				int jj_y = jj / 8; //0 ~ 7
				int jj_xx = jj_x % 4; //0~3
				int jj_xy = jj_x / 4; //0~1
				int jj_yx = jj_y % 4; //0~3
				int jj_yy = jj_y / 4; //0~1
				int zz = 16 * (jj_yy * 2 + jj_xy) + 4 * jj_yx + jj_xx;
				scaling_list_z[kk][zz] = s->scaling_list8[kk][jj];
			}
		}
		int ofst = 0 ;
		for (kk = 0; kk < 2; kk++) {
			for (jj = 0; jj < 32; jj++) {
#ifdef __KERNEL__
				unsigned long long t = 1ULL << 12;
				do_div(t, scaling_list_z[kk][2 * jj + 0]);
				int t0 = (int)t;

				t = 1ULL << 12;
				do_div(t, scaling_list_z[kk][2 * jj + 1]);
				int t1 = (int)t;
#else
				int t0 = (1ULL << 12) / scaling_list_z[kk][2 * jj + 0];
				int t1 = (1ULL << 12) / scaling_list_z[kk][2 * jj + 1];
#endif
				int tt = ((t0 & 0xfff) |
				          (t1 & 0xfff) << 12);
				GEN_VDMA_ACFG(chn, (VPU_BASE | 0x80800) + ofst, 0, tt);

				ofst += 4;
			}
		}
		for (kk = 0; kk < 2; kk++) {
			for (jj = 0; jj < 16; jj++) {
				int t0 = scaling_list_z[kk][4 * jj + 0];
				int t1 = scaling_list_z[kk][4 * jj + 1];
				int t2 = scaling_list_z[kk][4 * jj + 2];
				int t3 = scaling_list_z[kk][4 * jj + 3];
				int tt = ((t0 & 0x3f) |
				          (t1 & 0x3f) << 6 |
				          (t2 & 0x3f) << 12 |
				          (t3 & 0x3f) << 18);
				GEN_VDMA_ACFG(chn, (VPU_BASE | 0x80800) + ofst, 0, tt);
				ofst += 4;
			}
		}
	}

	GEN_VDMA_ACFG(chn, REG_VMAU_GBL_RUN, 0, VMAU_RUN);

	/**************************************************
	 DBLK configuration
	 *************************************************/
#if 0
	GEN_VDMA_ACFG(chn, REG_DBLK_TRIG, 0, DBLK_RESET);
	GEN_VDMA_ACFG(chn, REG_DBLK_DHA, 0, VRAM_DBLK_CHN_BASE);
	GEN_VDMA_ACFG(chn, REG_DBLK_GENDA, 0, VRAM_DBLK_CHN_SYNA);
	GEN_VDMA_ACFG(chn, REG_DBLK_GSIZE, 0, DBLK_GSIZE(MB_HEI, MB_WID));
	GEN_VDMA_ACFG(chn, REG_DBLK_GPOS, 0, DBLK_GPOS(s->first_mby, 0));
	GEN_VDMA_ACFG(chn, REG_DBLK_CTRL, 0, DBLK_CTRL(0, s->rotate, s->deblock));
	GEN_VDMA_ACFG(chn, REG_DBLK_GPIC_YA, 0, s->fb[0][0]/* - (MB_WID+3)*256*/);
	GEN_VDMA_ACFG(chn, REG_DBLK_GPIC_CA, 0, s->fb[0][1]/* - (MB_WID+3)*128*/);
	GEN_VDMA_ACFG(chn, REG_DBLK_GP_ENDA, 0, VRAM_DBLK_DOUT_SYNA);
	GEN_VDMA_ACFG(chn, REG_DBLK_GPIC_STR, 0, DBLK_GPIC_STR(MB_WID * 128, MB_WID * 256));
	GEN_VDMA_ACFG(chn, REG_DBLK_VTR, 0, DBLK_VTR(s->beta_offset, s->alpha_c0_offset, 0, 0,
	              s->frame_type, DBLK_FMT_H264));
	GEN_VDMA_ACFG(chn, REG_DBLK_TRIG, 0, DBLK_SLICE_RUN);
	//mosaic
	GEN_VDMA_ACFG(chn, REG_DBLK_MOS_CFG, 0, (((s->mos_sthd[1] & 0xF) << 28) |
	              ((s->mos_sthd[0] & 0xF) << 24) |
	              ((s->mos_gthd[1] & 0xFF) << 16) |
	              ((s->mos_gthd[0] & 0xFF) << 8)  |
	              (s->mosaic_en & 0x1)));
	GEN_VDMA_ACFG(chn, REG_DBLK_MOS_ADDR, 0, DBLK_MOS_BASE_ADDR);
#else
	GEN_VDMA_ACFG(chn, REG_DBLK_CFG, 0, (DBLK_CFG_ALPHA(s->alpha_c0_offset + 12)
	                                     | DBLK_CFG_BETA(s->beta_offset + 12)
	                                     | DBLK_CFG_NO_LFT(!s->deblock)));
	GEN_VDMA_ACFG(chn, REG_DBLK_TRIG, 0, DBLK_SLICE_RUN);
#endif
	/**************************************************
	 SDE configuration
	 *************************************************/
	GEN_VDMA_ACFG(chn, REG_SDE_STAT, 0, 0);
	GEN_VDMA_ACFG(chn, REG_SDE_GL_CTRL, 0, (SDE_MODE_STEP | SDE_EN));
	GEN_VDMA_ACFG(chn, REG_SDE_SL_GEOM, 0, SDE_SL_GEOM(MB_HEI, MB_WID,
	              s->first_mby, 0));
	GEN_VDMA_ACFG(chn, REG_SDE_CODEC_ID, 0, SDE_FMT_H264_ENC);
	//slice_info0 {desp_link_en[1], auto_syn_en[0]}
	GEN_VDMA_ACFG(chn, REG_SDE_CFG0, 0, 0x3);
	//slice_info1 {qp[8], slice_type[0]}
	GEN_VDMA_ACFG(chn, REG_SDE_CFG1, 0, ((s->qp << 8) |
	                                     (s->bs_head_en << 20) |//rbsp enable
	                                     (s->bs_rbsp_en << 7) |//rbsp enable
	                                     (s->mref_en << 6) | /*dual-ref enable*/
	                                     (s->dct8x8_en << 5) | /*dct8x8 enable*/
	                                     (s->skip_en << 4) |
	                                     (s->frame_type + 1)));
	//desp_addr
	GEN_VDMA_ACFG(chn, REG_SDE_CFG2, 0, VRAM_SDE_CHN_BASE);
	//sync_addr
	GEN_VDMA_ACFG(chn, REG_SDE_CFG3, 0, VRAM_SDE_SYNA);
	//bs_addr
	GEN_VDMA_ACFG(chn, REG_SDE_CFG4, 0, s->bs);
	//context table
	for (i = 0; i < 460; i++) {
		int idx = s->state[i];
		if (idx <= 63) {
			idx = 63 - idx;
		} else {
			idx -= 64;
		}
		GEN_VDMA_ACFG(chn, REG_SDE_CTX_TBL + i * 4, 0,
		              (lps_range[idx] | ((s->state[i] >> 6) & 0x1)));
	}
	//init sync
	GEN_VDMA_ACFG(chn, REG_SDE_SL_CTRL, 0, SDE_SLICE_INIT);
	/**************************************************
	 EMC init
	 *************************************************/
	char use_ddr_slow_mode = 0;
	if ((((s->frame_width + 15) / 16) - 1) > 32) {
		use_ddr_slow_mode = 0;
	} else {
		use_ddr_slow_mode = 1;
	}

	GEN_VDMA_ACFG(chn, REG_EMC_FRM_SIZE, 0,
	              (0x0) << 20 | //bs_full intc before slice end
	              (s->bs_size_en) << 19 | //bs space ctrl by bs_size
	              use_ddr_slow_mode << 18 | //ddr use slow mode
	              (s->qp_tab_en & 0x1) << 17 | //qp table en
	              //(s->i_4x4_dis & 0x1) << 16 | //4x4 close
	              (((s->frame_height + 15) / 16) - 1) << 8 |
	              (((s->frame_width + 15) / 16) - 1));
	GEN_VDMA_ACFG(chn, REG_EMC_BS_ADDR, 0, VDMA_ACFG_DHA(s->emc_bs_pa));
	GEN_VDMA_ACFG(chn, REG_EMC_DBLK_ADDR, 0, VDMA_ACFG_DHA(s->emc_dblk_pa));
	GEN_VDMA_ACFG(chn, REG_EMC_RECON_ADDR, 0, VDMA_ACFG_DHA(s->emc_recon_pa));
	GEN_VDMA_ACFG(chn, REG_EMC_MV_ADDR, 0, VDMA_ACFG_DHA(s->emc_mv_pa));
	GEN_VDMA_ACFG(chn, REG_EMC_SE_ADDR, 0, VDMA_ACFG_DHA(s->emc_se_pa));

	if ((s->rc_mb_en & 0x1) && s->frame_type) { //mb_rc+bu_rc or only mb_rc
		for (i = 0; i < s->mb_gp_num; i++) {

			int mb_gp_wth = (s->mb_width + 1) / 2;
			int mb_gp_wth_for_bu = mb_gp_wth << (s->rc_bu_level - 1);
			uint32_t r_idx = i;
			uint32_t bdry_2algn = (s->mb_width % 2) == 0;
			uint32_t mb_rinfo0_valid = 1;
			uint32_t mb_rinfo1_valid = !bdry_2algn && ((r_idx % mb_gp_wth) == (mb_gp_wth - 1)) ? 0 : 1;
			uint32_t bu_rinfo_valid = (r_idx % mb_gp_wth_for_bu) == (mb_gp_wth_for_bu - 1) ? 1 : 0;
			uint32_t bu_rinfo_idx = r_idx / mb_gp_wth_for_bu;
			uint32_t bdry_n2algn_num;
			if (bdry_2algn) {
				bdry_n2algn_num = 0;
			} else {
				bdry_n2algn_num = r_idx / ((s->mb_width + 1) / 2);
			}

			uint32_t mb_ref_info0 = s->mb_ref_info[r_idx * 2 - bdry_n2algn_num] & 0x7fff;
			uint32_t mb_ref_info1 = s->mb_ref_info[r_idx * 2 + 1 - bdry_n2algn_num] & 0x7fff;
			uint32_t mb_rinfo0 = (mb_rinfo0_valid << 15) | mb_ref_info0;
			uint32_t mb_rinfo1 = (mb_rinfo1_valid << 31) | (mb_ref_info1 << 16);
			uint32_t bu_rinfo = (bu_rinfo_valid << 31) | (s->bu_ref_info[bu_rinfo_idx] & 0x7fffffff);
			//fprintf(stderr,"i:%d,mb_ref_info0:0x%x,mb_ref_info1:0x%x,s->bu_ref_info:0x%x\n",i,mb_ref_info0,mb_ref_info1,s->bu_ref_info[bu_rinfo_idx] & 0x7fffffff);
			//fprintf(stderr,"s->mb_width:%d, mb_gp_wth:%d, s->rc_bu_level:%d, mb_gp_wth_for_bu:%d, r_idx:%d\n",s->mb_width, mb_gp_wth, s->rc_bu_level, mb_gp_wth_for_bu, r_idx);
			//fprintf(stderr,"mb_rinfo0:0x%x,mb_rinfo1:0x%x,bu_rinfo:0x%x\n",mb_rinfo0,mb_rinfo1,bu_rinfo);
			s->emc_rc_va[i * 2] = mb_rinfo1 | mb_rinfo0;
			s->emc_rc_va[i * 2 + 1] = bu_rinfo;
			//fprintf(stderr,"s->emc_rc_va[%d]:0x%x,s->emc_rc_va[%d]:0x%x\n",i*2,s->emc_rc_va[i*2],i*2+1,s->emc_rc_va[i*2+1]);
		}
		GEN_VDMA_ACFG(chn, REG_EMC_RC_RADDR, 0, VDMA_ACFG_DHA(s->emc_rc_pa));
	} else if ((s->rc_mb_en & 0x2) && s->frame_type) { //only bu_rc
		for (i = 0; i < s->rc_bu_num; i++) {
			uint32_t bu_rinfo = (1 << 31) | (s->bu_ref_info[i] & 0x7fffffff);
			s->emc_rc_va[i * 2 + 1] = bu_rinfo;
		}
		GEN_VDMA_ACFG(chn, REG_EMC_RC_RADDR, 0, VDMA_ACFG_DHA(s->emc_rc_pa));
	}
	if (s->rc_mb_en) {
		GEN_VDMA_ACFG(chn, REG_EMC_RC_WADDR, 0, VDMA_ACFG_DHA(s->emc_rc_pa));
	}

	if (s->qp_tab_en) {
		for (i = 0; i < s->qp_tab_len; i++) {
			s->emc_qpt_va[i] = s->qp_tab[i];
		}
	}
	if (s->ncu_mov_en) {
		for (i = 0; i < s->ncu_move_len; i++) {
			s->emc_ncu_va[i] = s->ncu_move_info[i];
		}
	}

	if (s->mb_mode_use) {
		int mb_mode_len = (s->mb_width * s->mb_height + 1) / 2;
		for (i = 0; i < mb_mode_len; i++) {
			s->emc_mod_va[i] = s->mb_mode_info[i];
		}
	}

	//rintf(stderr,"s->emc_qpt_va[0]=%x\n",s->emc_qpt_va[0]);
	//rintf(stderr,"s->emc_qpt_va[1]=%x\n",s->emc_qpt_va[1]);
	//rintf(stderr,"s->emc_qpt_va[2]=%x\n",s->emc_qpt_va[2]);
	GEN_VDMA_ACFG(chn, REG_EMC_QPT_ADDR, 0, VDMA_ACFG_DHA(s->emc_qpt_pa));
	GEN_VDMA_ACFG(chn, REG_EMC_CPX_ADDR, 0, VDMA_ACFG_DHA(s->emc_cpx_pa));
	GEN_VDMA_ACFG(chn, REG_EMC_MOD_ADDR, 0, VDMA_ACFG_DHA(s->emc_mod_pa));
	GEN_VDMA_ACFG(chn, REG_EMC_NCU_ADDR, 0, VDMA_ACFG_DHA(s->emc_ncu_pa));
	GEN_VDMA_ACFG(chn, REG_EMC_SAD_ADDR, 0, VDMA_ACFG_DHA(s->emc_sad_pa));
	GEN_VDMA_ACFG(chn, REG_EMC_BS_SIZE, 0, s->bs_size);
	GEN_VDMA_ACFG(chn, REG_EMC_SLV_INIT, 0, 0x1);
	/**************************************************
	 TOPMV/TOPPA init
	 *************************************************/
	/*   for(i=0; i<256; i++){ */
	/*     write_reg(VRAM_TOPMV_BA+i*4, 0); */
	/*     write_reg(VRAM_TOPPA_BA+i*4, 0); */
	/*   } */

	/**************************************************
	 SCH configuration
	 *************************************************/
	GEN_VDMA_ACFG(chn, REG_SCH_SCHC, 0, 0);
	GEN_VDMA_ACFG(chn, REG_SCH_BND, 0, 0);
	GEN_VDMA_ACFG(chn, REG_SCH_SCHG0, 0, 0);
	GEN_VDMA_ACFG(chn, REG_SCH_SCHG1, 0, 0);
	GEN_VDMA_ACFG(chn, REG_SCH_SCHE1, 0, 0);
	GEN_VDMA_ACFG(chn, REG_SCH_SCHE2, 0, 0);
	GEN_VDMA_ACFG(chn, REG_SCH_SCHE3, 0, 0);
	GEN_VDMA_ACFG(chn, REG_SCH_SCHE4, 0, 0);

#if 1
	GEN_VDMA_ACFG(chn, REG_SCH_SCHC, 0, (SCH_CH4_GS1 | SCH_CH4_PCH(0) |
	                                     SCH_CH3_GS1 | SCH_CH3_PCH(0) |
	                                     SCH_CH2_GS0 | SCH_CH2_PE | SCH_CH2_PCH(0) |
	                                     SCH_CH1_GS0 | SCH_CH1_PCH(0) |
	                                     (s->frame_type & 0x1) << 2));

	GEN_VDMA_ACFG(chn, REG_SCH_BND, 0, (SCH_CH4_HID(HID_SDE) | SCH_CH3_HID(HID_DBLK) |
	                                    SCH_CH2_HID(HID_VMAU) | SCH_CH1_HID(HID_MCE) |
	                                    SCH_DEPTH(SCH_FIFO_DEPTH) |
	                                    SCH_BND_G0F2 |
	                                    (s->frame_type & 0x1) << 0));
#else
	GEN_VDMA_ACFG(chn, REG_SCH_SCHC, 0, (SCH_CH4_GS1 | SCH_CH4_PE | SCH_CH4_PCH(0) |
	                                     SCH_CH3_GS1 | SCH_CH3_PE | SCH_CH3_PCH(0) |
	                                     SCH_CH2_GS0 | SCH_CH2_PE | SCH_CH2_PCH(0) |
	                                     SCH_CH1_GS0 | SCH_CH1_PCH(0) |
	                                     (s->frame_type & 0x1) << 2));

	GEN_VDMA_ACFG(chn, REG_SCH_BND, 0, (SCH_CH4_HID(HID_SDE) | SCH_CH3_HID(HID_DBLK) |
	                                    SCH_CH2_HID(HID_VMAU) | SCH_CH1_HID(HID_MCE) |
	                                    SCH_DEPTH(SCH_FIFO_DEPTH) |
	                                    SCH_BND_G1F4 | SCH_BND_G1F3 |
	                                    SCH_BND_G0F4 | SCH_BND_G0F3 | SCH_BND_G0F2 |
	                                    (s->frame_type & 0x1) << 0));
#endif

	/**************************************************
	 ODMA configuration
	 *************************************************/
	//when buf share is new addr(dynamic), else old addr(static)
	paddr_t odma_y_addr = s->buf_share_en ? buf_start_yaddr : s->fb[0][0];
	paddr_t odma_c_addr = s->buf_share_en ? buf_start_caddr : s->fb[0][1];
	//when buf share is base addr, else head addr
	paddr_t odma_com_addr0  = s->buf_share_en ? s->fb[0][0] : s->jh[0][0];
	paddr_t odma_com_addr1  = s->buf_share_en ? s->fb[0][1] : s->jh[0][1];
	//when buf share is beyond addr, else head spe addr
	paddr_t odma_com_addr2  = s->buf_share_en ? buf_beyond_yaddr : s->spe_y_addr;
	paddr_t odma_com_addr3  = s->buf_share_en ? buf_beyond_caddr : s->spe_c_addr;

	GEN_VDMA_ACFG(chn, REG_ODMA_CTRL, 0, (odma_spe_flag << 31 |
	                                      (s->buf_share_en << 30) |//buf share
	                                      (odma_alg_flag << 29) |
	                                      (s->jrfc_enable  << 28) |//compress_flag
	                                      (s->frame_height << 14) |
	                                      s->frame_width));

	GEN_VDMA_ACFG(chn, REG_ODMA_BDYA, 0, odma_y_addr);
	GEN_VDMA_ACFG(chn, REG_ODMA_BDCA, 0, odma_c_addr);

	GEN_VDMA_ACFG(chn, REG_ODMA_BSTR, 0,
	              ODMA_REC_STRDY(((s->frame_width + 15) / 16) * 16) |
	              ODMA_REC_STRDC(((s->frame_width + 15) / 16) * 8));

	GEN_VDMA_ACFG(chn, REG_ODMA_HDYA, 0, odma_com_addr0);
	GEN_VDMA_ACFG(chn, REG_ODMA_HDCA, 0, odma_com_addr1);
	GEN_VDMA_ACFG(chn, REG_ODMA_SPYA, 0, odma_com_addr2);
	GEN_VDMA_ACFG(chn, REG_ODMA_SPCA, 0, odma_com_addr3);

	GEN_VDMA_ACFG(chn, REG_ODMA_TRIG, 0,
	              ((0 << 6) |  //0 is crc en
	               (1 << 5) | //init
	               (0 << 4)  //ckg low means do clock-gating
	              ));

	/**************************************************
	 IFA configuration
	 *************************************************/

	GEN_VDMA_ACFG(chn, REG_IFA_FRM_SIZE, 0, (IFA_FRM_W(s->frame_width - 1)  |
	              IFA_FRM_H(s->frame_height - 1)));
	GEN_VDMA_ACFG(chn, REG_IFA_RAWY_BA, 0,  s->raw[0]);
	GEN_VDMA_ACFG(chn, REG_IFA_RAWC_BA, 0,  s->raw[1]);
	if (s->buf_share_en) {
		GEN_VDMA_ACFG(chn, REG_IFA_REFY_BA0, 0,  refy_addr_ba0);
		GEN_VDMA_ACFG(chn, REG_IFA_REFC_BA0, 0,  refc_addr_ba0);
		GEN_VDMA_ACFG(chn, REG_IFA_REFY_BA1, 0,  refy_addr_ba1);
		GEN_VDMA_ACFG(chn, REG_IFA_REFC_BA1, 0,  refc_addr_ba1);
	} else {
		GEN_VDMA_ACFG(chn, REG_IFA_REFY_BA0, 0,  s->fb[1][0]);
		GEN_VDMA_ACFG(chn, REG_IFA_REFC_BA0, 0,  s->fb[1][1]);
		GEN_VDMA_ACFG(chn, REG_IFA_REFY_BA1, 0,  s->fb[2][0]);
		GEN_VDMA_ACFG(chn, REG_IFA_REFC_BA1, 0,  s->fb[2][1]);
	}

	GEN_VDMA_ACFG(chn, REG_IFA_RAW_STR, 0, (IFA_STR_Y(s->stride[0]) |
	                                        IFA_STR_C(s->stride[1])));
	GEN_VDMA_ACFG(chn, REG_IFA_THRD_Y, 0,  s->rrs_thrd_y);
	GEN_VDMA_ACFG(chn, REG_IFA_THRD_C, 0, (IFA_THRD_U(s->rrs_thrd_u) |
	                                       IFA_THRD_V(s->rrs_thrd_v)));
	GEN_VDMA_ACFG(chn, REG_IFA_CTRL, 0, (IFA_CTRL_RAW_TYPE(s->raw_format == EFE_PLANE_NV12) |
	                                     IFA_CTRL_REF_BIDX(ref_mby_size) |
	                                     IFA_CTRL_RRS_EN(s->rrs_en)       |
	                                     IFA_CTRL_DUMP_EN(s->rrs_dump_en) |
	                                     IFA_CTRL_DDR_DW(!s->jrfd_enable) |
	                                     IFA_CTRL_RRS_SC(s->rrs_size_c)   |
	                                     IFA_CTRL_RRS_SY(s->rrs_size_y)   |
	                                     IFA_CTRL_UV_EN(s->rrs_uv_en)     |
	                                     IFA_CTRL_INIT));

	//sde slice header
	if (s->bs_head_en) {
		//printf("bs_head_rest\n");
		GEN_VDMA_ACFG(chn, REG_SDE_CFG5, 0, 0x1);
		//slice header lenth
		//printf("s->bs_head_len=0x%x\n",s->bs_head_len-1);
		GEN_VDMA_ACFG(chn, REG_SDE_CFG6, 0, s->bs_head_len - 1);
		for (i = 0; i < (s->bs_head_len + 3) / 4; i++) {
			//printf("s->bs_head_va[%x]=%x\n",i,s->bs_head_va[i]);
			GEN_VDMA_ACFG(chn, REG_SDE_CFG7, 0, s->bs_head_va[i]);
		}
		//printf("bs_head_work\n");
		GEN_VDMA_ACFG(chn, REG_SDE_CFG5, 0, 0x2);
	}
	//efe start
	uint8_t align8_flag = (s->frame_height & 0xF) != 0;

	GEN_VDMA_ACFG(chn, REG_EFE_CTRL, VDMA_ACFG_TERM, (EFE_TSE(s->use_intra_in_pframe) |
	              EFE_FMVP(s->use_fast_mvp) |
	              (0/*s->size_mode*/ << 29) |
	              s->raw_format |
	              EFE_X264_QP(s->qp) |
	              EFE_HALN8_FLAG(align8_flag) |
	              EFE_STEP_MODE(s->step_mode) |
	              EFE_DBLK_EN |
	              EFE_SLICE_TYPE(s->frame_type) |
	              EFE_EN | EFE_RUN));
}

void H264E_DumpInfo(_H264E_SliceInfo *s)
{
	printk(" emc_qpt_va              : %p\n", s->emc_qpt_va);
	printk(" emc_rc_va               : %p\n", s->emc_rc_va);
	printk(" emc_cpx_va              : %p\n", s->emc_cpx_va);
	printk(" emc_mod_va              : %p\n", s->emc_mod_va);
	printk(" emc_ncu_va              : %p\n", s->emc_ncu_va);
	printk(" emc_sad_va              : %p\n", s->emc_sad_va);
	printk(" mb_mode_info            : %p\n", s->mb_mode_info);

	printk("s->des_va   = 0x%08x\n", (unsigned int)s->des_va);
	printk("s->des_pa   = 0x%08x\n", (unsigned int)s->des_pa);
	//  printk("s->emc_bs_va = 0x%08x\n", s->emc_bs_va);
	printk("s->emc_bs_pa = 0x%08x\n", s->emc_bs_pa);

	/*x2000 add*/
	printk(" bs_head_en = %d\n", s->bs_head_en);
	printk(" bs_head_va = 0x%08x\n", (unsigned int)s->bs_head_va);
	printk(" bs_head_len = 0x%d\n", (unsigned int)s->bs_head_len);
	printk(" bs_rbsp_en = %d\n", s->bs_rbsp_en);

	printk(" stride[0]    = %d\n", s->stride[0]);
	printk(" stride[1]    = %d\n", s->stride[1]);
	printk(" state	= 0x%08x\n", (unsigned int)s->state);
	printk(" raw[0]   = 0x%08x\n", s->raw[0]);
	printk(" raw[1]   = 0x%08x\n", s->raw[1]);
	printk(" raw[2]   = 0x%08x\n", s->raw[2]);
	printk(" fb[0][0] = 0x%08x\n", s->fb[0][0]);
	printk(" fb[0][1] = 0x%08x\n", s->fb[0][1]);
	printk(" fb[1][0] = 0x%08x\n", s->fb[1][0]);
	printk(" fb[1][1] = 0x%08x\n", s->fb[1][1]);
	printk(" fb[2][0] = 0x%08x\n", s->fb[2][0]);
	printk(" fb[2][1] = 0x%08x\n", s->fb[2][1]);
	printk(" jh[0][0] = 0x%08x\n", s->jh[0][0]);
	printk(" jh[0][1] = 0x%08x\n", s->jh[0][1]);
	printk(" jh[1][0] = 0x%08x\n", s->jh[1][0]);
	printk(" jh[1][1] = 0x%08x\n", s->jh[1][1]);
	printk(" jh[2][0] = 0x%08x\n", s->jh[2][0]);
	printk(" jh[2][1] = 0x%08x\n", s->jh[2][1]);
	printk(" spe_y_addr = 0x%08x\n", s->spe_y_addr);
	printk(" spe_c_addr = 0x%08x\n", s->spe_c_addr);
	printk(" emc_recon_pa= 0x%08x\n", s->emc_recon_pa);
	printk(" emc_qpt_pa  = 0x%08x\n", s->emc_qpt_pa);
	printk(" emc_mv_pa   = 0x%08x\n", s->emc_mv_pa);
	printk(" emc_se_pa   = 0x%08x\n", s->emc_se_pa);
	printk(" emc_rc_pa   = 0x%08x\n", s->emc_rc_pa);
	printk(" emc_cpx_pa  = 0x%08x\n", s->emc_cpx_pa);
	printk(" emc_mod_pa  = 0x%08x\n", s->emc_mod_pa);
	printk(" emc_ncu_pa  = 0x%08x\n", s->emc_ncu_pa);
	printk(" emc_sad_pa  = 0x%08x\n", s->emc_sad_pa);
	/* 1. rc output [11]*/
	{
		printk(" frame_type              : %d\n", s->frame_type);
		printk(" mb_width                : %d\n", s->mb_width);
		printk(" mb_height               : %d\n", s->mb_height);
		printk(" frame_width             : %d\n", s->frame_width);
		printk(" frame_height            : %d\n", s->frame_height);
		printk(" first_mby               : %d\n", s->first_mby);
		printk(" last_mby                : %d\n", s->last_mby);
		printk(" qp                      : %d\n", s->qp);
		printk(" base_qp                 : %d\n", s->base_qp);
		printk(" max_qp                  : %d\n", s->max_qp);
		printk(" min_qp                  : %d\n", s->min_qp);
	}
	/* 2. motion cfg [25]*/
	{
		printk(" frm_re[0]              : %d\n", s->frm_re[0]);
		printk(" frm_re[1]              : %d\n", s->frm_re[1]);
		printk(" frm_re[2]              : %d\n", s->frm_re[2]);
		printk(" frm_re[3]              : %d\n", s->frm_re[3]);
		printk(" pskip_en               : %d\n", s->pskip_en);
		printk(" mref_en                : %d\n", s->mref_en);
		printk(" scl                    : %d\n", s->scl);
		printk(" hpel_en                : %d\n", s->hpel_en);
		printk(" qpel_en                : %d\n", s->qpel_en);
		printk(" ref_mode               : %d\n", s->ref_mode);
		printk(" max_sech_step_i        : %d\n", s->max_sech_step_i);
		printk(" max_mvrx_i             : %d\n", s->max_mvrx_i);
		printk(" max_mvry_i             : %d\n", s->max_mvry_i);
		printk(" lambda_scale_parameter : %d\n", s->lambda_scale_parameter);
		printk(" fs_en                  : %d\n", s->fs_en);
		printk(" fs_md                  : %d\n", s->fs_md);
		printk(" fs_px                  : %d\n", s->fs_px);
		printk(" fs_py                  : %d\n", s->fs_py);
		printk(" fs_rx                  : %d\n", s->fs_rx);
		printk(" fs_ry                  : %d\n", s->fs_ry);
		printk(" frm_mv_en              : %d\n", s->frm_mv_en);
		printk(" frm_mv_size            : %d\n", s->frm_mv_size);
		printk(" glb_mv_en              : %d\n", s->glb_mv_en);
		printk(" glb_mvx                : %d\n", s->glb_mvx);
		printk(" glb_mvy                : %d\n", s->glb_mvy);
		printk(" me_step_en             : %d\n", s->me_step_en);
		printk(" me_step_0              : %d\n", s->me_step_0);
		printk(" me_step_1              : %d\n", s->me_step_1);
	}
	/* 3. quant cfg [4]*/
	{
		int i = 0, j = 0;
		printk(" dct8x8_en    : %d\n", s->dct8x8_en);
		for (i = 0; i < 4; i++)
			for (j = 0; j < 16; j++) {
				printk(" scaling_list[%d][%d] : %d\n", i, j, s->scaling_list[i][j]);
			}
		for (i = 0; i < 2; i++)
			for (j = 0; j < 64; j++) {
				printk(" scaling_list8[%d][%d] : %d\n", i, j, s->scaling_list8[i][j]);
			}
		for (i = 0; i < 9; i++) {
			printk(" deadzone[%d] : %d\n", i, s->deadzone[i]);
		}
	}

	/* 4. loop filter cfg [4]*/
	{
		printk(" deblock                : %d\n", s->deblock);
		printk(" rotate                 : %d\n", s->rotate);
		printk(" alpha_c0_offset        : %d\n", s->alpha_c0_offset);
		printk(" beta_offset            : %d\n", s->beta_offset);
	}
	/* 5. do not douch, default value [56]*/
	{
		printk(" acmask_mode : %d\n", s->acmask_mode);
		printk(" intra_mode_msk : %d\n", s->intra_mode_msk);
		printk(" i_4x4_dis              : %d\n", s->i_4x4_dis);
		printk(" i_8x8_dis              : %d\n", s->i_8x8_dis);
		printk(" i_16x16_dis            : %d\n", s->i_16x16_dis);
		printk(" p_l0_dis               : %d\n", s->p_l0_dis);
		printk(" p_t8_dis               : %d\n", s->p_t8_dis);
		printk(" p_skip_dis             : %d\n", s->p_skip_dis);
		printk(" p_skip_pl0f_dis        : %d\n", s->p_skip_pl0f_dis);
		printk(" p_skip_pt8f_dis        : %d\n", s->p_skip_pt8f_dis);

		printk(" cost_bias_en           : %d\n", s->cost_bias_en);
		printk(" cost_bias_i_4x4        : %d\n", s->cost_bias_i_4x4);
		printk(" cost_bias_i_8x8        : %d\n", s->cost_bias_i_8x8);
		printk(" cost_bias_i_16x16      : %d\n", s->cost_bias_i_16x16);
		printk(" cost_bias_p_l0         : %d\n", s->cost_bias_p_l0);
		printk(" cost_bias_p_t8         : %d\n", s->cost_bias_p_t8);
		printk(" cost_bias_p_skip       : %d\n", s->cost_bias_p_skip);

		printk(" intra_lambda_y_bias_en : %d\n", s->intra_lambda_y_bias_en);
		printk(" intra_lambda_c_bias_en : %d\n", s->intra_lambda_c_bias_en);
		printk(" intra_lambda_bias_qp0  : %d\n", s->intra_lambda_bias_qp0);
		printk(" intra_lambda_bias_qp1  : %d\n", s->intra_lambda_bias_qp1);
		printk(" intra_lambda_bias_0    : %d\n", s->intra_lambda_bias_0);
		printk(" intra_lambda_bias_1    : %d\n", s->intra_lambda_bias_1);
		printk(" intra_lambda_bias_2    : %d\n", s->intra_lambda_bias_2);

		printk(" chroma_sse_bias_en     : %d\n", s->chroma_sse_bias_en);
		printk(" chroma_sse_bias_qp0    : %d\n", s->chroma_sse_bias_qp0);
		printk(" chroma_sse_bias_qp1    : %d\n", s->chroma_sse_bias_qp1);
		printk(" chroma_sse_bias_0      : %d\n", s->chroma_sse_bias_0);
		printk(" chroma_sse_bias_1      : %d\n", s->chroma_sse_bias_1);
		printk(" chroma_sse_bias_2      : %d\n", s->chroma_sse_bias_2);

		printk(" sse_lambda_bias_en     : %d\n", s->sse_lambda_bias_en);
		printk(" sse_lambda_bias        : %d\n", s->sse_lambda_bias);
		printk(" fbc_ep                 : %d\n", s->fbc_ep);
		printk(" jm_lambda2_en          : %d\n", s->jm_lambda2_en);
		printk(" inter_nei_en           : %d\n", s->inter_nei_en);
		printk(" skip_bias_en           : %d\n", s->skip_bias_en);

		printk(" ysse_thr               : %d\n", s->ysse_thr);
		printk(" csse_thr               : %d\n", s->csse_thr);
		printk(" dcm_en                 : %d\n", s->dcm_en);
		printk(" dcm_param              : %d\n", s->dcm_param);
		printk(" sde_prior              : %d\n", s->sde_prior);
		printk(" db_prior               : %d\n", s->db_prior);

		printk(" use_intra_in_pframe    : %d\n", s->use_intra_in_pframe);
		printk(" use_fast_mvp           : %d\n", s->use_fast_mvp);
		printk(" skip_en                : %d\n", s->skip_en);
		printk(" cqp_offset             : %d\n", s->cqp_offset);

		printk(" daisy_chain_en         : %d\n", s->daisy_chain_en);
		printk(" curr_thread_id         : %d\n", s->curr_thread_id);
		printk(" qp_tab_mode            : %d\n", s->qp_tab_mode);
		printk(" bs_size_en             : %d\n", s-> bs_size_en);
		printk(" bs_size                : %d\n", s->bs_size);
		printk(" raw_format             : %d\n", s->raw_format);

		printk(" size_mode              : %d\n", s->size_mode);
		printk(" step_mode              : %d\n", s->step_mode);
		printk(" mode_ctrl              : %d\n", s->mode_ctrl);
		int i = 0;
		for (i = 0; i < 40; i++) {
			printk("mode_ctrl_param[%d] : %d\n", i, s->mode_ctrl_param[i]);
		}

	}
	/* 6. select hardware output mode [11]*/
	{
		printk(" info_en                : %d\n", s->info_en);
		printk(" mvd_sum_all            : %d\n", s->mvd_sum_all);
		printk(" mvd_sum_abs            : %d\n", s->mvd_sum_abs);
		printk(" mv_sum_all             : %d\n", s->mv_sum_all);
		printk(" mv_sum_abs             : %d\n", s->mv_sum_abs);

		printk(" cfg_size_x             : %d\n", s->cfg_size_x);
		printk(" cfg_size_y             : %d\n", s->cfg_size_y);
		printk(" cfg_iw_thr             : %d\n", s->cfg_iw_thr);

		printk(" cfg_mvr_thr1           : %d\n", s->cfg_mvr_thr1);
		printk(" cfg_mvr_thr2           : %d\n", s->cfg_mvr_thr2);
		printk(" cfg_mvr_thr3           : %d\n", s->cfg_mvr_thr3);
	}
	/* 7. ipred bit&lambda ctrl [33]*/
	{
		printk(" mb_mode_val            : %d\n", s->mb_mode_val);
		printk(" bit_16_en              : %d\n", s->bit_16_en);
		printk(" bit_8_en               : %d\n", s->bit_8_en);
		printk(" bit_4_en               : %d\n", s->bit_4_en);
		printk(" bit_uv_en              : %d\n", s->bit_uv_en);
		printk(" lamb_16_en             : %d\n", s->lamb_16_en);
		printk(" lamb_8_en              : %d\n", s->lamb_8_en);
		printk(" lamb_4_en              : %d\n", s->lamb_4_en);
		printk(" lamb_uv_en             : %d\n", s->lamb_uv_en);
		printk(" lamb_uv_en             : %d\n", s->c_16_en);
		printk(" c_8_en                 : %d\n", s->c_8_en);
		printk(" c_4_en                 : %d\n", s->c_4_en);
		printk(" c_uv_en                : %d\n", s->c_uv_en);
		printk(" pri_16                 : %d\n", s->pri_16);
		printk(" pri_8                  : %d\n", s->pri_8);
		printk(" pri_4                  : %d\n", s->pri_4);
		printk(" pri_uv                 : %d\n", s->pri_uv);
		printk(" ref_neb_4              : %d\n", s->ref_neb_4);
		printk(" ref_neb_8              : %d\n", s->ref_neb_8);
		printk(" lambda_info16          : %d\n", s->lambda_info16);
		printk(" lambda_info8           : %d\n", s->lambda_info8);
		printk(" lambda_info4           : %d\n", s->lambda_info4);
		printk(" lambda_infouv          : %d\n", s->lambda_infouv);
		printk(" ref_4                  : %d\n", s->ref_4);
		printk(" ref_8                  : %d\n", s->ref_8);

		int i = 0;
		for (i = 0; i < 4; i++) {
			printk(" bit_16[%d]             : %d\n", i, s->bit_16[i]);
		}
		for (i = 0; i < 4; i++) {
			printk(" bit_uv[%d]             : %d\n", i, s->bit_uv[i]);
		}
		for (i = 0; i < 4; i++) {
			printk(" bit_4[%d]              : %d\n", i, s->bit_4[i]);
		}
		for (i = 0; i < 4; i++) {
			printk(" bit_8[%d]              : %d\n", i, s->bit_8[i]);
		}
		for (i = 0; i < 4; i++) {
			printk(" const_16[%d]           : %d\n", i, s->const_16[i]);
		}
		for (i = 0; i < 4; i++) {
			printk(" const_uv[%d]           : %d\n", i, s->const_uv[i]);
		}
		for (i = 0; i < 4; i++) {
			printk(" const_4[%d]            : %d\n", i, s->const_4[i]);
		}
		for (i = 0; i < 4; i++) {
			printk(" const_8[%d]            : %d\n", i, s->const_8[i]);
		}
	}
	/* 9. jrfc,jrfd [5]*/
	{
		printk(" jrfcd_flag              : %d\n", s->jrfcd_flag);
		printk(" jrfc_enable             : %d\n", s->jrfc_enable);
		printk(" jrfd_enable             : %d\n", s->jrfd_enable);
		printk(" lm_head_total           : %d\n", s->lm_head_total);
		printk(" cm_head_total           : %d\n", s->cm_head_total);
	}
	/* 10. eigen cfg for mosaic, color error [42]*/
	{
		printk(" mb_mode_use             : %d\n", s->mb_mode_use);
		printk(" force_i16dc             : %d\n", s->force_i16dc); //ipred
		printk(" force_i16               : %d\n", s->force_i16);
		printk(" refresh_en              : %d\n", s->refresh_en);
		printk(" refresh_mode            : %d\n", s->refresh_mode);
		printk(" refresh_bias            : %d\n", s->refresh_bias);
		printk(" refresh_cplx_thd        : %d\n", s->refresh_cplx_thd);
		printk(" cplx_thd_sel            : %d\n", s->cplx_thd_sel);
		printk(" diff_cplx_sel           : %d\n", s->diff_cplx_sel);
		printk(" diff_thd_sel            : %d\n", s->diff_thd_sel);
		printk(" i16dc_cplx_thd          : %d\n", s->i16dc_cplx_thd);
		printk(" i16dc_qp_base           : %d\n", s->i16dc_qp_base);
		printk(" i16dc_qp_sel            : %d\n", s->i16dc_qp_sel);
		printk(" i16_qp_base             : %d\n", s->i16_qp_base);
		printk(" i16_qp_sel              : %d\n", s->i16_qp_sel);
		printk(" diff_cplx_thd           : %d\n", s->diff_cplx_thd);

		int i = 0;
		for (i = 0; i < 2; i++) {
			printk(" diff_qp_base[%d]        : %d\n", i, s->diff_qp_base[i]);
		}
		for (i = 0; i < 2; i++) {
			printk(" diff_qp_sel[%d]         : %d\n", i, s->diff_qp_sel[i]);
		}
		for (i = 0; i < 24; i++) {
			printk(" cplx_thd_idx[%d]        : %d\n", i, s->cplx_thd_idx[i]);
		}
		for (i = 0; i < 8; i++) {
			printk(" cplx_thd[%d]            : %d\n", i, s->cplx_thd[i]);
		}
		for (i = 0; i < 3; i++) {
			printk(" diff_thd_base[%d]       : %d\n", i, s->diff_thd_base[i]);
		}
		for (i = 0; i < 3; i++) {
			printk(" diff_thd_ofst[%d]       : %d\n", i, s->diff_thd_ofst[i]);
		}
		printk(" sas_eigen_en            : %d\n", s->sas_eigen_en);
		printk(" crp_eigen_en            : %d\n", s->crp_eigen_en);
		printk(" sas_eigen_dump          : %d\n", s->sas_eigen_dump);
		printk(" crp_eigen_dump          : %d\n", s->crp_eigen_dump);
		printk(" rrs_en                  : %d\n", s->rrs_en);
		printk(" rrs_dump_en             : %d\n", s->rrs_dump_en);
		printk(" rrs_uv_en               : %d\n", s->rrs_uv_en);
		printk(" rrs_size_y              : %d\n", s->rrs_size_y);
		printk(" rrs_size_c              : %d\n", s->rrs_size_c);
		printk(" rrs_thrd_y              : %d\n", s->rrs_thrd_y);
		printk(" rrs_thrd_u              : %d\n", s->rrs_thrd_u);
		printk(" rrs_thrd_v              : %d\n", s->rrs_thrd_v);
	}
	/* 11. skin judge cfg [14]*/
	{
		printk(" skin_dt_en              : %d\n", s->skin_dt_en);
		printk(" skin_lvl                : %d\n", s->skin_lvl);
		printk(" skin_cnt_thd            : %d\n", s->skin_cnt_thd);
		printk(" ncu_mov_en              : %d\n", s->ncu_mov_en);
		printk(" ncu_move_len            : %d\n", s->ncu_move_len);
		if (s->ncu_move_info == NULL) {
			printk(" ncu_move_info is NULL \n");
		} else {
			printk(" ncu_move_info       : %p\n", s->ncu_move_info);
		}
		printk(" buf_share_en            : %d\n", s->buf_share_en);
		printk(" buf_share_size          : %d\n", s->buf_share_size);
		printk(" frame_idx               : %d\n", s->frame_idx);
		printk(" is_first_Pframe         : %d\n", s->is_first_Pframe);
		int i = 0, j = 0;
		for (i = 0; i < 3; i++)
			for (j = 0; j < 2; j++) {
				printk(" skin_pxlu_thd[%d][%d] : %d\n", i, j, s->skin_pxlu_thd[i][j]);
			}
		for (i = 0; i < 3; i++)
			for (j = 0; j < 2; j++) {
				printk(" skin_pxlv_thd[%d][%d] : %d\n", i, j, s->skin_pxlv_thd[i][j]);
			}
		for (i = 0; i < 4; i++) {
			printk(" skin_qp_ofst[%d] : %d\n", i, s->skin_qp_ofst[i]);
		}
		for (i = 0; i < 3; i++) {
			printk(" mult_factor[%d] : %d\n", i, s->mult_factor[i]);
		}
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++) {
				printk(" shift_factor[%d][%d] : %d\n", i, j, s->shift_factor[i][j]);
			}
		for (i = 0; i < 4; i++) {
			printk(" skin_ofst[%d] : %d\n", i, s->skin_ofst[i]);
		}
	}
	/* 12.qpg */
	{
		/* qp map */
		printk(" qp_tab_en               : %d\n", s->qp_tab_en);
		printk(" qp_tab_len              : %d\n", s->qp_tab_len);
		if (s->qp_tab == NULL) {
			printk(" qp_tab is NULL");
		} else {
			printk(" qp_tab : %p\n", s->qp_tab);
		}
		int i = 0;
		for (i = 0; i < 8; i++) {
			printk("roi_en[%d]   : %d\n", i, s->roi_info[i].roi_en);
			printk("roi_md[%d]   : %d\n", i, s->roi_info[i].roi_md);
			printk("roi_qp[%d]   : %d\n", i, s->roi_info[i].roi_qp);
			printk("roi_lmbx[%d] : %d\n", i, s->roi_info[i].roi_lmbx);
			printk("roi_rmbx[%d] : %d\n", i, s->roi_info[i].roi_rmbx);
			printk("roi_umby[%d] : %d\n", i, s->roi_info[i].roi_umby);
			printk("roi_bmby[%d] : %d\n", i, s->roi_info[i].roi_bmby);
		}
		/* qp cplx */
		printk(" sas_en                  : %d\n", s->sas_en);
		printk(" crp_en                  : %d\n", s->crp_en);
		printk(" sas_mthd                : %d\n", s->sas_mthd);
		for (i = 0; i < 7; i++) {
			printk(" qpg_mb_thd[%d] : %d\n", i, s->qpg_mb_thd[i]);
		}
		for (i = 0; i < 8; i++) {
			printk(" qpg_mbqp_ofst[%d] : %d\n", i, s->qpg_mbqp_ofst[i]);
		}
		for (i = 0; i < 5; i++) {
			printk(" qpg_flt_thd[%d] : %d\n", i, s->qpg_flt_thd[i]);
		}
		printk(" mbrc_qpg_sel            : %d\n", s->mbrc_qpg_sel);
		/* bu rc */
		printk(" rc_mb_en                : %d\n", s->rc_mb_en);
		printk(" rc_bu_wait_en           : %d\n", s->rc_bu_wait_en);
		printk(" rc_mb_wait_en           : %d\n", s->rc_mb_wait_en);
		printk(" rc_bu_num               : %d\n", s->rc_bu_num);
		printk(" rc_bu_size              : %d\n", s->rc_bu_size);
		printk(" rc_bu_level             : %d\n", s->rc_bu_level);
		printk(" rc_mb_level             : %d\n", s->rc_mb_level);
		if (s->mb_ref_info == NULL) {
			printk(" mb_ref_info is NULL");
		} else {
			printk(" mb_ref_info : %p\n", s->mb_ref_info);
		}
		if (s->bu_ref_info == NULL) {
			printk(" bu_ref_info is NULL");
		} else {
			printk(" bu_ref_info : %p\n", s->bu_ref_info);
		}
		printk(" rc_frm_tbs              : %d\n", s->rc_frm_tbs);
		printk(" avg_bu_bs               : %d\n", s->avg_bu_bs);
		for (i = 0; i < 6; i++) {
			printk(" tar_bs_thd[%d] : %d\n", i, s->tar_bs_thd[i]);
		}
		for (i = 0; i < 6; i++) {
			printk(" tar_bs_thd[%d] : %d\n", i, s->bu_alg0_qpo[i]);
		}
		for (i = 0; i < 6; i++) {
			printk(" bu_alg1_qpo[%d] : %d\n", i, s->bu_alg1_qpo[i]);
		}
		for (i = 0; i < 7; i++) {
			printk(" mb_cs_qpo[%d] : %d\n", i, s->mb_cs_qpo[i]);
		}
		for (i = 0; i < 2; i++) {
			printk(" mb_top_bs_qpo[%d] : %d\n", i, s->mb_top_bs_qpo[i]);
		}
		for (i = 0; i < 2; i++) {
			printk(" mb_rinfo_qpo[%d] : %d\n", i, s->mb_rinfo_qpo[i]);
		}
		for (i = 0; i < 2; i++) {
			printk(" mb_target_avg_bs[%d] : %d\n", i, s->mb_target_avg_bs[i]);
		}
		printk(" mb_gp_num               : %d\n", s->mb_gp_num);
		printk(" last_bu_size            : %d\n", s->last_bu_size);
		printk(" rc_bcfg_mode            : %d\n", s->rc_bcfg_mode);
	}

}
