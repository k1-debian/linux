#ifndef __T21_JZM_ENC_API_H__
#define __T21_JZM_ENC_API_H__

#ifndef __KERNEL__
	#include <stdint.h>
#endif

#include "jzm_common.h"
#include "../../api/helix_x264_enc.h"
//#include "enc_api_t21H264.h"
#include "enc_api_t21H265.h"

typedef uint8_t  JZ_U8;
typedef uint16_t JZ_U16;
typedef uint32_t JZ_U32;
typedef int8_t   JZ_S8;
typedef int16_t  JZ_S16;
typedef int32_t  JZ_S32;
typedef float    JZ_FL;
typedef int      JZ_BL;
typedef uint32_t JZ_FR;

typedef struct jzRC_ATTR_H264_CQP_S {
	JZ_U32                   u32Gop;
	JZ_U32                   u32SrcFrmRate;
	JZ_FR                    frDstFrmRate;
	JZ_U8                    u8IQp;
	JZ_U8                    u8PQp;
	JZ_U8                    u8BQp;
} RC_ATTR_H264_CQP_S;

typedef struct jzRC_ATTR_H264_CBR_S {
	JZ_U32                   u32Gop;
	JZ_U32                   u32StatTime;
	JZ_U32                   u32SrcFrmRate;
	JZ_FR                    frDstFrmRate;
	JZ_U32                   u32BitRate;
	JZ_U8                    u8FluctuateLevel;
	//JZ_U8                    u8StreamLevel;
} RC_ATTR_H264_CBR_S;

typedef struct jzRC_PARAM_H264_CBR_S {
	JZ_U8                    u8MaxIprop;
	JZ_U8                    u8MinIprop;
	JZ_U8                    u8MaxQp;
	JZ_U8                    u8MinQp;
	JZ_U8                    u8MaxIQp;
	JZ_U8                    u8MinIQp;
	JZ_S8                    s8IPQpDelta;
	JZ_U8                    u8QualityLvl;
	JZ_U8                    u8MaxReEncodeTimes;
	JZ_U8                    u8MaxIPQpDelta;
	JZ_U8                    u8MaxPPQpDelta;
	JZ_S8                    s8IQpBias;
} RC_PARAM_H264_CBR_S;

typedef struct jzRC_ATTR_H264_VBR_S {
	JZ_U32                   u32Gop;
	JZ_U32                   u32StatTime;
	JZ_U32                   u32SrcFrmRate;
	JZ_FR                    frDstFrmRate;
	JZ_U32                   u32MaxBitRate;
	JZ_U8                    u8MaxQp;
	JZ_U8                    u8MinQp;
	JZ_U8                    u8MinIQp;
	JZ_S8                    s8AvbrEn;
	//JZ_U8                    u8StreamLevel;
} RC_ATTR_H264_VBR_S;

typedef struct jzRC_PARAM_H264_VBR_S {
	JZ_U8                    u8ChangePos;
	JZ_U8                    u8MaxIprop;
	JZ_U8                    u8MinIprop;
	JZ_U8                    u8MaxReEncodeTimes;
	JZ_S8                    s8IPQpDelta;
	JZ_U8                    u8MaxIPQpDelta;
	JZ_U8                    u8MaxPPQpDelta;
	JZ_U8                    u8MinIQp;
	JZ_U8                    u8QualityLvl;
	JZ_S8                    s8RQDelta;
	JZ_S8                    s8IQpBias;
} RC_PARAM_H264_VBR_S;

typedef  RC_ATTR_H264_CQP_S    RC_ATTR_H265_CQP_S;
typedef  RC_ATTR_H264_CBR_S    RC_ATTR_H265_CBR_S;
typedef  RC_PARAM_H264_CBR_S   RC_PARAM_H265_CBR_S;
typedef  RC_ATTR_H264_VBR_S    RC_ATTR_H265_VBR_S;
typedef  RC_PARAM_H264_VBR_S   RC_PARAM_H265_VBR_S;

typedef struct jzRC_INSIDE_S {
	uint8_t IQpMode;
	uint8_t fstPQpMode;
	uint8_t avgPQpMode;
	uint8_t qpModel;
	uint8_t bitrLvlEn;
	uint8_t sceJudgeLvl;
	uint8_t sceJudgeEn;
	uint8_t useFstPEn;
	uint8_t avgQpNewEn;
	uint8_t avgPQpNum;
	uint8_t qpFluctEn;
	uint8_t qpFluctLvl;
	uint8_t fluctLvl;
	uint8_t bndEn;
	uint8_t BndLvl;
	uint8_t stmLvl;
	uint8_t t0Method;
	uint8_t avgPBitsNum;
	uint8_t gammaEn;
	uint8_t stillSize;
	uint8_t moveSize;
	int32_t qpgIMode;
	int32_t qpgPMode;
} RC_INSIDE_S;

typedef struct jzSI_PARAM_H264_S {
	/* motion */
	uint8_t frm_re0A;            /* BOOL (0) */
	uint8_t frm_re1A;            /* idem */
	uint8_t frm_re2A;            /* idem */
	uint8_t frm_re3A;            /* idem */

	uint8_t scl;                 /* 1~4 (3) */
	uint8_t hpel_en;             /* BOOL (1) */
	uint8_t qpel_en;             /* idem */
	uint32_t max_sech_step_i;    /* 0~63 (63) */
	uint32_t max_mvrx_i;         /* 1~511 (255) */
	uint32_t max_mvry_i;         /* idem */

	uint8_t fs_en;               /* BOOL */
	uint32_t fs_md;              /* BOOL */
	uint8_t fs_px;
	uint8_t fs_py;
	uint8_t fs_rx;
	uint8_t fs_ry;

	uint8_t frm_mv_en;           /* BOOL (1) */
	uint8_t frm_mv_size;         /* 0~5 */

	uint8_t glb_mv_en;           /* BOOL (1) */

	uint8_t me_step_en;          /* BOOL (1) */
	uint8_t me_step_0;           /* 1~63 (8) */
	uint8_t me_step_1;           /* 1~63 (10) */

	/* transform & quant & loop filter */
	int deadzone0Aiy4;           /* 0~63 (0x15) iy4 */
	int deadzone1Apy4;           /* 0~63 (0xb)  py4 */
	int deadzone2Aic4;           /* 0~63 (0x15) ic4 */
	int deadzone3Apc4;           /* 0~63 (0xb)  pc4 */
	int deadzone4Aiy8;           /* 0~63 (0x15) iy8 */
	int deadzone5Apy8;           /* 0~63 (0xb)  py8 */
	int deadzone6Aydc;           /* 0~63 (0x15) ydc */
	int deadzone7Aicdc;          /* 0~63 (0x15) icdc */
	int deadzone8Apcdc;          /* 0~63 (0xb)  pcdc */

	uint32_t acmask_mode;

	uint8_t deblock;             /* BOOL (1) */
	int8_t alpha_c0_offset;      /* -12~12even (0) */
	int8_t beta_offset;          /* idem */

	/* mode decision */
	uint8_t jm_lambda2_en;       /* BOOL (0) */
	uint8_t inter_nei_en;        /* BOOL (0) */
	uint8_t skip_bias_en;        /* 0~3 (0) */

	uint8_t  dcm_en;             /* BOOL (0) */
	uint32_t dcm_param;          /* (0x4304) */

	uint8_t use_intra_in_pframe; /* BOOL (1) */
	int8_t cqp_offset;           /* -16~15 (0) */

	uint8_t     mb_mode_useMASK; /* BOOL, for MASK, 0 will turn off the func. */

	/* eigen enable bool */
	uint8_t     sas_eigen_en;
	uint8_t     crp_eigen_en;

	/* refresh */
	uint8_t     refresh_enMASK;   /* BOOL */
	uint8_t     refresh_mode;     /* BOOL (0) */
	uint8_t     refresh_bias;     /* 0x0~0xf */
	uint8_t     refresh_cplx_thd; /* 0~255 (4) */

	/* force i16dc */
	uint8_t     force_i16dc;        /* BOOL */
	uint8_t     i16dc_cplx_thdIDR;  /* 0~255 (6 for IDR) */
	uint8_t     i16dc_cplx_thdELSE; /* 0~255 (3 for ELSE) */
	uint8_t     i16dc_qp_base;      /* 0~51 (33) */
	uint8_t     i16dc_qp_sel;       /* BOOL (0) */

	/* force i16 */
	uint8_t     force_i16;       /* BOOL */
	uint8_t     i16_qp_baseIDR;  /* 0~51 (25 for IDR) */
	uint8_t     i16_qp_baseELSE; /* 0~51 (28 for ELSE) */
	uint8_t     i16_qp_sel;      /* BOOL */

	/* rrs */
	uint8_t     rrs_enMASK;      /* BOOL */
	uint8_t     rrs_uv_enMASK;   /* BOOL */
	uint8_t     rrs_size_y;      /* 0~3 (3) */
	uint8_t     rrs_size_c;      /* 0~1 (1) */

	/* skin detection */
	uint8_t     skin_dt_en;      /* BOOL */
	uint8_t     skin_lvl;        /* 0~3 (0) */
	uint8_t     skin_cnt_thd;    /* 0~64 (20) */
} SI_PARAM_H264_S;

typedef struct jzRC_CFG_S {
	JZ_U16                   u16FrmWidth;               /* input  current frame size in width */
	JZ_U16                   u16FrmHeight;              /* input  current frame size in height */
	JZ_U16                   u16MbWidth;                /* input  current frame size in width */
	JZ_U16                   u16MbHeight;               /* input  current frame size in height */
	RC_MODE_E                rcMode;                    /* rate control mode */
	RC_PROTOCOL_E            encProtocol;
	RC_GOP_MODE_E            gopMode;
	RC_FRAME_SKIP_MODE_E     fskMode;                   /* input skip type */
	RC_PRIORITY_E            priority;
	JZ_U32                   qpgMode;
	RC_INTEL_S               intelMode;
	RC_SCENE_MODE_E          sceMode;
	JZ_S8                    s8StartQp;
	union {
		RC_ATTR_H264_CQP_S      attrH264Cqp;            /* CQP mode attribute parameter */
		RC_ATTR_H264_CBR_S      attrH264Cbr;            /* CBR mode attribut parametr */
		RC_ATTR_H264_VBR_S      attrH264Vbr;            /* VBR mode attribut parametr */
		RC_ATTR_H265_CQP_S      attrH265Cqp;            /* CQP mode attribute parameter */
		RC_ATTR_H265_CBR_S      attrH265Cbr;            /* CBR mode attribut parametr */
		RC_ATTR_H265_VBR_S      attrH265Vbr;            /* VBR mode attribut parametr */
	};
	union {
		RC_PARAM_H264_CBR_S     paramH264Cbr;           /* CBR mode high parametr */
		RC_PARAM_H264_VBR_S     paramH264Vbr;           /* VBR mode high parametr */
		RC_PARAM_H265_CBR_S     paramH265Cbr;           /* CBR mode high parametr */
		RC_PARAM_H265_VBR_S     paramH265Vbr;           /* VBR mode high parametr */
	};
	RC_FRMLOST_CFG_S         lostFrmCfg;
	RC_SUPERFRM_CFG_S        superFrmCfg;
	RC_GOP_NORMALP_S         gopNormalP;
	RC_GOP_SMARTP_S          gopSmartP;
	RC_GOP_DOULP_S           gopDoulP;
	RC_GOP_BIPREDP_S         gopBipredP;
	RC_INSIDE_S              rcInside;
	SI_PARAM_H264_S          siParamH264;
} RC_CFG_S;

struct frm_link_container {
	uint8_t magic_flag;

	/* addr. */
	uint32_t *p_mode;
	uint32_t *p_sad;
	uint32_t *p_cplx;

	/* blk level */
	uint8_t cfg_blk_wid;
	uint8_t cfg_blk_hei;
	uint16_t cfg_blk_i_thr;
	uint16_t cfg_blk_mvr_thr1;
	uint32_t cfg_blk_mvr_thr2;
	uint32_t cfg_blk_mvr_thr3;

	uint32_t blk_mvr0;
	uint32_t blk_mvr1;
	uint32_t blk_mvr2;
	uint32_t blk_mvr3;
	uint32_t blk_mvr4;
	uint32_t blk_mvr5;
	uint32_t blk_mvr6;
	uint32_t blk_mvr7;

	uint32_t blk_iw0;
	uint32_t blk_iw1;
	uint32_t blk_iw2;
	uint32_t blk_iw3;

	/* frm level */
	uint8_t cfg_info_en;
	uint8_t cfg_mvd_all;
	uint8_t cfg_mvd_abs;
	uint8_t cfg_mv_all;
	uint8_t cfg_mv_abs;
	uint32_t cfg_ssey_thr;
	uint32_t cfg_ssec_thr;

	uint32_t frm_sady;
	uint32_t frm_sadc;
	uint32_t frm_ssey0;
	uint32_t frm_ssey1;
	uint32_t frm_ssec0;
	uint32_t frm_ssec1;
	uint16_t frm_i16;
	uint16_t frm_i4;
	uint16_t frm_pl0;
	uint16_t frm_pskip;
	uint16_t frm_i8;
	uint16_t frm_pt8;
	uint32_t frm_mvx;
	uint32_t frm_mvy;
	uint32_t frm_mvdx;
	uint32_t frm_mvdy;
	uint16_t frm_ssey_mbs;
	uint16_t frm_ssec_mbs;
};

typedef struct jzVPU_RC_S {
	RC_CFG_S                 cfg;
	void                     *RCptr0;
	void                     *RCptr1;
	void                     *prevRaw;
	/* rc input */
	RC_FRM_IMP_TYPE_E        frmSkipType;               /* gop mode */
	JZ_BL                    bIFrmReq;                  /* whether require I frame */
	JZ_U32                   u32FrmActBs;               /* current frame actual encode bits */
	JZ_U32                   u32FrmMad;                 /* previous frame mad */
	JZ_U32                   u32FrmSse;                 /* previous frame sse */
	JZ_U32                   u32MvxSum;
	JZ_U32                   u32MvySum;
	JZ_U32                   u32MvCnt;
	JZ_U32                   u32MvSum;
	JZ_U32                   u32FrmI16Cnt;                /* previous frame mad */
	JZ_U32                   u32FrmPnsCnt;              /* previous frame mad */
	JZ_U32                   u32FrmPsCnt;               /* previous frame mad */
	JZ_U32                   u32FrmMvrRes[8];
	JZ_U16                   u16FrmIwRes[8];
	JZ_U32                   rgnc[4];
	JZ_U32                   u32CrpRgnc[5][8];            /* crp */
	JZ_U32                   u32SasCnt[5][3];             /* sas */
	JZ_S8                    ncuEn;                     /* ncu info */
	JZ_S8                    aezoneEn;
	RC_NCU_INFO_S            ncuInfo;
	RC_AE_ZONE_INFO_S        ae_zone;
	JZ_U8                    *fenc[2];
	JZ_U32                   stride[2];
	/* rc output */
	JZ_U32                   u32FrmCnt;
	RC_FRM_TYPE_E            frmType;
	RC_SCENE_TYPE_E          curFrmSce;
	JZ_U8                    u8FrmQp;                   /* current frame qp */
	JZ_U32                   u32FrmTgtBs;               /* current frame target bits */
	JZ_S8                    ChangeRef;
	union {
		//H264E_SliceInfo_t *h264SliceInfo;                  /* vpu api */
		H264E_SliceInfo_t *h264SliceInfo;                  /* vpu api */
		H265E_SLICE_INFO_S_T21 *h265SliceInfo;                  /* vpu api */
	};
	JZ_U32                   *vpuPri;
	JZ_U8                    *rcTtlAlloc;
	JZ_S32                   rcTtlSize;
	JZ_U8                    *u8StatQp;
	JZ_S32                   *s32StatBits;
	JZ_U8                    *u8StatSce;
	mb_eigen_s               *mbEigen;
	struct frm_link_container frmLink;
} VPU_RC_S;
void eprc_default_set_T21(VPU_RC_S *rc);
void JZ_VPU_RC_VIDEO_CFG_T21(VPU_RC_S *rc);
void JZ_VPU_RC_FRAME_START_T21(VPU_RC_S *rc);
void JZ_VPU_RC_FRAME_END_T21(VPU_RC_S *rc);
int  JZ_VPU_RC_FRAME_REPEATE_JUDGE_T21(VPU_RC_S *rc);
void JZ_VPU_RC_FREE_T21(VPU_RC_S *rc);

#define eprc_default_set eprc_default_set_T21
#define JZ_VPU_RC_VIDEO_CFG JZ_VPU_RC_VIDEO_CFG_T21
#define JZ_VPU_RC_FRAME_START   JZ_VPU_RC_FRAME_START_T21
#define JZ_VPU_RC_FRAME_END JZ_VPU_RC_FRAME_END_T21
#define JZ_VPU_RC_FRAME_REPEATE_JUDGE   JZ_VPU_RC_FRAME_REPEATE_JUDGE_T21
#define JZ_VPU_RC_FREE      JZ_VPU_RC_FREE_T21

#endif //__T21_JZM_ENC_API_H__
