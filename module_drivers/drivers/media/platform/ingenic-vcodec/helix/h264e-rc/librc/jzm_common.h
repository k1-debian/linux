#ifndef __JZM_SHARE_H__
#define __JZM_SHARE_H__

#ifndef __KERNEL__
	#include <stdint.h>
#else
	#include <linux/types.h>
#endif

typedef enum {
	SLICE_TYPE_B   = 1,
	SLICE_TYPE_P   = 0,
	SLICE_TYPE_IDR = 2,
	SLICE_TYPE_VI  = 6,
} RC_FRM_TYPE_E;

typedef enum {
	ENC_H264 = 0,
	ENC_H265 = 1
} RC_PROTOCOL_E;

typedef enum {
	CQP   = 0,
	CBR   = 1,
	VBR   = 2,
	SMART = 3
} RC_MODE_E;

typedef enum {
	SCE_NORMAL     = 0,
	SCE_STILL      = 1,
	SCE_MOVETOSTILL = 2,
	SCE_STILLTOMOVE = 3,
	SCE_MOVE       = 4,
	SCE_SHARPMOVE  = 5,
	SCE_SCENESWITCH = 6,
	SCE_INIT        = 7
} RC_SCENE_TYPE_E;

typedef enum {
	SCENE_0 = 0,
	SCENE_1 = 1,
	SCENE_2 = 2,
} RC_SCENE_MODE_E;

typedef enum {
	GMODE_N1X       = 0,
	GMODE_N2X       = 1,
	GMODE_N4X       = 2,
	GMODE_HN1_FALSE = 3,
	GMODE_HN1_TRUE  = 4,
	GMODE_H1M_FALSE = 5,
	GMODE_H1M_TRUE  = 6
} RC_FRAME_SKIP_MODE_E;

typedef enum {
	NORMALP = 0,
	SMARTP  = 1,
	DOULP   = 2,
	BIPREDP = 3
} RC_GOP_MODE_E;

typedef enum {
	FI_KEY     = 0,
	FI_LBASE   = 1,
	FI_SBASE   = 2,
	FI_ENHANCE = 3
} RC_FRM_IMP_TYPE_E;

typedef enum jzRC_INTEL_S {
	INTEL_CLOSE = 0,
	INTEL_OPEN  = 1,
	INTEL_AUTO  = 2,

} RC_INTEL_S;
typedef enum {
	SUPERFRM_NONE     = 0,
	SUPERFRM_DISCARD  = 1,
	SUPERFRM_REENCODE = 2,
	SUPERFRM_BUTT     = 3
} RC_SUPERFRM_MODE_E;

typedef enum {
	FRMLOST_NORMAL = 0,
	FRMLOST_PSKIP  = 1,
	FRMLOST_BUTT   = 2
} RC_FRMLOST_MODE_E;

typedef enum {
	PRIORITY_BITRATE_FIRST   = 1,
	PRIORITY_FRAMEBITS_FIRST = 2,
	PRIORITY_BUTT            = 3
} RC_PRIORITY_E;

typedef struct jzRC_FRMLOST_CFG_S {
	int8_t             bFrmLostEn;
	RC_FRMLOST_MODE_E  frmLostMode;
	uint32_t           u32FrmLostBpsThr;
	uint32_t           u32FrmLostGaps;
} RC_FRMLOST_CFG_S;

typedef struct jzRC_SUPERFRM_CFG_S {
	RC_SUPERFRM_MODE_E superFrmMode;
	uint32_t           u32SuperIFrmBitsThr;
	uint32_t           u32SuperPFrmBitsThr;
	uint32_t           u32SuperBFrmBitsThr;
} RC_SUPERFRM_CFG_S;

typedef struct jzRC_GOP_NORMALP_S {
	int8_t     s8IPQpDelta;
} RC_GOP_NORMALP_S;

typedef struct jzRC_GOP_SMARTP_S {
	uint32_t   u32BgInterval;
	int8_t     s8BgQpDelta;
	int8_t     s8ViQpDelta;
} RC_GOP_SMARTP_S;

typedef struct jzRC_GOP_DOULP_S {
	uint32_t   u32SPInterval;
	int8_t     s8SPQpDelta;
	int8_t     s8IPQpDelta;
} RC_GOP_DOULP_S;

typedef struct jzRC_GOP_BIPREDP_S {
	uint32_t   u32BFrmNum;
	int8_t     s8BQpDelta;
	int8_t     s8IPQpDelta;
} RC_GOP_BIPREDP_S;

typedef struct jzRC_NCU_INFO_S {
	uint32_t   yWidth;
	uint32_t   yHeight;
	uint32_t   yfrmNum;
	uint32_t   yBlkSize;
	uint32_t   yBufSize;
	uint32_t   yStride;
	uint8_t    *yBuf;
} RC_NCU_INFO_S;

typedef struct jzRC_AE_ZONE_INFO_S {
	unsigned int zone[15][15];
} RC_AE_ZONE_INFO_S;

typedef struct mb_eigen_s {
	uint8_t scene_info; //[0]: 1, move; 0, still; [1]: 1, simple; 0, complex.
} mb_eigen_s;
#endif //__JZM_SHARE_H__
