/****************************************************************
*****************************************************************/
#ifndef __HELIX_H__
#define __HELIX_H__

/****************************************************************
  VPU register map
*****************************************************************/
#define JZM_V2_TLB

#ifdef JZM_HUNT_SIM
	#include "hunt.h"
#else
	#ifndef __place_k0_data__
		#define __place_k0_data__
	#endif
	#ifndef __place_k0_text__
		#define __place_k0_text__
	#endif
#endif

#define VPU_BASE             0x13200000

#define HID_SCH              0x0
#define HID_JRFD         0x1
#define HID_EMC              0x3
#define HID_EFE              0x4
#define HID_MCE          0x5
#define HID_ODMA             0x6
#define HID_DBLK         0x7
#define HID_VMAU         0x8
#define HID_SDE          0x9
#define HID_AUX          0xA
#define HID_TCSM         0xB
#define HID_IFA              0xB //fixme
#define HID_JPGC         0xE
#define HID_SRAM             0xF

#define VPU_MAX_MB_WIDTH     256

#define MSCOPE_START(mbnum)  write_reg(VPU_BASE+0x24, mbnum)
#define MSCOPE_STOP()        write_reg(VPU_BASE+0x28, 0)

/********************************************
  SCH (Scheduler)
*********************************************/
#define TCSM_FLUSH           0xc0000
#define REG_SCH_GLBC         0x00000
#define SCH_GLBC_SLDE        (0x1<<31)
#ifdef JZM_V2_TLB
	#define SCH_TLBE_JPGC       (0x1<<26)
	#define SCH_TLBE_DBLK       (0x1<<25)
	#define SCH_TLBE_SDE        (0x1<<24)
	#define SCH_TLBE_EFE        (0x1<<23)
	#define SCH_TLBE_VDMA       (0x1<<22)
	#define SCH_TLBE_MCE        (0x1<<21)
#else
	#define SCH_GLBC_TLBE       (0x1<<30)
	#define SCH_GLBC_TLBINV     (0x1<<29)
#endif
#define SCH_INTE_RESERR      (0x1<<29)
#define SCH_INTE_BSFULL      (0x1<<28)
#define SCH_INTE_ACFGERR     (0x1<<20)
#define SCH_INTE_TLBERR      (0x1<<18)
#define SCH_INTE_BSERR       (0x1<<17)
#define SCH_INTE_ENDF        (0x1<<16)
#define SCH_GLBC_HIMAP       (0x1<<15)
#define SCH_GLBC_HIAXI       (0x1<<9)
#define SCH_GLBC_EPRI0       (0x0<<7)
#define SCH_GLBC_EPRI1       (0x1<<7)
#define SCH_GLBC_EPRI2       (0x2<<7)
#define SCH_GLBC_EPRI3       (0x3<<7)

#define REG_SCH_TLBA         0x00030

#ifdef JZM_V2_TLB
	#define REG_SCH_TLBC        0x00050
	#define SCH_TLBC_VPN        (0xFFFFF000)
	#define SCH_TLBC_RIDX(idx)  (((idx) & 0xFF)<<4)
	#define SCH_TLBC_INVLD      (0x1<<1)
	#define SCH_TLBC_RETRY      (0x1<<0)

	#define REG_SCH_TLBV        0x00054
	#define SCH_TLBV_CNM(cnm)   (((cnm) & 0xFFF)<<16)
	#define SCH_TLBV_GCN(gcn)   (((gcn) & 0xFFF)<<0)
	#define SCH_TLBV_RCI_MC     (0x1<<30)
	#define SCH_TLBV_RCI_EFE    (0x1<<31)
#endif

#define REG_SCH_STAT         0x00034
#define SCH_STAT_ENDF    (0x1<<0)
#define SCH_STAT_BPF     (0x1<<1)
#define SCH_STAT_ACFGERR (0x1<<2)
#define SCH_STAT_TIMEOUT (0x1<<3)
#define SCH_STAT_JPGEND  (0x1<<4)
#define SCH_STAT_BSERR   (0x1<<7)
#define SCH_STAT_TLBERR  (0x1F<<10)
#define SCH_STAT_SLDERR  (0x1<<16)

#define REG_SCH_SLDE0        0x00040
#define REG_SCH_SLDE1        0x00044
#define REG_SCH_SLDE2        0x00048
#define REG_SCH_SLDE3        0x0004C
#define SCH_SLD_VTAG(val)    (((val) & 0xFFF)<<20)
#define SCH_SLD_MASK(val)    (((val) & 0xFFF)<<8)
#define SCH_SLD_VLD          (0x1<<0)

#define REG_SCH_SCHC         0x00060
#define SCH_CH1_PCH(ch)      (((ch) & 0x3)<<0)
#define SCH_CH2_PCH(ch)      (((ch) & 0x3)<<8)
#define SCH_CH3_PCH(ch)      (((ch) & 0x3)<<16)
#define SCH_CH4_PCH(ch)      (((ch) & 0x3)<<24)
#define SCH_CH1_PE           (0x1<<2)
#define SCH_CH2_PE           (0x1<<10)
#define SCH_CH3_PE           (0x1<<18)
#define SCH_CH4_PE           (0x1<<26)
#define SCH_CH1_GS0          (0x0<<3)
#define SCH_CH1_GS1          (0x1<<3)
#define SCH_CH2_GS0          (0x0<<11)
#define SCH_CH2_GS1          (0x1<<11)
#define SCH_CH3_GS0          (0x0<<19)
#define SCH_CH3_GS1          (0x1<<19)
#define SCH_CH4_GS0          (0x0<<27)
#define SCH_CH4_GS1          (0x1<<27)

#define REG_SCH_BND          0x00064
#define SCH_CH1_HID(hid)     (((hid) & 0xF)<<16)
#define SCH_CH2_HID(hid)     (((hid) & 0xF)<<20)
#define SCH_CH3_HID(hid)     (((hid) & 0xF)<<24)
#define SCH_CH4_HID(hid)     (((hid) & 0xF)<<28)
#define SCH_BND_G0F1         (0x1<<0)
#define SCH_BND_G0F2         (0x1<<1)
#define SCH_BND_G0F3         (0x1<<2)
#define SCH_BND_G0F4         (0x1<<3)
#define SCH_BND_G1F1         (0x1<<4)
#define SCH_BND_G1F2         (0x1<<5)
#define SCH_BND_G1F3         (0x1<<6)
#define SCH_BND_G1F4         (0x1<<7)
#define SCH_DEPTH(val)       (((val-1) & 0xF)<<8)

#define REG_SCH_SCHG0        0x00068
#define REG_SCH_SCHG1        0x0006C
#define REG_SCH_SCHE1        0x00070
#define REG_SCH_SCHE2        0x00074
#define REG_SCH_SCHE3        0x00078
#define REG_SCH_SCHE4        0x0007C

#define DSA_SCH_CH1          (VPU_BASE | REG_SCH_SCHE1)
#define DSA_SCH_CH2          (VPU_BASE | REG_SCH_SCHE2)
#define DSA_SCH_CH3          (VPU_BASE | REG_SCH_SCHE3)
#define DSA_SCH_CH4          (VPU_BASE | REG_SCH_SCHE4)

/********************************************
  SW_RESET (VPU software reset)
*********************************************/
#define REG_CFGC_SW_RESET    0x00004
#define CFGC_SW_RESET_RST         (0x1<<1)
#define CFGC_SW_RESET_RST_CLR     (0x0<<1)
#define CFGC_SW_RESET_EARB_EMPT   (0x4)
/********************************************
  VDMA (VPU general-purpose DMA)
*********************************************/
#define REG_VDMA_LOCK        0x10000
#define REG_VDMA_UNLK        0x10004

#define REG_VDMA_TASKRG      0x10008
#define VDMA_ACFG_RUN        (0x1)
#define VDMA_DESC_RUN        (0x3)
#define VDMA_ACFG_CLR        (0x8)
#define VDMA_ACFG_SAFE       (0x4)
#define VDMA_ACFG_DHA(a)     (((unsigned int)(a)) & 0xFFFFFF80)
#define VDMA_DESC_DHA(a)     (((unsigned int)(a)) & 0xFFFF0)

#define REG_CFGC_ACM_CTRL    0x00084
#define REG_CFGC_ACM_STAT    0x00088
#define REG_CFGC_ACM_DHA     0x0008C

#define REG_VDMA_TASKST      0x1000C
#define VDMA_ACFG_ERR        (0x1<<3)
#define VDMA_ACFG_END        (0x1<<2)
#define VDMA_DESC_END        (0x1<<1)
#define VDMA_VPU_BUSY        (0x1<<0)

#define VDMA_DESC_EXTSEL     (0x1<<0)
#define VDMA_DESC_TLBSEL     (0x1<<1)
#define VDMA_DESC_LK         (0x1<<31)

#define VDMA_ACFG_VLD        (0x1<<31)
#define VDMA_ACFG_TERM       (0x1<<30)
#define VDMA_ACFG_IDX(a)     (((unsigned int)(a)) & 0xFFFFC)

#ifdef RW_REG_TEST
#define GEN_VDMA_ACFG(chn, reg, term, val)  write_reg(VPU_BASE+(reg), val)
#else
#define GEN_VDMA_ACFG(chn, reg, term, val)          \
	({*chn++ = val;                       \
		*chn++ = (VDMA_ACFG_VLD | (term) | VDMA_ACFG_IDX(reg)); \
	})
#endif

#define REG_EMC_FRM_SIZE     0x30000
#define REG_EMC_BS_ADDR      0x30004
#define REG_EMC_DBLK_ADDR    0x30008
#define REG_EMC_RECON_ADDR   0x3000c
#define REG_EMC_MV_ADDR      0x30010
#define REG_EMC_SE_ADDR      0x30014
#define REG_EMC_QPT_ADDR     0x30018
#define REG_EMC_RC_RADDR     0x3001c
#define REG_EMC_MOS_ADDR     0x30020
#define REG_EMC_SLV_INIT     0x30024
#define REG_EMC_DEBUG_INFO0  0x30028
#define REG_EMC_DEBUG_INFO1  0x3002c
#define REG_EMC_CRC_INFO0    0x30030
#define REG_EMC_CRC_INFO1    0x30034
#define REG_EMC_CRC_INFO2    0x30038
#define REG_EMC_CRC_INFO3    0x3003c
#define REG_EMC_BS_SIZE      0x30040
#define REG_EMC_BS_STAT      0x30044
#define REG_EMC_RC_WADDR     0x30048
#define REG_EMC_CPX_ADDR     0x3004c
#define REG_EMC_MOD_ADDR     0x30050
#define REG_EMC_SAD_ADDR     0x30054
#define REG_EMC_NCU_ADDR     0x30058

/********************************************
  EFE (Encoder Front End)
*********************************************/
#define REG_EFE_CTRL         0x40000
#define EFE_TSE(en)          (((en) & 0x1)<<31)
#define EFE_FMVP(en)         (((en) & 0x1)<<30)
#define EFE_ID_X264          (0x0<<14)
#define EFE_ID_JPEG          (0x1<<14)
#define EFE_ID_VP8           (0x2<<14)
#define EFE_X264_QP(qp)      (((qp) & 0x3F)<<8)
#define EFE_VP8_QTB(qtb)     (((qtb) & 0x7f)<<22)
#define EFE_VP8_QIDX(qp)     (((qp) & 0x3F)<<8)
#define EFE_VP8_LF(lf)       ((lf & 0x3F)<<16)
#define EFE_HALN8_FLAG(en)   (((en) & 0x1)<<7)
#define EFE_STEP_MODE(en)    (((en) & 0x1)<<6)
#define EFE_DBLK_EN          (0x1<<5)
#define EFE_SLICE_TYPE(a)    (((a) & 0x1)<<4)
#define EFE_PLANE_TILE       (0x0<<2)
#define EFE_PLANE_420P       (0x1<<2)
#define EFE_PLANE_NV12       (0x2<<2)
#define EFE_PLANE_NV21       (0x3<<2)
#define EFE_EN               (0x1<<1)
#define EFE_RUN              (0x1<<0)

#define REG_EFE_GEOM         0x40004
#define EFE_FST_MBY(mb)      (((mb) & 0xFF)<<24)
#define EFE_FST_MBX(mb)      (((0/*FIXME*/) & 0xFF)<<16)
#define EFE_LST_MBY(mb)      (((mb) & 0xFF)<<8)
#define EFE_LST_MBX(mb)      (((mb) & 0xFF)<<0)
#define EFE_JPGC_LST_MBY(mb) (((mb) & 0xFFFF)<<16)
#define EFE_JPGC_LST_MBX(mb) ((mb) & 0xFFFF)

#define REG_EFE_COEF_BA      0x4000C
#define REG_EFE_RAWY_SBA     0x40010
#define REG_EFE_RAWC_SBA     0x40014
#define REG_EFE_RAWU_SBA     0x40014
#define REG_EFE_TOPMV_BA     0x40018
#define REG_EFE_TOPPA_BA     0x4001C
#define REG_EFE_MECHN_BA     0x40020
#define REG_EFE_MAUCHN_BA    0x40024
#define REG_EFE_DBLKCHN_BA   0x40028
#define REG_EFE_SDECHN_BA    0x4002C
#define REG_EFE_RAW_DBA      0x40030
#define REG_EFE_RAWV_SBA     0x40034

#define REG_EFE_ROI_MAX_QP       0x40040
#define REG_EFE_ROI_BASE_INFO0   0x40044
#define REG_EFE_ROI_BASE_INFO1   0x40048
#define REG_EFE_ROI_POS_INFO0    0x4004C
#define REG_EFE_ROI_POS_INFO1    0x40050
#define REG_EFE_ROI_POS_INFO2    0x40054
#define REG_EFE_ROI_POS_INFO3    0x40058
#define REG_EFE_ROI_POS_INFO4    0x4005C
#define REG_EFE_ROI_POS_INFO5    0x40060
#define REG_EFE_ROI_POS_INFO6    0x40064
#define REG_EFE_ROI_POS_INFO7    0x40068

#define REG_EFE_QP_GEN_TAB       0x4006C

#define REG_EFE_QPG_CTRL         0x40074
#define REG_EFE_QPG_CFG0         0x40078
#define REG_EFE_QPG_CFG1         0x4007C
#define REG_EFE_QPG_CFG2         0x40080
#define REG_EFE_QPG_CFG3         0x40084
#define REG_EFE_QPG_CFG4         0x40088
#define REG_EFE_QPG_CFG5         0x4008C
#define REG_EFE_QPG_CFG6         0x40090
#define REG_EFE_QPG_RGNC_A       0x40094
#define REG_EFE_QPG_RGNC_B       0x40098
#define REG_EFE_QPG_RGNC_C       0x4009C
#define REG_EFE_QPG_RGNC_D       0x400A0
#define REG_EFE_QP_SUM           0x400A4

#define REG_EFE_EIGEN_CFG0   0x400C0
#define REG_EFE_EIGEN_CFG1   0x400C4
#define REG_EFE_EIGEN_CFG2   0x400C8
#define REG_EFE_EIGEN_CFG3   0x400CC
#define REG_EFE_EIGEN_CFG4   0x400D0
#define REG_EFE_EIGEN_CFG5   0x400D4
#define REG_EFE_EIGEN_CFG6   0x400D8
#define REG_EFE_DIFFY_CFG    0x400DC
#define REG_EFE_DIFFU_CFG    0x400E0
#define REG_EFE_DIFFV_CFG    0x400E4

#define REG_EFE_SKIN_CTRL    0x400AC
#define REG_EFE_SKIN_PTHD0   0x400B0
#define REG_EFE_SKIN_PTHD1   0x400B4
#define REG_EFE_SKIN_PTHD2   0x400B8
#define REG_EFE_SKIN_QP_OFST     0x400BC
#define REG_EFE_SKIN_PARAM0  0x400E8
#define REG_EFE_SKIN_PARAM1  0x400EC
#define REG_EFE_SKIN_PARAM2  0x400F0

#define REG_EFE_RAW_STRD     0x40038
#define EFE_RAW_STRDY(y)     (((y) & 0xFFFF)<<16)
#define EFE_RAW_STRDC(c)     (((c) & 0xFFFF)<<0)

#define REG_EFE_DBG_INFO     0x4003C
#define EFE_DBG_EN           (0x1<<31)
#define EFE_DBG_BP_MBX(x)    (((x) & 0xFFF)<<0)
#define EFE_DBG_BP_MBY(y)    (((y) & 0xFFF)<<16)

#define REG_EFE_MVRP         0x40100
#define REG_EFE_SSAD         0x40108
#define REG_EFE_DCS          0x4010C
#define EFE_DCS_CLR(th)      (0x1<<(th & 0xF))
#define EFE_DCS_EN(en)       (((en) & 0x1)<<16)
#define EFE_DCS_RT(rt)       (((rt) & 0xF)<<20)
#define EFE_DCS_OTH(oth)     (((oth) & 0xF)<<24)
#define REG_EFE_STAT         0x40110
#define REG_EFE_CQP_OFST     0x40120

#define EFE_RC_QPO_CFG(c4, c3, c2, c1)      \
	( ((c4) & 0x3F)<<18 |                           \
	  ((c3) & 0x3F)<<12 |                           \
	  ((c2) & 0x3F)<<6 |                            \
	  ((c1) & 0x3F)<<0                              \
	)

#define REG_EFE_RC_MINFO    0x40128
#define EFE_RC_MB_EN        (0x1<<0)
#define EFE_RC_MBGP_NUM(a)  (((a) & 0x3fff)<<1)
#define EFE_RC_MAD_CFG_RDY  (0x1<<15)
#define REG_EFE_RC_BINFO0   0x4012C
#define EFE_RC_BU_EN        (0x1<<0)
#define EFE_RC_BU_NUM(a)    (((a) & 0x7f)<<1)
#define EFE_RC_BU_CFG(a)    (((a) & 0x1)<<8)
#define EFE_RC_SLICE_TP(a)  (((a) & 0x1)<<10)
#define REG_EFE_RC_BINFO1   0x40130
#define EFE_RC_BU_SIZE(a)   (((a) & 0x3fff)<<0)
#define EFE_RC_BU_LSIZE(a)  (((a) & 0x3fff)<<14)
#define REG_EFE_RC_GP0THD   0x40134
#define REG_EFE_RC_GP1THD    0x40138
#define REG_EFE_RC_GP2THD    0x4013C
#define REG_EFE_RC_TBS      0x40140
#define REG_EFE_RC_BNQA0    0x40144
#define REG_EFE_RC_BPQA0    0x40148
#define REG_EFE_RC_BNQA1    0x4014C
#define REG_EFE_RC_BPQA1    0x40150
#define REG_EFE_RC_MNQCS    0x40154
#define REG_EFE_RC_MPQCS    0x40158
#define REG_EFE_RC_MTBQ     0x4015C
#define REG_EFE_RC_MRFQ     0x40160
#define REG_EFE_RC_MAMIN    0x40164
#define REG_EFE_RC_MAMAX    0x40168
#define REG_EFE_RC_BBS      0x4016C

/********************************************
  MCE (Motion Compensation/Estimation COMBO)
*********************************************/
//GLB_CTRL
#define REG_MCE_GLB_CTRL        0x50000
#define MCE_GLB_CTRL_FMV0(a)    (((a) & 0xf)<<31)
#define MCE_GLB_CTRL_STEP1(a)   (((a) & 0xf)<<27)
#define MCE_GLB_CTRL_STEP0(a)   (((a) & 0xf)<<23)
#define MCE_GLB_CTRL_MSTEP(a)   (((a) & 0x1)<<22)
#define MCE_GLB_CTRL_RBS(a)     (((a) & 0xff)<<14)
#define MCE_GLB_CTRL_FMS(a)     (((a) & 0x7)<<11)
#define MCE_GLB_CTRL_FRMMV(a)   (((a) & 0x1)<<10)
#define MCE_GLB_CTRL_GLBMV(a)   (((a) & 0x1)<<9)
#define MCE_GLB_CTRL_DCT8(a)    (((a) & 0x1)<<8)
#define MCE_GLB_CTRL_JRFD(a)    (((a) & 0x1)<<7)
#define MCE_GLB_CTRL_MREF(a)    (((a) & 0x1)<<6)
#define MCE_GLB_CTRL_PSKIP(a)   (((a) & 0x1)<<5)
#define MCE_GLB_CTRL_RM(a)      (((a) & 0x1)<<4)
#define MCE_GLB_CTRL_BF         (0x1<<3)
#define MCE_GLB_CTRL_CGE        (0x1<<2)
#define MCE_GLB_CTRL_WM         (0x1<<1)
#define MCE_GLB_CTRL_INIT       (0x1<<0)

//COMP_CTRL
#define REG_MCE_COMP_CTRL       0x50010
#define MCE_COMP_CTRL_CCE       (0x1<<31)
#define MCE_COMP_CTRL_CWT(a)    (((a) & 0x3)<<26)
#define MCE_COMP_CTRL_CRR       (0x1<<25)
#define MCE_COMP_CTRL_CIC       (0x1<<24)
#define MCE_COMP_CTRL_CAT       (0x1<<23)
#define MCE_COMP_CTRL_CTAP(a)   (((a) & 0x3)<<20)
#define MCE_COMP_CTRL_CSPT(a)   (((a) & 0x3)<<18)
#define MCE_COMP_CTRL_CSPP(a)   (((a) & 0x3)<<16)
#define MCE_COMP_CTRL_YCE       (0x1<<15)
#define MCE_COMP_CTRL_YWT(a)    (((a) & 0x3)<<10)
#define MCE_COMP_CTRL_YRR       (0x1<<9)
#define MCE_COMP_CTRL_YIC       (0x1<<8)
#define MCE_COMP_CTRL_YAT       (0x1<<7)
#define MCE_COMP_CTRL_YTAP(a)   (((a) & 0x3)<<4)
#define MCE_COMP_CTRL_YSPT(a)   (((a) & 0x3)<<2)
#define MCE_COMP_CTRL_YSPP(a)   (((a) & 0x3)<<0)

#define MCE_WT_BIAVG            0
#define MCE_WT_UNIWT            1
#define MCE_WT_BIWT             2
#define MCE_WT_IMWT             3

#define MCE_TAP_TAP2            0
#define MCE_TAP_TAP4            1
#define MCE_TAP_TAP6            2
#define MCE_TAP_TAP8            3

#define MCE_SPT_AUTO            0
#define MCE_SPT_SPEC            1
#define MCE_SPT_BILI            2
#define MCE_SPT_SYMM            3

#define MCE_SPP_HPEL            0
#define MCE_SPP_QPEL            1
#define MCE_SPP_EPEL            2

//ESTI_CTRL
#define REG_MCE_ESTI_CTRL       0x50040
#define MCE_ESTI_CTRL_LSP(a)    (((a) & 0xF)<<28)
#define MCE_ESTI_CTRL_FBG(a)    (((a) & 0x1)<<27)
#define MCE_ESTI_CTRL_CLMV      (0x1<<26)
#define MCE_ESTI_CTRL_SCL(a)    (((a) & 0x3)<<24)
#define MCE_ESTI_CTRL_MSS(a)    (((a) & 0xFF)<<16)
#define MCE_ESTI_CTRL_QRL(a)    (((a) & 0x3)<<14)
#define MCE_ESTI_CTRL_HRL(a)    (((a) & 0x3)<<12)
#define MCE_ESTI_CTRL_BDIR(a)   (((a) & 0x1)<<10)
#define MCE_ESTI_CTRL_PUE_64X64 (0x1<<9)
#define MCE_ESTI_CTRL_PUE_32X32 (0x1<<6)
#define MCE_ESTI_CTRL_PUE_32X16 (0x1<<5)
#define MCE_ESTI_CTRL_PUE_16X32 (0x1<<4)
#define MCE_ESTI_CTRL_PUE_16X16 (0x1<<3)
#define MCE_ESTI_CTRL_PUE_16X8  (0x1<<2)
#define MCE_ESTI_CTRL_PUE_8X16  (0x1<<1)
#define MCE_ESTI_CTRL_PUE_8X8   (0x1<<0)

//MRGI
#define REG_MCE_MRGI            0x50044
#define MCE_MRGI_MRGE_64X64     (0x1<<9)
#define MCE_MRGI_MRGE_32X32     (0x1<<6)
#define MCE_MRGI_MRGE_32X16     (0x1<<5)
#define MCE_MRGI_MRGE_16X32     (0x1<<4)
#define MCE_MRGI_MRGE_16X16     (0x1<<3)
#define MCE_MRGI_MRGE_16X8      (0x1<<2)
#define MCE_MRGI_MRGE_8X16      (0x1<<1)
#define MCE_MRGI_MRGE_8X8       (0x1<<0)

//MVR
#define REG_MCE_MVR             0x50048
#define MCE_MVR_MVRY(a)         (((a) & 0xFFFF)<<16)
#define MCE_MVR_MVRX(a)         (((a) & 0xFFFF)<<0)

//FRM_SIZE
#define REG_MCE_FRM_SIZE        0x50060
#define MCE_FRM_SIZE_FH(a)      (((a) & 0xFFFF)<<16)
#define MCE_FRM_SIZE_FW(a)      (((a) & 0xFFFF)<<0)
#define MCE_FRM_SIZE_LRE(a)     (((a) & 0x1)<<14)
#define MCE_FRM_SIZE_RRE(a)     (((a) & 0x1)<<15)
#define MCE_FRM_SIZE_TRE(a)     (((a) & 0x1)<<30)
#define MCE_FRM_SIZE_BRE(a)     (((a) & 0x1)<<31)

//FSC
#define REG_MCE_FSC             0x5004C
#define MCE_FSC_FSE(a)          (((a) & 0x1)<<31)
#define MCE_FSC_FSMD(a)          (((a) & 0x1)<<30)
#define MCE_FSC_RECY(a)         (((a) & 0x7F)<<24)
#define MCE_FSC_RECX(a)         (((a) & 0xFF)<<16)
#define MCE_FSC_PERY(a)         (((a) & 0xFF)<<8)
#define MCE_FSC_PERX(a)         (((a) & 0xFF)<<0)

//FRM_STRD
#define REG_MCE_FRM_STRD        0x50064
#define MCE_FRM_STRD_STRDC(a)   (((a) & 0xFFFF)<<16)
#define MCE_FRM_STRD_STRDY(a)   (((a) & 0xFFFF)<<0)

//SLC_SPOS
#define REG_MCE_SLC_SPOS        0x50068
#define MCE_SLC_SPOS_CU64Y(a)   (((a) & 0xFF)<<8)
#define MCE_SLC_SPOS_CU64X(a)   (((a) & 0xFF)<<0)

//REF
#define REG_MCE_REFY0        0x5006C
#define REG_MCE_REFC0        0x50070

#define REG_MCE_REFY1        0x50074
#define REG_MCE_REFC1        0x50078

#define REG_MCE_REFY0_1      0x50110  //!!!
#define REG_MCE_REFC0_1      0x50114

//GLB MV
#define REG_MCE_GLB_MV       0x5007C
#define MCE_GLB_MVX(a)       (((a) & 0xFFF)<<0)
#define MCE_GLB_MVY(a)       (((a) & 0xFFF)<<16)

//RLUT
#define SLUT_MCE_RLUT(l, i)     (0x50800 + (i)*8 + (l)*0x80)

//ILUT
#define SLUT_MCE_ILUT_Y         0x50900
#define SLUT_MCE_ILUT_C         0x50980
#define MCE_ILUT_INFO(fir, clip, idgl, edgl, dir,        \
                      rnd, sft, savg, srnd, sbias)       \
( ((fir) & 0x1)<<31 |                                    \
  ((clip) & 0x1)<<27 |                                   \
  ((idgl) & 0x1)<<26 |                                   \
  ((edgl) & 0x1)<<25 |                                   \
  ((dir) & 0x1)<<24 |                                    \
  ((rnd) & 0xFF)<<16 |                                   \
  ((sft) & 0xF)<<8 |                                     \
  ((savg) & 0x1)<<2 |                                    \
  ((srnd) & 0x1)<<1 |                                    \
  ((sbias) & 0x1)<<0                                     \
)

//CLUT
#define SLUT_MCE_CLUT_Y         0x50A00
#define SLUT_MCE_CLUT_C         0x50B00
#define MCE_CLUT_INFO(c4, c3, c2, c1)                    \
	( ((c4) & 0xFF)<<24 |                                    \
	  ((c3) & 0xFF)<<16 |                                    \
	  ((c2) & 0xFF)<<8 |                                     \
	  ((c1) & 0xFF)<<0                                       \
	)

//CHAIN/TMV/SYNC ADDR

#define REG_MCE_TMV_BA          0x50104
#define REG_MCE_CHN_SYNC        0x50108
#define REG_MCE_CHN_BA          0x5010C

/********************************************
  VMAU (VPU Matrix Arithmetic Unit)
*********************************************/
#define REG_VMAU_MCBP        0x80000

#define REG_VMAU_QTPARA      0x80004

#define REG_VMAU_MAIN_ADDR   0x80008

#define REG_VMAU_NCCHN_ADDR  0x8000C

#define REG_VMAU_CHN_LEN     0x80010

#define REG_VMAU_ACBP        0x80014

#define REG_VMAU_CPREDM_TLV  0x80018

#define REG_VMAU_YPREDM0     0x8001C

#define REG_VMAU_YPREDM1     0x80020

#define REG_VMAU_TOP_BASE    0x80028

#define REG_VMAU_CTX         0x8002C

#define REG_VMAU_MD_CFG0     0x80030
#define VMAU_MD_SLICE_I(a)   (((a) & 0x1)<<0)
#define VMAU_MD_SLICE_P(a)   (((a) & 0x1)<<1)
#define VMAU_MD_IS_DECODE(a) (((a) & 0x1)<<4)
#define VMAU_MD_I4_DIS(a)    (((a) & 0x1)<<8)
#define VMAU_MD_I16_DIS(a)   (((a) & 0x1)<<9)
#define VMAU_MD_PSKIP_DIS(a) (((a) & 0x1)<<10)
#define VMAU_MD_P_L0_DIS(a)  (((a) & 0x1)<<11)
#define VMAU_MD_I8_DIS(a)    (((a) & 0x1)<<12)
#define VMAU_MD_PT8_DIS(a)   (((a) & 0x1)<<13)
#define VMAU_MD_DREF_EN(a)   (((a) & 0x1)<<14)
#define VMAU_MD_DCT8_EN(a)   (((a) & 0x1)<<15)
#define VMAU_MD_FRM_REDGE(a) (((a) & 0xFF)<<16)
#define VMAU_MD_FRM_BEDGE(a) (((a) & 0xFF)<<24)

#define REG_VMAU_MD_CFG1     0x80034
#define VMAU_IPMY_BIAS_EN(a) (((a) & 0x1)<<0)
#define VMAU_IPMC_BIAS_EN(a) (((a) & 0x1)<<1)
#define VMAU_COST_BIAS_EN(a) (((a) & 0x1)<<2)
#define VMAU_CSSE_BIAS_EN(a) (((a) & 0x1)<<3)
#define VMAU_JMLAMBDA2_EN(a) (((a) & 0x1)<<4)
#define VMAU_INTER_NEI_EN(a) (((a) & 0x1)<<5)
#define VMAU_SKIP_BIAS_EN(a) (((a) & 0x3)<<6)
#define VMAU_LMD_BIAS_EN(a)  (((a) & 0x1)<<8)
#define VMAU_INFO_EN(a)      (((a) & 0x1)<<9)
#define VMAU_DCM_EN(a)       (((a) & 0x1)<<10)
#define VMAU_MVDS_ALL(a)     (((a) & 0x1)<<12)
#define VMAU_MVDS_ABS(a)     (((a) & 0x1)<<13)
#define VMAU_MVS_ALL(a)      (((a) & 0x1)<<14)
#define VMAU_MVS_ABS(a)      (((a) & 0x1)<<15)
#define VMAU_P_L0_BIAS(a)    (((a) & 0xF)<<16)
#define VMAU_PSKIP_BIAS(a)   (((a) & 0xF)<<20)
#define VMAU_I4_BIAS(a)      (((a) & 0xF)<<24)
#define VMAU_I16_BIAS(a)     (((a) & 0xF)<<28)

#define REG_VMAU_MD_CFG2     0x80038
#define VMAU_IPM_BIAS_0(a)   (((a) & 0xF)<<0)
#define VMAU_IPM_BIAS_1(a)   (((a) & 0xF)<<4)
#define VMAU_IPM_BIAS_2(a)   (((a) & 0xF)<<8)
#define VMAU_IPM_BIAS_QP0(a) (((a) & 0x3F)<<12)
#define VMAU_IPM_BIAS_QP1(a) (((a) & 0x3F)<<18)
#define VMAU_MD_FBC_EP(a)    (((a) & 0xFF)<<24)

#define REG_VMAU_MD_CFG3     0x8003C
#define VMAU_CSSE_BIAS_0(a)   (((a) & 0xF)<<0)
#define VMAU_CSSE_BIAS_1(a)   (((a) & 0xF)<<4)
#define VMAU_CSSE_BIAS_2(a)   (((a) & 0xF)<<8)
#define VMAU_CSSE_BIAS_QP0(a) (((a) & 0x3F)<<12)
#define VMAU_CSSE_BIAS_QP1(a) (((a) & 0x3F)<<18)
#define VMAU_LMD_BIAS(a)      (((a) & 0xF)<<24)
#define VMAU_PL0_FS_DIS(a)    (((a) & 0x1)<<28)
#define VMAU_PT8_FS_DIS(a)    (((a) & 0x1)<<29)
#define VMAU_PL0_COST_MAX(a)  (((a) & 0x1)<<30)
#define VMAU_PT8_COST_MAX(a)  (((a) & 0x1)<<31)

#define REG_VMAU_GBL_RUN     0x80040
#define VMAU_RUN             0x1
#define VMAU_STOP            0x2
#define VMAU_RESET           0x4

#define REG_VMAU_GBL_CTR     0x80044
#define VMAU_CTRL_FIFO_M     0x1
#define VMAU_CTRL_IRQ_EN     0x10
#define VMAU_CTRL_SLPOW      0x10000
#define VMAU_CTRL_TO_DBLK    0x1000000
#define VMAU_LAMBDA_THRETH(a) (((a) & 0x3)<<25)

#define REG_VMAU_STATUS      0x80048

#define REG_VMAU_CCHN_ADDR   0x8004C

#define REG_VMAU_VIDEO_TYPE  0x80050
#define VMAU_FMT_H264        0x1
#define VMAU_FMT_RV9         0x2
#define VMAU_FMT_VC1         0x3
#define VMAU_FMT_MPEG2       0x4
#define VMAU_FMT_MPEG4       0x5
#define VMAU_FMT_VP8         0x6
#define VMAU_I4_MSK(a)       (((a) & 0x1)<<3)
#define VMAU_I16_MSK(a)      (((a) & 0x1)<<4)
#define VMAU_I8_MSK(a)       (((a) & 0x1)<<5)
#define VMAU_PT8_MSK(a)      (((a) & 0x1)<<6)
#define VMAU_MODE_DEC        (0x0<<11)
#define VMAU_MODE_ENC        (0x1<<11)
#define VMAU_IS_ISLICE(a)    (((a) & 0x1)<<12)
#define VMAU_PREDM_MSK(a)    (((a) & 0x1FFFF)<<14)
#define VMAU_TSE(en)         (((en) & 0x1)<<31)

#define REG_VMAU_Y_GS        0x80054
#define VMAU_FRM_WID(a)      (((a) & 0x3FFF)<<0)
#define VMAU_FRM_HEI(a)      (((a) & 0x3FFF)<<16)

#define REG_VMAU_DEC_DONE    0x80058

#define REG_VMAU_ENC_DONE    0x8005C

#define REG_VMAU_POS         0x80060

#define REG_VMAU_MCF_STA     0x80064

#define REG_VMAU_DEC_YADDR   0x80068

#define REG_VMAU_DEC_UADDR   0x8006C

#define REG_VMAU_DEC_VADDR   0x80070

#define REG_VMAU_DEC_STR     0x80074

#define REG_VMAU_DEADZONE    0x80078
#define VMAU_DEADZONE0_IY(a) (((a) & 0xFF)<<0)
#define VMAU_DEADZONE1_PY(a) (((a) & 0xFF)<<8)
#define VMAU_DEADZONE2_IC(a) (((a) & 0xFF)<<16)
#define VMAU_DEADZONE3_PC(a) (((a) & 0xFF)<<24)

#define REG_VMAU_ACMASK      0x8007C

#define REG_VMAU_MD_CFG4     0x800D8
#define VMAU_YSSE_THR(a)     (((a) & 0xFFFFFF)<<0)
#define VMAU_I8_BIAS(a)      (((a) & 0xF)<<24)
#define VMAU_PT8_BIAS(a)     (((a) & 0xF)<<28)

#define REG_VMAU_MD_CFG5     0x800DC
#define VMAU_CSSE_THR(a)     (((a) & 0xFFFFFF)<<0)
#define VMAU_CQP_OFFSET(a)   (((a) & 0x1F)<<24)
#define VMAU_I4_COST_MAX(a)  (((a) & 0x1)<<29)
#define VMAU_I8_COST_MAX(a)  (((a) & 0x1)<<30)
#define VMAU_I16_COST_MAX(a) (((a) & 0x1)<<31)

#define REG_VMAU_MD_CFG6     0x800FC
#define VMAU_DCM_PARAM(a)    (((a) & 0xFFFFFF)<<0)
#define VMAU_SDE_PRIOR(a)    (((a) & 0xF)<<24)
#define VMAU_DB_PRIOR(a)     (((a) & 0xF)<<28)

#define REG_VMAU_MD_CFG7     0x80114
#define VMAU_CFG_SIZE_X(a)   (((a) & 0xF)<<0)
#define VMAU_CFG_SIZE_Y(a)   (((a) & 0xF)<<4)
#define VMAU_CFG_IW_THR(a)   (((a) & 0x1FF)<<8)
#define VMAU_CFG_BASEQP(a)   (((a) & 0x3F)<<17)
#define VMAU_CFG_ALPHA(a)    (((a) & 0xFF)<<23)
#define VMAU_PS_COST_MAX(a)  (((a) & 0x1)<<31)

#define REG_VMAU_MD_CFG8     0x80118
#define VMAU_CFG_MVR_THR1(a) (((a) & 0x7FFF)<<0)
#define VMAU_CFG_MVR_THR2(a) (((a) & 0x1FFFF)<<15)

#define REG_VMAU_MD_CFG9     0x8011C
#define VMAU_CFG_MVR_THR3(a) (((a) & 0xFFFFF)<<0)
#define VMAU_CFG_BETA(a)     (((a) & 0xFF)<<20)

#define REG_VMAU_MD_CFG10    0x80190

#define REG_VMAU_MD_CFG11    0x80194
#define VMAU_CFG_MD_RBIAS(a)   (((a) & 0xf) << 21)
#define VMAU_CFG_MD_RBIAS_EN(a) (((a) & 1) << 25)
#define VMAU_CFG_MD_PCDC_N0(a) (((a) & 7) << 26)
#define VMAU_CFG_MD_IFA_VLD(a) (((a) & 1) << 29)
#define VMAU_CFG_MD_SLV_VLD(a) (((a) & 1) << 30)
#define VMAU_CFG_MD_SET_VLD(a) (((a) & 1) << 31)

#define REG_VMAU_MD_MODE     0x80198

#define REG_VMAU_DEADZONE1   0x8019C

#define REG_VMAU_IPRED_CFG0   0x801C0
#define VMAU_IP_MD_VAL(a)     (((a) & 0x1)<<30)
#define VMAU_IP_REF_NEB_4(a)  (((a) & 0x1)<<29)
#define VMAU_IP_REF_NEB_8(a)  (((a) & 0x1)<<28)
#define VMAU_IP_REF_PRD_C(a)  (((a) & 0xF)<<24)
#define VMAU_IP_REF_PRD_4(a)  (((a) & 0xF)<<20)
#define VMAU_IP_REF_PRD_8(a)  (((a) & 0xF)<<16)
#define VMAU_IP_REF_PRD_16(a) (((a) & 0xF)<<12)
#define VMAU_IP_REF_CUV_EN(a) (((a) & 0x1)<<11)
#define VMAU_IP_REF_C4_EN(a)  (((a) & 0x1)<<10)
#define VMAU_IP_REF_C8_EN(a)  (((a) & 0x1)<<9)
#define VMAU_IP_REF_C16_EN(a) (((a) & 0x1)<<8)
#define VMAU_IP_REF_LMDUV_EN(a) (((a) & 0x1)<<7)
#define VMAU_IP_REF_LMD4_EN(a)  (((a) & 0x1)<<6)
#define VMAU_IP_REF_LMD8_EN(a)  (((a) & 0x1)<<5)
#define VMAU_IP_REF_LMD16_EN(a) (((a) & 0x1)<<4)
#define VMAU_IP_REF_BITUV_EN(a) (((a) & 0x1)<<3)
#define VMAU_IP_REF_BIT4_EN(a)  (((a) & 0x1)<<2)
#define VMAU_IP_REF_BIT8_EN(a)  (((a) & 0x1)<<1)
#define VMAU_IP_REF_BIT16_EN(a) (((a) & 0x1)<<0)

#define REG_VMAU_IPRED_CFG1  0x801C4
#define VMAU_IP_REF_4_BIT0(a)  (((a) & 0xF)<<0)
#define VMAU_IP_REF_4_BIT1(a)  (((a) & 0xF)<<4)
#define VMAU_IP_REF_4_BIT2(a)  (((a) & 0xF)<<8)
#define VMAU_IP_REF_4_BIT3(a)  (((a) & 0xF)<<12)
#define VMAU_IP_REF_8_BIT0(a)  (((a) & 0xF)<<16)
#define VMAU_IP_REF_8_BIT1(a)  (((a) & 0xF)<<20)
#define VMAU_IP_REF_8_BIT2(a)  (((a) & 0xF)<<24)
#define VMAU_IP_REF_8_BIT3(a)  (((a) & 0xF)<<28)

#define REG_VMAU_IPRED_CFG2  0x801C8
#define VMAU_IP_REF_16_BIT0(a) (((a) & 0xF)<<0)
#define VMAU_IP_REF_16_BIT1(a) (((a) & 0xF)<<4)
#define VMAU_IP_REF_16_BIT2(a) (((a) & 0xF)<<8)
#define VMAU_IP_REF_16_BIT3(a) (((a) & 0xF)<<12)
#define VMAU_IP_REF_C_BIT0(a)  (((a) & 0xF)<<16)
#define VMAU_IP_REF_C_BIT1(a)  (((a) & 0xF)<<20)
#define VMAU_IP_REF_C_BIT2(a)  (((a) & 0xF)<<24)
#define VMAU_IP_REF_C_BIT3(a)  (((a) & 0xF)<<28)

#define REG_VMAU_IPRED_CFG3  0x801CC
#define VMAU_IP_REF_LMD16_IFO(a) (((a) & 0xF)<<0)
#define VMAU_IP_REF_LMD8_IFO(a)  (((a) & 0xF)<<4)
#define VMAU_IP_REF_LMD4_IFO(a)  (((a) & 0xF)<<8)
#define VMAU_IP_REF_LMDUV_IFO(a) (((a) & 0xF)<<12)
#define VMAU_IP_REF_NEB_8REF(a)  (((a) & 0xF)<<16)
#define VMAU_IP_REF_NEB_4REF(a)  (((a) & 0xF)<<20)

#define REG_VMAU_IPRED_CFG4  0x801D0
#define VMAU_IP_REF_C4_IFO(a)  (((a) & 0xFFFF)<<0)
#define VMAU_IP_REF_C8_IFO(a)  (((a) & 0xFFFF)<<16)

#define REG_VMAU_IPRED_CFG5  0x801D4
#define VMAU_IP_REF_C16_IFO(a)  (((a) & 0xFFFF)<<0)
#define VMAU_IP_REF_CUV_IFO(a)  (((a) & 0xFFFF)<<16)

#define REG_VMAU_MEML        0x84000

#define REG_VMAU_QT          0x88000

#define REG_VMAU_CTX_CFBC    0x80174

/********************************************
  DBLK (deblock)
*********************************************/
#define REG_DBLK_DHA         0x70000

#define REG_DBLK_CFG         0x7005C
#define DBLK_CFG_ALPHA(a)    (((a) & 0x1F)<<0)
#define DBLK_CFG_BETA(a)     (((a) & 0x1F)<<8)
#define DBLK_CFG_NO_LFT(a)   (((a) & 0x1)<<17)

#define REG_DBLK_TRIG        0x70060
#define DBLK_RUN             0x1
#define DBLK_STOP            0x2
#define DBLK_RESET           0x4
#define DBLK_SLICE_RUN       0x8

#define REG_DBLK_CTRL        0x70064
#define DBLK_CTRL(expand, rotate, loop_filter)           \
	( ((expand) & 0x1)<<4 |                                  \
	  ((rotate) & 0x3)<<1 |                                  \
	  ((loop_filter) & 0x1)<<0                               \
	)

#define REG_DBLK_VTR         0x70068
#define DBLK_FMT_H264        0x1
#define DBLK_FMT_RV9         0x2
#define DBLK_FMT_VC1         0x3
#define DBLK_FMT_MPEG2       0x4
#define DBLK_FMT_MPEG4       0x5
#define DBLK_FMT_VP8         0x6
#define DBLK_FRM_I           0x0
#define DBLK_FRM_P           0x1
#define DBLK_FRM_B           0x2
#define DBLK_VTR(beta, alpha, vp8_spl, vp8_kf,           \
                 frm_typ, video_fmt)                     \
( ((beta) & 0xFF)<<24 |                                  \
  ((alpha) & 0xFF)<<16 |                                 \
  ((vp8_spl) & 0x1)<<9 |                                 \
  ((vp8_kf) & 0x1)<<5 |                                  \
  ((frm_typ) & 0x3)<<3 |                                 \
  ((video_fmt) & 0x7)<<0                                 \
)

#define REG_DBLK_FSTA        0x7006C

#define REG_DBLK_GSTA        0x70070
#define DBLK_STAT_DOEND  (0x1<<0)

#define REG_DBLK_GSIZE       0x70074
#define DBLK_GSIZE(mb_height, mb_width)                  \
	( ((mb_height) & 0xFFFF)<<16 |                           \
	  ((mb_width) & 0xFFFF)<<0                               \
	)

#define REG_DBLK_GENDA       0x70078

#define REG_DBLK_GPOS        0x7007C
#define DBLK_GPOS(first_mby, first_mbx)                  \
	( ((first_mby) & 0xFFFF)<<16 |                           \
	  ((first_mbx) & 0xFFFF)<<0                              \
	)

#define REG_DBLK_GPIC_STR    0x70080
#define DBLK_GPIC_STR(dst_strd_c, dst_strd_y)            \
	( ((dst_strd_c) & 0xFFFF)<<16 |                          \
	  ((dst_strd_y) & 0xFFFF)<<0                             \
	)

#define REG_DBLK_GPIC_YA     0x70084

#define REG_DBLK_GPIC_CA     0x70088

#define REG_DBLK_GP_ENDA     0x7008C

#define REG_DBLK_SLICE_ENDA  0x70090

#define REG_DBLK_BLK_CTRL    0x70094

#define REG_DBLK_BLK_FIFO    0x70098

#define REG_DBLK_MOS_CFG     0x70240
#define REG_DBLK_MOS_ADDR    0x70244
#define DBLK_MOS_BASE_ADDR   VPU_BASE + 0xC7000
#define REG_DBLK_MOS_CRC     0x70248
/********************************************
  SDE (stream parser/encoding)
*********************************************/
#define REG_SDE_STAT         0x90000
#define SDE_STAT_BSEND       (0x1<<1)

#define REG_SDE_SL_CTRL      0x90004
#define SDE_SLICE_INIT       (0x1<<1)
#define SDE_MB_RUN           (0x1<<0)

#define REG_SDE_SL_GEOM      0x90008
#define SDE_SL_GEOM(mb_height, mb_width,                 \
                    first_mby, first_mbx)                \
( ((mb_height) & 0xFF)<<24 |                             \
  ((mb_width) & 0xFF)<<16 |                              \
  ((first_mby) & 0xFF)<<8 |                              \
  ((first_mbx) & 0xFF)<<0                                \
)

#define REG_SDE_GL_CTRL      0x9000C
#define SDE_BP(mby, mbx)                                 \
	( ((mby) & 0xFF)<<24 |                                   \
	  ((mbx) & 0xFF)<<16                                     \
	)
#define SDE_MODE_AUTO        (0x0<<4)
#define SDE_MODE_STEP        (0x1<<4)
#define SDE_MODE_DEBUG       (0x2<<4)
#define SDE_EN               (0x1<<0)

#define REG_SDE_CODEC_ID     0x90010
#define SDE_FMT_H264_DEC     (0x1<<0)
#define SDE_FMT_H264_ENC     (0x1<<1)
#define SDE_FMT_VP8_DEC      (0x1<<2)
#define SDE_FMT_VC1_DEC      (0x1<<3)
#define SDE_FMT_MPEG2_DEC    (0x1<<4)
#define SDE_FMT_VP8_ENC      (0x1<<5)

#define REG_SDE_CFG0         0x90014
#define REG_SDE_CFG1         0x90018
#define REG_SDE_CFG2         0x9001C
#define REG_SDE_CFG3         0x90020
#define REG_SDE_CFG4         0x90024
#define REG_SDE_CFG5         0x90028
#define REG_SDE_CFG6         0x9002C
#define REG_SDE_CFG7         0x90030
#define REG_SDE_CFG8         0x90034
#define REG_SDE_CFG9         0x90038
#define REG_SDE_CFG10        0x9003C
#define REG_SDE_CFG11        0x90040
#define REG_SDE_CFG12        0x90044
#define REG_SDE_CFG13        0x90048
#define REG_SDE_CFG14        0x9004C
#define REG_SDE_CFG15        0x90050

#define REG_SDE_CTX_TBL      0x92000
#define REG_SDE_CQP_TBL      0x93800

/****************************************************************
  JPGC (jpeg codec)
*****************************************************************/
#define REG_JPGC_TRIG        0xE0000
#define REG_JPGC_GLBI        0xE0004
#define REG_JPGC_STAT        0xE0008
#define JPGC_STAT_ENDF       (0x1<<31)
#define REG_JPGC_BSA         0xE000C
#define REG_JPGC_P0A         0xE0010
#define REG_JPGC_P1A         0xE0014
#define REG_JPGC_P2A         0xE0018
#define REG_JPGC_P3A         0xE001C
#define REG_JPGC_NMCU        0xE0028
#define REG_JPGC_NRSM        0xE002C
#define REG_JPGC_P0C         0xE0030
#define REG_JPGC_P1C         0xE0034
#define REG_JPGC_P2C         0xE0038
#define REG_JPGC_P3C         0xE003C
#define REG_JPGC_WIDTH       0xE0040
#define REG_JPGC_MCUS        0xE0064
#define REG_JPGC_ZIGM0       0xE1000
#define REG_JPGC_ZIGM1       0xE1100
#define REG_JPGC_HUFB        0xE1200
#define REG_JPGC_HUFM        0xE1300
#define REG_JPGC_QMEM        0xE1400
#define REG_JPGC_HUFE        0xE1800
#define REG_JPGC_HUFS        0xE1800

#define JPGC_CORE_OPEN      (0x1<<0)
#define JPGC_BS_TRIG        (0x1<<1)
#define JPGC_PP_TRIG        (0x1<<2)
#define JPGC_TERM           (0x1<<3)
#define JPGC_RSTER_MD       (0x1<<8)

/****************************************************************
  ODMA
*****************************************************************/

#define REG_ODMA_TRIG 0x60000
#define REG_ODMA_CTRL 0x60004
#define REG_ODMA_BDYA 0x60008
#define REG_ODMA_BDCA 0x6000C
#define REG_ODMA_BSTR 0x60010
#define REG_ODMA_HDYA 0x60014
#define REG_ODMA_HDCA 0x60018
#define REG_ODMA_SPYA 0x6001C
#define REG_ODMA_SPCA 0x60020

#define ODMA_REC_STRDY(y)     (((y) & 0xFFFF)<<16)
#define ODMA_REC_STRDC(c)     (((c) & 0xFFFF)<<0)

/****************************************************************
  JRFD
*****************************************************************/

#define REG_JRFD_TRIG 0x10000
#define REG_JRFD_CTRL 0x10004
#define REG_JRFD_HDYA 0x10008
#define REG_JRFD_HDCA 0x1000C
#define REG_JRFD_HSTR 0x10010
#define REG_JRFD_BDYA 0x10014
#define REG_JRFD_BDCA 0x10018
#define REG_JRFD_BSTR 0x1001C
#define REG_JRFD_MHDY 0x10020
#define REG_JRFD_MHDC 0x10024
#define REG_JRFD_MBDY 0x10028
#define REG_JRFD_MBDC 0x1002C

#define JRFD_BODY_STRDY(y)     (((y) & 0xFFFF)<<16)
#define JRFD_BODY_STRDC(c)     (((c) & 0xFFFF)<<0)

/****************************************************************
  IFA
*****************************************************************/
#define REG_IFA_CTRL            0xB0000
#define REG_IFA_FRM_SIZE        0xB0004
#define REG_IFA_RAWY_BA         0xB0008
#define REG_IFA_RAWC_BA         0xB000C
#define REG_IFA_RAW_STR         0xB0010
#define REG_IFA_REFY_BA0        0xB0014
#define REG_IFA_REFC_BA0        0xB0018
#define REG_IFA_THRD_Y          0xB001C
#define REG_IFA_THRD_C          0xB0020
#define REG_IFA_REFY_BA1        0xB0030
#define REG_IFA_REFC_BA1        0xB0034

#define IFA_CTRL_INIT           ( 0x1 << 0 )
#define IFA_CTRL_UV_EN(a)       ( (a&0x1) << 1 )
#define IFA_CTRL_RRS_SY(a)      ( (a&0x3) << 2 )
#define IFA_CTRL_RRS_SC(a)      ( (a&0x1) << 4 )
#define IFA_CTRL_DDR_DW(a)      ( (a&0x1) << 5 )
#define IFA_CTRL_RRS_EN(a)      ( (a&0x1) << 6 )
#define IFA_CTRL_DUMP_EN(a)     ( (a&0x1) << 7 )
#define IFA_CTRL_REF_BIDX(a)    ( (a&0xff) << 8 )
#define IFA_CTRL_CGE            (0x1<<16) //1 mean close clk-gate, 0 default open
#define IFA_CTRL_RAW_TYPE(a)    ( ((a) & 0x1) << 17 )

#define IFA_FRM_W(a)      ( ((a)&0xFFF) << 0 )
#define IFA_FRM_H(a)      ( ((a)&0xFFF) << 16 )

#define IFA_STR_Y(a)      ( ((a)&0xFFFF) << 0 )
#define IFA_STR_C(a)      ( ((a)&0xFFFF) << 16 )

#define IFA_THRD_U(a)      ( ((a)&0x3FFF) << 0 )
#define IFA_THRD_V(a)      ( ((a)&0x3FFF) << 16 )

/****************************************************************
  VPU tables
*****************************************************************/

/********************************************
  Motion interpolation programable table
*********************************************/
#define IS_SKIRT  0
#define IS_MIRROR 1

#define IS_BIAVG  0
#define IS_WT1    1
#define IS_WT2    2
#define IS_FIXWT  3

#define IS_ILUT0  0
#define IS_ILUT1  2
#define IS_EC     1

#define IS_TCS     1
#define NOT_TCS    0
#define IS_SCS     1
#define NOT_SCS    0
#define IS_HLDGL   1
#define NOT_HLDGL  0
#define IS_AVSDGL  1
#define NOT_AVSDGL 0

#define INTP_HDIR  0
#define INTP_VDIR  1

enum IntpID {
	MPEG_HPEL = 0,
	MPEG_QPEL,
	H264_QPEL,
	H264_EPEL,
	RV8_TPEL,
	RV9_QPEL,
	RV9_CPEL,
	WMV2_QPEL,
	VC1_QPEL,
	AVS_QPEL,
	VP6_QPEL,
	VP8_QPEL,
	VP8_EPEL,
	VP8_BIL,
	VP8_FPEL, /*full-pixel for chroma*/
	HEVC_QPEL,
	HEVC_EPEL,
};

enum PosID {
	H0V0 = 0,
	H1V0,
	H2V0,
	H3V0,
	H0V1,
	H1V1,
	H2V1,
	H3V1,
	H0V2,
	H1V2,
	H2V2,
	H3V2,
	H0V3,
	H1V3,
	H2V3,
	H3V3,
};

enum TapTYP {
	TAP2 = 0,
	TAP4,
	TAP6,
	TAP8,
};

enum SPelSFT {
	HPEL = 0,
	QPEL,
	EPEL,
};

typedef struct IntpFMT_t {
	char tap;
	char intp_pkg[2];
	char hldgl;
	char avsdgl;
	char intp[2];
	char intp_dir[2];
	char intp_coef[2][8];
	char intp_rnd[2];
	char intp_sft[2];
	char intp_sintp[2];
	char intp_srnd[2];
	char intp_sbias[2];
} IntpFMT_t;

//__place_k0_data__
//static char AryFMT[] = {
//    IS_SKIRT, IS_MIRROR, IS_SKIRT, IS_SKIRT,
//    IS_SKIRT, IS_SKIRT, IS_SKIRT, IS_SKIRT,
//    IS_SKIRT, IS_SKIRT, IS_SKIRT, IS_SKIRT,
//    IS_SKIRT, IS_SKIRT, IS_SKIRT, IS_SKIRT, IS_SKIRT,
//};
//
//__place_k0_data__
//static char SubPel[] = {
//    HPEL, QPEL, QPEL, EPEL,
//    QPEL, QPEL, QPEL, QPEL,
//    QPEL, QPEL, QPEL, QPEL,
//    EPEL, HPEL, QPEL, QPEL, EPEL
//};

#if 0
__place_k0_data__
static IntpFMT_t IntpFMT[][16] = {
	{
		/************* MPEG_HPEL ***************/
		{/*H0V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H0V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H0V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H0V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0} },
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
	},

	{
		/************* MPEG_QPEL ***************/
		{/*H0V0*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {0},},
			{/*intp_rnd*/15, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {0},},
			{/*intp_rnd*/15, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {0},},
			{/*intp_rnd*/15, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/1, 0},
		},
		{/*H0V1*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {0},},
			{/*intp_rnd*/15, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/1, 1},
			{/*intp_srnd*/1, 1},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/1, 1},
			{/*intp_srnd*/1, 1},
			{/*intp_sbias*/1, 0},
		},
		{/*H0V2*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {0},},
			{/*intp_rnd*/15, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/1, 0},
		},
		{/*H0V3*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {0},},
			{/*intp_rnd*/15, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/1, 0},
		},
		{/*H1V3*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/1, 1},
			{/*intp_srnd*/1, 1},
			{/*intp_sbias*/0, 1},
		},
		{/*H2V3*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 1},
		},
		{/*H3V3*/
			TAP8, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 3, -6, 20, 20, -6, 3, -1}, {-1, 3, -6, 20, 20, -6, 3, -1},},
			{/*intp_rnd*/15, 15},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/1, 1},
			{/*intp_srnd*/1, 1},
			{/*intp_sbias*/1, 1},
		},
	},

	{
		/************* H264_QPEL ***************/
		{/*H0V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/1, 0},
		},
		{/*H0V1*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP6, {IS_TCS, IS_SCS}, IS_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 16},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 10},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP6, {IS_TCS, IS_SCS}, IS_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 16},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/1, 0},
		},
		{/*H0V2*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 10},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 10},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 10},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 1},
		},
		{/*H0V3*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/1, 0},
		},
		{/*H1V3*/
			TAP6, {IS_TCS, IS_SCS}, IS_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 16},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 1},
		},
		{/*H2V3*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 10},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 1},
		},
		{/*H3V3*/
			TAP6, {IS_TCS, IS_SCS}, IS_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 16},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/1, 1},
		},
	},

	{
		/************* H264_EPEL ***************/
		{/*H0V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/0},
		{/*H2V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/4, 0},
			{/*intp_sft*/3, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V0*/0},
		{/*H0V1*/0},
		{/*H1V1*/0},
		{/*H2V1*/0},
		{/*H3V1*/0},
		{/*H0V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/4, 0},
			{/*intp_sft*/3, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V2*/0},
		{/*H2V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/0, 32},
			{/*intp_sft*/0, 6},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V2*/0},
		{/*H0V3*/0},
		{/*H1V3*/0},
		{/*H2V3*/0},
		{/*H3V3*/0},
	},

	{
		/************* RV8_TPEL ***************/
		{/*H0V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 12, 6, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 6, 12, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/0},
		{/*H0V1*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 12, 6, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 12, 6, -1, 0, 0, 0, 0}, {-1, 12, 6, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/8, 0}, //{0,128}
			{/*intp_sft*/0, 8},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 6, 12, -1, 0, 0, 0, 0}, {-1, 12, 6, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/8, 0}, //{0,128}
			{/*intp_sft*/0, 8},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/0},
		{/*H0V2*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 6, 12, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 12, 6, -1, 0, 0, 0, 0}, {-1, 6, 12, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/8, 0}, //{0,128}
			{/*intp_sft*/0, 8},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 6, 12, -1, 0, 0, 0, 0}, {-1, 6, 12, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/8, 0}, //{0,128}
			{/*intp_sft*/0, 8},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/0},
		{/*H0V3*/0},
		{/*H1V3*/0},
		{/*H2V3*/0},
		{/*H3V3*/0},
	},

	{
		/************* RV9_QPEL ***************/
		{/*H0V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, -5, 52, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, -5, 20, 52, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V1*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, -5, 52, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 52, 20, -5, 1, 0, 0}, {1, -5, 52, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/32, 32},
			{/*intp_sft*/6, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 52, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 32},
			{/*intp_sft*/5, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 52, -5, 1, 0, 0}, {1, -5, 52, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/32, 32},
			{/*intp_sft*/6, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V2*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/16, 0},
			{/*intp_sft*/5, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 52, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/32, 16},
			{/*intp_sft*/6, 5},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 16},
			{/*intp_sft*/5, 5},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 52, -5, 1, 0, 0}, {1, -5, 20, 20, -5, 1, 0, 0},},
			{/*intp_rnd*/32, 16},
			{/*intp_sft*/6, 5},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V3*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, -5, 20, 52, -5, 1, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V3*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 52, 20, -5, 1, 0, 0}, {1, -5, 20, 52, -5, 1, 0, 0},},
			{/*intp_rnd*/32, 32},
			{/*intp_sft*/6, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V3*/
			TAP6, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -5, 20, 20, -5, 1, 0, 0}, {1, -5, 20, 52, -5, 1, 0, 0},},
			{/*intp_rnd*/16, 32},
			{/*intp_sft*/5, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 2},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
	},

	{
		/************* RV9_CPEL ***************/
		{/*H0V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/2, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {3, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 7},
			{/*intp_sft*/0, 4},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {3, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 4},
			{/*intp_sft*/0, 3},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {3, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 7},
			{/*intp_sft*/0, 4},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 4},
			{/*intp_sft*/0, 3},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 1},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 4},
			{/*intp_sft*/0, 3},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/2, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {1, 3, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 7},
			{/*intp_sft*/0, 4},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 3, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 4},
			{/*intp_sft*/0, 3},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V3*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 1},
			{/*intp_sft*/0, 2},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
	},

	{
		/************* WMV2_QPEL ***************/
		{/*H0V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/1, 0},
			{/*intp_srnd*/1, 0},
			{/*intp_sbias*/1, 0},
		},
		{/*H0V1*/0},
		{/*H1V1*/0},
		{/*H2V1*/0},
		{/*H3V1*/0},
		{/*H0V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {-1, 9, 9, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/8, 8},
			{/*intp_sft*/4, 4},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {-1, 9, 9, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/8, 8},
			{/*intp_sft*/4, 4},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {-1, 9, 9, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/8, 8},
			{/*intp_sft*/4, 4},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 1},
		},
		{/*H0V3*/0},
		{/*H1V3*/0},
		{/*H2V3*/0},
		{/*H3V3*/0},
	},

	{
		/************* VC1_QPEL ***************/
		{/*H0V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-4, 53, 18, -3, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/8, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-3, 18, 53, -4, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V1*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-4, 53, 18, -3, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/31, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-4, 53, 18, -3, 0, 0, 0, 0}, {-4, 53, 18, -3, 0, 0, 0, 0},},
			{/*intp_rnd*/31, 32},
			{/*intp_sft*/6, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-4, 53, 18, -3, 0, 0, 0, 0}, {-1, 9, 9, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/31, 8},
			{/*intp_sft*/6, 4},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-4, 53, 18, -3, 0, 0, 0, 0}, {-3, 18, 53, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/31, 32},
			{/*intp_sft*/6, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/7, 0},
			{/*intp_sft*/4, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {-4, 53, 18, -3, 0, 0, 0, 0},},
			{/*intp_rnd*/7, 32},
			{/*intp_sft*/4, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {-1, 9, 9, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/7, 8},
			{/*intp_sft*/4, 4},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-1, 9, 9, -1, 0, 0, 0, 0}, {-3, 18, 53, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/7, 32},
			{/*intp_sft*/4, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V3*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-3, 18, 53, -4, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/31, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V3*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-3, 18, 53, -4, 0, 0, 0, 0}, {-4, 53, 18, -3, 0, 0, 0, 0},},
			{/*intp_rnd*/31, 32},
			{/*intp_sft*/6, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V3*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-3, 18, 53, -4, 0, 0, 0, 0}, {-1, 9, 9, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/31, 8},
			{/*intp_sft*/6, 4},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V3*/
			TAP4, {NOT_TCS, IS_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_VDIR, INTP_HDIR},
			{/*intp_coef*/{-3, 18, 53, -4, 0, 0, 0, 0}, {-3, 18, 53, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/31, 32},
			{/*intp_sft*/6, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
	},

	{
		/************* AVS_QPEL ***************/
		{/*H0V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, -2, 96, 42, -7, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 5, 5, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/4, 0},
			{/*intp_sft*/3, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{0, -7, 42, 96, -2, -1, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V1*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, -2, 96, 42, -7, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, IS_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 5, 5, -1, 0, 0, 0, 0}, {-1, 5, -5, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 32},
			{/*intp_sft*/0, 6},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{0, -1, 5, 5, -1, 0, 0, 0}, {-1, -2, 96, 42, -7, 0, 0, 0},},
			{/*intp_rnd*/64, 0}, //{0,512}
			{/*intp_sft*/0, 10},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, IS_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 5, 5, -1, 0, 0, 0, 0}, {-1, 5, -5, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 32},
			{/*intp_sft*/0, 6},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/1, 0},
		},
		{/*H0V2*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 5, 5, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/4, 0},
			{/*intp_sft*/3, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, -2, 96, 42, -7, 0, 0, 0}, {0, -1, 5, 5, -1, 0, 0, 0},},
			{/*intp_rnd*/64, 0}, //{0,512}
			{/*intp_sft*/0, 10},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 5, 5, -1, 0, 0, 0, 0}, {-1, 5, 5, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 32},
			{/*intp_sft*/0, 6},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{0, -7, 42, 96, -2, -1, 0, 0}, {0, -1, 5, 5, -1, 0, 0, 0},},
			{/*intp_rnd*/64, 0}, //{0,512}
			{/*intp_sft*/0, 10},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V3*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{0, -7, 42, 96, -2, -1, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V3*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, IS_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 5, 5, -1, 0, 0, 0, 0}, {-1, 5, -5, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 32},
			{/*intp_sft*/0, 6},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/0, 1},
		},
		{/*H2V3*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{0, -1, 5, 5, -1, 0, 0, 0}, {0, -7, 42, 96, -2, -1, 0, 0},},
			{/*intp_rnd*/64, 0}, //{0,512}
			{/*intp_sft*/0, 10},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V3*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, IS_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 5, 5, -1, 0, 0, 0, 0}, {-1, 5, -5, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/0, 32},
			{/*intp_sft*/0, 6},
			{/*intp_sintp*/0, 1},
			{/*intp_srnd*/0, 1},
			{/*intp_sbias*/1, 1},
		},
	},

	{
		/************* VP6_QPEL ***************/
		{/*H0V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-4, 109, 24, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-4, 68, 68, -4, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 24, 109, -4, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V1*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-4, 109, 24, -1, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-4, 109, 24, -1, 0, 0, 0, 0}, {-4, 109, 24, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-4, 68, 68, -4, 0, 0, 0, 0}, {-4, 109, 24, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 24, 109, -4, 0, 0, 0, 0}, {-4, 109, 24, -1, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V2*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-4, 68, 68, -4, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-4, 109, 24, -1, 0, 0, 0, 0}, {-4, 68, 68, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-4, 68, 68, -4, 0, 0, 0, 0}, {-4, 68, 68, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 24, 109, -4, 0, 0, 0, 0}, {-4, 68, 68, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V3*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 24, 109, -4, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V3*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-4, 109, 24, -1, 0, 0, 0, 0}, {-1, 24, 109, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V3*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-4, 68, 68, -4, 0, 0, 0, 0}, {-1, 24, 109, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V3*/
			TAP4, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 24, 109, -4, 0, 0, 0, 0}, {-1, 24, 109, -4, 0, 0, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
	},

	{
		/************* VP8_QPEL ***************/
		{/*H0V0*/
			TAP6, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{2, -11, 108, 36, -8, 1, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{3, -16, 77, 77, -16, 3, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, -8, 36, 108, -11, 2, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V1*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{2, -11, 108, 36, -8, 1, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{2, -11, 108, 36, -8, 1, 0, 0}, {2, -11, 108, 36, -8, 1, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, -16, 77, 77, -16, 3, 0, 0}, {2, -11, 108, 36, -8, 1, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -8, 36, 108, -11, 2, 0, 0}, {2, -11, 108, 36, -8, 1, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V2*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{3, -16, 77, 77, -16, 3, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{2, -11, 108, 36, -8, 1, 0, 0}, {3, -16, 77, 77, -16, 3, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, -16, 77, 77, -16, 3, 0, 0}, {3, -16, 77, 77, -16, 3, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -8, 36, 108, -11, 2, 0, 0}, {3, -16, 77, 77, -16, 3, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V3*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, -8, 36, 108, -11, 2, 0, 0}, {0},},
			{/*intp_rnd*/64, 0},
			{/*intp_sft*/7, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V3*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{2, -11, 108, 36, -8, 1, 0, 0}, {1, -8, 36, 108, -11, 2, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V3*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, -16, 77, 77, -16, 3, 0, 0}, {1, -8, 36, 108, -11, 2, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V3*/
			TAP6, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, -8, 36, 108, -11, 2, 0, 0}, {1, -8, 36, 108, -11, 2, 0, 0},},
			{/*intp_rnd*/64, 64},
			{/*intp_sft*/7, 7},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
	},

	{
		/************* VP8_EPEL ***************/
		{/*H0V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/0},
		{/*H2V0*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/4, 0},
			{/*intp_sft*/3, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V0*/0},
		{/*H0V1*/0},
		{/*H1V1*/0},
		{/*H2V1*/0},
		{/*H3V1*/0},
		{/*H0V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/4, 0},
			{/*intp_sft*/3, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V2*/0},
		{/*H2V2*/
			TAP2, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/4, 4},
			{/*intp_sft*/3, 3},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V2*/0},
		{/*H0V3*/0},
		{/*H1V3*/0},
		{/*H2V3*/0},
		{/*H3V3*/0},
	},

	{
		/************* VP8_BIL ***************/
		{/*H0V0*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/2, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V0*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V0*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/2, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H0V1*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/2, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V1*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {3, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/2, 2},
			{/*intp_sft*/2, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V1*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {3, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/1, 2},
			{/*intp_sft*/1, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V1*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {3, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/2, 2},
			{/*intp_sft*/2, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H0V2*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/1, 0},
			{/*intp_sft*/1, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V2*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/2, 1},
			{/*intp_sft*/2, 1},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V2*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/1, 1},
			{/*intp_sft*/1, 1},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V2*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/2, 1},
			{/*intp_sft*/2, 1},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H0V3*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/2, 0},
			{/*intp_sft*/2, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V3*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{3, 1, 0, 0, 0, 0, 0, 0}, {1, 3, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/2, 2},
			{/*intp_sft*/2, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H2V3*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 1, 0, 0, 0, 0, 0, 0}, {1, 3, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/1, 2},
			{/*intp_sft*/1, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H3V3*/
			TAP2, {IS_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{1, 3, 0, 0, 0, 0, 0, 0}, {1, 3, 0, 0, 0, 0, 0, 0},},
			{/*intp_rnd*/2, 2},
			{/*intp_sft*/2, 2},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
	},

	{
		/************* VP8_FPEL ***************/
		{/*H0V0*/0},
		{/*H1V0*/0},
		{/*H2V0*/0},
		{/*H3V0*/0},
		{/*H0V1*/0},
		{/*H1V1*/0},
		{/*H2V1*/0},
		{/*H3V1*/0},
		{/*H0V2*/0},
		{/*H1V2*/0},
		{/*H2V2*/0},
		{/*H3V2*/0},
		{/*H0V3*/0},
		{/*H1V3*/0},
		{/*H2V3*/0},
		{/*H3V3*/0},
	},

	{
		/************* HEVC_QPEL ***************/
		{/*H0V0*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 4, -10, 58, 17, -5, 1, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{-1, 4, -11, 40, 40, -11, 4, -1}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, 0},
			{/*intp_coef*/{0, 1, -5, 17, 58, -10, 4, -1}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V1*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 4, -10, 58, 17, -5, 1, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V1*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 4, -10, 58, 17, -5, 1, 0}, {-1, 4, -10, 58, 17, -5, 1, 0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/0, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V1*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 4, -11, 40, 40, -11, 4, -1}, {-1, 4, -10, 58, 17, -5, 1, 0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/0, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V1*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{0, 1, -5, 17, 58, -10, 4, -1}, {-1, 4, -10, 58, 17, -5, 1, 0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/0, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V2*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{-1, 4, -11, 40, 40, -11, 4, -1}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 4, -10, 58, 17, -5, 1, 0}, {-1, 4, -11, 40, 40, -11, 4, -1},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/0, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 4, -11, 40, 40, -11, 4, -1}, {-1, 4, -11, 40, 40, -11, 4, -1},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/0, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{0, 1, -5, 17, 58, -10, 4, -1}, {-1, 4, -11, 40, 40, -11, 4, -1},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/0, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V3*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_VDIR, 0},
			{/*intp_coef*/{0, 1, -5, 17, 58, -10, 4, -1}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V3*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 4, -10, 58, 17, -5, 1, 0}, {0, 1, -5, 17, 58, -10, 4, -1},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/0, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V3*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-1, 4, -11, 40, 40, -11, 4, -1}, {0, 1, -5, 17, 58, -10, 4, -1},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/0, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V3*/
			TAP8, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 1}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{0, 1, -5, 17, 58, -10, 4, -1}, {0, 1, -5, 17, 58, -10, 4, -1},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/0, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
	},

	{
		/************* HEVC_EPEL ***************/
		{/*H0V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0},
			{/*intp_coef*/{1, 0, 0, 0, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0},
			{/*intp_srnd*/0},
			{/*intp_sbias*/0},
		},
		{/*H1V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-2, 58, 10, -2, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-4, 54, 16, -2, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-6, 46, 28, -4, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H4V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-4, 36, 36, -4, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H5V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-4, 28, 46, -6, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H6V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-2, 16, 54, -4, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H7V0*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/1, 0}, {INTP_HDIR, INTP_VDIR},
			{/*intp_coef*/{-2, 10, 58, -2, 0, 0, 0, 0}, {0},},
			{/*intp_rnd*/32, 0},
			{/*intp_sft*/6, 12},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V2*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V2*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V2*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V2*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H0V3*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H1V3*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H2V3*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
		{/*H3V3*/
			TAP4, {NOT_TCS, NOT_SCS}, NOT_HLDGL, NOT_AVSDGL,
			{/*intp*/0, 0}, {0, 0},
			{/*intp_coef*/{0}, {0},},
			{/*intp_rnd*/0, 0},
			{/*intp_sft*/0, 0},
			{/*intp_sintp*/0, 0},
			{/*intp_srnd*/0, 0},
			{/*intp_sbias*/0, 0},
		},
	},

};
#endif

#endif /*__HELIX_H__*/
