#ifndef _REGS_IPU_H_
#define _REGS_IPU_H_

#ifdef __KERNEL__

	/* Module for CLKGR */
	#define IDCT_CLOCK      (1 << 27)
	#define DBLK_CLOCK      (1 << 26)
	#define ME_CLOCK        (1 << 25)
	#define MC_CLOCK        (1 << 24)
	#define IPU_CLOCK       (1 << 13)

	/* Register offset */
	#define  IPU_FM_CTRL                0x0
	#define  IPU_STATUS                 0x4
	#define  IPU_D_FMT                  0x8
	#define  IPU_Y_ADDR                 0xc
	#define  IPU_U_ADDR                 0x10
	#define  IPU_IN_FM_GS               0x18
	#define  IPU_Y_STRIDE               0x1c
	#define  IPU_UV_STRIDE              0x20

	#define  IPU_OUT_ADDR               0x24
	#define  IPU_OUT_STRIDE             0x2c
	#define  IPU_OUT_V_ADDR             0x30
	#define  IPU_OUT_V_STRIDE           0x34

	#define  IPU_REG_CTRL               0x64
	#define  IPU_TRIGGER                0x74
	#define  IPU_GLB_CTRL               0x7C

	#define  IPU_NV_OUT_ADDR            0xb4
	#define  IPU_NV_OUT_STRIDE          0xb8
	#define  IPU_OSD_IN_CH0_Y_ADDR      0xbc
	#define  IPU_OSD_IN_CH0_Y_STRIDE    0xc0
	#define  IPU_OSD_IN_CH0_UV_ADDR     0xc4
	#define  IPU_OSD_IN_CH0_UV_STRIDE   0xc8
	#define  IPU_OSD_IN_CH1_Y_ADDR      0xcc
	#define  IPU_OSD_IN_CH1_Y_STRIDE    0xd0
	#define  IPU_OSD_IN_CH1_UV_ADDR     0xd4
	#define  IPU_OSD_IN_CH1_UV_STRIDE   0xd8
	#define  IPU_OSD_IN_CH2_Y_ADDR      0xdc
	#define  IPU_OSD_IN_CH2_Y_STRIDE    0xe0
	#define  IPU_OSD_IN_CH2_UV_ADDR     0xe4
	#define  IPU_OSD_IN_CH2_UV_STRIDE   0xe8
	#define  IPU_OSD_IN_CH3_Y_ADDR      0xec
	#define  IPU_OSD_IN_CH3_Y_STRIDE    0xf0
	#define  IPU_OSD_IN_CH3_UV_ADDR     0xf4
	#define  IPU_OSD_IN_CH3_UV_STRIDE   0xf8
	#define  IPU_OSD_CH0_GS             0xfc
	#define  IPU_OSD_CH1_GS             0x100
	#define  IPU_OSD_CH2_GS             0x104
	#define  IPU_OSD_CH3_GS             0x108
	#define  IPU_OSD_CH0_POS            0x10c
	#define  IPU_OSD_CH1_POS            0x110
	#define  IPU_OSD_CH2_POS            0x114
	#define  IPU_OSD_CH3_POS            0x118
	#define  IPU_OSD_CH0_PARA           0x11c
	#define  IPU_OSD_CH1_PARA           0x120
	#define  IPU_OSD_CH2_PARA           0x124
	#define  IPU_OSD_CH3_PARA           0x128
	#define  IPU_OSD_CH_BK_PARA         0x12c

	#define  IPU_OSD_CH0_BAK_ARGB       0x130
	#define  IPU_OSD_CH1_BAK_ARGB       0x134
	#define  IPU_OSD_CH2_BAK_ARGB       0x138
	#define  IPU_OSD_CH3_BAK_ARGB       0x13c
	#define  IPU_OSD_CH_B_BAK_ARGB      0x140

	#define  IPU_CH0_CSC_C0_COEF        0x148
	#define  IPU_CH0_CSC_C1_COEF        0x14c
	#define  IPU_CH0_CSC_C2_COEF        0x150
	#define  IPU_CH0_CSC_C3_COEF        0x154
	#define  IPU_CH0_CSC_C4_COEF        0x158
	#define  IPU_CH0_CSC_OFSET_PARA     0x15c

	#define  IPU_CH1_CSC_C0_COEF        0x160
	#define  IPU_CH1_CSC_C1_COEF        0x164
	#define  IPU_CH1_CSC_C2_COEF        0x168
	#define  IPU_CH1_CSC_C3_COEF        0x16c
	#define  IPU_CH1_CSC_C4_COEF        0x170
	#define  IPU_CH1_CSC_OFSET_PARA     0x174

	#define  IPU_CH2_CSC_C0_COEF        0x178
	#define  IPU_CH2_CSC_C1_COEF        0x17c
	#define  IPU_CH2_CSC_C2_COEF        0x180
	#define  IPU_CH2_CSC_C3_COEF        0x184
	#define  IPU_CH2_CSC_C4_COEF        0x188
	#define  IPU_CH2_CSC_OFSET_PARA     0x18c

	#define  IPU_CH3_CSC_C0_COEF        0x190
	#define  IPU_CH3_CSC_C1_COEF        0x194
	#define  IPU_CH3_CSC_C2_COEF        0x198
	#define  IPU_CH3_CSC_C3_COEF        0x19c
	#define  IPU_CH3_CSC_C4_COEF        0x1a0
	#define  IPU_CH3_CSC_OFSET_PARA     0x1a4

	#define  IPU_RD_ARB_CTL             0x1a8
	#define  IPU_CLK_NUM_ONE_FRA        0x1ac
	#define  IPU_OSD_LAY_PADDING_ARGB   0x1b0
	#define  IPU_TEST_1B4               0x1b4

	/* SET CH_BK_PARA Register*/
	#define  CH_BK_EN                   (1 << 0)
	#define  CH_BK_PIXEL_ALPHA          (0 << 2) //default
	#define  CH_BK_GLOBAL_ALPHA         (1 << 1)
	#define  CH_BK_PALPHA_X_GALPHA      (1 << 2)
	#define  CH_BKCH_BK_GLO_A(val)      ((val) << 3)

	#define  CH_BK_PIC_TYPE             ( 11 )
	#define  CH_BK_PIC_TYPE_ARGB        (0 << CH_BK_PIC_TYPE)
	#define  CH_BK_PIC_TYPE_RGB         (1 << CH_BK_PIC_TYPE)
	#define  CH_BK_PIC_TYPE_NV12        (2 << CH_BK_PIC_TYPE)
	#define  CH_BK_PIC_TYPE_NV21        (3 << CH_BK_PIC_TYPE)

	/*
	0000: ARGB          0001: ARBG
	0010: AGRB          0011: AGBR
	0100: ABRG          0101: ABGR
	1000: RGBA          1001: RBGA
	1010: GRBA          1011: GBRA
	1100: BRGA          1101: BGRA
	*/
	#define  CH_BK_ARGB_TYPE            ( 14 )
	#define  CH_BK_ARGB_TYPE_ARGB       (0 << CH_BK_ARGB_TYPE)
	#define  CH_BK_ARGB_TYPE_ARBG       (1 << CH_BK_ARGB_TYPE)
	#define  CH_BK_ARGB_TYPE_AGRB       (2 << CH_BK_ARGB_TYPE)
	#define  CH_BK_ARGB_TYPE_AGBR       (3 << CH_BK_ARGB_TYPE)
	#define  CH_BK_ARGB_TYPE_ABRG       (4 << CH_BK_ARGB_TYPE)
	#define  CH_BK_ARGB_TYPE_ABGR       (5 << CH_BK_ARGB_TYPE)
	#define  CH_BK_ARGB_TYPE_RGBA       (8 << CH_BK_ARGB_TYPE)
	#define  CH_BK_ARGB_TYPE_RBGA       (9 << CH_BK_ARGB_TYPE)
	#define  CH_BK_ARGB_TYPE_GRBA       (0xa << CH_BK_ARGB_TYPE)
	#define  CH_BK_ARGB_TYPE_GBRA       (0xb << CH_BK_ARGB_TYPE)
	#define  CH_BK_ARGB_TYPE_BRGA       (0xc << CH_BK_ARGB_TYPE)
	#define  CH_BK_ARGB_TYPE_BGRA       (0xd << CH_BK_ARGB_TYPE)

	#define  CH_BK_PREM                 (1 << 18)
	#define  CH_BK_MASK_EN              (1 << 19)
	#define  CH_BK_WHOLE_PIC_EN         (1 << 20)

	/*OSD_CHN_EN*/
	#define CHOOSE_OSD_CHN_RESIZE   (1 << 5)

	/* IPU_GLB_CTRL Register */
	#define IRQ_EN          (1 << 0)

	/* IPU_OSD_CTRL Register */
	#define OSD_PM          (1 << 2)

	/* IPU_TRIG Register */
	#define IPU_RUN         (1 << 0)
	#define IPU_STOP        (1 << 1)
	#define IPU_RESET       (1 << 3)

	/* IPU_D_FMT Register */
	#define BLK_SEL         (1 << 4)
	#define RGB_POS         (1 << 5)
	#define AL_PIX_EN       (1 << 6)

	/* REG_STATUS field define */
	#define OUT_END         (1 << 0)
	#define SIZE_ERR        (1 << 2)
	#define ID              (1 << 16)

	#define MSTATUS_SFT     (4)
	#define MSTATUS_MSK     (3)
	#define MSTATUS_IPU_FREE        (0 << MSTATUS_SFT)
	#define MSTATUS_IPU_RUNNING     (1 << MSTATUS_SFT)
	#define MSTATUS                 (2 << MSTATUS_SFT)

	/* REG_IPU_ADDR_CTRL */
	#define Y_RY            (1<<0)
	#define U_RY            (1<<1)
	#define V_RY            (1<<2)
	#define D_RY            (1<<3)
	#define PTS_RY          (1<<4)
	#define CTRL_RY         (1<<6)
	#define DF_RY           (1<<7)
	#define NV_D_RY         (1 << 17)
	#define NV_STR_RY       (1 << 20)
	#define OSD_CH0_RY      (1 << 21)
	#define OSD_CH1_RY      (1 << 22)
	#define OSD_CH2_RY      (1 << 23)
	#define OSD_CH3_RY      (1 << 24)
	#define RD_ARB_RY       (1 << 26)
	#define OSD_BK_RY       (1 << 27)

	/* REG_TLB_MONITOR */
	#define MIS_CNT_SFT     (1)
	#define MIS_CNT_MSK     (0x3FF)

	/* REG_TLB_CTRL */
	#define SRC_PAGE_SFT        (0)
	#define SRC_PAGE_MSK        (0xF)
	#define SRC_BURST_SFT       (4)
	#define SRC_BURST_MSK       (0xF)
	#define DEST_PAGE_SFT       (16)
	#define DEST_PAGE_MSK       (0xF)
	#define DEST_BURST_SFT      (20)
	#define DEST_BURST_MSK      (0xF)

	/* OSD_CTRL field define */
	#define GLB_ALPHA(val)  ((val) << 8)
	#define MOD_OSD(val)    ((val) << 0)

	/* FM_XYOFT field define */
	#define SCREEN_YOFT(val)    ((val) << 16)
	#define SCREEN_XOFT(val)    ((val) << 0)

	/* REG_IN_GS field define */
	#define IN_FM_W(val)    ((val) << 16)
	#define IN_FM_H(val)    ((val) << 0)

	/* REG_OUT_GS field define */
	#define OUT_FM_W(val)    ((val) << 16)
	#define OUT_FM_H(val)    ((val) << 0)

	/* REG_UV_STRIDE field define */
	#define U_STRIDE(val)     ((val) << 16)
	#define V_STRIDE(val)     ((val) << 0)

	/* REG_RSZ_COEF field definei */
	#define HCOEF(val)          ((val) << 16)
	#define VCOEF(val)          ((val) << 0)

	#if 1
		#define YUV_CSC_C0 0x4A8        /* 1.164 * 1024 */
		#define YUV_CSC_C1 0x662        /* 1.596 * 1024 */
		#define YUV_CSC_C2 0x191        /* 0.392 * 1024 */
		#define YUV_CSC_C3 0x341        /* 0.813 * 1024 */
		#define YUV_CSC_C4 0x811        /* 2.017 * 1024 */

		#define YUV_CSC_OFFSET_PARA         0x800010  /* chroma,luma */
	#else
		#define YUV_CSC_C0 0x400
		#define YUV_CSC_C1 0x59C
		#define YUV_CSC_C2 0x161
		#define YUV_CSC_C3 0x2DC
		#define YUV_CSC_C4 0x718
	#endif

	/* select ch */
	#define BK_OSD_PIC_MASK             (18)
	#define BK_OSD_PIC_NV21         ( 1 << BK_OSD_PIC_MASK )
	#define BK_OSD_PIC_NV12         ( 2 << BK_OSD_PIC_MASK )
	#define BK_OSD_PIC_ARGB         ( 3 << BK_OSD_PIC_MASK )
	#define CH3_OSD_PIC_MASK            (15)
	#define CH3_OSD_PIC_NV21        ( 1 << CH3_OSD_PIC_MASK )
	#define CH3_OSD_PIC_NV12        ( 2 << CH3_OSD_PIC_MASK )
	#define CH3_OSD_PIC_ARGB        ( 3 << CH3_OSD_PIC_MASK )
	#define CH2_OSD_PIC_MASK            (12)
	#define CH2_OSD_PIC_NV21        ( 1 << CH2_OSD_PIC_MASK )
	#define CH2_OSD_PIC_NV12        ( 2 << CH2_OSD_PIC_MASK )
	#define CH2_OSD_PIC_ARGB        ( 3 << CH2_OSD_PIC_MASK )
	#define CH1_OSD_PIC_MASK            (9)
	#define CH1_OSD_PIC_NV21        ( 1 << CH1_OSD_PIC_MASK )
	#define CH1_OSD_PIC_NV12        ( 2 << CH1_OSD_PIC_MASK )
	#define CH1_OSD_PIC_ARGB        ( 3 << CH1_OSD_PIC_MASK )
	#define CH0_OSD_PIC_MASK            (6)
	#define CH0_OSD_PIC_NV21        ( 1 << CH0_OSD_PIC_MASK )
	#define CH0_OSD_PIC_NV12        ( 2 << CH0_OSD_PIC_MASK )
	#define CH0_OSD_PIC_ARGB        ( 3 << CH0_OSD_PIC_MASK )
	#define OSD_CHN_EN_MASK             (0)
	#define OSD_CHN_EN_CH0          ( 1  << OSD_CHN_EN_MASK )
	#define OSD_CHN_EN_CH1          ( 2  << OSD_CHN_EN_MASK )
	#define OSD_CHN_EN_CH2          ( 4  << OSD_CHN_EN_MASK )
	#define OSD_CHN_EN_CH3          ( 8  << OSD_CHN_EN_MASK )
	#define OSD_CHN_EN_BKG          ( 16 << OSD_CHN_EN_MASK )
	#define OSD_CHN_EN_DST          ( 32 << OSD_CHN_EN_MASK )

#endif  /* #ifdef __KERNEL__ */

///////////////////////////////////////////

/* Data Format Register, export to libipu */
#define RGB_888_OUT_FMT             ( 1 << 25 )

#define RGB_OUT_OFT_BIT             ( 22 )
#define RGB_OUT_OFT_MASK            ( 7 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_RGB             ( 0 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_RBG             ( 1 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_GBR             ( 2 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_GRB             ( 3 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_BRG             ( 4 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_BGR             ( 5 << RGB_OUT_OFT_BIT )

#define OUT_FMT_BIT                 ( 19 )
#define OUT_FMT_MASK                ( 3 <<  OUT_FMT_BIT )
#define OUT_FMT_RGB555              ( 0 <<  OUT_FMT_BIT )
#define OUT_FMT_RGB565              ( 1 <<  OUT_FMT_BIT )
#define OUT_FMT_RGB888              ( 2 <<  OUT_FMT_BIT )
#define OUT_FMT_YUV422              ( 3 <<  OUT_FMT_BIT )
#define OUT_FMT_RGBAAA              ( 4 <<  OUT_FMT_BIT )
#define OUT_FMT_NV12                ( 6 <<  OUT_FMT_BIT )
#define OUT_FMT_NV21                ( 7 <<  OUT_FMT_BIT )
#define OUT_FMT_HSV                 ( 1 << 2 )                    /*Add HSV*/

#define YUV_PKG_OUT_OFT_BIT         ( 16 )
#define YUV_PKG_OUT_OFT_MASK        ( 7 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y1UY0V      ( 0 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y1VY0U      ( 1 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_UY1VY0      ( 2 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_VY1UY0      ( 3 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y0UY1V      ( 4 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y0VY1U      ( 5 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_UY0VY1      ( 6 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_VY0UY1      ( 7 << YUV_PKG_OUT_OFT_BIT )

#define IN_OFT_BIT                  ( 2 )
#define IN_OFT_MASK                 ( 3 << IN_OFT_BIT )
#define IN_OFT_Y1UY0V               ( 0 << IN_OFT_BIT )
#define IN_OFT_Y1VY0U               ( 1 << IN_OFT_BIT )
#define IN_OFT_UY1VY0               ( 2 << IN_OFT_BIT )
#define IN_OFT_VY1UY0               ( 3 << IN_OFT_BIT )

#define IN_FMT_BIT                  ( 0 )
#define IN_FMT_MASK                 ( 7 << IN_FMT_BIT )
#define IN_FMT_YUV420               ( 0 << IN_FMT_BIT )
#define IN_FMT_YUV420_B             ( 4 << IN_FMT_BIT )
#define IN_FMT_YUV422               ( 1 << IN_FMT_BIT )
#define IN_FMT_YUV444               ( 2 << IN_FMT_BIT )
#define IN_FMT_RGB_555              ( 0 << IN_FMT_BIT )
#define IN_FMT_RGB_888              ( 2 << IN_FMT_BIT )
#define IN_FMT_RGB_565              ( 3 << IN_FMT_BIT )
#define IN_FMT_YUV411               ( 3 << IN_FMT_BIT )

//hwang add
#define NV_SEL                      ( 1 << 7 )
#define NV_MODE_NV12                ( 0 << 8 )
#define NV_MODE_NV21                ( 1 << 8 )

#define __enable_blk_mode()     reg_bit_set(ipu, IPU_D_FMT, BLK_SEL)
#define __disable_blk_mode()    reg_bit_clr(ipu, IPU_D_FMT, BLK_SEL)
#define __clear_ipu_out_end()   reg_bit_clr(ipu, IPU_STATUS, OUT_END)
#define __ipu_enable_irq()                      \
	do {unsigned int val = IRQ_EN ;             \
		reg_bit_set(ipu, IPU_GLB_CTRL, val);    \
	}while(0)
#define __ipu_disable_irq()                     \
	do {unsigned int val = IRQ_EN ;             \
		reg_bit_clr(ipu, IPU_GLB_CTRL, val);    \
	}while(0)
#define __start_ipu()           reg_bit_set(ipu, IPU_TRIGGER, IPU_RUN)
#define __reset_ipu()           reg_bit_set(ipu, IPU_TRIGGER, IPU_RESET)

#endif // _REGS_IPU_H_
