/*
 * JZ4780 ddr definitions
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __DDR_H__
#define __DDR_H__

#include <soc/base.h>

/*
 * DDR Controller Registers
 **/
#define DDR_MEM_PHY_BASE        0x20000000

#define DDRC_AHB2_OFFSET    0xb34f0000
#define DDRC_APB_OFFSET     0xb3012000
#define DDR_PHY_OFFSET  (-0x4e0000 + 0x1000)

#define DDRC_STATUS     0x0
#define DDRC_CFG        0x8
#define DDRC_CTRL       0x10
#define DDRC_LMR        0x18
#define DDRC_DLP        0x20
#define DDRC_AUTOSR_EN      0x28
#define DDRC_AUTOSR_CNT     0x30
#define DDRC_REFCNT     0x38
#define DDRC_DBGINFO        0xE8
#define DDRC_TIMING(n)      (0x40 + 8 * (n - 1))
#define DDRC_MMAP0      0x78
#define DDRC_MMAP1      0x80
#define DDRC_HREGPRO        0xd8
#define DDRC_DWCFG      (DDRC_APB_OFFSET + 0x00)
#define DDRC_DWSTATUS       (DDRC_APB_OFFSET + 0x04)
#define DDRC_REMAP(n)       (DDRC_APB_OFFSET + 0x08 + 4 * (n - 1))
#define DDRC_CCHC(n)        (DDRC_APB_OFFSET + 0x20 + 4 * n)
#define DDRC_CGUC0      (DDRC_APB_OFFSET + 0x64)
#define DDRC_CGUC1      (DDRC_APB_OFFSET + 0x68)
#define DDRC_PREGPRO        (DDRC_APB_OFFSET + 0x6c)

/*
 * DDR Innophy registers
 * */
#define DDRP_INNOPHY_PHY_RST            (DDR_PHY_OFFSET + 0x000)
#define DDRP_INNOPHY_MEM_CFG            (DDR_PHY_OFFSET + 0x004)
#define DDRP_INNOPHY_DQ_WIDTH           (DDR_PHY_OFFSET + 0x07c)
#define DDRP_INNOPHY_CL             (DDR_PHY_OFFSET + 0x014)
#define DDRP_INNOPHY_CWL            (DDR_PHY_OFFSET + 0x01c)
#define DDRP_INNOPHY_AL             (DDR_PHY_OFFSET + 0x018)
#define DDRP_INNOPHY_PLL_FBDIV          (DDR_PHY_OFFSET + 0x080)
#define DDRP_INNOPHY_PLL_CTRL           (DDR_PHY_OFFSET + 0x084)
#define DDRP_INNOPHY_PLL_PDIV           (DDR_PHY_OFFSET + 0x088)
#define DDRP_INNOPHY_PLL_LOCK           (DDR_PHY_OFFSET + 0xc8)
#define DDRP_INNOPHY_TRAINING_CTRL      (DDR_PHY_OFFSET + 0x008)
#define DDRP_INNOPHY_CALIB_DONE         (DDR_PHY_OFFSET + 0xcc)
#define DDRP_INNOPHY_CALIB_DELAY_AL     (DDR_PHY_OFFSET + 0x190)
#define DDRP_INNOPHY_CALIB_DELAY_AH     (DDR_PHY_OFFSET + 0x194)
#define DDRP_INNOPHY_CALIB_BYPASS_AL        (DDR_PHY_OFFSET + 0x118)
#define DDRP_INNOPHY_CALIB_BYPASS_AH        (DDR_PHY_OFFSET + 0x158)
#define DDRP_INNOPHY_WL_MODE1           (DDR_PHY_OFFSET + 0x00c)
#define DDRP_INNOPHY_WL_MODE2           (DDR_PHY_OFFSET + 0x010)
#define DDRP_INNOPHY_WL_DONE            (DDR_PHY_OFFSET + 0x0c0)
#define DDRP_INNOPHY_INIT_COMP          (DDR_PHY_OFFSET + 0x0d0)
#define DDRP_CMD_NRCOMP     (DDR_PHY_OFFSET + 0x40)
#define DDRP_CMD_PRCOMP     (DDR_PHY_OFFSET + 0x44)
#define DDRP_CK_NRCOMP      (DDR_PHY_OFFSET + 0x48)
#define DDRP_CK_PRCOMP      (DDR_PHY_OFFSET + 0x4c)
#define DDRP_PDODT_RES0     (DDR_PHY_OFFSET + 0x100)
#define DDRP_PUODT_RES0     (DDR_PHY_OFFSET + 0x104)
#define DDRP_PDDRI_RES0     (DDR_PHY_OFFSET + 0x108)
#define DDRP_PUDRI_RES0     (DDR_PHY_OFFSET + 0x10c)
#define DDRP_PDODT_RES1     (DDR_PHY_OFFSET + 0x140)
#define DDRP_PUODT_RES1     (DDR_PHY_OFFSET + 0x144)
#define DDRP_PDDRI_RES1     (DDR_PHY_OFFSET + 0x148)
#define DDRP_PUDRI_RES1     (DDR_PHY_OFFSET + 0x14c)

/*
 * DDRC REGISTER BITS DEFINE
 * */

/* DDRC Status Register */
#define DDRC_DSTATUS_MISS   (1 << 6)
#define DDRC_ST_DPDN        (1 << 5) /* 0 DDR memory is NOT in deep-power-down state
                                        1 DDR memory is in deep-power-down state */
#define DDRC_ST_PDN     (1 << 4) /* 0 DDR memory is NOT in power-down state
                                    1 DDR memory is in power-down state */
#define DDRC_ST_AREF        (1 << 3) /* 0 DDR memory is NOT in auto-refresh state
                                        1 DDR memory is in auto-refresh state */
#define DDRC_ST_SREF        (1 << 2) /* 0 DDR memory is NOT in self-refresh state
                                        1 DDR memory is in self-refresh state */
#define DDRC_ST_CKE1        (1 << 1) /* 0 CKE1 Pin is low
                                        1 CKE1 Pin is high */
#define DDRC_ST_CKE0        (1 << 0) /* 0 CKE0 Pin is low
                                        1 CKE0 Pin is high */

/* DDRC Configure Register */
#define DDRC_CFG_ROW1_BIT   29 /* Row Address width. */
#define DDRC_CFG_ROW1_MASK  (0x7 << DDRC_CFG_ROW1_BIT)
#define DDRC_CFG_COL1_BIT   26 /* Row Address width. */
#define DDRC_CFG_COL1_MASK  (0x7 << DDRC_CFG_COL1_BIT)
#define DDRC_CFG_BA1        25 /* Bank Address width of DDR memory */
#define DDRC_CFG_IMBA       (1 << 16)
#define DDRC_CFG_ROW0_BIT   13 /* Row Address width. */
#define DDRC_CFG_ROW0_MASK  (0x7 << DDRC_CFG_ROW0_BIT)
#define DDRC_CFG_COL0_BIT   10 /* Row Address width. */
#define DDRC_CFG_COL0_MASK  (0x7 << DDRC_CFG_COL0_BIT)
#define DDRC_CFG_BA0        9 /* Bank Address width of DDR memory */

#define DDRC_CFG_TYPE_BIT   3
#define DDRC_CFG_TYPE_MASK  (0x7 << DDRC_CFG_TYPE_BIT)
#define DDRC_CFG_TYPE_DDR1  (2 << DDRC_CFG_TYPE_BIT)
#define DDRC_CFG_TYPE_MDDR  (3 << DDRC_CFG_TYPE_BIT)
#define DDRC_CFG_TYPE_DDR2  (4 << DDRC_CFG_TYPE_BIT)
#define DDRC_CFG_TYPE_LPDDR2    (5 << DDRC_CFG_TYPE_BIT)
#define DDRC_CFG_TYPE_DDR3  (6 << DDRC_CFG_TYPE_BIT)
#define DDRC_CFG_TYPE_LPDDR3    (7 << DDRC_CFG_TYPE_BIT)

#define DDRC_CFG_ODTEN      (1 << 2)  /* ODT EN */
#define DDRC_CFG_CS1EN      (1 << 1)  /* DDR Chip-Select-1 Enable */
#define DDRC_CFG_CS0EN      (1 << 0)  /* DDR Chip-Select-0 Enable */

/* DDRC Control Register */
#define DDRC_CTRL_DFI_RST   (1 << 23)
#define DDRC_CTRL_DLL_RST   (1 << 22)
#define DDRC_CTRL_CTL_RST   (1 << 21)
#define DDRC_CTRL_CFG_RST   (1 << 20)
#define DDRC_CTRL_ACTPD     (1 << 15) /* 0 Precharge all banks before entering power-down
                                         1 Do not precharge banks before entering power-down */
#define DDRC_CTRL_PDT_BIT   12 /* Power-Down Timer */
#define DDRC_CTRL_PDT_MASK  (0x7 << DDRC_CTRL_PDT_BIT)
#define DDRC_CTRL_PDT_DIS   (0 << DDRC_CTRL_PDT_BIT) /* power-down disabled */
#define DDRC_CTRL_PDT_8     (1 << DDRC_CTRL_PDT_BIT) /* Enter power-down after 8 tCK idle */
#define DDRC_CTRL_PDT_16    (2 << DDRC_CTRL_PDT_BIT) /* Enter power-down after 16 tCK idle */
#define DDRC_CTRL_PDT_32    (3 << DDRC_CTRL_PDT_BIT) /* Enter power-down after 32 tCK idle */
#define DDRC_CTRL_PDT_64    (4 << DDRC_CTRL_PDT_BIT) /* Enter power-down after 64 tCK idle */
#define DDRC_CTRL_PDT_128   (5 << DDRC_CTRL_PDT_BIT) /* Enter power-down after 128 tCK idle */

#define DDRC_CTRL_PD_CCE    (1 << 7) /* Power down clk freq change enable */
#define DDRC_CTRL_DPD       (1 << 6) /* 1 Drive external MDDR device entering Deep-Power-Down mode */
#define DDRC_CTRL_SR        (1 << 5) /* 1 Drive external DDR device entering self-refresh mode
                                        0 Drive external DDR device exiting self-refresh mode */
#define DDRC_CTRL_SR_CCE    (1 << 4) /* Self refresh clk stop request enable */
#define DDRC_CTRL_CKE       (1 << 1) /* 0 Not set CKE Pin High, 1 Set CKE Pin HIGH */
#define DDRC_CTRL_RESET     (1 << 0) /* 0 End resetting ddrc_controller, 1 Resetting ddrc_controller */

/* DDRC DFI low power handshake control register */
#define DDRC_DDLP_TCTLUDP_BIT   24
#define DDRC_DDLP_TCTLUDP_FF    (0xff << DDRC_DDLP_TCTLUDP_BIT)

/* DDRC Load-Mode-Register */
#define DDRC_LMR_DDR_ADDR_BIT   12 /* When performing a DDR command, DDRC_ADDR[13:0]
                                      corresponding to external DDR address Pin A[13:0] */
#define DDRC_LMR_DDR_ADDR_MASK  (0xfffff << DDRC_LMR_DDR_ADDR_BIT)

#define DDRC_LMR_MA_BIT     16 /* FOR LPDDR2, MA[9:0] */
#define DDRC_LMR_OP_BIT     24 /* FOR LPDDR2, OP[9:0] */

#define DDRC_LMR_BA_BIT     9 /* When performing a DDR command, BA[2:0]
                                 corresponding to external DDR address Pin BA[2:0]. */
#define DDRC_LMR_BA_MASK    (0x7 << DDRC_LMR_BA_BIT)
#define DDRC_LMR_CMD_BIT    6
#define DDRC_LMR_CMD_MASK   (0x7 << DDRC_LMR_CMD_BIT)
#define DDRC_LMR_CMD_PREC   (0 << DDRC_LMR_CMD_BIT)/* Precharge one bank/All banks */
#define DDRC_LMR_CMD_AUREF  (1 << DDRC_LMR_CMD_BIT)/* Auto-Refresh */
#define DDRC_LMR_CMD_LMR    (2 << DDRC_LMR_CMD_BIT)/* Load Mode Register */
#define DDRC_LMR_CMD_ZQCL_CS0   (3 << DDRC_LMR_CMD_BIT)/* ZQCL for DDR3 on CS0 */
#define DDRC_LMR_CMD_ZQCL_CS1   (4 << DDRC_LMR_CMD_BIT)/* ZQCL for DDR3 on CS1 */
#define DDRC_LMR_CMD_ZQCS_CS0   (5 << DDRC_LMR_CMD_BIT)/* ZQCS for DDR3 on CS0 */
#define DDRC_LMR_CMD_ZQCS_CS1   (6 << DDRC_LMR_CMD_BIT)/* ZQCS for DDR3 on CS1 */

#define DDRC_LMR_TMRD_BIT   1
#define DDRC_LMR_TMRD_MASK  (0x1f << DDRC_LMR_TMRD_BIT)
#define DDRC_LMR_START      (1 << 0) /* 0 No command is performed
                                        1 On the posedge of START, perform a command
                                        defined by CMD field */

/* DDRC  Auto-Refresh Counter */
#define DDRC_REFCNT_REF_EN      (1 << 0) /* Enable Refresh Counter */
#define DDRC_REFCNT_CLK_DIV_BIT     1  /* Clock Divider for auto-refresh counter. */
#define DDRC_REFCNT_CLK_DIV_MASK    (0x7 << DDRC_REFCNT_CLKDIV_BIT)

#define DDRC_REFCNT_PREREF_CNT_BIT  4
#define DDRC_REFCNT_PREREF_CNT_MASK (0xf << DDRC_REFCNT_PREREF_CNT_BIT)
#define DDRC_REFCNT_PREREF_CNT(val) (val << DDRC_REFCNT_PREREF_CNT_BIT)
#define DDRC_REFCNT_PREREF_CNT_DEFAULT  DDRC_REFCNT_PREREF_CNT(8)

#define DDRC_REFCNT_CNT_BIT     8  /* 8-bit counter */
#define DDRC_REFCNT_CNT_MASK        (0xff << DDRC_REFCNT_CNT_BIT)

#define DDRC_REFCNT_CON_BIT     16 /* Constant value used to compare with CNT value. */
#define DDRC_REFCNT_CON_MASK        (0xff << DDRC_REFCNT_CON_BIT)

#define DDRC_REFCNT_TRFC_BIT        24
#define DDRC_REFCNT_TRFC_MASK       (0x3f << DDRC_REFCNT_TRFC_BIT)

#define DDRC_REFCNT_PREREF_EN_BIT   30
#define DDRC_REFCNT_PREREF_EN       (1 << DDRC_REFCNT_PREREF_EN_BIT)

#define DDRC_REFCNT_PBREF_EN_BIT    31
#define DDRC_REFCNT_PBREF_EN        (1 << DDRC_REFCNT_PBREF_EN_BIT)

/* DDRC Memory Map Config Register */
#define DDRC_MMAP_BASE_BIT      8 /* base address */
#define DDRC_MMAP_BASE_MASK     (0xff << DDRC_MMAP_BASE_BIT)
#define DDRC_MMAP_MASK_BIT      0 /* address mask */
#define DDRC_MMAP_MASK_MASK     (0xff << DDRC_MMAP_MASK_BIT)

#define DDRC_MMAP0_BASE         (0x20 << DDRC_MMAP_BASE_BIT)
#define DDRC_MMAP1_BASE_64M     (0x24 << DDRC_MMAP_BASE_BIT) /*when bank0 is 128M*/
#define DDRC_MMAP1_BASE_128M        (0x28 << DDRC_MMAP_BASE_BIT) /*when bank0 is 128M*/
#define DDRC_MMAP1_BASE_256M        (0x30 << DDRC_MMAP_BASE_BIT) /*when bank0 is 128M*/

#define DDRC_MMAP_MASK_64_64        (0xfc << DDRC_MMAP_MASK_BIT)  /*mask for two 128M SDRAM*/
#define DDRC_MMAP_MASK_128_128      (0xf8 << DDRC_MMAP_MASK_BIT)  /*mask for two 128M SDRAM*/
#define DDRC_MMAP_MASK_256_256      (0xf0 << DDRC_MMAP_MASK_BIT)  /*mask for two 128M SDRAM*/

/* DDR device data width configure register */
#define DDRC_DWCFG_DFI_INIT_START   (1 << 3)

/* DDR device status register */
#define DDRC_DWSTATUS_DFI_INIT_COMP (1 << 0)

/* DDRC AHB Bus Register Protection Register */
#define DDRC_HREGPRO_HPRO_EN        (1 << 0)

/* DDRC APB Bus Register Protection Register */
#define DDRC_PREGPRO_PPRO_EN        (1 << 0)

/* DDRC clock gate unit configure 0 */
#define DDRC_CGU_PORT7      (1 << 28)
#define DDRC_CGU_PORT6      (1 << 24)
#define DDRC_CGU_PORT5      (1 << 20)
#define DDRC_CGU_PORT4      (1 << 16)
#define DDRC_CGU_PORT3      (1 << 12)
#define DDRC_CGU_PORT2      (1 << 8)
#define DDRC_CGU_PORT1      (1 << 4)
#define DDRC_CGU_PORT0      (1 << 0)

/* DDRC clock gate unit configure 1 */
#define DDRC_CGU_BWM        (1 << 8)
#define DDRC_CGU_PCTRL      (1 << 4)
#define DDRC_CGU_SCH        (1 << 1)
#define DDRC_CGU_PA     (1 << 0)

/* DDRC channel configure */
#define DDRC_CCHC_WTR_TIMEOUT_BIT   (28)
#define DDRC_CCHC_WTR_TIMEOUT_MSK   (0XF << DDRC_CCHC_WTR_TIMEOUT_BIT)
#define DDRC_CCHC_RTR_TIMEOUT_BIT   (24)
#define DDRC_CCHC_RTR_TIMEOUT_MSK   (0XF << DDRC_CCHC_RTR_TIMEOUT_BIT)
#define DDRC_CCHC_BW_LIMIT_WCNT_BIT (16)
#define DDRC_CCHC_BW_LIMIT_WCNT_MSK (0xff << DDRC_CCHC_BW_LIMIT_WCNT_BIT)
#define DDRC_CCHC_BW_LIMIT_RCNT_BIT (8)
#define DDRC_CCHC_BW_LIMIT_RCNT_MSK (0xff << DDRC_CCHC_BW_LIMIT_RCNT_BIT)
#define DDRC_CCHC_BW_LIMIT_WEN_BIT  (7)
#define DDRC_CCHC_BW_LIMIT_WEN      (1 << DDRC_CCHC_BW_LIMIT_WEN_BIT)
#define DDRC_CCHC_BW_LIMIT_REN_BIT  (6)
#define DDRC_CCHC_BW_LIMIT_REN      (1 << DDRC_CCHC_BW_LIMIT_REN_BIT)
#define DDRC_CCHC_TR_QOS_COMP_BIT   (4)
#define DDRC_CCHC_TR_QOS_COMP_MSK   (0x3 << DDRC_CCHC_TR_QOS_COMP_BIT)
#define DDRC_CCHC_TR_FIX_PRI_BIT    (2)
#define DDRC_CCHC_TR_FIX_PRI_MSK    (0x3 << DDRC_CCHC_TR_FIX_PRI_BIT)
#define DDRC_CCHC_TR_FIX_PRI_EN_BIT (1)
#define DDRC_CCHC_TR_FIX_PRI_EN     (1 << DDRC_CCHC_TR_FIX_PRI_EN_BIT)
#define DDRC_CCHC_PORT_PRI_BIT      (0)
#define DDRC_CCHC_PORT_PRI      (1 << DDRC_CCHC_PORT_PRI_BIT)

/*
 * DDR INNOPHY REGISTER BITS DEFINE
 * */

/* DDRP DQ Width Register */
#define DDRP_DQ_WIDTH_DQ_H      (1 << 1)
#define DDRP_DQ_WIDTH_DQ_L      (1 << 0)

/* DDRP Pll Ctrl Register */
#define DDRP_PLL_CTRL_PLLPDEN       (1 << 1)

/* DDRP Training Ctrl Register */
#define DDRP_TRAINING_CTRL_WL_BP    (1 << 3)
#define DDRP_TRAINING_CTRL_WL_START (1 << 2)
#define DDRP_TRAINING_CTRL_DSCSE_BP (1 << 1)
#define DDRP_TRAINING_CTRL_DSACE_START  (1 << 0)

/* DDRP Training Done Register */
#define DDRP_CALIB_DONE_HDQCFB      (1 << 3)
#define DDRP_CALIB_DONE_LDQCFB      (1 << 2)
#define DDRP_CALIB_DONE_HDQCFA      (1 << 1)
#define DDRP_CALIB_DONE_LDQCFA      (1 << 0)
#define DDRP_CALIB_DONE_HWRLFB      (1 << 3)
#define DDRP_CALIB_DONE_LWRLFB      (1 << 2)
#define DDRP_CALIB_DONE_HWRLFA      (1 << 1)
#define DDRP_CALIB_DONE_LWRLFA      (1 << 0)

/* DDRP CALIB BP Register */
#define DDRP_CALIB_BP_CYCLESELBH_BIT    4
#define DDRP_CALIB_BP_OPHCSELBH_BIT 3
#define DDRP_CALIB_BP_DLLSELBH_BIT  0

/* DDRP Init Complete Register */
#define DDRP_INIT_COMP          (1 << 0)

#define DDRP_PLL_LOCK           (1 << 3)

#define REG32(addr) *(volatile unsigned int *)(addr)

#define ddr_writel(value, reg)  (REG32(DDRC_BASE + reg) = (value))
#define ddr_readl(reg)      REG32(DDRC_BASE + reg)
#endif /* __DDR_H__ */
