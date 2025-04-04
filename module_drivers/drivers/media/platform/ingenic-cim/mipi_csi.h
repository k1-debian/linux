/*
 * Copyright (c) 2012 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * Core file for Ingenic Display Controller driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MIPI_CSI_H__
#define __MIPI_CSI_H__

/* csi host regs, base addr should be defined in board cfg */
#ifdef CONFIG_SOC_X1600
	#define DWC_CSI_CTRL_BASE       0xB0073000
#else
	#define DWC_CSI_CTRL_BASE       0xB0074000
#endif
#define INNO_CSI_PHY_BASE       0xB0076000

/* csi controller */
#define VERSION             0x00
#define N_LANES             0x04
#define PHY_SHUTDOWNZ           0x08
#define DPHY_RSTZ           0x0C
#define CSI2_RESETN         0x10
#define PHY_STATE           0x14
#define DATA_IDS_1          0x18
#define DATA_IDS_2          0x1C
#define ERR1                0x20
#define ERR2                0x24
#define MASK1               0x28
#define MASK2               0x2C
#define PHY_TST_CTRL0           0x30
#define PHY_TST_CTRL1           0x34
#define DEBUG               0x40

/* csi d-phy */
#define LANE_EN             0x000
#define CLK_CNT_TIME            0x100
#define L0_CNT_TIME         0x180
#define L1_CNT_TIME         0x200
#define L2_CNT_TIME         0x280
#define L3_CNT_TIME         0x300
#define CLK_CON_MODE            0x128
#define SWITCH_LVDS_BANK        0x080
#define MODEL_EN            0x2cc
#define LVDS_LOGICAL_EN         0x300

/* inno d-phy*/
#define RXPHY_REG_0_00          0x00
#define RXPHY_REG_0_0b          0x2c
#define RXPHY_REG_0_0d          0x34
#define RXPHY_REG_0_0e          0x38
#define RXPHY_REG_0_0f          0x3c
#define RXPHY_REG_0_12          0x48
#define RXPHY_REG_1_00          0x80
#define RXPHY_REG_2_0a          0x128
#define RXPHY_REG_2_10          0x140
#define RXPHY_REG_2_11          0x144
#define RXPHY_REG_2_12          0x148
#define RXPHY_REG_2_18          0x160 /*Clock lane count_time*/
#define RXPHY_REG_3_18          0x1e0 /*data lane0 count_time*/
#define RXPHY_REG_4_18          0x260 /*data lane1 count_time*/

/*M31 d-phy*/
#define PRECOUNTER_IN_CLK       0xb8
#define PRECOUNTER_IN_DATA      0xbc

typedef enum {
	ERR_NOT_INIT = 0xFE,
	ERR_ALREADY_INIT = 0xFD,
	ERR_NOT_COMPATIBLE = 0xFC,
	ERR_UNDEFINED = 0xFB,
	ERR_OUT_OF_BOUND = 0xFA,
	SUCCESS = 0
} csi_error_t;

#define dwc_csi_readl(reg)              \
	readl((unsigned int *)(DWC_CSI_CTRL_BASE + reg))
#define dwc_csi_writel(reg, value)          \
	writel((value), (unsigned int *)(DWC_CSI_CTRL_BASE + reg))

#define csi_core_write(addr, value) dwc_csi_writel(addr, value)
#define csi_core_read(addr) dwc_csi_readl(addr)

#define csi_phy_read(reg)              \
	readl((unsigned int *)(INNO_CSI_PHY_BASE + reg))
#define csi_phy_write(reg, value)          \
	writel((value), (unsigned int *)(INNO_CSI_PHY_BASE + reg))

/* function */
extern int csi_phy_init(void);
extern int csi_phy_start(int version, unsigned int lans, unsigned int mipi_clk);
extern int csi_phy_stop(int version);

extern void dump_csi_reg(void);
extern void check_csi_error(void);
extern unsigned char csi_set_on_lanes(unsigned char lanes);

#endif/*__MIPI_CSI_H__*/
