/*
 * Ingenic SoC MIPI-DSI lowlevel driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include "../include/jz_dsim.h"
#include "jz_mipi_dsi_regs.h"
#include "jz_mipi_dsih_hal.h"

#if (defined(CONFIG_SOC_X2000) || defined(CONFIG_SOC_X2100) || defined(CONFIG_SOC_M300) || defined(CONFIG_SOC_X1600))

struct dphy_pll_range {
	unsigned int start_clk_sel;
	unsigned int output_freq0;  /*start freq in same resolution*/
	unsigned int output_freq1;  /*end freq in same resolution*/
	unsigned int resolution;
};

struct dphy_pll_range dphy_pll_table[] = {
	{0,   63750000,  93125000,   312500},
	{95,  93750000,  186250000,  625000},
	{244, 187500000, 372500000,  1250000},
	{393, 375000000, 745000000,  2500000},
	{542, 750000000, 2750000000UL, 5000000},
};

int init_dsi_phy(struct dsi_device *dsi)
{
	int i;
	struct dphy_pll_range *pll;
	unsigned int output_freq = dsi->real_mipiclk;
	unsigned int pll_clk_sel = 0xffffffff;
	for (i = 0; i < ARRAY_SIZE(dphy_pll_table); i++) {
		pll = &dphy_pll_table[i];
		if (output_freq >= pll->output_freq0 && output_freq <= pll->output_freq1) {
			pll_clk_sel = pll->start_clk_sel + (output_freq - pll->output_freq0) / pll->resolution;
			break;
		}
	}
	if (pll_clk_sel == 0xffffffff) {
		printk("can not find appropriate pll freq set for dsi phy! output_freq: %d\n", output_freq);
		return ERR_DSI_PHY_FREQ_OUT_OF_BOUND;
	}

	//  printk("before setting dsi phy: pll_clk_sel: %x\n", readl(dsi->phy_address + 0x64));
	writel(pll_clk_sel, (unsigned int *)(dsi->phy_address + 0x64)); /* pll_clk_sel */
	//  printk("after setting dsi phy: pll_clk_sel: %x, output_freq: %d\n", readl(dsi->phy_address + 0x64), output_freq);
	return 0;
}

#endif

#if defined(CONFIG_SOC_X2600)
	#define DSI_PHY_BASE 0x400
#elif defined(CONFIG_SOC_X2500)
	#define DSI_PHY_BASE 0x0
#endif

#if (defined(CONFIG_SOC_X2500) || defined(CONFIG_SOC_X2600))

#define DSI_PHY_ANA_REG00       (DSI_PHY_BASE + 0x0 * 4)
#define DSI_PHY_ANA_REG01       (DSI_PHY_BASE + 0x1 * 4)
#define DSI_PHY_ANA_REG02       (DSI_PHY_BASE + 0x2 * 4)
#define DSI_PHY_ANA_REG03       (DSI_PHY_BASE + 0x3 * 4)
#define DSI_PHY_ANA_REG04       (DSI_PHY_BASE + 0x4 * 4)
#define DSI_PHY_ANA_REG05       (DSI_PHY_BASE + 0x5 * 4)
#define DSI_PHY_ANA_REG06       (DSI_PHY_BASE + 0x6 * 4)
#define DSI_PHY_ANA_REG07       (DSI_PHY_BASE + 0x7 * 4)
#define DSI_PHY_ANA_REG08       (DSI_PHY_BASE + 0x8 * 4)
#define DSI_PHY_ANA_REG0B       (DSI_PHY_BASE + 0xB * 4)
#define DSI_PHY_ANA_REG11       (DSI_PHY_BASE + 0x11 * 4)
#define DSI_PHY_ANA_REG1E       (DSI_PHY_BASE + 0x1E * 4)

#define DSI_PHY_DIG_REG00       (DSI_PHY_BASE + 0x80 + 0x0 * 4)
#define DSI_PHY_DIG_REG01       (DSI_PHY_BASE + 0x80 + 0x1 * 4)

#define DSI_PHY_LVDS_REG00      (DSI_PHY_BASE + 0x380 + 0x0 * 4)
#define DSI_PHY_LVDS_REG01      (DSI_PHY_BASE + 0x380 + 0x1 * 4)
#define DSI_PHY_LVDS_REG03      (DSI_PHY_BASE + 0x380 + 0x3 * 4)
#define DSI_PHY_LVDS_REG04      (DSI_PHY_BASE + 0x380 + 0x4 * 4)
#define DSI_PHY_LVDS_REG0B      (DSI_PHY_BASE + 0x380 + 0xB * 4)
#define DSI_PHY_LVDS_REG0C      (DSI_PHY_BASE + 0x380 + 0xC * 4)
#define DSI_PHY_LVDS_REG0D      (DSI_PHY_BASE + 0x380 + 0xD * 4)

#define DSI_PHY_HS_TLPX(lane)       (DSI_PHY_BASE + 0x114 + lane * 0x80)
#define DSI_PHY_HS_THS_PREPARE(lane)    (DSI_PHY_BASE + 0x118 + lane * 0x80)
#define DSI_PHY_THS_ZERO(lane)      (DSI_PHY_BASE + 0x11C + lane * 0x80)
#define DSI_PHY_HS_THS_TRAIL(lane)  (DSI_PHY_BASE + 0x120 + lane * 0x80)

struct lvds_pll_param {
	unsigned int pll_out_freq;  //KHz
	unsigned short postdiv;
	unsigned short prediv;
	unsigned int fbdiv;
};

struct dphy_timming_param {
	unsigned int freq_low;
	unsigned int freq_up;
	unsigned int val;
};

enum lane_index {
	CLK_LANE,
	DATA_LANE0,
	DATA_LANE1,
	DATA_LANE2,
	DATA_LANE3,
};

struct dphy_timming_param hs_tlpx_param_tab[] = {
	{80, 110, 0x02},
	{110, 300, 0x02},
	{300, 600, 0x03},
	{600, 1000, 0x05},
	{1000, 1200, 0x06},
	{1200, 1400, 0x09},
	{1400, 1600, 0x0d},
	{1600, 1800, 0x0e},
	{1800, 2000, 0x11},
	{2000, 2400, 0x13},
	{2400, 2500, 0x15},
};

struct dphy_timming_param ths_prepare_param_tab[] = {
	{80, 300, 0x7f},
	{300, 400, 0x7e},
	{400, 500, 0x7c},
	{500, 600, 0x70},
	{600, 700, 0x40},
	{700, 800, 0x02},
	{800, 1000, 0x08},
	{1000, 1400, 0x03},
	{1400, 1600, 0x42},
	{1600, 1800, 0x47},
	{1800, 2000, 0x64},
	{2000, 2200, 0x64},
	{2200, 2400, 0x33},
	{2400, 2500, 0x54},
};

struct dphy_timming_param clklane_ths_zero_param_tab[] = {
	{80, 110, 0x16},
	{110, 150, 0x16},
	{150, 200, 0x17},
	{200, 250, 0x17},
	{250, 300, 0x18},
	{300, 400, 0x19},
	{400, 500, 0x1B},
	{500, 600, 0x1D},
	{600, 700, 0x1E},
	{700, 800, 0x1F},
	{800, 1000, 0x20},
	{1000, 1200, 0x32},
	{1200, 1400, 0x32},
	{1400, 1600, 0x36},
	{1600, 1800, 0x7a},
	{1800, 2000, 0x7a},
	{2000, 2200, 0x7e},
	{2200, 2400, 0x7f},
	{2400, 2500, 0x7f},
};

struct dphy_timming_param datalane_ths_zero_param_tab[] = {
	{80, 110, 0x2},
	{110, 150, 0x3},
	{150, 200, 0x4},
	{200, 250, 0x5},
	{250, 300, 0x6},
	{300, 400, 0x7},
	{400, 500, 0x7},
	{500, 600, 0x8},
	{600, 700, 0x8},
	{700, 800, 0x9},
	{800, 1000, 0x9},
	{1000, 1200, 0x14},
	{1200, 1400, 0x14},
	{1400, 1600, 0x0E},
	{1600, 1800, 0x0E},
	{1800, 2000, 0x0E},
	{2000, 2200, 0x15},
	{2200, 2400, 0x15},
	{2400, 2500, 0x15},
};

struct dphy_timming_param ths_trail_param_tab[] = {
	{80, 110, 0x02},
	{110, 150, 0x02},
	{150, 200, 0x02},
	{200, 250, 0x04},
	{250, 300, 0x04},
	{300, 400, 0x04},
	{400, 500, 0x08},
	{500, 600, 0x10},
	{600, 700, 0x30},
	{700, 800, 0x30},
	{800, 1000, 0x30},
	{1000, 1200, 0x0f},
	{1200, 1400, 0x0f},
	{1400, 1600, 0x0f},
	{1600, 1800, 0x0f},
	{1800, 2000, 0x0b},
	{2000, 2200, 0x0b},
	{2200, 2400, 0x6a},
	{2400, 2500, 0x6a},

};

static unsigned short calc_inno_dphy_fbdiv(struct dsi_device *dsi)
{
	unsigned int real_mipi_clk = 0;
	real_mipi_clk = dsi->real_mipiclk / 1000000;    //  hz --- Mhz
	unsigned short fbdiv = 0;
	unsigned int prediv = 1;    //Fix value
	fbdiv = (real_mipi_clk * 2) / (12 / prediv);

	//printk("%s   real_mipi_clk = %d >>>>>>>>>>>>>>>>>>>>>>>>>>> \n",__func__,real_mipi_clk);
	if (fbdiv > 0x1ff) {
		fbdiv = 0x1ff;
	}
	return fbdiv;
}


static int set_mipi_hs_tlpx(struct dsi_device *dsi, unsigned int lane_idx)
{
	unsigned int clk_freq = dsi->real_mipiclk / 1000000;
	unsigned int reg = 0;
	unsigned int val = 0;
	int i = 0;
	unsigned int param_num = sizeof(hs_tlpx_param_tab) / sizeof(struct dphy_timming_param);
	reg = DSI_PHY_HS_TLPX(lane_idx);

	for (i = 0; i < param_num; i++) {
		if (clk_freq >= hs_tlpx_param_tab[i].freq_low && clk_freq < hs_tlpx_param_tab[i].freq_up) {
			break;
		}
	}
	if (i >= param_num) {
		printk("%s %d : can not find avalid param\r\n", __func__, __LINE__);
		return -1;
	}

	val = hs_tlpx_param_tab[i].val;
	writel(val, (unsigned int *)(dsi->phy_address + reg));

	return 0;
}

static int set_mipi_hs_ths_prepare(struct dsi_device *dsi, unsigned int lane_idx)
{
	unsigned int clk_freq = dsi->real_mipiclk / 1000000;
	unsigned int reg = 0;
	unsigned int val = 0;
	int i = 0;
	unsigned int param_num = sizeof(ths_prepare_param_tab) / sizeof(struct dphy_timming_param);
	reg = DSI_PHY_HS_THS_PREPARE(lane_idx);

	for (i = 0; i < param_num; i++) {
		if (clk_freq >= ths_prepare_param_tab[i].freq_low && clk_freq < ths_prepare_param_tab[i].freq_up) {
			break;
		}
	}
	if (i >= param_num) {
		printk("%s %d : can not find avalid param\r\n", __func__, __LINE__);
		return -1;
	}

	val = ths_prepare_param_tab[i].val;
	writel(val, (unsigned int *)(dsi->phy_address + reg));

	return 0;
}

static int set_mipi_ths_zero(struct dsi_device *dsi, unsigned int lane_idx)
{
	unsigned int clk_freq = dsi->real_mipiclk / 1000000;
	unsigned int reg = 0;
	unsigned int reg_hi = 0;
	unsigned int val = 0;
	unsigned int tmp = 0;
	int i = 0;
	unsigned int param_num = 0;
	struct dphy_timming_param *param = NULL;
	if (lane_idx == CLK_LANE) {
		param_num = sizeof(clklane_ths_zero_param_tab) / sizeof(struct dphy_timming_param);
		param = clklane_ths_zero_param_tab;
	} else {
		param_num = sizeof(datalane_ths_zero_param_tab) / sizeof(struct dphy_timming_param);
		param = datalane_ths_zero_param_tab;
	}
	reg = DSI_PHY_THS_ZERO(lane_idx);
	reg_hi = DSI_PHY_HS_THS_PREPARE(lane_idx);

	for (i = 0; i < param_num; i++) {
		if (clk_freq >= param[i].freq_low && clk_freq < param[i].freq_up) {
			break;
		}
	}
	if (i >= param_num) {
		printk("%s %d : can not find avalid param\r\n", __func__, __LINE__);
		return -1;
	}

	val = param[i].val;
	tmp = readl((unsigned int *)(dsi->phy_address + reg_hi));
	tmp &= ~(1 << 7);
	if (val & (1 << 6)) {
		tmp |= (1 << 7);
	}
	writel(tmp, (unsigned int *)(dsi->phy_address + reg_hi));

	writel(val & 0x3F, (unsigned int *)(dsi->phy_address + reg));

	return 0;
}

static int set_mipi_hs_ths_trail(struct dsi_device *dsi, unsigned int lane_idx)
{
	unsigned int clk_freq = dsi->real_mipiclk / 1000000;
	unsigned int reg = 0;
	unsigned int val = 0;
	int i = 0;
	unsigned int param_num = sizeof(ths_trail_param_tab) / sizeof(struct dphy_timming_param);
	reg = DSI_PHY_HS_THS_TRAIL(lane_idx);

	for (i = 0; i < param_num; i++) {
		if (clk_freq >= ths_trail_param_tab[i].freq_low && clk_freq < ths_trail_param_tab[i].freq_up) {
			break;
		}
	}
	if (i >= param_num) {
		printk("%s %d : can not find avalid param\r\n", __func__, __LINE__);
		return -1;
	}

	val = ths_trail_param_tab[i].val;
	writel(val, (unsigned int *)(dsi->phy_address + reg));

	return 0;
}

#endif

#if (defined(CONFIG_SOC_X2600))

struct lvds_pll_param lvds_pll_param_tab[] = {
	{30000000, 1, 1, 70},
	{31000000, 3, 1, 217},
	{31500000, 2, 1, 147},
	{32000000, 3, 1, 224},
	{33000000, 1, 1, 77},
	{34000000, 3, 1, 238},
	{34500000, 2, 1, 161},
	{35000000, 3, 1, 245},
	{36000000, 1, 1, 84},
	{37000000, 3, 1, 259},
	{37500000, 2, 1, 175},
	{38000000, 3, 1, 266},
	{39000000, 1, 1, 91},
	{40500000, 2, 1, 189},
	{42000000, 1, 1, 98},
	{43500000, 2, 1, 203},
	{45000000, 1, 1, 105},
	{46500000, 2, 1, 217},
	{48000000, 1, 1, 112},
	{49500000, 2, 1, 231},
	{51000000, 1, 1, 119},
	{52500000, 2, 1, 245},
	{54000000, 1, 1, 126},
	{55500000, 2, 1, 259},
	{57000000, 1, 1, 133},
	{60000000, 1, 1, 140},
	{63000000, 1, 1, 147},
	{66000000, 1, 1, 154},
	{69000000, 1, 1, 161},
	{72000000, 1, 1, 168},
	{75000000, 1, 1, 175},
	{78000000, 1, 1, 182},
	{81000000, 1, 1, 189},
	{84000000, 1, 1, 196},
	{87000000, 1, 1, 203},
	{90000000, 1, 1, 210},
};

static int get_pll_param_index(unsigned int pclk)
{
	int ret = -1;
	int i = 0;
	pclk = pclk * 1000;
	for (i = 0; i < sizeof(lvds_pll_param_tab) / sizeof(struct lvds_pll_param); i++) {
		if (pclk == lvds_pll_param_tab[i].pll_out_freq) {
			ret = i;
			break;
		}
	}
	return ret;
}

void lvds_rx_enable(struct dsi_device *dsi)
{
	unsigned int temp = 0;
	if (dsi->phy_mode == DSIPHY_MIPI_MODE) {
		return;
	}
	if (dsi->state == 1) {
		return;
	}
	mdelay(5);      // wait
	temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG00));
	temp |= (1 << 3);       // enable lvds rx
	writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG00));
	dsi->state = 1;
}

int init_dsi_phy(struct dsi_device *dsi)
{
	unsigned int temp = 0;
	unsigned short fbdiv = 0;
	printk("dsi phy address = 0x%x\n", (unsigned int)dsi->phy_address);

	if (dsi->phy_mode == DSIPHY_MIPI_MODE) {
		fbdiv = calc_inno_dphy_fbdiv(dsi);

		//S1 set prediv
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG03));
		temp &= ~0xff;
		if (fbdiv > 0xff) {
			temp |= (1 << 5);    //fbdiv bit8
		}
		temp |= 0x01;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG03));
		printk("DSI_PHY_ANA_REG03 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG03));

		//S2 set fbdiv
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG04));
		temp &= ~0xff;
		temp = fbdiv & 0xff;    //fbdiv bit 0-7
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG04));
		printk("DSI_PHY_ANA_REG04 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG04));

		//S3 PLL LDO
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG01));
		temp &= ~0xff;
		temp |= 0xE4;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG01));
		printk("DSI_PHY_ANA_REG01 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG01));

		//S4 set lane num
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG00));
		temp &= ~0xff;
		if (dsi->video_config->no_of_lanes == 4) {
			temp |= 0x7d;
			writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG00));
		} else {
			temp |= 0x4d;
			writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG00));
		}
		printk("DSI_PHY_ANA_REG00 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG00));

		//S5 reset analog
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG01));
		temp &= ~0xff;
		temp |= 0xe0;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG01));
		printk("DSI_PHY_ANA_REG01 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG01));

		// S6
		msleep(20);

		//S7  reset digital
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_DIG_REG00));
		temp &= ~0xff;
		temp |= 0x1e;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_DIG_REG00));
		printk("DSI_PHY_DIG_REG00 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_DIG_REG00));
		msleep(5);

		// S8 digital normal
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_DIG_REG00));
		temp &= ~0xff;
		temp |= 0x1f;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_DIG_REG00));
		printk("DSI_PHY_DIG_REG00 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_DIG_REG00));

		// S9 Func Mode select
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG03));
		temp &= ~0xff;
		temp |= 0x01;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG03));
		printk("DSI_PHY_LVDS_REG03 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG03));

		// S9-1  LPDT LANE0 PPI SYNC may not need
		temp = readl((unsigned int *)(dsi->phy_address + 0x580 + 0x30));
		temp |= 0x4;
		writel(temp, (unsigned int *)(dsi->phy_address + 0x580 + 0x30));


		{
			int lane = 0;
			for (lane = 0; lane <= DATA_LANE3; lane++) {
				set_mipi_hs_tlpx(dsi, lane);
				set_mipi_hs_ths_prepare(dsi, lane);
				set_mipi_hs_ths_trail(dsi, lane);
				set_mipi_ths_zero(dsi, lane);

			}
		}


		if (dsi->video_config->clk_lane_pn_swap) {
			// CLK LANE P/N SWAP
			temp = readl((unsigned int *)(dsi->phy_address + 0x500));
			temp |= (1 << 4);
			writel(temp, (unsigned int *)(dsi->phy_address + 0x500));
			printk("DSI_REG 680 : 0x%x\n", *(unsigned int *)(dsi->phy_address + 0x500));
		}

		if (dsi->video_config->lane0_pn_swap) {
			// LANE0 P/N SWAP
			temp = readl((unsigned int *)(dsi->phy_address + 0x580));
			temp |= (1 << 4);
			writel(temp, (unsigned int *)(dsi->phy_address + 0x580));
			printk("DSI_REG 680 : 0x%x\n", *(unsigned int *)(dsi->phy_address + 0x580));
		}

		if (dsi->video_config->lane1_pn_swap) {
			// LANE1 P/N SWAP
			temp = readl((unsigned int *)(dsi->phy_address + 0x600));
			temp |= (1 << 4);
			writel(temp, (unsigned int *)(dsi->phy_address + 0x600));
			printk("DSI_REG 680 : 0x%x\n", *(unsigned int *)(dsi->phy_address + 0x600));
		}

		if (dsi->video_config->lane2_pn_swap) {
			// LANE2 P/N SWAP
			temp = readl((unsigned int *)(dsi->phy_address + 0x680));
			temp |= (1 << 4);
			writel(temp, (unsigned int *)(dsi->phy_address + 0x680));
			printk("DSI_REG 680 : 0x%x\n", *(unsigned int *)(dsi->phy_address + 0x680));
		}

		if (dsi->video_config->lane3_pn_swap) {
			// LANE3 P/N SWAP
			temp = readl((unsigned int *)(dsi->phy_address + 0x700));
			temp |= (1 << 4);
			writel(temp, (unsigned int *)(dsi->phy_address + 0x700));
			printk("DSI_REG 700 : 0x%x\n", *(unsigned int *)(dsi->phy_address + 0x700));
		}
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG05));
		temp &= ~0x7;
		temp |= 0x0;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG05));
		printk("DSI_PHY_ANA_REG05 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG05));

		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG06));
		temp &= ~0xFF;
		temp |= 0x0;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG06));
		printk("DSI_PHY_ANA_REG06 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG06));

		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG07));
		temp &= ~0x77;
		temp |= 0x0;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG07));
		printk("DSI_PHY_ANA_REG07 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG07));

		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG08));
		temp &= ~0xf;
		temp |= 0x7;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG08));
		printk("DSI_PHY_ANA_REG08 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG08));

		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG0B));
		temp &= ~0xf;
		temp |= 0xf;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG0B));
		printk("DSI_PHY_ANA_REG0B : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG0B));

		/* temp = readl(dsi->phy_address + DSI_PHY_ANA_REG06); */
		/* //temp &= ~0xff; */
		/* temp |= (1 << 7) | (1 << 3); */
		/* writel(temp,(unsigned int*)(dsi->phy_address + DSI_PHY_ANA_REG06)); */
		/* printk("DSI_PHY_ANA_REG06 : 0x%x\n",*(unsigned int*)(dsi->phy_address + DSI_PHY_ANA_REG06)); */

		/* printk("DSI_PHY_ANA_REG1E : 0x%x\n",*(unsigned int*)(dsi->phy_address + DSI_PHY_ANA_REG1E)); */
		/* printk("DSI_PHY_ANA_REG08 : 0x%x\n",*(unsigned int*)(dsi->phy_address + DSI_PHY_ANA_REG08)); */
		//S10
		msleep(10);
	} else {    //LVDS-MODE
		//S1 select lvds mode
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG03));
		temp &= ~0xff;
		temp |= 0x2;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG03));
		printk("DSI_PHY_LVDS_REG03 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG03));

		// S1.1 LVDS color-coding / data_fmt{JEIDA,VESA(SPWG),vsync-pol,hsync-pol,de-pol}
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG0C));
		temp &= ~0xff;
		if (dsi->video_config->color_coding == COLOR_CODE_24BIT) {
			temp |= (1 << 5);
		}

		if (dsi->phy_mode == DSIPHY_LVDS_JEIDA_MODE) {
			temp |= (1 << 4);
		}

		if (dsi->video_config->v_polarity == 0) {
			temp &= ~(1 << 2);
		} else {
			temp |= (1 << 2);
		}

		if (dsi->video_config->h_polarity == 0) {
			temp &= ~(1 << 1);
		} else {
			temp |= (1 << 1);
		}

		if (dsi->video_config->data_en_polarity == 0) {
			temp &= ~(1 << 0);
		} else {
			temp |= (1 << 0);
		}

		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG0C));
		printk("DSI_PHY_LVDS_REG0C : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG0C));

		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG0D));
		temp &= ~0xff;
		temp |= 0x0;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG0D));
		printk("DSI_PHY_LVDS_REG0D : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG0D));

		/* temp = readl(dsi->phy_address + DSI_PHY_ANA_REG08); */
		/* temp &= ~(1 << 5); */
		/* writel(temp,(unsigned int*)(dsi->phy_address + DSI_PHY_ANA_REG08)); */
		/* printk("DSI_PHY_ANA_REG08 : 0x%x\n",*(unsigned int*)(dsi->phy_address + DSI_PHY_ANA_REG08)); */

		int param_idx = get_pll_param_index(dsi->video_config->pixel_clock);
		if (param_idx < 0) {
			printk("LVDS freq adapte failed !!!!!\n");
			return -1;
		}
		unsigned char prediv = lvds_pll_param_tab[param_idx].prediv;
		unsigned char postdiv = lvds_pll_param_tab[param_idx].postdiv;
		unsigned short fbdiv = lvds_pll_param_tab[param_idx].fbdiv;

		//S2 set prediv
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG03));
		temp &= ~0xff;
		temp |= prediv;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG03));
		printk("DSI_PHY_ANA_REG03 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG03));

		//S2-1 set postdiv
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG1E));
		temp |= (postdiv << 0);
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG1E));
		printk("DSI_PHY_ANA_REG1E : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG1E));

		//S3 set fbdiv
		if (fbdiv > 0x1ff) {
			printk("ERROR fbdiv over than 0x1ff %s \n", __func__);
		}
		if (fbdiv & 0x100) {
			temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG03));
			temp |= (1 << 5);   //fbdiv[8]
			writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG03));
			printk("DSI_PHY_ANA_REG03 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG03));

		}
		temp = fbdiv & 0xff;   //
		/* printk("%s pixel_clock = %d ======\n",__func__,dsi->video_config->pixel_clock); */
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG04));
		printk("DSI_PHY_ANA_REG04 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG04));

		//S4 enable ldo
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG08));
		temp &= ~0xff;
		temp |= 0x6e;
		//temp |= 0x4e;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG08));
		printk("DSI_PHY_ANA_REG08 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG08));

		//S5
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG01));
		temp &= ~0xff;
		temp |= 0xE4;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG01));
		printk("DSI_PHY_ANA_REG01 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG01));

		//S6 enable lane
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG00));
		temp &= ~0xff;
		temp |= 0x7d;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG00));
		printk("DSI_PHY_ANA_REG00 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG00));

		//S7
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG01));
		temp &= ~0xff;
		temp |= 0xE0;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG01));
		printk("DSI_PHY_ANA_REG01 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG01));

		//S8 MSB/LSB
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG00));
		temp &= ~0xff;
		temp |= 0x05;       // 0x0D-->MSB  0x0C-->LSB
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG00));
		printk("DSI_PHY_LVDS_REG00 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG00));

		//S9 enable lvds digital logic
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG01));
		temp &= ~0xff;
		temp |= 0x92;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG01));
		printk("DSI_PHY_LVDS_REG01 : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG01));

		//S10 enable lvds analog logic
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG0B));
		temp &= ~0xff;
		temp |= 0xF8;
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG0B));
		printk("DSI_PHY_LVDS_REG0B : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_LVDS_REG0B));

		// PLL to LVDS
		temp = readl((unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG1E));
		temp &= ~(3 << 5);
		writel(temp, (unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG1E));
		printk("DSI_PHY_ANA_REG1E : 0x%x\n", *(unsigned int *)(dsi->phy_address + DSI_PHY_ANA_REG1E));
		dsi->state = 0;
		msleep(5);

	}
	return 0;
}

#endif

#if (defined(CONFIG_SOC_X2500))

int init_dsi_phy(struct dsi_device *dsi)
{
	unsigned int temp = 0xffffffff;
	unsigned short fbdiv = 0;
	fbdiv = calc_inno_dphy_fbdiv(dsi);
	pr_debug("dsi phy address = 0x%x\n", dsi->phy_address);
	//power on ,reset, set pin_enable_ck/0/1/* of lanes to be used to high level and others to low
	pr_debug("%s,%d  step0 do nothing now.\n", __func__, __LINE__);

	//step1
	temp = *(volatile unsigned int *)(dsi->phy_address + 0x0C);
	temp &= ~0xff;
	if (fbdiv > 0xff) {
		temp |= (1 << 5);    //fbdiv bit8
	}
	temp |= 0x01;   //prediv
	writel(temp, (volatile unsigned int *)(dsi->phy_address + 0x0C));
	if ((*(volatile unsigned int *)(dsi->phy_address + 0x0C) & 0xff) != 0x01) {
		printk("%s,%d reg write error. Step1\n", __func__, __LINE__);
	}
	pr_debug("reg:0x03, value:0x%x\n", *(volatile unsigned int *)(dsi->phy_address + 0x0C));
	//step2
	temp = *(volatile unsigned int *)(dsi->phy_address + 0x10);
	temp &= ~0xff;
	temp = fbdiv & 0xff;    //fbdiv bit 0-7
	writel(temp, (volatile unsigned int *)(dsi->phy_address + 0x10));
	pr_debug("reg:0x04, value:0x%x\n", *(volatile unsigned int *)(dsi->phy_address + 0x10));
	msleep(20);
	//step3
	temp = *(volatile unsigned int *)(dsi->phy_address + 0x04);
	temp &= ~0xff;
	temp |= 0xe4;       //PLL LDO
	writel(temp, (volatile unsigned int *)(dsi->phy_address + 0x04));
	if ((*(volatile unsigned int *)(dsi->phy_address + 0x04) & 0xff) != 0xe4) {
		printk("%s,%d reg write error. Step3\n", __func__, __LINE__);
	}
	pr_debug("reg:0x01, value:0x%x\n", *(volatile unsigned int *)(dsi->phy_address + 0x04));
	//step4
	if (dsi->video_config->no_of_lanes == 4) {
		temp = *(volatile unsigned int *)(dsi->phy_address + 0x00);
		temp &= ~0xff;
		temp |= 0x7d;     //4lane
		writel(temp, (volatile unsigned int *)(dsi->phy_address + 0x00));
		if ((*(volatile unsigned int *)(dsi->phy_address + 0x00) & 0xff) != 0x7d) {
			printk("%s,%d reg write error. Step4\n", __func__, __LINE__);
		}
		/* printk("%s>>>>>>>>>>>>>>>>>>>>>>>> config as 4 lane \n",__func__); */
	} else if (dsi->video_config->no_of_lanes == 2) {
		temp = *(volatile unsigned int *)(dsi->phy_address + 0x00);
		temp &= ~0xff;
		temp |= 0x4d;     //2lane
		writel(temp, (volatile unsigned int *)(dsi->phy_address + 0x00));
		if ((*(volatile unsigned int *)(dsi->phy_address + 0x00) & 0xff) != 0x4d) {
			printk("%s,%d reg write error. Step4\n", __func__, __LINE__);
		}
	} else if (dsi->video_config->no_of_lanes == 1) {
		temp = *(volatile unsigned int *)(dsi->phy_address + 0x00);
		temp &= ~0xff;
		temp |= 0x45;     //1lane
		writel(temp, (volatile unsigned int *)(dsi->phy_address + 0x00));
		if ((*(volatile unsigned int *)(dsi->phy_address + 0x00) & 0xff) != 0x45) {
			printk("%s,%d reg write error. Step4\n", __func__, __LINE__);
		}
	}
	pr_debug("reg:0x00, value:0x%x\n", *(volatile unsigned int *)(dsi->phy_address + 0x00));
	//step5
	temp = *(volatile unsigned int *)(dsi->phy_address + 0x04);
	temp &= ~0xff;
	temp |= 0xe0;
	writel(temp, (volatile unsigned int *)(dsi->phy_address + 0x04));
	if ((*(volatile unsigned int *)(dsi->phy_address + 0x04) & 0xff) != 0xe0) {
		printk("%s,%d reg write error. Step5\n", __func__, __LINE__);
	}
	pr_debug("reg:0x01, value:0x%x\n", *(volatile unsigned int *)(dsi->phy_address + 0x04));

	//step6
	msleep(20);   //at lease 20ms, shortening need validation

	//step7
	temp = *(volatile unsigned int *)(dsi->phy_address + 0x80);
	temp &= ~0xff;
	temp |= 0x1e;
	writel(temp, (volatile unsigned int *)(dsi->phy_address + 0x80));
	if ((*(volatile unsigned int *)(dsi->phy_address + 0x80) & 0xff) != 0x1e) {
		printk("%s,%d reg write error. Step7\n", __func__, __LINE__);
	}
	pr_debug("reg:0x20, value:0x%x\n", *(volatile unsigned int *)(dsi->phy_address + 0x80));
	msleep(5);
	//step8
	temp = *(volatile unsigned int *)(dsi->phy_address + 0x80);
	temp &= ~0xff;
	temp |= 0x1f;
	writel(temp, (volatile unsigned int *)(dsi->phy_address + 0x80));
	if ((*(volatile unsigned int *)(dsi->phy_address + 0x80) & 0xff) != 0x1f) {
		printk("%s,%d reg write error. Step8\n", __func__, __LINE__);
	}
	pr_debug("reg:0x20, value:0x%x\n", *(volatile unsigned int *)(dsi->phy_address + 0x80));
	//step9

	{
		int lane = 0;
		for (lane = 0; lane <= DATA_LANE3; lane++) {
			set_mipi_hs_tlpx(dsi, lane);
			set_mipi_hs_ths_prepare(dsi, lane);
			set_mipi_hs_ths_trail(dsi, lane);
			set_mipi_ths_zero(dsi, lane);

		}
	}

	msleep(10);

	pr_debug("%s,%d  dsi phy init over now...\n", __func__, __LINE__);

	return 0;
}

#endif
