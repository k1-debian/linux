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

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/delay.h>
#include "mipi_csi.h"

void dump_csi_reg(void)
{
	printk("****>>>>> dump csi reg <<<<<******\n");
	printk("**********VERSION =%08x\n", csi_core_read(VERSION));
	printk("**********N_LANES =%08x\n", csi_core_read(N_LANES));
	printk("**********PHY_SHUTDOWNZ = %08x\n", csi_core_read(PHY_SHUTDOWNZ));
	printk("**********DPHY_RSTZ = %08x\n", csi_core_read(DPHY_RSTZ));
	printk("**********CSI2_RESETN =%08x\n", csi_core_read(CSI2_RESETN));
	printk("**********PHY_STATE = %08x\n", csi_core_read(PHY_STATE));
	printk("**********DATA_IDS_1 = %08x\n", csi_core_read(DATA_IDS_1));
	printk("**********DATA_IDS_2 = %08x\n", csi_core_read(DATA_IDS_2));
	printk("**********ERR1 = %08x\n", csi_core_read(ERR1));
	printk("**********ERR2 = %08x\n", csi_core_read(ERR2));
	printk("**********MASK1 =%08x\n", csi_core_read(MASK1));
	printk("**********MASK2 =%08x\n", csi_core_read(MASK2));
	printk("**********PHY_TST_CTRL0 = %08x\n", csi_core_read(PHY_TST_CTRL0));
	printk("**********PHY_TST_CTRL1 = %08x\n", csi_core_read(PHY_TST_CTRL1));
}

void check_csi_error(void)
{

	unsigned int temp1, temp2;
	while (1) {
		dump_csi_reg();
		temp1 = csi_core_read(ERR1);
		temp2 = csi_core_read(ERR2);
		if (temp1 != 0) {
			printk("error-------- 1:0x%08x\n", temp1);
		}
		if (temp2 != 0) {
			printk("error-------- 2:0x%08x\n", temp2);
		}
	}
}

static unsigned char csi_core_write_part(unsigned int address, unsigned int data, unsigned char shift, unsigned char width)
{
	unsigned int mask = (1 << width) - 1;
	unsigned int temp = csi_core_read(address);
	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;
	csi_core_write(address, temp);

	return 0;
}

static unsigned char  csi_event_disable(unsigned int  mask, unsigned char err_reg_no)
{
	switch (err_reg_no) {
	case 1:
		csi_core_write(MASK1, mask | csi_core_read(MASK1));
		break;
	case 2:
		csi_core_write(MASK2, mask | csi_core_read(MASK2));
		break;
	default:
		return ERR_OUT_OF_BOUND;
	}

	return 0;
}

unsigned char csi_set_on_lanes(unsigned char lanes)
{
	csi_core_write_part(N_LANES, (lanes - 1), 0, 2);
	return 0;
}

static int csi_phy_ready(void)
{
	int ready;

	// TODO: phy0: lane0 is ready. need to be update for other lane
	ready = csi_core_read(PHY_STATE);

	if ((ready & (1 << 10)) && (ready & (1 << 4))) {
		return 1;
	}

	return 0;
}

/* Reduce power consumption */
int csi_phy_set_bandgap(void)
{
	unsigned int reg;

	/*reset phy*/
	csi_core_write_part(PHY_SHUTDOWNZ, 0, 0, 1);
	csi_core_write_part(CSI2_RESETN, 0, 0, 1);
	csi_core_write_part(DPHY_RSTZ, 0, 0, 1);

	csi_core_write_part(CSI2_RESETN, 1, 0, 1);
	csi_core_write_part(DPHY_RSTZ, 1, 0, 1);
	csi_core_write_part(PHY_SHUTDOWNZ, 1, 0, 1);

	/* set bandgap (reg0b[7])*/
	reg = csi_phy_read(RXPHY_REG_0_0b);
	reg |= (0x1 << 7);
	csi_phy_write(RXPHY_REG_0_0b, reg);
	/* printk("debug ---- > reg0b[7]:0x%08x, %d\n", csi_phy_read(RXPHY_REG_0_0b), __LINE__); */
	return 0;
}

static unsigned int count_time_cal(unsigned int clk, int version)
{
	unsigned int count_time = 0;

	switch (version) {
	case 2: /*X1600*/
		if (clk >= 80 && clk < 110) {
			count_time = 0x2;
		} else if (clk >= 110 && clk < 150) {
			count_time = 0x3;
		} else if (clk >= 150 && clk < 300) {
			count_time = 0x6;
		} else if (clk >= 300 && clk < 400) {
			count_time = 0x8;
		} else if (clk >= 400 && clk < 500) {
			count_time = 0xb;
		} else if (clk >= 500 && clk < 600) {
			count_time = 0xe;
		} else if (clk >= 600 && clk < 700) {
			count_time = 0x10;
		} else if (clk >= 700 && clk < 800) {
			count_time = 0x12;
		} else if (clk >= 800 && clk < 1000) {
			count_time = 0x16;
		} else if (clk >= 1000 && clk < 1200) {
			count_time = 0x1e;
		} else if (clk >= 1200 && clk < 1400) {
			count_time = 0x23;
		} else if (clk >= 1400 && clk < 1600) {
			count_time = 0x2d;
		} else if (clk >= 1600 && clk < 1800) {
			count_time = 0x32;
		} else if (clk >= 1800 && clk < 2000) {
			count_time = 0x37;
		} else if (clk >= 2000 && clk < 2200) {
			count_time = 0x3c;
		} else if (clk >= 2200 && clk < 2400) {
			count_time = 0x41;
		} else if (clk >= 2400 && clk < 2500) {
			count_time = 0x46;
		} else {
			count_time = 0;
		}
		break;
	default:
		count_time = 0;
		break;
	}

	return count_time;
}

int data_lane_settle_time_cal(unsigned int mipi_clk)
{
	struct clk *clk_apb = NULL;
	unsigned long rate = 0;
	int cfgclk_in = 0; /*TODO:float*/
	int ui = 0;
	int Ths_settle = 0;
	int Ths_settle_min = 0;
	int Ths_settle_max = 0;
	int i = 0;

	clk_apb = clk_get(NULL, "div_apb");
	rate = clk_get_rate(clk_apb);
	cfgclk_in = 1000000000 / rate + 1;
	clk_put(clk_apb);

	ui = 1000 / (mipi_clk * 2); /*ns*/
	Ths_settle_min = 85 + (6 * ui);
	Ths_settle_max = 145 + (10 * ui);

	for (i = 5; i <= 9; i++) {
		Ths_settle = 4 * cfgclk_in + (i * cfgclk_in) + 8 * ui;
		if ((Ths_settle > Ths_settle_min) && (Ths_settle < Ths_settle_max)) {
			return i;
		}
	}
	return 0;
}

int clk_lane_settle_time_cal(void)
{
	struct clk *clk_apb = NULL;
	unsigned long rate = 0;
	int cfgclk_in = 0; /*TODO:float*/
	int Tclk_settle = 0;
	int Tclk_settle_min = 95;
	int Tclk_settle_max = 300;
	int i = 0;

	clk_apb = clk_get(NULL, "div_apb");
	rate = clk_get_rate(clk_apb);
	cfgclk_in = 1000000000 / rate + 1;
	clk_put(clk_apb);

	for (i = 6; i <= 25; i++) {
		Tclk_settle = 4 * cfgclk_in + (i * cfgclk_in);
		if (Tclk_settle > Tclk_settle_min && (Tclk_settle < Tclk_settle_max)) {
			return i;
		}
	}
	return 0;
}

int csi_phy_start(int version, unsigned int lans, unsigned int mipi_clk)
{
	int retries = 30;
	int i;
	unsigned int reg;
	unsigned int count_time;
	int precounter_data, precounter_clk;

	if (version == 1) {
		precounter_data = data_lane_settle_time_cal(mipi_clk);
		precounter_clk = clk_lane_settle_time_cal();
		if (precounter_data && precounter_clk) { /*x2000 m300*/
			csi_phy_write(PRECOUNTER_IN_DATA,
			              precounter_data | (precounter_data << 8) | (precounter_data << 16) | (precounter_data << 24));
			csi_phy_write(PRECOUNTER_IN_CLK,
			              precounter_clk | (precounter_clk << 8));
		} else {
			printk("[Warning] D-phy precounter using default value\n");
		}

		printk("*******%s***** lans = %d\n", __func__, lans);
		csi_set_on_lanes(lans);

		/* both csi0 csi1 */
		*(volatile unsigned int *) 0xb0074008 = 1;
		*(volatile unsigned int *) 0xb007400c = 1;
		*(volatile unsigned int *) 0xb0073008 = 1;
		*(volatile unsigned int *) 0xb007300c = 1;

		csi_core_write_part(CSI2_RESETN, 0, 0, 1);
		csi_core_write_part(CSI2_RESETN, 1, 0, 1);
	} else {
		/*reset phy*/
		csi_core_write_part(PHY_SHUTDOWNZ, 0, 0, 1);
		csi_core_write_part(CSI2_RESETN, 0, 0, 1);
		csi_core_write_part(DPHY_RSTZ, 0, 0, 1);

		printk("lans: %d\n", lans);
		csi_set_on_lanes(lans);

		csi_core_write_part(CSI2_RESETN, 1, 0, 1);
		csi_core_write_part(DPHY_RSTZ, 1, 0, 1);
		csi_core_write_part(PHY_SHUTDOWNZ, 1, 0, 1);

		udelay(1000);

		/*THS-SETTLE*/
		count_time = count_time_cal(mipi_clk, version);
		if (count_time) { /*x1600*/
			csi_phy_write(RXPHY_REG_2_18, count_time);
			csi_phy_write(RXPHY_REG_3_18, count_time);
			csi_phy_write(RXPHY_REG_4_18, count_time);
		} else {
			printk("[Warning], D-phy count_time using default value\n");
		}

		/*r/w phy register*/
		switch (lans) {
		case 1:
			csi_phy_write(RXPHY_REG_0_00, 0x75);
			break;
		case 2:
			csi_phy_write(RXPHY_REG_0_00, 0x7d);
			break;
		default:
			printk("Config lans error\n");
			break;
		}
		csi_phy_write(RXPHY_REG_2_0a, 0x3f);

		/* clear bandgap (reg0b[7])*/
		reg = csi_phy_read(RXPHY_REG_0_0b);
		reg &= ~(0x1 << 7);
		csi_phy_write(RXPHY_REG_0_0b, reg);
		/* printk("debug ---- > reg0b[7]:0x%08x\n", csi_phy_read(RXPHY_REG_0_0b)); */

	}

	/* MASK all interrupts */
	csi_event_disable(0xffffffff, 1);
	csi_event_disable(0xffffffff, 2);

	if (version == 2) {
		/* wait for phy ready */
		for (i = 0; i < retries; i++) {
			if (csi_phy_ready()) {
				break;
			}
			udelay(300);
		}

		if (i >= retries) {
			printk("CSI PHY is not ready\n");
			return -1;

		}
	}

	return 0;
}

int csi_phy_stop(int version)
{

	printk("csi_phy_stop being called \n");
	/*reset phy*/
	/* both csi0 csi1 */
	if (version == 1) {
		*(volatile unsigned int *) 0xb0074008 = 0;
		*(volatile unsigned int *) 0xb007400c = 0;
	} else {
		csi_phy_set_bandgap();
	}
	*(volatile unsigned int *) 0xb0073008 = 0;
	*(volatile unsigned int *) 0xb007300c = 0;
	//  csi_core_write_part(PHY_SHUTDOWNZ, 0, 0, 1);
	//  csi_core_write_part(DPHY_RSTZ, 0, 0, 1);
	csi_core_write_part(CSI2_RESETN, 0, 0, 1);

	return 0;
}
