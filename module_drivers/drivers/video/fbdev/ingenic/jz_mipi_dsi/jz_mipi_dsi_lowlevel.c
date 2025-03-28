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
#include "jz_mipi_dsi_phy.h"

void jz_dsih_dphy_reset(struct dsi_device *dsi, int reset)
{

	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_RSTZ, reset, 1, 1);
}

void jz_dsih_dphy_stop_wait_time(struct dsi_device *dsi,
                                 unsigned char no_of_byte_cycles)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_IF_CFG, no_of_byte_cycles, 8,
	                     8);
}

void jz_dsih_dphy_no_of_lanes(struct dsi_device *dsi, unsigned char no_of_lanes)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_IF_CFG, no_of_lanes - 1, 0, 2);
}

void jz_dsih_dphy_clock_en(struct dsi_device *dsi, int en)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_RSTZ, en, 2, 1);
}

void jz_dsih_dphy_shutdown(struct dsi_device *dsi, int powerup)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_RSTZ, powerup, 0, 1);
}

void jz_dsih_dphy_ulpm_enter(struct dsi_device *dsi)
{
	/* PHY_STATUS[6:1] == 6'h00 */
	if (mipi_dsih_read_part(dsi, R_DSI_HOST_PHY_STATUS, 1, 6) == 0x0) {
		printk("MIPI D-PHY is already in ULPM state now\n");
		return;
	}
	/* PHY_RSTZ[3:0] = 4'hF */
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_RSTZ, 0xF, 0, 4);
	/* PHY_ULPS_CTRL[3:0] = 4'h0 */
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_ULPS_CTRL, 0x0, 0, 4);
	/* PHY_TX_TRIGGERS[3:0] = 4'h0 */
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_TX_TRIGGERS, 0x0, 0, 4);
	/* PHY_STATUS[6:4] == 3'h3 */
	while (mipi_dsih_read_part(dsi, R_DSI_HOST_PHY_STATUS, 4, 3) != 0x3 ||
	       mipi_dsih_read_part(dsi, R_DSI_HOST_PHY_STATUS, 0, 2) != 0x1)
		;
	/* PHY_ULPS_CTRL [3:0] = 4'h5 */
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_ULPS_CTRL, 0x5, 0, 4);
	/* LPCLK_CTRL[1:0] = 2'h2 */
	mipi_dsih_write_part(dsi, R_DSI_HOST_LPCLK_CTRL, 0x2, 0, 2);
	/* PHY_STATUS[6:0] == 7'h1 */
	while (mipi_dsih_read_part(dsi, R_DSI_HOST_PHY_STATUS, 0, 7) != 0x1)
		;
	/* PHY_RSTZ[3] = 1'b0 */
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_RSTZ, 0x0, 3, 1);
	/* PHY_STATUS [0] == 1'b0 */
	while (mipi_dsih_read_part(dsi, R_DSI_HOST_PHY_STATUS, 0, 1) != 0x0)
		;
	printk("%s ...\n", __func__);
}

void jz_dsih_dphy_ulpm_exit(struct dsi_device *dsi)
{
	/* PHY_STATUS[6:1] == 6'h00 */
	if (mipi_dsih_read_part(dsi, R_DSI_HOST_PHY_STATUS, 1, 6) != 0x0) {
		printk("MIPI D-PHY is not in ULPM state now\n");
		return;
	}
	/* PHY_STATUS[0] == 1'b1 */
	if (mipi_dsih_read_part(dsi, R_DSI_HOST_PHY_STATUS, 0, 1) == 0x1) {
		goto step5;
	}

	/* PHY_RSTZ [3] = 1'b1 */
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_RSTZ, 0x1, 3, 1);
	/* PHY_STATUS[0] == 1'b1 */
	while (mipi_dsih_read_part(dsi, R_DSI_HOST_PHY_STATUS, 0, 1) != 0x1)
		;

step5:
	/* PHY_ULPS_CTRL[3:0] = 4'hF */
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_ULPS_CTRL, 0xF, 0, 4);
	/* PHY_STATUS [5] == 1'b1 && PHY_STATUS [3] == 1'b1 */
	while (mipi_dsih_read_part(dsi, R_DSI_HOST_PHY_STATUS, 5, 1) != 0x1 ||
	       mipi_dsih_read_part(dsi, R_DSI_HOST_PHY_STATUS, 3, 1) != 0x1)
		;
	/* Wait for 1 ms */
	mdelay(1);
	/* PHY_ULPS_CTRL [3:0] = 4'h0 */
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_ULPS_CTRL, 0x0, 0, 4);
	/* LPCLK_CTRL[1:0] = 2'h1 */
	mipi_dsih_write_part(dsi, R_DSI_HOST_LPCLK_CTRL, 0x1, 0, 2);
	/* PHY_STATUS [6:4] == 3'h3 && PHY_STATUS [1:0] == 2'h1 */
	while (mipi_dsih_read_part(dsi, R_DSI_HOST_PHY_STATUS, 4, 3) != 0x3 ||
	       mipi_dsih_read_part(dsi, R_DSI_HOST_PHY_STATUS, 0, 2) != 0x1)
		;
	/* PHY_RSTZ [3] = 1'b0 */
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_RSTZ, 0x0, 3, 1);
	printk("%s ...\n", __func__);
}

void jz_dsih_hal_power(struct dsi_device *dsi, int on)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_PWR_UP, on, 0, 1);
}

int jz_dsi_init_config(struct dsi_device *dsi)
{
	struct dsi_config *dsi_config;
	dsih_error_t err = OK;

	dsi_config = dsi->dsi_config;

	mipi_dsih_hal_dpi_color_mode_pol(dsi, !dsi_config->color_mode_polarity);
	mipi_dsih_hal_dpi_shut_down_pol(dsi, !dsi_config->shut_down_polarity);

	err = mipi_dsih_phy_hs2lp_config(dsi, dsi_config->max_hs_to_lp_cycles);
	err |= mipi_dsih_phy_lp2hs_config(dsi, dsi_config->max_lp_to_hs_cycles);
	err |= mipi_dsih_phy_bta_time(dsi, dsi_config->max_bta_cycles);
	if (err) {
		return err;
	}

	mipi_dsih_hal_dpi_lp_during_hfp(dsi, 1);
	mipi_dsih_hal_dpi_lp_during_hbp(dsi, 1);

	mipi_dsih_hal_dpi_lp_during_vactive(dsi, 1);
	mipi_dsih_hal_dpi_lp_during_vfp(dsi, 1);
	mipi_dsih_hal_dpi_lp_during_vbp(dsi, 1);
	mipi_dsih_hal_dpi_lp_during_vsync(dsi, 1);
	mipi_dsih_hal_dcs_wr_tx_type(dsi, 0, 1);
	mipi_dsih_hal_dcs_wr_tx_type(dsi, 1, 1);
	mipi_dsih_hal_dcs_wr_tx_type(dsi, 3, 1);    /* long packet */
	mipi_dsih_hal_dcs_rd_tx_type(dsi, 0, 1);

	mipi_dsih_hal_gen_wr_tx_type(dsi, 0, 1);
	mipi_dsih_hal_gen_wr_tx_type(dsi, 1, 1);
	mipi_dsih_hal_gen_wr_tx_type(dsi, 2, 1);
	mipi_dsih_hal_gen_wr_tx_type(dsi, 3, 1);    /* long packet */
	mipi_dsih_hal_gen_rd_tx_type(dsi, 0, 1);
	mipi_dsih_hal_gen_rd_tx_type(dsi, 1, 1);
	mipi_dsih_hal_gen_rd_tx_type(dsi, 2, 1);
	mipi_dsih_hal_gen_rd_vc(dsi, 0);
	mipi_dsih_hal_gen_eotp_rx_en(dsi, 0);
	mipi_dsih_hal_gen_eotp_tx_en(dsi, 0);
	mipi_dsih_hal_bta_en(dsi, 0);
	mipi_dsih_hal_gen_ecc_rx_en(dsi, 0);
	mipi_dsih_hal_gen_crc_rx_en(dsi, 0);
	return OK;
}

dsih_error_t jz_init_dsi(struct dsi_device *dsi)
{

	dsih_error_t err = OK;
	if (dsi->state == INITIALIZED) {
		return ERR_DSI_INVALID_INSTANCE;
	} else {
		jz_dsih_hal_power(dsi, 0);
		jz_dsih_hal_power(dsi, 1);
		err = jz_dsi_init_config(dsi);
		if (err) {
			return err;
		}
		jz_dsih_hal_power(dsi, 0);
		jz_dsih_hal_power(dsi, 1);

	}

	return err;
}

static irqreturn_t ingenicfb_irq_handler(int irq, void *data)
{
	unsigned int irq_flag;
	struct dsi_device *dsi = (struct dsi_device *)data;

	spin_lock(&dsi->irq_lock);

	irq_flag = mipi_dsih_read_word(dsi, R_DSI_HOST_INT_ST1);

	if (likely(irq_flag & (1 << 6))) {
		printk("\033[31m(l:%d, f:%s, F: %s) %d %s\033[0m\n", __LINE__, __func__, __FILE__, 0, "");
	}

	spin_unlock(&dsi->irq_lock);
	return IRQ_HANDLED;
}

void dsi_irq_test(struct dsi_device *dsi)
{
	mipi_dsih_write_word(dsi, R_DSI_HOST_INT_FORCE1, 1 << 6);
}

void jz_mipi_dsi_init_interrupt(struct dsi_device *dsi)
{
	/* fiexed */
	struct device *dev = &dsi->dsim_lcd_dev->dev;

	spin_lock_init(&dsi->irq_lock);

	mipi_dsih_write_word(dsi, R_DSI_HOST_INT_MSK0, 0x3fffff);
	mipi_dsih_write_word(dsi, R_DSI_HOST_INT_MSK1, 0xfff);

	if (devm_request_irq(dev, 18, ingenicfb_irq_handler, 0,
	                     "mipidsi", dsi)) {
		dev_err(dev, "request dsi irq failed\n");
	}
}

int jz_dsi_video_coding(struct dsi_device *dsi,
                        unsigned short *bytes_per_pixel_x100,
                        unsigned char *video_size_step,
                        unsigned short *video_size)
{
	struct video_config *video_config;
	dsih_error_t err_code = OK;
	video_config = dsi->video_config;

	switch (video_config->color_coding) {
	case COLOR_CODE_16BIT_CONFIG1:
	case COLOR_CODE_16BIT_CONFIG2:
	case COLOR_CODE_16BIT_CONFIG3:
		*bytes_per_pixel_x100 = 200;
		*video_size_step = 1;
		break;
	case COLOR_CODE_18BIT_CONFIG1:
	case COLOR_CODE_18BIT_CONFIG2:
		mipi_dsih_hal_dpi_18_loosely_packet_en(dsi,
		                                       video_config->
		                                       is_18_loosely);
		*bytes_per_pixel_x100 = 225;
		if (!video_config->is_18_loosely) { // 18bits per pixel and NOT loosely, packets should be multiples of 4
			*video_size_step = 4;
			//round up active H pixels to a multiple of 4
			for (; ((*video_size) % 4) != 0; (*video_size)++) {
				;
			}
		} else {
			*video_size_step = 1;
		}
		break;
	case COLOR_CODE_24BIT:
		*bytes_per_pixel_x100 = 300;    //burst mode
		*video_size_step = 1;   //no burst mode
		break;
	default:
		printk("invalid color coding\n");
		err_code = ERR_DSI_COLOR_CODING;
		break;
	}
	if (err_code == OK) {
		pr_debug("video_config->color_coding:%d\n",
		         video_config->color_coding);
		err_code =
		    mipi_dsih_hal_dpi_color_coding(dsi,
		                                   video_config->color_coding);
	}
	if (err_code != OK) {
		return err_code;
	}
	return 0;

}

void jz_dsi_dpi_cfg(struct dsi_device *dsi, unsigned int *ratio_clock_xPF,
                    unsigned short *bytes_per_pixel_x100)
{
	struct video_config *video_config;
	unsigned int hs_timeout = 0;
	int counter = 0;

	video_config = dsi->video_config;
	mipi_dsih_hal_dpi_video_mode_type(dsi, video_config->video_mode);

	/*HSA+HBP+HACT+HFP * 1 */
	mipi_dsih_hal_dpi_hline(dsi,
	                        (unsigned
	                         short)((video_config->h_total_pixels *
	                                 (*ratio_clock_xPF)) /
	                                PRECISION_FACTOR));
	mipi_dsih_hal_dpi_hbp(dsi,
	                      ((video_config->h_back_porch_pixels *
	                        (*ratio_clock_xPF)) / PRECISION_FACTOR));
	mipi_dsih_hal_dpi_hsa(dsi,
	                      ((video_config->h_sync_pixels *
	                        (*ratio_clock_xPF)) / PRECISION_FACTOR));
	mipi_dsih_hal_dpi_vactive(dsi, video_config->v_active_lines);
	mipi_dsih_hal_dpi_vfp(dsi,
	                      video_config->v_total_lines -
	                      (video_config->v_back_porch_lines +
	                       video_config->v_sync_lines +
	                       video_config->v_active_lines));
	mipi_dsih_hal_dpi_vbp(dsi, video_config->v_back_porch_lines);
	mipi_dsih_hal_dpi_vsync(dsi, video_config->v_sync_lines);

#ifdef CONFIG_DSI_DPI_DEBUG
	printk("hline:%d\n",
	       (unsigned
	        short)((video_config->h_total_pixels * (*ratio_clock_xPF)) /
	               PRECISION_FACTOR));
	printk("hbp:%d\n",
	       ((video_config->h_back_porch_pixels * (*ratio_clock_xPF)) /
	        PRECISION_FACTOR));
	printk("hsa:%d\n",
	       ((video_config->h_sync_pixels * (*ratio_clock_xPF)) /
	        PRECISION_FACTOR));
	printk("vactive:%d\n", video_config->v_active_lines);
	printk("vfp:%d\n",
	       video_config->v_total_lines - (video_config->v_back_porch_lines +
	                                      video_config->v_sync_lines +
	                                      video_config->v_active_lines));
	printk("vbp:%d\n", video_config->v_back_porch_lines);
	printk("vsync:%d\n", video_config->v_sync_lines);
#endif

	mipi_dsih_hal_dpi_hsync_pol(dsi, !video_config->h_polarity);    //active low
	mipi_dsih_hal_dpi_vsync_pol(dsi, !video_config->v_polarity);    //active low
	mipi_dsih_hal_dpi_dataen_pol(dsi, !video_config->data_en_polarity); // active high
	// HS timeout timing
	hs_timeout =
	    ((video_config->h_total_pixels * video_config->v_active_lines) +
	     (DSIH_PIXEL_TOLERANCE * (*bytes_per_pixel_x100)) / 100);
	for (counter = 0x80; (counter < hs_timeout) && (counter > 2); counter--) {
		if ((hs_timeout % counter) == 0) {
			mipi_dsih_hal_timeout_clock_division(dsi, counter);
			mipi_dsih_hal_lp_rx_timeout(dsi,
			                            (unsigned short)(hs_timeout
			                                    /
			                                    counter));
			mipi_dsih_hal_hs_tx_timeout(dsi,
			                            (unsigned short)(hs_timeout
			                                    /
			                                    counter));
			break;
		}
	}

}

void jz_dsih_hal_tx_escape_division(struct dsi_device *dsi,
                                    unsigned char tx_escape_division)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_CLKMGR_CFG, tx_escape_division, 0,
	                     8);
}

dsih_error_t jz_dsi_set_clock(struct dsi_device *dsi)
{
	init_dsi_phy(dsi);
	jz_dsih_dphy_stop_wait_time(dsi, 0x1C);
	jz_dsih_dphy_clock_en(dsi, 1);
	jz_dsih_dphy_shutdown(dsi, 1);
	jz_dsih_dphy_reset(dsi, 1);
	printk("configure master-phy is ok\n");
	//jz_dsih_hal_tx_escape_division(dsi, TX_ESC_CLK_DIV);
	jz_dsih_hal_tx_escape_division(dsi, dsi->tx_escape_div);
	return OK;
}
