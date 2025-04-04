/*
 * Ingenic SoC MIPI-DSI dsim driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/memory.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/notifier.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/atomic.h>
#include "../include/ingenicfb.h"
#include "../include/lcd_panel.h"
#include "../include/jz_dsim.h"
#include "jz_mipi_dsi_lowlevel.h"
#include "jz_mipi_dsih_hal.h"
#include "jz_mipi_dsi_regs.h"
#include "jz_mipi_dsi_phy.h"
extern struct jzfb_platform_data jzfb_platform_data;
#define jzfb_pdata jzfb_platform_data

struct mipi_dsim_ddi {
	int             bus_id;
	struct list_head        list;
	struct mipi_dsim_lcd_device *dsim_lcd_dev;
	struct mipi_dsim_lcd_driver *dsim_lcd_drv;
};

static LIST_HEAD(dsim_ddi_list);

static DEFINE_MUTEX(mipi_dsim_lock);

void dump_dsi_reg(struct dsi_device *dsi);
static DEFINE_MUTEX(dsi_lock);

int jz_dsi_video_cfg(struct dsi_device *dsi)
{
	dsih_error_t err_code = OK;
	unsigned short bytes_per_pixel_x100 = 0;    /* bpp x 100 because it can be 2.25 */
	unsigned short video_size = 0;
	unsigned int ratio_clock_xPF = 0;   /* holds dpi clock/byte clock times precision factor */
	unsigned short null_packet_size = 0;
	unsigned char video_size_step = 1;
	unsigned int total_bytes = 0;
	unsigned int bytes_per_chunk = 0;
	unsigned int no_of_chunks = 0;
	unsigned int bytes_left = 0;
	unsigned int chunk_overhead = 0;

	struct video_config *video_config;
	video_config = dsi->video_config;

	/* check DSI controller dsi */
	if ((dsi == NULL) || (video_config == NULL)) {
		return ERR_DSI_INVALID_INSTANCE;
	}
#if 1
	if (dsi->state == UBOOT_INITIALIZED) {
		/*no need to reconfig, just return*/
		printk("dsi has already been initialized by uboot!!\n");
		return OK;
	}
#endif
	if (dsi->state != INITIALIZED) {
		return ERR_DSI_INVALID_INSTANCE;
	}

	ratio_clock_xPF =
	    (video_config->byte_clock * PRECISION_FACTOR) /
	    (video_config->pixel_clock);

	video_size = video_config->h_active_pixels;
	/*  disable set up ACKs and error reporting */
	mipi_dsih_hal_dpi_frame_ack_en(dsi, video_config->receive_ack_packets);
	if (video_config->receive_ack_packets) {    /* if ACK is requested, enable BTA, otherwise leave as is */
		mipi_dsih_hal_bta_en(dsi, 1);
	}
	mipi_dsih_hal_dpi_lp_cmd_en(dsi, 1);
	/*0:switch to high speed transfer, 1 low power mode */
	mipi_dsih_write_word(dsi, R_DSI_HOST_CMD_MODE_CFG, 0);
	/*0:enable video mode, 1:enable cmd mode */
	mipi_dsih_hal_gen_set_mode(dsi, 0);
	mipi_dsih_hal_error_mask_0(dsi, INT_MASK0_ALL);
	mipi_dsih_hal_error_mask_1(dsi, INT_MASK1_ALL);
	err_code =
	    jz_dsi_video_coding(dsi, &bytes_per_pixel_x100, &video_size_step,
	                        &video_size);
	if (err_code) {
		return err_code;
	}

	jz_dsi_dpi_cfg(dsi, &ratio_clock_xPF, &bytes_per_pixel_x100);

	/* TX_ESC_CLOCK_DIV must be less than 20000KHz */
	/* jz_dsih_hal_tx_escape_division(dsi, TX_ESC_CLK_DIV); */
	jz_dsih_hal_tx_escape_division(dsi, dsi->tx_escape_div);

	/* video packetisation   */
	if (video_config->video_mode == VIDEO_BURST_WITH_SYNC_PULSES) { /* BURST */
		//mipi_dsih_hal_dpi_null_packet_en(dsi, 0);
		mipi_dsih_hal_dpi_null_packet_size(dsi, 0);
		//mipi_dsih_hal_dpi_multi_packet_en(dsi, 0);
		err_code =
		    err_code ? err_code : mipi_dsih_hal_dpi_chunks_no(dsi, 1);
		err_code =
		    err_code ? err_code :
		    mipi_dsih_hal_dpi_video_packet_size(dsi, video_size);
		if (err_code != OK) {
			return err_code;
		}
		/* BURST by default, returns to LP during ALL empty periods - energy saving */
		mipi_dsih_hal_dpi_lp_during_hfp(dsi, 1);
		mipi_dsih_hal_dpi_lp_during_hbp(dsi, 1);
		mipi_dsih_hal_dpi_lp_during_vactive(dsi, 1);
		mipi_dsih_hal_dpi_lp_during_vfp(dsi, 1);
		mipi_dsih_hal_dpi_lp_during_vbp(dsi, 1);
		mipi_dsih_hal_dpi_lp_during_vsync(dsi, 1);
#ifdef CONFIG_DSI_DPI_DEBUG
		/*      D E B U G               */
		{
			pr_info("burst video");
			pr_info("h line time %d ,",
			        (unsigned
			         short)((video_config->h_total_pixels *
			                 ratio_clock_xPF) / PRECISION_FACTOR));
			pr_info("video_size %d ,", video_size);
		}
#endif
	} else {        /* non burst transmission */
		null_packet_size = 0;
		/* bytes to be sent - first as one chunk */
		bytes_per_chunk =
		    (bytes_per_pixel_x100 * video_config->h_active_pixels) /
		    100 + VIDEO_PACKET_OVERHEAD;
		/* bytes being received through the DPI interface per byte clock cycle */
		total_bytes =
		    (ratio_clock_xPF * video_config->no_of_lanes *
		     (video_config->h_total_pixels -
		      video_config->h_back_porch_pixels -
		      video_config->h_sync_pixels)) / PRECISION_FACTOR;
		pr_debug("---->total_bytes:%d, bytes_per_chunk:%d\n", total_bytes,
		         bytes_per_chunk);
		/* check if the in pixels actually fit on the DSI link */
		if (total_bytes >= bytes_per_chunk) {
			chunk_overhead = total_bytes - bytes_per_chunk;
			/* overhead higher than 1 -> enable multi packets */
			if (chunk_overhead > 1) {
				for (video_size = video_size_step; video_size < video_config->h_active_pixels; video_size += video_size_step) { /* determine no of chunks */
					if ((((video_config->h_active_pixels *
					       PRECISION_FACTOR) / video_size) %
					     PRECISION_FACTOR) == 0) {
						no_of_chunks =
						    video_config->
						    h_active_pixels /
						    video_size;
						bytes_per_chunk =
						    (bytes_per_pixel_x100 *
						     video_size) / 100 +
						    VIDEO_PACKET_OVERHEAD;
						if (total_bytes >=
						    (bytes_per_chunk *
						     no_of_chunks)) {
							bytes_left =
							    total_bytes -
							    (bytes_per_chunk *
							     no_of_chunks);
							break;
						}
					}
				}
				/* prevent overflow (unsigned - unsigned) */
				if (bytes_left >
				    (NULL_PACKET_OVERHEAD * no_of_chunks)) {
					null_packet_size =
					    (bytes_left -
					     (NULL_PACKET_OVERHEAD *
					      no_of_chunks)) / no_of_chunks;
					if (null_packet_size > MAX_NULL_SIZE) { /* avoid register overflow */
						null_packet_size =
						    MAX_NULL_SIZE;
					}
				}
			} else {    /* no multi packets */
				no_of_chunks = 1;
#ifdef CONFIG_DSI_DPI_DEBUG
				/*      D E B U G               */
				{
					pr_info("no multi no null video");
					pr_info("h line time %d",
					        (unsigned
					         short)((video_config->
					                 h_total_pixels *
					                 ratio_clock_xPF) /
					                PRECISION_FACTOR));
					pr_info("video_size %d", video_size);
				}
#endif
				/* video size must be a multiple of 4 when not 18 loosely */
				for (video_size = video_config->h_active_pixels;
				     (video_size % video_size_step) != 0;
				     video_size++) {
					;
				}
			}
		} else {
			pr_err
			("resolution cannot be sent to display through current settings");
			err_code = ERR_DSI_OVERFLOW;

		}
	}
	err_code =
	    err_code ? err_code : mipi_dsih_hal_dpi_chunks_no(dsi,
	            no_of_chunks);
	err_code =
	    err_code ? err_code : mipi_dsih_hal_dpi_video_packet_size(dsi,
	            video_size);
	err_code =
	    err_code ? err_code : mipi_dsih_hal_dpi_null_packet_size(dsi,
	            null_packet_size);

	// mipi_dsih_hal_dpi_null_packet_en(dsi, null_packet_size > 0? 1: 0);
	// mipi_dsih_hal_dpi_multi_packet_en(dsi, (no_of_chunks > 1)? 1: 0);
#ifdef  CONFIG_DSI_DPI_DEBUG
	/*      D E B U G               */
	{
		pr_info("total_bytes %d ,", total_bytes);
		pr_info("bytes_per_chunk %d ,", bytes_per_chunk);
		pr_info("bytes left %d ,", bytes_left);
		pr_info("null packets %d ,", null_packet_size);
		pr_info("chunks %d ,", no_of_chunks);
		pr_info("video_size %d ", video_size);
	}
#endif
	mipi_dsih_hal_dpi_video_vc(dsi, video_config->virtual_channel);
	jz_dsih_dphy_no_of_lanes(dsi, video_config->no_of_lanes);
	/* enable high speed clock */
	mipi_dsih_dphy_enable_hs_clk(dsi, 1);
	pr_debug("video configure is ok!\n");
	return err_code;

}

/* set all register settings to MIPI DSI controller. */
dsih_error_t jz_dsi_phy_cfg(struct dsi_device *dsi)
{
	dsih_error_t err = OK;
	err = jz_dsi_set_clock(dsi);
	if (err) {
		return err;
	}
	err = jz_init_dsi(dsi);
	if (err) {
		return err;
	}
	return OK;
}

int jz_dsi_phy_open(struct dsi_device *dsi)
{
	struct video_config *video_config;
	video_config = dsi->video_config;

	pr_debug("entry %s()\n", __func__);

	jz_dsih_dphy_reset(dsi, 0);
	jz_dsih_dphy_stop_wait_time(dsi, 0x1c); /* 0x1c: */

	if (video_config->no_of_lanes > 4 || video_config->no_of_lanes < 1) {
		return ERR_DSI_OUT_OF_BOUND;
	}
	jz_dsih_dphy_no_of_lanes(dsi, video_config->no_of_lanes);

	jz_dsih_dphy_clock_en(dsi, 1);
	jz_dsih_dphy_shutdown(dsi, 1);
	jz_dsih_dphy_reset(dsi, 1);

	dsi->dsi_phy->status = INITIALIZED;
	return OK;
}

void jz_dsi_phy_stop(struct dsi_device *dsi)
{
	jz_dsih_dphy_reset(dsi, 0);
	jz_dsih_dphy_shutdown(dsi, 0);
	mipi_dsih_hal_power(dsi, 0);
	jz_dsih_dphy_clock_en(dsi, 0);
	clk_disable_unprepare(dsi->clk);
}

char *dump_dsi_reg_to_buf(struct dsi_device *dsi, char *buf)
{
	char *p = buf;

	p += sprintf(p, "dsi->address: %x\n", dsi->address);
	p += sprintf(p, "dsi->phy_address: %x\n", dsi->phy_address);
	p += sprintf(p, "dsi->dev: ===========>dump dsi reg\n");
	p += sprintf(p, "dsi->dev: VERSION------------:%08x\n", mipi_dsih_read_word(dsi, R_DSI_HOST_VERSION));
	p += sprintf(p, "dsi->dev: PWR_UP:------------:%08x\n", mipi_dsih_read_word(dsi, R_DSI_HOST_PWR_UP));
	p += sprintf(p, "dsi->dev: CLKMGR_CFG---------:%08x\n", mipi_dsih_read_word(dsi, R_DSI_HOST_CLKMGR_CFG));
	p += sprintf(p, "dsi->dev: DPI_VCID-----------:%08x\n", mipi_dsih_read_word(dsi, R_DSI_HOST_DPI_VCID));
	p += sprintf(p, "dsi->dev: DPI_COLOR_CODING---:%08x\n", mipi_dsih_read_word(dsi, R_DSI_HOST_DPI_COLOR_CODING));
	p += sprintf(p, "dsi->dev: DPI_CFG_POL--------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_DPI_CFG_POL));
	p += sprintf(p, "dsi->dev: DPI_LP_CMD_TIM-----:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_DPI_LP_CMD_TIM));
	p += sprintf(p, "dsi->dev: DBI_VCID-----------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_DBI_VCID));
	p += sprintf(p, "dsi->dev: DBI_CFG------------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_DBI_CFG));
	p += sprintf(p, "dsi->dev: DBI_PARTITIONING_EN:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_DBI_PARTITIONING_EN));
	p += sprintf(p, "dsi->dev: DBI_CMDSIZE--------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_DBI_CMDSIZE));
	p += sprintf(p, "dsi->dev: PCKHDL_CFG---------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_PCKHDL_CFG));
	p += sprintf(p, "dsi->dev: GEN_VCID-----------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_GEN_VCID));
	p += sprintf(p, "dsi->dev: MODE_CFG-----------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_MODE_CFG));
	p += sprintf(p, "dsi->dev: VID_MODE_CFG-------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_MODE_CFG));
	p += sprintf(p, "dsi->dev: VID_PKT_SIZE-------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_PKT_SIZE));
	p += sprintf(p, "dsi->dev: VID_NUM_CHUNKS-----:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_NUM_CHUNKS));
	p += sprintf(p, "dsi->dev: VID_NULL_SIZE------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_NULL_SIZE));
	p += sprintf(p, "dsi->dev: VID_HSA_TIME-------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_HSA_TIME));
	p += sprintf(p, "dsi->dev: VID_HBP_TIME-------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_HBP_TIME));
	p += sprintf(p, "dsi->dev: VID_HLINE_TIME-----:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_HLINE_TIME));
	p += sprintf(p, "dsi->dev: VID_VSA_LINES------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_VSA_LINES));
	p += sprintf(p, "dsi->dev: VID_VBP_LINES------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_VBP_LINES));
	p += sprintf(p, "dsi->dev: VID_VFP_LINES------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_VFP_LINES));
	p += sprintf(p, "dsi->dev: VID_VACTIVE_LINES--:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_VACTIVE_LINES));
	p += sprintf(p, "dsi->dev: EDPI_CMD_SIZE------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_EDPI_CMD_SIZE));
	p += sprintf(p, "dsi->dev: CMD_MODE_CFG-------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_CMD_MODE_CFG));
	p += sprintf(p, "dsi->dev: GEN_HDR------------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_GEN_HDR));
	p += sprintf(p, "dsi->dev: GEN_PLD_DATA-------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_GEN_PLD_DATA));
	p += sprintf(p, "dsi->dev: CMD_PKT_STATUS-----:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_CMD_PKT_STATUS));
	p += sprintf(p, "dsi->dev: TO_CNT_CFG---------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_TO_CNT_CFG));
	p += sprintf(p, "dsi->dev: HS_RD_TO_CNT-------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_HS_RD_TO_CNT));
	p += sprintf(p, "dsi->dev: LP_RD_TO_CNT-------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_LP_RD_TO_CNT));
	p += sprintf(p, "dsi->dev: HS_WR_TO_CNT-------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_HS_WR_TO_CNT));
	p += sprintf(p, "dsi->dev: LP_WR_TO_CNT_CFG---:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_LP_WR_TO_CNT));
	p += sprintf(p, "dsi->dev: BTA_TO_CNT---------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_BTA_TO_CNT));
	p += sprintf(p, "dsi->dev: SDF_3D-------------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_SDF_3D));
	p += sprintf(p, "dsi->dev: LPCLK_CTRL---------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_LPCLK_CTRL));
	p += sprintf(p, "dsi->dev: PHY_TMR_LPCLK_CFG--:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_PHY_TMR_LPCLK_CFG));
	p += sprintf(p, "dsi->dev: PHY_TMR_CFG--------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_PHY_TMR_CFG));
	p += sprintf(p, "dsi->dev: PHY_RSTZ-----------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_PHY_RSTZ));
	p += sprintf(p, "dsi->dev: PHY_IF_CFG---------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_PHY_IF_CFG));
	p += sprintf(p, "dsi->dev: PHY_ULPS_CTRL------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_PHY_ULPS_CTRL));
	p += sprintf(p, "dsi->dev: PHY_TX_TRIGGERS----:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_PHY_TX_TRIGGERS));
	p += sprintf(p, "dsi->dev: PHY_STATUS---------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_PHY_STATUS));
	p += sprintf(p, "dsi->dev: PHY_TST_CTRL0------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_PHY_TST_CTRL0));
	p += sprintf(p, "dsi->dev: PHY_TST_CTRL1------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_PHY_TST_CTRL1));
	p += sprintf(p, "dsi->dev: INT_ST0------------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_INT_ST0));
	p += sprintf(p, "dsi->dev: INT_ST1------------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_INT_ST1));
	p += sprintf(p, "dsi->dev: INT_MSK0-----------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_INT_MSK0));
	p += sprintf(p, "dsi->dev: INT_MSK1-----------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_INT_MSK1));
	p += sprintf(p, "dsi->dev: INT_FORCE0---------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_INT_FORCE0));
	p += sprintf(p, "dsi->dev: INT_FORCE1---------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_INT_FORCE1));
	p += sprintf(p, "dsi->dev: VID_SHADOW_CTRL----:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_SHADOW_CTRL));
	p += sprintf(p, "dsi->dev: DPI_VCID_ACT-------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_DPI_VCID_ACT));
	p += sprintf(p, "dsi->dev: DPI_COLOR_CODING_AC:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_DPI_COLOR_CODING_ACT));
	p += sprintf(p, "dsi->dev: DPI_LP_CMD_TIM_ACT-:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_DPI_LP_CMD_TIM_ACT));
	p += sprintf(p, "dsi->dev: VID_MODE_CFG_ACT---:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_MODE_CFG_ACT));
	p += sprintf(p, "dsi->dev: VID_PKT_SIZE_ACT---:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_PKT_SIZE_ACT));
	p += sprintf(p, "dsi->dev: VID_NUM_CHUNKS_ACT-:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_NUM_CHUNKS_ACT));
	p += sprintf(p, "dsi->dev: VID_HSA_TIME_ACT---:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_HSA_TIME_ACT));
	p += sprintf(p, "dsi->dev: VID_HBP_TIME_ACT---:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_HBP_TIME_ACT));
	p += sprintf(p, "dsi->dev: VID_HLINE_TIME_ACT-:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_HLINE_TIME_ACT));
	p += sprintf(p, "dsi->dev: VID_VSA_LINES_ACT--:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_VSA_LINES_ACT));
	p += sprintf(p, "dsi->dev: VID_VBP_LINES_ACT--:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_VBP_LINES_ACT));
	p += sprintf(p, "dsi->dev: VID_VFP_LINES_ACT--:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_VFP_LINES_ACT));
	p += sprintf(p, "dsi->dev: VID_VACTIVE_LINES_ACT:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_VID_VACTIVE_LINES_ACT));
	p += sprintf(p, "dsi->dev: SDF_3D_ACT---------:%08x\n",
	             mipi_dsih_read_word(dsi, R_DSI_HOST_SDF_3D_ACT));
	return p;
}
void dump_dsi_reg(struct dsi_device *dsi)
{

}

void set_base_dir_tx(struct dsi_device *dsi, void *param)
{
	int i = 0;
	register_config_t phy_direction[] = {
		{0xb4, 0x02},
		{0xb8, 0xb0},
		{0xb8, 0x100b0},
		{0xb4, 0x00},
		{0xb8, 0x000b0},
		{0xb8, 0x0000},
		{0xb4, 0x02},
		{0xb4, 0x00}
	};
	i = mipi_dsih_write_register_configuration(dsi, phy_direction,
	        (sizeof(phy_direction) /
	         sizeof(register_config_t)));
	if (i != (sizeof(phy_direction) / sizeof(register_config_t))) {
		pr_err("ERROR setting up testchip %d", i);
	}
}

static int jz_mipi_update_cfg(struct dsi_device *dsi)
{
	int ret;
	int st_mask = 0;
	int retry = 5;
	ret = jz_dsi_phy_open(dsi);
	if (ret) {
		pr_err("open the phy failed!\n");
		return ret;
	}

	mipi_dsih_write_word(dsi, R_DSI_HOST_CMD_MODE_CFG,
	                     0xffffff0);

	mipi_dsih_dphy_enable_hs_clk(dsi, 0);
	/*set command mode */
	mipi_dsih_write_word(dsi, R_DSI_HOST_MODE_CFG, 0x1);
	/*set this register for cmd size, default 0x6 */
	mipi_dsih_write_word(dsi, R_DSI_HOST_EDPI_CMD_SIZE, 6);

	/*
	 * jz_dsi_phy_cfg:
	 * PLL programming, config the output freq to DEFAULT_DATALANE_BPS.
	 * */
	ret = jz_dsi_phy_cfg(dsi);
	if (ret) {
		pr_err("phy configigure failed!\n");
		return ret;
	}

	pr_debug("wait for phy config ready\n");
	if (dsi->video_config->no_of_lanes == 2) {
		st_mask = 0x95;
	} else {
		st_mask = 0x15;
	}

	/*checkout phy clk lock and  clklane, datalane stopstate  */
	udelay(10);
	while ((mipi_dsih_read_word(dsi, R_DSI_HOST_PHY_STATUS) & st_mask) !=
	       st_mask && --retry) {
		pr_info("phy status = %08x\n", mipi_dsih_read_word(dsi, R_DSI_HOST_PHY_STATUS));
	}

	if (!retry) {
		pr_err("wait for phy config failed!\n");
		//return ret;
	}

	dsi->state = INITIALIZED;
	return 0;
}

/* for debug lcd power on/off */
int jz_mipi_dsi_set_client(struct dsi_device *dsi, int power)
{
	struct mipi_dsim_lcd_driver *client_drv = dsi->dsim_lcd_drv;
	struct mipi_dsim_lcd_device *client_dev = dsi->dsim_lcd_dev;

	switch (power) {
		case FB_BLANK_POWERDOWN:
			if (client_drv && client_drv->suspend) {
				client_drv->suspend(client_dev);
			}

			break;
		case FB_BLANK_UNBLANK:
			/* lcd panel power on. */
			if (client_drv && client_drv->power_on) {
				client_drv->power_on(client_dev, POWER_ON_LCD);
			}

			/* set lcd panel sequence commands. */
			if (client_drv && client_drv->set_sequence) {
				client_drv->set_sequence(client_dev);
			}

			break;
		case FB_BLANK_NORMAL:
			/* TODO. */
			break;

		default:
			break;
	}
	return 0;
}

static int jz_mipi_dsi_blank_mode(struct dsi_device *dsi, int power)
{
	struct mipi_dsim_lcd_driver *client_drv = dsi->dsim_lcd_drv;
	struct mipi_dsim_lcd_device *client_dev = dsi->dsim_lcd_dev;

	switch (power) {
		case FB_BLANK_POWERDOWN:
			if (dsi->suspended) {
				return 0;
			}

			if (client_drv && client_drv->suspend) {
				client_drv->suspend(client_dev);
			}

			jz_dsih_dphy_reset(dsi, 0);
			jz_dsih_dphy_shutdown(dsi, 0);
			mipi_dsih_hal_power(dsi, 0);
			jz_dsih_dphy_clock_en(dsi, 0);
			clk_disable_unprepare(dsi->clk);

			dsi->state = NOT_INITIALIZED;
			dsi->suspended = true;

			break;
		case FB_BLANK_UNBLANK:
			if (!dsi->suspended) {
				return 0;
			}

			clk_prepare_enable(dsi->clk);
			jz_mipi_update_cfg(dsi);

			/* lcd panel power on. */
			if (client_drv && client_drv->power_on) {
				client_drv->power_on(client_dev, POWER_ON_LCD);
			}

			/* set lcd panel sequence commands. */
			if (client_drv && client_drv->set_sequence) {
				client_drv->set_sequence(client_dev);
			}

			dsi->suspended = false;

			break;
		case FB_BLANK_NORMAL:
			/* TODO. */
			break;

		default:
			break;
	}

	return 0;
}

static int jz_mipi_dsi_ioctl(struct dsi_device *dsi, int cmd)
{
	struct mipi_dsim_lcd_driver *client_drv = dsi->dsim_lcd_drv;
	struct mipi_dsim_lcd_device *client_dev = dsi->dsim_lcd_dev;

	if (client_drv && client_drv->ioctl) {
		client_drv->ioctl(client_dev, cmd);
	}

	return 0;
}

int jz_dsi_mode_cfg(struct dsi_device *dsi, int mode)
{
	struct video_config *video_config;
	struct dsi_config *dsi_config;
	video_config = dsi->video_config;
	dsi_config = dsi->dsi_config;

	if (mode == 1) {
		/* video mode */
		jz_dsih_hal_power(dsi, 0);
		jz_dsi_video_cfg(dsi);
		jz_dsih_hal_power(dsi, 1);
	} else {
		mipi_dsih_write_word(dsi, R_DSI_HOST_EDPI_CMD_SIZE, 1024); /* 当设置太小时, 对于M31传输数据会有卡死的现象??, 这里设置成1024。*/
		mipi_dsih_dphy_enable_hs_clk(dsi, 1);
		mipi_dsih_dphy_auto_clklane_ctrl(dsi, 1);
		mipi_dsih_write_word(dsi, R_DSI_HOST_CMD_MODE_CFG, 1);

		/* cmd mode */
		/* color coding fix to 24bit ???? */
		mipi_dsih_hal_dpi_frame_ack_en(dsi, video_config->receive_ack_packets);
		if (video_config->receive_ack_packets) {    /* if ACK is requested, enable BTA, otherwise leave as is */
			mipi_dsih_hal_bta_en(dsi, 1);
		}
		if (dsi_config->te_mipi_en) {
			mipi_dsih_hal_tear_effect_ack_en(dsi, 1);
		} else {
			mipi_dsih_hal_tear_effect_ack_en(dsi, 0);
		}
		mipi_dsih_hal_gen_set_mode(dsi, 1);
		mipi_dsih_hal_dpi_color_coding(dsi, dsi->video_config->color_coding);
	}

	return 0;
}

static int jz_dsi_query_te(struct dsi_device *dsi)
{
	struct dsi_cmd_packet set_tear_on = {0x15, 0x35, 0x00};
	/* unsigned int cmd_mode_cfg; */

	if (!dsi->dsi_config->te_mipi_en) {
		return 0;
	}

	//return 0;

	/* According to DSI host spec. */
	/*
	Tearing effect request must always be triggered by a set_tear_on
	command in the DWC_mipi_dsi_host implementation
	*/
	write_command(dsi, set_tear_on);

	return 0;

}

/* define MIPI-DSI Master operations. */
static struct dsi_master_ops jz_master_ops = {
	.video_cfg      = jz_dsi_video_cfg,
	.mode_cfg       = jz_dsi_mode_cfg,
	.cmd_write      = write_command,    /*jz_dsi_wr_data, */
	.query_te       = jz_dsi_query_te,
	.cmd_read       = read_command, /*jz_dsi_rd_data, */
	.ioctl          = jz_mipi_dsi_ioctl,
	.set_blank_mode = jz_mipi_dsi_blank_mode,
};

int mipi_dsi_register_lcd_device(struct mipi_dsim_lcd_device *lcd_dev)
{
	struct mipi_dsim_ddi *dsim_ddi;

	if (!lcd_dev->name) {
		pr_err("dsim_lcd_device name is NULL.\n");
		return -EFAULT;
	}

	dsim_ddi = kzalloc(sizeof(struct mipi_dsim_ddi), GFP_KERNEL);
	if (!dsim_ddi) {
		pr_err("failed to allocate dsim_ddi object.\n");
		return -ENOMEM;
	}

	dsim_ddi->dsim_lcd_dev = lcd_dev;

	mutex_lock(&mipi_dsim_lock);
	list_add_tail(&dsim_ddi->list, &dsim_ddi_list);
	mutex_unlock(&mipi_dsim_lock);

	return 0;
}
EXPORT_SYMBOL(mipi_dsi_register_lcd_device);
static struct mipi_dsim_ddi *mipi_dsi_find_lcd_device(
    struct mipi_dsim_lcd_driver *lcd_drv)
{
	struct mipi_dsim_ddi *dsim_ddi, *next;
	struct mipi_dsim_lcd_device *lcd_dev;

	mutex_lock(&mipi_dsim_lock);

	list_for_each_entry_safe(dsim_ddi, next, &dsim_ddi_list, list) {
		if (!dsim_ddi) {
			goto out;
		}

		lcd_dev = dsim_ddi->dsim_lcd_dev;
		if (!lcd_dev) {
			continue;
		}

		if ((strcmp(lcd_drv->name, lcd_dev->name)) == 0) {
			/**
			 * bus_id would be used to identify
			 * connected bus.
			 */
			dsim_ddi->bus_id = lcd_dev->bus_id;
			mutex_unlock(&mipi_dsim_lock);

			return dsim_ddi;
		}

		list_del(&dsim_ddi->list);
		kfree(dsim_ddi);
	}

out:
	mutex_unlock(&mipi_dsim_lock);

	return NULL;
}

int mipi_dsi_register_lcd_driver(struct mipi_dsim_lcd_driver *lcd_drv)
{
	struct mipi_dsim_ddi *dsim_ddi;

	if (!lcd_drv->name) {
		pr_err("dsim_lcd_driver name is NULL.\n");
		return -EFAULT;
	}

	dsim_ddi = mipi_dsi_find_lcd_device(lcd_drv);
	if (!dsim_ddi) {
		pr_err("mipi_dsim_ddi object not found.\n");
		return -EFAULT;
	}

	dsim_ddi->dsim_lcd_drv = lcd_drv;

	pr_info("registered panel driver(%s) to mipi-dsi driver.\n",
	        lcd_drv->name);

	return 0;

}
EXPORT_SYMBOL(mipi_dsi_register_lcd_driver);

int mipi_dsi_lcd_query_status(struct dsi_device *dsi)
{
	int ret = 0;

	struct mipi_dsim_lcd_driver *lcd_drv = dsi->dsim_lcd_drv;
	struct mipi_dsim_lcd_device *lcd_dev = dsi->dsim_lcd_dev;

	if (lcd_drv && lcd_drv->query_status) {
		ret = lcd_drv->query_status(lcd_dev);
	}

	return ret;
}

int mipi_dsi_lcd_set_display_on(struct dsi_device *dsi, int en)
{
	int ret = 0;

	struct mipi_dsim_lcd_driver *lcd_drv = dsi->dsim_lcd_drv;
	struct mipi_dsim_lcd_device *lcd_dev = dsi->dsim_lcd_dev;

	if (lcd_drv && lcd_drv->set_display_on) {
		ret = lcd_drv->set_display_on(lcd_dev, en);
	}

	return ret;
}

static void dummy_release(struct device *dev)
{

}

static struct mipi_dsim_ddi *mipi_dsi_bind_lcd_ddi(
    struct dsi_device *dsim,
    const char *name)
{
	struct mipi_dsim_ddi *dsim_ddi, *next;
	struct mipi_dsim_lcd_driver *lcd_drv;
	struct mipi_dsim_lcd_device *lcd_dev;
	int ret;

	mutex_lock(&dsim->lock);

	list_for_each_entry_safe(dsim_ddi, next, &dsim_ddi_list, list) {
		lcd_drv = dsim_ddi->dsim_lcd_drv;
		lcd_dev = dsim_ddi->dsim_lcd_dev;

		pr_debug("dsi->dev: lcd_drv->name  = %s, name = %s\n",
		         lcd_drv->name, name);
		if ((strcmp(lcd_drv->name, name) == 0)) {
			lcd_dev->master = dsim;

			dev_set_name(&lcd_dev->dev, "%s", lcd_drv->name);

			ret = device_register(&lcd_dev->dev);
			if (ret < 0) {
				pr_err("can't register %s, status %d\n",
				       dev_name(&lcd_dev->dev), ret);
				mutex_unlock(&dsim->lock);
				return NULL;
			}

			lcd_dev->dev.release = dummy_release;

			dsim->dsim_lcd_dev = lcd_dev;
			dsim->dsim_lcd_drv = lcd_drv;

			mutex_unlock(&dsim->lock);

			return dsim_ddi;
		} else {
			pr_err("dsi->dev: lcd_drv->name is different with fb name\n");
		}
	}

	mutex_unlock(&dsim->lock);

	return NULL;
}

static struct mipi_dsim_ddi *mipi_dsi_unbind_lcd_ddi(struct dsi_device *dsim, const char *name)
{
	struct mipi_dsim_ddi *dsim_ddi, *next;
	struct mipi_dsim_lcd_driver *lcd_drv;
	struct mipi_dsim_lcd_device *lcd_dev;

	mutex_lock(&dsim->lock);
	list_for_each_entry_safe(dsim_ddi, next, &dsim_ddi_list, list) {

		lcd_drv = dsim_ddi->dsim_lcd_drv;
		lcd_dev = dsim_ddi->dsim_lcd_dev;

		pr_debug("dsi->dev: lcd_drv->name  = %s, name = %s\n",
		         lcd_drv->name, name);
		if ((strcmp(lcd_drv->name, name) == 0)) {

			device_unregister(&lcd_dev->dev);

			/*TODO:*/
			dsim->dsim_lcd_dev = NULL;
			dsim->dsim_lcd_drv = NULL;

			mutex_unlock(&dsim->lock);

			return dsim_ddi;
		} else {
			pr_err("dsi->dev: lcd_drv->name is different with fb name\n");
		}
	}

	mutex_unlock(&dsim->lock);

	return NULL;
}

struct dsi_device *jz_lvds_init(struct jzdsi_data *pdata, int panel_type)
{
	struct dsi_device *dsi;
	int ret = -1;
	dsi = (struct dsi_device *)kzalloc(sizeof(struct dsi_device), GFP_KERNEL);
	if (!dsi) {
		pr_err("dsi->dev: failed to allocate dsi object.\n");
		ret = -ENOMEM;
		goto err_dsi;
	}

	if (panel_type == LCD_TYPE_LVDS_VESA) {
		dsi->phy_mode = DSIPHY_LVDS_VESA_MODE;
	} else {
		dsi->phy_mode = DSIPHY_LVDS_JEIDA_MODE;
	}

	dsi->video_config = &(pdata->video_config);
	//dsi->video_config->h_polarity = pdata->modes->sync & FB_SYNC_HOR_HIGH_ACT;
	dsi->video_config->pixel_clock = pdata->video_config.pixel_clock;
	dsi->video_config->h_active_pixels = pdata->modes->xres;
	dsi->video_config->h_sync_pixels = pdata->modes->hsync_len;
	dsi->video_config->h_back_porch_pixels = pdata->modes->left_margin;
	dsi->video_config->h_total_pixels = pdata->modes->xres + pdata->modes->hsync_len + pdata->modes->left_margin + pdata->modes->right_margin;
	dsi->video_config->v_active_lines = pdata->modes->yres;
	//dsi->video_config->v_polarity =  pdata->modes->sync & FB_SYNC_VERT_HIGH_ACT;
	dsi->video_config->v_sync_lines = pdata->modes->vsync_len;
	dsi->video_config->v_back_porch_lines = pdata->modes->upper_margin;
	dsi->video_config->v_total_lines = pdata->modes->yres + pdata->modes->upper_margin + pdata->modes->lower_margin + pdata->modes->vsync_len;

	dsi->clk = clk_get(NULL, "gate_dsi");
	if (IS_ERR(dsi->clk)) {
		pr_err("failed to get dsi clock source\n");
		goto err_get_clk;
	}
	clk_prepare_enable(dsi->clk);
	dsi->phy_address = (unsigned int)ioremap(pdata->dsi_phy_iobase, 0x1000);
	if (!dsi->phy_address) {
		pr_err("Failed to ioremap dsi phy memory region!\n");
		goto err_iounmap_phyio;
	}

	mutex_init(&dsi->lock);

	init_dsi_phy(dsi);

	return dsi;

err_iounmap_phyio:
	clk_put(dsi->clk);
err_get_clk:
	kfree(dsi);
err_dsi:
	return NULL;

}
int dsi_change_video_config(struct dsi_device *dsi, struct lcd_panel *panel)
{
	struct jzdsi_data *pdata = &panel->dsi_pdata[panel->cur_active_videomode];
	dsi->video_config = &(pdata->video_config);
	dsi->video_config->pixel_clock = PICOS2KHZ(pdata->modes->pixclock);
	dsi->video_config->h_polarity = pdata->modes->sync & FB_SYNC_HOR_HIGH_ACT;
	dsi->video_config->h_active_pixels = pdata->modes->xres;
	dsi->video_config->h_sync_pixels = pdata->modes->hsync_len;
	dsi->video_config->h_back_porch_pixels = pdata->modes->left_margin;
	dsi->video_config->h_total_pixels = pdata->modes->xres + pdata->modes->hsync_len + pdata->modes->left_margin + pdata->modes->right_margin;
	dsi->video_config->v_active_lines = pdata->modes->yres;
	dsi->video_config->v_polarity =  pdata->modes->sync & FB_SYNC_VERT_HIGH_ACT;
	dsi->video_config->v_sync_lines = pdata->modes->vsync_len;
	dsi->video_config->v_back_porch_lines = pdata->modes->upper_margin;
	dsi->video_config->v_total_lines = pdata->modes->yres + pdata->modes->upper_margin + pdata->modes->lower_margin + pdata->modes->vsync_len;
	if (!dsi->video_config->byte_clock) {
		dsi->video_config->byte_clock = dsi->video_config->pixel_clock * (pdata->bpp_info / 8) / dsi->video_config->no_of_lanes;
		dsi->video_config->byte_clock = dsi->video_config->byte_clock * 3 / 2;  // KHz
	}

	if (dsi->video_config->byte_clock * 8 > dsi->dsi_config->max_bps * 1000) {
		dsi->video_config->byte_clock = dsi->dsi_config->max_bps * 1000 / 8;
		pr_err("+++++++++++++warning: DATALANE_BPS is over lcd max_bps allowed ,auto set it lcd max_bps\n");
	}

	dsi->real_mipiclk = dsi->video_config->byte_clock * 8 * 1000;
	/*dsi->video_config->byte_clock = dsi->video_config->pixel_clock / 4;*/

	pr_debug("%s, %d byte_clock %d KHz, pixel_clock:%d KHz\n", __func__, __LINE__, dsi->video_config->byte_clock, dsi->video_config->pixel_clock);
	return 0;

}

irqreturn_t dsi_irq_handler(int irq, void *data)
{
	struct dpu_ctrl *dctrl = (struct dpu_ctrl *)data;
	struct dsi_device *dsi = dctrl->dsi;
	int irq_flag0 = 0;
	int irq_flag1 = 0;
	irq_flag0 = mipi_dsih_hal_error_status_0(dsi, INT_ST0_ALL);
	irq_flag1 = mipi_dsih_hal_error_status_1(dsi, INT_ST1_ALL);

	if (atomic_read(&dsi->need_recovery)) {
		return IRQ_HANDLED;
	}
	spin_lock(&dsi->irq_lock);
	if (irq_flag0 & ACK_WITH_ERR_0) {
		printk("SoT error from the Acknowledge error report!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & ACK_WITH_ERR_1) {
		printk("SoT Sync error from the Acknowledge error report!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & ACK_WITH_ERR_2) {
		printk("EoT Sync error from the Acknowledge error report!!!\n");
		atomic_set(&dsi->need_recovery, 1);
		schedule_work(&dctrl->dsi_work);
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & ACK_WITH_ERR_3) {
		printk("Escape Mode Entry Command error from the Acknowledge error report!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & ACK_WITH_ERR_4) {
		printk("LP Transmit Sync error from the Acknowledge error report!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & ACK_WITH_ERR_5) {
		printk("Peripheral Timeout error from the Acknowledge error report!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & ACK_WITH_ERR_6) {
		printk("False Control error from the Acknowledge error report!!!\n");
		atomic_set(&dsi->need_recovery, 1);
		schedule_work(&dctrl->dsi_work);
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & ACK_WITH_ERR_7) {
		printk("reserved (specific to device) error from the Acknowledge error report!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & ACK_WITH_ERR_8) {
		printk(" ECC error, single-bit (detected and corrected) from the Acknowledge error report!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & ACK_WITH_ERR_9) {
		printk("ECC error, multi-bit (detected, not corrected) from the Acknowledge error report!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & ACK_WITH_ERR_10) {
		printk("checksum error (long packet only) from the Acknowledge error report!!!\n");
		atomic_set(&dsi->need_recovery, 1);
		schedule_work(&dctrl->dsi_work);
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & ACK_WITH_ERR_11) {
		printk("not recognized DSI data type from the Acknowledge error report!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & ACK_WITH_ERR_12) {
		printk("DSI VC ID Invalid from the Acknowledge error report!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & ACK_WITH_ERR_13) {
		printk("invalid transmission length from the Acknowledge error report!!!\n");
		atomic_set(&dsi->need_recovery, 1);
		schedule_work(&dctrl->dsi_work);
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & ACK_WITH_ERR_14) {
		printk("reserved (specific to device) from the Acknowledge error report!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & ACK_WITH_ERR_15) {
		printk("DSI protocol violation from the Acknowledge error report!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & DPHY_ERRORS_0) {
		printk("ErrEsc escape entry error from Lane 0 !!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & DPHY_ERRORS_1) {
		printk("ErrSyncEsc low-power data transmission synchronization error from Lane 0!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & DPHY_ERRORS_2) {
		printk("This bit indicates the ErrControl error from Lane 0.!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & DPHY_ERRORS_3) {
		printk("LP0 contention error ErrContentionLP0 from Lane 0!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag0 & DPHY_ERRORS_4) {
		printk("This bit indicates the LP1 contention error ErrContentionLP1 from Lane 0!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag1 & TO_HS_TX) {
		printk("high-speed transmission timeout counter reached the end!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag1 & TO_LP_RX) {
		printk("low-power reception timeout counter reached the end!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag1 & ECC_SINGLE_ERR) {
		printk("the ECC single error is detected and corrected in a received packet!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag1 & ECC_MULTI_ERR) {
		printk("the ECC multiple error is detected in a received packet!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag1 & CRC_ERR) {
		printk("the CRC error is detected in the received packet payload!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag1 & PKT_SIZE_ERR) {
		printk("the packet size error is detected during the packet reception!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag1 & EOPT_ERR) {
		printk("EoTp packet is not received at the end of the incoming peripheral transmission!!!\n");
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}
	if (irq_flag1 & DPI_PLD_WR_ERR) {
		printk("the payload FIFO becomes full and the data stored is corrupted!!!\n");
		atomic_set(&dsi->need_recovery, 1);
		schedule_work(&dctrl->dsi_work);
		spin_unlock(&dsi->irq_lock);
		return IRQ_HANDLED;
	}

	spin_unlock(&dsi->irq_lock);
	return IRQ_HANDLED;
}

struct dsi_device *jzdsi_init(struct jzdsi_data *pdata)
{
	struct dsi_device *dsi;
	struct dsi_phy *dsi_phy;
	struct mipi_dsim_ddi *dsim_ddi;
	int ret = -EINVAL;
	pr_debug("entry %s()\n", __func__);

	dsi = (struct dsi_device *)kzalloc(sizeof(struct dsi_device), GFP_KERNEL);
	if (!dsi) {
		pr_err("dsi->dev: failed to allocate dsi object.\n");
		ret = -ENOMEM;
		goto err_dsi;
	}

	dsi_phy = (struct dsi_phy *)kzalloc(sizeof(struct dsi_phy), GFP_KERNEL);
	if (!dsi_phy) {
		pr_err("dsi->dev: failed to allocate dsi phy  object.\n");
		ret = -ENOMEM;
		goto err_dsi_phy;
	}
	dsi->dsi_config = &(pdata->dsi_config);

	dsi_phy->status = NOT_INITIALIZED;
	dsi_phy->reference_freq = REFERENCE_FREQ;
	dsi_phy->bsp_pre_config = set_base_dir_tx;
	dsi->dsi_phy = dsi_phy;
	dsi->video_config = &(pdata->video_config);
	dsi->video_config->pixel_clock = PICOS2KHZ(pdata->modes->pixclock);
	dsi->video_config->h_polarity = pdata->modes->sync & FB_SYNC_HOR_HIGH_ACT;
	dsi->video_config->h_active_pixels = pdata->modes->xres;
	dsi->video_config->h_sync_pixels = pdata->modes->hsync_len;
	dsi->video_config->h_back_porch_pixels = pdata->modes->left_margin;
	dsi->video_config->h_total_pixels = pdata->modes->xres + pdata->modes->hsync_len + pdata->modes->left_margin + pdata->modes->right_margin;
	dsi->video_config->v_active_lines = pdata->modes->yres;
	dsi->video_config->v_polarity =  pdata->modes->sync & FB_SYNC_VERT_HIGH_ACT;
	dsi->video_config->v_sync_lines = pdata->modes->vsync_len;
	dsi->video_config->v_back_porch_lines = pdata->modes->upper_margin;
	dsi->video_config->v_total_lines = pdata->modes->yres + pdata->modes->upper_margin + pdata->modes->lower_margin + pdata->modes->vsync_len;

	pr_debug("dsi->video_config->h_total_pixels: %d\n", dsi->video_config->h_total_pixels);
	pr_debug("dsi->video_config->v_total_lines: %d\n", dsi->video_config->v_total_lines);
	pr_debug("jzfb_pdata.modes->refresh: %d\n", pdata->modes->refresh);
	pr_debug("jzfb_pdata.bpp: %d\n", pdata->bpp_info);
	pr_debug("dsi->video_config->no_of_lanes: %d\n", dsi->video_config->no_of_lanes);
	pr_debug("---dsi->video_config->byte_clock: %d\n", dsi->video_config->byte_clock);

	if (!dsi->video_config->byte_clock) {
		dsi->video_config->byte_clock = dsi->video_config->pixel_clock * (pdata->bpp_info / 8) / dsi->video_config->no_of_lanes;
		dsi->video_config->byte_clock = dsi->video_config->byte_clock * 3 / 2;  // KHz
	}

	if (dsi->video_config->byte_clock * 8 > dsi->dsi_config->max_bps * 1000) {
		dsi->video_config->byte_clock = dsi->dsi_config->max_bps * 1000 / 8;
		pr_err("+++++++++++++warning: DATALANE_BPS is over lcd max_bps allowed ,auto set it lcd max_bps\n");
	}

	dsi->real_mipiclk = dsi->video_config->byte_clock * 8 * 1000;
	/*dsi->video_config->byte_clock = dsi->video_config->pixel_clock / 4;*/

	pr_debug("%s, %d byte_clock %d KHz, pixel_clock:%d KHz\n", __func__, __LINE__, dsi->video_config->byte_clock, dsi->video_config->pixel_clock);

	if (dsi->video_config->byte_clock < 26000) {
		printk("byte clock %d should at least 26MHz, tx escaple clk should between 13MHz to 20MHz\n", dsi->video_config->byte_clock);
	}

	// Tlpx 50ns ~ 75ns
	dsi->tx_escape_div = (dsi->video_config->byte_clock + 19999) / 20000; // 50ns
	if (dsi->tx_escape_div < 2) {
		printk("tx escape clk set error\n");
	}

	dsi->master_ops = &jz_master_ops;

	dsi->clk = clk_get(NULL, "gate_dsi");
	if (IS_ERR(dsi->clk)) {
		pr_err("failed to get dsi clock source\n");
		goto err_put_clk;
	}
	clk_prepare_enable(dsi->clk);
	dsi->address = (unsigned int)ioremap(pdata->dsi_iobase, 0x1000);
	if (!dsi->address) {
		pr_err("Failed to ioremap register dsi memory region\n");
		goto err_iounmap;
	}
	dsi->phy_address = (unsigned int)ioremap(pdata->dsi_phy_iobase, 0x1000);
	if (!dsi->phy_address) {
		pr_err("Failed to ioremap dsi phy memory region!\n");
		goto err_iounmap;
	}

	mutex_init(&dsi->lock);
	spin_lock_init(&dsi->irq_lock);
	dsim_ddi = mipi_dsi_bind_lcd_ddi(dsi, pdata->modes->name);
	if (!dsim_ddi) {
		pr_err("dsi->dev: mipi_dsim_ddi object not found.\n");
		ret = -EINVAL;
		goto err_bind_lcd;
	}

	if (mipi_dsih_hal_get_power(dsi)) {
		dsi->state = UBOOT_INITIALIZED;
	} else {
		dsi->state = NOT_INITIALIZED; /*must be here for set_sequence function*/
	}

	if (dsi->state != UBOOT_INITIALIZED) {
		ret = jz_mipi_update_cfg(dsi);
		if (ret) {
			goto err_dsi_init;
		}

		if (dsim_ddi->dsim_lcd_drv && dsim_ddi->dsim_lcd_drv->probe_dev_id) {
			ret = dsim_ddi->dsim_lcd_drv->probe_dev_id(dsim_ddi->dsim_lcd_dev);
			if (ret < 0) {
				pr_err("Failed to probe_dev_id dsi lcd panel!\n");
				goto err_lcd_probe;
			}
		}

		if (dsim_ddi->dsim_lcd_drv && dsim_ddi->dsim_lcd_drv->probe) {
			ret = dsim_ddi->dsim_lcd_drv->probe(dsim_ddi->dsim_lcd_dev);
			if (ret < 0) {
				pr_err("Failed to probe dsi lcd panel!\n");
				goto err_lcd_probe;
			}
		}

		if (dsim_ddi->dsim_lcd_drv) {
			if (dsim_ddi->dsim_lcd_drv->power_on) {
				dsim_ddi->dsim_lcd_drv->power_on(dsim_ddi->dsim_lcd_dev, 1);
			}
			if (dsim_ddi->dsim_lcd_drv->set_sequence) {
				dsim_ddi->dsim_lcd_drv->set_sequence(dsim_ddi->dsim_lcd_dev);
			}
		} else {
			pr_err("lcd mipi panel init failed!\n");
			goto err_panel_init;
		}
		/* jz_mipi_dsi_init_interrupt(dsi); */
	} else {
		if (dsim_ddi->dsim_lcd_drv && dsim_ddi->dsim_lcd_drv->probe) {
			ret = dsim_ddi->dsim_lcd_drv->probe(dsim_ddi->dsim_lcd_dev);
			if (ret < 0) {
				pr_err("Failed to probe dsi lcd panel!\n");

				goto err_lcd_probe;
			}
		}

	}

	dsi->suspended = false;
	atomic_set(&dsi->need_recovery, 0);
#ifdef CONFIG_DSI_DPI_DEBUG /*test pattern */
	unsigned int tmp = 0;
	jz_dsi_video_cfg(dsi);

	tmp = mipi_dsih_read_word(dsi, R_DSI_HOST_VID_MODE_CFG);
	tmp |= 1 << 16 | 0 << 20 | 1 << 24;
	mipi_dsih_write_word(dsi, R_DSI_HOST_VID_MODE_CFG, tmp);
#endif

	return dsi;
err_panel_init:
	pr_err("dsi->dev: lcd mipi panel init error\n");
err_lcd_probe:
	jz_dsi_phy_stop(dsi);
	mipi_dsi_unbind_lcd_ddi(dsi, pdata->modes->name);
err_dsi_init:
	pr_err("dsi->dev: lcd mipi dsi init error\n");
err_bind_lcd:
	iounmap((void __iomem *)dsi->address);
err_iounmap:
	clk_put(dsi->clk);
err_put_clk:
	kfree(dsi_phy);
err_dsi_phy:
	kfree(dsi);
err_dsi:
	return NULL;
}

void jzdsi_remove(struct dsi_device *dsi)
{
	struct dsi_phy *dsi_phy;
	dsi_phy = dsi->dsi_phy;

	iounmap((void __iomem *)dsi->address);
	clk_disable(dsi->clk);
	clk_put(dsi->clk);
	kfree(dsi_phy);
	kfree(dsi);
}
