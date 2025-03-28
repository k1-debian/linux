/*
 * Ingenic SoC MIPI-DSI hal driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of thVe GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include "jz_mipi_dsih_hal.h"
#include "jz_mipi_dsi_regs.h"
#include <linux/delay.h>
/**
 * Write a 32-bit word to the DSI Host core
 * @param dsi pointer to structure holding the DSI Host core information
 * @param reg_address register offset in core
 * @param data 32-bit word to be written to register
 */
void mipi_dsih_write_word(struct dsi_device *dsi, unsigned int reg_address, unsigned int data)
{
	writel(data, (volatile unsigned int *)(dsi->address + reg_address));
}
/**
 * Write a bit field o a 32-bit word to the DSI Host core
 * @param dsi pointer to structure holding the DSI Host core information
 * @param reg_address register offset in core
 * @param data to be written to register
 * @param shift bit shift from the left (system is BIG ENDIAN)
 * @param width of bit field
 */
void mipi_dsih_write_part(struct dsi_device *dsi, unsigned int reg_address, unsigned int data, unsigned char shift, unsigned char width)
{
	unsigned int mask = (1 << width) - 1;
	unsigned int temp = mipi_dsih_read_word(dsi, reg_address);

	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;
	mipi_dsih_write_word(dsi, reg_address, temp);
}
/**
 * Write a 32-bit word to the DSI Host core
 * @param dsi pointer to structure holding the DSI Host core information
 * @param reg_address offset of register
 * @return 32-bit word value stored in register
 */
unsigned int mipi_dsih_read_word(struct dsi_device *dsi, unsigned int reg_address)
{
	return readl((volatile unsigned int *)(dsi->address + reg_address));
}
/**
 * Write a 32-bit word to the DSI Host core
 * @param dsi pointer to structure holding the DSI Host core information
 * @param reg_address offset of register in core
 * @param shift bit shift from the left (system is BIG ENDIAN)
 * @param width of bit field
 * @return bit field read from register
 */
unsigned int mipi_dsih_read_part(struct dsi_device *dsi, unsigned int reg_address, unsigned char shift, unsigned char width)
{
	return (mipi_dsih_read_word(dsi, reg_address) >> shift) & ((1 << width) - 1);
}
/**
 * Get DSI Host core version
 * @param dsi pointer to structure holding the DSI Host core information
 * @return ascii number of the version
 */
unsigned int mipi_dsih_hal_get_version(struct dsi_device *dsi)
{
	return mipi_dsih_read_word(dsi, R_DSI_HOST_VERSION);
}
/**
 * Modify power status of DSI Host core
 * @param dsi pointer to structure holding the DSI Host core information
 * @param on (1) or off (0)
 */
void mipi_dsih_hal_power(struct dsi_device *dsi, int on)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_PWR_UP, on, 0, 1);
}
/**
 * Get the power status of the DSI Host core
 * @param dsi pointer to structure holding the DSI Host core information
 * @return power status
 */
int mipi_dsih_hal_get_power(struct dsi_device *dsi)
{
	return (int)(mipi_dsih_read_word(dsi, R_DSI_HOST_PWR_UP));
}
/**
 * Write transmission escape timeout
 * a safe guard so that the state machine would reset if transmission
 * takes too long
 * @param dsi pointer to structure holding the DSI Host core information
 * @param tx_escape_division
 */
void mipi_dsih_hal_tx_escape_division(struct dsi_device *dsi, unsigned char tx_escape_division)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_CLKMGR_CFG, tx_escape_division, 0, 8);
}
/**
 * Write the DPI video virtual channel destination
 * @param dsi pointer to structure holding the DSI Host core information
 * @param vc virtual channel
 */
void mipi_dsih_hal_dpi_video_vc(struct dsi_device *dsi, unsigned char vc)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_DPI_VCID, (unsigned int)(vc), 0, 2);
}
/**
 * Get the DPI video virtual channel destination
 * @param dsi pointer to structure holding the DSI Host core information
 * @return virtual channel
 */
unsigned char mipi_dsih_hal_dpi_get_video_vc(struct dsi_device *dsi)
{
	return mipi_dsih_read_part(dsi, R_DSI_HOST_DPI_VCID, 0, 2);
}
/**
 * Set DPI video color coding
 * @param dsi pointer to structure holding the DSI Host core information
 * @param color_coding enum (configuration and color depth)
 * @return error code
 */
dsih_error_t mipi_dsih_hal_dpi_color_coding(struct dsi_device *dsi, dsih_color_coding_t color_coding)
{
	dsih_error_t err = OK;
	if (color_coding > 7) {
		{
			printk("invalid colour configuration");
		}
		err = ERR_DSI_COLOR_CODING;
	} else {
		mipi_dsih_write_part(dsi, R_DSI_HOST_DPI_COLOR_CODING, color_coding, 0, 4);
	}
	return err;
}
/**
 * Get DPI video color coding
 * @param dsi pointer to structure holding the DSI Host core information
 * @return color coding enum (configuration and color depth)
 */
dsih_color_coding_t mipi_dsih_hal_dpi_get_color_coding(struct dsi_device *dsi)
{
	return (dsih_color_coding_t)(mipi_dsih_read_part(dsi, R_DSI_HOST_DPI_COLOR_CODING, 0, 4));
}
/**
 * Get DPI video color depth
 * @param dsi pointer to structure holding the DSI Host core information
 * @return number of bits per pixel
 */
unsigned char mipi_dsih_hal_dpi_get_color_depth(struct dsi_device *dsi)
{
	unsigned char color_depth = 0;
	switch (mipi_dsih_read_part(dsi, R_DSI_HOST_DPI_COLOR_CODING, 0, 4)) {
	case 0:
	case 1:
	case 2:
		color_depth = 16;
		break;
	case 3:
	case 4:
		color_depth = 18;
		break;
	case 5:
		color_depth = 24;
		break;
	default:
		printk("###############please make sure your configure!!!################3\n");
		break;
	}
	return color_depth;
}
/**
 * Get DPI video pixel configuration
 * @param dsi pointer to structure holding the DSI Host core information
 * @return pixel configuration
 */
unsigned char mipi_dsih_hal_dpi_get_color_config(struct dsi_device *dsi)
{
	unsigned char color_config = 0;
	switch (mipi_dsih_read_part(dsi, R_DSI_HOST_DPI_COLOR_CODING, 0, 4)) {
	case 0:
		color_config = 1;
		break;
	case 1:
		color_config = 2;
		break;
	case 2:
		color_config = 3;
		break;
	case 3:
		color_config = 1;
		break;
	case 4:
		color_config = 2;
		break;
	case 5:
		color_config = 0;
		break;
	default:
		printk("@@@@@@@@@@@@please make sure your configure!!!@@@@@@@@@@@@\n");
		break;
	}
	return color_config;
}
/**
 * Set DPI loosely packetisation video (used only when color depth = 18
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable
 */
void mipi_dsih_hal_dpi_18_loosely_packet_en(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_DPI_COLOR_CODING, enable, 8, 1);
}
/**
 * Set DPI color mode pin polarity
 * @param dsi pointer to structure holding the DSI Host core information
 * @param active_low (1) or active high (0)
 */
void mipi_dsih_hal_dpi_color_mode_pol(struct dsi_device *dsi, int active_low)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_DPI_CFG_POL, active_low, 4, 1);
}
/**
 * Set DPI shut down pin polarity
 * @param dsi pointer to structure holding the DSI Host core information
 * @param active_low (1) or active high (0)
 */
void mipi_dsih_hal_dpi_shut_down_pol(struct dsi_device *dsi, int active_low)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_DPI_CFG_POL, active_low, 3, 1);
}
/**
 * Set DPI horizontal sync pin polarity
 * @param dsi pointer to structure holding the DSI Host core information
 * @param active_low (1) or active high (0)
 */
void mipi_dsih_hal_dpi_hsync_pol(struct dsi_device *dsi, int active_low)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_DPI_CFG_POL, active_low, 2, 1);
}
/**
 * Set DPI vertical sync pin polarity
 * @param dsi pointer to structure holding the DSI Host core information
 * @param active_low (1) or active high (0)
 */
void mipi_dsih_hal_dpi_vsync_pol(struct dsi_device *dsi, int active_low)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_DPI_CFG_POL, active_low, 1, 1);
}
/**
 * Set DPI data enable pin polarity
 * @param dsi pointer to structure holding the DSI Host core information
 * @param active_low (1) or active high (0)
 */
void mipi_dsih_hal_dpi_dataen_pol(struct dsi_device *dsi, int active_low)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_DPI_CFG_POL, active_low, 0, 1);
}
/**
 * Enable Low Power Command
 * @param enable the command transmissions only in low power mode.
 * @param enable (1) - disable (0)
 */
void mipi_dsih_hal_dpi_lp_cmd_en(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_VID_MODE_CFG, enable, 15, 1);
}
/**
 * Enable FRAME BTA ACK
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable (1) - disable (0)
 */
void mipi_dsih_hal_dpi_frame_ack_en(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_VID_MODE_CFG, enable, 14, 1);
}
/**
 * Enable null packets (value in null packet size will be taken in calculations)
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable (1) - disable (0)
 */
//void mipi_dsih_hal_dpi_null_packet_en(struct dsi_device * dsi, int enable)
//{
//  mipi_dsih_write_part(dsi, R_DSI_HOST_VID_MODE_CFG, enable, 10, 1);
//}
/**
 * Enable multi packets (value in no of chunks will be taken in calculations)
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable (1) - disable (0)
 */
//void mipi_dsih_hal_dpi_multi_packet_en(struct dsi_device * dsi, int enable)
//{
//  mipi_dsih_write_part(dsi, R_DSI_HOST_VID_MODE_CFG, enable, 9, 1);
//}
/**
 * Enable return to low power mode inside horizontal front porch periods when
 *  timing allows
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable (1) - disable (0)
 */
void mipi_dsih_hal_dpi_lp_during_hfp(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_VID_MODE_CFG, enable, 13, 1);
}
/**
 * Enable return to low power mode inside horizontal back porch periods when
 *  timing allows
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable (1) - disable (0)
 */
void mipi_dsih_hal_dpi_lp_during_hbp(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_VID_MODE_CFG, enable, 12, 1);
}
/**
 * Enable return to low power mode inside vertical active lines periods when
 *  timing allows
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable (1) - disable (0)
 */
void mipi_dsih_hal_dpi_lp_during_vactive(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_VID_MODE_CFG, enable, 11, 1);
}
/**
 * Enable return to low power mode inside vertical front porch periods when
 *  timing allows
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable (1) - disable (0)
 */
void mipi_dsih_hal_dpi_lp_during_vfp(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_VID_MODE_CFG, enable, 10, 1);
}
/**
 * Enable return to low power mode inside vertical back porch periods when
 * timing allows
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable (1) - disable (0)
 */
void mipi_dsih_hal_dpi_lp_during_vbp(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_VID_MODE_CFG, enable, 9, 1);
}
/**
 * Enable return to low power mode inside vertical sync periods when
 *  timing allows
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable (1) - disable (0)
 */
void mipi_dsih_hal_dpi_lp_during_vsync(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_VID_MODE_CFG, enable, 8, 1);
}
/**
 * Set DPI video mode type (burst/non-burst - with sync pulses or events)
 * @param dsi pointer to structure holding the DSI Host core information
 * @param type
 * @return error code
 */
dsih_error_t mipi_dsih_hal_dpi_video_mode_type(struct dsi_device *dsi, dsih_video_mode_t type)
{
	if (type < 3) {
		mipi_dsih_write_part(dsi, R_DSI_HOST_VID_MODE_CFG, type, 0, 2);
		return OK;
	} else {
		{
			printk("undefined type");
		}
		return ERR_DSI_OUT_OF_BOUND;
	}
}
/**
 * Write the null packet size - will only be taken into account when null
 * packets are enabled.
 * @param dsi pointer to structure holding the DSI Host core information
 * @param size of null packet
 * @return error code
 */
dsih_error_t mipi_dsih_hal_dpi_null_packet_size(struct dsi_device *dsi, unsigned short size)
{
	if (size < 0x3ff) { /* 10-bit field */
		mipi_dsih_write_part(dsi, R_DSI_HOST_VID_NULL_SIZE, size, 0, 13);
		return OK;
	} else {
		return ERR_DSI_OUT_OF_BOUND;
	}
}
/**
 * Write no of chunks to core - taken into consideration only when multi packet
 * is enabled
 * @param dsi pointer to structure holding the DSI Host core information
 * @param no of chunks
 */
dsih_error_t mipi_dsih_hal_dpi_chunks_no(struct dsi_device *dsi, unsigned short no)
{
	if (no < 0x3ff) {
		mipi_dsih_write_part(dsi, R_DSI_HOST_VID_NUM_CHUNKS, no, 0, 13);
		return OK;
	} else {
		return ERR_DSI_OUT_OF_BOUND;
	}
}
/**
 * Write video packet size. obligatory for sending video
 * @param dsi pointer to structure holding the DSI Host core information
 * @param size of video packet - containing information
 * @return error code
 */
dsih_error_t mipi_dsih_hal_dpi_video_packet_size(struct dsi_device *dsi, unsigned short size)
{
	if (size < 0x7ff) { /* 11-bit field */
		mipi_dsih_write_part(dsi, R_DSI_HOST_VID_PKT_SIZE, size, 0, 14);
		return OK;
	} else {
		return ERR_DSI_OUT_OF_BOUND;
	}
}
/**
 * Enable tear effect acknowledge
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable (1) - disable (0)
 */
void mipi_dsih_hal_tear_effect_ack_en(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_CMD_MODE_CFG, enable, 0, 1);
}
/**
 * Enable packets acknowledge request after each packet transmission
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable (1) - disable (0)
 */
void mipi_dsih_hal_cmd_ack_en(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_CMD_MODE_CFG, enable, 1, 1);
}
/**
 * Set DCS command packet transmission to transmission type
 * @param dsi pointer to structure holding the DSI Host core information
 * @param no_of_param of command
 * @param lp transmit in low power
 * @return error code
 */
dsih_error_t mipi_dsih_hal_dcs_wr_tx_type(struct dsi_device *dsi, unsigned no_of_param, int lp)
{
	switch (no_of_param) {
	case 0:
		mipi_dsih_write_part(dsi, R_DSI_HOST_CMD_MODE_CFG, lp, 16, 1);
		break;
	case 1:
		mipi_dsih_write_part(dsi, R_DSI_HOST_CMD_MODE_CFG, lp, 17, 1);
		break;
	default:
		mipi_dsih_write_part(dsi, R_DSI_HOST_CMD_MODE_CFG, lp, 19, 1);
		break;
	}
	return OK;
}
/**
 * Set DCS read command packet transmission to transmission type
 * @param dsi pointer to structure holding the DSI Host core information
 * @param no_of_param of command
 * @param lp transmit in low power
 * @return error code
 */
dsih_error_t mipi_dsih_hal_dcs_rd_tx_type(struct dsi_device *dsi, unsigned no_of_param, int lp)
{
	dsih_error_t err = OK;
	switch (no_of_param) {
	case 0:
		mipi_dsih_write_part(dsi, R_DSI_HOST_CMD_MODE_CFG, lp, 18, 1);
		break;
	default:
		printk("undefined DCS Read packet type");
		err = ERR_DSI_OUT_OF_BOUND;
		break;
	}
	return err;
}
/**
 * Set generic write command packet transmission to transmission type
 * @param dsi pointer to structure holding the DSI Host core information
 * @param no_of_param of command
 * @param lp transmit in low power
 * @return error code
 */
dsih_error_t mipi_dsih_hal_gen_wr_tx_type(struct dsi_device *dsi, unsigned no_of_param, int lp)
{
	switch (no_of_param) {
	case 0:
		mipi_dsih_write_part(dsi, R_DSI_HOST_CMD_MODE_CFG, lp, 8, 1);
		break;
	case 1:
		mipi_dsih_write_part(dsi, R_DSI_HOST_CMD_MODE_CFG, lp, 9, 1);
		break;
	case 2:
		mipi_dsih_write_part(dsi, R_DSI_HOST_CMD_MODE_CFG, lp, 10, 1);
		break;
	default:
		mipi_dsih_write_part(dsi, R_DSI_HOST_CMD_MODE_CFG, lp, 14, 1);
		break;
	}
	return OK;
}
/**
 * Set generic command packet transmission to transmission type
 * @param dsi pointer to structure holding the DSI Host core information
 * @param no_of_param of command
 * @param lp transmit in low power
 * @return error code
 */
dsih_error_t mipi_dsih_hal_gen_rd_tx_type(struct dsi_device *dsi, unsigned no_of_param, int lp)
{
	dsih_error_t err = OK;
	switch (no_of_param) {
	case 0:
		mipi_dsih_write_part(dsi, R_DSI_HOST_CMD_MODE_CFG, lp, 11, 1);
		break;
	case 1:
		mipi_dsih_write_part(dsi, R_DSI_HOST_CMD_MODE_CFG, lp, 12, 1);
		break;
	case 2:
		mipi_dsih_write_part(dsi, R_DSI_HOST_CMD_MODE_CFG, lp, 13, 1);
		break;
	default:
		printk("undefined Generic Read packet type");
		err = ERR_DSI_OUT_OF_BOUND;
		break;
	}
	return err;
}
/**
 * Configure maximum read packet size command transmission type
 * @param dsi pointer to structure holding the DSI Host core information
 * @param lp set to low power
 */
void mipi_dsih_hal_max_rd_size_tx_type(struct dsi_device *dsi, int lp)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_CMD_MODE_CFG, lp, 24, 1);
}
/**
 * Enable command mode (Generic interface)
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable 1
 */
void mipi_dsih_hal_gen_set_mode(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_MODE_CFG, enable, 0, 1);
}
/**
 * Retrieve the controller's status of whether command mode is ON or not
 * @param dsi pointer to structure holding the DSI Host core information
 * @return whether command mode is ON
 */
int mipi_dsih_hal_gen_get_mode(struct dsi_device *dsi)
{
	return mipi_dsih_read_part(dsi, R_DSI_HOST_MODE_CFG, 0, 1);
}
/**
 * Configure the Horizontal Line time
 * @param dsi pointer to structure holding the DSI Host core information
 * @param time taken to transmit the total of the horizontal line
 */
void mipi_dsih_hal_dpi_hline(struct dsi_device *dsi, unsigned short time)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_VID_HLINE_TIME, time, 0, 15);
}
/**
 * Configure the Horizontal back porch time
 * @param dsi pointer to structure holding the DSI Host core information
 * @param time taken to transmit the horizontal back porch
 */
void mipi_dsih_hal_dpi_hbp(struct dsi_device *dsi, unsigned short time)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_VID_HBP_TIME, time, 0, 12);
}
/**
 * Configure the Horizontal sync time
 * @param dsi pointer to structure holding the DSI Host core information
 * @param time taken to transmit the horizontal sync
 */
void mipi_dsih_hal_dpi_hsa(struct dsi_device *dsi, unsigned short time)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_VID_HSA_TIME, time, 0, 12);
}
/**
 * Configure the vertical active lines of the video stream
 * @param dsi pointer to structure holding the DSI Host core information
 * @param lines
 */
void mipi_dsih_hal_dpi_vactive(struct dsi_device *dsi, unsigned short lines)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_VID_VACTIVE_LINES, lines, 0, 14);
}
/**
 * Configure the vertical front porch lines of the video stream
 * @param dsi pointer to structure holding the DSI Host core information
 * @param lines
 */
void mipi_dsih_hal_dpi_vfp(struct dsi_device *dsi, unsigned short lines)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_VID_VFP_LINES, lines, 0, 10);
}
/**
 * Configure the vertical back porch lines of the video stream
 * @param dsi pointer to structure holding the DSI Host core information
 * @param lines
 */
void mipi_dsih_hal_dpi_vbp(struct dsi_device *dsi, unsigned short lines)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_VID_VBP_LINES, lines, 0, 10);
}
/**
 * Configure the vertical sync lines of the video stream
 * @param dsi pointer to structure holding the DSI Host core information
 * @param lines
 */
void mipi_dsih_hal_dpi_vsync(struct dsi_device *dsi, unsigned short lines)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_VID_VSA_LINES, lines, 0, 10);
}
/**
 * configure timeout divisions (so they would have more clock ticks)
 * @param dsi pointer to structure holding the DSI Host core information
 * @param byte_clk_division_factor no of hs cycles before transiting back to LP in
 *  (lane_clk / byte_clk_division_factor)
 */
void mipi_dsih_hal_timeout_clock_division(struct dsi_device *dsi, unsigned char byte_clk_division_factor)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_CLKMGR_CFG, byte_clk_division_factor, 8, 8);
}
/**
 * Configure the Low power receive time out
 * @param dsi pointer to structure holding the DSI Host core information
 * @param count (of byte cycles)
 */
void mipi_dsih_hal_lp_rx_timeout(struct dsi_device *dsi, unsigned short count)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_TO_CNT_CFG, count, 0, 16);
}
/**
 * Configure a high speed transmission time out7
 * @param dsi pointer to structure holding the DSI Host core information
 * @param count (byte cycles)
 */
void mipi_dsih_hal_hs_tx_timeout(struct dsi_device *dsi, unsigned short count)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_TO_CNT_CFG, count, 16, 16);
}
/**
 * Get the error 0 interrupt register status
 * @param dsi pointer to structure holding the DSI Host core information
 * @param mask the mask to be read from the register
 * @return error status 0 value
 */
unsigned int mipi_dsih_hal_error_status_0(struct dsi_device *dsi, unsigned int mask)
{
	return (mipi_dsih_read_word(dsi, R_DSI_HOST_INT_ST0) & mask);
}
/**
 * Get the error 1 interrupt register status
 * @param dsi pointer to structure holding the DSI Host core information
 * @param mask the mask to be read from the register
 * @return error status 1 value
 */
unsigned int mipi_dsih_hal_error_status_1(struct dsi_device *dsi, unsigned int mask)
{
	return (mipi_dsih_read_word(dsi, R_DSI_HOST_INT_ST1) & mask);
}
/**
 * Configure MASK (hiding) of interrupts coming from error 0 source
 * @param dsi pointer to structure holding the DSI Host core information
 * @param mask to be written to the register
 */
void mipi_dsih_hal_error_mask_0(struct dsi_device *dsi, unsigned int mask)
{
	mipi_dsih_write_word(dsi, R_DSI_HOST_INT_MSK0, mask);
}
/**
 * Get the ERROR MASK  0 register status
 * @param dsi pointer to structure holding the DSI Host core information
 * @param mask the bits to read from the mask register
 */
unsigned int mipi_dsih_hal_get_error_mask_0(struct dsi_device *dsi, unsigned int mask)
{
	return (mipi_dsih_read_word(dsi, R_DSI_HOST_INT_MSK0) & mask);
}
/**
 * Configure MASK (hiding) of interrupts coming from error 0 source
 * @param dsi pointer to structure holding the DSI Host core information
 * @param mask the mask to be written to the register
 */
void mipi_dsih_hal_error_mask_1(struct dsi_device *dsi, unsigned int mask)
{
	mipi_dsih_write_word(dsi, R_DSI_HOST_INT_MSK1, mask);
}
/**
 * Get the ERROR MASK  1 register status
 * @param dsi pointer to structure holding the DSI Host core information
 * @param mask the bits to read from the mask register
 */
unsigned int mipi_dsih_hal_get_error_mask_1(struct dsi_device *dsi, unsigned int mask)
{
	return (mipi_dsih_read_word(dsi, R_DSI_HOST_INT_MSK1) & mask);
}
/* DBI NOT IMPLEMENTED */
void mipi_dsih_hal_dbi_out_color_coding(struct dsi_device *dsi, unsigned char color_depth, unsigned char option);
void mipi_dsih_hal_dbi_in_color_coding(struct dsi_device *dsi, unsigned char color_depth, unsigned char option);
void mipi_dsih_hal_dbi_lut_size(struct dsi_device *dsi, unsigned char size);
void mipi_dsih_hal_dbi_partitioning_en(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_dbi_dcs_vc(struct dsi_device *dsi, unsigned char vc);
void mipi_dsih_hal_dbi_max_cmd_size(struct dsi_device *dsi, unsigned short size);
void mipi_dsih_hal_dbi_cmd_size(struct dsi_device *dsi, unsigned short size);
void mipi_dsih_hal_dbi_max_cmd_size(struct dsi_device *dsi, unsigned short size);
int mipi_dsih_hal_dbi_rd_cmd_busy(struct dsi_device *dsi);
int mipi_dsih_hal_dbi_read_fifo_full(struct dsi_device *dsi);
int mipi_dsih_hal_dbi_read_fifo_empty(struct dsi_device *dsi);
int mipi_dsih_hal_dbi_write_fifo_full(struct dsi_device *dsi);
int mipi_dsih_hal_dbi_write_fifo_empty(struct dsi_device *dsi);
int mipi_dsih_hal_dbi_cmd_fifo_full(struct dsi_device *dsi);
int mipi_dsih_hal_dbi_cmd_fifo_empty(struct dsi_device *dsi);

/**
 * Write command header in the generic interface
 * (which also sends DCS commands) as a subset
 * @param dsi pointer to structure holding the DSI Host core information
 * @param vc of destination
 * @param packet_type (or type of DCS command)
 * @param ls_byte (if DCS, it is the DCS command)
 * @param ms_byte (only parameter of short DCS packet)
 * @return error code
 */
dsih_error_t mipi_dsih_hal_gen_packet_header(struct dsi_device *dsi, unsigned char vc, unsigned char packet_type, unsigned char ms_byte, unsigned char ls_byte)
{
	if (vc < 4) {
		mipi_dsih_write_part(dsi, R_DSI_HOST_GEN_HDR, (ms_byte <<  16) | (ls_byte << 8) | ((vc << 6) | packet_type), 0, 24);
		return OK;
	}
	return  ERR_DSI_OVERFLOW;
}
/**
 * Write the payload of the long packet commands
 * @param dsi pointer to structure holding the DSI Host core information
 * @param payload array of bytes of payload
 * @return error code
 */
dsih_error_t mipi_dsih_hal_gen_packet_payload(struct dsi_device *dsi, unsigned int payload)
{
	if (mipi_dsih_hal_gen_write_fifo_full(dsi)) {
		return ERR_DSI_OVERFLOW;
	}
	mipi_dsih_write_word(dsi, R_DSI_HOST_GEN_PLD_DATA, payload);
	return OK;

}
/**
 * Write the payload of the long packet commands
 * @param dsi pointer to structure holding the DSI Host core information
 * @param payload pointer to 32-bit array to hold read information
 * @return error code
 */
dsih_error_t  mipi_dsih_hal_gen_read_payload(struct dsi_device *dsi, unsigned int *payload)
{
	*payload = mipi_dsih_read_word(dsi, R_DSI_HOST_GEN_PLD_DATA);
	return OK;
}

/**
 * Configure the read back virtual channel for the generic interface
 * @param dsi pointer to structure holding the DSI Host core information
 * @param vc to listen to on the line
 */
void mipi_dsih_hal_gen_rd_vc(struct dsi_device *dsi, unsigned char vc)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_GEN_VCID, vc, 0, 2);
}
/**
 * Enable EOTp reception
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable
 */
void mipi_dsih_hal_gen_eotp_rx_en(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_PCKHDL_CFG, enable, 1, 1);
}
/**
 * Enable EOTp transmission
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable
 */
void mipi_dsih_hal_gen_eotp_tx_en(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_PCKHDL_CFG, enable, 0, 1);
}
/**
 * Enable Bus Turn-around request
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable
 */
void mipi_dsih_hal_bta_en(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_PCKHDL_CFG, enable, 2, 1);
}
/**
 * Enable ECC reception, error correction and reporting
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable
 */
void mipi_dsih_hal_gen_ecc_rx_en(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_PCKHDL_CFG, enable, 3, 1);
}
/**
 * Enable CRC reception, error reporting
 * @param dsi pointer to structure holding the DSI Host core information
 * @param enable
 */
void mipi_dsih_hal_gen_crc_rx_en(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_PCKHDL_CFG, enable, 4, 1);
}
/**
 * Get status of read command
 * @param dsi pointer to structure holding the DSI Host core information
 * @return 1 if busy
 */
int mipi_dsih_hal_gen_rd_cmd_busy(struct dsi_device *dsi)
{
	return mipi_dsih_read_part(dsi, R_DSI_HOST_CMD_PKT_STATUS, 6, 1);
}
/**
 * Get the FULL status of generic read payload fifo
 * @param dsi pointer to structure holding the DSI Host core information
 * @return 1 if fifo full
 */
int mipi_dsih_hal_gen_read_fifo_full(struct dsi_device *dsi)
{
	return mipi_dsih_read_part(dsi, R_DSI_HOST_CMD_PKT_STATUS, 5, 1);
}
/**
 * Get the EMPTY status of generic read payload fifo
 * @param dsi pointer to structure holding the DSI Host core information
 * @return 1 if fifo empty
 */
int mipi_dsih_hal_gen_read_fifo_empty(struct dsi_device *dsi)
{
	return mipi_dsih_read_part(dsi, R_DSI_HOST_CMD_PKT_STATUS, 4, 1);
}
/**
 * Get the FULL status of generic write payload fifo
 * @param dsi pointer to structure holding the DSI Host core information
 * @return 1 if fifo full
 */
int mipi_dsih_hal_gen_write_fifo_full(struct dsi_device *dsi)
{
	return mipi_dsih_read_part(dsi, R_DSI_HOST_CMD_PKT_STATUS, 3, 1);
}
/**
 * Get the EMPTY status of generic write payload fifo
 * @param dsi pointer to structure holding the DSI Host core information
 * @return 1 if fifo empty
 */
int mipi_dsih_hal_gen_write_fifo_empty(struct dsi_device *dsi)
{
	return mipi_dsih_read_part(dsi, R_DSI_HOST_CMD_PKT_STATUS, 2, 1);
}
/**
 * Get the FULL status of generic command fifo
 * @param dsi pointer to structure holding the DSI Host core information
 * @return 1 if fifo full
 */
int mipi_dsih_hal_gen_cmd_fifo_full(struct dsi_device *dsi)
{
	return mipi_dsih_read_part(dsi, R_DSI_HOST_CMD_PKT_STATUS, 1, 1);
}
/**
 * Get the EMPTY status of generic command fifo
 * @param dsi pointer to structure holding the DSI Host core information
 * @return 1 if fifo empty
 */
int mipi_dsih_hal_gen_cmd_fifo_empty(struct dsi_device *dsi)
{
	return mipi_dsih_read_part(dsi, R_DSI_HOST_CMD_PKT_STATUS, 0, 1);
}
/* only if DPI */
/**
 * Configure how many cycles of byte clock would the PHY module take
 * to switch from high speed to low power
 * @param dsi pointer to structure holding the DSI Host core information
 * @param no_of_byte_cycles
 * @return error code
 */
dsih_error_t mipi_dsih_phy_hs2lp_config(struct dsi_device *dsi, unsigned char no_of_byte_cycles)
{
	if (no_of_byte_cycles < 0x100) {
		mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_TMR_CFG, no_of_byte_cycles, 24, 8);
	} else {
		return ERR_DSI_OVERFLOW;
	}
	return OK;
}
/**
 * Configure how many cycles of byte clock would the PHY module take
 * to switch from to low power high speed
 * @param dsi pointer to structure holding the DSI Host core information
 * @param no_of_byte_cycles
 * @return error code
 */
dsih_error_t mipi_dsih_phy_lp2hs_config(struct dsi_device *dsi, unsigned char no_of_byte_cycles)
{
	if (no_of_byte_cycles < 0x100) {
		mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_TMR_CFG, no_of_byte_cycles, 16, 8);
	} else {
		return ERR_DSI_OVERFLOW;
	}
	return OK;
}
/**
 * Configure how many cycles of byte clock would the PHY module take
 * to turn the bus around to start receiving
 * @param dsi pointer to structure holding the DSI Host core information
 * @param no_of_byte_cycles
 * @return error code
 */
dsih_error_t mipi_dsih_phy_bta_time(struct dsi_device *dsi, unsigned short no_of_byte_cycles)
{
	if (no_of_byte_cycles < 0x8000) { /* 12-bit field */
		mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_TMR_CFG, no_of_byte_cycles, 0, 15);
	} else {
		return ERR_DSI_OVERFLOW;
	}
	return OK;
}

void mipi_dsih_dphy_test_clock(struct dsi_device *dsi, int value)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_TST_CTRL0, value, 1, 1);
}
void mipi_dsih_dphy_test_clear(struct dsi_device *dsi, int value)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_TST_CTRL0, value, 0, 1);
}
void mipi_dsih_dphy_test_en(struct dsi_device *dsi, unsigned char on_falling_edge)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_PHY_TST_CTRL1, on_falling_edge, 16, 1);
}
void mipi_dsih_dphy_test_data_in(struct dsi_device *dsi, unsigned char test_data)
{
	mipi_dsih_write_word(dsi, R_DSI_HOST_PHY_TST_CTRL1, test_data);
}

void mipi_dsih_dphy_write(struct dsi_device *dsi, unsigned char address, unsigned char *data, unsigned char data_length)
{
	unsigned i = 0;
	if (data != 0) {
		/* set the TESTCLK input high in preparation to latch in the desired test mode */
		mipi_dsih_dphy_test_clock(dsi, 1);
		/* set the desired test code in the input 8-bit bus TESTDIN[7:0] */
		mipi_dsih_dphy_test_data_in(dsi, address);
		/* set TESTEN input high  */
		mipi_dsih_dphy_test_en(dsi, 1);
		/* drive the TESTCLK input low; the falling edge captures the chosen test code into the transceiver */
		mipi_dsih_dphy_test_clock(dsi, 0);
		/* set TESTEN input low to disable further test mode code latching  */
		mipi_dsih_dphy_test_en(dsi, 0);
		/* start writing MSB first */
		for (i = data_length; i > 0; i--) {
			/* set TESTDIN[7:0] to the desired test data appropriate to the chosen test mode */
			mipi_dsih_dphy_test_data_in(dsi, data[i - 1]);
			/* pulse TESTCLK high to capture this test data into the macrocell; repeat these two steps as necessary */
			mipi_dsih_dphy_test_clock(dsi, 1);
			mipi_dsih_dphy_test_clock(dsi, 0);
		}
	}
}
void mipi_dsih_dphy_enable_hs_clk(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_LPCLK_CTRL, enable, 0, 1);
}

void mipi_dsih_dphy_auto_clklane_ctrl(struct dsi_device *dsi, int enable)
{
	mipi_dsih_write_part(dsi, R_DSI_HOST_LPCLK_CTRL, enable, 1, 1);
}

void mipi_dsih_cmd_mode(struct dsi_device *dsi, int en)
{
	if (dsi == NULL || dsi->state != INITIALIZED) {
		printk("dsi is NULL, or dsi state error\n");
		return ;
	}

	if ((!mipi_dsih_hal_gen_get_mode(dsi)) && en) {
		/* disable video mode first */
		mipi_dsih_hal_gen_set_mode(dsi, 1);
	} else if ((mipi_dsih_hal_gen_get_mode(dsi)) && !en) {
		mipi_dsih_hal_gen_set_mode(dsi, 0);
	}
	return;
}

unsigned short mipi_dsih_gen_rd_packet(struct dsi_device *dsi, unsigned char vc, unsigned char data_type, unsigned char msb_byte, unsigned char lsb_byte, unsigned char bytes_to_read, unsigned char *read_buffer)
{
	dsih_error_t err_code = OK;
	int timeout = 0;
	int counter = 0;
	int i = 0;
	int last_count = 0;
	unsigned int temp[1] = {0};
	if (dsi == 0) {
		return 0;
	}
	if (dsi->state != INITIALIZED) {
		return 0;
	}
	if (bytes_to_read < 1) {
		return 0;
	}
	if (read_buffer == 0) {
		return 0;
	}

	if (!mipi_dsih_read_word(dsi, R_DSI_HOST_LPCLK_CTRL) & 1) {
		// current low power mode.
		// 如果当前为低escape mode， 需要确保控制器工作命令模式。
		/* make sure command mode is on */
		mipi_dsih_cmd_mode(dsi, 1);
	} else {
		// 如果当前处于高速模式，控制器会在video packet间隙发送命令。
		// 不需要强制切回到 cmd mode.
		// 此时通过gen 读数据时，超时时间会较长（因为控制器会选择一个合适的实际去发送命令，并且有BTA时序.）
	}

	/* make sure receiving is enabled */
	mipi_dsih_hal_bta_en(dsi, 1);

	/* listen to the same virtual channel as the one sent to */
	mipi_dsih_hal_gen_rd_vc(dsi, vc);
	for (timeout = 0; timeout < DSIH_FIFO_ACTIVE_WAIT; timeout++) {
		/* check if payload Tx fifo is not full */
		if (!mipi_dsih_hal_gen_cmd_fifo_full(dsi)) {
			//printk("I am here!\n");
			mipi_dsih_hal_gen_packet_header(dsi, vc, data_type, msb_byte, lsb_byte);
			break;
		}
	}
	if (!(timeout < DSIH_FIFO_ACTIVE_WAIT)) {
		printk("tx rd command timed out\n");
		return 0;
	}
	/* loop for the number of words to be read */
	for (timeout = 0; timeout < DSIH_FIFO_ACTIVE_WAIT; timeout++) {
		/* check if command transaction is done */
		if (!mipi_dsih_hal_gen_rd_cmd_busy(dsi)) {
			/* printk("is not busy\n"); */
			if (!mipi_dsih_hal_gen_read_fifo_empty(dsi)) {
				for (counter = 0; (!mipi_dsih_hal_gen_read_fifo_empty(dsi)); counter += 4) {
					err_code = mipi_dsih_hal_gen_read_payload(dsi, temp);
					if (err_code) {
						return 0;
					}
					if (counter < bytes_to_read) {
						for (i = 0; i < 4; i++) {
							if ((counter + i) < bytes_to_read) {
								/* put 32 bit temp in 4 bytes of buffer passed by user*/
								read_buffer[counter + i] = (unsigned char)(temp[0] >> (i * 8));
								last_count = i + counter;
							} else {
								if ((unsigned char)(temp[0] >> (i * 8)) != 0x00) {
									last_count = i + counter;
								}
							}
						}
					} else {
						last_count = counter;
						for (i = 0; i < 4; i++) {
							if ((unsigned char)(temp[0] >> (i * 8)) != 0x00) {
								last_count = i + counter;
							}
						}
					}
				}
				return last_count + 1;
			}
		}
		mdelay(1);
	}
	printk("rx command timed out\n");
	return 0;
}

dsih_error_t mipi_dsih_gen_wr_packet(struct dsi_device *dsi, unsigned char vc, unsigned char data_type, unsigned char *params, unsigned short param_length)
{
	dsih_error_t err_code = OK;
	/* active delay iterator */
	int timeout = 0;
	/* iterators */
	int i = 0;
	int j = 0;
	/* holds padding bytes needed */
	int compliment_counter = 0;
	unsigned char *payload = 0;
	/* temporary variable to arrange bytes into words */
	unsigned int temp = 0;
	unsigned short word_count = 0;
	if (dsi == 0) {
		return ERR_DSI_INVALID_INSTANCE;
	}
	if (dsi->state != INITIALIZED) {
		return ERR_DSI_INVALID_INSTANCE;
	}
	if ((params == 0) && (param_length != 0)) { /* pointer NULL */
		return ERR_DSI_OUT_OF_BOUND;
	}
	if (param_length > 2) {
		/* long packet - write word count to header, and the rest to payload */
		payload = params + (2 * sizeof(params[0]));
		word_count = (params[1] << 8) | params[0];
		if ((param_length - 2) < word_count) {
			printk("sent > input payload. complemented with zeroes\n");
			compliment_counter = (param_length - 2) - word_count;
		} else if ((param_length - 2) > word_count) {
			printk("Overflow - input > sent. payload truncated\n");
		}
		for (i = 0; i < (param_length - 2); i += j) {
			temp = 0;
			for (j = 0; (j < 4) && ((j + i) < (param_length - 2)); j++) {
				/* temp = (payload[i + 3] << 24) | (payload[i + 2] << 16) | (payload[i + 1] << 8) | payload[i]; */
				temp |= payload[i + j] << (j * 8);
			}
			/* check if payload Tx fifo is not full */
			for (timeout = 0; timeout < DSIH_FIFO_ACTIVE_WAIT; timeout++) {
				/*send data , or parameters*/
				if (!mipi_dsih_hal_gen_packet_payload(dsi, temp)) {
					break;
				}
			}
			if (!(timeout < DSIH_FIFO_ACTIVE_WAIT)) {
				return ERR_DSI_TIMEOUT;
			}
		}
		/* if word count entered by the user more than actual parameters received
		 * fill with zeroes - a fail safe mechanism, otherwise controller will
		 * want to send data from an empty buffer */
		for (i = 0; i < compliment_counter; i++) {
			/* check if payload Tx fifo is not full */
			for (timeout = 0; timeout < DSIH_FIFO_ACTIVE_WAIT; timeout++) {
				if (!mipi_dsih_hal_gen_packet_payload(dsi, 0x00)) {
					break;
				}
			}
			if (!(timeout < DSIH_FIFO_ACTIVE_WAIT)) {
				return ERR_DSI_TIMEOUT;
			}
		}
	}

	for (timeout = 0; timeout < DSIH_FIFO_ACTIVE_WAIT; timeout++) {
		/* check if payload Tx fifo is not full */
		if (!mipi_dsih_hal_gen_cmd_fifo_full(dsi)) {
			if (param_length == 0) {
				err_code |= mipi_dsih_hal_gen_packet_header(dsi, vc, data_type, 0x0, 0x0);
			} else if (param_length == 1) {
				err_code |= mipi_dsih_hal_gen_packet_header(dsi, vc, data_type, 0x0, params[0]);
			} else {
				/*make the header*/
				err_code |= mipi_dsih_hal_gen_packet_header(dsi, vc, data_type, params[1], params[0]);
			}
			break;
		}
	}
	if (!(timeout < DSIH_FIFO_ACTIVE_WAIT)) {
		printk("dsi_gen_write timeout!\n");
		err_code = ERR_DSI_TIMEOUT;
	}
	return err_code;
}

unsigned int  mipi_dsih_write_register_configuration(struct dsi_device *dsi, register_config_t *config, uint16_t config_length)
{
	unsigned short count = 0;
	if (dsi == 0) {
		return ERR_DSI_INVALID_INSTANCE;
	}
	for (count = 0; count < config_length; count++) {
		mipi_dsih_write_word(dsi, config[count].addr, config[count].data);
	}
	return count;
}

int write_command(struct dsi_device *dsi, struct dsi_cmd_packet cmd_data)
{
	unsigned int i, j;
	unsigned int packet_type;
	unsigned char dsi_command_param[MAX_WORD_COUNT] = {0};
	unsigned short word_count = 0;
	unsigned int ret;
	/*word count*/
	packet_type = cmd_data.packet_type;
	dsi_command_param[0] = cmd_data.cmd0_or_wc_lsb;
	dsi_command_param[1] = cmd_data.cmd1_or_wc_msb;
	if (packet_type == 0x39 || packet_type == 0x29) { //dcs long packet
		word_count = ((dsi_command_param[1] << 8) | dsi_command_param[0]);
		j = 2;
		/*payload: */
		for (i = 0; i < word_count; i++) {
			dsi_command_param[j++] = cmd_data.cmd_data[i];
		}

	} else if (packet_type == 0x05 || packet_type == 0x15 || packet_type == 0x23) { //dcs short packet
		word_count = 0;
	} else {
		printk("not support packet type, please checkout!,\n");
	}
	ret = mipi_dsih_gen_wr_packet(dsi, 0, packet_type, dsi_command_param, word_count + 2);
	if (ret < 0) {
		printk("gen_wr_packet failed. ret:%d\n", ret);
	}
	mdelay(1); //Some panels failed to init without this.

	return 0;
}

int read_command(struct dsi_device *dsi, struct dsi_cmd_packet cmd_data, unsigned char *buf, int count)
{

	unsigned int packet_type;
	unsigned char dsi_command_param[MAX_WORD_COUNT] = {0};
	unsigned short word_count = 0;
	unsigned int ret;
	/*word count*/
	packet_type = cmd_data.packet_type;
	dsi_command_param[0] = cmd_data.cmd0_or_wc_lsb;
	dsi_command_param[1] = cmd_data.cmd1_or_wc_msb;
	if (packet_type == 0x04 || packet_type == 0x14 || packet_type == 0x24 || packet_type == 0x06) { //dcs short packet
		word_count = 0;
	} else {
		printk("not support packet type, please checkout!,\n");
	}

	ret = mipi_dsih_gen_rd_packet(dsi, 0, packet_type, dsi_command_param[1], dsi_command_param[0], count, buf);
	if (ret < 0) {
		printk("gen_rd_packet failed. ret:%d\n", ret);
	}
	mdelay(1); //Some panels failed to init without this.

	return 0;
}
