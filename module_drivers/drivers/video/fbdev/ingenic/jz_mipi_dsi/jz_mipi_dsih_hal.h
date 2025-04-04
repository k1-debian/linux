/*
 * @file mipi_dsih_hal.h
 *
 *  SG DWC PT02
 */
#ifndef MIPI_DSIH_HAL_H_
#define MIPI_DSIH_HAL_H_

#include "../include/jz_dsim.h"

/**
 * @return the dsi host core version
 */
unsigned int mipi_dsih_hal_get_version(struct dsi_device *dsi);
/*enable dsihost*/
void mipi_dsih_hal_power(struct dsi_device *dsi, int on);
int mipi_dsih_hal_get_power(struct dsi_device *dsi);
/**
 * set the virtual channel ID that is indexes to the DPI video mode stream
 * @param vc virtual channel ID
 */
void mipi_dsih_hal_dpi_video_vc(struct dsi_device *dsi, unsigned char vc);
/**
 * @return vc virtual channel ID
 */
unsigned char mipi_dsih_hal_dpi_get_video_vc(struct dsi_device *dsi);

dsih_error_t mipi_dsih_hal_dpi_color_coding(struct dsi_device *dsi, dsih_color_coding_t color_coding);
dsih_color_coding_t mipi_dsih_hal_dpi_get_color_coding(struct dsi_device *dsi);
unsigned char mipi_dsih_hal_dpi_get_color_depth(struct dsi_device *dsi);
unsigned char mipi_dsih_hal_dpi_get_color_config(struct dsi_device *dsi);
void mipi_dsih_hal_dpi_18_loosely_packet_en(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_dpi_color_mode_pol(struct dsi_device *dsi, int active_low);
void mipi_dsih_hal_dpi_shut_down_pol(struct dsi_device *dsi, int active_low);
void mipi_dsih_hal_dpi_hsync_pol(struct dsi_device *dsi, int active_low);
void mipi_dsih_hal_dpi_vsync_pol(struct dsi_device *dsi, int active_low);
void mipi_dsih_hal_dpi_dataen_pol(struct dsi_device *dsi, int active_low);
void mipi_dsih_hal_dpi_lp_cmd_en(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_dpi_frame_ack_en(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_dpi_null_packet_en(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_dpi_multi_packet_en(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_dpi_lp_during_hfp(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_dpi_lp_during_hbp(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_dpi_lp_during_vactive(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_dpi_lp_during_vfp(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_dpi_lp_during_vbp(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_dpi_lp_during_vsync(struct dsi_device *dsi, int enable);

dsih_error_t mipi_dsih_hal_dpi_video_mode_type(struct dsi_device *dsi, dsih_video_mode_t type);
void mipi_dsih_hal_dpi_video_mode_en(struct dsi_device *dsi, int enable);
int mipi_dsih_hal_dpi_is_video_mode(struct dsi_device *dsi);
dsih_error_t mipi_dsih_hal_dpi_null_packet_size(struct dsi_device *dsi, unsigned short size);
dsih_error_t mipi_dsih_hal_dpi_chunks_no(struct dsi_device *dsi, unsigned short no);
dsih_error_t mipi_dsih_hal_dpi_video_packet_size(struct dsi_device *dsi, unsigned short size);

void mipi_dsih_hal_tear_effect_ack_en(struct dsi_device *dsi, int enable);

void mipi_dsih_hal_cmd_ack_en(struct dsi_device *dsi, int enable);
dsih_error_t mipi_dsih_hal_dcs_wr_tx_type(struct dsi_device *dsi, unsigned no_of_param, int lp);
dsih_error_t mipi_dsih_hal_dcs_rd_tx_type(struct dsi_device *dsi, unsigned no_of_param, int lp);
dsih_error_t mipi_dsih_hal_gen_wr_tx_type(struct dsi_device *dsi, unsigned no_of_param, int lp);
dsih_error_t mipi_dsih_hal_gen_rd_tx_type(struct dsi_device *dsi, unsigned no_of_param, int lp);
void mipi_dsih_hal_max_rd_size_type(struct dsi_device *dsi, int lp);
void mipi_dsih_hal_gen_cmd_mode_en(struct dsi_device *dsi, int enable);
int mipi_dsih_hal_gen_is_cmd_mode(struct dsi_device *dsi);

void mipi_dsih_hal_dpi_hline(struct dsi_device *dsi, unsigned short time);
void mipi_dsih_hal_dpi_hbp(struct dsi_device *dsi, unsigned short time);
void mipi_dsih_hal_dpi_hsa(struct dsi_device *dsi, unsigned short time);
void mipi_dsih_hal_dpi_vactive(struct dsi_device *dsi, unsigned short lines);
void mipi_dsih_hal_dpi_vfp(struct dsi_device *dsi, unsigned short lines);
void mipi_dsih_hal_dpi_vbp(struct dsi_device *dsi, unsigned short lines);
void mipi_dsih_hal_dpi_vsync(struct dsi_device *dsi, unsigned short lines);
dsih_error_t mipi_dsih_hal_gen_packet_header(struct dsi_device *dsi, unsigned char vc, unsigned char packet_type, unsigned char ms_byte, unsigned char ls_byte);
/*dsih_error_t mipi_dsih_hal_gen_packet_payload(struct dsi_device * dsi, unsigned int* payload, unsigned short payload_size);*/
dsih_error_t mipi_dsih_hal_gen_packet_payload(struct dsi_device *dsi, unsigned int payload);
dsih_error_t mipi_dsih_hal_gen_read_payload(struct dsi_device *dsi, unsigned int *payload);

void mipi_dsih_hal_timeout_clock_division(struct dsi_device *dsi, unsigned char byte_clk_division_factor);
void mipi_dsih_hal_lp_rx_timeout(struct dsi_device *dsi, unsigned short count);
void mipi_dsih_hal_hs_tx_timeout(struct dsi_device *dsi, unsigned short count);

unsigned int mipi_dsih_hal_error_status_0(struct dsi_device *dsi, unsigned int mask);
unsigned int mipi_dsih_hal_error_status_1(struct dsi_device *dsi, unsigned int mask);
void mipi_dsih_hal_error_mask_0(struct dsi_device *dsi, unsigned int mask);
void mipi_dsih_hal_error_mask_1(struct dsi_device *dsi, unsigned int mask);
unsigned int mipi_dsih_hal_get_error_mask_0(struct dsi_device *dsi, unsigned int mask);
unsigned int mipi_dsih_hal_get_error_mask_1(struct dsi_device *dsi, unsigned int mask);

void mipi_dsih_hal_dbi_out_color_coding(struct dsi_device *dsi, unsigned char color_depth, unsigned char option);
void mipi_dsih_hal_dbi_in_color_coding(struct dsi_device *dsi, unsigned char color_depth, unsigned char option);
void mipi_dsih_hal_dbi_lut_size(struct dsi_device *dsi, unsigned char size);
void mipi_dsih_hal_dbi_partitioning_en(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_dbi_dcs_vc(struct dsi_device *dsi, unsigned char vc);

void mipi_dsih_hal_dbi_cmd_size(struct dsi_device *dsi, unsigned short size);
void mipi_dsih_hal_dbi_max_cmd_size(struct dsi_device *dsi, unsigned short size);
int mipi_dsih_hal_dbi_rd_cmd_busy(struct dsi_device *dsi);
int mipi_dsih_hal_dbi_read_fifo_full(struct dsi_device *dsi);
int mipi_dsih_hal_dbi_read_fifo_empty(struct dsi_device *dsi);
int mipi_dsih_hal_dbi_write_fifo_full(struct dsi_device *dsi);
int mipi_dsih_hal_dbi_write_fifo_empty(struct dsi_device *dsi);
int mipi_dsih_hal_dbi_cmd_fifo_full(struct dsi_device *dsi);
int mipi_dsih_hal_dbi_cmd_fifo_empty(struct dsi_device *dsi);

void mipi_dsih_hal_gen_rd_vc(struct dsi_device *dsi, unsigned char vc);
void mipi_dsih_hal_gen_eotp_rx_en(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_gen_eotp_tx_en(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_bta_en(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_gen_ecc_rx_en(struct dsi_device *dsi, int enable);
void mipi_dsih_hal_gen_crc_rx_en(struct dsi_device *dsi, int enable);
int mipi_dsih_hal_gen_rd_cmd_busy(struct dsi_device *dsi);
int mipi_dsih_hal_gen_read_fifo_full(struct dsi_device *dsi);
int mipi_dsih_hal_gen_read_fifo_empty(struct dsi_device *dsi);
int mipi_dsih_hal_gen_write_fifo_full(struct dsi_device *dsi);
int mipi_dsih_hal_gen_write_fifo_empty(struct dsi_device *dsi);
int mipi_dsih_hal_gen_cmd_fifo_full(struct dsi_device *dsi);
int mipi_dsih_hal_gen_cmd_fifo_empty(struct dsi_device *dsi);

/* only if DPI */
dsih_error_t mipi_dsih_phy_hs2lp_config(struct dsi_device *dsi, unsigned char no_of_byte_cycles);
dsih_error_t mipi_dsih_phy_lp2hs_config(struct dsi_device *dsi, unsigned char no_of_byte_cycles);
dsih_error_t mipi_dsih_phy_bta_time(struct dsi_device *dsi, unsigned short no_of_byte_cycles);
/* */

void mipi_dsih_write_word(struct dsi_device *dsi, unsigned int reg_address, unsigned int data);
void mipi_dsih_write_part(struct dsi_device *dsi, unsigned int reg_address, unsigned int data, unsigned char shift, unsigned char width);
unsigned int mipi_dsih_read_word(struct dsi_device *dsi, unsigned int reg_address);
unsigned int mipi_dsih_read_part(struct dsi_device *dsi, unsigned int reg_address, unsigned char shift, unsigned char width);
void mipi_dsih_dphy_enable_hs_clk(struct dsi_device *dsi, int enable);
void mipi_dsih_dphy_auto_clklane_ctrl(struct dsi_device *dsi, int enable);
void mipi_dsih_dphy_test_clock(struct dsi_device *dsi, int value);
void mipi_dsih_dphy_test_clear(struct dsi_device *dsi, int value);
void mipi_dsih_dphy_test_en(struct dsi_device *dsi, unsigned char on_falling_edge);
void mipi_dsih_dphy_test_data_in(struct dsi_device *dsi, unsigned char test_data);
void mipi_dsih_dphy_write(struct dsi_device *dsi, unsigned char address, unsigned char *data, unsigned char data_length);
void mipi_dsih_cmd_mode(struct dsi_device *dsi, int en);

void mipi_dsih_hal_gen_set_mode(struct dsi_device *dsi, int enable);

unsigned int  mipi_dsih_write_register_configuration(struct dsi_device *dsi, register_config_t *config, uint16_t config_length);
unsigned short mipi_dsih_gen_rd_packet(struct dsi_device *dsi, unsigned char vc, unsigned char data_type, unsigned char msb_byte, unsigned char lsb_byte, unsigned char bytes_to_read, unsigned char *read_buffer);
dsih_error_t mipi_dsih_gen_wr_packet(struct dsi_device *dsi, unsigned char vc, unsigned char data_type, unsigned char *params, unsigned short param_length);

int write_command(struct dsi_device *dsi, struct dsi_cmd_packet cmd_data);
int read_command(struct dsi_device *dsi, struct dsi_cmd_packet cmd_data, unsigned char *buf, int count);
#endif /* MIPI_DSI_API_H_ */
