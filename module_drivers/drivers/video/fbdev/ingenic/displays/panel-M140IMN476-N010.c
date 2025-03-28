/*
 * module_drivers/drivers/video/fbdev/ingenic/fb_stage_wip/displays/panel-M140IMN476-N010.c
 *
 * Copyright (C) 2016 Ingenic Semiconductor Inc.
 *
 * This program is free software, you can redistribute it and/or modify it
 *
 * under the terms of the GNU General Public License version 2 as published by
 *
 * the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pwm_backlight.h>
#include <linux/delay.h>
#include <linux/lcd.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>

#include "../include/ingenicfb.h"
//#define _EDP_Pattern_
#define _link_train_enable_
#define _pcr_mk_printk_
#define _htotal_stable_check_
//#define _Msa_Active_Only_
#define SCRAMBLE_MODE 0x00
#define MSA_SW_MODE 0x80
#define EDP_IDLE_PTN_OFF 0x00

struct video_timing {
	unsigned short hfp;
	unsigned short hs;
	unsigned short hbp;
	unsigned short hact;
	unsigned short htotal;
	unsigned short vfp;
	unsigned short vs;
	unsigned short vbp;
	unsigned short vact;
	unsigned short vtotal;
	unsigned int pclk_khz;
};

#define _1080P_EDP_Panel_
#ifdef _1080P_EDP_Panel_
#define PCR_PLL_PREDIV 0x40
#define PCR_M 0x17 //148.5M           //hfp, hs, hbp,hact,htotal,vfp, vs, vbp, vact,vtotal,
#define LANE_CNT 2
#define FPS    60
#define HACT   1920
#define VACT   1080
#define HFP    88
#define HBP    148
#define HS     44
#define VFP    4
#define VBP    36
#define VS     5

struct video_timing video = {88, 44, 148, 1920, 2200, 4, 5, 36, 1080, 1125, 148500};
//
// DPU TFT-> MIPI DSI -> 8911 MIPI RX -> 8911 EDP Out
//
// EDP out pixclk 148.5M
// EDP MIPI RX Byte clk 应该大于148.5M, 因为还要提供额外的时序支撑.
#endif

#ifdef _1366x768_EDP_Panel_
#define PCR_PLL_PREDIV 0x44
#define PCR_M 0x17 //74M              //hfp, hs, hbp,hact,htotal,vfp, vs, vbp, vact,vtotal,
#define LANE_CNT 1
struct video_timing video = {100, 26, 100, 1366, 1592, 10, 10, 10, 768, 798, 76225};
#define FPS    60
#define HACT   1366
#define VACT   768
#define HFP    100
#define HBP    100
#define HS     26
#define VFP    10
#define VBP    10
#define VS     10
#endif

//sfr PIF = 0xEC;
struct board_gpio {
	short gpio;
	short active_level;
};
struct panel_dev {
	/* ingenic frame buffer */
	struct device *dev;
	struct lcd_panel *panel;

	/* common lcd framework */
	struct lcd_device *lcd;
	int power;

	struct board_gpio reset;
	struct board_gpio lcd_en;

	int i2c_id;
	struct i2c_client *client;
	char *panel_name;
};

////////////////////////////LT8911 begin/////////////////////////////////////////////
//#include "lt8911.h"
/******************* MIPI Input Config ********************/
/******************* MIPI Input Config ********************/

/******************* Edp Output Config ********************/

//#include "lt8911.c"
bool g_irq_flag = 0;
static bool edp_idle_flag = 1;
//#define DEBUG

#ifdef DEBUG
	#define debug(info, ...) printk(info, ##__VA_ARGS__)
#else
	#define debug(info, ...)
#endif

#define HTOTAL (HFP + HACT + HBP + HS)
#define VTOTAL (VFP + VACT + VBP + VS)

#define PIXCLK (HTOTAL * VTOTAL * FPS / 1000)

//hfp, hs, hbp, hact, htotal, vfp, vs, vbp, vact, vtotal, pixclk

////////////////////////////LT8911 end/////////////////////////////////////////////

static struct panel_dev *lt8911_info;

static void panel_enable(struct lcd_panel *panel)
{
}

static void panel_disable(struct lcd_panel *panel)
{
}

static struct lcd_panel_ops panel_ops = {
	.enable  = (void *)panel_enable,
	.disable = (void *)panel_disable,
};

static struct fb_videomode panel_modes = {
	.name                   = "lt8911",
	.refresh                = FPS,
	.xres                   = HACT,
	.yres                   = VACT,
	.left_margin            = HBP,
	.right_margin           = HFP,
	.upper_margin           = VBP,
	.lower_margin           = VFP,

	.hsync_len              = HS,
	.vsync_len              = VS,
	.sync                   = FB_SYNC_HOR_HIGH_ACT & FB_SYNC_VERT_HIGH_ACT,
	.vmode                  = FB_VMODE_NONINTERLACED,
	.flag                   = 0,
};

struct jzdsi_data jzdsi_pdata = {
	.modes = &panel_modes,
	.video_config.no_of_lanes = 4,
	.video_config.virtual_channel = 0,
	.video_config.color_coding = COLOR_CODE_24BIT,
	.video_config.video_mode = VIDEO_BURST_WITH_SYNC_PULSES,
	.video_config.receive_ack_packets = 0,  /* enable receiving of ack packets */
	.video_config.is_18_loosely = 0,
	.video_config.data_en_polarity = 1,
	.video_config.byte_clock = 125000, //148500, // driver will auto calculate byte_clock.
	.video_config.byte_clock_coef = MIPI_PHY_BYTE_CLK_COEF_MUL1,

	.dsi_config.max_lanes = 4,
	.dsi_config.max_hs_to_lp_cycles = 100,
	.dsi_config.max_lp_to_hs_cycles = 40,
	.dsi_config.max_bta_cycles = 4095,
	.dsi_config.color_mode_polarity = 1,
	.dsi_config.shut_down_polarity = 1,
	.dsi_config.max_bps = 2750,
	.bpp_info = 24,
};

static struct tft_config m140imn476_cfg = {
	.pix_clk_inv = 0,
	.de_dl = 0,
	.sync_dl = 0,
	.color_even = TFT_LCD_COLOR_EVEN_RGB,
	.color_odd = TFT_LCD_COLOR_ODD_RGB,
	.mode = TFT_LCD_MODE_PARALLEL_888,
};

struct lcd_panel lcd_panel[] = {
	[0] = {
		.name = "lt8911",
		.modes = &panel_modes,
		.num_modes = 1,
		.dsi_pdata = &jzdsi_pdata,

		.bpp = 24,
		.width = 699,
		.height = 197,

		.lcd_type = LCD_TYPE_MIPI_TFT,

		.tft_config = &m140imn476_cfg,
		.dsi_pdata = &jzdsi_pdata,

		.dither_enable = 0,
		.dither.dither_red = 0,
		.dither.dither_green = 0,
		.dither.dither_blue = 0,

		.ops = &panel_ops,
	},
};

static unsigned char i2c_read(struct i2c_client *client, unsigned char addr)
{
	return i2c_smbus_read_byte_data(client, addr);
}

static unsigned int i2c_write(struct i2c_client *client, unsigned char addr, unsigned char value)
{
	return i2c_smbus_write_byte_data(client, addr, value);
}
void lt8911_reset(struct panel_dev *lt8911_info)
{
	struct board_gpio *rst = &lt8911_info->reset;

	gpio_direction_output(rst->gpio, 0);
	mdelay(10);
	gpio_direction_output(rst->gpio, 1);
	mdelay(120);
	gpio_direction_output(rst->gpio, 0);
	mdelay(100);
}

void LT8911_ChipID(struct i2c_client *client)
{
	uint8_t value[3];
	i2c_write(client, 0xff, 0x81); //register bank
	i2c_write(client, 0x08, 0x7f);
	value[0] = i2c_read(client, 0x00);
	value[1] = i2c_read(client, 0x01);
	value[2] = i2c_read(client, 0x02);
	printk("value[0] = %x, value[1] = %x, value[2] = %x\n", value[0], value[1], value[2]);
}

void LT8911_SetVideoTiming(struct i2c_client *client, struct video_timing *video_format)
{
	int pclk_khz;
	uint8_t dessc_m;
	//edp msa
	i2c_write(client, 0xff, 0xa8);
	i2c_write(client, 0x2d, 0x88); //bit[7]：1 = register msa, 0 = hardware msa
#ifdef _Msa_Active_Only_
	i2c_write(client, 0x05, 0x00);
	i2c_write(client, 0x06, 0x00); //htotal
	i2c_write(client, 0x07, 0x00);
	i2c_write(client, 0x08, 0x00); //h_start
	i2c_write(client, 0x09, 0x00);
	i2c_write(client, 0x0a, 0x00); //hsa
	i2c_write(client, 0x0b, (u8)(video_format->hact / 256));
	i2c_write(client, 0x0c, (u8)(video_format->hact % 256)); //hactive
	i2c_write(client, 0x0d, 0x00);
	i2c_write(client, 0x0e, 0x00); //vtotal
	i2c_write(client, 0x11, 0x00);
	i2c_write(client, 0x12, 0x00);
	i2c_write(client, 0x14, 0x00);
	i2c_write(client, 0x15, (u8)(video_format->vact / 256));
	i2c_write(client, 0x16, (u8)(video_format->vact % 256)); //vactive
#else
	i2c_write(client, 0x05, (u8)(video_format->htotal / 256));
	i2c_write(client, 0x06, (u8)(video_format->htotal % 256)); //htotal
	i2c_write(client, 0x07, (u8)((video_format->hs + video_format->hbp) / 256));
	i2c_write(client, 0x08, (u8)((video_format->hs + video_format->hbp) % 256)); //h_start
	i2c_write(client, 0x09, (u8)(video_format->hs / 256));
	i2c_write(client, 0x0a, (u8)(video_format->hs % 256)); //hsa
	i2c_write(client, 0x0b, (u8)(video_format->hact / 256));
	i2c_write(client, 0x0c, (u8)(video_format->hact % 256)); //hactive
	i2c_write(client, 0x0d, (u8)(video_format->vtotal / 256));
	i2c_write(client, 0x0e, (u8)(video_format->vtotal % 256)); //vtotal
	i2c_write(client, 0x11, (u8)((video_format->vs + video_format->vbp) / 256));
	i2c_write(client, 0x12, (u8)((video_format->vs + video_format->vbp) % 256)); //v_start
	i2c_write(client, 0x14, (u8)(video_format->vs % 256)); //vsa
	i2c_write(client, 0x15, (u8)(video_format->vact / 256));
	i2c_write(client, 0x16, (u8)(video_format->vact % 256)); //vactive
#endif
	// ldvd de only mode to regenerate h/v sync.
	i2c_write(client, 0xff, 0xd8);
	i2c_write(client, 0x20, (u8)(video_format->hfp / 256));
	i2c_write(client, 0x21, (u8)(video_format->hfp % 256));
	i2c_write(client, 0x22, (u8)(video_format->hs / 256));
	i2c_write(client, 0x23, (u8)(video_format->hs % 256));
	i2c_write(client, 0x24, (u8)(video_format->htotal / 256));
	i2c_write(client, 0x25, (u8)(video_format->htotal % 256));
	i2c_write(client, 0x26, (u8)(video_format->vfp % 256));
	i2c_write(client, 0x27, (u8)(video_format->vs % 256));
	//de-sscpll to gererate pixel clock for pattern
	pclk_khz = video_format->pclk_khz;
	dessc_m = (pclk_khz * 4) / (25 * 1000);
	i2c_write(client, 0xff, 0x85);
	i2c_write(client, 0xaa, dessc_m); //MK[30:24]
	//  i2c_write(0xab,0xba);   //MK[23:16]
	//  i2c_write(0xac,0xe1);   //MK[15:8]
	//
	//      i2c_write(client,0xad,0x47);  //MK[7:0]
}

void LT8911_MIPI_Video_Timing(struct i2c_client *client, struct video_timing *video_format)
{
	i2c_write(client, 0xff, 0xd0);
	i2c_write(client, 0x0d, (u8)(video_format->vtotal / 256));
	i2c_write(client, 0x0e, (u8)(video_format->vtotal % 256)); //vtotal
	i2c_write(client, 0x0f, (u8)(video_format->vact / 256));
	i2c_write(client, 0x10, (u8)(video_format->vact % 256)); //vactive
	i2c_write(client, 0x11, (u8)(video_format->htotal / 256));
	i2c_write(client, 0x12, (u8)(video_format->htotal % 256)); //htotal
	i2c_write(client, 0x13, (u8)(video_format->hact / 256));
	i2c_write(client, 0x14, (u8)(video_format->hact % 256)); //hactive
	i2c_write(client, 0x15, (u8)(video_format->vs % 256)); //vsa
	i2c_write(client, 0x16, (u8)(video_format->hs % 256)); //hsa
	i2c_write(client, 0x17, (u8)(video_format->vfp / 256));
	i2c_write(client, 0x18, (u8)(video_format->vfp % 256)); //vfp
	i2c_write(client, 0x19, (u8)(video_format->hfp / 256));
	i2c_write(client, 0x1a, (u8)(video_format->hfp % 256)); //hfp
}

void LT8911_Init(struct i2c_client *client)
{
	int i;
	/* init */
	i2c_write(client, 0xff, 0x81);
	i2c_write(client, 0x08, 0x7f); //i2c over aux issue
	i2c_write(client, 0x49, 0xff); //enable 0x87xx

	i2c_write(client, 0xff, 0x82); //GPIO test output
	i2c_write(client, 0x5a, 0x0e);

	//for power consumption//
	i2c_write(client, 0xff, 0x81);
	i2c_write(client, 0x05, 0x06);
	i2c_write(client, 0x43, 0x00);
	i2c_write(client, 0x44, 0x1f);
	i2c_write(client, 0x45, 0xf7);
	i2c_write(client, 0x46, 0xf6);
	i2c_write(client, 0x49, 0x7f);

	i2c_write(client, 0xff, 0x82);
	i2c_write(client, 0x12, 0x33);

	/* mipi Rx analog */
	i2c_write(client, 0xff, 0x82);
	i2c_write(client, 0x32, 0x51);
	i2c_write(client, 0x35, 0x62); //EQ current 0x42
	i2c_write(client, 0x3a, 0x33); //0x77:EQ 12.5db  ,0x33:EQ 6.5db
	i2c_write(client, 0x3b, 0x33); //0x77:EQ 12.5db  ,0x33:EQ 6.5db
	i2c_write(client, 0x4c, 0x0c);
	i2c_write(client, 0x4d, 0x00);

	/* dessc_pcr  pll analog */
	i2c_write(client, 0xff, 0x82);
	i2c_write(client, 0x6a, 0x43); //final setting: 0x40
	i2c_write(client, 0x6b, PCR_PLL_PREDIV); //0x44:pre-div = 2
	//i2c_write(0x6b,0x44);
	i2c_write(client, 0x6e, 0x81);

	/* dessc pll digital */
	i2c_write(client, 0xff, 0x85);
	i2c_write(client, 0xa9, 0x31);
	i2c_write(client, 0xaa, 0x17);
	i2c_write(client, 0xab, 0xba);
	i2c_write(client, 0xac, 0xe1);
	i2c_write(client, 0xad, 0x47);
	i2c_write(client, 0xae, 0x01);
	i2c_write(client, 0xae, 0x11);

	/* Digital Top */
	i2c_write(client, 0xff, 0x85);
	i2c_write(client, 0xc0, 0x01); //select mipi Rx
#ifdef _6bit_
	i2c_write(client, 0xb0, 0xd0); //enable dither
#endif

	/* mipi Rx Digital */
	i2c_write(client, 0xff, 0xd0);
	//i2c_write(0x00, 0x00); //0x03: 3lanes, 0x00: 4lanes
	i2c_write(client, 0x02, 0x08);   //settle
	i2c_write(client, 0x08, 0x00);

	i2c_write(client, 0x0c, 0x80); //fifo position
	i2c_write(client, 0x1c, 0x80); //fifo position
	i2c_write(client, 0x24, 0x70); //pcr mode( de hs vs)

	//i2c_write(0x2d, 0x1f); //M up limit
	i2c_write(client, 0x31, 0x0a);  //M down limit

	/*stage1 hs mode*/
	i2c_write(client, 0x25, 0x90); //line limit
	i2c_write(client, 0x2a, 0x3a); //step in limit
	i2c_write(client, 0x21, 0x4f); //hs_step
	i2c_write(client, 0x22, 0xff);

	/*stage2 de mode*/
	i2c_write(client, 0x0a, 0x02); //de adjust pre line
	i2c_write(client, 0x38, 0x02); //de_threshold 1
	i2c_write(client, 0x39, 0x04); //de_threshold 2
	i2c_write(client, 0x3a, 0x08); //de_threshold 3
	i2c_write(client, 0x3b, 0x10); //de_threshold 4

	i2c_write(client, 0x3f, 0x02); //de_step 1
	i2c_write(client, 0x40, 0x04); //de_step 2
	i2c_write(client, 0x41, 0x08); //de_step 3
	i2c_write(client, 0x42, 0x10); //de_step 4

	/*stage2 hs mode*/
	i2c_write(client, 0x1e, 0x01); // hs threshold
	i2c_write(client, 0x23, 0xf0); // hs step

	i2c_write(client, 0x2b, 0x80); //stable out // V1.8 20200417

#ifdef _EDP_Pattern_
	i2c_write(client, 0x26, (PCR_M | 0x80));
#else
	i2c_write(client, 0x26, PCR_M);
#endif

	//  #ifndef _1080P_eDP_Panel_
	//  LT8911_MIPI_Video_Timing(&video); //defualt setting is 1080P
	//  #endif

	LT8911_MIPI_Video_Timing(client, &video); //defualt setting is 1080P

	i2c_write(client, 0xff, 0x81); //PCR reset
	i2c_write(client, 0x03, 0x7b);
	i2c_write(client, 0x03, 0xff);

	/* Txpll 2.7G*/
	i2c_write(client, 0xff, 0x87);
	i2c_write(client, 0x19, 0x31);
	i2c_write(client, 0x1a, 0x1b);
	i2c_write(client, 0xff, 0x82);
	i2c_write(client, 0x02, 0x42);
	i2c_write(client, 0x03, 0x00);
	i2c_write(client, 0x03, 0x01);
	i2c_write(client, 0x0a, 0x1b);
	i2c_write(client, 0x04, 0x2a);
	i2c_write(client, 0xff, 0x81);
	i2c_write(client, 0x09, 0xfc);
	i2c_write(client, 0x09, 0xfd);
	i2c_write(client, 0xff, 0x87);
	i2c_write(client, 0x0c, 0x11);

	for (i = 0; i < 5; i++) { //Check Tx PLL
		mdelay(5);
		if (i2c_read(client, 0x37) & 0x02) {
			printk("\r\nLT8911 tx pll locked");
			i2c_write(client, 0xff, 0x87);
			i2c_write(client, 0x1a, 0x36);
			i2c_write(client, 0xff, 0x82);
			i2c_write(client, 0x0a, 0x36);
			i2c_write(client, 0x04, 0x3a);
			break;
		} else {
			printk("\r\nLT8911 tx pll unlocked");
			i2c_write(client, 0xff, 0x81);
			i2c_write(client, 0x09, 0xfc);
			i2c_write(client, 0x09, 0xfd);
			i2c_write(client, 0xff, 0x87);
			i2c_write(client, 0x0c, 0x10);
			i2c_write(client, 0x0c, 0x11);
		}
	}

	/* tx phy */
	i2c_write(client, 0xff, 0x82);
	i2c_write(client, 0x11, 0x00);
	i2c_write(client, 0x13, 0x10);
	i2c_write(client, 0x14, 0x0c);
	i2c_write(client, 0x14, 0x08);
	i2c_write(client, 0x13, 0x20);
	i2c_write(client, 0xff, 0x82);
	i2c_write(client, 0x0e, 0x25);
	//i2c_write(0x12,0xff);
	i2c_write(client, 0xff, 0x80);
	i2c_write(client, 0x40, 0x22);

	/*eDP Tx Digital */
	i2c_write(client, 0xff, 0xa8);
#ifdef _EDP_Pattern_
	i2c_write(client, 0x24, 0x52);
	i2c_write(client, 0x27, 0x50); //0x50:Pattern; 0x10:mipi video
#else
	i2c_write(client, 0x27, 0x10); //0x50:Pattern; 0x10:mipi video
	//i2c_write(0x27,0x50); //0x50:Pattern; 0x10:mipi video
#endif

#ifdef _6bit_
	i2c_write(client, 0x17, 0x00);
	i2c_write(client, 0x18, 0x00);
#endif

	//  #ifndef _1080P_eDP_Panel_
	//  LT8911_eDP_Video_Timing(&video);
	//  #endif

	i2c_write(client, 0xff, 0xa0); //nvid = 0x080000;
	i2c_write(client, 0x00, 0x00);
	i2c_write(client, 0x01, 0x80);
}
void LT8911_InterruptEnable(struct i2c_client *client)
{
	i2c_write(client, 0xff, 0x85);
	i2c_write(client, 0x08, 0x3f); //fm clr

	i2c_write(client, 0x65, 0x7f); //vid chk hact clr
	i2c_write(client, 0x07, 0x7f); //vid chk clr
}

u8 DpcdRead(struct i2c_client *client, u32 Address)
{
	u8 DpcdValue = 0x00;
	u8 AddressH = 0x0f & (Address >> 16);
	u8 AddressM = 0xff & (Address >> 8);
	u8 AddressL = 0xff &  Address    ;
	char value;
	i2c_write(client, 0xff, 0xac);
	i2c_write(client, 0x00, 0x00); //Soft Link train
	i2c_write(client, 0xff, 0xa6);
	i2c_write(client, 0x2a, 0x01);

	i2c_write(client, 0xff, 0xa6);
	i2c_write(client, 0x2a, 0x01);
	i2c_write(client, 0x2b, (0x90 | AddressH)); //CMD
	i2c_write(client, 0x2b, AddressM); //addr[15:8]
	i2c_write(client, 0x2b, AddressL); //addr[7:0]
	i2c_write(client, 0x2b, 0x00); //data lenth
	i2c_write(client, 0x2c, 0x00); //start Aux read edid

	mdelay(10); //more than 10ms
	if ((i2c_read(client, 0x25) & 0x0f) == 0x0c) {
		value = i2c_read(client, 0x39);
		if (value == 0x22) {
			DpcdValue = i2c_read(client, 0x2b);
		} else {
			goto no_reply;
		}
	} else if ((value & 0x0f) == 0x0a) {
		goto reply_nack;
	} else if ((value & 0x0f) == 0x09) {
		goto reply_defer;
	} else {
		goto no_reply;
	}

	printk("\r\nDpcdRead: %x%x%x= %x ", AddressH, AddressM, AddressL, DpcdValue);
	return DpcdValue;
	/*error handle*/
no_reply:
	printk("\r\nDpcdRead error: no_reply ");
	return 0;
reply_nack:
	printk("\r\nDpcdRead error: reply_nack ");
	return 0;
reply_defer:
	printk("\r\nDpcdRead error: reply_nack ");
	return 0;
}

bool DpcdWrite(struct i2c_client *client, u32 Address, u8 value)
{
	u8 AddressH = 0x0f & (Address >> 16);
	u8 AddressM = 0xff & (Address >> 8);
	u8 AddressL = 0xff &  Address    ;
	u8 reg;
	printk("\r\nDpcdWrite: %#x,%#x,%#x= %#x ", AddressH, AddressM, AddressL, value);

	i2c_write(client, 0xff, 0xac);
	i2c_write(client, 0x00, 0x00); //Soft Link train
	i2c_write(client, 0xff, 0xa6);
	i2c_write(client, 0x2a, 0x01);

	i2c_write(client, 0xff, 0xa6);
	i2c_write(client, 0x2a, 0x01);
	i2c_write(client, 0x2b, (0x80 | AddressH)); //CMD
	i2c_write(client, 0x2b, AddressM); //addr[15:8]
	i2c_write(client, 0x2b, AddressL); //addr[7:0]
	i2c_write(client, 0x2b, 0x00); //data lenth
	i2c_write(client, 0x2b, value); //data lenth
	i2c_write(client, 0x2c, 0x00); //start Aux read edid

	mdelay(10); //more than 10ms
	if ((i2c_read(client, 0x25) & 0x0f) == 0x0c) {
		return 0;
	} else if ((reg & 0x0f) == 0x0a) {
		goto reply_nack;
	} else if ((reg & 0x0f) == 0x09) {
		goto reply_defer;
	} else {
		goto no_reply;
	}

	/*error handle*/
no_reply:
	printk("\r\nDpcdRead error: no_reply ");
	return 1;
reply_nack:
	printk("\r\nDpcdRead error: reply_nack ");
	return 1;
reply_defer:
	printk("\r\nDpcdRead error: reply_nack ");
	return 1;
}

void LT8911_LinkTrain(struct i2c_client *client)
{
	if (SCRAMBLE_MODE == 0x80) {
		DpcdWrite(client, 0x10a, 0x01);
	}

	i2c_write(client, 0xff, 0xa6);
	i2c_write(client, 0x2a, 0x00);

	i2c_write(client, 0xff, 0x81);
	i2c_write(client, 0x07, 0xfe);
	i2c_write(client, 0x07, 0xff);
	i2c_write(client, 0x0a, 0xfc);
	i2c_write(client, 0x0a, 0xfe);

	/* link train */
	i2c_write(client, 0xff, 0xa8);
	i2c_write(client, 0x2d, MSA_SW_MODE | EDP_IDLE_PTN_OFF); //edp output video ;

	i2c_write(client, 0xff, 0x85);
	i2c_write(client, 0x17, 0xc0);
	i2c_write(client, 0x1a, LANE_CNT);
	i2c_write(client, 0xa1, (SCRAMBLE_MODE | 0x03)); //scramble mode
	//i2c_write(0x13,0xd1);

#ifdef _link_train_enable_
	i2c_write(client, 0xff, 0xac);
	i2c_write(client, 0x00, 0x60);
	i2c_write(client, 0x01, 0x0a);
	i2c_write(client, 0x0c, 0x05);
	i2c_write(client, 0x0c, 0x45);
	printk("\r\n\33[35mLT8911 link trian: hardware linktrain start...\033[37m");
	//      mdelay(500);
#else
	i2c_write(client, 0xff, 0xac);
	i2c_write(client, 0x00, 0x00);
	i2c_write(client, 0x01, 0x0a);
	i2c_write(client, 0x14, 0x80);
	i2c_write(client, 0x14, 0x81);
	mdelay(50);
	i2c_write(client, 0x14, 0x84);
	mdelay(50);
	i2c_write(client, 0x14, 0xc0);
	printk("\r\n\33[35mLT8911 link trian: no handshake linktrain\033[37m");
#endif

	edp_idle_flag = 0;
}

void LT8911_LinkTrainResultCheck(struct i2c_client *client)
{
#ifdef _link_train_enable_
	u8 i;
	u8 val;
	//int ret;

	i2c_write(client, 0xff, 0xac);
	for (i = 0; i < 10; i++) {
		if (i2c_read(client, 0x82) & 0x20) {
			val = i2c_read(client, 0x82);
			if ((val & 0x1f) == 0x1e) {
				printk("\r\nLT8911_LinkTrainResultCheck: edp link train successed: %#x", val);
			} else {
				printk("\r\nLT8911_LinkTrainResultCheck: edp link train failed: %#x", val);
				//while(1);
				i2c_write(client, 0xff, 0xac);
				i2c_write(client, 0x00, 0x00);
				i2c_write(client, 0x01, 0x0a);
				i2c_write(client, 0x14, 0x80);
				i2c_write(client, 0x14, 0x81);
				mdelay(50);
				i2c_write(client, 0x14, 0x84);
				mdelay(50);
				i2c_write(client, 0x14, 0xc0);
				printk("\r\nLT8911_LinkTrainResultCheck: Enable eDP video output while linktrian fail");
				//while(1);
			}

			val = i2c_read(client, 0x83);
			printk("\r\nLT8911_LinkTrainResultCheck: panel link rate: %#x", val);
			val = i2c_read(client, 0x84);
			printk("\r\nLT8911_LinkTrainResultCheck: panel link count: %#x", val);
			return;
		} else {
			printk("\r\nLT8911_LinkTrainResultCheck: link trian on going...");
		}
		mdelay(100);
	}
#endif
}

/* mipi should be ready before configuring below video check setting*/
void LT8911_video_check(struct i2c_client *client)
{
	//u8 temp;
	int reg;
	/* mipi byte clk check*/
	i2c_write(client, 0xff, 0x85);
	i2c_write(client, 0x1d, 0x00); //FM select byte clk
	i2c_write(client, 0x40, 0xf7);
	i2c_write(client, 0x41, 0x30);
	i2c_write(client, 0xa1, 0x02); //video chech from mipi

	i2c_write(client, 0xff, 0x81); //video check rst
	i2c_write(client, 0x09, 0x7d);
	i2c_write(client, 0x09, 0xfd);

	i2c_write(client, 0xff, 0x85);
	mdelay(30);
	if (i2c_read(client, 0x50) == 0x03) {
		reg = i2c_read(client, 0x4d);
		reg = reg * 256 + i2c_read(client, 0x4e);
		reg = reg * 256 + i2c_read(client, 0x4f);

		printk("\r\nvideo check: mipi clk = %d", reg);
	} else {
		printk("\r\nvideo check: mipi clk unstable");
	}

	/* mipi vtotal check*/
	reg = i2c_read(client, 0x76);
	reg = reg * 256 + i2c_read(client, 0x77);

	printk("\r\nvideo check: Vtotal = %d", reg);

	/* mipi word count check*/
	i2c_write(client, 0xff, 0xd0);
	reg = i2c_read(client, 0x82);
	reg = reg * 256 + i2c_read(client, 0x83);
	reg = reg / 3;
	printk("\r\nvideo check: Hact(word counter) = %d", reg);
	//printdec_u32(reg);

	/* mipi Vact check*/
	reg = i2c_read(client, 0x85);
	reg = reg * 256 + i2c_read(client, 0x86);

	printk("\r\nvideo check: Vact = %d", reg);
	//printdec_u32(reg);
	printk("\r\nlane0 settle: %#x ", i2c_read(client, 0x88));
	printk("\r\nlane1 settle: %#x ", i2c_read(client, 0x8a));
	printk("\r\nlane2 settle: %#x ", i2c_read(client, 0x8c));
	printk("\r\nlane3 settle: %#x ", i2c_read(client, 0x8e));

	printk("\r\nlane0 sot: %#x ", i2c_read(client, 0x89));
	printk("\r\nlane1 sot: %#x ", i2c_read(client, 0x8b));
	printk("\r\nlane2 sot: %#x ", i2c_read(client, 0x8d));
	printk("\r\nlane3 sot: %#x ", i2c_read(client, 0x8f));
	printk("\r\n----------------------------------------------------------------------");
}

void LT8911_MainLoop(struct i2c_client *client, struct video_timing *video_format)
{
	u16 reg;
	u16 vtotal;
	static int flag_mipi_on = 1;

	vtotal = video_format->vtotal;
	i2c_write(client, 0xff, 0x85);
	//i2c_write(0x1d,0x00); //FM select byte clk
	//i2c_write(0x40,0xf7);
	//i2c_write(0x41,0x30);
	i2c_write(client, 0xa1, 0x02); //video chech from mipi

	i2c_write(client, 0xff, 0x81); //video check rst
	i2c_write(client, 0x09, 0x7d);
	i2c_write(client, 0x09, 0xfd);
	mdelay(50);

	i2c_write(client, 0xff, 0x85);
	reg = i2c_read(client, 0x76);
	reg = reg * 256 + i2c_read(client, 0x77);

	printk("\r\nPCR reset: %d", reg);
	printk("\r\nvtotal: %d", vtotal);
	if ((reg <= (vtotal + 3)) && (reg >= (vtotal - 3))) {
		if (!flag_mipi_on) {
			mdelay(1000);
			i2c_write(client, 0xff, 0x81); //PCR reset
			i2c_write(client, 0x03, 0x7b);
			i2c_write(client, 0x03, 0xff);
			mdelay(100);
			i2c_write(client, 0xff, 0xa8);
			i2c_write(client, 0x2d, 0x88); //edp disable idle pattern;
			flag_mipi_on = 1;
			printk("\r\nPCR reset0");
		}

		i2c_write(client, 0xff, 0xd0);
		if ((i2c_read(client, 0x84) & 0x40) == 0x00) {
			i2c_write(client, 0xff, 0x81); //PCR reset
			i2c_write(client, 0x03, 0x7b);
			i2c_write(client, 0x03, 0xff);
			mdelay(500);
			printk("\r\nPCR reset1");
		} else {

			//              printk("\r\npcr stable:%bx, mk: %bx, %bx, %bx, %bx",
			//              i2c_read(0x84),
			//              i2c_read(0x94),
			//              i2c_read(0x95),
			//              i2c_read(0x96),
			//              i2c_read(0x97));

			//              if((i2c_read(0x94)==0x17)&&((i2c_read(0x95)==0x53)||(i2c_read(0x95)==0x47)))
			//              {
			//                  return;
			//              }
			//              else
			//              {
			//                  i2c_write(0xff,0x81); //PCR reset
			//                  i2c_write(0x03,0x7b);
			//                  i2c_write(0x03,0xff);
			//                  mdelay(500);
			//                  printk("\r\nPCR reset2");
			//              }
		}
	} else {
		i2c_write(client, 0xff, 0xa8);
		i2c_write(client, 0x2d, 0x8c); //edp enable idle pattern;
		flag_mipi_on = 0;
	}
}

void LT8911_pcr_mk_print(struct i2c_client *client)
{
#ifdef _pcr_mk_printk_
	u8 loopx = 0;

	for (loopx = 0; loopx < 30; loopx++) {
		i2c_write(client, 0xff, 0xd0);
		printk("\r\npcr stable bit[4]:%x, mk: %x, %x, %x, %x",
		       i2c_read(client, 0x84),
		       i2c_read(client, 0x94),
		       i2c_read(client, 0x95),
		       i2c_read(client, 0x96),
		       i2c_read(client, 0x97));
		mdelay(500);
	}
#endif
}

void LT8911_htotal_stable_print(struct i2c_client *client)
{
#ifdef _htotal_stable_check_
	u8 loopx = 0;
	u32 val;

	for (loopx = 0; loopx < 30; loopx++) {
		i2c_write(client, 0xff, 0x85);
		val = i2c_read(client, 0x90);
		val = val * 256 + i2c_read(client, 0x91);
		printk("\r\nmipi htotal: %d\n",  val);
		mdelay(10);
	}
#endif
}

void LT8911_LinkTrainRresultDebug(struct i2c_client *client)
{
	u8 i;
	u8 val;

	i2c_write(client, 0xff, 0xac);
	for (i = 0; i < 10; i++) {
		val = i2c_read(client, 0x82);
		if (val & 0x20) {
			if ((val & 0x1f) == 0x1e) {
				//val = i2c_read(0x82);
				printk("\r\nedp link train successed: %x", val);
			} else {
				//val = i2c_read(0x82);
				printk("\r\nedp link train failed: %x", val);
			}
			val = i2c_read(client, 0x83);
			printk("\r\npanel link rate: %x", val);
			val = i2c_read(client, 0x84);
			printk("\r\npanel link count: %x", val);
			return;
		} else {
			printk("\r\nlink trian on going...");
		}
		mdelay(100);
	}
}

void IntbInterruptFun(struct i2c_client *client)
{
	g_irq_flag = 1;
}

void InterruptTaskHandle(struct i2c_client *client)
{
	if (g_irq_flag) { // interrupt event happened
		printk("\r\nirq task happened...");

		i2c_write(client, 0xff, 0xa8);
		i2c_write(client, 0x2d, 0x84); //edp output idle pattern;

		mdelay(1500);
		i2c_write(client, 0xff, 0x85); //clr LT
		i2c_write(client, 0x08, 0xc0);
		i2c_write(client, 0x08, 0x00);

		i2c_write(client, 0x65, 0xff); //vid chk hact clr
		i2c_write(client, 0x65, 0x7f); //vid chk hact clr

		i2c_write(client, 0x07, 0xff); //vid chk clr
		i2c_write(client, 0x07, 0x7f); //vid chk clr

		edp_idle_flag = 1;
		//      if(PIF == 0x08)  //clear MCU interrupt flag to handle next interrupt request.
		//      {
		//          PIF = 0x00;
		//          clr_IE0;
		//      }
		g_irq_flag = 0;
	}
}

int LT8911_Config(struct i2c_client *client)
{
	LT8911_ChipID(client);
	LT8911_SetVideoTiming(client, &video);
	LT8911_Init(client);
	LT8911_LinkTrain(client);

	LT8911_LinkTrainResultCheck(client);
	LT8911_video_check(client);
	//  LT8911_pcr_mk_print(client);
	//  LT8911_htotal_stable_print(client);
	//  while(1)
	//  {
	//      LT8911_MainLoop(client,&video);
	//      mdelay(500);
	//  }
	return 0;
}
#define POWER_IS_ON(pwr)        ((pwr) <= FB_BLANK_NORMAL)
static int panel_set_power(struct lcd_device *lcd, int power)
{
	struct panel_dev *lt8911_info = lcd_get_data(lcd);
	struct i2c_client *client = lt8911_info->client;

	struct board_gpio *lcd_en = &lt8911_info->lcd_en;
	struct board_gpio *rst = &lt8911_info->reset;

	gpio_direction_output(rst->gpio, 0);
	gpio_direction_output(lcd_en->gpio, 0);

	mdelay(15);
	lt8911_reset(lt8911_info);
	if (LT8911_Config(client)) {
		return -1;
	}

	lt8911_info->power = power;
	return 0;
}

static int panel_get_power(struct lcd_device *lcd)
{
	struct panel_dev *lt8911_info = lcd_get_data(lcd);

	return lt8911_info->power;
}

static struct lcd_ops panel_lcd_ops = {
	.set_power = panel_set_power,
	.get_power = panel_get_power,
};

int _of_get_named_gpio_lvl(struct device *dev, short *gpio, short *lvl, char *of_name)
{
	int ret = 0;
	enum of_gpio_flags flags;

	/* devm_gpio_request_one */
	*gpio = of_get_named_gpio_flags(dev->of_node, of_name, 0, &flags);
	if (gpio_is_valid(*gpio)) {
		*lvl = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = gpio_request_one(*gpio, GPIOF_DIR_OUT, of_name);
		if (ret < 0) {
			dev_err(dev, "Failed to request reset pin!\n");
			return ret;
		}
	} else {
		dev_warn(dev, "invalid gpio: %s\n", of_name);
		return -1;
	}
	return 0;
}

static int of_panel_parse(struct device *dev)
{
	struct panel_dev *lt8911_info = dev_get_drvdata(dev);
	int ret = 0;

	if ((ret = _of_get_named_gpio_lvl(dev,
	                                  &lt8911_info->reset.gpio,
	                                  &lt8911_info->reset.active_level,
	                                  "lt8911,reset-gpio"))) {
		return ret;
	}

	if ((ret = _of_get_named_gpio_lvl(dev,
	                                  &lt8911_info->lcd_en.gpio,
	                                  &lt8911_info->lcd_en.active_level,
	                                  "lt8911,lcd_en-gpio"))) {
		goto err_request_reset;
	}

	return 0;
err_request_reset:
	if (gpio_is_valid(lt8911_info->reset.gpio)) {
		gpio_free(lt8911_info->reset.gpio);
	}
	return ret;
}

static struct mipi_dsim_lcd_driver panel_dev_dsim_ddi_driver = {
	.name = "lt8911",
	.id = -1,
};

struct mipi_dsim_lcd_device panel_dev_device = {
	.name       = "lt8911",
	.id = 0,
};

static int lt8911_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0, i;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;

	lt8911_info = kzalloc(sizeof(struct panel_dev), GFP_KERNEL);
	if (lt8911_info == NULL) {
		dev_err(dev, "Failed to alloc memory!");
		return -ENOMEM;
	}
	lt8911_info->dev = dev;
	lt8911_info->client = client;
	dev_set_drvdata(dev, lt8911_info);

	// set dev_info.
	ret = of_panel_parse(dev);
	if (ret < 0) {
		goto err_of_parse;
	}

	// register lcd device.
	lt8911_info->lcd = lcd_device_register("panel_lcd", dev, lt8911_info, &panel_lcd_ops);
	if (IS_ERR_OR_NULL(lt8911_info->lcd)) {
		dev_err(dev, "Error register lcd!\n");
		ret = -EINVAL;
		goto err_of_parse;
	}

	mipi_dsi_register_lcd_device(&panel_dev_device);
	mipi_dsi_register_lcd_driver(&panel_dev_dsim_ddi_driver);

	ret = ingenicfb_register_panel(&lcd_panel[i]);
	if (ret < 0) {
		dev_err(dev, "Failed to register lcd panel!\n");
		goto err_lcd_register;
	}

	// register ingenicfb device.
	/* TODO: should this power status sync from uboot */
	lt8911_info->power = FB_BLANK_POWERDOWN;
	if (panel_set_power(lt8911_info->lcd, FB_BLANK_UNBLANK)) {
		goto err_lcd_register;
	}

	printk("\033[32mregister %s sucess.\033[0m\n", lt8911_info->panel_name);

	return 0;

err_lcd_register:
	lcd_device_unregister(lt8911_info->lcd);
err_of_parse:
	kfree(lt8911_info);
	printk("\033[31mregister %s dailed.\033[0m\n", lt8911_info->panel_name);
	return ret;
}

static int lt8911_remove(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id lt8911_match_table[] = {
	{.compatible = "mipi2edp,lt8911",},
	{ },
};
MODULE_DEVICE_TABLE(of, lt8911_match_table);

static const struct i2c_device_id lt8911_id[] = {
	{ "lt8911", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lt8911_id);

static struct i2c_driver lt8911_driver = {
	.probe      = lt8911_probe,
	.remove     = lt8911_remove,
	.driver = {
		.name     = "lt8911",
		.owner    = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(lt8911_match_table),
#endif
	},
	.id_table = lt8911_id,
};

static int __init lt8911_init(void)
{
	int ret;
	printk("\033[33m%s\033[0m\n", "lt8911 driver installing...");
	ret = i2c_add_driver(&lt8911_driver);
	if (ret != 0) {
		printk("lt8911 driver init failed!\n");
	}

	return ret;
}

static void __exit lt8911_exit(void)
{
	printk("lt8911 driver exited.\n");
	i2c_del_driver(&lt8911_driver);
}

/* module_i2c_driver(lt8911_driver); */
module_init(lt8911_init);
module_exit(lt8911_exit);

MODULE_DESCRIPTION("LT8911 Driver");
MODULE_LICENSE("GPL");
