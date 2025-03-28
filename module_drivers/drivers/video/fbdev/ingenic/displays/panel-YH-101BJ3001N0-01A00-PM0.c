/*
 * Copyright (C) 2016 Ingenic Semiconductor Inc.
 * This program is free software, you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
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
#include <linux/fb.h>
#include <linux/backlight.h>

#include "../include/ingenicfb.h"
#include "../include/jz_dsim.h"

static char panel_yh101_debug = 1;
#define PANEL_DEBUG_MSG(msg...)         \
	do {                    \
		if (panel_yh101_debug)      \
			printk(">>>>>>>>>>>>>>>>PANEL YH101: " msg);        \
	} while(0)

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
	struct backlight_device *backlight;
	int power;

	struct regulator *vcc;
	struct board_gpio rst;
	struct board_gpio lcd_pwm;

	struct mipi_dsim_lcd_device *dsim_dev;
};

struct panel_dev *panel;

#define lcd_to_master(a)    (a->dsim_dev->master)
#define lcd_to_master_ops(a)    ((lcd_to_master(a))->master_ops)

struct yh101 {
	struct device *dev;
	unsigned int power;
	unsigned int id;

	struct lcd_device *ld;
	struct backlight_device *bd;

	struct mipi_dsim_lcd_device *dsim_dev;

};

static struct dsi_cmd_packet fitipower_yh101_800_1280_4lane_cmd_list[] = {
	{0x15, 0xE0, 0x00},
	{0x15, 0xE1, 0x93},
	{0x15, 0xE2, 0x65},
	{0x15, 0xE3, 0xF8},
	{0x15, 0x80, 0x03},

	{0x15, 0xE0, 0x01},
	{0x15, 0x00, 0x00},
	{0x15, 0x01, 0x3B},

	{0x15, 0x0C, 0x74},

	{0x15, 0x17, 0x00},
	{0x15, 0x18, 0xAF},
	{0x15, 0x19, 0x00},
	{0x15, 0x1A, 0x00},
	{0x15, 0x1B, 0xAF},
	{0x15, 0x1C, 0x00},

	{0x15, 0x35, 0x26},

	{0x15, 0x37, 0x09},

	{0x15, 0x38, 0x04},
	{0x15, 0x39, 0x00},
	{0x15, 0x3A, 0x01},
	{0x15, 0x3C, 0x78},
	{0x15, 0x3D, 0xFF},
	{0x15, 0x3E, 0xFF},
	{0x15, 0x3F, 0x7F},

	{0x15, 0x40, 0x06},
	{0x15, 0x41, 0xA0},
	{0x15, 0x42, 0x81},
	{0x15, 0x43, 0x14}, //VFP=20
	{0x15, 0x44, 0x23}, //VBP=35
	{0x15, 0x45, 0x28}, //HBP=40

	{0x15, 0x55, 0x02},
	{0x15, 0x57, 0x69},
	{0x15, 0x59, 0x0A},
	{0x15, 0x5A, 0x2A},
	{0x15, 0x5B, 0x17},

	{0x15, 0x5D, 0x7F},
	{0x15, 0x5E, 0x6B},
	{0x15, 0x5F, 0x5C},
	{0x15, 0x60, 0x4F},
	{0x15, 0x61, 0x4D},
	{0x15, 0x62, 0x3F},
	{0x15, 0x63, 0x42},
	{0x15, 0x64, 0x2B},
	{0x15, 0x65, 0x44},
	{0x15, 0x66, 0x43},
	{0x15, 0x67, 0x43},
	{0x15, 0x68, 0x63},
	{0x15, 0x69, 0x52},
	{0x15, 0x6A, 0x5A},
	{0x15, 0x6B, 0x4F},
	{0x15, 0x6C, 0x4E},
	{0x15, 0x6D, 0x20},
	{0x15, 0x6E, 0x0F},
	{0x15, 0x6F, 0x00},

	{0x15, 0x70, 0x7F},
	{0x15, 0x71, 0x6B},
	{0x15, 0x72, 0x5C},
	{0x15, 0x73, 0x4F},
	{0x15, 0x74, 0x4D},
	{0x15, 0x75, 0x3F},
	{0x15, 0x76, 0x42},
	{0x15, 0x77, 0x2B},
	{0x15, 0x78, 0x44},
	{0x15, 0x79, 0x43},
	{0x15, 0x7A, 0x43},
	{0x15, 0x7B, 0x63},
	{0x15, 0x7C, 0x52},
	{0x15, 0x7D, 0x5A},
	{0x15, 0x7E, 0x4F},
	{0x15, 0x7F, 0x4E},
	{0x15, 0x80, 0x20},
	{0x15, 0x81, 0x0F},
	{0x15, 0x82, 0x00},

	{0x15, 0xE0, 0x02},
	{0x15, 0x00, 0x02}, //STV3   ->  STV2
	{0x15, 0x01, 0x02}, //Stv3   ->  STV2
	{0x15, 0x02, 0x00}, //STV4   ->  STV0
	{0x15, 0x03, 0x00}, //STV4   ->  STV0
	{0x15, 0x04, 0x1E}, //VDS    ->  VGH
	{0x15, 0x05, 0x1E}, //VDS    ->  VGH
	{0x15, 0x06, 0x1F}, //VSD    ->  VGL
	{0x15, 0x07, 0x1F}, //VSD    ->  VGL
	{0x15, 0x08, 0x1F},
	{0x15, 0x09, 0x17}, //VDD2   ->  FLM
	{0x15, 0x0A, 0x17}, //VDD2   ->  FLM
	{0x15, 0x0B, 0x37}, //VDD1   ->  INV_FLM
	{0x15, 0x0C, 0x37}, //VDD1   ->  INV_FLM
	{0x15, 0x0D, 0x47}, //CLK8   ->  CLK3
	{0x15, 0x0E, 0x47}, //CLK8   ->  CLK3
	{0x15, 0x0F, 0x45}, //CLK6   ->  CLK1
	{0x15, 0x10, 0x45}, //CLK6   ->  CLK1
	{0x15, 0x11, 0x4B}, //CLK4   ->  CLK7
	{0x15, 0x12, 0x4B}, //CLK4   ->  CLK7
	{0x15, 0x13, 0x49}, //CLK2   ->  CLK5
	{0x15, 0x14, 0x49}, //CLK2   ->  CLK5
	{0x15, 0x15, 0x1F}, //VGL

	{0x15, 0x16, 0x01}, //STV1   ->  STV1
	{0x15, 0x17, 0x01}, //STV1   ->  STV1
	{0x15, 0x18, 0x00}, //STV2   ->  STV0
	{0x15, 0x19, 0x00}, //STV2   ->  STV0
	{0x15, 0x1A, 0x1E}, //VDS    ->  VGH
	{0x15, 0x1B, 0x1E}, //VDS    ->  VGH
	{0x15, 0x1C, 0x1F}, //VSD    ->  VGL
	{0x15, 0x1D, 0x1F}, //VSD    ->  VGL
	{0x15, 0x1E, 0x1F},
	{0x15, 0x1F, 0x17}, //VDD2   ->  FLM
	{0x15, 0x20, 0x17}, //VDD2   ->  FLM
	{0x15, 0x21, 0x37}, //VDD1   ->  INV_FLM
	{0x15, 0x22, 0x37}, //VDD1   ->  INV_FLM
	{0x15, 0x23, 0x46}, //CLK7   ->  CLK2
	{0x15, 0x24, 0x46}, //CLK7   ->  CLK2
	{0x15, 0x25, 0x44}, //CLK5   ->  CLK0
	{0x15, 0x26, 0x44}, //CLK5   ->  CLK0
	{0x15, 0x27, 0x4A}, //CLK3   ->  CLK6
	{0x15, 0x28, 0x4A}, //CLK3   ->  CLK6
	{0x15, 0x29, 0x48}, //CLK1   ->  CLK4
	{0x15, 0x2A, 0x48}, //CLK1   ->  CLK4
	{0x15, 0x2B, 0x1F}, //VGL

	{0x15, 0x2C, 0x01}, //STV3   ->  STV1
	{0x15, 0x2D, 0x01},
	{0x15, 0x2E, 0x00}, //STV4   ->  STV0
	{0x15, 0x2F, 0x00},
	{0x15, 0x30, 0x1F}, //VDS    ->  VGL
	{0x15, 0x31, 0x1F},
	{0x15, 0x32, 0x1E}, //VSD    ->  VGH
	{0x15, 0x33, 0x1E},
	{0x15, 0x34, 0x1F}, //
	{0x15, 0x35, 0x17}, //VDD2   ->  FLM
	{0x15, 0x36, 0x17},
	{0x15, 0x37, 0x37}, //VDD1   ->  INV_FLM
	{0x15, 0x38, 0x37},
	{0x15, 0x39, 0x08}, //CLK8   ->  CLK4
	{0x15, 0x3A, 0x08},
	{0x15, 0x3B, 0x0A}, //CLK6   ->  CLK6
	{0x15, 0x3C, 0x0A},
	{0x15, 0x3D, 0x04}, //CLK4   ->  CLK0
	{0x15, 0x3E, 0x04},
	{0x15, 0x3F, 0x06}, //CLK2   ->  CLK2
	{0x15, 0x40, 0x06},
	{0x15, 0x41, 0x1F}, //VGL

	{0x15, 0x42, 0x02}, //STV1   ->  STV2
	{0x15, 0x43, 0x02},
	{0x15, 0x44, 0x00}, //STV2   ->  STV0
	{0x15, 0x45, 0x00},
	{0x15, 0x46, 0x1F}, //VDS    ->  VGL
	{0x15, 0x47, 0x1F},
	{0x15, 0x48, 0x1E}, //VSD    ->  VGH
	{0x15, 0x49, 0x1E},
	{0x15, 0x4A, 0x1F}, //
	{0x15, 0x4B, 0x17}, //VDD2   ->  FLM
	{0x15, 0x4C, 0x17},
	{0x15, 0x4D, 0x37}, //VDD1   ->  INV_FLM
	{0x15, 0x4E, 0x37},
	{0x15, 0x4F, 0x09}, //CLK7   ->  CLK5
	{0x15, 0x50, 0x09},
	{0x15, 0x51, 0x0B}, //CLK5   ->  CLK7
	{0x15, 0x52, 0x0B},
	{0x15, 0x53, 0x05}, //CLK3   ->  CLK1
	{0x15, 0x54, 0x05},
	{0x15, 0x55, 0x07}, //CLK1   ->  CLK3
	{0x15, 0x56, 0x07},
	{0x15, 0x57, 0x1F}, //VGL

	{0x15, 0x58, 0x40},
	{0x15, 0x5B, 0x30}, //STV_NUM,STV_S0
	{0x15, 0x5C, 0x16}, //STV_S0
	{0x15, 0x5D, 0x34}, //STV_W / S1
	{0x15, 0x5E, 0x05}, //STV_S2
	{0x15, 0x5F, 0x02}, //STV_S3
	{0x15, 0x63, 0x00}, //SETV_ON
	{0x15, 0x64, 0x6A}, //SETV_OFF
	{0x15, 0x67, 0x73},
	{0x15, 0x68, 0x1D}, //CKV_S0
	{0x15, 0x69, 0x08},
	{0x15, 0x6A, 0x6A},
	{0x15, 0x6B, 0x08}, //Dummy clk

	{0x15, 0x6C, 0x00},
	{0x15, 0x6D, 0x00},
	{0x15, 0x6E, 0x00},
	{0x15, 0x6F, 0x88},

	{0x15, 0x75, 0xFF},
	{0x15, 0x77, 0xDD}, //VEN_EN=1
	{0x15, 0x78, 0x3F},
	{0x15, 0x79, 0x15}, //0x0C
	{0x15, 0x7A, 0x17}, //VEN_S0
	{0x15, 0x7D, 0x14}, //VEN_ON
	{0x15, 0x7E, 0x82}, //VEN_OFF

	{0x15, 0xE0, 0x04},
	{0x15, 0x00, 0x0E},
	{0x15, 0x02, 0xB3},
	{0x15, 0x09, 0x61},
	{0x15, 0x0E, 0x48},

	{0x15, 0xE0, 0x00},

	{0x15, 0xE6, 0x02},
	{0x15, 0xE7, 0x0C},
};

static void panel_dev_sleep_in(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_cmd_packet data_to_send = {0x05, 0x10, 0x00};
	PANEL_DEBUG_MSG("enter %s %d \n", __func__, __LINE__);

	ops->cmd_write(lcd_to_master(lcd), data_to_send);
}

static void panel_dev_sleep_out(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_cmd_packet data_to_send = {0x05, 0x11, 0x00};
	PANEL_DEBUG_MSG("enter %s %d \n", __func__, __LINE__);

	ops->cmd_write(lcd_to_master(lcd), data_to_send);
}

static void panel_dev_display_on(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_cmd_packet data_to_send = {0x05, 0x29, 0x00};

	ops->cmd_write(lcd_to_master(lcd), data_to_send);
}

static void panel_dev_display_off(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_cmd_packet data_to_send = {0x05, 0x28, 0x00};

	ops->cmd_write(lcd_to_master(lcd), data_to_send);
}

static void panel_dev_power_on(struct mipi_dsim_lcd_device *dsim_dev, int power)
{
	struct board_gpio *rst = &panel->rst;
	struct board_gpio *pwm = &panel->lcd_pwm;

	msleep(50);
	gpio_direction_output(rst->gpio, 1);//1
	msleep(5);
	gpio_direction_output(rst->gpio, 0);//0
	msleep(10);
	gpio_direction_output(rst->gpio, 1);//1
	msleep(120);

	panel->power = power;
}

#if 0
static int panel_dev_read_id(struct mipi_dsim_lcd_device *dsim_dev)
{
	unsigned char buf[3] = {1};
	struct dsi_master_ops *ops = dsim_dev->master->master_ops;

	struct dsi_cmd_packet data_to_send0 = {0x14, 0xDA, 0x0};
	struct dsi_cmd_packet data_to_send1 = {0x14, 0xDB, 0x0};
	struct dsi_cmd_packet data_to_send2 = {0x14, 0xDC, 0x0};
	int ret = 0;
	int panel_id = 0;
	ret = ops->cmd_read(dsim_dev->master, data_to_send0, &buf[0], 1);
	ret = ops->cmd_read(dsim_dev->master, data_to_send1, &buf[1], 1);
	ret = ops->cmd_read(dsim_dev->master, data_to_send2, &buf[2], 1);
	panel_id = (buf[0] << 16) + (buf[1] << 8) + buf[2];
	printk("----------%s, (%d) id[0]: %x id[1]: %x, id[2]: %x, id:%08x \n", __func__, __LINE__, buf[0], buf[1], buf[2], panel_id);

	//  if(panel_id == PANEL_yh101_ID)
	return 0;
	//  else
	//      return -1;
}
#endif

static void panel_dev_panel_init(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	int  i;
	int num_of_param;
	struct dsi_cmd_packet *param_list = NULL;

	if (dsi->video_config->no_of_lanes == 4) {
		param_list = &fitipower_yh101_800_1280_4lane_cmd_list[0];
		num_of_param = ARRAY_SIZE(fitipower_yh101_800_1280_4lane_cmd_list);
	} else {
		PANEL_DEBUG_MSG("enter %s %d \n", __func__, __LINE__);
	}

	for (i = 0; i < num_of_param; i++) {
		ops->cmd_write(dsi,  param_list[i]);
	}
}

static void panel_dev_panel_init1(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	struct dsi_cmd_packet data_to_send = {0x15, 0x35, 0x00};
	ops->cmd_write(dsi,  data_to_send);
}

static void panel_dev_set_sequence(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct yh101 *lcd = dev_get_drvdata(&dsim_dev->dev);
	PANEL_DEBUG_MSG("enter %s %d \n", __func__, __LINE__);

	panel_dev_panel_init(panel);
	panel_dev_sleep_out(panel);
	msleep(120);
	panel_dev_display_on(panel);
	msleep(5);
	panel_dev_panel_init1(panel);
	lcd->power = FB_BLANK_UNBLANK;
}

static struct fb_videomode panel_modes[] = {
	[0] = {
		.name = "fitipower_yh101-lcd",
		.xres = 800,
		.yres = 1280,

		.refresh = 60,
		.left_margin = 40,  //hbp
		.right_margin = 40, //hfp
		.hsync_len = 40,    //hsync

		.upper_margin = 30, //vbp
		.lower_margin = 30, //vfp
		.vsync_len = 15,    //vsync
		.sync                   = FB_SYNC_HOR_HIGH_ACT & FB_SYNC_VERT_HIGH_ACT,
		.vmode                  = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
};

struct jzdsi_data jzdsi_pdata[] = {
	[0] = {
		.modes = &panel_modes[0],
		.video_config.no_of_lanes = 4,
		.video_config.virtual_channel = 0,
		.video_config.color_coding = COLOR_CODE_24BIT,
		.video_config.video_mode = VIDEO_BURST_WITH_SYNC_PULSES,
		.video_config.receive_ack_packets = 0,  /* enable receiving of ack packets */
		.video_config.is_18_loosely = 0,
		.video_config.data_en_polarity = 1,
		.video_config.byte_clock = 0, // driver will auto calculate byte_clock.
		.video_config.byte_clock_coef = MIPI_PHY_BYTE_CLK_COEF_MUL6_DIV5, // byte_clock *6/5.
		.video_config.lane0_pn_swap = 0,
		.video_config.lane1_pn_swap = 0,
		.video_config.lane2_pn_swap = 0,
		.video_config.lane3_pn_swap = 0,
		.video_config.clk_lane_pn_swap = 0,

		.dsi_config.max_lanes = 4,
		.dsi_config.max_hs_to_lp_cycles = 100,
		.dsi_config.max_lp_to_hs_cycles = 40,
		.dsi_config.max_bta_cycles = 4095,
		.dsi_config.color_mode_polarity = 1,
		.dsi_config.shut_down_polarity = 1,
		.dsi_config.max_bps = 2750,
		.bpp_info = 24,
	},
};

static struct tft_config yh101_cfg = {
	.pix_clk_inv = 0,
	.de_dl = 0,
	.sync_dl = 0,
	.color_even = TFT_LCD_COLOR_EVEN_RGB,
	.color_odd = TFT_LCD_COLOR_ODD_RGB,
	.mode = TFT_LCD_MODE_PARALLEL_888,
};

struct lcd_panel lcd_panel = {
	.num_modes = ARRAY_SIZE(panel_modes),
	.modes = panel_modes,
	.dsi_pdata = jzdsi_pdata,

	.lcd_type = LCD_TYPE_MIPI_TFT,
	.tft_config = &yh101_cfg,
	.bpp = 24,
	.width = 62,
	.height = 110,
	.dither_enable = 0,
	.dither.dither_red = 0,
	.dither.dither_green = 0,
	.dither.dither_blue = 0,
};

#define POWER_IS_ON(pwr)        ((pwr) <= FB_BLANK_NORMAL)
static int panel_set_power(struct lcd_device *lcd, int power)
{
	return 0;
}

static int panel_get_power(struct lcd_device *lcd)
{
	struct panel_dev *panel = lcd_get_data(lcd);

	return panel->power;
}

/*
* @ pannel_yh101_lcd_ops, register to kernel common backlight/lcd.c framworks.
*/
static struct lcd_ops panel_lcd_ops = {
	.set_power = panel_set_power,
	.get_power = panel_get_power,
};

static int of_panel_parse(struct device *dev)
{
	struct panel_dev *panel = dev_get_drvdata(dev);
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	int ret = 0;
	PANEL_DEBUG_MSG("enter %s %d \n", __func__, __LINE__);

	panel->rst.gpio = of_get_named_gpio_flags(np, "ingenic,rst-gpio", 0, &flags);
	if (gpio_is_valid(panel->rst.gpio)) {
		panel->rst.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = gpio_request_one(panel->rst.gpio, GPIOF_DIR_OUT, "rst");
		if (ret < 0) {
			dev_err(dev, "Failed to request rst pin!\n");
			return ret;
		}
		gpio_direction_output(panel->rst.gpio, !panel->rst.active_level);
	} else {
		dev_warn(dev, "invalid gpio rst.gpio: %d\n", panel->rst.gpio);
	}

	panel->lcd_pwm.gpio = of_get_named_gpio_flags(np, "ingenic,lcd-pwm-gpio", 0, &flags);
	if (gpio_is_valid(panel->lcd_pwm.gpio)) {
		panel->lcd_pwm.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = gpio_request_one(panel->lcd_pwm.gpio, GPIOF_DIR_OUT, "lcd-pwm");
		if (ret < 0) {
			dev_err(dev, "Failed to request lcd-pwm pin!\n");
		}
	} else {
		dev_warn(dev, "invalid gpio lcd-pwm.gpio: %d\n", panel->lcd_pwm.gpio);
	}

	return 0;
}

static int panel_dev_probe(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct yh101 *lcd;
	int ret;

	lcd = devm_kzalloc(&dsim_dev->dev, sizeof(struct yh101), GFP_KERNEL);
	if (!lcd) {
		dev_err(&dsim_dev->dev, "failed to allocate fitipower_yh101 structure.\n");
		return -ENOMEM;
	}

	lcd->dsim_dev = dsim_dev;
	lcd->dev = &dsim_dev->dev;

	lcd->ld = lcd_device_register("fitipower_yh101", lcd->dev, lcd,
	                              &panel_lcd_ops);
	if (IS_ERR(lcd->ld)) {
		dev_err(lcd->dev, "failed to register lcd ops.\n");
		return PTR_ERR(lcd->ld);
	}

	dev_set_drvdata(&dsim_dev->dev, lcd);

	dev_dbg(lcd->dev, "probed fitipower_yh101 panel driver.\n");

	panel->dsim_dev = dsim_dev;
	panel->lcd = lcd->ld;
	return 0;

}

static int panel_suspend(struct mipi_dsim_lcd_device *dsim_dev)
{
	return 0;
}

static int panel_resume(struct mipi_dsim_lcd_device *dsim_dev)
{
	return 0;
}

static struct mipi_dsim_lcd_driver panel_dev_dsim_ddi_driver = {
	.name = "fitipower_yh101-lcd",
	.id = -1,

	.power_on = panel_dev_power_on,
	.set_sequence = panel_dev_set_sequence,
	.probe = panel_dev_probe,
	.suspend = panel_suspend,
	.resume = panel_resume,
};

#ifdef CONFIG_PM
static int panel_pm_suspend(struct device *dev)
{
	panel_dev_display_off(panel);
	panel_dev_sleep_in(panel);

	return 0;
}

static int panel_pm_resume(struct device *dev)
{

	return 0;
}

static const struct dev_pm_ops panel_pm_ops = {
	.suspend = panel_pm_suspend,
	.resume = panel_pm_resume,
};
#endif

struct mipi_dsim_lcd_device panel_dev_device = {
	.name       = "fitipower_yh101-lcd",
	.id = 0,
};

/*
* @panel_probe

*   1. Register to ingenicfb.
*   2. Register to lcd.
*   3. Register to backlight if possible.

* @pdev

* @Return -
*/
static int panel_probe(struct platform_device *pdev)
{
	int ret = 0;
	PANEL_DEBUG_MSG("enter %s %d \n", __func__, __LINE__);

	panel = kzalloc(sizeof(struct panel_dev), GFP_KERNEL);
	if (panel == NULL) {
		dev_err(&pdev->dev, "Faile to alloc memory!");
		return -ENOMEM;
	}
	panel->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, panel);

	ret = of_panel_parse(&pdev->dev);
	if (ret < 0) {
		goto err_of_parse;
	}
	mipi_dsi_register_lcd_device(&panel_dev_device);
	mipi_dsi_register_lcd_driver(&panel_dev_dsim_ddi_driver);

	ret = ingenicfb_register_panel(&lcd_panel);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register lcd panel!\n");
		goto err_of_parse;
	}

	return 0;

err_of_parse:
	if (gpio_is_valid(panel->rst.gpio)) {
		gpio_free(panel->rst.gpio);
	}

	kfree(panel);
	return ret;
}

static int panel_remove(struct platform_device *pdev)
{
	struct panel_dev *panel = dev_get_drvdata(&pdev->dev);

	lcd_device_unregister(panel->lcd);
	device_unregister(&panel_dev_device.dev);
	gpio_free(panel->rst.gpio);
	return 0;
}

static const struct of_device_id panel_of_match[] = {
	{ .compatible = "ingenic,yh101", },
	{},
};

static struct platform_driver panel_driver = {
	.probe      = panel_probe,
	.remove     = panel_remove,
	.driver     = {
		.name   = "yh101",
		.of_match_table = panel_of_match,
#ifdef CONFIG_PM
		.pm = &panel_pm_ops,
#endif
	},
};

module_platform_driver(panel_driver);

MODULE_LICENSE("GPL");
