/*
 * driver/video/fbdev/ingenic/x2000_v12/displays/panel-axs15231b.c
 *
 * Copyright (C) 2016 Ingenic Semiconductor Inc.
 *
 * This program is free software, you can redistribute it and/or modify it
 *
 * under the terms of the GNU General Public License version 2 as published by
 *
 * the Free Software Foundation.
 */
#if 0
&pwmz {
	status = "okay";
	pinctrl - names = "default";
	pinctrl - 0 = <&pwm7_pb>;
};

&dpu {
	status = "okay";
	ingenic, disable - rdma - fb = <1>;
	/*Defines the init state of composer fb export infomations.*/
	ingenic, layer - exported = <1 0 0 0>;
	ingenic, layer - frames   = <1 1 1 1>;
	ingenic, layer - framesize = <240 1020>, <240 1020>, <240 1020>, <0 0>; /*Max framesize for each layer.*/
	layer, src - size          = <240 1020>, <240 1020>, <240 1020>, <0 0>; /*Layer src size should smaller than framesize*/
	layer, target - size       = <240 1020>, <240 1020>, <240 1020>, <0 0>; /*Target Size should smaller than src_size.*/
	layer, target - pos        = <0 0>, <0 0>, <0 0>, <0 0>; /*target pos , the start point of the target panel.*/
	layer, enable            = <1 0 0 0>;                                   /*layer enabled or disabled.*/
	ingenic, logo - pan - layer  = <0>;                                     /*on which layer should init logo pan on.*/

	port {
dpu_out_ep: endpoint {
			remote - endpoint = <&panel_axs15231b_ep>;
		};
	};
};

/ {
	display - dpi {
		compatible = "simple-bus";
#interrupt-cells = <1>;
#address-cells = <1>;
#size-cells = <1>;
		ranges = <>;
		panel_axs15231b {
			compatible = "ingenic,axs15231b";
			status = "okay";
			pinctrl - names = "default";
			ingenic, rst - gpio = <&gpd 19 GPIO_ACTIVE_LOW INGENIC_GPIO_NOBIAS>;
			ingenic, vdd - regu = "lcd_on";

			port {
panel_axs15231b_ep: endpoint {
					remote - endpoint = <&dpu_out_ep>;
				};
			};
		};

	};

	backlight {
		compatible = "pwm-backlight";
		pwms = <&pwmz 7 10000>; /* arg1: pwm channel id [0~15]. arg2: period in ns. */
		brightness - levels = <0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15>;
		default - brightness - level = <10>;
		/*default brightness is 5, you can echo x > /sys/devices/platform/backlight/backlight/backlight/brightness to adjust backlight*/
	};
};
#endif
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
#include<linux/regulator/consumer.h>

#include "../include/ingenicfb.h"
#include "../include/jz_dsim.h"

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

	struct board_gpio rst;
	struct board_gpio lcd_pwm;
	struct regulator *vdd_en_regu;  /* Digital Core supply */
	struct mipi_dsim_lcd_device *dsim_dev;
};

static struct panel_dev *panel;

#define lcd_to_master(a)    (a->dsim_dev->master)
#define lcd_to_master_ops(a)    ((lcd_to_master(a))->master_ops)

struct axs15231b {
	struct device *dev;
	unsigned int power;

	struct lcd_device *ld;
	struct backlight_device *bd;
	struct mipi_dsim_lcd_device *dsim_dev;

};
#define LCD_NO_INIT_CMD
//#define LCD_BIST_MODE
//#define LCD_RESET_DBG

#define LCD_INIT_CFG_V10
static struct dsi_cmd_packet fitipower_axs15231b_240_1020_cmd_list1[] = {
#ifndef LCD_NO_INIT_CMD

#ifdef LCD_INIT_CFG_V10

	{0x29, 0x09, 0x00, {0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5A, 0xA5}},
	{0x29, 0x12, 0x00, {0xA0, 0x00, 0x30, 0x00, 0x02, 0x00, 0x00, 0x0c, 0x3F, 0x20, 0x05, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00}},
#ifdef LCD_BIST_MODE
	{0x29, 0x20, 0x00, {0xA2, 0x21, 0x04, 0x0A, 0x3C, 0x32, 0x50, 0x32, 0xFC, 0xf0, 0x38, 0x7F, 0x7F, 0x7F, 0x20, 0xF8, 0x10, 0x02, 0xFF, 0xFF, 0xF0, 0x90, 0x01, 0x32, 0xA0, 0x91, 0xC0, 0x20, 0x7F, 0xFF, 0x00, 0x04}},
#else
	{0x29, 0x20, 0x00, {0xA2, 0x20, 0x04, 0x0A, 0x3C, 0x32, 0x50, 0x32, 0xFC, 0xf0, 0x38, 0x7F, 0x7F, 0x7F, 0x20, 0xF8, 0x10, 0x02, 0xFF, 0xFF, 0xF0, 0x90, 0x01, 0x32, 0xA0, 0x91, 0xC0, 0x20, 0x7F, 0xFF, 0x00, 0x04}},
#endif
	{0x29, 0x1f, 0x00, {0xD0, 0xFC, 0xF0, 0x32, 0x24, 0x08, 0x05, 0x10, 0x07, 0x59, 0x21, 0xC2, 0x40, 0x22, 0x02, 0xAA, 0x03, 0x10, 0x12, 0x60, 0x14, 0x1E, 0x51, 0x15, 0x00, 0xa7, 0x00, 0x00, 0x03, 0x3D, 0x12}},

	{0x29, 0x17, 0x00, {0xA3, 0xA0, 0x06, 0xA9, 0x00, 0x08, 0x02, 0x0A, 0x0a, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x55, 0x55}},

	{0x29, 0x1f, 0x00, {0xC1, 0x31, 0x04, 0x02, 0x02, 0x71, 0x05, 0x24, 0x55, 0x02, 0x00, 0x41, 0x01, 0x53, 0xFF, 0xFF, 0xFF, 0x4F, 0x52, 0x00, 0x4F, 0x52, 0x00, 0x45, 0x3B, 0x0B, 0x02, 0x0D, 0x00, 0xFF, 0x40}},

	{0x29, 0x0c, 0x00, {0xC3, 0x00, 0x00, 0x00, 0x50, 0x03, 0x00, 0x00, 0x00, 0x01, 0x80, 0x01}},
	{0x29, 0x1e, 0x00, {0xC4, 0x00, 0x24, 0x33, 0x80, 0x6C, 0xea, 0x64, 0x32, 0xC8, 0x64, 0xC8, 0x32, 0x90, 0x90, 0x11, 0x06, 0xDC, 0xFA, 0x00, 0x00, 0x80, 0xFE, 0x10, 0x10, 0x00, 0x0A, 0x0A, 0x44, 0x50}},
	{0x29, 0x18, 0x00, {0xC5, 0x18, 0x00, 0x00, 0x03, 0xFE, 0x45, 0x35, 0x40, 0x30, 0x10, 0x88, 0xDE, 0x0D, 0x08, 0x0F, 0x0F, 0x01, 0x45, 0x35, 0x40, 0x10, 0x10, 0x00}},

	{0x29, 0x15, 0x00, {0xC6, 0x05, 0x0A, 0x05, 0x0A, 0x00, 0xE0, 0x2E, 0x0B, 0x12, 0x22, 0x12, 0x22, 0x01, 0x00, 0x00, 0x02, 0x6A, 0x18, 0xC8, 0x22}},
	{0x29, 0x15, 0x00, {0xC7, 0x50, 0x36, 0x28, 0x00, 0xa2, 0x80, 0x8f, 0x00, 0x80, 0xff, 0x07, 0x11, 0x9c, 0x6f, 0xff, 0x26, 0x0c, 0x0d, 0x0e, 0x0f}},
	{0x29, 0x05, 0x00, {0xC9, 0x33, 0x44, 0x44, 0x01}},

	{0x29, 0x1c, 0x00, {0xCF, 0x2C, 0x1E, 0x88, 0x58, 0x13, 0x18, 0x56, 0x18, 0x1E, 0x68, 0x70, 0x00, 0x63, 0x05, 0x22, 0xC4, 0x0C, 0x77, 0x22, 0x44, 0xAA, 0x55, 0x04, 0x06, 0x0F, 0xA0, 0x08}},

	{0x29, 0x1f, 0x00, {0xD5, 0x28, 0x28, 0x89, 0x00, 0x35, 0x03, 0x7F, 0x77, 0x80, 0x7F, 0x77, 0x80, 0x04, 0x28, 0x03, 0x46, 0x20, 0x27, 0x20, 0x27, 0x84, 0x00, 0x00, 0x00, 0xFC, 0x53, 0x81, 0x28, 0x28, 0x00}},
	{0x29, 0x1f, 0x00, {0xD6, 0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE, 0x95, 0x00, 0x01, 0x83, 0x75, 0x36, 0x20, 0x75, 0x36, 0x20, 0x3F, 0x03, 0x03, 0x03, 0x10, 0x10, 0x00, 0x83, 0x51, 0x20, 0x01, 0x00}},
	{0x29, 0x14, 0x00, {0xD7, 0x1F, 0x02, 0x00, 0x0a, 0x08, 0x0E, 0x0c, 0x19, 0x1F, 0x18, 0x19, 0x1F, 0x1C, 0x20, 0x04, 0x00, 0x15, 0x2d, 0x1F}},
	{0x29, 0x0d, 0x00, {0xD8, 0x1F, 0x03, 0x01, 0x0b, 0x09, 0x0f, 0x0d, 0x19, 0x1F, 0x18, 0x19, 0x1F}},

	{0x29, 0x09, 0x00, {0xDF, 0x44, 0x73, 0x4B, 0x69, 0x00, 0x0A, 0x02, 0x90}},
	/////////////////2.2

	{0x29, 0x12, 0x00, {0xE0, 0x29, 0x0b, 0x0A, 0x0D, 0x09, 0x05, 0x11, 0x15, 0x43, 0x0F, 0x0A, 0x0D, 0x11, 0x25, 0x2C, 0x05, 0x00}},
	{0x29, 0x12, 0x00, {0xE1, 0x3a, 0x05, 0x0A, 0x0D, 0x09, 0x05, 0x11, 0x15, 0x43, 0x0F, 0x0A, 0x0D, 0x11, 0x25, 0x2C, 0x0b, 0x17}},

	/////gamma 2.0/////

	{0x29, 0x12, 0x00, {0xE2, 0x19, 0x20, 0x0A, 0x11, 0x09, 0x06, 0x11, 0x25, 0xD4, 0x22, 0x0B, 0x13, 0x12, 0x2D, 0x20, 0x2f, 0x03}},

	{0x29, 0x12, 0x00, {0xE3, 0x38, 0x20, 0x0A, 0x11, 0x09, 0x06, 0x11, 0x25, 0xC4, 0x21, 0x0A, 0x12, 0x11, 0x2C, 0x2e, 0x2f, 0x27}},

	/////gamma 2.4/////

	{0x29, 0x12, 0x00, {0xE4, 0x19, 0x20, 0x0D, 0x14, 0x0D, 0x08, 0x12, 0x2A, 0xD4, 0x26, 0x0E, 0x15, 0x13, 0x34, 0x39, 0x2f, 0x03}},

	{0x29, 0x12, 0x00, {0xE5, 0x38, 0x20, 0x0D, 0x13, 0x0D, 0x07, 0x12, 0x29, 0xC4, 0x25, 0x0D, 0x15, 0x12, 0x33, 0x39, 0x2f, 0x27}},

	{0x29, 0x09, 0x00, {0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},

#endif
#endif

};

static void panel_dev_sleep_in(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_cmd_packet data_to_send = {0x05, 0x10, 0x00};

	ops->cmd_write(lcd_to_master(lcd), data_to_send);
}

static void panel_dev_sleep_out(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_cmd_packet data_to_send = {0x05, 0x11, 0x00};
	ops->cmd_write(lcd_to_master(lcd), data_to_send);
}

static void panel_dev_display_on(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);//color bar turn off 0x29 command.
	struct dsi_cmd_packet data_to_send = {0x05, 0x29, 0x00};
	ops->cmd_write(lcd_to_master(lcd), data_to_send);
}
static void panel_dev_display_test(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);//color bar turn off 0x29 command.
	struct dsi_cmd_packet data_to_send = {0x05, 0x23, 0x00};
	ops->cmd_write(lcd_to_master(lcd), data_to_send);

}

static void panel_dev_display_off(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_cmd_packet data_to_send = {0x05, 0x28, 0x00};
	ops->cmd_write(lcd_to_master(lcd), data_to_send);
}

static void panel_dev_panel_init(struct panel_dev *lcd)
{
	int  i;
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);

	for (i = 0; i < ARRAY_SIZE(fitipower_axs15231b_240_1020_cmd_list1); i++) {
		ops->cmd_write(dsi,  fitipower_axs15231b_240_1020_cmd_list1[i]);
		msleep(10);
	}
}

static int panel_dev_ioctl(struct mipi_dsim_lcd_device *dsim_dev, int cmd)
{
	return 0;
}
static void panel_dev_set_sequence(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct axs15231b *lcd = dev_get_drvdata(&dsim_dev->dev);

	//printk("axs15231b >>>>>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);

	panel_dev_panel_init(panel);
	panel_dev_sleep_out(panel);
	msleep(200);
	panel_dev_display_on(panel);
	msleep(100);
	lcd->power = FB_BLANK_UNBLANK;
}
static void panel_dev_power_on(struct mipi_dsim_lcd_device *dsim_dev, int power)
{
	struct board_gpio *rst = &panel->rst;
	//printk("axs after power on >>>>>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);

	if (power) {
#ifdef LCD_RESET_DBG
		gpio_direction_output(rst->gpio, !rst->active_level);
		msleep(10);
#else
		gpio_direction_output(rst->gpio, !rst->active_level);
		msleep(10);
		gpio_direction_output(rst->gpio, rst->active_level);
		msleep(20);
		gpio_direction_output(rst->gpio, !rst->active_level);
		msleep(120);
#endif
	}

	panel->power = power;
}

static struct fb_videomode panel_modes = {
	.name = "fitipower_axs15231b-lcd",
	.xres = 240,
	.yres = 1020,

	.refresh = 59,
#ifdef LCD_NO_INIT_CMD
	.left_margin = 60,//hbp
	.right_margin = 40,//hfp
	.hsync_len = 10, //hsync

	.upper_margin = 60,//vbp
	.lower_margin = 120,//vfp
	.vsync_len = 10, //vsync
#elif defined LCD_INIT_CFG_V10
	.left_margin = 50,//hbp
	.right_margin = 50,//hfp
	.hsync_len = 10, //hsync

	.upper_margin = 60,//vbp
	.lower_margin = 80,//vfp
	.vsync_len = 10, //vsync
#endif
	.sync                   = FB_SYNC_HOR_HIGH_ACT & FB_SYNC_VERT_HIGH_ACT,
	.vmode                  = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct jzdsi_data jzdsi_pdata = {
	.modes = &panel_modes,
	.video_config.no_of_lanes = 1,
	.video_config.virtual_channel = 0,
	.video_config.color_coding = COLOR_CODE_24BIT,
	.video_config.video_mode = VIDEO_BURST_WITH_SYNC_PULSES,
	.video_config.receive_ack_packets = 0,  /* enable receiving of ack packets */
	.video_config.is_18_loosely = 0,
	.video_config.data_en_polarity = 1,
	.video_config.byte_clock = 75000, // driver will auto calculate byte_clock.
	.video_config.byte_clock_coef = MIPI_PHY_BYTE_CLK_COEF_MUL3_DIV2, // byte_clock *3/2.

	.dsi_config.max_lanes = 1,
	.dsi_config.max_hs_to_lp_cycles = 100,
	.dsi_config.max_lp_to_hs_cycles = 40,
	.dsi_config.max_bta_cycles = 4095,
	.dsi_config.color_mode_polarity = 1,
	.dsi_config.shut_down_polarity = 1,
	.dsi_config.max_bps = 2750,
	//  .max_bps = 650,  // 650 Mbps
	.bpp_info = 24,
};
static struct tft_config kd050hdfia019_cfg = {
	.pix_clk_inv = 0,
	.de_dl = 0,
	.sync_dl = 0,
	.color_even = TFT_LCD_COLOR_EVEN_RGB,
	.color_odd = TFT_LCD_COLOR_ODD_RGB,
	.mode = TFT_LCD_MODE_PARALLEL_888,
};

static struct lcd_panel lcd_panel = {
	.num_modes = 1,
	.modes = &panel_modes,
	.dsi_pdata = &jzdsi_pdata,

	.lcd_type = LCD_TYPE_MIPI_TFT,
	.tft_config = &kd050hdfia019_cfg,
	.bpp = 24,
	.width = 23,
	.height = 98,
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

/**
* @ pannel_axs15231b_lcd_ops, register to kernel common backlight/lcd.c framworks.
*/
static struct lcd_ops panel_lcd_ops = {
	//.early_set_power = panel_set_power,
	.set_power = panel_set_power,
	.get_power = panel_get_power,
};

static int of_panel_parse(struct device *dev)
{
	struct panel_dev *panel = dev_get_drvdata(dev);
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	int ret = 0;

	panel->rst.gpio = of_get_named_gpio_flags(np, "ingenic,rst-gpio", 0, &flags);
	if (gpio_is_valid(panel->rst.gpio)) {
		panel->rst.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		gpio_direction_output(panel->rst.gpio, !panel->rst.active_level);
	} else {
		dev_warn(dev, "invalid gpio rst.gpio: %d\n", panel->rst.gpio);
	}

	panel->vdd_en_regu = devm_regulator_get(dev, "lcd_on");
	if (IS_ERR(panel->vdd_en_regu)) {
		printk("\n\n*************No vdd_en_regu regulator found**********\n");
		if (PTR_ERR(panel->vdd_en_regu) == -EPROBE_DEFER) {
			return -EPROBE_DEFER;
		}
	}
	printk("\n\n***********axs15231b vdd_en_regu regulator found***********\n");
	regulator_set_voltage(panel->vdd_en_regu, 3300000UL, 3300000UL);
	regulator_enable(panel->vdd_en_regu);

	/**lcd_bl_pwm pin as normal io pin config pull up***/
	//  panel->lcd_pwm.gpio = of_get_named_gpio_flags(np, "ingenic,lcd-pwm-gpio", 0, &flags);
	//  if(gpio_is_valid(panel->lcd_pwm.gpio)) {
	//      panel->lcd_pwm.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	//      ret = gpio_direction_output(panel->lcd_pwm.gpio,1);
	//      if(ret < 0) {
	//          dev_err(dev, "Failed to request lcd-pwm pin!\n");
	//          return ret;
	//      }
	//  } else {
	//      dev_warn(dev, "invalid gpio lcd-pwm.gpio: %d\n", panel->lcd_pwm.gpio);
	//  }
	//

	return 0;
}

static int panel_dev_probe(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct axs15231b *lcd;
	int ret = 0;

	lcd = devm_kzalloc(&dsim_dev->dev, sizeof(struct axs15231b), GFP_KERNEL);
	if (!lcd) {
		dev_err(&dsim_dev->dev, "failed to allocate fitipower_axs15231b structure.\n");
		return -ENOMEM;
	}

	lcd->dsim_dev = dsim_dev;
	lcd->dev = &dsim_dev->dev;

	lcd->ld = lcd_device_register("fitipower_axs15231b", lcd->dev, lcd, &panel_lcd_ops);
	if (IS_ERR(lcd->ld)) {
		dev_err(lcd->dev, "failed to register lcd ops.\n");
		return PTR_ERR(lcd->ld);
	}

	dev_set_drvdata(&dsim_dev->dev, lcd);
	dev_dbg(lcd->dev, "probed fitipower_axs15231b panel driver.\n");
	panel->dsim_dev = dsim_dev;
	return 0;
}

static int panel_suspend(struct mipi_dsim_lcd_device *dsim_dev)
{
	panel_dev_display_off(panel);
	panel_dev_sleep_in(panel);
	return 0;
}

static int panel_resume(struct mipi_dsim_lcd_device *dsim_dev)
{
	return 0;
}

static struct mipi_dsim_lcd_driver panel_dev_dsim_ddi_driver = {
	.name = "fitipower_axs15231b-lcd",
	.id = -1,
	.power_on = panel_dev_power_on,
	.set_sequence = panel_dev_set_sequence,
	.probe = panel_dev_probe,
	.suspend = panel_suspend,
	.resume = panel_resume,
};

static struct mipi_dsim_lcd_device panel_dev_device = {
	.name       = "fitipower_axs15231b-lcd",
	.id = 0,
};

/**
* @panel_probe
*
*   1. Register to ingenicfb.
*   2. Register to lcd.
*   3. Register to backlight if possible.
*
* @pdev
*
* @Return -
*/
static int panel_probe(struct platform_device *pdev)
{
	int ret = 0;
	printk("------------------------\n");
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
	printk("%s-%s  panel_probe end.\n", __FILE__, __func__);
	return 0;

err_of_parse:
	kfree(panel);
	return ret;
}

static int panel_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id panel_of_match[] = {
	{ .compatible = "ingenic,axs15231b", },
	{},
};

static struct platform_driver panel_driver = {
	.probe      = panel_probe,
	.remove     = panel_remove,
	.driver     = {
		.name   = "axs15231b",
		.of_match_table = panel_of_match,
	},
};

module_platform_driver(panel_driver);
MODULE_LICENSE("GPL");
