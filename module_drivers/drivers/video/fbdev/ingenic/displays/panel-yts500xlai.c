/*
 * driver/video/fbdev/ingenic/x2000_v12/displays/panel-yts500xlai.c
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
#include <linux/fb.h>
#include <linux/backlight.h>

#include "../include/ingenicfb.h"
#include "../include/jz_dsim.h"

extern void dump_dsi_reg(struct dsi_device *dsi);

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
	struct board_gpio vdd_en;
	struct board_gpio rst;
	struct board_gpio lcd_te;
	struct board_gpio lcd_pwm;

	struct mipi_dsim_lcd_device *dsim_dev;
};

struct panel_dev *panel;

#define lcd_to_master(a)    (a->dsim_dev->master)
#define lcd_to_master_ops(a)    ((lcd_to_master(a))->master_ops)

struct yts500xlai {
	struct device *dev;
	unsigned int power;
	unsigned int id;

	struct lcd_device *ld;
	struct backlight_device *bd;

	struct mipi_dsim_lcd_device *dsim_dev;

};

static struct dsi_cmd_packet fitipower_yts500xlai_480_800_cmd_list1[] = {

	/**yts500xlai***/
	{0x39, 0x04, 0x00, {0xB9, 0xFF, 0x83, 0x89}}, //set extc.
	{0x39, 0x04, 0x00, {0xB9, 0xFF, 0x83, 0x94}}, //pdf->p21.
	{0x39, 0x0B, 0x00, {0xB1, 0x48, 0x12, 0x72, 0x09, 0x33, 0x54, 0x51, 0x51, 0x30, 0x43}}, //set power.
	//  {0x39, 0x07, 0x00, {0xBA, 0x63, 0x03, 0x68, 0x6B, 0xB2, 0xC0}}, //set mipi 4lane.
	{0x39, 0x07, 0x00, {0xBA, 0x61, 0x03, 0x68, 0x6B, 0xB2, 0xC0}}, //set mipi 2lane.
	{0x39, 0x07, 0x00, {0xB2, 0x00, 0x80, 0x64, 0x0C, 0x06, 0x2F}}, //set display.
	//  test color
	//  {0x39, 0x0C, 0x00, {0xB2, 0x00, 0x80, 0x64, 0x0C, 0x06, 0x2F, 0x00, 0x00, 0x00, 0x00, 0xC8}}, //set display to color test.
	//  test color
	{0x39, 0x1F, 0x00, {0xB4, 0x76, 0x74, 0x76, 0x74, 0x76, 0x74, 0x01, 0x05, 0x84, 0x35, 0x00, 0x3f, 0x76, 0x74, 0x76, 0x74, 0x76, 0x74, 0x01, 0x05, 0x84, 0x3F, 0x00, 0xFF, 0x81, 0x81, 0x81, 0x81, 0x08, 0x01}}, //set cyc.
	{0x39, 0x22, 0x00, {0xD3, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x12, 0x10, 0x32, 0x10, 0x00, 0x00, 0x00, 0x32, 0x13, 0xC0, 0x00, 0x00, 0x32, 0x10, 0x08, 0x00, 0x00, 0x47, 0x04, 0x02, 0x02, 0x47, 0x04, 0x00, 0x47, 0x0C, 0x40}}, //set w_d(0xD3).
	{0x39, 0x2D, 0x00, {0xD5, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x20, 0x21, 0x22, 0x23, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,  0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x19, 0x19, 0x18, 0x18}}, //set GIP.
	{0x39, 0x2D, 0x00, {0xD6, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00, 0x23, 0x22, 0x21, 0x20, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,  0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x19, 0x19}}, //set.
	{0x39, 0x03, 0x00, {0xB6, 0x34, 0x34}}, //set VCOM.
	{0x39, 0x3B, 0x00, {0xE0, 0x00, 0x06, 0x13, 0x18, 0x1C, 0x20, 0x23, 0x22, 0x48, 0x58, 0x6A, 0x6B, 0x74, 0x88, 0x8E, 0x94, 0xA2, 0xA6, 0xA3, 0xB2, 0xC5, 0x63, 0x61, 0x66, 0x6C, 0x6C, 0x73, 0x7F, 0x7F, 0x00, 0x06, 0x13, 0x18, 0x1C, 0x20, 0x23, 0x22, 0x48, 0x58, 0x6A, 0x6B, 0x75, 0x89, 0x8F, 0x95, 0xA3, 0xA6, 0xA3, 0xB3, 0xC5, 0x63, 0x62, 0x67, 0x6C, 0x71, 0x78, 0x7F, 0x7F}}, //set GAMMA.
	{0x15, 0xCC, 0x03}, //set pannal.
	{0x39, 0x03, 0x00, {0xC0, 0x1F, 0x31}}, //set.
	{0x15, 0xD4, 0x02}, //set.
	{0x15, 0xBD, 0x01}, //set.
	{0x15, 0xB1, 0x60}, //set GAS.
	{0x15, 0xBD, 0x00}, //set .
	{0x39, 0x08, 0x00, {0xBF, 0x40, 0x81, 0x50, 0x00, 0x1A, 0xFC, 0x01}}, //set power option HX5186 Mode.
	{0x15, 0x36, 0x02}, // .
	/**yts500xlai***/
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
	//  struct dsi_cmd_packet data_to_send = {0x15, 0x11, 0x00};
	//  while (1) {
	ops->cmd_write(lcd_to_master(lcd), data_to_send);
	//  }
}

static void panel_dev_display_on(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);//color bar turn off 0x29 command.
	struct dsi_cmd_packet data_to_send = {0x05, 0x29, 0x00};
	//  struct dsi_cmd_packet data_to_send = {0x15, 0x29, 0x00};

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
	unsigned int value;
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);

	for (i = 0; i < ARRAY_SIZE(fitipower_yts500xlai_480_800_cmd_list1); i++) {
		ops->cmd_write(dsi,  fitipower_yts500xlai_480_800_cmd_list1[i]);
		value = mipi_dsih_read_word(dsi, R_DSI_HOST_INT_ST0);
		if (value & 0x180000) {
			printk("-----2--->i=%d\n", i);
		}
	}
	//  dump_dsi_reg(dsi);
}

static int panel_dev_ioctl(struct mipi_dsim_lcd_device *dsim_dev, int cmd)
{
	return 0;
}
static void panel_dev_set_sequence(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct yts500xlai *lcd = dev_get_drvdata(&dsim_dev->dev);

	printk(">>>>>>>>>>>>>>>>>>>>%s %d\n", __func__, __LINE__);

	panel_dev_panel_init(panel);
	panel_dev_sleep_out(panel);
	msleep(150);
	panel_dev_display_on(panel);
	msleep(20);
	/* dump_dsi_reg(dsi); */
	lcd->power = FB_BLANK_UNBLANK;
}
static void panel_dev_power_on(struct mipi_dsim_lcd_device *dsim_dev, int power)
{
	struct board_gpio *vdd_en = &panel->vdd_en;
	struct board_gpio *rst = &panel->rst;
	struct board_gpio *lcd_te = &panel->lcd_te;
	printk(">>>>>>>>>>>>>>>>>>>>%s %d\n", __func__, __LINE__);

	gpio_direction_output(vdd_en->gpio, vdd_en->active_level);

	if (gpio_is_valid(lcd_te->gpio)) {
		gpio_direction_input(lcd_te->gpio);
	}
	msleep(50);
	gpio_direction_output(rst->gpio, 0);
	msleep(50);
	gpio_direction_output(rst->gpio, 1);
	msleep(120);

	panel->power = power;
}

static struct fb_videomode panel_modes = {
	.name = "fitipower_yts500xlai-lcd",
	.xres = 720,
	.yres = 1280,

	//  .refresh = 10,
	//  .refresh = 30,
	.refresh = 35,
	//  .refresh = 60, /*no display*/
	//debug1
	.left_margin = 88,//hbp
	.right_margin = 35,//hfp
	.hsync_len = 10, //hsync

	.upper_margin = 37,//vbp
	.lower_margin = 13,//vfp
	.vsync_len = 10, //vsync
	//debug2
	//  .left_margin = 48,//hbp
	//  .right_margin = 16,//hfp
	//  .hsync_len = 8, //hsync
	//
	//  .upper_margin = 3,//vbp
	//  .lower_margin = 5,//vfp
	//  .vsync_len = 8, //vsync

	.sync                   = FB_SYNC_HOR_HIGH_ACT & FB_SYNC_VERT_HIGH_ACT,
	.vmode                  = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

struct jzdsi_data jzdsi_pdata = {
	.modes = &panel_modes,
	.video_config.no_of_lanes = 2,
	.video_config.virtual_channel = 0,
	.video_config.color_coding = COLOR_CODE_16BIT_CONFIG1,
	.video_config.video_mode = VIDEO_BURST_WITH_SYNC_PULSES,
	.video_config.receive_ack_packets = 0,  /* enable receiving of ack packets */
	.video_config.is_18_loosely = 0,
	.video_config.data_en_polarity = 1,
	.video_config.byte_clock = 0, // driver will auto calculate byte_clock.
	//  .video_config.byte_clock_coef = MIPI_PHY_BYTE_CLK_COEF_MUL3_DIV2, //1: byte_clock *3/2.
	//  .video_config.byte_clock_coef = 2, // byte_clock *4/3 .
	.video_config.byte_clock_coef = 3, // byte_clock *5/4 35fps.

	.dsi_config.max_lanes = 2,
	.dsi_config.max_hs_to_lp_cycles = 100,
	.dsi_config.max_lp_to_hs_cycles = 40,
	//  .dsi_config.max_hs_to_lp_cycles = 200,
	//  .dsi_config.max_lp_to_hs_cycles = 80,
	.dsi_config.max_bta_cycles = 4095,
	.dsi_config.color_mode_polarity = 1,
	.dsi_config.shut_down_polarity = 1,
	.dsi_config.max_bps = 2750,
	//  .max_bps = 650,  // 650 Mbps
	//  .bpp_info = 24,
	.bpp_info = 16,
};
static struct tft_config kd050hdfia019_cfg = {
	.pix_clk_inv = 0,
	.de_dl = 0,
	.sync_dl = 0,
	.color_even = TFT_LCD_COLOR_EVEN_RGB,
	.color_odd = TFT_LCD_COLOR_ODD_RGB,
	.mode = TFT_LCD_MODE_PARALLEL_565,
};

struct lcd_panel lcd_panel = {
	.num_modes = 1,
	.modes = &panel_modes,
	.dsi_pdata = &jzdsi_pdata,

	.lcd_type = LCD_TYPE_MIPI_TFT,
	.tft_config = &kd050hdfia019_cfg,
	//  .bpp = 24,
	.bpp = 16,
	//  .width = 700,
	//  .height = 1230,
	.width = 68,
	.height = 121,
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
* @ pannel_yts500xlai_lcd_ops, register to kernel common backlight/lcd.c framworks.
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

	panel->rst.gpio = of_get_named_gpio_flags(np, "ingenic,rst-gpio", 0, &flags);
	if (gpio_is_valid(panel->rst.gpio)) {
		panel->rst.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = gpio_request_one(panel->rst.gpio, GPIOF_DIR_OUT, "rst");
		if (ret < 0) {
			dev_err(dev, "Failed to request rst pin!\n");
			goto err_request_rst;
		}
	} else {
		dev_warn(dev, "invalid gpio rst.gpio: %d\n", panel->rst.gpio);
	}

	return 0;
err_request_lcd_te:
	if (gpio_is_valid(panel->rst.gpio)) {
		gpio_free(panel->rst.gpio);
	}
err_request_rst:
	if (gpio_is_valid(panel->vdd_en.gpio)) {
		gpio_free(panel->vdd_en.gpio);
	}
	return ret;
}

static int panel_dev_probe(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct yts500xlai *lcd;
	lcd = devm_kzalloc(&dsim_dev->dev, sizeof(struct yts500xlai), GFP_KERNEL);
	if (!lcd) {
		dev_err(&dsim_dev->dev, "failed to allocate fitipower_yts500xlai structure.\n");
		return -ENOMEM;
	}

	lcd->dsim_dev = dsim_dev;
	lcd->dev = &dsim_dev->dev;

	lcd->ld = lcd_device_register("fitipower_yts500xlai", lcd->dev, lcd,
	                              &panel_lcd_ops);
	if (IS_ERR(lcd->ld)) {
		dev_err(lcd->dev, "failed to register lcd ops.\n");
		return PTR_ERR(lcd->ld);
	}

	dev_set_drvdata(&dsim_dev->dev, lcd);

	dev_dbg(lcd->dev, "probed fitipower_yts500xlai panel driver.\n");

	panel->dsim_dev = dsim_dev;

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

#ifdef CONFIG_PM
static int panel_pm_suspend(struct device *dev)
{
	struct board_gpio *vdd_en = &panel->vdd_en;

	printk(">>>>>>>>>>>>>>>>>>>>%s %d\n", __func__, __LINE__);
	panel_dev_display_off(panel);
	panel_dev_sleep_in(panel);
	gpio_direction_output(vdd_en->gpio, !vdd_en->active_level);

	return 0;
}

static int panel_pm_resume(struct device *dev)
{
	struct board_gpio *vdd_en = &panel->vdd_en;
	printk(">>>>>>>>>>>>>>>>>>>>%s %d\n", __func__, __LINE__);

	gpio_direction_output(vdd_en->gpio, vdd_en->active_level);

	return 0;
}

static const struct dev_pm_ops panel_pm_ops = {
	.suspend = panel_pm_suspend,
	.resume = panel_pm_resume,
};
#endif

static struct mipi_dsim_lcd_driver panel_dev_dsim_ddi_driver = {
	.name = "fitipower_yts500xlai-lcd",
	.id = -1,

	.power_on = panel_dev_power_on,
	.set_sequence = panel_dev_set_sequence,
	.probe = panel_dev_probe,
	//  .suspend = panel_suspend,
	.resume = panel_resume,
};

struct mipi_dsim_lcd_device panel_dev_device = {
	.name       = "fitipower_yts500xlai-lcd",
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
	kfree(panel);
	return ret;
}

static int panel_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id panel_of_match[] = {
	{ .compatible = "ingenic,yts500xlai", },
	{},
};

static struct platform_driver panel_driver = {
	.probe      = panel_probe,
	.remove     = panel_remove,
	.driver     = {
		.name   = "yts500xlai",
		.of_match_table = panel_of_match,
#ifdef CONFIG_PM
		.pm = &panel_pm_ops,
#endif
	},
};

module_platform_driver(panel_driver);
MODULE_LICENSE("GPL");
