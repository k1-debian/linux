/*
 * driver/video/fbdev/ingenic/x2000_v12/displays/panel-kd050hdfia019.c
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

	struct regulator *vcc;
	struct board_gpio vdd_en;
	struct board_gpio rst;
	struct board_gpio pwm;

	struct mipi_dsim_lcd_device *dsim_dev;
};

struct panel_dev *panel;

struct dsi_cmd_packet kd050hdfia019_cmd_list[] = {
	/* for KD050FWFIA019-C019A */
	{0x39, 0x06, 0x00, {0xFF, 0xFF, 0x98, 0x06, 0x04, 0x01}},
	{0x15, 0x08, 0x10},
	{0x15, 0x20, 0x00},
	{0x15, 0x21, 0x01},
	{0x15, 0x30, 0x01},
	{0x15, 0x31, 0x00},
	{0x15, 0x40, 0x16},
	{0x15, 0x41, 0x33},
	{0x15, 0x42, 0x03},
	{0x15, 0x43, 0x09},
	{0x15, 0x44, 0x06},
	{0x15, 0x50, 0x88},
	{0x15, 0x51, 0x88},
	{0x15, 0x52, 0x00},
	{0x15, 0x53, 0x49},
	{0x15, 0x55, 0x49},
	{0x15, 0x60, 0x07},
	{0x15, 0x61, 0x00},
	{0x15, 0x62, 0x07},
	{0x15, 0x63, 0x00},
	{0x15, 0xA0, 0x00},
	{0x15, 0xA1, 0x09},
	{0x15, 0xA2, 0x11},
	{0x15, 0xA3, 0x0B},
	{0x15, 0xA4, 0x05},
	{0x15, 0xA5, 0x08},
	{0x15, 0xA6, 0x06},
	{0x15, 0xA7, 0x04},
	{0x15, 0xA8, 0x09},
	{0x15, 0xA9, 0x0C},
	{0x15, 0xAA, 0x15},
	{0x15, 0xAB, 0x08},
	{0x15, 0xAC, 0x0F},
	{0x15, 0xAD, 0x12},
	{0x15, 0xAE, 0x09},
	{0x15, 0xAF, 0x00},
	{0x15, 0xC0, 0x00},
	{0x15, 0xC1, 0x09},
	{0x15, 0xC2, 0x10},
	{0x15, 0xC3, 0x0C},
	{0x15, 0xC4, 0x05},
	{0x15, 0xC5, 0x08},
	{0x15, 0xC6, 0x06},
	{0x15, 0xC7, 0x04},
	{0x15, 0xC8, 0x08},
	{0x15, 0xC9, 0x0C},
	{0x15, 0xCA, 0x14},
	{0x15, 0xCB, 0x08},
	{0x15, 0xCC, 0x0F},
	{0x15, 0xCD, 0x11},
	{0x15, 0xCE, 0x09},
	{0x15, 0xCF, 0x00},
	{0x39, 0x06, 0x00, {0xFF, 0xFF, 0x98, 0x06, 0x04, 0x06}},
	{0x15, 0x00, 0x20},
	{0x15, 0x01, 0x0A},
	{0x15, 0x02, 0x00},
	{0x15, 0x03, 0x00},
	{0x15, 0x04, 0x01},
	{0x15, 0x05, 0x01},
	{0x15, 0x06, 0x98},
	{0x15, 0x07, 0x06},
	{0x15, 0x08, 0x01},
	{0x15, 0x09, 0x80},
	{0x15, 0x0A, 0x00},
	{0x15, 0x0B, 0x00},
	{0x15, 0x0C, 0x01},
	{0x15, 0x0D, 0x01},
	{0x15, 0x0E, 0x05},
	{0x15, 0x0F, 0x00},
	{0x15, 0x10, 0xF0},
	{0x15, 0x11, 0xF4},
	{0x15, 0x12, 0x01},
	{0x15, 0x13, 0x00},
	{0x15, 0x14, 0x00},
	{0x15, 0x15, 0xC0},
	{0x15, 0x16, 0x08},
	{0x15, 0x17, 0x00},
	{0x15, 0x18, 0x00},
	{0x15, 0x19, 0x00},
	{0x15, 0x1A, 0x00},
	{0x15, 0x1B, 0x00},
	{0x15, 0x1C, 0x00},
	{0x15, 0x1D, 0x00},
	{0x15, 0x20, 0x01},
	{0x15, 0x21, 0x23},
	{0x15, 0x22, 0x45},
	{0x15, 0x23, 0x67},
	{0x15, 0x24, 0x01},
	{0x15, 0x25, 0x23},
	{0x15, 0x26, 0x45},
	{0x15, 0x27, 0x67},
	{0x15, 0x30, 0x11},
	{0x15, 0x31, 0x11},
	{0x15, 0x32, 0x00},
	{0x15, 0x33, 0xEE},
	{0x15, 0x34, 0xFF},
	{0x15, 0x35, 0xBB},
	{0x15, 0x36, 0xAA},
	{0x15, 0x37, 0xDD},
	{0x15, 0x38, 0xCC},
	{0x15, 0x39, 0x66},
	{0x15, 0x3A, 0x77},
	{0x15, 0x3B, 0x22},
	{0x15, 0x3C, 0x22},
	{0x15, 0x3D, 0x22},
	{0x15, 0x3E, 0x22},
	{0x15, 0x3F, 0x22},
	{0x15, 0x40, 0x22},
	{0x39, 0x06, 0x00, {0xFF, 0xFF, 0x98, 0x06, 0x04, 0x07}},
	{0x15, 0x17, 0x22},
	{0x15, 0x02, 0x77},
	{0x15, 0x26, 0xB2},
	{0x39, 0x06, 0x00, {0xFF, 0xFF, 0x98, 0x06, 0x04, 0x00}},
	{0x15, 0x3A, 0x70},
	{0x05, 0x11, 0x00},
	//  {0x05, 0x29, 0x00}, display on
};

struct kd050hdfia019 {
	struct device *dev;
	unsigned int power;
	unsigned int id;

	struct lcd_device *ld;
	struct backlight_device *bd;

	struct mipi_dsim_lcd_device *dsim_dev;
	struct kd050hdfia019_platform_data *ddi_pd;

};

#define lcd_to_master(a)    (a->dsim_dev->master)
#define lcd_to_master_ops(a)    ((lcd_to_master(a))->master_ops)

static void panel_dev_sleep_out(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_cmd_packet data_to_send = {0x05, 0x11, 0x00};

	ops->cmd_write(lcd_to_master(lcd), data_to_send);
}

static void panel_dev_display_on(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_cmd_packet data_to_send = {0x05, 0x29, 0x00};

	ops->cmd_write(lcd_to_master(lcd), data_to_send);
}

static void panel_dev_panel_condition_setting(struct panel_dev *lcd)
{
	int  i;
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	for (i = 0; i < ARRAY_SIZE(kd050hdfia019_cmd_list); i++) {
		ops->cmd_write(dsi,  kd050hdfia019_cmd_list[i]);
	}

}

static int panel_dev_ioctl(struct mipi_dsim_lcd_device *dsim_dev, int cmd)
{
	return 0;
}
static void panel_dev_set_sequence(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct panel_dev *lcd = dev_get_drvdata(&dsim_dev->dev);

	/* panel reset ... */

	panel_dev_panel_condition_setting(lcd);

	panel_dev_sleep_out(lcd);

	msleep(120);
	panel_dev_display_on(lcd);

	lcd->power = FB_BLANK_UNBLANK;
}
static int panel_dev_probe(struct mipi_dsim_lcd_device *dsim_dev)
{
	panel->dsim_dev = dsim_dev;
	dev_set_drvdata(&dsim_dev->dev, panel);
	return 0;
}
static struct mipi_dsim_lcd_driver panel_dev_dsim_ddi_driver = {
	.name = "kd050hdfia019-lcd",
	.id = 0,

	//  .power_on = panel_dev_power_on,
	.set_sequence = panel_dev_set_sequence,
	.ioctl        = panel_dev_ioctl,
	.probe = panel_dev_probe,
};

struct mipi_dsim_lcd_device panel_dev_device = {
	.name       = "kd050hdfia019-lcd",
	.id = 0,
	.platform_data = NULL,
};

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
	.name                   = "kd050hdfia019-lcd",
	.refresh                = 0,
	.xres                   = 480,
	.yres                   = 854,
	.pixclock               = (30000),//KHz
	.left_margin            = 40,
	.right_margin           = 50,
	.upper_margin           = 16,
	.lower_margin           = 4,

	.hsync_len              = 8,
	.vsync_len              = 10,
	.sync                   = FB_SYNC_HOR_HIGH_ACT & FB_SYNC_VERT_HIGH_ACT,
	.vmode                  = FB_VMODE_NONINTERLACED,
	.flag                   = 0,
};

struct jzdsi_data jzdsi_pdata = {
	.modes = &panel_modes,
	.video_config.no_of_lanes = 2,
	.video_config.virtual_channel = 0,
	.video_config.color_coding = COLOR_CODE_24BIT,
	.video_config.video_mode = VIDEO_BURST_WITH_SYNC_PULSES,
	.video_config.receive_ack_packets = 0,
	.video_config.is_18_loosely = 0,
	.video_config.data_en_polarity = 1,
	.video_config.byte_clock = 0, // driver will auto calculate byte_clock.
	.video_config.byte_clock_coef = MIPI_PHY_BYTE_CLK_COEF_MUL3_DIV2, //for auto calculate byte clock

	.dsi_config.max_lanes = 4,
	.dsi_config.max_hs_to_lp_cycles = 100,
	.dsi_config.max_lp_to_hs_cycles = 40,
	.dsi_config.max_bta_cycles = 4095,
	.dsi_config.color_mode_polarity = 1,
	.dsi_config.shut_down_polarity = 1,
	.dsi_config.max_bps = 650,
	.bpp_info = 24,
};

static struct tft_config kd050hdfia019_cfg = {
	.de_dl = 0,
	.sync_dl = 0,
	.color_even = TFT_LCD_COLOR_EVEN_RGB,
	.color_odd = TFT_LCD_COLOR_ODD_RGB,
	.mode = TFT_LCD_MODE_PARALLEL_888,
};

struct lcd_panel lcd_panel = {
	.name = "kd050hdfia019",
	.num_modes = 1,
	.modes = &panel_modes,
	.bpp = 24,
	.width = 62,
	.height = 110,

	.lcd_type = LCD_TYPE_MIPI_TFT,

	.tft_config = &kd050hdfia019_cfg,
	.dsi_pdata = &jzdsi_pdata,

	.dither_enable = 0,
	.dither.dither_red = 0,
	.dither.dither_green = 0,
	.dither.dither_blue = 0,

	.ops = &panel_ops,
};

#define POWER_IS_ON(pwr)        ((pwr) <= FB_BLANK_NORMAL)
static int panel_set_power(struct lcd_device *lcd, int power)
{
	struct panel_dev *panel = lcd_get_data(lcd);

	if (POWER_IS_ON(power) && !POWER_IS_ON(panel->power)) {
	}
	if (!POWER_IS_ON(power) && POWER_IS_ON(panel->power)) {
	}

	panel->power = power;
	return 0;
}

static int panel_get_power(struct lcd_device *lcd)
{
	struct panel_dev *panel = lcd_get_data(lcd);

	return panel->power;
}

/**
* @ pannel_kd050hdfia019_lcd_ops, register to kernel common backlight/lcd.c framworks.
*/
static struct lcd_ops panel_lcd_ops = {
	.set_power = panel_set_power,
	.get_power = panel_get_power,
};

static int of_panel_parse(struct device *dev)
{
	return 0;
}
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

	panel->lcd = lcd_device_register("panel_lcd", &pdev->dev, panel, &panel_lcd_ops);
	if (IS_ERR_OR_NULL(panel->lcd)) {
		dev_err(&pdev->dev, "Error register lcd!\n");
		ret = -EINVAL;
		goto err_of_parse;
	}

	/* TODO: should this power status sync from uboot */
	panel->power = FB_BLANK_POWERDOWN;
	panel_set_power(panel->lcd, FB_BLANK_UNBLANK);

	mipi_dsi_register_lcd_device(&panel_dev_device);
	mipi_dsi_register_lcd_driver(&panel_dev_dsim_ddi_driver);

	ret = ingenicfb_register_panel(&lcd_panel);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register lcd panel!\n");
		goto err_lcd_register;
	}

	return 0;

err_lcd_register:
	lcd_device_unregister(panel->lcd);
err_of_parse:
	kfree(panel);
	return ret;
}

static int panel_remove(struct platform_device *pdev)
{
	struct panel_dev *panel = dev_get_drvdata(&pdev->dev);

	panel_set_power(panel->lcd, FB_BLANK_POWERDOWN);
	return 0;
}

#ifdef CONFIG_PM
static int panel_suspend(struct device *dev)
{
	struct panel_dev *panel = dev_get_drvdata(dev);

	panel_set_power(panel->lcd, FB_BLANK_POWERDOWN);
	return 0;
}

static int panel_resume(struct device *dev)
{
	struct panel_dev *panel = dev_get_drvdata(dev);

	panel_set_power(panel->lcd, FB_BLANK_UNBLANK);
	return 0;
}

static const struct dev_pm_ops panel_pm_ops = {
	.suspend = panel_suspend,
	.resume = panel_resume,
};
#endif
static const struct of_device_id panel_of_match[] = {
	{ .compatible = "ingenic,kd050hdfia019", },
	{},
};

static struct platform_driver panel_driver = {
	.probe      = panel_probe,
	.remove     = panel_remove,
	.driver     = {
		.name   = "kd050hdfia019",
		.of_match_table = panel_of_match,
#ifdef CONFIG_PM
		.pm = &panel_pm_ops,
#endif
	},
};

module_platform_driver(panel_driver);
MODULE_LICENSE("GPL");
