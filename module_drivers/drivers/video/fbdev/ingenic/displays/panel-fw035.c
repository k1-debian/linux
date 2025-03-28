/*
 * driver/video/fbdev/ingenic/fb_v12/displays/fw035.c
 *
 * Copyright (C) 2016 Ingenic Semiconductor Inc.
 *
 * Author:clwang<chunlei.wang@ingenic.com>
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
	struct gpio_desc *cs;
	struct gpio_desc *rst;
	struct gpio_desc *vdd_en;
	struct gpio_desc *rd;
	struct gpio_desc *lcd_pwm;
};

static struct smart_lcd_data_table fw035_data_table[] = {
	/* LCD init code */
	{SMART_CONFIG_CMD, 0xE0},
	{SMART_CONFIG_PRM, 0x00},
	{SMART_CONFIG_PRM, 0x07},
	{SMART_CONFIG_PRM, 0x10},
	{SMART_CONFIG_PRM, 0x09},
	{SMART_CONFIG_PRM, 0x17},
	{SMART_CONFIG_PRM, 0x0B},
	{SMART_CONFIG_PRM, 0x40},
	{SMART_CONFIG_PRM, 0x8A},
	{SMART_CONFIG_PRM, 0x4B},
	{SMART_CONFIG_PRM, 0x0A},
	{SMART_CONFIG_PRM, 0x0D},
	{SMART_CONFIG_PRM, 0x0F},
	{SMART_CONFIG_PRM, 0x15},
	{SMART_CONFIG_PRM, 0x16},
	{SMART_CONFIG_PRM, 0x0F},

	{SMART_CONFIG_CMD, 0xE1},
	{SMART_CONFIG_PRM, 0x00},
	{SMART_CONFIG_PRM, 0x1A},
	{SMART_CONFIG_PRM, 0x1B},
	{SMART_CONFIG_PRM, 0x02},
	{SMART_CONFIG_PRM, 0x0D},
	{SMART_CONFIG_PRM, 0x05},
	{SMART_CONFIG_PRM, 0x30},
	{SMART_CONFIG_PRM, 0x35},
	{SMART_CONFIG_PRM, 0x43},
	{SMART_CONFIG_PRM, 0x02},
	{SMART_CONFIG_PRM, 0x0A},
	{SMART_CONFIG_PRM, 0x09},
	{SMART_CONFIG_PRM, 0x32},
	{SMART_CONFIG_PRM, 0x36},
	{SMART_CONFIG_PRM, 0x0F},

	{SMART_CONFIG_CMD, 0xB1},
	{SMART_CONFIG_PRM, 0xA0},
	{SMART_CONFIG_PRM, 0x11},

	{SMART_CONFIG_CMD, 0xB4},
	{SMART_CONFIG_PRM, 0x02},

	{SMART_CONFIG_CMD, 0xC0},
	{SMART_CONFIG_PRM, 0x17},
	{SMART_CONFIG_PRM, 0x15},

	{SMART_CONFIG_CMD, 0xC1},
	{SMART_CONFIG_PRM, 0x41},

	{SMART_CONFIG_CMD, 0xC5},
	{SMART_CONFIG_PRM, 0x00},
	{SMART_CONFIG_PRM, 0x0A},
	{SMART_CONFIG_PRM, 0x80},

	{SMART_CONFIG_CMD, 0xB6},
	{SMART_CONFIG_PRM, 0x02},

	{SMART_CONFIG_CMD, 0x36},
	{SMART_CONFIG_PRM, 0x48},

	{SMART_CONFIG_CMD, 0x3A},
	{SMART_CONFIG_PRM, 0x56},

	{SMART_CONFIG_CMD, 0xE9},
	{SMART_CONFIG_PRM, 0x00},

	{SMART_CONFIG_CMD, 0xF7},
	{SMART_CONFIG_PRM, 0xA9},
	{SMART_CONFIG_PRM, 0x51},
	{SMART_CONFIG_PRM, 0x2C},
	{SMART_CONFIG_PRM, 0x82},

	{SMART_CONFIG_CMD, 0x35},    //TE on
	{SMART_CONFIG_PRM, 0x00},

	{SMART_CONFIG_CMD, 0x11},
	{SMART_CONFIG_UDELAY, 120000},
	{SMART_CONFIG_CMD, 0x29},
	{SMART_CONFIG_CMD, 0x2c},
};

static struct fb_videomode panel_modes[] = {
	[0] = {
		.name = "320x480",
		.refresh = 60,
		.xres = 320,
		.yres = 480,
		.pixclock = (18432), //KHz
		.left_margin = 0,
		.right_margin = 0,
		.upper_margin = 0,
		.lower_margin = 0,
		.hsync_len = 0,
		.vsync_len = 0,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
};

struct smart_config fw035_cfg = {
	.te_anti_jit = 1,
	.te_md = 0,
	.te_switch = 1,
	.dc_md = 0,
	.wr_md = 1,
	.te_dp = 1,
	.smart_type = SMART_LCD_TYPE_8080,
	.pix_fmt = SMART_LCD_FORMAT_888,
	.dwidth = SMART_LCD_DWIDTH_8_BIT,
	.cwidth = SMART_LCD_CWIDTH_8_BIT,
	.bus_width = 8,

	.write_gram_cmd = 0x2c,
	.data_table = fw035_data_table,
	.length_data_table = ARRAY_SIZE(fw035_data_table),
};

static struct lcd_device *lcd_power = NULL;

/* SGM3146 supports 16 brightness step */
#define MAX_BRIGHTNESS_STEP     16
/* System support 256 brightness step */
#define CONVERT_FACTOR          (256/MAX_BRIGHTNESS_STEP)

static int panel_update_status(struct backlight_device *bd)
{
	struct panel_dev *panel = dev_get_drvdata(&bd->dev);
	int brightness = bd->props.brightness;
	unsigned int i;
	int pulse_num = MAX_BRIGHTNESS_STEP - brightness / CONVERT_FACTOR - 1;

	if (bd->props.fb_blank == FB_BLANK_POWERDOWN) {
		return 0;
	}

	if (bd->props.state & BL_CORE_SUSPENDED) {
		brightness = 0;
	}

	if (brightness && panel->lcd_pwm) {
		gpiod_set_value_cansleep(panel->lcd_pwm, 0);
		udelay(5000);
		gpiod_set_value_cansleep(panel->lcd_pwm, 1);
		udelay(100);

		for (i = pulse_num; i > 0; i--) {
			gpiod_set_value_cansleep(panel->lcd_pwm, 0);
			udelay(1);
			gpiod_set_value_cansleep(panel->lcd_pwm, 1);
			udelay(3);
		}
	} else if (panel->lcd_pwm) {
		gpiod_set_value_cansleep(panel->lcd_pwm, 0);
	}

	return 0;
}

static struct backlight_ops panel_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = panel_update_status,
};

static void panel_power_reset(struct gpio_desc *rst)
{
	gpiod_set_value_cansleep(rst, 0);
	mdelay(120);
	gpiod_set_value_cansleep(rst, 1);
	mdelay(120);
}

#define POWER_IS_ON(pwr)        ((pwr) <= FB_BLANK_NORMAL)
static int panel_set_power(struct lcd_device *lcd, int power)
{
	struct panel_dev *panel = lcd_get_data(lcd);
	if (POWER_IS_ON(power) && !POWER_IS_ON(panel->power)) {
		if (panel->vdd_en) {
			gpiod_set_value_cansleep(panel->vdd_en, 1);
		}
		if (panel->lcd_pwm) {
			gpiod_set_value_cansleep(panel->lcd_pwm, 1);
		}
		if (panel->rd) {
			gpiod_set_value_cansleep(panel->rd, 1);
		}
		gpiod_set_value_cansleep(panel->cs, 1);
		gpiod_set_value_cansleep(panel->rst, 1);
		mdelay(10);
		panel_power_reset(panel->rst);
		gpiod_set_value_cansleep(panel->cs, 0);
	}
	if (!POWER_IS_ON(power) && POWER_IS_ON(panel->power)) {
		gpiod_set_value_cansleep(panel->cs, 0);
		gpiod_set_value_cansleep(panel->rst, 0);
		gpiod_set_value_cansleep(panel->rd, 0);
		gpiod_set_value_cansleep(panel->vdd_en, 0);
	}

	panel->power = power;
	return 0;
}

static int panel_get_power(struct lcd_device *lcd)
{
	struct panel_dev *panel = lcd_get_data(lcd);

	return panel->power;
}

static struct lcd_ops panel_lcd_ops = {
	.set_power = panel_set_power,
	.get_power = panel_get_power,
};

static int panel_power_off(struct lcd_panel *lcd_panel)
{
	if (lcd_power) {
		panel_set_power(lcd_power, FB_BLANK_POWERDOWN);
	}

	return 0;
}

static int panel_power_on(struct lcd_panel *lcd_panel)
{
	if (lcd_power) {
		panel_set_power(lcd_power, FB_BLANK_UNBLANK);
	}

	return 0;
}

static struct lcd_panel_ops fw035_ops = {
	.disable  = (void *)panel_power_off,
	.enable = (void *)panel_power_on,
};

struct lcd_panel lcd_panel = {
	.name = "fw035",
	.num_modes = ARRAY_SIZE(panel_modes),
	.modes = panel_modes,
	.lcd_type = LCD_TYPE_SLCD,
	.width = 320,
	.height = 480,

	.smart_config = &fw035_cfg,

	.dither_enable = 0,
	.dither.dither_red = 0,
	.dither.dither_green = 0,
	.dither.dither_blue = 0,

	.ops = &fw035_ops,
};

static int of_panel_parse(struct device *dev)
{
	struct panel_dev *panel = dev_get_drvdata(dev);
	int ret = 0;
	panel->cs = devm_gpiod_get_optional(dev, "ingenic,cs", GPIOD_OUT_LOW);
	if (IS_ERR(panel->cs)) {
		ret = PTR_ERR(panel->cs);
		dev_warn(dev, "can't request lcd cs gpio, ret: %d\n", ret);
	}

	panel->rd = devm_gpiod_get_optional(dev, "ingenic,rd", GPIOD_OUT_LOW);
	if (IS_ERR(panel->rd)) {
		ret = PTR_ERR(panel->rd);
		dev_warn(dev, "can't request rd gpio, ret: %d\n", ret);
	}

	panel->vdd_en = devm_gpiod_get_optional(dev, "ingenic,vdd-en", GPIOD_OUT_LOW);
	if (IS_ERR(panel->vdd_en)) {
		ret = PTR_ERR(panel->vdd_en);
		dev_warn(dev, "can't request lcd vdd gpio, ret: %d\n", ret);
	}

	panel->rst = devm_gpiod_get_optional(dev, "ingenic,rst", GPIOD_OUT_LOW);
	if (IS_ERR(panel->rst)) {
		ret = PTR_ERR(panel->rst);
		dev_err(dev, "can't request lcd rst gpio, ret: %d\n", ret);
		return ret;
	}
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
	struct panel_dev *panel;
	struct backlight_properties props;

	memset(&props, 0, sizeof(props));
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

	lcd_power = panel->lcd;

	/* TODO: should this power status sync from uboot */
	panel->power = FB_BLANK_POWERDOWN;
	panel_set_power(panel->lcd, FB_BLANK_UNBLANK);

	props.type = BACKLIGHT_RAW;
	props.max_brightness = 255;
	panel->backlight = backlight_device_register("pwm-backlight.0",
	                   &pdev->dev, panel,
	                   &panel_backlight_ops,
	                   &props);
	if (IS_ERR_OR_NULL(panel->backlight)) {
		dev_err(panel->dev, "failed to register 'pwm-backlight.0'.\n");
		goto err_lcd_register;
	}
	panel->backlight->props.brightness = props.max_brightness;
	backlight_update_status(panel->backlight);
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
	{ .compatible = "ingenic,fw035", },
	{},
};

static struct platform_driver panel_driver = {
	.probe      = panel_probe,
	.remove     = panel_remove,
	.driver     = {
		.name   = "fw035",
		.of_match_table = panel_of_match,
#ifdef CONFIG_PM
		.pm = &panel_pm_ops,
#endif
	},
};

module_platform_driver(panel_driver);
MODULE_LICENSE("GPL");
