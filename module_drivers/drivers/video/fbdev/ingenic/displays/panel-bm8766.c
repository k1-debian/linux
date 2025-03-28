/*
 * ingenic_bsp/chip-x2000/fpga/dpu/bm8766.c
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
#include <soc/gpio.h>

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
	struct board_gpio vdd_en;
	struct board_gpio bl;
	struct board_gpio rst;
	struct board_gpio pwm;
};

static struct panel_dev *panel;

static void bm8766_power_on(struct lcd_panel *ppanel)
{
	struct panel_dev *panel_bm8766 = dev_get_drvdata(panel->dev);

	gpio_direction_output(panel_bm8766->bl.gpio, panel_bm8766->bl.active_level);
}

static void bm8766_power_off(struct lcd_panel *ppanel)
{
	struct panel_dev *panel_bm8766 = dev_get_drvdata(panel->dev);

	gpio_direction_output(panel_bm8766->bl.gpio, !panel_bm8766->bl.active_level);
	return;
}

static struct lcd_panel_ops bm8766_ops = {
	.enable  = (void *)bm8766_power_on,
	.disable = (void *)bm8766_power_off,
};

static struct fb_videomode jzfb_bm8766_videomode[] = {
	[0] = {
		.name = "800x480",
		.refresh = 30,
		//.refresh = 60,
		.xres = 800,
		.yres = 480,
		.pixclock = (16632),    //KHz
		//.pixclock = (33264),
		.left_margin = 88,
		.right_margin = 40,
		.upper_margin = 8,
		.lower_margin = 35,
		.hsync_len = 128,
		.vsync_len = 2,
		.sync = ~FB_SYNC_HOR_HIGH_ACT & ~FB_SYNC_VERT_HIGH_ACT,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
};

static struct tft_config bm8766_cfg = {
	.pix_clk_inv = 0,
	.de_dl = 0,
	.sync_dl = 0,
	.color_even = TFT_LCD_COLOR_EVEN_BGR,
	.color_odd = TFT_LCD_COLOR_ODD_BGR,
	.mode = TFT_LCD_MODE_PARALLEL_666,
};

struct lcd_panel lcd_panel = {
	.name = "bm8766",
	.num_modes = ARRAY_SIZE(jzfb_bm8766_videomode),
	.modes = jzfb_bm8766_videomode,
	.lcd_type = LCD_TYPE_TFT,
	.bpp = 24,
	.width = 800,
	.height = 480,

	.tft_config = &bm8766_cfg,

	/*T40 RGB real line is 666, so, for RGB888, we discard low 2 bytes, config as follows:*/
	.dither_enable = 1,
	.dither.dither_red = 1,
	.dither.dither_green = 1,
	.dither.dither_blue = 1,

	.ops = &bm8766_ops,
};

#if 0
static int panel_update_status(struct backlight_device *bd)
{
	struct panel_dev *panel = dev_get_drvdata(&bd->dev);
	int brightness = bd->props.brightness;
	unsigned int i;
	int pulse_num = MAX_BRIGHTNESS_STEP - brightness / CONVERT_FACTOR - 1;

	if (FB_BLANK_POWERDOWN == bd->pros.fb_blank) {
		return 0;
	}

	if (bd->props.state & BL_CORE_SUSPEND) {
		brightness = 0;
	}

	if (brightness) {
		gpio_direction_output(panel->pwm.gpio, 0);
		udelay(5000);
		gpio_direction_output(panel->pwm.gpio, 1);
		udelay(100);

		for (i = pulse_num; i > 0 ; i--) {
			gpio_direction_output(panel->pwm.gpio, 0);
			udelay(1);
			gpio_direction_output(panel->pwm.gpio, 1);
			udelay(3);
		}
	} else {
		gpio_direction_output(panel->pwm.gpio, 0);
	}

	return 0;
}

static struct backlight_ops panel_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = panel_update_status,
};
#endif

#define RESET(n)\
	gpio_direction_output(panel->rst.gpio, n)
#define POWER_IS_ON(pwr)     ((pwr) <= FB_BLANK_NORMAL)
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
 * @ panel_bm8766_lcd_ops, register to kernel common backlight/lcd.c frameworks.
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

	panel->bl.gpio = of_get_named_gpio_flags(np, "gpios", 0, &flags);
	if (gpio_is_valid(panel->bl.gpio)) {
		panel->bl.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = devm_gpio_request(dev, panel->bl.gpio, "backlight");
		if (ret < 0) {
			dev_err(dev, "Failed to request backlight pin!\n");
			return ret;
		}
	} else {
		dev_warn(dev, "invalid gpio backlight.gpio: %d\n", panel->bl.gpio);
	}

	return 0;
}

/**
 * @panel_probe
 *
 *  1. register to ingenicfb.
 *  2. register to lcd.
 *  3. register to backlight if possible.
 *
 *  @pdev
 *
 *  @return -
 * */
static int panel_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct backlight_properties props;   //backlight properties

	memset(&props, 0, sizeof(props));
	panel = kzalloc(sizeof(struct panel_dev), GFP_KERNEL);
	if (NULL == panel) {
		dev_err(&pdev->dev, "Failed to alloc memory!");
		return -ENOMEM;
	}
	panel->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, panel);

	/* panel pinctrl parse */
	ret = of_panel_parse(&pdev->dev);
	if (ret < 0) {
		goto err_of_parse;
	}

	panel->lcd = lcd_device_register("panel_lcd", &pdev->dev, panel, &panel_lcd_ops);
	if (IS_ERR_OR_NULL(panel->lcd)) {
		dev_err(&pdev->dev, "Error register lcd!");
		ret = -EINVAL;
		goto err_of_parse;
	}

	/* TODO: should this power status sync from uboot */
	panel->power = FB_BLANK_POWERDOWN;
	panel_set_power(panel->lcd, FB_BLANK_UNBLANK);

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
	{ .compatible = "ingenic,bm8766", },
	{ .compatible = "bm8766", },
	{},
};

static struct platform_driver panel_driver = {
	.probe = panel_probe,
	.remove = panel_remove,
	.driver = {
		.name = "bm8766",
		.of_match_table = panel_of_match,
#ifdef CONFIG_PM
		.pm = &panel_pm_ops,
#endif
	},
};

module_platform_driver(panel_driver);
