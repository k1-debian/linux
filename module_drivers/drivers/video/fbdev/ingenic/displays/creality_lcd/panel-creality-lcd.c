/*
 *
 * Copyright (C) 2016 Ingenic Semiconductor Inc.
 *
 * This program is free software, you can redistribute it and/or modify it
 *
 * under the terms of the GNU General Public License version 2 as published by
 *
 * the Free Software Foundation.
 */

#include "panel-creality-lcd.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/property.h> 
#include <linux/pwm_backlight.h>
#include <linux/delay.h>
#include <linux/lcd.h>

#include "../../include/ingenicfb.h"

struct gpio_spi {
	struct gpio_desc *sdo;
	struct gpio_desc *sdi;
	struct gpio_desc *sck;
	struct gpio_desc *cs;
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
	struct gpio_desc *vdd_en;
	struct gpio_desc *rst;
	struct gpio_spi spi;
};

static struct panel_dev *panel;

static struct creality_panel_detect panel_models_list[] = {
	{
		.name = "gc9503cv_ue_480_800",
		.detect = &lcd_gc9503cv_ue_detect,
		.init = &lcd_gc9503cv_ue_init
	},
	{
		.name = "st7701_tjc_480_800",
		.detect = &lcd_st7701_tjc_detect,
		.init = &lcd_st7701_tjc_init
	},	
	{
		.name = "pj_st7701_9bit_spi",
		.detect = &lcd_st7701_pj_detect,
		.init = &lcd_st7701_pj_init
	},
	{
		.name = "dc_st7701_9bit_spi",
		.detect = &lcd_st7701_dc_detect,
		.init = &lcd_st7701_dc_init
	},
	{
		.name = "st7701_tjc_lc_480_800",
		.detect = &lcd_st7701_tjc_lc_detect,
		.init = &lcd_st7701_tjc_lc_init
	},	
	{
		.name = "pj_lc_st7701_9bit_spi",
		.detect = &lcd_st7701_pj_lc_detect,
		.init = &lcd_st7701_pj_lc_init
	},
	{
		.name = "dc_lc_st7701_9bit_spi",
		.detect = &lcd_st7701_dc_lc_detect,
		.init = &lcd_st7701_dc_lc_init
	}
};

static struct creality_panel_detect *detected_panel;

#define RESET(n)\
	gpiod_set_value(panel->rst, n)

#define CS(n)\
	gpiod_set_value(panel->spi.cs, n)

#define SCK(n)\
	gpiod_set_value(panel->spi.sck, n)

#define SDO(n)\
	gpiod_set_value(panel->spi.sdo, n)

#define SDI()\
	(gpiod_get_value(panel->spi.sdi) == 0 ? 0 : 1)

void SPI_Send9Bit(unsigned int i)
{
	for (unsigned char n = 0; n < 9; n++) {
		if (i & 0x100) {
			SDO(1);
		} else {
			SDO(0);
		}
		i = i << 1;

		SCK(1);
		udelay(10);
		SCK(0);
		udelay(10);
	}
}

void SPI_SendCommand(unsigned char i)
{
	CS(1);
	SPI_Send9Bit((unsigned int)i & 0xff);
	CS(0);
}

void SPI_SendData(unsigned char i)
{
	CS(1);
	SPI_Send9Bit(((unsigned int)i & 0xff) | 0x100);
	CS(0);
}

void SPI_ReadData(unsigned char reg, unsigned char* dest, unsigned char len)
{
	CS(1);
	
	SPI_Send9Bit((unsigned int)reg & 0xff);

	gpiod_direction_input(panel->spi.sdo); // Output to High-Z
	udelay(20);

	if(len > 1) {
		// Dummy clock cycle
		SCK(1);
		udelay(10);
		SCK(0);
		udelay(10);
	}

	for(int i = 0;i < len; i++) {
		dest[i] = 0;
		for(int bit = 0; bit < 8; bit ++){
			dest[i] = (dest[i] << 1) | SDI();

			SCK(1);
			udelay(10);
			SCK(0);
			udelay(10);			
		}
	}
	CS(0);

	gpiod_direction_output(panel->spi.sdo, 0); // Output to normal output
	udelay(20);
}

void Reset_IC(void)
{
	RESET(0);
	udelay(10000);
	RESET(1);
	udelay(10);
	RESET(0);
	udelay(120000);  
}

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

static struct fb_videomode panel_modes[] = {
	[0] = {
		.name                   = "480x800",
		.refresh                = 50,
		.xres                   = 480,
		.yres                   = 800,
		.pixclock               = 0,
		.left_margin            = 10,
		.right_margin           = 10,
		.upper_margin           = 20,
		.lower_margin           = 20,
		.hsync_len              = 20,
		.vsync_len              = 4,
		.sync                   = FB_SYNC_HOR_HIGH_ACT & FB_SYNC_VERT_HIGH_ACT,
		.vmode                  = FB_VMODE_NONINTERLACED,
		.flag                   = 0,
	},
};

static struct tft_config creality_lcd_cfg = {
	.pix_clk_inv = 1,
	.de_dl = 0,
	.sync_dl = 0,
	.color_even = TFT_LCD_COLOR_EVEN_RGB,
	.color_odd = TFT_LCD_COLOR_ODD_RGB,
	.mode = TFT_LCD_MODE_PARALLEL_565,
};

struct lcd_panel lcd_panel = {
	.name = "creality_lcd",
	.num_modes = ARRAY_SIZE(panel_modes),
	.modes = panel_modes,
	.bpp = 16,
	.width = 55,
	.height = 94,

	.lcd_type = LCD_TYPE_TFT,

	.tft_config = &creality_lcd_cfg,
	.dither_enable = 0,
	.dither.dither_red = 0,
	.dither.dither_green = 0,
	.dither.dither_blue = 0,
	.ops = &panel_ops,
};

static void detect_panel(struct lcd_device *lcd)
{
	struct device *dev = &(lcd->dev);

	int panel_models_count = sizeof(panel_models_list) / sizeof(panel_models_list[0]);
	for(int i = 0; i < panel_models_count; i++) {
		if(panel_models_list[i].detect()) {
			detected_panel = &panel_models_list[i];
			dev_notice(dev,"LCD model detected: %s", detected_panel->name);
			return;
		}
	}	
	
	dev_err(dev, "LCD model detection failed.");
}

#define POWER_IS_ON(pwr)        ((pwr) <= FB_BLANK_NORMAL)
static int panel_set_power(struct lcd_device *lcd, int power)
{
	struct panel_dev *panel = lcd_get_data(lcd);
	struct gpio_desc *vdd_en = panel->vdd_en;

	if (POWER_IS_ON(power) && !POWER_IS_ON(panel->power)) {
		gpiod_set_value(vdd_en, 1);
		Reset_IC();

		if(detected_panel == NULL) {
			detect_panel(lcd);
		}
		
		if(detected_panel) {
			detected_panel->init();
		}
	}
	if (!POWER_IS_ON(power) && POWER_IS_ON(panel->power)) {
		gpiod_set_value(vdd_en, 0);
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

static int of_panel_parse(struct device *dev)
{
	struct panel_dev *panel = dev_get_drvdata(dev);
	struct fwnode_handle *fwnode = dev->fwnode;
	enum gpiod_flags dflags = GPIOD_ASIS; 

	int ret = 0;

	panel->vdd_en = fwnode_gpiod_get_index(fwnode, "ingenic,vdd-en", 0, dflags, NULL);
	if (IS_ERR(panel->vdd_en)) {
		dev_err(dev, "Failed to initialize vdd_en pin!\n");
		return ret;
	}

	panel->rst = fwnode_gpiod_get_index(fwnode, "ingenic,rst", 0, dflags, NULL);
	if (IS_ERR(panel->rst)) {
		dev_err(dev, "Failed to initialize rst pin!\n");
		goto err_request_rst;
	}

	panel->spi.sdi = fwnode_gpiod_get_index(fwnode, "ingenic,lcd-sdi", 0, dflags, NULL);
	if (IS_ERR(panel->spi.sdi)) {
		dev_err(dev, "Failed to initialize sdi pin!\n");
		goto err_request_sdi;
	}  

	panel->spi.sdo = fwnode_gpiod_get_index(fwnode, "ingenic,lcd-sdo", 0, dflags, NULL);
	if (IS_ERR(panel->spi.sdo)) {
		dev_err(dev, "Failed to initialize sdo pin!\n");
		goto err_request_sdo;
	}

	panel->spi.sck = fwnode_gpiod_get_index(fwnode, "ingenic,lcd-sck", 0, dflags, NULL);
	if (IS_ERR(panel->spi.sck)) {
		dev_err(dev, "Failed to initialize sck pin!\n");
		goto err_request_sck;
	}

	panel->spi.cs = fwnode_gpiod_get_index(fwnode, "ingenic,lcd-cs", 0, dflags, NULL);
	if (IS_ERR(panel->spi.cs)) {
		dev_err(dev, "Failed to initialize cs pin!\n");
		goto err_request_cs;
	}

	gpiod_direction_output(panel->vdd_en, 0);
	gpiod_direction_output(panel->rst, 0);
	gpiod_direction_input(panel->spi.sdi);
	gpiod_direction_output(panel->spi.sdo, 0);
	gpiod_direction_output(panel->spi.sck, 0);
	gpiod_direction_output(panel->spi.cs, 0);

	return 0;
err_request_cs:
	gpiod_put(panel->spi.cs);
err_request_sck:
	gpiod_put(panel->spi.sck);
err_request_sdo:
	gpiod_put(panel->spi.sdo);
err_request_sdi:
	gpiod_put(panel->spi.sdi);
err_request_rst:
	gpiod_put(panel->rst);
	return ret;
}
/**
* @panel_probe
*
*   1. Register to ingenicfb.
*   2. Register to lcd.
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
		dev_err(&pdev->dev, "Failed to alloc memory!");
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
	{ .compatible = "ingenic,creality_lcd", },
	{},
};

static struct platform_driver panel_driver = {
	.probe      = panel_probe,
	.remove     = panel_remove,
	.driver     = {
		.name   = "creality_lcd",
		.of_match_table = panel_of_match,
#ifdef CONFIG_PM
		.pm = &panel_pm_ops,
#endif
	},
};

module_platform_driver(panel_driver);
MODULE_LICENSE("GPL");
