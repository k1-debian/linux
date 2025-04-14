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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/property.h> 
#include <linux/pwm_backlight.h>
#include <linux/delay.h>
#include <linux/lcd.h>

#include "../include/ingenicfb.h"

struct gpio_spi {
	struct gpio_desc *sda;
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

#define RESET(n)\
	gpiod_set_value(panel->rst, n)

#define CS(n)\
	gpiod_set_value(panel->spi.cs, n)

#define SCK(n)\
	gpiod_set_value(panel->spi.sck, n)

#define SDA(n)\
	gpiod_set_value(panel->spi.sda, n)

void SPI_SendData9bit(unsigned int i)
{
	unsigned char n;

  CS(1);
	for (n = 0; n < 9; n++) {
		if (i & 0x100) {
			SDA(1);
		} else {
			SDA(0);
		}
		i = i << 1;

		SCK(1);
		udelay(10);
		SCK(0);
		udelay(10);
	}
  CS(0);
}

void Initial_IC(void)
{
	RESET(0);
	udelay(10000);
	RESET(1);
	udelay(10);
	RESET(0);
	udelay(120000);
	SPI_SendData9bit(0x11);
  SPI_SendData9bit(0xff);
  SPI_SendData9bit(0x177);
  SPI_SendData9bit(0x101);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x113);
  SPI_SendData9bit(0xef);
  SPI_SendData9bit(0x108);
  SPI_SendData9bit(0xff);
  SPI_SendData9bit(0x177);
  SPI_SendData9bit(0x101);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x110);
  SPI_SendData9bit(0xc0);
  SPI_SendData9bit(0x163);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0xc1);
  SPI_SendData9bit(0x11c);
  SPI_SendData9bit(0x108);
  SPI_SendData9bit(0xc2);
  SPI_SendData9bit(0x117);
  SPI_SendData9bit(0x102);
  SPI_SendData9bit(0xcc);
  SPI_SendData9bit(0x110);
  SPI_SendData9bit(0xb0);
  SPI_SendData9bit(0x106);
  SPI_SendData9bit(0x110);
  SPI_SendData9bit(0x116);
  SPI_SendData9bit(0x10d);
  SPI_SendData9bit(0x111);
  SPI_SendData9bit(0x106);
  SPI_SendData9bit(0x108);
  SPI_SendData9bit(0x107);
  SPI_SendData9bit(0x108);
  SPI_SendData9bit(0x122);
  SPI_SendData9bit(0x104);
  SPI_SendData9bit(0x114);
  SPI_SendData9bit(0x10f);
  SPI_SendData9bit(0x129);
  SPI_SendData9bit(0x12f);
  SPI_SendData9bit(0x11f);
  SPI_SendData9bit(0xb1);
  SPI_SendData9bit(0x10f);
  SPI_SendData9bit(0x118);
  SPI_SendData9bit(0x11e);
  SPI_SendData9bit(0x10c);
  SPI_SendData9bit(0x10f);
  SPI_SendData9bit(0x106);
  SPI_SendData9bit(0x108);
  SPI_SendData9bit(0x10a);
  SPI_SendData9bit(0x109);
  SPI_SendData9bit(0x124);
  SPI_SendData9bit(0x105);
  SPI_SendData9bit(0x110);
  SPI_SendData9bit(0x111);
  SPI_SendData9bit(0x12a);
  SPI_SendData9bit(0x134);
  SPI_SendData9bit(0x11f);
  SPI_SendData9bit(0xff);
  SPI_SendData9bit(0x177);
  SPI_SendData9bit(0x101);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x111);
  SPI_SendData9bit(0xb0);
  SPI_SendData9bit(0x14d);
  SPI_SendData9bit(0xb1);
  SPI_SendData9bit(0x130);
  SPI_SendData9bit(0xb2);
  SPI_SendData9bit(0x181);
  SPI_SendData9bit(0xb3);
  SPI_SendData9bit(0x180);
  SPI_SendData9bit(0xb5);
  SPI_SendData9bit(0x14e);
  SPI_SendData9bit(0xb7);
  SPI_SendData9bit(0x185);
  SPI_SendData9bit(0xb8);
  SPI_SendData9bit(0x132);
  SPI_SendData9bit(0xbb);
  SPI_SendData9bit(0x103);
  SPI_SendData9bit(0xc1);
  SPI_SendData9bit(0x108);
  SPI_SendData9bit(0xc2);
  SPI_SendData9bit(0x108);
  SPI_SendData9bit(0xd0);
  SPI_SendData9bit(0x188);
  SPI_SendData9bit(0xe0);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x102);
  SPI_SendData9bit(0xe1);
  SPI_SendData9bit(0x106);
  SPI_SendData9bit(0x128);
  SPI_SendData9bit(0x108);
  SPI_SendData9bit(0x128);
  SPI_SendData9bit(0x105);
  SPI_SendData9bit(0x128);
  SPI_SendData9bit(0x107);
  SPI_SendData9bit(0x128);
  SPI_SendData9bit(0x10e);
  SPI_SendData9bit(0x133);
  SPI_SendData9bit(0x133);
  SPI_SendData9bit(0xe2);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0xe3);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x133);
  SPI_SendData9bit(0x133);
  SPI_SendData9bit(0xe4);
  SPI_SendData9bit(0x144);
  SPI_SendData9bit(0x144);
  SPI_SendData9bit(0xe5);
  SPI_SendData9bit(0x111);
  SPI_SendData9bit(0x137);
  SPI_SendData9bit(0x12C);
  SPI_SendData9bit(0x18c);
  SPI_SendData9bit(0x113);
  SPI_SendData9bit(0x139);
  SPI_SendData9bit(0x12C);
  SPI_SendData9bit(0x18c);
  SPI_SendData9bit(0x10d);
  SPI_SendData9bit(0x133);
  SPI_SendData9bit(0x12C);
  SPI_SendData9bit(0x18c);
  SPI_SendData9bit(0x10f);
  SPI_SendData9bit(0x135);
  SPI_SendData9bit(0x12C);
  SPI_SendData9bit(0x18c);
  SPI_SendData9bit(0xe6);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x133);
  SPI_SendData9bit(0x133);
  SPI_SendData9bit(0xe7);
  SPI_SendData9bit(0x144);
  SPI_SendData9bit(0x144);
  SPI_SendData9bit(0xe8);
  SPI_SendData9bit(0x110);
  SPI_SendData9bit(0x136);
  SPI_SendData9bit(0x12C);
  SPI_SendData9bit(0x18c);
  SPI_SendData9bit(0x112);
  SPI_SendData9bit(0x138);
  SPI_SendData9bit(0x12C);
  SPI_SendData9bit(0x18c);
  SPI_SendData9bit(0x10c);
  SPI_SendData9bit(0x132);
  SPI_SendData9bit(0x12C);
  SPI_SendData9bit(0x18c);
  SPI_SendData9bit(0x10e);
  SPI_SendData9bit(0x134);
  SPI_SendData9bit(0x12C);
  SPI_SendData9bit(0x18c);
  SPI_SendData9bit(0xe9);
  SPI_SendData9bit(0x136);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0xeb);
  SPI_SendData9bit(0x102);
  SPI_SendData9bit(0x101);
  SPI_SendData9bit(0x1e4);
  SPI_SendData9bit(0x1e4);
  SPI_SendData9bit(0x144);
  SPI_SendData9bit(0x188);
  SPI_SendData9bit(0x140);
  SPI_SendData9bit(0xff);
  SPI_SendData9bit(0xed);
  SPI_SendData9bit(0x1bf);
  SPI_SendData9bit(0x1ca);
  SPI_SendData9bit(0x176);
  SPI_SendData9bit(0x154);
  SPI_SendData9bit(0x101);
  SPI_SendData9bit(0x1ff);
  SPI_SendData9bit(0x1ff);
  SPI_SendData9bit(0x1ff);
  SPI_SendData9bit(0x1ff);
  SPI_SendData9bit(0x1ff);
  SPI_SendData9bit(0x1ff);
  SPI_SendData9bit(0x110);
  SPI_SendData9bit(0x145);
  SPI_SendData9bit(0x167);
  SPI_SendData9bit(0x1ac);
  SPI_SendData9bit(0x1fb);
  SPI_SendData9bit(0xef);
  SPI_SendData9bit(0x108);
  SPI_SendData9bit(0x108);
  SPI_SendData9bit(0x108);
  SPI_SendData9bit(0x145);
  SPI_SendData9bit(0x13f);
  SPI_SendData9bit(0x154);
  SPI_SendData9bit(0xff);
  SPI_SendData9bit(0x177);
  SPI_SendData9bit(0x101);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x113);
  SPI_SendData9bit(0xe8);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x10e);
  SPI_SendData9bit(0xff);
  SPI_SendData9bit(0x177);
  SPI_SendData9bit(0x101);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x11);
  udelay(150000);
  SPI_SendData9bit(0xff);
  SPI_SendData9bit(0x177);
  SPI_SendData9bit(0x101);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x113);
  SPI_SendData9bit(0xe8);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x10c);
  udelay(10000);
  SPI_SendData9bit(0xe8);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0xff);
  SPI_SendData9bit(0x177);
  SPI_SendData9bit(0x101);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x36);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x3a);
  SPI_SendData9bit(0x166);
  SPI_SendData9bit(0x29);
  udelay(50000);
  SPI_SendData9bit(0xff);
  SPI_SendData9bit(0x177);
  SPI_SendData9bit(0x101);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x100);
  SPI_SendData9bit(0x110);
  SPI_SendData9bit(0xe5);
  SPI_SendData9bit(0x100);
  udelay(50000);
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

static struct tft_config st7701_dc_480x800_cfg = {
	.pix_clk_inv = 1,
	.de_dl = 0,
	.sync_dl = 0,
	.color_even = TFT_LCD_COLOR_EVEN_RGB,
	.color_odd = TFT_LCD_COLOR_ODD_RGB,
	.mode = TFT_LCD_MODE_PARALLEL_565,
};

struct lcd_panel lcd_panel = {
	.name = "st7701_dc_480x800",
	.num_modes = ARRAY_SIZE(panel_modes),
	.modes = panel_modes,
	.bpp = 16,
	.width = 55,
	.height = 94,

	.lcd_type = LCD_TYPE_TFT,

	.tft_config = &st7701_dc_480x800_cfg,
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
	struct gpio_desc *vdd_en = panel->vdd_en;

	if (POWER_IS_ON(power) && !POWER_IS_ON(panel->power)) {
		gpiod_set_value(vdd_en, 1);
		Initial_IC();
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

/**
* @ pannel_st7701_dc_480x800_lcd_ops, register to kernel common backlight/lcd.c framworks.
*/
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

	panel->spi.sda = fwnode_gpiod_get_index(fwnode, "ingenic,lcd-sda", 0, dflags, NULL);
	if (IS_ERR(panel->spi.sda)) {
    dev_err(dev, "Failed to initialize sda pin!\n");
    goto err_request_sda;
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
  gpiod_direction_output(panel->spi.sda, 0);
  gpiod_direction_output(panel->spi.sck, 0);
  gpiod_direction_output(panel->spi.cs, 0);

	return 0;
err_request_cs:
  gpiod_put(panel->spi.cs);
err_request_sck:
  gpiod_put(panel->spi.sck);
err_request_sda:
  gpiod_put(panel->spi.sda);
err_request_rst:
  gpiod_put(panel->rst);
	return ret;
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
	/* struct panel_dev *panel; */
	struct backlight_properties props;

	memset(&props, 0, sizeof(props));
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
	{ .compatible = "ingenic,st7701_dc_480x800", },
	{},
};

static struct platform_driver panel_driver = {
	.probe      = panel_probe,
	.remove     = panel_remove,
	.driver     = {
		.name   = "st7701_dc_480x800",
		.of_match_table = panel_of_match,
#ifdef CONFIG_PM
		.pm = &panel_pm_ops,
#endif
	},
};

module_platform_driver(panel_driver);
MODULE_LICENSE("GPL");
