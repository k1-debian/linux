/*
 * kernel-4.4.94/module_drivers/drivers/video/fbdev/ingenic/fb_stage/displays/panel-st7701s-rgb666.c
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

struct board_gpio {
	short gpio;
	short active_level;
};

struct gpio_spi {
	short sdo;
	short sdi;
	short sck;
	short cs;
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
	struct gpio_spi spi;
};

static struct panel_dev *panel;

#define RESET(n)\
	gpio_direction_output(panel->rst.gpio, n)

#define CS(n)\
	gpio_direction_output(panel->spi.cs, n)

#define SCK(n)\
	gpio_direction_output(panel->spi.sck, n)

#define SDO(n)\
	gpio_direction_output(panel->spi.sdo, n)

#if 0
#define SDI()\
	gpio_get_value(panel->spi.sdi)
#endif

void SPI_SendData(unsigned char i)
{
	unsigned char n;

	for (n = 0; n < 8; n++) {
		if (i & 0x80) {
			SDO(1);
		} else {
			SDO(0);
		}
		i = i << 1;

		SCK(0);
		udelay(10);
		SCK(1);
		udelay(10);
	}
}

void SPI_WriteComm(unsigned char c)
{
	CS(0);

	SDO(0);

	SCK(0);
	udelay(10);
	SCK(1);
	udelay(10);

	SPI_SendData(c);

	CS(1);
}

void SPI_WriteData(unsigned char d)
{
	CS(0);

	SDO(1);

	SCK(0);
	udelay(10);
	SCK(1);
	udelay(10);

	SPI_SendData(d);

	CS(1);
}

void Initial_IC(void)
{
	RESET(1);
	udelay(10000);
	RESET(0);
	udelay(10);
	RESET(1);
	udelay(120000); // Delay 120ms // This delay time is necessary
	SPI_WriteComm(0x11);//Attention !!! sleep out config or color display error.
	/* udelay(120000); // Delay 120ms // This delay time is necessary */
	//PAGE1
#if 1
	SPI_WriteComm(0xFF);
	SPI_WriteData(0x77);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x10);

	SPI_WriteComm(0xC0);
	SPI_WriteData(0x3B);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xC1);
	SPI_WriteData(0x0D);
	SPI_WriteData(0x02);

	SPI_WriteComm(0xC2);
	SPI_WriteData(0x31);
	SPI_WriteData(0x05);

	SPI_WriteComm(0xCD);//COLCTRL   (CDh/CD00h):Color Control.MDT=”1”, pixel collect to DB[17:0].
	SPI_WriteData(0x08);//D3,MDT=”1”, pixel collect to DB[17:0].

	SPI_WriteComm(0xB0);
	SPI_WriteData(0x00);//Positive Voltage Gamma Control
	SPI_WriteData(0x11);
	SPI_WriteData(0x18);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x11);
	SPI_WriteData(0x06);
	SPI_WriteData(0x07);
	SPI_WriteData(0x08);
	SPI_WriteData(0x07);
	SPI_WriteData(0x22);
	SPI_WriteData(0x04);
	SPI_WriteData(0x12);
	SPI_WriteData(0x0F);
	SPI_WriteData(0xAA);
	SPI_WriteData(0x31);
	SPI_WriteData(0x18);

	SPI_WriteComm(0xB1);
	SPI_WriteData(0x00);// //Negative Voltage Gamma Control
	SPI_WriteData(0x11);
	SPI_WriteData(0x19);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x12);
	SPI_WriteData(0x07);
	SPI_WriteData(0x08);
	SPI_WriteData(0x08);
	SPI_WriteData(0x08);
	SPI_WriteData(0x22);
	SPI_WriteData(0x04);
	SPI_WriteData(0x11);
	SPI_WriteData(0x11);
	SPI_WriteData(0xA9);
	SPI_WriteData(0x32);
	SPI_WriteData(0x18);
	//PAGE1

	SPI_WriteComm(0xFF);
	SPI_WriteData(0x77);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x11);

	SPI_WriteComm(0xB0);    SPI_WriteData(0x60); //Vop=4.7375v

	SPI_WriteComm(0xB1);    SPI_WriteData(0x32);//VCOM=32

	SPI_WriteComm(0xB2);    SPI_WriteData(0x07); //VGH=15v

	SPI_WriteComm(0xB3);    SPI_WriteData(0x80);

	SPI_WriteComm(0xB5);    SPI_WriteData(0x49);//VGL=-10.17v

	SPI_WriteComm(0xB7);    SPI_WriteData(0x85);

	SPI_WriteComm(0xB8);    SPI_WriteData(0x21);//AVDD=6.6 & AVCL=-4.6

	SPI_WriteComm(0xC1);    SPI_WriteData(0x78);

	SPI_WriteComm(0xC2);    SPI_WriteData(0x78);

	SPI_WriteComm(0xE0);
	SPI_WriteData(0x00);
	SPI_WriteData(0x1B);
	SPI_WriteData(0x02);

	SPI_WriteComm(0xE1);
	SPI_WriteData(0x08);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x07);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x44);
	SPI_WriteData(0x44);

	SPI_WriteComm(0xE2);
	SPI_WriteData(0x11);
	SPI_WriteData(0x11);
	SPI_WriteData(0x44);
	SPI_WriteData(0x44);
	SPI_WriteData(0xED);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0xEC);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xE3);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x11);
	SPI_WriteData(0x11);

	SPI_WriteComm(0xE4);
	SPI_WriteData(0x44);
	SPI_WriteData(0x44);

	SPI_WriteComm(0xE5);
	SPI_WriteData(0x0A);
	SPI_WriteData(0xE9);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x0C);
	SPI_WriteData(0xEB);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x0E);
	SPI_WriteData(0xED);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x10);
	SPI_WriteData(0xEF);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);

	SPI_WriteComm(0xE6);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x11);
	SPI_WriteData(0x11);

	SPI_WriteComm(0xE7);
	SPI_WriteData(0x44);
	SPI_WriteData(0x44);

	SPI_WriteComm(0xE8);
	SPI_WriteData(0x09);
	SPI_WriteData(0xE8);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x0B);
	SPI_WriteData(0xEA);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x0D);
	SPI_WriteData(0xEC);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x0F);
	SPI_WriteData(0xEE);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);

	SPI_WriteComm(0xEB);
	SPI_WriteData(0x02);
	SPI_WriteData(0x00);
	SPI_WriteData(0xE4);
	SPI_WriteData(0xE4);
	SPI_WriteData(0x88);
	SPI_WriteData(0x00);
	SPI_WriteData(0x40);

	SPI_WriteComm(0xEC);
	SPI_WriteData(0x3C);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xED);
	SPI_WriteData(0xAB);
	SPI_WriteData(0x89);
	SPI_WriteData(0x76);
	SPI_WriteData(0x54);
	SPI_WriteData(0x02);
	SPI_WriteData(0xFF);
	SPI_WriteData(0xFF);
	SPI_WriteData(0xFF);
	SPI_WriteData(0xFF);
	SPI_WriteData(0xFF);
	SPI_WriteData(0xFF);
	SPI_WriteData(0x20);
	SPI_WriteData(0x45);
	SPI_WriteData(0x67);
	SPI_WriteData(0x98);
	SPI_WriteData(0xBA);

	SPI_WriteComm(0x36);//MADCTL(36h/3600h): Display data access control
	SPI_WriteData(0x00);//BGR=0->RGB
	//  SPI_WriteData(0x08);//BGR=1->BGR

	//-----------VAP & VAN---------------
	SPI_WriteComm(0xFF);
	SPI_WriteData(0x77);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x13);

	SPI_WriteComm(0xE5);
	SPI_WriteData(0xE4);

	SPI_WriteComm(0xFF);
	SPI_WriteData(0x77);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);

	SPI_WriteComm(0x3A);
	SPI_WriteData(0x66);

	SPI_WriteComm(0x0C);//RDDCOLMOD (0Ch/0C00h): Read Display Pixel Format.
	SPI_WriteData(0x60);//VIPF[2:0]:“110” = 18-bit / pixel

	SPI_WriteComm(0x21);//NVON (21h/2100h): Display Inversion On
	udelay(1000); // Delay 1ms
	SPI_WriteComm(0x11);
	udelay(120000); // Delay 120ms // This delay time is necessary
	SPI_WriteComm(0x29);
	//  udelay(20000); // Delay 20ms // This delay time is necessary
#else
	SPI_WriteComm(0xFF);
	SPI_WriteData(0x77);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x13);

	SPI_WriteComm(0xEF);
	SPI_WriteData(0x08);

	SPI_WriteComm(0xFF);
	SPI_WriteData(0x77);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x10);

	SPI_WriteComm(0xC0);
	SPI_WriteData(0x3B);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xC1);
	SPI_WriteData(0x0D);
	SPI_WriteData(0x02);

	SPI_WriteComm(0xC2);
	SPI_WriteData(0x21);
	SPI_WriteData(0x08);

	SPI_WriteComm(0xCD);
	SPI_WriteData(0x08);

	SPI_WriteComm(0xB0);
	SPI_WriteData(0x00);
	SPI_WriteData(0x11);
	SPI_WriteData(0x18);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x11);
	SPI_WriteData(0x06);
	SPI_WriteData(0x07);
	SPI_WriteData(0x08);
	SPI_WriteData(0x07);
	SPI_WriteData(0x22);
	SPI_WriteData(0x04);
	SPI_WriteData(0x12);
	SPI_WriteData(0x0F);
	SPI_WriteData(0xAA);
	SPI_WriteData(0x31);
	SPI_WriteData(0x18);

	SPI_WriteComm(0xB1);
	SPI_WriteData(0x00);
	SPI_WriteData(0x11);
	SPI_WriteData(0x19);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x12);
	SPI_WriteData(0x07);
	SPI_WriteData(0x08);
	SPI_WriteData(0x08);
	SPI_WriteData(0x08);
	SPI_WriteData(0x22);
	SPI_WriteData(0x04);
	SPI_WriteData(0x11);
	SPI_WriteData(0x11);
	SPI_WriteData(0xA9);
	SPI_WriteData(0x32);
	SPI_WriteData(0x18);

	SPI_WriteComm(0xFF);
	SPI_WriteData(0x77);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x11);

	SPI_WriteComm(0xB0);    SPI_WriteData(0x60);

	SPI_WriteComm(0xB1);    SPI_WriteData(0x30);

	SPI_WriteComm(0xB2);    SPI_WriteData(0x87);

	SPI_WriteComm(0xB3);    SPI_WriteData(0x80);

	SPI_WriteComm(0xB5);    SPI_WriteData(0x49);

	SPI_WriteComm(0xB7);    SPI_WriteData(0x85);

	SPI_WriteComm(0xB8);    SPI_WriteData(0x21);

	SPI_WriteComm(0xC1);    SPI_WriteData(0x78);

	SPI_WriteComm(0xC2);    SPI_WriteData(0x78);

	udelay(20000); // Delay 20ms

	SPI_WriteComm(0xE0);
	SPI_WriteData(0x00);
	SPI_WriteData(0x1B);
	SPI_WriteData(0x02);

	SPI_WriteComm(0xE1);
	SPI_WriteData(0x08);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x07);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x44);
	SPI_WriteData(0x44);

	SPI_WriteComm(0xE2);
	SPI_WriteData(0x11);
	SPI_WriteData(0x11);
	SPI_WriteData(0x44);
	SPI_WriteData(0x44);
	SPI_WriteData(0xED);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0xEC);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xE3);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x11);
	SPI_WriteData(0x11);

	SPI_WriteComm(0xE4);
	SPI_WriteData(0x44);
	SPI_WriteData(0x44);

	SPI_WriteComm(0xE5);
	SPI_WriteData(0x0A);
	SPI_WriteData(0xE9);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x0c);
	SPI_WriteData(0xEB);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x0E);
	SPI_WriteData(0xED);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x10);
	SPI_WriteData(0xEF);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);

	SPI_WriteComm(0xE6);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x11);
	SPI_WriteData(0x11);

	SPI_WriteComm(0xE7);
	SPI_WriteData(0x44);
	SPI_WriteData(0x44);

	SPI_WriteComm(0xE8);
	SPI_WriteData(0x09);
	SPI_WriteData(0xE8);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x0B);
	SPI_WriteData(0xEA);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x0D);
	SPI_WriteData(0xEC);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x0F);
	SPI_WriteData(0xEE);
	SPI_WriteData(0xD8);
	SPI_WriteData(0xA0);

	SPI_WriteComm(0xEB);
	SPI_WriteData(0x02);
	SPI_WriteData(0x00);
	SPI_WriteData(0xE4);
	SPI_WriteData(0xE4);
	SPI_WriteData(0x88);
	SPI_WriteData(0x00);
	SPI_WriteData(0x40);

	SPI_WriteComm(0xEC);
	SPI_WriteData(0x3C);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xED);
	SPI_WriteData(0xAB);
	SPI_WriteData(0x89);
	SPI_WriteData(0x76);
	SPI_WriteData(0x54);
	SPI_WriteData(0x02);
	SPI_WriteData(0xFF);
	SPI_WriteData(0xFF);
	SPI_WriteData(0xFF);
	SPI_WriteData(0xFF);
	SPI_WriteData(0xFF);
	SPI_WriteData(0xFF);
	SPI_WriteData(0x20);
	SPI_WriteData(0x45);
	SPI_WriteData(0x67);
	SPI_WriteData(0x98);
	SPI_WriteData(0xBA);

	SPI_WriteComm(0xFF);
	SPI_WriteData(0x77);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);

	//  SPI_WriteComm(0x21);//NVON (21h/2100h): Display Inversion On
	udelay(1000); // Delay 1ms
	SPI_WriteComm(0x11);
	udelay(120000); // Delay 120ms // This delay time is necessary
	SPI_WriteComm(0x29);
	udelay(20000); // Delay 20ms // This delay time is necessary
#endif
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
		.name                   = "480X480",
		.refresh                = 60,
		.xres                   = 480,
		.yres                   = 480,
		.pixclock               = (14000),//KHz
		//      .left_margin            = 160,
		//      .right_margin           = 10,
		//      .upper_margin           = 2,
		//      .lower_margin           = 1,
		.left_margin            = 20,
		.right_margin           = 20,
		.upper_margin           = 16,
		.lower_margin           = 16,

		//      .hsync_len              = 16,
		//      .vsync_len              = 13,
		.hsync_len              = 20,
		.vsync_len              = 4,
		/* .sync                   = ~FB_SYNC_HOR_HIGH_ACT & ~FB_SYNC_VERT_HIGH_ACT, */
		.vmode                  = FB_VMODE_NONINTERLACED,
		.flag                   = 0,
	},
};

static struct tft_config st7701ds_spirgb_cfg = {
	.pix_clk_inv = 0,
	.de_dl = 0,
	.sync_dl = 0,
	.color_even = TFT_LCD_COLOR_EVEN_RGB,
	.color_odd = TFT_LCD_COLOR_ODD_RGB,
	//  .color_even = TFT_LCD_COLOR_EVEN_RBG,
	//  .color_odd = TFT_LCD_COLOR_ODD_RBG,
	.mode = TFT_LCD_MODE_PARALLEL_666,
};

struct lcd_panel lcd_panel = {
	.name = "st7701ds_spirgb",
	.num_modes = ARRAY_SIZE(panel_modes),
	.modes = panel_modes,
	.bpp = 32,
	.width = 72,
	.height = 70,

	.lcd_type = LCD_TYPE_TFT,

	.tft_config = &st7701ds_spirgb_cfg,
#if 0
	.dither_enable = 1,
	.dither.dither_red = 1,
	.dither.dither_green = 1,
	.dither.dither_blue = 1,
#else
	.dither_enable = 0,
	.dither.dither_red = 0,
	.dither.dither_green = 0,
	.dither.dither_blue = 0,
#endif
	.ops = &panel_ops,
};

#if 0
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

	if (brightness) {
		gpio_direction_output(panel->pwm.gpio, 0);
		udelay(5000);
		gpio_direction_output(panel->pwm.gpio, 1);
		udelay(100);

		for (i = pulse_num; i > 0; i--) {
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

#define POWER_IS_ON(pwr)        ((pwr) <= FB_BLANK_NORMAL)
static int panel_set_power(struct lcd_device *lcd, int power)
{
	struct panel_dev *panel = lcd_get_data(lcd);
	struct board_gpio *vdd_en = &panel->vdd_en;
	struct board_gpio *rst = &panel->rst;

	if (POWER_IS_ON(power) && !POWER_IS_ON(panel->power)) {
		/* *(unsigned int*)(0xb00101a4) = 0xf000000; */
		/* *(unsigned int*)(0xb00101b4) = 0xf000000; */
		/* *(unsigned int*)(0xb00101c4) = 0xf000000; */
		gpio_direction_output(vdd_en->gpio, 0);
		gpio_direction_output(rst->gpio, 1);
		//      printk("==============vdd_en=%d,rst=%d,func=%s,line=%d\n",vdd_en->gpio,rst->gpio,__func__,__LINE__);
		Initial_IC();
	}
	if (!POWER_IS_ON(power) && POWER_IS_ON(panel->power)) {
		gpio_direction_output(vdd_en->gpio, 1);
		//      printk("==============vdd_en=%d,rst=%d,func=%s,line=%d\n",vdd_en->gpio,rst->gpio,__func__,__LINE__);
	}

	panel->power = power;
	//      printk("==============power=%d,func=%s,line=%d\n",panel->power,__func__,__LINE__);
	return 0;
}

static int panel_get_power(struct lcd_device *lcd)
{
	struct panel_dev *panel = lcd_get_data(lcd);

	return panel->power;
}

/**
* @ pannel_st7701ds_spirgb_lcd_ops, register to kernel common backlight/lcd.c framworks.
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

	panel->vdd_en.gpio = of_get_named_gpio_flags(np, "ingenic,vdd-en-gpio", 0, &flags);
	if (gpio_is_valid(panel->vdd_en.gpio)) {
		panel->vdd_en.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = gpio_request_one(panel->vdd_en.gpio, GPIOF_DIR_OUT, "vdd_en");
		if (ret < 0) {
			dev_err(dev, "Failed to request vdd_en pin!\n");
			return ret;
		}
	} else {
		dev_warn(dev, "invalid gpio vdd_en.gpio: %d\n", panel->vdd_en.gpio);
	}

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
#if 0
	panel->pwm.gpio = of_get_named_gpio_flags(np, "ingenic,lcd-pwm-gpio", 0, &flags);
	if (gpio_is_valid(panel->pwm.gpio)) {
		panel->pwm.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = gpio_request_one(panel->pwm.gpio, GPIOF_DIR_OUT, "pwm");
		if (ret < 0) {
			dev_err(dev, "Failed to request pwm pin!\n");
			goto err_request_pwm;
		}
	} else {
		dev_warn(dev, "invalid gpio pwm.gpio: %d\n", panel->pwm.gpio);
	}
#endif

	panel->spi.sdo = of_get_named_gpio_flags(np, "ingenic,lcd-sdo-gpio", 0, &flags);
	if (gpio_is_valid(panel->spi.sdo)) {
		ret = gpio_request_one(panel->spi.sdo, GPIOF_DIR_OUT, "sdo");
		if (ret < 0) {
			dev_err(dev, "Failed to request sdo pin!\n");
			goto err_request_sdo;
		}
	} else {
		dev_warn(dev, "invalid gpio spi.sdo: %d\n", panel->spi.sdo);
	}

	panel->spi.sck = of_get_named_gpio_flags(np, "ingenic,lcd-sck-gpio", 0, &flags);
	if (gpio_is_valid(panel->spi.sck)) {
		ret = gpio_request_one(panel->spi.sck, GPIOF_DIR_OUT, "sck");
		if (ret < 0) {
			dev_err(dev, "Failed to request sck pin!\n");
			goto err_request_sck;
		}
	} else {
		dev_warn(dev, "invalid gpio spi.sck: %d\n", panel->spi.sck);
	}

	panel->spi.cs = of_get_named_gpio_flags(np, "ingenic,lcd-cs-gpio", 0, &flags);
	if (gpio_is_valid(panel->spi.cs)) {
		ret = gpio_request_one(panel->spi.cs, GPIOF_DIR_OUT, "cs");
		if (ret < 0) {
			dev_err(dev, "Failed to request cs pin!\n");
			goto err_request_cs;
		}
	} else {
		dev_warn(dev, "invalid gpio spi.cs: %d\n", panel->spi.cs);
	}

	return 0;
err_request_cs:
	if (gpio_is_valid(panel->spi.cs)) {
		gpio_free(panel->spi.cs);
	}
err_request_sck:
	if (gpio_is_valid(panel->spi.sck)) {
		gpio_free(panel->spi.sck);
	}
err_request_sdo:
	if (gpio_is_valid(panel->spi.sdo)) {
		gpio_free(panel->spi.sdo);
	}
err_request_pwm:
	if (gpio_is_valid(panel->rst.gpio)) {
		gpio_free(panel->rst.gpio);
	}
err_request_rst:
	if (gpio_is_valid(panel->rst.gpio)) {
		gpio_free(panel->rst.gpio);
	}
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

#if 0
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
#endif

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
	{ .compatible = "ingenic,st7701ds_spirgb", },
	{},
};

static struct platform_driver panel_driver = {
	.probe      = panel_probe,
	.remove     = panel_remove,
	.driver     = {
		.name   = "st7701ds_spirgb",
		.of_match_table = panel_of_match,
#ifdef CONFIG_PM
		.pm = &panel_pm_ops,
#endif
	},
};

module_platform_driver(panel_driver);
MODULE_LICENSE("GPL");
