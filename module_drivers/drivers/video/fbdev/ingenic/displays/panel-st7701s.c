/*
 * driver/video/fbdev/ingenic/x2000_v12/displays/panel-st7701s.c
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

static int test = 0;
static int power_flag;

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

static struct panel_dev *panel;

#define lcd_to_master(a)    (a->dsim_dev->master)
#define lcd_to_master_ops(a)    ((lcd_to_master(a))->master_ops)

struct st7701s {
	struct device *dev;
	unsigned int power;
	unsigned int id;

	struct lcd_device *ld;
	struct backlight_device *bd;

	struct mipi_dsim_lcd_device *dsim_dev;

};

static struct dsi_cmd_packet fitipower_st7701s_480_800_cmd_list1[] = {

	/**st7701s***/
	{0x39, 0x06, 0x00, {0xFF, 0x77, 0x01, 0x00, 0x00, 0x13}},
	{0x15, 0xEF, 0x08},
	{0x39, 0x06, 0x00, {0xFF, 0x77, 0x01, 0x00, 0x00, 0x10}},
	{0x39, 0x03, 0x00, {0xC0, 0x63, 0x00}},
	{0x39, 0x03, 0x00, {0xC1, 0x11, 0x0C}},
	{0x39, 0x03, 0x00, {0xC2, 0x07, 0x08}},
	{0x15, 0xCC, 0x10},
	{0x39, 0x11, 0x00, {0xB0, 0x00, 0x08, 0x0E, 0x0C, 0x10, 0x06, 0x01, 0x07, 0x07, 0x1E, 0x05, 0x13, 0x10, 0x2C, 0x34, 0x1E}},
	{0x39, 0x11, 0x00, {0xB1, 0x00, 0x0F, 0x16, 0x0D, 0x10, 0x05, 0x01, 0x08, 0x07, 0x1E, 0x04, 0x12, 0x11, 0x28, 0x30, 0x1E}},
	{0x39, 0x06, 0x00, {0xFF, 0x77, 0x01, 0x00, 0x00, 0x11}},
	{0x15, 0xB0, 0x72},
	{0x15, 0xB1, 0x8E},
	{0x15, 0xB2, 0x87},
	{0x15, 0xB3, 0x80},
	{0x15, 0xB5, 0x4C},
	{0x15, 0xB7, 0x85},
	{0x15, 0xB8, 0x20},
	{0x15, 0xC1, 0x78},
	{0x15, 0xC2, 0x78},
	{0x15, 0xD0, 0x88},
	{0x39, 0x07, 0x00, {0xE0, 0x00, 0x00, 0x02, 0x00, 0x00, 0x0C}},
	{0x39, 0x0C, 0x00, {0xE1, 0x06, 0x8C, 0x08, 0x8C, 0x07, 0x8C, 0x09, 0x8C, 0x00, 0x44, 0x44}},
	{0x39, 0x0D, 0x00, {0xE2, 0x00, 0x00, 0x04, 0x04, 0x00, 0x00, 0x03, 0xA0, 0x00, 0x00, 0x03, 0xA0}},
	{0x39, 0x05, 0x00, {0xE3, 0x00, 0x00, 0x33, 0x33}},
	{0x39, 0x03, 0x00, {0xE4, 0x44, 0x44}},
	{0x39, 0x11, 0x00, {0xE5, 0x0D, 0x31, 0x0C, 0xA0, 0x0F, 0x33, 0x0C, 0xA0, 0x09, 0x2D, 0x0C, 0xA0, 0x0B, 0x2F, 0x0C, 0xA0}},
	{0x39, 0x05, 0x00, {0xE6, 0x00, 0x00, 0x33, 0x33}},
	{0x39, 0x03, 0x00, {0xE7, 0x44, 0x44}},
	{0x39, 0x11, 0x00, {0xE8, 0x0E, 0x32, 0x0C, 0xA0, 0x10, 0x34, 0x0C, 0xA0, 0x0A, 0x2E, 0x0C, 0xA0, 0x0C, 0x30, 0x0C, 0xA0}},
	{0x39, 0x07, 0x00, {0xEB, 0x00, 0x01, 0xE4, 0xE4, 0x44, 0x00}},
	{0x39, 0x11, 0x00, {0xED, 0xF3, 0xC1, 0xAB, 0x0F, 0x67, 0x45, 0xFF, 0xFF, 0xFF, 0xFF, 0x54, 0x76, 0xF0, 0xBA, 0x1C, 0x3F}},
	{0x39, 0x07, 0x00, {0xEF, 0x10, 0x0D, 0x04, 0x08, 0x3F, 0x1F}},
	{0x39, 0x06, 0x00, {0xFF, 0x77, 0x01, 0x00, 0x00, 0x13}},
	{0x39, 0x03, 0x00, {0xE8, 0x00, 0x0E}},
	/**/
	{0x05, 0x11, 0x00},//delay 120ms
	/***/
	//  {0x39, 0x03, 0x00, {0xE8, 0x00, 0x0C}},//delay 20ms
	//  {0x39, 0x03, 0x00, {0xE8, 0x00, 0x00}},
	//  {0x39, 0x06, 0x00, {0xFF, 0x77, 0x01, 0x00, 0x00,0x00}},
	//  {0x39, 0x03, 0x00, {0x29, 0x36, 0x00}},
	/**st7701s***/

};

static struct dsi_cmd_packet fitipower_st7701s_480_800_cmd_list2[] = {
	{0x39, 0x03, 0x00, {0xE8, 0x00, 0x0C}},//delay 20ms
};

static struct dsi_cmd_packet fitipower_st7701s_480_800_cmd_list3[] = {
	{0x39, 0x03, 0x00, {0xE8, 0x00, 0x00}},
	{0x39, 0x06, 0x00, {0xFF, 0x77, 0x01, 0x00, 0x00, 0x00}},
	{0x39, 0x03, 0x00, {0x29, 0x36, 0x00}},
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
	//  struct dsi_cmd_packet data_to_send1 = {0x15, 0xc2, 0x20};//color bar test mode.
	//  while (1) {
	ops->cmd_write(lcd_to_master(lcd), data_to_send);
	//  ops->cmd_write(lcd_to_master(lcd), data_to_send1);
	//  }
}

static void panel_dev_display_on(struct panel_dev *lcd)
{
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);//color bar turn off 0x29 command.
	struct dsi_cmd_packet data_to_send = {0x05, 0x29, 0x00};
	/* struct dsi_cmd_packet data_to_send1 = {0x15, 0x51, 250}; */

	ops->cmd_write(lcd_to_master(lcd), data_to_send);
	/* ops->cmd_write(lcd_to_master(lcd), data_to_send1); */
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

	for (i = 0; i < ARRAY_SIZE(fitipower_st7701s_480_800_cmd_list1); i++) {
		ops->cmd_write(dsi,  fitipower_st7701s_480_800_cmd_list1[i]);
	}
	msleep(120);
	for (i = 0; i < ARRAY_SIZE(fitipower_st7701s_480_800_cmd_list2); i++) {
		ops->cmd_write(dsi,  fitipower_st7701s_480_800_cmd_list2[i]);
	}
	msleep(20);
	for (i = 0; i < ARRAY_SIZE(fitipower_st7701s_480_800_cmd_list3); i++) {
		ops->cmd_write(dsi,  fitipower_st7701s_480_800_cmd_list3[i]);
	}
}

static int panel_dev_ioctl(struct mipi_dsim_lcd_device *dsim_dev, int cmd)
{
	return 0;
}
static void panel_dev_set_sequence(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct st7701s *lcd = dev_get_drvdata(&dsim_dev->dev);

	printk(">>>>>>>>>>>>>>>>>>>>%s %d\n", __func__, __LINE__);

	panel_dev_panel_init(panel);
	panel_dev_sleep_out(panel);
	msleep(120);
	panel_dev_display_on(panel);
	msleep(5);
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
	if (power) {
		msleep(50);
		gpio_direction_output(rst->gpio, 0);
		msleep(50);
		gpio_direction_output(rst->gpio, 1);
		msleep(120);
	}

	panel->power = power;
}

static struct fb_videomode panel_modes = {
	.name = "fitipower_st7701s-lcd",
	.xres = 480,
	.yres = 800,
	//  .xres = 268,
	//  .yres = 800,

	//  .refresh = 10,
	.refresh = 60,

	.left_margin = 20,//hbp
	.right_margin = 20,//hfp
	.hsync_len = 20, //hsync

	//  .upper_margin = 16,//vbp
	.upper_margin = 18,//vbp
	.lower_margin = 16,//vfp
	.vsync_len = 4, //vsync

	.sync                   = FB_SYNC_HOR_HIGH_ACT & FB_SYNC_VERT_HIGH_ACT,
	.vmode                  = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct jzdsi_data jzdsi_pdata = {
	.modes = &panel_modes,
	.video_config.no_of_lanes = 2,
	.video_config.virtual_channel = 0,
	.video_config.color_coding = COLOR_CODE_24BIT,
	.video_config.video_mode = VIDEO_BURST_WITH_SYNC_PULSES,
	.video_config.receive_ack_packets = 0,  /* enable receiving of ack packets */
	.video_config.is_18_loosely = 0,
	.video_config.data_en_polarity = 1,
	.video_config.byte_clock = 0, // driver will auto calculate byte_clock.
	.video_config.byte_clock_coef = MIPI_PHY_BYTE_CLK_COEF_MUL3_DIV2, // byte_clock *3/2.

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
* @ pannel_st7701s_lcd_ops, register to kernel common backlight/lcd.c framworks.
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

	} else {
		dev_warn(dev, "invalid gpio vdd_en.gpio: %d\n", panel->vdd_en.gpio);
	}

	panel->rst.gpio = of_get_named_gpio_flags(np, "ingenic,rst-gpio", 0, &flags);
	if (gpio_is_valid(panel->rst.gpio)) {
		panel->rst.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;

	} else {
		dev_warn(dev, "invalid gpio rst.gpio: %d\n", panel->rst.gpio);
	}

	*(volatile  unsigned int *)0xb0010188 = (0x1 << 27);  //lcd_te high impedance config.
	*(volatile  unsigned int *)0xb0010198 = (0x1 << 27);

	panel->lcd_te.gpio = of_get_named_gpio_flags(np, "ingenic,lcd-te-gpio", 0, &flags);
	if (gpio_is_valid(panel->lcd_te.gpio)) {
		panel->lcd_te.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;

	} else {
		dev_warn(dev, "invalid gpio lcd_te.gpio: %d\n", panel->lcd_te.gpio);
	}
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

#define ST7701SID       0xFFFF0288
static int panel_detect_id(struct mipi_dsim_lcd_device *dsim_dev, unsigned char *buf, int count)
{
	int ret = 0;
	unsigned int *id = NULL;

	struct dsi_master_ops *ops = dsim_dev->master->master_ops;
	struct dsi_cmd_packet set_max_return_packet_size = {0x37, 0x04, 0x00};  //设置最大回读byte数.
	struct dsi_cmd_packet data_to_send0 = {0x14, 0xa1, 0x00};
	struct dsi_cmd_packet data_to_send1 = {0x14, 0xa1, 0x01};
	struct dsi_cmd_packet data_to_send2 = {0x14, 0xa1, 0x02};

	if (!power_flag) {
		panel_dev_power_on(dsim_dev, 1);
		power_flag = 1;
	}
	ret = ops->cmd_write(dsim_dev->master, set_max_return_packet_size);
	ret = ops->cmd_read(dsim_dev->master, data_to_send0, &buf[0], 4);

	id = (unsigned int *)buf;
	if (*id == ST7701SID) {
		return 0;
	}

	return -1;
}

static int panel_dev_probe(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct st7701s *lcd;
	int ret = 0;
	char buffer[4];
	ret = panel_detect_id(dsim_dev, buffer, 4);
	if (ret < 0) {
		return -EINVAL;
	}

	lcd = devm_kzalloc(&dsim_dev->dev, sizeof(struct st7701s), GFP_KERNEL);
	if (!lcd) {
		dev_err(&dsim_dev->dev, "failed to allocate fitipower_st7701s structure.\n");
		return -ENOMEM;
	}

	lcd->dsim_dev = dsim_dev;
	lcd->dev = &dsim_dev->dev;

	lcd->ld = lcd_device_register("fitipower_st7701s", lcd->dev, lcd,
	                              &panel_lcd_ops);
	if (IS_ERR(lcd->ld)) {
		dev_err(lcd->dev, "failed to register lcd ops.\n");
		return PTR_ERR(lcd->ld);
	}

	dev_set_drvdata(&dsim_dev->dev, lcd);

	dev_dbg(lcd->dev, "probed fitipower_st7701s panel driver.\n");

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
	.name = "fitipower_st7701s-lcd",
	.id = -1,

	.power_on = panel_dev_power_on,
	.set_sequence = panel_dev_set_sequence,
	.probe = panel_dev_probe,
	.suspend = panel_suspend,
	.resume = panel_resume,
};

static struct mipi_dsim_lcd_device panel_dev_device = {
	.name       = "fitipower_st7701s-lcd",
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
	{ .compatible = "ingenic,st7701s", },
	{},
};

static struct platform_driver panel_driver = {
	.probe      = panel_probe,
	.remove     = panel_remove,
	.driver     = {
		.name   = "st7701s",
		.of_match_table = panel_of_match,
#ifdef CONFIG_PM
		.pm = &panel_pm_ops,
#endif
	},
};

module_platform_driver(panel_driver);
MODULE_LICENSE("GPL");
