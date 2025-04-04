/*
 * driver/video/fbdev/ingenic/x2000_v12/displays/panel-ylym286a.c
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

struct panel_dev {
	/* ingenic frame buffer */
	struct i2c_client *client;
	struct device *dev;
	struct lcd_panel *panel;

	/* common lcd framework */
	struct lcd_device *lcd;
	int power;

	int i2c_id;
	struct board_gpio reset;
	struct board_gpio lcd_en;
	struct board_gpio bl_en;
	char *panel_name;
};

#define FPS    60
#define HACT   1920
#if 1

	#define VACT   540

	#define HFP    200
	#define HBP    200
#else
	#define VACT   1080

	#define HFP    88
	#define HBP    148

#endif
#define HS     44

#define VFP    4
#define VBP    36
#define VS     5

#include "lt9211.c"

static struct panel_dev *lt9211_info;

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
	.name                   = "lt9211-ylym286a",
	.refresh                = FPS,
	.xres                   = HACT,
	.yres                   = VACT,
	.left_margin            = HFP,
	.right_margin           = HBP,
	.upper_margin           = VFP,
	.lower_margin           = VBP,

	.hsync_len              = HS,
	.vsync_len              = VS,
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
	.video_config.receive_ack_packets = 0,  /* enable receiving of ack packets */
	.video_config.is_18_loosely = 0,
	.video_config.data_en_polarity = 1,
	.video_config.byte_clock = 0, // driver will auto calculate byte_clock.
	.video_config.byte_clock_coef = MIPI_PHY_BYTE_CLK_COEF_MUL1,

	.dsi_config.max_lanes = 2,
	.dsi_config.max_hs_to_lp_cycles = 100,
	.dsi_config.max_lp_to_hs_cycles = 40,
	.dsi_config.max_bta_cycles = 4095,
	.dsi_config.color_mode_polarity = 1,
	.dsi_config.shut_down_polarity = 1,
	.dsi_config.max_bps = 2750,
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

struct lcd_panel lcd_panel[] = {
	[0] = {
		.name = "ylym286a",
		.modes = &panel_modes,
		.num_modes = 1,
		.dsi_pdata = &jzdsi_pdata,

		.bpp = 24,
		.width = 699,
		.height = 197,

		.lcd_type = LCD_TYPE_MIPI_TFT,

		.tft_config = &kd050hdfia019_cfg,
		.dsi_pdata = &jzdsi_pdata,

		.dither_enable = 0,
		.dither.dither_red = 0,
		.dither.dither_green = 0,
		.dither.dither_blue = 0,

		.ops = &panel_ops,
	},
};

static unsigned char i2c_read(struct i2c_client *client, unsigned char addr)
{
	return i2c_smbus_read_byte_data(client, addr);
}

static unsigned int i2c_write(struct i2c_client *client, unsigned char addr, unsigned char value)
{
	return i2c_smbus_write_byte_data(client, addr, value);
}

void lt9211_reset(struct panel_dev *lt9211_info)
{
	struct board_gpio *rst = &lt9211_info->reset;

	gpio_direction_output(rst->gpio, 1);
	mdelay(1);
	gpio_direction_output(rst->gpio, 0);
	mdelay(100);
	gpio_direction_output(rst->gpio, 1);
	mdelay(100);
}

#define POWER_IS_ON(pwr)        ((pwr) <= FB_BLANK_NORMAL)
static int panel_set_power(struct lcd_device *lcd, int power)
{
	struct panel_dev *lt9211_info = lcd_get_data(lcd);
	struct i2c_client *client = lt9211_info->client;

	struct board_gpio *lcd_en = &lt9211_info->lcd_en;
	struct board_gpio *bl_en = &lt9211_info->bl_en;
	struct board_gpio *rst = &lt9211_info->reset;

	if (POWER_IS_ON(power) && !POWER_IS_ON(lt9211_info->power)) {
		gpio_direction_output(rst->gpio, 0);
		gpio_direction_output(lcd_en->gpio, 1);
		mdelay(15);

		lt9211_reset(lt9211_info);
		if (LT9211_MIPI2LVDS_Config(client)) {
			return -1;
		}

		gpio_direction_output(bl_en->gpio, 1);
	}
	if (!POWER_IS_ON(power) && POWER_IS_ON(lt9211_info->power)) {
		gpio_direction_output(bl_en->gpio, 0);
		gpio_direction_output(lcd_en->gpio, 0);
	}

	lt9211_info->power = power;
	return 0;
}

int panel_bridge_init(void)
{
	int ret = 0;
	struct i2c_client *client = lt9211_info->client;

	ret = LT9211_MIPI2LVDS_Config(client);

	return ret;
}
static int panel_get_power(struct lcd_device *lcd)
{
	struct panel_dev *lt9211_info = lcd_get_data(lcd);

	return lt9211_info->power;
}

static struct lcd_ops panel_lcd_ops = {
	.set_power = panel_set_power,
	.get_power = panel_get_power,
};

int _of_get_named_gpio_lvl(struct device *dev, short *gpio, short *lvl, char *of_name)
{
	int ret = 0;
	enum of_gpio_flags flags;

	/* devm_gpio_request_one */
	*gpio = of_get_named_gpio_flags(dev->of_node, of_name, 0, &flags);
	if (gpio_is_valid(*gpio)) {
		*lvl = (flags == OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = gpio_request_one(*gpio, GPIOF_DIR_OUT, of_name);
		if (ret < 0) {
			dev_err(dev, "Failed to request reset pin!\n");
			return ret;
		}
	} else {
		dev_warn(dev, "invalid gpio: %s\n", of_name);
		return -1;
	}
	return 0;
}

static int of_panel_parse(struct device *dev)
{
	struct panel_dev *lt9211_info = dev_get_drvdata(dev);
	int ret = 0;

	if ((ret = _of_get_named_gpio_lvl(dev,
	                                  &lt9211_info->reset.gpio,
	                                  &lt9211_info->reset.active_level,
	                                  "lt9211,reset-gpio"))) {
		return ret;
	}

	if ((ret = _of_get_named_gpio_lvl(dev,
	                                  &lt9211_info->lcd_en.gpio,
	                                  &lt9211_info->lcd_en.active_level,
	                                  "lt9211,lcd_en-gpio"))) {
		goto err_request_reset;
	}

	if ((ret = _of_get_named_gpio_lvl(dev,
	                                  &lt9211_info->bl_en.gpio,
	                                  &lt9211_info->bl_en.active_level,
	                                  "lt9211,bl_en-gpio"))) {
		goto err_request_reset;
	}

	/* devm_ */

	return 0;
err_request_reset:
	if (gpio_is_valid(lt9211_info->reset.gpio)) {
		gpio_free(lt9211_info->reset.gpio);
	}
	return ret;
}
#if 0
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
static int panel_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM
static int panel_suspend(struct device *dev)
{
	struct panel_dev *lt9211_info = dev_get_drvdata(dev);

	panel_set_power(lt9211_info->lcd, FB_BLANK_POWERDOWN);
	return 0;
}

static int panel_resume(struct device *dev)
{
	struct panel_dev *lt9211_info = dev_get_drvdata(dev);

	panel_set_power(lt9211_info->lcd, FB_BLANK_UNBLANK);
	return 0;
}

static const struct dev_pm_ops panel_pm_ops = {
	.suspend = panel_suspend,
	.resume = panel_resume,
};
#endif

static const struct of_device_id panel_of_match[] = {
	{ .compatible = "ingenic,ylym286a", },
	{},
};

static struct platform_driver panel_driver = {
	.probe      = panel_probe,
	.remove     = panel_remove,
	.driver     = {
		.name   = "ylym286a",
		.of_match_table = panel_of_match,
#ifdef CONFIG_PM
		.pm = &panel_pm_ops,
#endif
	},
};
#endif

static struct mipi_dsim_lcd_driver panel_dev_dsim_ddi_driver = {
	.name = "lt9211-ylym286a",
	.id = -1,

	/* .power_on = panel_dev_power_on, */
	/* .set_sequence = panel_dev_set_sequence, */
	/* .probe = panel_dev_probe, */
	/* .suspend = panel_suspend, */
	/* .resume = panel_resume, */
};

struct mipi_dsim_lcd_device panel_dev_device = {
	.name       = "lt9211-ylym286a",
	.id = 0,
};

static int lt9211_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0, i;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;

	lt9211_info = kzalloc(sizeof(struct panel_dev), GFP_KERNEL);
	if (lt9211_info == NULL) {
		dev_err(dev, "Failed to alloc memory!");
		return -ENOMEM;
	}
	lt9211_info->dev = dev;
	lt9211_info->client = client;
	dev_set_drvdata(dev, lt9211_info);

	// set dev_info.
	ret = of_panel_parse(dev);
	if (ret < 0) {
		goto err_of_parse;
	}

	// register lcd device.
	lt9211_info->lcd = lcd_device_register("panel_lcd", dev, lt9211_info, &panel_lcd_ops);
	if (IS_ERR_OR_NULL(lt9211_info->lcd)) {
		dev_err(dev, "Error register lcd!\n");
		ret = -EINVAL;
		goto err_of_parse;
	}

	of_property_read_string(np, "lt9211,panel_name", (const char **)&lt9211_info->panel_name);

	if (lt9211_info->panel_name) {
		for (i = 0; i < ARRAY_SIZE(lcd_panel); ++i) {
			if (strcmp(lcd_panel[i].name, lt9211_info->panel_name) == 0) {
				break;
			}
		}
		if (i == ARRAY_SIZE(lcd_panel)) {
			printk("\033[31munsupport panel!\033[0m\n");
			goto err_lcd_register;
		}
	} else {
		printk("\033[31mno found specid lcd panel!\033[0m\n");
		goto err_lcd_register;
	}

	mipi_dsi_register_lcd_device(&panel_dev_device);
	mipi_dsi_register_lcd_driver(&panel_dev_dsim_ddi_driver);

	ret = ingenicfb_register_panel(&lcd_panel[i]);
	if (ret < 0) {
		dev_err(dev, "Failed to register lcd panel!\n");
		goto err_lcd_register;
	}

	// register ingenicfb device.
	/* TODO: should this power status sync from uboot */
	lt9211_info->power = FB_BLANK_POWERDOWN;
	if (panel_set_power(lt9211_info->lcd, FB_BLANK_UNBLANK)) {
		goto err_lcd_register;
	}

	printk("\033[32mregister %s sucess.\033[0m\n", lt9211_info->panel_name);

	return 0;

err_lcd_register:
	lcd_device_unregister(lt9211_info->lcd);
err_of_parse:
	kfree(lt9211_info);
	printk("\033[31mregister %s dailed.\033[0m\n", lt9211_info->panel_name);
	return ret;
}

static int lt9211_remove(struct i2c_client *client)
{
	struct panel_dev *lt9211_info = dev_get_drvdata(&client->dev);

	panel_set_power(lt9211_info->lcd, FB_BLANK_POWERDOWN);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id lt9211_match_table[] = {
	{.compatible = "mipi2lvds,lt9211",},
	{ },
};
MODULE_DEVICE_TABLE(of, lt9211_match_table);
#endif

static const struct i2c_device_id lt9211_id[] = {
	{ "lt9211", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lt9211_id);

static struct i2c_driver lt9211_driver = {
	.probe      = lt9211_probe,
	.remove     = lt9211_remove,
	.driver = {
		.name     = "lt9211",
		.owner    = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(lt9211_match_table),
#endif
	},
	.id_table = lt9211_id,
};

static int __init lt9211_init(void)
{
	int ret;
	printk("\033[33m%s\033[0m\n", "lt9211 driver installing...");
	ret = i2c_add_driver(&lt9211_driver);
	if (ret != 0) {
		printk("lt9211 driver init failed!\n");
	}

	return ret;
}

static void __exit lt9211_exit(void)
{
	printk("lt9211 driver exited.\n");
	i2c_del_driver(&lt9211_driver);
}

/* module_i2c_driver(lt9211_driver); */
module_init(lt9211_init);
module_exit(lt9211_exit);

MODULE_DESCRIPTION("LT9211 Driver");
MODULE_LICENSE("GPL");
