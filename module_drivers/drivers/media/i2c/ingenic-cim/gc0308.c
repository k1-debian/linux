/*
 * gc0308 Camera Driver
 *
 * Copyright (C) 2010 Alberto Panizzo <maramaopercheseimorto@gmail.com>
 *
 * Based on ov772x, ov9640 drivers and previous non merged implementations.
 *
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2006, OmniVision
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-device.h>

#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#define REG_CHIP_ID     0x00
#define PID_GC0308      0x9b

#define  gc0308_DEFAULT_WIDTH    640
#define  gc0308_DEFAULT_HEIGHT   480

/*
 * Struct
 */
struct regval_list {
	uint8_t reg_num;
	unsigned char value;
};

/* Supported resolutions */
enum gc0308_width {
	W_VGA   = gc0308_DEFAULT_WIDTH,
};

enum gc0308_height {
	H_VGA   = gc0308_DEFAULT_HEIGHT,
};

struct gc0308_win_size {
	char *name;
	enum gc0308_width width;
	enum gc0308_height height;
	const struct regval_list *regs;
	unsigned int mbus_code;
};

struct gc0308_priv {
	struct v4l2_subdev      subdev;
	struct v4l2_ctrl_handler    hdl;
	struct clk          *clk;
	struct clk          *mclk;
	u32  cfmt_code;
	const struct gc0308_win_size    *win;
	struct v4l2_subdev_platform_data *sd_pdata;
	struct gpio_desc *resetb_gpio;
	struct gpio_desc *pwdn_gpio;

	int (*power)(struct i2c_client *client, int on);
	int (*reset)(struct i2c_client *client);

	struct regulator *reg;
};

unsigned short gc0308_read_reg(struct i2c_client *client, unsigned char reg, unsigned char *value)
{
	struct i2c_msg msg[2] = {
		[0] = {
			.addr   = client->addr,
			.flags  = 0,
			.len    = 1,
			.buf    = &reg,
		},
		[1] = {
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.len    = 1,
			.buf    = value,
		}
	};
	int ret;
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret > 0) {
		ret = 0;
	}

	return ret;
}

int gc0308_write_reg(struct i2c_client *client, unsigned char reg, unsigned char value)
{
	unsigned char buf[2] = {reg, value};
	struct i2c_msg msg = {
		.addr   = client->addr,
		.flags  = 0,
		.len    = 2,
		.buf    = buf,
	};
	int ret;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0) {
		ret = 0;
	}

	return ret;
}

/*
 * Registers settings
 */
#define ENDMARKER { 0xff, 0xff }
static const struct regval_list gc0308_init_regs[] = {
	{0xfe, 0x80},
	{0xfe, 0x00},   // set page0
	{0xd2, 0x10},   // close AEC
	{0x22, 0x55},   // close AWB

	{0x03, 0x01},
	{0x04, 0x2c},
	{0x5a, 0x56},
	{0x5b, 0x40},
	{0x5c, 0x4a},

	{0x22, 0x57},   // Open AWB
	{0x01, 0xfa},
	{0x02, 0x70},
	{0x0f, 0x01},


	{0xe2, 0x00},   //anti-flicker step [11:8]
	{0xe3, 0x64},   //anti-flicker step [7:0]

	{0xe4, 0x02},   //exp level 1  16.67fps
	{0xe5, 0x58},
	{0xe6, 0x03},   //exp level 2  12.5fps
	{0xe7, 0x20},
	{0xe8, 0x04},   //exp level 3  8.33fps
	{0xe9, 0xb0},
	{0xea, 0x09},   //exp level 4  4.00fps
	{0xeb, 0xc4},

	{0x05, 0x00},
	{0x06, 0x00},
	{0x07, 0x00},
	{0x08, 0x00},
	{0x09, 0x01},
	{0x0a, 0xe8},
	{0x0b, 0x02},
	{0x0c, 0x88},
	{0x0d, 0x02},
	{0x0e, 0x02},
	{0x10, 0x22},
	{0x11, 0xfd},
	{0x12, 0x2a},
	{0x13, 0x00},

	{0x15, 0x0a},
	{0x16, 0x05},
	{0x17, 0x01},
	{0x18, 0x44},
	{0x19, 0x44},
	{0x1a, 0x1e},
	{0x1b, 0x00},
	{0x1c, 0xc1},
	{0x1d, 0x08},
	{0x1e, 0x60},
	{0x1f, 0x03},   //16


	{0x20, 0xff},
	{0x21, 0xf8},
	{0x22, 0x57},
	{0x24, 0xa2},
	{0x25, 0x0f},
	{0x28, 0x11},   //add

	{0x26, 0x03},// 03
	{0x2f, 0x01},
	{0x30, 0xf7},
	{0x31, 0x50},
	{0x32, 0x00},
	{0x39, 0x04},
	{0x3a, 0x18},
	{0x3b, 0x20},
	{0x3c, 0x00},
	{0x3d, 0x00},
	{0x3e, 0x00},
	{0x3f, 0x00},
	{0x50, 0x10},
	{0x53, 0x82},
	{0x54, 0x80},
	{0x55, 0x80},
	{0x56, 0x82},
	{0x8b, 0x40},
	{0x8c, 0x40},
	{0x8d, 0x40},
	{0x8e, 0x2e},
	{0x8f, 0x2e},
	{0x90, 0x2e},
	{0x91, 0x3c},
	{0x92, 0x50},
	{0x5d, 0x12},
	{0x5e, 0x1a},
	{0x5f, 0x24},
	{0x60, 0x07},
	{0x61, 0x15},
	{0x62, 0x08},
	{0x64, 0x03},
	{0x66, 0xe8},
	{0x67, 0x86},
	{0x68, 0xa2},
	{0x69, 0x18},
	{0x6a, 0x0f},
	{0x6b, 0x00},
	{0x6c, 0x5f},
	{0x6d, 0x8f},
	{0x6e, 0x55},
	{0x6f, 0x38},
	{0x70, 0x15},
	{0x71, 0x33},
	{0x72, 0xdc},
	{0x73, 0x80},
	{0x74, 0x02},
	{0x75, 0x3f},
	{0x76, 0x02},
	{0x77, 0x36},
	{0x78, 0x88},
	{0x79, 0x81},
	{0x7a, 0x81},
	{0x7b, 0x22},
	{0x7c, 0xff},
	{0x93, 0x48},
	{0x94, 0x00},
	{0x95, 0x05},
	{0x96, 0xe8},
	{0x97, 0x40},
	{0x98, 0xf0},
	{0xb1, 0x38},
	{0xb2, 0x38},
	{0xbd, 0x38},
	{0xbe, 0x36},
	{0xd0, 0xc9},
	{0xd1, 0x10},

	{0xd3, 0x80},
	{0xd5, 0xf2},
	{0xd6, 0x16},
	{0xdb, 0x92},
	{0xdc, 0xa5},
	{0xdf, 0x23},
	{0xd9, 0x00},
	{0xda, 0x00},
	{0xe0, 0x09},

	{0xed, 0x04},
	{0xee, 0xa0},
	{0xef, 0x40},
	{0x80, 0x03},
	{0x80, 0x03},
	{0x9F, 0x10},
	{0xA0, 0x20},
	{0xA1, 0x38},
	{0xA2, 0x4E},
	{0xA3, 0x63},
	{0xA4, 0x76},
	{0xA5, 0x87},
	{0xA6, 0xA2},
	{0xA7, 0xB8},
	{0xA8, 0xCA},
	{0xA9, 0xD8},
	{0xAA, 0xE3},
	{0xAB, 0xEB},
	{0xAC, 0xF0},
	{0xAD, 0xF8},
	{0xAE, 0xFD},
	{0xAF, 0xFF},
	{0xc0, 0x00},
	{0xc1, 0x10},
	{0xc2, 0x1C},
	{0xc3, 0x30},
	{0xc4, 0x43},
	{0xc5, 0x54},
	{0xc6, 0x65},
	{0xc7, 0x75},
	{0xc8, 0x93},
	{0xc9, 0xB0},
	{0xca, 0xCB},
	{0xcb, 0xE6},
	{0xcc, 0xFF},
	{0xf0, 0x02},
	{0xf1, 0x01},
	{0xf2, 0x01},
	{0xf3, 0x30},
	{0xf9, 0x9f},
	{0xfa, 0x78},

	//Registers of Page1
	{0xfe, 0x01},
	{0x00, 0xf5},
	{0x02, 0x1a},
	{0x0a, 0xa0},
	{0x0b, 0x60},
	{0x0c, 0x08},
	{0x0e, 0x4c},
	{0x0f, 0x39},
	{0x11, 0x3f},
	{0x12, 0x72},
	{0x13, 0x13},
	{0x14, 0x42},
	{0x15, 0x43},
	{0x16, 0xc2},
	{0x17, 0xa8},
	{0x18, 0x18},
	{0x19, 0x40},
	{0x1a, 0xd0},
	{0x1b, 0xf5},
	{0x70, 0x40},
	{0x71, 0x58},
	{0x72, 0x30},
	{0x73, 0x48},
	{0x74, 0x20},
	{0x75, 0x60},
	{0x77, 0x20},
	{0x78, 0x32},
	{0x30, 0x03},
	{0x31, 0x40},
	{0x32, 0xe0},
	{0x33, 0xe0},
	{0x34, 0xe0},
	{0x35, 0xb0},
	{0x36, 0xc0},
	{0x37, 0xc0},
	{0x38, 0x04},
	{0x39, 0x09},
	{0x3a, 0x12},
	{0x3b, 0x1C},
	{0x3c, 0x28},
	{0x3d, 0x31},
	{0x3e, 0x44},
	{0x3f, 0x57},
	{0x40, 0x6C},
	{0x41, 0x81},
	{0x42, 0x94},
	{0x43, 0xA7},
	{0x44, 0xB8},
	{0x45, 0xD6},
	{0x46, 0xEE},
	{0x47, 0x0d},

	//Registers of Page0
	{0xfe, 0x00}, // set page0
	{0x10, 0x26},
	{0x11, 0x0d},  // fd,modified by mormo 2010/07/06
	{0x1a, 0x2a},  // 1e,modified by mormo 2010/07/06

	{0x1c, 0x49}, // c1,modified by mormo 2010/07/06
	{0x1d, 0x9a}, // 08,modified by mormo 2010/07/06
	{0x1e, 0x61}, // 60,modified by mormo 2010/07/06

	{0x3a, 0x20},

	{0x50, 0x14},  // 10,modified by mormo 2010/07/06
	{0x53, 0x80},
	{0x56, 0x80},

	{0x8b, 0x20}, //LSC
	{0x8c, 0x20},
	{0x8d, 0x20},
	{0x8e, 0x14},
	{0x8f, 0x10},
	{0x90, 0x14},

	{0x94, 0x02},
	{0x95, 0x07},
	{0x96, 0xe0},

	{0xb1, 0x40}, // YCPT
	{0xb2, 0x40},
	{0xb3, 0x40},
	{0xb6, 0xe0},

	{0xd0, 0xcb}, // AECT  c9,modifed by mormo 2010/07/06
	{0xd3, 0x48}, // 80,modified by mormor 2010/07/06
	{0xf2, 0x02},
	{0xf7, 0x12},
	{0xf8, 0x0a},

	//Registers of Page1
	{0xfe, 0x01},// set page1
	{0x02, 0x20},
	{0x04, 0x10},
	{0x05, 0x08},
	{0x06, 0x20},
	{0x08, 0x0a},

	{0x0e, 0x44},
	{0x0f, 0x32},
	{0x10, 0x41},
	{0x11, 0x37},
	{0x12, 0x22},
	{0x13, 0x19},
	{0x14, 0x44},
	{0x15, 0x44},

	{0x19, 0x50},
	{0x1a, 0xd8},

	{0x32, 0x10},

	{0x35, 0x00},
	{0x36, 0x80},
	{0x37, 0x00},
	//-----------Update the registers end---------//

	{0xfe, 0x00}, // set page0
	{0xd2, 0x90},

	//-----------GAMMA Select(3)---------------//
	{0x9F, 0x10},
	{0xA0, 0x20},
	{0xA1, 0x38},
	{0xA2, 0x4E},
	{0xA3, 0x63},
	{0xA4, 0x76},
	{0xA5, 0x87},
	{0xA6, 0xA2},
	{0xA7, 0xB8},
	{0xA8, 0xCA},
	{0xA9, 0xD8},
	{0xAA, 0xE3},
	{0xAB, 0xEB},
	{0xAC, 0xF0},
	{0xAD, 0xF8},
	{0xAE, 0xFD},
	{0xAF, 0xFF},

	//{0x14, 0x10},
	//{0x14 ,0x12},
	//{0x14, 0x11},
	{0x14, 0x13},


	/*set_antibanding*/
	{0x01, 0x6a},
	{0x02, 0x70},
	{0x0f, 0x00},
	{0xe2, 0x00},

	{0xe3, 0x96},
	{0xe4, 0x02},
	{0xe5, 0x58},
	{0xe6, 0x02},
	{0xe7, 0x58},

	{0xe8, 0x02},
	{0xe9, 0x58},
	{0xea, 0x0b},
	{0xeb, 0xb8},

	ENDMARKER,
};

static const struct regval_list gc0308_vga_regs[] = {
	{0xfe, 0x01}, {0x54, 0x11},
	{0x55, 0x03}, {0x56, 0x00},
	{0x57, 0x00}, {0x58, 0x00},
	{0x59, 0x00}, {0xfe, 0x00},
	{0x46, 0x00},
	ENDMARKER,
};

#if 1
static const struct regval_list gc0308_wb_auto_regs[] = {
	{0x5a, 0x56}, {0x5b, 0x40},
	{0x5c, 0x4a}, {0x22, 0x57},
	ENDMARKER,
};

static const struct regval_list gc0308_enable_output_regs[] = {
	{0xfe, 0x00},
	{0x25, 0x0f},
	ENDMARKER,
};

static const struct regval_list gc0308_disable_output_regs[] = {
	{0xfe, 0x00},
	{0x25, 0x00},
	ENDMARKER,
};

static const struct regval_list gc0308_effect_normal_regs[] = {
	{0x23, 0x00}, {0x2d, 0x0a},
	{0x20, 0x7f}, {0xd2, 0x90},
	{0x73, 0x00}, {0x77, 0x38},
	{0xb3, 0x40}, {0xb4, 0x80},
	{0xba, 0x00}, {0xbb, 0x00},
	ENDMARKER,
};
#endif

#define gc0308_SIZE(n, w, h, r, c) \
	{.name = n, .width = w , .height = h, .regs = r, .mbus_code = c }

static struct gc0308_win_size gc0308_supported_win_sizes[] = {
	gc0308_SIZE("VGA", W_VGA, H_VGA, gc0308_vga_regs, MEDIA_BUS_FMT_YUYV8_2X8),
};

#define N_WIN_SIZES (ARRAY_SIZE(gc0308_supported_win_sizes))

/*
 * General functions
 */
static struct gc0308_priv *to_gc0308(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct gc0308_priv,
	                    subdev);
}

static int gc0308_write_array(struct i2c_client *client,
                              const struct regval_list *vals)
{
	int ret;

	while ((vals->reg_num != 0xff) || (vals->value != 0xff)) {
		ret = gc0308_write_reg(client, vals->reg_num, vals->value);

		if (ret < 0) {
			return ret;
		}
		vals++;
		udelay(1000);
	}
	return 0;
}

/* OF probe functions */
static int gc0308_hw_power(struct i2c_client *client, int on)
{
	struct gc0308_priv *priv = to_gc0308(client);

	printk("%s: %s the camera\n", __func__, on ? "ENABLE" : "DISABLE");
	/* thses gpio should be set according to the active level in dt defines */

	msleep(5);
	if (desc_to_gpio(priv->pwdn_gpio)) {
		gpiod_direction_output(priv->pwdn_gpio, 0);
		msleep(10);
	}
	return 0;
}


static int gc0308_reset(struct i2c_client *client)
{
	struct gc0308_priv *priv = to_gc0308(client);

	if (priv->resetb_gpio) {
		gpiod_direction_output(priv->resetb_gpio, 1);
		msleep(10);
		gpiod_direction_output(priv->resetb_gpio, 0);
		msleep(10);
	}

	return 0;
}

static int gc0308_s_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int gc0308_g_register(struct v4l2_subdev *sd,
                             struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	reg->size = 1;
	if (reg->reg > 0xff) {
		return -EINVAL;
	}

	ret = gc0308_read_reg(client, reg->reg, reg->val);

	return 0;
}

static int gc0308_s_register(struct v4l2_subdev *sd,
                             const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg > 0xff ||
	    reg->val > 0xff) {
		return -EINVAL;
	}

	return gc0308_write_reg(client, reg->reg, reg->val);
}
#endif

static int gc0308_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	if (!enable) {
		dev_info(&client->dev, "stream down\n");
		gc0308_write_array(client, gc0308_disable_output_regs);
		return 0;
	}
	dev_info(&client->dev, "stream on\n");
	gc0308_write_array(client, gc0308_enable_output_regs);

	return 0;

}

/* Select the nearest higher resolution for capture */
static const struct gc0308_win_size *gc0308_select_win(u32 code, u32 *width, u32 *height)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(gc0308_supported_win_sizes); i++) {
		if ((*width >= gc0308_supported_win_sizes[i].width) &&
		    (*height >= gc0308_supported_win_sizes[i].height) &&
		    (code == gc0308_supported_win_sizes[i].mbus_code)) {
			*width = gc0308_supported_win_sizes[i].width;
			*height = gc0308_supported_win_sizes[i].height;
			return &gc0308_supported_win_sizes[i];
		}
	}
	return NULL;
}

static int gc0308_get_selection(struct v4l2_subdev *sd,
                                struct v4l2_subdev_state *state,
                                struct v4l2_subdev_selection *sel)
{
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct gc0308_priv *priv = to_gc0308(client);

	if (!priv->win) {
		return -EINVAL;
	}

	sel->r.top = 0;
	sel->r.left = 0;
	sel->r.width = priv->win->width;
	sel->r.height = priv->win->height;

	return 0;
}

static int gc0308_get_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct gc0308_priv *priv = to_gc0308(client);

	if (format->pad) {
		return -EINVAL;
	}
	if (priv->win) {
		mf->width = priv->win->width;
		mf->height = priv->win->height;
	} else {
		mf->width = gc0308_DEFAULT_WIDTH;
		mf->height = gc0308_DEFAULT_HEIGHT;
	}
	mf->code = priv->cfmt_code;

	mf->colorspace = V4L2_COLORSPACE_JPEG;
	mf->field   = V4L2_FIELD_NONE;

	return 0;
}

static int gc0308_set_params(struct i2c_client *client, u32 *width, u32 *height, u32 code)
{
	struct gc0308_priv *priv = to_gc0308(client);
	int ret;

	/* initialize the sensor with default data */
	ret = gc0308_write_array(client, gc0308_init_regs);
	if (ret < 0) {
		goto err;
	}

	/* set balance */
	ret = gc0308_write_array(client, gc0308_wb_auto_regs);
	if (ret < 0) {
		goto err;
	}

	/* set effect */
	ret = gc0308_write_array(client, gc0308_effect_normal_regs);
	if (ret < 0) {
		goto err;
	}

	/* set size win */
	ret = gc0308_write_array(client, priv->win->regs);
	if (ret < 0) {
		goto err;
	}

	priv->cfmt_code = code;
	*width = priv->win->width;
	*height = priv->win->height;

	return 0;

err:
	dev_err(&client->dev, "%s: Error %d", __func__, ret);
	gc0308_reset(client);
	priv->win = NULL;

	return ret;
}

static int gc0308_set_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc0308_priv *priv = to_gc0308(client);

	if (format->pad) {
		return -EINVAL;
	}

	/*
	 * select suitable win, but don't store it
	 */
	priv->win = gc0308_select_win(mf->code, &mf->width, &mf->height);
	if (!priv->win) {
		return -EINVAL;
	}

	mf->field   = V4L2_FIELD_NONE;
	mf->colorspace        = V4L2_COLORSPACE_JPEG;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		gc0308_set_params(client, &mf->width, &mf->height, mf->code);
	}
	return 0;
}

static int gc0308_enum_mbus_code(struct v4l2_subdev *sd,
                                 struct v4l2_subdev_state *state,
                                 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(gc0308_supported_win_sizes)) {
		return -EINVAL;
	}

	code->code = gc0308_supported_win_sizes[code->index].mbus_code;
	return 0;
}

static int gc0308_enum_frame_size(struct v4l2_subdev *sd,
                                  struct v4l2_subdev_state *state,
                                  struct v4l2_subdev_frame_size_enum *fse)
{
	__u32 index = fse->index;

	if (index >= N_WIN_SIZES) {
		return -EINVAL;
	}
	fse->code = gc0308_supported_win_sizes[index].mbus_code;
	fse->min_width = gc0308_supported_win_sizes[index].width;
	fse->max_width = fse->min_width;
	fse->min_height = gc0308_supported_win_sizes[index].height;
	fse->max_height = fse->min_height;
	return 0;
}

static struct v4l2_subdev_core_ops gc0308_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = gc0308_g_register,
	.s_register = gc0308_s_register,
#endif
	.s_power    = gc0308_s_power,
};

static struct v4l2_subdev_video_ops gc0308_subdev_video_ops = {
	.s_stream   = gc0308_s_stream,
};

static const struct v4l2_subdev_pad_ops gc0308_subdev_pad_ops = {
	.enum_mbus_code = gc0308_enum_mbus_code,
	.enum_frame_size = gc0308_enum_frame_size,
	.get_fmt    = gc0308_get_fmt,
	.set_fmt    = gc0308_set_fmt,
	.get_selection  = gc0308_get_selection,
};

static struct v4l2_subdev_ops gc0308_subdev_ops = {
	.core   = &gc0308_subdev_core_ops,
	.video  = &gc0308_subdev_video_ops,
	.pad    = &gc0308_subdev_pad_ops,
};

static int gc0308_video_probe(struct i2c_client *client)
{
	unsigned char retval;
	int ret = 0;

	gc0308_hw_power(client, 1);
	/*
	 * check and show product ID and manufacturer ID
	 */

	ret = gc0308_read_reg(client, REG_CHIP_ID, &retval);
	if (ret < 0) {
		return ret;
	}
	if (retval != PID_GC0308) {
		dev_err(&client->dev, "read sensor %s chip_id high %x is error\n",
		        client->name, retval);
		return -1;
	}

	dev_info(&client->dev, "read sensor %s id value:0x%x successed!\n",
	         client->name, retval);

	return 0;
}

static int gc0308_probe_dt(struct i2c_client *client,
                           struct gc0308_priv *priv)
{
	struct v4l2_subdev_platform_data *sd_pdata = priv->sd_pdata;
	struct device_node *np = client->dev.of_node;
	int supplies = 0, index = 0;
	int ret = 0;

	supplies = of_property_count_strings(np, "supplies-name");
	if (supplies > 0) {
		sd_pdata = devm_kzalloc(&client->dev, sizeof(struct v4l2_subdev_platform_data), GFP_KERNEL);
		sd_pdata->num_regulators = supplies;
		sd_pdata->regulators = devm_kzalloc(&client->dev, supplies * sizeof(struct regulator_bulk_data), GFP_KERNEL);
		if (!sd_pdata->regulators) {
			dev_err(&client->dev, "Failed to allocate regulators.!\n");
			devm_kfree(&client->dev, sd_pdata);
			return -ENOMEM;
		}
		for (index = 0; index < sd_pdata->num_regulators; index ++) {
			of_property_read_string_index(np, "supplies-name", index,
			                              &(sd_pdata->regulators[index].supply));
			dev_dbg(&client->dev, "sd_pdata->regulators[%d].supply: %s\n",
			        index, sd_pdata->regulators[index].supply);
		}
		ret = devm_regulator_bulk_get(&client->dev, sd_pdata->num_regulators, sd_pdata->regulators);
		if (ret) {
			dev_dbg(&client->dev, "get regulator comsumers failed!\n");
		}
	} else {
		/* Request the reset GPIO deasserted */
		priv->resetb_gpio = devm_gpiod_get_optional(&client->dev, "resetb",
		                    GPIOD_OUT_LOW);
		if (!priv->resetb_gpio) {
			dev_dbg(&client->dev, "resetb gpio is not assigned!\n");
		} else if (IS_ERR(priv->resetb_gpio)) {
			return PTR_ERR(priv->resetb_gpio);
		}

		/* Request the power down GPIO asserted */
		priv->pwdn_gpio = devm_gpiod_get_optional(&client->dev, "pwdn",
		                  GPIOD_OUT_LOW);
		if (!priv->pwdn_gpio) {
			dev_dbg(&client->dev, "pwdn gpio is not assigned!\n");
		} else if (IS_ERR(priv->pwdn_gpio)) {
			return PTR_ERR(priv->pwdn_gpio);
		}


	}
	return 0;
}

/*
 * i2c_driver functions
 */
static int gc0308_probe(struct i2c_client *client)
{
	struct gc0308_priv  *priv;
	struct i2c_adapter  *adapter = to_i2c_adapter(client->dev.parent);
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE
	                             | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&adapter->dev, "gc0308: I2C-Adapter doesn't support SMBUS\n");
		return -EIO;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct gc0308_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&adapter->dev,
		        "Failed to allocate memory for private data!\n");
		return -ENOMEM;
	}

	priv->mclk = devm_clk_get(&client->dev, "gate_cim_mclk");
	if (IS_ERR(priv->clk)) {
		dev_err(&adapter->dev, "mclk_get err!\n");
		return -EPROBE_DEFER;
	}

	ret = clk_prepare_enable(priv->mclk);
	if (ret) {
		dev_err(&adapter->dev, "mclk_enable err!\n");
		return -EPROBE_DEFER;
	}

	priv->clk = devm_clk_get(&client->dev, "div_cim");
	if (IS_ERR(priv->clk)) {
		return -EPROBE_DEFER;
	}

	clk_set_rate(priv->clk, 24000000);

	ret = gc0308_probe_dt(client, priv);
	if (ret) {
		goto err_clk;
	}

	v4l2_i2c_subdev_init(&priv->subdev, client, &gc0308_subdev_ops);

	/* add handler */
	v4l2_ctrl_handler_init(&priv->hdl, 2);

	ret = gc0308_video_probe(client);
	if (ret < 0) {
		goto err_videoprobe;
	}

	ret = v4l2_async_register_subdev(&priv->subdev);
	if (ret < 0) {
		goto err_videoprobe;
	}

	printk("gc0308 Probed\n");

	return 0;

err_videoprobe:
	v4l2_ctrl_handler_free(&priv->hdl);
err_clk:
	devm_clk_put(&client->dev, priv->clk);
	devm_clk_put(&client->dev, priv->mclk);
	devm_kfree(&client->dev, priv);
	return ret;
}

static void gc0308_remove(struct i2c_client *client)
{
	struct gc0308_priv *priv = to_gc0308(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	devm_clk_put(&client->dev, priv->clk);
	devm_clk_put(&client->dev, priv->mclk);
	v4l2_device_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);
}

static const struct i2c_device_id gc0308_id[] = {
	{ "gc0308",  0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, gc0308_id);
static const struct of_device_id gc0308_of_match[] = {
	{.compatible = "gc0308", },
	{},
};
MODULE_DEVICE_TABLE(of, gc0308_of_match);
static struct i2c_driver gc0308_i2c_driver = {
	.driver = {
		.name = "gc0308",
		.of_match_table = of_match_ptr(gc0308_of_match),
	},
	.probe    = gc0308_probe,
	.remove   = gc0308_remove,
	.id_table = gc0308_id,
};

module_i2c_driver(gc0308_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for smartsens gc0308 sensor");
MODULE_LICENSE("GPL v2");
