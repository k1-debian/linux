/*
 * ov9281 Camera Driver
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

#include <media/v4l2-clk.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-device.h>
#include <ingenic_camera.h>

#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#define REG_CHIP_ID_HIGH    0x300a
#define REG_CHIP_ID_LOW     0x300b
#define CHIP_ID_HIGH        0x92
#define CHIP_ID_LOW     0x81

#define  OV9281_DEFAULT_WIDTH    1280
#define  OV9281_DEFAULT_HEIGHT   720
/* #define  OV9281_DEFAULT_WIDTH    640 */
/* #define  OV9281_DEFAULT_HEIGHT   480 */

/* Private v4l2 controls */
#define V4L2_CID_PRIVATE_BALANCE  (V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_PRIVATE_EFFECT  (V4L2_CID_PRIVATE_BASE + 1)

#define REG_TC_VFLIP            0x3820
#define REG_TC_MIRROR           0x3821
#define OV9281_FLIP_VAL         ((unsigned char)0x04)
#define OV9281_FLIP_MASK        ((unsigned char)0x04)

/* whether sensor support high resolution (> vga) preview or not */
#define SUPPORT_HIGH_RESOLUTION_PRE     1

/*
 * Struct
 */
struct regval_list {
	u16 reg_num;
	u16 value;
};

struct mode_list {
	u16 index;
	const struct regval_list *mode_regs;
};

/* Supported resolutions */
enum ov9281_width {
	W_720P  = OV9281_DEFAULT_WIDTH,
	/* W_640 = OV9281_DEFAULT_WIDTH, */
};

enum ov9281_height {
	H_720P  = OV9281_DEFAULT_HEIGHT,
	/* H_480 = OV9281_DEFAULT_HEIGHT, */
};

struct ov9281_win_size {
	char *name;
	enum ov9281_width width;
	enum ov9281_height height;
	const struct regval_list *regs;
	unsigned int mbus_code;
};

struct ov9281_priv {
	struct v4l2_subdev      subdev;
	struct v4l2_ctrl_handler    hdl;
	u32 cfmt_code;
	struct v4l2_clk         *clk;
	struct v4l2_clk         *mclk;
	const struct ov9281_win_size    *win;

	int             model;
	u16             balance_value;
	u16             effect_value;
	u16             flag_vflip: 1;
	u16             flag_hflip: 1;

	struct v4l2_subdev_platform_data *sd_pdata;
	struct gpio_desc *resetb_gpio;
	struct gpio_desc *pwdn_gpio;
	struct gpio_desc *vcc_en_gpio;

	int (*power)(struct i2c_client *client, int on);
	int (*reset)(struct i2c_client *client);

	struct regulator *reg;
	struct regulator *reg1;
};

static inline int sensor_i2c_master_send(struct i2c_client *client,
        const char *buf, int count)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (char *)buf;

	ret = i2c_transfer(adap, &msg, 1);

	return (ret == 1) ? count : ret;
}

static inline int sensor_i2c_master_recv(struct i2c_client *client,
        char *buf, int count)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = buf;
	ret = i2c_transfer(adap, &msg, 1);

	return (ret == 1) ? count : ret;
}

unsigned char ov9281_read_reg(struct i2c_client *client, u16 reg)
{
	int ret;
	unsigned char retval;
	unsigned short r = cpu_to_be16(reg);

	ret = sensor_i2c_master_send(client, (u8 *)&r, 2);

	if (ret < 0) {
		return ret;
	}
	if (ret != 2) {
		return -EIO;
	}

	ret = sensor_i2c_master_recv(client, &retval, 1);
	if (ret < 0) {
		return ret;
	}
	if (ret != 1) {
		return -EIO;
	}
	return retval;
}

int ov9281_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	unsigned char msg[3];
	int ret;

	reg = cpu_to_be16(reg);

	memcpy(&msg[0], &reg, 2);
	memcpy(&msg[2], &val, 1);

	ret = sensor_i2c_master_send(client, msg, 3);

	if (ret < 0) {
		printk("RET<0\n");
		return ret;
	}
	if (ret < 3) {
		printk("RET<3\n");
		return -EIO;
	}

	return 0;
}

/*
 * Registers settings
 */

#define ENDMARKER { 0xffff, 0xff }

static const struct regval_list ov9281_init_regs[] = {
	{ 0x0103, 0x01 },
	{ 0x0302, 0x30 },
	{ 0x030d, 0x60 },
	{ 0x030e, 0x06 },
	{ 0x3001, 0x62 },
	{ 0x3004, 0x01 },
	{ 0x3005, 0xff },

	{ 0x3667, 0xda }, //choose PWM pin for DVP vsync output
	{ 0x3006, 0xe6 }, //[2]1:enable output for PWM pin ; [1]0:enable input for FSIN pin
	{ 0x0100, 0x01 },

	{ 0x3011, 0x0a },
	{ 0x3013, 0x18 },
	{ 0x3022, 0x07 },
	{ 0x3030, 0x10 },
	{ 0x3039, 0x2e },
	{ 0x303a, 0xf0 },
	{ 0x3500, 0x00 },
	{ 0x3501, 0x2a },
	{ 0x3502, 0x90 },
	{ 0x3503, 0x08 },
	{ 0x3505, 0x8c },
	{ 0x3507, 0x03 },
	{ 0x3508, 0x00 },
	//  { 0x3509, 0x40 },
	{ 0x3509, 0x30 },
	{ 0x3610, 0x80 },
	{ 0x3611, 0xa0 },
	{ 0x3620, 0x6f },
	{ 0x3632, 0x56 },
	{ 0x3633, 0x78 },
	{ 0x3662, 0x03 },
	{ 0x3666, 0x5a },
	{ 0x366f, 0x7e },
	{ 0x3680, 0x84 },
	{ 0x3712, 0x80 },
	{ 0x372d, 0x22 },
	{ 0x3731, 0x80 },
	{ 0x3732, 0x30 },
	{ 0x3778, 0x00 },
	{ 0x377d, 0x22 },
	{ 0x3788, 0x02 },
	{ 0x3789, 0xa4 },
	{ 0x378a, 0x00 },
	{ 0x378b, 0x4a },
	{ 0x3799, 0x20 },
	{ 0x3800, 0x00 },
	{ 0x3801, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3803, 0x00 },
	{ 0x3804, 0x05 },
	{ 0x3805, 0x0f },
	{ 0x3806, 0x03 },
	{ 0x3807, 0x2f },
	{ 0x3808, 0x05 },
	{ 0x3809, 0x00 },
	{ 0x380a, 0x02 },
	{ 0x380b, 0xd0 },
	{ 0x380c, 0x02 },
	{ 0x380d, 0xd8 },
	{ 0x380e, 0x03 },
	{ 0x380f, 0x54 },
	{ 0x3810, 0x00 },
	{ 0x3811, 0x08 },
	{ 0x3812, 0x00 },
	{ 0x3813, 0x08 },
	{ 0x3814, 0x11 },
	{ 0x3815, 0x11 },
	{ 0x3820, 0x40 },
	{ 0x3821, 0x00 },
	{ 0x3881, 0x42 },
	{ 0x38b1, 0x00 },
	{ 0x3920, 0xff },
	{ 0x4003, 0x40 },
	{ 0x4008, 0x04 },
	{ 0x4009, 0x0b },
	{ 0x400c, 0x00 },
	{ 0x400d, 0x07 },
	{ 0x4010, 0x40 },
	{ 0x4043, 0x40 },
	{ 0x4307, 0x30 },
	{ 0x4317, 0x01 },
	{ 0x4501, 0x00 },
	{ 0x4507, 0x00 },
	{ 0x4509, 0x00 },
	{ 0x450a, 0x08 },
	{ 0x4601, 0x04 },
	{ 0x470f, 0xe0 },
	{ 0x4f07, 0x00 },
	{ 0x4800, 0x00 },
	{ 0x5000, 0x9f },
	{ 0x5001, 0x00 },
	{ 0x5e00, 0x00 },
	{ 0x5d00, 0x0b },
	{ 0x5d01, 0x02 },

	{ 0x4f00, 0x01},
	{ 0x0100, 0x00},

	{ 0x320c, 0x8f},
	{ 0x302c, 0x00},
	{ 0x302d, 0x00},
	{ 0x302e, 0x00},
	{ 0x302f, 0x02},
	{ 0x303f, 0x01},
	{ 0x3023, 0x07},
	{ 0x4242, 0x00},

	//  { 0x3030, 0x80},
	//  { 0x3030, 0x10},
	//  { 0x3030, 0x90},
	{ 0x3030, 0x80},
	{ 0x3030, 0x84},

	ENDMARKER,
};

static const struct regval_list ov9281_wb_auto_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov9281_wb_incandescence_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov9281_wb_daylight_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov9281_wb_fluorescent_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov9281_wb_cloud_regs[] = {
	ENDMARKER,
};

static const struct mode_list ov9281_balance[] = {
	{0, ov9281_wb_auto_regs}, {1, ov9281_wb_incandescence_regs},
	{2, ov9281_wb_daylight_regs}, {3, ov9281_wb_fluorescent_regs},
	{4, ov9281_wb_cloud_regs},
};

static const struct regval_list ov9281_effect_normal_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov9281_effect_grayscale_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov9281_effect_sepia_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov9281_effect_colorinv_regs[] = {
	ENDMARKER,
};

static const struct regval_list ov9281_effect_sepiabluel_regs[] = {
	ENDMARKER,
};

static const struct mode_list ov9281_effect[] = {
	{0, ov9281_effect_normal_regs}, {1, ov9281_effect_grayscale_regs},
	{2, ov9281_effect_sepia_regs}, {3, ov9281_effect_colorinv_regs},
	{4, ov9281_effect_sepiabluel_regs},
};

static const struct regval_list ov9281_720p_regs[] = {
	{0x4708, 0x0},
	{0x3808, 0x05},
	{0x3809, 0x00},     /* ISP Horizontal Output Width: 1280 */
	{0x380a, 0x02},
	{0x380b, 0xd0},     /* ISP Vertical Output Width : 720*/
	{0x380c, 0x02},
	{0x380d, 0xd8},     /* HTS: Horizontal Timing Size: 728 */
	{0x380e, 0x03},
	{0x380f, 0x8e},     /* VTS: Vertical Timing Size: 910 */
	ENDMARKER,
};

static const struct regval_list ov9281_vga_regs[] = {
	{ 0x3808, 0x02 },
	{ 0x3809, 0x80 },
	{ 0x380a, 0x01 },
	{ 0x380b, 0xe0 },
	{ 0x380c, 0x02 },
	{ 0x380d, 0xd8 },
	{ 0x380e, 0x03 },
	{ 0x380f, 0x54 },
	ENDMARKER,
};

#define OV9281_SIZE(n, w, h, r, c) \
	{.name = n, .width = w , .height = h, .regs = r, .mbus_code = c }

static struct ov9281_win_size ov9281_supported_win_sizes[] = {
	OV9281_SIZE("720P", W_720P, H_720P, ov9281_720p_regs, MEDIA_BUS_FMT_Y8_1X8),
	/* OV9281_SIZE("vga", W_640,H_480,ov9281_vga_regs), */
};

#define N_WIN_SIZES (ARRAY_SIZE(ov9281_supported_win_sizes))
/*
static u32 ov9281_codes[] = {
    MEDIA_BUS_FMT_Y8_1X8,
};
*/
/*
 * Supported balance menus
 */
static const struct v4l2_querymenu ov9281_balance_menus[] = {
	{
		.id     = V4L2_CID_PRIVATE_BALANCE,
		.index      = 0,
		.name       = "auto",
	}, {
		.id     = V4L2_CID_PRIVATE_BALANCE,
		.index      = 1,
		.name       = "incandescent",
	}, {
		.id     = V4L2_CID_PRIVATE_BALANCE,
		.index      = 2,
		.name       = "fluorescent",
	},  {
		.id     = V4L2_CID_PRIVATE_BALANCE,
		.index      = 3,
		.name       = "daylight",
	},  {
		.id     = V4L2_CID_PRIVATE_BALANCE,
		.index      = 4,
		.name       = "cloudy-daylight",
	},

};

/*
 * Supported effect menus
 */
static const struct v4l2_querymenu ov9281_effect_menus[] = {
	{
		.id     = V4L2_CID_PRIVATE_EFFECT,
		.index      = 0,
		.name       = "none",
	}, {
		.id     = V4L2_CID_PRIVATE_EFFECT,
		.index      = 1,
		.name       = "mono",
	}, {
		.id     = V4L2_CID_PRIVATE_EFFECT,
		.index      = 2,
		.name       = "sepia",
	},  {
		.id     = V4L2_CID_PRIVATE_EFFECT,
		.index      = 3,
		.name       = "negative",
	}, {
		.id     = V4L2_CID_PRIVATE_EFFECT,
		.index      = 4,
		.name       = "aqua",
	},
};

/*
 * General functions
 */
static struct ov9281_priv *to_ov9281(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov9281_priv,
	                    subdev);
}

static int ov9281_write_array(struct i2c_client *client,
                              const struct regval_list *vals)
{
	int ret;

	while ((vals->reg_num != 0xffff) || (vals->value != 0xff)) {
		ret = ov9281_write_reg(client, vals->reg_num, vals->value);
		dev_vdbg(&client->dev, "array: 0x%02x, 0x%02x",
		         vals->reg_num, vals->value);

		if (ret < 0) {
			return ret;
		}
		vals++;
	}
	return 0;
}
#if 1
static int ov9281_mask_set(struct i2c_client *client,
                           u16  reg, u16  mask, u16  set)
{
	s32 val = ov9281_read_reg(client, reg);
	if (val < 0) {
		return val;
	}

	val &= ~mask;
	val |= set & mask;

	dev_vdbg(&client->dev, "masks: 0x%02x, 0x%02x", reg, val);

	return ov9281_write_reg(client, reg, val);
}
#endif

/* OF probe functions */
static int ov9281_hw_power(struct i2c_client *client, int on)
{
	struct ov9281_priv *priv = to_ov9281(client);

	dev_dbg(&client->dev, "%s: %s the camera\n",
	        __func__, on ? "ENABLE" : "DISABLE");

	/* thses gpio should be set according to the active level in dt defines */
	if (priv->vcc_en_gpio) {
		gpiod_direction_output(priv->vcc_en_gpio, on);
	}

	if (priv->pwdn_gpio) {
		gpiod_direction_output(priv->pwdn_gpio, on);
	}

	msleep(10);
	return 0;
}

static int ov9281_reset(struct i2c_client *client)
{
	struct ov9281_priv *priv = to_ov9281(client);

	if (priv->resetb_gpio) {
		gpiod_direction_output(priv->resetb_gpio, 1);
		msleep(10);
		gpiod_direction_output(priv->resetb_gpio, 0);
		msleep(10);
	}

	return 0;
}

static int ov9281_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd =
	    &container_of(ctrl->handler, struct ov9281_priv, hdl)->subdev;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ov9281_priv *priv = to_ov9281(client);

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		ctrl->val = priv->flag_vflip;
		break;
	case V4L2_CID_HFLIP:
		ctrl->val = priv->flag_hflip;
		break;
	case V4L2_CID_PRIVATE_BALANCE:
		ctrl->val = priv->balance_value;
		break;
	case V4L2_CID_PRIVATE_EFFECT:
		ctrl->val = priv->effect_value;
		break;
	default:
		break;
	}
	return 0;
}

static int ov9281_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd =
	    &container_of(ctrl->handler, struct ov9281_priv, hdl)->subdev;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ov9281_priv *priv = to_ov9281(client);
	int ret = 0;
	int i = 0;
	u16 value;

	int balance_count = ARRAY_SIZE(ov9281_balance);
	int effect_count = ARRAY_SIZE(ov9281_effect);

	switch (ctrl->id) {
	case V4L2_CID_PRIVATE_BALANCE:
		if (ctrl->val > balance_count) {
			return -EINVAL;
		}

		for (i = 0; i < balance_count; i++) {
			if (ctrl->val == ov9281_balance[i].index) {
				ret = ov9281_write_array(client,
				                         ov9281_balance[ctrl->val].mode_regs);
				priv->balance_value = ctrl->val;
				break;
			}
		}
		break;

	case V4L2_CID_PRIVATE_EFFECT:
		if (ctrl->val > effect_count) {
			return -EINVAL;
		}

		for (i = 0; i < effect_count; i++) {
			if (ctrl->val == ov9281_effect[i].index) {
				ret = ov9281_write_array(client,
				                         ov9281_effect[ctrl->val].mode_regs);
				priv->effect_value = ctrl->val;
				break;
			}
		}
		break;

	case V4L2_CID_VFLIP:
		value = ctrl->val ? OV9281_FLIP_VAL : 0x00;
		priv->flag_vflip = ctrl->val ? 1 : 0;
		ret = ov9281_mask_set(client, REG_TC_VFLIP, OV9281_FLIP_MASK, value);
		break;

	case V4L2_CID_HFLIP:
		value = ctrl->val ? OV9281_FLIP_VAL : 0x00;
		priv->flag_hflip = ctrl->val ? 1 : 0;
		ret = ov9281_mask_set(client, REG_TC_MIRROR, OV9281_FLIP_MASK, value);
		break;

	default:
		dev_err(&client->dev, "no V4L2 CID: 0x%x ", ctrl->id);
		return -EINVAL;
	}

	return ret;
}

static const struct v4l2_ctrl_ops ov9281_ctrl_ops = {
	.s_ctrl = ov9281_s_ctrl,
	.g_volatile_ctrl = ov9281_g_ctrl,
};
#if 0
static int ov9281_querymenu(struct v4l2_subdev *sd,
                            struct v4l2_querymenu *qm)
{
	switch (qm->id) {
	case V4L2_CID_PRIVATE_BALANCE:
		memcpy(qm->name, ov9281_balance_menus[qm->index].name,
		       sizeof(qm->name));
		break;

	case V4L2_CID_PRIVATE_EFFECT:
		memcpy(qm->name, ov9281_effect_menus[qm->index].name,
		       sizeof(qm->name));
		break;
	}

	return 0;
}
#endif

static int ov9281_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov9281_priv *priv = to_ov9281(client);

	if (priv->sd_pdata) {
		if (on) {
			return regulator_bulk_enable(priv->sd_pdata->num_regulators, priv->sd_pdata->regulators);
		} else {
			return regulator_bulk_disable(priv->sd_pdata->num_regulators, priv->sd_pdata->regulators);
		}
	} else if (priv->power) {
		return priv->power(client, on);
	} else {
		dev_err(&client->dev, "ov9281_s_power failde");
		return -EINVAL;
	}

	//  return soc_camera_set_power(&client->dev, ssdd, priv->clk, on);
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov9281_g_register(struct v4l2_subdev *sd,
                             struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	reg->size = 1;
	if (reg->reg > 0xff) {
		return -EINVAL;
	}

	ret = ov9281_read_reg(client, reg->reg);
	if (ret < 0) {
		return ret;
	}

	reg->val = ret;

	return 0;
}

static int ov9281_s_register(struct v4l2_subdev *sd,
                             const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg > 0xff ||
	    reg->val > 0xff) {
		return -EINVAL;
	}

	return ov9281_write_reg(client, reg->reg, reg->val);
}
#endif

/*
 * soc_camera_ops functions
 */
static int ov9281_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client  *client = v4l2_get_subdevdata(sd);

	if (!enable) {
		dev_info(&client->dev, "stream down\n");
		ov9281_write_reg(client, 0x0100, 0x00);
		return 0;
	}

	dev_info(&client->dev, "stream on\n");
	ov9281_write_reg(client, 0x0100, 0x01);

	return 0;
}

/* Select the nearest higher resolution for capture */
static const struct ov9281_win_size *ov9281_select_win(u32 *code, u32 *width, u32 *height)
{
	int i, default_size = ARRAY_SIZE(ov9281_supported_win_sizes) - 1;

	for (i = 0; i < ARRAY_SIZE(ov9281_supported_win_sizes); i++) {
		if ((*width >= ov9281_supported_win_sizes[i].width) &&
		    (*height >= ov9281_supported_win_sizes[i].height) &&
		    (*code == ov9281_supported_win_sizes[i].mbus_code)) {
			*width = ov9281_supported_win_sizes[i].width;
			*height = ov9281_supported_win_sizes[i].height;
			return &ov9281_supported_win_sizes[i];
		}
	}
	return NULL;
}

static int ov9281_get_selection(struct v4l2_subdev *sd,
                                struct v4l2_subdev_pad_config *cfg,
                                struct v4l2_subdev_selection *sel)
{
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ov9281_priv *priv = to_ov9281(client);

	if (!priv->win) {
		return -EINVAL;
	}

	sel->r.top = 0;
	sel->r.left = 0;
	sel->r.width = priv->win->width;
	sel->r.height = priv->win->height;

	return 0;
}

static int ov9281_get_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_pad_config *cfg,
                          struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client  *client = v4l2_get_subdevdata(sd);
	struct ov9281_priv *priv = to_ov9281(client);

	if (format->pad) {
		return -EINVAL;
	}

	if (priv->win) {
		mf->width = priv->win->width;
		mf->height = priv->win->height;
	} else {
		mf->width = OV9281_DEFAULT_WIDTH;
		mf->height = OV9281_DEFAULT_HEIGHT;
	}
	mf->code = priv->cfmt_code;

	mf->colorspace = V4L2_COLORSPACE_JPEG;
	mf->field   = V4L2_FIELD_NONE;

	return 0;
}

static int ov9281_set_params(struct i2c_client *client, u32 *width, u32 *height, u32 code)
{
	struct ov9281_priv       *priv = to_ov9281(client);
	int ret;

	int bala_index = priv->balance_value;
	int effe_index = priv->effect_value;

	/* select win */
	priv->win = ov9281_select_win(&code, width, height);

	/* select format */
	priv->cfmt_code = 0;

	/* reset hardware */
	ov9281_reset(client);

	/* initialize the sensor with default data */
	dev_dbg(&client->dev, "%s: Init default", __func__);
	ret = ov9281_write_array(client, ov9281_init_regs);
	if (ret < 0) {
		goto err;
	}

	/* set balance */
	ret = ov9281_write_array(client, ov9281_balance[bala_index].mode_regs);
	if (ret < 0) {
		goto err;
	}

	/* set effect */
	ret = ov9281_write_array(client, ov9281_effect[effe_index].mode_regs);
	if (ret < 0) {
		goto err;
	}

	/* set size win */
	ret = ov9281_write_array(client, priv->win->regs);
	if (ret < 0) {
		goto err;
	}

	priv->cfmt_code = code;
	*width = priv->win->width;
	*height = priv->win->height;

	return 0;

err:
	dev_err(&client->dev, "%s: Error %d", __func__, ret);
	ov9281_reset(client);
	priv->win = NULL;

	return ret;
}

static int ov9281_set_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_pad_config *cfg,
                          struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov9281_priv *priv = to_ov9281(client);

	if (format->pad) {
		return -EINVAL;
	}

	/*
	 * select suitable win, but don't store it
	 */
	priv->win = ov9281_select_win(&mf->code, &mf->width, &mf->height);
	if (!priv->win) {
		return -EINVAL;
	}

	mf->field   = V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_JPEG;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		ov9281_set_params(client, &mf->width,
		                  &mf->height, mf->code);
	if (cfg) {
		cfg->try_fmt = *mf;
	}
	return 0;
}

static int ov9281_enum_mbus_code(struct v4l2_subdev *sd,
                                 struct v4l2_subdev_pad_config *cfg,
                                 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(ov9281_supported_win_sizes)) {
		return -EINVAL;
	}

	code->code = ov9281_supported_win_sizes[code->index].mbus_code;
	return 0;
}

static int ov9281_enum_frame_size(struct v4l2_subdev *sd,
                                  struct v4l2_subdev_pad_config *cfg,
                                  struct v4l2_subdev_frame_size_enum *fse)
{
	int i, j;
	int num_valid = -1;
	__u32 index = fse->index;

	if (index >= N_WIN_SIZES) {
		return -EINVAL;
	}

#if 0
	j = ARRAY_SIZE(ov9281_codes);
	while (--j)
		if (fse->code == ov9281_codes[j]) {
			break;
		}
	for (i = 0; i < N_WIN_SIZES; i++) {
		if (index == ++num_valid) {
			fse->code = ov9281_codes[j];
			fse->min_width = ov9281_supported_win_sizes[index].width;
			fse->max_width = fse->min_width;
			fse->min_height = ov9281_supported_win_sizes[index].height;
			fse->max_height = fse->min_height;
			return 0;
		}
	}

	return -EINVAL;
#endif
	fse->code = ov9281_supported_win_sizes[index].mbus_code;
	fse->min_width = ov9281_supported_win_sizes[index].width;
	fse->max_width = fse->min_width;
	fse->min_height = ov9281_supported_win_sizes[index].height;
	fse->max_height = fse->min_height;
	return 0;
}
#if 0
static int ov9281_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	a->c.left   = 0;
	a->c.top    = 0;
	a->c.width  = OV9281_DEFAULT_WIDTH;
	a->c.height = OV9281_DEFAULT_HEIGHT;
	a->type     = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

static int ov9281_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left          = 0;
	a->bounds.top           = 0;
	a->bounds.width         = OV9281_DEFAULT_WIDTH;
	a->bounds.height        = OV9281_DEFAULT_HEIGHT;
	a->defrect          = a->bounds;
	a->type             = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator    = 1;
	a->pixelaspect.denominator  = 1;

	return 0;
}
#endif
static int ov9281_video_probe(struct i2c_client *client)
{
	unsigned char retval_high = 0, retval_low = 0;
	struct ov9281_priv *priv = to_ov9281(client);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	int ret;

	ret = ov9281_s_power(&priv->subdev, 1);
	if (ret < 0) {
		return ret;
	}
	/*
	 * check and show product ID and manufacturer ID
	 */

	retval_high = ov9281_read_reg(client, REG_CHIP_ID_HIGH);
	if (retval_high != CHIP_ID_HIGH) {
		dev_err(&client->dev, "read sensor %s chip_id high %x is error\n",
		        client->name, retval_high);
		ret = -EINVAL;
		goto done;
	}

	retval_low = ov9281_read_reg(client, REG_CHIP_ID_LOW);
	if (retval_low != CHIP_ID_LOW) {
		dev_err(&client->dev, "read sensor %s chip_id low %x is error\n",
		        client->name, retval_low);
		ret = -EINVAL;
		goto done;
	}

	dev_info(&client->dev, "read sensor %s id high:0x%x,low:%x successed!\n",
	         client->name, retval_high, retval_low);

	ret = v4l2_ctrl_handler_setup(&priv->hdl);

done:
	//  ov9281_s_power(&priv->subdev, 0);
	return ret;
}

static int ov9281_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
                                struct v4l2_mbus_config *cfg)
{
	/*
	    struct i2c_client *client = v4l2_get_subdevdata(sd);
	    struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);

	    cfg->flags = V4L2_MBUS_PCLK_SAMPLE_RISING | V4L2_MBUS_MASTER |
	        V4L2_MBUS_VSYNC_ACTIVE_LOW | V4L2_MBUS_HSYNC_ACTIVE_HIGH |
	        V4L2_MBUS_DATA_ACTIVE_HIGH;
	    cfg->type = V4L2_MBUS_PARALLEL;

	    cfg->flags = soc_camera_apply_board_flags(ssdd, cfg);
	*/
	/*int dts*/
	return 0;
}

static struct v4l2_subdev_core_ops ov9281_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov9281_g_register,
	.s_register = ov9281_s_register,
#endif
	.s_power    = ov9281_s_power,
	//.querymenu    = ov9281_querymenu,
};

static struct v4l2_subdev_video_ops ov9281_subdev_video_ops = {
	.s_stream   = ov9281_s_stream,
	//  .cropcap    = ov9281_cropcap,
	//  .g_crop     = ov9281_g_crop,
	//  .g_mbus_config  = ov9281_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops ov9281_subdev_pad_ops = {
	.enum_mbus_code = ov9281_enum_mbus_code,
	.enum_frame_size = ov9281_enum_frame_size,
	.get_fmt    = ov9281_get_fmt,
	.set_fmt    = ov9281_set_fmt,
	.get_mbus_config = ov9281_g_mbus_config,
	.get_selection  = ov9281_get_selection,
};

static struct v4l2_subdev_ops ov9281_subdev_ops = {
	.core   = &ov9281_subdev_core_ops,
	.video  = &ov9281_subdev_video_ops,
	.pad    = &ov9281_subdev_pad_ops,
};

static int ov9281_hw_reset(struct i2c_client *client)
{
	struct ov9281_priv *priv = to_ov9281(client);

	if (priv->resetb_gpio) {
		/* Active the resetb pin to perform a reset pulse */
		gpiod_direction_output(priv->resetb_gpio, 1);
		usleep_range(3000, 5000);
		gpiod_direction_output(priv->resetb_gpio, 0);
	}

	return 0;
}

static int ov9281_probe_dt(struct i2c_client *client,
                           struct ov9281_priv *priv)
{
	struct v4l2_subdev_platform_data *sd_pdata = priv->sd_pdata;
	struct device_node *np = client->dev.of_node;
	int supplies = 0, index = 0;

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

		devm_regulator_bulk_get(&client->dev, sd_pdata->num_regulators, sd_pdata->regulators);
		//  soc_camera_power_init(&client->dev, ssdd_dt);
	} else {
		/* Request the power down GPIO asserted */
		priv->pwdn_gpio = devm_gpiod_get_optional(&client->dev, "pwdn",
		                  GPIOD_OUT_HIGH);
		if (!priv->pwdn_gpio) {
			dev_dbg(&client->dev, "pwdn gpio is not assigned!\n");
		} else if (IS_ERR(priv->pwdn_gpio)) {
			return PTR_ERR(priv->pwdn_gpio);
		}

		/* Request the power down GPIO asserted */
		priv->vcc_en_gpio = devm_gpiod_get_optional(&client->dev, "vcc-en",
		                    GPIOD_OUT_HIGH);
		if (!priv->vcc_en_gpio) {
			dev_dbg(&client->dev, "vcc_en gpio is not assigned!\n");
		} else if (IS_ERR(priv->vcc_en_gpio)) {
			return PTR_ERR(priv->vcc_en_gpio);
		}

		/* Initialize the soc_camera_subdev_desc */
		priv->power = ov9281_hw_power;
		priv->reset = ov9281_hw_reset;
	}
	return 0;
}

#include <linux/regulator/consumer.h>
/*
 * i2c_driver functions
 */
static int ov9281_probe(struct i2c_client *client,
                        const struct i2c_device_id *did)
{
	struct ov9281_priv  *priv;
	struct i2c_adapter  *adapter = to_i2c_adapter(client->dev.parent);
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&adapter->dev,
		        "ov9281: I2C-Adapter doesn't support SMBUS\n");
		return -EIO;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct ov9281_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&adapter->dev,
		        "Failed to allocate memory for private data!\n");
		return -ENOMEM;
	}

	priv->mclk = v4l2_clk_get(&client->dev, "gate_cim_mclk");
	if (IS_ERR(priv->clk)) {
		dev_err(&adapter->dev, "mclk_get err!\n");
		return -EPROBE_DEFER;
	}

	ret = v4l2_clk_enable(priv->mclk);
	if (ret) {
		dev_err(&adapter->dev, "mclk_enable err!\n");
		return -EPROBE_DEFER;
	}

	priv->clk = v4l2_clk_get(&client->dev, "div_cim");
	if (IS_ERR(priv->clk)) {
		return -EPROBE_DEFER;
	}

	v4l2_clk_set_rate(priv->clk, 24000000);
	/**
	    if (!ssdd && !client->dev.of_node) {
	        dev_err(&client->dev, "Missing platform_data for driver\n");
	        ret = -EINVAL;
	        goto err_videoprobe;
	    }

	    if (!ssdd) {
	        ret = ov9281_probe_dt(client, priv);
	        if (ret)
	            goto err_clk;
	    }
	*/
	ret = ov9281_probe_dt(client, priv);
	if (ret) {
		goto err_probe_dt;
	}

	v4l2_i2c_subdev_init(&priv->subdev, client, &ov9281_subdev_ops);

	/* add handler */
	v4l2_ctrl_handler_init(&priv->hdl, 2);
	v4l2_ctrl_new_std(&priv->hdl, &ov9281_ctrl_ops,
	                  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov9281_ctrl_ops,
	                  V4L2_CID_HFLIP, 0, 1, 1, 0);
	priv->subdev.ctrl_handler = &priv->hdl;
	if (priv->hdl.error) {
		ret = priv->hdl.error;
		goto err_hdl;
	}

	ret = ov9281_video_probe(client);
	if (ret < 0) {
		goto err_detect;
	}

	ret = v4l2_async_register_subdev(&priv->subdev);
	if (ret < 0) {
		goto err_async_register_subdev;
	}

	dev_info(&adapter->dev, "ov9281 DVP snapshot Probed\n");

	return 0;

err_async_register_subdev:
err_detect:
	v4l2_ctrl_handler_free(&priv->hdl);
err_hdl:
	v4l2_device_unregister_subdev(&priv->subdev);
err_probe_dt:
	v4l2_clk_put(priv->clk);
	v4l2_clk_put(priv->mclk);
	devm_kfree(&client->dev, priv);
	return ret;
}

static int ov9281_remove(struct i2c_client *client)
{
	struct ov9281_priv       *priv = to_ov9281(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_clk_put(priv->clk);
	v4l2_clk_put(priv->mclk);
	v4l2_device_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);
	devm_kfree(&client->dev, priv);
	return 0;
}

static const struct i2c_device_id ov9281_id[] = {
	{ "ov9281",  0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov9281_id);
static const struct of_device_id ov9281_of_match[] = {
	{.compatible = "ovti,ov9281", },
	{},
};
MODULE_DEVICE_TABLE(of, ov9281_of_match);
static struct i2c_driver ov9281_i2c_driver = {
	.driver = {
		.name = "ov9281",
		.of_match_table = of_match_ptr(ov9281_of_match),
	},
	.probe    = ov9281_probe,
	.remove   = ov9281_remove,
	.id_table = ov9281_id,
};
module_i2c_driver(ov9281_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for Omni Vision ov9281 sensor");
MODULE_AUTHOR("Alberto Panizzo");
MODULE_LICENSE("GPL v2");
