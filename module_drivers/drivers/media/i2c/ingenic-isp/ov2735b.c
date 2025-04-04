/*
 * A V4L2 driver for OmniVision OV2735B cameras.
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.  Written
 * by Jonathan Corbet with substantial inspiration from Mark
 * McClelland's ovcamchip code.
 *
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-image-sizes.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>

#include <isp-sensor.h>

#define OV2735B_CHIP_ID_H   (0x27)
#define OV2735B_CHIP_ID_L   (0x35)
#define OV2735B_REG_END     0xff
#define OV2735B_REG_DELAY   0x00
#define OV2735B_PAGE_REG        0xfd

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct ov2735b_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	enum v4l2_xfer_func xfer_func;
	enum v4l2_field field;
	enum v4l2_ycbcr_encoding ycbcr_enc;
	enum v4l2_quantization quantization;
	void *regs;

};

struct ov2735b_gpio {
	int pin;
	int active_level;
};

struct ov2735b_supply {
	struct regulator *avdd;
	struct regulator *dvdd;
	struct regulator *dovdd;
};

struct ov2735b_info {
	struct ov2735b_supply supply;
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;

	struct clk *clk;
	struct clk *mclk;

	struct v4l2_ctrl *exposure;

	struct media_pad pad;

	struct v4l2_subdev_format *format;  /*current fmt.*/
	struct ov2735b_win_size *win;

	struct ov2735b_gpio reset;
	struct ov2735b_gpio ircutp;
	struct ov2735b_gpio ircutn;
};

/*
 * the part of driver maybe modify about different sensor and different board.
 */
struct again_lut {
	unsigned int value; /*sensor regs value*/
	unsigned int gain;  /*isp gain*/
};

struct again_lut ov2735b_again_lut[] = {
	{0x10,  0},
	{0x11,  5731},
	{0x12,  11136},
	{0x13,  16248},
	{0x14,  21097},
	{0x15,  25710},
	{0x16,  30109},
	{0x17,  34312},
	{0x18,  38336},
	{0x19,  42195},
	{0x1a,  45904},
	{0x1b,  49472},
	{0x1c,  52910},
	{0x1d,  56228},
	{0x1e,  59433},
	{0x1f,  62534},
	{0x20,  65536},
	{0x22,  71267},
	{0x24,  76672},
	{0x26,  81784},
	{0x28,  86633},
	{0x2a,  91246},
	{0x2c,  95645},
	{0x2e,  99848},
	{0x30,  103872},
	{0x32,  107731},
	{0x34,  111440},
	{0x36,  115008},
	{0x38,  118446},
	{0x3a,  121764},
	{0x3c,  124969},
	{0x3e,  128070},
	{0x40,  131072},
	{0x44,  136803},
	{0x48,  142208},
	{0x4c,  147320},
	{0x50,  152169},
	{0x54,  156782},
	{0x58,  161181},
	{0x5c,  165384},
	{0x60,  169408},
	{0x64,  173267},
	{0x68,  176976},
	{0x6c,  180544},
	{0x70,  183982},
	{0x74,  187300},
	{0x78,  190505},
	{0x7c,  193606},
	{0x80,  196608},
	{0x88,  202339},
	{0x90,  207744},
	{0x98,  212856},
	{0xa0,  217705},
	{0xa8,  222318},
	{0xb0,  226717},
	{0xb8,  230920},
	{0xc0,  234944},
	{0xc8,  238803},
	{0xd0,  242512},
	{0xd8,  246080},
	{0xe0,  249518},
	{0xe8,  252836},
	{0xf0,  256041},
	{0xf8,  259142},
};

static inline struct ov2735b_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov2735b_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov2735b_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};

static struct regval_list ov2735b_init_regs_1920_1080_30fps_dvp[] = {
	{0xfd, 0x00},
	{OV2735B_REG_DELAY, 0x05},
	{0xfd, 0x00},
	{0x2f, 0x10},
	{0x34, 0x00},
	{0x30, 0x15},
	{0x33, 0x01},
	{0x35, 0x20},
	{0x1d, 0xa5},
	{0xfd, 0x01},
	{0x0d, 0x00},
	{0x30, 0x00},
	{0x03, 0x01},
	{0x04, 0x8f},
	{0x01, 0x01},
	{0x09, 0x00},
	{0x0a, 0x20},
	{0x06, 0x0a},
	{0x24, 0x10},
	{0x01, 0x01},
	{0xfb, 0x73},
	{0x01, 0x01},
	{0xfd, 0x01},
	{0x1a, 0x6b},
	{0x1c, 0xea},
	{0x16, 0x0c},
	{0x21, 0x00},
	{0x11, 0x63},
	{0x19, 0xc3},
	{0x26, 0xda},
	{0x29, 0x01},
	{0x33, 0x6f},
	{0x2a, 0xd2},
	{0x2c, 0x40},
	{0xd0, 0x02},
	{0xd1, 0x01},
	{0xd2, 0x20},
	{0xd3, 0x04},
	{0xd4, 0x2a},
	{0x50, 0x00},
	{0x51, 0x2c},
	{0x52, 0x29},
	{0x53, 0x00},
	{0x55, 0x44},
	{0x58, 0x29},
	{0x5a, 0x00},
	{0x5b, 0x00},
	{0x5d, 0x00},
	{0x64, 0x2f},
	{0x66, 0x62},
	{0x68, 0x5b},
	{0x75, 0x46},
	{0x76, 0xf0},
	{0x77, 0x4f},
	{0x78, 0xef},
	{0x72, 0xcf},
	{0x73, 0x36},
	{0x7d, 0x0d},
	{0x7e, 0x0d},
	{0x8a, 0x77},
	{0x8b, 0x77},
	{0xfd, 0x01},
	{0xb1, 0x82},
	{0xb3, 0x0b},
	{0xb4, 0x14},
	{0x9d, 0x40},
	{0xa1, 0x05},
	{0x94, 0x44},
	{0x95, 0x33},
	{0x96, 0x1f},
	{0x98, 0x45},
	{0x9c, 0x10},
	{0xb5, 0x70},
	{0xa0, 0x00},
	{0x25, 0xe0},
	{0x20, 0x7b},
	{0x8f, 0x88},
	{0x91, 0x40},
	{0xfd, 0x01},
	{0xfd, 0x02},
	{0x5e, 0x03},
	{0xfd, 0x02},
	{0x60, 0xf0},
	{0xa1, 0x04},
	{0xa3, 0x40},
	{0xa5, 0x02},
	{0xa7, 0xc4},
	{0xfd, 0x01},
	{0x86, 0x77},
	{0x89, 0x77},
	{0x87, 0x74},
	{0x88, 0x74},
	{0xfc, 0xe0},
	{0xfe, 0xe0},
	{0xf0, 0x40},
	{0xf1, 0x40},
	{0xf2, 0x40},
	{0xf3, 0x40},
	{0xfd, 0x02},
	{0x36, 0x08},
	{0xa0, 0x00},
	{0xa1, 0x08},
	{0xa2, 0x04},
	{0xa3, 0x38},
	{0xa4, 0x00},
	{0xa5, 0x04},
	{0xa6, 0x03},
	{0xa7, 0xc0},
	{0xfd, 0x03},
	{0xc0, 0x01},//OTP transf
	{0xfd, 0x04},
	{0x22, 0x14},
	{0x23, 0x14},
	{0xfd, 0x01},
	{0x06, 0xe0},
	{0x01, 0x01},
	{0xfd, 0x00},
	{0x1b, 0x00},
	{0xfd, 0x01},
	{0x0d, 0x10},
	{0x0e, 0x05},
	{0x0f, 0x2b},
	{0x01, 0x01},
	{0xfd, 0x00},
	{0x36, 0x01},
	{0x37, 0x01},//fake stream off

	{OV2735B_REG_END, 0x00},    /* END MARKER */
};

#if 0
static struct regval_list ov2735b_init_regs_1920_1080_15fps_dvp[] = {
	{0xfd, 0x00},
	{OV2735B_REG_DELAY, 0x05},
	{0xfd, 0x00},
	{0x2f, 0x08},
	{0x34, 0x00},
	{0x30, 0x15},
	{0x33, 0x01},
	{0x35, 0x20},
	{0x1d, 0xa5},
	{0xfd, 0x01},
	{0x0d, 0x00},
	{0x30, 0x00},
	{0x03, 0x01},
	{0x04, 0x8f},
	{0x01, 0x01},
	{0x09, 0x00},
	{0x0a, 0x20},
	{0x06, 0x0a},
	{0x24, 0x10},
	{0x01, 0x01},
	{0xfb, 0x73},
	{0x01, 0x01},
	{0xfd, 0x01},
	{0x1a, 0x6b},
	{0x1c, 0xea},
	{0x16, 0x0c},
	{0x21, 0x00},
	{0x11, 0x63},
	{0x19, 0xc3},
	{0x26, 0xda},
	{0x29, 0x01},
	{0x33, 0x6f},
	{0x2a, 0xd2},
	{0x2c, 0x40},
	{0xd0, 0x02},
	{0xd1, 0x01},
	{0xd2, 0x20},
	{0xd3, 0x04},
	{0xd4, 0x2a},
	{0x50, 0x00},
	{0x51, 0x2c},
	{0x52, 0x29},
	{0x53, 0x00},
	{0x55, 0x44},
	{0x58, 0x29},
	{0x5a, 0x00},
	{0x5b, 0x00},
	{0x5d, 0x00},
	{0x64, 0x2f},
	{0x66, 0x62},
	{0x68, 0x5b},
	{0x75, 0x46},
	{0x76, 0xf0},
	{0x77, 0x4f},
	{0x78, 0xef},
	{0x72, 0xcf},
	{0x73, 0x36},
	{0x7d, 0x0d},
	{0x7e, 0x0d},
	{0x8a, 0x77},
	{0x8b, 0x77},
	{0xfd, 0x01},
	{0xb1, 0x82},
	{0xb3, 0x0b},
	{0xb4, 0x14},
	{0x9d, 0x40},
	{0xa1, 0x05},
	{0x94, 0x44},
	{0x95, 0x33},
	{0x96, 0x1f},
	{0x98, 0x45},
	{0x9c, 0x10},
	{0xb5, 0x70},
	{0xa0, 0x00},
	{0x25, 0xe0},
	{0x20, 0x7b},
	{0x8f, 0x88},
	{0x91, 0x40},
	{0xfd, 0x01},
	{0xfd, 0x02},
	{0x5e, 0x03},
	{0xfd, 0x02},
	{0x60, 0xf0},
	{0xa1, 0x04},
	{0xa3, 0x40},
	{0xa5, 0x02},
	{0xa7, 0xc4},
	{0xfd, 0x01},
	{0x86, 0x77},
	{0x89, 0x77},
	{0x87, 0x74},
	{0x88, 0x74},
	{0xfc, 0xe0},
	{0xfe, 0xe0},
	{0xf0, 0x40},
	{0xf1, 0x40},
	{0xf2, 0x40},
	{0xf3, 0x40},
	{0xfd, 0x02},
	{0x36, 0x08},
	{0xa0, 0x00},
	{0xa1, 0x08},
	{0xa2, 0x04},
	{0xa3, 0x38},
	{0xa4, 0x00},
	{0xa5, 0x04},
	{0xa6, 0x03},
	{0xa7, 0xc0},
	{0xfd, 0x03},
	{0xc0, 0x01},//OTP transf
	{0xfd, 0x04},
	{0x22, 0x14},
	{0x23, 0x14},
	{0xfd, 0x01},
	{0x06, 0xe0},
	{0x01, 0x01},
	{0xfd, 0x00},
	{0x1b, 0x00},
	{0xfd, 0x01},
	{0x0d, 0x10},
	{0x0e, 0x07},
	{0x0f, 0x6b},
	{0x01, 0x01},
	{0xfd, 0x00},
	{0x36, 0x01},
	{0x37, 0x01},//fake stream off

	{OV2735B_REG_END, 0x00},    /* END MARKER */
};
#endif

static struct regval_list ov2735b_stream_on[] = {
	{0xfd, 0x00},
	{0x36, 0x00},
	{0x37, 0x00},//fake stream on

	{OV2735B_REG_END, 0x00},
};

static struct regval_list ov2735b_stream_off[] = {
	{0xfd, 0x00},
	{0x36, 0x01},
	{0x37, 0x01},//fake stream off

	{OV2735B_REG_END, 0x00},
};

int ov2735b_read(struct v4l2_subdev *sd, unsigned char reg,
                 unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
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

static int ov2735b_write(struct v4l2_subdev *sd, unsigned char reg,
                         unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char buf[2] = {reg, value};
	struct i2c_msg msg = {
		.addr   = client->addr,
		.flags  = 0,
		.len    = 2,
		.buf    = buf,
	};
	int ret;
	unsigned int timeout  = 100;
	while (timeout--) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == -EAGAIN) {
			msleep(100);
			continue;
		} else {
			break;
		}
	}

	if (ret > 0) {
		ret = 0;
	}
	return ret;
}

#if 0
static int ov2735b_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != OV2735B_REG_END) {
		if (vals->reg_num == OV2735B_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = ov2735b_read(sd, vals->reg_num, &val);
			if (ret < 0) {
				return ret;
			}
			if (vals->reg_num == OV2735B_PAGE_REG) {
				val &= 0xf8;
				val |= (vals->value & 0x07);
				ret = ov2735b_write(sd, vals->reg_num, val);
				ret = ov2735b_read(sd, vals->reg_num, &val);
			}
		}
		vals++;
	}
	return 0;
}
#endif

static int ov2735b_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != OV2735B_REG_END) {
		if (vals->reg_num == OV2735B_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = ov2735b_write(sd, vals->reg_num, vals->value);
			if (ret < 0) {
				return ret;
			}
		}
		vals++;
	}
	return 0;
}

/*
 * Stuff that knows about the sensor.
 */
static int ov2735b_reset(struct v4l2_subdev *sd, u32 val)
{
	struct ov2735b_info *info = to_state(sd);

	if (val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(10);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(10);
	}
	return 0;
}

static int ov2735b_ircut(struct v4l2_subdev *sd, u32 val)
{
	struct ov2735b_info *info = to_state(sd);

	if (val) {
		gpio_direction_output(info->ircutp.pin, info->ircutp.active_level);
		gpio_direction_output(info->ircutn.pin, info->ircutn.active_level);
		msleep(10);
		gpio_direction_output(info->ircutp.pin, !info->ircutp.active_level);
	} else {
		gpio_direction_output(info->ircutp.pin, !info->ircutp.active_level);
		gpio_direction_output(info->ircutn.pin, !info->ircutn.active_level);
		msleep(10);
		gpio_direction_output(info->ircutp.pin, info->ircutp.active_level);
	}
	return 0;
}

static int ov2735b_init(struct v4l2_subdev *sd, u32 val)
{
	struct ov2735b_info *info = to_state(sd);
	int ret = 0;

	ret = ov2735b_write_array(sd, info->win->regs);

	return ret;
}

static int ov2735b_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;

	ret = ov2735b_read(sd, 0x02, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret, v);
	if (ret < 0) {
		return ret;
	}
	if (v != OV2735B_CHIP_ID_H) {
		return -ENODEV;
	}
	*ident = v;

	ret = ov2735b_read(sd, 0x03, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret, v);
	if (ret < 0) {
		return ret;
	}
	if (v != OV2735B_CHIP_ID_L) {
		return -ENODEV;
	}
	*ident = (*ident << 8) | v;

	ret = ov2735b_read(sd, 0x04, &v);
	pr_debug("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret, v);
	if (ret < 0) {
		return ret;
	}
	if (v != 0x05 && v != 0x06 && v != 0x07) {
		return -ENODEV;
	}

	return 0;
}

static struct ov2735b_win_size ov2735b_win_sizes[] = {
	/* 1920*1080 */
	{

		.sensor_info.fps            = 30 << 16 | 1,

		.width      = 1920,
		.height     = 1080,
		.mbus_code  = MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.xfer_func      = V4L2_XFER_FUNC_SRGB,
		.field          = V4L2_FIELD_NONE,
		.ycbcr_enc      = V4L2_YCBCR_ENC_DEFAULT,
		.quantization   = V4L2_QUANTIZATION_FULL_RANGE,
		.regs       = ov2735b_init_regs_1920_1080_30fps_dvp,
	}
};

#if 0
static int ov2735b_enum_mbus_code(struct v4l2_subdev *sd,
                                  struct v4l2_subdev_state *state,
                                  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= N_OV2735B_FMTS) {
		return -EINVAL;
	}

	code->code = ov2735b_formats[code->index].mbus_code;
	return 0;
}
#endif

/*
 * Set a format.
 */
static int ov2735b_set_fmt(struct v4l2_subdev *sd,
                           struct v4l2_subdev_state *state,
                           struct v4l2_subdev_format *format)
{
	if (format->pad) {
		return -EINVAL;
	}

	return 0;
}

static int ov2735b_get_fmt(struct v4l2_subdev *sd,
                           struct v4l2_subdev_state *state,
                           struct v4l2_subdev_format *format)
{
	struct ov2735b_info *info = to_state(sd);
	struct ov2735b_win_size *wsize = info->win;
	struct v4l2_mbus_framefmt *fmt = &format->format;
	int ret = 0;

	if (!info->win) {
		dev_err(sd->dev, "sensor win_size not set!\n");
		return -EINVAL;
	}

	fmt->width = wsize->width;
	fmt->height = wsize->height;
	fmt->code = wsize->mbus_code;
	fmt->colorspace = wsize->colorspace;
	fmt->xfer_func = wsize->xfer_func;
	fmt->field = wsize->field;
	fmt->ycbcr_enc = wsize->ycbcr_enc;
	fmt->quantization = wsize->quantization;

	ISP_SET_SENSOR_INFO(fmt, info);

	//  printk("----%s, %d, width: %d, height: %d, code: %x\n",
	//          __func__, __LINE__, fmt->width, fmt->height, fmt->code);

	return ret;
}

static int ov2735b_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov2735b_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov2735b_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov2735b_s_vflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

/*
 * GAIN is split between REG_GAIN and REG_VREF[7:6].  If one believes
 * the data sheet, the VREF parts should be the most significant, but
 * experience shows otherwise.  There seems to be little value in
 * messing with the VREF bits, so we leave them alone.
 */
static int ov2735b_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int ov2735b_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	printk("---%s, %d, s_gain: value: %d\n", __func__, __LINE__, value);

	return ret;
}

static unsigned int again_to_regval(int gain)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(ov2735b_again_lut); i++) {
		lut = &ov2735b_again_lut[i];

		if (gain <= lut->gain) {
			return lut->value;
		}
	}

	/*last value.*/
	return lut->value;
}

static int regval_to_again(unsigned int regval)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(ov2735b_again_lut); i++) {
		lut = &ov2735b_again_lut[i];

		if (regval == lut->value) {
			return lut->gain;
		}
	}
	printk("%s, %d regval not mapped to isp gain\n", __func__, __LINE__);
	return -EINVAL;
}

static int ov2735b_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	char v = 0;
	unsigned int reg_val = 0;
	int ret = 0;

	ret = ov2735b_read(sd, 0x23, &v);
	reg_val |= v ;

	*value = regval_to_again(reg_val);

	return ret;

}
/*set analog gain db value, map value to sensor register.*/
static int ov2735b_s_again(struct v4l2_subdev *sd, int value)
{
	struct ov2735b_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;

	if (value < info->again->minimum || value > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(value);
	}

	ret = ov2735b_write(sd, 0xfd, 0x01);
	ret += ov2735b_write(sd, 0x24, (unsigned char)reg_value);
	ret += ov2735b_write(sd, 0x01, 0x01);
	if (ret < 0) {
		printk("ov2735b_write error  %d\n", __LINE__);
		return ret;
	}
	return 0;
}

/*
 * Tweak autogain.
 */
static int ov2735b_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int ov2735b_s_exp(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	ret = ov2735b_write(sd, 0xfd, 0x01);
	ret += ov2735b_write(sd, 0x4, (unsigned char)(value & 0xff));
	ret += ov2735b_write(sd, 0x3, (unsigned char)((value & 0xff00) >> 8));
	ret += ov2735b_write(sd, 0x01, 0x01);

	if (ret < 0) {
		printk("ov2735b_write error  %d\n", __LINE__);
		return ret;
	}
	return ret;
}

static int ov2735b_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct ov2735b_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return ov2735b_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return ov2735b_g_again(sd, &info->again->val);
	}
	return -EINVAL;
}

static int ov2735b_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct ov2735b_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return ov2735b_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return ov2735b_s_contrast(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return ov2735b_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return ov2735b_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* ov2735b_s_gain turns off auto gain */
			return ov2735b_s_gain(sd, info->gain->val);
		}
		return ov2735b_s_autogain(sd, ctrl->val);
	case V4L2_CID_GAIN:
		return ov2735b_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return ov2735b_s_again(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return ov2735b_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops ov2735b_ctrl_ops = {
	.s_ctrl = ov2735b_s_ctrl,
	.g_volatile_ctrl = ov2735b_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov2735b_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = ov2735b_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int ov2735b_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	ov2735b_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

static int ov2735b_power(struct v4l2_subdev *sd, int on)
{
	struct ov2735b_info *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	info->supply.avdd  = devm_regulator_get_optional(&client->dev, "avdd");
	info->supply.dvdd  = devm_regulator_get_optional(&client->dev, "dvdd");
	info->supply.dovdd = devm_regulator_get_optional(&client->dev, "dovdd");

	if (IS_ERR(info->supply.avdd) || IS_ERR(info->supply.dvdd)
	    || IS_ERR(info->supply.dovdd)) {
		if ((PTR_ERR(info->supply.avdd) == -EPROBE_DEFER)
		    || (PTR_ERR(info->supply.avdd) == -EPROBE_DEFER)
		    || (PTR_ERR(info->supply.avdd) == -EPROBE_DEFER)) {
			return -EPROBE_DEFER;
		}
		printk("No ov2735b vdd regulator found\n");
	}

	if ((!IS_ERR(info->supply.avdd)) && (!IS_ERR(info->supply.avdd))
	    && (!IS_ERR(info->supply.avdd)))  {
		if (on) {
			ret = regulator_enable(info->supply.avdd);
			ret = regulator_enable(info->supply.dvdd);
			ret = regulator_enable(info->supply.dovdd);
			if (ret) {
				dev_err(&client->dev, "ov2735b vdd supply disable failed\n");
			}
		} else {
			ret = regulator_disable(info->supply.avdd);
			ret = regulator_disable(info->supply.dvdd);
			ret = regulator_disable(info->supply.dovdd);
			if (ret) {
				dev_err(&client->dev, "ov2735b vdd supply disable failed\n");
			}
		}
	} else {
		dev_err(&client->dev, "ov2735b vdd supply IS_ERR failed\n");
	}
	return ret;
}

int ov2735b_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;

	if (enable) {
		ret = ov2735b_write_array(sd, ov2735b_stream_on);
		pr_debug("ov2735b stream on\n");

	} else {
		ret = ov2735b_write_array(sd, ov2735b_stream_off);
		pr_debug("ov2735b stream off\n");
	}
	return ret;
}

int ov2735b_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval)
{
	struct ov2735b_info *info = to_state(sd);
	if (info->win->sensor_info.fps) {
		interval->interval.numerator = info->win->sensor_info.fps & 0xffff;
		interval->interval.denominator = info->win->sensor_info.fps >> 16;
		return 0;
	}
	return -EINVAL;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops ov2735b_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov2735b_g_register,
	.s_register = ov2735b_s_register,
#endif

};

static const struct v4l2_subdev_video_ops ov2735b_video_ops = {
	.s_stream = ov2735b_s_stream,
	.g_frame_interval = ov2735b_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ov2735b_pad_ops = {
	//.enum_frame_interval = ov2735b_enum_frame_interval,
	//.num_frame_size = ov2735b_enum_frame_size,
	//.enum_mbus_code = ov2735b_enum_mbus_code,
	.set_fmt = ov2735b_set_fmt,
	.get_fmt = ov2735b_get_fmt,
};

static const struct v4l2_subdev_ops ov2735b_ops = {
	.core = &ov2735b_core_ops,
	.video = &ov2735b_video_ops,
	.pad = &ov2735b_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int ov2735b_probe(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	struct ov2735b_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		return -ENOMEM;
	}
	sd = &info->sd;

	gpio = of_get_named_gpio(client->dev.of_node, "ingenic,rst-gpio", 0);
	if (gpio_is_valid(gpio)) {
		info->reset.pin = gpio;
		info->reset.active_level = INGENIC_ISP_GPIO_ACTIVE_LOW;
	}
	gpio = of_get_named_gpio(client->dev.of_node, "ingenic,ircutp-gpio", 0);
	if (gpio_is_valid(gpio)) {
		info->ircutp.pin = gpio;
		info->ircutp.active_level = INGENIC_ISP_GPIO_ACTIVE_HIGH;
	}
	gpio = of_get_named_gpio(client->dev.of_node, "ingenic,ircutn-gpio", 0);
	if (gpio_is_valid(gpio)) {
		info->ircutn.pin = gpio;
		info->ircutn.active_level = INGENIC_ISP_GPIO_ACTIVE_LOW;
	}

	v4l2_i2c_subdev_init(sd, client, &ov2735b_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ov2735b_power(sd, 1);

	/*clk*/
	info->mclk = devm_clk_get(&client->dev, "gate_cim_mclk");
	if (IS_ERR(info->mclk)) {
		ret = PTR_ERR(info->mclk);
		goto err_mclkget;
	}
	ret = clk_prepare_enable(info->mclk);
	if (ret) {
		dev_err(sd->dev, "mclk_enable err!\n");
	}

	info->clk = devm_clk_get(&client->dev, "div_cim");
	if (IS_ERR(info->clk)) {
		ret = PTR_ERR(info->clk);
		goto err_clkget;
	}

	ret = clk_set_rate(info->clk, 24000000);
	if (ret) {
		dev_err(sd->dev, "clk_set_rate err!\n");
	}

	ret = clk_prepare_enable(info->clk);
	if (ret) {
		dev_err(sd->dev, "clk_enable err!\n");
	}

	ov2735b_reset(sd, 1);
#if 1
	/* Make sure it's an ov2735b */
	ret = ov2735b_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
		        "chip found @ 0x%x (%s) is not an ov2735b chip.\n",
		        client->addr << 1, client->adapter->name);
		goto exit_deinit_power;
	}
#endif
	/*IRCUT ctl 0:off 1:on*/
	ov2735b_ircut(sd, 0);

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
	         client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &ov2735b_ctrl_ops,
	                  V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &ov2735b_ctrl_ops,
	                  V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &ov2735b_ctrl_ops,
	                  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &ov2735b_ctrl_ops,
	                  V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &ov2735b_ctrl_ops,
	                               V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &ov2735b_ctrl_ops,
	                                V4L2_CID_ANALOGUE_GAIN, 0, 259142, 1, 10000);

	/*unit exposure lines: */
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &ov2735b_ctrl_ops,
	                                   V4L2_CID_EXPOSURE, 4, 1899 - 4, 1, 1500);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->win = &ov2735b_win_sizes[0];
	ov2735b_init(sd, 1);

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if (ret < 0) {
		goto err_entity_init;
	}
	info->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0) {
		goto err_videoprobe;
	}

	dev_info(&client->dev, "ov2735b Probed\n");
	return 0;
err_videoprobe:
exit_deinit_power:
	ov2735b_power(sd, 0);
err_entity_init:
	devm_clk_put(&client->dev, info->clk);
err_clkget:
	devm_clk_put(&client->dev, info->mclk);
err_mclkget:
	devm_kfree(&client->dev, info);
	i2c_set_clientdata(client, NULL);
	return ret;
}

static void ov2735b_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2735b_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	devm_clk_put(&client->dev, info->clk);
	devm_clk_put(&client->dev, info->mclk);
}

static const struct i2c_device_id ov2735b_id[] = {
	{ "ov2735b", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov2735b_id);

static const struct of_device_id ov2735b_of_match[] = {
	{.compatible = "ovti,ov2735b", },
	{},
};
MODULE_DEVICE_TABLE(of, ov2735b_of_match);

static int ov2735b_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2735b_info *info = to_state(sd);

	ov2735b_power(sd, 0);
	clk_disable_unprepare(info->clk);
	return 0;
}

static int ov2735b_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2735b_info *info = to_state(sd);

	ov2735b_power(sd, 1);
	clk_prepare_enable(info->clk);
	ov2735b_reset(sd, 1);
	ov2735b_init(sd, 1);
	return 0;
}

const struct dev_pm_ops ov2735b_pm = {
	.suspend = ov2735b_suspend,
	.resume  = ov2735b_resume,
};

static struct i2c_driver ov2735b_driver = {
	.driver = {
		.name   = "ov2735b",
		.of_match_table = of_match_ptr(ov2735b_of_match),
		.pm =  &ov2735b_pm,
	},
	.probe      = ov2735b_probe,
	.remove     = ov2735b_remove,
	.id_table   = ov2735b_id,
};

module_i2c_driver(ov2735b_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for OmniVision ov2735b sensors");
MODULE_LICENSE("GPL");
