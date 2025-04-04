/*
 * A V4L2 driver for OmniVision gc2053 cameras.
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
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-image-sizes.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <isp-sensor.h>

#define GC2053_CHIP_ID_H    (0x20)
#define GC2053_CHIP_ID_L    (0x53)
#define GC2053_REG_END      0xff
#define GC2053_REG_DELAY    0x00

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct gc2053_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;
};

struct gc2053_gpio {
	int pin;
	int active_level;
};

struct gc2053_info {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;

	struct clk *clk;
	struct clk *mclk;

	struct v4l2_ctrl *exposure;

	struct media_pad pad;

	struct v4l2_subdev_format *format;  /*current fmt.*/
	struct gc2053_win_size *win;

	struct gc2053_gpio reset;
	struct gc2053_gpio pwdn;
	struct gc2053_gpio led;
};

/*
 * the part of driver maybe modify about different sensor and different board.
 */
struct again_lut {
	int index;
	unsigned int regb4;
	unsigned int regb3;
	unsigned int dpc;
	unsigned int blc;
	unsigned int gain;
};

static struct again_lut gc2053_again_lut[] = {
	{0, 0x00, 0x00, 0x01, 0x00, 0},           //  1.000000
	{1, 0x00, 0x10, 0x01, 0x0c, 13726},       //  1.156250
	{2, 0x00, 0x20, 0x01, 0x1b, 31176},       //   1.390625
	{3, 0x00, 0x30, 0x01, 0x2c, 44067},       //   1.593750
	{4, 0x00, 0x40, 0x01, 0x3f, 64793},       //   1.984375
	{5, 0x00, 0x50, 0x02, 0x16, 78620},       //   2.296875
	{6, 0x00, 0x60, 0x02, 0x35, 96179},       //   2.765625
	{7, 0x00, 0x70, 0x03, 0x16, 109137},      //    3.171875
	{8, 0x00, 0x80, 0x04, 0x02, 132535},      //   4.062500
	{9, 0x00, 0x90, 0x04, 0x31, 146065},      //    4.687500
	{10, 0x00, 0xa0, 0x05, 0x32, 163565},     //    5.640625
	{11, 0x00, 0xb0, 0x06, 0x35, 176745},     //    6.484375
	{12, 0x00, 0xc0, 0x08, 0x04, 195116},     //   7.875000
	{13, 0x00, 0x5a, 0x09, 0x19, 208558},     //    9.078125
	{14, 0x00, 0x83, 0x0b, 0x0f, 229100},     //   11.281250
	{15, 0x00, 0x93, 0x0d, 0x12, 242508},     //    13.000000
	{16, 0x00, 0x84, 0x10, 0x00, 262416},     //     16.046875
	{17, 0x00, 0x94, 0x12, 0x3a, 275706},     //      18.468750
	{18, 0x01, 0x2c, 0x1a, 0x02, 292248},     //     22.000000
	{19, 0x01, 0x3c, 0x1b, 0x20, 305567},     //      25.328125
	{20, 0x00, 0x8c, 0x20, 0x0f, 324958},     //     31.093750
	{21, 0x00, 0x9c, 0x26, 0x07, 338276},     //     35.796875
	{22, 0x02, 0x64, 0x36, 0x21, 358918},     //      44.531250
	{23, 0x02, 0x74, 0x37, 0x3a, 372262},     //      51.281250
	{24, 0x00, 0xc6, 0x3d, 0x02, 392095},     //     63.250000
	{25, 0x00, 0xdc, 0x3f, 0x3f, 415409},     //      80.937500
	{26, 0x02, 0x85, 0x3f, 0x3f, 421076},     //      85.937500
	{27, 0x02, 0x95, 0x3f, 0x3f, 440355},     //      105.375000
	{28, 0x00, 0xce, 0x3f, 0x3f, 444858},     //      110.515625
};

static inline struct gc2053_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc2053_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct gc2053_info, hdl)->sd;
}
struct regval_list {
	uint8_t reg_num;
	unsigned char value;
};

static struct regval_list gc2053_1920_1080_30fps_mipi_init_regs[] = {
	/*system*/
	{0xfe, 0x80},
	{0xfe, 0x80},
	{0xfe, 0x80},
	{0xfe, 0x00},
	{0xf2, 0x00},
	{0xf3, 0x00},
	{0xf4, 0x36},
	{0xf5, 0xc0},
	{0xf6, 0x44},
	{0xf7, 0x01},
	{0xf8, 0x63},
	{0xf9, 0x40},
	{0xfc, 0x8e},
	/*CISCTL & ANALOG*/
	{0xfe, 0x00},
	{0x87, 0x18},
	{0xee, 0x30},
	{0xd0, 0xb7},
	{0x03, 0x04},
	{0x04, 0x60},   // shutter time
	{0x05, 0x04},
	{0x06, 0x4c},
	{0x07, 0x00},
	{0x08, 0x11},
	{0x09, 0x00},
	{0x0a, 0x02},
	{0x0b, 0x00},
	{0x0c, 0x02},
	{0x0d, 0x04},
	{0x0e, 0x40},
	{0x12, 0xe2},
	{0x13, 0x16},
	{0x19, 0x0a},
	{0x21, 0x1c},
	{0x28, 0x0a},
	{0x29, 0x24},
	{0x2b, 0x04},
	{0x32, 0xf8},
	{0x37, 0x03},
	{0x39, 0x15},
	{0x43, 0x07},
	{0x44, 0x40},
	{0x46, 0x0b},
	{0x4b, 0x20},
	{0x4e, 0x08},
	{0x55, 0x20},
	{0x66, 0x05},
	{0x67, 0x05},
	{0x77, 0x01},
	{0x78, 0x00},
	{0x7c, 0x93},
	{0x8c, 0x12},
	{0x8d, 0x92},
	{0x90, 0x01},
	{0x9d, 0x10},
	{0xce, 0x7c},
	{0xd2, 0x41},
	{0xd3, 0xdc},
	{0xe6, 0x50},
	/*gain*/
	{0xb6, 0xc0},
	{0xb0, 0x70},
	{0xb1, 0x01},
	{0xb2, 0x00},
	{0xb3, 0x00},
	{0xb4, 0x00},
	{0xb8, 0x01},
	{0xb9, 0x00},
	/*blk*/
	{0x26, 0x30},
	{0xfe, 0x01},
	{0x40, 0x23},
	{0x55, 0x07},
	{0x60, 0x40},
	{0xfe, 0x04},
	{0x14, 0x78},
	{0x15, 0x78},
	{0x16, 0x78},
	{0x17, 0x78},
	/*window*/
	{0xfe, 0x01},
	{0x92, 0x00},
	{0x94, 0x03},
	{0x95, 0x04},
	{0x96, 0x38},
	{0x97, 0x07},
	{0x98, 0x80},
	/*ISP*/
	{0xfe, 0x01},
	{0x01, 0x05},
	{0x02, 0x89},
	{0x04, 0x01},
	{0x07, 0xa6},
	{0x08, 0xa9},
	{0x09, 0xa8},
	{0x0a, 0xa7},
	{0x0b, 0xff},
	{0x0c, 0xff},
	{0x0f, 0x00},
	{0x50, 0x1c},
	{0x89, 0x03},
	{0xfe, 0x04},
	{0x28, 0x86},
	{0x29, 0x86},
	{0x2a, 0x86},
	{0x2b, 0x68},
	{0x2c, 0x68},
	{0x2d, 0x68},
	{0x2e, 0x68},
	{0x2f, 0x68},
	{0x30, 0x4f},
	{0x31, 0x68},
	{0x32, 0x67},
	{0x33, 0x66},
	{0x34, 0x66},
	{0x35, 0x66},
	{0x36, 0x66},
	{0x37, 0x66},
	{0x38, 0x62},
	{0x39, 0x62},
	{0x3a, 0x62},
	{0x3b, 0x62},
	{0x3c, 0x62},
	{0x3d, 0x62},
	{0x3e, 0x62},
	{0x3f, 0x62},
	/*DVP & MIPI*/
	{0xfe, 0x01},
	{0x9a, 0x06},
	{0xfe, 0x00},
	{0x7b, 0x2a},
	{0x23, 0x2d},
	{0xfe, 0x03},
	{0x01, 0x27},
	{0x02, 0x5f},
	{0x03, 0xb6},
	{0x12, 0x80},
	{0x13, 0x07},
	{0x15, 0x12},
	{0xfe, 0x00},
	{0x3e, 0x00},
	{GC2053_REG_END, 0x0},
};
#if 0
static struct regval_list gc2053_init_regs_1920_1080_30fps_mipi[] = {
	/* mclk=24mhz,mipi data rate=390mbps/lane, row_time=28.2us frame length=1418,25fps*/
	/*system*/
	{0xfe, 0x80},
	{0xfe, 0x80},
	{0xfe, 0x80},
	{0xfe, 0x00},
	{0xf2, 0x00},
	{0xf3, 0x00},
	{0xf4, 0x36},
	{0xf5, 0xc0},
	{0xf6, 0x44},
	{0xf7, 0x01},
	{0xf8, 0x68},
	{0xf9, 0x40},
	{0xfc, 0x8e},
	/*CISCTL & ANALOG*/
	{0xfe, 0x00},
	{0x87, 0x18},
	{0xee, 0x30},
	{0xd0, 0xb7},
	{0x03, 0x04},
	{0x04, 0x60},
	{0x05, 0x04},
	{0x06, 0x4c},
	{0x07, 0x00},
	{0x08, 0x11},
	{0x09, 0x00},
	{0x0a, 0x02},
	{0x0b, 0x00},
	{0x0c, 0x02},
	{0x0d, 0x04},
	{0x0e, 0x40},
	{0x12, 0xe2},
	{0x13, 0x16},
	{0x19, 0x0a},
	{0x21, 0x1c},
	{0x28, 0x0a},
	{0x29, 0x24},
	{0x2b, 0x04},
	{0x32, 0xf8},
	{0x37, 0x03},
	{0x39, 0x15},
	{0x43, 0x07},
	{0x44, 0x40},
	{0x46, 0x0b},
	{0x4b, 0x20},
	{0x4e, 0x08},
	{0x55, 0x20},
	{0x66, 0x05},
	{0x67, 0x05},
	{0x77, 0x01},
	{0x78, 0x00},
	{0x7c, 0x93},
	{0x8c, 0x12},
	{0x8d, 0x92},
	{0x90, 0x00},/*use frame length to change fps*/
	{0x41, 0x05},
	{0x42, 0x8a},/*vts for 25fps mipi*/
	{0x9d, 0x10},
	{0xce, 0x7c},
	{0xd2, 0x41},
	{0xd3, 0xdc},
	{0xe6, 0x50},
	/*gain*/
	{0xb6, 0xc0},
	{0xb0, 0x70},
	{0xb1, 0x01},
	{0xb2, 0x00},
	{0xb3, 0x00},
	{0xb4, 0x00},
	{0xb8, 0x01},
	{0xb9, 0x00},
	/*blk*/
	{0x26, 0x30},
	{0xfe, 0x01},
	{0x40, 0x23},
	{0x55, 0x07},
	{0x60, 0x40},
	{0xfe, 0x04},
	{0x14, 0x78},
	{0x15, 0x78},
	{0x16, 0x78},
	{0x17, 0x78},
	/*window*/
	{0xfe, 0x01},
	{0x92, 0x00},
	{0x94, 0x03},
	{0x95, 0x04},
	{0x96, 0x38},
	{0x97, 0x07},
	{0x98, 0x80},
	/*ISP*/
	{0xfe, 0x01},
	{0x01, 0x05},
	{0x02, 0x89},
	{0x04, 0x01},
	{0x07, 0xa6},
	{0x08, 0xa9},
	{0x09, 0xa8},
	{0x0a, 0xa7},
	{0x0b, 0xff},
	{0x0c, 0xff},
	{0x0f, 0x00},
	{0x50, 0x1c},
	{0x89, 0x03},
	{0xfe, 0x04},
	{0x28, 0x86},
	{0x29, 0x86},
	{0x2a, 0x86},
	{0x2b, 0x68},
	{0x2c, 0x68},
	{0x2d, 0x68},
	{0x2e, 0x68},
	{0x2f, 0x68},
	{0x30, 0x4f},
	{0x31, 0x68},
	{0x32, 0x67},
	{0x33, 0x66},
	{0x34, 0x66},
	{0x35, 0x66},
	{0x36, 0x66},
	{0x37, 0x66},
	{0x38, 0x62},
	{0x39, 0x62},
	{0x3a, 0x62},
	{0x3b, 0x62},
	{0x3c, 0x62},
	{0x3d, 0x62},
	{0x3e, 0x62},
	{0x3f, 0x62},
	/****DVP & MIPI****/
	{0xfe, 0x01},
	{0x9a, 0x06},
	{0xfe, 0x00},
	{0x7b, 0x2a},
	{0x23, 0x2d},
	{0xfe, 0x03},
	{0x01, 0x27},
	{0x02, 0x5b},/*mipi drv cap*/
	{0x03, 0x8e},/*0xb6*/
	{0x12, 0x80},
	{0x13, 0x07},
	{0x15, 0x12},
	{0x29, 0x05},/*mipi pre*/
	{0x2a, 0x0a},/*mipi zero*/
	{0xfe, 0x00},
	//  {0x3e, 0x91},

	{GC2053_REG_END, 0x00}, /* END MARKER */
};
#endif

/*
 * the part of driver was fixed.
 */

static struct regval_list gc2053_stream_on_mipi[] = {
	{0xfe, 0x00},
	{0x3e, 0x91},
	{GC2053_REG_END, 0x00}, /* END MARKER */
};

static struct regval_list gc2053_stream_off_mipi[] = {
	{0xfe, 0x00},
	{0x3e, 0x00},
	{GC2053_REG_END, 0x00}, /* END MARKER */
};

static int gc2053_read(struct v4l2_subdev *sd, unsigned short reg,
                       unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg[2] = {
		[0] = {
			.addr   = client->addr,
			.flags  = 0,
			.len    = 1,
			.buf    = (unsigned char *) &reg,
		},
		[1] = {
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.len    = 1,
			.buf    = value,
		}
	};
	int ret = 0;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret > 0) {
		ret = 0;
	}

	return ret;
}

static int gc2053_write(struct v4l2_subdev *sd, unsigned short reg,
                        unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t buf[2] = {reg, value};
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

#if 0
static int gc2053_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != GC2053_REG_END) {
		if (vals->reg_num == GC2053_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = gc2053_read(sd, vals->reg_num, &val);
			if (ret < 0) {
				return ret;
			}
		}
		vals++;
	}
	return 0;
}
#endif

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int gc2053_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != GC2053_REG_END) {
		if (vals->reg_num == GC2053_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = gc2053_write(sd, vals->reg_num, vals->value);
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
#if 0
static int gc2053_lightup(struct v4l2_subdev *sd, u32 val)
{
	struct gc2053_info *info = to_state(sd);
	if (val) {
		gpio_direction_output(info->led.pin, info->led.active_level);
	} else {
		gpio_direction_output(info->led.pin, !info->led.active_level);
	}
	return 0;
}
#endif
static int gc2053_reset(struct v4l2_subdev *sd, u32 val)
{
	struct gc2053_info *info = to_state(sd);

	if (val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(10);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(10);
	}
	return 0;
}

static int gc2053_power(struct v4l2_subdev *sd, u32 val)
{
	struct gc2053_info *info = to_state(sd);

	if (val) {
		gpio_direction_output(info->pwdn.pin, !info->pwdn.active_level);
		msleep(10);
		gpio_direction_output(info->pwdn.pin, info->pwdn.active_level);
		msleep(10);
	}
	return 0;
}

static int gc2053_init(struct v4l2_subdev *sd, u32 val)
{
	struct gc2053_info *info = to_state(sd);
	int ret = 0;

	ret = gc2053_write_array(sd, info->win->regs);

	return ret;
}

static int gc2053_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;
	ret = gc2053_read(sd, 0xf0, &v);
	printk("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret, v);
	if (ret < 0) {
		return ret;
	}
	if (v != GC2053_CHIP_ID_H) {
		return -ENODEV;
	}
	*ident = v;

	ret = gc2053_read(sd, 0xf1, &v);
	printk("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret, v);
	if (ret < 0) {
		return ret;
	}
	if (v != GC2053_CHIP_ID_L) {
		return -ENODEV;
	}
	*ident = (*ident << 8) | v;

	return 0;
}

static struct gc2053_win_size gc2053_win_sizes[] = {
	{
		.sensor_info.mipi_cfg.clk       = 600,
		.sensor_info.mipi_cfg.twidth        = 1920,
		.sensor_info.mipi_cfg.theight       = 1080,
		.sensor_info.mipi_cfg.mipi_mode     = SENSOR_MIPI_OTHER_MODE,
		.sensor_info.mipi_cfg.mipi_vcomp_en     = 0,
		.sensor_info.mipi_cfg.mipi_hcomp_en     = 0,
		.sensor_info.mipi_cfg.mipi_crop_start0x = 0,
		.sensor_info.mipi_cfg.mipi_crop_start0y = 0,
		.sensor_info.mipi_cfg.mipi_crop_start1x = 0,
		.sensor_info.mipi_cfg.mipi_crop_start1y = 0,
		.sensor_info.mipi_cfg.mipi_crop_start2x = 0,
		.sensor_info.mipi_cfg.mipi_crop_start2y = 0,
		.sensor_info.mipi_cfg.mipi_crop_start3x = 0,
		.sensor_info.mipi_cfg.mipi_crop_start3y = 0,
		.sensor_info.mipi_cfg.hcrop_diff_en     = 0,
		.sensor_info.mipi_cfg.line_sync_mode    = 0,
		.sensor_info.mipi_cfg.work_start_flag   = 0,
		.sensor_info.mipi_cfg.data_type_en      = 0,
		.sensor_info.mipi_cfg.data_type_value   = 0,
		.sensor_info.mipi_cfg.del_start     = 0,
		.sensor_info.mipi_cfg.sensor_frame_mode = TX_SENSOR_DEFAULT_FRAME_MODE,
		.sensor_info.mipi_cfg.sensor_fid_mode   = 0,
		.sensor_info.mipi_cfg.sensor_mode       = TX_SENSOR_DEFAULT_MODE,
		.sensor_info.mipi_cfg.sensor_csi_fmt    = TX_SENSOR_RAW10,

		.width              = 1920,
		.height             = 1080,
		.sensor_info.fps                = 30 << 16 | 1,
		.sensor_info.total_height           = 1124,
		.mbus_code  = MEDIA_BUS_FMT_SRGGB10_1X10,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.regs       = gc2053_1920_1080_30fps_mipi_init_regs,
		//      .regs       = gc2053_init_regs_1920_1080_30fps_mipi,

	},
};

#if 0
static int gc2053_enum_mbus_code(struct v4l2_subdev *sd,
                                 struct v4l2_subdev_pad_state *state,
                                 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= N_gc2053_FMTS) {
		return -EINVAL;
	}

	code->code = gc2053_formats[code->index].mbus_code;
	return 0;
}
#endif

/*
 * Set a format.
 */
static int gc2053_set_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *format)
{
	if (format->pad) {
		return -EINVAL;
	}

	return 0;
}

static int gc2053_get_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *format)
{
	struct gc2053_info *info = to_state(sd);
	struct gc2053_win_size *wsize = info->win;
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
	ISP_SET_SENSOR_INFO(fmt, info);

	printk("----%s, %d, width: %d, height: %d, code: %x\n",
	       __func__, __LINE__, fmt->width, fmt->height, fmt->code);

	return ret;
}

static int gc2053_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc2053_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc2053_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc2053_s_vflip(struct v4l2_subdev *sd, int value)
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
static int gc2053_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int gc2053_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static unsigned int again_to_regval(int gain)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(gc2053_again_lut); i++) {
		lut = &gc2053_again_lut[i];

		if (gain <= lut->gain) {
			return lut->index;
		}
	}

	/*last value.*/
	return lut->index;
}
static int gc2053_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;

	return ret;
}
/*set analog gain db value, map value to sensor register.*/
static int gc2053_s_again(struct v4l2_subdev *sd, int again)
{
	struct gc2053_info *info = to_state(sd);
	struct again_lut *val_lut = gc2053_again_lut;
	unsigned int reg_value = 0;
	int ret = 0;

	if (again < info->again->minimum || again > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(again);
	}

	ret = gc2053_write(sd, 0xfe, 0x00);
	ret += gc2053_write(sd, 0xb4, val_lut[reg_value].regb4);
	ret += gc2053_write(sd, 0xb3, val_lut[reg_value].regb3);
	ret += gc2053_write(sd, 0xb8, val_lut[reg_value].dpc);
	ret += gc2053_write(sd, 0xb9, val_lut[reg_value].blc);
	if (ret < 0) {
		printk("%s : gc2053_write error %d~~~~\n", __func__, __LINE__);
		return ret;
	}
	return 0;

}

/*
 * Tweak autogain.
 */
static int gc2053_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc2053_s_exp(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	ret = gc2053_write(sd, 0xfe, 0x00);
	ret = gc2053_write(sd, 0x04, value & 0xff);
	ret += gc2053_write(sd, 0x03, ((value >> 8) & 0x3f));
	return ret;
}

static int gc2053_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct gc2053_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		return gc2053_g_gain(sd, &info->gain->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return gc2053_g_again(sd, &info->again->val);
	}
	return -EINVAL;
}

static int gc2053_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct gc2053_info *info = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return gc2053_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return gc2053_s_contrast(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return gc2053_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return gc2053_s_hflip(sd, ctrl->val);
	case V4L2_CID_AUTOGAIN:
		/* Only set manual gain if auto gain is not explicitly
		   turned on. */
		if (!ctrl->val) {
			/* gc2053_s_gain turns off auto gain */
			return gc2053_s_gain(sd, info->gain->val);
		}
		return gc2053_s_autogain(sd, ctrl->val);
	case V4L2_CID_GAIN:
		return gc2053_s_gain(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return gc2053_s_again(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return gc2053_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops gc2053_ctrl_ops = {
	.s_ctrl = gc2053_s_ctrl,
	.g_volatile_ctrl = gc2053_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int gc2053_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = gc2053_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int gc2053_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	gc2053_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

int gc2053_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;
	if (enable) {
		ret = gc2053_write_array(sd, gc2053_stream_on_mipi);
		printk("gc2053 stream on\n");

	} else {
		ret = gc2053_write_array(sd, gc2053_stream_off_mipi);
		printk("gc2053 stream off\n");
	}
	return ret;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops gc2053_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = gc2053_g_register,
	.s_register = gc2053_s_register,
#endif
	//  .g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	//  .try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	//  .s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	//  .g_ctrl = v4l2_subdev_g_ctrl,
	//  .s_ctrl = v4l2_subdev_s_ctrl,
	//  .queryctrl = v4l2_subdev_queryctrl,
	//  .querymenu = v4l2_subdev_querymenu,

};

static const struct v4l2_subdev_video_ops gc2053_video_ops = {
	.s_stream = gc2053_s_stream,
};

static const struct v4l2_subdev_pad_ops gc2053_pad_ops = {
	//.enum_frame_interval = gc2053_enum_frame_interval,
	//.num_frame_size = gc2053_enum_frame_size,
	//.enum_mbus_code = gc2053_enum_mbus_code,
	.set_fmt = gc2053_set_fmt,
	.get_fmt = gc2053_get_fmt,
};

static const struct v4l2_subdev_ops gc2053_ops = {
	.core = &gc2053_core_ops,
	.video = &gc2053_video_ops,
	.pad = &gc2053_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int gc2053_probe(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	struct gc2053_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		return -ENOMEM;
	}
	sd = &info->sd;

	gpio = of_get_named_gpio(client->dev.of_node, "ingenic,led-gpio", 0);
	if (gpio_is_valid(gpio)) {
		info->led.pin = gpio;
		info->led.active_level = INGENIC_ISP_GPIO_ACTIVE_HIGH;
	}
	gpio = of_get_named_gpio(client->dev.of_node, "ingenic,rst-gpio", 0);
	if (gpio_is_valid(gpio)) {
		info->reset.pin = gpio;
		info->reset.active_level = INGENIC_ISP_GPIO_ACTIVE_LOW;
	}
	gpio = of_get_named_gpio(client->dev.of_node, "ingenic,pwdn-gpio", 0);
	if (gpio_is_valid(gpio)) {
		info->pwdn.pin = gpio;
		info->pwdn.active_level = INGENIC_ISP_GPIO_ACTIVE_HIGH;
	}

	gc2053_power(sd, 1);
	gc2053_reset(sd, 1);

	v4l2_i2c_subdev_init(sd, client, &gc2053_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

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

#if 1
	/* Make sure it's an gc2053 */
	ret = gc2053_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
		        "chip found @ 0x%x (%s) is not an gc2053 chip.\n",
		        client->addr << 1, client->adapter->name);
		return ret;
	}
#endif
#if 1
	/*IRCUT ctl 0:off 1:on*/
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
	         client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &gc2053_ctrl_ops,
	                  V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &gc2053_ctrl_ops,
	                  V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &gc2053_ctrl_ops,
	                  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &gc2053_ctrl_ops,
	                  V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &gc2053_ctrl_ops,
	                               V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &gc2053_ctrl_ops,
	                                V4L2_CID_ANALOGUE_GAIN, 0, 444858, 1, 10000);

	/*unit exposure lines: */
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &gc2053_ctrl_ops,
	                                   V4L2_CID_EXPOSURE, 1, 0x440 + 36, 1, 500);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;
		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->win = &gc2053_win_sizes[0];

	gc2053_init(sd, 1);
	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if (ret < 0) {
		goto err_entity_init;
	}
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0) {
		goto err_videoprobe;
	}

	dev_info(&client->dev, "gc2053 Probed\n");
	return 0;
#endif
err_videoprobe:
err_entity_init:
	devm_clk_put(&client->dev, info->clk);
err_clkget:
	devm_clk_put(&client->dev, info->mclk);
err_mclkget:
	return ret;
}

static void gc2053_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2053_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	devm_clk_put(&client->dev, info->clk);
	devm_clk_put(&client->dev, info->mclk);
}

static const struct i2c_device_id gc2053_id[] = {
	{ "gc2053", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, gc2053_id);

static const struct of_device_id gc2053_of_match[] = {
	{.compatible = "galaxy,gc2053", },
	{},
};
MODULE_DEVICE_TABLE(of, gc2053_of_match);

static struct i2c_driver gc2053_driver = {
	.driver = {
		.name   = "gc2053",
		.of_match_table = of_match_ptr(gc2053_of_match),
	},
	.probe      = gc2053_probe,
	.remove     = gc2053_remove,
	.id_table   = gc2053_id,
};

module_i2c_driver(gc2053_driver);
MODULE_DESCRIPTION("A low-level driver for gc2053 sensors");
MODULE_LICENSE("GPL");
