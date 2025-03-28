/*
 * A V4L2 driver for OmniVision GC2155 cameras.
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

#define GC2155_CHIP_ID_H    (0x21)
#define GC2155_CHIP_ID_L    (0x55)
#define GC2155_REG_END      0xff
#define GC2155_REG_DELAY    0x00

#define AGAIN_MAX_DB 0x64
#define DGAIN_MAX_DB 0x64
#define LOG2_GAIN_SHIFT 16

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct gc2155_win_size {
	unsigned int width;
	unsigned int height;
	struct sensor_info sensor_info;
	unsigned int mbus_code;
	enum v4l2_colorspace colorspace;
	void *regs;
};

struct gc2155_gpio {
	int pin;
	int active_level;
};

struct gc2155_info {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *again;
	struct v4l2_ctrl *again_short;

	struct clk *clk;
	struct clk *sclka;

	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *exposure_short;

	struct media_pad pad;

	struct v4l2_subdev_format *format;  /*current fmt.*/
	struct gc2155_win_size *win;

	struct gc2155_gpio pwdn;
	struct gc2155_gpio reset;
};

static inline struct gc2155_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc2155_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct gc2155_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};

static struct regval_list gc2155_init_regs[] = {
	{0xfe, 0xf0},
	{0xfe, 0xf0},
	{0xfe, 0xf0},
	{0xfc, 0x06},
	{0xf6, 0x00},
	{0xf7, 0x1d},
	{0xf8, 0x84},
	{0xfa, 0x00},
	{0xf9, 0xfe},
	{0xf2, 0x00},
	{0xfe, 0x00},
	{0x03, 0x04},
	{0x04, 0xe2},
	{0x09, 0x00},
	{0x0a, 0x00},
	{0x0b, 0x00},
	{0x0c, 0x00},
	{0x0d, 0x04},
	{0x0e, 0xc0},
	{0x0f, 0x06},
	{0x10, 0x50},
	{0x12, 0x2e},
	{0x17, 0x14}, // mirror
	{0x18, 0x02},
	{0x19, 0x0e},
	{0x1a, 0x01},
	{0x1b, 0x4b},
	{0x1c, 0x07},
	{0x1d, 0x10},
	{0x1e, 0x98},
	{0x1f, 0x78},
	{0x20, 0x05},
	{0x21, 0x40},
	{0x22, 0xf0},
	{0x24, 0x16},
	{0x25, 0x01},
	{0x26, 0x10},
	{0x2d, 0x40},
	{0x30, 0x01},
	{0x31, 0x90},
	{0x33, 0x04},
	{0x34, 0x01},
	{0xfe, 0x00},
	{0x80, 0xff},
	{0x81, 0x2c},
	{0x82, 0xfa},
	{0x83, 0x00},
	{0x84, 0x02}, //y u yv
	{0x85, 0x08},
	{0x86, 0x02},
	{0x89, 0x03},
	{0x8a, 0x00},
	{0x8b, 0x00},
	{0xb0, 0x55},
	{0xc3, 0x11}, //00
	{0xc4, 0x20},
	{0xc5, 0x30},
	{0xc6, 0x38},
	{0xc7, 0x40},
	{0xec, 0x02},
	{0xed, 0x04},
	{0xee, 0x60},
	{0xef, 0x90},
	{0xb6, 0x01},
	{0x90, 0x01},
	{0x91, 0x00},
	{0x92, 0x00},
	{0x93, 0x00},
	{0x94, 0x00},
	{0x95, 0x04},
	{0x96, 0xb0},
	{0x97, 0x06},
	{0x98, 0x40},
	{0xfe, 0x00},
	{0x18, 0x02},
	{0x40, 0x42},
	{0x41, 0x00},
	{0x43, 0x5b}, //0X54
	{0x5e, 0x00},
	{0x5f, 0x00},
	{0x60, 0x00},
	{0x61, 0x00},
	{0x62, 0x00},
	{0x63, 0x00},
	{0x64, 0x00},
	{0x65, 0x00},
	{0x66, 0x20},
	{0x67, 0x20},
	{0x68, 0x20},
	{0x69, 0x20},
	{0x6a, 0x08},
	{0x6b, 0x08},
	{0x6c, 0x08},
	{0x6d, 0x08},
	{0x6e, 0x08},
	{0x6f, 0x08},
	{0x70, 0x08},
	{0x71, 0x08},
	{0x72, 0xf0},
	{0x7e, 0x3c},
	{0x7f, 0x00},
	{0xfe, 0x00},
	{0xfe, 0x01},
	{0x01, 0x08},
	{0x02, 0xc0},
	{0x03, 0x04},
	{0x04, 0x90},
	{0x05, 0x30},
	{0x06, 0x98},
	{0x07, 0x28},
	{0x08, 0x6c},
	{0x09, 0x00},
	{0x0a, 0xc2},
	{0x0b, 0x11},
	{0x0c, 0x10},
	{0x13, 0x2d},
	{0x17, 0x00},
	{0x1c, 0x11},
	{0x1e, 0x61},
	{0x1f, 0x30},
	{0x20, 0x40},
	{0x22, 0x80},
	{0x23, 0x20},
	{0x12, 0x35},
	{0x15, 0x50},
	{0x10, 0x31},
	{0x3e, 0x28},
	{0x3f, 0xe0},
	{0x40, 0xe0},
	{0x41, 0x08},
	{0xfe, 0x02},
	{0x0f, 0x05},
	{0xfe, 0x02},
	{0x90, 0x6c},
	{0x91, 0x03},
	{0x92, 0xc4},
	{0x97, 0x64},
	{0x98, 0x88},
	{0x9d, 0x08},
	{0xa2, 0x11},
	{0xfe, 0x00},
	{0xfe, 0x02},
	{0x80, 0xc1},
	{0x81, 0x08},
	{0x82, 0x05},
	{0x83, 0x04},
	{0x86, 0x80},
	{0x87, 0x30},
	{0x88, 0x15},
	{0x89, 0x80},
	{0x8a, 0x60},
	{0x8b, 0x30},
	{0xfe, 0x01},
	{0x21, 0x14},
	{0xfe, 0x02},
	{0x3c, 0x06},
	{0x3d, 0x40},
	{0x48, 0x30},
	{0x49, 0x06},
	{0x4b, 0x08},
	{0x4c, 0x20},
	{0xa3, 0x50},
	{0xa4, 0x30},
	{0xa5, 0x40},
	{0xa6, 0x80},
	{0xab, 0x40},
	{0xae, 0x0c},
	{0xb3, 0x42},
	{0xb4, 0x24},
	{0xb6, 0x50},
	{0xb7, 0x01},
	{0xb9, 0x28},
	{0xfe, 0x00},
	{0xfe, 0x02},
	{0x10, 0x0d},
	{0x11, 0x12},
	{0x12, 0x17},
	{0x13, 0x1c},
	{0x14, 0x27},
	{0x15, 0x34},
	{0x16, 0x44},
	{0x17, 0x55},
	{0x18, 0x6e},
	{0x19, 0x81},
	{0x1a, 0x91},
	{0x1b, 0x9c},
	{0x1c, 0xaa},
	{0x1d, 0xbb},
	{0x1e, 0xca},
	{0x1f, 0xd5},
	{0x20, 0xe0},
	{0x21, 0xe7},
	{0x22, 0xed},
	{0x23, 0xf6},
	{0x24, 0xfb},
	{0x25, 0xff},
	{0xfe, 0x02},
	{0x26, 0x0d},
	{0x27, 0x12},
	{0x28, 0x17},
	{0x29, 0x1c},
	{0x2a, 0x27},
	{0x2b, 0x34},
	{0x2c, 0x44},
	{0x2d, 0x55},
	{0x2e, 0x6e},
	{0x2f, 0x81},
	{0x30, 0x91},
	{0x31, 0x9c},
	{0x32, 0xaa},
	{0x33, 0xbb},
	{0x34, 0xca},
	{0x35, 0xd5},
	{0x36, 0xe0},
	{0x37, 0xe7},
	{0x38, 0xed},
	{0x39, 0xf6},
	{0x3a, 0xfb},
	{0x3b, 0xff},
	{0xfe, 0x02},
	{0xd1, 0x28},
	{0xd2, 0x28},
	{0xdd, 0x14},
	{0xde, 0x88},
	{0xed, 0x80},
	{0xfe, 0x01},
	{0xc2, 0x1f},
	{0xc3, 0x13},
	{0xc4, 0x0e},
	{0xc8, 0x16},
	{0xc9, 0x0f},
	{0xca, 0x0c},
	{0xbc, 0x52},
	{0xbd, 0x2c},
	{0xbe, 0x27},
	{0xb6, 0x47},
	{0xb7, 0x32},
	{0xb8, 0x30},
	{0xc5, 0x00},
	{0xc6, 0x00},
	{0xc7, 0x00},
	{0xcb, 0x00},
	{0xcc, 0x00},
	{0xcd, 0x00},
	{0xbf, 0x0e},
	{0xc0, 0x00},
	{0xc1, 0x00},
	{0xb9, 0x08},
	{0xba, 0x00},
	{0xbb, 0x00},
	{0xaa, 0x0a},
	{0xab, 0x0c},
	{0xac, 0x0d},
	{0xad, 0x02},
	{0xae, 0x06},
	{0xaf, 0x05},
	{0xb0, 0x00},
	{0xb1, 0x05},
	{0xb2, 0x02},
	{0xb3, 0x04},
	{0xb4, 0x04},
	{0xb5, 0x05},
	{0xd0, 0x00},
	{0xd1, 0x00},
	{0xd2, 0x00},
	{0xd6, 0x02},
	{0xd7, 0x00},
	{0xd8, 0x00},
	{0xd9, 0x00},
	{0xda, 0x00},
	{0xdb, 0x00},
	{0xd3, 0x00},
	{0xd4, 0x00},
	{0xd5, 0x00},
	{0xa4, 0x04},
	{0xa5, 0x00},
	{0xa6, 0x77},
	{0xa7, 0x77},
	{0xa8, 0x77},
	{0xa9, 0x77},
	{0xa1, 0x80},
	{0xa2, 0x80},
	{0xfe, 0x01},
	{0xdc, 0x35},
	{0xdd, 0x28},
	{0xdf, 0x0d},
	{0xe0, 0x70},
	{0xe1, 0x78},
	{0xe2, 0x70},
	{0xe3, 0x78},
	{0xe6, 0x90},
	{0xe7, 0x70},
	{0xe8, 0x90},
	{0xe9, 0x70},
	{0xfe, 0x00},
	{0xfe, 0x01},
	{0x4f, 0x00},
	{0x4f, 0x00},
	{0x4b, 0x01},
	{0x4f, 0x00},
	{0x4c, 0x01},
	{0x4d, 0x71},
	{0x4e, 0x02},
	{0x4c, 0x01},
	{0x4d, 0x91},
	{0x4e, 0x02},
	{0x4c, 0x01},
	{0x4d, 0x50},
	{0x4e, 0x02},
	{0x4c, 0x01},
	{0x4d, 0x70},
	{0x4e, 0x02},
	{0x4c, 0x01},
	{0x4d, 0x90},
	{0x4e, 0x02},
	{0x4c, 0x01},
	{0x4d, 0xb0},
	{0x4e, 0x02},
	{0x4c, 0x01},
	{0x4d, 0xd0},
	{0x4e, 0x02},
	{0x4c, 0x01},
	{0x4d, 0x4f},
	{0x4e, 0x02},
	{0x4c, 0x01},
	{0x4d, 0x6f},
	{0x4e, 0x02},
	{0x4c, 0x01},
	{0x4d, 0x8f},
	{0x4e, 0x02},
	{0x4c, 0x01},
	{0x4d, 0xaf},
	{0x4e, 0x02},
	{0x4c, 0x01},
	{0x4d, 0xcf},
	{0x4e, 0x02},
	{0x4c, 0x01},
	{0x4d, 0x6e},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0x8e},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0xae},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0xce},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0x4d},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0x6d},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0x8d},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0xad},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0xcd},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0x4c},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0x6c},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0x8c},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0xac},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0xcc},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0xec},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0x4b},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0x6b},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0x8b},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0xab},
	{0x4e, 0x03},
	{0x4c, 0x01},
	{0x4d, 0x8a},
	{0x4e, 0x04},
	{0x4c, 0x01},
	{0x4d, 0xaa},
	{0x4e, 0x04},
	{0x4c, 0x01},
	{0x4d, 0xca},
	{0x4e, 0x04},
	{0x4c, 0x01},
	{0x4d, 0xa9},
	{0x4e, 0x04},
	{0x4c, 0x01},
	{0x4d, 0xc9},
	{0x4e, 0x04},
	{0x4c, 0x01},
	{0x4d, 0xcb},
	{0x4e, 0x05},
	{0x4c, 0x01},
	{0x4d, 0xeb},
	{0x4e, 0x05},
	{0x4c, 0x02},
	{0x4d, 0x0b},
	{0x4e, 0x05},
	{0x4c, 0x02},
	{0x4d, 0x2b},
	{0x4e, 0x05},
	{0x4c, 0x02},
	{0x4d, 0x4b},
	{0x4e, 0x05},
	{0x4c, 0x01},
	{0x4d, 0xea},
	{0x4e, 0x05},
	{0x4c, 0x02},
	{0x4d, 0x0a},
	{0x4e, 0x05},
	{0x4c, 0x02},
	{0x4d, 0x2a},
	{0x4e, 0x05},
	{0x4c, 0x02},
	{0x4d, 0x6a},
	{0x4e, 0x06},
	{0x4c, 0x02},
	{0x4d, 0x29},
	{0x4e, 0x06},
	{0x4c, 0x02},
	{0x4d, 0x49},
	{0x4e, 0x06},
	{0x4c, 0x02},
	{0x4d, 0x69},
	{0x4e, 0x06},
	{0x4c, 0x02},
	{0x4d, 0x89},
	{0x4e, 0x06},
	{0x4c, 0x02},
	{0x4d, 0xa9},
	{0x4e, 0x06},
	{0x4c, 0x02},
	{0x4d, 0xc9},
	{0x4e, 0x06},
	{0x4c, 0x02},
	{0x4d, 0x48},
	{0x4e, 0x06},
	{0x4c, 0x02},
	{0x4d, 0x68},
	{0x4e, 0x06},
	{0x4c, 0x03},
	{0x4d, 0x09},
	{0x4e, 0x07},
	{0x4c, 0x02},
	{0x4d, 0xa8},
	{0x4e, 0x07},
	{0x4c, 0x02},
	{0x4d, 0xc8},
	{0x4e, 0x07},
	{0x4c, 0x02},
	{0x4d, 0xe8},
	{0x4e, 0x07},
	{0x4c, 0x03},
	{0x4d, 0x08},
	{0x4e, 0x07},
	{0x4c, 0x03},
	{0x4d, 0x28},
	{0x4e, 0x07},
	{0x4c, 0x02},
	{0x4d, 0x87},
	{0x4e, 0x07},
	{0x4c, 0x02},
	{0x4d, 0xa7},
	{0x4e, 0x07},
	{0x4c, 0x02},
	{0x4d, 0xc7},
	{0x4e, 0x07},
	{0x4c, 0x02},
	{0x4d, 0xe7},
	{0x4e, 0x07},
	{0x4c, 0x03},
	{0x4d, 0x07},
	{0x4e, 0x07},
	{0x4f, 0x01},
	{0xfe, 0x01},
	{0x50, 0x80},
	{0x51, 0xa8},
	{0x52, 0x57},
	{0x53, 0x38},
	{0x54, 0xc7},
	{0x56, 0x0e},
	{0x58, 0x08},
	{0x5b, 0x00},
	{0x5c, 0x74},
	{0x5d, 0x8b},
	{0x61, 0xd3},
	{0x62, 0x90},
	{0x63, 0xaa},
	{0x65, 0x04},
	{0x67, 0xb2},
	{0x68, 0xac},
	{0x69, 0x00},
	{0x6a, 0xb2},
	{0x6b, 0xac},
	{0x6c, 0xdc},
	{0x6d, 0xb0},
	{0x6e, 0x30},
	{0x6f, 0x40},
	{0x70, 0x05},
	{0x71, 0x80},
	{0x72, 0x80},
	{0x73, 0x30},
	{0x74, 0x01},
	{0x75, 0x01},
	{0x7f, 0x08},
	{0x76, 0x70},
	{0x77, 0x48},
	{0x78, 0xa0},
	{0xfe, 0x00},
	{0xfe, 0x02},
	{0xc0, 0x01},
	{0xc1, 0x4a},
	{0xc2, 0xf3},
	{0xc3, 0xfc},
	{0xc4, 0xe4},
	{0xc5, 0x48},
	{0xc6, 0xec},
	{0xc7, 0x45},
	{0xc8, 0xf8},
	{0xc9, 0x02},
	{0xca, 0xfe},
	{0xcb, 0x42},
	{0xcc, 0x00},
	{0xcd, 0x45},
	{0xce, 0xf0},
	{0xcf, 0x00},
	{0xe3, 0xf0},
	{0xe4, 0x45},
	{0xe5, 0xe8},
	{0xfe, 0x01},
	{0x9f, 0x42},
	{0xfe, 0x00},
	{0xfe, 0x00},
	{0xf2, 0x0f},
	{0xfe, 0x00},
	{0x05, 0x01},
	{0x06, 0x56},
	{0x07, 0x00},
	{0x08, 0x32},
	{0xfe, 0x01},
	{0x25, 0x00},
	{0x26, 0xfa},
	{0x27, 0x04},
	{0x28, 0xe2}, //20fps
	{0x29, 0x06},
	{0x2a, 0xd6}, //16fps
	{0x2b, 0x07},
	{0x2c, 0xd0}, //12fps
	{0x2d, 0x0b},
	{0x2e, 0xb8}, //8fps
	{0xfe, 0x00},
	{0xfe, 0x00},
	{0xfa, 0x00},
	{0xfd, 0x01},
	{0xfe, 0x00},
	{0x90, 0x01},
	{0x91, 0x00},
	{0x92, 0x00},
	{0x93, 0x00},
	{0x95, 0x04}, // win_size 320 * 240
	{0x96, 0xb0},
	{0x97, 0x06},
	{0x98, 0x40},

	{0x99, 0x11},
	{0x9a, 0x06},
	{0xfe, 0x01},
	{0xec, 0x01},
	{0xed, 0x02},
	{0xee, 0x30},
	{0xef, 0x48},
	{0xfe, 0x01},
	{0x74, 0x00},
	{0xfe, 0x01},
	{0x01, 0x04},
	{0x02, 0x60},
	{0x03, 0x02},
	{0x04, 0x48},
	{0x05, 0x18},
	{0x06, 0x4c},
	{0x07, 0x14},
	{0x08, 0x36},
	{0x0a, 0xc0},
	{0x21, 0x14},
	{0xfe, 0x00},
	{0xfe, 0x00},
	{0xc3, 0x11},
	{0xc4, 0x20},
	{0xc5, 0x30},
	{0xfa, 0x11}, //pclk rate
	{0x86, 0x02}, //pclk polar
	{0xfe, 0x00},

#if 1
	// 720P init
	{0xfe, 0x00},
	{0xb6, 0x01},
	{0xfd, 0x00},

	//subsample
	{0xfe, 0x00},
	{0x99, 0x55},
	{0x9a, 0x06},
	{0x9b, 0x00},
	{0x9c, 0x00},
	{0x9d, 0x01},
	{0x9e, 0x23},
	{0x9f, 0x00},
	{0xa0, 0x00},
	{0xa1, 0x01},
	{0xa2, 0x23},
	//crop window
	{0x90, 0x01},
	{0x91, 0x00},
	{0x92, 0x78},
	{0x93, 0x00},
	{0x94, 0x00},
	{0x95, 0x02},
	{0x96, 0xd0},
	{0x97, 0x05},
	{0x98, 0x00},
	// AWB
	{0xfe, 0x00},
	{0xec, 0x02},
	{0xed, 0x04},
	{0xee, 0x60},
	{0xef, 0x90},
	{0xfe, 0x01},
	{0x74, 0x01},
	// AEC
	{0xfe, 0x01},
	{0x01, 0x08},
	{0x02, 0xc0},
	{0x03, 0x04},
	{0x04, 0x90},
	{0x05, 0x30},
	{0x06, 0x98},
	{0x07, 0x28},
	{0x08, 0x6c},
	{0x0a, 0xc2},
	{0x21, 0x15},
	{0xfe, 0x00},
#if 0
	//banding setting 20fps fixed///
	{0xfe, 0x00},
	{0x03, 0x03},
	{0x04, 0xe8},
	{0x05, 0x01},
	{0x06, 0x56},
	{0x07, 0x00},
	{0x08, 0x32},
	{0xfe, 0x01},
	{0x25, 0x00},
	{0x26, 0xfa},
	{0x27, 0x04},
	{0x28, 0xe2}, //20fps
	{0x29, 0x04},
	{0x2a, 0xe2}, //16fps   5dc
	{0x2b, 0x04},
	{0x2c, 0xe2}, //16fps  6d6  5dc
	{0x2d, 0x04},
	{0x2e, 0xe2}, //8fps    bb8
	{0x3c, 0x00}, //8fps
	{0xfe, 0x00},
#endif
#endif
#if 0
	/*qvga*/
	{0xfe, 0x00},
	{0xfd, 0x01},
	// crop window
	{0xfe, 0x00},
	{0x90, 0x01},
	{0x91, 0x00},
	{0x92, 0xb4},
	{0x93, 0x00},
	{0x94, 0xf0},
	{0x95, 0x00},
	{0x96, 0xf0},
	{0x97, 0x01},
	{0x98, 0x40},
	// AWB
	{0xfe, 0x00},
	{0xec, 0x01},
	{0xed, 0x02},
	{0xee, 0x30},
	{0xef, 0x48},
	{0xfe, 0x01},
	{0x74, 0x00},
	//// AEC
	{0xfe, 0x01},
	{0x01, 0x04},
	{0x02, 0x60},
	{0x03, 0x02},
	{0x04, 0x48},
	{0x05, 0x18},
	{0x06, 0x4c},
	{0x07, 0x14},
	{0x08, 0x36},
	{0x0a, 0xc0},
	{0x21, 0x14},
#if 0
	{0x25, 0x01},
	{0x26, 0x90},
	{0x27, 0x03},
	{0x28, 0x20}, //50fps
	{0x29, 0x03},
	{0x2a, 0x20},
	{0x2b, 0x03},
	{0x2c, 0x20},
	{0x2d, 0x03},
	{0x2e, 0x20},
#endif
	{0xfe, 0x00},
	//// gamma
	{0xfe, 0x00},
	{0xc3, 0x11},
	{0xc4, 0x20},
	{0xc5, 0x30},
	{0xfe, 0x00},
#endif

	{0xfe, 0x00},
	{0x8d, 0x08},

	{GC2155_REG_END, 0x00},    /* END MARKER */

};

/*
 * the part of driver was fixed.
 */

static struct regval_list gc2155_stream_on_mipi[] = {
	/*
	{0x3000, 0x00},
	*/
	{GC2155_REG_END, 0x00}, /* END MARKER */
};

static struct regval_list gc2155_stream_off_mipi[] = {
	/*
	{0x3000, 0x01},
	*/
	{GC2155_REG_END, 0x00}, /* END MARKER */
};

static int gc2155_read(struct v4l2_subdev *sd, unsigned char reg,
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
	int ret = 0;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret > 0) {
		ret = 0;
	}

	return ret;
}

static int gc2155_write(struct v4l2_subdev *sd, unsigned char reg,
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
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0) {
		ret = 0;
	}

	return ret;
}

#if 0
static int gc2155_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != GC2155_REG_END) {
		if (vals->reg_num == GC2155_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = gc2155_read(sd, vals->reg_num, &val);
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
static int gc2155_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != GC2155_REG_END) {
		if (vals->reg_num == GC2155_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = gc2155_write(sd, vals->reg_num, vals->value);
			if (ret < 0) {
				return ret;
			}
		}
		vals++;
	}
	return 0;
}

static int gc2155_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;
	ret = gc2155_read(sd, 0xf0, &v);
	//  pr_info("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0) {
		return ret;
	}
	if (v != GC2155_CHIP_ID_H) {
		return -ENODEV;
	}
	*ident = v;

	ret = gc2155_read(sd, 0xf1, &v);
	//  pr_info("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret,v);
	if (ret < 0) {
		return ret;
	}
	if (v != GC2155_CHIP_ID_L) {
		return -ENODEV;
	}
	*ident = (*ident << 8) | v;

	return 0;
}

static struct gc2155_win_size gc2155_win_sizes[] = {
	{
		.width              = 1280,
		.height             = 720,
		.sensor_info.fps                = 25 << 16 | 1,
		                                     .sensor_info.total_width            = 1280,
		                                     .sensor_info.total_height           = 720,
		                                     .sensor_info.wdr_en             = 0,
		                                     .sensor_info.dvp_cfg.hfb_num        = 0,
		                                     .sensor_info.dvp_cfg.in_ver_para3       = 0,
		                                     .sensor_info.dvp_cfg.dvp_mode       = SENSOR_DVP_HREF_MODE,
		                                     .mbus_code  = MEDIA_BUS_FMT_YUYV8_2X8,
		                                     .colorspace = V4L2_COLORSPACE_SRGB,
		                                     .regs       = gc2155_init_regs,
	},
};

#if 0
static int gc2155_enum_mbus_code(struct v4l2_subdev *sd,
                                 struct v4l2_subdev_state *state,
                                 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= N_GC2155_FMTS) {
		return -EINVAL;
	}

	code->code = gc2155_formats[code->index].mbus_code;
	return 0;
}
#endif

/*
 * Set a format.
 */
static int gc2155_set_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *format)
{
	if (format->pad) {
		return -EINVAL;
	}

	return 0;
}

static int gc2155_get_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *format)
{
	struct gc2155_info *info = to_state(sd);
	struct gc2155_win_size *wsize = info->win;
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

	//  printk("----%s, %d, width: %d, height: %d, code: %x\n",
	//          __func__, __LINE__, fmt->width, fmt->height, fmt->code);

	return ret;
}

static int gc2155_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc2155_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc2155_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc2155_s_vflip(struct v4l2_subdev *sd, int value)
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
static int gc2155_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int gc2155_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

#if 0
static unsigned int again_to_regval(int gain)
{
	unsigned int again = (gain * 20) >> LOG2_GAIN_SHIFT;
	// Limit Max gain
	if (again > AGAIN_MAX_DB + DGAIN_MAX_DB) {
		again = AGAIN_MAX_DB + DGAIN_MAX_DB;
	}
	return again;
}

static int regval_to_again(unsigned int regval)
{
	return (((int32_t)regval) << LOG2_GAIN_SHIFT) / 20;
}
#endif

static int gc2155_g_again(struct v4l2_subdev *sd, __s32 *value)
{
#if 0
	struct gc2155_info *info = to_state(sd);
	unsigned char v = 0;
	unsigned int reg_val = 0;
	int ret = 0;

	ret += gc2155_read(sd, 0x3014, &v);
	reg_val = v;

	*value = regval_to_again(reg_val);
	return ret;
#endif
	return 0;
}

/*set analog gain db value, map value to sensor register.*/
static int gc2155_s_again(struct v4l2_subdev *sd, int again)
{
#if 0
	struct gc2155_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;

	//  printk("again = %d\n", again);
	reg_value = again_to_regval(again);
	ret += gc2155_write(sd, 0x3014, (unsigned char)(reg_value & 0xff));
	return ret;
#endif
	return 0;
}

/*
 * Tweak autogain.
 */
static int gc2155_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int gc2155_s_exp(struct v4l2_subdev *sd, int value)
{
	int ret = 0;
#if 0
	struct gc2155_info *info = to_state(sd);
	int shs = 0;
	unsigned char val = 0;
	int total_height = 0;

	//  printk("exp = %d\n", value);
	ret = gc2155_read(sd, 0x3019, &val);
	total_height |= val << 8;
	ret = gc2155_read(sd, 0x3018, &val);
	total_height |= val;

	if (info->win->sensor_info.wdr_en) {
		total_height *= 2;
		shs = total_height - value - 1;
		ret = gc2155_write(sd, 0x3024, (unsigned char)(shs & 0xff));
		ret += gc2155_write(sd, 0x3025, (unsigned char)((shs >> 8) & 0xff));
		ret += gc2155_write(sd, 0x3026, (unsigned char)((shs >> 16) & 0x3));
	} else {
		shs = total_height - value - 1;
		ret = gc2155_write(sd, 0x3020, (unsigned char)(shs & 0xff));
		ret += gc2155_write(sd, 0x3021, (unsigned char)((shs >> 8) & 0xff));
		ret += gc2155_write(sd, 0x3022, (unsigned char)((shs >> 16) & 0x3));
	}
#endif
	return ret;
}

static int gc2155_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct gc2155_info *info = to_state(sd);

	switch (ctrl->id) {
		case V4L2_CID_AUTOGAIN:
			return gc2155_g_gain(sd, &info->gain->val);
		case V4L2_CID_ANALOGUE_GAIN:
			return gc2155_g_again(sd, &info->again->val);
	}
	return -EINVAL;
}

static int gc2155_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct gc2155_info *info = to_state(sd);

	switch (ctrl->id) {
		case V4L2_CID_BRIGHTNESS:
			return gc2155_s_brightness(sd, ctrl->val);
		case V4L2_CID_CONTRAST:
			return gc2155_s_contrast(sd, ctrl->val);
		case V4L2_CID_VFLIP:
			return gc2155_s_vflip(sd, ctrl->val);
		case V4L2_CID_HFLIP:
			return gc2155_s_hflip(sd, ctrl->val);
		case V4L2_CID_AUTOGAIN:
			/* Only set manual gain if auto gain is not explicitly
			   turned on. */
			if (!ctrl->val) {
				/* gc2155_s_gain turns off auto gain */
				return gc2155_s_gain(sd, info->gain->val);
			}
			return gc2155_s_autogain(sd, ctrl->val);
		case V4L2_CID_GAIN:
			return gc2155_s_gain(sd, ctrl->val);
		case V4L2_CID_ANALOGUE_GAIN:
			return gc2155_s_again(sd, ctrl->val);
		case V4L2_CID_EXPOSURE:
			return gc2155_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops gc2155_ctrl_ops = {
	.s_ctrl = gc2155_s_ctrl,
	.g_volatile_ctrl = gc2155_g_volatile_ctrl,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int gc2155_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = gc2155_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int gc2155_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	gc2155_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

static int gc2155_core_init(struct v4l2_subdev *sd, u32 val)
{
	struct gc2155_info *info = to_state(sd);
	int ret = 0;
	ret = gc2155_write_array(sd, info->win->regs);
	return ret;
}

static int gc2155_power(struct v4l2_subdev *sd, u32 val)
{
	struct gc2155_info *info = to_state(sd);
	if (val) {
		gpio_direction_output(info->pwdn.pin, info->reset.active_level);
		msleep(10);
	} else {
		gpio_direction_output(info->pwdn.pin, !info->reset.active_level);
		msleep(10);
	}
	return 0;
}

static int gc2155_core_reset(struct v4l2_subdev *sd, u32 val)
{
	struct gc2155_info *info = to_state(sd);

	if (val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(10);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(10);
	}
	return 0;
}

int gc2155_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;

	if (enable) {
		gc2155_core_reset(sd, 1);
		gc2155_core_init(sd, 1);
		ret = gc2155_write_array(sd, gc2155_stream_on_mipi);
		pr_debug("gc2155 stream on\n");

	} else {
		ret = gc2155_write_array(sd, gc2155_stream_off_mipi);
		pr_debug("gc2155 stream off\n");
	}
	return ret;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops gc2155_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = gc2155_g_register,
	.s_register = gc2155_s_register,
#endif
	.init = gc2155_core_init,
	.reset = gc2155_core_reset,

};

static const struct v4l2_subdev_video_ops gc2155_video_ops = {
	.s_stream = gc2155_s_stream,
};

static const struct v4l2_subdev_pad_ops gc2155_pad_ops = {
	//.enum_frame_interval = gc2155_enum_frame_interval,
	//.num_frame_size = gc2155_enum_frame_size,
	//.enum_mbus_code = gc2155_enum_mbus_code,
	.set_fmt = gc2155_set_fmt,
	.get_fmt = gc2155_get_fmt,
};

static const struct v4l2_subdev_ops gc2155_ops = {
	.core = &gc2155_core_ops,
	.video = &gc2155_video_ops,
	.pad = &gc2155_pad_ops,
};

/* ----------------------------------------------------------------------- */

static int gc2155_probe(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	struct gc2155_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	unsigned long rate;
	int mclk_index = -1;
	char id_div[9];
	char id_mux[9];

	*(volatile unsigned int *)(0xb0010100) = 1;

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		return -ENOMEM;
	}
	sd = &info->sd;

	gpio = of_get_named_gpio(client->dev.of_node, "ingenic,pwdn-gpio", 0);
	if (gpio_is_valid(gpio)) {
		info->pwdn.pin = gpio;
		info->pwdn.active_level = INGENIC_ISP_GPIO_ACTIVE_LOW;
	}

	gpio = of_get_named_gpio(client->dev.of_node, "ingenic,rst-gpio", 0);
	if (gpio_is_valid(gpio)) {
		info->reset.pin = gpio;
		info->reset.active_level = INGENIC_ISP_GPIO_ACTIVE_LOW;
	}

	v4l2_i2c_subdev_init(sd, client, &gc2155_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	/*clk*/
#if 1

	of_property_read_u32(client->dev.of_node, "ingenic,mclk", &mclk_index);
	if (mclk_index == 0) {
		memcpy(id_div, "div_cim0", sizeof(id_div));
		memcpy(id_mux, "mux_cim0", sizeof(id_mux));
	} else if (mclk_index == 1) {
		memcpy(id_div, "div_cim1", sizeof(id_div));
		memcpy(id_mux, "mux_cim1", sizeof(id_mux));
	} else if (mclk_index == 2) {
		memcpy(id_div, "div_cim2", sizeof(id_div));
		memcpy(id_mux, "mux_cim2", sizeof(id_mux));
	} else {
		printk("Unkonwn mclk index\n");
	}

	info->clk = devm_clk_get(&client->dev, id_div);
	if (IS_ERR(info->clk)) {
		ret = PTR_ERR(info->clk);
		goto err_clkget;
	}
	info->sclka = devm_clk_get(&client->dev, id_mux);

	rate = clk_get_rate(info->clk);
	if (((rate / 1000) % 24000) != 0) {
		ret = clk_set_parent(info->sclka, clk_get(NULL, "epll"));
		info->sclka = devm_clk_get(&client->dev, "epll");
		if (IS_ERR(info->sclka)) {
			pr_err("get sclka failed\n");
		} else {
			rate = clk_get_rate(info->sclka);
			if (((rate / 1000) % 24000) != 0) {
				clk_set_rate(info->sclka, 120000000);
			}
		}
	}

	ret = clk_set_rate(info->clk, 24000000);
	if (ret) {
		dev_err(sd->dev, "clk_set_rate err!\n");
	}

	ret = clk_prepare_enable(info->clk);
	if (ret) {
		dev_err(sd->dev, "clk_enable err!\n");
	}
#endif

	gc2155_power(sd, 1);
	gc2155_core_reset(sd, 1);
#if 1
	/* Make sure it's an gc2155 */
	ret = gc2155_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
		        "chip found @ 0x%x (%s) is not an gc2155 chip.\n",
		        client->addr << 1, client->adapter->name);
		return ret;
	}
#endif

	info->win = &gc2155_win_sizes[0];

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
	         client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 10);
	v4l2_ctrl_new_std(&info->hdl, &gc2155_ctrl_ops,
	                  V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &gc2155_ctrl_ops,
	                  V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &gc2155_ctrl_ops,
	                  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &gc2155_ctrl_ops,
	                  V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &gc2155_ctrl_ops,
	                               V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &gc2155_ctrl_ops,
	                                V4L2_CID_ANALOGUE_GAIN, 0, 589824, 1, 10000);
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &gc2155_ctrl_ops,
	                                   V4L2_CID_EXPOSURE, 1, 1176, 1, 1000);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

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

	dev_info(&client->dev, "gc2155 Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	devm_clk_put(&client->dev, info->clk);
err_clkget:
	return ret;
}

static void gc2155_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2155_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	devm_clk_put(&client->dev, info->clk);
}

static const struct i2c_device_id gc2155_id[] = {
	{ "gc2155", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, gc2155_id);

static const struct of_device_id gc2155_of_match[] = {
	{.compatible = "GalaxyCore,gc2155", },
	{},
};
MODULE_DEVICE_TABLE(of, gc2155_of_match);

static struct i2c_driver gc2155_driver = {
	.driver = {
		.name   = "gc2155",
		.of_match_table = of_match_ptr(gc2155_of_match),
	},
	.probe      = gc2155_probe,
	.remove     = gc2155_remove,
	.id_table   = gc2155_id,
};

module_i2c_driver(gc2155_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for OmniVision gc2155 sensors");
MODULE_LICENSE("GPL");
