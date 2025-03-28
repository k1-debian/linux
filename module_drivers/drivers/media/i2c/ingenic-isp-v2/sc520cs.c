/*
 * A V4L2 driver for SmartSens sc520cs cameras.
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

#define sc520cs_CHIP_ID_H   (0xee)
#define sc520cs_CHIP_ID_L   (0x4b)
#define sc520cs_REG_END     0xffff
#define sc520cs_REG_DELAY   0xfffe

#define AGAIN_MAX_DB 0x64
#define DGAIN_MAX_DB 0x64
#define LOG2_GAIN_SHIFT 16

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

#define sc520cs_WDR_EN

struct sc520cs_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;
};

struct sc520cs_gpio {
	int pin;
	int active_level;
};

struct sc520cs_info {
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
	struct sc520cs_win_size *win;

	struct sc520cs_gpio reset;
	struct sc520cs_gpio pwdn;
};

void sc520cs_power_init_seq(struct sc520cs_info *info);
static inline struct sc520cs_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sc520cs_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct sc520cs_info, hdl)->sd;
}
struct regval_list {
	uint16_t reg_num;
	unsigned char value;
};

#if 1
/*
 * the part of driver maybe modify about different sensor and different board.
 */
struct again_lut {
	unsigned int value;
	unsigned int gain;
};

struct again_lut sc520cs_again_lut[] = {
	{0x340, 0},
	{0x342, 2886},
	{0x344, 5776},
	{0x346, 8494},
	{0x348, 11136},
	{0x34a, 13706},
	{0x34c, 16287},
	{0x34e, 18723},
	{0x350, 21097},
	{0x352, 23414},
	{0x354, 25746},
	{0x356, 27953},
	{0x358, 30109},
	{0x35a, 32217},
	{0x35c, 34345},
	{0x35e, 36361},
	{0x360, 38336},
	{0x362, 40270},
	{0x364, 42226},
	{0x366, 44082},
	{0x368, 45904},
	{0x36a, 47690},
	{0x36c, 49500},
	{0x36e, 51220},
	{0x370, 52910},
	{0x372, 54571},
	{0x374, 56254},
	{0x376, 57857},
	{0x378, 59433},
	{0x37a, 60984},
	{0x37c, 62558},
	{0x37e, 64059},
	{0x740, 65536},
	{0x742, 68468},
	{0x744, 71267},
	{0x746, 74030},
	{0x748, 76672},
	{0x74a, 79283},
	{0x74c, 81784},
	{0x74e, 84259},
	{0x750, 86633},
	{0x752, 88986},
	{0x754, 91246},
	{0x756, 93489},
	{0x2341, 96091},
	{0x2343, 98956},
	{0x2345, 101736},
	{0x2347, 104437},
	{0x2349, 107063},
	{0x234b, 109618},
	{0x234d, 112106},
	{0x234f, 114530},
	{0x2351, 116894},
	{0x2353, 119200},
	{0x2355, 121451},
	{0x2357, 123649},
	{0x2359, 125798},
	{0x235b, 127899},
	{0x235d, 129954},
	{0x235f, 131965},
	{0x2361, 133935},
	{0x2363, 135864},
	{0x2365, 137755},
	{0x2367, 139609},
	{0x2369, 141427},
	{0x236b, 143211},
	{0x236d, 144962},
	{0x236f, 146681},
	{0x2371, 148369},
	{0x2373, 150027},
	{0x2375, 151657},
	{0x2377, 153260},
	{0x2379, 154836},
	{0x237b, 156385},
	{0x237d, 157910},
	{0x237f, 159411},
	{0x2741, 161610},
	{0x2743, 164475},
	{0x2745, 167256},
	{0x2747, 169958},
	{0x2749, 172584},
	{0x274b, 175140},
	{0x274d, 177628},
	{0x274f, 180052},
	{0x2751, 182416},
	{0x2753, 184722},
	{0x2755, 186974},
	{0x2757, 189172},
	{0x2759, 191321},
	{0x275b, 193423},
	{0x275d, 195478},
	{0x275f, 197490},
	{0x2761, 199460},
	{0x2763, 201389},
	{0x2765, 203280},
	{0x2767, 205134},
	{0x2769, 206953},
	{0x276b, 208736},
	{0x276d, 210487},
	{0x276f, 212207},
	{0x2771, 213895},
	{0x2773, 215554},
	{0x2775, 217184},
	{0x2777, 218786},
	{0x2779, 220362},
	{0x277b, 221912},
	{0x277d, 223437},
	{0x277f, 224938},
	{0x2f41, 227146},
	{0x2f43, 230011},
	{0x2f45, 232792},
	{0x2f47, 235494},
	{0x2f49, 238120},
	{0x2f4b, 240676},
	{0x2f4d, 243164},
	{0x2f4f, 245588},
	{0x2f51, 247952},
	{0x2f53, 250258},
	{0x2f55, 252510},
	{0x2f57, 254708},
	{0x2f59, 256857},
	{0x2f5b, 258959},
	{0x2f5d, 261014},
	{0x2f5f, 263026},
	{0x2f61, 264996},
	{0x2f63, 266925},
	{0x2f65, 268816},
	{0x2f67, 270670},
	{0x2f69, 272489},
	{0x2f6b, 274273},
	{0x2f6d, 276023},
	{0x2f6f, 277743},
	{0x2f71, 279431},
	{0x2f73, 281090},
	{0x2f75, 282720},
	{0x2f77, 284322},
	{0x2f79, 285898},
	{0x2f7b, 287448},
	{0x2f7d, 288973},
	{0x2f7f, 290474},
	{0x3f41, 292682},
	{0x3f43, 295547},
	{0x3f45, 298328},
	{0x3f47, 301030},
	{0x3f49, 303656},
	{0x3f4b, 306212},
	{0x3f4d, 308700},
	{0x3f4f, 311124},
	{0x3f51, 313488},
	{0x3f53, 315794},
	{0x3f55, 318046},
	{0x3f57, 320244},
	{0x3f59, 322393},
	{0x3f5b, 324495},
	{0x3f5d, 326550},
	{0x3f5f, 328562},
	{0x3f61, 330532},
	{0x3f63, 332461},
	{0x3f65, 334352},
	{0x3f67, 336206},
	{0x3f69, 338025},
	{0x3f6b, 339809},
	{0x3f6d, 341559},
	{0x3f6f, 343279},
	{0x3f71, 344967},
	{0x3f73, 346626},
	{0x3f75, 348256},
	{0x3f77, 349858},
	{0x3f79, 351434},
	{0x3f7b, 352984},
	{0x3f7d, 354509},
	{0x3f7f, 356010},
};
#endif

static struct regval_list sc520cs_init_regs_1280_720_30fps_mipi[] = {
	{0x0103, 0x01},
	{0x36e9, 0x80},
	{0x37f9, 0x80},
	{0x301f, 0x17},
	{0x3200, 0x00},
	{0x3201, 0x00},
	{0x3202, 0x00},
	{0x3203, 0xfc},
	{0x3204, 0x0a},
	{0x3205, 0x27},
	{0x3206, 0x06},
	{0x3207, 0xab},
	{0x3208, 0x05},
	{0x3209, 0x00},
	{0x320a, 0x02},
	{0x320b, 0xd0},
	{0x3210, 0x00},
	{0x3211, 0x0a},
	{0x3212, 0x00},
	{0x3213, 0x04},
	{0x3215, 0x31},
	{0x3220, 0x81},
	{0x3301, 0x08},
	{0x3306, 0x60},
	{0x3308, 0x10},
	{0x3309, 0x80},
	{0x330b, 0xd0},
	{0x3314, 0x15},
	{0x331f, 0x71},
	{0x3333, 0x10},
	{0x3334, 0x40},
	{0x3364, 0x56},
	{0x336c, 0xcf},
	{0x3390, 0x09},
	{0x3391, 0x0f},
	{0x3392, 0x1f},
	{0x3393, 0x18},
	{0x3394, 0x28},
	{0x3395, 0x28},
	{0x33ad, 0x1c},
	{0x33af, 0x3f},
	{0x33b3, 0x30},
	{0x349f, 0x1e},
	{0x34a6, 0x09},
	{0x34a7, 0x0b},
	{0x34a8, 0x20},
	{0x34a9, 0x20},
	{0x34f8, 0x0f},
	{0x34f9, 0x10},
	{0x3633, 0x34},
	{0x3637, 0x52},
	{0x3638, 0xc1},
	{0x363a, 0x89},
	{0x3670, 0x48},
	{0x3690, 0x43},
	{0x3691, 0x43},
	{0x3692, 0x43},
	{0x3698, 0x89},
	{0x3699, 0x92},
	{0x369a, 0xa5},
	{0x369b, 0xca},
	{0x369c, 0x0b},
	{0x369d, 0x0f},
	{0x36a2, 0x09},
	{0x36a3, 0x0b},
	{0x36a4, 0x0f},
	{0x36b1, 0x38},
	{0x36b2, 0xc1},
	{0x3724, 0xb1},
	{0x3901, 0x04},
	{0x3905, 0x8c},
	{0x391d, 0x04},
	{0x3926, 0x20},
	{0x3e00, 0x00},
	{0x3e01, 0xf9},
	{0x3e02, 0x70},
	{0x3e09, 0x00},
	{0x4401, 0x13},
	{0x4402, 0x02},
	{0x4403, 0x09},
	{0x4404, 0x1b},
	{0x4405, 0x24},
	{0x4407, 0x0c},
	{0x440c, 0x2d},
	{0x440d, 0x2d},
	{0x440e, 0x22},
	{0x440f, 0x39},
	{0x4412, 0x01},
	{0x4424, 0x01},
	{0x442d, 0x00},
	{0x442e, 0x00},
	{0x4509, 0x28},
	{0x450d, 0x19},
	{0x451d, 0x28},
	{0x4526, 0x05},
	{0x5000, 0x4e},
	{0x5007, 0x00},
	{0x550e, 0x00},
	{0x550f, 0x42},
	{0x5780, 0x66},
	{0x5787, 0x08},
	{0x5788, 0x04},
	{0x5789, 0x01},
	{0x578d, 0x40},
	{0x5790, 0x08},
	{0x5791, 0x04},
	{0x5792, 0x01},
	{0x5799, 0x46},
	{0x57aa, 0xeb},
	{0x5900, 0xf1},
	{0x5901, 0x04},
	{0x59f0, 0x00},
	{0x5ae0, 0xfe},
	{0x5ae1, 0x40},
	{0x5ae2, 0x38},
	{0x5ae3, 0x30},
	{0x5ae4, 0x0c},
	{0x5ae5, 0x38},
	{0x5ae6, 0x30},
	{0x5ae7, 0x28},
	{0x5ae8, 0x3f},
	{0x5ae9, 0x34},
	{0x5aea, 0x2c},
	{0x5aeb, 0x3f},
	{0x5aec, 0x34},
	{0x5aed, 0x2c},
	{0x36e9, 0x23},
	{0x37f9, 0x23},
	{0x0100, 0x01},

	//  {0x4501, 0xcc},
	//  {0x3902, 0x80},
	//  {0x3e07, 0x40},

	{sc520cs_REG_DELAY, 10},
	{sc520cs_REG_END, 0x00},    /* END MARKER */
};

/*
 * the part of driver was fixed.
 */
static struct regval_list sc520cs_stream_on_mipi[] = {
	{0x0100, 0x01},
	{sc520cs_REG_END, 0x00},    /* END MARKER */
};

static struct regval_list sc520cs_stream_off_mipi[] = {
	{0x0100, 0x00},
	{sc520cs_REG_END, 0x00},    /* END MARKER */
};

static int sc520cs_read(struct v4l2_subdev *sd, unsigned short reg,
                        unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t buf[2] = {(reg >> 8) & 0xff, reg & 0xff};
	struct i2c_msg msg[2] = {
		[0] = {
			.addr   = client->addr,
			.flags  = 0,
			.len    = 2,
			.buf    = buf,
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

static int sc520cs_write(struct v4l2_subdev *sd, unsigned short reg,
                         unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	uint8_t buf[3] = {(reg >> 8) & 0xff, reg & 0xff, value};
	struct i2c_msg msg = {
		.addr   = client->addr,
		.flags  = 0,
		.len    = 3,
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
static int sc520cs_read_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	unsigned char val;
	while (vals->reg_num != sc520cs_REG_END) {
		if (vals->reg_num == sc520cs_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = sc520cs_read(sd, vals->reg_num, &val);
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
static int sc520cs_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	int ret;
	while (vals->reg_num != sc520cs_REG_END) {
		if (vals->reg_num == sc520cs_REG_DELAY) {
			msleep(vals->value);
		} else {
			ret = sc520cs_write(sd, vals->reg_num, vals->value);
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
static int sc520cs_reset(struct v4l2_subdev *sd, u32 val)
{
	struct sc520cs_info *info = to_state(sd);

	if (val) {
		gpio_direction_output(info->reset.pin, info->reset.active_level);
		msleep(10);
		gpio_direction_output(info->reset.pin, !info->reset.active_level);
		msleep(10);
	}
	return 0;
}
#endif

static int sc520cs_init(struct v4l2_subdev *sd, u32 val)
{
	struct sc520cs_info *info = to_state(sd);
	int ret = 0;

	ret = sc520cs_write_array(sd, info->win->regs);

	return ret;
}

static int sc520cs_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;
	ret = sc520cs_read(sd, 0x3107, &v);
	pr_info("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret, v);
	if (ret < 0) {
		return ret;
	}
	if (v != sc520cs_CHIP_ID_H) {
		return -ENODEV;
	}
	*ident = v;

	ret = sc520cs_read(sd, 0x3108, &v);
	pr_info("-----%s: %d ret = %d, v = 0x%02x\n", __func__, __LINE__, ret, v);
	if (ret < 0) {
		return ret;
	}
	if (v != sc520cs_CHIP_ID_L) {
		return -ENODEV;
	}
	*ident = (*ident << 8) | v;

	return 0;
}

static struct sc520cs_win_size sc520cs_win_sizes[] = {
	{
		.width                  = 1280,
		.height                 = 720,
		.sensor_info.fps            = 30 << 16 | 1,
		                                 .sensor_info.wdr_en         = 0,
		                                 .sensor_info.mipi_cfg.clk       = 450,
		                                 .sensor_info.mipi_cfg.twidth        = 1280,
		                                 .sensor_info.mipi_cfg.theight       = 720,
		                                 .sensor_info.mipi_cfg.mipi_mode     = SENSOR_MIPI_OTHER_MODE,
		                                 .sensor_info.mipi_cfg.mipi_vcomp_en = 0,
		                                 .sensor_info.mipi_cfg.mipi_hcomp_en = 0,
		                                 .sensor_info.mipi_cfg.mipi_crop_start0x = 0,//10,
		                                 .sensor_info.mipi_cfg.mipi_crop_start0y = 0,//4,
		                                 .sensor_info.mipi_cfg.mipi_crop_start1x = 0,
		                                 .sensor_info.mipi_cfg.mipi_crop_start1y = 0,
		                                 .sensor_info.mipi_cfg.mipi_crop_start2x = 0,
		                                 .sensor_info.mipi_cfg.mipi_crop_start2y = 0,
		                                 .sensor_info.mipi_cfg.mipi_crop_start3x = 0,
		                                 .sensor_info.mipi_cfg.mipi_crop_start3y = 0,
		                                 .sensor_info.mipi_cfg.hcrop_diff_en = 0,
		                                 .sensor_info.mipi_cfg.line_sync_mode    = 0,
		                                 .sensor_info.mipi_cfg.work_start_flag   = 0,
		                                 .sensor_info.mipi_cfg.data_type_en  = 0,
		                                 .sensor_info.mipi_cfg.data_type_value   = RAW10,
		                                 .sensor_info.mipi_cfg.del_start     = 0,
		                                 .sensor_info.mipi_cfg.sensor_frame_mode = TX_SENSOR_DEFAULT_FRAME_MODE,
		                                 .sensor_info.mipi_cfg.sensor_fid_mode   = 0,
		                                 .sensor_info.mipi_cfg.sensor_mode   = TX_SENSOR_DEFAULT_MODE,
		                                 .sensor_info.mipi_cfg.sensor_csi_fmt    = TX_SENSOR_RAW10,
		                                 .mbus_code              = MEDIA_BUS_FMT_SBGGR10_1X10,
		                                 .colorspace             = V4L2_COLORSPACE_SRGB,
		                                 .regs                   = sc520cs_init_regs_1280_720_30fps_mipi,
	},//[0]
};


/*
 * Set a format.
 */
static int sc520cs_set_fmt(struct v4l2_subdev *sd,
                           struct v4l2_subdev_state *state,
                           struct v4l2_subdev_format *format)
{
	if (format->pad) {
		return -EINVAL;
	}

	return 0;
}

static int sc520cs_get_fmt(struct v4l2_subdev *sd,
                           struct v4l2_subdev_state *state,
                           struct v4l2_subdev_format *format)
{
	struct sc520cs_info *info = to_state(sd);
	struct sc520cs_win_size *wsize = info->win;
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

#if 1
static int sc520cs_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc520cs_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc520cs_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc520cs_s_vflip(struct v4l2_subdev *sd, int value)
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
static int sc520cs_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;
	unsigned char gain = 0;

	*value = gain;
	return ret;
}

static int sc520cs_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static unsigned int again_to_regval(int gain)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(sc520cs_again_lut); i++) {
		lut = &sc520cs_again_lut[i];

		if (gain <= lut->gain) {
			return lut->value;
		}
	}

	/*last value.*/
	return lut->value;
}

#if 0
static int regval_to_again(unsigned int regval)
{
	struct again_lut *lut = NULL;
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(sc520cs_again_lut); i++) {
		lut = &sc520cs_again_lut[i];

		if (regval == lut->value) {
			return lut->gain;
		}
	}
	printk("%s, %d regval not mapped to isp gain\n", __func__, __LINE__);
	return -EINVAL;
}
#endif

static int sc520cs_g_again(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;

	return ret;
}

static int sc520cs_g_again_short(struct v4l2_subdev *sd, __s32 *value)
{
	int ret = 0;

	return ret;
}

/*set analog gain db value, map value to sensor register.*/
static int sc520cs_s_again(struct v4l2_subdev *sd, int again)
{
	struct sc520cs_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;
	if (again < info->again->minimum || again > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(again);
	}

	ret = sc520cs_write(sd, 0x3e09, (unsigned char)(reg_value & 0xff));
	ret += sc520cs_write(sd, 0x3e08, (unsigned char)(reg_value >> 8 & 0x3f));

	return ret;
}

static int sc520cs_s_again_short(struct v4l2_subdev *sd, int again)
{
	struct sc520cs_info *info = to_state(sd);
	unsigned int reg_value = 0;
	int ret = 0;
	if (again < info->again->minimum || again > info->again->maximum) {
		/* use default value. */
		reg_value = again_to_regval(info->again->default_value);
	} else {
		reg_value = again_to_regval(again);
	}

	ret = sc520cs_write(sd, 0x3e13, (unsigned char)(reg_value & 0xff));
	ret += sc520cs_write(sd, 0x3e12, (unsigned char)(reg_value >> 8 & 0x3f));

	return ret;
}

/*
 * Tweak autogain.
 */
static int sc520cs_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret = 0;

	return ret;
}

static int sc520cs_s_exp(struct v4l2_subdev *sd, int value)
{
	int ret = 0;
	unsigned int expo = 0;
#ifdef sc520cs_WDR_EN
	expo = value * 4;
	ret += sc520cs_write(sd, 0x3e00, (unsigned char)((expo >> 12) & 0x0f));
	ret += sc520cs_write(sd, 0x3e01, (unsigned char)((expo >> 4) & 0xff));
	ret += sc520cs_write(sd, 0x3e02, (unsigned char)((expo & 0x0f) << 4));
	if (value < 0x50) {
		ret += sc520cs_write(sd, 0x3812, 0x00);
		ret += sc520cs_write(sd, 0x3314, 0x14);
		ret += sc520cs_write(sd, 0x3812, 0x30);
	} else if (value > 0xa0) {
		ret += sc520cs_write(sd, 0x3812, 0x00);
		ret += sc520cs_write(sd, 0x3314, 0x04);
		ret += sc520cs_write(sd, 0x3812, 0x30);
	}
#else
	expo = value * 2;
	ret += sc520cs_write(sd, 0x3e00, (unsigned char)((expo >> 12) & 0x0f));
	ret += sc520cs_write(sd, 0x3e01, (unsigned char)((expo >> 4) & 0xff));
	ret += sc520cs_write(sd, 0x3e02, (unsigned char)((expo & 0x0f) << 4));
	if (value < 0x50) {
		ret += sc520cs_write(sd, 0x3812, 0x00);
		ret += sc520cs_write(sd, 0x3314, 0x14);
		ret += sc520cs_write(sd, 0x3812, 0x30);
	} else if (value > 0xa0) {
		ret += sc520cs_write(sd, 0x3812, 0x00);
		ret += sc520cs_write(sd, 0x3314, 0x04);
		ret += sc520cs_write(sd, 0x3812, 0x30);
	}
#endif

	return ret;
}

static int sc520cs_s_exp_short(struct v4l2_subdev *sd, int value)
{
	int ret = 0;
	unsigned int expo = 0;
	ret += sc520cs_write(sd, 0x3e04, (unsigned char)((expo >> 4) & 0xff));
	ret += sc520cs_write(sd, 0x3e05, (unsigned char)((expo & 0x0f) << 4));
	if (value < 0x50) {
		ret += sc520cs_write(sd, 0x3812, 0x00);
		ret += sc520cs_write(sd, 0x3314, 0x14);
		ret += sc520cs_write(sd, 0x3812, 0x30);
	} else if (value > 0xa0) {
		ret += sc520cs_write(sd, 0x3812, 0x00);
		ret += sc520cs_write(sd, 0x3314, 0x04);
		ret += sc520cs_write(sd, 0x3812, 0x30);
	}

	return ret;
}

static int sc520cs_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct sc520cs_info *info = to_state(sd);

	switch (ctrl->id) {
		case V4L2_CID_AUTOGAIN:
			return sc520cs_g_gain(sd, &info->gain->val);
		case V4L2_CID_ANALOGUE_GAIN:
			return sc520cs_g_again(sd, &info->again->val);
		case V4L2_CID_USER_ANALOG_GAIN_SHORT:
			return sc520cs_g_again_short(sd, &info->again_short->val);
	}
	return -EINVAL;
}

static int sc520cs_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct sc520cs_info *info = to_state(sd);

	switch (ctrl->id) {
		case V4L2_CID_BRIGHTNESS:
			return sc520cs_s_brightness(sd, ctrl->val);
		case V4L2_CID_CONTRAST:
			return sc520cs_s_contrast(sd, ctrl->val);
		case V4L2_CID_VFLIP:
			return sc520cs_s_vflip(sd, ctrl->val);
		case V4L2_CID_HFLIP:
			return sc520cs_s_hflip(sd, ctrl->val);
		case V4L2_CID_AUTOGAIN:
			/* Only set manual gain if auto gain is not explicitly
			   turned on. */
			if (!ctrl->val) {
				/* sc520cs_s_gain turns off auto gain */
				return sc520cs_s_gain(sd, info->gain->val);
			}
			return sc520cs_s_autogain(sd, ctrl->val);
		case V4L2_CID_GAIN:
			return sc520cs_s_gain(sd, ctrl->val);
		case V4L2_CID_ANALOGUE_GAIN:
			return sc520cs_s_again(sd, ctrl->val);
		case V4L2_CID_USER_ANALOG_GAIN_SHORT:
			return sc520cs_s_again_short(sd, ctrl->val);
		case V4L2_CID_EXPOSURE:
			return sc520cs_s_exp(sd, ctrl->val);
		case V4L2_CID_USER_EXPOSURE_SHORT:
			return sc520cs_s_exp_short(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops sc520cs_ctrl_ops = {
	.s_ctrl = sc520cs_s_ctrl,
	.g_volatile_ctrl = sc520cs_g_volatile_ctrl,
};
#endif

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int sc520cs_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	unsigned char val = 0;
	int ret;

	ret = sc520cs_read(sd, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 2;
	return ret;
}

static int sc520cs_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	sc520cs_write(sd, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

int sc520cs_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sc520cs_info *info = to_state(sd);
	int ret = 0;

	if (enable) {
		sc520cs_power_init_seq(info);
		sc520cs_init(sd, 1);
		ret = sc520cs_write_array(sd, sc520cs_stream_on_mipi);
		printk("sc520cs stream on\n");

	} else {
		ret = sc520cs_write_array(sd, sc520cs_stream_off_mipi);
		printk("sc520cs stream off\n");
	}
	return ret;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sc520cs_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = sc520cs_g_register,
	.s_register = sc520cs_s_register,
#endif

};

static const struct v4l2_subdev_video_ops sc520cs_video_ops = {
	.s_stream = sc520cs_s_stream,
};

static const struct v4l2_subdev_pad_ops sc520cs_pad_ops = {
	//.enum_frame_interval = sc520cs_enum_frame_interval,
	//.num_frame_size = sc520cs_enum_frame_size,
	//.enum_mbus_code = sc520cs_enum_mbus_code,
	.set_fmt = sc520cs_set_fmt,
	.get_fmt = sc520cs_get_fmt,
};

static const struct v4l2_subdev_ops sc520cs_ops = {
	.core = &sc520cs_core_ops,
	.video = &sc520cs_video_ops,
	.pad = &sc520cs_pad_ops,
};

void sc520cs_power_init_seq(struct sc520cs_info *info)
{
	*(volatile unsigned int *)0xb0010000 = 0x1; //set gpio power domain 1.8v

	gpio_direction_output(info->pwdn.pin, !info->pwdn.active_level);
	msleep(10);
	gpio_direction_output(info->pwdn.pin, info->pwdn.active_level);
	msleep(10);
	gpio_direction_output(info->pwdn.pin, !info->pwdn.active_level);
	msleep(10);
}
/* ----------------------------------------------------------------------- */

static int sc520cs_probe(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	struct sc520cs_info *info;
	int ret;
	unsigned int ident = 0;
	int gpio = -EINVAL;
	struct v4l2_ctrl_config cfg = {0};
	int mclk_index = -1;
	char id_div[9];

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		return -ENOMEM;
	}
	sd = &info->sd;

	gpio = of_get_named_gpio(client->dev.of_node, "ingenic,rst-gpio", 0);
	if (gpio_is_valid(gpio)) {
		info->reset.pin = gpio;
		info->reset.active_level = INGENIC_ISP_GPIO_ACTIVE_HIGH;
	}

	gpio = of_get_named_gpio(client->dev.of_node, "ingenic,pwdn-gpio", 0);
	if (gpio_is_valid(gpio)) {
		info->pwdn.pin = gpio;
		info->pwdn.active_level = INGENIC_ISP_GPIO_ACTIVE_HIGH;
	}

	sc520cs_power_init_seq(info);
	v4l2_i2c_subdev_init(sd, client, &sc520cs_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	/*clk*/
	of_property_read_u32(client->dev.of_node, "ingenic,mclk", &mclk_index);
	if (mclk_index == 0) {
		memcpy(id_div, "div_cim0", sizeof(id_div));
	} else if (mclk_index == 1) {
		memcpy(id_div, "div_cim1", sizeof(id_div));
	} else if (mclk_index == 2) {
		memcpy(id_div, "div_cim2", sizeof(id_div));
	} else {
		printk("Unkonwn mclk index\n");
	}

	info->clk = devm_clk_get(&client->dev, id_div);
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

	//sc520cs_reset(sd, 1);
#if 1
	/* Make sure it's an sc520cs */
	ret = sc520cs_detect(sd, &ident);
	if (ret) {
		v4l_err(client,
		        "chip found @ 0x%x (%s) is not an sc520cs chip.\n",
		        client->addr << 1, client->adapter->name);
		return ret;
	}
#endif

	/*IRCUT ctl 0:off 1:on*/
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
	         client->addr << 1, client->adapter->name);

	v4l2_ctrl_handler_init(&info->hdl, 8);
	v4l2_ctrl_new_std(&info->hdl, &sc520cs_ctrl_ops,
	                  V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&info->hdl, &sc520cs_ctrl_ops,
	                  V4L2_CID_CONTRAST, 0, 127, 1, 64);
	v4l2_ctrl_new_std(&info->hdl, &sc520cs_ctrl_ops,
	                  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->hdl, &sc520cs_ctrl_ops,
	                  V4L2_CID_HFLIP, 0, 1, 1, 0);
	info->gain = v4l2_ctrl_new_std(&info->hdl, &sc520cs_ctrl_ops,
	                               V4L2_CID_GAIN, 0, 255, 1, 128);
	info->again = v4l2_ctrl_new_std(&info->hdl, &sc520cs_ctrl_ops,
	                                V4L2_CID_ANALOGUE_GAIN, 0, 356010, 1, 10000);
	info->exposure = v4l2_ctrl_new_std(&info->hdl, &sc520cs_ctrl_ops,
	                                   V4L2_CID_EXPOSURE, 2, 0x5a0 - 3, 1, 1000);

	cfg.ops = &sc520cs_ctrl_ops;
	cfg.id = V4L2_CID_USER_EXPOSURE_SHORT;
	cfg.name = "expo short";
	cfg.type = V4L2_CTRL_TYPE_INTEGER;
	cfg.min = 5;
	cfg.max = 0x5a0 - 3;
	cfg.step = 1;
	cfg.def = 1000;
	info->exposure_short = v4l2_ctrl_new_custom(&info->hdl, &cfg, NULL);

	memset(&cfg, 0, sizeof(cfg));
	cfg.ops = &sc520cs_ctrl_ops;
	cfg.id = V4L2_CID_USER_ANALOG_GAIN_SHORT;
	cfg.name = "analog gain short";
	cfg.type = V4L2_CTRL_TYPE_INTEGER;
	cfg.min = 0;
	cfg.max = 356010;
	cfg.step = 1;
	cfg.def = 10000;
	info->again_short = v4l2_ctrl_new_custom(&info->hdl, &cfg, NULL);

	sd->ctrl_handler = &info->hdl;
	if (info->hdl.error) {
		int err = info->hdl.error;

		v4l2_ctrl_handler_free(&info->hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&info->hdl);

	info->win = &sc520cs_win_sizes[0];

	sc520cs_init(sd, 1);
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

	dev_info(&client->dev, "sc520cs Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	devm_clk_put(&client->dev, info->clk);
err_clkget:
	return ret;
}

static void sc520cs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc520cs_info *info = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->hdl);
	devm_clk_put(&client->dev, info->clk);
}

static const struct i2c_device_id sc520cs_id[] = {
	{ "sc520cs", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc520cs_id);

static const struct of_device_id sc520cs_of_match[] = {
	{.compatible = "smartsens,sc520cs", },
	{},
};
MODULE_DEVICE_TABLE(of, sc520cs_of_match);

static struct i2c_driver sc520cs_driver = {
	.driver = {
		.name   = "sc520cs",
		.of_match_table = of_match_ptr(sc520cs_of_match),
	},
	.probe      = sc520cs_probe,
	.remove     = sc520cs_remove,
	.id_table   = sc520cs_id,
};

module_i2c_driver(sc520cs_driver);
MODULE_DESCRIPTION("A low-level driver for SmartSens sc520cs sensors");
MODULE_LICENSE("GPL");
