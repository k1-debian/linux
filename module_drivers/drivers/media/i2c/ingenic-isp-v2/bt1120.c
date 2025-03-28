/*
 * A V4L2 driver for OmniVision cameras.
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
#include <linux/module.h>
#include <linux/i2c.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <isp-sensor.h>

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct bt1120_win_size {
	unsigned int width;
	unsigned int height;
	unsigned int mbus_code;
	struct sensor_info sensor_info;
	enum v4l2_colorspace colorspace;
	void *regs;

};

struct bt1120_gpio {
	int pin;
	int active_level;
};

struct bt1120_info {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;

	struct media_pad pad;

	struct v4l2_subdev_format *format;  /*current fmt.*/
	struct bt1120_win_size *win;

	struct notifier_block nb;
	int init_flag;
};

/*
 * the part of driver maybe modify about different sensor and different board.
 */
static inline struct bt1120_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct bt1120_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct bt1120_info, hdl)->sd;
}

#if 0
/*
 * Stuff that knows about the sensor.
 */
static int bt1120_xshutdn(struct v4l2_subdev *sd, u32 val)
{
	struct bt1120_info *info = to_state(sd);

	return 0;
}

static int bt1120_pwdn(struct v4l2_subdev *sd, u32 val)
{
	struct bt1120_info *info = to_state(sd);

	return 0;
}

static int bt1120_detect(struct v4l2_subdev *sd, unsigned int *ident)
{
	unsigned char v;
	int ret;

	return 0;

}
#endif

static struct bt1120_win_size bt1120_win_sizes[] = {
	{
		.width              = 640,
		.height             = 480,
		.sensor_info.fps                = 30 << 16 | 1,
		                                     .sensor_info.total_width            = 640,
		                                     .sensor_info.total_height           = 640,
		                                     .sensor_info.wdr_en             = 0,
		                                     .sensor_info.bt_cfg.interlace_en        = 0,
		                                     .sensor_info.bt_cfg.bt_sav_eav          = SAV_BF_EAV,
		                                     .mbus_code  = MEDIA_BUS_FMT_YUYV8_2X8,
		                                     .colorspace = V4L2_COLORSPACE_SRGB,
	},
};

/*
 * Set a format.
 */
static int bt1120_set_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *format)
{
	if (format->pad) {
		return -EINVAL;
	}

	return 0;
}

static int bt1120_get_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *format)
{
	struct bt1120_info *info = to_state(sd);
	struct bt1120_win_size *wsize = info->win;
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
	*(unsigned int *)fmt->reserved = (unsigned int)&wsize->sensor_info; /*reserved[0] reserved[1]*/

	return ret;
}

static int bt1120_core_reset(struct v4l2_subdev *sd, u32 val)
{
	int ret = 0;

	return ret;
}

static int bt1120_core_init(struct v4l2_subdev *sd, u32 val)
{
	int ret = 0;

	return ret;
}

int bt1120_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;

	if (enable) {
		printk("bt1120 stream on\n");
	} else {
		printk("bt1120 stream off\n");
	}
	return ret;
}

int bt1120_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct bt1120_info *info = to_state(sd);
	if (info->win->sensor_info.fps) {
		param->parm.capture.timeperframe.numerator = info->win->sensor_info.fps & 0xffff;
		param->parm.capture.timeperframe.denominator = info->win->sensor_info.fps >> 16;
		return 0;
	}
	return -EINVAL;
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops bt1120_core_ops = {
	.init = bt1120_core_init,
	.reset = bt1120_core_reset,

};

static const struct v4l2_subdev_video_ops bt1120_video_ops = {
	.s_stream = bt1120_s_stream,
};

static const struct v4l2_subdev_pad_ops bt1120_pad_ops = {
	.set_fmt = bt1120_set_fmt,
	.get_fmt = bt1120_get_fmt,
};

static const struct v4l2_subdev_ops bt1120_ops = {
	.core = &bt1120_core_ops,
	.video = &bt1120_video_ops,
	.pad = &bt1120_pad_ops,
};

/* ----------------------------------------------------------------------- */

#ifdef CONFIG_X2500_RISCV
extern int register_riscv_notifier(struct notifier_block *nb);
int bt1120_riscv_notifier(struct notifier_block *nb, unsigned long action, void *data)
{
	struct bt1120_info *info = container_of(nb, struct sc230ai_info, nb);
	struct v4l2_subdev *sd = &info->sd;
	struct ispcam_device *ispcam = container_of(sd->v4l2_dev, struct ispcam_device, v4l2_dev);
	struct message *message = (unsigned int)action | 0x80000000;
	unsigned int *data_paddr;
	unsigned int *vaddr;

	while (message->type != MESSAGE_END) {
		if ((message->type == MESSAGE_INIT_SENSOR0 && ispcam->dev_nr == 0) ||
		    (message->type == MESSAGE_INIT_SENSOR1 && ispcam->dev_nr == 1) ||
		    (message->type == MESSAGE_INIT_SENSOR2 && ispcam->dev_nr == 2)) {
			data_paddr = message->data;
			vaddr = (unsigned int)data_paddr | 0x80000000;
			info->init_flag = *vaddr;
			break;
		}
		message++;
	}
	return 0;
}
#endif

static int bt1120_probe(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	struct bt1120_info *info;
	int ret;

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		return -ENOMEM;
	}
	sd = &info->sd;

	v4l2_i2c_subdev_init(sd, client, &bt1120_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	info->win = &bt1120_win_sizes[0];

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&info->sd.entity, 1, &info->pad);
	if (ret < 0) {
		goto err_entity_init;
	}
	ret = v4l2_async_register_subdev(&info->sd);
	if (ret < 0) {
		goto err_videoprobe;
	}

	dev_info(&client->dev, "bt1120 Probed\n");
	return 0;
err_videoprobe:
err_entity_init:
	return ret;
}

static void bt1120_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
}

static const struct i2c_device_id bt1120_id[] = {
	{ "bt1120", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, bt1120_id);

static const struct of_device_id bt1120_of_match[] = {
	{.compatible = "ovit,bt1120", },
	{},
};
MODULE_DEVICE_TABLE(of, bt1120_of_match);

static struct i2c_driver bt1120_driver = {
	.driver = {
		.name   = "bt1120",
		.of_match_table = of_match_ptr(bt1120_of_match),
	},
	.probe      = bt1120_probe,
	.remove     = bt1120_remove,
	.id_table   = bt1120_id,
};

module_i2c_driver(bt1120_driver);
MODULE_AUTHOR("qpz <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("A low-level driver for SmartSens bt1120 sensors");
MODULE_LICENSE("GPL");
