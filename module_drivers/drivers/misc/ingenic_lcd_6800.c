/*
 * linux/drivers/misc/ingenic_lcd_6800.c - Ingenic lcd driver
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 * Author: <lichao.ren@ingenic.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/clk.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <soc/base.h>
#include <soc/gpio.h>
#include <ingenic_proc.h>
#include <linux/delay.h>

#define DRV_NAME    "ingenic-lcd"

#define LCD_DATA0   0
#define LCD_DATA1   1
#define LCD_DATA2   2
#define LCD_DATA3   3
#define LCD_DATA4   4
#define LCD_DATA5   5
#define LCD_DATA6   6
#define LCD_DATA7   7
#define LCD_VDD     8
#define LCD_RD      16
#define LCD_CS      23
#define LCD_DC      25
#define LCD_WR      26
#define LCD_RES     27

#define GPIOA_BASE  0xb0010000
#define PAT0S       0x44
#define PAT0C       0x48

#define RAM_PAGE_NUM    4
#define RAM_PAGE_SIZE   128

struct jz_lcd {
	struct device *dev;
	struct miscdevice mdev;
};

static struct gpio lcd_gpios[] = {
	/*申请GPIO时将数据线置为输入态，为了防止再未初始化上电序列时两边都为输出而烧屏*/
	{GPIO_PA(LCD_DATA0), GPIOF_IN, "lcd_date0" },
	{GPIO_PA(LCD_DATA1), GPIOF_IN, "lcd_date1" },
	{GPIO_PA(LCD_DATA2), GPIOF_IN, "lcd_date2" },
	{GPIO_PA(LCD_DATA3), GPIOF_IN, "lcd_date3" },
	{GPIO_PA(LCD_DATA4), GPIOF_IN, "lcd_date4" },
	{GPIO_PA(LCD_DATA5), GPIOF_IN, "lcd_date5" },
	{GPIO_PA(LCD_DATA6), GPIOF_IN, "lcd_date6" },
	{GPIO_PA(LCD_DATA7), GPIOF_IN, "lcd_date7" },
	/*将其他功能管脚置为输出态*/
	{GPIO_PA(LCD_VDD), GPIOF_OUT_INIT_LOW, "lcd_vdd" },
	{GPIO_PA(LCD_RD), GPIOF_OUT_INIT_HIGH, "lcd_rd" },
	{GPIO_PA(LCD_CS), GPIOF_OUT_INIT_HIGH, "lcd_cs" },
	{GPIO_PA(LCD_DC), GPIOF_OUT_INIT_LOW, "lcd_dc" },
	{GPIO_PA(LCD_WR), GPIOF_OUT_INIT_HIGH, "lcd_wr" },
	{GPIO_PA(LCD_RES), GPIOF_OUT_INIT_LOW, "lcd_rst" },
};

enum cmd_type {
	COMMAND = 0,
	DATA = 1
};

unsigned char instruct[] = {
	0xae, /* set  display off */
	0x04, /* set  lower column start address */
	0x10, /* set  higher column start address */
	0x40, /* set  display start line */
	0x81, /* set  contrast control */
	0x80,
	0xa1, /* set  segment remap  */
	0xa6, /* set  normal display */
	0xa8, /* set  multiplex ratio */
	0x1f, /* 1/32 */
	0xc8, /* set  com scan direction */
	0xd3, /* set  display offset  */
	0x00,
	0xd5, /* set  display clock divide/oscillator frequency */
	0xf0,
	0xD8, /*set area color mode off */
	0x05,
	0xD9, /* Set Pre-Charge Period */
	0xC2,
	0xda, /* set  com pin configuartion */
	0x12,
	0xdb, /* set  Vcomh */
	0x08,
	0xaf, /* set  display on */

};

static uint32_t inline lcd_readl(uint32_t reg_off)
{
	return readl(GPIOA_BASE + reg_off);
}

static void inline lcd_writel(uint32_t val, uint32_t reg_off)
{
	writel(val, GPIOA_BASE + reg_off);
}

static void writes(enum cmd_type type, unsigned char *buf, int size)
{
	int i;
	/*判断type 指令：下拉DC  数据：上拉DC*/
	if (type == DATA) {
		lcd_writel(1 << LCD_DC, PAT0S);
	} else {
		lcd_writel(1 << LCD_DC, PAT0C);
	}

	lcd_writel(1 << LCD_CS, PAT0C); //拉低cs，进行使能
	lcd_writel(1 << LCD_WR, PAT0S); //拉高设置为读状态，准备好数据后进行切换为写状态
	for (i = 0; i < size; i++) {
		lcd_writel(buf[i], PAT0S); //准备数据，将数据为1的写入
		lcd_writel(~buf[i] & 0xff, PAT0C); //准备数据，将数据为0的写入
		lcd_writel(1 << LCD_WR, PAT0C); //准备好数据后，将WR拉低进行写操作
		lcd_writel(1 << LCD_RD, PAT0S);
		lcd_writel(1 << LCD_RD, PAT0C); //数据锁存
		lcd_writel(1 << LCD_WR, PAT0S);
	}
	lcd_writel(1 << LCD_CS, PAT0S); //数据发送完成后，拉低cs，取消使能
}

static void instruct_config(void)
{
	int i;
	for (i = 0; i < sizeof(instruct); i++) {
		writes(COMMAND, instruct, sizeof(instruct));
	}
}

static void power_sequence_init(void)
{
	int i;
	/*初始化上电时序*/
	lcd_writel(1 << LCD_RES, PAT0S);
	lcd_writel(1 << LCD_RD, PAT0S);
	lcd_writel(1 << LCD_WR, PAT0S);
	lcd_writel(1 << LCD_CS, PAT0S);
	mdelay(10);
	lcd_writel(1 << LCD_RES, PAT0C);
	lcd_writel(1 << LCD_VDD, PAT0S); //上电使能
	mdelay(100);
	lcd_writel(1 << LCD_RES, PAT0S);
	mdelay(100);
	for (i = 0; i < 8; i++) {
		gpio_direction_output(GPIO_PA(i), 0);//初始化上电序列后将数据线置为输出态
	}
}

static ssize_t jz_lcd_write(struct file *file, const char *buf, size_t size, loff_t *pos)
{
	unsigned int y;
	/*cmd[0]: set page address
	 *cmd[1]: set lower column address
	 *cmd[2]: set higher column address
	 * */
	unsigned char cmd[3] = {0, 0x04, 0x10};
	for (y = 0; y < RAM_PAGE_NUM; y++) { //切换页
		cmd[0] = 0xb0 + y;
		writes(COMMAND, cmd, 3);
		writes(DATA, buf, RAM_PAGE_SIZE);
		buf += RAM_PAGE_SIZE;
	}
	return size;
}

static struct file_operations lcd_misc_fops = {
	.write      = jz_lcd_write,
};

static int jz_lcd_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	static struct jz_lcd *lcd;
	int ret = 0;

	lcd = devm_kzalloc(&pdev->dev, sizeof(struct jz_lcd), GFP_KERNEL);
	if (!lcd) {
		printk("lcd malloc failed!\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(res)) {
		dev_err(&pdev->dev, "get lcd resource failed!\n");
		ret = -ENXIO;
	}

	lcd->dev = &pdev->dev;
	lcd->mdev.minor = MISC_DYNAMIC_MINOR;
	lcd->mdev.name = DRV_NAME;
	lcd->mdev.fops = &lcd_misc_fops;

	ret = gpio_request_array(lcd_gpios, ARRAY_SIZE(lcd_gpios));
	if (ret) {
		dev_err(&pdev->dev, "request gpio failed!\n");
	}

	power_sequence_init();

	instruct_config();

	ret = misc_register(&lcd->mdev);
	if (ret < 0) {
		dev_err(lcd->dev, "lcd misc_register failed\n");
		ret = -EINVAL;
	}

	platform_set_drvdata(pdev, lcd);

	dev_info(lcd->dev, "lcd probe success.\n");
	return 0;
}

static const struct of_device_id lcd_of_match[] = {
	{ .compatible = "ingenic,x1600-dpu"},
	{},
};

static struct platform_driver jz_lcd_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(lcd_of_match),
	},
	.probe      = jz_lcd_probe,
};

static int __init jz_lcd_init(void)
{
	return platform_driver_register(&jz_lcd_driver);
}

static void __exit jz_lcd_exit(void)
{
	platform_driver_unregister(&jz_lcd_driver);
}

module_init(jz_lcd_init);
module_exit(jz_lcd_exit);

MODULE_DESCRIPTION("lcd driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("20230302");
