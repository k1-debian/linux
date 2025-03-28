/* linux/drivers/char/ingenic_pwm_smc.c
 *
 * Ingenic pwm_smc driver, use kernel char device framework
 *
 * Copyright (c) 2023 Ingenic
 * Author:renlichao <lichao.ren@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/pwm.h>
#include <linux/wait.h>
#include <linux/semaphore.h>
#include <linux/dma-mapping.h>
#include <linux/circ_buf.h>

#include <asm/div64.h>

#include <pwm-ingenic-v3.h>
#include <ingenic_dma.h>

#define PWM_BASE 0x13610000

struct pwm_form {
	uint16_t Low;
	uint16_t High;
};

struct pwm_cfg_info {
	int mode;       /*pwm的模式，当前配置为DMA_MODE_SMC*/
	int init_level;     /*init电平*/
	int finish_level;   /*idel电平*/
	int period_ns;      /*周期时间*/
	int duty_ns;        /*用于配置占空比*/
	int channel;        /*使用的pwm通道*/
	int clk_in;     /*pwm时钟*/
	int step;       /*匀速的步数*/
	int set_up_num;     /*加速的步数*/
	int cur_pos;        /*当前步数*/
};

struct ingenic_pwm_smc {
	int major;
	int minor;
	dev_t dev_num;
	int nr_devs;
	struct class *class;
	struct cdev cdev;
	struct device *dev;
	struct platform_device *pdev;
	struct mutex mutex;
	struct dma_channel *ch;         /*DMA结构参数*/
	struct pwm_cfg_info pwm_cfg_info;   /*PWM基础参数*/
	struct pwm_device *pwm;         /*PWM通道对象*/
	struct ingenic_pwm_chan *chan;      /*PWM驱动结构*/
	struct pwm_form *rise_buf;      /*存放加速表数据*/
	struct pwm_form even_buf;       /*匀速数据*/
	struct pwm_form *fall_buf;      /*存放减速表数据*/
	int open_ed;                /*设备是否被打开*/
};

enum pwm_smc_ioctl_cmd {
	PWM_SMC_GET_INFO,           /*获取结构体数据*/
	PWM_SMC_SET_INFO,           /*设置结构体数据*/
	PWM_SMC_CONFIG,             /*初始化SMC*/
	PWM_SMC_START,              /*启动SMC*/
	PWM_SMC_GEN_STOP,           /*缓停，返回当前步数*/
	PWM_SMC_QCK_STOP,           /*急停，返回当前步数*/
	PWM_SMC_GET_RUN_STEP_INFO,      /*获取当前步数*/
	PWM_SMC_REQUEST_CHANNEL,        /*申请pwm通道*/
	PWM_SMC_DISABLE,            /*释放pwm通道*/
};

struct dma_channel {
	struct hdma_desc *rise_desc;
	struct hdma_desc *even_desc;
	struct hdma_desc *fall_desc;
	struct dma_chan *chan;
	int channel;
};

__attribute__((__unused__)) static void dump_desc(char *title, struct hdma_desc *desc)
{
	printk("=========== %s =========== \n", title);
	printk("dcm:0x%08lx\n", desc->dcm);
	printk("dsa:0x%08x\n", (unsigned int)desc->dsa);
	printk("dta:0x%08x\n", (unsigned int)desc->dta);
	printk("dtc:0x%08lx\n", desc->dtc);
	printk("drt:0x%08lx\n", desc->drt);
}

static inline unsigned long pwm_smc_read(int offset)
{
	return readl((void *)CKSEG1ADDR(PWM_BASE + offset));
}

static inline void pwm_smc_write(int offset, unsigned long val)
{
	writel(val, (void *)CKSEG1ADDR(PWM_BASE + offset));
}

static struct hdma_desc _gdesc[3] __attribute__((aligned(128)));

static void build_desc(struct ingenic_pwm_smc *pwm_smc, struct dma_channel *ch)
{
	int req_type = 0;
	switch (ch->channel) {
		case 0:
			req_type = 0x2c;
			break;
		case 1:
			req_type = 0x2d;
			break;
		case 2:
			req_type = 0x2e;
			break;
		case 3:
			req_type = 0x2f;
			break;
		case 4:
			req_type = 0x30;
			break;
		case 5:
			req_type = 0x31;
			break;
		case 6:
			req_type = 0x32;
			break;
		case 7:
			req_type = 0x33;
			break;
		case 8:
			req_type = 0x34;
			break;
		case 9:
			req_type = 0x35;
			break;
		case 10:
			req_type = 0x36;
			break;
		case 11:
			req_type = 0x37;
			break;
		case 12:
			req_type = 0x38;
			break;
		case 13:
			req_type = 0x39;
			break;
		case 14:
			req_type = 0x3a;
			break;
		case 15:
			req_type = 0x3b;
			break;
		default:
			printk("This channel is not supported!!!\n");
			break;
	}
	ch->rise_desc = (struct hdma_desc *)CKSEG1ADDR(&_gdesc[0]);
	ch->even_desc = (struct hdma_desc *)CKSEG1ADDR(&_gdesc[1]);
	ch->fall_desc = (struct hdma_desc *)CKSEG1ADDR(&_gdesc[2]);
	ch->rise_desc->dcm = DCM_SAI | DCM_LINK;
	ch->rise_desc->dsa = virt_to_phys(pwm_smc->rise_buf);
	ch->rise_desc->dta = PWM_BASE + PWM_DR(ch->channel);
	ch->rise_desc->dtc = pwm_smc->pwm_cfg_info.set_up_num | ((((unsigned int)ch->even_desc & 0xff0) >> 4) << 24);
	ch->rise_desc->drt = req_type;

	ch->even_desc->dcm = DCM_LINK;
	ch->even_desc->dsa = virt_to_phys(&pwm_smc->even_buf);
	ch->even_desc->dta = PWM_BASE + PWM_DR(ch->channel);
	ch->even_desc->dtc = pwm_smc->pwm_cfg_info.step | ((((unsigned int)ch->fall_desc & 0xff0) >> 4) << 24);
	ch->even_desc->drt = req_type;

	ch->fall_desc->dcm = DCM_SAI;
	ch->fall_desc->dsa = virt_to_phys(pwm_smc->fall_buf);
	ch->fall_desc->dta = PWM_BASE + PWM_DR(ch->channel);
	ch->fall_desc->dtc = pwm_smc->pwm_cfg_info.set_up_num;
	ch->fall_desc->drt = req_type;
}

static void start_dma_chans(struct dma_channel *ch)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(ch->chan);
	writel(0, dmac->iomem + CH_DCS);
	writel((unsigned int)virt_to_phys(ch->rise_desc), dmac->iomem + CH_DDA);
	/* initiate descriptor fetch */
	writel(BIT(dmac->id), dmac->engine->iomem + DDRS);
	writel(1 | (1 << 30), dmac->iomem + CH_DCS);
}

static void restart_dma_chans(struct dma_channel *ch)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(ch->chan);

	writel(0, dmac->iomem + CH_DCS);
	writel((unsigned int)virt_to_phys(ch->fall_desc), dmac->iomem + CH_DDA);
	writel(BIT(dmac->id), dmac->engine->iomem + DDRS);
	writel(1 | (1 << 30), dmac->iomem + CH_DCS);
}

static void stop_dma_chans(struct dma_channel *ch)
{
	struct ingenic_dma_chan *dmac = to_ingenic_dma_chan(ch->chan);
	ch->rise_desc->dtc &= ~(0xff << 24);
	ch->even_desc->dtc &= ~(0xff << 24);
	ch->fall_desc->dtc &= ~(0xff << 24);
	writel(0, dmac->iomem + CH_DCS);
}

static void pwm_dma_config(struct ingenic_pwm_smc *pwm_smc)
{
	struct dma_channel *ch;
	//extra add 16 byte for descriptor align.
	ch = kmalloc(sizeof(struct dma_channel), GFP_KERNEL);
	if (IS_ERR_OR_NULL(ch)) {
		pr_err("dma request channel failed!\n");
	}
	ch->chan = pwm_smc->chan->dma_chan;
	ch->channel = pwm_smc->pwm_cfg_info.channel;
	pwm_smc->ch = ch;
	build_desc(pwm_smc, ch);
}

static void set_private_data(struct file *filp, struct ingenic_pwm_smc *pwm_smc)
{
	filp->private_data = pwm_smc;
}

static struct ingenic_pwm_smc *get_private_data(struct file *filp)
{
	return filp->private_data;
}

static int ingenic_pwm_smc_open(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
	struct ingenic_pwm_smc *pwm_smc = container_of(cdev, struct ingenic_pwm_smc, cdev);

	mutex_lock(&pwm_smc->mutex);
	pwm_smc->ch = NULL;
	pwm_smc->pwm = NULL;
	pwm_smc->chan = NULL;
	pwm_smc->rise_buf = NULL;
	pwm_smc->fall_buf = NULL;
	if (pwm_smc->open_ed) {
		mutex_unlock(&pwm_smc->mutex);
		return -EBUSY;
	}
	pwm_smc->open_ed = 1;

	set_private_data(filp, pwm_smc);

	mutex_unlock(&pwm_smc->mutex);

	return 0;
}

static int ingenic_pwm_smc_read(struct file *filp, char *user_buf, size_t count, loff_t *f_pos)
{
	print_dbg("-------------[%s]-------->\n", __func__);
	return count;
}

static int ingenic_pwm_smc_write(struct file *filp, const char *user_buf, size_t count, loff_t *f_pos)
{
	struct ingenic_pwm_smc *pwm_smc = get_private_data(filp);
	struct pwm_form *src_buf = NULL;
	int i = 0;
	int num = count / 4;
	src_buf = (struct pwm_form *)user_buf;

	for (i = 0; i < num; i++) {
		pwm_smc->rise_buf[i] = src_buf[i];
		pwm_smc->fall_buf[i] = src_buf[num - i - 1];
	}
	pwm_smc->even_buf.High = src_buf[num - 1].High;
	pwm_smc->even_buf.Low = src_buf[num - 1].Low;
	dma_cache_wback_inv((phys_addr_t)pwm_smc->rise_buf, pwm_smc->pwm_cfg_info.set_up_num * 4);
	dma_cache_wback_inv((phys_addr_t)pwm_smc->fall_buf, pwm_smc->pwm_cfg_info.set_up_num * 4);
	dma_cache_wback_inv((phys_addr_t)&pwm_smc->even_buf, sizeof(pwm_smc->even_buf));
	return count;
}

static long ingenic_pwm_smc_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
	struct ingenic_pwm_smc *pwm_smc = get_private_data(filp);
	struct ingenic_dma_chan *dmac = NULL;
	struct pwm_cfg_info *pwm_cfg_info = &pwm_smc->pwm_cfg_info;
	struct ingenic_pwm_chan *chan = NULL;
	int ret = 0, cnt = 0;
	int reg = 0;

	mutex_lock(&pwm_smc->mutex);
	switch (cmd) {
		case PWM_SMC_GET_INFO:
			pwm_smc->pwm_cfg_info.duty_ns = 5000;
			pwm_smc->pwm_cfg_info.period_ns = 10000;
			ret = copy_to_user((void *)args, pwm_cfg_info, sizeof(struct pwm_cfg_info));
			if (ret) {
				printk("[error]: pwm smc get info failed!!! ret : %d\n", ret);
			}
			break;
		case PWM_SMC_SET_INFO:
			ret = copy_from_user(pwm_cfg_info, (void *)args, sizeof(struct pwm_cfg_info));
			if (ret) {
				printk("[error]: pwm smc set info failed!!! ret : %d\n", ret);
			}
			break;
		case PWM_SMC_CONFIG:
			ret = pwm_config(pwm_smc->pwm, pwm_cfg_info->duty_ns, pwm_cfg_info->period_ns);
			if (ret < 0) {
				printk("[error]: pwm smc config failed!!! ret : %d\n", ret);
			}
			pwm_dma_config(pwm_smc);
			break;
		case PWM_SMC_START:
			pwm_smc_write(PWM_DCFF, 1 << pwm_cfg_info->channel);
			ret = pwm_enable(pwm_smc->pwm);
			if (ret < 0) {
				printk("[error]: pwm smc enable failed!!! ret : %d\n", ret);
			}
			reg = pwm_smc_read(PWM_DRE);
			reg &= ~(1 << pwm_cfg_info->channel);
			pwm_smc_write(PWM_DRE, reg);//取消dma request
			start_dma_chans(pwm_smc->ch);
			reg = pwm_smc_read(PWM_DRE);
			reg |= 1 << pwm_cfg_info->channel;
			pwm_smc_write(PWM_DRE, reg); // 使能dma请求.
			break;
		case PWM_SMC_GEN_STOP:
			dmac = to_ingenic_dma_chan(pwm_smc->ch->chan);
			cnt = pwm_smc_read(PWM_ON(pwm_cfg_info->channel));
			pwm_smc->pwm_cfg_info.cur_pos = cnt;

			reg = pwm_smc_read(PWM_DRE);
			reg &= ~(1 << pwm_cfg_info->channel);
			pwm_smc_write(PWM_DRE, reg);//取消dma request

			ret = readl(dmac->iomem + CH_DCS);
			writel(ret | (1 << 5), dmac->iomem + CH_DCS); //关dma

			restart_dma_chans(pwm_smc->ch);

			reg = pwm_smc_read(PWM_DRE);
			reg |= 1 << pwm_cfg_info->channel;
			pwm_smc_write(PWM_DRE, reg); // 使能dma请求.
			ret = copy_to_user((void *)args, pwm_cfg_info, sizeof(struct pwm_cfg_info));
			if (ret) {
				printk("[error]: pwm smc get info failed!!! ret : %d\n", ret);
			}
			break;
		case PWM_SMC_QCK_STOP:
			dmac = to_ingenic_dma_chan(pwm_smc->ch->chan);
			cnt = pwm_smc_read(PWM_ON(pwm_cfg_info->channel));
			pwm_smc->pwm_cfg_info.cur_pos = cnt;

			reg = pwm_smc_read(PWM_DRE);
			reg &= ~(1 << pwm_cfg_info->channel);
			pwm_smc_write(PWM_DRE, reg);//取消dma request

			ret = readl(dmac->iomem + CH_DCS);
			writel(ret | (1 << 5), dmac->iomem + CH_DCS);
			ret = copy_to_user((void *)args, pwm_cfg_info, sizeof(struct pwm_cfg_info));
			if (ret) {
				printk("[error]: pwm smc get info failed!!! ret : %d\n", ret);
			}
			break;
		case PWM_SMC_GET_RUN_STEP_INFO:
			cnt = pwm_smc_read(PWM_ON(pwm_cfg_info->channel));
			pwm_smc->pwm_cfg_info.cur_pos = cnt;
			ret = copy_to_user((void *)args, pwm_cfg_info, sizeof(struct pwm_cfg_info));
			if (ret) {
				printk("[error]: PWM_SMC_GET_RUN_STEP_INFO failed!!! \n");
			}
			break;
		case PWM_SMC_REQUEST_CHANNEL:
			pwm_smc->pwm = pwm_get(&pwm_smc->pdev->dev, NULL);
			if (IS_ERR(pwm_smc->pwm)) {
				ret = PTR_ERR(pwm_smc->pwm);
				printk("[error]: pwm smc channel request failed!!! ret : %d\n", ret);
				mutex_unlock(&pwm_smc->mutex);
				return -EINVAL;
			}
			chan = (struct ingenic_pwm_chan *)pwm_get_chip_data(pwm_smc->pwm);

			chan->mode = DMA_MODE_SMC;
			chan->finish_level = pwm_cfg_info->finish_level;
			pwm_smc->chan = chan;
			pwm_smc->rise_buf = kmalloc(pwm_cfg_info->set_up_num * 4, GFP_KERNEL);
			if (pwm_smc->rise_buf == NULL) {
				printk("[error] pwm_smc->rise_buf kzalloc!!!\n");
				mutex_unlock(&pwm_smc->mutex);
				return -EINVAL;
			}
			pwm_smc->fall_buf = kmalloc(pwm_cfg_info->set_up_num * 4, GFP_KERNEL);
			if (pwm_smc->fall_buf == NULL) {
				printk("[error] pwm_smc->fall_buf kzalloc!!!\n");
				kfree(pwm_smc->rise_buf);
				mutex_unlock(&pwm_smc->mutex);
				return -EINVAL;
			}
			break;
		case PWM_SMC_DISABLE:
			pwm_disable(pwm_smc->pwm);
			pwm_put(pwm_smc->pwm);
		default:
			break;
	}
	mutex_unlock(&pwm_smc->mutex);
	return ret;
}

static int ingenic_pwm_smc_close(struct inode *inode, struct file *filp)
{
	struct ingenic_pwm_smc *pwm_smc = get_private_data(filp);
	mutex_lock(&pwm_smc->mutex);
	stop_dma_chans(pwm_smc->ch);
	if (pwm_smc->pwm) {
		pwm_disable(pwm_smc->pwm);
		pwm_put(pwm_smc->pwm);
	}

	if (pwm_smc->rise_buf) {
		kfree(pwm_smc->rise_buf);
	}
	if (pwm_smc->fall_buf) {
		kfree(pwm_smc->fall_buf);
	}
	if (pwm_smc->ch) {
		kfree(pwm_smc->ch);
	}
	pwm_smc->open_ed = 0;
	mutex_unlock(&pwm_smc->mutex);
	return 0;

}

static struct file_operations ingenic_pwm_smc_ops = {
	.owner = THIS_MODULE,
	.write = ingenic_pwm_smc_write,
	.read = ingenic_pwm_smc_read,
	.open = ingenic_pwm_smc_open,
	.release = ingenic_pwm_smc_close,
	.unlocked_ioctl = ingenic_pwm_smc_ioctl,
};

static int ingenic_pwm_smc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct ingenic_pwm_smc *pwm_smc;
	dev_t devno;

	pwm_smc = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_pwm_smc), GFP_KERNEL);
	if (!pwm_smc) {
		dev_err(&pdev->dev, "alloc pwm_smc failed!\n");
		return -ENOMEM;
	}
	pwm_smc->pdev = pdev;

	platform_set_drvdata(pdev, pwm_smc);

	mutex_init(&pwm_smc->mutex);

	pwm_smc->minor = 0;
	pwm_smc->nr_devs = 1;
	ret = alloc_chrdev_region(&devno, pwm_smc->minor, pwm_smc->nr_devs, "ingenic_pwm_smc");
	if (ret) {
		dev_err(&pdev->dev, "alloc chrdev failed!\n");
		goto __err0;
	}
	pwm_smc->major = MAJOR(devno);
	pwm_smc->dev_num = MKDEV(pwm_smc->major, pwm_smc->minor);
	dev_dbg(&pdev->dev, "%s():%d >>> pwm_smc->major = %d, pwm_smc->minor = %d, pwm_smc->dev_num = %x\n",
	        __func__, __LINE__, pwm_smc->major, pwm_smc->minor, pwm_smc->dev_num);

	cdev_init(&pwm_smc->cdev, &ingenic_pwm_smc_ops);
	pwm_smc->cdev.owner = THIS_MODULE;
	ret = cdev_add(&pwm_smc->cdev, pwm_smc->dev_num, 1);
	if (ret) {
		dev_err(&pdev->dev, "cdev_add failed!\n");
		goto __err1;
	}

	pwm_smc->class = class_create("ingenic_pwm_smc");
	if (IS_ERR(pwm_smc->class)) {
		dev_err(&pdev->dev, "class_create failed!\n");
		ret = PTR_ERR(pwm_smc->class);
		goto __err2;
	}

	pwm_smc->dev = device_create(pwm_smc->class, NULL, pwm_smc->dev_num, NULL, "ingenic_pwm_smc");
	if (IS_ERR(pwm_smc->dev)) {
		dev_err(&pdev->dev, "device_create failed!\n");
		ret = PTR_ERR(pwm_smc->dev);
		goto __err3;
	}

	pwm_smc->dev->coherent_dma_mask = pdev->dev.coherent_dma_mask;
	printk("Ingenic pwm smc driver init successfully!\n");

	return 0;
__err3:
	class_destroy(pwm_smc->class);
__err2:
	cdev_del(&pwm_smc->cdev);
__err1:
	unregister_chrdev_region(pwm_smc->dev_num, pwm_smc->nr_devs);
__err0:
	kfree(pwm_smc);
	return ret;
}

static int ingenic_pwm_smc_remove(struct platform_device *pdev)
{
	struct ingenic_pwm_smc *pwm_smc = platform_get_drvdata(pdev);

	device_destroy(pwm_smc->class, pwm_smc->dev_num);
	class_destroy(pwm_smc->class);
	cdev_del(&pwm_smc->cdev);
	unregister_chrdev_region(pwm_smc->dev_num, pwm_smc->nr_devs);

	return 0;
}

static const struct of_device_id ingenic_pwm_smc_match[] = {
	{ .compatible = "ingenic,x2660-pwm-step-motor", },
	{}
};
MODULE_DEVICE_TABLE(of, ingenic_pwm_smc_match);

static struct platform_driver ingenic_pwm_smc_driver = {
	.probe          = ingenic_pwm_smc_probe,
	.remove         = ingenic_pwm_smc_remove,
	.driver     = {
		.name   = "ingenic_pwm_smc",
		.owner  = THIS_MODULE,
		.of_match_table = ingenic_pwm_smc_match,
	},
};

static int __init ingenic_pwm_smc_init(void)
{
	int ret;

	ret = platform_driver_register(&ingenic_pwm_smc_driver);
	if (ret) {
		platform_driver_unregister(&ingenic_pwm_smc_driver);
	}

	return ret;
}

static void __exit ingenic_pwm_smc_exit(void)
{
	platform_driver_unregister(&ingenic_pwm_smc_driver);
}

module_init(ingenic_pwm_smc_init);
module_exit(ingenic_pwm_smc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Ingenic pwm smc driver");
MODULE_AUTHOR("lichao.ren@ingenic.com");
