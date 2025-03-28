/**
 * drivers/mfd/ingenic-adc-aux.c
 *
 * aux1 aux2 channels voltage sample interface for Ingenic SoC
 *
 * Copyright(C)2012 Ingenic Semiconductor Co., LTD.
 * http://www.ingenic.cn
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */

#include <linux/err.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/proc_fs.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>/*for vfree*/
#include <linux/miscdevice.h>
#include <irq.h>

#include "ingenic_adc.h"

#ifdef CONFIG_SADC_AUX_VERF_3V3
	static unsigned int VREF_ADC = 3300;
#else
	static unsigned int VREF_ADC = 1800;
#endif

#ifdef CONFIG_SADC_AUX_12BIT
	#define AUXCONST    4096
	#define AUX_VOLT_LOW    0xfff
	#define AUX_VOLT_HIGH   0xfff0000
#else
	#define AUXCONST   1024
	#define AUX_VOLT_LOW    0x3ff
	#define AUX_VOLT_HIGH   0x3ff0000
#endif

#define ADC_MAGIC_NUMBER    'A'
#define ADC_ENABLE          _IO(ADC_MAGIC_NUMBER, 11)
#define ADC_DISABLE         _IO(ADC_MAGIC_NUMBER, 22)
#define ADC_SET_VREF        _IOW(ADC_MAGIC_NUMBER, 33, unsigned int)

#ifndef BITS_H2L
	#define BITS_H2L(msb, lsb)  ((0xFFFFFFFF >> (32-((msb)-(lsb)+1))) << (lsb))
#endif

/**
 *  * INIT_COMPLETION - reinitialize a completion structure
 *   * @x:  completion structure to be reinitialized
 *    *
 *     * This macro should be used to reinitialize a completion structure so it can
 *      * be reused. This is especially important after complete_all() is used.
 *       */
#define INIT_COMPLETION(x)      ((x).done = 0)

struct ingenic_adc_aux {
	struct platform_device *pdev;

	struct resource *mem;
	void __iomem *base;

	int irq;

	atomic_t clk_ref;

	const struct mfd_cell *cell;

	struct ingenic_adc *mfd;

	unsigned int voltage;

	struct completion read_completion;

	struct miscdevice mdev;
};

enum aux_ch {
	SADC_AUX0,
	SADC_AUX1,
	SADC_AUX2,
	SADC_AUX3,
	SADC_AUX4,
	SADC_AUX5,
};
extern int ingenic_adc_set_config(struct device *dev, uint32_t mask, uint32_t val);

static irqreturn_t ingenic_ingenic_adc_aux_irq_handler(int irq, void *devid)
{
	struct ingenic_adc_aux *ingenic_adc_aux = (struct ingenic_adc_aux *)devid;

	complete(&ingenic_adc_aux->read_completion);

	return IRQ_HANDLED;
}

static int ingenic_adc_aux_suspend(struct platform_device *pdev, pm_message_t state)
{
	//struct ingenic_adc_aux *adc_aux = platform_get_drvdata(pdev);
	//adc_aux->cell->disable(pdev);
	return 0;
}

static int ingenic_adc_aux_resume(struct platform_device *pdev)
{
	//struct ingenic_adc_aux *adc_aux = platform_get_drvdata(pdev);
	//adc_aux->cell->enable(pdev);
	return 0;
}

int ingenic_adc_aux_sample_volt(enum aux_ch channels, struct ingenic_adc_aux *ingenic_adc_aux)
{
	unsigned long tmp;
	unsigned int sadc_volt = 0;

	if (!ingenic_adc_aux) {
		printk("ingenic_adc_aux is null ! return\n");
		return -EINVAL;
	}

	INIT_COMPLETION(ingenic_adc_aux->read_completion);

	ingenic_adc_aux->mfd->enable(ingenic_adc_aux->pdev);

	enable_irq(ingenic_adc_aux->irq);

restart:
	tmp = wait_for_completion_interruptible_timeout(&ingenic_adc_aux->read_completion, HZ);
	if (tmp > 0) {
		if ((channels == 0) || (channels == 2)) {
			sadc_volt = readl(ingenic_adc_aux->base) & AUX_VOLT_LOW;
		} else {
			sadc_volt = (readl(ingenic_adc_aux->base - 2) & AUX_VOLT_HIGH) >> 16;
		}
	} else if (tmp == -ERESTARTSYS) {
		goto restart;
	} else {
		sadc_volt = tmp ? tmp : -ETIMEDOUT;
	}

	if (sadc_volt < 0) {
		printk("ingenic_adc_aux read value error!!\n");
		disable_irq(ingenic_adc_aux->irq);
		ingenic_adc_aux->mfd->disable(ingenic_adc_aux->pdev);
		return -EIO;
	}

	disable_irq(ingenic_adc_aux->irq);
	ingenic_adc_aux->mfd->disable(ingenic_adc_aux->pdev);

	sadc_volt = sadc_volt * VREF_ADC / AUXCONST;

	return sadc_volt;
}

int ingenic_adc_aux_open(struct inode *inode, struct file *filp)
{
	uint16_t val;
	struct miscdevice *dev = filp->private_data;
	struct ingenic_adc_aux *axu = container_of(dev, struct ingenic_adc_aux, mdev);

	if (atomic_inc_return(&axu->clk_ref) == 1) {
		val = *(volatile unsigned int *)(SADC_ENA);
		val &= ~BIT(15);
		*(volatile unsigned int *)(SADC_ENA) = val; //power on sadc.
		msleep(2);
	}
	return 0;
}

int ingenic_adc_aux_release(struct inode *inode, struct file *filp)
{
	uint16_t val;
	struct miscdevice *dev = filp->private_data;
	struct ingenic_adc_aux *axu = container_of(dev, struct ingenic_adc_aux, mdev);

	if (atomic_dec_return(&axu->clk_ref) == 0) {
		val = *(volatile unsigned int *)(SADC_ENA);
		val |= BIT(15);
		*(volatile unsigned int *)(SADC_ENA) = val; //power down sadc.
	}
	return 0;
}

ssize_t ingenic_adc_aux_read(struct file *filp, char *buf, size_t len, loff_t *off)
{
	unsigned int sadc_val = 0;
	struct miscdevice *dev = filp->private_data;
	struct ingenic_adc_aux *aux = container_of(dev, struct ingenic_adc_aux, mdev);

	sadc_val = ingenic_adc_aux_sample_volt(aux->pdev->id, aux);
	if (sadc_val < 0) {
		printk("ingenic_adc_aux read value error !!\n");
		return -EINVAL;
	}

	if (copy_to_user(buf, &sadc_val, sizeof(int))) {
		return -EFAULT;
	}

	return sizeof(int);
}

static long ingenic_adc_aux_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct miscdevice *dev = filp->private_data;
	struct ingenic_adc_aux *adc_aux = container_of(dev, struct ingenic_adc_aux, mdev);

	if (_IOC_TYPE(cmd) == ADC_MAGIC_NUMBER) {
		switch (cmd) {
		case ADC_ENABLE:
			ret = adc_aux->mfd->enable(adc_aux->pdev);
			break;
		case ADC_DISABLE:
			ret = adc_aux->mfd->disable(adc_aux->pdev);
			break;
		case ADC_SET_VREF:
			VREF_ADC = *(unsigned int *)arg;
			printk("VREF_ADC=%d\n", VREF_ADC);
			break;
		default:
			ret = -1;
			printk("%s:unsupported ioctl cmd\n", __func__);
		}
	}

	return ret;
}

struct file_operations ingenic_adc_aux_fops = {
	.owner = THIS_MODULE,
	.open = ingenic_adc_aux_open,
	.release = ingenic_adc_aux_release,
	.read = ingenic_adc_aux_read,
	.unlocked_ioctl = ingenic_adc_aux_ioctl,
};

extern int key_fun_init(struct ingenic_adc_aux *ingenic_adc_aux);
static int ingenic_adc_aux_probe(struct platform_device *pdev)
{
	int ret = 0;
	char aux_name[20];
	struct ingenic_adc *adc = dev_get_drvdata(pdev->dev.parent);

	struct ingenic_adc_aux *ingenic_adc_aux = NULL;

	ingenic_adc_aux = kzalloc(sizeof(*ingenic_adc_aux), GFP_KERNEL);
	if (!ingenic_adc_aux) {
		dev_err(&pdev->dev, "Failed to allocate driver structre\n");
		return -ENOMEM;
	}

	ingenic_adc_aux->cell = mfd_get_cell(pdev);
	if (!ingenic_adc_aux->cell) {
		ret = -ENOENT;
		dev_err(&pdev->dev, "Failed to get mfd cell for ingenic_adc_aux!\n");
		goto err_free;
	}

	ingenic_adc_aux->irq = platform_get_irq(pdev, 0);
	if (ingenic_adc_aux->irq < 0) {
		ret = ingenic_adc_aux->irq;
		dev_err(&pdev->dev, "Failed to get platform irq: %d\n", ret);
		goto err_free;
	}

	ingenic_adc_aux->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!ingenic_adc_aux->mem) {
		ret = -ENOENT;
		dev_err(&pdev->dev, "Failed to get platform mmio resource\n");
		goto err_free;
	}

	ingenic_adc_aux->mem = request_mem_region(ingenic_adc_aux->mem->start,
	                       resource_size(ingenic_adc_aux->mem), pdev->name);
	if (!ingenic_adc_aux->mem) {
		ret = -EBUSY;
		dev_err(&pdev->dev, "Failed to request mmio memory region\n");
		goto err_free;
	}

	ingenic_adc_aux->base = ioremap(ingenic_adc_aux->mem->start, resource_size(ingenic_adc_aux->mem));
	if (!ingenic_adc_aux->base) {
		ret = -EBUSY;
		dev_err(&pdev->dev, "Failed to ioremap mmio memory\n");
		goto err_free;
	}

	ingenic_adc_aux->mfd = adc;
	ingenic_adc_aux->pdev = pdev;
	ingenic_adc_aux->mdev.minor = MISC_DYNAMIC_MINOR;
	sprintf(aux_name, "ingenic_adc_aux_%d", pdev->id);
	ingenic_adc_aux->mdev.name = aux_name;
	ingenic_adc_aux->mdev.fops = &ingenic_adc_aux_fops;

	ret = misc_register(&ingenic_adc_aux->mdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "misc_register failed\n");
		goto err_free;
	}
	atomic_set(&ingenic_adc_aux->clk_ref, 0);

	init_completion(&ingenic_adc_aux->read_completion);

	ret = request_irq(ingenic_adc_aux->irq, ingenic_ingenic_adc_aux_irq_handler, 0, pdev->name, ingenic_adc_aux);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq %d\n", ret);
		goto err_free;
	}

	disable_irq(ingenic_adc_aux->irq);

	platform_set_drvdata(pdev, ingenic_adc_aux);

#ifdef CONFIG_ADC_BASED_KEY_FUN
	if (!strncmp(aux_name, "ingenic_adc_aux_1", strlen("ingenic_adc_aux_1"))) {
		key_fun_init(ingenic_adc_aux);
	}
#endif

	printk("ingenic sadc aux%d probe success\n", pdev->id);
	return 0;

err_free :
	kfree(ingenic_adc_aux);
	return ret;

}

static int ingenic_adc_aux_remove(struct platform_device *pdev)
{
	struct ingenic_adc_aux *ingenic_adc_aux = platform_get_drvdata(pdev);

	misc_deregister(&ingenic_adc_aux->mdev);
	free_irq(ingenic_adc_aux->irq, ingenic_adc_aux);
	iounmap(ingenic_adc_aux->base);
	release_mem_region(ingenic_adc_aux->mem->start, resource_size(ingenic_adc_aux->mem));
	kfree(ingenic_adc_aux);

	return 0;
}

static struct platform_driver ingenic_adc_aux_driver = {
	.probe  = ingenic_adc_aux_probe,
	.remove = ingenic_adc_aux_remove,
	.driver = {
		.name   = "ingenic-aux",
		.owner  = THIS_MODULE,
	},
	.suspend    = ingenic_adc_aux_suspend,
	.resume     = ingenic_adc_aux_resume,
};

static int __init ingenic_adc_aux_init(void)
{
	platform_driver_register(&ingenic_adc_aux_driver);

	return 0;
}

static void __exit ingenic_adc_aux_exit(void)
{
	platform_driver_unregister(&ingenic_adc_aux_driver);
}

module_init(ingenic_adc_aux_init);
module_exit(ingenic_adc_aux_exit);

MODULE_ALIAS("platform: ingenic ingenic_adc_aux");
MODULE_AUTHOR("Guo Xu<xu.guo@ingenic.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ingenic adc aux sample driver");
