/*
 * drivers/misc/ingenic_rsa.c - Ingenic rsa driver
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 * Author: mayuanjun <yuanjun.ma@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of.h>

#define MAX_KEY_LEN_WORD 64
#define DEV_NAME  "rsa"

#define MCU_BOOT_DDR 0xb3422000

#define DRV_NAME "jz-rsa"
#define MAX_KEY_LEN_WORD 64

struct jz_rsa {
	struct miscdevice mdev;
	struct device *dev;

	struct clk      *clk_gate;
	void __iomem *iomem;
	unsigned int irq;

	bool busy;

	struct mutex mutex;

	wait_queue_head_t wq;

	unsigned int n_len;
	unsigned int per_done;
	unsigned int rsa_done;
};

struct rsa_key {
	unsigned int *e;
	unsigned int *n;
	unsigned int rsa_mode;
};

struct rsa_data {
	unsigned int *input;
	unsigned int inlen;
	unsigned int *output;
};

#define RSA_PERPARE_KEY 0x1
#define RSA_DO_CRYPT    0x2

#define RSAC        0x0
#define RSAE        0x4
#define RSAN        0x8
#define RSAM        0xc
#define RSAP        0x10

#define RSAC_RSA_INT_M  (1 << 17)
#define RSAC_PER_INT_M  (1 << 16)
#define RSAC_RSA_2048   (1 << 7)
#define RSAC_RSAC   (1 << 6)
#define RSAC_RSAD   (1 << 5)
#define RSAC_RSAS   (1 << 4)
#define RSAC_PERC   (1 << 3)
#define RSAC_PERD   (1 << 2)
#define RSAC_PERS   (1 << 1)
#define RSAC_EN     (1 << 0)

#define mcu_boot()  \
	do {            \
		*(volatile unsigned int *)0xb3421030 &= ~1; \
	} while(0)

#define mcu_reset()     \
	do {            \
		*(volatile unsigned int *)0xb3421030 |= 1; \
	} while(0)

static int inline rsa_readl(struct jz_rsa *rsa, unsigned int offset)
{
	return readl(rsa->iomem + offset);
}

static void inline rsa_writel(struct jz_rsa *rsa, unsigned int offset, unsigned int val)
{
	writel(val, rsa->iomem + offset);
}

static int rsa_perpare_key(struct jz_rsa *rsa, struct rsa_key *rsa_key)
{
	unsigned int n[MAX_KEY_LEN_WORD];
	unsigned int e[MAX_KEY_LEN_WORD];
	unsigned int keylen = rsa_key->rsa_mode / 32;

	unsigned int tmp;
	unsigned int i;
	int ret = 0;

	mutex_lock(&rsa->mutex);

	rsa->per_done = 0;
	rsa->n_len = keylen;

	if (copy_from_user(e, rsa_key->e, keylen * 4)) {
		dev_err(rsa->dev, "Failed to copy rsa_key->e form user!\n");
		ret = -EFAULT;
		goto err;
	}
	if (copy_from_user(n, rsa_key->n, keylen * 4)) {
		dev_err(rsa->dev, "Failed to copy rsa_key->n form user!\n");
		ret = -EFAULT;
		goto err;
	}

	tmp = rsa_readl(rsa, RSAC);
	if (rsa_key->rsa_mode == 2048) {
		tmp |= RSAC_RSA_2048;
	} else if (rsa_key->rsa_mode == 1024) {
		tmp &= ~RSAC_RSA_2048;
	}
	tmp &= ~RSAC_PER_INT_M;
	rsa_writel(rsa, RSAC, tmp);

	for (i = 0; i < keylen; i++) {
		rsa_writel(rsa, RSAE, e[i]);
	}
	for (i = 0; i < keylen; i++) {
		rsa_writel(rsa, RSAN, n[i]);
	}
	rsa_writel(rsa, RSAC, RSAC_PERS | rsa_readl(rsa, RSAC));

	ret = wait_event_interruptible(rsa->wq, rsa->per_done == 1);
	if (ret < 0) {
		dev_err(rsa->dev, "key perpare can't wait irq\n");
		ret = -EFAULT;
		goto err;
	}

	rsa_writel(rsa, RSAC, RSAC_PERC | rsa_readl(rsa, RSAC));
	rsa_writel(rsa, RSAC, ~RSAC_PERC & rsa_readl(rsa, RSAC));

	mutex_unlock(&rsa->mutex);
	return 0;

err:
	mutex_unlock(&rsa->mutex);
	return ret;
}

static int rsa_do_crypt(struct jz_rsa *rsa, struct rsa_data *rsa_data)
{
	unsigned int input[MAX_KEY_LEN_WORD];
	unsigned int output[MAX_KEY_LEN_WORD];
	unsigned int *id = rsa_data->input;
	unsigned int *od = rsa_data->output;
	unsigned int keylen = rsa->n_len;
	unsigned int len = rsa_data->inlen;

	unsigned int i;
	int ret = 0;

	mutex_lock(&rsa->mutex);

	if (len % keylen) {
		pr_err("Data len err \n");
		ret = -EINVAL;
		goto err;
	}

	while (len > 0) {

		rsa->rsa_done = 0;

		if (copy_from_user(input, id, keylen * 4)) {
			dev_err(rsa->dev, "Failed to copy rsa_data->input form user!\n");
			ret = -EFAULT;
			goto err;
		}

		for (i = 0; i < keylen; i++) {
			rsa_writel(rsa, RSAM, input[i]);
		}

		rsa_writel(rsa, RSAC, ~RSAC_RSA_INT_M & rsa_readl(rsa, RSAC));
		rsa_writel(rsa, RSAC, RSAC_RSAS | rsa_readl(rsa, RSAC));

		ret = wait_event_interruptible(rsa->wq, rsa->rsa_done == 1);
		if (ret < 0) {
			dev_err(rsa->dev, "rsa crypt can't wait irq\n");
			ret = -EFAULT;
			goto err;
		}

		for (i = 0; i < keylen; i++) {
			output[keylen - 1 - i] = rsa_readl(rsa, RSAP);
		}

		rsa_writel(rsa, RSAC, rsa_readl(rsa, RSAC) | RSAC_RSAC);
		rsa_writel(rsa, RSAC, rsa_readl(rsa, RSAC) & ~RSAC_RSAC);

		if (copy_to_user(od, output, keylen * 4)) {
			dev_err(rsa->dev, "Failed to copy rsa_data->input form user!\n");
			ret = -EFAULT;
			goto err;
		}

		len -= keylen;
		id += keylen;
		od += keylen;

	}

	mutex_unlock(&rsa->mutex);
	return 0;

err:
	mutex_unlock(&rsa->mutex);
	return ret;

}

static int rsa_open(struct inode *inode, struct file *flip)
{
	struct miscdevice *mdev = flip->private_data;
	struct jz_rsa *rsa = container_of(mdev, struct jz_rsa, mdev);

	mutex_lock(&rsa->mutex);
	if (rsa->busy) {
		dev_err(rsa->dev, "Device is busy!\n");
		mutex_unlock(&rsa->mutex);
		return -EBUSY;
	}

	rsa->busy = 1;

	rsa_writel(rsa, RSAC, RSAC_EN | rsa_readl(rsa, RSAC));

	mutex_unlock(&rsa->mutex);

	return 0;
}

static int rsa_release(struct inode *inode, struct file *flip)
{
	struct miscdevice *mdev = flip->private_data;
	struct jz_rsa *rsa = container_of(mdev, struct jz_rsa, mdev);

	mutex_lock(&rsa->mutex);

	rsa->busy = 0;

	rsa_writel(rsa, RSAC, ~RSAC_EN & rsa_readl(rsa, RSAC));

	mutex_unlock(&rsa->mutex);

	return 0;
}

static long rsa_ioctl(struct file *flip, unsigned int cmd, unsigned long args)
{
	struct miscdevice *mdev = flip->private_data;
	struct jz_rsa *rsa = container_of(mdev, struct jz_rsa, mdev);
	struct rsa_key rsa_key;
	struct rsa_data rsa_data;
	int ret = 0;

	switch (cmd) {
	case RSA_PERPARE_KEY:

		if (copy_from_user(&rsa_key, (void *)args, sizeof(struct rsa_key))) {
			dev_err(rsa->dev, "Failed to copy rsa_key form user!\n");
			ret = -EFAULT;
			goto err;
		}

		ret = rsa_perpare_key(rsa, &rsa_key);

		break;
	case RSA_DO_CRYPT:
		if (rsa->n_len != 32 && rsa->n_len != 64) {
			dev_err(rsa->dev, "RSA key len err!\n");
			ret = -EINVAL;
			goto err;
		}
		if (copy_from_user(&rsa_data, (void *)args, sizeof(struct rsa_data))) {
			dev_err(rsa->dev, "Could not copy data from user!\n");
			ret = -EFAULT;
			goto err;
		}

		ret = rsa_do_crypt(rsa, &rsa_data);
		if (ret < 0) {
			dev_err(rsa->dev, "Failed to crypt data!\n");
			goto err;
		}

		break;
	default:
		dev_err(rsa->dev, "Unsupport IO cmd: %x\n", cmd);

	}

	return 0;
err:
	return ret;
}

static struct file_operations rsa_misc_fops = {
	.owner = THIS_MODULE,
	.open = rsa_open,
	.release = rsa_release,
	.unlocked_ioctl = rsa_ioctl,
};

static irqreturn_t rsa_irq(int irq, void *data)
{
	struct jz_rsa *rsa = (struct jz_rsa *)data;

	if (rsa_readl(rsa, RSAC) & RSAC_RSAD) {
		rsa_writel(rsa, RSAC, ~RSAC_RSAS & rsa_readl(rsa, RSAC));
		rsa_writel(rsa, RSAC, RSAC_RSA_INT_M | rsa_readl(rsa, RSAC));
		rsa->rsa_done = 1;
	} else if (rsa_readl(rsa, RSAC) & RSAC_PERD) {
		rsa_writel(rsa, RSAC, ~RSAC_PERS & rsa_readl(rsa, RSAC));
		rsa_writel(rsa, RSAC, RSAC_PER_INT_M | rsa_readl(rsa, RSAC));
		rsa->per_done = 1;
	}

	wake_up(&rsa->wq);

	return IRQ_HANDLED;
}

#define cpm_readl(o)         (*(volatile unsigned int *)(o))
#define cpm_writel(b, o)    (*(volatile unsigned int *)(o)) = (b)
#ifdef CONFIG_SOC_X2500
	#define CPM_RSACDR      (0xb000004C)
#else
	#define CPM_RSACDR      (0xb0000050)
#endif
#define CPM_CLKGR0      (0xb0000020)
static int jz_rsa_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct jz_rsa *rsa;
	struct resource *res;
	unsigned int tmp;

	rsa = kzalloc(sizeof(struct jz_rsa), GFP_KERNEL);
	if (IS_ERR_OR_NULL(rsa)) {
		pr_err("Failed to alloc mem fir jz_rsa!\n");
		ret = -ENOMEM;
		goto err;
	}

	rsa->dev = &pdev->dev;

	rsa->clk_gate = devm_clk_get(&pdev->dev, "gate_rsa");
	if (IS_ERR(rsa->clk_gate)) {
		dev_err(&pdev->dev, "cannot find rsa clock\n");
		ret = PTR_ERR(rsa->clk_gate);
		goto err1;
	}
	clk_prepare_enable(rsa->clk_gate);

	/* rsa cguclk SACLK/2 */
	tmp = cpm_readl(CPM_RSACDR);
	tmp &= ~(2 << 30 | 0xf | 1 << 27);
	tmp |= 1 << 29;
	tmp |= 2 << 0;

	cpm_writel(tmp, CPM_RSACDR);
	while (cpm_readl(CPM_RSACDR) & (1 << 28));
	tmp &= ~(1 << 29);
	cpm_writel(tmp, CPM_RSACDR);
	cpm_writel(0, CPM_CLKGR0);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(res)) {
		dev_err(rsa->dev, "No iomem resource!\n");
		ret = -ENOMEM;
		goto err1;
	}

	rsa->irq = platform_get_irq(pdev, 0);
	if (rsa->irq < 0) {
		dev_err(&pdev->dev, "No irq resource!\n");
		ret = -ENOMEM;
		goto err1;
	}

	rsa->iomem = devm_ioremap(rsa->dev, res->start, resource_size(res));
	if (IS_ERR_OR_NULL(rsa->iomem)) {
		dev_err(&pdev->dev, "Failed to remap io resources!\n");
		ret = -ENOMEM;
		goto err1;
	}

	ret = devm_request_irq(&pdev->dev, rsa->irq, rsa_irq, IRQF_ONESHOT, dev_name(&pdev->dev), rsa);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request irq!\n");
		goto err_request_irq;
	}

	init_waitqueue_head(&rsa->wq);

	platform_set_drvdata(pdev, rsa);

	mutex_init(&rsa->mutex);

	rsa->mdev.minor = MISC_DYNAMIC_MINOR;
	rsa->mdev.name = DEV_NAME;
	rsa->mdev.fops = &rsa_misc_fops;
	ret = misc_register(&rsa->mdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register misc driver!\n");
		goto err_misc_register;
	}

	dev_info(&pdev->dev, "rsa driver probe ok!\n");

	return 0;

err_misc_register:
	mutex_destroy(&rsa->mutex);
err_request_irq:
	devm_iounmap(rsa->dev, rsa->iomem);
err1:
	kfree(rsa);
err:
	dev_err(rsa->dev, "initialization failed.\n");
	return ret;

}

static int jz_rsa_remove(struct platform_device *pdev)
{
	struct jz_rsa *rsa = platform_get_drvdata(pdev);

	misc_deregister(&rsa->mdev);
	devm_iounmap(rsa->dev, rsa->iomem);
	kfree(rsa);

	return 0;
}

static const struct of_device_id ingenic_rsa_dt_match[] = {
	{ .compatible = "ingenic,rsa", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, aic_dt_match);

static struct platform_driver ingenic_rsa_driver = {
	.probe  = jz_rsa_probe,
	.remove = jz_rsa_remove,
	.driver = {
		.name   = "rsa",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ingenic_rsa_dt_match),
	},
};

module_platform_driver(ingenic_rsa_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("mayuanjun <yuanjun.ma@ingenic.com>");
MODULE_DESCRIPTION("ingenic rsa driver");
