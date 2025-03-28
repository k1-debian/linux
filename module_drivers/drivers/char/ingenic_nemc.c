/* linux/drivers/char/ingenic_nemc.c
*
* Ingenic cdbus driver, use kernel char device framework
*
* Copyright (c) 2022 Ingenic
* Author:zhangxu <rui.zhao@ingenic.com>
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
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <asm/div64.h>

#include <asm/irq.h>
#include <asm/io.h>

#include "ingenic_nemc.h"
#include "../clk/ingenic-v2/clk.h"

#if CONFIG_SOC_X1600
	#define NEMC_SMC0Rn(n)       (0x14 + (((n) - 1) * 4))
	#define NEMC_SACRn(n)        (0x34 + (((n) - 1) * 4))
	#define NEMC_SMC1Rn(n)       (0x54 + (((n) - 1) * 4))

	/* SMCORi */
	#define NEMC_SMC0R_SMT_SHIFT     0
	#define NEMC_SMC0R_SMT_MASK  (0x3 << NEMC_SMC0R_SMT_SHIFT)
	#define NEMC_SMC0R_BL_SHIFT      2
	#define NEMC_SMC0R_BL_MASK       (0x3 << NEMC_SMC0R_BL_SHIFT)
	#define NEMC_SMC0R_BW_SHIFT      6
	#define NEMC_SMC0R_BW_MASK       (0x3 << NEMC_SMC0R_BW_SHIFT)
	#define NEMC_SMC0R_BW_8          (0 << 6)
	#define NEMC_SMC0R_BW_16         (1 << 6)
	#define NEMC_SMC0R_TWAS_SHIFT    8
	#define NEMC_SMC0R_TWAS_MASK     (0xf << NEMC_SMC0R_TWAS_SHIFT)
	#define NEMC_SMC0R_TRAS_SHIFT    12
	#define NEMC_SMC0R_TRAS_MASK     (0xf << NEMC_SMC0R_TRAS_SHIFT)
	#define NEMC_SMC0R_TBP_SHIFT     16
	#define NEMC_SMC0R_TBP_MASK      (0xf << NEMC_SMC0R_TBP_SHIFT)
	#define NEMC_SMC0R_TAW_SHIFT     20
	#define NEMC_SMC0R_TAW_MASK      (0xf << NEMC_SMC0R_TAW_SHIFT)
	#define NEMC_SMC0R_TCH_SHIFT     24
	#define NEMC_SMC0R_TCH_MASK      (0x3f << NEMC_SMC0R_TCH_SHIFT)

	/* SMC1R_i */
	#define NEMC_SMC1R_TAVDS_SHIFT       0
	#define NEMC_SMC1R_TAVDS_MASK        (0xf << NEMC_SMC1R_TAVDS_SHIFT)
	#define NEMC_SMC1R_TAVDP_SHIFT       4
	#define NEMC_SMC1R_TAVDP_MASK        (0xf << NEMC_SMC1R_TAVDP_SHIFT)
	#define NEMC_SMC1R_TAVDH_SHIFT       8
	#define NEMC_SMC1R_TAVDH_MASK        (0x3 << NEMC_SMC1R_TAVDH_SHIFT)
	#define NEMC_SMC1R_TWRRV_SHIFT       16
	#define NEMC_SMC1R_TWRRV_MASK        (0xf << NEMC_SMC1R_TWRRV_SHIFT)
	#define NEMC_SMC1R_TRDRV_SHIFT       20
	#define NEMC_SMC1R_TRDRV_MASK        (0xf << NEMC_SMC1R_TRDRV_SHIFT)
	#define NEMC_SMC1R_STRV_SHIFT        24
	#define NEMC_SMC1R_STRV_MASK         (0x3f << NEMC_SMC1R_STRV_SHIFT)
	#define NEMC_SMC1R_WP_EN_SHIFT       31
	#define NEMC_SMC1R_WP_EN_MASK        (0x1 << NEMC_SMC1R_WP_EN_SHIFT)
#else
	#define NEMC_SMCRn(n)       (0x14 + (((n) - 1) * 4))
	#define NEMC_SACRn(n)        (0x34 + (((n) - 1) * 4))

	#define NEMC_SMCR_SMT           BIT(0)
	#define NEMC_SMCR_BW_SHIFT      6
	#define NEMC_SMCR_BW_MASK       (0x3 << NEMC_SMCR_BW_SHIFT)
	#define NEMC_SMCR_BW_8          (0 << 6)
	#define NEMC_SMCR_BW_16         (1 << 6)
	#define NEMC_SMCR_TAS_SHIFT     8
	#define NEMC_SMCR_TAS_MASK      (0xf << NEMC_SMCR_TAS_SHIFT)
	#define NEMC_SMCR_TAH_SHIFT     12
	#define NEMC_SMCR_TAH_MASK      (0xf << NEMC_SMCR_TAH_SHIFT)
	#define NEMC_SMCR_TBP_SHIFT     16
	#define NEMC_SMCR_TBP_MASK      (0xf << NEMC_SMCR_TBP_SHIFT)
	#define NEMC_SMCR_TAW_SHIFT     20
	#define NEMC_SMCR_TAW_MASK      (0xf << NEMC_SMCR_TAW_SHIFT)
	#define NEMC_SMCR_TSTRV_SHIFT   24
	#define NEMC_SMCR_TSTRV_MASK    (0x3f << NEMC_SMCR_TSTRV_SHIFT)
#endif

struct ingenic_nemc {
	int major;
	int minor;
	dev_t dev_num;
	int nr_devs;

	struct class *class;
	struct cdev cdev;
	struct device *dev;
	struct clk *clk;
	void __iomem *iomem;

	uint32_t clk_period;
	unsigned long banks_present;
};

static uint32_t ingenic_nemc_clk_period(struct ingenic_nemc *nemc)
{
	unsigned long rate;
	unsigned long cycle;
	unsigned long long ns;
	rate = _get_rate("div_ahb2");
	if (!rate) {
		return 0;
	}
	ns = 1000000000ULL;
	do_div(ns, rate);
	cycle = ns;
	return cycle;

}

static uint32_t ingenic_nemc_ns_to_cycles(struct ingenic_nemc *nemc, uint32_t ns)
{
	return (ns + nemc->clk_period - 1) / nemc->clk_period;
}

#if CONFIG_SOC_X1600
static bool ingenic_nemc_configure_bank(struct ingenic_nemc *nemc,
                                        unsigned int bank,
                                        struct device_node *node)
{
	uint32_t smc0r, smc1r, val, cycles;

	/*
	 * Conversion of tBP、tAW、tRA tWA cycle counts to values supported by the
	 * hardware (round up to the next supported value).
	 */
	static const uint32_t convert_tBP_tAW[] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,

		/* 11 - 12 -> 12 cycles */
		11, 11,

		/* 13 - 15 -> 15 cycles */
		12, 12, 12,

		/* 16 - 20 -> 20 cycles */
		13, 13, 13, 13, 13,

		/* 21 - 25 -> 25 cycles */
		14, 14, 14, 14, 14,

		/* 26 - 31 -> 31 cycles */
		15, 15, 15, 15, 15, 15
	};

	smc0r = readl(nemc->iomem + NEMC_SMC0Rn(bank));
	smc0r &= ~NEMC_SMC0R_SMT_MASK;
	smc1r = readl(nemc->iomem + NEMC_SMC1Rn(bank));
	smc1r &= ~NEMC_SMC1R_STRV_MASK;

	if (!of_property_read_u32(node, "ingenic,nemc-bus-width", &val)) {
		smc0r &= ~NEMC_SMC0R_BW_MASK;
		switch (val) {
		case 8:
			smc0r |= NEMC_SMC0R_BW_8;
			break;
		case 16:
			smc0r |= NEMC_SMC0R_BW_16;
			break;
		default:
			dev_err(nemc->dev, "unsupported bus width: %u\n", val);
			return false;
		}
	}

	if (of_property_read_u32(node, "ingenic,nemc-tRAS", &val) == 0) {
		smc0r &= ~NEMC_SMC0R_TRAS_MASK;
		cycles = ingenic_nemc_ns_to_cycles(nemc, val);
		if (cycles > 31) {
			dev_err(nemc->dev, "tRAS %u is too high (%u cycles)\n",
			        val, cycles);
			return false;
		}

		smc0r |= convert_tBP_tAW[cycles] << NEMC_SMC0R_TRAS_SHIFT;
	}

	if (of_property_read_u32(node, "ingenic,nemc-tCH", &val) == 0) {
		smc0r &= ~NEMC_SMC0R_TCH_MASK;
		cycles = ingenic_nemc_ns_to_cycles(nemc, val);
		if (cycles > 15) {
			dev_err(nemc->dev, "tCH %u is too high (%u cycles)\n",
			        val, cycles);
			return false;
		}

		smc0r |= cycles << NEMC_SMC0R_TCH_SHIFT;
	}

	if (of_property_read_u32(node, "ingenic,nemc-tBP", &val) == 0) {
		smc0r &= ~NEMC_SMC0R_TBP_MASK;
		cycles = ingenic_nemc_ns_to_cycles(nemc, val);
		if (cycles > 31) {
			dev_err(nemc->dev, "tBP %u is too high (%u cycles)\n",
			        val, cycles);
			return false;
		}

		smc0r |= convert_tBP_tAW[cycles] << NEMC_SMC0R_TBP_SHIFT;
	}

	if (of_property_read_u32(node, "ingenic,nemc-tAW", &val) == 0) {
		smc0r &= ~NEMC_SMC0R_TAW_MASK;
		cycles = ingenic_nemc_ns_to_cycles(nemc, val);
		if (cycles > 31) {
			dev_err(nemc->dev, "tAW %u is too high (%u cycles)\n",
			        val, cycles);
			return false;
		}

		smc0r |= convert_tBP_tAW[cycles] << NEMC_SMC0R_TAW_SHIFT;
	}

	if (of_property_read_u32(node, "ingenic,nemc-tSTRV", &val) == 0) {
		smc1r &= ~NEMC_SMC1R_STRV_MASK;
		cycles = ingenic_nemc_ns_to_cycles(nemc, val);
		if (cycles > 63) {
			dev_err(nemc->dev, "tSTRV %u is too high (%u cycles)\n",
			        val, cycles);
			return false;
		}

		smc1r |= cycles << NEMC_SMC1R_STRV_SHIFT;
	}

	writel(smc0r, nemc->iomem + NEMC_SMC0Rn(bank));
	writel(smc1r, nemc->iomem + NEMC_SMC1Rn(bank));
	return true;
}
#else
static bool ingenic_nemc_configure_bank(struct ingenic_nemc *nemc,
                                        unsigned int bank,
                                        struct device_node *node)
{
	uint32_t smcr, val, cycles;

	/*
	 *Conversion of tBP and tAW cycle counts to values supported by the
	 *hardware (round up to the next supported value).
	 */
	static const uint32_t convert_tBP_tAW[] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,

		/* 11 - 12 -> 12 cycles */
		11, 11,

		/* 13 - 15 -> 15 cycles */
		12, 12, 12,

		/* 16 - 20 -> 20 cycles */
		13, 13, 13, 13, 13,

		/* 21 - 25 -> 25 cycles */
		14, 14, 14, 14, 14,

		/* 26 - 31 -> 31 cycles */
		15, 15, 15, 15, 15, 15
	};

	smcr = readl(nemc->iomem + NEMC_SMCRn(bank));
	smcr &= ~NEMC_SMCR_SMT;

	if (!of_property_read_u32(node, "ingenic,nemc-bus-width", &val)) {
		smcr &= ~NEMC_SMCR_BW_MASK;
		switch (val) {
		case 8:
			smcr |= NEMC_SMCR_BW_8;
			break;
		case 16:
			smcr |= NEMC_SMCR_BW_16;
			break;
		default:
			dev_err(nemc->dev, "unsupported bus width: %u\n", val);
		}
	}

	if (of_property_read_u32(node, "ingenic,nemc-tAS", &val) == 0) {
		smcr &= ~NEMC_SMCR_TAS_MASK;
		cycles = ingenic_nemc_ns_to_cycles(nemc, val);
		if (cycles > 15) {
			dev_err(nemc->dev, "tAS %u is too high (%u cycles)\n",
			        val, cycles);
			return false;
		}
		smcr |= cycles << NEMC_SMCR_TAS_SHIFT;
	}

	if (of_property_read_u32(node, "ingenic,nemc-tAH", &val) == 0) {
		smcr &= ~NEMC_SMCR_TAH_MASK;
		cycles = ingenic_nemc_ns_to_cycles(nemc, val);
		if (cycles > 15) {
			dev_err(nemc->dev, "tAH %u is too high (%u cycles)\n",
			        val, cycles);
			return false;
		}

		smcr |= cycles << NEMC_SMCR_TAH_SHIFT;
	}

	if (of_property_read_u32(node, "ingenic,nemc-tBP", &val) == 0) {
		smcr &= ~NEMC_SMCR_TBP_MASK;
		cycles = ingenic_nemc_ns_to_cycles(nemc, val);
		if (cycles > 31) {
			dev_err(nemc->dev, "tBP %u is too high (%u cycles)\n",
			        val, cycles);
			return false;
		}

		smcr |= convert_tBP_tAW[cycles] << NEMC_SMCR_TBP_SHIFT;
	}

	if (of_property_read_u32(node, "ingenic,nemc-tAW", &val) == 0) {
		smcr &= ~NEMC_SMCR_TAW_MASK;
		cycles = ingenic_nemc_ns_to_cycles(nemc, val);
		if (cycles > 31) {
			dev_err(nemc->dev, "tAW %u is too high (%u cycles)\n",
			        val, cycles);
			return false;
		}

		smcr |= convert_tBP_tAW[cycles] << NEMC_SMCR_TAW_SHIFT;
	}

	if (of_property_read_u32(node, "ingenic,nemc-tSTRV", &val) == 0) {
		smcr &= ~NEMC_SMCR_TSTRV_MASK;
		cycles = ingenic_nemc_ns_to_cycles(nemc, val);
		if (cycles > 63) {
			dev_err(nemc->dev, "tSTRV %u is too high (%u cycles)\n",
			        val, cycles);
			return false;
		}

		smcr |= cycles << NEMC_SMCR_TSTRV_SHIFT;
	}

	writel(smcr, nemc->iomem + NEMC_SMCRn(bank));
	return true;
}
#endif
static int ingenic_nemc_open(struct inode *inode, struct file *filp)
{
	return 0;
}
static int ingenic_nemc_read(struct file *filp, char *user_buf, size_t count, loff_t *f_pos)
{

	return 0;
}

static int ingenic_nemc_write(struct file *filp, const char *user_buf, size_t count, loff_t *f_pos)
{
	return 0;
}
static long ingenic_nemc_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{

	return 0;
}

static int ingenic_nemc_close(struct inode *inode, struct file *filp)
{

	return 0;
}

static struct file_operations ingenic_nemc_ops = {
	.owner = THIS_MODULE,
	.write = ingenic_nemc_write,
	.read = ingenic_nemc_read,
	.open = ingenic_nemc_open,
	.release = ingenic_nemc_close,
	.unlocked_ioctl = ingenic_nemc_ioctl,
};

static int ingenic_nemc_probe(struct platform_device *pdev)
{
	struct ingenic_nemc *nemc;
	struct resource *res;
	dev_t devno;
	struct clk *clk_cgu;
	struct clk *clk_gate;

	struct device_node *child;
	const __be32 *prop;
	unsigned int bank;
	unsigned long referenced;
	int i = 0;
	int ret = 0;

	nemc = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_nemc), GFP_KERNEL);
	if (!nemc) {
		return -ENOMEM;
	}

	nemc->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "can't get IORESOURCE_MEM\n");
		return -ENODEV;
	}

	nemc->iomem = devm_ioremap(nemc->dev, res->start, res->end - res->start + 1);
	if (!nemc->iomem) {
		dev_err(&pdev->dev, "ioremap failed!\n");
		return -ENOMEM;
	}

	nemc->clk_period = ingenic_nemc_clk_period(nemc);
	if (!nemc->clk_period) {
		dev_err(&pdev->dev, "failed to calculate clock period\n");
		return -EINVAL;
	}

	/*
	 *   Iterate over child devices, check that they do not conflict with
	 *   each other, and register child devices for them. If a child device
	 *   has invalid properties, it is ignored and no platform device is
	 *   registered for it.
	 */
	for_each_child_of_node(nemc->dev->of_node, child) {
		referenced = 0;
		i = 0;
		while ((prop = of_get_address(child, i++, NULL, NULL))) {
			bank = of_read_number(prop, 1);
			if (bank < 1 || bank >= INGENIC_NEMC_NUM_BANKS) {
				dev_err(nemc->dev,
				        "%s requests invalid bank %u\n",
				        child->full_name, bank);

				/* Will continue the outer loop below. */
				referenced = 0;
				break;
			}

			referenced |= BIT(bank);
		}
		if (!referenced) {
			dev_err(nemc->dev, "%s has no addresses\n",
			        child->full_name);
			continue;
		} else if (nemc->banks_present & referenced) {
			dev_err(nemc->dev, "%s conflicts with another node\n",
			        child->full_name);
			continue;
		}

		/* Configure bank parameters. */
		for_each_set_bit(bank, &referenced, INGENIC_NEMC_NUM_BANKS) {
			if (!ingenic_nemc_configure_bank(nemc, bank, child)) {
				referenced = 0;
				break;
			}
		}

		if (referenced) {
			if (of_platform_device_create(child, NULL, nemc->dev)) {
				nemc->banks_present |= referenced;
			}
		}
	}

	platform_set_drvdata(pdev, nemc);
	clk_gate = devm_clk_get(&pdev->dev, "gate_nemc");
	if (IS_ERR(clk_gate)) {
		dev_err(&pdev->dev, "%s Cannot get clock: %s\n", __func__, "gate_nemc");
		goto __err0;
	}
	if ((ret = clk_prepare_enable(clk_gate)) < 0) {
		dev_err(&pdev->dev, "Enable gate_nemc clk failed\n");
		goto __err0;
	}

	nemc->minor = 0;
	nemc->nr_devs = 1;
	ret = alloc_chrdev_region(&devno, nemc->minor, nemc->nr_devs, "ingenic-nemc");
	if (ret) {
		dev_err(nemc->dev, "alloc chrdev failde\n");
		return ret;
	}
	nemc->major = MAJOR(devno);
	nemc->dev_num = MKDEV(nemc->major, nemc->minor);

	cdev_init(&nemc->cdev, &ingenic_nemc_ops);
	nemc->cdev.owner = THIS_MODULE;
	ret = cdev_add(&nemc->cdev, nemc->dev_num, 1);
	if (ret) {
		dev_err(&pdev->dev, "cdev_add failed\n");
		goto __err1;
	}

	nemc->class = class_create(THIS_MODULE, "ingenic-nemc");
	if (IS_ERR(nemc->class)) {
		dev_err(nemc->dev, "class_create failed");
		ret = PTR_ERR(nemc->class);
		goto __err2;
	}

	nemc->dev = device_create(nemc->class, NULL, nemc->dev_num, NULL, "nemc");
	if (IS_ERR(nemc->dev)) {
		dev_err(nemc->dev, "device_create failed");
		ret = PTR_ERR(nemc->dev);
		goto __err3;
	}
	dev_info(nemc->dev, "Ingenic nemc driver probe success\n");

	return 0;
__err3:
	class_destroy(nemc->class);
__err2:
	cdev_del(&nemc->cdev);
__err1:
	unregister_chrdev_region(nemc->dev_num, nemc->nr_devs);
__err0:
	kfree(nemc);
	return ret;
}

static int ingenic_nemc_remove(struct platform_device *pdev)
{
	struct ingenic_nemc *nemc = platform_get_drvdata(pdev);

	device_destroy(nemc->class, nemc->dev_num);
	class_destroy(nemc->class);
	cdev_del(&nemc->cdev);
	unregister_chrdev_region(nemc->dev_num, nemc->nr_devs);

	return 0;
}

static const struct of_device_id ingenic_nemc_match[] = {
	{ .compatible = "ingenic,x1600-nemc", },
	{ .compatible = "ingenic,x2000-nemc", },
	{ .compatible = "ingenic,m300-nemc", },
	{}
};
MODULE_DEVICE_TABLE(of, ingenic_nemc_match);

static struct platform_driver ingenic_nemc_driver = {
	.probe          = ingenic_nemc_probe,
	.remove         = ingenic_nemc_remove,
	.driver     = {
		.name   = "ingenic-nemc",
		.owner  = THIS_MODULE,
		.of_match_table = ingenic_nemc_match,
	},
};

static int __init ingenic_nemc_init(void)
{
	int ret;

	ret = platform_driver_register(&ingenic_nemc_driver);
	if (ret) {
		platform_driver_unregister(&ingenic_nemc_driver);
	}

	return ret;
}

static void __exit ingenic_nemc_exit(void)
{
	platform_driver_unregister(&ingenic_nemc_driver);
}

module_init(ingenic_nemc_init);
module_exit(ingenic_nemc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Ingenic nemc driver");
MODULE_AUTHOR("rui.zhao@ingenic.com");
