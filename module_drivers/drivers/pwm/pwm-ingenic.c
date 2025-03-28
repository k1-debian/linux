/*
 * Copyright (c) 2007 Ben Dooks
 * Copyright (c) 2008 Simtec Electronics
 *     Ben Dooks <ben@simtec.co.uk>, <ben-linux@fluff.org>
 * Copyright (c) 2013 Tomasz Figa <tomasz.figa@gmail.com>
 *
 * PWM driver for Samsung SoCs
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/time.h>

#include <linux/mfd/core.h>

#ifdef CONFIG_SOC_X2500
	#include <mfd/ingenic-tcu_v1.h>
#else
	#include <linux/mfd/ingenic-tcu.h>
#endif
/**
 * struct ingenic_pwm_channel - private data of PWM channel
 * @period_ns:  current period in nanoseconds programmed to the hardware
 * @duty_ns:    current duty time in nanoseconds programmed to the hardware
 */
struct ingenic_pwm_channel {
	struct ingenic_tcu_chn *chan;
	u32 period_ns;
	u32 duty_ns;
};

/**
 * struct ingenic_pwm_chip - private data of PWM chip
 * @chip:       generic PWM chip
 * @rtc_clk:        external RTC clock(can be ERR_PTR if not present)
 * @ext_clk:        external EXT clock(can be ERR_PTR if not present)
 * @output_mask:    output status for all channels - one bit per channel
 */
struct ingenic_pwm_chip {
	struct pwm_chip chip;
	struct clk *rtc_clk;
	struct clk *ext_clk;
	u32 output_mask;
};

static inline
struct ingenic_pwm_chip *to_ingenic_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct ingenic_pwm_chip, chip);
}
static inline
struct ingenic_tcu_chn *to_ingenic_tcu_channel(struct pwm_device *pwm)
{
	return ((struct ingenic_pwm_channel *)pwm_get_chip_data(pwm))->chan;
}

static int pwm_get_period_num(struct ingenic_pwm_chip *chip,
                              struct ingenic_tcu_chn *tcu_chan, int period_ns)
{
	unsigned long long rate;
	unsigned long long tmp;
	int div = 0;
	int period = 0;

	if (tcu_chan->clk_src == TCU_CLKSRC_EXT) {
		rate = (unsigned long long)clk_get_rate(chip->ext_clk);
	} else if (tcu_chan->clk_src == TCU_CLKSRC_RTC) {
		rate = (unsigned long long)clk_get_rate(chip->ext_clk);
	} else {
		printk("not support clk soruce\n");
		return -EINVAL;
	}

	tmp =  rate * period_ns;
	do_div(tmp, 1000000000);
	period = tmp;

	while (period > 0xffff && div < 6) {
		period >>= 2;
		++div;
	}

	if (div == 6) {
		return -EINVAL;
	}

	tcu_chan->full_num = period;
	tcu_chan->clk_div = div;
	return period;
}
static int pwm_ingenic_get_period(struct ingenic_pwm_chip *chip,
                                  struct pwm_device *pwm, int period_ns)
{
	struct ingenic_tcu_chn *tcu_chan;
	int period;

	tcu_chan = to_ingenic_tcu_channel(pwm);
	/*
	 * Compare minimum PWM frequency that can be achieved with possible
	 * divider settings and choose the lowest divisor that can generate
	 * frequencies lower than requested.
	 */
	tcu_chan->clk_src = TCU_CLKSRC_EXT;
	period = pwm_get_period_num(chip, tcu_chan, period_ns);
	if (period < 0) {
		tcu_chan->clk_src = TCU_CLKSRC_RTC;
		period = pwm_get_period_num(chip, tcu_chan, period_ns);
		if (period < 0) {
			dev_err(chip->chip.dev, "can not find right div\n");
		}
	}

	return period;
}

static int pwm_ingenic_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ingenic_pwm_chip *our_chip = to_ingenic_pwm_chip(chip);
	struct ingenic_pwm_channel *our_chan;
	struct mfd_cell *cell;

	if (!(our_chip->output_mask & BIT(pwm->hwpwm))) {
		dev_warn(chip->dev,
		         "tried to request PWM channel %d without output\n",
		         pwm->hwpwm);
		return -EINVAL;
	}

	cell = request_cell(pwm->hwpwm);
	if (!cell) {
		dev_err(chip->dev, "tried to request tcu channel %d busy\n",
		        pwm->hwpwm);
		return -EINVAL;
	}

	our_chan = devm_kzalloc(chip->dev, sizeof(*our_chan), GFP_KERNEL);
	if (!our_chan) {
		return -ENOMEM;
	}

	our_chan->chan = cell->platform_data;
	our_chan->chan->enable(cell->id);

	pwm_set_chip_data(pwm, our_chan);

	return 0;
}

static void pwm_ingenic_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ingenic_pwm_channel *our_chan;
	int id;

	our_chan = pwm_get_chip_data(pwm);

	id = pwm->hwpwm;
	our_chan->chan->disable(id);
	free_cell(id);
	devm_kfree(chip->dev, our_chan);
	pwm_set_chip_data(pwm, NULL);
}

static int pwm_ingenic_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ingenic_tcu_chn *tcu_chan;
	tcu_chan = to_ingenic_tcu_channel(pwm);
	ingenic_tcu_counter_begin(tcu_chan);

	return 0;
}

static void pwm_ingenic_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ingenic_tcu_chn *tcu_chan;
	tcu_chan = to_ingenic_tcu_channel(pwm);
	ingenic_tcu_counter_stop(tcu_chan);
}

static int pwm_ingenic_config(struct pwm_chip *chip, struct pwm_device *pwm,
                              int duty_ns, int period_ns)
{
	struct ingenic_pwm_chip *our_chip = to_ingenic_pwm_chip(chip);
	struct ingenic_pwm_channel *our_chan = pwm_get_chip_data(pwm);
	struct ingenic_tcu_chn *tcu_chan = our_chan->chan;
	u16 tfull = tcu_chan->full_num, thalf;

	/*
	 * We currently avoid using 64bit arithmetic by using the
	 * fact that anything faster than 1Hz is easily representable
	 * by 32bits.
	 */
	if (period_ns > NSEC_PER_SEC) {
		return -ERANGE;
	}

	if (period_ns == our_chan->period_ns && duty_ns == our_chan->duty_ns) {
		return 0;
	}

	/* Check to see if we are changing the clock rate of the PWM. */
	if (our_chan->period_ns != period_ns) {
		int period;

		period = pwm_ingenic_get_period(our_chip, pwm, period_ns);
		dev_dbg(our_chip->chip.dev, "duty_ns=%d, period_ns=%d (%u)\n",
		        duty_ns, period_ns, period);
		if (period < 0) {
			return -ERANGE;
		}
		tfull = period;
	}
	/* Period is too short. */
	if (tfull <= 1) {
		return -ERANGE;
	}

	/* Note that counters count down. */
	{
		unsigned long long tmp;
		tmp = (unsigned long long)tfull * duty_ns;
		do_div(tmp, period_ns);
		thalf = tmp;
	}

	/* 0% duty is not available */
	if (!thalf) {
		++thalf;
	}

	thalf = tfull - thalf;
	if (!thalf) {
		/* set pwm output 1 */
	}

	dev_info(our_chip->chip.dev,
	         "thalf=%u/%u\n", thalf, tfull);

	tcu_chan->irq_type = NULL_IRQ_MODE;
	tcu_chan->is_pwm = 1;
	tcu_chan->full_num = tfull;
	tcu_chan->half_num = thalf;
	tcu_chan->shutdown_mode = 0;

	ingenic_tcu_config(tcu_chan);

	our_chan->period_ns = period_ns;
	our_chan->duty_ns = duty_ns;

	return 0;
}

static int pwm_ingenic_set_polarity(struct pwm_chip *chip,
                                    struct pwm_device *pwm,
                                    enum pwm_polarity polarity)
{
	struct ingenic_tcu_chn *tcu_chan;
	bool invert = (polarity == PWM_POLARITY_NORMAL);

	tcu_chan = to_ingenic_tcu_channel(pwm);
	if (invert) {
		tcu_chan->init_level = 1;
	} else {
		tcu_chan->init_level = 0;
	}

	return 0;
}

static int pwm_ingenic_apply(struct pwm_chip *chip, struct pwm_device *pwm,
                             const struct pwm_state *state)
{
	int err;
	bool enabled = pwm->state.enabled;

	if (state->polarity != pwm->state.polarity) {
		if (enabled) {
			pwm_ingenic_disable(chip, pwm);
			enabled = false;
		}
		err = pwm_ingenic_set_polarity(chip, pwm, state->polarity);
		if (err) {
			return err;
		}
	}

	if (!state->enabled) {
		if (enabled) {
			pwm_ingenic_disable(chip, pwm);
		}
		return 0;
	}

	err = pwm_ingenic_config(chip, pwm, state->duty_cycle, state->period);
	if (err) {
		return err;
	}

	if (!enabled) {
		return pwm_ingenic_enable(chip, pwm);
	}
	return 0;
}

static const struct pwm_ops pwm_ingenic_ops = {
	.request    = pwm_ingenic_request,
	.free       = pwm_ingenic_free,
	.apply      = pwm_ingenic_apply,
	.owner      = THIS_MODULE,
};

#ifdef CONFIG_OF
/**
 * of_irq_find_parent - Given a device node, find its interrupt parent node
 * @child: pointer to device node
 *
 * Returns a pointer to the interrupt parent node, or NULL if the interrupt
 * parent could not be determined.
 */
struct device_node *of_pwm_find_parent(struct device_node *child)
{
	struct device_node *p;
	const __be32 *parp;

	if (!of_node_get(child)) {
		return NULL;
	}

	do {
		parp = of_get_property(child, "ingenic,timer-parent", NULL);
		if (parp == NULL) {
			p = of_get_parent(child);
		} else {
			p = of_find_node_by_phandle(be32_to_cpup(parp));
		}
		of_node_put(child);
	} while (!p);

	return p;
}

static void pwm_ingenic_parse_dt(struct ingenic_pwm_chip *chip)
{
	struct device_node *np = chip->chip.dev->of_node;
	struct device_node *child, *parent;
	int enable_pwm_num = 0;

	for_each_child_of_node(np, child) {
		const char *status;
		parent = of_pwm_find_parent(child);
		of_property_read_string(child, "status", &status);

		if (!strcmp(status, "okay")) {
			unsigned int info, id;
			of_property_read_u32(parent, "ingenic,channel-info", &info);
			id = info & (CHANNEL_BASE_OFF * 2 - 1);
			chip->output_mask |= BIT(id);
			enable_pwm_num ++;
		}
	}
	chip->chip.of_pwm_n_cells = enable_pwm_num;
}
static const struct of_device_id ingenic_pwm_matches[] = {
	{ .compatible = "ingenic,pwm", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, ingenic_pwm_matches);
#else
static int pwm_ingenic_parse_dt(struct ingenic_pwm_chip *chip)
{
	return -ENODEV;
}
#endif

static int pwm_ingenic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ingenic_pwm_chip *chip;
	struct device_node *np;
	unsigned int pwmnum;
	int ret = 0;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (chip == NULL) {
		return -ENOMEM;
	}

	np = pdev->dev.of_node;
	pwmnum = of_get_child_count(np);
	chip->chip.dev = &pdev->dev;
	chip->chip.dev->of_node = np;
	chip->chip.ops = &pwm_ingenic_ops;
	chip->chip.base = -1;
	chip->chip.npwm = pwmnum;

	if (IS_ENABLED(CONFIG_OF) && pdev->dev.of_node) {
		pwm_ingenic_parse_dt(chip);
		/* chip->chip.of_xlate = of_pwm_xlate_with_flags; */
	} else {
		if (!pdev->dev.platform_data) {
			dev_err(&pdev->dev, "no platform data specified\n");
			return -EINVAL;
		}
	}

	chip->ext_clk = devm_clk_get(&pdev->dev, "ext");
#ifdef CONFIG_SOC_X2500
	chip->rtc_clk = devm_clk_get(&pdev->dev, "rtc_ext");
#else
	chip->rtc_clk = devm_clk_get(&pdev->dev, "rtc");
#endif
	if (IS_ERR(chip->ext_clk) || IS_ERR(chip->rtc_clk)) {
		dev_err(dev, "failed to get timer base clk\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, chip);

	ret = pwmchip_add(&chip->chip);
	if (ret < 0) {
		dev_err(dev, "failed to register PWM chip\n");
		return ret;
	}

	return 0;
}

static int pwm_ingenic_remove(struct platform_device *pdev)
{
	struct ingenic_pwm_chip *chip = platform_get_drvdata(pdev);

	pwmchip_remove(&chip->chip);

	clk_disable_unprepare(chip->ext_clk);
	clk_disable_unprepare(chip->rtc_clk);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_ingenic_suspend(struct device *dev)
{
	struct ingenic_pwm_chip *chip = dev_get_drvdata(dev);
	unsigned int i;

	/*
	 * No one preserves these values during suspend so reset them.
	 * Otherwise driver leaves PWM unconfigured if same values are
	 * passed to pwm_config() next time.
	 */
	for (i = 0; i < chip->chip.npwm; ++i) {
		struct pwm_device *pwm = &chip->chip.pwms[i];
		struct ingenic_pwm_channel *chan = pwm_get_chip_data(pwm);
		int id;
		if (!chan) {
			continue;
		}

		id = pwm->hwpwm;
		chan->chan->disable(id);
	}

	return 0;
}

static int pwm_ingenic_resume(struct device *dev)
{
	struct ingenic_pwm_chip *chip = dev_get_drvdata(dev);
	unsigned int chan;

	/*
	 * Inverter setting must be preserved across suspend/resume
	 * as nobody really seems to configure it more than once.
	 */
	for (chan = 0; chan < chip->chip.npwm; ++chan) {
		if (chip->output_mask & BIT(chan)) {
			struct pwm_device *pwm = &chip->chip.pwms[chan];
			struct ingenic_pwm_channel *chan = pwm_get_chip_data(pwm);
			int id;
			if (!chan) {
				continue;
			}

			id = pwm->hwpwm;
			chan->chan->enable(id);
		}
	}

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_ingenic_pm_ops, pwm_ingenic_suspend,
                         pwm_ingenic_resume);

static struct platform_driver pwm_ingenic_driver = {
	.driver     = {
		.name   = "ingenic,pwm",
		.pm = &pwm_ingenic_pm_ops,
		.of_match_table = of_match_ptr(ingenic_pwm_matches),
	},
	.probe      = pwm_ingenic_probe,
	.remove     = pwm_ingenic_remove,
};

static int __init ingenic_pwm_init(void)
{
	return platform_driver_register(&pwm_ingenic_driver);
}

static void __exit ingenic_pwm_exit(void)
{
	platform_driver_unregister(&pwm_ingenic_driver);
}

device_initcall_sync(ingenic_pwm_init);
module_exit(ingenic_pwm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("bo.liu <bo.liu@ingenic.com>");
MODULE_ALIAS("platform:ingenic-pwm");
