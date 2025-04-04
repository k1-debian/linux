#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#define PARENT_APLL     1
#define PARENT_MPLL     2
#define BIU_FREQ_THRESHOLD  750000000

struct ingenic_cpufreq {
	struct device *dev;
	unsigned int latency;
	struct cpufreq_frequency_table *freq_table_dt;
	struct cpufreq_frequency_table *freq_table;
	unsigned int freq_count;
	struct mutex mutex;

	struct clk *mux;
	struct clk *mclk;
	struct clk *sclka;
	struct clk *clk_cpu_l2c_x1;
	struct clk *clk_cpu_l2c_x2;
	unsigned int arate;
	unsigned int mrate;

	struct regulator *regulator;
};

static struct ingenic_cpufreq *ingenic_cpufreq;

static unsigned long lpj_ref;
static int ingenic_cpu_freq_notifier(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs *freqs =  data;
	static unsigned int lpj_ref_freq;

	if (val == CPUFREQ_PRECHANGE) {
		lpj_ref = loops_per_jiffy;
	}

	if (val == CPUFREQ_POSTCHANGE) {
		lpj_ref_freq = freqs->old;
		loops_per_jiffy = cpufreq_scale(lpj_ref, lpj_ref_freq, freqs->new);
	}

	return NOTIFY_OK;
}

static struct notifier_block ingenic_cpufreq_notifier_block = {
	.notifier_call = ingenic_cpu_freq_notifier,
};

static void ingenic_cpufreq_get_freq_table(void)
{
	unsigned int a_rate = ingenic_cpufreq->arate;
	unsigned int m_rate = ingenic_cpufreq->mrate;
	unsigned int div[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
	unsigned int count = 0, dt_freq;
	int i, j;

	for (i = 0; i < ingenic_cpufreq->freq_count; i++) {
		dt_freq = ingenic_cpufreq->freq_table_dt[i].frequency * 1000;
		for (j = 0; j < 16; j++) {
			if (dt_freq == a_rate / div[j]) {
				ingenic_cpufreq->freq_table[count].frequency = ingenic_cpufreq->freq_table_dt[i].frequency;
				ingenic_cpufreq->freq_table[count].driver_data = PARENT_APLL;
				ingenic_cpufreq->freq_table[count].flags = ingenic_cpufreq->freq_table_dt[i].flags;
				count++;
				break;
			}
			if (dt_freq == m_rate / div[j]) {
				ingenic_cpufreq->freq_table[count].frequency = ingenic_cpufreq->freq_table_dt[i].frequency;
				ingenic_cpufreq->freq_table[count].driver_data = PARENT_MPLL;
				ingenic_cpufreq->freq_table[count].flags = ingenic_cpufreq->freq_table_dt[i].flags;
				count++;
				break;
			}
		}
	}
	ingenic_cpufreq->freq_count = count;

	ingenic_cpufreq->freq_table[count].driver_data = count;
	ingenic_cpufreq->freq_table[count].frequency = CPUFREQ_TABLE_END;

}

static int ingenic_cpufreq_init(struct cpufreq_policy *policy)
{
	ingenic_cpufreq->mclk = clk_get(NULL, "mpll");
	ingenic_cpufreq->sclka = clk_get(NULL, "sclka");
	ingenic_cpufreq->mux = clk_get(NULL, "mux_cpu_l2c");
	ingenic_cpufreq->clk_cpu_l2c_x1 = clk_get(NULL, "div_cpu_l2c_x1");
	ingenic_cpufreq->clk_cpu_l2c_x2 = clk_get(NULL, "div_cpu_l2c_x2");

	ingenic_cpufreq->arate = clk_get_rate(ingenic_cpufreq->sclka);
	ingenic_cpufreq->mrate = clk_get_rate(ingenic_cpufreq->mclk);
	ingenic_cpufreq_get_freq_table();

	if (ingenic_cpufreq->arate > BIU_FREQ_THRESHOLD) {
		policy->clk = ingenic_cpufreq->clk_cpu_l2c_x2;
	} else {
		policy->clk = ingenic_cpufreq->clk_cpu_l2c_x1;
	}

	ingenic_cpufreq->latency = 4000;

	cpufreq_generic_init(policy, ingenic_cpufreq->freq_table, ingenic_cpufreq->latency);
	return 0;
}

static int ingenic_cpufreq_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct cpufreq_frequency_table *freq_table = ingenic_cpufreq->freq_table;
	unsigned long freq_new, freq_old;
	unsigned long freq_round;
	struct clk *mux, *sclka, *mclk, *clk_cpu_l2c;
	unsigned int a_rate, m_rate, parent;
	struct dev_pm_opp *opp;
	unsigned long v;
	mutex_lock(&ingenic_cpufreq->mutex);
	freq_new = freq_table[index].frequency * 1000;
	freq_old = clk_get_rate(policy->clk);

	if (freq_old == freq_new) {
		mutex_unlock(&ingenic_cpufreq->mutex);
		return 0;
	}

	parent = freq_table[index].driver_data;
	mclk = ingenic_cpufreq->mclk;
	sclka = ingenic_cpufreq->sclka;
	mux = ingenic_cpufreq->mux;

	a_rate = ingenic_cpufreq->arate;
	m_rate = ingenic_cpufreq->mrate;

	if (freq_new > BIU_FREQ_THRESHOLD) {
		clk_cpu_l2c = ingenic_cpufreq->clk_cpu_l2c_x2;
	} else {
		clk_cpu_l2c = ingenic_cpufreq->clk_cpu_l2c_x1;
	}
	clk_get_rate(clk_cpu_l2c);

#ifdef  CONFIG_REGULATOR
	freq_round = clk_round_rate(clk_cpu_l2c, freq_new);

	opp = dev_pm_opp_find_freq_ceil(ingenic_cpufreq->dev, &freq_round);
	v = dev_pm_opp_get_voltage(opp);

	if (freq_round > freq_old) {
		regulator_set_voltage(ingenic_cpufreq->regulator, v, v);
		clk_set_rate(clk_cpu_l2c, freq_round);
	}
	if (freq_round < freq_old) {
		clk_set_rate(clk_cpu_l2c, freq_round);
		regulator_set_voltage(ingenic_cpufreq->regulator, v, v);
	}
#else
	clk_set_rate(clk_cpu_l2c, freq_new);
#endif

	if (parent == PARENT_APLL) {
		clk_set_parent(mux, sclka);
	} else if (parent == PARENT_MPLL) {
		clk_set_parent(mux, mclk);
	} else {
		printk("wrong clk parent\n");
	}

#ifdef  CONFIG_REGULATOR
	opp = dev_pm_opp_find_freq_exact(ingenic_cpufreq->dev, freq_new, 1);
	v = dev_pm_opp_get_voltage(opp);

	if (freq_new >= freq_round) {
		regulator_set_voltage(ingenic_cpufreq->regulator, v, v);
		clk_set_rate(clk_cpu_l2c, freq_new);
	}
	if (freq_new < freq_round) {
		clk_set_rate(clk_cpu_l2c, freq_new);
		regulator_set_voltage(ingenic_cpufreq->regulator, v, v);
	}
#else
	clk_set_rate(clk_cpu_l2c, freq_new);
#endif

	policy->clk = clk_cpu_l2c;
	policy->cur = freq_new / 1000;

#ifdef  CONFIG_REGULATOR
	if ((freq_new != clk_get_rate(clk_cpu_l2c)) || (v != regulator_get_voltage(ingenic_cpufreq->regulator))) {
		printk("warning : ");
		printk("from %ldKHZ to %ldKHZ(%lduV)  now: %ldKHZ(%duV)\n", freq_old / 1000, freq_new / 1000, v, clk_get_rate(clk_cpu_l2c) / 1000, regulator_get_voltage(ingenic_cpufreq->regulator));
	}
#else
	if (freq_new != clk_get_rate(clk_cpu_l2c)) {
		printk("warning : ");
		printk("from %ldKHZ to %ldKHZ  now: %ldKHZ\n", freq_old / 1000, freq_new / 1000, clk_get_rate(clk_cpu_l2c) / 1000);
	}
#endif

	mutex_unlock(&ingenic_cpufreq->mutex);

	return 0;
}

static int ingenic_cpufreq_suspend(struct cpufreq_policy *policy)
{
	return 0;
}

static int ingenic_cpufreq_resume(struct cpufreq_policy *policy)
{
	return 0;
}

static struct cpufreq_driver ingenic_cpufreq_driver = {
	.name       = "ingenic-cpufreq",
	.flags      = CPUFREQ_STICKY | CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.init       = ingenic_cpufreq_init,
	.verify     = cpufreq_generic_frequency_table_verify,
	.target_index   = ingenic_cpufreq_target,
	.get        = cpufreq_generic_get,
	.suspend    = ingenic_cpufreq_suspend,
	.resume     = ingenic_cpufreq_resume,
	.attr       = cpufreq_generic_attr,
};

static const struct of_device_id ingenic_cpufreq_match[] = {
	{.compatible = "ingenic,x2000-cpufreq",},
	{.compatible = "ingenic,m300-cpufreq",},
	{.compatible = "ingenic,x2500-cpufreq",},
	{},
};

MODULE_DEVICE_TABLE(of, ingenic_cpufreq_match);

static int ingenic_cpufreq_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *np;

	np = pdev->dev.of_node;
	if (!np) {
		return -ENODEV;
	}

	ingenic_cpufreq = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_cpufreq), GFP_KERNEL);
	if (!ingenic_cpufreq) {
		ret = -ENOMEM;
		goto err_put_node;
	}

	ingenic_cpufreq->dev = &pdev->dev;

	ret = dev_pm_opp_of_add_table(ingenic_cpufreq->dev);
	if (ret) {
		dev_err(ingenic_cpufreq->dev, "failed to init OPP table : %d\n", ret);
		goto err_put_node;
	}

	ret = dev_pm_opp_init_cpufreq_table(ingenic_cpufreq->dev, &ingenic_cpufreq->freq_table_dt);
	if (ret) {
		dev_err(ingenic_cpufreq->dev, "failed to init cpufreq table : %d\n", ret);
		goto err_free_opp;
	}

	ingenic_cpufreq->freq_count = dev_pm_opp_get_opp_count(ingenic_cpufreq->dev);

	ingenic_cpufreq->freq_table = kcalloc(ingenic_cpufreq->freq_count + 1, sizeof(struct cpufreq_frequency_table), GFP_ATOMIC);

#ifdef  CONFIG_REGULATOR
	ingenic_cpufreq->regulator = devm_regulator_get(ingenic_cpufreq->dev, "cpu-core");
	if (!ingenic_cpufreq->regulator) {
		printk("cannot get regulator for cpufreq\n");
	}

#endif

	mutex_init(&ingenic_cpufreq->mutex);

	cpufreq_register_notifier(&ingenic_cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);

	ret = cpufreq_register_driver(&ingenic_cpufreq_driver);
	if (ret) {
		dev_err(ingenic_cpufreq->dev, "%s : failed to register cpufreq driver\n", __func__);
		goto err_free_table;
	}

	of_node_put(np);

	printk("cpufreq register succeed\n");

	return 0;

err_free_table:
	dev_pm_opp_free_cpufreq_table(ingenic_cpufreq->dev, &ingenic_cpufreq->freq_table_dt);

err_free_opp:
	dev_pm_opp_of_remove_table(ingenic_cpufreq->dev);

err_put_node:
	of_node_put(np);
	dev_err(&pdev->dev, "%s : failed initialization\n", __func__);

	return ret;
}

static int ingenic_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&ingenic_cpufreq_driver);
	cpufreq_unregister_notifier(&ingenic_cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);

	return 0;
}

static struct platform_driver ingenic_cpufreq_platdrv = {
	.driver = {
		.name = "ingenic-cpufreq",
		.of_match_table = ingenic_cpufreq_match,
	},
	.probe  = ingenic_cpufreq_probe,
	.remove = ingenic_cpufreq_remove,
};
module_platform_driver(ingenic_cpufreq_platdrv);

MODULE_AUTHOR("Liu Si Hui <sihui.liu@ingenic.com>");
MODULE_DESCRIPTION("ingenic x2000-v12 cpufreq driver");
MODULE_LICENSE("GPL");
