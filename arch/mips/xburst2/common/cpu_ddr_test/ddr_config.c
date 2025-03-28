#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/syscore_ops.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <asm/idle.h>
#include <asm/mipsregs.h>
#include <asm/cacheops.h>
#include <linux/interrupt.h>

#include <soc/ddr.h>

#include <ccu.h>
#include "cpu_tcsm.h"

#define GMAC_MSC_CHANNEL 0  //DDR ch0
#define VPU_CHANNEL      1  //DDR ch1
#define DPU_VPU_CHANNEL  3  //DDR ch3
#define AHB2_CHANNEL     5  //DDR ch5
#define CPU_CHANNEL      6  //DDR ch6

char dri_ohm_list[32][9] = {
	"Infinity", "481",      "240.5",    "160.3",    "120.2",    "96.19",    "80.16",    "68.71",    "60.12",    "53.44",    "48.1",
	"43.72",    "40.08",    "37",       "34.35",    "32.06",    "60.12",    "53.44",    "48.1", "43.72",    "40.08",    "37",
	"34.35",    "32.06",    "30.06",    "28.29",    "26.72",    "25.31",    "24.05",    "22.9", "21.86",    "20.91",
};

char odt_ohm_list[32][9] = {
	"Infinity", "679",  "414.3",    "257.5",    "207.2",    "158.7",    "138.1",    "114.8",    "103.6",    "89.87",    "82.86",
	"73.85",    "69.05",    "62.68",    "59.19",    "54.44",    "103.6",    "89.87",    "82.86",    "73.85",    "69.05",    "62.68",
	"59.19",    "54.44",    "51.79",    "48.12",    "46.03",    "43.11",    "41.43",    "39.05",    "37.66",    "35.68",
};

struct ddr_statistics {
	struct dentry *ddr_debugfs;
	void *ddr_priv;
};

struct ddrc_chan {
	int chn;
	const char *desc;
};

static struct ddrc_chan ddrc_chans[] = {
	{0, "gmac&msc"},
	{1, "vpu&isp"},
	{3, "dpu&cim"},
	{5, "ahb2&audio&apb"},
	{6, "cpu"}
};

static struct ddr_statistics ddr_statistics;

static ssize_t ddr_read_protection(char *buf, unsigned int addr)
{
	unsigned int tmp;

	tmp = ddr_readl(addr);
	if (!(tmp & 1)) {
		strcpy(buf, "disable\n");
		return 8;
	} else if (tmp & 2) {
		strcpy(buf, "enable, un-protected register write occurred\n");
		return 46;
	} else {
		strcpy(buf, "enable, un-protected register write not occurred\n");
		return 50;
	}
}

static ssize_t ddr_read_ahb(struct file *file, char __user *user_buf,
                            size_t count, loff_t *ppos)
{
	char buf[50];
	return simple_read_from_buffer(user_buf, count, ppos, buf, ddr_read_protection(buf, DDRC_HREGPRO));
}

static ssize_t ddr_write_ahb(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
	unsigned int tmp;
	char buf[16];
	if (copy_from_user(buf, user_buf, min(count, (sizeof(buf) - 1)))) {
		return -EFAULT;
	}
	tmp = simple_strtoul(buf, NULL, 10) ? 1 : 0;
	ddr_writel(tmp, DDRC_HREGPRO);

	return count;
}

static ssize_t ddr_read_apb(struct file *file, char __user *user_buf,
                            size_t count, loff_t *ppos)
{
	char buf[50];
	return simple_read_from_buffer(user_buf, count, ppos, buf, ddr_read_protection(buf, DDRC_PREGPRO));
}

static ssize_t ddr_write_apb(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
	unsigned int tmp;
	char buf[16];
	if (copy_from_user(buf, user_buf, min(count, (sizeof(buf) - 1)))) {
		return -EFAULT;
	}
	tmp = simple_strtoul(buf, NULL, 10) ? 1 : 0;
	ddr_writel(tmp, DDRC_PREGPRO);

	return count;
}

static ssize_t ddr_read_strength(struct file *file, char __user *user_buf,
                                 size_t count, loff_t *ppos)
{
	char buf[50];
	int pos = 0;
	int tmp;
	unsigned int range_max;
#ifndef CONFIG_SOC_X2500
	range_max = 31;
#else
	range_max = 7;
#endif

	tmp = ddr_readl(DDRP_CMD_NRCOMP);

	pos = scnprintf(buf, 33,  "%d (range 0 ~ %d), %s(ohm)\n", tmp, range_max, dri_ohm_list[tmp]);

	return simple_read_from_buffer(user_buf, count, ppos, buf, pos);
}

static ssize_t ddr_write_strength(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
	unsigned int tmp;
	char buf[16];
	unsigned int range_max;
#ifndef CONFIG_SOC_X2500
	range_max = 31;
#else
	range_max = 7;
#endif
	if (copy_from_user(buf, user_buf, min(count, (sizeof(buf) - 1)))) {
		return -EFAULT;
	}
	tmp = simple_strtoul(buf, NULL, 10);
	if (tmp > 0x1f) {
		printk("can't set value %d, range is 0 ~ %d\n", tmp, range_max);
		tmp = 0x1f;
	} else if (tmp < 0) {
		printk("can't set value %d, range is 0 ~ %d\n", tmp, range_max);
		tmp = 0;
	}

	ddr_writel(tmp, DDRP_CMD_NRCOMP);
	ddr_writel(tmp, DDRP_CMD_PRCOMP);
	ddr_writel(tmp, DDRP_CK_NRCOMP);
	ddr_writel(tmp, DDRP_CK_PRCOMP);
#ifndef CONFIG_SOC_X2500
	ddr_writel(tmp, DDRP_PUDRI_RES0);
	ddr_writel(tmp, DDRP_PDDRI_RES0);
	ddr_writel(tmp, DDRP_PUDRI_RES1);
	ddr_writel(tmp, DDRP_PDDRI_RES1);
#endif
	return count;
}

#ifndef CONFIG_SOC_X2500
static ssize_t ddr_read_odt(struct file *file, char __user *user_buf,
                            size_t count, loff_t *ppos)
{
	char buf[50];
	int pos = 0;
	int tmp;

	tmp = ddr_readl(DDRP_PUODT_RES0);

	pos = scnprintf(buf, 32,  "%d (range 1 ~ 31), %s(ohm)\n", tmp, odt_ohm_list[tmp]);

	return simple_read_from_buffer(user_buf, count, ppos, buf, pos);
}

static ssize_t ddr_write_odt(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
	unsigned int tmp;
	char buf[16];
	if (copy_from_user(buf, user_buf, min(count, (sizeof(buf) - 1)))) {
		return -EFAULT;
	}
	tmp = simple_strtoul(buf, NULL, 10);
	if (tmp > 0x1f) {
		printk("can't set value %d, range is 0 ~ 31\n", tmp);
		tmp = 0x1f;
	} else if (tmp < 0) {
		printk("can't set value %d, range is 0 ~ 31\n", tmp);
		tmp = 0;
	}

	ddr_writel(tmp, DDRP_PDODT_RES0);
	ddr_writel(tmp, DDRP_PUODT_RES0);
	ddr_writel(tmp, DDRP_PDODT_RES1);
	ddr_writel(tmp, DDRP_PUODT_RES1);

	return count;
}
#endif

static inline struct ddrc_chan *get_ddrc_chn(int chn)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ddrc_chans); i++) {
		if (ddrc_chans[i].chn == chn) {
			return &ddrc_chans[i];
		}
	}

	printk("Error find mon_chn, chn: %d\n", chn);
	return NULL;
}

static ssize_t ddr_read_transaction_priority(struct file *file, char __user *user_buf,
        size_t count, loff_t *ppos)
{
	int ret = 0;
	int i = 0;
	unsigned char *buf;
	unsigned char *p;
	unsigned int reg;

	buf = kzalloc(512, GFP_KERNEL);
	if (!buf) {
		printk("Error alloc mem!\n");
		return -ENOMEM;
	}

	p = buf;
	strcpy(p, "range is (0~15)\n");
	p += 16;

	for (i = 0; i < ARRAY_SIZE(ddrc_chans); i++) {
		reg = ddr_readl(DDRC_CCHC(ddrc_chans[i].chn));
		ret = sprintf(p, "chn:%d,	desc:%s,	write priority:%d,	read priority:%d\n",
		              ddrc_chans[i].chn, ddrc_chans[i].desc, \
		              (reg & DDRC_CCHC_WTR_TIMEOUT_MSK) >> DDRC_CCHC_WTR_TIMEOUT_BIT, \
		              (reg & DDRC_CCHC_RTR_TIMEOUT_MSK) >> DDRC_CCHC_RTR_TIMEOUT_BIT);
		p += ret;
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, strlen(buf));
	kfree(buf);
	return ret;
}

static ssize_t ddr_write_transaction_priority(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct ddrc_chan *user_chn = NULL;
	char buf[16];
	char *token, *cur = buf;
	unsigned int val, val2, reg;

	if (copy_from_user(buf, user_buf, min(count, (sizeof(buf) - 1)))) {
		return -EFAULT;
	}

	token = strsep(&cur, ":");
	if (!token) {
		printk("Invalid port settings!\n");
		return -EINVAL;
	}

	val = simple_strtoul(token, NULL, 10);

	user_chn = get_ddrc_chn(val);
	if (NULL == user_chn) {
		return -EINVAL;
	}

	token = strsep(&cur, ":");
	if (!token) {
		printk("Invalid format!\n");
		return -EINVAL;
	}

	val = simple_strtoul(token, NULL, 10);
	if (val < 0 || val > 0xf) {
		printk("set %d error, range is (0~15)", val);
	}

	token = strsep(&cur, ":");
	if (token) {
		val2 = simple_strtoul(token, NULL, 10);
		if (val2 < 0 || val2 > 0xf) {
			printk("set %d error, range is (0~15)", val2);
		}
	} else {
		val2 = val;
	}

	reg = ~(DDRC_CCHC_WTR_TIMEOUT_MSK | DDRC_CCHC_RTR_TIMEOUT_MSK) &\
	      ddr_readl(DDRC_CCHC(user_chn->chn)) ;
	ddr_writel(reg | val << DDRC_CCHC_WTR_TIMEOUT_BIT | \
	           val2 << DDRC_CCHC_RTR_TIMEOUT_BIT, \
	           DDRC_CCHC(user_chn->chn));

	return count;
}

static ssize_t ddr_read_port_priority(struct file *file, char __user *user_buf,
                                      size_t count, loff_t *ppos)
{
	int ret = 0;
	int i = 0;
	unsigned char *buf;
	unsigned char *p;
	unsigned int reg;

	buf = kzalloc(512, GFP_KERNEL);
	if (!buf) {
		printk("Error alloc mem!\n");
		return -ENOMEM;
	}

	p = buf;
	strcpy(p, "range is (0~1)\n");
	p += 15;

	for (i = 0; i < ARRAY_SIZE(ddrc_chans); i++) {
		reg = ddr_readl(DDRC_CCHC(ddrc_chans[i].chn));
		ret = sprintf(p, "chn:%d,	desc:%s,	port priority:%s\n",
		              ddrc_chans[i].chn, ddrc_chans[i].desc, \
		              (reg & DDRC_CCHC_PORT_PRI) ? "high" : "low");
		p += ret;
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, strlen(buf));
	kfree(buf);
	return ret;
}

static ssize_t ddr_write_port_priority(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct ddrc_chan *user_chn = NULL;
	char buf[16];
	char *token, *cur = buf;
	unsigned int val, reg;

	if (copy_from_user(buf, user_buf, min(count, (sizeof(buf) - 1)))) {
		return -EFAULT;
	}

	token = strsep(&cur, ":");
	if (!token) {
		printk("Invalid port settings!\n");
		return -EINVAL;
	}

	val = simple_strtoul(token, NULL, 10);

	user_chn = get_ddrc_chn(val);
	if (NULL == user_chn) {
		return -EINVAL;
	}

	token = strsep(&cur, ":");
	if (!token) {
		printk("Invalid monitor settings!\n");
		return -EINVAL;
	}

	val = simple_strtoul(token, NULL, 10);
	if (val < 0 || val > 0xf) {
		printk("set %d error, range is (0~1)", val);
	}

	reg = ddr_readl(DDRC_CCHC(user_chn->chn)) & (~DDRC_CCHC_PORT_PRI);
	ddr_writel(reg | val << DDRC_CCHC_PORT_PRI_BIT, \
	           DDRC_CCHC(user_chn->chn));

	return count;
}

static ssize_t ddr_read_bandwidth_limit(struct file *file, char __user *user_buf,
                                        size_t count, loff_t *ppos)
{
	int ret = 0;
	int i = 0;
	unsigned char *buf;
	unsigned char *p;
	unsigned int reg;

	buf = kzalloc(512, GFP_KERNEL);
	if (!buf) {
		printk("Error alloc mem!\n");
		return -ENOMEM;
	}

	p = buf;
	strcpy(p, "range is (-1~255)\n");
	p += 18;

	for (i = 0; i < ARRAY_SIZE(ddrc_chans); i++) {
		reg = ddr_readl(DDRC_CCHC(ddrc_chans[i].chn));
		ret = sprintf(p, "chn:%d,	desc:%s\n	write:%d,%s\n	read:%d.%s\n",
		              ddrc_chans[i].chn, ddrc_chans[i].desc, \
		              (reg & DDRC_CCHC_BW_LIMIT_WCNT_MSK) >> DDRC_CCHC_BW_LIMIT_WCNT_BIT, \
		              (reg & DDRC_CCHC_BW_LIMIT_WEN) ? "enable" : "disable", \
		              (reg & DDRC_CCHC_BW_LIMIT_RCNT_MSK) >> DDRC_CCHC_BW_LIMIT_RCNT_BIT, \
		              (reg & DDRC_CCHC_BW_LIMIT_REN) ? "enable" : "disable");
		p += ret;
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, strlen(buf));
	kfree(buf);
	return ret;
}

static ssize_t ddr_write_bandwidth_limit(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct ddrc_chan *user_chn = NULL;
	char buf[16] = {0};
	char *token, *cur = buf;
	int val, val2, reg;

	if (copy_from_user(buf, user_buf, min(count, (sizeof(buf) - 1)))) {
		return -EFAULT;
	}

	token = strsep(&cur, ":");
	if (!token) {
		printk("Invalid port settings!\n");
		return -EINVAL;
	}

	val = simple_strtoul(token, NULL, 10);

	user_chn = get_ddrc_chn(val);
	if (NULL == user_chn) {
		return -EINVAL;
	}

	token = strsep(&cur, ":");
	if (!token) {
		printk("Invalid format!\n");
		return -EINVAL;
	}

	if (0 != kstrtol(token, 10, (void *)&val)) {
		printk("Invalid\n");
		return -EINVAL;
	}

	if (val < -1 || val > 0xff) {
		printk("set %d error, range is (-1~255)", val);
	}

	token = strsep(&cur, ":");
	if (token) {
		if (0 != kstrtol(token, 10, (void *)&val2)) {
			printk("Invalid111\n");
			return -EINVAL;
		} else if (val2 < -1 || val2 > 0xff) {
			printk("set %d error, range is (-1~255)", val2);
		}
	} else {
		val2 = val;
	}


	if (-1 == val) {
		ddr_writel(ddr_readl(DDRC_CCHC(user_chn->chn)) &\
		           (~DDRC_CCHC_BW_LIMIT_WEN), DDRC_CCHC(user_chn->chn));
	} else {
		reg = (~DDRC_CCHC_BW_LIMIT_WCNT_MSK) & ddr_readl(DDRC_CCHC(user_chn->chn));

		ddr_writel(reg | val << DDRC_CCHC_BW_LIMIT_WCNT_BIT | \
		           DDRC_CCHC_BW_LIMIT_WEN, \
		           DDRC_CCHC(user_chn->chn));
	}

	if (-1 == val2) {
		ddr_writel(ddr_readl(DDRC_CCHC(user_chn->chn)) &\
		           (~DDRC_CCHC_BW_LIMIT_REN), DDRC_CCHC(user_chn->chn));
	} else {
		reg = (~DDRC_CCHC_BW_LIMIT_RCNT_MSK) & ddr_readl(DDRC_CCHC(user_chn->chn));

		ddr_writel(reg | val2 << DDRC_CCHC_BW_LIMIT_RCNT_BIT | \
		           DDRC_CCHC_BW_LIMIT_REN, \
		           DDRC_CCHC(user_chn->chn));
	}

	reg = 0;
	return count;
}

static const struct file_operations ddr_ahb_fops = {
	.read   = ddr_read_ahb,
	.write  = ddr_write_ahb,
	.open   = simple_open,
	.llseek = default_llseek,
};

static const struct file_operations ddr_apb_fops = {
	.read   = ddr_read_apb,
	.write  = ddr_write_apb,
	.open   = simple_open,
	.llseek = default_llseek,
};

static const struct file_operations ddr_driver_strength_ops = {
	.read   = ddr_read_strength,
	.write  = ddr_write_strength,
	.open   = simple_open,
	.llseek = default_llseek,
};
#ifndef CONFIG_SOC_X2500
static const struct file_operations ddr_driver_odt_ops = {
	.read   = ddr_read_odt,
	.write  = ddr_write_odt,
	.open   = simple_open,
	.llseek = default_llseek,
};
#endif
static const struct file_operations ddr_transaction_priority_ops = {
	.read   = ddr_read_transaction_priority,
	.write  = ddr_write_transaction_priority,
	.open   = simple_open,
	.llseek = default_llseek,
};

static const struct file_operations ddr_port_priority_ops = {
	.read   = ddr_read_port_priority,
	.write  = ddr_write_port_priority,
	.open   = simple_open,
	.llseek = default_llseek,
};

static const struct file_operations ddr_bandwidth_limit_ops = {
	.read   = ddr_read_bandwidth_limit,
	.write  = ddr_write_bandwidth_limit,
	.open   = simple_open,
	.llseek = default_llseek,
};

static int __init ddr_init(void)
{
	struct dentry *d;
	struct ddr_statistics *ddr = &ddr_statistics;
	/* int i; */

	d = debugfs_create_dir("ddr_config", NULL);
	if (IS_ERR(d)) {
		pr_err("create debugfs for ddr failed.\n");
		return PTR_ERR(d);
	}

	ddr->ddr_debugfs = d;
	d = debugfs_create_file("ddr_ahb_protection", S_IWUSR | S_IRUGO, ddr->ddr_debugfs, ddr, &ddr_ahb_fops);
	if (IS_ERR_OR_NULL(d)) {
		goto err_node;
	}

	d = debugfs_create_file("ddr_apb_protection", S_IWUSR | S_IRUGO, ddr->ddr_debugfs, ddr, &ddr_apb_fops);
	if (IS_ERR_OR_NULL(d)) {
		goto err_node;
	}

#ifndef CONFIG_SOC_X2500
	d = debugfs_create_file("ddr_odt", S_IWUSR | S_IRUGO, ddr->ddr_debugfs, ddr, &ddr_driver_odt_ops);
	if (IS_ERR_OR_NULL(d)) {
		goto err_node;
	}
#endif
	d = debugfs_create_file("ddr_driver_strength", S_IWUSR | S_IRUGO, ddr->ddr_debugfs, ddr, &ddr_driver_strength_ops);
	if (IS_ERR_OR_NULL(d)) {
		goto err_node;
	}

	d = debugfs_create_file("ddr_transaction_priority", S_IWUSR | S_IRUGO, ddr->ddr_debugfs, ddr, &ddr_transaction_priority_ops);
	if (IS_ERR_OR_NULL(d)) {
		goto err_node;
	}

	d = debugfs_create_file("ddr_port_priority", S_IWUSR | S_IRUGO, ddr->ddr_debugfs, ddr, &ddr_port_priority_ops);
	if (IS_ERR_OR_NULL(d)) {
		goto err_node;
	}

	d = debugfs_create_file("ddr_bandwidth_limit", S_IWUSR | S_IRUGO, ddr->ddr_debugfs, ddr, &ddr_bandwidth_limit_ops);
	if (IS_ERR_OR_NULL(d)) {
		goto err_node;
	}

	return 0;
err_node:
	debugfs_remove_recursive(ddr->ddr_debugfs);

	return -1;
}

static void __exit ddr_deinit(void)
{
	debugfs_remove_recursive(ddr_statistics.ddr_debugfs);
}

late_initcall(ddr_init);
module_exit(ddr_deinit);
