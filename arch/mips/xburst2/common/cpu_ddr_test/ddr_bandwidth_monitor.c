#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/syscore_ops.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/spinlock.h>
#define DDR_MONITOR_BASE   0x134F0000
#define DDR_DYNAMIC_CLK    0x13012068
#define DDR_DWCFG      0x13012000
#define DDR_CMONC0     0x13012050

#define DB_MONITOR_BASE_OFF  0x88// DDR bandwidth monitor base offset address.

#define MAX_SUPPORTED_MONITORS  4

struct dbw_channel_reg {
	volatile unsigned int write;
	unsigned int reserve1;
	volatile unsigned int read;
	unsigned int reserve2;
};
struct dbw_reg {
	volatile unsigned int dbwcfg;
	unsigned int reserve;
	volatile unsigned int period;
	unsigned int reserver;
	struct dbw_channel_reg chan[4];
};


#define GMAC_MSC_CHANNEL 0  //DDR ch0
#define VPU_CHANNEL  1  //DDR ch1
#define DPU_VPU_CHANNEL  3  //DDR ch3
#define AHB2_CHANNEL     5  //DDR ch5
#define CPU_CHANNEL      6  //DDR ch6

struct ddr_mon_chan {
	int chn;
	const char *desc;
};

static struct ddr_mon_chan mon_chans[] = {
	{0, "gmac&msc"},
	{1, "vpu&isp"},
	{3, "dpu&cim"},
	{5, "ahb2&audio&apb"},
	{6, "cpu"}
};

static unsigned int default_monitor_chn[MAX_SUPPORTED_MONITORS] = {0, 3, 5, 6};

struct ddr_dbw_monitor {
	struct ddr_mon_chan *ch;
	unsigned int read;
	unsigned int write;
};


static inline struct ddr_mon_chan *get_mon_chn(int chn)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mon_chans); i++) {
		if (mon_chans[i].chn == chn) {
			return &mon_chans[i];
		}
	}

	printk("Error find mon_chn, chn: %d\n", chn);
	return NULL;
}

static int set_mon_chn(struct ddr_dbw_monitor *monitors, int chn)
{
	struct ddr_mon_chan *mon_chn = get_mon_chn(chn);

	printk("set _mon_chn :%d\n", chn);

	if (!mon_chn) {
		return -EINVAL;
	}

	monitors->ch = mon_chn;

	return 0;
}


struct ddr_statistics {
	struct dentry       *root;
	unsigned int periods;
	unsigned int output;
	int run;

	unsigned int ahb2_read_rate;
	unsigned int ahb2_write_rate;
	unsigned int cpu_read_rate;
	unsigned int cpu_write_rate;

	/* 1 package  = 4 * 32 = 8 * 16 */
	unsigned int pkg_cnt_to_cycle;

	int poll_periods_ms;
	struct dbw_reg *reg;

	struct ddr_dbw_monitor dbw_monitors[4];

	struct mutex        lock;
	struct timer_list timer;
};
static struct ddr_statistics ddr_statistics = {
	.periods = 25 * 1000 * 1000,
	.run = 0,
	.poll_periods_ms = 40,
	.output = 1,
};
#define BW_DONE (1 << 3)
#define BW_INT_EN (1 << 2)
#define BW_CLR  (1 << 1)
#define BW_EN   (1 << 0)

#define DDR_PACKAGE_TO_CYCLE 4
static void ddr_stat_output(struct ddr_statistics *ddr)
{
	uint64_t period = (uint64_t)ddr->reg->period;
#define occupancy_show(title,chn,id) do{                                    \
		uint64_t w_occupancy = (uint64_t)ddr->dbw_monitors[id].write * ddr->pkg_cnt_to_cycle;   \
		uint64_t r_occupancy = (uint64_t)ddr->dbw_monitors[id].read * ddr->pkg_cnt_to_cycle;    \
		uint64_t d = 100000 * w_occupancy;                          \
		uint64_t hi;                                        \
		do_div(d,period);                                   \
		hi = (uint32_t)d / 1000;                                \
		printk("%s chn:%d write occupancy rate:%lld.%03lld%%\n",title,chn,hi,d - hi * 1000);        \
		d = 100000 * r_occupancy;                               \
		do_div(d,period);                                   \
		hi = (uint32_t)d / 1000;                                \
		printk("%s chn:%d read occupancy rate:%lld.%03lld%%\n",title,chn, hi,d - hi * 1000);        \
	}while(0)
	occupancy_show(ddr->dbw_monitors[0].ch->desc, ddr->dbw_monitors[0].ch->chn, 0);
	occupancy_show(ddr->dbw_monitors[1].ch->desc, ddr->dbw_monitors[1].ch->chn, 1);
	occupancy_show(ddr->dbw_monitors[2].ch->desc, ddr->dbw_monitors[2].ch->chn, 2);
	occupancy_show(ddr->dbw_monitors[3].ch->desc, ddr->dbw_monitors[3].ch->chn, 3);
#undef occupancy_show
}

static struct ddr_dbw_monitor *get_dbw_monitor(struct ddr_statistics *ddr, int chn)
{
	struct ddr_dbw_monitor *m;
	int i = 0;

	for (i = 0; i < MAX_SUPPORTED_MONITORS; i++) {
		m = &ddr->dbw_monitors[i];
		if (m->ch->chn == chn) {
			return m;
		}
	}

	return NULL;
}

static ssize_t ddr_read_run(struct file *file, char __user *user_buf,
                            size_t count, loff_t *ppos)
{
	struct ddr_statistics *ddr = file->private_data;
	char *buf;

	mutex_lock(&ddr->lock);
	if (ddr->reg->dbwcfg & BW_EN) {
		buf = "Runing\n";
	} else {
		buf = "Stop\n";
	}
	mutex_unlock(&ddr->lock);
	return simple_read_from_buffer(user_buf, count, ppos, buf, strlen(buf));
}

static ssize_t ddr_write_run(struct file *file, const char __user *user_buf,
                             size_t count, loff_t *ppos)
{
	struct ddr_statistics *ddr = file->private_data;
	struct ddr_dbw_monitor *m;
	char buf[16];
	bool bv;
	int ret = 0;
	unsigned int val;
	int i = 0;
	if (copy_from_user(buf, user_buf, min(count, (sizeof(buf) - 1)))) {
		return -EFAULT;
	}

	if (strtobool(buf, &bv) == 0) {
		mutex_lock(&ddr->lock);

		if (bv) {
			val = readl((void *)CKSEG1ADDR(DDR_DYNAMIC_CLK));
			val &= ~(1 << 8);
			writel(val, (void *)CKSEG1ADDR(DDR_DYNAMIC_CLK));

			del_timer_sync(&ddr->timer);
			if (ddr->reg->period != ddr->periods) {
				ddr->poll_periods_ms = 0;
			}
			ddr->run = 1;
			ddr->reg->period = ddr->periods;
			val = ddr->reg->period;

			ddr->reg->dbwcfg = BW_EN | BW_CLR;
			while ((ddr->reg->dbwcfg & (BW_CLR | BW_EN)) != BW_EN);
			printk("run!\n");
			mod_timer(&ddr->timer, jiffies + msecs_to_jiffies(ddr->poll_periods_ms));
		} else {

			del_timer_sync(&ddr->timer);
			ddr->run = 0;
			for (i = 0; i < MAX_SUPPORTED_MONITORS; i++) {
				ddr->dbw_monitors[i].write = ddr->reg->chan[i].write;
				ddr->dbw_monitors[i].read = ddr->reg->chan[i].read;
			}

			if (ddr->output) {
				ddr_stat_output(ddr);
			}

			ddr->ahb2_write_rate = (m = get_dbw_monitor(ddr, AHB2_CHANNEL)) == NULL ? 0 : m->write;
			ddr->ahb2_read_rate = (m = get_dbw_monitor(ddr, AHB2_CHANNEL)) == NULL ? 0 : m->read;
			ddr->cpu_write_rate = (m = get_dbw_monitor(ddr, CPU_CHANNEL)) == NULL ? 0 : m->write;
			ddr->cpu_read_rate = (m = get_dbw_monitor(ddr, CPU_CHANNEL)) == NULL ? 0 : m->read;


			ddr->reg->dbwcfg = 0;
			printk("stopping!\n");
		}
		mutex_unlock(&ddr->lock);
	}

	return ret ? ret : count;
}

static const struct file_operations ddr_run_fops = {
	.read   = ddr_read_run,
	.write  = ddr_write_run,
	.open   = simple_open,
	.llseek = default_llseek,
};
static void ddr_stat_timer_handler(struct timer_list *timer)
{
	struct ddr_statistics *ddr = container_of(timer, struct ddr_statistics, timer);
	unsigned int dbwcfg = ddr->reg->dbwcfg;
	struct ddr_dbw_monitor *m;
	int i = 0;
	if (dbwcfg & BW_DONE) {
		if (ddr->run) {
			for (i = 0; i < MAX_SUPPORTED_MONITORS; i++) {
				ddr->dbw_monitors[i].write = ddr->reg->chan[i].write;
				ddr->dbw_monitors[i].read = ddr->reg->chan[i].read;
			}

			if (ddr->output) {
				ddr_stat_output(ddr);
			}
			if (ddr->reg->dbwcfg != dbwcfg) {
				ddr->poll_periods_ms = 0;
			}


			ddr->ahb2_write_rate = (m = get_dbw_monitor(ddr, AHB2_CHANNEL)) == NULL ? 0 : m->write;
			ddr->ahb2_read_rate = (m = get_dbw_monitor(ddr, AHB2_CHANNEL)) == NULL ? 0 : m->read;
			ddr->cpu_write_rate = (m = get_dbw_monitor(ddr, CPU_CHANNEL)) == NULL ? 0 : m->write;
			ddr->cpu_read_rate = (m = get_dbw_monitor(ddr, CPU_CHANNEL)) == NULL ? 0 : m->read;

			dbwcfg |= BW_CLR;
			ddr->reg->dbwcfg = dbwcfg;

			while (ddr->reg->dbwcfg & BW_CLR);
			mod_timer(&ddr->timer, jiffies + msecs_to_jiffies(ddr->poll_periods_ms));
		} else {
			ddr->reg->dbwcfg = 0;
		}
	} else {
		mod_timer(&ddr->timer, jiffies + msecs_to_jiffies(10));
		ddr->poll_periods_ms += 10;
	}
}

static int dbw_monitors_init(struct ddr_statistics *ddr)
{
	unsigned int cmonc0;
	int i;

	for (i = 0; i < MAX_SUPPORTED_MONITORS; i++) {
		ddr->dbw_monitors[i].ch = get_mon_chn(default_monitor_chn[i]);
	}

#if 0
	ddr->dbw_monitors[0].ch = &mon_chans[0];
	ddr->dbw_monitors[1].ch = &mon_chans[1];
	ddr->dbw_monitors[2].ch = &mon_chans[2];
	ddr->dbw_monitors[3].ch = &mon_chans[3];
#endif
	cmonc0 = readl((void *)CKSEG1ADDR(DDR_CMONC0));
	/*cmonc0[15:4]*/
	cmonc0 &= ~0xfff0;
	cmonc0 |= (ddr->dbw_monitors[0].ch->chn << 13) \
	          | (ddr->dbw_monitors[1].ch->chn << 10) \
	          | (ddr->dbw_monitors[2].ch->chn << 7)  \
	          | (ddr->dbw_monitors[3].ch->chn << 4);
	writel(cmonc0, (void *)CKSEG1ADDR(DDR_CMONC0));

	return 0;
}
static ssize_t ddr_read_monitors(struct file *file, char __user *user_buf,
                                 size_t count, loff_t *ppos)
{
	struct ddr_statistics *ddr = file->private_data;
	struct ddr_dbw_monitor *m = NULL;
	int i = 0;
	int ret = 0;

	unsigned char *buf = kzalloc(1024, GFP_KERNEL);
	unsigned char *p = buf;

	if (!buf) {
		printk("Error alloc mem!\n");
		return -ENOMEM;
	}

	for (i = 0; i < MAX_SUPPORTED_MONITORS; i++) {
		m = &ddr->dbw_monitors[i];
		ret = sprintf(p, "monitor:%d, chn:%d, desc:%s\n\t read:%d, write:%d\n",
		              i, m->ch->chn, m->ch->desc, m->read, m->write);
		p += ret;
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, strlen(buf));
	kfree(buf);
	return ret;
}

static ssize_t ddr_write_monitors(struct file *file, const char __user *user_buf,
                                  size_t count, loff_t *ppos)
{

	struct ddr_statistics *ddr = file->private_data;
	char buf[16];
	int i = 0;
	unsigned int val;
	char *token, *cur = buf;
	unsigned int cmonc0;

	if (copy_from_user(buf, user_buf, min(count, (sizeof(buf) - 1)))) {
		return -EFAULT;
	}

	for (i = 0; i < MAX_SUPPORTED_MONITORS; i++) {
		token = strsep(&cur, ":");
		if (!token) {
			printk("Invalid monitor settings!\n");
			return -EINVAL;
		}
		val = simple_strtoul(token, NULL, 10);

		if (set_mon_chn(&ddr->dbw_monitors[i], val) < 0) {
			return -EINVAL;
		}
	}

	cmonc0 = readl((void *)CKSEG1ADDR(DDR_CMONC0));
	/*cmonc0[15:4]*/
	cmonc0 &= ~0xfff0;
	cmonc0 |= (ddr->dbw_monitors[0].ch->chn << 13) \
	          | (ddr->dbw_monitors[1].ch->chn << 10) \
	          | (ddr->dbw_monitors[2].ch->chn << 7)  \
	          | (ddr->dbw_monitors[3].ch->chn << 4);
	writel(cmonc0, (void *)CKSEG1ADDR(DDR_CMONC0));

	return count;
}

static const struct file_operations ddr_monitors_fops = {
	.read   = ddr_read_monitors,
	.write  = ddr_write_monitors,
	.open   = simple_open,
	.llseek = default_llseek,
};


static int __init ddr_stat_init(void)
{
	struct dentry *d;
	struct ddr_statistics *ddr = &ddr_statistics;

	d = debugfs_create_dir("ddr", NULL);
	if (IS_ERR(d)) {
		pr_err("create debugfs for ddr failed.\n");
		return PTR_ERR(d);
	}


	ddr->root = d;

	d = debugfs_create_file("monitors", S_IWUSR | S_IRUGO, ddr->root,
	                        ddr, &ddr_monitors_fops);
	if (IS_ERR_OR_NULL(d)) {
		goto err_node;
	}

	debugfs_create_u32("periods", S_IWUSR | S_IRUGO, ddr->root,
	                   (u32 *)&ddr->periods);

	debugfs_create_u32("output", S_IWUSR | S_IRUGO, ddr->root,
	                   (u32 *)&ddr->output);

	debugfs_create_u32("ahb2_read_rate", S_IWUSR | S_IRUGO, ddr->root,
	                   (u32 *)&ddr->ahb2_read_rate);

	debugfs_create_u32("ahb2_write_rate", S_IWUSR | S_IRUGO, ddr->root,
	                   (u32 *)&ddr->ahb2_write_rate);

	debugfs_create_u32("cpu_read_rate", S_IWUSR | S_IRUGO, ddr->root,
	                   (u32 *)&ddr->cpu_read_rate);

	debugfs_create_u32("cpu_write_rate", S_IWUSR | S_IRUGO, ddr->root,
	                   (u32 *)&ddr->cpu_write_rate);



	d = debugfs_create_file("run", S_IWUSR | S_IRUGO, ddr->root,
	                        ddr, &ddr_run_fops);
	if (IS_ERR_OR_NULL(d)) {
		goto err_node;
	}
	mutex_init(&ddr->lock);

	ddr->reg = ioremap(DDR_MONITOR_BASE + DB_MONITOR_BASE_OFF, sizeof(struct dbw_reg));

	if (readl((void *)CKSEG1ADDR(DDR_DWCFG)) & 1) {
		/* 32bit ddr*/
		ddr->pkg_cnt_to_cycle = 4;

	} else {
		ddr->pkg_cnt_to_cycle = 8;

	}

	dbw_monitors_init(ddr);



	init_timers();
	ddr->timer.function = ddr_stat_timer_handler;

	return 0;
err_node:
	debugfs_remove_recursive(ddr->root);

	return -1;
}

static void __exit ddr_stat_deinit(void)
{
	struct ddr_statistics *ddr = &ddr_statistics;
	debugfs_remove_recursive(ddr->root);
}

late_initcall(ddr_stat_init);
module_exit(ddr_stat_deinit);
