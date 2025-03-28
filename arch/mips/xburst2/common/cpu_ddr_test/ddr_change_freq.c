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

/* #define ALL_OTHER_CPU_WAITE */
struct ddr_calib_value {
	unsigned int rate;
	unsigned int refcnt;
	unsigned char bypass_al;
	unsigned char bypass_ah;
};

typedef void (*ddr_change_code)(unsigned int, unsigned int,
                                unsigned int, unsigned int);

struct ddr_cfreq {
	struct dentry       *root;
	spinlock_t lock;
	struct timer_list timer;
	struct clk *clk_ddr;

	unsigned int ddr_cur_rate;
	ddr_change_code tcsm_change_ddr_rate;
};

static struct ddr_cfreq ddr_cfreq;

#define CPM_DDRCDR (0xb000002c)

static ssize_t ddr_read_rate(struct file *file, char __user *user_buf,
                             size_t count, loff_t *ppos)
{
	char buf[4];
	int pos = 0;

	pos = scnprintf(buf, 4, "%d\n", ddr_cfreq.ddr_cur_rate);
	return simple_read_from_buffer(user_buf, count, ppos, buf, pos);
}

static unsigned int get_ddr_parent_rate(void)
{
	unsigned int val;
	struct clk *pll;
	unsigned int pll_rate;

	val = REG32(CPM_DDRCDR);
	switch (val >> 30) {
	case 1:
		pll = clk_get(NULL, "apll");
		break;
	case 2:
		pll = clk_get(NULL, "mpll");
		break;
	default :
		printk("not support\n");
		return 0;
	}

	pll_rate = clk_get_rate(pll);
	return pll_rate;
}
static unsigned int get_ddr_rate(void)
{
	unsigned int pll_rate, ddr_rate;
	unsigned int val;

	val = REG32(CPM_DDRCDR);

	pll_rate = get_ddr_parent_rate();
	ddr_rate = pll_rate / ((val & 0xf) + 1);
	return ddr_rate;
}

static unsigned int pp_mask, ost_mask, mailbox_mask;
#ifdef ALL_OTHER_CPU_WAITE
	static unsigned int pp_pend, ost_pend, mailbox_pend;
#endif

static inline void unmask_others_cpu(void);
static void change_ddr_rate(unsigned char bal, unsigned char bah, unsigned int refcnt, int div)
{
	unsigned int val, cur_div;//, div;
	unsigned int hregpro, pregpro;
	unsigned int autoself_en;
#ifdef ALL_OTHER_CPU_WAITE
	unsigned int wait_cpu;
	unsigned int timeout = 10000;

	REG32(0xb0032000) = '0';

	div = change_div & 0xffff;
	wait_cpu = (change_div >> 16);
	while (!REG32(0xb2407ff8) && timeout --)
		;
	if (!timeout) {
		return;
	}
#endif
	hregpro = ddr_readl(DDRC_HREGPRO);
	pregpro = ddr_readl(DDRC_PREGPRO);
	autoself_en = ddr_readl(DDRC_AUTOSR_EN);

	ddr_writel(0, DDRC_HREGPRO);
	ddr_writel(0, DDRC_PREGPRO);
	ddr_writel(0, DDRC_AUTOSR_EN);

	// Disable DFI lowpower handshake
	/* val = ddr_readl(DDRC_DLP); */
	/* val &= ~(1); */
	/* ddr_writel(val, DDRC_DLP); */

	val = REG32(CPM_DDRCDR);
	cur_div = (val & 0xf);

	if (cur_div > div) {
		val = ddr_readl(DDRC_REFCNT);
		val &= ~(0x3ffff << 8 | 0xe);
		val |= refcnt;
		ddr_writel(val, DDRC_REFCNT);
	}
	/**
	 * don't use AHB0/2 APB BUS before freq exit.
	 */
	/**
	 * Set !change_en and ce
	 */
	val = REG32(CPM_DDRCDR);
	val |= ((1 << 29) | (1 << 25));
	REG32(CPM_DDRCDR) = val;
	/* while((REG32(CPM_DDRCDR) & (1 << 24))); */

	/**
	 * Set clock divider
	 */
	val = REG32(CPM_DDRCDR);
	val &= ~(0xf);
	val |= div;
	REG32(CPM_DDRCDR) = val;
	//  while((REG32(CPM_DDRCDR) & (1 << 28)));
	/**
	 * Polling PHY_FREQ_DONE
	 */
	while ((ddr_readl(DDRC_DWSTATUS) & (1 << 3 | 1 << 1)) != 0xa)
		;

	ddr_writel(DDRP_TRAINING_CTRL_DSCSE_BP, DDRP_INNOPHY_TRAINING_CTRL);
	ddr_writel(bal, DDRP_INNOPHY_CALIB_BYPASS_AL);
	ddr_writel(bah, DDRP_INNOPHY_CALIB_BYPASS_AH);

	/**
	 * Set Controller Freq Exit
	 */
	val = ddr_readl(DDRC_DWCFG);
	val |= (1 << 2);
	ddr_writel(val, DDRC_DWCFG);

	/**
	 * Clear Controller Freq Exit
	 */
	val = ddr_readl(DDRC_DWCFG);
	val &= ~(1 << 2);
	ddr_writel(val, DDRC_DWCFG);

	/**
	 * clear change_en and ce
	 */
	val = REG32(CPM_DDRCDR);
	val &= ~((1 << 29) | (1 << 25));
	REG32(CPM_DDRCDR) = val;

	unmask_others_cpu();

#ifdef ALL_OTHER_CPU_WAITE
	if (ost_pend) {
		set_ccu_oipr(get_ccu_oipr() | (1 << wait_cpu));
	}
	if (pp_pend) {
		set_ccu_pipr(get_ccu_pipr() | (1 << wait_cpu));
	}
	if (mailbox_pend) {
		set_ccu_mipr(get_ccu_mipr() | (1 << wait_cpu));
	}

	if (ost_mask) {
		set_ccu_oimr(get_ccu_oimr() | (1 << wait_cpu));
	}
	if (pp_mask) {
		set_ccu_pimr(get_ccu_pimr() | (1 << wait_cpu));
	}
	if (mailbox_mask) {
		set_ccu_mimr(get_ccu_mimr() | (1 << wait_cpu));
	}
	/* REG32(0xb2407ff8) = 0; */
#endif

	if (cur_div < div) {
		val = ddr_readl(DDRC_REFCNT);
		val &= ~(0x3ffff << 8 | 0xe);
		val |= refcnt;
		ddr_writel(val, DDRC_REFCNT);
	}

	if (autoself_en) {
		ddr_writel(1, DDRC_AUTOSR_EN);
	}
	if (hregpro & (1 << 1)) {
		ddr_writel(1, DDRC_HREGPRO);
	}
	if (pregpro & (1 << 1)) {
		ddr_writel(1, DDRC_PREGPRO);
	}
}

#define __read_32bit_register()                 \
	({ int __res;                               \
		__asm__ __volatile__(                   \
		                                        "move \t %0, $29\n\t"               \
		                                        : "=r" (__res));                    \
		__res;                                  \
	})

#define __write_32bit_register(value)                       \
	do {                                                    \
		__asm__ __volatile__(                               \
		        "move \t $29, %0\n\t"                           \
		        : : "Jr" ((unsigned int)(value)));              \
	} while (0)

#define read_sp_register() __read_32bit_register()
#define write_sp_register(val) __write_32bit_register(val)
static unsigned int change_count;
static int try_and_lock_ccu(unsigned int cpu)
{
	int timeout = 1000;
	unsigned int cslr;

	do {
		set_ccu_csar(cpu);
		cslr = get_ccu_cslr();
		if ((cslr & 1 << 31) && ((cslr & ((1 << CONFIG_NR_CPUS) - 1)) == cpu)) {
			break;
		}
	} while (timeout --);
	if (timeout < 0) {
		return -1;
	}
	return 0;
}
static void unlock_ccu(void)
{
	set_ccu_cslr(0);
}
static int mask_and_wait_others_cpu(unsigned int cur_cpu)
{
	unsigned int pp_mval, mb_mval, ost_mval;
	unsigned int max_cpu_num = CONFIG_NR_CPUS;
	unsigned int mask_cpus = ((1 << max_cpu_num) - 1);
	unsigned int mask_other_cpus = (mask_cpus & (~(1 << cur_cpu)));
	int timeout, ret = 0;

	pp_mval = get_ccu_pimr();
	mb_mval = get_ccu_mimr();
	ost_mval = get_ccu_oimr();

	pp_mask = pp_mval & (mask_cpus);
	mailbox_mask = mb_mval & (mask_cpus);
	ost_mask = ost_mval & (mask_cpus);

	try_and_lock_ccu(cur_cpu);
	if (ret < 0) {
		printk("try to lock ccu failed\n");
		return ret;
	}

	set_ccu_pimr(0);
	set_ccu_mimr(0);
	set_ccu_oimr(0);

	timeout = 100000;
	while (((get_ccu_cssr() & mask_other_cpus) != mask_other_cpus)
	       && timeout--);

	if (!get_ccu_cssr() && timeout >= 0) {
		printk("ERROR:get_ccu_cssr() %x timeout %d mask_other_cpus %x\n", get_ccu_cssr(), timeout, mask_other_cpus);
		printk("ERROR:get_ccu_cssr() %x timeout %d mask_other_cpus %x\n", get_ccu_cssr(), timeout, mask_other_cpus);
		printk("ERROR:get_ccu_cssr() %x timeout %d mask_other_cpus %x\n", get_ccu_cssr(), timeout, mask_other_cpus);
		printk("ERROR:get_ccu_cssr() %x timeout %d mask_other_cpus %x\n", get_ccu_cssr(), timeout, mask_other_cpus);
		printk("ERROR:get_ccu_cssr() %x timeout %d mask_other_cpus %x\n", get_ccu_cssr(), timeout, mask_other_cpus);
		printk("get_ccu_cssr() %x timeout %d mask_other_cpus %x\n", get_ccu_cssr(), timeout, mask_other_cpus);
		printk("SMP: ------------------wakeup--------CPU%d: cause %x\n", cur_cpu, read_c0_cause());
		printk("SMP: ------------------wakeup--------CPU%d: pp pending %x\n", cur_cpu, REG32(0xb2200100));
		printk("SMP: ------------------wakeup--------CPU%d: ost pending %x\n", cur_cpu, REG32(0xb2200180));
		printk("SMP: ------------------wakeup--------CPU%d: mailbox pending %x\n", cur_cpu, REG32(0xb2200140));

		printk("SMP: ------------------wakeup--------CPU%d: pp mask %x\n", cur_cpu, get_ccu_pimr());
		printk("SMP: ------------------wakeup--------CPU%d: mailbox mask %x\n", cur_cpu, get_ccu_mimr());
		printk("SMP: ------------------wakeup--------CPU%d: os mask %x\n", cur_cpu, get_ccu_oimr());
		while (1);
	}

	if (timeout < 0) {
		set_ccu_pimr(pp_mask);
		set_ccu_mimr(mailbox_mask);
		set_ccu_oimr(ost_mask);
		ret = -1;
	}
	unlock_ccu();

	return ret;
}
static inline void unmask_others_cpu(void)
{
	/* try_and_lock_ccu(cur_cpu); */
	/* if(ret < 0) { */
	/*  printk("try to lock ccu failed\n"); */
	/*  return ret; */
	/* } */
	set_ccu_pimr(pp_mask);
	set_ccu_mimr(mailbox_mask);
	set_ccu_oimr(ost_mask);
	/* unlock_ccu(); */
}
#ifdef ALL_OTHER_CPU_WAITE
static void disable_interrupt(void *args)
{
	unsigned int cpu = smp_processor_id();
	printk("SMP: --------------------------CPU%d is offline\n", cpu);

	if (cpu != *(unsigned int *)args) {
		unsigned int pp_mval, mb_mval, ost_mval;
		unsigned int pp_pval, mb_pval, ost_pval;
		pp_mval = get_ccu_pimr();
		mb_mval = get_ccu_mimr();
		ost_mval = get_ccu_oimr();

		pp_mask = pp_mval & (1 << cpu);
		mailbox_mask = mb_mval & (1 << cpu);
		ost_mask = ost_mval & (1 << cpu);

		set_ccu_pimr(pp_mval & (~(1 << cpu)));
		set_ccu_mimr(mb_mval & (~(1 << cpu)));
		set_ccu_oimr(ost_mval & (~(1 << cpu)));

		pp_pval = get_ccu_pipr();
		mb_pval = get_ccu_mipr();
		ost_pval = get_ccu_oipr();

		pp_pend = pp_pval & (1 << cpu);
		mailbox_pend = mb_pval & (1 << cpu);
		ost_pend = ost_pval & (1 << cpu);

		set_ccu_pipr(pp_pval & (~(1 << cpu)));
		set_ccu_mipr(mb_pval & (~(1 << cpu)));
		set_ccu_oipr(ost_pval & (~(1 << cpu)));
	}

	REG32(0xb2407ff8) = 0xa5a5a5a5;
	__asm__ __volatile__("wait \n\t");

	printk("SMP: ------------------wakeup--------CPU%d: cause %x\n", cpu, read_c0_cause());
	printk("SMP: ------------------wakeup--------CPU%d: pp pending %x\n", cpu, REG32(0xb2200100));
	printk("SMP: ------------------wakeup--------CPU%d: ost pending %x\n", cpu, REG32(0xb2200180));
	printk("SMP: ------------------wakeup--------CPU%d: mailbox pending %x\n", cpu, REG32(0xb2200140));
}
#endif

static ssize_t ddr_write_rate(struct file *file, const char __user *user_buf,
                              size_t count, loff_t *ppos)
{
	char buf[16];
	unsigned int change_rate;
	unsigned long flags;
	unsigned int cpu = smp_processor_id();
	unsigned int sp, change_div;
	int ret;
	struct ddr_calib_value *dcv, *curdcv;

	dcv = (struct ddr_calib_value *)(CPU_TCSM_DDR_CALIB);

	if (copy_from_user(buf, user_buf, min(count, (sizeof(buf) - 1)))) {
		return -EFAULT;
	}

	change_rate = simple_strtoul(buf, NULL, 10) * 1000000;
	printk("change_rate %d-----------------\n", change_rate);
	if (change_rate == ddr_cfreq.ddr_cur_rate) {
		printk("change rate equl current rate\n");
		return count;
	}

	change_div = get_ddr_parent_rate() / change_rate - 1;
	if (dcv[change_div].rate != change_rate) {
		int i;
		printk("rate %d not support %d\n", change_rate, dcv[change_div].rate);
		for (i = 0; i < 6; i ++) {
			printk("div %d, rate %d\n", i, dcv[i].rate);
		}
		printk("change_div %d\n", change_div);
		return count;
	}
	curdcv = &dcv[change_div];

	change_count ++;
	printk("------------cpu %d----------------%d------------------\n", cpu, change_count);

#ifdef ALL_OTHER_CPU_WAITE
	REG32(0xb2407ff8) = 0;
	change_div |= ((cpu ? 0 : 1) << 16);
	//  preempt_disable();
	/* on_each_cpu(disable_interrupt, &cpu, 0); */
	smp_call_function(disable_interrupt, &cpu, 0);
#endif

	spin_lock_irqsave(&ddr_cfreq.lock, flags);
	ret = mask_and_wait_others_cpu(cpu);
	if (ret < 0) {
		printk("lock failed\n");
		change_count --;
		spin_unlock_irqrestore(&ddr_cfreq.lock, flags);
		return ret;
	}
	sp = read_sp_register();
	write_sp_register(CPU_TCSM_SP);
	ddr_cfreq.tcsm_change_ddr_rate(curdcv->bypass_al,
	                               curdcv->bypass_ah, curdcv->refcnt, change_div);
	write_sp_register(sp);
	spin_unlock_irqrestore(&ddr_cfreq.lock, flags);
	//  preempt_enable();
	printk("change succeed\n");

	ddr_cfreq.ddr_cur_rate = get_ddr_rate();
	clk_set_rate(ddr_cfreq.clk_ddr, ddr_cfreq.ddr_cur_rate);
	/* ddr_cfreq.ddr_cur_rate = clk_get_rate(ddr_cfreq.clk_ddr); */
	printk("clk_get_rate(ddr_cfreq.clk_ddr) %d\n", (unsigned int)clk_get_rate(ddr_cfreq.clk_ddr));

	return count;
}

static const struct file_operations ddr_run_fops = {
	.read   = ddr_read_rate,
	.write  = ddr_write_rate,
	.open   = simple_open,
	.llseek = default_llseek,
};

static void load_func_to_tcsm(unsigned int *tcsm_addr, unsigned int *f_addr, unsigned int size)
{
	unsigned int instr;
	int offset;
	int i;
	for (i = 0; i < size / 4; i++) {
		instr = f_addr[i];
		if ((instr >> 26) == 2) {
			offset = instr & 0x3ffffff;
			offset = (offset << 2) - ((unsigned int)f_addr & 0xfffffff);
			if (offset > 0) {
				printk("f_addr[i]%x: %x--------------------------------------------------\n", (unsigned int)&f_addr[i], f_addr[i]);
				offset = ((unsigned int)tcsm_addr & 0xfffffff) + offset;
				/* offset = ((unsigned int)0xf4000000 & 0xfffffff) + offset; */
				instr = (2 << 26) | (offset >> 2);
			}
		}
		tcsm_addr[i] = instr;
	}
}

static int __init ddr_init(void)
{
	struct dentry *d;
	struct ddr_cfreq *cfreq = &ddr_cfreq;

	cfreq->clk_ddr = clk_get(NULL, "div_ddr");
	if (!cfreq->clk_ddr) {
		printk("get ddr clk failed\n");
		return -1;
	}

	cfreq->ddr_cur_rate = clk_get_rate(cfreq->clk_ddr);
	{
		unsigned int *tcsm_bank0 = (unsigned int *)CPU_TCSM_FUNC;
		/* unsigned int *tcsm_bank0 = (unsigned int *)0xb3423000; */
		unsigned int *func0 = (unsigned int *)change_ddr_rate;

		load_func_to_tcsm(tcsm_bank0, func0, CPU_TCSM_FUNC_SIZE);
		cfreq->tcsm_change_ddr_rate = (ddr_change_code)tcsm_bank0;
	}


	d = debugfs_create_dir("ddrfreq", NULL);
	if (IS_ERR(d)) {
		pr_err("create debugfs for ddr failed.\n");
		return PTR_ERR(d);
	}
	spin_lock_init(&cfreq->lock);

	cfreq->root = d;
	debugfs_create_u32("ddr_cur_rate", S_IWUSR | S_IRUGO, cfreq->root,
	                   (u32 *)&cfreq->ddr_cur_rate);

	d = debugfs_create_file("rate", S_IWUSR | S_IRUGO, cfreq->root,
	                        cfreq, &ddr_run_fops);
	if (IS_ERR_OR_NULL(d)) {
		goto err_node;
	}

	return 0;
err_node:
	debugfs_remove_recursive(cfreq->root);

	return -1;
}

static void __exit ddr_deinit(void)
{
	struct ddr_cfreq *cfreq = &ddr_cfreq;
	debugfs_remove_recursive(cfreq->root);
}

late_initcall(ddr_init);
module_exit(ddr_deinit);
