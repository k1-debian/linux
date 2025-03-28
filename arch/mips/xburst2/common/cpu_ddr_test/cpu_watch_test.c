#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/syscore_ops.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>

#include <linux/mfd/core.h>
//#include <mach/jz_tcu.h>

#include <asm/r4kcache.h>
#include <asm/mmu_context.h>


#include"../../../../../include/linux/mfd/ingenic-tcu_v2.h"

struct watch_struct {
	struct task_struct *child;
	unsigned int watchhi;
	unsigned int watchlo;

	/* debugfs */
	struct dentry *root;
};

struct watch_struct _cwt; /* cpu watch test */
struct watch_struct *cwt; /* cpu watch test */


static struct task_struct *trace_get_task_struct(pid_t pid)
{
	struct task_struct *child;

	rcu_read_lock();
	child = find_task_by_vpid(pid);
	if (child) {
		get_task_struct(child);
	}
	rcu_read_unlock();

	if (!child) {
		return ERR_PTR(-ESRCH);
	}
	return child;
}
char g_buffer[256];

static ssize_t cpu_watch_read_start(struct file *file, char __user *user_buf,
                                    size_t count, loff_t *ppos)
{
	char *buf = g_buffer;

	return simple_read_from_buffer(user_buf, count, ppos, buf, strlen(buf));

}

static void local_set_watch(void *args)
{
	struct watch_struct *watch = (struct watch_struct *)args;
	unsigned int cpu = smp_processor_id();
	unsigned int hi = 0;
	//  struct mips3264_watch_reg_state *watches =
	//&watch->child->thread.watch.mips3264;

	if (watch->watchhi & 0x40000000) {
		hi = watch->watchhi;
	} else {
		hi = (watch->watchhi & 0xffff) | (cpu_asid(cpu, watch->child->mm) << 16);
	}
	if (watch->watchlo == 0) {
		hi = 0;
	}
	//  watches->watchlo[0][cpu] = watch->watchlo;
	//  watches->watchhi[0][cpu] = hi;

	printk("watchlo: %x, watchhi: %x, cpu; %d\n", watch->watchlo, hi, cpu);
	write_c0_watchlo0(watch->watchlo);
	write_c0_watchhi0(hi);

}
static inline void jz_on_each_cpu(void (*func)(void *info), void *info)
{
	preempt_disable();
	smp_call_function(func, info, 1);
	func(info);
	preempt_enable();
}

static ssize_t cpu_watch_write_start(struct file *file, const char __user *user_buf,
                                     size_t count, loff_t *ppos)
{
	char s[20];
	char *s1, *s2;
	pid_t pid;
	unsigned int hi;
	unsigned int lo;
	struct task_struct *child = NULL;
	struct watch_struct watch;
	//  struct mips3264_watch_reg_state *watches;

	copy_from_user(g_buffer, user_buf, count);

	g_buffer[count] = 0;
	printk("%s\n", g_buffer);
	s1 = g_buffer;
	s2 = strchr(s1, ':');
	printk("s1 = %px s2 = %px\n", s1, s2);
	if (!s2 || count == 0) {
		pid = simple_strtoul(s1, 0, 0);
		child = trace_get_task_struct(pid);
		watch.child = child;
		watch.watchlo = 0;
		watch.watchhi = 0;
		jz_on_each_cpu(local_set_watch, &watch);
		//watches = &child->thread.watch.mips3264;
		//watches->trace_type = 0;
		clear_tsk_thread_flag(child, TIF_LOAD_WATCH);

		return count;
	}

	memcpy(s, s1, s2 - s1);
	s[s2 - s1]  = 0;
	pid = simple_strtoul(s, 0, 0);

	s1 = s2 + 1;
	s2 = strchr(s1, ':');
	printk("s1 = %px s2 = %px\n", s1, s2);
	memcpy(s, s1, s2 - s1);
	s[s2 - s1]  = 0;
	lo = simple_strtoul(s, 0, 0);

	s1 = s2 + 1;
	hi = simple_strtoul(s1, 0, 0);
	printk("pid = %d lo = 0x%08x hi = %d\n", pid, lo, hi);
	child = trace_get_task_struct(pid);
	printk("child = %px\n", child);
	//child->thread.watch.mips3264.watchlo[0] = lo;
	//child->thread.watch.mips3264.watchhi[0] = hi;
	watch.child = child;
	watch.watchlo = lo;
	watch.watchhi = hi;
	jz_on_each_cpu(local_set_watch, &watch);
	//watches = &child->thread.watch.mips3264;
	//watches->trace_type = 1;

	return count;
}

static const struct file_operations cpu_watch_start_ops = {
	.read = cpu_watch_read_start,
	.write = cpu_watch_write_start,
	.open = simple_open,
	.llseek = default_llseek,
};

static int create_debugfs(struct watch_struct *cwt)
{
	struct dentry *d;
	d = debugfs_create_dir("cpu_watch", NULL);
	if (IS_ERR(d)) {
		pr_err("Create debugfs for tcu test failed!\n");
		return PTR_ERR(d);
	}

	cwt->root = d;

	d = debugfs_create_file("start", S_IWUSR | S_IRUGO, cwt->root, cwt, &cpu_watch_start_ops);
	if (IS_ERR_OR_NULL(d)) {
		pr_err("Debugfs create failed!\n");
		goto err_node;
	}

	return 0;

err_node:
	debugfs_remove_recursive(cwt->root);
	return -1;
}

static int __init cpu_watch_test_init(void)
{
	int ret;

	cwt = &_cwt;

	ret = create_debugfs(cwt);

	printk("cpu watch test probe ok!\n");
	return 0;
}

static void __exit cpu_watch_test_deinit(void)
{

}

late_initcall(cpu_watch_test_init);
module_exit(cpu_watch_test_deinit);
