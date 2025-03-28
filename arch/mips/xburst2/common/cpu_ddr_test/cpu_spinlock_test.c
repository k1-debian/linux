#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/syscore_ops.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/vmalloc.h>
#include <linux/list.h>

#include "multithread_test.h"

#define PALLADIUM_TRIGER_2()    do {\
		volatile unsigned int *a; \
		a = (unsigned int *)0xAFFFFFFC; \
		*a = *a; \
	} while(0)

struct cpu_spinlock_test {
	void *multithread;
	spinlock_t lock;
	unsigned int cnt;
	atomic_t acnt;
} cpu_spinlock;


static int cpu_spinlock_test_func(void *param)
{
	struct cpu_spinlock_test *p = (struct cpu_spinlock_test *)param;
	unsigned int c;

	spin_lock(&p->lock);

#if 1
	c = atomic_read(&p->acnt);
	if (p->cnt != c) {
		//  PALLADIUM_TRIGER_2();
		printk("========= spinlock cnt %x != atomic cnt, %x\n", p->cnt, c);
		return -1;
	}
#endif
	p->cnt++;

	udelay(10);

	atomic_inc(&p->acnt);
	spin_unlock(&p->lock);

#ifdef CONFIG_FPGA_TEST
	mdelay(100);
#endif
	return 0;
}

static int __init cpu_spinlock_test_init(void)
{
	cpu_spinlock.multithread = multithread_test_init("cpu_spinlock_test", 1);
	spin_lock_init(&cpu_spinlock.lock);

	if (cpu_spinlock.multithread) {
		multithread_test_add_func(cpu_spinlock.multithread, "cpu_spinlock_test", cpu_spinlock_test_func, (void *)&cpu_spinlock);
	}
	return 0;
}
static void __exit cpu_spinlock_test_deinit(void)
{
}

late_initcall(cpu_spinlock_test_init);
module_exit(cpu_spinlock_test_deinit);
