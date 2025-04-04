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
#include <ccu.h>

#define PALLADIUM_TRIGER_2()    do {\
		volatile unsigned int *a; \
		a = (unsigned int *)0xAFFFFFFC; \
		*a = *a; \
	} while(0)

struct cpu_ccu_test {
	void *multithread;
	unsigned int cnt;
} cpu_ccu;


static int cpu_ccu_test_func(void *param)
{
	struct cpu_ccu_test *p = (struct cpu_ccu_test *)param;
	unsigned int cscr, cssr, csrr, rer;

	cscr = get_ccu_cscr();

	cssr = get_ccu_cssr();

	csrr = get_ccu_csrr();

	rer = get_ccu_rer();

	p->cnt++;

	if (p->cnt == 1000) {
		p->cnt = 0;
		printk("%s: cscr=0x%08x, cssr=0x%08x, csrr=0x%08x, rer=0x%08x\n", __func__, cscr, cssr, csrr, rer);
	}
#ifdef CONFIG_FPGA_TEST
	mdelay(100);
#endif
	return 0;
}

static int __init cpu_ccu_test_init(void)
{
	cpu_ccu.multithread = multithread_test_init("cpu_ccu_test", 1);

	if (cpu_ccu.multithread) {
		multithread_test_add_func(cpu_ccu.multithread, "cpu_ccu_test", cpu_ccu_test_func, (void *)&cpu_ccu);
	}
	return 0;
}
static void __exit cpu_ccu_test_deinit(void)
{
}

late_initcall(cpu_ccu_test_init);
module_exit(cpu_ccu_test_deinit);
