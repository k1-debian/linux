/*
 * Copyright (C) 2001, 2002, 2003 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

//#define DEBUG
//#define SMP_DEBUG

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/smp.h>
#include <linux/kernel_stat.h>

#include <asm/mmu_context.h>
#include <asm/io.h>
#include <asm/uasm.h>
#include <asm/r4kcache.h>
#include <asm/processor.h>

#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/sched/hotplug.h>
#include <linux/sched/task_stack.h>


#include <dt-bindings/interrupt-controller/mips-irq.h>

#include <core_base.h>
#include <ccu.h>

#ifdef SMP_DEBUG
static void xburst2_ccu_showregs(void)
{
	int cpu = smp_processor_id();
	unsigned int val;
	printk("CPU%d:\n", cpu);
#define P(reg) do {                             \
		val = get_ccu_##reg();                  \
		printk(#reg ":\t%08x\n", val);          \
	} while(0)

	P(cscr); P(cssr); P(csrr); P(pipr); P(pimr);
	P(mipr); P(mimr); P(oipr); P(oimr); P(rer);
	P(cslr); P(csar);
	//P(val);  P(lock);
	printk("cp0 status:\t%08x\n", read_c0_status());
	printk("cp0 cause:\t%08x\n", read_c0_cause());
}
static void dump_code(const unsigned int *handler, const int count)
{
	int i;
	pr_info("\t.set push\n");
	pr_info("\t.set noreorder\n");

	for (i = 0; i < count; i++) {
		pr_info("\t.word\t0x%08x\t\t# %p\n", handler[i], &handler[i]);
	}

	pr_info("\t.set\tpop\n");
}
#else
static inline void xburst2_ccu_showregs(void) {}
static inline void dump_code(const unsigned int *handler, const int count) {}
#endif
struct xburst2_mailbox {
	raw_spinlock_t lock;
	void *__iomem *iobase;
};
struct xburst2_smp {
	unsigned long context_sp, context_gp;
	unsigned long entry_base;
	struct xburst2_mailbox mailbox[NR_CPUS];
	struct xburst2_mailbox *__percpu *percpu_mailbox;
};
static struct xburst2_smp smp_core;
void ingenic_percpu_timerevent_init(unsigned int cpu_num);
void ingenic_percpu_irq_init(unsigned int cpu_num);

static irqreturn_t xburst2_mbox_interrupt(int irq, void *data)
{
	unsigned int action = 0;
	struct xburst2_mailbox *mailbox = *(struct xburst2_mailbox **)data;
	raw_spin_lock(&mailbox->lock);
	action = readl(mailbox->iobase);
	writel(0, mailbox->iobase);
	raw_spin_unlock(&mailbox->lock);
	if (!action) {
		pr_err("SMP[%d]:invalid mailboxes action is NULL\n", smp_processor_id());
		goto ipi_finish;
	}
	if (action & SMP_CALL_FUNCTION) {
		generic_smp_call_function_interrupt();
	}

	if (action & SMP_RESCHEDULE_YOURSELF) {
		scheduler_ipi();
	}
ipi_finish:
	return IRQ_HANDLED;
}

void percpu_mailbox_init(int cpu)
{
	smp_core.mailbox[cpu].iobase = ioremap(CCU_IO_BASE + cpu * 4 + 0x1000, 4);
	raw_spin_lock_init(&smp_core.mailbox[cpu].lock);
	*this_cpu_ptr(smp_core.percpu_mailbox) = &smp_core.mailbox[cpu];
	writel(0, smp_core.mailbox[cpu].iobase);
	enable_percpu_irq(CORE_MAILBOX_IRQ, IRQ_TYPE_NONE);
}
/*
 * Code to run on secondary just after probing the CPU
 */
static void xburst2_init_secondary(void)
{
	int cpu = smp_processor_id();
	unsigned int mb_msk = get_ccu_mimr();
	unsigned int pmsk = get_ccu_pimr();
	unsigned int cpu_num = read_c0_ebase() & 0x1ff;
	//  unsigned int imask = STATUSF_IP4 | STATUSF_IP3 | STATUSF_IP2;
	//  int ret;
	if (cpu == 0) {
		pr_info("BUG: cpu0 is booted.\n");
		dump_stack();
		while (1);
	}
	pr_info("#### now starting init for cpu : %d\n", cpu);
	clear_c0_cause(CAUSEF_IP);
	clear_c0_status(ST0_IM);
	mb_msk |= 1 << cpu;
	set_ccu_mimr(mb_msk);
	percpu_mailbox_init(cpu);
	pr_debug("percpu %x\n", read_c0_status());
	ingenic_percpu_irq_init(cpu_num);
	pmsk |= (1 << cpu);
	set_ccu_pimr(pmsk);

	/* jzcpu_timer_setup(); */

	xburst2_ccu_showregs();
	ingenic_percpu_timerevent_init(cpu_num);
	set_ccu_oimr((1 << cpu) | get_ccu_oimr());
}
static bool migrate_one_irq(int cpu, struct irq_desc *desc)
{
	struct irq_data *d = irq_desc_get_irq_data(desc);
	const struct cpumask *affinity = d->common->affinity;
	struct irq_chip *c;
	bool ret = false;

	if (irqd_is_per_cpu(d) || !cpumask_test_cpu(cpu, affinity)) {
		return false;
	}

	c = irq_data_get_irq_chip(d);
	if (!c->irq_set_affinity) {
		return false;
	}
	if (cpumask_any_and(affinity, cpu_online_mask) >= nr_cpu_ids) {
		affinity = cpu_online_mask;
		ret = true;
	}

	if (c->irq_set_affinity(d, affinity, true) == IRQ_SET_MASK_OK && ret) {
		cpumask_copy(d->common->affinity, affinity);
	}

	return ret;
}

static void migrate_irqs(int cpu)
{
	unsigned int i;
	struct irq_desc *desc;
	for_each_irq_desc(i, desc) {
		raw_spin_lock(&desc->lock);
		migrate_one_irq(cpu, desc);
		raw_spin_unlock(&desc->lock);
	}
}

/*
 * Do any tidying up before marking online and running the idle
 * loop
 */
static void xburst2_smp_finish(void)
{
	int cpu = smp_processor_id();
	migrate_irqs(cpu);
	xburst2_ccu_showregs();
	local_irq_enable();
	pr_info("[SMP] slave cpu%d start up finished.\n", smp_processor_id());
}
static void build_bounce_code(struct xburst2_smp *smp)
{
	unsigned long *spp, *gpp;
	unsigned int entry = (unsigned int)smp_bootstrap;
	unsigned int *p;
	p = (unsigned int *)__get_free_pages(GFP_KERNEL, 0);
	spp = (unsigned long *)&smp->context_sp;
	gpp = (unsigned long *)&smp->context_gp;
	smp->entry_base = (unsigned int)p;

#define STATUS_BITS  (ST0_CU0)
#define CAUSE_BITS   (1 << 27)
#define C0_CAUSE     13,0
#define C0_STATUS    12,0
#define C0_COUNT    9,0


#define C0_CONFIG_0  16,0
#define V0      2
#define V1      3
#define GP      28
#define SP      29
#define RA      31
	/* the instructions'max size is 128 byte */

	/* Disable IFU Small Buffer, system low power opt. */
	UASM_i_MFC0(&p, V0, 16, 7);
	uasm_i_ori(&p, V0, V0, 7 << 3);
	UASM_i_MTC0(&p, V0, 16, 7);

	/* kseg0 cache attribute */
	UASM_i_MFC0(&p, V0, C0_CONFIG_0);
	UASM_i_LA(&p, V1, ~7);
	uasm_i_and(&p, V0, V0, V1);
	uasm_i_ori(&p, V0, V0, 3);
	UASM_i_MTC0(&p, V0, C0_CONFIG_0);

	/* cause to DC disable. and status regitster reset */
	UASM_i_LA(&p, V0, CAUSE_BITS);
	UASM_i_MTC0(&p, V0, C0_CAUSE);

	UASM_i_MTC0(&p, 0, C0_COUNT);

	UASM_i_LA(&p, V0, STATUS_BITS);
	UASM_i_MTC0(&p, V0, C0_STATUS);

	UASM_i_LA(&p, SP, (unsigned long)spp);
	UASM_i_LW(&p, SP, 0, SP);
	UASM_i_LA(&p, GP, (unsigned long)gpp);
	UASM_i_LW(&p, GP, 0, GP);
	UASM_i_LA(&p, RA, entry);
	uasm_i_jr(&p, RA);
	uasm_i_nop(&p);
	dump_code((const unsigned int *)smp->entry_base, 24);
	//the code in ddr should be sure.
	dma_cache_wback_inv(smp->entry_base, 128);
}

/*
 * Setup the PC, SP, and GP of a secondary processor and start it
 * running!
 */
static int xburst2_boot_secondary(int cpu, struct task_struct *idle)
{
	unsigned int reset;//,mb_msk;
	//TODO: clk enable.
	/* set soft reset. */
	pr_info("[SMP] Booting CPU%d ...\n", cpu);
	/* set reset entry! */
	set_ccu_rer(smp_core.entry_base);

	reset = get_ccu_csrr();
	reset |= 1 << cpu;
	set_ccu_csrr(reset);
	smp_core.context_sp = __KSTK_TOS(idle);
	smp_core.context_gp = (unsigned long)task_thread_info(idle);
	wmb();
	/* clear soft reset. */
	reset &= ~(1 << cpu);
	set_ccu_csrr(reset);
	pr_debug("percpu %x\n", read_c0_status());

	return 0;
}

// prepare smp all core register but core0 should be hold.
// the code is initialise core ccu register.

static void __init xburst2_smp_setup(void)
{
	int i, num;
	//set smp all core register disable.
	set_ccu_mimr(0);
	//note:core reset register and core0 shouldn't be set.
	set_ccu_csrr(0xfffe);


	cpumask_clear_cpu(NR_CPUS, (struct cpumask *)cpu_possible_mask);
	cpumask_clear_cpu(NR_CPUS, (struct cpumask *)cpu_present_mask);

	//initial core0 register.

	cpumask_set_cpu(0, (struct cpumask *)cpu_possible_mask);
	cpumask_set_cpu(0, (struct cpumask *)cpu_present_mask);

	__cpu_number_map[0] = 0;
	__cpu_logical_map[0] = 0;

	for (i = 1, num = 0; i < NR_CPUS; i++) {
		cpumask_set_cpu(i, (struct cpumask *)cpu_possible_mask);
		cpumask_set_cpu(i, (struct cpumask *)cpu_present_mask);

		__cpu_number_map[i] = ++num;
		__cpu_logical_map[num] = i;
	}

	pr_info("[SMP] Slave CPU(s) %i available.\n", num);
}

//prepare initial code for every core.
static void __init xburst2_prepare_cpus(unsigned int max_cpus)
{
	int ret;
	pr_info("[SMP] Prepare %d cores., cpu: %d\n", max_cpus, smp_processor_id());//qiao

	if (max_cpus <= 1) {
		return;
	}

	smp_core.percpu_mailbox = alloc_percpu(struct xburst2_mailbox *);
	if (!smp_core.percpu_mailbox) {
		pr_err("ERROR:alloc mailbox percpu fail!\n");
	}

	ret = request_percpu_irq(CORE_MAILBOX_IRQ, xburst2_mbox_interrupt, "jz-mailbox",
	                         smp_core.percpu_mailbox);
	if (ret) {
		pr_err("ERROR: cpu%d request ost error\n", smp_processor_id());
	}
	percpu_mailbox_init(0);
	set_ccu_mimr(1 << 0);
	/* prepare slave cpus entry code */
	build_bounce_code(&smp_core);
	/* set reset entry point */
	set_ccu_rer(smp_core.entry_base);//qiao
	pr_debug("smp_bounce.base: %08lx\n", smp_core.entry_base);
}

static void xburst2_send_ipi_single(int cpu, unsigned int action)
{
	//unsigned int timeout=0x1000000;

	struct xburst2_mailbox *mailbox = per_cpu(*smp_core.percpu_mailbox, cpu);
	unsigned int val;
	unsigned long flags;
#if 0
	while ((readl(mailbox->iobase) & action) && (--timeout));
	if (timeout == 0) {
		pr_err("SMP[%d] action:%d will reenter\n", cpu, action);
	}
#endif

	raw_spin_lock_irqsave(&mailbox->lock, flags);
	val = readl(mailbox->iobase);
	writel(action | val, mailbox->iobase);
	raw_spin_unlock_irqrestore(&mailbox->lock, flags);
}

static void xburst2_send_ipi_mask(const struct cpumask *mask, unsigned int action)
{
	int i;
	for (i = 0; i < NR_CPUS; i++) {
		if (cpumask_test_cpu(i, (struct cpumask *)mask)) {
			xburst2_send_ipi_single(i, action);
		}
	}
}
/**
 * all cpu finish
 */
void xburst2_cpus_done(void)
{
	// FIXME: will remove this code.
	pr_debug("[SMP]: xburst2_cpus_done\n");
}

#ifdef CONFIG_HOTPLUG_CPU

void ingenic_percpu_timerevent_deinit(void);
void ingenic_percpu_irq_deinit(void);
static int xburst2_cpu_disable(void)
{
	unsigned int cpu = smp_processor_id();
	unsigned int val;
	if (cpu == 0) {
		return -EBUSY;
	}
	pr_info("SMP: CPU%d is offline\n", cpu);
	set_cpu_online(cpu, false);
	//  cpumask_clear_cpu(cpu, &cpu_callin_map);
	calculate_cpu_foreign_map();

	val = get_ccu_pimr();
	val &= ~(1 << cpu);
	set_ccu_pimr(val);

	val = get_ccu_oimr();
	val &= ~(1 << cpu);
	set_ccu_oimr(val);

	//val = get_ccu_mimr();
	//val &= ~(1 << cpu);
	//set_ccu_mimr(val);

	ingenic_percpu_irq_deinit();
	ingenic_percpu_timerevent_deinit();
	migrate_irqs(cpu);
	clear_tasks_mm_cpumask(cpu);

	pr_debug("disabled cpu %d\n", cpu);
	return 0;
}

void xburst2_cpu_die(unsigned int cpu)
{
	unsigned int timeout = 0x100000;
	unsigned int reset;
	if (cpu == 0) {
		return;
	}

	while ((get_ccu_cssr() & (1 << cpu)) == 0 && --timeout);
	if (timeout == 0) {
		pr_err("SMP[%d] disabled failed!\n", cpu);
		BUG_ON(1);
	}

	reset = get_ccu_csrr();
	reset |= 1 << cpu;
	set_ccu_csrr(reset); // keep reset state.

	//TODO: clk disable.
	pr_info("SMP[%d] is disabled\n", cpu);
}
#endif
//cpu idle loop will enter it.
void play_dead(void)
{
	unsigned int cpu = smp_processor_id();

	unsigned int val;
	pr_debug("---%s, cpu: %d\n", __func__, cpu);

	val = get_ccu_mimr();
	val &= ~(1 << cpu);
	set_ccu_mimr(val);
	idle_task_exit();
	local_irq_disable();
	local_flush_tlb_all();
	// Isn't smt thread core.
	if (!cpu_online(((cpu + 1) & 1) | (cpu & (~1)))) {
		blast_dcache32();
	}

	__asm__ __volatile__(".set	push            \n\t"
	                     ".set	mips32r2        \n\t"
	                     "sync                   \n\t"
	                     "wait                   \n\t"
	                     "nop                    \n\t"
	                     ".set pop               \n\t"
	                    );
	pr_err("ERROR:SMP[%d]: shouldn't run here.", cpu);
	while (1) {
		__asm__ __volatile__(".set	push        \n\t"
		                     ".set	mips32r2    \n\t"
		                     "wait               \n\t"
		                     "nop                \n\t"
		                     ".set pop           \n\t"
		                    );
	}
}

struct plat_smp_ops xburst2_smp_ops = {
	.send_ipi_single    = xburst2_send_ipi_single,
	.send_ipi_mask      = xburst2_send_ipi_mask,

	.init_secondary     = xburst2_init_secondary,
	.smp_finish         = xburst2_smp_finish,
	.boot_secondary     = xburst2_boot_secondary,

	//kernel init
	.smp_setup          = xburst2_smp_setup,   //platfor init
	//  .cpus_done          = xburst2_cpus_done,
	.prepare_cpus       = xburst2_prepare_cpus,  //main init


#ifdef CONFIG_HOTPLUG_CPU
	/* Ensure this CPU doesn't handle any more interrupts. */
	.cpu_disable        = xburst2_cpu_disable,
	/* This actually kills the CPU. */
	.cpu_die            = xburst2_cpu_die,

#endif
};
