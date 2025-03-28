/*
 * linux/arch/mips/xburst/soc-m200/common/pm_p0.c
 *
 *  M200 Power Management Routines
 *  Copyright (C) 2006 - 2012 Ingenic Semiconductor Inc.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

#include <linux/init.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/sysctl.h>
#include <linux/delay.h>
#include <linux/syscore_ops.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#include <asm/cacheops.h>
#include <asm/r4kcache.h>
#include <asm/fpu.h>

#include <soc/cache.h>
#include <soc/base.h>
#include <soc/cpm.h>
#include <jz_notifier.h>
#include <soc/tcsm_layout.h>

extern long long save_goto(unsigned int);
extern int restore_goto(void);
extern void show_wakeup_sources(void);

unsigned int pm_firmware_new[] ={
#include "core_sleep.hex"
};

void load_pm_firmware_new(unsigned int addr)
{
	void (*func)(unsigned int addr,unsigned int to);
	unsigned int firmware_size = sizeof(pm_firmware_new);

	if(firmware_size > TCSM_BANK_LEN * 1024)
		printk("WARN: firmware_size %d bigger than"						\
		       "TCSM_BANK_LEN %d\n", firmware_size, TCSM_BANK_LEN * 1024);

	func = (void (*)(unsigned int,unsigned int))addr;
	memcpy((void *)addr,pm_firmware_new,firmware_size);
	func(addr,0);
}
struct sleep_param
{
	unsigned int  pm_core_enter;
	unsigned char pmu_i2c_scl;           //default 0xff
	unsigned char pmu_i2c_sda;           //default 0xff
	unsigned char pmu_addr;               //default 0xff
	unsigned char pmu_reg;                //default 0xff
	unsigned char pmu_register_val;

	unsigned char pmu_pin;               //default 0xff
	unsigned char pmu_pin_func;          //default 0xff
	unsigned char uart_id;          //default 0xff

	unsigned int  prev_resume_pc;  //ddr is self-reflash default 0xffffffff
	unsigned int  post_resume_pc;  //ddr is ok. default 0xffffffff
	unsigned int  prev_sleep_pc;   //after flush cache. default 0xffffffff
	unsigned int  post_sleep_pc;   //before wait. default 0xffffffff

};
struct sleep_save_register
{
	unsigned int lcr;
	unsigned int opcr;
	unsigned int clkgr;
	unsigned int sleep_voice_enable;
	unsigned int ddr_training_space[20];
	suspend_state_t pm_state;
};

static struct sleep_save_register s_reg;
struct sleep_param *sleep_param;

static int soc_pm_enter(suspend_state_t state)
{
	unsigned int lcr,opcr, clkgr;

	/**
	   sleep code use PDMA_TCSM, so must enable pdma and nemc
	 */
	s_reg.clkgr = clkgr = cpm_inl(CPM_CLKGR);
	clkgr &= ~(1 << 21);
	cpm_outl(clkgr, CPM_CLKGR);

	memcpy(&s_reg.ddr_training_space,(void*)0x80000000,sizeof(s_reg.ddr_training_space));
	s_reg.opcr = cpm_inl(CPM_OPCR);
	s_reg.lcr = cpm_inl(CPM_LCR);

	load_pm_firmware_new(SLEEP_TCSM_SPACE);
	sleep_param = (struct sleep_param *)SLEEP_TCSM_SPACE;

	sleep_param->post_resume_pc = (unsigned int)restore_goto;
	sleep_param->uart_id = -1;
	/*
	 *   set OPCR.
	 */
	opcr = s_reg.opcr;
	lcr = s_reg.lcr;

	opcr &= ~((1 << 25) | (1 << 22) | (0xfff << 8) | (1 << 4) | (1 << 3) | (1 << 2));
	opcr |= (1 << 31) | (1 << 30) |  (1 << 25) | (0xfff << 8) | (1 << 4) | (1 << 3);
	lcr &= ~3;

#ifdef CONFIG_RTC_DRV_INGENIC
	opcr &= ~((1 << 4) | (1 << 2));
	opcr |= (1 << 2);
#endif

	if(s_reg.pm_state == PM_SUSPEND_STANDBY) {
		opcr &= ~((1 << 31) | (1 << 30));
	} else {
		lcr |= LCR_LPM_SLEEP;
	}
	cpm_outl(opcr,CPM_OPCR);
	cpm_outl(lcr,CPM_LCR);

	lcr = cpm_inl(CPM_CLKGR);
	lcr &= ~1; 		/* CPM_CLKGR bit0 NEMC: must not gate NEMC clock when sleeping */
	cpm_outl(lcr,CPM_CLKGR);

	printk("#####lcr:%08x\n", cpm_inl(CPM_LCR));
	printk("#####gate:%08x\n", cpm_inl(CPM_CLKGR));
	printk("#####opcr:%08x\n", cpm_inl(CPM_OPCR));
	printk("#####INT_MASK0:%08x\n", *(volatile unsigned int*)(0xB0001004));
	printk("#####INT_MASK1:%08x\n", *(volatile unsigned int*)(0xB0001024));
	printk("#####INT_PEND0:%08x\n", *(volatile unsigned int*)(0xB0001010));
	printk("#####INT_PEND1:%08x\n", *(volatile unsigned int*)(0xB0001030));

	mb();
	save_goto((unsigned int)sleep_param->pm_core_enter);
	mb();

	memcpy((void*)0x80000000,&s_reg.ddr_training_space,sizeof(s_reg.ddr_training_space));
	dma_cache_wback_inv(0x80000000,sizeof(s_reg.ddr_training_space));
	cpm_outl(s_reg.lcr,CPM_LCR);
	cpm_outl(s_reg.opcr,CPM_OPCR);
	cpm_outl(s_reg.clkgr, CPM_CLKGR);

	show_wakeup_sources();

	return 0;
}

static int soc_valid(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_ON:
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		return 1;

	default:
		return 0;
	}
}
static void soc_finish(void)
{
	printk("x1000 pm end irq_num=%d!\n", pm_wakeup_irq() );
	s_reg.pm_state = 0;
}
static int soc_begin(suspend_state_t state)
{
	printk("x1000 suspend begin\n");

	s_reg.pm_state = state;
	if(s_reg.pm_state == PM_SUSPEND_STANDBY) {
		jz_notifier_call(NOTEFY_PROI_HIGH, JZ_PM_SUSPEND_STANDBY, &s_reg.pm_state);
	}
	return 0;
}

/*
 * Initialize power interface
 */
struct platform_suspend_ops pm_ops = {
	.valid = soc_valid,
	.begin = soc_begin,
	.enter = soc_pm_enter,
	.end = soc_finish,
};

int __init soc_pm_init(void)
{
	volatile unsigned int lcr,opcr;

	suspend_set_ops(&pm_ops);

	/* init opcr and lcr for idle */
	lcr = cpm_inl(CPM_LCR);
	lcr &= ~(0x3);		/* LCR.SLEEP.DS=1'b0,LCR.LPM=2'b00*/
	lcr |= 0xff << 8;	/* power stable time */
	cpm_outl(lcr,CPM_LCR);

	opcr = cpm_inl(CPM_OPCR);
	opcr |= 0xff << 8;	/* EXCLK stable time */
	opcr &= ~(1 << 4);	/* EXCLK oscillator is disabled in Sleep mode */
	cpm_outl(opcr,CPM_OPCR);

	/* sysfs */
	return 0;
}

arch_initcall(soc_pm_init);
