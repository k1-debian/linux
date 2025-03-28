/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/tty.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/syscore_ops.h>
#include <linux/platform_device.h>
#include <linux/sched/rt.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/module.h>


#include <soc/base.h>
#include <soc/cpm.h>
#include <soc/extal.h>

#include <asm/reboot.h>

#include <ingenic_proc.h>


#define WDT_TCSR        (0x0c)  /* rw, 32, 0x???????? */
#define WDT_TCER        (0x04)  /* rw, 32, 0x???????? */
#define WDT_TDR         (0x00)  /* rw, 32, 0x???????? */
#define WDT_TCNT        (0x08)  /* rw, 32, 0x???????? */

#define TCU_TSSR    (0x2C)   /* Timer Stop Set Register */
#define TCU_TSCR    (0x3C)   /* Timer Stop Clear Register */

#define RTCCR_WRDY      BIT(7)
#define WENR_WEN                BIT(31)

#define RECOVERY_SIGNATURE  (0x001a1a)
#define REBOOT_SIGNATURE    (0x003535)
#define PANIC_SIGNATURE     (0x004343)
#define UNMSAK_SIGNATURE    (0x7c0000)//do not use these bits

static void wdt_start_count(int msecs)
{
	int time = JZ_EXTAL_RTC / 64 * msecs / 1000;
	if (time > 65535) {
		time = 65535;
	}

	cpm_clear_bit(2, CPM_OPCR); // Select EXTCLK/512 as RTC CLK
	cpm_clear_bit(8, CPM_CLKGR1);   // enable TCU0 Clk.

	outl(1 << 16, WDT_IOBASE + TCU_TSCR);   // enable wdt src clk.

	outl(0, WDT_IOBASE + WDT_TCER);     // disable wdt counter.
	outl(0, WDT_IOBASE + WDT_TCNT);     //counter
	outl(time, WDT_IOBASE + WDT_TDR);   //data
	outl((3 << 3 | 1 << 1 | 1 << 10), WDT_IOBASE + WDT_TCSR);
	outl(1, WDT_IOBASE + WDT_TCER);     // start wdt counter
}

static void __attribute__((unused)) wdt_stop_count(void)
{
	outl(1 << 16, TCU_IOBASE + TCU_TSCR);
	outl(0, WDT_IOBASE + WDT_TCNT);     //counter
	outl(65535, WDT_IOBASE + WDT_TDR);  //data
	outl(1 << 16, TCU_IOBASE + TCU_TSSR);
}

static inline void write_cpm_cpspr(unsigned int val)
{
	while (cpm_inl(CPM_CPSPR) != val) {
		cpm_outl(0x5a5a, CPM_CPSPPR);
		cpm_outl(val, CPM_CPSPR);
		cpm_outl(0x0, CPM_CPSPPR);
		udelay(100);
	}
}

void jz_wdt_restart(char *command)
{
	printk("Restarting after 4 ms\n");
	if ((command != NULL) && !strcmp(command, "recovery")) {
		printk("set RECOVERY_SIGNATURE\n");
		write_cpm_cpspr(RECOVERY_SIGNATURE);
	} else if ((command != NULL) && !strcmp(command, "panic")) {
		write_cpm_cpspr(PANIC_SIGNATURE);
	} else {
		write_cpm_cpspr(REBOOT_SIGNATURE);
	}

	wdt_start_count(4);
	mdelay(200);
	while (1) {
		printk("check wdt.\n");
	}
}



int __init reset_init(void)
{
	//pm_power_off = jz_hibernate;
	_machine_restart = jz_wdt_restart;
	return 0;
}
arch_initcall(reset_init);

///////////////////////////////////////////////////////////////////////////////////////////////////
struct wdt_reset {
	unsigned stop;
	unsigned msecs;
	unsigned count;
};

/* ============================reset proc=================================== */
static char *reset_command[] = {"wdt", "recovery"};

static int reset_show(struct seq_file *filq, void *v)
{
	int len = 0, i;

	for (i = 0; i < ARRAY_SIZE(reset_command); i++) {
		seq_printf(filq, "%s\t", reset_command[i]);
	}
	seq_printf(filq, "\n");

	return len;
}

static int reset_write(struct file *file, const char __user *buffer,
                       size_t usize, loff_t *off)
{
	int command_size = 0;
	int i;

	if (usize == 0) {
		return -EINVAL;
	}

	command_size = ARRAY_SIZE(reset_command);
	for (i = 0; i < command_size; i++) {
		if (!strncmp(buffer, reset_command[i], strlen(reset_command[i]))) {
			break;
		}
	}
	if (i == command_size) {
		return -EINVAL;
	}

	local_irq_disable();
	switch (i) {
		case 0:
			jz_wdt_restart(NULL);
			break;
		case 1:
			jz_wdt_restart("recovery");
			break;
		default:
			printk("not support command %d\n", i);
	}

	return usize;
}
static struct jz_single_file_ops reset_proc_fops = {
	.read = reset_show,
	.write = reset_write,
};
/* ============================reset proc end=============================== */

static int __init init_reset(void)
{
	struct wdt_reset *wdt;
	struct proc_dir_entry *p, *res;

	wdt = kmalloc(sizeof(struct wdt_reset), GFP_KERNEL);
	if (!wdt) {
		return -ENOMEM;
	}

	wdt->count = 0;
	wdt->msecs = 3000;

	wdt->stop = 1;

	p = jz_proc_mkdir("reset");
	if (!p) {
		pr_warn("create_proc_entry for common reset failed.\n");
		return -ENODEV;
	}

	res = jz_proc_create_data("reset", 0444, p, &reset_proc_fops, wdt);

	return 0;
}
module_init(init_reset);
