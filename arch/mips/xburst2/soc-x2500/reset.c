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

#define RTC_RTCCR       (0x00)  /* rw, 32, 0x00000081 */
#define RTC_RTCSR       (0x04)  /* rw, 32, 0x???????? */
#define RTC_RTCSAR      (0x08)  /* rw, 32, 0x???????? */
#define RTC_RTCGR       (0x0c)  /* rw, 32, 0x0??????? */
#define RTC_HCR         (0x20)  /* rw, 32, 0x00000000 */
#define RTC_HWFCR       (0x24)  /* rw, 32, 0x0000???0 */
#define RTC_HRCR        (0x28)  /* rw, 32, 0x00000??0 */
#define RTC_HWCR        (0x2c)  /* rw, 32, 0x00000008 */
#define RTC_HWRSR       (0x30)  /* rw, 32, 0x00000000 */
#define RTC_HSPR        (0x34)  /* rw, 32, 0x???????? */
#define RTC_WENR        (0x3c)  /* rw, 32, 0x00000000 */
#define RTC_CKPCR       (0x40)  /* rw, 32, 0x00000010 */
#define RTC_OWIPCR      (0x44)  /* rw, 32, 0x00000010 */
#define RTC_PWRONCR     (0x48)  /* rw, 32, 0x???????? */

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
#define UNMSAK_SIGNATURE    (0x7c0000)//do not use these bits

#if 0
static void wdt_start_count(int msecs)
{
	int time = JZ_EXTAL_RTC / 64 * msecs / 1000;
	if (time > 65535) {
		time = 65535;
	}

	//  outl(1 << 16,TCU_IOBASE + TCU_TSCR);

	outl(0, WDT_IOBASE + WDT_TCNT);     //counter
	outl(time, WDT_IOBASE + WDT_TDR);   //data
	outl((3 << 3 | 2 << 0 | 1 << 10), WDT_IOBASE + WDT_TCSR);
	outl(0, WDT_IOBASE + WDT_TCER);
	outl(1, WDT_IOBASE + WDT_TCER);
}
#endif
static void __attribute__((unused)) wdt_stop_count(void)
{
	outl(1 << 16, TCU_IOBASE + TCU_TSCR);
	outl(0, WDT_IOBASE + WDT_TCNT);     //counter
	outl(65535, WDT_IOBASE + WDT_TDR);  //data
	outl(1 << 16, TCU_IOBASE + TCU_TSSR);
}

static int inline rtc_write_reg(int reg, int value)
{
	int timeout = 0x2000;
	while (!(inl(RTC_IOBASE + RTC_RTCCR) & RTCCR_WRDY) && timeout--);
	if (!timeout) {
		printk("WARN:NO USE RTC!!!!!\n");
		return -1;
	}
	outl(0xa55a, (RTC_IOBASE + RTC_WENR));
	while (!(inl(RTC_IOBASE + RTC_RTCCR) & RTCCR_WRDY));
	while (!(inl(RTC_IOBASE + RTC_WENR) & WENR_WEN));
	while (!(inl(RTC_IOBASE + RTC_RTCCR) & RTCCR_WRDY));
	outl(value, (RTC_IOBASE + reg));
	while (!(inl(RTC_IOBASE + RTC_RTCCR) & RTCCR_WRDY));

	return 0;
}

/*
 * Function: Keep power for CPU core when reset.
 * So that EPC, tcsm and so on can maintain it's status after reset-key pressed.
 */
static int inline reset_keep_power(void)
{
	return rtc_write_reg(RTC_PWRONCR,
	                     inl(RTC_IOBASE + RTC_PWRONCR) & ~(1 << 0));
}

#define HWFCR_WAIT_TIME(x) ((x > 0x7fff ? 0x7fff: (0x7ff*(x)) / 2000) << 5)
#define HRCR_WAIT_TIME(x) ((((x) > 1875 ? 1875: (x)) / 125) << 11)

void jz_hibernate(void)
{
	uint32_t rtc_rtccr;

	local_irq_disable();
	/* Set minimum wakeup_n pin low-level assertion time for wakeup: 1000ms */
	rtc_write_reg(RTC_HWFCR, HWFCR_WAIT_TIME(1000));

	/* Set reset pin low-level assertion time after wakeup: must  > 60ms */
	rtc_write_reg(RTC_HRCR, HRCR_WAIT_TIME(125));

	/* clear wakeup status register */
	rtc_write_reg(RTC_HWRSR, 0x0);

	rtc_write_reg(RTC_HWCR, 0x0);

	rtc_rtccr = inl(RTC_IOBASE + RTC_RTCCR);
	rtc_rtccr |= 0x1 << 0;
	rtc_write_reg(RTC_RTCCR, rtc_rtccr);

	/* Put CPU to hibernate mode */
	rtc_write_reg(RTC_HCR, 0x1);

	/*poweroff the pmu*/
	//  jz_notifier_call(NOTEFY_PROI_HIGH, JZ_POST_HIBERNATION, NULL);

	mdelay(200);

	while (1) {
		printk("%s:We should NOT come here.%08x\n", __func__, inl(RTC_IOBASE + RTC_HCR));
	}
}

void jz_wdt_restart(char *command)
{
	return;
#if 0
	printk("Restarting after 4 ms\n");
	if ((command != NULL) && !strcmp(command, "recovery")) {
		while (cpm_inl(CPM_CPPSR) != RECOVERY_SIGNATURE) {
			printk("set RECOVERY_SIGNATURE\n");
			cpm_outl(0x5a5a, CPM_CPSPPR);
			cpm_outl(RECOVERY_SIGNATURE, CPM_CPPSR);
			cpm_outl(0x0, CPM_CPSPPR);
			udelay(100);
		}
	} else {
		cpm_outl(0x5a5a, CPM_CPSPPR);
		cpm_outl(REBOOT_SIGNATURE, CPM_CPPSR);
		cpm_outl(0x0, CPM_CPSPPR);
	}

	wdt_start_count(4);
	mdelay(200);
	while (1) {
		printk("check wdt.\n");
	}
#endif
}

static void hibernate_restart(void)
{
	uint32_t rtc_rtcsr, rtc_rtccr;

	while (!(inl(RTC_IOBASE + RTC_RTCCR) & RTCCR_WRDY));

	rtc_rtcsr = inl(RTC_IOBASE + RTC_RTCSR);
	rtc_rtccr = inl(RTC_IOBASE + RTC_RTCCR);

	rtc_write_reg(RTC_RTCSAR, rtc_rtcsr + 5);
	rtc_rtccr &= ~(1 << 4 | 1 << 1);
	rtc_rtccr |= 0x3 << 2;
	rtc_write_reg(RTC_RTCCR, rtc_rtccr);

	/* Clear reset status */
	cpm_outl(0, CPM_RSR);

	/* Set minimum wakeup_n pin low-level assertion time for wakeup: 1000ms */
	rtc_write_reg(RTC_HWFCR, HWFCR_WAIT_TIME(1000));

	/* Set reset pin low-level assertion time after wakeup: must  > 60ms */
	rtc_write_reg(RTC_HRCR, HRCR_WAIT_TIME(125));

	/* clear wakeup status register */
	rtc_write_reg(RTC_HWRSR, 0x0);

	rtc_write_reg(RTC_HWCR, 0x1);


	rtc_rtccr = inl(RTC_IOBASE + RTC_RTCCR);
	rtc_rtccr |= 0x1 << 0;
	rtc_write_reg(RTC_RTCCR, rtc_rtccr);


	/* Put CPU to hibernate mode */
	rtc_write_reg(RTC_HCR, 0x1);

	mdelay(200);
	while (1) {
		printk("%s:We should NOT come here.%08x\n", __func__, inl(RTC_IOBASE + RTC_HCR));
	}
}

#ifdef CONFIG_HIBERNATE_RESET
void jz_hibernate_restart(char *command)
{
	local_irq_disable();

	if ((command != NULL) && !strcmp(command, "recovery")) {
		jz_wdt_restart(command);
	}

	hibernate_restart();
}
#endif

int __init reset_init(void)
{
	pm_power_off = jz_hibernate;
#ifdef CONFIG_HIBERNATE_RESET
	_machine_restart = jz_hibernate_restart;
#else
	_machine_restart = jz_wdt_restart;
#endif
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
static char *reset_command[] = {"wdt", "hibernate", "recovery", "poweroff"};

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
		hibernate_restart();
		break;
	case 2:
		jz_wdt_restart("recovery");
		break;
	case 3:
		jz_hibernate();
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
