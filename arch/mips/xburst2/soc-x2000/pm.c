#include <linux/init.h>
#include <linux/pm.h>
#include <linux/ctype.h>
#include <linux/module.h>

#include <soc/gpio.h>
#include <linux/gpio.h>

#include <soc/cache.h>
#include <soc/base.h>
#include <asm/io.h>
#include <soc/ddr.h>
#include <soc/base.h>
#include <soc/cpm.h>
#include <soc/pm.h>
#include <soc/pm_sleep.h>
#include <soc/pm_fastboot.h>
#if defined CONFIG_KP_AXP
	#include <linux/regulator/machine.h>
#endif


#define SLEEP_LEVEL     (*(volatile unsigned int *)0xb0000034 & 0xf)
#define SLEEP_LEVEL_MAGIC   ((*(volatile unsigned int *)0xb0000034 >> 16) & 0xffff)
#define SLEEP_MAGIC     (0x0915)


#ifdef DEBUG_PM
static void x2000_pm_gate_check(void)
{
	unsigned int gate0 = cpm_inl(CPM_CLKGR);
	unsigned int gate1 = cpm_inl(CPM_CLKGR1);
	int i;
	int x;

	printk("gate0 = 0x%08x\n", gate0);
	printk("gate1 = 0x%08x\n", gate1);
	for (i = 0; i < 32; i++) {
		x = (gate0 >> i) & 1;
		if (x == 0) {
			printk("warning : bit[%d] in clk gate0 is enabled\n", i);
		}
	}
	for (i = 0; i < 32; i++) {
		x = (gate1 >> i) & 1;
		if (x == 0) {
			printk("warning : bit[%d] in clk gate1 is enabled\n", i);
		}
	}

}
#endif



void load_func_to_tcsm(unsigned int *tcsm_addr, unsigned int *f_addr, unsigned int size)
{
	unsigned int instr;
	int offset;
	int i;
#ifdef DEBUG_PM
	printk("tcsm addr = %p %p size = %d\n", tcsm_addr, f_addr, size);
#endif
	for (i = 0; i < size / 4; i++) {
		instr = f_addr[i];
		if ((instr >> 26) == 2) {
			offset = instr & 0x3ffffff;
			offset = (offset << 2) - ((unsigned int)f_addr & 0xfffffff);
			if (offset > 0) {
				offset = ((unsigned int)tcsm_addr & 0xfffffff) + offset;
				instr = (2 << 26) | (offset >> 2);
			}
		}
		tcsm_addr[i] = instr;
	}
}

static int soc_pm_idle(void)
{
	printk("soc pm idle \n");

	load_func_to_sram();

	soc_pm_idle_config();

	return 0;
}

static int soc_pm_idle_pd(void)
{
	printk("soc pm idle pd \n");

	load_func_to_sram();

	soc_pm_idle_pd_config();

	soc_set_reset_entry();

	return 0;
}

static void soc_pm_sleep(void)
{
	printk("soc pm sleep \n");

	load_func_to_sram();

	soc_pm_sleep_config();

	soc_set_reset_entry();
}


static void soc_pm_fastboot(void)
{
	printk("soc pm fastboot \n");

	soc_pm_fastboot_config();

	load_func_to_rtc_ram();

	sys_save();
}


static void goto_sleep(unsigned int sleep_addr)
{
	mb();
	save_goto(sleep_addr);
	mb();
}

int x2000_pm_enter(suspend_state_t state)
{
	unsigned int sleep_addr = 0;
	unsigned int sleep_level = 0;

	printk("x2000 pm enter!!\n");

	sleep_param->uart_base = (UART0_IOBASE | 0xa0000000) + UART_OFF * bc_idx;
	sleep_param->state = state;

	if (SLEEP_LEVEL_MAGIC == SLEEP_MAGIC) {
		if ((state == PM_SUSPEND_STANDBY) && (SLEEP_LEVEL == IDLE)) {
			soc_pm_idle();
			sleep_addr = NORMAL_SLEEP_CODE_ADDR;
		} else if ((state == PM_SUSPEND_STANDBY) && (SLEEP_LEVEL == IDLE_PD)) {
			soc_pm_idle_pd();
			sleep_addr = NORMAL_SLEEP_CODE_ADDR;
		} else if ((state == PM_SUSPEND_MEM) && (SLEEP_LEVEL == SLEEP)) {
			soc_pm_sleep();
			sleep_addr = NORMAL_SLEEP_CODE_ADDR;
		} else if ((state == PM_SUSPEND_MEM) && (SLEEP_LEVEL == FASTBOOT)) {
			soc_pm_fastboot();
			sleep_addr = FASTBOOT_SLEEP_CODE_ADDR;
			sleep_level = FASTBOOT;
		} else {
			printk("error suspend state or suspend level ! \n");
			return -1;
		}
	} else {
		if (state == PM_SUSPEND_STANDBY) {
			soc_pm_idle();
			sleep_addr = NORMAL_SLEEP_CODE_ADDR;
		} else if (state == PM_SUSPEND_MEM) {
			soc_pm_sleep();
			sleep_addr = NORMAL_SLEEP_CODE_ADDR;
		} else {
			printk("error suspend state ! \n");
			return -1;
		}
	}


#ifdef DEBUG_PM
	printk("LCR: %08x\n", cpm_inl(CPM_LCR));
	printk("OPCR: %08x\n", cpm_inl(CPM_OPCR));
	x2000_pm_gate_check();
#endif


	goto_sleep(sleep_addr);


	if (sleep_level == FASTBOOT) {
		soc_pm_wakeup_fastboot();
	} else {
		soc_pm_wakeup_idle_sleep();
	}

	return 0;
}

static void pm_set_sleep_level(suspend_state_t state)
{

	if (SLEEP_LEVEL_MAGIC == SLEEP_MAGIC) {
		if ((state == PM_SUSPEND_STANDBY) && (SLEEP_LEVEL == IDLE)) {
			*(volatile unsigned int *)0xb0000034 |= 0x8 << 4;
		} else if ((state == PM_SUSPEND_STANDBY) && (SLEEP_LEVEL == IDLE_PD)) {
			*(volatile unsigned int *)0xb0000034 |= 0x9 << 4;
		} else if ((state == PM_SUSPEND_MEM) && (SLEEP_LEVEL == SLEEP)) {
			*(volatile unsigned int *)0xb0000034 |= 0xa << 4;
		} else if ((state == PM_SUSPEND_MEM) && (SLEEP_LEVEL == FASTBOOT)) {
			*(volatile unsigned int *)0xb0000034 |= 0xb << 4;
		}
	}
	return;
}

int pm_get_sleep_level(void)
{
	int fastboot_flag = 0;
	fastboot_flag = (*(volatile unsigned int *)0xb0000034 & 0xff);
	switch (fastboot_flag) {
	case 0x80:
		return IDLE;
	case 0x91:
		return IDLE_PD;
	case 0xa2:
		return SLEEP;
	case 0xb3:
		return FASTBOOT;
	}
	return -1;
}
EXPORT_SYMBOL_GPL(pm_get_sleep_level);

static void pm_clear_sleep_level(void)
{
	if (SLEEP_LEVEL_MAGIC == SLEEP_MAGIC) {
		*(volatile unsigned int *)0xb0000034 &= ~(0xf << 4);
	}
	return;
}


static int x2000_pm_begin(suspend_state_t state)
{
	printk("x2000 suspend begin!\n");
	pm_set_sleep_level(state);
	return 0;
}

static void x2000_pm_end(void)
{
	printk("x2000 pm end!\n");
	pm_clear_sleep_level();

}
#if defined CONFIG_KP_AXP
static int x2000_suspend_prepare(void)
{
	return regulator_suspend_prepare(PM_SUSPEND_MEM);
}
static void x2000_suspend_finish(void)
{
	if (regulator_suspend_finish()) {
		pr_err("%s: Suspend finish failed\n", __func__);
	}
}
#endif

static int ingenic_pm_valid(suspend_state_t state)
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
static const struct platform_suspend_ops x2000_pm_ops = {
	.valid      = ingenic_pm_valid,
	.begin      = x2000_pm_begin,
	.enter      = x2000_pm_enter,
	.end        = x2000_pm_end,
#if defined CONFIG_KP_AXP
	.prepare = x2000_suspend_prepare,
	.finish = x2000_suspend_finish,
#endif
};


int bc_idx = 0;
static int __init suspend_console_setup(char *str)
{

	char buf[32];
	char *s;

	strncpy(buf, str, sizeof(buf) - 1);

	for (s = buf; *s; s++)
		if (isdigit(*s) || *s == ',') {
			break;
		}

	bc_idx = simple_strtoul(s, NULL, 10);

	return 0;
}

__setup("console=", suspend_console_setup);




/*
 * Initialize suspend interface
 */
static int __init pm_init(void)
{

#ifdef X2000_SLEEP_GPIO
	gpio_request(X2000_SLEEP_GPIO, "sleep_gpio");
	gpio_direction_output(X2000_SLEEP_GPIO, 1);
#endif

	suspend_set_ops(&x2000_pm_ops);


	return 0;
}

late_initcall(pm_init);
