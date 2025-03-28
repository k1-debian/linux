#include <linux/init.h>
#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/ctype.h>
#include <linux/gpio.h>

#include <soc/cache.h>
#include <soc/base.h>
#include <asm/io.h>
#include <soc/ddr.h>
#include <soc/base.h>
#include <soc/cpm.h>
#include <soc/gpio.h>

struct sleep_param {
	suspend_state_t state;
	unsigned int pdt;
	unsigned int dpd;
	unsigned int dlp;
	unsigned int autorefresh;
	unsigned int cpu_div;
	unsigned int uart_base;
	unsigned int sleep_level;
	unsigned int cpccr;
	unsigned int cpapcr;
	unsigned int cpmpcr;
	unsigned int cpepcr;
};
#define reg_ddr_phy(x)   (*(volatile unsigned int *)(0xb3011000 + ((x) << 2)))

#define SLEEP_MEMORY_START      0xb2400000
#define SLEEP_MEMORY_END        0xb2401FFC
#define SLEEP_RESUME_SP         SLEEP_MEMORY_END
#define SLEEP_RESUME_BOOTUP_TEXT    SLEEP_MEMORY_START

#define SLEEP_CPU_RESUME_BOOTUP_TEXT    SLEEP_RESUME_BOOTUP_TEXT
#define SLEEP_CPU_RESUME_BOOTUP_LEN 256
#define SLEEP_CPU_RESUME_TEXT       (SLEEP_CPU_RESUME_BOOTUP_TEXT + SLEEP_CPU_RESUME_BOOTUP_LEN)
#define SLEEP_CPU_RESUME_LEN        (2596)
#define SLEEP_CPU_SLEEP_TEXT        (SLEEP_CPU_RESUME_TEXT + SLEEP_CPU_RESUME_LEN)
#define SLEEP_CPU_SLEEP_LEN     (3072)
#define NORMAL_PARAM_ADDR   (SLEEP_CPU_SLEEP_TEXT + SLEEP_CPU_SLEEP_LEN)
#define NORMAL_PARAM_LEN        (sizeof(struct sleep_param))

#define SLEEP_CPU_RESUME_SP     SLEEP_RESUME_SP


#define sleep_param ((struct sleep_param *)NORMAL_PARAM_ADDR)

#define DDRPAPB_BASE    (0xb3011000)
#define REG_INNOPHY_PLL_CTRL    (DDRPAPB_BASE + 0x14c)
#define REG_INNOPHY_PLL_LOCK    (DDRPAPB_BASE + 0x180)
#define REG_INNOPHY_TRAINING_CTRL   (DDRPAPB_BASE + 0x008)
#define REG_INNOPHY_CALIB_MODE  (DDRPAPB_BASE + 0x54)
#define REG_INNOPHY_CALIB_DONE  (DDRPAPB_BASE + 0x184)

// RISCV
#define RISCV_BASE  0xb2a00000
#define CCU_BASE    0xb2200000

/*************************************** gpio ***************************************/
// #define X2600_SLEEP_GPIO GPIO_PE(01)

#define readl(addr)     (*(volatile unsigned int *)(addr))
#define writel(b, addr) (*(volatile unsigned int *)(addr)) = (b)

#define gpio_readl(o)       readl(0xb3601000 + (o))
#define gpio_writel(b, o)   writel(b, 0xb3601000 + (o))

#define cpm_readl(o)        readl(0xb0000000 + (o))
#define cpm_writel(b, o)    writel(b, 0xb0000000 + (o))

#define GPIO_PXPAT0(n)  (0x40 + (n) * 0x1000) /* Port Pattern 0 Register */
#define GPIO_PXPAT0S(n) (0x44 + (n) * 0x1000) /* Port Pattern 0 Set Register */
#define GPIO_PXPAT0C(n) (0x48 + (n) * 0x1000) /* Port Pattern 0 Clear Register */

/*************************************** debug ***************************************/
#define DEBUG_PM

/*#define LOW_POWER_IDLE 1*/

#define PRINT_DEBUG

#ifdef PRINT_DEBUG

#define OFF_TDR         (0x00)
#define OFF_LCR         (0x0C)
#define OFF_LSR         (0x14)
#define LSR_TDRQ        (1 << 5)
#define LSR_TEMT        (1 << 6)

extern void show_wakeup_sources(void);

static int __init suspend_console_setup(char *str);
__setup("console=", suspend_console_setup);

#define U_IOBASE (uart_iobase + 0xa0000000)
#define TCSM_PCHAR(x)                                                   \
	*((volatile unsigned int*)(U_IOBASE+OFF_TDR)) = x;              \
	while ((*((volatile unsigned int*)(U_IOBASE + OFF_LSR)) & (LSR_TDRQ | LSR_TEMT)) != (LSR_TDRQ | LSR_TEMT))
#else
#define TCSM_PCHAR(x)
#endif

#define TCSM_DELAY(x)                       \
	do{                         \
		register unsigned int i = x;                \
		while(i--)                      \
			__asm__ volatile("nop\n\t");            \
	}while(0)

static int uart_iobase = 0x0;
static inline void ddrp_auto_calibration(void)
{
	unsigned int reg_val = ddr_readl(DDRP_INNOPHY_TRAINING_CTRL);
	unsigned int timeout = 0xffffff;
	unsigned int wait_cal_done = DDRP_CALIB_DONE_HDQCFA | DDRP_CALIB_DONE_LDQCFA;

	reg_val &= ~(DDRP_TRAINING_CTRL_DSCSE_BP);
	reg_val |= DDRP_TRAINING_CTRL_DSACE_START;
	ddr_writel(reg_val, DDRP_INNOPHY_TRAINING_CTRL);

	while (!((ddr_readl(DDRP_INNOPHY_CALIB_DONE) & wait_cal_done) == wait_cal_done) && --timeout) {
		TCSM_PCHAR('t');
	}

	if (!timeout) {
		TCSM_PCHAR('f');
	}

	if (ddr_readl(DDRP_INNOPHY_CALIB_ERR) & (1 << 6)) {
		TCSM_PCHAR('t');
	}

	ddr_writel(0, DDRP_INNOPHY_TRAINING_CTRL);
}

static int __init suspend_console_setup(char *str)
{
	char buf[32];
	char *s;
	int bc_idx;

	strncpy(buf, str, sizeof(buf) - 1);

	for (s = buf; *s; s++)
		if (isdigit(*s) || *s == ',') {
			break;
		}

	bc_idx = simple_strtoul(s, NULL, 10);
	switch (bc_idx) {
		case 0:
			uart_iobase = UART0_IOBASE; break;
		case 1:
			uart_iobase = UART1_IOBASE; break;
		case 2:
			uart_iobase = UART2_IOBASE; break;
		case 3:
			uart_iobase = UART3_IOBASE; break;
		default:
			break;
	}

	return 0;
}
static inline void serial_put_hex(unsigned int x)
{
	int i;
	unsigned int d;
	for (i = 7; i >= 0; i--) {
		d = (x  >> (i * 4)) & 0xf;
		if (d < 10) {
			d += '0';
		} else {
			d += 'A' - 10;
		}
		TCSM_PCHAR(d);
	}
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');
}

/*************************************************************************************/

/*-----------------------------------------------------------------------------
 *  extern function declare
 *-----------------------------------------------------------------------------*/
extern long long save_goto(unsigned int func);
extern int restore_goto(unsigned int func);

#if 1
static void load_func_to_tcsm(unsigned int *tcsm_addr, unsigned int *f_addr, unsigned int size)
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
#ifdef LOW_POWER_IDLE
	//unsigned int clk_gate = cpm_inl(CPM_CLKGR);
	unsigned int clk_gate1 = cpm_inl(CPM_CLKGR1);
#endif
	unsigned int lcr = cpm_inl(CPM_LCR);
	unsigned int opcr = cpm_inl(CPM_OPCR);

	lcr &= ~ 0x3; // low power mode: IDLE
	cpm_outl(lcr, CPM_LCR);

	//opcr &= ~(1 << 28);
	opcr &= ~(1 << 30);
	opcr &= ~(1 << 31);
	cpm_outl(opcr, CPM_OPCR);
#ifdef LOW_POWER_IDLE
	/*close the JPEGD :OK*/
	clk_gate1 |= (1 << 18);

	/*close the JPEGE :OK*/
	clk_gate1 |= (1 << 19);

	cpm_outl(clk_gate1, CPM_CLKGR1);
#endif
	return 0;
}


static int soc_pm_sleep(void)
{
	unsigned int lcr = cpm_inl(CPM_LCR);
	unsigned int opcr = cpm_inl(CPM_OPCR);
	unsigned int clk_gate1;

	lcr &= ~ 0x3;
	lcr |= 1 << 0; // low power mode: SLEEP
	cpm_outl(lcr, CPM_LCR);

	opcr &= ~(1 << 31);
	opcr |= (1 << 30) | (1 << 4) | (1 << 24);
	opcr &= ~(1 << 3);
	opcr &= ~(1 << 2);
	cpm_outl(opcr, CPM_OPCR);

	clk_gate1 = cpm_inl(CPM_CLKGR1);
	clk_gate1 &= ~(1 << 11);
	cpm_outl(clk_gate1, CPM_CLKGR1);

	// HOLD RISCV RESET

	return 0;
}

static int soc_post_wakeup(void)
{
	unsigned int lcr = cpm_inl(CPM_LCR);

	printk("post wakeup!\n");

	lcr &= ~0x3; // low power mode: IDLE
	cpm_outl(lcr, CPM_LCR);

	return 0;
}

static noinline void cpu_resume_bootup(void)
{
	*(volatile unsigned int *)0xb2200f00 = 0xbfc00000;
	__asm__ volatile(
	    ".set push	\n\t"
	    ".set noreorder	\n\t"
	    ".set mips32r2	\n\t"
	    "move $29, %0	\n\t"
	    "jr   %1	\n\t"
	    "nop		\n\t"
	    "nop		\n\t"
	    "nop		\n\t"
	    ".set pop	\n\t"
	    :
	    :"r"(SLEEP_CPU_RESUME_SP), "r"(SLEEP_CPU_RESUME_TEXT)
	    :
	);
}

static noinline void cpu_resume(void)
{
	//unsigned int cpu_id;
	TCSM_PCHAR('R');
#ifdef LOW_POWER_IDLE
	/*notice: please wait pll on ,then restore pll ahb0,ahb2 and so on clk div and clk source*/

	/*Restore APLL configuration*/
	cpm_writel(sleep_param->cpapcr, CPM_CPAPCR);
	/*wait for apll on flag*/
	while (!(cpm_readl(CPM_CPAPCR) & (1 << 3))) {
		TCSM_DELAY(10);
	}

	/*Restore MPLL configuration*/
	cpm_writel(sleep_param->cpmpcr, CPM_CPMPCR);
	/*wait for mpll on flag*/
	while (!(cpm_readl(CPM_CPMPCR) & (1 << 3))) {
		TCSM_DELAY(10);
	}

	/*Restore EPLL configuration*/
	cpm_writel(sleep_param->cpepcr, CPM_CPEPCR);
	/*wait for epll on flag*/
	while (!(cpm_readl(CPM_CPEPCR) & (1 << 3))) {
		TCSM_DELAY(10);
	}

	TCSM_DELAY(100);
#endif
#ifdef X2600_SLEEP_GPIO
	/* resume normal voltage */
	gpio_writel(1 << (X2600_SLEEP_GPIO % 32), GPIO_PXPAT0S(X2600_SLEEP_GPIO / 32));
	/* wait voltage to stabilize */
	TCSM_DELAY(100);
	/* resume clk div */
	cpm_writel(0x55700000 | (sleep_param->cpccr & 0xFFFFF), CPM_CPCCR);
	while (cpm_readl(CPM_CPCSR) & 7);
	/* resume clk source */
	cpm_writel(sleep_param->cpccr, CPM_CPCCR);
	while ((cpm_readl(CPM_CPCSR) & 0xf0000000) != 0xf0000000);
#endif

	if ((sleep_param->state == PM_SUSPEND_MEM) || (sleep_param->state == PM_SUSPEND_STANDBY)) {

		int tmp;
		/*notice:please  wait for apll ,mpll,epll stabilize,otherwise read register may be fail*/
		unsigned int ddrc_ctrl = ddr_readl(DDRC_CTRL);
		/*enable pll */
		tmp = ddr_readl(DDRP_INNOPHY_PLL_CTRL);
		tmp &= ~(1 << 3);
		ddr_writel(tmp, DDRP_INNOPHY_PLL_CTRL);

		//serial_put_hex(ddr_readl(DDRP_INNOPHY_PLL_CTRL));
		/* pll lock 0x60 */
		while (!(ddr_readl(DDRP_INNOPHY_PLL_LOCK) & (1 << 2))) {
			//serial_put_hex(ddr_readl(DDRP_INNOPHY_PLL_LOCK));
		}

		TCSM_PCHAR('\r');
		TCSM_PCHAR('\n');
		TCSM_PCHAR('6');
		/* dfi_init_start = 0, wait dfi_init_complete */
		*(volatile unsigned int *)0xb3012000 &= ~(1 << 3);
		while (!(*(volatile unsigned int *)0xb3012004 & 0x1));

		/* bufferen_core = 1 */
		tmp = *(volatile unsigned int *)0xb3012000;
		tmp &= ~(1 << 4);
		*(volatile unsigned int *)0xb3012000 = tmp;

		/* exit sr */
		ddrc_ctrl = ddr_readl(DDRC_CTRL);
		ddrc_ctrl &= ~(1 << 5);
		ddr_writel(ddrc_ctrl, DDRC_CTRL);
		while (ddr_readl(DDRC_STATUS) & (1 << 2));

		TCSM_DELAY(1000);
		TCSM_PCHAR('1');
		ddrp_auto_calibration();
		TCSM_PCHAR('2');

		/* restore ddr auto-sr */
		ddr_writel(sleep_param->autorefresh, DDRC_AUTOSR_EN);
		TCSM_PCHAR('3');

		/* restore ddr LPEN */
		ddr_writel(sleep_param->dlp, DDRC_DLP);

		/* restore ddr deep power down state */
		ddrc_ctrl = ddr_readl(DDRC_CTRL);
		ddrc_ctrl |= sleep_param->pdt;
		ddrc_ctrl |= sleep_param->dpd;
		ddr_writel(ddrc_ctrl, DDRC_CTRL);

		TCSM_PCHAR('4');
	}

	// add n
	/* __sync(); */
	/* __fast_iob(); */
	TCSM_PCHAR('S');
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');
	TCSM_PCHAR('?');
	TCSM_PCHAR('?');
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');

	__asm__ volatile(
	    ".set push	\n\t"
	    ".set mips32r2	\n\t"
	    "jr.hb %0	\n\t"
	    "nop		\n\t"
	    "nop		\n\t"
	    "nop		\n\t"
	    "nop		\n\t"
	    "nop		\n\t"
	    ".set pop 	\n\t"
	    :
	    : "r"(restore_goto)
	    :
	);
}


static noinline void cpu_sleep(void)
{

	//unsigned int ddrc_ctrl = ddr_readl(DDRC_CTRL);
	//unsigned int tmp;
	//unsigned int reg_ddr_dasr;
	//unsigned int reg_ddr_ctrl;

	blast_dcache32();
	blast_scache64();
	__sync();
	__fast_iob();


	TCSM_PCHAR('D');
	if ((sleep_param->state == PM_SUSPEND_MEM) || (sleep_param->state == PM_SUSPEND_STANDBY)) {
		unsigned int tmp, pll;
		unsigned int ddrc_ctrl = ddr_readl(DDRC_CTRL);

		/* save ddr low power state */
		sleep_param->pdt = ddrc_ctrl & DDRC_CTRL_PDT_MASK;
		sleep_param->dpd = ddrc_ctrl & DDRC_CTRL_DPD;
		sleep_param->dlp = ddr_readl(DDRC_DLP);
		sleep_param->autorefresh = ddr_readl(DDRC_AUTOSR_EN);
		/*ddr_writel(ddrc_ctrl,DDRC_CTRL);*/

		/*ddr disable deep power down */
		ddrc_ctrl &= ~(DDRC_CTRL_PDT_MASK);
		ddrc_ctrl &= ~(DDRC_CTRL_DPD);
		ddr_writel(ddrc_ctrl, DDRC_CTRL);

		/*ddr disable LPEN*/
		ddr_writel(0, DDRC_DLP);

		/*ddr disable auto-sr */
		ddr_writel(0, DDRC_AUTOSR_EN);
		tmp = *(volatile unsigned int *)0xa0000000;

		/* DDR self refresh, */
		ddrc_ctrl = ddr_readl(DDRC_CTRL);
		ddrc_ctrl |= 1 << 5;
		ddr_writel(ddrc_ctrl, DDRC_CTRL);
		while (!(ddr_readl(DDRC_STATUS) & (1 << 2)));

		/*bufferen_core = 0 */
		*(volatile unsigned int *)0xb3012000 |= (1 << 4);

		/* dfi_init_start = 1 */
		*(volatile unsigned int *)0xb3012000 |= (1 << 3);

		{
			int i;
			for (i = 0; i < 4; i++) {

				__asm__ volatile("ssnop\t\n");
				__asm__ volatile("ssnop\t\n");
				__asm__ volatile("ssnop\t\n");
				__asm__ volatile("ssnop\t\n");
				__asm__ volatile("ssnop\t\n");
				__asm__ volatile("ssnop\t\n");
				__asm__ volatile("ssnop\t\n");
				__asm__ volatile("ssnop\t\n");
			}
		}
		/*disable pll */
		pll = ddr_readl(DDRP_INNOPHY_PLL_CTRL);
		pll |= 1 << 3;
		ddr_writel(pll, DDRP_INNOPHY_PLL_CTRL);

#ifdef X2600_SLEEP_GPIO
		sleep_param->cpccr = cpm_readl(CPM_CPCCR);
		/* set cpu clk to 24M */
		cpm_writel(0x55700000, CPM_CPCCR);
		while ((cpm_readl(CPM_CPCSR) & 0xf0000007) != 0xf0000000);
		gpio_writel(1 << (X2600_SLEEP_GPIO % 32), GPIO_PXPAT0C(X2600_SLEEP_GPIO / 32));
#endif

#ifdef LOW_POWER_IDLE
		/*disable  EPLL*/
		sleep_param->cpepcr = cpm_readl(CPM_CPEPCR);
		tmp = cpm_readl(CPM_CPEPCR);
		tmp &= ~(1 << 0);
		cpm_writel(tmp, CPM_CPEPCR);

		/*disable  MPLL*/
		sleep_param->cpmpcr = cpm_readl(CPM_CPMPCR);
		tmp = cpm_readl(CPM_CPMPCR);
		tmp &= ~(1 << 0);
		cpm_writel(tmp, CPM_CPMPCR);

		/*disable  APLL*/
		sleep_param->cpapcr = cpm_readl(CPM_CPAPCR);
		tmp = cpm_readl(CPM_CPAPCR);
		tmp &= ~(1 << 0);
		cpm_writel(tmp, CPM_CPAPCR);
#endif
	} // end state if

	TCSM_PCHAR('W');

	__asm__ volatile(
	    ".set push	\n\t"
	    ".set mips32r2	\n\t"
	    "wait		\n\t"
	    "nop		\n\t"
	    "nop		\n\t"
	    ".set pop	\n\t"
	);

	TCSM_PCHAR('N');
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');
	TCSM_PCHAR('?');
	TCSM_PCHAR('?');
	TCSM_PCHAR('?');
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');

	__asm__ volatile(
	    ".set push	\n\t"
	    ".set mips32r2	\n\t"
	    "jr.hb %0	\n\t"
	    "nop		\n\t"
	    ".set pop 	\n\t"
	    :
	    : "r"(SLEEP_CPU_RESUME_BOOTUP_TEXT)
	    :
	);
}


#endif
int x2600_pm_enter(suspend_state_t state)
{
	//unsigned int cpu_id;

	printk("x2600 pm enter!!\n");
#if 1
	sleep_param->uart_base = UART2_IOBASE + 0xa0000000;
	sleep_param->state = state;

	load_func_to_tcsm((unsigned int *)SLEEP_CPU_RESUME_BOOTUP_TEXT, (unsigned int *)cpu_resume_bootup, SLEEP_CPU_RESUME_BOOTUP_LEN);
	load_func_to_tcsm((unsigned int *)SLEEP_CPU_RESUME_TEXT, (unsigned int *)cpu_resume, SLEEP_CPU_RESUME_LEN);
	load_func_to_tcsm((unsigned int *)SLEEP_CPU_SLEEP_TEXT, (unsigned int *)cpu_sleep, SLEEP_CPU_SLEEP_LEN);

	if (state == PM_SUSPEND_STANDBY) {
		soc_pm_idle();
	} else if (state == PM_SUSPEND_MEM) {
		soc_pm_sleep();
	} else {
		printk("WARNING : unsupport pm suspend state\n");
	}

#ifdef DEBUG_PM
	printk("LCR: %08x\n", cpm_inl(CPM_LCR));
	printk("OPCR: %08x\n", cpm_inl(CPM_OPCR));
#endif

	/* set reset entry */
	*(volatile unsigned int *)0xb2200f00 = SLEEP_CPU_RESUME_BOOTUP_TEXT;

	mb();
	save_goto((unsigned int)SLEEP_CPU_SLEEP_TEXT);
	mb();

	soc_post_wakeup();

	show_wakeup_sources();

#endif
	return 0;
}

static int x2600_pm_begin(suspend_state_t state)
{
	printk("x2600 suspend begin\n");
	return 0;
}

static void x2600_pm_end(void)
{
	printk("x2600 pm end!\n");
}

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
static const struct platform_suspend_ops x2600_pm_ops = {
	.valid      = ingenic_pm_valid,
	.begin      = x2600_pm_begin,
	.enter      = x2600_pm_enter,
	.end        = x2600_pm_end,
};

/*
 * Initialize suspend interface
 */
static int __init pm_init(void)
{

#ifdef X2600_SLEEP_GPIO
	gpio_request(X2600_SLEEP_GPIO, "sleep_gpio");
	gpio_direction_output(X2600_SLEEP_GPIO, 1);
#endif

	suspend_set_ops(&x2600_pm_ops);

	return 0;
}

late_initcall(pm_init);
