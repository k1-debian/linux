#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <asm/prom.h>
#include <soc/base.h>

#define CLKGATE		0x20
#define CTRL_OFF	0x0
#define STATUS_OFF	0x8
#define DATA0_OFF	0xc

enum socid {
	X1000 = 0xff00,
	X1000E = 0xff01,
	X1500 = 0xff02,
	X1000_NEW = 0xff08,
	X1000E_NEW = 0xff09,
	X1500_NEW = 0xff0a,
};

static int read_socid(void)
{
	void __iomem *efuse_iobase = (void __iomem *)CKSEG1ADDR(EFUSE_IOBASE);
	void __iomem *cpm_iobase = (void __iomem *)CKSEG1ADDR(CPM_IOBASE);
	uint32_t val = 0, timeout = 10;
	uint32_t clkgat, clk_efuse_enable = 0;
	int ret = 0;

	clkgat = readl(cpm_iobase + CLKGATE);
	if(clkgat & (0x1 << 1)) {
		clk_efuse_enable = 1;
		writel(clkgat & ~(0x1 << 1), cpm_iobase + CLKGATE);
	}

	writel(0, efuse_iobase + STATUS_OFF);

	val = 0x3c << 21 | 1 << 16 | 1;
	writel(val, efuse_iobase + CTRL_OFF);

	while(!(readl(efuse_iobase + STATUS_OFF) & 1)) {
		timeout --;
		if(!timeout) {
			ret = -EBUSY;
			goto efuse_fail;
		}
	}
	ret = readl(efuse_iobase + DATA0_OFF);
efuse_fail:
	if(clk_efuse_enable)
		writel(clkgat, cpm_iobase + CLKGATE);
	return ret;
}

static int __init check_socid(void) {

	int socid = read_socid();
	if (socid < 0) {
		pr_err("socid: efuse busy !\n");
		return -EBUSY;
	}
	switch(socid) {
	case X1000:
	case X1500:
	case X1000E:
	case X1000_NEW:
	case X1000E_NEW:
	case X1500_NEW:
	case 0:
		break;
	default:
		pr_err("socid: unknown x1000 socid !\n");
		return -ENODEV;
	}

	return socid;
}

static void __init ddr_param_change(int ddr_size)
{
	char *rmem = strstr(arcs_cmdline, " rmem=");
	char *mem = strstr(arcs_cmdline, " mem=");

	int rmem_size = 0;
	if (rmem) {
		rmem += 6;
		rmem_size = simple_strtoul(rmem, NULL, 10);
	}
	if (mem)
		mem += 5;

	if (ddr_size < 64)
		return;

	if (mem) {
		char buf[64];
		sprintf(buf, "%dM@0x0", ddr_size-rmem_size);
		memcpy(mem, buf, strlen(buf));
	}

	if (mem && rmem) {
		char buf[64];
		sprintf(buf, "%dM@0x%x", rmem_size, (ddr_size-rmem_size)*1024*1024);
		memcpy(rmem, buf, strlen(buf));
	}
}

void __init soc_is_x1000e(void)
{
	int socid;

	socid = check_socid();
	if(socid == X1000E || socid == X1000E_NEW){
		ddr_param_change(64);
	}
}

MODULE_DESCRIPTION("x1000 socid driver special used by itself");
