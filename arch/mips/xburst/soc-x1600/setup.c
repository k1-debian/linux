/*
 * INGENIC SOC Setup
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2006 Ingenic Semiconductor Inc.
 */
#include <linux/module.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <linux/irqchip.h>
#include <linux/libfdt.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <asm/prom.h>
#include <soc/cpm.h>
#include <soc/base.h>
#include <soc/ddr.h>

static int ddr_size = 0;
module_param(ddr_size, int, S_IRUGO);
MODULE_PARM_DESC(ddr_size, "ddr size");

extern void *get_fdt_addr(void);

static void __init cpm_reset(void)
{
#ifndef CONFIG_FPGA_TEST
	unsigned long clkgr = cpm_inl(CPM_CLKGR);

	clkgr &= ~(1 << 28      /* DES */
	           | 1 << 26       /* TVE */
	           | 1 << 13       /* SADC */
	          );
	cpm_outl(clkgr, CPM_CLKGR);

	/* TODO set default clkgr here */
#endif
}

static int __init setup_init(void)
{
	cpm_reset();
	/* Set bus priority here */
	*(volatile unsigned int *)0xb34f0240 = 0x00010003;
	*(volatile unsigned int *)0xb34f0244 = 0x00010003;

	return 0;
}

static unsigned int __init init_ddr_size(void)
{
	unsigned int ddr_cfg = 0;
	unsigned int col0 = 0;
	unsigned int row0 = 0;
	unsigned int ba0 = 0;
	unsigned int dq_width_bytes = 16 / 8;   // 固定16bit 位宽.
	ddr_cfg = inl(DDRC_H2_IOBASE + DDRC_CFG);

	col0 = ((ddr_cfg & DDRC_CFG_COL0_MASK) >> DDRC_CFG_COL0_BIT) + 8;
	row0 = ((ddr_cfg & DDRC_CFG_ROW0_MASK) >> DDRC_CFG_ROW0_BIT) + 12;
	ba0  = ((ddr_cfg & (1 << DDRC_CFG_BA0)) >> DDRC_CFG_BA0) == 1 ? 3 : 2; // 3: 2**3 = 8 bank, 2: 2**2 = 4 bank.

	ddr_size = (1 << (col0 + row0 + ba0)) / 1024 / 1024 * dq_width_bytes;   // ddr_size in MBytes.

	printk("ddr_size: %d MBytes\n", ddr_size);

	return  ddr_size;
}

unsigned int __init arch_get_ddr_size(void)
{
	return ddr_size;
}

void __init plat_mem_setup(void)
{

	set_io_port_base(IO_BASE);

	/*Not have ioport*/
	ioport_resource.start   = 0x00000000;
	ioport_resource.end     = 0xffffffff;
	iomem_resource.start    = 0x00000000;
	iomem_resource.end      = 0xffffffff;

	setup_init();

	init_ddr_size();

	__dt_setup_arch(get_fdt_addr());

	return;
}

void __init device_tree_init(void)
{
	unflatten_and_copy_device_tree();
}

static int __init plat_of_populate(void)
{
	of_platform_default_populate(NULL, NULL, NULL);
	return 0;
}
arch_initcall(plat_of_populate);

void __init plat_time_init(void)
{
	of_clk_init(NULL);

	timer_probe();
}

void __init arch_init_irq(void)
{

	irqchip_init();
}
