/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  INGENIC SoC prom code
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/of_fdt.h>
#include <asm/fw/fw.h>
#include <asm/io.h>
#include <linux/kernel.h>
#ifdef CONFIG_XBURST_MXUV2
	#include <asm/current.h>
	#include <linux/sched.h>
#endif

extern struct plat_smp_ops ingenic_soc_smp_ops;

static void *_fw_fdt_addr;

void __init prom_init(void)
{
	fw_init_cmdline();

	if (fw_arg0 == 0 && fw_arg1 == 0xffffffffUL) {
		_fw_fdt_addr = phys_to_virt(fw_arg2);
	} else if ((int)fw_arg0 == -2) { /*UHI*/
		_fw_fdt_addr = (void *)fw_arg1;
	} else if ((void *)__dtb_start != (void *)__dtb_end) {
		_fw_fdt_addr = __dtb_start;
	} else {
		panic("no dtb found!\n");
	}

	//mips_machtype = MACH_INGENIC_X1600;
}

void __init prom_free_prom_memory(void)
{
}

const char *get_system_type(void)
{
	return "XBurst-Based";
}

void __init *get_fdt_addr(void)
{
	return _fw_fdt_addr;
}
#ifdef CONFIG_XBURST_MXUV2
noinline struct xburst_cop2_state *get_current_cp2(void)
{
	return &(current->thread.cp2);
}
#endif
