/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  JZ4740 SoC prom code
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <asm/bootinfo.h>
#include <linux/of_fdt.h>

#include <soc/base.h>

static __init void prom_init_cmdline(int argc, char *argv[])
{
	unsigned int count = COMMAND_LINE_SIZE - 1;
	int i;
	char *dst = &(arcs_cmdline[0]);
	char *src;

	for (i = 1; i < argc && count; ++i) {
		src = argv[i];
		while (*src && count) {
			*dst++ = *src++;
			--count;
		}
		*dst++ = ' ';
	}
	if (i > 1) {
		--dst;
	}

	*dst = 0;
}
extern struct plat_smp_ops xburst2_smp_ops;
static void *_fw_fdt_addr;

void __init prom_init(void)
{
	prom_init_cmdline((int)fw_arg0, (char **)fw_arg1);

	if (fw_arg0 == 0 && fw_arg1 == 0xffffffffUL) {
		_fw_fdt_addr = phys_to_virt(fw_arg2);
	} else if ((int)fw_arg0 == -2) { /*UHI*/
		_fw_fdt_addr = (void *)fw_arg1;
	} else if ((void *)__dtb_start != (void *)__dtb_end) {
		_fw_fdt_addr = __dtb_start;
	} else {
		panic("no dtb found!\n");
	}

	mips_machtype = MACH_INGENIC_X2000;
#ifdef CONFIG_SMP
	register_smp_ops(&xburst2_smp_ops);
#endif
}

void __init prom_free_prom_memory(void)
{
}

const char *get_system_type(void)
{
	//  return CONFIG_BOARD_NAME;
	return "xburst2-based";
}
void __init *get_fdt_addr(void)
{
	return _fw_fdt_addr;
}
