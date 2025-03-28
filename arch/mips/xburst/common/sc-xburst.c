/*
 * Copyright (C) 2006 Chris Dearman (chris@mips.com),
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mm.h>

#include <asm/mipsregs.h>
#include <asm/bcache.h>
#include <asm/cacheops.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/mmu_context.h>
#include <asm/r4kcache.h>

/*
 * MIPS32/MIPS64 L2 cache handling
 */
static unsigned long scache_size __read_mostly;
static void (* r4k_blast_scache)(void);

void __weak platform_early_l2_init(void)
{
}

static void r4k_blast_scache_setup(void)
{
	unsigned long sc_lsize = cpu_scache_line_size();

	/*default*/
	r4k_blast_scache = blast_scache32;

	if (scache_size == 0) {
		//r4k_blast_scache = (void *)blast_inclusive_scache;
		r4k_blast_scache = cache_noop;
	} else if (sc_lsize == 16)
		r4k_blast_scache = blast_scache16;
	else if (sc_lsize == 32)
		r4k_blast_scache = blast_scache32;
	else if (sc_lsize == 64)
		r4k_blast_scache = blast_scache64;
	else if (sc_lsize == 128)
		r4k_blast_scache = blast_scache128;
}

/*
 * Writeback and invalidate the secondary cache before DMA.
 */
static void mips_sc_wback_inv(unsigned long addr, unsigned long size)
{
	__sync();

	if (size >= scache_size)
		r4k_blast_scache();
	else
		blast_scache_range(addr, addr + size);

	__fast_iob();
}

/*
 * Invalidate the secondary cache before DMA.
 */
static void mips_sc_inv(unsigned long addr, unsigned long size)
{
	unsigned long lsize = cpu_scache_line_size();
	unsigned long almask = ~(lsize - 1);

	if (size >= scache_size) {
		r4k_blast_scache();
	}else {
		cache_op(Hit_Writeback_Inv_SD, addr & almask);
		cache_op(Hit_Writeback_Inv_SD, (addr + size - 1) & almask);
		blast_inv_scache_range(addr, addr + size);
	}

	__fast_iob();
}

static struct bcache_ops mips_sc_ops = {
	.bc_enable = (void *)cache_noop,
	.bc_disable = (void *)cache_noop,
	.bc_wback_inv = mips_sc_wback_inv,
	.bc_inv = mips_sc_inv,
};

static inline int __init mips_sc_probe(void)
{
	struct cpuinfo_mips *c = &current_cpu_data;
	unsigned int config1, config2;
	unsigned int tmp;

	/* Mark as not present until probe completed */
	c->scache.flags |= MIPS_CACHE_NOT_PRESENT;

	/* Ignore anything but MIPSxx processors */
	if (!(c->isa_level & (MIPS_CPU_ISA_M32R1 | MIPS_CPU_ISA_M32R2 |
			MIPS_CPU_ISA_M32R6 | MIPS_CPU_ISA_M64R1 |
			MIPS_CPU_ISA_M64R2 | MIPS_CPU_ISA_M64R6)))
		return 0;

	/*
	 * Do we need some platform specific probing before
	 * we configure L2?
	 */
	platform_early_l2_init();

	/* Does this MIPS32/MIPS64 CPU have a config2 register? */
	config1 = read_c0_config1();
	if (!(config1 & MIPS_CONF_M))
		return 0;

	config2 = read_c0_config2();
	
	tmp = (config2 >> 4) & 0x0f;
	if (0 < tmp && tmp <= 7)
		c->scache.linesz = 2 << tmp;
	else
		return 0;

	tmp = (config2 >> 8) & 0x0f;
	if (tmp <= 7)
		c->scache.sets = 64 << tmp;
	else
		return 0;

	tmp = (config2 >> 0) & 0x0f;
	if ((tmp <= 7) || (tmp == 15))
		c->scache.ways = tmp + 1;
	else
		return 0;

	c->scache.waysize = c->scache.sets * c->scache.linesz;
	c->scache.waybit = __ffs(c->scache.waysize);
	c->scache.flags &= ~MIPS_CACHE_NOT_PRESENT;

	scache_size = c->scache.ways * c->scache.sets * c->scache.linesz;
	write_c0_ecc(0x0);

	r4k_blast_scache_setup();
	return 1;
}

static char *way_string[] = { NULL, "direct mapped", "2-way",
	"3-way", "4-way", "5-way", "6-way", "7-way", "8-way",
	"9-way", "10-way", "11-way", "12-way",
	"13-way", "14-way", "15-way", "16-way",
};

int ingenic_sc_init(void)
{
	struct cpuinfo_mips *c = &current_cpu_data;
	int found = mips_sc_probe();

	if (found) {
		bcops = &mips_sc_ops;
		bc_enable();
		bc_prefetch_enable();
	}

	if (!scache_size)
		return 0;

	printk("Unified secondary cache %ldkB %s, linesize %d bytes.\n",
	       scache_size >> 10, way_string[c->scache.ways], c->scache.linesz);

	return found;
}
