/*
 * Use this function to generate cpu-feature-override.h head file.
 * Make sure of that, here is no cpu-feature-override.h in mach-xxx directory.
 *
 */

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#define __ASM_MACH_INGENIC_CPU_FEATURE_OVERRIDES_H__
#include <asm/cpu-features.h>
#include <asm/mipsregs.h>

static int cpu_proc_show(struct seq_file *m, void *v)
{

#define PRINT(ARGS...) seq_printf (m, ##ARGS)

#define PRT0(X,Y) PRINT("#define " "%s()\t(%d * 1024)\n",#X,(Y)/1024)
#define PRT1(X,Y) PRINT("#define " "%s()\t%d\n",#X,(Y))
	PRT0(cpu_dcache_size, (1 << current_cpu_data.dcache.waybit)*current_cpu_data.dcache.ways);
	PRT1(cpu_dcache_ways, current_cpu_data.dcache.ways);
	PRT1(cpu_dcache_line_size, current_cpu_data.dcache.linesz);

	PRT0(cpu_icache_size, (1 << current_cpu_data.icache.waybit)*current_cpu_data.icache.ways);
	PRT1(cpu_icache_ways, current_cpu_data.icache.ways);
	PRT1(cpu_icache_line_size, current_cpu_data.icache.linesz);

#define PRT2(X) PRINT("#define " "%-30s\t%d\n",#X,X? 1: 0)
	PRT2(cpu_has_tlb);
	PRT2(cpu_has_4kex);
	PRT2(cpu_has_3k_cache);
	PRT2(cpu_has_4k_cache);
	PRT2(cpu_has_tx39_cache);
	PRT2(cpu_has_fpu);
	PRT2(cpu_has_32fpr);
	PRT2(cpu_has_counter);
	PRT2(cpu_has_watch);
	PRT2(cpu_has_divec);
	PRT2(cpu_has_vce);
	PRT2(cpu_has_cache_cdex_p);
	PRT2(cpu_has_cache_cdex_s);
	PRT2(cpu_has_prefetch);
	PRT2(cpu_has_mcheck);
	PRT2(cpu_has_ejtag);
	PRT2(cpu_has_llsc);
	PRT2(cpu_has_mips16);
	PRT2(cpu_has_mdmx);
	PRT2(cpu_has_mips3d);
	PRT2(cpu_has_smartmips);
	PRT2(cpu_has_vtag_icache);
	PRT2(cpu_has_dc_aliases);
	PRT2(cpu_has_ic_fills_f_dc);
	PRT2(cpu_has_pindexed_dcache);
	PRT2(cpu_icache_snoops_remote_store);
	PRT2(cpu_has_mips32r1);
	PRT2(cpu_has_mips32r2);
	PRT2(cpu_has_mips64r1);
	PRT2(cpu_has_mips64r2);
	PRT2(cpu_has_dsp);
	PRT2(cpu_has_mipsmt);
	PRT2(cpu_has_userlocal);
	PRT2(cpu_has_nofpuex);
	PRT2(cpu_has_64bits);
	PRT2(cpu_has_64bit_zero_reg);
	PRT2(cpu_has_vint);
	PRT2(cpu_has_veic);
	PRT2(cpu_has_inclusive_pcaches);

	PRINT("\n");
	PRINT("cp0 prid:\t%08x\n", read_c0_prid());
	PRINT("cp0 config0:\t%08x\n", read_c0_config());
	PRINT("cp0 config1:\t%08x\n", read_c0_config1());
	PRINT("cp0 config2:\t%08x\n", read_c0_config2());
	PRINT("cp0 config3:\t%08x\n", read_c0_config3());
	PRINT("cp0 config4:\t%08x\n", read_c0_config4());

	return 0;
}

static int cpu_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_proc_show, pde_data(inode));
}
static const struct proc_ops cpu_proc_fops = {
	.proc_read = seq_read,
	.proc_open = cpu_open,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};
static int __init init_cpu_features(void)
{
	proc_create("cpu-feature-override", 0444, NULL, &cpu_proc_fops);

	return 0;
}

module_init(init_cpu_features);
