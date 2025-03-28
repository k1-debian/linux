#include <linux/sched/task_stack.h>
#include <linux/ptrace.h>
#include <linux/sched.h>
#include <asm/processor.h>
#include <mxuv3.h>

#define WORDCODE

__attribute__((optimize("O0")))
void __init_mxuv3(void)
{
	unsigned int register val asm("t0");
	val = 0;
	/* open mxuv3 and set thread status ST0_CU2 */
	set_c0_status(ST0_CU2);
	KSTK_STATUS(current) |= ST0_CU2;
	//printk("%s(%d):smp_processor_id()=%d, read_c0_status()=0x%x, task_cpu(current)=%d, KSTK_STATUS(current)=0x%lx, current->tgid=%d, current->pid=%d\n", __func__, __LINE__, smp_processor_id(), read_c0_status(), task_cpu(current), KSTK_STATUS(current), current->tgid, current->pid);

	/* dump mxuv3 control and status register */
#ifdef WORDCODE
	//asm volatile(".word   0x70100203 | (0x0 << 11)\n\t":"=r"(val));
	//printk("%s(%d):smp_processor_id()=%d, read_c0_status()=0x%x, KSTK_STATUS(current)=0x%lx, current->tgid=%d, current->pid=%d, mir=0x%x\n", __func__, __LINE__, smp_processor_id(), read_c0_status(), KSTK_STATUS(current), current->tgid, current->pid, val);
	//asm volatile(".word   0x70100203 | (0x1 << 11) \n\t":"=r"(val));
	//printk("%s(%d):smp_processor_id()=%d, read_c0_status()=0x%x, KSTK_STATUS(current)=0x%lx, current->tgid=%d, current->pid=%d, mscr=0x%x\n", __func__, __LINE__, smp_processor_id(), read_c0_status(), KSTK_STATUS(current), current->tgid, current->pid, val);
	//dump_stack();
#else
	CFCMXU(val, 0);
	printk("mir=0x%x\n", val);
	CFCMXU(val, 1);
	printk("mcsr=0x%x\n", val);
#endif
}

__attribute__((optimize("O0")))
void __save_mxuv3(void *tsk_void)
{
	register void *base asm("t0");
#ifndef WORDCODE
	register void *base1 asm("t1");
#endif
	struct task_struct *tsk = tsk_void;
	struct xburst_mxu_struct *mxu = &tsk->thread.mxu;
	//printk("%s(%d):smp_processor_id()=%d, read_c0_status()=0x%x, KSTK_STATUS(current)=0x%lx, current->tgid=%d, current->pid=%d, , KSTK_STATUS(tsk)=0x%lx, task->tgid=%d, task->pid=%d\n", __func__, __LINE__, smp_processor_id(), read_c0_status(), KSTK_STATUS(current), current->tgid, current->pid, KSTK_STATUS(tsk), tsk->tgid, tsk->pid);
	//dump_stack();
	//printk("%s(%d):smp_processor_id()=%d, read_c0_status()=0x%x, task_cpu(current)=%d, KSTK_STATUS(current)=0x%lx, current->tgid=%d, current->pid=%d, task_cpu(p)=%d, KSTK_STATUS(p)=0x%lx, p->tgid=%d, p->pid=%d\n", __func__, __LINE__, smp_processor_id(), read_c0_status(), task_cpu(current), KSTK_STATUS(current), current->tgid, current->pid, task_cpu(tsk), KSTK_STATUS(tsk), tsk->tgid, tsk->pid);
#ifdef WORDCODE
	/**
	 * sao ins: hex(0x700000d5 | 8 << 21 | 1 << 16 | 0 << 11 | 1 << 9)
	 * 0x700000d5   ->sa0
	 * 8            ->t0
	 * 1            ->32>>5
	 * 0            ->vpr0
	 * 1            ->nn    // o means to be divided by 512/256=2, so n=[0,1]
	 * because t0 is fixed, so 0x700000d5 | 8 << 21 = 0x710000d5
	 * so the word format as follows:
	 * asm volatile(".word  0x710000d5 | offset << 16 | vprn << 11 | n << 9 \n\t");
	 * mfsum ins: hex(0x4a60000f | vss << 11 | vrd << 7)
	 * for we fixed vrd to 0, so the word format as follows:
	 * asm volatile(".word 0x4a60000f | vss << 11 \n\t");
	 */
	base = &mxu->vpr[0];
	asm volatile(".word	0x710000d5 | 0 << 16 | 0 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 0 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[1];
	asm volatile(".word	0x710000d5 | 0 << 16 | 1 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 1 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[2];
	asm volatile(".word	0x710000d5 | 0 << 16 | 2 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 2 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[3];
	asm volatile(".word	0x710000d5 | 0 << 16 | 3 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 3 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[4];
	asm volatile(".word	0x710000d5 | 0 << 16 | 4 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 4 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[5];
	asm volatile(".word	0x710000d5 | 0 << 16 | 5 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 5 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[6];
	asm volatile(".word	0x710000d5 | 0 << 16 | 6 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 6 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[7];
	asm volatile(".word	0x710000d5 | 0 << 16 | 7 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 7 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[8];
	asm volatile(".word	0x710000d5 | 0 << 16 | 8 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 8 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[9];
	asm volatile(".word	0x710000d5 | 0 << 16 | 9 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 9 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[10];
	asm volatile(".word	0x710000d5 | 0 << 16 | 10 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 10 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[11];
	asm volatile(".word	0x710000d5 | 0 << 16 | 11 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 11 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[12];
	asm volatile(".word	0x710000d5 | 0 << 16 | 12 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 12 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[13];
	asm volatile(".word	0x710000d5 | 0 << 16 | 13 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 13 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[14];
	asm volatile(".word	0x710000d5 | 0 << 16 | 14 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 14 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[15];
	asm volatile(".word	0x710000d5 | 0 << 16 | 15 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 15 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[16];
	asm volatile(".word	0x710000d5 | 0 << 16 | 16 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 16 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[17];
	asm volatile(".word	0x710000d5 | 0 << 16 | 17 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 17 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[18];
	asm volatile(".word	0x710000d5 | 0 << 16 | 18 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 18 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[19];
	asm volatile(".word	0x710000d5 | 0 << 16 | 19 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 19 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[20];
	asm volatile(".word	0x710000d5 | 0 << 16 | 20 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 20 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[21];
	asm volatile(".word	0x710000d5 | 0 << 16 | 21 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 21 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[22];
	asm volatile(".word	0x710000d5 | 0 << 16 | 22 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 22 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[23];
	asm volatile(".word	0x710000d5 | 0 << 16 | 23 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 23 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[24];
	asm volatile(".word	0x710000d5 | 0 << 16 | 24 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 24 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[25];
	asm volatile(".word	0x710000d5 | 0 << 16 | 25 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 25 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[26];
	asm volatile(".word	0x710000d5 | 0 << 16 | 26 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 26 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[27];
	asm volatile(".word	0x710000d5 | 0 << 16 | 27 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 27 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[28];
	asm volatile(".word	0x710000d5 | 0 << 16 | 28 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 28 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[29];
	asm volatile(".word	0x710000d5 | 0 << 16 | 29 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 29 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[30];
	asm volatile(".word	0x710000d5 | 0 << 16 | 30 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 30 << 11 | 1 << 9 \n\t");

	base = &mxu->vpr[31];
	asm volatile(".word	0x710000d5 | 0 << 16 | 31 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 31 << 11 | 1 << 9 \n\t");

	base = &mxu->vsr[0];
	asm volatile(".word 0x4a60000f | 0 << 11 \n\t");
	asm volatile(".word	0x710000d5 | 0 << 16 | 0 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 0 << 11 | 1 << 9 \n\t");

	base = &mxu->vsr[1];
	asm volatile(".word 0x4a60000f | 1 << 11 \n\t");
	asm volatile(".word	0x710000d5 | 0 << 16 | 0 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 0 << 11 | 1 << 9 \n\t");

	base = &mxu->vsr[2];
	asm volatile(".word 0x4a60000f | 2 << 11 \n\t");
	asm volatile(".word	0x710000d5 | 0 << 16 | 0 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 0 << 11 | 1 << 9 \n\t");

	base = &mxu->vsr[3];
	asm volatile(".word 0x4a60000f | 3 << 11 \n\t");
	asm volatile(".word	0x710000d5 | 0 << 16 | 0 << 11 | 0 << 9 \n\t");
	asm volatile(".word	0x710000d5 | 1 << 16 | 0 << 11 | 1 << 9 \n\t");

	base = &mxu->csr;
	asm volatile(".word	0x7010f803 | 9 << 6 \n\t"); // t1 <- mxu.csr
	asm volatile(".word	0xac000000 | 8 << 21 | 9 << 16 \n\t"); // sw $t1 0($t0)
#else
	base = &mxu->vpr[0];
	SA(o, VR0, 0, base, 0);
	SA(o, VR0, 1, base, 32);

	base = &mxu->vpr[1];
	SA(o, VR1, 0, base, 0);
	SA(o, VR1, 1, base, 32);

	base = &mxu->vpr[2];
	SA(o, VR2, 0, base, 0);
	SA(o, VR2, 1, base, 32);

	base = &mxu->vpr[3];
	SA(o, VR3, 0, base, 0);
	SA(o, VR3, 1, base, 32);

	base = &mxu->vpr[4];
	SA(o, VR4, 0, base, 0);
	SA(o, VR4, 1, base, 32);

	base = &mxu->vpr[5];
	SA(o, VR5, 0, base, 0);
	SA(o, VR5, 1, base, 32);

	base = &mxu->vpr[6];
	SA(o, VR6, 0, base, 0);
	SA(o, VR6, 1, base, 32);

	base = &mxu->vpr[7];
	SA(o, VR7, 0, base, 0);
	SA(o, VR7, 1, base, 32);

	base = &mxu->vpr[8];
	SA(o, VR8, 0, base, 0);
	SA(o, VR8, 1, base, 32);

	base = &mxu->vpr[9];
	SA(o, VR9, 0, base, 0);
	SA(o, VR9, 1, base, 32);

	base = &mxu->vpr[10];
	SA(o, VR10, 0, base, 0);
	SA(o, VR10, 1, base, 32);

	base = &mxu->vpr[11];
	SA(o, VR11, 0, base, 0);
	SA(o, VR11, 1, base, 32);

	base = &mxu->vpr[12];
	SA(o, VR12, 0, base, 0);
	SA(o, VR12, 1, base, 32);

	base = &mxu->vpr[13];
	SA(o, VR13, 0, base, 0);
	SA(o, VR13, 1, base, 32);

	base = &mxu->vpr[14];
	SA(o, VR14, 0, base, 0);
	SA(o, VR14, 1, base, 32);

	base = &mxu->vpr[15];
	SA(o, VR15, 0, base, 0);
	SA(o, VR15, 1, base, 32);

	base = &mxu->vpr[16];
	SA(o, VR16, 0, base, 0);
	SA(o, VR16, 1, base, 32);

	base = &mxu->vpr[17];
	SA(o, VR17, 0, base, 0);
	SA(o, VR17, 1, base, 32);

	base = &mxu->vpr[18];
	SA(o, VR18, 0, base, 0);
	SA(o, VR18, 1, base, 32);

	base = &mxu->vpr[19];
	SA(o, VR19, 0, base, 0);
	SA(o, VR19, 1, base, 32);

	base = &mxu->vpr[20];
	SA(o, VR20, 0, base, 0);
	SA(o, VR20, 1, base, 32);

	base = &mxu->vpr[21];
	SA(o, VR21, 0, base, 0);
	SA(o, VR21, 1, base, 32);

	base = &mxu->vpr[22];
	SA(o, VR22, 0, base, 0);
	SA(o, VR22, 1, base, 32);

	base = &mxu->vpr[23];
	SA(o, VR23, 0, base, 0);
	SA(o, VR23, 1, base, 32);

	base = &mxu->vpr[24];
	SA(o, VR24, 0, base, 0);
	SA(o, VR24, 1, base, 32);

	base = &mxu->vpr[25];
	SA(o, VR25, 0, base, 0);
	SA(o, VR25, 1, base, 32);

	base = &mxu->vpr[26];
	SA(o, VR26, 0, base, 0);
	SA(o, VR26, 1, base, 32);

	base = &mxu->vpr[27];
	SA(o, VR27, 0, base, 0);
	SA(o, VR27, 1, base, 32);

	base = &mxu->vpr[28];
	SA(o, VR28, 0, base, 0);
	SA(o, VR28, 1, base, 32);

	base = &mxu->vpr[29];
	SA(o, VR29, 0, base, 0);
	SA(o, VR29, 1, base, 32);

	base = &mxu->vpr[30];
	SA(o, VR30, 0, base, 0);
	SA(o, VR30, 1, base, 32);

	base = &mxu->vpr[31];
	SA(o, VR31, 0, base, 0);
	SA(o, VR31, 1, base, 32);

	base = &mxu->vsr[0];
	MFSUM(VR0, VS0);
	SA(o, VR0, 0, base, 0);
	SA(o, VR0, 1, base, 32);

	base = &mxu->vsr[1];
	MFSUM(VR0, VS1);
	SA(o, VR0, 0, base, 0);
	SA(o, VR0, 1, base, 32);

	base = &mxu->vsr[2];
	MFSUM(VR0, VS2);
	SA(o, VR0, 0, base, 0);
	SA(o, VR0, 1, base, 32);

	base = &mxu->vsr[3];
	MFSUM(VR0, VS3);
	SA(o, VR0, 0, base, 0);
	SA(o, VR0, 1, base, 32);

	base = &mxu->csr;
	CFCMXU(base1, 31);
	asm volatile("sw $t1, 0($t0)\n\t");
#endif
}

__attribute__((optimize("O0")))
void __restore_mxuv3(void *tsk_void)
{
	register void *base asm("t0");
	struct task_struct *tsk = tsk_void;
	struct xburst_mxu_struct *mxu = &tsk->thread.mxu;
	//printk("%s(%d):smp_processor_id()=%d, read_c0_status()=0x%x, KSTK_STATUS(current)=0x%lx, current->tgid=%d, current->pid=%d, , KSTK_STATUS(tsk)=0x%lx, task->tgid=%d, task->pid=%d\n", __func__, __LINE__, smp_processor_id(), read_c0_status(), KSTK_STATUS(current), current->tgid, current->pid, KSTK_STATUS(tsk), tsk->tgid, tsk->pid);
	//dump_stack();
	//printk("%s(%d):smp_processor_id()=%d, read_c0_status()=0x%x, task_cpu(current)=%d, KSTK_STATUS(current)=0x%lx, current->tgid=%d, current->pid=%d, task_cpu(p)=%d, KSTK_STATUS(p)=0x%lx, p->tgid=%d, p->pid=%d\n", __func__, __LINE__, smp_processor_id(), read_c0_status(), task_cpu(current), KSTK_STATUS(current), current->tgid, current->pid, task_cpu(tsk), KSTK_STATUS(tsk), tsk->tgid, tsk->pid);
#ifdef WORDCODE
	/**
	 * lao ins: hex(0x70001811 | 8 << 21 | 1 << 16 | 1 << 14 | 0 << 6)
	 * cmd:     0x70001811   ->la0
	 * base:    8            ->t0
	 * offset:  1            ->32>>5
	 * nn:      1            ->nn    // o means to be divided by 512/256=2, so n=[0,1]
	 * vpr:     0            ->vpr0
	 * because t0 is fixed, so 0x70001811 | 8 << 21 = 0x71001811
	 * so the word format as follows:
	 * asm volatile(".word  0x71001811 | offset << 16 | n << 14 | vprn << 6 \n\t");
	 *
	 * mtsum ins: hex(0x4a60001d | vrs << 16 | vsd << 6)
	 * for we fixed vrs to 0, so the word format as follows:
	 * asm volatile(".word 0x4a60001d | vss << 6 \n\t");
	 */
	base = &mxu->csr;
	asm volatile(".word 0x8c000000 | 8 << 21 | 9 << 16\n\t");// lw $t1, 0($t0)
	asm volatile(".word 0x7011f803 | 9 << 21 \n\t");// mxu.csr <- t1

	base = &mxu->vsr[0];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 0 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 0 << 6 \n\t");
	asm volatile(".word 0x4a60001d | 0 << 6 \n\t");

	base = &mxu->vsr[1];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 0 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 0 << 6 \n\t");
	asm volatile(".word 0x4a60001d | 1 << 6 \n\t");

	base = &mxu->vsr[2];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 0 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 0 << 6 \n\t");
	asm volatile(".word 0x4a60001d | 2 << 6 \n\t");

	base = &mxu->vsr[3];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 0 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 0 << 6 \n\t");
	asm volatile(".word 0x4a60001d | 3 << 6 \n\t");

	base = &mxu->vpr[0];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 0 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 0 << 6 \n\t");

	base = &mxu->vpr[1];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 1 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 1 << 6 \n\t");

	base = &mxu->vpr[2];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 2 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 2 << 6 \n\t");

	base = &mxu->vpr[3];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 3 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 3 << 6 \n\t");

	base = &mxu->vpr[4];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 4 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 4 << 6 \n\t");

	base = &mxu->vpr[5];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 5 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 5 << 6 \n\t");

	base = &mxu->vpr[6];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 6 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 6 << 6 \n\t");

	base = &mxu->vpr[7];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 7 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 7 << 6 \n\t");

	base = &mxu->vpr[8];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 8 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 8 << 6 \n\t");

	base = &mxu->vpr[9];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 9 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 9 << 6 \n\t");

	base = &mxu->vpr[10];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 10 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 10 << 6 \n\t");

	base = &mxu->vpr[11];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 11 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 11 << 6 \n\t");

	base = &mxu->vpr[12];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 12 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 12 << 6 \n\t");

	base = &mxu->vpr[13];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 13 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 13 << 6 \n\t");

	base = &mxu->vpr[14];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 14 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 14 << 6 \n\t");

	base = &mxu->vpr[15];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 15 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 15 << 6 \n\t");

	base = &mxu->vpr[16];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 16 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 16 << 6 \n\t");

	base = &mxu->vpr[17];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 17 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 17 << 6 \n\t");

	base = &mxu->vpr[18];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 18 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 18 << 6 \n\t");

	base = &mxu->vpr[19];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 19 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 19 << 6 \n\t");

	base = &mxu->vpr[20];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 20 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 20 << 6 \n\t");

	base = &mxu->vpr[21];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 21 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 21 << 6 \n\t");

	base = &mxu->vpr[22];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 22 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 22 << 6 \n\t");

	base = &mxu->vpr[23];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 23 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 23 << 6 \n\t");

	base = &mxu->vpr[24];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 24 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 24 << 6 \n\t");

	base = &mxu->vpr[25];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 25 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 25 << 6 \n\t");

	base = &mxu->vpr[26];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 26 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 26 << 6 \n\t");

	base = &mxu->vpr[27];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 27 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 27 << 6 \n\t");

	base = &mxu->vpr[28];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 28 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 28 << 6 \n\t");

	base = &mxu->vpr[29];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 29 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 29 << 6 \n\t");

	base = &mxu->vpr[30];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 30 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 30 << 6 \n\t");

	base = &mxu->vpr[31];
	asm volatile(".word	0x71001811 | 0 << 16 | 0 << 14 | 31 << 6 \n\t");
	asm volatile(".word	0x71001811 | 1 << 16 | 1 << 14 | 31 << 6 \n\t");
#else
	base = &mxu->csr;
	asm volatile("lw $t1, 0($t0)\n\t");
	CTCMXU(31, base1);

	base = &mxu->vsr[0];
	LA(o, VR0, 0, base, 0);
	LA(o, VR0, 1, base, 32);
	MTSUM(VS0, VR0);

	base = &mxu->vsr[1];
	LA(o, VR0, 0, base, 0);
	LA(o, VR0, 1, base, 32);
	MTSUM(VS1, VR0);

	base = &mxu->vsr[2];
	LA(o, VR0, 0, base, 0);
	LA(o, VR0, 1, base, 32);
	MTSUM(VS2, VR0);

	base = &mxu->vsr[3];
	LA(o, VR0, 0, base, 0);
	LA(o, VR0, 1, base, 32);
	MTSUM(VS3, VR0);

	base = &mxu->vpr[0];
	LA(o, VR0, 0, base, 0);
	LA(o, VR0, 1, base, 32);

	base = &mxu->vpr[1];
	LA(o, VR1, 0, base, 0);
	LA(o, VR1, 1, base, 32);

	base = &mxu->vpr[2];
	LA(o, VR2, 0, base, 0);
	LA(o, VR2, 1, base, 32);

	base = &mxu->vpr[3];
	LA(o, VR3, 0, base, 0);
	LA(o, VR3, 1, base, 32);

	base = &mxu->vpr[4];
	LA(o, VR4, 0, base, 0);
	LA(o, VR4, 1, base, 32);

	base = &mxu->vpr[5];
	LA(o, VR5, 0, base, 0);
	LA(o, VR5, 1, base, 32);

	base = &mxu->vpr[6];
	LA(o, VR6, 0, base, 0);
	LA(o, VR6, 1, base, 32);

	base = &mxu->vpr[7];
	LA(o, VR7, 0, base, 0);
	LA(o, VR7, 1, base, 32);

	base = &mxu->vpr[8];
	LA(o, VR8, 0, base, 0);
	LA(o, VR8, 1, base, 32);

	base = &mxu->vpr[9];
	LA(o, VR9, 0, base, 0);
	LA(o, VR9, 1, base, 32);

	base = &mxu->vpr[10];
	LA(o, VR10, 0, base, 0);
	LA(o, VR10, 1, base, 32);

	base = &mxu->vpr[11];
	LA(o, VR11, 0, base, 0);
	LA(o, VR11, 1, base, 32);

	base = &mxu->vpr[12];
	LA(o, VR12, 0, base, 0);
	LA(o, VR12, 1, base, 32);

	base = &mxu->vpr[13];
	LA(o, VR13, 0, base, 0);
	LA(o, VR13, 1, base, 32);

	base = &mxu->vpr[14];
	LA(o, VR14, 0, base, 0);
	LA(o, VR14, 1, base, 32);

	base = &mxu->vpr[15];
	LA(o, VR15, 0, base, 0);
	LA(o, VR15, 1, base, 32);

	base = &mxu->vpr[16];
	LA(o, VR16, 0, base, 0);
	LA(o, VR16, 1, base, 32);

	base = &mxu->vpr[17];
	LA(o, VR17, 0, base, 0);
	LA(o, VR17, 1, base, 32);

	base = &mxu->vpr[18];
	LA(o, VR18, 0, base, 0);
	LA(o, VR18, 1, base, 32);

	base = &mxu->vpr[19];
	LA(o, VR19, 0, base, 0);
	LA(o, VR19, 1, base, 32);

	base = &mxu->vpr[20];
	LA(o, VR20, 0, base, 0);
	LA(o, VR20, 1, base, 32);

	base = &mxu->vpr[21];
	LA(o, VR21, 0, base, 0);
	LA(o, VR21, 1, base, 32);

	base = &mxu->vpr[22];
	LA(o, VR22, 0, base, 0);
	LA(o, VR22, 1, base, 32);

	base = &mxu->vpr[23];
	LA(o, VR23, 0, base, 0);
	LA(o, VR23, 1, base, 32);

	base = &mxu->vpr[24];
	LA(o, VR24, 0, base, 0);
	LA(o, VR24, 1, base, 32);

	base = &mxu->vpr[25];
	LA(o, VR25, 0, base, 0);
	LA(o, VR25, 1, base, 32);

	base = &mxu->vpr[26];
	LA(o, VR26, 0, base, 0);
	LA(o, VR26, 1, base, 32);

	base = &mxu->vpr[27];
	LA(o, VR27, 0, base, 0);
	LA(o, VR27, 1, base, 32);

	base = &mxu->vpr[28];
	LA(o, VR28, 0, base, 0);
	LA(o, VR28, 1, base, 32);

	base = &mxu->vpr[29];
	LA(o, VR29, 0, base, 0);
	LA(o, VR29, 1, base, 32);

	base = &mxu->vpr[30];
	LA(o, VR30, 0, base, 0);
	LA(o, VR30, 1, base, 32);

	base = &mxu->vpr[31];
	LA(o, VR31, 0, base, 0);
	LA(o, VR31, 1, base, 32);
#endif
}
