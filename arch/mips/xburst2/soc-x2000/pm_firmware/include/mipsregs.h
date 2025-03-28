/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994, 1995, 1996, 1997, 2000, 2001 by Ralf Baechle
 * Copyright (C) 2000 Silicon Graphics, Inc.
 * Modified for further R[236]000 support by Paul M. Antoine, 1996.
 * Kevin D. Kissell, kevink@mips.com and Carsten Langgaard, carstenl@mips.com
 * Copyright (C) 2000, 07 MIPS Technologies, Inc.
 * Copyright (C) 2003, 2004  Maciej W. Rozycki
 */
#ifndef _ASM_MIPSREGS_H
#define _ASM_MIPSREGS_H

/*
 * Coprocessor 0 register names
 */
#define CP0_INDEX $0
#define CP0_RANDOM $1
#define CP0_ENTRYLO0 $2
#define CP0_ENTRYLO1 $3
#define CP0_CONF $3
#define CP0_CONTEXT $4
#define CP0_PAGEMASK $5
#define CP0_WIRED $6
#define CP0_INFO $7
#define CP0_BADVADDR $8
#define CP0_COUNT $9
#define CP0_ENTRYHI $10
#define CP0_COMPARE $11
#define CP0_STATUS $12
#define CP0_CAUSE $13
#define CP0_EPC $14
#define CP0_PRID $15
#define CP0_CONFIG $16
#define CP0_LLADDR $17
#define CP0_WATCHLO $18
#define CP0_WATCHHI $19
#define CP0_XCONTEXT $20
#define CP0_FRAMEMASK $21
#define CP0_DIAGNOSTIC $22
#define CP0_DEBUG $23
#define CP0_DEPC $24
#define CP0_PERFORMANCE $25
#define CP0_ECC $26
#define CP0_CACHEERR $27
#define CP0_TAGLO $28
#define CP0_TAGHI $29
#define CP0_ERROREPC $30
#define CP0_DESAVE $31

/*
 * R4640/R4650 cp0 register names.  These registers are listed
 * here only for completeness; without MMU these CPUs are not useable
 * by Linux.  A future ELKS port might take make Linux run on them
 * though ...
 */
#define CP0_IBASE $0
#define CP0_IBOUND $1
#define CP0_DBASE $2
#define CP0_DBOUND $3
#define CP0_CALG $17
#define CP0_IWATCH $18
#define CP0_DWATCH $19

/*
 * Coprocessor 0 Set 1 register names
 */
#define CP0_S1_DERRADDR0  $26
#define CP0_S1_DERRADDR1  $27
#define CP0_S1_INTCONTROL $20

/*
 * Coprocessor 0 Set 2 register names
 */
#define CP0_S2_SRSCTL     $12   /* MIPSR2 */

/*
 * Coprocessor 0 Set 3 register names
 */
#define CP0_S3_SRSMAP     $12   /* MIPSR2 */

/*
 *  TX39 Series
 */
#define CP0_TX39_CACHE  $7

/*
 * Coprocessor 1 (FPU) register names
 */
#define CP1_REVISION   $0
#define CP1_STATUS     $31

/*
 * FPU Status Register Values
 */
/*
 * Status Register Values
 */

#define FPU_CSR_FLUSH   0x01000000  /* flush denormalised results to 0 */
#define FPU_CSR_COND    0x00800000  /* $fcc0 */
#define FPU_CSR_COND0   0x00800000  /* $fcc0 */
#define FPU_CSR_COND1   0x02000000  /* $fcc1 */
#define FPU_CSR_COND2   0x04000000  /* $fcc2 */
#define FPU_CSR_COND3   0x08000000  /* $fcc3 */
#define FPU_CSR_COND4   0x10000000  /* $fcc4 */
#define FPU_CSR_COND5   0x20000000  /* $fcc5 */
#define FPU_CSR_COND6   0x40000000  /* $fcc6 */
#define FPU_CSR_COND7   0x80000000  /* $fcc7 */

/*
 * Bits 18 - 20 of the FPU Status Register will be read as 0,
 * and should be written as zero.
 */
#define FPU_CSR_RSVD    0x001c0000
/* ... but FPU2 uses that bits */
#define FPU_CSR_NAN2008 0x00040000
#define FPU_CSR_ABS2008 0x00080000
#define FPU_CSR_MAC2008 0x00100000

#define FPU_CSR_DEFAULT 0x00000000

/*
 * X the exception cause indicator
 * E the exception enable
 * S the sticky/flag bit
*/
#define FPU_CSR_ALL_X   0x0003f000
#define FPU_CSR_UNI_X   0x00020000
#define FPU_CSR_INV_X   0x00010000
#define FPU_CSR_DIV_X   0x00008000
#define FPU_CSR_OVF_X   0x00004000
#define FPU_CSR_UDF_X   0x00002000
#define FPU_CSR_INE_X   0x00001000

#define FPU_CSR_ALL_E   0x00000f80
#define FPU_CSR_INV_E   0x00000800
#define FPU_CSR_DIV_E   0x00000400
#define FPU_CSR_OVF_E   0x00000200
#define FPU_CSR_UDF_E   0x00000100
#define FPU_CSR_INE_E   0x00000080

#define FPU_CSR_ALL_S   0x0000007c
#define FPU_CSR_INV_S   0x00000040
#define FPU_CSR_DIV_S   0x00000020
#define FPU_CSR_OVF_S   0x00000010
#define FPU_CSR_UDF_S   0x00000008
#define FPU_CSR_INE_S   0x00000004

/* Bits 0 and 1 of FPU Status Register specify the rounding mode */
#define FPU_CSR_RM  0x00000003
#define FPU_CSR_RN  0x0 /* nearest */
#define FPU_CSR_RZ  0x1 /* towards zero */
#define FPU_CSR_RU  0x2 /* towards +Infinity */
#define FPU_CSR_RD  0x3 /* towards -Infinity */


/*
 * Macros to access the system control coprocessor
 */

#define __read_32bit_c0_register(source, sel)               \
	({ int __res;                               \
		if (sel == 0)                           \
			__asm__ __volatile__(                   \
			                                        "mfc0\t%0, " #source "\n\t"         \
			                                        : "=r" (__res));                \
		else                                \
			__asm__ __volatile__(                   \
			                                        ".set\tmips32\n\t"              \
			                                        "mfc0\t%0, " #source ", " #sel "\n\t"       \
			                                        ".set\tmips0\n\t"               \
			                                        : "=r" (__res));                \
		__res;                              \
	})

#define __read_64bit_c0_register(source, sel)               \
	({ unsigned long long __res;                        \
		if (sizeof(unsigned long) == 4)                 \
			__res = __read_64bit_c0_split(source, sel);     \
		else if (sel == 0)                      \
			__asm__ __volatile__(                   \
			                                        ".set\tmips3\n\t"               \
			                                        "dmfc0\t%0, " #source "\n\t"            \
			                                        ".set\tmips0"                   \
			                                        : "=r" (__res));                \
		else                                \
			__asm__ __volatile__(                   \
			                                        ".set\tmips64\n\t"              \
			                                        "dmfc0\t%0, " #source ", " #sel "\n\t"      \
			                                        ".set\tmips0"                   \
			                                        : "=r" (__res));                \
		__res;                              \
	})

#define __write_32bit_c0_register(register, sel, value)         \
	do {                                    \
		if (sel == 0)                           \
			__asm__ __volatile__(                   \
			                                        "mtc0\t%z0, " #register "\n\t"          \
			                                        : : "Jr" ((unsigned int)(value)));      \
		else                                \
			__asm__ __volatile__(                   \
			                                        ".set\tmips32\n\t"              \
			                                        "mtc0\t%z0, " #register ", " #sel "\n\t"    \
			                                        ".set\tmips0"                   \
			                                        : : "Jr" ((unsigned int)(value)));      \
	} while (0)

#define __write_64bit_c0_register(register, sel, value)         \
	do {                                    \
		if (sizeof(unsigned long) == 4)                 \
			__write_64bit_c0_split(register, sel, value);       \
		else if (sel == 0)                      \
			__asm__ __volatile__(                   \
			                                        ".set\tmips3\n\t"               \
			                                        "dmtc0\t%z0, " #register "\n\t"         \
			                                        ".set\tmips0"                   \
			                                        : : "Jr" (value));              \
		else                                \
			__asm__ __volatile__(                   \
			                                        ".set\tmips64\n\t"              \
			                                        "dmtc0\t%z0, " #register ", " #sel "\n\t"   \
			                                        ".set\tmips0"                   \
			                                        : : "Jr" (value));              \
	} while (0)

#define __read_ulong_c0_register(reg, sel)              \
	((sizeof(unsigned long) == 4) ?                 \
	 (unsigned long) __read_32bit_c0_register(reg, sel) :        \
	 (unsigned long) __read_64bit_c0_register(reg, sel))

#define __write_ulong_c0_register(reg, sel, val)            \
	do {                                    \
		if (sizeof(unsigned long) == 4)                 \
			__write_32bit_c0_register(reg, sel, val);       \
		else                                \
			__write_64bit_c0_register(reg, sel, val);       \
	} while (0)

/*
 * On RM7000/RM9000 these are uses to access cop0 set 1 registers
 */
#define __read_32bit_c0_ctrl_register(source)               \
	({ int __res;                               \
		__asm__ __volatile__(                       \
		        "cfc0\t%0, " #source "\n\t"             \
		        : "=r" (__res));                    \
		__res;                              \
	})

#define __write_32bit_c0_ctrl_register(register, value)         \
	do {                                    \
		__asm__ __volatile__(                       \
		        "ctc0\t%z0, " #register "\n\t"              \
		        : : "Jr" ((unsigned int)(value)));          \
	} while (0)

/*
 * These versions are only needed for systems with more than 38 bits of
 * physical address space running the 32-bit kernel.  That's none atm :-)
 */
#define __read_64bit_c0_split(source, sel)              \
	({                                  \
		unsigned long long __val;                   \
		unsigned long __flags;                      \
		\
		local_irq_save(__flags);                    \
		if (sel == 0)                           \
			__asm__ __volatile__(                   \
			                                        ".set\tmips64\n\t"              \
			                                        "dmfc0\t%M0, " #source "\n\t"           \
			                                        "dsll\t%L0, %M0, 32\n\t"            \
			                                        "dsra\t%M0, %M0, 32\n\t"            \
			                                        "dsra\t%L0, %L0, 32\n\t"            \
			                                        ".set\tmips0"                   \
			                                        : "=r" (__val));                \
		else                                \
			__asm__ __volatile__(                   \
			                                        ".set\tmips64\n\t"              \
			                                        "dmfc0\t%M0, " #source ", " #sel "\n\t"     \
			                                        "dsll\t%L0, %M0, 32\n\t"            \
			                                        "dsra\t%M0, %M0, 32\n\t"            \
			                                        "dsra\t%L0, %L0, 32\n\t"            \
			                                        ".set\tmips0"                   \
			                                        : "=r" (__val));                \
		local_irq_restore(__flags);                 \
		\
		__val;                              \
	})

#define __write_64bit_c0_split(source, sel, val)            \
	do {                                    \
		unsigned long __flags;                      \
		\
		local_irq_save(__flags);                    \
		if (sel == 0)                           \
			__asm__ __volatile__(                   \
			                                        ".set\tmips64\n\t"              \
			                                        "dsll\t%L0, %L0, 32\n\t"            \
			                                        "dsrl\t%L0, %L0, 32\n\t"            \
			                                        "dsll\t%M0, %M0, 32\n\t"            \
			                                        "or\t%L0, %L0, %M0\n\t"             \
			                                        "dmtc0\t%L0, " #source "\n\t"           \
			                                        ".set\tmips0"                   \
			                                        : : "r" (val));                 \
		else                                \
			__asm__ __volatile__(                   \
			                                        ".set\tmips64\n\t"              \
			                                        "dsll\t%L0, %L0, 32\n\t"            \
			                                        "dsrl\t%L0, %L0, 32\n\t"            \
			                                        "dsll\t%M0, %M0, 32\n\t"            \
			                                        "or\t%L0, %L0, %M0\n\t"             \
			                                        "dmtc0\t%L0, " #source ", " #sel "\n\t"     \
			                                        ".set\tmips0"                   \
			                                        : : "r" (val));                 \
		local_irq_restore(__flags);                 \
	} while (0)

#define read_c0_index()     __read_32bit_c0_register($0, 0)
#define write_c0_index(val) __write_32bit_c0_register($0, 0, val)

#define read_c0_random()    __read_32bit_c0_register($1, 0)
#define write_c0_random(val)    __write_32bit_c0_register($1, 0, val)

#define read_c0_entrylo0()  __read_ulong_c0_register($2, 0)
#define write_c0_entrylo0(val)  __write_ulong_c0_register($2, 0, val)

#define read_c0_entrylo1()  __read_ulong_c0_register($3, 0)
#define write_c0_entrylo1(val)  __write_ulong_c0_register($3, 0, val)

#define read_c0_conf()      __read_32bit_c0_register($3, 0)
#define write_c0_conf(val)  __write_32bit_c0_register($3, 0, val)

#define read_c0_context()   __read_ulong_c0_register($4, 0)
#define write_c0_context(val)   __write_ulong_c0_register($4, 0, val)

#define read_c0_userlocal() __read_ulong_c0_register($4, 2)
#define write_c0_userlocal(val) __write_ulong_c0_register($4, 2, val)

#define read_c0_pagemask()  __read_32bit_c0_register($5, 0)
#define write_c0_pagemask(val)  __write_32bit_c0_register($5, 0, val)

#define read_c0_pagegrain() __read_32bit_c0_register($5, 1)
#define write_c0_pagegrain(val) __write_32bit_c0_register($5, 1, val)

#define read_c0_wired()     __read_32bit_c0_register($6, 0)
#define write_c0_wired(val) __write_32bit_c0_register($6, 0, val)

#define read_c0_info()      __read_32bit_c0_register($7, 0)

#define read_c0_cache()     __read_32bit_c0_register($7, 0) /* TX39xx */
#define write_c0_cache(val) __write_32bit_c0_register($7, 0, val)

#define read_c0_badvaddr()  __read_ulong_c0_register($8, 0)
#define write_c0_badvaddr(val)  __write_ulong_c0_register($8, 0, val)

#define read_c0_count()     __read_32bit_c0_register($9, 0)
#define write_c0_count(val) __write_32bit_c0_register($9, 0, val)

#define read_c0_count2()    __read_32bit_c0_register($9, 6) /* pnx8550 */
#define write_c0_count2(val)    __write_32bit_c0_register($9, 6, val)

#define read_c0_count3()    __read_32bit_c0_register($9, 7) /* pnx8550 */
#define write_c0_count3(val)    __write_32bit_c0_register($9, 7, val)

#define read_c0_entryhi()   __read_ulong_c0_register($10, 0)
#define write_c0_entryhi(val)   __write_ulong_c0_register($10, 0, val)

#define read_c0_compare()   __read_32bit_c0_register($11, 0)
#define write_c0_compare(val)   __write_32bit_c0_register($11, 0, val)

#define read_c0_compare2()  __read_32bit_c0_register($11, 6) /* pnx8550 */
#define write_c0_compare2(val)  __write_32bit_c0_register($11, 6, val)

#define read_c0_compare3()  __read_32bit_c0_register($11, 7) /* pnx8550 */
#define write_c0_compare3(val)  __write_32bit_c0_register($11, 7, val)

#define read_c0_status()    __read_32bit_c0_register($12, 0)

#define write_c0_status(val)    __write_32bit_c0_register($12, 0, val)

#define read_c0_cause()     __read_32bit_c0_register($13, 0)
#define write_c0_cause(val) __write_32bit_c0_register($13, 0, val)

#define read_c0_epc()       __read_ulong_c0_register($14, 0)
#define write_c0_epc(val)   __write_ulong_c0_register($14, 0, val)

#define read_c0_prid()      __read_32bit_c0_register($15, 0)
#define read_c0_cmgcrbase()     __read_ulong_c0_register($15, 3)

#define read_c0_config()    __read_32bit_c0_register($16, 0)
#define read_c0_config1()   __read_32bit_c0_register($16, 1)
#define read_c0_config2()   __read_32bit_c0_register($16, 2)
#define read_c0_config3()   __read_32bit_c0_register($16, 3)
#define read_c0_config4()   __read_32bit_c0_register($16, 4)
#define read_c0_config5()   __read_32bit_c0_register($16, 5)
#define read_c0_config6()   __read_32bit_c0_register($16, 6)
#define read_c0_config7()   __read_32bit_c0_register($16, 7)
#define write_c0_config(val)    __write_32bit_c0_register($16, 0, val)
#define write_c0_config1(val)   __write_32bit_c0_register($16, 1, val)
#define write_c0_config2(val)   __write_32bit_c0_register($16, 2, val)
#define write_c0_config3(val)   __write_32bit_c0_register($16, 3, val)
#define write_c0_config4(val)   __write_32bit_c0_register($16, 4, val)
#define write_c0_config5(val)   __write_32bit_c0_register($16, 5, val)
#define write_c0_config6(val)   __write_32bit_c0_register($16, 6, val)
#define write_c0_config7(val)   __write_32bit_c0_register($16, 7, val)

/*
 * The WatchLo register.  There may be up to 8 of them.
 */
#define read_c0_watchlo0()  __read_ulong_c0_register($18, 0)
#define read_c0_watchlo1()  __read_ulong_c0_register($18, 1)
#define read_c0_watchlo2()  __read_ulong_c0_register($18, 2)
#define read_c0_watchlo3()  __read_ulong_c0_register($18, 3)
#define read_c0_watchlo4()  __read_ulong_c0_register($18, 4)
#define read_c0_watchlo5()  __read_ulong_c0_register($18, 5)
#define read_c0_watchlo6()  __read_ulong_c0_register($18, 6)
#define read_c0_watchlo7()  __read_ulong_c0_register($18, 7)
#define write_c0_watchlo0(val)  __write_ulong_c0_register($18, 0, val)
#define write_c0_watchlo1(val)  __write_ulong_c0_register($18, 1, val)
#define write_c0_watchlo2(val)  __write_ulong_c0_register($18, 2, val)
#define write_c0_watchlo3(val)  __write_ulong_c0_register($18, 3, val)
#define write_c0_watchlo4(val)  __write_ulong_c0_register($18, 4, val)
#define write_c0_watchlo5(val)  __write_ulong_c0_register($18, 5, val)
#define write_c0_watchlo6(val)  __write_ulong_c0_register($18, 6, val)
#define write_c0_watchlo7(val)  __write_ulong_c0_register($18, 7, val)

/*
 * The WatchHi register.  There may be up to 8 of them.
 */
#define read_c0_watchhi0()  __read_32bit_c0_register($19, 0)
#define read_c0_watchhi1()  __read_32bit_c0_register($19, 1)
#define read_c0_watchhi2()  __read_32bit_c0_register($19, 2)
#define read_c0_watchhi3()  __read_32bit_c0_register($19, 3)
#define read_c0_watchhi4()  __read_32bit_c0_register($19, 4)
#define read_c0_watchhi5()  __read_32bit_c0_register($19, 5)
#define read_c0_watchhi6()  __read_32bit_c0_register($19, 6)
#define read_c0_watchhi7()  __read_32bit_c0_register($19, 7)

#define write_c0_watchhi0(val)  __write_32bit_c0_register($19, 0, val)
#define write_c0_watchhi1(val)  __write_32bit_c0_register($19, 1, val)
#define write_c0_watchhi2(val)  __write_32bit_c0_register($19, 2, val)
#define write_c0_watchhi3(val)  __write_32bit_c0_register($19, 3, val)
#define write_c0_watchhi4(val)  __write_32bit_c0_register($19, 4, val)
#define write_c0_watchhi5(val)  __write_32bit_c0_register($19, 5, val)
#define write_c0_watchhi6(val)  __write_32bit_c0_register($19, 6, val)
#define write_c0_watchhi7(val)  __write_32bit_c0_register($19, 7, val)

#define read_c0_xcontext()  __read_ulong_c0_register($20, 0)
#define write_c0_xcontext(val)  __write_ulong_c0_register($20, 0, val)

#define read_c0_intcontrol()    __read_32bit_c0_ctrl_register($20)
#define write_c0_intcontrol(val) __write_32bit_c0_ctrl_register($20, val)

#define read_c0_framemask() __read_32bit_c0_register($21, 0)
#define write_c0_framemask(val) __write_32bit_c0_register($21, 0, val)

#define read_c0_diag()      __read_32bit_c0_register($22, 0)
#define write_c0_diag(val)  __write_32bit_c0_register($22, 0, val)

#define read_c0_diag1()     __read_32bit_c0_register($22, 1)
#define write_c0_diag1(val) __write_32bit_c0_register($22, 1, val)

#define read_c0_diag2()     __read_32bit_c0_register($22, 2)
#define write_c0_diag2(val) __write_32bit_c0_register($22, 2, val)

#define read_c0_diag3()     __read_32bit_c0_register($22, 3)
#define write_c0_diag3(val) __write_32bit_c0_register($22, 3, val)

#define read_c0_diag4()     __read_32bit_c0_register($22, 4)
#define write_c0_diag4(val) __write_32bit_c0_register($22, 4, val)

#define read_c0_diag5()     __read_32bit_c0_register($22, 5)
#define write_c0_diag5(val) __write_32bit_c0_register($22, 5, val)

#define read_c0_debug()     __read_32bit_c0_register($23, 0)
#define write_c0_debug(val) __write_32bit_c0_register($23, 0, val)

#define read_c0_depc()      __read_ulong_c0_register($24, 0)
#define write_c0_depc(val)  __write_ulong_c0_register($24, 0, val)

/*
 * MIPS32 / MIPS64 performance counters
 */
#define read_c0_perfctrl0() __read_32bit_c0_register($25, 0)
#define write_c0_perfctrl0(val) __write_32bit_c0_register($25, 0, val)
#define read_c0_perfcntr0() __read_32bit_c0_register($25, 1)
#define write_c0_perfcntr0(val) __write_32bit_c0_register($25, 1, val)
#define read_c0_perfcntr0_64()  __read_64bit_c0_register($25, 1)
#define write_c0_perfcntr0_64(val) __write_64bit_c0_register($25, 1, val)
#define read_c0_perfctrl1() __read_32bit_c0_register($25, 2)
#define write_c0_perfctrl1(val) __write_32bit_c0_register($25, 2, val)
#define read_c0_perfcntr1() __read_32bit_c0_register($25, 3)
#define write_c0_perfcntr1(val) __write_32bit_c0_register($25, 3, val)
#define read_c0_perfcntr1_64()  __read_64bit_c0_register($25, 3)
#define write_c0_perfcntr1_64(val) __write_64bit_c0_register($25, 3, val)
#define read_c0_perfctrl2() __read_32bit_c0_register($25, 4)
#define write_c0_perfctrl2(val) __write_32bit_c0_register($25, 4, val)
#define read_c0_perfcntr2() __read_32bit_c0_register($25, 5)
#define write_c0_perfcntr2(val) __write_32bit_c0_register($25, 5, val)
#define read_c0_perfcntr2_64()  __read_64bit_c0_register($25, 5)
#define write_c0_perfcntr2_64(val) __write_64bit_c0_register($25, 5, val)
#define read_c0_perfctrl3() __read_32bit_c0_register($25, 6)
#define write_c0_perfctrl3(val) __write_32bit_c0_register($25, 6, val)
#define read_c0_perfcntr3() __read_32bit_c0_register($25, 7)
#define write_c0_perfcntr3(val) __write_32bit_c0_register($25, 7, val)
#define read_c0_perfcntr3_64()  __read_64bit_c0_register($25, 7)
#define write_c0_perfcntr3_64(val) __write_64bit_c0_register($25, 7, val)

#define read_c0_ecc()       __read_32bit_c0_register($26, 0)
#define write_c0_ecc(val)   __write_32bit_c0_register($26, 0, val)

#define read_c0_derraddr0() __read_ulong_c0_register($26, 1)
#define write_c0_derraddr0(val) __write_ulong_c0_register($26, 1, val)

#define read_c0_cacheerr()  __read_32bit_c0_register($27, 0)

#define read_c0_derraddr1() __read_ulong_c0_register($27, 1)
#define write_c0_derraddr1(val) __write_ulong_c0_register($27, 1, val)

#define read_c0_taglo()     __read_32bit_c0_register($28, 0)
#define write_c0_taglo(val) __write_32bit_c0_register($28, 0, val)

#define read_c0_dtaglo()    __read_32bit_c0_register($28, 2)
#define write_c0_dtaglo(val)    __write_32bit_c0_register($28, 2, val)

#define read_c0_ddatalo()   __read_32bit_c0_register($28, 3)
#define write_c0_ddatalo(val)   __write_32bit_c0_register($28, 3, val)

#define read_c0_staglo()    __read_32bit_c0_register($28, 4)
#define write_c0_staglo(val)    __write_32bit_c0_register($28, 4, val)

#define read_c0_taghi()     __read_32bit_c0_register($29, 0)
#define write_c0_taghi(val) __write_32bit_c0_register($29, 0, val)

#define read_c0_errorepc()  __read_ulong_c0_register($30, 0)
#define write_c0_errorepc(val)  __write_ulong_c0_register($30, 0, val)

/* MIPSR2 */
#define read_c0_hwrena()    __read_32bit_c0_register($7, 0)
#define write_c0_hwrena(val)    __write_32bit_c0_register($7, 0, val)

#define read_c0_intctl()    __read_32bit_c0_register($12, 1)
#define write_c0_intctl(val)    __write_32bit_c0_register($12, 1, val)

#define read_c0_srsctl()    __read_32bit_c0_register($12, 2)
#define write_c0_srsctl(val)    __write_32bit_c0_register($12, 2, val)

#define read_c0_srsmap()    __read_32bit_c0_register($12, 3)
#define write_c0_srsmap(val)    __write_32bit_c0_register($12, 3, val)

#define read_c0_ebase()     __read_32bit_c0_register($15, 1)
#define write_c0_ebase(val) __write_32bit_c0_register($15, 1, val)

/* MIPSR3 */
#define read_c0_segctl0()   __read_32bit_c0_register($5, 2)
#define write_c0_segctl0(val)   __write_32bit_c0_register($5, 2, val)

#define read_c0_segctl1()   __read_32bit_c0_register($5, 3)
#define write_c0_segctl1(val)   __write_32bit_c0_register($5, 3, val)

#define read_c0_segctl2()   __read_32bit_c0_register($5, 4)
#define write_c0_segctl2(val)   __write_32bit_c0_register($5, 4, val)

/* Cavium OCTEON (cnMIPS) */
#define read_c0_cvmcount()  __read_ulong_c0_register($9, 6)
#define write_c0_cvmcount(val)  __write_ulong_c0_register($9, 6, val)

#define read_c0_cvmctl()    __read_64bit_c0_register($9, 7)
#define write_c0_cvmctl(val)    __write_64bit_c0_register($9, 7, val)

#define read_c0_cvmmemctl() __read_64bit_c0_register($11, 7)
#define write_c0_cvmmemctl(val) __write_64bit_c0_register($11, 7, val)
/*
 * The cacheerr registers are not standardized.  On OCTEON, they are
 * 64 bits wide.
 */
#define read_octeon_c0_icacheerr()  __read_64bit_c0_register($27, 0)
#define write_octeon_c0_icacheerr(val)  __write_64bit_c0_register($27, 0, val)

#define read_octeon_c0_dcacheerr()  __read_64bit_c0_register($27, 1)
#define write_octeon_c0_dcacheerr(val)  __write_64bit_c0_register($27, 1, val)

/* BMIPS3300 */
#define read_c0_brcm_config_0()     __read_32bit_c0_register($22, 0)
#define write_c0_brcm_config_0(val) __write_32bit_c0_register($22, 0, val)

#define read_c0_brcm_bus_pll()      __read_32bit_c0_register($22, 4)
#define write_c0_brcm_bus_pll(val)  __write_32bit_c0_register($22, 4, val)

#define read_c0_brcm_reset()        __read_32bit_c0_register($22, 5)
#define write_c0_brcm_reset(val)    __write_32bit_c0_register($22, 5, val)

/* BMIPS43xx */
#define read_c0_brcm_cmt_intr()     __read_32bit_c0_register($22, 1)
#define write_c0_brcm_cmt_intr(val) __write_32bit_c0_register($22, 1, val)

#define read_c0_brcm_cmt_ctrl()     __read_32bit_c0_register($22, 2)
#define write_c0_brcm_cmt_ctrl(val) __write_32bit_c0_register($22, 2, val)

#define read_c0_brcm_cmt_local()    __read_32bit_c0_register($22, 3)
#define write_c0_brcm_cmt_local(val)    __write_32bit_c0_register($22, 3, val)

#define read_c0_brcm_config_1()     __read_32bit_c0_register($22, 5)
#define write_c0_brcm_config_1(val) __write_32bit_c0_register($22, 5, val)

#define read_c0_brcm_cbr()      __read_32bit_c0_register($22, 6)
#define write_c0_brcm_cbr(val)      __write_32bit_c0_register($22, 6, val)

/* BMIPS5000 */
#define read_c0_brcm_config()       __read_32bit_c0_register($22, 0)
#define write_c0_brcm_config(val)   __write_32bit_c0_register($22, 0, val)

#define read_c0_brcm_mode()     __read_32bit_c0_register($22, 1)
#define write_c0_brcm_mode(val)     __write_32bit_c0_register($22, 1, val)

#define read_c0_brcm_action()       __read_32bit_c0_register($22, 2)
#define write_c0_brcm_action(val)   __write_32bit_c0_register($22, 2, val)

#define read_c0_brcm_edsp()     __read_32bit_c0_register($22, 3)
#define write_c0_brcm_edsp(val)     __write_32bit_c0_register($22, 3, val)

#define read_c0_brcm_bootvec()      __read_32bit_c0_register($22, 4)
#define write_c0_brcm_bootvec(val)  __write_32bit_c0_register($22, 4, val)

#define read_c0_brcm_sleepcount()   __read_32bit_c0_register($22, 7)
#define write_c0_brcm_sleepcount(val)   __write_32bit_c0_register($22, 7, val)

/*
 * Macros to access the floating point coprocessor control registers
 */
#define read_32bit_cp1_register(source)                 \
	({                                  \
		int __res;                          \
		\
		__asm__ __volatile__(                       \
		        "	.set	push					\n" \
		        "	.set	reorder					\n"  \
		        "	# gas fails to assemble cfc1 for some archs,	\n"  \
		        "	# like Octeon.					\n"    \
		        "	.set	mips1					\n"    \
		        "	cfc1	%0,"STR(source)"			\n"   \
		        "	.set	pop					\n"  \
		        : "=r" (__res));                        \
		__res;                              \
	})

#define write_32bit_cp1_register(dest,value)                            \
	({                                  \
		__asm__ __volatile__(                       \
		        "	.set	push					\n" \
		        "	.set	reorder					\n"  \
		        "	# gas fails to assemble cfc1 for some archs,	\n"  \
		        "	# like Octeon.					\n"    \
		        "	.set	mips1					\n"    \
		        "       ctc1    %0,"STR(dest)"                          \n"     \
		        "	.set	pop					\n"  \
		        :: "r" (value));                                                \
	})

/*
 * Macros to access the DSP ASE registers
 */

#define rddsp(mask)                         \
	({                                  \
		unsigned int __res;                     \
		\
		__asm__ __volatile__(                       \
		        "	.set	push				\n"      \
		        "	.set	dsp				\n"       \
		        "	.set	noat				\n"      \
		        "	# rddsp $1, %x1				\n"        \
		        "	.word	0x7c000cb8 | (%x1 << 16)	\n"        \
		        "	move	%0, $1				\n"        \
		        "	.set	pop				\n"       \
		        : "=r" (__res)                          \
		        : "i" (mask));                          \
		__res;                              \
	})

#define wrdsp(val, mask)                        \
	do {                                    \
		__asm__ __volatile__(                       \
		        "	.set	push					\n" \
		        "	.set	dsp					\n"  \
		        "	.set	noat					\n" \
		        "	move	$1, %0					\n"   \
		        "	# wrdsp $1, %x1					\n"   \
		        "	.word	0x7c2004f8 | (%x1 << 11)		\n"   \
		        "	.set	pop					\n"  \
		        :                               \
		        : "r" (val), "i" (mask));                   \
	} while (0)

#define _dsp_mfxxx(ins)                         \
	({                                  \
		unsigned long __treg;                       \
		\
		__asm__ __volatile__(                       \
		        "	.set	push					\n" \
		        "	.set	dsp					\n"  \
		        "	.set	noat					\n" \
		        "	.word	(0x00000810 | %1)			\n" \
		        "	move	%0, $1					\n"   \
		        "	.set	pop					\n"  \
		        : "=r" (__treg)                         \
		        : "i" (ins));                           \
		__treg;                             \
	})

#define _dsp_mtxxx(val, ins)                        \
	do {                                    \
		__asm__ __volatile__(                       \
		        "	.set	push					\n" \
		        "	.set	dsp					\n"  \
		        "	.set	noat					\n" \
		        "	move	$1, %0					\n"   \
		        "	.word	(0x00200011 | %1)			\n" \
		        "	.set	pop					\n"  \
		        :                               \
		        : "r" (val), "i" (ins));                    \
	} while (0)

#define _dsp_mflo(reg) _dsp_mfxxx((reg << 21) | 0x0002)
#define _dsp_mfhi(reg) _dsp_mfxxx((reg << 21) | 0x0000)

#define _dsp_mtlo(val, reg) _dsp_mtxxx(val, ((reg << 11) | 0x0002))
#define _dsp_mthi(val, reg) _dsp_mtxxx(val, ((reg << 11) | 0x0000))

#define mflo0() _dsp_mflo(0)
#define mflo1() _dsp_mflo(1)
#define mflo2() _dsp_mflo(2)
#define mflo3() _dsp_mflo(3)

#define mfhi0() _dsp_mfhi(0)
#define mfhi1() _dsp_mfhi(1)
#define mfhi2() _dsp_mfhi(2)
#define mfhi3() _dsp_mfhi(3)

#define mtlo0(x) _dsp_mtlo(x, 0)
#define mtlo1(x) _dsp_mtlo(x, 1)
#define mtlo2(x) _dsp_mtlo(x, 2)
#define mtlo3(x) _dsp_mtlo(x, 3)

#define mthi0(x) _dsp_mthi(x, 0)
#define mthi1(x) _dsp_mthi(x, 1)
#define mthi2(x) _dsp_mthi(x, 2)
#define mthi3(x) _dsp_mthi(x, 3)

/*
 * TLB operations.
 *
 * It is responsibility of the caller to take care of any TLB hazards.
 */
static inline void tlb_probe(void)
{
	__asm__ __volatile__(
	    ".set noreorder\n\t"
	    "tlbp\n\t"
	    ".set reorder");
}

static inline void tlb_read(void)
{

	__asm__ __volatile__(
	    ".set noreorder\n\t"
	    "tlbr\n\t"
	    ".set reorder");
}

static inline void tlb_write_indexed(void)
{
	__asm__ __volatile__(
	    ".set noreorder\n\t"
	    "tlbwi\n\t"
	    ".set reorder");
}

static inline void tlb_write_random(void)
{
	__asm__ __volatile__(
	    ".set noreorder\n\t"
	    "tlbwr\n\t"
	    ".set reorder");
}

static inline void tlbinvf(void)
{
	__asm__ __volatile__(
	    ".set push\n\t"
	    ".set noreorder\n\t"
	    ".word 0x42000004\n\t"
	    ".set pop");
}

/*
 * SMTC Linux requires shutting-down microthread scheduling
 * during CP0 register read-modify-write sequences.
 */
#define __BUILD_SET_C0(name)                    \
	static inline unsigned int                  \
	set_c0_##name(unsigned int set)                 \
	{                               \
		unsigned int res, new;                  \
		\
		res = read_c0_##name();                 \
		new = res | set;                    \
		write_c0_##name(new);                   \
		\
		return res;                     \
	}                               \
	\
	static inline unsigned int                  \
	clear_c0_##name(unsigned int clear)             \
	{                               \
		unsigned int res, new;                  \
		\
		res = read_c0_##name();                 \
		new = res & ~clear;                 \
		write_c0_##name(new);                   \
		\
		return res;                     \
	}                               \
	\
	static inline unsigned int                  \
	change_c0_##name(unsigned int change, unsigned int val)     \
	{                               \
		unsigned int res, new;                  \
		\
		res = read_c0_##name();                 \
		new = res & ~change;                    \
		new |= (val & change);                  \
		write_c0_##name(new);                   \
		\
		return res;                     \
	}


__BUILD_SET_C0(status)
__BUILD_SET_C0(cause)
__BUILD_SET_C0(config)
__BUILD_SET_C0(intcontrol)
__BUILD_SET_C0(intctl)
__BUILD_SET_C0(srsmap)
__BUILD_SET_C0(brcm_config_0)
__BUILD_SET_C0(brcm_bus_pll)
__BUILD_SET_C0(brcm_reset)
__BUILD_SET_C0(brcm_cmt_intr)
__BUILD_SET_C0(brcm_cmt_ctrl)
__BUILD_SET_C0(brcm_config)
__BUILD_SET_C0(brcm_mode)

#endif /* _ASM_MIPSREGS_H */
