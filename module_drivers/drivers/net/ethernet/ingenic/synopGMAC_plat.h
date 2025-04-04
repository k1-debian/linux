/* ===================================================================================
 * Copyright (c) <2009> Synopsys, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software annotated with this license and associated documentation files
 * (the "Software"), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * =================================================================================== */

/**\file
 *  This file serves as the wrapper for the platform/OS dependent functions
 *  It is needed to modify these functions accordingly based on the platform and the
 *  OS. Whenever the synopsys GMAC driver ported on to different platform, this file
 *  should be handled at most care.
 *  The corresponding function definitions for non-inline functions are available in
 *  synopGMAC_plat.c file.
 * \internal
 * -------------------------------------REVISION HISTORY---------------------------
 * Synopsys                 01/Aug/2007            Created
 */

#ifndef SYNOP_GMAC_PLAT_H
#define SYNOP_GMAC_PLAT_H 1

#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#define TR0(fmt, args...) printk(KERN_CRIT "SynopGMAC: " fmt, ##args)

extern int debug_enable;

#if 0
	#ifdef DEBUG
		#undef TR
		#define TR(fmt, args...) printk(KERN_CRIT "SynopGMAC: " fmt, ##args)
	#else
		#define TR(fmt, args...) /* not debugging: nothing */
	#endif
#endif

#define TR(fmt, args...)                        \
	do {                                \
		if (debug_enable) {                 \
			printk(KERN_CRIT "SynopGMAC: " fmt, ##args);    \
		}                           \
	} while(0)

#if 0
typedef int bool;
enum synopGMAC_boolean {
	false = 0,
	true = 1
};
#endif

#if (defined(CONFIG_SOC_X2000) || defined(CONFIG_SOC_M300) || defined(CONFIG_SOC_X2100) || defined(CONFIG_SOC_X2500) || defined(CONFIG_SOC_X1600) || defined(CONFIG_SOC_X2600))
	#define LPI_SUPPORT
#endif

#define DEFAULT_DELAY_VARIABLE  10
#define DEFAULT_LOOP_VARIABLE   10000

/* There are platform related endian conversions
 *
 */

#define LE32_TO_CPU __le32_to_cpu
#define BE32_TO_CPU __be32_to_cpu
#define CPU_TO_LE32 __cpu_to_le32

/* Error Codes */
#define ESYNOPGMACNOERR   0
#define ESYNOPGMACNOMEM   1
#define ESYNOPGMACPHYERR  2
#define ESYNOPGMACBUSY    3

struct Network_interface_data {
	u32 unit;
	u32 addr;
	u32 data;
};

/**
  * These are the wrapper function prototypes for OS/platform related routines
  */

void *plat_alloc_memory(u32);
void   plat_free_memory(void *);

void *plat_alloc_consistent_dmaable_memory(struct device *, u32, u32 *);
void   plat_free_consistent_dmaable_memory(struct device *, u32, void *, u32);

void   plat_delay(u32);

/**
 * The Low level function to read register contents from Hardware.
 *
 * @param[in] pointer to the base of register map
 * @param[in] Offset from the base
 * \return  Returns the register contents
 */
static u32 __inline__ synopGMACReadReg(u32 *RegBase, u32 RegOffset)
{

	u32 addr = (u32)RegBase + RegOffset;
	u32 data = readl((void *)addr);
	//printk("=====>%s: RegBase = 0x%08x RegOffset = 0x%08x RegData = 0x%08x\n", __FUNCTION__, (u32)RegBase, RegOffset, data );
	return data;
}

/**
 * The Low level function to write to a register in Hardware.
 *
 * @param[in] pointer to the base of register map
 * @param[in] Offset from the base
 * @param[in] Data to be written
 * \return  void
 */
static void  __inline__ synopGMACWriteReg(u32 *RegBase, u32 RegOffset, u32 RegData)
{

	u32 addr = (u32)RegBase + RegOffset;
	//printk("=====>%s RegBase = 0x%08x RegOffset = 0x%08x RegData = 0x%08x\n", __FUNCTION__,(u32) RegBase, RegOffset, RegData );
	writel(RegData, (void *)addr);
	//*((volatile unsigned int *)addr) = RegData;
	return;
}

/**
 * The Low level function to set bits of a register in Hardware.
 *
 * @param[in] pointer to the base of register map
 * @param[in] Offset from the base
 * @param[in] Bit mask to set bits to logical 1
 * \return  void
 */
static void __inline__ synopGMACSetBits(u32 *RegBase, u32 RegOffset, u32 BitPos)
{
	u32 addr = (u32)RegBase + RegOffset;
	u32 data = readl((void *)addr);
	data |= BitPos;
	//  TR("%s !!!!!!!!!!!!! RegOffset = 0x%08x RegData = 0x%08x\n", __FUNCTION__, RegOffset, data );
	writel(data, (void *)addr);
	//  TR("%s !!!!!!!!!!!!! RegOffset = 0x%08x RegData = 0x%08x\n", __FUNCTION__, RegOffset, data );
	return;
}

/**
 * The Low level function to clear bits of a register in Hardware.
 *
 * @param[in] pointer to the base of register map
 * @param[in] Offset from the base
 * @param[in] Bit mask to clear bits to logical 0
 * \return  void
 */
static void __inline__ synopGMACClearBits(u32 *RegBase, u32 RegOffset, u32 BitPos)
{
	u32 addr = (u32)RegBase + RegOffset;
	u32 data = readl((void *)addr);
	data &= (~BitPos);
	//  TR("%s !!!!!!!!!!!!!! RegOffset = 0x%08x RegData = 0x%08x\n", __FUNCTION__, RegOffset, data );
	writel(data, (void *)addr);
	//  TR("%s !!!!!!!!!!!!! RegOffset = 0x%08x RegData = 0x%08x\n", __FUNCTION__, RegOffset, data );
	return;
}

/**
 * The Low level function to Check the setting of the bits.
 *
 * @param[in] pointer to the base of register map
 * @param[in] Offset from the base
 * @param[in] Bit mask to set bits to logical 1
 * \return  returns TRUE if set to '1' returns FALSE if set to '0'. Result undefined there are no bit set in the BitPos argument.
 *
 */
static bool __inline__ synopGMACCheckBits(u32 *RegBase, u32 RegOffset, u32 BitPos)
{
	u32 addr = (u32)RegBase + RegOffset;
	u32 data = readl((void *)addr);
	data &= BitPos;
	if (data) {
		return true;
	} else {
		return false;
	}

}

#endif
