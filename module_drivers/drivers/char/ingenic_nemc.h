/*
 * ingenic external memory controller (NEMC)
*/
#ifndef __LINUX_JZ4780_NEMC_H__
#define __LINUX_JZ4780_NEMC_H__

#include <linux/types.h>

struct device;

/*
 *  * Number of NEMC banks. Note that there are actually 2, but they are numbered
 *   * from 1.
 *    */
#define INGENIC_NEMC_NUM_BANKS   2

/**
 *  * enum jz4780_nemc_bank_type - device types which can be connected to a bank
 *   * @JZ4780_NEMC_BANK_SRAM: SRAM
 *    * @JZ4780_NEMC_BANK_NAND: NAND
 *     */
enum ingenic_nemc_bank_type {
	INGENIC_NEMC_BANK_SRAM,
	INGENIC_NEMC_BANK_NAND,
};

unsigned int ingenic_nemc_num_banks(struct device *dev);

void ingenic_nemc_set_type(struct device *dev, unsigned int bank,
                           enum ingenci_nemc_bank_type type);
void ingenic_nemc_assert(struct device *dev, unsigned int bank,
                         bool assert);

#endif /* __LINUX_JZ4780_NEMC_H__ */
