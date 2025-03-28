/*
 *  Copyright (C) 2010 Ingenic Semiconductor Inc.
 *
 *  Author: <zpzhong@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_INGENIC_IRQ_H__
#define __ASM_MACH_INGENIC_IRQ_H__

#include <dt-bindings/interrupt-controller/mips-irq.h>

/* IRQ for MIPS CPU */
// #define MIPS_CPU_IRQ_BASE 		0
#define MIPS_CPU_IRQ(x)			(MIPS_CPU_IRQ_BASE + (x))

#define XBURST_INT			MIPS_CPU_IRQ(2)
#define XBURST_SYS_OST			MIPS_CPU_IRQ(3)

#define MIPS_CPU_IRQS			(MIPS_CPU_IRQ(7) + 1 - MIPS_CPU_IRQ_BASE)
#define XBURST_SOC_IRQ_BASE		MIPS_CPU_IRQS

#define INTC_CHIP_NUM	2

#ifndef NR_IRQS
#define NR_IRQS		(200)
#endif

#endif
