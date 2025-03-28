#ifndef __INGENIC_PDMA_H__
#define __INGENIC_PDMA_H__
#include<generated/autoconf.h>

#define INGENIC_DMA_TYPE_REQ_MSK    0xff
#define INGENIC_DMA_TYPE_CH_SFT     8
#define INGENIC_DMA_TYPE_CH_MSK     (0xff << INGENIC_DMA_TYPE_CH_SFT)
#define INGENIC_DMA_TYPE_CH_EN      (1 << 16)
#define INGENIC_DMA_TYPE_PROG       (1 << 17)
#define INGENIC_DMA_TYPE_SPEC       (1 << 18)

#define INGENIC_DMA_CTRL_BUS_SFT    19
#define INGENIC_DMA_CTRL_BUS_MSK    (3 << INGENIC_DMA_CTRL_BUS_SFT)
#define INGENIC_DMA_AHB2_CONTROL    (0 << 19)
#define INGENIC_DMA_AHB_MCU_CONTROL (1 << 19)

#define INGENIC_DMA_CH(ch)      ((((ch) << INGENIC_DMA_TYPE_CH_SFT) & INGENIC_DMA_TYPE_CH_MSK) | INGENIC_DMA_TYPE_CH_EN)
#define INGENIC_DMA_TYPE(type)  ((type) & INGENIC_DMA_TYPE_REQ_MSK)
#define INGENIC_DMA_TYPE_CH(type, ch)   (INGENIC_DMA_TYPE((type)) | INGENIC_DMA_CH((ch)))
#define INGENIC_DMA_PG_CH(type, ch) (INGENIC_DMA_TYPE_CH((type), (ch)) | INGENIC_DMA_TYPE_PROG_MSK)
#define INGENIC_DMA_SP_CH(type, ch) (INGENIC_DMA_PG_CH(type, id) | INGENIC_DMA_TYPE_SPEC_MSK)

// BUS CTRL
#define INGENIC_DMA_CTRL_AHB2_BUS   0
#define INGENIC_DMA_CTRL_AHB_MCU_BUS    1
#define INGENIC_GET_CTRL_BUS(bus)   (bus >> INGENIC_DMA_CTRL_BUS_SFT)
#define INGENIC_DMA_AHB_MCU_TYPE(type)  (INGENIC_DMA_TYPE(type) | INGENIC_DMA_AHB_MCU_CONTROL)

#ifdef CONFIG_SOC_X1000
	#include <dt-bindings/dma/x1000-dma.h>
#endif

#if (defined(CONFIG_SOC_X2000) || defined(CONFIG_SOC_M300) || defined(CONFIG_SOC_X2100))
	#include <dt-bindings/dma/ingenic-pdma-x2000.h>
#endif

#if (defined(CONFIG_SOC_X1600) || defined(CONFIG_SOC_X1660))
	#include <dt-bindings/dma/ingenic-pdma-x1600.h>
#endif

#ifdef CONFIG_SOC_X2600
	#include <dt-bindings/dma/ingenic-pdma-x2600.h>
#endif

#ifdef CONFIG_SOC_AD100
	#include <dt-bindings/dma/ingenic-pdma-ad100.h>
#endif

#ifdef CONFIG_SOC_X2500
	#include <dt-bindings/dma/ingenic-pdma-x2500.h>
#endif

#endif  /* __INGENIC_PDMA_H__ */
