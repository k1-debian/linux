
#ifndef __JZSOC_SOC_DEV_H__
#define __JZSOC_SOC_DEV_H__

/*
 * Define the module base addresses
 */

/* AHB0 BUS Devices Base */
#define HARB0_IOBASE            0x13000000
#define DDRC_BASE               0xb34f0000
#define DDRC1_IOBASE            0x13010000 /*DDR_APB_BASE*/
#define DDRC_IOBASE             0x134f0000 /*TODO:*/
#define DDRPHY_IOBASE           0x13011000 /*TODO:*/
#define AXI_ARB1_IOBASE         0x13013000 /*TODO:*/
#define AXI_ARB2_IOBASE         0x13014000 /*TODO:*/
#define LCDC_IOBASE             0x13050000
#define MSC0_IOBASE             0x13060000
#define MSC1_IOBASE             0x13070000
#define IPU_IOBASE              0x13080000
#define BSCALER_IOBASE          0x13090000
#define MONITOR_IOBASE          0x130a0000
#define I2D_IOBASE              0x130b0000
#define VO_IOBASE               0x130c0000
#define DRAW_BOX_IOBASE         0x130d0000
#define ISP_IOBASE              0x13300000
#define LZMA_IOBASE             0x13090000

/* AHB1 BUS Devices Base */
#define RTC_IOBASE              0x132a0000
#define EL150_IOBASE            0x13200000
#define RADIX_IOBASE            0x13100000
#define RADIX_IOBASE_UNIT(ID)   (RADIX_IOBASE + 0x400000 * ID)
#define AVPU_IOBASE             0x13200000
#define AVPU_IOBASE_UNIT(ID)    (AVPU_IOBASE + 0x400000 * ID)

/* AHB2 BUS Devices Base */
#define HARB2_IOBASE            0x13400000
#define NEMC_IOBASE             0x13410000
#define PDMA_IOBASE             0x13420000
#define AES_IOBASE              0x13430000
#define SFC_IOBASE              0x13440000
#define HASH_IOBASE             0x13480000
#define GMAC_IOBASE             0x134b0000
#define RSA_IOBASE              0x134c0000
#define OTG_IOBASE              0x13500000
#define EFUSE_IOBASE            0x13540000
#define INTC_IOBASE             0x10001000
#define DDRC_H2_IOBASE          0x134f0000 /* DDR Controller Register @AHB2 */

/* CPU and OST */
#define G_OST_IOBASE            0x12000000 /* G_OST_BASE */
#define N_OST_IOBASE            0x12100000 /* N_OST_BASE */
#define CCU_IOBASE              0x12200000
#define INTCN_IOBASE            0x12300000
#define SRAM_IOBASE             0x12400000
#define NNDMA_IOBASE            0x12500000
#define LEPOST_IOBASE           0x12600000  /* RISC-V OST */
#define LEPCCU_IOBASE           0x12700000  /* RISC-V CCU */

/* APB BUS Devices Base */
#define CPM_IOBASE              0x10000000
#define TCU_IOBASE              0x10002000
#define MIPI_DSI_TX_IOBASE      0x10003000
#define MIPI_DSI_PHY_IOBASE     0x10004000
#define GPIO_IOBASE             0x10010000
#define AIC0_IOBASE             0x10020000
#define CODEC_IOBASE            0x10021000
#define MIPI_PHY_IOBASE         0x10022000
#define MIPI_CSI_IOBASE         0x10023000
#define UART0_IOBASE            0x10030000
#define UART1_IOBASE            0x10031000
#define UART2_IOBASE            0x10032000
#define UART3_IOBASE            0x10033000
#define DMIC_IOBASE             0x10034000
#define SSISLV_IOBASE           0x10040000
#define SSI0_IOBASE             0x10043000
#define SSI1_IOBASE             0x10044000
#define I2C0_IOBASE             0x10050000
#define I2C1_IOBASE             0x10051000
#define I2C2_IOBASE             0x10052000
#define I2C3_IOBASE             0x10053000
#define MIPI_RX_4L_IOBASE       0x10054000
#define USB_IOBASE              0x10060000
#define DES_IOBASE              0x10061000
#define SADC_IOBASE             0x10070000
#define DTRNG_IOBASE            0x10072000
#define WDT_IOBASE              0x10002000

/* NAND CHIP Base Address*/
#define NEMC_CS1_IOBASE         0X1b000000
#define NEMC_CS2_IOBASE         0X1a000000
#define NEMC_CS3_IOBASE         0X19000000
#define NEMC_CS4_IOBASE         0X18000000
#define NEMC_CS5_IOBASE         0X17000000
#define NEMC_CS6_IOBASE         0X16000000

#endif
