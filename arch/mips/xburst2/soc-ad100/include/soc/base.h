
#ifndef __JZSOC_SOC_DEV_H__
#define __JZSOC_SOC_DEV_H__

/*
 * Define the module base addresses
 */

/* AHB0 BUS Devices Base */
#define HARB0_IOBASE    0x13000000 /* 64KB, AHB Bus Aribtier */
#define DDR_PHY_IOBASE  0x13011000 /* 4KB, DDR PHY */
#define DDRC_H0_IOBASE  0x13012000 /* 4KB, DDR Controller Registers@ASHB0 */
#define DPU_IOBASE      0x13050000 /* 64KB, LCD Controller */
#define CIM_IOBASE      0x13060000 /* 64KB, Camera Interface Module */
#define ROTATE_IOBASE   0x13070000 /* 64KB, Rotate DMA */
#define G2D_IOBASE      0x13100000 /* 64KB, Gal 2D */
#define JPEGD_IOBASE    0x13200000 /* 64KB, Jpeg decoder */
#define JPEGE_IOBASE    0x13210000 /* 64KB, Jpeg encoder */
#define H264D_IOBASE    0x13300000 /* 64KB, H264 encoder */


/* AHB2 Bus Devices Base */
#define HARB2_IOBASE    0x13400000 /* 64KB, AHB Bus Arbiter */
#define NEMC_IOBASE     0x13410000 /* 64KB, External Normal Memory/Boot ROM/OTP Controller */
#define PDMA_IOBASE     0x13420000 /* 64KB, Programmable DMA Controller */
#define AES_IOBASE      0x13430000 /* 64KB, AES */
#define SFC_IOBASE      0x13440000 /* 64KB, SPI Flash Controller */
#define MSC0_IOBASE     0x13450000 /* 64KB, MMC SD Controller0 */
#define MSC1_IOBASE     0x13460000 /* 64KB, MMC SD Controller1 */
#define HASH_IOBASE     0x13470000 /* 64KB, HASH */
#define EFUSE_IOBASE    0x13480000 /* 64KB, EFUSE */
#define CAN0_IOBASE     0x13480000 /* 64KB, CAN0 */
#define CAN1_IOBASE     0x13490000 /* 64KB, CAN1 */
#define MAC_IOBASE      0x134b0000 /* 64KB, GMAC/MAC */
#define DMIC_IOBASE     0x134C0000 /* 64KB, DMIC */
#define DDRC_H2_IOBASE  0x134f0000 /* 4KB, DDR Controller Register @AHB2 */
#define OTG_IOBASE      0x13500000 /* 256k OTG2.0 Controller */
#define USB_IOBASE      0x13540000 /* 256k USB Controller */
#define INTC_IOBASE     0x10001000 /* Interrupt Controller */ /* TODO: to be fix */

/* APB BUS Devices Base */
#define CPM_IOBASE          0x10000000 /* 4KB, Clocks and Power Manager */
#define AUDIO_IOBASE        0x10020000 /* 64KB, Aduio System */
#define CODEC_PHY_IOBASE    0x10021000 /* 64KB, Aduio System */
#define MIPI_DSI_IOBASE     0x10023000 /* 64KB, Aduio System */
#define MIPI_TXPHY_IOBASE   0x10024000 /* 64KB, Aduio System */

#define UART0_IOBASE    0x10030000 /* 4KB, UART0 Controller */
#define UART1_IOBASE    0x10031000 /* 4KB, UART1 Controller */
#define UART2_IOBASE    0x10032000 /* 4KB, UART2 Controller */
#define UART3_IOBASE    0x10033000 /* 4KB, UART3 Controller */
#define UART4_IOBASE    0x10034000 /* 4KB, UART4 Controller */
#define UART5_IOBASE    0x10035000 /* 4KB, UART5 Controller */
#define UART6_IOBASE    0x10036000 /* 4KB, UART6 Controller */
#define UART7_IOBASE    0x10037000 /* 4KB, UART7 Controller */
#define SSI0_IOBASE     0x10043000 /* 4KB, Synchronous Serial Interface */
#define SSI1_IOBASE     0x10044000 /* 4KB, Synchronous Serial Interface */
#define SSI_SLV_IOBASE  0x10045000 /* 4KB, Synchronous Serial SLV Interface */
#define I2C0_IOBASE     0x10050000 /* 4KB, I2C 0 Bus Interface */
#define I2C1_IOBASE     0x10051000 /* 4KB, I2C 1 Bus Interface */
#define I2C2_IOBASE     0x10052000 /* 4KB, I2C 2 Bus Interface */
#define I2C3_IOBASE     0x10053000 /* 4KB, I2C 3 Bus Interface */
#define PCM_IOBASE      0x10071000 /* 4KB,  PCM  Interface */
#define DTRNG_IOBASE    0x10072000 /* 4KB, DTRNG Base */
#define USB_PHY_IOBASE  0x10078000 /* 4KB, USB PHY Base */
#define WDT_IOBASE   0x13630000
/* MCU_AHB_BUS Devices Base */
#define GPIO_IOBASE 0x13601000 /* 4KB, General-Purpose I/O */
#define PWM_IOBASE  0x13610000 /* 64KB, PWM */

/* MCU_APB_BUS Devices Base */
#define RTC_IOBASE  0x10003000 /* 4KB, Real-Time Clock */

/* TODO: Others To Be add */
/* NAND CHIP Base Address */
#define NEMC_CS1_IOBASE 0X1b000000
#define NEMC_CS2_IOBASE 0X1a000000


#define OST_IOBASE  0x12000000
#define TCU_IOBASE  0x10002000

#define DDRC_BASE       0xb34f0000

#endif
