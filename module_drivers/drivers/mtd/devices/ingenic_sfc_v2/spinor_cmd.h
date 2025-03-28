#ifndef __SPINOR_CMD_H
#define __SPINOR_CMD_H

/* Flash opcodes. */
#define SPINOR_OP_RSTEN     0x66    /* reset enable */
#define SPINOR_OP_RST       0x99    /* reset */
#define SPINOR_OP_WREN      0x06    /* Write enable */
#define SPINOR_OP_RDSR      0x05    /* Read status register */
#define SPINOR_OP_RDSR_1    0x35    /* Read status1 register */
#define SPINOR_OP_RDSR_2    0x15    /* Read status2 register */
#define SPINOR_OP_WRSR      0x01    /* Write status register 1 byte */
#define SPINOR_OP_WRSR_1    0x31    /* Write status1 register 1 byte */
#define SPINOR_OP_WRSR_2    0x11    /* Write status2 register 1 byte */
#define SPINOR_OP_READ      0x03    /* Read data bytes (low frequency) */
#define SPINOR_OP_READ_FAST 0x0b    /* Read data bytes (high frequency) */
#define SPINOR_OP_READ_1_1_2    0x3b    /* Read data bytes (Dual SPI) */
#define SPINOR_OP_READ_1_1_4    0x6b    /* Read data bytes (Quad SPI) */
#define SPINOR_OP_PP        0x02    /* Page program (up to 256 bytes) */
#define SPINOR_OP_QPP       0x32    /* Page program (up to 256 bytes) */
#define SPINOR_OP_BE_4K     0x20    /* Erase 4KiB block */
#define SPINOR_OP_BE_4K_PMC 0xd7    /* Erase 4KiB block on PMC chips */
#define SPINOR_OP_BE_32K    0x52    /* Erase 32KiB block */
#define SPINOR_OP_CHIP_ERASE    0xc7    /* Erase whole flash chip */
#define SPINOR_OP_SE        0xd8    /* Sector erase (usually 64KiB) */
#define SPINOR_OP_RDID      0x9f    /* Read JEDEC ID */
#define SPINOR_OP_RDCR      0x35    /* Read configuration register */
#define SPINOR_OP_RDFSR     0x70    /* Read flag status register */
#define SPINOR_OP_DIE_SEL   0xc2    /* Software Die Select */
#define SPINOR_OP_READ_DIE_ID   0xf8    /* Read Active Die ID */

/* 4-byte address opcodes - used on Spansion and some Macronix flashes. */
#define SPINOR_OP_READ4     0x13    /* Read data bytes (low frequency) */
#define SPINOR_OP_READ4_FAST    0x0c    /* Read data bytes (high frequency) */
#define SPINOR_OP_READ4_1_1_2   0x3c    /* Read data bytes (Dual SPI) */
#define SPINOR_OP_READ4_1_1_4   0x6c    /* Read data bytes (Quad SPI) */
#define SPINOR_OP_PP_4B     0x12    /* Page program (up to 256 bytes) */
#define SPINOR_OP_QPP_4B    0x34    /* Page program (up to 256 bytes) */
#define SPINOR_OP_SE_4B     0xdc    /* Sector erase (usually 64KiB) */
#define SPINOR_OP_BE_32K_4B 0x5c    /* Erase 32KiB block */

/* Used for SST flashes only. */
#define SPINOR_OP_BP        0x02    /* Byte program */
#define SPINOR_OP_WRDI      0x04    /* Write disable */
#define SPINOR_OP_AAI_WP    0xad    /* Auto address increment word program */

/* Used for Macronix and Winbond flashes. */
#define SPINOR_OP_EN4B      0xb7    /* Enter 4-byte mode */
#define SPINOR_OP_EX4B      0xe9    /* Exit 4-byte mode */

/* Used for Spansion flashes only. */
#define SPINOR_OP_BRWR      0x17    /* Bank register write */

/* Status Register bits. */
#define SR_WIP          1   /* Write in progress */
#define SR_WEL          2   /* Write enable latch */
#define SR_SQE          (1 << 1)    /* QUAD MODE enable */
/* meaning of other SR_* bits may differ between vendors */
#define SR_BP0          4   /* Block protect 0 */
#define SR_BP1          8   /* Block protect 1 */
#define SR_BP2          0x10    /* Block protect 2 */
#define SR_SRWD         0x80    /* SR write protect */

#define SR_QUAD_EN_MX       0x40    /* Macronix Quad I/O */

/* Flag Status Register bits */
#define FSR_READY       0x80

/* Configuration Register bits. */
#define CR_QUAD_EN_SPAN     0x2 /* Spansion Quad I/O */

#define CMD_WREN    0x06    /* Write Enable */
#define CMD_EN4B                0xB7
#define CMD_EX4B                0xE9
/* nor cmd entry deep power down and Release from Deep Power-Down and Read Device ID */
#define CMD_DP (0xB9)
#define CMD_RDP (0xAB)

#define BUFFER_SIZE PAGE_SIZE

#define NOR_SIZE_16M    0x1000000

#endif
