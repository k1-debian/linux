#ifndef __SPINAND_CMD_H
#define __SPINAND_CMD_H

#define SPINAND_CMD_RDID                0x9f    /* read spi nand id */
#define SPINAND_CMD_WREN                0x06    /* spi nand write enable */
#define SPINAND_CMD_PRO_LOAD                    0x02    /* program load */
#define SPINAND_CMD_PRO_LOAD_X4                 0x32    /* program load fast*/
#define SPINAND_CMD_PRO_EN              0x10    /* program load execute */
#define SPINAND_CMD_PARD                0x13    /* read page data to spi nand cache */
#define SPINAND_CMD_PLRd                0x84    /* program load random data */
#define SPINAND_CMD_PLRd_X4             0xc4    /* program load random data x4*/
#define SPINAND_CMD_RDCH                0x03    /* read from spi nand cache */
#define SPINAND_CMD_RDCH_X4             0x6b    /* read from spi nand cache */
#define SPINAND_CMD_FRCH                0x0b    /* fast read from spi nand cache */
#define SPINAND_CMD_FRCH_IO             0xeb    /* for Quad I/O SPI mode */
#define SPINAND_CMD_ERASE_128K                  0xd8    /* erase spi nand block 128K */
#define SPINAND_CMD_GET_FEATURE                 0x0f    /* get spi nand feature */
#define SPINAND_CMD_SET_FEATURE                 0x1f    /* set spi nand feature */
#define SPINAND_CMD_RESET               0xff    /* reset nand flash device */

#define SPINAND_ADDR_PROTECT    0xa0    /* protect addr */
#define SPINAND_ADDR_STATUS     0xc0    /* get feature status addr */
#define SPINAND_ADDR_FEATURE    0xb0    /* set feature addr */

#define SPINAND_IS_BUSY                 (1 << 0)                /* PROGRAM EXECUTE, PAGE READ, BLOCK ERASE, or RESET command executing */
#define SPINAND_OP_BL_128K      (128 * 1024)

#endif
