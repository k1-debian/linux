#ifndef __SFC_REGISTER_H
#define __SFC_REGISTER_H

/* SFC register */

#define SFC_GLB0            (0x0000)
#define SFC_DEV_CONF            (0x0004)
#define SFC_DEV_STA_EXP         (0x0008)
#define SFC_DEV0_STA_RT         (0x000c)
#define SFC_DEV_STA_MSK         (0x0010)
#define SFC_TRAN_CONF0(n)       (0x0014 + (n * 4))
#define SFC_TRAN_LEN            (0x002c)
#define SFC_DEV_ADDR(n)         (0x0030 + (n * 4))
#define SFC_DEV_ADDR_PLUS(n)        (0x0048 + (n * 4))
#define SFC_MEM_ADDR            (0x0060)
#define SFC_TRIG            (0x0064)
#define SFC_SR              (0x0068)
#define SFC_SCR             (0x006c)
#define SFC_INTC            (0x0070)
#define SFC_FSM             (0x0074)
#define SFC_CGE             (0x0078)
#define SFC_CMD_IDX                     (0x007c)
#define SFC_COL_ADDR                    (0x0080)
#define SFC_ROW_ADDR                    (0x0084)
#define SFC_STA_ADDR0                   (0x0088)
#define SFC_STA_ADDR1                   (0x008c)
#define SFC_DES_ADDR                    (0x0090)
#define SFC_GLB1            (0x0094)
#define SFC_DEV1_STA_RT         (0x0098)
#define SFC_TRAN_CONF1(n)       (0x009c + (n * 4))
#define SFC_CDT                         (0x0800)  //0x800 ~ 0xbff
#define SFC_RM_DR           (0x1000)

/* For SFC_GLB0 */
#define GLB0_POLL_TIME_OFFSET       (16)
#define GLB0_POLL_TIME_MSK      (0xffff << GLB0_POLL_TIME_OFFSET)
#define GLB0_DES_EN         (1 << 15)
#define GLB0_CDT_EN         (1 << 14)
#define GLB0_TRAN_DIR           (1 << 13)
#define GLB0_TRAN_DIR_WRITE     (1)
#define GLB0_TRAN_DIR_READ      (0)
#define GLB0_THRESHOLD_OFFSET       (7)
#define GLB0_THRESHOLD_MSK      (0x3f << GLB0_THRESHOLD_OFFSET)
#define GLB0_OP_MODE            (1 << 6)
#define GLB0_OP_OFFSET                   (6)
#define SLAVE_MODE          (0x0)
#define DMA_MODE            (0x1)
#define GLB0_PHASE_NUM_OFFSET       (3)
#define GLB0_PHASE_NUM_MSK      (0x7  << GLB0_PHASE_NUM_OFFSET)
#define GLB0_WP_EN          (1 << 2)
#define GLB0_BURST_MD_OFFSET        (0)
#define GLB0_BURST_MD_MSK       (0x3  << GLB0_BURST_MD_OFFSET)

/* For SFC_DEV_CONF */
#define DEV_CONF_STA_ENDIAN_OFFSET  (31)
#define STA_ENDIAN_LSB          (0)
#define STA_ENDIAN_MSB          (1)
#define DEV_CONF_SMP_DELAY_OFFSET   (16)
#define DEV_CONF_SMP_DELAY_MSK      (0x1f << DEV_CONF_SMP_DELAY_OFFSET)
#define DEV_CONF_SMP_DELAY_0        (0)
#define DEV_CONF_SMP_DELAY_45       (1)
#define DEV_CONF_SMP_DELAY_90       (2)
#define DEV_CONF_SMP_DELAY_135      (3)
#define DEV_CONF_SMP_DELAY_180      (4)
#define DEV_CONF_SMP_DELAY_225      (5)
#define DEV_CONF_SMP_DELAY_270      (6)
#define DEV_CONF_SMP_DELAY_315      (7)
#define DEV_CONF_SMP_DELAY_1        (8)
#define DEV_CONF_SMP_DELAY_2        (16)
#define DEV_CONF_CMD_TYPE       (0x1 << 15)
#define DEV_CONF_STA_TYPE_OFFSET    (13)
#define DEV_CONF_STA_TYPE_MSK       (0x3 << DEV_CONF_STA_TYPE_OFFSET)
#define DEV_CONF_THOLD_OFFSET       (11)
#define DEV_CONF_THOLD_MSK      (0x3 << DEV_CONF_THOLD_OFFSET)
#define DEV_CONF_TSETUP_OFFSET      (9)
#define DEV_CONF_TSETUP_MSK     (0x3 << DEV_CONF_TSETUP_OFFSET)
#define DEV_CONF_TSH_OFFSET     (5)
#define DEV_CONF_TSH_MSK        (0xf << DEV_CONF_TSH_OFFSET)
#define DEV_CONF_CPHA           (0x1 << 4)
#define DEV_CONF_CPOL           (0x1 << 3)
#define DEV_CONF_CEDL           (0x1 << 2)
#define DEV_CONF_HOLDDL         (0x1 << 1)
#define DEV_CONF_WPDL           (0x1 << 0)

/* For SFC_TRAN_CONF0 */
#define TRAN_CONF0_CLK_MODE     (29)
#define TRAN_CONF0_CLK_MODE_MSK     (0x7 << TRAN_CONF0_CLK_MODE)
#define TRAN_CONF0_CLK_MODE_SSS     (0)
#define TRAN_CONF0_CLK_MODE_SSD     (1)
#define TRAN_CONF0_CLK_MODE_SDS     (2)
#define TRAN_CONF0_CLK_MODE_SDD     (3)
#define TRAN_CONF0_CLK_MODE_DSS     (4)
#define TRAN_CONF0_CLK_MODE_DSD     (5)
#define TRAN_CONF0_CLK_MODE_DDS     (6)
#define TRAN_CONF0_CLK_MODE_DDD     (7)
#define TRAN_CONF0_ADDR_WIDTH_OFFSET    (26)
#define TRAN_CONF0_ADDR_WIDTH_MSK   (0x7 << TRAN_CONF0_ADDR_WIDTH_OFFSET)
#define TRAN_CONF0_POLLEN       (1 << 25)
#define TRAN_CONF0_POLL_OFFSET      (25)
#define TRAN_CONF0_CMDEN        (1 << 24)
#define TRAN_CONF0_FMAT         (1 << 23)
#define TRAN_CONF0_FMAT_OFFSET      (23)
#define TRAN_CONF0_DMYBITS_OFFSET   (17)
#define TRAN_CONF0_DMYBITS_MSK      (0x3f << TRAN_CONF0_DMYBITS_OFFSET)
#define TRAN_CONF0_DATEEN       (1 << 16)
#define TRAN_CONF0_DATEEN_OFFSET    (16)
#define TRAN_CONF0_CMD_OFFSET       (0)
#define TRAN_CONF0_CMD_MSK      (0xffff << TRAN_CONF0_CMD_OFFSET)

/* For SFC_TRAN_CONF1 */
#define TRAN_CONF1_DATA_ENDIAN      (1 << 18)
#define TRAN_CONF1_DATA_ENDIAN_OFFSET   (18)
#define TRAN_CONF1_DATA_ENDIAN_LSB  (0)
#define TRAN_CONF1_DATA_ENDIAN_MSB  (1)
#define TRAN_CONF1_WORD_UNIT_OFFSET (16)
#define TRAN_CONF1_WORD_UNIT_MSK    (3 << 16)
#define TRAN_CONF1_TRAN_MODE_OFFSET (4)
#define TRAN_CONF1_TRAN_MODE_MSK    (0xf << TRAN_CONF1_TRAN_MODE_OFFSET)
#define TRAN_CONF1_SPI_STANDARD     (0x0)
#define TRAN_CONF1_SPI_DUAL     (0x1)
#define TRAN_CONF1_SPI_QUAD     (0x5)
#define TRAN_CONF1_SPI_IO_QUAD      (0x6)
#define TRAN_CONF1_SPI_OCTAL        (0x9)
#define TRAN_CONF1_SPI_IO_OCTAL     (0xa)
#define TRAN_CONF1_SPI_FULL_OCTAL   (0xb)


/* For SFC_TRIG */
#define TRIG_FLUSH          (1 << 2)
#define TRIG_STOP           (1 << 1)
#define TRIG_START          (1 << 0)

/* For SFC_SR */
#define SFC_WORKING     (1 << 7)
#define SFC_BUSY        (0x3 << 5)
#define SFC_END         (1 << 4)
#define SFC_TREQ        (1 << 3)
#define SFC_RREQ        (1 << 2)
#define SFC_OVER        (1 << 1)
#define SFC_UNDER       (1 << 0)

/* For SFC_SCR */
#define CLR_END         (1 << 4)
#define CLR_TREQ        (1 << 3)
#define CLR_RREQ        (1 << 2)
#define CLR_OVER        (1 << 1)
#define CLR_UNDER       (1 << 0)

//SFC_CMD_IDX
#define CMD_IDX_MSK                     (0x3f << 0)
#define CDT_DATAEN_MSK                  (0x1 << 31)
#define CDT_DATAEN_OFF                  (31)
#define CDT_DIR_MSK                     (0x1 << 30)
#define CDT_DIR_OFF                     (30)

/* For SFC_GLB1 */
#define GLB1_DQS_EN         (1 << 2)
#define GLB1_CHIP_SEL_OFFSET        (0)
#define GLB1_CHIP_SEL_MSK       (0x3 << 0)
#define GLB1_CHIP_SEL_0         (0)
#define GLB1_CHIP_SEL_1         (1)
#define GLB1_CHIP_SEL_01        (2)

#define N_MAX           6
#define MAX_SEGS        128

#define CHANNEL_0       0
#define CHANNEL_1       1
#define CHANNEL_2       2
#define CHANNEL_3       3
#define CHANNEL_4       4
#define CHANNEL_5       5

#define ENABLE          1
#define DISABLE         0

#define COM_CMD         1   // common cmd
#define POLL_CMD        2   // the cmd will poll the status of flash,ext: read status

#define DMA_OPS         1
#define CPU_OPS         0

#define TM_STD_SPI      0
#define TM_DI_DO_SPI    1
#define TM_DIO_SPI      2
#define TM_FULL_DIO_SPI 3
#define TM_QI_QO_SPI    5
#define TM_QIO_SPI      6
#define TM_FULL_QIO_SPI 7
#define TM_OCTAL_SPT        9
#define TM_OCTAL_IO_SPI     10
#define TM_OCTAL_FULL_SPI   11


#define DEFAULT_CDT     1
#define UPDATE_CDT      2
#define DEFAULT_ADDRSIZE    3
#define DEFAULT_ADDRMODE    0


#define THRESHOLD       32

#define SFC_NOR_RATE    110
#define DEF_ADDR_LEN    3
#define DEF_TCHSH       6
#define DEF_TSLCH       6
#define DEF_TSHSL_R     20
#define DEF_TSHSL_W     50


#endif
