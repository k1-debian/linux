#ifndef __INGENIC_MMC_REG_H__
#define __INGENIC_MMC_REG_H__
#define MMC_BOOT_AREA_PROTECTED     (0x1234)    /* Can not modified the area protected */
#define MMC_BOOT_AREA_OPENED        (0x4321)    /* Can modified the area protected */

#define MSC_CTRL            0x000
#define MSC_STAT            0x004
#define MSC_CLKRT           0x008
#define MSC_CMDAT           0x00C
#define MSC_RESTO           0x010
#define MSC_RDTO            0x014
#define MSC_BLKLEN          0x018
#define MSC_NOB             0x01C
#define MSC_SNOB            0x020
#define MSC_IMASK           0x024
#define MSC_IFLG            0x028
#define MSC_CMD             0x02C
#define MSC_ARG             0x030
#define MSC_RES             0x034
#define MSC_RXFIFO          0x038
#define MSC_TXFIFO          0x03C
#define MSC_LPM             0x040
#define MSC_DMAC            0x044
#define MSC_DMANDA          0x048
#define MSC_DMADA           0x04C
#define MSC_DMALEN          0x050
#define MSC_DMACMD          0x054
#define MSC_CTRL2           0x058
#define MSC_RTCNT           0x05C
#define MSC_DEBUG           0x0FC

/* MSC Clock and Control Register (MSC_CTRL) */
#define CTRL_SEND_CCSD          (1 << 15)       /*send command completion signal disable to ceata */
#define CTRL_SEND_AS_CCSD       (1 << 14)       /*send internally generated stop after sending ccsd */
#define CTRL_EXIT_MULTIPLE      (1 << 7)
#define CTRL_EXIT_TRANSFER      (1 << 6)
#define CTRL_START_READWAIT     (1 << 5)
#define CTRL_STOP_READWAIT      (1 << 4)
#define CTRL_RESET              (1 << 3)
#define CTRL_START_OP           (1 << 2)
#define CTRL_CLOCK_SHF          0
#define CTRL_CLOCK_MASK         (0x3 << CTRL_CLOCK_SHF)
#define CTRL_CLOCK_STOP         (0x1 << CTRL_CLOCK_SHF) /* Stop MMC/SD clock */
#define CTRL_CLOCK_START        (0x2 << CTRL_CLOCK_SHF) /* Start MMC/SD clock */

/* MSC Control 2 Register (MSC_CTRL2) */
#define CTRL2_PIP_SHF           24
#define CTRL2_PIP_MASK          (0x1f << CTRL2_PIP_SHF)
#define CTRL2_RST_EN            (1 << 23)
#define CTRL2_STPRM             (1 << 4)
#define CTRL2_SVC               (1 << 3)
#define CTRL2_SMS_SHF           0
#define CTRL2_SMS_MASK          (0x7 << CTRL2_SMS_SHF)
#define CTRL2_SMS_DEFSPD        (0x0 << CTRL2_SMS_SHF)
#define CTRL2_SMS_HISPD         (0x1 << CTRL2_SMS_SHF)
#define CTRL2_SMS_SDR12         (0x2 << CTRL2_SMS_SHF)
#define CTRL2_SMS_SDR25         (0x3 << CTRL2_SMS_SHF)
#define CTRL2_SMS_SDR50         (0x4 << CTRL2_SMS_SHF)

/* MSC Status Register (MSC_STAT) */
#define STAT_AUTO_CMD12_DONE            (1 << 31)
#define STAT_AUTO_CMD23_DONE            (1 << 30)
#define STAT_SVS                        (1 << 29)
#define STAT_PIN_LEVEL_SHF              24
#define STAT_PIN_LEVEL_MASK             (0x1f << STAT_PIN_LEVEL_SHF)
#define STAT_BCE                        (1 << 20)
#define STAT_BDE                        (1 << 19)
#define STAT_BAE                        (1 << 18)
#define STAT_BAR                        (1 << 17)
#define STAT_IS_RESETTING               (1 << 15)
#define STAT_SDIO_INT_ACTIVE            (1 << 14)
#define STAT_PRG_DONE                   (1 << 13)
#define STAT_DATA_TRAN_DONE             (1 << 12)
#define STAT_END_CMD_RES                (1 << 11)
#define STAT_DATA_FIFO_AFULL            (1 << 10)
#define STAT_IS_READWAIT                (1 << 9)
#define STAT_CLK_EN                     (1 << 8)
#define STAT_DATA_FIFO_FULL             (1 << 7)
#define STAT_DATA_FIFO_EMPTY            (1 << 6)
#define STAT_CRC_RES_ERR                (1 << 5)
#define STAT_CRC_READ_ERROR             (1 << 4)
#define STAT_CRC_WRITE_ERROR_SHF        2
#define STAT_CRC_WRITE_ERROR_MASK       (0x3 << STAT_CRC_WRITE_ERROR_SHF)
#define STAT_CRC_WRITE_ERROR_NO         (0 << STAT_CRC_WRITE_ERROR_SHF)
#define STAT_CRC_WRITE_ERROR            (1 << STAT_CRC_WRITE_ERROR_SHF)
#define STAT_CRC_WRITE_ERROR_NOSTS      (2 << STAT_CRC_WRITE_ERROR_SHF)
#define STAT_TIME_OUT_RES               (1 << 1)
#define STAT_TIME_OUT_READ              (1 << 0)

/* MSC Bus Clock Control Register (MSC_CLKRT) */
#define CLKRT_CLK_RATE_SHF      0
#define CLKRT_CLK_RATE_MASK     (0x7 << CLKRT_CLK_RATE_SHF)
#define CLKRT_CLK_RATE_DIV_1    (0x0 << CLKRT_CLK_RATE_SHF)     /* CLK_SRC */
#define CLKRT_CLK_RATE_DIV_2    (0x1 << CLKRT_CLK_RATE_SHF)     /* 1/2 of CLK_SRC */
#define CLKRT_CLK_RATE_DIV_4    (0x2 << CLKRT_CLK_RATE_SHF)     /* 1/4 of CLK_SRC */
#define CLKRT_CLK_RATE_DIV_8    (0x3 << CLKRT_CLK_RATE_SHF)     /* 1/8 of CLK_SRC */
#define CLKRT_CLK_RATE_DIV_16   (0x4 << CLKRT_CLK_RATE_SHF)     /* 1/16 of CLK_SRC */
#define CLKRT_CLK_RATE_DIV_32   (0x5 << CLKRT_CLK_RATE_SHF)     /* 1/32 of CLK_SRC */
#define CLKRT_CLK_RATE_DIV_64   (0x6 << CLKRT_CLK_RATE_SHF)     /* 1/64 of CLK_SRC */
#define CLKRT_CLK_RATE_DIV_128  (0x7 << CLKRT_CLK_RATE_SHF)     /* 1/128 of CLK_SRC */

/* MSC Command Sequence Control Register (MSC_CMDAT) */
#define CMDAT_CCS_EXPECTED      (1 << 31)       /* interrupts are enabled in ce-ata */
#define CMDAT_READ_CEATA        (1 << 30)
#define CMDAT_DIS_BOOT          (1 << 27)
#define CMDAT_ENB_BOOT          (1 << 26)
#define CMDAT_EXP_BOOT_ACK      (1 << 25)
#define CMDAT_BOOT_MODE         (1 << 24)
#define CMDAT_AUTO_CMD23        (1 << 18)
#define CMDAT_SDIO_PRDT         (1 << 17)       /* exact 2 cycle */
#define CMDAT_AUTO_CMD12        (1 << 16)
#define CMDAT_RTRG_SHF          14
#define CMDAT_RTRG_EQUALT_16    (0x0 << CMDAT_RTRG_SHF) /*reset value*/
#define CMDAT_RTRG_EQUALT_32    (0x1 << CMDAT_RTRG_SHF)
#define CMDAT_RTRG_EQUALT_64    (0x2 << CMDAT_RTRG_SHF)
#define CMDAT_RTRG_EQUALT_96    (0x3 << CMDAT_RTRG_SHF)
#define CMDAT_TTRG_SHF          12
#define CMDAT_TTRG_LESS_16      (0x0 << CMDAT_TTRG_SHF) /*reset value*/
#define CMDAT_TTRG_LESS_32      (0x1 << CMDAT_TTRG_SHF)
#define CMDAT_TTRG_LESS_64      (0x2 << CMDAT_TTRG_SHF)
#define CMDAT_TTRG_LESS_96      (0x3 << CMDAT_TTRG_SHF)
#define CMDAT_IO_ABORT          (1 << 11)
#define CMDAT_BUS_WIDTH_SHF     9
#define CMDAT_BUS_WIDTH_MASK    (0x3 << CMDAT_BUS_WIDTH_SHF)
#define CMDAT_BUS_WIDTH_1BIT    (0x0 << CMDAT_BUS_WIDTH_SHF)    /* 1-bit data bus */
#define CMDAT_BUS_WIDTH_4BIT    (0x2 << CMDAT_BUS_WIDTH_SHF)    /* 4-bit data bus */
#define CMDAT_BUS_WIDTH_8BIT    (0x3 << CMDAT_BUS_WIDTH_SHF)    /* 8-bit data bus */
#define CMDAT_INIT              (1 << 7)
#define CMDAT_BUSY              (1 << 6)
#define CMDAT_STREAM_BLOCK      (1 << 5)
#define CMDAT_WRITE_READ        (1 << 4)
#define CMDAT_DATA_EN           (1 << 3)
#define CMDAT_RESPONSE_SHF      0
#define CMDAT_RESPONSE_MASK     (0x7 << CMDAT_RESPONSE_SHF)
#define CMDAT_RESPONSE_NONE     (0x0 << CMDAT_RESPONSE_SHF)     /* No response */
#define CMDAT_RESPONSE_R1       (0x1 << CMDAT_RESPONSE_SHF)     /* Format R1 and R1b */
#define CMDAT_RESPONSE_R2       (0x2 << CMDAT_RESPONSE_SHF)     /* Format R2 */
#define CMDAT_RESPONSE_R3       (0x3 << CMDAT_RESPONSE_SHF)     /* Format R3 */
#define CMDAT_RESPONSE_R4       (0x4 << CMDAT_RESPONSE_SHF)     /* Format R4 */
#define CMDAT_RESPONSE_R5       (0x5 << CMDAT_RESPONSE_SHF)     /* Format R5 */
#define CMDAT_RESPONSE_R6       (0x6 << CMDAT_RESPONSE_SHF)     /* Format R6 */
#define CMDAT_RESRONSE_R7       (0x7 << CMDAT_RESPONSE_SHF)     /* Format R7 */

/* MSC Interrupts Mask Register (MSC_IMASK) */
#define IMASK_DMA_DATA_DONE     (1 << 31)
#define IMASK_WR_ALL_DONE       (1 << 23)
#define IMASK_AUTO_CMD23_DONE   (1 << 30)
#define IMASK_SVS               (1 << 29)
#define IMASK_PIN_LEVEL_SHF     24
#define IMASK_PIN_LEVEL_MASK    (0x1f << IMASK_PIN_LEVEL_SHF)
#define IMASK_BCE               (1 << 20)
#define IMASK_BDE               (1 << 19)
#define IMASK_BAE               (1 << 18)
#define IMASK_BAR               (1 << 17)
#define IMASK_DMAEND            (1 << 16)
#define IMASK_AUTO_CMD12_DONE   (1 << 15)
#define IMASK_DATA_FIFO_FULL    (1 << 14)
#define IMASK_DATA_FIFO_EMP     (1 << 13)
#define IMASK_CRC_RES_ERR       (1 << 12)
#define IMASK_CRC_READ_ERR      (1 << 11)
#define IMASK_CRC_WRITE_ERR     (1 << 10)
#define IMASK_TIME_OUT_RES      (1 << 9)
#define IMASK_TIME_OUT_READ     (1 << 8)
#define IMASK_SDIO              (1 << 7)
#define IMASK_TXFIFO_WR_REQ     (1 << 6)
#define IMASK_RXFIFO_RD_REQ     (1 << 5)
#define IMASK_END_CMD_RES       (1 << 2)
#define IMASK_PRG_DONE          (1 << 1)
#define IMASK_DATA_TRAN_DONE    (1 << 0)

/* MSC Interrupts Status Register (MSC_IREG) */
#define IFLG_DMA_DATA_DONE      (1 << 31)
#define IFLG_WR_ALL_DONE        (1 << 23)
#define IFLG_AUTO_CMD23_DONE    (1 << 30)
#define IFLG_SVS                (1 << 29)
#define IFLG_PIN_LEVEL_SHF      24
#define IFLG_PIN_LEVEL_MASK     (0x1f << IFLG_PIN_LEVEL_SHF)
#define IFLG_BCE                (1 << 20)
#define IFLG_BDE                (1 << 19)
#define IFLG_BAE                (1 << 18)
#define IFLG_BAR                (1 << 17)
#define IFLG_DMAEND             (1 << 16)
#define IFLG_AUTO_CMD12_DONE    (1 << 15)
#define IFLG_DATA_FIFO_FULL     (1 << 14)
#define IFLG_DATA_FIFO_EMP      (1 << 13)
#define IFLG_CRC_RES_ERR        (1 << 12)
#define IFLG_CRC_READ_ERR       (1 << 11)
#define IFLG_CRC_WRITE_ERR      (1 << 10)
#define IFLG_TIMEOUT_RES        (1 << 9)
#define IFLG_TIMEOUT_READ       (1 << 8)
#define IFLG_SDIO               (1 << 7)
#define IFLG_TXFIFO_WR_REQ      (1 << 6)
#define IFLG_RXFIFO_RD_REQ      (1 << 5)
#define IFLG_END_CMD_RES        (1 << 2)
#define IFLG_PRG_DONE           (1 << 1)
#define IFLG_DATA_TRAN_DONE     (1 << 0)

/* MSC Low Power Mode Register (MSC_LPM) */
#define LPM_DRV_SEL_SHF         30
#define LPM_DRV_SEL_MASK        (0x3 << LPM_DRV_SEL_SHF)
#define LPM_SMP_SEL             (1 << 29)
#define LPM_LPM                 (1 << 0)

/* MSC DMA Control Register (MSC_DMAC) */
#define DMAC_MODE_SEL   (1 << 7)
#define DMAC_AOFST_SHF  5
#define DMAC_AOFST_MASK (0x3 << DMAC_AOFST_SHF)
#define DMAC_AOFST_0    (0 << DMAC_AOFST_SHF)
#define DMAC_AOFST_1    (1 << DMAC_AOFST_SHF)
#define DMAC_AOFST_2    (2 << DMAC_AOFST_SHF)
#define DMAC_AOFST_3    (3 << DMAC_AOFST_SHF)
#define DMAC_ALIGNEN    (1 << 4)
#define DMAC_INCR_SHF   2
#define DMAC_INCR_MASK  (0x3 << DMAC_INCR_SHF)
#define DMAC_INCR_16    (0 << DMAC_INCR_SHF)
#define DMAC_INCR_32    (1 << DMAC_INCR_SHF)
#define DMAC_INCR_64    (2 << DMAC_INCR_SHF)
#define DMAC_DMASEL     (1 << 1)
#define DMAC_DMAEN      (1 << 0)

/* MSC DMA Command Register (MSC_DMACMD) */
#define DMACMD_IDI_SHF          24
#define DMACMD_IDI_MASK         (0xff << DMACMD_IDI_SHF)
#define DMACMD_ID_SHF           16
#define DMACMD_ID_MASK          (0xff << DMACMD_ID_SHF)
#define DMACMD_OFFSET_SHF       9
#define DMACMD_OFFSET_MASK      (0x3 << DMACMD_OFFSET_SHF)
#define DMACMD_ALIGN_EN         (1 << 8)
#define DMACMD_ENDI             (1 << 1)
#define DMACMD_LINK             (1 << 0)

#endif  /* __INGENIC_MMC_REG_H__ */
