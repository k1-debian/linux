/*
 *
 * Copyright (C) 1992, 1994 by Theodore Ts'o.
 *
 * Redistribution of this file is permitted under the terms of the GNU
 * Public License (GPL)
 *
 * These are the UART port assignments, expressed as offsets from the base
 * register.  These assignments should hold for any serial port based on
 * a 8250, 16450, or 16550(A).
 */

#ifndef __INGENIC_UART_H__
#define __INGENIC_UART_H__

/*
    fix: some definitions in origin kernel conflicts.
    include/uapi/linux/serial_reg.h
*/
#ifdef UART_IIR_ID
	#undef UART_IIR_ID
#endif

#ifdef UART_MSR_ANY_DELTA
	#undef UART_MSR_ANY_DELTA
#endif

#ifdef UART_TCR
	#undef UART_TCR
#endif

/*
 * DLAB=0
 */
#define UART_RX     0   /* In:  Receive buffer */
#define UART_TX     0   /* Out: Transmit buffer */

#define UART_IER    1   /* Out: Interrupt Enable Register */
#define UART_IER_RETOIE         0x20 /* Receive Fifo Empty Timeout Interrupt Enable */
#define UART_IER_RTOIE      0x10 /* Receiver Time Out Interrupt Enable */
#define UART_IER_MSI        0x08 /* Enable Modem status interrupt */
#define UART_IER_RLSI       0x04 /* Enable receiver line status interrupt */
#define UART_IER_THRI       0x02 /* Enable Transmitter holding register int. */
#define UART_IER_RDI        0x01 /* Enable receiver data interrupt */

#define UART_IIR    2   /* In:  Interrupt ID Register */
#define UART_IIR_NO_INT     0x01 /* No interrupts pending */
#define UART_IIR_ID     0x06 /* Mask for the interrupt ID */
#define UART_IIR_MSI        0x00 /* Modem status interrupt */
#define UART_IIR_THRI       0x02 /* Transmitter holding register empty */
#define UART_IIR_RDI        0x04 /* Receiver data interrupt */
#define UART_IIR_RLSI       0x06 /* Receiver line status interrupt */

#define UART_IIR_BUSY       0x07 /* DesignWare APB Busy Detect */

#define UART_FCR    2   /* Out: FIFO Control Register */
#define UART_FCR_ENABLE_FIFO    0x01 /* Enable the FIFO */
#define UART_FCR_CLEAR_RCVR 0x02 /* Clear the RCVR FIFO */
#define UART_FCR_CLEAR_XMIT 0x04 /* Clear the XMIT FIFO */
#define UART_FCR_DMA_SELECT 0x08 /* For DMA applications */
#define UART_FCR_UME        0x10 /* UME */

#define UART_FCR_R_TRIG_00  0x00
#define UART_FCR_R_TRIG_01  0x40
#define UART_FCR_R_TRIG_10  0x80
#define UART_FCR_R_TRIG_11  0xc0
#define UART_FCR_TRIGGER_MASK   0xC0 /* Mask for the FIFO trigger range */

#define UART_LCR    3   /* Out: Line Control Register */
/*
 * Note: if the word length is 5 bits (UART_LCR_WLEN5), then setting
 * UART_LCR_STOP will select 1.5 stop bits, not 2 stop bits.
 */
#define UART_LCR_DLAB       0x80 /* Divisor latch access bit */
#define UART_LCR_SBC        0x40 /* Set break control */
#define UART_LCR_SPAR       0x20 /* Stick parity (?) */
#define UART_LCR_EPAR       0x10 /* Even parity select */
#define UART_LCR_PARITY     0x08 /* Parity Enable */
#define UART_LCR_STOP       0x04 /* Stop bits: 0=1 bit, 1=2 bits */
#define UART_LCR_WLEN5      0x00 /* Wordlength: 5 bits */
#define UART_LCR_WLEN6      0x01 /* Wordlength: 6 bits */
#define UART_LCR_WLEN7      0x02 /* Wordlength: 7 bits */
#define UART_LCR_WLEN8      0x03 /* Wordlength: 8 bits */

/*
 * Access to some registers depends on register access / configuration
 * mode.
 */
#define UART_LCR_CONF_MODE_A    UART_LCR_DLAB   /* Configutation mode A */
#define UART_LCR_CONF_MODE_B    0xBF        /* Configutation mode B */

#define UART_MCR    4   /* Out: Modem Control Register */
#define UART_MCR_MDCE       0x80 /* Enable modem function */
#define UART_MCR_FCM        0x40 /* flow control by hardware */
#define UART_MCR_LOOP       0x10 /* Enable loopback test mode */
#define UART_MCR_RTS        0x02 /* RTS complement */

#define UART_LSR    5   /* In:  Line Status Register */
#define UART_LSR_FIFOE      0x80 /* Fifo error */
#define UART_LSR_TEMT       0x40 /* Transmitter empty */
#define UART_LSR_THRE       0x20 /* Transmit-hold-register empty */
#define UART_LSR_BI     0x10 /* Break interrupt indicator */
#define UART_LSR_FE     0x08 /* Frame error indicator */
#define UART_LSR_PE     0x04 /* Parity error indicator */
#define UART_LSR_OE     0x02 /* Overrun error indicator */
#define UART_LSR_DR     0x01 /* Receiver data ready */
#define UART_LSR_BRK_ERROR_BITS (UART_LSR_BI|UART_LSR_FE|UART_LSR_PE|UART_LSR_OE) /* BI, FE, PE, OE bits */

#define UART_MSR    6   /* In:  Modem Status Register */
#define UART_MSR_CTS        0x10 /* Clear to Send */
#define UART_MSR_DCTS       0x01 /* Delta CTS */
#define UART_MSR_ANY_DELTA  0x0F /* Any of the delta bits! */
#define UART_RCR   16
#define UART_TCR   17
#define UART_URFCR 18

#define UART_SCR    7   /* I/O: Scratch Register */

/*
 * The Ingenic xburst on-chip UARTs define these bits
 */

#define UART_ISR    8
#define UART_ISR_MASK            0x0f
#define UART_ISR_IID_MODEM       0x00
#define UART_ISR_IID_TRAN        0x02
#define UART_ISR_IID_REC         0x04
#define UART_ISR_IID_ERR         0x06
#define UART_ISR_IID_RETO        0x0a
#define UART_ISR_IID_TIME        0x0c

#define UART_UMR    9
#define UART_UACR   10
#define UART_RCR    16 /* RX FIFO Counter Register */
#define UART_TCR    17 /* TX FIFO Counter Register */

/*
 * DLAB=1
 */
#define UART_DLL    0   /* Out: Divisor Latch Low */
#define UART_DLM    1   /* Out: Divisor Latch High */

#define DESC_NUM 10
#define DESC_SIZE 32
#endif /* __INGENIC_UART_H__ */
