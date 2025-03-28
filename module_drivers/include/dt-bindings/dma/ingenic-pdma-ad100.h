#ifndef __INGENIC_PDMA_AD100_H__
#define __INGENIC_PDMA_AD100_H__
//#include<generated/autoconf.h>

/* #define INGENIC_DMA_REQ_DMIC_RX      0x5 */
/* #define INGENIC_DMA_REQ_I2S_TX       0x6 */
/* #define INGENIC_DMA_REQ_I2S_RX       0x7 */
/* #define INGENIC_DMA_REQ_SADC_RX      0x9 */

/* #define INGENIC_DMA_REQ_MSC1_TX      0x1c */
/* #define INGENIC_DMA_REQ_MSC1_RX      0x1d */
/* #define INGENIC_DMA_REQ_MSC2_TX      0x1e */
/* #define INGENIC_DMA_REQ_MSC2_RX      0x1f */

/* #define INGENIC_DMA_REQ_I2C4_TX      0x2c */
/* #define INGENIC_DMA_REQ_I2C4_RX      0x2d */
/* #define INGENIC_DMA_REQ_DES_TX       0x2e */
/* #define INGENIC_DMA_REQ_DES_RX       0x2f */

#define INGENIC_DMA_REQ_AUTO_TX     0x8
// AD100 PDMA TYPE
#define INGENIC_DMA_REQ_PCM_TX      0X6
#define INGENIC_DMA_REQ_PCM_RX      0X7
#define INGENIC_DMA_REQ_UART5_TX    0xa
#define INGENIC_DMA_REQ_UART5_RX    0xb
#define INGENIC_DMA_REQ_UART4_TX    0xc
#define INGENIC_DMA_REQ_UART4_RX    0xd
#define INGENIC_DMA_REQ_UART3_TX    0xe
#define INGENIC_DMA_REQ_UART3_RX    0xf
#define INGENIC_DMA_REQ_UART2_TX    0x10
#define INGENIC_DMA_REQ_UART2_RX    0x11
#define INGENIC_DMA_REQ_UART1_TX    0x12
#define INGENIC_DMA_REQ_UART1_RX    0x13
#define INGENIC_DMA_REQ_UART0_TX    0x14
#define INGENIC_DMA_REQ_UART0_RX    0x15
#define INGENIC_DMA_REQ_SSI0_TX     0x16
#define INGENIC_DMA_REQ_SSI0_RX     0x17
#define INGENIC_DMA_REQ_SSI1_TX     0x18
#define INGENIC_DMA_REQ_SSI1_RX     0x19
#define INGENIC_DMA_REQ_SLV0_TX     0x1a
#define INGENIC_DMA_REQ_SLV0_RX     0x1b

#define INGENIC_DMA_REQ_CAN0_TX     0x20
#define INGENIC_DMA_REQ_CAN0_RX     0x21
#define INGENIC_DMA_REQ_CAN1_TX     0x22
#define INGENIC_DMA_REQ_CAN1_RX     0x23
#define INGENIC_DMA_REQ_I2C0_TX     0x24
#define INGENIC_DMA_REQ_I2C0_RX     0x25
#define INGENIC_DMA_REQ_I2C1_TX     0x26
#define INGENIC_DMA_REQ_I2C1_RX     0x27
#define INGENIC_DMA_REQ_I2C2_TX     0x28
#define INGENIC_DMA_REQ_I2C2_RX     0x29
#define INGENIC_DMA_REQ_I2C3_TX     0x2a
#define INGENIC_DMA_REQ_I2C3_RX     0x2b

#define INGENIC_DMA_REQ_UART6_TX    0x30
#define INGENIC_DMA_REQ_UART6_RX    0x31
#define INGENIC_DMA_REQ_UART7_TX    0x32
#define INGENIC_DMA_REQ_UART7_RX    0x33

#define INGENIC_DMA_REQ_AIC_LOOP_RX 0x3d
#define INGENIC_DMA_REQ_AIC_TX      0x3e
#define INGENIC_DMA_REQ_AIC_RX      0x3f

/* define for AD100 PDMA_MCU*/
#define INGENIC_DMA_REQ_PWM0_TX         0x2c
#define INGENIC_DMA_REQ_PWM1_TX         0x2d
#define INGENIC_DMA_REQ_PWM2_TX         0x2e
#define INGENIC_DMA_REQ_PWM3_TX         0x2f
#define INGENIC_DMA_REQ_PWM4_TX         0x30
#define INGENIC_DMA_REQ_PWM5_TX         0x31
#define INGENIC_DMA_REQ_PWM6_TX         0x32
#define INGENIC_DMA_REQ_PWM7_TX         0x33
#define INGENIC_DMA_REQ_PWM8_TX         0x34
#define INGENIC_DMA_REQ_PWM9_TX         0x35
#define INGENIC_DMA_REQ_PWM10_TX         0x36
#define INGENIC_DMA_REQ_PWM11_TX         0x37
#define INGENIC_DMA_REQ_PWM12_TX         0x38
#define INGENIC_DMA_REQ_PWM13_TX         0x39
#define INGENIC_DMA_REQ_PWM14_TX         0x3a
#define INGENIC_DMA_REQ_PWM15_TX         0x3b
#define INGENIC_DMA_REQ_SADC_SEQ1_RX    0x00
#define INGENIC_DMA_REQ_SADC_SEQ2_RX    0x01

#endif  /* __INGENIC_PDMA_H__ */
