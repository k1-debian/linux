#ifndef _DT_BINDINGS_INTERRUPT_CONTROLLER_X2000_IRQ_H
#define _DT_BINDINGS_INTERRUPT_CONTROLLER_X2000_IRQ_H

#include <dt-bindings/interrupt-controller/mips-irq.h>

#define IRQ_AUDIO           (0)
#define IRQ_OTG             (1)
#define IRQ_RESERVED0_2         (2)
#define IRQ_PDMA            (3)
#define IRQ_PDMAD           (4)
#define IRQ_PDMAM           (5)
#define IRQ_PWM             (6)
#define IRQ_SFC             (7)
#define IRQ_SSI1            (8)
#define IRQ_SSI0            (9)
#define IRQ_MIPI_DSI            (10)
#define IRQ_SADC            (11)
#define IRQ_MIPI_CSI_4          (12)
#define IRQ_GPIO4           (13)
#define IRQ_GPIO3           (14)
#define IRQ_GPIO2           (15)
#define IRQ_GPIO1           (16)
#define IRQ_GPIO0           (17)
#define IRQ_VIC1            (18)
#define IRQ_VIC0            (19)
#define IRQ_ISP1            (20)
#define IRQ_ISP0            (21)
#define IRQ_HASH            (22)
#define IRQ_AES             (23)
#define IRQ_RSA             (24)
#define IRQ_TCU2            (25)
#define IRQ_TCU1            (26)
#define IRQ_TCU0            (27)
#define IRQ_MIPI_CSI2           (28)
#define IRQ_ROTATE          (29)
#define IRQ_CIM             (30)
#define IRQ_LCD             (31)

#define IRQ_RTC             (32 + 0)
#define IRQ_SOFT            (32 + 1)
#define IRQ_DTRNG           (32 + 2)
#define IRQ_SCC             (32 + 3)
#define IRQ_MSC1            (32 + 4)
#define IRQ_MSC0            (32 + 5)
#define IRQ_UART9           (32 + 6)
#define IRQ_UART8           (32 + 7)
#define IRQ_UART7           (32 + 8)
#define IRQ_UART6           (32 + 9)
#define IRQ_UART5           (32 + 10)
#define IRQ_UART4           (32 + 11)
#define IRQ_UART3           (32 + 12)
#define IRQ_UART2           (32 + 13)
#define IRQ_UART1           (32 + 14)
#define IRQ_UART0           (32 + 15)
#define IRQ_MSC2            (32 + 16)
#define IRQ_HARB2           (32 + 17)
#define IRQ_HARB0           (32 + 18)
#define IRQ_CPM             (32 + 19)
#define IRQ_DDR             (32 + 20)
#define IRQ_GMAC1           (32 + 21)
#define IRQ_EFUSE           (32 + 22)
#define IRQ_GMAC0           (32 + 23)
#define IRQ_I2C5            (32 + 24)
#define IRQ_I2C4            (32 + 25)
#define IRQ_I2C3            (32 + 26)
#define IRQ_I2C2            (32 + 27)
#define IRQ_I2C1            (32 + 28)
#define IRQ_I2C0            (32 + 29)
#define IRQ_HELIX           (32 + 30)
#define IRQ_FELIX           (32 + 31)

#endif
