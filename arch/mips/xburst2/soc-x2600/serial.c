/*
 * JZ SOC serial routines for early_printk.
 *
 * Copyright (C) 2006 Ingenic Semiconductor Inc.
 *
 */

#include <asm/io.h>

#include <soc/base.h>

#define UART_BASE   UART0_IOBASE
#define UART_OFF    0x1000

#define OFF_TDR     (0x00)
#define OFF_LCR     (0x0C)
#define OFF_LSR     (0x14)

#define LSR_TDRQ    (1 << 5)
#define LSR_TEMT    (1 << 6)

static void check_uart(char c);

static volatile u8 *uart_base;
typedef void (*putchar_f_t)(char);
static putchar_f_t putchar_f = check_uart;

static void putchar(char ch)
{
	int timeout = 10000;
	volatile u8 *base = uart_base;
	/* Wait for fifo to shift out some bytes */
	while ((base[OFF_LSR] & (LSR_TDRQ | LSR_TEMT))
	       != (LSR_TDRQ | LSR_TEMT) && timeout--)
		;
	base[OFF_TDR] = (u8)ch;
}

static void putchar_dummy(char ch)
{
	return;
}

static void check_uart(char c)
{
	/* We Couldn't use ioremap() here */
	volatile u8 *base = (volatile u8 *)CKSEG1ADDR(UART0_IOBASE);
	int i = 0;
	for (i = 0; i < 4; i++) {
		if (base[OFF_LCR]) {
			break;
		}
		base += UART_OFF;
	}

	if (i < 4) {
		uart_base = base;
		putchar_f = putchar;
		putchar_f(c);
	} else {
		putchar_f = putchar_dummy;
	}
}

/* used by early printk */
void prom_putchar(char c)
{
	putchar_f(c);
}

void prom_putstr(char *s)
{
	while (*s) {
		if (*s == '\n') {
			putchar_f('\r');
		}
		putchar_f(*s);
		s++;
	}
}

#if 1
static char pbuffer[4096];
void prom_printk(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vsnprintf(pbuffer, 4096, fmt, args);
	va_end(args);

	prom_putstr(pbuffer);
}
#endif
