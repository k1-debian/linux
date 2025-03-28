/*
 * linux/arch/mips/boot/compressed/misc.c
 *
 * This is a collection of several routines from gzip-1.0.3
 * adapted for Linux.
 *
 * malloc by Hannu Savolainen 1993 and Matthias Urlichs 1994
 *
 * Adapted for JZSOC by Peter Wei, 2008
 *
 */

#include <soc/base.h>
#include <asm/addrspace.h>
//#include <stdarg.h>

#define size_t  int
#define NULL 0

/*
 * gzip declarations
 */

#define OF(args)  args
#define STATIC static

#undef memset
#undef memcpy
#define memzero(s, n)     memset ((s), 0, (n))

typedef unsigned char  uch;
typedef unsigned short ush;
typedef unsigned long  ulg;

#define WSIZE 0x8000        /* Window size must be at least 32k, */
/* and a power of two */

static uch *inbuf;       /* input buffer */
static uch window[WSIZE];    /* Sliding window buffer */

static unsigned insize = 0;  /* valid bytes in inbuf */
static unsigned inptr = 0;   /* index of next byte to be processed in inbuf */
static unsigned outcnt = 0;  /* bytes in output buffer */

/* gzip flag byte */
#define ASCII_FLAG   0x01 /* bit 0 set: file probably ASCII text */
#define CONTINUATION 0x02 /* bit 1 set: continuation of multi-part gzip file */
#define EXTRA_FIELD  0x04 /* bit 2 set: extra field present */
#define ORIG_NAME    0x08 /* bit 3 set: original file name present */
#define COMMENT      0x10 /* bit 4 set: file comment present */
#define ENCRYPTED    0x20 /* bit 5 set: file is encrypted */
#define RESERVED     0xC0 /* bit 6,7:   reserved */

#define get_byte()  (inptr < insize ? inbuf[inptr++] : fill_inbuf())

/* Diagnostic functions */
#ifdef DEBUG
	#define Assert(cond,msg) {if(!(cond)) error(msg);}
	#define Trace(x) fprintf x
	#define Tracev(x) {if (verbose) fprintf x ;}
	#define Tracevv(x) {if (verbose>1) fprintf x ;}
	#define Tracec(c,x) {if (verbose && (c)) fprintf x ;}
	#define Tracecv(c,x) {if (verbose>1 && (c)) fprintf x ;}
#else
	#define Assert(cond,msg)
	#define Trace(x)
	#define Tracev(x)
	#define Tracevv(x)
	#define Tracec(c,x)
	#define Tracecv(c,x)
#endif

static int  fill_inbuf(void);
static void flush_window(void);
static void error(char *m);
#if 0
	static void gzip_mark(void **);
	static void gzip_release(void **);
#endif
void *memset(void *s, int c, size_t n);
void *memcpy(void *__dest, __const void *__src, size_t __n);

extern void flushcaches(void); /* defined in head.S */

char *input_data;
int input_len;

static long bytes_out = 0;
static uch *output_data;
static unsigned long output_ptr = 0;
static int is_no_uart = 1;

static void *malloc(int size);
static void free(void *where);
static void error(char *m);

#if 1

/*
 * Add puts func just for debug.
 * If it does not work,check the follows macro define.
 * Have a pleasant debug day.
 */

static volatile int uart_base;

#define OFF_LSR     (0x14)
#define OFF_LCR     (0x0C)
#define OFF_TDR     (0x00)
#define OFF_SPR     (0X1C)
#define UART_OFF    0x1000
#define UART_LSR_TDRQ   (1 << 5)        /* 1: transmit FIFO half "empty" */
#define UART_LSR_TEMT   (1 << 6)    /* 1: transmit FIFO and shift registers empty */

void check_uart(void)
{
	volatile char *base = (volatile char *)CKSEG1ADDR(UART0_IOBASE);
	int i;
	for (i = 0; i < 16; i++) {
		// if (base[OFF_SPR] == 0xa9) {
		if (base[OFF_LCR]) {
			is_no_uart = 0;
			break;
		}
		base += UART_OFF;
	}

	uart_base = (int)base;
}

static void serial_putc(const char c)
{
	volatile char *uart_lsr = (volatile char *)(uart_base + OFF_LSR);
	volatile char *uart_tdr = (volatile char *)(uart_base + OFF_TDR);

	if (is_no_uart) {
		return;
	}

	if (c == '\n') {
		serial_putc('\r');
	}

	/* Wait for fifo to shift out some bytes */
	while (!((*uart_lsr & (UART_LSR_TDRQ | UART_LSR_TEMT)) == 0x60));

	*uart_tdr = (char)c;
}
static void puts(const char *s)
{
	while (*s) {
		serial_putc(*s++);
	}
}

static int hex2asc(int n)
{
	n &= 15;
	if (n > 9) {
		return ('a' - 10) + n;
	} else {
		return '0' + n;
	}
}

//int printf(char *fmt,...)
//{
//   va_list ap;
//   char scratch[16];
//   va_start(ap,fmt);
//
//    for(;;){
//        switch(*fmt){
//        case 0:
//            va_end(ap);
//            return 0;
//        case '%':
//            switch(fmt[1]) {
//            case 'p':
//            case 'X':
//            case 'x': {
//                unsigned n = va_arg(ap, unsigned);
//                char *p = scratch + 15;
//                *p = 0;
//                do {
//                    *--p = hex2asc(n);
//                    n = n >> 4;
//                } while(n != 0);
//                while(p > (scratch + 7)) *--p = '0';
//      while (*p) serial_putc(*p++);
//                fmt += 2;
//                continue;
//            }
//            case 'd': {
//                int n = va_arg(ap, int);
//                char *p = scratch + 15;
//                *p = 0;
//                if(n < 0) {
//                    serial_putc('-');
//                    n = -n;
//                }
//                do {
//                    *--p = (n % 10) + '0';
//                    n /= 10;
//                } while(n != 0);
//      while (*p) serial_putc(*p++);
//                fmt += 2;
//                continue;
//            }
//            case 's': {
//                char *s = va_arg(ap, char*);
//                if(s == 0) s = "(null)";
//      while (*s) serial_putc(*s++);
//                fmt += 2;
//                continue;
//            }
//            }
//            serial_putc(*fmt++);
//            break;
//        case '\n':
//            serial_putc('\r');
//        default:
//            serial_putc(*fmt++);
//        }
//    }
//}


#else
static void puts(const char *str)
{
}
#endif

extern unsigned char _end[];
static unsigned long free_mem_ptr;
static unsigned long free_mem_end_ptr;

#define HEAP_SIZE             0x10000

#include "../../../../lib/inflate.c"

#if 0
static void *malloc(int size)
{
	void *p;

	if (size < 0) {
		error("Malloc error\n");
	}
	if (free_mem_ptr == 0) {
		error("Memory error\n");
	}

	free_mem_ptr = (free_mem_ptr + 3) & ~3; /* Align */

	p = (void *)free_mem_ptr;
	free_mem_ptr += size;

	if (free_mem_ptr >= free_mem_end_ptr) {
		error("\nOut of memory\n");
	}

	return p;
}

static void free(void *where)
{
	/* Don't care */
}


static void gzip_mark(void **ptr)
{
	*ptr = (void *) free_mem_ptr;
}

static void gzip_release(void **ptr)
{
	free_mem_ptr = (long) * ptr;
}
#endif
void *memset(void *s, int c, size_t n)
{
	int i;
	char *ss = (char *)s;

	for (i = 0; i < n; i++) {
		ss[i] = c;
	}
	return s;
}

void *memcpy(void *__dest, __const void *__src, size_t __n)
{
	int i = 0;
	unsigned char *d = (unsigned char *)__dest, *s = (unsigned char *)__src;

	for (i = __n >> 3; i > 0; i--) {
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
	}

	if (__n & 1 << 2) {
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
		*d++ = *s++;
	}

	if (__n & 1 << 1) {
		*d++ = *s++;
		*d++ = *s++;
	}

	if (__n & 1) {
		*d++ = *s++;
	}

	return __dest;
}

/* ===========================================================================
 * Fill the input buffer. This is called only when the buffer is empty
 * and at least one byte is really needed.
 */
static int fill_inbuf(void)
{
	if (insize != 0) {
		error("ran out of input data\n");
	}

	inbuf = input_data;
	insize = input_len;
	inptr = 1;
	return inbuf[0];
}

/* ===========================================================================
 * Write the output window window[0..outcnt-1] and update crc and bytes_out.
 * (Used for the decompressed data only.)
 */
static void flush_window(void)
{
	ulg c = crc;         /* temporary variable */
	unsigned n;
	uch *in, *out, ch;

	in = window;
	out = &output_data[output_ptr];
	for (n = 0; n < outcnt; n++) {
		ch = *out++ = *in++;
		c = crc_32_tab[((int)c ^ ch) & 0xff] ^ (c >> 8);
	}
	crc = c;
	bytes_out += (ulg)outcnt;
	output_ptr += (ulg)outcnt;
	outcnt = 0;
}

static void error(char *x)
{
	puts("\n\n");
	puts(x);
	puts("\n\n -- System halted");

	while (1);  /* Halt */
}

void decompress_kernel(unsigned int imageaddr, unsigned int imagesize, unsigned int loadaddr)
{
	input_data = (char *)imageaddr;
	input_len = imagesize;
	output_ptr = 0;
	output_data = (uch *)loadaddr;
	free_mem_ptr = (unsigned long)_end;
	free_mem_end_ptr = free_mem_ptr + HEAP_SIZE;

	check_uart();
	makecrc();
	puts("Uncompressing Linux...\n");
	gunzip();
	flushcaches();
	puts("Ok, booting the kernel.\n");
#if 0
	{
		int i;
		int *p = loadaddr;
		for (i = 0; i < 512; i++) {
			if (i % 4 == 0) {
				printf("\n");
			}
			printf("%x ", p[i]);
		}
		printf("\n");
	}
#endif
}
