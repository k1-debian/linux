/*
 * JZSOC GPIO port, usually used in arch code.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZSOC_JZ_GPIO_H__
#define __JZSOC_JZ_GPIO_H__

enum gpio_function {
	GPIO_FUNC_0	 = 0x10,  //0000, GPIO as function 0 / device 0
	GPIO_FUNC_1	 = 0x11,  //0001, GPIO as function 1 / device 1
	GPIO_FUNC_2	 = 0x12,  //0010, GPIO as function 2 / device 2
	GPIO_FUNC_3	 = 0x13,  //0011, GPIO as function 3 / device 3
	GPIO_OUTPUT0 = 0x14,  //0100, GPIO output low  level
	GPIO_OUTPUT1 = 0x15,  //0101, GPIO output high level
	GPIO_INPUT	 = 0x16,  //0110, GPIO as input
	GPIO_INT_LO	 = 0x18,  //1000, Low  Level trigger interrupt
	GPIO_INT_HI	 = 0x19,  //1001, High Level trigger interrupt
	GPIO_INT_FE	 = 0x1a,  //1010, Fall Edge trigger interrupt
	GPIO_INT_RE	 = 0x1b,  //1011, Rise Edge trigger interrupt
	GPIO_INT_MASK_LO   = 0x1c,  //1100, Port is low level triggered interrupt input. Interrupt is masked.
	GPIO_INT_MASK_HI   = 0x1d,  //1101, Port is high level triggered interrupt input. Interrupt is masked.
	GPIO_INT_MASK_FE   = 0x1e,  //1110, Port is fall edge triggered interrupt input. Interrupt is masked.
	GPIO_INT_MASK_RE   = 0x1f,  //1111, Port is rise edge triggered interrupt input. Interrupt is masked.
	GPIO_PULL_FUNC = 0x80,
	GPIO_PULL_HIZ  = 0x80,  // no pull 
	GPIO_PULL_ENABLE = 0xa0,  // enable pull
	GPIO_PULL = GPIO_PULL_ENABLE,
	GPIO_INPUT_PULL = GPIO_INPUT | GPIO_PULL_ENABLE, // gpio input and pull
};
#define GPIO_AS_FUNC(func)  (((func) & 0x10) && (! ((func) & 0xc)))

enum gpio_port {
	GPIO_PORT_A, GPIO_PORT_B,
	GPIO_PORT_C, GPIO_PORT_D,
	/* this must be last */
	GPIO_NR_PORTS,
};

#define GPIO_PA(n)      (0 * 32 + (n))
#define GPIO_PB(n)      (1 * 32 + (n))
#define GPIO_PC(n)      (2 * 32 + (n))
#define GPIO_PD(n)      (3 * 32 + (n))
#define GPIO_PE(n)      (4 * 32 + (n))

/*
 * This functions are used in special driver which need
 * operate the device IO pin.
 */
int jzgpio_set_func(enum gpio_port port,
		    enum gpio_function func,unsigned long pins);

#endif
