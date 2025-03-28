/*
 * JZSOC CPM register definition.
 *
 * CPM (Clock reset and Power control Management)
 *
 * Copyright (C) 2019 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __CPM_H__
#define __CPM_H__
#include <soc/base.h>

#define CPM_CPCCR       (0x00)
#define CPM_CPPCR       (0x0c)
#define CPM_CPAPCR      (0x10)
#define CPM_CPMPCR      (0x14)
#define CPM_CPAPACR     (0x18)
#define CPM_CPMPACR     (0x1c)
#define CPM_DDRCDR      (0x2c)
#define CPM_EL150CDR    (0x30)
#define CPM_CPPSR       (0x34)
#define CPM_CPSPPR      (0x38)
#define CPM_USBPCR      (0x3c)
#define CPM_USBRDT      (0x40)
#define CPM_USBVBFIL    (0x44)
#define CPM_USBPCR1     (0x48)
#define CPM_RSACDR      (0x4c)
#define CPM_MACCDR      (0x54)
#define CPM_CPEPCR      (0x58)
#define CPM_CPEPACR     (0x5c)
#define CPM_SFCCDR      (0x60)
#define CPM_LPCDR       (0x64)
#define CPM_MSC0CDR     (0x68)
#define CPM_MSC1CDR     (0x6c)
#define CPM_I2STCDR     (0x70)
#define CPM_I2STCDR1    (0x78)
#define CPM_SSICDR      (0x74)
#define CPM_ISPCDR      (0x80)
#define CPM_I2SRCDR     (0x84)
#define CPM_I2SRCDR1    (0x88)
#define CPM_BSCALERCDR  (0xa0)
#define CPM_EXCLKDS     (0x8c)
#define CPM_CIM0CDR     (0x90)
#define CPM_CIM1CDR     (0x94)
#define CPM_CIM2CDR     (0x98)
#define CPM_SOFTAPP     (0x9c)
#define CPM_BSCALERCDR  (0xa0)
#define CPM_RADIXCDR    (0xa4)
#define CPM_INTR        (0xb0)
#define CPM_INTRE       (0xb4)
#define CPM_BT0CDR      (0xb8)
#define CPM_DRCG        (0xd0)
#define CPM_CPCSR       (0xd4)
#define CPM_CPVPCR      (0xe0)
#define CPM_CPVPACR     (0xe4)
#define CPM_MACPHY      (0xe8)


#define CPM_LCR         (0x04)
#define CPM_CLKGR       (0x20)/* def changed*/
#define CPM_OPCR        (0x24)
#define CPM_CLKGR1      (0x28)/* def changed*/
#define CPM_SRBC        (0xc4)
#define SRBC_USB_SR BIT (12)
#define CPM_MESTSEL     (0xec)

#define CPM_MEMCTRL_MA0     (0xf0)
#define CPM_MEMCTRL_MA1     (0xf4)
#define CPM_MEMCTRL_MA2     (0xf8)

#define CPM_RSR     (0x08)

#ifndef BIT
	#define BIT(nr)  (1UL << nr)
#endif

/*USB Parameter Control Register*/
#define USBPCR_USB_MODE                 BIT(31)
#define USBPCR_AVLD_REG                 BIT(30)
#define USBPCR_IDPULLUP_MASK_BIT        28
#define USBPCR_IDPULLUP_MASK            (0x3 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_IDPULLUP_OTG             (0x0 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_IDPULLUP_ALWAYS_SUSPEND  (0x1 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_IDPULLUP_ALWAYS          (0x2 << USBPCR_IDPULLUP_MASK_BIT)
#define USBPCR_INCR_MASK                BIT(27)
#define USBPCR_POR_BIT                  22
#define USBPCR_POR                      BIT(USBPCR_POR_BIT)

/*USB Reset Detect Timer Register*/
#define USBRDT_RESUME_INTEEN            BIT(31) /*RW*/
#define USBRDT_RESUME_INTERCLR          BIT(30) /*W0*/
#define USBRDT_RESUME_SPEED_BIT         28  /*RW*/
#define USBRDT_RESUME_SPEED_MSK         (0x3 << USBRDT_RESUME_SPEED_BIT)
#define USBRDT_RESUME_SPEED_HIGH        (0x0 << USBRDT_RESUME_SPEED_BIT)
#define USBRDT_RESUME_SPEED_FULL        (0x1 << USBRDT_RESUME_SPEED_BIT)
#define USBRDT_RESUME_SPEED_LOW         (0x2 << USBRDT_RESUME_SPEED_BIT)
#define USBRDT_RESUME_STATUS            BIT(27) /*RO*/
#define USBRDT_HB_MASK                  BIT(26)
#define USBRDT_VBFIL_LD_EN              BIT(25)
#define USBRDT_IDDIG_EN                 BIT(1)
#define USBRDT_IDDIG_REG                BIT(0)
#define USBRDT_USBRDT_MSK               (0x7fffff)
#define USBRDT_USBRDT(x)                ((x) & USBRDT_USBRDT_MSK)
#define USBRDT_UTMI_RST                 BIT(27)

/*USB VBUS Jitter Filter Register*/
#define USBVBFIL_USBVBFIL(x)        ((x) & 0xffff)
#define USBVBFIL_IDDIGFIL(x)        ((x) & (0xffff << 16))

/*USB Parameter Control Register1*/
#define USBPCR1_BVLD_REG    BIT(31)
#define USBPCR1_DPPULLDOWN  BIT(29)
#define USBPCR1_DMPULLDOWN  BIT(28)
#define USBPCR1_PORT_RST    BIT(21)

/*Oscillator and Power Control Register*/
#define OPCR_USB_SPENDN     BIT(7)
#define OPCR_USB_PHY_GATE   BIT(23)

#define LCR_LPM_MASK        (0x3)
#define LCR_LPM_SLEEP       (0x1)

#define CPM_LCR_PD_X2D      (0x1<<31)
#define CPM_LCR_PD_VPU      (0x1<<30)
#define CPM_LCR_PD_MASK     (0x3<<30)
#define CPM_LCR_X2DS        (0x1<<27)
#define CPM_LCR_VPUS        (0x1<<26)
#define CPM_LCR_STATUS_MASK (0x3<<26)

#define OPCR_ERCS       (0x1<<2)
#define OPCR_PD         (0x1<<3) //T31 delete
#define OPCR_IDLE       (0x1<<31)

#define CLKGR1_VPU              (0x1<<0)

#define cpm_inl(off)        inl(CPM_IOBASE + (off))
#define cpm_outl(val,off)   outl(val,CPM_IOBASE + (off))
#define cpm_clear_bit(val,off)  do{cpm_outl((cpm_inl(off) & ~(1 << (val))),off);}while(0)
#define cpm_set_bit(val,off)    do{cpm_outl((cpm_inl(off) | (1 << (val))),off);}while(0)
#define cpm_test_bit(val,off)   (cpm_inl(off) & (0x1 << (val)))

#endif
/* __CPM_H__ */
