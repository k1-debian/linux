#ifndef __RTC_H__
#define __RTC_H__

/*
 * RTC registers offset address definition
 */
#define RTC_RTCCR       (0x00)  /* rw, 32, 0x00000081 */
#define RTC_RTCSR       (0x04)  /* rw, 32, 0x???????? */
#define RTC_RTCSAR      (0x08)  /* rw, 32, 0x???????? */
#define RTC_RTCGR       (0x0c)  /* rw, 32, 0x0??????? */

#define RTC_HCR         (0x20)  /* rw, 32, 0x00000000 */
#define RTC_HWFCR       (0x24)  /* rw, 32, 0x0000???0 */
#define RTC_HRCR        (0x28)  /* rw, 32, 0x00000??0 */
#define RTC_HWCR        (0x2c)  /* rw, 32, 0x00000008 */
#define RTC_HWRSR       (0x30)  /* rw, 32, 0x00000000 */
#define RTC_HSPR        (0x34)  /* rw, 32, 0x???????? */
#define RTC_WENR        (0x3c)  /* rw, 32, 0x00000000 */
#define RTC_WKUPPINCR       (0x48)  /* rw, 32, 0x00050064*/

/*
 * RTC registers common define
 */

/* RTC control register(RTCCR) */
#define RTCCR_WRDY      BIT(7)
#define RTCCR_1HZ       BIT(6)
#define RTCCR_1HZIE     BIT(5)
#define RTCCR_AF        BIT(4)
#define RTCCR_AIE       BIT(3)
#define RTCCR_AE        BIT(2)
#define RTCCR_SELEXC        BIT(1)
#define RTCCR_RTCE      BIT(0)


/* Generate the bit field mask from msb to lsb */
#define BITS_H2L(msb, lsb)  ((0xFFFFFFFF >> (32-((msb)-(lsb)+1))) << (lsb))

/* RTC regulator register(RTCGR) */
#define RTCGR_LOCK      BIT(31)
#define RTCGR_ADJC_LSB      16
#define RTCGR_ADJC_MASK     BITS_H2L(25, RTCGR_ADJC_LSB)
#define RTCGR_NC1HZ_LSB     0
#define RTCGR_NC1HZ_MASK    BITS_H2L(15, RTCGR_NC1HZ_LSB)

/* Hibernate control register(HCR) */
#define HCR_PD          BIT(0)

/* Hibernate wakeup filter counter register(HWFCR) */
#define HWFCR_LSB       5
#define HWFCR_MASK      BITS_H2L(15, HWFCR_LSB)
#define HWFCR_WAIT_TIME(ms, clk) (((ms)*(clk)/1000+0xf) > HWFCR_MASK ? HWFCR_MASK : ((ms)*(clk)/1000+0xf) & HWFCR_MASK)

/* Hibernate reset counter register(HRCR) */
#define HRCR_LSB        11
#define HRCR_MASK       BITS_H2L(14, HRCR_LSB)
#define HRCR_WAIT_TIME(ms, clk) (((ms)*(clk)/1000 + 0x3ff - 0x800) > HRCR_MASK ? HRCR_MASK : \
                                 ((ms)*(clk)/1000 + 0x3ff - 0x800) & HRCR_MASK)


/* Hibernate wakeup control register(HWCR) */
/* Power detect default value; this value means enable */
#define EPDET_LSB       3
#define EPDET_DEFAULT           (0x5aa5a5a << EPDET_LSB)
#define EPDET_ENABLE        (0x5aa5a5a << EPDET_LSB)
#define EPDET_DISABLE       (0x1a55a5a5 << EPDET_LSB)
#define HWCR_EALM       BIT(0)


/* Hibernate wakeup status register(HWRSR) */
#define HWRSR_APD       BIT(8)
#define HWRSR_HR        BIT(5)
#define HWRSR_PPR       BIT(4)
#define HWRSR_PIN       BIT(1)
#define HWRSR_ALM       BIT(0)

/* write enable pattern register(WENR) */
#define WENR_WEN        BIT(31)
#define WENR_WENPAT_LSB     0
#define WENR_WENPAT_MASK    BITS_H2L(15, WENR_WENPAT_LSB)
#define WENR_WENPAT_WRITABLE    (0xa55a)

/* Hibernate scratch pattern register(HSPR) */
#define HSPR_RTCV               0x52544356      /* The value is 'RTCV', means rtc is valid */

/*WKUP_PIN_RST control register (WKUPPINCR)*/
#define WKUPPINCR_BIAS_CTRL (0x1 << 16) /* fix value*/
#define WKUPPINCR_OSC_EN        BIT(18)
#define WKUPPINCR_P_JUD_LEN_LSB 4
#define WKUPPINCR_P_JUD_LEN_MASK    BITS_H2L(7, WKUPPINCR_P_JUD_LEN_LSB)
#define WKUPPINCR_P_JUD_LEN(s)  ((s) << WKUPPINCR_P_JUD_LEN_LSB) > WKUPPINCR_P_JUD_LEN_MASK ? \
	WKUPPINCR_P_JUD_LEN_MASK : (s) << WKUPPINCR_P_JUD_LEN_LSB
#define WKUPPINCR_P_JUD_EN  (0x4)
/* The divider is decided by the RTC clock frequency. */
#define RTC_FREQ_DIVIDER    (32768 - 1)
#endif  /* __RTC_H__ */
