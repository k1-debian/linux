#include <linux/slab.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/syscore_ops.h>
#include <linux/clk.h>

#include <dt-bindings/clock/ingenic-x2600.h>

#include "clk.h"

/*******************************************************************************
 *      FIXED CLK
 ********************************************************************************/
static struct ingenic_fixed_rate_clock x2600_fixed_rate_ext_clks[] __initdata = {
	FRATE(CLK_EXT, "ext", NULL, 0, 24000000),
	FRATE(CLK_RTC_EXT, "rtc_ext", NULL, 0, 32768),
	FRATE(CLK_EXT_DIV_512, "ext_div_512", NULL, 0, 46875),
};

/*******************************************************************************
 *      PLL
 ********************************************************************************/

static struct ingenic_pll_rate_table x2600_pll_rate_table[] = {
	PLL_RATE(1800000000, 75, 1, 1, 1),
	PLL_RATE(1600000000, 200, 3, 1, 1),
	PLL_RATE(1500000000, 125, 2, 1, 1),
	PLL_RATE(1404000000, 117, 2, 1, 1),
	PLL_RATE(1400000000, 175, 3, 1, 1),
	PLL_RATE(1392000000, 58, 1, 1, 1),
	PLL_RATE(1296000000, 54, 1, 1, 1),
	PLL_RATE(1200000000, 50, 1, 1, 1),
	PLL_RATE(1176000000, 49, 1, 1, 1),
	PLL_RATE(1148000000, 287, 3, 1, 2),
	PLL_RATE(1134000000, 189, 2, 1, 2),
	PLL_RATE(1120000000, 140, 3, 1, 1),
	PLL_RATE(1092000000, 91, 1, 1, 2),
	PLL_RATE(1064000000, 133, 3, 1, 1),
	PLL_RATE(1050000000, 175, 2, 1, 2),
	PLL_RATE(1036000000, 259, 3, 1, 2),
	PLL_RATE(1008000000, 42, 1, 1, 1),
	PLL_RATE(1000000000, 125, 3, 1, 1),
	PLL_RATE(980000000, 245, 3, 1, 2),
	PLL_RATE(966000000, 161, 2, 1, 2),
	PLL_RATE(952000000, 119, 3, 1, 1),
	PLL_RATE(924000000, 77, 1, 1, 2),
	PLL_RATE(900000000, 75, 1, 1, 2),
	PLL_RATE(896000000, 112, 3, 1, 1),
	PLL_RATE(882000000, 147, 2, 1, 2),
	PLL_RATE(868000000, 217, 3, 1, 2),
	PLL_RATE(840000000, 35, 1, 1, 1),
	PLL_RATE(812000000, 203, 3, 1, 2),
	PLL_RATE(800000000, 100, 1, 1, 3),
	PLL_RATE(798000000, 133, 2, 1, 2),
	PLL_RATE(784000000, 98, 1, 1, 3),
	PLL_RATE(756000000, 63, 1, 1, 2),
	PLL_RATE(728000000, 91, 1, 1, 3),
	PLL_RATE(714000000, 119, 2, 1, 2),
	PLL_RATE(700000000, 175, 2, 1, 3),
	PLL_RATE(693000000, 231, 4, 1, 2),
	PLL_RATE(672000000, 28, 1, 1, 1),
	PLL_RATE(672000000, 28, 1, 1, 1),
	PLL_RATE(651000000, 217, 4, 1, 2),
	PLL_RATE(630000000, 105, 2, 1, 2),
	PLL_RATE(609000000, 203, 4, 1, 2),
	PLL_RATE(588000000, 49, 1, 1, 2),
	PLL_RATE(567000000, 189, 2, 1, 4),
	PLL_RATE(546000000, 91, 1, 1, 4),
	PLL_RATE(532000000, 133, 2, 1, 3),
	PLL_RATE(525000000, 175, 2, 1, 4),
	PLL_RATE(518000000, 259, 3, 1, 4),
	PLL_RATE(504000000, 42, 1, 1, 2),
	PLL_RATE(490000000, 245, 3, 1, 4),
	PLL_RATE(483000000, 161, 2, 1, 4),
	PLL_RATE(476000000, 119, 2, 1, 3),
	PLL_RATE(462000000, 77, 1, 1, 4),
	PLL_RATE(448000000, 56, 1, 1, 3),
	PLL_RATE(441000000, 147, 2, 1, 4),
	PLL_RATE(434000000, 217, 3, 1, 4),
	PLL_RATE(420000000, 35, 1, 1, 2),
};

/*PLL HWDESC*/
static struct ingenic_pll_hwdesc apll_hwdesc = \
        PLL_DESC(CPM_CPAPCR, 20, 12, 14, 6, 11, 3, 8, 3, 3, 0);

static struct ingenic_pll_hwdesc mpll_hwdesc = \
        PLL_DESC(CPM_CPMPCR, 20, 12, 14, 6, 11, 3, 8, 3, 3, 0);

static struct ingenic_pll_hwdesc epll_hwdesc = \
        PLL_DESC(CPM_CPEPCR, 20, 12, 14, 6, 11, 3, 8, 3, 3, 0);

static struct ingenic_pll_clock x2600_pll_clks[] __initdata = {
	PLL(CLK_PLL_APLL, "apll", "ext", &apll_hwdesc, x2600_pll_rate_table),
	PLL(CLK_PLL_MPLL, "mpll", "ext", &mpll_hwdesc, x2600_pll_rate_table),
	PLL(CLK_PLL_EPLL, "epll", "ext", &epll_hwdesc, x2600_pll_rate_table),
};

/*******************************************************************************
 *      MUX
 ********************************************************************************/
PNAME(mux_table_0)  = {"stop",  "ext",      "apll"};
PNAME(mux_table_1)  = {"stop",  "sclka",    "mpll"};
PNAME(mux_table_2)  = {"sclka", "mpll",     "epll"};
PNAME(mux_table_3)  = {"sclka",     "mpll",     "epll",     "ext"};
PNAME(mux_table_4)  = {"sclka",     "epll"};
PNAME(mux_table_5)  = {"stop",  "ext"};
PNAME(mux_table_6)  = {"sclka", "mpll",     "ext"};
PNAME(mux_table_7)  = {"ext_div_512",   "rtc_ext"};

static unsigned int ingenic_mux_table[] = {0, 1, 2, 3, 4};

static struct ingenic_mux_clock x2600_mux_clks[] __initdata = {
	MUX(CLK_MUX_SCLKA,  "sclka",        ingenic_mux_table, mux_table_0, CPM_CPCCR,  30, 2, 0),
	MUX(CLK_MUX_CPU_L2C,    "mux_cpu_l2c",      ingenic_mux_table, mux_table_1, CPM_CPCCR,  28, 2, 0),
	MUX(CLK_MUX_AHB0,   "mux_ahb0",     ingenic_mux_table, mux_table_1, CPM_CPCCR,  26, 2, 0),
	MUX(CLK_MUX_AHB2,   "mux_ahb2",     ingenic_mux_table, mux_table_1, CPM_CPCCR,  24, 2, 0),

	MUX(CLK_MUX_DDR,    "mux_ddr",      ingenic_mux_table, mux_table_1, CPM_DDRCDR, 30, 2, 0),
	MUX(CLK_MUX_MACPHY, "mux_macphy",       ingenic_mux_table, mux_table_2, CPM_MACPHYCDR,  30, 2, 0),
	MUX(CLK_MUX_I2SCS,  "mux_i2scs",        ingenic_mux_table, mux_table_2, CPM_I2SCDR, 30, 2, 0),
	MUX(CLK_MUX_I2SDE,  "mux_i2s_den",      ingenic_mux_table, mux_table_4, CPM_I2SCDR1,    30, 1, 0),
	MUX(CLK_MUX_PCMCS,  "mux_pcmcs",        ingenic_mux_table, mux_table_2, CPM_PCMCDR, 30, 2, 0),
	MUX(CLK_MUX_PCMDE,  "mux_pcm_den",      ingenic_mux_table, mux_table_4, CPM_PCMCDR1,    30, 1, 0),
	MUX(CLK_MUX_DMIC,   "mux_dmic",     ingenic_mux_table, mux_table_5, CPM_DMICCR, 8, 1, 0),
	MUX(CLK_MUX_LCD,    "mux_lcd",      ingenic_mux_table, mux_table_2, CPM_LPCDR,  30, 2, 0),
	MUX(CLK_MUX_MSC0,   "mux_msc0",     ingenic_mux_table, mux_table_6, CPM_MSC0CDR,    30, 2, 0),
	MUX(CLK_MUX_MSC1,   "mux_msc1",     ingenic_mux_table, mux_table_6, CPM_MSC1CDR,    30, 2, 0),
	MUX(CLK_MUX_SFC,    "mux_sfc",      ingenic_mux_table, mux_table_2, CPM_SFCCDR, 30, 2, 0),
	MUX(CLK_MUX_SSI,    "mux_ssi",      ingenic_mux_table, mux_table_2, CPM_SSICDR, 30, 2, 0),
	MUX(CLK_MUX_CIM,    "mux_cim",      ingenic_mux_table, mux_table_2, CPM_CIMCDR, 30, 2, 0),
	MUX(CLK_MUX_TPC,    "mux_tpc",      ingenic_mux_table, mux_table_2, CPM_TPCCDR, 30, 2, 0),
	MUX(CLK_MUX_G2D,    "mux_g2d",      ingenic_mux_table, mux_table_2, CPM_G2DCDR, 30, 2, 0),
	MUX(CLK_MUX_PWM,    "mux_pwm",      ingenic_mux_table, mux_table_2, CPM_PWMCDR, 30, 2, 0),
	MUX(CLK_MUX_CAN0,   "mux_can0",     ingenic_mux_table, mux_table_3, CPM_CAN0CDR,    30, 2, 0),
	MUX(CLK_MUX_CAN1,   "mux_can1",     ingenic_mux_table, mux_table_3, CPM_CAN1CDR,    30, 2, 0),
	MUX(CLK_MUX_SADC,   "mux_sadc",     ingenic_mux_table, mux_table_3, CPM_SADCCDR,    30, 2, 0),
	MUX(CLK_MUX_WDT,    "mux_wdt",      ingenic_mux_table, mux_table_7, CPM_OPCR,   2, 1, 0),

};

/*******************************************************************************
 *      DIV BUS
 ********************************************************************************/

static struct ingenic_bus_clock x2600_bus_div_clks[] __initdata = {
	BUS_DIV(CLK_DIV_CPU,        "div_cpu",      "mux_cpu_l2c",      CPM_CPCCR,  0,  4, 0, 0, CPM_CPCSR, 0, 22, BUS_DIV_SELF),
	BUS_DIV(CLK_DIV_L2C,        "div_l2c",      "mux_cpu_l2c",      CPM_CPCCR,  4,  4, 0, 0, CPM_CPCSR, 0, 22, BUS_DIV_SELF),
	BUS_DIV(CLK_DIV_AHB0,       "div_ahb0",     "mux_ahb0",     CPM_CPCCR,  8,  4, 0, 0, CPM_CPCSR, 1, 21, BUS_DIV_SELF),
	BUS_DIV(CLK_DIV_AHB2,       "div_ahb2",     "mux_ahb2",     CPM_CPCCR,  12, 4, 0, 0, CPM_CPCSR, 2, 20, BUS_DIV_SELF),
	BUS_DIV(CLK_DIV_APB,        "div_apb",      "mux_ahb2",     CPM_CPCCR,  16, 4, 0, 0, CPM_CPCSR, 2, 20, BUS_DIV_SELF),
	BUS_DIV(CLK_DIV_CPU_L2C_X1, "div_cpu_l2c_x1",   "mux_cpu_l2c",      CPM_CPCCR,  0,  4, 4, 4, CPM_CPCSR, 0, 22, BUS_DIV_ONE),
	BUS_DIV(CLK_DIV_CPU_L2C_X2, "div_cpu_l2c_x2",   "mux_cpu_l2c",      CPM_CPCCR,  0,  4, 4, 4, CPM_CPCSR, 0, 22, BUS_DIV_TWO),
};

/*******************************************************************************
 *      DIV
 ********************************************************************************/

static struct clk_div_table x2600_clk_div_table[256] = {};

static struct ingenic_div_clock x2600_div_clks[] __initdata = {
	DIV(CLK_DIV_DDR,        "div_ddr",      "mux_ddr",      CPM_DDRCDR, 4,  0, NULL),
	DIV(CLK_DIV_MACPHY,     "div_macphy",       "mux_macphy",       CPM_MACPHYCDR,  8,  0, NULL),
	DIV(CLK_DIV_LCD,        "div_lcd",      "mux_lcd",      CPM_LPCDR,  8,  0, NULL),
	DIV(CLK_DIV_MSC0,       "div_msc0",     "mux_msc0",     CPM_MSC0CDR,    8,  0, x2600_clk_div_table),
	DIV(CLK_DIV_MSC1,       "div_msc1",     "mux_msc1",     CPM_MSC1CDR,    8,  0, x2600_clk_div_table),
	DIV(CLK_DIV_SFC,        "div_sfc",      "mux_sfc",      CPM_SFCCDR, 8,  0, NULL),
	DIV(CLK_DIV_SSI,        "div_ssi",      "mux_ssi",      CPM_SSICDR, 8,  0, NULL),
	DIV(CLK_DIV_CIM,        "div_cim",      "mux_cim",      CPM_CIMCDR, 8,  0, NULL),
	DIV(CLK_DIV_TPC,        "div_tpc",      "mux_tpc",      CPM_TPCCDR, 4,  0, NULL),
	DIV(CLK_DIV_G2D,        "div_g2d",      "mux_g2d",      CPM_G2DCDR, 4,  0, NULL),
	DIV(CLK_DIV_PWM,        "div_pwm",      "mux_pwm",      CPM_PWMCDR, 4,  0, NULL),
	DIV(CLK_DIV_CAN0,       "div_can0",     "mux_can0",     CPM_CAN0CDR,    8,  0, NULL),
	DIV(CLK_DIV_CAN1,       "div_can1",     "mux_can1",     CPM_CAN1CDR,    8,  0, NULL),
	DIV(CLK_DIV_SADC,       "div_sadc",     "mux_sadc",     CPM_SADCCDR,    8,  0, NULL),

};

/*******************************************************************************
 *      FRACTIONAL-DIVIDER
 ********************************************************************************/

static struct ingenic_fra_div_clock x2600_fdiv_clks[] __initdata = {
	FRA_DIV(CLK_DIV_I2SMN,  "div_i2s_mn",   "mux_i2scs",    CPM_I2SCDR, 20, 9, 0, 20),
	FRA_DIV(CLK_DIV_I2SD,   "div_i2s_d",    "mux_i2s_den",  CPM_I2SCDR1,    20, 9, 0, 20),
	FRA_DIV(CLK_DIV_PCMMN,  "div_pcm_mn",   "mux_pcmcs",    CPM_PCMCDR, 20, 9, 0, 20),
	FRA_DIV(CLK_DIV_PCMD,   "div_pcm_d",    "mux_pcm_den",  CPM_PCMCDR1,    20, 9, 0, 20),
};

/*******************************************************************************
 *      GATE
 ********************************************************************************/
static struct ingenic_gate_clock x2600_gate_clks[] __initdata = {

	GATE(CLK_GATE_NEMC, "gate_nemc",        "div_ahb2", CPM_CLKGR,  31,  0,             CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_INTC, "gate_intc",        "div_ahb2", CPM_CLKGR, 30, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_RTC,  "gate_rtc",     "rtc_ext",  CPM_CLKGR,  29, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_OST,  "gate_ost",     "ext",      CPM_CLKGR,  28, CLK_IGNORE_UNUSED,  CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_DTRNG,    "gate_dtrng",       "div_apb",  CPM_CLKGR,  27, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_I2S,  "gate_i2s",     "div_i2s_mn",   CPM_CLKGR, 26,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_DMIC, "gate_dmic",        "div_dmic", CPM_CLKGR, 25,  0,          CLK_GATE_SET_TO_DISABLE),   //TODO
	GATE(CLK_GATE_AIC,  "gate_aic",     "div_apb",  CPM_CLKGR, 24,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_MIPI_DSI, "gate_dsi",     "div_apb",  CPM_CLKGR, 23,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SSISLV,   "gate_ssislv",      "div_apb",  CPM_CLKGR, 22,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SSI1, "gate_ssi1",        "div_ssi",  CPM_CLKGR, 21,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SSI0, "gate_ssi0",        "div_ssi",  CPM_CLKGR, 20,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART7,    "gate_uart7",       "div_apb",  CPM_CLKGR, 19,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART6,    "gate_uart6",       "div_apb",  CPM_CLKGR, 18,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART5,    "gate_uart5",       "div_apb",  CPM_CLKGR, 17,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART4,    "gate_uart4",       "div_apb",  CPM_CLKGR, 16,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART3,    "gate_uart3",       "div_apb",  CPM_CLKGR, 15,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART2,    "gate_uart2",       "div_apb",  CPM_CLKGR, 14,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART1,    "gate_uart1",       "div_apb",  CPM_CLKGR, 13,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART0,    "gate_uart0",       "div_apb",  CPM_CLKGR, 12,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_I2C3, "gate_i2c3",        "div_apb",  CPM_CLKGR, 11,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_I2C2, "gate_i2c2",        "div_apb",  CPM_CLKGR, 10,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_I2C1, "gate_i2c1",        "div_apb",  CPM_CLKGR, 9,  0,           CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_I2C0, "gate_i2c0",        "div_apb",  CPM_CLKGR, 8,  0,           CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_MAC,  "gate_gmac",        "div_ahb2", CPM_CLKGR, 6,  0,           CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_USB,  "gate_usb",     "div_ahb2", CPM_CLKGR, 5,  0,           CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_OTG,  "gate_otg",     "div_ahb2", CPM_CLKGR, 4,  0,           CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_MSC1, "gate_msc1",        "div_msc1", CPM_CLKGR, 3,  0,           CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_MSC0, "gate_msc0",        "div_msc0", CPM_CLKGR, 2,  0,           CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SFC,  "gate_sfc",     "div_sfc",  CPM_CLKGR, 1,  0,           CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_EFUSE,    "gate_efuse",       "div_ahb2", CPM_CLKGR, 0,  0,           CLK_GATE_SET_TO_DISABLE),

	GATE(CLK_GATE_AHB0, "gate_ahb0",        "div_ahb0", CPM_CLKGR1,  31, CLK_IGNORE_UNUSED, CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_APB0, "gate_apb0",        "div_ahb0", CPM_CLKGR1,  30, CLK_IGNORE_UNUSED, CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_AHB2, "gate_ahb2",        "div_ahb2", CPM_CLKGR1,  29, CLK_IGNORE_UNUSED, CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_APB,  "gate_apb",     "div_apb",  CPM_CLKGR1,  28, CLK_IGNORE_UNUSED, CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_ARB,  "gate_arb",     "div_ahb0", CPM_CLKGR1,  27, CLK_IGNORE_UNUSED, CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_DDR,  "gate_ddr",     "div_ddr",  CPM_CLKGR1,  23, CLK_IGNORE_UNUSED, CLK_GATE_SET_TO_DISABLE),

	GATE(CLK_GATE_PCM,  "gate_pcm1",    "div_pcm_mn",   CPM_CLKGR1, 22,  0,             CLK_GATE_SET_TO_DISABLE),   // pcm1是设备时钟
	GATE(CLK_GATE_PCM,  "gate_pcm",     "div_pcm_mn",   CPM_CLKGR1, 21,  0,             CLK_GATE_SET_TO_DISABLE),   // pcm0是总线时钟，访问寄存器的

	GATE(CLK_GATE_BUS_MON,  "gate_bus_mon",     "div_ahb0", CPM_CLKGR1, 20,  0,             CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_JPEGE,    "gate_jpege",       "div_ahb0", CPM_CLKGR1, 19,  0,             CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_JPEGD,    "gate_jpegd",       "div_ahb0", CPM_CLKGR1, 18,  0,             CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_FELIX,    "gate_felix",       "div_ahb0", CPM_CLKGR1, 17,  0,             CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_ROT,  "gate_rot",     "div_ahb0", CPM_CLKGR1, 16,  0,             CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_G2D,  "gate_g2d",     "div_g2d",  CPM_CLKGR1, 15,  0,             CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_LCD,  "gate_lcd",     "div_lcd",  CPM_CLKGR1, 14,  0,             CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_CIM,  "gate_cim",     "div_cim",  CPM_CLKGR1, 13,  0,             CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_TCSM, "gate_tcsm",        "div_ahb2", CPM_CLKGR1, 11,  0,             CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SADC, "gate_sadc",        "div_ahb2", CPM_CLKGR1, 10,  0,             CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_TCU1, "gate_tcu1",        "div_ahb2", CPM_CLKGR1, 9,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_TCU0, "gate_tcu0",        "div_ahb2", CPM_CLKGR1, 8,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_TPC,  "gate_tpc",     "div_tpc",  CPM_CLKGR1, 7,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_PWM,  "gate_pwm",     "div_pwm",  CPM_CLKGR1, 6,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_CAN1, "gate_can1",        "div_can1", CPM_CLKGR1, 5,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_CAN0, "gate_can0",        "div_can0", CPM_CLKGR1, 4,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_HASH, "gate_hash",        "div_ahb2", CPM_CLKGR1, 3,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_AES,  "gate_aes",     "div_ahb2", CPM_CLKGR1, 2,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_PDMA1,    "gate_pdma1",       "div_ahb2", CPM_CLKGR1, 1,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_PDMA, "gate_pdma",        "div_ahb2", CPM_CLKGR1, 0,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_USBPHY,   "gate_usbphy",      "div_apb",  CPM_OPCR,   23, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_CIM_MCLK, "gate_cim_mclk",    "div_apb",  CPM_OPCR,   5, 0,           CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_CE_I2ST,   "ce_i2st",      "div_i2s_mn",   CPM_I2SCDR, 29, 0, 0),
	GATE(CLK_CE_PCM,    "ce_pcm",       "div_pcm_mn",   CPM_PCMCDR, 29, 0, 0),
	GATE(CLK_CE_DMIC,   "ce_dmic",      "div_dmic", CPM_DMICCR, 0, 0, 0),

};

static void clk_div_table_generate(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(x2600_clk_div_table); i++) {
		x2600_clk_div_table[i].val = i;
		x2600_clk_div_table[i].div = (i + 1) * 4;
	}

}

static const struct of_device_id ext_clk_match[] __initconst = {
	{ .compatible = "ingenic,fixed-clock", .data = (void *)0, },
	{},
};

/* Register x2600 clocks. */
static void __init x2600_clk_init(struct device_node *np)
{
	struct ingenic_clk_provider *ctx;
	void __iomem *reg_base;

	printk("x2600 Clock Power Management Unit init!\n");

	if (np) {
		reg_base = of_iomap(np, 0);
		if (!reg_base) {
			panic("%s: failed to map registers\n", __func__);
		}
	}

	ctx = ingenic_clk_init(np, reg_base, NR_CLKS);
	if (!ctx) {
		panic("%s: unable to allocate context.\n", __func__);
	}

	/* Register Ext Clocks From DT */
	ingenic_clk_of_register_fixed_ext(ctx, x2600_fixed_rate_ext_clks,
	                                  ARRAY_SIZE(x2600_fixed_rate_ext_clks), ext_clk_match);

	/* Register PLLs. */
	ingenic_clk_register_pll(ctx, x2600_pll_clks,
	                         ARRAY_SIZE(x2600_pll_clks), reg_base);

	/* Register Muxs */
	ingenic_clk_register_mux(ctx, x2600_mux_clks, ARRAY_SIZE(x2600_mux_clks));

	/* Register Bus Divs */
	ingenic_clk_register_bus_div(ctx, x2600_bus_div_clks, ARRAY_SIZE(x2600_bus_div_clks));

	/* Register Divs */
	clk_div_table_generate();
	ingenic_clk_register_cgu_div(ctx, x2600_div_clks, ARRAY_SIZE(x2600_div_clks));

	/* Register Fractional Divs */
	ingenic_clk_register_fra_div(ctx, x2600_fdiv_clks, ARRAY_SIZE(x2600_fdiv_clks));

	/* Register Gates */
	ingenic_clk_register_gate(ctx, x2600_gate_clks, ARRAY_SIZE(x2600_gate_clks));

	/* Register Powers */
	//ingenic_power_register_gate(ctx, x2600_gate_power, ARRAY_SIZE(x2600_gate_power));

	ingenic_clk_of_add_provider(np, ctx);

	//ingenic_clk_of_dump(ctx);

	pr_info("=========== x2600 clocks: =============\n"
	        "\tapll     = %lu , mpll     = %lu, ddr = %lu\n"
	        "\tcpu_clk  = %lu , l2c_clk  = %lu\n"
	        "\tahb0_clk = %lu , ahb2_clk = %lu\n"
	        "\tapb_clk  = %lu , ext_clk  = %lu\n\n",
	        _get_rate("apll"),  _get_rate("mpll"), _get_rate("div_ddr"),
	        _get_rate("div_cpu"), _get_rate("div_l2c"),
	        _get_rate("div_ahb0"), _get_rate("div_ahb2"),
	        _get_rate("div_apb"), _get_rate("ext"));

}

CLK_OF_DECLARE_DRIVER(x2600_clk, "ingenic,x2600-clocks", x2600_clk_init);
