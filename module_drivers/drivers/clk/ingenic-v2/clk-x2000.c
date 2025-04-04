#include <linux/slab.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/syscore_ops.h>
#include <linux/clk.h>

#include <dt-bindings/clock/ingenic-x2000.h>

#include "clk.h"
#include "clk-pll-v1.h"

/*******************************************************************************
 *      FIXED CLK
 ********************************************************************************/
static struct ingenic_fixed_rate_clock x2000_fixed_rate_ext_clks[] __initdata = {
	FRATE(CLK_EXT, "ext", NULL, 0, 24000000),
	FRATE(CLK_RTC_EXT, "rtc_ext", NULL, 0, 32768),
	FRATE(CLK_EXT_DIV_512, "ext_div_512", NULL, 0, 46875),
};

/*******************************************************************************
 *      PLL
 ********************************************************************************/

static struct ingenic_pll_rate_table x2000_pll_rate_table[] = {
	PLL_RATE(1500000000, 124, 1, 1),
	PLL_RATE(1200000000, 49, 0, 1),
	PLL_RATE(800000000, 199, 2, 2),
	PLL_RATE(600000000, 49, 0, 2),
	PLL_RATE(300000000, 49, 0, 3),
};

/*PLL HWDESC*/
static struct ingenic_pll_hwdesc apll_hwdesc = \
    PLL_DESC(CPM_CPAPCR, 20, 9, 14, 6, 11, 3, 3, 0, pll_od_encode);

static struct ingenic_pll_hwdesc mpll_hwdesc = \
    PLL_DESC(CPM_CPMPCR, 20, 9, 14, 6, 11, 3, 3, 0, pll_od_encode);

static struct ingenic_pll_hwdesc epll_hwdesc = \
    PLL_DESC(CPM_CPEPCR, 20, 9, 14, 6, 11, 3, 3, 0, pll_od_encode);

static struct ingenic_pll_clock x2000_pll_clks[] __initdata = {
	PLL(CLK_PLL_APLL, "apll", "ext", &apll_hwdesc, x2000_pll_rate_table),
	PLL(CLK_PLL_MPLL, "mpll", "ext", &mpll_hwdesc, x2000_pll_rate_table),
	PLL(CLK_PLL_EPLL, "epll", "ext", &epll_hwdesc, x2000_pll_rate_table),
};

/*******************************************************************************
 *      MUX
 ********************************************************************************/
PNAME(mux_table_0)  = {"stop",  "ext",      "apll"};
PNAME(mux_table_1)  = {"stop",  "sclka",    "mpll"};
PNAME(mux_table_2)  = {"sclka", "mpll",     "epll"};
PNAME(mux_table_3)  = {"sclka",     "mpll",     "ext"};
PNAME(mux_table_4)  = {"sclka",     "epll"};
PNAME(mux_table_5)  = {"mux_ahb2",  "ext"};
PNAME(mux_table_6)  = {"ext",   "div_i2s3"};
PNAME(mux_table_7)  = {"div_i2s0",  "div_i2s1", "div_i2s2", "div_i2s3"};
PNAME(mux_table_8)  = {"ext_div_512",   "rtc_ext"};

static unsigned int ingenic_mux_table[] = {0, 1, 2, 3};

static struct ingenic_mux_clock x2000_mux_clks[] __initdata = {
	MUX(CLK_MUX_SCLKA,  "sclka",        ingenic_mux_table, mux_table_0, CPM_CPCCR,  30, 2, 0),
	MUX(CLK_MUX_CPU_L2C,    "mux_cpu_l2c",      ingenic_mux_table, mux_table_1, CPM_CPCCR,  28, 2, 0),
	MUX(CLK_MUX_AHB0,   "mux_ahb0",     ingenic_mux_table, mux_table_1, CPM_CPCCR,  26, 2, 0),
	MUX(CLK_MUX_AHB2,   "mux_ahb2",     ingenic_mux_table, mux_table_1, CPM_CPCCR,  24, 2, 0),

	MUX(CLK_MUX_DDR,    "mux_ddr",      ingenic_mux_table, mux_table_1, CPM_DDRCDR, 30, 2, 0),
	MUX(CLK_MUX_MACPHY, "mux_macphy",       ingenic_mux_table, mux_table_2, CPM_MACCDR, 30, 2, 0),
	MUX(CLK_MUX_MACTXPHY,   "mux_mactxphy",     ingenic_mux_table, mux_table_2, CPM_MACTXCDR,   30, 2, 0),
	MUX(CLK_MUX_MACTXPHY1,  "mux_mactxphy1",    ingenic_mux_table, mux_table_2, CPM_MACTXCDR1,  30, 2, 0),
	MUX(CLK_MUX_MACPTP, "mux_macptp",       ingenic_mux_table, mux_table_2, CPM_MACPTP, 30, 2, 0),
	MUX(CLK_MUX_I2S0,   "mux_i2s0",     ingenic_mux_table, mux_table_4, CPM_I2S0CDR,    30, 1, 0),
	MUX(CLK_MUX_I2S1,   "mux_i2s1",     ingenic_mux_table, mux_table_4, CPM_I2S1CDR,    30, 1, 0),
	MUX(CLK_MUX_I2S2,   "mux_i2s2",     ingenic_mux_table, mux_table_4, CPM_I2S2CDR,    30, 1, 0),
	MUX(CLK_MUX_I2S3,   "mux_i2s3",     ingenic_mux_table, mux_table_4, CPM_I2S3CDR,    30, 1, 0),
	MUX(CLK_MUX_AUDIO_RAM,  "mux_audio_ram",    ingenic_mux_table, mux_table_5, CPM_AUDIOCR,    30, 1, 0),
	MUX(CLK_MUX_SPDIF,  "mux_spdif",        ingenic_mux_table, mux_table_7, CPM_AUDIOCR,    3,  2, CLK_SET_RATE_PARENT | CLK_SET_RATE_NO_REPARENT),
	MUX(CLK_MUX_PCM,    "mux_pcm",      ingenic_mux_table, mux_table_7, CPM_AUDIOCR,    1,  2, CLK_SET_RATE_PARENT | CLK_SET_RATE_NO_REPARENT),
	MUX(CLK_MUX_DMIC,   "mux_dmic",     ingenic_mux_table, mux_table_6, CPM_AUDIOCR,    0,  1, CLK_SET_RATE_PARENT | CLK_SET_RATE_NO_REPARENT),
	MUX(CLK_MUX_LCD,    "mux_lcd",      ingenic_mux_table, mux_table_2, CPM_LPCDR,  30, 2, 0),
	MUX(CLK_MUX_MSC0,   "mux_msc0",     ingenic_mux_table, mux_table_3, CPM_MSC0CDR,    30, 2, 0),
	MUX(CLK_MUX_MSC1,   "mux_msc1",     ingenic_mux_table, mux_table_3, CPM_MSC1CDR,    30, 2, 0),
	MUX(CLK_MUX_MSC2,   "mux_msc2",     ingenic_mux_table, mux_table_3, CPM_MSC2CDR,    30, 2, 0),
	MUX(CLK_MUX_SFC,    "mux_sfc",      ingenic_mux_table, mux_table_2, CPM_SFCCDR, 30, 2, 0),
	MUX(CLK_MUX_SSI,    "mux_ssi",      ingenic_mux_table, mux_table_2, CPM_SSICDR, 30, 2, 0),
	MUX(CLK_MUX_CIM,    "mux_cim",      ingenic_mux_table, mux_table_2, CPM_CIMCDR, 30, 2, 0),
	MUX(CLK_MUX_PWM,    "mux_pwm",      ingenic_mux_table, mux_table_2, CPM_PWMCDR, 30, 2, 0),
	MUX(CLK_MUX_ISP,    "mux_isp",      ingenic_mux_table, mux_table_2, CPM_ISPCDR, 30, 2, 0),
	MUX(CLK_MUX_RSA,    "mux_rsa",      ingenic_mux_table, mux_table_2, CPM_RSACDR, 30, 2, 0),
	MUX(CLK_MUX_WDT,    "mux_wdt",      ingenic_mux_table, mux_table_8, CPM_OPCR,   2, 1, 0),

};

/*******************************************************************************
 *      DIV BUS
 ********************************************************************************/

static struct ingenic_bus_clock x2000_bus_div_clks[] __initdata = {
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

static struct clk_div_table x2000_clk_div_table[256] = {};

static struct ingenic_div_clock x2000_div_clks[] __initdata = {
	DIV(CLK_DIV_DDR,        "div_ddr",      "mux_ddr",      CPM_DDRCDR, 4,  0, NULL),
	DIV(CLK_DIV_MACPHY,     "div_macphy",       "mux_macphy",       CPM_MACCDR, 8,  0, NULL),
	DIV(CLK_DIV_MACTXPHY,       "div_mactxphy0",    "mux_mactxphy",     CPM_MACTXCDR,   8,  0, NULL),
	DIV(CLK_DIV_MACTXPHY1,      "div_mactxphy1",    "mux_mactxphy1",    CPM_MACTXCDR1,  8,  0, NULL),
	DIV(CLK_DIV_MACPTP,     "div_macptp",       "mux_macptp",       CPM_MACPTP, 8,  0, NULL),
	DIV(CLK_DIV_LCD,        "div_lcd",      "mux_lcd",      CPM_LPCDR,  8,  0, NULL),
	DIV(CLK_DIV_MSC0,       "div_msc0",     "mux_msc0",     CPM_MSC0CDR,    8,  0, x2000_clk_div_table),
	DIV(CLK_DIV_MSC1,       "div_msc1",     "mux_msc1",     CPM_MSC1CDR,    8,  0, x2000_clk_div_table),
	DIV(CLK_DIV_MSC2,       "div_msc2",     "mux_msc2",     CPM_MSC2CDR,    8,  0, x2000_clk_div_table),
	DIV(CLK_DIV_SFC,        "div_sfc",      "mux_sfc",      CPM_SFCCDR, 8,  0, NULL),
	DIV(CLK_DIV_SSI,        "div_ssi",      "mux_ssi",      CPM_SSICDR, 8,  0, NULL),
	DIV(CLK_DIV_CIM,        "div_cim",      "mux_cim",      CPM_CIMCDR, 8,  0, NULL),
	DIV(CLK_DIV_PWM,        "div_pwm",      "mux_pwm",      CPM_PWMCDR, 4,  0, NULL),
	DIV(CLK_DIV_ISP,        "div_isp",      "mux_isp",      CPM_ISPCDR, 4,  0, NULL),
	DIV(CLK_DIV_RSA,        "div_rsa",      "mux_rsa",      CPM_RSACDR, 4,  0, NULL),

};

/*******************************************************************************
 *      FRACTIONAL-DIVIDER
 ********************************************************************************/

static struct ingenic_fra_div_clock x2000_fdiv_clks[] __initdata = {
	FRA_DIV(CLK_DIV_I2S0,   "div_i2s0", "mux_i2s0", CPM_I2S0CDR,    20, 9, 0, 20),
	FRA_DIV(CLK_DIV_I2S1,   "div_i2s1", "mux_i2s1", CPM_I2S1CDR,    20, 9, 0, 20),
	FRA_DIV(CLK_DIV_I2S2,   "div_i2s2", "mux_i2s2", CPM_I2S2CDR,    20, 9, 0, 20),
	FRA_DIV(CLK_DIV_I2S3,   "div_i2s3", "mux_i2s3", CPM_I2S3CDR,    20, 9, 0, 20),
};

/*******************************************************************************
 *      GATE
 ********************************************************************************/
static struct ingenic_gate_clock x2000_gate_clks[] __initdata = {

	GATE(CLK_GATE_DDR,  "gate_ddr",     "div_ddr",  CPM_CLKGR,  31, CLK_IGNORE_UNUSED,  CLK_GATE_SET_TO_DISABLE),
	//  GATE(CLK_GATE_IPU,  "gate_ipu",     "div_ipu",  CPM_CLKGR,  30, 0,           CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_AHB0, "gate_ahb0",        "div_ahb0", CPM_CLKGR,  29, CLK_IGNORE_UNUSED,  CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_APB0, "gate_apb0",        "div_ahb0", CPM_CLKGR,  28, CLK_IGNORE_UNUSED,  CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_RTC,  "gate_rtc",     "rtc_ext",  CPM_CLKGR,  27, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SSI1, "gate_ssi1",        "div_ssi",  CPM_CLKGR,  26, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_RSA,  "gate_rsa",     "div_rsa",  CPM_CLKGR,  25, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_AES,  "gate_aes",     "div_ahb2", CPM_CLKGR,  24, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_LCD,  "gate_lcd",     "div_lcd",  CPM_CLKGR,  23, CLK_IGNORE_UNUSED,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_CIM,  "gate_cim",     "div_cim",  CPM_CLKGR,  22, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_PDMA, "gate_pdma",        "div_ahb2", CPM_CLKGR,  21, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_OST,  "gate_ost",     "ext",      CPM_CLKGR,  20, CLK_IGNORE_UNUSED,  CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SSI0, "gate_ssi0",        "div_ssi",  CPM_CLKGR,  19, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_TCU,  "gate_tcu",     "div_apb",  CPM_CLKGR,  18, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_DTRNG,    "gate_dtrng",       "div_apb",  CPM_CLKGR,  17, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART2,    "gate_uart2",       "div_apb",  CPM_CLKGR,  16, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART1,    "gate_uart1",       "div_apb",  CPM_CLKGR,  15, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART0,    "gate_uart0",       "div_apb",  CPM_CLKGR,  14, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SADC, "gate_sadc",        "div_apb",  CPM_CLKGR,  13, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_HELIX,    "gate_helix",       "div_ahb0", CPM_CLKGR,  12, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_AUDIO,    "gate_audio",       "div_ahb2", CPM_CLKGR,  11, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SMB3, "gate_i2c3",        "div_apb",  CPM_CLKGR,  10, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SMB2, "gate_i2c2",        "div_apb",  CPM_CLKGR,  9,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SMB1, "gate_i2c1",        "div_apb",  CPM_CLKGR,  8,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SMB0, "gate_i2c0",        "div_apb",  CPM_CLKGR,  7,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SCC,  "gate_scc",     "div_apb",  CPM_CLKGR,  6,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_MSC1, "gate_msc1",        "div_msc1", CPM_CLKGR,  5,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_MSC0, "gate_msc0",        "div_msc0", CPM_CLKGR,  4,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_OTG,  "gate_otg",     "div_ahb2", CPM_CLKGR,  3,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SFC,  "gate_sfc",     "div_sfc",  CPM_CLKGR,  2,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_EFUSE,    "gate_efuse",       "div_ahb2", CPM_CLKGR,  1,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_NEMC, "gate_nemc",        "div_ahb2", CPM_CLKGR,  0,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_ARB,  "gate_arb",     "div_ahb0", CPM_CLKGR1, 30, CLK_IGNORE_UNUSED,  CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_MIPI_DSI, "gate_dsi",     "div_ahb0", CPM_CLKGR1, 29, CLK_IGNORE_UNUSED,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_MIPI_CSI, "gate_csi",     "div_ahb0", CPM_CLKGR1, 28, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_INTC, "gate_intc",        "div_ahb2", CPM_CLKGR1, 26, CLK_IGNORE_UNUSED,  CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_MSC2, "gate_msc2",        "div_msc2", CPM_CLKGR1, 25, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_GMAC1,    "gate_gmac1",       "div_ahb2", CPM_CLKGR1, 24, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_GMAC0,    "gate_gmac0",       "div_ahb2", CPM_CLKGR1, 23, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART9,    "gate_uart9",       "div_apb",  CPM_CLKGR1, 22, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART8,    "gate_uart8",       "div_apb",  CPM_CLKGR1, 21, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART7,    "gate_uart7",       "div_apb",  CPM_CLKGR1, 20, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART6,    "gate_uart6",       "div_apb",  CPM_CLKGR1, 19, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART5,    "gate_uart5",       "div_apb",  CPM_CLKGR1, 18, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART4,    "gate_uart4",       "div_apb",  CPM_CLKGR1, 17, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_UART3,    "gate_uart3",       "div_apb",  CPM_CLKGR1, 16, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SPDIF,    "gate_spdif",       "mux_spdif",    CPM_CLKGR1, 14, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_DMIC, "gate_dmic",        "mux_dmic", CPM_CLKGR1, 13, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_PCM,  "gate_pcm",     "mux_pcm",  CPM_CLKGR1, 12, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_I2S3, "gate_i2s3",        "div_i2s3", CPM_CLKGR1, 11, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_I2S2, "gate_i2s2",        "div_i2s2", CPM_CLKGR1, 10, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_I2S1, "gate_i2s1",        "div_i2s1", CPM_CLKGR1, 9,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_I2S0, "gate_i2s0",        "div_i2s0", CPM_CLKGR1, 8,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_ROT,  "gate_rot",     "div_ahb0", CPM_CLKGR1, 7,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_HASH, "gate_hash",        "div_ahb2", CPM_CLKGR1, 6,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_PWM,  "gate_pwm",     "div_pwm",  CPM_CLKGR1, 5,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_FELIX,    "gate_felix",       "div_ahb0", CPM_CLKGR1, 4,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_ISP1, "gate_isp1",        "div_apb",  CPM_CLKGR1, 3,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_ISP0, "gate_isp0",        "div_apb",  CPM_CLKGR1, 2,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SMB5, "gate_i2c5",        "div_apb",  CPM_CLKGR1, 1,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_SMB4, "gate_i2c4",        "div_apb",  CPM_CLKGR1, 0,  0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_USBPHY,   "gate_usbphy",      "div_apb",  CPM_OPCR,   23, 0,          CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_GATE_CIM_MCLK, "gate_cim_mclk",    "div_apb",  CPM_OPCR,   5, 0,           CLK_GATE_SET_TO_DISABLE),
	GATE(CLK_CE_I2S0,   "ce_i2s0",      "div_i2s0", CPM_I2S0CDR, 29, 0, 0),
	GATE(CLK_CE_I2S1,   "ce_i2s1",      "div_i2s1", CPM_I2S1CDR, 29, 0, 0),
	GATE(CLK_CE_I2S2,   "ce_i2s2",      "div_i2s2", CPM_I2S2CDR, 29, 0, 0),
	GATE(CLK_CE_I2S3,   "ce_i2s3",      "div_i2s3", CPM_I2S3CDR, 29, 0, 0),
};

static struct ingenic_gate_power x2000_gate_power[] __initdata = {

	POWER(POWER_ISP0,   "power_isp0",       "gate_isp0",    CPM_LCR,  31, 27, CLK_IGNORE_UNUSED,    CLK_GATE_SET_TO_DISABLE, POWER_GATE_WAIT),
	POWER(POWER_ISP1,   "power_isp1",       "gate_isp1",    CPM_LCR,  30, 26, CLK_IGNORE_UNUSED,    CLK_GATE_SET_TO_DISABLE, POWER_GATE_WAIT),
	POWER(POWER_FELIX,  "power_felix",      "gate_felix",   CPM_LCR,  29, 25, CLK_IGNORE_UNUSED,    CLK_GATE_SET_TO_DISABLE, POWER_GATE_WAIT),
	POWER(POWER_HELIX,  "power_helix",      "gate_helix",   CPM_LCR,  28, 24, CLK_IGNORE_UNUSED,    CLK_GATE_SET_TO_DISABLE, POWER_GATE_WAIT),

	POWER(PD_MEM_NEMC,  "pd_mem_nemc",      "gate_nemc",    CPM_MPDCR,  28, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_USB,   "pd_mem_usb",       "gate_usb", CPM_MPDCR,  27, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_SFC,   "pd_mem_sfc",       "gate_sfc", CPM_MPDCR,  26, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_PDMA_SEC,  "pd_mem_pdma_sec",  "gate_pdma",    CPM_MPDCR,  25, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_PDMA,  "pd_mem_pdma",      "gate_pdma",    CPM_MPDCR,  24, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_MSC2,  "pd_mem_msc2",      "gate_msc2",    CPM_MPDCR,  23, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_MSC1,  "pd_mem_msc1",      "gate_msc1",    CPM_MPDCR,  22, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_MSC0,  "pd_mem_msc0",      "gate_msc0",    CPM_MPDCR,  21, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_GMAC1, "pd_mem_gmac1",     "gate_gmac1",   CPM_MPDCR,  20, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_GMAC0, "pd_mem_gmac0",     "gate_gmac0",   CPM_MPDCR,  19, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_DMIC,  "pd_mem_dmic",      "gate_dmic",    CPM_MPDCR,  18, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_AUDIO, "pd_mem_audio",     "gate_audio",   CPM_MPDCR,  17, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_AES,   "pd_mem_aes",       "gate_aes", CPM_MPDCR,  16, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_DSI,   "pd_mem_dsi",       "gate_dsi", CPM_MPDCR,  12, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_SSI1,  "pd_mem_ssi1",      "gate_ssi1",    CPM_MPDCR,  11, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_SSI0,  "pd_mem_ssi0",      "gate_ssi0",    CPM_MPDCR,  10, 0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_UART9, "pd_mem_uart9",     "gate_uart9",   CPM_MPDCR,  9,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_UART8, "pd_mem_uart8",     "gate_uart8",   CPM_MPDCR,  8,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_UART7, "pd_mem_uart7",     "gate_uart7",   CPM_MPDCR,  7,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_UART6, "pd_mem_uart6",     "gate_uart6",   CPM_MPDCR,  6,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_UART5, "pd_mem_uart5",     "gate_uart5",   CPM_MPDCR,  5,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_UART4, "pd_mem_uart4",     "gate_uart4",   CPM_MPDCR,  4,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_UART3, "pd_mem_uart3",     "gate_uart3",   CPM_MPDCR,  3,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_UART2, "pd_mem_uart2",     "gate_uart2",   CPM_MPDCR,  2,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_UART1, "pd_mem_uart1",     "gate_uart1",   CPM_MPDCR,  1,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_UART0, "pd_mem_uart0",     "gate_uart0",   CPM_MPDCR,  0,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_ROT,   "pd_mem_rot",       "gate_rot", CPM_MPDCR1, 8,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_CIM,   "pd_mem_cim",       "gate_cim", CPM_MPDCR1, 7,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_DPU,   "pd_mem_dpu",       "gate_dpu", CPM_MPDCR1, 6,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_DDR_CH6,   "pd_mem_ddr_ch6",   "gate_ddr", CPM_MPDCR1, 5,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_DDR_CH5,   "pd_mem_ddr_ch5",   "gate_ddr", CPM_MPDCR1, 4,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_DDR_CH3,   "pd_mem_ddr_ch3",   "gate_ddr", CPM_MPDCR1, 3,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_DDR_CH1,   "pd_mem_ddr_ch1",   "gate_ddr", CPM_MPDCR1, 2,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_DDR_CH0,   "pd_mem_ddr_ch0",   "gate_ddr", CPM_MPDCR1, 1,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),
	POWER(PD_MEM_DDR_TOP,   "pd_mem_ddr_top",   "gate_ddr", CPM_MPDCR1, 0,  0, CLK_IGNORE_UNUSED,   CLK_GATE_SET_TO_DISABLE, 0),

};

static void clk_div_table_generate(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(x2000_clk_div_table); i++) {
		x2000_clk_div_table[i].val = i;
		x2000_clk_div_table[i].div = (i + 1) * 4;
	}

}

static const struct of_device_id ext_clk_match[] __initconst = {
	{ .compatible = "ingenic,fixed-clock", .data = (void *)0, },
	{},
};
struct ingenic_clk_reg_sleep x2000_clk_reg_sleep[] = {

	REG_SLEEP(CPM_MACCDR,   29, 28, 0),
	REG_SLEEP(CPM_MACTXCDR, 29, 28, 0),
	REG_SLEEP(CPM_MACTXCDR1, 29, 28, 0),
	REG_SLEEP(CPM_MACPTP,   29, 28, 0),
	REG_SLEEP(CPM_I2S0CDR,  0, 0, 0),
	REG_SLEEP(CPM_I2S1CDR,  0, 0, 0),
	REG_SLEEP(CPM_I2S2CDR,  0, 0, 0),
	REG_SLEEP(CPM_I2S3CDR,  0, 0, 0),
	REG_SLEEP(CPM_I2S0CDR1, 0, 0, 0),
	REG_SLEEP(CPM_I2S1CDR1, 0, 0, 0),
	REG_SLEEP(CPM_I2S2CDR1, 0, 0, 0),
	REG_SLEEP(CPM_I2S3CDR1, 0, 0, 0),
	REG_SLEEP(CPM_AUDIOCR,  0, 0, 0),
	REG_SLEEP(CPM_LPCDR,    29, 28, 0),
	REG_SLEEP(CPM_MSC0CDR,    29, 28, 0),
	REG_SLEEP(CPM_MSC1CDR,    29, 28, 0),
	REG_SLEEP(CPM_MSC2CDR,    29, 28, 0),
	REG_SLEEP(CPM_SFCCDR,    29, 28, 0),
	REG_SLEEP(CPM_SSICDR,    29, 28, 0),
	REG_SLEEP(CPM_CIMCDR,    29, 28, 0),
	REG_SLEEP(CPM_PWMCDR,    29, 28, 0),
	REG_SLEEP(CPM_ISPCDR,    29, 28, 0),
	REG_SLEEP(CPM_RSACDR,    29, 28, 0),
};

unsigned int x2000_nr_clk_reg_sleep = ARRAY_SIZE(x2000_clk_reg_sleep);

/* Register x2000 clocks. */
static void __init x2000_clk_init(struct device_node *np)
{
	struct ingenic_clk_provider *ctx;
	void __iomem *reg_base;

	printk("x2000 Clock Power Management Unit init!\n");

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
	ingenic_clk_of_register_fixed_ext(ctx, x2000_fixed_rate_ext_clks,
	                                  ARRAY_SIZE(x2000_fixed_rate_ext_clks), ext_clk_match);

	/* Register PLLs. */
	ingenic_clk_register_pll(ctx, x2000_pll_clks,
	                         ARRAY_SIZE(x2000_pll_clks), reg_base);

	/* Register Muxs */
	ingenic_clk_register_mux(ctx, x2000_mux_clks, ARRAY_SIZE(x2000_mux_clks));

	/* Register Bus Divs */
	ingenic_clk_register_bus_div(ctx, x2000_bus_div_clks, ARRAY_SIZE(x2000_bus_div_clks));

	/* Register Divs */
	clk_div_table_generate();
	ingenic_clk_register_cgu_div(ctx, x2000_div_clks, ARRAY_SIZE(x2000_div_clks));

	/* Register Fractional Divs */
	ingenic_clk_register_fra_div(ctx, x2000_fdiv_clks, ARRAY_SIZE(x2000_fdiv_clks));

	/* Register Gates */
	ingenic_clk_register_gate(ctx, x2000_gate_clks, ARRAY_SIZE(x2000_gate_clks));

	/* Register Powers */
	ingenic_power_register_gate(ctx, x2000_gate_power, ARRAY_SIZE(x2000_gate_power));

	ingenic_clk_of_add_provider(np, ctx);

	ingenic_clk_sleep_init(reg_base, x2000_clk_reg_sleep, x2000_nr_clk_reg_sleep);

	//ingenic_clk_of_dump(ctx);

	pr_info("=========== x2000 clocks: =============\n"
	        "\tapll     = %lu , mpll     = %lu\n"
	        "\tcpu_clk  = %lu , l2c_clk  = %lu\n"
	        "\tahb0_clk = %lu , ahb2_clk = %lu\n"
	        "\tapb_clk  = %lu , ext_clk  = %lu\n\n",
	        _get_rate("apll"),  _get_rate("mpll"),
	        _get_rate("div_cpu"), _get_rate("div_l2c"),
	        _get_rate("div_ahb0"), _get_rate("div_ahb2"),
	        _get_rate("div_apb"), _get_rate("ext"));

}

CLK_OF_DECLARE_DRIVER(x2000_clk, "ingenic,x2000-clocks", x2000_clk_init);
