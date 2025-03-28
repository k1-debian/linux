#ifndef __INGENIC_CLK_H
#define __INGENIC_CLK_H

#include <linux/clk-provider.h>
#include "clk-pll.h"
#include "clk-div.h"
#include "clk-bus.h"
#include "power-gate.h"
#include <soc/cpm.h>

struct clk;

/**
 * struct ingenic_clk_provider: information about clock provider
 * @reg_base: virtual address for the register base.
 * @clk_data: holds clock related data like clk* and number of clocks.
 * @lock: maintains exclusion between callbacks for a given clock-provider.
 */
struct ingenic_clk_provider {
	void __iomem *reg_base;
	struct clk_onecell_data clk_data;
	spinlock_t lock;
};

/**
 * struct ingenic_clock_alias: information about mux clock
 * @id: platform specific id of the clock.
 * @dev_name: name of the device to which this clock belongs.
 * @alias: optional clock alias name to be assigned to this clock.
 */
struct ingenic_clock_alias {
	unsigned int        id;
	const char      *dev_name;
	const char      *alias;
};

#define ALIAS(_id, dname, a)    \
	{                           \
		.id     = _id,              \
		          .dev_name   = dname,            \
		                        .alias      = a,                \
	}

#define MHZ (1000 * 1000)

/**
 * struct ingenic_fixed_rate_clock: information about fixed-rate clock
 * @id: platform specific id of the clock.
 * @name: name of this fixed-rate clock.
 * @parent_name: optional parent clock name.
 * @flags: optional fixed-rate clock flags.
 * @fixed-rate: fixed clock rate of this clock.
 */
struct ingenic_fixed_rate_clock {
	unsigned int        id;
	char            *name;
	const char      *parent_name;
	unsigned long       flags;
	unsigned long       fixed_rate;
};

#define FRATE(_id, cname, pname, f, frate)      \
	{                       \
		.id     = _id,          \
		          .name       = cname,        \
		                        .parent_name    = pname,        \
		                            .flags      = (f | CLK_IGNORE_UNUSED),          \
		                                .fixed_rate = frate,        \
	}

/*
 * struct ingenic_fixed_factor_clock: information about fixed-factor clock
 * @id: platform specific id of the clock.
 * @name: name of this fixed-factor clock.
 * @parent_name: parent clock name.
 * @mult: fixed multiplication factor.
 * @div: fixed division factor.
 * @flags: optional fixed-factor clock flags.
 */
struct ingenic_fixed_factor_clock {
	unsigned int        id;
	char            *name;
	const char      *parent_name;
	unsigned long       mult;
	unsigned long       div;
	unsigned long       flags;
};

#define FFACTOR(_id, cname, pname, m, d, f)     \
	{                       \
		.id     = _id,          \
		          .name       = cname,        \
		                        .parent_name    = pname,        \
		                            .mult       = m,            \
		                                .div        = d,            \
		                                    .flags      = f,            \
	}

/**
 * struct ingenic_mux_clock: information about mux clock
 * @id: platform specific id of the clock.
 * @dev_name: name of the device to which this clock belongs.
 * @name: name of this mux clock.
 * @table: mux table in regs.
 * @parent_names: array of pointer to parent clock names.
 * @num_parents: number of parents listed in @parent_names.
 * @flags: optional flags for basic clock.
 * @offset: offset of the register for configuring the mux.
 * @shift: starting bit location of the mux control bit-field in @reg.
 * @width: width of the mux control bit-field in @reg.
 * @mux_flags: flags for mux-type clock.
 * @alias: optional clock alias name to be assigned to this clock.
 */
struct ingenic_mux_clock {
	unsigned int        id;
	const char      *dev_name;
	const char      *name;
	unsigned int        *table;
	const char      **parent_names;
	u8          num_parents;
	unsigned long       flags;
	unsigned long       offset;
	u8          shift;
	u8          width;
	u8          mux_flags;
	const char      *alias;
};

#define __MUX(_id, dname, cname, tb, pnames, o, s, w, f, mf, a) \
	{                           \
		.id     = _id,              \
		          .dev_name   = dname,            \
		                        .name       = cname,            \
		                                      .table      = tb,               \
		                                          .parent_names   = pnames,           \
		                                              .num_parents    = ARRAY_SIZE(pnames),       \
		                                                  .flags      = f,                \
		                                                      .offset     = o,                \
		                                                          .shift      = s,                \
		                                                              .width      = w,                \
		                                                                  .mux_flags  = mf,               \
		                                                                      .alias      = a,                \
	}

#define MUX(_id, cname, tb, pnames, o, s, w, f)         \
	__MUX(_id, NULL, cname, tb, pnames, o, s, w, f, 0, cname)

#define MUX_A(_id, cname, tb, pnames, o, s, w, a)           \
	__MUX(_id, NULL, cname, tb, pnames, o, s, w, 0, 0, a)

#define MUX_F(_id, cname, tb, pnames, o, s, w, f, mf)       \
	__MUX(_id, NULL, cname, tb, pnames, o, s, w, f, mf, NULL)

#define MUX_FA(_id, cname, tb, pnames, o, s, w, f, mf, a)       \
	__MUX(_id, NULL, cname, tb, pnames, o, s, w, f, mf, a)

/**
 * @id: platform specific id of the clock.
 * struct ingenic_div_clock: information about div clock
 * @dev_name: name of the device to which this clock belongs.
 * @name: name of this div clock.
 * @parent_name: name of the parent clock.
 * @flags: optional flags for basic clock.
 * @offset: offset of the register for configuring the div.
 * @shift: starting bit location of the div control bit-field in @reg.
 * @busy_offset: offset of the register waiting for div stable.
 * @busy_shift: bit-field of the busy bit in busy_reg, by finding the busy shift, we can also find the ce, and stop bit.
 * @en_shift: points to some gateble clk in div cfg, ugly we gate clock in div clocks.
 * @div_flags: divider flags.
 * @alias: optional clock alias name to be assigned to this clock.
 * @table: clk div table if possible.
 */
struct ingenic_div_clock {
	unsigned int        id;
	const char      *dev_name;
	const char      *name;
	const char      *parent_name;
	unsigned long       flags;
	unsigned long       offset;
	u8          shift;
	u8          width;

	unsigned long       busy_offset;
	int         busy_shift;
	int         en_shift;
	int         stop_shift;

	int             div_flags;
	const char      *alias;
	struct clk_div_table    *table;
};

#define __DIV(_id, dname, cname, pname, o, s, w, bs, e, st, f, df, a, t)    \
	{                           \
		.id     = _id,              \
		          .dev_name   = dname,            \
		                        .name       = cname,            \
		                                      .parent_name    = pname,            \
		                                          .flags      = (f | CLK_IGNORE_UNUSED),              \
		                                              .offset     = o,                \
		                                                  .shift      = s,                \
		                                                      .width      = w,                \
		                                                          .div_flags  = df,               \
		                                                              .busy_shift = bs,               \
		                                                                  .en_shift   = e,                \
		                                                                      .stop_shift = st,               \
		                                                                          .alias      = a,                \
		                                                                              .table      = t,                \
	}

#define DIV(_id, cname, pname, o, w, df, t)             \
	__DIV(_id, NULL, cname, pname, o, 0, w, 28, 29, 27, 0, df, cname, t)

#define DIV_EN(_id, cname, pname, o, s, w, bs, e)               \
	__DIV(_id, NULL, cname, pname, o, s, w, bs, e, 0, 0, NULL, NULL)

/**
 * @id: platform specific id of the clock.
 * struct ingenic_bus_clock: information about div clock
 * @dev_name: name of the device to which this clock belongs.
 * @name: name of this div clock.
 * @parent_name: name of the parent clock.
 * @flags: optional flags for basic clock.
 * @cfg_offset: offset of the register for configuring the div.
 * @div_shift: starting bit location of the div control bit-field in @reg.
 * @div_width: bit width of the div field.
 * @busy_offset: offset of the register waiting for div stable.
 * @busy_shift: bit-field of the busy bit in busy_reg, by finding the busy shift, we can also find the ce, and stop bit.
 * @en_shift: points to some gateble clk in div cfg, ugly we gate clock in div clocks.
 * @div_flags: flags for div-type clock, For example, DIV_NO_BUSY.
 * @alias: optional clock alias name to be assigned to this clock.
 * @table: clk div table if possible.
 */

struct ingenic_bus_clock {
	unsigned int        id;
	const char      *dev_name;
	const char      *name;
	const char      *parent_name;
	unsigned long       flags;
	unsigned long       cfg_offset;
	u8          div_shift1;
	u8          div_width1;
	u8          div_shift2;
	u8          div_width2;
	int             ce_shift;

	int             busy_offset;
	int         busy_shift;

	int             div_flags;
	int             div_flags_2;
	const char      *alias;
	struct clk_div_table    *table;
};

#define __BUS_DIV(_id, dname, cname, pname, o, s1, w1, s2, w2, bo, bs, ce, f, df, df2, a, t)    \
	{                           \
		.id     = _id,              \
		          .dev_name   = dname,            \
		                        .name       = cname,            \
		                                      .parent_name    = pname,            \
		                                          .flags      = (f | CLK_IGNORE_UNUSED),  \
		                                              .cfg_offset = o,                \
		                                                  .div_shift1 = s1,               \
		                                                      .div_width1 = w1,               \
		                                                          .div_shift2 = s2,               \
		                                                              .div_width2 = w2,               \
		                                                                  .ce_shift   = ce,               \
		                                                                      .busy_offset    = bo,               \
		                                                                          .busy_shift = bs,               \
		                                                                              .div_flags  = df,               \
		                                                                                  .alias      = a,                \
		                                                                                      .table      = t,                \
		                                                                                          .div_flags_2    = df2,              \
	}

#define BUS_DIV(_id, cname, pname, o, s1, w1, s2, w2, bo, bs, ce, df2)              \
	__BUS_DIV(_id, NULL, cname, pname, o, s1, w1, s2, w2, bo, bs, ce, CLK_GET_RATE_NOCACHE, 0, df2, cname, NULL)

struct ingenic_fra_div_clock {
	unsigned int        id;
	const char      *dev_name;
	const char      *name;
	const char      *parent_name;
	unsigned long       flags;

	unsigned long       offset;
	u8          mshift;
	u8          mwidth;
	u8          nshift;
	u8          nwidth;

	int             div_flags;
	const char      *alias;
};

#define __FRA_DIV(_id, dname, cname, pname, f, o, ms, mw, ns, nw, df, a)    \
	{                                   \
		.id     = _id,      \
		          .dev_name   = dname,    \
		                        .name       = cname,    \
		                                      .parent_name    = pname,    \
		                                          .flags      = f,        \
		                                              .offset     = o,        \
		                                                  .mshift     = ms,       \
		                                                      .mwidth     = mw,       \
		                                                          .nshift     = ns,       \
		                                                              .nwidth     = nw,       \
		                                                                  .div_flags  = df,       \
		                                                                      .alias      = a,        \
	}

#define FRA_DIV(_id, cname, pname, o, ms, mw, ns, nw)   \
	__FRA_DIV(_id, NULL, cname, pname, 0, o, ms, mw, ns, nw, 0, cname)

/**
 * struct ingenic_gate_clock: information about gate clock
 * @id: platform specific id of the clock.
 * @dev_name: name of the device to which this clock belongs.
 * @name: name of this gate clock.
 * @parent_name: name of the parent clock.
 * @pwc_name: name of the power control clk under this clock gate.
 * @flags: optional flags for basic clock.
 * @offset: offset of the register for configuring the gate.
 * @bit_idx: bit index of the gate control bit-field in @reg.
 * @sram_offset: offset of the corresponding sram contrl reg.
 * @sram_shift: shift for the sram control bit idx in sram_reg.
 * @gate_flags: flags for gate-type clock.
 * @alias: optional clock alias name to be assigned to this clock.
 * @gate_flags: flags for gate clock.
 * @alias: alias name for this clock.
 */
struct ingenic_gate_clock {
	unsigned int        id;
	const char      *dev_name;
	const char      *name;
	const char      *parent_name;

	unsigned long       flags;
	unsigned long       offset;
	u8          bit_idx;
	int         sram_offset;
	int         sram_shift;
	u8          gate_flags;
	const char      *alias;

};

#define __GATE(_id, dname, cname, pname, o, b, f, so, sps, gf, a)   \
	{                           \
		.id     = _id,              \
		          .dev_name   = dname,            \
		                        .name       = cname,            \
		                                      .parent_name    = pname,            \
		                                          .flags      = f,                \
		                                              .offset     = o,                \
		                                                  .bit_idx    = b,                \
		                                                      .sram_offset    = so,               \
		                                                          .sram_shift     = sps,              \
		                                                              .gate_flags = gf,               \
		                                                                  .alias      = a,                \
	}

#define GATE(_id, cname, pname, o, b, f, gf)            \
	__GATE(_id, NULL, cname, pname, o, b, f, -1, -1, gf, cname)

#define GATE_SRAM(_id, cname, pname, o, b, f, gf, so, sps)          \
	__GATE(_id, NULL, cname, pname, NULL, o, b, f, so, sps, gf, cname)

/**
 * struct ingenic_power_clock: information about power clock
 * @id: platform specific id of the clock.
 * @dev_name: name of the device to which this clock belongs.
 * @name: name of this gate clock.
 * @parent_name: name of the parent clock.
 * @flags: optional flags for basic clock.
 * @offset: offset of the register for configuring the gate.
 * @ctrl_bit: bit index of the power control bit-field in @reg.
 * @wait_bit: bit index of the power status bit-field in @reg.
 * @delay_ms: wait for ms time stable @reg.
 * @gate_flags: flags for gate-type clock.
 * @alias: optional clock alias name to be assigned to this clock.
 * @clk_flags: ingneic special flags.
 */
struct ingenic_gate_power {
	unsigned int        id;
	const char      *dev_name;
	const char      *name;
	const char      *parent_name;

	unsigned long       flags;
	unsigned long       offset;
	u8          ctrl_bit;
	u8          wait_bit;
	u8          gate_flags;
	const char      *alias;
	unsigned long       power_flags;
};

#define __POWER(_id, dname, cname, pname, f, o, cb, wb, gf, a, pf)  \
	{                           \
		.id     = _id,              \
		          .dev_name   = dname,            \
		                        .name       = cname,            \
		                                      .parent_name    = pname,            \
		                                          .flags      = f,                \
		                                              .offset     = o,                \
		                                                  .ctrl_bit   = cb,               \
		                                                      .wait_bit   = wb,               \
		                                                          .gate_flags = gf,               \
		                                                              .alias      = a,                \
		                                                                  .power_flags    = pf,               \
	}

#define POWER(_id, cname, pname, o, cb, wb, f, gf, pf)          \
	__POWER(_id, NULL, cname, pname, f, o, cb, wb, gf, cname, pf)

#define PNAME(x) static const char *x[] __initdata

#define REG_SLEEP(o, c, b, f)   \
	{   \
		.offset = o,    \
		          .ce = c,    \
		                .busy = b,  \
		                        .flags = f, \
	}

/**
 * struct ingenic_clk_reg_sleep: register dump of clock controller registers.
 * @offset: clock register offset from the controller base address.
 * @value: the value to be register at offset.
 */
struct ingenic_clk_reg_sleep {
	u32 offset;
	u32 value;
	u32 ce;
	u32 busy;
	u32 flags;
};

/**
 * struct ingenic_pll_clock: information about pll clock
 * @id: platform specific id of the clock.
 * @dev_name: name of the device to which this clock belongs.
 * @name: name of this pll clock.
 * @parent_name: name of the parent clock.
 * @flags: optional flags for basic clock.
 * @con_offset: offset of the register for configuring the PLL.
 * @lock_offset: offset of the register for locking the PLL.
 * @type: Type of PLL to be registered.
 * @alias: optional clock alias name to be assigned to this clock.
 */
struct ingenic_pll_clock {
	unsigned int        id;
	const char      *dev_name;
	const char      *name;
	const char      *parent_name;

	unsigned long       flags;
	int         con_offset;
	int         lock_offset;
	const char              *alias;
	struct ingenic_pll_hwdesc *hwdesc;
	struct ingenic_pll_rate_table *rate_table;
};

#define PLL(_id, _name, _parent_name, _hwdesc, _rtable) \
	{   \
		.id = _id,  \
		      .dev_name = _name,  \
		                  .name = _name,  \
		                          .parent_name = _parent_name,    \
		                              .hwdesc = _hwdesc,  \
		                                        .rate_table = _rtable,  \
	}

struct ingenic_clock_reg_cache {
	struct list_head node;
	void __iomem *reg_base;
	struct ingenic_clk_reg_sleep *reg_sleep;
	unsigned int nr_reg_sleep;
};

struct ingenic_cmu_info {
	/* list of pll clocks and respective count */
	struct ingenic_pll_clock *pll_clks;
	unsigned int nr_pll_clks;
	/* list of mux clocks and respective count */
	struct ingenic_mux_clock *mux_clks;
	unsigned int nr_mux_clks;
	/* list of div clocks and respective count */
	struct ingenic_div_clock *div_clks;
	unsigned int nr_div_clks;
	/* list of gate clocks and respective count */
	struct ingenic_gate_clock *gate_clks;
	unsigned int nr_gate_clks;
	/* list of fixed clocks and respective count */
	struct ingenic_fixed_rate_clock *fixed_clks;
	unsigned int nr_fixed_clks;
	/* list of fixed factor clocks and respective count */
	struct ingenic_fixed_factor_clock *fixed_factor_clks;
	unsigned int nr_fixed_factor_clks;
	/* total number of clocks with IDs assigned*/
	unsigned int nr_clk_ids;

	/* list and number of clocks registers */
	unsigned long *clk_regs;
	unsigned int nr_clk_regs;
};

struct ingenic_cpm_info {
	struct ingenic_power_clock *pwc_clks;
	unsigned int nr_pwc_clks;

};

extern struct ingenic_clk_provider *__init ingenic_clk_init(
    struct device_node *np, void __iomem *base,
    unsigned long nr_clks);
extern void __init ingenic_clk_of_add_provider(struct device_node *np,
        struct ingenic_clk_provider *ctx);
extern void __init ingenic_clk_of_register_fixed_ext(
    struct ingenic_clk_provider *ctx,
    struct ingenic_fixed_rate_clock *fixed_rate_clk,
    unsigned int nr_fixed_rate_clk,
    const struct of_device_id *clk_matches);

extern void ingenic_clk_add_lookup(struct ingenic_clk_provider *ctx,
                                   struct clk *clk, unsigned int id);

extern void __init ingenic_clk_register_alias(struct ingenic_clk_provider *ctx,
        const struct ingenic_clock_alias *list,
        unsigned int nr_clk);
extern void __init ingenic_clk_register_fixed_rate(
    struct ingenic_clk_provider *ctx,
    const struct ingenic_fixed_rate_clock *clk_list,
    unsigned int nr_clk);
extern void __init ingenic_clk_register_fixed_factor(
    struct ingenic_clk_provider *ctx,
    const struct ingenic_fixed_factor_clock *list,
    unsigned int nr_clk);
extern void __init ingenic_clk_register_mux(struct ingenic_clk_provider *ctx,
        const struct ingenic_mux_clock *clk_list,
        unsigned int nr_clk);
extern void __init ingenic_clk_register_cgu_div(struct ingenic_clk_provider *ctx,
        const struct ingenic_div_clock *clk_list,
        unsigned int nr_clk);

extern void __init ingenic_clk_register_gate(struct ingenic_clk_provider *ctx,
        const struct ingenic_gate_clock *list,
        unsigned int nr_clk);

extern void __init ingenic_power_register_gate(struct ingenic_clk_provider *ctx,
        const struct ingenic_gate_power *list,
        unsigned int nr_clk);

extern void __init ingenic_clk_register_pll(struct ingenic_clk_provider *ctx,
        const struct ingenic_pll_clock *pll_list,
        unsigned int nr_clk, void __iomem *base);

extern struct ingenic_clk_provider __init *ingenic_cmu_register_one(
    struct device_node *,
    struct ingenic_cmu_info *);

extern unsigned long _get_rate(const char *clk_name);

extern void ingenic_clk_save(void __iomem *base,
                             struct ingenic_clk_reg_sleep *rd,
                             unsigned int num_regs);
extern void ingenic_clk_restore(void __iomem *base,
                                struct ingenic_clk_reg_sleep *rd,
                                unsigned int num_regs);

extern void __init ingenic_clk_register_bus_div(struct ingenic_clk_provider *ctx,
        const struct ingenic_bus_clock *list,
        unsigned int nr_clk);

extern void __init ingenic_clk_register_fra_div(struct ingenic_clk_provider *ctx,
        const struct ingenic_fra_div_clock *list,
        unsigned int nr_clk);

extern void __init ingenic_clk_sleep_init(void __iomem *reg_base,
        struct ingenic_clk_reg_sleep *rdump,
        unsigned long nr_rdump);

void ingenic_clk_of_dump(struct ingenic_clk_provider *ctx);
#endif /* __INGENIC_CLK_H */
