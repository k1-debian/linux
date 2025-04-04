#include <linux/slab.h>
#include <linux/clkdev.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/of_address.h>
#include <linux/syscore_ops.h>

#include "clk.h"

static LIST_HEAD(clock_reg_cache_list);

void ingenic_clk_save(void __iomem *base,
                      struct ingenic_clk_reg_sleep *rd,
                      unsigned int num_regs)
{
	for (; num_regs > 0; --num_regs, ++rd) {
		rd->value = readl(base + rd->offset);
	}
}

void ingenic_clk_restore(void __iomem *base,
                         struct ingenic_clk_reg_sleep *rd,
                         unsigned int num_regs)
{
	unsigned int timeout = 0xffff;
	for (; num_regs > 0; --num_regs, ++rd) {

		if (!rd->ce && !rd->busy) {
			writel(rd->value, base + rd->offset);
		} else {
			writel(rd->value | 1 << rd->ce, base + rd->offset);

			timeout = 0xffff;

#if 0
			while (--timeout && readl(base + rd->offset) & (1 << rd->busy));

			if (!timeout) {
				printk("Error restore clk regs: %x\n", rd->offset);
			}
#endif
		}
	}
}

/* setup the essentials required to support clock lookup using ccf */
struct ingenic_clk_provider *__init ingenic_clk_init(struct device_node *np,
        void __iomem *base, unsigned long nr_clks)
{
	struct ingenic_clk_provider *ctx;
	struct clk **clk_table;
	int i;

	ctx = kzalloc(sizeof(struct ingenic_clk_provider), GFP_KERNEL);
	if (!ctx) {
		panic("could not allocate clock provider context.\n");
	}

	clk_table = kcalloc(nr_clks, sizeof(struct clk *), GFP_KERNEL);
	if (!clk_table) {
		panic("could not allocate clock lookup table\n");
	}

	for (i = 0; i < nr_clks; ++i) {
		clk_table[i] = ERR_PTR(-ENOENT);
	}

	ctx->reg_base = base;
	ctx->clk_data.clks = clk_table;
	ctx->clk_data.clk_num = nr_clks;
	spin_lock_init(&ctx->lock);

	return ctx;
}

void __init ingenic_clk_of_add_provider(struct device_node *np,
                                        struct ingenic_clk_provider *ctx)
{
	if (np) {
		if (of_clk_add_provider(np, of_clk_src_onecell_get,
		                        &ctx->clk_data)) {
			panic("could not register clk provider\n");
		}
	}
}

void ingenic_clk_of_dump(struct ingenic_clk_provider *ctx)
{
	struct clk_onecell_data *clk_data;
	struct clk *clk;
	int i;

	clk_data = &ctx->clk_data;

	for (i = 0; i < clk_data->clk_num; i++) {

		clk = clk_data->clks[i];
		if (clk != ERR_PTR(-ENOENT)) {
			printk("clk->id: %d clk->name: %s \n", i,  __clk_get_name(clk));
		} else {
			printk("clk->id: %d , clk: %p\n", i, clk);
		}
	}

}

/* add a clock instance to the clock lookup table used for dt based lookup */
void ingenic_clk_add_lookup(struct ingenic_clk_provider *ctx, struct clk *clk,
                            unsigned int id)
{
	if (ctx->clk_data.clks && id) {
		ctx->clk_data.clks[id] = clk;
	}
}

/* register a list of aliases */
void __init ingenic_clk_register_alias(struct ingenic_clk_provider *ctx,
                                       const struct ingenic_clock_alias *list,
                                       unsigned int nr_clk)
{
	struct clk *clk;
	unsigned int idx, ret;

	if (!ctx->clk_data.clks) {
		pr_err("%s: clock table missing\n", __func__);
		return;
	}

	for (idx = 0; idx < nr_clk; idx++, list++) {
		if (!list->id) {
			pr_err("%s: clock id missing for index %d\n", __func__,
			       idx);
			continue;
		}

		clk = ctx->clk_data.clks[list->id];
		if (!clk) {
			pr_err("%s: failed to find clock %d\n", __func__,
			       list->id);
			continue;
		}

		ret = clk_register_clkdev(clk, list->alias, list->dev_name);
		if (ret)
			pr_err("%s: failed to register lookup %s\n",
			       __func__, list->alias);
	}
}

/* register a list of fixed clocks */
void __init ingenic_clk_register_fixed_rate(struct ingenic_clk_provider *ctx,
        const struct ingenic_fixed_rate_clock *list,
        unsigned int nr_clk)
{
	struct clk *clk;
	unsigned int idx, ret;

	for (idx = 0; idx < nr_clk; idx++, list++) {
		clk = clk_register_fixed_rate(NULL, list->name,
		                              list->parent_name, list->flags, list->fixed_rate);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n", __func__,
			       list->name);
			continue;
		}

		ingenic_clk_add_lookup(ctx, clk, list->id);

		ret = clk_register_clkdev(clk, list->name, NULL);
		if (ret)
			pr_err("%s: failed to register clock lookup for %s",
			       __func__, list->name);
	}
}

/* register a list of fixed factor clocks */
void __init ingenic_clk_register_fixed_factor(struct ingenic_clk_provider *ctx,
        const struct ingenic_fixed_factor_clock *list, unsigned int nr_clk)
{
	struct clk *clk;
	unsigned int idx;

	for (idx = 0; idx < nr_clk; idx++, list++) {
		clk = clk_register_fixed_factor(NULL, list->name,
		                                list->parent_name, list->flags, list->mult, list->div);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n", __func__,
			       list->name);
			continue;
		}

		ingenic_clk_add_lookup(ctx, clk, list->id);
	}
}

/* register a list of mux clocks */
void __init ingenic_clk_register_mux(struct ingenic_clk_provider *ctx,
                                     const struct ingenic_mux_clock *list,
                                     unsigned int nr_clk)
{
	struct clk *clk;
	unsigned int idx, ret;

	for (idx = 0; idx < nr_clk; idx++, list++) {
		if (list->table) {
			clk = clk_register_mux_table(NULL, list->name, list->parent_names,
			                             list->num_parents, list->flags,
			                             ctx->reg_base + list->offset,
			                             list->shift, BIT(list->width) - 1, list->mux_flags, list->table, &ctx->lock);
		} else {
			clk = clk_register_mux(NULL, list->name, list->parent_names,
			                       list->num_parents, list->flags,
			                       ctx->reg_base + list->offset,
			                       list->shift, list->width, list->mux_flags, &ctx->lock);

		}
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n", __func__,
			       list->name);
			continue;
		}

		ingenic_clk_add_lookup(ctx, clk, list->id);

		/* register a clock lookup only if a clock alias is specified */
		if (list->alias) {
			ret = clk_register_clkdev(clk, list->alias,
			                          list->dev_name);
			if (ret)
				pr_err("%s: failed to register lookup %s\n",
				       __func__, list->alias);
		}
	}
}

/* register a list of div clocks */
void __init ingenic_clk_register_cgu_div(struct ingenic_clk_provider *ctx,
        const struct ingenic_div_clock *list,
        unsigned int nr_clk)
{
	struct clk *clk;
	unsigned int idx, ret;

	for (idx = 0; idx < nr_clk; idx++, list++) {
		if (list->table)
			clk = clk_register_cgu_divider_table(NULL, list->name,
			                                     list->parent_name, list->flags,
			                                     ctx->reg_base + list->offset,
			                                     list->shift, list->width,
			                                     list->busy_shift, list->en_shift, list->stop_shift,
			                                     list->div_flags, list->table, &ctx->lock);
		else
			clk = clk_register_cgu_divider(NULL, list->name,
			                               list->parent_name, list->flags,
			                               ctx->reg_base + list->offset,
			                               list->shift, list->width,
			                               list->busy_shift, list->en_shift, list->stop_shift,
			                               list->div_flags, &ctx->lock);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n", __func__,
			       list->name);
			continue;
		}

		ingenic_clk_add_lookup(ctx, clk, list->id);

		/* register a clock lookup only if a clock alias is specified */
		if (list->alias) {
			ret = clk_register_clkdev(clk, list->alias,
			                          list->dev_name);
			if (ret)
				pr_err("%s: failed to register lookup %s\n",
				       __func__, list->alias);
		}
	}
}

/* register a list of div clocks */
void __init ingenic_clk_register_bus_div(struct ingenic_clk_provider *ctx,
        const struct ingenic_bus_clock *list,
        unsigned int nr_clk)
{
	struct clk *clk;
	unsigned int idx, ret;

	for (idx = 0; idx < nr_clk; idx++, list++) {
		if (list->table)
			clk = clk_register_bus_divider_table(NULL, list->name,
			                                     list->parent_name, list->flags,
			                                     ctx->reg_base + list->cfg_offset,
			                                     list->div_shift1, list->div_width1,
			                                     list->div_shift2, list->div_width2,
			                                     ctx->reg_base + list->busy_offset,
			                                     list->busy_shift, list->ce_shift,
			                                     list->div_flags, list->div_flags_2,
			                                     list->table, &ctx->lock);
		else
			clk = clk_register_bus_divider(NULL, list->name,
			                               list->parent_name, list->flags,
			                               ctx->reg_base + list->cfg_offset,
			                               list->div_shift1, list->div_width1,
			                               list->div_shift2, list->div_width2,
			                               ctx->reg_base + list->busy_offset,
			                               list->busy_shift, list->ce_shift,
			                               list->div_flags, list->div_flags_2,
			                               &ctx->lock);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n", __func__,
			       list->name);
			continue;
		}

		ingenic_clk_add_lookup(ctx, clk, list->id);

		/* register a clock lookup only if a clock alias is specified */
		if (list->alias) {
			ret = clk_register_clkdev(clk, list->alias,
			                          list->dev_name);
			if (ret)
				pr_err("%s: failed to register lookup %s\n",
				       __func__, list->alias);
		}
	}
}

void __init ingenic_clk_register_fra_div(struct ingenic_clk_provider *ctx,
        const struct ingenic_fra_div_clock *list,
        unsigned int nr_clk)
{
	struct clk *clk;
	unsigned int idx, ret;

	for (idx = 0; idx < nr_clk; idx++, list++) {
		clk = clk_register_fractional_divider(NULL,
		                                      list->name, list->parent_name, list->flags,
		                                      ctx->reg_base + list->offset,
		                                      list->mshift, list->mwidth, list->nshift, list->nwidth,
		                                      list->div_flags, &ctx->lock);

		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n", __func__,
			       list->name);
			continue;
		}

		ingenic_clk_add_lookup(ctx, clk, list->id);

		/* register a clock lookup only if a clock alias is specified */
		if (list->alias) {
			ret = clk_register_clkdev(clk, list->alias,
			                          list->dev_name);
			if (ret) {
				pr_err("%s: failed to register lookup %s\n", __func__, list->alias);
			}
		}
	}
}

/* register a list of gate clocks */
void __init ingenic_clk_register_gate(struct ingenic_clk_provider *ctx,
                                      const struct ingenic_gate_clock *list,
                                      unsigned int nr_clk)
{
	struct clk *clk;
	unsigned int idx, ret;

	for (idx = 0; idx < nr_clk; idx++, list++) {

		clk = clk_register_gate(NULL, list->name, list->parent_name, list->flags,
		                        ctx->reg_base + list->offset, list->bit_idx,
		                        list->gate_flags, &ctx->lock);

		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n", __func__,
			       list->name);
			continue;
		}

		/* register a clock lookup only if a clock alias is specified */
		if (list->alias) {
			ret = clk_register_clkdev(clk, list->alias,
			                          list->dev_name);
			if (ret)
				pr_err("%s: failed to register lookup %s\n",
				       __func__, list->alias);
		}

		ingenic_clk_add_lookup(ctx, clk, list->id);

	}

}

void __init ingenic_power_register_gate(struct ingenic_clk_provider *ctx,
                                        const struct ingenic_gate_power *list,
                                        unsigned int nr_clk)
{
	struct clk *clk;
	unsigned int idx, ret;

	for (idx = 0; idx < nr_clk; idx++, list++) {

		clk = power_register_gate(NULL, list->name, list->parent_name, list->flags,
		                          ctx->reg_base + list->offset, list->ctrl_bit, list->wait_bit,
		                          list->gate_flags, list->power_flags, &ctx->lock);

		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n", __func__,
			       list->name);
			continue;
		}

		/* register a clock lookup only if a clock alias is specified */
		if (list->alias) {
			ret = clk_register_clkdev(clk, list->alias,
			                          list->dev_name);
			if (ret)
				pr_err("%s: failed to register lookup %s\n",
				       __func__, list->alias);
		}

		ingenic_clk_add_lookup(ctx, clk, list->id);

	}

}
/*
 * obtain the clock speed of all external fixed clock sources from device
 * tree and register it
 */
void __init ingenic_clk_of_register_fixed_ext(struct ingenic_clk_provider *ctx,
        struct ingenic_fixed_rate_clock *fixed_rate_clk,
        unsigned int nr_fixed_rate_clk,
        const struct of_device_id *clk_matches)
{
	const struct of_device_id *match;
	struct device_node *clk_np;
	u32 freq;
	u32 index = 0;

	for_each_matching_node_and_match(clk_np, clk_matches, &match) {
		if (of_property_read_u32(clk_np, "clock-frequency", &freq)) {
			continue;
		}
		fixed_rate_clk[index].fixed_rate = freq;
		index++;
	}
	ingenic_clk_register_fixed_rate(ctx, fixed_rate_clk, nr_fixed_rate_clk);
}

/* utility function to get the rate of a specified clock */
unsigned long _get_rate(const char *clk_name)
{
	struct clk *clk;

	clk = __clk_lookup(clk_name);
	if (!clk) {
		pr_err("%s: could not find clock %s\n", __func__, clk_name);
		return 0;
	}

	return clk_get_rate(clk);
}

#ifdef CONFIG_PM
static int ingenic_clk_suspend(void)
{
	struct ingenic_clock_reg_cache *reg_cache;

	list_for_each_entry(reg_cache, &clock_reg_cache_list, node)
	ingenic_clk_save(reg_cache->reg_base, reg_cache->reg_sleep,
	                 reg_cache->nr_reg_sleep);
	return 0;
}

static void ingenic_clk_resume(void)
{
	struct ingenic_clock_reg_cache *reg_cache;

	list_for_each_entry(reg_cache, &clock_reg_cache_list, node)
	ingenic_clk_restore(reg_cache->reg_base, reg_cache->reg_sleep,
	                    reg_cache->nr_reg_sleep);
}

static struct syscore_ops ingenic_clk_syscore_ops = {
	.suspend = ingenic_clk_suspend,
	.resume = ingenic_clk_resume,
};

void __init ingenic_clk_sleep_init(void __iomem *reg_base,
                                   struct ingenic_clk_reg_sleep  *reg_sleep,
                                   unsigned long nr_reg_sleep)
{
	struct ingenic_clock_reg_cache *reg_cache;

	reg_cache = kzalloc(sizeof(struct ingenic_clock_reg_cache),
	                    GFP_KERNEL);
	if (!reg_cache) {
		panic("could not allocate register reg_cache.\n");
	}

	reg_cache->reg_sleep = reg_sleep;
	reg_cache->nr_reg_sleep = nr_reg_sleep;

	if (list_empty(&clock_reg_cache_list)) {
		register_syscore_ops(&ingenic_clk_syscore_ops);
	}

	reg_cache->reg_base = reg_base;
	list_add_tail(&reg_cache->node, &clock_reg_cache_list);
}

#else
void __init ingenic_clk_sleep_init(void __iomem *reg_base,
                                   struct ingenic_clk_reg_sleep  *reg_sleep,
                                   unsigned long nr_reg_sleep)
{

}
#endif
