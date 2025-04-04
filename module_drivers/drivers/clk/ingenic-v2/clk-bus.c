#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include "clk.h"

static int clk_bus_wait(void __iomem *reg, u8 shift)
{
	unsigned int timeout = 0xfffff;

	while (((readl(reg) >> shift) & 1) && --timeout);
	if (!timeout) {
		printk("WARNING : why cannot wait bus stable ???\n");
		return -EIO;
	} else {
		return 0;
	}
}

struct clk_bus_divider {
	struct clk_divider div;
	const struct clk_ops *div_ops;

	void __iomem *busy_reg;

	int busy_shift;
	int ce_shift;

	int shift1;
	int width1;
	int shift2;
	int width2;

	int div_flags;

	spinlock_t *lock;
};

//static inline struct clk_bus *to_clk_bus_divider(struct clk_hw *hw)
static inline struct clk_bus_divider *to_clk_bus_divider(struct clk_hw *hw)
{
	struct clk_divider *div = container_of(hw, struct clk_divider, hw);

	return container_of(div, struct clk_bus_divider, div);
}

static unsigned long clk_bus_divider_recalc_rate(struct clk_hw *hw,
        unsigned long parent_rate)
{
	struct clk_bus_divider *bus_div = to_clk_bus_divider(hw);
	struct clk_divider *divider = &bus_div->div;

	divider->shift = bus_div->shift1;
	divider->width = bus_div->width1;

	return bus_div->div_ops->recalc_rate(&bus_div->div.hw, parent_rate);
}

static long clk_bus_divider_round_rate(struct clk_hw *hw, unsigned long rate,
                                       unsigned long *prate)
{
	struct clk_bus_divider *bus_div = to_clk_bus_divider(hw);

	return bus_div->div_ops->round_rate(&bus_div->div.hw, rate, prate);
}

static int clk_bus_divider_set_rate(struct clk_hw *hw, unsigned long rate,
                                    unsigned long parent_rate)
{
	struct clk_bus_divider *bus_div = to_clk_bus_divider(hw);
	struct clk_divider *divider = &bus_div->div;
	int ret;
	unsigned long flags = 0;
	unsigned int val;
	int ce = bus_div->ce_shift;

	if (bus_div->lock) {
		spin_lock_irqsave(bus_div->lock, flags);
	}

	/* set bus rate . */
	if (bus_div->div_flags == BUS_DIV_SELF) {
		ret = bus_div->div_ops->set_rate(&bus_div->div.hw, rate, parent_rate);
	} else if (bus_div->div_flags == BUS_DIV_ONE) {

		divider->shift = bus_div->shift1;
		divider->width = bus_div->width1;
		ret = bus_div->div_ops->set_rate(&bus_div->div.hw, rate, parent_rate);

		divider->shift = bus_div->shift2;
		divider->width = bus_div->width2;
		ret = bus_div->div_ops->set_rate(&bus_div->div.hw, rate, parent_rate);
	} else if (bus_div->div_flags == BUS_DIV_TWO) {

		divider->shift = bus_div->shift1;
		divider->width = bus_div->width1;
		ret = bus_div->div_ops->set_rate(&bus_div->div.hw, rate, parent_rate);

		divider->shift = bus_div->shift2;
		divider->width = bus_div->width2;
		ret = bus_div->div_ops->set_rate(&bus_div->div.hw, rate / 2, parent_rate);
	}

	/* ce  */
	if (ce > 0) {
		val = readl(bus_div->div.reg);
		val |= (1 << ce);
		writel(val, bus_div->div.reg);

		ret = clk_bus_wait(bus_div->busy_reg, bus_div->busy_shift);
		if (ret < 0) {
			pr_err("wait bus clk: (%s)  stable timeout!\n", __clk_get_name(hw->clk));
		}

		val &= ~(1 << ce);
		writel(val, bus_div->div.reg);
	}
	if (bus_div->lock) {
		spin_unlock_irqrestore(bus_div->lock, flags);
	}

	return ret;
}

static struct clk_ops clk_bus_divider_ops = {
	.recalc_rate = clk_bus_divider_recalc_rate,
	.round_rate = clk_bus_divider_round_rate,
	.set_rate = clk_bus_divider_set_rate,
};

struct clk *_register_bus_divider(struct device *dev, const char *name,
                                  const char *parent_name, unsigned long flags,
                                  void __iomem *reg, u8 shift1, u8 width1, u8 shift2, u8 width2,
                                  void __iomem *busy_reg, u8 busy_shift,
                                  int ce_shift,
                                  u8 clk_divider_flags, u8 div_flags,
                                  const struct clk_div_table *table,
                                  spinlock_t *lock)
{
	struct clk_bus_divider *bus_div;
	struct clk *clk;
	struct clk_init_data init;

	bus_div = kzalloc(sizeof(*bus_div), GFP_KERNEL);
	if (!bus_div) {
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &clk_bus_divider_ops;
	init.flags = flags;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	bus_div->busy_reg = busy_reg;
	bus_div->busy_shift = busy_shift;
	bus_div->ce_shift = ce_shift;
	bus_div->lock = lock;

	bus_div->div.reg = reg;
	bus_div->div.shift = shift1;
	bus_div->div.width = width1;
	bus_div->shift1 = shift1;
	bus_div->width1 = width1;
	bus_div->shift2 = shift2;
	bus_div->width2 = width2;
	//bus_div->div.lock = lock;
	bus_div->div.lock = NULL;   /* keep common block unlocked. add lock in this file */
	bus_div->div.table = table;
	bus_div->div.flags = clk_divider_flags ;
	bus_div->div_ops = &clk_divider_ops;
	bus_div->div_flags = div_flags ;

	bus_div->div.hw.init = &init;

	clk = clk_register(dev, &bus_div->div.hw);
	if (IS_ERR(clk)) {
		kfree(bus_div);
	}

	return clk;
}

/**
 * clk_register_bus_divider_table - register a table based divider clock with
 * the clock framework
 * @dev: device registering this clock
 * @name: name of this clock
 * @parent_name: name of clock's parent
 * @flags: framework-specific flags
 * @reg: register address to adjust divider
 * @shift: number of bits to shift the bitfield
 * @busy_reg: register address of busy bit waiting.
 * @busy_shift: bit index of busy in busy register.
 * @width: width of the bitfield
 * @clk_divider_flags: divider-specific flags for this clock
 * @table: array of divider/value pairs ending with a div set to 0
 * @lock: shared register lock for this clock
 */
struct clk *clk_register_bus_divider_table(struct device *dev, const char *name,
        const char *parent_name, unsigned long flags,
        void __iomem *reg, u8 shift1, u8 width1, u8 shift2, u8 width2,
        void __iomem *busy_reg, u8 busy_shift, int ce_shift,
        u8 clk_divider_flags, u8 div_flags,
        const struct clk_div_table *table,
        spinlock_t *lock)
{
	return _register_bus_divider(dev, name, parent_name, flags, reg, shift1,
	                             width1, shift2, width2, busy_reg, busy_shift, ce_shift, clk_divider_flags, div_flags, table, lock);
}

/**
 * clk_register_bus_divider - register a divider clock with the clock framework
 * @dev: device registering this clock
 * @name: name of this clock
 * @parent_name: name of clock's parent
 * @flags: framework-specific flags
 * @reg: register address to adjust divider
 * @shift: number of bits to shift the bitfield
 * @width: width of the bitfield
 * @busy_reg: register address of busy bit waiting.
 * @busy_shift: bit index of busy in busy register.
 * @clk_divider_flags: divider-specific flags for this clock
 * @lock: shared register lock for this clock
 */
struct clk *clk_register_bus_divider(struct device *dev, const char *name,
                                     const char *parent_name, unsigned long flags,
                                     void __iomem *reg, u8 shift1, u8 width1, u8 shift2, u8 width2,
                                     void __iomem *busy_reg, u8 busy_shift, int ce_shift,
                                     u8 clk_divider_flags, u8 div_flags, spinlock_t *lock)
{
	return _register_bus_divider(dev, name, parent_name, flags, reg, shift1, width1, shift2, width2, busy_reg, busy_shift, ce_shift, clk_divider_flags, div_flags, NULL, lock);
}
