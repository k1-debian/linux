#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include "clk.h"

struct clk_cgu_divider {
	struct clk_divider div;
	const struct clk_ops *div_ops;
	int busy_shift;
	int en_shift;
	int stop_shift;
	spinlock_t *lock;
};

static int clk_cgu_wait(void __iomem *reg, u8 shift)
{
	unsigned int timeout = 0xfffff;

	while (((readl(reg) >> shift) & 1) && --timeout);
	if (!timeout) {
		printk("WARNING : why cannot wait cgu stable ???\n");
		return -ETIMEDOUT;
	} else {
		return 0;
	}

}

static int cgu_divider_enable(struct clk_cgu_divider *cgu_div)
{
	int ret;
	unsigned int val;
	int ce, stop;

	val = readl(cgu_div->div.reg);

	ce = cgu_div->en_shift;
	stop = cgu_div->stop_shift;

	val |= (1 << ce);
	val &= ~(1 << stop);
	writel(val, cgu_div->div.reg);

	ret = clk_cgu_wait(cgu_div->div.reg, cgu_div->busy_shift);
	if (ret < 0) {
		printk("wait cgu stable timeout!\n");
	}

	val &= ~(1 << ce);
	writel(val, cgu_div->div.reg);

	return ret;
}

static void cgu_divider_disable(struct clk_cgu_divider *cgu_div)
{
	unsigned int val;
	int ce, stop;

	val = readl(cgu_div->div.reg);

	ce = cgu_div->en_shift;
	stop = cgu_div->stop_shift;

	val |= (1 << ce);
	val |= (1 << stop);
	writel(val, cgu_div->div.reg);

	val &= ~(1 << ce);
	writel(val, cgu_div->div.reg);

}

static inline struct clk_cgu_divider *to_clk_cgu_divider(struct clk_hw *hw)
{
	struct clk_divider *div = container_of(hw, struct clk_divider, hw);

	return container_of(div, struct clk_cgu_divider, div);
}

static unsigned long clk_cgu_divider_recalc_rate(struct clk_hw *hw,
        unsigned long parent_rate)
{
	struct clk_cgu_divider *cgu_div = to_clk_cgu_divider(hw);

	return cgu_div->div_ops->recalc_rate(&cgu_div->div.hw, parent_rate);
}

static long clk_cgu_divider_round_rate(struct clk_hw *hw, unsigned long rate,
                                       unsigned long *prate)
{
	struct clk_cgu_divider *cgu_div = to_clk_cgu_divider(hw);

	return  cgu_div->div_ops->round_rate(&cgu_div->div.hw, rate, prate);
}

static int clk_cgu_divider_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct clk_cgu_divider *cgu_div = to_clk_cgu_divider(hw);
	int ret;
	unsigned long flags = 0;

	if (cgu_div->lock) {
		spin_lock_irqsave(cgu_div->lock, flags);
	}

	ret = cgu_div->div_ops->set_rate(&cgu_div->div.hw, rate, parent_rate);

	cgu_divider_enable(cgu_div);

	if (cgu_div->lock) {
		spin_unlock_irqrestore(cgu_div->lock, flags);
	}

	return ret;
}

static void clk_cgu_divider_disable(struct clk_hw *hw)
{
	struct clk_cgu_divider *cgu_div = to_clk_cgu_divider(hw);
	unsigned long flags = 0;

	if (cgu_div->lock) {
		spin_lock_irqsave(cgu_div->lock, flags);
	}

	cgu_divider_disable(cgu_div);

	if (cgu_div->lock) {
		spin_unlock_irqrestore(cgu_div->lock, flags);
	}

}

static int clk_cgu_divider_enable(struct clk_hw *hw)
{
	struct clk_cgu_divider *cgu_div = to_clk_cgu_divider(hw);
	unsigned long flags = 0;

	if (cgu_div->lock) {
		spin_lock_irqsave(cgu_div->lock, flags);
	}

	cgu_divider_enable(cgu_div);

	if (cgu_div->lock) {
		spin_unlock_irqrestore(cgu_div->lock, flags);
	}

	return 0;
}

static struct clk_ops clk_cgu_divider_ops = {
	.recalc_rate = clk_cgu_divider_recalc_rate,
	.round_rate = clk_cgu_divider_round_rate,
	.set_rate = clk_cgu_divider_set_rate,
	.enable = clk_cgu_divider_enable,
	.disable = clk_cgu_divider_disable,
};

struct clk *_register_cgu_divider(struct device *dev, const char *name,
                                  const char *parent_name, unsigned long flags,
                                  void __iomem *reg, u8 shift, u8 width, u8 busy_shift,
                                  int en_shift, u8 stop_shift, u8 clk_divider_flags,
                                  const struct clk_div_table *table,
                                  spinlock_t *lock)
{
	struct clk_cgu_divider *cgu_div;
	struct clk *clk;
	struct clk_init_data init;

	cgu_div = kzalloc(sizeof(*cgu_div), GFP_KERNEL);
	if (!cgu_div) {
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &clk_cgu_divider_ops;
	init.flags = flags;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	cgu_div->busy_shift = busy_shift;
	cgu_div->en_shift = en_shift;
	cgu_div->stop_shift = stop_shift;
	cgu_div->lock = lock;

	cgu_div->div.reg = reg;
	cgu_div->div.shift = shift;
	cgu_div->div.width = width;
	//cgu_div->div.lock = lock;
	cgu_div->div.lock = NULL;   /* keep common block unlocked. add lock in this file */
	cgu_div->div.table = table;
	cgu_div->div.flags = clk_divider_flags;
	cgu_div->div_ops = &clk_divider_ops;

	cgu_div->div.hw.init = &init;

	clk = clk_register(dev, &cgu_div->div.hw);
	if (IS_ERR(clk)) {
		kfree(cgu_div);
	}

	return clk;
}

/**
 * clk_register_divider_table - register a table based divider clock with
 * the clock framework
 * @dev: device registering this clock
 * @name: name of this clock
 * @parent_name: name of clock's parent
 * @flags: framework-specific flags
 * @reg: register address to adjust divider
 * @shift: number of bits to shift the bitfield
 * @busy_shift: bit index of busy in busy register.
 * @width: width of the bitfield
 * @clk_divider_flags: divider-specific flags for this clock
 * @table: array of divider/value pairs ending with a div set to 0
 * @lock: shared register lock for this clock
 */
struct clk *clk_register_cgu_divider_table(struct device *dev, const char *name,
        const char *parent_name, unsigned long flags,
        void __iomem *reg, u8 shift, u8 width, u8 busy_shift,
        int en_shift, u8 stop_shift, u8 clk_divider_flags,
        const struct clk_div_table *table,
        spinlock_t *lock)
{
	return _register_cgu_divider(dev, name, parent_name, flags, reg, shift,
	                             width, busy_shift, en_shift, stop_shift, clk_divider_flags, table, lock);
}

/**
 * clk_register_divider - register a divider clock with the clock framework
 * @dev: device registering this clock
 * @name: name of this clock
 * @parent_name: name of clock's parent
 * @flags: framework-specific flags
 * @reg: register address to adjust divider
 * @shift: number of bits to shift the bitfield
 * @width: width of the bitfield
 * @busy_shift: bit index of busy in busy register.
 * @clk_divider_flags: divider-specific flags for this clock
 * @lock: shared register lock for this clock
 */
struct clk *clk_register_cgu_divider(struct device *dev, const char *name,
                                     const char *parent_name, unsigned long flags,
                                     void __iomem *reg, u8 shift, u8 width, u8 busy_shift,
                                     int en_shift, u8 stop_shift,
                                     u8 clk_divider_flags, spinlock_t *lock)
{
	return _register_cgu_divider(dev, name, parent_name, flags, reg, shift, width, busy_shift, en_shift, stop_shift, clk_divider_flags, NULL, lock);
}
