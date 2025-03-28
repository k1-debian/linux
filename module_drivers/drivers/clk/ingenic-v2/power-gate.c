/*
 * Copyright (C) 2010-2011 Canonical Ltd <jeremy.kerr@canonical.com>
 * Copyright (C) 2011-2012 Mike Turquette, Linaro Ltd <mturquette@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Gated clock implementation
 */

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/string.h>

#include "power-gate.h"

struct power_gate {
	struct clk_hw hw;
	void __iomem    *reg;
	u8              ctrl_bit;
	u8              wait_bit;
	u8              flags;
	unsigned long   power_flags;
	spinlock_t      *lock;

};

/**
 * DOC: basic gatable clock which can gate and ungate it's ouput
 *
 * Traits of this clock:
 * prepare - clk_(un)prepare only ensures parent is (un)prepared
 * enable - clk_enable and clk_disable are functional & control gating
 * rate - inherits rate from parent.  No clk_set_rate support
 * parent - fixed parent.  No clk_set_parent support
 */

#define to_power_gate(_hw) container_of(_hw, struct power_gate, hw)

/*
 * It works on following logic:
 *
 * For enabling clock, enable = 1
 *  set2dis = 1 -> clear bit    -> set = 0
 *  set2dis = 0 -> set bit  -> set = 1
 *
 * For disabling clock, enable = 0
 *  set2dis = 1 -> set bit  -> set = 1
 *  set2dis = 0 -> clear bit    -> set = 0
 *
 * So, result is always: enable xor set2dis.
 */
static int power_gate_is_enabled(struct clk_hw *hw)
{
	u32 reg;
	struct power_gate *gate = to_power_gate(hw);

	reg = readl(gate->reg);

	/* if a set bit disables this clk, flip it before masking */
	if (gate->flags & CLK_GATE_SET_TO_DISABLE) {
		reg ^= BIT(gate->wait_bit);
	}

	reg &= BIT(gate->wait_bit);

	return reg ? 1 : 0;
}

static void power_gate_endisable(struct clk_hw *hw, int enable)
{
	struct power_gate *gate = to_power_gate(hw);
	int set = gate->flags & CLK_GATE_SET_TO_DISABLE ? 1 : 0;
	unsigned long flags;
	u32 reg;

	set ^= enable;

	if (gate->lock) {
		spin_lock_irqsave(gate->lock, flags);
	} else {
		__acquire(gate->lock);
	}

	reg = readl(gate->reg);

	if (set) {
		reg |= BIT(gate->ctrl_bit);
	} else {
		reg &= ~BIT(gate->ctrl_bit);
	}
	writel(reg, gate->reg);

	if (gate->lock) {
		spin_unlock_irqrestore(gate->lock, flags);
	} else {
		__release(gate->lock);
	}
}

static int power_gate_enable(struct clk_hw *hw)
{
	struct power_gate *gate = to_power_gate(hw);

	power_gate_endisable(hw, 1);

	if (gate->power_flags & POWER_GATE_WAIT) {
		while (!power_gate_is_enabled(hw));
	}

	return 0;
}

static void power_gate_disable(struct clk_hw *hw)
{
	struct power_gate *gate = to_power_gate(hw);

	power_gate_endisable(hw, 0);

	if (gate->power_flags & POWER_GATE_WAIT) {
		while (power_gate_is_enabled(hw));
	}
}

const struct clk_ops power_gate_ops = {
	.enable = power_gate_enable,
	.disable = power_gate_disable,
	.is_enabled = power_gate_is_enabled,
};
EXPORT_SYMBOL_GPL(power_gate_ops);

/**
 * clk_register_gate - register a gate clock with the clock framework
 * @dev: device that is registering this clock
 * @name: name of this clock
 * @parent_name: name of this clock's parent
 * @flags: framework-specific flags for this clock
 * @reg: register address to control gating of this clock
 * @bit_idx: which bit in the register controls gating of this clock
 * @clk_gate_flags: gate-specific flags for this clock
 * @lock: shared register lock for this clock
 */
struct clk *power_register_gate(struct device *dev, const char *name,
                                const char *parent_name, unsigned long flags,
                                void __iomem *reg, u8 ctrl_bit, u8 wait_bit,
                                u8 clk_gate_flags, unsigned long power_flags, spinlock_t *lock)
{
	struct power_gate *gate;
	struct clk *clk;
	struct clk_init_data init;

	/* allocate the gate */
	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate) {
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &power_gate_ops;
	init.flags = flags;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	/* struct power_gate assignments */
	gate->reg = reg;
	gate->ctrl_bit = ctrl_bit;
	gate->wait_bit = wait_bit;
	gate->flags = clk_gate_flags;
	gate->power_flags = power_flags;
	gate->lock = lock;
	gate->hw.init = &init;

	clk = clk_register(dev, &gate->hw);

	if (IS_ERR(clk)) {
		kfree(gate);
	}

	return clk;
}
EXPORT_SYMBOL_GPL(power_register_gate);

void power_unregister_gate(struct clk *clk)
{
	struct power_gate *gate;
	struct clk_hw *hw;

	hw = __clk_get_hw(clk);
	if (!hw) {
		return;
	}

	gate = to_power_gate(hw);

	clk_unregister(clk);
	kfree(gate);
}
EXPORT_SYMBOL_GPL(power_unregister_gate);
