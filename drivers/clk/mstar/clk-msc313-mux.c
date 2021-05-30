// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Daniel Palmer <daniel@thingy.jp>
 */

#include <linux/clk-provider.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "clk-msc313-mux.h"

#define deglitch_to_mux(_hw) container_of(_hw, struct msc313_mux, deglitch_hw)
#define mux_to_mux(_hw) container_of(_hw, struct msc313_mux, mux_hw)

static int msc313_mux_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct msc313_mux *mux = mux_to_mux(hw);

	return regmap_field_write(mux->mux, index);
}

static u8 msc313_mux_mux_get_parent(struct clk_hw *hw)
{
	struct msc313_mux *mux = mux_to_mux(hw);
	unsigned int index;

	regmap_field_read(mux->mux, &index);

	return index;
}

static const struct clk_ops msc313_mux_mux_ops = {
	.set_parent = msc313_mux_mux_set_parent,
	.get_parent = msc313_mux_mux_get_parent,
};

static int msc313_mux_deglitch_enable(struct clk_hw *hw)
{
	struct msc313_mux *mux = deglitch_to_mux(hw);

	if (mux->gate)
		regmap_field_write(mux->gate, 0);

	return 0;
}

static void msc313_mux_deglitch_disable(struct clk_hw *hw)
{
	struct msc313_mux *mux = deglitch_to_mux(hw);

	if (mux->gate)
		regmap_field_write(mux->gate, 1);
}

static int msc313_mux_deglitch_is_enabled(struct clk_hw *hw)
{
	struct msc313_mux *mux = deglitch_to_mux(hw);
	unsigned int notgated;

	if (mux->gate) {
		regmap_field_read(mux->gate, &notgated);
		return !notgated;
	}

	return 1;
}

static int msc313_mux_deglitch_set_parent(struct clk_hw *hw, u8 index)
{
	struct msc313_mux *mux = deglitch_to_mux(hw);

	if (!mux->deglitch)
		return -ENOTSUPP;

	return regmap_field_write(mux->deglitch, index);
}

static u8 msc313_mux_deglitch_get_parent(struct clk_hw *hw)
{
	struct msc313_mux *mux = deglitch_to_mux(hw);
	unsigned int index = 0;

	if (mux->deglitch){
		regmap_field_read(mux->deglitch, &index);
	}

	return index;
}

static const struct clk_ops msc313_mux_deglitch_ops = {
	.enable = msc313_mux_deglitch_enable,
	.disable = msc313_mux_deglitch_disable,
	.is_enabled = msc313_mux_deglitch_is_enabled,
	.set_parent = msc313_mux_deglitch_set_parent,
	.get_parent = msc313_mux_deglitch_get_parent,
};

struct clk_hw *msc313_mux_xlate(struct of_phandle_args *clkspec, void *data)
{
	struct msc313_muxes *muxes = data;
	unsigned int of_idx = clkspec->args[0];
	unsigned int idx = of_idx / 2;

	/* mux, deglitch, mux, deglitch,.. */
	if (of_idx >= muxes->muxes_data->num_muxes * 2)
		return ERR_PTR(-EINVAL);

	if (of_idx % 2)
		return &muxes->muxes[idx].deglitch_hw;

	return &muxes->muxes[idx].mux_hw;
}

struct msc313_muxes *msc313_mux_register_muxes(struct device *dev,
		struct regmap *regmap, const struct msc313_muxes_data *muxes_data,
		int (*fill_clk_parent_data)(struct clk_parent_data*, void*, const void*, const struct msc313_muxes*, unsigned int, unsigned int), void *data)
{
	const struct msc313_mux_data *mux_data = muxes_data->muxes;
	struct clk_init_data mux_init = {
		.ops = &msc313_mux_mux_ops,
	};
	struct clk_init_data deglitch_init = {
		.ops = &msc313_mux_deglitch_ops,
	};
	struct clk_parent_data *dynamic_parent_data = NULL;
	struct msc313_muxes *muxes;
	struct msc313_mux *mux;
	struct clk_hw *clk_hw;
	int i, ret, mux_parent;

	/*
	 * If using the dynamic clk_parent_data mode you have to have both
	 * a callback and data.
	 */
	if ((fill_clk_parent_data && !data) || (!fill_clk_parent_data && data))
		return ERR_PTR(-EINVAL);

	muxes = devm_kzalloc(dev, struct_size(muxes, muxes, muxes_data->num_muxes), GFP_KERNEL);
        if (!muxes)
		return ERR_PTR(-ENOMEM);

	muxes->muxes_data = muxes_data;
        mux = muxes->muxes;

	for (i = 0; i < muxes_data->num_muxes; i++, mux++, mux_data++) {
		struct reg_field gate_field = REG_FIELD(mux_data->offset,
				mux_data->gate_shift, mux_data->gate_shift);
		struct reg_field mux_field = REG_FIELD(mux_data->offset,
				mux_data->mux_shift, mux_data->mux_shift + (mux_data->mux_width - 1));
		struct reg_field deglitch_field = REG_FIELD(mux_data->offset,
				mux_data->deglitch_shift, mux_data->deglitch_shift);

		if(!mux_data->name)
			continue;

		if (mux_data->gate_shift != -1) {
			mux->gate = devm_regmap_field_alloc(dev, regmap, gate_field);
			if (IS_ERR(mux->gate))
				return (struct msc313_muxes*) mux->gate;
		}

		mux->mux =  devm_regmap_field_alloc(dev, regmap, mux_field);
		if (IS_ERR(mux->mux))
			return (struct msc313_muxes*) mux->mux;

		if (mux_data->deglitch_shift != -1){
			mux->deglitch =  devm_regmap_field_alloc(dev, regmap, deglitch_field);
			if (IS_ERR(mux->deglitch))
				return (struct msc313_muxes*) mux->deglitch;
		}

		mux_init.name = devm_kasprintf(dev, GFP_KERNEL, "%s_mux", mux_data->name);
		if (!mux_init.name)
			return ERR_PTR(-ENOMEM);

		if (fill_clk_parent_data) {
			dynamic_parent_data = devm_kcalloc(dev, mux_data->num_parents,
					sizeof(*dynamic_parent_data), GFP_KERNEL);
			for (mux_parent = 0; mux_parent < mux_data->num_parents; mux_parent++){
				ret = fill_clk_parent_data(&dynamic_parent_data[mux_parent], data,
						mux_data->parent_data, muxes, i, mux_parent);
				if (ret)
					return ERR_PTR(ret);
			}
			mux_init.parent_data = dynamic_parent_data;
		}
		else
			mux_init.parent_data = mux_data->clk_parent_data;

		mux_init.num_parents = mux_data->num_parents;
		mux_init.flags = mux_data->mux_flags;
		clk_hw = &mux->mux_hw;
		clk_hw->init = &mux_init;

		ret = devm_clk_hw_register(dev, clk_hw);
		if (ret)
			return ERR_PTR(ret);

		mux->deglitch_parents[0].hw = &mux->mux_hw;
		mux->deglitch_parents[1].fw_name = "deglitch";

		deglitch_init.name = mux_data->name;
		deglitch_init.parent_data = mux->deglitch_parents;
		deglitch_init.num_parents = mux->deglitch ? 2 : 1;
		deglitch_init.flags = mux_data->flags;
		clk_hw = &mux->deglitch_hw;
		clk_hw->init = &deglitch_init;

		ret = devm_clk_hw_register(dev, clk_hw);
		if (ret)
			return ERR_PTR(ret);

		if (dynamic_parent_data)
			devm_kfree(dev, dynamic_parent_data);
	}

	return muxes;
}
