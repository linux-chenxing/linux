// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Daniel Palmer
 */

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/of_address.h>
#include <linux/module.h>

/*
 */

struct mstar_clkgen_gates {
	spinlock_t lock;
	struct clk_onecell_data *clk_data;
};

static const struct of_device_id mstar_clkgen_gates_of_match[] = {
	{
		.compatible = "mstar,clkgen-gates",
	},
	{}
};
MODULE_DEVICE_TABLE(of, mstar_clkgen_gates_of_match);

static int msc313e_clkgen_mux_probe(struct platform_device *pdev)
{
	struct mstar_clkgen_gates *gates;
	const struct of_device_id *id;
	struct resource *base;
	struct clk* clk;
	int ret, numparents, numshifts, numgates,
		gateindex;
	const char *name;
	const char *parents[32];
	u32 gateshift, outputflags;

	if (!pdev->dev.of_node)
		return -ENODEV;

	id = of_match_node(mstar_clkgen_gates_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	numparents = of_clk_parent_fill(pdev->dev.of_node, parents,
			ARRAY_SIZE(parents));

	if(numparents <= 0){
		dev_info(&pdev->dev, "failed to get clock parents\n");
		return numparents;
	}

	if(numparents != of_clk_get_parent_count(pdev->dev.of_node)){
		dev_info(&pdev->dev, "waiting for parents\n");
		return -EPROBE_DEFER;
	}

	base = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR(base)){
		ret = PTR_ERR(base);
		goto out;
	}

	numgates = of_property_count_strings(pdev->dev.of_node,
			"clock-output-names");
	if (!numgates) {
		dev_info(&pdev->dev, "output names need to be specified");
		ret = -ENODEV;
		goto out;
	}

	if(numparents != numgates){
		dev_info(&pdev->dev, "number of outputs must match never of parents");
		ret = -EINVAL;
		goto out;
	}

	numshifts = of_property_count_u32_elems(pdev->dev.of_node, "shifts");
	if(!numshifts){
		dev_info(&pdev->dev, "shifts need to be specified");
		ret =-ENODEV;
		goto out;
	}

	if(numshifts != numparents){
		dev_info(&pdev->dev, "number of shifts must match number of parents");
		ret = -EINVAL;
		goto out;
	}

	gates = devm_kzalloc(&pdev->dev, sizeof(struct mstar_clkgen_gates),
			GFP_KERNEL);
	if(!gates){
		ret = -ENOMEM;
		return ret;
	}

	spin_lock_init(&gates->lock);

	gates->clk_data = devm_kzalloc(&pdev->dev, sizeof(struct clk_onecell_data),
			GFP_KERNEL);
	if (!gates->clk_data){
		ret = -ENOMEM;
		goto out;
	}

	gates->clk_data->clk_num = numgates;
	gates->clk_data->clks = devm_kcalloc(&pdev->dev, numgates,
			sizeof(struct clk *), GFP_KERNEL);
	if (!gates->clk_data->clks){
		ret = -ENOMEM;
		goto out;
	}

	for (gateindex = 0; gateindex < numgates; gateindex++) {
		ret = of_property_read_string_index(pdev->dev.of_node, "clock-output-names",
				gateindex, &name);
		if(ret)
			goto out;

		ret = of_property_read_u32_index(pdev->dev.of_node, "shifts",
				gateindex, &gateshift);
		if(ret)
			goto out;

		outputflags = 0;
		of_property_read_u32_index(pdev->dev.of_node, "output-flags",
				gateindex, &outputflags);
		if(!ret){
			dev_dbg(&pdev->dev, "applying flags %x to output %d",
					outputflags, gateindex);
		}

		clk = clk_register_gate(&pdev->dev, name, parents[gateindex], outputflags,
				base, gateshift, CLK_GATE_SET_TO_DISABLE, &gates->lock);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			goto out;
		}
		gates->clk_data->clks[gateindex] = clk;
	}

	platform_set_drvdata(pdev, gates);

	return of_clk_add_provider(pdev->dev.of_node,
			of_clk_src_onecell_get, gates->clk_data);
out:
	return ret;
}

static int msc313e_clkgen_mux_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver mstar_clkgen_gates = {
	.driver = {
		.name = "mstar-clkgen-gates",
		.of_match_table = mstar_clkgen_gates_of_match,
	},
	.probe = msc313e_clkgen_mux_probe,
	.remove = msc313e_clkgen_mux_remove,
};
module_platform_driver(mstar_clkgen_gates);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_DESCRIPTION("MStar MSC313e clkgen mux driver");
