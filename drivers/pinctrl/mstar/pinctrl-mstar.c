// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Daniel Palmer <daniel@thingy.jp>
 */

#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/slab.h>
#include <linux/regmap.h>

#include "../core.h"
#include "../devicetree.h"
#include "../pinconf.h"
#include "../pinmux.h"
#include "pinctrl-mstar.h"

static int mstar_pinctrl_dt_node_to_map(struct pinctrl_dev *pctldev, struct device_node *np,
		struct pinctrl_map **map, unsigned int *num_maps)
{
	return pinconf_generic_dt_node_to_map(pctldev, np, map, num_maps,
			PIN_MAP_TYPE_INVALID);
}

static void mstar_pinctrl_dt_free_map(struct pinctrl_dev *pctldev, struct pinctrl_map *map,
		unsigned int num_maps)
{
	kfree(map);
}

const struct pinctrl_ops msc313_pinctrl_ops = {
	.get_groups_count	= pinctrl_generic_get_group_count,
	.get_group_name		= pinctrl_generic_get_group_name,
	.get_group_pins		= pinctrl_generic_get_group_pins,
	.dt_node_to_map		= mstar_pinctrl_dt_node_to_map,
	.dt_free_map		= mstar_pinctrl_dt_free_map,
};

static int mstar_pinctrl_set_mux(struct pinctrl_dev *pctldev, unsigned int func,
			   unsigned int group)
{
	struct msc313_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);
	const char *grpname = pinctrl_generic_get_group_name(pctldev, group);
	struct function_desc *funcdesc = pinmux_generic_get_function(pctldev, func);
	struct msc313_pinctrl_function *function = funcdesc->data;
	int i, ret = 0;

	if (function != NULL) {
		if (function->reg >= 0 && function->values != NULL) {
			for (i = 0; i < function->numgroups; i++) {
				if (strcmp(function->groups[i], grpname) == 0) {
					dev_dbg(pinctrl->dev, "updating mux reg %x\n", (unsigned int) function->reg);
					ret = regmap_update_bits(pinctrl->regmap, function->reg,
							function->mask, function->values[i]);
					if (ret)
						dev_dbg(pinctrl->dev, "failed to update register\n");
					break;
				}
			}
		} else {
			dev_dbg(pinctrl->dev, "reg or values not found\n");
		}
	} else {
		dev_info(pinctrl->dev, "missing function data\n");
	}

	return ret;
}

const struct pinmux_ops mstar_pinmux_ops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name   = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux             = mstar_pinctrl_set_mux,
	.strict              = true,
};

int mstar_pinctrl_parse_groups(struct msc313_pinctrl *pinctrl)
{
	int i, ret;

	for (i = 0; i < pinctrl->info->ngroups; i++) {
		const struct msc313_pinctrl_group *grp = &pinctrl->info->groups[i];

		ret = pinctrl_generic_add_group(pinctrl->pctl, grp->name,
				(int *) grp->pins, grp->numpins, NULL);
	}
	return ret;
}

int mstar_pinctrl_parse_functions(struct msc313_pinctrl *pinctrl)
{
	int i, ret;

	for (i = 0; i < pinctrl->info->nfunctions; i++) {
		const struct msc313_pinctrl_function *func =  &pinctrl->info->functions[i];

		// clear any existing value for the function
		if (func->reg >= 0) {
			regmap_update_bits(pinctrl->regmap, func->reg,
					func->mask, 0);
		}

		ret = pinmux_generic_add_function(pinctrl->pctl, func->name,
				(const char **) func->groups, func->numgroups, (void *) func);
		if (ret < 0) {
			dev_err(pinctrl->dev, "failed to add function: %d", ret);
			goto out;
		}
	}
out:
	return ret;
}
