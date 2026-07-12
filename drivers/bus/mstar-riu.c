/*
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

struct mstar_riu {
	struct clk *clk;
};

static int mstar_riu_probe(struct platform_device *pdev)
{
	struct mstar_riu* riu;
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;

	if (np)
		of_platform_populate(np, NULL, NULL, &pdev->dev);

	riu = devm_kzalloc(&pdev->dev, sizeof(*riu), GFP_KERNEL);
	if(!riu){
		ret = -ENOMEM;
		goto out;
	}

	riu->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(riu->clk)) {
		ret = PTR_ERR(riu->clk);
		goto out;
	}

	ret = clk_prepare_enable(riu->clk);

out:
	return ret;
}

static int mstar_riu_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id mstar_riu_of_match[] = {
	{ .compatible = "mstar,riubrdg", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mstar_riu_of_match);

static struct platform_driver mstar_riu_driver = {
	.probe = mstar_riu_probe,
	.remove = mstar_riu_remove,
	.driver = {
		.name = "mstar-riu",
		.of_match_table = mstar_riu_of_match,
	},
};

module_platform_driver(mstar_riu_driver);

MODULE_DESCRIPTION("MStar RIU");
MODULE_AUTHOR("Daniel Palmer <daniel@thingy.jp>");
MODULE_LICENSE("GPL v2");
