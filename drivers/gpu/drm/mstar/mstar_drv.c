/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Based on the sunxi DRM driver
 */

#include <linux/component.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_generic.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "mstar_drm.h"
#include "mstar_framebuffer.h"

#define DRIVER_NAME "mstar-drm"

DEFINE_DRM_GEM_DMA_FOPS(mstar_drv_fops);

static const struct drm_driver mstar_drv_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,

	.fops = &mstar_drv_fops,
	.name = "mstar-drm",
	.desc = "MStar DRM driver",
	.date = "20210706",
	.major = 1,
	.minor = 0,

	DRM_GEM_DMA_DRIVER_OPS,
};

static const struct drm_mode_config_funcs drv_mode_config_funcs = {
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int mstar_drv_bind(struct device *dev)
{
	struct drm_device *drm;
	struct mstar_drv *drv;
	int ret;

	drm = drm_dev_alloc(&mstar_drv_driver, dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	drv = devm_kzalloc(dev, sizeof(*drv), GFP_KERNEL);
	if (!drv) {
		ret = -ENOMEM;
		goto free_drm;
	}

	dev_set_drvdata(dev, drm);
	drm->dev_private = drv;

	ret = of_reserved_mem_device_init(dev);
	if (ret && ret != -ENODEV) {
		dev_err(drm->dev, "Couldn't claim our memory region\n");
		goto free_drm;
	}

	drm_mode_config_init(drm);
	//drm->mode_config.allow_fb_modifiers = true;
	drm->mode_config.min_width = 0;
	drm->mode_config.min_height = 0;
	drm->mode_config.max_width = 8198;
	drm->mode_config.max_height = 8198;
	drm->mode_config.funcs = &drv_mode_config_funcs;

	ret = component_bind_all(drm->dev, drm);
	if (ret) {
		dev_err(drm->dev, "Couldn't bind all pipelines components\n");
		goto cleanup_mode_config;
	}

	/* drm_vblank_init calls kcalloc, which can fail */
	ret = drm_vblank_init(drm, drm->mode_config.num_crtc);
	if (ret)
		goto cleanup_mode_config;

	//drm->irq_enabled = true;

	/* Remove early framebuffers (ie. simplefb) */
	//remove_conflicting_framebuffers(NULL, "xx", false);

	mstar_framebuffer_init(drm);

	/* Enable connectors polling */
	drm_kms_helper_poll_init(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto finish_poll;

	drm_fbdev_generic_setup(drm, 16);

	return 0;

finish_poll:
	drm_kms_helper_poll_fini(drm);
cleanup_mode_config:
	drm_mode_config_cleanup(drm);
	of_reserved_mem_device_release(dev);

free_drm:
	drm_dev_put(drm);

	return ret;
}

static void mstar_drv_unbind(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	drm_dev_unregister(drm);
	drm_kms_helper_poll_fini(drm);
	drm_atomic_helper_shutdown(drm);
	drm_mode_config_cleanup(drm);

	component_unbind_all(dev, NULL);
	of_reserved_mem_device_release(dev);

	drm_dev_put(drm);
}

static const struct component_master_ops mstar_drv_master_ops = {
	.bind	= mstar_drv_bind,
	.unbind	= mstar_drv_unbind,
};

static int compare_of(struct device *dev, void *data)
{
	struct device_node *np = data;

	return dev->of_node == np;
}

static int mstar_drm_probe(struct platform_device *pdev)
{
	return drm_of_component_probe(&pdev->dev, compare_of, &mstar_drv_master_ops);
}

static int mstar_drm_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id mstar_drm_dt_ids[] = {
	{ .compatible = "sstar,ssd20xd-drm" },
	{},
};
MODULE_DEVICE_TABLE(of, mstar_drm_dt_ids);

static struct platform_driver mstar_drm_driver = {
	.probe = mstar_drm_probe,
	.remove = mstar_drm_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = mstar_drm_dt_ids,
	},
};

module_platform_driver(mstar_drm_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Mstar DRM Driver");
MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
