/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_of.h>
#include <linux/of.h>

struct mstar_ttl {
	struct drm_encoder encoder;
};

int mstar_ttl_init(struct drm_device *drm, struct device_node *of_node)
{
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	struct device_node *ep;
	struct mstar_ttl *ttl;
	int ret;

	printk("%s:%d\n", __func__, __LINE__);

	ep = of_graph_get_endpoint_by_regs(of_node, 0, 0);
	if (!ep)
		return -ENODEV;

	printk("%s:%d\n", __func__, __LINE__);

	ret = drm_of_find_panel_or_bridge(of_node, 0, 0, &panel, &bridge);
	if (ret) {
		of_node_put(ep);
		return ret;
	}

	printk("%s:%d\n", __func__, __LINE__);

	ttl = drmm_simple_encoder_alloc(drm, struct mstar_ttl, encoder, DRM_MODE_ENCODER_NONE);

	if(IS_ERR(ttl))
		return PTR_ERR(ttl);

	printk("%s:%d\n", __func__, __LINE__);

	ttl->encoder.possible_crtcs = 1;

	if (!panel)
		goto cleanup;

	bridge = drm_panel_bridge_add_typed(panel,
			DRM_MODE_CONNECTOR_Unknown);
	if (IS_ERR(bridge))
		return PTR_ERR(bridge);

	ret = drm_bridge_attach(&ttl->encoder, bridge, NULL, 0);
	if (!ret)
		return 0;

cleanup_panel:
	if (panel)
		drm_panel_bridge_remove(bridge);

	printk("%s:%d\n", __func__, __LINE__);

cleanup:
	drm_encoder_cleanup(&ttl->encoder);

	return ret;
}
