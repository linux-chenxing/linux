// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <video/videomode.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_edid.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#define DRIVER_NAME "mstar hdmi"

struct mstar_hdmi {
	struct drm_bridge bridge;
	struct drm_encoder encoder;
	struct drm_connector *connector;
	struct drm_device *drm_device;
	struct i2c_adapter *ddc_adpt;
	void __iomem *regs;
	bool dvi_mode;
};

static inline struct mstar_hdmi *hdmi_ctx_from_bridge(struct drm_bridge *b)
{
	return container_of(b, struct mstar_hdmi, bridge);
}

static void mstar_hdmi_bridge_atomic_enable(struct drm_bridge *bridge,
					  struct drm_bridge_state *old_state)
{
	struct drm_atomic_state *state = old_state->base.state;
	struct mstar_hdmi *hdmi = hdmi_ctx_from_bridge(bridge);

	printk("%s:%d\n", __func__, __LINE__);
}

static enum drm_connector_status mstar_hdmi_bridge_detect(struct drm_bridge *bridge)
{
	struct mstar_hdmi *hdmi = hdmi_ctx_from_bridge(bridge);

	return connector_status_connected;
}

static struct edid *mstar_hdmi_bridge_get_edid(struct drm_bridge *bridge,
					     struct drm_connector *connector)
{
	struct mstar_hdmi *hdmi = hdmi_ctx_from_bridge(bridge);
	struct edid *edid;

	printk("%s:%d\n", __func__, __LINE__);

	if (!hdmi->ddc_adpt)
		return NULL;
	edid = drm_get_edid(connector, hdmi->ddc_adpt);
	if (!edid)
		return NULL;
	hdmi->dvi_mode = !drm_detect_monitor_audio(edid);
	return edid;
}

static const struct drm_bridge_funcs mstar_hdmi_bridge_funcs = {
	.atomic_duplicate_state	= drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_bridge_destroy_state,
	.atomic_reset		= drm_atomic_helper_bridge_reset,
	.atomic_enable		= mstar_hdmi_bridge_atomic_enable,
	.detect			= mstar_hdmi_bridge_detect,
	.get_edid		= mstar_hdmi_bridge_get_edid,
};

static int mstar_hdmi_bind(struct device *dev, struct device *master, void *data)
{
	struct mstar_hdmi *hdmi = dev_get_drvdata(dev);
	struct drm_device *drm_device = data;
	int ret;

	printk("%s:%d\n", __func__, __LINE__);
	hdmi->drm_device = drm_device;

	hdmi->bridge.funcs = &mstar_hdmi_bridge_funcs;
	hdmi->bridge.of_node = dev->of_node;
	hdmi->bridge.ops = DRM_BRIDGE_OP_EDID | DRM_BRIDGE_OP_DETECT; //DRM_BRIDGE_OP_HPD;
	hdmi->bridge.type = DRM_MODE_CONNECTOR_HDMIA;

	drm_bridge_add(&hdmi->bridge);

	ret = drm_simple_encoder_init(drm_device, &hdmi->encoder,
			DRM_MODE_ENCODER_TMDS);
	if(ret)
		return ret;

	hdmi->encoder.possible_crtcs = 1;

	ret = drm_bridge_attach(&hdmi->encoder, &hdmi->bridge, NULL,
			DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		goto err_cleanup_encoder;

	hdmi->connector = drm_bridge_connector_init(drm_device, &hdmi->encoder);
	if (IS_ERR(hdmi->connector)) {
		DRM_ERROR("Unable to create bridge connector\n");
		ret = PTR_ERR(hdmi->connector);
		goto err_cleanup_encoder;
	}

	drm_connector_attach_encoder(hdmi->connector, &hdmi->encoder);

	return 0;

err_cleanup_encoder:
	drm_encoder_cleanup(&hdmi->encoder);
	return ret;
}

static void mstar_hdmi_unbind(struct device *dev, struct device *master,
			   void *data)
{
	struct mstar_hdmi *hdmi = dev_get_drvdata(dev);
}

static const struct component_ops mstar_hdmi_component_ops = {
	.bind = mstar_hdmi_bind,
	.unbind = mstar_hdmi_unbind,
};

static int mstar_hdmi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *i2c_np;
	struct mstar_hdmi *hdmi;
	int ret;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	hdmi->regs = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(hdmi->regs))
			return PTR_ERR(hdmi->regs);

	i2c_np = of_parse_phandle(np, "ddc-i2c-bus", 0);
	if (!i2c_np) {
		dev_err(dev, "Failed to find ddc-i2c-bus node in %pOF\n", np);
		return -EINVAL;
	}

	hdmi->ddc_adpt = of_find_i2c_adapter_by_node(i2c_np);
	of_node_put(i2c_np);
	if (!hdmi->ddc_adpt) {
		dev_err(dev, "Failed to get ddc i2c adapter by node\n");
		return -EPROBE_DEFER;
	}

	platform_set_drvdata(pdev, hdmi);

	ret = component_add(&pdev->dev, &mstar_hdmi_component_ops);
	if (ret)
		return ret;

	return 0;
}

static int mstar_hdmi_remove(struct platform_device *pdev)
{
	struct mstar_hdmi *hdmi = platform_get_drvdata(pdev);

	drm_bridge_remove(&hdmi->bridge);
	component_del(&pdev->dev, &mstar_hdmi_component_ops);

	return 0;
}

static const struct of_device_id mstar_hdmi_of_match[] = {
	{
		.compatible = "sstar,ssd20xd-hdmi",
	},
	{}
};
MODULE_DEVICE_TABLE(of, mstar_hdmi_of_match);

struct platform_driver mstar_hdmi_driver = {
	.probe = mstar_hdmi_probe,
	.remove = mstar_hdmi_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = mstar_hdmi_of_match,
	},
};
module_platform_driver(mstar_hdmi_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
