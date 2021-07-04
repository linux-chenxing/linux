#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_plane.h>
#include <linux/component.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define DRIVER_NAME "mstar-mop"

#define ADDR_SHIFT	4

static const uint32_t mop_formats[] = {
	DRM_FORMAT_NV12,
};

static const uint64_t mop_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

struct mstar_mop;

struct mstar_mop_window {
	struct mstar_mop *mop;
	struct regmap_field *en;
	struct regmap_field *yaddrl, *yaddrh;
	struct regmap_field *caddrl, *caddrh;
	struct regmap_field *hst;
	struct regmap_field *hend;
	struct regmap_field *vst;
	struct regmap_field *vend;
	struct regmap_field *pitch;
	struct regmap_field *src_width;
	struct regmap_field *src_height;
	struct regmap_field *scale_h;
	struct regmap_field *scale_v;
	struct drm_plane drm_plane;
};

#define plane_to_mop_window(plane) container_of(plane, struct mstar_mop_window, drm_plane)

struct mstar_mop_data {
	unsigned int num_windows;
	unsigned int windows_start;
	unsigned int window_len;
};

struct mstar_mop {
	struct device *dev;
	const struct mstar_mop_data *data;
	struct regmap_field *swrst;
	struct regmap_field *gw_hsize;
	struct regmap_field *gw_vsize;
	struct regmap_field *commit_all;
	struct mstar_mop_window windows[];
};

static const struct reg_field swrst_field = REG_FIELD(0x0, 0, 0);
static const struct reg_field gw_hsize_field = REG_FIELD(0x1c, 0, 12);
static const struct reg_field gw_vsize_field = REG_FIELD(0x20, 0, 12);
static const struct reg_field commit_all_field = REG_FIELD(0x1fc, 8, 8);

static const struct regmap_config mstar_mop_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static void mstar_mop_dump_window(struct device *dev, struct mstar_mop_window *win)
{
	unsigned int en;
	unsigned int yaddrl, yaddrh;
	unsigned int caddrl, caddrh;
	unsigned int hst, hend;
	unsigned int vst, vend;
	unsigned int pitch;
	unsigned int srcw, srch;
	unsigned int scaleh, scalev;
	u32 yaddr, caddr;

	regmap_field_read(win->en, &en);

	regmap_field_read(win->yaddrl, &yaddrl);
	regmap_field_read(win->yaddrh, &yaddrh);
	yaddr = (yaddrh << 16 | yaddrl) << ADDR_SHIFT;

	regmap_field_read(win->caddrl, &caddrl);
	regmap_field_read(win->caddrh, &caddrh);
	caddr = (caddrh << 16 | caddrl) << ADDR_SHIFT;

	regmap_field_read(win->hst, &hst);
	regmap_field_read(win->hend, &hend);
	regmap_field_read(win->vst, &vst);
	regmap_field_read(win->vend, &vend);
	regmap_field_read(win->pitch, &pitch);
	regmap_field_read(win->src_width, &srcw);
	regmap_field_read(win->src_height, &srch);
	regmap_field_read(win->scale_h, &scaleh);
	regmap_field_read(win->scale_v, &scalev);

	dev_info(dev, "Window dump\n"
		      "enabled: %d\n"
		      "yaddr: 0x%08x\n"
		      "caddr: 0x%08x\n"
		      "horizontal start: %d, end %d\n"
		      "vertical start: %d, end %d\n"
		      "pitch: %d\n"
		      "source width: %d, height: %d\n"
		      "scale horizontal: %d, vertical: %d\n",
		      en,
		      yaddr,
		      caddr,
		      hst, hend,
		      vst, vend,
		      pitch,
		      srcw, srch,
		      scaleh, scalev);
}

static int mop_plane_atomic_check(struct drm_plane *plane,
				    struct drm_atomic_state *state)
{
	printk("%s\n", __func__);
	return 0;
}

static void mstar_mop_plane_atomic_update(struct drm_plane *plane,
				    struct drm_atomic_state *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct mstar_mop_window *window = plane_to_mop_window(plane);
	struct mstar_mop *mop = window->mop;

	printk("%s\n", __func__);

	regmap_field_write(window->en, new_state->crtc ? 1 : 0);

	regmap_field_force_write(mop->commit_all, 1);
	regmap_field_force_write(mop->commit_all, 0);

	mstar_mop_dump_window(mop->dev, window);
}

static const struct drm_plane_helper_funcs mop_plane_helper_funcs = {
	.atomic_check = mop_plane_atomic_check,
	.atomic_update = mstar_mop_plane_atomic_update,
};

static const struct drm_plane_funcs mop_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

static int mstar_mop_bind(struct device *dev, struct device *master,
			 void *data)
{
	struct mstar_mop *mop = dev_get_drvdata(dev);
	struct drm_device *drm_device = data;
	int i, ret;

	for(i = 0; i < mop->data->num_windows; i++) {
		struct mstar_mop_window *window = &mop->windows[i];

		ret = drm_universal_plane_init(drm_device,
					     &window->drm_plane,
					     0,
					     &mop_plane_funcs,
					     mop_formats,
					     ARRAY_SIZE(mop_formats),
					     mop_format_modifiers,
					     DRM_PLANE_TYPE_OVERLAY,
					     "window %d", i);
		if(ret)
			return ret;

		drm_plane_helper_add(&window->drm_plane, &mop_plane_helper_funcs);
	}

	return 0;
}

static void mstar_mop_unbind(struct device *dev, struct device *master,
			    void *data)
{
	struct mstar_mop *mop = dev_get_drvdata(dev);
	int i;

	for(i = 0; i < mop->data->num_windows; i++)
		drm_plane_cleanup(&mop->windows[i].drm_plane);
}

static const struct component_ops mstar_mop_component_ops = {
	.bind	= mstar_mop_bind,
	.unbind	= mstar_mop_unbind,
};

static int mstar_mop_probe(struct platform_device *pdev)
{
	const struct mstar_mop_data *match_data;
	struct device *dev = &pdev->dev;
	unsigned int hsize, vsize;
	struct mstar_mop *mop;
	struct regmap *regmap;
	void __iomem *base;
	int i, ret;

	match_data = of_device_get_match_data(dev);
	if (!match_data)
		return -EINVAL;

	mop = devm_kzalloc(dev, struct_size(mop, windows, match_data->num_windows), GFP_KERNEL);
	if (!mop)
		return -ENOMEM;

	mop->dev = dev;
	mop->data = match_data;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio(dev, base, &mstar_mop_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	mop->swrst = devm_regmap_field_alloc(dev, regmap, swrst_field);
	mop->gw_hsize = devm_regmap_field_alloc(dev, regmap, gw_hsize_field);
	mop->gw_vsize = devm_regmap_field_alloc(dev, regmap, gw_vsize_field);
	mop->commit_all = devm_regmap_field_alloc(dev, regmap, commit_all_field);

	regmap_field_read(mop->gw_hsize, &hsize);
	regmap_field_read(mop->gw_vsize, &vsize);
	dev_info(dev, "MStar MOP\n"
		      "global window size; height: %d, width: %d\n",
		      hsize, vsize
		);

	for (i = 0; i < match_data->num_windows; i++){
		unsigned int offset = match_data->windows_start + (match_data->window_len * i);
		unsigned int en;

		struct reg_field en_field = REG_FIELD(offset + 0, 0, 0);
		struct reg_field yaddrl_field = REG_FIELD(offset + 0x8, 0, 15);
		struct reg_field yaddrh_field = REG_FIELD(offset + 0xc, 0, 11);
		struct reg_field caddrl_field = REG_FIELD(offset + 0x10, 0, 15);
		struct reg_field caddrh_field = REG_FIELD(offset + 0x14, 0, 11);
		struct reg_field hst_field = REG_FIELD(offset + 0x18, 0, 12);
		struct reg_field hend_field = REG_FIELD(offset + 0x1c, 0, 12);
		struct reg_field vst_field = REG_FIELD(offset + 0x20, 0, 12);
		struct reg_field vend_field = REG_FIELD(offset + 0x24, 0, 12);
		struct reg_field pitch_field = REG_FIELD(offset + 0x28, 0, 12);
		struct reg_field srcw_field = REG_FIELD(offset + 0x2c, 0, 12);
		struct reg_field srch_field = REG_FIELD(offset + 0x30, 0, 12);
		struct reg_field scaleh_field = REG_FIELD(offset + 0x34, 0, 12);
		struct reg_field scalev_field = REG_FIELD(offset + 0x38, 0, 12);
		struct mstar_mop_window *window = &mop->windows[i];

		window->mop = mop;

		window->en = devm_regmap_field_alloc(dev, regmap, en_field);
		window->yaddrl = devm_regmap_field_alloc(dev, regmap, yaddrl_field);
		window->yaddrh = devm_regmap_field_alloc(dev, regmap, yaddrh_field);
		window->caddrl = devm_regmap_field_alloc(dev, regmap, caddrl_field);
		window->caddrh = devm_regmap_field_alloc(dev, regmap, caddrh_field);
		window->hst = devm_regmap_field_alloc(dev, regmap, hst_field);
		window->hend = devm_regmap_field_alloc(dev, regmap, hend_field);
		window->vst = devm_regmap_field_alloc(dev, regmap, vst_field);
		window->vend = devm_regmap_field_alloc(dev, regmap, vend_field);
		window->pitch = devm_regmap_field_alloc(dev, regmap, pitch_field);
		window->src_height = devm_regmap_field_alloc(dev, regmap, srcw_field);
		window->src_width = devm_regmap_field_alloc(dev, regmap, srch_field);
		window->scale_h = devm_regmap_field_alloc(dev, regmap, scaleh_field);
		window->scale_v = devm_regmap_field_alloc(dev, regmap, scalev_field);
		mstar_mop_dump_window(dev, window);
	}

	dev_set_drvdata(dev, mop);

	return component_add(&pdev->dev, &mstar_mop_component_ops);
}

static int mstar_mop_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &mstar_mop_component_ops);
	return 0;
}

static const struct mstar_mop_data ssd20xd_mopg_data = {
	.num_windows = 16,
	.windows_start = 0x200,
	.window_len = 0x40,
};

static const struct mstar_mop_data ssd20xd_mops_data = {
	.num_windows = 1,
	.windows_start = 0x20,
	.window_len = 0x40,
};

static const struct of_device_id mstar_mop_ids[] = {
	{
		.compatible = "sstar,ssd20xd-mopg",
		.data = &ssd20xd_mopg_data,
	},
	{
		.compatible = "sstar,ssd20xd-mops",
		.data = &ssd20xd_mops_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, mstar_mop_ids);

static struct platform_driver mstar_mop_driver = {
	.probe = mstar_mop_probe,
	.remove = mstar_mop_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = mstar_mop_ids,
	},
};

module_platform_driver(mstar_mop_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
