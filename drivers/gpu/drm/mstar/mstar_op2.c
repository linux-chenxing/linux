#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_vblank.h>
#include <linux/component.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "mstar_drm.h"
#include "mstar_ttl.h"
#include "mstar_top.h"

#define DRIVER_NAME "mstar-op2"

#define REG_40			0x40
#define REG_HTT			0x44
#define REG_VTT			0x48
#define REG_HSYNC_START		0x4c
#define REG_HSYNC_END		0x50
#define REG_VSYNC_START		0x54
#define REG_VSYNC_END		0x58
#define REG_HFDE_START		0x5c
#define REG_HFDE_END		0x60
#define REG_VFDE_START		0x64
#define REG_VFDE_END		0x68
//#define REG_FRAME_COLOR		0x6c
#define REG_HDE_START		0x70
#define REG_HDE_END		0x74
#define REG_VDE_START		0x78
#define REG_VDE_END		0x7c

#define REG_COLOR_MATRIX_CTRL	0xf0
#define REG_COLOR_MATRIX_0	0xf4
#define REG_COLOR_MATRIX_1	0xf8
#define REG_COLOR_MATRIX_2	0xfc
#define REG_COLOR_MATRIX_3	0x100
#define REG_COLOR_MATRIX_4	0x104
#define REG_COLOR_MATRIX_5	0x108
#define REG_COLOR_MATRIX_6	0x10c
#define REG_COLOR_MATRIX_7	0x110
#define REG_COLOR_MATRIX_8	0x114

#define REG_DITHER		0x1c8
#define REG_OUTPUT_CTRL		0x1f8

static struct reg_field tgenexthsen_field = REG_FIELD(REG_40, 2, 2);

static struct reg_field htt_field = REG_FIELD(REG_HTT, 0, 12);
static struct reg_field vtt_field = REG_FIELD(REG_VTT, 0, 12);

static struct reg_field hsync_start_field = REG_FIELD(REG_HSYNC_START, 0, 12);
static struct reg_field hsync_end_field = REG_FIELD(REG_HSYNC_END, 0, 12);
static struct reg_field vsync_start_field = REG_FIELD(REG_VSYNC_START, 0, 12);
static struct reg_field vsync_end_field = REG_FIELD(REG_VSYNC_END, 0, 12);
/* display moves around when messing with these */
static struct reg_field hfde_start_field = REG_FIELD(REG_HFDE_START, 0, 12);
static struct reg_field hfde_end_field = REG_FIELD(REG_HFDE_END, 0, 12);
static struct reg_field vfde_start_field = REG_FIELD(REG_VFDE_START, 0, 12);
static struct reg_field vfde_end_field = REG_FIELD(REG_VFDE_END, 0, 12);
/* these cause garbage on the screen sometimes */
static struct reg_field hde_start_field = REG_FIELD(REG_HDE_START, 0, 12);
static struct reg_field hde_end_field = REG_FIELD(REG_HDE_END, 0, 12);
static struct reg_field vde_start_field = REG_FIELD(REG_VDE_START, 0, 12);
static struct reg_field vde_end_field = REG_FIELD(REG_VDE_END, 0, 12);

static struct reg_field output_swap_b = REG_FIELD(REG_OUTPUT_CTRL, 0, 1);
static struct reg_field output_swap_g = REG_FIELD(REG_OUTPUT_CTRL, 2, 3);
static struct reg_field output_swap_r = REG_FIELD(REG_OUTPUT_CTRL, 4, 5);
static struct reg_field output_mode_field = REG_FIELD(REG_OUTPUT_CTRL, 6, 7);
#define OP2_OUTPUTMODE_888	0
#define OP2_OUTPUTMODE_666	1
#define OP2_OUTPUTMODE_565	2
static struct reg_field output_swap_ml_field = REG_FIELD(REG_OUTPUT_CTRL, 8, 8);

struct mstar_op2 {
	struct device *dev;
	struct drm_crtc drm_crtc;
	struct regmap_field *htt, *vtt;
	struct regmap_field *hsync_start, *hsync_end;
	struct regmap_field *vsync_start, *vsync_end;
	struct regmap_field *hfde_start, *hfde_end;
	struct regmap_field *vfde_start, *vfde_end;
	struct regmap_field *hde_start, *hde_end;
	struct regmap_field *vde_start, *vde_end;
	struct regmap_field *output_mode;
	struct regmap_field *swap_b, *swap_g, *swap_r;
	struct regmap_field *swap_ml;
};

#define crtc_to_op2(crtc) container_of(crtc, struct mstar_op2, drm_crtc)

static const struct regmap_config mstar_op2_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static void mstar_op2_dump_config(struct mstar_op2 *op2)
{
	unsigned htt, vtt;
	unsigned hsync_start, hsync_end;
	unsigned vsync_start, vsync_end;
	unsigned hfde_start, hfde_end;
	unsigned vfde_start, vfde_end;
	unsigned hde_start, hde_end;
	unsigned vde_start, vde_end;
	unsigned swap_b, swap_g, swap_r;
	unsigned output_mode;

	regmap_field_read(op2->htt, &htt);
	regmap_field_read(op2->vtt, &vtt);
	regmap_field_read(op2->hsync_start, &hsync_start);
	regmap_field_read(op2->hsync_end, &hsync_end);
	regmap_field_read(op2->vsync_start, &vsync_start);
	regmap_field_read(op2->vsync_end, &vsync_end);
	regmap_field_read(op2->hfde_start, &hfde_start);
	regmap_field_read(op2->hfde_end, &hfde_end);
	regmap_field_read(op2->vfde_start, &vfde_start);
	regmap_field_read(op2->vfde_end, &vfde_end);
	regmap_field_read(op2->hde_start, &hde_start);
	regmap_field_read(op2->hde_end, &hde_end);
	regmap_field_read(op2->vde_start, &vde_start);
	regmap_field_read(op2->vde_end, &vde_end);

	regmap_field_read(op2->swap_b, &swap_b);
	regmap_field_read(op2->swap_g, &swap_g);
	regmap_field_read(op2->swap_r, &swap_r);
	regmap_field_read(op2->output_mode, &output_mode);

	dev_info(op2->dev, "htt: %d, vtt: %d\n", htt, vtt);
	dev_info(op2->dev, "hsync start: %d, hsync end: %d\n", hsync_start, hsync_end);
	dev_info(op2->dev, "vsync start: %d, vsync end: %d\n", vsync_start, vsync_end);
	dev_info(op2->dev, "hfde start: %d, hfde end: %d\n", hfde_start, hfde_end);
	dev_info(op2->dev, "vfde start: %d, vfde end: %d\n", vfde_start, vfde_end);
	dev_info(op2->dev, "hde start: %d, hde end: %d\n", hde_start, hde_end);
	dev_info(op2->dev, "vde start: %d, vde end: %d\n", vde_start, vde_end);
	dev_info(op2->dev, "b swap: %d, g swap: %d, r swap: %d, mode: %d\n",
			swap_b, swap_g, swap_r, output_mode);
}

static int mstar_op2_enable_vblank(struct drm_crtc *crtc)
{
	struct mstar_drv *drv = crtc->dev->dev_private;

	WARN_ON(!drv->top);

	if (likely(drv->top))
		mstar_top_enable_vblank(drv->top);

	return 0;
}

static void mstar_op2_disable_vblank(struct drm_crtc *crtc)
{
	struct mstar_drv *drv = crtc->dev->dev_private;

	WARN_ON(!drv->top);

	if (likely(drv->top))
		mstar_top_disable_vblank(drv->top);
}

static const struct drm_crtc_funcs mstar_op2_crtc_funcs = {
	.reset			= drm_atomic_helper_crtc_reset,
	.destroy		= drm_crtc_cleanup,
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.enable_vblank		= mstar_op2_enable_vblank,
	.disable_vblank		= mstar_op2_disable_vblank,
};

static void mstar_op2_mode_set_nofb(struct drm_crtc *crtc)
{
	struct mstar_op2 *op2 = crtc_to_op2(crtc);
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;

	dev_info(op2->dev, "set mode: %d(%d) x %d(%d)\n", mode->hdisplay, mode->htotal, mode->vdisplay, mode->vtotal);
	dev_info(op2->dev, "set mode: hsync s %d, hsync e %d\n", mode->hsync_start, mode->hsync_end);
	dev_info(op2->dev, "set mode: vsync s %d, vsync e %d\n", mode->vsync_start, mode->vsync_end);

	/* total area */
	regmap_field_write(op2->htt, mode->htotal -1);
	regmap_field_write(op2->vtt, mode->vtotal -1);

	/* active area */
	regmap_field_write(op2->hfde_start, 0);
	regmap_field_write(op2->hfde_end, mode->hdisplay - 1);
	regmap_field_write(op2->hde_start, 0);
	regmap_field_write(op2->hde_end, mode->hdisplay - 1);

	regmap_field_write(op2->vfde_start, 0);
	regmap_field_write(op2->vfde_end, mode->vdisplay - 1);
	regmap_field_write(op2->vde_start, 0);
	regmap_field_write(op2->vde_end, mode->vdisplay - 1);

	/* sync */
	regmap_field_write(op2->hsync_start, mode->hsync_start);
	regmap_field_write(op2->hsync_end, mode->hsync_end);
	regmap_field_write(op2->vsync_start, mode->vsync_start);
	regmap_field_write(op2->vsync_end, mode->vsync_end);

	mstar_op2_dump_config(op2);
}

static void mstar_op2_atomic_enable(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	drm_crtc_vblank_on(crtc);
}

static void mstar_op2_atomic_disable(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	drm_crtc_vblank_off(crtc);
}

static void mstar_op2_atomic_flush(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	struct drm_pending_vblank_event *event = crtc_state->event;

	if (event) {
		crtc_state->event = NULL;

		spin_lock_irq(&crtc->dev->event_lock);
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static const struct drm_crtc_helper_funcs mstar_op2_helper_funcs = {
	.mode_set_nofb	= mstar_op2_mode_set_nofb,
	.atomic_flush	= mstar_op2_atomic_flush,
	.atomic_enable	= mstar_op2_atomic_enable,
	.atomic_disable	= mstar_op2_atomic_disable,
};

static int mstar_op2_bind(struct device *dev, struct device *master,
			 void *data)
{
	struct mstar_op2 *op2 = dev_get_drvdata(dev);
	struct drm_device *drm = data;
	struct mstar_drv *drv = drm->dev_private;
	struct drm_plane *plane = NULL, *primary = NULL, *cursor = NULL;
	int ret;
	u32 output;

	drm_for_each_plane(plane, drm) {
		if (plane->type == DRM_PLANE_TYPE_PRIMARY)
			primary = plane;
		else if (plane->type == DRM_PLANE_TYPE_CURSOR)
			cursor = plane;
	}

	if (primary == NULL)
		return -ENODEV;

	ret = drm_crtc_init_with_planes(drm,
					&op2->drm_crtc,
					primary,
					cursor,
					&mstar_op2_crtc_funcs,
					"op2");
	if(ret)
		return ret;

	drm_crtc_helper_add(&op2->drm_crtc, &mstar_op2_helper_funcs);

	/* Try to work out what is connected, default to TTL */
	ret = of_property_read_u32(dev->of_node,"mstar,op2-output", &output);
	if (!ret && output != 0) {
		dev_info(op2->dev, "Forcing output port to %d\n", output);
		goto dsi_hdmi;
	}

	dev_info(op2->dev, "Trying to probe TTL output\n");
	/* set the port so the encoder can find us */
	op2->drm_crtc.port = of_graph_get_port_by_id(dev->of_node, 0);

	/* create a fake encoder for ttl output */
	ret = mstar_ttl_init(drm, dev->of_node);

	return ret;

dsi_hdmi:
	/* */
	printk("%s:%d - %d\n", __func__, __LINE__, ret);
	op2->drm_crtc.port = of_graph_get_port_by_id(dev->of_node, output);

	return 0;
}

static void mstar_op2_unbind(struct device *dev, struct device *master,
			    void *data)
{
	struct mstar_op2 *op2 = dev_get_drvdata(dev);

	drm_crtc_cleanup(&op2->drm_crtc);
}

static const struct component_ops mstar_op2_component_ops = {
	.bind	= mstar_op2_bind,
	.unbind	= mstar_op2_unbind,
};

static int mstar_op2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mstar_op2 *op2;
	struct regmap *regmap;
	void __iomem *base;
	u8 chanswap[16];
	int ret;

	op2 = devm_kzalloc(dev, sizeof(*op2), GFP_KERNEL);
	if (!op2)
		return -ENOMEM;

	op2->dev = dev;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio(dev, base, &mstar_op2_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/* horizontal settings */
	op2->htt = devm_regmap_field_alloc(dev, regmap, htt_field);
	op2->hsync_start = devm_regmap_field_alloc(dev, regmap, hsync_start_field);
	op2->hsync_end = devm_regmap_field_alloc(dev, regmap, hsync_end_field);
	op2->hfde_start = devm_regmap_field_alloc(dev, regmap, hfde_start_field);
	op2->hfde_end = devm_regmap_field_alloc(dev, regmap, hfde_end_field);
	op2->hde_start = devm_regmap_field_alloc(dev, regmap, hde_start_field);
	op2->hde_end = devm_regmap_field_alloc(dev, regmap, hde_end_field);

	/* vertical settings */
	op2->vtt = devm_regmap_field_alloc(dev, regmap, vtt_field);
	op2->vsync_start = devm_regmap_field_alloc(dev, regmap, vsync_start_field);
	op2->vsync_end = devm_regmap_field_alloc(dev, regmap, vsync_end_field);
	op2->vfde_start = devm_regmap_field_alloc(dev, regmap, vfde_start_field);
	op2->vfde_end = devm_regmap_field_alloc(dev, regmap, vfde_end_field);
	op2->vde_start = devm_regmap_field_alloc(dev, regmap, vde_start_field);
	op2->vde_end = devm_regmap_field_alloc(dev, regmap, vde_end_field);

	/* output control */
	op2->swap_b = devm_regmap_field_alloc(dev, regmap, output_swap_b);
	op2->swap_g = devm_regmap_field_alloc(dev, regmap, output_swap_g);
	op2->swap_r = devm_regmap_field_alloc(dev, regmap, output_swap_r);
	op2->output_mode = devm_regmap_field_alloc(dev, regmap, output_mode_field);
	op2->swap_ml = devm_regmap_field_alloc(dev, regmap, output_swap_ml_field);

	ret = of_property_read_variable_u8_array(dev->of_node, "mstar,op2-channelswap",
			chanswap, 1, ARRAY_SIZE(chanswap));
	if (ret > 0) {
		regmap_field_write(op2->swap_b, chanswap[0]);
		regmap_field_write(op2->swap_g, chanswap[1]);
		regmap_field_write(op2->swap_r, chanswap[2]);
	}
	/* no idea what this does, needed for dongshanpione screen */
	//regmap_field_write(op2->swap_ml, 0);

	/* Setup the color matrix, for now using values that the vendor code wrote */
	regmap_write(regmap, REG_COLOR_MATRIX_0, 0x077f);
	regmap_write(regmap, REG_COLOR_MATRIX_1, 0x04a9);
	regmap_write(regmap, REG_COLOR_MATRIX_2, 0x0000);
	regmap_write(regmap, REG_COLOR_MATRIX_3, 0x129c);
	regmap_write(regmap, REG_COLOR_MATRIX_4, 0x04a9);
	regmap_write(regmap, REG_COLOR_MATRIX_5, 0x1178);
	regmap_write(regmap, REG_COLOR_MATRIX_6, 0x1070);
	regmap_write(regmap, REG_COLOR_MATRIX_7, 0x04a6);
	regmap_write(regmap, REG_COLOR_MATRIX_8, 0x08bb);
	regmap_write(regmap, REG_COLOR_MATRIX_CTRL, 0xb);

	dev_set_drvdata(dev, op2);

	mstar_op2_dump_config(op2);

	return component_add(&pdev->dev, &mstar_op2_component_ops);
}

static int mstar_op2_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &mstar_op2_component_ops);
	return 0;
}

static const struct of_device_id mstar_op2_ids[] = {
	{
		.compatible = "sstar,ssd20xd-op2",
	},
	{},
};
MODULE_DEVICE_TABLE(of, mstar_op2_ids);

static struct platform_driver mstar_op2_driver = {
	.probe = mstar_op2_probe,
	.remove = mstar_op2_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = mstar_op2_ids,
	},
};
module_platform_driver(mstar_op2_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
