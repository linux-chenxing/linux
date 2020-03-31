/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_plane.h>
#include <linux/component.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>

#include "mstar_drm.h"
#include "mstar_ge.h"
#include "mstar_gop.h"

#define DRIVER_NAME "mstar-gop"

// Needs to be confirmed..
#define WINDOW_START 0x200

#define MSTAR_GOP_REG_CONFIG		0x00

static struct reg_field gop_rst_field = REG_FIELD(MSTAR_GOP_REG_CONFIG, 0, 0);
/* 1 for progressive, 0 for interlaced */
static struct reg_field gop_scan_type_field = REG_FIELD(MSTAR_GOP_REG_CONFIG, 3, 3);
/* 1 for yuv, 0 for rgb */
static struct reg_field gop_colorspace_field = REG_FIELD(MSTAR_GOP_REG_CONFIG, 10, 10);
static struct reg_field gop_hsmask_field = REG_FIELD(MSTAR_GOP_REG_CONFIG, 14, 14);
static struct reg_field gop_alphainv_field = REG_FIELD(MSTAR_GOP_REG_CONFIG, 15, 15);
// bits 12 and 13 are meant to control flipping but they don't seem to be writable on ssd202d


/* Length of the global window in pixels/2 */
static struct reg_field stretch_window_size_h_field = REG_FIELD(0xc0, 0, 11);
#define STRETCH_WINDOW_SIZE_H_SHIFT 1
// sets the global window size
static struct reg_field stretch_window_size_v_field = REG_FIELD(0xc4, 0, 11);
// screen gets torn
static struct reg_field stretch_window_coordinate_h_field = REG_FIELD(0xc8, 0, 11);
// screen shifts down
static struct reg_field stretch_window_coordinate_v_field = REG_FIELD(0xd0, 0, 11);

//

// triggers updating the registers
static struct reg_field gop_commit_all_field = REG_FIELD(0x1fc, 8, 8);


// window registers
//0x24 - line length
//0x28 - ?? starts off as 0xff with original fw


static struct reg_field gop_dst_field = REG_FIELD(MSTAR_GOP_REG_DST_RI, 0, 2);

struct mstar_gop_data {
	const uint32_t *formats;
	const unsigned int num_formats;
	const enum drm_plane_type type;
	const unsigned int num_windows;
	const unsigned int addr_shift;
	const bool has_stretching;

	/*
	 * Register offsets for registers that are at different locations
	 * depending on the gop version/instance.
	 */
	const unsigned int offset_hstart;
	const unsigned int offset_hend;
	const unsigned int offset_vstart;
	const unsigned int offset_vend;
	const unsigned int offset_pitch;

	/*
	 * callbacks to convert DRM color formats
	 * to GOP formats
	 */
	int (*drm_color_to_gop)(u32 fourcc);
	int (*gop_color_to_drm)(void);
};

struct mstar_gop;

struct mstar_gop_window {
	struct mstar_gop *gop;
	struct drm_plane drm_plane;
	struct regmap_field *en;
	struct regmap_field *format;
	struct regmap_field *addrl;
	struct regmap_field *addrh;
	struct regmap_field *hstart;
	struct regmap_field *hend;
	struct regmap_field *vstart;
	struct regmap_field *vend;
	struct regmap_field *pitch;
};

#define plane_to_gop_window(plane) container_of(plane, struct mstar_gop_window, drm_plane)

struct mstar_gop {
	struct device *dev;
	struct drm_device *drm_device;
	struct clk *fclk; /* vendor code says this is only needed when setting the palette */
	const struct mstar_gop_data *data;

	struct regmap_field *rst;
	struct regmap_field *scan_type;
	struct regmap_field *colorspace;
	struct regmap_field *dst;

	struct regmap_field *stretch_window_size_h;
	struct regmap_field *stretch_window_size_v;
	struct regmap_field *stretch_window_coordinate_h;
	struct regmap_field *stretch_window_coordinate_v;

	struct regmap_field *commit_all;

	struct mstar_gop_window windows[];
};

static const struct regmap_config mstar_gop_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static irqreturn_t mstar_gop_irq(int irq, void *data)
{
	return IRQ_HANDLED;
}

static const char* dsts[] = {
	"ip_main",
	"ip_sub",
	"op",
	"mvop",
	"sub_mvop",
	"unknown",
	"frc",
	"unknown",
};

static void mstar_gop_dump(struct mstar_gop *gop)
{
	unsigned int val, rst, scan_type, colorspace, dst;
	unsigned int stretchh, stretchv, coordinateh, coordinatev;
	int i;

	regmap_field_read(gop->rst, &rst);
	regmap_field_read(gop->scan_type, &scan_type);
	regmap_field_read(gop->colorspace, &colorspace);
	regmap_field_read(gop->dst, &dst);

	regmap_field_read(gop->stretch_window_size_h, &stretchh);
	stretchh = stretchh << STRETCH_WINDOW_SIZE_H_SHIFT;
	regmap_field_read(gop->stretch_window_size_v, &stretchv);
	regmap_field_read(gop->stretch_window_coordinate_h, &coordinateh);
	regmap_field_read(gop->stretch_window_coordinate_v, &coordinatev);

	dev_info(gop->dev,
		 "rst: %d\n"
		 "scan_type: %s\n"
		 "colorspace: %s\n"
		 "dst: %s\n"
		 "stretch window: %d x %d (%d:%d)\n",
		 rst,
		 scan_type ? "progressive" : "interlaced",
		 colorspace ? "yuv" : "rgb",
		 dsts[dst],
		 stretchh, stretchv, coordinateh, coordinatev);

	for (i = 0; i < gop->data->num_windows; i++){
		struct mstar_gop_window *window = &gop->windows[i];
		u32 en, format, addr, hstart, hend, vstart, vend, pitch;

		regmap_field_read(window->en, &en);
		regmap_field_read(window->format, &format);
		regmap_field_read(window->addrh, &val);
		addr = val << 16;
		regmap_field_read(window->addrl, &val);
		addr |= val;
		addr = addr << gop->data->addr_shift;

		regmap_field_read(window->hstart, &hstart);
		regmap_field_read(window->hend, &hend);
		regmap_field_read(window->vstart, &vstart);
		regmap_field_read(window->vend, &vend);
		regmap_field_read(window->pitch, &pitch);

		dev_info(gop->dev,
			"window 1:\n"
			"en: %d\n"
			"format: %d\n"
			"addr: 0x%08x\n"
			"hstart: %d, hend %d, vstart: %d, vend: %d\n"
			"pitch: %d\n",
			en,
			format,
			addr,
			hstart * 4, hend * 4, vstart, vend,
			pitch << gop->data->addr_shift
		);

	}

}


static void mstar_gop_reset(struct mstar_gop *gop)
{
	regmap_field_force_write(gop->rst, 1);
	mdelay(10);
	regmap_field_force_write(gop->rst, 0);
	mdelay(10);

	mstar_gop_dump(gop);
}

static int gop_ssd20xd_gop0_drm_color_to_gop(u32 fourcc)
{
	switch(fourcc){
	case DRM_FORMAT_ARGB1555:
		return 0x3;
	case DRM_FORMAT_ARGB4444:
		return 0x4;
	};

	return -ENOTSUPP;
}

static int gop_ssd20xd_gop0_gop_color_to_drm(void)
{
	return -ENOTSUPP;
}

static int gop_ssd20xd_gop1_drm_color_to_gop(u32 fourcc)
{
	switch(fourcc){
	case DRM_FORMAT_ARGB1555:
		return 0x0;
	case DRM_FORMAT_RGB565:
		return 0x1;
	};

	return -ENOTSUPP;
}

static int gop_ssd20xd_gop1_gop_color_to_drm(void)
{
	return -ENOTSUPP;
}

static int gop_plane_atomic_check(struct drm_plane *plane,
				  struct drm_atomic_state *state)
{

	return 0;
}

static void gop_plane_atomic_update(struct drm_plane *plane,
				    struct drm_atomic_state *state)
{
	struct mstar_gop_window *window = plane_to_gop_window(plane);
	struct mstar_gop *gop = window->gop;
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = new_state->fb;
	struct drm_gem_cma_object *gem = drm_fb_cma_get_gem_obj(fb, 0);
	u32 addr;

	if (!gem)
		return;

	// global window first
	/* Not sure why but the output colour space needs to be YUV */
	regmap_field_force_write(gop->colorspace, 1);

	regmap_field_write(gop->stretch_window_size_h,
			   new_state->crtc_w >> STRETCH_WINDOW_SIZE_H_SHIFT);
	regmap_field_write(gop->stretch_window_size_v, new_state->crtc_h);
	regmap_field_write(gop->stretch_window_coordinate_h, new_state->crtc_x);
	regmap_field_write(gop->stretch_window_coordinate_v, new_state->crtc_y);

	// gop window

	regmap_field_write(window->en, new_state->crtc ? 1 : 0);
	regmap_field_write(window->format, gop->data->drm_color_to_gop(fb->format->format));

	regmap_field_write(window->hstart, new_state->crtc_x);
	regmap_field_write(window->vstart, new_state->crtc_y);

	// This seems to be the same as pitch?
	regmap_field_write(window->hend, fb->pitches[0] >> gop->data->addr_shift);
	regmap_field_write(window->vend, new_state->crtc_y + new_state->crtc_h);

	regmap_field_write(window->pitch, fb->pitches[0] >> gop->data->addr_shift);

	addr = gem->paddr >> window->gop->data->addr_shift;

	regmap_field_write(window->addrh, addr >> 16);
	regmap_field_write(window->addrl, addr);

	regmap_field_force_write(window->gop->commit_all, 1);

	mstar_gop_dump(window->gop);
}

static const struct drm_plane_helper_funcs gop_plane_helper_funcs = {
	.prepare_fb = drm_gem_plane_helper_prepare_fb,
	.atomic_check = gop_plane_atomic_check,
	.atomic_update = gop_plane_atomic_update,
};

static const struct drm_plane_funcs gop_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

static const uint64_t mop_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static int mstar_gop_bind(struct device *dev, struct device *master,
			 void *data)
{
	struct mstar_gop *gop = dev_get_drvdata(dev);
	struct drm_device *drm_device = data;
	int i, ret;

	gop->drm_device = drm_device;

	for (i = 0; i < gop->data->num_windows; i++){
		struct mstar_gop_window *window = &gop->windows[i];
		ret = drm_universal_plane_init(drm_device,
					     &window->drm_plane,
					     0,
					     &gop_plane_funcs,
					     gop->data->formats,
					     gop->data->num_formats,
					     mop_format_modifiers,
					     gop->data->type,
					     NULL);
		if(ret)
			return ret;

		drm_plane_helper_add(&window->drm_plane, &gop_plane_helper_funcs);
	}

	return 0;
}

static void mstar_gop_unbind(struct device *dev, struct device *master, void *data)
{
}

static const struct component_ops mstar_gop_ops = {
	.bind	= mstar_gop_bind,
	.unbind	= mstar_gop_unbind,
};

static int mstar_gop_probe(struct platform_device *pdev)
{
	const struct mstar_gop_data *match_data;
	struct device *dev = &pdev->dev;
	struct regmap *regmap;
	struct mstar_gop *gop;
	__iomem void *regs;
	int i,irq, ret;

	match_data = of_device_get_match_data(dev);
	if (!match_data)
		return -EINVAL;

	gop = devm_kzalloc(dev, struct_size(gop, windows, match_data->num_windows), GFP_KERNEL);
	if (!gop)
		return -ENOMEM;

	gop->data = match_data;
	gop->dev = dev;

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	regmap = devm_regmap_init_mmio(dev, regs, &mstar_gop_regmap_config);
	if(IS_ERR(regmap))
		return PTR_ERR(regmap);

	gop->rst = regmap_field_alloc(regmap, gop_rst_field);
	gop->scan_type = regmap_field_alloc(regmap, gop_scan_type_field);
	gop->colorspace = regmap_field_alloc(regmap, gop_colorspace_field);
	gop->dst = regmap_field_alloc(regmap, gop_dst_field);

	gop->stretch_window_size_h = regmap_field_alloc(regmap, stretch_window_size_h_field);
	gop->stretch_window_size_v = regmap_field_alloc(regmap, stretch_window_size_v_field);
	gop->stretch_window_coordinate_h = regmap_field_alloc(regmap, stretch_window_coordinate_h_field);
	gop->stretch_window_coordinate_v = regmap_field_alloc(regmap, stretch_window_coordinate_v_field);

	gop->commit_all = regmap_field_alloc(regmap, gop_commit_all_field);

	for (i = 0; i < match_data->num_windows; i++){
		struct mstar_gop_window *window = &gop->windows[i];
		unsigned int winoffset = WINDOW_START + 0x40 * i;
		struct reg_field en_field = REG_FIELD(winoffset + 0, 0, 0);
		struct reg_field format_field = REG_FIELD(winoffset + 0, 4, 7);
		struct reg_field addrl_field = REG_FIELD(winoffset + 0x4, 0, 15);
		struct reg_field addrh_field = REG_FIELD(winoffset + 0x8, 0, 11);
		struct reg_field hstart_field = REG_FIELD(winoffset + match_data->offset_hstart, 0, 15);
		struct reg_field hend_field = REG_FIELD(winoffset + match_data->offset_hend, 0, 15);
		struct reg_field vstart_field = REG_FIELD(winoffset + match_data->offset_vstart, 0, 15);
		struct reg_field vend_field = REG_FIELD(winoffset + match_data->offset_vend, 0, 15);
		struct reg_field pitch_field = REG_FIELD(winoffset + match_data->offset_pitch, 0, 10);

		window->gop = gop;

		window->en = devm_regmap_field_alloc(dev, regmap, en_field);
		window->format = devm_regmap_field_alloc(dev, regmap, format_field);
		window->addrl = devm_regmap_field_alloc(dev, regmap, addrl_field);
		window->addrh = devm_regmap_field_alloc(dev, regmap, addrh_field);

		window->hstart = devm_regmap_field_alloc(dev, regmap, hstart_field);
		window->hend = devm_regmap_field_alloc(dev, regmap, hend_field);
		window->vstart = devm_regmap_field_alloc(dev, regmap, vstart_field);
		window->vend = devm_regmap_field_alloc(dev, regmap, vend_field);
		window->pitch = devm_regmap_field_alloc(dev, regmap, pitch_field);
	}

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!irq){
		dev_info(gop->dev, "no interrupt provided");
		goto no_irq;
	}
	else {
		ret = devm_request_irq(&pdev->dev, irq, mstar_gop_irq, IRQF_SHARED,
			dev_name(&pdev->dev), gop);
		if (ret)
			return ret;
	}

no_irq:
	gop->fclk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(gop->fclk)) {
		//return PTR_ERR(gop->fclk);
	}

	dev_set_drvdata(dev, gop);

	mstar_gop_reset(gop);

	return component_add(&pdev->dev, &mstar_gop_ops);
}

static int mstar_gop_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &mstar_gop_ops);
	return 0;
}

static const uint32_t ssd20xd_gop0_formats[] = {
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_ARGB1555,
};

static const uint32_t ssd20xd_gop1_formats[] = {
	DRM_FORMAT_YUV422,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_ARGB1555,
};

static const struct mstar_gop_data ssd20xd_gop0_data = {
	.formats = ssd20xd_gop0_formats,
	.num_formats = ARRAY_SIZE(ssd20xd_gop0_formats),
	.type = DRM_PLANE_TYPE_CURSOR,
	.num_windows = 1,
	.addr_shift = 4,
	.has_stretching = false,
	.offset_hstart = 0xc,
	.offset_hend = 0x10,
	.offset_vstart = 0x14,
	.offset_vend = 0x18,
	.offset_pitch = 0x1c,
	.drm_color_to_gop = gop_ssd20xd_gop0_drm_color_to_gop,
	.gop_color_to_drm = gop_ssd20xd_gop0_gop_color_to_drm,
};

static const struct mstar_gop_data ssd20xd_gop1_data = {
	.formats = ssd20xd_gop1_formats,
	.num_formats = ARRAY_SIZE(ssd20xd_gop1_formats),
	.type = DRM_PLANE_TYPE_PRIMARY,
	.num_windows = 1,
	.addr_shift = 4,
	.has_stretching = true,
	.offset_hstart = 0x10, // confirmed
	.offset_hend = 0x14, // confirmed
	.offset_vstart = 0x18, // confirmed
	.offset_vend = 0x20, // confirmed
	.offset_pitch = 0x24,
	.drm_color_to_gop = gop_ssd20xd_gop1_drm_color_to_gop,
	.gop_color_to_drm = gop_ssd20xd_gop1_gop_color_to_drm,
};

static const struct of_device_id mstar_gop_dt_ids[] = {
	{
		.compatible = "sstar,ssd20xd-gop0",
		.data = &ssd20xd_gop0_data,
	},
	{
		.compatible = "sstar,ssd20xd-gop1",
		.data = &ssd20xd_gop1_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, mstar_gop_dt_ids);

static struct platform_driver mstar_gop_driver = {
	.probe = mstar_gop_probe,
	.remove = mstar_gop_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = mstar_gop_dt_ids,
	},
};

module_platform_driver(mstar_gop_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
