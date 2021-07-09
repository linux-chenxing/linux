/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef _MSTAR_TOP_H_
#define _MSTAR_TOP_H_

struct mstar_top {
	struct drm_device *drm_device;
	struct regmap_field *reset;
	struct regmap_field *vsync_pos_flag;
	struct regmap_field *vsync_pos_mask;
	struct regmap_field *mace_src;
};

void mstar_top_enable_vblank(struct mstar_top *top);
void mstar_top_disable_vblank(struct mstar_top *top);

#endif /* _MSTAR_TOP_H_ */
