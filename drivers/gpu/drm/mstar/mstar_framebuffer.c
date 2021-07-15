// SPDX-License-Identifier: GPL-2.0-or-later

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>

#include "mstar_framebuffer.h"

static const struct drm_mode_config_funcs mstar_framebuffer_mode_config_funcs = {
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
	.fb_create		= drm_gem_fb_create,
};

void mstar_framebuffer_init(struct drm_device *drm)
{
	drm_mode_config_reset(drm);
	drm->mode_config.max_width = 8192;
	drm->mode_config.max_height = 8192;
	drm->mode_config.funcs = &mstar_framebuffer_mode_config_funcs;
}
