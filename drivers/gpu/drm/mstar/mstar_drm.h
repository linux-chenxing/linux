/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef _MSTAR_DRM_H_
#define _MSTAR_MSTAR_DRM_H_

struct mstar_ge;
struct mstar_top;

struct mstar_drv {
	struct device *dev;
	struct mstar_top* top;
	struct mstar_ge* ge;
};

#endif /* _MSTAR_DRM_H_ */
