// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021, Daniel Palmer<daniel@thingy.jp>
 */

#include <linux/mtd/spi-nor.h>

#include "core.h"

static const struct flash_info zbit_parts[] = {
	/* zbit */
	{ "zb25vq128", INFO(0x5e4018, 0, 64 * 1024, 256)
		       FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
		       NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)},
};

const struct spi_nor_manufacturer spi_nor_zbit = {
	.name = "zbit",
	.parts = zbit_parts,
	.nparts = ARRAY_SIZE(zbit_parts),
};
