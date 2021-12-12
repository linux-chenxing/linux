// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Daniel Palmer <daniel@thingy.jp>
 *
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mtd/spinand.h>

#define SPINAND_MFR_XINCUN			0x9C

static SPINAND_OP_VARIANTS(read_cache_variants,
		SPINAND_PAGE_READ_FROM_CACHE_QUADIO_OP(0, 2, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_X4_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_DUALIO_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_X2_OP(0, 1, NULL, 0),
		SPINAND_PAGE_READ_FROM_CACHE_OP(false, 0, 1, NULL, 0));

static SPINAND_OP_VARIANTS(write_cache_variants,
		SPINAND_PROG_LOAD(true, 0, NULL, 0));

static SPINAND_OP_VARIANTS(update_cache_variants,
		SPINAND_PROG_LOAD(false, 0, NULL, 0));

static int xcsp1aawhnt_ooblayout_ecc(struct mtd_info *mtd, int section,
					struct mtd_oob_region *region)
{
	if (section > 3)
		return -ERANGE;

	/* ECC is not user accessible */
	region->offset = 0;
	region->length = 0;

	return 0;
}

static int xcsp1aawhnt_ooblayout_free(struct mtd_info *mtd, int section,
				    struct mtd_oob_region *region)
{
	if (section > 3)
		return -ERANGE;

	/*
	 * No ECC data is stored in the accessible OOB so the full 16 bytes
	 * of each spare region is available to the user. Apparently also
	 * covered by the internal ECC.
	 */
	if (section) {
		region->offset = 16 * section;
		region->length = 16;
	} else {
		/* First byte in spare0 area is used for bad block marker */
		region->offset = 1;
		region->length = 15;
	}

	return 0;
}

static const struct mtd_ooblayout_ops xcsp1aawhnt_ooblayout = {
	.ecc = xcsp1aawhnt_ooblayout_ecc,
	.free = xcsp1aawhnt_ooblayout_free,
};

static const struct spinand_info xincun_spinand_table[] = {
	SPINAND_INFO("XCSP1AAWH-NT",
		     SPINAND_ID(SPINAND_READID_METHOD_OPCODE_ADDR, 0x01),
		     NAND_MEMORG(1, 2048, 128, 64, 1024, 20, 1, 1, 1),
		     NAND_ECCREQ(8, 512),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
					      &write_cache_variants,
					      &update_cache_variants),
		     SPINAND_HAS_QE_BIT,
		     SPINAND_ECCINFO(&xcsp1aawhnt_ooblayout,
				     NULL)),
};

static const struct spinand_manufacturer_ops xincun_spinand_manuf_ops = {
};

const struct spinand_manufacturer xincun_spinand_manufacturer = {
	.id = SPINAND_MFR_XINCUN,
	.name = "XINCUN",
	.chips = xincun_spinand_table,
	.nchips = ARRAY_SIZE(xincun_spinand_table),
	.ops = &xincun_spinand_manuf_ops,
};
