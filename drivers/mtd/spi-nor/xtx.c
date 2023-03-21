// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022, Carl Richard Theodor Schneider <dev.linux@xxxxxxxxxxx>
 * Copyright (C) 2022, Reimu NotMoe <reimu@sudomaker.com>
 */

// Salvaged from:
// https://www.spinics.net/lists/kernel/msg4227361.html

// It's a SHAME that the upstream didn't accept it.

#include <linux/mtd/spi-nor.h>

#include "core.h"

static const struct flash_info xtx_parts[] = {
	/* XTX (Shenzhen Xin Tian Xia Tech) */
	{ "xt25f16b-kgd", INFO(0x8b4015, 0, 64 * 1024, 32)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ |
			SPI_NOR_QUAD_READ) },
	{ "xt25f16b", INFO(0x0b4015, 0, 64 * 1024, 32)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ |
			SPI_NOR_QUAD_READ) },
	{ "xt25f32b", INFO(0x0b4016, 0, 64 * 1024, 64)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ |
			SPI_NOR_QUAD_READ) },
	{ "xt25f128b", INFO(0x0b4018, 0, 64 * 1024, 256)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ |
			SPI_NOR_QUAD_READ) },
};

const struct spi_nor_manufacturer spi_nor_xtx = {
	.name = "xtx",
	.parts = xtx_parts,
	.nparts = ARRAY_SIZE(xtx_parts),
};
