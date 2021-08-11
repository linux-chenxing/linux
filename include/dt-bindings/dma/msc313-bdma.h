/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020 Daniel Palmer <daniel@thingy.jp>
 */

#ifndef __DT_BINDINGS_DMA_MSC313_BDMA_DMA_H__
#define __DT_BINDINGS_DMA_MSC313_BDMA_DMA_H__

#define MSC313_BDMA_SLAVE_QSPI	0x5
#define MSC313_BDMA_SLAVE_PM51	0x6

// For the main bdma
#define MSC313_BDMA_P3_SLAVE_MSPI0 0x7
#define MSC313_BDMA_P3_SLAVE_MSPI1 0x10
// For the pspi bdmas?
#define MSC313_BDMA_P3_SLAVE_PSPI 0x9

#endif /* __DT_BINDINGS_DMA_MSC313_BDMA_DMA_H__ */
