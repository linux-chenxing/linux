/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef _SOC_MSTAR_RIUXIU_H_
#define _SOC_MSTAR_RIUXIU_H_

#include <linux/io.h>

/*
 * RIU accessors.
 *
 * These are for accessing split 32bit registers in MStar and
 * third party IPs.
 *
 * To access a split 32bit register in an MStar IP use
 * the _abs() version with the full address for the lower
 * half of the register.
 *
 * To access a split 32bit register for a third party IP where
 * the offset from the start of the registers needs to be
 * doubled to get the right address use the non-abs version.
 */
static inline u32 riu_readl_abs(void __iomem *base)
{
	return readw_relaxed(base + 4) << 16 | readw(base);
}

static inline u32 riu_readl(void __iomem *base, unsigned int offset)
{
	return riu_readl_abs(base + (offset * 2));
}

static inline u32 riu_readl_relaxed_abs(void __iomem *base)
{
	return readw_relaxed(base + 4) << 16 | readw_relaxed(base);
}

static inline u32 riu_readl_relaxed(void __iomem *base, unsigned int offset)
{
	return riu_readl_relaxed_abs(base + (offset * 2));
}

static inline void riu_writel_relaxed_abs(void __iomem *base, u32 value)
{
	/*
	 * Do not change this order. For EMAC at least the write order
	 * must be the lower half and then the upper half otherwise it
	 * doesn't work because writing the transmit buffer register no
	 * longer triggers sending a frame.
	 */
	writew_relaxed(value, base);
	writew_relaxed(value >> 16, base + 4);
}

static inline void riu_writel_relaxed(void __iomem *base, unsigned int offset, u32 value)
{
	riu_writel_relaxed_abs(base + (offset * 2), value);
}

static inline void riu_writel_abs(void __iomem *base, u32 value)
{
	/*
	 * Do not change this order. For EMAC at least the write order
	 * must be the lower half and then the upper half otherwise it
	 * doesn't work because writing the transmit buffer register no
	 * longer triggers sending a frame.
	 */
	writew(value, base);
	writew_relaxed(value >> 16, base + 4);
}

static inline void riu_writel(void __iomem *base, unsigned int offset, u32 value)
{
	riu_writel_abs(base + (offset * 2), value);
}

/*
 * XIU accessors, use these whenever possible for third party IPs.
 * For MStar IPs just use plain readl(), writel() etc.
 */

static inline u32 xiu_readl(void __iomem *base, unsigned int offset)
{
	return readl(base + (offset * 2));
}

static inline u32 xiu_readl_relaxed(void __iomem *base, unsigned int offset)
{
	return readl_relaxed(base + (offset * 2));
}

static inline void xiu_writel(void __iomem *base, unsigned int offset, u32 value)
{

	writel_relaxed(value, base + (offset * 2));
}

static inline void xiu_writel_relaxed(void __iomem *base, unsigned int offset, u32 value)
{
	writel_relaxed(value, base + (offset * 2));
}

#endif
