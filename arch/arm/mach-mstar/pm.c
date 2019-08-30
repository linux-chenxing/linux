// SPDX-License-Identifier: GPL-2.0
/*
 */

#include <asm/suspend.h>
#include <asm/fncpy.h>
#include <asm/cacheflush.h>

#include <linux/suspend.h>
#include <linux/io.h>
#include <linux/genalloc.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#define COMPAT_PMSLEEP "mstar,msc313-pmsleep"

struct mstar_pm_info {
	u32 pmsleep;	// 0x0
	u32 pmgpio;	// 0x4
	u32 miu0;	// 0x8
	u32 miu1;	// 0xc
	u32 miu2;	// 0x10
	u32 pmuart;	// 0x14
};

#define MSTARV7_PM_WAKESRC		0x20
#define MSTARV7_PM_RESUMEADDR		0xec

#define MSTARV7_PM_SIZE			SZ_16K
#define MSTARV7_PM_INFO_OFFSET		0
#define MSTARV7_PM_INFO_SIZE		SZ_4K
#define MSTARV7_PM_SUSPEND_OFFSET	(MSTARV7_PM_INFO_OFFSET + MSTARV7_PM_INFO_SIZE)
#define MSTARV7_PM_SUSPEND_SIZE		SZ_4K

static struct mstar_pm_info __iomem *pm_info;
static void __iomem *pm_suspend_code;

static struct regmap *pmsleep;

extern void msc313_suspend_imi(struct mstar_pm_info *pm_info);
void (*msc313_suspend_imi_fn)(struct mstar_pm_info *pm_info);
extern void msc313_resume_imi(void);

static int msc313_suspend_ready(unsigned long ret)
{
	local_flush_tlb_all();
	flush_cache_all();
	msc313_suspend_imi_fn(pm_info);
	return 0;
}

static int msc313_suspend_enter(suspend_state_t state)
{
	switch (state)
	{
		case PM_SUSPEND_MEM:
			// Now prepare our wake up source
			//setbank	0x0
			//clearbits 0x10 0x16 //[1]:SAR, [2]:WOL, [4]:RTC
			regmap_write(pmsleep, MSTARV7_PM_WAKESRC, 0xe9);
			cpu_suspend(0, msc313_suspend_ready);
            	break;
        	default:
			return -EINVAL;
	}
    return 0;
}

static void msc313_suspend_finish(void)
{
}

/* sequence: begin, prepare, prepare_late, enter, wake, finish, end */
struct platform_suspend_ops msc313_suspend_ops = {
	.enter    = msc313_suspend_enter,
	.valid    = suspend_valid_only_mem,
	.finish   = msc313_suspend_finish,
};

int __init msc313_pm_init(void)
{
	int ret = 0;
	struct device_node *node;
	struct platform_device *pdev;
	struct gen_pool *imi_pool;
	void __iomem *imi_base;
	void __iomem *virt;
	phys_addr_t phys;
	unsigned int resume_pbase;

	pmsleep = syscon_regmap_lookup_by_compatible(COMPAT_PMSLEEP);
	if(!pmsleep)
		return -ENODEV;

	node = of_find_compatible_node(NULL, NULL, "mmio-sram");
	if (!node) {
		pr_warn("%s: failed to find imi node\n", __func__);
		return -ENODEV;
	}

	/*pdev = of_find_device_by_node(node);
	if (!pdev) {
		pr_warn("%s: failed to find imi device\n", __func__);
		ret = -ENODEV;
		goto put_node;
	}

	imi_pool = gen_pool_get(&pdev->dev, NULL);
	if (!imi_pool) {
		pr_warn("%s: imi pool unavailable!\n", __func__);
		ret = -ENODEV;
		goto put_node;
	}

	imi_base = gen_pool_alloc(imi_pool, MSTARV7_PM_SIZE);
	if (!imi_base) {
		pr_warn("%s: unable to alloc pm memory in imi!\n", __func__);
		ret = -ENOMEM;
		goto put_node;
	}*/

	phys = 0xa0000000;
	//phys = gen_pool_virt_to_phys(imi_pool, imi_base);
	virt = __arm_ioremap_exec(phys, MSTARV7_PM_SIZE, false);
	pm_info = (struct mstar_pm_info*) (virt + MSTARV7_PM_INFO_OFFSET);
	pm_suspend_code = virt + MSTARV7_PM_SUSPEND_OFFSET;

	node = of_find_compatible_node(NULL, NULL, COMPAT_PMSLEEP);
	pm_info->pmsleep = (u32) of_iomap(node, 0);
	pm_info->pmgpio	= (u32) ioremap(0x1f001e00, 0x200);
	pm_info->miu0	= (u32) ioremap(0x1f202000, 0x200);
	pm_info->miu1	= (u32) ioremap(0x1f202200, 0x200);
	pm_info->miu2	= (u32) ioremap(0x1f202400, 0x200);
	pm_info->pmuart	= (u32) ioremap(0x1f221000, 0x200);

	msc313_suspend_imi_fn = fncpy(pm_suspend_code,
			(void*)&msc313_suspend_imi, MSTARV7_PM_SUSPEND_SIZE);

	/* setup the resume addr for the bootrom */
	resume_pbase = __pa_symbol(msc313_resume_imi);
	regmap_write(pmsleep, MSTARV7_PM_RESUMEADDR, resume_pbase & 0xffff);
	regmap_write(pmsleep, MSTARV7_PM_RESUMEADDR + 4, (resume_pbase >> 16) & 0xffff);

	suspend_set_ops(&msc313_suspend_ops);

	printk("pm code is at %px, pm info is at %px, pmsleep is at %x, pmgpio is at %x\n",
			pm_suspend_code, pm_info, pm_info->pmsleep, pm_info->pmgpio);

put_node:
	of_node_put(node);

	return ret;
}
