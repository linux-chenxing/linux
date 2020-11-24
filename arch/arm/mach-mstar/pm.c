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

#include <soc/mstar/pmsleep.h>

#define COMPAT_PMSLEEP	"mstar,msc313-pmsleep"
#define COMPAT_MIU	"mstar,msc313-miu"

#define MSTARV7_PM_SIZE			SZ_16K
#define MSTARV7_PM_INFO_OFFSET		0
#define MSTARV7_PM_INFO_SIZE		SZ_4K
#define MSTARV7_PM_SUSPEND_OFFSET	(MSTARV7_PM_INFO_OFFSET + MSTARV7_PM_INFO_SIZE)
#define MSTARV7_PM_SUSPEND_SIZE		SZ_4K

struct mstar_pm_info {
	u32 pmsleep;	// 0x0
	u32 pmgpio;	// 0x4
	u32 miu_ana;	// 0x8
	u32 miu_dig;	// 0xc
	u32 miu_dig1;	// 0x10
	u32 pmuart;	// 0x14
};

static struct mstar_pm_info __iomem *pm_info;
static void __iomem *pm_suspend_code;

static struct regmap *pmsleep;

extern void msc313_suspend_imi(struct mstar_pm_info *pm_info);
extern void msc313_resume_imi(void);
static void (*msc313_suspend_imi_fn)(struct mstar_pm_info *pm_info);

static int msc313_suspend_ready(unsigned long ret)
{
	local_flush_tlb_all();
	flush_cache_all();
	msc313_suspend_imi_fn(pm_info);
	return 0;
}

static int msc313_suspend_enter(suspend_state_t state)
{
	switch (state){
	case PM_SUSPEND_MEM:
		regmap_update_bits(pmsleep, MSTAR_PMSLEEP_REG24,
			MSTAR_PMSLEEP_REG24_POWEROFF, MSTAR_PMSLEEP_REG24_POWEROFF);
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
static const struct platform_suspend_ops msc313_suspend_ops = {
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

	pdev = of_find_device_by_node(node);
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
	}

	phys = gen_pool_virt_to_phys(imi_pool, imi_base);
	virt = __arm_ioremap_exec(phys, MSTARV7_PM_SIZE, false);
	pm_info = (struct mstar_pm_info*) (virt + MSTARV7_PM_INFO_OFFSET);
	pm_suspend_code = virt + MSTARV7_PM_SUSPEND_OFFSET;

	node = of_find_compatible_node(NULL, NULL, COMPAT_PMSLEEP);
	pm_info->pmsleep = (u32) of_iomap(node, 0);
	pm_info->pmgpio	= (u32) ioremap(0x1f001e00, 0x200);

	node = of_find_compatible_node(NULL, NULL, COMPAT_MIU);
	pm_info->miu_ana  = (u32) of_iomap(node, 0);
	pm_info->miu_dig  = (u32) of_iomap(node, 1);
	pm_info->miu_dig1 = (u32) of_iomap(node, 2);
	pm_info->pmuart	  = (u32) ioremap(0x1f221000, 0x200);

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
