// SPDX-License-Identifier: GPL-2.0
/*
 * Feels like i'm acting in a totally different way than most MIPS platforms
 *  out there...
 */

#include <linux/irqchip.h>
#include <linux/of_fdt.h>
#include <linux/of_clk.h>
#include <asm/bootinfo.h>
#include <asm/time.h>
#include <asm/prom.h>

const char *get_system_type(void)
{
	return "MStar MIPS SoC [Linux-ChenXing]";
}

/* TODO, do it better way */
void __init plat_time_init(void)
{
	void __iomem *anamisc = (void *)KSEG1ADDR(0x1f221800);
	unsigned long freq;

	of_clk_init(NULL);

	freq = 12000000
		/* / (1 << (readw(anamisc+0x64) & 0x3)) */
		* (1 << (readw(anamisc+0x64) >> 2 & 0x3))
		* readw(anamisc+0x68)
	;

	printk("CPU clock frequency: %ld MHz\n", freq / 1000000);

	mips_hpt_frequency = freq / 2;
	write_c0_compare(read_c0_count());

	timer_probe();
}

void __init plat_mem_setup(void)
{
	void *dtb;

	set_io_port_base((unsigned long) KSEG1);

	dtb = get_fdt();
	if (dtb == NULL)
		panic("no dtb found");

	__dt_setup_arch(dtb);
}

void __init prom_init(void)
{
#if defined(CONFIG_MIPS_MT_SMP)
	if (register_vsmp_smp_ops())
		panic("failed to register_vsmp_smp_ops()");
#endif
}

void __init arch_init_irq(void)
{
	irqchip_init();
}
