// SPDX-License-Identifier: GPL-2.0-only
/*
 * Spin Table SMP initialisation
 *
 * Copyright (C) 2013 ARM Ltd.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/smp.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>

#include <asm/cacheflush.h>
#include <asm/cpu_ops.h>
#include <asm/cputype.h>
#include <asm/io.h>
#include <asm/smp_plat.h>

extern void secondary_holding_pen(void);
volatile unsigned long __section(".mmuoff.data.read")
secondary_holding_pen_release = INVALID_HWID;

static phys_addr_t cpu_release_addr[NR_CPUS];

/*
 * Write secondary_holding_pen_release in a way that is guaranteed to be
 * visible to all observers, irrespective of whether they're taking part
 * in coherency or not.  This is necessary for the hotplug code to work
 * reliably.
 */
static void write_pen_release(u64 val)
{
	void *start = (void *)&secondary_holding_pen_release;
	unsigned long size = sizeof(secondary_holding_pen_release);

	secondary_holding_pen_release = val;
	dcache_clean_inval_poc((unsigned long)start, (unsigned long)start + size);
}


static int smp_spin_table_cpu_init(unsigned int cpu)
{
	struct device_node *dn;
	int ret;

	dn = of_get_cpu_node(cpu, NULL);
	if (!dn)
		return -ENODEV;

	/*
	 * Determine the address from which the CPU is polling.
	 */
	ret = of_property_read_u64(dn, "cpu-release-addr",
				   &cpu_release_addr[cpu]);
	if (ret)
		pr_err("CPU %d: missing or invalid cpu-release-addr property\n",
		       cpu);

	of_node_put(dn);

	return ret;
}

static int smp_spin_table_cpu_prepare(unsigned int cpu)
{
	__le64 __iomem *release_addr;
	phys_addr_t pa_holding_pen = __pa_symbol(secondary_holding_pen);

	if (!cpu_release_addr[cpu])
		return -ENODEV;

	/*
	 * The cpu-release-addr may or may not be inside the linear mapping.
	 * As ioremap_cache will either give us a new mapping or reuse the
	 * existing linear mapping, we can use it to cover both cases. In
	 * either case the memory will be MT_NORMAL.
	 */
	release_addr = ioremap_cache(cpu_release_addr[cpu],
				     sizeof(*release_addr));
	if (!release_addr)
		return -ENOMEM;

	/*
	 * We write the release address as LE regardless of the native
	 * endianness of the kernel. Therefore, any boot-loaders that
	 * read this address need to convert this address to the
	 * boot-loader's endianness before jumping. This is mandated by
	 * the boot protocol.
	 */
	writeq_relaxed(pa_holding_pen, release_addr);
	dcache_clean_inval_poc((__force unsigned long)release_addr,
			    (__force unsigned long)release_addr +
				    sizeof(*release_addr));

	/*
	 * Send an event to wake up the secondary CPU.
	 */
	sev();

	iounmap(release_addr);

	return 0;
}


#ifdef CONFIG_HOTPLUG_CPU
/*
 * LAB ONLY: CPU offline/online for Apple t6000 (j314s) on spin-table. A dying
 * core asks PMGR to stop it and enters deep WFI with power-down enabled (as
 * m1n1's smp_stop_cpu/cpu_sleep do). Bringing it back asks PMGR to start it:
 * the core re-enters through RVBAR, which still points at the resident m1n1,
 * whose secondary path re-runs its CPU init and jumps back into the holding
 * pen through the spin table. Only one core may restart at a time (the
 * installed m1n1 shares one reset stack).
 */
static bool apple_hotplug;
core_param(apple_hotplug, apple_hotplug, bool, 0644);

#define APPLE_T6000_CPU_START	0x28e0d4000UL
#define APPLE_T6000_IMPL_BASE	0x210050000UL

static void __iomem *apple_cpu_start_base;
static bool apple_cpu_stopped[NR_CPUS];

static bool apple_hotplug_ok(void)
{
	return apple_hotplug && of_machine_is_compatible("apple,t6000");
}

static unsigned long apple_impl_reg(u64 mpidr)
{
	return APPLE_T6000_IMPL_BASE + MPIDR_AFFINITY_LEVEL(mpidr, 1) * 0x1000000UL +
	       MPIDR_AFFINITY_LEVEL(mpidr, 0) * 0x100000UL;
}

static bool smp_spin_table_cpu_can_disable(unsigned int cpu)
{
	return apple_hotplug_ok();
}

static int smp_spin_table_cpu_disable(unsigned int cpu)
{
	if (!apple_hotplug_ok())
		return -EOPNOTSUPP;
	if (!apple_cpu_start_base)
		apple_cpu_start_base = ioremap(APPLE_T6000_CPU_START, PAGE_SIZE);
	return apple_cpu_start_base ? 0 : -ENOMEM;
}

static void smp_spin_table_cpu_die(unsigned int cpu)
{
	u64 mpidr = cpu_logical_map(cpu);
	u32 cl = MPIDR_AFFINITY_LEVEL(mpidr, 1), core = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	u64 cyc;

	writel_relaxed(BIT(4 * cl + core), apple_cpu_start_base + 0x0);
	dsb(sy);

	/*
	 * Single core, cluster stays up: like m1n1's cpu_sleep(false), only this
	 * core's CYC_OVRD changes (ACC_OVRD is cluster-wide). FIQ/IRQ mode 2,
	 * WFI mode 3 (power down), WFI retention disabled.
	 */
	asm volatile("mrs %0, s3_5_c15_c5_0" : "=r"(cyc));
	cyc &= ~(GENMASK_ULL(23, 20) | GENMASK_ULL(25, 24));
	cyc |= (2ULL << 22) | (2ULL << 20) | (3ULL << 24) | BIT_ULL(0);
	asm volatile("msr s3_5_c15_c5_0, %0; isb" :: "r"(cyc));

	for (;;)
		asm volatile("dsb sy; wfi");
}

static int smp_spin_table_cpu_kill(unsigned int cpu)
{
	u64 mpidr = cpu_logical_map(cpu);
	void __iomem *impl = ioremap(apple_impl_reg(mpidr) + 0x100, 8);
	u64 st = 0xff;
	int i;

	if (!impl)
		return 0;
	for (i = 0; i < 50; i++) {
		st = readq(impl);
		if (!(st & 0xff))
			break;
		mdelay(1);
	}
	iounmap(impl);
	pr_info("apple_hotplug: cpu%u stop status %llx after %d ms\n", cpu, st, i);
	/* The core is parked in power-down WFI either way: restart it through PMGR. */
	apple_cpu_stopped[cpu] = true;
	return !(st & 0xff);
}

static int apple_cpu_restart(unsigned int cpu)
{
	u64 mpidr = cpu_logical_map(cpu);
	u32 cl = MPIDR_AFFINITY_LEVEL(mpidr, 1), core = MPIDR_AFFINITY_LEVEL(mpidr, 0);

	apple_cpu_stopped[cpu] = false;
	write_pen_release(mpidr);
	writel_relaxed(BIT(4 * cl + core), apple_cpu_start_base + 0x4);
	writel_relaxed(BIT(core), apple_cpu_start_base + 0x8 + 4 * cl);
	dsb(sy);
	sev();
	pr_info("apple_hotplug: cpu%u start requested (cluster %u core %u)\n", cpu, cl, core);
	return 0;
}
#endif

static int smp_spin_table_cpu_boot(unsigned int cpu)
{
#ifdef CONFIG_HOTPLUG_CPU
	if (apple_cpu_stopped[cpu])
		return apple_cpu_restart(cpu);
#endif
	/*
	 * Update the pen release flag.
	 */
	write_pen_release(cpu_logical_map(cpu));

	/*
	 * Send an event, causing the secondaries to read pen_release.
	 */
	sev();

	return 0;
}

const struct cpu_operations smp_spin_table_ops = {
	.name		= "spin-table",
	.cpu_init	= smp_spin_table_cpu_init,
	.cpu_prepare	= smp_spin_table_cpu_prepare,
	.cpu_boot	= smp_spin_table_cpu_boot,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_can_disable = smp_spin_table_cpu_can_disable,
	.cpu_disable	= smp_spin_table_cpu_disable,
	.cpu_die	= smp_spin_table_cpu_die,
	.cpu_kill	= smp_spin_table_cpu_kill,
#endif
};
