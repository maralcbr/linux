// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SMC Reboot/Poweroff Handler
 * Copyright The Asahi Linux Contributors
 */

#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/mfd/macsmc.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/suspend.h>
#include <asm/daifflags.h>

struct macsmc_reboot_nvmem {
	struct nvmem_cell *shutdown_flag;
	struct nvmem_cell *boot_stage;
	struct nvmem_cell *boot_error_count;
	struct nvmem_cell *panic_count;
};

static const char * const nvmem_names[] = {
	"shutdown_flag",
	"boot_stage",
	"boot_error_count",
	"panic_count",
};

enum boot_stage {
	BOOT_STAGE_SHUTDOWN		= 0x00, /* Clean shutdown */
	BOOT_STAGE_IBOOT_DONE		= 0x2f, /* Last stage of bootloader */
	BOOT_STAGE_KERNEL_STARTED	= 0x30, /* Normal OS booting */
};

struct macsmc_reboot {
	struct device *dev;
	struct apple_smc *smc;
	struct notifier_block reboot_notify;

	union {
		struct macsmc_reboot_nvmem nvm;
		struct nvmem_cell *nvm_cells[ARRAY_SIZE(nvmem_names)];
	};
};

/* Helpers to read/write a u8 given a struct nvmem_cell */
static int nvmem_cell_get_u8(struct nvmem_cell *cell)
{
	size_t len;
	void *bfr;
	u8 val;

	bfr = nvmem_cell_read(cell, &len);
	if (IS_ERR(bfr))
		return PTR_ERR(bfr);

	if (len < 1) {
		kfree(bfr);
		return -EINVAL;
	}

	val = *(u8 *)bfr;
	kfree(bfr);
	return val;
}

static int nvmem_cell_set_u8(struct nvmem_cell *cell, u8 val)
{
	return nvmem_cell_write(cell, &val, sizeof(val));
}

/*
 * SMC 'MBSE' key actions:
 *
 * 'offw' - shutdown warning
 * 'slpw' - sleep warning
 * 'rest' - restart warning
 * 'off1' - shutdown (needs PMU bit set to stay on)
 * 'susp' - suspend
 * 'phra' - restart ("PE Halt Restart Action"?)
 * 'panb' - panic beginning
 * 'pane' - panic end
 */

static int macsmc_prepare_atomic(struct sys_off_data *data)
{
	struct macsmc_reboot *reboot = data->cb_data;

	dev_info(reboot->dev, "Preparing SMC for atomic mode\n");

	apple_smc_enter_atomic(reboot->smc);
	return NOTIFY_OK;
}

static int macsmc_power_off(struct sys_off_data *data)
{
	struct macsmc_reboot *reboot = data->cb_data;

	dev_info(reboot->dev, "Issuing power off (off1)\n");

	if (apple_smc_write_u32_atomic(reboot->smc, SMC_KEY(MBSE), SMC_KEY(off1)) < 0) {
		dev_err(reboot->dev, "Failed to issue MBSE = off1 (power_off)\n");
	} else {
		mdelay(100);
		WARN_ONCE(1, "Unable to power off system\n");
	}

	return NOTIFY_OK;
}

static int macsmc_restart(struct sys_off_data *data)
{
	struct macsmc_reboot *reboot = data->cb_data;

	dev_info(reboot->dev, "Issuing restart (phra)\n");

	if (apple_smc_write_u32_atomic(reboot->smc, SMC_KEY(MBSE), SMC_KEY(phra)) < 0) {
		dev_err(reboot->dev, "Failed to issue MBSE = phra (restart)\n");
	} else {
		mdelay(100);
		WARN_ONCE(1, "Unable to restart system\n");
	}

	return NOTIFY_OK;
}

static int macsmc_reboot_notify(struct notifier_block *this, unsigned long action, void *data)
{
	struct macsmc_reboot *reboot = container_of(this, struct macsmc_reboot, reboot_notify);
	u8 shutdown_flag;
	u32 val;

	switch (action) {
	case SYS_RESTART:
		val = SMC_KEY(rest);
		shutdown_flag = 0;
		break;
	case SYS_POWER_OFF:
		val = SMC_KEY(offw);
		shutdown_flag = 1;
		break;
	default:
		return NOTIFY_DONE;
	}

	dev_info(reboot->dev, "Preparing for reboot (%p4ch)\n", &val);

	/* On the Mac Mini, this will turn off the LED for power off */
	if (apple_smc_write_u32(reboot->smc, SMC_KEY(MBSE), val) < 0)
		dev_err(reboot->dev, "Failed to issue MBSE = %p4ch (reboot_prepare)\n", &val);

	/* Set the boot_stage to 0, which means we're doing a clean shutdown/reboot. */
	if (reboot->nvm.boot_stage &&
	    nvmem_cell_set_u8(reboot->nvm.boot_stage, BOOT_STAGE_SHUTDOWN) < 0)
		dev_err(reboot->dev, "Failed to write boot_stage\n");

	/*
	 * Set the PMU flag to actually reboot into the off state.
	 * Without this, the device will just reboot. We make it optional in case it is no longer
	 * necessary on newer hardware.
	 */
	if (reboot->nvm.shutdown_flag &&
	    nvmem_cell_set_u8(reboot->nvm.shutdown_flag, shutdown_flag) < 0)
		dev_err(reboot->dev, "Failed to write shutdown_flag\n");

	return NOTIFY_OK;
}

static void macsmc_power_init_error_counts(struct macsmc_reboot *reboot)
{
	int boot_error_count, panic_count;

	if (!reboot->nvm.boot_error_count || !reboot->nvm.panic_count)
		return;

	boot_error_count = nvmem_cell_get_u8(reboot->nvm.boot_error_count);
	if (boot_error_count < 0) {
		dev_err(reboot->dev, "Failed to read boot_error_count (%d)\n", boot_error_count);
		return;
	}

	panic_count = nvmem_cell_get_u8(reboot->nvm.panic_count);
	if (panic_count < 0) {
		dev_err(reboot->dev, "Failed to read panic_count (%d)\n", panic_count);
		return;
	}

	if (!boot_error_count && !panic_count)
		return;

	dev_warn(reboot->dev, "PMU logged %d boot error(s) and %d panic(s)\n",
		 boot_error_count, panic_count);

	if (nvmem_cell_set_u8(reboot->nvm.panic_count, 0) < 0)
		dev_err(reboot->dev, "Failed to reset panic_count\n");
	if (nvmem_cell_set_u8(reboot->nvm.boot_error_count, 0) < 0)
		dev_err(reboot->dev, "Failed to reset boot_error_count\n");
}


/*
 * LAB ONLY: one-way replay of the sleep-entry sequence macOS 15.6.1 runs on
 * j314s (traced under the m1n1 hypervisor), to measure the power floor Linux
 * reaches once every CPU is powered down. There is no resume path: the Mac
 * stays asleep until the lab's out-of-band reset. bit0 = SMC keys,
 * bit1 = CPU power-down, bit2 = PMGR/PMP register writes, bit3 = the
 * post-slpa PMGR/watchdog/error-handler writes, bit4 = run from s2idle
 * prepare instead of prepare_late, bit5 = disable all three SoC watchdog channels, bit6 = tell the AOP the AP is idle before slpa, bit7 = dump every PMGR power-state register before slpa.
 */
static unsigned int lab_s2r_floor;
module_param(lab_s2r_floor, uint, 0644);
MODULE_PARM_DESC(lab_s2r_floor, "LAB ONLY: replay macOS sleep entry at s2idle prepare_late and never return (bit0 SMC keys, bit1 CPU power-down, bit2 PMGR/PMP writes)");

static struct macsmc_reboot *lab_reboot;
int lab_rtkit_aop_ap_idle(void);

#define LAB_PMGR_SLEEP_REQ	0x28e3d8328ULL	/* PMGR reg 41 + 0x18328: macOS writes 4, later 1 */
#define LAB_PMP_DVFS4		0x28e3d0860ULL	/* SOC-DEV-DVFS + 4: macOS writes 1 << 61 */


/* macOS 15.6.1 j314s MMIO writes between MBSE=slpa and the CPU power-down, in order (hv trace). */
static const struct { u64 addr; u8 width; u64 val; } lab_post_slpa[] = {
	{ 0x2922b000cULL, 4, 0x0ULL },	/* wdt[0]+0x0c */
	{ 0x282000a00ULL, 4, 0x0ULL },	/* error-handler[7]+0xa00 */
	{ 0x304000a00ULL, 4, 0x0ULL },	/* error-handler[8]+0xa00 */
	{ 0x383000a00ULL, 4, 0x0ULL },	/* error-handler[9]+0xa00 */
	{ 0x402000a00ULL, 4, 0x0ULL },	/* error-handler[10]+0xa00 */
	{ 0x28e0ad000ULL, 4, 0x0ULL },	/* error-handler[2]+0x2d000 */
	{ 0x28e0f8000ULL, 4, 0x0ULL },	/* error-handler[2]+0x78000 */
	{ 0x28e0f9000ULL, 4, 0x0ULL },	/* error-handler[2]+0x79000 */
	{ 0x28e0fa000ULL, 4, 0x0ULL },	/* error-handler[2]+0x7a000 */
	{ 0x28e0fb000ULL, 4, 0x0ULL },	/* error-handler[2]+0x7b000 */
	{ 0x28e0fc000ULL, 4, 0x0ULL },	/* error-handler[2]+0x7c000 */
	{ 0x28e0fd000ULL, 4, 0x0ULL },	/* error-handler[2]+0x7d000 */
	{ 0x28e070000ULL, 4, 0x0ULL },	/* error-handler[1]+0x70000 */
	{ 0x28e5d8000ULL, 4, 0x0ULL },	/* pmgr[2]+0x58000 */
	{ 0x210e20660ULL, 8, 0x0ULL },	/* pmgr[8]+0x660 */
	{ 0x210e48080ULL, 8, 0x0ULL },	/* pmgr[10]+0x8080 */
	{ 0x211e20660ULL, 8, 0x0ULL },	/* pmgr[15]+0x660 */
	{ 0x211e48080ULL, 8, 0x0ULL },	/* pmgr[17]+0x8080 */
	{ 0x212e20660ULL, 8, 0x0ULL },	/* pmgr[24]+0x660 */
	{ 0x212e48080ULL, 8, 0x0ULL },	/* pmgr[26]+0x8080 */
	{ 0x58c008014ULL, 4, 0x1ULL },	/* pmgr[56]+0x8014 */
	{ 0x58c0082bcULL, 4, 0x1ULL },	/* pmgr[56]+0x82bc */
	{ 0x58c0082bcULL, 4, 0x2ULL },	/* pmgr[56]+0x82bc */
	{ 0x58c0082bcULL, 4, 0x4ULL },	/* pmgr[56]+0x82bc */
	{ 0x210e20020ULL, 8, 0x100101ULL },	/* pmgr[8]+0x20 */
	{ 0x211e20020ULL, 8, 0x100101ULL },	/* pmgr[15]+0x20 */
	{ 0x210e40250ULL, 8, 0xff092df01000ULL },	/* pmgr[10]+0x250 */
	{ 0x211e40250ULL, 8, 0x10ff092df01000ULL },	/* pmgr[17]+0x250 */
	{ 0x212e40250ULL, 8, 0x10ff092df01000ULL },	/* pmgr[26]+0x250 */
	{ 0x210e40240ULL, 8, 0x10000000ULL },	/* pmgr[10]+0x240 */
	{ 0x211e40240ULL, 8, 0x10000000ULL },	/* pmgr[17]+0x240 */
	{ 0x212e40240ULL, 8, 0x10000000ULL },	/* pmgr[26]+0x240 */
	{ 0x210e48400ULL, 8, 0x5b2002df02000ULL },	/* pmgr[10]+0x8400 */
	{ 0x211e48400ULL, 8, 0x5b2005df05200ULL },	/* pmgr[17]+0x8400 */
	{ 0x212e48400ULL, 8, 0x5b2005df05200ULL },	/* pmgr[26]+0x8400 */
	{ 0x28e5e0000ULL, 4, 0x0ULL },	/* pmgr[2]+0x60000 */
	{ 0x28e5e4000ULL, 4, 0x0ULL },	/* pmgr[2]+0x64000 */
	{ 0x28e0c8200ULL, 4, 0x0ULL },	/* error-handler[2]+0x48200 */
	{ 0x28e0c8400ULL, 4, 0x0ULL },	/* error-handler[2]+0x48400 */
	{ 0x28e0c8600ULL, 4, 0x0ULL },	/* error-handler[2]+0x48600 */
	{ 0x28e0c8000ULL, 4, 0x0ULL },	/* error-handler[2]+0x48000 */
	{ 0x210e20020ULL, 8, 0x40000500101ULL },	/* pmgr[8]+0x20 */
	{ 0x211e20020ULL, 8, 0x40000500101ULL },	/* pmgr[15]+0x20 */
	{ 0x212e20020ULL, 8, 0x40000500101ULL },	/* pmgr[24]+0x20 */
	{ 0x210e20020ULL, 8, 0x40000500101ULL },	/* pmgr[8]+0x20 */
	{ 0x211e20020ULL, 8, 0x40000500101ULL },	/* pmgr[15]+0x20 */
	{ 0x212e20020ULL, 8, 0x40000500101ULL },	/* pmgr[24]+0x20 */
	{ 0x28e20c800ULL, 4, 0x557ULL },	/* pmgr[38]+0x800 */
	{ 0x28e20c800ULL, 4, 0x775ULL },	/* pmgr[38]+0x800 */
	{ 0x28e0801e8ULL, 4, 0x0ULL },	/* error-handler[2]+0x1e8 */
	{ 0x28e098000ULL, 4, 0x0ULL },	/* error-handler[2]+0x18000 */
	{ 0x28e098004ULL, 4, 0x20ULL },	/* error-handler[2]+0x18004 */
	{ 0x28e098008ULL, 4, 0x20ULL },	/* error-handler[2]+0x18008 */
	{ 0x28e09800cULL, 4, 0x0ULL },	/* error-handler[2]+0x1800c */
	{ 0x28e5e4000ULL, 4, 0x0ULL },	/* pmgr[2]+0x64000 */
	{ 0x28e098014ULL, 4, 0x0ULL },	/* error-handler[2]+0x18014 */
	{ 0x28e098018ULL, 4, 0x20ULL },	/* error-handler[2]+0x18018 */
	{ 0x28e0c8400ULL, 4, 0x0ULL },	/* error-handler[2]+0x48400 */
	{ 0x28e098020ULL, 4, 0x0ULL },	/* error-handler[2]+0x18020 */
	{ 0x28e580188ULL, 4, 0x400024fULL },	/* pmgr[2]+0x188 */
	{ 0x28e038164ULL, 4, 0x80100000ULL },	/* error-handler[1]+0x38164 */
	{ 0x28e03816cULL, 4, 0x80100000ULL },	/* error-handler[1]+0x3816c */
	{ 0x28e038168ULL, 4, 0x80100000ULL },	/* error-handler[1]+0x38168 */
	{ 0x28e03815cULL, 4, 0x80100000ULL },	/* error-handler[1]+0x3815c */
	{ 0x28e038170ULL, 4, 0x80100000ULL },	/* error-handler[1]+0x38170 */
	{ 0x28e224000ULL, 4, 0x219ULL },	/* pmgr[46]+0x00 */
	{ 0x28e224000ULL, 4, 0x21bULL },	/* pmgr[46]+0x00 */
	{ 0x28e224000ULL, 4, 0x8000021aULL },	/* pmgr[46]+0x00 */
	{ 0x28e224000ULL, 4, 0x21aULL },	/* pmgr[46]+0x00 */
	{ 0x28e224000ULL, 4, 0x21aULL },	/* pmgr[46]+0x00 */
	{ 0x28e224000ULL, 4, 0x21aULL },	/* pmgr[46]+0x00 */
	{ 0x2925c4000ULL, 4, 0x1ULL },	/* pmgr[36]+0x4000 */
};

static char lab_post_skip[128];
module_param_string(lab_post_skip, lab_post_skip, sizeof(lab_post_skip), 0644);
MODULE_PARM_DESC(lab_post_skip, "LAB ONLY: comma-separated indexes of post-slpa writes to skip");

static bool lab_post_skipped(int idx)
{
	char buf[sizeof(lab_post_skip)], *cur = buf, *tok;
	int n;

	strscpy(buf, lab_post_skip, sizeof(buf));
	while ((tok = strsep(&cur, ",")))
		if (!kstrtoint(tok, 10, &n) && n == idx)
			return true;
	return false;
}

static bool lab_eh_early = true;
module_param(lab_eh_early, bool, 0644);
MODULE_PARM_DESC(lab_eh_early, "LAB ONLY: write the watchdog/error-handler entries of the post-slpa table before the SMC handshake");

static bool lab_post_is_eh(int i)
{
	u64 a = lab_post_slpa[i].addr;

	return a == 0x2922b000cULL || (a & 0xfff) == 0xa00 ||
	       (a >= 0x28e030000ULL && a < 0x28e100000ULL);
}

static bool lab_post_fast = true;
module_param(lab_post_fast, bool, 0644);
MODULE_PARM_DESC(lab_post_fast, "LAB ONLY: replay post-slpa writes back to back (pre-mapped, no prints or delays)");

static void __iomem *lab_post_map[ARRAY_SIZE(lab_post_slpa)];

static void lab_replay_post_slpa_fast(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lab_post_slpa); i++) {
		if (!lab_post_map[i] || lab_post_skipped(i) || (lab_eh_early && lab_post_is_eh(i)))
			continue;
		if (lab_post_slpa[i].width == 8)
			writeq_relaxed(lab_post_slpa[i].val, lab_post_map[i]);
		else
			writel_relaxed(lab_post_slpa[i].val, lab_post_map[i]);
	}
	dsb(sy);
}

static int lab_probe_set(const char *val, const struct kernel_param *kp)
{
	void __iomem *p;
	int idx;
	u64 v;

	if (kstrtoint(val, 10, &idx) || idx < 0 || idx >= ARRAY_SIZE(lab_post_slpa))
		return -EINVAL;
	pr_emerg("lab_probe: [%d] %llx w%u reading\n", idx, lab_post_slpa[idx].addr, lab_post_slpa[idx].width);
	p = ioremap_np(lab_post_slpa[idx].addr & PAGE_MASK, PAGE_SIZE);
	if (!p)
		return -ENOMEM;
	p += lab_post_slpa[idx].addr & ~PAGE_MASK;
	v = lab_post_slpa[idx].width == 8 ? readq(p) : readl(p);
	dsb(sy);
	mdelay(5);
	pr_emerg("lab_probe: [%d] %llx = %llx (macOS writes %llx)\n", idx, lab_post_slpa[idx].addr, v,
		 lab_post_slpa[idx].val);
	iounmap(p - (lab_post_slpa[idx].addr & ~PAGE_MASK));
	return 0;
}

static int lab_probe_write_set(const char *val, const struct kernel_param *kp)
{
	void __iomem *p;
	int idx;

	if (kstrtoint(val, 10, &idx) || idx < 0 || idx >= ARRAY_SIZE(lab_post_slpa))
		return -EINVAL;
	pr_emerg("lab_probe: [%d] %llx w%u writing %llx\n", idx, lab_post_slpa[idx].addr,
		 lab_post_slpa[idx].width, lab_post_slpa[idx].val);
	p = ioremap_np(lab_post_slpa[idx].addr & PAGE_MASK, PAGE_SIZE);
	if (!p)
		return -ENOMEM;
	if (lab_post_slpa[idx].width == 8)
		writeq(lab_post_slpa[idx].val, p + (lab_post_slpa[idx].addr & ~PAGE_MASK));
	else
		writel(lab_post_slpa[idx].val, p + (lab_post_slpa[idx].addr & ~PAGE_MASK));
	dsb(sy);
	mdelay(20);
	pr_emerg("lab_probe: [%d] write ok\n", idx);
	iounmap(p);
	return 0;
}

static const struct kernel_param_ops lab_probe_write_ops = { .set = lab_probe_write_set };
module_param_cb(lab_probe_write, &lab_probe_write_ops, NULL, 0200);
MODULE_PARM_DESC(lab_probe_write, "LAB ONLY: write the macOS value of post-slpa table entry N while awake");

static const struct kernel_param_ops lab_probe_ops = { .set = lab_probe_set };
module_param_cb(lab_probe, &lab_probe_ops, NULL, 0200);
MODULE_PARM_DESC(lab_probe, "LAB ONLY: read post-slpa table entry N and log it");

static void lab_replay_post_slpa(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lab_post_slpa); i++) {
		void __iomem *p;

		if (lab_post_skipped(i))
			continue;
		pr_emerg("lab_s2r_floor: post[%d] %llx w%u = %llx\n", i, lab_post_slpa[i].addr,
			 lab_post_slpa[i].width, lab_post_slpa[i].val);
		mdelay(20);
		p = ioremap_np(lab_post_slpa[i].addr & PAGE_MASK, PAGE_SIZE);
		if (!p)
			continue;
		p += lab_post_slpa[i].addr & ~PAGE_MASK;
		if (lab_post_slpa[i].width == 8)
			writeq(lab_post_slpa[i].val, p);
		else
			writel(lab_post_slpa[i].val, p);
		dsb(sy);
		mdelay(20);
	}
}

static void lab_floor_cpu(void *unused)
{
	u64 acc, cyc;

	local_daif_mask();
	asm volatile("mrs %0, s3_5_c15_c6_0" : "=r"(acc));
	acc &= ~(GENMASK_ULL(18, 13) | GENMASK_ULL(26, 25));
	acc |= BIT_ULL(13) | (3ULL << 17) | (3ULL << 25) | BIT_ULL(32) | BIT_ULL(34);
	asm volatile("msr s3_5_c15_c6_0, %0" :: "r"(acc));
	asm volatile("mrs %0, s3_5_c15_c5_0" : "=r"(cyc));
	cyc |= 0x3000001ULL;
	asm volatile("msr s3_5_c15_c5_0, %0; isb" :: "r"(cyc));
	for (;;)
		asm volatile("dsb sy; wfi");
}

static int lab_s2r_run(void);

static int lab_s2r_prepare(void)
{
	return (lab_s2r_floor & 16) ? lab_s2r_run() : 0;
}

static int lab_s2r_prepare_late(void)
{
	return (lab_s2r_floor & 16) ? 0 : lab_s2r_run();
}


/* PMGR power-state registers of every ADT pmgr device (j314s, macOS 15.6.1 ADT). */
static const u64 lab_ps_regs[] = {
	0x28e080000ULL, 0x28e080008ULL, 0x28e080020ULL, 0x28e080028ULL, 0x28e080030ULL, 0x28e080038ULL,
	0x28e080040ULL, 0x28e080048ULL, 0x28e080050ULL, 0x28e080058ULL, 0x28e080060ULL, 0x28e080068ULL,
	0x28e080070ULL, 0x28e080100ULL, 0x28e080108ULL, 0x28e080110ULL, 0x28e080118ULL, 0x28e080120ULL,
	0x28e080128ULL, 0x28e080130ULL, 0x28e080138ULL, 0x28e080140ULL, 0x28e080148ULL, 0x28e080150ULL,
	0x28e080158ULL, 0x28e080160ULL, 0x28e080168ULL, 0x28e080170ULL, 0x28e080178ULL, 0x28e080180ULL,
	0x28e080188ULL, 0x28e080190ULL, 0x28e080198ULL, 0x28e0801a0ULL, 0x28e0801a8ULL, 0x28e0801b0ULL,
	0x28e0801b8ULL, 0x28e0801c0ULL, 0x28e0801c8ULL, 0x28e0801d0ULL, 0x28e0801d8ULL, 0x28e0801e0ULL,
	0x28e0801e8ULL, 0x28e0801f0ULL, 0x28e0801f8ULL, 0x28e080200ULL, 0x28e080208ULL, 0x28e080210ULL,
	0x28e080218ULL, 0x28e080220ULL, 0x28e080228ULL, 0x28e080230ULL, 0x28e080238ULL, 0x28e080240ULL,
	0x28e080248ULL, 0x28e080250ULL, 0x28e080258ULL, 0x28e080260ULL, 0x28e080268ULL, 0x28e080270ULL,
	0x28e080278ULL, 0x28e080280ULL, 0x28e080288ULL, 0x28e080290ULL, 0x28e080298ULL, 0x28e0802a0ULL,
	0x28e0802a8ULL, 0x28e0802b0ULL, 0x28e0802b8ULL, 0x28e0802c0ULL, 0x28e0802c8ULL, 0x28e0802d0ULL,
	0x28e0802d8ULL, 0x28e0802e0ULL, 0x28e0802e8ULL, 0x28e0802f0ULL, 0x28e0802f8ULL, 0x28e080300ULL,
	0x28e080308ULL, 0x28e080310ULL, 0x28e080318ULL, 0x28e080320ULL, 0x28e080328ULL, 0x28e080330ULL,
	0x28e080338ULL, 0x28e080340ULL, 0x28e080348ULL, 0x28e080350ULL, 0x28e080358ULL, 0x28e080360ULL,
	0x28e080368ULL, 0x28e080370ULL, 0x28e080378ULL, 0x28e080380ULL, 0x28e080388ULL, 0x28e080390ULL,
	0x28e080398ULL, 0x28e0803a0ULL, 0x28e0803a8ULL, 0x28e0803b0ULL, 0x28e0803b8ULL, 0x28e0803c0ULL,
	0x28e580100ULL, 0x28e580108ULL, 0x28e580110ULL, 0x28e580118ULL, 0x28e580120ULL, 0x28e580128ULL,
	0x28e580130ULL, 0x28e580138ULL, 0x28e580140ULL, 0x28e580148ULL, 0x28e580150ULL, 0x28e580158ULL,
	0x28e580160ULL, 0x28e580168ULL, 0x28e580170ULL, 0x28e580178ULL, 0x28e580180ULL, 0x28e580188ULL,
	0x28e580190ULL, 0x28e580198ULL, 0x28e5801a0ULL, 0x28e5801a8ULL, 0x28e5801b0ULL, 0x28e5801b8ULL,
	0x28e5801c0ULL, 0x28e5801c8ULL, 0x28e5801d0ULL, 0x28e5801d8ULL, 0x28e5801e0ULL, 0x28e5801e8ULL,
	0x28e5801f0ULL, 0x28e5801f8ULL, 0x28e580200ULL, 0x28e580208ULL, 0x28e580210ULL, 0x28e580218ULL,
	0x28e580220ULL, 0x28e580228ULL, 0x28e580230ULL, 0x28e580238ULL, 0x28e580240ULL, 0x28e580248ULL,
	0x28e580250ULL, 0x28e580258ULL, 0x28e580260ULL, 0x28e580268ULL, 0x28e580270ULL, 0x28e580278ULL,
	0x28e580280ULL, 0x28e580288ULL, 0x28e580290ULL, 0x28e580298ULL, 0x28e5802a0ULL, 0x28e5802a8ULL,
	0x28e5802b0ULL, 0x28e5802b8ULL, 0x28e5802c0ULL, 0x28e5802c8ULL, 0x28e5802d0ULL, 0x28e5802d8ULL,
	0x28e5802e0ULL, 0x28e5802e8ULL, 0x28e5802f0ULL, 0x28e5802f8ULL, 0x28e580300ULL, 0x28e580308ULL,
	0x28e580310ULL, 0x28e580318ULL, 0x28e580320ULL, 0x28e580328ULL, 0x28e580350ULL, 0x28e580378ULL,
	0x28e580380ULL, 0x28e580388ULL, 0x28e580390ULL, 0x28e580398ULL, 0x28e5803a0ULL, 0x28e5803a8ULL,
	0x28e5803b0ULL, 0x28e5803b8ULL, 0x28e5803c0ULL, 0x28e580c00ULL, 0x28e580c08ULL, 0x28e588000ULL,
	0x28e588008ULL, 0x28e588010ULL, 0x28e588018ULL, 0x28e588020ULL, 0x292280000ULL, 0x292280058ULL,
	0x292280060ULL, 0x292280068ULL, 0x292280070ULL, 0x292280078ULL, 0x292280080ULL, 0x292280088ULL,
	0x292280090ULL, 0x292280098ULL, 0x2922800a0ULL, 0x2922800a8ULL, 0x2922800b0ULL, 0x2922800b8ULL,
	0x2922800c0ULL, 0x2922800c8ULL, 0x2922800d0ULL, 0x2922800d8ULL, 0x2922800e0ULL, 0x2922800e8ULL
};

static void lab_dump_ps(const char *when)
{
	char line[160];
	int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(lab_ps_regs); i++) {
		void __iomem *p = ioremap_np(lab_ps_regs[i] & PAGE_MASK, PAGE_SIZE);
		u32 v = p ? readl(p + (lab_ps_regs[i] & ~PAGE_MASK)) : 0xffffffff;

		if (p)
			iounmap(p);
		n += scnprintf(line + n, sizeof(line) - n, " %llx=%08x", lab_ps_regs[i], v);
		if (n > 120 || i == ARRAY_SIZE(lab_ps_regs) - 1) {
			pr_emerg("lab_ps %s:%s\n", when, line);
			n = 0;
		}
	}
}

static void lab_dump_wdt(const char *when)
{
	void __iomem *w = ioremap_np(0x2922b0000ULL, 0x40);
	int ch;

	if (!w)
		return;
	for (ch = 0; ch < 3; ch++)
		pr_emerg("lab_wdt %s: ch%d ctl %08x count %08x bite %08x\n", when, ch,
			 readl(w + 0x0c + 0x10 * ch), readl(w + 0x10 * ch), readl(w + 0x04 + 0x10 * ch));
	iounmap(w);
}

static int lab_s2r_run(void)
{
	struct apple_smc *smc = lab_reboot->smc;
	void __iomem *req, *dvfs;
	int ret;

	if (!lab_s2r_floor)
		return 0;

	lab_dump_wdt("entry");
	if (lab_s2r_floor & 32) {
		void __iomem *w = ioremap_np(0x2922b0000ULL, 0x40);
		int ch;

		for (ch = 0; w && ch < 3; ch++)
			writel(0, w + 0x0c + 0x10 * ch);
		lab_dump_wdt("all-off");
	}

	req = ioremap_np(LAB_PMGR_SLEEP_REQ & PAGE_MASK, PAGE_SIZE);
	dvfs = ioremap_np(LAB_PMP_DVFS4 & PAGE_MASK, PAGE_SIZE);
	if (!req || !dvfs)
		return 0;

	if ((lab_s2r_floor & 8) && lab_post_fast) {
		int i;

		for (i = 0; i < ARRAY_SIZE(lab_post_slpa); i++) {
			lab_post_map[i] = ioremap_np(lab_post_slpa[i].addr & PAGE_MASK, PAGE_SIZE);
			if (lab_post_map[i])
				lab_post_map[i] += lab_post_slpa[i].addr & ~PAGE_MASK;
		}
		pr_emerg("lab_s2r_floor: fast mode: post-slpa writes and CPU power-down follow slpa with no output\n");
		if (lab_eh_early) {
			for (i = 0; i < ARRAY_SIZE(lab_post_slpa); i++) {
				if (!lab_post_map[i] || !lab_post_is_eh(i) || lab_post_skipped(i))
					continue;
				writel(lab_post_slpa[i].val, lab_post_map[i]);
			}
			dsb(sy);
			pr_emerg("lab_s2r_floor: error handlers masked before the handshake\n");
		}
	}
	if (lab_s2r_floor & 4)
		writel(4, req + (LAB_PMGR_SLEEP_REQ & ~PAGE_MASK));
	if (lab_s2r_floor & 1) {
		ret = apple_smc_write_u32(smc, SMC_KEY(MBSE), SMC_KEY(slpw));
		pr_emerg("lab_s2r_floor: MBSE=slpw %d\n", ret);
		ret = apple_smc_write_u32(smc, SMC_KEY(pmVC), 0x100);
		pr_emerg("lab_s2r_floor: pmVC=0x100 %d\n", ret);
		ret = apple_smc_write_u8(smc, SMC_KEY(WKTP), 2);
		pr_emerg("lab_s2r_floor: WKTP=2 %d\n", ret);
	}
	if (lab_s2r_floor & 4) {
		writeq(1ULL << 61, dvfs + (LAB_PMP_DVFS4 & ~PAGE_MASK));
		writel(1, req + (LAB_PMGR_SLEEP_REQ & ~PAGE_MASK));
	}
	if (lab_s2r_floor & 128)
		lab_dump_ps("before-slpa");
	if (lab_s2r_floor & 64) {
		ret = lab_rtkit_aop_ap_idle();
		pr_emerg("lab_s2r_floor: AOP SetAPPower(IDLE) %d\n", ret);
	}
	if (lab_s2r_floor & 1) {
		ret = apple_smc_write_u32(smc, SMC_KEY(MBSE), SMC_KEY(slpa));
		if (!((lab_s2r_floor & 8) && lab_post_fast))
			pr_emerg("lab_s2r_floor: MBSE=slpa %d\n", ret);
	}
	if ((lab_s2r_floor & 8) && lab_post_fast) {
		lab_replay_post_slpa_fast();
		if (lab_s2r_floor & 2) {
			smp_call_function(lab_floor_cpu, NULL, 0);
			lab_floor_cpu(NULL);
		}
	}
	if ((lab_s2r_floor & 8) && !lab_post_fast) {
		pr_emerg("lab_s2r_floor: replaying %zu post-slpa writes\n", ARRAY_SIZE(lab_post_slpa));
		mdelay(50);
		lab_replay_post_slpa();
		pr_emerg("lab_s2r_floor: post-slpa replay done\n");
	}
	if (lab_s2r_floor & 2) {
		pr_emerg("lab_s2r_floor: powering down all CPUs, no return\n");
		mdelay(100);
		smp_call_function(lab_floor_cpu, NULL, 0);
		mdelay(50);
		lab_floor_cpu(NULL);
	}
	return 0;
}

static const struct platform_s2idle_ops lab_s2r_ops = {
	.prepare = lab_s2r_prepare,
	.prepare_late = lab_s2r_prepare_late,
};

static int macsmc_reboot_probe(struct platform_device *pdev)
{
	struct apple_smc *smc = dev_get_drvdata(pdev->dev.parent);
	struct macsmc_reboot *reboot;
	int ret, i;

	/*
	 * MFD will probe this device even without a node in the device tree,
	 * thus bail out early if the SMC on the current machines does not
	 * support reboot and has no node in the device tree.
	 */
	if (!pdev->dev.of_node)
		return -ENODEV;

	reboot = devm_kzalloc(&pdev->dev, sizeof(*reboot), GFP_KERNEL);
	if (!reboot)
		return -ENOMEM;

	reboot->dev = &pdev->dev;
	reboot->smc = smc;

	platform_set_drvdata(pdev, reboot);

	for (i = 0; i < ARRAY_SIZE(nvmem_names); i++) {
		struct nvmem_cell *cell;

		cell = devm_nvmem_cell_get(&pdev->dev,
					   nvmem_names[i]);
		if (IS_ERR(cell)) {
			if (PTR_ERR(cell) == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			dev_warn(&pdev->dev, "Missing NVMEM cell %s (%ld)\n",
				 nvmem_names[i], PTR_ERR(cell));
			/* Non fatal, we'll deal with it */
			cell = NULL;
		}
		reboot->nvm_cells[i] = cell;
	}

	/* Set the boot_stage to indicate we're running the OS kernel */
	if (reboot->nvm.boot_stage &&
	    nvmem_cell_set_u8(reboot->nvm.boot_stage, BOOT_STAGE_KERNEL_STARTED) < 0)
		dev_err(reboot->dev, "Failed to write boot_stage\n");

	/* Display and clear the error counts */
	macsmc_power_init_error_counts(reboot);

	reboot->reboot_notify.notifier_call = macsmc_reboot_notify;

	ret = devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_POWER_OFF_PREPARE,
					    SYS_OFF_PRIO_HIGH, macsmc_prepare_atomic, reboot);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to register power-off prepare handler\n");
	ret = devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_POWER_OFF, SYS_OFF_PRIO_HIGH,
					    macsmc_power_off, reboot);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to register power-off handler\n");

	ret = devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_RESTART_PREPARE,
					    SYS_OFF_PRIO_HIGH, macsmc_prepare_atomic, reboot);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to register restart prepare handler\n");
	ret = devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_RESTART, SYS_OFF_PRIO_HIGH,
					    macsmc_restart, reboot);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to register restart handler\n");

	ret = devm_register_reboot_notifier(&pdev->dev, &reboot->reboot_notify);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to register reboot notifier\n");

	lab_reboot = reboot;
	s2idle_set_ops(&lab_s2r_ops);

	dev_info(&pdev->dev, "Handling reboot and poweroff requests via SMC\n");

	return 0;
}

static const struct of_device_id macsmc_reboot_of_table[] = {
	{ .compatible = "apple,smc-reboot", },
	{}
};
MODULE_DEVICE_TABLE(of, macsmc_reboot_of_table);

static struct platform_driver macsmc_reboot_driver = {
	.driver = {
		.name = "macsmc-reboot",
		.of_match_table = macsmc_reboot_of_table,
	},
	.probe = macsmc_reboot_probe,
};
module_platform_driver(macsmc_reboot_driver);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Apple SMC reboot/poweroff driver");
MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
