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
 * bit1 = CPU power-down, bit2 = PMGR/PMP register writes.
 */
static unsigned int lab_s2r_floor;
module_param(lab_s2r_floor, uint, 0644);
MODULE_PARM_DESC(lab_s2r_floor, "LAB ONLY: replay macOS sleep entry at s2idle prepare_late and never return (bit0 SMC keys, bit1 CPU power-down, bit2 PMGR/PMP writes)");

static struct macsmc_reboot *lab_reboot;

#define LAB_PMGR_SLEEP_REQ	0x28e3d8328ULL	/* PMGR reg 41 + 0x18328: macOS writes 4, later 1 */
#define LAB_PMP_DVFS4		0x28e3d0860ULL	/* SOC-DEV-DVFS + 4: macOS writes 1 << 61 */

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

static int lab_s2r_prepare_late(void)
{
	struct apple_smc *smc = lab_reboot->smc;
	void __iomem *req, *dvfs;
	int ret;

	if (!lab_s2r_floor)
		return 0;

	req = ioremap(LAB_PMGR_SLEEP_REQ & PAGE_MASK, PAGE_SIZE);
	dvfs = ioremap(LAB_PMP_DVFS4 & PAGE_MASK, PAGE_SIZE);
	if (!req || !dvfs)
		return 0;

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
	if (lab_s2r_floor & 1) {
		ret = apple_smc_write_u32(smc, SMC_KEY(MBSE), SMC_KEY(slpa));
		pr_emerg("lab_s2r_floor: MBSE=slpa %d\n", ret);
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
