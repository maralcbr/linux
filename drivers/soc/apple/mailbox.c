// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple mailbox driver
 *
 * Copyright The Asahi Linux Contributors
 *
 * This driver adds support for two mailbox variants (called ASC and M3 by
 * Apple) found in Apple SoCs such as the M1. It consists of two FIFOs used to
 * exchange 64+32 bit messages between the main CPU and a co-processor.
 * Various coprocessors implement different IPC protocols based on these simple
 * messages and shared memory buffers.
 *
 * Both the main CPU and the co-processor see the same set of registers but
 * the first FIFO (A2I) is always used to transfer messages from the application
 * processor (us) to the I/O processor and the second one (I2A) for the
 * other direction.
 */

#include <clocksource/arm_arch_timer.h>
#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <linux/soc/apple/mailbox.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/timekeeping.h>
#include <linux/types.h>

#ifdef CONFIG_ARM64
#include <asm/sysreg.h>
#endif

#define APPLE_ASC_MBOX_CONTROL_FULL BIT(16)
#define APPLE_ASC_MBOX_CONTROL_EMPTY BIT(17)

#define APPLE_ASC_MBOX_A2I_CONTROL 0x110
#define APPLE_ASC_MBOX_A2I_SEND0 0x800
#define APPLE_ASC_MBOX_A2I_SEND1 0x808
#define APPLE_ASC_MBOX_A2I_RECV0 0x810
#define APPLE_ASC_MBOX_A2I_RECV1 0x818

#define APPLE_ASC_MBOX_I2A_CONTROL 0x114
#define APPLE_ASC_MBOX_I2A_SEND0 0x820
#define APPLE_ASC_MBOX_I2A_SEND1 0x828
#define APPLE_ASC_MBOX_I2A_RECV0 0x830
#define APPLE_ASC_MBOX_I2A_RECV1 0x838

/* ASCWrap v6 embeds the unchanged ASC mailbox block at this offset. */
#define APPLE_ASCWRAP_V6_MBOX_OFFSET 0x8000
#define APPLE_ASCWRAP_V6_WRAPPER_SIZE 0x88000
#define APPLE_ASCWRAP_V6_IOP_VBAR_SIZE 0x8
#define APPLE_ASCWRAP_V6_CPU_CONTROL 0x44
#define APPLE_ASCWRAP_V6_CPU_RUN BIT(4)
#define APPLE_ASCWRAP_V6_CPU_STOP_SECOND_CLEAR BIT(5)

#define APPLE_T8140_GFX_WRAPPER_BASE 0x482600000ULL
#define APPLE_T8140_GFX_IOP_VBAR_BASE 0x482050000ULL
#define APPLE_T8140_GFX1_WRAPPER_BASE 0x482e00000ULL
#define APPLE_T8140_GFX1_IOP_VBAR_BASE 0x482850000ULL

static_assert(APPLE_ASCWRAP_V6_MBOX_OFFSET + APPLE_ASC_MBOX_A2I_CONTROL ==
	      0x8110);
static_assert(APPLE_ASCWRAP_V6_MBOX_OFFSET + APPLE_ASC_MBOX_I2A_CONTROL ==
	      0x8114);
static_assert(APPLE_ASCWRAP_V6_MBOX_OFFSET + APPLE_ASC_MBOX_A2I_SEND0 ==
	      0x8800);
static_assert(APPLE_ASCWRAP_V6_MBOX_OFFSET + APPLE_ASC_MBOX_I2A_RECV0 ==
	      0x8830);

/*
 * The T8140 AOP "setup port" (ADT aop-exclave-mailbox): the second mailbox
 * the J700 AOP firmware boots through.  Layout measured on hardware by the
 * native m1n1 microphone host (artifacts/j700-native-lpmic-20260906).
 */
#define APPLE_T8140_AOP_SETUP_A2I_CONTROL 0x000
#define APPLE_T8140_AOP_SETUP_I2A_CONTROL 0x004
#define APPLE_T8140_AOP_SETUP_A2I_SEND0   0x180
#define APPLE_T8140_AOP_SETUP_A2I_SEND1   0x188
#define APPLE_T8140_AOP_SETUP_I2A_RECV0   0x1b0
#define APPLE_T8140_AOP_SETUP_I2A_RECV1   0x1b8

#define APPLE_T8015_MBOX_A2I_CONTROL	0x108
#define APPLE_T8015_MBOX_I2A_CONTROL	0x10c

#define APPLE_M3_MBOX_CONTROL_FULL BIT(16)
#define APPLE_M3_MBOX_CONTROL_EMPTY BIT(17)

#define APPLE_M3_MBOX_A2I_CONTROL 0x50
#define APPLE_M3_MBOX_A2I_SEND0 0x60
#define APPLE_M3_MBOX_A2I_SEND1 0x68
#define APPLE_M3_MBOX_A2I_RECV0 0x70
#define APPLE_M3_MBOX_A2I_RECV1 0x78

#define APPLE_M3_MBOX_I2A_CONTROL 0x80
#define APPLE_M3_MBOX_I2A_SEND0 0x90
#define APPLE_M3_MBOX_I2A_SEND1 0x98
#define APPLE_M3_MBOX_I2A_RECV0 0xa0
#define APPLE_M3_MBOX_I2A_RECV1 0xa8

#define APPLE_M3_MBOX_IRQ_ENABLE 0x48
#define APPLE_M3_MBOX_IRQ_ACK 0x4c
#define APPLE_M3_MBOX_IRQ_A2I_EMPTY BIT(0)
#define APPLE_M3_MBOX_IRQ_A2I_NOT_EMPTY BIT(1)
#define APPLE_M3_MBOX_IRQ_I2A_EMPTY BIT(2)
#define APPLE_M3_MBOX_IRQ_I2A_NOT_EMPTY BIT(3)

#define APPLE_MBOX_MSG1_OUTCNT GENMASK(56, 52)
#define APPLE_MBOX_MSG1_INCNT GENMASK(51, 48)
#define APPLE_MBOX_MSG1_OUTPTR GENMASK(47, 44)
#define APPLE_MBOX_MSG1_INPTR GENMASK(43, 40)
#define APPLE_MBOX_MSG1_MSG GENMASK(31, 0)

#define APPLE_MBOX_TX_TIMEOUT 500

/* LAB ONLY: piggyback on existing j314s AOP traffic, never add a timer IRQ. */
static unsigned int lab_s2idle_wake_ms;
module_param(lab_s2idle_wake_ms, uint, 0644);
MODULE_PARM_DESC(lab_s2idle_wake_ms,
		 "LAB ONLY: wake j314s s2idle on AOP RX after this many ms from mailbox noirq suspend (0=off)");

static struct apple_mbox *lab_aop_mbox;
static struct device *lab_diag_dev;
static atomic64_t lab_wake_deadline = ATOMIC64_INIT(0);

/* LAB ONLY: retain evidence in RAM; never print from the receive IRQ. */
struct apple_mbox_lab_sample {
	u64 rx, checks, first_counter, last_counter, first_boot, last_boot;
	u64 before, expired, no_deadline, wake_requests, deadline;
	u64 first_check_boot, last_check_boot;
};

static DEFINE_RAW_SPINLOCK(lab_diag_lock);
static struct apple_mbox_lab_sample lab_live, lab_resume;
static bool lab_collect, lab_noirq, lab_match, lab_resume_seen;
static bool lab_direct_complete, lab_syscore, lab_runtime_suspended;
static bool lab_prepare_active, lab_prepare_removing, lab_selected;
static unsigned int lab_ms, lab_target, lab_counter_rate;
static int lab_prepare_irq;
static u64 lab_prepare_counter, lab_prepare_boot, lab_noirq_counter;
static u64 lab_noirq_boot, lab_exit_before, lab_exit_resume;
static u64 lab_resume_counter, lab_resume_boot;
static resource_size_t lab_resource;
static char lab_prepare_compatible[64];

static bool apple_mbox_lab_syscore(struct device *dev)
{
#ifdef CONFIG_PM_SLEEP
	return dev && dev->power.syscore;
#else
	return false;
#endif
}

static u32 apple_mbox_lab_counter_rate(void)
{
#ifdef CONFIG_ARM64
	return read_sysreg(cntfrq_el0);
#else
	return 0;
#endif
}

static void apple_mbox_lab_rx_entry(struct apple_mbox *mbox)
{
	unsigned long flags;
	u64 counter, boot;

	if (mbox != READ_ONCE(lab_aop_mbox) ||
	    !READ_ONCE(lab_collect) || !atomic64_read(&lab_wake_deadline))
		return;
	counter = arch_timer_read_counter();
	boot = ktime_get_boottime_ns();
	raw_spin_lock_irqsave(&lab_diag_lock, flags);
	if (lab_collect && atomic64_read(&lab_wake_deadline)) {
		if (!lab_live.rx) {
			lab_live.first_counter = counter;
			lab_live.first_boot = boot;
		}
		if (!lab_live.rx)
			lab_s2idle_trace_step("aop.rx.first", 0, 0);
		lab_live.rx++;
		lab_live.last_counter = counter;
		lab_live.last_boot = boot;
	}
	raw_spin_unlock_irqrestore(&lab_diag_lock, flags);
}

static void apple_mbox_lab_wake(struct apple_mbox *mbox)
{
	unsigned long flags;
	u64 deadline, boot;
	bool wake = false;

	if (mbox != READ_ONCE(lab_aop_mbox) || !READ_ONCE(lab_collect))
		return;
	boot = ktime_get_boottime_ns();
	raw_spin_lock_irqsave(&lab_diag_lock, flags);
	deadline = atomic64_read(&lab_wake_deadline);
	if (lab_collect) {
		if (!lab_live.checks)
			lab_live.first_check_boot = boot;
		lab_live.last_check_boot = boot;
		lab_live.checks++;
		if (!deadline)
			lab_live.no_deadline++;
		else if (boot < deadline)
			lab_live.before++;
		else
			lab_live.expired++;
	}
	if (deadline && boot >= deadline &&
	    atomic64_cmpxchg(&lab_wake_deadline, deadline, 0) == deadline) {
		lab_live.wake_requests++;
		wake = true;
	}
	raw_spin_unlock_irqrestore(&lab_diag_lock, flags);
	if (wake) {
		lab_s2idle_trace_step("aop.deadline", 0, 0);
		pm_system_wakeup();
	}
}

static int apple_mbox_lab_suspend_noirq(struct device *dev)
{
	struct apple_mbox *mbox = dev_get_drvdata(dev);
	unsigned long flags;

	/* Record callback arrival even if probe did not select the target. */
	if (dev != READ_ONCE(lab_diag_dev))
		return 0;
	raw_spin_lock_irqsave(&lab_diag_lock, flags);
	lab_noirq = true;
	lab_match = mbox && mbox == READ_ONCE(lab_aop_mbox);
	lab_direct_complete = dev->power.direct_complete;
	lab_syscore = apple_mbox_lab_syscore(dev);
	lab_runtime_suspended = pm_runtime_status_suspended(dev);
	lab_target = pm_suspend_target_state;
	lab_noirq_counter = arch_timer_read_counter();
	lab_noirq_boot = ktime_get_boottime_ns();
	atomic64_set(&lab_wake_deadline, 0);
	if (lab_match && lab_ms && lab_target == PM_SUSPEND_TO_IDLE) {
		lab_live.deadline = lab_noirq_boot + (u64)lab_ms * NSEC_PER_MSEC;
		atomic64_set(&lab_wake_deadline, lab_live.deadline);
		lab_collect = true;
	}
	raw_spin_unlock_irqrestore(&lab_diag_lock, flags);
	return 0;
}

static int apple_mbox_lab_resume_noirq(struct device *dev)
{
	unsigned long flags;

	if (dev != READ_ONCE(lab_diag_dev))
		return 0;
	raw_spin_lock_irqsave(&lab_diag_lock, flags);
	lab_resume_counter = arch_timer_read_counter();
	lab_resume_boot = ktime_get_boottime_ns();
	lab_resume = lab_live;
	lab_resume.deadline = atomic64_read(&lab_wake_deadline);
	lab_exit_resume = lab_s2idle_exit_count();
	lab_resume_seen = true;
	lab_collect = false;
	atomic64_set(&lab_wake_deadline, 0);
	raw_spin_unlock_irqrestore(&lab_diag_lock, flags);
	return 0;
}

static int apple_mbox_lab_notify(struct notifier_block *nb,
				unsigned long event, void *unused)
{
	struct device *dev = READ_ONCE(lab_diag_dev);
	struct apple_mbox *mbox;
	struct apple_mbox_lab_sample sample;
	struct device_node *root;
	unsigned long flags;
	const char *compatible = "missing";

	if (event == PM_SUSPEND_PREPARE) {
		root = of_find_node_by_path("/");
		if (root)
			of_property_read_string(root, "compatible", &compatible);
		strscpy(lab_prepare_compatible, compatible,
			sizeof(lab_prepare_compatible));
		of_node_put(root);
		mbox = dev ? dev_get_drvdata(dev) : NULL;
		raw_spin_lock_irqsave(&lab_diag_lock, flags);
		memset(&lab_live, 0, sizeof(lab_live));
		memset(&lab_resume, 0, sizeof(lab_resume));
		lab_collect = lab_noirq = lab_match = lab_resume_seen = false;
		lab_ms = READ_ONCE(lab_s2idle_wake_ms);
		lab_selected = mbox && mbox == READ_ONCE(lab_aop_mbox);
		lab_prepare_irq = mbox ? mbox->irq_recv_not_empty : -1;
		lab_prepare_active = mbox && READ_ONCE(mbox->active);
		lab_prepare_removing = mbox && READ_ONCE(mbox->removing);
		lab_direct_complete = dev && dev->power.direct_complete;
		lab_syscore = apple_mbox_lab_syscore(dev);
		lab_runtime_suspended = dev && pm_runtime_status_suspended(dev);
		lab_target = pm_suspend_target_state;
		lab_noirq_counter = lab_noirq_boot = 0;
		lab_resume_counter = lab_resume_boot = 0;
		lab_prepare_counter = arch_timer_read_counter();
		lab_prepare_boot = ktime_get_boottime_ns();
		lab_counter_rate = apple_mbox_lab_counter_rate();
		lab_exit_before = lab_s2idle_exit_count();
		lab_exit_resume = lab_exit_before;
		raw_spin_unlock_irqrestore(&lab_diag_lock, flags);
	} else if (event == PM_POST_SUSPEND) {
		raw_spin_lock_irqsave(&lab_diag_lock, flags);
		sample = lab_resume_seen ? lab_resume : lab_live;
		lab_collect = false;
		raw_spin_unlock_irqrestore(&lab_diag_lock, flags);
		pr_info("AOP-WAKE-DIAG BEGIN\n");
		pr_info("AOP-WAKE-DIAG PREPARE ms=%u compatible=%s resource=%pa selected=%u irq=%d active=%u removing=%u counter=%llu boot=%llu rate=%u\n",
			 lab_ms, lab_prepare_compatible, &lab_resource, lab_selected,
			 lab_prepare_irq, lab_prepare_active,
			 lab_prepare_removing, lab_prepare_counter, lab_prepare_boot,
			 lab_counter_rate);
		pr_info("AOP-WAKE-DIAG NOIRQ reached=%u match=%u target=%u direct_complete=%u syscore=%u runtime_suspended=%u counter=%llu boot=%llu armed_deadline=%llu\n",
			 lab_noirq, lab_match, lab_target, lab_direct_complete,
			 lab_syscore, lab_runtime_suspended, lab_noirq_counter,
			 lab_noirq_boot, lab_live.deadline);
		pr_info("AOP-WAKE-DIAG RX entries=%llu post_checks=%llu first_counter=%llu last_counter=%llu first_boot=%llu last_boot=%llu before=%llu expired=%llu no_deadline=%llu wake_requests=%llu first_check_boot=%llu last_check_boot=%llu last_delta_ns=%lld\n",
			 sample.rx, sample.checks, sample.first_counter,
			 sample.last_counter, sample.first_boot, sample.last_boot,
			 sample.before, sample.expired, sample.no_deadline,
			 sample.wake_requests, sample.first_check_boot,
			 sample.last_check_boot,
			 (s64)(sample.last_check_boot - lab_live.deadline));
		pr_info("AOP-WAKE-DIAG RESUME_NOIRQ reached=%u pending_deadline=%llu counter=%llu boot=%llu exits_before=%llu exits_resume=%llu exits_post=%llu\n",
			 lab_resume_seen, sample.deadline, lab_resume_counter,
			 lab_resume_boot, lab_exit_before,
			 lab_exit_resume, lab_s2idle_exit_count());
		pr_info("AOP-WAKE-DIAG END\n");
	}
	return NOTIFY_OK;
}

static struct notifier_block lab_diag_notifier = {
	.notifier_call = apple_mbox_lab_notify,
};

static const struct dev_pm_ops apple_mbox_lab_pm_ops = {
	.suspend_noirq = apple_mbox_lab_suspend_noirq,
	.resume_noirq = apple_mbox_lab_resume_noirq,
};

static void apple_mbox_lab_release(void *data)
{
	if (data == READ_ONCE(lab_aop_mbox)) {
		atomic64_set(&lab_wake_deadline, 0);
		WRITE_ONCE(lab_aop_mbox, NULL);
	}
}

struct apple_mbox_hw {
	unsigned int reg_offset;
	bool is_ascwrap_v6;
	bool ap_initializes_mailboxes;

	unsigned int control_full;
	unsigned int control_empty;

	unsigned int a2i_control;
	unsigned int a2i_send0;
	unsigned int a2i_send1;

	unsigned int i2a_control;
	unsigned int i2a_recv0;
	unsigned int i2a_recv1;

	bool has_irq_controls;
	unsigned int irq_enable;
	unsigned int irq_ack;
	unsigned int irq_bit_recv_not_empty;
	unsigned int irq_bit_send_empty;
};

int apple_mbox_ascwrap_v6_get_lifecycle(
	struct apple_mbox *mbox,
	struct apple_mbox_ascwrap_v6_lifecycle *lifecycle)
{
	if (!mbox || !lifecycle)
		return -EINVAL;
	if (READ_ONCE(mbox->removing))
		return -ENODEV;
	if (!mbox->hw->is_ascwrap_v6)
		return -EOPNOTSUPP;

	*lifecycle = mbox->ascwrap_v6;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_mbox_ascwrap_v6_get_lifecycle);

int apple_mbox_ascwrap_v6_require_safe_cpu_lifecycle(struct apple_mbox *mbox)
{
	if (!mbox || !mbox->hw->is_ascwrap_v6)
		return -EOPNOTSUPP;
	if (READ_ONCE(mbox->removing))
		return -ENODEV;

	/* Future provider revisions may report additional lifecycle work. */
	if (mbox->ascwrap_v6.missing)
		return -EOPNOTSUPP;

	return 0;
}
EXPORT_SYMBOL_GPL(apple_mbox_ascwrap_v6_require_safe_cpu_lifecycle);

static void apple_mbox_ascwrap_v6_stop_cpu_locked(struct apple_mbox *mbox)
{
	void __iomem *cpu_control;
	u32 val;

	if (!mbox->ascwrap_v6_cpu_running)
		return;

	cpu_control = (u8 __iomem *)mbox->wrapper_regs +
		      APPLE_ASCWRAP_V6_CPU_CONTROL;

	/* AppleASCWrapV6::_runCPU(false): clear RUN, reread, clear bit 5. */
	val = readl(cpu_control);
	writel(val & ~APPLE_ASCWRAP_V6_CPU_RUN, cpu_control);
	val = readl(cpu_control);
	writel(val & ~APPLE_ASCWRAP_V6_CPU_STOP_SECOND_CLEAR, cpu_control);

	mbox->ascwrap_v6_cpu_running = false;
	pm_runtime_mark_last_busy(mbox->dev);
	pm_runtime_put_autosuspend(mbox->dev);
}

/*
 * A failed resume leaves the runtime PM error set, and runtime PM then fails
 * every later resume with -EINVAL without retrying the power domain.  The
 * provider keeps no state across a failed resume, so mark it suspended again
 * and let the next start retry the domain.
 */
static void apple_mbox_clear_resume_error(struct apple_mbox *mbox)
{
	if (mbox->dev->power.runtime_error)
		pm_runtime_set_suspended(mbox->dev);
}

int apple_mbox_ascwrap_v6_start_cpu(struct apple_mbox *mbox)
{
	void __iomem *cpu_control;
	u32 val;
	int ret = 0;

	if (!mbox || !mbox->hw->is_ascwrap_v6 || !mbox->wrapper_regs)
		return -EOPNOTSUPP;

	mutex_lock(&mbox->lifecycle_lock);
	if (mbox->removing) {
		ret = -ENODEV;
		goto out_unlock;
	}
	if (mbox->ascwrap_v6.missing) {
		ret = -EOPNOTSUPP;
		goto out_unlock;
	}
	if (mbox->ascwrap_v6_cpu_running) {
		ret = -EBUSY;
		goto out_unlock;
	}

	ret = pm_runtime_resume_and_get(mbox->dev);
	if (ret) {
		dev_err(mbox->dev,
			"ASC CPU start: runtime resume failed: %d (active=%d status=%d disable_depth=%d runtime_error=%d)\n",
			ret, mbox->active, mbox->dev->power.runtime_status,
			mbox->dev->power.disable_depth,
			mbox->dev->power.runtime_error);
		apple_mbox_clear_resume_error(mbox);
		goto out_unlock;
	}

	cpu_control = (u8 __iomem *)mbox->wrapper_regs +
		      APPLE_ASCWRAP_V6_CPU_CONTROL;

	/* AppleASCWrapV6::_runCPU(true): read, set RUN, write. */
	val = readl(cpu_control);
	writel(val | APPLE_ASCWRAP_V6_CPU_RUN, cpu_control);
	mbox->ascwrap_v6_cpu_running = true;

out_unlock:
	mutex_unlock(&mbox->lifecycle_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_mbox_ascwrap_v6_start_cpu);

int apple_mbox_ascwrap_v6_stop_cpu(struct apple_mbox *mbox)
{
	int ret = 0;

	if (!mbox || !mbox->hw->is_ascwrap_v6 || !mbox->wrapper_regs)
		return -EOPNOTSUPP;

	mutex_lock(&mbox->lifecycle_lock);
	if (mbox->removing) {
		ret = -ENODEV;
		goto out_unlock;
	}

	apple_mbox_ascwrap_v6_stop_cpu_locked(mbox);

out_unlock:
	mutex_unlock(&mbox->lifecycle_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_mbox_ascwrap_v6_stop_cpu);

int apple_mbox_send(struct apple_mbox *mbox, const struct apple_mbox_msg msg,
		    bool atomic)
{
	unsigned long flags;
	int ret;
	u32 mbox_ctrl;
	long t;

	spin_lock_irqsave(&mbox->tx_lock, flags);
	if (unlikely(READ_ONCE(mbox->removing))) {
		spin_unlock_irqrestore(&mbox->tx_lock, flags);
		return -ENODEV;
	}
	mbox_ctrl = readl_relaxed(mbox->regs + mbox->hw->a2i_control);

	while (mbox_ctrl & mbox->hw->control_full) {
		if (atomic || mbox->irq_send_empty < 0) {
			ret = readl_poll_timeout_atomic(
				mbox->regs + mbox->hw->a2i_control, mbox_ctrl,
				!(mbox_ctrl & mbox->hw->control_full), 100,
				APPLE_MBOX_TX_TIMEOUT * 1000);

			if (ret) {
				spin_unlock_irqrestore(&mbox->tx_lock, flags);
				return ret;
			}

			break;
		}
		/*
		 * The interrupt is level triggered and will keep firing as long as the
		 * FIFO is empty. It will also keep firing if the FIFO was empty
		 * at any point in the past until it has been acknowledged at the
		 * mailbox level. By acknowledging it here we can ensure that we will
		 * only get the interrupt once the FIFO has been cleared again.
		 * If the FIFO is already empty before the ack it will fire again
		 * immediately after the ack.
		 */
		if (mbox->hw->has_irq_controls) {
			writel_relaxed(mbox->hw->irq_bit_send_empty,
				       mbox->regs + mbox->hw->irq_ack);
		}
		reinit_completion(&mbox->tx_empty);
		if (!mbox->tx_irq_unmasked) {
			mbox->tx_irq_unmasked = true;
			enable_irq(mbox->irq_send_empty);
		}
		spin_unlock_irqrestore(&mbox->tx_lock, flags);

		t = wait_for_completion_interruptible_timeout(
			&mbox->tx_empty,
			msecs_to_jiffies(APPLE_MBOX_TX_TIMEOUT));
		if (t <= 0) {
			/* A late IRQ may already have masked this under tx_lock. */
			spin_lock_irqsave(&mbox->tx_lock, flags);
			if (mbox->tx_irq_unmasked) {
				disable_irq_nosync(mbox->irq_send_empty);
				mbox->tx_irq_unmasked = false;
			}
			spin_unlock_irqrestore(&mbox->tx_lock, flags);
			return t < 0 ? t : -ETIMEDOUT;
		}

		spin_lock_irqsave(&mbox->tx_lock, flags);
		if (unlikely(READ_ONCE(mbox->removing))) {
			spin_unlock_irqrestore(&mbox->tx_lock, flags);
			return -ENODEV;
		}
		mbox_ctrl = readl_relaxed(mbox->regs + mbox->hw->a2i_control);
	}

	writeq_relaxed(msg.msg0, mbox->regs + mbox->hw->a2i_send0);
	writeq_relaxed(FIELD_PREP(APPLE_MBOX_MSG1_MSG, msg.msg1),
		       mbox->regs + mbox->hw->a2i_send1);

	spin_unlock_irqrestore(&mbox->tx_lock, flags);

	return 0;
}
EXPORT_SYMBOL(apple_mbox_send);

static irqreturn_t apple_mbox_send_empty_irq(int irq, void *data)
{
	struct apple_mbox *mbox = data;

	/*
	 * We don't need to acknowledge the interrupt at the mailbox level
	 * here even if supported by the hardware. It will keep firing but that
	 * doesn't matter since it's disabled at the main interrupt controller.
	 * apple_mbox_send will acknowledge it before enabling
	 * it at the main controller again.
	 */
	spin_lock(&mbox->tx_lock);
	if (mbox->tx_irq_unmasked) {
		disable_irq_nosync(mbox->irq_send_empty);
		mbox->tx_irq_unmasked = false;
	}
	complete(&mbox->tx_empty);
	spin_unlock(&mbox->tx_lock);

	return IRQ_HANDLED;
}

static int apple_mbox_poll_locked(struct apple_mbox *mbox)
{
	struct apple_mbox_msg msg;
	int ret = 0;
	u32 mbox_ctrl;

	/* A stopped receiver may still have firmware traffic in its FIFO. */
	if (!mbox->rx)
		return 0;
	mbox_ctrl = readl_relaxed(mbox->regs + mbox->hw->i2a_control);

	while (!(mbox_ctrl & mbox->hw->control_empty)) {
		msg.msg0 = readq_relaxed(mbox->regs + mbox->hw->i2a_recv0);
		msg.msg1 = FIELD_GET(
			APPLE_MBOX_MSG1_MSG,
			readq_relaxed(mbox->regs + mbox->hw->i2a_recv1));

		mbox->rx(mbox, msg, mbox->cookie);
		ret++;
		mbox_ctrl = readl_relaxed(mbox->regs + mbox->hw->i2a_control);
	}

	/*
	 * The interrupt will keep firing even if there are no more messages
	 * unless we also acknowledge it at the mailbox level here.
	 * There's no race if a message comes in between the check in the while
	 * loop above and the ack below: If a new messages arrives inbetween
	 * those two the interrupt will just fire again immediately after the
	 * ack since it's level triggered.
	 */
	if (mbox->hw->has_irq_controls) {
		writel_relaxed(mbox->hw->irq_bit_recv_not_empty,
			       mbox->regs + mbox->hw->irq_ack);
	}

	return ret;
}

static irqreturn_t apple_mbox_recv_irq(int irq, void *data)
{
	struct apple_mbox *mbox = data;

	apple_mbox_lab_rx_entry(mbox);
	spin_lock(&mbox->rx_lock);
	if (likely(!READ_ONCE(mbox->removing)))
		apple_mbox_poll_locked(mbox);
	spin_unlock(&mbox->rx_lock);
	apple_mbox_lab_wake(mbox);

	return IRQ_HANDLED;
}

int apple_mbox_poll(struct apple_mbox *mbox)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&mbox->rx_lock, flags);
	if (unlikely(READ_ONCE(mbox->removing)))
		ret = -ENODEV;
	else
		ret = apple_mbox_poll_locked(mbox);
	spin_unlock_irqrestore(&mbox->rx_lock, flags);

	return ret;
}
EXPORT_SYMBOL(apple_mbox_poll);

int apple_mbox_start(struct apple_mbox *mbox)
{
	u32 control;
	int ret = 0;

	mutex_lock(&mbox->lifecycle_lock);
	if (mbox->removing) {
		ret = -ENODEV;
		goto out_unlock;
	}
	if (mbox->active)
		goto out_unlock;

	ret = pm_runtime_resume_and_get(mbox->dev);
	if (ret) {
		dev_err(mbox->dev,
			"mailbox start: runtime resume failed: %d (active=%d status=%d disable_depth=%d runtime_error=%d)\n",
			ret, mbox->active, mbox->dev->power.runtime_status,
			mbox->dev->power.disable_depth,
			mbox->dev->power.runtime_error);
		apple_mbox_clear_resume_error(mbox);
		goto out_unlock;
	}

	if (mbox->hw->ap_initializes_mailboxes) {
		/* The primary T8140 outbox requires its enable bit before use. */
		control = readl(mbox->regs + mbox->hw->i2a_control);
		writel(control | BIT(0), mbox->regs + mbox->hw->i2a_control);
	}

	/*
	 * Only some variants of this mailbox HW provide interrupt control
	 * at the mailbox level. We therefore need to handle enabling/disabling
	 * interrupts at the main interrupt controller anyway for hardware that
	 * doesn't. Just always keep the interrupts we care about enabled at
	 * the mailbox level so that both hardware revisions behave almost
	 * the same.
	 */
	if (mbox->hw->has_irq_controls) {
		writel_relaxed(mbox->hw->irq_bit_recv_not_empty |
				       mbox->hw->irq_bit_send_empty,
			       mbox->regs + mbox->hw->irq_enable);
	}

	enable_irq(mbox->irq_recv_not_empty);
	mbox->active = true;

out_unlock:
	mutex_unlock(&mbox->lifecycle_lock);
	return ret;
}
EXPORT_SYMBOL(apple_mbox_start);

static void apple_mbox_stop_locked(struct apple_mbox *mbox)
{
	if (!mbox->active)
		return;

	mbox->active = false;
	disable_irq(mbox->irq_recv_not_empty);
	pm_runtime_mark_last_busy(mbox->dev);
	pm_runtime_put_autosuspend(mbox->dev);
}

void apple_mbox_stop(struct apple_mbox *mbox)
{
	mutex_lock(&mbox->lifecycle_lock);
	apple_mbox_stop_locked(mbox);
	mutex_unlock(&mbox->lifecycle_lock);
}
EXPORT_SYMBOL(apple_mbox_stop);

/*
 * Stop the provider-owned CPU, then quiesce every transport edge before
 * devres releases IRQs or MMIO mappings. Device links order consumer removal
 * first; the callback clear and waiter wake also cover future consumers that
 * violate that ownership rule.
 */
static void apple_mbox_quiesce(void *data)
{
	struct apple_mbox *mbox = data;
	unsigned long flags;
	bool had_rx;
	bool was_active;

	mutex_lock(&mbox->lifecycle_lock);
	if (mbox->removing) {
		mutex_unlock(&mbox->lifecycle_lock);
		return;
	}
	apple_mbox_ascwrap_v6_stop_cpu_locked(mbox);

	/* Serialize the removal fence with every MMIO send critical section. */
	spin_lock_irqsave(&mbox->tx_lock, flags);
	WRITE_ONCE(mbox->removing, true);
	spin_unlock_irqrestore(&mbox->tx_lock, flags);

	/*
	 * Publish the inactive state under the lifecycle lock, then drop it
	 * before disable_irq() waits for a callback. A callback is allowed to
	 * finish a send or stop request without an rx_lock -> lifecycle_lock
	 * inversion. The removal fence prevents any matching restart.
	 */
	was_active = mbox->active;
	mbox->active = false;
	mutex_unlock(&mbox->lifecycle_lock);
	if (was_active) {
		disable_irq(mbox->irq_recv_not_empty);
		pm_runtime_mark_last_busy(mbox->dev);
		pm_runtime_put_autosuspend(mbox->dev);
	}

	/* Serialize once-only masking with the IRQ and send-error paths. */
	spin_lock_irqsave(&mbox->tx_lock, flags);
	if (mbox->tx_irq_unmasked) {
		disable_irq_nosync(mbox->irq_send_empty);
		mbox->tx_irq_unmasked = false;
	}
	spin_unlock_irqrestore(&mbox->tx_lock, flags);
	/* The handler takes tx_lock, so drain it after releasing the lock. */
	if (mbox->irq_send_empty >= 0)
		synchronize_irq(mbox->irq_send_empty);
	complete_all(&mbox->tx_empty);
	synchronize_irq(mbox->irq_recv_not_empty);

	spin_lock_irqsave(&mbox->rx_lock, flags);
	had_rx = mbox->rx;
	mbox->rx = NULL;
	mbox->cookie = NULL;
	spin_unlock_irqrestore(&mbox->rx_lock, flags);

	if (WARN_ON_ONCE(had_rx))
		dev_warn(mbox->dev,
			 "consumer callback survived provider remove ordering\n");

	pm_runtime_barrier(mbox->dev);
}

struct apple_mbox *apple_mbox_get(struct device *dev, int index)
{
	struct of_phandle_args args;
	struct platform_device *pdev;
	struct apple_mbox *mbox;
	int ret;

	ret = of_parse_phandle_with_args(dev->of_node, "mboxes", "#mbox-cells",
					 index, &args);
	if (ret || !args.np)
		return ERR_PTR(ret);

	pdev = of_find_device_by_node(args.np);
	of_node_put(args.np);

	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	mbox = platform_get_drvdata(pdev);
	if (!mbox) {
		mbox = ERR_PTR(-EPROBE_DEFER);
		goto out_put_pdev;
	}

	if (!device_link_add(dev, &pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		mbox = ERR_PTR(-ENODEV);
		goto out_put_pdev;
	}

out_put_pdev:
	put_device(&pdev->dev);

	return mbox;
}
EXPORT_SYMBOL(apple_mbox_get);

struct apple_mbox *apple_mbox_get_byname(struct device *dev, const char *name)
{
	int index;

	index = of_property_match_string(dev->of_node, "mbox-names", name);
	if (index < 0)
		return ERR_PTR(index);

	return apple_mbox_get(dev, index);
}
EXPORT_SYMBOL(apple_mbox_get_byname);

static int apple_mbox_probe(struct platform_device *pdev)
{
	int ret;
	char *irqname;
	const char *firmware_role;
	struct apple_mbox *mbox;
	struct device *dev = &pdev->dev;
	struct resource *res, *iop_vbar_res;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	mbox->dev = &pdev->dev;
	mbox->hw = of_device_get_match_data(dev);
	if (!mbox->hw)
		return -EINVAL;

	if (mbox->hw->is_ascwrap_v6) {
		/*
		 * Validate the entire provider identity before mapping either
		 * resource.  These are fixed T8140 CPU-physical windows and role
		 * assignments, not generic consumer-provided aliases.
		 */
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "wrapper");
		iop_vbar_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							    "iop-vbar");
		if (!res || !iop_vbar_res ||
		    resource_size(res) != APPLE_ASCWRAP_V6_WRAPPER_SIZE ||
		    resource_size(iop_vbar_res) != APPLE_ASCWRAP_V6_IOP_VBAR_SIZE)
			return dev_err_probe(dev, -EINVAL,
					     "invalid ASCWrap v6 provider resources\n");

		ret = of_property_read_string(dev->of_node, "apple,firmware-role",
					      &firmware_role);
		if (ret)
			return dev_err_probe(dev, ret,
					     "missing ASCWrap v6 firmware role\n");

		if (!strcmp(firmware_role, "GFX")) {
			mbox->ascwrap_v6.role = APPLE_MBOX_ASCWRAP_V6_ROLE_GFX;
			if (res->start != APPLE_T8140_GFX_WRAPPER_BASE ||
			    iop_vbar_res->start != APPLE_T8140_GFX_IOP_VBAR_BASE)
				return dev_err_probe(dev, -EINVAL,
						     "GFX ASCWrap v6 resources do not match T8140\n");
		} else if (!strcmp(firmware_role, "GFX1")) {
			mbox->ascwrap_v6.role = APPLE_MBOX_ASCWRAP_V6_ROLE_GFX1;
			if (res->start != APPLE_T8140_GFX1_WRAPPER_BASE ||
			    iop_vbar_res->start != APPLE_T8140_GFX1_IOP_VBAR_BASE)
				return dev_err_probe(dev, -EINVAL,
						     "GFX1 ASCWrap v6 resources do not match T8140\n");
		} else {
			return dev_err_probe(dev, -EINVAL,
					     "invalid ASCWrap v6 firmware role\n");
		}

		mbox->ascwrap_v6.wrapper_start = res->start;
		mbox->ascwrap_v6.wrapper_size = resource_size(res);
		mbox->ascwrap_v6.iop_vbar_start = iop_vbar_res->start;
		mbox->ascwrap_v6.iop_vbar_size = resource_size(iop_vbar_res);
		/* resetState() is software-only; the matching stop lives above. */
		mbox->ascwrap_v6.missing = 0;

		mbox->wrapper_regs =
			devm_platform_ioremap_resource_byname(pdev, "wrapper");
		if (IS_ERR(mbox->wrapper_regs))
			return PTR_ERR(mbox->wrapper_regs);

		mbox->iop_vbar_regs =
			devm_platform_ioremap_resource_byname(pdev, "iop-vbar");
		if (IS_ERR(mbox->iop_vbar_regs))
			return PTR_ERR(mbox->iop_vbar_regs);

		mbox->regs = (u8 __iomem *)mbox->wrapper_regs +
			     mbox->hw->reg_offset;
	} else {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res || resource_size(res) < mbox->hw->reg_offset +
						mbox->hw->i2a_recv1 + sizeof(u64))
			return dev_err_probe(dev, -EINVAL,
					     "mailbox register window is too small\n");

		mbox->regs = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(mbox->regs))
			return PTR_ERR(mbox->regs);
		mbox->regs = (u8 __iomem *)mbox->regs + mbox->hw->reg_offset;
	}

	mbox->irq_recv_not_empty =
		platform_get_irq_byname(pdev, "recv-not-empty");
	if (mbox->irq_recv_not_empty < 0)
		return mbox->irq_recv_not_empty;

	/*
	 * Some coprocessors (e.g. the T6021 ANE ASC) have no send-empty line.
	 * apple_mbox_send then polls the A2I control register instead.
	 */
	mbox->irq_send_empty =
		platform_get_irq_byname_optional(pdev, "send-empty");
	if (mbox->irq_send_empty < 0 && mbox->irq_send_empty != -ENXIO)
		return mbox->irq_send_empty;

	spin_lock_init(&mbox->rx_lock);
	spin_lock_init(&mbox->tx_lock);
	mutex_init(&mbox->lifecycle_lock);
	init_completion(&mbox->tx_empty);

	irqname = devm_kasprintf(dev, GFP_KERNEL, "%s-recv", dev_name(dev));
	if (!irqname)
		return -ENOMEM;

	ret = devm_request_irq(dev, mbox->irq_recv_not_empty,
			       apple_mbox_recv_irq,
			       IRQF_NO_AUTOEN | IRQF_NO_SUSPEND, irqname, mbox);
	if (ret)
		return ret;

	if (mbox->irq_send_empty >= 0) {
		irqname = devm_kasprintf(dev, GFP_KERNEL, "%s-send",
					 dev_name(dev));
		if (!irqname)
			return -ENOMEM;

		ret = devm_request_irq(dev, mbox->irq_send_empty,
				       apple_mbox_send_empty_irq,
				       IRQF_NO_AUTOEN | IRQF_NO_SUSPEND,
				       irqname, mbox);
		if (ret)
			return ret;
	}

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, apple_mbox_quiesce, mbox);
	if (ret)
		return ret;

	/* Restrict this disposable lab hook to the owner's j314s AOP mailbox. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (of_machine_is_compatible("apple,j314s") && res &&
	    res->start == 0x293408000ULL) {
		WRITE_ONCE(lab_aop_mbox, mbox);
		ret = devm_add_action_or_reset(dev, apple_mbox_lab_release, mbox);
		if (ret)
			return ret;
	}
	platform_set_drvdata(pdev, mbox);
	/* Retain identity even if the live DT fails the board-selection check. */
	if (res && res->start == 0x293408000ULL) {
		lab_resource = res->start;
		WRITE_ONCE(lab_diag_dev, dev);
	}
	return 0;
}

static void apple_mbox_remove(struct platform_device *pdev)
{
	struct apple_mbox *mbox = platform_get_drvdata(pdev);

	if (READ_ONCE(lab_diag_dev) == &pdev->dev)
		WRITE_ONCE(lab_diag_dev, NULL);
	apple_mbox_lab_release(mbox);
	/* Block new phandle lookups before quiescing the provider. */
	platform_set_drvdata(pdev, NULL);
	apple_mbox_quiesce(mbox);
}

static const struct apple_mbox_hw apple_mbox_t8015_hw = {
	.control_full = APPLE_ASC_MBOX_CONTROL_FULL,
	.control_empty = APPLE_ASC_MBOX_CONTROL_EMPTY,

	.a2i_control = APPLE_T8015_MBOX_A2I_CONTROL,
	.a2i_send0 = APPLE_ASC_MBOX_A2I_SEND0,
	.a2i_send1 = APPLE_ASC_MBOX_A2I_SEND1,

	.i2a_control = APPLE_T8015_MBOX_I2A_CONTROL,
	.i2a_recv0 = APPLE_ASC_MBOX_I2A_RECV0,
	.i2a_recv1 = APPLE_ASC_MBOX_I2A_RECV1,

	.has_irq_controls = false,
};

static const struct apple_mbox_hw apple_mbox_asc_hw = {
	.control_full = APPLE_ASC_MBOX_CONTROL_FULL,
	.control_empty = APPLE_ASC_MBOX_CONTROL_EMPTY,

	.a2i_control = APPLE_ASC_MBOX_A2I_CONTROL,
	.a2i_send0 = APPLE_ASC_MBOX_A2I_SEND0,
	.a2i_send1 = APPLE_ASC_MBOX_A2I_SEND1,

	.i2a_control = APPLE_ASC_MBOX_I2A_CONTROL,
	.i2a_recv0 = APPLE_ASC_MBOX_I2A_RECV0,
	.i2a_recv1 = APPLE_ASC_MBOX_I2A_RECV1,

	.has_irq_controls = false,
};

static const struct apple_mbox_hw apple_mbox_t8140_asc_hw = {
	.ap_initializes_mailboxes = true,

	.control_full = APPLE_ASC_MBOX_CONTROL_FULL,
	.control_empty = APPLE_ASC_MBOX_CONTROL_EMPTY,

	.a2i_control = APPLE_ASC_MBOX_A2I_CONTROL,
	.a2i_send0 = APPLE_ASC_MBOX_A2I_SEND0,
	.a2i_send1 = APPLE_ASC_MBOX_A2I_SEND1,

	.i2a_control = APPLE_ASC_MBOX_I2A_CONTROL,
	.i2a_recv0 = APPLE_ASC_MBOX_I2A_RECV0,
	.i2a_recv1 = APPLE_ASC_MBOX_I2A_RECV1,

	.has_irq_controls = false,
};

static const struct apple_mbox_hw apple_mbox_ascwrap_v6_hw = {
	.reg_offset = APPLE_ASCWRAP_V6_MBOX_OFFSET,
	.is_ascwrap_v6 = true,

	.control_full = APPLE_ASC_MBOX_CONTROL_FULL,
	.control_empty = APPLE_ASC_MBOX_CONTROL_EMPTY,

	.a2i_control = APPLE_ASC_MBOX_A2I_CONTROL,
	.a2i_send0 = APPLE_ASC_MBOX_A2I_SEND0,
	.a2i_send1 = APPLE_ASC_MBOX_A2I_SEND1,

	.i2a_control = APPLE_ASC_MBOX_I2A_CONTROL,
	.i2a_recv0 = APPLE_ASC_MBOX_I2A_RECV0,
	.i2a_recv1 = APPLE_ASC_MBOX_I2A_RECV1,

	.has_irq_controls = false,
};

static const struct apple_mbox_hw apple_mbox_t8140_aop_setup_hw = {
	.control_full = APPLE_ASC_MBOX_CONTROL_FULL,
	.control_empty = APPLE_ASC_MBOX_CONTROL_EMPTY,

	.a2i_control = APPLE_T8140_AOP_SETUP_A2I_CONTROL,
	.a2i_send0 = APPLE_T8140_AOP_SETUP_A2I_SEND0,
	.a2i_send1 = APPLE_T8140_AOP_SETUP_A2I_SEND1,

	.i2a_control = APPLE_T8140_AOP_SETUP_I2A_CONTROL,
	.i2a_recv0 = APPLE_T8140_AOP_SETUP_I2A_RECV0,
	.i2a_recv1 = APPLE_T8140_AOP_SETUP_I2A_RECV1,

	.has_irq_controls = false,
};

static const struct apple_mbox_hw apple_mbox_m3_hw = {
	.control_full = APPLE_M3_MBOX_CONTROL_FULL,
	.control_empty = APPLE_M3_MBOX_CONTROL_EMPTY,

	.a2i_control = APPLE_M3_MBOX_A2I_CONTROL,
	.a2i_send0 = APPLE_M3_MBOX_A2I_SEND0,
	.a2i_send1 = APPLE_M3_MBOX_A2I_SEND1,

	.i2a_control = APPLE_M3_MBOX_I2A_CONTROL,
	.i2a_recv0 = APPLE_M3_MBOX_I2A_RECV0,
	.i2a_recv1 = APPLE_M3_MBOX_I2A_RECV1,

	.has_irq_controls = true,
	.irq_enable = APPLE_M3_MBOX_IRQ_ENABLE,
	.irq_ack = APPLE_M3_MBOX_IRQ_ACK,
	.irq_bit_recv_not_empty = APPLE_M3_MBOX_IRQ_I2A_NOT_EMPTY,
	.irq_bit_send_empty = APPLE_M3_MBOX_IRQ_A2I_EMPTY,
};

static const struct of_device_id apple_mbox_of_match[] = {
	{ .compatible = "apple,t8140-ascwrap-v6", .data = &apple_mbox_ascwrap_v6_hw },
	{ .compatible = "apple,t8140-asc-mailbox", .data = &apple_mbox_t8140_asc_hw },
	{ .compatible = "apple,t8140-aop-setup-mailbox", .data = &apple_mbox_t8140_aop_setup_hw },
	{ .compatible = "apple,t6030-agx-asc-mailbox", .data = &apple_mbox_asc_hw },
	{ .compatible = "apple,t8122-agx-asc-mailbox", .data = &apple_mbox_asc_hw },
	{ .compatible = "apple,asc-mailbox-v4", .data = &apple_mbox_asc_hw },
	{ .compatible = "apple,t8015-asc-mailbox", .data = &apple_mbox_t8015_hw },
	{ .compatible = "apple,m3-mailbox-v2", .data = &apple_mbox_m3_hw },
	{}
};
MODULE_DEVICE_TABLE(of, apple_mbox_of_match);

static struct platform_driver apple_mbox_driver = {
	.driver = {
		.name = "apple-mailbox",
		.of_match_table = apple_mbox_of_match,
		.pm = &apple_mbox_lab_pm_ops,
	},
	.probe = apple_mbox_probe,
	.remove = apple_mbox_remove,
};
/* Register globally so POST_SUSPEND also exposes an absent AOP provider. */
static int __init apple_mbox_lab_init(void)
{
	int ret;

	ret = register_pm_notifier(&lab_diag_notifier);
	if (ret)
		return ret;
	ret = platform_driver_register(&apple_mbox_driver);
	if (ret)
		unregister_pm_notifier(&lab_diag_notifier);
	return ret;
}
module_init(apple_mbox_lab_init);

static void __exit apple_mbox_lab_exit(void)
{
	unregister_pm_notifier(&lab_diag_notifier);
	platform_driver_unregister(&apple_mbox_driver);
}
module_exit(apple_mbox_lab_exit);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_DESCRIPTION("Apple Mailbox driver");
