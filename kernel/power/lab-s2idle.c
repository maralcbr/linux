// SPDX-License-Identifier: GPL-2.0-only
/* LAB ONLY: volatile PM breadcrumbs. Never survives a hard reset. */
#include <linux/debugfs.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/ktime.h>
#include <linux/moduleparam.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#ifdef CONFIG_ARM64
#include <clocksource/arm_arch_timer.h>
#endif

#define LAB_TRACE_RECORDS 1024

struct lab_trace_record {
	u64 sequence;
	u64 boottime;
	u64 counter;
	u64 device;
	int status;
	char stage[40];
};

static bool lab_s2idle_trace;
core_param(lab_s2idle_trace, lab_s2idle_trace, bool, 0600);
static DEFINE_RAW_SPINLOCK(lab_trace_lock);
static struct lab_trace_record lab_trace_records[LAB_TRACE_RECORDS];
static u64 lab_trace_sequence;
static u64 lab_trace_cycle_start;

void lab_s2idle_trace_step(const char *stage, u64 device, int status)
{
	struct lab_trace_record *record;
	unsigned long flags;

	if (!READ_ONCE(lab_s2idle_trace))
		return;
	raw_spin_lock_irqsave(&lab_trace_lock, flags);
	record = &lab_trace_records[lab_trace_sequence % LAB_TRACE_RECORDS];
	record->sequence = ++lab_trace_sequence;
	record->boottime = ktime_get_boot_fast_ns();
#ifdef CONFIG_ARM64
	record->counter = arch_timer_read_counter();
#endif
	record->device = device;
	record->status = status;
	strscpy(record->stage, stage, sizeof(record->stage));
	raw_spin_unlock_irqrestore(&lab_trace_lock, flags);
}
EXPORT_SYMBOL_GPL(lab_s2idle_trace_step);

static struct lab_trace_record *lab_trace_snapshot(u64 *end)
{
	struct lab_trace_record *records;
	unsigned long flags;

	records = kmalloc_objs(*records, LAB_TRACE_RECORDS);
	if (!records)
		return NULL;
	raw_spin_lock_irqsave(&lab_trace_lock, flags);
	memcpy(records, lab_trace_records, sizeof(lab_trace_records));
	*end = lab_trace_sequence;
	raw_spin_unlock_irqrestore(&lab_trace_lock, flags);
	return records;
}

static int lab_trace_show(struct seq_file *m, void *unused)
{
	struct lab_trace_record *records;
	u64 end, i;

	records = lab_trace_snapshot(&end);
	if (!records)
		return -ENOMEM;
	seq_puts(m, "LAB ONLY: volatile RAM; lost on hard reset\n");
	for (i = end > LAB_TRACE_RECORDS ? end - LAB_TRACE_RECORDS : 0; i < end; i++) {
		struct lab_trace_record *r = &records[i % LAB_TRACE_RECORDS];

		seq_printf(m, "%llu boot_ns=%llu counter=%llu %s device=%llx status=%d\n",
			   r->sequence, r->boottime, r->counter, r->stage,
			   r->device, r->status);
	}
	kfree(records);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(lab_trace);

static int lab_trace_notify(struct notifier_block *nb, unsigned long event, void *data)
{
	struct lab_trace_record *records;
	u64 end, i;

	if (!READ_ONCE(lab_s2idle_trace))
		return NOTIFY_DONE;
	if (event == PM_SUSPEND_PREPARE) {
		lab_s2idle_trace_step("prepare", 0, 0);
		lab_trace_cycle_start = READ_ONCE(lab_trace_sequence) - 1;
	} else if (event == PM_POST_SUSPEND) {
		lab_s2idle_trace_step("post", 0, 0);
		records = lab_trace_snapshot(&end);
		if (!records)
			return NOTIFY_DONE;
		pr_info("LAB-S2IDLE-TRACE BEGIN (volatile RAM)\n");
		for (i = max(lab_trace_cycle_start,
			     end > LAB_TRACE_RECORDS ? end - LAB_TRACE_RECORDS : 0); i < end; i++) {
			struct lab_trace_record *r = &records[i % LAB_TRACE_RECORDS];

			pr_info("LAB-S2IDLE-TRACE %llu boot_ns=%llu counter=%llu %s device=%llx status=%d\n",
				r->sequence, r->boottime, r->counter, r->stage,
				r->device, r->status);
		}
		pr_info("LAB-S2IDLE-TRACE END\n");
		kfree(records);
	}
	return NOTIFY_DONE;
}

static struct notifier_block lab_trace_nb = {
	.notifier_call = lab_trace_notify,
};

static int __init lab_trace_init(void)
{
	debugfs_create_file("pm_lab_s2idle", 0400, NULL, NULL, &lab_trace_fops);
	return register_pm_notifier(&lab_trace_nb);
}
late_initcall(lab_trace_init);
