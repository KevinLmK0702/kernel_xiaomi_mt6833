/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * Window Assisted Load Tracking (WALT) implementation credits:
 * Srivatsa Vaddagiri, Steve Muckle, Syed Rameez Mustafa, Joonwoo Park,
 * Pavan Kumar Kondeti, Olav Haugan
 *
 * 2016-03-06: Integration with EAS/refactoring by Vikram Mulukutla
 *             and Todd Kjos
 */

#include <linux/acpi.h>
#include <linux/syscore_ops.h>
#include <linux/timekeeping.h>
#include <linux/debugfs.h>
#include <linux/pid.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <trace/events/sched.h>
#include "sched.h"
#include "walt.h"

#define WINDOW_STATS_RECENT		0
#define WINDOW_STATS_MAX		1
#define WINDOW_STATS_MAX_RECENT_AVG	2
#define WINDOW_STATS_AVG		3
#define WINDOW_STATS_INVALID_POLICY	4

#define EXITING_TASK_MARKER	0xdeaddead

static __read_mostly unsigned int walt_ravg_hist_size = 5;
static __read_mostly unsigned int walt_window_stats_policy =
	WINDOW_STATS_MAX_RECENT_AVG;
static __read_mostly unsigned int walt_account_wait_time = 1;
static __read_mostly unsigned int walt_freq_account_wait_time = 0;
static __read_mostly unsigned int walt_io_is_busy = 0;

unsigned int sysctl_sched_walt_init_task_load_pct = 15;

/* true -> use PELT based load stats, false -> use window-based load stats */
bool __read_mostly walt_disabled = false;

/*
 * Window size (in ns). Adjust for the tick size so that the window
 * rollover occurs just before the tick boundary.
 */
__read_mostly unsigned int walt_ravg_window =
					    (20000000 / TICK_NSEC) * TICK_NSEC;
#define MIN_SCHED_RAVG_WINDOW ((10000000 / TICK_NSEC) * TICK_NSEC)
#define MAX_SCHED_RAVG_WINDOW ((1000000000 / TICK_NSEC) * TICK_NSEC)

static unsigned int sync_cpu;
static ktime_t ktime_last;
static __read_mostly bool walt_ktime_suspended;

static unsigned int task_load(struct task_struct *p)
{
	return p->ravg.demand;
}

static inline void fixup_cum_window_demand(struct rq *rq, s64 delta)
{
	rq->cum_window_demand += delta;
	if (unlikely((s64)rq->cum_window_demand < 0))
		rq->cum_window_demand = 0;
}

/******************************************************************************
 * Related thread groups (RTG), reduced port of Qualcomm's implementation
 *
 * Qualcomm's RTG couples cgroup colocation, per-cluster group load accounting
 * (grp_time) and EAS preferred-cluster placement to the rtg_boost_freq of the
 * walt governor. None of that infrastructure exists in this kernel, so only
 * the part the governor actually consumes is provided:
 *
 *  - a group is a plain id, stored per task in p->ravg.grp_id,
 *  - a group's load is the sum of the demand of its queued tasks,
 *  - a group counts as "active" once its load exceeds half of what the least
 *    capable CPU can contribute in one WALT window; the walt governor then
 *    raises the utilization to rtg_boost_freq while the group runs.
 *
 * Group ids are assigned with sched_set_group_id() (same name and prototype
 * as Qualcomm's API) and, for manual use, through the debugfs file
 * /sys/kernel/debug/sched_rtg: "echo <pid> <group_id> > ...", group_id 0
 * removes the task from its group. Children inherit the group of their
 * parent.
 *****************************************************************************/
#define MAX_NUM_CGROUP_COLOC_ID	20
#define RTG_BOOST_DEMAND_PCT	50

struct walt_related_thread_group {
	atomic64_t	load;
};

/*
 * Statically allocated on purpose: a group id is looked up from the very
 * first enqueue (i.e. before any late_initcall runs), and a failed dynamic
 * allocation would leave NULL entries behind that every accessor - including
 * the governor's rtgb_active() on a hot path - would have to check.
 */
static struct walt_related_thread_group
		related_thread_groups[MAX_NUM_CGROUP_COLOC_ID];

static atomic64_t walt_rtg_total_load = ATOMIC64_INIT(0);

/*
 * Per-CPU view of the same accounting: the demand a related thread group
 * contributes to one runqueue.  The governor must only raise the frequency of
 * the CPUs a group actually loads, so rtgb_active() is evaluated per CPU (see
 * walt_rtgb_active()) instead of once for the whole system - a globally
 * active group used to push every cluster at the same time.
 *
 * Statically allocated for the same reason as related_thread_groups[].
 */
static atomic64_t rtg_cpu_load[NR_CPUS][MAX_NUM_CGROUP_COLOC_ID];

static inline struct walt_related_thread_group *
walt_lookup_group(unsigned int id)
{
	if (id == 0 || id >= MAX_NUM_CGROUP_COLOC_ID)
		return NULL;

	return &related_thread_groups[id];
}

/* Account a change of @p's demand against its related thread group */
static void walt_grp_load_add(struct rq *rq, struct task_struct *p, s64 delta)
{
	struct walt_related_thread_group *grp;
	int cpu;

	if (!delta)
		return;

	grp = walt_lookup_group(p->ravg.grp_id);
	if (!grp)
		return;

	atomic64_add(delta, &grp->load);
	atomic64_add(delta, &walt_rtg_total_load);

	/*
	 * @rq is the runqueue @p is (or is about to be) queued on.  Every sched
	 * class calls the inc/dec helpers symmetrically on enqueue/dequeue, and
	 * migration goes through deactivate/activate, so the per-CPU view stays
	 * balanced without extra bookkeeping.
	 */
	cpu = rq->cpu;
	if (cpu >= 0 && cpu < nr_cpu_ids)
		atomic64_add(delta, &rtg_cpu_load[cpu][p->ravg.grp_id]);
}

/*
 * Is any related thread group loading @cpu enough to ask the walt governor
 * for rtg_boost_freq?  @cpu's own capacity is the yardstick, so "active"
 * means "the group occupies at least RTG_BOOST_DEMAND_PCT of this CPU":
 * small enough that any real workload triggers the boost, large enough that
 * an idle group does not.
 */
static bool walt_rtgb_active(int cpu)
{
	u64 threshold;
	int i;

	if (cpu < 0 || cpu >= nr_cpu_ids)
		return false;

	threshold = mult_frac((u64)walt_ravg_window,
			      capacity_orig_of(cpu) * RTG_BOOST_DEMAND_PCT,
			      SCHED_CAPACITY_SCALE * 100);

	/*
	 * Cheap global pre-filter: the per-CPU loads are a subset of the total,
	 * so if the total is below the threshold no CPU can be active.
	 */
	if (atomic64_read(&walt_rtg_total_load) < (s64)threshold)
		return false;

	for (i = 1; i < MAX_NUM_CGROUP_COLOC_ID; i++) {
		if (atomic64_read(&rtg_cpu_load[cpu][i]) >= (s64)threshold)
			return true;
	}

	return false;
}

int sched_set_group_id(struct task_struct *p, unsigned int group_id)
{
	struct rq *rq;
	struct rq_flags rf;

	if (group_id >= MAX_NUM_CGROUP_COLOC_ID)
		return -EINVAL;

	if (group_id == p->ravg.grp_id)
		return 0;

	raw_spin_lock_irq(&p->pi_lock);

	rq = __task_rq_lock(p, &rf);
	if (task_on_rq_queued(p))
		walt_grp_load_add(rq, p, -(s64)p->ravg.demand);

	p->ravg.grp_id = group_id;

	if (task_on_rq_queued(p))
		walt_grp_load_add(rq, p, p->ravg.demand);
	__task_rq_unlock(rq, &rf);

	raw_spin_unlock_irq(&p->pi_lock);

	return 0;
}
EXPORT_SYMBOL(sched_set_group_id);

unsigned int sched_get_group_id(struct task_struct *p)
{
	return p->ravg.grp_id;
}
EXPORT_SYMBOL(sched_get_group_id);

#ifdef CONFIG_DEBUG_FS
static int rtg_show(struct seq_file *s, void *unused)
{
	int i, cpu;

	seq_printf(s, "total_load=%lld\n",
		   (long long)atomic64_read(&walt_rtg_total_load));

	seq_printf(s, "active_cpus=");
	for_each_possible_cpu(cpu) {
		if (walt_rtgb_active(cpu))
			seq_printf(s, "%d ", cpu);
	}
	seq_printf(s, "\n");

	for (i = 1; i < MAX_NUM_CGROUP_COLOC_ID; i++) {
		seq_printf(s, "group%d load=%lld cpu_load=", i,
			   (long long)atomic64_read(
					&related_thread_groups[i].load));
		for_each_possible_cpu(cpu)
			seq_printf(s, "%d:%lld ", cpu,
				   (long long)atomic64_read(
						&rtg_cpu_load[cpu][i]));
		seq_printf(s, "\n");
	}

	return 0;
}

static int rtg_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtg_show, NULL);
}

static ssize_t rtg_write(struct file *file, const char __user *ubuf,
			 size_t count, loff_t *ppos)
{
	struct task_struct *task;
	char kbuf[32];
	int pid, gid, ret;

	if (count == 0 || count >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, count))
		return -EFAULT;
	kbuf[count] = '\0';

	if (sscanf(kbuf, "%d %d", &pid, &gid) != 2 || gid < 0)
		return -EINVAL;

	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (task)
		get_task_struct(task);
	rcu_read_unlock();

	if (!task)
		return -ESRCH;

	ret = sched_set_group_id(task, (unsigned int)gid);
	put_task_struct(task);

	return ret ? ret : count;
}

static const struct file_operations rtg_fops = {
	.open		= rtg_open,
	.read		= seq_read,
	.write		= rtg_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init walt_rtg_debugfs_init(void)
{
	debugfs_create_file("sched_rtg", 0644, NULL, NULL, &rtg_fops);

	return 0;
}
late_initcall(walt_rtg_debugfs_init);
#endif /* CONFIG_DEBUG_FS */

void
walt_inc_cumulative_runnable_avg(struct rq *rq,
				 struct task_struct *p)
{
	rq->cumulative_runnable_avg += p->ravg.demand;
	rq->pred_demands_sum += p->ravg.pred_demand;
	walt_grp_load_add(rq, p, (s64)p->ravg.demand);

	/*
	 * Add a task's contribution to the cumulative window demand when
	 *
	 * (1) task is enqueued with on_rq = 1 i.e migration,
	 *     prio/cgroup/class change.
	 * (2) task is waking for the first time in this window.
	 */
	if (p->on_rq || (p->last_sleep_ts < rq->window_start))
		fixup_cum_window_demand(rq, p->ravg.demand);
}

void
walt_dec_cumulative_runnable_avg(struct rq *rq,
				 struct task_struct *p)
{
	rq->cumulative_runnable_avg -= p->ravg.demand;
	BUG_ON((s64)rq->cumulative_runnable_avg < 0);

	rq->pred_demands_sum -= p->ravg.pred_demand;
	if ((s64)rq->pred_demands_sum < 0)
		rq->pred_demands_sum = 0;

	walt_grp_load_add(rq, p, -(s64)p->ravg.demand);

	/*
	 * on_rq will be 1 for sleeping tasks. So check if the task
	 * is migrating or dequeuing in RUNNING state to change the
	 * prio/cgroup/class.
	 */
	if (task_on_rq_migrating(p) || p->state == TASK_RUNNING)
		fixup_cum_window_demand(rq, -(s64)p->ravg.demand);
}

void
walt_fixup_cumulative_runnable_avg(struct rq *rq,
				   struct task_struct *p, u64 new_task_load)
{
	s64 task_load_delta = (s64)new_task_load - task_load(p);

	rq->cumulative_runnable_avg += task_load_delta;
	if ((s64)rq->cumulative_runnable_avg < 0)
		panic("cra less than zero: tld: %lld, task_load(p) = %u\n",
			task_load_delta, task_load(p));

	walt_grp_load_add(rq, p, task_load_delta);

	fixup_cum_window_demand(rq, task_load_delta);
}

/*
 * Adjust the predicted demand sum of @rq for a new prediction of @p. Unlike
 * the demand fixup above this is allowed to underflow: the prediction is a
 * heuristic that can be revised downwards, so clamp instead of panicking.
 */
static void walt_fixup_pred_demand(struct rq *rq, struct task_struct *p,
				   u32 new_pred)
{
	s64 delta = (s64)new_pred - (s64)p->ravg.pred_demand;

	rq->pred_demands_sum += delta;
	if ((s64)rq->pred_demands_sum < 0)
		rq->pred_demands_sum = 0;
}

u64 walt_ktime_clock(void)
{
	/*
	 * Return the last observed timestamp while the system is suspended
	 * (or while timekeeping is already suspended, which happens when MTK's
	 * s2idle path nests syscore_suspend inside a suspend): reading the
	 * clock there trips ktime_get()'s WARN_ON(timekeeping_suspended).
	 */
	if (unlikely(walt_ktime_suspended || timekeeping_suspended))
		return ktime_to_ns(ktime_last);

	ktime_last = ktime_get();
	return ktime_to_ns(ktime_last);
}

/******************************************************************************
 * WALT -> cpufreq governor callbacks (ported from Qualcomm's WALT)
 *****************************************************************************/

DEFINE_PER_CPU(struct waltgov_callback *, waltgov_cb_data);

void waltgov_add_callback(int cpu, struct waltgov_callback *cb,
			  void (*func)(struct waltgov_callback *cb, u64 time,
				       unsigned int flags))
{
	if (WARN_ON(!cb || !func))
		return;

	if (WARN_ON(per_cpu(waltgov_cb_data, cpu)))
		return;

	cb->func = func;
	rcu_assign_pointer(per_cpu(waltgov_cb_data, cpu), cb);
}

void waltgov_remove_callback(int cpu)
{
	rcu_assign_pointer(per_cpu(waltgov_cb_data, cpu), NULL);
}

void waltgov_run_callback(struct rq *rq, unsigned int flags)
{
	struct waltgov_callback *cb;

	rcu_read_lock_sched();
	cb = rcu_dereference_sched(
		*per_cpu_ptr(&waltgov_cb_data, cpu_of(rq)));
	if (cb)
		cb->func(cb, walt_ktime_clock(), flags);
	rcu_read_unlock_sched();
}

/*
 * Extra WALT load information for the "walt" cpufreq governor, ported from
 * Qualcomm's walt_cpu_load: the new task load (nl), the predictive demand sum
 * (pl), the related thread group state and the window start.
 *
 * The utilization itself is deliberately *not* returned here. The governor
 * reads it through boosted_cpu_util(), exactly like the platform's schedutil
 * governor does, so that the schedtune boost of the foreground cgroup is
 * honoured; a raw WALT sum would make every foreground workload look lighter
 * than the platform asked for.
 */
void waltgov_cpu_load(int cpu, struct walt_cpu_load *walt_load)
{
	struct rq *rq = cpu_rq(cpu);
	u64 util, pl, nl;

	memset(walt_load, 0, sizeof(*walt_load));

	if (unlikely(walt_disabled))
		return;

	/*
	 * The RTG boost is independent of sysctl_sched_use_walt_cpu_util: that
	 * knob only decides whether the *scheduler* takes its placement
	 * decisions from WALT, while a loaded related thread group still wants
	 * its CPUs pushed.
	 */
	walt_load->rtgb_active = walt_rtgb_active(cpu);

	if (!sysctl_sched_use_walt_cpu_util)
		return;

	util = rq->prev_runnable_sum;
	util <<= SCHED_CAPACITY_SHIFT;
	do_div(util, walt_ravg_window);

	pl = rq->pred_demands_sum;
	pl <<= SCHED_CAPACITY_SHIFT;
	do_div(pl, walt_ravg_window);
	walt_load->pl = (unsigned long)pl;

	nl = rq->nt_prev_runnable_sum;
	nl <<= SCHED_CAPACITY_SHIFT;
	do_div(nl, walt_ravg_window);
	walt_load->nl = min_t(u64, nl, util);

	walt_load->ws = rq->window_start;
}

static void walt_resume(void)
{
	walt_ktime_suspended = false;
}

static int walt_suspend(void)
{
	/*
	 * Do not read the clock here: this syscore callback has been observed
	 * to run after timekeeping_suspend() on this platform and ktime_get()
	 * warns once timekeeping is suspended. walt_ktime_clock() keeps
	 * ktime_last up to date, so the timestamp frozen across suspend is the
	 * last value observed before it.
	 */
	walt_ktime_suspended = true;
	return 0;
}

static struct syscore_ops walt_syscore_ops = {
	.resume	= walt_resume,
	.suspend = walt_suspend
};

static int __init walt_init_ops(void)
{
	register_syscore_ops(&walt_syscore_ops);
	return 0;
}
late_initcall(walt_init_ops);

#ifdef CONFIG_CFS_BANDWIDTH
void walt_inc_cfs_cumulative_runnable_avg(struct cfs_rq *cfs_rq,
		struct task_struct *p)
{
	cfs_rq->cumulative_runnable_avg += p->ravg.demand;
}

void walt_dec_cfs_cumulative_runnable_avg(struct cfs_rq *cfs_rq,
		struct task_struct *p)
{
	cfs_rq->cumulative_runnable_avg -= p->ravg.demand;
}
#endif

static int exiting_task(struct task_struct *p)
{
	if (p->flags & PF_EXITING) {
		if (p->ravg.sum_history[0] != EXITING_TASK_MARKER) {
			p->ravg.sum_history[0] = EXITING_TASK_MARKER;
		}
		return 1;
	}
	return 0;
}

/*
 * A task is considered "new" for the first WALT_NEW_TASK_ACTIVE_WINDOWS
 * windows of its life (~100ms at the 20ms default window). The load of such
 * tasks is tracked separately so that the walt governor can spot app
 * startup / fork bursts and ramp up without waiting for the demand to build.
 */
#define WALT_NEW_TASK_ACTIVE_WINDOWS	5

static inline bool is_new_task(struct task_struct *p)
{
	return !is_idle_task(p) && !exiting_task(p) &&
		p->ravg.active_windows < WALT_NEW_TASK_ACTIVE_WINDOWS;
}

static int __init set_walt_ravg_window(char *str)
{
	unsigned int adj_window;
	bool no_walt = walt_disabled;

	get_option(&str, &walt_ravg_window);

	/* Adjust for CONFIG_HZ */
	adj_window = (walt_ravg_window / TICK_NSEC) * TICK_NSEC;

	/* Warn if we're a bit too far away from the expected window size */
	WARN(adj_window < walt_ravg_window - NSEC_PER_MSEC,
	     "tick-adjusted window size %u, original was %u\n", adj_window,
	     walt_ravg_window);

	walt_ravg_window = adj_window;

	walt_disabled = walt_disabled ||
			(walt_ravg_window < MIN_SCHED_RAVG_WINDOW ||
			 walt_ravg_window > MAX_SCHED_RAVG_WINDOW);

	WARN(!no_walt && walt_disabled,
	     "invalid window size, disabling WALT\n");

	return 0;
}

early_param("walt_ravg_window", set_walt_ravg_window);

static void
update_window_start(struct rq *rq, u64 wallclock)
{
	s64 delta;
	int nr_windows;

	delta = wallclock - rq->window_start;
	/* If the MPM global timer is cleared, set delta as 0 to avoid kernel BUG happening */
	if (delta < 0) {
		delta = 0;
		WARN_ONCE(1, "WALT wallclock appears to have gone backwards or reset\n");
	}

	if (delta < walt_ravg_window)
		return;

	nr_windows = div64_u64(delta, walt_ravg_window);
	rq->window_start += (u64)nr_windows * (u64)walt_ravg_window;

	rq->cum_window_demand = rq->cumulative_runnable_avg;
}

extern unsigned long capacity_curr_of(int cpu);
/*
 * Translate absolute delta time accounted on a CPU
 * to a scale where 1024 is the capacity of the most
 * capable CPU running at FMAX
 */
static u64 scale_exec_time(u64 delta, struct rq *rq)
{
	unsigned long capcurr = capacity_curr_of(cpu_of(rq));

	return (delta * capcurr) >> SCHED_CAPACITY_SHIFT;
}

static int cpu_is_waiting_on_io(struct rq *rq)
{
	if (!walt_io_is_busy)
		return 0;

	return atomic_read(&rq->nr_iowait);
}

void walt_account_irqtime(int cpu, struct task_struct *curr,
				 u64 delta, u64 wallclock)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long flags, nr_windows;
	u64 cur_jiffies_ts;

	raw_spin_lock_irqsave(&rq->lock, flags);

	/*
	 * cputime (wallclock) uses sched_clock so use the same here for
	 * consistency.
	 */
	delta += sched_clock() - wallclock;
	cur_jiffies_ts = get_jiffies_64();

	if (is_idle_task(curr))
		walt_update_task_ravg(curr, rq, IRQ_UPDATE, walt_ktime_clock(),
				 delta);

	nr_windows = cur_jiffies_ts - rq->irqload_ts;

	if (nr_windows) {
		if (nr_windows < 10) {
			/* Decay CPU's irqload by 3/4 for each window. */
			rq->avg_irqload *= (3 * nr_windows);
			rq->avg_irqload = div64_u64(rq->avg_irqload,
						    4 * nr_windows);
		} else {
			rq->avg_irqload = 0;
		}
		rq->avg_irqload += rq->cur_irqload;
		rq->cur_irqload = 0;
	}

	rq->cur_irqload += delta;
	rq->irqload_ts = cur_jiffies_ts;
	raw_spin_unlock_irqrestore(&rq->lock, flags);
}


#define WALT_HIGH_IRQ_TIMEOUT 3

u64 walt_irqload(int cpu) {
	struct rq *rq = cpu_rq(cpu);
	s64 delta;
	delta = get_jiffies_64() - rq->irqload_ts;

        /*
	 * Current context can be preempted by irq and rq->irqload_ts can be
	 * updated by irq context so that delta can be negative.
	 * But this is okay and we can safely return as this means there
	 * was recent irq occurrence.
	 */

        if (delta < WALT_HIGH_IRQ_TIMEOUT)
		return rq->avg_irqload;
        else
		return 0;
}

int walt_cpu_high_irqload(int cpu) {
	return walt_irqload(cpu) >= sysctl_sched_walt_cpu_high_irqload;
}

static int account_busy_for_cpu_time(struct rq *rq, struct task_struct *p,
				     u64 irqtime, int event)
{
	if (is_idle_task(p)) {
		/* TASK_WAKE && TASK_MIGRATE is not possible on idle task! */
		if (event == PICK_NEXT_TASK)
			return 0;

		/* PUT_PREV_TASK, TASK_UPDATE && IRQ_UPDATE are left */
		return irqtime || cpu_is_waiting_on_io(rq);
	}

	if (event == TASK_WAKE)
		return 0;

	if (event == PUT_PREV_TASK || event == IRQ_UPDATE ||
					 event == TASK_UPDATE)
		return 1;

	/* Only TASK_MIGRATE && PICK_NEXT_TASK left */
	return walt_freq_account_wait_time;
}

/*
 * Account cpu activity in its busy time counters (rq->curr/prev_runnable_sum)
 */
static void update_cpu_busy_time(struct task_struct *p, struct rq *rq,
	     int event, u64 wallclock, u64 irqtime)
{
	int new_window, nr_full_windows = 0;
	int p_is_curr_task = (p == rq->curr);
	u64 mark_start = p->ravg.mark_start;
	u64 window_start = rq->window_start;
	u32 window_size = walt_ravg_window;
	u64 delta;
	u64 nt_delta = 0;

	new_window = mark_start < window_start;
	if (new_window) {
		nr_full_windows = div64_u64((window_start - mark_start),
						window_size);
		if (p->ravg.active_windows < USHRT_MAX)
			p->ravg.active_windows++;
	}

	/* Handle per-task window rollover. We don't care about the idle
	 * task or exiting tasks. */
	if (new_window && !is_idle_task(p) && !exiting_task(p)) {
		u32 curr_window = 0;

		if (!nr_full_windows)
			curr_window = p->ravg.curr_window;

		p->ravg.prev_window = curr_window;
		p->ravg.curr_window = 0;
	}

	if (!account_busy_for_cpu_time(rq, p, irqtime, event)) {
		/* account_busy_for_cpu_time() = 0, so no update to the
		 * task's current window needs to be made. This could be
		 * for example
		 *
		 *   - a wakeup event on a task within the current
		 *     window (!new_window below, no action required),
		 *   - switching to a new task from idle (PICK_NEXT_TASK)
		 *     in a new window where irqtime is 0 and we aren't
		 *     waiting on IO */

		if (!new_window)
			return;

		/* A new window has started. The RQ demand must be rolled
		 * over if p is the current task. */
		if (p_is_curr_task) {
			u64 prev_sum = 0;

			/* p is either idle task or an exiting task */
			if (!nr_full_windows) {
				prev_sum = rq->curr_runnable_sum;
			}

			rq->prev_runnable_sum = prev_sum;
			rq->curr_runnable_sum = 0;

			/*
			 * Roll the new task load over with the runnable sum.
			 * Idle/exiting tasks never contribute to it.
			 */
			rq->nt_prev_runnable_sum = nr_full_windows ? 0 :
						rq->nt_curr_runnable_sum;
			rq->nt_curr_runnable_sum = 0;
		}

		return;
	}

	if (!new_window) {
		/* account_busy_for_cpu_time() = 1 so busy time needs
		 * to be accounted to the current window. No rollover
		 * since we didn't start a new window. An example of this is
		 * when a task starts execution and then sleeps within the
		 * same window. */

		if (!irqtime || !is_idle_task(p) || cpu_is_waiting_on_io(rq))
			delta = wallclock - mark_start;
		else
			delta = irqtime;
		delta = scale_exec_time(delta, rq);
		rq->curr_runnable_sum += delta;
		if (!is_idle_task(p) && !exiting_task(p))
			p->ravg.curr_window += delta;
		if (is_new_task(p))
			rq->nt_curr_runnable_sum += delta;

		return;
	}

	if (!p_is_curr_task) {
		/* account_busy_for_cpu_time() = 1 so busy time needs
		 * to be accounted to the current window. A new window
		 * has also started, but p is not the current task, so the
		 * window is not rolled over - just split up and account
		 * as necessary into curr and prev. The window is only
		 * rolled over when a new window is processed for the current
		 * task.
		 *
		 * Irqtime can't be accounted by a task that isn't the
		 * currently running task. */

		if (!nr_full_windows) {
			/* A full window hasn't elapsed, account partial
			 * contribution to previous completed window. */
			delta = scale_exec_time(window_start - mark_start, rq);
			if (!exiting_task(p))
				p->ravg.prev_window += delta;
		} else {
			/* Since at least one full window has elapsed,
			 * the contribution to the previous window is the
			 * full window (window_size). */
			delta = scale_exec_time(window_size, rq);
			if (!exiting_task(p))
				p->ravg.prev_window = delta;
		}
		rq->prev_runnable_sum += delta;
		if (is_new_task(p))
			rq->nt_prev_runnable_sum += delta;

		/* Account piece of busy time in the current window. */
		delta = scale_exec_time(wallclock - window_start, rq);
		rq->curr_runnable_sum += delta;
		if (!exiting_task(p))
			p->ravg.curr_window = delta;
		if (is_new_task(p))
			rq->nt_curr_runnable_sum += delta;

		return;
	}

	if (!irqtime || !is_idle_task(p) || cpu_is_waiting_on_io(rq)) {
		/* account_busy_for_cpu_time() = 1 so busy time needs
		 * to be accounted to the current window. A new window
		 * has started and p is the current task so rollover is
		 * needed. If any of these three above conditions are true
		 * then this busy time can't be accounted as irqtime.
		 *
		 * Busy time for the idle task or exiting tasks need not
		 * be accounted.
		 *
		 * An example of this would be a task that starts execution
		 * and then sleeps once a new window has begun. */

		if (!nr_full_windows) {
			/* A full window hasn't elapsed, account partial
			 * contribution to previous completed window. */
			delta = scale_exec_time(window_start - mark_start, rq);
			if (!is_idle_task(p) && !exiting_task(p))
				p->ravg.prev_window += delta;
			if (is_new_task(p))
				nt_delta = delta;

			delta += rq->curr_runnable_sum;
		} else {
			/* Since at least one full window has elapsed,
			 * the contribution to the previous window is the
			 * full window (window_size). */
			delta = scale_exec_time(window_size, rq);
			if (!is_idle_task(p) && !exiting_task(p))
				p->ravg.prev_window = delta;
			if (is_new_task(p))
				nt_delta = delta;

		}
		/*
		 * Rollover for the normal runnable sum is done here by overwriting
		 * the values in prev_runnable_sum and curr_runnable_sum. The new
		 * task load is rolled over the same way: the load accumulated for
		 * the current window becomes the previous window, plus this task's
		 * own share of the window that just ended.
		 */
		rq->prev_runnable_sum = delta;
		rq->nt_prev_runnable_sum =
				rq->nt_curr_runnable_sum + nt_delta;

		/* Account piece of busy time in the current window. */
		delta = scale_exec_time(wallclock - window_start, rq);
		rq->curr_runnable_sum = delta;
		if (!is_idle_task(p) && !exiting_task(p))
			p->ravg.curr_window = delta;
		rq->nt_curr_runnable_sum = is_new_task(p) ? delta : 0;

		return;
	}

	if (irqtime) {
		/* account_busy_for_cpu_time() = 1 so busy time needs
		 * to be accounted to the current window. A new window
		 * has started and p is the current task so rollover is
		 * needed. The current task must be the idle task because
		 * irqtime is not accounted for any other task.
		 *
		 * Irqtime will be accounted each time we process IRQ activity
		 * after a period of idleness, so we know the IRQ busy time
		 * started at wallclock - irqtime. */

		BUG_ON(!is_idle_task(p));
		mark_start = wallclock - irqtime;

		/* Roll window over. If IRQ busy time was just in the current
		 * window then that is all that need be accounted. */
		rq->prev_runnable_sum = rq->curr_runnable_sum;
		rq->nt_prev_runnable_sum = rq->nt_curr_runnable_sum;
		rq->nt_curr_runnable_sum = 0;
		if (mark_start > window_start) {
			rq->curr_runnable_sum = scale_exec_time(irqtime, rq);
			return;
		}

		/* The IRQ busy time spanned multiple windows. Process the
		 * busy time preceding the current window start first. */
		delta = window_start - mark_start;
		if (delta > window_size)
			delta = window_size;
		delta = scale_exec_time(delta, rq);
		rq->prev_runnable_sum += delta;

		/* Process the remaining IRQ busy time in the current window. */
		delta = wallclock - window_start;
		rq->curr_runnable_sum = scale_exec_time(delta, rq);

		return;
	}

	BUG();
}

static int account_busy_for_task_demand(struct task_struct *p, int event)
{
	/* No need to bother updating task demand for exiting tasks
	 * or the idle task. */
	if (exiting_task(p) || is_idle_task(p))
		return 0;

	/* When a task is waking up it is completing a segment of non-busy
	 * time. Likewise, if wait time is not treated as busy time, then
	 * when a task begins to run or is migrated, it is not running and
	 * is completing a segment of non-busy time. */
	if (event == TASK_WAKE || (!walt_account_wait_time &&
			 (event == PICK_NEXT_TASK || event == TASK_MIGRATE)))
		return 0;

	return 1;
}

/*
 * Called when new window is starting for a task, to record cpu usage over
 * recently concluded window(s). Normally 'samples' should be 1. It can be > 1
 * when, say, a real-time task runs without preemption for several windows at a
 * stretch.
 */
#define INC_STEP		8
#define DEC_STEP		2
#define CONSISTENT_THRES	16
#define INC_STEP_BIG		16

/*
 * bucket_increase - update the count of all buckets
 *
 * @buckets: array of buckets tracking busy time of a task
 * @idx: the index of bucket to be incremented
 *
 * Each time a complete window finishes, count of bucket that runtime
 * falls in (@idx) is incremented. Counts of all other buckets are
 * decayed. The rate of increase and decay could be different based
 * on current count in the bucket.
 */
static inline void bucket_increase(u8 *buckets, int idx)
{
	int i, step;

	for (i = 0; i < NUM_BUSY_BUCKETS; i++) {
		if (idx != i) {
			if (buckets[i] > DEC_STEP)
				buckets[i] -= DEC_STEP;
			else
				buckets[i] = 0;
		} else {
			step = buckets[i] >= CONSISTENT_THRES ?
						INC_STEP_BIG : INC_STEP;
			if (buckets[i] > U8_MAX - step)
				buckets[i] = U8_MAX;
			else
				buckets[i] += step;
		}
	}
}

static inline int busy_to_bucket(u32 normalized_rt)
{
	int bidx;

	bidx = mult_frac(normalized_rt, NUM_BUSY_BUCKETS, walt_ravg_window);
	bidx = min(bidx, NUM_BUSY_BUCKETS - 1);

	/*
	 * Combine lowest two buckets. The lowest frequency falls into
	 * 2nd bucket and thus keep predicting lowest bucket is not
	 * useful.
	 */
	if (!bidx)
		bidx++;

	return bidx;
}

/*
 * get_pred_busy - calculate predicted demand for a task on runqueue
 *
 * @p: task whose prediction is being updated
 * @start: starting bucket. returned prediction should not be lower than
 *         this bucket.
 * @runtime: runtime of the task. returned prediction should not be lower
 *           than this runtime.
 *
 * A new predicted busy time is returned for task @p based on @runtime passed
 * in. The function searches through buckets that represent busy time equal to
 * or bigger than @runtime and attempts to find the bucket to use for
 * prediction. Once found, it searches through historical busy time and returns
 * the latest that falls into the bucket. If no such busy time exists, it
 * returns the medium of that bucket.
 */
static u32 get_pred_busy(struct task_struct *p, int start, u32 runtime)
{
	int i;
	u8 *buckets = p->ravg.busy_buckets;
	u32 *hist = p->ravg.sum_history;
	u32 dmin, dmax;
	int first = NUM_BUSY_BUCKETS, final;
	u32 ret = runtime;

	/* skip prediction for new tasks due to lack of history */
	if (unlikely(is_new_task(p)))
		goto out;

	/* find minimal bucket index to pick */
	for (i = start; i < NUM_BUSY_BUCKETS; i++) {
		if (buckets[i]) {
			first = i;
			break;
		}
	}

	/* if no higher buckets are filled, predict runtime */
	if (first >= NUM_BUSY_BUCKETS)
		goto out;

	/* compute the bucket for prediction */
	final = first;

	/* determine demand range for the predicted bucket */
	if (final < 2) {
		/* lowest two buckets are combined */
		dmin = 0;
		final = 1;
	} else {
		dmin = mult_frac(final, walt_ravg_window, NUM_BUSY_BUCKETS);
	}
	dmax = mult_frac(final + 1, walt_ravg_window, NUM_BUSY_BUCKETS);

	/*
	 * search through runtime history and return first runtime that falls
	 * into the range of predicted bucket.
	 */
	for (i = 0; i < walt_ravg_hist_size; i++) {
		if (hist[i] >= dmin && hist[i] < dmax) {
			ret = hist[i];
			break;
		}
	}

	/* no historical runtime within bucket found, use average of the bin */
	if (ret < dmin)
		ret = (dmin + dmax) / 2;

	/*
	 * when updating in middle of a window, runtime could be higher than
	 * all recorded history. Always predict at least runtime.
	 */
	ret = max(runtime, ret);

out:
	return ret;
}

static inline u32 calc_pred_demand(struct task_struct *p)
{
	if (p->ravg.pred_demand >= p->ravg.curr_window)
		return p->ravg.pred_demand;

	return get_pred_busy(p, busy_to_bucket(p->ravg.curr_window),
			     p->ravg.curr_window);
}

static inline u32 predict_and_update_buckets(struct task_struct *p,
					     u32 runtime)
{
	int bidx = busy_to_bucket(runtime);
	u32 pred_demand = get_pred_busy(p, bidx, runtime);

	bucket_increase(p->ravg.busy_buckets, bidx);

	return pred_demand;
}
static void update_history(struct rq *rq, struct task_struct *p,
			 u32 runtime, int samples, int event)
{
	u32 *hist = &p->ravg.sum_history[0];
	int ridx, widx;
	u32 max = 0, avg, demand, pred_demand;
	u64 sum = 0;

	/* Ignore windows where task had no activity */
	if (!runtime || is_idle_task(p) || exiting_task(p) || !samples)
			goto done;

	/* Push new 'runtime' value onto stack */
	widx = walt_ravg_hist_size - 1;
	ridx = widx - samples;
	for (; ridx >= 0; --widx, --ridx) {
		hist[widx] = hist[ridx];
		sum += hist[widx];
		if (hist[widx] > max)
			max = hist[widx];
	}

	for (widx = 0; widx < samples && widx < walt_ravg_hist_size; widx++) {
		hist[widx] = runtime;
		sum += hist[widx];
		if (hist[widx] > max)
			max = hist[widx];
	}

	p->ravg.sum = 0;

	if (walt_window_stats_policy == WINDOW_STATS_RECENT) {
		demand = runtime;
	} else if (walt_window_stats_policy == WINDOW_STATS_MAX) {
		demand = max;
	} else {
		avg = div64_u64(sum, walt_ravg_hist_size);
		if (walt_window_stats_policy == WINDOW_STATS_AVG)
			demand = avg;
		else
			demand = max(avg, runtime);
	}

	pred_demand = predict_and_update_buckets(p, runtime);

	/*
	 * A throttled deadline sched class task gets dequeued without
	 * changing p->on_rq. Since the dequeue decrements hmp stats
	 * avoid decrementing it here again.
	 *
	 * When window is rolled over, the cumulative window demand
	 * is reset to the cumulative runnable average (contribution from
	 * the tasks on the runqueue). If the current task is dequeued
	 * already, it's demand is not included in the cumulative runnable
	 * average. So add the task demand separately to cumulative window
	 * demand.
	 */
	/*
	 * Keep the rq's predicted demand sum in sync with the new prediction
	 * before p->ravg.pred_demand is updated below.
	 */
	if (task_on_rq_queued(p) &&
	    (!task_has_dl_policy(p) || !p->dl.dl_throttled))
		walt_fixup_pred_demand(rq, p, pred_demand);

	if (!task_has_dl_policy(p) || !p->dl.dl_throttled) {
		if (task_on_rq_queued(p))
			p->sched_class->fixup_cumulative_runnable_avg(rq, p,
								      demand);
		else if (rq->curr == p)
			fixup_cum_window_demand(rq, demand);
	}

	p->ravg.demand = demand;
	p->ravg.pred_demand = pred_demand;

done:
	trace_walt_update_history(rq, p, runtime, samples, event);
	return;
}

/*
 * Predictive demand of a task is calculated at the window roll-over. If the
 * task's busy time in the current window exceeds the prediction, update it
 * here to reflect what the task needs.
 */
static void update_task_pred_demand(struct rq *rq, struct task_struct *p,
				    int event)
{
	u32 new, old;

	if (is_idle_task(p) || exiting_task(p))
		return;

	if (event != PUT_PREV_TASK && event != TASK_UPDATE &&
	    (!walt_freq_account_wait_time ||
	     (event != TASK_MIGRATE && event != PICK_NEXT_TASK)))
		return;

	/*
	 * TASK_UPDATE can be called on a sleeping task, when it is moved
	 * between related groups.
	 */
	if (event == TASK_UPDATE && !p->on_rq && !walt_freq_account_wait_time)
		return;

	new = calc_pred_demand(p);
	old = p->ravg.pred_demand;

	if (old >= new)
		return;

	if (task_on_rq_queued(p) &&
	    (!task_has_dl_policy(p) || !p->dl.dl_throttled))
		walt_fixup_pred_demand(rq, p, new);

	p->ravg.pred_demand = new;
}

static void add_to_task_demand(struct rq *rq, struct task_struct *p,
				u64 delta)
{
	delta = scale_exec_time(delta, rq);
	p->ravg.sum += delta;
	if (unlikely(p->ravg.sum > walt_ravg_window))
		p->ravg.sum = walt_ravg_window;
}

/*
 * Account cpu demand of task and/or update task's cpu demand history
 *
 * ms = p->ravg.mark_start;
 * wc = wallclock
 * ws = rq->window_start
 *
 * Three possibilities:
 *
 *	a) Task event is contained within one window.
 *		window_start < mark_start < wallclock
 *
 *		ws   ms  wc
 *		|    |   |
 *		V    V   V
 *		|---------------|
 *
 *	In this case, p->ravg.sum is updated *iff* event is appropriate
 *	(ex: event == PUT_PREV_TASK)
 *
 *	b) Task event spans two windows.
 *		mark_start < window_start < wallclock
 *
 *		ms   ws   wc
 *		|    |    |
 *		V    V    V
 *		-----|-------------------
 *
 *	In this case, p->ravg.sum is updated with (ws - ms) *iff* event
 *	is appropriate, then a new window sample is recorded followed
 *	by p->ravg.sum being set to (wc - ws) *iff* event is appropriate.
 *
 *	c) Task event spans more than two windows.
 *
 *		ms ws_tmp			   ws  wc
 *		|  |				   |   |
 *		V  V				   V   V
 *		---|-------|-------|-------|-------|------
 *		   |				   |
 *		   |<------ nr_full_windows ------>|
 *
 *	In this case, p->ravg.sum is updated with (ws_tmp - ms) first *iff*
 *	event is appropriate, window sample of p->ravg.sum is recorded,
 *	'nr_full_window' samples of window_size is also recorded *iff*
 *	event is appropriate and finally p->ravg.sum is set to (wc - ws)
 *	*iff* event is appropriate.
 *
 * IMPORTANT : Leave p->ravg.mark_start unchanged, as update_cpu_busy_time()
 * depends on it!
 */
static void update_task_demand(struct task_struct *p, struct rq *rq,
	     int event, u64 wallclock)
{
	u64 mark_start = p->ravg.mark_start;
	u64 delta, window_start = rq->window_start;
	int new_window, nr_full_windows;
	u32 window_size = walt_ravg_window;

	new_window = mark_start < window_start;
	if (!account_busy_for_task_demand(p, event)) {
		if (new_window)
			/* If the time accounted isn't being accounted as
			 * busy time, and a new window started, only the
			 * previous window need be closed out with the
			 * pre-existing demand. Multiple windows may have
			 * elapsed, but since empty windows are dropped,
			 * it is not necessary to account those. */
			update_history(rq, p, p->ravg.sum, 1, event);
		return;
	}

	if (!new_window) {
		/* The simple case - busy time contained within the existing
		 * window. */
		add_to_task_demand(rq, p, wallclock - mark_start);
		return;
	}

	/* Busy time spans at least two windows. Temporarily rewind
	 * window_start to first window boundary after mark_start. */
	delta = window_start - mark_start;
	nr_full_windows = div64_u64(delta, window_size);
	window_start -= (u64)nr_full_windows * (u64)window_size;

	/* Process (window_start - mark_start) first */
	add_to_task_demand(rq, p, window_start - mark_start);

	/* Push new sample(s) into task's demand history */
	update_history(rq, p, p->ravg.sum, 1, event);
	if (nr_full_windows)
		update_history(rq, p, scale_exec_time(window_size, rq),
			       nr_full_windows, event);

	/* Roll window_start back to current to process any remainder
	 * in current window. */
	window_start += (u64)nr_full_windows * (u64)window_size;

	/* Process (wallclock - window_start) next */
	mark_start = window_start;
	add_to_task_demand(rq, p, wallclock - mark_start);
}

/* Reflect task activity on its demand and cpu's busy time statistics */
void walt_update_task_ravg(struct task_struct *p, struct rq *rq,
	     int event, u64 wallclock, u64 irqtime)
{
	u64 window_start;
	unsigned int flags = 0;

	if (walt_disabled || !rq->window_start)
		return;

	/* there's a bug here - there are many cases where
	 * we enter here without holding this lock, coming from
	 * walt_fixup_busy_time - looks like in 4.14 we don't
	 * hold the dest_rq at time of migration, but I haven't
	 * yet worked out if it is safe to always lock dest_rq there.
	 *
	 * temporarily disable this assert to continue checking the
	 * rest of the locking here.
	 */
	//lockdep_assert_held(&rq->lock);

	window_start = rq->window_start;
	update_window_start(rq, wallclock);
	if (rq->window_start != window_start)
		flags |= WALT_CPUFREQ_ROLLOVER;

	if (!p->ravg.mark_start)
		goto done;

	if (event == TASK_MIGRATE)
		flags |= WALT_CPUFREQ_IC_MIGRATION;

	update_task_demand(p, rq, event, wallclock);
	update_cpu_busy_time(p, rq, event, wallclock, irqtime);
	update_task_pred_demand(rq, p, event);

done:
	/*
	 * Let the "walt" cpufreq governor look at the new data. IRQ_UPDATE is
	 * left out because it runs from the IRQ accounting path (i.e. on every
	 * interrupt); the regular events - tick, wakeup, migration and window
	 * rollover - provide enough granularity.
	 */
	if (event != IRQ_UPDATE)
		waltgov_run_callback(rq, flags);

	trace_walt_update_task_ravg(p, rq, event, wallclock, irqtime);

	p->ravg.mark_start = wallclock;
}

static void reset_task_stats(struct task_struct *p)
{
	u32 sum = 0;

	if (exiting_task(p))
		sum = EXITING_TASK_MARKER;

	memset(&p->ravg, 0, sizeof(struct ravg));
	/* Retain EXITING_TASK marker */
	p->ravg.sum_history[0] = sum;
}

void walt_mark_task_starting(struct task_struct *p)
{
	u64 wallclock;
	struct rq *rq = task_rq(p);

	if (!rq->window_start) {
		reset_task_stats(p);
		return;
	}

	wallclock = walt_ktime_clock();
	p->ravg.mark_start = wallclock;
}

void walt_set_window_start(struct rq *rq, struct rq_flags *rf)
{
	if (likely(rq->window_start))
		return;

	if (cpu_of(rq) == sync_cpu) {
		rq->window_start = 1;
	} else {
		struct rq *sync_rq = cpu_rq(sync_cpu);
		rq_unpin_lock(rq, rf);
		double_lock_balance(rq, sync_rq);
		rq->window_start = sync_rq->window_start;
		rq->curr_runnable_sum = rq->prev_runnable_sum = 0;
		rq->nt_curr_runnable_sum = rq->nt_prev_runnable_sum = 0;
		raw_spin_unlock(&sync_rq->lock);
		rq_repin_lock(rq, rf);
	}

	rq->curr->ravg.mark_start = rq->window_start;
}

void walt_migrate_sync_cpu(int cpu)
{
	if (cpu == sync_cpu)
		sync_cpu = smp_processor_id();
}

void walt_fixup_busy_time(struct task_struct *p, int new_cpu)
{
	struct rq *src_rq = task_rq(p);
	struct rq *dest_rq = cpu_rq(new_cpu);
	u64 wallclock;

	if (!p->on_rq && p->state != TASK_WAKING)
		return;

	if (exiting_task(p)) {
		return;
	}

	if (p->state == TASK_WAKING)
		double_rq_lock(src_rq, dest_rq);

	wallclock = walt_ktime_clock();

//#define LOCK_CONDITION(rq) (debug_locks && !lockdep_is_held(&rq->lock))
//	WARN(LOCK_CONDITION(task_rq(p)), "task_rq(p) not held. p->state=%08lx new_cpu=%d task_cpu=%d", p->state, new_cpu, p->cpu);
//	WARN(LOCK_CONDITION(dest_rq), "dest_rq not held. p->state=%08lx new_cpu=%d task_cpu=%d", p->state, new_cpu, p->cpu);

	/*
	 * It seems that in lots of cases we don't have
	 * dest_rq locked when we get here, which means
	 * we can't be sure to the WALT stats - someone
	 * needs to fix this.
	 */
	walt_update_task_ravg(task_rq(p)->curr, task_rq(p),
			TASK_UPDATE, wallclock, 0);
	walt_update_task_ravg(dest_rq->curr, dest_rq,
			TASK_UPDATE, wallclock, 0);

//	WARN(LOCK_CONDITION(task_rq(p)), "task_rq(p) not held after rq update. p->state=%08lx new_cpu=%d task_cpu=%d", p->state, new_cpu, p->cpu);
	walt_update_task_ravg(p, task_rq(p), TASK_MIGRATE, wallclock, 0);

	/*
	 * When a task is migrating during the wakeup, adjust
	 * the task's contribution towards cumulative window
	 * demand.
	 */
	if (p->state == TASK_WAKING &&
	    p->last_sleep_ts >= src_rq->window_start) {
		fixup_cum_window_demand(src_rq, -(s64)p->ravg.demand);
		fixup_cum_window_demand(dest_rq, p->ravg.demand);
	}

	if (p->ravg.curr_window) {
		src_rq->curr_runnable_sum -= p->ravg.curr_window;
		dest_rq->curr_runnable_sum += p->ravg.curr_window;

		if (is_new_task(p)) {
			src_rq->nt_curr_runnable_sum -= p->ravg.curr_window;
			dest_rq->nt_curr_runnable_sum += p->ravg.curr_window;
		}
	}

	if (p->ravg.prev_window) {
		src_rq->prev_runnable_sum -= p->ravg.prev_window;
		dest_rq->prev_runnable_sum += p->ravg.prev_window;

		if (is_new_task(p)) {
			src_rq->nt_prev_runnable_sum -= p->ravg.prev_window;
			dest_rq->nt_prev_runnable_sum += p->ravg.prev_window;
		}
	}

	if ((s64)src_rq->prev_runnable_sum < 0) {
		src_rq->prev_runnable_sum = 0;
		WARN_ON(1);
	}
	if ((s64)src_rq->curr_runnable_sum < 0) {
		src_rq->curr_runnable_sum = 0;
		WARN_ON(1);
	}
	if ((s64)src_rq->nt_prev_runnable_sum < 0)
		src_rq->nt_prev_runnable_sum = 0;
	if ((s64)src_rq->nt_curr_runnable_sum < 0)
		src_rq->nt_curr_runnable_sum = 0;

	trace_walt_migration_update_sum(src_rq, p);
	trace_walt_migration_update_sum(dest_rq, p);

	if (p->state == TASK_WAKING)
		double_rq_unlock(src_rq, dest_rq);
}

void walt_init_new_task_load(struct task_struct *p)
{
	int i;
	u32 init_load_windows =
			div64_u64((u64)sysctl_sched_walt_init_task_load_pct *
                          (u64)walt_ravg_window, 100);
	u32 init_load_pct = current->init_load_pct;

	p->init_load_pct = 0;
	memset(&p->ravg, 0, sizeof(struct ravg));

	if (init_load_pct) {
		init_load_windows = div64_u64((u64)init_load_pct *
			  (u64)walt_ravg_window, 100);
	}

	p->ravg.demand = init_load_windows;
	for (i = 0; i < RAVG_HIST_SIZE_MAX; ++i)
		p->ravg.sum_history[i] = init_load_windows;

	/* A child inherits the related thread group of its parent */
	p->ravg.grp_id = current->ravg.grp_id;
}
