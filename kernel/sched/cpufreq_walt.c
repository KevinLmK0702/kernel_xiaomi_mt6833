/*
 * CPUFreq governor "walt" - port of Qualcomm's WALT cpufreq governor
 * (kernel/sched/walt/cpufreq_walt.c) on top of the MTK schedutil framework
 * used by this kernel.
 *
 * Deviations from the original Qualcomm driver (this kernel's WALT backport
 * does not provide the newer WALT infrastructure):
 *  - no waltgov callbacks; the standard cpufreq update-util hooks are used,
 *  - "rtg_boost_freq" is not provided (no related-thread-group support),
 *  - "new task load" (nl) is unavailable and is always 0,
 *  - "pl" is approximated with WALT's current window accumulated load,
 *  - util -> frequency goes through the platform mapping used by schedutil
 *    (upower tables / mt_cpufreq helpers) instead of a linear formula.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <uapi/linux/sched/types.h>
#include <linux/slab.h>
#include <trace/events/power.h>
#include <trace/events/sched.h>

#include "sched.h"
#include "tune.h"
#include "walt.h"
#include "cpufreq_schedutil.h"

#define CREATE_TRACE_POINTS
#include "walt_trace.h"

static struct cpufreq_governor walt_gov;
unsigned long boosted_cpu_util(int cpu);

/* Hooked by MTK performance drivers, defined in cpufreq_schedutil.c */
extern void (*cpufreq_notifier_fp)(int cluster_id, unsigned long freq);

#define SUGOV_KTHREAD_PRIORITY	50

struct sugov_tunables {
	struct gov_attr_set attr_set;
	unsigned int up_rate_limit_us;
	unsigned int down_rate_limit_us;

	/* Tunables ported from Qualcomm's walt governor */
	unsigned int hispeed_load;		/* %% of average capacity */
	unsigned int hispeed_freq;		/* kHz */
	unsigned int adaptive_low_freq;		/* kHz */
	unsigned int adaptive_high_freq;	/* kHz */
	unsigned int target_load_thresh;	/* capacity scale */
	unsigned int target_load_shift;
	bool pl;
	int boost;
};

struct sugov_policy {
	struct cpufreq_policy *policy;

	struct sugov_tunables *tunables;
	struct list_head tunables_hook;

	raw_spinlock_t update_lock;  /* For shared policies */
	u64 last_freq_update_time;
	s64 min_rate_limit_ns;
	s64 up_rate_delay_ns;
	s64 down_rate_delay_ns;
	unsigned int next_freq;
	unsigned int cached_raw_freq;

	/* WALT: average capacity tracking (from Qualcomm's waltgov) */
	u64 last_ws;
	u64 curr_cycles;
	u64 last_cyc_update_time;
	unsigned long avg_cap;

	/* Last decision, exposed through the read-only walt/decision node */
	u64 last_time;
	u64 update_count;
	u64 hispeed_hits;
	u64 pl_hits;
	unsigned long last_util;
	unsigned long last_max;
	unsigned long last_avg_cap;
	unsigned long last_pl;
	unsigned int last_raw_freq;
	unsigned int last_freq;
	unsigned int last_cpu;
	unsigned int last_flags;

	/* The next fields are only needed if fast switch cannot be used. */
	struct irq_work irq_work;
	struct kthread_work work;
	struct mutex work_lock;
	struct kthread_worker worker;
	struct task_struct *thread;
	bool work_in_progress;

	bool need_freq_update;
};

struct sugov_cpu {
	struct update_util_data update_util;
	struct sugov_policy *sg_policy;
	unsigned int cpu;

	bool iowait_boost_pending;
	unsigned int iowait_boost;
	unsigned int iowait_boost_max;
	u64 last_update;

	/* The fields below are only needed when sharing a policy. */
	unsigned long util;
	unsigned long max;
	unsigned int flags;
	unsigned long min_boost;

	/* WALT load info collected by cpu_util_freq_walt() */
	struct walt_cpu_load walt_load;

	/* The field below is for single-CPU policies only. */
#ifdef CONFIG_NO_HZ_COMMON
	unsigned long saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct sugov_cpu, sugov_cpu);

/************************ Governor internals ***********************/

static bool sugov_should_update_freq(struct sugov_policy *sg_policy, u64 time)
{
	s64 delta_ns;

	struct cpufreq_policy *policy = sg_policy->policy;

	if (policy->governor != &walt_gov ||
		!policy->governor_data)
		return false;

	/*
	 * Since cpufreq_update_util() is called with rq->lock held for
	 * the @target_cpu, our per-cpu data is fully serialized.
	 *
	 * However, drivers cannot in general deal with cross-cpu
	 * requests, so while get_next_freq() will work, our
	 * sugov_update_commit() call may not for the fast switching platforms.
	 *
	 * Hence stop here for remote requests if they aren't supported
	 * by the hardware, as calculating the frequency is pointless if
	 * we cannot in fact act on it.
	 *
	 * For the slow switching platforms, the kthread is always scheduled on
	 * the right set of CPUs and any CPU can find the next frequency and
	 * schedule the kthread.
	 */
	if (sg_policy->policy->fast_switch_enabled &&
	    !cpufreq_can_do_remote_dvfs(sg_policy->policy))
		return false;

	if (sg_policy->work_in_progress)
		return false;

	if (unlikely(sg_policy->need_freq_update)) {
		sg_policy->need_freq_update = false;
		/*
		 * This happens when limits change, so forget the previous
		 * next_freq value and force an update.
		 */
		sg_policy->next_freq = UINT_MAX;
		return true;
	}

	/* No need to recalculate next freq for min_rate_limit_us
	 * at least. However we might still decide to further rate
	 * limit once frequency change direction is decided, according
	 * to the separate rate limits.
	 */

	delta_ns = time - sg_policy->last_freq_update_time;
	return delta_ns >= sg_policy->min_rate_limit_ns;
}

static bool sugov_up_down_rate_limit(struct sugov_policy *sg_policy, u64 time,
				     unsigned int next_freq)
{
	s64 delta_ns;

	delta_ns = time - sg_policy->last_freq_update_time;

	if (next_freq > sg_policy->next_freq &&
	    delta_ns < sg_policy->up_rate_delay_ns)
			return true;

	if (next_freq < sg_policy->next_freq &&
	    delta_ns < sg_policy->down_rate_delay_ns)
			return true;

	return false;
}

static void sugov_update_commit(struct sugov_policy *sg_policy, u64 time,
				unsigned int next_freq)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	int cid = arch_get_cluster_id(policy->cpu);

	if (sg_policy->next_freq == next_freq)
		return;

	if (sugov_up_down_rate_limit(sg_policy, time, next_freq))
		return;

	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = time;

	if (cpufreq_notifier_fp)
		cpufreq_notifier_fp(cid, next_freq);

#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	mt_cpufreq_set_by_wfi_load_cluster(cid, next_freq);
	policy->cur = next_freq;
	trace_sched_util(cid, next_freq, time);
#else
	if (policy->fast_switch_enabled) {
		next_freq = cpufreq_driver_fast_switch(policy, next_freq);
		if (!next_freq)
			return;

		policy->cur = next_freq;
		trace_cpu_frequency(next_freq, smp_processor_id());
	} else {
		sg_policy->work_in_progress = true;
		irq_work_queue(&sg_policy->irq_work);
	}
#endif
}

#ifdef CONFIG_NONLINEAR_FREQ_CTL

#include "cpufreq_schedutil_plus.c"
#else
/**
 * get_next_freq - Compute a new frequency for a given cpufreq policy.
 * @sg_policy: schedutil policy object to compute the new frequency for.
 * @util: Current CPU utilization.
 * @max: CPU capacity.
 *
 * If the utilization is frequency-invariant, choose the new frequency to be
 * proportional to it, that is
 *
 * next_freq = C * max_freq * util / max
 *
 * Otherwise, approximate the would-be frequency-invariant utilization by
 * util_raw * (curr_freq / max_freq) which leads to
 *
 * next_freq = C * curr_freq * util_raw / max
 *
 * Take C = 1.25 for the frequency tipping point at (util / max) = 0.8.
 *
 * The lowest driver-supported frequency which is equal or greater than the raw
 * next_freq (as calculated above) is returned, subject to policy min/max and
 * cpufreq driver limitations.
 */
static unsigned int get_next_freq(struct sugov_policy *sg_policy,
				  unsigned long util, unsigned long max)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int freq = arch_scale_freq_invariant() ?
				policy->cpuinfo.max_freq : policy->cur;

	freq = freq * util / max;
	freq = freq / SCHED_CAPACITY_SCALE * capacity_margin;

	sg_policy->cached_raw_freq = freq;
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	return freq;
#else
	return cpufreq_driver_resolve_freq(policy, freq);
#endif
}
#endif

/*
 * WALT governor tunables and logic, ported from Qualcomm's
 * kernel/sched/walt/cpufreq_walt.c.
 */
#define WALT_TARGET_LOAD		80
#define WALT_KHZ			1000
#define WALT_DEFAULT_HISPEED_LOAD	90
#define WALT_DEFAULT_TARGET_LOAD_THRESH	1024
#define WALT_DEFAULT_TARGET_LOAD_SHIFT	4

static unsigned long walt_freq_to_util(struct sugov_policy *sg_policy,
				       unsigned int freq)
{
	unsigned long max_cap = arch_scale_cpu_capacity(NULL,
							sg_policy->policy->cpu);

	return mult_frac(max_cap, freq, sg_policy->policy->cpuinfo.max_freq);
}

static unsigned long walt_target_util(struct sugov_policy *sg_policy,
				      unsigned int freq)
{
	unsigned long util = walt_freq_to_util(sg_policy, freq);

	return mult_frac(util, WALT_TARGET_LOAD, 100);
}

static void walt_track_cycles(struct sugov_policy *sg_policy,
			      unsigned int prev_freq, u64 upto)
{
	u64 delta_ns, cycles;
	u64 next_ws = sg_policy->last_ws + walt_ravg_window;

	upto = min(upto, next_ws);
	/* Track cycles in current window */
	delta_ns = upto - sg_policy->last_cyc_update_time;
	delta_ns *= prev_freq;
	do_div(delta_ns, (NSEC_PER_SEC / WALT_KHZ));
	cycles = delta_ns;
	sg_policy->curr_cycles += cycles;
	sg_policy->last_cyc_update_time = upto;
}

static void walt_calc_avg_cap(struct sugov_policy *sg_policy, u64 curr_ws,
			      unsigned int prev_freq)
{
	u64 last_ws = sg_policy->last_ws;
	unsigned int avg_freq;

	/* The WALT window may have been reset (suspend, window resize) */
	if (curr_ws <= last_ws) {
		sg_policy->last_ws = curr_ws;
		sg_policy->curr_cycles = 0;
		sg_policy->last_cyc_update_time = curr_ws;
		return;
	}

	/* If we skipped some windows */
	if (curr_ws > (last_ws + walt_ravg_window)) {
		avg_freq = prev_freq;
		/* Reset tracking history */
		sg_policy->last_cyc_update_time = curr_ws;
	} else {
		walt_track_cycles(sg_policy, prev_freq, curr_ws);
		avg_freq = sg_policy->curr_cycles;
		avg_freq /= walt_ravg_window / (NSEC_PER_SEC / WALT_KHZ);
	}
	sg_policy->avg_cap = walt_freq_to_util(sg_policy, avg_freq);
	sg_policy->curr_cycles = 0;
	sg_policy->last_ws = curr_ws;
}

/*
 * Mirrors waltgov_walt_adjust(). The original RTG boost and new task load
 * (nl) inputs are not available in this kernel's WALT, so only the hispeed
 * and load prediction (pl) parts are applied here.
 */
static void walt_adjust_util(struct sugov_cpu *sg_cpu, unsigned long cpu_util,
			     unsigned long *util, unsigned long *max)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct sugov_tunables *tunables = sg_policy->tunables;
	bool is_hiload;

	is_hiload = (cpu_util >= mult_frac(sg_policy->avg_cap,
					   tunables->hispeed_load, 100));

	if (is_hiload && tunables->hispeed_freq) {
		unsigned long hs = walt_target_util(sg_policy,
						    tunables->hispeed_freq);

		if (hs > *util)
			sg_policy->hispeed_hits++;
		*util = max(*util, hs);
	}

	if (tunables->pl && sg_cpu->walt_load.pl) {
		if (sg_cpu->walt_load.pl > *util)
			sg_policy->pl_hits++;
		*util = max(*util, sg_cpu->walt_load.pl);
	}
}

/*
 * Save the last decision and emit it through the tracepoint, so that the
 * governor can be tuned from data instead of guesses.
 */
static void waltgov_trace_decision(struct sugov_policy *sg_policy, int cpu,
				   unsigned long util, unsigned long max,
				   unsigned int freq, unsigned int flags)
{
	struct sugov_tunables *tunables = sg_policy->tunables;
	unsigned long pl = per_cpu(sugov_cpu, cpu).walt_load.pl;
	bool hiload = (util >= mult_frac(sg_policy->avg_cap,
					 tunables->hispeed_load, 100));

	sg_policy->last_time = walt_ktime_clock();
	sg_policy->last_cpu = cpu;
	sg_policy->last_util = util;
	sg_policy->last_max = max;
	sg_policy->last_avg_cap = sg_policy->avg_cap;
	sg_policy->last_pl = pl;
	sg_policy->last_raw_freq = sg_policy->cached_raw_freq;
	sg_policy->last_freq = freq;
	sg_policy->last_flags = flags;
	sg_policy->update_count++;

	trace_waltgov_next_freq(cpu, util, max, sg_policy->cached_raw_freq,
				freq, sg_policy->avg_cap, pl, hiload,
				tunables->boost, flags);
}

/*
 * Qualcomm maps frequencies as
 *	freq ~ max((1 + 2^-shift) * util, 1.25 * thresh) * fmax / cap
 * for util >= target_load_thresh, i.e. the 25%% headroom of the platform
 * mapping is reduced above the threshold. The platform mapping adds a
 * 1.25 factor itself, so pre-scale the utilization accordingly.
 */
static unsigned long walt_apply_target_load(struct sugov_tunables *tunables,
					    unsigned long util)
{
	unsigned long thresh = tunables->target_load_thresh;
	unsigned long scaled;

	if (!thresh || util < thresh)
		return util;

	scaled = mult_frac(util, 100 + (100 >> tunables->target_load_shift),
			   125);

	return max(scaled, thresh);
}

static unsigned int walt_get_next_freq(struct sugov_policy *sg_policy,
				       unsigned long util, unsigned long max)
{
	util = walt_apply_target_load(sg_policy->tunables, util);

	return get_next_freq(sg_policy, util, max);
}

/*
 * Adaptive frequency band of the original governor: hold the frequency at
 * adaptive_high_freq for anything in between adaptive_low_freq and
 * adaptive_high_freq, and never go below adaptive_low_freq while set.
 */
static unsigned int walt_apply_adaptive_freq(struct cpufreq_policy *policy,
					     unsigned int freq)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	struct sugov_tunables *tunables = sg_policy->tunables;

	if (tunables->adaptive_high_freq) {
		if (freq < tunables->adaptive_low_freq)
			freq = tunables->adaptive_low_freq;
		else if (freq <= tunables->adaptive_high_freq)
			freq = tunables->adaptive_high_freq;
	}

	return cpufreq_driver_resolve_freq(policy, freq);
}

static void sugov_get_util(struct sugov_cpu *sg_cpu, unsigned long *util,
			   unsigned long *max)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned long max_cap;

	max_cap = arch_scale_cpu_capacity(NULL, sg_cpu->cpu);

	if (likely(!walt_disabled && sysctl_sched_use_walt_cpu_util))
		*util = cpu_util_freq_walt(sg_cpu->cpu, &sg_cpu->walt_load);
	else
		*util = boosted_cpu_util(sg_cpu->cpu);

	if (idle_cpu(sg_cpu->cpu))
		*util = 0;

	*util = min(*util, max_cap);
	*util = uclamp_util(cpu_rq(sg_cpu->cpu), *util);

	/* Qualcomm waltgov "boost" tunable (percent, may be negative) */
	if (sg_policy->tunables->boost)
		*util = mult_frac(*util, 100 + sg_policy->tunables->boost, 100);

	*max = max_cap;
}

static void sugov_set_iowait_boost(struct sugov_cpu *sg_cpu, u64 time,
				   unsigned int flags)
{
	unsigned int max_boost;

	if (flags & SCHED_CPUFREQ_IOWAIT) {
		if (sg_cpu->iowait_boost_pending)
			return;

		sg_cpu->iowait_boost_pending = true;

		/*
		 * Boost FAIR tasks only up to the CPU clamped utilization.
		 *
		 * Since DL tasks have a much more advanced bandwidth control,
		 * it's safe to assume that IO boost does not apply to
		 * those tasks.
		 * Instead, since RT tasks are currently not utiliation clamped,
		 * we don't want to apply clamping on IO boost while there is
		 * blocked RT utilization.
		 */
		max_boost = sg_cpu->iowait_boost_max;
		max_boost = uclamp_util(cpu_rq(sg_cpu->cpu), max_boost);

		if (sg_cpu->iowait_boost) {
			sg_cpu->iowait_boost <<= 1;
			if (sg_cpu->iowait_boost > max_boost)
				sg_cpu->iowait_boost = max_boost;
		} else {
			sg_cpu->iowait_boost = sg_cpu->min_boost;
		}
	} else if (sg_cpu->iowait_boost) {
		s64 delta_ns = time - sg_cpu->last_update;

		/* Clear iowait_boost if the CPU apprears to have been idle. */
		if (delta_ns > TICK_NSEC) {
			sg_cpu->iowait_boost = 0;
			sg_cpu->iowait_boost_pending = false;
		}
	}
}

static void sugov_iowait_boost(struct sugov_cpu *sg_cpu, unsigned long *util,
			       unsigned long *max)
{
	unsigned int boost_util, boost_max;

	if (!sg_cpu->iowait_boost)
		return;

	if (sg_cpu->iowait_boost_pending) {
		sg_cpu->iowait_boost_pending = false;
	} else {
		sg_cpu->iowait_boost >>= 1;
		if (sg_cpu->iowait_boost < sg_cpu->min_boost) {
			sg_cpu->iowait_boost = 0;
			return;
		}
	}

	boost_util = sg_cpu->iowait_boost;
	boost_max = sg_cpu->iowait_boost_max;

	if (*util * boost_max < *max * boost_util) {
		*util = boost_util;
		*max = boost_max;
	}
}

#ifdef CONFIG_NO_HZ_COMMON
static bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu)
{
	unsigned long idle_calls = tick_nohz_get_idle_calls_cpu(sg_cpu->cpu);
	bool ret = idle_calls == sg_cpu->saved_idle_calls;

	sg_cpu->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu) { return false; }
#endif /* CONFIG_NO_HZ_COMMON */

static void sugov_update_single(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util, max;
	unsigned int next_f;
	bool busy;
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	int cid;
#endif

	sugov_set_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	if (!sugov_should_update_freq(sg_policy, time))
		return;

	busy = sugov_cpu_is_busy(sg_cpu);

	if (flags & SCHED_CPUFREQ_DL) {
		next_f = policy->cpuinfo.max_freq;
	} else {
		sugov_get_util(sg_cpu, &util, &max);
		walt_calc_avg_cap(sg_policy, sg_cpu->walt_load.ws,
				  policy->cur);
		walt_adjust_util(sg_cpu, util, &util, &max);
		sugov_iowait_boost(sg_cpu, &util, &max);
		next_f = walt_get_next_freq(sg_policy, util, max);
		next_f = walt_apply_adaptive_freq(policy, next_f);
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
		next_f = clamp_val(next_f, policy->min, policy->max);
		cid = arch_get_cluster_id(sg_policy->policy->cpu);
		next_f = mt_cpufreq_find_close_freq(cid, next_f);
#endif
		waltgov_trace_decision(sg_policy, sg_cpu->cpu, util, max,
				       next_f, flags);
		/*
		 * Do not reduce the frequency if the CPU has not been idle
		 * recently, as the reduction is likely to be premature then.
		 */
		if (busy && next_f < sg_policy->next_freq &&
		    sg_policy->next_freq != UINT_MAX) {
			next_f = sg_policy->next_freq;

			/* Reset cached freq as next_freq has changed */
			sg_policy->cached_raw_freq = 0;
		}
	}

	sugov_update_commit(sg_policy, time, next_f);
}

static unsigned int sugov_next_freq_shared(struct sugov_cpu *sg_cpu, u64 time)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util = 0, max = 1;
	unsigned int j;
	unsigned int next_f;
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	int cid;
#endif

	for_each_cpu(j, policy->cpus) {
		struct sugov_cpu *j_sg_cpu = &per_cpu(sugov_cpu, j);
		unsigned long j_util, j_max;
		s64 delta_ns;

		/*
		 * If the CPU utilization was last updated before the previous
		 * frequency update and the time elapsed between the last update
		 * of the CPU utilization and the last frequency update is long
		 * enough, don't take the CPU into account as it probably is
		 * idle now (and clear iowait_boost for it).
		 */
		delta_ns = time - j_sg_cpu->last_update;
		if (delta_ns > TICK_NSEC) {
			j_sg_cpu->iowait_boost = 0;
			j_sg_cpu->iowait_boost_pending = false;
			if (idle_cpu(j))
				continue;
		}
		if (j_sg_cpu->flags & SCHED_CPUFREQ_DL)
			return policy->cpuinfo.max_freq;

		j_util = j_sg_cpu->util;
		j_max = j_sg_cpu->max;

		if (j_util * max > j_max * util) {
			util = j_util;
			max = j_max;
		}

		walt_adjust_util(j_sg_cpu, j_util, &util, &max);

		sugov_iowait_boost(j_sg_cpu, &util, &max);
	}

	next_f = walt_get_next_freq(sg_policy, util, max);
	next_f = walt_apply_adaptive_freq(policy, next_f);

#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	next_f = clamp_val(next_f, policy->min, policy->max);
	cid = arch_get_cluster_id(sg_policy->policy->cpu);
	next_f = mt_cpufreq_find_close_freq(cid, next_f);
#endif
	waltgov_trace_decision(sg_policy, sg_cpu->cpu, util, max, next_f,
			       sg_cpu->flags);
	return next_f;
}

static void sugov_update_shared(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned long util, max;
	unsigned int next_f;

	sugov_get_util(sg_cpu, &util, &max);

	raw_spin_lock(&sg_policy->update_lock);

	sg_cpu->util = util;
	sg_cpu->max = max;
	sg_cpu->flags = flags;

	sugov_set_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	if (sugov_should_update_freq(sg_policy, time)) {
		walt_calc_avg_cap(sg_policy, sg_cpu->walt_load.ws,
				  sg_policy->policy->cur);
		if (flags & SCHED_CPUFREQ_DL)
			next_f = sg_policy->policy->cpuinfo.max_freq;
		else
			next_f = sugov_next_freq_shared(sg_cpu, time);


		sugov_update_commit(sg_policy, time, next_f);
	}

	raw_spin_unlock(&sg_policy->update_lock);
}

static void sugov_work(struct kthread_work *work)
{
	struct sugov_policy *sg_policy = container_of(work, struct sugov_policy, work);

	mutex_lock(&sg_policy->work_lock);
	__cpufreq_driver_target(sg_policy->policy, sg_policy->next_freq,
				CPUFREQ_RELATION_L);
	mutex_unlock(&sg_policy->work_lock);

	sg_policy->work_in_progress = false;
}

static void sugov_irq_work(struct irq_work *irq_work)
{
	struct sugov_policy *sg_policy;

	sg_policy = container_of(irq_work, struct sugov_policy, irq_work);

	/*
	 * For RT and deadline tasks, the schedutil governor shoots the
	 * frequency to maximum. Special care must be taken to ensure that this
	 * kthread doesn't result in the same behavior.
	 *
	 * This is (mostly) guaranteed by the work_in_progress flag. The flag is
	 * updated only at the end of the sugov_work() function and before that
	 * the schedutil governor rejects all other frequency scaling requests.
	 *
	 * There is a very rare case though, where the RT thread yields right
	 * after the work_in_progress flag is cleared. The effects of that are
	 * neglected for now.
	 */
	kthread_queue_work(&sg_policy->worker, &sg_policy->work);
}

/************************** sysfs interface ************************/

static struct sugov_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);

static inline struct sugov_tunables *to_sugov_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct sugov_tunables, attr_set);
}

static DEFINE_MUTEX(min_rate_lock);

static void update_min_rate_limit_ns(struct sugov_policy *sg_policy)
{
	mutex_lock(&min_rate_lock);
	sg_policy->min_rate_limit_ns = min(sg_policy->up_rate_delay_ns,
					   sg_policy->down_rate_delay_ns);
	mutex_unlock(&min_rate_lock);
}

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->up_rate_limit_us);
}

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->down_rate_limit_us);
}

static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->up_rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->up_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_ns(sg_policy);
	}

	return count;
}

static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->down_rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->down_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_ns(sg_policy);
	}

	return count;
}

int walt_set_down_rate_limit_us(int cpu, unsigned int rate_limit_us)
{
	struct cpufreq_policy *policy;
	struct sugov_policy *sg_policy;
	struct sugov_tunables *tunables;
	struct gov_attr_set *attr_set;

	policy = cpufreq_cpu_get(cpu);
	if (!policy)
		return -EINVAL;

	if (policy->governor != &walt_gov)
		return -ENOENT;

	mutex_lock(&global_tunables_lock);
	sg_policy = policy->governor_data;
	if (!sg_policy) {
		mutex_unlock(&global_tunables_lock);
		cpufreq_cpu_put(policy);
		return -EINVAL;
	}

	tunables = sg_policy->tunables;
	tunables->down_rate_limit_us = rate_limit_us;
	attr_set = &tunables->attr_set;

	mutex_lock(&attr_set->update_lock);
	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->down_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_ns(sg_policy);
	}
	mutex_unlock(&attr_set->update_lock);
	mutex_unlock(&global_tunables_lock);

	if (policy)
		cpufreq_cpu_put(policy);
	return 0;
}
EXPORT_SYMBOL(walt_set_down_rate_limit_us);

int walt_set_up_rate_limit_us(int cpu, unsigned int rate_limit_us)
{
	struct cpufreq_policy *policy;
	struct sugov_policy *sg_policy;
	struct sugov_tunables *tunables;
	struct gov_attr_set *attr_set;

	policy = cpufreq_cpu_get(cpu);
	if (!policy)
		return -EINVAL;

	if (policy->governor != &walt_gov)
		return -ENOENT;

	mutex_lock(&global_tunables_lock);
	sg_policy = policy->governor_data;
	if (!sg_policy) {
		mutex_unlock(&global_tunables_lock);
		cpufreq_cpu_put(policy);
		return -EINVAL;
	}

	tunables = sg_policy->tunables;
	tunables->up_rate_limit_us = rate_limit_us;
	attr_set = &tunables->attr_set;

	mutex_lock(&attr_set->update_lock);
	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->up_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_ns(sg_policy);
	}
	mutex_unlock(&attr_set->update_lock);
	mutex_unlock(&global_tunables_lock);

	if (policy)
		cpufreq_cpu_put(policy);
	return 0;
}
EXPORT_SYMBOL(walt_set_up_rate_limit_us);

static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);

#define WALTGOV_SHOW(_name)						\
static ssize_t _name##_show(struct gov_attr_set *attr_set, char *buf)	\
{									\
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);	\
									\
	return sprintf(buf, "%u\n", tunables->_name);			\
}

#define WALTGOV_STORE(_name)						\
static ssize_t _name##_store(struct gov_attr_set *attr_set,		\
			     const char *buf, size_t count)		\
{									\
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);	\
	unsigned int val;						\
									\
	if (kstrtouint(buf, 10, &val))					\
		return -EINVAL;						\
	tunables->_name = val;						\
	return count;							\
}

WALTGOV_SHOW(hispeed_load);
WALTGOV_STORE(hispeed_load);
WALTGOV_SHOW(hispeed_freq);
WALTGOV_STORE(hispeed_freq);
WALTGOV_SHOW(adaptive_low_freq);
WALTGOV_STORE(adaptive_low_freq);
WALTGOV_SHOW(adaptive_high_freq);
WALTGOV_STORE(adaptive_high_freq);
WALTGOV_SHOW(target_load_thresh);
WALTGOV_STORE(target_load_thresh);
WALTGOV_SHOW(target_load_shift);
WALTGOV_STORE(target_load_shift);
#undef WALTGOV_SHOW
#undef WALTGOV_STORE

static ssize_t pl_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sprintf(buf, "%d\n", tunables->pl);
}

static ssize_t pl_store(struct gov_attr_set *attr_set, const char *buf,
			size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;
	tunables->pl = val;

	return count;
}

static ssize_t boost_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sprintf(buf, "%d\n", tunables->boost);
}

static ssize_t boost_store(struct gov_attr_set *attr_set, const char *buf,
			   size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	if (val < -100 || val > 1000)
		return -EINVAL;
	tunables->boost = val;

	return count;
}

static struct governor_attr hispeed_load = __ATTR_RW(hispeed_load);
static struct governor_attr hispeed_freq = __ATTR_RW(hispeed_freq);
static struct governor_attr adaptive_low_freq = __ATTR_RW(adaptive_low_freq);
static struct governor_attr adaptive_high_freq = __ATTR_RW(adaptive_high_freq);
static struct governor_attr target_load_thresh = __ATTR_RW(target_load_thresh);
static struct governor_attr target_load_shift = __ATTR_RW(target_load_shift);
static struct governor_attr pl = __ATTR_RW(pl);
static struct governor_attr boost = __ATTR_RW(boost);

static ssize_t decision_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	int len = 0;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		len += sprintf(buf + len,
			"policy%u cpu=%u util=%lu max=%lu avg_cap=%lu pl=%lu raw_freq=%u freq=%u flags=0x%x updates=%llu hispeed_hits=%llu pl_hits=%llu\n",
				sg_policy->policy->cpu, sg_policy->last_cpu,
				sg_policy->last_util, sg_policy->last_max,
				sg_policy->last_avg_cap, sg_policy->last_pl,
				sg_policy->last_raw_freq, sg_policy->last_freq,
				sg_policy->last_flags,
				(unsigned long long)sg_policy->update_count,
				(unsigned long long)sg_policy->hispeed_hits,
				(unsigned long long)sg_policy->pl_hits);
	}

	return len;
}

static struct governor_attr decision = {
	.attr = { .name = "decision", .mode = 0444 },
	.show = decision_show,
};

static struct attribute *sugov_attributes[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&hispeed_load.attr,
	&hispeed_freq.attr,
	&adaptive_low_freq.attr,
	&adaptive_high_freq.attr,
	&target_load_thresh.attr,
	&target_load_shift.attr,
	&pl.attr,
	&boost.attr,
	&decision.attr,
	NULL
};

static void sugov_tunables_free(struct kobject *kobj)
{
	struct gov_attr_set *attr_set = container_of(kobj, struct gov_attr_set, kobj);

	kfree(to_sugov_tunables(attr_set));
}

static struct kobj_type sugov_tunables_ktype = {
	.default_attrs = sugov_attributes,
	.sysfs_ops = &governor_sysfs_ops,
	.release = &sugov_tunables_free,
};

/********************** cpufreq governor interface *********************/


static struct sugov_policy *sugov_policy_alloc(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;

	sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
	if (!sg_policy)
		return NULL;

	sg_policy->policy = policy;
	raw_spin_lock_init(&sg_policy->update_lock);
	return sg_policy;
}

static void sugov_policy_free(struct sugov_policy *sg_policy)
{
	kfree(sg_policy);
}

static int sugov_kthread_create(struct sugov_policy *sg_policy)
{
	struct task_struct *thread;
	struct sched_param param = { .sched_priority = MAX_USER_RT_PRIO / 2 };
	struct cpufreq_policy *policy = sg_policy->policy;
	int ret;

	/* kthread only required for slow path */
	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&sg_policy->work, sugov_work);
	kthread_init_worker(&sg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
				"sugov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create sugov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		return ret;
	}

	sg_policy->thread = thread;

	/* Kthread is bound to all CPUs by default */
	if (!policy->dvfs_possible_from_any_cpu)
		kthread_bind_mask(thread, policy->related_cpus);

	init_irq_work(&sg_policy->irq_work, sugov_irq_work);
	mutex_init(&sg_policy->work_lock);

	wake_up_process(thread);

	return 0;
}

static void sugov_kthread_stop(struct sugov_policy *sg_policy)
{
	/* kthread only required for slow path */
	if (sg_policy->policy->fast_switch_enabled)
		return;

	kthread_flush_worker(&sg_policy->worker);
	kthread_stop(sg_policy->thread);
	mutex_destroy(&sg_policy->work_lock);
}

static struct sugov_tunables *sugov_tunables_alloc(struct sugov_policy *sg_policy)
{
	struct sugov_tunables *tunables;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);
		if (!have_governor_per_policy())
			global_tunables = tunables;
	}
	return tunables;
}

static void sugov_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		global_tunables = NULL;
}

static int sugov_init(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;
	struct sugov_tunables *tunables;
	int ret = 0;

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	sg_policy = sugov_policy_alloc(policy);
	if (!sg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = sugov_kthread_create(sg_policy);
	if (ret)
		goto free_sg_policy;

	mutex_lock(&global_tunables_lock);

	if (global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = sg_policy;
		sg_policy->tunables = global_tunables;

		gov_attr_set_get(&global_tunables->attr_set, &sg_policy->tunables_hook);
		goto out;
	}

	tunables = sugov_tunables_alloc(sg_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	tunables->up_rate_limit_us = cpufreq_policy_transition_delay_us(policy);
	tunables->down_rate_limit_us = cpufreq_policy_transition_delay_us(policy);

	tunables->hispeed_load = WALT_DEFAULT_HISPEED_LOAD;
	tunables->hispeed_freq = 0;
	tunables->adaptive_low_freq = 0;
	tunables->adaptive_high_freq = 0;
	tunables->target_load_thresh = WALT_DEFAULT_TARGET_LOAD_THRESH;
	tunables->target_load_shift = WALT_DEFAULT_TARGET_LOAD_SHIFT;
	tunables->pl = false;
	tunables->boost = 0;

	policy->governor_data = sg_policy;
	sg_policy->tunables = tunables;

	ret = kobject_init_and_add(&tunables->attr_set.kobj, &sugov_tunables_ktype,
				   get_governor_parent_kobj(policy), "%s",
				   walt_gov.name);
	if (ret)
		goto fail;

out:
	mutex_unlock(&global_tunables_lock);
	return 0;

fail:
	kobject_put(&tunables->attr_set.kobj);
	policy->governor_data = NULL;
	sugov_clear_global_tunables();

stop_kthread:
	sugov_kthread_stop(sg_policy);
	mutex_unlock(&global_tunables_lock);

free_sg_policy:
	sugov_policy_free(sg_policy);

disable_fast_switch:
	cpufreq_disable_fast_switch(policy);

	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static void sugov_exit(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	struct sugov_tunables *tunables = sg_policy->tunables;
	unsigned int count;

	mutex_lock(&global_tunables_lock);

	count = gov_attr_set_put(&tunables->attr_set, &sg_policy->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		sugov_clear_global_tunables();

	mutex_unlock(&global_tunables_lock);

	sugov_kthread_stop(sg_policy);
	sugov_policy_free(sg_policy);
	cpufreq_disable_fast_switch(policy);
}

static int sugov_start(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	sg_policy->up_rate_delay_ns =
		sg_policy->tunables->up_rate_limit_us * NSEC_PER_USEC;
	sg_policy->down_rate_delay_ns =
		sg_policy->tunables->down_rate_limit_us * NSEC_PER_USEC;
	update_min_rate_limit_ns(sg_policy);
	sg_policy->last_freq_update_time = 0;
	sg_policy->next_freq = UINT_MAX;
	sg_policy->work_in_progress = false;
	sg_policy->need_freq_update = false;
	sg_policy->cached_raw_freq = 0;

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->cpu = cpu;
		sg_cpu->sg_policy = sg_policy;
		sg_cpu->flags = SCHED_CPUFREQ_DL;
		sg_cpu->iowait_boost_max = capacity_orig_of(cpu);
		sg_cpu->min_boost =
			(SCHED_CAPACITY_SCALE * policy->cpuinfo.min_freq) /
			policy->cpuinfo.max_freq;
	}

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util,
					     policy_is_shared(policy) ?
							sugov_update_shared :
							sugov_update_single);
	}
	return 0;
}

static void sugov_stop(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_sched();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&sg_policy->irq_work);
		kthread_cancel_work_sync(&sg_policy->work);
	}
}

static void sugov_limits(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&sg_policy->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&sg_policy->work_lock);
	}

	sg_policy->need_freq_update = true;
}

static struct cpufreq_governor walt_gov = {
	.name = "walt",
	.owner = THIS_MODULE,
	.dynamic_switching = true,
	.init = sugov_init,
	.exit = sugov_exit,
	.start = sugov_start,
	.stop = sugov_stop,
	.limits = sugov_limits,
};

static int __init waltgov_register(void)
{
	return cpufreq_register_governor(&walt_gov);
}
fs_initcall(waltgov_register);
