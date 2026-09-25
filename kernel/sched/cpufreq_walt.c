/*
 * CPUFreq governor "walt" - port of Qualcomm's WALT cpufreq governor
 * (kernel/sched/walt/cpufreq_walt.c) on top of the MTK schedutil framework
 * used by this kernel.
 *
 * Deviations from the original Qualcomm driver:
 *  - the governor is driven through the waltgov callbacks exported by WALT
 *    (window rollover, wakeup, tick, migration) instead of the generic
 *    cpufreq update-util hooks,
 *  - "rtg_boost_freq" is driven by a reduced port of Qualcomm's RTG: tasks
 *    carry a group id (sched_set_group_id()), a group's load is the sum of
 *    its queued tasks' demand and the boost triggers while that load exceeds
 *    half of the least capable CPU. Cgroup colocation, group time accounting
 *    (grp_time) and preferred-cluster placement are not ported,
 *  - "pl" uses WALT's predictive demand sum (bucket based prediction); the
 *    conservative_pl scaling of the original driver is not applied,
 *  - the schedutil iowait boost and the SCHED_CPUFREQ_DL "jump to max"
 *    handling are not part of the original governor and are dropped here.
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
	unsigned int rtg_boost_freq;		/* kHz (requires RTG support) */
	unsigned int adaptive_low_freq;		/* kHz */
	unsigned int adaptive_high_freq;	/* kHz */
	unsigned int target_load_thresh;	/* capacity scale */
	unsigned int target_load_shift;
	bool pl;
	int boost;
	/*
	 * When set, a hispeed_freq / rtg_boost_freq of 0 means "use the
	 * per-cluster default" (see walt_hispeed_freq()); clearing it makes 0
	 * mean "disabled" again.
	 */
	bool auto_boost;
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

	/*
	 * Per-cluster default boost frequencies, derived in sugov_start() from
	 * this policy's maximum.  They stand in while the matching tunable is 0
	 * and auto_boost is on.
	 */
	unsigned int def_hispeed_freq;
	unsigned int def_rtg_boost_freq;

	/* Last decision, exposed through the read-only walt/decision node */
	u64 last_time;
	u64 update_count;
	u64 hispeed_hits;
	u64 pl_hits;
	u64 nl_hits;
	unsigned long last_util;
	unsigned long last_max;
	unsigned long last_avg_cap;
	unsigned long last_pl;
	unsigned long last_nl;
	bool last_rtgb;
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
	struct waltgov_callback cb;
	struct sugov_policy *sg_policy;
	unsigned int cpu;

	u64 last_update;

	/* The fields below are only needed when sharing a policy. */
	unsigned long util;
	unsigned long max;
	unsigned int flags;

	/* WALT load info filled by waltgov_cpu_load() */
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
#define WALT_NL_RATIO			75
#define WALT_KHZ			1000
#define WALT_DEFAULT_HISPEED_LOAD	90
#define WALT_DEFAULT_TARGET_LOAD_THRESH	1024
#define WALT_DEFAULT_TARGET_LOAD_SHIFT	4

/*
 * Per-cluster defaults for the two frequency boosts, in %% of the policy's
 * maximum.  They are used while the matching tunable is 0 and auto_boost is
 * on: with both tunables at 0 and every cluster identical there would be
 * nothing WALT-specific left in the governor's behaviour.
 */
#define WALT_DEFAULT_HISPEED_PCT	80
#define WALT_DEFAULT_RTG_BOOST_PCT	70

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

	/*
	 * Above the target load threshold the original governor keeps only 6%
	 * headroom instead of the usual 25%, i.e. the target utilization at
	 * that frequency is 94% of the capacity rather than 80%.
	 */
	if (util >= sg_policy->tunables->target_load_thresh)
		util = mult_frac(util, 94, 100);
	else
		util = mult_frac(util, WALT_TARGET_LOAD, 100);

	return util;
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
		/*
		 * The window sequence restarted, so the accumulated average belongs
		 * to a different timeline: drop it instead of letting is_hiload()
		 * compare against a stale capacity indefinitely.  avg_cap == 0 also
		 * suppresses the hispeed boost until a full window is known.
		 */
		sg_policy->avg_cap = 0;
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
 * Effective hispeed / rtg_boost frequency: the tunable when it is set,
 * otherwise the per-cluster default while auto_boost is on.  0 means the boost
 * is disabled.
 */
static unsigned int walt_hispeed_freq(struct sugov_policy *sg_policy)
{
	unsigned int freq = sg_policy->tunables->hispeed_freq;

	if (!freq && sg_policy->tunables->auto_boost)
		freq = sg_policy->def_hispeed_freq;

	return freq;
}

static unsigned int walt_rtg_boost_freq(struct sugov_policy *sg_policy)
{
	unsigned int freq = sg_policy->tunables->rtg_boost_freq;

	if (!freq && sg_policy->tunables->auto_boost)
		freq = sg_policy->def_rtg_boost_freq;

	return freq;
}

/*
 * Mirrors waltgov_walt_adjust() of the original governor: apply the RTG
 * boost, the hispeed boost, the new task load (nl) fast ramp and the
 * predictive load (pl) of WALT.
 */
static void walt_adjust_util(struct sugov_cpu *sg_cpu, unsigned long cpu_util,
			     unsigned long *util, unsigned long *max)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct sugov_tunables *tunables = sg_policy->tunables;
	unsigned long pl = sg_cpu->walt_load.pl;
	unsigned long nl = sg_cpu->walt_load.nl;
	bool is_migration = sg_cpu->flags & WALT_CPUFREQ_IC_MIGRATION;
	bool is_hiload;

	if (sg_cpu->walt_load.rtgb_active) {
		unsigned int rtf = walt_rtg_boost_freq(sg_policy);

		if (rtf) {
			unsigned long rtgb = walt_target_util(sg_policy, rtf);

			*util = max(*util, rtgb);
		}
	}

	/*
	 * avg_cap == 0 means no WALT window has been accounted yet (or the
	 * window sequence just restarted): without it the comparison below would
	 * be trivially true and every first request would jump to hispeed_freq.
	 */
	is_hiload = sg_policy->avg_cap &&
		    (cpu_util >= mult_frac(sg_policy->avg_cap,
					   tunables->hispeed_load, 100));

	if (is_hiload && !is_migration) {
		unsigned int hsf = walt_hispeed_freq(sg_policy);

		if (hsf) {
			unsigned long hs = walt_target_util(sg_policy, hsf);

			if (hs > *util)
				sg_policy->hispeed_hits++;
			*util = max(*util, hs);
		}
	}

	/*
	 * A large part of the load comes from newly started tasks (app launch,
	 * forks): jump to the maximum frequency instead of building up over
	 * several windows.
	 */
	if (is_hiload && nl >= mult_frac(cpu_util, WALT_NL_RATIO, 100)) {
		sg_policy->nl_hits++;
		*util = *max;
	}

	if (tunables->pl && pl) {
		if (pl > *util)
			sg_policy->pl_hits++;
		*util = max(*util, pl);
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
	struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);
	unsigned long pl = sg_cpu->walt_load.pl;
	unsigned long nl = sg_cpu->walt_load.nl;
	bool rtgb = sg_cpu->walt_load.rtgb_active;
	bool hiload = (util >= mult_frac(sg_policy->avg_cap,
					 tunables->hispeed_load, 100));

	sg_policy->last_time = walt_ktime_clock();
	sg_policy->last_cpu = cpu;
	sg_policy->last_util = util;
	sg_policy->last_max = max;
	sg_policy->last_avg_cap = sg_policy->avg_cap;
	sg_policy->last_pl = pl;
	sg_policy->last_nl = nl;
	sg_policy->last_rtgb = rtgb;
	sg_policy->last_raw_freq = sg_policy->cached_raw_freq;
	sg_policy->last_freq = freq;
	sg_policy->last_flags = flags;
	sg_policy->update_count++;

	trace_waltgov_next_freq(cpu, util, max, sg_policy->cached_raw_freq,
				freq, sg_policy->avg_cap, pl, hiload,
				tunables->boost, flags, nl, rtgb);
}

/*
 * Qualcomm's linear util -> frequency mapping:
 *
 *	freq = (1 + 1/4) * util * fmax / cap		(25% headroom)
 *
 * and, once the utilization is above target_load_thresh and there is no
 * significant RT load on the CPU,
 *
 *	freq = max((1 + 2^-target_load_shift) * util,
 *		   1.25 * target_load_thresh) * fmax / cap
 *
 * i.e. the headroom shrinks towards the target load.
 */
static unsigned long walt_map_util_freq(struct sugov_policy *sg_policy,
					unsigned long util, unsigned long cap,
					int cpu)
{
	unsigned long fmax = sg_policy->policy->cpuinfo.max_freq;
	unsigned int shift = sg_policy->tunables->target_load_shift;
	unsigned long rt_util = READ_ONCE(cpu_rq(cpu)->rt.avg.util_avg);

	if (util >= sg_policy->tunables->target_load_thresh &&
	    rt_util < (cap >> 2))
		return max((fmax + (fmax >> shift)) * util,
			   (fmax + (fmax >> 2)) *
				sg_policy->tunables->target_load_thresh) / cap;

	return (fmax + (fmax >> 2)) * util / cap;
}

static unsigned int walt_get_next_freq(struct sugov_policy *sg_policy,
				       unsigned long util, unsigned long max,
				       int cpu)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	struct sugov_tunables *tunables = sg_policy->tunables;
	unsigned int freq, raw_freq;
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	int cid;
#endif

	raw_freq = walt_map_util_freq(sg_policy, util, max, cpu);
	sg_policy->cached_raw_freq = raw_freq;

	freq = raw_freq;

	/*
	 * Adaptive frequency band of the original governor: hold the frequency
	 * at adaptive_high_freq for anything in between adaptive_low_freq and
	 * adaptive_high_freq, and never go below adaptive_low_freq while set.
	 */
	if (tunables->adaptive_high_freq) {
		if (freq < tunables->adaptive_low_freq)
			freq = tunables->adaptive_low_freq;
		else if (freq <= tunables->adaptive_high_freq)
			freq = tunables->adaptive_high_freq;
	}

	freq = clamp_val(freq, policy->min, policy->max);
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	cid = arch_get_cluster_id(policy->cpu);
	freq = mt_cpufreq_find_close_freq(cid, freq);
#else
	freq = cpufreq_driver_resolve_freq(policy, freq);
#endif
	return freq;
}

/*
 * Adaptive frequency band of the original governor is applied inside
 * walt_get_next_freq(); this helper only exists for readability of the
 * update paths below.
 */
static void sugov_get_util(struct sugov_cpu *sg_cpu, unsigned long *util,
			   unsigned long *max)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned long max_cap;

	max_cap = arch_scale_cpu_capacity(NULL, sg_cpu->cpu);

	/*
	 * Read the utilization the way the stock schedutil governor of this
	 * kernel does - boosted_cpu_util() adds schedtune's boost for the
	 * foreground cgroup on top of the WALT/PELT utilization. Taking the raw
	 * WALT sum instead (as the original Qualcomm governor can, because it
	 * has no schedtune) lost that boost and made every boosted workload look
	 * lighter than the platform asked for.
	 */
	*util = boosted_cpu_util(sg_cpu->cpu);

	/* WALT's own load signals: nl, pl, running related thread group */
	waltgov_cpu_load(sg_cpu->cpu, &sg_cpu->walt_load);

	if (idle_cpu(sg_cpu->cpu))
		*util = 0;

	*util = min(*util, max_cap);
	*util = uclamp_util(cpu_rq(sg_cpu->cpu), *util);

	/* Qualcomm waltgov "boost" tunable (percent, may be negative) */
	if (sg_policy->tunables->boost) {
		int b = sg_policy->tunables->boost;

		*util = mult_frac(*util, 100 + b, 100);
		sg_cpu->walt_load.nl = mult_frac(sg_cpu->walt_load.nl,
						 100 + b, 100);
	}

	*max = max_cap;
}

/*
 * The original governor has no iowait boost and it cannot be fed by WALT's
 * events anyway, so the schedutil variant of it is not carried over.
 */

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

static void waltgov_update_freq_single(struct waltgov_callback *cb, u64 time,
				       unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(cb, struct sugov_cpu, cb);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util, max;
	unsigned int next_f;
	bool busy;

	if (!sg_policy->tunables->pl && (flags & WALT_CPUFREQ_PL))
		return;

	sg_cpu->flags = flags;
	sg_cpu->last_update = time;

	if (!sugov_should_update_freq(sg_policy, time))
		return;

	busy = sugov_cpu_is_busy(sg_cpu);

	sugov_get_util(sg_cpu, &util, &max);

	/*
	 * WALT may invoke this callback for a remote rq as well (task
	 * migration), so serialise the decision with the policy lock just like
	 * the shared-policy path does.
	 */
	raw_spin_lock(&sg_policy->update_lock);

	/*
	 * The average capacity only makes sense once a WALT window is known;
	 * without one (WALT utilization disabled by sysctl) an update would
	 * only reset the tracker with a bogus window and leave avg_cap stale.
	 */
	if (sg_cpu->walt_load.ws)
		walt_calc_avg_cap(sg_policy, sg_cpu->walt_load.ws, policy->cur);
	walt_adjust_util(sg_cpu, util, &util, &max);
	next_f = walt_get_next_freq(sg_policy, util, max, sg_cpu->cpu);
	waltgov_trace_decision(sg_policy, sg_cpu->cpu, util, max, next_f,
			       flags);

	/*
	 * Do not reduce the frequency if the CPU has not been idle recently,
	 * as the reduction is likely to be premature then.
	 */
	if (busy && next_f < sg_policy->next_freq &&
	    sg_policy->next_freq != UINT_MAX) {
		next_f = sg_policy->next_freq;

		/* Reset cached freq as next_freq has changed */
		sg_policy->cached_raw_freq = 0;
	}

	sugov_update_commit(sg_policy, time, next_f);

	raw_spin_unlock(&sg_policy->update_lock);
}

static unsigned int sugov_next_freq_shared(struct sugov_cpu *sg_cpu, u64 time)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util = 0, max = 1;
	unsigned int j;
	unsigned int next_f;

	for_each_cpu(j, policy->cpus) {
		struct sugov_cpu *j_sg_cpu = &per_cpu(sugov_cpu, j);
		unsigned long j_util, j_max;
		s64 delta_ns;

		/*
		 * If the CPU utilization was last updated before the previous
		 * frequency update and the time elapsed since then is long enough,
		 * don't take the CPU into account as it probably is idle now.
		 */
		delta_ns = time - j_sg_cpu->last_update;
		if (delta_ns > TICK_NSEC && idle_cpu(j))
			continue;

		j_util = j_sg_cpu->util;
		j_max = j_sg_cpu->max;

		if (j_util * max > j_max * util) {
			util = j_util;
			max = j_max;
		}

		walt_adjust_util(j_sg_cpu, j_util, &util, &max);
	}

	next_f = walt_get_next_freq(sg_policy, util, max, sg_cpu->cpu);
	waltgov_trace_decision(sg_policy, sg_cpu->cpu, util, max, next_f,
			       sg_cpu->flags);
	return next_f;
}

static void waltgov_update_freq_shared(struct waltgov_callback *cb, u64 time,
				       unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(cb, struct sugov_cpu, cb);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned long util, max;
	unsigned int next_f;

	if (!sg_policy->tunables->pl && (flags & WALT_CPUFREQ_PL))
		return;

	sugov_get_util(sg_cpu, &util, &max);

	raw_spin_lock(&sg_policy->update_lock);

	sg_cpu->util = util;
	sg_cpu->max = max;
	sg_cpu->flags = flags;

	sg_cpu->last_update = time;

	if (sugov_should_update_freq(sg_policy, time)) {
		if (sg_cpu->walt_load.ws)
			walt_calc_avg_cap(sg_policy, sg_cpu->walt_load.ws,
					  sg_policy->policy->cur);
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
WALTGOV_SHOW(rtg_boost_freq);
WALTGOV_STORE(rtg_boost_freq);
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
static struct governor_attr rtg_boost_freq = __ATTR_RW(rtg_boost_freq);
static struct governor_attr adaptive_low_freq = __ATTR_RW(adaptive_low_freq);
static struct governor_attr adaptive_high_freq = __ATTR_RW(adaptive_high_freq);
static struct governor_attr target_load_thresh = __ATTR_RW(target_load_thresh);
static struct governor_attr target_load_shift = __ATTR_RW(target_load_shift);
static struct governor_attr pl = __ATTR_RW(pl);
static struct governor_attr boost = __ATTR_RW(boost);

static ssize_t auto_boost_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sprintf(buf, "%d\n", tunables->auto_boost);
}

static ssize_t auto_boost_store(struct gov_attr_set *attr_set,
				const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;
	tunables->auto_boost = val;

	return count;
}

/*
 * With auto_boost on, hispeed_freq / rtg_boost_freq left at 0 select the
 * per-cluster default instead of "disabled".
 */
static struct governor_attr auto_boost = __ATTR_RW(auto_boost);

static ssize_t decision_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	int len = 0;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		len += sprintf(buf + len,
			"policy%u cpu=%u util=%lu max=%lu avg_cap=%lu pl=%lu nl=%lu rtgb=%d raw_freq=%u freq=%u flags=0x%x updates=%llu hispeed_hits=%llu pl_hits=%llu nl_hits=%llu\n",
				sg_policy->policy->cpu, sg_policy->last_cpu,
				sg_policy->last_util, sg_policy->last_max,
				sg_policy->last_avg_cap, sg_policy->last_pl,
				sg_policy->last_nl, sg_policy->last_rtgb,
				sg_policy->last_raw_freq, sg_policy->last_freq,
				sg_policy->last_flags,
				(unsigned long long)sg_policy->update_count,
				(unsigned long long)sg_policy->hispeed_hits,
				(unsigned long long)sg_policy->pl_hits,
				(unsigned long long)sg_policy->nl_hits);
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
	&rtg_boost_freq.attr,
	&pl.attr,
	&boost.attr,
	&auto_boost.attr,
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
	tunables->rtg_boost_freq = 0;
	tunables->adaptive_low_freq = 0;
	tunables->adaptive_high_freq = 0;
	tunables->target_load_thresh = WALT_DEFAULT_TARGET_LOAD_THRESH;
	tunables->target_load_shift = WALT_DEFAULT_TARGET_LOAD_SHIFT;
	tunables->pl = true;
	tunables->boost = 0;
	tunables->auto_boost = true;

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

	/*
	 * Per-cluster defaults for the two boosts.  Derived from this policy's
	 * maximum so that each cluster gets a sensible value, unlike a single
	 * global tunable which cannot fit all of them.
	 */
	sg_policy->def_hispeed_freq =
		mult_frac(policy->cpuinfo.max_freq, WALT_DEFAULT_HISPEED_PCT, 100);
	sg_policy->def_rtg_boost_freq =
		mult_frac(policy->cpuinfo.max_freq, WALT_DEFAULT_RTG_BOOST_PCT, 100);

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->cpu = cpu;
		sg_cpu->sg_policy = sg_policy;
	}

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		waltgov_add_callback(cpu, &sg_cpu->cb,
				     policy_is_shared(policy) ?
						waltgov_update_freq_shared :
						waltgov_update_freq_single);
	}
	return 0;
}

static void sugov_stop(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		waltgov_remove_callback(cpu);

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

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_WALT
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &walt_gov;
}
#endif
