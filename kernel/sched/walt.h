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
 */

#ifndef __WALT_H
#define __WALT_H

/*
 * WALT -> cpufreq governor callbacks, ported from Qualcomm's WALT
 * (struct waltgov_callback / waltgov_run_callback). WALT runs them from its
 * own event points - window rollover, task wakeup, tick and migration - so
 * that the "walt" governor no longer has to piggyback on the generic cpufreq
 * update-util hooks.
 */
#define WALT_CPUFREQ_ROLLOVER		(1U << 0)
#define WALT_CPUFREQ_CONTINUE		(1U << 1)
#define WALT_CPUFREQ_IC_MIGRATION	(1U << 2)
#define WALT_CPUFREQ_PL			(1U << 3)
#define WALT_CPUFREQ_EARLY_DET		(1U << 4)
#define WALT_CPUFREQ_BOOST_UPDATE	(1U << 5)

struct waltgov_callback {
	void (*func)(struct waltgov_callback *cb, u64 time, unsigned int flags);
};

DECLARE_PER_CPU(struct waltgov_callback *, waltgov_cb_data);

void waltgov_add_callback(int cpu, struct waltgov_callback *cb,
			  void (*func)(struct waltgov_callback *cb, u64 time,
				       unsigned int flags));
void waltgov_remove_callback(int cpu);
void waltgov_run_callback(struct rq *rq, unsigned int flags);

/*
 * Related thread group (RTG) support for the rtg_boost_freq of the walt
 * governor. Same names/prototypes as Qualcomm's API, reduced implementation.
 */
int sched_set_group_id(struct task_struct *p, unsigned int group_id);
unsigned int sched_get_group_id(struct task_struct *p);

#ifdef CONFIG_SCHED_WALT

void walt_update_task_ravg(struct task_struct *p, struct rq *rq, int event,
		u64 wallclock, u64 irqtime);
void walt_inc_cumulative_runnable_avg(struct rq *rq, struct task_struct *p);
void walt_dec_cumulative_runnable_avg(struct rq *rq, struct task_struct *p);
void walt_fixup_cumulative_runnable_avg(struct rq *rq, struct task_struct *p,
					u64 new_task_load);

void walt_fixup_busy_time(struct task_struct *p, int new_cpu);
void walt_init_new_task_load(struct task_struct *p);
void walt_mark_task_starting(struct task_struct *p);
void walt_set_window_start(struct rq *rq, struct rq_flags *rf);
void walt_migrate_sync_cpu(int cpu);
u64 walt_ktime_clock(void);
void walt_account_irqtime(int cpu, struct task_struct *curr, u64 delta,
                                  u64 wallclock);

u64 walt_irqload(int cpu);
int walt_cpu_high_irqload(int cpu);

/*
 * WALT load info consumed by the cpufreq governor(s), ported from
 * Qualcomm's kernel/sched/walt (walt_cpu_load/waltgov_cpu_load). The
 * utilization itself is read by the governor through boosted_cpu_util(),
 * i.e. the same value the stock schedutil governor uses.
 */
struct walt_cpu_load {
	unsigned long nl;
	unsigned long pl;
	bool rtgb_active;
	u64 ws;
};

void waltgov_cpu_load(int cpu, struct walt_cpu_load *walt_load);

#else /* CONFIG_SCHED_WALT */

static inline void walt_update_task_ravg(struct task_struct *p, struct rq *rq,
		int event, u64 wallclock, u64 irqtime) { }
static inline void walt_inc_cumulative_runnable_avg(struct rq *rq, struct task_struct *p) { }
static inline void walt_dec_cumulative_runnable_avg(struct rq *rq, struct task_struct *p) { }
static inline void walt_fixup_cumulative_runnable_avg(struct rq *rq,
						      struct task_struct *p,
						      u64 new_task_load) { }
static inline void walt_fixup_busy_time(struct task_struct *p, int new_cpu) { }
static inline void walt_init_new_task_load(struct task_struct *p) { }
static inline void walt_mark_task_starting(struct task_struct *p) { }
static inline void walt_set_window_start(struct rq *rq, struct rq_flags *rf) { }
static inline void walt_migrate_sync_cpu(int cpu) { }
static inline u64 walt_ktime_clock(void) { return 0; }

static inline void waltgov_add_callback(int cpu, struct waltgov_callback *cb,
			  void (*func)(struct waltgov_callback *cb, u64 time,
				       unsigned int flags)) { }
static inline void waltgov_remove_callback(int cpu) { }
static inline void waltgov_run_callback(struct rq *rq, unsigned int flags) { }
static inline int sched_set_group_id(struct task_struct *p,
				     unsigned int group_id) { return -EINVAL; }
static inline unsigned int sched_get_group_id(struct task_struct *p) { return 0; }

#define walt_cpu_high_irqload(cpu) false

#endif /* CONFIG_SCHED_WALT */

#if defined(CONFIG_CFS_BANDWIDTH) && defined(CONFIG_SCHED_WALT)
void walt_inc_cfs_cumulative_runnable_avg(struct cfs_rq *rq,
		struct task_struct *p);
void walt_dec_cfs_cumulative_runnable_avg(struct cfs_rq *rq,
		struct task_struct *p);
#else
static inline void walt_inc_cfs_cumulative_runnable_avg(struct cfs_rq *rq,
		struct task_struct *p) { }
static inline void walt_dec_cfs_cumulative_runnable_avg(struct cfs_rq *rq,
		struct task_struct *p) { }
#endif

extern bool walt_disabled;

#endif
