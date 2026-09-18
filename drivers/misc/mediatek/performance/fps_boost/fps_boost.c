// SPDX-License-Identifier: GPL-2.0
/*
 * fps_boost - frame-aware CPU boost for MediaTek 4.14 (experimental)
 *
 * Watches the *rendered frame cadence* of the main GPU rendering head and,
 * when the achieved frame rate drops below the target, applies up to three
 * complementary actuators:
 *
 *  1) cpufreq floor - raise each cluster's minimum frequency for a while.
 *                     On 4.14 this must be carried in user_policy.min, see
 *                     fb_apply_policy() below for why policy->min does not
 *                     work.
 *  2) WALT RTG      - attach the renderer to a related-thread-group id so the
 *                     ported "walt" cpufreq governor can push its target
 *                     utilisation to rtg_boost_freq while the group is busy.
 *  3) graded boost  - the strength of (1) grows with the size of the fps
 *                     deficit instead of being a fixed binary step.
 *
 * Frame sources:
 *  - primary: DRM present (mtk_drm_present_fp), once per presented frame,
 *    always available in-kernel;
 *  - optional: the GED-KPI main-head notifier (ged_kpi_fps_notify_fp), used to
 *    learn the target fps the app/GED is aiming at and - only while the
 *    display source is silent - as a coarse cadence fallback.
 *
 * Everything is tunable at runtime through /proc/fps_boost/ and disabled by
 * default (enable = 0).
 *
 * This is a user-space-independent complement to MTK FPSGO/perfmgr.  The
 * frequency ceiling still belongs to MTK's own governor/thermal, so boost_pct
 * and hold_ms may need on-device tuning.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/cpufreq.h>
#include <linux/rwsem.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/cpumask.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/ratelimit.h>

#define FPS_BOOST_PROC_DIR	"fps_boost"

/* ------------------------------------------------------------------ */
/* callbacks provided by the display / GPU stack                       */
extern void (*ged_kpi_fps_notify_fp)(int pid,
		unsigned long long frame_interval_ns,
		int target_fps,
		int target_fps_margin,
		int is_sf);

extern void (*mtk_drm_present_fp)(void);

/* WALT coupling (ported "walt" scheduler stack): the renderer is attached to
 * a related-thread-group id; the governor reacts to the group's load.
 */
extern int sched_set_group_id(struct task_struct *p, unsigned int group_id);
extern unsigned int sched_get_group_id(struct task_struct *p);

#define FB_RTG_MAX_ID	19	/* MAX_NUM_CGROUP_COLOC_ID - 1 (walt.c) */

/* ------------------------------------------------------------------ */
/* tunables (all exposed under /proc/fps_boost/)                       */
static int  fb_enable;
static int  fb_target_fps = 60;
static int  fb_margin_fps = 1;
static int  fb_boost_pct  = 60;	/* base strength: % of the cluster max */
static int  fb_hold_ms    = 250;	/* keep boosting at least this long */
static int  fb_sample_ms  = 200;	/* control loop period */
static int  fb_rtg_id     = 1;	/* WALT RTG id for the renderer, 0 = off */
/* RTG attach policy: 0 = off, 1 = only while boosting, 2 = whenever the
 * renderer is tracked (the group's own load then decides if the governor
 * acts at all - no attach/detach churn when the boost toggles).
 */
static int  fb_rtg_mode   = 2;
static int  fb_floor_en   = 1;	/* raise the cpufreq floor */
static int  fb_graded     = 1;	/* scale boost_pct with the fps deficit */
static int  fb_ged_target = 1;	/* prefer the target fps GED reports */

#define FB_WARMUP_SAMPLES 8

/* A commit gap longer than this means idle, not a slow frame. */
#define FB_PRESENT_MAX_GAP_NS	100000000ULL	/* 100 ms */
#define FB_IDLE_TIMEOUT_NS	500000000ULL	/* 500 ms without a frame */
#define FB_PCT_QUANTUM		5		/* graded steps of 5 % */

/* ------------------------------------------------------------------ */
/* state                                                               */
static int  fb_pid;			/* pid of the main rendering head */
static int  fb_ged_tgt;			/* target fps reported by GED, 0 = none */
static u64  fb_last_interval_ns;
static u64  fb_ema_interval_ns;		/* EMA frame interval (ns) */
static u32  fb_sample_count;
static u32  fb_all_samples;		/* includes SF-head frames (diagnostic) */
static u32  fb_ged_samples;		/* cadence-fallback samples */
static bool fb_boosting;
static u32  fb_cur_pct;			/* strength currently applied */
static unsigned long fb_last_jank_jiffies;
static bool fb_applied;			/* cpufreq floor currently applied */
static u32  fb_boost_events;
static u64  fb_last_present_ns;
static u64  fb_last_ged_ns;

static int  fb_rtg_pid;			/* pid currently in the WALT group */
static int  fb_rtg_cur_id;		/* group id in use for that pid */
static bool fb_rtg_attached;

static struct workqueue_struct *fb_wq;
static struct delayed_work fb_dwork;
static DEFINE_MUTEX(fb_lock);

static struct proc_dir_entry *fb_proc_root;

/* ------------------------------------------------------------------ */
/* interval feed: EMA + counters                                       */
static void fb_note_interval(unsigned long long interval_ns)
{
	if (interval_ns <= 0 || interval_ns > (5ULL * 1000000000ULL))
		interval_ns = fb_last_interval_ns ? fb_last_interval_ns
						  : 16666666ULL;	/* 60fps */

	fb_last_interval_ns = interval_ns;

	if (fb_ema_interval_ns == 0)
		fb_ema_interval_ns = interval_ns;
	else
		fb_ema_interval_ns = (fb_ema_interval_ns * 3 + interval_ns) >> 2;

	if (fb_sample_count < 0xffffffffU)
		fb_sample_count++;
}

/* Forget the measured cadence: a newly tracked renderer must not inherit the
 * previous one's frame interval, and after an idle period the old EMA is
 * worthless.  The activity timestamps are deliberately left alone, they are
 * what detects the idle period in the first place.
 */
static void fb_reset_ema(void)
{
	fb_last_interval_ns = 0;
	fb_ema_interval_ns = 0;
	fb_sample_count = 0;
}

/* Newest sign of life from either frame source */
static u64 fb_last_activity_ns(void)
{
	return max(fb_last_present_ns, fb_last_ged_ns);
}

/* Target fps to judge against: GED's number when it looks sane and the user
 * asked for it, otherwise the per-app tunable.
 */
static unsigned int fb_effective_target(void)
{
	if (fb_ged_target && fb_ged_tgt > 0 && fb_ged_tgt <= 300)
		return (unsigned int)fb_ged_tgt;

	return (unsigned int)fb_target_fps;
}

/* Boost strength for a measured fps: fb_boost_pct at the jank threshold,
 * growing linearly with the deficit up to 100 %, quantised so that EMA jitter
 * does not rewrite cpufreq every single tick.
 */
static unsigned int fb_tier_pct(unsigned int fps, unsigned int target)
{
	unsigned int pct, deficit;

	if (!fb_graded || !target || fps >= target)
		return (unsigned int)fb_boost_pct;

	deficit = target - fps;
	pct = (unsigned int)fb_boost_pct +
	      ((100 - (unsigned int)fb_boost_pct) * deficit) / target;
	pct = DIV_ROUND_UP(pct, FB_PCT_QUANTUM) * FB_PCT_QUANTUM;

	return min(pct, 100U);
}

/* ------------------------------------------------------------------ */
/* frame sources                                                       */
/* GED-KPI main-head notifier: carries the app's target fps and, on ROMs whose
 * display path does not feed us, a coarse cadence fallback.
 */
static void fb_frame_notify(int pid, unsigned long long interval_ns,
			    int target_fps, int target_fps_margin, int is_sf)
{
	u64 now;

	if (!fb_enable)
		return;

	if (fb_all_samples < 0xffffffffU)
		fb_all_samples++;

	if (is_sf)
		return;

	if (target_fps > 0 && target_fps <= 300)
		fb_ged_tgt = target_fps;

	if (fb_pid != pid) {
		fb_pid = pid;
		fb_reset_ema();
		pr_info_ratelimited("[fps_boost] now tracking pid %d\n", pid);
	}

	now = ktime_get_mono_fast_ns();

	/* GED hands us the frame's CPU time, not the frame interval, so only
	 * trust it while the display source has gone quiet.
	 */
	if (!fb_last_present_ns ||
	    now - fb_last_present_ns > FB_IDLE_TIMEOUT_NS) {
		fb_ged_samples++;
		fb_note_interval(interval_ns);
	}

	fb_last_ged_ns = now;
	(void)target_fps_margin;
}

/* Display present notifier: called once per DRM atomic commit, i.e. per frame
 * the display actually presents.  Always active in-kernel (no HAL needed).
 */
static void fb_present_notify(void)
{
	u64 now, gap;

	if (!fb_enable)
		return;

	/* ktime_get_mono_fast_ns(): plain clocksource read, no seqcount retry
	 * loop - this runs once per presented frame.
	 */
	now = ktime_get_mono_fast_ns();
	if (fb_last_present_ns && now > fb_last_present_ns) {
		gap = now - fb_last_present_ns;
		/* feed the EMA only for active rendering; idle gaps would
		 * drag the measured fps down and keep the boost on forever
		 */
		if (gap <= FB_PRESENT_MAX_GAP_NS)
			fb_note_interval(gap);
	} else {
		fb_note_interval(0ULL);
	}
	fb_last_present_ns = now;
}

/* ------------------------------------------------------------------ */
/* cpufreq actuator: raise / restore the boost floor.                  */
/*                                                                     */
/* NOTE: poking policy->min and calling cpufreq_update_policy() does    */
/* nothing on 4.14 - the update path rebuilds the policy from           */
/* user_policy (cpufreq.c: "new_policy.min = policy->user_policy.min")  */
/* and throws the manual floor away, so the boost never reached the     */
/* hardware.  The floor is therefore carried in user_policy.min,        */
/* exactly like a scaling_min_freq write would, remembering the         */
/* previous value so it can be handed back on release.                  */
/* ------------------------------------------------------------------ */
static unsigned int fb_saved_min[NR_CPUS];
static unsigned int fb_boost_floor[NR_CPUS];

static int fb_apply_policy(struct cpufreq_policy *pol, bool on,
			   unsigned int pct)
{
	unsigned int cpu, want;
	bool changed = false;

	if (!pol || !pol->cpuinfo.max_freq)
		return -EINVAL;

	cpu = pol->cpu;
	if (cpu >= ARRAY_SIZE(fb_saved_min))
		return -EINVAL;

	down_write(&pol->rwsem);

	if (on) {
		want = pol->cpuinfo.max_freq * min(pct, 100U) / 100;
		if (want > pol->cpuinfo.max_freq)
			want = pol->cpuinfo.max_freq;

		if (!fb_boost_floor[cpu]) {
			/* first tick of this boost: remember the base */
			fb_saved_min[cpu] = pol->user_policy.min;
		} else if (pol->user_policy.min != fb_boost_floor[cpu]) {
			/* someone (sysfs, thermal) moved the floor while we
			 * were boosting: adopt it as the new base
			 */
			fb_saved_min[cpu] = pol->user_policy.min;
		}

		/* never lower what the user asked for */
		if (want < fb_saved_min[cpu])
			want = fb_saved_min[cpu];

		fb_boost_floor[cpu] = want;

		if (pol->user_policy.min != want) {
			pol->user_policy.min = want;
			changed = true;
		}
	} else {
		if (!fb_boost_floor[cpu])
			goto unlock;	/* nothing of ours to undo */

		/* only step out of the way while the floor is still ours, so a
		 * scaling_min_freq written during the boost is preserved
		 */
		if (pol->user_policy.min == fb_boost_floor[cpu] &&
		    pol->user_policy.min != fb_saved_min[cpu]) {
			pol->user_policy.min = fb_saved_min[cpu];
			changed = true;
		}

		fb_boost_floor[cpu] = 0;
		fb_saved_min[cpu] = 0;
	}

unlock:
	up_write(&pol->rwsem);

	/* the update path takes rwsem itself, hence after up_write() */
	if (changed)
		cpufreq_update_policy(cpu);

	return 0;
}

/* ------------------------------------------------------------------ */
/* WALT related-thread-group actuator                                  */
static struct task_struct *fb_find_task(int pid)
{
	struct task_struct *task;

	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (task)
		get_task_struct(task);
	rcu_read_unlock();

	return task;
}

static int fb_rtg_desired(bool boosting)
{
	if (!fb_rtg_id || fb_pid <= 0 || !fb_enable)
		return 0;

	if (fb_rtg_mode == 2)
		return fb_rtg_id;
	if (fb_rtg_mode == 1 && boosting)
		return fb_rtg_id;

	return 0;
}

static void fb_rtg_detach(void)
{
	struct task_struct *task;

	if (!fb_rtg_attached)
		return;

	task = fb_find_task(fb_rtg_pid);
	if (task) {
		/* only undo our own attachment */
		if (sched_get_group_id(task) == (unsigned int)fb_rtg_cur_id)
			sched_set_group_id(task, 0);
		put_task_struct(task);
	}

	pr_info_ratelimited("[fps_boost] rtg: pid %d left group %d\n",
			    fb_rtg_pid, fb_rtg_cur_id);
	fb_rtg_attached = false;
	fb_rtg_pid = 0;
}

static void fb_apply_rtg(bool boosting)
{
	struct task_struct *task;
	int want = fb_rtg_desired(boosting);

	if (!want) {
		fb_rtg_detach();
		return;
	}

	if (fb_rtg_attached) {
		if (fb_rtg_pid == fb_pid && fb_rtg_cur_id == want)
			return;	/* membership already correct */
		fb_rtg_detach();
	}

	task = fb_find_task(fb_pid);
	if (!task) {
		pr_info_ratelimited("[fps_boost] rtg: pid %d not found\n",
				    fb_pid);
		return;
	}

	if (!sched_set_group_id(task, (unsigned int)want)) {
		fb_rtg_attached = true;
		fb_rtg_pid = fb_pid;
		fb_rtg_cur_id = want;
		pr_info_ratelimited("[fps_boost] rtg: pid %d -> group %d\n",
				    fb_rtg_pid, fb_rtg_cur_id);
	}
	put_task_struct(task);
}

/* ------------------------------------------------------------------ */
/* actuators (workqueue context only, never under fb_lock)             */
static void fb_apply_actuator(bool on, unsigned int pct)
{
	struct cpufreq_policy *pol;
	unsigned int cpu;
	bool want_floor = on && fb_floor_en;

	if (want_floor || fb_applied) {
		for_each_possible_cpu(cpu) {
			pol = cpufreq_cpu_get(cpu);
			if (!pol)
				continue;

			/* cpufreq_cpu_get() hands out the cluster policy for
			 * every cpu in it; touch each policy once only, one
			 * cpufreq_update_policy() per cluster is enough
			 */
			if (pol->cpu == cpu)
				fb_apply_policy(pol, want_floor, pct);
			cpufreq_cpu_put(pol);
		}
	}

	if (want_floor != fb_applied) {
		if (want_floor)
			fb_boost_events++;
		fb_applied = want_floor;
	}

	/* keep the WALT group membership in sync with the boost state */
	fb_apply_rtg(on);
}

/* ------------------------------------------------------------------ */
/* control loop                                                        */
static void fb_control(struct work_struct *work)
{
	unsigned int target, fps = 0;
	bool want = false;
	bool requeue = true;

	mutex_lock(&fb_lock);

	target = fb_effective_target();

	if (!fb_enable) {
		fb_boosting = false;
		want = false;
		requeue = false;	/* loop halts while disabled; enable re-queues */
	} else if (fb_last_activity_ns() &&
		   ktime_get_mono_fast_ns() - fb_last_activity_ns() >
		   FB_IDLE_TIMEOUT_NS) {
		/* nothing presented for a while -> idle: relax and forget the
		 * stale cadence so the next burst re-warms
		 */
		fb_reset_ema();
		want = false;
	} else if (fb_ema_interval_ns == 0 ||
		   fb_sample_count < FB_WARMUP_SAMPLES) {
		/* no frames yet / warming up -> stay relaxed */
		want = false;
	} else {
		int f, tg;

		fps = (unsigned int)(1000000000ULL / fb_ema_interval_ns);
		f = (int)fps;
		tg = (int)target;

		if (f < tg - fb_margin_fps) {
			/* dropping frames -> want boost */
			want = true;
			fb_last_jank_jiffies = jiffies;
		} else if (fb_boosting) {
			/* release once comfortably above target and hold expired */
			if (f >= tg + fb_margin_fps &&
			    time_after(jiffies,
				       fb_last_jank_jiffies +
				       msecs_to_jiffies(fb_hold_ms)))
				want = false;
			else
				want = true;
		}
	}

	fb_boosting = want;
	fb_cur_pct = want ? fb_tier_pct(fps, target) : 0;
	mutex_unlock(&fb_lock);

	fb_apply_actuator(want, fb_cur_pct);

	if (requeue)
		queue_delayed_work(fb_wq, &fb_dwork,
				   msecs_to_jiffies(fb_sample_ms));
}

/* ------------------------------------------------------------------ */
/* proc interface                                                      */
static int fb_status_show(struct seq_file *m, void *v)
{
	u64 ema = fb_ema_interval_ns;
	unsigned int fps = ema ? (unsigned int)(1000000000ULL / ema) : 0;

	seq_printf(m, "enable      : %d\n", fb_enable);
	seq_printf(m, "target_fps  : %d\n", fb_target_fps);
	seq_printf(m, "eff_target  : %u\n", fb_effective_target());
	seq_printf(m, "ged_target  : %d\n", fb_ged_tgt);
	seq_printf(m, "ged_target_en: %d\n", fb_ged_target);
	seq_printf(m, "margin_fps  : %d\n", fb_margin_fps);
	seq_printf(m, "boost_pct   : %d\n", fb_boost_pct);
	seq_printf(m, "cur_pct     : %u\n", fb_cur_pct);
	seq_printf(m, "graded      : %d\n", fb_graded);
	seq_printf(m, "hold_ms     : %d\n", fb_hold_ms);
	seq_printf(m, "sample_ms   : %d\n", fb_sample_ms);
	seq_printf(m, "floor_en    : %d\n", fb_floor_en);
	seq_printf(m, "pid         : %d\n", fb_pid);
	seq_printf(m, "ema_fps     : %u\n", fps);
	seq_printf(m, "samples     : %u\n", fb_sample_count);
	seq_printf(m, "all_samples : %u\n", fb_all_samples);
	seq_printf(m, "ged_samples : %u\n", fb_ged_samples);
	seq_printf(m, "boosting    : %u\n", fb_boosting);
	seq_printf(m, "boost_events: %u\n", fb_boost_events);
	seq_printf(m, "rtg_id      : %d\n", fb_rtg_id);
	seq_printf(m, "rtg_mode    : %d\n", fb_rtg_mode);
	seq_printf(m, "rtg_set     : %u\n", fb_rtg_attached);
	seq_printf(m, "rtg_pid     : %d\n", fb_rtg_pid);
	return 0;
}

static int fb_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, fb_status_show, NULL);
}

/* ------------------------------------------------------------------ */
/* proc rw nodes: hand-rolled ops using PDE_DATA(inode) as the variable
 * pointer passed to proc_create_data().  We deliberately do NOT use
 * DEFINE_SIMPLE_ATTRIBUTE: its simple_attr_open() does
 *   attr->data = inode->i_private
 * but on procfs inode->i_private holds the proc_dir_entry (PDE), not our
 * data, so writes there would corrupt the PDE and crash.  procfs data is
 * accessed via PDE_DATA(inode).
 */
static int fb_node_open(struct inode *inode, struct file *file)
{
	file->private_data = PDE_DATA(inode);
	return nonseekable_open(inode, file);
}

static ssize_t fb_node_read(struct file *file, char __user *ubuf,
			    size_t len, loff_t *ppos)
{
	int *p = file->private_data;
	char buf[32];
	int n;

	n = snprintf(buf, sizeof(buf), "%d\n", p ? *p : 0);
	return simple_read_from_buffer(ubuf, len, ppos, buf, n);
}

static ssize_t fb_node_write(struct file *file, const char __user *ubuf,
			     size_t len, loff_t *ppos)
{
	int *p = file->private_data;
	char kbuf[32];
	long val;
	int ret;

	if (len == 0)
		return 0;
	if (len > sizeof(kbuf) - 1)
		len = sizeof(kbuf) - 1;
	if (copy_from_user(kbuf, ubuf, len))
		return -EFAULT;
	kbuf[len] = '\0';

	ret = kstrtol(kbuf, 0, &val);
	if (ret)
		return ret;

	mutex_lock(&fb_lock);
	if (p == &fb_enable) {
		fb_enable = val ? 1 : 0;
	} else if (p == &fb_target_fps) {
		fb_target_fps = clamp_val(val, 1, 300);
	} else if (p == &fb_margin_fps) {
		fb_margin_fps = clamp_val(val, 0, 120);
	} else if (p == &fb_boost_pct) {
		fb_boost_pct = clamp_val(val, 5, 100);
	} else if (p == &fb_hold_ms) {
		fb_hold_ms = clamp_val(val, 10, 10000);
	} else if (p == &fb_sample_ms) {
		fb_sample_ms = clamp_val(val, 20, 5000);
	} else if (p == &fb_rtg_id) {
		fb_rtg_id = clamp_val(val, 0, FB_RTG_MAX_ID);
	} else if (p == &fb_rtg_mode) {
		fb_rtg_mode = clamp_val(val, 0, 2);
	} else if (p == &fb_floor_en) {
		fb_floor_en = val ? 1 : 0;
	} else if (p == &fb_graded) {
		fb_graded = val ? 1 : 0;
	} else if (p == &fb_ged_target) {
		fb_ged_target = val ? 1 : 0;
	} else {
		*p = (int)val;
	}
	mutex_unlock(&fb_lock);

	/* one tick so the new setting is acted upon right away; the actuator
	 * itself never runs synchronously from here.
	 */
	if (fb_wq)
		queue_delayed_work(fb_wq, &fb_dwork, 0);

	return len;
}

static const struct file_operations fb_node_fops = {
	.owner	= THIS_MODULE,
	.open	= fb_node_open,
	.read	= fb_node_read,
	.write	= fb_node_write,
	.llseek	= no_llseek,
};

static const struct file_operations fb_status_fops = {
	.owner	= THIS_MODULE,
	.open	= fb_status_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release= single_release,
};

/* ------------------------------------------------------------------ */
static int __init fb_init(void)
{
	struct proc_dir_entry *e;

	fb_wq = create_singlethread_workqueue("fps_boost");
	if (!fb_wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&fb_dwork, fb_control);

	fb_proc_root = proc_mkdir(FPS_BOOST_PROC_DIR, NULL);
	if (!fb_proc_root) {
		destroy_workqueue(fb_wq);
		return -ENOMEM;
	}

	e = proc_create("status", 0444, fb_proc_root, &fb_status_fops);
	/* rw nodes: variable pointer is stored via proc_create_data() and
	 * fetched with PDE_DATA(inode) in fb_node_open().  (Do NOT use
	 * DEFINE_SIMPLE_ATTRIBUTE here - see fb_node_fops comment.)
	 */
	e = proc_create_data("enable", 0644, fb_proc_root,
			     &fb_node_fops, &fb_enable);
	e = proc_create_data("target_fps", 0644, fb_proc_root,
			     &fb_node_fops, &fb_target_fps);
	e = proc_create_data("margin_fps", 0644, fb_proc_root,
			     &fb_node_fops, &fb_margin_fps);
	e = proc_create_data("boost_pct", 0644, fb_proc_root,
			     &fb_node_fops, &fb_boost_pct);
	e = proc_create_data("hold_ms", 0644, fb_proc_root,
			     &fb_node_fops, &fb_hold_ms);
	e = proc_create_data("sample_ms", 0644, fb_proc_root,
			     &fb_node_fops, &fb_sample_ms);
	e = proc_create_data("rtg_id", 0644, fb_proc_root,
			     &fb_node_fops, &fb_rtg_id);
	e = proc_create_data("rtg_mode", 0644, fb_proc_root,
			     &fb_node_fops, &fb_rtg_mode);
	e = proc_create_data("floor_en", 0644, fb_proc_root,
			     &fb_node_fops, &fb_floor_en);
	e = proc_create_data("graded", 0644, fb_proc_root,
			     &fb_node_fops, &fb_graded);
	e = proc_create_data("ged_target", 0644, fb_proc_root,
			     &fb_node_fops, &fb_ged_target);
	if (!e)
		pr_warn("[fps_boost] proc node creation incomplete\n");

	/* subscribe to frame notifiers (display present is the primary source) */
	ged_kpi_fps_notify_fp = fb_frame_notify;
	mtk_drm_present_fp = fb_present_notify;

	queue_delayed_work(fb_wq, &fb_dwork, msecs_to_jiffies(1000));
	pr_info("[fps_boost] loaded (disable by /proc/%s/enable=0)\n",
		FPS_BOOST_PROC_DIR);
	return 0;
}

static void __exit fb_exit(void)
{
	cancel_delayed_work_sync(&fb_dwork);

	mutex_lock(&fb_lock);
	if (ged_kpi_fps_notify_fp == fb_frame_notify)
		ged_kpi_fps_notify_fp = NULL;
	if (mtk_drm_present_fp == fb_present_notify)
		mtk_drm_present_fp = NULL;
	fb_enable = 0;
	fb_boosting = false;
	mutex_unlock(&fb_lock);

	fb_apply_actuator(false, (unsigned int)fb_boost_pct);
	destroy_workqueue(fb_wq);

	if (fb_proc_root)
		remove_proc_entry(FPS_BOOST_PROC_DIR, NULL);
}

module_init(fb_init);
module_exit(fb_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Frame-aware CPU boost (experimental)");
MODULE_AUTHOR("fps_boost");
