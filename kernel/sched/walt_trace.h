/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Trace events for the "walt" cpufreq governor (Qualcomm waltgov port).
 *
 * Consumed from cpufreq_walt.c only (CREATE_TRACE_POINTS lives there).
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM walt

#if !defined(_TRACE_WALTGOV_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_WALTGOV_H

#include <linux/tracepoint.h>

TRACE_EVENT(waltgov_next_freq,

	TP_PROTO(int cpu, unsigned long util, unsigned long max,
		 unsigned int raw_freq, unsigned int freq,
		 unsigned long avg_cap, unsigned long pl, int hiload,
		 int boost, unsigned int flags, unsigned long nl, int rtgb),

	TP_ARGS(cpu, util, max, raw_freq, freq, avg_cap, pl, hiload, boost,
		flags, nl, rtgb),

	TP_STRUCT__entry(
		__field(	int,		cpu		)
		__field(	unsigned long,	util		)
		__field(	unsigned long,	max		)
		__field(	unsigned int,	raw_freq	)
		__field(	unsigned int,	freq		)
		__field(	unsigned long,	avg_cap		)
		__field(	unsigned long,	pl		)
		__field(	int,		hiload		)
		__field(	int,		boost		)
		__field(	unsigned int,	flags		)
		__field(	unsigned long,	nl		)
		__field(	int,		rtgb		)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->util		= util;
		__entry->max		= max;
		__entry->raw_freq	= raw_freq;
		__entry->freq		= freq;
		__entry->avg_cap	= avg_cap;
		__entry->pl		= pl;
		__entry->hiload		= hiload;
		__entry->boost		= boost;
		__entry->flags		= flags;
		__entry->nl		= nl;
		__entry->rtgb		= rtgb;
	),

	TP_printk("cpu=%d util=%lu max=%lu raw_freq=%u freq=%u avg_cap=%lu pl=%lu hiload=%d boost=%d flags=0x%x nl=%lu rtgb=%d",
		  __entry->cpu, __entry->util, __entry->max,
		  __entry->raw_freq, __entry->freq, __entry->avg_cap,
		  __entry->pl, __entry->hiload, __entry->boost,
		  __entry->flags, __entry->nl, __entry->rtgb)
);

#endif /* _TRACE_WALTGOV_H */

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE walt_trace

/* This part must be outside protection */
#include <trace/define_trace.h>
