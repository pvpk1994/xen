/* SPDX-License-Identifier: MIT */
/*
 * Multi-resource ARINC653 scheduler for Xen
 *
 * Unlike ARINC653 (one global schedule for all CPUs), this scheduler
 * maintains a per-CPU schedule table that has independent ARINC653
 * major frame per CPU. Each pCPU's table is delivered separately via
 * adjust_global(putinfo) using the cpu selector in struct
 * xen_sysctl_sched_arinc653 interface.
 */

#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/timer.h>
#include <xen/softirq.h>
#include <xen/time.h>
#include <xen/errno.h>
#include <xen/list.h>
#include <xen/guest_access.h>
#include <public/sysctl.h>

#include "private.h"

#define DEFAULT_TIMESLICE	MILLISECS(10)
#define IDLE_TASK(cpu)		(sched_idle_unit(cpu))

/*
 * Return a pointer to multi-ARINC653 scheduler specific data information
 * associated with given UNIT
 */
#define ARINC653_MULTI_UNIT(unit)		((multi_a653_unit_t *)(unit)->priv)

/*
 * Given the schedule ops pointer, return the multi-ARINC653 specific
 * global scheduler ops
 */
#define ARINC653_MULTI_SCHED_PRIV(ops)		((multi_a653_sched_priv_t *) ((ops)->sched_data))

/*
 * Per-CPU data lives in the CPU's sched_res space
 */
#define ARINC653_MULTI_CPU(cpu)			((multi_a653_pcpu_t *) get_sched_res(cpu)->sched_priv)

typedef struct multi_a653_unit_s {
	struct sched_unit	*unit;
	bool			awake;
	struct list_head	list;
} multi_a653_unit_t;

typedef struct sched_entry_s {
	xen_domain_handle_t	dom_handle;
	int			unit_id;
	s_time_t		runtime;
	struct sched_unit	*unit;
} sched_entry_t;

typedef struct multi_a653_pcpu_s {
	unsigned int		cpuid;
	sched_entry_t		schedule[ARINC653_MAX_DOMAINS_PER_SCHEDULE];
	unsigned int		num_schedule_entries;
	s_time_t		major_frame;
	s_time_t		next_major_frame;
	unsigned int		sched_index;
	s_time_t		next_switch_time;
} multi_a653_pcpu_t;

typedef struct multi_a653_sched_priv_s {
	spinlock_t		lock;
	struct list_head	unit_list;
} multi_a653_sched_priv_t;

static int cf_check multi_a653_init(struct scheduler *ops)
{
	multi_a653_sched_priv_t *prv;

	prv = xzalloc(multi_a653_sched_priv_t);
	if (!prv)
		return -ENOMEM;

	ops->sched_data = prv;
	spin_lock_init(&prv->lock);
	INIT_LIST_HEAD(&prv->unit_list);

	return 0;
}

static void *cf_check multi_a653_alloc_pdata(const struct scheduler *ops,
					     int cpu)
{
	multi_a653_pcpu_t *m_a653_cpu = xzalloc(multi_a653_pcpu_t);

	if (!m_a653_cpu)
		return ERR_PTR(-ENOMEM);

	m_a653_cpu->cpuid = cpu;
	return m_a653_cpu;
}

static void cf_check multi_a653_free_pdata(const struct scheduler *ops,
					   void *pcpu, int cpu)
{
	xfree(pcpu);
}

static void cf_check multi_a653_deinit(struct scheduler *ops)
{
	xfree(ARINC653_MULTI_SCHED_PRIV(ops));
	ops->sched_data = NULL;
}

static void *cf_check multi_a653_alloc_udata(const struct scheduler *ops,
					     struct sched_unit *unit, void *dd)
{
	multi_a653_sched_priv_t *prv = ARINC653_MULTI_SCHED_PRIV(ops);
	multi_a653_unit_t *ma_unit = xmalloc(multi_a653_unit_t);
	unsigned long flags;

	if (!ma_unit)
		return NULL;

	ma_unit->unit = unit;
	ma_unit->awake = false;

	spin_lock_irqsave(&prv->lock, flags);

	/* Add non-Idle units to global sched queue */
	if (!is_idle_unit(unit))
		list_add(&ma_unit->list, &prv->unit_list);

	spin_unlock_irqrestore(&prv->lock, flags);

	return ma_unit;
}

static void cf_check multi_a653_free_udata(const struct scheduler *ops, void *priv)
{
	multi_a653_sched_priv_t *prv = ARINC653_MULTI_SCHED_PRIV(ops);
	multi_a653_unit_t *ma_unit = priv;
	unsigned long flags;

	if (!ma_unit)
		return;

	spin_lock_irqsave(&prv->lock, flags);

	/* Remove non-Idle units from global sched queue */
	if (!is_idle_unit(ma_unit->unit))
		list_del(&ma_unit->list);

	spin_unlock_irqrestore(&prv->lock, flags);

	xfree(ma_unit);
}

static const struct scheduler sched_arinc653_multi_def = {
	.name		=		"Multi ARINC653 Scheduler",
	.opt_name	=		"multi-arinc653",
	.sched_id	=		XEN_SCHEDULER_ARINC653_MULTI,
	.sched_data	=		NULL,

	.init		=		multi_a653_init,
	.deinit		=		multi_a653_deinit,

	.alloc_pdata	=		multi_a653_alloc_pdata,
	.free_pdata	=		multi_a653_free_pdata,
	.deinit_pdata	=		NULL, /* No unsets for this scheduler */

	.alloc_udata	=		multi_a653_alloc_udata,
	.free_udata	=		multi_a653_free_udata,
};

REGISTER_SCHEDULER(sched_arinc653_multi_def);
