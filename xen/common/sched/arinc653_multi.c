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
	bool			armed;
} multi_a653_pcpu_t;

typedef struct multi_a653_sched_priv_s {
	spinlock_t		lock;
	struct list_head	unit_list;
} multi_a653_sched_priv_t;

#ifdef CONFIG_SYSCTL
static int dom_handle_cmp(const xen_domain_handle_t h1,
			  const xen_domain_handle_t h2)
{
	return memcmp(h1, h2, sizeof(xen_domain_handle_t));
}

/* Caller needs to hold global scheduler lock */
static struct sched_unit *find_matching_unit(const struct scheduler *ops,
					     xen_domain_handle_t handle, int unit_id)
{
	multi_a653_unit_t *ma_unit;

	/* Iterate through the global scheduler list to find matching unit */
	list_for_each_entry(ma_unit, &ARINC653_MULTI_SCHED_PRIV(ops)->unit_list, list)
		if (dom_handle_cmp(ma_unit->unit->domain->handle, handle) == 0 &&
		    unit_id == ma_unit->unit->unit_id)
			return ma_unit->unit;

	return NULL;
}

/* Caller should be holding global scheduler and per-CPU locks */
static void update_pcpu_units(const struct scheduler *ops, multi_a653_pcpu_t *ma_cpu)
{
	unsigned int i;

	for (i = 0; i < ma_cpu->num_schedule_entries; i++)
		ma_cpu->schedule[i].unit = find_matching_unit(ops, ma_cpu->schedule[i].dom_handle,
							      ma_cpu->schedule[i].unit_id);
}

static int multi_a653_sched_set(const struct scheduler *ops, unsigned int cpu,
				struct xen_sysctl_arinc653_schedule *schedule)
{
	multi_a653_sched_priv_t *priv = ARINC653_MULTI_SCHED_PRIV(ops);
	multi_a653_pcpu_t *ma_cpu;
	s_time_t total = 0;
	unsigned long flags;
	spinlock_t *lock;

	/* Userspace-delivered table validation */
	if (schedule->major_frame <= 0 || schedule->num_sched_entries < 1 ||
	    schedule->num_sched_entries > ARINC653_MAX_DOMAINS_PER_SCHEDULE)
		return -EINVAL;

	/* Compute total valid runtime (of schedule entries) */
	for (int i = 0; i < schedule->num_sched_entries; i++) {
		if (schedule->sched_entries[i].runtime <= 0)
			return -EINVAL;

		total += schedule->sched_entries[i].runtime;
	}

	if (total > schedule->major_frame)
		return -EINVAL;

	/* obtain both global scheduler lock and per-CPU lock */
	lock = pcpu_schedule_lock_irqsave(cpu, &flags);
	spin_lock(&priv->lock);

	ma_cpu = ARINC653_MULTI_CPU(cpu);
	ma_cpu->num_schedule_entries = schedule->num_sched_entries;
	ma_cpu->major_frame = schedule->major_frame;

	/* Import all the schedule entries */
	for (int i = 0; i < schedule->num_sched_entries; i++) {
		memcpy(ma_cpu->schedule[i].dom_handle,
		       schedule->sched_entries[i].dom_handle,
		       sizeof(ma_cpu->schedule[i].dom_handle));

		ma_cpu->schedule[i].unit_id = schedule->sched_entries[i].vcpu_id;
		ma_cpu->schedule[i].runtime = schedule->sched_entries[i].runtime;
	}

	update_pcpu_units(ops, ma_cpu);

	/*
	 * Bindings are live now. However, do not let do_schedule run from
	 * these slots until every listed unit is homed on this CPU
	 */
	ma_cpu->armed = false;

	/* Release the locks after per-CPU table manipulation and global sched-table walks */
	spin_unlock(&priv->lock);
	pcpu_schedule_unlock_irqrestore(lock, flags, cpu);

	/*
	 * Phase-2: Attempt to re-home units to the current CPU
	 * Hand only those units to the core scheduler that do not
	 * currently have the current CPU as their master CPU
	 */
	for (int i = 0; i < ma_cpu->num_schedule_entries; i++) {
		struct sched_unit *u = ma_cpu->schedule[i].unit;

		if (!u || is_idle_unit(u) || (sched_unit_master(u) == cpu))
			continue;

		sched_unit_repick(u);
	}

	return 0;
}
#endif /* CONFIG_SYSCTL */

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

/* Set a default idle frame */
static void init_pdata(multi_a653_pcpu_t *ma653_cpu, unsigned int cpu)
{
	ma653_cpu->cpuid		=	cpu;
	ma653_cpu->num_schedule_entries	=	0;
	ma653_cpu->major_frame		=	DEFAULT_TIMESLICE;
	ma653_cpu->schedule[0].runtime	=	DEFAULT_TIMESLICE;
	ma653_cpu->next_major_frame	=	0;
	ma653_cpu->sched_index		=	0;
	ma653_cpu->next_switch_time	=	0;
	ma653_cpu->armed		=	false;
}

static spinlock_t *cf_check multi_a653_switch_sched(struct scheduler *new_ops,
						    unsigned int cpu, void *pdata,
						    void *vdata)
{
	struct sched_resource *sr = get_sched_res(cpu);
	const multi_a653_unit_t *ma_unit = vdata;
	multi_a653_pcpu_t *ma_cpu = pdata;

	ASSERT(ma_cpu && ma_unit && is_idle_unit(ma_unit->unit));

	init_pdata(ma_cpu, cpu);
	sched_idle_unit(cpu)->priv = vdata;

	return &sr->_lock;
}

static void cf_check multi_a653_do_sched(const struct scheduler *ops,
					 struct sched_unit *prev, s_time_t now,
					 bool tasklet_work_scheduled)
{
	const unsigned int cpu =
		sched_get_resource_cpu(smp_processor_id());
	multi_a653_pcpu_t *ma_cpu = ARINC653_MULTI_CPU(cpu);
	struct sched_unit *new_task;

	ASSERT(ma_cpu->major_frame > 0);

	/* Advance current cpu's own major-frame (hp) */
	if (now >= ma_cpu->next_major_frame) {
		s_time_t mf = ma_cpu->major_frame;
		s_time_t rem_time = (now - ma_cpu->next_major_frame) % mf;

		ma_cpu->sched_index = 0;
		ma_cpu->next_major_frame = (now - rem_time) + mf;
		ma_cpu->next_switch_time = (now - rem_time) +
					   ma_cpu->schedule[0].runtime;
	}

	while ((now >= ma_cpu->next_switch_time) &&
	       (++ma_cpu->sched_index < ma_cpu->num_schedule_entries))
		ma_cpu->next_switch_time +=
		ma_cpu->schedule[ma_cpu->sched_index].runtime;


	if (ma_cpu->sched_index >= ma_cpu->num_schedule_entries)
		ma_cpu->next_switch_time = ma_cpu->next_major_frame;

	/*
	 * An unarmed pCPU may hold a published schedule table whose
	 * units are still homed elsewhere. Do not let do_schedule run such
	 * entries until units corresponding to these entries are homed on
	 * the current CPU.
	 */
	new_task = IDLE_TASK(cpu);

	if (ma_cpu->armed &&
	    ma_cpu->sched_index < ma_cpu->num_schedule_entries)
		new_task = ma_cpu->schedule[ma_cpu->sched_index].unit;

	/* Check to see if the new task is in a valid runnable state */
	if (!((new_task != NULL) && ARINC653_MULTI_UNIT(new_task) != NULL &&
	    ARINC653_MULTI_UNIT(new_task)->awake &&
	    unit_runnable_state(new_task)))
		new_task = IDLE_TASK(cpu);

	BUG_ON(new_task == NULL);
	BUG_ON(now >= ma_cpu->next_major_frame);

	prev->next_time = ma_cpu->next_switch_time - now;

	/* Tasklet work (that runs in IDLE context) overrides everything */
	if (tasklet_work_scheduled)
		new_task = IDLE_TASK(cpu);

	/*
	 * NOTE: If we ever encounter a unit whose master-CPU isnt the CPU
	 * we are on at this stage (although should never happen for
	 * multi-ARINC653), to err on the side of caution, instead of
	 * triggering a migration, we let IDLE context take over
	 */
	if (!is_idle_unit(new_task) && (sched_unit_master(new_task) != cpu))
		new_task = IDLE_TASK(cpu);

	prev->next_task = new_task;
	new_task->migrated = false;

	BUG_ON(prev->next_time <= 0);
}

static void cf_check multi_a653_unit_sleep(const struct scheduler *ops,
					   struct sched_unit *unit)
{
	if (ARINC653_MULTI_UNIT(unit))
		ARINC653_MULTI_UNIT(unit)->awake = false;

	/*
	 * If the unit being put to sleep is the one that is running
	 * currently, raise a softirq to invoke the scheduler to switch
	 * DomUs
	 */
	if (get_sched_res(sched_unit_master(unit))->curr == unit)
		cpu_raise_softirq(sched_unit_master(unit), SCHEDULE_SOFTIRQ);
}

static void cf_check multi_a653_unit_wake(const struct scheduler *ops,
					  struct sched_unit *unit)
{
	if (ARINC653_MULTI_UNIT(unit) != NULL)
		ARINC653_MULTI_UNIT(unit)->awake = true;

	cpu_raise_softirq(sched_unit_master(unit), SCHEDULE_SOFTIRQ);
}

static struct sched_resource *cf_check multi_a653_pick_res(const struct scheduler *ops,
							   const struct sched_unit *unit)
{
	const cpumask_t *online;
	unsigned int cpu, scan_cpu;
	unsigned long flags;
	multi_a653_sched_priv_t *priv = ARINC653_MULTI_SCHED_PRIV(ops);

	online = cpupool_domain_master_cpumask(unit->domain);

	/* To scan through each pCPU table, we need to acquire global sched lock */
	spin_lock_irqsave(&priv->lock, flags);

	for_each_cpu(scan_cpu, online) {
		const multi_a653_pcpu_t *ma_cpu = ARINC653_MULTI_CPU(scan_cpu);

		for (int i = 0; i < ma_cpu->num_schedule_entries; i++) {
			/*
			 * If the binding is found, we know for certain
			 * that the current unit needs to have
			 * re-homing of its master CPU to the current
			 * CPU
			 */
			if (ma_cpu->schedule[i].unit == unit) {
				spin_unlock_irqrestore(&priv->lock, flags);
				return get_sched_res(scan_cpu);
			}
		}
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	/* Fallback for units whose binding is not present yet/not found */
	cpu = cpumask_first(online);

	/*
	 * If the current unit's master CPU belongs to `online` pool
	 * (or)
	 * `online` pool is empty
	 * Select current unit's master CPU as the resource to pick.
	 */
	if (cpumask_test_cpu(sched_unit_master(unit), online) ||
	   (cpu >= nr_cpu_ids))
		cpu = sched_unit_master(unit);

	return get_sched_res(cpu);
}

#ifdef CONFIG_SYSCTL
static int cf_check multi_a653_adjust_global(const struct scheduler *ops,
					     struct xen_sysctl_scheduler_op *sc)
{
	struct xen_sysctl_arinc653_schedule local_sched;
	unsigned int cpu = sc->u.sched_arinc653.cpu;
	int rc = -EINVAL;

	/* CPU must belong to this scheduler pool */
	if (cpu >= nr_cpu_ids || get_sched_res(cpu) == NULL ||
	    get_sched_res(cpu)->scheduler != ops)
		return rc;

	switch (sc->cmd)
	{
	case XEN_SYSCTL_SCHEDOP_putinfo:
		if (copy_from_guest(&local_sched, sc->u.sched_arinc653.schedule, 1)) {
			rc = -EFAULT;
			break;
		}

		rc = multi_a653_sched_set(ops, cpu, &local_sched);
		break;
	}

	return rc;
}
#endif

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

	.sleep		=		multi_a653_unit_sleep,
	.wake		=		multi_a653_unit_wake,

	.switch_sched	=		multi_a653_switch_sched,
	.do_schedule	=		multi_a653_do_sched,
	.pick_resource	=		multi_a653_pick_res,

#ifdef CONFIG_SYSCTL
	.adjust_global	=		multi_a653_adjust_global,
#endif
};

REGISTER_SCHEDULER(sched_arinc653_multi_def);
