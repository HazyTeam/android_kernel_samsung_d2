/*
 * kernel/workqueue.c - generic async execution with shared worker pool
 *
 * Copyright (C) 2002		Ingo Molnar
 *
 *   Derived from the taskqueue/keventd code by:
 *     David Woodhouse <dwmw2@infradead.org>
 *     Andrew Morton
 *     Kai Petzke <wpp@marie.physik.tu-berlin.de>
 *     Theodore Ts'o <tytso@mit.edu>
 *
 * Made to use alloc_percpu by Christoph Lameter.
 *
 * Copyright (C) 2010		SUSE Linux Products GmbH
 * Copyright (C) 2010		Tejun Heo <tj@kernel.org>
 *
 * This is the generic async execution mechanism.  Work items as are
 * executed in process context.  The worker pool is shared and
 * automatically managed.  There is one worker pool for each CPU and
 * one extra for works which are better served by workers which are
 * not bound to any specific CPU.
 *
 * Please read Documentation/workqueue.txt for details.
 */

#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/signal.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/notifier.h>
#include <linux/kthread.h>
#include <linux/hardirq.h>
#include <linux/mempolicy.h>
#include <linux/freezer.h>
#include <linux/kallsyms.h>
#include <linux/debug_locks.h>
#include <linux/lockdep.h>
#include <linux/idr.h>

#include "workqueue_sched.h"

enum {
	/* global_cwq flags */
	GCWQ_DISASSOCIATED	= 1 << 0,	/* cpu can't serve workers */
	GCWQ_FREEZING		= 1 << 1,	/* freeze in progress */

	/* pool flags */
	POOL_MANAGE_WORKERS	= 1 << 0,	/* need to manage workers */
	POOL_MANAGING_WORKERS	= 1 << 1,	/* managing workers */

	/* worker flags */
	WORKER_STARTED		= 1 << 0,	/* started */
	WORKER_DIE		= 1 << 1,	/* die die die */
	WORKER_IDLE		= 1 << 2,	/* is idle */
	WORKER_PREP		= 1 << 3,	/* preparing to run works */
	WORKER_ROGUE		= 1 << 4,	/* not bound to any cpu */
	WORKER_REBIND		= 1 << 5,	/* mom is home, come back */
	WORKER_CPU_INTENSIVE	= 1 << 6,	/* cpu intensive */
	WORKER_UNBOUND		= 1 << 7,	/* worker is unbound */

	WORKER_NOT_RUNNING	= WORKER_PREP | WORKER_ROGUE | WORKER_REBIND |
				  WORKER_CPU_INTENSIVE | WORKER_UNBOUND,

	/* gcwq->trustee_state */
	TRUSTEE_START		= 0,		/* start */
	TRUSTEE_IN_CHARGE	= 1,		/* trustee in charge of gcwq */
	TRUSTEE_BUTCHER		= 2,		/* butcher workers */
	TRUSTEE_RELEASE		= 3,		/* release workers */
	TRUSTEE_DONE		= 4,		/* trustee is done */

	NR_STD_WORKER_POOLS	= 2,		/* # standard pools per cpu */

	BUSY_WORKER_HASH_ORDER	= 6,		/* 64 pointers */

	MAX_IDLE_WORKERS_RATIO	= 4,		/* 1/4 of busy can be idle */
	IDLE_WORKER_TIMEOUT	= 300 * HZ,	/* keep idle ones for 5 mins */

	MAYDAY_INITIAL_TIMEOUT  = HZ / 100 >= 2 ? HZ / 100 : 2,
						/* call for help after 10ms
						   (min two ticks) */
	MAYDAY_INTERVAL		= HZ / 10,	/* and then every 100ms */
	CREATE_COOLDOWN		= HZ,		/* time to breath after fail */
	TRUSTEE_COOLDOWN	= HZ / 10,	/* for trustee draining */

	/*
	 * Rescue workers are used only on emergencies and shared by
	 * all cpus.  Give -20.
	 */
	RESCUER_NICE_LEVEL	= -20,
	HIGHPRI_NICE_LEVEL	= -20,

	WQ_NAME_LEN		= 24,
};

/*
 * Structure fields follow one of the following exclusion rules.
 *
 * I: Modifiable by initialization/destruction paths and read-only for
 *    everyone else.
 *
 * P: Preemption protected.  Disabling preemption is enough and should
 *    only be modified and accessed from the local cpu.
 *
 * L: gcwq->lock protected.  Access with gcwq->lock held.
 *
 * X: During normal operation, modification requires gcwq->lock and
 *    should be done only from local cpu.  Either disabling preemption
 *    on local cpu or grabbing gcwq->lock is enough for read access.
 *    If GCWQ_DISASSOCIATED is set, it's identical to L.
 *
 * F: wq->flush_mutex protected.
 *
 * W: workqueue_lock protected.
 */

struct global_cwq;
struct worker_pool;

/*
 * The poor guys doing the actual heavy lifting.  All on-duty workers
 * are either serving the manager role, on idle list or on busy hash.
 */
struct worker {
	/* on idle list while idle, on busy hash table while busy */
	union {
		struct list_head	entry;	/* L: while idle */
		struct hlist_node	hentry;	/* L: while busy */
	};

	struct work_struct	*current_work;	/* L: work being processed */
	work_func_t		current_func;	/* L: current_work's fn */
	struct cpu_workqueue_struct *current_cwq; /* L: current_work's cwq */
	struct list_head	scheduled;	/* L: scheduled works */
	struct task_struct	*task;		/* I: worker task */
	struct worker_pool	*pool;		/* I: the associated pool */
	/* 64 bytes boundary on 64bit, 32 on 32bit */
	unsigned long		last_active;	/* L: last active timestamp */
	unsigned int		flags;		/* X: flags */
	int			id;		/* I: worker id */
	struct work_struct	rebind_work;	/* L: rebind worker to cpu */
};

struct worker_pool {
	struct global_cwq	*gcwq;		/* I: the owning gcwq */
	unsigned int		flags;		/* X: flags */

	struct list_head	worklist;	/* L: list of pending works */
	int			nr_workers;	/* L: total number of workers */
	int			nr_idle;	/* L: currently idle ones */

	struct list_head	idle_list;	/* X: list of idle workers */
	struct timer_list	idle_timer;	/* L: worker idle timeout */
	struct timer_list	mayday_timer;	/* L: SOS timer for workers */

	struct ida		worker_ida;	/* L: for worker IDs */
	struct worker		*first_idle;	/* L: first idle worker */
};

/*
 * Global per-cpu workqueue.  There's one and only one for each cpu
 * and all works are queued and processed here regardless of their
 * target workqueues.
 */
struct global_cwq {
	spinlock_t		lock;		/* the gcwq lock */
	unsigned int		cpu;		/* I: the associated cpu */
	unsigned int		flags;		/* L: GCWQ_* flags */

	/* workers are chained either in busy_hash or pool idle_list */
	struct hlist_head	busy_hash[BUSY_WORKER_HASH_SIZE];
						/* L: hash of busy workers */

	struct worker_pool	pools[2];	/* normal and highpri pools */

	struct task_struct	*trustee;	/* L: for gcwq shutdown */
	unsigned int		trustee_state;	/* L: trustee state */
	wait_queue_head_t	trustee_wait;	/* trustee wait */
} ____cacheline_aligned_in_smp;

/*
 * The per-CPU workqueue.  The lower WORK_STRUCT_FLAG_BITS of
 * work_struct->data are used for flags and thus cwqs need to be
 * aligned at two's power of the number of flag bits.
 */
struct cpu_workqueue_struct {
	struct worker_pool	*pool;		/* I: the associated pool */
	struct workqueue_struct *wq;		/* I: the owning workqueue */
	int			work_color;	/* L: current color */
	int			flush_color;	/* L: flushing color */
	int			refcnt;		/* L: reference count */
	int			nr_in_flight[WORK_NR_COLORS];
						/* L: nr of in_flight works */
	int			nr_active;	/* L: nr of active works */
	int			max_active;	/* L: max active works */
	struct list_head	delayed_works;	/* L: delayed works */
	struct list_head	pwqs_node;	/* WR: node on wq->pwqs */
	struct list_head	mayday_node;	/* MD: node on wq->maydays */

	/*
	 * Release of unbound pwq is punted to system_wq.  See put_pwq()
	 * and pwq_unbound_release_workfn() for details.  pool_workqueue
	 * itself is also sched-RCU protected so that the first pwq can be
	 * determined without grabbing wq->mutex.
	 */
	struct work_struct	unbound_release_work;
	struct rcu_head		rcu;
} __aligned(1 << WORK_STRUCT_FLAG_BITS);

/*
 * Structure used to wait for workqueue flush.
 */
struct wq_flusher {
	struct list_head	list;		/* WQ: list of flushers */
	int			flush_color;	/* WQ: flush color waiting for */
	struct completion	done;		/* flush completion */
};

struct wq_device;

/*
 * The externally visible workqueue.  It relays the issued work items to
 * the appropriate worker_pool through its pool_workqueues.
 */
struct workqueue_struct {
	struct list_head	pwqs;		/* WR: all pwqs of this wq */
	struct list_head	list;		/* PL: list of all workqueues */

	struct mutex		mutex;		/* protects this wq */
	int			work_color;	/* WQ: current work color */
	int			flush_color;	/* WQ: current flush color */
	atomic_t		nr_pwqs_to_flush; /* flush in progress */
	struct wq_flusher	*first_flusher;	/* WQ: first flusher */
	struct list_head	flusher_queue;	/* WQ: flush waiters */
	struct list_head	flusher_overflow; /* WQ: flush overflow list */

	struct list_head	maydays;	/* MD: pwqs requesting rescue */
	struct worker		*rescuer;	/* I: rescue worker */

	int			nr_drainers;	/* WQ: drain in progress */
	int			saved_max_active; /* WQ: saved pwq max_active */

	struct workqueue_attrs	*unbound_attrs;	/* WQ: only for unbound wqs */
	struct pool_workqueue	*dfl_pwq;	/* WQ: only for unbound wqs */

#ifdef CONFIG_SYSFS
	struct wq_device	*wq_dev;	/* I: for sysfs interface */
#endif
#ifdef CONFIG_LOCKDEP
	struct lockdep_map	lockdep_map;
#endif
	char			name[WQ_NAME_LEN]; /* I: workqueue name */

	/* hot fields used during command issue, aligned to cacheline */
	unsigned int		flags ____cacheline_aligned; /* WQ: WQ_* flags */
	struct pool_workqueue __percpu *cpu_pwqs; /* I: per-cpu pwqs */
	struct pool_workqueue __rcu *numa_pwq_tbl[]; /* FR: unbound pwqs indexed by node */
};

static struct kmem_cache *pwq_cache;

static int wq_numa_tbl_len;		/* highest possible NUMA node id + 1 */
static cpumask_var_t *wq_numa_possible_cpumask;
					/* possible CPUs of each node */

static bool wq_disable_numa;
module_param_named(disable_numa, wq_disable_numa, bool, 0444);

static bool wq_numa_enabled;		/* unbound NUMA affinity enabled */

/* buf for wq_update_unbound_numa_attrs(), protected by CPU hotplug exclusion */
static struct workqueue_attrs *wq_update_unbound_numa_attrs_buf;

static DEFINE_MUTEX(wq_pool_mutex);	/* protects pools and workqueues list */
static DEFINE_SPINLOCK(wq_mayday_lock);	/* protects wq->maydays list */

static LIST_HEAD(workqueues);		/* PL: list of all workqueues */
static bool workqueue_freezing;		/* PL: have wqs started freezing? */

/* the per-cpu worker pools */
static DEFINE_PER_CPU_SHARED_ALIGNED(struct worker_pool [NR_STD_WORKER_POOLS],
				     cpu_worker_pools);

static DEFINE_IDR(worker_pool_idr);	/* PR: idr of all pools */

/* PL: hash of all unbound pools keyed by pool->attrs */
static DEFINE_HASHTABLE(unbound_pool_hash, UNBOUND_POOL_HASH_ORDER);

/* I: attributes used when instantiating standard unbound pools on demand */
static struct workqueue_attrs *unbound_std_wq_attrs[NR_STD_WORKER_POOLS];

struct workqueue_struct *system_wq __read_mostly;
EXPORT_SYMBOL(system_wq);
struct workqueue_struct *system_highpri_wq __read_mostly;
EXPORT_SYMBOL_GPL(system_highpri_wq);
struct workqueue_struct *system_long_wq __read_mostly;
EXPORT_SYMBOL_GPL(system_long_wq);
struct workqueue_struct *system_unbound_wq __read_mostly;
EXPORT_SYMBOL_GPL(system_unbound_wq);
struct workqueue_struct *system_freezable_wq __read_mostly;
EXPORT_SYMBOL_GPL(system_freezable_wq);

#define CREATE_TRACE_POINTS
#include <trace/events/workqueue.h>

#define for_each_worker_pool(pool, gcwq)				\
	for ((pool) = &(gcwq)->pools[0];				\
	     (pool) < &(gcwq)->pools[NR_WORKER_POOLS]; (pool)++)

#define for_each_std_worker_pool(pool, cpu)				\
	for ((pool) = &std_worker_pools(cpu)[0];			\
	     (pool) < &std_worker_pools(cpu)[NR_STD_WORKER_POOLS]; (pool)++)

#define for_each_busy_worker(worker, i, pool)				\
	hash_for_each(pool->busy_hash, i, worker, hentry)

static inline int __next_wq_cpu(int cpu, const struct cpumask *mask,
				unsigned int sw)
{
	if (cpu < nr_cpu_ids) {
		if (sw & 1) {
			cpu = cpumask_next(cpu, mask);
			if (cpu < nr_cpu_ids)
				return cpu;
		}
		if (sw & 2)
			return WORK_CPU_UNBOUND;
	}
	return WORK_CPU_END;
}

static inline int __next_pwq_cpu(int cpu, const struct cpumask *mask,
				 struct workqueue_struct *wq)
{
	return __next_wq_cpu(cpu, mask, !(wq->flags & WQ_UNBOUND) ? 1 : 2);
}

/*
 * CPU iterators
 *
 * An extra cpu number is defined using an invalid cpu number
 * (WORK_CPU_UNBOUND) to host workqueues which are not bound to any
 * specific CPU.  The following iterators are similar to for_each_*_cpu()
 * iterators but also considers the unbound CPU.
 *
 * for_each_wq_cpu()		: possible CPUs + WORK_CPU_UNBOUND
 * for_each_online_wq_cpu()	: online CPUs + WORK_CPU_UNBOUND
 * for_each_pwq_cpu()		: possible CPUs for bound workqueues,
 *				  WORK_CPU_UNBOUND for unbound workqueues
 */
#define for_each_wq_cpu(cpu)						\
	for ((cpu) = __next_wq_cpu(-1, cpu_possible_mask, 3);		\
	     (cpu) < WORK_CPU_END;					\
	     (cpu) = __next_wq_cpu((cpu), cpu_possible_mask, 3))

#define for_each_online_wq_cpu(cpu)					\
	for ((cpu) = __next_wq_cpu(-1, cpu_online_mask, 3);		\
	     (cpu) < WORK_CPU_END;					\
	     (cpu) = __next_wq_cpu((cpu), cpu_online_mask, 3))

#define for_each_pwq_cpu(cpu, wq)					\
	for ((cpu) = __next_pwq_cpu(-1, cpu_possible_mask, (wq));	\
	     (cpu) < WORK_CPU_END;					\
	     (cpu) = __next_pwq_cpu((cpu), cpu_possible_mask, (wq)))

#ifdef CONFIG_DEBUG_OBJECTS_WORK

static struct debug_obj_descr work_debug_descr;

static void *work_debug_hint(void *addr)
{
	return ((struct work_struct *) addr)->func;
}

/*
 * fixup_init is called when:
 * - an active object is initialized
 */
static int work_fixup_init(void *addr, enum debug_obj_state state)
{
	struct work_struct *work = addr;

	switch (state) {
	case ODEBUG_STATE_ACTIVE:
		cancel_work_sync(work);
		debug_object_init(work, &work_debug_descr);
		return 1;
	default:
		return 0;
	}
}

/*
 * fixup_activate is called when:
 * - an active object is activated
 * - an unknown object is activated (might be a statically initialized object)
 */
static int work_fixup_activate(void *addr, enum debug_obj_state state)
{
	struct work_struct *work = addr;

	switch (state) {

	case ODEBUG_STATE_NOTAVAILABLE:
		/*
		 * This is not really a fixup. The work struct was
		 * statically initialized. We just make sure that it
		 * is tracked in the object tracker.
		 */
		if (test_bit(WORK_STRUCT_STATIC_BIT, work_data_bits(work))) {
			debug_object_init(work, &work_debug_descr);
			debug_object_activate(work, &work_debug_descr);
			return 0;
		}
		WARN_ON_ONCE(1);
		return 0;

	case ODEBUG_STATE_ACTIVE:
		WARN_ON(1);

	default:
		return 0;
	}
}

/*
 * fixup_free is called when:
 * - an active object is freed
 */
static int work_fixup_free(void *addr, enum debug_obj_state state)
{
	struct work_struct *work = addr;

	switch (state) {
	case ODEBUG_STATE_ACTIVE:
		cancel_work_sync(work);
		debug_object_free(work, &work_debug_descr);
		return 1;
	default:
		return 0;
	}
}

static struct debug_obj_descr work_debug_descr = {
	.name		= "work_struct",
	.debug_hint	= work_debug_hint,
	.fixup_init	= work_fixup_init,
	.fixup_activate	= work_fixup_activate,
	.fixup_free	= work_fixup_free,
};

static inline void debug_work_activate(struct work_struct *work)
{
	debug_object_activate(work, &work_debug_descr);
}

static inline void debug_work_deactivate(struct work_struct *work)
{
	debug_object_deactivate(work, &work_debug_descr);
}

void __init_work(struct work_struct *work, int onstack)
{
	if (onstack)
		debug_object_init_on_stack(work, &work_debug_descr);
	else
		debug_object_init(work, &work_debug_descr);
}
EXPORT_SYMBOL_GPL(__init_work);

void destroy_work_on_stack(struct work_struct *work)
{
	debug_object_free(work, &work_debug_descr);
}
EXPORT_SYMBOL_GPL(destroy_work_on_stack);

#else
static inline void debug_work_activate(struct work_struct *work) { }
static inline void debug_work_deactivate(struct work_struct *work) { }
#endif

/* allocate ID and assign it to @pool */
static int worker_pool_assign_id(struct worker_pool *pool)
{
	int ret;

	lockdep_assert_held(&wq_pool_mutex);

	ret = idr_alloc(&worker_pool_idr, pool, 0, 0, GFP_KERNEL);
	if (ret >= 0) {
		pool->id = ret;
		return 0;
	}
	return ret;
}

/**
 * unbound_pwq_by_node - return the unbound pool_workqueue for the given node
 * @wq: the target workqueue
 * @node: the node ID
 *
 * This must be called either with pwq_lock held or sched RCU read locked.
 * If the pwq needs to be used beyond the locking in effect, the caller is
 * responsible for guaranteeing that the pwq stays online.
 */
static struct pool_workqueue *unbound_pwq_by_node(struct workqueue_struct *wq,
						  int node)
{
	assert_rcu_or_wq_mutex(wq);
	return rcu_dereference_raw(wq->numa_pwq_tbl[node]);
}

static unsigned int work_color_to_flags(int color)
{
	return color << WORK_STRUCT_COLOR_SHIFT;
}

static int get_work_color(struct work_struct *work)
{
	return (*work_data_bits(work) >> WORK_STRUCT_COLOR_SHIFT) &
		((1 << WORK_STRUCT_COLOR_BITS) - 1);
}

static int work_next_color(int color)
{
	return (color + 1) % WORK_NR_COLORS;
}

/*
 * While queued, %WORK_STRUCT_PWQ is set and non flag bits of a work's data
 * contain the pointer to the queued pwq.  Once execution starts, the flag
 * is cleared and the high bits contain OFFQ flags and pool ID.
 *
 * set_work_pwq(), set_work_pool_and_clear_pending(), mark_work_canceling()
 * and clear_work_data() can be used to set the pwq, pool or clear
 * work->data.  These functions should only be called while the work is
 * owned - ie. while the PENDING bit is set.
 *
 * get_work_pool() and get_work_pwq() can be used to obtain the pool or pwq
 * corresponding to a work.  Pool is available once the work has been
 * queued anywhere after initialization until it is sync canceled.  pwq is
 * available only while the work item is queued.
 *
 * %WORK_OFFQ_CANCELING is used to mark a work item which is being
 * canceled.  While being canceled, a work item may have its PENDING set
 * but stay off timer and worklist for arbitrarily long and nobody should
 * try to steal the PENDING bit.
 */
static inline void set_work_data(struct work_struct *work, unsigned long data,
				 unsigned long flags)
{
	WARN_ON_ONCE(!work_pending(work));
	atomic_long_set(&work->data, data | flags | work_static(work));
}

static void set_work_pwq(struct work_struct *work, struct pool_workqueue *pwq,
			 unsigned long extra_flags)
{
	set_work_data(work, (unsigned long)pwq,
		      WORK_STRUCT_PENDING | WORK_STRUCT_PWQ | extra_flags);
}

static void set_work_pool_and_keep_pending(struct work_struct *work,
					   int pool_id)
{
	set_work_data(work, (unsigned long)pool_id << WORK_OFFQ_POOL_SHIFT,
		      WORK_STRUCT_PENDING);
}

static void set_work_pool_and_clear_pending(struct work_struct *work,
					    int pool_id)
{
	/*
	 * The following wmb is paired with the implied mb in
	 * test_and_set_bit(PENDING) and ensures all updates to @work made
	 * here are visible to and precede any updates by the next PENDING
	 * owner.
	 */
	smp_wmb();
	set_work_data(work, (unsigned long)cpu << WORK_OFFQ_CPU_SHIFT, 0);
}

static void clear_work_data(struct work_struct *work)
{
	smp_wmb();	/* see set_work_cpu_and_clear_pending() */
	set_work_data(work, WORK_STRUCT_NO_CPU, 0);
}

static struct cpu_workqueue_struct *get_work_cwq(struct work_struct *work)
{
	unsigned long data = atomic_long_read(&work->data);

	if (data & WORK_STRUCT_CWQ)
		return (void *)(data & WORK_STRUCT_WQ_DATA_MASK);
	else
	{
		WARN_ON_ONCE(1);
		return NULL;
	}
}

static struct global_cwq *get_work_gcwq(struct work_struct *work)
{
	unsigned long data = atomic_long_read(&work->data);
	unsigned int cpu;

	if (data & WORK_STRUCT_CWQ)
		return ((struct cpu_workqueue_struct *)
			(data & WORK_STRUCT_WQ_DATA_MASK))->pool->gcwq;

	cpu = data >> WORK_OFFQ_CPU_SHIFT;
	if (cpu == WORK_CPU_NONE)
		return NULL;

	BUG_ON(cpu >= nr_cpu_ids && cpu != WORK_CPU_UNBOUND);
	return get_gcwq(cpu);
}

static void mark_work_canceling(struct work_struct *work)
{
	struct global_cwq *gcwq = get_work_gcwq(work);
	unsigned long cpu = gcwq ? gcwq->cpu : WORK_CPU_NONE;

	set_work_data(work, (cpu << WORK_OFFQ_CPU_SHIFT) | WORK_OFFQ_CANCELING,
		      WORK_STRUCT_PENDING);
}

static bool work_is_canceling(struct work_struct *work)
{
	unsigned long data = atomic_long_read(&work->data);

	return !(data & WORK_STRUCT_CWQ) && (data & WORK_OFFQ_CANCELING);
}

/*
 * Policy functions.  These define the policies on how the global worker
 * pools are managed.  Unless noted otherwise, these functions assume that
 * they're being called with gcwq->lock held.
 */

static bool __need_more_worker(struct worker_pool *pool)
{
	return !atomic_read(get_pool_nr_running(pool));
}

/*
 * Need to wake up a worker?  Called from anything but currently
 * running workers.
 *
 * Note that, because unbound workers never contribute to nr_running, this
 * function will always return %true for unbound pools as long as the
 * worklist isn't empty.
 */
static bool need_more_worker(struct worker_pool *pool)
{
	return !list_empty(&pool->worklist) && __need_more_worker(pool);
}

/* Can I start working?  Called from busy but !running workers. */
static bool may_start_working(struct worker_pool *pool)
{
	return pool->nr_idle;
}

/* Do I need to keep working?  Called from currently running workers. */
static bool keep_working(struct worker_pool *pool)
{
	return !list_empty(&pool->worklist) &&
		atomic_read(&pool->nr_running) <= 1;
}

/* Do we need a new worker?  Called from manager. */
static bool need_to_create_worker(struct worker_pool *pool)
{
	return need_more_worker(pool) && !may_start_working(pool);
}

/* Do I need to be the manager? */
static bool need_to_manage_workers(struct worker_pool *pool)
{
	return need_to_create_worker(pool) ||
		(pool->flags & POOL_MANAGE_WORKERS);
}

/* Do we have too many workers and should some go away? */
static bool too_many_workers(struct worker_pool *pool)
{
	bool managing = mutex_is_locked(&pool->manager_arb);
	int nr_idle = pool->nr_idle + managing; /* manager is considered idle */
	int nr_busy = pool->nr_workers - nr_idle;

	/*
	 * nr_idle and idle_list may disagree if idle rebinding is in
	 * progress.  Never return %true if idle_list is empty.
	 */
	if (list_empty(&pool->idle_list))
		return false;

	return nr_idle > 2 && (nr_idle - 2) * MAX_IDLE_WORKERS_RATIO >= nr_busy;
}

/*
 * Wake up functions.
 */

/* Return the first worker.  Safe with preemption disabled */
static struct worker *first_worker(struct worker_pool *pool)
{
	if (unlikely(list_empty(&pool->idle_list)))
		return NULL;

	return list_first_entry(&pool->idle_list, struct worker, entry);
}

/**
 * wake_up_worker - wake up an idle worker
 * @pool: worker pool to wake worker from
 *
 * Wake up the first idle worker of @pool.
 *
 * CONTEXT:
 * spin_lock_irq(pool->lock).
 */
static void wake_up_worker(struct worker_pool *pool)
{
	struct worker *worker = first_worker(pool);

	if (likely(worker))
		wake_up_process(worker->task);
}

/**
 * wq_worker_waking_up - a worker is waking up
 * @task: task waking up
 * @cpu: CPU @task is waking up to
 *
 * This function is called during try_to_wake_up() when a worker is
 * being awoken.
 *
 * CONTEXT:
 * spin_lock_irq(rq->lock)
 */
void wq_worker_waking_up(struct task_struct *task, int cpu)
{
	struct worker *worker = kthread_data(task);

	if (!(worker->flags & WORKER_NOT_RUNNING)) {
		WARN_ON_ONCE(worker->pool->cpu != cpu);
		atomic_inc(&worker->pool->nr_running);
	}
}

/**
 * wq_worker_sleeping - a worker is going to sleep
 * @task: task going to sleep
 * @cpu: CPU in question, must be the current CPU number
 *
 * This function is called during schedule() when a busy worker is
 * going to sleep.  Worker on the same cpu can be woken up by
 * returning pointer to its task.
 *
 * CONTEXT:
 * spin_lock_irq(rq->lock)
 *
 * RETURNS:
 * Worker task on @cpu to wake up, %NULL if none.
 */
struct task_struct *wq_worker_sleeping(struct task_struct *task, int cpu)
{
	struct worker *worker = kthread_data(task), *to_wakeup = NULL;
	struct worker_pool *pool;

	/*
	 * Rescuers, which may not have all the fields set up like normal
	 * workers, also reach here, let's not access anything before
	 * checking NOT_RUNNING.
	 */
	if (worker->flags & WORKER_NOT_RUNNING)
		return NULL;

	pool = worker->pool;

	/* this can only happen on the local cpu */
	if (WARN_ON_ONCE(cpu != raw_smp_processor_id()))
		return NULL;

	/*
	 * The counterpart of the following dec_and_test, implied mb,
	 * worklist not empty test sequence is in insert_work().
	 * Please read comment there.
	 *
	 * NOT_RUNNING is clear.  This means that trustee is not in
	 * charge and we're running on the local cpu w/ rq lock held
	 * and preemption disabled, which in turn means that none else
	 * could be manipulating idle_list, so dereferencing idle_list
	 * without gcwq lock is safe.
	 */
	if (atomic_dec_and_test(nr_running) && !list_empty(&pool->worklist))
		to_wakeup = first_worker(pool);
	return to_wakeup ? to_wakeup->task : NULL;
}

/**
 * worker_set_flags - set worker flags and adjust nr_running accordingly
 * @worker: self
 * @flags: flags to set
 * @wakeup: wakeup an idle worker if necessary
 *
 * Set @flags in @worker->flags and adjust nr_running accordingly.  If
 * nr_running becomes zero and @wakeup is %true, an idle worker is
 * woken up.
 *
 * CONTEXT:
 * spin_lock_irq(pool->lock)
 */
static inline void worker_set_flags(struct worker *worker, unsigned int flags,
				    bool wakeup)
{
	struct worker_pool *pool = worker->pool;

	WARN_ON_ONCE(worker->task != current);

	/*
	 * If transitioning into NOT_RUNNING, adjust nr_running and
	 * wake up an idle worker as necessary if requested by
	 * @wakeup.
	 */
	if ((flags & WORKER_NOT_RUNNING) &&
	    !(worker->flags & WORKER_NOT_RUNNING)) {
		if (wakeup) {
			if (atomic_dec_and_test(&pool->nr_running) &&
			    !list_empty(&pool->worklist))
				wake_up_worker(pool);
		} else
			atomic_dec(&pool->nr_running);
	}

	worker->flags |= flags;
}

/**
 * worker_clr_flags - clear worker flags and adjust nr_running accordingly
 * @worker: self
 * @flags: flags to clear
 *
 * Clear @flags in @worker->flags and adjust nr_running accordingly.
 *
 * CONTEXT:
 * spin_lock_irq(pool->lock)
 */
static inline void worker_clr_flags(struct worker *worker, unsigned int flags)
{
	struct worker_pool *pool = worker->pool;
	unsigned int oflags = worker->flags;

	WARN_ON_ONCE(worker->task != current);

	worker->flags &= ~flags;

	/*
	 * If transitioning out of NOT_RUNNING, increment nr_running.  Note
	 * that the nested NOT_RUNNING is not a noop.  NOT_RUNNING is mask
	 * of multiple flags, not a single flag.
	 */
	if ((flags & WORKER_NOT_RUNNING) && (oflags & WORKER_NOT_RUNNING))
		if (!(worker->flags & WORKER_NOT_RUNNING))
			atomic_inc(get_pool_nr_running(pool));
}

/**
 * busy_worker_head - return the busy hash head for a work
 * @gcwq: gcwq of interest
 * @work: work to be hashed
 *
 * Return hash head of @gcwq for @work.
 *
 * CONTEXT:
 * spin_lock_irq(gcwq->lock).
 *
 * RETURNS:
 * Pointer to the hash head.
 */
static struct hlist_head *busy_worker_head(struct global_cwq *gcwq,
					   struct work_struct *work)
{
	const int base_shift = ilog2(sizeof(struct work_struct));
	unsigned long v = (unsigned long)work;

	/* simple shift and fold hash, do we need something better? */
	v >>= base_shift;
	v += v >> BUSY_WORKER_HASH_ORDER;
	v &= BUSY_WORKER_HASH_MASK;

	return &gcwq->busy_hash[v];
}

/**
 * __find_worker_executing_work - find worker which is executing a work
 * @gcwq: gcwq of interest
 * @bwh: hash head as returned by busy_worker_head()
 * @work: work to find worker for
 *
 * Find a worker which is executing @work on @gcwq.  @bwh should be
 * the hash head obtained by calling busy_worker_head() with the same
 * work.
 *
 * CONTEXT:
 * spin_lock_irq(gcwq->lock).
 *
 * RETURNS:
 * Pointer to worker which is executing @work if found, NULL
 * otherwise.
 */
static struct worker *__find_worker_executing_work(struct global_cwq *gcwq,
						   struct hlist_head *bwh,
						   struct work_struct *work)
{
	struct worker *worker;
	struct hlist_node *tmp;

	hlist_for_each_entry(worker, tmp, bwh, hentry)
		if (worker->current_work == work &&
		    worker->current_func == work->func)
			return worker;
	return NULL;
}

/**
 * find_worker_executing_work - find worker which is executing a work
 * @gcwq: gcwq of interest
 * @work: work to find worker for
 *
 * Find a worker which is executing @work on @gcwq by searching
 * @gcwq->busy_hash which is keyed by the address of @work.  For a worker
 * to match, its current execution should match the address of @work and
 * its work function.  This is to avoid unwanted dependency between
 * unrelated work executions through a work item being recycled while still
 * being executed.
 *
 * This is a bit tricky.  A work item may be freed once its execution
 * starts and nothing prevents the freed area from being recycled for
 * another work item.  If the same work item address ends up being reused
 * before the original execution finishes, workqueue will identify the
 * recycled work item as currently executing and make it wait until the
 * current execution finishes, introducing an unwanted dependency.
 *
 * This function checks the work item address and work function to avoid
 * false positives.  Note that this isn't complete as one may construct a
 * work function which can introduce dependency onto itself through a
 * recycled work item.  Well, if somebody wants to shoot oneself in the
 * foot that badly, there's only so much we can do, and if such deadlock
 * actually occurs, it should be easy to locate the culprit work function.
 *
 * CONTEXT:
 * spin_lock_irq(gcwq->lock).
 *
 * RETURNS:
 * Pointer to worker which is executing @work if found, NULL
 * otherwise.
 */
static struct worker *find_worker_executing_work(struct global_cwq *gcwq,
						 struct work_struct *work)
{
	return __find_worker_executing_work(gcwq, busy_worker_head(gcwq, work),
					    work);
}

/**
 * move_linked_works - move linked works to a list
 * @work: start of series of works to be scheduled
 * @head: target list to append @work to
 * @nextp: out paramter for nested worklist walking
 *
 * Schedule linked works starting from @work to @head.  Work series to
 * be scheduled starts at @work and includes any consecutive work with
 * WORK_STRUCT_LINKED set in its predecessor.
 *
 * If @nextp is not NULL, it's updated to point to the next work of
 * the last scheduled work.  This allows move_linked_works() to be
 * nested inside outer list_for_each_entry_safe().
 *
 * CONTEXT:
 * spin_lock_irq(pool->lock).
 */
static void move_linked_works(struct work_struct *work, struct list_head *head,
			      struct work_struct **nextp)
{
	struct work_struct *n;

	/*
	 * Linked worklist will always end before the end of the list,
	 * use NULL for list head.
	 */
	list_for_each_entry_safe_from(work, n, NULL, entry) {
		list_move_tail(&work->entry, head);
		if (!(*work_data_bits(work) & WORK_STRUCT_LINKED))
			break;
	}

	/*
	 * If we're already inside safe list traversal and have moved
	 * multiple works to the scheduled queue, the next position
	 * needs to be updated.
	 */
	if (nextp)
		*nextp = n;
}

/**
 * get_pwq - get an extra reference on the specified pool_workqueue
 * @pwq: pool_workqueue to get
 *
 * Obtain an extra reference on @pwq.  The caller should guarantee that
 * @pwq has positive refcnt and be holding the matching pool->lock.
 */
static void get_pwq(struct pool_workqueue *pwq)
{
	lockdep_assert_held(&pwq->pool->lock);
	WARN_ON_ONCE(pwq->refcnt <= 0);
	pwq->refcnt++;
}

/**
 * put_pwq - put a pool_workqueue reference
 * @pwq: pool_workqueue to put
 *
 * Drop a reference of @pwq.  If its refcnt reaches zero, schedule its
 * destruction.  The caller should be holding the matching pool->lock.
 */
static void put_pwq(struct pool_workqueue *pwq)
{
	lockdep_assert_held(&pwq->pool->lock);
	if (likely(--pwq->refcnt))
		return;
	if (WARN_ON_ONCE(!(pwq->wq->flags & WQ_UNBOUND)))
		return;
	/*
	 * @pwq can't be released under pool->lock, bounce to
	 * pwq_unbound_release_workfn().  This never recurses on the same
	 * pool->lock as this path is taken only for unbound workqueues and
	 * the release work item is scheduled on a per-cpu workqueue.  To
	 * avoid lockdep warning, unbound pool->locks are given lockdep
	 * subclass of 1 in get_unbound_pool().
	 */
	schedule_work(&pwq->unbound_release_work);
}

/**
 * put_pwq_unlocked - put_pwq() with surrounding pool lock/unlock
 * @pwq: pool_workqueue to put (can be %NULL)
 *
 * put_pwq() with locking.  This function also allows %NULL @pwq.
 */
static void put_pwq_unlocked(struct pool_workqueue *pwq)
{
	if (pwq) {
		/*
		 * As both pwqs and pools are sched-RCU protected, the
		 * following lock operations are safe.
		 */
		spin_lock_irq(&pwq->pool->lock);
		put_pwq(pwq);
		spin_unlock_irq(&pwq->pool->lock);
	}
}

static void pwq_activate_delayed_work(struct work_struct *work)
{
	struct pool_workqueue *pwq = get_work_pwq(work);

	trace_workqueue_activate_work(work);
	move_linked_works(work, &pwq->pool->worklist, NULL);
	__clear_bit(WORK_STRUCT_DELAYED_BIT, work_data_bits(work));
	pwq->nr_active++;
}

static void pwq_activate_first_delayed(struct pool_workqueue *pwq)
{
	struct work_struct *work = list_first_entry(&pwq->delayed_works,
						    struct work_struct, entry);

	pwq_activate_delayed_work(work);
}

/**
 * pwq_dec_nr_in_flight - decrement pwq's nr_in_flight
 * @pwq: pwq of interest
 * @color: color of work which left the queue
 *
 * A work either has completed or is removed from pending queue,
 * decrement nr_in_flight of its pwq and handle workqueue flushing.
 *
 * CONTEXT:
 * spin_lock_irq(pool->lock).
 */
static void pwq_dec_nr_in_flight(struct pool_workqueue *pwq, int color)
{
	/* uncolored work items don't participate in flushing or nr_active */
	if (color == WORK_NO_COLOR)
		goto out_put;

	pwq->nr_in_flight[color]--;

	pwq->nr_active--;
	if (!list_empty(&pwq->delayed_works)) {
		/* one down, submit a delayed one */
		if (pwq->nr_active < pwq->max_active)
			pwq_activate_first_delayed(pwq);
	}

	/* is flush in progress and are we at the flushing tip? */
	if (likely(pwq->flush_color != color))
		goto out_put;

	/* are there still in-flight works? */
	if (pwq->nr_in_flight[color])
		goto out_put;

	/* this pwq is done, clear flush_color */
	pwq->flush_color = -1;

	/*
	 * If this was the last pwq, wake up the first flusher.  It
	 * will handle the rest.
	 */
	if (atomic_dec_and_test(&pwq->wq->nr_pwqs_to_flush))
		complete(&pwq->wq->first_flusher->done);
out_put:
	put_pwq(pwq);
}

/**
 * try_to_grab_pending - steal work item from worklist and disable irq
 * @work: work item to steal
 * @is_dwork: @work is a delayed_work
 * @flags: place to store irq state
 *
 * Try to grab PENDING bit of @work.  This function can handle @work in any
 * stable state - idle, on timer or on worklist.  Return values are
 *
 *  1		if @work was pending and we successfully stole PENDING
 *  0		if @work was idle and we claimed PENDING
 *  -EAGAIN	if PENDING couldn't be grabbed at the moment, safe to busy-retry
 *  -ENOENT	if someone else is canceling @work, this state may persist
 *		for arbitrarily long
 *
 * On >= 0 return, the caller owns @work's PENDING bit.  To avoid getting
 * interrupted while holding PENDING and @work off queue, irq must be
 * disabled on entry.  This, combined with delayed_work->timer being
 * irqsafe, ensures that we return -EAGAIN for finite short period of time.
 *
 * On successful return, >= 0, irq is disabled and the caller is
 * responsible for releasing it using local_irq_restore(*@flags).
 *
 * This function is safe to call from any context including IRQ handler.
 */
static int try_to_grab_pending(struct work_struct *work, bool is_dwork,
			       unsigned long *flags)
{
	struct worker_pool *pool;
	struct pool_workqueue *pwq;

	local_irq_save(*flags);

	/* try to steal the timer if it exists */
	if (is_dwork) {
		struct delayed_work *dwork = to_delayed_work(work);

		/*
		 * dwork->timer is irqsafe.  If del_timer() fails, it's
		 * guaranteed that the timer is not queued anywhere and not
		 * running on the local CPU.
		 */
		if (likely(del_timer(&dwork->timer)))
			return 1;
	}

	/* try to claim PENDING the normal way */
	if (!test_and_set_bit(WORK_STRUCT_PENDING_BIT, work_data_bits(work)))
		return 0;

	/*
	 * The queueing is in progress, or it is already queued. Try to
	 * steal it from ->worklist without clearing WORK_STRUCT_PENDING.
	 */
	pool = get_work_pool(work);
	if (!pool)
		goto fail;

	spin_lock(&pool->lock);
	/*
	 * work->data is guaranteed to point to pwq only while the work
	 * item is queued on pwq->wq, and both updating work->data to point
	 * to pwq on queueing and to pool on dequeueing are done under
	 * pwq->pool->lock.  This in turn guarantees that, if work->data
	 * points to pwq which is associated with a locked pool, the work
	 * item is currently queued on that pool.
	 */
	pwq = get_work_pwq(work);
	if (pwq && pwq->pool == pool) {
		debug_work_deactivate(work);

		/*
		 * A delayed work item cannot be grabbed directly because
		 * it might have linked NO_COLOR work items which, if left
		 * on the delayed_list, will confuse pwq->nr_active
		 * management later on and cause stall.  Make sure the work
		 * item is activated before grabbing.
		 */
		if (*work_data_bits(work) & WORK_STRUCT_DELAYED)
			pwq_activate_delayed_work(work);

		list_del_init(&work->entry);
		pwq_dec_nr_in_flight(get_work_pwq(work), get_work_color(work));

		/* work->data points to pwq iff queued, point to pool */
		set_work_pool_and_keep_pending(work, pool->id);

		spin_unlock(&pool->lock);
		return 1;
	}
	spin_unlock(&pool->lock);
fail:
	local_irq_restore(*flags);
	if (work_is_canceling(work))
		return -ENOENT;
	cpu_relax();
	return -EAGAIN;
}

/**
 * insert_work - insert a work into a pool
 * @pwq: pwq @work belongs to
 * @work: work to insert
 * @head: insertion point
 * @extra_flags: extra WORK_STRUCT_* flags to set
 *
 * Insert @work which belongs to @pwq after @head.  @extra_flags is or'd to
 * work_struct flags.
 *
 * CONTEXT:
 * spin_lock_irq(pool->lock).
 */
static void insert_work(struct pool_workqueue *pwq, struct work_struct *work,
			struct list_head *head, unsigned int extra_flags)
{
	struct worker_pool *pool = pwq->pool;

	/* we own @work, set data and link */
	set_work_pwq(work, pwq, extra_flags);
	list_add_tail(&work->entry, head);
	get_pwq(pwq);

	/*
	 * Ensure either wq_worker_sleeping() sees the above
	 * list_add_tail() or we see zero nr_running to avoid workers lying
	 * around lazily while there are works to be processed.
	 */
	smp_mb();

	if (__need_more_worker(pool))
		wake_up_worker(pool);
}

/*
 * Test whether @work is being queued from another work executing on the
 * same workqueue.
 */
static bool is_chained_work(struct workqueue_struct *wq)
{
	struct worker *worker;

	worker = current_wq_worker();
	/*
	 * Return %true iff I'm a worker execuing a work item on @wq.  If
	 * I'm @worker, it's safe to dereference it without locking.
	 */
	return worker && worker->current_pwq->wq == wq;
}

static void __queue_work(int cpu, struct workqueue_struct *wq,
			 struct work_struct *work)
{
	struct pool_workqueue *pwq;
	struct worker_pool *last_pool;
	struct list_head *worklist;
	unsigned int work_flags;
	unsigned int req_cpu = cpu;

	/*
	 * While a work item is PENDING && off queue, a task trying to
	 * steal the PENDING will busy-loop waiting for it to either get
	 * queued or lose PENDING.  Grabbing PENDING and queueing should
	 * happen with IRQ disabled.
	 */
	WARN_ON_ONCE(!irqs_disabled());

	debug_work_activate(work);

	/* if dying, only works from the same workqueue are allowed */
	if (unlikely(wq->flags & WQ_DRAINING) &&
	    WARN_ON_ONCE(!is_chained_work(wq)))
		return;

	/* determine the pwq to use */
	if (!(wq->flags & WQ_UNBOUND)) {
		struct worker_pool *last_pool;

		if (cpu == WORK_CPU_UNBOUND)
			cpu = raw_smp_processor_id();

		/*
		 * It's multi cpu.  If @work was previously on a different
		 * cpu, it might still be running there, in which case the
		 * work needs to be queued on that cpu to guarantee
		 * non-reentrancy.
		 */
		gcwq = get_gcwq(cpu);
		last_gcwq = get_work_gcwq(work);

		if (last_gcwq && last_gcwq != gcwq) {
			struct worker *worker;

			spin_lock(&last_gcwq->lock);

			worker = find_worker_executing_work(last_gcwq, work);

			if (worker && worker->current_cwq->wq == wq)
				gcwq = last_gcwq;
			else {
				/* meh... not running there, queue here */
				spin_unlock(&last_gcwq->lock);
				spin_lock(&gcwq->lock);
			}
		} else {
			spin_lock(&gcwq->lock);
		}
	} else {
		gcwq = get_gcwq(WORK_CPU_UNBOUND);
		spin_lock(&gcwq->lock);
	}

	/* gcwq determined, get cwq and queue */
	cwq = get_cwq(gcwq->cpu, wq);
	trace_workqueue_queue_work(req_cpu, cwq, work);

	if (WARN_ON(!list_empty(&work->entry))) {
		spin_unlock(&gcwq->lock);
		return;
	}

	cwq->nr_in_flight[cwq->work_color]++;
	work_flags = work_color_to_flags(cwq->work_color);

	if (likely(cwq->nr_active < cwq->max_active)) {
		trace_workqueue_activate_work(work);
		cwq->nr_active++;
		worklist = &cwq->pool->worklist;
	} else {
		work_flags |= WORK_STRUCT_DELAYED;
		worklist = &cwq->delayed_works;
	}

	insert_work(cwq, work, worklist, work_flags);

	spin_unlock_irqrestore(&gcwq->lock, flags);
}

/**
 * queue_work - queue work on a workqueue
 * @wq: workqueue to use
 * @work: work to queue
 *
 * Returns 0 if @work was already on a queue, non-zero otherwise.
 *
 * We queue the work to the CPU on which it was submitted, but if the CPU dies
 * it can be processed by another CPU.
 */
int queue_work(struct workqueue_struct *wq, struct work_struct *work)
{
	int ret;

	ret = queue_work_on(get_cpu(), wq, work);
	put_cpu();

	return ret;
}
EXPORT_SYMBOL_GPL(queue_work);

/**
 * queue_work_on - queue work on specific cpu
 * @cpu: CPU number to execute work on
 * @wq: workqueue to use
 * @work: work to queue
 *
 * Returns 0 if @work was already on a queue, non-zero otherwise.
 *
 * We queue the work to a specific CPU, the caller must ensure it
 * can't go away.
 */
int
queue_work_on(int cpu, struct workqueue_struct *wq, struct work_struct *work)
{
	int ret = 0;

	if (!test_and_set_bit(WORK_STRUCT_PENDING_BIT, work_data_bits(work))) {
		__queue_work(cpu, wq, work);
		ret = 1;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(queue_work_on);

static void delayed_work_timer_fn(unsigned long __data)
{
	struct delayed_work *dwork = (struct delayed_work *)__data;
	struct cpu_workqueue_struct *cwq = get_work_cwq(&dwork->work);

	if (cwq != NULL)
		__queue_work(smp_processor_id(), cwq->wq, &dwork->work);
}

/**
 * queue_delayed_work - queue work on a workqueue after delay
 * @wq: workqueue to use
 * @dwork: delayable work to queue
 * @delay: number of jiffies to wait before queueing
 *
 * Returns 0 if @work was already on a queue, non-zero otherwise.
 */
int queue_delayed_work(struct workqueue_struct *wq,
			struct delayed_work *dwork, unsigned long delay)
{
	if (delay == 0)
		return queue_work(wq, &dwork->work);

	return queue_delayed_work_on(-1, wq, dwork, delay);
}
EXPORT_SYMBOL_GPL(queue_delayed_work);

/**
 * queue_delayed_work_on - queue work on specific CPU after delay
 * @cpu: CPU number to execute work on
 * @wq: workqueue to use
 * @dwork: work to queue
 * @delay: number of jiffies to wait before queueing
 *
 * Returns 0 if @work was already on a queue, non-zero otherwise.
 */
int queue_delayed_work_on(int cpu, struct workqueue_struct *wq,
			struct delayed_work *dwork, unsigned long delay)
{
	int ret = 0;
	struct timer_list *timer = &dwork->timer;
	struct work_struct *work = &dwork->work;

	if (!test_and_set_bit(WORK_STRUCT_PENDING_BIT, work_data_bits(work))) {
		unsigned int lcpu;

		WARN_ON_ONCE(timer_pending(timer));
		WARN_ON_ONCE(!list_empty(&work->entry));

		timer_stats_timer_set_start_info(&dwork->timer);

		/*
		 * This stores cwq for the moment, for the timer_fn.
		 * Note that the work's gcwq is preserved to allow
		 * reentrance detection for delayed works.
		 */
		if (!(wq->flags & WQ_UNBOUND)) {
			struct global_cwq *gcwq = get_work_gcwq(work);

			if (gcwq && gcwq->cpu != WORK_CPU_UNBOUND)
				lcpu = gcwq->cpu;
			else
				lcpu = raw_smp_processor_id();
		} else
			lcpu = WORK_CPU_UNBOUND;

		set_work_cwq(work, get_cwq(lcpu, wq), 0);

		timer->expires = jiffies + delay;
		timer->data = (unsigned long)dwork;
		timer->function = delayed_work_timer_fn;

		if (unlikely(cpu >= 0))
			add_timer_on(timer, cpu);
		else
			add_timer(timer);
		ret = 1;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(queue_delayed_work_on);

/**
 * worker_enter_idle - enter idle state
 * @worker: worker which is entering idle state
 *
 * @worker is entering idle state.  Update stats and idle timer if
 * necessary.
 *
 * LOCKING:
 * spin_lock_irq(pool->lock).
 */
static void worker_enter_idle(struct worker *worker)
{
	struct worker_pool *pool = worker->pool;

	if (WARN_ON_ONCE(worker->flags & WORKER_IDLE) ||
	    WARN_ON_ONCE(!list_empty(&worker->entry) &&
			 (worker->hentry.next || worker->hentry.pprev)))
		return;

	/* can't use worker_set_flags(), also called from start_worker() */
	worker->flags |= WORKER_IDLE;
	pool->nr_idle++;
	worker->last_active = jiffies;

	/* idle_list is LIFO */
	list_add(&worker->entry, &pool->idle_list);

	if (likely(!(worker->flags & WORKER_ROGUE))) {
		if (too_many_workers(pool) && !timer_pending(&pool->idle_timer))
			mod_timer(&pool->idle_timer,
				  jiffies + IDLE_WORKER_TIMEOUT);
	} else
		wake_up_all(&gcwq->trustee_wait);

	/*
	 * Sanity check nr_running.  Because trustee releases gcwq->lock
	 * between setting %WORKER_ROGUE and zapping nr_running, the
	 * warning may trigger spuriously.  Check iff trustee is idle.
	 */
	WARN_ON_ONCE(gcwq->trustee_state == TRUSTEE_DONE &&
		     pool->nr_workers == pool->nr_idle &&
		     atomic_read(get_pool_nr_running(pool)));
}

/**
 * worker_leave_idle - leave idle state
 * @worker: worker which is leaving idle state
 *
 * @worker is leaving idle state.  Update stats.
 *
 * LOCKING:
 * spin_lock_irq(pool->lock).
 */
static void worker_leave_idle(struct worker *worker)
{
	struct worker_pool *pool = worker->pool;

	if (WARN_ON_ONCE(!(worker->flags & WORKER_IDLE)))
		return;
	worker_clr_flags(worker, WORKER_IDLE);
	pool->nr_idle--;
	list_del_init(&worker->entry);
}

/**
 * worker_maybe_bind_and_lock - try to bind %current to worker_pool and lock it
 * @pool: target worker_pool
 *
 * Bind %current to the cpu of @pool if it is associated and lock @pool.
 *
 * Works which are scheduled while the cpu is online must at least be
 * scheduled to a worker which is bound to the cpu so that if they are
 * flushed from cpu callbacks while cpu is going down, they are
 * guaranteed to execute on the cpu.
 *
 * This function is to be used by unbound workers and rescuers to bind
 * themselves to the target cpu and may race with cpu going down or
 * coming online.  kthread_bind() can't be used because it may put the
 * worker to already dead cpu and set_cpus_allowed_ptr() can't be used
 * verbatim as it's best effort and blocking and pool may be
 * [dis]associated in the meantime.
 *
 * This function tries set_cpus_allowed() and locks pool and verifies the
 * binding against %POOL_DISASSOCIATED which is set during
 * %CPU_DOWN_PREPARE and cleared during %CPU_ONLINE, so if the worker
 * enters idle state or fetches works without dropping lock, it can
 * guarantee the scheduling requirement described in the first paragraph.
 *
 * CONTEXT:
 * Might sleep.  Called without any lock but returns with pool->lock
 * held.
 *
 * RETURNS:
 * %true if the associated pool is online (@worker is successfully
 * bound), %false if offline.
 */
static bool worker_maybe_bind_and_lock(struct worker_pool *pool)
__acquires(&pool->lock)
{
	while (true) {
		/*
		 * The following call may fail, succeed or succeed
		 * without actually migrating the task to the cpu if
		 * it races with cpu hotunplug operation.  Verify
		 * against POOL_DISASSOCIATED.
		 */
		if (!(pool->flags & POOL_DISASSOCIATED))
			set_cpus_allowed_ptr(current, pool->attrs->cpumask);

		spin_lock_irq(&pool->lock);
		if (pool->flags & POOL_DISASSOCIATED)
			return false;
		if (task_cpu(current) == pool->cpu &&
		    cpumask_equal(&current->cpus_allowed, pool->attrs->cpumask))
			return true;
		spin_unlock_irq(&pool->lock);

		/*
		 * We've raced with CPU hot[un]plug.  Give it a breather
		 * and retry migration.  cond_resched() is required here;
		 * otherwise, we might deadlock against cpu_stop trying to
		 * bring down the CPU on non-preemptive kernel.
		 */
		cpu_relax();
		cond_resched();
	}
}

/*
 * Function for worker->rebind_work used to rebind rogue busy workers
 * to the associated cpu which is coming back online.  This is
 * scheduled by cpu up but can race with other cpu hotplug operations
 * and may be executed twice without intervening cpu down.
 */
static void worker_rebind_fn(struct work_struct *work)
{
	struct worker *worker = container_of(work, struct worker, rebind_work);
	struct global_cwq *gcwq = worker->pool->gcwq;

	if (worker_maybe_bind_and_lock(worker))
		worker_clr_flags(worker, WORKER_REBIND);

	spin_unlock_irq(&gcwq->lock);
}

static struct worker *alloc_worker(void)
{
	struct worker *worker;

	worker = kzalloc(sizeof(*worker), GFP_KERNEL);
	if (worker) {
		INIT_LIST_HEAD(&worker->entry);
		INIT_LIST_HEAD(&worker->scheduled);
		/* on creation a worker is in !idle && prep state */
		worker->flags = WORKER_PREP;
	}
	return worker;
}

/**
 * create_worker - create a new workqueue worker
 * @pool: pool the new worker will belong to
 * @bind: whether to set affinity to @cpu or not
 *
 * Create a new worker which is bound to @pool.  The returned worker
 * can be started by calling start_worker() or destroyed using
 * destroy_worker().
 *
 * CONTEXT:
 * Might sleep.  Does GFP_KERNEL allocations.
 *
 * RETURNS:
 * Pointer to the newly created worker.
 */
static struct worker *create_worker(struct worker_pool *pool, bool bind)
{
	struct global_cwq *gcwq = pool->gcwq;
	bool on_unbound_cpu = gcwq->cpu == WORK_CPU_UNBOUND;
	const char *pri = worker_pool_pri(pool) ? "H" : "";
	struct worker *worker = NULL;
	int id = -1;

	spin_lock_irq(&gcwq->lock);
	while (ida_get_new(&pool->worker_ida, &id)) {
		spin_unlock_irq(&gcwq->lock);
		if (!ida_pre_get(&pool->worker_ida, GFP_KERNEL))
			goto fail;
		spin_lock_irq(&gcwq->lock);
	}
	spin_unlock_irq(&gcwq->lock);

	worker = alloc_worker();
	if (!worker)
		goto fail;

	worker->pool = pool;
	worker->id = id;

	if (!on_unbound_cpu)
		worker->task = kthread_create_on_node(worker_thread,
					worker, cpu_to_node(gcwq->cpu),
					"kworker/%u:%d%s", gcwq->cpu, id, pri);
	else
		worker->task = kthread_create(worker_thread, worker,
					      "kworker/u:%d%s", id, pri);
	if (IS_ERR(worker->task))
		goto fail;

	if (worker_pool_pri(pool))
		set_user_nice(worker->task, HIGHPRI_NICE_LEVEL);

	/*
	 * A rogue worker will become a regular one if CPU comes
	 * online later on.  Make sure every worker has
	 * PF_THREAD_BOUND set.
	 */
	if (bind && !on_unbound_cpu)
		kthread_bind(worker->task, gcwq->cpu);
	else {
		worker->task->flags |= PF_THREAD_BOUND;
		if (on_unbound_cpu)
			worker->flags |= WORKER_UNBOUND;
	}

	return worker;
fail:
	if (id >= 0) {
		spin_lock_irq(&pool->lock);
		ida_remove(&pool->worker_ida, id);
		spin_unlock_irq(&pool->lock);
	}
	kfree(worker);
	return NULL;
}

/**
 * start_worker - start a newly created worker
 * @worker: worker to start
 *
 * Make the pool aware of @worker and start it.
 *
 * CONTEXT:
 * spin_lock_irq(pool->lock).
 */
static void start_worker(struct worker *worker)
{
	worker->flags |= WORKER_STARTED;
	worker->pool->nr_workers++;
	worker_enter_idle(worker);
	wake_up_process(worker->task);
}

/**
 * destroy_worker - destroy a workqueue worker
 * @worker: worker to be destroyed
 *
 * Destroy @worker and adjust @pool stats accordingly.
 *
 * CONTEXT:
 * spin_lock_irq(pool->lock) which is released and regrabbed.
 */
static void destroy_worker(struct worker *worker)
{
	struct worker_pool *pool = worker->pool;
	int id = worker->id;

	/* sanity check frenzy */
	BUG_ON(worker->current_work);
	BUG_ON(!list_empty(&worker->scheduled));

	if (worker->flags & WORKER_STARTED)
		pool->nr_workers--;
	if (worker->flags & WORKER_IDLE)
		pool->nr_idle--;

	/*
	 * Once WORKER_DIE is set, the kworker may destroy itself at any
	 * point.  Pin to ensure the task stays until we're done with it.
	 */
	get_task_struct(worker->task);

	list_del_init(&worker->entry);
	worker->flags |= WORKER_DIE;

	spin_unlock_irq(&gcwq->lock);

	kthread_stop(worker->task);
	put_task_struct(worker->task);
	kfree(worker);

	spin_lock_irq(&gcwq->lock);
	ida_remove(&pool->worker_ida, id);
}

static void idle_worker_timeout(unsigned long __pool)
{
	struct worker_pool *pool = (void *)__pool;
	struct global_cwq *gcwq = pool->gcwq;

	spin_lock_irq(&gcwq->lock);

	if (too_many_workers(pool)) {
		struct worker *worker;
		unsigned long expires;

		/* idle_list is kept in LIFO order, check the last one */
		worker = list_entry(pool->idle_list.prev, struct worker, entry);
		expires = worker->last_active + IDLE_WORKER_TIMEOUT;

		if (time_before(jiffies, expires))
			mod_timer(&pool->idle_timer, expires);
		else {
			/* it's been idle for too long, wake up manager */
			pool->flags |= POOL_MANAGE_WORKERS;
			wake_up_worker(pool);
		}
	}

	spin_unlock_irq(&pool->lock);
}

static void send_mayday(struct work_struct *work)
{
	struct pool_workqueue *pwq = get_work_pwq(work);
	struct workqueue_struct *wq = pwq->wq;

	lockdep_assert_held(&wq_mayday_lock);

	if (!wq->rescuer)
		return;

	/* mayday mayday mayday */
	if (list_empty(&pwq->mayday_node)) {
		list_add_tail(&pwq->mayday_node, &wq->maydays);
		wake_up_process(wq->rescuer->task);
	}
}

static void pool_mayday_timeout(unsigned long __pool)
{
	struct worker_pool *pool = (void *)__pool;
	struct work_struct *work;

	spin_lock_irq(&wq_mayday_lock);		/* for wq->maydays */
	spin_lock(&pool->lock);

	if (need_to_create_worker(pool)) {
		/*
		 * We've been trying to create a new worker but
		 * haven't been successful.  We might be hitting an
		 * allocation deadlock.  Send distress signals to
		 * rescuers.
		 */
		list_for_each_entry(work, &pool->worklist, entry)
			send_mayday(work);
	}

	spin_unlock(&pool->lock);
	spin_unlock_irq(&wq_mayday_lock);

	mod_timer(&pool->mayday_timer, jiffies + MAYDAY_INTERVAL);
}

/**
 * maybe_create_worker - create a new worker if necessary
 * @pool: pool to create a new worker for
 *
 * Create a new worker for @pool if necessary.  @pool is guaranteed to
 * have at least one idle worker on return from this function.  If
 * creating a new worker takes longer than MAYDAY_INTERVAL, mayday is
 * sent to all rescuers with works scheduled on @pool to resolve
 * possible allocation deadlock.
 *
 * On return, need_to_create_worker() is guaranteed to be %false and
 * may_start_working() %true.
 *
 * LOCKING:
 * spin_lock_irq(pool->lock) which may be released and regrabbed
 * multiple times.  Does GFP_KERNEL allocations.  Called only from
 * manager.
 *
 * RETURNS:
 * %false if no action was taken and pool->lock stayed locked, %true
 * otherwise.
 */
static bool maybe_create_worker(struct worker_pool *pool)
__releases(&pool->lock)
__acquires(&pool->lock)
{
	if (!need_to_create_worker(pool))
		return false;
restart:
	spin_unlock_irq(&pool->lock);

	/* if we don't make progress in MAYDAY_INITIAL_TIMEOUT, call for help */
	mod_timer(&pool->mayday_timer, jiffies + MAYDAY_INITIAL_TIMEOUT);

	while (true) {
		struct worker *worker;

		worker = create_worker(pool, true);
		if (worker) {
			del_timer_sync(&pool->mayday_timer);
			spin_lock_irq(&pool->lock);
			start_worker(worker);
			if (WARN_ON_ONCE(need_to_create_worker(pool)))
				goto restart;
			return true;
		}

		if (!need_to_create_worker(pool))
			break;

		__set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(CREATE_COOLDOWN);

		if (!need_to_create_worker(pool))
			break;
	}

	del_timer_sync(&pool->mayday_timer);
	spin_lock_irq(&pool->lock);
	if (need_to_create_worker(pool))
		goto restart;
	return true;
}

/**
 * maybe_destroy_worker - destroy workers which have been idle for a while
 * @pool: pool to destroy workers for
 *
 * Destroy @pool workers which have been idle for longer than
 * IDLE_WORKER_TIMEOUT.
 *
 * LOCKING:
 * spin_lock_irq(pool->lock) which may be released and regrabbed
 * multiple times.  Called only from manager.
 *
 * RETURNS:
 * %false if no action was taken and pool->lock stayed locked, %true
 * otherwise.
 */
static bool maybe_destroy_workers(struct worker_pool *pool)
{
	bool ret = false;

	while (too_many_workers(pool)) {
		struct worker *worker;
		unsigned long expires;

		worker = list_entry(pool->idle_list.prev, struct worker, entry);
		expires = worker->last_active + IDLE_WORKER_TIMEOUT;

		if (time_before(jiffies, expires)) {
			mod_timer(&pool->idle_timer, expires);
			break;
		}

		destroy_worker(worker);
		ret = true;
	}

	return ret;
}

/**
 * manage_workers - manage worker pool
 * @worker: self
 *
 * Assume the manager role and manage the worker pool @worker belongs
 * to.  At any given time, there can be only zero or one manager per
 * pool.  The exclusion is handled automatically by this function.
 *
 * The caller can safely start processing works on false return.  On
 * true return, it's guaranteed that need_to_create_worker() is false
 * and may_start_working() is true.
 *
 * CONTEXT:
 * spin_lock_irq(pool->lock) which may be released and regrabbed
 * multiple times.  Does GFP_KERNEL allocations.
 *
 * RETURNS:
 * spin_lock_irq(pool->lock) which may be released and regrabbed
 * multiple times.  Does GFP_KERNEL allocations.
 */
static bool manage_workers(struct worker *worker)
{
	struct worker_pool *pool = worker->pool;
	struct global_cwq *gcwq = pool->gcwq;
	bool ret = false;

	if (pool->flags & POOL_MANAGING_WORKERS)
		return ret;

	pool->flags &= ~POOL_MANAGE_WORKERS;
	pool->flags |= POOL_MANAGING_WORKERS;

	/*
	 * Destroy and then create so that may_start_working() is true
	 * on return.
	 */
	ret |= maybe_destroy_workers(pool);
	ret |= maybe_create_worker(pool);

	pool->flags &= ~POOL_MANAGING_WORKERS;

	/*
	 * The trustee might be waiting to take over the manager
	 * position, tell it we're done.
	 */
	if (unlikely(gcwq->trustee))
		wake_up_all(&gcwq->trustee_wait);

	return ret;
}

/**
 * move_linked_works - move linked works to a list
 * @work: start of series of works to be scheduled
 * @head: target list to append @work to
 * @nextp: out paramter for nested worklist walking
 *
 * Schedule linked works starting from @work to @head.  Work series to
 * be scheduled starts at @work and includes any consecutive work with
 * WORK_STRUCT_LINKED set in its predecessor.
 *
 * If @nextp is not NULL, it's updated to point to the next work of
 * the last scheduled work.  This allows move_linked_works() to be
 * nested inside outer list_for_each_entry_safe().
 *
 * CONTEXT:
 * spin_lock_irq(gcwq->lock).
 */
static void move_linked_works(struct work_struct *work, struct list_head *head,
			      struct work_struct **nextp)
{
	struct work_struct *n;

	/*
	 * Linked worklist will always end before the end of the list,
	 * use NULL for list head.
	 */
	list_for_each_entry_safe_from(work, n, NULL, entry) {
		list_move_tail(&work->entry, head);
		if (!(*work_data_bits(work) & WORK_STRUCT_LINKED))
			break;
	}

	/*
	 * If we're already inside safe list traversal and have moved
	 * multiple works to the scheduled queue, the next position
	 * needs to be updated.
	 */
	if (nextp)
		*nextp = n;
}

static void cwq_activate_delayed_work(struct work_struct *work)
{
	struct cpu_workqueue_struct *cwq = get_work_cwq(work);

	trace_workqueue_activate_work(work);
	move_linked_works(work, &cwq->pool->worklist, NULL);
	__clear_bit(WORK_STRUCT_DELAYED_BIT, work_data_bits(work));
	cwq->nr_active++;
}

static void cwq_activate_first_delayed(struct cpu_workqueue_struct *cwq)
{
	struct work_struct *work = list_first_entry(&cwq->delayed_works,
						    struct work_struct, entry);

	cwq_activate_delayed_work(work);
}

/**
 * cwq_dec_nr_in_flight - decrement cwq's nr_in_flight
 * @cwq: cwq of interest
 * @color: color of work which left the queue
 * @delayed: for a delayed work
 *
 * A work either has completed or is removed from pending queue,
 * decrement nr_in_flight of its cwq and handle workqueue flushing.
 *
 * CONTEXT:
 * spin_lock_irq(gcwq->lock).
 */
static void cwq_dec_nr_in_flight(struct cpu_workqueue_struct *cwq, int color,
				 bool delayed)
{
	/* ignore uncolored works */
	if (color == WORK_NO_COLOR)
		return;

	cwq->nr_in_flight[color]--;

	if (!delayed) {
		cwq->nr_active--;
		if (!list_empty(&cwq->delayed_works)) {
			/* one down, submit a delayed one */
			if (cwq->nr_active < cwq->max_active)
				cwq_activate_first_delayed(cwq);
		}
	}

	/* is flush in progress and are we at the flushing tip? */
	if (likely(cwq->flush_color != color))
		return;

	/* are there still in-flight works? */
	if (cwq->nr_in_flight[color])
		return;

	/* this cwq is done, clear flush_color */
	cwq->flush_color = -1;

	/*
	 * If this was the last cwq, wake up the first flusher.  It
	 * will handle the rest.
	 */
	if (atomic_dec_and_test(&cwq->wq->nr_cwqs_to_flush))
		complete(&cwq->wq->first_flusher->done);
}

/**
 * process_one_work - process single work
 * @worker: self
 * @work: work to process
 *
 * Process @work.  This function contains all the logics necessary to
 * process a single work including synchronization against and
 * interaction with other workers on the same cpu, queueing and
 * flushing.  As long as context requirement is met, any worker can
 * call this function to process a work.
 *
 * CONTEXT:
 * spin_lock_irq(gcwq->lock) which is released and regrabbed.
 */
static void process_one_work(struct worker *worker, struct work_struct *work)
__releases(&gcwq->lock)
__acquires(&gcwq->lock)
{
	struct cpu_workqueue_struct *cwq = get_work_cwq(work);
	struct worker_pool *pool = worker->pool;
	struct global_cwq *gcwq = pool->gcwq;
	struct hlist_head *bwh = busy_worker_head(gcwq, work);
	bool cpu_intensive = cwq->wq->flags & WQ_CPU_INTENSIVE;
	int work_color;
	struct worker *collision;
#ifdef CONFIG_LOCKDEP
	/*
	 * It is permissible to free the struct work_struct from
	 * inside the function that is called from it, this we need to
	 * take into account for lockdep too.  To avoid bogus "held
	 * lock freed" warnings as well as problems when looking into
	 * work->lockdep_map, make a copy and use that here.
	 */
	struct lockdep_map lockdep_map;

	lockdep_copy_map(&lockdep_map, &work->lockdep_map);
#endif
	/*
	 * Ensure we're on the correct CPU.  DISASSOCIATED test is
	 * necessary to avoid spurious warnings from rescuers servicing the
	 * unbound or a disassociated pool.
	 */
	WARN_ON_ONCE(!(worker->flags & WORKER_UNBOUND) &&
		     !(pool->flags & POOL_DISASSOCIATED) &&
		     raw_smp_processor_id() != pool->cpu);

	/*
	 * A single work shouldn't be executed concurrently by
	 * multiple workers on a single cpu.  Check whether anyone is
	 * already processing the work.  If so, defer the work to the
	 * currently executing one.
	 */
	collision = find_worker_executing_work(pool, work);
	if (unlikely(collision)) {
		move_linked_works(work, &collision->scheduled, NULL);
		return;
	}

	/* claim and process */
	debug_work_deactivate(work);
	hlist_add_head(&worker->hentry, bwh);
	worker->current_work = work;
	worker->current_func = work->func;
	worker->current_cwq = cwq;
	work_color = get_work_color(work);

	/* record the current cpu number in the work data and dequeue */
	set_work_cpu(work, gcwq->cpu);
	list_del_init(&work->entry);

	/*
	 * CPU intensive works don't participate in concurrency
	 * management.  They're the scheduler's responsibility.
	 */
	if (unlikely(cpu_intensive))
		worker_set_flags(worker, WORKER_CPU_INTENSIVE, true);

	/*
	 * Unbound gcwq isn't concurrency managed and work items should be
	 * executed ASAP.  Wake up another worker if necessary.
	 */
	if ((worker->flags & WORKER_UNBOUND) && need_more_worker(pool))
		wake_up_worker(pool);

	spin_unlock_irq(&gcwq->lock);

	smp_wmb();	/* paired with test_and_set_bit(PENDING) */
	work_clear_pending(work);

	lock_map_acquire_read(&cwq->wq->lockdep_map);
	lock_map_acquire(&lockdep_map);
	trace_workqueue_execute_start(work);
	worker->current_func(work);
	/*
	 * While we must be careful to not use "work" after this, the trace
	 * point will only record its address.
	 */
	trace_workqueue_execute_end(work);
	lock_map_release(&lockdep_map);
	lock_map_release(&cwq->wq->lockdep_map);

	if (unlikely(in_atomic() || lockdep_depth(current) > 0)) {
		pr_err("BUG: workqueue leaked lock or atomic: %s/0x%08x/%d\n"
		       "     last function: %pf\n",
		       current->comm, preempt_count(), task_pid_nr(current),
		       worker->current_func);
		debug_show_held_locks(current);
		dump_stack();
	}

	/*
	 * The following prevents a kworker from hogging CPU on !PREEMPT
	 * kernels, where a requeueing work item waiting for something to
	 * happen could deadlock with stop_machine as such work item could
	 * indefinitely requeue itself while all other CPUs are trapped in
	 * stop_machine.
	 */
	cond_resched();

	spin_lock_irq(&gcwq->lock);

	/* clear cpu intensive status */
	if (unlikely(cpu_intensive))
		worker_clr_flags(worker, WORKER_CPU_INTENSIVE);

	/* we're done with it, release */
	hlist_del_init(&worker->hentry);
	worker->current_work = NULL;
	worker->current_func = NULL;
	worker->current_cwq = NULL;
	cwq_dec_nr_in_flight(cwq, work_color);
}

/**
 * process_scheduled_works - process scheduled works
 * @worker: self
 *
 * Process all scheduled works.  Please note that the scheduled list
 * may change while processing a work, so this function repeatedly
 * fetches a work from the top and executes it.
 *
 * CONTEXT:
 * spin_lock_irq(pool->lock) which may be released and regrabbed
 * multiple times.
 */
static void process_scheduled_works(struct worker *worker)
{
	while (!list_empty(&worker->scheduled)) {
		struct work_struct *work = list_first_entry(&worker->scheduled,
						struct work_struct, entry);
		process_one_work(worker, work);
	}
}

/**
 * worker_thread - the worker thread function
 * @__worker: self
 *
 * The worker thread function.  All workers belong to a worker_pool -
 * either a per-cpu one or dynamic unbound one.  These workers process all
 * work items regardless of their specific target workqueue.  The only
 * exception is work items which belong to workqueues with a rescuer which
 * will be explained in rescuer_thread().
 */
static int worker_thread(void *__worker)
{
	struct worker *worker = __worker;
	struct worker_pool *pool = worker->pool;
	struct global_cwq *gcwq = pool->gcwq;

	/* tell the scheduler that this is a workqueue worker */
	worker->task->flags |= PF_WQ_WORKER;
woke_up:
	spin_lock_irq(&gcwq->lock);

	/* DIE can be set only while we're idle, checking here is enough */
	if (worker->flags & WORKER_DIE) {
		spin_unlock_irq(&gcwq->lock);
		worker->task->flags &= ~PF_WQ_WORKER;
		return 0;
	}

	worker_leave_idle(worker);
recheck:
	/* no more worker necessary? */
	if (!need_more_worker(pool))
		goto sleep;

	/* do we need to manage? */
	if (unlikely(!may_start_working(pool)) && manage_workers(worker))
		goto recheck;

	/*
	 * ->scheduled list can only be filled while a worker is
	 * preparing to process a work or actually processing it.
	 * Make sure nobody diddled with it while I was sleeping.
	 */
	WARN_ON_ONCE(!list_empty(&worker->scheduled));

	/*
	 * Finish PREP stage.  We're guaranteed to have at least one idle
	 * worker or that someone else has already assumed the manager
	 * role.  This is where @worker starts participating in concurrency
	 * management if applicable and concurrency management is restored
	 * after being rebound.  See rebind_workers() for details.
	 */
	worker_clr_flags(worker, WORKER_PREP | WORKER_REBOUND);

	do {
		struct work_struct *work =
			list_first_entry(&pool->worklist,
					 struct work_struct, entry);

		if (likely(!(*work_data_bits(work) & WORK_STRUCT_LINKED))) {
			/* optimization path, not strictly necessary */
			process_one_work(worker, work);
			if (unlikely(!list_empty(&worker->scheduled)))
				process_scheduled_works(worker);
		} else {
			move_linked_works(work, &worker->scheduled, NULL);
			process_scheduled_works(worker);
		}
	} while (keep_working(pool));

	worker_set_flags(worker, WORKER_PREP, false);
sleep:
	if (unlikely(need_to_manage_workers(pool)) && manage_workers(worker))
		goto recheck;

	/*
	 * pool->lock is held and there's no work to process and no need to
	 * manage, sleep.  Workers are woken up only while holding
	 * pool->lock or from local cpu, so setting the current state
	 * before releasing pool->lock is enough to prevent losing any
	 * event.
	 */
	worker_enter_idle(worker);
	__set_current_state(TASK_INTERRUPTIBLE);
	spin_unlock_irq(&pool->lock);
	schedule();
	goto woke_up;
}

/**
 * rescuer_thread - the rescuer thread function
 * @__rescuer: self
 *
 * Workqueue rescuer thread function.  There's one rescuer for each
 * workqueue which has WQ_MEM_RECLAIM set.
 *
 * Regular work processing on a pool may block trying to create a new
 * worker which uses GFP_KERNEL allocation which has slight chance of
 * developing into deadlock if some works currently on the same queue
 * need to be processed to satisfy the GFP_KERNEL allocation.  This is
 * the problem rescuer solves.
 *
 * When such condition is possible, the pool summons rescuers of all
 * workqueues which have works queued on the pool and let them process
 * those works so that forward progress can be guaranteed.
 *
 * This should happen rarely.
 */
static int rescuer_thread(void *__rescuer)
{
	struct worker *rescuer = __rescuer;
	struct workqueue_struct *wq = rescuer->rescue_wq;
	struct list_head *scheduled = &rescuer->scheduled;

	set_user_nice(current, RESCUER_NICE_LEVEL);

	/*
	 * Mark rescuer as worker too.  As WORKER_PREP is never cleared, it
	 * doesn't participate in concurrency management.
	 */
	rescuer->task->flags |= PF_WQ_WORKER;
repeat:
	set_current_state(TASK_INTERRUPTIBLE);

	if (kthread_should_stop()) {
		__set_current_state(TASK_RUNNING);
		rescuer->task->flags &= ~PF_WQ_WORKER;
		return 0;
	}

	/* see whether any pwq is asking for help */
	spin_lock_irq(&wq_mayday_lock);

	while (!list_empty(&wq->maydays)) {
		struct pool_workqueue *pwq = list_first_entry(&wq->maydays,
					struct pool_workqueue, mayday_node);
		struct worker_pool *pool = pwq->pool;
		struct work_struct *work, *n;

		__set_current_state(TASK_RUNNING);
		list_del_init(&pwq->mayday_node);

		spin_unlock_irq(&wq_mayday_lock);

		/* migrate to the target cpu if possible */
		worker_maybe_bind_and_lock(pool);
		rescuer->pool = pool;

		/*
		 * Slurp in all works issued via this workqueue and
		 * process'em.
		 */
		WARN_ON_ONCE(!list_empty(&rescuer->scheduled));
		list_for_each_entry_safe(work, n, &pool->worklist, entry)
			if (get_work_pwq(work) == pwq)
				move_linked_works(work, scheduled, &n);

		process_scheduled_works(rescuer);

		/*
		 * Leave this pool.  If keep_working() is %true, notify a
		 * regular worker; otherwise, we end up with 0 concurrency
		 * and stalling the execution.
		 */
		if (keep_working(pool))
			wake_up_worker(pool);

		rescuer->pool = NULL;
		spin_unlock(&pool->lock);
		spin_lock(&wq_mayday_lock);
	}

	spin_unlock_irq(&wq_mayday_lock);

	/* rescuers should never participate in concurrency management */
	WARN_ON_ONCE(!(rescuer->flags & WORKER_NOT_RUNNING));
	schedule();
	goto repeat;
}

struct wq_barrier {
	struct work_struct	work;
	struct completion	done;
};

static void wq_barrier_func(struct work_struct *work)
{
	struct wq_barrier *barr = container_of(work, struct wq_barrier, work);
	complete(&barr->done);
}

/**
 * insert_wq_barrier - insert a barrier work
 * @pwq: pwq to insert barrier into
 * @barr: wq_barrier to insert
 * @target: target work to attach @barr to
 * @worker: worker currently executing @target, NULL if @target is not executing
 *
 * @barr is linked to @target such that @barr is completed only after
 * @target finishes execution.  Please note that the ordering
 * guarantee is observed only with respect to @target and on the local
 * cpu.
 *
 * Currently, a queued barrier can't be canceled.  This is because
 * try_to_grab_pending() can't determine whether the work to be
 * grabbed is at the head of the queue and thus can't clear LINKED
 * flag of the previous work while there must be a valid next work
 * after a work with LINKED flag set.
 *
 * Note that when @worker is non-NULL, @target may be modified
 * underneath us, so we can't reliably determine pwq from @target.
 *
 * CONTEXT:
 * spin_lock_irq(pool->lock).
 */
static void insert_wq_barrier(struct pool_workqueue *pwq,
			      struct wq_barrier *barr,
			      struct work_struct *target, struct worker *worker)
{
	struct list_head *head;
	unsigned int linked = 0;

	/*
	 * debugobject calls are safe here even with pool->lock locked
	 * as we know for sure that this will not trigger any of the
	 * checks and call back into the fixup functions where we
	 * might deadlock.
	 */
	INIT_WORK_ONSTACK(&barr->work, wq_barrier_func);
	__set_bit(WORK_STRUCT_PENDING_BIT, work_data_bits(&barr->work));
	init_completion(&barr->done);

	/*
	 * If @target is currently being executed, schedule the
	 * barrier to the worker; otherwise, put it after @target.
	 */
	if (worker)
		head = worker->scheduled.next;
	else {
		unsigned long *bits = work_data_bits(target);

		head = target->entry.next;
		/* there can already be other linked works, inherit and set */
		linked = *bits & WORK_STRUCT_LINKED;
		__set_bit(WORK_STRUCT_LINKED_BIT, bits);
	}

	debug_work_activate(&barr->work);
	insert_work(pwq, &barr->work, head,
		    work_color_to_flags(WORK_NO_COLOR) | linked);
}

/**
 * flush_workqueue_prep_pwqs - prepare pwqs for workqueue flushing
 * @wq: workqueue being flushed
 * @flush_color: new flush color, < 0 for no-op
 * @work_color: new work color, < 0 for no-op
 *
 * Prepare pwqs for workqueue flushing.
 *
 * If @flush_color is non-negative, flush_color on all pwqs should be
 * -1.  If no pwq has in-flight commands at the specified color, all
 * pwq->flush_color's stay at -1 and %false is returned.  If any pwq
 * has in flight commands, its pwq->flush_color is set to
 * @flush_color, @wq->nr_pwqs_to_flush is updated accordingly, pwq
 * wakeup logic is armed and %true is returned.
 *
 * The caller should have initialized @wq->first_flusher prior to
 * calling this function with non-negative @flush_color.  If
 * @flush_color is negative, no flush color update is done and %false
 * is returned.
 *
 * If @work_color is non-negative, all pwqs should have the same
 * work_color which is previous to @work_color and all will be
 * advanced to @work_color.
 *
 * CONTEXT:
 * mutex_lock(wq->mutex).
 *
 * RETURNS:
 * %true if @flush_color >= 0 and there's something to flush.  %false
 * otherwise.
 */
static bool flush_workqueue_prep_pwqs(struct workqueue_struct *wq,
				      int flush_color, int work_color)
{
	bool wait = false;
	struct pool_workqueue *pwq;

	if (flush_color >= 0) {
		WARN_ON_ONCE(atomic_read(&wq->nr_pwqs_to_flush));
		atomic_set(&wq->nr_pwqs_to_flush, 1);
	}

	for_each_pwq(pwq, wq) {
		struct worker_pool *pool = pwq->pool;

		spin_lock_irq(&pool->lock);

		if (flush_color >= 0) {
			WARN_ON_ONCE(pwq->flush_color != -1);

			if (pwq->nr_in_flight[flush_color]) {
				pwq->flush_color = flush_color;
				atomic_inc(&wq->nr_pwqs_to_flush);
				wait = true;
			}
		}

		if (work_color >= 0) {
			WARN_ON_ONCE(work_color != work_next_color(pwq->work_color));
			pwq->work_color = work_color;
		}

		spin_unlock_irq(&pool->lock);
	}

	if (flush_color >= 0 && atomic_dec_and_test(&wq->nr_pwqs_to_flush))
		complete(&wq->first_flusher->done);

	return wait;
}

/**
 * flush_workqueue - ensure that any scheduled work has run to completion.
 * @wq: workqueue to flush
 *
 * This function sleeps until all work items which were queued on entry
 * have finished execution, but it is not livelocked by new incoming ones.
 */
void flush_workqueue(struct workqueue_struct *wq)
{
	struct wq_flusher this_flusher = {
		.list = LIST_HEAD_INIT(this_flusher.list),
		.flush_color = -1,
		.done = COMPLETION_INITIALIZER_ONSTACK(this_flusher.done),
	};
	int next_color;

	lock_map_acquire(&wq->lockdep_map);
	lock_map_release(&wq->lockdep_map);

	mutex_lock(&wq->mutex);

	/*
	 * Start-to-wait phase
	 */
	next_color = work_next_color(wq->work_color);

	if (next_color != wq->flush_color) {
		/*
		 * Color space is not full.  The current work_color
		 * becomes our flush_color and work_color is advanced
		 * by one.
		 */
		WARN_ON_ONCE(!list_empty(&wq->flusher_overflow));
		this_flusher.flush_color = wq->work_color;
		wq->work_color = next_color;

		if (!wq->first_flusher) {
			/* no flush in progress, become the first flusher */
			WARN_ON_ONCE(wq->flush_color != this_flusher.flush_color);

			wq->first_flusher = &this_flusher;

			if (!flush_workqueue_prep_pwqs(wq, wq->flush_color,
						       wq->work_color)) {
				/* nothing to flush, done */
				wq->flush_color = next_color;
				wq->first_flusher = NULL;
				goto out_unlock;
			}
		} else {
			/* wait in queue */
			WARN_ON_ONCE(wq->flush_color == this_flusher.flush_color);
			list_add_tail(&this_flusher.list, &wq->flusher_queue);
			flush_workqueue_prep_pwqs(wq, -1, wq->work_color);
		}
	} else {
		/*
		 * Oops, color space is full, wait on overflow queue.
		 * The next flush completion will assign us
		 * flush_color and transfer to flusher_queue.
		 */
		list_add_tail(&this_flusher.list, &wq->flusher_overflow);
	}

	mutex_unlock(&wq->mutex);

	wait_for_completion(&this_flusher.done);

	/*
	 * Wake-up-and-cascade phase
	 *
	 * First flushers are responsible for cascading flushes and
	 * handling overflow.  Non-first flushers can simply return.
	 */
	if (wq->first_flusher != &this_flusher)
		return;

	mutex_lock(&wq->mutex);

	/* we might have raced, check again with mutex held */
	if (wq->first_flusher != &this_flusher)
		goto out_unlock;

	wq->first_flusher = NULL;

	WARN_ON_ONCE(!list_empty(&this_flusher.list));
	WARN_ON_ONCE(wq->flush_color != this_flusher.flush_color);

	while (true) {
		struct wq_flusher *next, *tmp;

		/* complete all the flushers sharing the current flush color */
		list_for_each_entry_safe(next, tmp, &wq->flusher_queue, list) {
			if (next->flush_color != wq->flush_color)
				break;
			list_del_init(&next->list);
			complete(&next->done);
		}

		WARN_ON_ONCE(!list_empty(&wq->flusher_overflow) &&
			     wq->flush_color != work_next_color(wq->work_color));

		/* this flush_color is finished, advance by one */
		wq->flush_color = work_next_color(wq->flush_color);

		/* one color has been freed, handle overflow queue */
		if (!list_empty(&wq->flusher_overflow)) {
			/*
			 * Assign the same color to all overflowed
			 * flushers, advance work_color and append to
			 * flusher_queue.  This is the start-to-wait
			 * phase for these overflowed flushers.
			 */
			list_for_each_entry(tmp, &wq->flusher_overflow, list)
				tmp->flush_color = wq->work_color;

			wq->work_color = work_next_color(wq->work_color);

			list_splice_tail_init(&wq->flusher_overflow,
					      &wq->flusher_queue);
			flush_workqueue_prep_pwqs(wq, -1, wq->work_color);
		}

		if (list_empty(&wq->flusher_queue)) {
			WARN_ON_ONCE(wq->flush_color != wq->work_color);
			break;
		}

		/*
		 * Need to flush more colors.  Make the next flusher
		 * the new first flusher and arm pwqs.
		 */
		WARN_ON_ONCE(wq->flush_color == wq->work_color);
		WARN_ON_ONCE(wq->flush_color != next->flush_color);

		list_del_init(&next->list);
		wq->first_flusher = next;

		if (flush_workqueue_prep_pwqs(wq, wq->flush_color, -1))
			break;

		/*
		 * Meh... this color is already done, clear first
		 * flusher and repeat cascading.
		 */
		wq->first_flusher = NULL;
	}

out_unlock:
	mutex_unlock(&wq->mutex);
}
EXPORT_SYMBOL_GPL(flush_workqueue);

/**
 * drain_workqueue - drain a workqueue
 * @wq: workqueue to drain
 *
 * Wait until the workqueue becomes empty.  While draining is in progress,
 * only chain queueing is allowed.  IOW, only currently pending or running
 * work items on @wq can queue further work items on it.  @wq is flushed
 * repeatedly until it becomes empty.  The number of flushing is detemined
 * by the depth of chaining and should be relatively short.  Whine if it
 * takes too long.
 */
void drain_workqueue(struct workqueue_struct *wq)
{
	unsigned int flush_cnt = 0;
	struct pool_workqueue *pwq;

	/*
	 * __queue_work() needs to test whether there are drainers, is much
	 * hotter than drain_workqueue() and already looks at @wq->flags.
	 * Use __WQ_DRAINING so that queue doesn't have to check nr_drainers.
	 */
	mutex_lock(&wq->mutex);
	if (!wq->nr_drainers++)
		wq->flags |= __WQ_DRAINING;
	mutex_unlock(&wq->mutex);
reflush:
	flush_workqueue(wq);

	mutex_lock(&wq->mutex);

	for_each_pwq(pwq, wq) {
		bool drained;

		spin_lock_irq(&pwq->pool->lock);
		drained = !pwq->nr_active && list_empty(&pwq->delayed_works);
		spin_unlock_irq(&pwq->pool->lock);

		if (drained)
			continue;

		if (++flush_cnt == 10 ||
		    (flush_cnt % 100 == 0 && flush_cnt <= 1000))
			pr_warn("workqueue %s: drain_workqueue() isn't complete after %u tries\n",
				wq->name, flush_cnt);

		mutex_unlock(&wq->mutex);
		goto reflush;
	}

	if (!--wq->nr_drainers)
		wq->flags &= ~__WQ_DRAINING;
	mutex_unlock(&wq->mutex);
}
EXPORT_SYMBOL_GPL(drain_workqueue);

static bool start_flush_work(struct work_struct *work, struct wq_barrier *barr)
{
	struct worker *worker = NULL;
	struct worker_pool *pool;
	struct pool_workqueue *pwq;

	might_sleep();

	local_irq_disable();
	pool = get_work_pool(work);
	if (!pool) {
		local_irq_enable();
		return false;
	}

	spin_lock(&pool->lock);
	/* see the comment in try_to_grab_pending() with the same code */
	pwq = get_work_pwq(work);
	if (pwq) {
		if (unlikely(pwq->pool != pool))
			goto already_gone;
	} else {
		worker = find_worker_executing_work(pool, work);
		if (!worker)
			goto already_gone;
		pwq = worker->current_pwq;
	}

	insert_wq_barrier(pwq, barr, work, worker);
	spin_unlock_irq(&pool->lock);

	/*
	 * If @max_active is 1 or rescuer is in use, flushing another work
	 * item on the same workqueue may lead to deadlock.  Make sure the
	 * flusher is not running on the same workqueue by verifying write
	 * access.
	 */
	if (pwq->wq->saved_max_active == 1 || pwq->wq->rescuer)
		lock_map_acquire(&pwq->wq->lockdep_map);
	else
		lock_map_acquire_read(&pwq->wq->lockdep_map);
	lock_map_release(&pwq->wq->lockdep_map);

	return true;
already_gone:
	spin_unlock_irq(&pool->lock);
	return false;
}

/**
 * flush_work - wait for a work to finish executing the last queueing instance
 * @work: the work to flush
 *
 * Wait until @work has finished execution.  @work is guaranteed to be idle
 * on return if it hasn't been requeued since flush started.
 *
 * RETURNS:
 * %true if flush_work() waited for the work to finish execution,
 * %false if it was already idle.
 */
bool flush_work(struct work_struct *work)
{
	struct wq_barrier barr;

	lock_map_acquire(&work->lockdep_map);
	lock_map_release(&work->lockdep_map);

	if (start_flush_work(work, &barr, true)) {
		wait_for_completion(&barr.done);
		destroy_work_on_stack(&barr.work);
		return true;
	} else
		return false;
}
EXPORT_SYMBOL_GPL(flush_work);

static bool wait_on_cpu_work(struct global_cwq *gcwq, struct work_struct *work)
{
	struct wq_barrier barr;
	struct worker *worker;

	spin_lock_irq(&gcwq->lock);

	worker = find_worker_executing_work(gcwq, work);
	if (unlikely(worker))
		insert_wq_barrier(worker->current_cwq, &barr, work, worker);

	spin_unlock_irq(&gcwq->lock);

	if (unlikely(worker)) {
		wait_for_completion(&barr.done);
		destroy_work_on_stack(&barr.work);
		return true;
	} else
		return false;
}

static bool wait_on_work(struct work_struct *work)
{
	bool ret = false;
	int cpu;

	might_sleep();

	lock_map_acquire(&work->lockdep_map);
	lock_map_release(&work->lockdep_map);

	for_each_gcwq_cpu(cpu)
		ret |= wait_on_cpu_work(get_gcwq(cpu), work);
	return ret;
}

/**
 * flush_work_sync - wait until a work has finished execution
 * @work: the work to flush
 *
 * Wait until @work has finished execution.  On return, it's
 * guaranteed that all queueing instances of @work which happened
 * before this function is called are finished.  In other words, if
 * @work hasn't been requeued since this function was called, @work is
 * guaranteed to be idle on return.
 *
 * RETURNS:
 * %true if flush_work_sync() waited for the work to finish execution,
 * %false if it was already idle.
 */
bool flush_work_sync(struct work_struct *work)
{
	struct wq_barrier barr;
	bool pending, waited;

	/* we'll wait for executions separately, queue barr only if pending */
	pending = start_flush_work(work, &barr, false);

	/* wait for executions to finish */
	waited = wait_on_work(work);

	/* wait for the pending one */
	if (pending) {
		wait_for_completion(&barr.done);
		destroy_work_on_stack(&barr.work);
	}

	return pending || waited;
}
EXPORT_SYMBOL_GPL(flush_work_sync);

/*
 * Upon a successful return (>= 0), the caller "owns" WORK_STRUCT_PENDING bit,
 * so this work can't be re-armed in any way.
 */
static int try_to_grab_pending(struct work_struct *work)
{
	struct global_cwq *gcwq;
	int ret = -1;

	if (!test_and_set_bit(WORK_STRUCT_PENDING_BIT, work_data_bits(work)))
		return 0;

	/*
	 * The queueing is in progress, or it is already queued. Try to
	 * steal it from ->worklist without clearing WORK_STRUCT_PENDING.
	 */
	gcwq = get_work_gcwq(work);
	if (!gcwq)
		return ret;

	spin_lock_irq(&gcwq->lock);
	if (!list_empty(&work->entry)) {
		/*
		 * This work is queued, but perhaps we locked the wrong gcwq.
		 * In that case we must see the new value after rmb(), see
		 * insert_work()->wmb().
		 */
		smp_rmb();
		if (gcwq == get_work_gcwq(work)) {
			debug_work_deactivate(work);

			/*
			 * A delayed work item cannot be grabbed directly
			 * because it might have linked NO_COLOR work items
			 * which, if left on the delayed_list, will confuse
			 * cwq->nr_active management later on and cause
			 * stall.  Make sure the work item is activated
			 * before grabbing.
			 */
			if (*work_data_bits(work) & WORK_STRUCT_DELAYED)
				cwq_activate_delayed_work(work);

			list_del_init(&work->entry);
			cwq_dec_nr_in_flight(get_work_cwq(work),
				get_work_color(work),
				*work_data_bits(work) & WORK_STRUCT_DELAYED);
			ret = 1;
		}
	}
	spin_unlock_irq(&gcwq->lock);

	return ret;
}

static bool __cancel_work_timer(struct work_struct *work,
				struct timer_list* timer)
{
	int ret;

	do {
		ret = (timer && likely(del_timer(timer)));
		if (!ret)
			ret = try_to_grab_pending(work);
		wait_on_work(work);
	} while (unlikely(ret < 0));

	clear_work_data(work);
	return ret;
}

/**
 * cancel_work_sync - cancel a work and wait for it to finish
 * @work: the work to cancel
 *
 * Cancel @work and wait for its execution to finish.  This function
 * can be used even if the work re-queues itself or migrates to
 * another workqueue.  On return from this function, @work is
 * guaranteed to be not pending or executing on any CPU.
 *
 * cancel_work_sync(&delayed_work->work) must not be used for
 * delayed_work's.  Use cancel_delayed_work_sync() instead.
 *
 * The caller must ensure that the workqueue on which @work was last
 * queued can't be destroyed before this function returns.
 *
 * RETURNS:
 * %true if @work was pending, %false otherwise.
 */
bool cancel_work_sync(struct work_struct *work)
{
	return __cancel_work_timer(work, false);
}
EXPORT_SYMBOL_GPL(cancel_work_sync);

/**
 * flush_delayed_work - wait for a dwork to finish executing the last queueing
 * @dwork: the delayed work to flush
 *
 * Delayed timer is cancelled and the pending work is queued for
 * immediate execution.  Like flush_work(), this function only
 * considers the last queueing instance of @dwork.
 *
 * RETURNS:
 * %true if flush_work() waited for the work to finish execution,
 * %false if it was already idle.
 */
bool flush_delayed_work(struct delayed_work *dwork)
{
	local_irq_disable();
	if (del_timer_sync(&dwork->timer))
		__queue_work(dwork->cpu, dwork->wq, &dwork->work);
	local_irq_enable();
	return flush_work(&dwork->work);
}
EXPORT_SYMBOL(flush_delayed_work);

/**
 * cancel_delayed_work - cancel a delayed work
 * @dwork: delayed_work to cancel
 *
 * Kill off a pending delayed_work.  Returns %true if @dwork was pending
 * and canceled; %false if wasn't pending.  Note that the work callback
 * function may still be running on return, unless it returns %true and the
 * work doesn't re-arm itself.  Explicitly flush or use
 * cancel_delayed_work_sync() to wait on it.
 *
 * This function is safe to call from any context including IRQ handler.
 */
bool cancel_delayed_work(struct delayed_work *dwork)
{
	unsigned long flags;
	int ret;

	do {
		ret = try_to_grab_pending(&dwork->work, true, &flags);
	} while (unlikely(ret == -EAGAIN));

	if (unlikely(ret < 0))
		return false;

	set_work_pool_and_clear_pending(&dwork->work,
					get_work_pool_id(&dwork->work));
	local_irq_restore(flags);
	return ret;
}
EXPORT_SYMBOL(cancel_delayed_work);

/**
 * cancel_delayed_work_sync - cancel a delayed work and wait for it to finish
 * @dwork: the delayed work cancel
 *
 * This is cancel_work_sync() for delayed works.
 *
 * RETURNS:
 * %true if @dwork was pending, %false otherwise.
 */
bool cancel_delayed_work_sync(struct delayed_work *dwork)
{
	return __cancel_work_timer(&dwork->work, true);
}
EXPORT_SYMBOL(cancel_delayed_work_sync);

/**
 * schedule_on_each_cpu - execute a function synchronously on each online CPU
 * @func: the function to call
 *
 * schedule_on_each_cpu() executes @func on each online CPU using the
 * system workqueue and blocks until all CPUs have completed.
 * schedule_on_each_cpu() is very slow.
 *
 * RETURNS:
 * 0 on success, -errno on failure.
 */
int schedule_on_each_cpu(work_func_t func)
{
	int cpu;
	struct work_struct __percpu *works;

	works = alloc_percpu(struct work_struct);
	if (!works)
		return -ENOMEM;

	get_online_cpus();

	for_each_online_cpu(cpu) {
		struct work_struct *work = per_cpu_ptr(works, cpu);

		INIT_WORK(work, func);
		schedule_work_on(cpu, work);
	}

	for_each_online_cpu(cpu)
		flush_work(per_cpu_ptr(works, cpu));

	put_online_cpus();
	free_percpu(works);
	return 0;
}

/**
 * flush_scheduled_work - ensure that any scheduled work has run to completion.
 *
 * Forces execution of the kernel-global workqueue and blocks until its
 * completion.
 *
 * Think twice before calling this function!  It's very easy to get into
 * trouble if you don't take great care.  Either of the following situations
 * will lead to deadlock:
 *
 *	One of the work items currently on the workqueue needs to acquire
 *	a lock held by your code or its caller.
 *
 *	Your code is running in the context of a work routine.
 *
 * They will be detected by lockdep when they occur, but the first might not
 * occur very often.  It depends on what work items are on the workqueue and
 * what locks they need, which you have no control over.
 *
 * In most situations flushing the entire workqueue is overkill; you merely
 * need to know that a particular work item isn't queued and isn't running.
 * In such cases you should use cancel_delayed_work_sync() or
 * cancel_work_sync() instead.
 */
void flush_scheduled_work(void)
{
	flush_workqueue(system_wq);
}
EXPORT_SYMBOL(flush_scheduled_work);

/**
 * execute_in_process_context - reliably execute the routine with user context
 * @fn:		the function to execute
 * @ew:		guaranteed storage for the execute work structure (must
 *		be available when the work executes)
 *
 * Executes the function immediately if process context is available,
 * otherwise schedules the function for delayed execution.
 *
 * Returns:	0 - function was executed
 *		1 - function was scheduled for execution
 */
int execute_in_process_context(work_func_t fn, struct execute_work *ew)
{
	if (!in_interrupt()) {
		fn(&ew->work);
		return 0;
	}

	INIT_WORK(&ew->work, fn);
	schedule_work(&ew->work);

	return 1;
}
EXPORT_SYMBOL_GPL(execute_in_process_context);

#ifdef CONFIG_SYSFS
/*
 * Workqueues with WQ_SYSFS flag set is visible to userland via
 * /sys/bus/workqueue/devices/WQ_NAME.  All visible workqueues have the
 * following attributes.
 *
 *  per_cpu	RO bool	: whether the workqueue is per-cpu or unbound
 *  max_active	RW int	: maximum number of in-flight work items
 *
 * Unbound workqueues have the following extra attributes.
 *
 *  id		RO int	: the associated pool ID
 *  nice	RW int	: nice value of the workers
 *  cpumask	RW mask	: bitmask of allowed CPUs for the workers
 */
struct wq_device {
	struct workqueue_struct		*wq;
	struct device			dev;
};

static struct workqueue_struct *dev_to_wq(struct device *dev)
{
	struct wq_device *wq_dev = container_of(dev, struct wq_device, dev);

	return wq_dev->wq;
}

static ssize_t wq_per_cpu_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct workqueue_struct *wq = dev_to_wq(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", (bool)!(wq->flags & WQ_UNBOUND));
}

static ssize_t wq_max_active_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct workqueue_struct *wq = dev_to_wq(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", wq->saved_max_active);
}

static ssize_t wq_max_active_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct workqueue_struct *wq = dev_to_wq(dev);
	int val;

	if (sscanf(buf, "%d", &val) != 1 || val <= 0)
		return -EINVAL;

	workqueue_set_max_active(wq, val);
	return count;
}

static struct device_attribute wq_sysfs_attrs[] = {
	__ATTR(per_cpu, 0444, wq_per_cpu_show, NULL),
	__ATTR(max_active, 0644, wq_max_active_show, wq_max_active_store),
	__ATTR_NULL,
};

static ssize_t wq_pool_ids_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct workqueue_struct *wq = dev_to_wq(dev);
	const char *delim = "";
	int node, written = 0;

	rcu_read_lock_sched();
	for_each_node(node) {
		written += scnprintf(buf + written, PAGE_SIZE - written,
				     "%s%d:%d", delim, node,
				     unbound_pwq_by_node(wq, node)->pool->id);
		delim = " ";
	}
	written += scnprintf(buf + written, PAGE_SIZE - written, "\n");
	rcu_read_unlock_sched();

	return written;
}

static ssize_t wq_nice_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct workqueue_struct *wq = dev_to_wq(dev);
	int written;

	mutex_lock(&wq->mutex);
	written = scnprintf(buf, PAGE_SIZE, "%d\n", wq->unbound_attrs->nice);
	mutex_unlock(&wq->mutex);

	return written;
}

/* prepare workqueue_attrs for sysfs store operations */
static struct workqueue_attrs *wq_sysfs_prep_attrs(struct workqueue_struct *wq)
{
	struct workqueue_attrs *attrs;

	attrs = alloc_workqueue_attrs(GFP_KERNEL);
	if (!attrs)
		return NULL;

	mutex_lock(&wq->mutex);
	copy_workqueue_attrs(attrs, wq->unbound_attrs);
	mutex_unlock(&wq->mutex);
	return attrs;
}

static ssize_t wq_nice_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct workqueue_struct *wq = dev_to_wq(dev);
	struct workqueue_attrs *attrs;
	int ret;

	attrs = wq_sysfs_prep_attrs(wq);
	if (!attrs)
		return -ENOMEM;

	if (sscanf(buf, "%d", &attrs->nice) == 1 &&
	    attrs->nice >= -20 && attrs->nice <= 19)
		ret = apply_workqueue_attrs(wq, attrs);
	else
		ret = -EINVAL;

	free_workqueue_attrs(attrs);
	return ret ?: count;
}

static ssize_t wq_cpumask_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct workqueue_struct *wq = dev_to_wq(dev);
	int written;

	mutex_lock(&wq->mutex);
	written = cpumask_scnprintf(buf, PAGE_SIZE, wq->unbound_attrs->cpumask);
	mutex_unlock(&wq->mutex);

	written += scnprintf(buf + written, PAGE_SIZE - written, "\n");
	return written;
}

static ssize_t wq_cpumask_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct workqueue_struct *wq = dev_to_wq(dev);
	struct workqueue_attrs *attrs;
	int ret;

	attrs = wq_sysfs_prep_attrs(wq);
	if (!attrs)
		return -ENOMEM;

	ret = cpumask_parse(buf, attrs->cpumask);
	if (!ret)
		ret = apply_workqueue_attrs(wq, attrs);

	free_workqueue_attrs(attrs);
	return ret ?: count;
}

static ssize_t wq_numa_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct workqueue_struct *wq = dev_to_wq(dev);
	int written;

	mutex_lock(&wq->mutex);
	written = scnprintf(buf, PAGE_SIZE, "%d\n",
			    !wq->unbound_attrs->no_numa);
	mutex_unlock(&wq->mutex);

	return written;
}

static ssize_t wq_numa_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct workqueue_struct *wq = dev_to_wq(dev);
	struct workqueue_attrs *attrs;
	int v, ret;

	attrs = wq_sysfs_prep_attrs(wq);
	if (!attrs)
		return -ENOMEM;

	ret = -EINVAL;
	if (sscanf(buf, "%d", &v) == 1) {
		attrs->no_numa = !v;
		ret = apply_workqueue_attrs(wq, attrs);
	}

	free_workqueue_attrs(attrs);
	return ret ?: count;
}

static struct device_attribute wq_sysfs_unbound_attrs[] = {
	__ATTR(pool_ids, 0444, wq_pool_ids_show, NULL),
	__ATTR(nice, 0644, wq_nice_show, wq_nice_store),
	__ATTR(cpumask, 0644, wq_cpumask_show, wq_cpumask_store),
	__ATTR(numa, 0644, wq_numa_show, wq_numa_store),
	__ATTR_NULL,
};

static struct bus_type wq_subsys = {
	.name				= "workqueue",
	.dev_attrs			= wq_sysfs_attrs,
};

static int __init wq_sysfs_init(void)
{
	return subsys_virtual_register(&wq_subsys, NULL);
}
core_initcall(wq_sysfs_init);

static void wq_device_release(struct device *dev)
{
	struct wq_device *wq_dev = container_of(dev, struct wq_device, dev);

	kfree(wq_dev);
}

/**
 * workqueue_sysfs_register - make a workqueue visible in sysfs
 * @wq: the workqueue to register
 *
 * Expose @wq in sysfs under /sys/bus/workqueue/devices.
 * alloc_workqueue*() automatically calls this function if WQ_SYSFS is set
 * which is the preferred method.
 *
 * Workqueue user should use this function directly iff it wants to apply
 * workqueue_attrs before making the workqueue visible in sysfs; otherwise,
 * apply_workqueue_attrs() may race against userland updating the
 * attributes.
 *
 * Returns 0 on success, -errno on failure.
 */
int workqueue_sysfs_register(struct workqueue_struct *wq)
{
	struct wq_device *wq_dev;
	int ret;

	/*
	 * Adjusting max_active or creating new pwqs by applyting
	 * attributes breaks ordering guarantee.  Disallow exposing ordered
	 * workqueues.
	 */
	if (WARN_ON(wq->flags & __WQ_ORDERED))
		return -EINVAL;

	wq->wq_dev = wq_dev = kzalloc(sizeof(*wq_dev), GFP_KERNEL);
	if (!wq_dev)
		return -ENOMEM;

	wq_dev->wq = wq;
	wq_dev->dev.bus = &wq_subsys;
	wq_dev->dev.init_name = wq->name;
	wq_dev->dev.release = wq_device_release;

	/*
	 * unbound_attrs are created separately.  Suppress uevent until
	 * everything is ready.
	 */
	dev_set_uevent_suppress(&wq_dev->dev, true);

	ret = device_register(&wq_dev->dev);
	if (ret) {
		kfree(wq_dev);
		wq->wq_dev = NULL;
		return ret;
	}

	if (wq->flags & WQ_UNBOUND) {
		struct device_attribute *attr;

		for (attr = wq_sysfs_unbound_attrs; attr->attr.name; attr++) {
			ret = device_create_file(&wq_dev->dev, attr);
			if (ret) {
				device_unregister(&wq_dev->dev);
				wq->wq_dev = NULL;
				return ret;
			}
		}
	}

	kobject_uevent(&wq_dev->dev.kobj, KOBJ_ADD);
	return 0;
}

/**
 * workqueue_sysfs_unregister - undo workqueue_sysfs_register()
 * @wq: the workqueue to unregister
 *
 * If @wq is registered to sysfs by workqueue_sysfs_register(), unregister.
 */
static void workqueue_sysfs_unregister(struct workqueue_struct *wq)
{
	struct wq_device *wq_dev = wq->wq_dev;

	if (!wq->wq_dev)
		return;

	wq->wq_dev = NULL;
	device_unregister(&wq_dev->dev);
}
#else	/* CONFIG_SYSFS */
static void workqueue_sysfs_unregister(struct workqueue_struct *wq)	{ }
#endif	/* CONFIG_SYSFS */

/**
 * free_workqueue_attrs - free a workqueue_attrs
 * @attrs: workqueue_attrs to free
 *
 * Undo alloc_workqueue_attrs().
 */
void free_workqueue_attrs(struct workqueue_attrs *attrs)
{
	if (attrs) {
		free_cpumask_var(attrs->cpumask);
		kfree(attrs);
	}
}

/**
 * alloc_workqueue_attrs - allocate a workqueue_attrs
 * @gfp_mask: allocation mask to use
 *
 * Allocate a new workqueue_attrs, initialize with default settings and
 * return it.  Returns NULL on failure.
 */
struct workqueue_attrs *alloc_workqueue_attrs(gfp_t gfp_mask)
{
	struct workqueue_attrs *attrs;

	attrs = kzalloc(sizeof(*attrs), gfp_mask);
	if (!attrs)
		goto fail;
	if (!alloc_cpumask_var(&attrs->cpumask, gfp_mask))
		goto fail;

	cpumask_copy(attrs->cpumask, cpu_possible_mask);
	return attrs;
fail:
	free_workqueue_attrs(attrs);
	return NULL;
}

static void copy_workqueue_attrs(struct workqueue_attrs *to,
				 const struct workqueue_attrs *from)
{
	to->nice = from->nice;
	cpumask_copy(to->cpumask, from->cpumask);
}

/* hash value of the content of @attr */
static u32 wqattrs_hash(const struct workqueue_attrs *attrs)
{
	u32 hash = 0;

	hash = jhash_1word(attrs->nice, hash);
	hash = jhash(cpumask_bits(attrs->cpumask),
		     BITS_TO_LONGS(nr_cpumask_bits) * sizeof(long), hash);
	return hash;
}

/* content equality test */
static bool wqattrs_equal(const struct workqueue_attrs *a,
			  const struct workqueue_attrs *b)
{
	if (a->nice != b->nice)
		return false;
	if (!cpumask_equal(a->cpumask, b->cpumask))
		return false;
	return true;
}

/**
 * init_worker_pool - initialize a newly zalloc'd worker_pool
 * @pool: worker_pool to initialize
 *
 * Initiailize a newly zalloc'd @pool.  It also allocates @pool->attrs.
 * Returns 0 on success, -errno on failure.  Even on failure, all fields
 * inside @pool proper are initialized and put_unbound_pool() can be called
 * on @pool safely to release it.
 */
static int init_worker_pool(struct worker_pool *pool)
{
	spin_lock_init(&pool->lock);
	pool->id = -1;
	pool->cpu = -1;
	pool->node = NUMA_NO_NODE;
	pool->flags |= POOL_DISASSOCIATED;
	INIT_LIST_HEAD(&pool->worklist);
	INIT_LIST_HEAD(&pool->idle_list);
	hash_init(pool->busy_hash);

	init_timer_deferrable(&pool->idle_timer);
	pool->idle_timer.function = idle_worker_timeout;
	pool->idle_timer.data = (unsigned long)pool;

	setup_timer(&pool->mayday_timer, pool_mayday_timeout,
		    (unsigned long)pool);

	mutex_init(&pool->manager_arb);
	mutex_init(&pool->manager_mutex);
	idr_init(&pool->worker_idr);

	INIT_HLIST_NODE(&pool->hash_node);
	pool->refcnt = 1;

	/* shouldn't fail above this point */
	pool->attrs = alloc_workqueue_attrs(GFP_KERNEL);
	if (!pool->attrs)
		return -ENOMEM;
	return 0;
}

static void rcu_free_pool(struct rcu_head *rcu)
{
	struct worker_pool *pool = container_of(rcu, struct worker_pool, rcu);

	idr_destroy(&pool->worker_idr);
	free_workqueue_attrs(pool->attrs);
	kfree(pool);
}

/**
 * put_unbound_pool - put a worker_pool
 * @pool: worker_pool to put
 *
 * Put @pool.  If its refcnt reaches zero, it gets destroyed in sched-RCU
 * safe manner.  get_unbound_pool() calls this function on its failure path
 * and this function should be able to release pools which went through,
 * successfully or not, init_worker_pool().
 *
 * Should be called with wq_pool_mutex held.
 */
static void put_unbound_pool(struct worker_pool *pool)
{
	struct worker *worker;

	lockdep_assert_held(&wq_pool_mutex);

	if (--pool->refcnt)
		return;

	/* sanity checks */
	if (WARN_ON(!(pool->flags & POOL_DISASSOCIATED)) ||
	    WARN_ON(!list_empty(&pool->worklist)))
		return;

	/* release id and unhash */
	if (pool->id >= 0)
		idr_remove(&worker_pool_idr, pool->id);
	hash_del(&pool->hash_node);

	/*
	 * Become the manager and destroy all workers.  Grabbing
	 * manager_arb prevents @pool's workers from blocking on
	 * manager_mutex.
	 */
	mutex_lock(&pool->manager_arb);
	mutex_lock(&pool->manager_mutex);
	spin_lock_irq(&pool->lock);

	while ((worker = first_worker(pool)))
		destroy_worker(worker);
	WARN_ON(pool->nr_workers || pool->nr_idle);

	spin_unlock_irq(&pool->lock);
	mutex_unlock(&pool->manager_mutex);
	mutex_unlock(&pool->manager_arb);

	/* shut down the timers */
	del_timer_sync(&pool->idle_timer);
	del_timer_sync(&pool->mayday_timer);

	/* sched-RCU protected to allow dereferences from get_work_pool() */
	call_rcu_sched(&pool->rcu, rcu_free_pool);
}

/**
 * get_unbound_pool - get a worker_pool with the specified attributes
 * @attrs: the attributes of the worker_pool to get
 *
 * Obtain a worker_pool which has the same attributes as @attrs, bump the
 * reference count and return it.  If there already is a matching
 * worker_pool, it will be used; otherwise, this function attempts to
 * create a new one.  On failure, returns NULL.
 *
 * Should be called with wq_pool_mutex held.
 */
static struct worker_pool *get_unbound_pool(const struct workqueue_attrs *attrs)
{
	u32 hash = wqattrs_hash(attrs);
	struct worker_pool *pool;
	int node;

	lockdep_assert_held(&wq_pool_mutex);

	/* do we already have a matching pool? */
	hash_for_each_possible(unbound_pool_hash, pool, hash_node, hash) {
		if (wqattrs_equal(pool->attrs, attrs)) {
			pool->refcnt++;
			goto out_unlock;
		}
	}

	/* nope, create a new one */
	pool = kzalloc(sizeof(*pool), GFP_KERNEL);
	if (!pool || init_worker_pool(pool) < 0)
		goto fail;

	if (workqueue_freezing)
		pool->flags |= POOL_FREEZING;

	lockdep_set_subclass(&pool->lock, 1);	/* see put_pwq() */
	copy_workqueue_attrs(pool->attrs, attrs);

	/* if cpumask is contained inside a NUMA node, we belong to that node */
	if (wq_numa_enabled) {
		for_each_node(node) {
			if (cpumask_subset(pool->attrs->cpumask,
					   wq_numa_possible_cpumask[node])) {
				pool->node = node;
				break;
			}
		}
	}

	if (worker_pool_assign_id(pool) < 0)
		goto fail;

	/* create and start the initial worker */
	if (create_and_start_worker(pool) < 0)
		goto fail;

	/* install */
	hash_add(unbound_pool_hash, &pool->hash_node, hash);
out_unlock:
	return pool;
fail:
	if (pool)
		put_unbound_pool(pool);
	return NULL;
}

static void rcu_free_pwq(struct rcu_head *rcu)
{
	kmem_cache_free(pwq_cache,
			container_of(rcu, struct pool_workqueue, rcu));
}

/*
 * Scheduled on system_wq by put_pwq() when an unbound pwq hits zero refcnt
 * and needs to be destroyed.
 */
static void pwq_unbound_release_workfn(struct work_struct *work)
{
	struct pool_workqueue *pwq = container_of(work, struct pool_workqueue,
						  unbound_release_work);
	struct workqueue_struct *wq = pwq->wq;
	struct worker_pool *pool = pwq->pool;
	bool is_last;

	if (WARN_ON_ONCE(!(wq->flags & WQ_UNBOUND)))
		return;

	/*
	 * Unlink @pwq.  Synchronization against wq->mutex isn't strictly
	 * necessary on release but do it anyway.  It's easier to verify
	 * and consistent with the linking path.
	 */
	mutex_lock(&wq->mutex);
	list_del_rcu(&pwq->pwqs_node);
	is_last = list_empty(&wq->pwqs);
	mutex_unlock(&wq->mutex);

	mutex_lock(&wq_pool_mutex);
	put_unbound_pool(pool);
	mutex_unlock(&wq_pool_mutex);

	call_rcu_sched(&pwq->rcu, rcu_free_pwq);

	/*
	 * If we're the last pwq going away, @wq is already dead and no one
	 * is gonna access it anymore.  Free it.
	 */
	if (is_last) {
		free_workqueue_attrs(wq->unbound_attrs);
		kfree(wq);
	}
}

/**
 * pwq_adjust_max_active - update a pwq's max_active to the current setting
 * @pwq: target pool_workqueue
 *
 * If @pwq isn't freezing, set @pwq->max_active to the associated
 * workqueue's saved_max_active and activate delayed work items
 * accordingly.  If @pwq is freezing, clear @pwq->max_active to zero.
 */
static void pwq_adjust_max_active(struct pool_workqueue *pwq)
{
	struct workqueue_struct *wq = pwq->wq;
	bool freezable = wq->flags & WQ_FREEZABLE;

	/* for @wq->saved_max_active */
	lockdep_assert_held(&wq->mutex);

	/* fast exit for non-freezable wqs */
	if (!freezable && pwq->max_active == wq->saved_max_active)
		return;

	spin_lock_irq(&pwq->pool->lock);

	if (!freezable || !(pwq->pool->flags & POOL_FREEZING)) {
		pwq->max_active = wq->saved_max_active;

		while (!list_empty(&pwq->delayed_works) &&
		       pwq->nr_active < pwq->max_active)
			pwq_activate_first_delayed(pwq);

		/*
		 * Need to kick a worker after thawed or an unbound wq's
		 * max_active is bumped.  It's a slow path.  Do it always.
		 */
		wake_up_worker(pwq->pool);
	} else {
		pwq->max_active = 0;
	}

	spin_unlock_irq(&pwq->pool->lock);
}

/* initialize newly alloced @pwq which is associated with @wq and @pool */
static void init_pwq(struct pool_workqueue *pwq, struct workqueue_struct *wq,
		     struct worker_pool *pool)
{
	BUG_ON((unsigned long)pwq & WORK_STRUCT_FLAG_MASK);

	memset(pwq, 0, sizeof(*pwq));

	pwq->pool = pool;
	pwq->wq = wq;
	pwq->flush_color = -1;
	pwq->refcnt = 1;
	INIT_LIST_HEAD(&pwq->delayed_works);
	INIT_LIST_HEAD(&pwq->pwqs_node);
	INIT_LIST_HEAD(&pwq->mayday_node);
	INIT_WORK(&pwq->unbound_release_work, pwq_unbound_release_workfn);
}

/* sync @pwq with the current state of its associated wq and link it */
static void link_pwq(struct pool_workqueue *pwq)
{
	struct workqueue_struct *wq = pwq->wq;

	lockdep_assert_held(&wq->mutex);

	/* may be called multiple times, ignore if already linked */
	if (!list_empty(&pwq->pwqs_node))
		return;

	/*
	 * Set the matching work_color.  This is synchronized with
	 * wq->mutex to avoid confusing flush_workqueue().
	 */
	pwq->work_color = wq->work_color;

	/* sync max_active to the current setting */
	pwq_adjust_max_active(pwq);

	/* link in @pwq */
	list_add_rcu(&pwq->pwqs_node, &wq->pwqs);
}

/* obtain a pool matching @attr and create a pwq associating the pool and @wq */
static struct pool_workqueue *alloc_unbound_pwq(struct workqueue_struct *wq,
					const struct workqueue_attrs *attrs)
{
	struct worker_pool *pool;
	struct pool_workqueue *pwq;

	lockdep_assert_held(&wq_pool_mutex);

	pool = get_unbound_pool(attrs);
	if (!pool)
		return NULL;

	pwq = kmem_cache_alloc_node(pwq_cache, GFP_KERNEL, pool->node);
	if (!pwq) {
		put_unbound_pool(pool);
		return NULL;
	}

	init_pwq(pwq, wq, pool);
	return pwq;
}

/* undo alloc_unbound_pwq(), used only in the error path */
static void free_unbound_pwq(struct pool_workqueue *pwq)
{
	lockdep_assert_held(&wq_pool_mutex);

	if (pwq) {
		put_unbound_pool(pwq->pool);
		kmem_cache_free(pwq_cache, pwq);
	}
}

/**
 * wq_calc_node_mask - calculate a wq_attrs' cpumask for the specified node
 * @attrs: the wq_attrs of interest
 * @node: the target NUMA node
 * @cpu_going_down: if >= 0, the CPU to consider as offline
 * @cpumask: outarg, the resulting cpumask
 *
 * Calculate the cpumask a workqueue with @attrs should use on @node.  If
 * @cpu_going_down is >= 0, that cpu is considered offline during
 * calculation.  The result is stored in @cpumask.  This function returns
 * %true if the resulting @cpumask is different from @attrs->cpumask,
 * %false if equal.
 *
 * If NUMA affinity is not enabled, @attrs->cpumask is always used.  If
 * enabled and @node has online CPUs requested by @attrs, the returned
 * cpumask is the intersection of the possible CPUs of @node and
 * @attrs->cpumask.
 *
 * The caller is responsible for ensuring that the cpumask of @node stays
 * stable.
 */
static bool wq_calc_node_cpumask(const struct workqueue_attrs *attrs, int node,
				 int cpu_going_down, cpumask_t *cpumask)
{
	if (!wq_numa_enabled || attrs->no_numa)
		goto use_dfl;

	/* does @node have any online CPUs @attrs wants? */
	cpumask_and(cpumask, cpumask_of_node(node), attrs->cpumask);
	if (cpu_going_down >= 0)
		cpumask_clear_cpu(cpu_going_down, cpumask);

	if (cpumask_empty(cpumask))
		goto use_dfl;

	/* yeap, return possible CPUs in @node that @attrs wants */
	cpumask_and(cpumask, attrs->cpumask, wq_numa_possible_cpumask[node]);
	return !cpumask_equal(cpumask, attrs->cpumask);

use_dfl:
	cpumask_copy(cpumask, attrs->cpumask);
	return false;
}

/* install @pwq into @wq's numa_pwq_tbl[] for @node and return the old pwq */
static struct pool_workqueue *numa_pwq_tbl_install(struct workqueue_struct *wq,
						   int node,
						   struct pool_workqueue *pwq)
{
	struct pool_workqueue *old_pwq;

	lockdep_assert_held(&wq->mutex);

	/* link_pwq() can handle duplicate calls */
	link_pwq(pwq);

	old_pwq = rcu_access_pointer(wq->numa_pwq_tbl[node]);
	rcu_assign_pointer(wq->numa_pwq_tbl[node], pwq);
	return old_pwq;
}

/**
 * apply_workqueue_attrs - apply new workqueue_attrs to an unbound workqueue
 * @wq: the target workqueue
 * @attrs: the workqueue_attrs to apply, allocated with alloc_workqueue_attrs()
 *
 * Apply @attrs to an unbound workqueue @wq.  Unless disabled, on NUMA
 * machines, this function maps a separate pwq to each NUMA node with
 * possibles CPUs in @attrs->cpumask so that work items are affine to the
 * NUMA node it was issued on.  Older pwqs are released as in-flight work
 * items finish.  Note that a work item which repeatedly requeues itself
 * back-to-back will stay on its current pwq.
 *
 * Performs GFP_KERNEL allocations.  Returns 0 on success and -errno on
 * failure.
 */
int apply_workqueue_attrs(struct workqueue_struct *wq,
			  const struct workqueue_attrs *attrs)
{
	struct workqueue_attrs *new_attrs, *tmp_attrs;
	struct pool_workqueue **pwq_tbl, *dfl_pwq;
	int node, ret;

	/* only unbound workqueues can change attributes */
	if (WARN_ON(!(wq->flags & WQ_UNBOUND)))
		return -EINVAL;

	/* creating multiple pwqs breaks ordering guarantee */
	if (WARN_ON((wq->flags & __WQ_ORDERED) && !list_empty(&wq->pwqs)))
		return -EINVAL;

	pwq_tbl = kzalloc(wq_numa_tbl_len * sizeof(pwq_tbl[0]), GFP_KERNEL);
	new_attrs = alloc_workqueue_attrs(GFP_KERNEL);
	tmp_attrs = alloc_workqueue_attrs(GFP_KERNEL);
	if (!pwq_tbl || !new_attrs || !tmp_attrs)
		goto enomem;

	/* make a copy of @attrs and sanitize it */
	copy_workqueue_attrs(new_attrs, attrs);
	cpumask_and(new_attrs->cpumask, new_attrs->cpumask, cpu_possible_mask);

	/*
	 * We may create multiple pwqs with differing cpumasks.  Make a
	 * copy of @new_attrs which will be modified and used to obtain
	 * pools.
	 */
	copy_workqueue_attrs(tmp_attrs, new_attrs);

	/*
	 * CPUs should stay stable across pwq creations and installations.
	 * Pin CPUs, determine the target cpumask for each node and create
	 * pwqs accordingly.
	 */
	get_online_cpus();

	mutex_lock(&wq_pool_mutex);

	/*
	 * If something goes wrong during CPU up/down, we'll fall back to
	 * the default pwq covering whole @attrs->cpumask.  Always create
	 * it even if we don't use it immediately.
	 */
	dfl_pwq = alloc_unbound_pwq(wq, new_attrs);
	if (!dfl_pwq)
		goto enomem_pwq;

	for_each_node(node) {
		if (wq_calc_node_cpumask(attrs, node, -1, tmp_attrs->cpumask)) {
			pwq_tbl[node] = alloc_unbound_pwq(wq, tmp_attrs);
			if (!pwq_tbl[node])
				goto enomem_pwq;
		} else {
			dfl_pwq->refcnt++;
			pwq_tbl[node] = dfl_pwq;
		}
	}

	mutex_unlock(&wq_pool_mutex);

	/* all pwqs have been created successfully, let's install'em */
	mutex_lock(&wq->mutex);

	copy_workqueue_attrs(wq->unbound_attrs, new_attrs);

	/* save the previous pwq and install the new one */
	for_each_node(node)
		pwq_tbl[node] = numa_pwq_tbl_install(wq, node, pwq_tbl[node]);

	/* @dfl_pwq might not have been used, ensure it's linked */
	link_pwq(dfl_pwq);
	swap(wq->dfl_pwq, dfl_pwq);

	mutex_unlock(&wq->mutex);

	/* put the old pwqs */
	for_each_node(node)
		put_pwq_unlocked(pwq_tbl[node]);
	put_pwq_unlocked(dfl_pwq);

	put_online_cpus();
	ret = 0;
	/* fall through */
out_free:
	free_workqueue_attrs(tmp_attrs);
	free_workqueue_attrs(new_attrs);
	kfree(pwq_tbl);
	return ret;

enomem_pwq:
	free_unbound_pwq(dfl_pwq);
	for_each_node(node)
		if (pwq_tbl && pwq_tbl[node] != dfl_pwq)
			free_unbound_pwq(pwq_tbl[node]);
	mutex_unlock(&wq_pool_mutex);
	put_online_cpus();
enomem:
	ret = -ENOMEM;
	goto out_free;
}

/**
 * wq_update_unbound_numa - update NUMA affinity of a wq for CPU hot[un]plug
 * @wq: the target workqueue
 * @cpu: the CPU coming up or going down
 * @online: whether @cpu is coming up or going down
 *
 * This function is to be called from %CPU_DOWN_PREPARE, %CPU_ONLINE and
 * %CPU_DOWN_FAILED.  @cpu is being hot[un]plugged, update NUMA affinity of
 * @wq accordingly.
 *
 * If NUMA affinity can't be adjusted due to memory allocation failure, it
 * falls back to @wq->dfl_pwq which may not be optimal but is always
 * correct.
 *
 * Note that when the last allowed CPU of a NUMA node goes offline for a
 * workqueue with a cpumask spanning multiple nodes, the workers which were
 * already executing the work items for the workqueue will lose their CPU
 * affinity and may execute on any CPU.  This is similar to how per-cpu
 * workqueues behave on CPU_DOWN.  If a workqueue user wants strict
 * affinity, it's the user's responsibility to flush the work item from
 * CPU_DOWN_PREPARE.
 */
static void wq_update_unbound_numa(struct workqueue_struct *wq, int cpu,
				   bool online)
{
	int node = cpu_to_node(cpu);
	int cpu_off = online ? -1 : cpu;
	struct pool_workqueue *old_pwq = NULL, *pwq;
	struct workqueue_attrs *target_attrs;
	cpumask_t *cpumask;

	lockdep_assert_held(&wq_pool_mutex);

	if (!wq_numa_enabled || !(wq->flags & WQ_UNBOUND))
		return;

	/*
	 * We don't wanna alloc/free wq_attrs for each wq for each CPU.
	 * Let's use a preallocated one.  The following buf is protected by
	 * CPU hotplug exclusion.
	 */
	target_attrs = wq_update_unbound_numa_attrs_buf;
	cpumask = target_attrs->cpumask;

	mutex_lock(&wq->mutex);
	if (wq->unbound_attrs->no_numa)
		goto out_unlock;

	copy_workqueue_attrs(target_attrs, wq->unbound_attrs);
	pwq = unbound_pwq_by_node(wq, node);

	/*
	 * Let's determine what needs to be done.  If the target cpumask is
	 * different from wq's, we need to compare it to @pwq's and create
	 * a new one if they don't match.  If the target cpumask equals
	 * wq's, the default pwq should be used.  If @pwq is already the
	 * default one, nothing to do; otherwise, install the default one.
	 */
	if (wq_calc_node_cpumask(wq->unbound_attrs, node, cpu_off, cpumask)) {
		if (cpumask_equal(cpumask, pwq->pool->attrs->cpumask))
			goto out_unlock;
	} else {
		if (pwq == wq->dfl_pwq)
			goto out_unlock;
		else
			goto use_dfl_pwq;
	}

	mutex_unlock(&wq->mutex);

	/* create a new pwq */
	pwq = alloc_unbound_pwq(wq, target_attrs);
	if (!pwq) {
		pr_warning("workqueue: allocation failed while updating NUMA affinity of \"%s\"\n",
			   wq->name);
		goto out_unlock;
	}

	/*
	 * Install the new pwq.  As this function is called only from CPU
	 * hotplug callbacks and applying a new attrs is wrapped with
	 * get/put_online_cpus(), @wq->unbound_attrs couldn't have changed
	 * inbetween.
	 */
	mutex_lock(&wq->mutex);
	old_pwq = numa_pwq_tbl_install(wq, node, pwq);
	goto out_unlock;

use_dfl_pwq:
	spin_lock_irq(&wq->dfl_pwq->pool->lock);
	get_pwq(wq->dfl_pwq);
	spin_unlock_irq(&wq->dfl_pwq->pool->lock);
	old_pwq = numa_pwq_tbl_install(wq, node, wq->dfl_pwq);
out_unlock:
	mutex_unlock(&wq->mutex);
	put_pwq_unlocked(old_pwq);
}

static int alloc_and_link_pwqs(struct workqueue_struct *wq)
{
	bool highpri = wq->flags & WQ_HIGHPRI;
	int cpu;

	if (!(wq->flags & WQ_UNBOUND)) {
		wq->cpu_pwqs = alloc_percpu(struct pool_workqueue);
		if (!wq->cpu_pwqs)
			return -ENOMEM;

		for_each_possible_cpu(cpu) {
			struct pool_workqueue *pwq =
				per_cpu_ptr(wq->cpu_pwqs, cpu);
			struct worker_pool *cpu_pools =
				per_cpu(cpu_worker_pools, cpu);

			init_pwq(pwq, wq, &cpu_pools[highpri]);

			mutex_lock(&wq->mutex);
			link_pwq(pwq);
			mutex_unlock(&wq->mutex);
		}
		return 0;
	} else {
		return apply_workqueue_attrs(wq, unbound_std_wq_attrs[highpri]);
	}
}

static int wq_clamp_max_active(int max_active, unsigned int flags,
			       const char *name)
{
	int lim = flags & WQ_UNBOUND ? WQ_UNBOUND_MAX_ACTIVE : WQ_MAX_ACTIVE;

	if (max_active < 1 || max_active > lim)
		pr_warn("workqueue: max_active %d requested for %s is out of range, clamping between %d and %d\n",
			max_active, name, 1, lim);

	return clamp_val(max_active, 1, lim);
}

struct workqueue_struct *__alloc_workqueue_key(const char *fmt,
					       unsigned int flags,
					       int max_active,
					       struct lock_class_key *key,
					       const char *lock_name, ...)
{
	size_t tbl_size = 0;
	va_list args;
	struct workqueue_struct *wq;
	struct pool_workqueue *pwq;

	/* allocate wq and format name */
	if (flags & WQ_UNBOUND)
		tbl_size = wq_numa_tbl_len * sizeof(wq->numa_pwq_tbl[0]);

	wq = kzalloc(sizeof(*wq) + tbl_size, GFP_KERNEL);
	if (!wq)
		return NULL;

	if (flags & WQ_UNBOUND) {
		wq->unbound_attrs = alloc_workqueue_attrs(GFP_KERNEL);
		if (!wq->unbound_attrs)
			goto err_free_wq;
	}

	va_start(args, lock_name);
	vsnprintf(wq->name, sizeof(wq->name), fmt, args);
	va_end(args);

	max_active = max_active ?: WQ_DFL_ACTIVE;
	max_active = wq_clamp_max_active(max_active, flags, wq->name);

	/* init wq */
	wq->flags = flags;
	wq->saved_max_active = max_active;
	mutex_init(&wq->mutex);
	atomic_set(&wq->nr_pwqs_to_flush, 0);
	INIT_LIST_HEAD(&wq->pwqs);
	INIT_LIST_HEAD(&wq->flusher_queue);
	INIT_LIST_HEAD(&wq->flusher_overflow);
	INIT_LIST_HEAD(&wq->maydays);

	lockdep_init_map(&wq->lockdep_map, lock_name, key, 0);
	INIT_LIST_HEAD(&wq->list);

	if (alloc_and_link_pwqs(wq) < 0)
		goto err_free_wq;

	/*
	 * Workqueues which may be used during memory reclaim should
	 * have a rescuer to guarantee forward progress.
	 */
	if (flags & WQ_MEM_RECLAIM) {
		struct worker *rescuer;

		rescuer = alloc_worker();
		if (!rescuer)
			goto err_destroy;

		rescuer->rescue_wq = wq;
		rescuer->task = kthread_create(rescuer_thread, rescuer, "%s",
					       wq->name);
		if (IS_ERR(rescuer->task)) {
			kfree(rescuer);
			goto err_destroy;
		}

		wq->rescuer = rescuer;
		rescuer->task->flags |= PF_NO_SETAFFINITY;
		wake_up_process(rescuer->task);
	}

	if ((wq->flags & WQ_SYSFS) && workqueue_sysfs_register(wq))
		goto err_destroy;

	/*
	 * wq_pool_mutex protects global freeze state and workqueues list.
	 * Grab it, adjust max_active and add the new @wq to workqueues
	 * list.
	 */
	mutex_lock(&wq_pool_mutex);

	mutex_lock(&wq->mutex);
	for_each_pwq(pwq, wq)
		pwq_adjust_max_active(pwq);
	mutex_unlock(&wq->mutex);

	list_add(&wq->list, &workqueues);

	mutex_unlock(&wq_pool_mutex);

	return wq;

err_free_wq:
	free_workqueue_attrs(wq->unbound_attrs);
	kfree(wq);
	return NULL;
err_destroy:
	destroy_workqueue(wq);
	return NULL;
}
EXPORT_SYMBOL_GPL(__alloc_workqueue_key);

/**
 * destroy_workqueue - safely terminate a workqueue
 * @wq: target workqueue
 *
 * Safely destroy a workqueue. All work currently pending will be done first.
 */
void destroy_workqueue(struct workqueue_struct *wq)
{
	struct pool_workqueue *pwq;
	int node;

	/* drain it before proceeding with destruction */
	drain_workqueue(wq);

	/* sanity checks */
	mutex_lock(&wq->mutex);
	for_each_pwq(pwq, wq) {
		int i;

		for (i = 0; i < WORK_NR_COLORS; i++) {
			if (WARN_ON(pwq->nr_in_flight[i])) {
				mutex_unlock(&wq->mutex);
				return;
			}
		}

		if (WARN_ON((pwq != wq->dfl_pwq) && (pwq->refcnt > 1)) ||
		    WARN_ON(pwq->nr_active) ||
		    WARN_ON(!list_empty(&pwq->delayed_works))) {
			mutex_unlock(&wq->mutex);
			return;
		}
	}
	mutex_unlock(&wq->mutex);

	/*
	 * wq list is used to freeze wq, remove from list after
	 * flushing is complete in case freeze races us.
	 */
	mutex_lock(&wq_pool_mutex);
	list_del_init(&wq->list);
	mutex_unlock(&wq_pool_mutex);

	workqueue_sysfs_unregister(wq);

	if (wq->rescuer) {
		kthread_stop(wq->rescuer->task);
		kfree(wq->rescuer);
		wq->rescuer = NULL;
	}

	if (!(wq->flags & WQ_UNBOUND)) {
		/*
		 * The base ref is never dropped on per-cpu pwqs.  Directly
		 * free the pwqs and wq.
		 */
		free_percpu(wq->cpu_pwqs);
		kfree(wq);
	} else {
		/*
		 * We're the sole accessor of @wq at this point.  Directly
		 * access numa_pwq_tbl[] and dfl_pwq to put the base refs.
		 * @wq will be freed when the last pwq is released.
		 */
		for_each_node(node) {
			pwq = rcu_access_pointer(wq->numa_pwq_tbl[node]);
			RCU_INIT_POINTER(wq->numa_pwq_tbl[node], NULL);
			put_pwq_unlocked(pwq);
		}

		/*
		 * Put dfl_pwq.  @wq may be freed any time after dfl_pwq is
		 * put.  Don't access it afterwards.
		 */
		pwq = wq->dfl_pwq;
		wq->dfl_pwq = NULL;
		put_pwq_unlocked(pwq);
	}
}
EXPORT_SYMBOL_GPL(destroy_workqueue);

/**
 * workqueue_set_max_active - adjust max_active of a workqueue
 * @wq: target workqueue
 * @max_active: new max_active value.
 *
 * Set max_active of @wq to @max_active.
 *
 * CONTEXT:
 * Don't call from IRQ context.
 */
void workqueue_set_max_active(struct workqueue_struct *wq, int max_active)
{
	struct pool_workqueue *pwq;

	/* disallow meddling with max_active for ordered workqueues */
	if (WARN_ON(wq->flags & __WQ_ORDERED))
		return;

	max_active = wq_clamp_max_active(max_active, wq->flags, wq->name);

	mutex_lock(&wq->mutex);

	wq->saved_max_active = max_active;

	for_each_pwq(pwq, wq)
		pwq_adjust_max_active(pwq);

	mutex_unlock(&wq->mutex);
}
EXPORT_SYMBOL_GPL(workqueue_set_max_active);

/**
 * current_is_workqueue_rescuer - is %current workqueue rescuer?
 *
 * Determine whether %current is a workqueue rescuer.  Can be used from
 * work functions to determine whether it's being run off the rescuer task.
 */
bool current_is_workqueue_rescuer(void)
{
	struct worker *worker = current_wq_worker();

	return worker && worker->rescue_wq;
}

/**
 * workqueue_congested - test whether a workqueue is congested
 * @cpu: CPU in question
 * @wq: target workqueue
 *
 * Test whether @wq's cpu workqueue for @cpu is congested.  There is
 * no synchronization around this function and the test result is
 * unreliable and only useful as advisory hints or for debugging.
 *
 * If @cpu is WORK_CPU_UNBOUND, the test is performed on the local CPU.
 * Note that both per-cpu and unbound workqueues may be associated with
 * multiple pool_workqueues which have separate congested states.  A
 * workqueue being congested on one CPU doesn't mean the workqueue is also
 * contested on other CPUs / NUMA nodes.
 *
 * RETURNS:
 * %true if congested, %false otherwise.
 */
bool workqueue_congested(int cpu, struct workqueue_struct *wq)
{
	struct pool_workqueue *pwq;
	bool ret;

	rcu_read_lock_sched();

	if (cpu == WORK_CPU_UNBOUND)
		cpu = smp_processor_id();

	if (!(wq->flags & WQ_UNBOUND))
		pwq = per_cpu_ptr(wq->cpu_pwqs, cpu);
	else
		pwq = unbound_pwq_by_node(wq, cpu_to_node(cpu));

	ret = !list_empty(&pwq->delayed_works);
	rcu_read_unlock_sched();

	return ret;
}
EXPORT_SYMBOL_GPL(workqueue_congested);

/**
 * work_busy - test whether a work is currently pending or running
 * @work: the work to be tested
 *
 * Test whether @work is currently pending or running.  There is no
 * synchronization around this function and the test result is
 * unreliable and only useful as advisory hints or for debugging.
 *
 * RETURNS:
 * OR'd bitmask of WORK_BUSY_* bits.
 */
unsigned int work_busy(struct work_struct *work)
{
	struct worker_pool *pool;
	unsigned long flags;
	unsigned int ret = 0;

	if (work_pending(work))
		ret |= WORK_BUSY_PENDING;

	local_irq_save(flags);
	pool = get_work_pool(work);
	if (pool) {
		spin_lock(&pool->lock);
		if (find_worker_executing_work(pool, work))
			ret |= WORK_BUSY_RUNNING;
		spin_unlock(&pool->lock);
	}
	local_irq_restore(flags);

	return ret;
}
EXPORT_SYMBOL_GPL(work_busy);

/**
 * set_worker_desc - set description for the current work item
 * @fmt: printf-style format string
 * @...: arguments for the format string
 *
 * This function can be called by a running work function to describe what
 * the work item is about.  If the worker task gets dumped, this
 * information will be printed out together to help debugging.  The
 * description can be at most WORKER_DESC_LEN including the trailing '\0'.
 */
void set_worker_desc(const char *fmt, ...)
{
	struct worker *worker = current_wq_worker();
	va_list args;

	if (worker) {
		va_start(args, fmt);
		vsnprintf(worker->desc, sizeof(worker->desc), fmt, args);
		va_end(args);
		worker->desc_valid = true;
	}
}

/**
 * print_worker_info - print out worker information and description
 * @log_lvl: the log level to use when printing
 * @task: target task
 *
 * If @task is a worker and currently executing a work item, print out the
 * name of the workqueue being serviced and worker description set with
 * set_worker_desc() by the currently executing work item.
 *
 * This function can be safely called on any task as long as the
 * task_struct itself is accessible.  While safe, this function isn't
 * synchronized and may print out mixups or garbages of limited length.
 */
void print_worker_info(const char *log_lvl, struct task_struct *task)
{
	work_func_t *fn = NULL;
	char name[WQ_NAME_LEN] = { };
	char desc[WORKER_DESC_LEN] = { };
	struct pool_workqueue *pwq = NULL;
	struct workqueue_struct *wq = NULL;
	bool desc_valid = false;
	struct worker *worker;

	if (!(task->flags & PF_WQ_WORKER))
		return;

	/*
	 * This function is called without any synchronization and @task
	 * could be in any state.  Be careful with dereferences.
	 */
	worker = probe_kthread_data(task);

	/*
	 * Carefully copy the associated workqueue's workfn and name.  Keep
	 * the original last '\0' in case the original contains garbage.
	 */
	probe_kernel_read(&fn, &worker->current_func, sizeof(fn));
	probe_kernel_read(&pwq, &worker->current_pwq, sizeof(pwq));
	probe_kernel_read(&wq, &pwq->wq, sizeof(wq));
	probe_kernel_read(name, wq->name, sizeof(name) - 1);

	/* copy worker description */
	probe_kernel_read(&desc_valid, &worker->desc_valid, sizeof(desc_valid));
	if (desc_valid)
		probe_kernel_read(desc, worker->desc, sizeof(desc) - 1);

	if (fn || name[0] || desc[0]) {
		printk("%sWorkqueue: %s %pf", log_lvl, name, fn);
		if (desc[0])
			pr_cont(" (%s)", desc);
		pr_cont("\n");
	}
}

/*
 * CPU hotplug.
 *
 * There are two challenges in supporting CPU hotplug.  Firstly, there
 * are a lot of assumptions on strong associations among work, cwq and
 * gcwq which make migrating pending and scheduled works very
 * difficult to implement without impacting hot paths.  Secondly,
 * gcwqs serve mix of short, long and very long running works making
 * blocked draining impractical.
 *
 * This is solved by allowing a gcwq to be detached from CPU, running
 * it with unbound (rogue) workers and allowing it to be reattached
 * later if the cpu comes back online.  A separate thread is created
 * to govern a gcwq in such state and is called the trustee of the
 * gcwq.
 *
 * Trustee states and their descriptions.
 *
 * START	Command state used on startup.  On CPU_DOWN_PREPARE, a
 *		new trustee is started with this state.
 *
 * IN_CHARGE	Once started, trustee will enter this state after
 *		assuming the manager role and making all existing
 *		workers rogue.  DOWN_PREPARE waits for trustee to
 *		enter this state.  After reaching IN_CHARGE, trustee
 *		tries to execute the pending worklist until it's empty
 *		and the state is set to BUTCHER, or the state is set
 *		to RELEASE.
 *
 * BUTCHER	Command state which is set by the cpu callback after
 *		the cpu has went down.  Once this state is set trustee
 *		knows that there will be no new works on the worklist
 *		and once the worklist is empty it can proceed to
 *		killing idle workers.
 *
 * RELEASE	Command state which is set by the cpu callback if the
 *		cpu down has been canceled or it has come online
 *		again.  After recognizing this state, trustee stops
 *		trying to drain or butcher and clears ROGUE, rebinds
 *		all remaining workers back to the cpu and releases
 *		manager role.
 *
 * DONE		Trustee will enter this state after BUTCHER or RELEASE
 *		is complete.
 *
 *          trustee                 CPU                draining
 *         took over                down               complete
 * START -----------> IN_CHARGE -----------> BUTCHER -----------> DONE
 *                        |                     |                  ^
 *                        | CPU is back online  v   return workers |
 *                         ----------------> RELEASE --------------
 */

/**
 * trustee_wait_event_timeout - timed event wait for trustee
 * @cond: condition to wait for
 * @timeout: timeout in jiffies
 *
 * wait_event_timeout() for trustee to use.  Handles locking and
 * checks for RELEASE request.
 *
 * CONTEXT:
 * spin_lock_irq(gcwq->lock) which may be released and regrabbed
 * multiple times.  To be used by trustee.
 *
 * RETURNS:
 * Positive indicating left time if @cond is satisfied, 0 if timed
 * out, -1 if canceled.
 */
#define trustee_wait_event_timeout(cond, timeout) ({			\
	long __ret = (timeout);						\
	while (!((cond) || (gcwq->trustee_state == TRUSTEE_RELEASE)) &&	\
	       __ret) {							\
		spin_unlock_irq(&gcwq->lock);				\
		__wait_event_timeout(gcwq->trustee_wait, (cond) ||	\
			(gcwq->trustee_state == TRUSTEE_RELEASE),	\
			__ret);						\
		spin_lock_irq(&gcwq->lock);				\
	}								\
	gcwq->trustee_state == TRUSTEE_RELEASE ? -1 : (__ret);		\
})

/**
 * trustee_wait_event - event wait for trustee
 * @cond: condition to wait for
 *
 * wait_event() for trustee to use.  Automatically handles locking and
 * checks for CANCEL request.
 *
 * CONTEXT:
 * spin_lock_irq(gcwq->lock) which may be released and regrabbed
 * multiple times.  To be used by trustee.
 *
 * RETURNS:
 * 0 if @cond is satisfied, -1 if canceled.
 */
#define trustee_wait_event(cond) ({					\
	long __ret1;							\
	__ret1 = trustee_wait_event_timeout(cond, MAX_SCHEDULE_TIMEOUT);\
	__ret1 < 0 ? -1 : 0;						\
})

static bool gcwq_is_managing_workers(struct global_cwq *gcwq)
{
	struct worker_pool *pool;

	for_each_worker_pool(pool, gcwq)
		if (pool->flags & POOL_MANAGING_WORKERS)
			return true;
	return false;
}

static bool gcwq_has_idle_workers(struct global_cwq *gcwq)
{
	struct worker_pool *pool;

	for_each_worker_pool(pool, gcwq)
		if (!list_empty(&pool->idle_list))
			return true;
	return false;
}

static int __cpuinit trustee_thread(void *__gcwq)
{
	struct global_cwq *gcwq = __gcwq;
	struct worker_pool *pool;
	struct worker *worker;
	struct work_struct *work;
	struct hlist_node *pos;
	long rc;
	int i;

	BUG_ON(gcwq->cpu != smp_processor_id());

	spin_lock_irq(&gcwq->lock);
	/*
	 * Claim the manager position and make all workers rogue.
	 * Trustee must be bound to the target cpu and can't be
	 * cancelled.
	 */
	BUG_ON(gcwq->cpu != smp_processor_id());
	rc = trustee_wait_event(!gcwq_is_managing_workers(gcwq));
	BUG_ON(rc < 0);

	for_each_worker_pool(pool, gcwq) {
		pool->flags |= POOL_MANAGING_WORKERS;

		list_for_each_entry(worker, &pool->idle_list, entry)
			worker->flags |= WORKER_ROGUE;
	}

	for_each_busy_worker(worker, i, pos, gcwq)
		worker->flags |= WORKER_ROGUE;

	/*
	 * Call schedule() so that we cross rq->lock and thus can
	 * guarantee sched callbacks see the rogue flag.  This is
	 * necessary as scheduler callbacks may be invoked from other
	 * cpus.
	 */
	spin_unlock_irq(&gcwq->lock);
	schedule();
	spin_lock_irq(&gcwq->lock);

	/*
	 * Sched callbacks are disabled now.  Zap nr_running.  After
	 * this, nr_running stays zero and need_more_worker() and
	 * keep_working() are always true as long as the worklist is
	 * not empty.
	 */
	for_each_worker_pool(pool, gcwq)
		atomic_set(get_pool_nr_running(pool), 0);

	spin_unlock_irq(&gcwq->lock);
	for_each_worker_pool(pool, gcwq)
		del_timer_sync(&pool->idle_timer);
	spin_lock_irq(&gcwq->lock);

	/*
	 * We're now in charge.  Notify and proceed to drain.  We need
	 * to keep the gcwq running during the whole CPU down
	 * procedure as other cpu hotunplug callbacks may need to
	 * flush currently running tasks.
	 */
	gcwq->trustee_state = TRUSTEE_IN_CHARGE;
	wake_up_all(&gcwq->trustee_wait);

	/*
	 * The original cpu is in the process of dying and may go away
	 * anytime now.  When that happens, we and all workers would
	 * be migrated to other cpus.  Try draining any left work.  We
	 * want to get it over with ASAP - spam rescuers, wake up as
	 * many idlers as necessary and create new ones till the
	 * worklist is empty.  Note that if the gcwq is frozen, there
	 * may be frozen works in freezable cwqs.  Don't declare
	 * completion while frozen.
	 */
	while (true) {
		bool busy = false;

		for_each_worker_pool(pool, gcwq)
			busy |= pool->nr_workers != pool->nr_idle;

		if (!busy && !(gcwq->flags & GCWQ_FREEZING) &&
		    gcwq->trustee_state != TRUSTEE_IN_CHARGE)
			break;

		for_each_worker_pool(pool, gcwq) {
			int nr_works = 0;

			list_for_each_entry(work, &pool->worklist, entry) {
				send_mayday(work);
				nr_works++;
			}

			list_for_each_entry(worker, &pool->idle_list, entry) {
				if (!nr_works--)
					break;
				wake_up_process(worker->task);
			}

			if (need_to_create_worker(pool)) {
				spin_unlock_irq(&gcwq->lock);
				worker = create_worker(pool, false);
				spin_lock_irq(&gcwq->lock);
				if (worker) {
					worker->flags |= WORKER_ROGUE;
					start_worker(worker);
				}
			}
		}

		/* give a breather */
		if (trustee_wait_event_timeout(false, TRUSTEE_COOLDOWN) < 0)
			break;
	}

	/*
	 * Either all works have been scheduled and cpu is down, or
	 * cpu down has already been canceled.  Wait for and butcher
	 * all workers till we're canceled.
	 */
	do {
		rc = trustee_wait_event(gcwq_has_idle_workers(gcwq));

		i = 0;
		for_each_worker_pool(pool, gcwq) {
			while (!list_empty(&pool->idle_list)) {
				worker = list_first_entry(&pool->idle_list,
							  struct worker, entry);
				destroy_worker(worker);
			}
			i |= pool->nr_workers;
		}
	} while (i && rc >= 0);

	/*
	 * At this point, either draining has completed and no worker
	 * is left, or cpu down has been canceled or the cpu is being
	 * brought back up.  There shouldn't be any idle one left.
	 * Tell the remaining busy ones to rebind once it finishes the
	 * currently scheduled works by scheduling the rebind_work.
	 */
	for_each_worker_pool(pool, gcwq)
		WARN_ON(!list_empty(&pool->idle_list));

	for_each_busy_worker(worker, i, pos, gcwq) {
		struct work_struct *rebind_work = &worker->rebind_work;
		unsigned long worker_flags = worker->flags;

		/*
		 * Rebind_work may race with future cpu hotplug
		 * operations.  Use a separate flag to mark that
		 * rebinding is scheduled.  The morphing should
		 * be atomic.
		 */
		worker_flags |= WORKER_REBIND;
		worker_flags &= ~WORKER_ROGUE;
		ACCESS_ONCE(worker->flags) = worker_flags;

		/* queue rebind_work, wq doesn't matter, use the default one */
		if (test_and_set_bit(WORK_STRUCT_PENDING_BIT,
				     work_data_bits(rebind_work)))
			continue;

		debug_work_activate(rebind_work);
		insert_work(get_cwq(gcwq->cpu, system_wq), rebind_work,
			    worker->scheduled.next,
			    work_color_to_flags(WORK_NO_COLOR));
	}

	/* relinquish manager role */
	for_each_worker_pool(pool, gcwq)
		pool->flags &= ~POOL_MANAGING_WORKERS;

	/* notify completion */
	gcwq->trustee = NULL;
	gcwq->trustee_state = TRUSTEE_DONE;
	wake_up_all(&gcwq->trustee_wait);
	spin_unlock_irq(&gcwq->lock);
	return 0;
}

/**
 * wait_trustee_state - wait for trustee to enter the specified state
 * @gcwq: gcwq the trustee of interest belongs to
 * @state: target state to wait for
 *
 * Wait for the trustee to reach @state.  DONE is already matched.
 *
 * CONTEXT:
 * spin_lock_irq(gcwq->lock) which may be released and regrabbed
 * multiple times.  To be used by cpu_callback.
 */
static void __cpuinit wait_trustee_state(struct global_cwq *gcwq, int state)
__releases(&gcwq->lock)
__acquires(&gcwq->lock)
{
	if (!(gcwq->trustee_state == state ||
	      gcwq->trustee_state == TRUSTEE_DONE)) {
		spin_unlock_irq(&gcwq->lock);
		__wait_event(gcwq->trustee_wait,
			     gcwq->trustee_state == state ||
			     gcwq->trustee_state == TRUSTEE_DONE);
		spin_lock_irq(&gcwq->lock);
	}
}

static int __devinit workqueue_cpu_callback(struct notifier_block *nfb,
						unsigned long action,
						void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;
	struct global_cwq *gcwq = get_gcwq(cpu);
	struct task_struct *new_trustee = NULL;
	struct worker *new_workers[NR_WORKER_POOLS] = { };
	struct worker_pool *pool;
	unsigned long flags;
	int i;

	action &= ~CPU_TASKS_FROZEN;

	switch (action) {
	case CPU_DOWN_PREPARE:
		new_trustee = kthread_create(trustee_thread, gcwq,
					     "workqueue_trustee/%d\n", cpu);
		if (IS_ERR(new_trustee))
			return notifier_from_errno(PTR_ERR(new_trustee));
		kthread_bind(new_trustee, cpu);
		/* fall through */
	case CPU_UP_PREPARE:
		i = 0;
		for_each_worker_pool(pool, gcwq) {
			BUG_ON(pool->first_idle);
			new_workers[i] = create_worker(pool, false);
			if (!new_workers[i++])
				goto err_destroy;
		}
	}

	/* some are called w/ irq disabled, don't disturb irq status */
	spin_lock_irqsave(&gcwq->lock, flags);

	switch (action) {
	case CPU_DOWN_PREPARE:
		/* initialize trustee and tell it to acquire the gcwq */
		BUG_ON(gcwq->trustee || gcwq->trustee_state != TRUSTEE_DONE);
		gcwq->trustee = new_trustee;
		gcwq->trustee_state = TRUSTEE_START;
		wake_up_process(gcwq->trustee);
		wait_trustee_state(gcwq, TRUSTEE_IN_CHARGE);
		/* fall through */
	case CPU_UP_PREPARE:
		i = 0;
		for_each_worker_pool(pool, gcwq) {
			BUG_ON(pool->first_idle);
			pool->first_idle = new_workers[i++];
		}
		break;

	case CPU_DYING:
		/*
		 * Before this, the trustee and all workers except for
		 * the ones which are still executing works from
		 * before the last CPU down must be on the cpu.  After
		 * this, they'll all be diasporas.
		 */
		gcwq->flags |= GCWQ_DISASSOCIATED;
		break;

	case CPU_POST_DEAD:
		gcwq->trustee_state = TRUSTEE_BUTCHER;
		/* fall through */
	case CPU_UP_CANCELED:
		for_each_worker_pool(pool, gcwq) {
			destroy_worker(pool->first_idle);
			pool->first_idle = NULL;
		}
		break;

	case CPU_DOWN_FAILED:
	case CPU_ONLINE:
		gcwq->flags &= ~GCWQ_DISASSOCIATED;
		if (gcwq->trustee_state != TRUSTEE_DONE) {
			gcwq->trustee_state = TRUSTEE_RELEASE;
			wake_up_process(gcwq->trustee);
			wait_trustee_state(gcwq, TRUSTEE_DONE);
		}

		/*
		 * Trustee is done and there might be no worker left.
		 * Put the first_idle in and request a real manager to
		 * take a look.
		 */
		for_each_worker_pool(pool, gcwq) {
			spin_unlock_irq(&gcwq->lock);
			kthread_bind(pool->first_idle->task, cpu);
			spin_lock_irq(&gcwq->lock);
			pool->flags |= POOL_MANAGE_WORKERS;
			start_worker(pool->first_idle);
			pool->first_idle = NULL;
		}
		break;
	}

	spin_unlock_irqrestore(&gcwq->lock, flags);

	return notifier_from_errno(0);

err_destroy:
	if (new_trustee)
		kthread_stop(new_trustee);

	spin_lock_irqsave(&gcwq->lock, flags);
	for (i = 0; i < NR_WORKER_POOLS; i++)
		if (new_workers[i])
			destroy_worker(new_workers[i]);
	spin_unlock_irqrestore(&gcwq->lock, flags);

	return NOTIFY_BAD;
}

/**
 * rebind_workers - rebind all workers of a pool to the associated CPU
 * @pool: pool of interest
 *
 * @pool->cpu is coming online.  Rebind all workers to the CPU.
 */
static void rebind_workers(struct worker_pool *pool)
{
	struct worker *worker;
	int wi;

	lockdep_assert_held(&pool->manager_mutex);

	/*
	 * Restore CPU affinity of all workers.  As all idle workers should
	 * be on the run-queue of the associated CPU before any local
	 * wake-ups for concurrency management happen, restore CPU affinty
	 * of all workers first and then clear UNBOUND.  As we're called
	 * from CPU_ONLINE, the following shouldn't fail.
	 */
	for_each_pool_worker(worker, wi, pool)
		WARN_ON_ONCE(set_cpus_allowed_ptr(worker->task,
						  pool->attrs->cpumask) < 0);

	spin_lock_irq(&pool->lock);

	for_each_pool_worker(worker, wi, pool) {
		unsigned int worker_flags = worker->flags;

		/*
		 * A bound idle worker should actually be on the runqueue
		 * of the associated CPU for local wake-ups targeting it to
		 * work.  Kick all idle workers so that they migrate to the
		 * associated CPU.  Doing this in the same loop as
		 * replacing UNBOUND with REBOUND is safe as no worker will
		 * be bound before @pool->lock is released.
		 */
		if (worker_flags & WORKER_IDLE)
			wake_up_process(worker->task);

		/*
		 * We want to clear UNBOUND but can't directly call
		 * worker_clr_flags() or adjust nr_running.  Atomically
		 * replace UNBOUND with another NOT_RUNNING flag REBOUND.
		 * @worker will clear REBOUND using worker_clr_flags() when
		 * it initiates the next execution cycle thus restoring
		 * concurrency management.  Note that when or whether
		 * @worker clears REBOUND doesn't affect correctness.
		 *
		 * ACCESS_ONCE() is necessary because @worker->flags may be
		 * tested without holding any lock in
		 * wq_worker_waking_up().  Without it, NOT_RUNNING test may
		 * fail incorrectly leading to premature concurrency
		 * management operations.
		 */
		WARN_ON_ONCE(!(worker_flags & WORKER_UNBOUND));
		worker_flags |= WORKER_REBOUND;
		worker_flags &= ~WORKER_UNBOUND;
		ACCESS_ONCE(worker->flags) = worker_flags;
	}

	spin_unlock_irq(&pool->lock);
}

/**
 * restore_unbound_workers_cpumask - restore cpumask of unbound workers
 * @pool: unbound pool of interest
 * @cpu: the CPU which is coming up
 *
 * An unbound pool may end up with a cpumask which doesn't have any online
 * CPUs.  When a worker of such pool get scheduled, the scheduler resets
 * its cpus_allowed.  If @cpu is in @pool's cpumask which didn't have any
 * online CPU before, cpus_allowed of all its workers should be restored.
 */
static void restore_unbound_workers_cpumask(struct worker_pool *pool, int cpu)
{
	static cpumask_t cpumask;
	struct worker *worker;
	int wi;

	lockdep_assert_held(&pool->manager_mutex);

	/* is @cpu allowed for @pool? */
	if (!cpumask_test_cpu(cpu, pool->attrs->cpumask))
		return;

	/* is @cpu the only online CPU? */
	cpumask_and(&cpumask, pool->attrs->cpumask, cpu_online_mask);
	if (cpumask_weight(&cpumask) != 1)
		return;

	/* as we're called from CPU_ONLINE, the following shouldn't fail */
	for_each_pool_worker(worker, wi, pool)
		WARN_ON_ONCE(set_cpus_allowed_ptr(worker->task,
						  pool->attrs->cpumask) < 0);
}

/*
 * Workqueues should be brought up before normal priority CPU notifiers.
 * This will be registered high priority CPU notifier.
 */
static int __cpuinit workqueue_cpu_up_callback(struct notifier_block *nfb,
					       unsigned long action,
					       void *hcpu)
{
	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_UP_PREPARE:
	case CPU_UP_CANCELED:
	case CPU_DOWN_FAILED:
	case CPU_ONLINE:
		return workqueue_cpu_callback(nfb, action, hcpu);
	}
	return NOTIFY_OK;
}

/*
 * Workqueues should be brought down after normal priority CPU notifiers.
 * This will be registered as low priority CPU notifier.
 */
static int __cpuinit workqueue_cpu_down_callback(struct notifier_block *nfb,
						 unsigned long action,
						 void *hcpu)
{
	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_DOWN_PREPARE:
	case CPU_DYING:
	case CPU_POST_DEAD:
		return workqueue_cpu_callback(nfb, action, hcpu);
	}
	return NOTIFY_OK;
}

#ifdef CONFIG_SMP

struct work_for_cpu {
	struct work_struct work;
	long (*fn)(void *);
	void *arg;
	long ret;
};

static void work_for_cpu_fn(struct work_struct *work)
{
	struct work_for_cpu *wfc = container_of(work, struct work_for_cpu, work);

	wfc->ret = wfc->fn(wfc->arg);
}

/**
 * work_on_cpu - run a function in user context on a particular cpu
 * @cpu: the cpu to run on
 * @fn: the function to run
 * @arg: the function arg
 *
 * This will return the value @fn returns.
 * It is up to the caller to ensure that the cpu doesn't go offline.
 * The caller must not hold any locks which would prevent @fn from completing.
 */
long work_on_cpu(int cpu, long (*fn)(void *), void *arg)
{
	struct work_for_cpu wfc = { .fn = fn, .arg = arg };

	INIT_WORK_ONSTACK(&wfc.work, work_for_cpu_fn);
	schedule_work_on(cpu, &wfc.work);
	flush_work(&wfc.work);
	return wfc.ret;
}
EXPORT_SYMBOL_GPL(work_on_cpu);
#endif /* CONFIG_SMP */

#ifdef CONFIG_FREEZER

/**
 * freeze_workqueues_begin - begin freezing workqueues
 *
 * Start freezing workqueues.  After this function returns, all freezable
 * workqueues will queue new works to their frozen_works list instead of
 * pool->worklist.
 *
 * CONTEXT:
 * Grabs and releases workqueue_lock and pool->lock's.
 */
void freeze_workqueues_begin(void)
{
	unsigned int cpu;

	spin_lock(&workqueue_lock);

	BUG_ON(workqueue_freezing);
	workqueue_freezing = true;

	for_each_gcwq_cpu(cpu) {
		struct global_cwq *gcwq = get_gcwq(cpu);
		struct worker_pool *pool;
		struct workqueue_struct *wq;

		spin_lock_irq(&gcwq->lock);

		BUG_ON(gcwq->flags & GCWQ_FREEZING);
		gcwq->flags |= GCWQ_FREEZING;

		list_for_each_entry(wq, &workqueues, list) {
			struct cpu_workqueue_struct *cwq = get_cwq(cpu, wq);

			if (cwq && wq->flags & WQ_FREEZABLE)
				cwq->max_active = 0;
		}

		spin_unlock_irq(&gcwq->lock);
	}

	spin_unlock(&workqueue_lock);
}

/**
 * freeze_workqueues_busy - are freezable workqueues still busy?
 *
 * Check whether freezing is complete.  This function must be called
 * between freeze_workqueues_begin() and thaw_workqueues().
 *
 * CONTEXT:
 * Grabs and releases wq_pool_mutex.
 *
 * RETURNS:
 * %true if some freezable workqueues are still busy.  %false if freezing
 * is complete.
 */
bool freeze_workqueues_busy(void)
{
	bool busy = false;
	struct workqueue_struct *wq;
	struct pool_workqueue *pwq;

	mutex_lock(&wq_pool_mutex);

	WARN_ON_ONCE(!workqueue_freezing);

	list_for_each_entry(wq, &workqueues, list) {
		if (!(wq->flags & WQ_FREEZABLE))
			continue;
		/*
		 * nr_active is monotonically decreasing.  It's safe
		 * to peek without lock.
		 */
		rcu_read_lock_sched();
		for_each_pwq(pwq, wq) {
			WARN_ON_ONCE(pwq->nr_active < 0);
			if (pwq->nr_active) {
				busy = true;
				rcu_read_unlock_sched();
				goto out_unlock;
			}
		}
		rcu_read_unlock_sched();
	}
out_unlock:
	mutex_unlock(&wq_pool_mutex);
	return busy;
}

/**
 * thaw_workqueues - thaw workqueues
 *
 * Thaw workqueues.  Normal queueing is restored and all collected
 * frozen works are transferred to their respective pool worklists.
 *
 * CONTEXT:
 * Grabs and releases workqueue_lock and pool->lock's.
 */
void thaw_workqueues(void)
{
	unsigned int cpu;

	spin_lock(&workqueue_lock);

	if (!workqueue_freezing)
		goto out_unlock;

	for_each_wq_cpu(cpu) {
		struct worker_pool *pool;
		struct workqueue_struct *wq;

		for_each_std_worker_pool(pool, cpu) {
			spin_lock_irq(&pool->lock);

			WARN_ON_ONCE(!(pool->flags & POOL_FREEZING));
			pool->flags &= ~POOL_FREEZING;

			list_for_each_entry(wq, &workqueues, list) {
				struct pool_workqueue *pwq = get_pwq(cpu, wq);

				if (!pwq || pwq->pool != pool ||
				    !(wq->flags & WQ_FREEZABLE))
					continue;

				/* restore max_active and repopulate worklist */
				pwq_set_max_active(pwq, wq->saved_max_active);
			}

			wake_up_worker(pool);

			spin_unlock_irq(&pool->lock);
		}
	}

	workqueue_freezing = false;
out_unlock:
	spin_unlock(&workqueue_lock);
}
#endif /* CONFIG_FREEZER */

static int __init init_workqueues(void)
{
	unsigned int cpu;
	int i;

	/* make sure we have enough bits for OFFQ CPU number */
	BUILD_BUG_ON((1LU << (BITS_PER_LONG - WORK_OFFQ_CPU_SHIFT)) <
		     WORK_CPU_LAST);

	cpu_notifier(workqueue_cpu_up_callback, CPU_PRI_WORKQUEUE_UP);
	hotcpu_notifier(workqueue_cpu_down_callback, CPU_PRI_WORKQUEUE_DOWN);

	/* initialize gcwqs */
	for_each_gcwq_cpu(cpu) {
		struct global_cwq *gcwq = get_gcwq(cpu);
		struct worker_pool *pool;

		spin_lock_init(&gcwq->lock);
		gcwq->cpu = cpu;
		gcwq->flags |= GCWQ_DISASSOCIATED;

		for (i = 0; i < BUSY_WORKER_HASH_SIZE; i++)
			INIT_HLIST_HEAD(&gcwq->busy_hash[i]);

		for_each_worker_pool(pool, gcwq) {
			pool->gcwq = gcwq;
			INIT_LIST_HEAD(&pool->worklist);
			INIT_LIST_HEAD(&pool->idle_list);

			init_timer_deferrable(&pool->idle_timer);
			pool->idle_timer.function = idle_worker_timeout;
			pool->idle_timer.data = (unsigned long)pool;

			setup_timer(&pool->mayday_timer, gcwq_mayday_timeout,
				    (unsigned long)pool);

			ida_init(&pool->worker_ida);
		}

		gcwq->trustee_state = TRUSTEE_DONE;
		init_waitqueue_head(&gcwq->trustee_wait);
	}

	/* create the initial worker */
	for_each_online_gcwq_cpu(cpu) {
		struct global_cwq *gcwq = get_gcwq(cpu);
		struct worker_pool *pool;

		if (cpu != WORK_CPU_UNBOUND)
			gcwq->flags &= ~GCWQ_DISASSOCIATED;

		for_each_worker_pool(pool, gcwq) {
			struct worker *worker;

			worker = create_worker(pool, true);
			BUG_ON(!worker);
			spin_lock_irq(&gcwq->lock);
			start_worker(worker);
			spin_unlock_irq(&gcwq->lock);
		}
	}

	system_wq = alloc_workqueue("events", 0, 0);
	system_highpri_wq = alloc_workqueue("events_highpri", WQ_HIGHPRI, 0);
	system_long_wq = alloc_workqueue("events_long", 0, 0);
	system_unbound_wq = alloc_workqueue("events_unbound", WQ_UNBOUND,
					    WQ_UNBOUND_MAX_ACTIVE);
	system_freezable_wq = alloc_workqueue("events_freezable",
					      WQ_FREEZABLE, 0);
	BUG_ON(!system_wq || !system_highpri_wq || !system_long_wq ||
	       !system_unbound_wq || !system_freezable_wq);
	return 0;
}
early_initcall(init_workqueues);
