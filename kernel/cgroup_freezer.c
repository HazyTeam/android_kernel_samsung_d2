/*
 * cgroup_freezer.c -  control group freezer subsystem
 *
 * Copyright IBM Corporation, 2007
 *
 * Author : Cedric Le Goater <clg@fr.ibm.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2.1 of the GNU Lesser General Public License
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it would be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/export.h>
#include <linux/slab.h>
#include <linux/cgroup.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/freezer.h>
#include <linux/seq_file.h>

/*
 * A cgroup is freezing if any FREEZING flags are set.  FREEZING_SELF is
 * set if "FROZEN" is written to freezer.state cgroupfs file, and cleared
 * for "THAWED".  FREEZING_PARENT is set if the parent freezer is FREEZING
 * for whatever reason.  IOW, a cgroup has FREEZING_PARENT set if one of
 * its ancestors has FREEZING_SELF set.
 */
enum freezer_state_flags {
	CGROUP_FREEZER_ONLINE	= (1 << 0), /* freezer is fully online */
	CGROUP_FREEZING_SELF	= (1 << 1), /* this freezer is freezing */
	CGROUP_FREEZING_PARENT	= (1 << 2), /* the parent freezer is freezing */
	CGROUP_FROZEN		= (1 << 3), /* this and its descendants frozen */

	/* mask for all FREEZING flags */
	CGROUP_FREEZING		= CGROUP_FREEZING_SELF | CGROUP_FREEZING_PARENT,
};

struct freezer {
	struct cgroup_subsys_state	css;
	unsigned int			state;
	spinlock_t			lock;
};

static inline struct freezer *cgroup_freezer(struct cgroup *cgroup)
{
	return container_of(cgroup_subsys_state(cgroup, freezer_subsys_id),
			    struct freezer, css);
}

static inline struct freezer *task_freezer(struct task_struct *task)
{
	return container_of(task_subsys_state(task, freezer_subsys_id),
			    struct freezer, css);
}

static struct freezer *parent_freezer(struct freezer *freezer)
{
	struct cgroup *pcg = freezer->css.cgroup->parent;

	if (pcg)
		return cgroup_freezer(pcg);
	return NULL;
}

bool cgroup_freezing(struct task_struct *task)
{
	bool ret;

	rcu_read_lock();
	ret = task_freezer(task)->state & CGROUP_FREEZING;
	rcu_read_unlock();

	return ret;
}

/*
 * cgroups_write_string() limits the size of freezer state strings to
 * CGROUP_LOCAL_BUFFER_SIZE
 */
static const char *freezer_state_strs(unsigned int state)
{
	if (state & CGROUP_FROZEN)
		return "FROZEN";
	if (state & CGROUP_FREEZING)
		return "FREEZING";
	return "THAWED";
};

struct cgroup_subsys freezer_subsys;

static struct cgroup_subsys_state *freezer_css_alloc(struct cgroup *cgroup)
{
	struct freezer *freezer;

	freezer = kzalloc(sizeof(struct freezer), GFP_KERNEL);
	if (!freezer)
		return ERR_PTR(-ENOMEM);

	spin_lock_init(&freezer->lock);
	freezer->state = CGROUP_THAWED;
	return &freezer->css;
}

static void freezer_destroy(struct cgroup *cgroup)
{
	struct freezer *freezer = cgroup_freezer(cgroup);

	if (freezer->state != CGROUP_THAWED)
		atomic_dec(&system_freezing_cnt);
	kfree(freezer);
}

/* task is frozen or will freeze immediately when next it gets woken */
static bool is_task_frozen_enough(struct task_struct *task)
{
	return frozen(task) ||
		(task_is_stopped_or_traced(task) && freezing(task));
}

/*
 * The call to cgroup_lock() in the freezer.state write method prevents
 * a write to that file racing against an attach, and hence the
 * can_attach() result will remain valid until the attach completes.
 */
static int freezer_can_attach(struct cgroup *new_cgroup,
			      struct cgroup_taskset *tset)
{
	struct freezer *freezer;
	struct task_struct *task;

	/*
	 * Anything frozen can't move or be moved to/from.
	 */
	cgroup_taskset_for_each(task, new_cgroup, tset)
		if (cgroup_freezing(task))
			return -EBUSY;

	freezer = cgroup_freezer(new_cgroup);
	if (freezer->state != CGROUP_THAWED)
		return -EBUSY;

	return 0;
}

static void freezer_fork(struct task_struct *task)
{
	struct freezer *freezer;

	rcu_read_lock();
	freezer = task_freezer(task);

	/*
	 * The root cgroup is non-freezable, so we can skip the
	 * following check.
	 */
	if (!freezer->css.cgroup->parent)
		goto out;

	spin_lock_irq(&freezer->lock);
	BUG_ON(freezer->state == CGROUP_FROZEN);

	/* Locking avoids race with FREEZING -> THAWED transitions. */
	if (freezer->state == CGROUP_FREEZING)
		freeze_task(task);

	spin_unlock_irq(&freezer->lock);
out:
	rcu_read_unlock();
}

/*
 * caller must hold freezer->lock
 */
static void update_if_frozen(struct cgroup *cgroup,
				 struct freezer *freezer)
{
	struct cgroup_iter it;
	struct task_struct *task;
	unsigned int nfrozen = 0, ntotal = 0;
	enum freezer_state old_state = freezer->state;

	cgroup_iter_start(cgroup, &it);
	while ((task = cgroup_iter_next(cgroup, &it))) {
		ntotal++;
		if (freezing(task) && is_task_frozen_enough(task))
			nfrozen++;
	}

	if (old_state == CGROUP_THAWED) {
		BUG_ON(nfrozen > 0);
	} else if (old_state == CGROUP_FREEZING) {
		if (nfrozen == ntotal)
			freezer->state = CGROUP_FROZEN;
	} else { /* old_state == CGROUP_FROZEN */
		BUG_ON(nfrozen != ntotal);
	}

	cgroup_iter_end(cgroup, &it);
}

static int freezer_read(struct cgroup *cgroup, struct cftype *cft,
			struct seq_file *m)
{
	struct freezer *freezer;
	enum freezer_state state;

	if (!cgroup_lock_live_group(cgroup))
		return -ENODEV;

	freezer = cgroup_freezer(cgroup);
	spin_lock_irq(&freezer->lock);
	state = freezer->state;
	if (state == CGROUP_FREEZING) {
		/* We change from FREEZING to FROZEN lazily if the cgroup was
		 * only partially frozen when we exitted write. */
		update_if_frozen(cgroup, freezer);
		state = freezer->state;
	}
	spin_unlock_irq(&freezer->lock);
	cgroup_unlock();

	seq_puts(m, freezer_state_strs[state]);
	seq_putc(m, '\n');
	return 0;
}

static int try_to_freeze_cgroup(struct cgroup *cgroup, struct freezer *freezer)
{
	struct cgroup_iter it;
	struct task_struct *task;
	unsigned int num_cant_freeze_now = 0;

	cgroup_iter_start(cgroup, &it);
	while ((task = cgroup_iter_next(cgroup, &it))) {
		if (!freeze_task(task))
			continue;
		if (is_task_frozen_enough(task))
			continue;
		if (!freezing(task) && !freezer_should_skip(task))
			num_cant_freeze_now++;
	}
	cgroup_iter_end(cgroup, &it);

	return num_cant_freeze_now ? -EBUSY : 0;
}

static void unfreeze_cgroup(struct cgroup *cgroup, struct freezer *freezer)
{
	struct cgroup_iter it;
	struct task_struct *task;

	cgroup_iter_start(cgroup, &it);
	while ((task = cgroup_iter_next(cgroup, &it)))
		__thaw_task(task);
	cgroup_iter_end(cgroup, &it);
}

/**
 * freezer_apply_state - apply state change to a single cgroup_freezer
 * @freezer: freezer to apply state change to
 * @freeze: whether to freeze or unfreeze
 * @state: CGROUP_FREEZING_* flag to set or clear
 *
 * Set or clear @state on @cgroup according to @freeze, and perform
 * freezing or thawing as necessary.
 */
static void freezer_apply_state(struct freezer *freezer, bool freeze,
				unsigned int state)
{
	/* also synchronizes against task migration, see freezer_attach() */
	lockdep_assert_held(&freezer->lock);

	if (!(freezer->state & CGROUP_FREEZER_ONLINE))
		return;

	if (freeze) {
		if (!(freezer->state & CGROUP_FREEZING))
			atomic_inc(&system_freezing_cnt);
		freezer->state |= state;
		freeze_cgroup(freezer);
	} else {
		bool was_freezing = freezer->state & CGROUP_FREEZING;

		freezer->state &= ~state;

		if (!(freezer->state & CGROUP_FREEZING)) {
			if (was_freezing)
				atomic_dec(&system_freezing_cnt);
			freezer->state &= ~CGROUP_FROZEN;
			unfreeze_cgroup(freezer);
		}
	}
}

/**
 * freezer_change_state - change the freezing state of a cgroup_freezer
 * @freezer: freezer of interest
 * @freeze: whether to freeze or thaw
 *
 * Freeze or thaw @freezer according to @freeze.  The operations are
 * recursive - all descendants of @freezer will be affected.
 */
static void freezer_change_state(struct freezer *freezer, bool freeze)
{
	struct cgroup *pos;

	/* update @freezer */
	spin_lock_irq(&freezer->lock);
	freezer_apply_state(freezer, freeze, CGROUP_FREEZING_SELF);
	spin_unlock_irq(&freezer->lock);

	/*
	 * Update all its descendants in pre-order traversal.  Each
	 * descendant will try to inherit its parent's FREEZING state as
	 * CGROUP_FREEZING_PARENT.
	 */
	rcu_read_lock();
	cgroup_for_each_descendant_pre(pos, freezer->css.cgroup) {
		struct freezer *pos_f = cgroup_freezer(pos);
		struct freezer *parent = parent_freezer(pos_f);

		/*
		 * Our update to @parent->state is already visible which is
		 * all we need.  No need to lock @parent.  For more info on
		 * synchronization, see freezer_post_create().
		 */
		spin_lock_irq(&pos_f->lock);
		freezer_apply_state(pos_f, parent->state & CGROUP_FREEZING,
				    CGROUP_FREEZING_PARENT);
		spin_unlock_irq(&pos_f->lock);
	}
	rcu_read_unlock();
}

static int freezer_write(struct cgroup *cgroup, struct cftype *cft,
			 const char *buffer)
{
	bool freeze;

	if (strcmp(buffer, freezer_state_strs(0)) == 0)
		freeze = false;
	else if (strcmp(buffer, freezer_state_strs(CGROUP_FROZEN)) == 0)
		freeze = true;
	else
		return -EINVAL;

	freezer_change_state(cgroup_freezer(cgroup), freeze);
	return 0;
}

static u64 freezer_self_freezing_read(struct cgroup *cgroup, struct cftype *cft)
{
	struct freezer *freezer = cgroup_freezer(cgroup);

	return (bool)(freezer->state & CGROUP_FREEZING_SELF);
}

static u64 freezer_parent_freezing_read(struct cgroup *cgroup, struct cftype *cft)
{
	struct freezer *freezer = cgroup_freezer(cgroup);

	return (bool)(freezer->state & CGROUP_FREEZING_PARENT);
}

static struct cftype files[] = {
	{
		.name = "state",
		.flags = CFTYPE_NOT_ON_ROOT,
		.read_seq_string = freezer_read,
		.write_string = freezer_write,
	},
	{
		.name = "self_freezing",
		.flags = CFTYPE_NOT_ON_ROOT,
		.read_u64 = freezer_self_freezing_read,
	},
	{
		.name = "parent_freezing",
		.flags = CFTYPE_NOT_ON_ROOT,
		.read_u64 = freezer_parent_freezing_read,
	},
	{ }	/* terminate */
};

struct cgroup_subsys freezer_subsys = {
	.name		= "freezer",
	.css_alloc	= freezer_css_alloc,
	.css_online	= freezer_css_online,
	.css_offline	= freezer_css_offline,
	.css_free	= freezer_css_free,
	.subsys_id	= freezer_subsys_id,
	.attach		= freezer_attach,
	.fork		= freezer_fork,
	.base_cftypes	= files,
};
