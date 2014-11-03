
/*
   rbd.c -- Export ceph rados objects as a Linux block device


   based on drivers/block/osdblk.c:

   Copyright 2009 Red Hat, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; see the file COPYING.  If not, write to
   the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.



   For usage instructions, please refer to:

                 Documentation/ABI/testing/sysfs-bus-rbd

 */

#include <linux/ceph/libceph.h>
#include <linux/ceph/osd_client.h>
#include <linux/ceph/mon_client.h>
#include <linux/ceph/decode.h>
#include <linux/parser.h>
#include <linux/bsearch.h>

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/slab.h>

#include "rbd_types.h"

#define RBD_DEBUG	/* Activate rbd_assert() calls */

/*
 * The basic unit of block I/O is a sector.  It is interpreted in a
 * number of contexts in Linux (blk, bio, genhd), but the default is
 * universally 512 bytes.  These symbols are just slightly more
 * meaningful than the bare numbers they represent.
 */
#define	SECTOR_SHIFT	9
#define	SECTOR_SIZE	(1ULL << SECTOR_SHIFT)

/*
 * Increment the given counter and return its updated value.
 * If the counter is already 0 it will not be incremented.
 * If the counter is already at its maximum value returns
 * -EINVAL without updating it.
 */
static int atomic_inc_return_safe(atomic_t *v)
{
	unsigned int counter;

	counter = (unsigned int)__atomic_add_unless(v, 1, 0);
	if (counter <= (unsigned int)INT_MAX)
		return (int)counter;

	atomic_dec(v);

	return -EINVAL;
}

/* Decrement the counter.  Return the resulting value, or -EINVAL */
static int atomic_dec_return_safe(atomic_t *v)
{
	int counter;

	counter = atomic_dec_return(v);
	if (counter >= 0)
		return counter;

	atomic_inc(v);

	return -EINVAL;
}

#define RBD_DRV_NAME "rbd"
#define RBD_DRV_NAME_LONG "rbd (rados block device)"

#define RBD_MINORS_PER_MAJOR	256		/* max minors per blkdev */

#define RBD_SNAP_DEV_NAME_PREFIX	"snap_"
#define RBD_MAX_SNAP_NAME_LEN	\
			(NAME_MAX - (sizeof (RBD_SNAP_DEV_NAME_PREFIX) - 1))

#define RBD_MAX_SNAP_COUNT	510	/* allows max snapc to fit in 4KB */

#define RBD_SNAP_HEAD_NAME	"-"

#define	BAD_SNAP_INDEX	U32_MAX		/* invalid index into snap array */

/* This allows a single page to hold an image name sent by OSD */
#define RBD_IMAGE_NAME_LEN_MAX	(PAGE_SIZE - sizeof (__le32) - 1)
#define RBD_IMAGE_ID_LEN_MAX	64

#define RBD_OBJ_PREFIX_LEN_MAX	64

/* Feature bits */

#define RBD_FEATURE_LAYERING	(1<<0)
#define RBD_FEATURE_STRIPINGV2	(1<<1)
#define RBD_FEATURES_ALL \
	    (RBD_FEATURE_LAYERING | RBD_FEATURE_STRIPINGV2)

/* Features supported by this (client software) implementation. */

#define RBD_FEATURES_SUPPORTED	(RBD_FEATURES_ALL)

/*
 * An RBD device name will be "rbd#", where the "rbd" comes from
 * RBD_DRV_NAME above, and # is a unique integer identifier.
 * MAX_INT_FORMAT_WIDTH is used in ensuring DEV_NAME_LEN is big
 * enough to hold all possible device names.
 */
#define DEV_NAME_LEN		32
#define MAX_INT_FORMAT_WIDTH	((5 * sizeof (int)) / 2 + 1)

/*
 * block device image metadata (in-memory version)
 */
struct rbd_image_header {
	u64 image_size;
	char *object_prefix;
	__u8 obj_order;
	__u8 crypt_type;
	__u8 comp_type;
	struct ceph_snap_context *snapc;
	size_t snap_names_len;
	u32 total_snaps;

	char *snap_names;
	u64 *snap_sizes;

	u64 obj_version;
};

/*
 * An rbd image specification.
 *
 * The tuple (pool_id, image_id, snap_id) is sufficient to uniquely
 * identify an image.  Each rbd_dev structure includes a pointer to
 * an rbd_spec structure that encapsulates this identity.
 *
 * Each of the id's in an rbd_spec has an associated name.  For a
 * user-mapped image, the names are supplied and the id's associated
 * with them are looked up.  For a layered image, a parent image is
 * defined by the tuple, and the names are looked up.
 *
 * An rbd_dev structure contains a parent_spec pointer which is
 * non-null if the image it represents is a child in a layered
 * image.  This pointer will refer to the rbd_spec structure used
 * by the parent rbd_dev for its own identity (i.e., the structure
 * is shared between the parent and child).
 *
 * Since these structures are populated once, during the discovery
 * phase of image construction, they are effectively immutable so
 * we make no effort to synchronize access to them.
 *
 * Note that code herein does not assume the image name is known (it
 * could be a null pointer).
 */
struct rbd_spec {
	u64		pool_id;
	const char	*pool_name;

	const char	*image_id;
	const char	*image_name;

	u64		snap_id;
	const char	*snap_name;

	struct kref	kref;
};

/*
 * an instance of the client.  multiple devices may share an rbd client.
 */
struct rbd_client {
	struct ceph_client	*client;
	struct kref		kref;
	struct list_head	node;
};

struct rbd_img_request;
typedef void (*rbd_img_callback_t)(struct rbd_img_request *);

#define	BAD_WHICH	U32_MAX		/* Good which or bad which, which? */

struct rbd_obj_request;
typedef void (*rbd_obj_callback_t)(struct rbd_obj_request *);

enum obj_request_type {
	OBJ_REQUEST_NODATA, OBJ_REQUEST_BIO, OBJ_REQUEST_PAGES
};

enum obj_req_flags {
	OBJ_REQ_DONE,		/* completion flag: not done = 0, done = 1 */
	OBJ_REQ_IMG_DATA,	/* object usage: standalone = 0, image = 1 */
	OBJ_REQ_KNOWN,		/* EXISTS flag valid: no = 0, yes = 1 */
	OBJ_REQ_EXISTS,		/* target exists: no = 0, yes = 1 */
};

struct rbd_obj_request {
	const char		*object_name;
	u64			offset;		/* object start byte */
	u64			length;		/* bytes from offset */
	unsigned long		flags;

	/*
	 * An object request associated with an image will have its
	 * img_data flag set; a standalone object request will not.
	 *
	 * A standalone object request will have which == BAD_WHICH
	 * and a null obj_request pointer.
	 *
	 * An object request initiated in support of a layered image
	 * object (to check for its existence before a write) will
	 * have which == BAD_WHICH and a non-null obj_request pointer.
	 *
	 * Finally, an object request for rbd image data will have
	 * which != BAD_WHICH, and will have a non-null img_request
	 * pointer.  The value of which will be in the range
	 * 0..(img_request->obj_request_count-1).
	 */
	union {
		struct rbd_obj_request	*obj_request;	/* STAT op */
		struct {
			struct rbd_img_request	*img_request;
			u64			img_offset;
			/* links for img_request->obj_requests list */
			struct list_head	links;
		};
	};
	u32			which;		/* posn image request list */

	enum obj_request_type	type;
	union {
		struct bio	*bio_list;
		struct {
			struct page	**pages;
			u32		page_count;
		};
	};
	struct page		**copyup_pages;
	u32			copyup_page_count;

	struct ceph_osd_request	*osd_req;

	u64			xferred;	/* bytes transferred */
	int			result;

	rbd_obj_callback_t	callback;
	struct completion	completion;

	struct kref		kref;
};

enum img_req_flags {
	IMG_REQ_WRITE,		/* I/O direction: read = 0, write = 1 */
	IMG_REQ_CHILD,		/* initiator: block = 0, child image = 1 */
	IMG_REQ_LAYERED,	/* ENOENT handling: normal = 0, layered = 1 */
};

struct rbd_img_request {
	struct rbd_device	*rbd_dev;
	u64			offset;	/* starting image byte offset */
	u64			length;	/* byte count from offset */
	unsigned long		flags;
	union {
		u64			snap_id;	/* for reads */
		struct ceph_snap_context *snapc;	/* for writes */
	};
	union {
		struct request		*rq;		/* block request */
		struct rbd_obj_request	*obj_request;	/* obj req initiator */
	};
	struct page		**copyup_pages;
	u32			copyup_page_count;
	spinlock_t		completion_lock;/* protects next_completion */
	u32			next_completion;
	rbd_img_callback_t	callback;
	u64			xferred;/* aggregate bytes transferred */
	int			result;	/* first nonzero obj_request result */

	u32			obj_request_count;
	struct list_head	obj_requests;	/* rbd_obj_request structs */

	struct kref		kref;
};

#define for_each_obj_request(ireq, oreq) \
	list_for_each_entry(oreq, &(ireq)->obj_requests, links)
#define for_each_obj_request_from(ireq, oreq) \
	list_for_each_entry_from(oreq, &(ireq)->obj_requests, links)
#define for_each_obj_request_safe(ireq, oreq, n) \
	list_for_each_entry_safe_reverse(oreq, n, &(ireq)->obj_requests, links)

struct rbd_mapping {
	u64                     size;
	u64                     features;
	bool			read_only;
};

/*
 * a single device
 */
struct rbd_device {
	int			dev_id;		/* blkdev unique id */

	int			major;		/* blkdev assigned major */
	struct gendisk		*disk;		/* blkdev's gendisk and rq */

	u32			image_format;	/* Either 1 or 2 */
	struct rbd_client	*rbd_client;

	char			name[DEV_NAME_LEN]; /* blkdev name, e.g. rbd3 */

	spinlock_t		lock;		/* queue lock */

	struct rbd_image_header	header;
	char			obj[RBD_MAX_OBJ_NAME_LEN]; /* rbd image name */
	int			obj_len;
	char			obj_md_name[RBD_MAX_MD_NAME_LEN]; /* hdr nm. */
	char			pool_name[RBD_MAX_POOL_NAME_LEN];
	int			poolid;

	struct ceph_osd_event   *watch_event;
	struct ceph_osd_request *watch_request;

	/* protects updating the header */
	struct rw_semaphore     header_rwsem;
	/* name of the snapshot this device reads from */
	char                    snap_name[RBD_MAX_SNAP_NAME_LEN];
	/* id of the snapshot this device reads from */
	u64                     snap_id;	/* current snapshot id */
	/* whether the snap_id this device reads from still exists */
	bool                    snap_exists;
	bool			read_only;

	struct list_head	node;

	/* sysfs related */
	struct device		dev;
	unsigned long		open_count;	/* protected by lock */
};

/*
 * Flag bits for rbd_dev->flags.  If atomicity is required,
 * rbd_dev->lock is used to protect access.
 *
 * Currently, only the "removing" flag (which is coupled with the
 * "open_count" field) requires atomic access.
 */
enum rbd_dev_flags {
	RBD_DEV_FLAG_EXISTS,	/* mapped snapshot has not been deleted */
	RBD_DEV_FLAG_REMOVING,	/* this mapping is being removed */
};

static DEFINE_MUTEX(ctl_mutex);	  /* Serialize open/close/setup/teardown */

static LIST_HEAD(rbd_dev_list);    /* devices */
static DEFINE_SPINLOCK(rbd_dev_list_lock);

static LIST_HEAD(rbd_client_list);		/* clients */
static DEFINE_SPINLOCK(rbd_client_list_lock);

static int __rbd_init_snaps_header(struct rbd_device *rbd_dev);
static void rbd_dev_release(struct device *dev);
static void __rbd_remove_snap_dev(struct rbd_device *rbd_dev,
				  struct rbd_snap *snap);

static ssize_t rbd_add(struct bus_type *bus, const char *buf,
		       size_t count);
static ssize_t rbd_remove(struct bus_type *bus, const char *buf,
			  size_t count);

static struct bus_attribute rbd_bus_attrs[] = {
	__ATTR(add, S_IWUSR, NULL, rbd_add),
	__ATTR(remove, S_IWUSR, NULL, rbd_remove),
	__ATTR_NULL
};

static struct bus_type rbd_bus_type = {
	.name		= "rbd",
	.bus_attrs	= rbd_bus_attrs,
};

static void rbd_root_dev_release(struct device *dev)
{
}

static struct device rbd_root_dev = {
	.init_name =    "rbd",
	.release =      rbd_root_dev_release,
};

static __printf(2, 3)
void rbd_warn(struct rbd_device *rbd_dev, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;

	if (!rbd_dev)
		printk(KERN_WARNING "%s: %pV\n", RBD_DRV_NAME, &vaf);
	else if (rbd_dev->disk)
		printk(KERN_WARNING "%s: %s: %pV\n",
			RBD_DRV_NAME, rbd_dev->disk->disk_name, &vaf);
	else if (rbd_dev->spec && rbd_dev->spec->image_name)
		printk(KERN_WARNING "%s: image %s: %pV\n",
			RBD_DRV_NAME, rbd_dev->spec->image_name, &vaf);
	else if (rbd_dev->spec && rbd_dev->spec->image_id)
		printk(KERN_WARNING "%s: id %s: %pV\n",
			RBD_DRV_NAME, rbd_dev->spec->image_id, &vaf);
	else	/* punt */
		printk(KERN_WARNING "%s: rbd_dev %p: %pV\n",
			RBD_DRV_NAME, rbd_dev, &vaf);
	va_end(args);
}

#ifdef RBD_DEBUG
#define rbd_assert(expr)						\
		if (unlikely(!(expr))) {				\
			printk(KERN_ERR "\nAssertion failure in %s() "	\
						"at line %d:\n\n"	\
					"\trbd_assert(%s);\n\n",	\
					__func__, __LINE__, #expr);	\
			BUG();						\
		}
#else /* !RBD_DEBUG */
#  define rbd_assert(expr)	((void) 0)
#endif /* !RBD_DEBUG */

static struct device *rbd_get_dev(struct rbd_device *rbd_dev)
{
	return get_device(&rbd_dev->dev);
}

static void rbd_put_dev(struct rbd_device *rbd_dev)
{
	put_device(&rbd_dev->dev);
}

static int __rbd_refresh_header(struct rbd_device *rbd_dev);

static int rbd_open(struct block_device *bdev, fmode_t mode)
{
	struct rbd_device *rbd_dev = bdev->bd_disk->private_data;

	if ((mode & FMODE_WRITE) && rbd_dev->read_only)
		return -EROFS;

	mutex_lock_nested(&ctl_mutex, SINGLE_DEPTH_NESTING);
	rbd_get_dev(rbd_dev);
	set_device_ro(bdev, rbd_dev->read_only);
	rbd_dev->open_count++;
	mutex_unlock(&ctl_mutex);

	return 0;
}

static void rbd_release(struct gendisk *disk, fmode_t mode)
{
	struct rbd_device *rbd_dev = disk->private_data;

	mutex_lock_nested(&ctl_mutex, SINGLE_DEPTH_NESTING);
	BUG_ON(!rbd_dev->open_count);
	rbd_dev->open_count--;
	rbd_put_dev(rbd_dev);
	mutex_unlock(&ctl_mutex);
}

static const struct block_device_operations rbd_bd_ops = {
	.owner			= THIS_MODULE,
	.open			= rbd_open,
	.release		= rbd_release,
};

/*
 * Initialize an rbd client instance.  Success or not, this function
 * consumes ceph_opts.
 */
static struct rbd_client *rbd_client_create(struct ceph_options *ceph_opts)
{
	struct rbd_client *rbdc;
	int ret = -ENOMEM;

	dout("%s:\n", __func__);
	rbdc = kmalloc(sizeof(struct rbd_client), GFP_KERNEL);
	if (!rbdc)
		goto out_opt;

	kref_init(&rbdc->kref);
	INIT_LIST_HEAD(&rbdc->node);

	mutex_lock_nested(&ctl_mutex, SINGLE_DEPTH_NESTING);

	rbdc->client = ceph_create_client(ceph_opts, rbdc, 0, 0);
	if (IS_ERR(rbdc->client))
		goto out_mutex;
	ceph_opts = NULL; /* Now rbdc->client is responsible for ceph_opts */

	ret = ceph_open_session(rbdc->client);
	if (ret < 0)
		goto out_err;

	spin_lock(&rbd_client_list_lock);
	list_add_tail(&rbdc->node, &rbd_client_list);
	spin_unlock(&rbd_client_list_lock);

	mutex_unlock(&ctl_mutex);
	dout("%s: rbdc %p\n", __func__, rbdc);

	return rbdc;

out_err:
	ceph_destroy_client(rbdc->client);
out_mutex:
	mutex_unlock(&ctl_mutex);
	kfree(rbdc);
out_opt:
	if (ceph_opts)
		ceph_destroy_options(ceph_opts);
	dout("%s: error %d\n", __func__, ret);

	return ERR_PTR(ret);
}

static struct rbd_client *__rbd_get_client(struct rbd_client *rbdc)
{
	kref_get(&rbdc->kref);

	return rbdc;
}

/*
 * Find a ceph client with specific addr and configuration.
 */
static struct rbd_client *__rbd_client_find(struct ceph_options *ceph_opts)
{
	struct rbd_client *client_node;

	if (ceph_opts->flags & CEPH_OPT_NOSHARE)
		return NULL;

	list_for_each_entry(client_node, &rbd_client_list, node)
		if (!ceph_compare_options(ceph_opts, client_node->client))
			return client_node;
	return NULL;
}

/*
 * mount options
 */
enum {
	Opt_last_int,
	/* int args above */
	Opt_last_string,
	/* string args above */
	Opt_read_only,
	Opt_read_write,
	/* Boolean args above */
	Opt_last_bool,
};

static match_table_t rbdopt_tokens = {
	/* int args above */
	/* string args above */
	{Opt_read_only, "read_only"},
	{Opt_read_only, "ro"},		/* Alternate spelling */
	{Opt_read_write, "read_write"},
	{Opt_read_write, "rw"},		/* Alternate spelling */
	/* Boolean args above */
	{-1, NULL}
};

struct rbd_options {
	bool	read_only;
};

#define RBD_READ_ONLY_DEFAULT	false

static int parse_rbd_opts_token(char *c, void *private)
{
	struct rbd_options *rbd_opts = private;
	substring_t argstr[MAX_OPT_ARGS];
	int token, intval, ret;

	token = match_token(c, rbd_opts_tokens, argstr);
	if (token < 0)
		return -EINVAL;

	if (token < Opt_last_int) {
		ret = match_int(&argstr[0], &intval);
		if (ret < 0) {
			pr_err("bad mount option arg (not int) "
			       "at '%s'\n", c);
			return ret;
		}
		dout("got int token %d val %d\n", token, intval);
	} else if (token > Opt_last_int && token < Opt_last_string) {
		dout("got string token %d val %s\n", token,
		     argstr[0].from);
	} else if (token > Opt_last_string && token < Opt_last_bool) {
		dout("got Boolean token %d\n", token);
	} else {
		dout("got token %d\n", token);
	}

	switch (token) {
	case Opt_read_only:
		rbdopt->read_only = true;
		break;
	case Opt_read_write:
		rbdopt->read_only = false;
		break;
	default:
		BUG_ON(token);
	}
	return 0;
}

/*
 * Get a ceph client with specific addr and configuration, if one does
 * not exist create it.  Either way, ceph_opts is consumed by this
 * function.
 */
static struct rbd_client *rbd_get_client(const char *mon_addr,
					 size_t mon_addr_len,
					 char *options)
{
	struct rbd_client *rbdc;
	struct ceph_options *ceph_opts;
	struct rbd_options *rbd_opts;

	rbd_opts = kzalloc(sizeof(*rbd_opts), GFP_KERNEL);
	if (!rbd_opts)
		return ERR_PTR(-ENOMEM);

	rbd_opts->read_only = RBD_READ_ONLY_DEFAULT;

	ceph_opts = ceph_parse_options(options, mon_addr,
					mon_addr + mon_addr_len,
					parse_rbd_opts_token, rbd_opts);
	if (IS_ERR(ceph_opts)) {
		kfree(rbd_opts);
		return ERR_CAST(ceph_opts);
	}

	spin_lock(&rbd_client_list_lock);
	rbdc = __rbd_client_find(ceph_opts);
	if (rbdc) {
		/* using an existing client */
		kref_get(&rbdc->kref);
		spin_unlock(&rbd_client_list_lock);

		ceph_destroy_options(ceph_opts);
		kfree(rbd_opts);

		return rbdc;
	}
	spin_unlock(&rbd_client_list_lock);

	rbdc = rbd_client_create(ceph_opts, rbd_opts);

	if (IS_ERR(rbdc))
		kfree(rbd_opts);

	return rbdc;
}

/*
 * Destroy ceph client
 *
 * Caller must hold rbd_client_list_lock.
 */
static void rbd_client_release(struct kref *kref)
{
	struct rbd_client *rbdc = container_of(kref, struct rbd_client, kref);

	dout("%s: rbdc %p\n", __func__, rbdc);
	spin_lock(&rbd_client_list_lock);
	list_del(&rbdc->node);
	spin_unlock(&rbd_client_list_lock);

	ceph_destroy_client(rbdc->client);
	kfree(rbdc);
}

/*
 * Drop reference to ceph client node. If it's not referenced anymore, release
 * it.
 */
static void rbd_put_client(struct rbd_client *rbdc)
{
	if (rbdc)
		kref_put(&rbdc->kref, rbd_client_release);
}

/*
 * Destroy requests collection
 */
static void rbd_coll_release(struct kref *kref)
{
	struct rbd_req_coll *coll =
		container_of(kref, struct rbd_req_coll, kref);

	dout("rbd_coll_release %p\n", coll);
	kfree(coll);
}

/*
 * Create a new header structure, translate header format from the on-disk
 * header.
 */
static int rbd_header_from_disk(struct rbd_image_header *header,
				 struct rbd_image_header_ondisk *ondisk,
				 int allocated_snaps,
				 gfp_t gfp_flags)
{
	int i;
	u32 snap_count;

	if (memcmp(ondisk, RBD_HEADER_TEXT, sizeof(RBD_HEADER_TEXT)))
		return -ENXIO;

	snap_count = le32_to_cpu(ondisk->snap_count);
	header->snapc = kmalloc(sizeof(struct ceph_snap_context) +
				snap_count * sizeof(u64),
				gfp_flags);
	if (!header->snapc)
		return -ENOMEM;

	header->snap_names_len = le64_to_cpu(ondisk->snap_names_len);
	if (snap_count) {
		header->snap_names = kmalloc(header->snap_names_len,
					     gfp_flags);
		if (!header->snap_names)
			goto err_snapc;
		header->snap_sizes = kmalloc(snap_count * sizeof(u64),
					     gfp_flags);
		if (!header->snap_sizes)
			goto err_names;
	} else {
		header->snap_names = NULL;
		header->snap_sizes = NULL;
	}
	memcpy(header->block_name, ondisk->block_name,
	       sizeof(ondisk->block_name));

	header->image_size = le64_to_cpu(ondisk->image_size);
	header->obj_order = ondisk->options.order;
	header->crypt_type = ondisk->options.crypt_type;
	header->comp_type = ondisk->options.comp_type;

	atomic_set(&header->snapc->nref, 1);
	header->snapc->seq = le64_to_cpu(ondisk->snap_seq);
	header->snapc->num_snaps = snap_count;
	header->total_snaps = snap_count;

	if (snap_count && allocated_snaps == snap_count) {
		int i;

		for (i = 0; i < snap_count; i++) {
			header->snapc->snaps[i] =
				le64_to_cpu(ondisk->snaps[i].id);
			header->snap_sizes[i] =
				le64_to_cpu(ondisk->snaps[i].image_size);
		}

		/* copy snapshot names */
		memcpy(header->snap_names, &ondisk->snaps[snap_count],
			header->snap_names_len);
	}

	return 0;

err_sizes:
	kfree(header->snap_sizes);
	header->snap_sizes = NULL;
err_names:
	kfree(header->snap_names);
	header->snap_names = NULL;
err_snapc:
	kfree(header->snapc);
	header->snapc = NULL;

	return -ENOMEM;
}

static int snap_by_name(struct rbd_image_header *header, const char *snap_name,
			u64 *seq, u64 *size)
{
	int i;
	char *p = header->snap_names;

	for (i = 0; i < header->total_snaps; i++) {
		if (!strcmp(snap_name, p)) {

			/* Found it.  Pass back its id and/or size */

			if (seq)
				*seq = header->snapc->snaps[i];
			if (size)
				*size = header->snap_sizes[i];
			return i;
		}
		p += strlen(p) + 1;	/* Skip ahead to the next name */
	}
	return -ENOENT;
}

static int rbd_header_set_snap(struct rbd_device *dev, u64 *size)
{
	struct rbd_image_header *header = &dev->header;
	struct ceph_snap_context *snapc = header->snapc;
	int ret = -ENOENT;

	BUILD_BUG_ON(sizeof (dev->snap_name) < sizeof (RBD_SNAP_HEAD_NAME));

	down_write(&dev->header_rwsem);

	if (!memcmp(dev->snap_name, RBD_SNAP_HEAD_NAME,
		    sizeof (RBD_SNAP_HEAD_NAME))) {
		if (header->total_snaps)
			snapc->seq = header->snap_seq;
		else
			snapc->seq = 0;
		dev->snap_id = CEPH_NOSNAP;
		dev->snap_exists = false;
		dev->read_only = dev->rbd_client->rbd_opts->read_only;
		if (size)
			*size = header->image_size;
	} else {
		ret = snap_by_name(header, dev->snap_name, &snapc->seq, size);
		if (ret < 0)
			goto done;
		dev->snap_id = snapc->seq;
		dev->snap_exists = true;
		dev->read_only = true;	/* No choice for snapshots */
	}

	ret = 0;
done:
	up_write(&dev->header_rwsem);
	return ret;
}

static void rbd_header_free(struct rbd_image_header *header)
{
	ceph_put_snap_context(header->snapc);
	kfree(header->snap_names);
	kfree(header->snap_sizes);
}

/*
 * get the actual striped segment name, offset and length
 */
static u64 rbd_get_segment(struct rbd_image_header *header,
			   const char *block_name,
			   u64 ofs, u64 len,
			   char *seg_name, u64 *segofs)
{
	u64 seg = ofs >> header->obj_order;

	if (seg_name)
		snprintf(seg_name, RBD_MAX_SEG_NAME_LEN,
			 "%s.%012llx", object_prefix, seg);

	ofs = ofs & ((1 << header->obj_order) - 1);
	len = min_t(u64, len, (1 << header->obj_order) - ofs);

	if (segofs)
		*segofs = ofs;

	return len;
}

static int rbd_get_num_segments(struct rbd_image_header *header,
				u64 ofs, u64 len)
{
	u64 start_seg = ofs >> header->obj_order;
	u64 end_seg = (ofs + len - 1) >> header->obj_order;
	return end_seg - start_seg + 1;
}

/*
 * returns the size of an object in the image
 */
static u64 rbd_obj_bytes(struct rbd_image_header *header)
{
	return 1 << header->obj_order;
}

/*
 * bio helpers
 */

static void bio_chain_put(struct bio *chain)
{
	struct bio *tmp;

	while (chain) {
		tmp = chain;
		chain = chain->bi_next;
		bio_put(tmp);
	}
}

/*
 * zeros a bio chain, starting at specific offset
 */
static void zero_bio_chain(struct bio *chain, int start_ofs)
{
	struct bio_vec *bv;
	unsigned long flags;
	void *buf;
	int i;
	int pos = 0;

	while (chain) {
		bio_for_each_segment(bv, chain, i) {
			if (pos + bv->bv_len > start_ofs) {
				int remainder = max(start_ofs - pos, 0);
				buf = bvec_kmap_irq(bv, &flags);
				memset(buf + remainder, 0,
				       bv->bv_len - remainder);
				bvec_kunmap_irq(buf, &flags);
			}
			pos += bv->bv_len;
		}

		chain = chain->bi_next;
	}
}

/*
 * bio_chain_clone - clone a chain of bios up to a certain length.
 * might return a bio_pair that will need to be released.
 */
static struct bio *bio_chain_clone(struct bio **old, struct bio **next,
				   struct bio_pair **bp,
				   int len, gfp_t gfpmask)
{
	struct bio *old_chain = *old;
	struct bio *new_chain = NULL;
	struct bio *tail;
	int total = 0;

	if (*bp) {
		bio_pair_release(*bp);
		*bp = NULL;
	}

	while (old_chain && (total < len)) {
		struct bio *tmp;

		tmp = bio_kmalloc(gfpmask, old_chain->bi_max_vecs);
		if (!tmp)
			goto err_out;
		gfpmask &= ~__GFP_WAIT;	/* can't wait after the first */

		if (total + old_chain->bi_size > len) {
			struct bio_pair *bp;

			/*
			 * this split can only happen with a single paged bio,
			 * split_bio will BUG_ON if this is not the case
			 */
			dout("bio_chain_clone split! total=%d remaining=%d"
			     "bi_size=%u\n",
			     total, len - total, old_chain->bi_size);

			/* split the bio. We'll release it either in the next
			   call, or it will have to be released outside */
			bp = bio_split(old_chain, (len - total) / SECTOR_SIZE);
			if (!bp)
				goto err_out;

			__bio_clone(tmp, &bp->bio1);

			*next = &bp->bio2;
		} else {
			__bio_clone(tmp, old_chain);
			*next = old_chain->bi_next;
		}

		tmp->bi_bdev = NULL;
		tmp->bi_next = NULL;
		if (new_chain)
			tail->bi_next = tmp;
		else
			new_chain = tmp;
		tail = tmp;
		old_chain = old_chain->bi_next;

		total += tmp->bi_size;
	}

	rbd_assert(total == len);

	*old = old_chain;

	return new_chain;

err_out:
	dout("bio_chain_clone with err\n");
	bio_chain_put(new_chain);
	return NULL;
}

/*
 * helpers for osd request op vectors.
 */
static struct ceph_osd_req_op *rbd_create_rw_ops(int num_ops,
					int opcode, u32 payload_len)
{
	struct ceph_osd_req_op *ops;

	ops = kzalloc(sizeof (*ops) * (num_ops + 1), GFP_NOIO);
	if (!ops)
		return NULL;

	ops[0].op = opcode;

	/* The bio layer requires at least sector-sized I/O */

	if (ondisk->options.order < SECTOR_SHIFT)
		return false;

	/* If we use u64 in a few spots we may be able to loosen this */

	if (ondisk->options.order > 8 * sizeof (int) - 1)
		return false;

	/*
	 * op extent offset and length will be set later on
	 * in calc_raw_layout()
	 */
	ops[0].payload_len = payload_len;

	return ops;
}

static void rbd_destroy_ops(struct ceph_osd_req_op *ops)
{
	kfree(ops);
}

static void rbd_coll_end_req_index(struct request *rq,
				   struct rbd_req_coll *coll,
				   int index,
				   int ret, u64 len)
{
	struct request_queue *q;
	int min, max, i;

	dout("rbd_coll_end_req_index %p index %d ret %d len %llu\n",
	     coll, index, ret, (unsigned long long) len);

	if (!rq)
		return;

	if (!coll) {
		blk_end_request(rq, ret, len);
		return;
	}

	q = rq->q;

	spin_lock_irq(q->queue_lock);
	coll->status[index].done = 1;
	coll->status[index].rc = ret;
	coll->status[index].bytes = len;
	max = min = coll->num_done;
	while (max < coll->total && coll->status[max].done)
		max++;

	for (i = min; i<max; i++) {
		__blk_end_request(rq, coll->status[i].rc,
				  coll->status[i].bytes);
		coll->num_done++;
		kref_put(&coll->kref, rbd_coll_release);
	}
	spin_unlock_irq(q->queue_lock);
}

static void rbd_coll_end_req(struct rbd_request *req,
			     int ret, u64 len)
{
	rbd_coll_end_req_index(req->rq, req->coll, req->coll_index, ret, len);
}

/*
 * Send ceph osd request
 */
static int rbd_do_request(struct request *rq,
			  struct rbd_device *rbd_dev,
			  struct ceph_snap_context *snapc,
			  u64 snapid,
			  const char *object_name, u64 ofs, u64 len,
			  struct bio *bio,
			  struct page **pages,
			  int num_pages,
			  int flags,
			  struct ceph_osd_req_op *ops,
			  struct rbd_req_coll *coll,
			  int coll_index,
			  void (*rbd_cb)(struct ceph_osd_request *req,
					 struct ceph_msg *msg),
			  struct ceph_osd_request **linger_req,
			  u64 *ver)
{
	struct ceph_osd_request *req;
	struct ceph_file_layout *layout;
	int ret;
	u64 bno;
	struct timespec mtime = CURRENT_TIME;
	struct rbd_request *req_data;
	struct ceph_osd_request_head *reqhead;
	struct ceph_osd_client *osdc;

	req_data = kzalloc(sizeof(*req_data), GFP_NOIO);
	if (!req_data) {
		if (coll)
			rbd_coll_end_req_index(rq, coll, coll_index,
					       -ENOMEM, len);
		return -ENOMEM;
	}

	if (coll) {
		req_data->coll = coll;
		req_data->coll_index = coll_index;
	}

	dout("rbd_do_request obj=%s ofs=%lld len=%lld\n", obj, len, ofs);

	osdc = &dev->rbd_client->client->osdc;
	req = ceph_osdc_alloc_request(osdc, flags, snapc, ops,
					false, GFP_NOIO, pages, bio);
	if (!req) {
		ret = -ENOMEM;
		goto done_pages;
	}

	req->r_callback = rbd_cb;

	req_data->rq = rq;
	req_data->bio = bio;
	req_data->pages = pages;
	req_data->len = len;

	req->r_priv = req_data;

	reqhead = req->r_request->front.iov_base;
	reqhead->snapid = cpu_to_le64(CEPH_NOSNAP);

	strncpy(req->r_oid, object_name, sizeof(req->r_oid));
	req->r_oid_len = strlen(req->r_oid);

	layout = &req->r_file_layout;
	memset(layout, 0, sizeof(*layout));
	layout->fl_stripe_unit = cpu_to_le32(1 << RBD_MAX_OBJ_ORDER);
	layout->fl_stripe_count = cpu_to_le32(1);
	layout->fl_object_size = cpu_to_le32(1 << RBD_MAX_OBJ_ORDER);
	layout->fl_pg_pool = cpu_to_le32(dev->poolid);
	ret = ceph_calc_raw_layout(osdc, layout, snapid, ofs, &len, &bno,
				   req, ops);
	BUG_ON(ret != 0);

	ceph_osdc_build_request(req, ofs, &len,
				ops,
				snapc,
				&mtime,
				req->r_oid, req->r_oid_len);

	if (linger_req) {
		ceph_osdc_set_request_linger(osdc, req);
		*linger_req = req;
	}

	ret = ceph_osdc_start_request(osdc, req, false);
	if (ret < 0)
		goto done_err;

	if (!rbd_cb) {
		ret = ceph_osdc_wait_request(osdc, req);
		if (ver)
			*ver = le64_to_cpu(req->r_reassert_version.version);
		dout("reassert_ver=%llu\n",
			(unsigned long long)
				le64_to_cpu(req->r_reassert_version.version));
		ceph_osdc_put_request(req);
	}
	return ret;

done_err:
	bio_chain_put(req_data->bio);
	ceph_osdc_put_request(req);
done_pages:
	rbd_coll_end_req(req_data, ret, len);
	kfree(req_data);
	return ret;
}

/*
 * Ceph osd op callback
 */
static void rbd_req_cb(struct ceph_osd_request *req, struct ceph_msg *msg)
{
	struct rbd_request *req_data = req->r_priv;
	struct ceph_osd_reply_head *replyhead;
	struct ceph_osd_op *op;
	__s32 rc;
	u64 bytes;
	int read_op;

	/* parse reply */
	replyhead = msg->front.iov_base;
	WARN_ON(le32_to_cpu(replyhead->num_ops) == 0);
	op = (void *)(replyhead + 1);
	rc = le32_to_cpu(replyhead->result);
	bytes = le64_to_cpu(op->extent.length);
	read_op = (le16_to_cpu(op->op) == CEPH_OSD_OP_READ);

	dout("rbd_req_cb bytes=%llu readop=%d rc=%d\n",
		(unsigned long long) bytes, read_op, (int) rc);

	if (rc == -ENOENT && read_op) {
		zero_bio_chain(req_data->bio, 0);
		rc = 0;
	} else if (rc == 0 && read_op && bytes < req_data->len) {
		zero_bio_chain(req_data->bio, bytes);
		bytes = req_data->len;
	}

	rbd_coll_end_req(req_data, rc, bytes);

	if (req_data->bio)
		bio_chain_put(req_data->bio);

	ceph_osdc_put_request(req);
	kfree(req_data);
}

static void rbd_simple_req_cb(struct ceph_osd_request *req, struct ceph_msg *msg)
{
	ceph_osdc_put_request(req);
}

/*
 * Do a synchronous ceph osd operation
 */
static int rbd_req_sync_op(struct rbd_device *rbd_dev,
			   struct ceph_snap_context *snapc,
			   u64 snapid,
			   int flags,
			   struct ceph_osd_req_op *ops,
			   const char *object_name,
			   u64 ofs, u64 inbound_size,
			   char *inbound,
			   struct ceph_osd_request **linger_req,
			   u64 *ver)
{
	int ret;
	struct page **pages;
	int num_pages;

	rbd_assert(ops != NULL);

	num_pages = calc_pages_for(ofs, inbound_size);
	pages = ceph_alloc_page_vector(num_pages, GFP_KERNEL);
	if (IS_ERR(pages))
		return PTR_ERR(pages);

	ret = rbd_do_request(NULL, rbd_dev, snapc, snapid,
			  object_name, ofs, inbound_size, NULL,
			  pages, num_pages,
			  flags,
			  ops,
			  NULL, 0,
			  NULL,
			  linger_req, ver);
	if (ret < 0)
		goto done;

	if ((flags & CEPH_OSD_FLAG_READ) && inbound)
		ret = ceph_copy_from_page_vector(pages, inbound, ofs, ret);

done:
	ceph_release_page_vector(pages, num_pages);
	return ret;
}

/*
 * Do an asynchronous ceph osd operation
 */
static int rbd_do_op(struct request *rq,
		     struct rbd_device *rbd_dev,
		     struct ceph_snap_context *snapc,
		     u64 ofs, u64 len,
		     struct bio *bio,
		     struct rbd_req_coll *coll,
		     int coll_index)
{
	char *seg_name;
	u64 seg_ofs;
	u64 seg_len;
	int ret;
	struct ceph_osd_req_op *ops;
	u32 payload_len;
	int opcode;
	int flags;
	u64 snapid;

	seg_name = rbd_segment_name(rbd_dev, ofs);
	if (!seg_name)
		return -ENOMEM;
	seg_len = rbd_segment_length(rbd_dev, ofs, len);
	seg_ofs = rbd_segment_offset(rbd_dev, ofs);

	if (rq_data_dir(rq) == WRITE) {
		opcode = CEPH_OSD_OP_WRITE;
		flags = CEPH_OSD_FLAG_WRITE|CEPH_OSD_FLAG_ONDISK;
		snapid = CEPH_NOSNAP;
		payload_len = seg_len;
	} else {
		opcode = CEPH_OSD_OP_READ;
		flags = CEPH_OSD_FLAG_READ;
		snapc = NULL;
		snapid = rbd_dev->spec->snap_id;
		payload_len = 0;
	}

	ret = -ENOMEM;
	ops = rbd_create_rw_ops(1, opcode, payload_len);
	if (!ops)
		goto done;

	/* we've taken care of segment sizes earlier when we
	   cloned the bios. We should never have a segment
	   truncated at this point */
	rbd_assert(seg_len == len);

	ret = rbd_do_request(rq, rbd_dev, snapc, snapid,
			     seg_name, seg_ofs, seg_len,
			     bio,
			     NULL, 0,
			     flags,
			     ops,
			     coll, coll_index,
			     rbd_req_cb, 0, NULL);

	rbd_destroy_ops(ops);
done:
	kfree(seg_name);
	return ret;
}

/*
 * Request async osd write
 */
static int rbd_req_write(struct request *rq,
			 struct rbd_device *rbd_dev,
			 struct ceph_snap_context *snapc,
			 u64 ofs, u64 len,
			 struct bio *bio,
			 struct rbd_req_coll *coll,
			 int coll_index)
{
	return rbd_do_op(rq, rbd_dev, snapc, CEPH_NOSNAP,
			 CEPH_OSD_OP_WRITE,
			 CEPH_OSD_FLAG_WRITE | CEPH_OSD_FLAG_ONDISK,
			 ofs, len, bio, coll, coll_index);
}

/*
 * Request async osd read
 */
static int rbd_req_read(struct request *rq,
			 struct rbd_device *rbd_dev,
			 u64 snapid,
			 u64 ofs, u64 len,
			 struct bio *bio,
			 struct rbd_req_coll *coll,
			 int coll_index)
{
	return rbd_do_op(rq, rbd_dev, NULL,
			 snapid,
			 CEPH_OSD_OP_READ,
			 CEPH_OSD_FLAG_READ,
			 2,
			 ofs, len, bio, coll, coll_index);
}

/*
 * Request sync osd read
 */
static int rbd_req_sync_read(struct rbd_device *dev,
			  struct ceph_snap_context *snapc,
			  u64 snapid,
			  const char *obj,
			  u64 ofs, u64 len,
			  char *buf,
			  u64 *ver)
{
	return rbd_req_sync_op(dev, NULL,
			       snapid,
			       CEPH_OSD_OP_READ,
			       CEPH_OSD_FLAG_READ,
			       NULL,
			       1, obj, ofs, len, buf, NULL, ver);
}

/*
 * Request sync osd watch
 */
static int rbd_req_sync_notify_ack(struct rbd_device *dev,
				   u64 ver,
				   u64 notify_id,
				   const char *obj)
{
	struct ceph_osd_req_op *ops;
	struct page **pages = NULL;
	int ret;

	ret = rbd_create_rw_ops(&ops, 1, CEPH_OSD_OP_NOTIFY_ACK, 0);
	if (ret < 0)
		return ret;

	ops[0].watch.ver = cpu_to_le64(ver);
	ops[0].watch.cookie = notify_id;
	ops[0].watch.flag = 0;

	ret = rbd_do_request(NULL, dev, NULL, CEPH_NOSNAP,
			  obj, 0, 0, NULL,
			  pages, 0,
			  CEPH_OSD_FLAG_READ,
			  ops,
			  1,
			  NULL, 0,
			  rbd_simple_req_cb, 0, NULL);

	rbd_destroy_ops(ops);
	return ret;
}

static void rbd_watch_cb(u64 ver, u64 notify_id, u8 opcode, void *data)
{
	struct rbd_device *dev = (struct rbd_device *)data;
	u64 hver;
	int rc;

	if (!dev)
		return;

	dout("rbd_watch_cb %s notify_id=%lld opcode=%d\n", dev->obj_md_name,
		notify_id, (int)opcode);
	mutex_lock_nested(&ctl_mutex, SINGLE_DEPTH_NESTING);
	rc = __rbd_update_snaps(dev);
	hver = dev->header.obj_version;
	mutex_unlock(&ctl_mutex);
	if (rc)
		pr_warning(RBD_DRV_NAME "%d got notification but failed to "
			   " update snaps: %d\n", dev->major, rc);

	rbd_req_sync_notify_ack(dev, hver, notify_id, dev->obj_md_name);
}

/*
 * Request sync osd watch
 */
static int rbd_req_sync_watch(struct rbd_device *dev,
			      const char *obj,
			      u64 ver)
{
	struct ceph_osd_req_op *ops;
	struct ceph_osd_client *osdc = &dev->rbd_client->client->osdc;

	int ret = rbd_create_rw_ops(&ops, 1, CEPH_OSD_OP_WATCH, 0);
	if (ret < 0)
		return ret;

	ret = ceph_osdc_create_event(osdc, rbd_watch_cb, 0,
				     (void *)dev, &dev->watch_event);
	if (ret < 0)
		goto fail;

	ops[0].watch.ver = cpu_to_le64(ver);
	ops[0].watch.cookie = cpu_to_le64(dev->watch_event->cookie);
	ops[0].watch.flag = 1;

	ret = rbd_req_sync_op(dev, NULL,
			      CEPH_NOSNAP,
			      0,
			      CEPH_OSD_FLAG_WRITE | CEPH_OSD_FLAG_ONDISK,
			      ops,
			      1, obj, 0, 0, NULL,
			      &dev->watch_request, NULL);

	if (ret < 0)
		goto fail_event;

	rbd_destroy_ops(ops);
	return 0;

fail_event:
	ceph_osdc_cancel_event(rbd_dev->watch_event);
	rbd_dev->watch_event = NULL;
fail:
	rbd_destroy_ops(ops);
	return ret;
}

/*
 * Request sync osd unwatch
 */
static int rbd_req_sync_unwatch(struct rbd_device *dev,
				const char *obj)
{
	struct ceph_osd_req_op *ops;

	int ret = rbd_create_rw_ops(&ops, 1, CEPH_OSD_OP_WATCH, 0);
	if (ret < 0)
		return ret;

	ops[0].watch.ver = 0;
	ops[0].watch.cookie = cpu_to_le64(dev->watch_event->cookie);
	ops[0].watch.flag = 0;

	ret = rbd_req_sync_op(dev, NULL,
			      CEPH_NOSNAP,
			      0,
			      CEPH_OSD_FLAG_WRITE | CEPH_OSD_FLAG_ONDISK,
			      ops,
			      1, obj, 0, 0, NULL, NULL, NULL);

	rbd_destroy_ops(ops);
	ceph_osdc_cancel_event(dev->watch_event);
	dev->watch_event = NULL;
	return ret;
}

#if 0
/*
 * Request sync osd read
 */
static int rbd_req_sync_exec(struct rbd_device *rbd_dev,
			     const char *object_name,
			     const char *class_name,
			     const char *method_name,
			     const char *data,
			     int len,
			     u64 *ver)
{
	struct ceph_osd_req_op *ops;
	int class_name_len = strlen(class_name);
	int method_name_len = strlen(method_name);
	int ret;

	ops = rbd_create_rw_ops(1, CEPH_OSD_OP_CALL,
				    class_name_len + method_name_len + len);
	if (!ops)
		return -ENOMEM;

	ops[0].cls.class_name = class_name;
	ops[0].cls.class_len = (__u8) class_name_len;
	ops[0].cls.method_name = method_name;
	ops[0].cls.method_len = (__u8) method_name_len;
	ops[0].cls.argc = 0;
	ops[0].cls.indata = outbound;
	ops[0].cls.indata_len = outbound_size;

	ret = rbd_req_sync_op(rbd_dev, NULL,
			       CEPH_NOSNAP,
			       flags, ops,
			       object_name, 0, inbound_size, inbound,
			       NULL, ver);

	rbd_destroy_ops(ops);

	dout("cls_exec returned %d\n", ret);
	return ret;
}
#endif

static struct rbd_req_coll *rbd_alloc_coll(int num_reqs)
{
	struct rbd_req_coll *coll =
			kzalloc(sizeof(struct rbd_req_coll) +
			        sizeof(struct rbd_req_status) * num_reqs,
				GFP_ATOMIC);

	if (!coll)
		return NULL;
	coll->total = num_reqs;
	kref_init(&coll->kref);
	return coll;
}

/*
 * block device queue callback
 */
static void rbd_rq_fn(struct request_queue *q)
{
	struct rbd_device *rbd_dev = q->queuedata;
	struct request *rq;
	struct bio_pair *bp = NULL;

	while ((rq = blk_fetch_request(q))) {
		struct bio *bio;
		struct bio *rq_bio, *next_bio = NULL;
		bool do_write;
		int size, op_size = 0;
		u64 ofs;
		int num_segs, cur_seg = 0;
		struct rbd_req_coll *coll;
		struct ceph_snap_context *snapc;

		dout("fetched request\n");

		/* filter out block requests we don't understand */
		if ((rq->cmd_type != REQ_TYPE_FS)) {
			__blk_end_request_all(rq, 0);
			continue;
		}

		/* deduce our operation (read, write) */
		do_write = (rq_data_dir(rq) == WRITE);
		if (do_write && rbd_dev->mapping.read_only) {
			__blk_end_request_all(rq, -EROFS);
			continue;
		}

		spin_unlock_irq(q->queue_lock);

		down_read(&rbd_dev->header_rwsem);

		if (!rbd_dev->exists) {
			rbd_assert(rbd_dev->spec->snap_id != CEPH_NOSNAP);
			up_read(&rbd_dev->header_rwsem);
			dout("request for non-existent snapshot");
			spin_lock_irq(q->queue_lock);
			__blk_end_request_all(rq, -ENXIO);
			continue;
		}

		snapc = ceph_get_snap_context(rbd_dev->header.snapc);

		up_read(&rbd_dev->header_rwsem);

		size = blk_rq_bytes(rq);
		ofs = blk_rq_pos(rq) * SECTOR_SIZE;
		bio = rq->bio;

		dout("%s 0x%x bytes at 0x%llx\n",
		     do_write ? "write" : "read",
		     size, blk_rq_pos(rq) * SECTOR_SIZE);

		num_segs = rbd_get_num_segments(&rbd_dev->header, ofs, size);
		if (num_segs <= 0) {
			spin_lock_irq(q->queue_lock);
			__blk_end_request_all(rq, num_segs);
			ceph_put_snap_context(snapc);
			continue;
		}
		coll = rbd_alloc_coll(num_segs);
		if (!coll) {
			spin_lock_irq(q->queue_lock);
			__blk_end_request_all(rq, -ENOMEM);
			ceph_put_snap_context(snapc);
			continue;
		}

		do {
			/* a bio clone to be passed down to OSD req */
			dout("rq->bio->bi_vcnt=%d\n", rq->bio->bi_vcnt);
			op_size = rbd_get_segment(&rbd_dev->header,
						  rbd_dev->header.block_name,
						  ofs, size,
						  NULL, NULL);
			kref_get(&coll->kref);
			bio = bio_chain_clone(&rq_bio, &next_bio, &bp,
					      op_size, GFP_ATOMIC);
			if (!bio) {
				rbd_coll_end_req_index(rq, coll, cur_seg,
						       -ENOMEM, op_size);
				goto next_seg;
			}


			/* init OSD command: write or read */
			if (do_write)
				rbd_req_write(rq, rbd_dev,
					      snapc,
					      ofs,
					      op_size, bio,
					      coll, cur_seg);
			else
				rbd_req_read(rq, rbd_dev,
					     rbd_dev->mapping.snap_id,
					     ofs,
					     op_size, bio,
					     coll, cur_seg);

next_seg:
			size -= op_size;
			ofs += op_size;

			cur_seg++;
			rq_bio = next_bio;
		} while (size > 0);
		kref_put(&coll->kref, rbd_coll_release);

		if (bp)
			bio_pair_release(bp);
		spin_lock_irq(q->queue_lock);

		ceph_put_snap_context(snapc);
	}
}

/*
 * a queue callback. Makes sure that we don't create a bio that spans across
 * multiple osd objects. One exception would be with a single page bios,
 * which we handle later at bio_chain_clone_range()
 */
static int rbd_merge_bvec(struct request_queue *q, struct bvec_merge_data *bmd,
			  struct bio_vec *bvec)
{
	struct rbd_device *rbd_dev = q->queuedata;
	sector_t sector_offset;
	sector_t sectors_per_obj;
	sector_t obj_sector_offset;
	int ret;

	/*
	 * Find how far into its rbd object the partition-relative
	 * bio start sector is to offset relative to the enclosing
	 * device.
	 */
	sector_offset = get_start_sect(bmd->bi_bdev) + bmd->bi_sector;
	sectors_per_obj = 1 << (rbd_dev->header.obj_order - SECTOR_SHIFT);
	obj_sector_offset = sector_offset & (sectors_per_obj - 1);

	/*
	 * Compute the number of bytes from that offset to the end
	 * of the object.  Account for what's already used by the bio.
	 */
	ret = (int) (sectors_per_obj - obj_sector_offset) << SECTOR_SHIFT;
	if (ret > bmd->bi_size)
		ret -= bmd->bi_size;
	else
		ret = 0;

	/*
	 * Don't send back more than was asked for.  And if the bio
	 * was empty, let the whole thing through because:  "Note
	 * that a block device *must* allow a single page to be
	 * added to an empty bio."
	 */
	rbd_assert(bvec->bv_len <= PAGE_SIZE);
	if (ret > (int) bvec->bv_len || !bmd->bi_size)
		ret = (int) bvec->bv_len;

	return ret;
}

static void rbd_free_disk(struct rbd_device *rbd_dev)
{
	struct gendisk *disk = rbd_dev->disk;

	if (!disk)
		return;

	if (disk->flags & GENHD_FL_UP)
		del_gendisk(disk);
	if (disk->queue)
		blk_cleanup_queue(disk->queue);
	put_disk(disk);
}

/*
 * reload the ondisk the header 
 */
static int rbd_read_header(struct rbd_device *rbd_dev,
			   struct rbd_image_header *header)
{
	ssize_t rc;
	struct rbd_image_header_ondisk *dh;
	u32 snap_count = 0;
	u64 ver;
	size_t len;

	/*
	 * First reads the fixed-size header to determine the number
	 * of snapshots, then re-reads it, along with all snapshot
	 * records as well as their stored names.
	 */
	len = sizeof (*dh);
	while (1) {
		dh = kmalloc(len, GFP_KERNEL);
		if (!dh)
			return -ENOMEM;

		rc = rbd_req_sync_read(rbd_dev,
				       CEPH_NOSNAP,
				       rbd_dev->header_name,
				       0, len,
				       (char *)dh, &ver);
		if (rc < 0)
			goto out_dh;

		rc = rbd_header_from_disk(header, dh, snap_count);
		if (rc < 0) {
			if (rc == -ENXIO)
				pr_warning("unrecognized header format"
					   " for image %s\n",
					   rbd_dev->image_name);
			goto out_dh;
		}

		if (snap_count == header->total_snaps)
			break;

		snap_count = header->total_snaps;
		len = sizeof (*dh) +
			snap_count * sizeof(struct rbd_image_snap_ondisk) +
			header->snap_names_len;

		rbd_header_free(header);
		kfree(dh);
	}
	header->obj_version = ver;

out_dh:
	kfree(dh);
	return rc;
}

static void __rbd_remove_all_snaps(struct rbd_device *rbd_dev)
{
	struct rbd_snap *snap;

	while (!list_empty(&rbd_dev->snaps)) {
		snap = list_first_entry(&rbd_dev->snaps, struct rbd_snap, node);
		__rbd_remove_snap_dev(rbd_dev, snap);
	}
}

static int rbd_obj_read_sync(struct rbd_device *rbd_dev,
				const char *object_name,
				u64 offset, u64 length,
				char *buf, u64 *version)

{
	struct ceph_osd_req_op *op;
	struct rbd_obj_request *obj_request;
	struct ceph_osd_client *osdc;
	struct page **pages = NULL;
	u32 page_count;
	size_t size;
	int ret;

	page_count = (u32) calc_pages_for(offset, length);
	pages = ceph_alloc_page_vector(page_count, GFP_KERNEL);
	if (IS_ERR(pages))
		ret = PTR_ERR(pages);

	ret = -ENOMEM;
	obj_request = rbd_obj_request_create(object_name, offset, length,
							OBJ_REQUEST_PAGES);
	if (!obj_request)
		goto out;

	obj_request->pages = pages;
	obj_request->page_count = page_count;

	obj_request->osd_req = rbd_osd_req_create(rbd_dev, false, obj_request);
	if (!obj_request->osd_req)
		goto out;

	osd_req_op_extent_init(obj_request->osd_req, 0, CEPH_OSD_OP_READ,
					offset, length, 0, 0);
	osd_req_op_extent_osd_data_pages(obj_request->osd_req, 0,
					obj_request->pages,
					obj_request->length,
					obj_request->offset & ~PAGE_MASK,
					false, false);
	rbd_osd_req_format_read(obj_request);

	ret = rbd_obj_request_submit(osdc, obj_request);
	if (ret)
		goto out;
	ret = rbd_obj_request_wait(obj_request);
	if (ret)
		goto out;

	ret = obj_request->result;
	if (ret < 0)
		goto out;

	rbd_assert(obj_request->xferred <= (u64) SIZE_MAX);
	size = (size_t) obj_request->xferred;
	ceph_copy_from_page_vector(pages, buf, 0, size);
	rbd_assert(size <= (size_t)INT_MAX);
	ret = (int)size;
out:
	if (obj_request)
		rbd_obj_request_put(obj_request);
	else
		ceph_release_page_vector(pages, page_count);

	return ret;
}

/*
 * only read the first part of the ondisk header, without the snaps info
 */
static int __rbd_refresh_header(struct rbd_device *rbd_dev)
{
	int ret;
	struct rbd_image_header h;
	u64 snap_seq;
	int follow_seq = 0;

	ret = rbd_read_header(rbd_dev, &h);
	if (ret < 0)
		return ret;

	down_write(&rbd_dev->header_rwsem);

	/* Update image size, and check for resize of mapped image */
	rbd_dev->header.image_size = h.image_size;
	rbd_update_mapping_size(rbd_dev);

	snap_seq = rbd_dev->header.snapc->seq;
	if (rbd_dev->header.total_snaps &&
	    rbd_dev->header.snapc->snaps[0] == snap_seq)
		/* pointing at the head, will need to follow that
		   if head moves */
		follow_seq = 1;

	ceph_put_snap_context(rbd_dev->header.snapc);
	kfree(rbd_dev->header.snap_names);
	kfree(rbd_dev->header.snap_sizes);

	rbd_dev->header.obj_version = h.obj_version;
	rbd_dev->header.image_size = h.image_size;
	rbd_dev->header.total_snaps = h.total_snaps;
	rbd_dev->header.snapc = h.snapc;
	rbd_dev->header.snap_names = h.snap_names;
	rbd_dev->header.snap_names_len = h.snap_names_len;
	rbd_dev->header.snap_sizes = h.snap_sizes;
	if (follow_seq)
		rbd_dev->header.snapc->seq = rbd_dev->header.snapc->snaps[0];
	else
		rbd_dev->header.snapc->seq = snap_seq;

	ret = __rbd_init_snaps_header(rbd_dev);

	up_write(&rbd_dev->header_rwsem);

	return ret;
}

static int rbd_dev_refresh(struct rbd_device *rbd_dev, u64 *hver)
{
	int ret;

	rbd_assert(rbd_image_format_valid(rbd_dev->image_format));
	mutex_lock_nested(&ctl_mutex, SINGLE_DEPTH_NESTING);
	if (rbd_dev->image_format == 1)
		ret = rbd_dev_v1_refresh(rbd_dev, hver);
	else
		ret = rbd_dev_v2_refresh(rbd_dev, hver);
	mutex_unlock(&ctl_mutex);

	return ret;
}

static int rbd_init_disk(struct rbd_device *rbd_dev)
{
	struct gendisk *disk;
	struct request_queue *q;
	u64 segment_size;

	/* create gendisk info */
	disk = alloc_disk(RBD_MINORS_PER_MAJOR);
	if (!disk)
		return -ENOMEM;

	snprintf(disk->disk_name, sizeof(disk->disk_name), RBD_DRV_NAME "%d",
		 rbd_dev->dev_id);
	disk->major = rbd_dev->major;
	disk->first_minor = 0;
	disk->fops = &rbd_bd_ops;
	disk->private_data = rbd_dev;

	q = blk_init_queue(rbd_request_fn, &rbd_dev->lock);
	if (!q)
		goto out_disk;

	/* We use the default size, but let's be explicit about it. */
	blk_queue_physical_block_size(q, SECTOR_SIZE);

	/* set io sizes to object size */
	segment_size = rbd_obj_bytes(&rbd_dev->header);
	blk_queue_max_hw_sectors(q, segment_size / SECTOR_SIZE);
	blk_queue_max_segment_size(q, segment_size);
	blk_queue_io_min(q, segment_size);
	blk_queue_io_opt(q, segment_size);

	blk_queue_merge_bvec(q, rbd_merge_bvec);
	disk->queue = q;

	q->queuedata = rbd_dev;

	rbd_dev->disk = disk;

	return 0;
out_disk:
	put_disk(disk);

	return -ENOMEM;
}

/*
  sysfs
*/

static struct rbd_device *dev_to_rbd_dev(struct device *dev)
{
	return container_of(dev, struct rbd_device, dev);
}

static ssize_t rbd_size_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct rbd_device *rbd_dev = dev_to_rbd_dev(dev);

	return sprintf(buf, "%llu\n",
		(unsigned long long)rbd_dev->mapping.size);
}

/*
 * Note this shows the features for whatever's mapped, which is not
 * necessarily the base image.
 */
static ssize_t rbd_features_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct rbd_device *rbd_dev = dev_to_rbd_dev(dev);

	return sprintf(buf, "0x%016llx\n",
			(unsigned long long)rbd_dev->mapping.features);
}

static ssize_t rbd_major_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct rbd_device *rbd_dev = dev_to_rbd_dev(dev);

	if (rbd_dev->major)
		return sprintf(buf, "%d\n", rbd_dev->major);

	return sprintf(buf, "(none)\n");

}

static ssize_t rbd_client_id_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct rbd_device *rbd_dev = dev_to_rbd_dev(dev);

	return sprintf(buf, "client%lld\n",
			ceph_client_id(rbd_dev->rbd_client->client));
}

static ssize_t rbd_pool_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct rbd_device *rbd_dev = dev_to_rbd_dev(dev);

	return sprintf(buf, "%s\n", rbd_dev->spec->pool_name);
}

static ssize_t rbd_pool_id_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct rbd_device *rbd_dev = dev_to_rbd_dev(dev);

	return sprintf(buf, "%llu\n",
			(unsigned long long) rbd_dev->spec->pool_id);
}

static ssize_t rbd_name_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct rbd_device *rbd_dev = dev_to_rbd_dev(dev);

	if (rbd_dev->spec->image_name)
		return sprintf(buf, "%s\n", rbd_dev->spec->image_name);

	return sprintf(buf, "(unknown)\n");
}

static ssize_t rbd_image_id_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct rbd_device *rbd_dev = dev_to_rbd_dev(dev);

	return sprintf(buf, "%s\n", rbd_dev->spec->image_id);
}

/*
 * Shows the name of the currently-mapped snapshot (or
 * RBD_SNAP_HEAD_NAME for the base image).
 */
static ssize_t rbd_snap_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct rbd_device *rbd_dev = dev_to_rbd_dev(dev);

	return sprintf(buf, "%s\n", rbd_dev->spec->snap_name);
}

/*
 * For an rbd v2 image, shows the pool id, image id, and snapshot id
 * for the parent image.  If there is no parent, simply shows
 * "(no parent image)".
 */
static ssize_t rbd_parent_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct rbd_device *rbd_dev = dev_to_rbd_dev(dev);
	struct rbd_spec *spec = rbd_dev->parent_spec;
	int count;
	char *bufp = buf;

	if (!spec)
		return sprintf(buf, "(no parent image)\n");

	count = sprintf(bufp, "pool_id %llu\npool_name %s\n",
			(unsigned long long) spec->pool_id, spec->pool_name);
	if (count < 0)
		return count;
	bufp += count;

	count = sprintf(bufp, "image_id %s\nimage_name %s\n", spec->image_id,
			spec->image_name ? spec->image_name : "(unknown)");
	if (count < 0)
		return count;
	bufp += count;

	count = sprintf(bufp, "snap_id %llu\nsnap_name %s\n",
			(unsigned long long) spec->snap_id, spec->snap_name);
	if (count < 0)
		return count;
	bufp += count;

	count = sprintf(bufp, "overlap %llu\n", rbd_dev->parent_overlap);
	if (count < 0)
		return count;
	bufp += count;

	return (ssize_t) (bufp - buf);
}

static ssize_t rbd_image_refresh(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t size)
{
	struct rbd_device *rbd_dev = dev_to_rbd_dev(dev);
	int ret;

	ret = rbd_dev_refresh(rbd_dev);
	if (ret)
		rbd_warn(rbd_dev, ": manual header refresh error (%d)\n", ret);

	return ret < 0 ? ret : size;
}

static DEVICE_ATTR(size, S_IRUGO, rbd_size_show, NULL);
static DEVICE_ATTR(major, S_IRUGO, rbd_major_show, NULL);
static DEVICE_ATTR(client_id, S_IRUGO, rbd_client_id_show, NULL);
static DEVICE_ATTR(pool, S_IRUGO, rbd_pool_show, NULL);
static DEVICE_ATTR(pool_id, S_IRUGO, rbd_pool_id_show, NULL);
static DEVICE_ATTR(name, S_IRUGO, rbd_name_show, NULL);
static DEVICE_ATTR(refresh, S_IWUSR, NULL, rbd_image_refresh);
static DEVICE_ATTR(current_snap, S_IRUGO, rbd_snap_show, NULL);
static DEVICE_ATTR(parent, S_IRUGO, rbd_parent_show, NULL);

static struct attribute *rbd_attrs[] = {
	&dev_attr_size.attr,
	&dev_attr_major.attr,
	&dev_attr_client_id.attr,
	&dev_attr_pool.attr,
	&dev_attr_pool_id.attr,
	&dev_attr_name.attr,
	&dev_attr_current_snap.attr,
	&dev_attr_parent.attr,
	&dev_attr_refresh.attr,
	NULL
};

static struct attribute_group rbd_attr_group = {
	.attrs = rbd_attrs,
};

static const struct attribute_group *rbd_attr_groups[] = {
	&rbd_attr_group,
	NULL
};

static void rbd_sysfs_dev_release(struct device *dev)
{
}

static struct device_type rbd_device_type = {
	.name		= "rbd",
	.groups		= rbd_attr_groups,
	.release	= rbd_sysfs_dev_release,
};

static struct rbd_spec *rbd_spec_get(struct rbd_spec *spec)
{
	kref_get(&spec->kref);

	return spec;
}

static void rbd_spec_free(struct kref *kref);
static void rbd_spec_put(struct rbd_spec *spec)
{
	if (spec)
		kref_put(&spec->kref, rbd_spec_free);
}

static struct rbd_spec *rbd_spec_alloc(void)
{
	struct rbd_spec *spec;

	spec = kzalloc(sizeof (*spec), GFP_KERNEL);
	if (!spec)
		return NULL;
	kref_init(&spec->kref);

	return spec;
}

static void rbd_spec_free(struct kref *kref)
{
	struct rbd_spec *spec = container_of(kref, struct rbd_spec, kref);

	kfree(spec->pool_name);
	kfree(spec->image_id);
	kfree(spec->image_name);
	kfree(spec->snap_name);
	kfree(spec);
}

static struct rbd_device *rbd_dev_create(struct rbd_client *rbdc,
				struct rbd_spec *spec)
{
	struct rbd_device *rbd_dev;

	rbd_dev = kzalloc(sizeof (*rbd_dev), GFP_KERNEL);
	if (!rbd_dev)
		return NULL;

	spin_lock_init(&rbd_dev->lock);
	rbd_dev->flags = 0;
	atomic_set(&rbd_dev->parent_ref, 0);
	INIT_LIST_HEAD(&rbd_dev->node);
	init_rwsem(&rbd_dev->header_rwsem);

	rbd_dev->spec = spec;
	rbd_dev->rbd_client = rbdc;

	/* Initialize the layout used for all rbd requests */

	rbd_dev->layout.fl_stripe_unit = cpu_to_le32(1 << RBD_MAX_OBJ_ORDER);
	rbd_dev->layout.fl_stripe_count = cpu_to_le32(1);
	rbd_dev->layout.fl_object_size = cpu_to_le32(1 << RBD_MAX_OBJ_ORDER);
	rbd_dev->layout.fl_pg_pool = cpu_to_le32((u32) spec->pool_id);

	return rbd_dev;
}

static void rbd_dev_destroy(struct rbd_device *rbd_dev)
{
	rbd_spec_put(rbd_dev->parent_spec);
	kfree(rbd_dev->header_name);
	rbd_put_client(rbd_dev->rbd_client);
	rbd_spec_put(rbd_dev->spec);
	kfree(rbd_dev);
}

static bool rbd_snap_registered(struct rbd_snap *snap)
{
	bool ret = snap->dev.type == &rbd_snap_device_type;
	bool reg = device_is_registered(&snap->dev);

	rbd_assert(!ret ^ reg);

	return ret;
}

static void rbd_remove_snap_dev(struct rbd_snap *snap)
{
	list_del(&snap->node);
	if (device_is_registered(&snap->dev))
		device_unregister(&snap->dev);
}

static int rbd_register_snap_dev(struct rbd_snap *snap,
				  struct device *parent)
{
	struct device *dev = &snap->dev;
	int ret;

	dev->type = &rbd_snap_device_type;
	dev->parent = parent;
	dev->release = rbd_snap_dev_release;
	dev_set_name(dev, "%s%s", RBD_SNAP_DEV_NAME_PREFIX, snap->name);
	dout("%s: registering device for snapshot %s\n", __func__, snap->name);

	ret = device_register(dev);

	return ret;
}

static struct rbd_snap *__rbd_add_snap_dev(struct rbd_device *rbd_dev,
						const char *snap_name,
						u64 snap_id, u64 snap_size,
						u64 snap_features)
{
	struct rbd_snap *snap;
	int ret;

	snap = kzalloc(sizeof (*snap), GFP_KERNEL);
	if (!snap)
		return ERR_PTR(-ENOMEM);

	ret = -ENOMEM;
	snap->name = kstrdup(snap_name, GFP_KERNEL);
	if (!snap->name)
		goto err;

	snap->id = snap_id;
	snap->size = snap_size;
	snap->features = snap_features;

	return snap;

err:
	kfree(snap->name);
	kfree(snap);

	return ERR_PTR(ret);
}

/*
 * search for the previous snap in a null delimited string list
 */
const char *rbd_prev_snap_name(const char *name, const char *start)
{
	if (name < start + 2)
		return NULL;

	name -= 2;
	while (*name) {
		if (name == start)
			return start;
		name--;
	}
	return name + 1;
}

/*
 * compare the old list of snapshots that we have to what's in the header
 * and update it accordingly. Note that the header holds the snapshots
 * in a reverse order (from newest to oldest) and we need to go from
 * older to new so that we don't get a duplicate snap name when
 * doing the process (e.g., removed snapshot and recreated a new
 * one with the same name.
 */
static int __rbd_init_snaps_header(struct rbd_device *rbd_dev)
{
	const char *name, *first_name;
	int i = rbd_dev->header.total_snaps;
	struct rbd_snap *snap, *old_snap = NULL;
	struct list_head *p, *n;

	first_name = rbd_dev->header.snap_names;
	name = first_name + rbd_dev->header.snap_names_len;

	list_for_each_prev_safe(p, n, &rbd_dev->snaps) {
		u64 cur_id;

		old_snap = list_entry(p, struct rbd_snap, node);

		if (i)
			cur_id = rbd_dev->header.snapc->snaps[i - 1];

		if (!i || old_snap->id < cur_id) {
			/*
			 * old_snap->id was skipped, thus was
			 * removed.  If this rbd_dev is mapped to
			 * the removed snapshot, record that it no
			 * longer exists, to prevent further I/O.
			 */
			if (rbd_dev->snap_id == old_snap->id)
				rbd_dev->snap_exists = false;
			__rbd_remove_snap_dev(rbd_dev, old_snap);
			continue;
		}
		if (old_snap->id == cur_id) {
			/* we have this snapshot already */
			i--;
			name = rbd_prev_snap_name(name, first_name);
			continue;
		}
		for (; i > 0;
		     i--, name = rbd_prev_snap_name(name, first_name)) {
			if (!name) {
				WARN_ON(1);
				return -EINVAL;
			}
			cur_id = rbd_dev->header.snapc->snaps[i];
			/* snapshot removal? handle it above */
			if (cur_id >= old_snap->id)
				break;
			/* a new snapshot */
			snap = __rbd_add_snap_dev(rbd_dev, i - 1, name);
			if (IS_ERR(snap))
				return PTR_ERR(snap);

			/* note that we add it backward so using n and not p */
			list_add(&snap->node, n);
			p = &snap->node;
		}
	}
	/* we're done going over the old snap list, just add what's left */
	for (; i > 0; i--) {
		name = rbd_prev_snap_name(name, first_name);
		if (!name) {
			WARN_ON(1);
			return -EINVAL;
		}
		snap = __rbd_add_snap_dev(rbd_dev, i - 1, name);
		if (IS_ERR(snap))
			return PTR_ERR(snap);
		list_add(&snap->node, &rbd_dev->snaps);
	}

	return 0;
}

static int rbd_bus_add_dev(struct rbd_device *rbd_dev)
{
	int ret;
	struct device *dev;
	struct rbd_snap *snap;

	mutex_lock_nested(&ctl_mutex, SINGLE_DEPTH_NESTING);
	dev = &rbd_dev->dev;

	dev->bus = &rbd_bus_type;
	dev->type = &rbd_device_type;
	dev->parent = &rbd_root_dev;
	dev->release = rbd_dev_device_release;
	dev_set_name(dev, "%d", rbd_dev->dev_id);
	ret = device_register(dev);
	if (ret < 0)
		goto out;
	ret = 0;    /* rbd_req_sync_exec() can return positive */

	list_for_each_entry(snap, &rbd_dev->snaps, node) {
		ret = rbd_register_snap_dev(snap, &rbd_dev->dev);
		if (ret < 0)
			break;
	}
out:
	mutex_unlock(&ctl_mutex);
	return ret;
}

static void rbd_bus_del_dev(struct rbd_device *rbd_dev)
{
	device_unregister(&rbd_dev->dev);
}

static int rbd_init_watch_dev(struct rbd_device *rbd_dev)
{
	int ret, rc;

	do {
		ret = rbd_req_sync_watch(rbd_dev);
		if (ret == -ERANGE) {
			rc = rbd_refresh_header(rbd_dev, NULL);
			if (rc < 0)
				return rc;
		}
	} while (ret == -ERANGE);

	return ret;
}

static atomic64_t rbd_dev_id_max = ATOMIC64_INIT(0);

/*
 * Get a unique rbd identifier for the given new rbd_dev, and add
 * the rbd_dev to the global list.  The minimum rbd id is 1.
 */
static void rbd_dev_id_get(struct rbd_device *rbd_dev)
{
	rbd_dev->dev_id = atomic64_inc_return(&rbd_dev_id_max);

	spin_lock(&rbd_dev_list_lock);
	list_add_tail(&rbd_dev->node, &rbd_dev_list);
	spin_unlock(&rbd_dev_list_lock);
	dout("rbd_dev %p given dev id %llu\n", rbd_dev,
		(unsigned long long) rbd_dev->dev_id);
}

/*
 * Remove an rbd_dev from the global list, and record that its
 * identifier is no longer in use.
 */
static void rbd_dev_id_put(struct rbd_device *rbd_dev)
{
	struct list_head *tmp;
	int rbd_id = rbd_dev->dev_id;
	int max_id;

	rbd_assert(rbd_id > 0);

	dout("rbd_dev %p released dev id %llu\n", rbd_dev,
		(unsigned long long) rbd_dev->dev_id);
	spin_lock(&rbd_dev_list_lock);
	list_del_init(&rbd_dev->node);

	/*
	 * If the id being "put" is not the current maximum, there
	 * is nothing special we need to do.
	 */
	if (rbd_id != atomic64_read(&rbd_dev_id_max)) {
		spin_unlock(&rbd_dev_list_lock);
		return;
	}

	/*
	 * We need to update the current maximum id.  Search the
	 * list to find out what it is.  We're more likely to find
	 * the maximum at the end, so search the list backward.
	 */
	max_id = 0;
	list_for_each_prev(tmp, &rbd_dev_list) {
		struct rbd_device *rbd_dev;

		rbd_dev = list_entry(tmp, struct rbd_device, node);
		if (rbd_dev->id > max_id)
			max_id = rbd_dev->id;
	}
	spin_unlock(&rbd_dev_list_lock);

	/*
	 * The max id could have been updated by rbd_dev_id_get(), in
	 * which case it now accurately reflects the new maximum.
	 * Be careful not to overwrite the maximum value in that
	 * case.
	 */
	atomic64_cmpxchg(&rbd_dev_id_max, rbd_id, max_id);
	dout("  max dev id has been reset\n");
}

/*
 * Skips over white space at *buf, and updates *buf to point to the
 * first found non-space character (if any). Returns the length of
 * the token (string of non-white space characters) found.  Note
 * that *buf must be terminated with '\0'.
 */
static inline size_t next_token(const char **buf)
{
        /*
        * These are the characters that produce nonzero for
        * isspace() in the "C" and "POSIX" locales.
        */
        const char *spaces = " \f\n\r\t\v";

        *buf += strspn(*buf, spaces);	/* Find start of token */

	return strcspn(*buf, spaces);   /* Return token length */
}

/*
 * Finds the next token in *buf, and if the provided token buffer is
 * big enough, copies the found token into it.  The result, if
 * copied, is guaranteed to be terminated with '\0'.  Note that *buf
 * must be terminated with '\0' on entry.
 *
 * Returns the length of the token found (not including the '\0').
 * Return value will be 0 if no token is found, and it will be >=
 * token_size if the token would not fit.
 *
 * The *buf pointer will be updated to point beyond the end of the
 * found token.  Note that this occurs even if the token buffer is
 * too small to hold it.
 */
static inline size_t copy_token(const char **buf,
				char *token,
				size_t token_size)
{
        size_t len;

	len = next_token(buf);
	if (len < token_size) {
		memcpy(token, *buf, len);
		*(token + len) = '\0';
	}
	*buf += len;

        return len;
}

/*
 * Finds the next token in *buf, dynamically allocates a buffer big
 * enough to hold a copy of it, and copies the token into the new
 * buffer.  The copy is guaranteed to be terminated with '\0'.  Note
 * that a duplicate buffer is created even for a zero-length token.
 *
 * Returns a pointer to the newly-allocated duplicate, or a null
 * pointer if memory for the duplicate was not available.  If
 * the lenp argument is a non-null pointer, the length of the token
 * (not including the '\0') is returned in *lenp.
 *
 * If successful, the *buf pointer will be updated to point beyond
 * the end of the found token.
 *
 * Note: uses GFP_KERNEL for allocation.
 */
static inline char *dup_token(const char **buf, size_t *lenp)
{
	char *dup;
	size_t len;

	len = next_token(buf);
	dup = kmemdup(*buf, len + 1, GFP_KERNEL);
	if (!dup)
		return NULL;
	*(dup + len) = '\0';
	*buf += len;

	if (lenp)
		*lenp = len;

	return dup;
}

static int rbd_dev_v2_refresh(struct rbd_device *rbd_dev, u64 *hver)
{
	int ret;
	__u8 obj_order;

	down_write(&rbd_dev->header_rwsem);

	/* Grab old order first, to see if it changes */

	obj_order = rbd_dev->header.obj_order,
	ret = rbd_dev_v2_image_size(rbd_dev);
	if (ret)
		goto out;
	if (rbd_dev->header.obj_order != obj_order) {
		ret = -EIO;
		goto out;
	}
	rbd_update_mapping_size(rbd_dev);

	ret = rbd_dev_v2_snap_context(rbd_dev, hver);
	dout("rbd_dev_v2_snap_context returned %d\n", ret);
	if (ret)
		goto out;
	ret = rbd_dev_snaps_update(rbd_dev);
	dout("rbd_dev_snaps_update returned %d\n", ret);
	if (ret)
		goto out;
	ret = rbd_dev_snaps_register(rbd_dev);
	dout("rbd_dev_snaps_register returned %d\n", ret);
out:
	up_write(&rbd_dev->header_rwsem);

	return ret;
}

/*
 * Parse the options provided for an "rbd add" (i.e., rbd image
 * mapping) request.  These arrive via a write to /sys/bus/rbd/add,
 * and the data written is passed here via a NUL-terminated buffer.
 * Returns 0 if successful or an error code otherwise.
 *
 * The information extracted from these options is recorded in
 * the other parameters which return dynamically-allocated
 * structures:
 *  ceph_opts
 *      The address of a pointer that will refer to a ceph options
 *      structure.  Caller must release the returned pointer using
 *      ceph_destroy_options() when it is no longer needed.
 *  rbd_opts
 *	Address of an rbd options pointer.  Fully initialized by
 *	this function; caller must release with kfree().
 *  spec
 *	Address of an rbd image specification pointer.  Fully
 *	initialized by this function based on parsed options.
 *	Caller must release with rbd_spec_put().
 *
 * The options passed take this form:
 *  <mon_addrs> <options> <pool_name> <image_name> [<snap_id>]
 * where:
 *  <mon_addrs>
 *      A comma-separated list of one or more monitor addresses.
 *      A monitor address is an ip address, optionally followed
 *      by a port number (separated by a colon).
 *        I.e.:  ip1[:port1][,ip2[:port2]...]
 *  <options>
 *      A comma-separated list of ceph and/or rbd options.
 *  <pool_name>
 *      The name of the rados pool containing the rbd image.
 *  <image_name>
 *      The name of the image in that pool to map.
 *  <snap_id>
 *      An optional snapshot id.  If provided, the mapping will
 *      present data from the image at the time that snapshot was
 *      created.  The image head is used if no snapshot id is
 *      provided.  Snapshot mappings are always read-only.
 */
static int rbd_add_parse_args(const char *buf,
				struct ceph_options **ceph_opts,
				struct rbd_options **opts,
				struct rbd_spec **rbd_spec)
{
	size_t len;
	char *options;
	const char *mon_addrs;
	char *snap_name;
	size_t mon_addrs_size;
	struct rbd_spec *spec = NULL;
	struct rbd_options *rbd_opts = NULL;
	struct ceph_options *copts;
	int ret;

	/* The first four tokens are required */

	len = next_token(&buf);
	if (!len) {
		rbd_warn(NULL, "no monitor address(es) provided");
		return -EINVAL;
	}
	mon_addrs = buf;
	mon_addrs_size = len + 1;
	buf += len;

	ret = -EINVAL;
	options = dup_token(&buf, NULL);
	if (!options)
		return -ENOMEM;
	if (!*options) {
		rbd_warn(NULL, "no options provided");
		goto out_err;
	}

	spec = rbd_spec_alloc();
	if (!spec)
		goto out_mem;

	spec->pool_name = dup_token(&buf, NULL);
	if (!spec->pool_name)
		goto out_mem;
	if (!*spec->pool_name) {
		rbd_warn(NULL, "no pool name provided");
		goto out_err;
	}

	spec->image_name = dup_token(&buf, NULL);
	if (!spec->image_name)
		goto out_mem;
	if (!*spec->image_name) {
		rbd_warn(NULL, "no image name provided");
		goto out_err;
	}

	/*
	 * Snapshot name is optional; default is to use "-"
	 * (indicating the head/no snapshot).
	 */
	len = next_token(&buf);
	if (!len) {
		buf = RBD_SNAP_HEAD_NAME; /* No snapshot supplied */
		len = sizeof (RBD_SNAP_HEAD_NAME) - 1;
	} else if (len > RBD_MAX_SNAP_NAME_LEN) {
		ret = -ENAMETOOLONG;
		goto out_err;
	}
	snap_name = kmemdup(buf, len + 1, GFP_KERNEL);
	if (!snap_name)
		goto out_mem;
	*(snap_name + len) = '\0';
	spec->snap_name = snap_name;

	/* Initialize all rbd options to the defaults */

	rbd_opts = kzalloc(sizeof (*rbd_opts), GFP_KERNEL);
	if (!rbd_opts)
		goto out_mem;

	rbd_opts->read_only = RBD_READ_ONLY_DEFAULT;

	copts = ceph_parse_options(options, mon_addrs,
					mon_addrs + mon_addrs_size - 1,
					parse_rbd_opts_token, rbd_opts);
	if (IS_ERR(copts)) {
		ret = PTR_ERR(copts);
		goto out_err;
	}
	kfree(options);

	*ceph_opts = copts;
	*opts = rbd_opts;
	*rbd_spec = spec;

	return 0;
out_mem:
	ret = -ENOMEM;
out_err:
	kfree(rbd_opts);
	rbd_spec_put(spec);
	kfree(options);

	return ret;
}

/*
 * An rbd format 2 image has a unique identifier, distinct from the
 * name given to it by the user.  Internally, that identifier is
 * what's used to specify the names of objects related to the image.
 *
 * A special "rbd id" object is used to map an rbd image name to its
 * id.  If that object doesn't exist, then there is no v2 rbd image
 * with the supplied name.
 *
 * This function will record the given rbd_dev's image_id field if
 * it can be determined, and in that case will return 0.  If any
 * errors occur a negative errno will be returned and the rbd_dev's
 * image_id field will be unchanged (and should be NULL).
 */
static int rbd_dev_image_id(struct rbd_device *rbd_dev)
{
	int ret;
	size_t size;
	char *object_name;
	void *response;
	char *image_id;

	/*
	 * When probing a parent image, the image id is already
	 * known (and the image name likely is not).  There's no
	 * need to fetch the image id again in this case.  We
	 * do still need to set the image format though.
	 */
	if (rbd_dev->spec->image_id) {
		rbd_dev->image_format = *rbd_dev->spec->image_id ? 2 : 1;

		return 0;
	}

	/*
	 * First, see if the format 2 image id file exists, and if
	 * so, get the image's persistent id from it.
	 */
	size = sizeof (RBD_ID_PREFIX) + strlen(rbd_dev->spec->image_name);
	object_name = kmalloc(size, GFP_NOIO);
	if (!object_name)
		return -ENOMEM;
	sprintf(object_name, "%s%s", RBD_ID_PREFIX, rbd_dev->spec->image_name);
	dout("rbd id object name is %s\n", object_name);

	/* Response will be an encoded string, which includes a length */

	size = sizeof (__le32) + RBD_IMAGE_ID_LEN_MAX;
	response = kzalloc(size, GFP_NOIO);
	if (!response) {
		ret = -ENOMEM;
		goto out;
	}

	/* If it doesn't exist we'll assume it's a format 1 image */

	ret = rbd_obj_method_sync(rbd_dev, object_name,
				"rbd", "get_id", NULL, 0,
				response, RBD_IMAGE_ID_LEN_MAX);
	dout("%s: rbd_obj_method_sync returned %d\n", __func__, ret);
	if (ret == -ENOENT) {
		image_id = kstrdup("", GFP_KERNEL);
		ret = image_id ? 0 : -ENOMEM;
		if (!ret)
			rbd_dev->image_format = 1;
	} else if (ret > sizeof (__le32)) {
		void *p = response;

		image_id = ceph_extract_encoded_string(&p, p + ret,
						NULL, GFP_NOIO);
		ret = IS_ERR(image_id) ? PTR_ERR(image_id) : 0;
		if (!ret)
			rbd_dev->image_format = 2;
	} else {
		ret = -EINVAL;
	}

	if (!ret) {
		rbd_dev->spec->image_id = image_id;
		dout("image_id is %s\n", image_id);
	}
out:
	kfree(response);
	kfree(object_name);

	return ret;
}

/*
 * Undo whatever state changes are made by v1 or v2 header info
 * call.
 */
static void rbd_dev_unprobe(struct rbd_device *rbd_dev)
{
	struct rbd_image_header	*header;

	/* Drop parent reference unless it's already been done (or none) */

	if (rbd_dev->parent_overlap)
		rbd_dev_parent_put(rbd_dev);

	/* Free dynamic fields from the header, then zero it out */

	header = &rbd_dev->header;
	ceph_put_snap_context(header->snapc);
	kfree(header->snap_sizes);
	kfree(header->snap_names);
	kfree(header->object_prefix);
	memset(header, 0, sizeof (*header));
}

static int rbd_dev_v2_header_onetime(struct rbd_device *rbd_dev)
{
	int ret;

	ret = rbd_dev_v2_object_prefix(rbd_dev);
	if (ret)
		goto out_err;

	/*
	 * Get the and check features for the image.  Currently the
	 * features are assumed to never change.
	 */
	ret = rbd_dev_v2_features(rbd_dev);
	if (ret)
		goto out_err;

	/* If the image supports fancy striping, get its parameters */

	if (rbd_dev->header.features & RBD_FEATURE_STRIPINGV2) {
		ret = rbd_dev_v2_striping_info(rbd_dev);
		if (ret < 0)
			goto out_err;
	}
	/* No support for crypto and compression type format 2 images */

	return 0;
out_err:
	rbd_dev->header.features = 0;
	kfree(rbd_dev->header.object_prefix);
	rbd_dev->header.object_prefix = NULL;

	return ret;
}

static int rbd_dev_probe_parent(struct rbd_device *rbd_dev)
{
	struct rbd_device *parent = NULL;
	struct rbd_spec *parent_spec;
	struct rbd_client *rbdc;
	int ret;

	if (!rbd_dev->parent_spec)
		return 0;
	/*
	 * We need to pass a reference to the client and the parent
	 * spec when creating the parent rbd_dev.  Images related by
	 * parent/child relationships always share both.
	 */
	parent_spec = rbd_spec_get(rbd_dev->parent_spec);
	rbdc = __rbd_get_client(rbd_dev->rbd_client);

	ret = -ENOMEM;
	parent = rbd_dev_create(rbdc, parent_spec);
	if (!parent)
		goto out_err;

	ret = rbd_dev_image_probe(parent, false);
	if (ret < 0)
		goto out_err;
	rbd_dev->parent = parent;
	atomic_set(&rbd_dev->parent_ref, 1);

	return 0;
out_err:
	if (parent) {
		rbd_dev_unparent(rbd_dev);
		kfree(rbd_dev->header_name);
		rbd_dev_destroy(parent);
	} else {
		rbd_put_client(rbdc);
		rbd_spec_put(parent_spec);
	}

	return ret;
}

static int rbd_dev_device_setup(struct rbd_device *rbd_dev)
{
	int ret;

	/* generate unique id: find highest unique id, add one */
	rbd_dev_id_get(rbd_dev);

	/* Fill in the device name, now that we have its id. */
	BUILD_BUG_ON(DEV_NAME_LEN
			< sizeof (RBD_DRV_NAME) + MAX_INT_FORMAT_WIDTH);
	sprintf(rbd_dev->name, "%s%d", RBD_DRV_NAME, rbd_dev->dev_id);

	/* Get our block major device number. */

	ret = register_blkdev(0, rbd_dev->name);
	if (ret < 0)
		goto err_out_id;
	rbd_dev->major = ret;

	/* Set up the blkdev mapping. */

	ret = rbd_init_disk(rbd_dev);
	if (ret)
		goto err_out_blkdev;

	ret = rbd_dev_mapping_set(rbd_dev);
	if (ret)
		goto err_out_disk;
	set_capacity(rbd_dev->disk, rbd_dev->mapping.size / SECTOR_SIZE);

	ret = rbd_bus_add_dev(rbd_dev);
	if (ret)
		goto err_out_mapping;

	/* Everything's ready.  Announce the disk to the world. */

	set_bit(RBD_DEV_FLAG_EXISTS, &rbd_dev->flags);
	add_disk(rbd_dev->disk);

	pr_info("%s: added with size 0x%llx\n", rbd_dev->disk->disk_name,
		(unsigned long long) rbd_dev->mapping.size);

	return ret;

err_out_mapping:
	rbd_dev_mapping_clear(rbd_dev);
err_out_disk:
	rbd_free_disk(rbd_dev);
err_out_blkdev:
	unregister_blkdev(rbd_dev->major, rbd_dev->name);
err_out_id:
	rbd_dev_id_put(rbd_dev);
	rbd_dev_mapping_clear(rbd_dev);

	return ret;
}

static int rbd_dev_header_name(struct rbd_device *rbd_dev)
{
	struct rbd_spec *spec = rbd_dev->spec;
	size_t size;

	/* Record the header object name for this rbd image. */

	rbd_assert(rbd_image_format_valid(rbd_dev->image_format));

	if (rbd_dev->image_format == 1)
		size = strlen(spec->image_name) + sizeof (RBD_SUFFIX);
	else
		size = sizeof (RBD_HEADER_PREFIX) + strlen(spec->image_id);

	rbd_dev->header_name = kmalloc(size, GFP_KERNEL);
	if (!rbd_dev->header_name)
		return -ENOMEM;

	if (rbd_dev->image_format == 1)
		sprintf(rbd_dev->header_name, "%s%s",
			spec->image_name, RBD_SUFFIX);
	else
		sprintf(rbd_dev->header_name, "%s%s",
			RBD_HEADER_PREFIX, spec->image_id);
	return 0;
}

static void rbd_dev_image_release(struct rbd_device *rbd_dev)
{
	rbd_dev_unprobe(rbd_dev);
	kfree(rbd_dev->header_name);
	rbd_dev->header_name = NULL;
	rbd_dev->image_format = 0;
	kfree(rbd_dev->spec->image_id);
	rbd_dev->spec->image_id = NULL;

	rbd_dev_destroy(rbd_dev);
}

/*
 * Probe for the existence of the header object for the given rbd
 * device.  If this image is the one being mapped (i.e., not a
 * parent), initiate a watch on its header object before using that
 * object to get detailed information about the rbd image.
 */
static int rbd_dev_image_probe(struct rbd_device *rbd_dev, bool mapping)
{
	int ret;
	int tmp;

	/*
	 * Get the id from the image id object.  Unless there's an
	 * error, rbd_dev->spec->image_id will be filled in with
	 * a dynamically-allocated string, and rbd_dev->image_format
	 * will be set to either 1 or 2.
	 */
	ret = rbd_dev_image_id(rbd_dev);
	if (ret)
		return ret;
	rbd_assert(rbd_dev->spec->image_id);
	rbd_assert(rbd_image_format_valid(rbd_dev->image_format));

	ret = rbd_dev_header_name(rbd_dev);
	if (ret)
		goto err_out_format;

	if (mapping) {
		ret = rbd_dev_header_watch_sync(rbd_dev, true);
		if (ret)
			goto out_header_name;
	}

	if (rbd_dev->image_format == 1)
		ret = rbd_dev_v1_header_info(rbd_dev);
	else
		ret = rbd_dev_v2_header_info(rbd_dev);
	if (ret)
		goto err_out_watch;

	ret = rbd_dev_spec_update(rbd_dev);
	if (ret)
		goto err_out_probe;

	ret = rbd_dev_probe_parent(rbd_dev);
	if (ret)
		goto err_out_probe;

	dout("discovered format %u image, header name is %s\n",
		rbd_dev->image_format, rbd_dev->header_name);

	return 0;
err_out_probe:
	rbd_dev_unprobe(rbd_dev);
err_out_watch:
	if (mapping) {
		tmp = rbd_dev_header_watch_sync(rbd_dev, false);
		if (tmp)
			rbd_warn(rbd_dev, "unable to tear down "
					"watch request (%d)\n", tmp);
	}
out_header_name:
	kfree(rbd_dev->header_name);
	rbd_dev->header_name = NULL;
err_out_format:
	rbd_dev->image_format = 0;
	kfree(rbd_dev->spec->image_id);
	rbd_dev->spec->image_id = NULL;

	dout("probe failed, returning %d\n", ret);

	return ret;
}

static ssize_t rbd_add(struct bus_type *bus,
		       const char *buf,
		       size_t count)
{
	struct rbd_device *rbd_dev = NULL;
	struct ceph_options *ceph_opts = NULL;
	struct rbd_options *rbd_opts = NULL;
	struct rbd_spec *spec = NULL;
	struct rbd_client *rbdc;
	struct ceph_osd_client *osdc;
	bool read_only;
	int rc = -ENOMEM;

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	/* parse add command */
	rc = rbd_add_parse_args(buf, &ceph_opts, &rbd_opts, &spec);
	if (rc < 0)
		goto err_out_module;
	read_only = rbd_opts->read_only;
	kfree(rbd_opts);
	rbd_opts = NULL;	/* done with this */

	rbdc = rbd_get_client(ceph_opts);
	if (IS_ERR(rbdc)) {
		rc = PTR_ERR(rbdc);
		goto err_out_args;
	}

	/* pick the pool */
	osdc = &rbdc->client->osdc;
	rc = ceph_pg_poolid_by_name(osdc->osdmap, spec->pool_name);
	if (rc < 0)
		goto err_out_client;
	spec->pool_id = (u64)rc;

	/* The ceph file layout needs to fit pool id in 32 bits */

	if (spec->pool_id > (u64)U32_MAX) {
		rbd_warn(NULL, "pool id too large (%llu > %u)\n",
				(unsigned long long)spec->pool_id, U32_MAX);
		rc = -EIO;
		goto err_out_client;
	}

	rbd_dev = rbd_dev_create(rbdc, spec);
	if (!rbd_dev)
		goto err_out_client;
	rbdc = NULL;		/* rbd_dev now owns this */
	spec = NULL;		/* rbd_dev now owns this */

	rc = rbd_dev_image_probe(rbd_dev, true);
	if (rc < 0)
		goto err_out_rbd_dev;

	/* If we are mapping a snapshot it must be marked read-only */

	if (rbd_dev->spec->snap_id != CEPH_NOSNAP)
		read_only = true;
	rbd_dev->mapping.read_only = read_only;

	rc = rbd_dev_device_setup(rbd_dev);
	if (rc) {
		rbd_dev_image_release(rbd_dev);
		goto err_out_module;
	}

	return count;

err_out_rbd_dev:
	rbd_dev_destroy(rbd_dev);
err_out_client:
	rbd_put_client(rbdc);
err_out_args:
	rbd_spec_put(spec);
err_out_module:
	module_put(THIS_MODULE);

	dout("Error adding device %s\n", buf);

	return (ssize_t)rc;
}

static struct rbd_device *__rbd_get_dev(unsigned long dev_id)
{
	struct list_head *tmp;
	struct rbd_device *rbd_dev;

	spin_lock(&rbd_dev_list_lock);
	list_for_each(tmp, &rbd_dev_list) {
		rbd_dev = list_entry(tmp, struct rbd_device, node);
		if (rbd_dev->dev_id == dev_id) {
			spin_unlock(&rbd_dev_list_lock);
			return rbd_dev;
		}
	}
	spin_unlock(&rbd_dev_list_lock);
	return NULL;
}

static void rbd_dev_device_release(struct device *dev)
{
	struct rbd_device *rbd_dev = dev_to_rbd_dev(dev);

	rbd_free_disk(rbd_dev);
	clear_bit(RBD_DEV_FLAG_EXISTS, &rbd_dev->flags);
	rbd_dev_mapping_clear(rbd_dev);
	unregister_blkdev(rbd_dev->major, rbd_dev->name);
	rbd_dev->major = 0;
	rbd_dev_id_put(rbd_dev);
	rbd_dev_mapping_clear(rbd_dev);
}

static void rbd_dev_remove_parent(struct rbd_device *rbd_dev)
{
	while (rbd_dev->parent) {
		struct rbd_device *first = rbd_dev;
		struct rbd_device *second = first->parent;
		struct rbd_device *third;

		/*
		 * Follow to the parent with no grandparent and
		 * remove it.
		 */
		while (second && (third = second->parent)) {
			first = second;
			second = third;
		}
		rbd_assert(second);
		rbd_dev_image_release(second);
		first->parent = NULL;
		first->parent_overlap = 0;

		rbd_assert(first->parent_spec);
		rbd_spec_put(first->parent_spec);
		first->parent_spec = NULL;
	}
}

static ssize_t rbd_remove(struct bus_type *bus,
			  const char *buf,
			  size_t count)
{
	struct rbd_device *rbd_dev = NULL;
	int target_id;
	unsigned long ul;
	int ret;

	ret = strict_strtoul(buf, 10, &ul);
	if (ret)
		return ret;

	/* convert to int; abort if we lost anything in the conversion */
	target_id = (int) ul;
	if (target_id != ul)
		return -EINVAL;

	mutex_lock_nested(&ctl_mutex, SINGLE_DEPTH_NESTING);

	rbd_dev = __rbd_get_dev(target_id);
	if (!rbd_dev) {
		ret = -ENOENT;
		goto done;
	}

	if (rbd_dev->open_count) {
		ret = -EBUSY;
		goto done;
	}

	spin_lock_irq(&rbd_dev->lock);
	if (rbd_dev->open_count)
		ret = -EBUSY;
	else
		set_bit(RBD_DEV_FLAG_REMOVING, &rbd_dev->flags);
	spin_unlock_irq(&rbd_dev->lock);
	if (ret < 0)
		goto done;
	rbd_bus_del_dev(rbd_dev);
	ret = rbd_dev_header_watch_sync(rbd_dev, false);
	if (ret)
		rbd_warn(rbd_dev, "failed to cancel watch event (%d)\n", ret);
	rbd_dev_image_release(rbd_dev);
	module_put(THIS_MODULE);
	ret = count;
done:
	mutex_unlock(&ctl_mutex);
	return ret;
}

/*
 * create control files in sysfs
 * /sys/bus/rbd/...
 */
static int rbd_sysfs_init(void)
{
	int ret;

	ret = device_register(&rbd_root_dev);
	if (ret < 0)
		return ret;

	ret = bus_register(&rbd_bus_type);
	if (ret < 0)
		device_unregister(&rbd_root_dev);

	return ret;
}

static void rbd_sysfs_cleanup(void)
{
	bus_unregister(&rbd_bus_type);
	device_unregister(&rbd_root_dev);
}

static int rbd_slab_init(void)
{
	rbd_assert(!rbd_img_request_cache);
	rbd_img_request_cache = kmem_cache_create("rbd_img_request",
					sizeof (struct rbd_img_request),
					__alignof__(struct rbd_img_request),
					0, NULL);
	if (!rbd_img_request_cache)
		return -ENOMEM;

	rbd_assert(!rbd_obj_request_cache);
	rbd_obj_request_cache = kmem_cache_create("rbd_obj_request",
					sizeof (struct rbd_obj_request),
					__alignof__(struct rbd_obj_request),
					0, NULL);
	if (!rbd_obj_request_cache)
		goto out_err;

	rbd_assert(!rbd_segment_name_cache);
	rbd_segment_name_cache = kmem_cache_create("rbd_segment_name",
					MAX_OBJ_NAME_SIZE + 1, 1, 0, NULL);
	if (rbd_segment_name_cache)
		return 0;
out_err:
	if (rbd_obj_request_cache) {
		kmem_cache_destroy(rbd_obj_request_cache);
		rbd_obj_request_cache = NULL;
	}

	kmem_cache_destroy(rbd_img_request_cache);
	rbd_img_request_cache = NULL;

	return -ENOMEM;
}

static void rbd_slab_exit(void)
{
	rbd_assert(rbd_segment_name_cache);
	kmem_cache_destroy(rbd_segment_name_cache);
	rbd_segment_name_cache = NULL;

	rbd_assert(rbd_obj_request_cache);
	kmem_cache_destroy(rbd_obj_request_cache);
	rbd_obj_request_cache = NULL;

	rbd_assert(rbd_img_request_cache);
	kmem_cache_destroy(rbd_img_request_cache);
	rbd_img_request_cache = NULL;
}

static int __init rbd_init(void)
{
	int rc;

	if (!libceph_compatible(NULL)) {
		rbd_warn(NULL, "libceph incompatibility (quitting)");

		return -EINVAL;
	}
	rc = rbd_slab_init();
	if (rc)
		return rc;
	rc = rbd_sysfs_init();
	if (rc)
		rbd_slab_exit();
	else
		pr_info("loaded " RBD_DRV_NAME_LONG "\n");

	return rc;
}

static void __exit rbd_exit(void)
{
	rbd_sysfs_cleanup();
	rbd_slab_exit();
}

module_init(rbd_init);
module_exit(rbd_exit);

MODULE_AUTHOR("Sage Weil <sage@newdream.net>");
MODULE_AUTHOR("Yehuda Sadeh <yehuda@hq.newdream.net>");
MODULE_DESCRIPTION("rados block device");

/* following authorship retained from original osdblk.c */
MODULE_AUTHOR("Jeff Garzik <jeff@garzik.org>");

MODULE_LICENSE("GPL");
