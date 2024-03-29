/*
 * Copyright (C) 2007-2008 Advanced Micro Devices, Inc.
 * Author: Joerg Roedel <joerg.roedel@amd.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#define pr_fmt(fmt)    "%s: " fmt, __func__

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/bug.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/iommu.h>
#include <linux/scatterlist.h>

static ssize_t show_iommu_group(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned int groupid;

	if (iommu_device_group(dev, &groupid))
		return 0;

	return sprintf(buf, "%u", groupid);
}
static DEVICE_ATTR(iommu_group, S_IRUGO, show_iommu_group, NULL);

static int add_iommu_group(struct device *dev, void *data)
{
	unsigned int groupid;

	if (iommu_device_group(dev, &groupid) == 0)
		return device_create_file(dev, &dev_attr_iommu_group);

	return 0;
}

static int remove_iommu_group(struct device *dev)
{
	unsigned int groupid;

	if (iommu_device_group(dev, &groupid) == 0)
		device_remove_file(dev, &dev_attr_iommu_group);

	return 0;
}

static int iommu_device_notifier(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	struct device *dev = data;

	if (action == BUS_NOTIFY_ADD_DEVICE)
		return add_iommu_group(dev, NULL);
	else if (action == BUS_NOTIFY_DEL_DEVICE)
		return remove_iommu_group(dev);

	return 0;
}

static struct notifier_block iommu_device_nb = {
	.notifier_call = iommu_device_notifier,
};

static void iommu_bus_init(struct bus_type *bus, struct iommu_ops *ops)
{
	bus_register_notifier(bus, &iommu_device_nb);
	bus_for_each_dev(bus, NULL, NULL, add_iommu_group);
}

/**
 * bus_set_iommu - set iommu-callbacks for the bus
 * @bus: bus.
 * @ops: the callbacks provided by the iommu-driver
 *
 * This function is called by an iommu driver to set the iommu methods
 * used for a particular bus. Drivers for devices on that bus can use
 * the iommu-api after these ops are registered.
 * This special function is needed because IOMMUs are usually devices on
 * the bus itself, so the iommu drivers are not initialized when the bus
 * is set up. With this function the iommu-driver can set the iommu-ops
 * afterwards.
 */
int bus_set_iommu(struct bus_type *bus, struct iommu_ops *ops)
{
	if (bus->iommu_ops != NULL)
		return -EBUSY;

	bus->iommu_ops = ops;

	/* Do IOMMU specific setup for this bus-type */
	iommu_bus_init(bus, ops);

	return 0;
}
EXPORT_SYMBOL_GPL(bus_set_iommu);

bool iommu_present(struct bus_type *bus)
{
	return bus->iommu_ops != NULL;
}
EXPORT_SYMBOL_GPL(iommu_present);

struct iommu_group *iommu_group_get_by_id(int id)
{
	struct kobject *group_kobj;
	struct iommu_group *group;
	const char *name;

	if (!iommu_group_kset)
		return NULL;

	name = kasprintf(GFP_KERNEL, "%d", id);
	if (!name)
		return NULL;

	group_kobj = kset_find_obj(iommu_group_kset, name);
	kfree(name);

	if (!group_kobj)
		return NULL;

	group = container_of(group_kobj, struct iommu_group, kobj);
	BUG_ON(group->id != id);

	kobject_get(group->devices_kobj);
	kobject_put(&group->kobj);

	return group;
}
EXPORT_SYMBOL_GPL(iommu_group_get_by_id);

/**
 * iommu_set_fault_handler() - set a fault handler for an iommu domain
 * @domain: iommu domain
 * @handler: fault handler
 * @token: user data, will be passed back to the fault handler
 *
 * This function should be used by IOMMU users which want to be notified
 * whenever an IOMMU fault happens.
 *
 * The fault handler itself should return 0 on success, and an appropriate
 * error code otherwise.
 */
void iommu_set_fault_handler(struct iommu_domain *domain,
					iommu_fault_handler_t handler,
					void *token)
{
	BUG_ON(!domain);

	domain->handler = handler;
	domain->handler_token = token;
}
EXPORT_SYMBOL_GPL(iommu_set_fault_handler);

struct iommu_domain *iommu_domain_alloc(struct bus_type *bus, int flags)
{
	struct iommu_domain *domain;
	int ret;

	if (bus == NULL || bus->iommu_ops == NULL)
		return NULL;

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain)
		return NULL;

	domain->ops = bus->iommu_ops;

	ret = domain->ops->domain_init(domain, flags);
	if (ret)
		goto out_free;

	return domain;

out_free:
	kfree(domain);

	return NULL;
}
EXPORT_SYMBOL_GPL(iommu_domain_alloc);

void iommu_domain_free(struct iommu_domain *domain)
{
	if (likely(domain->ops->domain_destroy != NULL))
		domain->ops->domain_destroy(domain);

	kfree(domain);
}
EXPORT_SYMBOL_GPL(iommu_domain_free);

int iommu_attach_device(struct iommu_domain *domain, struct device *dev)
{
	if (unlikely(domain->ops->attach_dev == NULL))
		return -ENODEV;

	return domain->ops->attach_dev(domain, dev);
}
EXPORT_SYMBOL_GPL(iommu_attach_device);

void iommu_detach_device(struct iommu_domain *domain, struct device *dev)
{
	if (unlikely(domain->ops->detach_dev == NULL))
		return;

	domain->ops->detach_dev(domain, dev);
}
EXPORT_SYMBOL_GPL(iommu_detach_device);

/*
 * IOMMU groups are really the natrual working unit of the IOMMU, but
 * the IOMMU API works on domains and devices.  Bridge that gap by
 * iterating over the devices in a group.  Ideally we'd have a single
 * device which represents the requestor ID of the group, but we also
 * allow IOMMU drivers to create policy defined minimum sets, where
 * the physical hardware may be able to distiguish members, but we
 * wish to group them at a higher level (ex. untrusted multi-function
 * PCI devices).  Thus we attach each device.
 */
static int iommu_group_do_attach_device(struct device *dev, void *data)
{
	struct iommu_domain *domain = data;

	return iommu_attach_device(domain, dev);
}

int iommu_attach_group(struct iommu_domain *domain, struct iommu_group *group)
{
	return iommu_group_for_each_dev(group, domain,
					iommu_group_do_attach_device);
}
EXPORT_SYMBOL_GPL(iommu_attach_group);

static int iommu_group_do_detach_device(struct device *dev, void *data)
{
	struct iommu_domain *domain = data;

	iommu_detach_device(domain, dev);

	return 0;
}

void iommu_detach_group(struct iommu_domain *domain, struct iommu_group *group)
{
	iommu_group_for_each_dev(group, domain, iommu_group_do_detach_device);
}
EXPORT_SYMBOL_GPL(iommu_detach_group);

phys_addr_t iommu_iova_to_phys(struct iommu_domain *domain, dma_addr_t iova)
{
	if (unlikely(domain->ops->iova_to_phys == NULL))
		return 0;

	return domain->ops->iova_to_phys(domain, iova);
}
EXPORT_SYMBOL_GPL(iommu_iova_to_phys);

int iommu_domain_has_cap(struct iommu_domain *domain,
			 unsigned long cap)
{
	if (unlikely(domain->ops->domain_has_cap == NULL))
		return 0;

	return domain->ops->domain_has_cap(domain, cap);
}
EXPORT_SYMBOL_GPL(iommu_domain_has_cap);

static size_t iommu_pgsize(struct iommu_domain *domain,
			   unsigned long addr_merge, size_t size)
{
	unsigned int pgsize_idx;
	size_t pgsize;

	/* Max page size that still fits into 'size' */
	pgsize_idx = __fls(size);

	/* need to consider alignment requirements ? */
	if (likely(addr_merge)) {
		/* Max page size allowed by address */
		unsigned int align_pgsize_idx = __ffs(addr_merge);
		pgsize_idx = min(pgsize_idx, align_pgsize_idx);
	}

	/* build a mask of acceptable page sizes */
	pgsize = (1UL << (pgsize_idx + 1)) - 1;

	/* throw away page sizes not supported by the hardware */
	pgsize &= domain->ops->pgsize_bitmap;

	/* make sure we're still sane */
	BUG_ON(!pgsize);

	/* pick the biggest page */
	pgsize_idx = __fls(pgsize);
	pgsize = 1UL << pgsize_idx;

	return pgsize;
}

int iommu_map(struct iommu_domain *domain, unsigned long iova,
	      phys_addr_t paddr, size_t size, int prot)
{
	unsigned long orig_iova = iova;
	unsigned int min_pagesz;
	size_t orig_size = size;
	int ret = 0;

	if (unlikely(domain->ops->unmap == NULL ||
		     domain->ops->pgsize_bitmap == 0UL))
		return -ENODEV;

	/* find out the minimum page size supported */
	min_pagesz = 1 << __ffs(domain->ops->pgsize_bitmap);

	/*
	 * both the virtual address and the physical one, as well as
	 * the size of the mapping, must be aligned (at least) to the
	 * size of the smallest page supported by the hardware
	 */
	if (!IS_ALIGNED(iova | paddr | size, min_pagesz)) {
		pr_err("unaligned: iova 0x%lx pa 0x%pa size 0x%zx min_pagesz 0x%x\n",
		       iova, &paddr, size, min_pagesz);
		return -EINVAL;
	}

	pr_debug("map: iova 0x%lx pa 0x%pa size 0x%zx\n", iova, &paddr, size);

	while (size) {
		size_t pgsize = iommu_pgsize(domain, iova | paddr, size);

		pr_debug("mapping: iova 0x%lx pa 0x%pa pgsize 0x%zx\n",
			 iova, &paddr, pgsize);

		ret = domain->ops->map(domain, iova, paddr, pgsize, prot);
		if (ret)
			break;

		iova += pgsize;
		paddr += pgsize;
		size -= pgsize;
	}

	/* unroll mapping in case something went wrong */
	if (ret)
		iommu_unmap(domain, orig_iova, orig_size - size);

	return ret;
}
EXPORT_SYMBOL_GPL(iommu_map);

size_t iommu_unmap(struct iommu_domain *domain, unsigned long iova, size_t size)
{
	size_t unmapped_page, unmapped = 0;
	unsigned int min_pagesz;

	if (unlikely(domain->ops->unmap == NULL ||
		     domain->ops->pgsize_bitmap == 0UL))
		return -ENODEV;

	/* find out the minimum page size supported */
	min_pagesz = 1 << __ffs(domain->ops->pgsize_bitmap);

	/*
	 * The virtual address, as well as the size of the mapping, must be
	 * aligned (at least) to the size of the smallest page supported
	 * by the hardware
	 */
	if (!IS_ALIGNED(iova | size, min_pagesz)) {
		pr_err("unaligned: iova 0x%lx size 0x%zx min_pagesz 0x%x\n",
		       iova, size, min_pagesz);
		return -EINVAL;
	}

	pr_debug("unmap this: iova 0x%lx size 0x%zx\n", iova, size);

	/*
	 * Keep iterating until we either unmap 'size' bytes (or more)
	 * or we hit an area that isn't mapped.
	 */
	while (unmapped < size) {
		size_t pgsize = iommu_pgsize(domain, iova, size - unmapped);

		unmapped_page = domain->ops->unmap(domain, iova, pgsize);
		if (!unmapped_page)
			break;

		pr_debug("unmapped: iova 0x%lx size 0x%zx\n",
			 iova, unmapped_page);

		iova += unmapped_page;
		unmapped += unmapped_page;
	}

	return unmapped;
}
EXPORT_SYMBOL_GPL(iommu_unmap);

int iommu_map_range(struct iommu_domain *domain, unsigned int iova,
		    struct scatterlist *sg, unsigned int len, int prot)
{
	if (unlikely(domain->ops->map_range == NULL))
		return -ENODEV;

	BUG_ON(iova & (~PAGE_MASK));

	return domain->ops->map_range(domain, iova, sg, len, prot);
}
EXPORT_SYMBOL_GPL(iommu_map_range);

int iommu_unmap_range(struct iommu_domain *domain, unsigned int iova,
		      unsigned int len)
{
	if (unlikely(domain->ops->unmap_range == NULL))
		return -ENODEV;

	BUG_ON(iova & (~PAGE_MASK));

	return domain->ops->unmap_range(domain, iova, len);
}
EXPORT_SYMBOL_GPL(iommu_unmap_range);

phys_addr_t iommu_get_pt_base_addr(struct iommu_domain *domain)
{
	if (unlikely(domain->ops->get_pt_base_addr == NULL))
		return 0;

	return domain->ops->get_pt_base_addr(domain);
}
EXPORT_SYMBOL_GPL(iommu_get_pt_base_addr);

static int __init iommu_init(void)
{
	iommu_group_kset = kset_create_and_add("iommu_groups",
					       NULL, kernel_kobj);
	ida_init(&iommu_group_ida);
	mutex_init(&iommu_group_mutex);

	BUG_ON(!iommu_group_kset);

	return 0;
}
subsys_initcall(iommu_init);

int iommu_domain_get_attr(struct iommu_domain *domain,
			  enum iommu_attr attr, void *data)
{
	struct iommu_domain_geometry *geometry;
	int ret = 0;

	switch (attr) {
	case DOMAIN_ATTR_GEOMETRY:
		geometry  = data;
		*geometry = domain->geometry;

		break;
	default:
		if (!domain->ops->domain_get_attr)
			return -EINVAL;

		ret = domain->ops->domain_get_attr(domain, attr, data);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(iommu_domain_get_attr);

int iommu_domain_set_attr(struct iommu_domain *domain,
			  enum iommu_attr attr, void *data)
{
	if (!domain->ops->domain_set_attr)
		return -EINVAL;

	return domain->ops->domain_set_attr(domain, attr, data);
}
EXPORT_SYMBOL_GPL(iommu_domain_set_attr);
