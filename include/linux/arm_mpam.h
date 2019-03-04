// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018 Arm Ltd.

#ifndef __LINUX_ARM_MPAM_H
#define __LINUX_ARM_MPAM_H

#include <linux/acpi.h> // TODO: kill me
#include <linux/cacheinfo.h>
#include <linux/cpumask.h>
#include <linux/resctrl_types.h>
#include <linux/topology.h>
#include <linux/types.h>

#include <asm/acpi.h> // TODO: kill me

/* Bits for irq:flags, must match the ACPI definition */
#define MPAM_IRQ_MODE_LEVEL    0x1
#define MPAM_IRQ_FLAGS_MASK    0x7f

struct mpam_device;

/*
 * mpam_devices discovered from firmware tables should be created between
 * a mpam_discovery_start() and mpam_discovery_{failed,complete}() call.
 * This lets the driver group the  calls it has to schedule on remote CPUs.
 */

/*
 * Call before creating any devices to allocate internal structures.
 */
int mpam_discovery_start(void);


enum mpam_class_types {
	MPAM_CLASS_CACHE,	/* Well known caches, e.g. L2 */
	MPAM_CLASS_MEMORY,	/* Main memory */
	MPAM_CLASS_UNKNOWN,	/* Everything else, e.g. TLBs etc */
};

struct mpam_device * __init
__mpam_device_create(u8 level_idx, enum mpam_class_types type,
		     int component_id, const struct cpumask *fw_affinity,
		     phys_addr_t hwpage_address);

/*
 * Create a device for a well known cache, e.g. L2.
 * @level_idx and @cache_id will be used to match the cache via cacheinfo
 * to learn the component affinity and export domain/resources via resctrl.
 * If the device can only be accessed from a smaller set of CPUs, provide
 * this as @device_affinity, which can otherwise be NULL.
 *
 * Returns the new device, or an ERR_PTR().
 */
static inline __init struct mpam_device *
mpam_device_create_cache(u8 level_idx, int cache_id,
			 const struct cpumask *device_affinity,
			 phys_addr_t hwpage_address)
{
	return __mpam_device_create(level_idx, MPAM_CLASS_CACHE, cache_id,
				    device_affinity, hwpage_address);
}

/*
 * Create a device for a main memory.
 * For NUMA systems @nid allows multiple components to be created,
 * which will be exported as resctrl domains. MSCs for memory must
 * be accessible from any cpu.
 */
static inline __init struct mpam_device *
mpam_device_create_memory(int nid, phys_addr_t hwpage_address)
{
	int cpu;
	u32 swizzled_hwid;
	u64 min_hwid = INVALID_HWID;
	const struct cpumask *nid_affinity = cpumask_of_node(nid);

	/*
	 * We can't use the nid as the component id, as it may never
	 * match a cache, and the topology may be inside-out. Instead,
	 * find the cpu in nid_affinity with the smallest MPIDR, and use
	 * that. This matches how cacheinfo generates ids on arm64.
	 * TODO: share that code by making it less ACPI specific.
	 */
	for_each_cpu(cpu, nid_affinity) {
		if (min_hwid > cpu_logical_map(cpu))
			min_hwid = cpu_logical_map(cpu);
	}
	WARN_ON_ONCE(min_hwid == INVALID_HWID);
	swizzled_hwid = ARCH_PHYSID_TO_U32(min_hwid);

	return __mpam_device_create(~0, MPAM_CLASS_MEMORY, swizzled_hwid,
				    nid_affinity, hwpage_address);
}

/*
 * Create a device of an unknown type.
 * Unknown devices will contribute to the platform partid/pmg limit,
 * but will only ever be configured unrestricted for in-use partids.
 * These are never exported via resctrl.
 */
static inline __init struct mpam_device *
mpam_device_create(u8 level_idx, int component_id,
		   const struct cpumask *affinity,
		   phys_addr_t hwpage_address)
{
	return __mpam_device_create(level_idx, MPAM_CLASS_UNKNOWN,
				    component_id, affinity, hwpage_address);
}


/* Set the interrupt properties from the firmware table */
void mpam_device_set_error_irq(struct mpam_device *dev, u32 irq, u32 flags);
void mpam_device_set_overflow_irq(struct mpam_device *dev, u32 irq, u32 flags);

/*
 * We've given up on discovering mpam, unregister, free and release everything.
 * Called instead of mpam_discovery_complete().
 */
void mpam_discovery_failed(void);

/* Reset every device, configuring every partid unrestricted */
void mpam_reset_devices(void);

/*
 * All devices have been created, build all the internal lists and try and
 * probe any missing devices. Enabling MPAM for use will be scheduled once
 * all devices have been probed.
 */
void mpam_discovery_complete(void);

/* Do we export anything 'alloc_capable' or 'mon_capable' via resctrl? */
bool mpam_resctrl_alloc_capable(void);
bool mpam_resctrl_mon_capable(void);

u32 mpam_resctrl_num_closid(void);
u32 mpam_resctrl_num_rmid(void);

/* Get the specific resctrl resource */
struct rdt_resource *mpam_resctrl_get_resource(enum resctrl_resource_level l);

/* reset cached configurations, then all devices */
void resctrl_arch_reset_resources(void);

#endif /* __LINUX_ARM_MPAM_H */
