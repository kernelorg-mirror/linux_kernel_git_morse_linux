// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2025 Arm Ltd.

#define pr_fmt(fmt) "%s:%s: " fmt, KBUILD_MODNAME, __func__

#include <linux/acpi.h>
#include <linux/arm_mpam.h>
#include <linux/cacheinfo.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gfp.h>
#include <linux/list.h>
#include <linux/lockdep.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/srcu.h>
#include <linux/types.h>

#include "mpam_internal.h"

/*
 * mpam_list_lock protects the SRCU lists when writing. Once the
 * mpam_enabled key is enabled these lists are read-only,
 * unless the error interrupt disables the driver.
 */
static DEFINE_MUTEX(mpam_list_lock);
static LIST_HEAD(mpam_all_msc);

static struct srcu_struct mpam_srcu;

/*
 * Number of MSCs that have been probed. Once all MSC have been probed MPAM
 * can be enabled.
 */
static atomic_t mpam_num_msc;

static void mpam_discovery_complete(void)
{
	pr_err("Discovered all MSC\n");
}

static int mpam_dt_count_msc(void)
{
	int count = 0;
	struct device_node *np;

	for_each_compatible_node(np, NULL, "arm,mpam-msc") {
		if (of_device_is_available(np))
			count++;
	}

	return count;
}

static int mpam_dt_parse_resource(struct mpam_msc *msc, struct device_node *np,
				  u32 ris_idx)
{
	int err = 0;
	u32 level = 0;
	unsigned long cache_id;
	struct device *dev = &msc->pdev->dev;
	struct device_node *cache __free(device_node) = NULL;
	struct device_node *parent __free(device_node) = of_get_parent(np);

	if (of_device_is_compatible(np, "arm,mpam-cache")) {
		cache = of_parse_phandle(np, "arm,mpam-device", 0);
		if (!cache) {
			dev_err_once(dev, "Failed to read phandle\n");
			return -EINVAL;
		}
	} else if (of_device_is_compatible(parent, "cache")) {
		cache = parent;
	} else {
		/* For now, only caches are supported */
		cache = NULL;
		return err;
	}

	err = of_property_read_u32(cache, "cache-level", &level);
	if (err) {
		dev_err_once(dev, "Failed to read cache-level\n");
		return err;
	}

	cache_id = cache_of_calculate_id(cache);
	if (cache_id == ~0) {
		dev_err_once(dev, "Failed to calculate cache-id\n");
		return -ENOENT;
	}

	return mpam_ris_create(msc, ris_idx, MPAM_CLASS_CACHE, level, cache_id);
}

static int mpam_dt_parse_resources(struct mpam_msc *msc, void *ignored)
{
	u64 ris_idx = 0;
	int err, num_ris = 0;
	struct device_node *np;

	np = msc->pdev->dev.of_node;
	for_each_available_child_of_node_scoped(np, iter) {
		err = of_property_read_reg(iter, 0, &ris_idx, NULL);
		if (!err) {
			num_ris++;
			err = mpam_dt_parse_resource(msc, iter, ris_idx);
			if (err)
				return err;
		}
	}

	if (!num_ris)
		mpam_dt_parse_resource(msc, np, 0);

	return err;
}

/*
 * An MSC can control traffic from a set of CPUs, but may only be accessible
 * from a (hopefully wider) set of CPUs. The common reason for this is power
 * management. If all the CPUs in a cluster are in PSCI:CPU_SUSPEND, the
 * corresponding cache may also be powered off. By making accesses from
 * one of those CPUs, we ensure this isn't the case.
 */
static int update_msc_accessibility(struct mpam_msc *msc)
{
	struct device *dev = &msc->pdev->dev;
	struct device_node *parent;
	u32 affinity_id;
	int err;

	if (!acpi_disabled) {
		err = device_property_read_u32(&msc->pdev->dev, "cpu_affinity",
					       &affinity_id);
		if (err)
			cpumask_copy(&msc->accessibility, cpu_possible_mask);
		else
			acpi_pptt_get_cpus_from_container(affinity_id,
							  &msc->accessibility);

		return 0;
	}

	/* Where an MSC can be accessed from depends on the path to of_node. */
	parent = of_get_parent(msc->pdev->dev.of_node);
	if (parent == of_root) {
		cpumask_copy(&msc->accessibility, cpu_possible_mask);
		err = 0;
	} else {
		err = -EINVAL;
		dev_err_once(dev, "Cannot determine accessibility of MSC.\n");
	}
	of_node_put(parent);

	return err;
}

static int fw_num_msc;

static void mpam_msc_drv_remove(struct platform_device *pdev)
{
	struct mpam_msc *msc = platform_get_drvdata(pdev);

	if (!msc)
		return;

	mutex_lock(&mpam_list_lock);
	platform_set_drvdata(pdev, NULL);
	list_del_rcu(&msc->all_msc_list);
	synchronize_srcu(&mpam_srcu);
	mutex_unlock(&mpam_list_lock);
}

static int mpam_msc_drv_probe(struct platform_device *pdev)
{
	int err;
	struct mpam_msc *msc;
	struct resource *msc_res;
	struct device *dev = &pdev->dev;
	void *plat_data = pdev->dev.platform_data;

	mutex_lock(&mpam_list_lock);
	do {
		msc = devm_kzalloc(&pdev->dev, sizeof(*msc), GFP_KERNEL);
		if (!msc) {
			err = -ENOMEM;
			break;
		}

		mutex_init(&msc->probe_lock);
		mutex_init(&msc->part_sel_lock);
		mutex_init(&msc->outer_mon_sel_lock);
		raw_spin_lock_init(&msc->inner_mon_sel_lock);
		msc->id = pdev->id;
		msc->pdev = pdev;
		INIT_LIST_HEAD_RCU(&msc->all_msc_list);
		INIT_LIST_HEAD_RCU(&msc->ris);

		err = update_msc_accessibility(msc);
		if (err)
			break;
		if (cpumask_empty(&msc->accessibility)) {
			dev_err_once(dev, "MSC is not accessible from any CPU!");
			err = -EINVAL;
			break;
		}

		if (device_property_read_u32(&pdev->dev, "pcc-channel",
					     &msc->pcc_subspace_id))
			msc->iface = MPAM_IFACE_MMIO;
		else
			msc->iface = MPAM_IFACE_PCC;

		if (msc->iface == MPAM_IFACE_MMIO) {
			void __iomem *io;

			io = devm_platform_get_and_ioremap_resource(pdev, 0,
								    &msc_res);
			if (IS_ERR(io)) {
				dev_err_once(dev, "Failed to map MSC base address\n");
				err = PTR_ERR(io);
				break;
			}
			msc->mapped_hwpage_sz = msc_res->end - msc_res->start;
			msc->mapped_hwpage = io;
		}

		list_add_rcu(&msc->all_msc_list, &mpam_all_msc);
		platform_set_drvdata(pdev, msc);
	} while (0);
	mutex_unlock(&mpam_list_lock);

	if (!err) {
		/* Create RIS entries described by firmware */
		if (!acpi_disabled)
			err = acpi_mpam_parse_resources(msc, plat_data);
		else
			err = mpam_dt_parse_resources(msc, plat_data);
	}

	if (err && msc)
		mpam_msc_drv_remove(pdev);

	if (!err && atomic_add_return(1, &mpam_num_msc) == fw_num_msc)
		mpam_discovery_complete();

	return err;
}

static const struct of_device_id mpam_of_match[] = {
	{ .compatible = "arm,mpam-msc", },
	{},
};
MODULE_DEVICE_TABLE(of, mpam_of_match);

static struct platform_driver mpam_msc_driver = {
	.driver = {
		.name = "mpam_msc",
		.of_match_table = of_match_ptr(mpam_of_match),
	},
	.probe = mpam_msc_drv_probe,
	.remove = mpam_msc_drv_remove,
};

/*
 * MSC that are hidden under caches are not created as platform devices
 * as there is no cache driver. Caches are also special-cased in
 * update_msc_accessibility().
 */
static void mpam_dt_create_foundling_msc(void)
{
	struct platform_device *pdev;
	struct device_node *cache;

	for_each_compatible_node(cache, NULL, "cache") {
		struct device_node *cache_device;

		if (of_node_check_flag(cache, OF_POPULATED))
			continue;

		cache_device = of_find_matching_node_and_match(cache, mpam_of_match, NULL);
		if (!cache_device)
			continue;
		of_node_put(cache_device);

		pdev = of_platform_device_create(cache, "cache", NULL);
		if (!pdev)
			pr_err_once("Failed to create MSC devices under caches\n");
	}
}

static int __init mpam_msc_driver_init(void)
{
	if (!system_supports_mpam())
		return -EOPNOTSUPP;

	init_srcu_struct(&mpam_srcu);

	if (!acpi_disabled)
		fw_num_msc = acpi_mpam_count_msc();
	else
		fw_num_msc = mpam_dt_count_msc();

	if (fw_num_msc <= 0) {
		pr_err("No MSC devices found in firmware\n");
		return -EINVAL;
	}

	if (acpi_disabled)
		mpam_dt_create_foundling_msc();

	return platform_driver_register(&mpam_msc_driver);
}
subsys_initcall(mpam_msc_driver_init);
