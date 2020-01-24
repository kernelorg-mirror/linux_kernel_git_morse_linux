// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2020 Arm Ltd.

#define pr_fmt(fmt) "mpam: " fmt

#include <linux/arm_mpam.h>
#include <linux/cacheinfo.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/debugfs.h>
#include <linux/list.h>
#include <linux/resctrl.h>
#include <linux/types.h>

#include <asm/mpam.h>

#include "mpam_internal.h"

static bool _true = 1;
struct dentry *debugfs_dir;

static void create_feature_with_attribute(struct dentry *parent,
					  struct mpam_class *class,
					  const char *feature_name,
					  const char *attribute_name,
					  u16 *attribute_val)
{
	struct dentry *feature;;

	feature = debugfs_create_dir(feature_name, parent);
	if (!feature)
		return;

	debugfs_create_u16(attribute_name, 0444, feature, attribute_val);
}

static const char *get_class_format(enum mpam_class_types t)
{
	switch (t) {
	case MPAM_CLASS_CACHE:
		return "cache_L";
	case MPAM_CLASS_MEMORY:
		return "mem-";
	default:
		return "unknown-";
	}
}

static int cpu_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "%*pbl\n", cpumask_pr_args(to_cpumask(s->private)));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(cpu);

void mpam_debugfs_populate_class(struct mpam_class *class, struct dentry *dc)
{

	if (mpam_has_feature(mpam_feat_ccap_part, class->features))
		debugfs_create_bool("ccap_part", 0444, dc, &_true);
	if (mpam_has_feature(mpam_feat_cpor_part, class->features))
		create_feature_with_attribute(dc, class, "cpor_part",
					      "cpbm_wd", &class->cpbm_wd);

	if (mpam_has_feature(mpam_feat_mbw_part, class->features))
		create_feature_with_attribute(dc, class, "mbw_part",
					      "mbw_pbm_bits",
					      &class->mbw_pbm_bits);

	/* TODO: are these three part of mbw_part? */
	if (mpam_has_feature(mpam_feat_mbw_min, class->features))
		debugfs_create_bool("mbw_min", 0444, dc, &_true);
	if (mpam_has_feature(mpam_feat_mbw_max, class->features))
		debugfs_create_bool("mbw_max", 0444, dc, &_true);
	if (mpam_has_feature(mpam_feat_mbw_prop, class->features))
		debugfs_create_bool("mbw_prop", 0444, dc, &_true);

	if (mpam_has_feature(mpam_feat_intpri_part, class->features))
		create_feature_with_attribute(dc, class, "intpri_part",
					      "intpri_wd",
					      &class->intpri_wd);
	if (mpam_has_feature(mpam_feat_dspri_part, class->features))
		create_feature_with_attribute(dc, class, "dspri_part",
					      "dspri_wd",
					      &class->dspri_wd);
	if (mpam_has_feature(mpam_feat_msmon_csu, class->features))
		create_feature_with_attribute(dc, class, "msmon_csu",
					      "num_csu_mon",
					      &class->num_csu_mon);
	if (mpam_has_feature(mpam_feat_msmon_mbwu, class->features))
		create_feature_with_attribute(dc, class, "msmon_mbwu",
					      "num_mbwu_mon",
					      &class->num_mbwu_mon);
	if (mpam_has_feature(mpam_feat_msmon_capt, class->features))
		debugfs_create_bool("msmon_capt", 0444, dc, &_true);

	/* TODO: what was bwa_wd again? */

	debugfs_create_file("cpus", 0444, dc, &class->fw_affinity, &cpu_fops);
}

void mpam_debugfs_init(void)
{
	char *class_name;
	struct mpam_class *class;
	struct dentry *debugfs_classes;

	mpam_class_list_lock_held(); /* tis a mutex */

	debugfs_dir = debugfs_create_dir("mpam", NULL);
	debugfs_classes = debugfs_create_dir("classes", debugfs_dir);

	list_for_each_entry(class, &mpam_classes, classes_list) {
		struct dentry *dc;

		class_name = kasprintf(GFP_KERNEL, "%s%u",
					get_class_format(class->type),
					class->level);
		if (!class_name) {
			/* TODO: fini */
			break;
		}

		dc = debugfs_create_dir(class_name, debugfs_classes);
		if (!dc) {
			/* TODO: fini */
			break;
		}

		mpam_debugfs_populate_class(class, dc);
	}
}
