// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2024 Arm Ltd.

#define pr_fmt(fmt) "mpam: resctrl: " fmt

#include <linux/arm_mpam.h>
#include <linux/cacheinfo.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/errno.h>
#include <linux/limits.h>
#include <linux/list.h>
#include <linux/printk.h>
#include <linux/rculist.h>
#include <linux/resctrl.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wait.h>

#include <asm/mpam.h>

#include "mpam_internal.h"

DECLARE_WAIT_QUEUE_HEAD(resctrl_mon_ctx_waiters);

/*
 * The classes we've picked to map to resctrl resources, wrapped
 * in with their resctrl structure.
 * Class pointer may be NULL.
 */
static struct mpam_resctrl_res mpam_resctrl_controls[RDT_NUM_RESOURCES];

/*
 * The classes we've picked to map to resctrl events.
 * Resctrl believes all the worlds a Xeon, and these are all on the L3. This
 * array lets us find the actual class backing the event counters. e.g.
 * the only memory bandwidth counters may be on the memory controller, but to
 * make use of them, we pretend they are on L3.
 * Class pointer may be NULL.
 */
static struct mpam_class *mpam_resctrl_counters[QOS_NUM_EVENTS];

static bool exposed_alloc_capable;
static bool exposed_mon_capable;

/*
 * MPAM emulates CDP by setting different PARTID in the I/D fields of MPAM1_EL1.
 * This applies globally to all traffic the CPU generates.
 */
static bool cdp_enabled;

/*
 * If resctrl_init() succeeded, resctrl_exit() can be used to remove support
 * for the filesystem in the event of an error.
 */
static bool resctrl_enabled;

/*
 * mpam_resctrl_pick_caches() needs to know the size of the caches. cacheinfo
 * populates this from a device_initcall(). mpam_resctrl_setup() must wait.
 */
static bool cacheinfo_ready;
static DECLARE_WAIT_QUEUE_HEAD(wait_cacheinfo_ready);

/* A dummy mon context to use when the monitors were allocated up front */
u32 __mon_is_rmid_idx = USE_RMID_IDX;
void *mon_is_rmid_idx = &__mon_is_rmid_idx;
u32 __mon_is_assigned = USE_ABMC_ASSIGNED;
void *mon_is_assigned = &__mon_is_assigned;

/* Lock to protect the abmc structures */
static DEFINE_MUTEX(abmc_lock);

bool resctrl_arch_alloc_capable(void)
{
	return exposed_alloc_capable;
}

bool resctrl_arch_mon_capable(void)
{
	return exposed_mon_capable;
}

bool resctrl_arch_is_llc_occupancy_enabled(void)
{
	return mpam_resctrl_counters[QOS_L3_OCCUP_EVENT_ID];
}

bool resctrl_arch_is_mbm_local_enabled(void)
{
	return mpam_resctrl_counters[QOS_L3_MBM_LOCAL_EVENT_ID];
}

bool resctrl_arch_is_mbm_total_enabled(void)
{
	return mpam_resctrl_counters[QOS_L3_MBM_TOTAL_EVENT_ID];
}

bool resctrl_arch_get_cdp_enabled(enum resctrl_res_level rid)
{
	switch (rid) {
	case RDT_RESOURCE_L2:
	case RDT_RESOURCE_L3:
		return cdp_enabled;
	case RDT_RESOURCE_MBA:
	default:
		/*
		 * x86's MBA control doesn't support CDP, so user-space doesn't
		 * expect it.
		 */
		return false;
	}
}

/**
 * resctrl_reset_task_closids() - Reset the PARTID/PMG values for all tasks.
 *
 * At boot, all existing tasks use partid zero for D and I.
 * To enable/disable CDP emulation, all these tasks need relabelling.
 */
static void resctrl_reset_task_closids(void)
{
	struct task_struct *p, *t;

	read_lock(&tasklist_lock);
	for_each_process_thread(p, t) {
		resctrl_arch_set_closid_rmid(t, RESCTRL_RESERVED_CLOSID,
					     RESCTRL_RESERVED_RMID);
	}
	read_unlock(&tasklist_lock);
}

static void mpam_resctrl_update_mbm_cntrs(void)
{
	struct mpam_resctrl_res *res = &mpam_resctrl_controls[RDT_RESOURCE_L3];
	struct rdt_resource *l3 = &res->resctrl_res;
	u8 mon_per_partid = 1;

	if (!res->class || !res->mpam_monitors_assigned)
		return;

	/* When cdp is in use, monitors are allocated in pairs */
	if (cdp_enabled)
		mon_per_partid = 2;
	l3->mon.num_mbm_cntrs = res->class->props.num_mbwu_mon / mon_per_partid;
}

/* When CDP is enabled, monitors are allocated in pairs */
static int effective_num_cntrs(struct rdt_resource *r)
{
	int mon_per_partid = 1;

	if (cdp_enabled)
		mon_per_partid = 2;

	return r->mon.num_mbm_cntrs * mon_per_partid;
}

int resctrl_arch_set_cdp_enabled(enum resctrl_res_level ignored, bool enable)
{
	u64 regval;
	u32 partid, partid_i, partid_d;

	cdp_enabled = enable;

	partid = RESCTRL_RESERVED_CLOSID;

	if (enable) {
		partid_d = resctrl_get_config_index(partid, CDP_CODE);
		partid_i = resctrl_get_config_index(partid, CDP_DATA);
		regval = FIELD_PREP(MPAM1_EL1_PARTID_D, partid_d) |
			 FIELD_PREP(MPAM1_EL1_PARTID_I, partid_i);
	} else {
		regval = FIELD_PREP(MPAM1_EL1_PARTID_D, partid) |
			 FIELD_PREP(MPAM1_EL1_PARTID_I, partid);
	}

 	/* With CDP only half the monitors are available */
	mpam_resctrl_update_mbm_cntrs();

	resctrl_reset_task_closids();

	WRITE_ONCE(arm64_mpam_global_default, regval);

	return 0;
}

static bool mpam_resctrl_hide_cdp(enum resctrl_res_level rid)
{
	return cdp_enabled && !resctrl_arch_get_cdp_enabled(rid);
}

/*
 * MSC may raise an error interrupt if it sees an out or range partid/pmg,
 * and go on to truncate the value. Regardless of what the hardware supports,
 * only the system wide safe value is safe to use.
 */
u32 resctrl_arch_get_num_closid(struct rdt_resource *ignored)
{
	return mpam_partid_max + 1;
}

u32 resctrl_arch_system_num_rmid_idx(void)
{
	u8 closid_shift = fls(mpam_pmg_max);
	u32 num_partid = resctrl_arch_get_num_closid(NULL);

	return num_partid << closid_shift;
}

u32 resctrl_arch_rmid_idx_encode(u32 closid, u32 rmid)
{
	u8 closid_shift = fls(mpam_pmg_max);

	WARN_ON_ONCE(closid_shift > 8);

	return (closid << closid_shift) | rmid;
}

void resctrl_arch_rmid_idx_decode(u32 idx, u32 *closid, u32 *rmid)
{
	u8 closid_shift = fls(mpam_pmg_max);
	u32 pmg_mask = ~(~0 << closid_shift);

	WARN_ON_ONCE(closid_shift > 8);

	*closid = idx >> closid_shift;
	*rmid = idx & pmg_mask;
}

void resctrl_arch_sched_in(struct task_struct *tsk)
{
	lockdep_assert_preemption_disabled();

	mpam_thread_switch(tsk);
}

void resctrl_arch_set_cpu_default_closid_rmid(int cpu, u32 closid, u32 rmid)
{
	WARN_ON_ONCE(closid > U16_MAX);
	WARN_ON_ONCE(rmid > U8_MAX);

	if (!cdp_enabled) {
		mpam_set_cpu_defaults(cpu, closid, closid, rmid, rmid);
	} else {
		/*
		 * When CDP is enabled, resctrl halves the closid range and we
		 * use odd/even partid for one closid.
		 */
		u32 partid_d = resctrl_get_config_index(closid, CDP_DATA);
		u32 partid_i = resctrl_get_config_index(closid, CDP_CODE);

		mpam_set_cpu_defaults(cpu, partid_d, partid_i, rmid, rmid);
	}
}

void resctrl_arch_sync_cpu_closid_rmid(void *info)
{
	struct resctrl_cpu_defaults *r = info;

	lockdep_assert_preemption_disabled();

	if (r) {
		resctrl_arch_set_cpu_default_closid_rmid(smp_processor_id(),
							 r->closid, r->rmid);
	}

	resctrl_arch_sched_in(current);
}

void resctrl_arch_set_closid_rmid(struct task_struct *tsk, u32 closid, u32 rmid)
{
	WARN_ON_ONCE(closid > U16_MAX);
	WARN_ON_ONCE(rmid > U8_MAX);

	if (!cdp_enabled) {
		mpam_set_task_partid_pmg(tsk, closid, closid, rmid, rmid);
	} else {
		u32 partid_d = resctrl_get_config_index(closid, CDP_DATA);
		u32 partid_i = resctrl_get_config_index(closid, CDP_CODE);

		mpam_set_task_partid_pmg(tsk, partid_d, partid_i, rmid, rmid);
	}
}

bool resctrl_arch_match_closid(struct task_struct *tsk, u32 closid)
{
	u64 regval = mpam_get_regval(tsk);
	u32 tsk_closid = FIELD_GET(MPAM1_EL1_PARTID_D, regval);

	if (cdp_enabled)
		tsk_closid >>= 1;

	return tsk_closid == closid;
}

/* The task's pmg is not unique, the partid must be considered too */
bool resctrl_arch_match_rmid(struct task_struct *tsk, u32 closid, u32 rmid)
{
	u64 regval = mpam_get_regval(tsk);
	u32 tsk_closid = FIELD_GET(MPAM1_EL1_PARTID_D, regval);
	u32 tsk_rmid = FIELD_GET(MPAM1_EL1_PMG_D, regval);

	if (cdp_enabled)
		tsk_closid >>= 1;

	return (tsk_closid == closid) && (tsk_rmid == rmid);
}

struct rdt_resource *resctrl_arch_get_resource(enum resctrl_res_level l)
{
	if (l >= RDT_NUM_RESOURCES)
		return NULL;

	return &mpam_resctrl_controls[l].resctrl_res;
}

static void *resctrl_arch_mon_ctx_alloc_no_wait(struct rdt_resource *r,
						enum resctrl_event_id evtid)
{
	struct mpam_resctrl_res *res;
	u32 *ret;

	res = container_of(r, struct mpam_resctrl_res, resctrl_res);

	if (res->class != mpam_resctrl_counters[evtid])
		return ERR_PTR(-EINVAL);

	/* Pre-allocated monitors? */
	switch (evtid) {
	case QOS_L3_MBM_LOCAL_EVENT_ID:
	case QOS_L3_MBM_TOTAL_EVENT_ID:
		if (res->mpam_monitors_free_runing)
			return mon_is_rmid_idx;
		else if (res->mpam_monitors_assigned) {
			return mon_is_assigned;
		}

		ret = kmalloc(sizeof(*ret), GFP_ATOMIC);
		if (!ret)
			return ERR_PTR(-ENOMEM);

		pr_warn_once("Counter assignment disabled with insufficient monitors: falling back to best effort\n");

		*ret = mpam_alloc_mbwu_mon(res->class);
		return ret;
	case QOS_L3_OCCUP_EVENT_ID:
		ret = kmalloc(sizeof(*ret), GFP_ATOMIC);
		if (!ret)
			return ERR_PTR(-ENOMEM);

		*ret = mpam_alloc_csu_mon(res->class);
		return ret;
	}
	
	WARN_ON_ONCE(1);
	return ERR_PTR(-EIO);
}

void *resctrl_arch_mon_ctx_alloc(struct rdt_resource *r,
				 enum resctrl_event_id evtid)
{
	DEFINE_WAIT(wait);
	void *ret;

	might_sleep();

	do {
		prepare_to_wait(&resctrl_mon_ctx_waiters, &wait,
				TASK_INTERRUPTIBLE);
		ret = resctrl_arch_mon_ctx_alloc_no_wait(r, evtid);
		if (PTR_ERR(ret) == -ENOSPC)
			schedule();
	} while (PTR_ERR(ret) == -ENOSPC && !signal_pending(current));
	finish_wait(&resctrl_mon_ctx_waiters, &wait);

	return ret;
}

void resctrl_arch_mon_ctx_free(struct rdt_resource *r,
			       enum resctrl_event_id evtid, void *arch_mon_ctx)
{
	struct mpam_resctrl_res *res;
	u32 mon = *(u32 *)arch_mon_ctx;

	if (mon == USE_RMID_IDX || mon == USE_ABMC_ASSIGNED)
		return;

	kfree(arch_mon_ctx);
	arch_mon_ctx = NULL;

	res = container_of(r, struct mpam_resctrl_res, resctrl_res);

	switch (evtid) {
	case QOS_L3_OCCUP_EVENT_ID:
		mpam_free_csu_mon(res->class, mon);
		break;
	case QOS_L3_MBM_TOTAL_EVENT_ID:
	case QOS_L3_MBM_LOCAL_EVENT_ID:
		mpam_free_mbwu_mon(res->class, mon);
		break;
	}

	wake_up(&resctrl_mon_ctx_waiters);
}

static enum mon_filter_options resctrl_evt_config_to_mpam(u32 local_evt_cfg)
{
	switch (local_evt_cfg) {
	case READS_TO_LOCAL_MEM:
		return COUNT_READ;
	case NON_TEMP_WRITE_TO_LOCAL_MEM:
		return COUNT_WRITE;
	default:
		return COUNT_BOTH;
	}
}

int resctrl_arch_rmid_read(struct rdt_resource	*r, struct rdt_mon_domain *d,
			   u32 closid, u32 rmid, enum resctrl_event_id eventid,
			   u64 *val, void *arch_mon_ctx)
{
	int err;
	u32 idx;
	u64 cdp_val;
	struct mon_cfg cfg;
	struct mpam_resctrl_dom *dom;
	u32 mon = *(u32 *)arch_mon_ctx;
	enum mpam_device_features type;

	resctrl_arch_rmid_read_context_check();

	dom = container_of(d, struct mpam_resctrl_dom, resctrl_mon_dom);

	cfg.match_pmg = true;
	cfg.pmg = rmid;

	switch (eventid) {
	case QOS_L3_OCCUP_EVENT_ID:
		type = mpam_feat_msmon_csu;
		break;
	case QOS_L3_MBM_LOCAL_EVENT_ID:
		type = mpam_feat_msmon_mbwu;
		cfg.opts = resctrl_evt_config_to_mpam(dom->mbm_local_evt_cfg);
		break;
	case QOS_L3_MBM_TOTAL_EVENT_ID:
		type = mpam_feat_msmon_mbwu;
		cfg.opts = resctrl_evt_config_to_mpam(dom->mbm_total_evt_cfg);
		break;
	default:
		return -EINVAL;
	}

	cfg.mon = mon;
	idx = resctrl_arch_rmid_idx_encode(closid, rmid);
	if (cfg.mon == USE_RMID_IDX)
		cfg.mon = idx;

	if (cfg.mon == USE_ABMC_ASSIGNED) {
		cfg.mon = dom->abmc_idx_to_mon[idx];
	}
	if (cfg.mon == ABMC_UNALLOCATED) {
		return -EINVAL;
	}

	if (irqs_disabled()) {
		/* Check if we can access this domain without an IPI */
		err = -EIO;
	} else {
		if (cdp_enabled) {
			cfg.partid = closid << 1;
			err = mpam_msmon_read(dom->comp, &cfg, type, val);
			if (err)
				return err;

			cfg.partid += 1;
			err = mpam_msmon_read(dom->comp, &cfg, type, &cdp_val);
			if (!err)
				*val += cdp_val;
		} else {
			cfg.partid = closid;
			err = mpam_msmon_read(dom->comp, &cfg, type, val);
		}
	}

	return err;
}

void resctrl_arch_reset_rmid(struct rdt_resource *r, struct rdt_mon_domain *d,
			     u32 closid, u32 rmid, enum resctrl_event_id eventid)
{
	struct mon_cfg cfg;
	struct mpam_resctrl_dom *dom;

	if (eventid != QOS_L3_MBM_LOCAL_EVENT_ID)
		return;

	cfg.mon = resctrl_arch_rmid_idx_encode(closid, rmid);
	cfg.match_pmg = true;
	cfg.pmg = rmid;

	dom = container_of(d, struct mpam_resctrl_dom, resctrl_mon_dom);

	if (cdp_enabled) {
		cfg.partid = closid << 1;
		mpam_msmon_reset_mbwu(dom->comp, &cfg);

		cfg.partid += 1;
		mpam_msmon_reset_mbwu(dom->comp, &cfg);
	} else {
		cfg.partid = closid;
		mpam_msmon_reset_mbwu(dom->comp, &cfg);
	}
}

void resctrl_arch_update_cntr(struct rdt_resource *r, struct rdt_mon_domain *d,
                              enum resctrl_event_id evtid, u32 val)
{
	int i;
	u32 pmg;
	u64 ignored;
	struct mpam_resctrl_dom *dom;
	struct mon_cfg mon_cfg = { 0 };

	dom = container_of(d, struct mpam_resctrl_dom, resctrl_mon_dom);

	switch (evtid) {
	case QOS_L3_OCCUP_EVENT_ID:
		WARN_ON_ONCE(1);
		return;
	case QOS_L3_MBM_TOTAL_EVENT_ID:
	case QOS_L3_MBM_LOCAL_EVENT_ID:
		break;
	}

	mutex_lock(&abmc_lock);
	for (i = 0; i < resctrl_arch_system_num_rmid_idx(); i++) {
		mon_cfg.mon = dom->abmc_idx_to_mon[i];
		if (mon_cfg.mon == ABMC_UNALLOCATED)
			continue;
		resctrl_arch_rmid_idx_decode(i, &mon_cfg.partid, &pmg);
		mon_cfg.pmg = pmg;

		mpam_msmon_read(dom->comp, &mon_cfg, mpam_feat_msmon_mbwu,
				&ignored);
	}
	mutex_unlock(&abmc_lock);
}

int resctrl_arch_config_cntr(struct rdt_resource *r, struct rdt_mon_domain *d,
			     enum resctrl_event_id evtid, u32 rmid, u32 closid,
			     u32 cntr_id, bool assign)
{
	u32 idx = resctrl_arch_rmid_idx_encode(closid, rmid);
	struct mon_cfg mon_cfg = { 0 };
	struct mpam_resctrl_res *res;
	struct mpam_resctrl_dom *dom;
	u64 ignored;
	int err = 0;

	res = container_of(r, struct mpam_resctrl_res, resctrl_res);
	dom = container_of(d, struct mpam_resctrl_dom, resctrl_mon_dom);

	if (WARN_ON_ONCE(idx > resctrl_arch_system_num_rmid_idx()))
		return -EIO;

	/* resctrl cntr_id must not exceed the value published to resctrl */
	if (WARN_ON_ONCE(cntr_id > r->mon.num_mbm_cntrs))
		return -EIO;

	mutex_lock(&abmc_lock);
	if (!res->mpam_monitors_assigned) {
		err = -EINVAL;
		goto out_unlock;
	}

	switch (evtid) {
	case QOS_L3_OCCUP_EVENT_ID:
		WARN_ON_ONCE(1);
		err = -EINVAL;
		goto out_unlock;
	case QOS_L3_MBM_TOTAL_EVENT_ID:
	case QOS_L3_MBM_LOCAL_EVENT_ID:
		break;
	}

	mon_cfg.match_pmg = true;
	if (assign)
		dom->abmc_idx_to_mon[idx] = mon_cfg.mon;
	else
		dom->abmc_idx_to_mon[idx] = ABMC_UNALLOCATED;

	if (cdp_enabled) {
		mon_cfg.partid = closid << 1;
		mon_cfg.mon = res->ambc_cntr_id_to_mon[cntr_id << 1];
		mpam_msmon_reset_mbwu(dom->comp, &mon_cfg);
		/* Force a hardware access to start the monitor */
		mpam_msmon_read(dom->comp, &mon_cfg, mpam_feat_msmon_mbwu,
				&ignored);

		mon_cfg.partid += 1;
		mon_cfg.mon = res->ambc_cntr_id_to_mon[(cntr_id << 1) + 1];
		mpam_msmon_reset_mbwu(dom->comp, &mon_cfg);
		mpam_msmon_read(dom->comp, &mon_cfg, mpam_feat_msmon_mbwu,
				&ignored);
	} else {
		mon_cfg.partid = closid;
		mon_cfg.mon = res->ambc_cntr_id_to_mon[cntr_id];
		mpam_msmon_reset_mbwu(dom->comp, &mon_cfg);
		mpam_msmon_read(dom->comp, &mon_cfg, mpam_feat_msmon_mbwu,
				&ignored);
	}

out_unlock:
	mutex_unlock(&abmc_lock);
	return err;
}

/*
 * The rmid realloc threshold should be for the smallest cache exposed to
 * resctrl.
 */
static void update_rmid_limits(unsigned int size)
{
	u32 num_unique_pmg = resctrl_arch_system_num_rmid_idx();

	if (WARN_ON_ONCE(!size))
		return;

	if (resctrl_rmid_realloc_limit && size > resctrl_rmid_realloc_limit)
		return;

	resctrl_rmid_realloc_limit = size;
	resctrl_rmid_realloc_threshold = size / num_unique_pmg;
}

static bool cache_has_usable_cpor(struct mpam_class *class)
{
	struct mpam_props *cprops = &class->props;

	if (!mpam_has_feature(mpam_feat_cpor_part, cprops))
		return false;

	/* TODO: Scaling is not yet supported */
	/* resctrl uses u32 for all bitmap configurations */
	return (class->props.cpbm_wd <= 32);
}

static bool mba_class_use_mbw_part(struct mpam_props *cprops)
{
	return (mpam_has_feature(mpam_feat_mbw_part, cprops) &&
		cprops->mbw_pbm_bits);
}

static bool mba_class_use_mbw_max(struct mpam_props *cprops)
{
	return (mpam_has_feature(mpam_feat_mbw_max, cprops) &&
		cprops->bwa_wd);
}

static bool class_has_usable_mba(struct mpam_props *cprops)
{
	return mba_class_use_mbw_part(cprops) || mba_class_use_mbw_max(cprops);
}

static bool cache_has_usable_csu(struct mpam_class *class)
{
	struct mpam_props *cprops;

	if (!class)
		return false;

	cprops = &class->props;

	if (!mpam_has_feature(mpam_feat_msmon_csu, cprops))
		return false;

	/*
	 * CSU counters settle on the value, so we can get away with
	 * having only one.
	 */
	if (!cprops->num_csu_mon)
		return false;

	return (mpam_partid_max > 1) || (mpam_pmg_max != 0);
}

static bool class_has_usable_mbwu(struct mpam_class *class)
{
	struct mpam_props *cprops = &class->props;

	if (!mpam_has_feature(mpam_feat_msmon_mbwu, cprops))
		return false;

	/* ABMC support means fewer monitors than needed can be supported */
	if (cprops->num_mbwu_mon == 0)
		return false;

	return (mpam_partid_max > 1) || (mpam_pmg_max != 0);
}

/*
 * Calculate the percentage change from each implemented bit in the control
 * This can return 0 when BWA_WD is greater than 6. (100 / (1<<7) == 0)
 */
static u32 get_mba_granularity(struct mpam_props *cprops)
{
	if (mba_class_use_mbw_part(cprops)) {
		return max(MAX_MBA_BW / cprops->mbw_pbm_bits, 1);
	} else if (mba_class_use_mbw_max(cprops)) {
		/*
		 * bwa_wd is the number of bits implemented in the 0.xxx
		 * fixed point fraction. 1 bit is 50%, 2 is 25% etc.
		 */
		return max(MAX_MBA_BW / (1 << cprops->bwa_wd), 1);
	}

	return 0;
}

static u32 mbw_pbm_to_percent(const unsigned long mbw_pbm, struct mpam_props *cprops)
{
	u32 num_bits = bitmap_weight(&mbw_pbm, (unsigned int)cprops->mbw_pbm_bits);

	if (cprops->mbw_pbm_bits == 0)
		return 0;

	return (num_bits * MAX_MBA_BW) / cprops->mbw_pbm_bits;
}

static u32 mbw_max_to_percent(u16 mbw_max, struct mpam_props *cprops)
{
	u32 max_fract = 0xffff;

	max_fract >>= 16 - cprops->bwa_wd;
	mbw_max >>= 16 - cprops->bwa_wd;

	return (mbw_max * 100) / max_fract;
}

static u32 percent_to_mbw_pbm(u8 pc, struct mpam_props *cprops)
{
	u8 num_bits = (pc * cprops->mbw_pbm_bits) / MAX_MBA_BW;

	if (!num_bits)
		return 0;

	/* TODO: pick bits at random to avoid contention */
	return (1 << num_bits) - 1;
}

static u16 percent_to_mbw_max(u8 pc, struct mpam_props *cprops)
{
	u32 value;

	if (WARN_ON_ONCE(cprops->bwa_wd > 16))
		return 100;

	value = ((pc << cprops->bwa_wd) + 50) / 100;

	if (value < 1)
		return 0;
	return (value - 1) << (16 - cprops->bwa_wd);
}

/* Find the L3 component that holds this CPU */
static struct mpam_component *__topology_l3_equivalent(int cpu)
{
	struct mpam_component *l3_iter;
	struct mpam_resctrl_res *res;
	struct mpam_class *l3;

	res = &mpam_resctrl_controls[RDT_RESOURCE_L3];
	l3 = res->class;
	if (!l3)
		return NULL;

	list_for_each_entry(l3_iter, &l3->components, class_list) {
		if (cpumask_test_cpu(cpu, &l3_iter->affinity))
			return l3_iter;
	}

	return NULL;
}

static bool __topology_matches_l3(struct mpam_class *victim,
				  cpumask_var_t tmp_cpumask)
{
	struct mpam_component *victim_iter, *l3_iter;
	int cpu;

	/*
	 * Walk the two component lists and compare the affinity masks.
	 * These lists/masks are static, the resctrl domain versions depend on
	 * which CPUs are online.
	 */
	list_for_each_entry(victim_iter, &victim->components, class_list) {
		cpu = cpumask_any(&victim_iter->affinity);
		l3_iter = __topology_l3_equivalent(cpu);
		if (!l3_iter) {
			pr_debug("%s: Failed to find matching component\n", __func__);
			return false;
		}

		/* Any differing bits in the affinity mask? */
		cpumask_xor(tmp_cpumask, &l3_iter->affinity, &victim_iter->affinity);
		if (!cpumask_empty(tmp_cpumask)) {
			pr_debug("%s: Mismatched CPU mask\n", __func__);
			return false;
		}
	}

	return true;
}

/*
 * resctrl expects all the worlds a Xeon, and all counters are on the
 * L3. We play fast and loose with this, mapping counters on other
 * classes - provided the CPU->domain mapping is the same kind of shape.
 * Using cacheinfo directly would make this work even if resctrl can't
 * use the L3 - but cacheinfo can't tell us anything about offline CPUs.
 * Use the mpam_class we picked for L3 so we can use its domain list
 * for this check.
 */
static bool topology_matches_l3(struct mpam_class *victim)
{
	bool matches;
	cpumask_var_t tmp_cpumask;

	if (!alloc_cpumask_var(&tmp_cpumask, GFP_KERNEL))
		return false;

	matches = __topology_matches_l3(victim, tmp_cpumask);

	free_cpumask_var(tmp_cpumask);

	return matches;
}

/* Test whether we can export MPAM_CLASS_CACHE:{2,3}? */
static void mpam_resctrl_pick_caches(void)
{
	int idx;
	struct mpam_class *class;
	struct mpam_resctrl_res *res;

	lockdep_assert_cpus_held();

	idx = srcu_read_lock(&mpam_srcu);
	list_for_each_entry_rcu(class, &mpam_classes, classes_list) {
		if (class->type != MPAM_CLASS_CACHE) {
			pr_debug("pick_caches: Class is not a cache\n");
			continue;
		}

		if (class->level != 2 && class->level != 3) {
			pr_debug("pick_caches: not L2 or L3\n");
			continue;
		}

		if (!cache_has_usable_cpor(class)) {
			pr_debug("pick_caches: Cache misses CPOR\n");
			continue;
		}

		if (!cpumask_equal(&class->affinity, cpu_possible_mask)) {
			pr_debug("pick_caches: Class has missing CPUs\n");
			continue;
		}

		if (class->level == 2)
			res = &mpam_resctrl_controls[RDT_RESOURCE_L2];
		else
			res = &mpam_resctrl_controls[RDT_RESOURCE_L3];
		res->class = class;
		exposed_alloc_capable = true;
	}
	srcu_read_unlock(&mpam_srcu, idx);
}

static void mpam_resctrl_pick_mba(void)
{
	struct mpam_class *class, *candidate_class = NULL;
	struct mpam_resctrl_res *res;
	int idx;

	lockdep_assert_cpus_held();

	idx = srcu_read_lock(&mpam_srcu);
	list_for_each_entry_rcu(class, &mpam_classes, classes_list) {
		struct mpam_props *cprops = &class->props;

		if (class->level < 3) {
			pr_debug("pick_mba: class is before L3\n");
			continue;
		}

		if (!class_has_usable_mba(cprops)) {
			pr_debug("pick_mba: class has no bandwidth control\n");
			continue;
		}

		if (!cpumask_equal(&class->affinity, cpu_possible_mask)) {
			pr_debug("pick_mba: class has missing CPUs\n");
			continue;
		}

		if (!topology_matches_l3(class)) {
			pr_debug("pick_mba: class topology doesn't match L3\n");
			continue;
		}

		/*
		 * mba_sc reads the mbm_local counter, and waggles the MBA controls.
		 * mbm_local is implicitly part of the L3, pick a resource to be MBA
		 * that as close as possible to the L3.
		 */
		if (!candidate_class || class->level < candidate_class->level)
			candidate_class = class;
	}
	srcu_read_unlock(&mpam_srcu, idx);

	if (candidate_class) {
		res = &mpam_resctrl_controls[RDT_RESOURCE_MBA];
		res->class = candidate_class;
		exposed_alloc_capable = true;
	}
}

static void counter_update_class(enum resctrl_event_id evt_id,
				 struct mpam_class *class)
{
	struct mpam_class *existing_class = mpam_resctrl_counters[evt_id];

	if (existing_class) {
		if (class->level == 3)
			return; /* L3 wins */
		else if (existing_class->level < class->level)
			return; /* closer is better */
	}

	mpam_resctrl_counters[evt_id] = class;
	exposed_mon_capable = true;
}

static void mpam_resctrl_pick_counters(void)
{
	struct mpam_class *class;
	unsigned int cache_size;
	bool has_csu, has_mbwu;
	int idx;

	lockdep_assert_cpus_held();

	idx = srcu_read_lock(&mpam_srcu);
	list_for_each_entry_rcu(class, &mpam_classes, classes_list) {
		struct mpam_props *cprops = &class->props;

		if (class->level < 3)
			continue;

		if (!cpumask_equal(&class->affinity, cpu_possible_mask))
			continue;

		has_csu = cache_has_usable_csu(class);
		if (has_csu && topology_matches_l3(class)) {
			/* CSU counters only make sense on a cache. */
			switch (class->type) {
			case MPAM_CLASS_CACHE:
				/* Assume cache levels are the same size for all CPUs... */
				cache_size = get_cpu_cacheinfo_size(smp_processor_id(), class->level);
				if (!cache_size) {
					pr_debug("pick_caches: Could not read cache size\n");
					continue;
				}

				if (mpam_has_feature(mpam_feat_msmon_csu, cprops))
					update_rmid_limits(cache_size);

				counter_update_class(QOS_L3_OCCUP_EVENT_ID, class);
				break;
			default:
				break;
			}
		}

		has_mbwu = class_has_usable_mbwu(class);
		if (has_mbwu && topology_matches_l3(class)) {
			/*
			 * MBWU counters may be 'local' or 'total' depending on where
			 * they are in the topology. Counters on caches are assumed to
			 * be local. If it's on the memory controller, its assumed to
			 * be global.
			 * TODO: check mbm_local matches NUMA boundaries...
			 */
			switch (class->type) {
			case MPAM_CLASS_CACHE:
				counter_update_class(QOS_L3_MBM_LOCAL_EVENT_ID, class);
				break;
			case MPAM_CLASS_MEMORY:
				counter_update_class(QOS_L3_MBM_TOTAL_EVENT_ID, class);
				break;
			default:
				break;
			}
		}
	}
	srcu_read_unlock(&mpam_srcu, idx);
}

bool resctrl_arch_is_evt_configurable(enum resctrl_event_id evt)
{
	struct mpam_class *class;
	struct mpam_props *cprops;

	class = mpam_resctrl_counters[evt];
	if (!class)
		return false;

	cprops = &class->props;

	return mpam_has_feature(mpam_feat_msmon_mbwu_rwbw, cprops);
}

u32 resctrl_arch_mon_event_config_get(struct rdt_mon_domain *d,
				       enum resctrl_event_id eventid)
{
	struct mpam_resctrl_dom *dom;

	dom = container_of(d, struct mpam_resctrl_dom, resctrl_mon_dom);
	
	switch (eventid) {
	case QOS_L3_OCCUP_EVENT_ID:
		break;
	case QOS_L3_MBM_TOTAL_EVENT_ID:
		return dom->mbm_total_evt_cfg & MAX_EVT_CONFIG_BITS;
	case QOS_L3_MBM_LOCAL_EVENT_ID:
		return dom->mbm_local_evt_cfg & MAX_EVT_CONFIG_BITS;
	}

	/* Never expect to get here */
	WARN_ON_ONCE(1);

	return INVALID_CONFIG_VALUE;
}

void resctrl_arch_mon_event_config_set(struct rdt_mon_domain *d,
				       enum resctrl_event_id eventid, u32 val)
{
	struct mpam_resctrl_dom *dom;

	dom = container_of(d, struct mpam_resctrl_dom, resctrl_mon_dom);
	switch (eventid) {
	case QOS_L3_OCCUP_EVENT_ID:
		break;
	case QOS_L3_MBM_TOTAL_EVENT_ID:
		dom->mbm_total_evt_cfg = val & MAX_EVT_CONFIG_BITS;
		break;
	case QOS_L3_MBM_LOCAL_EVENT_ID:
		dom->mbm_local_evt_cfg = val & MAX_EVT_CONFIG_BITS;
		break;
	}

	/* TODO: update hardware. */

	return;
}

void resctrl_arch_reset_rmid_all(struct rdt_resource *r, struct rdt_mon_domain *d)
{
	struct mpam_resctrl_dom *dom;

	dom = container_of(d, struct mpam_resctrl_dom, resctrl_mon_dom);
	dom->mbm_local_evt_cfg = MPAM_RESTRL_EVT_CONFIG_VALID;
	dom->mbm_total_evt_cfg = MPAM_RESTRL_EVT_CONFIG_VALID;
	mpam_msmon_reset_all_mbwu(dom->comp);
}

static int mpam_resctrl_abmc_alloc_domain(struct rdt_mon_domain *d, int nid)
{
	int i, num_idx;
	struct mpam_resctrl_dom *dom;

	lockdep_assert_held(&abmc_lock);

	num_idx = resctrl_arch_system_num_rmid_idx();
	dom = container_of(d, struct mpam_resctrl_dom, resctrl_mon_dom);
	dom->abmc_idx_to_mon = kzalloc_node(num_idx * sizeof(dom->abmc_idx_to_mon),
					 GFP_KERNEL, nid);
	if (!dom->abmc_idx_to_mon) {
		pr_debug("Failed to allocate ABMC monitor array\n");
		return -ENOMEM;
	}

	for (i = 0; i < num_idx; i++)
		dom->abmc_idx_to_mon[i] = ABMC_UNALLOCATED;

	return 0;
}

static int mpam_resctrl_abmc_free_domain(struct mpam_resctrl_res *res,
					  struct rdt_mon_domain *d)
{
	struct mpam_resctrl_dom *dom;

	lockdep_assert_held(&abmc_lock);

	dom = container_of(d, struct mpam_resctrl_dom, resctrl_mon_dom);
	kfree(dom->abmc_idx_to_mon);
	dom->abmc_idx_to_mon = NULL;

	return 0;
}

static void mpam_resctrl_abmc_disable(struct mpam_resctrl_res *res)
{
	struct rdt_resource *r = &res->resctrl_res;
	struct rdt_mon_domain *d;
	int i;

	lockdep_assert_held(&abmc_lock);

	list_for_each_entry(d, &r->mon_domains, hdr.list) {
		mpam_resctrl_abmc_free_domain(res, d);
		resctrl_arch_reset_rmid_all(r, d);
	}

	for (i = 0; i < effective_num_cntrs(r); i++) {
		mpam_free_mbwu_mon(res->class, res->ambc_cntr_id_to_mon[i]);
	}
	kfree(res->ambc_cntr_id_to_mon);
	res->ambc_cntr_id_to_mon = NULL;

	res->mpam_monitors_assigned = false;
}

static int mpam_resctrl_abmc_enable(struct mpam_resctrl_res *res)
{
	struct rdt_mon_domain *d;
	int i, num_mon, err = 0, nid;
	struct rdt_resource *r = &res->resctrl_res;

	lockdep_assert_held(&abmc_lock);

	/*
	 * Allocate all the monitors from the mpam driver as resctrl wants to
	 * allocate them itself.
	 */
	num_mon = effective_num_cntrs(r);
	res->ambc_cntr_id_to_mon = kzalloc(num_mon * sizeof(res->ambc_cntr_id_to_mon),
					 GFP_KERNEL);
	if (!res->ambc_cntr_id_to_mon) {
		pr_debug("Failed to allocate ABMC monitor array\n");
		return -ENOMEM;
	}

	for (i = 0; i < num_mon; i++) {
		err = mpam_alloc_mbwu_mon(res->class);
		if (err < 0) {
			pr_debug("Failed to allocate %u monitors for ABMC\n", num_mon);
			break;
		}

		res->ambc_cntr_id_to_mon[i] = err;
		err = 0;
 	}

	if (err) {
		num_mon = i;

		/* Free as many monitors as were allocated */
		for (i = 0; i < num_mon; i++)
			mpam_free_mbwu_mon(res->class, res->ambc_cntr_id_to_mon[i]);

		return -EBUSY;
	}

	list_for_each_entry(d, &r->mon_domains, hdr.list) {
		nid = cpu_to_node(cpumask_any(&d->hdr.cpu_mask));
		err = mpam_resctrl_abmc_alloc_domain(d, nid);
		if (err)
			break;
		resctrl_arch_reset_rmid_all(r, d);
	}

	if (err)
		mpam_resctrl_abmc_disable(res);
	else
		res->mpam_monitors_assigned = true;

	return err;
}

/* Is ABMC enabled for this resource? */
bool resctrl_arch_mbm_cntr_assign_enabled(struct rdt_resource *r)
{
	struct mpam_resctrl_res *res;
	bool ret;

	res = container_of(r, struct mpam_resctrl_res, resctrl_res);

	mutex_lock(&abmc_lock);
	ret = res->mpam_monitors_assigned;
	mutex_unlock(&abmc_lock);

	return ret;
}

/* Toggle, and IPI to update hardware */
int resctrl_arch_mbm_cntr_assign_set(struct rdt_resource *r, bool enable)
{
	struct mpam_resctrl_res *res;
	bool enabled;
	int err = 0;

	if (!r->mon.mbm_cntr_assignable)
		return 0;

	if (!r->mon.num_mbm_cntrs) {
		pr_debug("No monitors available for this resource\n");
		return -EINVAL;
	}

	res = container_of(r, struct mpam_resctrl_res, resctrl_res);
	if (!res->class) {
		pr_debug("Bad class for ABMC\n");
		return -EINVAL;
	}

	mutex_lock(&abmc_lock);
	enabled = res->mpam_monitors_assigned;
	if (enable && !enabled)
		err = mpam_resctrl_abmc_enable(res);
	else if (enabled && !enable)
		mpam_resctrl_abmc_disable(res);
	mutex_unlock(&abmc_lock);

	/*
	 * Hardware doesn't need updating to enable this, only once a counter
	 * has been allocated.
	 */

	return err;
}

static int mpam_resctrl_control_init(struct mpam_resctrl_res *res,
				     enum resctrl_res_level type)
{
	struct mpam_class *class = res->class;
	struct mpam_props *cprops = &class->props;
	struct rdt_resource *r = &res->resctrl_res;

	switch (res->resctrl_res.rid) {
	case RDT_RESOURCE_L2:
	case RDT_RESOURCE_L3:
		r->alloc_capable = true;
		r->schema_fmt = RESCTRL_SCHEMA_BITMAP;
		r->cache.arch_has_sparse_bitmasks = true;

		/* TODO: Scaling is not yet supported */
		r->cache.cbm_len = class->props.cpbm_wd;
		/* mpam_devices will reject empty bitmaps */
		r->cache.min_cbm_bits = 1;

		if (r->rid == RDT_RESOURCE_L2) {
			r->name = "L2";
			r->ctrl_scope = RESCTRL_L2_CACHE;
		} else {
			r->name = "L3";
			r->ctrl_scope = RESCTRL_L3_CACHE;
		}

		/*
		 * Which bits are shared with other ...things...
		 * Unknown devices use partid-0 which uses all the bitmap
		 * fields. Until we configured the SMMU and GIC not to do this
		 * 'all the bits' is the correct answer here.
		 */
		r->cache.shareable_bits = resctrl_get_default_ctrl(r);
		break;
	case RDT_RESOURCE_MBA:
		r->alloc_capable = true;
		r->schema_fmt = RESCTRL_SCHEMA_RANGE;
		r->ctrl_scope = RESCTRL_L3_CACHE;

		r->membw.delay_linear = true;
		r->membw.throttle_mode = THREAD_THROTTLE_UNDEFINED;
		r->membw.min_bw = get_mba_granularity(cprops);
		r->membw.max_bw = MAX_MBA_BW;
		r->membw.bw_gran = get_mba_granularity(cprops);

		r->name = "MB";

		/* Round up to at least 1% */
		if (!r->membw.bw_gran)
			r->membw.bw_gran = 1;

		break;
	default:
		break;
	}

	return 0;
}

static void mpam_resctrl_monitor_init(struct mpam_class *class,
				      enum resctrl_event_id type)
{
	struct mpam_resctrl_res *res = &mpam_resctrl_controls[RDT_RESOURCE_L3];
	struct rdt_resource *l3 = &res->resctrl_res;
	struct mpam_props *cprops;

	/* Did we find anything for this monitor type? */
	if (!mpam_resctrl_counters[type])
		return;

	/* There also needs to be an L3 class */
	if (!res->class)
		return;

	cprops = &res->class->props;

	switch (type) {
	case QOS_L3_MBM_TOTAL_EVENT_ID:
	case QOS_L3_MBM_LOCAL_EVENT_ID:
		/*
		 * resctrl expects the bandwidth counters to be free running,
		 * which means to expose the files in the filesystem we need
		 * as many monitors as resctrl has control/monitor groups.
		 * Otherwise, enable ABMC.
		 */
		if (cprops->num_mbwu_mon >= resctrl_arch_system_num_rmid_idx())
			res->mpam_monitors_free_runing = true;
		else
			res->mpam_monitors_assigned = true;

		if (res->mpam_monitors_assigned) {
			l3->mon.mbm_cntr_assignable = true;
			mpam_resctrl_update_mbm_cntrs();
			
			/* Where supported, ABMC has to be enabled by default */
			mutex_lock(&abmc_lock);
			mpam_resctrl_abmc_enable(res);
			mutex_unlock(&abmc_lock);
		}
		break;
	default:
		break;
	}

	/* Called multiple times!, once per event type */
	if (exposed_mon_capable) {
		l3->mon_capable = true;

		/*
		 * Unfortunately, num_rmid doesn't mean anything for
		 * mpam, and its exposed to user-space!
		 * num-rmid is supposed to mean the number of groups
		 * that can be created, both control or monitor groups.
		 * For mpam, each control group has its own pmg/rmid
		 * space.
		 */
		l3->mon.num_rmid = 1;
	}
}

int mpam_resctrl_setup(void)
{
	int err = 0;
	enum resctrl_event_id j;
	enum resctrl_res_level i;
	struct mpam_class *class;
	struct mpam_resctrl_res *res;

	wait_event(wait_cacheinfo_ready, cacheinfo_ready);

	cpus_read_lock();
	for (i = 0; i < RDT_NUM_RESOURCES; i++) {
		res = &mpam_resctrl_controls[i];
		INIT_LIST_HEAD(&res->resctrl_res.ctrl_domains);
		INIT_LIST_HEAD(&res->resctrl_res.mon_domains);
		INIT_LIST_HEAD(&res->resctrl_res.evt_list);
		res->resctrl_res.rid = i;
	}

	/* Find some classes to use for controls */
	mpam_resctrl_pick_caches();
	mpam_resctrl_pick_mba();

	/* Initialise the resctrl structures from the classes */
	for (i = 0; i < RDT_NUM_RESOURCES; i++) {
		res = &mpam_resctrl_controls[i];
		if (!res->class)
			continue;	// dummy resource

		err = mpam_resctrl_control_init(res, i);
		if (err)
			break;
	}

	/* Find some classes to use for monitors */
	mpam_resctrl_pick_counters();

	for (j = 0; j < QOS_NUM_EVENTS; j++) {
		class = mpam_resctrl_counters[j];
		if (!class)
			continue;	// dummy resource

		mpam_resctrl_monitor_init(class, j);
	}

	cpus_read_unlock();

	if (!err && !exposed_alloc_capable && !exposed_mon_capable)
		err = -EOPNOTSUPP;

	if (!err) {
		if (!is_power_of_2(mpam_pmg_max + 1)) {
			/*
			 * If not all the partid*pmg values are valid indexes,
			 * resctrl may allocate pmg that don't exist. This
			 * should cause an error interrupt.
			 */
			pr_warn("Number of PMG is not a power of 2! resctrl may misbehave");
		}

		err = resctrl_init();
		if (!err)
			WRITE_ONCE(resctrl_enabled, true);
	}

	return err;
}

static void mpam_resctrl_exit(void)
{
	if (!READ_ONCE(resctrl_enabled))
		return;

	WRITE_ONCE(resctrl_enabled, false);
	resctrl_exit();
}

u32 resctrl_arch_get_config(struct rdt_resource *r, struct rdt_ctrl_domain *d,
			    u32 closid, enum resctrl_conf_type type)
{
	u32 partid;
	struct mpam_config *cfg;
	struct mpam_props *cprops;
	struct mpam_resctrl_res *res;
	struct mpam_resctrl_dom *dom;
	enum mpam_device_features configured_by;

	lockdep_assert_cpus_held();

	if (!mpam_is_enabled())
		return resctrl_get_default_ctrl(r);

	res = container_of(r, struct mpam_resctrl_res, resctrl_res);
	dom = container_of(d, struct mpam_resctrl_dom, resctrl_ctrl_dom);
	cprops = &res->class->props;

	/*
	 * When CDP is enabled, but the resource doesn't support it,
	 * the control is cloned across both partids.
	 * Pick one at random to read:
	 */
	if (mpam_resctrl_hide_cdp(r->rid))
		type = CDP_DATA;

	partid = resctrl_get_config_index(closid, type);
	cfg = &dom->comp->cfg[partid];

	switch (r->rid) {
	case RDT_RESOURCE_L2:
	case RDT_RESOURCE_L3:
		configured_by = mpam_feat_cpor_part;
		break;
	case RDT_RESOURCE_MBA:
		if (mba_class_use_mbw_part(cprops)) {
			configured_by = mpam_feat_mbw_part;
			break;
		} else if (mpam_has_feature(mpam_feat_mbw_max, cprops)) {
			configured_by = mpam_feat_mbw_max;
			break;
		}
		fallthrough;
	default:
		return -EINVAL;
	}

	if (!r->alloc_capable || partid >= resctrl_arch_get_num_closid(r) ||
	    !mpam_has_feature(configured_by, cfg))
		return resctrl_get_default_ctrl(r);

	switch (configured_by) {
	case mpam_feat_cpor_part:
		/* TODO: Scaling is not yet supported */
		return cfg->cpbm;
	case mpam_feat_mbw_part:
		/* TODO: Scaling is not yet supported */
		return mbw_pbm_to_percent(cfg->mbw_pbm, cprops);
	case mpam_feat_mbw_max:
		return mbw_max_to_percent(cfg->mbw_max, cprops);
	default:
		return -EINVAL;
	}
}

int resctrl_arch_update_one(struct rdt_resource *r, struct rdt_ctrl_domain *d,
			    u32 closid, enum resctrl_conf_type t, u32 cfg_val)
{
	int err;
	u32 partid;
	struct mpam_config cfg;
	struct mpam_props *cprops;
	struct mpam_resctrl_res *res;
	struct mpam_resctrl_dom *dom;

	lockdep_assert_cpus_held();
	lockdep_assert_irqs_enabled();

	/*
	 * NOTE: don't check the CPU as mpam_apply_config() doesn't care,
	 * and resctrl_arch_update_domains() depends on this.
	 */
	res = container_of(r, struct mpam_resctrl_res, resctrl_res);
	dom = container_of(d, struct mpam_resctrl_dom, resctrl_ctrl_dom);
	cprops = &res->class->props;

	partid = resctrl_get_config_index(closid, t);
	if (!r->alloc_capable || partid >= resctrl_arch_get_num_closid(r))
		return -EINVAL;

	cfg.features = 0;
	switch (r->rid) {
	case RDT_RESOURCE_L2:
	case RDT_RESOURCE_L3:
		/* TODO: Scaling is not yet supported */
		cfg.cpbm = cfg_val;
		mpam_set_feature(mpam_feat_cpor_part, &cfg);
		break;
	case RDT_RESOURCE_MBA:
		if (mba_class_use_mbw_part(cprops)) {
			cfg.mbw_pbm = percent_to_mbw_pbm(cfg_val, cprops);
			mpam_set_feature(mpam_feat_mbw_part, &cfg);
			break;
		} else if (mpam_has_feature(mpam_feat_mbw_max, cprops)) {
			cfg.mbw_max = percent_to_mbw_max(cfg_val, cprops);
			mpam_set_feature(mpam_feat_mbw_max, &cfg);
			break;
		}
		fallthrough;
	default:
		return -EINVAL;
	}

	/*
	 * When CDP is enabled, but the resource doesn't support it, we need to
	 * apply the same configuration to the other partid.
	 */
	if (mpam_resctrl_hide_cdp(r->rid)) {
		partid = resctrl_get_config_index(closid, CDP_CODE);
		err = mpam_apply_config(dom->comp, partid, &cfg);
		if (err)
			return err;

		partid = resctrl_get_config_index(closid, CDP_DATA);
		return mpam_apply_config(dom->comp, partid, &cfg);

	} else {
		return mpam_apply_config(dom->comp, partid, &cfg);
	}
}

/* TODO: this is IPI heavy */
int resctrl_arch_update_domains(struct rdt_resource *r, u32 closid)
{
	int err = 0;
	enum resctrl_conf_type t;
	struct rdt_ctrl_domain *d;
	struct resctrl_staged_config *cfg;

	lockdep_assert_cpus_held();
	lockdep_assert_irqs_enabled();

	list_for_each_entry(d, &r->ctrl_domains, hdr.list) {
		for (t = 0; t < CDP_NUM_TYPES; t++) {
			cfg = &d->staged_config[t];
			if (!cfg->have_new_ctrl)
				continue;

			err = resctrl_arch_update_one(r, d, closid, t,
						      cfg->new_ctrl);
			if (err)
				return err;
		}
	}

	return err;
}

void resctrl_arch_reset_all_ctrls(struct rdt_resource *r)
{
	struct mpam_resctrl_res *res;

	lockdep_assert_cpus_held();

	if (!mpam_is_enabled())
		return;

	res = container_of(r, struct mpam_resctrl_res, resctrl_res);
	mpam_reset_class_locked(res->class);
}

static void mpam_resctrl_domain_hdr_init(int cpu, struct mpam_class *class,
					 struct mpam_component *comp,
					 struct rdt_domain_hdr *hdr)
{
	struct mpam_component *l3_comp;

	INIT_LIST_HEAD(&hdr->list);
	if (class->type == MPAM_CLASS_CACHE) {
		hdr->id = comp->comp_id;
	} else if (topology_matches_l3(class)) {
		/* Use the corresponding L3 component ID as the domain ID */
		l3_comp = __topology_l3_equivalent(cpu);
		if (l3_comp)
			hdr->id = l3_comp->comp_id;
		else
			hdr->id = comp->comp_id;
	} else {
		/* TODO: if this matches the numa topology, use the nid to look
		 * like SNC
		 */
		/*
		 * Otherwise, expose the ID used by the firmware table code.
		 */
		hdr->id = comp->comp_id;
	}
	cpumask_set_cpu(cpu, &hdr->cpu_mask);
}

static bool mpam_resctrl_offline_domain_hdr(unsigned int cpu,
					    struct rdt_domain_hdr *hdr)
{
	cpumask_clear_cpu(cpu, &hdr->cpu_mask);
	if (cpumask_empty(&hdr->cpu_mask)) {
		list_del(&hdr->list);
		return true;
	}

	return false;
}

static struct mpam_resctrl_dom *
mpam_resctrl_alloc_domain(unsigned int cpu, struct mpam_resctrl_res *res)
{
	int err;
	struct mpam_resctrl_dom *dom;
	struct rdt_mon_domain *mon_d;
	struct rdt_ctrl_domain *ctrl_d;
	struct mpam_class *class = res->class;
	struct mpam_component *comp_iter, *comp;

	comp = NULL;
	list_for_each_entry(comp_iter, &class->components, class_list) {
		if (cpumask_test_cpu(cpu, &comp_iter->affinity)) {
			comp = comp_iter;
			break;
		}
	}

	/* cpu with unknown exported component? */
	if (WARN_ON_ONCE(!comp))
		return ERR_PTR(-EINVAL);

	dom = kzalloc_node(sizeof(*dom), GFP_KERNEL, cpu_to_node(cpu));
	if (!dom)
		return ERR_PTR(-ENOMEM);

	dom->comp = comp;
	dom->mbm_local_evt_cfg = MPAM_RESTRL_EVT_CONFIG_VALID;
	dom->mbm_total_evt_cfg = MPAM_RESTRL_EVT_CONFIG_VALID;

	ctrl_d = &dom->resctrl_ctrl_dom;
	mpam_resctrl_domain_hdr_init(cpu, class, comp, &ctrl_d->hdr);
	ctrl_d->hdr.type = RESCTRL_CTRL_DOMAIN;
	/* TODO: this list should be sorted */
	list_add_tail(&ctrl_d->hdr.list, &res->resctrl_res.ctrl_domains);
	err = resctrl_online_ctrl_domain(&res->resctrl_res, ctrl_d);
	if (err) {
		mpam_resctrl_offline_domain_hdr(cpu, &ctrl_d->hdr);
		return ERR_PTR(err);
	}

	mon_d = &dom->resctrl_mon_dom;
	mpam_resctrl_domain_hdr_init(cpu, class, comp, &mon_d->hdr);
	mon_d->hdr.type = RESCTRL_MON_DOMAIN;

	mutex_lock(&abmc_lock);
	err = mpam_resctrl_abmc_alloc_domain(mon_d, cpu_to_node(cpu));
	mutex_unlock(&abmc_lock);
	
	/* TODO: this list should be sorted */
	list_add_tail(&mon_d->hdr.list, &res->resctrl_res.mon_domains);
	err = resctrl_online_mon_domain(&res->resctrl_res, mon_d);
	if (err) {
		mpam_resctrl_offline_domain_hdr(cpu, &mon_d->hdr);
		resctrl_offline_ctrl_domain(&res->resctrl_res, ctrl_d);
		return ERR_PTR(err);
	}

	return dom;
}

/* Like resctrl_get_domain_from_cpu(), but for offline CPUs */
static struct mpam_resctrl_dom *
mpam_get_domain_from_cpu(int cpu, struct mpam_resctrl_res *res)
{
	struct rdt_ctrl_domain *d;
	struct mpam_resctrl_dom *dom;

	lockdep_assert_cpus_held();

	list_for_each_entry(d, &res->resctrl_res.ctrl_domains, hdr.list) {
		dom = container_of(d, struct mpam_resctrl_dom, resctrl_ctrl_dom);

		if (cpumask_test_cpu(cpu, &dom->comp->affinity))
			return dom;
	}

	return NULL;
}

int mpam_resctrl_online_cpu(unsigned int cpu)
{
	int i;
	struct mpam_resctrl_dom *dom;
	struct mpam_resctrl_res *res;

	for (i = 0; i < RDT_NUM_RESOURCES; i++) {
		res = &mpam_resctrl_controls[i];
		if (!res->class)
			continue;	// dummy_resource;

		dom = mpam_get_domain_from_cpu(cpu, res);
		if (!dom)
			dom = mpam_resctrl_alloc_domain(cpu, res);
		if (IS_ERR(dom))
			return PTR_ERR(dom);

		cpumask_set_cpu(cpu, &dom->resctrl_ctrl_dom.hdr.cpu_mask);
		cpumask_set_cpu(cpu, &dom->resctrl_mon_dom.hdr.cpu_mask);
	}

	resctrl_online_cpu(cpu);
	return 0;
}

int mpam_resctrl_offline_cpu(unsigned int cpu)
{
	int i;
	struct mpam_resctrl_res *res;
	struct mpam_resctrl_dom *dom;
	struct rdt_mon_domain *mon_d;
	struct rdt_ctrl_domain *ctrl_d;

	resctrl_offline_cpu(cpu);

	for (i = 0; i < RDT_NUM_RESOURCES; i++) {
		res = &mpam_resctrl_controls[i];
		if (!res->class)
			continue;	// dummy resource

		ctrl_d = resctrl_get_ctrl_domain_from_cpu(cpu, &res->resctrl_res);
		if (WARN_ON_ONCE(!ctrl_d))
			continue;

		resctrl_offline_ctrl_domain(&res->resctrl_res, ctrl_d);
		if (!mpam_resctrl_offline_domain_hdr(cpu, &ctrl_d->hdr))
			continue;

		dom = container_of(ctrl_d, struct mpam_resctrl_dom, resctrl_ctrl_dom);
		mon_d = &dom->resctrl_mon_dom;
		resctrl_offline_mon_domain(&res->resctrl_res, mon_d);
		if (!mpam_resctrl_offline_domain_hdr(cpu, &mon_d->hdr))
			continue;

		kfree(dom);
	}

	return 0;
}

/*
 * The driver is detaching an MSC from this class, if resctrl was using it,
 * pull on resctrl_exit().
 */
void mpam_resctrl_teardown_class(struct mpam_class *class)
{
	int i;
	bool found = false;
	struct mpam_resctrl_res *res;

	might_sleep();

	for (i = 0; i < RDT_NUM_RESOURCES; i++) {
		res = &mpam_resctrl_controls[i];
		if (res->class == class) {
			found = true;
			break;
		}
	}

	for (i = 0; i < QOS_NUM_EVENTS; i++) {
		if (mpam_resctrl_counters[i] == class) {
			found = true;
			break;
		}
	}

	if (found)
		mpam_resctrl_exit();
}

static int __init __cacheinfo_ready(void)
{
	cacheinfo_ready = true;
	wake_up(&wait_cacheinfo_ready);

	return 0;
}
device_initcall_sync(__cacheinfo_ready);

#ifdef CONFIG_MPAM_KUNIT_TEST
#include "test_mpam_resctrl.c"
#endif
