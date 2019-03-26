/*
 * Copyright (C) 2015-2019 OPPO, Inc.
 * Author: Chuck Huang <huangcheng-m@oppo.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/cpufreq.h>
#include <mt-plat/mtk_sched.h>
#include <mtk_ppm_api.h>
#include <mtk_mcdi_governor.h>
#include <sched_ctl.h>
#include <mtk_mcdi_state.h>
#include "hypnus_op.h"

extern void sched_get_nr_running_avg(int *avg, int *iowait_avg);
extern int mt_gpufreq_scene_protect(unsigned int min_freq, unsigned int max_freq);
extern unsigned int mt_gpufreq_get_dvfs_table_num(void);
extern struct g_opp_table_info *mt_gpufreq_get_dvfs_table(void);
extern unsigned int mt_gpufreq_get_min_limit_freq(void);
//extern u32 kbasep_get_gl_utilization(void);

enum g_limited_idx_enum {
	IDX_THERMAL_PROTECT_LIMITED = 0,
	IDX_LOW_BATT_LIMITED,
	IDX_BATT_PERCENT_LIMITED,
	IDX_BATT_OC_LIMITED,
	IDX_PBM_LIMITED,
	IDX_SCENE_LIMITED,
	NUMBER_OF_LIMITED_IDX,
};

enum {
	IDX_SCENE_MIN_LIMITED,
	NR_IDX_POWER_MIN_LIMITED,
};

#define MTK_SCHED_HEAVY_TASK_THRES  950 //max=1023
static int mt_get_running_avg(int *avg, int *big_avg, int *iowait)
{
	sched_get_nr_running_avg(avg, iowait);
	*big_avg = sched_get_nr_heavy_task_by_threshold(MTK_SCHED_HEAVY_TASK_THRES);
	if (*avg >= 100 * NR_CPUS)
		*avg = 100 * NR_CPUS;
	*big_avg *= 100;

	return 0;
}

static int mt_get_cpu_load(u32 cpu)
{
	return sched_get_percpu_load(cpu, 1, 1);
}

static int mt_set_boost(u32 boost)
{
	set_sched_boost(boost);
	return 0;
}

static int cpufreq_set_cpu_scaling_limit(unsigned int cpu, unsigned int min,
        unsigned int max)
{
	struct cpufreq_policy *policy;

	if (!cpu_online(cpu))
		return 0;

	policy = cpufreq_cpu_get(cpu);

	if (!policy)
		return -1;

	policy->user_policy.min = min;
	policy->user_policy.max = max;
	cpufreq_cpu_put(policy);

	return cpufreq_update_policy(cpu);
}

static int mt_set_cpu_freq_limit(u32 c_index, u32 min, u32 max)
{
	struct hypnus_data *hypdata = hypnus_get_hypdata();
	struct cpumask *pmask = NULL;
	int cpu;

	if (!hypdata)
		return -EINVAL;

	pmask = &hypdata->cluster_data[c_index].cluster_mask;
	mt_ppm_sysboost_set_freq_limit(BOOST_BY_UT, c_index, min, max);
	cpu = cpumask_first(pmask);
	if (cpufreq_set_cpu_scaling_limit(cpu, min, max) < 0) {
		pr_info("Fail to update cpu %d, min: %u, max: %u\n",
			cpu, min, max);
		return -1;
	}
	return 0;
}

static int mt_get_gpu_load(u32 gpu_index)
{
	return 0;
}

static int mt_get_gpu_freq(u32 gpu_index, unsigned int *min,
				unsigned int *cur, unsigned int *max)
{
	return 0;
}

static int mt_set_gpu_freq_limit(u32 gpu_index, u32 min_freq, u32 max_freq)
{
	struct hypnus_data *hypdata = hypnus_get_hypdata();
	int ret = 0;

	if (!hypdata || gpu_index >= hypdata->gpu_nr) {
		pr_err("GPU index %d is not supported!\n", gpu_index);
		return -EINVAL;
	}

	if (min_freq > max_freq) {
		pr_err("GPU: Invalid min_freq:%u max_freq:%u\n", min_freq, max_freq);
		return -EINVAL;
	}

	ret = mt_gpufreq_scene_protect(min_freq, max_freq);
	if (ret) {
		pr_err("GPU freq setting min_freq:%u max_freq:%u Failed\n", min_freq, max_freq);
		return ret;
	}
	return 0;
}

static int mt_set_lpm_gov(u32 type)
{
	switch (type) {
	case LPM_DEFAULT:
	set_mcdi_enable_status(true);
	set_mcdi_s_state(MCDI_STATE_SODI3);
    break;
	case LPM_USE_GOVERNOR:
	case LPM_DISABLE_SLEEP:
	set_mcdi_enable_status(false);
	break;
	default:
	pr_err("Error: Invalid decision for LPM:%d\n",type);
	break;
	}
	return 0;
}

int __weak mt_set_ddr_state(u32 state)
{
	return 0;
}

#ifdef USE_FPSGO
int __weak mt_set_fpsgo_engine(u32 enable)
{
	return 0;
}
#endif

static int mt_set_thermal_policy(bool use_default)
{
	return 0;
}

static int mt_unisolate_cpu(int cpu)
{
	return sched_deisolate_cpu(cpu);
}

static u64 mt_get_frame_cnt(void)
{
	return 0;
}

static int mt_get_display_resolution(unsigned int *xres, unsigned int *yres)
{
	return 0;
}


static struct hypnus_chipset_operations mediatek_op = {
	.name = "mediatek",
	.get_running_avg = mt_get_running_avg,
	.get_cpu_load = mt_get_cpu_load,
	.get_gpu_load = mt_get_gpu_load,
	.get_gpu_freq = mt_get_gpu_freq,
	.set_boost = mt_set_boost,
	.set_cpu_freq_limit = mt_set_cpu_freq_limit,
	.set_gpu_freq_limit = mt_set_gpu_freq_limit,
	.set_lpm_gov = mt_set_lpm_gov,
	.set_storage_scaling = NULL,
	.set_ddr_state = mt_set_ddr_state,
#ifdef USE_FPSGO
	.set_fpsgo_engine = mt_set_fpsgo_engine,
#endif
	.set_thermal_policy = mt_set_thermal_policy,
	.isolate_cpu = sched_isolate_cpu,
	.unisolate_cpu = mt_unisolate_cpu,
	.set_sched_prefer_idle = NULL,
	.get_frame_cnt = mt_get_frame_cnt,
	.get_display_resolution = mt_get_display_resolution,
};

void hypnus_chipset_op_init(struct hypnus_data *pdata)
{
	pdata->cops = &mediatek_op;
}
