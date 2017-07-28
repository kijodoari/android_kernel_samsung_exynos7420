/*
 * drivers/cpufreq/cpufreq_nexus.c
 *
 * Copyright (C) 2017 Lukas Berger
 *
 * Uses code from cpufreq_alucard and cpufreq_interactive
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include "cpufreq_governor.h"

static struct cpufreq_nexus_tunables *global_tunables = NULL;
static DEFINE_MUTEX(cpufreq_governor_nexus_mutex);

struct cpufreq_nexus_cpuinfo {
	int init;

	int cpu;
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *freq_table;

	cputime64_t prev_idle;
	cputime64_t prev_wall;

	unsigned int down_delay_counter;
	unsigned int up_delay_counter;

	struct delayed_work work;
	struct mutex timer_mutex;
};

struct cpufreq_nexus_tunables {
	// load at which the cpugov decides to scale down
	#define DEFAULT_DOWN_LOAD 80
	unsigned int down_load;

	// delay in timer-ticks to scale down CPU
	#define DEFAULT_DOWN_DELAY 0
	unsigned int down_delay;

	// frequency-steps if cpugov scales down
	#define DEFAULT_DOWN_STEP 2
	unsigned int down_step;

	// load at which the cpugov decides to scale up
	#define DEFAULT_UP_LOAD 90
	unsigned int up_load;

	// delay in timer-ticks to scale up CPU
	#define DEFAULT_UP_DELAY 2
	unsigned int up_delay;

	// frequency-steps if cpugov scales up
	#define DEFAULT_UP_STEP 1
	unsigned int up_step;

	// interval of the scaling-timer
	#define DEFAULT_TIMER_RATE 20000
	unsigned int timer_rate;

	// indicates if I/O-time should be added to cputime
	#define DEFAULT_IO_IS_BUSY 1
	int io_is_busy;

	// minimal frequency chosen by the cpugov
	unsigned int freq_min;
	int freq_min_do_revalidate;

	// maximal frequency chosen by the cpugov
	unsigned int freq_max;
	int freq_max_do_revalidate;

	// frequency used when governor is in boost-mode
	unsigned int freq_boost;
	int freq_boost_do_revalidate;

	// simple boost to freq_max
	#define DEFAULT_BOOST 0
	int boost;

	// time in usecs when current boostpulse ends
	u64 boostpulse;

	// ktime when current boostpulse ends
	u64 boostpulse_end;

	// time in usecs when current boostpulse ends
	#define DEFAULT_BOOSTPULSE_DURATION 50000
	u64 boostpulse_duration;

	// determines if the power-efficient frequency-selection should be used
	#define DEFAULT_POWER_EFFICIENT 1
	int power_efficient;

	// used frequency-coefficient to support SoCs with a non-linear frequency-tables
	#define DEFAULT_NON_LINEAR_FREQUENCY_SCALER 108000
	int non_linear_frequency_scaler;
};

static DEFINE_PER_CPU(struct cpufreq_nexus_cpuinfo, gov_cpuinfo);

static unsigned int choose_frequency(struct cpufreq_nexus_cpuinfo *cpuinfo, int *index, unsigned int base_freq) {
	struct cpufreq_policy *policy;
	struct cpufreq_nexus_tunables *tunables;

	// checks have already been done in cpufreq_nexus_timer
	policy = cpuinfo->policy;
	tunables = policy->governor_data;

	if (tunables->power_efficient) {
		cpufreq_frequency_table_target(policy, cpuinfo->freq_table, base_freq,
			CPUFREQ_RELATION_C, index);
	} else {
		cpufreq_frequency_table_target(policy, cpuinfo->freq_table, base_freq,
			CPUFREQ_RELATION_L, index);
		if (cpuinfo->freq_table[*index].frequency != policy->cur) {
			cpufreq_frequency_table_target(policy, cpuinfo->freq_table, base_freq,
				CPUFREQ_RELATION_C, index);
		}
	}

	return cpuinfo->freq_table[*index].frequency;
}

static void cpufreq_nexus_timer(struct work_struct *work)
{
	struct cpufreq_nexus_cpuinfo *cpuinfo;
	struct cpufreq_policy *policy;
	struct cpufreq_nexus_tunables *tunables;
	int delay, real_delay, cpu, load;
	unsigned int index = 0;
	unsigned int freq = 0,
				 next_freq = 0;
	unsigned int ktime_now = ktime_to_us(ktime_get());
	cputime64_t curr_idle, curr_wall, idle, wall;

	cpuinfo = container_of(work, struct cpufreq_nexus_cpuinfo, work.work);
	if (!cpuinfo)
		return;

	policy = cpuinfo->policy;
	if (!policy)
		return;

	tunables = policy->governor_data;
	cpu = cpuinfo->cpu;

	if (mutex_lock_interruptible(&cpuinfo->timer_mutex))
		return;

	if (!cpu_online(cpu))
		goto exit;

	// calculate new load
	curr_idle = get_cpu_idle_time(cpu, &curr_wall, tunables->io_is_busy);
	idle = (curr_idle - cpuinfo->prev_idle);
	wall = (curr_wall - cpuinfo->prev_wall);

	cpuinfo->prev_idle = curr_idle;
	cpuinfo->prev_wall = curr_wall;

	if (cpuinfo->init) {
		// apply the current cputimes and skip this sample
		cpuinfo->init = 0;
		goto requeue;
	}
	
	// revalidate custom frequencies
	if (tunables->freq_min_do_revalidate) {
		tunables->freq_min = choose_frequency(cpuinfo, &index, tunables->freq_min);
		tunables->freq_min_do_revalidate = 0;
	}
	if (tunables->freq_max_do_revalidate) {
		tunables->freq_max = choose_frequency(cpuinfo, &index, tunables->freq_max);
		tunables->freq_max_do_revalidate = 0;
	}
	if (tunables->freq_boost_do_revalidate) {
		tunables->freq_boost = choose_frequency(cpuinfo, &index, tunables->freq_boost);
		tunables->freq_boost_do_revalidate = 0;
	}

	// calculate frequencies
	freq = policy->cur;

	if (wall >= idle) {
		load = (wall > idle ? (100 * (wall - idle)) / wall : 0);

		if (tunables->boost || ktime_now < tunables->boostpulse_end) {
			freq = tunables->freq_boost;
		} else {
			if (load >= tunables->up_load) {
				if (tunables->up_delay == 0 || cpuinfo->up_delay_counter >= tunables->up_delay) {
					freq = min(policy->cur + (tunables->up_step * tunables->non_linear_frequency_scaler), policy->max);
					cpuinfo->up_delay_counter = 0;
				} else if (tunables->up_delay > 0) {
					cpuinfo->up_delay_counter++;
				}
				cpuinfo->down_delay_counter = 0;
			} else if (load <= tunables->down_load) {
				if (tunables->down_delay == 0 || cpuinfo->down_delay_counter >= tunables->down_delay) {
					freq = max(policy->cur - (tunables->down_step * tunables->non_linear_frequency_scaler), policy->min);
					cpuinfo->down_delay_counter = 0;
				} else if (tunables->down_delay > 0) {
					cpuinfo->down_delay_counter++;
				}
				cpuinfo->up_delay_counter = 0;
			} else {
				cpuinfo->up_delay_counter = 0;
				cpuinfo->down_delay_counter = 0;
			}

			// apply tunables
			if (next_freq < tunables->freq_min)
				next_freq = max(policy->min, tunables->freq_min);

			if (next_freq > tunables->freq_max)
				next_freq = min(policy->max, tunables->freq_max);
		}

		// choose frequency
		next_freq = choose_frequency(cpuinfo, &index, freq);

		if (next_freq != policy->cur) {
			__cpufreq_driver_target(policy, next_freq, CPUFREQ_RELATION_C);
		}
	}

	// requeue work-timer
requeue:
	delay = usecs_to_jiffies(tunables->timer_rate < 1000 ? 1000 : tunables->timer_rate);
	real_delay = delay;
	if (num_online_cpus() > 1) {
		delay -= jiffies % delay;
	}

	queue_delayed_work_on(cpu, system_wq, &cpuinfo->work, delay);

exit:
	mutex_unlock(&cpuinfo->timer_mutex);
}

#define gov_show_store(_name) \
	gov_show(_name);          \
	gov_store(_name)

#define gov_sys_pol_show_store(_name)                                         \
	gov_sys_show(_name);                                                      \
	gov_sys_store(_name);                                                     \
	gov_pol_show(_name);                                                      \
	gov_pol_store(_name);                                                     \
	static struct global_attr _name##_gov_sys =                               \
		__ATTR(_name, 0666, show_##_name##_gov_sys, store_##_name##_gov_sys); \
	static struct freq_attr _name##_gov_pol =                                 \
		__ATTR(_name, 0666, show_##_name##_gov_pol, store_##_name##_gov_pol)

#define gov_sys_pol_store(_name)                            \
	gov_sys_store(_name);                                   \
	gov_pol_store(_name);                                   \
	static struct global_attr _name##_gov_sys =             \
		__ATTR(_name, 0666, NULL, store_##_name##_gov_sys); \
	static struct freq_attr _name##_gov_pol =               \
		__ATTR(_name, 0666, NULL, store_##_name##_gov_pol)

/*
 * Show-Macros
 */
#define gov_show(_name)                                                 \
static ssize_t show_##_name                                             \
(struct cpufreq_nexus_tunables *tunables, char *buf)                    \
{                                                                       \
	return sprintf(buf, "%llu\n", (unsigned long long)tunables->_name); \
}

#define gov_sys_show(_name)                               \
static ssize_t show_##_name##_gov_sys                     \
(struct kobject *kobj, struct attribute *attr, char *buf) \
{                                                         \
	return show_##_name(global_tunables, buf);          \
}

#define gov_pol_show(_name)                                                             \
static ssize_t show_##_name##_gov_pol                                                   \
(struct cpufreq_policy *policy, char *buf)                                              \
{                                                                                       \
	return show_##_name((struct cpufreq_nexus_tunables *)policy->governor_data, buf); \
}

/*
 * Store-Macros
 */
#define gov_store(_name)                                                 \
static ssize_t store_##_name                                             \
(struct cpufreq_nexus_tunables *tunables, const char *buf, size_t count) \
{                                                                        \
	unsigned long val = 0;                                               \
	int ret = kstrtoul(buf, 0, &val);                                    \
	if (ret < 0)                                                         \
		return ret;                                                      \
	tunables->_name = val;                                               \
	return count;                                                        \
}

#define gov_sys_store(_name)                                                  \
static ssize_t store_##_name##_gov_sys                                        \
(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count) \
{                                                                             \
	return store_##_name(global_tunables, buf, count);                      \
}

#define gov_pol_store(_name)                                                                    \
static ssize_t store_##_name##_gov_pol                                                          \
(struct cpufreq_policy *policy, const char *buf, size_t count)                                  \
{                                                                                               \
	return store_##_name((struct cpufreq_nexus_tunables *)policy->governor_data, buf, count); \
}

static ssize_t show_freq_min(struct cpufreq_nexus_tunables *tunables, char *buf)
{
	return sprintf(buf, "%u\n", tunables->freq_min);
}

static ssize_t store_freq_min(struct cpufreq_nexus_tunables *tunables, const char *buf, size_t count)  {
	unsigned long val = 0;
	int ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	tunables->freq_min = val;
	tunables->freq_min_do_revalidate = 1;

	return count;
}

static ssize_t show_freq_max(struct cpufreq_nexus_tunables *tunables, char *buf)
{
	return sprintf(buf, "%u\n", tunables->freq_max);
}

static ssize_t store_freq_max(struct cpufreq_nexus_tunables *tunables, const char *buf, size_t count)  {
	unsigned long val = 0;
	int ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	tunables->freq_max = val;
	tunables->freq_max_do_revalidate = 1;

	return count;
}

static ssize_t show_freq_boost(struct cpufreq_nexus_tunables *tunables, char *buf)
{
	return sprintf(buf, "%u\n", tunables->freq_boost);
}

static ssize_t store_freq_boost(struct cpufreq_nexus_tunables *tunables, const char *buf, size_t count)  {
	unsigned long val = 0;
	int ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	tunables->freq_boost = val;
	tunables->freq_boost_do_revalidate = 1;

	return count;
}

static ssize_t store_boostpulse(struct cpufreq_nexus_tunables *tunables, const char *buf, size_t count)  {
	unsigned long val = 0;
	int ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	tunables->boostpulse_end = ktime_to_us(ktime_get()) + tunables->boostpulse_duration;
	return count;
}

gov_show_store(down_load);
gov_show_store(down_delay);
gov_show_store(down_step);
gov_show_store(up_load);
gov_show_store(up_delay);
gov_show_store(up_step);
gov_show_store(timer_rate);
gov_show_store(io_is_busy);
gov_show_store(boost);
gov_show_store(boostpulse_duration);
gov_show_store(power_efficient);
gov_show_store(non_linear_frequency_scaler);

gov_sys_pol_show_store(down_load);
gov_sys_pol_show_store(down_delay);
gov_sys_pol_show_store(down_step);
gov_sys_pol_show_store(up_load);
gov_sys_pol_show_store(up_delay);
gov_sys_pol_show_store(up_step);
gov_sys_pol_show_store(timer_rate);
gov_sys_pol_show_store(io_is_busy);
gov_sys_pol_show_store(freq_min);
gov_sys_pol_show_store(freq_max);
gov_sys_pol_show_store(freq_boost);
gov_sys_pol_show_store(boost);
gov_sys_pol_store(boostpulse);
gov_sys_pol_show_store(boostpulse_duration);
gov_sys_pol_show_store(power_efficient);
gov_sys_pol_show_store(non_linear_frequency_scaler);

static struct attribute *attributes_gov_sys[] = {
	&down_load_gov_sys.attr,
	&down_delay_gov_sys.attr,
	&down_step_gov_sys.attr,
	&up_load_gov_sys.attr,
	&up_delay_gov_sys.attr,
	&up_step_gov_sys.attr,
	&timer_rate_gov_sys.attr,
	&io_is_busy_gov_sys.attr,
	&freq_min_gov_sys.attr,
	&freq_max_gov_sys.attr,
	&freq_boost_gov_sys.attr,
	&boost_gov_sys.attr,
	&boostpulse_gov_sys.attr,
	&boostpulse_duration_gov_sys.attr,
	&power_efficient_gov_sys.attr,
	&non_linear_frequency_scaler_gov_sys.attr,
	NULL // NULL has to be terminating entry
};

static struct attribute_group attribute_group_gov_sys = {
	.attrs = attributes_gov_sys,
	.name = "nexus",
};

static struct attribute *attributes_gov_pol[] = {
	&down_load_gov_pol.attr,
	&down_delay_gov_pol.attr,
	&down_step_gov_pol.attr,
	&up_load_gov_pol.attr,
	&up_delay_gov_pol.attr,
	&up_step_gov_pol.attr,
	&timer_rate_gov_pol.attr,
	&io_is_busy_gov_pol.attr,
	&freq_min_gov_pol.attr,
	&freq_max_gov_pol.attr,
	&freq_boost_gov_pol.attr,
	&boost_gov_pol.attr,
	&boostpulse_gov_pol.attr,
	&boostpulse_duration_gov_pol.attr,
	&power_efficient_gov_pol.attr,
	&non_linear_frequency_scaler_gov_pol.attr,
	NULL // NULL has to be terminating entry
};

static struct attribute_group attribute_group_gov_pol = {
	.attrs = attributes_gov_pol,
	.name = "nexus",
};

static struct attribute_group *get_attribute_group(void) {
	if (have_governor_per_policy())
		return &attribute_group_gov_pol;
	else
		return &attribute_group_gov_sys;
}

static int cpufreq_governor_nexus(struct cpufreq_policy *policy, unsigned int event) {
	int rc = 0;
	int cpu, delay, work_cpu;
	struct cpufreq_nexus_cpuinfo *cpuinfo;
	struct cpufreq_nexus_tunables *tunables;

	cpu = policy->cpu;

	switch (event) {
		case CPUFREQ_GOV_POLICY_INIT:
			pr_info("%s: received CPUFREQ_GOV_POLICY_INIT for cpu%d\n", __func__, cpu);
			mutex_lock(&cpufreq_governor_nexus_mutex);

			if (!have_governor_per_policy()) {
				tunables = global_tunables;
			}

			tunables = kzalloc(sizeof(struct cpufreq_nexus_tunables), GFP_KERNEL);
			if (!tunables) {
				pr_err("%s: POLICY_INIT: kzalloc failed\n", __func__);
				mutex_unlock(&cpufreq_governor_nexus_mutex);
				return -ENOMEM;
			}

			tunables->down_load = DEFAULT_DOWN_LOAD;
			tunables->down_delay = DEFAULT_DOWN_DELAY;
			tunables->down_step = DEFAULT_DOWN_STEP;
			tunables->up_load = DEFAULT_UP_LOAD;
			tunables->up_delay = DEFAULT_UP_DELAY;
			tunables->up_step = DEFAULT_UP_STEP;
			tunables->timer_rate = DEFAULT_TIMER_RATE;
			tunables->io_is_busy = DEFAULT_IO_IS_BUSY;
			tunables->freq_min = policy->min;
			tunables->freq_max = policy->max;
			tunables->freq_boost = policy->max;
			tunables->boost = DEFAULT_BOOST;
			tunables->boostpulse = 0;
			tunables->boostpulse_duration = DEFAULT_BOOSTPULSE_DURATION;
			tunables->power_efficient = DEFAULT_POWER_EFFICIENT;
			tunables->non_linear_frequency_scaler = DEFAULT_NON_LINEAR_FREQUENCY_SCALER;

			rc = sysfs_create_group(get_governor_parent_kobj(policy), get_attribute_group());
			if (rc) {
				pr_err("%s: POLICY_INIT: sysfs_create_group failed\n", __func__);
				kfree(tunables);
				mutex_unlock(&cpufreq_governor_nexus_mutex);
				return rc;
			}

			policy->governor_data = tunables;

			mutex_unlock(&cpufreq_governor_nexus_mutex);

			break;

		case CPUFREQ_GOV_POLICY_EXIT:
			pr_info("%s: received CPUFREQ_GOV_POLICY_EXIT for cpu%d\n", __func__, cpu);
			cpuinfo = &per_cpu(gov_cpuinfo, cpu);
			tunables = policy->governor_data;

			mutex_lock(&cpufreq_governor_nexus_mutex);
			sysfs_remove_group(get_governor_parent_kobj(policy), get_attribute_group());

			if (tunables)
				kfree(tunables);

			mutex_unlock(&cpufreq_governor_nexus_mutex);

			break;

		case CPUFREQ_GOV_START:
			pr_info("%s: received CPUFREQ_GOV_START for cpu%d\n", __func__, cpu);
			if (!cpu_online(cpu) || !policy->cur)
				return -EINVAL;

			mutex_lock(&cpufreq_governor_nexus_mutex);

			for_each_cpu(work_cpu, policy->cpus) {

				cpuinfo = &per_cpu(gov_cpuinfo, work_cpu);
				tunables = policy->governor_data;

				cpuinfo->cpu = work_cpu;
				cpuinfo->freq_table = cpufreq_frequency_get_table(cpu);
				cpuinfo->policy = policy;
				cpuinfo->init = 1;

				mutex_init(&cpuinfo->timer_mutex);

				delay = usecs_to_jiffies(tunables->timer_rate);
				if (num_online_cpus() > 1) {
					delay -= jiffies % delay;
				}

				INIT_DEFERRABLE_WORK(&cpuinfo->work, cpufreq_nexus_timer);
				queue_delayed_work_on(work_cpu, system_wq, &cpuinfo->work, delay);

			};

			mutex_unlock(&cpufreq_governor_nexus_mutex);

			break;

		case CPUFREQ_GOV_STOP:
			pr_info("%s: received CPUFREQ_GOV_STOP for cpu%d\n", __func__, cpu);

			for_each_cpu(work_cpu, policy->cpus) {

				cpuinfo = &per_cpu(gov_cpuinfo, work_cpu);
				tunables = policy->governor_data;

				cancel_delayed_work_sync(&cpuinfo->work);

			}

			break;

		case CPUFREQ_GOV_LIMITS:
			pr_info("%s: received CPUFREQ_GOV_LIMITS for cpu%d\n", __func__, cpu);
			mutex_lock(&cpufreq_governor_nexus_mutex);

			cpuinfo = &per_cpu(gov_cpuinfo, cpu);
			if (policy->max < cpuinfo->policy->cur) {
				__cpufreq_driver_target(cpuinfo->policy,
					policy->max, CPUFREQ_RELATION_H);
			} else if (policy->min > cpuinfo->policy->cur) {
				__cpufreq_driver_target(cpuinfo->policy,
					policy->min, CPUFREQ_RELATION_L);
			}

			mutex_unlock(&cpufreq_governor_nexus_mutex);

			break;
	}

	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_NEXUS
static
#endif
struct cpufreq_governor cpufreq_gov_nexus = {
	.name = "nexus",
	.governor = cpufreq_governor_nexus,
	.owner = THIS_MODULE,
};

static int __init cpufreq_nexus_init(void) {
	return cpufreq_register_governor(&cpufreq_gov_nexus);
}

static void __exit cpufreq_nexus_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_nexus);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_NEXUS
fs_initcall(cpufreq_nexus_init);
#else
module_init(cpufreq_nexus_init);
#endif
module_exit(cpufreq_nexus_exit);

MODULE_AUTHOR("Lukas Berger <mail@lukasberger.at>");
MODULE_DESCRIPTION("'cpufreq_nexus' - cpufreq-governor for interactive "
	"and battery-based devices");
MODULE_LICENSE("GPL");
