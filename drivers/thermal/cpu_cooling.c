/*
 *  linux/drivers/thermal/cpu_cooling.c
 *
 *  Copyright (C) 2012	Samsung Electronics Co., Ltd(http://www.samsung.com)
 *  Copyright (C) 2012  Amit Daniel <amit.kachhap@linaro.org>
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/cpu_cooling.h>

/**
 * struct cpufreq_cooling_device
 * @id: unique integer value corresponding to each cpufreq_cooling_device
 *	registered.
 * @cool_dev: thermal_cooling_device pointer to keep track of the the
 *	egistered cooling device.
 * @cpufreq_state: integer value representing the current state of cpufreq
 *	cooling	devices.
 * @cpufreq_val: integer value representing the absolute value of the clipped
 *	frequency.
 * @allowed_cpus: all the cpus involved for this cpufreq_cooling_device.
 * @node: list_head to link all cpufreq_cooling_device together.
 *
 * This structure is required for keeping information of each
 * cpufreq_cooling_device registered as a list whose head is represented by
 * cooling_cpufreq_list. In order to prevent corruption of this list a
 * mutex lock cooling_cpufreq_lock is used.
 */
struct cpufreq_cooling_device {
	int id;
	struct thermal_cooling_device *cool_dev;
	unsigned int cpufreq_state;
	unsigned int cpufreq_val;
	struct cpumask allowed_cpus;
	struct list_head node;
};
static LIST_HEAD(cooling_cpufreq_list);
static DEFINE_IDR(cpufreq_idr);
static DEFINE_MUTEX(cooling_cpufreq_lock);

static unsigned int cpufreq_dev_count;

/* notify_table passes value to the CPUFREQ_ADJUST callback function. */
#define NOTIFY_INVALID NULL
static struct cpufreq_cooling_device *notify_device;

/**
 * get_idr - function to get a unique id.
 * @idr: struct idr * handle used to create a id.
 * @id: int * value generated by this function.
 */
static int get_idr(struct idr *idr, int *id)
{
	int err;
again:
	if (unlikely(idr_pre_get(idr, GFP_KERNEL) == 0))
		return -ENOMEM;

	mutex_lock(&cooling_cpufreq_lock);
	err = idr_get_new(idr, NULL, id);
	mutex_unlock(&cooling_cpufreq_lock);

	if (unlikely(err == -EAGAIN))
		goto again;
	else if (unlikely(err))
		return err;

	*id = *id & MAX_IDR_MASK;
	return 0;
}

/**
 * release_idr - function to free the unique id.
 * @idr: struct idr * handle used for creating the id.
 * @id: int value representing the unique id.
 */
static void release_idr(struct idr *idr, int id)
{
	mutex_lock(&cooling_cpufreq_lock);
	idr_remove(idr, id);
	mutex_unlock(&cooling_cpufreq_lock);
}

/* Below code defines functions to be used for cpufreq as cooling device */

/**
 * is_cpufreq_valid - function to check if a cpu has frequency transition policy.
 * @cpu: cpu for which check is needed.
 */
static int is_cpufreq_valid(int cpu)
{
	struct cpufreq_policy policy;
	return !cpufreq_get_policy(&policy, cpu);
}

/**
 * get_cpu_frequency - get the absolute value of frequency from level.
 * @cpu: cpu for which frequency is fetched.
 * @level: level of frequency of the CPU
 *	e.g level=1 --> 1st MAX FREQ, LEVEL=2 ---> 2nd MAX FREQ, .... etc
 */
static unsigned int get_cpu_frequency(unsigned int cpu, unsigned long level)
{
	int ret = 0, i = 0;
	unsigned long level_index;
	bool descend = false;
	struct cpufreq_frequency_table *table =
					cpufreq_frequency_get_table(cpu);
	if (!table)
		return ret;

	while (table[i].frequency != CPUFREQ_TABLE_END) {
		if (table[i].frequency == CPUFREQ_ENTRY_INVALID)
			continue;

		/*check if table in ascending or descending order*/
		if ((table[i + 1].frequency != CPUFREQ_TABLE_END) &&
			(table[i + 1].frequency < table[i].frequency)
			&& !descend) {
			descend = true;
		}

		/*return if level matched and table in descending order*/
		if (descend && i == level)
			return table[i].frequency;
		i++;
	}
	i--;

	if (level > i || descend)
		return ret;
	level_index = i - level;

	/*Scan the table in reverse order and match the level*/
	while (i >= 0) {
		if (table[i].frequency == CPUFREQ_ENTRY_INVALID)
			continue;
		/*return if level matched*/
		if (i == level_index)
			return table[i].frequency;
		i--;
	}
	return ret;
}

/**
 * cpufreq_apply_cooling - function to apply frequency clipping.
 * @cpufreq_device: cpufreq_cooling_device pointer containing frequency
 *	clipping data.
 * @cooling_state: value of the cooling state.
 */
static int cpufreq_apply_cooling(struct cpufreq_cooling_device *cpufreq_device,
				unsigned long cooling_state)
{
	unsigned int cpuid, clip_freq;
	struct cpumask *maskPtr = &cpufreq_device->allowed_cpus;
	unsigned int cpu = cpumask_any(maskPtr);


	/* Check if the old cooling action is same as new cooling action */
	if (cpufreq_device->cpufreq_state == cooling_state)
		return 0;

	clip_freq = get_cpu_frequency(cpu, cooling_state);
	if (!clip_freq)
		return -EINVAL;

	cpufreq_device->cpufreq_state = cooling_state;
	cpufreq_device->cpufreq_val = clip_freq;
	notify_device = cpufreq_device;

	for_each_cpu(cpuid, maskPtr) {
		if (is_cpufreq_valid(cpuid))
			cpufreq_update_policy(cpuid);
	}

	notify_device = NOTIFY_INVALID;

	return 0;
}

/**
 * cpufreq_thermal_notifier - notifier callback for cpufreq policy change.
 * @nb:	struct notifier_block * with callback info.
 * @event: value showing cpufreq event for which this function invoked.
 * @data: callback-specific data
 */
static int cpufreq_thermal_notifier(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned long max_freq = 0;

	if (event != CPUFREQ_ADJUST || notify_device == NOTIFY_INVALID)
		return 0;

	if (cpumask_test_cpu(policy->cpu, &notify_device->allowed_cpus))
		max_freq = notify_device->cpufreq_val;

	/* Never exceed user_policy.max*/
	if (max_freq > policy->user_policy.max)
		max_freq = policy->user_policy.max;

	if (policy->max != max_freq)
		cpufreq_verify_within_limits(policy, 0, max_freq);

	return 0;
}

/*
 * cpufreq cooling device callback functions are defined below
 */

/**
 * cpufreq_get_max_state - callback function to get the max cooling state.
 * @cdev: thermal cooling device pointer.
 * @state: fill this variable with the max cooling state.
 */
static int cpufreq_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct cpufreq_cooling_device *cpufreq_device = cdev->devdata;
	struct cpumask *maskPtr = &cpufreq_device->allowed_cpus;
	unsigned int cpu;
	struct cpufreq_frequency_table *table;
	unsigned long count = 0;
	int i = 0;

	cpu = cpumask_any(maskPtr);
	table = cpufreq_frequency_get_table(cpu);
	if (!table) {
		*state = 0;
		return 0;
	}

	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++) {
		if (table[i].frequency == CPUFREQ_ENTRY_INVALID)
			continue;
		count++;
	}

	if (count > 0) {
		*state = --count;
		return 0;
	}

	return -EINVAL;
}

/**
 * cpufreq_get_cur_state - callback function to get the current cooling state.
 * @cdev: thermal cooling device pointer.
 * @state: fill this variable with the current cooling state.
 */
static int cpufreq_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct cpufreq_cooling_device *cpufreq_device = cdev->devdata;

	*state = cpufreq_device->cpufreq_state;
	return 0;
}

/**
 * cpufreq_set_cur_state - callback function to set the current cooling state.
 * @cdev: thermal cooling device pointer.
 * @state: set this variable to the current cooling state.
 */
static int cpufreq_set_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long state)
{
	struct cpufreq_cooling_device *cpufreq_device = cdev->devdata;

	return cpufreq_apply_cooling(cpufreq_device, state);
}

/* Bind cpufreq callbacks to thermal cooling device ops */
static struct thermal_cooling_device_ops const cpufreq_cooling_ops = {
	.get_max_state = cpufreq_get_max_state,
	.get_cur_state = cpufreq_get_cur_state,
	.set_cur_state = cpufreq_set_cur_state,
};

/* Notifier for cpufreq policy change */
static struct notifier_block thermal_cpufreq_notifier_block = {
	.notifier_call = cpufreq_thermal_notifier,
};

/**
 * cpufreq_cooling_register - function to create cpufreq cooling device.
 * @clip_cpus: cpumask of cpus where the frequency constraints will happen.
 */
struct thermal_cooling_device *cpufreq_cooling_register(
	const struct cpumask *clip_cpus)
{
	struct thermal_cooling_device *cool_dev;
	struct cpufreq_cooling_device *cpufreq_dev = NULL;
	unsigned int min = 0, max = 0;
	char dev_name[THERMAL_NAME_LENGTH];
	int ret = 0, i;
	struct cpufreq_policy policy;

	/*Verify that all the clip cpus have same freq_min, freq_max limit*/
	for_each_cpu(i, clip_cpus) {
		/*continue if cpufreq policy not found and not return error*/
		if (!cpufreq_get_policy(&policy, i))
			continue;
		if (min == 0 && max == 0) {
			min = policy.cpuinfo.min_freq;
			max = policy.cpuinfo.max_freq;
		} else {
			if (min != policy.cpuinfo.min_freq ||
				max != policy.cpuinfo.max_freq)
				return ERR_PTR(-EINVAL);
		}
	}
	cpufreq_dev = kzalloc(sizeof(struct cpufreq_cooling_device),
			GFP_KERNEL);
	if (!cpufreq_dev)
		return ERR_PTR(-ENOMEM);

	cpumask_copy(&cpufreq_dev->allowed_cpus, clip_cpus);

	ret = get_idr(&cpufreq_idr, &cpufreq_dev->id);
	if (ret) {
		kfree(cpufreq_dev);
		return ERR_PTR(-EINVAL);
	}

	sprintf(dev_name, "thermal-cpufreq-%d", cpufreq_dev->id);

	cool_dev = thermal_cooling_device_register(dev_name, cpufreq_dev,
						&cpufreq_cooling_ops);
	if (!cool_dev) {
		release_idr(&cpufreq_idr, cpufreq_dev->id);
		kfree(cpufreq_dev);
		return ERR_PTR(-EINVAL);
	}
	cpufreq_dev->cool_dev = cool_dev;
	cpufreq_dev->cpufreq_state = 0;
	mutex_lock(&cooling_cpufreq_lock);

	/* Register the notifier for first cpufreq cooling device */
	if (cpufreq_dev_count == 0)
		cpufreq_register_notifier(&thermal_cpufreq_notifier_block,
						CPUFREQ_POLICY_NOTIFIER);
	cpufreq_dev_count++;

	mutex_unlock(&cooling_cpufreq_lock);
	return cool_dev;
}
EXPORT_SYMBOL(cpufreq_cooling_register);

/**
 * cpufreq_cooling_unregister - function to remove cpufreq cooling device.
 * @cdev: thermal cooling device pointer.
 */
void cpufreq_cooling_unregister(struct thermal_cooling_device *cdev)
{
	struct cpufreq_cooling_device *cpufreq_dev = cdev->devdata;

	mutex_lock(&cooling_cpufreq_lock);
	cpufreq_dev_count--;

	/* Unregister the notifier for the last cpufreq cooling device */
	if (cpufreq_dev_count == 0) {
		cpufreq_unregister_notifier(&thermal_cpufreq_notifier_block,
					CPUFREQ_POLICY_NOTIFIER);
	}
	mutex_unlock(&cooling_cpufreq_lock);

	thermal_cooling_device_unregister(cpufreq_dev->cool_dev);
	release_idr(&cpufreq_idr, cpufreq_dev->id);
	kfree(cpufreq_dev);
}
EXPORT_SYMBOL(cpufreq_cooling_unregister);
