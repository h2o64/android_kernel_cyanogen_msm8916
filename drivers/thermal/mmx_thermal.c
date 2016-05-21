/*
 * drivers/thermal/mmx_thermal.c
 *
 * Copyright (C) 2014-2016, Sultanxda <sultanxda@gmail.com>
 * Copyright (C) 2016, Louis Popi <theh2o64@gmail.com>
 *
 * Originally based off the MSM8x60 thermal implementation by:
 * Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "msm-thermal: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/slab.h>
#include <linux/string.h>

#define MAJOR_VERSION	1
#define MINOR_VERSION	1

#define DEFAULT_SAMPLING_MS 3000
#define UNTHROTTLE_ZONE (-1)

/* 
 * Initialize as 8 zones (big + little) I hope DT gathering will override it
 * Morever, the user must use the same amount of big and little zones
 * as 'struct thermal_zone *zone[nr_thermal_zones];' needs to have the same
 * size as the zone copied by memcpy so two things needs to be done :
 * TODO: Be able to dynamically get the number of zones for each cluster
 * and generate the strutures consequently
 * TODO: Find a way to make 'struct thermal_zone zone' in msm_thermal_main 
 * the exact same size as the copied struct
 * Additionnally, here are the features to implement :
 * - adding a battery sensor to adapt input voltage in FAN5105 port
 */

/* Numbers of zones of each type */
#define NR_THERMAL_ZONES_MAX 8 // Set to 8 the max number of per cluster zone
static uint32_t nr_thermal_zones = 0;
static uint32_t nr_thermal_zones_big = 0;
static uint32_t nr_thermal_zones_little = 0;

struct throttle_policy {
	char *arch;
	int32_t curr_zone;
	uint32_t freq;
};

struct thermal_config {
	struct qpnp_vadc_chip *vadc_dev;
	enum qpnp_vadc_channels adc_chan;
	uint8_t enabled;
	uint32_t sampling_ms;
	uint64_t max_temp;

};

struct thermal_zone {
	char *arch;
	uint32_t freq;
	int64_t trip_degC;
	int64_t reset_degC;
};

struct thermal_policy {
	spinlock_t lock;
	struct delayed_work dwork;
	struct thermal_config conf;
	struct throttle_policy throttle;
	struct thermal_zone zone[NR_THERMAL_ZONES_MAX*2];
	struct thermal_zone zone_big[NR_THERMAL_ZONES_MAX];
	struct thermal_zone zone_little[NR_THERMAL_ZONES_MAX];
	struct workqueue_struct *wq;
};

static struct thermal_policy *t_policy_g;

static void update_online_cpu_policy(void);

static void msm_thermal_main(struct work_struct *work)
{
	struct thermal_policy *t = container_of(work, typeof(*t), dwork.work);
	struct qpnp_vadc_result result;
	int32_t curr_zone, old_zone, i, ret;
	int64_t temp;
	int cpu, vote = 0;

	/* Get the VADC channel hooked up */
	ret = qpnp_vadc_read(t->conf.vadc_dev, t->conf.adc_chan, &result);
	if (ret) {
		pr_err("%s: Unable to read ADC channel\n", __func__);
		goto reschedule;
	}

	temp = result.physical;
	old_zone = t->throttle.curr_zone;

	spin_lock(&t->lock);

	for (i = 0; i < nr_thermal_zones; i++) {

		/*
		 * If we reach a very high temperature (specified by the DT)
		 * just shutdown all the little CPU (4,5,6,7) until issue 
		 * is fixed
		 */
		if (t->conf.max_temp && temp >= t->conf.max_temp) {
			for_each_present_cpu(cpu) {
				/*
				 * Only shutdown online little cores when the temperature is critical
				 * since 4x(sampling_ms)
				 */
				if (cpu >= 4 && cpu_online(cpu) && vote == 4) {
					pr_debug("%s: Critical temperature (%lld°c) reached, shutting down cpu%d.\n",
									__func__, t->conf.max_temp, cpu);
					cpu_down(cpu);
					vote = 0; // Reset vote count
				} else if (cpu >= 4 && cpu_online(cpu)) {
					vote++;
				} else {
					pr_debug("%s: No little core to stop, just wait and see ..\n", __func__);
				}
			}
			break;
		}

		if (!t->zone[i].freq) {
			/*
			 * The current thermal zone is not configured, so use
			 * the previous one and exit.
			 */
			t->throttle.curr_zone = i - 1;
			break;
		}

		if (i == (nr_thermal_zones - 1)) {
			/* Highest zone has been reached, so use it and exit */
			t->throttle.curr_zone = i;
			break;
		}

		if (temp > t->zone[i].reset_degC) {
			/*
			 * If temp is less than the trip temp for the next
			 * thermal zone and is greater than or equal to the
			 * trip temp for the current zone, then exit here and
			 * use the current index as the thermal zone.
			 * Otherwise, keep iterating until this is true (or
			 * until we hit the highest thermal zone).
			 */
			if (temp < t->zone[i + 1].trip_degC &&
				(temp >= t->zone[i].trip_degC ||
				old_zone != UNTHROTTLE_ZONE)) {
				t->throttle.curr_zone = i;
				break;
			} else if (!i && old_zone == UNTHROTTLE_ZONE) {
				/*
				 * Don't keep looping if the CPU is currently
				 * unthrottled and the temp is below the first
				 * zone's trip point.
				 */
				break;
			}
		} else if (!i) {
			/*
			 * Unthrottle CPU if temp is at or below the first
			 * zone's reset temp.
			 */
			t->throttle.curr_zone = UNTHROTTLE_ZONE;
			break;
		}
	}

	curr_zone = t->throttle.curr_zone;

	/*
	 * Update throttle freq. Setting throttle.freq to 0
	 * tells the CPU notifier to unthrottle.
	 * Setting throttle.freq to -1 tells the notfier to
	 * shutdown one CPU of the cluster.
	 */
	if (t->throttle.freq == 0)
		t->throttle.freq = -1;
	else if (curr_zone == UNTHROTTLE_ZONE)
		t->throttle.freq = 0;
	else
		t->throttle.freq = t->zone[curr_zone].freq;

	t->throttle.arch = t->zone[curr_zone].arch;
	spin_unlock(&t->lock);

	/* Only update CPU policy when the throttle zone changes */
	if (curr_zone != old_zone)
		update_online_cpu_policy();

reschedule:
	queue_delayed_work(t->wq, &t->dwork,
				msecs_to_jiffies(t->conf.sampling_ms));
}

static int do_cpu_throttle(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	struct thermal_policy *t = t_policy_g;
	uint32_t throttle_freq;
	int cpu;

	if (val != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	spin_lock(&t->lock);
	throttle_freq = t->throttle.freq;
	spin_unlock(&t->lock);

	if (t->throttle.freq == -1) {
		for_each_present_cpu(cpu) {
				if (cpu >= 4 && cpu_online(cpu))
					cpu_down(cpu);
				else
					pr_debug("%s: No little core to stop, just wait and see ..\n", __func__);
			}
	} else {
		if (throttle_freq)
			policy->max = throttle_freq;
		else
			policy->max = policy->cpuinfo.max_freq;
	}

	if (policy->min > policy->max)
		policy->min = policy->max;

	return NOTIFY_OK;
}

static struct notifier_block cpu_throttle_nb = {
	.notifier_call = do_cpu_throttle,
};

static void update_online_cpu_policy(void)
{
	uint32_t cpu;

	/* Trigger cpufreq notifier for online CPUs */
	get_online_cpus();
	for_each_online_cpu(cpu)
		cpufreq_update_policy(cpu);
	put_online_cpus();
}

#define NUM_COLS 3
static struct thermal_zone *msm_thermal_parse_zone_dt(struct device_node *np, 
				char *arch, char *prop)
{
	int len, nf, i, j;
	u32 data;
	const uint32_t *zone = NULL;
	struct thermal_zone *tbl;

	/* 
	 * Zones gathering :
	 * Syntax < freq | trip_degC | reset_degC >
	 */

	zone = of_get_property(np, prop, &len);
	if (zone == NULL) {
		pr_err("%s: No %s zones defined\n", __func__, arch);
	} else {
		len /= sizeof(data);
		nf = len / NUM_COLS;
		tbl = kzalloc((nf + 1) * sizeof(*tbl), GFP_KERNEL);

		for (i = 0, j = 0; i < nf; i++, j += 2) {
			/* Set the arch of each zone as a header */
			tbl[i].arch = arch;

			of_property_read_u32_index(np, prop, j + 1, &data);
			tbl[i].freq = data;

			of_property_read_u32_index(np, prop, j + 2, &data);
			tbl[i].trip_degC = data;

			of_property_read_u32_index(np, prop, j + 3, &data);
			tbl[i].reset_degC = data;
		}
		nr_thermal_zones = nr_thermal_zones + i;
	}
	return tbl;
}

static int msm_thermal_parse_dt(struct platform_device *pdev,
			struct thermal_policy *t)
{
	struct device_node *np = pdev->dev.of_node;
	struct thermal_zone *tmp_tbl;
	int ret, j, k, max;
	uint32_t tmp = 0;

	/* Big cluster */
	tmp = nr_thermal_zones;
	tmp_tbl = msm_thermal_parse_zone_dt(np, "big", "qcom,throttle_big_zones");
	memcpy(t->zone_big, tmp_tbl, sizeof (t->zone_big));
	nr_thermal_zones_big = tmp - nr_thermal_zones;

	/* Little cluster */
	tmp = nr_thermal_zones;
	tmp_tbl = msm_thermal_parse_zone_dt(np, "little", "qcom,throttle_little_zones");
	memcpy(t->zone_little, tmp_tbl, sizeof (t->zone_little));
	nr_thermal_zones_little = tmp - nr_thermal_zones;

	/* Copy the big zone and little zone in right order into common zone */
	if (nr_thermal_zones_big > nr_thermal_zones_little || nr_thermal_zones_big == nr_thermal_zones_little)
		max = nr_thermal_zones_big;
	else
		max = nr_thermal_zones_little;

	for (j = 0, k = 0; k <= nr_thermal_zones; k++) {
		/* If the start throttling temp of big is colder than little's one, put it before */
		if (t->zone_big[j].trip_degC < t->zone_little[j].trip_degC)
			t->zone[k] = t->zone_big[j];
		else
			t->zone[k] = t->zone_little[j];

		if (j == max)
			j = 0;
		j++; /* Next zone */
	}

	/* Set VADC sensor chanel */
	t->conf.vadc_dev = qpnp_get_vadc(&pdev->dev, "thermal");
	if (IS_ERR(t->conf.vadc_dev)) {
		ret = PTR_ERR(t->conf.vadc_dev);
		if (ret != -EPROBE_DEFER)
			pr_err("%s: VADC property missing\n", __func__);
		return ret;
	}

	ret = of_property_read_u32(np, "qcom,adc-channel", &t->conf.adc_chan);
	if (ret)
		pr_err("%s: ADC-channel property missing\n", __func__);

	/* Set max_temp value */
	ret = of_property_read_u64(np, "qcom,critical_temp", &t->conf.max_temp);
	if (ret)
		pr_err("%s: Critical temp property missing\n", __func__);

	/* Set sampling rate */
	ret = of_property_read_u32(np, "qcom,sampling_rate", &t->conf.sampling_ms);
	if (ret)
		pr_err("%s: Sampling rate property missing\n", __func__);

	return ret;
}

static struct thermal_policy *alloc_thermal_policy(void)
{
	struct thermal_policy *t;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t) {
		pr_err("%s: Failed to allocate thermal policy\n", __func__);
		return NULL;
	}

	t->wq = alloc_workqueue("msm_thermal_wq",
					WQ_HIGHPRI | WQ_NON_REENTRANT, 0);
	if (!t->wq) {
		pr_err("%s: Failed to allocate workqueue\n", __func__);
		goto free_t;
	}

	return t;

free_t:
	kfree(t);
	return NULL;
}

static int msm_thermal_probe(struct platform_device *pdev)
{
	struct thermal_policy *t;
	int ret;

	t = alloc_thermal_policy();
	if (!t)
		return -ENOMEM;

	ret = msm_thermal_parse_dt(pdev, t);
	if (ret)
		goto free_mem;

	t->conf.sampling_ms = DEFAULT_SAMPLING_MS;

	/* Boot up unthrottled */
	t->throttle.curr_zone = UNTHROTTLE_ZONE;

	/* Allow global thermal policy access */
	t_policy_g = t;

	spin_lock_init(&t->lock);

	INIT_DELAYED_WORK(&t->dwork, msm_thermal_main);

	cpufreq_register_notifier(&cpu_throttle_nb, CPUFREQ_POLICY_NOTIFIER);

	return 0;

free_mem:
	kfree(t);
	return ret;
}

static struct of_device_id msm_thermal_match_table[] = {
	{.compatible = "qcom,mmx_thermal"},
	{ },
};

static struct platform_driver msm_thermal_device = {
	.probe = msm_thermal_probe,
	.driver = {
		.name = "mmx_thermal",
		.owner = THIS_MODULE,
		.of_match_table = msm_thermal_match_table,
	},
};

static int __init msm_thermal_init(void)
{
	pr_info("mmx_thermal: version %d.%d by sultanxda and h2o64\n",
		 MAJOR_VERSION, MINOR_VERSION);
	return platform_driver_register(&msm_thermal_device);
}
device_initcall(msm_thermal_init);

