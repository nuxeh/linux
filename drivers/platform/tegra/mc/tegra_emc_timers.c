/*
 * Copyright (C) 2013-2014 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * This file has various timers that may or may not be running in the
 * EMC.
 */

#define pr_fmt(fmt)	"[dram-timers] " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/seq_file.h>
#include <linux/thermal.h>
#include <linux/timer.h>
#include <linux/atomic.h>

#include <mach/tegra_emc.h>

#define TEGRA_DRAM_THERM_MAX_STATE     1

/* In ms - timer periods. */
static u32 timer_period_mr4 = 1000;
static u32 timer_period_training = 100;

/* MR4 specific. */
static atomic_t mr4_do_poll;
static u32 prev_temp = 0xffffffff; /* i.e -1. */
static u32 test_mode;
static int dram_temp_override;

static void emc_mr4_poll(unsigned long nothing);
static void emc_train(unsigned long nothing);

/* The timers themselves. */
static struct timer_list emc_timer_mr4 =
	TIMER_DEFERRED_INITIALIZER(emc_mr4_poll, 0, 0);
static struct timer_list emc_timer_training =
	TIMER_DEFERRED_INITIALIZER(emc_train, 0, 0);

static void emc_mr4_poll(unsigned long nothing)
{
	int dram_temp;

	if (!test_mode)
		dram_temp = tegra_emc_get_dram_temperature();
	else
		dram_temp = dram_temp_override;

	if (prev_temp == dram_temp)
		goto reset;

	if (WARN(dram_temp < 0, "Unable to read DRAM temp (MR4)!\n"))
		goto reset;

	switch (dram_temp) {
	case 0:
	case 1:
	case 2:
	case 3:
		/*
		 * Temp is fine - go back to regular refresh.
		 */
		pr_info("Setting nominal refresh + timings.\n");
		tegra_emc_set_over_temp_state(DRAM_OVER_TEMP_NONE);
		break;
	case 4:
		pr_info("Enabling 2x refresh.\n");
		tegra_emc_set_over_temp_state(DRAM_OVER_TEMP_REFRESH_X2);
		break;
	case 5:
		pr_info("Enabling 4x refresh.\n");
		tegra_emc_set_over_temp_state(DRAM_OVER_TEMP_REFRESH_X4);
		break;
	case 6:
		pr_info("Enabling 4x refresh + derating.\n");
		tegra_emc_set_over_temp_state(DRAM_OVER_TEMP_THROTTLE);
		break;
	default:
		WARN(1, "%s: Invalid DRAM temp state %d\n",
		     __func__, dram_temp);
		break;
	}
	prev_temp = dram_temp;

reset:
	if (atomic_read(&mr4_do_poll) == 0)
		return;

	if (mod_timer(&emc_timer_mr4,
		      jiffies + msecs_to_jiffies(timer_period_mr4)))
		pr_err("Failed to restart timer!!!\n");
}

/*
 * Tell the dram thermal driver to start polling for the DRAM temperature. This
 * should be invoked when there is reason to believe the DRAM temperature is
 * high.
 */
void tegra_emc_timer_mr4_start(void)
{
	atomic_set(&mr4_do_poll, 1);
	mod_timer(&emc_timer_mr4,
		  jiffies + msecs_to_jiffies(timer_period_mr4));
}

/*
 * Stop the DRAM thermal driver from polling for the DRAM temperature. If there
 * is no reason to expect the DRAM to be very hot then there is no reason to
 * poll for the DRAM's temperature.
 */
void tegra_emc_timer_mr4_stop(void)
{
	atomic_set(&mr4_do_poll, 0);
}

static void emc_train(unsigned long nothing)
{
	emc_do_periodic_compensation();

	mod_timer(&emc_timer_training,
		  jiffies + msecs_to_jiffies(timer_period_training));
}

void tegra_emc_timer_training_start(void)
{
	mod_timer(&emc_timer_training,
		  jiffies + msecs_to_jiffies(timer_period_training));
}

void tegra_emc_timer_training_stop(void)
{
	del_timer(&emc_timer_training);
}

static int tegra_dram_cd_max_state(struct thermal_cooling_device *tcd,
				   unsigned long *state)
{
	*state = TEGRA_DRAM_THERM_MAX_STATE;
	return 0;
}

static int tegra_dram_cd_cur_state(struct thermal_cooling_device *tcd,
				   unsigned long *state)
{
	*state = (unsigned long)atomic_read(&mr4_do_poll);
	return 0;
}

static int tegra_dram_cd_set_state(struct thermal_cooling_device *tcd,
				   unsigned long state)
{
	if (state == (unsigned long)atomic_read(&mr4_do_poll))
		return 0;

	if (state)
		tegra_emc_timer_mr4_start();
	else
		tegra_emc_timer_mr4_stop();
	return 0;
}

/*
 * Cooling device support.
 */
static struct thermal_cooling_device_ops emc_dram_cd_ops = {
	.get_max_state = tegra_dram_cd_max_state,
	.get_cur_state = tegra_dram_cd_cur_state,
	.set_cur_state = tegra_dram_cd_set_state,
};

#ifdef CONFIG_DEBUG_FS
static struct dentry *emc_timers_debugfs;

static int __get_mr4_do_poll(void *data, u64 *val)
{
	*val = atomic_read(&mr4_do_poll);

	return 0;
}
static int __set_mr4_do_poll(void *data, u64 val)
{
	atomic_set(&mr4_do_poll, (unsigned int)val);

	/* Explicitly wake up the DRAM monitoring thread. */
	if (atomic_read(&mr4_do_poll))
		tegra_emc_timer_mr4_start();

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(mr4_do_poll_fops, __get_mr4_do_poll, __set_mr4_do_poll,
								     "%llu\n");
#endif

int tegra_emc_timers_init(struct dentry *parent)
{
#ifdef CONFIG_DEBUG_FS
	struct dentry *dram_therm_debugfs;
	struct dentry *training_debugfs;

	/*
	 * If caller doesn't want us to make a debugfs dir it can pass NULL.
	 * This should never be a top level debugfs dir. For subsequent debugfs
	 * errors return 0 - don't fail the entire EMC driver init if debugfs
	 * has problems. We do a best effort here.
	 */
	if (!parent)
		return 0;

	emc_timers_debugfs = debugfs_create_dir("emc_timers", parent);
	if (!emc_timers_debugfs)
		return 0;

	dram_therm_debugfs = debugfs_create_dir("dram_therm",
						emc_timers_debugfs);
	if (!dram_therm_debugfs)
		return 0;

	/* DRAM thermals. */
	debugfs_create_u32("timer_period", S_IRUGO | S_IWUSR,
			   dram_therm_debugfs, &timer_period_mr4);

	debugfs_create_u32("test_mode", S_IRUGO | S_IWUSR,
			   dram_therm_debugfs, &test_mode);
	debugfs_create_u32("dram_temp_override", S_IRUGO | S_IWUSR,
			   dram_therm_debugfs, &dram_temp_override);
	debugfs_create_file("do_poll", S_IRUGO | S_IWUSR,
			    dram_therm_debugfs, NULL, &mr4_do_poll_fops);

	if (tegra_emc_get_dram_type() == DRAM_TYPE_LPDDR4) {
		/* Training. */
		training_debugfs = debugfs_create_dir("training", emc_timers_debugfs);
		if (!training_debugfs)
			return 0;

		debugfs_create_u32("timer_period", S_IRUGO | S_IWUSR,
				   training_debugfs, &timer_period_training);
	}
#endif
	return 0;
}

/*
 * This is separated from the generic tegra_emc_timers_init() since this must
 * run later than the regular EMC init runs. How annoying. Thankfully this has
 * no dependency on the debugfs or timer portion of this code.
 */
static __init int tegra_emc_therm_init(void)
{
	void *ret;

	ret = thermal_cooling_device_register("tegra-dram", NULL,
					      &emc_dram_cd_ops);
	if (IS_ERR(ret))
		return PTR_ERR(ret);
	if (ret == NULL)
		return -ENODEV;

	pr_info("DRAM derating cdev registered.\n");

	return 0;
}
late_initcall(tegra_emc_therm_init);
