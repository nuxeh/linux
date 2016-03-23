/*
 * drivers/net/wireless/bcmdhd/include/dhd_custom_sysfs_tegra.h
 *
 * NVIDIA Tegra Sysfs for BCMDHD driver
 *
 * Copyright (C) 2014-2015 NVIDIA Corporation. All rights reserved.
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

#ifndef _dhd_custom_sysfs_tegra_h_
#define _dhd_custom_sysfs_tegra_h_

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/stat.h>
#include <linux/debugfs.h>
#include <linux/fs.h>

#ifndef UNUSED_PARAMETER
#define UNUSED_PARAMETER(x)	(void)(x)
#endif

/* initialization */

int
tegra_sysfs_register(struct device *dev);

void
tegra_sysfs_unregister(struct device *dev);

void
tegra_sysfs_on(void);

void
tegra_sysfs_off(void);

void
tegra_sysfs_suspend(void);

void
tegra_sysfs_resume(void);

/* ping histogram */

void
tegra_sysfs_histogram_ping_request(struct sk_buff *skb);

void
tegra_sysfs_histogram_ping_reply(struct sk_buff *skb);

void
tegra_sysfs_histogram_ping_work_start(void);

void
tegra_sysfs_histogram_ping_work_stop(void);

ssize_t
tegra_sysfs_histogram_ping_show(struct device *dev,
	struct device_attribute *attr,
	char *buf);

ssize_t
tegra_sysfs_histogram_ping_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count);

/* rssi histogram */

void
tegra_sysfs_histogram_rssi_work_start(void);

void
tegra_sysfs_histogram_rssi_work_stop(void);

ssize_t
tegra_sysfs_histogram_rssi_show(struct device *dev,
	struct device_attribute *attr,
	char *buf);

ssize_t
tegra_sysfs_histogram_rssi_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count);

/* scan histogram */

#define TEGRA_SYSFS_HISTOGRAM_SCAN_REQUEST(netdev, request, request_size)\
	{\
		char *netif = netdev ? netdev->name : "";\
		tcpdump_pkt_save('X', netif, __func__, __LINE__,\
			(const unsigned char *) request, request_size, 0);\
	}\

#define TEGRA_SYSFS_HISTOGRAM_SCAN_RESULTS(netdev, results, results_size)\
	{\
		char *netif = netdev ? netdev->name : "";\
		tcpdump_pkt_save('x', netif, __func__, __LINE__,\
			(const unsigned char *) results, results_size, 0);\
	}\

void
tegra_sysfs_histogram_scan_work_start(void);

void
tegra_sysfs_histogram_scan_work_stop(void);

ssize_t
tegra_sysfs_histogram_scan_show(struct device *dev,
	struct device_attribute *attr,
	char *buf);

ssize_t
tegra_sysfs_histogram_scan_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count);

ssize_t
tegra_debugfs_histogram_scan_read(struct file *filp,
	char __user *buff, size_t count, loff_t *offp);

ssize_t
tegra_debugfs_histogram_scan_write(struct file *filp,
	const char __user *buff, size_t count, loff_t *offp);

/* stat histogram */

void
tegra_sysfs_histogram_stat_work_run(unsigned int ms);

void
tegra_sysfs_histogram_stat_work_start(void);

void
tegra_sysfs_histogram_stat_work_stop(void);

ssize_t
tegra_sysfs_histogram_stat_show(struct device *dev,
	struct device_attribute *attr,
	char *buf);

ssize_t
tegra_sysfs_histogram_stat_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count);

/* tcpdump histogram */

void
tcpdump_pkt_save(char tag, const char *netif, const char *func, int line,
	const unsigned char *data,
	unsigned int data_nonpaged_len,
	unsigned int data_paged_len);

void
tegra_sysfs_histogram_tcpdump_rx(struct sk_buff *skb,
	const char *func, int line);

void
tegra_sysfs_histogram_tcpdump_tx(struct sk_buff *skb,
	const char *func, int line);

void
tegra_sysfs_histogram_tcpdump_work_start(void);

void
tegra_sysfs_histogram_tcpdump_work_stop(void);

ssize_t
tegra_sysfs_histogram_tcpdump_show(struct device *dev,
	struct device_attribute *attr,
	char *buf);

ssize_t
tegra_sysfs_histogram_tcpdump_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count);

ssize_t
tegra_debugfs_histogram_tcpdump_read(struct file *filp,
	char __user *buff, size_t count, loff_t *offp);

ssize_t
tegra_debugfs_histogram_tcpdump_write(struct file *filp,
	const char __user *buff, size_t count, loff_t *offp);

void
tegra_sysfs_resume_capture(void);

void
tegra_sysfs_suspend_capture(void);

void
tegra_sysfs_control_pkt(int number);

void
tegra_sysfs_dpc_pkt(void);
#endif  /* _dhd_custom_sysfs_tegra_h_ */
