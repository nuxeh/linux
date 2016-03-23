/*
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _LINUX_TEGRA_BPMP_H
#define _LINUX_TEGRA_BPMP_H

#include <linux/kernel.h>

#define MRQ_PING		0
#define MRQ_QUERY_TAG		1
#define MRQ_DO_IDLE		2
#define MRQ_TOLERATE_IDLE	3
#define MRQ_MODULE_LOAD		4
#define MRQ_MODULE_UNLOAD	5
#define MRQ_SWITCH_CLUSTER	6
#define MRQ_TRACE_MODIFY	7
#define MRQ_WRITE_TRACE		8
#define MRQ_THREADED_PING	9
#define MRQ_CPUIDLE_USAGE	10
#define MRQ_MODULE_MAIL		11
#define MRQ_SCX_ENABLE		12
#define MRQ_BPMPIDLE_USAGE	14
#define MRQ_HEAP_USAGE		15
#define MRQ_SCLK_SKIP_SET_RATE	16
#define MRQ_ENABLE_SUSPEND	17
#define MRQ_PASR_MASK		18
#define MRQ_DEBUGFS		19

/* Tegra PM states as known to BPMP */
#define TEGRA_PM_CC1	9
#define TEGRA_PM_CC4	12
#define TEGRA_PM_CC6	14
#define TEGRA_PM_CC7	15
#define TEGRA_PM_SC1	17
#define TEGRA_PM_SC2	18
#define TEGRA_PM_SC3	19
#define TEGRA_PM_SC4	20
#define TEGRA_PM_SC7	23

typedef void (*bpmp_mrq_handler)(int mrq, void *data, int ch);

#if defined(CONFIG_TEGRA_BPMP) && defined(CONFIG_ARCH_TEGRA_21x_SOC)
void tegra_bpmp_get_smmu_data(phys_addr_t *start, size_t *size);
#else
static inline void tegra_bpmp_get_smmu_data(phys_addr_t *start, size_t *size) {}
#endif

#ifdef CONFIG_TEGRA_BPMP
void tegra_bpmp_init_early(void);
void tegra_bpmp_trace_printk(void);
int tegra_bpmp_send(int mrq, void *data, int sz);
int tegra_bpmp_send_receive_atomic(int mrq, void *ob_data, int ob_sz,
		void *ib_data, int ib_sz);
int tegra_bpmp_send_receive(int mrq, void *ob_data, int ob_sz,
		void *ib_data, int ib_sz);
int tegra_bpmp_request_mrq(int mrq, bpmp_mrq_handler handler, void *data);
int tegra_bpmp_cancel_mrq(int mrq);
int tegra_bpmp_request_module_mrq(uint32_t module_base,
		bpmp_mrq_handler handler, void *data);
void tegra_bpmp_cancel_module_mrq(uint32_t module_base);
uint32_t tegra_bpmp_mail_readl(int ch, int offset);
void tegra_bpmp_mail_return(int ch, int code, int v);
void tegra_bpmp_mail_return_data(int ch, int code, void *data, int sz);
#else
static inline void tegra_bpmp_init_early(void) {}
static inline void tegra_bpmp_trace_printk(void) {}
static inline int tegra_bpmp_send(int mrq, void *data, int sz)
{ return -ENODEV; }
static inline int tegra_bpmp_send_receive_atomic(int mrq, void *ob_data,
		int ob_sz, void *ib_data, int ib_sz) { return -ENODEV; }
static inline int tegra_bpmp_send_receive(int mrq, void *ob_data, int ob_sz,
		void *ib_data, int ib_sz) { return -ENODEV; }
static inline int tegra_bpmp_request_mrq(int mrq, bpmp_mrq_handler handler,
		void *data) { return -ENODEV; }
static inline int tegra_bpmp_cancel_mrq(int mrq) { return -ENODEV; }
static inline int tegra_bpmp_request_module_mrq(uint32_t module_base,
		bpmp_mrq_handler handler, void *data) { return -ENODEV; }
static inline void tegra_bpmp_cancel_module_mrq(uint32_t module_base) {}
static inline uint32_t tegra_bpmp_mail_readl(int ch, int offset) { return 0; }
static inline void tegra_bpmp_mail_return(int ch, int code, int v) {}
static inline void tegra_bpmp_mail_return_data(int ch, int code,
		void *data, int sz) { }
#endif

#endif
