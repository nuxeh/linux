/*
 * Copyright (C) 2010-2012 Google, Inc.
 * Copyright (C) 2013-2014, NVIDIA Corporation.  All rights reserved.
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
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

#ifndef __MACH_TEGRA_MC_H
#define __MACH_TEGRA_MC_H

/* !!!FIXME!!! IMPLEMENT ME */
#define tegra_mc_set_priority(client, prio) \
	do { /* nothing for now */ } while (0)

/*
 * Maximum number of unique interrupts . DT specifies the actual number.
 */
#define MC_MAX_INTR_COUNT	32

extern int mc_intr_count;

struct mc_client {
	const char *name;
	const char *swgroup;
	const int swgid;
	unsigned int intr_counts[MC_MAX_INTR_COUNT];
};

extern void __iomem *mc;
extern void __iomem *mc1;

#include <linux/io.h>
#include <linux/debugfs.h>

/*
 * This must be either 1 or 2.
 */
extern int mc_channels;

/*
 * Check if dual channel or not.
 */
static inline int mc_dual_channel(void)
{
	return mc_channels == 2;
}

/*
 * Read and write functions for hitting the MC. mc_ind corresponds to the MC
 * you wish to write to: 0 -> MC0, 1 -> MC1. If a chip does not have a
 * secondary MC then reads/writes to said MC are silently dropped.
 */
static inline u32 __mc_readl(int mc_ind, u32 reg)
{
	if (WARN(!mc, "Read before MC init'ed"))
		return 0;

	if (!mc_ind)
		return readl(mc + reg);
	else if (mc_dual_channel())
		return readl(mc1 + reg);
	else
		return 0;
}

static inline void __mc_writel(int mc_ind, u32 val, u32 reg)
{
	if (WARN(!mc, "Write before MC init'ed"))
		return;

	if (!mc_ind)
		writel(val, mc + reg);
	else if (mc_dual_channel())
		writel(val, mc1 + reg);
}

static inline u32 __mc_raw_readl(int mc_ind, u32 reg)
{
	if (WARN(!mc, "Read before MC init'ed"))
		return 0;

	if (!mc_ind)
		return __raw_readl(mc + reg);
	else if (mc_dual_channel())
		return __raw_readl(mc1 + reg);

	return 0;
}

static inline void __mc_raw_writel(int mc_ind, u32 val, u32 reg)
{
	if (WARN(!mc, "Write before MC init'ed"))
		return;

	if (!mc_ind)
		__raw_writel(val, mc + reg);
	else if (mc_dual_channel())
		__raw_writel(val, mc1 + reg);
}

#define mc_readl(reg)       __mc_readl(0, reg)
#define mc_writel(val, reg) __mc_writel(0, val, reg)

int tegra_mc_get_tiled_memory_bandwidth_multiplier(void);

/*
 * Tegra11 has dual 32-bit memory channels, while
 * Tegra12 has single 64-bit memory channel. Tegra21
 * has either dual 32 bit channels (LP4) or a single
 * 64 bit channel (LP3).
 *
 * MC effectively operates as 64-bit bus.
 */
static inline int tegra_mc_get_effective_bytes_width(void)
{
#if defined(CONFIG_ARCH_TEGRA_12x_SOC) ||     \
	defined(CONFIG_ARCH_TEGRA_11x_SOC) || \
	defined(CONFIG_ARCH_TEGRA_21x_SOC)
	return 8;
#else
	return 4;
#endif
}

unsigned long tegra_emc_bw_to_freq_req(unsigned long bw);
unsigned long tegra_emc_freq_req_to_bw(unsigned long freq);
#if defined(CONFIG_ARCH_TEGRA_12x_SOC)
void         tegra12_mc_latency_allowance_save(u32 **pctx);
void         tegra12_mc_latency_allowance_restore(u32 **pctx);
#endif
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
void         tegra21_mc_latency_allowance_save(u32 **pctx);
void         tegra21_mc_latency_allowance_restore(u32 **pctx);
#endif

/*
 * API for reading carveout info.
 */
enum carveout_desc {
	MC_SECURITY_CARVEOUT2 = 0,
	MC_NR_CARVEOUTS
};

struct mc_carveout_info {
	enum carveout_desc desc;

	u64 base;
	u64 size;
};

int mc_get_carveout_info(struct mc_carveout_info *inf, int *nr,
			 enum carveout_desc co);

/* API to get freqency switch latency at given MC freq.
 * freq_khz: Frequncy in KHz.
 * retruns latency in microseconds.
 */
static inline unsigned tegra_emc_dvfs_latency(unsigned int freq_khz)
{
	/* The latency data is not available based on freq.
	 * H/W expects it to be around 3 to 4us.
	 */
	return 4;
}

#define TEGRA_MC_CLIENT_AFI		0
#define TEGRA_MC_CLIENT_DC		2
#define TEGRA_MC_CLIENT_DCB		3
#define TEGRA_MC_CLIENT_EPP		4
#define TEGRA_MC_CLIENT_G2		5
#define TEGRA_MC_CLIENT_ISP		8
#define TEGRA_MC_CLIENT_MSENC		11
#define TEGRA_MC_CLIENT_MPE		11
#define TEGRA_MC_CLIENT_NV		12
#define TEGRA_MC_CLIENT_SATA		15
#define TEGRA_MC_CLIENT_VDE		16
#define TEGRA_MC_CLIENT_VI		17
#define TEGRA_MC_CLIENT_VIC		18
#define TEGRA_MC_CLIENT_XUSB_HOST	19
#define TEGRA_MC_CLIENT_XUSB_DEV	20
#define TEGRA_MC_CLIENT_TSEC		22
#define TEGRA_MC_CLIENT_ISPB		33
#define TEGRA_MC_CLIENT_GPU		34
#define TEGRA_MC_CLIENT_NVDEC		37
#define TEGRA_MC_CLIENT_NVJPG		40
#define TEGRA_MC_CLIENT_TSECB		45

int tegra_mc_flush(int id);
int tegra_mc_flush_done(int id);

/*
 * Necessary bit fields for various MC registers. Add to these as
 * necessary.
 */
#define MC_EMEM_ARB_MISC0_MC_EMC_SAME_FREQ_BIT			(1 << 27)

#endif
