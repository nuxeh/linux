/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/tegra-soc.h>
#include "fuse.h"

#ifndef __TEGRA12x_FUSE_OFFSETS_H
#define __TEGRA12x_FUSE_OFFSETS_H

/* private_key4 */
#define DEVKEY_START_OFFSET		0x2A
#define DEVKEY_START_BIT		12

/* arm_debug_dis */
#define JTAG_START_OFFSET		0x0
#define JTAG_START_BIT			12

/* security_mode */
#define ODM_PROD_START_OFFSET		0x0
#define ODM_PROD_START_BIT		11

/* boot_device_info */
#define SB_DEVCFG_START_OFFSET		0x2C
#define SB_DEVCFG_START_BIT		12

/* reserved_sw[2:0] */
#define SB_DEVSEL_START_OFFSET		0x2C
#define SB_DEVSEL_START_BIT		28

/* private_key0 -> private_key3 (SBK) */
#define SBK_START_OFFSET		0x22
#define SBK_START_BIT			12

/* reserved_sw[7:4] */
#define SW_RESERVED_START_OFFSET	0x2E
#define SW_RESERVED_START_BIT		0
#define SW_RESERVED_SIZE_BITS       4

/* reserved_sw[3] */
#define IGNORE_DEVSEL_START_OFFSET	0x2C
#define IGNORE_DEVSEL_START_BIT		31

/* public key */
#define PUBLIC_KEY_START_OFFSET		0x0A
#define PUBLIC_KEY_START_BIT		30

/* pkc_disable */
#define PKC_DISABLE_START_OFFSET        0x5A
#define PKC_DISABLE_START_BIT           9

/* video vp8 enable */
#define VP8_ENABLE_START_OFFSET		0x2E
#define VP8_ENABLE_START_BIT		4

/* odm lock */
#define ODM_LOCK_START_OFFSET		0x0
#define ODM_LOCK_START_BIT		6

/* reserved_odm0 -> reserved_odm7 */
#define ODM_RESERVED_DEVSEL_START_OFFSET	0x2E
#define ODM_RESERVED_START_BIT			5

/* AID */
#ifdef CONFIG_AID_FUSE
#define AID_START_OFFSET			0x72
#define AID_START_BIT				0
#endif

#define FUSE_VENDOR_CODE	0x200
#define FUSE_VENDOR_CODE_MASK	0xf
#define FUSE_FAB_CODE		0x204
#define FUSE_FAB_CODE_MASK	0x3f
#define FUSE_LOT_CODE_0		0x208
#define FUSE_LOT_CODE_1		0x20c
#define FUSE_WAFER_ID		0x210
#define FUSE_WAFER_ID_MASK	0x3f
#define FUSE_X_COORDINATE	0x214
#define FUSE_X_COORDINATE_MASK	0x1ff
#define FUSE_Y_COORDINATE	0x218
#define FUSE_Y_COORDINATE_MASK	0x1ff
#define FUSE_GPU_INFO		0x390
#define FUSE_GPU_INFO_MASK	(1<<2)
#define FUSE_SPARE_BIT		0x300
#define TEGRA_FUSE_SUPPLY	"vpp_fuse"

#define PGM_TIME_US 12

DEVICE_ATTR(public_key, 0440, tegra_fuse_show, tegra_fuse_store);
DEVICE_ATTR(pkc_disable, 0440, tegra_fuse_show, tegra_fuse_store);
DEVICE_ATTR(vp8_enable, 0440, tegra_fuse_show, tegra_fuse_store);
DEVICE_ATTR(odm_lock, 0440, tegra_fuse_show, tegra_fuse_store);

unsigned long long tegra_chip_uid(void)
{

	u64 uid = 0ull;
	u32 reg;
	u32 cid;
	u32 vendor;
	u32 fab;
	u32 lot;
	u32 wafer;
	u32 x;
	u32 y;
	u32 i;

	/* This used to be so much easier in prior chips. Unfortunately, there
	   is no one-stop shopping for the unique id anymore. It must be
	   constructed from various bits of information burned into the fuses
	   during the manufacturing process. The 64-bit unique id is formed
	   by concatenating several bit fields. The notation used for the
	   various fields is <fieldname:size_in_bits> with the UID composed
	   thusly:

	   <CID:4><VENDOR:4><FAB:6><LOT:26><WAFER:6><X:9><Y:9>

	   Where:

		Field    Bits  Position Data
		-------  ----  -------- ----------------------------------------
		CID        4     60     Chip id
		VENDOR     4     56     Vendor code
		FAB        6     50     FAB code
		LOT       26     24     Lot code (5-digit base-36-coded-decimal,
					re-encoded to 26 bits binary)
		WAFER      6     18     Wafer id
		X          9      9     Wafer X-coordinate
		Y          9      0     Wafer Y-coordinate
		-------  ----
		Total     64
	*/

	/* chip id is 3 for tegra 12x */
	cid = 3;

	vendor = tegra_fuse_readl(FUSE_VENDOR_CODE) & FUSE_VENDOR_CODE_MASK;
	fab = tegra_fuse_readl(FUSE_FAB_CODE) & FUSE_FAB_CODE_MASK;

	/* Lot code must be re-encoded from a 5 digit base-36 'BCD' number
	   to a binary number. */
	lot = 0;
	reg = tegra_fuse_readl(FUSE_LOT_CODE_0) << 2;

	for (i = 0; i < 5; ++i) {
		u32 digit = (reg & 0xFC000000) >> 26;
		BUG_ON(digit >= 36);
		lot *= 36;
		lot += digit;
		reg <<= 6;
	}

	wafer = tegra_fuse_readl(FUSE_WAFER_ID) & FUSE_WAFER_ID_MASK;
	x = tegra_fuse_readl(FUSE_X_COORDINATE) & FUSE_X_COORDINATE_MASK;
	y = tegra_fuse_readl(FUSE_Y_COORDINATE) & FUSE_Y_COORDINATE_MASK;

	uid = ((unsigned long long)cid  << 60ull)
	    | ((unsigned long long)vendor << 56ull)
	    | ((unsigned long long)fab << 50ull)
	    | ((unsigned long long)lot << 24ull)
	    | ((unsigned long long)wafer << 18ull)
	    | ((unsigned long long)x << 9ull)
	    | ((unsigned long long)y << 0ull);
	return uid;
}

int tegra_fuse_add_sysfs_variables(struct platform_device *pdev,
					bool odm_security_mode)
{
	dev_attr_odm_lock.attr.mode = 0640;
	if (odm_security_mode) {
		dev_attr_public_key.attr.mode =  0440;
		dev_attr_pkc_disable.attr.mode = 0440;
		dev_attr_vp8_enable.attr.mode = 0440;
	} else {
		dev_attr_public_key.attr.mode =  0640;
		dev_attr_pkc_disable.attr.mode = 0640;
		dev_attr_vp8_enable.attr.mode = 0640;
	}
	CHK_ERR(&pdev->dev, sysfs_create_file(&pdev->dev.kobj,
				&dev_attr_public_key.attr));
	CHK_ERR(&pdev->dev, sysfs_create_file(&pdev->dev.kobj,
				&dev_attr_pkc_disable.attr));
	CHK_ERR(&pdev->dev, sysfs_create_file(&pdev->dev.kobj,
				&dev_attr_vp8_enable.attr));
	CHK_ERR(&pdev->dev, sysfs_create_file(&pdev->dev.kobj,
				&dev_attr_odm_lock.attr));

	return 0;
}

int tegra_fuse_rm_sysfs_variables(struct platform_device *pdev)
{
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_public_key.attr);
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_pkc_disable.attr);
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_vp8_enable.attr);
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_odm_lock.attr);

	return 0;
}

int tegra_fuse_ch_sysfs_perm(struct device *dev, struct kobject *kobj)
{
	CHK_ERR(dev, sysfs_chmod_file(kobj,
				&dev_attr_public_key.attr, 0440));
	CHK_ERR(dev, sysfs_chmod_file(kobj,
				&dev_attr_pkc_disable.attr, 0440));
	CHK_ERR(dev, sysfs_chmod_file(kobj,
				&dev_attr_vp8_enable.attr, 0440));

	return 0;
}
#endif /* __TEGRA12x_FUSE_OFFSETS_H */
