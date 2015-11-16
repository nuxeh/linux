/*
 * Generic DWMAC platform driver
 *
 * Copyright (C) 2007-2011  STMicroelectronics Ltd
 * Copyright (C) 2015 Joachim Eastwood <manabian@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "stmmac.h"
#include "stmmac_platform.h"

static int dwmac_generic_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct clk *clk;
	int ret;

	clk = clk_get(&pdev->dev, NULL);
	if (!IS_ERR(clk)) {
		dev_info(&pdev->dev, "enabling clock...\n");
		clk_prepare_enable(clk);
	} else
		dev_err(&pdev->dev, "no clock found\n");

	if (1) {
		void __iomem *reg = ioremap(0xC0004144, 4);

		BUG_ON(!reg);
		writel(readl(reg) | (1<<3), reg);
		iounmap(reg);
		mdelay(100);
	}

	if (0) {
		void __iomem *reg = ioremap(0xC0004408, 4);

		BUG_ON(!reg);
		pr_info("checking resetn\n");
		pr_info("%s: reg %p was %08x\n", __func__, reg, readl(reg));
		writel(readl(reg) | (1<<11), reg);
		pr_info("%s: reg %p now %08x\n", __func__, reg, readl(reg));	
		iounmap(reg);
	}

	if (1) {
		void __iomem *reg = ioremap(0xC0004100, 4);

		BUG_ON(!reg);
		pr_info("checking MEM_PD_REG0\n");
		pr_info("%s: reg %p was %08x\n", __func__, reg, readl(reg));
		writel(readl(reg) & ~(3 << 2), reg);
		pr_info("%s: reg %p now %08x\n", __func__, reg, readl(reg));	
		iounmap(reg);
	}		

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	if (pdev->dev.of_node) {
		plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
		if (IS_ERR(plat_dat)) {
			dev_err(&pdev->dev, "dt configuration failed\n");
			return PTR_ERR(plat_dat);
		}
	} else {
		plat_dat = dev_get_platdata(&pdev->dev);
		if (!plat_dat) {
			dev_err(&pdev->dev, "no platform data provided\n");
			return  -EINVAL;
		}

		/* Set default value for multicast hash bins */
		plat_dat->multicast_filter_bins = HASH_TABLE_SIZE;

		/* Set default value for unicast filter entries */
		plat_dat->unicast_filter_entries = 1;
	}

	/* Custom initialisation (if needed) */
	if (plat_dat->init) {
		ret = plat_dat->init(pdev, plat_dat->bsp_priv);
		if (ret)
			return ret;
	}

	return stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
}

static const struct of_device_id dwmac_generic_match[] = {
	{ .compatible = "amlogic,meson8b-dwmac" },
	{ .compatible = "st,spear600-gmac"},
	{ .compatible = "snps,dwmac-3.610"},
	{ .compatible = "snps,dwmac-3.70a"},
	{ .compatible = "snps,dwmac-3.710"},
	{ .compatible = "snps,dwmac"},
	{ }
};
MODULE_DEVICE_TABLE(of, dwmac_generic_match);

static struct platform_driver dwmac_generic_driver = {
	.probe  = dwmac_generic_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name           = STMMAC_RESOURCE_NAME,
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = of_match_ptr(dwmac_generic_match),
	},
};
module_platform_driver(dwmac_generic_driver);

MODULE_DESCRIPTION("Generic dwmac driver");
MODULE_LICENSE("GPL v2");
