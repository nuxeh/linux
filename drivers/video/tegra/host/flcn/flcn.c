/*
* Tegra flcn common driver
*
* Copyright (c) 2011-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>         /* for kzalloc */
#include <asm/byteorder.h>      /* for parsing ucode image wrt endianness */
#include <linux/delay.h>	/* for udelay */
#include <linux/export.h>
#include <linux/scatterlist.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <linux/tegra-powergate.h>
#include <linux/tegra-soc.h>
#include <linux/tegra_pm_domains.h>

#include "dev.h"
#include "class_ids.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "nvhost_vm.h"
#include "nvhost_scale.h"
#include "nvhost_channel.h"

#include "flcn.h"
#include "hw_flcn.h"

#include "t124/hardware_t124.h" /* for nvhost opcodes*/
#include "t124/t124.h"
#include "t210/t210.h"

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#include "t186/t186.h"
#endif

static int nvhost_flcn_init_sw(struct platform_device *dev);

static inline struct flcn *get_flcn(struct platform_device *dev)
{
	return (struct flcn *)nvhost_get_private_data(dev);
}
static inline void set_flcn(struct platform_device *dev, struct flcn *flcn)
{
	nvhost_set_private_data(dev, flcn);
}

#define FLCN_IDLE_TIMEOUT_DEFAULT	10000	/* 10 milliseconds */
#define FLCN_IDLE_CHECK_PERIOD		10	/* 10 usec */
static int flcn_wait_idle(struct platform_device *pdev,
				u32 *timeout)
{
	nvhost_dbg_fn("");

	if (!*timeout)
		*timeout = FLCN_IDLE_TIMEOUT_DEFAULT;

	do {
		u32 check = min_t(u32, FLCN_IDLE_CHECK_PERIOD, *timeout);
		u32 w = host1x_readl(pdev, flcn_idlestate_r());

		if (!w) {
			nvhost_dbg_fn("done");
			return 0;
		}
		udelay(FLCN_IDLE_CHECK_PERIOD);
		*timeout -= check;
	} while (*timeout || !tegra_platform_is_silicon());

	dev_err(&pdev->dev, "flcn flcn idle timeout");

	return -1;
}

static int flcn_dma_wait_idle(struct platform_device *pdev, u32 *timeout)
{
	nvhost_dbg_fn("");

	if (!*timeout)
		*timeout = FLCN_IDLE_TIMEOUT_DEFAULT;

	do {
		u32 check = min_t(u32, FLCN_IDLE_CHECK_PERIOD, *timeout);
		u32 dmatrfcmd = host1x_readl(pdev, flcn_dmatrfcmd_r());
		u32 idle_v = flcn_dmatrfcmd_idle_v(dmatrfcmd);

		if (flcn_dmatrfcmd_idle_true_v() == idle_v) {
			nvhost_dbg_fn("done");
			return 0;
		}
		udelay(FLCN_IDLE_CHECK_PERIOD);
		*timeout -= check;
	} while (*timeout || !tegra_platform_is_silicon());

	dev_err(&pdev->dev, "dma idle timeout");

	return -1;
}


static int flcn_dma_pa_to_internal_256b(struct platform_device *pdev,
					      phys_addr_t pa,
					      u32 internal_offset,
					      bool imem)
{
	u32 cmd = flcn_dmatrfcmd_size_256b_f();
	u32 pa_offset =  flcn_dmatrffboffs_offs_f(pa);
	u32 i_offset = flcn_dmatrfmoffs_offs_f(internal_offset);
	u32 timeout = 0; /* default*/

	if (imem)
		cmd |= flcn_dmatrfcmd_imem_true_f();

	host1x_writel(pdev, flcn_dmatrfmoffs_r(), i_offset);
	host1x_writel(pdev, flcn_dmatrffboffs_r(), pa_offset);
	host1x_writel(pdev, flcn_dmatrfcmd_r(), cmd);

	return flcn_dma_wait_idle(pdev, &timeout);

}

static int flcn_setup_ucode_image(struct platform_device *dev,
				   u32 *ucode_ptr,
				   const struct firmware *ucode_fw)
{
	struct flcn *v = get_flcn(dev);
	/* image data is little endian. */
	struct ucode_v1_flcn ucode;
	int w;

	nvhost_dbg_fn("");

	/* copy the whole thing taking into account endianness */
	for (w = 0; w < ucode_fw->size/sizeof(u32); w++)
		ucode_ptr[w] = le32_to_cpu(((u32 *)ucode_fw->data)[w]);

	ucode.bin_header = (struct ucode_bin_header_v1_flcn *)ucode_ptr;
	/* endian problems would show up right here */
	if (ucode.bin_header->bin_magic != 0x10de) {
		dev_err(&dev->dev,
			   "failed to get firmware magic");
		return -EINVAL;
	}
	if (ucode.bin_header->bin_ver != 1) {
		dev_err(&dev->dev,
			   "unsupported firmware version");
		return -ENOENT;
	}
	/* shouldn't be bigger than what firmware thinks */
	if (ucode.bin_header->bin_size > ucode_fw->size) {
		dev_err(&dev->dev,
			   "ucode image size inconsistency");
		return -EINVAL;
	}

	nvhost_dbg_info("ucode bin header: magic:0x%x ver:%d size:%d",
			ucode.bin_header->bin_magic,
			ucode.bin_header->bin_ver,
			ucode.bin_header->bin_size);
	nvhost_dbg_info("ucode bin header: os bin (header,data) offset size: 0x%x, 0x%x %d",
			ucode.bin_header->os_bin_header_offset,
			ucode.bin_header->os_bin_data_offset,
			ucode.bin_header->os_bin_size);
	nvhost_dbg_info("ucode bin header: fce bin (header,data) offset size: 0x%x, 0x%x %d",
			ucode.bin_header->fce_bin_header_offset,
			ucode.bin_header->fce_bin_data_offset,
			ucode.bin_header->fce_bin_size);

	ucode.os_header = (struct ucode_os_header_v1_flcn *)
		(((void *)ucode_ptr) + ucode.bin_header->os_bin_header_offset);

	nvhost_dbg_info("os ucode header: os code (offset,size): 0x%x, 0x%x",
			ucode.os_header->os_code_offset,
			ucode.os_header->os_code_size);
	nvhost_dbg_info("os ucode header: os data (offset,size): 0x%x, 0x%x",
			ucode.os_header->os_data_offset,
			ucode.os_header->os_data_size);
	nvhost_dbg_info("os ucode header: num apps: %d", ucode.os_header->num_apps);

	if (ucode.bin_header->fce_bin_header_offset != 0xa5a5a5a5) {
		ucode.fce_header = (struct ucode_fce_header_v1_flcn *)
			(((void *)ucode_ptr) +
			 ucode.bin_header->fce_bin_header_offset);
		nvhost_dbg_info("fce ucode header: offset, buffer_size, size: 0x%x 0x%x 0x%x",
				ucode.fce_header->fce_ucode_offset,
				ucode.fce_header->fce_ucode_buffer_size,
				ucode.fce_header->fce_ucode_size);
		v->fce.size        = ucode.fce_header->fce_ucode_size;
		v->fce.data_offset =
			ucode.bin_header->fce_bin_data_offset;
	}

	v->os.size = ucode.bin_header->os_bin_size;
	v->os.bin_data_offset = ucode.bin_header->os_bin_data_offset;
	v->os.code_offset = ucode.os_header->os_code_offset;
	v->os.data_offset = ucode.os_header->os_data_offset;
	v->os.data_size   = ucode.os_header->os_data_size;

	return 0;
}

static int flcn_read_ucode(struct platform_device *dev, const char *fw_name)
{
	struct flcn *v = get_flcn(dev);
	const struct firmware *ucode_fw;
	int err;
	DEFINE_DMA_ATTRS(attrs);

	nvhost_dbg_fn("");

	v->dma_addr = 0;
	v->mapped = NULL;

	ucode_fw = nvhost_client_request_firmware(dev, fw_name);
	if (!ucode_fw) {
		nvhost_dbg_fn("request firmware failed");
		dev_err(&dev->dev, "failed to get firmware\n");
		err = -ENOENT;
		return err;
	}

	v->size = ucode_fw->size;
	dma_set_attr(DMA_ATTR_READ_ONLY, &attrs);

	v->mapped = dma_alloc_attrs(&dev->dev,
				v->size, &v->dma_addr,
				GFP_KERNEL, &attrs);
	if (!v->mapped) {
		dev_err(&dev->dev, "dma memory allocation failed");
		err = -ENOMEM;
		goto clean_up;
	}

	err = flcn_setup_ucode_image(dev, v->mapped, ucode_fw);
	if (err) {
		dev_err(&dev->dev, "failed to parse firmware image\n");
		goto clean_up;
	}

	v->valid = true;

	nvhost_vm_map_static(dev, v->mapped, v->dma_addr, v->size);

	release_firmware(ucode_fw);

	return 0;

 clean_up:
	if (v->mapped) {
		dma_free_attrs(&dev->dev,
			v->size, v->mapped,
			v->dma_addr, &attrs);
		v->mapped = NULL;
		v->dma_addr = 0;
	}
	release_firmware(ucode_fw);
	return err;
}

static int flcn_wait_mem_scrubbing(struct platform_device *dev)
{
	int retries = FLCN_IDLE_TIMEOUT_DEFAULT / FLCN_IDLE_CHECK_PERIOD;
	nvhost_dbg_fn("");

	do {
		u32 w = host1x_readl(dev, flcn_dmactl_r()) &
			(flcn_dmactl_dmem_scrubbing_m() |
			 flcn_dmactl_imem_scrubbing_m());

		if (!w) {
			nvhost_dbg_fn("done");
			return 0;
		}
		udelay(FLCN_IDLE_CHECK_PERIOD);
	} while (--retries || !tegra_platform_is_silicon());

	nvhost_err(&dev->dev, "Falcon mem scrubbing timeout");
	return -ETIMEDOUT;
}

int nvhost_flcn_finalize_poweron(struct platform_device *pdev)
{
	struct flcn *v;
	u32 timeout;
	u32 offset;
	int err = 0;

	err = nvhost_flcn_init_sw(pdev);
	if (err)
		return err;

	v = get_flcn(pdev);

	err = flcn_wait_mem_scrubbing(pdev);
	if (err)
		return err;

	host1x_writel(pdev, flcn_dmactl_r(), 0);

	host1x_writel(pdev, flcn_dmatrfbase_r(),
			(v->dma_addr + v->os.bin_data_offset) >> 8);

	for (offset = 0; offset < v->os.data_size; offset += 256)
		flcn_dma_pa_to_internal_256b(pdev,
					   v->os.data_offset + offset,
					   offset, false);

	flcn_dma_pa_to_internal_256b(pdev, v->os.code_offset,
					   0, true);

	/* setup falcon interrupts and enable interface */
	host1x_writel(pdev, flcn_irqmset_r(),
			     (flcn_irqmset_ext_f(0xff)    |
					   flcn_irqmset_swgen1_set_f() |
					   flcn_irqmset_swgen0_set_f() |
					   flcn_irqmset_exterr_set_f() |
					   flcn_irqmset_halt_set_f()   |
					   flcn_irqmset_wdtmr_set_f()));
	host1x_writel(pdev, flcn_irqdest_r(),
			     (flcn_irqdest_host_ext_f(0xff) |
					   flcn_irqdest_host_swgen1_host_f() |
					   flcn_irqdest_host_swgen0_host_f() |
					   flcn_irqdest_host_exterr_host_f() |
					   flcn_irqdest_host_halt_host_f()));
	host1x_writel(pdev, flcn_itfen_r(),
			     (flcn_itfen_mthden_enable_f() |
					flcn_itfen_ctxen_enable_f()));

	/* boot falcon */
	host1x_writel(pdev, flcn_bootvec_r(), flcn_bootvec_vec_f(0));
	host1x_writel(pdev, flcn_cpuctl_r(),
			flcn_cpuctl_startcpu_true_f());

	timeout = 0; /* default */

	err = flcn_wait_idle(pdev, &timeout);
	if (err != 0) {
		dev_err(&pdev->dev, "boot failed due to timeout");
		return err;
	}

	return 0;
}

static int nvhost_flcn_init_sw(struct platform_device *dev)
{
	int err = 0;
	struct nvhost_device_data *pdata = nvhost_get_devdata(dev);
	struct flcn *v = get_flcn(dev);

	nvhost_dbg_fn("in dev:%p v:%p", dev, v);

	if (v)
		return 0;

	v = kzalloc(sizeof(*v), GFP_KERNEL);
	if (!v) {
		dev_err(&dev->dev, "couldn't alloc flcn support");
		err = -ENOMEM;
		goto clean_up;
	}
	set_flcn(dev, v);
	nvhost_dbg_fn("primed dev:%p v:%p", dev, v);

	err = flcn_read_ucode(dev, pdata->firmware_name);
	if (err || !v->valid)
		goto clean_up;

	return 0;

 clean_up:
	nvhost_err(&dev->dev, "failed");
	return err;
}

int nvhost_vic_finalize_poweron(struct platform_device *pdev)
{
	struct flcn *v;
	int err;

	err = nvhost_flcn_finalize_poweron(pdev);
	if (err)
		return err;

	v = get_flcn(pdev);

	host1x_writel(pdev, VIC_UCLASS_METHOD_OFFSET * 4,
		      NVA0B6_VIDEO_COMPOSITOR_SET_APPLICATION_ID >> 2);
	host1x_writel(pdev, VIC_UCLASS_METHOD_DATA * 4, 1);
	host1x_writel(pdev, VIC_UCLASS_METHOD_OFFSET * 4,
		      NVA0B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_SIZE >> 2);
	host1x_writel(pdev, VIC_UCLASS_METHOD_DATA * 4, v->fce.size);
	host1x_writel(pdev, VIC_UCLASS_METHOD_OFFSET * 4,
		      NVA0B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_OFFSET >> 2);
	host1x_writel(pdev, VIC_UCLASS_METHOD_DATA * 4,
		      (v->dma_addr + v->fce.data_offset) >> 8);

	return 0;
}

int nvhost_vic_aggregate_constraints(struct platform_device *dev,
				     int clk_index,
				     unsigned long floor_rate,
				     unsigned long pixelrate,
				     unsigned long bw_constraint)
{
	struct nvhost_device_data *pdata = nvhost_get_devdata(dev);

	if (!pdata) {
		dev_err(&dev->dev, "no platform data\n");
		/* return 0 to fall-back on default policy */
		return 0;
	}

	/* Fall-back to default policy if pixelrate
	 * is unavailable or clk index is incorrect.
	 * Here clk_index 2 is for floor client.
	 */
	if (!pixelrate || clk_index != 2)
		return 0;

	/* Compute VIC frequency based on pixelrate */
	return pixelrate / pdata->num_ppc;
}

static struct of_device_id tegra_flcn_of_match[] = {
#ifdef CONFIG_ARCH_TEGRA_VIC
	{ .compatible = "nvidia,tegra124-vic",
		.data = (struct nvhost_device_data *)&t124_vic_info },
#endif
	{ .compatible = "nvidia,tegra124-msenc",
		.data = (struct nvhost_device_data *)&t124_msenc_info },
#ifdef CONFIG_ARCH_TEGRA_VIC
	{ .compatible = "nvidia,tegra210-vic",
		.data = (struct nvhost_device_data *)&t21_vic_info },
#endif
#ifdef TEGRA_21X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra210-nvenc",
		.data = (struct nvhost_device_data *)&t21_msenc_info },
#endif
#ifdef TEGRA_21X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra210-nvjpg",
		.data = (struct nvhost_device_data *)&t21_nvjpg_info },
#endif
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	{ .compatible = "nvidia,tegra186-vic",
		.data = (struct nvhost_device_data *)&t18_vic_info },
#endif
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	{ .compatible = "nvidia,tegra186-nvjpg",
		.data = (struct nvhost_device_data *)&t18_nvjpg_info },
#endif
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	{ .compatible = "nvidia,tegra186-nvenc",
		.data = (struct nvhost_device_data *)&t18_msenc_info },
#endif
	{ },
};

static int flcn_probe(struct platform_device *dev)
{
	int err;
	struct nvhost_device_data *pdata = NULL;

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_flcn_of_match, &dev->dev);
		if (match)
			pdata = (struct nvhost_device_data *)match->data;
	} else
		pdata = (struct nvhost_device_data *)dev->dev.platform_data;

	if (!pdata) {
		dev_err(&dev->dev, "no platform data\n");
		return -ENODATA;
	}

	nvhost_dbg_fn("dev:%p pdata:%p", dev, pdata);

	err = nvhost_check_bondout(pdata->bond_out_id);
	if (err) {
		dev_err(&dev->dev, "flcn unit not present. err:%d", err);
		return err;
	}

	pdata->pdev = dev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(dev, pdata);

	err = nvhost_client_device_get_resources(dev);
	if (err)
		return err;

	dev->dev.platform_data = NULL;

	nvhost_module_init(dev);

#ifdef CONFIG_PM_GENERIC_DOMAINS
#ifndef CONFIG_PM_GENERIC_DOMAINS_OF
	pdata->pd.name = kstrdup(dev->name, GFP_KERNEL);
	if (!pdata->pd.name)
		return -ENOMEM;
#endif
	err = nvhost_module_add_domain(&pdata->pd, dev);
#endif

	err = nvhost_client_device_init(dev);
	if (err) {
		nvhost_dbg_fn("failed to init client device for %s",
			      dev->name);
		pm_runtime_put(&dev->dev);
		return err;
	}

	return 0;
}

static int __exit flcn_remove(struct platform_device *dev)
{
	nvhost_client_device_release(dev);
	return 0;
}

static struct platform_device_id flcn_id_table[] = {
	{ .name = "vic03" },
	{ .name = "msenc" },
	{ .name = "msenc" },
	{ .name = "nvjpg" },
	{},
};
static struct platform_driver flcn_driver = {
	.probe = flcn_probe,
	.remove = __exit_p(flcn_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "falcon",
#ifdef CONFIG_OF
		.of_match_table = tegra_flcn_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
	.id_table = flcn_id_table,
};

static struct of_device_id tegra21x_flcn_domain_match[] = {
	{.compatible = "nvidia,tegra210-vic03-pd",
	 .data = (struct nvhost_device_data *)&t21_vic_info},
	{.compatible = "nvidia,tegra210-msenc-pd",
	 .data = (struct nvhost_device_data *)&t21_msenc_info},
	{.compatible = "nvidia,tegra210-nvjpg-pd",
	 .data = (struct nvhost_device_data *)&t21_nvjpg_info},
	{},
};

static int __init flcn_init(void)
{
	int ret;

	ret = nvhost_domain_init(tegra21x_flcn_domain_match);
	if (ret)
		return ret;

	return platform_driver_register(&flcn_driver);
}

static void __exit flcn_exit(void)
{
	platform_driver_unregister(&flcn_driver);
}

module_init(flcn_init);
module_exit(flcn_exit);
