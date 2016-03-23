/*
 * include/linux/nvhost.h
 *
 * Tegra graphics host driver
 *
 * Copyright (c) 2009-2015, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __LINUX_NVHOST_H
#define __LINUX_NVHOST_H

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/devfreq.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_qos.h>
#include <linux/time.h>

struct nvhost_master;
struct nvhost_hwctx;
struct nvhost_device_power_attr;
struct nvhost_device_profile;
struct mem_mgr;
struct nvhost_as_moduleops;
struct nvhost_ctrl_sync_fence_info;
struct nvhost_sync_timeline;
struct nvhost_sync_pt;
struct sync_pt;

#define NVHOST_MODULE_MAX_CLOCKS		8
#define NVHOST_MODULE_MAX_SYNCPTS		8
#define NVHOST_MODULE_MAX_WAITBASES		3
#define NVHOST_MODULE_MAX_MODMUTEXES		5
#define NVHOST_MODULE_MAX_IORESOURCE_MEM	3
#define NVHOST_MODULE_NO_POWERGATE_ID		.powergate_id = -1
#define NVHOST_DEFAULT_CLOCKGATE_DELAY		.clockgate_delay = 25
#define NVHOST_MODULE_MAX_IORESOURCE_MEM 3
#define NVHOST_NAME_SIZE			24
#define NVSYNCPT_INVALID			(-1)

#define NVSYNCPT_AVP_0			(10)	/* t20, t30, t114, t148 */
#define NVSYNCPT_3D			(22)	/* t20, t30, t114, t148 */
#define NVSYNCPT_VBLANK0		(26)	/* t20, t30, t114, t148 */
#define NVSYNCPT_VBLANK1		(27)	/* t20, t30, t114, t148 */

#define NVMODMUTEX_ISP_0		(1)	/* t124, t132, t210 */
#define NVMODMUTEX_ISP_1		(2)	/* t124, t132, t210 */
#define NVMODMUTEX_NVJPG		(3)	/* t210 */
#define NVMODMUTEX_NVDEC		(4)	/* t210 */
#define NVMODMUTEX_MSENC		(5)	/* t124, t132, t210 */
#define NVMODMUTEX_TSECA		(6)	/* t124, t132, t210 */
#define NVMODMUTEX_TSECB		(7)	/* t124, t132, t210 */
#define NVMODMUTEX_VI			(8)	/* t124, t132, t210 */
#define NVMODMUTEX_VI_0			(8)	/* t148 */
#define NVMODMUTEX_VIC			(10)	/* t124, t132, t210 */
#define NVMODMUTEX_VI_1			(11)	/* t124, t132, t210 */

enum nvhost_power_sysfs_attributes {
	NVHOST_POWER_SYSFS_ATTRIB_CLOCKGATE_DELAY = 0,
	NVHOST_POWER_SYSFS_ATTRIB_POWERGATE_DELAY,
	NVHOST_POWER_SYSFS_ATTRIB_FORCE_ON,
	NVHOST_POWER_SYSFS_ATTRIB_MAX
};

struct nvhost_notification {
	struct {			/* 0000- */
		__u32 nanoseconds[2];	/* nanoseconds since Jan. 1, 1970 */
	} time_stamp;			/* -0007 */
	__u32 info32;	/* info returned depends on method 0008-000b */
#define	NVHOST_CHANNEL_FIFO_ERROR_IDLE_TIMEOUT	8
#define	NVHOST_CHANNEL_GR_ERROR_SW_NOTIFY	13
#define	NVHOST_CHANNEL_GR_SEMAPHORE_TIMEOUT	24
#define	NVHOST_CHANNEL_GR_ILLEGAL_NOTIFY	25
#define	NVHOST_CHANNEL_FIFO_ERROR_MMU_ERR_FLT	31
#define	NVHOST_CHANNEL_PBDMA_ERROR		32
#define	NVHOST_CHANNEL_RESETCHANNEL_VERIF_ERROR	43
	__u16 info16;	/* info returned depends on method 000c-000d */
	__u16 status;	/* user sets bit 15, NV sets status 000e-000f */
#define	NVHOST_CHANNEL_SUBMIT_TIMEOUT		1
};

struct nvhost_gating_register {
	u64 addr;
	u32 prod;
	u32 disable;
};

struct nvhost_clock {
	char *name;
	unsigned long default_rate;
	u32 moduleid;
	int reset;
	unsigned long devfreq_rate;
};

/*
 * Defines HW and SW class identifiers.
 *
 * This is module ID mapping between userspace and kernelspace.
 * The values of enum entries' are referred from NvRmModuleID enum defined
 * in below userspace file:
 * $TOP/vendor/nvidia/tegra/core/include/nvrm_module.h
 * Please make sure each entry below has same value as set in above file.
 */
enum nvhost_module_identifier {

	/* Specifies external memory (DDR RAM, etc) */
	NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER = 75,

	/* Specifies CBUS floor client module */
	NVHOST_MODULE_ID_CBUS_FLOOR = 119,

	/* Specifies shared EMC client module */
	NVHOST_MODULE_ID_EMC_SHARED,
	NVHOST_MODULE_ID_MAX
};

struct nvhost_device_data {
	int		version;	/* ip version number of device */
	int		id;		/* Separates clients of same hw */
	void __iomem	*aperture[NVHOST_MODULE_MAX_IORESOURCE_MEM];
	struct device_dma_parameters dma_parms;

	u32		modulemutexes[NVHOST_MODULE_MAX_MODMUTEXES];
	u32		moduleid;	/* Module id for user space API */

	/* Should we toggle the engine SLCG when we turn on the domain? */
	bool		poweron_toggle_slcg;

	/* Flag to set SLCG notifier (for the modules other than VIC) */
	bool slcg_notifier_enable;

	/* Used to serialize channel when map-at-submit is used w/o mlocks */
	u32		last_submit_syncpt_id;
	u32		last_submit_syncpt_value;

	u32		class;		/* Device class */
	bool		exclusive;	/* True if only one user at a time */
	bool		keepalive;	/* Do not power gate when opened */
	bool		serialize;	/* Serialize submits in the channel */
	bool		push_work_done;	/* Push_op done into push buffer */
	bool		poweron_reset;	/* Reset the engine before powerup */
	bool		virtual_dev;	/* True if virtualized device */
	char		*devfs_name;	/* Name in devfs */

	char		*firmware_name;	/* Name of firmware */

	int		powergate_id;
	bool		engine_can_cg;	/* True if CG is enabled */
	bool		can_powergate;	/* True if module can be power gated */
	int		clockgate_delay;/* Delay before clock gated */
	int		powergate_delay;/* Delay before power gated */
	struct nvhost_clock clocks[NVHOST_MODULE_MAX_CLOCKS];/* Clock names */

	/* Clock gating registers */
	struct nvhost_gating_register *engine_cg_regs;


	struct platform_device *master;	/* Master of a slave device */
	struct platform_device *slave;	/* Slave device to create in probe */
	int		slave_initialized;

	int		num_clks;	/* Number of clocks opened for dev */
	struct clk	*clk[NVHOST_MODULE_MAX_CLOCKS];
	struct mutex	lock;		/* Power management lock */
	struct list_head client_list;	/* List of clients and rate requests */

	int		num_channels;	/* Max num of channel supported */
	int		num_mapped_chs;	/* Num of channel mapped to device */
	int		num_ppc;	/* Number of pixels per clock cycle */

	/* Channel(s) assigned for the module */
	struct nvhost_channel **channels;

	/* device node for channel operations */
	dev_t cdev_region;
	struct device *node;
	struct cdev cdev;

	/* Address space device node */
	struct device *as_node;
	struct cdev as_cdev;

	/* device node for ctrl block */
	struct device *ctrl_node;
	struct cdev ctrl_cdev;
	const struct file_operations *ctrl_ops;    /* ctrl ops for the module */

	/* address space operations */
	const struct nvhost_as_moduleops *as_ops;

	struct kobject *power_kobj;	/* kobject to hold power sysfs entries */
	struct nvhost_device_power_attr *power_attrib;	/* sysfs attributes */
	struct dentry *debugfs;		/* debugfs directory */

	u32 nvhost_timeout_default;

	/* QoS id that denotes minimum frequency */
	unsigned int			qos_id;
	/* Data for devfreq usage */
	struct devfreq			*power_manager;
	/* Private device profile data */
	struct nvhost_device_profile	*power_profile;
	/* Should we read load estimate from hardware? */
	bool				actmon_enabled;
	/* Should we do linear emc scaling? */
	bool				linear_emc;
	/* Offset to actmon registers */
	u32				actmon_regs;
	/* Devfreq governor name */
	const char			*devfreq_governor;

	/* Marks if the device is booted when pm runtime is disabled */
	bool				booted;

	void *private_data;		/* private platform data */
	struct platform_device *pdev;	/* owner platform_device */
	void *virt_priv;		/* private data for virtualized dev */

	struct mutex no_poweroff_req_mutex;
	struct dev_pm_qos_request no_poweroff_req;
	int no_poweroff_req_count;

	struct notifier_block		toggle_slcg_notifier;

#ifdef CONFIG_PM_GENERIC_DOMAINS
	struct generic_pm_domain pd;	/* power domain representing power partition */
#endif

	/* Finalize power on. Can be used for context restore. */
	int (*finalize_poweron)(struct platform_device *dev);

	/*
	 * Reset the unit. Used for timeout recovery, resetting the unit on
	 * probe and when un-powergating.
	 */
	void (*reset)(struct platform_device *dev);

	/* Device is busy. */
	void (*busy)(struct platform_device *);

	/* Device is idle. */
	void (*idle)(struct platform_device *);

	/* Scaling init is run on device registration */
	void (*scaling_init)(struct platform_device *dev);

	/* Scaling deinit is called on device unregistration */
	void (*scaling_deinit)(struct platform_device *dev);

	/* Postscale callback is called after frequency change */
	void (*scaling_post_cb)(struct nvhost_device_profile *profile,
				unsigned long freq);

	/* Preparing for power off. Used for context save. */
	int (*prepare_poweroff)(struct platform_device *dev);

	/* paring for power off. Used for context save. */
	int (*aggregate_constraints)(struct platform_device *dev,
				     int clk_index,
				     unsigned long floor_rate,
				     unsigned long pixel_rate,
				     unsigned long bw_rate);

	/* Called after successful client device init. This can
	 * be used in cases where the hardware specifics differ
	 * between hardware revisions */
	int (*hw_init)(struct platform_device *dev);

	/* Allocates a context handler for the device */
	struct nvhost_hwctx_handler *(*alloc_hwctx_handler)(u32 syncpt,
			struct nvhost_channel *ch);

	/* bond out device id */
	unsigned int bond_out_id;

	phys_addr_t carveout_addr;
	phys_addr_t carveout_size;

	u64 mamask_addr;
	u32 mamask_val;
	u64 borps_addr;
	u32 borps_val;

	/* Actmon IRQ from hintstatus_r */
	unsigned int actmon_irq;

	/* Is the device already forced on? */
	bool forced_on;

	/* Override flag for a device */
	bool forced_map_on_open;

	/* channel user context list */
	struct mutex userctx_list_lock;
	struct list_head userctx_list;
};


static inline
struct nvhost_device_data *nvhost_get_devdata(struct platform_device *pdev)
{
	return (struct nvhost_device_data *)platform_get_drvdata(pdev);
}

static inline bool nvhost_dev_is_virtual(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	return pdata->virtual_dev;
}

struct nvhost_device_power_attr {
	struct platform_device *ndev;
	struct kobj_attribute power_attr[NVHOST_POWER_SYSFS_ATTRIB_MAX];
};

void host1x_writel(struct platform_device *dev, u32 r, u32 v);
u32 host1x_readl(struct platform_device *dev, u32 r);

/* public host1x power management APIs */
bool nvhost_module_powered_ext(struct platform_device *dev);
int nvhost_module_busy_ext(struct platform_device *dev);
void nvhost_module_idle_ext(struct platform_device *dev);

/* public api to register/unregister a subdomain */
void nvhost_register_client_domain(struct generic_pm_domain *domain);
void nvhost_unregister_client_domain(struct generic_pm_domain *domain);

/* public APIs required to submit in-kernel work */
int nvhost_channel_map(struct nvhost_device_data *pdata,
			struct nvhost_channel **ch,
			void *identifier);
void nvhost_putchannel(struct nvhost_channel *ch, int cnt);
/* Allocate memory for a job. Just enough memory will be allocated to
 * accomodate the submit announced in submit header.
 */
struct nvhost_job *nvhost_job_alloc(struct nvhost_channel *ch,
		int num_cmdbufs, int num_relocs, int num_waitchks,
		int num_syncpts);
/* Decrement reference job, free if goes to zero. */
void nvhost_job_put(struct nvhost_job *job);

/* Add a gather with IOVA address to job */
int nvhost_job_add_client_gather_address(struct nvhost_job *job,
		u32 num_words, u32 class_id, dma_addr_t gather_address);
int nvhost_channel_submit(struct nvhost_job *job);

/* common device management APIs */
int nvhost_client_device_get_resources(struct platform_device *dev);
int nvhost_client_device_release(struct platform_device *dev);
int nvhost_client_device_init(struct platform_device *dev);
int nvhost_check_bondout(unsigned int id);

/* common runtime pm and power domain APIs */
int nvhost_module_init(struct platform_device *ndev);
int nvhost_module_add_domain(struct generic_pm_domain *domain,
	struct platform_device *pdev);
extern const struct dev_pm_ops nvhost_module_pm_ops;

/* public host1x sync-point management APIs */
u32 nvhost_get_syncpt_client_managed(const char *syncpt_name);
u32 nvhost_get_syncpt_host_managed(struct platform_device *pdev,
				   u32 param);
u32 nvhost_get_syncpt_host_managed_by_name(const char *syncpt_name);
void nvhost_free_syncpt(u32 id);
const char *nvhost_syncpt_get_name(struct platform_device *dev, int id);
u32 nvhost_syncpt_incr_max_ext(struct platform_device *dev, u32 id, u32 incrs);
void nvhost_syncpt_cpu_incr_ext(struct platform_device *dev, u32 id);
int nvhost_syncpt_read_ext_check(struct platform_device *dev, u32 id, u32 *val);
int nvhost_syncpt_wait_timeout_ext(struct platform_device *dev, u32 id, u32 thresh,
	u32 timeout, u32 *value, struct timespec *ts);
int nvhost_syncpt_create_fence_single_ext(struct platform_device *dev,
	u32 id, u32 thresh, const char *name, int *fence_fd);
int nvhost_syncpt_is_expired_ext(struct platform_device *dev,
	u32 id, u32 thresh);
void nvhost_syncpt_set_min_eq_max_ext(struct platform_device *dev, u32 id);
int nvhost_syncpt_nb_pts_ext(struct platform_device *dev);
bool nvhost_syncpt_is_valid_pt_ext(struct platform_device *dev, u32 id);

/* public host1x interrupt management APIs */
int nvhost_intr_register_notifier(struct platform_device *pdev,
				  u32 id, u32 thresh,
				  void (*callback)(void *, int),
				  void *private_data);


#ifdef CONFIG_TEGRA_GRHOST
void nvhost_debug_dump_device(struct platform_device *pdev);
const struct firmware *
nvhost_client_request_firmware(struct platform_device *dev,
	const char *fw_name);
#else
static inline void nvhost_debug_dump_device(struct platform_device *pdev) {}

static inline const struct firmware *
nvhost_client_request_firmware(struct platform_device *dev,
	const char *fw_name)
{
	return NULL;
}
#endif

#ifdef CONFIG_TEGRA_GRHOST_SYNC
struct sync_fence *nvhost_sync_create_fence(
		struct platform_device *pdev,
		struct nvhost_ctrl_sync_fence_info *pts,
		u32 num_pts,
		const char *name);
int nvhost_sync_create_fence_fd(
		struct platform_device *pdev,
		struct nvhost_ctrl_sync_fence_info *pts,
		u32 num_pts,
		const char *name,
		s32 *fence_fd);
struct sync_fence *nvhost_sync_fdget(int fd);
int nvhost_sync_num_pts(struct sync_fence *fence);
u32 nvhost_sync_pt_id(struct sync_pt *pt);
u32 nvhost_sync_pt_thresh(struct sync_pt *pt);
int nvhost_sync_fence_set_name(int fence_fd, const char *name);

#else
static inline struct sync_fence *nvhost_sync_create_fence(
		struct platform_device *pdev,
		struct nvhost_ctrl_sync_fence_info *pts,
		u32 num_pts,
		const char *name)
{
	return ERR_PTR(-EINVAL);
}

static inline int nvhost_sync_create_fence_fd(
		struct platform_device *pdev,
		struct nvhost_ctrl_sync_fence_info *pts,
		u32 num_pts,
		const char *name,
		s32 *fence_fd)
{
	return -EINVAL;
}

static inline struct sync_fence *nvhost_sync_fdget(int fd)
{
	return NULL;
}

static inline int nvhost_sync_num_pts(struct sync_fence *fence)
{
	return 0;
}

static inline u32 nvhost_sync_pt_id(struct sync_pt *pt)
{
	return NVSYNCPT_INVALID;
}

static inline u32 nvhost_sync_pt_thresh(struct sync_pt *pt)
{
	return 0;
}

static inline int nvhost_sync_fence_set_name(int fence_fd, const char *name)
{
	return -EINVAL;
}

#endif

/* Hacky way to get access to struct nvhost_device_data for VI device. */
extern struct nvhost_device_data t20_vi_info;
extern struct nvhost_device_data t30_vi_info;
extern struct nvhost_device_data t11_vi_info;
extern struct nvhost_device_data t14_vi_info;

#endif
