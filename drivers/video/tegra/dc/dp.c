/*
 * drivers/video/tegra/dc/dp.c
 *
 * Copyright (c) 2011-2015, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/tegra-soc.h>
#include <linux/clk/tegra.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#include <mach/dc.h>
#include <mach/fb.h>

#include "dp.h"
#include "sor.h"
#include "sor_regs.h"
#include "dpaux_regs.h"
#include "dpaux.h"
#include "dc_priv.h"
#include "edid.h"
#include "dp_lt.h"

#ifdef CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT
#include "fake_panel.h"
#endif /*CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT*/

static bool tegra_dp_debug = true;
module_param(tegra_dp_debug, bool, 0644);
MODULE_PARM_DESC(tegra_dp_debug, "Enable to print all link configs");

static struct tegra_hpd_ops hpd_ops;

static inline void tegra_dp_reset(struct tegra_dc_dp_data *dp);
static inline void tegra_dp_default_int(struct tegra_dc_dp_data *dp,
					bool enable);
static void tegra_dp_hpd_config(struct tegra_dc_dp_data *dp);

static inline u32 tegra_dpaux_readl(struct tegra_dc_dp_data *dp, u32 reg)
{
	return readl(dp->aux_base + reg * 4);
}

static inline void tegra_dpaux_writel(struct tegra_dc_dp_data *dp,
	u32 reg, u32 val)
{
	writel(val, dp->aux_base + reg * 4);
}

static inline void tegra_dpaux_clk_enable(struct tegra_dc_dp_data *dp)
{
	if (tegra_platform_is_linsim())
		return;
	clk_prepare_enable(dp->dpaux_clk);
}

static inline void tegra_dpaux_clk_disable(struct tegra_dc_dp_data *dp)
{
	if (tegra_platform_is_linsim())
		return;
	clk_disable_unprepare(dp->dpaux_clk);
}

static inline void tegra_dp_clk_enable(struct tegra_dc_dp_data *dp)
{
	if (!tegra_is_clk_enabled(dp->parent_clk))
		clk_prepare_enable(dp->parent_clk);
}

static inline void tegra_dp_clk_disable(struct tegra_dc_dp_data *dp)
{
	if (tegra_is_clk_enabled(dp->parent_clk))
		clk_disable_unprepare(dp->parent_clk);
}

static inline void tegra_dpaux_write_field(struct tegra_dc_dp_data *dp,
					u32 reg, u32 mask, u32 val)
{
	u32 reg_val = tegra_dpaux_readl(dp, reg);
	reg_val = (reg_val & ~mask) | (val & mask);
	tegra_dpaux_writel(dp, reg, reg_val);
}

static inline void tegra_dp_int_en(struct tegra_dc_dp_data *dp, u32 intr)
{
	u32 val;

	/* clear pending interrupt */
	tegra_dpaux_writel(dp, DPAUX_INTR_AUX, intr);

	val = tegra_dpaux_readl(dp, DPAUX_INTR_EN_AUX);
	val |= intr;

	tegra_dpaux_writel(dp, DPAUX_INTR_EN_AUX, val);
}

static inline void tegra_dp_int_dis(struct tegra_dc_dp_data *dp, u32 intr)
{
	u32 val;

	val = tegra_dpaux_readl(dp, DPAUX_INTR_EN_AUX);
	val &= ~intr;

	tegra_dpaux_writel(dp, DPAUX_INTR_EN_AUX, val);
}

static inline void tegra_dp_enable_irq(u32 irq)
{
	if (tegra_platform_is_fpga())
		return;

	enable_irq(irq);
}

static inline void tegra_dp_disable_irq(u32 irq)
{
	if (tegra_platform_is_fpga())
		return;

	disable_irq(irq);
}

#define is_hotplug_supported(dp) \
({ \
	!tegra_platform_is_fpga() && \
	!tegra_platform_is_linsim() && \
	tegra_dc_is_ext_dp_panel(dp->dc) && \
	dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP; \
})

static inline void tegra_dp_pending_hpd(struct tegra_dc_dp_data *dp)
{
	if (!is_hotplug_supported(dp))
		return;

	tegra_hpd_set_pending_evt(&dp->hpd_data);
}

static inline unsigned long
tegra_dc_dpaux_poll_register(struct tegra_dc_dp_data *dp,
				u32 reg, u32 mask, u32 exp_val,
				u32 poll_interval_us, u32 timeout_ms)
{
	unsigned long	timeout_jf = jiffies + msecs_to_jiffies(timeout_ms);
	u32		reg_val	   = 0;

	if (tegra_platform_is_linsim())
		return 0;

	do {
		usleep_range(poll_interval_us, poll_interval_us << 1);
		reg_val = tegra_dpaux_readl(dp, reg);
	} while (((reg_val & mask) != exp_val) &&
		time_after(timeout_jf, jiffies));

	if ((reg_val & mask) == exp_val)
		return 0;	/* success */
	dev_dbg(&dp->dc->ndev->dev,
		"dpaux_poll_register 0x%x: timeout\n", reg);
	return jiffies - timeout_jf + 1;
}

static inline int tegra_dpaux_wait_transaction(struct tegra_dc_dp_data *dp)
{
	int err = 0;

	if (unlikely(tegra_platform_is_fpga())) {
		if (tegra_dc_dpaux_poll_register(dp, DPAUX_DP_AUXCTL,
				DPAUX_DP_AUXCTL_TRANSACTREQ_MASK,
				DPAUX_DP_AUXCTL_TRANSACTREQ_DONE,
				100, DP_AUX_TIMEOUT_MS) != 0)
			err = -EFAULT;
	}

	if (likely(tegra_platform_is_silicon())) {
		INIT_COMPLETION(dp->aux_tx);
		tegra_dp_int_en(dp, DPAUX_INTR_EN_AUX_TX_DONE);
		if (tegra_dpaux_readl(dp, DPAUX_DP_AUXCTL) &
				DPAUX_DP_AUXCTL_TRANSACTREQ_PENDING) {
			if (!wait_for_completion_timeout(&dp->aux_tx,
				msecs_to_jiffies(DP_AUX_TIMEOUT_MS)))
				err = -EBUSY;
		}
		tegra_dp_int_dis(dp, DPAUX_INTR_EN_AUX_TX_DONE);
	}

	if (err)
		dev_err(&dp->dc->ndev->dev, "dp: aux tx timeout\n");
	return err;
}


/*
 * To config DPAUX Transaction Control
 * o Inputs
 *  - dp    : pointer to DP information
 *  - cmd   : transaction command DPAUX_DP_AUXCTL_CMD_xxx
 *  - addr  : transaction address (20 bit sink device AUX reg addr space)
 *  - p_wrdt: pointer to the write data buffer / NULL:no write data
 *  - size  : 1-16: number of byte to read/write
 *            0   : address only transaction
 * o Outputs
 *  - return: error status; 0:no error / !0:error
 */
static int tegra_dp_aux_tx_config(struct tegra_dc_dp_data *dp,
				u32 cmd, u32 addr, u8 *p_wrdt, u32 size)
{
	int i;
	u32  *data = (u32 *)p_wrdt;

	if (DP_AUX_MAX_BYTES < size)
		goto fail;

	switch (cmd) {
	case DPAUX_DP_AUXCTL_CMD_I2CWR:
	case DPAUX_DP_AUXCTL_CMD_I2CRD:
	case DPAUX_DP_AUXCTL_CMD_I2CREQWSTAT:
	case DPAUX_DP_AUXCTL_CMD_MOTWR:
	case DPAUX_DP_AUXCTL_CMD_MOTRD:
	case DPAUX_DP_AUXCTL_CMD_MOTREQWSTAT:
	case DPAUX_DP_AUXCTL_CMD_AUXWR:
	case DPAUX_DP_AUXCTL_CMD_AUXRD:
		tegra_dpaux_write_field(dp, DPAUX_DP_AUXCTL,
					DPAUX_DP_AUXCTL_CMD_MASK, cmd);
		break;
	default:
		goto fail;
	};
	tegra_dpaux_write_field(dp, DPAUX_DP_AUXCTL,
				DPAUX_DP_AUXCTL_CMDLEN_MASK,
				size ? size - 1 : 0);
	tegra_dpaux_write_field(dp, DPAUX_DP_AUXCTL,
			DPAUX_DP_AUXCTL_ADDRESS_ONLY_MASK,
			(0 == size) ? DPAUX_DP_AUXCTL_ADDRESS_ONLY_TRUE :
				DPAUX_DP_AUXCTL_ADDRESS_ONLY_FALSE);

	tegra_dpaux_writel(dp, DPAUX_DP_AUXADDR, addr);
	for (i = 0; size && data && i < (DP_AUX_MAX_BYTES / 4); ++i)
		tegra_dpaux_writel(dp, DPAUX_DP_AUXDATA_WRITE_W(i), data[i]);

	return 0;
fail:
	return -EINVAL;
}

int tegra_dc_dpaux_write_chunk_locked(struct tegra_dc_dp_data *dp,
	u32 cmd, u32 addr, u8 *data, u32 *size, u32 *aux_stat)
{
	int err = 0;
	u32 timeout_retries = DP_AUX_TIMEOUT_MAX_TRIES;
	u32 defer_retries	= DP_AUX_DEFER_MAX_TRIES;

	WARN_ON(!mutex_is_locked(&dp->dpaux_lock));

	switch (cmd) {
	case DPAUX_DP_AUXCTL_CMD_I2CWR:
	case DPAUX_DP_AUXCTL_CMD_MOTWR:
	case DPAUX_DP_AUXCTL_CMD_AUXWR:
		break;
	default:
		dev_err(&dp->dc->ndev->dev,
			"dp: invalid aux write cmd: 0x%x\n", cmd);
		return -EINVAL;
	};

	err = tegra_dp_aux_tx_config(dp, cmd, addr, data, *size);
	if (err < 0) {
		dev_err(&dp->dc->ndev->dev, "dp: incorrect aux tx params\n");
		return err;
	}

	if (tegra_platform_is_silicon()) {
		*aux_stat = tegra_dpaux_readl(dp, DPAUX_DP_AUXSTAT);
		if (!(*aux_stat & DPAUX_DP_AUXSTAT_HPD_STATUS_PLUGGED)) {
			dev_err(&dp->dc->ndev->dev, "dp: HPD is not detected\n");
			return -EFAULT;
		}
	}

	while ((timeout_retries > 0) && (defer_retries > 0)) {
		if ((timeout_retries != DP_AUX_TIMEOUT_MAX_TRIES) ||
		    (defer_retries != DP_AUX_DEFER_MAX_TRIES))
			usleep_range(DP_DPCP_RETRY_SLEEP_NS,
				DP_DPCP_RETRY_SLEEP_NS << 1);

		tegra_dpaux_write_field(dp, DPAUX_DP_AUXCTL,
					DPAUX_DP_AUXCTL_TRANSACTREQ_MASK,
					DPAUX_DP_AUXCTL_TRANSACTREQ_PENDING);

		if (tegra_dpaux_wait_transaction(dp))
			dev_err(&dp->dc->ndev->dev,
				"dp: aux write transaction timeout\n");

		*aux_stat = tegra_dpaux_readl(dp, DPAUX_DP_AUXSTAT);

		/* Ignore I2C errors on fpga */
		if (tegra_platform_is_fpga())
			*aux_stat &= ~DPAUX_DP_AUXSTAT_REPLYTYPE_I2CNACK;

		if ((*aux_stat & DPAUX_DP_AUXSTAT_TIMEOUT_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_RX_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_SINKSTAT_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_NO_STOP_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_NACK) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_I2CNACK)) {
			if (timeout_retries-- > 0) {
				dev_info(&dp->dc->ndev->dev,
					"dp: aux write retry (0x%x) -- %d\n",
					*aux_stat, timeout_retries);
				/* clear the error bits */
				tegra_dpaux_writel(dp, DPAUX_DP_AUXSTAT,
					*aux_stat);
				continue;
			} else {
				dev_err(&dp->dc->ndev->dev,
					"dp: aux write got error (0x%x)\n",
					*aux_stat);
				return -EFAULT;
			}
		}

		if ((*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_I2CDEFER) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_DEFER)) {
			if (defer_retries-- > 0) {
				dev_info(&dp->dc->ndev->dev,
					"dp: aux write defer (0x%x) -- %d\n",
					*aux_stat, defer_retries);
				/* clear the error bits */
				tegra_dpaux_writel(dp, DPAUX_DP_AUXSTAT,
					*aux_stat);
				continue;
			} else {
				dev_err(&dp->dc->ndev->dev,
					"dp: aux write defer exceeds max retries "
					"(0x%x)\n",
					*aux_stat);
				return -EFAULT;
			}
		}

		if ((*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_MASK) ==
			DPAUX_DP_AUXSTAT_REPLYTYPE_ACK) {
			(*size)++;
			return 0;
		} else {
			dev_err(&dp->dc->ndev->dev,
				"dp: aux write failed (0x%x)\n", *aux_stat);
			return -EFAULT;
		}
	}
	/* Should never come to here */
	return -EFAULT;
}

int tegra_dc_dpaux_write(struct tegra_dc_dp_data *dp, u32 cmd, u32 addr,
	u8 *data, u32 *size, u32 *aux_stat)
{
	u32	cur_size = 0;
	u32	finished = 0;
	int	ret	 = 0;

	if (*size == 0) {
		dev_err(&dp->dc->ndev->dev,
			"dp: aux write size can't be 0\n");
		return -EINVAL;
	}

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return ret;

	mutex_lock(&dp->dpaux_lock);
	do {
		cur_size = *size - finished;
		if (cur_size > DP_AUX_MAX_BYTES)
			cur_size = DP_AUX_MAX_BYTES;

		ret = tegra_dc_dpaux_write_chunk_locked(dp, cmd, addr,
			data, &cur_size, aux_stat);

		finished += cur_size;
		addr += cur_size;
		data += cur_size;

		if (ret)
			break;
	} while (*size > finished);
	mutex_unlock(&dp->dpaux_lock);

	*size = finished;
	return ret;
}

int tegra_dc_dpaux_read_chunk_locked(struct tegra_dc_dp_data *dp,
	u32 cmd, u32 addr, u8 *data, u32 *size, u32 *aux_stat)
{
	int err = 0;
	u32 timeout_retries = DP_AUX_TIMEOUT_MAX_TRIES;
	u32 defer_retries	= DP_AUX_DEFER_MAX_TRIES;

	WARN_ON(!mutex_is_locked(&dp->dpaux_lock));

	switch (cmd) {
	case DPAUX_DP_AUXCTL_CMD_I2CRD:
	case DPAUX_DP_AUXCTL_CMD_I2CREQWSTAT:
	case DPAUX_DP_AUXCTL_CMD_MOTREQWSTAT:
	case DPAUX_DP_AUXCTL_CMD_MOTRD:
	case DPAUX_DP_AUXCTL_CMD_AUXRD:
		break;
	default:
		dev_err(&dp->dc->ndev->dev,
			"dp: invalid aux read cmd: 0x%x\n", cmd);
		return -EINVAL;
	};

	err = tegra_dp_aux_tx_config(dp, cmd, addr, NULL, *size);
	if (err < 0) {
		dev_err(&dp->dc->ndev->dev, "dp: incorrect aux tx params\n");
		return err;
	}

	if (tegra_platform_is_silicon()) {
		*aux_stat = tegra_dpaux_readl(dp, DPAUX_DP_AUXSTAT);
		if (!(*aux_stat & DPAUX_DP_AUXSTAT_HPD_STATUS_PLUGGED)) {
			dev_err(&dp->dc->ndev->dev, "dp: HPD is not detected\n");
			return -EFAULT;
		}
	}

	while ((timeout_retries > 0) && (defer_retries > 0)) {
		if ((timeout_retries != DP_AUX_TIMEOUT_MAX_TRIES) ||
		    (defer_retries != DP_AUX_DEFER_MAX_TRIES))
			usleep_range(DP_DPCP_RETRY_SLEEP_NS,
				DP_DPCP_RETRY_SLEEP_NS << 1);

		tegra_dpaux_write_field(dp, DPAUX_DP_AUXCTL,
					DPAUX_DP_AUXCTL_TRANSACTREQ_MASK,
					DPAUX_DP_AUXCTL_TRANSACTREQ_PENDING);

		if (tegra_dpaux_wait_transaction(dp))
			dev_err(&dp->dc->ndev->dev,
				"dp: aux read transaction timeout\n");

		*aux_stat = tegra_dpaux_readl(dp, DPAUX_DP_AUXSTAT);

		/* Ignore I2C errors on fpga */
		if (!tegra_platform_is_silicon())
			*aux_stat &= ~DPAUX_DP_AUXSTAT_REPLYTYPE_I2CNACK;

		if ((*aux_stat & DPAUX_DP_AUXSTAT_TIMEOUT_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_RX_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_SINKSTAT_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_NO_STOP_ERROR_PENDING)) {
			if (timeout_retries-- > 0) {
				dev_info(&dp->dc->ndev->dev,
					"dp: aux read retry (0x%x) -- %d\n",
					*aux_stat, timeout_retries);
				/* clear the error bits */
				tegra_dpaux_writel(dp, DPAUX_DP_AUXSTAT,
					*aux_stat);
				continue; /* retry */
			} else {
				dev_err(&dp->dc->ndev->dev,
					"dp: aux read got error (0x%x)\n",
					*aux_stat);
				return -EFAULT;
			}
		}

		if ((*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_I2CDEFER) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_DEFER)) {
			if (defer_retries-- > 0) {
				dev_info(&dp->dc->ndev->dev,
					"dp: aux read defer (0x%x) -- %d\n",
					*aux_stat, defer_retries);
				/* clear the error bits */
				tegra_dpaux_writel(dp, DPAUX_DP_AUXSTAT,
					*aux_stat);
				continue;
			} else {
				dev_err(&dp->dc->ndev->dev,
					"dp: aux read defer exceeds max retries "
					"(0x%x)\n", *aux_stat);
				return -EFAULT;
			}
		}

		if ((*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_MASK) ==
			DPAUX_DP_AUXSTAT_REPLYTYPE_ACK) {
			int i;
			u32 temp_data[4];

			for (i = 0; i < DP_AUX_MAX_BYTES/4; ++i)
				temp_data[i] = tegra_dpaux_readl(dp,
					DPAUX_DP_AUXDATA_READ_W(i));

			*size = ((*aux_stat) & DPAUX_DP_AUXSTAT_REPLY_M_MASK);
			memcpy(data, temp_data, *size);

			return 0;
		} else {
			dev_err(&dp->dc->ndev->dev,
				"dp: aux read failed (0x%x\n", *aux_stat);
			return -EFAULT;
		}
	}
	/* Should never come to here */
	return -EFAULT;
}

int tegra_dc_dpaux_read(struct tegra_dc_dp_data *dp, u32 cmd, u32 addr,
	u8 *data, u32 *size, u32 *aux_stat)
{
	u32	finished = 0;
	u32	cur_size;
	int	ret	 = 0;

	if (*size == 0) {
		dev_err(&dp->dc->ndev->dev,
			"dp: aux read size can't be 0\n");
		return -EINVAL;
	}

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return  ret;

	mutex_lock(&dp->dpaux_lock);
	do {
		cur_size = *size - finished;
		if (cur_size > DP_AUX_MAX_BYTES)
			cur_size = DP_AUX_MAX_BYTES;

		ret = tegra_dc_dpaux_read_chunk_locked(dp, cmd, addr,
			data, &cur_size, aux_stat);

		if (ret)
			break;

		/* cur_size should be the real size returned */
		addr += cur_size;
		data += cur_size;
		finished += cur_size;

	} while (*size > finished);
	mutex_unlock(&dp->dpaux_lock);

	*size = finished;
	return ret;
}

/* I2C read over DPAUX cannot handle more than 16B per transaction due to
 * DPAUX transaction limitation.
 * This requires breaking each read into multiple i2c write/read transaction */
static int tegra_dc_i2c_read(struct tegra_dc_dp_data *dp, u32 i2c_addr,
	u32 addr, u8 *data, u32 *size, u32 *aux_stat)
{
	u32	finished = 0;
	u32	cur_size;
	int	ret	 = 0;
	u32	len;
	u8	iaddr	 = (u8)addr;

	if (*size == 0) {
		dev_err(&dp->dc->ndev->dev,
			"dp: i2c read size can't be 0\n");
		return -EINVAL;
	}

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return ret;

	mutex_lock(&dp->dpaux_lock);
	do {
		cur_size = *size - finished;
		if (cur_size > DP_AUX_MAX_BYTES)
			cur_size = DP_AUX_MAX_BYTES;

		len = 1;
		ret = tegra_dc_dpaux_write_chunk_locked(dp,
			DPAUX_DP_AUXCTL_CMD_MOTWR,
			i2c_addr, &iaddr, &len, aux_stat);
		if (!ret) {
			ret = tegra_dc_dpaux_read_chunk_locked(dp,
				DPAUX_DP_AUXCTL_CMD_I2CRD,
				i2c_addr, data, &cur_size, aux_stat);
		}
		if (ret)
			break;

		iaddr += cur_size;
		data += cur_size;
		finished += cur_size;
	} while (*size > finished);
	mutex_unlock(&dp->dpaux_lock);

	*size = finished;
	return ret;
}

int tegra_dc_dp_dpcd_read(struct tegra_dc_dp_data *dp, u32 cmd,
	u8 *data_ptr)
{
	u32 size = 1;
	u32 status = 0;
	int ret = 0;

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return ret;

	mutex_lock(&dp->dpaux_lock);
	ret = tegra_dc_dpaux_read_chunk_locked(dp, DPAUX_DP_AUXCTL_CMD_AUXRD,
		cmd, data_ptr, &size, &status);
	mutex_unlock(&dp->dpaux_lock);
	if (ret)
		dev_err(&dp->dc->ndev->dev,
			"dp: Failed to read DPCD data. CMD 0x%x, Status 0x%x\n",
			cmd, status);

	return ret;
}

static int tegra_dc_dp_i2c_xfer(struct tegra_dc *dc, struct i2c_msg *msgs,
	int num)
{
	struct i2c_msg *pmsg;
	int i;
	u32 aux_stat;
	int status = 0;
	u32 len = 0;
	u32 start_addr;
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	for (i = 0; i < num; ++i) {
		pmsg = &msgs[i];

		if (!pmsg->flags) { /* write */
			/* Ignore the write-for-read command now as it is
			   already handled in the read operations */
		} else if (pmsg->flags & I2C_M_RD) { /* Read */
			len = pmsg->len;
			start_addr = 0;
			status = tegra_dc_i2c_read(dp, pmsg->addr, start_addr,
				pmsg->buf, &len, &aux_stat);
			if (status) {
				dev_err(&dp->dc->ndev->dev,
					"dp: Failed for I2C read"
					" addr:%d, size:%d, stat:0x%x\n",
					pmsg->addr, len, aux_stat);
				return status;
			}
		} else {
			/* No other functionalities are supported for now */
			dev_err(&dp->dc->ndev->dev,
				"dp: i2x_xfer: Unknown flag 0x%x\n",
				pmsg->flags);
			return -EINVAL;
		}
	}
	return i;
}

static i2c_transfer_func_t tegra_dp_hpd_op_edid_read(void *drv_data)
{
	struct tegra_dc_dp_data *dp = drv_data;

	return (dp->edid_src == EDID_SRC_DT) ?
		tegra_dc_edid_blob : tegra_dc_dp_i2c_xfer;
}

int tegra_dc_dp_dpcd_write(struct tegra_dc_dp_data *dp, u32 cmd,
	u8 data)
{
	u32 size = 1;
	u32 status = 0;
	int ret;

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return 0;

	mutex_lock(&dp->dpaux_lock);
	ret = tegra_dc_dpaux_write_chunk_locked(dp, DPAUX_DP_AUXCTL_CMD_AUXWR,
		cmd, &data, &size, &status);
	mutex_unlock(&dp->dpaux_lock);
	if (ret)
		dev_err(&dp->dc->ndev->dev,
			"dp: Failed to write DPCD data. CMD 0x%x, Status 0x%x\n",
			cmd, status);
	return ret;
}

static inline int tegra_dp_dpcd_write_field(struct tegra_dc_dp_data *dp,
					u32 cmd, u8 mask, u8 data)
{
	u8 dpcd_data;
	int ret;

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return 0;

	might_sleep();

	CHECK_RET(tegra_dc_dp_dpcd_read(dp, cmd, &dpcd_data));
	dpcd_data &= ~mask;
	dpcd_data |= data;
	CHECK_RET(tegra_dc_dp_dpcd_write(dp, cmd, dpcd_data));

	return 0;
}

static inline u64 tegra_div64(u64 dividend, u32 divisor)
{
	do_div(dividend, divisor);
	return dividend;
}

#ifdef CONFIG_DEBUG_FS
static int dbg_dp_show(struct seq_file *s, void *unused)
{
#define DUMP_REG(a) seq_printf(s, "%-32s  %03x	%08x\n",	\
		#a, a, tegra_dpaux_readl(dp, a))

	struct tegra_dc_dp_data *dp = s->private;

	tegra_dc_io_start(dp->dc);
	tegra_dpaux_clk_enable(dp);

	DUMP_REG(DPAUX_INTR_EN_AUX);
	DUMP_REG(DPAUX_INTR_AUX);
	DUMP_REG(DPAUX_DP_AUXADDR);
	DUMP_REG(DPAUX_DP_AUXCTL);
	DUMP_REG(DPAUX_DP_AUXSTAT);
	DUMP_REG(DPAUX_HPD_CONFIG);
	DUMP_REG(DPAUX_HPD_IRQ_CONFIG);
	DUMP_REG(DPAUX_DP_AUX_CONFIG);
	DUMP_REG(DPAUX_HYBRID_PADCTL);
	DUMP_REG(DPAUX_HYBRID_SPARE);

	tegra_dpaux_clk_disable(dp);
	tegra_dc_io_end(dp->dc);

	return 0;
}

static int dbg_dp_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dp_show, inode->i_private);
}

static const struct file_operations dbg_fops = {
	.open		= dbg_dp_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int lane_count_show(struct seq_file *s, void *unused)
{
	struct tegra_dc_dp_data *dp = s->private;
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;

	seq_puts(s, "\n");
	seq_printf(s,
		"\tDP Lane_Count: \t%d\n",
		cfg->lane_count);
	return 0;
}

static ssize_t lane_count_set(struct file *file, const char __user *buf,
						size_t count, loff_t *off)
{
	struct seq_file *s = file->private_data;
	struct tegra_dc_dp_data *dp = s->private;
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	long lane_count = 0;
	int ret = 0;

	if (dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP)
		return -EINVAL;

	ret = kstrtol_from_user(buf, count, 10, &lane_count);
	if (ret < 0)
		return ret;

	if (cfg->lane_count == lane_count)
		return -EINVAL;

	/* disable the dc and output controllers */
	if (dp->dc->enabled)
		tegra_dc_disable(dp->dc);

	dev_info(&dp->dc->ndev->dev, "Setting the lanecount from %d to %ld\n",
			cfg->lane_count, lane_count);

	cfg->lane_count = lane_count;

	/* check if needed or not for validity purpose */
	ret = tegra_dc_dp_calc_config(dp, dp->mode, cfg);
	if (!ret)
		dev_info(&dp->dc->ndev->dev,
			"Unable to set lane_count properly\n");

	/* disable the dc and output controllers */
	if (!dp->dc->enabled)
		tegra_dc_enable(dp->dc);

	return count;
}

static int lane_count_open(struct inode *inode, struct file *file)
{
	return single_open(file, lane_count_show, inode->i_private);
}

static const struct file_operations lane_count_fops = {
	.open		= lane_count_open,
	.read		= seq_read,
	.write		= lane_count_set,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int link_speed_show(struct seq_file *s, void *unused)
{
	struct tegra_dc_dp_data *dp = s->private;
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;

	seq_puts(s, "\n");
	seq_printf(s,
		"\tDP Link Speed: \t%d\n",
		cfg->link_bw);
	return 0;
}

static ssize_t link_speed_set(struct file *file, const char __user *buf,
						size_t count, loff_t *off)
{
	struct seq_file *s = file->private_data;
	struct tegra_dc_dp_data *dp = s->private;
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	long link_speed = 0;
	int ret = 0;

	if (dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP)
		return -EINVAL;

	ret = kstrtol_from_user(buf, count, 10, &link_speed);
	if (ret < 0)
		return ret;

	if (cfg->link_bw == link_speed)
		return -EINVAL;

	/* disable the dc and output controllers */
	if (dp->dc->enabled)
		tegra_dc_disable(dp->dc);

	dev_info(&dp->dc->ndev->dev, "Setting the linkspeed from %d to %ld\n",
			cfg->link_bw, link_speed);

	cfg->link_bw = link_speed;

	/* check if needed or not for validity purpose */
	ret = tegra_dc_dp_calc_config(dp, dp->mode, cfg);
	if (!ret)
		dev_info(&dp->dc->ndev->dev,
			"Unable to set linkspeed properly\n");

	/* disable the dc and output controllers */
	if (!dp->dc->enabled)
		tegra_dc_enable(dp->dc);

	return count;
}

static int link_speed_open(struct inode *inode, struct file *file)
{
	return single_open(file, link_speed_show, inode->i_private);
}

static const struct file_operations link_speed_fops = {
	.open		= link_speed_open,
	.read		= seq_read,
	.write		= link_speed_set,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_hotplug_show(struct seq_file *m, void *unused)
{
	struct tegra_dc_dp_data *dp = m->private;
	struct tegra_dc *dc = dp->dc;

	if (WARN_ON(!dp || !dc || !dc->out))
		return -EINVAL;

	seq_printf(m, "dp hpd state: %d\n", dc->out->hotplug_state);
	return 0;
}

static int dbg_hotplug_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_hotplug_show, inode->i_private);
}

/*
 * sw control for hpd.
 * 0 is normal state, hw drives hpd.
 * -1 is force deassert, sw drives hpd.
 * 1 is force assert, sw drives hpd.
 * before releasing to hw, sw must ensure hpd state is normal i.e. 0
 */
static ssize_t dbg_hotplug_write(struct file *file, const char __user *addr,
	size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data; /* single_open() initialized */
	struct tegra_dc_dp_data *dp = m->private;
	struct tegra_dc *dc = dp->dc;
	int ret;
	long new_state;

	if (WARN_ON(!dp || !dc || !dc->out))
		return -EINVAL;

	ret = kstrtol_from_user(addr, len, 10, &new_state);
	if (ret < 0)
		return ret;

	if (dc->out->hotplug_state == TEGRA_HPD_STATE_NORMAL
		&& new_state != TEGRA_HPD_STATE_NORMAL
		&& tegra_dc_hotplug_supported(dc)) {
		/* SW controlled hotplug. Ignore hpd HW interrupts. */
		tegra_dp_int_dis(dp, DPAUX_INTR_EN_AUX_PLUG_EVENT |
				DPAUX_INTR_EN_AUX_UNPLUG_EVENT |
				DPAUX_INTR_EN_AUX_PLUG_EVENT);
	} else if (dc->out->hotplug_state != TEGRA_HPD_STATE_NORMAL
		&& new_state == TEGRA_HPD_STATE_NORMAL
		&& tegra_dc_hotplug_supported(dc)) {
		/* Enable hpd HW interrupts */
		tegra_dp_int_en(dp, DPAUX_INTR_EN_AUX_PLUG_EVENT |
				DPAUX_INTR_EN_AUX_UNPLUG_EVENT |
				DPAUX_INTR_EN_AUX_PLUG_EVENT);
	}

	dc->out->hotplug_state = new_state;

	tegra_dp_pending_hpd(dp);

	return len;
}

static const struct file_operations dbg_hotplug_fops = {
	.open = dbg_hotplug_open,
	.read = seq_read,
	.write = dbg_hotplug_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct dentry *dpdir;

static void tegra_dc_dp_debug_create(struct tegra_dc_dp_data *dp)
{
	struct dentry *retval;

	dpdir = debugfs_create_dir("tegra_dp", NULL);
	if (!dpdir)
		return;
	retval = debugfs_create_file("regs", S_IRUGO, dpdir, dp, &dbg_fops);
	if (!retval)
		goto free_out;
	retval = debugfs_create_file("lanes", S_IRUGO, dpdir, dp,
		&lane_count_fops);
	if (!retval)
		goto free_out;
	retval = debugfs_create_file("linkspeed", S_IRUGO, dpdir, dp,
		&link_speed_fops);
	if (!retval)
		goto free_out;

	/* hotplug not allowed for eDP */
	if (is_hotplug_supported(dp)) {
		retval = debugfs_create_file("hotplug", S_IRUGO, dpdir, dp,
			&dbg_hotplug_fops);
		if (!retval)
			goto free_out;
	}

	return;
free_out:
	debugfs_remove_recursive(dpdir);
	dpdir = NULL;
	return;
}
#else
static inline void tegra_dc_dp_debug_create(struct tegra_dc_dp_data *dp)
{ }
#endif

static void tegra_dpaux_enable(struct tegra_dc_dp_data *dp)
{
	/* do not enable interrupt for now. */
	tegra_dpaux_writel(dp, DPAUX_INTR_EN_AUX, 0x0);

	/* clear interrupt */
	tegra_dpaux_writel(dp, DPAUX_INTR_AUX, 0xffffffff);

	tegra_dpaux_writel(dp, DPAUX_HYBRID_PADCTL,
		DPAUX_HYBRID_PADCTL_AUX_DRVZ_OHM_50 |
		DPAUX_HYBRID_PADCTL_AUX_CMH_V0_70 |
		0x18 << DPAUX_HYBRID_PADCTL_AUX_DRVI_SHIFT |
		DPAUX_HYBRID_PADCTL_AUX_INPUT_RCV_ENABLE);

	tegra_dpaux_pad_power(dp->dc,
		dp->dc->ndev->id == 0 ? TEGRA_DPAUX_INSTANCE_0 :
		TEGRA_DPAUX_INSTANCE_1, true);
}

static int tegra_dp_panel_power_state(struct tegra_dc_dp_data *dp, u8 state)
{
	u32 retry = 0;
	int ret;

	do {
		ret = tegra_dc_dp_dpcd_write(dp, NV_DPCD_SET_POWER, state);
	} while ((retry++ < DP_POWER_ON_MAX_TRIES) && ret);

	return ret;
}

__maybe_unused
static void tegra_dc_dp_dump_link_cfg(struct tegra_dc_dp_data *dp,
	const struct tegra_dc_dp_link_config *cfg)
{
	if (!tegra_dp_debug)
		return;

	BUG_ON(!cfg);

	dev_info(&dp->dc->ndev->dev, "DP config: cfg_name               "
		"cfg_value\n");
	dev_info(&dp->dc->ndev->dev, "           Lane Count             %d\n",
		cfg->max_lane_count);
	dev_info(&dp->dc->ndev->dev, "           SupportEnhancedFraming %s\n",
		cfg->support_enhanced_framing ? "Y" : "N");
	dev_info(&dp->dc->ndev->dev, "           SupportAltScrmbRstFffe %s\n",
		cfg->alt_scramber_reset_cap ? "Y" : "N");
	dev_info(&dp->dc->ndev->dev, "           Bandwidth              %d\n",
		cfg->max_link_bw);
	dev_info(&dp->dc->ndev->dev, "           bpp                    %d\n",
		cfg->bits_per_pixel);
	dev_info(&dp->dc->ndev->dev, "           EnhancedFraming        %s\n",
		cfg->enhanced_framing ? "Y" : "N");
	dev_info(&dp->dc->ndev->dev, "           Scramble_enabled       %s\n",
		cfg->scramble_ena ? "Y" : "N");
	dev_info(&dp->dc->ndev->dev, "           LinkBW                 %d\n",
		cfg->link_bw);
	dev_info(&dp->dc->ndev->dev, "           lane_count             %d\n",
		cfg->lane_count);
	dev_info(&dp->dc->ndev->dev, "           activespolarity        %d\n",
		cfg->activepolarity);
	dev_info(&dp->dc->ndev->dev, "           active_count           %d\n",
		cfg->active_count);
	dev_info(&dp->dc->ndev->dev, "           tu_size                %d\n",
		cfg->tu_size);
	dev_info(&dp->dc->ndev->dev, "           active_frac            %d\n",
		cfg->active_frac);
	dev_info(&dp->dc->ndev->dev, "           watermark              %d\n",
		cfg->watermark);
	dev_info(&dp->dc->ndev->dev, "           hblank_sym             %d\n",
		cfg->hblank_sym);
	dev_info(&dp->dc->ndev->dev, "           vblank_sym             %d\n",
		cfg->vblank_sym);
};

/* Calcuate if given cfg can meet the mode request. */
/* Return true if mode is possible, false otherwise. */
bool tegra_dc_dp_calc_config(struct tegra_dc_dp_data *dp,
	const struct tegra_dc_mode *mode,
	struct tegra_dc_dp_link_config *cfg)
{
	const u32	link_rate = 27 * cfg->link_bw * 1000 * 1000;
	const u64	f	  = 100000;	/* precision factor */

	u32	num_linkclk_line; /* Number of link clocks per line */
	u64	ratio_f; /* Ratio of incoming to outgoing data rate */
	u64	frac_f;
	u64	activesym_f;	/* Activesym per TU */
	u64	activecount_f;
	u32	activecount;
	u32	activepolarity;
	u64	approx_value_f;
	u32	activefrac		  = 0;
	u64	accumulated_error_f	  = 0;
	u32	lowest_neg_activecount	  = 0;
	u32	lowest_neg_activepolarity = 0;
	u32	lowest_neg_tusize	  = 64;
	u32	num_symbols_per_line;
	u64	lowest_neg_activefrac	  = 0;
	u64	lowest_neg_error_f	  = 64 * f;
	u64	watermark_f;

	int	i;
	bool	neg;
	unsigned long rate;

	cfg->is_valid = false;

	rate = tegra_dc_pclk_round_rate(dp->sor->dc, dp->sor->dc->mode.pclk);

	if (!link_rate || !cfg->lane_count || !rate ||
		!cfg->bits_per_pixel)
		return false;

	if ((u64)rate * cfg->bits_per_pixel >=
		(u64)link_rate * 8 * cfg->lane_count) {
		dev_dbg(&dp->dc->ndev->dev,
			"Requested rate calc > link_rate calc\n");
		return false;
	}

	num_linkclk_line = (u32)tegra_div64(
		(u64)link_rate * mode->h_active, rate);

	ratio_f = (u64)rate * cfg->bits_per_pixel * f;
	ratio_f /= 8;
	ratio_f = tegra_div64(ratio_f, link_rate * cfg->lane_count);

	for (i = 64; i >= 32; --i) {
		activesym_f	= ratio_f * i;
		activecount_f	= tegra_div64(activesym_f, (u32)f) * f;
		frac_f		= activesym_f - activecount_f;
		activecount	= (u32)tegra_div64(activecount_f, (u32)f);

		if (frac_f < (f / 2)) /* fraction < 0.5 */
			activepolarity = 0;
		else {
			activepolarity = 1;
			frac_f = f - frac_f;
		}

		if (frac_f != 0) {
			frac_f = tegra_div64((f * f),  frac_f); /* 1/fraction */
			if (frac_f > (15 * f))
				activefrac = activepolarity ? 1 : 15;
			else
				activefrac = activepolarity ?
					(u32)tegra_div64(frac_f, (u32)f) + 1 :
					(u32)tegra_div64(frac_f, (u32)f);
		}

		if (activefrac == 1)
			activepolarity = 0;

		if (activepolarity == 1)
			approx_value_f = activefrac ? tegra_div64(
				activecount_f + (activefrac * f - f) * f,
				(activefrac * f)) :
				activecount_f + f;
		else
			approx_value_f = activefrac ?
				activecount_f + tegra_div64(f, activefrac) :
				activecount_f;

		if (activesym_f < approx_value_f) {
			accumulated_error_f = num_linkclk_line *
				tegra_div64(approx_value_f - activesym_f, i);
			neg = true;
		} else {
			accumulated_error_f = num_linkclk_line *
				tegra_div64(activesym_f - approx_value_f, i);
			neg = false;
		}

		if ((neg && (lowest_neg_error_f > accumulated_error_f)) ||
			(accumulated_error_f == 0)) {
			lowest_neg_error_f = accumulated_error_f;
			lowest_neg_tusize = i;
			lowest_neg_activecount = activecount;
			lowest_neg_activepolarity = activepolarity;
			lowest_neg_activefrac = activefrac;

			if (accumulated_error_f == 0)
				break;
		}
	}

	if (lowest_neg_activefrac == 0) {
		cfg->activepolarity = 0;
		cfg->active_count   = lowest_neg_activepolarity ?
			lowest_neg_activecount : lowest_neg_activecount - 1;
		cfg->tu_size	      = lowest_neg_tusize;
		cfg->active_frac    = 1;
	} else {
		cfg->activepolarity = lowest_neg_activepolarity;
		cfg->active_count   = (u32)lowest_neg_activecount;
		cfg->tu_size	      = lowest_neg_tusize;
		cfg->active_frac    = (u32)lowest_neg_activefrac;
	}

	dev_dbg(&dp->dc->ndev->dev,
		"dp: sor configuration: polarity: %d active count: %d "
		"tu size: %d, active frac: %d\n",
		cfg->activepolarity, cfg->active_count, cfg->tu_size,
		cfg->active_frac);

	watermark_f = tegra_div64(ratio_f * cfg->tu_size * (f - ratio_f), f);
	cfg->watermark = (u32)tegra_div64(watermark_f + lowest_neg_error_f,
		f) + cfg->bits_per_pixel / 4 - 1;
	num_symbols_per_line = (mode->h_active * cfg->bits_per_pixel) /
		(8 * cfg->lane_count);
	if (cfg->watermark > 30) {
		dev_dbg(&dp->dc->ndev->dev,
			"dp: sor setting: unable to get a good tusize, "
			"force watermark to 30.\n");
		cfg->watermark = 30;
		return false;
	} else if (cfg->watermark > num_symbols_per_line) {
		dev_dbg(&dp->dc->ndev->dev,
			"dp: sor setting: force watermark to the number "
			"of symbols in the line.\n");
		cfg->watermark = num_symbols_per_line;
		return false;
	}

	/* Refer to dev_disp.ref for more information. */
	/* # symbols/hblank = ((SetRasterBlankEnd.X + SetRasterSize.Width - */
	/*                      SetRasterBlankStart.X - 7) * link_clk / pclk) */
	/*                      - 3 * enhanced_framing - Y */
	/* where Y = (# lanes == 4) 3 : (# lanes == 2) ? 6 : 12 */
	cfg->hblank_sym = (int)tegra_div64((u64)(mode->h_back_porch +
			mode->h_front_porch + mode->h_sync_width - 7)
		* link_rate, rate)
		- 3 * cfg->enhanced_framing - (12 / cfg->lane_count);

	if (cfg->hblank_sym < 0)
		cfg->hblank_sym = 0;


	/* Refer to dev_disp.ref for more information. */
	/* # symbols/vblank = ((SetRasterBlankStart.X - */
	/*                      SetRasterBlankEen.X - 25) * link_clk / pclk) */
	/*                      - Y - 1; */
	/* where Y = (# lanes == 4) 12 : (# lanes == 2) ? 21 : 39 */
	cfg->vblank_sym = (int)tegra_div64((u64)(mode->h_active - 25)
		* link_rate, rate) - (36 / cfg->lane_count) - 4;

	if (cfg->vblank_sym < 0)
		cfg->vblank_sym = 0;

	cfg->is_valid = true;

	return true;
}

static int tegra_dp_init_max_link_cfg(struct tegra_dc_dp_data *dp,
					struct tegra_dc_dp_link_config *cfg)
{

#ifdef CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT
	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		tegra_dc_init_fake_panel_link_cfg(cfg);
	else
#endif
	{
		u8 dpcd_data;
		int ret;

		CHECK_RET(tegra_dc_dp_dpcd_read(dp, NV_DPCD_MAX_LANE_COUNT,
			&dpcd_data));

		cfg->max_lane_count = dpcd_data & NV_DPCD_MAX_LANE_COUNT_MASK;
		cfg->tps3_supported =
		(dpcd_data & NV_DPCD_MAX_LANE_COUNT_TPS3_SUPPORTED_YES) ?
		true : false;

		cfg->support_enhanced_framing =
		(dpcd_data & NV_DPCD_MAX_LANE_COUNT_ENHANCED_FRAMING_YES) ?
		true : false;

		CHECK_RET(tegra_dc_dp_dpcd_read(dp, NV_DPCD_MAX_DOWNSPREAD,
			&dpcd_data));
		cfg->downspread =
		(dpcd_data & NV_DPCD_MAX_DOWNSPREAD_VAL_0_5_PCT) ?
		true : false;

		cfg->support_fast_lt = (dpcd_data &
		NV_DPCD_MAX_DOWNSPREAD_NO_AUX_HANDSHAKE_LT_T) ? true : false;

		CHECK_RET(tegra_dc_dp_dpcd_read(dp,
			NV_DPCD_TRAINING_AUX_RD_INTERVAL, &dpcd_data));
		cfg->aux_rd_interval = dpcd_data;

		CHECK_RET(tegra_dc_dp_dpcd_read(dp, NV_DPCD_MAX_LINK_BANDWIDTH,
			&cfg->max_link_bw));

		CHECK_RET(tegra_dc_dp_dpcd_read(dp, NV_DPCD_EDP_CONFIG_CAP,
			&dpcd_data));
		cfg->alt_scramber_reset_cap =
		(dpcd_data & NV_DPCD_EDP_CONFIG_CAP_ASC_RESET_YES) ?
		true : false;

		cfg->only_enhanced_framing =
		(dpcd_data & NV_DPCD_EDP_CONFIG_CAP_FRAMING_CHANGE_YES) ?
		true : false;

		cfg->edp_cap = (dpcd_data &
		NV_DPCD_EDP_CONFIG_CAP_DISPLAY_CONTROL_CAP_YES) ? true : false;
	}

	cfg->bits_per_pixel = dp->dc->out->depth ? : 24;

	cfg->lane_count = cfg->max_lane_count;

	cfg->link_bw = (dp->pdata && dp->pdata->link_bw) ?
			dp->pdata->link_bw : cfg->max_link_bw;

	cfg->enhanced_framing = cfg->support_enhanced_framing;

	tegra_dc_dp_calc_config(dp, dp->mode, cfg);

	dp->max_link_cfg = *cfg;
	return 0;
}

static int tegra_dc_dp_set_assr(struct tegra_dc_dp_data *dp, bool ena)
{
	int ret;

	u8 dpcd_data = ena ?
		NV_DPCD_EDP_CONFIG_SET_ASC_RESET_ENABLE :
		NV_DPCD_EDP_CONFIG_SET_ASC_RESET_DISABLE;

	CHECK_RET(tegra_dc_dp_dpcd_write(dp, NV_DPCD_EDP_CONFIG_SET,
			dpcd_data));

	/* Also reset the scrambler to 0xfffe */
	tegra_dc_sor_set_internal_panel(dp->sor, ena);
	return 0;
}


static int tegra_dp_set_link_bandwidth(struct tegra_dc_dp_data *dp, u8 link_bw)
{
	tegra_dc_sor_set_link_bandwidth(dp->sor, link_bw);

	/* Sink side */
	return tegra_dc_dp_dpcd_write(dp, NV_DPCD_LINK_BANDWIDTH_SET, link_bw);
}

static int tegra_dp_set_enhanced_framing(struct tegra_dc_dp_data *dp,
						bool enable)
{
	int ret;

	if (enable) {
		tegra_sor_write_field(dp->sor,
			NV_SOR_DP_LINKCTL(dp->sor->portnum),
			NV_SOR_DP_LINKCTL_ENHANCEDFRAME_ENABLE,
			NV_SOR_DP_LINKCTL_ENHANCEDFRAME_ENABLE);

		CHECK_RET(tegra_dp_dpcd_write_field(dp, NV_DPCD_LANE_COUNT_SET,
				NV_DPCD_LANE_COUNT_SET_ENHANCEDFRAMING_T,
				NV_DPCD_LANE_COUNT_SET_ENHANCEDFRAMING_T));
	}

	return 0;
}

static int tegra_dp_set_lane_count(struct tegra_dc_dp_data *dp, u8 lane_cnt)
{
	int ret;

	tegra_sor_power_lanes(dp->sor, lane_cnt, true);

	CHECK_RET(tegra_dp_dpcd_write_field(dp, NV_DPCD_LANE_COUNT_SET,
				NV_DPCD_LANE_COUNT_SET_MASK,
				lane_cnt));

	return 0;
}

static void tegra_dp_link_cal(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_sor_data *sor = dp->sor;
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	u32 load_adj;

	switch (cfg->link_bw) {
	case SOR_LINK_SPEED_G1_62:
		load_adj = 0x3;
		break;
	case SOR_LINK_SPEED_G2_7:
		load_adj = 0x4;
		break;
	case SOR_LINK_SPEED_G5_4:
		load_adj = 0x6;
		break;
	default:
		BUG();
	}

	tegra_sor_write_field(sor, NV_SOR_PLL1,
			NV_SOR_PLL1_LOADADJ_DEFAULT_MASK,
			load_adj << NV_SOR_PLL1_LOADADJ_SHIFT);
}

static void tegra_dp_irq_evt_worker(struct work_struct *work)
{
#define LANE0_1_CR_CE_SL_MASK (0x7 | (0x7 << 4))
#define LANE0_CR_CE_SL_MASK (0x7)
#define INTERLANE_ALIGN_MASK (0x1)

	struct tegra_dc_dp_data *dp = container_of(to_delayed_work(work),
					struct tegra_dc_dp_data,
					irq_evt_dwork);
	u32 aux_stat = tegra_dpaux_readl(dp, DPAUX_DP_AUXSTAT);
	bool link_stable = !!true;
	u8 dpcd_200h_205h[6];
	u32 n_lanes = dp->lt_data.n_lanes;

	tegra_dc_io_start(dp->dc);

	if (aux_stat & DPAUX_DP_AUXSTAT_SINKSTAT_ERROR_PENDING) {
		int cnt;

		/*
		 * HW failed to automatically read DPCD
		 * offsets 0x200-0x205. Initiate SW transaction.
		 */
		for (cnt = 0; cnt < 6; cnt++) {
			tegra_dc_dp_dpcd_read(dp, 0x200 + cnt,
						&dpcd_200h_205h[cnt]);
		}
	} else  {
		u32 aux_sinkstat_lo = tegra_dpaux_readl(dp,
					DPAUX_DP_AUX_SINKSTATLO);
		u32 aux_sinkstat_hi = tegra_dpaux_readl(dp,
					DPAUX_DP_AUX_SINKSTATHI);

		dpcd_200h_205h[0] = aux_sinkstat_lo & 0xff;
		dpcd_200h_205h[1] = (aux_sinkstat_lo >> 8) & 0xff;
		dpcd_200h_205h[2] = (aux_sinkstat_lo >> 16) & 0xff;
		dpcd_200h_205h[3] = (aux_sinkstat_lo >> 24) & 0xff;
		dpcd_200h_205h[4] = aux_sinkstat_hi & 0xff;
		dpcd_200h_205h[5] = (aux_sinkstat_hi >> 8) & 0xff;
	}

	switch (n_lanes) {
	case 4:
		link_stable &= !!((dpcd_200h_205h[3] &
				LANE0_1_CR_CE_SL_MASK) ==
				LANE0_1_CR_CE_SL_MASK);
	case 2:
		link_stable &= !!((dpcd_200h_205h[2] &
				LANE0_1_CR_CE_SL_MASK) ==
				LANE0_1_CR_CE_SL_MASK);
	case 1:
		link_stable &= !!((dpcd_200h_205h[2] &
				LANE0_CR_CE_SL_MASK) ==
				LANE0_CR_CE_SL_MASK);
	default:
		link_stable &= !!(dpcd_200h_205h[4] &
				INTERLANE_ALIGN_MASK);
	}

	if (!link_stable)
		tegra_dp_lt_set_pending_evt(&dp->lt_data);
	else
		dev_info(&dp->dc->ndev->dev,
			"dp: link stable, ignore irq event\n");

	tegra_dc_io_end(dp->dc);

#undef LANE0_1_CR_CE_SL_MASK
#undef LANE0_CR_CE_SL_MASK
#undef INTERLANE_ALIGN_MASK
}

static irqreturn_t tegra_dp_irq(int irq, void *ptr)
{
	struct tegra_dc_dp_data *dp = ptr;
	struct tegra_dc *dc = dp->dc;
	u32 status;

	if (tegra_platform_is_fpga())
		return IRQ_NONE;

	if (dp->suspended) {
		dev_info(&dc->ndev->dev,
			"dp: irq received while suspended, ignoring\n");
		return IRQ_HANDLED;
	}

	tegra_dc_io_start(dc);

	/* clear pending bits */
	status = tegra_dpaux_readl(dp, DPAUX_INTR_AUX);
	tegra_dpaux_writel(dp, DPAUX_INTR_AUX, status);

	tegra_dc_io_end(dc);

	if (status & (DPAUX_INTR_AUX_PLUG_EVENT_PENDING |
		DPAUX_INTR_AUX_UNPLUG_EVENT_PENDING)) {
		if (status & DPAUX_INTR_AUX_PLUG_EVENT_PENDING) {
			dev_info(&dp->dc->ndev->dev,
				"dp: plug event received\n");
			complete_all(&dp->hpd_plug);
		} else {
			dev_info(&dp->dc->ndev->dev,
				"dp: unplug event received\n");
			INIT_COMPLETION(dp->hpd_plug);
		}
		tegra_dp_pending_hpd(dp);
	} else if (status & DPAUX_INTR_AUX_IRQ_EVENT_PENDING) {
		dev_info(&dp->dc->ndev->dev, "dp: irq event received%s\n",
			dp->enabled ? "" : ", ignoring");
		if (dp->enabled) {
			cancel_delayed_work(&dp->irq_evt_dwork);
			schedule_delayed_work(&dp->irq_evt_dwork,
						msecs_to_jiffies(
						HPD_IRQ_EVENT_TIMEOUT_MS));
		}
	}

	if (status & DPAUX_INTR_AUX_TX_DONE_PENDING)
		complete_all(&dp->aux_tx);

	return IRQ_HANDLED;
}

static void _tegra_dpaux_init(struct tegra_dc_dp_data *dp)
{
	if (dp->sor->safe_clk)
		tegra_sor_safe_clk_enable(dp->sor);
	tegra_dpaux_clk_enable(dp);

	tegra_dc_io_start(dp->dc);

	tegra_dp_reset(dp);

	tegra_dpaux_enable(dp);

	if (dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		tegra_dp_hpd_config(dp);
		tegra_dp_default_int(dp, true);
	}

	tegra_dc_io_end(dp->dc);
}

static void tegra_dpaux_init(struct tegra_dc_dp_data *dp)
{
	BUG_ON(!dp->dc || !dp);

	tegra_set_dpaux_addr(dp->aux_base, dp->dc->ndev->id);

	_tegra_dpaux_init(dp);

	if (dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP)
		tegra_dp_enable_irq(dp->irq);
}

static int tegra_dc_dp_hotplug_init(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	/* SOR partition must be awake for dpaux */
	tegra_dc_unpowergate_locked(dc);

	/*
	 * dp interrupts are received by dpaux.
	 * Initialize dpaux to receive hotplug events.
	 */
	tegra_dpaux_init(dp);

	return 0;
}

static int tegra_dc_dp_init(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp;
	struct resource *res;
	struct resource *base_res;
	struct resource of_dp_res;
	void __iomem *base;
	struct clk *clk;
	struct clk *parent_clk;
	int err;
	u32 irq;
	struct device_node *np = dc->ndev->dev.of_node;

	struct device_node *np_dp =
		(dc->ndev->id) ? of_find_node_by_path(DPAUX1_NODE)
		: of_find_node_by_path(DPAUX_NODE);
	struct device_node *np_panel = NULL;

	dp = devm_kzalloc(&dc->ndev->dev, sizeof(*dp), GFP_KERNEL);
	if (!dp) {
		of_node_put(np_dp);
		return -ENOMEM;
	}

	dp->edid_src = EDID_SRC_PANEL;
	if (np) {
		if (np_dp && (of_device_is_available(np_dp) ||
			(dc->out->type == TEGRA_DC_OUT_FAKE_DP))) {
			irq = of_irq_to_resource(np_dp, 0, NULL);
			if (!irq) {
				err = -ENOENT;
				goto err_free_dp;
			}
			of_address_to_resource(np_dp, 0, &of_dp_res);
			res = &of_dp_res;
			np_panel = tegra_get_panel_node_out_type_check(dc,
				TEGRA_DC_OUT_DP);
			if (np_panel && of_device_is_available(np_panel)) {
				if (of_property_read_bool(np_panel,
					"nvidia,edid"))
					dp->edid_src = EDID_SRC_DT;
			}
		} else {
			err = -EINVAL;
			goto err_free_dp;
		}
	} else {
		irq = platform_get_irq_byname(dc->ndev, "irq_dp");
		if (irq <= 0) {
			dev_err(&dc->ndev->dev, "dp: no irq\n");
			err = -ENOENT;
			goto err_free_dp;
		}
		res = platform_get_resource_byname(dc->ndev,
			IORESOURCE_MEM, "dpaux");
	}
	if (!res) {
		dev_err(&dc->ndev->dev, "dp: no mem resources for dpaux\n");
		err = -EFAULT;
		goto err_free_dp;
	}

	base_res = devm_request_mem_region(&dc->ndev->dev,
		res->start, resource_size(res),
		dc->ndev->name);
	if (!base_res) {
		dev_err(&dc->ndev->dev, "dp: request_mem_region failed\n");
		err = -EFAULT;
		goto err_free_dp;
	}

	base = devm_ioremap(&dc->ndev->dev, res->start, resource_size(res));
	if (!base) {
		dev_err(&dc->ndev->dev, "dp: registers can't be mapped\n");
		err = -EFAULT;
		goto err_release_resource_reg;
	}


#ifdef CONFIG_TEGRA_NVDISPLAY
	clk = clk_get(NULL, dc->ndev->id ? "dpaux1" : "dpaux");
#else
	clk = clk_get_sys(dc->ndev->id ? "dpaux1" : "dpaux", NULL);
#endif
	if (IS_ERR_OR_NULL(clk)) {
		dev_err(&dc->ndev->dev, "dp: dc clock %s.edp unavailable\n",
			dev_name(&dc->ndev->dev));
		err = -EFAULT;
		goto err_iounmap_reg;
	}

#ifdef CONFIG_TEGRA_NVDISPLAY
	parent_clk = clk_get(NULL, "pll_dp");
#else
	parent_clk = tegra_get_clock_by_name("pll_dp");
#endif
	if (IS_ERR_OR_NULL(parent_clk)) {
		dev_err(&dc->ndev->dev, "dp: clock pll_dp unavailable\n");
		err = -EFAULT;
		goto err_iounmap_reg;
	}

	if (!tegra_platform_is_fpga()) {
		if (request_threaded_irq(irq, NULL, tegra_dp_irq,
					IRQF_ONESHOT, "tegra_dp", dp)) {
			dev_err(&dc->ndev->dev,
				"dp: request_irq %u failed\n", irq);
			err = -EBUSY;
			goto err_get_clk;
		}
	}

	if (dc->out->type != TEGRA_DC_OUT_FAKE_DP)
		tegra_dp_disable_irq(irq);

	dp->dc = dc;
	dp->aux_base = base;
	dp->res = res;
	dp->aux_base_res = base_res;
	dp->dpaux_clk = clk;
	dp->parent_clk = parent_clk;
	dp->mode = &dc->mode;
	dp->sor = tegra_dc_sor_init(dc, &dp->link_cfg);
	dp->irq = irq;
	dp->pdata = dc->pdata->default_out->dp_out;
	dp->suspended = false;

	if (IS_ERR_OR_NULL(dp->sor)) {
		err = PTR_ERR(dp->sor);
		dp->sor = NULL;
		goto err_get_clk;
	}

	init_completion(&dp->aux_tx);
	init_completion(&dp->hpd_plug);

	mutex_init(&dp->dpaux_lock);

	tegra_dc_set_outdata(dc, dp);

	/*
	 * We don't really need hpd driver for eDP.
	 * Nevertheless, go ahead and init hpd driver.
	 * eDP uses some of its fields to interact with panel.
	 */
	tegra_hpd_init(&dp->hpd_data, dc, dp, &hpd_ops);

	tegra_dp_lt_init(&dp->lt_data, dp);

	INIT_DELAYED_WORK(&dp->irq_evt_dwork, tegra_dp_irq_evt_worker);

	tegra_dc_dp_debug_create(dp);
	of_node_put(np_dp);

	return 0;

err_get_clk:
	clk_put(clk);
err_iounmap_reg:
	devm_iounmap(&dc->ndev->dev, base);
err_release_resource_reg:
	devm_release_mem_region(&dc->ndev->dev,
		res->start,
		resource_size(res));

	if (!np_dp || !of_device_is_available(np_dp))
		release_resource(res);
err_free_dp:
	devm_kfree(&dc->ndev->dev, dp);
	of_node_put(np_dp);

	return err;
}

static void tegra_dp_hpd_config(struct tegra_dc_dp_data *dp)
{
#define TEGRA_DP_HPD_UNPLUG_MIN_US	2000
#define TEGRA_DP_HPD_PLUG_MIN_US	250
#define TEGRA_DP_HPD_IRQ_MIN_US		250

	u32 val;

	val = TEGRA_DP_HPD_PLUG_MIN_US |
		(TEGRA_DP_HPD_UNPLUG_MIN_US <<
		DPAUX_HPD_CONFIG_UNPLUG_MIN_TIME_SHIFT);
	tegra_dpaux_writel(dp, DPAUX_HPD_CONFIG, val);

	tegra_dpaux_writel(dp, DPAUX_HPD_IRQ_CONFIG, TEGRA_DP_HPD_IRQ_MIN_US);

#undef TEGRA_DP_HPD_IRQ_MIN_US
#undef TEGRA_DP_HPD_PLUG_MIN_US
#undef TEGRA_DP_HPD_UNPLUG_MIN_US
}

static void tegra_dp_dpcd_init(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	u32 size_ieee_oui = 3, auxstat;
	u8 data_ieee_oui_be[3] = {(NV_IEEE_OUI >> 16) & 0xff,
		(NV_IEEE_OUI >> 8) & 0xff,
		NV_IEEE_OUI & 0xff};

	/* Check DP version */
	if (tegra_dc_dp_dpcd_read(dp, NV_DPCD_REV, &dp->revision))
		dev_err(&dp->dc->ndev->dev,
			"dp: failed to read the revision number from sink\n");

	if (tegra_dp_init_max_link_cfg(dp, cfg))
		dev_err(&dp->dc->ndev->dev,
			"dp: failed to init link configuration\n");

	tegra_dc_dpaux_write(dp, DPAUX_DP_AUXCTL_CMD_AUXWR,
		NV_DPCD_SOURCE_IEEE_OUI, data_ieee_oui_be, &size_ieee_oui,
		&auxstat);
}

void tegra_dp_tpg(struct tegra_dc_dp_data *dp, u32 tp, u32 n_lanes)
{
	if (tp == TRAINING_PATTERN_DISABLE)
		tegra_dc_dp_dpcd_write(dp, NV_DPCD_TRAINING_PATTERN_SET,
			(tp | NV_DPCD_TRAINING_PATTERN_SET_SC_DISABLED_F));
	else
		tegra_dc_dp_dpcd_write(dp, NV_DPCD_TRAINING_PATTERN_SET,
			(tp | NV_DPCD_TRAINING_PATTERN_SET_SC_DISABLED_T));

	tegra_sor_tpg(dp->sor, tp, n_lanes);
}

static void tegra_dp_tu_config(struct tegra_dc_dp_data *dp,
				const struct tegra_dc_dp_link_config *cfg)
{
	struct tegra_dc_sor_data *sor = dp->sor;
	u32 reg_val;

	tegra_sor_write_field(sor, NV_SOR_DP_LINKCTL(sor->portnum),
			NV_SOR_DP_LINKCTL_TUSIZE_MASK,
			(cfg->tu_size << NV_SOR_DP_LINKCTL_TUSIZE_SHIFT));

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_WATERMARK_MASK,
				cfg->watermark);

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_ACTIVESYM_COUNT_MASK,
				(cfg->active_count <<
				NV_SOR_DP_CONFIG_ACTIVESYM_COUNT_SHIFT));

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_ACTIVESYM_FRAC_MASK,
				(cfg->active_frac <<
				NV_SOR_DP_CONFIG_ACTIVESYM_FRAC_SHIFT));

	reg_val = cfg->activepolarity ?
		NV_SOR_DP_CONFIG_ACTIVESYM_POLARITY_POSITIVE :
		NV_SOR_DP_CONFIG_ACTIVESYM_POLARITY_NEGATIVE;
	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_ACTIVESYM_POLARITY_POSITIVE,
				reg_val);

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_ACTIVESYM_CNTL_ENABLE,
				NV_SOR_DP_CONFIG_ACTIVESYM_CNTL_ENABLE);

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_RD_RESET_VAL_NEGATIVE,
				NV_SOR_DP_CONFIG_RD_RESET_VAL_NEGATIVE);
}

void tegra_dp_update_link_config(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;

	tegra_dp_set_link_bandwidth(dp, cfg->link_bw);
	tegra_dp_set_lane_count(dp, cfg->lane_count);
	tegra_dp_link_cal(dp);
	tegra_dp_tu_config(dp, cfg);
}

static void tegra_dp_hpd_op_edid_ready(void *drv_data)
{
	struct tegra_dc_dp_data *dp = drv_data;
	struct tegra_dc *dc = dp->dc;

	/* in mm */
	dc->out->h_size = dc->out->h_size ? : dp->hpd_data.mon_spec.max_x * 10;
	dc->out->v_size = dc->out->v_size ? : dp->hpd_data.mon_spec.max_y * 10;

	/*
	 * EDID specifies either the acutal screen sizes or
	 * the aspect ratios. The panel file can choose to
	 * trust the value as the actual sizes by leaving
	 * width/height to 0s
	 */
	dc->out->width = dc->out->width ? : dc->out->h_size;
	dc->out->height = dc->out->height ? : dc->out->v_size;
}

static void tegra_dp_hpd_op_edid_recheck(void *drv_data)
{
	struct tegra_dc_dp_data *dp = drv_data;

	/*
	 * Hpd dropped but soon came back.
	 * Panel might have gone out of sync.
	 * Initiate LT if hpd driver found no
	 * change in edid and DP controller is enabled.
	 * Otherwise, next unblank would anyways
	 * initiate LT.
	 */
	if (tegra_hpd_get_state(&dp->hpd_data) ==
		STATE_DONE_ENABLED && dp->enabled)
		tegra_dp_lt_set_pending_evt(&dp->lt_data);
}

static inline void tegra_dp_reset(struct tegra_dc_dp_data *dp)
{
	if (tegra_platform_is_linsim())
		return;
	tegra_periph_reset_assert(dp->dpaux_clk);
	mdelay(2);
	tegra_periph_reset_deassert(dp->dpaux_clk);
	mdelay(1);
}

static inline void tegra_dp_default_int(struct tegra_dc_dp_data *dp,
					bool enable)
{
	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return;

	if (enable)
		tegra_dp_int_en(dp, DPAUX_INTR_EN_AUX_IRQ_EVENT |
				DPAUX_INTR_EN_AUX_PLUG_EVENT |
				DPAUX_INTR_EN_AUX_UNPLUG_EVENT);
	else
		tegra_dp_int_dis(dp, DPAUX_INTR_EN_AUX_IRQ_EVENT);
}

static int tegra_edp_edid_read(struct tegra_dc_dp_data *dp)
{
	struct tegra_hpd_data *data = &dp->hpd_data;

	BUG_ON(!data);

	memset(&data->mon_spec, 0, sizeof(data->mon_spec));

	return tegra_edid_get_monspecs(data->edid, &data->mon_spec);
}

static void tegra_edp_mode_set(struct tegra_dc_dp_data *dp)
{
	struct fb_videomode *best_edp_fbmode = dp->hpd_data.mon_spec.modedb;

	if (best_edp_fbmode)
		tegra_dc_set_fb_mode(dp->dc, best_edp_fbmode, false);
	else
		tegra_dc_set_default_videomode(dp->dc);
}

static int tegra_edp_wait_plug_hpd(struct tegra_dc_dp_data *dp)
{
#define TEGRA_DP_HPD_PLUG_TIMEOUT_MS	1000

	u32 val;
	int err = 0;

	might_sleep();

	if (!tegra_platform_is_silicon()) {
		msleep(TEGRA_DP_HPD_PLUG_TIMEOUT_MS);
		return 0;
	}

	val = tegra_dpaux_readl(dp, DPAUX_DP_AUXSTAT);
	if (likely(val & DPAUX_DP_AUXSTAT_HPD_STATUS_PLUGGED))
		err = 0;
	else if (!wait_for_completion_timeout(&dp->hpd_plug,
		msecs_to_jiffies(TEGRA_DP_HPD_PLUG_TIMEOUT_MS)))
		err = -ENODEV;

	return err;

#undef TEGRA_DP_HPD_PLUG_TIMEOUT_MS
}

static void tegra_dc_dp_enable(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	struct tegra_dc_sor_data *sor = dp->sor;
	int ret;

	if (dp->enabled)
		return;

	tegra_dc_io_start(dc);

	/* For eDP, driver gets to decide the best mode. */
	if (!tegra_dc_is_ext_dp_panel(dc) &&
		dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		int err;

		/*
		 * Hotplug for internal panels is not supported.
		 * Wait till the panel asserts hpd
		 */
		err = tegra_edp_wait_plug_hpd(dp);
		if (err < 0) {
			tegra_dc_io_end(dc);
			dc->connected = false;
			dev_err(&dc->ndev->dev,
				"edp: plug hpd wait timeout\n");
			return;
		}

		err = tegra_edp_edid_read(dp);
		if (err < 0)
			dev_warn(&dc->ndev->dev, "edp: edid read failed\n");
		else
			tegra_dp_hpd_op_edid_ready(dp);
		tegra_edp_mode_set(dp);
		tegra_dc_setup_clk(dc, dc->clk);
	}

	ret = tegra_dp_panel_power_state(dp, NV_DPCD_SET_POWER_VAL_D0_NORMAL);
	if (ret < 0) {
		dev_err(&dp->dc->ndev->dev,
			"dp: failed to power on panel (0x%x)\n", ret);
		tegra_dc_io_end(dc);
		return;
	}

	tegra_dp_dpcd_init(dp);

	tegra_dc_sor_enable_dp(dp->sor);

	tegra_dp_set_enhanced_framing(dp, cfg->enhanced_framing);

	if (cfg->alt_scramber_reset_cap)
		tegra_dc_dp_set_assr(dp, true);
	else
		tegra_dc_sor_set_internal_panel(dp->sor, false);

	tegra_dc_dp_dpcd_write(dp, NV_DPCD_MAIN_LINK_CHANNEL_CODING_SET,
			NV_DPCD_MAIN_LINK_CHANNEL_CODING_SET_ANSI_8B10B);

	tegra_sor_writel(sor, NV_SOR_LVDS, 0);

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_IDLE_BEFORE_ATTACH_ENABLE,
				NV_SOR_DP_CONFIG_IDLE_BEFORE_ATTACH_ENABLE);

	tegra_dp_set_link_bandwidth(dp, cfg->link_bw);
	tegra_dp_set_lane_count(dp, cfg->lane_count);
	tegra_dp_link_cal(dp);
	tegra_dp_tu_config(dp, cfg);

	tegra_dp_tpg(dp, TRAINING_PATTERN_DISABLE, cfg->lane_count);

	tegra_sor_port_enable(sor, true);
	tegra_sor_config_xbar(dp->sor);

	tegra_dp_clk_enable(dp);
	tegra_sor_config_dp_clk(dp->sor);
	tegra_dc_setup_clk(dc, dc->clk);

	dp->enabled = true;

	if (likely(dc->out->type != TEGRA_DC_OUT_FAKE_DP)) {
		tegra_dp_lt_set_pending_evt(&dp->lt_data);
		ret = tegra_dp_lt_wait_for_completion(&dp->lt_data,
							LT_TIMEOUT_MS);
		if (!ret)
			dev_err(&dp->dc->ndev->dev,
				"dp: link training failed\n");
	} else {
		/*
		 * Fake panel. Just enable host.
		 * No not engage with panel.
		 */
		tegra_sor_tpg(dp->sor, TRAINING_PATTERN_DISABLE,
				dp->link_cfg.lane_count);
		tegra_dc_sor_attach(dp->sor);
	}
	#if 0
	if (tegra_dc_is_ext_dp_panel(dc) &&
		dc->out->type != TEGRA_DC_OUT_FAKE_DP)
		tegra_dphdcp_set_plug(dp->dphdcp, true);
	#endif
	dc->connected = true;
	tegra_dc_io_end(dc);

	return;
}

void tegra_dc_dp_enable_link(struct tegra_dc_dp_data *dp)
{
	if (!dp->enabled)
		tegra_dc_dp_enable(dp->dc);
	else
		tegra_dc_sor_attach(dp->sor);
}

static void tegra_dc_dp_destroy(struct tegra_dc *dc)
{
	struct device_node *np_dp =
		(dc->ndev->id) ? of_find_node_by_path(DPAUX1_NODE) :
		of_find_node_by_path(DPAUX_NODE);
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	if (dp->sor)
		tegra_dc_sor_destroy(dp->sor);

	tegra_hpd_shutdown(&dp->hpd_data);

#ifndef CONFIG_TEGRA_NVDISPLAY
	clk_put(dp->dpaux_clk);
	clk_put(dp->parent_clk);
#endif

	devm_iounmap(&dc->ndev->dev, dp->aux_base);
	devm_release_mem_region(&dc->ndev->dev,
		dp->res->start,
		resource_size(dp->res));
	if (!np_dp || !of_device_is_available(np_dp))
		release_resource(dp->res);
	devm_kfree(&dc->ndev->dev, dp);
	of_node_put(np_dp);
}

static void tegra_dc_dp_disable(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);
	int ret;

	if (!dp->enabled)
		return;

	dp->enabled = false;

	tegra_dc_io_start(dc);
	#if 0
	if (tegra_dc_is_ext_dp_panel(dc) &&
		dc->out->type != TEGRA_DC_OUT_FAKE_DP)
		tegra_dphdcp_set_plug(dp->dphdcp, false);
	#endif
	cancel_delayed_work_sync(&dp->irq_evt_dwork);

	if (dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		tegra_dp_lt_force_disable(&dp->lt_data);
		ret = tegra_dp_lt_wait_for_completion(&dp->lt_data,
							LT_TIMEOUT_MS);
		WARN_ON(!ret);
	}

	tegra_dc_sor_detach(dp->sor);
	tegra_dc_sor_disable(dp->sor, false);

	tegra_dp_clk_disable(dp);

	tegra_dc_io_end(dc);
}

void tegra_dc_dp_pre_disable_link(struct tegra_dc_dp_data *dp)
{
	tegra_dc_sor_pre_detach(dp->sor);
}

void tegra_dc_dp_disable_link(struct tegra_dc_dp_data *dp, bool powerdown)
{
	tegra_dc_sor_detach(dp->sor);

	if (powerdown)
		tegra_dc_dp_disable(dp->dc);
}

static long tegra_dc_dp_setup_clk(struct tegra_dc *dc, struct clk *clk)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);
	struct clk *dc_parent_clk;

	if (!tegra_platform_is_silicon())
		return tegra_dc_pclk_round_rate(dc, dc->mode.pclk);

	if (clk == dc->clk) {
		dc_parent_clk = clk_get_sys(NULL,
				dc->out->parent_clk ? : "pll_d_out0");
		clk_set_parent(dc->clk, dc_parent_clk);
	}

	tegra_sor_setup_clk(dp->sor, clk, false);

	/* fixed pll_dp@270MHz */
	clk_set_rate(dp->parent_clk, 270000000);

	return tegra_dc_pclk_round_rate(dc, dc->mode.pclk);
}

static bool tegra_dc_dp_hpd_state(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);
	u32 val;

	if (WARN_ON(!dc || !dc->out))
		return false;

	if (dc->out->type == TEGRA_DC_OUT_FAKE_DP ||
		tegra_platform_is_linsim())
		return true;

	tegra_dpaux_clk_enable(dp);
	tegra_dc_io_start(dc);
	val = tegra_dpaux_readl(dp, DPAUX_DP_AUXSTAT);
	tegra_dc_io_end(dc);

	return !!(val & DPAUX_DP_AUXSTAT_HPD_STATUS_PLUGGED);
}

/* used by tegra_dc_probe() to detect connection(HPD) status at boot */
static bool tegra_dc_dp_detect(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	if (tegra_platform_is_linsim())
		return true;

	tegra_dp_pending_hpd(dp);

	return tegra_dc_hpd(dc);
}

static void tegra_dc_dp_suspend(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	if (dp->suspended)
		return;

	dp->suspended = true;

	/* do not process hpd in suspend. Disable dpaux clocks. */
	if (dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP)
		tegra_dp_disable_irq(dp->irq);
	if (dp->sor->safe_clk)
		tegra_sor_safe_clk_disable(dp->sor);
	tegra_dpaux_clk_disable(dp);

	tegra_dc_powergate_locked(dc);
}

static void tegra_dc_dp_resume(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	if (!dp->suspended)
		return;

	tegra_dc_unpowergate_locked(dc);

	/* Get ready to receive any hpd event */
	_tegra_dpaux_init(dp);
	if (dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP)
		tegra_dp_enable_irq(dp->irq);

	tegra_dp_pending_hpd(dp);

	dp->suspended = false;
}

static void tegra_dc_dp_modeset_notifier(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	tegra_dc_io_start(dc);
	tegra_dpaux_clk_enable(dp);

	tegra_dc_sor_modeset_notifier(dp->sor, false);
	/* Pixel clock may be changed in new mode,
	 * recalculate link config */
	if (!tegra_platform_is_linsim())
		tegra_dc_dp_calc_config(dp, dp->mode, &dp->link_cfg);


	tegra_dpaux_clk_disable(dp);
	tegra_dc_io_end(dc);
}

static bool tegra_dp_check_dc_constraint(const struct fb_videomode *mode)
{
	return (mode->hsync_len >= 1) && (mode->vsync_len >= 1) &&
		(mode->lower_margin + mode->vsync_len +
		mode->upper_margin > 1) &&
		(mode->xres >= 16) && (mode->yres >= 16);
}

static bool tegra_dp_mode_filter(const struct tegra_dc *dc,
				struct fb_videomode *mode)
{
	if (!mode->pixclock)
		return false;

	if (mode->xres > 4096)
		return false;

	if (mode->pixclock && tegra_dc_get_out_max_pixclock(dc) &&
		mode->pixclock > tegra_dc_get_out_max_pixclock(dc))
		return false;

	/*
	 * Work around for modes that fail the constraint:
	 * V_FRONT_PORCH >= V_REF_TO_SYNC + 1
	 */
	if (mode->lower_margin == 1) {
		mode->lower_margin++;
		mode->upper_margin--;
	}

	if (!tegra_dp_check_dc_constraint(mode))
		return false;

	return true;
}

static bool (*tegra_dp_op_get_mode_filter(void *drv_data))
	(const struct tegra_dc *dc, struct fb_videomode *mode) {
	return tegra_dp_mode_filter;
}

static bool tegra_dp_hpd_op_get_hpd_state(void *drv_data)
{
	struct tegra_dc_dp_data *dp = drv_data;

	return tegra_dc_hpd(dp->dc);
}

static void tegra_dp_hpd_op_init(void *drv_data)
{
	struct tegra_dc_dp_data *dp = drv_data;

#ifdef CONFIG_SWITCH
	if (tegra_dc_is_ext_dp_panel(dp->dc)) {
		dp->hpd_data.hpd_switch_name = "dp";
		dp->hpd_data.audio_switch_name = "dp_audio";
	}
#endif
}

static struct tegra_hpd_ops hpd_ops = {
	.edid_read = tegra_dp_hpd_op_edid_read,
	.edid_ready = tegra_dp_hpd_op_edid_ready,
	.edid_recheck = tegra_dp_hpd_op_edid_recheck,
	.get_mode_filter = tegra_dp_op_get_mode_filter,
	.get_hpd_state = tegra_dp_hpd_op_get_hpd_state,
	.init = tegra_dp_hpd_op_init,
};

struct tegra_dc_out_ops tegra_dc_dp_ops = {
	.init	   = tegra_dc_dp_init,
	.destroy   = tegra_dc_dp_destroy,
	.enable	   = tegra_dc_dp_enable,
	.disable   = tegra_dc_dp_disable,
	.detect    = tegra_dc_dp_detect,
	.setup_clk = tegra_dc_dp_setup_clk,
	.modeset_notifier = tegra_dc_dp_modeset_notifier,
	.hpd_state = tegra_dc_dp_hpd_state,
	.suspend = tegra_dc_dp_suspend,
	.resume = tegra_dc_dp_resume,
	.hotplug_init = tegra_dc_dp_hotplug_init,
};
