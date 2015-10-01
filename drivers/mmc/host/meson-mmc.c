/*
 * mesonsd.c - Meson SDH Controller
 *
 * Copyright (C) 2015 Endless Mobile, Inc.
 * Author: Carlo Caione <carlo@endlessm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#define DRIVER_NAME	"meson-mmc"

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>

#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>

#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

#define SDIO_ARGU		(0x00)
#define SDIO_SEND		(0x04)
#define SDIO_CONF		(0x08)
#define SDIO_IRQS		(0x0c)
#define SDIO_IRQC		(0x10)
#define SDIO_MULT		(0x14)
#define SDIO_ADDR		(0x18)
#define SDIO_EXT		(0x1c)

#define REG_IRQS_RESP_CRC7	BIT(5)
#define REG_IRQS_RD_CRC16	BIT(6)
#define REG_IRQS_WR_CRC16	BIT(7)
#define REG_IRQS_CMD_INT	BIT(9)

#define REG_IRQC_ARC_CMD_INT	BIT(4)
#define REG_IRQC_SOFT_RESET	BIT(15)

#define REG_CONF_WR_CRC_S	(29)
#define REG_CONF_WR_NWR_S	(23)
#define REG_CONF_M_END_S	(21)
#define REG_CONF_ARGU_BITS_S	(12)
#define REG_CONF_CLK_DIV_M	(0x3ff)
#define REG_CONF_BUS_WIDTH	BIT(20)

#define REG_MULT_PORT_SEL_M	(0x3)
#define REG_MULT_RD_INDEX_M	(0x0f)
#define REG_MULT_RD_INDEX_S	(12)
#define REG_MULT_WR_RD_OUT_IND	BIT(8)

#define REG_SEND_CMD_COMMAND_M	(0xff)
#define REG_SEND_CMD_RESP_S	(8)
#define REG_SEND_RESP_NO_CRC7	BIT(16)
#define REG_SEND_RESP_HAVE_DATA	BIT(17)
#define REG_SEND_RESP_CRC7_F_8	BIT(18)
#define REG_SEND_CHECK_BUSY_D0	BIT(19)
#define REG_SEND_CMD_SEND_DATA	BIT(20)
#define REG_SEND_REP_PACK_N_M	(0xff)
#define REG_SEND_REP_PACK_N_S	(24)

#define REG_EXT_DAT_RW_NUM_M	(0x3fff)
#define REG_EXT_DAT_RW_NUM_S	(16)

#define CLK_DIV			(0x1f4)

#define SDIO_BOUNCE_REQ_SIZE	(128 * 1024)

enum mmcif_state {
	STATE_IDLE,
	STATE_REQUEST,
	STATE_IOS,
	STATE_TIMEOUT,
	STATE_STOP,
};

struct meson_mmc_host {
	struct mmc_host		*mmc;
	struct mmc_request	*mrq;
	struct delayed_work     timeout_work;
	struct clk		*clk_sdio;
	spinlock_t		lock;
	void __iomem		*base;
	long			timeout;
	int			irq;
	int			ferror;
	unsigned int		bus_width;
	unsigned int		port;
	enum mmcif_state	state;
};

static int meson_mmc_clk_set_rate(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct meson_mmc_host *host = mmc_priv(mmc);
	unsigned long clk_rate;
	unsigned int clk_ios = ios->clock;
	unsigned int clk_div;
	u32 conf_reg;

	clk_rate = clk_get_rate(host->clk_sdio) / 2;
	if (clk_rate < 0) {
		dev_err(mmc_dev(mmc), "cannot get clock rate\n");
		return -EINVAL;
	}

	clk_div = clk_rate / clk_ios - !(clk_rate % clk_ios);

	conf_reg = readl(host->base + SDIO_CONF);
	conf_reg &= ~REG_CONF_CLK_DIV_M;
	conf_reg |= clk_div;
	writel(conf_reg, host->base + SDIO_CONF);

	dev_dbg(mmc_dev(mmc), "clk_ios: %d, clk_div: %d, clk_rate: %ld\n",
			clk_ios, clk_div, clk_rate);

	return 0;
}

static void meson_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct meson_mmc_host *host = mmc_priv(mmc);
	u32 reg;

	if (host->state != STATE_IDLE) {
		dev_dbg(mmc_dev(mmc), "%s() rejected, state %u\n",
			__func__, host->state);
		return;
	}

	host->state = STATE_IOS;

	reg = readl(host->base + SDIO_CONF);

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		host->bus_width = 0;
		reg &= ~REG_CONF_BUS_WIDTH;
		dev_dbg(mmc_dev(mmc), "bus width: 1bit\n");
		break;
	case MMC_BUS_WIDTH_4:
		host->bus_width = 1;
		reg |= REG_CONF_BUS_WIDTH;
		dev_dbg(mmc_dev(mmc), "bus width: 4bit\n");
		break;
	case MMC_BUS_WIDTH_8:
	default:
		dev_err(mmc_dev(mmc), "SDIO controller doesn't support 8bit data bus\n");
		host->ferror = -EINVAL;
		return;
	}

	writel(reg, host->base + SDIO_CONF);

	if (ios->clock && ios->power_mode)
		host->ferror = meson_mmc_clk_set_rate(mmc, ios);

	host->state = STATE_IDLE;
}

static void meson_mmc_soft_reset(struct meson_mmc_host *host)
{
	u32 irqc;

	irqc = readl(host->base + SDIO_IRQC);
	irqc |= REG_IRQC_SOFT_RESET;
	writel(irqc, host->base + SDIO_IRQC);
	udelay(2);
}

static void meson_mmc_start_cmd(struct mmc_host *mmc,
				struct mmc_request *mrq)
{
	struct meson_mmc_host *host = mmc_priv(mmc);
	unsigned int pack_size;
	u32 irqc, irqs, mult;
	u32 send = 0;
	u32 ext = 0;

	switch (mmc_resp_type(mrq->cmd)) {
	case MMC_RSP_R1:
	case MMC_RSP_R1B:
	case MMC_RSP_R3:
		send |= (45 << REG_SEND_CMD_RESP_S);
		break;
	case MMC_RSP_R2:
		send |= (133 << REG_SEND_CMD_RESP_S);
		send |= REG_SEND_RESP_CRC7_F_8;
		break;
	default:
		break;
	}

	if (!(mrq->cmd->flags & MMC_RSP_CRC))
		send |= REG_SEND_RESP_NO_CRC7;

	if (mrq->cmd->flags & MMC_RSP_BUSY)
		send |= REG_SEND_CHECK_BUSY_D0;

	if (mrq->data) {
		send &= ~(REG_SEND_REP_PACK_N_M << REG_SEND_REP_PACK_N_S);
		send |= ((mrq->data->blocks - 1) << REG_SEND_REP_PACK_N_S);

		ext &= ~(REG_EXT_DAT_RW_NUM_M << REG_EXT_DAT_RW_NUM_S);
		if (host->bus_width)
			pack_size = mrq->data->blksz * 8 + (16 - 1) * 4;
		else
			pack_size = mrq->data->blksz * 8 + (16 - 1);
		ext |= (pack_size << REG_EXT_DAT_RW_NUM_S);

		if (mrq->data->flags & MMC_DATA_WRITE)
			send |= REG_SEND_CMD_SEND_DATA;
		else
			send |= REG_SEND_RESP_HAVE_DATA;
	}

	send &= ~REG_SEND_CMD_COMMAND_M;
	send |= (0x40 | mrq->cmd->opcode);

	meson_mmc_soft_reset(host);

	irqc = readl(host->base + SDIO_IRQC);
	irqc |= REG_IRQC_ARC_CMD_INT;

	irqs = readl(host->base + SDIO_IRQS);
	irqs |= REG_IRQS_CMD_INT;

	mult = readl(host->base + SDIO_MULT);
	mult &= ~REG_MULT_PORT_SEL_M;
	mult |= host->port;
	mult |= (1 << 31);
	writel(mult, host->base + SDIO_MULT);
	writel(irqs, host->base + SDIO_IRQS);
	writel(irqc, host->base + SDIO_IRQC);

	writel(mrq->cmd->arg, host->base + SDIO_ARGU);
	writel(ext, host->base + SDIO_EXT);
	writel(send, host->base + SDIO_SEND);
}

static int meson_mmc_map_dma(struct meson_mmc_host *host,
			     struct mmc_data *data,
			     unsigned int flags)
{	u32 dma_len;
	struct scatterlist *sg = data->sg;

	if (sg->offset & 3 || sg->length & 3) {
		dev_err(mmc_dev(host->mmc),
			"unaligned scatterlist: os %x length %d\n",
			sg->offset, sg->length);
		return -EINVAL;
	}

	dma_len = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
			     ((data->flags & MMC_DATA_READ) ?
			     DMA_FROM_DEVICE : DMA_TO_DEVICE));
	if (dma_len == 0) {
		dev_err(mmc_dev(host->mmc), "dma_map_sg failed\n");
		return -ENOMEM;
	}

	return 0;
}

static void meson_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct meson_mmc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&host->lock, flags);

	if (host->state != STATE_IDLE) {
		dev_dbg(mmc_dev(mmc), "%s() rejected, state %u\n",
			__func__, host->state);
		mrq->cmd->error = -EAGAIN;
		spin_unlock_irqrestore(&host->lock, flags);
		mmc_request_done(mmc, mrq);
		return;
	}

	if (host->ferror) {
		cmd->error = host->ferror;
		spin_unlock_irqrestore(&host->lock, flags);
		mmc_request_done(mmc, mrq);
		return;
	}

	dev_dbg(mmc_dev(mmc), "CMD%d(%08x) arg %x len %d flags %08x\n",
		cmd->opcode & 0x3f, cmd->opcode, cmd->arg,
		mrq->data ? mrq->data->blksz * mrq->data->blocks : 0,
		mrq->cmd->flags);

	/* Filter out CMD 5/52/53 */
	if (cmd->opcode == SD_IO_SEND_OP_COND ||
	    cmd->opcode == SD_IO_RW_DIRECT ||
	    cmd->opcode == SD_IO_RW_EXTENDED) {
		dev_dbg(mmc_dev(host->mmc), "CMD%d not supported\n",
			cmd->opcode);
		cmd->error = -EINVAL;
		spin_unlock_irqrestore(&host->lock, flags);
		mmc_request_done(mmc, mrq);
		return;
	}

	host->state = STATE_REQUEST;

	if (data) {
		ret = meson_mmc_map_dma(host, data, data->flags);
		if (ret < 0) {
			dev_err(mmc_dev(mmc), "map DMA failed\n");
			cmd->error = ret;
			data->error = ret;
			host->state = STATE_IDLE;
			spin_unlock_irqrestore(&host->lock, flags);
			mmc_request_done(mmc, mrq);
			return;
		}
		writel(sg_dma_address(data->sg), host->base + SDIO_ADDR);
	}

	host->mrq = mrq;
	schedule_delayed_work(&host->timeout_work, host->timeout);
	meson_mmc_start_cmd(mmc, mrq);

	spin_unlock_irqrestore(&host->lock, flags);
}

static irqreturn_t meson_mmc_irq(int irq, void *data)
{
	struct meson_mmc_host *host = (void *) data;
	struct mmc_request *mrq = host->mrq;
	u32 irqs;

	irqs = readl(host->base + SDIO_IRQS);
	if (mrq && (irqs & REG_IRQS_CMD_INT))
		return IRQ_WAKE_THREAD;

	return IRQ_HANDLED;
}

void meson_mmc_read_response(struct meson_mmc_host *host)
{
	struct mmc_command *cmd = host->mrq->cmd;
	u32 mult;
	int i, resp[4] = { 0 };

	mult = readl(host->base + SDIO_MULT);
	mult |= REG_MULT_WR_RD_OUT_IND;
	mult &= ~(REG_MULT_RD_INDEX_M << REG_MULT_RD_INDEX_S);
	writel(mult, host->base + SDIO_MULT);

	if (cmd->flags & MMC_RSP_136) {
		for (i = 0; i <= 3; i++)
			resp[3 - i] = readl(host->base + SDIO_ARGU);
		cmd->resp[0] = (resp[0] << 8) | ((resp[1] >> 24) & 0xff);
		cmd->resp[1] = (resp[1] << 8) | ((resp[2] >> 24) & 0xff);
		cmd->resp[2] = (resp[2] << 8) | ((resp[3] >> 24) & 0xff);
		cmd->resp[3] = (resp[3] << 8);
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		cmd->resp[0] = readl(host->base + SDIO_ARGU);
	}
}

struct mmc_command meson_mmc_cmd = {
	.opcode	= MMC_STOP_TRANSMISSION,
	.flags	= MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC,
};

struct mmc_request meson_mmc_stop = {
	.cmd = &meson_mmc_cmd,
};

static irqreturn_t meson_mmc_irq_thread(int irq, void *irq_data)
{
	struct meson_mmc_host *host = (void *) irq_data;
	struct mmc_data *data;
	unsigned long flags;
	struct mmc_request *mrq;
	u32 irqs, send;

	spin_lock_irqsave(&host->lock, flags);

	cancel_delayed_work_sync(&host->timeout_work);
	mrq = host->mrq;
	data = mrq->data;

	if (!mrq) {
		spin_unlock_irqrestore(&host->lock, flags);
		return IRQ_HANDLED;
	}

	if (host->state == STATE_STOP)
		goto out;

	if (host->state != STATE_REQUEST) {
		dev_dbg(mmc_dev(host->mmc), "%s() rejected, state %u\n",
			__func__, host->state);
		mrq->cmd->error = -EAGAIN;
		goto out;
	}

	irqs = readl(host->base + SDIO_IRQS);
	send = readl(host->base + SDIO_SEND);

	mrq->cmd->error = 0;

	if (!data) {
		if (!((irqs & REG_IRQS_RESP_CRC7) ||
		      (send & REG_SEND_RESP_NO_CRC7)))
			mrq->cmd->error = -EILSEQ;
		else
			meson_mmc_read_response(host);
	} else {
		if (!((irqs & REG_IRQS_RD_CRC16) ||
		      (irqs & REG_IRQS_WR_CRC16))) {
			mrq->cmd->error = -EILSEQ;
		} else {
			data->bytes_xfered = data->blksz * data->blocks;
			dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
				     ((data->flags & MMC_DATA_READ) ?
				     DMA_FROM_DEVICE : DMA_TO_DEVICE));

			if (mrq->stop) {
				host->state = STATE_STOP;
				meson_mmc_start_cmd(host->mmc, &meson_mmc_stop);
				spin_unlock_irqrestore(&host->lock, flags);
				return IRQ_HANDLED;
			}
		}
	}

out:
	host->state = STATE_IDLE;
	host->mrq = NULL;
	spin_unlock_irqrestore(&host->lock, flags);
	mmc_request_done(host->mmc, mrq);

	return IRQ_HANDLED;
}

static void meson_mmc_timeout(struct work_struct *work)
{
	struct meson_mmc_host *host = container_of(work,
						   struct meson_mmc_host,
						   timeout_work.work);
	struct mmc_request *mrq = host->mrq;
	unsigned long flags;
	u32 irqc;

	spin_lock_irqsave(&host->lock, flags);

	if (host->state == STATE_IDLE) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	dev_err(mmc_dev(host->mmc), "Timeout on CMD%u\n", mrq->cmd->opcode);

	host->state = STATE_TIMEOUT;

	irqc = readl(host->base + SDIO_IRQC);
	irqc &= ~REG_IRQC_ARC_CMD_INT;
	writel(irqc, host->base + SDIO_IRQC);

	mrq->cmd->error = -ETIMEDOUT;

	host->state = STATE_IDLE;
	host->mrq = NULL;
	spin_unlock_irqrestore(&host->lock, flags);

	mmc_request_done(host->mmc, mrq);
}

static void meson_mmc_reset(struct mmc_host *mmc)
{
	struct meson_mmc_host *host = mmc_priv(mmc);
	u32 reg = 0;

	dev_dbg(mmc_dev(mmc), "resetting mmc controller\n");

	reg =  (2 << REG_CONF_WR_CRC_S);
	reg |= (2 << REG_CONF_WR_NWR_S);
	reg |= (3 << REG_CONF_M_END_S);
	reg |= (39 << REG_CONF_ARGU_BITS_S);

	reg |= CLK_DIV;
	writel(reg, host->base + SDIO_CONF);

	reg = readl(host->base + SDIO_IRQS);
	reg |= (REG_IRQS_CMD_INT);
	writel(reg, host->base + SDIO_IRQS);
}

static struct mmc_host_ops meson_mmc_ops = {
	.request	 = meson_mmc_request,
	.set_ios	 = meson_mmc_set_ios,
	.get_cd		 = mmc_gpio_get_cd,
};

static int meson_mmc_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct meson_mmc_host *host;
	struct device_node *node = pdev->dev.of_node;
	int ret, sdio_port;

	mmc = mmc_alloc_host(sizeof(struct meson_mmc_host), &pdev->dev);
	if (!mmc) {
		dev_err(&pdev->dev, "mmc alloc host failed\n");
		return -ENOMEM;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;
	spin_lock_init(&host->lock);

	host->base = devm_ioremap_resource(&pdev->dev,
				platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (IS_ERR(host->base)) {
		ret = PTR_ERR(host->base);
		goto error_free_host;
	}

	host->irq = platform_get_irq(pdev, 0);
	ret = devm_request_threaded_irq(&pdev->dev, host->irq, meson_mmc_irq,
			meson_mmc_irq_thread, 0, "meson_mmc", host);
	if (ret)
		goto error_free_host;

	host->clk_sdio = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(host->clk_sdio)) {
		dev_err(&pdev->dev, "Could not get clk81 clock\n");
		ret = PTR_ERR(host->clk_sdio);
		goto error_free_host;
	}

	mmc->ops = &meson_mmc_ops;

	/* we do not support scatter lists in hardware */
	mmc->max_segs = 1;
	mmc->max_req_size = SDIO_BOUNCE_REQ_SIZE;
	mmc->max_seg_size = mmc->max_req_size;
	mmc->max_blk_count = 256;
	mmc->max_blk_size = mmc->max_req_size / mmc->max_blk_count;
	mmc->f_min = 300000;
	mmc->f_max = 50000000;
	mmc->caps |= MMC_CAP_4_BIT_DATA;
	mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED;
	mmc->ocr_avail = MMC_VDD_33_34;

	INIT_DELAYED_WORK(&host->timeout_work, meson_mmc_timeout);
	host->timeout = msecs_to_jiffies(2000);
	host->port = 0;

	if (!of_property_read_u32(node, "meson,sdio-port", &sdio_port))
		host->port = sdio_port;

	ret = mmc_of_parse(mmc);
	if (ret)
		goto error_free_host;

	platform_set_drvdata(pdev, mmc);

	meson_mmc_reset(mmc);
	mmc_add_host(mmc);

	dev_info(&pdev->dev, "base:0x%p irq:%u port:%u\n",
		 host->base, host->irq, host->port);

	return 0;

error_free_host:
	mmc_free_host(mmc);

	return ret;
}

static int meson_mmc_remove(struct platform_device *pdev)
{
	struct mmc_host	*mmc = platform_get_drvdata(pdev);
	struct meson_mmc_host *host = mmc_priv(mmc);

	mmc_remove_host(mmc);
	disable_irq(host->irq);

	mmc_free_host(mmc);

	cancel_delayed_work_sync(&host->timeout_work);

	return 0;
}

static const struct of_device_id meson_mmc_of_match[] = {
	{ .compatible = "amlogic,meson-mmc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_mmc_of_match);

static struct platform_driver meson_mmc_driver = {
	.probe   = meson_mmc_probe,
	.remove  = meson_mmc_remove,
	.driver  = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(meson_mmc_of_match),
	},
};

module_platform_driver(meson_mmc_driver);

MODULE_DESCRIPTION("Meson Secure Digital Host Driver");
MODULE_AUTHOR("Carlo Caione <carlo@endlessm.com>");
MODULE_LICENSE("GPL");
