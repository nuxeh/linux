/*
 * drivers/video/tegra/host/nvhost_job.c
 *
 * Tegra Graphics Host Job
 *
 * Copyright (c) 2010-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/err.h>
#include <linux/vmalloc.h>
#include <linux/sort.h>
#include <linux/scatterlist.h>
#include <trace/events/nvhost.h>
#include "nvhost_channel.h"
#include "nvhost_job.h"
#include "nvhost_syncpt.h"
#include "dev.h"
#include "chip_support.h"
#include "nvhost_vm.h"

/* Magic to use to fill freed handle slots */
#define BAD_MAGIC 0xdeadbeef

static size_t job_size(u32 num_cmdbufs, u32 num_relocs, u32 num_waitchks,
			u32 num_syncpts)
{
	s64 num_unpins = num_cmdbufs + num_relocs;
	s64 total;

	total = sizeof(struct nvhost_job)
			+ num_relocs * sizeof(struct nvhost_reloc)
			+ num_relocs * sizeof(struct nvhost_reloc_shift)
			+ num_waitchks * sizeof(struct nvhost_waitchk)
			+ num_cmdbufs * sizeof(struct nvhost_job_gather)
			+ num_unpins * sizeof(dma_addr_t)
			+ num_unpins * sizeof(struct nvhost_pinid)
			+ num_syncpts * sizeof(struct nvhost_job_syncpt);

	if(total > ULONG_MAX)
		return 0;
	return (size_t)total;
}


static void init_fields(struct nvhost_job *job,
		u32 num_cmdbufs, u32 num_relocs, u32 num_waitchks,
		u32 num_syncpts)
{
	int num_unpins = num_cmdbufs + num_relocs;
	void *mem = job;

	/* First init state to zero */

	/*
	 * Redistribute memory to the structs.
	 * Overflows and negative conditions have
	 * already been checked in job_alloc().
	 */
	mem += sizeof(struct nvhost_job);
	job->relocarray = num_relocs ? mem : NULL;
	mem += num_relocs * sizeof(struct nvhost_reloc);
	job->relocshiftarray = num_relocs ? mem : NULL;
	mem += num_relocs * sizeof(struct nvhost_reloc_shift);
	job->waitchk = num_waitchks ? mem : NULL;
	mem += num_waitchks * sizeof(struct nvhost_waitchk);
	job->gathers = num_cmdbufs ? mem : NULL;
	mem += num_cmdbufs * sizeof(struct nvhost_job_gather);
	job->addr_phys = num_unpins ? mem : NULL;
	mem += num_unpins * sizeof(dma_addr_t);
	job->pin_ids = num_unpins ? mem : NULL;
	mem += num_unpins * sizeof(struct nvhost_pinid);
	job->sp = num_syncpts ? mem : NULL;

	job->reloc_addr_phys = job->addr_phys;
	job->gather_addr_phys = &job->addr_phys[num_relocs];
}

struct nvhost_job *nvhost_job_alloc(struct nvhost_channel *ch,
		int num_cmdbufs, int num_relocs, int num_waitchks,
		int num_syncpts)
{
	struct nvhost_job *job = NULL;
	size_t size =
		job_size(num_cmdbufs, num_relocs, num_waitchks, num_syncpts);

	if(!size)
		return NULL;
	if(size <= PAGE_SIZE)
		job = kzalloc(size, GFP_KERNEL);
	else
		job = vzalloc(size);
	if (!job)
		return NULL;

	kref_init(&job->ref);
	job->ch = ch;
	job->size = size;

	init_fields(job, num_cmdbufs, num_relocs, num_waitchks, num_syncpts);

	return job;
}
EXPORT_SYMBOL(nvhost_job_alloc);

void nvhost_job_get(struct nvhost_job *job)
{
	kref_get(&job->ref);
}

static void job_free(struct kref *ref)
{
	struct nvhost_job *job = container_of(ref, struct nvhost_job, ref);

	if (job->vm) {
		nvhost_vm_put(job->vm);
		job->vm = NULL;
	}

	if (job->error_notifier_ref)
		dma_buf_put(job->error_notifier_ref);
	if (job->size <= PAGE_SIZE)
		kfree(job);
	else
		vfree(job);
}

void nvhost_job_put(struct nvhost_job *job)
{
	kref_put(&job->ref, job_free);
}
EXPORT_SYMBOL(nvhost_job_put);

int nvhost_job_add_client_gather_address(struct nvhost_job *job,
		u32 num_words, u32 class_id, dma_addr_t gather_address)
{
	nvhost_job_add_gather(job, 0, num_words, 0, class_id, 0);

	job->gathers[0].mem_base = gather_address;

	return 0;
}
EXPORT_SYMBOL(nvhost_job_add_client_gather_address);

void nvhost_job_add_gather(struct nvhost_job *job,
		u32 mem_id, u32 words, u32 offset, u32 class_id, int pre_fence)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(job->ch->dev);
	struct nvhost_job_gather *cur_gather =
			&job->gathers[job->num_gathers];

	cur_gather->words = words;
	cur_gather->mem_id = mem_id;
	cur_gather->offset = offset;
	cur_gather->class_id = class_id ? class_id : pdata->class;
	cur_gather->pre_fence = pre_fence;
	job->num_gathers += 1;
}

void nvhost_job_set_notifier(struct nvhost_job *job, u32 error)
{
	struct nvhost_notification *error_notifier;
	struct timespec time_data;
	void *va;
	u64 nsec;

	if (!job->error_notifier_ref)
		return;

	/* map handle and clear error notifier struct */
	va = dma_buf_vmap(job->error_notifier_ref);
	if (!va) {
		dma_buf_put(job->error_notifier_ref);
		dev_err(&job->ch->dev->dev, "Cannot map notifier handle\n");
		return;
	}

	error_notifier = va + job->error_notifier_offset;

	getnstimeofday(&time_data);
	nsec = ((u64)time_data.tv_sec) * 1000000000u +
		(u64)time_data.tv_nsec;
	error_notifier->time_stamp.nanoseconds[0] =
		(u32)nsec;
	error_notifier->time_stamp.nanoseconds[1] =
		(u32)(nsec >> 32);
	error_notifier->info32 = error;
	error_notifier->status = 0xffff;
	dev_err(&job->ch->dev->dev, "error notifier set to %d\n", error);

	dma_buf_vunmap(job->error_notifier_ref, va);
}

/*
 * Check driver supplied waitchk structs for syncpt thresholds
 * that have already been satisfied and NULL the comparison (to
 * avoid a wrap condition in the HW).
 */
static int do_waitchks(struct nvhost_job *job, struct nvhost_syncpt *sp,
		u32 patch_mem, struct dma_buf *buf)
{
	int i;

	/* compare syncpt vs wait threshold */
	for (i = 0; i < job->num_waitchk; i++) {
		struct nvhost_waitchk *wait = &job->waitchk[i];

		/* validate syncpt id */
		if (!nvhost_syncpt_is_valid_hw_pt(sp, wait->syncpt_id))
			continue;

		if (!wait->mem)
			continue;

		/* skip all other gathers */
		if (patch_mem != wait->mem)
			continue;

		trace_nvhost_syncpt_wait_check(wait->mem, wait->offset,
				wait->syncpt_id, wait->thresh,
				nvhost_syncpt_read(sp, wait->syncpt_id));
		if (nvhost_syncpt_is_expired(sp,
		    wait->syncpt_id, wait->thresh) ||
		    nvhost_get_channel_policy() == MAP_CHANNEL_ON_SUBMIT) {
			void *patch_addr = NULL;

			/*
			 * NULL an already satisfied WAIT_SYNCPT host method,
			 * by patching its args in the command stream. The
			 * method data is changed to reference a reserved
			 * (never given out or incr) graphics host syncpt
			 * with a matching threshold value of 0, so is
			 * guaranteed to be popped by the host HW.
			 */
			dev_dbg(&syncpt_to_dev(sp)->dev->dev,
			    "drop WAIT id %d (%s) thresh 0x%x, min 0x%x\n",
			    wait->syncpt_id,
			    syncpt_op().name(sp, wait->syncpt_id),
			    wait->thresh,
			    nvhost_syncpt_read_min(sp, wait->syncpt_id));

			/* patch the wait */
			patch_addr = dma_buf_kmap(buf,
					wait->offset >> PAGE_SHIFT);
			if (patch_addr) {
				nvhost_syncpt_patch_wait(sp,
					(patch_addr +
					 (wait->offset & ~PAGE_MASK)));
				dma_buf_kunmap(buf,
						wait->offset >> PAGE_SHIFT,
						patch_addr);
			} else {
				pr_err("Couldn't map cmdbuf for wait check\n");
			}
		}

		wait->mem = 0;
	}
	return 0;
}

static int pin_job_mem(struct nvhost_job *job)
{
	int pin_count = 0;
	struct dma_buf *dmabufs[job->num_relocs + job->num_gathers];
	int err = 0;
	int i;

	for (i = 0; i < job->num_relocs; i++) {
		struct nvhost_reloc *reloc = &job->relocarray[i];
		struct dma_buf *dmabuf = dma_buf_get(reloc->target);
		if (IS_ERR(dmabuf)) {
			err = PTR_ERR(dmabuf);
			goto err_map;
		}

		err = nvhost_vm_map_dmabuf(job->vm, dmabuf,
					   &job->addr_phys[pin_count]);
		dma_buf_put(dmabuf);
		if (err)
			goto err_map;

		dmabufs[pin_count] = dmabuf;

		pin_count++;
	}

	for (i = 0; i < job->num_gathers; i++) {
		struct nvhost_job_gather *g = &job->gathers[i];
		struct dma_buf *dmabuf = dma_buf_get(g->mem_id);
		if (IS_ERR(dmabuf)) {
			err = PTR_ERR(dmabuf);
			goto err_map;
		}

		err = nvhost_vm_map_dmabuf(job->vm, dmabuf,
					   &job->addr_phys[pin_count]);
		dma_buf_put(dmabuf);
		if (err)
			goto err_map;

		dmabufs[pin_count] = dmabuf;

		pin_count++;
	}

	/* pin the buffers to the hardware */
	job->pin = nvhost_vm_pin_buffers(job->vm);

err_map:
	i = pin_count;
	while (i--)
		nvhost_vm_unmap_dmabuf(job->vm, dmabufs[i]);

	return err ? err : pin_count;
}

static int do_relocs(struct nvhost_job *job,
		u32 cmdbuf_mem, struct dma_buf *buf)
{
	int i = 0;
	int last_page = -1;
	void *cmdbuf_page_addr = NULL;

	/* pin & patch the relocs for one gather */
	while (i < job->num_relocs) {
		struct nvhost_reloc *reloc = &job->relocarray[i];
		struct nvhost_reloc_shift *shift = &job->relocshiftarray[i];

		/* skip all other gathers */
		if (cmdbuf_mem != reloc->cmdbuf_mem) {
			i++;
			continue;
		}

		if (last_page != reloc->cmdbuf_offset >> PAGE_SHIFT) {
			if (cmdbuf_page_addr)
				dma_buf_kunmap(buf, last_page,
						cmdbuf_page_addr);

			cmdbuf_page_addr = dma_buf_kmap(buf,
					reloc->cmdbuf_offset >> PAGE_SHIFT);
			last_page = reloc->cmdbuf_offset >> PAGE_SHIFT;

			if (unlikely(!cmdbuf_page_addr)) {
				pr_err("Couldn't map cmdbuf for relocation\n");
				return -ENOMEM;
			}
		}

		__raw_writel(
			(job->reloc_addr_phys[i] +
				reloc->target_offset) >> shift->shift,
			(void __iomem *)(cmdbuf_page_addr +
				(reloc->cmdbuf_offset & ~PAGE_MASK)));

		/* remove completed reloc from the job */
		if (i != job->num_relocs - 1) {
			struct nvhost_reloc *reloc_last =
				&job->relocarray[job->num_relocs - 1];
			struct nvhost_reloc_shift *shift_last =
				&job->relocshiftarray[job->num_relocs - 1];
			reloc->cmdbuf_mem	= reloc_last->cmdbuf_mem;
			reloc->cmdbuf_offset	= reloc_last->cmdbuf_offset;
			reloc->target		= reloc_last->target;
			reloc->target_offset	= reloc_last->target_offset;
			shift->shift		= shift_last->shift;
			job->reloc_addr_phys[i] =
				job->reloc_addr_phys[job->num_relocs - 1];
			job->num_relocs--;
		} else {
			break;
		}
	}

	if (cmdbuf_page_addr)
		dma_buf_kunmap(buf, last_page, cmdbuf_page_addr);

	return 0;
}


int nvhost_job_pin(struct nvhost_job *job, struct nvhost_syncpt *sp)
{
	int err = 0, i = 0, j = 0;
	int nb_hw_pts = nvhost_syncpt_nb_hw_pts(sp);
	DECLARE_BITMAP(waitchk_mask, nb_hw_pts);

	bitmap_zero(waitchk_mask, nb_hw_pts);
	for (i = 0; i < job->num_waitchk; i++) {
		u32 syncpt_id = job->waitchk[i].syncpt_id;
		if (nvhost_syncpt_is_valid_hw_pt(sp, syncpt_id))
			set_bit(syncpt_id, waitchk_mask);
	}

	/* get current syncpt values for waitchk */
	for_each_set_bit(i, waitchk_mask, nb_hw_pts)
		nvhost_syncpt_update_min(sp, i);

	/* pin memory */
	err = pin_job_mem(job);
	if (err <= 0)
		goto fail;

	/* patch gathers */
	for (i = 0; i < job->num_gathers; i++) {
		struct nvhost_job_gather *g = &job->gathers[i];

		/* process each gather mem only once */
		if (!g->buf) {
			g->buf = dma_buf_get(g->mem_id);
			if (IS_ERR(g->buf)) {
				err = PTR_ERR(g->buf);
				g->buf = NULL;
				break;
			}

			g->mem_base = job->gather_addr_phys[i];

			for (j = 0; j < job->num_gathers; j++) {
				struct nvhost_job_gather *tmp =
					&job->gathers[j];
				if (!tmp->buf && tmp->mem_id == g->mem_id) {
					tmp->buf = g->buf;
					tmp->mem_base = g->mem_base;
				}
			}
			err = do_relocs(job, g->mem_id,  g->buf);
			if (!err)
				err = do_waitchks(job, sp,
						g->mem_id, g->buf);
			dma_buf_put(g->buf);
			if (err)
				break;
		}
	}
fail:
	return err;
}

void nvhost_job_unpin(struct nvhost_job *job)
{
	if (job->vm)
		nvhost_vm_unpin_buffers(job->vm, job->pin);
}

/**
 * Debug routine used to dump job entries
 */
void nvhost_job_dump(struct device *dev, struct nvhost_job *job)
{
	dev_info(dev, "    SYNCPT_ID   %d\n",
		job->sp->id);
	dev_info(dev, "    SYNCPT_VAL  %d\n",
		job->sp->fence);
	dev_info(dev, "    FIRST_GET   0x%x\n",
		job->first_get);
	dev_info(dev, "    TIMEOUT     %d\n",
		job->timeout);
	dev_info(dev, "    NUM_SLOTS   %d\n",
		job->num_slots);
}
