// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (C) 2014 Ingenic Semiconductor Co., Ltd.
 * Author: qipengzhen <aric.pzqi@ingenic.com>
 *
 * Copyright (C) 2023 SudoMaker, Ltd.
 * Author: Reimu NotMoe <reimu@sudomaker.com>
 *
 * Based on various drivers.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "cim.h"

static void x1000_cim_dump_dma_desc(struct x1000_cim *cim)
{
#ifdef CONFIG_VIDEO_INGENIC_CIM_X1000_DEBUG
	struct cim_dma_desc *dma_desc = cim->dma_descs;

	printk("=> dma_desc: count=%u, virt=%p, phys=0x%x\n", cim->dma_desc_cnt, dma_desc, cim->dma_descs_phys);

	for (unsigned i = 0; i < cim->dma_desc_cnt; i++) {
		printk("dma_desc[%d].next: %08x\n", i, dma_desc[i].next);
		printk("dma_desc[%d].id: %08x\n", i, dma_desc[i].id);
		printk("dma_desc[%d].buf: %08x\n", i, dma_desc[i].buf);
		printk("dma_desc[%d].cmd: %08x\n", i, dma_desc[i].cmd);
		printk("dma_desc[%d].cb_frame: %08x\n", i, dma_desc[i].cb_frame);
		printk("dma_desc[%d].cb_len: %08x\n", i, dma_desc[i].cb_len);
		printk("dma_desc[%d].cr_frame: %08x\n", i, dma_desc[i].cr_frame);
		printk("dma_desc[%d].cr_len: %08x\n", i, dma_desc[i].cr_len);

	}
#endif
}

static inline struct x1000_cim_buffer *
vb2_v4l2_to_cim_buffer(const struct vb2_v4l2_buffer *p)
{
	return container_of(p, struct x1000_cim_buffer, vb);
}

static inline struct x1000_cim_buffer *
vb2_to_cim_buffer(const struct vb2_buffer *p)
{
	return vb2_v4l2_to_cim_buffer(to_vb2_v4l2_buffer(p));
}

static inline u32 x1000_cim_dma_desc_phys(struct x1000_cim *cim, unsigned index)
{
	return cim->dma_descs_phys + sizeof(struct cim_dma_desc) * index;
}

static inline struct cim_dma_desc *x1000_cim_dma_desc_phys_to_virt(struct x1000_cim *cim, dma_addr_t phys)
{
	u32 off = phys - cim->dma_descs_phys;
	return (struct cim_dma_desc *)((uint8_t *)cim->dma_descs + off);
}

static int x1000_cim_dma_alloc_desc(struct x1000_cim *cim, unsigned count)
{
	dma_addr_t dma_descs_phys;

	BUG_ON(count == 0);

	cim->dma_descs = dma_alloc_coherent(cim->dev,
			sizeof(struct cim_dma_desc) * count,
			&dma_descs_phys, GFP_KERNEL);

	if (!cim->dma_descs)
		return -ENOMEM;

	cim->dma_descs_phys = dma_descs_phys;
	cim->dma_desc_cnt = count;

	return 0;
}

static void x1000_cim_dma_free_desc(struct x1000_cim *cim)
{
	if (cim->dma_descs) {
		dma_free_coherent(cim->dev,
			sizeof(struct cim_dma_desc) * cim->dma_desc_cnt,
			cim->dma_descs, cim->dma_descs_phys);

		cim->dma_desc_cnt = 0;
		cim->dma_descs = NULL;
		cim->dma_descs_phys = 0;
	}
}

static int x1000_cim_dma_setup_desc(struct x1000_cim *cim, struct vb2_buffer *vb)
{
	struct cim_dma_desc *dma_descs = cim->dma_descs;
	dma_addr_t vmem_phys = vb2_dma_contig_plane_dma_addr(vb, 0);

	if (!vmem_phys) {
		dev_err(cim->dev, "Invalid DMA address for plane\n");
		return -EFAULT;
	}

	dma_descs[vb->index].id = vb->index;
	dma_descs[vb->index].buf = vmem_phys;
	dma_descs[vb->index].cmd = cim->fmt.sizeimage >> 2 | CIM_CMD_EOFINT | CIM_CMD_OFRCV;


	if (vb->index == 0) {
		cim->dma_desc_head = dma_descs;
	}

	if (vb->index == (cim->dma_desc_cnt - 1)) { // Last desc
		dma_descs[vb->index].next = x1000_cim_dma_desc_phys(cim, 0);
		dma_descs[vb->index].cmd |= CIM_CMD_STOP;

		cim->dma_desc_tail = &dma_descs[vb->index];
	} else {
		dma_descs[vb->index].next = x1000_cim_dma_desc_phys(cim, vb->index + 1);
	}

	return 0;
}

static void x1000_cim_dma_start(struct x1000_cim *cim)
{
	unsigned long temp = 0;
	u32 regval;

	// To prevent the RXFIFO from overflowing, it is important to enable
	// DMA before enabling the CIM control.
	// Enabling the CIM control before DMA can increase the likelihood
	// of RXFIFO overflow.

	regval = (u32)(cim->dma_descs_phys);
	writel(regval, cim->regs + CIM_DA);

	writel(0, cim->regs + CIM_STATE);

	/*enable dma*/
	temp = readl(cim->regs + CIM_CTRL);
	temp |= CIM_CTRL_DMA_EN;
	writel(temp, cim->regs + CIM_CTRL);

	/* clear rx fifo */
	temp = readl(cim->regs + CIM_CTRL);
	temp |= CIM_CTRL_RXF_RST;
	writel(temp, cim->regs + CIM_CTRL);

	temp = readl(cim->regs + CIM_CTRL);
	temp &= ~(CIM_CTRL_RXF_RST);
	writel(temp, cim->regs + CIM_CTRL);

	/* enable cim */
	temp = readl(cim->regs + CIM_CTRL);
	temp |= CIM_CTRL_ENA;
	writel(temp, cim->regs + CIM_CTRL);
}

static void x1000_cim_dma_stop(struct x1000_cim *cim)
{
	unsigned long temp = 0;

	/* unmask all interrupts. */
	writel(0xffffffff, cim->regs + CIM_IMR);

	/* clear rx fifo */
	temp = readl(cim->regs + CIM_CTRL);
	temp |= CIM_CTRL_RXF_RST | CIM_CTRL_CIM_RST;
	writel(temp, cim->regs + CIM_CTRL);

	writel(0, cim->regs + CIM_STATE);

	/* disable dma & cim */
	temp = readl(cim->regs + CIM_CTRL);
	temp &= ~(CIM_CTRL_ENA | CIM_CTRL_DMA_EN);
	writel(temp, cim->regs + CIM_CTRL);
}

static int x1000_cim_enable_clock(struct x1000_cim *cim)
{
	int ret;

	ret = clk_prepare_enable(cim->cim_clk);
	if (ret) {
		dev_err(cim->dev, "failed to enable CIM clock\n");
		return ret;
	}

	ret = clk_prepare_enable(cim->lcd_clk);
	if (ret) {
		dev_err(cim->dev, "failed to enable LCD clock\n");
		return ret;
	}

	return 0;
}

static void x1000_cim_disable_clock(struct x1000_cim *cim)
{
	clk_disable_unprepare(cim->cim_clk);
	clk_disable_unprepare(cim->lcd_clk);
}

static int x1000_cim_set_bus_param(struct x1000_cim *cim)
{
	struct v4l2_mbus_config_parallel *bus = &cim->bus;
	struct v4l2_pix_format *fmt = &cim->fmt;

	unsigned long cfg_reg = 0;
	unsigned long ctrl_reg = 0;
	unsigned long ctrl2_reg = 0;
	unsigned long fs_reg = 0;
	unsigned long temp = 0;

	/*PCLK Polarity Set*/
	cfg_reg = (bus->flags & V4L2_MBUS_PCLK_SAMPLE_FALLING) ?
		cfg_reg | CIM_CFG_PCP_HIGH : cfg_reg & (~CIM_CFG_PCP_HIGH);

	/*VSYNC Polarity Set*/
	cfg_reg = (bus->flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH) ?
		cfg_reg | CIM_CFG_VSP_HIGH : cfg_reg & (~CIM_CFG_VSP_HIGH);

	/*HSYNC Polarity Set*/
	cfg_reg = (bus->flags & V4L2_MBUS_HSYNC_ACTIVE_LOW) ?
		cfg_reg | CIM_CFG_HSP_HIGH : cfg_reg & (~CIM_CFG_HSP_HIGH);

	cfg_reg |= CIM_CFG_DMA_BURST_INCR64 | CIM_CFG_DSM_GCM | CIM_CFG_PACK_Y0UY1V;

	ctrl_reg |= CIM_CTRL_DMA_SYNC | CIM_CTRL_FRC_1;

	ctrl2_reg |= CIM_CTRL2_APM | CIM_CTRL2_OPE |
		(1 << CIM_CTRL2_OPG_BIT);

	// FIXME: Frame size check doesn't work as intended.
	// Needs revisit.
#if 0
	ctrl2_reg |= CIM_CTRL2_FSC | CIM_CTRL2_ARIF;
#endif

	fs_reg = (fmt->width -1) << CIM_FS_FHS_BIT | (fmt->height -1)
		<< CIM_FS_FVS_BIT | 1 << CIM_FS_BPP_BIT;

	// BS0 BS1 BS2 BS3 must be 00,01,02,03 when pack is b100
	if (cfg_reg & CIM_CFG_PACK_Y0UY1V)
		cfg_reg |= CIM_CFG_BS1_2_OBYT1 | CIM_CFG_BS2_2_OBYT2 | CIM_CFG_BS3_2_OBYT3;

	writel(cfg_reg, cim->regs + CIM_CFG);
	writel(ctrl_reg, cim->regs + CIM_CTRL);
	writel(ctrl2_reg, cim->regs + CIM_CTRL2);
	writel(fs_reg, cim->regs + CIM_FS);

#ifdef CONFIG_VIDEO_INGENIC_CIM_X1000_DEBUG
	pr_info("=> x1000_cim_set_bus_param: %ux%u, bus->flags=0x%x\n", fmt->width, fmt->height, bus->flags);
	x1000_cim_dump_regs(cim);
#endif

	/* Enable end of frame (EOF) interrupt */
	temp = readl(cim->regs + CIM_IMR);
	temp &= ~(CIM_IMR_EOFM | CIM_IMR_STPM | CIM_IMR_STPM_1);
	writel(temp, cim->regs + CIM_IMR);

	return 0;
}

static int x1000_cim_queue_setup(struct vb2_queue *vq,
				 unsigned int *nbuffers,
				 unsigned int *nplanes,
				 unsigned int sizes[],
				 struct device *alloc_devs[])
{
	struct x1000_cim *cim = vb2_get_drv_priv(vq);
	unsigned long size = cim->fmt.sizeimage;

	// We don't support multi planes (separate buffers for Y/UV, etc).
	*nplanes = 1;

	if (!*nbuffers || *nbuffers > cim->max_buffer_cnt)
		*nbuffers = cim->max_buffer_cnt;

	if (size * *nbuffers > cim->max_vmem_bytes)
		*nbuffers = cim->max_vmem_bytes / size;

	sizes[0] = size;
	alloc_devs[0] = cim->dev;

	cim->streaming_started = 0;
	cim->sequence = 0;

	if (x1000_cim_dma_alloc_desc(cim, *nbuffers))
		return -ENOMEM;

	return 0;
};

static int x1000_cim_buffer_init(struct vb2_buffer *vb)
{
	struct x1000_cim *cim = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct x1000_cim_buffer *buf = container_of(vbuf, struct x1000_cim_buffer, vb);
	int ret;

	ret = x1000_cim_dma_setup_desc(cim, vb);
	if (ret) {
		dev_err(cim->dev, "failed to setup DMA descriptors\n");
		return ret;
	}

#ifdef CONFIG_VIDEO_INGENIC_CIM_X1000_DEBUG
	dev_info(cim->dev, "cim_buffer_init\n");
#endif

	return 0;
}

static int x1000_cim_buffer_prepare(struct vb2_buffer *vb)
{
	struct x1000_cim *cim = vb2_get_drv_priv(vb->vb2_queue);

	unsigned long size = cim->fmt.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(cim->dev, "buffer too small (%lu < %lu)\n",
			vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	return 0;
}

static void x1000_cim_buffer_queue(struct vb2_buffer *vb)
{
	struct x1000_cim *cim = vb2_get_drv_priv(vb->vb2_queue);
	struct x1000_cim_buffer *buf = vb2_to_cim_buffer(vb);
	u32 regval = 0;
	unsigned long flags;

	// dev_info(cim->dev, "cim_buffer_queue\n");

	spin_lock_irqsave(&cim->qlock, flags);

	list_add_tail(&buf->queue, &cim->buf_list);

	if (cim->buf_active == NULL) {
		cim->buf_active = buf; // Buffer to be transferred.
	}

	if (!cim->streaming_started) {
		goto out;
	}

	if ((vb->index > cim->dma_desc_cnt)) {
		dev_err(cim->dev, "vb->index (%u) larger than dma_desc_cnt (%u)\n",
				vb->index, cim->dma_desc_cnt);
		goto out;
	}


	if (cim->dma_desc_head != cim->dma_desc_tail) { /* DMA descriptor added dynamically */
		cim->dma_descs[vb->index].cmd |= CIM_CMD_STOP;

		cim->dma_desc_tail->next = x1000_cim_dma_desc_phys(cim, vb->index);
		cim->dma_desc_tail->cmd &= (~CIM_CMD_STOP);	/* unlink link last dma desc */

		cim->dma_desc_tail = &cim->dma_descs[vb->index]; /* update newly tail */
	} else {
		if (!cim->dma_started) {
			/*
			 * DMA stoppage can occur when all buffers have been dequeued by the user application.
			 * To address this issue:
			 * 1. CIM_CMD_STOP is set for the current video buffer.
			 * 2. CIM_DA is reconfigured to initiate a new DMA transfer.
			 *
			 * Known issues:
			 * - CIM will drop one frame if the user application fails to queue a buffer in time.
			 */
			cim->dma_descs[vb->index].cmd |= CIM_CMD_STOP;

			cim->dma_desc_head = &cim->dma_descs[vb->index];
			cim->dma_desc_tail = &cim->dma_descs[vb->index];

			/* Configure register CIMDA */
			regval = x1000_cim_dma_desc_phys(cim, vb->index);
			writel(regval, cim->regs + CIM_DA);

			cim->dma_started = true;

		} else {
			cim->dma_descs[vb->index].cmd |= CIM_CMD_STOP;

			cim->dma_desc_tail->next = x1000_cim_dma_desc_phys(cim, vb->index);
			cim->dma_desc_tail->cmd &= (~CIM_CMD_STOP);

			cim->dma_desc_tail = &cim->dma_descs[vb->index];
		}
	}

	out:
	spin_unlock_irqrestore(&cim->qlock, flags);
}

static int x1000_cim_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct x1000_cim *cim = vb2_get_drv_priv(vq);
	const struct x1000_cim_format *cim_fmt;
	unsigned long flags;
	int ret;

	cim_fmt = x1000_cim_find_format(&cim->fmt.pixelformat, NULL);
	if (!cim_fmt)
		return -EINVAL;

	dev_dbg(cim->dev, "Starting capture\n");

	cim->sequence = 0;

	ret = video_device_pipeline_alloc_start(&cim->vdev);
	if (ret < 0)
		goto err;

	spin_lock_irqsave(&cim->qlock, flags);

	x1000_cim_set_bus_param(cim);
	x1000_cim_dump_dma_desc(cim);
	x1000_cim_dma_start(cim);
	cim->streaming_started = true;
	cim->dma_started = true;

	spin_unlock_irqrestore(&cim->qlock, flags);

	ret = v4l2_subdev_call(cim->src_subdev, video, s_stream, 1);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		goto err;

	dev_info(cim->dev, "Done starting capture\n");

	x1000_cim_dump_regs(cim);

	return 0;

err:
	x1000_cim_dma_stop(cim);
	cim->streaming_started = false;
	cim->dma_started = false;

	return ret;
}

static void x1000_cim_stop_streaming(struct vb2_queue *vq)
{
	struct x1000_cim *cim = vb2_get_drv_priv(vq);
	struct x1000_cim_buffer *buf, *node;
	unsigned long flags;

	dev_info(cim->dev, "Stopping capture\n");

	v4l2_subdev_call(cim->src_subdev, video, s_stream, 0);

	spin_lock_irqsave(&cim->qlock, flags);

	cim->buf_active = NULL;
	x1000_cim_dma_stop(cim);

	list_for_each_entry_safe(buf, node, &cim->buf_list, queue) {
		list_del_init(&buf->queue);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	cim->dma_started = false;
	cim->streaming_started = false;

	spin_unlock_irqrestore(&cim->qlock, flags);

	video_device_pipeline_stop(&cim->vdev);
	x1000_cim_dma_free_desc(cim);
}

static const struct vb2_ops x1000_cim_qops = {
	.queue_setup		= x1000_cim_queue_setup,
	.buf_init		= x1000_cim_buffer_init,
	.buf_prepare		= x1000_cim_buffer_prepare,
	.buf_queue		= x1000_cim_buffer_queue,
	.start_streaming	= x1000_cim_start_streaming,
	.stop_streaming		= x1000_cim_stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static irqreturn_t x1000_cim_irq(int irq, void *data)
{
	struct x1000_cim *cim = (struct x1000_cim *)data;
	unsigned long status = 0, temp = 0;
	unsigned long flags = 0;
	u32 regval = 0;

	/* judged cim->dma_desc_head->id */
	if (cim->dma_desc_head->id > cim->dma_desc_cnt) {
		dev_warn(cim->dev, "dma_desc_head->id overflowed (%u)\n",
				cim->dma_desc_head->id);
		return IRQ_NONE;
	}

	spin_lock_irqsave(&cim->qlock, flags);

	/* read interrupt status register */
	status = readl(cim->regs + CIM_STATE);
	if (!status) {
		dev_warn(cim->dev, "we have nothing to do in interrupt\n");
		spin_unlock_irqrestore(&cim->qlock, flags);
		return IRQ_NONE;
	}

	if (!(status & CIM_STATE_RXOF_STOP_EOF)) {
		/* other irq */
		dev_warn(cim->dev, "irq_handle status is 0x%lx, not judged in irq_handle\n", status);
		spin_unlock_irqrestore(&cim->qlock, flags);
		return IRQ_HANDLED;
	}

	if (status & CIM_STATE_DMA_STOP) {
		/* clear dma interrupt status */
		temp = readl(cim->regs + CIM_STATE);
		temp &= (~CIM_STATE_DMA_STOP);
		writel(temp, cim->regs + CIM_STATE);

		cim->dma_started = false;
	}

	if (status & CIM_STATE_DMA_EOF) {
		/* clear dma interrupt status */
		temp = readl(cim->regs + CIM_STATE);
		temp &= (~CIM_STATE_DMA_EOF);
		writel(temp, cim->regs + CIM_STATE);

		if (cim->buf_active) {
			struct vb2_v4l2_buffer *vbuf = &cim->buf_active->vb;
			struct x1000_cim_buffer *buf = container_of(vbuf, struct x1000_cim_buffer, vb);

			list_del_init(&buf->queue);
			vbuf->field = cim->fmt.field;
			vbuf->sequence = cim->sequence++;
			vb2_buffer_done(&vbuf->vb2_buf, VB2_BUF_STATE_DONE);

#ifdef CONFIG_VIDEO_INGENIC_CIM_X1000_DEBUG
			if ((vbuf->sequence % 60) == 0)  {
				cim->debug_ms_start = ktime_to_ms(ktime_get_real());
			} else if ((vbuf->sequence % 60) == 59) {
				u64 debug_ms_end = ktime_to_ms(ktime_get_real());
				u32 ms = debug_ms_end - cim->debug_ms_start;

				u64 fps = 60 * 1000;
				do_div(fps, ms);

				pr_info("=> fps: %lld, start: %lld, end: %lld, sequence: %d\n",\
					fps, cim->debug_ms_start, debug_ms_end, cim->sequence);
			}
#endif
		}

		if (list_empty(&cim->buf_list)) {
			cim->buf_active = NULL;
			spin_unlock_irqrestore(&cim->qlock, flags);
			return IRQ_HANDLED;
		}

		if (cim->dma_desc_head != cim->dma_desc_tail) {
			cim->dma_desc_head = x1000_cim_dma_desc_phys_to_virt(cim, cim->dma_desc_head->next);
		}

		if (!cim->dma_started && !list_empty(&cim->buf_list)) {
			// This paragraph describes the conditions for stopping DMA:
			// 1. When the DMA descriptor reaches the end and there are no more
			// descriptors to be transferred, the DMA will stop.
			// 2. If the capture list is not empty, DMA should be restarted
			// at this point.

			regval = x1000_cim_dma_desc_phys(cim, cim->dma_desc_head->id);
			writel(regval, cim->regs + CIM_DA);

			cim->dma_started = true;
		}

		// Update dma active buffer
		// FIXME:
		// We make the assumption that an EOF interrupt results in the DMA transferring
		// only one frame. However, it is possible that during a system's busy state,
		// two or more frames may be transferred in a single EOF interrupt.

		cim->buf_active = list_first_entry(&cim->buf_list, struct x1000_cim_buffer, queue);

		spin_unlock_irqrestore(&cim->qlock, flags);

		return IRQ_HANDLED;
	}

	spin_unlock_irqrestore(&cim->qlock, flags);
	return IRQ_HANDLED;
}

int x1000_cim_dma_register(struct x1000_cim *cim, int irq)
{
	struct vb2_queue *q = &cim->queue;
	int ret;

	spin_lock_init(&cim->qlock);
	mutex_init(&cim->lock);

	INIT_LIST_HEAD(&cim->buf_list);

	q->min_buffers_needed = 1;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->lock = &cim->lock;
	q->drv_priv = cim;
	q->buf_struct_size = sizeof(struct x1000_cim_buffer);
	q->ops = &x1000_cim_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->dev = cim->dev;

	ret = x1000_cim_enable_clock(cim);
	if (ret)
		return ret;

	ret = vb2_queue_init(q);
	if (ret < 0) {
		dev_err(cim->dev, "failed to initialize VB2 queue\n");
		goto err_free_mutex;
	}

	ret = v4l2_device_register(cim->dev, &cim->v4l);
	if (ret) {
		dev_err(cim->dev, "Couldn't register the v4l2 device\n");
		goto err_free_mutex;
	}

	ret = devm_request_irq(cim->dev, irq, x1000_cim_irq, 0,
			       dev_name(cim->dev), cim);
	if (ret) {
		dev_err(cim->dev, "Couldn't register our interrupt\n");
		goto err_unregister_device;
	}

	return 0;

err_unregister_device:
	v4l2_device_unregister(&cim->v4l);

err_free_mutex:
	mutex_destroy(&cim->lock);
	return ret;
}

void x1000_cim_dma_unregister(struct x1000_cim *cim)
{
	v4l2_device_unregister(&cim->v4l);
	mutex_destroy(&cim->lock);
}
