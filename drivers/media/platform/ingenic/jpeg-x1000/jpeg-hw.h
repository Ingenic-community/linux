/*
 *
 * Copyright (c) 2012 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * Author: Gao Wei <wei.gao@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef JPEG_HW_H_
#define JPEG_HW_H_

#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>

#include <linux/clk.h>
#include <linux/syscalls.h>



#include <linux/io.h>
#include <linux/videodev2.h>

#include "jpeg-regs.h"

#define INGENIC_JPEG_MIN_WIDTH			32
#define INGENIC_JPEG_MIN_HEIGHT			32
#define INGENIC_JPEG_MAX_WIDTH			8192
#define INGENIC_JPEG_MAX_HEIGHT			8192
#define INGENIC_JPEG_ENCODE			0
#define INGENIC_JPEG_RAW_IN_565			0
#define INGENIC_JPEG_RAW_IN_422			1
#define INGENIC_JPEG_RAW_OUT_422		0
#define INGENIC_JPEG_RAW_OUT_420		1

#define vpu_readl(vpu, offset)		__raw_readl((vpu)->iomem + offset)
#define vpu_writel(vpu, offset, value)	__raw_writel((value), (vpu)->iomem + offset)

#define CLEAR_REG_BIT(regs,offset,bm)				\
	do {							\
		unsigned int stat;				\
		stat = readl(regs + offset);			\
		writel(stat & ~(bm), regs + offset);		\
	} while(0)

#define CHECK_SCH_STAT(STAT, fmt, args...) do {	\
		if(sch_stat & STAT)			\
			dev_err(jpeg->dev, fmt, ##args);	\
	}while(0)

static inline void cpm_writel(unsigned int val, unsigned int off)
{
	writel(val, (void __iomem *)0xb0000000 + off);
}

static inline unsigned int cpm_readl(unsigned int off)
{
	return readl((void __iomem *)0xb0000000 + off);
}

static int inline jpeg_reset(void __iomem *regs)
{
	int timeout = 0xffffff;
	unsigned int srbc = cpm_readl(CPM_SRBC);

	cpm_writel(srbc | (1 << 30), CPM_SRBC);
	while (!(cpm_readl(CPM_SRBC) & (1 << 29)) && --timeout);

	if (timeout == 0) {
		cpm_writel(srbc, CPM_SRBC);
		return -1;
	} else {
		cpm_writel(srbc | (1 << 31), CPM_SRBC);
		cpm_writel(srbc, CPM_SRBC);
	}

	return 0;
}

#if 0
static inline int jpeg_poweron(struct x1000_jpeg *jpeg)
{
	if (cpm_readl(CPM_OPCR) & OPCR_IDLE)
		return -EBUSY;

	clk_enable(jpeg->clk);
	__asm__ __volatile__ (
		"mfc0  $2, $16,  7   \n\t"
		"ori   $2, $2, 0x340 \n\t"
		"andi  $2, $2, 0x3ff \n\t"
		"mtc0  $2, $16,  7  \n\t"
		"nop                  \n\t");
	jpeg_reset(jpeg->regs);
	enable_irq(jpeg->irq);
	wake_lock(&jpeg->wake_lock);
	dev_dbg(jpeg->dev, "[%d:%d] on\n", current->tgid, current->pid);

	return 0;
}

static long jpeg_off(struct x1000_jpeg *jpeg)
{
	disable_irq_nosync(jpeg->irq);

	__asm__ __volatile__ (
		"mfc0  $2, $16,  7   \n\t"
		"andi  $2, $2, 0xbf \n\t"
		"mtc0  $2, $16,  7  \n\t"
		"nop                  \n\t");

	cpm_clear_bit(31,CPM_OPCR);
	clk_disable(jpeg->clk);
	/* Clear completion use_count here to avoid a unhandled irq after vpu off */
	wake_unlock(&jpeg->wake_lock);
	dev_dbg(jpeg->dev, "[%d:%d] off\n", current->tgid, current->pid);

	return 0;
}
#endif
#if 0
static inline void jpeg_input_raw_mode(void __iomem *regs, unsigned long mode)
{
}

static inline void jpeg_proc_mode(void __iomem *regs, unsigned long mode)
{
}

static inline void jpeg_subsampling_mode(void __iomem *regs, unsigned int mode)
{
}

static inline unsigned int jpeg_get_subsampling_mode(void __iomem *regs)
{
}
#endif

static inline void jpeg_start(void __iomem *regs, unsigned int desc_phys)
{
        writel(desc_phys | VDMA_ACFG_RUN, regs + REG_VMDA_TRIG);
}

static inline unsigned int jpeg_get_sch_stat(void __iomem *regs)
{
        return readl(regs + REG_SCH_STAT);
}

static inline unsigned int jpeg_get_aux_stat(void __iomem *regs)
{
        return readl(regs + REG_AUX_STAT);
}

static inline void jpeg_set_hiaxi(void __iomem *regs)
{
        writel(SCH_GLBC_HIAXI, regs + REG_SCH_GLBC);
}

static inline void jpeg_clear_stat(void __iomem *regs)
{
        writel(0, regs + REG_JPGC_STAT);
}
static inline void jpeg_clear_int(void __iomem *regs)
{
	unsigned long reg;

        reg = readl(regs + REG_SCH_GLBC);
        writel(reg | SCH_INTE_ACFGERR | SCH_INTE_TLBERR
                | SCH_INTE_BSERR | SCH_INTE_ENDF, regs + REG_SCH_GLBC);
}

static inline unsigned int jpeg_compressed_size(void __iomem *regs)
{
	unsigned long reg, jpeg_size = 0;

        reg = readl(regs + REG_JPGC_STAT);
        jpeg_size = reg & 0xffffff;

	return (unsigned int)jpeg_size;
}

#endif /* JPEG_HW_H_ */
