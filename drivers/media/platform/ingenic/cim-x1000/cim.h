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

#pragma once

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/videobuf2-core.h>

/*
 * CIM registers
 */
#define CIM_CFG				(0x00)
#define CIM_CTRL			(0x04)
#define CIM_STATE			(0x08)
#define CIM_IID				(0x0c)
#define CIM_DA				(0x20)
#define CIM_FA				(0x24)
#define CIM_FID				(0x28)
#define CIM_CMD				(0x2c)
#define CIM_SIZE			(0x30)
#define CIM_OFFSET			(0x34)
#define CIM_CTRL2			(0x50)
#define CIM_FS				(0x54)
#define CIM_IMR				(0x58)

/* CIM Configuration Register (CIMCFG) */
#define CIM_CFG_VSP_HIGH		(1 << 14) 			/* VSYNC Polarity: 1-falling edge active */
#define CIM_CFG_HSP_HIGH		(1 << 13) 			/* HSYNC Polarity: 1-falling edge active */
#define CIM_CFG_PCP_HIGH		(1 << 12) 			/* PCLK working edge: 1-falling */

#define CIM_CFG_DMA_BURST_TYPE		10
#define CIM_CFG_DMA_BURST_INCR8		(0 << CIM_CFG_DMA_BURST_TYPE)
#define CIM_CFG_DMA_BURST_INCR16	(1 << CIM_CFG_DMA_BURST_TYPE)
#define CIM_CFG_DMA_BURST_INCR32	(2 << CIM_CFG_DMA_BURST_TYPE)
#define CIM_CFG_DMA_BURST_INCR64	(3 << CIM_CFG_DMA_BURST_TYPE)

#define CIM_CFG_PACK			4
#define CIM_CFG_PACK_VY1UY0		(0 << CIM_CFG_PACK)
#define CIM_CFG_PACK_Y0VY1U		(1 << CIM_CFG_PACK)
#define CIM_CFG_PACK_UY0VY1		(2 << CIM_CFG_PACK)
#define CIM_CFG_PACK_Y1UY0V		(3 << CIM_CFG_PACK)
#define CIM_CFG_PACK_Y0UY1V		(4 << CIM_CFG_PACK)
#define CIM_CFG_PACK_UY1VY0		(5 << CIM_CFG_PACK)
#define CIM_CFG_PACK_Y1VY0U		(6 << CIM_CFG_PACK)
#define CIM_CFG_PACK_VY0UY1		(7 << CIM_CFG_PACK)

#define CIM_CFG_BS0             	16
#define CIM_CFG_BS0_2_OBYT0		(0 << CIM_CFG_BS0)
#define CIM_CFG_BS1_2_OBYT0     	(1 << CIM_CFG_BS0)
#define CIM_CFG_BS2_2_OBYT0     	(2 << CIM_CFG_BS0)
#define CIM_CFG_BS3_2_OBYT0     	(3 << CIM_CFG_BS0)

#define CIM_CFG_BS1             	18
#define CIM_CFG_BS0_2_OBYT1		(0 << CIM_CFG_BS1)
#define CIM_CFG_BS1_2_OBYT1     	(1 << CIM_CFG_BS1)
#define CIM_CFG_BS2_2_OBYT1     	(2 << CIM_CFG_BS1)
#define CIM_CFG_BS3_2_OBYT1     	(3 << CIM_CFG_BS1)

#define CIM_CFG_BS2             	20
#define CIM_CFG_BS0_2_OBYT2		(0 << CIM_CFG_BS2)
#define CIM_CFG_BS1_2_OBYT2     	(1 << CIM_CFG_BS2)
#define CIM_CFG_BS2_2_OBYT2     	(2 << CIM_CFG_BS2)
#define CIM_CFG_BS3_2_OBYT2     	(3 << CIM_CFG_BS2)

#define CIM_CFG_BS3             	22
#define CIM_CFG_BS0_2_OBYT3		(0 << CIM_CFG_BS3)
#define CIM_CFG_BS1_2_OBYT3     	(1 << CIM_CFG_BS3)
#define CIM_CFG_BS2_2_OBYT3     	(2 << CIM_CFG_BS3)
#define CIM_CFG_BS3_2_OBYT3     	(3 << CIM_CFG_BS3)

#define CIM_CFG_DSM			0
#define CIM_CFG_DSM_CPM			(0 << CIM_CFG_DSM)		/* CCIR656 Progressive Mode */
#define CIM_CFG_DSM_CIM			(1 << CIM_CFG_DSM)		/* CCIR656 Interlace Mode */
#define CIM_CFG_DSM_GCM			(2 << CIM_CFG_DSM)		/* Gated Clock Mode */

/* CIM State Register  (CIM_STATE) */
#define CIM_STATE_DMA_EEOF		(1 << 11)			/* DMA Line EEOf irq */
#define CIM_STATE_DMA_STOP		(1 << 10)			/* DMA stop irq */
#define CIM_STATE_DMA_EOF		(1 << 9)			/* DMA end irq */
#define CIM_STATE_DMA_SOF		(1 << 8)			/* DMA start irq */
#define CIM_STATE_SIZE_ERR		(1 << 3)			/* Frame size check error */
#define CIM_STATE_RXF_OF		(1 << 2)			/* RXFIFO over flow irq */
#define CIM_STATE_RXF_EMPTY		(1 << 1)			/* RXFIFO empty irq */
#define CIM_STATE_STP_ACK		(1 << 0)			/* CIM disabled status */
#define CIM_STATE_RXOF_STOP_EOF		(CIM_STATE_RXF_OF | CIM_STATE_DMA_STOP | CIM_STATE_DMA_EOF)

/* CIM DMA Command Register (CIM_CMD) */
#define CIM_CMD_SOFINT			(1 << 31)			/* enable DMA start irq */
#define CIM_CMD_EOFINT			(1 << 30)			/* enable DMA end irq */
#define CIM_CMD_EEOFINT			(1 << 29)			/* enable DMA EEOF irq */
#define CIM_CMD_STOP			(1 << 28)			/* enable DMA stop irq */
#define CIM_CMD_OFRCV			(1 << 27)

/* CIM Control Register (CIMCR) */
#define CIM_CTRL_FRC_BIT		16
#define CIM_CTRL_FRC_1			(0x0 << CIM_CTRL_FRC_BIT) 	/* Sample every n+1 frame */
#define CIM_CTRL_FRC_10			(0x9 << CIM_CTRL_FRC_BIT)

#define CIM_CTRL_DMA_SYNC		(1 << 7)        		/*when change DA, do frame sync */
#define CIM_CTRL_STP_REQ		(1 << 4) 			/*request to stop */
#define CIM_CTRL_CIM_RST		(1 << 3)
#define CIM_CTRL_DMA_EN			(1 << 2) 			/* Enable DMA */
#define CIM_CTRL_RXF_RST		(1 << 1) 			/* RxFIFO reset */
#define CIM_CTRL_ENA			(1 << 0) 			/* Enable CIM */

/* CIM Control Register 2 (CIMCR2) */
#define CIM_CTRL2_FSC			(1 << 23)			/* enable frame size check */
#define CIM_CTRL2_ARIF			(1 << 22)			/* enable auto-recovery for incomplete frame */
#define CIM_CTRL2_OPG_BIT		4				/* option priority configuration */
#define CIM_CTRL2_OPG_MASK		(0x3 << CIM_CTRL2_OPG_BIT)
#define CIM_CTRL2_OPE			(1 << 2)			/* optional priority mode enable */
#define CIM_CTRL2_APM			(1 << 0)			/* auto priority mode enable*/

/* CIM Interrupt Mask Register (CIMIMR) */
#define CIM_IMR_STPM			(1<<10)
#define CIM_IMR_EOFM			(1<<9)
#define CIM_IMR_SOFM			(1<<8)
#define CIM_IMR_FSEM			(1<<3)
#define CIM_IMR_RFIFO_OFM		(1<<2)
#define CIM_IMR_STPM_1			(1<<0)

/* CIM Frame Size Register (CIM_FS) */
#define CIM_FS_FVS_BIT			16				/* vertical size of the frame */
#define CIM_FS_FVS_MASK			(0x1fff << CIM_FS_FVS_BIT)
#define CIM_FS_BPP_BIT			14				/* bytes per pixel */
#define CIM_FS_BPP_MASK			(0x3 << CIM_FS_BPP_BIT)
#define CIM_FS_FHS_BIT			0       			/* horizontal size of the frame */
#define CIM_FS_FHS_MASK			(0x1fff << CIM_FS_FHS_BIT)

#define CIM_MAX_HEIGHT		8192U
#define CIM_MAX_WIDTH		8192U

/*
 * Structures
 */
struct cim_dma_desc {
	dma_addr_t next;
	unsigned int id;
	unsigned int buf;
	unsigned int cmd;
	/* only used when SEP = 1 */
	unsigned int cb_frame;
	unsigned int cb_len;
	unsigned int cr_frame;
	unsigned int cr_len;
} __attribute__ ((aligned (32)));

/* buffer for one video frame */
struct x1000_cim_buffer {
	/* common v4l buffer stuff -- must be first */
	struct vb2_v4l2_buffer vb;
	struct list_head queue;
};

enum csi_subdev_pads {
	CSI_SUBDEV_SINK,
	CSI_SUBDEV_SOURCE,

	CSI_SUBDEV_PADS,
};

extern const struct v4l2_subdev_ops x1000_cim_subdev_ops;

struct x1000_cim_format {
	u32			mbus_code;
	u32			fourcc;
	u8			bpp;
	unsigned int		hsub;
	unsigned int		vsub;
};

const struct x1000_cim_format *x1000_cim_find_format(const u32 *fourcc,
						     const u32 *mbus);

struct x1000_cim {
	/* Device resources */
	struct device			*dev;

	void __iomem			*regs;
	struct clk			*cim_clk;
	struct clk			*lcd_clk;

	unsigned			max_buffer_cnt;
	unsigned			max_vmem_bytes;

	unsigned			dma_desc_cnt;
	struct cim_dma_desc		*dma_descs, *dma_desc_head, *dma_desc_tail;
	dma_addr_t			dma_descs_phys;

	struct vb2_v4l2_buffer		**current_buf;

	const struct x1000_cim_format	*cim_fmt;
	struct v4l2_mbus_config_parallel	bus;

	/* Main Device */
	struct v4l2_device		v4l;
	struct media_device		mdev;
	struct video_device		vdev;
	struct media_pad		vdev_pad;
	struct v4l2_pix_format		fmt;

	/* Local subdev */
	struct v4l2_subdev		subdev;
	struct media_pad		subdev_pads[CSI_SUBDEV_PADS];
	struct v4l2_mbus_framefmt	subdev_fmt;

	/* V4L2 Async variables */
	struct v4l2_async_notifier	notifier;
	struct v4l2_subdev		*src_subdev;
	int				src_pad;

	/* V4L2 variables */
	struct mutex			lock;

	/* Videobuf2 */
	struct vb2_queue		queue;
	struct list_head		buf_list;
	struct x1000_cim_buffer		*buf_active;
	spinlock_t			qlock;
	bool				streaming_started;
	bool				dma_started;
	unsigned int			sequence;

#ifdef CONFIG_VIDEO_INGENIC_CIM_X1000_DEBUG
	/* Debugging */
	s64 debug_ms_start;
#endif
};

void x1000_cim_dump_regs(struct x1000_cim *cim);

int x1000_cim_dma_register(struct x1000_cim *cim, int irq);
void x1000_cim_dma_unregister(struct x1000_cim *cim);

int x1000_cim_v4l2_register(struct x1000_cim *cim);
