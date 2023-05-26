/*
 * Ingenic MMC/SD Controller driver
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 * Written by Large Dipper <ykli@ingenic.com>.
 *
 * Modified by qipengzhen <aric.pzqi@ingenic.com> 2016-04-21
 *
 * Copyright (C) 2023 SudoMaker, Ltd.
 * Author: Reimu NotMoe <reimu@sudomaker.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <asm-generic/delay.h>

#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/clk-provider.h>

#define	MSC_CTRL			0x000
#define	MSC_STAT			0x004
#define	MSC_CLKRT			0x008
#define	MSC_CMDAT			0x00C
#define	MSC_RESTO			0x010
#define	MSC_RDTO			0x014
#define	MSC_BLKLEN			0x018
#define	MSC_NOB				0x01C
#define	MSC_SNOB			0x020
#define	MSC_IMASK			0x024
#define	MSC_IFLG			0x028
#define	MSC_CMD				0x02C
#define	MSC_ARG				0x030
#define	MSC_RES				0x034
#define	MSC_RXFIFO			0x038
#define	MSC_TXFIFO			0x03C
#define	MSC_LPM				0x040
#define	MSC_DMAC			0x044
#define	MSC_DMANDA			0x048
#define	MSC_DMADA			0x04C
#define	MSC_DMALEN			0x050
#define	MSC_DMACMD			0x054
#define	MSC_CTRL2			0x058
#define	MSC_RTCNT			0x05C
#define	MSC_DEBUG			0x0FC

/* MSC Clock and Control Register (MSC_CTRL) */
#define CTRL_SEND_CCSD			(1 << 15)		/*send command completion signal disable to ceata */
#define CTRL_SEND_AS_CCSD		(1 << 14)		/*send internally generated stop after sending ccsd */
#define CTRL_EXIT_MULTIPLE		(1 << 7)
#define CTRL_EXIT_TRANSFER		(1 << 6)
#define CTRL_START_READWAIT		(1 << 5)
#define CTRL_STOP_READWAIT		(1 << 4)
#define CTRL_RESET			(1 << 3)
#define CTRL_START_OP			(1 << 2)
#define CTRL_CLOCK_SHF			0
#define CTRL_CLOCK_MASK			(0x3 << CTRL_CLOCK_SHF)
#define CTRL_CLOCK_STOP			(0x1 << CTRL_CLOCK_SHF)	/* Stop MMC/SD clock */
#define CTRL_CLOCK_START		(0x2 << CTRL_CLOCK_SHF)	/* Start MMC/SD clock */

/* MSC Control 2 Register (MSC_CTRL2) */
#define	CTRL2_PIP_SHF			24
#define	CTRL2_PIP_MASK			(0x1f << CTRL2_PIP_SHF)
#define	CTRL2_RST_EN			(1 << 23)
#define	CTRL2_STPRM			(1 << 4)
#define	CTRL2_SVC			(1 << 3)
#define	CTRL2_SMS_SHF			0
#define	CTRL2_SMS_MASK			(0x7 << CTRL2_SMS_SHF)
#define	CTRL2_SMS_DEFSPD		(0x0 << CTRL2_SMS_SHF)
#define	CTRL2_SMS_HISPD			(0x1 << CTRL2_SMS_SHF)
#define	CTRL2_SMS_SDR12			(0x2 << CTRL2_SMS_SHF)
#define	CTRL2_SMS_SDR25			(0x3 << CTRL2_SMS_SHF)
#define	CTRL2_SMS_SDR50			(0x4 << CTRL2_SMS_SHF)

/* MSC Status Register (MSC_STAT) */
#define STAT_AUTO_CMD12_DONE			(1 << 31)
#define STAT_AUTO_CMD23_DONE			(1 << 30)
#define STAT_SVS				(1 << 29)
#define STAT_PIN_LEVEL_SHF			24
#define STAT_PIN_LEVEL_MASK			(0x1f << STAT_PIN_LEVEL_SHF)
#define STAT_BCE				(1 << 20)
#define STAT_BDE				(1 << 19)
#define STAT_BAE				(1 << 18)
#define STAT_BAR				(1 << 17)
#define STAT_IS_RESETTING			(1 << 15)
#define STAT_SDIO_INT_ACTIVE			(1 << 14)
#define STAT_PRG_DONE				(1 << 13)
#define STAT_DATA_TRAN_DONE			(1 << 12)
#define STAT_END_CMD_RES			(1 << 11)
#define STAT_DATA_FIFO_AFULL			(1 << 10)
#define STAT_IS_READWAIT			(1 << 9)
#define STAT_CLK_EN				(1 << 8)
#define STAT_DATA_FIFO_FULL			(1 << 7)
#define STAT_DATA_FIFO_EMPTY			(1 << 6)
#define STAT_CRC_RES_ERR			(1 << 5)
#define STAT_CRC_READ_ERROR			(1 << 4)
#define STAT_CRC_WRITE_ERROR_SHF		2
#define STAT_CRC_WRITE_ERROR_MASK		(0x3 << STAT_CRC_WRITE_ERROR_SHF)
#define STAT_CRC_WRITE_ERROR_NO			(0 << STAT_CRC_WRITE_ERROR_SHF)
#define STAT_CRC_WRITE_ERROR			(1 << STAT_CRC_WRITE_ERROR_SHF)
#define STAT_CRC_WRITE_ERROR_NOSTS		(2 << STAT_CRC_WRITE_ERROR_SHF)
#define STAT_TIME_OUT_RES			(1 << 1)
#define STAT_TIME_OUT_READ			(1 << 0)

/* MSC Bus Clock Control Register (MSC_CLKRT) */
#define	CLKRT_CLK_RATE_SHF	0
#define	CLKRT_CLK_RATE_MASK	(0x7 << CLKRT_CLK_RATE_SHF)
#define CLKRT_CLK_RATE_DIV_1	(0x0 << CLKRT_CLK_RATE_SHF)		/* CLK_SRC */
#define CLKRT_CLK_RATE_DIV_2	(0x1 << CLKRT_CLK_RATE_SHF)		/* 1/2 of CLK_SRC */
#define CLKRT_CLK_RATE_DIV_4	(0x2 << CLKRT_CLK_RATE_SHF)		/* 1/4 of CLK_SRC */
#define CLKRT_CLK_RATE_DIV_8	(0x3 << CLKRT_CLK_RATE_SHF)		/* 1/8 of CLK_SRC */
#define CLKRT_CLK_RATE_DIV_16	(0x4 << CLKRT_CLK_RATE_SHF)		/* 1/16 of CLK_SRC */
#define CLKRT_CLK_RATE_DIV_32	(0x5 << CLKRT_CLK_RATE_SHF)		/* 1/32 of CLK_SRC */
#define CLKRT_CLK_RATE_DIV_64	(0x6 << CLKRT_CLK_RATE_SHF)		/* 1/64 of CLK_SRC */
#define CLKRT_CLK_RATE_DIV_128	(0x7 << CLKRT_CLK_RATE_SHF)		/* 1/128 of CLK_SRC */

/* MSC Command Sequence Control Register (MSC_CMDAT) */
#define	CMDAT_CCS_EXPECTED		(1 << 31)		/* interrupts are enabled in ce-ata */
#define	CMDAT_READ_CEATA		(1 << 30)
#define	CMDAT_DIS_BOOT			(1 << 27)
#define	CMDAT_ENB_BOOT			(1 << 26)
#define	CMDAT_EXP_BOOT_ACK		(1 << 25)
#define	CMDAT_BOOT_MODE			(1 << 24)
#define	CMDAT_AUTO_CMD23		(1 << 18)
#define	CMDAT_SDIO_PRDT			(1 << 17)		/* exact 2 cycle */
#define	CMDAT_AUTO_CMD12		(1 << 16)
#define	CMDAT_RTRG_SHF			14
#define CMDAT_RTRG_EQUALT_16		(0x0 << CMDAT_RTRG_SHF)	/*reset value*/
#define CMDAT_RTRG_EQUALT_32		(0x1 << CMDAT_RTRG_SHF)
#define CMDAT_RTRG_EQUALT_64		(0x2 << CMDAT_RTRG_SHF)
#define CMDAT_RTRG_EQUALT_96		(0x3 << CMDAT_RTRG_SHF)
#define	CMDAT_TTRG_SHF			12
#define CMDAT_TTRG_LESS_16		(0x0 << CMDAT_TTRG_SHF)	/*reset value*/
#define CMDAT_TTRG_LESS_32		(0x1 << CMDAT_TTRG_SHF)
#define CMDAT_TTRG_LESS_64		(0x2 << CMDAT_TTRG_SHF)
#define CMDAT_TTRG_LESS_96		(0x3 << CMDAT_TTRG_SHF)
#define	CMDAT_IO_ABORT			(1 << 11)
#define	CMDAT_BUS_WIDTH_SHF		9
#define	CMDAT_BUS_WIDTH_MASK		(0x3 << CMDAT_BUS_WIDTH_SHF)
#define CMDAT_BUS_WIDTH_1BIT		(0x0 << CMDAT_BUS_WIDTH_SHF)	/* 1-bit data bus */
#define CMDAT_BUS_WIDTH_4BIT		(0x2 << CMDAT_BUS_WIDTH_SHF)	/* 4-bit data bus */
#define CMDAT_BUS_WIDTH_8BIT		(0x3 << CMDAT_BUS_WIDTH_SHF)	/* 8-bit data bus */
#define	CMDAT_INIT			(1 << 7)
#define	CMDAT_BUSY			(1 << 6)
#define	CMDAT_STREAM_BLOCK		(1 << 5)
#define	CMDAT_WRITE_READ		(1 << 4)
#define	CMDAT_DATA_EN			(1 << 3)
#define	CMDAT_RESPONSE_SHF		0
#define	CMDAT_RESPONSE_MASK		(0x7 << CMDAT_RESPONSE_SHF)
#define CMDAT_RESPONSE_NONE 		(0x0 << CMDAT_RESPONSE_SHF)		/* No response */
#define CMDAT_RESPONSE_R1		(0x1 << CMDAT_RESPONSE_SHF)		/* Format R1 and R1b */
#define CMDAT_RESPONSE_R2	  	(0x2 << CMDAT_RESPONSE_SHF)		/* Format R2 */
#define CMDAT_RESPONSE_R3	  	(0x3 << CMDAT_RESPONSE_SHF)		/* Format R3 */
#define CMDAT_RESPONSE_R4	  	(0x4 << CMDAT_RESPONSE_SHF)		/* Format R4 */
#define CMDAT_RESPONSE_R5	  	(0x5 << CMDAT_RESPONSE_SHF)		/* Format R5 */
#define CMDAT_RESPONSE_R6	  	(0x6 << CMDAT_RESPONSE_SHF)		/* Format R6 */
#define CMDAT_RESRONSE_R7		(0x7 << CMDAT_RESPONSE_SHF)		/* Format R7 */

/* MSC Interrupts Mask Register (MSC_IMASK) */
#define	IMASK_DMA_DATA_DONE		(1 << 31)
#define	IMASK_WR_ALL_DONE		(1 << 23)
#define	IMASK_AUTO_CMD23_DONE		(1 << 30)
#define	IMASK_SVS			(1 << 29)
#define	IMASK_PIN_LEVEL_SHF		24
#define	IMASK_PIN_LEVEL_MASK		(0x1f << IMASK_PIN_LEVEL_SHF)
#define	IMASK_BCE			(1 << 20)
#define	IMASK_BDE			(1 << 19)
#define	IMASK_BAE			(1 << 18)
#define	IMASK_BAR			(1 << 17)
#define	IMASK_DMAEND			(1 << 16)
#define	IMASK_AUTO_CMD12_DONE		(1 << 15)
#define	IMASK_DATA_FIFO_FULL		(1 << 14)
#define	IMASK_DATA_FIFO_EMP		(1 << 13)
#define	IMASK_CRC_RES_ERR		(1 << 12)
#define	IMASK_CRC_READ_ERR		(1 << 11)
#define	IMASK_CRC_WRITE_ERR		(1 << 10)
#define	IMASK_TIME_OUT_RES		(1 << 9)
#define	IMASK_TIME_OUT_READ		(1 << 8)
#define	IMASK_SDIO			(1 << 7)
#define	IMASK_TXFIFO_WR_REQ		(1 << 6)
#define	IMASK_RXFIFO_RD_REQ		(1 << 5)
#define	IMASK_END_CMD_RES		(1 << 2)
#define	IMASK_PRG_DONE			(1 << 1)
#define	IMASK_DATA_TRAN_DONE		(1 << 0)

/* MSC Interrupts Status Register (MSC_IREG) */
#define	IFLG_DMA_DATA_DONE		(1 << 31)
#define	IFLG_WR_ALL_DONE		(1 << 23)
#define	IFLG_AUTO_CMD23_DONE		(1 << 30)
#define	IFLG_SVS			(1 << 29)
#define	IFLG_PIN_LEVEL_SHF		24
#define	IFLG_PIN_LEVEL_MASK		(0x1f << IFLG_PIN_LEVEL_SHF)
#define	IFLG_BCE			(1 << 20)
#define	IFLG_BDE			(1 << 19)
#define	IFLG_BAE			(1 << 18)
#define	IFLG_BAR			(1 << 17)
#define	IFLG_DMAEND			(1 << 16)
#define	IFLG_AUTO_CMD12_DONE		(1 << 15)
#define	IFLG_DATA_FIFO_FULL		(1 << 14)
#define	IFLG_DATA_FIFO_EMP		(1 << 13)
#define	IFLG_CRC_RES_ERR		(1 << 12)
#define	IFLG_CRC_READ_ERR		(1 << 11)
#define	IFLG_CRC_WRITE_ERR		(1 << 10)
#define	IFLG_TIMEOUT_RES		(1 << 9)
#define	IFLG_TIMEOUT_READ		(1 << 8)
#define	IFLG_SDIO			(1 << 7)
#define	IFLG_TXFIFO_WR_REQ		(1 << 6)
#define	IFLG_RXFIFO_RD_REQ		(1 << 5)
#define	IFLG_END_CMD_RES		(1 << 2)
#define	IFLG_PRG_DONE			(1 << 1)
#define	IFLG_DATA_TRAN_DONE		(1 << 0)

/* MSC Low Power Mode Register (MSC_LPM) */
#define	LPM_DRV_SEL_SHF			30
#define	LPM_DRV_SEL_MASK		(0x3 << LPM_DRV_SEL_SHF)
#define	LPM_SMP_SEL			(1 << 29)
#define	LPM_LPM				(1 << 0)

/* MSC DMA Control Register (MSC_DMAC) */
#define	DMAC_MODE_SEL		(1 << 7)
#define	DMAC_AOFST_SHF		5
#define	DMAC_AOFST_MASK		(0x3 << DMAC_AOFST_SHF)
#define	DMAC_AOFST_0		(0 << DMAC_AOFST_SHF)
#define	DMAC_AOFST_1		(1 << DMAC_AOFST_SHF)
#define	DMAC_AOFST_2		(2 << DMAC_AOFST_SHF)
#define	DMAC_AOFST_3		(3 << DMAC_AOFST_SHF)
#define	DMAC_ALIGNEN		(1 << 4)
#define	DMAC_INCR_SHF		2
#define	DMAC_INCR_MASK		(0x3 << DMAC_INCR_SHF)
#define	DMAC_INCR_16		(0 << DMAC_INCR_SHF)
#define	DMAC_INCR_32		(1 << DMAC_INCR_SHF)
#define	DMAC_INCR_64		(2 << DMAC_INCR_SHF)
#define	DMAC_DMASEL		(1 << 1)
#define	DMAC_DMAEN		(1 << 0)

/* MSC DMA Command Register (MSC_DMACMD) */
#define	DMACMD_IDI_SHF			24
#define	DMACMD_IDI_MASK			(0xff << DMACMD_IDI_SHF)
#define	DMACMD_ID_SHF			16
#define	DMACMD_ID_MASK			(0xff << DMACMD_ID_SHF)
#define	DMACMD_OFFSET_SHF		9
#define	DMACMD_OFFSET_MASK		(0x3 << DMACMD_OFFSET_SHF)
#define	DMACMD_ALIGN_EN			(1 << 8)
#define	DMACMD_ENDI			(1 << 1)
#define	DMACMD_LINK			(1 << 0)


enum {
	EVENT_CMD_COMPLETE = 0,
	EVENT_TRANS_COMPLETE,
	EVENT_DMA_COMPLETE,
	EVENT_DATA_COMPLETE,
	EVENT_STOP_COMPLETE,
	EVENT_ERROR,
};

enum ingenic_mmc_state {
	STATE_IDLE = 0,
	STATE_WAITING_RESP,
	STATE_WAITING_DATA,
	STATE_SENDING_STOP,
	STATE_ERROR,
};

struct sdma_desc {
	volatile u32 nda;
	volatile u32 da;
	volatile u32 len;
	volatile u32 dcmd;
};

struct desc_hd {
	struct sdma_desc *dma_desc;
	dma_addr_t dma_desc_phys_addr;
	struct desc_hd *next;
};

/**
 * struct ingenic_mmc_host - Ingenic MMC/SD Controller host structure
 * @pdata: The platform data.
 * @dev: The mmc device pointer.
 * @irq: Interrupt of MSC.
 * @clk: Main Clk of MSC, including cgu and clk gate.
 * @pwc_clk: Power of MSC module, MSC register can be access when pwc_clk on.
 * @power: Power regulator of SD/MMC attached to SD slot.
 * @mrq: mmc_request pointer which includes all the information
 *	of the current request, or NULL when the host is idle.
 * @cmd: Command information of mmc_request.
 * @data: Data information of mmc_request, or NULL when mrq without
 *	data request.
 * @mmc: The mmc_host representing this slot.
 * @pending_events: Bitmask of events flagged by the interrupt handler
 *	to be processed by the state machine.
 * @iomem: Pointer to MSC registers.
 * @detect_timer: Timer used for debouncing card insert interrupts.
 * @request_timer: Timer used for preventing request time out.
 * @flags: Random state bits associated with the slot.
 * @cmdat: Variable for MSC_CMDAT register.
 * @cmdat_def: Defalt CMDAT register value for every request.
 * @gpio: Information of gpio including cd, wp and pwr.
 * @index: Number of each MSC host.
 * @decshds[]: Descriptor DMA information structure.
 * @state: It's the state for request.
 * @list: List head for manually detect card such as wifi.
 * @lock: Lock the registers operation.
 * @double_enter: Prevent state machine reenter.
 * @timeout_cnt: The count of timeout second.
 */
#define MAX_DMA_DESCS			CONFIG_MMC_INGENIC_DMA_DESCS	/* max count of sg */
//#define INGENIC_MMC_CARD_PRESENT	0
#define INGENIC_MMC_CARD_NEED_INIT	1
#define INGENIC_MMC_USE_PIO		2
#define INGENIC_MMC_IS_SDIO		3

struct ingenic_mmc_host {
	void __iomem				*iomem;
	struct device				*dev;
	struct clk				*clk;
	struct mmc_request			*mrq;
	struct mmc_command			*cmd;
	struct mmc_data				*data;
	struct mmc_host				*mmc;
	struct timer_list			detect_timer;
	struct timer_list			request_timer;
	struct tasklet_struct			tasklet;
	struct sdma_desc			*dma_descs;
	dma_addr_t				dma_descs_phys;
	enum ingenic_mmc_state			state;
	spinlock_t				lock;
	unsigned long				pending_events;
	unsigned long				flags;
	unsigned int				cmdat;
	unsigned int				cmdat_def;
	unsigned int				index;
	unsigned int				double_enter;
	int					timeout_cnt;
	int					irq;
};

struct ingenic_mmc_priv {
	void (*get_clk_name)(int id, char *cgu_name, char *gate_name);
};

/* Register access macros */
#define msc_readl(port,reg)						\
	readl((port)->iomem + MSC_##reg)
#define msc_writel(port,reg,value)				\
	writel((value), (port)->iomem + MSC_##reg)


/**
 * MMC driver parameters
 */
#define TIMEOUT_PERIOD		3000	/* msc operation timeout detect period */
#define PIO_THRESHOLD		64	/* use pio mode if data length < PIO_THRESHOLD */
#define CLK_CTRL

#define ERROR_IFLG (							\
		IFLG_CRC_RES_ERR	|				\
		IFLG_CRC_READ_ERR	|				\
		IFLG_CRC_WRITE_ERR	|				\
		IFLG_TIMEOUT_RES	|				\
		IFLG_TIMEOUT_READ)

/*
 * Error status including CRC_READ_ERROR, CRC_WRITE_ERROR,
 * CRC_RES_ERR, TIME_OUT_RES, TIME_OUT_READ
 */
#define ERROR_STAT		0x3f

#define ingenic_mmc_check_pending(host, event)		\
	test_and_clear_bit(event, &host->pending_events)
#define ingenic_mmc_set_pending(host, event)			\
	set_bit(event, &host->pending_events)
#define is_pio_mode(host)						\
	(host->flags & (1 << INGENIC_MMC_USE_PIO))
#define enable_pio_mode(host)					\
	(host->flags |= (1 << INGENIC_MMC_USE_PIO))
#define disable_pio_mode(host)					\
	(host->flags &= ~(1 << INGENIC_MMC_USE_PIO))

static LIST_HEAD(manual_list);

/*-------------------End structure and macro define------------------------*/


static void ingenic_mmc_dump_reg(struct ingenic_mmc_host *host)
{
#ifdef CONFIG_MMC_INGENIC_DEBUG
	dev_info(host->dev,"\nREG dump:\n"
			 "\tCTRL2\t= 0x%08X\n"
			 "\tSTAT\t= 0x%08X\n"
			 "\tCLKRT\t= 0x%08X\n"
			 "\tCMDAT\t= 0x%08X\n"
			 "\tRESTO\t= 0x%08X\n"
			 "\tRDTO\t= 0x%08X\n"
			 "\tBLKLEN\t= 0x%08X\n"
			 "\tNOB\t= 0x%08X\n"
			 "\tSNOB\t= 0x%08X\n"
			 "\tIMASK\t= 0x%08X\n"
			 "\tIFLG\t= 0x%08X\n"
			 "\tCMD\t= 0x%08X\n"
			 "\tARG\t= 0x%08X\n"
			 "\tRES\t= 0x%08X\n"
			 "\tLPM\t= 0x%08X\n"
			 "\tDMAC\t= 0x%08X\n"
			 "\tDMANDA\t= 0x%08X\n"
			 "\tDMADA\t= 0x%08X\n"
			 "\tDMALEN\t= 0x%08X\n"
			 "\tDMACMD\t= 0x%08X\n"
			 "\tRTCNT\t= 0x%08X\n"
			 "\tDEBUG\t= 0x%08X\n",

			 msc_readl(host, CTRL2),
			 msc_readl(host, STAT),
			 msc_readl(host, CLKRT),
			 msc_readl(host, CMDAT),
			 msc_readl(host, RESTO),
			 msc_readl(host, RDTO),
			 msc_readl(host, BLKLEN),
			 msc_readl(host, NOB),
			 msc_readl(host, SNOB),
			 msc_readl(host, IMASK),
			 msc_readl(host, IFLG),
			 msc_readl(host, CMD),
			 msc_readl(host, ARG),
			 msc_readl(host, RES),
			 msc_readl(host, LPM),
			 msc_readl(host, DMAC),
			 msc_readl(host, DMANDA),
			 msc_readl(host, DMADA),
			 msc_readl(host, DMALEN),
			 msc_readl(host, DMACMD),
			 msc_readl(host, RTCNT),
			 msc_readl(host, DEBUG));
#endif
}


/*
 * Functional functions.
 *
 * These small function will be called frequently.
 */
static inline void enable_msc_irq(struct ingenic_mmc_host *host, unsigned long bits)
{
	unsigned long imsk;

	spin_lock_bh(&host->lock);
	imsk = msc_readl(host, IMASK);
	imsk &= ~bits;
	msc_writel(host, IMASK, imsk);
	spin_unlock_bh(&host->lock);
}

static inline void clear_msc_irq(struct ingenic_mmc_host *host, unsigned long bits)
{
	msc_writel(host, IFLG, bits);
}

static inline void disable_msc_irq(struct ingenic_mmc_host *host, unsigned long bits)
{
	unsigned long imsk;

	spin_lock_bh(&host->lock);
	imsk = msc_readl(host, IMASK);
	imsk |= bits;
	msc_writel(host, IMASK, imsk);
	spin_unlock_bh(&host->lock);
}

static void ingenic_mmc_reset(struct ingenic_mmc_host *host)
{
	unsigned int clkrt = msc_readl(host, CLKRT);
	unsigned int cnt = 1000;
	int vl;

	msc_writel(host, CTRL, CTRL_RESET);
	vl = msc_readl(host,CTRL);
	vl &= ~CTRL_RESET;
	msc_writel(host, CTRL, vl);

	while ((msc_readl(host, STAT) & STAT_IS_RESETTING) && (--cnt));

	if (host->flags & INGENIC_MMC_IS_SDIO)
		msc_writel(host, CTRL, CTRL_CLOCK_START);
	else
		msc_writel(host, LPM, LPM_LPM);

	msc_writel(host, IMASK, 0xffffffff);
	msc_writel(host, IFLG, 0xffffffff);

	msc_writel(host, CLKRT, clkrt);
}

static inline void ingenic_mmc_stop_dma(struct ingenic_mmc_host *host)
{
	dev_warn(host->dev, "%s\n", __func__);

	/*
	 * Theoretically, DMA can't be stopped when transfering, so we can only
	 * diable it when it is out of DMA request.
	 */
	msc_writel(host, DMAC, 0);
}

static inline int request_need_stop(struct mmc_request *mrq)
{
	return mrq->stop ? 1 : 0;
}

static inline void ingenic_mmc_clk_onoff(struct ingenic_mmc_host *host, unsigned int on)
{
	if (on) {
		clk_prepare_enable(host->clk);
	} else {
		clk_disable_unprepare(host->clk);
	}
}

static inline int check_error_status(struct ingenic_mmc_host *host, unsigned int status)
{
	if (status & ERROR_STAT) {
		dev_err(host->dev, "Error status->0x%08X: cmd=%d, state=%d\n",
				status, host->cmd->opcode, host->state);
		return -1;
	}
	return 0;
}

static int ingenic_mmc_polling_status(struct ingenic_mmc_host *host, unsigned int status)
{
	unsigned int cnt = 1000 * 1000;
	while(!(msc_readl(host, STAT) & (status | ERROR_STAT))				\
	      && (--cnt));

	if (unlikely(!cnt)) {
		dev_err(host->dev, "polling status(0x%08X) time out, "
				"op=%d, status=0x%08X\n", status,
				host->cmd->opcode, msc_readl(host, STAT));
		return -1;
	}

	if (msc_readl(host, STAT) & ERROR_STAT) {
		dev_err(host->dev, "polling status(0x%08X) error, "
				"op=%d, status=0x%08X\n", status,
				host->cmd->opcode, msc_readl(host, STAT));
		return -1;
	}

	return 0;
}

static void send_stop_command(struct ingenic_mmc_host *host)
{
	struct mmc_command *stop_cmd = host->mrq->stop;

	msc_writel(host, CMD, stop_cmd->opcode);
	msc_writel(host, ARG, stop_cmd->arg);
	msc_writel(host, CMDAT, CMDAT_BUSY | CMDAT_RESPONSE_R1);
	msc_writel(host, RESTO, 0xff);
	msc_writel(host, CTRL, CTRL_START_OP);

	if (ingenic_mmc_polling_status(host, STAT_END_CMD_RES))
		stop_cmd->error = -EIO;
}
static void ingenic_mmc_command_done(struct ingenic_mmc_host *host, struct mmc_command *cmd)
{
	unsigned long res;

	if ((host->cmdat & CMDAT_RESPONSE_MASK) == CMDAT_RESPONSE_R2) {
		int i;
		res = msc_readl(host, RES);
		for (i = 0 ; i < 4 ; i++) {
			cmd->resp[i] = res << 24;
			res = msc_readl(host, RES);
			cmd->resp[i] |= res << 8;
			res = msc_readl(host, RES);
			cmd->resp[i] |= res >> 8;
		}
	} else {
		res = msc_readl(host, RES);
		cmd->resp[0] = res << 24;
		res = msc_readl(host, RES);
		cmd->resp[0] |= res << 8;
		res = msc_readl(host, RES);
		cmd->resp[0] |= res & 0xff;
	}

	clear_msc_irq(host, IFLG_END_CMD_RES);
}

static void ingenic_mmc_data_done(struct ingenic_mmc_host *host)
{
	struct mmc_data *data = host->data;

	if (data->error == 0)
		data->bytes_xfered = (data->blocks * data->blksz);
	else {
		ingenic_mmc_stop_dma(host);
		data->bytes_xfered = 0;
		dev_err(host->dev, "error when request done\n");
	}

	del_timer_sync(&host->request_timer);
	mmc_request_done(host->mmc, host->mrq);
}

/*------------------------End functional functions-------------------------*/

/*
 * State machine.
 *
 * The state machine is the manager of the mmc_request. It's triggered by
 * MSC interrupt and work in interrupt context.
 */
static void ingenic_mmc_state_machine(struct ingenic_mmc_host *host, unsigned int status)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_data *data = host->data;

	WARN_ON(host->double_enter++);
start:
	dev_dbg(host->dev, "enter state: %d\n", host->state);

	switch (host->state) {
	case STATE_IDLE:
		dev_warn(host->dev, "WARN: enter state machine with IDLE\n");
		break;

	case STATE_WAITING_RESP:
		if (!ingenic_mmc_check_pending(host, EVENT_CMD_COMPLETE))
			break;
		if (unlikely(check_error_status(host, status) != 0)) {
			host->state = STATE_ERROR;
			clear_msc_irq(host, IFLG_CRC_RES_ERR
						  | IFLG_TIMEOUT_RES
						  | IFLG_END_CMD_RES);
			goto start;
		}
		ingenic_mmc_command_done(host, mrq->cmd);
		if (!data) {
			host->state = STATE_IDLE;
			del_timer_sync(&host->request_timer);
			mmc_request_done(host->mmc, host->mrq);
			break;
		}
		host->state = STATE_WAITING_DATA;
		break;

	case STATE_WAITING_DATA:
		if (!ingenic_mmc_check_pending(host, EVENT_DATA_COMPLETE))
			break;
		if (unlikely(check_error_status(host, status) != 0)) {
			clear_msc_irq(host, IFLG_DATA_TRAN_DONE
						  | IFLG_CRC_READ_ERR
						  | IFLG_CRC_WRITE_ERR
						  | IFLG_TIMEOUT_READ);
			if (request_need_stop(host->mrq))
				send_stop_command(host);
			host->state = STATE_ERROR;
			goto start;
		}

		if (request_need_stop(host->mrq)) {
			if (likely(msc_readl(host, STAT) & STAT_AUTO_CMD12_DONE)) {
				disable_msc_irq(host, IMASK_AUTO_CMD12_DONE);
				clear_msc_irq(host, IFLG_AUTO_CMD12_DONE);
				host->state = STATE_IDLE;
				ingenic_mmc_data_done(host);
			} else {
				enable_msc_irq(host, IMASK_AUTO_CMD12_DONE);
				if (msc_readl(host, STAT) & STAT_AUTO_CMD12_DONE) {
					disable_msc_irq(host, IMASK_AUTO_CMD12_DONE);
					clear_msc_irq(host, IFLG_AUTO_CMD12_DONE);
					host->state = STATE_IDLE;
					ingenic_mmc_data_done(host);
				} else
					host->state = STATE_SENDING_STOP;
			}
		} else {
			host->state = STATE_IDLE;
			ingenic_mmc_data_done(host);
		}
		break;

	case STATE_SENDING_STOP:
		if (!ingenic_mmc_check_pending(host, EVENT_STOP_COMPLETE))
			break;
		host->state = STATE_IDLE;
		ingenic_mmc_data_done(host);
		break;

	case STATE_ERROR:
		if (host->state == STATE_WAITING_DATA)
			host->data->error = -1;
		host->cmd->error = -1;

		if (data) {
			data->bytes_xfered = 0;
			/* Whether should we stop DMA here? */
		}
		del_timer_sync(&host->request_timer);
		host->state = STATE_IDLE;
		mmc_request_done(host->mmc, host->mrq);
		break;
	}

	dev_dbg(host->dev, "exit state: %d\n", host->state);
	host->double_enter--;
}

static irqreturn_t ingenic_mmc_thread_handle(int irq, void *dev_id)
{
	struct ingenic_mmc_host *host = (struct ingenic_mmc_host *)dev_id;
	unsigned int iflg, imask, pending, status;

start:
	iflg = msc_readl(host, IFLG);
	imask = msc_readl(host, IMASK);
	pending = iflg & ~imask;
	status = msc_readl(host, STAT);
	dev_dbg(host->dev, "%s: iflg-0x%08X imask-0x%08X status-0x%08X\n",
			__func__, iflg, imask, status);

	if (!pending) {
		goto out;
	} else if (pending & IFLG_SDIO) {
		mmc_signal_sdio_irq(host->mmc);
		goto out;
	} else if (pending & ERROR_IFLG) {
		unsigned int mask = ERROR_IFLG;

		dev_dbg(host->dev, "%s: iflg-0x%08X imask-0x%08X status-0x%08X\n",
				__func__, iflg, imask, status);

		dev_dbg(host->dev, "err%d cmd%d iflg%08X status%08X\n",
				host->state, host->cmd ? host->cmd->opcode : -1, iflg, status);

		if (host->state == STATE_WAITING_RESP)
			mask |= IMASK_END_CMD_RES;
		else if (host->state == STATE_WAITING_DATA)
			mask |= IMASK_WR_ALL_DONE | IMASK_DMA_DATA_DONE;

		clear_msc_irq(host, mask);
		disable_msc_irq(host, mask);

		/*
		 * It seems that cmd53 CRC error occurs frequently
		 * at 50mHz clk, but it disappear at 40mHz. In case of
		 * it happens, we add retry here to try to fix the error.
		 */
		if ((host->cmd->opcode == 53)
		    && (status & STAT_CRC_READ_ERROR)) {
			dev_err(host->dev, "cmd53 crc error, retry.\n");
			host->cmd->error = -1;
			host->cmd->retries = 1;
			host->data->bytes_xfered = 0;
			del_timer_sync(&host->request_timer);
			host->state = STATE_IDLE;
			mmc_request_done(host->mmc, host->mrq);
			goto out;
		}
		host->state = STATE_ERROR;
		ingenic_mmc_state_machine(host, status);
		goto out;

	} else if (pending & IFLG_END_CMD_RES) {
		ingenic_mmc_set_pending(host, EVENT_CMD_COMPLETE);
		disable_msc_irq(host, IMASK_END_CMD_RES |		\
						IMASK_CRC_RES_ERR | IMASK_TIME_OUT_RES);
		ingenic_mmc_state_machine(host, status);
	} else if (pending & IFLG_WR_ALL_DONE) {
		ingenic_mmc_set_pending(host, EVENT_DATA_COMPLETE);
		clear_msc_irq(host, IFLG_WR_ALL_DONE
					  | IFLG_DMAEND
					  | IFLG_DATA_TRAN_DONE
					  | IFLG_PRG_DONE);
		disable_msc_irq(host, IMASK_WR_ALL_DONE | IMASK_CRC_WRITE_ERR);
		ingenic_mmc_state_machine(host, status);

	} else if (pending & IFLG_DMA_DATA_DONE) {
		ingenic_mmc_set_pending(host, EVENT_DATA_COMPLETE);
		clear_msc_irq(host, IFLG_DATA_TRAN_DONE | IFLG_DMAEND |
					  IFLG_DMA_DATA_DONE);
		disable_msc_irq(host, IMASK_DMA_DATA_DONE | IMASK_CRC_READ_ERR);
		ingenic_mmc_state_machine(host, status);
	} else if (pending & IFLG_AUTO_CMD12_DONE) {
		ingenic_mmc_set_pending(host, EVENT_STOP_COMPLETE);
		clear_msc_irq(host, IFLG_AUTO_CMD12_DONE);
		disable_msc_irq(host, IMASK_AUTO_CMD12_DONE);
		ingenic_mmc_state_machine(host, status);

	} else
		dev_warn(host->dev, "state-%d: Nothing happens?!\n", host->state);

	/*
	 * Check if the status has already changed. If so, goto start so that
	 * we can avoid an interrupt.
	 */
	if (status != msc_readl(host, STAT)) {
		goto start;
	}

out:
	return IRQ_HANDLED;
}
/*--------------------------End state machine------------------------------*/

/*
 * DMA handler.
 *
 * Descriptor DMA transfer that can handle scatter gather list directly
 * without bounce buffer which may cause a big deal of memcpy.
 */
static inline void sg_to_desc(struct scatterlist *sgentry, struct sdma_desc *sd)
{
	sd->da = sg_phys(sgentry);
	sd->len = sg_dma_len(sgentry);
	sd->dcmd = DMACMD_LINK;
}

static void ingenic_mmc_submit_dma(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	unsigned i = 0;
	struct scatterlist *sgentry;
	struct sdma_desc *sd = host->dma_descs;

	if (data->sg_len > MAX_DMA_DESCS) {
		dev_err(host->dev, "logic error: sg_len too large\n");
		BUG();
	}

	dma_map_sg(host->dev, data->sg, data->sg_len,
			   data->flags & MMC_DATA_WRITE
			   ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

	for_each_sg(data->sg, sgentry, data->sg_len, i) {
		sg_to_desc(sgentry, sd);
		sd++;
	}

	dma_unmap_sg(host->dev, data->sg, data->sg_len,
				 data->flags & MMC_DATA_WRITE
				 ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

	if (data->sg_len) {
		sd[-1].dcmd |= DMACMD_ENDI;
		sd[-1].dcmd &= ~DMACMD_LINK;
	}
}

static inline unsigned int get_incr(unsigned int dma_len)
{
	return 2;
}

static inline void ingenic_mmc_dma_start(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	dma_addr_t dma_addr = sg_phys(data->sg);
	unsigned int dma_len = sg_dma_len(data->sg);
	unsigned int dmac;

	dmac = (get_incr(dma_len) << DMAC_INCR_SHF) | DMAC_DMAEN;

	if ((dma_addr & 0x3) || (dma_len & 0x3)) {
		dmac |= DMAC_ALIGNEN;
		if (dma_addr & 0x3)
			dmac |= (dma_addr % 4) << DMAC_AOFST_SHF;
	}

	msc_writel(host, DMANDA, host->dma_descs_phys);
	msc_writel(host, DMAC, dmac);
}

/*----------------------------End DMA handler------------------------------*/

/*
 * PIO transfer mode.
 *
 * Functions of PIO read/write mode that can handle 1, 2 or 3 bytes transfer
 * even though the FIFO register is 32-bits width.
 * It's better just used for test.
 */
static int wait_cmd_response(struct ingenic_mmc_host *host)
{
	if (ingenic_mmc_polling_status(host, STAT_END_CMD_RES) < 0) {
		dev_err(host->dev, "PIO mode: command response error\n");
		return -1;
	}
	msc_writel(host, IFLG, IFLG_END_CMD_RES);
	return 0;
}

static void do_pio_read(struct ingenic_mmc_host *host,
						unsigned int *addr, unsigned int cnt)
{
	int i = 0;
	unsigned int status = 0;

	for (i = 0; i < cnt / 4; i++) {
		while ((status = msc_readl(host, STAT)) & STAT_DATA_FIFO_EMPTY);

		if (check_error_status(host, status)) {
			host->data->error = -1;
			return;
		}
		*addr++ = msc_readl(host, RXFIFO);
	}

	/*
	 * These codes handle the last 1, 2 or 3 bytes transfer.
	 */
	if (cnt & 3) {
		u32 n = cnt & 3;
		u32 data = msc_readl(host, RXFIFO);
		u8 *p = (u8 *)addr;

		while (n--) {
			*p++ = data;
			data >>= 8;
		}
	}
}

static void do_pio_write(struct ingenic_mmc_host *host,
						 unsigned int *addr, unsigned int cnt)
{
	int i = 0;
	unsigned int status = 0;

	for (i = 0; i < (cnt / 4); i++) {
		while ((status = msc_readl(host, STAT)) & STAT_DATA_FIFO_FULL);

		if (check_error_status(host, status)) {
			host->data->error = -1;
			return;
		}

		msc_writel(host, TXFIFO, *addr++);
	}

	/*
	 * These codes handle the last 1, 2 or 3 bytes transfer.
	 */
	if (cnt & 3) {
		u32 data = 0;
		u8 *p = (u8 *)addr;

		for (i = 0; i < (cnt & 3); i++)
			data |= *p++ << (8 * i);

		msc_writel(host, TXFIFO, data);
	}
}

static inline void pio_trans_start(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	unsigned int *addr = sg_virt(data->sg);
	unsigned int cnt = sg_dma_len(data->sg);

	if (data->flags & MMC_DATA_WRITE)
		do_pio_write(host, addr, cnt);
	else
		do_pio_read(host, addr, cnt);
}

static void pio_trans_done(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	if (data->error == 0)
		data->bytes_xfered = data->blocks * data->blksz;
	else
		data->bytes_xfered = 0;

	if (host->mrq->stop) {
		if (ingenic_mmc_polling_status(host, STAT_AUTO_CMD12_DONE) < 0)
			data->error = -EIO;
	}

	if (data->flags & MMC_DATA_WRITE) {
		if (ingenic_mmc_polling_status(host, STAT_PRG_DONE) < 0) {
			data->error = -EIO;
		}
		clear_msc_irq(host, IFLG_PRG_DONE);
	} else {
		if (ingenic_mmc_polling_status(host, STAT_DATA_TRAN_DONE) < 0) {
			data->error = -EIO;
		}
		clear_msc_irq(host, IFLG_DATA_TRAN_DONE);
	}
}

/*-------------------------End PIO transfer mode---------------------------*/

/*
 * Achieve mmc_request here.
 */
static void ingenic_mmc_data_pre(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	unsigned int nob = data->blocks;
	unsigned long cmdat,imsk;

	msc_writel(host, RDTO, 0xffffff);
	msc_writel(host, NOB, nob);
	msc_writel(host, BLKLEN, data->blksz);
	cmdat = CMDAT_DATA_EN;

	msc_writel(host, CMDAT, CMDAT_DATA_EN);


	if (data->flags & MMC_DATA_WRITE) {
		cmdat |= CMDAT_WRITE_READ;
		imsk = IMASK_WR_ALL_DONE | IMASK_CRC_WRITE_ERR;
	} else if (data->flags & MMC_DATA_READ) {
		cmdat &= ~CMDAT_WRITE_READ;
		imsk = IMASK_DMA_DATA_DONE
			| IMASK_TIME_OUT_READ
			| IMASK_CRC_READ_ERR;
	} else {
		dev_err(host->dev, "data direction confused\n");
		BUG_ON(1);
	}
	host->cmdat |= cmdat;

	if (!is_pio_mode(host)) {
		ingenic_mmc_submit_dma(host, data);
		clear_msc_irq(host, IFLG_PRG_DONE);
		enable_msc_irq(host, imsk);
	}
}

static void ingenic_mmc_data_start(struct ingenic_mmc_host *host, struct mmc_data *data)
{
	if (is_pio_mode(host)) {
		pio_trans_start(host, data);
		pio_trans_done(host, data);
		del_timer_sync(&host->request_timer);
		disable_pio_mode(host);
		mmc_request_done(host->mmc, host->mrq);
	} else {
		ingenic_mmc_dma_start(host, data);
	}
}

static void ingenic_mmc_command_start(struct ingenic_mmc_host *host, struct mmc_command *cmd)
{
	unsigned long cmdat = 0;
	unsigned long imsk;

	if (cmd->flags & MMC_RSP_BUSY)
		cmdat |= CMDAT_BUSY;
	if (request_need_stop(host->mrq))
		cmdat |= CMDAT_AUTO_CMD12;


	switch (mmc_resp_type(cmd)) {
#define _CASE(S,D) case MMC_RSP_##S: cmdat |= CMDAT_RESPONSE_##D; break
		_CASE(R1, R1); 	/* r1 = r5,r6,r7 */
		_CASE(R1B, R1);
		_CASE(R2, R2);
		_CASE(R3, R3); 	/* r3 = r4 */
	default:
		break;
#undef _CASE
	}
	host->cmdat |= cmdat;
	if (!is_pio_mode(host)) {
		imsk = IMASK_TIME_OUT_RES | IMASK_END_CMD_RES;
		enable_msc_irq(host, imsk);
		host->state = STATE_WAITING_RESP;
	}
	msc_writel(host, CMD, cmd->opcode);
	msc_writel(host, ARG, cmd->arg);
	msc_writel(host, CMDAT, host->cmdat);
	msc_writel(host, CTRL, CTRL_START_OP);
	if (is_pio_mode(host)) {
		if (wait_cmd_response(host) < 0) {
			cmd->error = -ETIMEDOUT;
			del_timer_sync(&host->request_timer);
			mmc_request_done(host->mmc, host->mrq);
			return;
		}
		ingenic_mmc_command_done(host, host->cmd);
		if (!host->data) {
			del_timer_sync(&host->request_timer);
			mmc_request_done(host->mmc, host->mrq);
		}
	}
}

static void ingenic_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct ingenic_mmc_host *host = mmc_priv(mmc);

	/*
	 * It means that this request may flush cache in interrupt context.
	 * It never happens in design, but we add BUG_ON here to prevent it.
	 */
	if ((host->state != STATE_IDLE) && (mrq->data != NULL)) {
		dev_warn(host->dev, "operate in non-idle state\n");
		WARN_ON(1);
	}

	host->mrq = mrq;
	host->data = mrq->data;
	host->cmd = mrq->cmd;
	if (host->data)
		dev_dbg(host->dev, "op:%d arg:0x%08X sz:%uk\n",
				host->cmd->opcode, host->cmd->arg,
				host->data->blocks >> 1);
	else
		dev_dbg(host->dev, "op:%d\n", host->cmd->opcode);

	host->cmdat = host->cmdat_def;
	if(host->data) {
		if ((host->data->sg_len == 1)
		    && (sg_dma_len(host->data->sg)) < PIO_THRESHOLD) {
			enable_pio_mode(host);
		}

		ingenic_mmc_data_pre(host, host->data);
	}
	/*
	 * We would get mmc_request_done at last, unless some terrible error
	 * occurs such as intensity rebounding of VDD, that maybe result in
	 * no action to complete the request.
	 */
	host->timeout_cnt = 0;
	mod_timer(&host->request_timer, jiffies +
			  msecs_to_jiffies(TIMEOUT_PERIOD));
	ingenic_mmc_command_start(host, host->cmd);
	if (host->data) {
		ingenic_mmc_data_start(host, host->data);

	}
	if (unlikely(test_and_clear_bit(INGENIC_MMC_CARD_NEED_INIT, &host->flags)))
		host->cmdat_def &= ~CMDAT_INIT;
}

static void ingenic_mmc_request_timeout(struct timer_list *t)
{
	struct ingenic_mmc_host *host = from_timer(host, t, request_timer);
	unsigned int status = msc_readl(host, STAT);
	if (host->timeout_cnt++ < (3000 / TIMEOUT_PERIOD)) {
		dev_warn(host->dev, "timeout %dms op:%d %s sz:%d state:%d "
				 "STAT:0x%08X DMALEN:0x%08X blks:%d/%d clk:%s\n",
				 host->timeout_cnt * TIMEOUT_PERIOD,
				 host->cmd->opcode,
				 host->data
				 ? (host->data->flags & MMC_DATA_WRITE ? "w" : "r")
				 : "",
				 host->data ? host->data->blocks << 9 : 0,
				 host->state,
				 status,
				 msc_readl(host, DMALEN),
				 msc_readl(host, SNOB),
				 msc_readl(host, NOB),
				 __clk_is_enabled(host->clk) ? "enable" : "disable");
		mod_timer(&host->request_timer, jiffies +
				  msecs_to_jiffies(TIMEOUT_PERIOD));
		return;

	} else if (host->timeout_cnt++ < (60000 / TIMEOUT_PERIOD)) {
		mod_timer(&host->request_timer, jiffies +
				  msecs_to_jiffies(TIMEOUT_PERIOD));
		return;
	}

	dev_err(host->dev, "request time out, op=%d arg=0x%08X, "
			"sz:%dB state=%d, status=0x%08X, pending=0x%08X, nr_desc=%d\n",
			host->cmd->opcode, host->cmd->arg,
			host->data ? host->data->blocks << 9 : -1,
			host->state, status, (u32)host->pending_events,
			host->data ? host->data->sg_len : 0);
	ingenic_mmc_dump_reg(host);

#ifdef CONFIG_MMC_INGENIC_DEBUG
	if (host->data) {
		int i;
		dev_err(host->dev, "Descriptor dump:\n");
		for (i = 0; i < MAX_DMA_DESCS; i++) {
			unsigned int *desc = (unsigned int *)&host->dma_descs[i];
			dev_err(host->dev, "\t%03d\t nda=%08X da=%08X len=%08X dcmd=%08X\n",
					i, *desc, *(desc+1), *(desc+2), *(desc+3));
		}
		dev_err(host->dev, "\n");
	}
#endif

	if (host->mrq) {
		if (request_need_stop(host->mrq)) {
			send_stop_command(host);
		}
		host->cmd->error = -ENOMEDIUM;
		host->state = STATE_IDLE;
		mmc_request_done(host->mmc, host->mrq);
	}
}

/*---------------------------End mmc_request-------------------------------*/

/*
 * Other mmc_ops except request.
 */

static void ingenic_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ingenic_mmc_host *host = mmc_priv(mmc);

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		host->cmdat_def &= ~CMDAT_BUS_WIDTH_MASK;
		host->cmdat_def |= CMDAT_BUS_WIDTH_1BIT;
		break;
	case MMC_BUS_WIDTH_4:
		host->cmdat_def &= ~CMDAT_BUS_WIDTH_MASK;
		host->cmdat_def |= CMDAT_BUS_WIDTH_4BIT;
		break;
	case MMC_BUS_WIDTH_8:
		host->cmdat_def &= ~CMDAT_BUS_WIDTH_MASK;
		host->cmdat_def |= CMDAT_BUS_WIDTH_8BIT;
		break;
	}

	if (ios->clock) {
		unsigned int clk_set = 0, clkrt = 0;
		unsigned int clk_want = ios->clock;
		unsigned int lpm = 0;

		clk_set_rate(host->clk, ios->clock);

		clk_set = clk_get_rate(host->clk);

		while (clk_want < clk_set) {
			clkrt++;
			clk_set >>= 1;
		}

		/* discard this warning on board 4785 fpga */
		if ((clk_want > 3000000) && clkrt) {
			dev_err(host->dev, "CLKRT must be set to 0 "
					"when MSC works during normal r/w: "
					"ios->clock=%d clk_want=%d "
					"clk_set=%d clkrt=%X,\n",
					ios->clock, clk_want, clk_set, clkrt);
			WARN_ON(1);
		}

		if (clkrt > 7) {
			dev_err(host->dev, "invalid value of CLKRT: "
					"ios->clock=%d clk_want=%d "
					"clk_set=%d clkrt=%X,\n",
					ios->clock, clk_want, clk_set, clkrt);
			WARN_ON(1);
			return;
		}
		if (!clkrt)
			dev_dbg(host->dev, "clk_want: %u, clk_set: %luHz\n",
					 ios->clock, clk_get_rate(host->clk));

		msc_writel(host, CLKRT, clkrt);

		/* sample immediately at clk rising edge */
		if (clk_set > 25000000)
			lpm = (0x2 << LPM_DRV_SEL_SHF);

		if (host->flags & INGENIC_MMC_IS_SDIO) {
			msc_writel(host, LPM, lpm);
			msc_writel(host, CTRL, CTRL_CLOCK_START);
		} else {
			lpm |= LPM_LPM;
			msc_writel(host, LPM, lpm);
		}
	}

	switch (ios->power_mode) {
	case MMC_POWER_UP:
		ingenic_mmc_reset(host);
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd);
		host->cmdat_def |= CMDAT_INIT;
		set_bit(INGENIC_MMC_CARD_NEED_INIT, &host->flags);
		clk_prepare_enable(host->clk);
		break;
	case MMC_POWER_ON:
		break;
	default:
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);
		clk_disable_unprepare(host->clk);
		break;
	}
}

static void ingenic_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct ingenic_mmc_host *host = mmc_priv(mmc);

	if (enable) {
		enable_msc_irq(host, IMASK_SDIO);
	} else {
		clear_msc_irq(host, IFLG_SDIO);
		disable_msc_irq(host, IMASK_SDIO);
	}
}

static void ingenic_mmc_init_card(struct mmc_host *mmc, struct mmc_card *card) {
	struct ingenic_mmc_host *host = mmc_priv(mmc);

#ifdef CONFIG_MMC_INGENIC_DEBUG
	dev_info(host->dev, "card type is %d\n", card->type);
#endif

	if (card->type == MMC_TYPE_SDIO || card->type == MMC_TYPE_SD_COMBO) {
		host->flags |= INGENIC_MMC_IS_SDIO;
	} else {
		host->flags &= ~INGENIC_MMC_IS_SDIO;
	}
}

static const struct mmc_host_ops ingenic_mmc_ops = {
	.request		= ingenic_mmc_request,
	.set_ios		= ingenic_mmc_set_ios,
	.get_ro			= mmc_gpio_get_ro,
	.get_cd			= mmc_gpio_get_cd,
	.enable_sdio_irq	= ingenic_mmc_enable_sdio_irq,
	.init_card		= ingenic_mmc_init_card,
};

/*
 * Platform driver and initialization.
 */

static void ingenic_mmc_dma_desc_init(struct ingenic_mmc_host *host)
{
	for (unsigned i = 0; i < MAX_DMA_DESCS; i++) {
		struct sdma_desc *sd = &host->dma_descs[i];
		sd->nda = host->dma_descs_phys + sizeof(struct sdma_desc) * (i + 1);
	}
}

static const struct of_device_id mmc_ingenic_of_match[] = {
	{.compatible = "ingenic,x1000-mmc", },
	{.compatible = "ingenic,x1600-mmc", },
	{.compatible = "ingenic,x1800-mmc", },
	{},
};
MODULE_DEVICE_TABLE(of, mmc_ingenic_of_match);

static int mmc_ingenic_probe(struct platform_device *pdev)
{
	struct ingenic_mmc_host *host;
	struct mmc_host *mmc;
	struct resource *regs;
	int ret = 0;

	mmc = mmc_alloc_host(sizeof(struct ingenic_mmc_host), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->dev = &pdev->dev;

	ret = mmc_of_parse(mmc);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "could not parse device properties\n");
		goto err_free_host;
	}

	mmc_regulator_get_supply(mmc);

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		ret = host->irq;
		goto err_free_host;
	}

	host->clk = devm_clk_get(&pdev->dev, "mmc");
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		dev_err(&pdev->dev, "Failed to get mmc clock\n");
		goto err_free_host;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->iomem = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(host->iomem)) {
		ret = PTR_ERR(host->iomem);
		goto err_free_host;
	}

	mmc->ops = &ingenic_mmc_ops;
	mmc->f_min = 200000;
	if (!mmc->f_max)
		mmc->f_max = 24000000;

	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	mmc->max_busy_timeout = TIMEOUT_PERIOD;

	mmc->max_segs = MAX_DMA_DESCS;
	mmc->max_blk_size = 512;
	mmc->max_blk_count = 4096;
	mmc->max_req_size = 4096 * 512;
	mmc->max_seg_size = mmc->max_req_size;

	host->mmc = mmc;
	spin_lock_init(&host->lock);
	timer_setup(&host->request_timer, ingenic_mmc_request_timeout, 0);

	host->cmdat_def = CMDAT_RTRG_EQUALT_16 | CMDAT_TTRG_LESS_16 |
						CMDAT_BUS_WIDTH_1BIT;

	ret = devm_request_threaded_irq(host->dev, host->irq, NULL,
			ingenic_mmc_thread_handle, IRQF_ONESHOT, dev_name(host->dev), host);

	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq: %d\n", ret);
		goto err_free_host;
	}

	platform_set_drvdata(pdev, host);
	ret = mmc_add_host(mmc);

	if (ret) {
		dev_err(&pdev->dev, "Failed to add mmc host: %d\n", ret);
		goto err_free_irq;
	}

	host->dma_descs = dmam_alloc_coherent(&pdev->dev, sizeof(struct sdma_desc) * MAX_DMA_DESCS,
						&host->dma_descs_phys,
						GFP_KERNEL);
	if (!host->dma_descs) {
		dev_err(&pdev->dev, "Failed to allocate DMA descriptors\n");
		ret = -ENOMEM;
		goto err_free_irq;
	}

	ingenic_mmc_dma_desc_init(host);

	dev_info(&pdev->dev, "Ingenic SD/MMC card driver registered\n");

	dev_info(&pdev->dev, "Using %s, %d-bit mode\n",
		 test_bit(INGENIC_MMC_USE_PIO, &host->flags) ? "PIO" : "DMA",
		 (mmc->caps & MMC_CAP_8_BIT_DATA) ? 8 :
		 ((mmc->caps & MMC_CAP_4_BIT_DATA) ? 4 : 1));

	return 0;

err_free_irq:
	free_irq(host->irq, host);
err_free_host:
	mmc_free_host(mmc);
	clk_disable_unprepare(host->clk);

	return ret;
}

static int __exit mmc_ingenic_remove(struct platform_device *pdev)
{
	struct ingenic_mmc_host *host = platform_get_drvdata(pdev);

	mmc_remove_host(host->mmc);
	mmc_free_host(host->mmc);

	free_irq(host->irq, host);

	dmam_free_coherent(&pdev->dev, sizeof(struct sdma_desc) * MAX_DMA_DESCS, host->dma_descs, host->dma_descs_phys);

	clk_put(host->clk);
	iounmap(host->iomem);
	kfree(host);

	return 0;
}

static int mmc_ingenic_suspend(struct device *dev)
{
	return pinctrl_pm_select_sleep_state(dev);
}

static int mmc_ingenic_resume(struct device *dev)
{
	return pinctrl_select_default_state(dev);
}

static DEFINE_SIMPLE_DEV_PM_OPS(mmc_ingenic_pm_ops, mmc_ingenic_suspend,
						 mmc_ingenic_resume);

static struct platform_driver mmc_ingenic_driver = {
	.driver	= {
		.name = "ingenic-mmc",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.owner = THIS_MODULE,
		.pm = &mmc_ingenic_pm_ops,
		.of_match_table = of_match_ptr(mmc_ingenic_of_match),
	},
	.probe = mmc_ingenic_probe,
	.remove = mmc_ingenic_remove,
};

module_platform_driver(mmc_ingenic_driver);

MODULE_DESCRIPTION("Ingenic X series Multimedia Card Interface driver");
MODULE_AUTHOR("Large Dipper <ykli@ingenic.com>");
MODULE_AUTHOR("bo.liu <bo.liu@ingenic.cn>");
MODULE_AUTHOR("qipengzhen <aric.pzqi@ingenic.com>");
MODULE_AUTHOR("Reimu NotMoe <reimu@sudomaker.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("20230524");
