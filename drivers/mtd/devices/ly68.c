// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Lyontek LY68 series PSRAM chips
 *
 * Copyright (C) 2023 Reimu NotMoe <reimu@sudomaker.com>
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/sizes.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>

struct ly68_caps {
	unsigned int size;
};

struct ly68_flash {
	struct spi_device	*spi;
	struct mutex		lock;
	struct mtd_info		mtd;
	unsigned		op_sleep_ratio;
	u64			ts_last_op, ts_sleep_till;
	const struct ly68_caps	*caps;
};

#define LY68_PAGE_SIZE			SZ_1K
#define LY68_CMD_WRITE			0x02
#define LY68_CMD_READ			0x0b
#define LY68_WHOLE_CHIP_REFRESH_TIME	65000000

#define to_ly68_flash(x) container_of(x, struct ly68_flash, mtd)

inline void ly68_dynamic_sleep(u32 nsecs)
{
	if (nsecs > LY68_WHOLE_CHIP_REFRESH_TIME)
		nsecs = LY68_WHOLE_CHIP_REFRESH_TIME;

	if (nsecs < 10 * 1000)
		ndelay(nsecs);
	else
		usleep_range(nsecs / 1000, nsecs / 1000 + 1);
}

static void ly68_addr2cmd(struct ly68_flash *flash, u32 addr, u8 *cmd)
{
	cmd[0] = (addr >> 16) & 0xff;
	cmd[1] = (addr >> 8) & 0xff;
	cmd[2] = (addr >> 0) & 0xff;
}

static int ly68_write(struct mtd_info *mtd, loff_t to, size_t len,
			    size_t *retlen, const unsigned char *buf)
{
	struct ly68_flash *flash = to_ly68_flash(mtd);
	struct spi_transfer transfer[2] = {};
	struct spi_message message;
	u8 command[4];
	u64 ts_op_start, ts_op_end, ts_cur;
	u32 t_op, t_sleep;
	int ret, cmd_len;

	spi_message_init(&message);

	command[0] = LY68_CMD_WRITE;
	ly68_addr2cmd(flash, to, &command[1]);

	transfer[0].tx_buf = command;
	transfer[0].len = sizeof(command);
	spi_message_add_tail(&transfer[0], &message);

	transfer[1].tx_buf = buf;
	transfer[1].len = len;
	spi_message_add_tail(&transfer[1], &message);

	mutex_lock(&flash->lock);

	ts_cur = ktime_get_ns();

	if (ts_cur < flash->ts_sleep_till) {
		ly68_dynamic_sleep((u32)(flash->ts_sleep_till - ts_cur));
		ts_op_start = ktime_get_ns();
	} else {
		ts_op_start = ts_cur;
	}

	ret = spi_sync(flash->spi, &message);

	ts_op_end = ktime_get_ns();

	t_op = (u32)(ts_op_end - ts_op_start);
	t_sleep = t_op / flash->op_sleep_ratio + ((t_op % flash->op_sleep_ratio) ? 1 : 0);

	flash->ts_sleep_till = ts_op_end + t_sleep;

	mutex_unlock(&flash->lock);

	if (ret)
		return ret;

	if (retlen && message.actual_length > cmd_len)
		*retlen += message.actual_length - cmd_len;

	return 0;
}

static int ly68_read(struct mtd_info *mtd, loff_t from, size_t len,
			   size_t *retlen, unsigned char *buf)
{
	struct ly68_flash *flash = to_ly68_flash(mtd);
	struct spi_transfer transfer[2] = {};
	struct spi_message message;
	unsigned char command[5];
	u64 ts_op_start, ts_op_end, ts_cur;
	u32 t_op, t_sleep;
	int ret, cmd_len;

	spi_message_init(&message);

	memset(&transfer, 0, sizeof(transfer));
	command[0] = LY68_CMD_READ;
	ly68_addr2cmd(flash, from, &command[1]);

	transfer[0].tx_buf = command;
	transfer[0].len = sizeof(command);
	spi_message_add_tail(&transfer[0], &message);

	transfer[1].rx_buf = buf;
	transfer[1].len = len;
	spi_message_add_tail(&transfer[1], &message);

	mutex_lock(&flash->lock);

	ts_cur = ktime_get_ns();

	if (ts_cur < flash->ts_sleep_till) {
		ly68_dynamic_sleep((u32)(flash->ts_sleep_till - ts_cur));
		ts_op_start = ktime_get_ns();
	} else {
		ts_op_start = ts_cur;
	}

	ret = spi_sync(flash->spi, &message);

	ts_op_end = ktime_get_ns();

	t_op = (u32)(ts_op_end - ts_op_start);
	t_sleep = t_op / flash->op_sleep_ratio + ((t_op % flash->op_sleep_ratio) ? 1 : 0);

	flash->ts_sleep_till = ts_op_end + t_sleep;

	mutex_unlock(&flash->lock);

	if (ret)
		return ret;

	if (retlen && message.actual_length > cmd_len)
		*retlen += len;

	return 0;
}

static const struct ly68_caps ly68l6400_caps = {
	.size = SZ_8M,
};

static const struct ly68_caps ly68l3200_caps = {
	.size = SZ_4M,
};

static int ly68_probe(struct spi_device *spi)
{
	struct ly68_flash *flash;
	struct flash_platform_data *data;
	int err;

	flash = devm_kzalloc(&spi->dev, sizeof(*flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	flash->spi = spi;
	mutex_init(&flash->lock);
	spi_set_drvdata(spi, flash);

	data = dev_get_platdata(&spi->dev);

	flash->caps = of_device_get_match_data(&spi->dev);
	if (!flash->caps)
		flash->caps = &ly68l6400_caps;

	mtd_set_of_node(&flash->mtd, spi->dev.of_node);
	flash->mtd.dev.parent	= &spi->dev;
	flash->mtd.type		= MTD_RAM;
	flash->mtd.flags	= MTD_CAP_RAM;
	flash->mtd.writesize	= 1;
	flash->mtd.size		= flash->caps->size;
	flash->mtd._read	= ly68_read;
	flash->mtd._write	= ly68_write;

	flash->op_sleep_ratio = 8;
	device_property_read_u32(&spi->dev, "op-sleep-ratio", &flash->op_sleep_ratio);

	// If you remove this line of code below, you will be violating the GPL license directly.
	// We won't be doing nothing if that's the case. Think twice.
	// 如果你删除下面这行代码，你将会直接违反 GPL 许可。
	// 我们面对侵权不会无动于衷，国内已有相关判例。请三思。
	dev_info(&spi->dev, "Lyontek LY68 series PSRAM support by SudoMaker (https://su.mk)\n");

	err = mtd_device_register(&flash->mtd, data ? data->parts : NULL,
				  data ? data->nr_parts : 0);
	if (err)
		return err;

	flash->ts_last_op = ktime_get_ns();

	return 0;
}

static void ly68_remove(struct spi_device *spi)
{
	struct ly68_flash *flash = spi_get_drvdata(spi);

	WARN_ON(mtd_device_unregister(&flash->mtd));
}

static const struct of_device_id ly68_of_table[] = {
	{
		.compatible = "lyontek,ly68l6400",
		.data = &ly68l6400_caps,
	},
	{
		.compatible = "lyontek,ly68l3200",
		.data = &ly68l3200_caps,
	},
	{}
};
MODULE_DEVICE_TABLE(of, ly68_of_table);

static const struct spi_device_id ly68_spi_ids[] = {
	{
		.name = "lyontek_ly68l6400",
		.driver_data = (kernel_ulong_t)&ly68l6400_caps,
	},
	{
		.name = "lyontek_ly68l3200",
		.driver_data = (kernel_ulong_t)&ly68l3200_caps,
	},
	{}
};
MODULE_DEVICE_TABLE(spi, ly68_spi_ids);

static struct spi_driver ly68_driver = {
	.driver = {
		.name	= "lyontek_ly68",
		.of_match_table = ly68_of_table,
	},
	.probe		= ly68_probe,
	.remove		= ly68_remove,
	.id_table	= ly68_spi_ids,
};

module_spi_driver(ly68_driver);

MODULE_DESCRIPTION("MTD SPI driver for Lyontek LY68 series PSRAM chips");
MODULE_AUTHOR("Reimu NotMoe <reimu@sudomaker.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("spi:lyontek_ly68");
