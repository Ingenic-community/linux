// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Zero copy spidev driver
 *
 * Copyright (C) 2023 Reimu NotMoe <reimu@sudomaker.com>
 *
 * Based on the original spidev driver.
 */

#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/vmalloc.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>


/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);

static_assert(N_SPI_MINORS > 0 && N_SPI_MINORS <= 256);

/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_MODE_X_MASK | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
				| SPI_TX_QUAD | SPI_TX_OCTAL | SPI_RX_DUAL \
				| SPI_RX_QUAD | SPI_RX_OCTAL \
				| SPI_RX_CPHA_FLIP)

struct spidev_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* TX/RX buffers are NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*smol_tx_buffer;
	u8			*smol_rx_buffer;
	u32			speed_hz;
};

struct spidev_dio_mapping {
	struct page **pages;
	unsigned page_cnt;
	void *vm_addr;
};

struct spidev_dio {
	struct spi_transfer spi_xfer;
	struct spidev_dio_mapping map_tx, map_rx;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned smol_bufsiz = 4096;
module_param(smol_bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(smol_bufsiz, "smol buffer size");

/*-------------------------------------------------------------------------*/

static ssize_t
spidev_sync(struct spidev_data *spidev, struct spi_message *message)
{
	int status;
	struct spi_device *spi;

	spin_lock_irq(&spidev->spi_lock);
	spi = spidev->spi;
	spin_unlock_irq(&spidev->spi_lock);

	if (spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_sync(spi, message);

	if (status == 0)
		status = message->actual_length;

	return status;
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	return -ENOSYS;
}

/* Write-only message with current device setup */
static ssize_t
spidev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	return -ENOSYS;
}

static void calc_pages(unsigned long in_user_addr, unsigned in_user_len, unsigned *out_page_cnt, unsigned long *out_first_page_addr, unsigned *out_first_page_offset)
{
	unsigned page_cnt, first_page_offset;
	unsigned long first_page_addr;

	first_page_offset = in_user_addr % PAGE_SIZE;
	first_page_addr = in_user_addr - first_page_offset;
	page_cnt = ((first_page_offset + in_user_len - 1) / PAGE_SIZE) + 1;

	if (out_page_cnt)
		*out_page_cnt = page_cnt;

	if (out_first_page_addr)
		*out_first_page_addr = first_page_addr;

	if (out_first_page_offset)
		*out_first_page_offset = first_page_offset;
}

static void *spidev_dio_map_user(unsigned long user_addr, unsigned user_len, struct spidev_dio_mapping *dio_mapping) {
	unsigned page_cnt, first_page_offset;
	unsigned long first_page_addr;

	long pup_rc = 0;
	struct page **cur_pages = NULL;
	void *vm_addr;

	calc_pages(user_addr, user_len, &page_cnt, &first_page_addr, &first_page_offset);

	cur_pages = kcalloc(page_cnt, sizeof(*cur_pages), GFP_KERNEL);
	if (!cur_pages) {
		goto bad_ending;
	}

	pup_rc = pin_user_pages(first_page_addr, page_cnt, 0, cur_pages, NULL);

	if (pup_rc != page_cnt) {
		pr_err("pin_user_pages: %ld != %u\n", pup_rc, page_cnt);
		goto bad_ending;
	}

	vm_addr = vmap(cur_pages, page_cnt, 0, PAGE_KERNEL);
	if (!vm_addr) {
		pr_err("vmap failed\n");
		goto bad_ending;
	}

	dio_mapping->pages = cur_pages;
	dio_mapping->page_cnt = page_cnt;
	dio_mapping->vm_addr = vm_addr;

	return vm_addr + first_page_offset;

bad_ending:
	if (pup_rc > 0)
		unpin_user_pages(cur_pages, pup_rc);

	if (cur_pages)
		kfree(cur_pages);

	return NULL;
}

static void spidev_dio_unmap_user(struct spidev_dio_mapping *dio_mapping) {
	if (dio_mapping->vm_addr)
		vunmap(dio_mapping->vm_addr);

	if (dio_mapping->pages) {
		unpin_user_pages(dio_mapping->pages, dio_mapping->page_cnt);
		kfree(dio_mapping->pages);
	}
}

static int spidev_message_ginormous(struct spidev_data *spidev,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spidev_dio	*dios;

	unsigned		total = 0;
	int			status = -EFAULT;

	spi_message_init(&msg);

	dios = kcalloc(n_xfers, sizeof(*dios), GFP_KERNEL);
	if (dios == NULL)
		return -ENOMEM;

	for (unsigned i=0; i<n_xfers; i++) {
		struct spidev_dio *cur_dio = &dios[i];
		struct spi_ioc_transfer *cur_uspi_xfer = &u_xfers[i];
		struct spi_transfer *cur_kspi_xfer = &cur_dio->spi_xfer;

		void *buf;

		if (cur_uspi_xfer->len) {
			total += cur_uspi_xfer->len;
			cur_kspi_xfer->len = cur_uspi_xfer->len;

			if (cur_uspi_xfer->tx_buf) {
				buf = spidev_dio_map_user(cur_uspi_xfer->tx_buf, cur_uspi_xfer->len, &cur_dio->map_tx);

				if (buf)
					cur_kspi_xfer->tx_buf = buf;
				else
					goto end;
			}

			if (cur_uspi_xfer->rx_buf) {
				buf = spidev_dio_map_user(cur_uspi_xfer->rx_buf, cur_uspi_xfer->len, &cur_dio->map_rx);

				if (buf)
					cur_kspi_xfer->rx_buf = buf;
				else
					goto end;
			}
		}

		cur_kspi_xfer->cs_change = !!cur_uspi_xfer->cs_change;
		cur_kspi_xfer->tx_nbits = cur_uspi_xfer->tx_nbits;
		cur_kspi_xfer->rx_nbits = cur_uspi_xfer->rx_nbits;
		cur_kspi_xfer->bits_per_word = cur_uspi_xfer->bits_per_word;
		cur_kspi_xfer->delay.value = cur_uspi_xfer->delay_usecs;
		cur_kspi_xfer->delay.unit = SPI_DELAY_UNIT_USECS;
		cur_kspi_xfer->speed_hz = cur_uspi_xfer->speed_hz;
		cur_kspi_xfer->word_delay.value = cur_uspi_xfer->word_delay_usecs;
		cur_kspi_xfer->word_delay.unit = SPI_DELAY_UNIT_USECS;
		if (!cur_kspi_xfer->speed_hz)
			cur_kspi_xfer->speed_hz = spidev->speed_hz;

		spi_message_add_tail(cur_kspi_xfer, &msg);
	}

	status = spidev_sync(spidev, &msg);

end:
	for (unsigned i=0; i<n_xfers; i++) {
		struct spidev_dio *cur_dio = &dios[i];
		spidev_dio_unmap_user(&cur_dio->map_tx);
		spidev_dio_unmap_user(&cur_dio->map_rx);
	}

	kfree(dios);

	if (status == 0)
		status = total;

	return status;
}

static int spidev_message_smol(struct spidev_data *spidev,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		n, total, tx_total, rx_total;
	u8			*tx_buf, *rx_buf;
	int			status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	tx_buf = spidev->smol_tx_buffer;
	rx_buf = spidev->smol_rx_buffer;
	total = 0;
	tx_total = 0;
	rx_total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		/* Ensure that also following allocations from rx_buf/tx_buf will meet
		 * DMA alignment requirements.
		 */
		unsigned int len_aligned = ALIGN(u_tmp->len, ARCH_KMALLOC_MINALIGN);

		k_tmp->len = u_tmp->len;

		total += k_tmp->len;

		if (u_tmp->rx_buf) {
			/* this transfer needs space in RX bounce buffer */
			rx_total += len_aligned;
			if (rx_total > smol_bufsiz) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->rx_buf = rx_buf;
			rx_buf += len_aligned;
		}
		if (u_tmp->tx_buf) {
			/* this transfer needs space in TX bounce buffer */
			tx_total += len_aligned;
			if (tx_total > smol_bufsiz) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->tx_buf = tx_buf;
			if (copy_from_user(tx_buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
			tx_buf += len_aligned;
		}

		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->tx_nbits = u_tmp->tx_nbits;
		k_tmp->rx_nbits = u_tmp->rx_nbits;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay.value = u_tmp->delay_usecs;
		k_tmp->delay.unit = SPI_DELAY_UNIT_USECS;
		k_tmp->speed_hz = u_tmp->speed_hz;
		k_tmp->word_delay.value = u_tmp->word_delay_usecs;
		k_tmp->word_delay.unit = SPI_DELAY_UNIT_USECS;
		if (!k_tmp->speed_hz)
			k_tmp->speed_hz = spidev->speed_hz;
#ifdef VERBOSE
		dev_dbg(&spidev->spi->dev,
			"  xfer len %u %s%s%s%dbits %u usec %u usec %uHz\n",
			k_tmp->len,
			k_tmp->rx_buf ? "rx " : "",
			k_tmp->tx_buf ? "tx " : "",
			k_tmp->cs_change ? "cs " : "",
			k_tmp->bits_per_word ? : spidev->spi->bits_per_word,
			k_tmp->delay.value,
			k_tmp->word_delay.value,
			k_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}

	status = spidev_sync(spidev, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, k_tmp->rx_buf,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
		}
	}
	status = total;

done:
	kfree(k_xfers);
	return status;
}

static int spidev_message(struct spidev_data *spidev,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	unsigned total_xfer_bytes = 0;

	for (unsigned i=0; i<n_xfers; i++) {
		struct spi_ioc_transfer *cur_uspi_xfer = &u_xfers[i];
		total_xfer_bytes += cur_uspi_xfer->len;

		/* Since the function returns the total length of transfers
		 * on success, restrict the total to positive int values to
		 * avoid the return value looking like an error.  Also check
		 * each transfer length to avoid arithmetic overflow.
		 */
		if (total_xfer_bytes > INT_MAX || cur_uspi_xfer->len > INT_MAX) {
			return -EMSGSIZE;
		}
	}

	if (total_xfer_bytes == 0)
		return 0;

	if (total_xfer_bytes > smol_bufsiz)
		return spidev_message_ginormous(spidev, u_xfers, n_xfers);
	else
		return spidev_message_smol(spidev, u_xfers, n_xfers);
}

static struct spi_ioc_transfer *
spidev_get_ioc_message(unsigned int cmd, struct spi_ioc_transfer __user *u_ioc,
		unsigned *n_ioc)
{
	u32	tmp;

	/* Check type, command number and direction */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC
			|| _IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
			|| _IOC_DIR(cmd) != _IOC_WRITE)
		return ERR_PTR(-ENOTTY);

	tmp = _IOC_SIZE(cmd);
	if ((tmp % sizeof(struct spi_ioc_transfer)) != 0)
		return ERR_PTR(-EINVAL);
	*n_ioc = tmp / sizeof(struct spi_ioc_transfer);
	if (*n_ioc == 0)
		return NULL;

	/* copy into scratch area */
	return memdup_user(u_ioc, tmp);
}

static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			retval = 0;
	struct spidev_data	*spidev;
	struct spi_device	*spi;
	u32			tmp;
	unsigned		n_ioc;
	struct spi_ioc_transfer	*ioc;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	spidev = filp->private_data;
	spin_lock_irq(&spidev->spi_lock);
	spi = spi_dev_get(spidev->spi);
	spin_unlock_irq(&spidev->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&spidev->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
	case SPI_IOC_RD_MODE32:
		tmp = spi->mode;

		{
			struct spi_controller *ctlr = spi->controller;

			if (ctlr->use_gpio_descriptors && ctlr->cs_gpiods &&
			    ctlr->cs_gpiods[spi->chip_select])
				tmp &= ~SPI_CS_HIGH;
		}

		if (cmd == SPI_IOC_RD_MODE)
			retval = put_user(tmp & SPI_MODE_MASK,
					  (__u8 __user *)arg);
		else
			retval = put_user(tmp & SPI_MODE_MASK,
					  (__u32 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = put_user(spidev->speed_hz, (__u32 __user *)arg);
		break;

	/* write requests */
	case SPI_IOC_WR_MODE:
	case SPI_IOC_WR_MODE32:
		if (cmd == SPI_IOC_WR_MODE)
			retval = get_user(tmp, (u8 __user *)arg);
		else
			retval = get_user(tmp, (u32 __user *)arg);
		if (retval == 0) {
			struct spi_controller *ctlr = spi->controller;
			u32	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			if (ctlr->use_gpio_descriptors && ctlr->cs_gpiods &&
			    ctlr->cs_gpiods[spi->chip_select])
				tmp |= SPI_CS_HIGH;

			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = tmp & SPI_MODE_USER_MASK;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u32	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ: {
		u32 save;

		retval = get_user(tmp, (__u32 __user *)arg);
		if (retval)
			break;
		if (tmp == 0) {
			retval = -EINVAL;
			break;
		}

		save = spi->max_speed_hz;

		spi->max_speed_hz = tmp;
		retval = spi_setup(spi);
		if (retval == 0) {
			spidev->speed_hz = tmp;
			dev_dbg(&spi->dev, "%d Hz (max)\n", spidev->speed_hz);
		}

		spi->max_speed_hz = save;
		break;
	}
	default:
		/* segmented and/or full-duplex I/O request */
		/* Check message and copy into scratch area */
		ioc = spidev_get_ioc_message(cmd,
				(struct spi_ioc_transfer __user *)arg, &n_ioc);
		if (IS_ERR(ioc)) {
			retval = PTR_ERR(ioc);
			break;
		}
		if (!ioc)
			break;	/* n_ioc is also 0 */

		/* translate to spi_message, execute */
		retval = spidev_message(spidev, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	mutex_unlock(&spidev->buf_lock);
	spi_dev_put(spi);
	return retval;
}

#ifdef CONFIG_COMPAT
static long
spidev_compat_ioc_message(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct spi_ioc_transfer __user	*u_ioc;
	int				retval = 0;
	struct spidev_data		*spidev;
	struct spi_device		*spi;
	unsigned			n_ioc, n;
	struct spi_ioc_transfer		*ioc;

	u_ioc = (struct spi_ioc_transfer __user *) compat_ptr(arg);

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	spidev = filp->private_data;
	spin_lock_irq(&spidev->spi_lock);
	spi = spi_dev_get(spidev->spi);
	spin_unlock_irq(&spidev->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* SPI_IOC_MESSAGE needs the buffer locked "normally" */
	mutex_lock(&spidev->buf_lock);

	/* Check message and copy into scratch area */
	ioc = spidev_get_ioc_message(cmd, u_ioc, &n_ioc);
	if (IS_ERR(ioc)) {
		retval = PTR_ERR(ioc);
		goto done;
	}
	if (!ioc)
		goto done;	/* n_ioc is also 0 */

	/* Convert buffer pointers */
	for (n = 0; n < n_ioc; n++) {
		ioc[n].rx_buf = (uintptr_t) compat_ptr(ioc[n].rx_buf);
		ioc[n].tx_buf = (uintptr_t) compat_ptr(ioc[n].tx_buf);
	}

	/* translate to spi_message, execute */
	retval = spidev_message(spidev, ioc, n_ioc);
	kfree(ioc);

done:
	mutex_unlock(&spidev->buf_lock);
	spi_dev_put(spi);
	return retval;
}

static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	if (_IOC_TYPE(cmd) == SPI_IOC_MAGIC
			&& _IOC_NR(cmd) == _IOC_NR(SPI_IOC_MESSAGE(0))
			&& _IOC_DIR(cmd) == _IOC_WRITE)
		return spidev_compat_ioc_message(filp, cmd, arg);

	return spidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int spidev_open(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev = NULL, *iter;
	int			status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(iter, &device_list, device_entry) {
		if (iter->devt == inode->i_rdev) {
			status = 0;
			spidev = iter;
			break;
		}
	}

	if (!spidev) {
		pr_debug("spidev: nothing for minor %d\n", iminor(inode));
		goto err_find_dev;
	}

	if (!spidev->smol_tx_buffer) {
		spidev->smol_tx_buffer = kmalloc(smol_bufsiz, GFP_KERNEL);
		if (!spidev->smol_tx_buffer) {
			status = -ENOMEM;
			goto err_find_dev;
		}
	}

	if (!spidev->smol_rx_buffer) {
		spidev->smol_rx_buffer = kmalloc(smol_bufsiz, GFP_KERNEL);
		if (!spidev->smol_rx_buffer) {
			status = -ENOMEM;
			goto err_alloc_rx_buf;
		}
	}

	spidev->users++;
	filp->private_data = spidev;
	stream_open(inode, filp);

	mutex_unlock(&device_list_lock);
	return 0;

err_alloc_rx_buf:
	kfree(spidev->smol_tx_buffer);
	spidev->smol_tx_buffer = NULL;
err_find_dev:
	mutex_unlock(&device_list_lock);
	return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev;
	int			dofree;

	mutex_lock(&device_list_lock);
	spidev = filp->private_data;
	filp->private_data = NULL;

	spin_lock_irq(&spidev->spi_lock);
	/* ... after we unbound from the underlying device? */
	dofree = (spidev->spi == NULL);
	spin_unlock_irq(&spidev->spi_lock);

	/* last close? */
	spidev->users--;
	if (!spidev->users) {

		kfree(spidev->smol_tx_buffer);
		spidev->smol_tx_buffer = NULL;

		kfree(spidev->smol_rx_buffer);
		spidev->smol_rx_buffer = NULL;

		if (dofree)
			kfree(spidev);
		else
			spidev->speed_hz = spidev->spi->max_speed_hz;
	}
#ifdef CONFIG_SPI_SLAVE
	if (!dofree)
		spi_slave_abort(spidev->spi);
#endif
	mutex_unlock(&device_list_lock);

	return 0;
}

static const struct file_operations spidev_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	spidev_write,
	.read =		spidev_read,
	.unlocked_ioctl = spidev_ioctl,
	.compat_ioctl = spidev_compat_ioctl,
	.open =		spidev_open,
	.release =	spidev_release,
	.llseek =	no_llseek,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *spidev_class;

static const struct spi_device_id spidev_spi_ids[] = {
	{ .name = "spidev-zerocopy" },
	{},
};
MODULE_DEVICE_TABLE(spi, spidev_spi_ids);

/*
 * spidev should never be referenced in DT without a specific compatible string,
 * it is a Linux implementation thing rather than a description of the hardware.
 */
static int spidev_of_check(struct device *dev)
{
	if (device_property_match_string(dev, "compatible", "spidev") < 0)
		return 0;

	dev_err(dev, "spidev listed directly in DT is not supported\n");
	return -EINVAL;
}

static const struct of_device_id spidev_dt_ids[] = {
	{ .compatible = "sudomaker,spidev-zerocopy", .data = &spidev_of_check },
	{},
};
MODULE_DEVICE_TABLE(of, spidev_dt_ids);

/* Dummy SPI devices not to be used in production systems */
static int spidev_acpi_check(struct device *dev)
{
	dev_warn(dev, "do not use this driver in production systems!\n");
	return 0;
}

static const struct acpi_device_id spidev_acpi_ids[] = {
	/*
	 * The ACPI SPT000* devices are only meant for development and
	 * testing. Systems used in production should have a proper ACPI
	 * description of the connected peripheral and they should also use
	 * a proper driver instead of poking directly to the SPI bus.
	 */
	{ "SPT0001", (kernel_ulong_t)&spidev_acpi_check },
	{ "SPT0002", (kernel_ulong_t)&spidev_acpi_check },
	{ "SPT0003", (kernel_ulong_t)&spidev_acpi_check },
	{},
};
MODULE_DEVICE_TABLE(acpi, spidev_acpi_ids);

/*-------------------------------------------------------------------------*/

static int spidev_probe(struct spi_device *spi)
{
	int (*match)(struct device *dev);
	struct spidev_data	*spidev;
	int			status;
	unsigned long		minor;

	match = device_get_match_data(&spi->dev);
	if (match) {
		status = match(&spi->dev);
		if (status)
			return status;
	}

	/* Allocate driver data */
	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	if (!spidev)
		return -ENOMEM;

	/* Initialize the driver data */
	spidev->spi = spi;
	spin_lock_init(&spidev->spi_lock);
	mutex_init(&spidev->buf_lock);

	INIT_LIST_HEAD(&spidev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		spidev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(spidev_class, &spi->dev, spidev->devt,
				    spidev, "spidev%d.%d",
				    spi->master->bus_num, spi->chip_select);
		status = PTR_ERR_OR_ZERO(dev);
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&spidev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	spidev->speed_hz = spi->max_speed_hz;

	if (status == 0)
		spi_set_drvdata(spi, spidev);
	else
		kfree(spidev);

	return status;
}

static void spidev_remove(struct spi_device *spi)
{
	struct spidev_data	*spidev = spi_get_drvdata(spi);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spidev->spi_lock);
	spidev->spi = NULL;
	spin_unlock_irq(&spidev->spi_lock);

	list_del(&spidev->device_entry);
	device_destroy(spidev_class, spidev->devt);
	clear_bit(MINOR(spidev->devt), minors);
	if (spidev->users == 0)
		kfree(spidev);
	mutex_unlock(&device_list_lock);
}

static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name =		"spidev",
		.of_match_table = spidev_dt_ids,
		.acpi_match_table = spidev_acpi_ids,
	},
	.probe =	spidev_probe,
	.remove =	spidev_remove,
	.id_table =	spidev_spi_ids,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init spidev_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	status = register_chrdev(SPIDEV_MAJOR, "spi", &spidev_fops);
	if (status < 0)
		return status;

	spidev_class = class_create(THIS_MODULE, "spidev");
	if (IS_ERR(spidev_class)) {
		unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
		return PTR_ERR(spidev_class);
	}

	status = spi_register_driver(&spidev_spi_driver);
	if (status < 0) {
		class_destroy(spidev_class);
		unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
	}
	return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
	spi_unregister_driver(&spidev_spi_driver);
	class_destroy(spidev_class);
	unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
}
module_exit(spidev_exit);

MODULE_AUTHOR("Andrea Paterniani, <a.paterniani@swapp-eng.it>");
MODULE_AUTHOR("Reimu NotMoe <reimu@sudomaker.com>");
MODULE_DESCRIPTION("User mode SPI device interface with zero copy");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev_zerocopy");
