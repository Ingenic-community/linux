#ifndef _LINUX_LOOP_H
#define _LINUX_LOOP_H

/*
 * include/linux/loop.h
 *
 * Written by Theodore Ts'o, 3/29/93.
 *
 * Copyright 1993 by Theodore Ts'o.  Redistribution of this file is
 * permitted under the GNU General Public License.
 */

#define LO_NAME_SIZE	64
#define LO_KEY_SIZE	32

#ifdef __KERNEL__
#include <linux/version.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/module.h>
#include <linux/spinlock.h>

struct loop_func_table;

struct loop_device {
	int		lo_number;
	int		lo_refcnt;
	loff_t		lo_offset;
	loff_t		lo_sizelimit;
	int		lo_flags;
	int		(*transfer)(struct loop_device *, int cmd,
				    char *raw_buf, char *loop_buf, int size,
				    sector_t real_block);
	struct loop_func_table *lo_encryption;
	char		lo_file_name[LO_NAME_SIZE];
	char		lo_crypt_name[LO_NAME_SIZE];
	char		lo_encrypt_key[LO_KEY_SIZE];
	int		lo_encrypt_key_size;
#if LINUX_VERSION_CODE >= 0x30600
	kuid_t		lo_key_owner;	/* Who set the key */
#else
	uid_t		lo_key_owner;	/* Who set the key */
#endif
	__u32           lo_init[2];
	int		(*ioctl)(struct loop_device *, int cmd,
				 unsigned long arg);

	struct file *	lo_backing_file;
	struct block_device *lo_device;
	void		*key_data;

	int		old_gfp_mask;

	spinlock_t		lo_lock;
	struct completion	lo_done;
	atomic_t		lo_pending;

	struct request_queue	*lo_queue;

	struct bio		*lo_bio_que0;
	struct bio		*lo_bio_free0;
	struct bio		*lo_bio_free1;
	int			lo_bio_flshMax;
	int			lo_bio_flshCnt;
	wait_queue_head_t	lo_bio_wait;
	wait_queue_head_t	lo_buf_wait;
	sector_t		lo_offs_sec;
	sector_t		lo_iv_remove;
	spinlock_t		lo_ioctl_spin;
	int			lo_ioctl_busy;
	wait_queue_head_t	lo_ioctl_wait;
	struct request_queue	*lo_backingQueue;
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
	void			(*lo_keyscrub_fn)(void *);
	void			*lo_keyscrub_ptr;
#endif
};

#endif /* __KERNEL__ */

/*
 * Loop flags
 */
#define LO_FLAGS_DO_BMAP	1
#define LO_FLAGS_READ_ONLY	2
#define LO_FLAGS_INITIALIZED	4

#include <asm/posix_types.h>	/* for __kernel_old_dev_t */
#include <asm/types.h>		/* for __u64 */

/* Backwards compatibility version */
struct loop_info {
	int		   lo_number;		/* ioctl r/o */
	__kernel_old_dev_t lo_device; 		/* ioctl r/o */
	unsigned long	   lo_inode; 		/* ioctl r/o */
	__kernel_old_dev_t lo_rdevice; 		/* ioctl r/o */
	int		   lo_offset;
	int		   lo_encrypt_type;
	int		   lo_encrypt_key_size; 	/* ioctl w/o */
	int		   lo_flags;			/* ioctl r/o */
	char		   lo_name[LO_NAME_SIZE];
	unsigned char	   lo_encrypt_key[LO_KEY_SIZE]; /* ioctl w/o */
	unsigned long	   lo_init[2];
	char		   reserved[4];
};

struct loop_info64 {
	__u64		   lo_device;			/* ioctl r/o */
	__u64		   lo_inode;			/* ioctl r/o */
	__u64		   lo_rdevice;			/* ioctl r/o */
	__u64		   lo_offset;
	__u64		   lo_sizelimit;/* bytes, 0 == max available */
	__u32		   lo_number;			/* ioctl r/o */
	__u32		   lo_encrypt_type;
	__u32		   lo_encrypt_key_size;		/* ioctl w/o */
	__u32		   lo_flags;			/* ioctl r/o */
	__u8		   lo_file_name[LO_NAME_SIZE];
	__u8		   lo_crypt_name[LO_NAME_SIZE];
	__u8		   lo_encrypt_key[LO_KEY_SIZE]; /* ioctl w/o */
	__u64		   lo_init[2];
};

/*
 * Loop filter types
 */

#define LO_CRYPT_NONE		0
#define LO_CRYPT_XOR		1
#define LO_CRYPT_DES		2
#define LO_CRYPT_FISH2		3    /* Twofish encryption */
#define LO_CRYPT_BLOW		4
#define LO_CRYPT_CAST128	5
#define LO_CRYPT_IDEA		6
#define LO_CRYPT_DUMMY		9
#define LO_CRYPT_SKIPJACK	10
#define LO_CRYPT_AES		16
#define LO_CRYPT_CRYPTOAPI	18
#define MAX_LO_CRYPT		20

#ifdef __KERNEL__
/* Support for loadable transfer modules */
struct loop_func_table {
	int number;	/* filter type */
	int (*transfer)(struct loop_device *lo, int cmd, char *raw_buf,
			char *loop_buf, int size, sector_t real_block);
	int (*init)(struct loop_device *, struct loop_info64 *);
	/* release is called from loop_unregister_transfer or clr_fd */
	int (*release)(struct loop_device *);
	int (*ioctl)(struct loop_device *, int cmd, unsigned long arg);
	struct module *owner;
};

int loop_register_transfer(struct loop_func_table *funcs);
int loop_unregister_transfer(int number);

#endif
/*
 * IOCTL commands --- we will commandeer 0x4C ('L')
 */

#define LOOP_SET_FD		0x4C00
#define LOOP_CLR_FD		0x4C01
#define LOOP_SET_STATUS		0x4C02
#define LOOP_GET_STATUS		0x4C03
#define LOOP_SET_STATUS64	0x4C04
#define LOOP_GET_STATUS64	0x4C05
#define LOOP_CHANGE_FD		0x4C06

#define LOOP_MULTI_KEY_SETUP     0x4C4D
#define LOOP_MULTI_KEY_SETUP_V3  0x4C4E
#define LOOP_RECOMPUTE_DEV_SIZE  0x4C52
#endif
