/*
 *  linux/drivers/block/loop.c
 *
 *  Written by Theodore Ts'o, 3/29/93
 *
 * Copyright 1993 by Theodore Ts'o.  Redistribution of this file is
 * permitted under the GNU General Public License.
 *
 * DES encryption plus some minor changes by Werner Almesberger, 30-MAY-1993
 * more DES encryption plus IDEA encryption by Nicholas J. Leon, June 20, 1996
 *
 * Modularized and updated for 1.1.16 kernel - Mitch Dsouza 28th May 1994
 * Adapted for 1.3.59 kernel - Andries Brouwer, 1 Feb 1996
 *
 * Fixed do_loop_request() re-entrancy - Vincent.Renardias@waw.com Mar 20, 1997
 *
 * Added devfs support - Richard Gooch <rgooch@atnf.csiro.au> 16-Jan-1998
 *
 * Handle sparse backing files correctly - Kenn Humborg, Jun 28, 1998
 *
 * Loadable modules and other fixes by AK, 1998
 *
 * Make real block number available to downstream transfer functions, enables
 * CBC (and relatives) mode encryption requiring unique IVs per data block.
 * Reed H. Petty, rhp@draper.net
 *
 * Maximum number of loop devices now dynamic via max_loop module parameter.
 * Russell Kroll <rkroll@exploits.org> 19990701
 *
 * Maximum number of loop devices when compiled-in now selectable by passing
 * max_loop=<1-255> to the kernel on boot.
 * Erik I. Bolsø, <eriki@himolde.no>, Oct 31, 1999
 *
 * Completely rewrite request handling to be make_request_fn style and
 * non blocking, pushing work to a helper thread. Lots of fixes from
 * Al Viro too.
 * Jens Axboe <axboe@suse.de>, Nov 2000
 *
 * Support up to 256 loop devices
 * Heinz Mauelshagen <mge@sistina.com>, Feb 2002
 *
 * AES transfer added. IV is now passed as (512 byte) sector number.
 * Jari Ruusu, May 18 2001
 *
 * External encryption module locking bug fixed.
 * Ingo Rohloff <rohloff@in.tum.de>, June 21 2001
 *
 * Make device backed loop work with swap (pre-allocated buffers + queue rewrite).
 * Jari Ruusu, September 2 2001
 *
 * Ported 'pre-allocated buffers + queue rewrite' to BIO for 2.5 kernels
 * Ben Slusky <sluskyb@stwing.org>, March 1 2002
 * Jari Ruusu, March 27 2002
 *
 * File backed code now uses file->f_op->read/write. Based on Andrew Morton's idea.
 * Jari Ruusu, May 23 2002
 *
 * Exported hard sector size correctly, fixed file-backed-loop-on-tmpfs bug,
 * plus many more enhancements and optimizations.
 * Adam J. Richter <adam@yggdrasil.com>, Aug 2002
 *
 * Added support for removing offset from IV computations.
 * Jari Ruusu, September 21 2003
 *
 * Added support for MD5 IV computation and multi-key operation.
 * Jari Ruusu, October 8 2003
 *
 *
 * Still To Fix:
 * - Advisory locking is ignored here.
 * - Should use an own CAP_* category instead of CAP_SYS_ADMIN
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/bio.h>
#include <linux/stat.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/wait.h>
#include <linux/blkdev.h>
#include <linux/blkpg.h>
#include <linux/init.h>
#ifdef CONFIG_DEVFS_FS
# include <linux/devfs_fs_kernel.h>
#endif
#include <linux/swap.h>
#include <linux/slab.h>
#include <linux/loop.h>
#include <linux/suspend.h>
#include <linux/writeback.h>
#include <linux/buffer_head.h>		/* for invalidate_bdev() */
#include <linux/completion.h>
#include <linux/kthread.h>
#if defined(CONFIG_COMPAT)
# include <linux/compat.h>
#endif
#include <linux/mqueue.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/backing-dev.h>

#include <asm/uaccess.h>
#include <asm/byteorder.h>
#if (defined(CONFIG_BLK_DEV_LOOP_PADLOCK) || defined(CONFIG_BLK_DEV_LOOP_INTELAES)) && (defined(CONFIG_X86) || defined(CONFIG_X86_64))
# include <asm/processor.h>
#endif
#if defined(CONFIG_BLK_DEV_LOOP_INTELAES) && (defined(CONFIG_X86) || defined(CONFIG_X86_64))
# include <asm/fpu/api.h>
#endif

#if defined(CONFIG_X86) && !defined(CONFIG_X86_64)
# define X86_ASM  1
#endif
#if defined(CONFIG_X86_64)
# define AMD64_ASM  1
#endif

#include "../misc/aes.h"
#include "../misc/md5.h"

//#define LOOP_HAVE_CONGESTED_FN  1

static int max_loop = 8;

#ifdef MODULE
module_param(max_loop, int, 0);
MODULE_PARM_DESC(max_loop, "Maximum number of loop devices (1-256)");
#else
static int __init max_loop_setup(char *str)
{
	int y;

	if (get_option(&str, &y) == 1)
		max_loop = y;
	return 1;
}
__setup("max_loop=", max_loop_setup);
#endif

static struct gendisk **disks;

/*
 * Transfer functions
 */
static int transfer_none(struct loop_device *lo, int cmd, char *raw_buf,
			 char *loop_buf, int size, sector_t real_block)
{
	/* this code is only called from file backed loop  */
	/* and that code expects this function to be no-op */

	cond_resched();
	return 0;
}

static int transfer_xor(struct loop_device *lo, int cmd, char *raw_buf,
			char *loop_buf, int size, sector_t real_block)
{
	char	*in, *out, *key;
	int	i, keysize;

	if (cmd == READ) {
		in = raw_buf;
		out = loop_buf;
	} else {
		in = loop_buf;
		out = raw_buf;
	}

	key = lo->lo_encrypt_key;
	keysize = lo->lo_encrypt_key_size;
	for (i = 0; i < size; i++)
		*out++ = *in++ ^ key[(i & 511) % keysize];
	cond_resched();
	return 0;
}

static int xor_init(struct loop_device *lo, struct loop_info64 *info)
{
	if (info->lo_encrypt_key_size <= 0)
		return -EINVAL;
	return 0;
}

static struct loop_func_table none_funcs = {
	.number = LO_CRYPT_NONE,
	.transfer = transfer_none,
};

static struct loop_func_table xor_funcs = {
	.number = LO_CRYPT_XOR,
	.transfer = transfer_xor,
	.init = xor_init,
};

#ifdef CONFIG_BLK_DEV_LOOP_AES
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
# define KEY_ALLOC_COUNT  128
#else
# define KEY_ALLOC_COUNT  64
#endif

typedef struct {
    aes_context *keyPtr[KEY_ALLOC_COUNT];
    unsigned    keyMask;
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
    u_int32_t   *partialMD5;
    u_int32_t   partialMD5buf[8];
    rwlock_t    rwlock;
    unsigned    reversed;
    unsigned    blocked;
    struct timer_list timer;
#if LINUX_VERSION_CODE >= 0x40f00
    struct loop_device *lo;    
#endif
#else
    u_int32_t   partialMD5[4];
#endif
#if defined(CONFIG_BLK_DEV_LOOP_PADLOCK) && (defined(CONFIG_X86) || defined(CONFIG_X86_64))
    u_int32_t   padlock_cw_e;
    u_int32_t   padlock_cw_d;
#endif
} AESmultiKey;

#if (defined(CONFIG_BLK_DEV_LOOP_PADLOCK) || defined(CONFIG_BLK_DEV_LOOP_INTELAES)) && (defined(CONFIG_X86) || defined(CONFIG_X86_64))
/* This function allocates AES context structures at special address such */
/* that returned address % 16 == 8 . That way expanded encryption and */
/* decryption keys in AES context structure are always 16 byte aligned */
static void *specialAligned_kmalloc(size_t size, unsigned int flags)
{
    void *pn, **ps;
    pn = kmalloc(size + (16 + 8), flags);
    if(!pn) return (void *)0;
    ps = (void **)((((unsigned long)pn + 15) & ~((unsigned long)15)) + 8);
    *(ps - 1) = pn;
    return (void *)ps;
}
static void specialAligned_kfree(void *ps)
{
    if(ps) kfree(*((void **)ps - 1));
}
# define specialAligned_ctxSize     ((sizeof(aes_context) + 15) & ~15)
#else
# define specialAligned_kmalloc     kmalloc
# define specialAligned_kfree       kfree
# define specialAligned_ctxSize     sizeof(aes_context)
#endif

#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
static void keyScrubWork(AESmultiKey *m)
{
    aes_context *a0, *a1;
    u_int32_t *p;
    int x, y, z;

    z = m->keyMask + 1;
    for(x = 0; x < z; x++) {
        a0 = m->keyPtr[x];
        a1 = m->keyPtr[x + z];
        memcpy(a1, a0, sizeof(aes_context));
        m->keyPtr[x] = a1;
        m->keyPtr[x + z] = a0;
        p = (u_int32_t *) a0;
        y = sizeof(aes_context) / sizeof(u_int32_t);
        while(y > 0) {
            *p ^= 0xFFFFFFFF;
            p++;
            y--;
        }
    }

    x = m->reversed;    /* x is 0 or 4 */
    m->reversed ^= 4;
    y = m->reversed;    /* y is 4 or 0 */
    p = &m->partialMD5buf[x];
    memcpy(&m->partialMD5buf[y], p, 16);
    m->partialMD5 = &m->partialMD5buf[y];
    p[0] ^= 0xFFFFFFFF;
    p[1] ^= 0xFFFFFFFF;
    p[2] ^= 0xFFFFFFFF;
    p[3] ^= 0xFFFFFFFF;

    /* try to flush dirty cache data to RAM */
#if !defined(CONFIG_XEN) && (defined(CONFIG_X86_64) || (defined(CONFIG_X86) && !defined(CONFIG_M386) && !defined(CONFIG_CPU_386)))
    __asm__ __volatile__ ("wbinvd": : :"memory");
#else
    mb();
#endif
}

/* called only from loop thread process context */
static void keyScrubThreadFn(AESmultiKey *m)
{
    write_lock(&m->rwlock);
    if(!m->blocked) keyScrubWork(m);
    write_unlock(&m->rwlock);
}

#if LINUX_VERSION_CODE >= 0x40f00
# define KeyScrubTimerFnParamType struct timer_list *
#elif defined(NEW_TIMER_VOID_PTR_PARAM)
# define KeyScrubTimerFnParamType void *
#else
# define KeyScrubTimerFnParamType unsigned long
#endif

static void keyScrubTimerFn(KeyScrubTimerFnParamType);

static void keyScrubTimerInit(struct loop_device *lo)
{
    AESmultiKey     *m;

#if LINUX_VERSION_CODE >= 0x40f00
    m = (AESmultiKey *)lo->key_data;
    mod_timer(&m->timer, jiffies + HZ);
#else
    unsigned long   expire;

    m = (AESmultiKey *)lo->key_data;
    expire = jiffies + HZ;
    m->timer.expires = expire;
    m->timer.data = (KeyScrubTimerFnParamType)lo;
    m->timer.function = keyScrubTimerFn;
    add_timer(&m->timer);
#endif
}

/* called only from timer handler context */
static void keyScrubTimerFn(KeyScrubTimerFnParamType d)
{
#if LINUX_VERSION_CODE >= 0x40f00
    AESmultiKey *m = from_timer(m, d, timer);
    struct loop_device *lo = m->lo;
#else
    struct loop_device *lo = (struct loop_device *)d;
#endif
    extern void loop_add_keyscrub_fn(struct loop_device *, void (*)(void *), void *);

    /* rw lock needs process context, so make loop thread do scrubbing */
    loop_add_keyscrub_fn(lo, (void (*)(void*))keyScrubThreadFn, lo->key_data);
    /* start timer again */
    keyScrubTimerInit(lo);
}
#endif

static AESmultiKey *allocMultiKey(void)
{
    AESmultiKey *m;
    aes_context *a;
    int x = 0, n;

    m = (AESmultiKey *) kmalloc(sizeof(AESmultiKey), GFP_KERNEL);
    if(!m) return 0;
    memset(m, 0, sizeof(AESmultiKey));
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
    m->partialMD5 = &m->partialMD5buf[0];
    rwlock_init(&m->rwlock);
#if LINUX_VERSION_CODE >= 0x40f00
    timer_setup(&m->timer, keyScrubTimerFn, 0);
#else
    init_timer(&m->timer);
#endif
    again:
#endif

    n = PAGE_SIZE / specialAligned_ctxSize;
    if(!n) n = 1;

    a = (aes_context *) specialAligned_kmalloc(specialAligned_ctxSize * n, GFP_KERNEL);
    if(!a) {
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
        if(x) specialAligned_kfree(m->keyPtr[0]);
#endif
        kfree(m);
        return 0;
    }

    while((x < KEY_ALLOC_COUNT) && n) {
        m->keyPtr[x] = a;
        a = (aes_context *)((unsigned char *)a + specialAligned_ctxSize);
        x++;
        n--;
    }
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
    if(x < 2) goto again;
#endif
    return m;
}

static void clearAndFreeMultiKey(AESmultiKey *m)
{
    aes_context *a;
    int x, n;

#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
    /* stop scrub timer. loop thread was killed earlier */
    del_timer_sync(&m->timer);
    /* make sure allocated keys are in original order */
    if(m->reversed) keyScrubWork(m);
#endif
    n = PAGE_SIZE / specialAligned_ctxSize;
    if(!n) n = 1;

    x = 0;
    while(x < KEY_ALLOC_COUNT) {
        a = m->keyPtr[x];
        if(!a) break;
        memset(a, 0, specialAligned_ctxSize * n);
        specialAligned_kfree(a);
        x += n;
    }

    memset(m, 0, sizeof(AESmultiKey));
    kfree(m);
}

static int multiKeySetup(struct loop_device *lo, unsigned char *k, int version3)
{
    AESmultiKey *m;
    aes_context *a;
    int x, y, n, err = 0;
    union {
        u_int32_t     w[16];
        unsigned char b[64];
    } un;

#if LINUX_VERSION_CODE >= 0x30600
    if(!uid_eq(lo->lo_key_owner, current_uid()) && !capable(CAP_SYS_ADMIN))
        return -EPERM;
#elif LINUX_VERSION_CODE >= 0x2061c
    if(lo->lo_key_owner != current_uid() && !capable(CAP_SYS_ADMIN))
        return -EPERM;
#else
    if(lo->lo_key_owner != current->uid && !capable(CAP_SYS_ADMIN))
        return -EPERM;
#endif

    m = (AESmultiKey *)lo->key_data;
    if(!m) return -ENXIO;

#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
    /* temporarily prevent loop thread from messing with keys */
    write_lock(&m->rwlock);
    m->blocked = 1;
    /* make sure allocated keys are in original order */
    if(m->reversed) keyScrubWork(m);
    write_unlock(&m->rwlock);
#endif
    n = PAGE_SIZE / specialAligned_ctxSize;
    if(!n) n = 1;

    x = 0;
    while(x < KEY_ALLOC_COUNT) {
        if(!m->keyPtr[x]) {
            a = (aes_context *) specialAligned_kmalloc(specialAligned_ctxSize * n, GFP_KERNEL);
            if(!a) {
                err = -ENOMEM;
                goto error_out;
            }
            y = x;
            while((y < (x + n)) && (y < KEY_ALLOC_COUNT)) {
                m->keyPtr[y] = a;
                a = (aes_context *)((unsigned char *)a + specialAligned_ctxSize);
                y++;
            }
        }
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
        if(x >= 64) {
            x++;
            continue;
        }
#endif
        if(copy_from_user(&un.b[0], k, 32)) {
            err = -EFAULT;
            goto error_out;
        }
        aes_set_key(m->keyPtr[x], &un.b[0], lo->lo_encrypt_key_size, 0);
        k += 32;
        x++;
    }

    m->partialMD5[0] = 0x67452301;
    m->partialMD5[1] = 0xefcdab89;
    m->partialMD5[2] = 0x98badcfe;
    m->partialMD5[3] = 0x10325476;
    if(version3) {
        /* only first 128 bits of iv-key is used */
        if(copy_from_user(&un.b[0], k, 16)) {
            err = -EFAULT;
            goto error_out;
        }
#if defined(__BIG_ENDIAN)
        un.w[0] = cpu_to_le32(un.w[0]);
        un.w[1] = cpu_to_le32(un.w[1]);
        un.w[2] = cpu_to_le32(un.w[2]);
        un.w[3] = cpu_to_le32(un.w[3]);
#endif
        memset(&un.b[16], 0, 48);
        md5_transform_CPUbyteorder(&m->partialMD5[0], &un.w[0]);
        lo->lo_flags |= 0x080000;  /* multi-key-v3 (info exported to user space) */
    }

    m->keyMask = 0x3F;          /* range 0...63 */
    lo->lo_flags |= 0x100000;   /* multi-key (info exported to user space) */
    memset(&un.b[0], 0, 32);
error_out:
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
    /* re-enable loop thread key scrubbing */
    write_lock(&m->rwlock);
    m->blocked = 0;
    write_unlock(&m->rwlock);
#endif
    return err;
}

static int keySetup_aes(struct loop_device *lo, struct loop_info64 *info)
{
    AESmultiKey     *m;
    union {
        u_int32_t     w[8]; /* needed for 4 byte alignment for b[] */
        unsigned char b[32];
    } un;

    lo->key_data = m = allocMultiKey();
    if(!m) return(-ENOMEM);
    memcpy(&un.b[0], &info->lo_encrypt_key[0], 32);
    aes_set_key(m->keyPtr[0], &un.b[0], info->lo_encrypt_key_size, 0);
    memset(&info->lo_encrypt_key[0], 0, sizeof(info->lo_encrypt_key));
    memset(&un.b[0], 0, 32);
#if defined(CONFIG_BLK_DEV_LOOP_PADLOCK) && (defined(CONFIG_X86) || defined(CONFIG_X86_64))
    switch(info->lo_encrypt_key_size) {
    case 256:   /* bits */
    case 32:    /* bytes */
        /* 14 rounds, AES, software key gen, normal oper, encrypt, 256-bit key */
        m->padlock_cw_e = 14 | (1<<7) | (2<<10);
        /* 14 rounds, AES, software key gen, normal oper, decrypt, 256-bit key */
        m->padlock_cw_d = 14 | (1<<7) | (1<<9) | (2<<10);
        break;
    case 192:   /* bits */
    case 24:    /* bytes */
        /* 12 rounds, AES, software key gen, normal oper, encrypt, 192-bit key */
        m->padlock_cw_e = 12 | (1<<7) | (1<<10);
        /* 12 rounds, AES, software key gen, normal oper, decrypt, 192-bit key */
        m->padlock_cw_d = 12 | (1<<7) | (1<<9) | (1<<10);
        break;
    default:
        /* 10 rounds, AES, software key gen, normal oper, encrypt, 128-bit key */
        m->padlock_cw_e = 10 | (1<<7);
        /* 10 rounds, AES, software key gen, normal oper, decrypt, 128-bit key */
        m->padlock_cw_d = 10 | (1<<7) | (1<<9);
        break;
    }
#endif
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
#if LINUX_VERSION_CODE >= 0x40f00
    m->lo = lo;
#endif
    keyScrubTimerInit(lo);
#endif
    return(0);
}

static int keyClean_aes(struct loop_device *lo)
{
    if(lo->key_data) {
        clearAndFreeMultiKey((AESmultiKey *)lo->key_data);
        lo->key_data = 0;
    }
    return(0);
}

static int handleIoctl_aes(struct loop_device *lo, int cmd, unsigned long arg)
{
    int err;

    switch (cmd) {
    case LOOP_MULTI_KEY_SETUP:
        err = multiKeySetup(lo, (unsigned char *)arg, 0);
        break;
    case LOOP_MULTI_KEY_SETUP_V3:
        err = multiKeySetup(lo, (unsigned char *)arg, 1);
        break;
    default:
        err = -EINVAL;
    }
    return err;
}

void loop_compute_sector_iv(sector_t devSect, u_int32_t *ivout)
{
    if(sizeof(sector_t) == 8) {
        ivout[0] = cpu_to_le32(devSect);
        ivout[1] = cpu_to_le32((u_int64_t)devSect>>32);
        ivout[3] = ivout[2] = 0;
    } else {
        ivout[0] = cpu_to_le32(devSect);
        ivout[3] = ivout[2] = ivout[1] = 0;
    }
}

void loop_compute_md5_iv_v3(sector_t devSect, u_int32_t *ivout, u_int32_t *data)
{
    int         x;
#if defined(__BIG_ENDIAN)
    int         y, e;
#endif
    u_int32_t   buf[16];

#if defined(__BIG_ENDIAN)
    y = 7;
    e = 16;
    do {
        if (!y) {
            e = 12;
            /* md5_transform_CPUbyteorder wants data in CPU byte order */
            /* devSect is already in CPU byte order -- no need to convert */
            if(sizeof(sector_t) == 8) {
                /* use only 56 bits of sector number */
                buf[12] = devSect;
                buf[13] = (((u_int64_t)devSect >> 32) & 0xFFFFFF) | 0x80000000;
            } else {
                /* 32 bits of sector number + 24 zero bits */
                buf[12] = devSect;
                buf[13] = 0x80000000;
            }
            /* 4024 bits == 31 * 128 bit plaintext blocks + 56 bits of sector number */
            /* For version 3 on-disk format this really should be 4536 bits, but can't be */
            /* changed without breaking compatibility. V3 uses MD5-with-wrong-length IV */
            buf[14] = 4024;
            buf[15] = 0;
        }
        x = 0;
        do {
            buf[x    ] = cpu_to_le32(data[0]);
            buf[x + 1] = cpu_to_le32(data[1]);
            buf[x + 2] = cpu_to_le32(data[2]);
            buf[x + 3] = cpu_to_le32(data[3]);
            x += 4;
            data += 4;
        } while (x < e);
        md5_transform_CPUbyteorder(&ivout[0], &buf[0]);
    } while (--y >= 0);
    ivout[0] = cpu_to_le32(ivout[0]);
    ivout[1] = cpu_to_le32(ivout[1]);
    ivout[2] = cpu_to_le32(ivout[2]);
    ivout[3] = cpu_to_le32(ivout[3]);
#else
    x = 6;
    do {
        md5_transform_CPUbyteorder(&ivout[0], data);
        data += 16;
    } while (--x >= 0);
    memcpy(buf, data, 48);
    /* md5_transform_CPUbyteorder wants data in CPU byte order */
    /* devSect is already in CPU byte order -- no need to convert */
    if(sizeof(sector_t) == 8) {
        /* use only 56 bits of sector number */
        buf[12] = devSect;
        buf[13] = (((u_int64_t)devSect >> 32) & 0xFFFFFF) | 0x80000000;
    } else {
        /* 32 bits of sector number + 24 zero bits */
        buf[12] = devSect;
        buf[13] = 0x80000000;
    }
    /* 4024 bits == 31 * 128 bit plaintext blocks + 56 bits of sector number */
    /* For version 3 on-disk format this really should be 4536 bits, but can't be */
    /* changed without breaking compatibility. V3 uses MD5-with-wrong-length IV */
    buf[14] = 4024;
    buf[15] = 0;
    md5_transform_CPUbyteorder(&ivout[0], &buf[0]);
#endif
}

/* this function exists for compatibility with old external cipher modules */
void loop_compute_md5_iv(sector_t devSect, u_int32_t *ivout, u_int32_t *data)
{
    ivout[0] = 0x67452301;
    ivout[1] = 0xefcdab89;
    ivout[2] = 0x98badcfe;
    ivout[3] = 0x10325476;
    loop_compute_md5_iv_v3(devSect, ivout, data);
}

/* Some external modules do not know if md5_transform_CPUbyteorder() */
/* is asmlinkage or not, so here is C language wrapper for them. */
void md5_transform_CPUbyteorder_C(u_int32_t *hash, u_int32_t const *in)
{
    md5_transform_CPUbyteorder(hash, in);
}

#if defined(CONFIG_X86_64) && defined(AMD64_ASM)
# define HAVE_MD5_2X_IMPLEMENTATION  1
#endif
#if defined(HAVE_MD5_2X_IMPLEMENTATION)
/*
 * This 2x code is currently only available on little endian AMD64
 * This 2x code assumes little endian byte order
 * Context A input data is at zero offset, context B at data + 512 bytes
 * Context A ivout at zero offset, context B at ivout + 16 bytes
 */
void loop_compute_md5_iv_v3_2x(sector_t devSect, u_int32_t *ivout, u_int32_t *data)
{
    int         x;
    u_int32_t   buf[2*16];

    x = 6;
    do {
        md5_transform_CPUbyteorder_2x(&ivout[0], data, data + (512/4));
        data += 16;
    } while (--x >= 0);
    memcpy(&buf[0], data, 48);
    memcpy(&buf[16], data + (512/4), 48);
    /* md5_transform_CPUbyteorder wants data in CPU byte order */
    /* devSect is already in CPU byte order -- no need to convert */
    if(sizeof(sector_t) == 8) {
        /* use only 56 bits of sector number */
        buf[12] = devSect;
        buf[13] = (((u_int64_t)devSect >> 32) & 0xFFFFFF) | 0x80000000;
        buf[16 + 12] = ++devSect;
        buf[16 + 13] = (((u_int64_t)devSect >> 32) & 0xFFFFFF) | 0x80000000;
    } else {
        /* 32 bits of sector number + 24 zero bits */
        buf[12] = devSect;
        buf[16 + 13] = buf[13] = 0x80000000;
        buf[16 + 12] = ++devSect;
    }
    /* 4024 bits == 31 * 128 bit plaintext blocks + 56 bits of sector number */
    /* For version 3 on-disk format this really should be 4536 bits, but can't be */
    /* changed without breaking compatibility. V3 uses MD5-with-wrong-length IV */
    buf[16 + 14] = buf[14] = 4024;
    buf[16 + 15] = buf[15] = 0;
    md5_transform_CPUbyteorder_2x(&ivout[0], &buf[0], &buf[16]);
}
#endif /* defined(HAVE_MD5_2X_IMPLEMENTATION) */

/*
 * Special requirements for transfer functions:
 * (1) Plaintext data (loop_buf) may change while it is being read.
 * (2) On 2.2 and older kernels ciphertext buffer (raw_buf) may be doing
 *     writes to disk at any time, so it can't be used as temporary buffer.
 */
static int transfer_aes(struct loop_device *lo, int cmd, char *raw_buf,
          char *loop_buf, int size, sector_t devSect)
{
    aes_context     *a;
    AESmultiKey     *m;
    int             x;
    unsigned        y;
    u_int64_t       iv[4], *dip;

    if(!size || (size & 511)) {
        return -EINVAL;
    }
    m = (AESmultiKey *)lo->key_data;
    y = m->keyMask;
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
    read_lock(&m->rwlock);
#endif
    if(cmd == READ) {
#if defined(HAVE_MD5_2X_IMPLEMENTATION)
        /* if possible, use faster 2x MD5 implementation, currently AMD64 only (#6) */
        while((size >= (2*512)) && y) {
            /* multi-key mode, decrypt 2 sectors at a time */
            a = m->keyPtr[((unsigned)devSect    ) & y];
            /* decrypt using fake all-zero IV, first sector */
            memset(iv, 0, 16);
            x = 15;
            do {
                memcpy(&iv[2], raw_buf, 16);
                aes_decrypt(a, raw_buf, loop_buf);
                *((u_int64_t *)(&loop_buf[0])) ^= iv[0];
                *((u_int64_t *)(&loop_buf[8])) ^= iv[1];
                raw_buf += 16;
                loop_buf += 16;
                memcpy(iv, raw_buf, 16);
                aes_decrypt(a, raw_buf, loop_buf);
                *((u_int64_t *)(&loop_buf[0])) ^= iv[2];
                *((u_int64_t *)(&loop_buf[8])) ^= iv[3];
                raw_buf += 16;
                loop_buf += 16;
            } while(--x >= 0);
            a = m->keyPtr[((unsigned)devSect + 1) & y];
            /* decrypt using fake all-zero IV, second sector */
            memset(iv, 0, 16);
            x = 15;
            do {
                memcpy(&iv[2], raw_buf, 16);
                aes_decrypt(a, raw_buf, loop_buf);
                *((u_int64_t *)(&loop_buf[0])) ^= iv[0];
                *((u_int64_t *)(&loop_buf[8])) ^= iv[1];
                raw_buf += 16;
                loop_buf += 16;
                memcpy(iv, raw_buf, 16);
                aes_decrypt(a, raw_buf, loop_buf);
                *((u_int64_t *)(&loop_buf[0])) ^= iv[2];
                *((u_int64_t *)(&loop_buf[8])) ^= iv[3];
                raw_buf += 16;
                loop_buf += 16;
            } while(--x >= 0);
            /* compute correct IV */
            memcpy(&iv[0], &m->partialMD5[0], 16);
            memcpy(&iv[2], &m->partialMD5[0], 16);
            loop_compute_md5_iv_v3_2x(devSect, (u_int32_t *)iv, (u_int32_t *)(loop_buf - 1008));
            /* XOR with correct IV now */
            *((u_int64_t *)(loop_buf - 1024)) ^= iv[0];
            *((u_int64_t *)(loop_buf - 1016)) ^= iv[1];
            *((u_int64_t *)(loop_buf - 512)) ^= iv[2];
            *((u_int64_t *)(loop_buf - 504)) ^= iv[3];
            size -= 2*512;
            devSect += 2;
        }
#endif /* defined(HAVE_MD5_2X_IMPLEMENTATION) */
        while(size) {
            /* decrypt one sector at a time */
            a = m->keyPtr[((unsigned)devSect) & y];
            /* decrypt using fake all-zero IV */
            memset(iv, 0, 16);
            x = 15;
            do {
                memcpy(&iv[2], raw_buf, 16);
                aes_decrypt(a, raw_buf, loop_buf);
                *((u_int64_t *)(&loop_buf[0])) ^= iv[0];
                *((u_int64_t *)(&loop_buf[8])) ^= iv[1];
                raw_buf += 16;
                loop_buf += 16;
                memcpy(iv, raw_buf, 16);
                aes_decrypt(a, raw_buf, loop_buf);
                *((u_int64_t *)(&loop_buf[0])) ^= iv[2];
                *((u_int64_t *)(&loop_buf[8])) ^= iv[3];
                raw_buf += 16;
                loop_buf += 16;
            } while(--x >= 0);
            if(y) {
                /* multi-key mode, compute correct IV */
                memcpy(iv, &m->partialMD5[0], 16);
                loop_compute_md5_iv_v3(devSect, (u_int32_t *)iv, (u_int32_t *)(loop_buf - 496));
            } else {
                /* single-key mode, compute correct IV  */
                loop_compute_sector_iv(devSect, (u_int32_t *)iv);
            }
            /* XOR with correct IV now */
            *((u_int64_t *)(loop_buf - 512)) ^= iv[0];
            *((u_int64_t *)(loop_buf - 504)) ^= iv[1];
            size -= 512;
            devSect++;
        }
    } else {
#if defined(HAVE_MD5_2X_IMPLEMENTATION) && (LINUX_VERSION_CODE >= 0x20400)
        /* if possible, use faster 2x MD5 implementation, currently AMD64 only (#5) */
        while((size >= (2*512)) && y) {
            /* multi-key mode, encrypt 2 sectors at a time */
            memcpy(raw_buf, loop_buf, 2*512);
            memcpy(&iv[0], &m->partialMD5[0], 16);
            memcpy(&iv[2], &m->partialMD5[0], 16);
            loop_compute_md5_iv_v3_2x(devSect, (u_int32_t *)iv, (u_int32_t *)(&raw_buf[16]));
            /* first sector */
            a = m->keyPtr[((unsigned)devSect    ) & y];
            dip = &iv[0];
            x = 15;
            do {
                *((u_int64_t *)(&raw_buf[0])) ^= dip[0];
                *((u_int64_t *)(&raw_buf[8])) ^= dip[1];
                aes_encrypt(a, raw_buf, raw_buf);
                dip = (u_int64_t *)raw_buf;
                raw_buf += 16;
                *((u_int64_t *)(&raw_buf[0])) ^= dip[0];
                *((u_int64_t *)(&raw_buf[8])) ^= dip[1];
                aes_encrypt(a, raw_buf, raw_buf);
                dip = (u_int64_t *)raw_buf;
                raw_buf += 16;
            } while(--x >= 0);
            /* second sector */
            a = m->keyPtr[((unsigned)devSect + 1) & y];
            dip = &iv[2];
            x = 15;
            do {
                *((u_int64_t *)(&raw_buf[0])) ^= dip[0];
                *((u_int64_t *)(&raw_buf[8])) ^= dip[1];
                aes_encrypt(a, raw_buf, raw_buf);
                dip = (u_int64_t *)raw_buf;
                raw_buf += 16;
                *((u_int64_t *)(&raw_buf[0])) ^= dip[0];
                *((u_int64_t *)(&raw_buf[8])) ^= dip[1];
                aes_encrypt(a, raw_buf, raw_buf);
                dip = (u_int64_t *)raw_buf;
                raw_buf += 16;
            } while(--x >= 0);
            loop_buf += 2*512;
            size -= 2*512;
            devSect += 2;
        }
#endif /* defined(HAVE_MD5_2X_IMPLEMENTATION) && (LINUX_VERSION_CODE >= 0x20400) */
        while(size) {
            /* encrypt one sector at a time */
            a = m->keyPtr[((unsigned)devSect) & y];
            if(y) {
                /* multi-key mode encrypt, linux 2.4 and newer */
                memcpy(raw_buf, loop_buf, 512);
                memcpy(iv, &m->partialMD5[0], 16);
                loop_compute_md5_iv_v3(devSect, (u_int32_t *)iv, (u_int32_t *)(&raw_buf[16]));
                dip = iv;
                x = 15;
                do {
                    *((u_int64_t *)(&raw_buf[0])) ^= dip[0];
                    *((u_int64_t *)(&raw_buf[8])) ^= dip[1];
                    aes_encrypt(a, raw_buf, raw_buf);
                    dip = (u_int64_t *)raw_buf;
                    raw_buf += 16;
                    *((u_int64_t *)(&raw_buf[0])) ^= dip[0];
                    *((u_int64_t *)(&raw_buf[8])) ^= dip[1];
                    aes_encrypt(a, raw_buf, raw_buf);
                    dip = (u_int64_t *)raw_buf;
                    raw_buf += 16;
                } while(--x >= 0);
                loop_buf += 512;
            } else {
                /* single-key mode encrypt */
                loop_compute_sector_iv(devSect, (u_int32_t *)iv);
                dip = iv;
                x = 15;
                do {
                    iv[2] = *((u_int64_t *)(&loop_buf[0])) ^ dip[0];
                    iv[3] = *((u_int64_t *)(&loop_buf[8])) ^ dip[1];
                    aes_encrypt(a, (unsigned char *)(&iv[2]), raw_buf);
                    dip = (u_int64_t *)raw_buf;
                    loop_buf += 16;
                    raw_buf += 16;
                    iv[2] = *((u_int64_t *)(&loop_buf[0])) ^ dip[0];
                    iv[3] = *((u_int64_t *)(&loop_buf[8])) ^ dip[1];
                    aes_encrypt(a, (unsigned char *)(&iv[2]), raw_buf);
                    dip = (u_int64_t *)raw_buf;
                    loop_buf += 16;
                    raw_buf += 16;
                } while(--x >= 0);
            }
            size -= 512;
            devSect++;
        }
    }
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
    read_unlock(&m->rwlock);
#endif
    cond_resched();
    return(0);
}

#if defined(CONFIG_BLK_DEV_LOOP_PADLOCK) && (defined(CONFIG_X86) || defined(CONFIG_X86_64))
static __inline__ void padlock_flush_key_context(void)
{
    __asm__ __volatile__("pushf; popf" : : : "cc");
}

static __inline__ void padlock_rep_xcryptcbc(void *cw, void *k, void *s, void *d, void *iv, unsigned long cnt)
{
    __asm__ __volatile__(".byte 0xF3,0x0F,0xA7,0xD0"
                         : "+a" (iv), "+c" (cnt), "+S" (s), "+D" (d) /*output*/
                         : "b" (k), "d" (cw) /*input*/
                         : "cc", "memory" /*modified*/ );
}

typedef struct {
#if defined(HAVE_MD5_2X_IMPLEMENTATION)
    u_int64_t   iv[2*2];
#else
    u_int64_t   iv[2];
#endif
    u_int32_t   cw[4];
    u_int32_t   dummy1[4];
} Padlock_IV_CW;

static int transfer_padlock_aes(struct loop_device *lo, int cmd, char *raw_buf,
          char *loop_buf, int size, sector_t devSect)
{
    aes_context     *a;
    AESmultiKey     *m;
    unsigned        y;
    Padlock_IV_CW   ivcwua;
    Padlock_IV_CW   *ivcw;

    /* ivcw->iv and ivcw->cw must have 16 byte alignment */
    ivcw = (Padlock_IV_CW *)(((unsigned long)&ivcwua + 15) & ~((unsigned long)15));
    ivcw->cw[3] = ivcw->cw[2] = ivcw->cw[1] = 0;

    if(!size || (size & 511) || (((unsigned long)raw_buf | (unsigned long)loop_buf) & 15)) {
        return -EINVAL;
    }
    m = (AESmultiKey *)lo->key_data;
    y = m->keyMask;
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
    read_lock(&m->rwlock);
#endif
    if(cmd == READ) {
        ivcw->cw[0] = m->padlock_cw_d;
#if defined(HAVE_MD5_2X_IMPLEMENTATION)
        /* if possible, use faster 2x MD5 implementation, currently AMD64 only (#4) */
        while((size >= (2*512)) && y) {
            /* decrypt using fake all-zero IV */
            memset(&ivcw->iv[0], 0, 2*16);
            a = m->keyPtr[((unsigned)devSect    ) & y];
            padlock_flush_key_context();
            padlock_rep_xcryptcbc(&ivcw->cw[0], &a->aes_d_key[0], raw_buf, loop_buf, &ivcw->iv[0], 32);
            a = m->keyPtr[((unsigned)devSect + 1) & y];
            padlock_flush_key_context();
            padlock_rep_xcryptcbc(&ivcw->cw[0], &a->aes_d_key[0], raw_buf + 512, loop_buf + 512, &ivcw->iv[2], 32);
            /* compute correct IV */
            memcpy(&ivcw->iv[0], &m->partialMD5[0], 16);
            memcpy(&ivcw->iv[2], &m->partialMD5[0], 16);
            loop_compute_md5_iv_v3_2x(devSect, (u_int32_t *)(&ivcw->iv[0]), (u_int32_t *)(&loop_buf[16]));
            /* XOR with correct IV now */
            *((u_int64_t *)(&loop_buf[0])) ^= ivcw->iv[0];
            *((u_int64_t *)(&loop_buf[8])) ^= ivcw->iv[1];
            *((u_int64_t *)(&loop_buf[512 + 0])) ^= ivcw->iv[2];
            *((u_int64_t *)(&loop_buf[512 + 8])) ^= ivcw->iv[3];
            size -= 2*512;
            raw_buf += 2*512;
            loop_buf += 2*512;
            devSect += 2;
        }
#endif /* defined(HAVE_MD5_2X_IMPLEMENTATION) */
        while(size) {
            a = m->keyPtr[((unsigned)devSect) & y];
            padlock_flush_key_context();
            if(y) {
                /* decrypt using fake all-zero IV */
                memset(&ivcw->iv[0], 0, 16);
                padlock_rep_xcryptcbc(&ivcw->cw[0], &a->aes_d_key[0], raw_buf, loop_buf, &ivcw->iv[0], 32);
                /* compute correct IV */
                memcpy(&ivcw->iv[0], &m->partialMD5[0], 16);
                loop_compute_md5_iv_v3(devSect, (u_int32_t *)(&ivcw->iv[0]), (u_int32_t *)(&loop_buf[16]));
                /* XOR with correct IV now */
                *((u_int64_t *)(&loop_buf[ 0])) ^= ivcw->iv[0];
                *((u_int64_t *)(&loop_buf[ 8])) ^= ivcw->iv[1];
            } else {
                loop_compute_sector_iv(devSect, (u_int32_t *)(&ivcw->iv[0]));
                padlock_rep_xcryptcbc(&ivcw->cw[0], &a->aes_d_key[0], raw_buf, loop_buf, &ivcw->iv[0], 32);
            }
            size -= 512;
            raw_buf += 512;
            loop_buf += 512;
            devSect++;
        }
    } else {
        ivcw->cw[0] = m->padlock_cw_e;
#if defined(HAVE_MD5_2X_IMPLEMENTATION)
        /* if possible, use faster 2x MD5 implementation, currently AMD64 only (#3) */
        while((size >= (2*512)) && y) {
            memcpy(raw_buf, loop_buf, 2*512);
            memcpy(&ivcw->iv[0], &m->partialMD5[0], 16);
            memcpy(&ivcw->iv[2], &m->partialMD5[0], 16);
            loop_compute_md5_iv_v3_2x(devSect, (u_int32_t *)(&ivcw->iv[0]), (u_int32_t *)(&raw_buf[16]));
            a = m->keyPtr[((unsigned)devSect    ) & y];
            padlock_flush_key_context();
            padlock_rep_xcryptcbc(&ivcw->cw[0], &a->aes_e_key[0], raw_buf, raw_buf, &ivcw->iv[0], 32);
            a = m->keyPtr[((unsigned)devSect + 1) & y];
            padlock_flush_key_context();
            padlock_rep_xcryptcbc(&ivcw->cw[0], &a->aes_e_key[0], raw_buf + 512, raw_buf + 512, &ivcw->iv[2], 32);
            size -= 2*512;
            raw_buf += 2*512;
            loop_buf += 2*512;
            devSect += 2;
        }
#endif /* defined(HAVE_MD5_2X_IMPLEMENTATION) */
        while(size) {
            a = m->keyPtr[((unsigned)devSect) & y];
            padlock_flush_key_context();
            if(y) {
                memcpy(raw_buf, loop_buf, 512);
                memcpy(&ivcw->iv[0], &m->partialMD5[0], 16);
                loop_compute_md5_iv_v3(devSect, (u_int32_t *)(&ivcw->iv[0]), (u_int32_t *)(&raw_buf[16]));
                padlock_rep_xcryptcbc(&ivcw->cw[0], &a->aes_e_key[0], raw_buf, raw_buf, &ivcw->iv[0], 32);
            } else {
                loop_compute_sector_iv(devSect, (u_int32_t *)(&ivcw->iv[0]));
                padlock_rep_xcryptcbc(&ivcw->cw[0], &a->aes_e_key[0], loop_buf, raw_buf, &ivcw->iv[0], 32);
            }
            size -= 512;
            raw_buf += 512;
            loop_buf += 512;
            devSect++;
        }
    }
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
    read_unlock(&m->rwlock);
#endif
    cond_resched();
    return(0);
}
#endif

#if defined(CONFIG_BLK_DEV_LOOP_INTELAES) && (defined(CONFIG_X86) || defined(CONFIG_X86_64))
asmlinkage extern void intel_aes_cbc_encrypt(const aes_context *, void *src, void *dst, size_t len, void *iv);
asmlinkage extern void intel_aes_cbc_decrypt(const aes_context *, void *src, void *dst, size_t len, void *iv);
asmlinkage extern void intel_aes_cbc_enc_4x512(aes_context **, void *src, void *dst, void *iv);

static int transfer_intel_aes(struct loop_device *lo, int cmd, char *raw_buf,
          char *loop_buf, int size, sector_t devSect)
{
    aes_context     *acpa[4];
    AESmultiKey     *m;
    unsigned        y;
    u_int64_t       ivua[(4*2)+2];
    u_int64_t       *iv;

    /* make iv 16 byte aligned */
    iv = (u_int64_t *)(((unsigned long)&ivua + 15) & ~((unsigned long)15));

    if(!size || (size & 511) || (((unsigned long)raw_buf | (unsigned long)loop_buf) & 15)) {
        return -EINVAL;
    }
    m = (AESmultiKey *)lo->key_data;
    y = m->keyMask;
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
    read_lock(&m->rwlock);
#endif
    kernel_fpu_begin(); /* intel_aes_* code uses xmm registers */
    if(cmd == READ) {
#if defined(HAVE_MD5_2X_IMPLEMENTATION)
        /* if possible, use faster 2x MD5 implementation, currently AMD64 only (#2) */
        while((size >= (2*512)) && y) {
            acpa[0] = m->keyPtr[((unsigned)devSect    ) & y];
            acpa[1] = m->keyPtr[((unsigned)devSect + 1) & y];
            /* decrypt using fake all-zero IV */
            memset(iv, 0, 2*16);
            intel_aes_cbc_decrypt(acpa[0], raw_buf,       loop_buf,       512, &iv[0]);
            intel_aes_cbc_decrypt(acpa[1], raw_buf + 512, loop_buf + 512, 512, &iv[2]);
            /* compute correct IV, use 2x parallelized version */
            memcpy(&iv[0], &m->partialMD5[0], 16);
            memcpy(&iv[2], &m->partialMD5[0], 16);
            loop_compute_md5_iv_v3_2x(devSect, (u_int32_t *)iv, (u_int32_t *)(&loop_buf[16]));
            /* XOR with correct IV now */
            *((u_int64_t *)(&loop_buf[0])) ^= iv[0];
            *((u_int64_t *)(&loop_buf[8])) ^= iv[1];
            *((u_int64_t *)(&loop_buf[512 + 0])) ^= iv[2];
            *((u_int64_t *)(&loop_buf[512 + 8])) ^= iv[3];
            size -= 2*512;
            raw_buf += 2*512;
            loop_buf += 2*512;
            devSect += 2;
        }
#endif /* defined(HAVE_MD5_2X_IMPLEMENTATION) */
        while(size) {
            acpa[0] = m->keyPtr[((unsigned)devSect) & y];
            if(y) {
                /* decrypt using fake all-zero IV */
                memset(iv, 0, 16);
                intel_aes_cbc_decrypt(acpa[0], raw_buf, loop_buf, 512, iv);
                /* compute correct IV */
                memcpy(iv, &m->partialMD5[0], 16);
                loop_compute_md5_iv_v3(devSect, (u_int32_t *)iv, (u_int32_t *)(&loop_buf[16]));
                /* XOR with correct IV now */
                *((u_int64_t *)(&loop_buf[0])) ^= iv[0];
                *((u_int64_t *)(&loop_buf[8])) ^= iv[1];
            } else {
                loop_compute_sector_iv(devSect, (u_int32_t *)iv);
                intel_aes_cbc_decrypt(acpa[0], raw_buf, loop_buf, 512, iv);
            }
            size -= 512;
            raw_buf += 512;
            loop_buf += 512;
            devSect++;
        }
    } else {
        /* if possible, use faster 4-chains at a time encrypt implementation (#1) */
        while(size >= (4*512)) {
            acpa[0] = m->keyPtr[((unsigned)devSect    ) & y];
            acpa[1] = m->keyPtr[((unsigned)devSect + 1) & y];
            acpa[2] = m->keyPtr[((unsigned)devSect + 2) & y];
            acpa[3] = m->keyPtr[((unsigned)devSect + 3) & y];
            if(y) {
                memcpy(raw_buf, loop_buf, 4*512);
                memcpy(&iv[0], &m->partialMD5[0], 16);
                memcpy(&iv[2], &m->partialMD5[0], 16);
                memcpy(&iv[4], &m->partialMD5[0], 16);
                memcpy(&iv[6], &m->partialMD5[0], 16);
#if defined(HAVE_MD5_2X_IMPLEMENTATION)
                /* use 2x parallelized version */
                loop_compute_md5_iv_v3_2x(devSect,     (u_int32_t *)(&iv[0]), (u_int32_t *)(&raw_buf[        16]));
                loop_compute_md5_iv_v3_2x(devSect + 2, (u_int32_t *)(&iv[4]), (u_int32_t *)(&raw_buf[0x400 + 16]));
#else
                loop_compute_md5_iv_v3(devSect,     (u_int32_t *)(&iv[0]), (u_int32_t *)(&raw_buf[        16]));
                loop_compute_md5_iv_v3(devSect + 1, (u_int32_t *)(&iv[2]), (u_int32_t *)(&raw_buf[0x200 + 16]));
                loop_compute_md5_iv_v3(devSect + 2, (u_int32_t *)(&iv[4]), (u_int32_t *)(&raw_buf[0x400 + 16]));
                loop_compute_md5_iv_v3(devSect + 3, (u_int32_t *)(&iv[6]), (u_int32_t *)(&raw_buf[0x600 + 16]));
#endif
                intel_aes_cbc_enc_4x512(&acpa[0], raw_buf, raw_buf, iv);
            } else {
                loop_compute_sector_iv(devSect,     (u_int32_t *)(&iv[0]));
                loop_compute_sector_iv(devSect + 1, (u_int32_t *)(&iv[2]));
                loop_compute_sector_iv(devSect + 2, (u_int32_t *)(&iv[4]));
                loop_compute_sector_iv(devSect + 3, (u_int32_t *)(&iv[6]));
                intel_aes_cbc_enc_4x512(&acpa[0], loop_buf, raw_buf, iv);
            }
            size -= 4*512;
            raw_buf += 4*512;
            loop_buf += 4*512;
            devSect += 4;
        }
        /* encrypt the rest (if any) using slower 1-chain at a time implementation */
        while(size) {
            acpa[0] = m->keyPtr[((unsigned)devSect) & y];
            if(y) {
                memcpy(raw_buf, loop_buf, 512);
                memcpy(iv, &m->partialMD5[0], 16);
                loop_compute_md5_iv_v3(devSect, (u_int32_t *)iv, (u_int32_t *)(&raw_buf[16]));
                intel_aes_cbc_encrypt(acpa[0], raw_buf, raw_buf, 512, iv);
            } else {
                loop_compute_sector_iv(devSect, (u_int32_t *)iv);
                intel_aes_cbc_encrypt(acpa[0], loop_buf, raw_buf, 512, iv);
            }
            size -= 512;
            raw_buf += 512;
            loop_buf += 512;
            devSect++;
        }
    }
    kernel_fpu_end(); /* intel_aes_* code uses xmm registers */
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
    read_unlock(&m->rwlock);
#endif
    cond_resched();
    return(0);
}
#endif

static struct loop_func_table funcs_aes = {
    number:     16,     /* 16 == AES */
    transfer:   transfer_aes,
    init:       keySetup_aes,
    release:    keyClean_aes,
    ioctl:      handleIoctl_aes
};

#if defined(CONFIG_BLK_DEV_LOOP_PADLOCK) && (defined(CONFIG_X86) || defined(CONFIG_X86_64))
static struct loop_func_table funcs_padlock_aes = {
    number:     16,     /* 16 == AES */
    transfer:   transfer_padlock_aes,
    init:       keySetup_aes,
    release:    keyClean_aes,
    ioctl:      handleIoctl_aes
};
#endif

#if defined(CONFIG_BLK_DEV_LOOP_INTELAES) && (defined(CONFIG_X86) || defined(CONFIG_X86_64))
static struct loop_func_table funcs_intel_aes = {
    number:     16,     /* 16 == AES */
    transfer:   transfer_intel_aes,
    init:       keySetup_aes,
    release:    keyClean_aes,
    ioctl:      handleIoctl_aes
};
#endif

#if defined(CONFIG_BLK_DEV_LOOP_PADLOCK) && (defined(CONFIG_X86) || defined(CONFIG_X86_64))
static int CentaurHauls_ID_and_enabled_ACE(void)
{
    unsigned int eax = 0, ebx = 0, ecx = 0, edx = 0;

    /* check for "CentaurHauls" ID string, and enabled ACE */
    cpuid(0x00000000, &eax, &ebx, &ecx, &edx);
    if((ebx == 0x746e6543) && (edx == 0x48727561) && (ecx == 0x736c7561)
      && (cpuid_eax(0xC0000000) >= 0xC0000001)
      && ((cpuid_edx(0xC0000001) & 0xC0) == 0xC0)) {
        return 1;   /* ACE enabled */
    }
    return 0;
}
#endif

EXPORT_SYMBOL(loop_compute_sector_iv);
EXPORT_SYMBOL(loop_compute_md5_iv_v3);
EXPORT_SYMBOL(loop_compute_md5_iv);
EXPORT_SYMBOL(md5_transform_CPUbyteorder_C);
#endif /* CONFIG_BLK_DEV_LOOP_AES */

/* xfer_funcs[0] is special - its release function is never called */
static struct loop_func_table *xfer_funcs[MAX_LO_CRYPT] = {
	&none_funcs,
	&xor_funcs,
#ifdef CONFIG_BLK_DEV_LOOP_AES
        [LO_CRYPT_AES] = &funcs_aes,
#endif
};

/*
 *  First number of 'lo_prealloc' is the default number of RAM pages
 *  to pre-allocate for each device backed loop. Every (configured)
 *  device backed loop pre-allocates this amount of RAM pages unless
 *  later 'lo_prealloc' numbers provide an override. 'lo_prealloc'
 *  overrides are defined in pairs: loop_index,number_of_pages
 */
static int lo_prealloc[9] = { 256, -1, 0, -1, 0, -1, 0, -1, 0 };
#define LO_PREALLOC_MIN 4    /* minimum user defined pre-allocated RAM pages */
#define LO_PREALLOC_MAX 4096 /* maximum user defined pre-allocated RAM pages */

#ifdef MODULE
static int dummy1;
module_param_array(lo_prealloc, int, &dummy1, 0);
MODULE_PARM_DESC(lo_prealloc, "Number of pre-allocated pages [,index,pages]...");
#else
static int __init lo_prealloc_setup(char *str)
{
	int x, y, z;

	for (x = 0; x < (sizeof(lo_prealloc) / sizeof(int)); x++) {
		z = get_option(&str, &y);
		if (z > 0)
			lo_prealloc[x] = y;
		if (z < 2)
			break;
	}
	return 1;
}
__setup("lo_prealloc=", lo_prealloc_setup);
#endif

/*
 *  First number of 'lo_threads' is the default number of helper threads to
 *  create for each device backed loop device. Every (configured) device
 *  backed loop device has this many threads unless later 'lo_threads'
 *  numbers provide an override. File backed loops always have 1 helper
 *  thread. 'lo_threads' overrides are defined in pairs: loop_index,threads
 *
 *  This value is ignored on 2.6.18 and older kernels.
 */
static int lo_threads[9] = { 1, -1, 0, -1, 0, -1, 0, -1, 0 };
#define LO_THREADS_MIN 1    /* minimum user defined thread count */
#define LO_THREADS_MAX 4    /* maximum user defined thread count */

#ifdef MODULE
static int dummy2;
module_param_array(lo_threads, int, &dummy2, 0);
MODULE_PARM_DESC(lo_threads, "Number of threads per loop [,index,threads]...");
#else
static int __init lo_threads_setup(char *str)
{
	int x, y, z;

	for (x = 0; x < (sizeof(lo_threads) / sizeof(int)); x++) {
		z = get_option(&str, &y);
		if (z > 0)
			lo_threads[x] = y;
		if (z < 2)
			break;
	}
	return 1;
}
__setup("lo_threads=", lo_threads_setup);
#endif

/*
 * This is loop helper thread nice value in range
 * from 0 (low priority) to -20 (high priority).
 */
static int lo_nice = -1;

#ifdef MODULE
module_param(lo_nice, int, 0);
MODULE_PARM_DESC(lo_nice, "Loop thread scheduler nice (0 ... -20)");
#else
static int __init lo_nice_setup(char *str)
{
	int y;

	if (get_option(&str, &y) == 1)
		lo_nice = y;
	return 1;
}
__setup("lo_nice=", lo_nice_setup);
#endif

struct loop_bio_extension {
	struct bio		*bioext_merge;
	struct loop_device	*bioext_loop;
	struct bio_vec          *bioext_bi_io_vec_orig;
	sector_t		bioext_iv;
	int			bioext_index;
	int			bioext_size;
	unsigned int            bioext_bi_max_vecs_orig;
	unsigned int		bioext_done_offset;
};	

#define LOOP_COMPAT_BI_SECTOR  bi_iter.bi_sector
#define LOOP_COMPAT_BI_SIZE    bi_iter.bi_size
#define LOOP_COMPAT_BI_IDX     bi_iter.bi_idx
#define LOOP_COMPAT_BI_CNT     __bi_cnt
#define LOOP_COMPAT_BIO_ENDIO(a,b)  do { (a)->bi_status = (blk_status_t)(b); bio_endio(a); } while(0)
#define LOOP_COMPAT_BLK_STS_IOERR     BLK_STS_IOERR
#define LOOP_COMPAT_OPF  bi_opf

static struct loop_device **loop_dev_ptr_arr;

static void loop_prealloc_cleanup(struct loop_device *lo)
{
	struct bio *bio;
	struct loop_bio_extension *extension;

	while ((bio = lo->lo_bio_free0)) {
		lo->lo_bio_free0 = bio->bi_next;
		extension = bio->bi_private;
		bio->bi_io_vec = extension->bioext_bi_io_vec_orig;
		bio->bi_max_vecs = extension->bioext_bi_max_vecs_orig;
		bio->bi_vcnt = 1;
		__free_page(bio->bi_io_vec[0].bv_page);
		kfree(extension);
		bio->bi_next = NULL;
		bio_put(bio);
	}
	while ((bio = lo->lo_bio_free1)) {
		lo->lo_bio_free1 = bio->bi_next;
		/* bi_opf was used for other purpose */
		bio->bi_opf = 0;
		/* bi_size was used for other purpose */
		bio->LOOP_COMPAT_BI_SIZE = 0;
		/* bi_cnt was used for other purpose */
		atomic_set(&bio->LOOP_COMPAT_BI_CNT, 1);
		bio->bi_next = NULL;
		bio_put(bio);
	}
}

static int loop_prealloc_init(struct loop_device *lo, int y, int *nrqp)
{
	struct bio *bio;
	struct loop_bio_extension *extension;
	int x, z = 0;

	if(!y) {
		y = lo_prealloc[0];
		for (x = 1; x < (sizeof(lo_prealloc) / sizeof(int)); x += 2) {
			if (lo_prealloc[x + 1] && (lo->lo_number == lo_prealloc[x])) {
				y = lo_prealloc[x + 1];
				break;
			}
		}
		z = y;
	}
	if(z < 128) z = 128;
	*nrqp = z;
	z = (y * 3) / 4;
	if(z > 127) z = 127;
	lo->lo_bio_flshMax = z;
	lo->lo_bio_flshCnt = 0;

	for (x = 0; x < y; x++) {
		bio = bio_alloc(NULL, 1, 0, GFP_KERNEL);
		if (!bio) {
			fail1:
			loop_prealloc_cleanup(lo);
			return 1;
		}
		bio->bi_io_vec[0].bv_page = alloc_page(GFP_KERNEL);
		if (!bio->bi_io_vec[0].bv_page) {
			fail2:
			bio->bi_next = NULL;
			bio_put(bio);
			goto fail1;
		}
		memset(page_address(bio->bi_io_vec[0].bv_page), 0, PAGE_SIZE);
		bio->bi_vcnt = 1;
		extension = kmalloc(sizeof(struct loop_bio_extension), GFP_KERNEL);
		if (!extension) {
			__free_page(bio->bi_io_vec[0].bv_page);
			goto fail2;
		}
		bio->bi_private = extension;
		extension->bioext_bi_io_vec_orig = bio->bi_io_vec;
		extension->bioext_bi_max_vecs_orig = bio->bi_max_vecs;
		bio->bi_next = lo->lo_bio_free0;
		lo->lo_bio_free0 = bio;

		bio = bio_alloc(NULL, 1, 0, GFP_KERNEL);
		if (!bio)
			goto fail1;
		bio->bi_vcnt = 1;
		bio->bi_next = lo->lo_bio_free1;
		lo->lo_bio_free1 = bio;
	}
	return 0;
}

static void loop_add_queue_last(struct loop_device *lo, struct bio *bio, struct bio **q)
{
	unsigned long flags;

	spin_lock_irqsave(&lo->lo_lock, flags);
	if (*q) {
		bio->bi_next = (*q)->bi_next;
		(*q)->bi_next = bio;
	} else {
		bio->bi_next = bio;
	}
	*q = bio;
	spin_unlock_irqrestore(&lo->lo_lock, flags);

	if (waitqueue_active(&lo->lo_bio_wait))
		wake_up_interruptible_all(&lo->lo_bio_wait);
}

static struct bio *loop_get_bio(struct loop_device *lo)
{
	struct bio *bio = NULL, *last;

	spin_lock_irq(&lo->lo_lock);
	if ((last = lo->lo_bio_que0)) {
		bio = last->bi_next;
		if (bio == last)
			lo->lo_bio_que0 = NULL;
		else
			last->bi_next = bio->bi_next;
		bio->bi_next = NULL;
	}
	spin_unlock_irq(&lo->lo_lock);
	return bio;
}

static void loop_put_buffer(struct loop_device *lo, struct bio *b, int flist)
{
	unsigned long flags;

	spin_lock_irqsave(&lo->lo_lock, flags);
	if(!flist) {
		b->bi_next = lo->lo_bio_free0;
		lo->lo_bio_free0 = b;
	} else {
		b->bi_next = lo->lo_bio_free1;
		lo->lo_bio_free1 = b;
	}
	spin_unlock_irqrestore(&lo->lo_lock, flags);

	if (waitqueue_active(&lo->lo_buf_wait))
		wake_up_all(&lo->lo_buf_wait);
}

static void loop_end_io_transfer(struct bio *bio)
{
	struct loop_bio_extension *extension = bio->bi_private;
	struct bio *merge = extension->bioext_merge;
	struct loop_device *lo = extension->bioext_loop;
	struct bio *origbio = merge->bi_private;
	int err = (int)bio->bi_status;

	if (err) {
		merge->LOOP_COMPAT_BI_SIZE = err; /* used as error code */
		if(err == LOOP_COMPAT_BLK_STS_IOERR)
			merge->bi_opf = 0;
		smp_wmb();
		printk(KERN_ERR "loop%d: loop_end_io_transfer err=%d opf=0x%x sec=%llu len=%d\n", lo->lo_number, err,
			bio->bi_opf, (unsigned long long)extension->bioext_iv, extension->bioext_size);
	}
	if (bio_data_dir(bio) == WRITE) {
		loop_put_buffer(lo, bio, 0);
		if (!atomic_dec_and_test(&merge->LOOP_COMPAT_BI_CNT)) {
			return;
		}
		origbio->bi_next = NULL;
		smp_rmb();
		LOOP_COMPAT_BIO_ENDIO(origbio, merge->bi_opf ? (int)merge->LOOP_COMPAT_BI_SIZE : LOOP_COMPAT_BLK_STS_IOERR);
		loop_put_buffer(lo, merge, 1);
		if (atomic_dec_and_test(&lo->lo_pending))
			wake_up_interruptible_all(&lo->lo_bio_wait);
	} else {
		loop_add_queue_last(lo, bio, &lo->lo_bio_que0);
	}
}

static struct bio *loop_get_buffer(struct loop_device *lo, struct bio *orig_bio,
		struct bio **merge_ptr, int *flushPtr, int origHasData, int *lastVePtr, unsigned long *nfp)
{
	struct bio *bio = NULL, *merge = *merge_ptr, *fbtst;
	struct loop_bio_extension *extension;
	int nzCnt, flsh = 0, firstVec, lastVec, doAdvIdx = 1;
	unsigned int len;

	spin_lock_irq(&lo->lo_lock);
	if (!merge) {
		merge = lo->lo_bio_free1;
		if (merge) {
			lo->lo_bio_free1 = merge->bi_next;
		}
	}
	if (merge) {
		bio = lo->lo_bio_free0;
		if (bio) {
			lo->lo_bio_free0 = bio->bi_next;
		}
	}
	fbtst = lo->lo_bio_free0;
	if(!fbtst || !fbtst->bi_next) {
		flsh = 1;
	}
	fbtst = lo->lo_bio_free1;
	if(!fbtst || !fbtst->bi_next) {
		flsh = 1;
	}
	spin_unlock_irq(&lo->lo_lock);

	*flushPtr = flsh;

	if (!(*merge_ptr) && merge) {
		/*
		 * initialize "merge-bio" which is used as
		 * rendezvous point among multiple vecs
		 */
		*merge_ptr = merge;
		merge->LOOP_COMPAT_BI_SECTOR = orig_bio->LOOP_COMPAT_BI_SECTOR + lo->lo_offs_sec;
		merge->LOOP_COMPAT_BI_SIZE = 0; /* used as error code */
		merge->bi_opf = 1;		/* used as 1=OK 0=error */
		merge->bi_ioprio = 1;		/* used as first vec flag */
		smp_wmb();
		merge->LOOP_COMPAT_BI_IDX = orig_bio->LOOP_COMPAT_BI_IDX;
		nzCnt = 1;
		if(origHasData) {
			/* compute total number of vecs that this driver must process */
			unsigned int orIdx = orig_bio->LOOP_COMPAT_BI_IDX;
			unsigned int orSiz = orig_bio->LOOP_COMPAT_BI_SIZE;
			unsigned int orDon = orig_bio->bi_iter.bi_bvec_done;
			unsigned int vecRem;
			do {
				vecRem = orig_bio->bi_io_vec[orIdx].bv_len - orDon;
				while((vecRem > PAGE_SIZE) && (orSiz > PAGE_SIZE)) {
					vecRem -= PAGE_SIZE;
					orSiz -= PAGE_SIZE;
					nzCnt++;
				}
				if(orSiz <= vecRem)
					break;
				if(vecRem) {
					orSiz -= vecRem;
					nzCnt++;
				}
				orIdx++;
				orDon = 0;
			} while(1);
		}
		atomic_set(&merge->LOOP_COMPAT_BI_CNT, nzCnt);
		merge->bi_private = orig_bio;
	}

	if (!bio)
		return NULL;

	extension = bio->bi_private;
	if(origHasData) {
		try_next_vec:
		len = orig_bio->bi_io_vec[merge->LOOP_COMPAT_BI_IDX].bv_len - orig_bio->bi_iter.bi_bvec_done;
		if(!len) {
			/* badly formatted request detected */
			merge->LOOP_COMPAT_BI_IDX++;
			orig_bio->bi_iter.bi_bvec_done = 0;
			goto try_next_vec;
		}
		firstVec = merge->bi_ioprio;
		lastVec = ((orig_bio->LOOP_COMPAT_BI_SIZE <= len) && (len <= PAGE_SIZE)) ? 1 : 0;
	} else {
		len = 0;
		firstVec = 1;
		lastVec = 1;
	}
	*lastVePtr = lastVec;

	/*
	 * initialize one page "buffer-bio"
	 */
	bio->bi_io_vec = extension->bioext_bi_io_vec_orig;
	bio->bi_max_vecs = extension->bioext_bi_max_vecs_orig;
	bio->bi_vcnt = 1;
	bio_reset(bio, NULL, 0);
	bio->bi_cookie = BLK_QC_T_NONE;
	bio->bi_private = extension;
	bio->LOOP_COMPAT_BI_SECTOR = merge->LOOP_COMPAT_BI_SECTOR;
	bio->bi_next = NULL;
	if(lo->lo_device) bio_set_dev(bio, lo->lo_device);

	/* read-ahead bit needs to be cleared to work around kernel bug */
	/* that causes I/O errors on -EWOULDBLOCK I/O elevator failures */
	*nfp = orig_bio->LOOP_COMPAT_OPF & (REQ_PREFLUSH | REQ_FUA | REQ_SYNC | REQ_IDLE);

	if(orig_bio->LOOP_COMPAT_OPF & REQ_PREFLUSH) {
		if(!firstVec) {
			*nfp &= ~REQ_PREFLUSH;
		} else {
			*flushPtr = 1;
		}
	}
	if(orig_bio->LOOP_COMPAT_OPF & REQ_FUA) {
		if(!lastVec) {
			*nfp &= ~REQ_FUA;
		} else {
			*flushPtr = 1;
		}
	}
	if(orig_bio->LOOP_COMPAT_OPF & REQ_SYNC) {
		if(!lastVec) {
			*nfp &= ~REQ_SYNC;
		} else {
			*flushPtr = 1;
		}
	}
	if(!(orig_bio->LOOP_COMPAT_OPF & REQ_IDLE)) {
		if(!lastVec) {
			*nfp |= REQ_IDLE;
		}
	}
	if(flsh) {
		*nfp &= ~REQ_IDLE;
	}

	bio->LOOP_COMPAT_BI_IDX = 0;
	if(origHasData) {
		/* original bio has data */
		if(len > orig_bio->LOOP_COMPAT_BI_SIZE)
			len = orig_bio->LOOP_COMPAT_BI_SIZE;
		if(len > PAGE_SIZE) {
			len = PAGE_SIZE;
			doAdvIdx = 0;
		}
		bio->LOOP_COMPAT_BI_SIZE = len;
		extension->bioext_done_offset = orig_bio->bi_iter.bi_bvec_done;
		bio->bi_io_vec[0].bv_len = len;
		bio->bi_io_vec[0].bv_offset = 0;
	} else {
		/* original bio does not have data */
		bio->bi_io_vec = 0;
		bio->bi_max_vecs = 0;
		bio->bi_vcnt = 0;
		extension->bioext_done_offset = 0;
		bio->LOOP_COMPAT_BI_SIZE = 0;
		bio->LOOP_COMPAT_BI_SECTOR = 0;
	}

	bio->bi_end_io = loop_end_io_transfer;

	/*
	 * initialize "buffer-bio" extension. This extension is
	 * permanently glued to above "buffer-bio" via bio->bi_private
	 */
	extension->bioext_merge = merge;
	extension->bioext_loop = lo;
	extension->bioext_iv = merge->LOOP_COMPAT_BI_SECTOR - lo->lo_iv_remove;
	extension->bioext_index = merge->LOOP_COMPAT_BI_IDX;
	extension->bioext_size = len;

	/*
	 * prepare "merge-bio" for next vec
	 */
	merge->bi_ioprio = 0; /* next is not first vec */
	merge->LOOP_COMPAT_BI_SECTOR += len >> 9;
	if(doAdvIdx) {
		merge->LOOP_COMPAT_BI_IDX++;
		orig_bio->bi_iter.bi_bvec_done = 0;
	} else {
		orig_bio->bi_iter.bi_bvec_done += PAGE_SIZE;
	}
	orig_bio->LOOP_COMPAT_BI_SIZE -= len;
	bio->bi_iter.bi_bvec_done = 0;
	return bio;
}

static int figure_loop_size(struct loop_device *lo, struct block_device *bdev)
{
	loff_t size, offs;
	sector_t x;
	int err = 0;

	size = i_size_read(lo->lo_backing_file->f_mapping->host);
	offs = lo->lo_offset;
	if (!(lo->lo_flags & LO_FLAGS_DO_BMAP))
		offs &= ~((loff_t)511);
	if ((offs > 0) && (offs < size)) {
		size -= offs;
	} else {
		if (offs)
			err = -EINVAL;
		lo->lo_offset = 0;
		lo->lo_offs_sec = lo->lo_iv_remove = 0;
	}
	if ((lo->lo_sizelimit > 0) && (lo->lo_sizelimit <= size)) {
		size = lo->lo_sizelimit;
	} else {
		if (lo->lo_sizelimit)
			err = -EINVAL;
		lo->lo_sizelimit = 0;
	}
	size >>= 9;

	/*
	 * Unfortunately, if we want to do I/O on the device,
	 * the number of 512-byte sectors has to fit into a sector_t.
	 */
	x = (sector_t)size;
	if ((loff_t)x != size) {
		err = -EFBIG;
		size = 0;
	}

	set_capacity(disks[lo->lo_number], size);	/* 512 byte units */
	i_size_write(bdev->bd_inode, size << 9);	/* byte units */
	return err;
}

static inline int lo_do_transfer(struct loop_device *lo, int cmd, char *rbuf,
				 char *lbuf, int size, sector_t rblock)
{
	if (!lo->transfer)
		return 0;

	return lo->transfer(lo, cmd, rbuf, lbuf, size, rblock);
}

static int loop_file_io(struct file *file, char *buf, int size, loff_t *ppos, int w)
{
	int x, y, z;

	y = 0;
	do {
		z = size - y;
		if (w) {
			x = kernel_write(file, buf + y, z, ppos);
		} else {
			x = kernel_read(file, buf + y, z, ppos);
			if (!x)
				return 1;
		}
		if (x < 0) {
			if ((x == -EAGAIN) || (x == -ENOMEM) || (x == -ERESTART) || (x == -EINTR)) {
				set_current_state(TASK_INTERRUPTIBLE);
				schedule_timeout(HZ / 2);
				continue;
			}
			return 1;
		}
		y += x;
	} while (y < size);
	return 0;
}

static int do_bio_filebacked(struct loop_device *lo, struct bio *bio)
{
	loff_t pos;
	struct file *file = lo->lo_backing_file;
	char *data, *buf;
	unsigned int size, len, more;
	sector_t IV;
	struct page *pg;
	unsigned int vecPGoffs;
	unsigned int oln;

	if(!bio_has_data(bio))
		return 0;

	pos = ((loff_t) bio->LOOP_COMPAT_BI_SECTOR << 9) + lo->lo_offset;
	buf = page_address(lo->lo_bio_free0->bi_io_vec[0].bv_page);
	IV = bio->LOOP_COMPAT_BI_SECTOR;
	if (!lo->lo_iv_remove)
		IV += lo->lo_offs_sec;
	do {
		pg = bio->bi_io_vec[bio->LOOP_COMPAT_BI_IDX].bv_page;
		len = bio->bi_io_vec[bio->LOOP_COMPAT_BI_IDX].bv_len - bio->bi_iter.bi_bvec_done;
		if(len > bio->LOOP_COMPAT_BI_SIZE)
			len = bio->LOOP_COMPAT_BI_SIZE;
		oln = len;
		vecPGoffs = bio->bi_io_vec[bio->LOOP_COMPAT_BI_IDX].bv_offset + bio->bi_iter.bi_bvec_done;
		bio->bi_iter.bi_bvec_done = 0;
		pg += vecPGoffs / PAGE_SIZE;
		vecPGoffs %= PAGE_SIZE;
		while (len > 0) {
			data = kmap(pg) + vecPGoffs;
			size = len;
			if((size + vecPGoffs) > PAGE_SIZE)
				size = PAGE_SIZE - vecPGoffs;
			vecPGoffs = 0;
			if (!lo->lo_encryption) {
				/* this code relies that NONE transfer is a no-op */
				buf = data;
			}
			if (bio_data_dir(bio) == WRITE) {
				if (lo_do_transfer(lo, WRITE, buf, data, size, IV)) {
					printk(KERN_ERR "loop%d: write transfer error, sector %llu\n", lo->lo_number, (unsigned long long)IV);
					goto kunmap_and_out;
				}
				if (loop_file_io(file, buf, size, &pos, 1)) {
					printk(KERN_ERR "loop%d: write i/o error, sector %llu\n", lo->lo_number, (unsigned long long)IV);
					goto kunmap_and_out;
				}
			} else {
				if (loop_file_io(file, buf, size, &pos, 0)) {
					printk(KERN_ERR "loop%d: read i/o error, sector %llu\n", lo->lo_number, (unsigned long long)IV);
					goto kunmap_and_out;
				}
				if (lo_do_transfer(lo, READ, buf, data, size, IV)) {
					printk(KERN_ERR "loop%d: read transfer error, sector %llu\n", lo->lo_number, (unsigned long long)IV);
					goto kunmap_and_out;
				}
				flush_dcache_page(pg);
			}
			len -= size;
			IV += size >> 9;
			kunmap(pg);
			pg++;
		}
		bio->LOOP_COMPAT_BI_IDX++;
		more = ((bio->LOOP_COMPAT_BI_SIZE -= oln) > 0) ? 1 : 0;
	} while (more);
	return 0;

kunmap_and_out:
	kunmap(pg);
	return LOOP_COMPAT_BLK_STS_IOERR;
}

static void loop_unplug_backingdev(struct request_queue *bq)
{
	struct blk_plug *plug = current->plug;
	if(plug) {
		/* A thread may sleep and wait for new buffers from previously submitted requests. */
		/* Make sure requests are actually sent to backing device, and not just queued. */
		struct bio_list *blistTmp = current->bio_list;
		current->bio_list = NULL;
		blk_finish_plug(plug);	/* clears current->plug */
		current->bio_list = blistTmp;
		blk_start_plug(plug); 	/* sets current->plug */
	}
}

/* this is never called, but address of this function is special cased in some places */
static void loop_end_io_change_fd_request(struct bio *bio)
{
}

static void loop_submit_bio_real(struct bio *old_bio)
{
	struct bio *new_bio, *merge;
	struct loop_device *lo;
	struct loop_bio_extension *extension;
	int rw = bio_data_dir(old_bio), y, x, flushFlag = 0, origHasData, lastVec = 0;
	char *md;
	wait_queue_entry_t waitq;
	unsigned long new_f;
	unsigned int old_op = bio_op(old_bio);

	if(unlikely(old_bio->bi_end_io == loop_end_io_change_fd_request)) {
		/* change fd request */
		lo = (struct loop_device *)old_bio->bi_bdev;
	} else {
		/* normal bio request */
		lo = old_bio->bi_bdev->bd_disk->private_data;
	}
	if(unlikely(!lo))
		goto out;
	if(unlikely(!(lo->lo_flags & LO_FLAGS_INITIALIZED)))
		goto out;

	if(unlikely((old_op == REQ_OP_DISCARD)
		 || (old_op == REQ_OP_SECURE_ERASE)
		 || (old_op == REQ_OP_WRITE_ZEROES))) {
		old_bio->bi_next = NULL;
		old_bio->bi_status = BLK_STS_NOTSUPP;
		bio_endio(old_bio);
		return;
	}

	set_current_state(TASK_RUNNING);
	origHasData = bio_has_data(old_bio);
	if ((rw == WRITE) && (lo->lo_flags & LO_FLAGS_READ_ONLY))
		goto out;
	atomic_inc(&lo->lo_pending);

	/*
	 * file backed, queue for loop_thread to handle
	 */
	if (lo->lo_flags & LO_FLAGS_DO_BMAP) {
		loop_add_queue_last(lo, old_bio, &lo->lo_bio_que0);
		return;
	}

	/*
	 * device backed, just remap bdev & sector for NONE transfer
	 */
	if (!lo->lo_encryption) {
		old_bio->LOOP_COMPAT_BI_SECTOR += lo->lo_offs_sec;
		bio_set_dev(old_bio, lo->lo_device);
		submit_bio_noacct(old_bio);
		if (atomic_dec_and_test(&lo->lo_pending))
			wake_up_interruptible_all(&lo->lo_bio_wait);
		return;
	}

	/*
	 * device backed, start reads and writes now if buffer available
	 */
	merge = NULL;
	init_waitqueue_entry(&waitq, current);
	try_next_old_bio_vec:
	new_f = 0;
	new_bio = loop_get_buffer(lo, old_bio, &merge, &flushFlag, origHasData, &lastVec, &new_f);
	if (!new_bio) {
		/* wait for buffer to be freed, and try again */
		spin_lock_irq(&lo->lo_lock);
		lo->lo_bio_flshCnt = 0;
		spin_unlock_irq(&lo->lo_lock);
 		loop_unplug_backingdev(lo->lo_backingQueue);
		add_wait_queue(&lo->lo_buf_wait, &waitq);
		for (;;) {
			set_current_state(TASK_UNINTERRUPTIBLE);
			x = 0;
			spin_lock_irq(&lo->lo_lock);
			if (!merge && lo->lo_bio_free1) {
				/* don't sleep if merge bio is available */
				x = 1;
			}
			if (merge && lo->lo_bio_free0) {
				/* don't sleep if buffer bio is available */
				x = 1;
			}
			spin_unlock_irq(&lo->lo_lock);
			if (x)
				break;
			schedule();
		}
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&lo->lo_buf_wait, &waitq);
		goto try_next_old_bio_vec;
	}
	if ((rw == WRITE) && origHasData) {
		sector_t tmpIV;
		struct page *pg;
		char *addrPtr;
		unsigned int vecPGoffs, lenRem, lenNow;

		extension = new_bio->bi_private;
		y = extension->bioext_index;
		pg = old_bio->bi_io_vec[y].bv_page;
		vecPGoffs = old_bio->bi_io_vec[y].bv_offset + extension->bioext_done_offset;
		lenRem = extension->bioext_size;
		addrPtr = page_address(new_bio->bi_io_vec[0].bv_page);
		pg += vecPGoffs / PAGE_SIZE;
		vecPGoffs %= PAGE_SIZE;
		tmpIV = extension->bioext_iv;
		while(lenRem > 0) {
			md = kmap(pg) + vecPGoffs;
			lenNow = lenRem;
			if((lenNow + vecPGoffs) > PAGE_SIZE)
				lenNow = PAGE_SIZE - vecPGoffs;
			vecPGoffs = 0;
			if (lo_do_transfer(lo, WRITE, addrPtr, md, lenNow, tmpIV)) {
				merge->bi_opf = 0;
				smp_wmb();
			}
			kunmap(pg);
			pg++;
			lenRem -= lenNow;
			addrPtr += lenNow;
			tmpIV += lenNow >> 9;
		}
	}

	x = 0;
	spin_lock_irq(&lo->lo_lock);
	if((++lo->lo_bio_flshCnt >= lo->lo_bio_flshMax) || flushFlag) {
		x = 1;
		lo->lo_bio_flshCnt = 0;
		new_f &= ~REQ_IDLE;
	}
	spin_unlock_irq(&lo->lo_lock);

	bio_set_op_attrs(new_bio, old_op, new_f);

	/* A thread may sleep and wait for new buffers from previously submitted requests. */
	/* Make sure requests are actually sent to backing device, and not just queued. */
	{
		struct bio_list *blistTmp = current->bio_list;
		current->bio_list = NULL;
		submit_bio_noacct(new_bio);
		current->bio_list = blistTmp;
	}

	if (x)
		loop_unplug_backingdev(lo->lo_backingQueue);

	/* other vecs may need processing too */
	if (!lastVec)
		goto try_next_old_bio_vec;
	return;

out:
	old_bio->bi_next = NULL;
	bio_io_error(old_bio);
	return;
}

struct loop_switch_request {
	struct file *file;
	struct completion wait;
};

static void do_loop_switch(struct loop_device *lo, struct loop_switch_request *p)
{
	struct file *file = p->file;
	struct file *old_file=lo->lo_backing_file;
	struct address_space *mapping = file->f_mapping;
	
	/* This code runs on file backed loop only */
	/* no need to worry about -1 old_gfp_mask */
	mapping_set_gfp_mask(old_file->f_mapping, lo->old_gfp_mask);
	lo->lo_backing_file = file;
	memset(lo->lo_file_name, 0, LO_NAME_SIZE);
	lo->old_gfp_mask = mapping_gfp_mask(mapping);
	mapping_set_gfp_mask(mapping, (lo->old_gfp_mask & ~(__GFP_IO|__GFP_FS)) | __GFP_HIGH);
	complete(&p->wait);
}

/*
 * worker thread that handles reads/writes to file backed loop devices,
 * to avoid blocking in our make_request_fn. it also does loop decrypting
 * on reads for block backed loop, as that is too heavy to do from
 * b_end_io context where irqs may be disabled.
 */
static int loop_thread(void *data)
{
	struct loop_device *lo = data;
	struct bio *bio, *xbio, *merge;
	struct loop_bio_extension *extension;
	int x = 0, y;
	wait_queue_entry_t waitq;
	char *md;
	static const struct rlimit loop_rlim_defaults[RLIM_NLIMITS] = INIT_RLIMITS;
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
	void (*keyscrubFn)(void *) = 0;
#endif

	init_waitqueue_entry(&waitq, current);
	memcpy(&current->signal->rlim[0], &loop_rlim_defaults[0], sizeof(current->signal->rlim));

	/*
	 * loop can be used in an encrypted device,
	 * hence, it mustn't be stopped at all
	 * because it could be indirectly used during suspension
	 */
	current->flags |= PF_NOFREEZE;
	current->flags |= PF_LOCAL_THROTTLE;

	if (lo_nice > 0)
		lo_nice = 0;
	if (lo_nice < -20)
		lo_nice = -20;
	set_user_nice(current, lo_nice);

	atomic_inc(&lo->lo_pending);

	/*
	 * up sem, we are running
	 */
	complete(&lo->lo_done);

	for (;;) {
		add_wait_queue(&lo->lo_bio_wait, &waitq);
		for (;;) {
			set_current_state(TASK_INTERRUPTIBLE);
			if (!atomic_read(&lo->lo_pending))
				break;

			x = 0;
			spin_lock_irq(&lo->lo_lock);
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
			if((keyscrubFn = lo->lo_keyscrub_fn) != 0) {
				lo->lo_keyscrub_fn = 0;
				x = 1;
			}
#endif
			if (lo->lo_bio_que0) {
				/* don't sleep if device backed READ needs processing */
				/* don't sleep if file backed READ/WRITE needs processing */
				x = 1;
			}
			spin_unlock_irq(&lo->lo_lock);
			if (x)
				break;

			schedule();
		}
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&lo->lo_bio_wait, &waitq);

#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
		if(keyscrubFn) {
			(*keyscrubFn)(lo->lo_keyscrub_ptr);
			keyscrubFn = 0;
		}
#endif
		/*
		 * could be woken because of tear-down, not because of
		 * pending work
		 */
		if (!atomic_read(&lo->lo_pending))
			break;

		bio = loop_get_bio(lo);
		if (!bio)
			continue;

		if (lo->lo_flags & LO_FLAGS_DO_BMAP) {
			/* request is for file backed device */
			if(unlikely(bio->bi_end_io == loop_end_io_change_fd_request)) {
				do_loop_switch(lo, bio->bi_private);
				bio->bi_next = NULL;
				bio_put(bio);
			} else {
				y = do_bio_filebacked(lo, bio);
				bio->bi_next = NULL;
				LOOP_COMPAT_BIO_ENDIO(bio, y);
			}
		} else {
			/* device backed read has completed, do decrypt now */
			extension = bio->bi_private;
			merge = extension->bioext_merge;
			y = extension->bioext_index;
			xbio = merge->bi_private;
			if(extension->bioext_size) {
				sector_t tmpIV;
				struct page *pg;
				char *addrPtr;
				unsigned int vecPGoffs, lenRem, lenNow;

				pg = xbio->bi_io_vec[y].bv_page;
				vecPGoffs = xbio->bi_io_vec[y].bv_offset + extension->bioext_done_offset;
				lenRem = extension->bioext_size;
				addrPtr = page_address(bio->bi_io_vec[0].bv_page);
				pg += vecPGoffs / PAGE_SIZE;
				vecPGoffs %= PAGE_SIZE;
				tmpIV = extension->bioext_iv;
				while(lenRem > 0) {
					md = kmap(pg) + vecPGoffs;
					lenNow = lenRem;
					if((lenNow + vecPGoffs) > PAGE_SIZE)
						lenNow = PAGE_SIZE - vecPGoffs;
					vecPGoffs = 0;
					if (lo_do_transfer(lo, READ, addrPtr, md, lenNow, tmpIV)) {
						merge->bi_opf = 0;
						smp_wmb();
					}
					flush_dcache_page(pg);
					kunmap(pg);
					pg++;
					lenRem -= lenNow;
					addrPtr += lenNow;
					tmpIV += lenNow >> 9;
				}
			}
			loop_put_buffer(lo, bio, 0);
			if (!atomic_dec_and_test(&merge->LOOP_COMPAT_BI_CNT))
				continue;
			xbio->bi_next = NULL;
			smp_rmb();
			LOOP_COMPAT_BIO_ENDIO(xbio, merge->bi_opf ? (int)merge->LOOP_COMPAT_BI_SIZE : LOOP_COMPAT_BLK_STS_IOERR);
			loop_put_buffer(lo, merge, 1);
		}

		/*
		 * woken both for pending work and tear-down, lo_pending
		 * will hit zero then
		 */
		if (atomic_dec_and_test(&lo->lo_pending))
			break;
	}

	complete(&lo->lo_done);
	return 0;
}

static void loop_set_softblksz(struct loop_device *lo, struct block_device *bdev)
{
	int	bs, x;

	if (lo->lo_device)
		bs = block_size(lo->lo_device);
	else
		bs = PAGE_SIZE;
	if (lo->lo_flags & LO_FLAGS_DO_BMAP) {
		x = (int) bdev->bd_inode->i_size;
		if ((bs == 8192) && (x & 0x1E00))
			bs = 4096;
		if ((bs == 4096) && (x & 0x0E00))
			bs = 2048;
		if ((bs == 2048) && (x & 0x0600))
			bs = 1024;
		if ((bs == 1024) && (x & 0x0200))
			bs = 512;
	}
	set_blocksize(bdev, bs);
}

/* 
 * loop_change_fd switches the backing store of a loopback device to a 
 * new file. This is useful for operating system installers to free up the
 * original file and in High Availability environments to switch to an 
 * alternative location for the content in case of server meltdown.
 * This can only work if the loop device is used read-only, file backed,
 * and if the new backing store is the same size and type as the old
 * backing store.
 */
static int loop_change_fd(struct loop_device *lo, unsigned int arg)
{
	struct file *file, *old_file;
	struct inode *inode;
	struct loop_switch_request w;
	struct bio *bio;
	int error;

	error = -EINVAL;
	/* loop must be read-only */
	if (!(lo->lo_flags & LO_FLAGS_READ_ONLY))
		goto out;

	/* loop must be file backed */
	if (!(lo->lo_flags & LO_FLAGS_DO_BMAP))
		goto out;

	error = -EBADF;
	file = fget(arg);
	if (!file)
		goto out;

	inode = file->f_mapping->host;
	old_file = lo->lo_backing_file;

	error = -EINVAL;
	/* new backing store must be file backed */
	if (!S_ISREG(inode->i_mode))
		goto out_putf;

	/* new backing store must support reads */
	if (!(file->f_mode & FMODE_CAN_READ))
		goto out_putf;

	/* new backing store must be same size as the old one */
	if(i_size_read(inode) != i_size_read(old_file->f_mapping->host))
		goto out_putf;

	/* loop must be in properly initialized state */
	if(!(lo->lo_flags & LO_FLAGS_INITIALIZED))
		goto out_putf;

	error = -ENOMEM;
	bio = bio_alloc(NULL, 1, 0, GFP_KERNEL);
	if (!bio)
		goto out_putf;

	/* wait for loop thread to do the switch */
	init_completion(&w.wait);
	w.file = file;
	bio->bi_private = &w;
	bio->bi_end_io = loop_end_io_change_fd_request;
	bio_set_op_attrs(bio, REQ_OP_READ, 0);
	bio->bi_bdev = (struct block_device *)lo;
	loop_submit_bio_real(bio);
	wait_for_completion(&w.wait);

	fput(old_file);
	return 0;
	
out_putf:
	fput(file);
out:
	return error;
}

static int loop_get_threads_count(struct loop_device *lo)
{
	int x, y;

	if (lo->lo_flags & LO_FLAGS_DO_BMAP) {
		/* file backed has only 1 pre-allocated page, so limit to 1 helper thread */
		return 1;
	}

	y = lo_threads[0];
	for (x = 1; x < (sizeof(lo_threads) / sizeof(int)); x += 2) {
		if (lo_threads[x + 1] && (lo->lo_number == lo_threads[x])) {
			y = lo_threads[x + 1];
			break;
		}
	}
	return y;
}

#if defined(LOOP_HAVE_CONGESTED_FN)
static int loop_congested(void *data, int bits)
{
	struct loop_device *lo = data;
	struct bio *bio;
	int ret = 0;
	unsigned long flags;
	const int cong = (1 << BDI_sync_congested) | (1 << BDI_async_congested);

	if(lo && lo->lo_backingQueue) {
		/* check if backing device is congested */
		ret |= bdi_congested(&lo->lo_backingQueue->backing_dev_info, bits);
		/* check if loop device is low on resources */
		spin_lock_irqsave(&lo->lo_lock, flags);
		bio = lo->lo_bio_free0;
		if(!bio || !bio->bi_next) {
			ret |= cong;
		}
		bio = lo->lo_bio_free1;
		if(!bio || !bio->bi_next) {
			ret |= cong;
		}
		spin_unlock_irqrestore(&lo->lo_lock, flags);
	}
	return (ret & bits);
}
#endif

static int loop_set_fd(struct loop_device *lo, unsigned int ldom,
		       struct block_device *bdev, unsigned int arg)
{
	struct file	*file;
	struct inode	*inode;
	struct block_device *lo_device = NULL;
	int		lo_flags = 0;
	int		error;
	int		x, y, nrq;
	struct task_struct *t[LO_THREADS_MAX];

	error = -EBADF;
	file = fget(arg);
	if (!file)
		goto out;

	error = -EINVAL;
	inode = file->f_mapping->host;

	if (!(file->f_mode & FMODE_WRITE))
		lo_flags |= LO_FLAGS_READ_ONLY;

	init_completion(&lo->lo_done);
	spin_lock_init(&lo->lo_lock);
	init_waitqueue_head(&lo->lo_bio_wait);
	init_waitqueue_head(&lo->lo_buf_wait);
	atomic_set(&lo->lo_pending, 0);
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
	lo->lo_keyscrub_fn = 0;
#endif
	lo->lo_offset = lo->lo_sizelimit = 0;
	lo->lo_offs_sec = lo->lo_iv_remove = 0;
	lo->lo_encryption = NULL;
	lo->lo_encrypt_key_size = 0;
	lo->transfer = NULL;
	lo->lo_crypt_name[0] = 0;
	lo->lo_file_name[0] = 0;
	lo->lo_init[1] = lo->lo_init[0] = 0;
	lo->lo_key_owner = GLOBAL_ROOT_UID;
	lo->ioctl = NULL;
	lo->key_data = NULL;	
	lo->lo_bio_que0 = NULL;
	lo->lo_bio_free1 = lo->lo_bio_free0 = NULL;
	lo->lo_bio_flshMax = lo->lo_bio_flshCnt = 0;

	if (S_ISBLK(inode->i_mode)) {
		lo_device = I_BDEV(inode);
		if (lo_device == bdev) {
			error = -EBUSY;
			goto out_putf;
		}
		if (loop_prealloc_init(lo, 0, &nrq)) {
			error = -ENOMEM;
			goto out_putf;
		}
		if (bdev_read_only(lo_device))
			lo_flags |= LO_FLAGS_READ_ONLY;
		else
			filemap_fdatawrite(inode->i_mapping);
	} else if (S_ISREG(inode->i_mode)) {
		/*
		 * If we can't read - sorry. If we only can't write - well,
		 * it's going to be read-only.
		 */
		if (!(file->f_mode & FMODE_CAN_READ))
			goto out_putf;

		if (!(file->f_mode & FMODE_CAN_WRITE))
			lo_flags |= LO_FLAGS_READ_ONLY;

		lo_flags |= LO_FLAGS_DO_BMAP;
		if (loop_prealloc_init(lo, 1, &nrq)) {
			error = -ENOMEM;
			goto out_putf;
		}
	} else
		goto out_putf;

	get_file(file);

	if (!(ldom & FMODE_WRITE))
		lo_flags |= LO_FLAGS_READ_ONLY;

	bdev->bd_read_only = (lo_flags & LO_FLAGS_READ_ONLY) != 0;

	lo->lo_device = lo_device;
	lo->lo_flags = lo_flags;
	if(lo_flags & LO_FLAGS_READ_ONLY)
		lo->lo_flags |= 0x200000; /* export to user space */
	lo->lo_backing_file = file;
	if (figure_loop_size(lo, bdev)) {
		error = -EFBIG;
		goto out_cleanup;
	}

	/*
	 * add limits based on lower level device
	 */
	blk_queue_dma_alignment(lo->lo_queue, 511);
#if LINUX_VERSION_CODE >= 0x6000b
	blk_queue_logical_block_size(lo->lo_queue, 512);
	blk_queue_physical_block_size(lo->lo_queue, 512);
	lo->lo_queue->limits.io_min = 512;
	lo->lo_queue->limits.io_opt = 0;
#else
	blk_set_default_limits(&lo->lo_queue->limits);
#endif
	blk_queue_bounce_limit(lo->lo_queue, BLK_BOUNCE_HIGH);
	blk_queue_max_segment_size(lo->lo_queue, PAGE_SIZE);
	blk_queue_segment_boundary(lo->lo_queue, PAGE_SIZE - 1);
	blk_queue_max_segments(lo->lo_queue, BLK_MAX_SEGMENTS);
	blk_queue_max_hw_sectors(lo->lo_queue, 1024);
	lo->lo_queue->nr_requests = (unsigned long)nrq;
	lo->lo_queue->queue_flags &= ~((1UL << QUEUE_FLAG_WC) | (1UL << QUEUE_FLAG_FUA));
	lo->lo_backingQueue = 0;

	/*
	 * we remap to a block device, make sure we correctly stack limits
	 */
	if (S_ISBLK(inode->i_mode) && lo_device) {
		struct request_queue *q = bdev_get_queue(lo_device);

		blk_queue_logical_block_size(lo->lo_queue, queue_logical_block_size(q));
		blk_queue_physical_block_size(lo->lo_queue, queue_physical_block_size(q));
		lo->lo_queue->limits.io_min = q->limits.io_min;
		if(lo->lo_queue->limits.io_min > (BLK_MAX_SEGMENTS * PAGE_SIZE))
			lo->lo_queue->limits.io_min = (BLK_MAX_SEGMENTS * PAGE_SIZE);
		lo->lo_queue->limits.io_opt = q->limits.io_opt;
		if(lo->lo_queue->limits.io_opt > (BLK_MAX_SEGMENTS * PAGE_SIZE))
			lo->lo_queue->limits.io_opt = (BLK_MAX_SEGMENTS * PAGE_SIZE);
		if(q->queue_flags & (1UL << QUEUE_FLAG_WC))
			lo->lo_queue->queue_flags |= (1UL << QUEUE_FLAG_WC);
		if(q->queue_flags & (1UL << QUEUE_FLAG_FUA))
			lo->lo_queue->queue_flags |= (1UL << QUEUE_FLAG_FUA);
		lo->lo_backingQueue = q;
	}

	if (lo_flags & LO_FLAGS_DO_BMAP) {
		lo->old_gfp_mask = mapping_gfp_mask(inode->i_mapping);
		mapping_set_gfp_mask(inode->i_mapping, (lo->old_gfp_mask & ~(__GFP_IO|__GFP_FS)) | __GFP_HIGH);
	} else {
		lo->old_gfp_mask = -1;
	}

	loop_set_softblksz(lo, bdev);

	y = loop_get_threads_count(lo);
	for(x = 0; x < y; x++) {
		if(y > 1) {
			t[x] = kthread_create(loop_thread, lo, "loop%d%c", lo->lo_number, x + 'a');
		} else {
			t[x] = kthread_create(loop_thread, lo, "loop%d", lo->lo_number);
		}
		if (IS_ERR(t[x])) {
			error = PTR_ERR(t[x]);
			while(--x >= 0) {
				kthread_stop(t[x]);
			}
			goto out_mapping;
		}
	}
	for(x = 0; x < y; x++) {
		wake_up_process(t[x]);
		wait_for_completion(&lo->lo_done);
	}

	fput(file);
#if defined(LOOP_HAVE_CONGESTED_FN)
	lo->lo_queue->backing_dev_info.congested_data = lo;
	lo->lo_queue->backing_dev_info.congested_fn = loop_congested;
#endif
	wmb();
	lo->lo_queue->queuedata = lo;
	__module_get(THIS_MODULE);
	return 0;

 out_mapping:
	if(lo->old_gfp_mask != -1)
		mapping_set_gfp_mask(inode->i_mapping, lo->old_gfp_mask);
 out_cleanup:
	loop_prealloc_cleanup(lo);
	fput(file);
 out_putf:
	fput(file);
 out:
	return error;
}

static int loop_release_xfer(struct loop_device *lo)
{
	int err = 0;
	struct loop_func_table *xfer = lo->lo_encryption;

	if (xfer) {
		lo->transfer = NULL;
		if (xfer->release)
			err = xfer->release(lo);
		lo->lo_encryption = NULL;
		module_put(xfer->owner);
	}
	return err;
}

static int loop_init_xfer(struct loop_device *lo, struct loop_func_table *xfer, struct loop_info64 *i)
{
	int err = 0;

	if (xfer) {
		struct module *owner = xfer->owner;

		if(!try_module_get(owner))
			return -EINVAL;
		if (xfer->init)
			err = xfer->init(lo, i);
		if (err)
			module_put(owner);
		else
			lo->lo_encryption = xfer;
	}
	return err;
}

static int loop_clr_fd(struct loop_device *lo, struct block_device *bdev)
{
	struct file *filp = lo->lo_backing_file;
	int gfp = lo->old_gfp_mask;
	int bdocnt, x, y;

	/* sync /dev/loop? device */
	sync_blockdev(bdev);
	/* sync backing /dev/hda? device */
	sync_blockdev(lo->lo_device);

	for(x = 0; x < 20; x++) {
		spin_lock(&lo->lo_ioctl_spin);
		bdocnt = lo->lo_refcnt;
		spin_unlock(&lo->lo_ioctl_spin);
		if(bdocnt == 1) break;
		/* work around reference count race */
		msleep(50);
	}

	if (bdocnt != 1)	/* one for this fd being open */
		return -EBUSY;
	if (filp==NULL)
		return -EINVAL;

	lo->lo_queue->queuedata = NULL;
	lo->lo_flags &= ~(LO_FLAGS_INITIALIZED);
	lo->lo_backingQueue = 0;
	y = loop_get_threads_count(lo);
	for(x = 0; x < y; x++) {
		if (atomic_dec_and_test(&lo->lo_pending))
			wake_up_interruptible_all(&lo->lo_bio_wait);
	}
	for(x = 0; x < y; x++) {
		wait_for_completion(&lo->lo_done);
	}
	loop_prealloc_cleanup(lo);
	lo->lo_backing_file = NULL;
	loop_release_xfer(lo);
	lo->transfer = NULL;
	lo->ioctl = NULL;
	lo->lo_device = NULL;
	lo->lo_encryption = NULL;
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
	lo->lo_keyscrub_fn = 0;
#endif
	lo->lo_offset = lo->lo_sizelimit = 0;
	lo->lo_offs_sec = lo->lo_iv_remove = 0;
	lo->lo_encrypt_key_size = 0;
	lo->lo_flags = 0;
	lo->lo_init[1] = lo->lo_init[0] = 0;
	lo->lo_key_owner = GLOBAL_ROOT_UID;
	lo->key_data = NULL;
	memset(lo->lo_encrypt_key, 0, LO_KEY_SIZE);
	memset(lo->lo_crypt_name, 0, LO_NAME_SIZE);
	memset(lo->lo_file_name, 0, LO_NAME_SIZE);
	invalidate_bdev(bdev);
	set_capacity(disks[lo->lo_number], 0);
	if (gfp != -1)
		mapping_set_gfp_mask(filp->f_mapping, gfp);
	fput(filp);
	module_put(THIS_MODULE);
	return 0;
}

static int loop_set_status(struct loop_device *lo, struct block_device *bdev, struct loop_info64 *info)
{
	int err;
	struct loop_func_table *xfer = NULL;
	kuid_t uid = current_uid();

	if (lo->lo_encrypt_key_size && !uid_eq(lo->lo_key_owner, uid) && !capable(CAP_SYS_ADMIN))
		return -EPERM;
	if ((unsigned int) info->lo_encrypt_key_size > LO_KEY_SIZE)
		return -EINVAL;

	err = loop_release_xfer(lo);
	if (err)
		return err;

	if ((loff_t)info->lo_offset < 0) {
		/* negative offset == remove offset from IV computations */
		lo->lo_offset = -(info->lo_offset);
		lo->lo_iv_remove = lo->lo_offset >> 9;
	} else {
		/* positive offset == include offset in IV computations */
		lo->lo_offset = info->lo_offset;
		lo->lo_iv_remove = 0;
	}
	lo->lo_offs_sec = lo->lo_offset >> 9;
	lo->lo_sizelimit = info->lo_sizelimit;
	err = figure_loop_size(lo, bdev);
	if (err)
		return err;
	loop_set_softblksz(lo, bdev);

	if (info->lo_encrypt_type) {
		unsigned int type = info->lo_encrypt_type;

		if (type >= MAX_LO_CRYPT)
			return -EINVAL;
		xfer = xfer_funcs[type];
		if (xfer == NULL)
			return -EINVAL;
	} else if(!(lo->lo_flags & LO_FLAGS_DO_BMAP)) {
		blk_queue_max_hw_sectors(lo->lo_queue, PAGE_SIZE >> 9);
	}
	err = loop_init_xfer(lo, xfer, info);
	if (err)
		return err;

	if (!xfer)
		xfer = &none_funcs;
	lo->transfer = xfer->transfer;
	lo->ioctl = xfer->ioctl;
	
	memcpy(lo->lo_file_name, info->lo_file_name, LO_NAME_SIZE);
	memcpy(lo->lo_crypt_name, info->lo_crypt_name, LO_NAME_SIZE);
	lo->lo_file_name[LO_NAME_SIZE-1] = 0;
	lo->lo_crypt_name[LO_NAME_SIZE-1] = 0;
	lo->lo_encrypt_key_size = info->lo_encrypt_key_size;
	lo->lo_init[0] = info->lo_init[0];
	lo->lo_init[1] = info->lo_init[1];
	if (info->lo_encrypt_key_size) {
		memcpy(lo->lo_encrypt_key, info->lo_encrypt_key,
		       info->lo_encrypt_key_size);
		lo->lo_key_owner = uid;
	}	

	lo->lo_flags |= LO_FLAGS_INITIALIZED;
	return 0;
}

static int loop_get_status(struct loop_device *lo, struct loop_info64 *info)
{
	struct file *file = lo->lo_backing_file;
	struct kstat stat;
	int error;

	error = vfs_getattr(&file->f_path, &stat, STATX_BASIC_STATS, AT_STATX_SYNC_AS_STAT);
	if (error)
		return error;
	memset(info, 0, sizeof(*info));
	info->lo_number = lo->lo_number;
	info->lo_device = huge_encode_dev(stat.dev);
	info->lo_inode = stat.ino;
	info->lo_rdevice = huge_encode_dev(lo->lo_device ? stat.rdev : stat.dev);
	info->lo_offset = lo->lo_iv_remove ? -(lo->lo_offset) : lo->lo_offset;
	info->lo_sizelimit = lo->lo_sizelimit;
	info->lo_flags = lo->lo_flags;
	memcpy(info->lo_file_name, lo->lo_file_name, LO_NAME_SIZE);
	memcpy(info->lo_crypt_name, lo->lo_crypt_name, LO_NAME_SIZE);
	info->lo_encrypt_type = lo->lo_encryption ? lo->lo_encryption->number : 0;
	if (lo->lo_encrypt_key_size && capable(CAP_SYS_ADMIN)) {
		info->lo_encrypt_key_size = lo->lo_encrypt_key_size;
		memcpy(info->lo_encrypt_key, lo->lo_encrypt_key,
		       lo->lo_encrypt_key_size);
		info->lo_init[0] = lo->lo_init[0];
		info->lo_init[1] = lo->lo_init[1];
	}
	return 0;
}

static void
loop_info64_from_old(const struct loop_info *info, struct loop_info64 *info64)
{
	memset(info64, 0, sizeof(*info64));
	info64->lo_number = info->lo_number;
	info64->lo_device = info->lo_device;
	info64->lo_inode = info->lo_inode;
	info64->lo_rdevice = info->lo_rdevice;
	info64->lo_offset = info->lo_offset;
	info64->lo_encrypt_type = info->lo_encrypt_type;
	info64->lo_encrypt_key_size = info->lo_encrypt_key_size;
	info64->lo_flags = info->lo_flags;
	info64->lo_init[0] = info->lo_init[0];
	info64->lo_init[1] = info->lo_init[1];
	if (info->lo_encrypt_type == LO_CRYPT_CRYPTOAPI)
		memcpy(info64->lo_crypt_name, info->lo_name, LO_NAME_SIZE);
	else
		memcpy(info64->lo_file_name, info->lo_name, LO_NAME_SIZE);
	memcpy(info64->lo_encrypt_key, info->lo_encrypt_key, LO_KEY_SIZE);
}

static int
loop_info64_to_old(struct loop_info64 *info64, struct loop_info *info)
{
	memset(info, 0, sizeof(*info));
	info->lo_number = info64->lo_number;
	info->lo_device = info64->lo_device;
	info->lo_inode = info64->lo_inode;
	info->lo_rdevice = info64->lo_rdevice;
	info->lo_offset = info64->lo_offset;
	info->lo_encrypt_type = info64->lo_encrypt_type;
	info->lo_encrypt_key_size = info64->lo_encrypt_key_size;
	info->lo_flags = info64->lo_flags;
	info->lo_init[0] = info64->lo_init[0];
	info->lo_init[1] = info64->lo_init[1];
	if (info->lo_encrypt_type == LO_CRYPT_CRYPTOAPI)
		memcpy(info->lo_name, info64->lo_crypt_name, LO_NAME_SIZE);
	else
		memcpy(info->lo_name, info64->lo_file_name, LO_NAME_SIZE);
	memcpy(info->lo_encrypt_key, info64->lo_encrypt_key, LO_KEY_SIZE);

	/* error in case values were truncated */
	if (info->lo_device != info64->lo_device ||
	    info->lo_rdevice != info64->lo_rdevice ||
	    info->lo_inode != info64->lo_inode ||
	    info->lo_offset != info64->lo_offset ||
	    info64->lo_sizelimit)
		return -EOVERFLOW;

	return 0;
}

static int
loop_set_status_old(struct loop_device *lo, struct block_device *bdev, const struct loop_info *arg, void *stkBuf)
{
	struct loop_info info;
	struct loop_info64 info64;

	if(!stkBuf) {
		if (copy_from_user(&info, arg, sizeof (struct loop_info)))
			return -EFAULT;
		loop_info64_from_old(&info, &info64);
		memset(&info.lo_encrypt_key[0], 0, sizeof(info.lo_encrypt_key));
	} else {
		loop_info64_from_old((struct loop_info *)stkBuf, &info64);
	}
	return loop_set_status(lo, bdev, &info64);
}

static int
loop_set_status64(struct loop_device *lo, struct block_device *bdev, struct loop_info64 *arg)
{
	struct loop_info64 info64;

	if (copy_from_user(&info64, arg, sizeof (struct loop_info64)))
		return -EFAULT;
	return loop_set_status(lo, bdev, &info64);
}

static int
loop_get_status_old(struct loop_device *lo, struct loop_info *arg, void *stkBuf)
{
	struct loop_info info;
	struct loop_info64 info64;
	int err = 0;

	if (!arg)
		err = -EINVAL;
	if (!err)
		err = loop_get_status(lo, &info64);
	if (!err)
		err = loop_info64_to_old(&info64, stkBuf ? (struct loop_info *)stkBuf : &info);
	if (!stkBuf && !err && copy_to_user(arg, &info, sizeof(info)))
		err = -EFAULT;

	return err;
}

static int
loop_get_status64(struct loop_device *lo, struct loop_info64 *arg) {
	struct loop_info64 info64;
	int err = 0;

	if (!arg)
		err = -EINVAL;
	if (!err)
		err = loop_get_status(lo, &info64);
	if (!err && copy_to_user(arg, &info64, sizeof(info64)))
		err = -EFAULT;

	return err;
}

static int lo_ioctl(struct block_device *bdev, fmode_t ldom, unsigned int cmd, unsigned long arg)
{
	struct loop_device *lo = bdev->bd_disk->private_data;
	int err;
	wait_queue_entry_t waitq;

	/*
	 * mutual exclusion - lock
	 */
	init_waitqueue_entry(&waitq, current);
	add_wait_queue(&lo->lo_ioctl_wait, &waitq);
	for (;;) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		spin_lock(&lo->lo_ioctl_spin);
		err = lo->lo_ioctl_busy;
		if(!err) lo->lo_ioctl_busy = 1;
		spin_unlock(&lo->lo_ioctl_spin);
		if(!err) break;
		schedule();
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&lo->lo_ioctl_wait, &waitq);

	/*
	 * LOOP_SET_FD can only be called when no device is attached.
	 * All other ioctls can only be called when a device is attached.
	 */
	if (bdev->bd_disk->queue->queuedata != NULL) {
		if (cmd == LOOP_SET_FD) {
			err = -EBUSY;
			goto out_err;
		}
	} else {
		if (cmd != LOOP_SET_FD) {
			err = -ENXIO;
			goto out_err;
		}
	}

	switch (cmd) {
	case LOOP_SET_FD:
		err = loop_set_fd(lo, ldom, bdev, arg);
		break;
	case LOOP_CHANGE_FD:
		err = loop_change_fd(lo, arg);
		break;
	case LOOP_CLR_FD:
		err = loop_clr_fd(lo, bdev);
		break;
	case LOOP_SET_STATUS:
		err = loop_set_status_old(lo, bdev, (struct loop_info *) arg, 0);
		break;
	case LOOP_GET_STATUS:
		err = loop_get_status_old(lo, (struct loop_info *) arg, 0);
		break;
	case LOOP_SET_STATUS64:
		err = loop_set_status64(lo, bdev, (struct loop_info64 *) arg);
		break;
	case LOOP_GET_STATUS64:
		err = loop_get_status64(lo, (struct loop_info64 *) arg);
		break;
	case LOOP_RECOMPUTE_DEV_SIZE:
		err = figure_loop_size(lo, bdev);
		break;
	default:
		err = lo->ioctl ? lo->ioctl(lo, cmd, arg) : -EINVAL;
	}
out_err:
	/*
	 * mutual exclusion - unlock
	 */
	spin_lock(&lo->lo_ioctl_spin);
	lo->lo_ioctl_busy = 0;
	spin_unlock(&lo->lo_ioctl_spin);
	wake_up_all(&lo->lo_ioctl_wait);

	return err;
}

#if defined(CONFIG_COMPAT)
struct loop_info32 {
	compat_int_t	lo_number;      /* ioctl r/o */
	compat_dev_t	lo_device;      /* ioctl r/o */
	compat_ulong_t	lo_inode;       /* ioctl r/o */
	compat_dev_t	lo_rdevice;     /* ioctl r/o */
	compat_int_t	lo_offset;
	compat_int_t	lo_encrypt_type;
	compat_int_t	lo_encrypt_key_size;    /* ioctl w/o */
	compat_int_t	lo_flags;       /* ioctl r/o */
	char		lo_name[LO_NAME_SIZE];
	unsigned char	lo_encrypt_key[LO_KEY_SIZE]; /* ioctl w/o */
	compat_ulong_t	lo_init[2];
	char		reserved[4];
};

static int lo_compat_ioctl(struct block_device *bdev, fmode_t ldom, unsigned int cmd, unsigned long arg)
{
	struct loop_device *lo = bdev->bd_disk->private_data;
	int err;
	wait_queue_entry_t waitq;
	struct loop_info l;
	struct loop_info32 *ul = (struct loop_info32 *)arg;

	/*
	 * mutual exclusion - lock
	 */
	init_waitqueue_entry(&waitq, current);
	add_wait_queue(&lo->lo_ioctl_wait, &waitq);
	for (;;) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		spin_lock(&lo->lo_ioctl_spin);
		err = lo->lo_ioctl_busy;
		if(!err) lo->lo_ioctl_busy = 1;
		spin_unlock(&lo->lo_ioctl_spin);
		if(!err) break;
		schedule();
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&lo->lo_ioctl_wait, &waitq);

	/*
	 * LOOP_SET_FD can only be called when no device is attached.
	 * All other ioctls can only be called when a device is attached.
	 */
	if (bdev->bd_disk->queue->queuedata != NULL) {
		if (cmd == LOOP_SET_FD) {
			err = -EBUSY;
			goto out_err;
		}
	} else {
		if (cmd != LOOP_SET_FD) {
			err = -ENXIO;
			goto out_err;
		}
	}

	switch (cmd) {
	case LOOP_SET_FD:
		err = loop_set_fd(lo, ldom, bdev, arg);
		break;
	case LOOP_CHANGE_FD:
		err = loop_change_fd(lo, arg);
		break;
	case LOOP_CLR_FD:
		err = loop_clr_fd(lo, bdev);
		break;
	case LOOP_SET_STATUS:
		memset(&l, 0, sizeof(l));
		err = get_user(l.lo_number, &ul->lo_number);
		err |= get_user(l.lo_device, &ul->lo_device);
		err |= get_user(l.lo_inode, &ul->lo_inode);
		err |= get_user(l.lo_rdevice, &ul->lo_rdevice);
		err |= copy_from_user(&l.lo_offset, &ul->lo_offset,
		        8 + (unsigned long)l.lo_init - (unsigned long)&l.lo_offset);
		if (err) {
			err = -EFAULT;
		} else {
			err = loop_set_status_old(lo, bdev, (struct loop_info *) arg, &l);
		}
		memset(&l, 0, sizeof(l));
		break;
	case LOOP_GET_STATUS:
		err = loop_get_status_old(lo, (struct loop_info *) arg, &l);
		if (!err) {
			err = put_user(l.lo_number, &ul->lo_number);
			err |= put_user(l.lo_device, &ul->lo_device);
			err |= put_user(l.lo_inode, &ul->lo_inode);
			err |= put_user(l.lo_rdevice, &ul->lo_rdevice);
			err |= copy_to_user(&ul->lo_offset, &l.lo_offset,
				(unsigned long)l.lo_init - (unsigned long)&l.lo_offset);
			if (err)
				err = -EFAULT;
		}
		memset(&l, 0, sizeof(l));
		break;
	case LOOP_SET_STATUS64:
		err = loop_set_status64(lo, bdev, (struct loop_info64 *) arg);
		break;
	case LOOP_GET_STATUS64:
		err = loop_get_status64(lo, (struct loop_info64 *) arg);
		break;
	case LOOP_RECOMPUTE_DEV_SIZE:
		err = figure_loop_size(lo, bdev);
		break;
	default:
		err = lo->ioctl ? lo->ioctl(lo, cmd, arg) : -EINVAL;
	}
out_err:
	/*
	 * mutual exclusion - unlock
	 */
	spin_lock(&lo->lo_ioctl_spin);
	lo->lo_ioctl_busy = 0;
	spin_unlock(&lo->lo_ioctl_spin);
	wake_up_all(&lo->lo_ioctl_wait);

	return err;
}
#endif

static int lo_open(struct block_device *bdev, fmode_t mode)
{
	struct loop_device *lo = bdev->bd_disk->private_data;
	
	spin_lock(&lo->lo_ioctl_spin);
	lo->lo_refcnt++;
	spin_unlock(&lo->lo_ioctl_spin);
	return 0;
}

static void lo_release(struct gendisk *disk, fmode_t mode)
{
	struct loop_device *lo = disk->private_data;

	spin_lock(&lo->lo_ioctl_spin);
	lo->lo_refcnt--;
	spin_unlock(&lo->lo_ioctl_spin);
}

static const struct block_device_operations lo_fops = {
	.owner =	THIS_MODULE,
	.open =         lo_open,
	.release =      lo_release,
	.ioctl =	lo_ioctl,
	.submit_bio =   loop_submit_bio_real,
#if defined(CONFIG_COMPAT)
	.compat_ioctl = lo_compat_ioctl,
#endif
};

/*
 * And now the modules code and kernel interface.
 */
MODULE_LICENSE("GPL");
MODULE_ALIAS_BLOCKDEV_MAJOR(LOOP_MAJOR);

int loop_register_transfer(struct loop_func_table *funcs)
{
	unsigned int n = funcs->number;

	if (n >= MAX_LO_CRYPT || xfer_funcs[n])
		return -EINVAL;
	xfer_funcs[n] = funcs;
	return 0;
}

int loop_unregister_transfer(int number)
{
	unsigned int n = number;
	struct loop_device *lo;
	struct loop_func_table *xfer;
	int x;

	if (n == 0 || n >= MAX_LO_CRYPT || (xfer = xfer_funcs[n]) == NULL)
		return -EINVAL;
	xfer_funcs[n] = NULL;
	for (x = 0; x < max_loop; x++) {
		lo = loop_dev_ptr_arr[x];
		if (!lo)
			continue;
		if (lo->lo_encryption == xfer)
			loop_release_xfer(lo);
	}
	return 0;
}

EXPORT_SYMBOL(loop_register_transfer);
EXPORT_SYMBOL(loop_unregister_transfer);

int __init loop_init(void)
{
	int	i;

#ifdef CONFIG_BLK_DEV_LOOP_AES
#if defined(CONFIG_BLK_DEV_LOOP_PADLOCK) && (defined(CONFIG_X86) || defined(CONFIG_X86_64))
	if((boot_cpu_data.x86 >= 6) && CentaurHauls_ID_and_enabled_ACE()) {
		xfer_funcs[LO_CRYPT_AES] = &funcs_padlock_aes;
		printk(KERN_INFO "loop: padlock hardware AES enabled\n");
	} else
#endif
#if defined(CONFIG_BLK_DEV_LOOP_INTELAES) && (defined(CONFIG_X86) || defined(CONFIG_X86_64))
	if((boot_cpu_data.x86 >= 6) && ((cpuid_ecx(1) & 0x02000000) == 0x02000000)) {
		xfer_funcs[LO_CRYPT_AES] = &funcs_intel_aes;
		printk("loop: Intel hardware AES enabled\n");
	} else
#endif
#endif
	{ } /* needed because of above else statements */

	if ((max_loop < 1) || (max_loop > 256)) {
		printk(KERN_WARNING "loop: invalid max_loop (must be between"
				    " 1 and 256), using default (8)\n");
		max_loop = 8;
	}

	if (register_blkdev(LOOP_MAJOR, "loop"))
		return -EIO;

	loop_dev_ptr_arr = kmalloc(max_loop * sizeof(struct loop_device *), GFP_KERNEL);
	if (!loop_dev_ptr_arr)
		goto out_mem1;

	disks = kmalloc(max_loop * sizeof(struct gendisk *), GFP_KERNEL);
	if (!disks)
		goto out_mem2;

	for (i = 0; i < max_loop; i++) {
		loop_dev_ptr_arr[i] = kmalloc(sizeof(struct loop_device), GFP_KERNEL);
		if (!loop_dev_ptr_arr[i])
			goto out_mem3;
	}

	for (i = 0; i < max_loop; i++) {
		disks[i] = blk_alloc_disk(NUMA_NO_NODE);
		if (!disks[i])
			goto out_mem4;
		disks[i]->minors = 1;
		disks[i]->queue->queuedata = NULL;
	}

	for (i = 0; i < (sizeof(lo_prealloc) / sizeof(int)); i += 2) {
		if (!lo_prealloc[i])
			continue;
		if (lo_prealloc[i] < LO_PREALLOC_MIN)
			lo_prealloc[i] = LO_PREALLOC_MIN;
		if (lo_prealloc[i] > LO_PREALLOC_MAX)
			lo_prealloc[i] = LO_PREALLOC_MAX;
	}
	for (i = 0; i < (sizeof(lo_threads) / sizeof(int)); i += 2) {
		if (!lo_threads[i])
			continue;
		if (lo_threads[i] < LO_THREADS_MIN)
			lo_threads[i] = LO_THREADS_MIN;
		if (lo_threads[i] > LO_THREADS_MAX)
			lo_threads[i] = LO_THREADS_MAX;
	}

#ifdef CONFIG_DEVFS_FS
	devfs_mk_dir("loop");
#endif

	for (i = 0; i < max_loop; i++) {
		struct loop_device *lo = loop_dev_ptr_arr[i];
		struct gendisk *disk = disks[i];
		memset(lo, 0, sizeof(struct loop_device));
		lo->lo_number = i;
		lo->lo_queue = disk->queue;
		spin_lock_init(&lo->lo_ioctl_spin);
		init_waitqueue_head(&lo->lo_ioctl_wait);
		disk->major = LOOP_MAJOR;
		disk->first_minor = i;
		disk->fops = &lo_fops;
		sprintf(disk->disk_name, "loop%d", i);
#ifdef CONFIG_DEVFS_FS
		sprintf(disk->devfs_name, "loop/%d", i);
#endif
		disk->private_data = lo;
		if(add_disk(disk)) printk(KERN_ERR "loop: add_disk() failed\n");
	}

#ifdef CONFIG_BLK_DEV_LOOP_AES
#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
	printk(KERN_INFO "loop: AES key scrubbing enabled\n");
#endif
#endif
	printk(KERN_INFO "loop: loaded (max %d devices)\n", max_loop);
	return 0;

out_mem4:
	while (i--)
		put_disk(disks[i]);
	i = max_loop;
out_mem3:
	while (i--)
		kfree(loop_dev_ptr_arr[i]);
	kfree(disks);
out_mem2:
	kfree(loop_dev_ptr_arr);
out_mem1:
	unregister_blkdev(LOOP_MAJOR, "loop");
	printk(KERN_ERR "loop: ran out of memory\n");
	return -ENOMEM;
}

void loop_exit(void)
{
	int i;

	for (i = 0; i < max_loop; i++) {
		del_gendisk(disks[i]);
		put_disk(disks[i]);
		kfree(loop_dev_ptr_arr[i]);
	}
#ifdef CONFIG_DEVFS_FS
	devfs_remove("loop");
#endif
	unregister_blkdev(LOOP_MAJOR, "loop");
	kfree(disks);
	kfree(loop_dev_ptr_arr);
}

module_init(loop_init);
module_exit(loop_exit);

#ifdef CONFIG_BLK_DEV_LOOP_KEYSCRUB
void loop_add_keyscrub_fn(struct loop_device *lo, void (*fn)(void *), void *ptr)
{
    lo->lo_keyscrub_ptr = ptr;
    wmb();
    lo->lo_keyscrub_fn = fn;
    wake_up_interruptible(&lo->lo_bio_wait);
}
EXPORT_SYMBOL(loop_add_keyscrub_fn);
#endif
