/*
 * Implement AES algorithm in Intel AES-NI instructions.
 *
 * The white paper of AES-NI instructions can be downloaded from:
 *   http://softwarecommunity.intel.com/isn/downloads/intelavx/AES-Instructions-Set_WP.pdf
 *
 * Copyright (C) 2008, Intel Corp.
 *    Author: Huang Ying <ying.huang@intel.com>
 *            Vinodh Gopal <vinodh.gopal@intel.com>
 *            Kahraman Akdemir
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*
 * Modified by Jari Ruusu, October 2009
 *  - Adapted for loop-AES
 */

/*
 * Modified by Jari Ruusu, March 2010
 *  - Added parallelized 4x512 CBC encrypt
 */

#if !defined(ALIGN64BYTES)
# define ALIGN64BYTES 64
#endif

	.file	"aes-intel32.S"
	.globl intel_aes_cbc_encrypt
	.globl intel_aes_cbc_decrypt
	.globl intel_aes_cbc_enc_4x512
	.text

#define STATE1	%xmm0
#define STATE2	%xmm4
#define STATE3	%xmm5
#define STATE	STATE1
#define IN1	%xmm1
#define IN2	%xmm7
#define IN3	%xmm6
#define IN	IN1
#define KEY	%xmm2
#define IV	%xmm3

#define KEYP	%edi
#define INP	%esi
#define OUTP	%edx
#define LEN	%ecx
#define IVP	%ebx
#define NRND	%eax
#define TKEYP	%ebp

/*
 * void intel_aes_cbc_encrypt(const aes_context *, void *src, void *dst, size_t len, void *iv)
 *
 * Stack after reg saves:   36(%esp) = void *iv
 *                          32(%esp) = size_t len
 *                          28(%esp) = void *dst
 *                          24(%esp) = void *src
 *                          20(%esp) = aes_context *
 */
	.align	ALIGN64BYTES
intel_aes_cbc_encrypt:
	push %edi
	push %esi
	push %ebx
	push %ebp
	mov 20(%esp),KEYP
	mov 24(%esp),INP
	mov 28(%esp),OUTP
	mov 32(%esp),LEN
	mov 36(%esp),IVP
	mov 4(KEYP), NRND
	add $8, KEYP
	movups (IVP), STATE	# load iv as initial state
.align 4
.Lcbc_enc_loop:
	movups (INP), IN	# load input
	pxor IN, STATE

	movaps (KEYP), KEY		# key
	mov KEYP, TKEYP
	pxor KEY, STATE		# round 0
	add $0x30, TKEYP
	cmp $12, NRND
	jb .Lenc128
	lea 0x20(TKEYP), TKEYP
	je .Lenc192
	add $0x20, TKEYP
	movaps -0x60(TKEYP), KEY
	# aesenc KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc2
	movaps -0x50(TKEYP), KEY
	# aesenc KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc2
.align 4
.Lenc192:
	movaps -0x40(TKEYP), KEY
	# aesenc KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc2
	movaps -0x30(TKEYP), KEY
	# aesenc KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc2
.align 4
.Lenc128:
	movaps -0x20(TKEYP), KEY
	# aesenc KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc2
	movaps -0x10(TKEYP), KEY
	# aesenc KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc2
	movaps (TKEYP), KEY
	# aesenc KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc2
	movaps 0x10(TKEYP), KEY
	# aesenc KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc2
	movaps 0x20(TKEYP), KEY
	# aesenc KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc2
	movaps 0x30(TKEYP), KEY
	# aesenc KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc2
	movaps 0x40(TKEYP), KEY
	# aesenc KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc2
	movaps 0x50(TKEYP), KEY
	# aesenc KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc2
	movaps 0x60(TKEYP), KEY
	# aesenc KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc2
	movaps 0x70(TKEYP), KEY
	# aesenclast KEY, STATE	# last round
	.byte 0x66, 0x0f, 0x38, 0xdd, 0xc2

	movups STATE, (OUTP)	# store output
	sub $16, LEN
	add $16, INP
	add $16, OUTP
	cmp $16, LEN
	jge .Lcbc_enc_loop
	emms
	pop %ebp
	pop %ebx
	pop %esi
	pop %edi
	ret

	.size intel_aes_cbc_encrypt,.-intel_aes_cbc_encrypt
	.type intel_aes_cbc_encrypt,@function

/*
 * void intel_aes_cbc_decrypt(const aes_context *, void *src, void *dst, size_t len, void *iv)
 *
 * Stack after reg saves:   36(%esp) = void *iv
 *                          32(%esp) = size_t len
 *                          28(%esp) = void *dst
 *                          24(%esp) = void *src
 *                          20(%esp) = aes_context *
 */
	.align	ALIGN64BYTES
intel_aes_cbc_decrypt:
	push %edi
	push %esi
	push %ebx
	push %ebp
	mov 20(%esp),KEYP
	mov 24(%esp),INP
	mov 28(%esp),OUTP
	mov 32(%esp),LEN
	mov 36(%esp),IVP
	mov 4(KEYP), NRND
	add $264, KEYP
	movups (IVP), IV
	cmp $48, LEN
	jb .Lcbc_dec_loop1
.align 4
.Lcbc_dec_loop3:
	movups (INP), IN1
	movaps IN1, STATE1
	movups 0x10(INP), IN2
	movaps IN2, STATE2
	movups 0x20(INP), IN3
	movaps IN3, STATE3

	movaps (KEYP), KEY		# key
	mov KEYP, TKEYP
	pxor KEY, STATE1		# round 0
	pxor KEY, STATE2
	pxor KEY, STATE3
	add $0x30, TKEYP
	cmp $12, NRND
	jb .L4dec128
	lea 0x20(TKEYP), TKEYP
	je .L4dec192
	add $0x20, TKEYP
	movaps -0x60(TKEYP), KEY
	# aesdec KEY, STATE1
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	# aesdec KEY, STATE2
	.byte 0x66, 0x0f, 0x38, 0xde, 0xe2
	# aesdec KEY, STATE3
	.byte 0x66, 0x0f, 0x38, 0xde, 0xea
	movaps -0x50(TKEYP), KEY
	# aesdec KEY, STATE1
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	# aesdec KEY, STATE2
	.byte 0x66, 0x0f, 0x38, 0xde, 0xe2
	# aesdec KEY, STATE3
	.byte 0x66, 0x0f, 0x38, 0xde, 0xea
.align 4
.L4dec192:
	movaps -0x40(TKEYP), KEY
	# aesdec KEY, STATE1
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	# aesdec KEY, STATE2
	.byte 0x66, 0x0f, 0x38, 0xde, 0xe2
	# aesdec KEY, STATE3
	.byte 0x66, 0x0f, 0x38, 0xde, 0xea
	movaps -0x30(TKEYP), KEY
	# aesdec KEY, STATE1
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	# aesdec KEY, STATE2
	.byte 0x66, 0x0f, 0x38, 0xde, 0xe2
	# aesdec KEY, STATE3
	.byte 0x66, 0x0f, 0x38, 0xde, 0xea
.align 4
.L4dec128:
	movaps -0x20(TKEYP), KEY
	# aesdec KEY, STATE1
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	# aesdec KEY, STATE2
	.byte 0x66, 0x0f, 0x38, 0xde, 0xe2
	# aesdec KEY, STATE3
	.byte 0x66, 0x0f, 0x38, 0xde, 0xea
	movaps -0x10(TKEYP), KEY
	# aesdec KEY, STATE1
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	# aesdec KEY, STATE2
	.byte 0x66, 0x0f, 0x38, 0xde, 0xe2
	# aesdec KEY, STATE3
	.byte 0x66, 0x0f, 0x38, 0xde, 0xea
	movaps (TKEYP), KEY
	# aesdec KEY, STATE1
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	# aesdec KEY, STATE2
	.byte 0x66, 0x0f, 0x38, 0xde, 0xe2
	# aesdec KEY, STATE3
	.byte 0x66, 0x0f, 0x38, 0xde, 0xea
	movaps 0x10(TKEYP), KEY
	# aesdec KEY, STATE1
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	# aesdec KEY, STATE2
	.byte 0x66, 0x0f, 0x38, 0xde, 0xe2
	# aesdec KEY, STATE3
	.byte 0x66, 0x0f, 0x38, 0xde, 0xea
	movaps 0x20(TKEYP), KEY
	# aesdec KEY, STATE1
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	# aesdec KEY, STATE2
	.byte 0x66, 0x0f, 0x38, 0xde, 0xe2
	# aesdec KEY, STATE3
	.byte 0x66, 0x0f, 0x38, 0xde, 0xea
	movaps 0x30(TKEYP), KEY
	# aesdec KEY, STATE1
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	# aesdec KEY, STATE2
	.byte 0x66, 0x0f, 0x38, 0xde, 0xe2
	# aesdec KEY, STATE3
	.byte 0x66, 0x0f, 0x38, 0xde, 0xea
	movaps 0x40(TKEYP), KEY
	# aesdec KEY, STATE1
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	# aesdec KEY, STATE2
	.byte 0x66, 0x0f, 0x38, 0xde, 0xe2
	# aesdec KEY, STATE3
	.byte 0x66, 0x0f, 0x38, 0xde, 0xea
	movaps 0x50(TKEYP), KEY
	# aesdec KEY, STATE1
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	# aesdec KEY, STATE2
	.byte 0x66, 0x0f, 0x38, 0xde, 0xe2
	# aesdec KEY, STATE3
	.byte 0x66, 0x0f, 0x38, 0xde, 0xea
	movaps 0x60(TKEYP), KEY
	# aesdec KEY, STATE1
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	# aesdec KEY, STATE2
	.byte 0x66, 0x0f, 0x38, 0xde, 0xe2
	# aesdec KEY, STATE3
	.byte 0x66, 0x0f, 0x38, 0xde, 0xea
	movaps 0x70(TKEYP), KEY
	# aesdeclast KEY, STATE1	# last round
	.byte 0x66, 0x0f, 0x38, 0xdf, 0xc2
	# aesdeclast KEY, STATE2
	.byte 0x66, 0x0f, 0x38, 0xdf, 0xe2
	# aesdeclast KEY, STATE3
	.byte 0x66, 0x0f, 0x38, 0xdf, 0xea

	pxor IV, STATE1
	pxor IN1, STATE2
	pxor IN2, STATE3
	movaps IN3, IV
	movups STATE1, (OUTP)
	movups STATE2, 0x10(OUTP)
	movups STATE3, 0x20(OUTP)
	sub $48, LEN
	add $48, INP
	add $48, OUTP
	cmp $48, LEN
	jge .Lcbc_dec_loop3
	cmp $16, LEN
	jb .Lcbc_dec_ret
.align 4
.Lcbc_dec_loop1:
	movups (INP), IN
	movaps IN, STATE

	movaps (KEYP), KEY		# key
	mov KEYP, TKEYP
	pxor KEY, STATE		# round 0
	add $0x30, TKEYP
	cmp $12, NRND
	jb .Ldec128
	lea 0x20(TKEYP), TKEYP
	je .Ldec192
	add $0x20, TKEYP
	movaps -0x60(TKEYP), KEY
	# aesdec KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	movaps -0x50(TKEYP), KEY
	# aesdec KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
.align 4
.Ldec192:
	movaps -0x40(TKEYP), KEY
	# aesdec KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	movaps -0x30(TKEYP), KEY
	# aesdec KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
.align 4
.Ldec128:
	movaps -0x20(TKEYP), KEY
	# aesdec KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	movaps -0x10(TKEYP), KEY
	# aesdec KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	movaps (TKEYP), KEY
	# aesdec KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	movaps 0x10(TKEYP), KEY
	# aesdec KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	movaps 0x20(TKEYP), KEY
	# aesdec KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	movaps 0x30(TKEYP), KEY
	# aesdec KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	movaps 0x40(TKEYP), KEY
	# aesdec KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	movaps 0x50(TKEYP), KEY
	# aesdec KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	movaps 0x60(TKEYP), KEY
	# aesdec KEY, STATE
	.byte 0x66, 0x0f, 0x38, 0xde, 0xc2
	movaps 0x70(TKEYP), KEY
	# aesdeclast KEY, STATE		# last round
	.byte 0x66, 0x0f, 0x38, 0xdf, 0xc2

	pxor IV, STATE
	movups STATE, (OUTP)
	movaps IN, IV
	sub $16, LEN
	add $16, INP
	add $16, OUTP
	cmp $16, LEN
	jge .Lcbc_dec_loop1
.Lcbc_dec_ret:
	emms
	pop %ebp
	pop %ebx
	pop %esi
	pop %edi
	ret

	.size intel_aes_cbc_decrypt,.-intel_aes_cbc_decrypt
	.type intel_aes_cbc_decrypt,@function

/*
 * void intel_aes_cbc_enc_4x512(aes_context **, void *src, void *dst, void *iv)
 *
 * Stack after reg saves:   32(%esp) = void *iv
 *                          28(%esp) = void *dst
 *                          24(%esp) = void *src
 *                          20(%esp) = aes_context **
 */
	.align	ALIGN64BYTES
intel_aes_cbc_enc_4x512:
	push %edi
	push %esi
	push %ebx
	push %ebp
	mov 20(%esp),%edi
	mov 32(%esp),%esi
	mov 24(%esp),%ebp
	mov (%edi),%eax			# pointer to context struct 1
	mov 4(%edi),%ebx		# pointer to context struct 2
	mov 8(%edi),%ecx		# pointer to context struct 3
	mov 12(%edi),%edx		# pointer to context struct 4
	mov 4(%eax),%edi		# number of rounds (10/12/14)
	movups (%esi),%xmm0		# load IV as initial state
	movups 0x10(%esi),%xmm1
	movups 0x20(%esi),%xmm2
	movups 0x30(%esi),%xmm3
	sub $10,%edi
	mov $0x200,%esi			# 512 byte CBC chain
	shl $4,%edi
	add $0x38,%edi			# 0x38 / 0x58 / 0x78
.align 4
.Lcbc_enc_loop4:
	movups (%ebp),%xmm4		# load input
	movups 0x200(%ebp),%xmm5
	movups 0x400(%ebp),%xmm6
	movups 0x600(%ebp),%xmm7
	add $16,%ebp
	mov %ebp,24(%esp)
	mov 28(%esp),%ebp
	pxor %xmm4,%xmm0		# CBC-mode XOR
	pxor %xmm5,%xmm1
	pxor %xmm6,%xmm2
	pxor %xmm7,%xmm3

	movaps 0x08(%eax),%xmm4		# round 0 key
	movaps 0x08(%ebx),%xmm5
	movaps 0x08(%ecx),%xmm6
	movaps 0x08(%edx),%xmm7
	pxor %xmm4,%xmm0		# round 0 XOR
	pxor %xmm5,%xmm1
	pxor %xmm6,%xmm2
	pxor %xmm7,%xmm3

	cmp $0x58,%edi
	jb .L4enc128
	je .L4enc192

	movaps -0x60(%eax,%edi,1),%xmm4
	movaps -0x60(%ebx,%edi,1),%xmm5
	movaps -0x60(%ecx,%edi,1),%xmm6
	movaps -0x60(%edx,%edi,1),%xmm7
	# aesenc %xmm4,%xmm0
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc4
	# aesenc %xmm5,%xmm1
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xcd
	# aesenc %xmm6,%xmm2
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xd6
	# aesenc %xmm7,%xmm3
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xdf

	movaps -0x50(%eax,%edi,1),%xmm4
	movaps -0x50(%ebx,%edi,1),%xmm5
	movaps -0x50(%ecx,%edi,1),%xmm6
	movaps -0x50(%edx,%edi,1),%xmm7
	# aesenc %xmm4,%xmm0
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc4
	# aesenc %xmm5,%xmm1
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xcd
	# aesenc %xmm6,%xmm2
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xd6
	# aesenc %xmm7,%xmm3
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xdf

.align 4
.L4enc192:
	movaps -0x40(%eax,%edi,1),%xmm4
	movaps -0x40(%ebx,%edi,1),%xmm5
	movaps -0x40(%ecx,%edi,1),%xmm6
	movaps -0x40(%edx,%edi,1),%xmm7
	# aesenc %xmm4,%xmm0
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc4
	# aesenc %xmm5,%xmm1
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xcd
	# aesenc %xmm6,%xmm2
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xd6
	# aesenc %xmm7,%xmm3
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xdf

	movaps -0x30(%eax,%edi,1),%xmm4
	movaps -0x30(%ebx,%edi,1),%xmm5
	movaps -0x30(%ecx,%edi,1),%xmm6
	movaps -0x30(%edx,%edi,1),%xmm7
	# aesenc %xmm4,%xmm0
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc4
	# aesenc %xmm5,%xmm1
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xcd
	# aesenc %xmm6,%xmm2
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xd6
	# aesenc %xmm7,%xmm3
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xdf

.align 4
.L4enc128:
	movaps -0x20(%eax,%edi,1),%xmm4
	movaps -0x20(%ebx,%edi,1),%xmm5
	movaps -0x20(%ecx,%edi,1),%xmm6
	movaps -0x20(%edx,%edi,1),%xmm7
	# aesenc %xmm4,%xmm0
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc4
	# aesenc %xmm5,%xmm1
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xcd
	# aesenc %xmm6,%xmm2
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xd6
	# aesenc %xmm7,%xmm3
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xdf

	movaps -0x10(%eax,%edi,1),%xmm4
	movaps -0x10(%ebx,%edi,1),%xmm5
	movaps -0x10(%ecx,%edi,1),%xmm6
	movaps -0x10(%edx,%edi,1),%xmm7
	# aesenc %xmm4,%xmm0
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc4
	# aesenc %xmm5,%xmm1
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xcd
	# aesenc %xmm6,%xmm2
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xd6
	# aesenc %xmm7,%xmm3
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xdf

	movaps (%eax,%edi,1),%xmm4
	movaps (%ebx,%edi,1),%xmm5
	movaps (%ecx,%edi,1),%xmm6
	movaps (%edx,%edi,1),%xmm7
	# aesenc %xmm4,%xmm0
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc4
	# aesenc %xmm5,%xmm1
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xcd
	# aesenc %xmm6,%xmm2
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xd6
	# aesenc %xmm7,%xmm3
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xdf

	movaps 0x10(%eax,%edi,1),%xmm4
	movaps 0x10(%ebx,%edi,1),%xmm5
	movaps 0x10(%ecx,%edi,1),%xmm6
	movaps 0x10(%edx,%edi,1),%xmm7
	# aesenc %xmm4,%xmm0
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc4
	# aesenc %xmm5,%xmm1
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xcd
	# aesenc %xmm6,%xmm2
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xd6
	# aesenc %xmm7,%xmm3
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xdf

	movaps 0x20(%eax,%edi,1),%xmm4
	movaps 0x20(%ebx,%edi,1),%xmm5
	movaps 0x20(%ecx,%edi,1),%xmm6
	movaps 0x20(%edx,%edi,1),%xmm7
	# aesenc %xmm4,%xmm0
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc4
	# aesenc %xmm5,%xmm1
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xcd
	# aesenc %xmm6,%xmm2
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xd6
	# aesenc %xmm7,%xmm3
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xdf

	movaps 0x30(%eax,%edi,1),%xmm4
	movaps 0x30(%ebx,%edi,1),%xmm5
	movaps 0x30(%ecx,%edi,1),%xmm6
	movaps 0x30(%edx,%edi,1),%xmm7
	# aesenc %xmm4,%xmm0
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc4
	# aesenc %xmm5,%xmm1
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xcd
	# aesenc %xmm6,%xmm2
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xd6
	# aesenc %xmm7,%xmm3
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xdf

	movaps 0x40(%eax,%edi,1),%xmm4
	movaps 0x40(%ebx,%edi,1),%xmm5
	movaps 0x40(%ecx,%edi,1),%xmm6
	movaps 0x40(%edx,%edi,1),%xmm7
	# aesenc %xmm4,%xmm0
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc4
	# aesenc %xmm5,%xmm1
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xcd
	# aesenc %xmm6,%xmm2
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xd6
	# aesenc %xmm7,%xmm3
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xdf

	movaps 0x50(%eax,%edi,1),%xmm4
	movaps 0x50(%ebx,%edi,1),%xmm5
	movaps 0x50(%ecx,%edi,1),%xmm6
	movaps 0x50(%edx,%edi,1),%xmm7
	# aesenc %xmm4,%xmm0
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc4
	# aesenc %xmm5,%xmm1
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xcd
	# aesenc %xmm6,%xmm2
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xd6
	# aesenc %xmm7,%xmm3
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xdf

	movaps 0x60(%eax,%edi,1),%xmm4
	movaps 0x60(%ebx,%edi,1),%xmm5
	movaps 0x60(%ecx,%edi,1),%xmm6
	movaps 0x60(%edx,%edi,1),%xmm7
	# aesenc %xmm4,%xmm0
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xc4
	# aesenc %xmm5,%xmm1
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xcd
	# aesenc %xmm6,%xmm2
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xd6
	# aesenc %xmm7,%xmm3
	.byte 0x66, 0x0f, 0x38, 0xdc, 0xdf

	movaps 0x70(%eax,%edi,1),%xmm4
	movaps 0x70(%ebx,%edi,1),%xmm5
	movaps 0x70(%ecx,%edi,1),%xmm6
	movaps 0x70(%edx,%edi,1),%xmm7
	# aesenclast %xmm4,%xmm0	# last round
	.byte 0x66, 0x0f, 0x38, 0xdd, 0xc4
	# aesenclast %xmm5,%xmm1
	.byte 0x66, 0x0f, 0x38, 0xdd, 0xcd
	# aesenclast %xmm6,%xmm2
	.byte 0x66, 0x0f, 0x38, 0xdd, 0xd6
	# aesenclast %xmm7,%xmm3
	.byte 0x66, 0x0f, 0x38, 0xdd, 0xdf

	sub $16,%esi
	movups %xmm0,(%ebp)		# store output
	movups %xmm1,0x200(%ebp)
	movups %xmm2,0x400(%ebp)
	movups %xmm3,0x600(%ebp)
	add $16,%ebp
	mov %ebp,28(%esp)
	mov 24(%esp),%ebp
	cmp $16,%esi
	jge .Lcbc_enc_loop4
	emms
	pop %ebp
	pop %ebx
	pop %esi
	pop %edi
	ret

	.size intel_aes_cbc_enc_4x512,.-intel_aes_cbc_enc_4x512
	.type intel_aes_cbc_enc_4x512,@function

#if defined(__ELF__) && defined(SECTION_NOTE_GNU_STACK)
	.section .note.GNU-stack,"",@progbits
#endif
