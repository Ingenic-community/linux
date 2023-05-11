//
// Copyright (c) 2001, Dr Brian Gladman <brg@gladman.uk.net>, Worcester, UK.
// All rights reserved.
//
// TERMS
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted subject to the following conditions:
//
//  1. Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//  3. The copyright holder's name must not be used to endorse or promote
//     any products derived from this software without his specific prior
//     written permission.
//
//  This software is provided 'as is' with no express or implied warranties
//  of correctness or fitness for purpose.

// Modified by Jari Ruusu,  December 24 2001
//  - Converted syntax to GNU CPP/assembler syntax
//  - C programming interface converted back to "old" API
//  - Minor portability cleanups and speed optimizations

// Modified by Jari Ruusu,  April 11 2002
//  - Added above copyright and terms to resulting object code so that
//    binary distributions can avoid legal trouble

// Modified by Jari Ruusu,  June 12 2004
//  - Converted 32 bit x86 code to 64 bit AMD64 code
//  - Re-wrote encrypt and decrypt code from scratch

// An AES (Rijndael) implementation for the AMD64. This version only
// implements the standard AES block length (128 bits, 16 bytes). This code
// does not preserve the rax, rcx, rdx, rsi, rdi or r8-r11 registers or the
// artihmetic status flags. However, the rbx, rbp and r12-r15 registers are
// preserved across calls.

// void aes_set_key(aes_context *cx, const unsigned char key[], const int key_len, const int f)
// void aes_encrypt(const aes_context *cx, const unsigned char in_blk[], unsigned char out_blk[])
// void aes_decrypt(const aes_context *cx, const unsigned char in_blk[], unsigned char out_blk[])

#if defined(USE_UNDERLINE)
# define aes_set_key _aes_set_key
# define aes_encrypt _aes_encrypt
# define aes_decrypt _aes_decrypt
#endif
#if !defined(ALIGN64BYTES)
# define ALIGN64BYTES 64
#endif

	.file	"aes-amd64.S"
	.globl	aes_set_key
	.globl	aes_encrypt
	.globl	aes_decrypt

	.section .rodata
copyright:
	.ascii "    \000"
	.ascii "Copyright (c) 2001, Dr Brian Gladman <brg@gladman.uk.net>, Worcester, UK.\000"
	.ascii "All rights reserved.\000"
	.ascii "    \000"
	.ascii "TERMS\000"
	.ascii "    \000"
	.ascii " Redistribution and use in source and binary forms, with or without\000"
	.ascii " modification, are permitted subject to the following conditions:\000"
	.ascii "    \000"
	.ascii " 1. Redistributions of source code must retain the above copyright\000"
	.ascii "    notice, this list of conditions and the following disclaimer.\000"
	.ascii "    \000"
	.ascii " 2. Redistributions in binary form must reproduce the above copyright\000"
	.ascii "    notice, this list of conditions and the following disclaimer in the\000"
	.ascii "    documentation and/or other materials provided with the distribution.\000"
	.ascii "    \000"
	.ascii " 3. The copyright holder's name must not be used to endorse or promote\000"
	.ascii "    any products derived from this software without his specific prior\000"
	.ascii "    written permission.\000"
	.ascii "    \000"
	.ascii " This software is provided 'as is' with no express or implied warranties\000"
	.ascii " of correctness or fitness for purpose.\000"
	.ascii "    \000"

#define tlen	1024	// length of each of 4 'xor' arrays (256 32-bit words)

// offsets in context structure

#define nkey	0	// key length, size 4
#define nrnd	4	// number of rounds, size 4
#define ekey	8	// encryption key schedule base address, size 256
#define dkey	264	// decryption key schedule base address, size 256

// This macro performs a forward encryption cycle. It is entered with
// the first previous round column values in I1E, I2E, I3E and I4E and
// exits with the final values OU1, OU2, OU3 and OU4 registers.

#define fwd_rnd(p1,p2,I1E,I1B,I1H,I2E,I2B,I2H,I3E,I3B,I3R,I4E,I4B,I4R,OU1,OU2,OU3,OU4) \
	movl	p2(%rbp),OU1		;\
	movl	p2+4(%rbp),OU2		;\
	movl	p2+8(%rbp),OU3		;\
	movl	p2+12(%rbp),OU4		;\
	movzbl	I1B,%edi		;\
	movzbl	I2B,%esi		;\
	movzbl	I3B,%r8d		;\
	movzbl	I4B,%r13d		;\
	shrl	$8,I3E			;\
	shrl	$8,I4E			;\
	xorl	p1(,%rdi,4),OU1		;\
	xorl	p1(,%rsi,4),OU2		;\
	xorl	p1(,%r8,4),OU3		;\
	xorl	p1(,%r13,4),OU4		;\
	movzbl	I2H,%esi		;\
	movzbl	I3B,%r8d		;\
	movzbl	I4B,%r13d		;\
	movzbl	I1H,%edi		;\
	shrl	$8,I3E			;\
	shrl	$8,I4E			;\
	xorl	p1+tlen(,%rsi,4),OU1	;\
	xorl	p1+tlen(,%r8,4),OU2	;\
	xorl	p1+tlen(,%r13,4),OU3	;\
	xorl	p1+tlen(,%rdi,4),OU4	;\
	shrl	$16,I1E			;\
	shrl	$16,I2E			;\
	movzbl	I3B,%r8d		;\
	movzbl	I4B,%r13d		;\
	movzbl	I1B,%edi		;\
	movzbl	I2B,%esi		;\
	xorl	p1+2*tlen(,%r8,4),OU1	;\
	xorl	p1+2*tlen(,%r13,4),OU2	;\
	xorl	p1+2*tlen(,%rdi,4),OU3	;\
	xorl	p1+2*tlen(,%rsi,4),OU4	;\
	shrl	$8,I4E			;\
	movzbl	I1H,%edi		;\
	movzbl	I2H,%esi		;\
	shrl	$8,I3E			;\
	xorl	p1+3*tlen(,I4R,4),OU1	;\
	xorl	p1+3*tlen(,%rdi,4),OU2	;\
	xorl	p1+3*tlen(,%rsi,4),OU3	;\
	xorl	p1+3*tlen(,I3R,4),OU4

// This macro performs an inverse encryption cycle. It is entered with
// the first previous round column values in I1E, I2E, I3E and I4E and
// exits with the final values OU1, OU2, OU3 and OU4 registers.

#define inv_rnd(p1,p2,I1E,I1B,I1R,I2E,I2B,I2R,I3E,I3B,I3H,I4E,I4B,I4H,OU1,OU2,OU3,OU4) \
	movl	p2+12(%rbp),OU4		;\
	movl	p2+8(%rbp),OU3		;\
	movl	p2+4(%rbp),OU2		;\
	movl	p2(%rbp),OU1		;\
	movzbl	I4B,%edi		;\
	movzbl	I3B,%esi		;\
	movzbl	I2B,%r8d		;\
	movzbl	I1B,%r13d		;\
	shrl	$8,I2E			;\
	shrl	$8,I1E			;\
	xorl	p1(,%rdi,4),OU4		;\
	xorl	p1(,%rsi,4),OU3		;\
	xorl	p1(,%r8,4),OU2		;\
	xorl	p1(,%r13,4),OU1		;\
	movzbl	I3H,%esi		;\
	movzbl	I2B,%r8d		;\
	movzbl	I1B,%r13d		;\
	movzbl	I4H,%edi		;\
	shrl	$8,I2E			;\
	shrl	$8,I1E			;\
	xorl	p1+tlen(,%rsi,4),OU4	;\
	xorl	p1+tlen(,%r8,4),OU3	;\
	xorl	p1+tlen(,%r13,4),OU2	;\
	xorl	p1+tlen(,%rdi,4),OU1	;\
	shrl	$16,I4E			;\
	shrl	$16,I3E			;\
	movzbl	I2B,%r8d		;\
	movzbl	I1B,%r13d		;\
	movzbl	I4B,%edi		;\
	movzbl	I3B,%esi		;\
	xorl	p1+2*tlen(,%r8,4),OU4	;\
	xorl	p1+2*tlen(,%r13,4),OU3	;\
	xorl	p1+2*tlen(,%rdi,4),OU2	;\
	xorl	p1+2*tlen(,%rsi,4),OU1	;\
	shrl	$8,I1E			;\
	movzbl	I4H,%edi		;\
	movzbl	I3H,%esi		;\
	shrl	$8,I2E			;\
	xorl	p1+3*tlen(,I1R,4),OU4	;\
	xorl	p1+3*tlen(,%rdi,4),OU3	;\
	xorl	p1+3*tlen(,%rsi,4),OU2	;\
	xorl	p1+3*tlen(,I2R,4),OU1

// AES (Rijndael) Encryption Subroutine

// rdi = pointer to AES context
// rsi = pointer to input plaintext bytes
// rdx = pointer to output ciphertext bytes

	.text
	.align	ALIGN64BYTES
aes_encrypt:
	movl	(%rsi),%eax		// read in plaintext
	movl	4(%rsi),%ecx
	movl	8(%rsi),%r10d
	movl	12(%rsi),%r11d

	pushq	%rbp
	leaq	ekey+16(%rdi),%rbp	// encryption key pointer
	movq	%rdx,%r9		// pointer to out block
	movl	nrnd(%rdi),%edx		// number of rounds
	pushq	%rbx
	pushq	%r13
	pushq	%r14
	pushq	%r15

	xorl	-16(%rbp),%eax		// xor in first round key
	xorl	-12(%rbp),%ecx
	xorl	-8(%rbp),%r10d
	xorl	-4(%rbp),%r11d

	subl	$10,%edx
	je	aes_15
	addq	$32,%rbp
	subl	$2,%edx
	je	aes_13
	addq	$32,%rbp

	fwd_rnd(aes_ft_tab,-64,%eax,%al,%ah,%ecx,%cl,%ch,%r10d,%r10b,%r10,%r11d,%r11b,%r11,%ebx,%edx,%r14d,%r15d)
	fwd_rnd(aes_ft_tab,-48,%ebx,%bl,%bh,%edx,%dl,%dh,%r14d,%r14b,%r14,%r15d,%r15b,%r15,%eax,%ecx,%r10d,%r11d)
	jmp	aes_13
	.align	ALIGN64BYTES
aes_13:	fwd_rnd(aes_ft_tab,-32,%eax,%al,%ah,%ecx,%cl,%ch,%r10d,%r10b,%r10,%r11d,%r11b,%r11,%ebx,%edx,%r14d,%r15d)
	fwd_rnd(aes_ft_tab,-16,%ebx,%bl,%bh,%edx,%dl,%dh,%r14d,%r14b,%r14,%r15d,%r15b,%r15,%eax,%ecx,%r10d,%r11d)
	jmp	aes_15
	.align	ALIGN64BYTES
aes_15:	fwd_rnd(aes_ft_tab,0,  %eax,%al,%ah,%ecx,%cl,%ch,%r10d,%r10b,%r10,%r11d,%r11b,%r11,%ebx,%edx,%r14d,%r15d)
	fwd_rnd(aes_ft_tab,16, %ebx,%bl,%bh,%edx,%dl,%dh,%r14d,%r14b,%r14,%r15d,%r15b,%r15,%eax,%ecx,%r10d,%r11d)
	fwd_rnd(aes_ft_tab,32, %eax,%al,%ah,%ecx,%cl,%ch,%r10d,%r10b,%r10,%r11d,%r11b,%r11,%ebx,%edx,%r14d,%r15d)
	fwd_rnd(aes_ft_tab,48, %ebx,%bl,%bh,%edx,%dl,%dh,%r14d,%r14b,%r14,%r15d,%r15b,%r15,%eax,%ecx,%r10d,%r11d)
	fwd_rnd(aes_ft_tab,64, %eax,%al,%ah,%ecx,%cl,%ch,%r10d,%r10b,%r10,%r11d,%r11b,%r11,%ebx,%edx,%r14d,%r15d)
	fwd_rnd(aes_ft_tab,80, %ebx,%bl,%bh,%edx,%dl,%dh,%r14d,%r14b,%r14,%r15d,%r15b,%r15,%eax,%ecx,%r10d,%r11d)
	fwd_rnd(aes_ft_tab,96, %eax,%al,%ah,%ecx,%cl,%ch,%r10d,%r10b,%r10,%r11d,%r11b,%r11,%ebx,%edx,%r14d,%r15d)
	fwd_rnd(aes_ft_tab,112,%ebx,%bl,%bh,%edx,%dl,%dh,%r14d,%r14b,%r14,%r15d,%r15b,%r15,%eax,%ecx,%r10d,%r11d)
	fwd_rnd(aes_ft_tab,128,%eax,%al,%ah,%ecx,%cl,%ch,%r10d,%r10b,%r10,%r11d,%r11b,%r11,%ebx,%edx,%r14d,%r15d)
	fwd_rnd(aes_fl_tab,144,%ebx,%bl,%bh,%edx,%dl,%dh,%r14d,%r14b,%r14,%r15d,%r15b,%r15,%eax,%ecx,%r10d,%r11d)

	popq	%r15
	popq	%r14
	popq	%r13
	popq	%rbx
	popq	%rbp

	movl	%eax,(%r9)		// move final values to the output array.
	movl	%ecx,4(%r9)
	movl	%r10d,8(%r9)
	movl	%r11d,12(%r9)
	ret

	.size aes_encrypt,.-aes_encrypt
	.type aes_encrypt,@function

// AES (Rijndael) Decryption Subroutine

// rdi = pointer to AES context
// rsi = pointer to input ciphertext bytes
// rdx = pointer to output plaintext bytes

	.align	ALIGN64BYTES
aes_decrypt:
	movl	12(%rsi),%eax		// read in ciphertext
	movl	8(%rsi),%ecx
	movl	4(%rsi),%r10d
	movl	(%rsi),%r11d

	pushq	%rbp
	leaq	dkey+16(%rdi),%rbp	// decryption key pointer
	movq	%rdx,%r9		// pointer to out block
	movl	nrnd(%rdi),%edx		// number of rounds
	pushq	%rbx
	pushq	%r13
	pushq	%r14
	pushq	%r15

	xorl	-4(%rbp),%eax		// xor in first round key
	xorl	-8(%rbp),%ecx
	xorl	-12(%rbp),%r10d
	xorl	-16(%rbp),%r11d

	subl	$10,%edx
	je	aes_25
	addq	$32,%rbp
	subl	$2,%edx
	je	aes_23
	addq	$32,%rbp

	inv_rnd(aes_it_tab,-64,%r11d,%r11b,%r11,%r10d,%r10b,%r10,%ecx,%cl,%ch,%eax,%al,%ah,%r15d,%r14d,%edx,%ebx)
	inv_rnd(aes_it_tab,-48,%r15d,%r15b,%r15,%r14d,%r14b,%r14,%edx,%dl,%dh,%ebx,%bl,%bh,%r11d,%r10d,%ecx,%eax)
	jmp	aes_23
	.align	ALIGN64BYTES
aes_23:	inv_rnd(aes_it_tab,-32,%r11d,%r11b,%r11,%r10d,%r10b,%r10,%ecx,%cl,%ch,%eax,%al,%ah,%r15d,%r14d,%edx,%ebx)
	inv_rnd(aes_it_tab,-16,%r15d,%r15b,%r15,%r14d,%r14b,%r14,%edx,%dl,%dh,%ebx,%bl,%bh,%r11d,%r10d,%ecx,%eax)
	jmp	aes_25
	.align	ALIGN64BYTES
aes_25:	inv_rnd(aes_it_tab,0,  %r11d,%r11b,%r11,%r10d,%r10b,%r10,%ecx,%cl,%ch,%eax,%al,%ah,%r15d,%r14d,%edx,%ebx)
	inv_rnd(aes_it_tab,16, %r15d,%r15b,%r15,%r14d,%r14b,%r14,%edx,%dl,%dh,%ebx,%bl,%bh,%r11d,%r10d,%ecx,%eax)
	inv_rnd(aes_it_tab,32, %r11d,%r11b,%r11,%r10d,%r10b,%r10,%ecx,%cl,%ch,%eax,%al,%ah,%r15d,%r14d,%edx,%ebx)
	inv_rnd(aes_it_tab,48, %r15d,%r15b,%r15,%r14d,%r14b,%r14,%edx,%dl,%dh,%ebx,%bl,%bh,%r11d,%r10d,%ecx,%eax)
	inv_rnd(aes_it_tab,64, %r11d,%r11b,%r11,%r10d,%r10b,%r10,%ecx,%cl,%ch,%eax,%al,%ah,%r15d,%r14d,%edx,%ebx)
	inv_rnd(aes_it_tab,80, %r15d,%r15b,%r15,%r14d,%r14b,%r14,%edx,%dl,%dh,%ebx,%bl,%bh,%r11d,%r10d,%ecx,%eax)
	inv_rnd(aes_it_tab,96, %r11d,%r11b,%r11,%r10d,%r10b,%r10,%ecx,%cl,%ch,%eax,%al,%ah,%r15d,%r14d,%edx,%ebx)
	inv_rnd(aes_it_tab,112,%r15d,%r15b,%r15,%r14d,%r14b,%r14,%edx,%dl,%dh,%ebx,%bl,%bh,%r11d,%r10d,%ecx,%eax)
	inv_rnd(aes_it_tab,128,%r11d,%r11b,%r11,%r10d,%r10b,%r10,%ecx,%cl,%ch,%eax,%al,%ah,%r15d,%r14d,%edx,%ebx)
	inv_rnd(aes_il_tab,144,%r15d,%r15b,%r15,%r14d,%r14b,%r14,%edx,%dl,%dh,%ebx,%bl,%bh,%r11d,%r10d,%ecx,%eax)

	popq	%r15
	popq	%r14
	popq	%r13
	popq	%rbx
	popq	%rbp

	movl	%eax,12(%r9)		// move final values to the output array.
	movl	%ecx,8(%r9)
	movl	%r10d,4(%r9)
	movl	%r11d,(%r9)
	ret

	.size aes_decrypt,.-aes_decrypt
	.type aes_decrypt,@function

// AES (Rijndael) Key Schedule Subroutine

// This macro performs a column mixing operation on an input 32-bit
// word to give a 32-bit result. It uses each of the 4 bytes in the
// the input column to index 4 different tables of 256 32-bit words
// that are xored together to form the output value.

#define mix_col(p1)			 \
	movzbl	%bl,%ecx		;\
	movl	p1(,%rcx,4),%eax	;\
	movzbl	%bh,%ecx		;\
	ror	$16,%ebx		;\
	xorl	p1+tlen(,%rcx,4),%eax	;\
	movzbl	%bl,%ecx		;\
	xorl	p1+2*tlen(,%rcx,4),%eax	;\
	movzbl	%bh,%ecx		;\
	xorl	p1+3*tlen(,%rcx,4),%eax

// Key Schedule Macros

#define ksc4(p1)			 \
	rol	$24,%ebx		;\
	mix_col(aes_fl_tab)		;\
	ror	$8,%ebx			;\
	xorl	4*p1+aes_rcon_tab,%eax	;\
	xorl	%eax,%esi		;\
	xorl	%esi,%ebp		;\
	movl	%esi,16*p1(%rdi)	;\
	movl	%ebp,16*p1+4(%rdi)	;\
	xorl	%ebp,%edx		;\
	xorl	%edx,%ebx		;\
	movl	%edx,16*p1+8(%rdi)	;\
	movl	%ebx,16*p1+12(%rdi)

#define ksc6(p1)			 \
	rol	$24,%ebx		;\
	mix_col(aes_fl_tab)		;\
	ror	$8,%ebx			;\
	xorl	4*p1+aes_rcon_tab,%eax	;\
	xorl	24*p1-24(%rdi),%eax	;\
	movl	%eax,24*p1(%rdi)	;\
	xorl	24*p1-20(%rdi),%eax	;\
	movl	%eax,24*p1+4(%rdi)	;\
	xorl	%eax,%esi		;\
	xorl	%esi,%ebp		;\
	movl	%esi,24*p1+8(%rdi)	;\
	movl	%ebp,24*p1+12(%rdi)	;\
	xorl	%ebp,%edx		;\
	xorl	%edx,%ebx		;\
	movl	%edx,24*p1+16(%rdi)	;\
	movl	%ebx,24*p1+20(%rdi)

#define ksc8(p1)			 \
	rol	$24,%ebx		;\
	mix_col(aes_fl_tab)		;\
	ror	$8,%ebx			;\
	xorl	4*p1+aes_rcon_tab,%eax	;\
	xorl	32*p1-32(%rdi),%eax	;\
	movl	%eax,32*p1(%rdi)	;\
	xorl	32*p1-28(%rdi),%eax	;\
	movl	%eax,32*p1+4(%rdi)	;\
	xorl	32*p1-24(%rdi),%eax	;\
	movl	%eax,32*p1+8(%rdi)	;\
	xorl	32*p1-20(%rdi),%eax	;\
	movl	%eax,32*p1+12(%rdi)	;\
	pushq	%rbx			;\
	movl	%eax,%ebx		;\
	mix_col(aes_fl_tab)		;\
	popq	%rbx			;\
	xorl	%eax,%esi		;\
	xorl	%esi,%ebp		;\
	movl	%esi,32*p1+16(%rdi)	;\
	movl	%ebp,32*p1+20(%rdi)	;\
	xorl	%ebp,%edx		;\
	xorl	%edx,%ebx		;\
	movl	%edx,32*p1+24(%rdi)	;\
	movl	%ebx,32*p1+28(%rdi)

// rdi = pointer to AES context
// rsi = pointer to key bytes
// rdx = key length, bytes or bits
// rcx = ed_flag, 1=encrypt only, 0=both encrypt and decrypt

	.align	ALIGN64BYTES
aes_set_key:
	pushfq
	pushq	%rbp
	pushq	%rbx

	movq	%rcx,%r11		// ed_flg
	movq	%rdx,%rcx		// key length
	movq	%rdi,%r10		// AES context

	cmpl	$128,%ecx
	jb	aes_30
	shrl	$3,%ecx
aes_30:	cmpl	$32,%ecx
	je	aes_32
	cmpl	$24,%ecx
	je	aes_32
	movl	$16,%ecx
aes_32:	shrl	$2,%ecx
	movl	%ecx,nkey(%r10)
	leaq	6(%rcx),%rax		// 10/12/14 for 4/6/8 32-bit key length
	movl	%eax,nrnd(%r10)
	leaq	ekey(%r10),%rdi		// key position in AES context
	cld
	movl	%ecx,%eax		// save key length in eax
	rep ;	movsl			// words in the key schedule
	movl	-4(%rsi),%ebx		// put some values in registers
	movl	-8(%rsi),%edx		// to allow faster code
	movl	-12(%rsi),%ebp
	movl	-16(%rsi),%esi

	cmpl	$4,%eax			// jump on key size
	je	aes_36
	cmpl	$6,%eax
	je	aes_35

	ksc8(0)
	ksc8(1)
	ksc8(2)
	ksc8(3)
	ksc8(4)
	ksc8(5)
	ksc8(6)
	jmp	aes_37
aes_35:	ksc6(0)
	ksc6(1)
	ksc6(2)
	ksc6(3)
	ksc6(4)
	ksc6(5)
	ksc6(6)
	ksc6(7)
	jmp	aes_37
aes_36:	ksc4(0)
	ksc4(1)
	ksc4(2)
	ksc4(3)
	ksc4(4)
	ksc4(5)
	ksc4(6)
	ksc4(7)
	ksc4(8)
	ksc4(9)
aes_37:	cmpl	$0,%r11d		// ed_flg
	jne	aes_39

// compile decryption key schedule from encryption schedule - reverse
// order and do mix_column operation on round keys except first and last

	movl	nrnd(%r10),%eax		// kt = cx->d_key + nc * cx->Nrnd
	shl	$2,%rax
	leaq	dkey(%r10,%rax,4),%rdi
	leaq	ekey(%r10),%rsi		// kf = cx->e_key

	movsq				// copy first round key (unmodified)
	movsq
	subq	$32,%rdi
	movl	$1,%r9d
aes_38:					// do mix column on each column of
	lodsl				// each round key
	movl	%eax,%ebx
	mix_col(aes_im_tab)
	stosl
	lodsl
	movl	%eax,%ebx
	mix_col(aes_im_tab)
	stosl
	lodsl
	movl	%eax,%ebx
	mix_col(aes_im_tab)
	stosl
	lodsl
	movl	%eax,%ebx
	mix_col(aes_im_tab)
	stosl
	subq	$32,%rdi

	incl	%r9d
	cmpl	nrnd(%r10),%r9d
	jb	aes_38

	movsq				// copy last round key (unmodified)
	movsq
aes_39:	popq	%rbx
	popq	%rbp
	popfq
	ret

	.size aes_set_key,.-aes_set_key
	.type aes_set_key,@function

// finite field multiplies by {02}, {04} and {08}

#define f2(x)	((x<<1)^(((x>>7)&1)*0x11b))
#define f4(x)	((x<<2)^(((x>>6)&1)*0x11b)^(((x>>6)&2)*0x11b))
#define f8(x)	((x<<3)^(((x>>5)&1)*0x11b)^(((x>>5)&2)*0x11b)^(((x>>5)&4)*0x11b))

// finite field multiplies required in table generation

#define f3(x)	(f2(x) ^ x)
#define f9(x)	(f8(x) ^ x)
#define fb(x)	(f8(x) ^ f2(x) ^ x)
#define fd(x)	(f8(x) ^ f4(x) ^ x)
#define fe(x)	(f8(x) ^ f4(x) ^ f2(x))

// These defines generate the forward table entries

#define u0(x)	((f3(x) << 24) | (x << 16) | (x << 8) | f2(x))
#define u1(x)	((x << 24) | (x << 16) | (f2(x) << 8) | f3(x))
#define u2(x)	((x << 24) | (f2(x) << 16) | (f3(x) << 8) | x)
#define u3(x)	((f2(x) << 24) | (f3(x) << 16) | (x << 8) | x)

// These defines generate the inverse table entries

#define v0(x)	((fb(x) << 24) | (fd(x) << 16) | (f9(x) << 8) | fe(x))
#define v1(x)	((fd(x) << 24) | (f9(x) << 16) | (fe(x) << 8) | fb(x))
#define v2(x)	((f9(x) << 24) | (fe(x) << 16) | (fb(x) << 8) | fd(x))
#define v3(x)	((fe(x) << 24) | (fb(x) << 16) | (fd(x) << 8) | f9(x))

// These defines generate entries for the last round tables

#define w0(x)	(x)
#define w1(x)	(x <<  8)
#define w2(x)	(x << 16)
#define w3(x)	(x << 24)

// macro to generate inverse mix column tables (needed for the key schedule)

#define im_data0(p1) \
	.long	p1(0x00),p1(0x01),p1(0x02),p1(0x03),p1(0x04),p1(0x05),p1(0x06),p1(0x07) ;\
	.long	p1(0x08),p1(0x09),p1(0x0a),p1(0x0b),p1(0x0c),p1(0x0d),p1(0x0e),p1(0x0f) ;\
	.long	p1(0x10),p1(0x11),p1(0x12),p1(0x13),p1(0x14),p1(0x15),p1(0x16),p1(0x17) ;\
	.long	p1(0x18),p1(0x19),p1(0x1a),p1(0x1b),p1(0x1c),p1(0x1d),p1(0x1e),p1(0x1f)
#define im_data1(p1) \
	.long	p1(0x20),p1(0x21),p1(0x22),p1(0x23),p1(0x24),p1(0x25),p1(0x26),p1(0x27) ;\
	.long	p1(0x28),p1(0x29),p1(0x2a),p1(0x2b),p1(0x2c),p1(0x2d),p1(0x2e),p1(0x2f) ;\
	.long	p1(0x30),p1(0x31),p1(0x32),p1(0x33),p1(0x34),p1(0x35),p1(0x36),p1(0x37) ;\
	.long	p1(0x38),p1(0x39),p1(0x3a),p1(0x3b),p1(0x3c),p1(0x3d),p1(0x3e),p1(0x3f)
#define im_data2(p1) \
	.long	p1(0x40),p1(0x41),p1(0x42),p1(0x43),p1(0x44),p1(0x45),p1(0x46),p1(0x47) ;\
	.long	p1(0x48),p1(0x49),p1(0x4a),p1(0x4b),p1(0x4c),p1(0x4d),p1(0x4e),p1(0x4f) ;\
	.long	p1(0x50),p1(0x51),p1(0x52),p1(0x53),p1(0x54),p1(0x55),p1(0x56),p1(0x57) ;\
	.long	p1(0x58),p1(0x59),p1(0x5a),p1(0x5b),p1(0x5c),p1(0x5d),p1(0x5e),p1(0x5f)
#define im_data3(p1) \
	.long	p1(0x60),p1(0x61),p1(0x62),p1(0x63),p1(0x64),p1(0x65),p1(0x66),p1(0x67) ;\
	.long	p1(0x68),p1(0x69),p1(0x6a),p1(0x6b),p1(0x6c),p1(0x6d),p1(0x6e),p1(0x6f) ;\
	.long	p1(0x70),p1(0x71),p1(0x72),p1(0x73),p1(0x74),p1(0x75),p1(0x76),p1(0x77) ;\
	.long	p1(0x78),p1(0x79),p1(0x7a),p1(0x7b),p1(0x7c),p1(0x7d),p1(0x7e),p1(0x7f)
#define im_data4(p1) \
	.long	p1(0x80),p1(0x81),p1(0x82),p1(0x83),p1(0x84),p1(0x85),p1(0x86),p1(0x87) ;\
	.long	p1(0x88),p1(0x89),p1(0x8a),p1(0x8b),p1(0x8c),p1(0x8d),p1(0x8e),p1(0x8f) ;\
	.long	p1(0x90),p1(0x91),p1(0x92),p1(0x93),p1(0x94),p1(0x95),p1(0x96),p1(0x97) ;\
	.long	p1(0x98),p1(0x99),p1(0x9a),p1(0x9b),p1(0x9c),p1(0x9d),p1(0x9e),p1(0x9f)
#define im_data5(p1) \
	.long	p1(0xa0),p1(0xa1),p1(0xa2),p1(0xa3),p1(0xa4),p1(0xa5),p1(0xa6),p1(0xa7) ;\
	.long	p1(0xa8),p1(0xa9),p1(0xaa),p1(0xab),p1(0xac),p1(0xad),p1(0xae),p1(0xaf) ;\
	.long	p1(0xb0),p1(0xb1),p1(0xb2),p1(0xb3),p1(0xb4),p1(0xb5),p1(0xb6),p1(0xb7) ;\
	.long	p1(0xb8),p1(0xb9),p1(0xba),p1(0xbb),p1(0xbc),p1(0xbd),p1(0xbe),p1(0xbf)
#define im_data6(p1) \
	.long	p1(0xc0),p1(0xc1),p1(0xc2),p1(0xc3),p1(0xc4),p1(0xc5),p1(0xc6),p1(0xc7) ;\
	.long	p1(0xc8),p1(0xc9),p1(0xca),p1(0xcb),p1(0xcc),p1(0xcd),p1(0xce),p1(0xcf) ;\
	.long	p1(0xd0),p1(0xd1),p1(0xd2),p1(0xd3),p1(0xd4),p1(0xd5),p1(0xd6),p1(0xd7) ;\
	.long	p1(0xd8),p1(0xd9),p1(0xda),p1(0xdb),p1(0xdc),p1(0xdd),p1(0xde),p1(0xdf)
#define im_data7(p1) \
	.long	p1(0xe0),p1(0xe1),p1(0xe2),p1(0xe3),p1(0xe4),p1(0xe5),p1(0xe6),p1(0xe7) ;\
	.long	p1(0xe8),p1(0xe9),p1(0xea),p1(0xeb),p1(0xec),p1(0xed),p1(0xee),p1(0xef) ;\
	.long	p1(0xf0),p1(0xf1),p1(0xf2),p1(0xf3),p1(0xf4),p1(0xf5),p1(0xf6),p1(0xf7) ;\
	.long	p1(0xf8),p1(0xf9),p1(0xfa),p1(0xfb),p1(0xfc),p1(0xfd),p1(0xfe),p1(0xff)

// S-box data - 256 entries

#define sb_data0(p1) \
	.long	p1(0x63),p1(0x7c),p1(0x77),p1(0x7b),p1(0xf2),p1(0x6b),p1(0x6f),p1(0xc5) ;\
	.long	p1(0x30),p1(0x01),p1(0x67),p1(0x2b),p1(0xfe),p1(0xd7),p1(0xab),p1(0x76) ;\
	.long	p1(0xca),p1(0x82),p1(0xc9),p1(0x7d),p1(0xfa),p1(0x59),p1(0x47),p1(0xf0) ;\
	.long	p1(0xad),p1(0xd4),p1(0xa2),p1(0xaf),p1(0x9c),p1(0xa4),p1(0x72),p1(0xc0)
#define sb_data1(p1) \
	.long	p1(0xb7),p1(0xfd),p1(0x93),p1(0x26),p1(0x36),p1(0x3f),p1(0xf7),p1(0xcc) ;\
	.long	p1(0x34),p1(0xa5),p1(0xe5),p1(0xf1),p1(0x71),p1(0xd8),p1(0x31),p1(0x15) ;\
	.long	p1(0x04),p1(0xc7),p1(0x23),p1(0xc3),p1(0x18),p1(0x96),p1(0x05),p1(0x9a) ;\
	.long	p1(0x07),p1(0x12),p1(0x80),p1(0xe2),p1(0xeb),p1(0x27),p1(0xb2),p1(0x75)
#define sb_data2(p1) \
	.long	p1(0x09),p1(0x83),p1(0x2c),p1(0x1a),p1(0x1b),p1(0x6e),p1(0x5a),p1(0xa0) ;\
	.long	p1(0x52),p1(0x3b),p1(0xd6),p1(0xb3),p1(0x29),p1(0xe3),p1(0x2f),p1(0x84) ;\
	.long	p1(0x53),p1(0xd1),p1(0x00),p1(0xed),p1(0x20),p1(0xfc),p1(0xb1),p1(0x5b) ;\
	.long	p1(0x6a),p1(0xcb),p1(0xbe),p1(0x39),p1(0x4a),p1(0x4c),p1(0x58),p1(0xcf)
#define sb_data3(p1) \
	.long	p1(0xd0),p1(0xef),p1(0xaa),p1(0xfb),p1(0x43),p1(0x4d),p1(0x33),p1(0x85) ;\
	.long	p1(0x45),p1(0xf9),p1(0x02),p1(0x7f),p1(0x50),p1(0x3c),p1(0x9f),p1(0xa8) ;\
	.long	p1(0x51),p1(0xa3),p1(0x40),p1(0x8f),p1(0x92),p1(0x9d),p1(0x38),p1(0xf5) ;\
	.long	p1(0xbc),p1(0xb6),p1(0xda),p1(0x21),p1(0x10),p1(0xff),p1(0xf3),p1(0xd2)
#define sb_data4(p1) \
	.long	p1(0xcd),p1(0x0c),p1(0x13),p1(0xec),p1(0x5f),p1(0x97),p1(0x44),p1(0x17) ;\
	.long	p1(0xc4),p1(0xa7),p1(0x7e),p1(0x3d),p1(0x64),p1(0x5d),p1(0x19),p1(0x73) ;\
	.long	p1(0x60),p1(0x81),p1(0x4f),p1(0xdc),p1(0x22),p1(0x2a),p1(0x90),p1(0x88) ;\
	.long	p1(0x46),p1(0xee),p1(0xb8),p1(0x14),p1(0xde),p1(0x5e),p1(0x0b),p1(0xdb)
#define sb_data5(p1) \
	.long	p1(0xe0),p1(0x32),p1(0x3a),p1(0x0a),p1(0x49),p1(0x06),p1(0x24),p1(0x5c) ;\
	.long	p1(0xc2),p1(0xd3),p1(0xac),p1(0x62),p1(0x91),p1(0x95),p1(0xe4),p1(0x79) ;\
	.long	p1(0xe7),p1(0xc8),p1(0x37),p1(0x6d),p1(0x8d),p1(0xd5),p1(0x4e),p1(0xa9) ;\
	.long	p1(0x6c),p1(0x56),p1(0xf4),p1(0xea),p1(0x65),p1(0x7a),p1(0xae),p1(0x08)
#define sb_data6(p1) \
	.long	p1(0xba),p1(0x78),p1(0x25),p1(0x2e),p1(0x1c),p1(0xa6),p1(0xb4),p1(0xc6) ;\
	.long	p1(0xe8),p1(0xdd),p1(0x74),p1(0x1f),p1(0x4b),p1(0xbd),p1(0x8b),p1(0x8a) ;\
	.long	p1(0x70),p1(0x3e),p1(0xb5),p1(0x66),p1(0x48),p1(0x03),p1(0xf6),p1(0x0e) ;\
	.long	p1(0x61),p1(0x35),p1(0x57),p1(0xb9),p1(0x86),p1(0xc1),p1(0x1d),p1(0x9e)
#define sb_data7(p1) \
	.long	p1(0xe1),p1(0xf8),p1(0x98),p1(0x11),p1(0x69),p1(0xd9),p1(0x8e),p1(0x94) ;\
	.long	p1(0x9b),p1(0x1e),p1(0x87),p1(0xe9),p1(0xce),p1(0x55),p1(0x28),p1(0xdf) ;\
	.long	p1(0x8c),p1(0xa1),p1(0x89),p1(0x0d),p1(0xbf),p1(0xe6),p1(0x42),p1(0x68) ;\
	.long	p1(0x41),p1(0x99),p1(0x2d),p1(0x0f),p1(0xb0),p1(0x54),p1(0xbb),p1(0x16)

// Inverse S-box data - 256 entries

#define ib_data0(p1) \
	.long	p1(0x52),p1(0x09),p1(0x6a),p1(0xd5),p1(0x30),p1(0x36),p1(0xa5),p1(0x38) ;\
	.long	p1(0xbf),p1(0x40),p1(0xa3),p1(0x9e),p1(0x81),p1(0xf3),p1(0xd7),p1(0xfb) ;\
	.long	p1(0x7c),p1(0xe3),p1(0x39),p1(0x82),p1(0x9b),p1(0x2f),p1(0xff),p1(0x87) ;\
	.long	p1(0x34),p1(0x8e),p1(0x43),p1(0x44),p1(0xc4),p1(0xde),p1(0xe9),p1(0xcb)
#define ib_data1(p1) \
	.long	p1(0x54),p1(0x7b),p1(0x94),p1(0x32),p1(0xa6),p1(0xc2),p1(0x23),p1(0x3d) ;\
	.long	p1(0xee),p1(0x4c),p1(0x95),p1(0x0b),p1(0x42),p1(0xfa),p1(0xc3),p1(0x4e) ;\
	.long	p1(0x08),p1(0x2e),p1(0xa1),p1(0x66),p1(0x28),p1(0xd9),p1(0x24),p1(0xb2) ;\
	.long	p1(0x76),p1(0x5b),p1(0xa2),p1(0x49),p1(0x6d),p1(0x8b),p1(0xd1),p1(0x25)
#define ib_data2(p1) \
	.long	p1(0x72),p1(0xf8),p1(0xf6),p1(0x64),p1(0x86),p1(0x68),p1(0x98),p1(0x16) ;\
	.long	p1(0xd4),p1(0xa4),p1(0x5c),p1(0xcc),p1(0x5d),p1(0x65),p1(0xb6),p1(0x92) ;\
	.long	p1(0x6c),p1(0x70),p1(0x48),p1(0x50),p1(0xfd),p1(0xed),p1(0xb9),p1(0xda) ;\
	.long	p1(0x5e),p1(0x15),p1(0x46),p1(0x57),p1(0xa7),p1(0x8d),p1(0x9d),p1(0x84)
#define ib_data3(p1) \
	.long	p1(0x90),p1(0xd8),p1(0xab),p1(0x00),p1(0x8c),p1(0xbc),p1(0xd3),p1(0x0a) ;\
	.long	p1(0xf7),p1(0xe4),p1(0x58),p1(0x05),p1(0xb8),p1(0xb3),p1(0x45),p1(0x06) ;\
	.long	p1(0xd0),p1(0x2c),p1(0x1e),p1(0x8f),p1(0xca),p1(0x3f),p1(0x0f),p1(0x02) ;\
	.long	p1(0xc1),p1(0xaf),p1(0xbd),p1(0x03),p1(0x01),p1(0x13),p1(0x8a),p1(0x6b)
#define ib_data4(p1) \
	.long	p1(0x3a),p1(0x91),p1(0x11),p1(0x41),p1(0x4f),p1(0x67),p1(0xdc),p1(0xea) ;\
	.long	p1(0x97),p1(0xf2),p1(0xcf),p1(0xce),p1(0xf0),p1(0xb4),p1(0xe6),p1(0x73) ;\
	.long	p1(0x96),p1(0xac),p1(0x74),p1(0x22),p1(0xe7),p1(0xad),p1(0x35),p1(0x85) ;\
	.long	p1(0xe2),p1(0xf9),p1(0x37),p1(0xe8),p1(0x1c),p1(0x75),p1(0xdf),p1(0x6e)
#define ib_data5(p1) \
	.long	p1(0x47),p1(0xf1),p1(0x1a),p1(0x71),p1(0x1d),p1(0x29),p1(0xc5),p1(0x89) ;\
	.long	p1(0x6f),p1(0xb7),p1(0x62),p1(0x0e),p1(0xaa),p1(0x18),p1(0xbe),p1(0x1b) ;\
	.long	p1(0xfc),p1(0x56),p1(0x3e),p1(0x4b),p1(0xc6),p1(0xd2),p1(0x79),p1(0x20) ;\
	.long	p1(0x9a),p1(0xdb),p1(0xc0),p1(0xfe),p1(0x78),p1(0xcd),p1(0x5a),p1(0xf4)
#define ib_data6(p1) \
	.long	p1(0x1f),p1(0xdd),p1(0xa8),p1(0x33),p1(0x88),p1(0x07),p1(0xc7),p1(0x31) ;\
	.long	p1(0xb1),p1(0x12),p1(0x10),p1(0x59),p1(0x27),p1(0x80),p1(0xec),p1(0x5f) ;\
	.long	p1(0x60),p1(0x51),p1(0x7f),p1(0xa9),p1(0x19),p1(0xb5),p1(0x4a),p1(0x0d) ;\
	.long	p1(0x2d),p1(0xe5),p1(0x7a),p1(0x9f),p1(0x93),p1(0xc9),p1(0x9c),p1(0xef)
#define ib_data7(p1) \
	.long	p1(0xa0),p1(0xe0),p1(0x3b),p1(0x4d),p1(0xae),p1(0x2a),p1(0xf5),p1(0xb0) ;\
	.long	p1(0xc8),p1(0xeb),p1(0xbb),p1(0x3c),p1(0x83),p1(0x53),p1(0x99),p1(0x61) ;\
	.long	p1(0x17),p1(0x2b),p1(0x04),p1(0x7e),p1(0xba),p1(0x77),p1(0xd6),p1(0x26) ;\
	.long	p1(0xe1),p1(0x69),p1(0x14),p1(0x63),p1(0x55),p1(0x21),p1(0x0c),p1(0x7d)

// The rcon_table (needed for the key schedule)
//
// Here is original Dr Brian Gladman's source code:
//	_rcon_tab:
//	%assign x   1
//	%rep 29
//	    dd  x
//	%assign x f2(x)
//	%endrep
//
// Here is precomputed output (it's more portable this way):

	.section .rodata
	.align	ALIGN64BYTES
aes_rcon_tab:
	.long	0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80
	.long	0x1b,0x36,0x6c,0xd8,0xab,0x4d,0x9a,0x2f
	.long	0x5e,0xbc,0x63,0xc6,0x97,0x35,0x6a,0xd4
	.long	0xb3,0x7d,0xfa,0xef,0xc5

// The forward xor tables

	.align	ALIGN64BYTES
aes_ft_tab:
	sb_data0(u0)
	sb_data1(u0)
	sb_data2(u0)
	sb_data3(u0)
	sb_data4(u0)
	sb_data5(u0)
	sb_data6(u0)
	sb_data7(u0)

	sb_data0(u1)
	sb_data1(u1)
	sb_data2(u1)
	sb_data3(u1)
	sb_data4(u1)
	sb_data5(u1)
	sb_data6(u1)
	sb_data7(u1)

	sb_data0(u2)
	sb_data1(u2)
	sb_data2(u2)
	sb_data3(u2)
	sb_data4(u2)
	sb_data5(u2)
	sb_data6(u2)
	sb_data7(u2)

	sb_data0(u3)
	sb_data1(u3)
	sb_data2(u3)
	sb_data3(u3)
	sb_data4(u3)
	sb_data5(u3)
	sb_data6(u3)
	sb_data7(u3)

	.align	ALIGN64BYTES
aes_fl_tab:
	sb_data0(w0)
	sb_data1(w0)
	sb_data2(w0)
	sb_data3(w0)
	sb_data4(w0)
	sb_data5(w0)
	sb_data6(w0)
	sb_data7(w0)

	sb_data0(w1)
	sb_data1(w1)
	sb_data2(w1)
	sb_data3(w1)
	sb_data4(w1)
	sb_data5(w1)
	sb_data6(w1)
	sb_data7(w1)

	sb_data0(w2)
	sb_data1(w2)
	sb_data2(w2)
	sb_data3(w2)
	sb_data4(w2)
	sb_data5(w2)
	sb_data6(w2)
	sb_data7(w2)

	sb_data0(w3)
	sb_data1(w3)
	sb_data2(w3)
	sb_data3(w3)
	sb_data4(w3)
	sb_data5(w3)
	sb_data6(w3)
	sb_data7(w3)

// The inverse xor tables

	.align	ALIGN64BYTES
aes_it_tab:
	ib_data0(v0)
	ib_data1(v0)
	ib_data2(v0)
	ib_data3(v0)
	ib_data4(v0)
	ib_data5(v0)
	ib_data6(v0)
	ib_data7(v0)

	ib_data0(v1)
	ib_data1(v1)
	ib_data2(v1)
	ib_data3(v1)
	ib_data4(v1)
	ib_data5(v1)
	ib_data6(v1)
	ib_data7(v1)

	ib_data0(v2)
	ib_data1(v2)
	ib_data2(v2)
	ib_data3(v2)
	ib_data4(v2)
	ib_data5(v2)
	ib_data6(v2)
	ib_data7(v2)

	ib_data0(v3)
	ib_data1(v3)
	ib_data2(v3)
	ib_data3(v3)
	ib_data4(v3)
	ib_data5(v3)
	ib_data6(v3)
	ib_data7(v3)

	.align	ALIGN64BYTES
aes_il_tab:
	ib_data0(w0)
	ib_data1(w0)
	ib_data2(w0)
	ib_data3(w0)
	ib_data4(w0)
	ib_data5(w0)
	ib_data6(w0)
	ib_data7(w0)

	ib_data0(w1)
	ib_data1(w1)
	ib_data2(w1)
	ib_data3(w1)
	ib_data4(w1)
	ib_data5(w1)
	ib_data6(w1)
	ib_data7(w1)

	ib_data0(w2)
	ib_data1(w2)
	ib_data2(w2)
	ib_data3(w2)
	ib_data4(w2)
	ib_data5(w2)
	ib_data6(w2)
	ib_data7(w2)

	ib_data0(w3)
	ib_data1(w3)
	ib_data2(w3)
	ib_data3(w3)
	ib_data4(w3)
	ib_data5(w3)
	ib_data6(w3)
	ib_data7(w3)

// The inverse mix column tables

	.align	ALIGN64BYTES
aes_im_tab:
	im_data0(v0)
	im_data1(v0)
	im_data2(v0)
	im_data3(v0)
	im_data4(v0)
	im_data5(v0)
	im_data6(v0)
	im_data7(v0)

	im_data0(v1)
	im_data1(v1)
	im_data2(v1)
	im_data3(v1)
	im_data4(v1)
	im_data5(v1)
	im_data6(v1)
	im_data7(v1)

	im_data0(v2)
	im_data1(v2)
	im_data2(v2)
	im_data3(v2)
	im_data4(v2)
	im_data5(v2)
	im_data6(v2)
	im_data7(v2)

	im_data0(v3)
	im_data1(v3)
	im_data2(v3)
	im_data3(v3)
	im_data4(v3)
	im_data5(v3)
	im_data6(v3)
	im_data7(v3)

#if defined(__ELF__) && defined(SECTION_NOTE_GNU_STACK)
	.section .note.GNU-stack,"",@progbits
#endif
