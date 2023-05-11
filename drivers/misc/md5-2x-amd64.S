//
//  md5-2x-amd64.S
//
//  Written by Jari Ruusu, October 1 2003
//
//  Copyright 2003 by Jari Ruusu.
//  Redistribution of this file is permitted under the GNU Public License.
//

//  Modified by Jari Ruusu,  June 12 2004
//   - Converted 32 bit x86 code to 64 bit AMD64 code

//  Modified by Jari Ruusu,  April 11 2010
//   - Added another parallel MD5 transform computation

// A MD5 transform implementation for AMD64 compatible processors.
// This code does not preserve the rax, rcx, rdx, rsi, rdi or r8-r11
// registers or the artihmetic status flags. However, the rbx, rbp and
// r12-r15 registers are preserved across calls.

// void md5_transform_CPUbyteorder_2x(u_int32_t *hashAB, u_int32_t *inA, u_int32_t *inB)

#if defined(USE_UNDERLINE)
# define md5_transform_CPUbyteorder_2x _md5_transform_CPUbyteorder_2x
#endif
#if !defined(ALIGN64BYTES)
# define ALIGN64BYTES 64
#endif

	.file	"md5-2x-amd64.S"
	.globl	md5_transform_CPUbyteorder_2x

// rdi = pointer to u_int32_t hash[4 + 4] array which is read and written
//       hash[0...3] are for first MD5, hash[4...7] are for second MD5
// rsi = pointer to u_int32_t in[16] array, first MD5, read only
// rdx = pointer to u_int32_t in[16] array, second MD5, read only

	.text
	.align	ALIGN64BYTES
md5_transform_CPUbyteorder_2x:
	push	%rbx
	push	%rbp
	push	%r12
	push	%r13
	push	%r14
	push	%r15

	movl	12(%rdi),%eax		;	movl	12+16(%rdi),%ebx
	movl	8(%rdi),%ecx		;	movl	8+16(%rdi),%r13d
	movl	(%rdi),%r8d		;	movl	16(%rdi),%r11d
	movl	4(%rdi),%r9d		;	movl	4+16(%rdi),%r12d
	movl	(%rsi),%r10d		;	movl	(%rdx),%ebp
	prefetcht0 60(%rsi)		;	prefetcht0 60(%rdx)
	movl	%eax,%r15d		;	movl	%ebx,%r14d
	xorl	%ecx,%eax		;	xorl	%r13d,%ebx

#define REPEAT1(p1Aw,p1Bw,p2Ax,p2Bx,p3Az,p3Bz,p4c,p5s,p6Nin,p7ANz,p7BNz,p8ANy,p8BNy) \
	addl	$p4c,p1Aw		;	addl	$p4c,p1Bw		;\
	andl	p2Ax,%eax		;	andl	p2Bx,%ebx		;\
	addl	%r10d,p1Aw		;	addl	%ebp,p1Bw		;\
	xorl	p3Az,%eax		;	xorl	p3Bz,%ebx		;\
	movl	p6Nin*4(%rsi),%r10d	;	movl	p6Nin*4(%rdx),%ebp	;\
	addl	%eax,p1Aw		;	addl	%ebx,p1Bw		;\
	movl	p7ANz,%eax		;	movl	p7BNz,%ebx		;\
	roll	$p5s,p1Aw		;	roll	$p5s,p1Bw		;\
	xorl	p8ANy,%eax		;	xorl	p8BNy,%ebx		;\
	addl	p2Ax,p1Aw		;	addl	p2Bx,p1Bw

	REPEAT1(%r8d,%r11d,%r9d,%r12d,%r15d,%r14d,0xd76aa478,7,1,%ecx,%r13d,%r9d,%r12d)
	REPEAT1(%r15d,%r14d,%r8d,%r11d,%ecx,%r13d,0xe8c7b756,12,2,%r9d,%r12d,%r8d,%r11d)
	REPEAT1(%ecx,%r13d,%r15d,%r14d,%r9d,%r12d,0x242070db,17,3,%r8d,%r11d,%r15d,%r14d)
	REPEAT1(%r9d,%r12d,%ecx,%r13d,%r8d,%r11d,0xc1bdceee,22,4,%r15d,%r14d,%ecx,%r13d)
	REPEAT1(%r8d,%r11d,%r9d,%r12d,%r15d,%r14d,0xf57c0faf,7,5,%ecx,%r13d,%r9d,%r12d)
	REPEAT1(%r15d,%r14d,%r8d,%r11d,%ecx,%r13d,0x4787c62a,12,6,%r9d,%r12d,%r8d,%r11d)
	REPEAT1(%ecx,%r13d,%r15d,%r14d,%r9d,%r12d,0xa8304613,17,7,%r8d,%r11d,%r15d,%r14d)
	REPEAT1(%r9d,%r12d,%ecx,%r13d,%r8d,%r11d,0xfd469501,22,8,%r15d,%r14d,%ecx,%r13d)
	REPEAT1(%r8d,%r11d,%r9d,%r12d,%r15d,%r14d,0x698098d8,7,9,%ecx,%r13d,%r9d,%r12d)
	REPEAT1(%r15d,%r14d,%r8d,%r11d,%ecx,%r13d,0x8b44f7af,12,10,%r9d,%r12d,%r8d,%r11d)
	REPEAT1(%ecx,%r13d,%r15d,%r14d,%r9d,%r12d,0xffff5bb1,17,11,%r8d,%r11d,%r15d,%r14d)
	REPEAT1(%r9d,%r12d,%ecx,%r13d,%r8d,%r11d,0x895cd7be,22,12,%r15d,%r14d,%ecx,%r13d)
	REPEAT1(%r8d,%r11d,%r9d,%r12d,%r15d,%r14d,0x6b901122,7,13,%ecx,%r13d,%r9d,%r12d)
	REPEAT1(%r15d,%r14d,%r8d,%r11d,%ecx,%r13d,0xfd987193,12,14,%r9d,%r12d,%r8d,%r11d)
	REPEAT1(%ecx,%r13d,%r15d,%r14d,%r9d,%r12d,0xa679438e,17,15,%r8d,%r11d,%r15d,%r14d)

	addl	$0x49b40821,%r9d	;	addl	$0x49b40821,%r12d
	andl	%ecx,%eax		;	andl	%r13d,%ebx
	addl	%r10d,%r9d		;	addl	%ebp,%r12d
	xorl	%r8d,%eax		;	xorl	%r11d,%ebx
	movl	1*4(%rsi),%r10d		;	movl	1*4(%rdx),%ebp
	addl	%eax,%r9d		;	addl	%ebx,%r12d
	movl	%ecx,%eax		;	movl	%r13d,%ebx
	roll	$22,%r9d		;	roll	$22,%r12d
	addl	%ecx,%r9d		;	addl	%r13d,%r12d

#define REPEAT2(p1Aw,p1Bw,p2Ax,p2Bx,p3Ay,p3By,p4Az,p4Bz,p5c,p6s,p7Nin,p8ANy,p8BNy) \
	xorl	p2Ax,%eax		;	xorl	p2Bx,%ebx		;\
	addl	$p5c,p1Aw		;	addl	$p5c,p1Bw		;\
	andl	p4Az,%eax		;	andl	p4Bz,%ebx		;\
	addl	%r10d,p1Aw		;	addl	%ebp,p1Bw		;\
	xorl	p3Ay,%eax		;	xorl	p3By,%ebx		;\
	movl	p7Nin*4(%rsi),%r10d	;	movl	p7Nin*4(%rdx),%ebp	;\
	addl	%eax,p1Aw		;	addl	%ebx,p1Bw		;\
	movl	p8ANy,%eax		;	movl	p8BNy,%ebx		;\
	roll	$p6s,p1Aw		;	roll	$p6s,p1Bw		;\
	addl	p2Ax,p1Aw		;	addl	p2Bx,p1Bw

	REPEAT2(%r8d,%r11d,%r9d,%r12d,%ecx,%r13d,%r15d,%r14d,0xf61e2562,5,6,%r9d,%r12d)
	REPEAT2(%r15d,%r14d,%r8d,%r11d,%r9d,%r12d,%ecx,%r13d,0xc040b340,9,11,%r8d,%r11d)
	REPEAT2(%ecx,%r13d,%r15d,%r14d,%r8d,%r11d,%r9d,%r12d,0x265e5a51,14,0,%r15d,%r14d)
	REPEAT2(%r9d,%r12d,%ecx,%r13d,%r15d,%r14d,%r8d,%r11d,0xe9b6c7aa,20,5,%ecx,%r13d)
	REPEAT2(%r8d,%r11d,%r9d,%r12d,%ecx,%r13d,%r15d,%r14d,0xd62f105d,5,10,%r9d,%r12d)
	REPEAT2(%r15d,%r14d,%r8d,%r11d,%r9d,%r12d,%ecx,%r13d,0x02441453,9,15,%r8d,%r11d)
	REPEAT2(%ecx,%r13d,%r15d,%r14d,%r8d,%r11d,%r9d,%r12d,0xd8a1e681,14,4,%r15d,%r14d)
	REPEAT2(%r9d,%r12d,%ecx,%r13d,%r15d,%r14d,%r8d,%r11d,0xe7d3fbc8,20,9,%ecx,%r13d)
	REPEAT2(%r8d,%r11d,%r9d,%r12d,%ecx,%r13d,%r15d,%r14d,0x21e1cde6,5,14,%r9d,%r12d)
	REPEAT2(%r15d,%r14d,%r8d,%r11d,%r9d,%r12d,%ecx,%r13d,0xc33707d6,9,3,%r8d,%r11d)
	REPEAT2(%ecx,%r13d,%r15d,%r14d,%r8d,%r11d,%r9d,%r12d,0xf4d50d87,14,8,%r15d,%r14d)
	REPEAT2(%r9d,%r12d,%ecx,%r13d,%r15d,%r14d,%r8d,%r11d,0x455a14ed,20,13,%ecx,%r13d)
	REPEAT2(%r8d,%r11d,%r9d,%r12d,%ecx,%r13d,%r15d,%r14d,0xa9e3e905,5,2,%r9d,%r12d)
	REPEAT2(%r15d,%r14d,%r8d,%r11d,%r9d,%r12d,%ecx,%r13d,0xfcefa3f8,9,7,%r8d,%r11d)
	REPEAT2(%ecx,%r13d,%r15d,%r14d,%r8d,%r11d,%r9d,%r12d,0x676f02d9,14,12,%r15d,%r14d)

	xorl	%ecx,%eax		;	xorl	%r13d,%ebx
	addl	$0x8d2a4c8a,%r9d	;	addl	$0x8d2a4c8a,%r12d
	andl	%r8d,%eax		;	andl	%r11d,%ebx
	addl	%r10d,%r9d		;	addl	%ebp,%r12d
	xorl	%r15d,%eax		;	xorl	%r14d,%ebx
	movl	5*4(%rsi),%r10d		;	movl	5*4(%rdx),%ebp
	addl	%eax,%r9d		;	addl	%ebx,%r12d
	movl	%ecx,%eax		;	movl	%r13d,%ebx
	roll	$20,%r9d		;	roll	$20,%r12d
	xorl	%r15d,%eax		;	xorl	%r14d,%ebx
	addl	%ecx,%r9d		;	addl	%r13d,%r12d

#define REPEAT3(p1Aw,p1Bw,p2Ax,p2Bx,p3c,p4s,p5Nin,p6ANy,p6BNy,p7ANz,p7BNz) \
	addl	$p3c,p1Aw		;	addl	$p3c,p1Bw		;\
	xorl	p2Ax,%eax		;	xorl	p2Bx,%ebx		;\
	addl	%r10d,p1Aw		;	addl	%ebp,p1Bw		;\
	movl	p5Nin*4(%rsi),%r10d	;	movl	p5Nin*4(%rdx),%ebp	;\
	addl	%eax,p1Aw		;	addl	%ebx,p1Bw		;\
	movl	p6ANy,%eax		;	movl	p6BNy,%ebx		;\
	roll	$p4s,p1Aw		;	roll	$p4s,p1Bw		;\
	xorl	p7ANz,%eax		;	xorl	p7BNz,%ebx		;\
	addl	p2Ax,p1Aw		;	addl	p2Bx,p1Bw

	REPEAT3(%r8d,%r11d,%r9d,%r12d,0xfffa3942,4,8,%r9d,%r12d,%ecx,%r13d)
	REPEAT3(%r15d,%r14d,%r8d,%r11d,0x8771f681,11,11,%r8d,%r11d,%r9d,%r12d)
	REPEAT3(%ecx,%r13d,%r15d,%r14d,0x6d9d6122,16,14,%r15d,%r14d,%r8d,%r11d)
	REPEAT3(%r9d,%r12d,%ecx,%r13d,0xfde5380c,23,1,%ecx,%r13d,%r15d,%r14d)
	REPEAT3(%r8d,%r11d,%r9d,%r12d,0xa4beea44,4,4,%r9d,%r12d,%ecx,%r13d)
	REPEAT3(%r15d,%r14d,%r8d,%r11d,0x4bdecfa9,11,7,%r8d,%r11d,%r9d,%r12d)
	REPEAT3(%ecx,%r13d,%r15d,%r14d,0xf6bb4b60,16,10,%r15d,%r14d,%r8d,%r11d)
	REPEAT3(%r9d,%r12d,%ecx,%r13d,0xbebfbc70,23,13,%ecx,%r13d,%r15d,%r14d)
	REPEAT3(%r8d,%r11d,%r9d,%r12d,0x289b7ec6,4,0,%r9d,%r12d,%ecx,%r13d)
	REPEAT3(%r15d,%r14d,%r8d,%r11d,0xeaa127fa,11,3,%r8d,%r11d,%r9d,%r12d)
	REPEAT3(%ecx,%r13d,%r15d,%r14d,0xd4ef3085,16,6,%r15d,%r14d,%r8d,%r11d)
	REPEAT3(%r9d,%r12d,%ecx,%r13d,0x04881d05,23,9,%ecx,%r13d,%r15d,%r14d)
	REPEAT3(%r8d,%r11d,%r9d,%r12d,0xd9d4d039,4,12,%r9d,%r12d,%ecx,%r13d)
	REPEAT3(%r15d,%r14d,%r8d,%r11d,0xe6db99e5,11,15,%r8d,%r11d,%r9d,%r12d)
	REPEAT3(%ecx,%r13d,%r15d,%r14d,0x1fa27cf8,16,2,%r15d,%r14d,%r8d,%r11d)

	addl	$0xc4ac5665,%r9d	;	addl	$0xc4ac5665,%r12d
	xorl	%ecx,%eax		;	xorl	%r13d,%ebx
	addl	%r10d,%r9d		;	addl	%ebp,%r12d
	movl	(%rsi),%r10d		;	movl	(%rdx),%ebp
	addl	%eax,%r9d		;	addl	%ebx,%r12d
	movl	%r15d,%eax		;	movl	%r14d,%ebx
	roll	$23,%r9d		;	roll	$23,%r12d
	notl	%eax			;	notl	%ebx
	addl	%ecx,%r9d		;	addl	%r13d,%r12d

#define REPEAT4(p1Aw,p1Bw,p2Ax,p2Bx,p3Ay,p3By,p4c,p5s,p6Nin,p7ANz,p7BNz) \
	addl	$p4c,p1Aw		;	addl	$p4c,p1Bw		;\
	orl	p2Ax,%eax		;	orl	p2Bx,%ebx		;\
	addl	%r10d,p1Aw		;	addl	%ebp,p1Bw		;\
	xorl	p3Ay,%eax		;	xorl	p3By,%ebx		;\
	movl	p6Nin*4(%rsi),%r10d	;	movl	p6Nin*4(%rdx),%ebp	;\
	addl	%eax,p1Aw		;	addl	%ebx,p1Bw		;\
	movl	p7ANz,%eax		;	movl	p7BNz,%ebx		;\
	roll	$p5s,p1Aw		;	roll	$p5s,p1Bw		;\
	notl	%eax			;	notl	%ebx			;\
	addl	p2Ax,p1Aw		;	addl	p2Bx,p1Bw

	REPEAT4(%r8d,%r11d,%r9d,%r12d,%ecx,%r13d,0xf4292244,6,7,%ecx,%r13d)
	REPEAT4(%r15d,%r14d,%r8d,%r11d,%r9d,%r12d,0x432aff97,10,14,%r9d,%r12d)
	REPEAT4(%ecx,%r13d,%r15d,%r14d,%r8d,%r11d,0xab9423a7,15,5,%r8d,%r11d)
	REPEAT4(%r9d,%r12d,%ecx,%r13d,%r15d,%r14d,0xfc93a039,21,12,%r15d,%r14d)
	REPEAT4(%r8d,%r11d,%r9d,%r12d,%ecx,%r13d,0x655b59c3,6,3,%ecx,%r13d)
	REPEAT4(%r15d,%r14d,%r8d,%r11d,%r9d,%r12d,0x8f0ccc92,10,10,%r9d,%r12d)
	REPEAT4(%ecx,%r13d,%r15d,%r14d,%r8d,%r11d,0xffeff47d,15,1,%r8d,%r11d)
	REPEAT4(%r9d,%r12d,%ecx,%r13d,%r15d,%r14d,0x85845dd1,21,8,%r15d,%r14d)
	REPEAT4(%r8d,%r11d,%r9d,%r12d,%ecx,%r13d,0x6fa87e4f,6,15,%ecx,%r13d)
	REPEAT4(%r15d,%r14d,%r8d,%r11d,%r9d,%r12d,0xfe2ce6e0,10,6,%r9d,%r12d)
	REPEAT4(%ecx,%r13d,%r15d,%r14d,%r8d,%r11d,0xa3014314,15,13,%r8d,%r11d)
	REPEAT4(%r9d,%r12d,%ecx,%r13d,%r15d,%r14d,0x4e0811a1,21,4,%r15d,%r14d)
	REPEAT4(%r8d,%r11d,%r9d,%r12d,%ecx,%r13d,0xf7537e82,6,11,%ecx,%r13d)
	REPEAT4(%r15d,%r14d,%r8d,%r11d,%r9d,%r12d,0xbd3af235,10,2,%r9d,%r12d)
	REPEAT4(%ecx,%r13d,%r15d,%r14d,%r8d,%r11d,0x2ad7d2bb,15,9,%r8d,%r11d)

	addl	$0xeb86d391,%r9d	;	addl	$0xeb86d391,%r12d
	orl	%ecx,%eax		;	orl	%r13d,%ebx
	addl	%r10d,%r9d		;	addl	%ebp,%r12d
	xorl	%r15d,%eax		;	xorl	%r14d,%ebx
	addl	%eax,%r9d		;	addl	%ebx,%r12d
	roll	$21,%r9d		;	roll	$21,%r12d
	addl	%ecx,%r9d		;	addl	%r13d,%r12d

	addl	%r8d,(%rdi)		;	addl	%r11d,16(%rdi)
	addl	%r9d,4(%rdi)		;	addl	%r12d,4+16(%rdi)
	addl	%ecx,8(%rdi)		;	addl	%r13d,8+16(%rdi)
	addl	%r15d,12(%rdi)		;	addl	%r14d,12+16(%rdi)

	pop	%r15
	pop	%r14
	pop	%r13
	pop	%r12
	pop	%rbp
	pop	%rbx
	ret

	.size md5_transform_CPUbyteorder_2x,.-md5_transform_CPUbyteorder_2x
	.type md5_transform_CPUbyteorder_2x,@function

#if defined(__ELF__) && defined(SECTION_NOTE_GNU_STACK)
	.section .note.GNU-stack,"",@progbits
#endif
