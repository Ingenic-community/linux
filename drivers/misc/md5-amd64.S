//
//  md5-amd64.S
//
//  Written by Jari Ruusu, October 1 2003
//
//  Copyright 2003 by Jari Ruusu.
//  Redistribution of this file is permitted under the GNU Public License.
//

//  Modified by Jari Ruusu,  June 12 2004
//   - Converted 32 bit x86 code to 64 bit AMD64 code

// A MD5 transform implementation for AMD64 compatible processors.
// This code does not preserve the rax, rcx, rdx, rsi, rdi or r8-r11
// registers or the artihmetic status flags. However, the rbx, rbp and
// r12-r15 registers are preserved across calls.

// void md5_transform_CPUbyteorder(u_int32_t *hash, u_int32_t *in)

#if defined(USE_UNDERLINE)
# define md5_transform_CPUbyteorder _md5_transform_CPUbyteorder
#endif
#if !defined(ALIGN64BYTES)
# define ALIGN64BYTES 64
#endif

	.file	"md5-amd64.S"
	.globl	md5_transform_CPUbyteorder

// rdi = pointer to hash[4] array which is read and written
// rsi = pointer to in[16] array which is read only

	.text
	.align	ALIGN64BYTES
md5_transform_CPUbyteorder:
	movl	12(%rdi),%eax
	movl	8(%rdi),%ecx
	movl	(%rdi),%r8d
	movl	4(%rdi),%r9d
	movl	(%rsi),%r10d
	prefetcht0 60(%rsi)
	movl	%eax,%edx
	xorl	%ecx,%eax

#define REPEAT1(p1w,p2x,p3z,p4c,p5s,p6Nin,p7Nz,p8Ny) \
	addl	$p4c,p1w		;\
	andl	p2x,%eax		;\
	addl	%r10d,p1w		;\
	xorl	p3z,%eax		;\
	movl	p6Nin*4(%rsi),%r10d	;\
	addl	%eax,p1w		;\
	movl	p7Nz,%eax		;\
	roll	$p5s,p1w		;\
	xorl	p8Ny,%eax		;\
	addl	p2x,p1w

	REPEAT1(%r8d,%r9d,%edx,0xd76aa478, 7, 1,%ecx,%r9d)
	REPEAT1(%edx,%r8d,%ecx,0xe8c7b756,12, 2,%r9d,%r8d)
	REPEAT1(%ecx,%edx,%r9d,0x242070db,17, 3,%r8d,%edx)
	REPEAT1(%r9d,%ecx,%r8d,0xc1bdceee,22, 4,%edx,%ecx)
	REPEAT1(%r8d,%r9d,%edx,0xf57c0faf, 7, 5,%ecx,%r9d)
	REPEAT1(%edx,%r8d,%ecx,0x4787c62a,12, 6,%r9d,%r8d)
	REPEAT1(%ecx,%edx,%r9d,0xa8304613,17, 7,%r8d,%edx)
	REPEAT1(%r9d,%ecx,%r8d,0xfd469501,22, 8,%edx,%ecx)
	REPEAT1(%r8d,%r9d,%edx,0x698098d8, 7, 9,%ecx,%r9d)
	REPEAT1(%edx,%r8d,%ecx,0x8b44f7af,12,10,%r9d,%r8d)
	REPEAT1(%ecx,%edx,%r9d,0xffff5bb1,17,11,%r8d,%edx)
	REPEAT1(%r9d,%ecx,%r8d,0x895cd7be,22,12,%edx,%ecx)
	REPEAT1(%r8d,%r9d,%edx,0x6b901122, 7,13,%ecx,%r9d)
	REPEAT1(%edx,%r8d,%ecx,0xfd987193,12,14,%r9d,%r8d)
	REPEAT1(%ecx,%edx,%r9d,0xa679438e,17,15,%r8d,%edx)

	addl	$0x49b40821,%r9d
	andl	%ecx,%eax
	addl	%r10d,%r9d
	xorl	%r8d,%eax
	movl	1*4(%rsi),%r10d
	addl	%eax,%r9d
	movl	%ecx,%eax
	roll	$22,%r9d
	addl	%ecx,%r9d

#define REPEAT2(p1w,p2x,p3y,p4z,p5c,p6s,p7Nin,p8Ny) \
	xorl	p2x,%eax		;\
	addl	$p5c,p1w		;\
	andl	p4z,%eax		;\
	addl	%r10d,p1w		;\
	xorl	p3y,%eax		;\
	movl	p7Nin*4(%rsi),%r10d	;\
	addl	%eax,p1w		;\
	movl	p8Ny,%eax		;\
	roll	$p6s,p1w		;\
	addl	p2x,p1w

	REPEAT2(%r8d,%r9d,%ecx,%edx,0xf61e2562, 5, 6,%r9d)
	REPEAT2(%edx,%r8d,%r9d,%ecx,0xc040b340, 9,11,%r8d)
	REPEAT2(%ecx,%edx,%r8d,%r9d,0x265e5a51,14, 0,%edx)
	REPEAT2(%r9d,%ecx,%edx,%r8d,0xe9b6c7aa,20, 5,%ecx)
	REPEAT2(%r8d,%r9d,%ecx,%edx,0xd62f105d, 5,10,%r9d)
	REPEAT2(%edx,%r8d,%r9d,%ecx,0x02441453, 9,15,%r8d)
	REPEAT2(%ecx,%edx,%r8d,%r9d,0xd8a1e681,14, 4,%edx)
	REPEAT2(%r9d,%ecx,%edx,%r8d,0xe7d3fbc8,20, 9,%ecx)
	REPEAT2(%r8d,%r9d,%ecx,%edx,0x21e1cde6, 5,14,%r9d)
	REPEAT2(%edx,%r8d,%r9d,%ecx,0xc33707d6, 9, 3,%r8d)
	REPEAT2(%ecx,%edx,%r8d,%r9d,0xf4d50d87,14, 8,%edx)
	REPEAT2(%r9d,%ecx,%edx,%r8d,0x455a14ed,20,13,%ecx)
	REPEAT2(%r8d,%r9d,%ecx,%edx,0xa9e3e905, 5, 2,%r9d)
	REPEAT2(%edx,%r8d,%r9d,%ecx,0xfcefa3f8, 9, 7,%r8d)
	REPEAT2(%ecx,%edx,%r8d,%r9d,0x676f02d9,14,12,%edx)

	xorl	%ecx,%eax
	addl	$0x8d2a4c8a,%r9d
	andl	%r8d,%eax
	addl	%r10d,%r9d
	xorl	%edx,%eax
	movl	5*4(%rsi),%r10d
	addl	%eax,%r9d
	movl	%ecx,%eax
	roll	$20,%r9d
	xorl	%edx,%eax
	addl	%ecx,%r9d

#define REPEAT3(p1w,p2x,p3c,p4s,p5Nin,p6Ny,p7Nz) \
	addl	$p3c,p1w		;\
	xorl	p2x,%eax		;\
	addl	%r10d,p1w		;\
	movl	p5Nin*4(%rsi),%r10d	;\
	addl	%eax,p1w		;\
	movl	p6Ny,%eax		;\
	roll	$p4s,p1w		;\
	xorl	p7Nz,%eax		;\
	addl	p2x,p1w

	REPEAT3(%r8d,%r9d,0xfffa3942, 4, 8,%r9d,%ecx)
	REPEAT3(%edx,%r8d,0x8771f681,11,11,%r8d,%r9d)
	REPEAT3(%ecx,%edx,0x6d9d6122,16,14,%edx,%r8d)
	REPEAT3(%r9d,%ecx,0xfde5380c,23, 1,%ecx,%edx)
	REPEAT3(%r8d,%r9d,0xa4beea44, 4, 4,%r9d,%ecx)
	REPEAT3(%edx,%r8d,0x4bdecfa9,11, 7,%r8d,%r9d)
	REPEAT3(%ecx,%edx,0xf6bb4b60,16,10,%edx,%r8d)
	REPEAT3(%r9d,%ecx,0xbebfbc70,23,13,%ecx,%edx)
	REPEAT3(%r8d,%r9d,0x289b7ec6, 4, 0,%r9d,%ecx)
	REPEAT3(%edx,%r8d,0xeaa127fa,11, 3,%r8d,%r9d)
	REPEAT3(%ecx,%edx,0xd4ef3085,16, 6,%edx,%r8d)
	REPEAT3(%r9d,%ecx,0x04881d05,23, 9,%ecx,%edx)
	REPEAT3(%r8d,%r9d,0xd9d4d039, 4,12,%r9d,%ecx)
	REPEAT3(%edx,%r8d,0xe6db99e5,11,15,%r8d,%r9d)
	REPEAT3(%ecx,%edx,0x1fa27cf8,16, 2,%edx,%r8d)

	addl	$0xc4ac5665,%r9d
	xorl	%ecx,%eax
	addl	%r10d,%r9d
	movl	(%rsi),%r10d
	addl	%eax,%r9d
	movl	%edx,%eax
	roll	$23,%r9d
	notl	%eax
	addl	%ecx,%r9d

#define REPEAT4(p1w,p2x,p3y,p4c,p5s,p6Nin,p7Nz) \
	addl	$p4c,p1w		;\
	orl	p2x,%eax		;\
	addl	%r10d,p1w		;\
	xorl	p3y,%eax		;\
	movl	p6Nin*4(%rsi),%r10d	;\
	addl	%eax,p1w		;\
	movl	p7Nz,%eax		;\
	roll	$p5s,p1w		;\
	notl	%eax			;\
	addl	p2x,p1w

	REPEAT4(%r8d,%r9d,%ecx,0xf4292244, 6, 7,%ecx)
	REPEAT4(%edx,%r8d,%r9d,0x432aff97,10,14,%r9d)
	REPEAT4(%ecx,%edx,%r8d,0xab9423a7,15, 5,%r8d)
	REPEAT4(%r9d,%ecx,%edx,0xfc93a039,21,12,%edx)
	REPEAT4(%r8d,%r9d,%ecx,0x655b59c3, 6, 3,%ecx)
	REPEAT4(%edx,%r8d,%r9d,0x8f0ccc92,10,10,%r9d)
	REPEAT4(%ecx,%edx,%r8d,0xffeff47d,15, 1,%r8d)
	REPEAT4(%r9d,%ecx,%edx,0x85845dd1,21, 8,%edx)
	REPEAT4(%r8d,%r9d,%ecx,0x6fa87e4f, 6,15,%ecx)
	REPEAT4(%edx,%r8d,%r9d,0xfe2ce6e0,10, 6,%r9d)
	REPEAT4(%ecx,%edx,%r8d,0xa3014314,15,13,%r8d)
	REPEAT4(%r9d,%ecx,%edx,0x4e0811a1,21, 4,%edx)
	REPEAT4(%r8d,%r9d,%ecx,0xf7537e82, 6,11,%ecx)
	REPEAT4(%edx,%r8d,%r9d,0xbd3af235,10, 2,%r9d)
	REPEAT4(%ecx,%edx,%r8d,0x2ad7d2bb,15, 9,%r8d)

	addl	$0xeb86d391,%r9d
	orl	%ecx,%eax
	addl	%r10d,%r9d
	xorl	%edx,%eax
	addl	%eax,%r9d
	roll	$21,%r9d
	addl	%ecx,%r9d

	addl	%r8d,(%rdi)
	addl	%r9d,4(%rdi)
	addl	%ecx,8(%rdi)
	addl	%edx,12(%rdi)
	ret

	.size md5_transform_CPUbyteorder,.-md5_transform_CPUbyteorder
	.type md5_transform_CPUbyteorder,@function

#if defined(__ELF__) && defined(SECTION_NOTE_GNU_STACK)
	.section .note.GNU-stack,"",@progbits
#endif
