//
//  md5-x86.S
//
//  Written by Jari Ruusu, October 1 2003
//
//  Copyright 2003 by Jari Ruusu.
//  Redistribution of this file is permitted under the GNU Public License.
//

// A MD5 transform implementation for x86 compatible processors. This
// version uses i386 instruction set but instruction scheduling is optimized
// for Pentium-2. This code does not preserve the eax, ecx or edx registers
// or the artihmetic status flags. However, the ebx, esi, edi, and ebp
// registers are preserved across calls.

// void md5_transform_CPUbyteorder(u_int32_t *hash, u_int32_t *in)

#if defined(USE_UNDERLINE)
# define md5_transform_CPUbyteorder _md5_transform_CPUbyteorder
#endif
#if !defined(ALIGN32BYTES)
# define ALIGN32BYTES 32
#endif

	.file	"md5-x86.S"
	.globl	md5_transform_CPUbyteorder
	.text
	.align	ALIGN32BYTES

md5_transform_CPUbyteorder:
	push	%ebp
	mov	4+4(%esp),%eax		// pointer to 'hash' input
	mov	8+4(%esp),%ebp		// pointer to 'in' array
	push	%ebx
	push	%esi
	push	%edi

	mov	(%eax),%esi
	mov	4(%eax),%edi
	mov	8(%eax),%ecx
	mov	12(%eax),%eax
	mov	(%ebp),%ebx
	mov	%eax,%edx
	xor	%ecx,%eax

#define REPEAT1(p1w,p2x,p3z,p4c,p5s,p6Nin,p7Nz,p8Ny) \
	add	$p4c,p1w		;\
	and	p2x,%eax		;\
	add	%ebx,p1w		;\
	xor	p3z,%eax		;\
	mov	p6Nin*4(%ebp),%ebx	;\
	add	%eax,p1w		;\
	mov	p7Nz,%eax		;\
	rol	$p5s,p1w		;\
	xor	p8Ny,%eax		;\
	add	p2x,p1w

	REPEAT1(%esi,%edi,%edx,0xd76aa478, 7, 1,%ecx,%edi)
	REPEAT1(%edx,%esi,%ecx,0xe8c7b756,12, 2,%edi,%esi)
	REPEAT1(%ecx,%edx,%edi,0x242070db,17, 3,%esi,%edx)
	REPEAT1(%edi,%ecx,%esi,0xc1bdceee,22, 4,%edx,%ecx)
	REPEAT1(%esi,%edi,%edx,0xf57c0faf, 7, 5,%ecx,%edi)
	REPEAT1(%edx,%esi,%ecx,0x4787c62a,12, 6,%edi,%esi)
	REPEAT1(%ecx,%edx,%edi,0xa8304613,17, 7,%esi,%edx)
	REPEAT1(%edi,%ecx,%esi,0xfd469501,22, 8,%edx,%ecx)
	REPEAT1(%esi,%edi,%edx,0x698098d8, 7, 9,%ecx,%edi)
	REPEAT1(%edx,%esi,%ecx,0x8b44f7af,12,10,%edi,%esi)
	REPEAT1(%ecx,%edx,%edi,0xffff5bb1,17,11,%esi,%edx)
	REPEAT1(%edi,%ecx,%esi,0x895cd7be,22,12,%edx,%ecx)
	REPEAT1(%esi,%edi,%edx,0x6b901122, 7,13,%ecx,%edi)
	REPEAT1(%edx,%esi,%ecx,0xfd987193,12,14,%edi,%esi)
	REPEAT1(%ecx,%edx,%edi,0xa679438e,17,15,%esi,%edx)

	add	$0x49b40821,%edi
	and	%ecx,%eax
	add	%ebx,%edi
	xor	%esi,%eax
	mov	1*4(%ebp),%ebx
	add	%eax,%edi
	mov	%ecx,%eax
	rol	$22,%edi
	add	%ecx,%edi

#define REPEAT2(p1w,p2x,p3y,p4z,p5c,p6s,p7Nin,p8Ny) \
	xor	p2x,%eax		;\
	add	$p5c,p1w		;\
	and	p4z,%eax		;\
	add	%ebx,p1w		;\
	xor	p3y,%eax		;\
	mov	p7Nin*4(%ebp),%ebx	;\
	add	%eax,p1w		;\
	mov	p8Ny,%eax		;\
	rol	$p6s,p1w		;\
	add	p2x,p1w

	REPEAT2(%esi,%edi,%ecx,%edx,0xf61e2562, 5, 6,%edi)
	REPEAT2(%edx,%esi,%edi,%ecx,0xc040b340, 9,11,%esi)
	REPEAT2(%ecx,%edx,%esi,%edi,0x265e5a51,14, 0,%edx)
	REPEAT2(%edi,%ecx,%edx,%esi,0xe9b6c7aa,20, 5,%ecx)
	REPEAT2(%esi,%edi,%ecx,%edx,0xd62f105d, 5,10,%edi)
	REPEAT2(%edx,%esi,%edi,%ecx,0x02441453, 9,15,%esi)
	REPEAT2(%ecx,%edx,%esi,%edi,0xd8a1e681,14, 4,%edx)
	REPEAT2(%edi,%ecx,%edx,%esi,0xe7d3fbc8,20, 9,%ecx)
	REPEAT2(%esi,%edi,%ecx,%edx,0x21e1cde6, 5,14,%edi)
	REPEAT2(%edx,%esi,%edi,%ecx,0xc33707d6, 9, 3,%esi)
	REPEAT2(%ecx,%edx,%esi,%edi,0xf4d50d87,14, 8,%edx)
	REPEAT2(%edi,%ecx,%edx,%esi,0x455a14ed,20,13,%ecx)
	REPEAT2(%esi,%edi,%ecx,%edx,0xa9e3e905, 5, 2,%edi)
	REPEAT2(%edx,%esi,%edi,%ecx,0xfcefa3f8, 9, 7,%esi)
	REPEAT2(%ecx,%edx,%esi,%edi,0x676f02d9,14,12,%edx)

	xor	%ecx,%eax
	add	$0x8d2a4c8a,%edi
	and	%esi,%eax
	add	%ebx,%edi
	xor	%edx,%eax
	mov	5*4(%ebp),%ebx
	add	%eax,%edi
	mov	%ecx,%eax
	rol	$20,%edi
	xor	%edx,%eax
	add	%ecx,%edi

#define REPEAT3(p1w,p2x,p3c,p4s,p5Nin,p6Ny,p7Nz) \
	add	$p3c,p1w		;\
	xor	p2x,%eax		;\
	add	%ebx,p1w		;\
	mov	p5Nin*4(%ebp),%ebx	;\
	add	%eax,p1w		;\
	mov	p6Ny,%eax		;\
	rol	$p4s,p1w		;\
	xor	p7Nz,%eax		;\
	add	p2x,p1w

	REPEAT3(%esi,%edi,0xfffa3942, 4, 8,%edi,%ecx)
	REPEAT3(%edx,%esi,0x8771f681,11,11,%esi,%edi)
	REPEAT3(%ecx,%edx,0x6d9d6122,16,14,%edx,%esi)
	REPEAT3(%edi,%ecx,0xfde5380c,23, 1,%ecx,%edx)
	REPEAT3(%esi,%edi,0xa4beea44, 4, 4,%edi,%ecx)
	REPEAT3(%edx,%esi,0x4bdecfa9,11, 7,%esi,%edi)
	REPEAT3(%ecx,%edx,0xf6bb4b60,16,10,%edx,%esi)
	REPEAT3(%edi,%ecx,0xbebfbc70,23,13,%ecx,%edx)
	REPEAT3(%esi,%edi,0x289b7ec6, 4, 0,%edi,%ecx)
	REPEAT3(%edx,%esi,0xeaa127fa,11, 3,%esi,%edi)
	REPEAT3(%ecx,%edx,0xd4ef3085,16, 6,%edx,%esi)
	REPEAT3(%edi,%ecx,0x04881d05,23, 9,%ecx,%edx)
	REPEAT3(%esi,%edi,0xd9d4d039, 4,12,%edi,%ecx)
	REPEAT3(%edx,%esi,0xe6db99e5,11,15,%esi,%edi)
	REPEAT3(%ecx,%edx,0x1fa27cf8,16, 2,%edx,%esi)

	add	$0xc4ac5665,%edi
	xor	%ecx,%eax
	add	%ebx,%edi
	mov	(%ebp),%ebx
	add	%eax,%edi
	mov	%edx,%eax
	rol	$23,%edi
	not	%eax
	add	%ecx,%edi

#define REPEAT4(p1w,p2x,p3y,p4c,p5s,p6Nin,p7Nz) \
	add	$p4c,p1w		;\
	or	p2x,%eax		;\
	add	%ebx,p1w		;\
	xor	p3y,%eax		;\
	mov	p6Nin*4(%ebp),%ebx	;\
	add	%eax,p1w		;\
	mov	p7Nz,%eax		;\
	rol	$p5s,p1w		;\
	not	%eax			;\
	add	p2x,p1w

	REPEAT4(%esi,%edi,%ecx,0xf4292244, 6, 7,%ecx)
	REPEAT4(%edx,%esi,%edi,0x432aff97,10,14,%edi)
	REPEAT4(%ecx,%edx,%esi,0xab9423a7,15, 5,%esi)
	REPEAT4(%edi,%ecx,%edx,0xfc93a039,21,12,%edx)
	REPEAT4(%esi,%edi,%ecx,0x655b59c3, 6, 3,%ecx)
	REPEAT4(%edx,%esi,%edi,0x8f0ccc92,10,10,%edi)
	REPEAT4(%ecx,%edx,%esi,0xffeff47d,15, 1,%esi)
	REPEAT4(%edi,%ecx,%edx,0x85845dd1,21, 8,%edx)
	REPEAT4(%esi,%edi,%ecx,0x6fa87e4f, 6,15,%ecx)
	REPEAT4(%edx,%esi,%edi,0xfe2ce6e0,10, 6,%edi)
	REPEAT4(%ecx,%edx,%esi,0xa3014314,15,13,%esi)
	REPEAT4(%edi,%ecx,%edx,0x4e0811a1,21, 4,%edx)
	REPEAT4(%esi,%edi,%ecx,0xf7537e82, 6,11,%ecx)
	REPEAT4(%edx,%esi,%edi,0xbd3af235,10, 2,%edi)
	REPEAT4(%ecx,%edx,%esi,0x2ad7d2bb,15, 9,%esi)

	add	$0xeb86d391,%edi
	or	%ecx,%eax
	add	%ebx,%edi
	xor	%edx,%eax
	mov	4+16(%esp),%ebp		// pointer to 'hash' output
	add	%eax,%edi
	rol	$21,%edi
	add	%ecx,%edi

	add	%esi,(%ebp)
	add	%edi,4(%ebp)
	add	%ecx,8(%ebp)
	add	%edx,12(%ebp)

	pop	%edi
	pop	%esi
	pop	%ebx
	pop	%ebp
	ret

	.size md5_transform_CPUbyteorder,.-md5_transform_CPUbyteorder
	.type md5_transform_CPUbyteorder,@function

#if defined(__ELF__) && defined(SECTION_NOTE_GNU_STACK)
	.section .note.GNU-stack,"",@progbits
#endif
