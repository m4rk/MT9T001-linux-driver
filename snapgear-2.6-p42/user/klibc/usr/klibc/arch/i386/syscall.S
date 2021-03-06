/*
 * arch/i386/syscall.S
 *
 * Common tail-handling code for system calls.
 *
 * The arguments are on the stack; the system call number in %eax.
 */

#define ARG(n)	(4*n+20)(%esp)

	.text
	.align	4
	.globl	__syscall_common
	.type	__syscall_common,@function
__syscall_common:
#ifdef _REGPARM
	xchgl	%ebx,(%esp)
#else
	popl	%eax
	pushl	%ebx
#endif
	pushl	%esi
	pushl	%edi
	pushl	%ebp

#ifdef _REGPARM
	xchgl	%eax,%ebx
	xchgl	%ecx,%edx
	movl	ARG(0),%esi
	movl	ARG(1),%edi
	movl	ARG(2),%ebp
#else
	movl	ARG(0),%ebx		# Syscall arguments
	movl	ARG(1),%ecx
	movl	ARG(2),%edx
	movl	ARG(3),%esi
	movl	ARG(4),%edi
	movl	ARG(5),%ebp
#endif
	.globl __syscall_common_tail
__syscall_common_tail:
	int	$0x80

	cmpl	$-4095,%eax

	popl	%ebp
	popl	%edi
	popl	%esi
	popl	%ebx

	jb	1f

	# Error return, must set errno
	negl	%eax
	movl	%eax,errno
	orl	$-1,%eax		# Return -1

1:
	ret

	.size	__syscall_common,.-__syscall_common

#ifndef _REGPARM

	.globl	__syscall_varadic
	.type	__syscall_varadic,@function
__syscall_varadic = __syscall_common

#endif
