/****************************************************************************
 *
 * Copyright 2017 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/

 /*	$NetBSD: _setjmp.S,v 1.12 2013/04/19 13:45:45 matt Exp $	*/

/*
 * Copyright (c) 1997 Mark Brinicombe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Mark Brinicombe
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <tinyara/config.h>
#include <arch/setjmp.h>

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.syntax	unified
	.file	"setjmp.S"

	.globl	setjmp
	.type	setjmp, function

	.globl	longjmp
	.type	longjmp, function

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text

#if defined(__thumb__)
	.thumb
#endif

	.align 4

/****************************************************************************
 * Name: up_saveusercontext
 ****************************************************************************/


setjmp:
	.fnstart
	ldr	r1, .L_setjmp_magic

#if defined(CONFIG_ARCH_FPU)
	add	r2, r0, #(_JB_REG_D8 * 4)
	vstmia	r2, {d8-d15}
	vmrs	r2, fpscr
	str	r2, [r0, #(_JB_REG_FPSCR * 4)]
#endif /* !_STANDALONE && __ARM_ARCH >= 6 */

	str	r1, [r0]

	add	r0, r0, #(_JB_REG_R4 * 4)

	/* Store integer registers */
#ifndef __thumb__
	stmia	r0, {r4-r14}
#else
	stmia	r0, {r4-r12}
	str	r13, [r0, #((_JB_REG_R13 - _JB_REG_R4) * 4)]
	str	r14, [r0, #((_JB_REG_R14 - _JB_REG_R4) * 4)]
#endif

	mov	r0, #0x00000000
	bx	lr

	.fnend
	.size	setjmp, . - setjmp

.L_setjmp_magic:
#if defined(CONFIG_ARCH_FPU)
	.word _JB_MAGIC_SETJMP_VFP
#else
	.word	_JB_MAGIC_SETJMP
#endif

longjmp:
	.fnstart
	ldr	r2, [r0]			/* get magic from jmp_buf */
	ldr	ip, .L_setjmp_magic		/* load magic */
	teq	ip, r2				/* magic correct? */
	bne	botch				/*   no, botch */

#if defined(CONFIG_ARCH_FPU)
	add	ip, r0, #(_JB_REG_D8 * 4)
	vldmia	ip, {d8-d15}
	ldr	ip, [r0, #(_JB_REG_FPSCR * 4)]
	vmsr	fpscr, ip
#endif /* !_STANDALONE && __ARM_ARCH >= 6 */

	add	r0, r0, #(_JB_REG_R4 * 4)
   /* Restore integer registers */
#ifndef __thumb__
	ldmia	r0, {r4-r14}
#else
	ldmia	r0, {r4-r12}
	ldr	r13, [r0, #((_JB_REG_R13 - _JB_REG_R4) * 4)]
	ldr	r14, [r0, #((_JB_REG_R14 - _JB_REG_R4) * 4)]
#endif

#ifndef __thumb__
	/* Validate sp and r14 */
	teq	sp, #0
	it	ne
	teqne	r14, #0
	it	eq
	beq	botch
#else
	cmp sp, #0
	beq botch
	cmp lr, #0
	beq botch
#endif

	/* Set return value */
	movs	r0, r1
	it	eq
	moveq	r0, #0x00000001
	bx lr

	/* validation failed, die die die. */
botch:
	b	.
	.fnend
	.size	longjmp, . - longjmp
