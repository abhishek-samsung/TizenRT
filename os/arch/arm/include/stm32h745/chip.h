/************************************************************************************
 * arch/arm/include/stm32h745/chip.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *           Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_STM32H745_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32H745_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <tinyara/config.h>


#if defined(CONFIG_STM32H745_STM32H745ZI)
#  define STM32H745_SRAM1_SIZE       (512*1024)  /* 512Kb SRAM1 on AHB bus Matrix */
#else
#  error "Unsupported STM32H745 chip"
#endif

#if defined(CONFIG_STM32H745_STM32H745ZI)
#  define STM32H745_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32H745_NGTIM32                  2   /* 32-bit general timers TIM2 and 5 with DMA */
#  define STM32H745_NGTIM16                  2   /* 16-bit general timers TIM3 and 4 with DMA */
#  define STM32H745_NGTIMNDMA                3   /* 16-bit general timers TIM15-17 without DMA */
#  define STM32H745_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32H745_NUART                    2   /* UART 4-5 */
#  define STM32H745_NUSART                   3   /* USART 1-3 */
#  define STM32H745_NPORTS                   9   /* 9 GPIO ports, GPIOA-I */
#  define STM32H745_NCRC                     1   /* CRC */
#endif /* CONFIG_STM32H745_STM32H745ZI */


/* NVIC priority levels *************************************************************/
/* 16 Programmable interrupt levels */

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_STM32H745_CHIP_H */
