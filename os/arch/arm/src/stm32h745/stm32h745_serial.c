/****************************************************************************
 * arch/arm/src/stm32h745/stm32h745_serial.c
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdint.h>

#include <tinyara/irq.h>
#include <tinyara/arch.h>
#include <tinyara/serial/serial.h>

#include <arch/serial.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include <stm32h7xx_hal.h>
#include <stm32h7xx_ll_usart.h>


#define CONSOLE_DEV             g_uart3             /* USART3 is console */
#define TTYS0_DEV               g_uart3             /* USART3 is ttyS0 */
#define TTYS1_DEV               g_uart3             /* USART3 is ttyS0 */
#define TTYS2_DEV               g_uart3             /* USART3 is ttyS0 */


struct stm32h745_up_dev_s
{
  uint8_t parity;
  uint8_t bits;
  uint8_t stopbit;
  uint32_t baud;
  uint32_t irq;
  uint32_t tx;
  uint32_t rx;
  uint32_t rts;
  uint32_t cts;
  uint8_t FlowControl;
  bool txint_enable;
  bool rxint_enable;
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  uint8_t iflow:1;      /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  uint8_t oflow:1;      /* output flow control (CTS) enabled */
#endif
  uint8_t tx_level;
};

static int  stm32h745_up_setup(struct uart_dev_s *dev);
static void stm32h745_up_shutdown(struct uart_dev_s *dev);
static int  stm32h745_up_attach(struct uart_dev_s *dev);
static void stm32h745_up_detach(struct uart_dev_s *dev);
static int  stm32h745_up_interrupt(int irq, void *context, FAR void *arg);
static int  stm32h745_up_ioctl(FAR struct uart_dev_s *dev, int cmd, unsigned long arg);
static int  stm32h745_up_receive(struct uart_dev_s *dev, uint8_t *status);
static void stm32h745_up_rxint(struct uart_dev_s *dev, bool enable);
static bool stm32h745_up_rxavailable(struct uart_dev_s *dev);
static void stm32h745_up_send(struct uart_dev_s *dev, int ch);
static void stm32h745_up_txint(struct uart_dev_s *dev, bool enable);
static bool stm32h745_up_txready(struct uart_dev_s *dev);
static bool stm32h745_up_txempty(struct uart_dev_s *dev);

static const struct uart_ops_s g_uart_ops = 
{
  .setup         = stm32h745_up_setup,
  .shutdown      = stm32h745_up_shutdown,
  .attach        = stm32h745_up_attach,
  .detach        = stm32h745_up_detach,
  .ioctl         = stm32h745_up_ioctl,
  .receive       = stm32h745_up_receive,
  .rxint         = stm32h745_up_rxint,
  .rxavailable   = stm32h745_up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = NULL,
#endif
  .send          = stm32h745_up_send,
  .txint         = stm32h745_up_txint,
  .txready       = stm32h745_up_txready,
  .txempty       = stm32h745_up_txempty,
};

static char g_uart3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_USART3_TXBUFSIZE];

static struct stm32h745_up_dev_s g_uart3priv = 
{

  .parity = CONFIG_USART3_PARITY,
  .bits = CONFIG_USART3_BITS,
#if (CONFIG_USART3_2STOP)
  .stopbit = 2,
#else
  .stopbit = 1,
#endif
  .baud = CONFIG_USART3_BAUD,
  .irq  = STM32H745_IRQ_USART3,
  .tx   = 0,
  .rx   = 0,
  .rts  = 0,
  .cts  = 0,
  .FlowControl  = UART_HWCONTROL_NONE,
  .txint_enable = false,
  .rxint_enable = false,
};

static uart_dev_t g_uart3 = 
{
  .isconsole = false,
  .recv = 
  {
    .size   = CONFIG_USART3_RXBUFSIZE,
    .buffer = g_uart3rxbuffer,
  },
  .xmit = 
  {
    .size   = CONFIG_USART3_TXBUFSIZE,
    .buffer = g_uart3txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart3priv,
};

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/
void up_serialinit(void)
{
    uart_register("/dev/console", &CONSOLE_DEV);
    //uart_register("/dev/ttyS0", &TTYS0_DEV);
    //uart_register("/dev/ttyS1", &TTYS1_DEV);
    //uart_register("/dev/ttyS2", &TTYS2_DEV);
}

/****************************************************************************
 * Private Functions - stm32h745_up_setup
 ****************************************************************************/
static int  stm32h745_up_setup(struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Private Functions - stm32h745_up_shutdown
 ****************************************************************************/
static void stm32h745_up_shutdown(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Private Functions - stm32h745_up_attach
 ****************************************************************************/
static int  stm32h745_up_attach(struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Private Functions - stm32h745_up_detach
 ****************************************************************************/
static void stm32h745_up_detach(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Private Functions - stm32h745_up_interrupt
 ****************************************************************************/
static int  stm32h745_up_interrupt(int irq, void *context, FAR void *arg)
{
  return OK;
}

/****************************************************************************
 * Private Functions - stm32h745_up_ioctl
 ****************************************************************************/
static int  stm32h745_up_ioctl(FAR struct uart_dev_s *dev, int cmd, unsigned long arg)
{
  return OK;
}


/****************************************************************************
 * Private Functions - stm32h745_up_receive
 ****************************************************************************/
static int  stm32h745_up_receive(struct uart_dev_s *dev, uint8_t *status)
{
  return OK;
}

/****************************************************************************
 * Private Functions - stm32h745_up_rxint
 ****************************************************************************/
static void stm32h745_up_rxint(struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Private Functions - stm32h745_up_rxavailable
 ****************************************************************************/
static bool stm32h745_up_rxavailable(struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Private Functions - stm32h745_up_send
 ****************************************************************************/
extern UART_HandleTypeDef huart3;
static void stm32h745_up_send(struct uart_dev_s *dev, int ch)
{
#if 0
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFF);
#else
  LL_USART_TransmitData8(USART3, (uint8_t)ch);
#endif
}

/****************************************************************************
 * Private Functions - stm32h745_up_txint
 ****************************************************************************/
static void stm32h745_up_txint(struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Private Functions - stm32h745_up_txready
 ****************************************************************************/
static bool stm32h745_up_txready(struct uart_dev_s *dev)
{
#if 0
    return true;
#else
    return LL_USART_IsActiveFlag_TXE_TXFNF(USART3);
#endif
}


/****************************************************************************
 * Private Functions - stm32h745_up_txempty
 ****************************************************************************/
static bool stm32h745_up_txempty(struct uart_dev_s *dev)
{
#if 0  
    return true;
#else
    return LL_USART_IsActiveFlag_TXE_TXFNF(USART3);
#endif    
}








