/****************************************************************************
 * arch/arm/src/am67/am67_serial.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/serial/serial.h>
#include <nuttx/serial/uart_16550.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include "arm_internal.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define AM67_UART_MDR1_OFFSET  0x20   /* MDR1: 0 = UART 16x, 7 = disabled */
#define AM67_TISCI_DEV_UART6   158u   /* J722S TISCI device ID, MAIN_UART6 */

#if defined(USE_SERIALDRIVER) /* && defined(HAVE_UART_DEVICE)*/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: open_uart
 *
 * Description:
 *   Initialize UART by clearing the register at offset 0x20 from base.
 *
 ****************************************************************************/

static void open_uart(void)
{
  putreg32(0, CONFIG_16550_UART0_BASE + AM67_UART_MDR1_OFFSET);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_16550_UART1
int am67_tisci_device_on(uint32_t id);

/****************************************************************************
 * Name: am67_uart6_enable
 *
 * Description:
 *   Power MAIN_UART6 (NuttX UART1) through the DMSC and put it in UART 16x
 *   mode. The px4-r5f overlay keeps Linux off this UART, so nobody else
 *   turns it on, and MDR1 resets to 0x7 (disabled). Must run before the
 *   port is opened.
 *
 ****************************************************************************/

int am67_uart6_enable(void)
{
  int ret = am67_tisci_device_on(AM67_TISCI_DEV_UART6);

  if (ret < 0)
    {
      return ret;
    }

  putreg32(0, CONFIG_16550_UART1_BASE + AM67_UART_MDR1_OFFSET);
  return 0;
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Initialize the serial port by performing early initialization, main
 *   initialization, and opening the UART.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
  u16550_earlyserialinit();
  u16550_serialinit();
  open_uart();
}

/****************************************************************************
 * Name: uart_getreg
 *
 * Description:
 *   Read a value from the UART register at the specified offset from the
 *   base address.
 *
 ****************************************************************************/

uart_datawidth_t uart_getreg(FAR struct u16550_s *priv, unsigned int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: uart_putreg
 *
 * Description:
 *   Write a value to the UART register at the specified offset from the
 *   base address.
 *
 ****************************************************************************/

void uart_putreg(FAR struct u16550_s *priv, unsigned int offset,
                 uart_datawidth_t value)
{
  putreg32(value, priv->uartbase + offset);
}

#endif /* USE_SERIALDRIVER */
