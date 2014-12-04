
/****************************************************************************
 * configs/nucleo-f4x1re/src/stm32_mrf24j40.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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

/* MRF24J40 Connections
 *
 * --- ------ -------------- -----------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- -----------------------------------------------------
 * 29  PB6    nCS            Chip Select (same as CC3000, thus cannot use both)
 * 30  PA5    PA5-SPI1-SCK   -
 * 31  PA6    PA6-SPI1-MISO  -
 * 32  PA7    PA7-SPI1-MOSI  -
 * XX  PC7    IRQ            Interrupt request
 *     PA9    nRST           Reset signal
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <debug.h>

#include <stdint.h>
#include <stdio.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <nuttx/ieee802154/mrf24j40.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "nucleo-f4x1re.h"

#ifdef CONFIG_IEEE802154_MRF24J40

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* MRF24J40 is on SPI1 */

#ifndef CONFIG_STM32_SPI1
# error "Need CONFIG_STM32_SPI1 in the configuration"
#endif

/* SPI Assumptions **********************************************************/

#define MRF24J40_SPI_PORTNO 1   /* On SPI1 */
#define MRF24J40_DEVNO      0   /* Only one MRF24J40 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_mrf24j40lower_s
{
  const struct mrf24j40_lower_s lower;    /* Low-level MCU interface, FIRST */
  xcpt_t                        handler;  /* MRF24J40 interrupt handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_attach(FAR const struct mrf24j40_lower_s *lower, xcpt_t handler);
static void up_enable(FAR const struct mrf24j40_lower_s *lower, int state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The MRF24J40 normal provides interrupts to the MCU via a GPIO pin.  The
 * following structure provides an MCU-independent mechanixm for controlling
 * the MRF24J40 GPIO interrupt.
 */

static struct stm32_mrf24j40lower_s g_mrf24j40lower =
{
  .lower =
  {
    .attach  = up_attach,
    .enable  = up_enable,
  },
  .handler = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: struct mrf24j40_lower_s methods
 ****************************************************************************/

static int up_attach(FAR const struct mrf24j40_lower_s *lower, xcpt_t handler)
{
  FAR struct stm32_mrf24j40lower_s *priv = (FAR struct stm32_mrf24j40lower_s *)lower;

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  return OK;
}

static void up_enable(FAR const struct mrf24j40_lower_s *lower, int state)
{
  FAR struct stm32_mrf24j40lower_s *priv = (FAR struct stm32_mrf24j40lower_s *)lower;
  DEBUGASSERT(priv->handler);

  if(state)
    {
      (void)stm32_gpiosetevent(GPIO_MRF24J40_INTR, false, true, true, priv->handler);
    }
  else
    {
      (void)stm32_gpiosetevent(GPIO_MRF24J40_INTR, false, true, true, NULL);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ieeeinitialize
 ****************************************************************************/
#define GPIO_MRF24J40  (GPIO_PORTA | GPIO_PIN9 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)

int stm32_mrf24j40initialize(void)
{
  FAR struct spi_dev_s *spi;
  int ret;

  /* Assumptions:
   * 1) SPI1 pins were configured in up_spi.c early in the boot-up phase.
   * 2) Clocking for the SPI1 peripheral was also provided earlier in boot-up.
   */

  spi = up_spiinitialize(MRF24J40_SPI_PORTNO);
  if (!spi)
    {
      lldbg("Failed to initialize SPI port %d\n", MRF24J40_SPI_PORTNO);
      return -EAGAIN;
    }

  /* Reset the device */
  stm32_configgpio(GPIO_MRF24J40_RESET);
  stm32_gpiowrite(GPIO_MRF24J40_RESET, 0);
  usdelay(1000);
  stm32_gpiowrite(GPIO_MRF24J40_RESET, 1);

  /* Bind the SPI port to the MRF24J40 driver */

  ret = mrf24j40_register(spi, &g_mrf24j40lower.lower, MRF24J40_DEVNO);
  if (ret < 0)
    {
      lldbg("Failed to bind SPI port %d MRF24J40 device %d: %d\n",
             MRF24J40_SPI_PORTNO, MRF24J40_DEVNO, ret);
      return -EAGAIN;
    }

  llvdbg("Bound SPI port %d to MRF24J40 device %d\n",
        MRF24J40_SPI_PORTNO, MRF24J40_DEVNO);

  return ret;
}

#endif /* CONFIG_IEEE802154_MRF24J40 */
