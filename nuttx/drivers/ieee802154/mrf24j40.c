/****************************************************************************
 * drivers/ieee802154/mrf24j40.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <assert.h>

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <semaphore.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>

#include <nuttx/ieee802154/mrf24j40.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef CONFIG_SCHED_HPWORK
#error High priority work queue required in this driver
#endif

#ifndef CONFIG_MRF24J40_SPIMODE
#  define CONFIG_MRF24J40_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_MRF24J40_FREQUENCT
#  define CONFIG_MRF24J40_FREQUENCT 10000000
#endif

/* MRF24J40 Registers *******************************************************************/

#define MRF24J40_RXMCR     0x00
#define MRF24J40_PANIDL    0x01
#define MRF24J40_PANIDH    0x02
#define MRF24J40_SADRL     0x03
#define MRF24J40_SADRH     0x04
#define MRF24J40_EADR0     0x05
#define MRF24J40_EADR1     0x06
#define MRF24J40_EADR2     0x07
#define MRF24J40_EADR3     0x08
#define MRF24J40_EADR4     0x09
#define MRF24J40_EADR5     0x0A
#define MRF24J40_EADR6     0x0B
#define MRF24J40_EADR7     0x0C
#define MRF24J40_RXFLUSH   0x0D
#define MRF24J40_ORDER     0x10
#define MRF24J40_TXMCR     0x11
#define MRF24J40_ACKTMOUT  0x12
#define MRF24J40_ESLOTG1   0x13
#define MRF24J40_SYMTICKL  0x14
#define MRF24J40_SYMTICKH  0x15
#define MRF24J40_PACON0    0x16
#define MRF24J40_PACON1    0x17
#define MRF24J40_PACON2    0x18
#define MRF24J40_TXBCON0   0x1A
#define MRF24J40_TXNCON    0x1B
#define MRF24J40_TXG1CON   0x1C
#define MRF24J40_TXG2CON   0x1D
#define MRF24J40_ESLOTG23  0x1E
#define MRF24J40_ESLOTG45  0x1F
#define MRF24J40_ESLOTG67  0x20
#define MRF24J40_TXPEND    0x21
#define MRF24J40_WAKECON   0x22
#define MRF24J40_FRMOFFSET 0x23
#define MRF24J40_TXSTAT    0x24
#define MRF24J40_TXBCON1   0x25
#define MRF24J40_GATECLK   0x26
#define MRF24J40_TXTIME    0x27
#define MRF24J40_HSYMTMRL  0x28
#define MRF24J40_HSYMTMRH  0x29
#define MRF24J40_SOFTRST   0x2A
#define MRF24J40_SECCON0   0x2C
#define MRF24J40_SECCON1   0x2C
#define MRF24J40_TXSTBL    0x2E
#define MRF24J40_RXSR      0x30
#define MRF24J40_INTSTAT   0x31
#define MRF24J40_INTCON    0x32
#define MRF24J40_GPIO      0x33
#define MRF24J40_TRISGPIO  0x34
#define MRF24J40_SLPACK    0x35
#define MRF24J40_RFCTL     0x36
#define MRF24J40_SECCR2    0x37
#define MRF24J40_BBREG0    0x38
#define MRF24J40_BBREG1    0x39
#define MRF24J40_BBREG2    0x3A
#define MRF24J40_BBREG3    0x3B
#define MRF24J40_BBREG4    0x3C
#define MRF24J40_BBREG6    0x3E
#define MRF24J40_CCAEDTH   0x3F

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* A MRF24J40 device instance */

struct mrf24j40_dev_s
{
  struct work_s                     irqwork; /* Interrupt continuation work queue support */
  FAR struct spi_dev_s              *spi;    /* Saved SPI interface instance */
  FAR const struct mrf24j40_lower_s *lower;  /* Low-level MCU-specific support */
  sem_t                             sem;     /* Access serialization semaphore */
  int                               opened;  /* device can only be opened once */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     mrf24j40_open(FAR struct file *filep);
static int     mrf24j40_close(FAR struct file *filep);
static int     mrf24j40_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations mrf24j40_fops =
{
  mrf24j40_open,  /* open */
  mrf24j40_close, /* close */
  0,              /* read */
  0,              /* write */
  0,              /* seek */
  mrf24j40_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0             /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mrf24j40_lock
 *
 * Description:
 *   Acquire exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static void mrf24j40_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK        (spi, 1);
  SPI_SETBITS     (spi, 8);
  SPI_SETMODE     (spi, 0);
  SPI_SETFREQUENCY(spi, 10000000);
}

/****************************************************************************
 * Name: mrf24j40_unlock
 *
 * Description:
 *   Release exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static inline void mrf24j40_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi,0);
}

/****************************************************************************
 * Name: mrf24j40_semtake
 *
 * Description:
 *   Acquire the semaphore used for access serialization.
 *
 ****************************************************************************/

static void mrf24j40_semtake(FAR struct mrf24j40_dev_s *dev)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&dev->sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */
      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: mrf24j40_semgive
 *
 * Description:
 *   Release the semaphore used for access serialization.
 *
 ****************************************************************************/

static inline void mrf24j40_semgive(FAR struct mrf24j40_dev_s *dev)
{
  sem_post(&dev->sem);
}

/****************************************************************************
 * Name: mrf24j40_open
 *
 * Description:
 *   Open the MRF24J40 device.
 *
 ****************************************************************************/

static int mrf24j40_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mrf24j40_dev_s *dev = inode->i_private;

  mrf24j40_semtake(dev);

  if (dev->opened)
    {
      return -EMFILE;
    }
  else
    {
      dev->opened = TRUE;
    }
  mrf24j40_semgive(dev);
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_close
 *
 * Description:
 *   Close the MRF24J40 device.
 *
 ****************************************************************************/

static int mrf24j40_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mrf24j40_dev_s *dev = inode->i_private;
  int ret = OK;

  mrf24j40_semtake(dev);

  if(!dev->opened)
    {
    ret = -EIO;
    }
  else
    {
    dev->opened = FALSE;
    }

  mrf24j40_semgive(dev);
  return ret;
}

/****************************************************************************
 * Name: mrf24j40_open
 *
 * Description:
 *   Control the MRF24J40 device. This is where the real operations happen.
 *
 ****************************************************************************/

static int mrf24j40_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mrf24j40_dev_s *dev = inode->i_private;
  int ret = EINVAL;

  mrf24j40_semtake(dev);

  switch(cmd)
    {
    }

  mrf24j40_semgive(dev);
  return ret;
}

/****************************************************************************
 * Name: mrf24j40_irqworker
 *
 * Description:
 *   Perform interrupt handling logic outside of the interrupt handler (on
 *   the work queue thread).
 *
 * Parameters:
 *   arg     - The reference to the driver structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void mrf24j40_irqworker(FAR void *arg)
{
  FAR struct mrf24j40_dev_s *priv = (FAR struct mrf24j40_dev_s *)arg;

  DEBUGASSERT(priv);
  DEBUGASSERT(priv->spi);

  /* Get exclusive access to the SPI bus */
  mrf24j40_lock(priv->spi);

  /* Do IRQ work */

  /* Re-Enable GPIO interrupts */

  priv->lower->enable(priv->lower, TRUE);

  /* Release lock on the SPI bus */

  mrf24j40_unlock(priv->spi);
}

/****************************************************************************
 * Name: mrf24j40_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mrf24j40_interrupt(int irq, FAR void *context)
{
  register FAR struct mrf24j40_dev_s *priv = (FAR struct mrf24j40_dev_s *)context;

  /* In complex environments, we cannot do SPI transfers from the interrupt
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(work_available(&priv->irqwork));

  /* Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in enc_irqworker() when the work is completed.
   */

  priv->lower->enable(priv->lower, FALSE);
  return work_queue(HPWORK, &priv->irqwork, mrf24j40_irqworker, (FAR void *)priv, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mrf24j40_initialize
 *
 * Description:
 *   Register /dev/mrf%d
 *
 ****************************************************************************/

int mrf24j40_initialize(FAR struct spi_dev_s *spi, FAR const struct mrf24j40_lower_s *lower, int minor)
{
  char devname[16];
  FAR struct mrf24j40_dev_s *dev;

  dev = kmm_zalloc(sizeof(struct mrf24j40_dev_s));

  if (!dev)
    {
      return -EAGAIN;
    }

  /* attach irq */
  if (lower->attach(lower, mrf24j40_interrupt) != 0)
    {
      free(dev);
      return -EAGAIN;
    }

  sprintf(devname, "/dev/mrf%d", minor);

  (void)register_driver(devname, &mrf24j40_fops, 0666, dev);

  return 0;


  free(dev);

  return -EAGAIN;
}

