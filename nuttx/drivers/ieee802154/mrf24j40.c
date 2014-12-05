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
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <semaphore.h>
#include <unistd.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>

#include <nuttx/ieee802154/mrf24j40.h>
#include <nuttx/ieee802154/ieee802154.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef CONFIG_SCHED_HPWORK
#error High priority work queue required in this driver
#endif

#ifndef CONFIG_IEEE802154_MRF24J40_SPIMODE
#  define CONFIG_IEEE802154_MRF24J40_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_IEEE802154_MRF24J40_FREQUENCY
#  define CONFIG_IEEE802154_MRF24J40_FREQUENCY 1000000
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

#define MRF24J40_RFCON0    0x80000200
#define MRF24J40_RFCON1    0x80000201
#define MRF24J40_RFCON2    0x80000202
#define MRF24J40_RFCON3    0x80000203
#define MRF24J40_RFCON5    0x80000205
#define MRF24J40_RFCON6    0x80000206
#define MRF24J40_RFCON7    0x80000207
#define MRF24J40_RFCON8    0x80000208
#define MRF24J40_SLPCAL0   0x80000209
#define MRF24J40_SLPCAL1   0x8000020A
#define MRF24J40_SLPCAL2   0x8000020B
#define MRF24J40_RFSTATE   0x8000020F
#define MRF24J40_RSSI      0x80000210
#define MRF24J40_SLPCON0   0x80000211
#define MRF24J40_SLPCON1   0x80000220
#define MRF24J40_WAKETIMEL 0x80000222
#define MRF24J40_WAKETIMEH 0x80000223
#define MRF24J40_REMCNTL   0x80000224
#define MRF24J40_REMCNTH   0x80000225
#define MRF24J40_MAINCNT0  0x80000226
#define MRF24J40_MAINCNT1  0x80000227
#define MRF24J40_MAINCNT2  0x80000228
#define MRF24J40_MAINCNT3  0x80000229
#define MRF24J40_TESTMODE  0x8000022F
#define MRF24J40_ASSOEADR0 0x80000230
#define MRF24J40_ASSOEADR1 0x80000231
#define MRF24J40_ASSOEADR2 0x80000232
#define MRF24J40_ASSOEADR3 0x80000233
#define MRF24J40_ASSOEADR4 0x80000234
#define MRF24J40_ASSOEADR5 0x80000235
#define MRF24J40_ASSOEADR6 0x80000236
#define MRF24J40_ASSOEADR7 0x80000237
#define MRF24J40_ASSOSADR0 0x80000238
#define MRF24J40_ASSOSADR1 0x80000239
#define MRF24J40_UPNONCE0  0x80000240
#define MRF24J40_UPNONCE1  0x80000241
#define MRF24J40_UPNONCE2  0x80000242
#define MRF24J40_UPNONCE3  0x80000243
#define MRF24J40_UPNONCE4  0x80000244
#define MRF24J40_UPNONCE5  0x80000245
#define MRF24J40_UPNONCE6  0x80000246
#define MRF24J40_UPNONCE7  0x80000247
#define MRF24J40_UPNONCE8  0x80000248
#define MRF24J40_UPNONCE9  0x80000249
#define MRF24J40_UPNONCE10 0x8000024A
#define MRF24J40_UPNONCE11 0x8000024B
#define MRF24J40_UPNONCE12 0x8000024C

/* Definitions for the device structure */

#define MRF24J40_RXMODE_NORMAL  0
#define MRF24J40_RXMODE_PROMISC 1
#define MRF24J40_RXMODE_NOCRC   2

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* A MRF24J40 device instance */

struct mrf24j40_dev_s
{
  struct work_s                     irqwork;  /* Interrupt continuation work queue support */
  FAR struct spi_dev_s              *spi;     /* Saved SPI interface instance */
  FAR const struct mrf24j40_lower_s *lower;   /* Low-level MCU-specific support */
  sem_t                             sem;      /* Access serialization semaphore */
  int                               opened;   /* this device can only be opened once */

  /* real interesting data. actually stored in the device, but copied here when set. */

  uint8_t                           eaddr[8]; /* extended address, FFFFFFFFFFFFFFFF = not set */
  uint8_t                           panid[2]; /* PAN identifier, FFFF = not set */
  uint8_t                           saddr[2]; /* short address, FFFF = not set */
  uint8_t                           channel;  /* 11 to 26 for the 2.4 GHz band */
  uint8_t                           rxmode;   /* Reception mode: Main, no CRC, promiscuous */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     mrf24j40_open(FAR struct file *filep);
static int     mrf24j40_close(FAR struct file *filep);
static int     mrf24j40_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static ssize_t mrf24j40_read(FAR struct file *filep, FAR char *buffer, size_t len);
static ssize_t mrf24j40_write(FAR struct file *filep, FAR const char *buffer, size_t len);


/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations mrf24j40_fops =
{
  mrf24j40_open,  /* open */
  mrf24j40_close, /* close */
  mrf24j40_read,  /* read */
  mrf24j40_write, /* write */
  0,              /* seek */
  mrf24j40_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0             /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
#ifndef CONFIG_SPI_EXCHANGE
#error CONFIG_SPI_EXCHANGE required for this driver
#endif

/* hardware access routines */

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
  SPI_SETFREQUENCY(spi, CONFIG_IEEE802154_MRF24J40_FREQUENCY );
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

static void mrf24j40_setreg(FAR struct spi_dev_s *spi, uint32_t addr, uint8_t val)
{
  uint8_t buf[3];
  int     len;
  if(!(addr&0x80000000))
    {
    /* 6-bit address */
    addr  &= 0x3F;
    addr <<= 1;
    addr  |= 0x01;
    buf[0] = addr;
    len    = 1;
    }
  else
    {
    /* 10-bit address */
    addr  &= 0x3FF;
    addr <<= 5;
    addr  |= 0x8010;
    buf[0] = (addr >>   8);
    buf[1] = (addr & 0xFF);
    len    = 2;
    }
  buf[len++] = val;

  mrf24j40_lock(spi);
  SPI_SELECT(spi, SPIDEV_IEEE802154, true);
  SPI_SNDBLOCK(spi, buf, len);
  SPI_SELECT(spi, SPIDEV_IEEE802154, false);
  mrf24j40_unlock(spi);
}

static uint8_t mrf24j40_getreg(FAR struct spi_dev_s *spi, uint32_t addr)
{
  uint8_t buf[3];
  uint8_t rx[3];
  int     len;
  if(!(addr&0x80000000))
    {
    /* 6-bit address */
    addr  &= 0x3F;
    addr <<= 1;
    buf[0] = addr;
    len    = 1;
    }
  else
    {
    /* 10-bit address */
    addr  &= 0x3FF;
    addr <<= 5;
    addr  |= 0x8000;
    buf[0] = (addr >>   8);
    buf[1] = (addr & 0xFF);
    len    = 2;
    }
  buf[len++] = 0xFF; /* dummy */

  mrf24j40_lock(spi);
  SPI_SELECT(spi, SPIDEV_IEEE802154, true);
  SPI_EXCHANGE(spi, buf, rx, len);
  SPI_SELECT(spi, SPIDEV_IEEE802154, false);
  mrf24j40_unlock(spi);

  return rx[len-1];
}

/****************************************************************************
 * Name: mrf24j40_initialize
 *
 * Description:
 *   Reset the device and put in in order of operation
 *
 ****************************************************************************/

static int mrf24j40_initialize(FAR struct mrf24j40_dev_s *dev)
{
  /*  1. SOFTRST (0x2A) = 0x07 – Perform a software Reset. The bits will be automatically cleared to ‘0’ by hardware.*/
  /*  2. PACON2 (0x18) = 0x98 – Initialize FIFOEN = 1 and TXONTS = 0x6.*/
  /*  3. TXSTBL (0x2E) = 0x95 – Initialize RFSTBL = 0x9.*/
  /*  4. RFCON0 (0x200) = 0x03 – Initialize RFOPT = 0x03.*/
  /*  5. RFCON1 (0x201) = 0x01 – Initialize VCOOPT = 0x02.*/
  /*  6. RFCON2 (0x202) = 0x80 – Enable PLL (PLLEN = 1).*/
  /*  7. RFCON6 (0x206) = 0x90 – Initialize TXFIL = 1 and 20MRECVR = 1.*/
  /*  8. RFCON7 (0x207) = 0x80 – Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).*/
  /*  9. RFCON8 (0x208) = 0x10 – Initialize RFVCO = 1.*/
  /* 10. SLPCON1 (0x220) = 0x21 – Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01.*/
  /* 11. BBREG2 (0x3A) = 0x80 – Set CCA mode to ED.*/
  /* 12. CCAEDTH = 0x60 – Set CCA ED threshold.*/
  /* 13. BBREG6 (0x3E) = 0x40 – Set appended RSSI value to RXFIFO.*/

  mrf24j40_setreg(dev->spi, MRF24J40_SOFTRST, 0x07);

  /* wait end of reset */
  while(mrf24j40_getreg(dev->spi, MRF24J40_SOFTRST) & 0x07);

  mrf24j40_setreg(dev->spi, MRF24J40_PACON2 , 0x98);
  mrf24j40_setreg(dev->spi, MRF24J40_TXSTBL , 0x95);
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON0 , 0x03);
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON1 , 0x01);
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON2 , 0x80);
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON6 , 0x90);
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON7 , 0x80);
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON8 , 0x10);
  mrf24j40_setreg(dev->spi, MRF24J40_SLPCON1, 0x21);
  mrf24j40_setreg(dev->spi, MRF24J40_BBREG2 , 0x80);
  mrf24j40_setreg(dev->spi, MRF24J40_CCAEDTH, 0x60);
  mrf24j40_setreg(dev->spi, MRF24J40_BBREG6 , 0x40);

  /*14. Enable interrupts*/

  /*16. Set transmitter power - See “REGISTER 2-62: RF CONTROL 3 REGISTER (ADDRESS: 0x203)”.*/

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setchan
 *
 * Description:
 *   Define the current radio channel the device is operating on.
 *   In the 2.4 GHz, there are 16 channels, each 2 MHz wide, 5 MHz spacing:
 *   Chan   MHz       Chan   MHz       Chan   MHz       Chan   MHz
 *     11  2405         15  2425         19  2445         23  2465
 *     12  2410         16  2430         20  2450         24  2470
 *     13  2415         17  2435         21  2455         25  2475
 *     14  2420         18  2440         22  2460         26  2480
 *
 ****************************************************************************/

static int mrf24j40_setchan(FAR struct mrf24j40_dev_s *dev, int chan)
{
  if(chan<11 || chan>26)
    {
      return -EINVAL;
    }

  /* 15. Set channel – See Section 3.4 “Channel Selection”. */

  mrf24j40_setreg(dev->spi, MRF24J40_RFCON0, (chan - 11) << 4 | 0x03);

  /* 17. RFCTL (0x36) = 0x04 – Reset RF state machine.
   * 18. RFCTL (0x36) = 0x00.
   */

  mrf24j40_setreg(dev->spi, MRF24J40_RFCTL, 0x04);
  mrf24j40_setreg(dev->spi, MRF24J40_RFCTL, 0x00);

  /* 19. Delay at least 192 μs. */

  usleep(192);

  dev->channel = chan;

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setrxmode
 *
 * Description:
 *   Set the RX mode (normal or promiscuous)
 *
 ****************************************************************************/

int mrf24j40_setrxmode(FAR struct mrf24j40_dev_s *dev, int mode)
{
  uint8_t reg;
  if(mode<MRF24J40_RXMODE_NORMAL || mode>MRF24J40_RXMODE_NOCRC)
    {
      return -EINVAL;
    }
  reg = mrf24j40_getreg(dev->spi, MRF24J40_RXMCR);
  reg &= ~0x03;
  reg |= mode;
  mrf24j40_setreg(dev->spi, MRF24J40_RXMCR, reg);
  dev->rxmode = mode;
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_energydetect
 *
 * Description:
 *   Measure the RSSI level for the current channel.
 *
 ****************************************************************************/

static int mrf24j40_energydetect(FAR struct mrf24j40_dev_s *dev)
{
  uint8_t reg;
  /*set RSSI average duration to 8 symbols */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_TXBCON1);
  reg |= 0x30;
  mrf24j40_setreg(dev->spi, MRF24J40_TXBCON1, reg);

  /* 1. Set RSSIMODE1 0x3E<7> – Initiate RSSI calculation. */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_BBREG6);
  reg |= 0x80;
  mrf24j40_setreg(dev->spi, MRF24J40_BBREG6, reg);

  /* 2. Wait until RSSIRDY 0x3E<0> is set to ‘1’ – RSSI calculation is complete. */

  while( !(mrf24j40_getreg(dev->spi, MRF24J40_BBREG6) & 0x01) );

  /* 3. Read RSSI 0x210<7:0> – The RSSI register contains the averaged RSSI received power level for 8 symbol periods. */

  return mrf24j40_getreg(dev->spi, MRF24J40_RSSI);
}

/* interrupt management routines */

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

  /* Read and store INTSTAT - this clears the register. */

  /* Do work according to the pending interrupts */

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

/* device access routines */

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
 * Name: mrf24j40_read
 *
 * Description:
 *   Return a packet from the receive queue. The buffer must be a pointer to a
 *   struct ieee802154_packet_s structure, with a correct length.
 *
 ****************************************************************************/

static ssize_t mrf24j40_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  return -EACCES;
}

/****************************************************************************
 * Name: mrf24j40_write
 *
 * Description:
 *   Put a packet in the send queue. The packet will be sent as soon as possible.
 *
 ****************************************************************************/

static ssize_t mrf24j40_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
  return -EACCES;
}

/****************************************************************************
 * Name: mrf24j40_ioctl
 *
 * Description:
 *   Control the MRF24J40 device. This is where the real operations happen.
 *
 ****************************************************************************/

static int mrf24j40_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mrf24j40_dev_s *dev = inode->i_private;
  int ret = -EINVAL;

  mrf24j40_semtake(dev);

  switch(cmd)
    {
      case MAC854IOCSCHAN:
        ret = mrf24j40_setchan(dev, (int)arg);
        break;

      case MAC854IOCGCHAN:
        *((int*)arg) = dev->channel;
        ret = OK;
        break;

      case MAC854IOCGED:
        *((int*)arg) = mrf24j40_energydetect(dev);
        ret = OK;
        break;

      case MAC854IOCGPROMISC:
        *((int*)arg) = (dev->rxmode==MRF24J40_RXMODE_PROMISC);
        break;

      case MAC854IOCSPROMISC:
        mrf24j40_setrxmode(dev, (int)arg?MRF24J40_RXMODE_PROMISC:MRF24J40_RXMODE_NORMAL);
        break;

    }

  mrf24j40_semgive(dev);
  return ret;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mrf24j40_register
 *
 * Description:
 *   Register /dev/mrf%d
 *
 ****************************************************************************/

int mrf24j40_register(FAR struct spi_dev_s *spi, FAR const struct mrf24j40_lower_s *lower, int minor)
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

  /* Default device params */
  mrf24j40_setrxmode(dev, MRF24J40_RXMODE_NORMAL);
  mrf24j40_setchan  (dev, 11);


  dev->spi     = spi;
  mrf24j40_initialize(dev);

  sem_init(&dev->sem, 0, 1);
  sprintf(devname, "/dev/mrf%d", minor);

  return register_driver(devname, &mrf24j40_fops, 0666, dev);

  free(dev);

  return -EAGAIN;
}

