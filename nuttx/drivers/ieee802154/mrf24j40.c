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
#include <string.h>
#include <errno.h>
#include <semaphore.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>

#include <nuttx/ieee802154/mrf24j40.h>
#include <nuttx/ieee802154/ieee802154.h>

#include "mrf24j40.h"

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
#  define CONFIG_IEEE802154_MRF24J40_FREQUENCY 8000000
#endif

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
  sem_t                             rxsem;    /* Reader semaphore to wait on packet reception */
  int                               opened;   /* this device can only be opened once */

  /* real interesting data. actually stored in the device, but copied here when set. */

  uint8_t                           eaddr[8]; /* extended address, FFFFFFFFFFFFFFFF = not set */
  uint8_t                           panid[2]; /* PAN identifier, FFFF = not set */
  uint8_t                           saddr[2]; /* short address, FFFF = not set */
  uint8_t                           channel;  /* 11 to 26 for the 2.4 GHz band */
  uint8_t                           rxmode;   /* Reception mode: Main, no CRC, promiscuous */
  int                               txpower;  /* TX power in dbm TODO: change to mBm (millibel = 1/10 dBm) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    mrf24j40_lock(FAR struct spi_dev_s *spi);
static void    mrf24j40_setreg(FAR struct spi_dev_s *spi, uint32_t addr, uint8_t val);
static uint8_t mrf24j40_getreg(FAR struct spi_dev_s *spi, uint32_t addr);
static int     mrf24j40_initialize(FAR struct mrf24j40_dev_s *dev);
static int     mrf24j40_setchan(FAR struct mrf24j40_dev_s *dev, int chan);
static int     mrf24j40_setpanid(FAR struct mrf24j40_dev_s *dev, FAR uint8_t *panid);
static int     mrf24j40_setsaddr(FAR struct mrf24j40_dev_s *dev, FAR uint8_t *panid);
static int     mrf24j40_seteaddr(FAR struct mrf24j40_dev_s *dev, FAR uint8_t *panid);
static int     mrf24j40_setrxmode(FAR struct mrf24j40_dev_s *dev, int mode);
static int     mrf24j40_energydetect(FAR struct mrf24j40_dev_s *dev);
static int     mrf24j40_settxpower(FAR struct mrf24j40_dev_s *dev, int dbm);
static int     mrf24j40_regdump(FAR struct mrf24j40_dev_s *dev);
static void    mrf24j40_irqwork_rx(FAR struct mrf24j40_dev_s *dev);
static void    mrf24j40_irqworker(FAR void *arg);
static int     mrf24j40_interrupt(int irq, FAR void *context);

static void    mrf24j40_semtake(FAR struct mrf24j40_dev_s *dev);
static int     mrf24j40_open(FAR struct file *filep);
static int     mrf24j40_close(FAR struct file *filep);
static ssize_t mrf24j40_read(FAR struct file *filep, FAR char *buffer, size_t len);
static ssize_t mrf24j40_write(FAR struct file *filep, FAR const char *buffer, size_t len);
static int     mrf24j40_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

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

/* These are pointers to ALL registered MRF24J40 devices.
 * This table is used during irqs to find the context
 * Only one device is supported for now.
 * More devices can be supported in the future by lookup them up
 * using the IRQ number. See the ENC28J60 or CC3000 drivers for reference.
 */

FAR struct mrf24j40_dev_s *g_mrf24j40_devices[1];

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
      addr  &= 0x3F; /* 6-bit address */
      addr <<= 1;
      addr  |= 0x01; /* writing */
      buf[0] = addr;
      len    = 1;
    }
  else
    {
      addr  &= 0x3FF; /* 10-bit address */
      addr <<= 5;
      addr  |= 0x8010; /* writing long */
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
  /* Software reset */

  mrf24j40_setreg(dev->spi, MRF24J40_SOFTRST  , 0x07); /* 00000111 Reset */
  while(mrf24j40_getreg(dev->spi, MRF24J40_SOFTRST) & 0x07);

  /* Apply recommended settings */

  mrf24j40_setreg(dev->spi, MRF24J40_PACON2   , 0x98); /* 10011000 Enable FIFO (default), TXONTS=6 (recommended), TXONT<8:7>=0 (default) */
  mrf24j40_setreg(dev->spi, MRF24J40_TXSTBL   , 0x95); /* 10010101 set the SIFS period. RFSTBL=9, MSIFS=5, aMinSIFSPeriod=14 (min 12) */
  mrf24j40_setreg(dev->spi, MRF24J40_TXPEND   , 0x7C); /* 01111100 set the LIFS period, MLIFS=1Fh=31 aMinLIFSPeriod=40 (min 40) */
  mrf24j40_setreg(dev->spi, MRF24J40_TXTIME   , 0x30); /* 00110000 set the turnaround time, TURNTIME=3 aTurnAroundTime=12 */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON0   , 0x03); /* 00000011 Default channel 11, recommended RF options */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON1   , 0x02); /* 00000010 VCO optimization, recommended value */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON2   , 0x80); /* 10000000 Enable PLL */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON6   , 0x90); /* 10010000 TX filter enable, fast 20M recovery, No bat monitor*/
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON7   , 0x80); /* 10000000 Sleep clock on internal 100 kHz */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON8   , 0x10); /* 00010000 VCO control bit, as recommended */
  mrf24j40_setreg(dev->spi, MRF24J40_SLPCON1  , 0x01); /* 00000001 no CLKOUT, default divisor */
  mrf24j40_setreg(dev->spi, MRF24J40_BBREG2   , 0x8E); /* 10001110 CCA mode ED, no carrier sense, recommenced CS threshold */
  mrf24j40_setreg(dev->spi, MRF24J40_CCAEDTH  , 0x60); /* 01100000 CCA ED threshold, recommended, -69dBm */
  mrf24j40_setreg(dev->spi, MRF24J40_BBREG6   , 0x40); /* 01000000 Append RSSI to rx packets */

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
      lowsyslog(LOG_ERR, "Invalid chan: %d\n",chan);
      return -EINVAL;
    }

  /* 15. Set channel – See Section 3.4 “Channel Selection”. */

  mrf24j40_setreg(dev->spi, MRF24J40_RFCON0, (chan - 11) << 4 | 0x03);

  /* 17. RFCTL (0x36) = 0x04 – Reset RF state machine.
   * 18. RFCTL (0x36) = 0x00.
   */

  mrf24j40_setreg(dev->spi, MRF24J40_RFCTL, 0x04);
  mrf24j40_setreg(dev->spi, MRF24J40_RFCTL, 0x00);

  /* 19. Delay at least 192 μs.
   * We are using up_udelay instead of usleep because usleep
   * can only be used in tasks.
   */

  up_udelay(192);

  dev->channel = chan;

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setpanid
 *
 * Description:
 *   Define the PAN ID the device is operating on.
 *
 ****************************************************************************/

static int mrf24j40_setpanid(FAR struct mrf24j40_dev_s *dev, FAR uint8_t *panid)
{
  mrf24j40_setreg(dev->spi, MRF24J40_PANIDH, panid[0]);
  mrf24j40_setreg(dev->spi, MRF24J40_PANIDL, panid[1]);

  dev->panid[0] = panid[0];
  dev->panid[1] = panid[1];

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setsaddr
 *
 * Description:
 *   Define the device short address. The following addresses are special:
 *   FFFEh : Broadcast
 *   FFFFh : Unspecified
 *
 ****************************************************************************/

static int mrf24j40_setsaddr(FAR struct mrf24j40_dev_s *dev, FAR uint8_t *saddr)
{
  mrf24j40_setreg(dev->spi, MRF24J40_SADRH, saddr[0]);
  mrf24j40_setreg(dev->spi, MRF24J40_SADRL, saddr[1]);

  dev->saddr[0] = saddr[0];
  dev->saddr[1] = saddr[1];

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setsaddr
 *
 * Description:
 *   Define the device extended address. The following addresses are special:
 *   FFFFFFFFFFFFFFFFh : Unspecified
 *
 ****************************************************************************/

static int mrf24j40_seteaddr(FAR struct mrf24j40_dev_s *dev, FAR uint8_t *eaddr)
{
  int i;

  for (i=0; i<8; i++)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_EADR0 + i, eaddr[i]);
      dev->eaddr[i] = eaddr[i];
    }


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
  if (mode<MRF24J40_RXMODE_NORMAL || mode>MRF24J40_RXMODE_NOCRC)
    {
      return -EINVAL;
    }
  reg = mrf24j40_getreg(dev->spi, MRF24J40_RXMCR);
  reg &= ~0x03;
  reg |= mode;
  mrf24j40_setreg(dev->spi, MRF24J40_RXMCR, reg);

  /* set mode options */
  if (mode != MRF24J40_RXMODE_NORMAL)
    {
      /* Promisc and error modes: Disable auto ACK */
      reg = mrf24j40_getreg(dev->spi, MRF24J40_RXMCR);
      reg |= MRF24J40_RXMCR_NOACKRSP;
      mrf24j40_setreg(dev->spi, MRF24J40_RXMCR, reg);
    }
  else
    {
      /* Normal mode : enable auto-ACK */
      reg = mrf24j40_getreg(dev->spi, MRF24J40_RXMCR);
      reg &= ~MRF24J40_RXMCR_NOACKRSP;
      mrf24j40_setreg(dev->spi, MRF24J40_RXMCR, reg);
    }
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

  mrf24j40_setreg(dev->spi, MRF24J40_BBREG6, 0x80);

  /* 2. Wait until RSSIRDY 0x3E<0> is set to ‘1’ – RSSI calculation is complete. */

  while( !(mrf24j40_getreg(dev->spi, MRF24J40_BBREG6) & 0x01) );

  /* 3. Read RSSI 0x210<7:0> – The RSSI register contains the averaged RSSI received power level for 8 symbol periods. */

  reg = mrf24j40_getreg(dev->spi, MRF24J40_RSSI);

  mrf24j40_setreg(dev->spi, MRF24J40_BBREG6, 0x40);

  return reg;
}

/****************************************************************************
 * Name: mrf24j40_settxpower
 *
 * Description:
 *   Define the transmit power. Value is passed in dBm, it is rounded to
 *   the nearest value. Some MRF modules have a power amplifier, this routine
 *   does not care about this. We only change the CHIP output power.
 *
 ****************************************************************************/

static int mrf24j40_settxpower(FAR struct mrf24j40_dev_s *dev, int dbm)
{
  /* for the moment, do not go into details. Only manage the coarse power. */
  uint8_t reg;
  int save_dbm = dbm;
  if(dbm <= -30 && dbm > -40)
    {
      reg = 0xC0;
      dbm += 30;
    }
  else if(dbm <= -20)
    {
      reg = 0x80;
      dbm += 20;
    }
  else if(dbm <= -10)
    {
      reg = 0x40;
      dbm += 10;
    }
  else if(dbm <= 0)
    {
      reg = 0x00;
    }
  else
    {
      return -EINVAL;
    }

  switch(dbm)
    {
      case -9:
      case -8:
      case -7:
      case -6: reg |= 0x07; break;
      case -5: reg |= 0x06; break;
      case -4: reg |= 0x05; break;
      case -3: reg |= 0x04; break;
      case -2: reg |= 0x03; break;
      case -1: reg |= 0x02; break;
      case  0: reg |= 0x00; break; /* value 0x01 is 0.5 db, not used */
      default: return -EINVAL;
    }
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON3, reg);
  dev->txpower = save_dbm;
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_regdump
 *
 * Description:
 *   Display the value of all registers.
 *
 ****************************************************************************/

static int mrf24j40_regdump(FAR struct mrf24j40_dev_s *dev)
{
  uint32_t i;
  lowsyslog(LOG_NOTICE, "short regs\n");
  for (i=0;i<0x40;i++)
    {
      if ((i&15)==0)
        lowsyslog(LOG_NOTICE, "%02x: ",i);
      lowsyslog(LOG_NOTICE, "%02x ", mrf24j40_getreg(dev->spi, i));
      if ((i&15)==15)
        lowsyslog(LOG_NOTICE, "\n");
    }
  lowsyslog(LOG_NOTICE, "long regs\n");
  for (i=0x80000200;i<0x80000250;i++)
    {
      if ((i&15)==0)
        lowsyslog(LOG_NOTICE, "%02x: ", i&0xfff);
      lowsyslog(LOG_NOTICE, "%02x ", mrf24j40_getreg(dev->spi, i));
      if ((i&15)==15)
        lowsyslog(LOG_NOTICE, "\n");
    }
  return 0;
}

/* interrupt management routines */

/****************************************************************************
 * Name: mrf24j40_irqwork_rx
 *
 * Description:
 *   Manage packet reception
 *
 ****************************************************************************/
static struct ieee802154_packet_s rxpk;

static void mrf24j40_irqwork_rx(FAR struct mrf24j40_dev_s *dev)
{
  uint32_t addr;
  uint32_t index;

  lowsyslog(LOG_NOTICE, "rx!");

  /* disable packet reception */
  mrf24j40_setreg(dev->spi, MRF24J40_BBREG1, MRF24J40_BBREG1_RXDECINV);

  /* read packet */
  addr = 0x8000300;
  rxpk.len = mrf24j40_getreg(dev->spi, addr++);
  lowsyslog(LOG_NOTICE, "len %3d\n", rxpk.len);

  for(index = 0; index < rxpk.len; index++)
    {
      rxpk.data[index] = mrf24j40_getreg(dev->spi, addr++);
    }

  rxpk.lqi  = mrf24j40_getreg(dev->spi, addr++);
  rxpk.rssi = mrf24j40_getreg(dev->spi, addr++);

  sem_post(&dev->rxsem);

  /* enable packet reception */
  mrf24j40_setreg(dev->spi, MRF24J40_BBREG1, 0);

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
  FAR struct mrf24j40_dev_s *dev = (FAR struct mrf24j40_dev_s *)arg;
  uint8_t intstat;

  DEBUGASSERT(dev);
  DEBUGASSERT(dev->spi);

  /* Read and store INTSTAT - this clears the register. */

  intstat = mrf24j40_getreg(dev->spi, MRF24J40_INTSTAT);
  lowsyslog(LOG_NOTICE, "INT%02X\n", intstat);

  /* Do work according to the pending interrupts */

  if( (intstat & MRF24J40_INTSTAT_RXIF) )
    {
      /* A packet was received, retrieve it */
      mrf24j40_irqwork_rx(dev);
    }

  /* Re-Enable GPIO interrupts */
  dev->lower->enable(dev->lower, TRUE);

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
  /* To support multiple devices,
   * retrieve the priv structure using the irq number */

  register FAR struct mrf24j40_dev_s *dev = g_mrf24j40_devices[0];

  /* In complex environments, we cannot do SPI transfers from the interrupt
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(work_available(&dev->irqwork));

  /* Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in enc_irqworker() when the work is completed.
   */

  dev->lower->enable(dev->lower, FALSE);
  return work_queue(HPWORK, &dev->irqwork, mrf24j40_irqworker, (FAR void *)dev, 0);
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

      /* Enable interrupts (only rx for now)*/

      mrf24j40_setreg(dev->spi, MRF24J40_INTCON, ~(MRF24J40_INTCON_RXIE) );
      dev->lower->enable(dev->lower, TRUE);

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
      /* Disable interrupts */

      mrf24j40_setreg(dev->spi, MRF24J40_INTCON, 0xFF );
      dev->lower->enable(dev->lower, FALSE);

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
  FAR struct inode *inode = filep->f_inode;
  FAR struct mrf24j40_dev_s *dev = inode->i_private;
  int ret = OK;
  mrf24j40_semtake(dev);

  if (len<sizeof(struct ieee802154_packet_s))
    {
      ret = -EINVAL;
      goto done;
    }

  /* block task until a packet is received */

  ret = sem_trywait(&dev->rxsem);
  if (ret < 0)
    {
      ret = -errno;
      goto done;
    }

  /* return packet */

  ret = sizeof(struct ieee802154_packet_s);
  memcpy(buffer, &rxpk, ret);

done:
  mrf24j40_semgive(dev);
  return ret;
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
  FAR struct inode                     *inode = filep->f_inode;
  FAR struct mrf24j40_dev_s            *dev   = inode->i_private;
  FAR const struct ieee802154_packet_s *packet;
  uint8_t  reg;
  uint32_t addr;
  int      ret = OK;

  mrf24j40_semtake(dev);

  /* sanity checks */

  if (len<sizeof(struct ieee802154_packet_s))
    {
      ret = -EINVAL;
      goto done;
    }

  packet = (FAR const struct ieee802154_packet_s*) buffer;
  if (packet->len > 126)
    {
      ret = -EPERM;
      goto done;
    }

  /* Copy packet to normal TX buffer
   * Beacons and GTS transmission is handled via IOCTLs
   */

  addr = 0x80000000;

  /* header len, 0, TODO for security modes */

  mrf24j40_setreg(dev->spi, addr++, 0);

  /* frame length */

  mrf24j40_setreg(dev->spi, addr++, packet->len);

  /* frame data */

  for (ret=0;ret<len;ret++) /* this sets the correct val for ret */
    {
      mrf24j40_setreg(dev->spi, addr++, packet->data[ret]);
    }

  /* TODO: if the frame control field contains 
   * an acknowledgment request, set the TXNACKREQ bit.
   * See IEEE 802.15.4/2003 7.2.1.1 page 112 for info.
   */

  reg = MRF24J40_TXNCON_TXNTRIG;
  if (packet->data[0] & IEEE802154_FC1_ACKREQ)
    {
      reg |= MRF24J40_TXNCON_TXNACKREQ;
    }

  /* trigger packet emission */
  mrf24j40_setreg(dev->spi, MRF24J40_TXNCON, reg);

done:
  mrf24j40_semgive(dev);
  return ret;
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

      case MAC854IOCSPANID:
        ret = mrf24j40_setpanid(dev, (uint8_t*)arg);
        break;

      case MAC854IOCGPANID:
        *((uint8_t*)(arg+0)) = dev->panid[0];
        *((uint8_t*)(arg+1)) = dev->panid[1];
        ret = OK;
        break;

      case MAC854IOCSSADDR:
        ret = mrf24j40_setsaddr(dev, (uint8_t*)arg);
        break;

      case MAC854IOCGSADDR:
        *((uint8_t*)(arg+0)) = dev->saddr[0];
        *((uint8_t*)(arg+1)) = dev->saddr[1];
        ret = OK;
        break;

      case MAC854IOCSEADDR:
        ret = mrf24j40_seteaddr(dev, (uint8_t*)arg);
        break;

      case MAC854IOCGEADDR:
        memcpy((uint8_t*)arg, dev->eaddr, 8);
        ret = OK;
        break;

      case MAC854IOCSPROMISC:
        ret = mrf24j40_setrxmode(dev, (int)arg?MRF24J40_RXMODE_PROMISC:MRF24J40_RXMODE_NORMAL);
        break;

      case MAC854IOCGPROMISC:
        *((int*)arg) = (dev->rxmode==MRF24J40_RXMODE_PROMISC);
        ret = OK;
        break;

      case MAC854IOCGED:
        *((uint8_t*)arg) = mrf24j40_energydetect(dev);
        ret = OK;
        break;

      case MAC854IOCSTXP:
        ret = mrf24j40_settxpower(dev, (int)arg);
        break;

      case MAC854IOCGTXP:
        *((int*)arg) = dev->txpower;
        break;

      case 1000: /* register dump */
        ret = mrf24j40_regdump(dev);
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
  int ret = -EAGAIN;

  if (minor != 0)
    {
      ret = -EINVAL; /* only one device is supported, with minor 0 */
      goto fail;
    }

  dev = kmm_zalloc(sizeof(struct mrf24j40_dev_s));

  if (!dev)
    {
      ret = -ENOMEM;
      goto freefail;
    }

  /* attach irq */
  if (lower->attach(lower, mrf24j40_interrupt) != 0)
    {
      goto freefail;
    }

  g_mrf24j40_devices[minor] = dev;

  dev->lower   = lower;

  dev->spi     = spi;
  mrf24j40_initialize(dev);
  /* Default device params */
  mrf24j40_setrxmode(dev, 3);
  mrf24j40_setchan  (dev, 11);

  mrf24j40_setpanid(dev, (uint8_t*)"\xff\xff");
  mrf24j40_setsaddr(dev, (uint8_t*)"\xff\xff");
  mrf24j40_seteaddr(dev, (uint8_t*)"\xff\xff\xff\xff\xff\xff\xff\xff");

  /*16. Set transmitter power - See “REGISTER 2-62: RF CONTROL 3 REGISTER (ADDRESS: 0x203)”.*/

  mrf24j40_settxpower(dev, 0);

  sem_init(&dev->sem, 0, 1);
  sem_init(&dev->rxsem, 0, 0);

  sprintf(devname, "/dev/mrf%d", minor);

  return register_driver(devname, &mrf24j40_fops, 0666, dev);

freefail:
  free(dev);
fail:
  return ret;
}

