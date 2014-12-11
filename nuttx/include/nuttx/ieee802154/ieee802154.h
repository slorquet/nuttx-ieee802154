/****************************************************************************
 * include/nuttx/ieee802154/ieee802154.h
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

#ifndef __INCLUDE_NUTTX_IEEE802154_IEEE802154_H
#define __INCLUDE_NUTTX_IEEE802154_IEEE802154_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* None at the moment */

/* IEEE 802.15.4 MAC Interface **********************************************/

/* IOCTL commands
 *
 * Discrete IEEE802.15.4 drivers do not support the character driver read() or
 * seek() methods.  The remaining driver methods behave as follows:
 *
 * 1) The write() method will send a packet on the radio interface with the 
 *    current settings.
 * 2) The poll() method can be used to notify a client if there is a change
 *    in any of the joystick button inputs.  This feature, of course,
 *    depends upon interrupt GPIO support from the platform.  NOTE: that
 *    semantics of poll() for POLLIN are atypical:  The successful poll
 *    means that the button data has changed and has nothing to with the
 *    availability of data to be read; data is always available to be
 *    read.
 * 3) The ioctl() method supports the commands documented below:
 */

#define MAC854IOCGCHAN     _MAC854IOC(0x0001) /* Get current channel (int*) */
#define MAC854IOCSCHAN     _MAC854IOC(0x0002) /* Set current channel (int) */
#define MAC854IOCGPANID    _MAC854IOC(0x0003) /* Get PAN ID (uint8_t[2]) */
#define MAC854IOCSPANID    _MAC854IOC(0x0004) /* Set PAN ID (uint8_t[2]) */
#define MAC854IOCGSADDR    _MAC854IOC(0x0005) /* Get Short address (uint8_t[2]) */
#define MAC854IOCSSADDR    _MAC854IOC(0x0006) /* Set Short address (uint8_t[2]) */
#define MAC854IOCGEADDR    _MAC854IOC(0x0007) /* Get Ext address (uint8_t[8]) */
#define MAC854IOCSEADDR    _MAC854IOC(0x0008) /* Set Ext address (uint8_t[8]) */
#define MAC854IOCGPROMISC  _MAC854IOC(0x0009) /* Get Promiscuous mode (int*) */
#define MAC854IOCSPROMISC  _MAC854IOC(0x000A) /* Set Promiscuous mode (int) */
#define MAC854IOCGTXP      _MAC854IOC(0x000B) /* Get TX power (int*) */
#define MAC854IOCSTXP      _MAC854IOC(0x000C) /* Set TX power in mdBm (int) */

#define MAC854IOCGCCA      _MAC854IOC(0x000D) /* Get Clear Channel Assessement settings (struct ieee802154_cca_s*) */
#define MAC854IOCSCCA      _MAC854IOC(0x000E) /* Set Clear Channel Assessement settings (struct ieee802154_cca_s*) */

#define MAC854IOCGORDER    _MAC854IOC(0x000F) /* Get Beacon and Superframe orders (uint8_t*) */
#define MAC854IOCSORDER    _MAC854IOC(0x0010) /* Set Beacon and Superframe orders (uint8_t) */

#define MAC854IOCGED       _MAC854IOC(0x0011) /* Run energy detection on current channel (uint8_t*) */

/* IEEE 802.15.4 definitions */

/* Frame control field masks, 2 bytes 
 * Seee IEEE 802.15.4/2003 7.2.1.1 page 112
 */

#define IEEE802154_FC1_FTYPE  0xE0 /* Frame type, bits 0-2 */
#define IEEE802154_FC1_SEC    0x10 /* Security Enabled, bit 3 */
#define IEEE802154_FC1_PEND   0x08 /* Frame pending, bit 4 */
#define IEEE802154_FC1_ACKREQ 0x04 /* Acknowledge request, bit 5 */
#define IEEE802154_FC1_INTRA  0x02 /* Intra PAN, bit 6 */
#define IEEE802154_FC2_DADR   0x30 /* Dest   addressing mode, bits 10-11 */
#define IEEE802154_FC2_SADR   0x03 /* Source addressing mode, bits 14-15 */

/* Frame types */

#define IEEE802154_FRAME_BEACON  0x00
#define IEEE802154_FRAME_DATA    0x01
#define IEEE802154_FRAME_ACK     0x02
#define IEEE802154_FRAME_COMMAND 0x03

/* Addressing modes */

#define IEEE802154_ADDR_NONE     0x00
#define IEEE802154_ADDR_SHORT    0x02
#define IEEE802154_ADDR_EXT      0x03

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ieee802154_packet_s {
  uint8_t len;
  uint8_t data[127];
  uint8_t lqi;
  uint8_t rssi;
};

struct ieee802154_cca_s
{
  uint8_t                           use_ed  : 1; /* CCA using ED */
  uint8_t                           use_cs  : 1; /* CCA using carrier sense */
  uint8_t                           edth;     /* Energy detection threshold for CCA */
  uint8_t                           csth;     /* Carrier sense threshold for CCA */
};

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_IEEE802154__MRF24J40_H */
