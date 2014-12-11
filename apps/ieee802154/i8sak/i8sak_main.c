/****************************************************************************
 * ieee802154/i8/i8_main.c
 * IEEE 802.15.4 Swiss Army Knife
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/ieee802154/ieee802154.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ieee802154_packet_s packet;

int setchan(int fd, int chan)
{
  int ret = ioctl(fd, MAC854IOCSCHAN, (unsigned long)chan );
  if (ret<0)
    {
      printf("MAC854IOCSCHAN failed\n");
    }
  return ret;
}

int setcca(int fd, struct ieee802154_cca_s *cca)
{
  int ret = ioctl(fd, MAC854IOCSCCA, (unsigned long)cca );
  if (ret<0)
    {
      printf("MAC854IOCSCCA failed\n");
    }
  return ret;
}

int getcca(int fd, struct ieee802154_cca_s *cca)
{
  int ret = ioctl(fd, MAC854IOCGCCA, (unsigned long)cca );
  if (ret<0)
    {
      printf("MAC854IOCGCCA failed\n");
    }
  return ret;
}

int scan(int fd)
{
  int chan,scan,oldchan;
  int ret = OK;
  uint8_t energy;
  uint8_t levels[16];

  /* save original channel */

  ret = ioctl(fd, MAC854IOCGCHAN, (unsigned long)&oldchan);
  if (ret)
    {
      printf("MAC854IOCGCHAN failed\n");
      return ret;
    }

  /* do scan */
  
  printf("Scanning channels ");
  memset(levels, 0, 16);
  for(scan=0;scan<256;scan++)
    {
      printf("."); fflush(stdout);
      for (chan=0; chan<16; chan++)
        {
          ret = setchan(fd, chan+11);
          if (ret<0)
            {
              printf("Device is not an IEEE 802.15.4 interface!\n");
              return ret;
            }

          ret = ioctl(fd, MAC854IOCGED, (unsigned long)&energy);
          if (ret<0)
            {
              printf("Device is not an IEEE 802.15.4 interface!\n");
              return ret;
            }
          if(energy > levels[chan])
            {
              levels[chan] = energy;
            }
        }
    }
  printf("\n");
  for (chan=0;chan < 16; chan++)
    {
      printf("%2d : [%3d] ",chan+11, levels[chan]);
      energy = levels[chan] >> 3;
      while(energy-- > 0) printf("#");
      printf("\n");
    }

  /* restore original channel */

  ret = setchan(fd, oldchan);

  return ret;
}

/****************************************************************************
 * Name : status
 *
 * Description :
 *   Show device status
 ****************************************************************************/

static int status(int fd)
{
  int ret,i;
  uint8_t panid[2], saddr[2], eaddr[8];
  int promisc, chan;
  struct ieee802154_cca_s cca;

  /* Get information */

  ret = ioctl(fd, MAC854IOCGPANID, (unsigned long)&panid);
  if (ret)
    {
      printf("MAC854IOCGPANID failed\n");
      return ret;
    }
  ret = ioctl(fd, MAC854IOCGCHAN, (unsigned long)&chan);
  if (ret)
    {
      printf("MAC854IOCGCHAN failed\n");
      return ret;
    }

  ret = ioctl(fd, MAC854IOCGSADDR, (unsigned long)&saddr);
  if (ret)
    {
      printf("MAC854IOCGSADDR failed\n");
      return ret;
    }
  ret = ioctl(fd, MAC854IOCGEADDR, (unsigned long)&eaddr);
  if (ret)
    {
      printf("MAC854IOCGEADR failed\n");
      return ret;
    }
  ret = ioctl(fd, MAC854IOCGPROMISC, (unsigned long)&promisc);
  if (ret)
    {
      printf("MAC854IOCGPROMISC failed\n");
      return ret;
    }
  ret = ioctl(fd, MAC854IOCGCCA, (unsigned long)&cca);
  if (ret)
    {
      printf("MAC854IOCGCCA failed\n");
      return ret;
    }
#if 0
  ret = ioctl(fd, MAC854IOCGORDER, (unsigned long)&order);
  if (ret)
    {
      printf("MAC854IOCGORDER failed\n");
      return ret;
    }
#endif
  /* Display */

  printf("PANID %02X%02X CHAN %2d (%d MHz)\nSADDR %02X%02X EADDR ", 
          panid[0], panid[1], chan, 2350+(5*chan), saddr[0], saddr[1]);
  for (i=0; i<8; i++)
    {
      printf("%02X", eaddr[i]);
    }
  printf("\nCCA: ");
  if (cca.use_ed)
    {
      printf("ED (%d) ", cca.edth);
    }
  if (cca.use_cs)
    {
      printf("CS (%d)", cca.csth);
    }
  printf("\nPromisc:%s\n", promisc?"Yes":"No");
  return 0;
}

/****************************************************************************
 * Name : display
 *
 * Description :
 *   Display a single packet
 ****************************************************************************/

static int display(FAR struct ieee802154_packet_s *pack)
{
  printf("Packet len=%d\n", pack->len);
  return 0;
}

/****************************************************************************
 * Name : sniff
 *
 * Description :
 *   Listen for all packets with a valid CRC on a given channel.
 ****************************************************************************/

static int sniff(int fd, int chan)
{
  int ret;
  ret = setchan(fd, chan);
  if (ret<0)
    {
      return ret;
    }
#if 0
  ret = ioctl(fd, MAC854IOCSPROMISC, TRUE);
  if (ret<0)
    {
      printf("Device is not an IEEE 802.15.4 interface!\n");
      return ret;
    }
#endif
  printf("Listening on channel %d\n",chan);
  while (1)
    {
      ret = read(fd, &packet, sizeof(struct ieee802154_packet_s));
      if(ret < 0)
        {
          if (errno == EAGAIN)
            {
            usleep(1000);
            continue;
            }
          else
            {
              printf("read: errno=%d\n",errno);
              break;
            }
        } 

      /* Display packet */
      display(&packet);
    }
  
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * usage
 ****************************************************************************/

int usage(void)
{
  printf("i8 <device> <op> <arg>\n"
         "  scan\n"
         "  dump\n"
         "  stat\n"
         "  chan <ch>\n"
         "  edth <off|rssi>\n"
         "  csth <off|corr>\n"
         );
  return ERROR;
}

/****************************************************************************
 * ieeedump_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int i8_main(int argc, char *argv[])
#endif
{
  int fd;
  int ret = OK;
  unsigned long arg;
  struct ieee802154_cca_s cca;

  printf("IEEE packet sniffer/dumper\n");
  if (argc<3)
    {
      return usage();
    }

  if (argc==4)
    {
    arg = atol(argv[3]);
    }
  /* open device */

  fd = open(argv[1], O_RDWR);
  if (fd<0)
    {
      printf("cannot open %s, errno=%d\n", argv[1], errno);
      ret = errno;
      goto exit;
    }

  /* get mode */
  if (!strcmp(argv[2], "scan"))
    {
    ret = scan(fd);
    }
  else if (!strcmp(argv[2], "dump"))
    {
    ret = ioctl(fd, 1000, 0);
    }
  else if (!strcmp(argv[2], "stat"))
    {
    ret = status(fd);
    }
  else if (!strcmp(argv[2], "chan"))
    {
      if(argc != 4)
        {
        ret = usage();
        }
      ret = setchan(fd, arg);
    }
  else if (!strcmp(argv[2], "edth"))
    {
      if(argc != 4)
        {
        goto usage;
        }
      ret = getcca(fd, &cca);
      if(!strcmp("off",argv[3]))
        {
          cca.use_ed = 0;
        }
      else
        {
          cca.use_ed = 1;
          cca.edth   = arg;
        }
      ret = setcca(fd, &cca);
    }
  else if (!strcmp(argv[2], "csth"))
    {
      if(argc != 4)
        {
        goto usage;
        }
      ret = getcca(fd, &cca);
      if(!strcmp("off",argv[3]))
        {
          cca.use_cs = 0;
        }
      else
        {
          cca.use_cs = 1;
          cca.csth   = arg;
        }
      ret = setcca(fd, &cca);
    }
  else if (!strcmp(argv[2], "snif"))
    {
    ret = sniff(fd,arg);
    }
  else
    {
usage:
    ret = usage();
    }
 
  close(fd);
exit:
  return ret;
}

