/****************************************************************************
 * examples/ieeedump/sniffer_main.c
 * IEEE 802.15.4 Packet Sniffer/Dumper
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

int scan(int fd)
{
  int chan;
  int ret = OK;
  int energy;

  printf("Scanning channels...\n");
  for (chan=11; chan<26; chan++)
    {
      printf("%02X : "); fflush(stdout);
      ret = ioctl(fd, MAC854IOCSCHAN, (unsigned long)chan);
      if (ret<0)
        {
          printf("Device is not an IEEE 802.15.4 interface!\n");
          break;
        }
      ret = ioctl(fd, MAC854IOCGED, (unsigned long)&energy);
      if (ret<0)
        {
          printf("Device is not an IEEE 802.15.4 interface!\n");
          break;
        }
      energy >>= 3;
      while(energy-- > 0) printf("#");
      printf("\n");
    }
  return ret;
}

int sniff(int fd, int chan)
{
  int ret;
  ret = ioctl(fd, MAC854IOCSCHAN, chan);
  if (ret<0)
    {
      printf("Device is not an IEEE 802.15.4 interface!\n");
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int usage(void)
{
  printf("snif8 <device> scan|<chan>\n"
         );
  return ERROR;
}

/****************************************************************************
 * ieeedump_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int snif8_main(int argc, char *argv[])
#endif
{
  int fd;
  int ret = OK;

  printf("IEEE packet sniffer/dumper\n");
  if (argc<3)
    {
      return usage();
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
  else
    {
    ret = monitor(fd,atoi(argv[2]));
    }
 
exit_close:
  close(fd);
exit:
  return ret;
}

