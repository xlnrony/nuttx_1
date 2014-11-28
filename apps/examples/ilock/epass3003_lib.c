/****************************************************************************
 * examples/ilock/epass3003_lib.c
 *
 *   Copyright (C) 2011, 2013-2014 xlnrony. All rights reserved.
 *   Author: xlnrony <xlnrony@gmail.com>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#include <sys/types.h>
#include <sys/stat.h>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/usb/usbhost.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Sanity checking */

#ifndef CONFIG_USBHOST
#  error "CONFIG_USBHOST is not defined"
#endif

#ifdef CONFIG_USBHOST_INT_DISABLE
#  error "Interrupt endpoints are disabled (CONFIG_USBHOST_INT_DISABLE)"
#endif

#ifndef CONFIG_NFILE_DESCRIPTORS
#  error "CONFIG_NFILE_DESCRIPTORS > 0 needed"
#endif

#ifndef CONFIG_USBHOST_EPASS3003
#  error "CONFIG_USBHOST_EPASS3003 is not defined"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct usbhid_epass3003report_s
{
//  uint8_t id;

  uint8_t pktsizeh;   //  数据包长度-高字节
  uint8_t pktsizel;   //  数据包长度-低字节

  uint8_t blkoffseth;  //  数据块偏移-高字节
  uint8_t blkoffsetl;  //  数据块偏移-低字节

  uint8_t blksizeh;	//  数据块长度-高字节
  uint8_t blksizel;	//  数据块长度-低字节

  uint8_t buffer[58];	//  存放数据块。
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ilock_main
 ****************************************************************************/

int epass3003_transmit_apdu(int fd, FAR uint8_t *txbuf, size_t txpktlen, FAR uint8_t *rxbuf, FAR size_t *rxlen)
{
  struct usbhid_epass3003report_s rpt = {0};

  int txpktsize = txpktlen;
  int txoffset = 0;
  int txsize = 0;
  int ret;

  int rxpktsize = 0;
  int rxoffset = 0;
  int rxsize = 0;

  while(txpktlen)
    {
      //-------------------------------------------------------------------------------
      //  prepare a data package
      txsize = txpktlen > 58 ? 58 : txpktlen;
      if(0 == txsize)
        {
          break;
        }
      rpt.pktsizeh = txpktsize >> 8;
      rpt.pktsizel = txpktsize;
      rpt.blkoffseth= txoffset >> 8;
      rpt.blkoffsetl = txoffset;
      rpt.blksizeh = txsize >> 8;
      rpt.blksizel = txsize;
      memmove(rpt.buffer, txbuf, txsize)   ;
      txoffset += txsize;
      txbuf += txsize;
      txpktlen -= txsize;
      //-------------------------------------------------------------------------------
      //  send this package
      ret = write(fd, &rpt, sizeof(struct usbhid_epass3003report_s));
      if (ret < 0)
        {
          return ret;
        }
    }
  while(1)
    {
      memset(&rpt, 0, sizeof(rpt));

      ret = read(fd, &rpt, sizeof(struct usbhid_epass3003report_s));
      if(ret < 0)
        {
          return ret;
        }
      else
        {
          if(0 == rpt.pktsizeh && 0 == rpt.pktsizel)
            {
              usleep(1000);
            }
          else
            {
              rxpktsize = (rpt.pktsizeh << 8) + rpt.pktsizel;
              rxoffset = (rpt.blkoffseth << 8) +  rpt.blkoffsetl;
              rxsize = (rpt.blksizeh << 8) + rpt.blksizel;
              memmove(rxbuf + rxoffset, rpt.buffer, rxsize);
              if(rxoffset + rxsize == rxpktsize)
                {
                  *rxlen = rxpktsize;
                  break;
                }
            }
        }
    }

  if(rxpktsize == 2)
    {
      if(rxbuf[0] == 0x61)
        {
          uint8_t getresp[32] =
          {
            0x00,0xc0,0x00,0x00,0x00
          };
          getresp[4] = rxbuf[1];
          return epass3003_transmit_apdu(fd, getresp, 5, rxbuf, rxlen);
        }
    }
  return 0;
}
