/****************************************************************************
 * examples/ilock/keypad_lib.c
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
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>
#include <errno.h>

#include "led_lib.h"
#include "keypad_lib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

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

int keypad_readln(char *buf, size_t buflen, bool postip)
{
  ssize_t n;
  int fd;
  int ret;
  ssize_t i;

  DEBUGASSERT(buf != NULL && buflen > 0)

//  keypaddbg("keypad_readln: Opening device %s\n", CONFIG_KEYPAD_DEVNAME);
  fd = open(CONFIG_KEYPAD_DEVNAME, O_RDONLY);
  if (fd < 0)
    {
      ret = errno;
      keypaddbg("keypad_readln: open() failed: %d\n", ret);
      return ret;
    }

  /* Loop until there is a read failure */

  i = 0;
  memset(buf, 0, buflen);
  if (postip)
    {
      led1_op(INDC_ALWAYS, IND_RED, UINT32_MAX , 0);
      led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
      led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
    }

  do
    {
      /* Read a buffer of data */

      n = read(fd, buf, 1);
      if (n > 0)
        {
          if (*buf == ';')
            {
              *buf = 0x00;
            }
          else if (*buf == ':' && i > 0)
            {
              i = 0;
              memset(buf, 0, buflen);
              if (postip)
                {
                  led1_op(INDC_ALWAYS, IND_RED, UINT32_MAX, 0);
                  led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                  led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                }
            }
          else
            {
              if (postip)
                {
                  switch(i)
                    {
                      case 0:
                        led1_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                        led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        break;
                      case 1:
                        led1_op(INDC_ALWAYS, IND_BLUE, UINT32_MAX, 0);
                        led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        break;
                      case 2:
                        led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        led2_op(INDC_ALWAYS, IND_RED, UINT32_MAX, 0);
                        led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        break;
                      case 3:
                        led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        led2_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                        led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        break;
                      case 4:
                        led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        led2_op(INDC_ALWAYS, IND_BLUE, UINT32_MAX, 0);
                        led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        break;
                      case 5:
                        led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        led3_op(INDC_ALWAYS, IND_RED, UINT32_MAX, 0);
                        break;
                      case 6:
                        led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        led3_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                        break;
                      case 7:
                        led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        led3_op(INDC_ALWAYS, IND_BLUE, UINT32_MAX, 0);
                        break;
                      case 8:
                        led1_op(INDC_ALWAYS, IND_RED, UINT32_MAX, 0);
                        led2_op(INDC_ALWAYS, IND_RED, UINT32_MAX, 0);
                        led3_op(INDC_ALWAYS, IND_RED, UINT32_MAX, 0);
                        break;
                      case 9:
                        led1_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                        led2_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                        led3_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                        break;
                      case 10:
                        led1_op(INDC_ALWAYS, IND_BLUE, UINT32_MAX, 0);
                        led2_op(INDC_ALWAYS, IND_BLUE, UINT32_MAX, 0);
                        led3_op(INDC_ALWAYS, IND_BLUE, UINT32_MAX, 0);
                        break;
                      default:
                        led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                        break;
                    }
                }
            }
        }
    }
  while (n >= 0 && (n == 1 ? (*buf++ != 0x00 && ++i < buflen) : true));

  if (n<0)
    {
      ret = -errno;
      keypaddbg("keypad_readln: read() failed: %d\n", ret);
    }

  keypaddbg("keypad_readln: Closing device %s\n", CONFIG_KEYPAD_DEVNAME);
  close(fd);

  return ret;
}

