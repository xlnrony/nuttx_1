/****************************************************************************
 * examples/ilock/led_lib.c
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
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>
#include <errno.h>

#include "led_lib.h"

#if defined(CONFIG_INDICATOR)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
static int led1_fd;
static int led2_fd;
static int led3_fd;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void led_op(int fd, int cmd, uint8_t type, uint32_t delay, uint32_t interval)
{
  int ret;
  struct ind_ctl_s indctl;

  indctl.type = type;
  indctl.delay = delay;
  indctl.interval =interval;

  ret = ioctl(fd, cmd, (unsigned long)&indctl);
  if (ret < 0)
    {
      inddbg("led_op: ioctl failed: %d\n", errno);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

inline void led1_op(int cmd, uint8_t type, uint32_t delay, uint32_t interval)
{
  led_op(led1_fd, cmd, type, delay, interval);
}

inline void led2_op(int cmd, uint8_t type, uint32_t delay, uint32_t interval)
{
  led_op(led2_fd, cmd, type, delay, interval);
}

inline void led3_op(int cmd, uint8_t type, uint32_t delay, uint32_t interval)
{
  led_op(led3_fd, cmd, type, delay, interval);
}

int led_init(void)
{
  int ret;
  led1_fd = open(CONFIG_LED1_DEVNAME, 0);
  if (led1_fd < 0)
    {
      ret = -errno;
      inddbg("led_init: open %s failed: %d\n", CONFIG_LED1_DEVNAME, ret);
      return ret;
    }

  led2_fd = open(CONFIG_LED2_DEVNAME, 0);
  if (led2_fd < 0)
    {
      inddbg("led_init: open %s failed: %d\n", CONFIG_LED2_DEVNAME, errno);
      return ret;
    }

  led3_fd = open(CONFIG_LED3_DEVNAME, 0);
  if (led3_fd < 0)
    {
      inddbg("led_init: open %s failed: %d\n", CONFIG_LED3_DEVNAME, errno);
      return ret;
    }

  return OK;
}

void led_deinit(void)
{
  if (led1_fd > 0)
    {
      close(led1_fd);
    }
  if (led2_fd > 0)
    {
      close(led2_fd);
    }
  if (led3_fd > 0)
    {
      close(led3_fd);
    }
}

#endif


