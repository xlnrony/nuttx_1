/****************************************************************************
 * examples/ilock/gpio_lib.c
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

#include <nuttx/gpio/gpio.h>

#include "gpio_lib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
static int magnet_fd;
static int closesw_fd;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void gpio_write(int fd, bool value)
{
  int ret;
  DEBUGASSERT(fd>0);
  ret = ioctl(fd, GPIOC_WRITE, (unsigned long)value);
  if (ret < 0)
    {
      gpiodbg("gpio_write: GPIOC_WRITE ioctl failed: %d\n", errno);
    }
}

static bool gpio_read(int fd)
{
  int ret;
  bool value = false;

  DEBUGASSERT(fd>0);
  ret = ioctl(fd, GPIOC_READ, (unsigned long)&value);
  if (ret < 0)
    {
      gpiodbg("gpio_write: GPIOC_READ ioctl failed: %d\n", errno);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

inline void magnet_write(bool value)
{
  gpio_write(magnet_fd, value);
}

inline bool closesw_read(void)
{
  return gpio_read(closesw_fd);
}

void gpio_init(void)
{
  magnet_fd = open(CONFIG_MAGNET_VCC_DEVNAME, 0);
  if (magnet_fd < 0)
    {
      gpiodbg("gpio_init: open %s failed: %d\n", CONFIG_MAGNET_VCC_DEVNAME, errno);
    }

  closesw_fd = open(CONFIG_CLOSE_SW_DEVNAME, 0);
  if (closesw_fd < 0)
    {
      gpiodbg("gpio_init: open %s failed: %d\n", CONFIG_CLOSE_SW_DEVNAME, errno);
    }
}

void gpio_deinit(void)
{
  if (magnet_fd > 0)
    {
      close(magnet_fd);
    }
  if (closesw_fd > 0)
    {
      close(closesw_fd);
    }
}


