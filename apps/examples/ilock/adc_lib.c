/****************************************************************************
 * examples/ilock/adc_lib.c
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
#include <nuttx/analog/adc.h>

#include "adc_lib.h"

#if defined(CONFIG_ADC)

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

static int32_t adc_op(const char * path, unsigned long channel)
{
  int32_t ret;
  int fd;
  ssize_t n;
  struct adc_msg_s sample;

  fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      ret = -errno;
      avdbg("adc_op: open %s failed: %d\n", path, ret);
      return ret;
    }

  ret = ioctl(fd, ANIOC_TRIGGER, channel);
  if (ret < 0)
    {
      ret = -errno;
      avdbg("adc_op: ANIOC_TRIGGER ioctl failed: %d\n", ret);
      goto errout;
    }

  n = read(fd, &sample, sizeof(struct adc_msg_s));
  if (n < 0)
    {
      ret = -errno;
      avdbg("adc_op: read failed: %d\n", ret);
    }
  else if (n == 0)
    {
      avdbg("adc_op: No data read, Ignoring\n");
      ret = -EBADMSG;
    }
  else
    {
      DEBUGASSERT(n == sizeof(struct adc_msg_s));
      ret = sample.am_data;
    }

errout:
  close(fd);
  return ret;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

inline int32_t adc_shock_resistor_op(void)
{
  return adc_op(CONFIG_ADC1_DEVNAME, 4);
}

inline int32_t adc_photo_resistor_op(void)
{
  return adc_op(CONFIG_ADC2_DEVNAME, 5);
}

inline int32_t adc_infra_red_op(void)
{
  return adc_op(CONFIG_ADC2_DEVNAME, 6);
}

#endif

