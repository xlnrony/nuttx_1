/****************************************************************************
 * configs/ilstm407/src/stm32_led.c
 *
 *   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/gpio/indicator.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "stm32.h"
#include "stm32f407.h"

#ifdef CONFIG_INDICATOR

/****************************************************************************
 * Definitions
 ****************************************************************************/
struct stm32_dev_s
{
  uint32_t 				pinset;
};

static int stm32_buzzer_setup(FAR struct ind_dev_s *dev);
static int stm32_buzzer_shutdown(FAR struct ind_dev_s *dev);
static void stm32_buzzer_ioctl(FAR struct ind_dev_s *dev, uint8_t color);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ind_ops_s stm32_buzzer_ops =
{
  .setup    = stm32_buzzer_setup,
  .shutdown = stm32_buzzer_shutdown,
  .ioctl    = stm32_buzzer_ioctl
};

static struct stm32_dev_s stm32_buzzer_priv =
{
  .pinset		= GPIO_BUZZER,
};

static struct ind_dev_s stm32_buzzer_dev =
{
  .ops = &stm32_buzzer_ops,
  .priv = &stm32_buzzer_priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_buzzer_setup(FAR struct ind_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->priv;
  int ret;
  ret = stm32_configgpio(priv->pinset);
  if (ret < 0)
    {
      inddbg("stm32_buzzer_setup: stm32_configgpio failed: %d\n", ret);
    }
  return ret;
}

static int stm32_buzzer_shutdown(FAR struct ind_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->priv;

  stm32_unconfiggpio(priv->pinset);

  return OK;
}

static void stm32_buzzer_ioctl(FAR struct ind_dev_s *dev, uint8_t color)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->priv;

  switch(color)
    {
      case IND_RED:
      case IND_GREEN:
      case IND_BLUE:
      case IND_ON:
        stm32_gpiowrite(priv->pinset, false);
        break;
      case IND_NONE:
        stm32_gpiowrite(priv->pinset, true);
        break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stm32_buzzer_initialize(void)
{
  int ret;
//////////////////////////////////////////////////////////////////////////////////////////////////
  ret = ind_register(CONFIG_BUZZER_DEVNAME, &stm32_buzzer_dev);
  if (ret < 0)
    {
      inddbg("stm32_buzzer_initialize: led_register failed: %d\n", ret);
    }
}

#endif /* CONFIG_GPIO */
