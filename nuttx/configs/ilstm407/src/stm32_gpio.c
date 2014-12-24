/****************************************************************************
 * configs/ilstm407/src/stm32_gpio.c
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
#include <nuttx/gpio/gpio.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "stm32.h"
#include "stm32f407.h"

#ifdef CONFIG_GPIO

/****************************************************************************
 * Definitions
 ****************************************************************************/
struct stm32_dev_s
{
  uint32_t 				pinset;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_gpio_setup(FAR struct gpio_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->priv;
  int                         ret;

  ret = stm32_configgpio(priv->pinset);

  return ret;
}

static int stm32_gpio_shutdown(FAR struct gpio_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->priv;
  int                         ret;

  ret = stm32_unconfiggpio(priv->pinset);

  return ret;
}

static int stm32_gpio_ioctl(FAR struct gpio_dev_s *dev, int cmd, unsigned long arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->priv;
  int ret = OK;
  bool *b;

  gpiodbg("cmd=%d arg=%ld\n", cmd, arg);

  switch (cmd)
    {
      case GPIOC_WRITE:
        if ((priv->pinset & GPIO_MODE_MASK) == GPIO_OUTPUT)
          {
            stm32_gpiowrite(priv->pinset,(bool)arg);
          }
        else
          {
            ret = -EACCES;
          }
        break;
      case GPIOC_READ:
        if ((priv->pinset & GPIO_MODE_MASK) == GPIO_INPUT)
          {
            b = (bool *) arg;
            *b = stm32_gpioread(priv->pinset);
          }
        else
          {
            ret = -EACCES;
          }
        break;

      /* Unsupported or invalid command */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static const struct gpio_ops_s stm32_gpio_ops =
{
  .setup    = stm32_gpio_setup,
  .shutdown = stm32_gpio_shutdown,
  .ioctl    = stm32_gpio_ioctl
};
///////////////////////////////////////////////////////////////////////////////////////////
static struct stm32_dev_s stm32_gpio_priv_magnet_vcc =
{
  .pinset         = GPIO_MAGNET_VCC,
};

static struct gpio_dev_s stm32_gpio_dev_magnet_vcc =
{
  .ops = &stm32_gpio_ops,
  .priv= &stm32_gpio_priv_magnet_vcc,
};
///////////////////////////////////////////////////////////////////////////////////////////
static struct stm32_dev_s stm32_gpio_priv_close_sw =
{
  .pinset         = GPIO_CLOSE_SW,
};

static struct gpio_dev_s stm32_gpio_dev_close_sw =
{
  .ops = &stm32_gpio_ops,
  .priv= &stm32_gpio_priv_close_sw,
};
///////////////////////////////////////////////////////////////////////////////////////////

void stm32_gpio_initialize(void)
{
  int ret;
////////////////////////////////////////////////////////////////////////////////////////////////
  ret = gpio_register(CONFIG_MAGNET_VCC_DEVNAME, &stm32_gpio_dev_magnet_vcc);
  if (ret < 0)
    {
      gpiodbg("gpio_register(%s) failed: %d\n", CONFIG_MAGNET_VCC_DEVNAME, ret);
    }
////////////////////////////////////////////////////////////////////////////////////////////////
  ret = gpio_register(CONFIG_CLOSE_SW_DEVNAME, &stm32_gpio_dev_close_sw);
  if (ret < 0)
    {
      gpiodbg("gpio_register(%s) failed: %d\n", CONFIG_CLOSE_SW_DEVNAME, ret);
    }
}

#endif /* CONFIG_GPIO */
