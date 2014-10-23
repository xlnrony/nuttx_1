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
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "stm32.h"
#include "stm32f4discovery.h"

#ifdef CONFIG_SGPIO

/****************************************************************************
 * Definitions
 ****************************************************************************/
struct stm32_dev_s
{
  uint32_t pinset;
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
  
  gpiodbg("cmd=%d arg=%ld\n", cmd, arg);

  switch (cmd)
    {
      case GPIOC_WRITE:
		 stm32_gpiowrite(priv->pinset,bool(arg)); 		
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
static const struct stm32_dev_s stm32_gpio_priv_led1_b =
{
  .cfgset         = GPIO_LED1_B,
};

static const struct gpio_dev_s stm32_gpio_dev_led1_b =
{
  .ops = &stm32_gpio_ops,
  .priv= &stm32_gpio_priv_led1_b,
};

static const struct stm32_dev_s stm32_gpio_priv_led1_g =
{
  .cfgset         = GPIO_LED1_G,
};

static const struct gpio_dev_s stm32_gpio_dev_led1_g =
{
  .ops = &stm32_gpio_ops,
  .priv= &stm32_gpio_priv_led1_g,
};

static const struct stm32_dev_s stm32_gpio_priv_led1_r =
{
  .cfgset         = GPIO_LED1_R,
};

static const struct gpio_dev_s stm32_gpio_dev_led1_r =
{
  .ops = &stm32_gpio_ops,
  .priv= &stm32_gpio_priv_led1_r,
};
///////////////////////////////////////////////////////////////////////////////////////////
static const struct stm32_dev_s stm32_gpio_priv_led2_b =
{
  .cfgset         = GPIO_LED2_B,
};

static const struct gpio_dev_s stm32_gpio_dev_led2_b =
{
  .ops = &stm32_gpio_ops,
  .priv= &stm32_gpio_priv_led2_b,
};

static const struct stm32_dev_s stm32_gpio_priv_led2_g =
{
  .cfgset         = GPIO_LED2_G,
};

static const struct gpio_dev_s stm32_gpio_dev_led2_g =
{
  .ops = &stm32_gpio_ops,
  .priv= &stm32_gpio_priv_led2_g,
};

static const struct stm32_dev_s stm32_gpio_priv_led2_r =
{
  .cfgset         = GPIO_LED2_R,
};

static const struct gpio_dev_s stm32_gpio_dev_led2_r =
{
  .ops = &stm32_gpio_ops,
  .priv= &stm32_gpio_priv_led2_r,
};
///////////////////////////////////////////////////////////////////////////////////////////
static const struct stm32_dev_s stm32_gpio_priv_led3_b =
{
  .cfgset         = GPIO_LED3_B,
};

static const struct gpio_dev_s stm32_gpio_dev_led3_b =
{
  .ops = &stm32_gpio_ops,
  .priv= &stm32_gpio_priv_led3_b,
};

static const struct stm32_dev_s stm32_gpio_priv_led3_g =
{
  .cfgset         = GPIO_LED3_G,
};

static const struct gpio_dev_s stm32_gpio_dev_led3_g =
{
  .ops = &stm32_gpio_ops,
  .priv= &stm32_gpio_priv_led3_g,
};

static const struct stm32_dev_s stm32_gpio_priv_led3_r =
{
  .cfgset         = GPIO_LED3_R,
};

static const struct gpio_dev_s stm32_gpio_dev_led3_r =
{
  .ops = &stm32_gpio_ops,
  .priv= &stm32_gpio_priv_led3_r,
};
///////////////////////////////////////////////////////////////////////////////////////////


#endif /* CONFIG_SGPIO */
