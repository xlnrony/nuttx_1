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

#ifdef CONFIG_LED

/****************************************************************************
 * Definitions
 ****************************************************************************/
struct stm32_dev_s
{
  uint32_t 				red_pinset;
  uint32_t 				green_pinset;
  uint32_t 				blue_pinset;	
};

static int stm32_led_setup(FAR struct ind_dev_s *dev);
static int stm32_led_shutdown(FAR struct ind_dev_s *dev);
static void stm32_led_ioctl(FAR struct ind_dev_s *dev, uint8_t color);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ind_ops_s stm32_led_ops =
{
  .setup    = stm32_led_setup,
  .shutdown = stm32_led_shutdown,
  .ioctl    = stm32_led_ioctl
};

///////////////////////////////////////////////////////////////////////////////////////////
static struct stm32_dev_s stm32_led_priv_led1 =
{
  .red_pinset		= GPIO_LED1_R,
  .green_pinset	= GPIO_LED1_G,
  .blue_pinset		= GPIO_LED1_B,
};

static struct led_dev_s stm32_led_dev_led1 =
{
  .ops = &stm32_led_ops,
  .priv = &stm32_led_priv_led1,
};
///////////////////////////////////////////////////////////////////////////////////////////
static struct stm32_dev_s stm32_led_priv_led2 =
{
  .red_pinset		= GPIO_LED2_R,
  .green_pinset	= GPIO_LED2_G,
  .blue_pinset		= GPIO_LED2_B,
};

static struct led_dev_s stm32_led_dev_led2 =
{
  .ops = &stm32_led_ops,
  .priv = &stm32_led_priv_led2,
};

///////////////////////////////////////////////////////////////////////////////////////////
static struct stm32_dev_s stm32_led_priv_led3 =
{
  .red_pinset		= GPIO_LED3_R,
  .green_pinset	= GPIO_LED3_G,
  .blue_pinset		= GPIO_LED3_B,
};

static struct led_dev_s stm32_led_dev_led3 =
{
  .ops = &stm32_led_ops,
  .priv = &stm32_led_priv_led3,
};
///////////////////////////////////////////////////////////////////////////////////////////


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_led_setup(FAR struct ind_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->priv;
  int                         ret;
	
  ret = stm32_configgpio(priv->red_pinset);
  if (ret == OK)
  	{
     ret = stm32_configgpio(priv->green_pinset);
	  if (ret == OK)
	    {
          ret = stm32_configgpio(priv->blue_pinset);
		   if (ret < 0)
		   	{
				leddbg("stm32_led_setup: stm32_configgpio(blue_pinset)failed: %d\n", ret);
		   	}
	  	}			
	  else
	  	{
         leddbg("stm32_led_setup: stm32_configgpio(green_pinset) failed: %d\n", ret);
	  	}
  	}
  else
  	{
      leddbg("stm32_led_setup: stm32_configgpio(red_pinset) failed: %d\n", ret);
  	}
  return ret;
}

static int stm32_led_shutdown(FAR struct ind_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->priv;
	
  stm32_unconfiggpio(priv->red_pinset);
  stm32_unconfiggpio(priv->green_pinset);  
  stm32_unconfiggpio(priv->blue_pinset);

  return OK;
}

static void stm32_led_ioctl(FAR struct ind_dev_s *dev, uint8_t color)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->priv;

  switch(color)
    {
      case LED_RED:
	    stm32_gpiowrite(priv->red_pinset, false);
	    stm32_gpiowrite(priv->green_pinset, true);
	    stm32_gpiowrite(priv->blue_pinset, true);
	    break;
	  case LED_GREEN:
	    stm32_gpiowrite(priv->red_pinset, true);
	    stm32_gpiowrite(priv->green_pinset, false);
	    stm32_gpiowrite(priv->blue_pinset, true);
	    break;
	  case LED_BLUE:
	    stm32_gpiowrite(priv->red_pinset, true);
	    stm32_gpiowrite(priv->green_pinset, true);
	    stm32_gpiowrite(priv->blue_pinset, false);
	    break;
	  case LED_NONE:
	    stm32_gpiowrite(priv->red_pinset, true);
	    stm32_gpiowrite(priv->green_pinset, true);
	    stm32_gpiowrite(priv->blue_pinset, true);
	    break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stm32_led_initialize(void)
{
  int ret;
//////////////////////////////////////////////////////////////////////////////////////////////////
  /* Register the led driver at "/dev/led1" */
  ret = ind_register(CONFIG_LED1_DEVNAME, &stm32_led_dev_led1);
  if (ret < 0)
    {
      leddbg("led_register failed: %d\n", ret);
    }			
//////////////////////////////////////////////////////////////////////////////////////////////////
  /* Register the led driver at "/dev/led2" */
  ret = ind_register(CONFIG_LED2_DEVNAME, &stm32_led_dev_led2);
  if (ret < 0)
    {
      leddbg("gpio_register failed: %d\n", ret);
    }			
/////////////////////////////////////////////////////////////////////////////////////////////////
  /* Register the led driver at "/dev/led3" */
  ret = ind_register(CONFIG_LED3_DEVNAME, &stm32_led_dev_led3);
  if (ret < 0)
    {
      leddbg("gpio_register failed: %d\n", ret);
    }				
}

#endif /* CONFIG_GPIO */
