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
  	
  struct work_s     work;        
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void stm32_keypad_keydown_work(FAR void *arg)
{

}

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
      case GPIOC_DELAY:
		 if ((priv->pinset & GPIO_MODE_MASK) == GPIO_OUTPUT)
		   {
		 	  stm32_gpiowrite(priv->pinset,(bool)arg); 		
		 	}
		 else
		 	{
	         ret = -EACCES;
		 	}
        break;
      case GPIOC_TWINKLE:
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
static struct stm32_dev_s stm32_gpio_priv_led1_b =
{
  .pinset         = GPIO_LED1_B,
};

static struct gpio_dev_s stm32_gpio_dev_led1_b =
{
  .ops = &stm32_gpio_ops,
  .priv = &stm32_gpio_priv_led1_b,
};

static struct stm32_dev_s stm32_gpio_priv_led1_g =
{
  .pinset         = GPIO_LED1_G,
};

static struct gpio_dev_s stm32_gpio_dev_led1_g =
{
  .ops = &stm32_gpio_ops,
  .priv = &stm32_gpio_priv_led1_g,
};

static struct stm32_dev_s stm32_gpio_priv_led1_r =
{
  .pinset         = GPIO_LED1_R,
};

static struct gpio_dev_s stm32_gpio_dev_led1_r =
{
  .ops = &stm32_gpio_ops,
  .priv = &stm32_gpio_priv_led1_r,
};
///////////////////////////////////////////////////////////////////////////////////////////
static struct stm32_dev_s stm32_gpio_priv_led2_b =
{
  .pinset         = GPIO_LED2_B,
};

static struct gpio_dev_s stm32_gpio_dev_led2_b =
{
  .ops = &stm32_gpio_ops,
  .priv = &stm32_gpio_priv_led2_b,
};

static struct stm32_dev_s stm32_gpio_priv_led2_g =
{
  .pinset         = GPIO_LED2_G,
};

static struct gpio_dev_s stm32_gpio_dev_led2_g =
{
  .ops = &stm32_gpio_ops,
  .priv = &stm32_gpio_priv_led2_g,
};

static struct stm32_dev_s stm32_gpio_priv_led2_r =
{
  .pinset         = GPIO_LED2_R,
};

static struct gpio_dev_s stm32_gpio_dev_led2_r =
{
  .ops = &stm32_gpio_ops,
  .priv = &stm32_gpio_priv_led2_r,
};
///////////////////////////////////////////////////////////////////////////////////////////
static struct stm32_dev_s stm32_gpio_priv_led3_b =
{
  .pinset         = GPIO_LED3_B,
};

static struct gpio_dev_s stm32_gpio_dev_led3_b =
{
  .ops = &stm32_gpio_ops,
  .priv = &stm32_gpio_priv_led3_b,
};

static struct stm32_dev_s stm32_gpio_priv_led3_g =
{
  .pinset         = GPIO_LED3_G,
};

static struct gpio_dev_s stm32_gpio_dev_led3_g =
{
  .ops = &stm32_gpio_ops,
  .priv = &stm32_gpio_priv_led3_g,
};

static struct stm32_dev_s stm32_gpio_priv_led3_r =
{
  .pinset         = GPIO_LED3_R,
};

static struct gpio_dev_s stm32_gpio_dev_led3_r =
{
  .ops = &stm32_gpio_ops,
  .priv= &stm32_gpio_priv_led3_r,
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

static struct stm32_dev_s stm32_gpio_priv_magnet_gnd =
{
  .pinset         = GPIO_MAGNET_GND,
};

static struct gpio_dev_s stm32_gpio_dev_magnet_gnd =
{
  .ops = &stm32_gpio_ops,
  .priv= &stm32_gpio_priv_magnet_gnd,
};
///////////////////////////////////////////////////////////////////////////////////////////
static struct stm32_dev_s stm32_gpio_priv_buzzer =
{
  .pinset         = GPIO_BUZZER,
};

static struct gpio_dev_s stm32_gpio_dev_buzzer =
{
  .ops = &stm32_gpio_ops,
  .priv= &stm32_gpio_priv_buzzer,
};

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
//////////////////////////////////////////////////////////////////////////////////////////////////
  /* Register the GPIO driver at "/dev/led1b" */
//  stm32_configgpio(stm32_gpio_priv_led1_b.pinset);	
  ret = gpio_register(CONFIG_LED1_BLUE_DEVNAME, &stm32_gpio_dev_led1_b);
  if (ret < 0)
    {
      gpiodbg("gpio_register failed: %d\n", ret);
    }			
/////////////////////////////////////////////////////////////////////////////////////////////////
  /* Register the GPIO driver at "/dev/led1g" */
//  stm32_configgpio(stm32_gpio_priv_led1_g.pinset);	
  ret = gpio_register(CONFIG_LED1_GREEN_DEVNAME, &stm32_gpio_dev_led1_g);
  if (ret < 0)
    {
      gpiodbg("gpio_register failed: %d\n", ret);
    }			
//////////////////////////////////////////////////////////////////////////////////////////////////
  /* Register the GPIO driver at "/dev/led1r" */
//  stm32_configgpio(stm32_gpio_priv_led1_r.pinset);	
  ret = gpio_register(CONFIG_LED1_RED_DEVNAME, &stm32_gpio_dev_led1_r);
  if (ret < 0)
    {
      gpiodbg("gpio_register failed: %d\n", ret);
    }			
//////////////////////////////////////////////////////////////////////////////////////////////////
  /* Register the GPIO driver at "/dev/led2b" */
//  stm32_configgpio(stm32_gpio_priv_led2_b.pinset);	
  ret = gpio_register(CONFIG_LED2_BLUE_DEVNAME, &stm32_gpio_dev_led2_b);
  if (ret < 0)
    {
      gpiodbg("gpio_register failed: %d\n", ret);
    }			
//////////////////////////////////////////////////////////////////////////////////////////////////
  /* Register the GPIO driver at "/dev/led2g" */
//  stm32_configgpio(stm32_gpio_priv_led2_g.pinset);	
  ret = gpio_register(CONFIG_LED2_GREEN_DEVNAME, &stm32_gpio_dev_led2_g);
  if (ret < 0)
    {
      gpiodbg("gpio_register failed: %d\n", ret);
    }			
//////////////////////////////////////////////////////////////////////////////////////////////////
  /* Register the GPIO driver at "/dev/led2r" */
//  stm32_configgpio(stm32_gpio_priv_led2_r.pinset);	
  ret = gpio_register(CONFIG_LED2_RED_DEVNAME, &stm32_gpio_dev_led2_r);
  if (ret < 0)
    {
      gpiodbg("gpio_register failed: %d\n", ret);
    }			
/////////////////////////////////////////////////////////////////////////////////////////////////
  /* Register the GPIO driver at "/dev/led3b" */
//  stm32_configgpio(stm32_gpio_priv_led3_b.pinset);	
  ret = gpio_register(CONFIG_LED3_BLUE_DEVNAME, &stm32_gpio_dev_led3_b);
  if (ret < 0)
    {
      gpiodbg("gpio_register failed: %d\n", ret);
    }			
/////////////////////////////////////////////////////////////////////////////////////////////////
  /* Register the GPIO driver at "/dev/led3g" */
//  stm32_configgpio(stm32_gpio_priv_led3_g.pinset);	
  ret = gpio_register(CONFIG_LED3_GREEN_DEVNAME, &stm32_gpio_dev_led3_g);
  if (ret < 0)
    {
      gpiodbg("gpio_register failed: %d\n", ret);
    }			
/////////////////////////////////////////////////////////////////////////////////////////////////
  /* Register the GPIO driver at "/dev/led3r" */
//  stm32_configgpio(stm32_gpio_priv_led3_r.pinset);	
  ret = gpio_register(CONFIG_LED3_RED_DEVNAME, &stm32_gpio_dev_led3_r);
  if (ret < 0)
    {
      gpiodbg("gpio_register failed: %d\n", ret);
    }		
////////////////////////////////////////////////////////////////////////////////////////////////
  ret = gpio_register(CONFIG_MAGNET_VCC_DEVNAME, &stm32_gpio_dev_magnet_vcc);
  if (ret < 0)
    {
      gpiodbg("gpio_register failed: %d\n", ret);
    }		
////////////////////////////////////////////////////////////////////////////////////////////////
  ret = gpio_register(CONFIG_MAGNET_GND_DEVNAME, &stm32_gpio_dev_magnet_gnd);
  if (ret < 0)
    {
      gpiodbg("gpio_register failed: %d\n", ret);
    }		
////////////////////////////////////////////////////////////////////////////////////////////////
  ret = gpio_register(CONFIG_BUZZER_DEVNAME, &stm32_gpio_dev_buzzer);
  if (ret < 0)
    {
      gpiodbg("gpio_register failed: %d\n", ret);
    }		
////////////////////////////////////////////////////////////////////////////////////////////////	
  ret = gpio_register(CONFIG_CLOSE_SW_DEVNAME, &stm32_gpio_dev_close_sw);
  if (ret < 0)
    {
      gpiodbg("gpio_register failed: %d\n", ret);
    }		
	
}

#endif /* CONFIG_GPIO */
