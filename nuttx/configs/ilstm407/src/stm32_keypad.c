/****************************************************************************
 * configs/ilstm407/src/stm32_keypad.c
 *
 *   Copyright (C) 2011-2014 Gregory Nutt. All rights reserved.
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

#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <arch/board/board.h>
#include <nuttx/gpio/keypad.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "stm32.h"
#include "stm32f407.h"

#ifdef CONFIG_KEYPAD

/****************************************************************************
 * Definitions
 ****************************************************************************/
 #define KEYPAD_TEST_KEY_TICKS MSEC2TICK(100)  

struct stm32_dev_s
{
uint8_t 					keycode;
struct work_s       work;         /* For cornercase error handling by the worker thread */
};

static int stm32_keypad_setup(FAR struct keypad_dev_s *dev);
static int stm32_keypad_shutdown(FAR struct keypad_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_dev_s stm32_keypad_priv = {0};

const uint8_t encoding[] = {0xd7, 0xee, 0xde, 0xbe, 0xed, 0xdd, 0xbd, 0xeb, 0xdb, 0xbb, 0xe7, 0xb7, 0x7e, 0x7d, 0x7b, 0x77};

static const struct keypad_ops_s stm32_keypad_ops =
{
  .setup    = stm32_keypad_setup,
  .shutdown = stm32_keypad_shutdown,
};

static struct keypad_dev_s stm32_keypad_dev =
{
  .ops = &stm32_keypad_ops,
  .priv = &stm32_keypad_priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_keypad_abcd_handler(int irq, FAR void *context);
static int stm32_keypad_123_handler(int irq, FAR void *context);
 
static void stm32_keypad_test_key_down_with_interrupt(void)
{
  stm32_configgpio(GPIO_KEY_A_OUT);
  stm32_configgpio(GPIO_KEY_B_OUT);
  stm32_configgpio(GPIO_KEY_C_OUT);
  stm32_configgpio(GPIO_KEY_D_OUT);
	
  stm32_gpiosetevent(GPIO_KEY_1_IN, false, true, false, stm32_keypad_123_handler);
  stm32_gpiosetevent(GPIO_KEY_2_IN, false, true, false, stm32_keypad_123_handler);
  stm32_gpiosetevent(GPIO_KEY_3_IN, false, true, false, stm32_keypad_123_handler);	
}

static void stm32_keypad_test_key_down_without_interrupt(void)
{
  stm32_configgpio(GPIO_KEY_A_OUT);
  stm32_configgpio(GPIO_KEY_B_OUT);
  stm32_configgpio(GPIO_KEY_C_OUT);
  stm32_configgpio(GPIO_KEY_D_OUT);
	
  stm32_gpiosetevent(GPIO_KEY_1_IN, false, true, false, NULL);
  stm32_gpiosetevent(GPIO_KEY_2_IN, false, true, false, NULL);
  stm32_gpiosetevent(GPIO_KEY_3_IN, false, true, false, NULL);	
}

static void stm32_keypad_test_key_up_with_interrupt(void)
{
  stm32_gpiosetevent(GPIO_KEY_A_IN, true, false, false, stm32_keypad_abcd_handler);
  stm32_gpiosetevent(GPIO_KEY_B_IN, true, false, false, stm32_keypad_abcd_handler);
  stm32_gpiosetevent(GPIO_KEY_C_IN, true, false, false, stm32_keypad_abcd_handler);	
  stm32_gpiosetevent(GPIO_KEY_D_IN, true, false, false, stm32_keypad_abcd_handler);	

  stm32_configgpio(GPIO_KEY_1_OUT);
  stm32_configgpio(GPIO_KEY_2_OUT);
  stm32_configgpio(GPIO_KEY_3_OUT);
}

static void stm32_keypad_test_key_up_without_interrupt(void)
{
  stm32_gpiosetevent(GPIO_KEY_A_IN, false, true, false, NULL);
  stm32_gpiosetevent(GPIO_KEY_B_IN, false, true, false, NULL);
  stm32_gpiosetevent(GPIO_KEY_C_IN, false, true, false, NULL);	
  stm32_gpiosetevent(GPIO_KEY_D_IN, false, true, false, NULL);	

  stm32_configgpio(GPIO_KEY_1_OUT);
  stm32_configgpio(GPIO_KEY_2_OUT);
  stm32_configgpio(GPIO_KEY_3_OUT);
}


static void stm32_keypad_delay_test_key(FAR void *arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)arg;
  bool keya_in, keyb_in, keyc_in, keyd_in;
  bool key1_in, key2_in, key3_in;
  uint8_t scan1, scan2;
	
  key1_in = stm32_gpioread(GPIO_KEY_1_IN);
  key2_in = stm32_gpioread(GPIO_KEY_2_IN);
  key3_in = stm32_gpioread(GPIO_KEY_3_IN);
	
  if (key1_in && key2_in && key3_in)
    {
      stm32_keypad_test_key_down_with_interrupt();
	   return;
	 }

  scan1 = 0xf0 & ~((!key1_in ? 1 << 4 : 0) | (!key2_in ? 1 << 5 : 0) | (!key3_in ? 1 << 6 : 0));

  stm32_keypad_test_key_up_without_interrupt();

  keya_in = stm32_gpioread(GPIO_KEY_A_IN);
  keyb_in = stm32_gpioread(GPIO_KEY_B_IN);
  keyc_in = stm32_gpioread(GPIO_KEY_C_IN);
  keyd_in = stm32_gpioread(GPIO_KEY_D_IN);
	
  scan2 = 0x0f & ~((!keya_in ? 1 << 0 : 0) | (!keyb_in ? 1 << 1 : 0) | (!keyc_in ? 1 << 2 : 0) | (!keyd_in ? 1 << 3 : 0));

  priv->keycode = scan1 | scan2;
	 
  stm32_keypad_test_key_up_with_interrupt();
}


static int stm32_keypad_abcd_handler(int irq, FAR void *context)
{
  struct keypad_dev_s *dev = &stm32_keypad_dev;
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->priv;
  uint8_t i;
	
  for (i = 0; i <= 11; i++) 
    {
      if (priv->keycode == encoding[i])
	    {
	       keypad_putbuffer(dev, 0x30 + i);
			break;
        }
    }

  keypad_notify(dev);
	
  stm32_keypad_test_key_down_with_interrupt();

  return OK;
}

static int stm32_keypad_123_handler(int irq, FAR void *context)
{
  stm32_keypad_test_key_down_without_interrupt();
  work_queue(HPWORK, &stm32_keypad_priv.work, stm32_keypad_delay_test_key, &stm32_keypad_priv, KEYPAD_TEST_KEY_TICKS);
  return OK;
}

static int stm32_keypad_setup(FAR struct keypad_dev_s *dev)
{
  stm32_keypad_test_key_down_with_interrupt();

  return OK;
}

static int stm32_keypad_shutdown(FAR struct keypad_dev_s *dev)
{
  stm32_unconfiggpio(GPIO_KEY_A_OUT);
  stm32_unconfiggpio(GPIO_KEY_B_OUT);
  stm32_unconfiggpio(GPIO_KEY_C_OUT);
  stm32_unconfiggpio(GPIO_KEY_D_OUT);
  stm32_unconfiggpio(GPIO_KEY_1_OUT);
  stm32_unconfiggpio(GPIO_KEY_2_OUT);
  stm32_unconfiggpio(GPIO_KEY_3_OUT);

  return OK;
}

void stm32_keypad_initialize(void)
{
  int ret;

  /* Register the keypad driver at "/dev/keypad" */
  ret = keypad_register(CONFIG_KEYPAD_DEVNAME, &stm32_keypad_dev);
  if (ret < 0)
    {
      keypaddbg("keypad_register failed: %d\n", ret);
    }			
}

#endif /* CONFIG_KEYPAD */


