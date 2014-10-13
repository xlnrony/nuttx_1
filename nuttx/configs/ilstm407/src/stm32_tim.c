/************************************************************************************
 * configs/ilstm407/src/stm32_tim.c
 * 
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/analog/adc.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"

#include "stm32.h"
#include "stm32f4discovery.h"

#ifdef CONFIG_ADC

/************************************************************************************
 * Definitions
 ************************************************************************************/


/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: tim_devinit
 *
 * Description:
 *   All STM32 architectures must provide the following interface to work with
 *   examples/adc.
 *
 ************************************************************************************/

int tim_devinit(void)
{
  struct adc_dev_s *tim;
  int ret;

#ifdef CONFIG_STM32_TIM3
  /* Call stm32_cap_init() to get an instance of the CAP interface */

  tim = stm32_cap_init(3, 1); 
  if (tim == NULL)
    {
      adbg("ERROR: Failed to get TIM interface\n");
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/tim3in0" */

  ret = adc_register(CONFIG_TIM_DEVNAME, tim);
  if (ret < 0)
    {
      adbg("adc_register failed: %d\n", ret);
      return ret;
    }
	
  return OK;	
#else
  return -ENOSYS;
#endif
}

#endif /* CONFIG_ADC */
