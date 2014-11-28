/************************************************************************************
 * configs/stm3240g-eval/src/up_adc.c
 * arch/arm/src/board/up_adc.c
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
#include "stm32f407.h"

#ifdef CONFIG_ADC

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Configuration ************************************************************/
/* Up to 3 ADC interfaces are supported */

#if STM32_NADC < 3
#  undef CONFIG_STM32_ADC3
#endif

#if STM32_NADC < 2
#  undef CONFIG_STM32_ADC2
#endif

#if STM32_NADC < 1
#  undef CONFIG_STM32_ADC1
#endif

#if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2) || defined(CONFIG_STM32_ADC3)
#ifndef CONFIG_STM32_ADC1
#  warning "Channel information only available for ADC1"
#endif

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
 * Name: stm32_adc_initialize
 *
 * Description:
 *   Called at application startup time to initialize the ADC functionality.
 *
 ************************************************************************************/

void stm32_adc_initialize(void)
{
  struct adc_dev_s *adc;
  int ret;

  /* Check if we have already initialized */

//////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef CONFIG_STM32_ADC1
  /* Call stm32_adcinitialize() to get an instance of the ADC interface */

  adc = stm32_adcinitialize(1);
  if (adc == NULL)
    {
      adbg("ERROR: Failed to get ADC interface\n");
    }
  else
    {
      /* Register the ADC driver at "/dev/adc1" */

      ret = adc_register(CONFIG_ADC1_DEVNAME, adc);
      if (ret < 0)
        {
          adbg("adc_register failed: %d\n", ret);
        }
    }

#endif
//////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef CONFIG_STM32_ADC2
  /* Call stm32_adcinitialize() to get an instance of the ADC interface */

  adc = stm32_adcinitialize(1);
  if (adc == NULL)
    {
      adbg("ERROR: Failed to get ADC interface\n");
    }
  else
    {
      /* Register the ADC driver at "/dev/adc2" */

      ret = adc_register(CONFIG_ADC2_DEVNAME, adc);
      if (ret < 0)
        {
          adbg("adc_register failed: %d\n", ret);
        }
    }
#endif
//////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef CONFIG_STM32_ADC3
  /* Call stm32_adcinitialize() to get an instance of the ADC interface */

  adc = stm32_adcinitialize(3);
  if (adc == NULL)
    {
      adbg("ERROR: Failed to get ADC interface\n");
    }
  else
    {
      /* Register the ADC driver at "/dev/adc0" */

      ret = adc_register(CONFIG_ADC3_DEVNAME, adc);
      if (ret < 0)
        {
          adbg("adc_register failed: %d\n", ret);
        }
    }
#endif
//////////////////////////////////////////////////////////////////////////////////////////////////
}

#endif /* CONFIG_STM32_ADC1 || CONFIG_STM32_ADC2 || CONFIG_STM32_ADC3 */
#endif /* CONFIG_ADC */
