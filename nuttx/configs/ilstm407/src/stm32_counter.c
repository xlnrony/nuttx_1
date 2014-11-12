/****************************************************************************
 * configs/ilstm407/src/stm32_counter.c
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
#include <arch/irq.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/analog/adc.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "ram_vectors.h"
#include "stm32.h"
#include "stm32f407.h"

#ifndef CONFIG_ARCH_HIPRI_INTERRUPT
#  error CONFIG_ARCH_HIPRI_INTERRUPT is required
#endif

#ifndef CONFIG_ARCH_RAMVECTORS
#  error CONFIG_ARCH_RAMVECTORS is required
#endif

#ifndef CONFIG_STM32_TIM6
#  error CONFIG_STM32_TIM6 is required
#endif

#ifndef CONFIG_TIM6_FREQUENCY
#  define CONFIG_TIM6_FREQUENCY 2000
#endif

#ifndef CONFIG_TIM6_PERIOD
#  define CONFIG_TIM6_PERIOD (CONFIG_TIM6_FREQUENCY / 1)
#endif

#ifndef CONFIG_ARCH_IRQPRIO
#  error CONFIG_ARCH_IRQPRIO is required
#endif

#ifdef CONFIG_ADC

/****************************************************************************
 * Definitions
 ****************************************************************************/
struct stm32_dev_s
{
  struct stm32_tim_dev_s * 	timdev;
  up_vector_t 							timhandler;	
};

static void stm32_counter_tim6_handler(void);

static void stm32_counter_reset(FAR struct adc_dev_s *dev);
static int stm32_counter_setup(FAR struct adc_dev_s *dev);
static void stm32_counter_shutdown(FAR struct adc_dev_s *dev);
static void stm32_counter_rxint(FAR struct adc_dev_s *dev, bool enable);
static int stm32_counter_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s stm32_counter_ops =
{
  .ao_reset 			=	stm32_counter_reset,
  .ao_setup    		=	stm32_counter_setup,
  .ao_shutdown 	= 	stm32_counter_shutdown,
  .ao_rxint			= stm32_counter_rxint,
  .ao_ioctl			=	stm32_counter_ioctl
};

static struct stm32_dev_s stm32_counter_priv_magnetlx =
{
  .timhandler			= stm32_counter_tim6_handler
};

static struct adc_dev_s stm32_counter_dev_magnetlx =
{
  .ad_ops = &stm32_counter_ops,
  .ad_priv = &stm32_counter_priv_magnetlx
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*
static void stm32_counter_tim6_work(FAR void *arg)
{
  adc_receive(&stm32_counter_dev_magnetlx, stm32_counter_priv_magnetlx.tim, stm32_counter_priv_magnetlx.count);
}*/

static void stm32_counter_tim6_handler(void)
{
  uint16_t count;

  modifyreg16(STM32_TIM3_BASE + STM32_GTIM_CR1_OFFSET, GTIM_CR1_CEN, 0);	
  STM32_TIM_ACKINT(stm32_counter_priv_magnetlx.timdev, 0);
  count = getreg16(STM32_TIM3_BASE + STM32_GTIM_CNT_OFFSET);
  adc_receive(&stm32_counter_dev_magnetlx, 6, count);	
}

static void stm32_counter_reset(FAR struct adc_dev_s *dev)
{

}

static void stm32_counter_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  irqstate_t 	flags;
  int  			prescaler;	

  flags = irqsave();

  if (enable)
    {
	  up_enable_irq(STM32_IRQ_TIM6);
	  STM32_TIM_ENABLEINT(priv->timdev, 0);

	  STM32_TIM_SETMODE(priv->timdev, STM32_TIM_MODE_PULSE | STM32_TIM_ONLYFLOW);
		
	  STM32_TIM_SETPERIOD(priv->timdev, CONFIG_TIM6_PERIOD);
	  adbg("TIM6 period=%d cyles; interrupt rate=%d Hz\n",
	         CONFIG_TIM6_PERIOD, CONFIG_TIM6_FREQUENCY/CONFIG_TIM6_PERIOD);
	
	  prescaler = STM32_TIM_SETCLOCK(priv->timdev, CONFIG_TIM6_FREQUENCY);
	  adbg("TIM6 CLKIN=%d Hz, Frequency=%d Hz, prescaler=%d\n",
	         STM32_APB1_TIM6_CLKIN, CONFIG_TIM6_FREQUENCY, prescaler);

	  modifyreg16(STM32_TIM3_BASE + STM32_GTIM_EGR_OFFSET, 0, GTIM_EGR_UG);
	  modifyreg16(STM32_TIM3_BASE + STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_CEN);
  }
  else
  {
	  modifyreg16(STM32_TIM3_BASE + STM32_GTIM_CR1_OFFSET, GTIM_CR1_CEN, 0);
		
	  up_disable_irq(STM32_IRQ_TIM6);
	  STM32_TIM_DISABLEINT(priv->timdev, 0);
  }
	
  irqrestore(flags);	
}


static int stm32_counter_setup(FAR struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  int                         	ret;

  modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM3EN);

  modifyreg16(STM32_TIM3_BASE + STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_URS);

  modifyreg16(STM32_TIM3_BASE + STM32_GTIM_CCMR1_OFFSET, GTIM_CCMR1_CC1S_MASK, 0x01<<GTIM_CCMR1_CC1S_SHIFT);

  modifyreg16(STM32_TIM3_BASE + STM32_GTIM_CCER_OFFSET, GTIM_CCER_CC1P, 0);
  modifyreg16(STM32_TIM3_BASE + STM32_GTIM_CCER_OFFSET, GTIM_CCER_CC1NP, 0);

  modifyreg16(STM32_TIM3_BASE + STM32_GTIM_SMCR_OFFSET, GTIM_SMCR_SMS_MASK, GTIM_SMCR_EXTCLK1);
  modifyreg16(STM32_TIM3_BASE + STM32_GTIM_SMCR_OFFSET, GTIM_SMCR_TS_MASK, GTIM_SMCR_TI1FP1);

  stm32_configgpio(GPIO_TIM3_CH1IN);

  priv->timdev = stm32_tim_init(6);
  if (!priv->timdev)
    {
      adbg("stm32_tim_init(6) failed\n");
      return -ENODEV;
    }

  ret = up_ramvec_attach(STM32_IRQ_TIM6, stm32_counter_tim6_handler);
  if (ret < 0)
    {
      adbg("up_ramvec_attach failed: %d\n", ret);
      return ret;
    }

  /* Set the priority of the TIM6 interrupt vector */

  ret = up_prioritize_irq(STM32_IRQ_TIM6, NVIC_SYSH_HIGH_PRIORITY);
  if (ret < 0)
    {
      adbg("up_prioritize_irq failed: %d\n", ret);
      return ret;
    }

  return ret;
}

static void stm32_counter_shutdown(FAR struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

  irq_detach(STM32_IRQ_TIM6);
  stm32_tim_deinit(priv->timdev);

  modifyreg16(STM32_TIM3_BASE + STM32_GTIM_CR1_OFFSET, GTIM_CR1_CEN, 0);	
  modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM3EN, 0);	
  stm32_unconfiggpio(GPIO_TIM3_CH1IN);
}

static int stm32_counter_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

void stm32_counter_initialize(void)
{
  int ret;
//////////////////////////////////////////////////////////////////////////////////////////////////
  ret = adc_register(CONFIG_MAGNET_LX_DEVNAME, &stm32_counter_dev_magnetlx);
  if (ret < 0)
    {
      adbg("gpio_register failed: %d\n", ret);
    }			
}

#endif /* CONFIG_ADC */
