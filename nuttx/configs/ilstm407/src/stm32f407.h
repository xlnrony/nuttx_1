/****************************************************************************
 * configs/ilstm407/src/stm32f407.h
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
 ****************************************************************************/

#ifndef __CONFIGS_STM32F407_H
#define __CONFIGS_STM32F407_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration *************************************************************/

/* Assume that we have everything */

#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1
#define HAVE_SDIO       1

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB device monitor if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#if !defined(CONFIG_USBDEV_TRACE) || !defined(CONFIG_SYSTEM_USBMONITOR)
#  undef HAVE_USBMONITOR
#endif

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO support
 * is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_STM32_SDIO)
#  undef HAVE_SDIO
#endif

#undef SDIO_MINOR      /* Any minor number, default 0 */
#define SDIO_SLOTNO 0  /* Only one slot */

#ifdef HAVE_SDIO

#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != 0
#    warning "Only one MMC/SD slot, slot 0"
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  endif

#  if defined(CONFIG_NSH_MMCSDMINOR)
#    define SDIO_MINOR CONFIG_NSH_MMCSDMINOR
#  else
#    define SDIO_MINOR 0
#  endif

  /* SD card bringup does not work if performed on the IDLE thread because it
   * will cause waiting.  Use either:
   *
   *  CONFIG_NSH_ARCHINIT=y, OR
   *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
   */

#  if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_BOARD_INITTHREAD)
#    warning "SDIO initialization cannot be perfomed on the IDLE thread"
#    undef HAVE_SDIO
#  endif
#endif

/* STM32F4 Discovery GPIOs **************************************************/
/* LEDs */

#define GPIO_LED1_B       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN2)
#define GPIO_LED1_G       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_LED1_R       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
				                            
#define GPIO_LED2_B       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN5)
#define GPIO_LED2_G       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN6)
#define GPIO_LED2_R       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN7)

#define GPIO_LED3_B       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN3)
#define GPIO_LED3_G       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN4)
#define GPIO_LED3_R       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN7)

#define GPIO_MAGNET_VCC       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                    					  GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN7)

#define GPIO_MAGNET_GND      (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                    					  GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN15)

#define GPIO_BUZZER      (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                    					  GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)

#define GPIO_CLOSE_SW		(GPIO_INPUT|GPIO_FLOAT|GPIO_PORTE|GPIO_PIN1)

#define GPIO_KEY_A_OUT       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            		GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN8)
#define GPIO_KEY_A_IN 		  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN8)
				                            
#define GPIO_KEY_B_OUT       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            		GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN9)
#define GPIO_KEY_B_IN 		  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN9)

#define GPIO_KEY_C_OUT       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            		GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN10)
#define GPIO_KEY_C_IN 		  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN10)

#define GPIO_KEY_D_OUT       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            		GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN11)
#define GPIO_KEY_D_IN 		  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN11)

#define GPIO_KEY_1_OUT       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            		GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12)
#define GPIO_KEY_1_IN 		  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN12)

#define GPIO_KEY_2_OUT       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            		GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)
#define GPIO_KEY_2_IN 		  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN13)

#define GPIO_KEY_3_OUT       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|\
				                            		GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)
#define GPIO_KEY_3_IN 		  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN14)
				                    					  
/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=y && CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the STM32F4Discovery board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality. This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBHOST)
void stm32_usbhost_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=n && CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_STM32_SDIO)
void stm32_sdio_initialize(void);
#endif

#ifdef CONFIG_ADC

void stm32_adc_initialize(void);
void stm32_tim_initialize(void);

#endif

#ifdef CONFIG_GPIO
void stm32_gpio_initialize(void);
#endif

#ifdef CONFIG_KEYPAD
void stm32_keypad_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_STM32F407_H */
