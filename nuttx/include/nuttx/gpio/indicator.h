/****************************************************************************
 * include/nuttx/gpio/indicator.h
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

#ifndef __INCLUDE_NUTTX_INDICATOR_H
#define __INCLUDE_NUTTX_INDICATOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/wqueue.h>

#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_INDICATOR

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define INDC_ALWAYS	_INDC(1)
#define INDC_TWINKLE 	_INDC(2)

#define IND_NONE			0
#define IND_RED 				1
#define IND_GREEN			2
#define IND_BLUE			3
#define IND_ON         0xff

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ind_ctl_s
{
	uint8_t type;
	uint32_t delay;
	uint32_t interval;
};

#if !defined(CONFIG_BUILD_PROTECTED) || defined(__KERNEL__)

struct ind_dev_s;
struct ind_ops_s
{
  /* This method is called when the driver is opened.  The lower half driver
   * should configure and initialize the device so that it is ready for use.
   * It should not, however, output pulses until the start method is called.
   */

  CODE int (*setup)(FAR struct ind_dev_s *dev);

  /* This method is called when the driver is closed.  The lower half driver
   * should stop pulsed output, free any resources, disable the timer hardware, and
   * put the system into the lowest possible power usage state
   */

  CODE int (*shutdown)(FAR struct ind_dev_s *dev);

  CODE void (*ioctl)(FAR struct ind_dev_s *dev, uint8_t color);
};

struct ind_dev_s
{
  uint8_t                 crefs;    /* The number of times the device has been opened */
  sem_t                  exclsem;  /* Supports mutual exclusion */
  uint8_t					 type;
  uint32_t					 count;
  uint32_t 				 interval;
  struct work_s      work;       
  FAR const struct ind_ops_s *ops;
  void                   *priv;       /* Used by the arch-specific logic */	
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN int ind_register(FAR const char *path, FAR struct ind_dev_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* !defined(CONFIG_BUILD_PROTECTED) || defined(__KERNEL__) */
#endif /* CONFIG_INDICATOR */
#endif /* __INCLUDE_NUTTX_LED_H */
