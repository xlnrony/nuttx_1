/****************************************************************************
 * drivers/gpio/led.c
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
 * Compilation Switches
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/gpio/led.h>

#include <arch/irq.h>

#ifdef CONFIG_LED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int led_open(FAR struct file *filep);
static int led_close(FAR struct file *filep);
static int led_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations led_fops =
{
  led_open,   /* open */
  led_close,  	/* close */
  0,                /* read */
  0,  					/* write */
  0,               	/* seek */
  led_ioctl 		/* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0        /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: led_open
 *
 * Description:
 *   This function is called whenever the LED device is opened.
 *
 ************************************************************************************/

static int led_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct led_dev_s *dev = inode->i_private;
  uint8_t                     tmp;
  int                         ret;

  ledvdbg("crefs: %d\n", dev->crefs);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&dev->exclsem);
  if (ret < 0)
    {
      ret = -get_errno();
      goto errout;
    }

  /* Increment the count of references to the device.  If this the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = dev->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* Check if this is the first time that the driver has been opened. */

  if (tmp == 1)
    {
      /* Yes.. perform one time hardware initialization. */

      DEBUGASSERT(dev->ops->setup != NULL);
      ledvdbg("calling setup\n");

      ret = dev->ops->setup(dev);
      if (ret < 0)
        {
          goto errout_with_sem;
        }
    }

  /* Save the new open count on success */

  dev->crefs = tmp;
  ret = OK;

errout_with_sem:
  sem_post(&dev->exclsem);

errout:
  return ret;
}

/************************************************************************************
 * Name: led_close
 *
 * Description:
 *   This function is called when the LED device is closed.
 *
 ************************************************************************************/

static int led_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct led_dev_s *dev = inode->i_private;
  int                         ret;

  ledvdbg("crefs: %d\n", dev->crefs);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&dev->exclsem);
  if (ret < 0)
    {
      ret = -get_errno();
      goto errout;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (dev->crefs > 1)
    {
      dev->crefs--;
    }
  else
    {
      /* There are no more references to the port */

      dev->crefs = 0;

      /* Disable the PWM device */

      DEBUGASSERT(dev->ops->shutdown != NULL);
      ledvdbg("calling shutdown: %d\n");

      dev->ops->shutdown(dev);
    }
  ret = OK;

//errout_with_sem:
  sem_post(&dev->exclsem);

errout:
  return ret;
}

static void led_always_work(FAR void *arg)
{
  FAR struct led_dev_s *dev = arg;
  dev->ops->ioctl(dev, LED_NONE);
}

static void led_twinkle_work(FAR void *arg)
{
  FAR struct led_dev_s *dev = arg;
  if (--dev->count > 0)
  	{
	  if (dev->count % 2 != 0)
	    {
	      dev->ops->ioctl(dev, dev->color);
	    }
	  else
	    {
	      dev->ops->ioctl(dev, LED_NONE);
	    }
	  work_queue(HPWORK, &dev->work, led_twinkle_work, dev, dev->interval);
  	}
  else
  	{
	  dev->ops->ioctl(dev, LED_NONE);
  	}
}

static int led_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct led_dev_s *dev = inode->i_private;
  ssize_t                         ret = OK;

  ledvdbg("cmd: %d arg: %ld\n", cmd, arg);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&dev->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  if (dev->crefs == 0)
    {
      ret = -ENODEV;
    }
  else
  	{
     DEBUGASSERT(dev->ops->ioctl != NULL);
	  struct led_ctl *ledctl = (struct led_ctl *)arg;
	  switch(cmd)
	  	{
		  case LEDC_ALWAYS:
				dev->ops->ioctl(dev, ledctl->color);
				work_cancel(HPWORK, &dev->work);
				if (ledctl->delay != UINT32_MAX)
				  {
				    work_queue(HPWORK, &dev->work, led_always_work, dev, ledctl->delay);
				  }
		    break;
		  case LEDC_TWINKLE:
				dev->count = ledctl->delay / ledctl->interval;
				dev->color = ledctl->color;
				dev->interval = ledctl->interval;
				if (dev->count % 2 != 0)
				  {
				    dev->ops->ioctl(dev, dev->color);
				  }
				else
				  {
				    dev->ops->ioctl(dev, LED_NONE);
				  }
				work_cancel(HPWORK, &dev->work);
				work_queue(HPWORK, &dev->work, led_twinkle_work, dev, dev->interval);
		    break;
	      default:
	        ret = -ENOTTY;
	        break;
	  	}
  	}

  sem_post(&dev->exclsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int led_register(FAR const char *path, FAR struct led_dev_s *dev)
{
  /* Initialize the PWM device structure (it was already zeroed by kmm_zalloc()) */

  sem_init(&dev->exclsem, 0, 1);

  /* Register the LED device */

  ledvdbg("Registering %s\n", path);
  return register_driver(path, &led_fops, 0666, dev);
}

#endif /* CONFIG_LED */
