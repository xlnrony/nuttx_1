/****************************************************************************
 * drivers/gpio/gpio.c
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
#include <nuttx/gpio/gpio.h>

#include <arch/irq.h>

#ifdef CONFIG_GPIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     gpio_open(FAR struct file *filep);
static int     gpio_close(FAR struct file *filep);
static ssize_t gpio_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t gpio_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int     gpio_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

#ifndef CONFIG_DISABLE_POLL
static int gpio_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);
#endif                        

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations gpio_fops =
{
  gpio_open,  /* open */
  gpio_close, /* close */
  gpio_read,  /* read */
  gpio_write, /* write */
  0,         /* seek */
  gpio_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , gpio_poll        /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: gpio_open
 *
 * Description:
 *   This function is called whenever the GPIO device is opened.
 *
 ************************************************************************************/

static int gpio_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct gpio_dev_s *dev = inode->i_private;
  uint8_t                     tmp;
  int                         ret;

  gpiovdbg("crefs: %d\n", dev->crefs);

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
      gpiovdbg("calling setup\n");

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
 * Name: gpio_close
 *
 * Description:
 *   This function is called when the GPIO device is closed.
 *
 ************************************************************************************/

static int gpio_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct gpio_dev_s *dev = inode->i_private;
  int                         ret;

  gpiovdbg("crefs: %d\n", dev->crefs);

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
      gpiovdbg("calling shutdown: %d\n");

      dev->ops->shutdown(dev);
    }
  ret = OK;

//errout_with_sem:
  sem_post(&dev->exclsem);

errout:
  return ret;
}

/************************************************************************************
 * Name: gpio_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ************************************************************************************/

static ssize_t gpio_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct gpio_dev_s *dev = inode->i_private;
  int                         ret;

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
     DEBUGASSERT(dev->ops->read != NULL);
     ret = dev->ops->read(dev, buffer, buflen);
  	}

  sem_post(&dev->exclsem);
  return ret;
}

/************************************************************************************
 * Name: gpio_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ************************************************************************************/

static ssize_t gpio_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct gpio_dev_s *dev = inode->i_private;
  int                         ret;

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
     DEBUGASSERT(dev->ops->write != NULL);
     ret = dev->ops->write(dev, buffer, buflen);
  	}

  sem_post(&dev->exclsem);
  return ret;
}

/************************************************************************************
 * Name: gpio_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the PWM work is done.
 *
 ************************************************************************************/

static int gpio_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct gpio_dev_s *dev = inode->i_private;
  int                         ret;

  gpiovdbg("cmd: %d arg: %ld\n", cmd, arg);

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
     ret = dev->ops->ioctl(dev, cmd, arg);
  	}

  sem_post(&dev->exclsem);
  return ret;
}

#ifndef CONFIG_DISABLE_POLL
static void gpio_pollnotify(FAR struct gpio_dev_s *dev)
{
  int i;

  for (i = 0; i < CONFIG_HIDKBD_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = dev->fds[i];
      if (fds)
        {
          fds->revents |= (fds->events & POLLIN);
          if (fds->revents != 0)
            {
              uvdbg("Report events: %02x\n", fds->revents);
              sem_post(fds->sem);
            }
        }
    }
}
#endif


#ifndef CONFIG_DISABLE_POLL
static int gpio_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode           *inode;
  FAR struct gpio_dev_s *dev;
  int                         ret = OK;
  int                         i;

  uvdbg("Entry\n");
  DEBUGASSERT(filep && filep->f_inode && fds);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(dev);
  usbhost_takesem(&dev->exclsem);

  /* Check if the keyboard is still connected.  We need to disable interrupts
   * momentarily to assure that there are no asynchronous disconnect events.
   */

  if (priv->crefs == 0)
    {
      ret = -ENODEV;
    }
  else if (setup)
    {
      /* This is a request to set up the poll.  Find an available slot for
       * the poll structure reference
       */

      for (i = 0; i < CONFIG_HIDKBD_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!dev->fds[i])
            {
              /* Bind the poll structure and this slot */

              dev->fds[i] = fds;
              fds->priv    = &dev->fds[i];
              break;
            }
        }

      if (i >= CONFIG_HIDKBD_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? Notify
       * the POLLIN event if there is buffered keyboard data.
       */

      if (priv->headndx != priv->tailndx)
        {
          usbhost_pollnotify(priv);
        }
    }
  else
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot);

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  sem_post(&priv->exclsem);
  return ret;
}
#endif


/****************************************************************************
 * Public Functions
 ****************************************************************************/

int gpio_register(FAR const char *path, FAR struct gpio_dev_s *dev)
{
  /* Initialize the PWM device structure (it was already zeroed by kmm_zalloc()) */

  sem_init(&dev->exclsem, 0, 1);

  /* Register the GPIO device */

  gpiovdbg("Registering %s\n", path);
  return register_driver(path, &gpio_fops, 0666, dev);
}

#endif /* CONFIG_GPIO */
