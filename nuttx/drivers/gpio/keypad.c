/****************************************************************************
 * drivers/gpio/keypad.c
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
#include <poll.h>
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/gpio/keypad.h>

#include <arch/irq.h>

#ifdef CONFIG_KEYPAD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     		keypad_open(FAR struct file *filep);
static int     		keypad_close(FAR struct file *filep);
static ssize_t 	keypad_read(FAR struct file *filep, FAR char *buffer, size_t buflen);

#ifndef CONFIG_DISABLE_POLL
static int keypad_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);
#endif                        

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations keypad_fops =
{
  keypad_open,  /* open */
  keypad_close, /* close */
  keypad_read,  /* read */
  0,  /* write */
  0,  /* seek */
  0   /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , keypad_poll        /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#define keypad_givesem(s) sem_post(s)

/****************************************************************************
 * Name: keypad_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static void keypad_takesem(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}


/************************************************************************************
 * Name: keypad_open
 *
 * Description:
 *   This function is called whenever the keypad device is opened.
 *
 ************************************************************************************/

static int keypad_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct keypad_dev_s *dev = inode->i_private;
  uint8_t                     tmp;
  int                         ret;

  keypadvdbg("crefs: %d\n", dev->crefs);

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
      dev->headndx = 0;
      dev->tailndx = 0;
#ifndef CONFIG_DISABLE_POLL
	  dev->empty = true;
#endif
			
      DEBUGASSERT(dev->ops->setup != NULL);
      keypadvdbg("calling setup\n");
		
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
 * Name: keypad_close
 *
 * Description:
 *   This function is called when the keypad device is closed.
 *
 ************************************************************************************/

static int keypad_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct keypad_dev_s *dev = inode->i_private;
  int                         ret;

  keypadvdbg("crefs: %d\n", dev->crefs);

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
      dev->headndx = 0;
      dev->tailndx = 0;
#ifndef CONFIG_DISABLE_POLL
	  dev->empty = true;
#endif

      /* Disable the keypad device */

      DEBUGASSERT(dev->ops->shutdown != NULL);
      keypadvdbg("calling shutdown: %d\n");

      dev->ops->shutdown(dev);
    }
  ret = OK;

//errout_with_sem:
  sem_post(&dev->exclsem);

errout:
  return ret;
}

/************************************************************************************
 * Name: keypad_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ************************************************************************************/

static ssize_t keypad_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode           *inode;
  FAR struct keypad_dev_s *dev;
  size_t                      nbytes;
  unsigned int                tail;
  int                         ret;

  uvdbg("Entry\n");
  DEBUGASSERT(filep && filep->f_inode && buffer);
  inode = filep->f_inode;
  dev  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(dev);
  keypad_takesem(&dev->exclsem);

  /* Check if the keyboard is still connected.  We need to disable interrupts
   * momentarily to assure that there are no asynchronous disconnect events.
   */

  if (dev->crefs == 0)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB keyboard is no longer connected.  Refuse any further attempts
       * to access the driver.
       */

      ret = -ENODEV;
    }
  else
    {
      /* Is there keyboard data now? */

      while (dev->tailndx == dev->headndx)
        {
          /* No.. were we open non-blocking? */

          if (filep->f_oflags & O_NONBLOCK)
            {
              /* Yes.. then return a failure */

              ret = -EAGAIN;
              goto errout;
            }

          /* Wait for data to be available */

          keypadvdbg("Waiting...\n");

          dev->waiting = true;
          keypad_givesem(&dev->exclsem);
          keypad_takesem(&dev->waitsem);
          keypad_takesem(&dev->exclsem);

          /* Did the keyboard become disconnected while we were waiting */

          if (dev->crefs == 0)
            {
              ret = -ENODEV;
              goto errout;
            }
        }

      /* Read data from our internal buffer of received characters */

      for (tail  = dev->tailndx, nbytes = 0;
           tail != dev->headndx && nbytes < buflen;
           nbytes++)
        {
           /* Copy the next keyboard character into the user buffer */

           *buffer++ = dev->buffer[tail];

           /* Handle wrap-around of the tail index */

           if (++tail >= CONFIG_HIDKBD_BUFSIZE)
             {
               tail = 0;
             }
        }

      ret = nbytes;

      /* Update the tail index (perhaps marking the buffer empty) */

      dev->tailndx = tail;
    }

errout:
  keypad_givesem(&dev->exclsem);
  return ret;
}

#ifndef CONFIG_DISABLE_POLL
void keypad_pollnotify(FAR struct keypad_dev_s *dev)
{
  int i;

  for (i = 0; i < CONFIG_KEYPAD_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = dev->fds[i];
      if (fds)
        {
          fds->revents |= (fds->events & POLLIN);
          if (fds->revents != 0)
            {
              keypadvdbg("Report events: %02x\n", fds->revents);
              sem_post(fds->sem);
            }
        }
    }
}
#endif

#ifndef CONFIG_DISABLE_POLL
static int keypad_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode           *inode;
  FAR struct keypad_dev_s *dev;
  int                         ret = OK;
  int                         i;

  uvdbg("Entry\n");
  DEBUGASSERT(filep && filep->f_inode && fds);
  inode = filep->f_inode;
  dev  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(dev);
  keypad_takesem(&dev->exclsem);

  /* Check if the keyboard is still connected.  We need to disable interrupts
   * momentarily to assure that there are no asynchronous disconnect events.
   */

  if (dev->crefs == 0)
    {
      ret = -ENODEV;
    }
  else if (setup)
    {
      /* This is a request to set up the poll.  Find an available slot for
       * the poll structure reference
       */

      for (i = 0; i < CONFIG_KEYPAD_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_KEYPAD_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? Notify
       * the POLLIN event if there is buffered keyboard data.
       */

      if (dev->headndx != dev->tailndx)
        {
          keypad_pollnotify(dev);
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
  sem_post(&dev->exclsem);
  return ret;
}
#endif


/****************************************************************************
 * Public Functions
 ****************************************************************************/

void keypad_putbuffer(FAR struct keypad_dev_s *dev, uint8_t keycode)
{
  register unsigned int head;
  register unsigned int tail;

  /* Copy the next keyboard character into the user buffer. */

  head = dev->headndx;
  dev->buffer[head] = keycode;

  /* Increment the head index */

  if (++head >= CONFIG_KEYPAD_BUFSIZE)
    {
      head = 0;
    }

  /* If the buffer is full, then increment the tail index to make space.  Is
   * it better to lose old keystrokes or new?
   */

  tail = dev->tailndx;
  if (tail == head)
    {
      if (++tail >= CONFIG_KEYPAD_BUFSIZE)
        {
          tail = 0;
        }

      /* Save the updated tail index */

      dev->tailndx = tail;
    }

  /* Save the updated head index */

  dev->headndx = head;
}

void keypad_notify(FAR struct keypad_dev_s *dev)
{
  bool                        newstate;

  keypad_takesem(&dev->exclsem);

  newstate = (dev->headndx == dev->tailndx);
  if (!newstate)
    {
      /* Yes.. Is there a thread waiting for keyboard data now? */

      if (dev->waiting)
        {
          /* Yes.. wake it up */

          keypad_givesem(&dev->waitsem);
          dev->waiting = false;
        }

      /* Did we just transition from no data available to data
       * available?  If so, wake up any threads waiting for the
       * POLLIN event.
       */
#ifndef CONFIG_DISABLE_POLL
      if (dev->empty)
        {
          keypad_pollnotify(dev);
        }
#endif			
    }

#ifndef CONFIG_DISABLE_POLL
  dev->empty = newstate;
#endif

  keypad_givesem(&dev->exclsem);
}

int keypad_register(FAR const char *path, FAR struct keypad_dev_s *dev)
{
  /* Initialize the PWM device structure (it was already zeroed by kmm_zalloc()) */

  sem_init(&dev->exclsem, 0, 1);
  sem_init(&dev->waitsem, 0, 0);

  /* Register the keypad device */

  keypadvdbg("Registering %s\n", path);
  return register_driver(path, &keypad_fops, 0444, dev);
}

#endif /* CONFIG_KEYPAD */
