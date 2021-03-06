/****************************************************************************
 * drivers/usbhost/usbhost_epass3003.c
 *
 *   Copyright (C) 2011-2014 xlnrony. All rights reserved.
 *   Author: xlnrony <xlnrony@gmail.com>
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

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>
#include <semaphore.h>
#include <time.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/kthread.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/hid.h>

/* Don't compile if prerequisites are not met */

#if defined(CONFIG_USBHOST) && !defined(CONFIG_USBHOST_INT_DISABLE) && CONFIG_NFILE_DESCRIPTORS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Worker thread is needed, unfortunately, to handle some cornercase failure
 * conditions.  This is kind of wasteful and begs for a re-design.
 */

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* Provide some default values for other configuration settings */

#ifndef CONFIG_EPASS3003_NPOLLWAITERS
#  define CONFIG_EPASS3003_NPOLLWAITERS 2
#endif

/* Driver support ***********************************************************/
/* This format is used to construct the /dev/kbd[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "/dev/epass3003%c"
#define DEV_NAMELEN         17

/* Used in usbhost_cfgdesc() */

#define USBHOST_IFFOUND     0x01 /* Required I/F descriptor found */
#define USBHOST_EPINFOUND   0x02 /* Required interrupt IN EP descriptor found */
#define USBHOST_EPOUTFOUND  0x04 /* Optional interrupt OUT EP descriptor found */
#define USBHOST_RQDFOUND    (USBHOST_IFFOUND|USBHOST_EPINFOUND)
#define USBHOST_ALLFOUND    (USBHOST_RQDFOUND|USBHOST_EPOUTFOUND)

#define USBHOST_MAX_CREFS   0x7fff

/* Debug ********************************************************************/
/* Both CONFIG_DEBUG_INPUT and CONFIG_DEBUG_USB could apply to this file.
 * We assume here that CONFIG_DEBUG_INPUT might be enabled separately, but
 * CONFIG_DEBUG_USB implies both.
 */

#ifndef CONFIG_DEBUG_INPUT
#  undef  idbg
#  define idbg    udbg
#  undef  illdbg
#  define illdbg  ulldbg
#  undef  ivdbg
#  define ivdbg   uvdbg
#  undef  illvdbg
#  define illvdbg ullvdbg
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure contains the internal, private state of the USB host
 * keyboard storage class.
 */

struct usbhost_state_s
{
  /* This is the externally visible portion of the state */

  struct usbhost_class_s  class;

  /* This is an instance of the USB host driver bound to this class instance */

  struct usbhost_driver_s *drvr;

  /* The remainder of the fields are provide o the keyboard class driver */

  char                    devchar;      /* Character identifying the /dev/kbd[n] device */
  volatile bool           disconnected; /* TRUE: Device has been disconnected */
//  volatile bool           open;         /* TRUE: The epass3003 device is open */
  uint8_t                 ifno;         /* Interface number */
  int16_t                 crefs;        /* Reference count on the driver instance */
  sem_t                   exclsem;      /* Used to maintain mutual exclusive access */
  FAR struct usb_ctrlreq_s  *ctrlreq;      /* The allocated request buffer */  
  FAR uint8_t            *tbuffer;      /* The allocated transfer buffer */
  size_t                  tbuflen;      /* Size of the allocated transfer buffer */
  struct work_s           work;         /* For cornercase error handling by the worker thread */

  /* Endpoints:
   * EP0 (Control):
   * - Receiving and responding to requests for USB control and class data.
   * - IN data when polled by the HID class driver (Get_Report)
   * - OUT data from the host.
   * EP Interrupt IN:
   * - Receiving asynchronous (unrequested) IN data from the device.
   * EP Interrrupt OUT (optional):
   * - Transmitting low latency OUT data to the device.
   * - If not present, EP0 used.
   */

  usbhost_ep_t            epin;         /* Interrupt IN endpoint */
  usbhost_ep_t            epout;        /* Optional interrupt OUT endpoint */

};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphores */

static void usbhost_takesem(sem_t *sem);
#define usbhost_givesem(s) sem_post(s);

/* Memory allocation services */

static inline FAR struct usbhost_state_s *usbhost_allocclass(void);
static inline void usbhost_freeclass(FAR struct usbhost_state_s *class);

/* Device name management */

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv);
static void usbhost_freedevno(FAR struct usbhost_state_s *priv);
static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv, char *devname);

/* Keyboard polling thread */

static void usbhost_destroy(FAR void *arg);

/* Helpers for usbhost_connect() */

static inline int usbhost_cfgdesc(FAR struct usbhost_state_s *priv,
                                  FAR const uint8_t *configdesc, int desclen,
                                  uint8_t funcaddr);
static inline int usbhost_devinit(FAR struct usbhost_state_s *priv);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(const uint8_t *val);
static inline void usbhost_putle16(uint8_t *dest, uint16_t val);
static inline uint32_t usbhost_getle32(const uint8_t *val);
#if 0 /* Not used */
static void usbhost_putle32(uint8_t *dest, uint32_t val);
#endif

/* Transfer descriptor memory management */

static inline int usbhost_tdalloc(FAR struct usbhost_state_s *priv);
static inline int usbhost_tdfree(FAR struct usbhost_state_s *priv);

static inline int usbhost_cralloc(FAR struct usbhost_state_s *priv);
static inline int usbhost_crfree(FAR struct usbhost_state_s *priv);

/* struct usbhost_registry_s methods */

static struct usbhost_class_s *usbhost_create(FAR struct usbhost_driver_s *drvr,
                                              FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr);
static int usbhost_disconnected(FAR struct usbhost_class_s *class);

/* Driver methods.  We export the keyboard as a standard character driver */

static int usbhost_open(FAR struct file *filep);
static int usbhost_close(FAR struct file *filep);
static ssize_t usbhost_read(FAR struct file *filep,
                            FAR char *buffer, size_t len);
static ssize_t usbhost_write(FAR struct file *filep,
                             FAR const char *buffer, size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID information that will  be
 * used to associate the USB host keyboard class driver to a connected USB
 * device.
 */

static const const struct usbhost_id_s g_epass3003_id =
{
  USB_CLASS_HID,            /* base     */
  USBHID_SUBCLASS_NONE,   /* subclass */
  USBHID_PROTOCOL_NONE, /* proto    */
  0x096e,                        /* vid      */
  0x0702                         /* pid      */
};

/* This is the USB host storage class's registry entry */

static struct usbhost_registry_s g_epass3003 =
{
  NULL,                    /* flink     */
  usbhost_create,          /* create    */
  1,                       /* nids      */
  &g_epass3003_id             /* id[]      */
};

static const struct file_operations g_epass3003_fops =
{
  usbhost_open,            /* open      */
  usbhost_close,           /* close     */
  usbhost_read,            /* read      */
  usbhost_write,           /* write     */
  0,                       /* seek      */
  0                        /* ioctl     */
#ifndef CONFIG_DISABLE_POLL
  , 0           /* poll      */
#endif
};

/* This is a bitmap that is used to allocate device names /dev/kbda-z. */

static uint32_t g_devinuse;

/* The following are used to managed the class creation operation */

static sem_t                   g_exclsem; /* For mutually exclusive thread creation */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static void usbhost_takesem(sem_t *sem)
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

/****************************************************************************
 * Name: usbhost_allocclass
 *
 * Description:
 *   This is really part of the logic that implements the create() method
 *   of struct usbhost_registry_s.  This function allocates memory for one
 *   new class instance.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s.  NULL is returned on failure; this function will
 *   will fail only if there are insufficient resources to create another
 *   USB host class instance.
 *
 ****************************************************************************/

static inline FAR struct usbhost_state_s *usbhost_allocclass(void)
{
  FAR struct usbhost_state_s *priv;

  DEBUGASSERT(!up_interrupt_context());
  priv = (FAR struct usbhost_state_s *)kmm_malloc(sizeof(struct usbhost_state_s));
  uvdbg("Allocated: %p\n", priv);;
  return priv;
}

/****************************************************************************
 * Name: usbhost_freeclass
 *
 * Description:
 *   Free a class instance previously allocated by usbhost_allocclass().
 *
 * Input Parameters:
 *   class - A reference to the class instance to be freed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline void usbhost_freeclass(FAR struct usbhost_state_s *class)
{
  DEBUGASSERT(class != NULL);

  /* Free the class instance. */

  uvdbg("Freeing: %p\n", class);;
  kmm_free(class);
}

/****************************************************************************
 * Name: Device name management
 *
 * Description:
 *   Some tiny functions to coordinate management of device names.
 *
 ****************************************************************************/

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv)
{
  irqstate_t flags;
  int devno;

  flags = irqsave();
  for (devno = 0; devno < 26; devno++)
    {
      uint32_t bitno = 1 << devno;
      if ((g_devinuse & bitno) == 0)
        {
          g_devinuse |= bitno;
          priv->devchar = 'a' + devno;
          irqrestore(flags);
          return OK;
        }
    }

  irqrestore(flags);
  return -EMFILE;
}

static void usbhost_freedevno(FAR struct usbhost_state_s *priv)
{
  int devno = 'a' - priv->devchar;

  if (devno >= 0 && devno < 26)
    {
      irqstate_t flags = irqsave();
      g_devinuse &= ~(1 << devno);
      irqrestore(flags);
    }
}

static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv, char *devname)
{
  (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->devchar);
}

/****************************************************************************
 * Name: usbhost_destroy
 *
 * Description:
 *   The USB device has been disconnected and the reference count on the USB
 *   host class instance has gone to 1.. Time to destroy the USB host class
 *   instance.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_destroy(FAR void *arg)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)arg;
  char devname[DEV_NAMELEN];

  DEBUGASSERT(priv != NULL);
  uvdbg("crefs: %d\n", priv->crefs);

  /* Unregister the driver */

  uvdbg("Unregister driver\n");
  usbhost_mkdevname(priv, devname);
  (void)unregister_driver(devname);

  /* Release the device name used by this connection */

  usbhost_freedevno(priv);

  /* Free the interrupt endpoints */

  if (priv->epin)
    {
      DRVR_EPFREE(priv->drvr, priv->epin);
    }

  if (priv->epout)
    {
      DRVR_EPFREE(priv->drvr, priv->epout);
    }

  /* Free any transfer buffers */

  usbhost_tdfree(priv);
  usbhost_crfree(priv);

  /* Destroy the semaphores */

  sem_destroy(&priv->exclsem);

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(priv->drvr);

  /* And free the class instance.  Hmmm.. this may execute on the worker
   * thread and the work structure is part of what is getting freed.  That
   * should be okay because once the work contained is removed from the
   * queue, it should not longer be accessed by the worker thread.
   */

  usbhost_freeclass(priv);
}

/****************************************************************************
 * Name: usbhost_cfgdesc
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   priv - The USB host class instance.
 *   configdesc - A pointer to a uint8_t buffer container the configuration
 *     descriptor.
 *   desclen - The length in bytes of the configuration descriptor.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static inline int usbhost_cfgdesc(FAR struct usbhost_state_s *priv,
                                  FAR const uint8_t *configdesc, int desclen,
                                  uint8_t funcaddr)
{
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  FAR struct usbhost_epdesc_s epindesc;
  FAR struct usbhost_epdesc_s epoutdesc;
  int remaining;
  uint8_t found = 0;
  bool done = false;
  int ret;

  DEBUGASSERT(priv != NULL &&
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Keep the compiler from complaining about uninitialized variables */

  memset(&epindesc, 0, sizeof(struct usbhost_epdesc_s));
  memset(&epoutdesc, 0, sizeof(struct usbhost_epdesc_s));

  /* Verify that we were passed a configuration descriptor */

  cfgdesc = (FAR struct usb_cfgdesc_s *)configdesc;
  if (cfgdesc->type != USB_DESC_TYPE_CONFIG)
    {
      return -EINVAL;
    }

  /* Get the total length of the configuration descriptor (little endian).
   * It might be a good check to get the number of interfaces here too.
  */

  remaining = (int)usbhost_getle16(cfgdesc->totallen);

  /* Skip to the next entry descriptor */

  configdesc += cfgdesc->len;
  remaining  -= cfgdesc->len;

  /* Loop where there are more descriptors to examine */

  while (remaining >= sizeof(struct usb_desc_s) && !done)
    {
      /* What is the next descriptor? */

      desc = (FAR struct usb_desc_s *)configdesc;
      switch (desc->type)
        {
        /* Interface descriptor. We really should get the number of endpoints
         * from this descriptor too.
         */

        case USB_DESC_TYPE_INTERFACE:
          {
            FAR struct usb_ifdesc_s *ifdesc = (FAR struct usb_ifdesc_s *)configdesc;

            uvdbg("Interface descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Did we already find what we needed from a preceding interface? */

            if ((found & USBHOST_RQDFOUND) == USBHOST_RQDFOUND)
              {
                /* Yes.. then break out of the loop and use the preceding
                 * interface.
                 */

                done       = true;
              }
            else
              {
                /* Otherwise, save the interface number and discard any
                 * endpoints previously found
                 */

                priv->ifno = ifdesc->ifno;
                found      = USBHOST_IFFOUND;
              }
          }
          break;

        /* HID descriptor */

        case USBHID_DESCTYPE_HID:
            uvdbg("HID descriptor\n");
            break;

        /* Endpoint descriptor.  We expect one or two interrupt endpoints,
         * a required IN endpoint and an optional OUT endpoint.
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)configdesc;

            uvdbg("Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for an interrupt endpoint. */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_INT)
              {
                /* Yes.. it is a interrupt endpoint.  IN or OUT? */

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an interrupt OUT endpoint.  There not be more than one
                     * interrupt OUT endpoint.
                     */

                    if ((found & USBHOST_EPOUTFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know what
                         * to do with this.
                         */

                        return -EINVAL;
                      }

                    found |= USBHOST_EPOUTFOUND;

                    /* Save the interrupt OUT endpoint information */

                    epoutdesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    epoutdesc.in           = false;
                    epoutdesc.funcaddr     = funcaddr;
                    epoutdesc.xfrtype      = USB_EP_ATTR_XFER_INT;
                    epoutdesc.interval     = epdesc->interval;
                    epoutdesc.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);
                    uvdbg("Interrupt OUT EP addr:%d mxpacketsize:%d\n",
                          epoutdesc.addr, epoutdesc.mxpacketsize);
                  }
                else
                  {
                    /* It is an interrupt IN endpoint.  There should be only
                     * one interrupt IN endpoint.
                     */

                    if ((found & USBHOST_EPINFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know what
                         * to do with this.
                         */

                        return -EINVAL;
                      }

                    found |= USBHOST_EPINFOUND;

                    /* Save the interrupt IN endpoint information */

                    epindesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    epindesc.in           = 1;
                    epindesc.funcaddr     = funcaddr;
                    epindesc.xfrtype      = USB_EP_ATTR_XFER_INT;
                    epindesc.interval     = epdesc->interval;
                    epindesc.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);
                    uvdbg("Interrupt IN EP addr:%d mxpacketsize:%d\n",
                          epindesc.addr, epindesc.mxpacketsize);
                  }
              }
          }
          break;

        /* Other descriptors are just ignored for now */

        default:
          uvdbg("Other descriptor: %d\n", desc->type);
          break;
        }

      /* What we found everything that we are going to find? */

      if (found == USBHOST_ALLFOUND)
        {
          /* Yes.. then break out of the loop and use the preceding interface */

          done = true;
        }

      /* Increment the address of the next descriptor */

      configdesc += desc->len;
      remaining  -= desc->len;
    }

  /* Sanity checking... did we find all of things that we need? */

  if ((found & USBHOST_RQDFOUND) != USBHOST_RQDFOUND)
    {
      ulldbg("ERROR: Found IF:%s EPIN:%s\n",
             (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
             (found & USBHOST_EPINFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the endpoints.  First, the required interrupt
   * IN endpoint.
   */

  ret = DRVR_EPALLOC(priv->drvr, &epindesc, &priv->epin);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate interrupt IN endpoint\n");
      return ret;
    }

  /* Then the optional interrupt OUT endpoint */

  ullvdbg("Found EPOOUT:%s\n",
         (found & USBHOST_EPOUTFOUND) != 0 ? "YES" : "NO");

  if ((found & USBHOST_EPOUTFOUND) != 0)
    {
      ret = DRVR_EPALLOC(priv->drvr, &epoutdesc, &priv->epout);
      if (ret != OK)
        {
          udbg("ERROR: Failed to allocate interrupt OUT endpoint\n");
          (void)DRVR_EPFREE(priv->drvr, priv->epin);
          return ret;
        }
    }

  ullvdbg("Endpoints allocated\n");
  return OK;
}

/****************************************************************************
 * Name: usbhost_devinit
 *
 * Description:
 *   The USB device has been successfully connected.  This completes the
 *   initialization operations.  It is first called after the
 *   configuration descriptor has been received.
 *
 *   This function is called from the connect() method.  This function always
 *   executes on the thread of the caller of connect().
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline int usbhost_devinit(FAR struct usbhost_state_s *priv)
{
  char devname[DEV_NAMELEN];
  int ret;

  /* Set aside a transfer buffer for exclusive use by the keyboard class driver */

  ret = usbhost_tdalloc(priv);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate transfer buffer\n");
      return ret;
    }

  ret = usbhost_cralloc(priv);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate ctrlreq buffer\n");
      return ret;
    }

  /* Increment the reference count.  This will prevent usbhost_destroy() from
   * being called asynchronously if the device is removed.
   */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);

  /* Register the driver */

  uvdbg("Register driver\n");
  usbhost_mkdevname(priv, devname);
  ret = register_driver(devname, &g_epass3003_fops, 0666, priv);

  /* We now have to be concerned about asynchronous modification of crefs
   * because the driver has been registered.
   */

  usbhost_takesem(&priv->exclsem);
  DEBUGASSERT(priv->crefs >= 2);

  /* Decrement the reference count */

  priv->crefs--;

  /* Check if we successfully initialized.  If so, handle a corner case
   * where (1) open() has been called so the reference count was > 2, but
   * the device has been disconnected. In this case, the class instance
   * needs to persist until close()
   * is called.
   */

  if (ret == OK && priv->crefs <= 1 && priv->disconnected)
    {
      /* The will cause the enumeration logic to disconnect the class
       * driver.
       */

      ret = -ENODEV;
    }

  /* Release the semaphore... there is a race condition here.
   * Decrementing the reference count and releasing the semaphore
   * allows usbhost_destroy() to execute (on the worker thread);
   * the class driver instance could get destroyed before we are
   * ready to handle it!
   */

  usbhost_givesem(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: usbhost_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Values:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t usbhost_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: usbhost_putle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/****************************************************************************
 * Name: usbhost_getle32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline uint32_t usbhost_getle32(const uint8_t *val)
{
 /* Little endian means LS halfword first in byte stream */

  return (uint32_t)usbhost_getle16(&val[2]) << 16 | (uint32_t)usbhost_getle16(val);
}

/****************************************************************************
 * Name: usbhost_putle32
 *
 * Description:
 *   Put a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

#if 0 /* Not used */
static void usbhost_putle32(uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  usbhost_putle16(dest, (uint16_t)(val & 0xffff));
  usbhost_putle16(dest+2, (uint16_t)(val >> 16));
}
#endif

/****************************************************************************
 * Name: usbhost_tdalloc
 *
 * Description:
 *   Allocate transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On success, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_tdalloc(FAR struct usbhost_state_s *priv)
{
  DEBUGASSERT(priv && priv->tbuffer == NULL);
  return DRVR_ALLOC(priv->drvr, &priv->tbuffer, &priv->tbuflen);
}

/****************************************************************************
 * Name: usbhost_tdfree
 *
 * Description:
 *   Free transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On success, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_tdfree(FAR struct usbhost_state_s *priv)
{
  int ret = OK;
  DEBUGASSERT(priv);

  if (priv->tbuffer)
    {
      DEBUGASSERT(priv->drvr);
      ret = DRVR_FREE(priv->drvr, priv->tbuffer);
      priv->tbuffer = NULL;
      priv->tbuflen = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_cralloc
 *
 * Description:
 *   Allocate ctrlreq memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On success, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_cralloc(FAR struct usbhost_state_s *priv)
{
  size_t buflen;
  int ret;
  DEBUGASSERT(priv && priv->ctrlreq == NULL);
  ret = DRVR_ALLOC(priv->drvr, (FAR uint8_t **)&priv->ctrlreq, &buflen);
  DEBUGASSERT(buflen >= sizeof(struct usb_ctrlreq_s));
  return ret;
}

/****************************************************************************
 * Name: usbhost_crfree
 *
 * Description:
 *   Free ctrlreq memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On success, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_crfree(FAR struct usbhost_state_s *priv)
{
  int ret = OK;
  DEBUGASSERT(priv);

  if (priv->ctrlreq)
    {
      DEBUGASSERT(priv->drvr);
      ret = DRVR_FREE(priv->drvr, (FAR uint8_t *)priv->ctrlreq);
      priv->ctrlreq = NULL;
    }

  return ret;
}


/****************************************************************************
 * struct usbhost_registry_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_create
 *
 * Description:
 *   This function implements the create() method of struct usbhost_registry_s.
 *   The create() method is a callback into the class implementation.  It is
 *   used to (1) create a new instance of the USB host class state and to (2)
 *   bind a USB host driver "session" to the class instance.  Use of this
 *   create() method will support environments where there may be multiple
 *   USB ports and multiple USB devices simultaneously connected.
 *
 * Input Parameters:
 *   drvr - An instance of struct usbhost_driver_s that the class
 *     implementation will "bind" to its state structure and will
 *     subsequently use to communicate with the USB host driver.
 *   id - In the case where the device supports multiple base classes,
 *     subclasses, or protocols, this specifies which to configure for.
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s that can be used by the USB host driver to communicate
 *   with the USB host class.  NULL is returned on failure; this function
 *   will fail only if the drvr input parameter is NULL or if there are
 *   insufficient resources to create another USB host class instance.
 *
 ****************************************************************************/

static FAR struct usbhost_class_s *usbhost_create(FAR struct usbhost_driver_s *drvr,
                                                  FAR const struct usbhost_id_s *id)
{
  FAR struct usbhost_state_s *priv;

  /* Allocate a USB host class instance */

  priv = usbhost_allocclass();
  if (priv)
    {
      /* Initialize the allocated storage class instance */

      memset(priv, 0, sizeof(struct usbhost_state_s));

      /* Assign a device number to this class instance */

      if (usbhost_allocdevno(priv) == OK)
        {
         /* Initialize class method function pointers */

          priv->class.connect      = usbhost_connect;
          priv->class.disconnected = usbhost_disconnected;

          /* The initial reference count is 1... One reference is held by
           * the driver.
           */

          priv->crefs              = 1;

          /* Initialize semaphores */

          sem_init(&priv->exclsem, 0, 1);

          /* Bind the driver to the storage class instance */

          priv->drvr               = drvr;

          /* Return the instance of the USB keyboard class driver */

          return &priv->class;
        }
    }

  /* An error occurred. Free the allocation and return NULL on all failures */

  if (priv)
    {
      usbhost_freeclass(priv);
    }

  return NULL;
}

/****************************************************************************
 * struct usbhost_class_s methods
 ****************************************************************************/
/****************************************************************************
 * Name: usbhost_connect
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration
 *     descriptor.
 *   desclen - The length in bytes of the configuration descriptor.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 *   NOTE that the class instance remains valid upon return with a failure.  It is
 *   the responsibility of the higher level enumeration logic to call
 *   CLASS_DISCONNECTED to free up the class driver resources.
 *
 * Assumptions:
 *   - This function will *not* be called from an interrupt handler.
 *   - If this function returns an error, the USB host controller driver
 *     must call to DISCONNECTED method to recover from the error
 *
 ****************************************************************************/

static int usbhost_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)class;
  int ret;

  DEBUGASSERT(priv != NULL &&
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the endpoints */

  ret = usbhost_cfgdesc(priv, configdesc, desclen, funcaddr);
  if (ret != OK)
    {
      udbg("usbhost_cfgdesc() failed: %d\n", ret);
    }
  else
    {
      /* Now configure the device and register the NuttX driver */

      ret = usbhost_devinit(priv);
      if (ret != OK)
        {
          udbg("usbhost_devinit() failed: %d\n", ret);
        }
    }

  /* ERROR handling:  Do nothing. If we return and error during connection,
   * the driver is required to call the DISCONNECT method.  Possibilities:
   *
   * - Failure occurred before the kbdpoll task was started successfully.
   *   In this case, the disconnection will have to be handled on the worker
   *   task.
   * - Failure occurred after the kbdpoll task was started successfully.  In
   *   this case, the disconnection can be performed on the kbdpoll thread.
   */

  return ret;
}

/****************************************************************************
 * Name: usbhost_disconnected
 *
 * Description:
 *   This function implements the disconnected() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to inform the class that the USB device has
 *   been disconnected.
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to
 *     create().
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

static int usbhost_disconnected(struct usbhost_class_s *class)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)class;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  /* Set an indication to any users of the device that the device is no
   * longer available.
   */

  flags              = irqsave();
  priv->disconnected = true;

  /* Now check the number of references on the class instance.  If it is one,
   * then we can free the class instance now.  Otherwise, we will have to
   * wait until the holders of the references free them by closing the
   * block driver.
   */

  ullvdbg("crefs: %d\n", priv->crefs);
  if (priv->crefs == 1)
    {
      /* Destroy the class instance.  If we are executing from an interrupt
       * handler, then defer the destruction to the worker thread.
       * Otherwise, destroy the instance now.
       */

      if (up_interrupt_context())
        {
          /* Destroy the instance on the worker thread. */

          uvdbg("Queuing destruction: worker %p->%p\n", priv->work.worker, usbhost_destroy);
          DEBUGASSERT(priv->work.worker == NULL);
          (void)work_queue(HPWORK, &priv->work, usbhost_destroy, priv, 0);
       }
      else
        {
          /* Do the work now */

          usbhost_destroy(priv);
        }
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Character driver methods
 ****************************************************************************/
/****************************************************************************
 * Name: usbhost_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int usbhost_open(FAR struct file *filep)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;
  irqstate_t flags;
  int ret;

  uvdbg("Entry\n");
  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv && priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);
  usbhost_takesem(&priv->exclsem);

  /* Check if the keyboard device is still connected.  We need to disable
   * interrupts momentarily to assure that there are no asynchronous disconnect
   * events.
   */

  flags = irqsave();
  if (priv->disconnected)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse any further
       * attempts to open the driver.
       */

      ret = -ENODEV;
    }
  else
    {
      /* Otherwise, just increment the reference count on the driver */

      priv->crefs++;
//      priv->open = true;
      ret        = OK;
    }
  irqrestore(flags);

  usbhost_givesem(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: usbhost_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int usbhost_close(FAR struct file *filep)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;

  uvdbg("Entry\n");
  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Decrement the reference count on the driver */

  DEBUGASSERT(priv->crefs > 1);
  usbhost_takesem(&priv->exclsem);
  priv->crefs--;

  /* Is this the last reference (other than the one held by the USB host
   * controller driver)
   */

  if (priv->crefs <= 1)
    {
      irqstate_t flags;

      /* Yes.. then the driver is no longer open */

//      priv->open    = false;

      /* We need to disable interrupts momentarily to assure that there are
       * no asynchronous disconnect events.
       */

      flags = irqsave();

      /* Check if the USB keyboard device is still connected.  If the device is
       * no longer connected, then unregister the driver and free the driver
       * class instance.
       */

      if (priv->disconnected)
        {
          /* Destroy the class instance (we can't use priv after this; we can't
           * 'give' the semaphore)
           */

          usbhost_destroy(priv);
          irqrestore(flags);
          return OK;
        }

      irqrestore(flags);
    }

  usbhost_givesem(&priv->exclsem);
  return OK;
}

/****************************************************************************
 * Name: usbhost_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t usbhost_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;
  FAR struct usb_ctrlreq_s   *ctrlreq;
  int                         ret;

  uvdbg("Entry\n");
  DEBUGASSERT(filep && filep->f_inode && buffer);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv && priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);

  DEBUGASSERT(len <= priv->tbuflen);
  
  usbhost_takesem(&priv->exclsem);

  /* Check if the keyboard is still connected.  We need to disable interrupts
   * momentarily to assure that there are no asynchronous disconnect events.
   */

  if (priv->disconnected)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB keyboard is no longer connected.  Refuse any further attempts
       * to access the driver.
       */

      ret = -ENODEV;
      goto errout;
    }

  /* Format the HID report request:
   *
   *   bmRequestType 10100001
   *   bRequest      GET_REPORT (0x01)
   *   wValue        Report Type and Report Index
   *   wIndex        Interface Number
   *   wLength       Descriptor Length
   *   Data          Descriptor Data
   */

  ctrlreq       = priv->ctrlreq;
  ctrlreq->type = USB_REQ_DIR_IN|USB_REQ_TYPE_CLASS|USB_REQ_RECIPIENT_INTERFACE;
  ctrlreq->req  = USBHID_REQUEST_GETREPORT;

  usbhost_putle16(ctrlreq->value, (USBHID_REPORTTYPE_INPUT << 8));
  usbhost_putle16(ctrlreq->index, priv->ifno);
  usbhost_putle16(ctrlreq->len,   len);

  /* Send HID report request */

  ret = DRVR_CTRLIN(priv->drvr, ctrlreq, priv->tbuffer);

  /* Check for errors -- Bail if an excessive number of errors
   * are encountered.
   */

  if (ret != OK)
    {
      udbg("ERROR: GETREPORT/INPUT, DRVR_CTRLIN returned: %d\n",ret);
      goto errout;		  
    }

  /* The report was received correctly. 
   */
  memcpy(buffer, priv->tbuffer, len); 
  ret = len;
	  
errout:
  usbhost_givesem(&priv->exclsem);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: usbhost_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t usbhost_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;
  FAR struct usb_ctrlreq_s   *ctrlreq;
  int                         ret;

  uvdbg("Entry\n");
  DEBUGASSERT(filep && filep->f_inode && buffer);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv && priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);

  DEBUGASSERT(len <= priv->tbuflen);
  
  usbhost_takesem(&priv->exclsem);

  /* Check if the keyboard is still connected.  We need to disable interrupts
   * momentarily to assure that there are no asynchronous disconnect events.
   */

  if (priv->disconnected)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB keyboard is no longer connected.  Refuse any further attempts
       * to access the driver.
       */

      ret = -ENODEV;
      goto errout;
    }

  /* Format the HID report request:
   *
   *   bmRequestType 10100001
   *   bRequest      GET_REPORT (0x01)
   *   wValue        Report Type and Report Index
   *   wIndex        Interface Number
   *   wLength       Descriptor Length
   *   Data          Descriptor Data
   */

  memcpy(priv->tbuffer, buffer, len); 

  ctrlreq       = priv->ctrlreq;
  ctrlreq->type = USB_REQ_DIR_OUT|USB_REQ_TYPE_CLASS|USB_REQ_RECIPIENT_INTERFACE;
  ctrlreq->req  = USBHID_REQUEST_SETREPORT;

  usbhost_putle16(ctrlreq->value, (USBHID_REPORTTYPE_OUTPUT << 8));
  usbhost_putle16(ctrlreq->index, priv->ifno);
  usbhost_putle16(ctrlreq->len,   len);

  /* Send HID report request */

  ret = DRVR_CTRLOUT(priv->drvr, ctrlreq, priv->tbuffer);

  /* Check for errors -- Bail if an excessive number of errors
   * are encountered.
   */

  if (ret != OK)
    {
      udbg("ERROR: SETREPORT/OUTPUT, DRVR_CTRLOUT returned: %d\n",ret);
      goto errout;		  
    }

  /* The report was received correctly. 
   */
  ret = len;
	  
errout:
  usbhost_givesem(&priv->exclsem);
  return (ssize_t)ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_kbdinit
 *
 * Description:
 *   Initialize the USB storage HID keyboard class driver.  This function
 *   should be called be platform-specific code in order to initialize and
 *   register support for the USB host HID keyboard class device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_epass3003init(void)
{
  /* Perform any one-time initialization of the class implementation */

  sem_init(&g_exclsem, 0, 1);

  /* Advertise our availability to support (certain) devices */

  return usbhost_registerclass(&g_epass3003);
}

#endif /* CONFIG_USBHOST)&& !CONFIG_USBHOST_INT_DISABLE && CONFIG_NFILE_DESCRIPTORS */
