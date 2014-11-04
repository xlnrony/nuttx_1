/****************************************************************************
 * drivers/usbhost/usbhost_storage.c
 *
 *   Copyright (C) 2010-2013 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/scsi.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/storage.h>

/* Don't compile if prerequisites are not met */

#if defined(CONFIG_USBHOST) && !defined(CONFIG_USBHOST_BULK_DISABLE) && CONFIG_NFILE_DESCRIPTORS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* Driver support ***********************************************************/
/* This format is used to construct the /dev/sd[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "/dev/jksafekey%c"
#define DEV_NAMELEN         17

/* Used in usbhost_connect() */

#define USBHOST_IFFOUND     0x01
#define USBHOST_BINFOUND    0x02
#define USBHOST_BOUTFOUND   0x04
#define USBHOST_ALLFOUND    0x07

#define USBHOST_RETRY_USEC  (50*1000)  /* Retry each 50 milliseconds */
#define USBHOST_MAX_RETRIES 100        /* Give up after 5 seconds */
#define USBHOST_MAX_CREFS   INT16_MAX  /* Max cref count before signed overflow */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure contains the internal, private state of the USB host mass
 * storage class.
 */

struct usbhost_state_s
{
  /* This is the externally visible portion of the state */

  struct usbhost_class_s  class;

  /* This is an instance of the USB host driver bound to this class instance */

  struct usbhost_driver_s *drvr;

  /* The remainder of the fields are provide to the mass storage class */

  char                    devchar;       /* Character identifying the /dev/sd[n] device */
  volatile bool           disconnected; /* TRUE: Device has been disconnected */
  uint8_t                 ifno;         /* Interface number */
  int16_t                 crefs;        /* Reference count on the driver instance */
  sem_t                   exclsem;      /* Used to maintain mutual exclusive access */
  struct work_s           work;         /* For interacting with the worker thread */
  FAR uint8_t            *tbuffer;      /* The allocated transfer buffer */
  size_t                  tbuflen;      /* Size of the allocated transfer buffer */
  usbhost_ep_t            bulkin;       /* Bulk IN endpoint */
  usbhost_ep_t            bulkout;      /* Bulk OUT endpoint */
};

/* This is how struct usbhost_state_s looks to the free list logic */

struct usbhost_freestate_s
{
  FAR struct usbhost_freestate_s *flink;
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

/* CBW/CSW debug helpers */

#if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_VERBOSE)
static void usbhost_dumpcbw(FAR struct usbmsc_cbw_s *cbw);
static void usbhost_dumpcsw(FAR struct usbmsc_csw_s *csw);
#else
#  define usbhost_dumpcbw(cbw);
#  define usbhost_dumpcsw(csw);
#endif

/* CBW helpers */

static inline void usbhost_readcbw (const char *cdb, uint32_t len, 
																	 FAR struct usbmsc_cbw_s *cbw);
static inline void usbhost_writecbw(const char *cdb, uint32_t len, 
																	 FAR struct usbmsc_cbw_s *cbw);

/* Command helpers */

/* Worker thread actions */

static void usbhost_destroy(FAR void *arg);

/* Helpers for usbhost_connect() */

static inline int usbhost_cfgdesc(FAR struct usbhost_state_s *priv,
                                  FAR const uint8_t *configdesc, int desclen,
                                  uint8_t funcaddr);
static inline int usbhost_devinit(FAR struct usbhost_state_s *priv);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(const uint8_t *val);
static inline uint16_t usbhost_getbe16(const uint8_t *val);
static inline void usbhost_putle16(uint8_t *dest, uint16_t val);
static inline void usbhost_putbe16(uint8_t *dest, uint16_t val);
static inline uint32_t usbhost_getle32(const uint8_t *val);
static inline uint32_t usbhost_getbe32(const uint8_t *val);
static void usbhost_putle32(uint8_t *dest, uint32_t val);
static void usbhost_putbe32(uint8_t *dest, uint32_t val);

/* Transfer descriptor memory management */

static inline int usbhost_talloc(FAR struct usbhost_state_s *priv);
static inline int usbhost_tfree(FAR struct usbhost_state_s *priv);
static FAR struct usbmsc_cbw_s *usbhost_cbwalloc(FAR struct usbhost_state_s *priv);

/* struct usbhost_registry_s methods */

static struct usbhost_class_s *usbhost_create(FAR struct usbhost_driver_s *drvr,
                                              FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr);
static int usbhost_disconnected(FAR struct usbhost_class_s *class);

/* struct block_operations methods */

static int usbhost_open(FAR struct file *filep);
static int usbhost_close(FAR struct file *filep);
static ssize_t usbhost_read(FAR struct file *filep, FAR char *buffer, size_t len);
static ssize_t usbhost_write(FAR struct file *filep, FAR const char *buffer, size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID information that will  be
 * used to associate the USB host mass storage class to a connected USB
 * device.
 */

static const const struct usbhost_id_s g_jksafekey_id =
{
  USB_CLASS_MASS_STORAGE, /* base     */
  USBMSC_SUBCLASS_SCSI,  /* subclass */
  USBMSC_PROTO_BULKONLY, /* proto    */
  0xabcd,                      /* vid      */
  0x1000                       /* pid      */
};

/* This is the USB host storage class's registry entry */

static struct usbhost_registry_s g_jksafekey =
{
  NULL,                   			/* flink    */
  usbhost_create,         	/* create   */
  1,                      				/* nids     */
  &g_jksafekey_id         	/* id[]     */
};

static const struct file_operations g_jksafekey_fops =
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

/* This is a bitmap that is used to allocate device names /dev/sda-z. */

static uint32_t g_devinuse;

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
      /* The only case that an error should occr here is if the wait was
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

  /* We are not executing from an interrupt handler so we can just call
   * kmm_malloc() to get memory for the class instance.
   */

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

  /* Free the class instance (calling sched_kmm_free() in case we are executing
   * from an interrupt handler.
   */

  uvdbg("Freeing: %p\n", class);;
  kmm_free(class);
}

/****************************************************************************
 * Name: Device name management
 *
 * Description:
 *   Some tiny functions to coordinate management of mass storage device names.
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
 * Name: CBW/CSW debug helpers
 *
 * Description:
 *   The following functions are helper functions used to dump CBWs and CSWs.
 *
 * Input Parameters:
 *   cbw/csw - A reference to the CBW/CSW to dump.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_VERBOSE)
static void usbhost_dumpcbw(FAR struct usbmsc_cbw_s *cbw)
{
  int i;

  uvdbg("CBW:\n");
  uvdbg("  signature: %08x\n", usbhost_getle32(cbw->signature));
  uvdbg("  tag:       %08x\n", usbhost_getle32(cbw->tag));
  uvdbg("  datlen:    %08x\n", usbhost_getle32(cbw->datlen));
  uvdbg("  flags:     %02x\n", cbw->flags);
  uvdbg("  lun:       %02x\n", cbw->lun);
  uvdbg("  cdblen:    %02x\n", cbw->cdblen);

  uvdbg("CDB:\n");
  for (i = 0; i < cbw->cdblen; i += 8)
    {
      uvdbg("  %02x %02x %02x %02x %02x %02x %02x %02x\n",
            cbw->cdb[i],   cbw->cdb[i+1], cbw->cdb[i+2], cbw->cdb[i+3],
            cbw->cdb[i+4], cbw->cdb[i+5], cbw->cdb[i+6], cbw->cdb[i+7]);
    }
}

static void usbhost_dumpcsw(FAR struct usbmsc_csw_s *csw)
{
  uvdbg("CSW:\n");
  uvdbg("  signature: %08x\n", usbhost_getle32(csw->signature));
  uvdbg("  tag:       %08x\n", usbhost_getle32(csw->tag));
  uvdbg("  residue:   %08x\n", usbhost_getle32(csw->residue));
  uvdbg("  status:    %02x\n", csw->status);
}
#endif

/****************************************************************************
 * Name: CBW helpers
 *
 * Description:
 *   The following functions are helper functions used to format CBWs.
 *
 * Input Parameters:
 *   cbw - A reference to allocated and initialized CBW to be built.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline void
usbhost_readcbw (const char *cdb, uint32_t len, FAR struct usbmsc_cbw_s *cbw)
{
  /* Format the CBW */
  usbhost_putle32(cbw->tag, rand() << 16 | rand());
  usbhost_putle32(cbw->datlen, len);
  cbw->flags = USBMSC_CBWFLAG_IN;

  /* Format the CDB */
  cbw->cdblen  = USBMSC_MAXCDBLEN;
  memcpy(cbw->cdb, cdb, USBMSC_MAXCDBLEN);

  usbhost_dumpcbw(cbw);
}

static inline void
usbhost_writecbw(const char *cdb, uint32_t len, FAR struct usbmsc_cbw_s *cbw)
{
  /* Format the CBW */
  usbhost_putle32(cbw->tag, rand() << 16 | rand());
  usbhost_putle32(cbw->datlen, len);
  cbw->flags = 0;

  /* Format the CDB */
  cbw->cdblen  = USBMSC_MAXCDBLEN;
  memcpy(cbw->cdb, cdb, USBMSC_MAXCDBLEN);

  usbhost_dumpcbw(cbw);
}

/****************************************************************************
 * Name: usbhost_destroy
 *
 * Description:
 *   The USB mass storage device has been disconnected and the refernce count
 *   on the USB host class instance has gone to 1.. Time to destroy the USB
 *   host class instance.
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

  /* Unregister the block driver */

  usbhost_mkdevname(priv, devname);
  (void)unregister_driver(devname);

  /* Release the device name used by this connection */

  usbhost_freedevno(priv);

  /* Free the bulk endpoints */

  if (priv->bulkout)
    {
      DRVR_EPFREE(priv->drvr, priv->bulkout);
    }

  if (priv->bulkin)
    {
      DRVR_EPFREE(priv->drvr, priv->bulkin);
    }

  /* Free any transfer buffers */

  usbhost_tfree(priv);

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
 *   configdesc - A pointer to a uint8_t buffer container the configuration descripor.
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
  FAR struct usbhost_epdesc_s bindesc;
  FAR struct usbhost_epdesc_s boutdesc;
  int remaining;
  uint8_t found = 0;
  int ret;

  DEBUGASSERT(priv != NULL &&
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Keep the compiler from complaining about uninitialized variables */

  memset(&bindesc, 0, sizeof(struct usbhost_epdesc_s));
  memset(&boutdesc, 0, sizeof(struct usbhost_epdesc_s));

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

  /* Loop where there are more dscriptors to examine */

  while (remaining >= sizeof(struct usb_desc_s))
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

            /* Save the interface number and mark ONLY the interface found */

            priv->ifno = ifdesc->ifno;
            found      = USBHOST_IFFOUND;
          }
          break;

        /* Endpoint descriptor.  We expect two bulk endpoints, an IN and an
         * OUT.
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)configdesc;

            uvdbg("Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for a bulk endpoint.  We only support the bulk-only
             * protocol so I suppose anything else should really be an error.
             */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_BULK)
              {
                /* Yes.. it is a bulk endpoint.  IN or OUT? */

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an OUT bulk endpoint.  There should be only one
                     * bulk OUT endpoint.
                     */

                    if ((found & USBHOST_BOUTFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }
                    found |= USBHOST_BOUTFOUND;

                    /* Save the bulk OUT endpoint information */

                    boutdesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    boutdesc.in           = false;
                    boutdesc.funcaddr     = funcaddr;
                    boutdesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    boutdesc.interval     = epdesc->interval;
                    boutdesc.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);
                    uvdbg("Bulk OUT EP addr:%d mxpacketsize:%d\n",
                          boutdesc.addr, boutdesc.mxpacketsize);
                  }
                else
                  {
                    /* It is an IN bulk endpoint.  There should be only one
                     * bulk IN endpoint.
                     */

                    if ((found & USBHOST_BINFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }
                    found |= USBHOST_BINFOUND;

                    /* Save the bulk IN endpoint information */

                    bindesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    bindesc.in           = 1;
                    bindesc.funcaddr     = funcaddr;
                    bindesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    bindesc.interval     = epdesc->interval;
                    bindesc.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);
                    uvdbg("Bulk IN EP addr:%d mxpacketsize:%d\n",
                          bindesc.addr, bindesc.mxpacketsize);
                  }
              }
          }
          break;

        /* Other descriptors are just ignored for now */

        default:
          break;
        }

      /* If we found everything we need with this interface, then break out
       * of the loop early.
       */

      if (found == USBHOST_ALLFOUND)
        {
          break;
        }

      /* Increment the address of the next descriptor */

      configdesc += desc->len;
      remaining  -= desc->len;
    }

  /* Sanity checking... did we find all of things that we need? Hmmm..  I wonder..
   * can we work read-only or write-only if only one bulk endpoint found?
   */

  if (found != USBHOST_ALLFOUND)
    {
      ulldbg("ERROR: Found IF:%s BIN:%s BOUT:%s\n",
             (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
             (found & USBHOST_BINFOUND) != 0 ? "YES" : "NO",
             (found & USBHOST_BOUTFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the endpoints */

  ret = DRVR_EPALLOC(priv->drvr, &boutdesc, &priv->bulkout);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate Bulk OUT endpoint\n");
      return ret;
    }

  ret = DRVR_EPALLOC(priv->drvr, &bindesc, &priv->bulkin);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate Bulk IN endpoint\n");
      (void)DRVR_EPFREE(priv->drvr, priv->bulkout);
      return ret;
    }

  ullvdbg("Endpoints allocated\n");
  return OK;
}

/****************************************************************************
 * Name: usbhost_initvolume
 *
 * Description:
 *   The USB mass storage device has been successfully connected.  This
 *   completes the initialization operations.  It is first called after the
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
  int ret = OK;

  DEBUGASSERT(priv != NULL);

  /* Set aside a transfer buffer for exclusive use by the mass storage driver */

  ret = usbhost_talloc(priv);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate transfer buffer\n");
      return ret;
    }

  /* Increment the reference count.  This will prevent usbhost_destroy() from
   * being called asynchronously if the device is removed.
   */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);


  /* Register the block driver */

  char devname[DEV_NAMELEN];

  uvdbg("Register driver\n");
  usbhost_mkdevname(priv, devname);
  ret = register_driver(devname, &g_jksafekey_fops, 0666, priv);

  /* Decrement the reference count.  We incremented the reference count
   * above so that usbhost_destroy() could not be called.  We now have to
   * be concerned about asynchronous modification of crefs because the block
   * driver has been registerd.
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
 * Name: usbhost_getbe16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit big endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the big endian value.
 *
 * Returned Values:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t usbhost_getbe16(const uint8_t *val)
{
  return (uint16_t)val[0] << 8 | (uint16_t)val[1];
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
 * Name: usbhost_putbe16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit big endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_putbe16(uint8_t *dest, uint16_t val)
{
  dest[0] = val >> 8; /* Big endian means MS byte first in byte stream */
  dest[1] = val & 0xff;
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
 * Name: usbhost_getbe32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit big endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline uint32_t usbhost_getbe32(const uint8_t *val)
{
  /* Big endian means MS halfword first in byte stream */

  return (uint32_t)usbhost_getbe16(val) << 16 | (uint32_t)usbhost_getbe16(&val[2]);
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

static void usbhost_putle32(uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  usbhost_putle16(dest, (uint16_t)(val & 0xffff));
  usbhost_putle16(dest+2, (uint16_t)(val >> 16));
}

/****************************************************************************
 * Name: usbhost_putbe32
 *
 * Description:
 *   Put a (possibly unaligned) 32-bit big endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_putbe32(uint8_t *dest, uint32_t val)
{
  /* Big endian means MS halfword first in byte stream */

  usbhost_putbe16(dest, (uint16_t)(val >> 16));
  usbhost_putbe16(dest+2, (uint16_t)(val & 0xffff));
}

/****************************************************************************
 * Name: usbhost_talloc
 *
 * Description:
 *   Allocate transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_talloc(FAR struct usbhost_state_s *priv)
{
  DEBUGASSERT(priv && priv->tbuffer == NULL);
  return DRVR_ALLOC(priv->drvr, &priv->tbuffer, &priv->tbuflen);
}

/****************************************************************************
 * Name: usbhost_tfree
 *
 * Description:
 *   Free transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_tfree(FAR struct usbhost_state_s *priv)
{
  int result = OK;
  DEBUGASSERT(priv);

  if (priv->tbuffer)
    {
      DEBUGASSERT(priv->drvr);
      result         = DRVR_FREE(priv->drvr, priv->tbuffer);
      priv->tbuffer = NULL;
      priv->tbuflen = 0;
    }

  return result;
}

/****************************************************************************
 * Name: usbhost_cbwalloc
 *
 * Description:
 *   Initialize a CBW (re-using the allocated transfer buffer). Upon
 *   successful return, the CBW is cleared and has the CBW signature in place.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static FAR struct usbmsc_cbw_s *usbhost_cbwalloc(FAR struct usbhost_state_s *priv)
{
  FAR struct usbmsc_cbw_s *cbw = NULL;

  DEBUGASSERT(priv->tbuffer && priv->tbuflen >= sizeof(struct usbmsc_cbw_s))

  /* Initialize the CBW sructure */

  cbw = (FAR struct usbmsc_cbw_s *)priv->tbuffer;
  memset(cbw, 0, sizeof(struct usbmsc_cbw_s));
  usbhost_putle32(cbw->signature, USBMSC_CBW_SIGNATURE);
  return cbw;
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

  /* Allocate a USB host mass storage class instance */

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

          /* The initial reference count is 1... One reference is held by the driver */

          priv->crefs              = 1;

          /* Initialize semphores (this works okay in the interrupt context) */

          sem_init(&priv->exclsem, 0, 1);

          /* Bind the driver to the storage class instance */

          priv->drvr               = drvr;

          /* NOTE: We do not yet know the geometry of the USB mass storage device */

          /* Return the instance of the USB mass storage class */

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
 *   configdesc - A pointer to a uint8_t buffer container the configuration descripor.
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

  /* Parse the configuration descriptor to get the bulk I/O endpoints */

  ret = usbhost_cfgdesc(priv, configdesc, desclen, funcaddr);
  if (ret != OK)
    {
      udbg("usbhost_cfgdesc() failed: %d\n", ret);
    }
  else
    {
      /* Now configure the LUNs and register the block driver(s) */

      ret = usbhost_devinit(priv);
      if (ret != OK)
        {
          udbg("usbhost_initvolume() failed: %d\n", ret);
        }
    }

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

  /* Set an indication to any users of the mass storage device that the device
   * is no longer available.
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
 * struct block_operations methods
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
 *   Read the specified numer of sectors from the read-ahead buffer or from
 *   the physical device.
 *
 ****************************************************************************/

static ssize_t usbhost_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;
  ssize_t ret = 0;
  int result;

  uvdbg("usbhost_read\n");
  DEBUGASSERT(filep && filep->f_inode && buffer);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv && priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);

  usbhost_takesem(&priv->exclsem);

  /* Check if the mass storage device is still connected */

  if (priv->disconnected)
    {
      /* No... the block driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse any attempt to read
       * from the device.
       */

      ret = -ENODEV;
    }
  else if (len > USBMSC_MAXCDBLEN)
    {
      FAR struct usbmsc_cbw_s *cbw;

      /* Assume allocation failure */

      ret = -ENOMEM;

      /* Initialize a CBW (re-using the allocated transfer buffer) */

      cbw = usbhost_cbwalloc(priv);
      if (cbw)
        {
          /* Loop in the event that EAGAIN is returned (mean that the
           * transaction was NAKed and we should try again.
           */

          do
            {
              /* Assume some device failure */

              ret = -ENODEV;

              /* Construct and send the CBW */

				len -= USBMSC_MAXCDBLEN; 
              usbhost_readcbw(buffer, len, cbw);
              result = DRVR_TRANSFER(priv->drvr, priv->bulkout,
                                     (uint8_t*)cbw, USBMSC_CBW_SIZEOF);
              if (result == OK)
                {
                  /* Receive the user data */

                  result = DRVR_TRANSFER(priv->drvr, priv->bulkin,
                                         (uint8_t*)buffer + USBMSC_MAXCDBLEN, len);
                  if (result == OK)
                    {
                      /* Receive the CSW */

                      result = DRVR_TRANSFER(priv->drvr, priv->bulkin,
                                             priv->tbuffer, USBMSC_CSW_SIZEOF);
                      if (result == OK)
                        {
                          FAR struct usbmsc_csw_s *csw;

                          /* Check the CSW status */

                          csw = (FAR struct usbmsc_csw_s *)priv->tbuffer;
                          if (csw->status == 0)
                            {
                              ret = USBMSC_MAXCDBLEN + len;
                            }
                        }
                    }
                }
            } while (result == -EAGAIN);
        }
    }

  usbhost_givesem(&priv->exclsem);

  /* On success, return the number of blocks read */

  return ret;
}

/****************************************************************************
 * Name: usbhost_write
 *
 * Description:
 *   Write the specified number of sectors to the write buffer or to the
 *   physical device.
 *
 ****************************************************************************/

static ssize_t usbhost_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;
  ssize_t ret;
  int result;

  uvdbg("usbhost_write\n");
  DEBUGASSERT(filep && filep->f_inode && buffer);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv && priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);

  usbhost_takesem(&priv->exclsem);

  /* Check if the mass storage device is still connected */

  if (priv->disconnected)
    {
      /* No... the block driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse any attempt to
       * write to the device.
       */

      ret = -ENODEV;
    }
  else
    {
      FAR struct usbmsc_cbw_s *cbw;

     /* Assume allocation failure */

      ret = -ENOMEM;

      /* Initialize a CBW (re-using the allocated transfer buffer) */

      cbw = usbhost_cbwalloc(priv);
      if (cbw)
        {
          /* Assume some device failure */

          ret = -ENODEV;

          /* Construct and send the CBW */

			len -= USBMSC_MAXCDBLEN;
          usbhost_writecbw(buffer, len, cbw);

          result = DRVR_TRANSFER(priv->drvr, priv->bulkout,
                                 (uint8_t*)cbw, USBMSC_CBW_SIZEOF);
          if (result == OK)
            {
              /* Send the user data */

              result = DRVR_TRANSFER(priv->drvr, priv->bulkout,
                                     (uint8_t*)buffer + USBMSC_MAXCDBLEN, len);
              if (result == OK)
                {
                  /* Receive the CSW */

                  result = DRVR_TRANSFER(priv->drvr, priv->bulkin,
                                         priv->tbuffer, USBMSC_CSW_SIZEOF);
                  if (result == OK)
                    {
                      FAR struct usbmsc_csw_s *csw;

                      /* Check the CSW status */

                      csw = (FAR struct usbmsc_csw_s *)priv->tbuffer;
                      if (csw->status == 0)
                        {
                          ret = USBMSC_MAXCDBLEN + len;
                        }
                    }
                }
            }
        }
    }

  usbhost_givesem(&priv->exclsem);

  /* On success, return the number of blocks written */

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_storageinit
 *
 * Description:
 *   Initialize the USB host storage class.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host storage class.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_jksafekeyinit(void)
{
  /* Advertise our availability to support (certain) mass storage devices */

  return usbhost_registerclass(&g_jksafekey);
}

#endif  /* CONFIG_USBHOST && !CONFIG_USBHOST_BULK_DISABLE && !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 */
