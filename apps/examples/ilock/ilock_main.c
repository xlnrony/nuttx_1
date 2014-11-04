/****************************************************************************
 * examples/ilock/ilock_main.c
 *
 *   Copyright (C) 2011, 2013-2014 xlnrony. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/gpio/gpio.h>
#include <nuttx/usb/usbhost.h>

#include "epass3003_lib.h"
#include "jksafekey_lib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Sanity checking */

#ifndef CONFIG_USBHOST
#  error "CONFIG_USBHOST is not defined"
#endif

#ifdef CONFIG_USBHOST_INT_DISABLE
#  error "Interrupt endpoints are disabled (CONFIG_USBHOST_INT_DISABLE)"
#endif

#ifndef CONFIG_NFILE_DESCRIPTORS
#  error "CONFIG_NFILE_DESCRIPTORS > 0 needed"
#endif

/* Provide some default values for other configuration settings */

#ifndef CONFIG_EXAMPLES_ILOCK_DEFPRIO
#  define CONFIG_EXAMPLES_ILOCK_DEFPRIO 100
#endif

#ifndef CONFIG_EXAMPLES_ILOCK_STACKSIZE
#  define CONFIG_EXAMPLES_ILOCK_STACKSIZE 2048
#endif

#ifndef CONFIG_EXAMPLES_EPASS3003_DEVNAME
#  define CONFIG_EXAMPLES_EPASS3003_DEVNAME "/dev/epass3003a"
#endif

#ifndef CONFIG_EXAMPLES_JKSAFEKEY_DEVNAME
#  define CONFIG_EXAMPLES_JKSAFEKEY_DEVNAME "/dev/jksafekeya"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ilock_main
 ****************************************************************************/

#if 0
int ilock_main(int argc, char *argv[])
{
//  pid_t pid;
//  ssize_t nbytes;
  int fd;
  int ret;
  bool connected = false;
  int rhpndx;
  static FAR struct usbhost_connection_s *g_usbconn;
  

  /* First, register all of the USB host HID keyboard class driver */

  printf("epass3003_main: Register class drivers\n");
  ret = usbhost_epass3003init();
  if (ret != OK)
    {
      printf("epass3003_main: Failed to register the KBD class\n");
    }

  /* Then get an instance of the USB host interface.  The platform-specific
   * code must provide a wrapper called arch_usbhost_initialize() that will
   * perform the actual USB host initialization.
   */

  printf("epass3003_main: Initialize epass3003 driver\n");
  g_usbconn = usbhost_initialize(0);
  if (g_usbconn)
    {
      /* Start a thread to handle device connection. */

      printf("epass3003_main: Start epass3003_waiter\n");

      /* Wait for the device to change state.
       *
       * REVISIT:  This will not handle USB implementations (such as the the
       * SAMA5) which have multiple downstream, root hub ports.  In such cases,
       * connected must be an array with dimension equal to the number of root
       * hub ports.
       */

      rhpndx = usbhost_connection_wait(g_usbconn, &connected);
      DEBUGASSERT(rhpndx == OK);

      connected = !connected;
      printf("epass3003_waiter: %s\n", connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (!connected)
        {
          /* Yes.. enumerate the newly connected device */
          printf("epass3003_main: epass3003_waiter returned but not connected\n");
	   return 0;
        }

      ret = usbhost_connection_enumerate(g_usbconn, rhpndx);
      if (ret < 0)
      	 {
          printf("usbhost_connection_enumerate failed: %d\n", ret);
          fflush(stdout);
	   return 0;
      	 }
      printf("Opening device %s\n", CONFIG_EXAMPLES_EPASS3003_DEVNAME);
      fd = open(CONFIG_EXAMPLES_EPASS3003_DEVNAME, O_RDWR);
      if (fd < 0)
        {
          printf("Failed: %d\n", errno);
          fflush(stdout);
	      return 0;
        }
	  
      printf("Device %s opened\n", CONFIG_EXAMPLES_EPASS3003_DEVNAME);
      fflush(stdout);

      uint8_t txbuf[]="\x00\x84\x00\x00\x08";
      size_t txpktlen=5;
      uint8_t rxbuf[128]={0};
      size_t rxlen=128;
      char onebyte[4]={0};
      char rxfmtbuf[384]={0};

      ret = epass3003_transmit_apdu(fd, txbuf, txpktlen, rxbuf, &rxlen);
      if (ret < 0)
      	 {
          printf("epass3003_transmit_apdu failed: %d\n", ret);
          fflush(stdout);
          goto errout;
      	 }

      int i;	
      for(i=0;i<rxlen;i++)
        {
          sprintf(onebyte, "%02x ", rxbuf[i]);
          strcat(rxfmtbuf, onebyte);
      	 }
      printf("epass3003_transmit_apdu result:%s\n", rxfmtbuf);  
	  
errout:
      printf("Closing device %s\n", CONFIG_EXAMPLES_EPASS3003_DEVNAME);
      fflush(stdout);
      close(fd);
	  
    }

  return 0;
}

#endif

#if 0
int ilock_main(int argc, char *argv[])
{
  int fd;
  int errval = 0;
  int ret;

  if (argc != 3)
  	{
	   printf("gpio:main: Need two extra parameters\n");
      errval = 3;
      goto errout;
  	}
	
  printf("gpio_main: Hardware initialized. Opening the gpio device: %s\n",
          argv[1]);

  fd = open(argv[1], 0);
  if (fd < 0)
    {
      printf("adc_main: open %s failed: %d\n", argv[1], errno);
      errval = 1;
      goto errout;
    }

  fflush(stdout);

  ret = ioctl(fd, GPIOC_WRITE, *argv[2] - '0');
  if (ret < 0)
    {
      int errcode = errno;
      printf("gpio_main: GPIOC_WRITE ioctl failed: %d\n", errcode);
      errval = 2;
      goto errout_with_dev;
    }
	
  getchar();
	
  close(fd);
  fflush(stdout);
	
  return OK;
	
  /* Error exits */

errout_with_dev:
  close(fd);

errout:
  printf("Terminating!\n");
  fflush(stdout);
  return errval;
}
#endif

int ilock_main(int argc, char *argv[])
{
  int fd;
  int ret;
  
  printf("Opening device %s\n", CONFIG_EXAMPLES_JKSAFEKEY_DEVNAME);
  fd = open(CONFIG_EXAMPLES_JKSAFEKEY_DEVNAME, O_RDWR);
  if (fd < 0)
    {
      printf("Failed: %d\n", errno);
      fflush(stdout);
    return 0;
    }

  printf("Device %s opened\n", CONFIG_EXAMPLES_JKSAFEKEY_DEVNAME);
  fflush(stdout);

  ret = jksafekey_verify_pin(fd, "123466");
  if (ret < 0)
  	 {
      printf("jksafekey_verify_pin failed: %d\n", ret);
      fflush(stdout);
      goto errout;
  	 }

#if 0
  uint8_t pubkey[128] = {0};	

  ret = jksafekey_get_pubkey(fd, AT_SIGNATURE, pubkey);
  if (ret < 0)
  	 {
      printf("jksafekey_get_pubkey failed: %d\n", ret);
      fflush(stdout);
      goto errout;
  	 }

  int i;	
  char onebyte[4]={0};
  char rxfmtbuf[384]={0};
	
  for(i=0;i<128;i++)
    {
      sprintf(onebyte, "%02x ", pubkey[i]);
      strcat(rxfmtbuf, onebyte);
  	 }
  printf("jksafekey_get_pubkey result:%s\n", rxfmtbuf);  
#endif	

errout:
  printf("Closing device %s\n", CONFIG_EXAMPLES_JKSAFEKEY_DEVNAME);
  fflush(stdout);
  close(fd);

  return 0;
}


