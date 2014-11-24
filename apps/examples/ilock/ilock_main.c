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
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/clock.h>
#include <nuttx/systemreset.h>
#include <nuttx/analog/adc.h>
#include <nuttx/gpio/gpio.h>
#include <nuttx/usb/usbhost.h>

#include <apps/netutils/netlib.h>

#include "epass3003_lib.h"
#include "jksafekey_lib.h"
#include "keypad_lib.h"
#include "led_lib.h"
#include "adc_lib.h"
#include "config_lib.h"

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

#ifndef CONFIG_EPASS3003_DEVNAME
#  define CONFIG_EPASS3003_DEVNAME "/dev/epass3003a"
#endif

#ifndef CONFIG_JKSAFEKEY_DEVNAME
#  define CONFIG_JKSAFEKEY_DEVNAME "/dev/jksafekeya"
#endif

#ifndef CONFIG_ILOCK_SETTING_PIN
#  define CONFIG_ILOCK_SETTING_PIN "159357"
#endif

#ifndef CONFIG_ILOCK_IFNAME
#  define CONFIG_ILOCK_IFNAME "eth0"
#endif

#ifndef CONFIG_ILOCK_SVRADDR
#  define CONFIG_ILOCK_SVRADDR 0x0a000080
#endif

#ifdef CONFIG_CPP_HAVE_VARARGS

/* C-99 style variadic macros are supported */

#ifdef CONFIG_DEBUG_ILOCK
#  define ilockdbg(format, ...)    dbg(format, ##__VA_ARGS__)
#  define ilocklldbg(format, ...)  lldbg(format, ##__VA_ARGS__)
#  define ilockvdbg(format, ...)   vdbg(format, ##__VA_ARGS__)
#  define ilockllvdbg(format, ...) llvdbg(format, ##__VA_ARGS__)
#else
#  define ilockdbg(x...)
#  define ilocklldbg(x...)
#  define ilockvdbg(x...)
#  define ilockllvdbg(x...)
#endif

#else /* CONFIG_CPP_HAVE_VARARGS */

/* Variadic macros NOT supported */

#ifdef CONFIG_DEBUG_ILOCK
#  define ilockdbg     dbg
#  define ilockvdbg   vdbg
#  define ilocklldbg   lldbg
#  define ilockllvdbg llvdbg
#else
#  define ilockdbg 		(void)
#  define ilockvdbg		(void)
#  define ilocklldbg		(void)
#  define ilockllvdbg	(void)
#endif

#endif /* CONFIG_CPP_HAVE_VARARGS */


/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t unlock_step = 3;
static uint32_t magnet_delay =0;
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

#if 0
int ilock_main(int argc, char *argv[])
{
  int fd;
  PLUG_RV ret;
  
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

#if 0
  ret = jksafekey_verify_pin(fd, "123466");
  if (ret < 0)
  	 {
      printf("jksafekey_verify_pin failed: %d\n", ret);
      fflush(stdout);
      goto errout;
  	 }
#endif	

  uint8_t pubkey[128] = {0};	

  ret = jksafekey_get_pubkey(fd, AT_SIGNATURE, pubkey);
  if (ret != RV_OK)
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

errout:
  printf("Closing device %s\n", CONFIG_EXAMPLES_JKSAFEKEY_DEVNAME);
  fflush(stdout);
  close(fd);

  return 0;
}
#endif

#if 0
int ilock_main(int argc, char *argv[])
{
  struct adc_msg_s sample;
  size_t readsize;
  ssize_t nbytes;

  int mvfd;
  int mgfd;
  int timfd;
	
  int errval = 0;
  int ret;

  printf("Opening the gpio device: %s\n", CONFIG_MAGNET_VCC_DEVNAME);
  mvfd = open(CONFIG_MAGNET_VCC_DEVNAME, 0);
  if (mvfd < 0)
    {
      printf("open %s failed: %d\n", CONFIG_MAGNET_VCC_DEVNAME, errno);
      errval = 1;
      goto errout;
    }

  printf("Opening the gpio device: %s\n", CONFIG_MAGNET_GND_DEVNAME);
  mgfd = open(CONFIG_MAGNET_GND_DEVNAME, 0);
  if (mgfd < 0)
    {
      printf("open %s failed: %d\n", CONFIG_MAGNET_GND_DEVNAME, errno);
      errval = 1;
      goto errout_with_mvfd;
    }

  fflush(stdout);

  ret = ioctl(mvfd, GPIOC_WRITE, 1);
  if (ret < 0)
    {
      printf("GPIOC_WRITE ioctl failed: %d\n", errno);
      errval = 2;
      goto errout_with_mgfd;
    }

  sleep(1);

  ret = ioctl(mgfd, GPIOC_WRITE, 0);
  if (ret < 0)
    {
      printf("GPIOC_WRITE ioctl failed: %d\n", errno);
      errval = 2;
      goto errout_with_mgfd;
    }

  sleep(1);

  printf("Opening the TIM device: %s\n", CONFIG_MAGNET_LX_DEVNAME);
  timfd = open(CONFIG_MAGNET_LX_DEVNAME, O_RDONLY);
  if (timfd < 0)
    {
      printf("open %s failed: %d\n", CONFIG_MAGNET_LX_DEVNAME, errno);
      errval = 2;
      goto errout_with_mgfd;
    }
	
  readsize = sizeof(struct adc_msg_s);
  nbytes = read(timfd, &sample, readsize);
  if (nbytes < 0)
    {
      errval = errno;
      if (errval != EINTR)
        {
          printf("read %s failed: %d\n", CONFIG_MAGNET_LX_DEVNAME, errval);
          errval = 3;
          goto errout_with_timfd;
        }

      printf("Interrupted read...\n");
    }
  else if (nbytes == 0)
    {
      printf("No data read, Ignoring\n");
    }
  else
    {
      printf("channel: %d value: %d\n", sample.am_channel, sample.am_data);
    }
  fflush(stdout);
	
  errval = OK;
	
  /* Error exits */

errout_with_timfd:
  close(timfd);
errout_with_mgfd:
  close(mgfd);	
errout_with_mvfd:
  close(mvfd);

errout:
  printf("Terminating!\n");
  fflush(stdout);
  return errval;
}
#endif

void light_alert_in_task(void) 
{
  bool b_temp;
  static uint32_t check_infra_red_delay0 = 0;
  static uint32_t check_infra_red_delay1 = 0;
  static uint32_t check_unlock_delay0 = 0;
  static uint32_t unlock_time_out = 0;	
  if (unlock_step == 1) 
    {
      //shock_alert_enabled = 0;
      if (adc_infra_red_op() > config.infra_red_threshold) 
        {
          if (check_infra_red_delay0 > 0)
            {
              check_infra_red_delay0--;
              if (check_infra_red_delay0 == 0 ||false ||adc_photo_resistor_op() < config.photo_resistor_threshold)
				  {
				    check_infra_red_delay0 = 0;
                  unlock_step = 2;
                  unlock_time_out = CONFIG_UNLOCK_FIRST_TIME_OUT;
				    led1_op(LED_ALWAYS, LED_GREEN, SEC2TICK(3), 0);
				    led2_op(LED_ALWAYS, LED_GREEN, SEC2TICK(3), 0);
                }
            } 
		   else 
            {
              check_infra_red_delay0 = SEC2TICK(5);
            }
        } 
      else 
	    {
          check_infra_red_delay0 = 0;
      	 }
      if (magnet_delay == 0) {
          if (check_unlock_delay0 > 0) {
              check_unlock_delay0--;
              if (check_unlock_delay0 == 0) {
                  unlock_step = 3;
                  //alert_type |= ALERT_LOCK;
                  //shock_alert_enabled = 1;
				    led3_op(LED_ALWAYS, LED_BLUE, SEC2TICK(3), 0);
              }
          } else {
              check_unlock_delay0 = SEC2TICK(5);
          }
      } else {
          check_unlock_delay0 = 0;
      }
    }
  else if (unlock_step == 2) 
	{
      if (unlock_time_out > 0) 
	    {
          unlock_time_out--;
        }
	  else 
		{
          unlock_time_out = CONFIG_UNLOCK_SECOND_TIME_OUT;
          //alert_type |= ALERT_NOLOCK_TIME_OUT;
          //act_P17(1, 60);
          led2_op(LED_ALWAYS, LED_RED, SEC2TICK(3), 0); //没关门超时
       }
      if (adc_infra_red_op() < config.infra_red_threshold) 
		{
          if (check_infra_red_delay1 > 0) 
		     {
              check_infra_red_delay1--;
              if (check_infra_red_delay1 == 0 && magnet_delay == 0 && true && adc_photo_resistor_op() >= config.photo_resistor_threshold) 
				  {
                  unlock_step = 3;
                  //alert_type |= ALERT_LOCK;
                  //shock_alert_enabled = 1;
                  led3_op(LED_ALWAYS, LED_BLUE, SEC2TICK(3), 0); //门阀扭闭通知
                }
            }
		   else 
			 {
              check_infra_red_delay1 = SEC2TICK(5);
            }
        } 
      else 
        {
          check_infra_red_delay1 = 0;
        }
    }
  else if (unlock_step == 3) 
	{
      b_temp = !P34;
      if (b_temp && (!P34_last || check_illegal_unlock_timeout())) {
          illegal_unlock("门开报警");
      }
      P34_last = b_temp;

      b_temp = get_adc_result(1) < P11_voltage;
      if (b_temp && (!P11_last || check_illegal_unlock_timeout())) {
          illegal_unlock("光感报警");
      }
      P11_last = b_temp;

      b_temp = get_adc_result(0) > P10_voltage;
      if (b_temp && (!P10_last || check_illegal_unlock_timeout())) {
          illegal_unlock("上锁报警");
      }
      P10_last = b_temp;
    }
}


void netlib_genmacaddr(uint8_t *macaddr) {
    srand(clock_systimer());
    macaddr[0] = 0xfc;
    macaddr[1] = 0xfc;
    macaddr[2] = 0xfc;
    macaddr[3] = rand();
    macaddr[4] = rand();
    macaddr[5] = rand();
}

in_addr_t netlib_formataddr(FAR const char *cp)
{
  unsigned int a, b, c, d;
  uint32_t result;

  sscanf(cp, "%3u%3u%3u%3u", &a, &b, &c, &d);
  result   = a << 8;
  result  |= b;
  result <<= 8;
  result  |= c;
  result <<= 8;
  result  |= d;
  return HTONL(result);
}

 int unlock_task(int argc, char *argv[])
{
  char keybuf[16];
  int ret;
  while(true)
    {
    	ret = keypad_readln(keybuf, sizeof(keybuf), false);
		if (ret < 0)
		{
			return ret;
		}
		if (keybuf[0]  == ':')
		  {
	        ret = keypad_readln(keybuf, sizeof(keybuf), false);
		    if (ret < 0)
		      {
		        return ret;
		      }
		    switch(keybuf[0])
		      {
               case '0': //随机mac地址
               	led1_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					netlib_genmacaddr(config.macaddr);
					netlib_setmacaddr(CONFIG_ILOCK_IFNAME, config.macaddr);
					save_config();				
					ret = keypad_readln(keybuf, sizeof(keybuf), true);
				   	if (ret < 0)
				     {
				       return ret;
				     }
               	led1_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
                 	break;
               case '1': //出厂复位网络参数
               	led1_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					ret = keypad_readln(keybuf, sizeof(keybuf), false);
				   	if (ret < 0)
					  {
				       return ret;
					  }
					if (strcmp(keybuf, CONFIG_ILOCK_SETTING_PIN) == 0) 
					  {
					    config.hostaddr.s_addr = CONFIG_NSH_IPADDR;
					    config.netmask.s_addr = CONFIG_NSH_NETMASK;					   
					    config.dripaddr.s_addr = CONFIG_NSH_DRIPADDR;		
					    config.svraddr.s_addr = CONFIG_ILOCK_SVRADDR;		

					    netlib_sethostaddr(CONFIG_ILOCK_IFNAME, &config.hostaddr);
					    netlib_setnetmask(CONFIG_ILOCK_IFNAME, &config.netmask);
					    netlib_setdraddr(CONFIG_ILOCK_IFNAME, &config.dripaddr);

					    save_config();				

					    ret = keypad_readln(keybuf, sizeof(keybuf), true);
					    if (ret < 0)
					      {
					        return ret;
					      }
					  }
               	led1_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
                  break;
              case '2': //震动阀值
               	led1_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					ret = keypad_readln(keybuf, sizeof(keybuf), false);
				   	if (ret < 0)
					  {
				       return ret;
					  }
					if (strcmp(keybuf, CONFIG_ILOCK_SETTING_PIN) == 0) 
					  {
//                      shock_alert_sampled = 1;

					    ret = keypad_readln(keybuf, sizeof(keybuf), true);
					    if (ret < 0)
					      {
					        return ret;
					      }

//                      shock_alert_sampled = 0;
					    config.shockthreshold = 0;
					    save_config();				
					  }
               	led1_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
                  break;
              case '3': //上锁检测
               	led1_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					ret = keypad_readln(keybuf, sizeof(keybuf), false);
				   	if (ret < 0)
					  {
				       return ret;
					  }
					if (strcmp(keybuf, CONFIG_ILOCK_SETTING_PIN) == 0) 
					  {
//                      P10_min_voltage = 0xff;
//                      P10_max_voltage = 0x00;

//                      lock_check_sampled = 1;

					    ret = keypad_readln(keybuf, sizeof(keybuf), true);
					    if (ret < 0)
					      {
					        return ret;
					      }

//                      lock_check_sampled = 0;

//                      P10_voltage = ((unsigned short) P10_min_voltage + (unsigned short) P10_max_voltage) / 2;

					    config.lockthreshold = 0;
					    save_config();				
					  }
               	led1_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
                  break;
              case '4': //光感阀值
               	led1_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					ret = keypad_readln(keybuf, sizeof(keybuf), false);
				   	if (ret < 0)
					  {
				       return ret;
					  }
					if (strcmp(keybuf, CONFIG_ILOCK_SETTING_PIN) == 0) 
					  {
					    ret = keypad_readln(keybuf, sizeof(keybuf), true);
					    if (ret < 0)
					      {
					        return ret;
					      }

//                      P11_voltage = atoi(pwd);
					    config.lightthreshold = 0;
					    save_config();				
					  }
               	led1_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
				   break;
              case '5': //主机IP设置
               	led1_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					ret = keypad_readln(keybuf, sizeof(keybuf), false);
				   	if (ret < 0)
					  {
				       return ret;
					  }
					if (strcmp(keybuf, CONFIG_ILOCK_SETTING_PIN) == 0) 
					  {
					    ret = keypad_readln(keybuf, sizeof(keybuf), true);
					    if (ret < 0)
					      {
					        return ret;
					      }

					    config.hostaddr.s_addr = netlib_formataddr(keybuf);
					    netlib_sethostaddr(CONFIG_ILOCK_IFNAME, &config.hostaddr);
							
					    save_config();				
					  }
               	led1_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
                 break;
              case '6': //网络掩码
               	led1_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					ret = keypad_readln(keybuf, sizeof(keybuf), false);
				   	if (ret < 0)
					  {
				       return ret;
					  }
					if (strcmp(keybuf, CONFIG_ILOCK_SETTING_PIN) == 0) 
					  {
					    ret = keypad_readln(keybuf, sizeof(keybuf), true);
					    if (ret < 0)
					      {
					        return ret;
					      }

					    config.netmask.s_addr = netlib_formataddr(keybuf);
					    netlib_setnetmask(CONFIG_ILOCK_IFNAME, &config.netmask);
							
					    save_config();				
					  }
               	led1_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
                  break;
              case '7': //网关
               	led1_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					ret = keypad_readln(keybuf, sizeof(keybuf), false);
				   	if (ret < 0)
					  {
				       return ret;
					  }
					if (strcmp(keybuf, CONFIG_ILOCK_SETTING_PIN) == 0) 
					  {
					    ret = keypad_readln(keybuf, sizeof(keybuf), true);
					    if (ret < 0)
					      {
					        return ret;
					      }

					    config.dripaddr.s_addr = netlib_formataddr(keybuf);
					    netlib_setdraddr(CONFIG_ILOCK_IFNAME, &config.dripaddr);
							
					    save_config();				
					  }
               	led1_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
                  break;
              case '8': //服务器ip
               	led1_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_GREEN, UINT32_MAX, 0);
					ret = keypad_readln(keybuf, sizeof(keybuf), false);
				   	if (ret < 0)
					  {
				       return ret;
					  }
					if (strcmp(keybuf, CONFIG_ILOCK_SETTING_PIN) == 0) 
					  {
					    ret = keypad_readln(keybuf, sizeof(keybuf), true);
					    if (ret < 0)
					      {
					        return ret;
					      }

					    config.svraddr.s_addr = netlib_formataddr(keybuf);
							
					    save_config();				
					  }
               	led1_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led2_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
					led3_op(LEDC_ALWAYS, LED_NONE, UINT32_MAX, 0);
                  break;
              case ':':
                  up_systemreset();
                  break;
		      }
		  }
       else 
		  {
            if (unlock_step == 3) 
			   {
                need_authorize = 1;
              }
	        else 
			   {
                net_unlock_delay_count = UNLOCK_DELAY;
              }
         }
     }
}

int ilock_main(int argc, char *argv[])
{
	led_init();
	led1_op(LEDC_TWINKLE, LED_RED, SEC2TICK(5), MSEC2TICK(500));
	led2_op(LEDC_TWINKLE, LED_GREEN, SEC2TICK(5), MSEC2TICK(500));
	led3_op(LEDC_TWINKLE, LED_BLUE, SEC2TICK(5), MSEC2TICK(500));
	getchar();
	led_deinit();
	return OK;
}
