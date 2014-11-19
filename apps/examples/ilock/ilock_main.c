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

#include <nuttx/clock.h>
#include <nuttx/analog/adc.h>
#include <nuttx/gpio/gpio.h>
#include <nuttx/usb/usbhost.h>

#include <apps/netutils/netlib.h>

#include "epass3003_lib.h"
#include "jksafekey_lib.h"
#include "led_lib.h"

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

void netlib_genmacaddr(uint8_t *macaddr) {
    srand(clock_systimer());
    macaddr[0] = 0xfc;
    macaddr[1] = 0xfc;
    macaddr[2] = 0xfc;
    macaddr[3] = rand();
    macaddr[4] = rand();
    macaddr[5] = rand();
}

 int unlock_task(int argc, char *argv[])
{
  uint8_t keybuf[16];
  int ret;
  while(true)
    {
    	ret = keypad_readln(keybuf, 16, false);
		if (ret < 0)
		{
			return ret;
		}
		if (keybuf[0]  == ':')
		  {
	        ret = keypad_readln(keybuf, 16, false);
		    if (ret < 0)
		      {
		        return ret;
		      }
		    switch(keybuf[0])
		      {
               case '0': //随机mac地址
					uint8_t macaddr[IFHWADDRLEN];

					netlib_genmacaddr(macaddr);
					netlib_setmacaddr("eth0", macaddr)
									
                  assign_macaddr(macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);
                  app_eprom_flush();

			        ret = keypad_readln(keybuf, 16, true);
				    if (ret < 0)
				      {
				        return ret;
				      }
                  break;
               case '1': //出厂复位网络参数
                  lcd_set_tip("恢复出厂网络设置");
                  act_PB3(0, 0xffffffff, GREEN);
                  act_PB1(0, 0xffffffff, GREEN);
                  act_PB2(0, 0xffffffff, GREEN);
                  disable_act = 1;
                  key_board_with_copy_to_buffer(1, 0);
                  if (check_config_password()) {
                      assign_hostip(0xff, 0xff, 0xff, 0xff);
                      assign_netmask(0xff, 0xff, 0xff, 0xff);
                      assign_gateway(0xff, 0xff, 0xff, 0xff);
                      assign_serverip(0xff, 0xff, 0xff, 0xff);
                      app_eprom_flush();

                      key_board_with_copy_to_buffer(0, 1);
                  }
                  disable_act = 0;
                  act_PB3(0, 0, GREEN);
                  act_PB1(0, 0, GREEN);
                  act_PB2(0, 0, GREEN);
                  break;
              case '2': //震动阀值
                  lcd_set_tip("设置震动监测阀值");
                  act_PB3(0, 0xffffffff, GREEN);
                  act_PB1(0, 0xffffffff, GREEN);
                  act_PB2(0, 0xffffffff, GREEN);
                  disable_act = 1;
                  key_board_with_copy_to_buffer(1, 0);
                  if (check_config_password()) {
                      shock_alert_sampled = 1;

                      key_board_with_copy_to_buffer(0, 1);

                      shock_alert_sampled = 0;

                      app_eprom_write(CONFIG_START_ADDR + 26, P15_voltage >> 8);
                      app_eprom_write(CONFIG_START_ADDR + 27, P15_voltage & 0x00ff);
                      app_eprom_flush();
                  }
                  disable_act = 0;
                  act_PB3(0, 0, GREEN);
                  act_PB1(0, 0, GREEN);
                  act_PB2(0, 0, GREEN);
                  break;
              case '3': //上锁检测
                  lcd_set_tip("设置上锁监测阀值");
                  act_PB3(0, 0xffffffff, GREEN);
                  act_PB1(0, 0xffffffff, GREEN);
                  act_PB2(0, 0xffffffff, GREEN);
                  disable_act = 1;
                  key_board_with_copy_to_buffer(1, 0);
                  if (check_config_password()) {
                      P10_min_voltage = 0xff;
                      P10_max_voltage = 0x00;

                      lock_check_sampled = 1;

                      key_board_with_copy_to_buffer(0, 1);

                      lock_check_sampled = 0;

                      P10_voltage = ((unsigned short) P10_min_voltage + (unsigned short) P10_max_voltage) / 2;
                      app_eprom_write(CONFIG_START_ADDR + 28, P10_voltage >> 8);
                      app_eprom_write(CONFIG_START_ADDR + 29, P10_voltage & 0x00ff);
                      app_eprom_flush();
                  }
                  disable_act = 0;
                  act_PB3(0, 0, GREEN);
                  act_PB1(0, 0, GREEN);
                  act_PB2(0, 0, GREEN);
                  break;
              case '4': //光感阀值
                  lcd_set_tip("设置光感监测阀值");
                  act_PB3(0, 0xffffffff, GREEN);
                  act_PB1(0, 0xffffffff, GREEN);
                  act_PB2(0, 0xffffffff, GREEN);
                  disable_act = 1;
                  key_board_with_copy_to_buffer(1, 0);
                  if (check_config_password()) {
                      key_board_with_copy_to_buffer(0, 1);

                      P11_voltage = atoi(pwd);
                      app_eprom_write(CONFIG_START_ADDR + 30, P11_voltage >> 8);
                      app_eprom_write(CONFIG_START_ADDR + 31, P11_voltage & 0x00ff);
                      app_eprom_flush();
                  }
                  disable_act = 0;
                  act_PB3(0, 0, GREEN);
                  act_PB1(0, 0, GREEN);
                  act_PB2(0, 0, GREEN);
                  break;
              case '5': //主机IP设置
                  lcd_set_tip("设置主机网络地址");
                  act_PB3(0, 0xffffffff, GREEN);
                  act_PB1(0, 0xffffffff, GREEN);
                  act_PB2(0, 0xffffffff, GREEN);
                  disable_act = 1;
                  key_board_with_copy_to_buffer(1, 0);
                  if (check_config_password()) {
                      key_board_with_copy_to_buffer(0, 1);

                      format_ip();

                      assign_hostip(ip_address0, ip_address1, ip_address2, ip_address3);
                      app_eprom_flush();
                  }
                  disable_act = 0;
                  act_PB3(0, 0, GREEN);
                  act_PB1(0, 0, GREEN);
                  act_PB2(0, 0, GREEN);
                  break;
              case '6': //网络掩码
                  lcd_set_tip("设置主机掩码地址");
                  act_PB3(0, 0xffffffff, GREEN);
                  act_PB1(0, 0xffffffff, GREEN);
                  act_PB2(0, 0xffffffff, GREEN);
                  disable_act = 1;
                  key_board_with_copy_to_buffer(1, 0);
                  if (check_config_password()) {
                      key_board_with_copy_to_buffer(0, 1);
                      format_ip();

                      assign_netmask(ip_address0, ip_address1, ip_address2, ip_address3);
                      app_eprom_flush();
                  }
                  disable_act = 0;
                  act_PB3(0, 0, GREEN);
                  act_PB1(0, 0, GREEN);
                  act_PB2(0, 0, GREEN);
                  break;
              case '7': //网关
                  lcd_set_tip("设置主机网关地址");
                  act_PB3(0, 0xffffffff, GREEN);
                  act_PB1(0, 0xffffffff, GREEN);
                  act_PB2(0, 0xffffffff, GREEN);
                  disable_act = 1;
                  key_board_with_copy_to_buffer(1, 0);
                  if (check_config_password()) {
                      key_board_with_copy_to_buffer(0, 1);

                      format_ip();

                      assign_gateway(ip_address0, ip_address1, ip_address2, ip_address3);
                      app_eprom_flush();
                  }
                  disable_act = 0;
                  act_PB3(0, 0, GREEN);
                  act_PB1(0, 0, GREEN);
                  act_PB2(0, 0, GREEN);
                  break;
              case '8': //服务器ip
                  lcd_set_tip("设置服务主机地址");
                  act_PB3(0, 0xffffffff, GREEN);
                  act_PB1(0, 0xffffffff, GREEN);
                  act_PB2(0, 0xffffffff, GREEN);
                  disable_act = 1;
                  key_board_with_copy_to_buffer(1, 0);
                  if (check_config_password()) {
                      key_board_with_copy_to_buffer(0, 1);

                      format_ip();

                      assign_serverip(ip_address0, ip_address1, ip_address2, ip_address3);
                      app_eprom_flush();
                  }
                  disable_act = 0;
                  act_PB3(0, 0, GREEN);
                  act_PB1(0, 0, GREEN);
                  act_PB2(0, 0, GREEN);
                  break;
              case 0x3A:
                  soft_reset();
                  break;
              default: //显示ip信息
                  sprintf(lcd_temp[0], "H%bu.%bu.%bu.%bu", hostaddr[0], hostaddr[1], hostaddr[2], hostaddr[3]);
                  sprintf(lcd_temp[1], "N%bu.%bu.%bu.%bu", netmask[0], netmask[1], netmask[2], netmask[3]);
                  sprintf(lcd_temp[2], "G%bu.%bu.%bu.%bu", gateway[0], gateway[1], gateway[2], gateway[3]);
                  sprintf(lcd_temp[3], "S%bu.%bu.%bu.%bu", server_ip[0], server_ip[1], server_ip[2], server_ip[3]);
                  lcd_save_all(lcd_temp);
                  key_board_with_copy_to_buffer(0, 0);
                  lcd_load_all();
                  sprintf(lcd_temp[0], "震动阀值:[%bu]", P15_voltage);
                  sprintf(lcd_temp[1], "上锁阀值:[%bu]", P10_voltage);
                  sprintf(lcd_temp[2], "光感阀值:[%bu]", P11_voltage);
                  sprintf(lcd_temp[3], "MAC:%.2bX%.2bX%.2bX%.2bX%.2bX%.2bX", macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);
                  lcd_save_all(lcd_temp);
                  key_board_with_copy_to_buffer(0, 0);
                  lcd_load_all();
                  break;

		      }
		  }
		
    }
	

/*    while (1) {
        sprintf(serial_no_tip, "序列号%.10lu", serial_no);
        lcd_set_tip(serial_no_tip);
        key_board_with_copy_to_buffer(1, 0);
        if (pwd[0] == 0x3A) {
            key_board_with_copy_to_buffer(0, 0);
            switch (pwd[0]) {
                case '0': //随机mac地址
                    lcd_set_tip("设置随机硬件位址");
                    disable_act = 1;

                    gen_macaddr(macaddr);
                    assign_macaddr(macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);
                    app_eprom_flush();

                    key_board_with_copy_to_buffer(0, 1);
                    disable_act = 0;
                    break;
                case '1': //出厂复位网络参数
                    lcd_set_tip("恢复出厂网络设置");
                    act_PB3(0, 0xffffffff, GREEN);
                    act_PB1(0, 0xffffffff, GREEN);
                    act_PB2(0, 0xffffffff, GREEN);
                    disable_act = 1;
                    key_board_with_copy_to_buffer(1, 0);
                    if (check_config_password()) {
                        assign_hostip(0xff, 0xff, 0xff, 0xff);
                        assign_netmask(0xff, 0xff, 0xff, 0xff);
                        assign_gateway(0xff, 0xff, 0xff, 0xff);
                        assign_serverip(0xff, 0xff, 0xff, 0xff);
                        app_eprom_flush();

                        key_board_with_copy_to_buffer(0, 1);
                    }
                    disable_act = 0;
                    act_PB3(0, 0, GREEN);
                    act_PB1(0, 0, GREEN);
                    act_PB2(0, 0, GREEN);
                    break;
                case '2': //震动阀值
                    lcd_set_tip("设置震动监测阀值");
                    act_PB3(0, 0xffffffff, GREEN);
                    act_PB1(0, 0xffffffff, GREEN);
                    act_PB2(0, 0xffffffff, GREEN);
                    disable_act = 1;
                    key_board_with_copy_to_buffer(1, 0);
                    if (check_config_password()) {
                        shock_alert_sampled = 1;

                        key_board_with_copy_to_buffer(0, 1);

                        shock_alert_sampled = 0;

                        app_eprom_write(CONFIG_START_ADDR + 26, P15_voltage >> 8);
                        app_eprom_write(CONFIG_START_ADDR + 27, P15_voltage & 0x00ff);
                        app_eprom_flush();
                    }
                    disable_act = 0;
                    act_PB3(0, 0, GREEN);
                    act_PB1(0, 0, GREEN);
                    act_PB2(0, 0, GREEN);
                    break;
                case '3': //上锁检测
                    lcd_set_tip("设置上锁监测阀值");
                    act_PB3(0, 0xffffffff, GREEN);
                    act_PB1(0, 0xffffffff, GREEN);
                    act_PB2(0, 0xffffffff, GREEN);
                    disable_act = 1;
                    key_board_with_copy_to_buffer(1, 0);
                    if (check_config_password()) {
                        P10_min_voltage = 0xff;
                        P10_max_voltage = 0x00;

                        lock_check_sampled = 1;

                        key_board_with_copy_to_buffer(0, 1);

                        lock_check_sampled = 0;

                        P10_voltage = ((unsigned short) P10_min_voltage + (unsigned short) P10_max_voltage) / 2;
                        app_eprom_write(CONFIG_START_ADDR + 28, P10_voltage >> 8);
                        app_eprom_write(CONFIG_START_ADDR + 29, P10_voltage & 0x00ff);
                        app_eprom_flush();
                    }
                    disable_act = 0;
                    act_PB3(0, 0, GREEN);
                    act_PB1(0, 0, GREEN);
                    act_PB2(0, 0, GREEN);
                    break;
                case '4': //光感阀值
                    lcd_set_tip("设置光感监测阀值");
                    act_PB3(0, 0xffffffff, GREEN);
                    act_PB1(0, 0xffffffff, GREEN);
                    act_PB2(0, 0xffffffff, GREEN);
                    disable_act = 1;
                    key_board_with_copy_to_buffer(1, 0);
                    if (check_config_password()) {
                        key_board_with_copy_to_buffer(0, 1);

                        P11_voltage = atoi(pwd);
                        app_eprom_write(CONFIG_START_ADDR + 30, P11_voltage >> 8);
                        app_eprom_write(CONFIG_START_ADDR + 31, P11_voltage & 0x00ff);
                        app_eprom_flush();
                    }
                    disable_act = 0;
                    act_PB3(0, 0, GREEN);
                    act_PB1(0, 0, GREEN);
                    act_PB2(0, 0, GREEN);
                    break;
                case '5': //主机IP设置
                    lcd_set_tip("设置主机网络地址");
                    act_PB3(0, 0xffffffff, GREEN);
                    act_PB1(0, 0xffffffff, GREEN);
                    act_PB2(0, 0xffffffff, GREEN);
                    disable_act = 1;
                    key_board_with_copy_to_buffer(1, 0);
                    if (check_config_password()) {
                        key_board_with_copy_to_buffer(0, 1);

                        format_ip();

                        assign_hostip(ip_address0, ip_address1, ip_address2, ip_address3);
                        app_eprom_flush();
                    }
                    disable_act = 0;
                    act_PB3(0, 0, GREEN);
                    act_PB1(0, 0, GREEN);
                    act_PB2(0, 0, GREEN);
                    break;
                case '6': //网络掩码
                    lcd_set_tip("设置主机掩码地址");
                    act_PB3(0, 0xffffffff, GREEN);
                    act_PB1(0, 0xffffffff, GREEN);
                    act_PB2(0, 0xffffffff, GREEN);
                    disable_act = 1;
                    key_board_with_copy_to_buffer(1, 0);
                    if (check_config_password()) {
                        key_board_with_copy_to_buffer(0, 1);
                        format_ip();

                        assign_netmask(ip_address0, ip_address1, ip_address2, ip_address3);
                        app_eprom_flush();
                    }
                    disable_act = 0;
                    act_PB3(0, 0, GREEN);
                    act_PB1(0, 0, GREEN);
                    act_PB2(0, 0, GREEN);
                    break;
                case '7': //网关
                    lcd_set_tip("设置主机网关地址");
                    act_PB3(0, 0xffffffff, GREEN);
                    act_PB1(0, 0xffffffff, GREEN);
                    act_PB2(0, 0xffffffff, GREEN);
                    disable_act = 1;
                    key_board_with_copy_to_buffer(1, 0);
                    if (check_config_password()) {
                        key_board_with_copy_to_buffer(0, 1);

                        format_ip();

                        assign_gateway(ip_address0, ip_address1, ip_address2, ip_address3);
                        app_eprom_flush();
                    }
                    disable_act = 0;
                    act_PB3(0, 0, GREEN);
                    act_PB1(0, 0, GREEN);
                    act_PB2(0, 0, GREEN);
                    break;
                case '8': //服务器ip
                    lcd_set_tip("设置服务主机地址");
                    act_PB3(0, 0xffffffff, GREEN);
                    act_PB1(0, 0xffffffff, GREEN);
                    act_PB2(0, 0xffffffff, GREEN);
                    disable_act = 1;
                    key_board_with_copy_to_buffer(1, 0);
                    if (check_config_password()) {
                        key_board_with_copy_to_buffer(0, 1);

                        format_ip();

                        assign_serverip(ip_address0, ip_address1, ip_address2, ip_address3);
                        app_eprom_flush();
                    }
                    disable_act = 0;
                    act_PB3(0, 0, GREEN);
                    act_PB1(0, 0, GREEN);
                    act_PB2(0, 0, GREEN);
                    break;
                case 0x3A:
                    soft_reset();
                    break;
                default: //显示ip信息
                    sprintf(lcd_temp[0], "H%bu.%bu.%bu.%bu", hostaddr[0], hostaddr[1], hostaddr[2], hostaddr[3]);
                    sprintf(lcd_temp[1], "N%bu.%bu.%bu.%bu", netmask[0], netmask[1], netmask[2], netmask[3]);
                    sprintf(lcd_temp[2], "G%bu.%bu.%bu.%bu", gateway[0], gateway[1], gateway[2], gateway[3]);
                    sprintf(lcd_temp[3], "S%bu.%bu.%bu.%bu", server_ip[0], server_ip[1], server_ip[2], server_ip[3]);
                    lcd_save_all(lcd_temp);
                    key_board_with_copy_to_buffer(0, 0);
                    lcd_load_all();
                    sprintf(lcd_temp[0], "震动阀值:[%bu]", P15_voltage);
                    sprintf(lcd_temp[1], "上锁阀值:[%bu]", P10_voltage);
                    sprintf(lcd_temp[2], "光感阀值:[%bu]", P11_voltage);
                    sprintf(lcd_temp[3], "MAC:%.2bX%.2bX%.2bX%.2bX%.2bX%.2bX", macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);
                    lcd_save_all(lcd_temp);
                    key_board_with_copy_to_buffer(0, 0);
                    lcd_load_all();
                    break;
            }
        } else {
            if (unlock_step == 3) {
                need_authorize = 1;
            } else {
                net_unlock_delay_count = UNLOCK_DELAY;
            }
        }
    }
*/

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
