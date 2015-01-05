
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

#include <arch/systemreset.h>

#include <nuttx/clock.h>
#include <nuttx/analog/adc.h>
#include <nuttx/gpio/gpio.h>
#include <nuttx/usb/usbhost.h>

#include <apps/netutils/netlib.h>

#include "ilock_debug.h"
#include "ilock_protocal.h"
#include "ilock_authorize.h"

#include "epass3003_lib.h"
#include "jksafekey_lib.h"
#include "keypad_lib.h"
#include "led_lib.h"
#include "adc_lib.h"
#include "config_lib.h"
#include "gpio_lib.h"
#include "buzzer_lib.h"
#include "file_lib.h"
#include "ipconflict_lib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

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

#ifndef CONFIG_UNLOCK_FIRST_TIME_OUT
#  define CONFIG_UNLOCK_FIRST_TIME_OUT SEC2TICK(600)
#endif

#ifndef CONFIG_UNLOCK_SECOND_TIME_OUT
#  define CONFIG_UNLOCK_SECOND_TIME_OUT SEC2TICK(60)
#endif

#ifndef CONFIG_MAGNET_DELAY
#  define CONFIG_MAGNET_DELAY SEC2TICK(10)
#endif

#ifndef CONFIG_SCAN_TASK_STACKSIZE
#  define CONFIG_SCAN_TASK_STACKSIZE 4096
#endif

#ifndef CONFIG_UNLOCK_TASK_STACKSIZE
#  define CONFIG_UNLOCK_TASK_STACKSIZE 4096
#endif

#ifndef CONFIG_NET_TASK_STACKSIZE
#  define CONFIG_NET_TASK_STACKSIZE 4096
#endif

#ifndef CONFIG_LOG_TASK_STACKSIZE
#  define CONFIG_LOG_TASK_STACKSIZE 4096
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_unlock_step = 3;
static uint32_t g_magnet_delay = 0;
static bool g_shock_resistor_sampled = false;
static bool g_infra_red_sampled = false;
static int32_t g_min_infra_red;
static int32_t g_max_infra_red;

/****************************************************************************
 * Public Data
 ****************************************************************************/

FAR struct socket g_psock;

bool g_connected = false;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void netlib_genmacaddr(uint8_t * macaddr)
{
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
  result = a << 8;
  result |= b;
  result <<= 8;
  result |= c;
  result <<= 8;
  result |= d;
  return HTONL(result);
}

bool sensor_check_timeout(uint32_t *last_tick)
{
  if (ABS(clock_systimer() - *last_tick) > SEC2TICK(20))
    {
      *last_tick = clock_systimer();
      return true;
    }
  else
    {
      return false;
    }
}

void unlock_step_in_task(void)
{
  bool flag;
  static uint32_t check_infra_red_delay0 = 0;
  static uint32_t check_infra_red_delay1 = 0;
  static uint32_t check_unlock_delay0 = 0;
  static uint32_t unlock_time_out = 0;
  static uint32_t last_close_sw_tick = 0;
  static uint32_t last_photo_resistor_tick = 0;
  static uint32_t last_infra_red_tick = 0;
  static uint32_t last_shock_resistor_tick = 0;
  static uint32_t last_ip_conflict_tick = 0;

  if (g_unlock_step == 1)
    {
      // shock_alert_enabled = 0;
      if (adc_infra_red_op() > config->infra_red_threshold)
        {
          if (check_infra_red_delay0 > 0)
            {
              check_infra_red_delay0--;
              if (check_infra_red_delay0 == 0 || !closesw_read()
                  || adc_photo_resistor_op() < config->photo_resistor_threshold)
                {
                  check_infra_red_delay0 = 0;
                  g_unlock_step = 2;
                  unlock_time_out = CONFIG_UNLOCK_FIRST_TIME_OUT;
                  led1_op(INDC_ALWAYS, IND_GREEN, SEC2TICK(3), 0);
                  led2_op(INDC_ALWAYS, IND_GREEN, SEC2TICK(3), 0);
                  ilockdbg("door opened!\n");
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
      if (g_magnet_delay == 0)
        {
          if (check_unlock_delay0 > 0)
            {
              check_unlock_delay0--;
              if (check_unlock_delay0 == 0)
                {
                  g_unlock_step = 3;
                  auth_send_alert_to_disk_or_net(ALERT_LOCK);
                  // shock_alert_enabled = 1;
                  led3_op(INDC_ALWAYS, IND_BLUE, SEC2TICK(3), 0);
                  ilockdbg("door closing and magnet released!\n");
                }
            }
          else
            {
              check_unlock_delay0 = SEC2TICK(5);
            }
        }
      else
        {
          check_unlock_delay0 = 0;
        }
    }
  else if (g_unlock_step == 2)
    {
      if (unlock_time_out > 0)
        {
          unlock_time_out--;
        }
      else
        {
          unlock_time_out = CONFIG_UNLOCK_SECOND_TIME_OUT;
          auth_send_alert_to_disk_or_net(ALERT_NOLOCK_TIME_OUT);
          buzzer_op(INDC_ALWAYS, IND_ON, SEC2TICK(3), 0);
          led2_op(INDC_ALWAYS, IND_RED, SEC2TICK(3), 0);        // 没关门超时
          ilockdbg("door opened time out!\n");
        }
      if (adc_infra_red_op() < config->infra_red_threshold)
        {
          if (check_infra_red_delay1 > 0)
            {
              check_infra_red_delay1--;
              if (check_infra_red_delay1 == 0 && g_magnet_delay == 0 &&
                  closesw_read() &&
                  adc_photo_resistor_op() >= config->photo_resistor_threshold)
                {
                  g_unlock_step = 3;
                  auth_send_alert_to_disk_or_net(ALERT_LOCK);
                  // shock_alert_enabled = 1;
                  led3_op(INDC_ALWAYS, IND_BLUE, SEC2TICK(3), 0);       // 门阀扭闭通知
                  ilockdbg("door closed.\n");
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
  else if (g_unlock_step == 3)
    {
      if (sensor_check_timeout(&last_close_sw_tick))
        {
          flag = !closesw_read();
          if (flag)
            {
              auth_send_alert_to_disk_or_net(ALERT_CLOSE_SWITCH);
              led1_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(200));
              buzzer_op(INDC_TWINKLE, IND_ON, SEC2TICK(3), MSEC2TICK(200));
              ilockdbg("close switch alert!\n");
            }
        }

      if (sensor_check_timeout(&last_photo_resistor_tick))
        {
          flag = adc_photo_resistor_op() < config->photo_resistor_threshold;
          if (flag)
            {
              auth_send_alert_to_disk_or_net(ALERT_PHOTO_RESISTOR);
              led1_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(200));
              buzzer_op(INDC_TWINKLE, IND_ON, SEC2TICK(3), MSEC2TICK(200));
              ilockdbg("photo resistor alert!\n");
            }
        }

      if (sensor_check_timeout(&last_infra_red_tick))
        {
          flag = adc_infra_red_op() > config->infra_red_threshold;
          if (flag)
            {
              auth_send_alert_to_disk_or_net(ALERT_INFRA_RED);
              led1_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(200));
              buzzer_op(INDC_TWINKLE, IND_ON, SEC2TICK(3), MSEC2TICK(200));
              ilockdbg("infra red alert!\n");
            }
        }

      if (sensor_check_timeout(&last_shock_resistor_tick))
        {
          flag = adc_shock_resistor_op() > config->shock_resistor_threshold;
          if (flag)
            {
              auth_send_alert_to_disk_or_net(ALERT_SHOCK_RESISTOR);
              led3_op(INDC_TWINKLE, IND_RED, SEC2TICK(5), MSEC2TICK(200));
              buzzer_op(INDC_TWINKLE, IND_ON, SEC2TICK(5), MSEC2TICK(200));
              ilockdbg("shock resistor alert!\n");
            }
        }

      if (sensor_check_timeout(&last_ip_conflict_tick))
        {
          flag = ipconflict_read();
          if (flag)
            {
              led1_op(INDC_ALWAYS, IND_RED, SEC2TICK(10), 0);
              led2_op(INDC_ALWAYS, IND_RED, SEC2TICK(10), 0);
              led3_op(INDC_ALWAYS, IND_RED, SEC2TICK(10), 0);
              ilockdbg("ip conflict alert!\n");
            }
        }
    }
}

void threshold_sample_in_task(void)
{
  int32_t t;
  if (g_shock_resistor_sampled)
    {
      t = adc_shock_resistor_op();
      if (t > config->shock_resistor_threshold)
        {
          config->shock_resistor_threshold = t;
        }
    } ;
  if (g_infra_red_sampled)
    {
      t = adc_infra_red_op();
      if (t < g_min_infra_red)
        {
          g_min_infra_red = t;
        }
      if (t > g_max_infra_red)
        {
          g_max_infra_red = t;
        }
    }
}

void act_magnet_in_task(void)
{
  if (g_magnet_delay > 0)
    {
      g_magnet_delay--;
      if (g_magnet_delay == 0)
        {
          magnet_write(true);
        }
    }
}

void heart_beat_in_task(void)
{
  static uint32_t last_heart_beat_tick = 0;

  if (g_connected && ABS(clock_systimer() - last_heart_beat_tick) > SEC2TICK(10))
    {
      last_heart_beat_tick = clock_systimer();
      (void)protocal_send_heart_beat();
    }
}

static int scan_task(int argc, char *argv[])
{
  while (true)
    {
      usleep(USEC_PER_TICK);
      act_magnet_in_task();
      unlock_step_in_task();
      heart_beat_in_task();
      threshold_sample_in_task();
    }
}

void act_unlock(void)
{
  auth_init();
  g_unlock_step = 1;
  g_magnet_delay = CONFIG_MAGNET_DELAY;
  magnet_write(false);
  led1_op(INDC_ALWAYS, IND_GREEN, SEC2TICK(3), 0);
}

bool authorize(char *pwd)
{
  int jksafekey_fd;
  PLUG_RV ret;
  uint8_t pubkey[CONFIG_PUBKEY_SIZE] = { 0 };

  jksafekey_fd = open(CONFIG_JKSAFEKEY_DEVNAME, O_RDWR);
  if (jksafekey_fd < 0)
    {
      ilockdbg("opening device %s Failed: %d\n",
               CONFIG_JKSAFEKEY_DEVNAME, errno);
      buzzer_op(INDC_TWINKLE, IND_ON, SEC2TICK(1), MSEC2TICK(200));
      led1_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(200));
      goto errclose;
    }

  ret = jksafekey_get_pubkey(jksafekey_fd, AT_SIGNATURE, pubkey);
  if (ret != RV_OK)
    {
      ilockdbg("jksafekey_get_pubkey failed: %d\n", ret);
      buzzer_op(INDC_TWINKLE, IND_ON, SEC2TICK(1), MSEC2TICK(200));
      led1_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(200));
      led2_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(200));
      goto errclose;
    }

  ret = jksafekey_verify_pin(jksafekey_fd, pwd);
  if (ret < 0)
    {
      ilockdbg("jksafekey_verify_pin failed: %d\n", ret);
      auth_send_log_to_disk_or_net(LOG_KEY_PASSWORD_ERROR, pubkey);
      buzzer_op(INDC_TWINKLE, IND_ON, SEC2TICK(3), MSEC2TICK(200));
      led3_op(INDC_ALWAYS, IND_RED, SEC2TICK(3), MSEC2TICK(200));
      goto errclose;
    }

  close(jksafekey_fd);

  auth_time_out_check();
  if (!auth_this_time(pubkey) && (auth_init(), !auth_this_time(pubkey)))
    {
      led1_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(200));
      led3_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(200));
      buzzer_op(INDC_TWINKLE, IND_ON, SEC2TICK(1), MSEC2TICK(200));
      goto errout;
    }
  if (auth_need_more())
    {
      auth_send_log_to_disk_or_net(LOG_HALF_UNLOCK, pubkey);
      led1_op(INDC_TWINKLE, IND_GREEN, SEC2TICK(3), MSEC2TICK(200));
      goto errout;
    }

  auth_send_log_to_disk_or_net_by_unlock_type(pubkey);
  act_unlock();

  return true;

errclose:
  close(jksafekey_fd);
errout:
  return false;
}

static int unlock_task(int argc, char *argv[])
{
  char keybuf[16];
  int ret;

  while (true)
    {
      ret = keypad_readln(keybuf, sizeof(keybuf), false);
      if (ret < 0)
        {
          return ret;
        }
      if (keybuf[0] == ':')
        {
          ret = keypad_readln(keybuf, sizeof(keybuf), false);
          if (ret < 0)
            {
              return ret;
            }
          switch (keybuf[0])
            {
              case '0':          // 随机mac地址
                led1_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                netlib_genmacaddr(config->macaddr);
                netlib_setmacaddr(CONFIG_ILOCK_IFNAME, config->macaddr);

                (void)save_config();

                ret = keypad_readln(keybuf, sizeof(keybuf), true);
                if (ret < 0)
                  {
                    return ret;
                  }
                led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                break;
              case '1':          // 出厂复位网络参数
                led1_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                ret = keypad_readln(keybuf, sizeof(keybuf), false);
                if (ret < 0)
                  {
                    return ret;
                  }
                if (strcmp(keybuf, CONFIG_ILOCK_SETTING_PIN) == 0)
                  {
                    config->hostaddr.s_addr = inet_addr(CONFIG_HOSTADDR_DEF_VALUE);
                    config->netmask.s_addr = inet_addr(CONFIG_NETMASK_DEF_VALUE);
                    config->dripaddr.s_addr = inet_addr(CONFIG_DRIPADDR_DEF_VALUE);
                    config->svraddr.s_addr = inet_addr(CONFIG_SVRADDR_DEF_VALUE);

                    netlib_sethostaddr(CONFIG_ILOCK_IFNAME, &config->hostaddr);
                    netlib_setnetmask(CONFIG_ILOCK_IFNAME, &config->netmask);
                    netlib_setdraddr(CONFIG_ILOCK_IFNAME, &config->dripaddr);

                    (void)save_config();

                    ret = keypad_readln(keybuf, sizeof(keybuf), true);
                    if (ret < 0)
                      {
                        return ret;
                      }
                  }
                led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                break;
              case '2':          // 震动阀值
                led1_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                ret = keypad_readln(keybuf, sizeof(keybuf), false);
                if (ret < 0)
                  {
                    return ret;
                  }
                if (strcmp(keybuf, CONFIG_ILOCK_SETTING_PIN) == 0)
                  {
                    g_shock_resistor_sampled = true;

                    ret = keypad_readln(keybuf, sizeof(keybuf), true);
                    if (ret < 0)
                      {
                        return ret;
                      }
                    g_shock_resistor_sampled = false;

                    (void)save_config();
                  }
                led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                break;
              case '3':          // 上锁检测
                led1_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                ret = keypad_readln(keybuf, sizeof(keybuf), false);
                if (ret < 0)
                  {
                    return ret;
                  }
                if (strcmp(keybuf, CONFIG_ILOCK_SETTING_PIN) == 0)
                  {
                    g_min_infra_red = INT32_MAX;
                    g_max_infra_red = 0;

                    g_infra_red_sampled = 1;

                    ret = keypad_readln(keybuf, sizeof(keybuf), true);
                    if (ret < 0)
                      {
                        return ret;
                      }

                    g_infra_red_sampled = 0;

                    config->infra_red_threshold = (g_min_infra_red + g_max_infra_red) / 2;
                    (void)save_config();
                  }
                led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                break;
              case '4':          // 光感阀值
                led1_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
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

                    config->photo_resistor_threshold = atoi(keybuf);

                    (void)save_config();
                  }
                led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                break;
              case '5':          // 主机IP设置
                led1_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
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

                    config->hostaddr.s_addr = netlib_formataddr(keybuf);
                    netlib_sethostaddr(CONFIG_ILOCK_IFNAME, &config->hostaddr);

                    (void)save_config();
                  }
                led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                break;
              case '6':          // 网络掩码
                led1_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
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

                    config->netmask.s_addr = netlib_formataddr(keybuf);
                    netlib_setnetmask(CONFIG_ILOCK_IFNAME, &config->netmask);

                    (void)save_config();
                  }
                led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                break;
              case '7':          // 网关
                led1_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
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

                    config->dripaddr.s_addr = netlib_formataddr(keybuf);
                    netlib_setdraddr(CONFIG_ILOCK_IFNAME, &config->dripaddr);

                    (void)save_config();
                  }
                led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                break;
              case '8':          // 服务器ip
                led1_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_GREEN, UINT32_MAX, 0);
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

                    config->svraddr.s_addr = netlib_formataddr(keybuf);

                    (void)save_config();
                  }
                led1_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led2_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                led3_op(INDC_ALWAYS, IND_NONE, UINT32_MAX, 0);
                break;
              case ':':
                up_systemreset();
                break;
            }
        }
      else
        {
          if (g_unlock_step == 3)
            {
              authorize(keybuf);
            }
          else
            {
              g_magnet_delay = CONFIG_MAGNET_DELAY;
            }
        }
    }
}

static int net_task(int argc, char *argv[])
{
  int ret;
  int sockfd;
  FAR struct socket *psock;
//  int flags;
//  struct timeval tv;
  struct sockaddr_in svraddr;


  while(true)
    {
      sockfd = socket(PF_INET, SOCK_STREAM, 0);
      if (sockfd < 0)
        {
          ret = -errno;
          ndbg("socket failure %d\n", ret);
          goto errout;
        }

      psock = sockfd_socket(sockfd);
      if (!psock)
        {
          ret = -EFAULT;
          nlldbg("Failed to convert sockfd=%d to a socket structure\n", sockfd);
          goto errout_with_sockfd;
        }

      ret = net_clone(psock, &g_psock);
      if (ret < 0)
        {
          nlldbg("net_clone failed: %d\n", ret);
          goto errout;
        }

      psock_close(psock);

      /*
            flags = fcntl(g_sockfd, F_GETFL, 0);
            if (flags == -1)
              {
                ndbg("fcntl(F_GETFL) failed: %d\n", -errno);
                goto errout_with_socket;
              }

            ndbg("fcntl(F_GETFL) return: %d\n", flags);

            if (fcntl(g_sockfd, F_SETFL, flags & ~O_NDELAY) < 0)
              {
                ndbg("fcntl(O_NDELAY) failed: %d\n", -errno);
                goto			errout_with_socket;
              }
      */

      /*    tv.tv_sec  = 10;
          tv.tv_usec = 0;

          (void)setsockopt(g_sockfd, SOL_SOCKET, SO_RCVTIMEO, (FAR const void *)&tv,
                           sizeof(struct timeval));
          (void)setsockopt(g_sockfd, SOL_SOCKET, SO_SNDTIMEO, (FAR const void *)&tv,
                           sizeof(struct timeval));
      */

      svraddr.sin_family      = AF_INET;
      svraddr.sin_port        = HTONS(config->svrport);
      svraddr.sin_addr.s_addr = config->svraddr.s_addr;
      ret = psock_connect(&g_psock, (struct sockaddr*)&svraddr, sizeof(struct sockaddr_in));
      if (ret < 0)
        {
          ret = -errno;
          ndbg("connect failure: %d\n", ret);
          goto errout_with_g_psock;
        }

      ret = protocal_send_connect();
      if (ret < 0)
        {
          ret = -errno;
          goto errout_with_g_psock;
        }

      g_connected = true;

      while(protocal_recv() == OK);

errout_with_g_psock:
      g_connected = false;
      psock_close(&g_psock);
      led2_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(200));
      sleep(10);
    }
errout:
  return ret;
errout_with_sockfd:
  close(sockfd);
  return ret;
}

static int log_task(int argc, char *argv[])
{
  while(true)
    {
      sleep(60);
      auth_send_history_log_to_net();
      auth_send_history_alert_to_net();
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ilock_main(int argc, char *argv[])
{
  int ret;
  pid_t scan_pid;
  pid_t unlock_pid;
  pid_t net_pid;
  pid_t log_pid;

  mkdir(CONFIG_ALT_FILE_PATH, 0777);
  mkdir(CONFIG_LOG_FILE_PATH, 0777);

  ret = led_init();
  if (ret < 0)
    {
      goto errout;
    }

  ret = buzzer_init();
  if (ret < 0)
    {
      goto errout;
    }

  ret = gpio_init();
  if (ret < 0)
    {
      goto errout;
    }

  ret = config_init();
  if (ret < 0)
    {
      goto errout;
    }
  load_config();

  ret = netlib_setmacaddr(CONFIG_ILOCK_IFNAME, config->macaddr);
  if (ret < 0)
    {
      ret = -errno;
      ilockdbg("netlib_setmacaddr %s failed: %d\n", CONFIG_ILOCK_IFNAME, ret);
      goto errout;
    }

  ret = netlib_sethostaddr(CONFIG_ILOCK_IFNAME, &config->hostaddr);
  if (ret < 0)
    {
      ret = -errno;
      ilockdbg("netlib_sethostaddr %s failed: %d\n", CONFIG_ILOCK_IFNAME, ret);
      goto errout;
    }

  ret = netlib_setnetmask(CONFIG_ILOCK_IFNAME, &config->netmask);
  if (ret < 0)
    {
      ret = -errno;
      ilockdbg("netlib_setnetmask %s failed: %d\n", CONFIG_ILOCK_IFNAME, ret);
      goto errout;
    }

  ret = netlib_setdraddr(CONFIG_ILOCK_IFNAME, &config->dripaddr);
  if (ret < 0)
    {
      ret = -errno;
      ilockdbg("netlib_setdraddr %s failed: %d\n", CONFIG_ILOCK_IFNAME, ret);
      goto errout;
    }

  ret = ipconflict_init();
  if (ret < 0)
    {
      goto errout;
    }

  led1_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(200));
  led2_op(INDC_TWINKLE, IND_GREEN, SEC2TICK(3), MSEC2TICK(200));
  led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(200));
  buzzer_op(INDC_TWINKLE, IND_ON, SEC2TICK(3), MSEC2TICK(200));

  net_pid = task_create("iLockNet", 50, CONFIG_NET_TASK_STACKSIZE, net_task, NULL);
  if (net_pid < 0)
    {
      ret = -errno;
      ilockdbg("iLockNet task_create failed: %d\n", ret);
      goto errout;
    }

  log_pid = task_create("iLockLog", 50, CONFIG_LOG_TASK_STACKSIZE, log_task, NULL);
  if (log_pid < 0)
    {
      ret = -errno;
      ilockdbg("iLockLog task_create failed: %d\n", ret);
      goto errout;
    }

  unlock_pid = task_create("iLockUnlock", 50, CONFIG_UNLOCK_TASK_STACKSIZE, unlock_task, NULL);
  if (unlock_pid < 0)
    {
      ret = -errno;
      ilockdbg("iLockUnlock task_create failed: %d\n", ret);
      goto errout;
    }

  scan_pid = task_create("iLockScan", 50, CONFIG_SCAN_TASK_STACKSIZE, scan_task, NULL);
  if (scan_pid < 0)
    {
      ret = -errno;
      ilockdbg("iLockScan task_create failed: %d\n", ret);
      goto errout;
    }



  printf("Press any key to exit ......\n");
  getchar();

errout:

  if (scan_pid > 0)
    {
      task_delete(scan_pid);
    }

  if (unlock_pid > 0)
    {
      task_delete(unlock_pid);
    }

  if (log_pid > 0)
    {
      task_delete(log_pid);
    }

  if (net_pid > 0)
    {
      task_delete(net_pid);
    }

  ipconflict_deinit();

  config_deinit();

  gpio_deinit();
  buzzer_deinit();
  led_deinit();
  return ret;
}
