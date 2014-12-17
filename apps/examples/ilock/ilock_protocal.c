/****************************************************************************
 * examples/ilock/protocal.c
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
#include <version.h>

#include <nuttx/systemreset.h>
#include <nuttx/clock.h>
#include <nuttx/gpio/indicator.h>

#include "ilock_debug.h"
#include "ilock_protocal.h"
#include "ilock_authorize.h"
#include "ilock_main.h"

#include "config_lib.h"
#include "led_lib.h"
#include "buzzer_lib.h"
#include "file_lib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

const size_t protocal_size[LAST_CATEGORY + 1] = {HEART_BEAT_CATEGORY_RECV_SIZE, LOG_CATEGORY_RECV_SIZE,
                                                 DOWNLOAD_PUBKEY_CATEGORY_RECV_SIZE, CLEAR_PUBKEY_CATEGORY_RECV_SIZE,
                                                 ASSIGN_SN_CATEGORY_RECV_SIZE, ASSIGN_HOSTADDR_CATEGORY_RECV_SIZE,
                                                 ASSIGN_NETMASK_CATEGORY_RECV_SIZE, ASSIGN_DRIPADDR_CATEGORY_RECV_SIZE,
                                                 ASSIGN_SVRADDR_CATEGORY_RECV_SIZE, ASSIGN_MACADDR_CATEGORY_RECV_SIZE,
                                                 CLEAR_ALL_PUBKEY_CATEGORY_RECV_SIZE, SOFT_RESET_CATEGORY_RECV_SIZE,
                                                 UNLOCK_CATEGORY_RECV_SIZE, UPLOAD_PUBKEY_CATEGORY_RECV_SIZE,
                                                 REMOTE_AUTHORIZE_CATEGORY_RECV_SIZE, TIME_SYNC_CATEGORY_RECV_SIZE,
                                                 ALERT_CATEGORY_RECV_SIZE, CONNECT_CATEGORY_RECV_SIZE,
                                                 TIME_VIEW_CATEGORY_RECV_SIZE, SENSOR_VIEW_CATEGORY_RECV_SIZE,
                                                 ASSIGN_SHOCK_RESISTOR_THRESHOLD_CATEGORY_RECV_SIZE,
                                                 ASSIGN_INFRA_RED_THRESHOLD_CATEGORY_RECV_SIZE,
                                                 ASSIGN_PHOTO_RESISTOR_THRESHOLD_CATEGORY_RECV_SIZE,
                                                 DOWNLOAD_FIRMWARE_CATEGORY_RECV_SIZE,
                                                 CRC32_FIRMWARE_CATEGORY_RECV_SIZE, UPDATE_FIRMWARE_CATEGORY_RECV_SIZE,
                                                 VIEW_NET_ADDR_CATEGORY_RECV_SIZE,
                                                 CLEAR_ALL_PUBKEY_IN_GROUP_CATEGORY_RECV_SIZE, VERSION_VIEW_CATEGORY_RECV_SIZE,
                                                 DEFINE_CONFIG_PASSWORD_CATEGORY_RECV_SIZE,
                                                 REMOTE_AUTHORIZE_WITH_PUBKEY_CATEGORY_RECV_SIZE,
                                                 LOG_EX_CATEGORY_RECV_SIZE
                                                };

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void protocal_make_magic(struct protocal_s *p)
{
  p->head.magic[0] = MAGIC_ONE;
  p->head.magic[1] = MAGIC_TWO;
  p->head.magic[2] = MAGIC_THREE;
  p->head.magic[3] = MAGIC_FOUR;
}

int protocal_send_heart_beat(int sockfd)
{
  int ret = -EIO;
  ssize_t nsent;
  struct protocal_s p;

  if (sockfd == 0)
    {
      ret = -ENODEV;
      goto errout;
    }

  protocal_make_magic(&p);

  p.head.category = HEART_BEAT_CATEGORY;
  p.head.serial_no = config->serial_no;

  nsent = send(sockfd, &p, HEART_BEAT_CATEGORY_SEND_SIZE, 0);
  if (nsent < 0)
    {
      ret = -errno;
      ilockdbg("protocal_send_heart_beat: send failed: %d\n", ret);
      goto errout;
    }
  else if (nsent != HEART_BEAT_CATEGORY_SEND_SIZE)
    {
      ilockdbg("protocal_send_heart_beat: Bad send length=%d: %d of \n",
               nsent, HEART_BEAT_CATEGORY_SEND_SIZE);
      goto errout;
    }

  ret = OK;
errout:
  return ret;
}

int protocal_send_connect(int sockfd)
{
  int ret = -EIO;
  ssize_t nsent;
  struct protocal_s p;

  if (sockfd == 0)
    {
      ret = -ENODEV;
      goto errout;
    }

  protocal_make_magic(&p);

  p.head.category = CONNECT_CATEGORY;
  p.head.serial_no = config->serial_no;

  p.body.connect_category.connect_knl_version = version();
  p.body.connect_category.connect_app_version = CONFIG_EXAMPLES_ILOCK_VERSION;

  nsent = send(sockfd, &p, CONNECT_CATEGORY_SEND_SIZE, 0);
  if (nsent < 0)
    {
      ret = -errno;
      ilockdbg("protocal_send_connect: send failed: %d\n", ret);
      goto errout;
    }
  else if (nsent != CONNECT_CATEGORY_SEND_SIZE)
    {
      ilockdbg("protocal_send_connect: Bad send length=%d: %d of \n",
               nsent, CONNECT_CATEGORY_SEND_SIZE);
      goto errout;
    }

  ret = OK;
errout:
  return ret;
}

int protocal_send_alert(int sockfd, uint32_t serial_no, uint8_t alert_type, uint8_t tm[6])
{
  int ret = -EIO;
  ssize_t nsent;
  struct protocal_s p;

  if (sockfd == 0)
    {
      ret = -ENODEV;
      goto errout;
    }

  protocal_make_magic(&p);

  p.head.category = ALERT_CATEGORY;
  p.head.serial_no = config->serial_no;

  p.body.alert_category.alert_type = alert_type;
  memcpy(p.body.alert_category.alert_time, tm, 6);

  nsent = send(sockfd, &p, ALERT_CATEGORY_SEND_SIZE, 0);
  if (nsent < 0)
    {
      ret = -errno;
      ilockdbg("protocal_send_alert: send failed: %d\n", ret);
      goto errout;
    }
  else if (nsent != ALERT_CATEGORY_SEND_SIZE)
    {
      ilockdbg("protocal_send_alert: Bad send length=%d: %d of \n",
               nsent, ALERT_CATEGORY_SEND_SIZE);
      goto errout;
    }

  ret = OK;
errout:
  return ret;
}

int protocal_send_log(int sockfd, int32_t serial_no, bool *log_group, uint8_t *log_pubkey, uint8_t flag, uint8_t tm[6])
{
  int ret = -EIO;
  int i;
  ssize_t nsent;
  struct protocal_s p;

  if (sockfd == 0)
    {
      ret = -ENODEV;
      goto errout;
    }

  protocal_make_magic(&p);

  p.head.category = LOG_EX_CATEGORY;
  p.head.serial_no = config->serial_no;

  for(i = 0; i < CONFIG_GROUP_SIZE; i++)
    {
      p.body.log_ex_category.log_group[i] = log_group[i];
    }

  memcpy(p.body.log_ex_category.log_pubkey, log_pubkey, CONFIG_PUBKEY_SIZE);
  p.body.log_ex_category.flag = flag;
  memcpy(p.body.log_ex_category.log_time, tm, 6);

  nsent = send(sockfd, &p, LOG_CATEGORY_SEND_SIZE, 0);
  if (nsent < 0)
    {
      ret = -errno;
      ilockdbg("protocal_send_log: send failed: %d\n", ret);
      goto errout;
    }
  else if (nsent != LOG_CATEGORY_SEND_SIZE)
    {
      ilockdbg("protocal_send_log: Bad send length=%d: %d of \n",
               nsent, LOG_CATEGORY_SEND_SIZE);
      goto errout;
    }

  ret = OK;
errout:
  return ret;
}

static int protocal_send_time_view(int sockfd)
{
  int ret = -EIO;
  ssize_t nsent;
  struct timespec ts;
  struct tm *tm;

  struct protocal_s p;

  if (sockfd == 0)
    {
      ret = -ENODEV;
      goto errout;
    }

  protocal_make_magic(&p);

  p.head.category = TIME_VIEW_CATEGORY;
  p.head.serial_no = config->serial_no;

  clock_gettime(CLOCK_REALTIME, &ts);
  tm = gmtime(&ts.tv_sec);

  p.body.time_view_category.view_time[0] = tm->tm_year;
  p.body.time_view_category.view_time[1] = tm->tm_mon;
  p.body.time_view_category.view_time[2] = tm->tm_mday;
  p.body.time_view_category.view_time[3] = tm->tm_hour;
  p.body.time_view_category.view_time[4] = tm->tm_min;
  p.body.time_view_category.view_time[5] = tm->tm_sec;

  nsent = send(sockfd, &p, TIME_VIEW_CATEGORY_SEND_SIZE, 0);
  if (nsent < 0)
    {
      ret = -errno;
      ilockdbg("protocal_send_time_view: send failed: %d\n", ret);
      goto errout;
    }
  else if (nsent != TIME_VIEW_CATEGORY_SEND_SIZE)
    {
      ilockdbg("protocal_send_time_view: Bad send length=%d: %d of \n",
               nsent, TIME_VIEW_CATEGORY_SEND_SIZE);
      goto errout;
    }

  ret = OK;
errout:
  return ret;
}

static int protocal_send_sensor_view(int sockfd)
{
  int ret = -EIO;
  ssize_t nsent;
  struct protocal_s p;

  if (sockfd == 0)
    {
      ret = -ENODEV;
      goto errout;
    }

  protocal_make_magic(&p);

  p.head.category = SENSOR_VIEW_CATEGORY;
  p.head.serial_no = config->serial_no;

  p.body.sensor_view_category.view_shock_resistor_threshold = config->shock_resistor_threshold;
  p.body.sensor_view_category.view_infra_red_threshold = config->infra_red_threshold;
  p.body.sensor_view_category.view_photo_resistor_threshold = config->photo_resistor_threshold;
  p.body.sensor_view_category.view_battery_voltage = 0;
  p.body.sensor_view_category.view_power_voltage = 0;

  nsent = send(sockfd, &p, SENSOR_VIEW_CATEGORY_SEND_SIZE, 0);
  if (nsent < 0)
    {
      ret = -errno;
      ilockdbg("protocal_send_sensor_view: send failed: %d\n", ret);
      goto errout;
    }
  else if (nsent != SENSOR_VIEW_CATEGORY_SEND_SIZE)
    {
      ilockdbg("protocal_send_sensor_view: Bad send length=%d: %d of \n",
               nsent, SENSOR_VIEW_CATEGORY_SEND_SIZE);
      goto errout;
    }

  ret = OK;
errout:
  return ret;
}

static int protocal_send_net_addr_view(int sockfd)
{
  int ret = -EIO;
  ssize_t nsent;
  struct protocal_s p;

  if (sockfd == 0)
    {
      ret = -ENODEV;
      goto errout;
    }

  protocal_make_magic(&p);

  p.head.category = VIEW_NET_ADDR_CATEGORY;
  p.head.serial_no = config->serial_no;

  p.body.view_net_addr_category.view_hostaddr = config->hostaddr.s_addr;
  p.body.view_net_addr_category.view_netmask = config->netmask.s_addr;
  p.body.view_net_addr_category.view_dripaddr = config->dripaddr.s_addr;
  p.body.view_net_addr_category.view_svraddr = config->svraddr.s_addr;
  memcpy(p.body.view_net_addr_category.view_macaddr, config->macaddr, IFHWADDRLEN);

  nsent = send(sockfd, &p, VIEW_NET_ADDR_CATEGORY_SEND_SIZE, 0);
  if (nsent < 0)
    {
      ret = -errno;
      ilockdbg("protocal_send_net_addr_view: send failed: %d\n", ret);
      goto errout;
    }
  else if (nsent != VIEW_NET_ADDR_CATEGORY_SEND_SIZE)
    {
      ilockdbg("protocal_send_net_addr_view: Bad send length=%d: %d of \n",
               nsent, VIEW_NET_ADDR_CATEGORY_SEND_SIZE);
      goto errout;
    }

  ret = OK;
errout:
  return ret;
}

/*
int protocal_send_pubkey(int sockfd, uint8_t pubkey[CONFIG_PUBKEY_SIZE])
{
  int ret = -ENODATA;
  ssize_t nsent;
  struct protocal_s p;

  protocal_make_magic(&p);

  p.head.category = UPLOAD_PUBKEY_CATEGORY;

  p.head.serial_no = config->serial_no;
  memcpy(p.body.upload_pubkey_category.upload_pubkey, pubkey, CONFIG_PUBKEY_SIZE);

  nsent = send(sockfd, &p, UPLOAD_PUBKEY_CATEGORY_SEND_SIZE, 0);
  if (nsent < 0)
    {
      ret = -errno;
      ilockdbg("protocal_send_pubkey: send failed: %d\n", ret);
      goto errout;
    }
  else if (nsent != UPLOAD_PUBKEY_CATEGORY_SEND_SIZE)
    {
      ilockdbg("protocal_send_pubkey: Bad send length=%d: %d of \n",
               nsent, UPLOAD_PUBKEY_CATEGORY_SEND_SIZE);
      goto errout;
    }

  ret = OK;
errout:
  return ret;
}
*/

static int protocal_send_ok(int sockfd, uint8_t category)
{
  int ret = -EIO;
  ssize_t nsent;
  struct protocal_s p;

  if (sockfd == 0)
    {
      ret = -ENODEV;
      goto errout;
    }

  protocal_make_magic(&p);

  p.head.category = category;
  p.head.serial_no = config->serial_no;

  nsent = send(sockfd, &p, PROTOCAL_HEAD_SIZE, 0);
  if (nsent < 0)
    {
      ret = -errno;
      ilockdbg("protocal_send_ok: send failed: %d\n", ret);
      goto errout;
    }
  else if (nsent != PROTOCAL_HEAD_SIZE)
    {
      ilockdbg("protocal_send_ok: Bad send length=%d: %d of \n",
               nsent, PROTOCAL_HEAD_SIZE);
      goto errout;
    }

  ret = OK;
errout:
  return ret;
}

static int protocal_send_download_firmware_ok(int sockfd, uint32_t firmware_crc32, uint32_t firmware_len,  uint32_t firmware_pos)
{
  int ret = -EIO;
  ssize_t nsent;
  struct protocal_s p;

  if (sockfd == 0)
    {
      ret = -ENODEV;
      goto errout;
    }

  protocal_make_magic(&p);

  p.head.category = DOWNLOAD_FIRMWARE_CATEGORY;
  p.head.serial_no = config->serial_no;

  p.body.download_firmware_category.firmware_crc32 = firmware_crc32;
  p.body.download_firmware_category.firmware_len = firmware_len;
  p.body.download_firmware_category.firmware_pos = firmware_pos;

  nsent = send(sockfd, &p, DOWNLOAD_FIRMWARE_CATEGORY_SEND_SIZE, 0);
  if (nsent < 0)
    {
      ret = -errno;
      ilockdbg("protocal_send_download_firmware_ok: send failed: %d\n", ret);
      goto errout;
    }
  else if (nsent != DOWNLOAD_FIRMWARE_CATEGORY_SEND_SIZE)
    {
      ilockdbg("protocal_send_download_firmware_ok: Bad send length=%d: %d of \n",
               nsent, DOWNLOAD_FIRMWARE_CATEGORY_SEND_SIZE);
      goto errout;
    }

  ret = OK;
errout:
  return ret;
}

static int protocal_send_crc32_firmware(int sockfd, uint8_t crc32_firmware_success)
{
  int ret = -EIO;
  ssize_t nsent;
  struct protocal_s p;

  if (sockfd == 0)
    {
      ret = -ENODEV;
      goto errout;
    }

  protocal_make_magic(&p);

  p.head.category = CRC32_FIRMWARE_CATEGORY;
  p.head.serial_no = config->serial_no;
  p.body.crc32_firmware_category.crc32_success = crc32_firmware_success;

  nsent = send(sockfd, &p, CRC32_FIRMWARE_CATEGORY_SEND_SIZE, 0);
  if (nsent < 0)
    {
      ret = -errno;
      ilockdbg("protocal_send_crc32_firmware: send failed: %d\n", ret);
      goto errout;
    }
  else if (nsent != CRC32_FIRMWARE_CATEGORY_SEND_SIZE)
    {
      ilockdbg("protocal_send_crc32_firmware: Bad send length=%d: %d of \n",
               nsent, CRC32_FIRMWARE_CATEGORY_SEND_SIZE);
      goto errout;
    }

  ret = OK;
errout:
  return ret;
}

static int protocal_send_version(int sockfd)
{
  int ret = -EIO;
  ssize_t nsent;
  struct protocal_s p;

  if (sockfd == 0)
    {
      ret = -ENODEV;
      goto errout;
    }

  protocal_make_magic(&p);

  p.head.category = VERSION_VIEW_CATEGORY;
  p.head.serial_no = config->serial_no;

  p.body.version_view_category.view_knl_version = version();
  p.body.version_view_category.view_app_version = CONFIG_EXAMPLES_ILOCK_VERSION;

  nsent = send(sockfd, &p, VERSION_VIEW_CATEGORY_SEND_SIZE, 0);
  if (nsent < 0)
    {
      ret = -errno;
      ilockdbg("protocal_send_version: send failed: %d\n", ret);
      goto errout;
    }
  else if (nsent != VERSION_VIEW_CATEGORY_SEND_SIZE)
    {
      ilockdbg("protocal_send_version: Bad send length=%d: %d of \n",
               nsent, VERSION_VIEW_CATEGORY_SEND_SIZE);
      goto errout;
    }

  ret = OK;
errout:
  return ret;
}

int protocal_set_time(struct protocal_s * protocal)
{
  int ret = OK;
  struct timespec ts;
  struct tm tm;

  tm.tm_year = protocal->body.time_view_category.view_time[0];
  tm.tm_mon = protocal->body.time_view_category.view_time[1];
  tm.tm_mday = protocal->body.time_view_category.view_time[2];
  tm.tm_hour = protocal->body.time_view_category.view_time[3];
  tm.tm_min = protocal->body.time_view_category.view_time[4];
  tm.tm_sec = protocal->body.time_view_category.view_time[5];

  ts.tv_sec  = mktime(&tm);
  ts.tv_nsec = 0;

  ret = clock_settime(CLOCK_REALTIME, &ts);
  if (ret < 0)
    {
      ret = -errno;
      ilockdbg("protocal_set_time: clock_settime failed: %d\n", ret);
    }

  return ret;
}

static int protocal_deal(int sockfd, struct protocal_s * protocal)
{
  int ret = OK;
  int i;
  uint8_t pubkey[CONFIG_PUBKEY_SIZE];
  uint8_t pos;
  uint32_t firmware_crc32;
  uint32_t firmware_len;
  uint32_t firmware_pos;
  uint8_t *firmware_cache;
  uint32_t firmware_cache_len;

  if (protocal->head.magic[0] == MAGIC_ONE &&
      protocal->head.magic[1] == MAGIC_TWO &&
      protocal->head.magic[2] == MAGIC_THREE &&
      protocal->head.magic[3] == MAGIC_FOUR)
    {
//        ET0 = 0;
//        last_heart_beat = tick_count;
//        ET0 = 1;
      switch (protocal->head.category)
        {
          //            case HEART_BEAT_CATEGORY:
          //                break;
          case ASSIGN_SHOCK_RESISTOR_THRESHOLD_CATEGORY:
            config->shock_resistor_threshold = protocal->body.assign_shock_resistor_threshold_category.assign_shock_resistor_threshold;
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, ASSIGN_SHOCK_RESISTOR_THRESHOLD_CATEGORY);
            break;
          case ASSIGN_INFRA_RED_THRESHOLD_CATEGORY:
            config->infra_red_threshold = protocal->body.assign_infra_red_threshold_category.assign_infra_red_threshold;
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, ASSIGN_INFRA_RED_THRESHOLD_CATEGORY);
            break;
          case ASSIGN_PHOTO_RESISTOR_THRESHOLD_CATEGORY:
            config->photo_resistor_threshold = protocal->body.assign_photo_resistor_threshold_category.assign_photo_resistor_threshold;
            ret  = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, ASSIGN_PHOTO_RESISTOR_THRESHOLD_CATEGORY);
            break;
          case SENSOR_VIEW_CATEGORY:
            ret = protocal_send_sensor_view(sockfd);
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            break;
          case TIME_VIEW_CATEGORY:
            ret = protocal_send_time_view(sockfd);
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            break;
          case TIME_SYNC_CATEGORY:
            ret = protocal_set_time(protocal);
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, TIME_SYNC_CATEGORY);
            break;
          case REMOTE_AUTHORIZE_CATEGORY:
            memset(pubkey, 0, CONFIG_PUBKEY_SIZE);
            auth_time_out_check();
            if (!auth_this_time(pubkey) && (auth_init(), !auth_this_time(pubkey)))
              {
                led1_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(500));
                led3_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(500));
                buzzer_op(INDC_TWINKLE, IND_ON, SEC2TICK(1), MSEC2TICK(500));
              }
            else if (auth_need_more())
              {
                auth_set_auth_unlock(sockfd, pubkey);
                led1_op(INDC_TWINKLE, IND_GREEN, SEC2TICK(3), MSEC2TICK(500));
              }
            else
              {
                auth_send_log_to_disk_or_net(sockfd, LOG_AUTH_UNLOCK, pubkey);
                act_unlock();
              }
            (void)protocal_send_ok(sockfd, REMOTE_AUTHORIZE_CATEGORY);
            break;
          case REMOTE_AUTHORIZE_WITH_PUBKEY_CATEGORY_RECV_SIZE:
            auth_time_out_check();
            if (!auth_this_time(protocal->body.remote_authorize_with_pubkey_category.remote_authorize_pubkey) &&
                (auth_init(), !auth_this_time(protocal->body.remote_authorize_with_pubkey_category.remote_authorize_pubkey)))
              {
                led1_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(500));
                led3_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(500));
                buzzer_op(INDC_TWINKLE, IND_ON, SEC2TICK(1), MSEC2TICK(500));
              }
            else if (auth_need_more())
              {
                auth_set_auth_unlock(sockfd, pubkey);
                led1_op(INDC_TWINKLE, IND_GREEN, SEC2TICK(3), MSEC2TICK(500));
              }
            else
              {
                auth_send_log_to_disk_or_net(sockfd, LOG_AUTH_UNLOCK, pubkey);
                act_unlock();
              }
            (void)protocal_send_ok(sockfd, REMOTE_AUTHORIZE_CATEGORY);
          case UNLOCK_CATEGORY:
            memset(pubkey, 0, CONFIG_PUBKEY_SIZE);
            if (protocal->body.unlock_category.unlock_flag == NEED_CHECK_HALF_UNLOCK && (auth_time_out_check(), !auth_if_half_unlock()))
              {
                auth_set_temp_unlock(sockfd, pubkey);
                led1_op(INDC_TWINKLE, IND_GREEN, SEC2TICK(3), MSEC2TICK(500));
              }
            else
              {
                auth_send_log_to_disk_or_net(sockfd, LOG_TEMP_UNLOCK, pubkey);
                act_unlock();
                led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
              }
            (void)protocal_send_ok(sockfd, UNLOCK_CATEGORY);
            break;
          case SOFT_RESET_CATEGORY:
            up_systemreset();
            break;
          case ASSIGN_SN_CATEGORY:
            config->serial_no = protocal->body.assign_sn_category.assigned_sn;
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, ASSIGN_SN_CATEGORY);
            break;
          case ASSIGN_HOSTADDR_CATEGORY:
            config->hostaddr.s_addr = protocal->body.assign_hostaddr_category.hostaddr;
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, ASSIGN_HOSTADDR_CATEGORY);
            break;
          case ASSIGN_NETMASK_CATEGORY:
            config->netmask.s_addr = protocal->body.assign_netmask_category.netmask;
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, ASSIGN_NETMASK_CATEGORY);
            break;
          case ASSIGN_DRIPADDR_CATEGORY:
            config->dripaddr.s_addr = protocal->body.assign_dripaddr_cateory.dripaddr;
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, ASSIGN_DRIPADDR_CATEGORY);
            break;
          case ASSIGN_SVRADDR_CATEGORY:
            config->svraddr.s_addr = protocal->body.assign_svraddr_category.svraddr;
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, ASSIGN_SVRADDR_CATEGORY);
            break;
          case ASSIGN_MAC_CATEGORY:
            memcpy(config->macaddr, protocal->body.assign_macaddr_category.macaddr,  IFHWADDRLEN);
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, ASSIGN_SVRADDR_CATEGORY);
            break;
          case UPLOAD_PUBKEY_CATEGORY:
//		  cancelled
//         ret = protocal_send_pubkey(sockfd, pubkey);
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            break;
          case DOWNLOAD_PUBKEY_CATEGORY:
            pos = 0xff;
            for (i = 0; i < CONFIG_GROUP_SIZE; i++)
              {
                if (memcmp(config->keyslots[i].pubkey, CONFIG_PUBKEY_DEF_VALUE, CONFIG_PUBKEY_SIZE) == 0)
                  {
                    if (pos > i)
                      pos = i;

                  }
                else if (memcmp(config->keyslots[i].pubkey, protocal->body.download_pubkey_category.download_pubkey, CONFIG_PUBKEY_SIZE) == 0)
                  {
                    pos = i;
                  }
              }
            if (pos == 0xff)
              pos = 0;

            config->keyslots[pos].group[protocal->body.download_pubkey_category.download_group_no] = true;
            memcpy(config->keyslots[pos].pubkey, protocal->body.download_pubkey_category.download_pubkey, CONFIG_PUBKEY_SIZE);
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, DOWNLOAD_PUBKEY_CATEGORY);
            break;
          case CLEAR_PUBKEY_CATEGORY:
            for (i = 0; i < CONFIG_GROUP_SIZE; i++)
              {
                if (memcmp(config->keyslots[i].pubkey, protocal->body.clear_pubkey_category.clear_pubkey, CONFIG_PUBKEY_SIZE) == 0)
                  {
                    memset(config->keyslots[i].pubkey, 0xff, sizeof(config->keyslots[i].pubkey));
                    memset(config->keyslots[i].group, 0, sizeof(config->keyslots[i].group));
                  }
              }
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, CLEAR_PUBKEY_CATEGORY);
            break;
          case CLEAR_ALL_PUBKEY_CATEGORY:
            for (i = 0; i < CONFIG_GROUP_SIZE; i++)
              {
                memset(config->keyslots[i].pubkey, 0xff, sizeof(config->keyslots[i].pubkey));
                memset(config->keyslots[i].group, false, sizeof(config->keyslots[i].group));
              }
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, CLEAR_ALL_PUBKEY_CATEGORY);
            break;
          case DOWNLOAD_FIRMWARE_CATEGORY:
            firmware_crc32 = protocal->body.download_firmware_category.firmware_crc32;
            firmware_len = protocal->body.download_firmware_category.firmware_len;
            firmware_pos = protocal->body.download_firmware_category.firmware_pos;
            firmware_cache = protocal->body.download_firmware_category.firmware;

            firmware_cache_len = firmware_len - firmware_pos;
            if (firmware_cache_len > FIRMWARE_CACHE_SIZE)
              {
                ret = filewrite(CONFIG_FIRMWARE_BIN_PATH, firmware_pos, firmware_cache, FIRMWARE_CACHE_SIZE);
                if (ret == OK)
                  {
                    ret =	protocal_send_download_firmware_ok(sockfd, firmware_crc32, firmware_len, firmware_pos + FIRMWARE_CACHE_SIZE);
                  }
              }
            else
              {
                ret = filewrite(CONFIG_FIRMWARE_BIN_PATH, firmware_pos, firmware_cache, firmware_cache_len);
                if (ret == OK)
                  {
                    ret =	protocal_send_download_firmware_ok(sockfd, firmware_crc32, firmware_len, firmware_pos + firmware_cache_len);
                    (void)protocal_send_crc32_firmware(sockfd, firmware_crc32 == filecrc32(CONFIG_FIRMWARE_BIN_PATH) ? 1 : 0);
                  }
              }

            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            break;
          case UPDATE_FIRMWARE_CATEGORY:
            ret = filetouch(CONFIG_UPDATE_FM_FLAG_PATH);
            if (ret == OK)
              {
                up_systemreset();
              }
            break;
          case VIEW_NET_ADDR_CATEGORY:
            ret =	protocal_send_net_addr_view(sockfd);
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            break;
          case CLEAR_ALL_PUBKEY_IN_GROUP_CATEGORY:
            for (i = 0; i <= CONFIG_GROUP_SIZE; i++)
              {
                if (config->keyslots[i].group[protocal->body.clear_all_pubkey_in_group_category.clear_group_no])
                  {
                    memset(config->keyslots[i].pubkey, 0xff, sizeof(config->keyslots[i].pubkey));
                    memset(config->keyslots[i].group, false, sizeof(config->keyslots[i].group));
                  }
              }
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, CLEAR_ALL_PUBKEY_IN_GROUP_CATEGORY);
            break;
          case VERSION_VIEW_CATEGORY:
            ret = protocal_send_version(sockfd);
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            break;
          case DEFINE_CONFIG_PASSWORD_CATEGORY:
            memcpy(config->config_password, protocal->body.define_config_password_category.config_password, 6);
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, DEFINE_CONFIG_PASSWORD_CATEGORY);
            break;
          default:
            ret = -EBADMSG;
            break;
        }
    }
  else
    {
      ret = -EBADMSG;
    }
  return ret;
}

int protocal_recv(int sockfd)
{
  int ret;
  uint8_t recv_buf[sizeof(struct protocal_s)];
  ssize_t recv_pos, recv_len;
  size_t recv_size;

  recv_pos = 0;
  while (recv_pos < PROTOCAL_HEAD_SIZE)
    {
      recv_len = recv(sockfd, &recv_buf[recv_pos], PROTOCAL_HEAD_SIZE - recv_pos, 0);

      if (recv_len < 0)
        {
          ret = -errno;
          ilockdbg("protocal_recv: protocal head recv failed: %d\n", ret);
          goto errout;
        }
      else if (recv_len == 0)
        {
          ret = -EHOSTDOWN;
          ilockdbg("protocal_recv: in protocal head recv, the server closed the connection\n");
          goto errout;
        }
      recv_pos += recv_len;
    }

  recv_size = protocal_size[((struct protocal_s *)recv_buf)->head.category];

  recv_pos = PROTOCAL_HEAD_SIZE;
  while (recv_pos < recv_size)
    {
      recv_len = recv(sockfd, &recv_buf[recv_pos], recv_size - recv_pos, 0);

      if (recv_len < 0)
        {
          ret = -errno;
          ilockdbg("protocal_recv: protocal body recv failed: %d\n", ret);
          goto errout;
        }
      else if (recv_len == 0)
        {
          ret = -EHOSTDOWN;
          ilockdbg("protocal_recv: in protocal body recv, the server closed the connection\n");
          goto errout;
        }
      recv_pos += recv_len;
    };

  ret = protocal_deal(sockfd, (struct protocal_s *) recv_buf);
  if (ret < 0)
    {
      ilockdbg("protocal_recv: protocal deal failed: %d\n", ret);
    }
errout:
  return ret;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/
