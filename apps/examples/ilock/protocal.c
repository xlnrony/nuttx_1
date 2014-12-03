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

#include "protocal.h"

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
                                                 ASSIGN_SN_CATEGORY_RECV_SIZE, ASSIGN_HOSTIP_CATEGORY_RECV_SIZE,
                                                 ASSIGN_NETMASK_CATEGORY_RECV_SIZE, ASSIGN_GATEWAY_CATEGORY_RECV_SIZE,
                                                 ASSIGN_SERVERIP_CATEGORY_RECV_SIZE, ASSIGN_MAC_CATEGORY_RECV_SIZE,
                                                 CLEAR_ALL_PUBKEY_CATEGORY_RECV_SIZE, SOFT_RESET_CATEGORY_RECV_SIZE,
                                                 UNLOCK_CATEGORY_RECV_SIZE, UPLOAD_PUBKEY_CATEGORY_RECV_SIZE,
                                                 REMOTE_AUTHORIZE_CATEGORY_RECV_SIZE, TIME_SYNC_CATEGORY_RECV_SIZE,
                                                 ALERT_CATEGORY_RECV_SIZE, CONNECT_CATEGORY_RECV_SIZE,
                                                 TIME_VIEW_CATEGORY_RECV_SIZE, SENSOR_VIEW_CATEGORY_RECV_SIZE,
                                                 ASSIGN_SHOCK_CHECK_VOLTAGE_CATEGORY_RECV_SIZE,
                                                 ASSIGN_LOCK_CHECK_VOLTAGE_CATEGORY_RECV_SIZE,
                                                 ASSIGN_LIGHT_CHECK_VOLTAGE_CATEGORY_RECV_SIZE,
                                                 DOWNLOAD_FIRMWARE_CATEGORY_RECV_SIZE,
                                                 CRC32_FIRMWARE_CATEGORY_RECV_SIZE, UPDATE_FIRMWARE_CATEGORY_RECV_SIZE,
                                                 VIEW_NET_ADDR_CATEGORY_RECV_SIZE,
                                                 CLEAR_ALL_PUBKEY_IN_GROUP_CATEGORY_RECV_SIZE, VERSION_VIEW_CATEGORY_RECV_SIZE,
                                                 DEFINE_CONFIG_PASSWORD_CATEGORY_RECV_SIZE
                                                };

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void protocal_make_magic(struct protocal_s *p)
{
  p->head.magic[0] = MAGIC_ONE;
  p->head.magic[1] = MAGIC_TWO;
  p->head.magic[2] = MAGIC_THREE;
  p->head.magic[3] = MAGIC_FOUR;
}

int protocal_send_heart_beat(int sockfd)
{
  int ret;
  ssize_t nsent;
  struct protocal_s p = {0};

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
  int ret;
  ssize_t nsent;
  struct protocal_s p = {0};

  protocal_make_magic(&p);

  p.head.category = CONNECT_CATEGORY;
  p.head.serial_no = config->serial_no;

  p.body.connect_category.connet_knl_version = config.knl_version;
  p.body.connect_category.connect_app_version = config.app_version;

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

int protocal_send_alert(int sockfd, unsigned long serial_no, unsigned char alert_type, unsigned char time[6])
{
  int ret;
  ssize_t nsent;
  struct protocal_s p = {0};

  protocal_make_magic(&p);

  p.head.category = ALERT_CATEGORY;
  p.head.serial_no = SWAP32(serial_no);

  p.body.alert_category.alert_type = alert_type;
  memcpy(p.body.alert_category.alert_time, time, 6);

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

int protocal_send_log(int sockfd, unsigned long serial_no, unsigned char log_group_no, unsigned char *log_pubkey, unsigned char flag, unsigned char time[6])
{
  int ret = -ENODATA;
  ssize_t nsent;
  struct protocal_s p = {0};

  protocal_make_magic(&p);

  p.head.category = LOG_CATEGORY;
  p.head.serial_no = SWAP32(serial_no);

  p.body.log_category.log_group_no = log_group_no;
  memcpy(p.body.log_category.log_pubkey, log_pubkey, 128);
  p.body.log_category.flag = flag;
  memcpy(p.body.log_category.log_time, time, 6);

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

int protocal_send_time_view(int sockfd)
{
  int ret = -ENODATA;
  ssize_t nsent;
  struct timespec ts;
  struct tm *tm;

  struct protocal_s p = {0};

  protocal_make_magic(&p);

  p.head.category = TIME_VIEW_CATEGORY;
  p.head.serial_no = config.serial_no;

  clock_gettime(CLOCK_REALTIME, &ts);
  tm = gmtime(ts.tv_sec);

  p.body.time_view_category.view_time[0] = tm.tm_year;
  p.body.time_view_category.view_time[1] = tm.tm_mon;
  p.body.time_view_category.view_time[2] = tm.tm_mday;
  p.body.time_view_category.view_time[3] = tm.tm_hour;
  p.body.time_view_category.view_time[4] = tm.tm_min;
  p.body.time_view_category.view_time[5] = tm.tm_sec;

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

int protocal_send_sensor_view(int sockfd)
{
  int ret = -ENODATA;
  ssize_t nsent;
  struct protocal_s p = {0};

  protocal_make_magic(&p);

  p.head.category = SENSOR_VIEW_CATEGORY;
  p.head.serial_no = config.serial_no;

  p.body.sensor_view_category.view_shock_resistor_threshold = config.shock_resistor_threshold;
  p.body.sensor_view_category.view_infra_red_threshold = config.infra_red_threshold;
  p.body.sensor_view_category.view_photo_resistor_threshold = config.photo_resistor_threshold;
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

int protocal_send_net_addr_view(int sockfd)
{
  int ret = -ENODATA;
  ssize_t nsent;
  struct protocal_s p = {0};

  protocal_make_magic(&p);

  p.head.category = VIEW_NET_ADDR_CATEGORY;
  p.head.serial_no = config.serial_no;

  p.body.view_net_addr_category.view_hostaddr = config.hostaddr.s_addr;
  p.body.view_net_addr_category.view_netmask = config.netmask.s_addr;
  p.body.view_net_addr_category.view_dripaddr = config.dripaddr.s_addr;
  p.body.view_net_addr_category.view_svraddr = config.svraddr.s_addr;
  memcpy(p.body.view_net_addr_category.view_macaddr, config.macaddr, IFHWADDRLEN);

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

int protocal_send_pubkey(int sockfd, uint8_t pubkey[CONFIG_PUBKEY_SIZE])
{
  int ret = -ENODATA;
  ssize_t nsent;
  struct protocal_s p = {0};

  protocal_make_magic(&p);

  p->head.category = UPLOAD_PUBKEY_CATEGORY;

  p->head.serial_no = config.serial_no;
  memcpy(p->body.upload_pubkey_category.upload_pubkey, pubkey, CONFIG_PUBKEY_SIZE);

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

int protocal_send_ok(int sockfd, uint8_t category)
{
  int ret = -ENODATA;
  ssize_t nsent;
  struct protocal_s p = {0};

  protocal_make_magic(&p);

  p.head.category = category;
  p.head.serial_no = SWAP32(serial_no);

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

int protocal_send_download_firmware_ok(void)
{
  int ret = -ENODATA;
  ssize_t nsent;
  struct protocal_s p = {0};

  protocal_make_magic(&p);

  p.head.category = DOWNLOAD_FIRMWARE_CATEGORY;
  p.head.serial_no = config.serial_no;

//  p.body.download_firmware_category.firmware_crc32 = SWAP32(firmware_crc32);
//  p.body.download_firmware_category.firmware_len = SWAP32(firmware_len);
// p.body.download_firmware_category.firmware_pos = SWAP32(next_download_firmware_pos);

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

int protocal_send_crc32_firmware(int sockfd)
{
  int ret = -ENODATA;
  ssize_t nsent;
  struct protocal_s p = {0};

  protocal_make_magic(&p);

  p.head.category = CRC32_FIRMWARE_CATEGORY;
  p.head.serial_no = config.serial_no;
//  p.body.crc32_firmware_category.crc32_success = crc32_firmware_success;

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

int protocal_send_version(int sockfd)
{
  int ret = -ENODATA;
  ssize_t nsent;
  struct protocal_s p = {0};

  protocal_make_magic(&p);

  p.head.category = VERSION_VIEW_CATEGORY;
  p.head.serial_no = config.serial_no;

  p.body.version_view_category.view_knl_version = config.knl_version;
  p.body.version_view_category.view_app_version = config.app_version;

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

int protocal_set_time(struct protocal * protocal)
{
  int ret = OK;
  struct timespec ts;
  struct tm *tm;

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
    }*/

  return ret;
}

int protocal_deal(int sockfd, struct protocal * protocal)
{
  unsigned char i;
  unsigned char j;
  unsigned char pubkey[CONFIG_PUBKEY_SIZE];
  unsigned long l;
  bool flag;
  unsigned char pos;

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
            pubkey = {0};
//            memset(pubkey, 0, CONFIG_PUBKEY_SIZE);
            auth_time_out_check();
            if (!auth_this_time(pubkey) && (auth_init(), !auth_this_time(pubkey)))
              {
                led1_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(500));
                led3_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(500));
                buzzer_op(INDC_TWINKLE, IND_ON, SEC2TICK(1), MSEC2TICK(500));
              }
            else if (auth_need_more())
              {
                auth_set_auth_unlock();
                led1_op(INDC_TWINKLE, IND_GREEN, SEC2TICK(3), MSEC2TICK(500));
              }
            else
              {
                act_unlock(LOG_AUTH_UNLOCK);
              }
            (void)protocal_send_ok(sockfd, REMOTE_AUTHORIZE_CATEGORY);
            break;
          case REMOTE_AUTHORIZE_WITH_PUBKEY_CATEGORY_RECV_SIZE:
            auth_time_out_check();
            if (!auth_this_time(protocal->body.remote_authrize_with_pubkey_category.remote_authrize_pubkey) &&
                (auth_init(), !auth_this_time(protocal->body.remote_authrize_with_pubkey_category.remote_authrize_pubkey)))
              {
                led1_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(500));
                led3_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(500));
                buzzer_op(INDC_TWINKLE, IND_ON, SEC2TICK(1), MSEC2TICK(500));
              }
            else if (auth_need_more())
              {
                auth_set_auth_unlock();
                led1_op(INDC_TWINKLE, IND_GREEN, SEC2TICK(3), MSEC2TICK(500));
              }
            else
              {
                act_unlock(LOG_AUTH_UNLOCK);
              }
            (void)protocal_send_ok(sockfd, REMOTE_AUTHORIZE_CATEGORY);
          case UNLOCK_CATEGORY:
            pubkey = {0};
//			  memset(pubkey, 0, CONFIG_PUBKEY_SIZE);
            if (protocal->body.unlock_category.unlock_flag == NEED_CHECK_HALF_UNLOCK && (auth_time_out_check(), !auth_half_unlock()))
              {
                auth_set_temp_unlock();
                g_log_type |= LOG_HALF_UNLOCK;
                led1_op(INDC_TWINKLE, IND_GREEN, SEC2TICK(3), MSEC2TICK(500));
              }
            else
              {
                act_unlock(LOG_TEMP_UNLOCK);
                led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
              }
            (void)protocal_send_ok(sockfd, UNLOCK_CATEGORY);
            break;
          case SOFT_RESET_CATEGORY:
            up_systemreset();
            break;
          case ASSIGN_SN_CATEGORY:
            config.serial_no = protocal->body.assign_sn_category.assigned_sn;
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, ASSIGN_SN_CATEGORY);
            break;
          case ASSIGN_HOSTADDR_CATEGORY:
            config.hostaddr = protocal->body.assign_hostaddr_category.hostaddr;
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, ASSIGN_HOSTADDR_CATEGORY);
            break;
          case ASSIGN_NETMASK_CATEGORY:
            config.netmask = protocal->body.assign_netmask_category.netmask;
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, ASSIGN_NETMASK_CATEGORY);
            break;
          case ASSIGN_DRIPADDR_CATEGORY:
            config.dripaddr = protocal->body.assign_dripaddr_cateory.dripaddr;
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, ASSIGN_DRIPADDR_CATEGORY);
            break;
          case ASSIGN_SVRADDR_CATEGORY:
            config.svraddr = protocal->body.assign_svraddr_category.svraddr;
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, ASSIGN_SVRADDR_CATEGORY);
            break;
          case ASSIGN_MAC_CATEGORY:
            memcpy(config.macaddr, protocal->body.assign_mac_category.macaddr,  IFHWADDRLEN);
            ret = save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            (void)protocal_send_ok(sockfd, ASSIGN_SVRADDR_CATEGORY);
            break;
          case UPLOAD_PUBKEY_CATEGORY:
//            ret = protocal_send_pubkey(sockfd, pubkey);
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            break;
          case DOWNLOAD_PUBKEY_CATEGORY:
            pos = 0xff;
            for (i = 0; i < CONFIG_GROUP_SIZE; i++)
              {
                flag = 0;
				  if (memcmp(config.keyslots[i]))
                for (i = 0; i <= 127; i++)
                  {
                    _pubkey[i] = app_eprom_read(PUBKEY_START_ADDR + j * 129 + i + 1);
                    if (_pubkey[i] != 0xff && flag == 0) flag = 1;
                  }
                if (flag)
                  {
                    for (i = 0; i <= 127; i++)
                      {
                        if (_pubkey[i] != protocal->body.download_pubkey_category.download_pubkey[i])
                          {
                            break;
                          }
                      }
                    if (i > 127)
                      {
                        pos = j;
                      }

                  }
                else
                  {
                    if (pos > j)
                      pos = j;
                  }
              }
            if (pos == 0xff)
              pos = 0;
            app_eprom_write(PUBKEY_START_ADDR + pos * 129, protocal->body.download_pubkey_category.download_group_no);
            for (i = 0; i <= 127; i++)
              {
                app_eprom_write(PUBKEY_START_ADDR + pos * 129 + i + 1, protocal->body.download_pubkey_category.download_pubkey[i]);
              }
            app_eprom_flush();
            act_PB2(0, 60, BLUE);
            send_ok_category[DOWNLOAD_PUBKEY_CATEGORY]++;
            break;
          case CLEAR_PUBKEY_CATEGORY:
            for (j = 0; j <= SIZEOFKEYSTORESPACE - 1; j++)
              {
                for (i = 0; i <= 127; i++)
                  {
                    if (app_eprom_read(PUBKEY_START_ADDR + j * 129 + i + 1) != protocal->body.clear_pubkey_category.clear_pubkey[i])
                      {
                        break;
                      }
                  }
                if (i > 127)
                  {
                    for (i = 0; i <= 128; i++)
                      {
                        app_eprom_write(PUBKEY_START_ADDR + j * 129 + i, 0xff);
                      }
                  }
              }
            app_eprom_flush();
            act_PB2(0, 60, BLUE);
            send_ok_category[CLEAR_PUBKEY_CATEGORY]++;
            break;
          case CLEAR_ALL_PUBKEY_CATEGORY:
            for (j = 0; j <= SIZEOFKEYSTORESPACE - 1; j++)
              {
                for (i = 0; i <= 128; i++)
                  {
                    app_eprom_write(PUBKEY_START_ADDR + j * 129 + i, 0xff);
                  }
              }
            app_eprom_flush();
            act_PB2(0, 60, BLUE);
            send_ok_category[CLEAR_ALL_PUBKEY_CATEGORY]++;
            break;
          case DOWNLOAD_FIRMWARE_CATEGORY:
            ET0 = 0;
            firmware_crc32 = SWAP32(protocal->body.download_firmware_category.firmware_crc32);
            firmware_len = SWAP32(protocal->body.download_firmware_category.firmware_len);
            firmware_pos = SWAP32(protocal->body.download_firmware_category.firmware_pos);
            l = firmware_len - firmware_pos;
            if (l > FIRMWARE_CACHE_SIZE)
              {
                memcpy(firmware_cache, protocal->body.download_firmware_category.firmware, FIRMWARE_CACHE_SIZE);
                firmware_cache_len = FIRMWARE_CACHE_SIZE;
              }
            else
              {
                memcpy(firmware_cache, protocal->body.download_firmware_category.firmware, l);
                firmware_cache_len = l;
                download_firmware_completed = 1;
              }
            ET0 = 1;
            act_PB2(0, 60, BLUE);
            break;
          case UPDATE_FIRMWARE_CATEGORY:
            update_firmware_flag = 1;
            break;
          case VIEW_NET_ADDR_CATEGORY:
            view_net_addr_flag = 1;
            act_PB2(0, 60, BLUE);
            break;
          case CLEAR_ALL_PUBKEY_IN_GROUP_CATEGORY:
            for (j = 0; j <= SIZEOFKEYSTORESPACE - 1; j++)
              {
                if (app_eprom_read(PUBKEY_START_ADDR + j * 129) == protocal->body.clear_all_pubkey_in_group_category.clear_group_no)
                  {
                    for (i = 0; i <= 128; i++)
                      {
                        app_eprom_write(PUBKEY_START_ADDR + j * 129 + i, 0xff);
                      }
                  }
              }
            app_eprom_flush();
            act_PB2(0, 60, BLUE);
            send_ok_category[CLEAR_ALL_PUBKEY_IN_GROUP_CATEGORY]++;
            break;
          case VERSION_VIEW_CATEGORY:
            send_version_flag = 1;
            break;
          case DEFINE_CONFIG_PASSWORD_CATEGORY:
            for (j = 0; j <= 5; j++)
              {
                app_eprom_write(CONFIG_START_ADDR + 29 + j, protocal->body.define_config_password_category.config_password[j]);
              }
            app_eprom_flush();
            act_PB2(0, 60, BLUE);
            send_ok_category[DEFINE_CONFIG_PASSWORD_CATEGORY]++;
            break;
          default:
            break;
        }
    }
}

void protocal_newdata(struct protocal_state *s, unsigned char *dat, unsigned short len)
{
  unsigned short l;
  if (s->recv_dat != NULL && s->recv_dat > s->temp_recv_protocal)
    {
      l = s->recv_dat - s->temp_recv_protocal;
      if (l >= sizeof (((struct protocal_s *) NULL)->head))
        {
          l = protocal_size[((struct protocal_s *) (s->temp_recv_protocal))->head.category] - l;
          if (len >= l)
            {
              memcpy(s->recv_dat, dat, l);
              s->recv_dat = NULL;
              protocal_deal((struct protocal_s *) s->temp_recv_protocal);
              dat += l;
              len -= l;
            }
          else
            {
              memcpy(s->recv_dat, dat, len);
              s->recv_dat += len;
              return;
            }
        }
      else
        {
          l = sizeof (((struct protocal_s *) NULL)->head) - l;
          if (len >= l)
            {
              memcpy(s->recv_dat, dat, l);
              s->recv_dat += l;
              dat += l;
              len -= l;
              l = protocal_size[((struct protocal_s *) (s->temp_recv_protocal))->head.category] - sizeof (((struct protocal *) NULL)->head);
              if (len >= l)
                {
                  memcpy(s->recv_dat, dat, l);
                  s->recv_dat = NULL;
                  protocal_deal((struct protocal_s *) s->temp_recv_protocal);
                  dat += l;
                  len -= l;
                }
              else
                {
                  memcpy(s->recv_dat, dat, len);
                  s->recv_dat += len;
                  return;
                }
            }
          else
            {
              memcpy(s->recv_dat, dat, len);
              s->recv_dat += len;
              return;
            }
        }
    }
  while (len >= sizeof (((struct protocal_s *) NULL)->head) && len >= (l = protocal_size[((struct protocal_s *) (dat))->head.category]))
    {
      protocal_deal((struct protocal_s *) dat);
      dat += l;
      len -= l;
    }
  memcpy(s->temp_recv_protocal, dat, len);
  s->recv_dat = s->temp_recv_protocal + len;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/
