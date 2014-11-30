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

int protocal_send_heart_beat(void)
{
  struct protocal_s p;

  protocal_make_magic(p);
  p.head.category = HEART_BEAT_CATEGORY;
  p.head.serial_no = SWAP32(serial_no);

  s->send_protocal_pos += HEART_BEAT_CATEGORY_SEND_SIZE;
  ET0 = 1;
  return 1;
}

int protocal_send_connect(void)
{
  struct protocal *p;
  struct protocal_state *s = (struct protocal_state *) protocal_conn->appstate;
  ET0 = 0;
  if (s->send_dat != NULL || !net_connected() || (s->send_protocal_pos + CONNECT_CATEGORY_SEND_SIZE) >= sizeof (s->send_protocal))
    {
      ET0 = 1;
      return 0;
    }

  p = (struct protocal *) (s->send_protocal + s->send_protocal_pos);

  make_magic(p);
  p->head.category = CONNECT_CATEGORY;

  p->head.serial_no = SWAP32(serial_no);
  p->body.connect_category.connect_iap_version = SWAP32(iap_version);
  p->body.connect_category.connect_app_version = SWAP32(app_version);

  s->send_protocal_pos += CONNECT_CATEGORY_SEND_SIZE;
  ET0 = 1;
  return 1;
}

int protocal_send_alert(unsigned long serial_no, unsigned char alert_type, unsigned char time[6])
{
  struct protocal *p;
  struct protocal_state *s = (struct protocal_state *) protocal_conn->appstate;
  ET0 = 0;
  if (s->send_dat != NULL || !net_connected() || (s->send_protocal_pos + ALERT_CATEGORY_SEND_SIZE) >= sizeof (s->send_protocal))
    {
      ET0 = 1;
      return 0;
    }

  p = (struct protocal *) (s->send_protocal + s->send_protocal_pos);

  make_magic(p);
  p->head.category = ALERT_CATEGORY;

  p->head.serial_no = SWAP32(serial_no);
  p->body.alert_category.alert_type = alert_type;
  memcpy(p->body.alert_category.alert_time, time, 6);

  //    p->body.alert_category.alert_time[0] = time[0];
  //    p->body.alert_category.alert_time[1] = time[1];
  //    p->body.alert_category.alert_time[2] = time[2];
  //    p->body.alert_category.alert_time[3] = time[3];
  //    p->body.alert_category.alert_time[4] = time[4];
  //    p->body.alert_category.alert_time[5] = time[5];

  s->send_protocal_pos += ALERT_CATEGORY_SEND_SIZE;
  ET0 = 1;
  return 1;
}

int protocal_send_log(unsigned long serial_no, unsigned char log_group_no, unsigned char *log_pubkey, unsigned char flag, unsigned char time[6])
{
  struct protocal *p;
  struct protocal_state *s = (struct protocal_state *) protocal_conn->appstate;
  ET0 = 0;
  if (s->send_dat != NULL || !net_connected() || (s->send_protocal_pos + LOG_CATEGORY_SEND_SIZE) >= sizeof (s->send_protocal))
    {
      ET0 = 1;
      return 0;
    }

  p = (struct protocal *) (s->send_protocal + s->send_protocal_pos);

  make_magic(p);

  p->head.category = LOG_CATEGORY;

  p->head.serial_no = SWAP32(serial_no);
  p->body.log_category.log_group_no = log_group_no;
  memcpy(p->body.log_category.log_pubkey, log_pubkey, 128);
  p->body.log_category.flag = flag;
  memcpy(p->body.log_category.log_time, time, 6);
  //    p->body.log_category.log_time[0] = time[0];
  //    p->body.log_category.log_time[1] = time[1];
  //    p->body.log_category.log_time[2] = time[2];
  //    p->body.log_category.log_time[3] = time[3];
  //    p->body.log_category.log_time[4] = time[4];
  //    p->body.log_category.log_time[5] = time[5];

  s->send_protocal_pos += LOG_CATEGORY_SEND_SIZE;
  ET0 = 1;
  return 1;
}

int protocal_send_time_view(void)
{
  struct protocal *p;
  struct protocal_state *s = (struct protocal_state *) protocal_conn->appstate;
  ET0 = 0;
  if (s->send_dat != NULL || !net_connected() || (s->send_protocal_pos + TIME_VIEW_CATEGORY_SEND_SIZE) >= sizeof (s->send_protocal))
    {
      ET0 = 1;
      return 0;
    }

  p = (struct protocal *) (s->send_protocal + s->send_protocal_pos);

  make_magic(p);

  p->head.category = TIME_VIEW_CATEGORY;

  p->head.serial_no = SWAP32(serial_no);

  gmtime(second_count, p->body.time_view_category.view_time);

  //    p->body.time_view_category.view_time[0] = DS1302_ReadYear();
  //    p->body.time_view_category.view_time[1] = DS1302_ReadMonth();
  //    p->body.time_view_category.view_time[2] = DS1302_ReadDay();
  //    p->body.time_view_category.view_time[3] = DS1302_ReadHour();
  //    p->body.time_view_category.view_time[4] = DS1302_ReadMinute();
  //    p->body.time_view_category.view_time[5] = DS1302_ReadSecond();

  s->send_protocal_pos += TIME_VIEW_CATEGORY_SEND_SIZE;
  ET0 = 1;
  return 1;
}

int protocal_send_sensor_view(void)
{
  struct protocal *p;
  struct protocal_state *s = (struct protocal_state *) protocal_conn->appstate;
  ET0 = 0;
  if (s->send_dat != NULL || !net_connected() || (s->send_protocal_pos + SENSOR_VIEW_CATEGORY_SEND_SIZE) >= sizeof (s->send_protocal))
    {
      ET0 = 1;
      return 0;
    }

  p = (struct protocal *) (s->send_protocal + s->send_protocal_pos);

  make_magic(p);

  p->head.category = SENSOR_VIEW_CATEGORY;

  p->head.serial_no = SWAP32(serial_no);
  p->body.sensor_view_category.view_shock_voltage = SWAP(P15_voltage);
  p->body.sensor_view_category.view_lock_voltage = SWAP(P10_voltage);
  p->body.sensor_view_category.view_light_voltage = SWAP(P11_voltage);
  p->body.sensor_view_category.view_bat_voltage = SWAP(bat_voltage);
  p->body.sensor_view_category.view_pow_voltage = SWAP(pow_voltage);

  s->send_protocal_pos += SENSOR_VIEW_CATEGORY_SEND_SIZE;
  ET0 = 1;
  return 1;
}

int protocal_send_net_addr_view(void)
{
  struct protocal *p;
  struct protocal_state *s = (struct protocal_state *) protocal_conn->appstate;
  ET0 = 0;
  if (s->send_dat != NULL || !net_connected() || (s->send_protocal_pos + VIEW_NET_ADDR_CATEGORY_SEND_SIZE) >= sizeof (s->send_protocal))
    {
      ET0 = 1;
      return 0;
    }

  p = (struct protocal *) (s->send_protocal + s->send_protocal_pos);

  make_magic(p);

  p->head.category = VIEW_NET_ADDR_CATEGORY;

  p->head.serial_no = SWAP32(serial_no);
  memcpy(p->body.view_net_addr_category.view_hostip, uip_hostaddr, 4);
  memcpy(p->body.view_net_addr_category.view_netmask, uip_arp_netmask, 4);
  memcpy(p->body.view_net_addr_category.view_gateway, uip_arp_draddr, 4);
  memcpy(p->body.view_net_addr_category.view_serverip, server_ip_addr, 4);
  memcpy(p->body.view_net_addr_category.view_macaddr, uip_ethaddr.addr, 6);

  s->send_protocal_pos += VIEW_NET_ADDR_CATEGORY_SEND_SIZE;
  ET0 = 1;
  return 1;
}

int protocal_send_pubkey(void)
{
  struct protocal *p;
  struct protocal_state *s = (struct protocal_state *) protocal_conn->appstate;
  ET0 = 0;
  if (s->send_dat != NULL || !net_connected() || (s->send_protocal_pos + UPLOAD_PUBKEY_CATEGORY_SEND_SIZE) >= sizeof (s->send_protocal))
    {
      ET0 = 1;
      return 0;
    }

  p = (struct protocal *) (s->send_protocal + s->send_protocal_pos);

  make_magic(p);

  p->head.category = UPLOAD_PUBKEY_CATEGORY;

  p->head.serial_no = SWAP32(serial_no);
  memcpy(p->body.upload_pubkey_category.upload_pubkey, pubkey, 128);

  s->send_protocal_pos += UPLOAD_PUBKEY_CATEGORY_SEND_SIZE;
  ET0 = 1;
  return 1;
}



int protocal_send_ok(int sockfd, uint8_t category)
{
  int ret;
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


void protocal_deal(int sockfd, struct protocal * protocal)
{
  unsigned char i;
  unsigned char j;
  unsigned char _pubkey[128];
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
            save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            protocal_send_ok(sockfd, ASSIGN_SHOCK_RESISTOR_THRESHOLD_CATEGORY);
            break;
          case ASSIGN_INFRA_RED_THRESHOLD_CATEGORY:
            config->infra_red_threshold = protocal->body.assign_infra_red_threshold_category.assign_infra_red_threshold;
            save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            protocal_send_ok(sockfd, ASSIGN_INFRA_RED_THRESHOLD_CATEGORY);
            break;
          case ASSIGN_PHOTO_RESISTOR_THRESHOLD_CATEGORY:
            config->photo_resistor_threshold = protocal->body.assign_photo_resistor_threshold_category.assign_photo_resistor_threshold;
            save_config();
            led3_op(INDC_TWINKLE, IND_BLUE, SEC2TICK(3), MSEC2TICK(500));
            protocal_send_ok(sockfd, ASSIGN_PHOTO_RESISTOR_THRESHOLD_CATEGORY);
            break;
          case SENSOR_VIEW_CATEGORY:
            view_sensor_flag = 1;
            act_PB2(0, 60, BLUE);
            break;
          case TIME_VIEW_CATEGORY:
            view_time_flag = 1;
            act_PB2(0, 60, BLUE);
            break;
          case TIME_SYNC_CATEGORY:
            second_count = mktime(protocal->body.time_sync_category.sync_time);
            //                DS1302_OpenWrite();
            //                DS1302_StopDS1302();
            //                DS1302_SetSecond(protocal->body.time_sync_category.sync_time[5]);
            //                DS1302_SetMinute(protocal->body.time_sync_category.sync_time[4]);
            //                DS1302_SetHour(protocal->body.time_sync_category.sync_time[3]);
            //                DS1302_SetDay(protocal->body.time_sync_category.sync_time[2]);
            //                DS1302_SetMonth(protocal->body.time_sync_category.sync_time[1]);
            //                DS1302_SetYear(protocal->body.time_sync_category.sync_time[0]);
            //                DS1302_ReadOnly();
            act_PB2(0, 60, BLUE);
            send_ok_category[TIME_SYNC_CATEGORY]++;
            break;
          case REMOTE_AUTHORIZE_CATEGORY:
            memset(pubkey, 0, 128);
            group_no_time_out_check();
            if (!find_pub_key())
              {
                clear_check();
                if (!find_pub_key())
                  {
                    act_PB2(0, 60, RED);
                    act_PB3(0, 60, RED);
                    act_P17(0, 20);
                    send_ok_category[REMOTE_AUTHORIZE_CATEGORY]++;
                    lcd_add_log(key_no_bind);
                    return;
                  }
              }
            if (find_group_no())
              {
                remote_auth_unlock_flag = 1;
                //last_group_no = group_no;
                log_flag |= LOG_HALF_UNLOCK;
                act_PB3(0, 60, GREEN);
                send_ok_category[REMOTE_AUTHORIZE_CATEGORY]++;
                lcd_add_log(key_half_unlock);
                return;
              }

            act_net_unlock(LOG_AUTH_UNLOCK);
            send_ok_category[REMOTE_AUTHORIZE_CATEGORY]++;
            break;
          case UNLOCK_CATEGORY:
            memset(pubkey, 0, 128);
            if (protocal->body.unlock_category.unlock_flag == NEED_CHECK_HALF_UNLOCK)
              {
                group_no_time_out_check();
                if (!check_half_unlock())
                  {
                    set_omnipotent_group_no();
                    //last_group_no = group_no;
                    log_flag |= LOG_HALF_UNLOCK;
                    act_PB3(0, 60, GREEN);
                    send_ok_category[UNLOCK_CATEGORY]++;
                    lcd_add_log(key_half_unlock);
                    return;
                    //                        log_flag |= LOG_DENNY_UNLOCK;
                    //                        act_PB2(0, 60, BLUE);
                    //                        lcd_add_log("¾Ü¾ø¿ªËø");
                  }
              }
            act_net_unlock(LOG_TEMP_UNLOCK);
            act_PB2(0, 60, BLUE);
            send_ok_category[UNLOCK_CATEGORY]++;
            break;
          case SOFT_RESET_CATEGORY:
            soft_reset();
            break;
          case ASSIGN_SN_CATEGORY:

            app_eprom_write(CONFIG_START_ADDR + 16, protocal->body.assign_sn_category.assigned_sn[3]);
            app_eprom_write(CONFIG_START_ADDR + 17, protocal->body.assign_sn_category.assigned_sn[2]);
            app_eprom_write(CONFIG_START_ADDR + 18, protocal->body.assign_sn_category.assigned_sn[1]);
            app_eprom_write(CONFIG_START_ADDR + 19, protocal->body.assign_sn_category.assigned_sn[0]);
            app_eprom_flush();
            act_PB2(0, 60, BLUE);
            send_ok_category[ASSIGN_SN_CATEGORY]++;
            break;
          case ASSIGN_HOSTIP_CATEGORY:
            assign_hostip(protocal->body.assign_hostip_category.hostip[3],
                          protocal->body.assign_hostip_category.hostip[2],
                          protocal->body.assign_hostip_category.hostip[1],
                          protocal->body.assign_hostip_category.hostip[0]);
            //                app_eprom_write(CONFIG_START_ADDR + 0, protocal->body.assign_host_ip_category.host_ip[3]);
            //                app_eprom_write(CONFIG_START_ADDR + 1, protocal->body.assign_host_ip_category.host_ip[2]);
            //                app_eprom_write(CONFIG_START_ADDR + 2, protocal->body.assign_host_ip_category.host_ip[1]);
            //                app_eprom_write(CONFIG_START_ADDR + 3, protocal->body.assign_host_ip_category.host_ip[0]);
            //                app_eprom_flush();
            app_eprom_flush();
            act_PB2(0, 60, BLUE);
            send_ok_category[ASSIGN_HOSTIP_CATEGORY]++;
            break;
          case ASSIGN_NETMASK_CATEGORY:
            assign_netmask(protocal->body.assign_netmask_category.netmask[3],
                           protocal->body.assign_netmask_category.netmask[2],
                           protocal->body.assign_netmask_category.netmask[1],
                           protocal->body.assign_netmask_category.netmask[0]
                          );
            app_eprom_flush();
            //                app_eprom_write(CONFIG_START_ADDR + 8, protocal->body.assign_netmask_category.netmask[3]);
            //                app_eprom_write(CONFIG_START_ADDR + 9, protocal->body.assign_netmask_category.netmask[2]);
            //                app_eprom_write(CONFIG_START_ADDR + 10, protocal->body.assign_netmask_category.netmask[1]);
            //                app_eprom_write(CONFIG_START_ADDR + 11, protocal->body.assign_netmask_category.netmask[0]);
            //                app_eprom_flush();
            act_PB2(0, 60, BLUE);
            send_ok_category[ASSIGN_NETMASK_CATEGORY]++;
            break;
          case ASSIGN_GATEWAY_CATEGORY:
            assign_gateway(
              protocal->body.assign_gateway_cateory.gateway[3],
              protocal->body.assign_gateway_cateory.gateway[2],
              protocal->body.assign_gateway_cateory.gateway[1],
              protocal->body.assign_gateway_cateory.gateway[0]
            );
            app_eprom_flush();
            //                app_eprom_write(CONFIG_START_ADDR + 4, protocal->body.assign_gateway_cateory.gateway[3]);
            //                app_eprom_write(CONFIG_START_ADDR + 5, protocal->body.assign_gateway_cateory.gateway[2]);
            //                app_eprom_write(CONFIG_START_ADDR + 6, protocal->body.assign_gateway_cateory.gateway[1]);
            //                app_eprom_write(CONFIG_START_ADDR + 7, protocal->body.assign_gateway_cateory.gateway[0]);
            //                app_eprom_flush();
            act_PB2(0, 60, BLUE);
            send_ok_category[ASSIGN_GATEWAY_CATEGORY]++;
            break;
          case ASSIGN_SERVERIP_CATEGORY:
            assign_serverip(protocal->body.assign_serverip_category.serverip[3],
                            protocal->body.assign_serverip_category.serverip[2],
                            protocal->body.assign_serverip_category.serverip[1],
                            protocal->body.assign_serverip_category.serverip[0]
                           );
            app_eprom_flush();
            //                app_eprom_write(CONFIG_START_ADDR + 12, protocal->body.assign_server_ip_category.server_ip[3]);
            //                app_eprom_write(CONFIG_START_ADDR + 13, protocal->body.assign_server_ip_category.server_ip[2]);
            //                app_eprom_write(CONFIG_START_ADDR + 14, protocal->body.assign_server_ip_category.server_ip[1]);
            //                app_eprom_write(CONFIG_START_ADDR + 15, protocal->body.assign_server_ip_category.server_ip[0]);
            //                app_eprom_flush();
            act_PB2(0, 60, BLUE);
            send_ok_category[ASSIGN_SERVERIP_CATEGORY]++;
            break;
          case ASSIGN_MAC_CATEGORY:
            assign_macaddr(protocal->body.assign_mac_category.macaddr[0],
                           protocal->body.assign_mac_category.macaddr[1],
                           protocal->body.assign_mac_category.macaddr[2],
                           protocal->body.assign_mac_category.macaddr[3],
                           protocal->body.assign_mac_category.macaddr[4],
                           protocal->body.assign_mac_category.macaddr[5]);
            app_eprom_flush();
            act_PB2(0, 60, BLUE);
            send_ok_category[ASSIGN_MAC_CATEGORY]++;
            break;
          case UPLOAD_PUBKEY_CATEGORY:
            upload_pubkey_flag = 1;
            act_PB2(0, 60, BLUE);
            break;
          case DOWNLOAD_PUBKEY_CATEGORY:
            pos = 0xff;
            for (j = 0; j <= SIZEOFKEYSTORESPACE - 1; j++)
              {
                flag = 0;
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
