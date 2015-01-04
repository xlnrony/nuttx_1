
/****************************************************************************
 * examples/ilock/protocal.h
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

#ifndef __APPS_INCLUDE_PROTOCAL_H
#define __APPS_INCLUDE_PROTOCAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <net/if.h>
#include <netinet/in.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAGIC_ONE 					'L'
#define MAGIC_TWO 					'K'
#define MAGIC_THREE					'0'
#define MAGIC_FOUR 					'0'

#define HEART_BEAT_CATEGORY 													0
#define LOG_CATEGORY 																		1
#define DOWNLOAD_PUBKEY_CATEGORY 										2
#define CLEAR_PUBKEY_CATEGORY 												3
#define ASSIGN_SN_CATEGORY 														4
#define ASSIGN_HOSTADDR_CATEGORY  										5
#define ASSIGN_NETMASK_CATEGORY  											6
#define ASSIGN_DRIPADDR_CATEGORY  										7
#define ASSIGN_SVRADDR_CATEGORY											8
#define ASSIGN_MAC_CATEGORY 	   												9
#define CLEAR_ALL_PUBKEY_CATEGORY  										10
#define SOFT_RESET_CATEGORY		   												11
#define UNLOCK_CATEGORY		   														12
#define UPLOAD_PUBKEY_CATEGORY	   											13
#define REMOTE_AUTHORIZE_CATEGORY  										14
#define TIME_SYNC_CATEGORY		   												15
#define ALERT_CATEGORY			   														16
#define CONNECT_CATEGORY		 													17
#define TIME_VIEW_CATEGORY		   												18
#define SENSOR_VIEW_CATEGORY	   			   									19
#define ASSIGN_SHOCK_RESISTOR_THRESHOLD_CATEGORY	   	20
#define ASSIGN_INFRA_RED_THRESHOLD_CATEGORY	   			21
#define ASSIGN_PHOTO_RESISTOR_THRESHOLD_CATEGORY	   	22
#define DOWNLOAD_FIRMWARE_CATEGORY									23
#define CRC32_FIRMWARE_CATEGORY											24
#define UPDATE_FIRMWARE_CATEGORY											25
#define VIEW_NET_ADDR_CATEGORY												26
#define CLEAR_ALL_PUBKEY_IN_GROUP_CATEGORY					27
#define VERSION_VIEW_CATEGORY													28
#define DEFINE_CONFIG_PASSWORD_CATEGORY							29
#define REMOTE_AUTHORIZE_WITH_PUBKEY_CATEGORY  		30
#define LOG_EX_CATEGORY 																31
#define LAST_CATEGORY																	31

#define ALERT_SHOCK_RESISTOR														1
#define ALERT_CLOSE_SWITCH															2
//#define ALERT_NET_DISCONNECT          										4
#define ALERT_INFRA_RED																8
#define ALERT_LOCK																			16
#define ALERT_NOLOCK_TIME_OUT													32
#define ALERT_PHOTO_RESISTOR           										64
//#define ALERT_NET_CONNECT               											128

#define LOG_HALF_UNLOCK						1
#define LOG_UNLOCK									2
#define LOG_KEY_PASSWORD_ERROR		4
#define LOG_AUTH_UNLOCK						8
#define LOG_TEMP_UNLOCK						16
#define LOG_DENNY_UNLOCK					32

#define SKIP_CHECK_HALF_UNLOCK 		0
#define NEED_CHECK_HALF_UNLOCK 		1

#define FIRMWARE_CACHE_SIZE				1280

#define PROTOCAL_HEAD_SIZE																				(sizeof(((struct protocal_s *)NULL)->head))
#define HEART_BEAT_CATEGORY_RECV_SIZE													(PROTOCAL_HEAD_SIZE)
#define HEART_BEAT_CATEGORY_SEND_SIZE													(PROTOCAL_HEAD_SIZE)
#define LOG_CATEGORY_RECV_SIZE 																	(PROTOCAL_HEAD_SIZE)
#define LOG_CATEGORY_SEND_SIZE 																	(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.log_category))
#define DOWNLOAD_PUBKEY_CATEGORY_RECV_SIZE 										(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.download_pubkey_category))
#define DOWNLOAD_PUBKEY_CATEGORY_SEND_SIZE 									(PROTOCAL_HEAD_SIZE)
#define CLEAR_PUBKEY_CATEGORY_RECV_SIZE 												(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.clear_pubkey_category))
#define CLEAR_PUBKEY_CATEGORY_SEND_SIZE 												(PROTOCAL_HEAD_SIZE)
#define ASSIGN_SN_CATEGORY_RECV_SIZE 														(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.assign_sn_category))
#define ASSIGN_SN_CATEGORY_SEND_SIZE 													(PROTOCAL_HEAD_SIZE)
#define ASSIGN_HOSTADDR_CATEGORY_RECV_SIZE  										(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.assign_hostaddr_category))
#define ASSIGN_HOSTADDR_CATEGORY_SEND_SIZE  									(PROTOCAL_HEAD_SIZE)
#define ASSIGN_NETMASK_CATEGORY_RECV_SIZE  										(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.assign_netmask_category))
#define ASSIGN_NETMASK_CATEGORY_SEND_SIZE  										(PROTOCAL_HEAD_SIZE)
#define ASSIGN_DRIPADDR_CATEGORY_RECV_SIZE  										(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.assign_dripaddr_cateory))
#define ASSIGN_DRIPADDR_CATEGORY_SEND_SIZE  										(PROTOCAL_HEAD_SIZE)
#define ASSIGN_SVRADDR_CATEGORY_RECV_SIZE											(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.assign_svraddr_category))
#define ASSIGN_SVRADDR_CATEGORY_SEND_SIZE											(PROTOCAL_HEAD_SIZE)
#define ASSIGN_MACADDR_CATEGORY_RECV_SIZE 	   									(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.assign_macaddr_category))
#define ASSIGN_MACADDR_CATEGORY_SEND_SIZE 	   									(PROTOCAL_HEAD_SIZE)
#define CLEAR_ALL_PUBKEY_CATEGORY_RECV_SIZE  									(PROTOCAL_HEAD_SIZE)
#define CLEAR_ALL_PUBKEY_CATEGORY_SEND_SIZE  									(PROTOCAL_HEAD_SIZE)
#define SOFT_RESET_CATEGORY_RECV_SIZE		   											(PROTOCAL_HEAD_SIZE)
#define SOFT_RESET_CATEGORY_SEND_SIZE		   											(PROTOCAL_HEAD_SIZE)
#define UNLOCK_CATEGORY_RECV_SIZE		   														(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.unlock_category))
#define UNLOCK_CATEGORY_SEND_SIZE		   													(PROTOCAL_HEAD_SIZE)
#define UPLOAD_PUBKEY_CATEGORY_RECV_SIZE	   										(PROTOCAL_HEAD_SIZE)
#define UPLOAD_PUBKEY_CATEGORY_SEND_SIZE	   										(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.upload_pubkey_category))
#define REMOTE_AUTHORIZE_CATEGORY_RECV_SIZE  									(PROTOCAL_HEAD_SIZE)
#define REMOTE_AUTHORIZE_CATEGORY_SEND_SIZE  									(PROTOCAL_HEAD_SIZE)
#define TIME_SYNC_CATEGORY_RECV_SIZE		   												(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.time_sync_category))
#define TIME_SYNC_CATEGORY_SEND_SIZE		   												(PROTOCAL_HEAD_SIZE)
#define ALERT_CATEGORY_RECV_SIZE			   													(PROTOCAL_HEAD_SIZE)
#define ALERT_CATEGORY_SEND_SIZE			   													(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.alert_category))
#define CONNECT_CATEGORY_RECV_SIZE		 													(PROTOCAL_HEAD_SIZE)
#define CONNECT_CATEGORY_SEND_SIZE		 													(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.connect_category))
#define TIME_VIEW_CATEGORY_RECV_SIZE		   												(PROTOCAL_HEAD_SIZE)
#define TIME_VIEW_CATEGORY_SEND_SIZE		   												(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.time_view_category))
#define SENSOR_VIEW_CATEGORY_RECV_SIZE	   												(PROTOCAL_HEAD_SIZE)
#define SENSOR_VIEW_CATEGORY_SEND_SIZE	   											(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.sensor_view_category))
#define ASSIGN_SHOCK_RESISTOR_THRESHOLD_CATEGORY_RECV_SIZE	(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.assign_shock_resistor_threshold_category))
#define ASSIGN_SHOCK_RESISTOR_THRESHOLD_CATEGORY_SEND_SIZE	(PROTOCAL_HEAD_SIZE)
#define ASSIGN_INFRA_RED_THRESHOLD_CATEGORY_RECV_SIZE				(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.assign_infra_red_threshold_category))
#define ASSIGN_INFRA_RED_THRESHOLD_CATEGORY_SEND_SIZE				(PROTOCAL_HEAD_SIZE)
#define ASSIGN_PHOTO_RESISTOR_THRESHOLD_CATEGORY_RECV_SIZE	(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.assign_photo_resistor_threshold_category))
#define ASSIGN_PHOTO_RESISTOR_THRESHOLD_CATEGORY_SEND_SIZE	(PROTOCAL_HEAD_SIZE)
#define DOWNLOAD_FIRMWARE_CATEGORY_RECV_SIZE								(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.download_firmware_category))
#define DOWNLOAD_FIRMWARE_CATEGORY_SEND_SIZE								(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.download_firmware_category)-sizeof(((struct protocal_s *)NULL)->body.download_firmware_category.firmware))
#define CRC32_FIRMWARE_CATEGORY_RECV_SIZE											(PROTOCAL_HEAD_SIZE)
#define CRC32_FIRMWARE_CATEGORY_SEND_SIZE											(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.crc32_firmware_category))
#define UPDATE_FIRMWARE_CATEGORY_RECV_SIZE										(PROTOCAL_HEAD_SIZE)
#define UPDATE_FIRMWARE_CATEGORY_SEND_SIZE										(PROTOCAL_HEAD_SIZE)
#define VIEW_NET_ADDR_CATEGORY_RECV_SIZE											(PROTOCAL_HEAD_SIZE)
#define VIEW_NET_ADDR_CATEGORY_SEND_SIZE											(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.view_net_addr_category))
#define CLEAR_ALL_PUBKEY_IN_GROUP_CATEGORY_RECV_SIZE					(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.clear_all_pubkey_in_group_category))
#define CLEAR_ALL_PUBKEY_IN_GROUP_CATEGORY_SEND_SIZE					(PROTOCAL_HEAD_SIZE)
#define VERSION_VIEW_CATEGORY_RECV_SIZE												(PROTOCAL_HEAD_SIZE)
#define VERSION_VIEW_CATEGORY_SEND_SIZE												(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.version_view_category))
#define DEFINE_CONFIG_PASSWORD_CATEGORY_RECV_SIZE						(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.define_config_password_category))
#define DEFINE_CONFIG_PASSWORD_CATEGORY_SEND_SIZE						(PROTOCAL_HEAD_SIZE)
#define REMOTE_AUTHORIZE_WITH_PUBKEY_CATEGORY_RECV_SIZE 		(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.remote_authorize_with_pubkey_category))
#define REMOTE_AUTHORIZE_WITH_PUBKEY_CATEGORY_SEND_SIZE 		(PROTOCAL_HEAD_SIZE)
#define LOG_EX_CATEGORY_RECV_SIZE 															(PROTOCAL_HEAD_SIZE)
#define LOG_EX_CATEGORY_SEND_SIZE 															(PROTOCAL_HEAD_SIZE+sizeof(((struct protocal_s *)NULL)->body.log_ex_category))

/****************************************************************************
 * Public Types
 ****************************************************************************/
#pragma pack(1)
struct protocal_s
{
  struct
  {
    char magic[4];
    uint32_t serial_no;
    uint8_t category;
  } head;

  union
  {
    struct
    {
      uint32_t connect_knl_version;
      uint32_t connect_app_version;
    } connect_category;

    struct
    {
      uint8_t sync_time[6];
    } time_sync_category;

    struct
    {
      uint8_t view_time[6];
    } time_view_category;

    struct
    {
      uint8_t alert_type;
      uint8_t alert_time[6];
    } alert_category;

    struct
    {
      uint8_t log_group_no;
      uint8_t log_pubkey[CONFIG_PUBKEY_SIZE];
      uint8_t flag;
      uint8_t log_time[6];
    } log_category;

    struct
    {
      uint8_t log_group[CONFIG_GROUP_SIZE];
      uint8_t log_pubkey[CONFIG_PUBKEY_SIZE];
      uint8_t flag;
      uint8_t log_time[6];
    } log_ex_category;

    struct
    {
      uint8_t download_group_no;
      uint8_t download_pubkey[CONFIG_PUBKEY_SIZE];
    } download_pubkey_category;

    struct
    {
      uint8_t upload_pubkey[CONFIG_PUBKEY_SIZE];
    } upload_pubkey_category;

    struct
    {
      uint8_t clear_pubkey[CONFIG_PUBKEY_SIZE];
    } clear_pubkey_category;

    struct
    {
      uint8_t remote_authorize_pubkey[CONFIG_PUBKEY_SIZE];
    } remote_authorize_with_pubkey_category;

    struct
    {
      uint8_t macaddr[IFHWADDRLEN];
    } assign_macaddr_category;

    struct
    {
      uint32_t assigned_sn;
    } assign_sn_category;

    struct
    {
      in_addr_t hostaddr;
    } assign_hostaddr_category;

    struct
    {
      in_addr_t netmask;
    } assign_netmask_category;

    struct
    {
      in_addr_t dripaddr;
    } assign_dripaddr_cateory;

    struct
    {
      in_addr_t svraddr;
    } assign_svraddr_category;

    struct
    {
      uint8_t unlock_flag;
    } unlock_category;

    struct
    {
      uint16_t view_shock_resistor_threshold;
      uint16_t view_infra_red_threshold;
      uint16_t view_photo_resistor_threshold;
      uint16_t view_battery_voltage;
      uint16_t view_power_voltage;
    } sensor_view_category;

    struct
    {
      uint32_t assign_shock_resistor_threshold;
    } assign_shock_resistor_threshold_category;

    struct
    {
      uint16_t assign_infra_red_threshold;
    } assign_infra_red_threshold_category;

    struct
    {
      uint16_t assign_photo_resistor_threshold;
    } assign_photo_resistor_threshold_category;

    struct
    {
      uint32_t firmware_crc32;
      uint32_t firmware_len;
      uint32_t firmware_pos;
      uint8_t firmware[FIRMWARE_CACHE_SIZE];
    } download_firmware_category;

    struct
    {
      uint32_t view_hostaddr;
      uint32_t view_netmask;
      uint32_t view_dripaddr;
      uint32_t view_svraddr;
      uint8_t view_macaddr[IFHWADDRLEN];
    } view_net_addr_category;

    struct
    {
      uint8_t crc32_success;
    } crc32_firmware_category;

    struct
    {
      uint8_t clear_group_no;
    } clear_all_pubkey_in_group_category;

    struct
    {
      uint32_t view_knl_version;
      uint32_t view_app_version;
    } version_view_category;

    struct
    {
      uint8_t config_password[6];
    } define_config_password_category;
  } body;
};
#pragma pack()

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

EXTERN int protocal_recv(void);
EXTERN int protocal_send_heart_beat(void);
EXTERN int protocal_send_connect(void);
EXTERN int protocal_send_alert(uint32_t serial_no, uint8_t alert_type, uint8_t tm[6]);
EXTERN int protocal_send_log(int32_t serial_no, bool *log_group, uint8_t *log_pubkey, uint8_t flag, uint8_t tm[6]);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif                                 /* __APPS_INCLUDE_PROTOCAL_H */
