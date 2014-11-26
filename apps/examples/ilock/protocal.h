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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HEART_BEAT_CATEGORY 					0
#define LOG_CATEGORY 							1
#define DOWNLOAD_PUBKEY_CATEGORY 				2
#define CLEAR_PUBKEY_CATEGORY 					3
#define ASSIGN_SN_CATEGORY 						4
#define ASSIGN_HOSTIP_CATEGORY  				5
#define ASSIGN_NETMASK_CATEGORY  				6
#define ASSIGN_GATEWAY_CATEGORY  				7
#define ASSIGN_SERVERIP_CATEGORY				8
#define ASSIGN_MAC_CATEGORY 	   				9
#define CLEAR_ALL_PUBKEY_CATEGORY  				10
#define SOFT_RESET_CATEGORY		   				11
#define UNLOCK_CATEGORY		   					12
#define UPLOAD_PUBKEY_CATEGORY	   				13			
#define REMOTE_AUTHORIZE_CATEGORY  				14
#define TIME_SYNC_CATEGORY		   				15
#define ALERT_CATEGORY			   				16
#define CONNECT_CATEGORY		 				17
#define TIME_VIEW_CATEGORY		   				18	
#define SENSOR_VIEW_CATEGORY	   			   	19	
#define ASSIGN_SHOCK_CHECK_VOLTAGE_CATEGORY	   	20
#define ASSIGN_LOCK_CHECK_VOLTAGE_CATEGORY	   	21
#define ASSIGN_LIGHT_CHECK_VOLTAGE_CATEGORY	   	22
#define DOWNLOAD_FIRMWARE_CATEGORY				23
#define CRC32_FIRMWARE_CATEGORY					24
#define UPDATE_FIRMWARE_CATEGORY				25
#define VIEW_NET_ADDR_CATEGORY					26
#define CLEAR_ALL_PUBKEY_IN_GROUP_CATEGORY		27
#define VERSION_VIEW_CATEGORY					28
#define DEFINE_CONFIG_PASSWORD_CATEGORY			29
#define LAST_CATEGORY							29

#define ALERT_SHOCK 				1
#define ALERT_LIGHT 				2
//#define ALERT_NET_DISCONNECT		4
#define ALERT_POWER 				8
#define ALERT_LOCK					16
#define ALERT_NOLOCK_TIME_OUT		32
//#define ALERT_LOW_POWER				64
//#define ALERT_NET_CONNECT			128

#define LOG_HALF_UNLOCK				1
#define LOG_UNLOCK					2
#define LOG_KEY_PASSWORD_ERROR		4
#define LOG_AUTH_UNLOCK				8	
#define LOG_TEMP_UNLOCK				16
#define LOG_DENNY_UNLOCK			32

#define SKIP_CHECK_HALF_UNLOCK 		0
#define NEED_CHECK_HALF_UNLOCK 		1

#define FIRMWARE_CACHE_SIZE			1280


#define HEART_BEAT_CATEGORY_RECV_SIZE					(sizeof(((struct protocal *)NULL)->head))
#define HEART_BEAT_CATEGORY_SEND_SIZE					(sizeof(((struct protocal *)NULL)->head))
#define LOG_CATEGORY_RECV_SIZE 							(sizeof(((struct protocal *)NULL)->head))
#define LOG_CATEGORY_SEND_SIZE 							(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.log_category))
#define DOWNLOAD_PUBKEY_CATEGORY_RECV_SIZE 				(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.download_pubkey_category))
#define DOWNLOAD_PUBKEY_CATEGORY_SEND_SIZE 				(sizeof(((struct protocal *)NULL)->head))
#define CLEAR_PUBKEY_CATEGORY_RECV_SIZE 				(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.clear_pubkey_category))
#define CLEAR_PUBKEY_CATEGORY_SEND_SIZE 				(sizeof(((struct protocal *)NULL)->head))
#define ASSIGN_SN_CATEGORY_RECV_SIZE 					(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.assign_sn_category))
#define ASSIGN_SN_CATEGORY_SEND_SIZE 					(sizeof(((struct protocal *)NULL)->head))
#define ASSIGN_HOSTIP_CATEGORY_RECV_SIZE  				(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.assign_hostip_category))
#define ASSIGN_HOSTIP_CATEGORY_SEND_SIZE  				(sizeof(((struct protocal *)NULL)->head))
#define ASSIGN_NETMASK_CATEGORY_RECV_SIZE  				(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.assign_netmask_category))
#define ASSIGN_NETMASK_CATEGORY_SEND_SIZE  				(sizeof(((struct protocal *)NULL)->head))
#define ASSIGN_GATEWAY_CATEGORY_RECV_SIZE  				(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.assign_gateway_cateory))
#define ASSIGN_GATEWAY_CATEGORY_SEND_SIZE  				(sizeof(((struct protocal *)NULL)->head))
#define ASSIGN_SERVERIP_CATEGORY_RECV_SIZE				(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.assign_serverip_category))
#define ASSIGN_SERVERIP_CATEGORY_SEND_SIZE				(sizeof(((struct protocal *)NULL)->head))
#define ASSIGN_MAC_CATEGORY_RECV_SIZE 	   				(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.assign_mac_category))
#define ASSIGN_MAC_CATEGORY_SEND_SIZE 	   				(sizeof(((struct protocal *)NULL)->head))
#define CLEAR_ALL_PUBKEY_CATEGORY_RECV_SIZE  			(sizeof(((struct protocal *)NULL)->head))
#define CLEAR_ALL_PUBKEY_CATEGORY_SEND_SIZE  			(sizeof(((struct protocal *)NULL)->head))
#define SOFT_RESET_CATEGORY_RECV_SIZE		   			(sizeof(((struct protocal *)NULL)->head))
#define SOFT_RESET_CATEGORY_SEND_SIZE		   			(sizeof(((struct protocal *)NULL)->head))
#define UNLOCK_CATEGORY_RECV_SIZE		   				(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.unlock_category))
#define UNLOCK_CATEGORY_SEND_SIZE		   				(sizeof(((struct protocal *)NULL)->head))
#define UPLOAD_PUBKEY_CATEGORY_RECV_SIZE	   			(sizeof(((struct protocal *)NULL)->head))
#define UPLOAD_PUBKEY_CATEGORY_SEND_SIZE	   			(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.upload_pubkey_category))
#define REMOTE_AUTHORIZE_CATEGORY_RECV_SIZE  			(sizeof(((struct protocal *)NULL)->head))
#define REMOTE_AUTHORIZE_CATEGORY_SEND_SIZE  			(sizeof(((struct protocal *)NULL)->head))
#define TIME_SYNC_CATEGORY_RECV_SIZE		   			(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.time_sync_category))
#define TIME_SYNC_CATEGORY_SEND_SIZE		   			(sizeof(((struct protocal *)NULL)->head))
#define ALERT_CATEGORY_RECV_SIZE			   			(sizeof(((struct protocal *)NULL)->head))
#define ALERT_CATEGORY_SEND_SIZE			   			(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.alert_category))
#define CONNECT_CATEGORY_RECV_SIZE		 				(sizeof(((struct protocal *)NULL)->head))
#define CONNECT_CATEGORY_SEND_SIZE		 				(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.connect_category))
#define TIME_VIEW_CATEGORY_RECV_SIZE		   			(sizeof(((struct protocal *)NULL)->head))
#define TIME_VIEW_CATEGORY_SEND_SIZE		   			(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.time_view_category))
#define SENSOR_VIEW_CATEGORY_RECV_SIZE	   				(sizeof(((struct protocal *)NULL)->head))
#define SENSOR_VIEW_CATEGORY_SEND_SIZE	   				(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.sensor_view_category))
#define ASSIGN_SHOCK_CHECK_VOLTAGE_CATEGORY_RECV_SIZE	(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.assign_shock_check_voltage_category))
#define ASSIGN_SHOCK_CHECK_VOLTAGE_CATEGORY_SEND_SIZE	(sizeof(((struct protocal *)NULL)->head))
#define ASSIGN_LOCK_CHECK_VOLTAGE_CATEGORY_RECV_SIZE	(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.assign_lock_check_voltage_category))
#define ASSIGN_LOCK_CHECK_VOLTAGE_CATEGORY_SEND_SIZE	(sizeof(((struct protocal *)NULL)->head))
#define ASSIGN_LIGHT_CHECK_VOLTAGE_CATEGORY_RECV_SIZE	(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.assign_light_check_voltage_category))
#define ASSIGN_LIGHT_CHECK_VOLTAGE_CATEGORY_SEND_SIZE	(sizeof(((struct protocal *)NULL)->head))
#define DOWNLOAD_FIRMWARE_CATEGORY_RECV_SIZE			(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.download_firmware_category))
#define DOWNLOAD_FIRMWARE_CATEGORY_SEND_SIZE			(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.download_firmware_category)-sizeof(((struct protocal *)NULL)->body.download_firmware_category.firmware))
#define CRC32_FIRMWARE_CATEGORY_RECV_SIZE				(sizeof(((struct protocal *)NULL)->head))
#define CRC32_FIRMWARE_CATEGORY_SEND_SIZE				(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.crc32_firmware_category))
#define UPDATE_FIRMWARE_CATEGORY_RECV_SIZE				(sizeof(((struct protocal *)NULL)->head))
#define UPDATE_FIRMWARE_CATEGORY_SEND_SIZE				(sizeof(((struct protocal *)NULL)->head))
#define VIEW_NET_ADDR_CATEGORY_RECV_SIZE				(sizeof(((struct protocal *)NULL)->head))
#define VIEW_NET_ADDR_CATEGORY_SEND_SIZE				(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.view_net_addr_category))
#define CLEAR_ALL_PUBKEY_IN_GROUP_CATEGORY_RECV_SIZE	(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.clear_all_pubkey_in_group_category))
#define CLEAR_ALL_PUBKEY_IN_GROUP_CATEGORY_SEND_SIZE	(sizeof(((struct protocal *)NULL)->head))
#define VERSION_VIEW_CATEGORY_RECV_SIZE					(sizeof(((struct protocal *)NULL)->head))
#define VERSION_VIEW_CATEGORY_SEND_SIZE					(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.version_view_category))
#define DEFINE_CONFIG_PASSWORD_CATEGORY_RECV_SIZE		(sizeof(((struct protocal *)NULL)->head)+sizeof(((struct protocal *)NULL)->body.define_config_password_category))
#define DEFINE_CONFIG_PASSWORD_CATEGORY_SEND_SIZE		(sizeof(((struct protocal *)NULL)->head))

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
          uint8_t log_pubkey[128];
          uint8_t flag;
          uint8_t log_time[6];
        } log_category;

      struct
	    {
          uint8_t download_group_no;
          uint8_t download_pubkey[128];
        } download_pubkey_category;

      struct
	    {
          uint8_t upload_pubkey[128];
        } upload_pubkey_category;

      struct {
          uint8_t clear_pubkey[128];
      } clear_pubkey_category;

      struct {
          unsigned char macaddr[6];
      } assign_mac_category;

      struct {
          unsigned char assigned_sn[4];
      } assign_sn_category;

      struct {
          unsigned char hostip[4];
      } assign_hostip_category;

      struct {
          unsigned char netmask[4];
      } assign_netmask_category;

      struct {
          unsigned char gateway[4];
      } assign_gateway_cateory;

      struct {
          unsigned char serverip[4];
      } assign_serverip_category;

      struct {
          unsigned char unlock_flag;
      } unlock_category;

      struct {
          unsigned int view_shock_voltage;
          unsigned int view_lock_voltage;
          unsigned int view_light_voltage;
		unsigned int view_bat_voltage;
		unsigned int view_pow_voltage;
      } sensor_view_category;

      struct {
          unsigned int assign_shock_check_voltage;
      } assign_shock_check_voltage_category;

      struct {
          unsigned int assign_lock_check_voltage;
      } assign_lock_check_voltage_category;

      struct {
          unsigned int assign_light_check_voltage;
      } assign_light_check_voltage_category;

      struct {
          unsigned long firmware_crc32;
          unsigned long firmware_len;
          unsigned long firmware_pos;
          unsigned char firmware[FIRMWARE_CACHE_SIZE];
      } download_firmware_category;

	struct {
		unsigned char view_hostip[4];
          unsigned char view_netmask[4];
          unsigned char view_gateway[4];
          unsigned char view_serverip[4];
          unsigned char view_macaddr[6];
	} view_net_addr_category;

      struct {
          unsigned char crc32_success;
      } crc32_firmware_category;

	struct {
	 	unsigned char clear_group_no;
	} clear_all_pubkey_in_group_category;

	struct {
          unsigned long view_iap_version;
          unsigned long view_app_version;
	} version_view_category;

	struct {
		unsigned char config_password[6];
	} define_config_password_category;
  } body;
};


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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_PROTOCAL */
#endif /* __APPS_INCLUDE_PROTOCAL_H */

