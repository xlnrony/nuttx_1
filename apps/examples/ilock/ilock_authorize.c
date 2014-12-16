/****************************************************************************
 * examples/ilock/ilock_authorize.c
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
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <nuttx/clock.h>
#include <nuttx/gpio/indicator.h>

#include "ilock_protocal.h"
#include "ilock_authorize.h"

#include "led_lib.h"
#include "config_lib.h"
#include "file_lib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct log_file_block_s
{
  int32_t log_sn;
  uint8_t log_group[CONFIG_GROUP_SIZE];
  uint8_t log_pubkey[CONFIG_PUBKEY_SIZE];
  uint8_t type;
  uint8_t time[6];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_last_time = 0;
static bool g_group[CONFIG_GROUP_SIZE] = { true };
static bool g_check[CONFIG_GROUP_SIZE] = { false };

static uint8_t g_unlock_type = LOG_UNLOCK;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void auth_send_log_to_disk_or_net(int sockfd, uint8_t log_type, uint8_t *pubkey)
{
  int ret;
  int i;
  struct log_file_block_s log_file_block;
  struct timespec ts;
  struct tm *tm;

  char file_path[25];
  //unsigned long time;

  if (log_type != 0)
    {
      clock_gettime(CLOCK_REALTIME, &ts);
      tm = gmtime(&ts.tv_sec);

      log_file_block.time[0] = tm->tm_year;
      log_file_block.time[1] = tm->tm_mon;
      log_file_block.time[2] = tm->tm_mday;
      log_file_block.time[3] = tm->tm_hour;
      log_file_block.time[4] = tm->tm_min;
      log_file_block.time[5] = tm->tm_sec;

      ret = protocal_send_log(sockfd, config->serial_no, g_group, pubkey, log_type, log_file_block.time);
      if (ret < 0)
        {
          log_file_block.log_sn = config->serial_no;
          for(i = 0; i < CONFIG_GROUP_SIZE; i++)
            {
              log_file_block.log_group[i] = g_group[i];
            }
          memcpy(log_file_block.log_pubkey, pubkey, CONFIG_PUBKEY_SIZE);
          log_file_block.type = log_type;

          sprintf(file_path, "/mnt/sd/log/%8x.log",clock_systimer());

          ret = filewrite(file_path, 0, &log_file_block, sizeof(struct log_file_block_s));
          if (ret < 0)
            {
              led1_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(200));
              led1_op(INDC_TWINKLE, IND_RED, SEC2TICK(3), MSEC2TICK(200));
              led1_op(INDC_ALWAYS, IND_RED, SEC2TICK(3), MSEC2TICK(200));
            }
        }
    }
}

void auth_send_log_to_disk_or_net_by_unlock_type(int sockfd, uint8_t *pubkey)
{
  auth_send_log_to_disk_or_net(sockfd, g_unlock_type, pubkey);
}

void auth_init(void)
{
  int i;
  g_unlock_type = LOG_UNLOCK;
  for (i = 0; i < CONFIG_GROUP_SIZE; i++)
    {
      g_group[i] = true;
      g_check[i] = false;
    }
}

void auth_time_out_check(void)
{
  if (ABS(clock_systimer() - g_last_time) > SEC2TICK(600))
    {
      auth_init();
    }
}

void auth_set_temp_unlock(int sockfd, uint8_t *pubkey)
{
  g_unlock_type = LOG_TEMP_UNLOCK;
  g_last_time = clock_systimer();
  auth_send_log_to_disk_or_net(sockfd, LOG_HALF_UNLOCK, pubkey);
}

void auth_set_auth_unlock(int sockfd, uint8_t *pubkey)
{
  g_unlock_type = LOG_AUTH_UNLOCK;
  auth_send_log_to_disk_or_net(sockfd, LOG_HALF_UNLOCK, pubkey);
}

bool auth_this_time(uint8_t *pubkey)
{
  int i, j;
  bool ret = false;
  for (i = 0; i < CONFIG_GROUP_SIZE; i++)
    {
      if (memcmp(config->keyslots[i].pubkey, pubkey, CONFIG_PUBKEY_SIZE) == 0)
        {
          break;
        }
    }
  if (i < CONFIG_GROUP_SIZE)
    {
      for (j = 0; j < CONFIG_GROUP_SIZE; j++)
        {
          if (!config->keyslots[i].group[j])
            {
              g_group[j] = false;
            }
          if (g_group[j])
            {
              ret = true;
            }
        }
    }
  if (ret)
    {
      g_last_time = clock_systimer();
      g_check[i] = true;
    }
  return ret;
}

bool auth_if_half_unlock(void)
{
  int i;
  for (i = 0; i <= CONFIG_GROUP_SIZE - 1; i++)
    {
      if (g_check[i])
        return true;
    }
  return false;
}

bool auth_need_more(void)
{
  int i, j;

  if (g_unlock_type == LOG_TEMP_UNLOCK)
    {
      return false;
    }
  for (i = 0; i < CONFIG_GROUP_SIZE; i++)
    {
      if (g_group[i])
        {
          for (j = 0; j < CONFIG_GROUP_SIZE; j++)
            {
              if (g_check[j])
                {
                  continue;
                }

              if (config->keyslots[j].group[i])
                {
                  return true;
                }
            }
        }
    }
  return false;
}

