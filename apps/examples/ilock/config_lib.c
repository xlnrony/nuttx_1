/****************************************************************************
 * examples/ilock/config_lib.c
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

#include <apps/inifile.h>
#include <apps/netutils/netlib.h>

#include "config_lib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SETTING_FILE_PATH
#define CONFIG_SETTING_FILE_PATH "/mnt/sd/config.ini"
#endif

#define CONFIG_NET_SECTION_NAME 			"NET"
#define CONFIG_SENSOR_SECTION_NAME 	"SENSOR"
#define CONFIG_KEYSLOT_SECTION_NAME 	"SLOT"

#define CONFIG_MACADDR_VAR_NAME 		"MACADDR"

#define CONFIG_HOSTADDR_VAR_NAME 	"HOSTADDR"
#define CONFIG_NETMASK_VAR_NAME 		"NETMASK"
#define CONFIG_DRIPADDR_VAR_NAME 	"DRIPADDR"
#define CONFIG_SVRADDR_VAR_NAME		"SVRADDR"

#define CONFIG_PUBKEY_VAR_NAME		    "PUBKEY"
#define CONFIG_GROUP_VAR_NAME		    "GROUP"

#define CONFIG_SHOCK_RESISTOR_VAR_NAME			"SHOCK_RESISTOR"
#define CONFIG_INFRA_RED_VAR_NAME						"INFRA_RED"
#define CONFIG_PHOTO_RESISTOR_VAR_NAME			"PHOTO_RESISTOR"

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct config_s *config;

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int nibble2bin(char ascii)
{
  if (ascii >= '0' && ascii <= '9')
    {
      return (ascii - 0x30);
    }
  else if (ascii >= 'a' && ascii <= 'f')
    {
      return (ascii - 'a' + 10);
    }
  else if (ascii >= 'A' && ascii <= 'F')
    {
      return (ascii - 'A' + 10);
    }

  return -EINVAL;
}

static int byte2bin(FAR const char *ascii)
{
  int nibble;
  int byte;

  /* Get the MS nibble (big endian order) */

  nibble = nibble2bin(*ascii++);
  if (nibble < 0)
    {
      return nibble;
    }

  byte = (nibble << 4);

  /* Get the MS nibble */

  nibble = nibble2bin(*ascii);
  if (nibble < 0)
    {
      return nibble;
    }

  byte |= nibble;
  return byte;
}

static int hex2bin(FAR uint8_t * dest, FAR const char *src, int nsrcbytes)
{
  int byte;

  /* An even number of source bytes is expected */

  if ((nsrcbytes & 1) != 0)
    {
      return -EINVAL;
    }

  /* Convert src bytes in groups of 2, writing one byte to the output on each
   * pass through the loop. */

  while (nsrcbytes > 0)
    {
      /* Get the MS nibble (big endian order) */

      byte = byte2bin(src);
      if (byte < 0)
        {
          return byte;
        }

      src += 2;

      /* And write the byte to the destination */

      *dest++ = byte;
      nsrcbytes -= 2;
    }

  return OK;
}

static int bin2hex(FAR char *dest, FAR const uint8_t * src, int nsrcbytes)
{
  while (nsrcbytes > 0)
    {
      sprintf(dest, "%2x", *src);
      src++;
      dest += 2;
      nsrcbytes -= 2;
    }

  return OK;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void load_config(void)
{
  INIHANDLE inifile;
  char *macaddr;
  char *hostaddr;
  char *netmask;
  char *dripaddr;
  char *svraddr;
  char *pubkey;
  int i, j;
  char keyslot[7];
  char group[8];

  inifile = inifile_initialize(CONFIG_SETTING_FILE_PATH);
  if (inifile != NULL)
    {
      macaddr = inifile_read_string(inifile, CONFIG_NET_SECTION_NAME,
                                    CONFIG_MACADDR_VAR_NAME,
                                    CONFIG_MACADDR_DEF_VALUE);
      if (!netlib_hwmacconv(macaddr, config->macaddr))
        {
          config->macaddr[0] = 0xFC;
          config->macaddr[1] = 0xFC;
          config->macaddr[2] = 0xFC;
          config->macaddr[3] = 0xAB;
          config->macaddr[4] = 0xAB;
          config->macaddr[5] = 0xAB;
        }
      inifile_free_string(macaddr);

      hostaddr = inifile_read_string(inifile, CONFIG_NET_SECTION_NAME,
                                     CONFIG_HOSTADDR_VAR_NAME,
                                     CONFIG_HOSTADDR_DEF_VALUE);
      config->hostaddr.s_addr = inet_addr(hostaddr);
      inifile_free_string(hostaddr);

      netmask = inifile_read_string(inifile, CONFIG_NET_SECTION_NAME,
                                    CONFIG_NETMASK_VAR_NAME,
                                    CONFIG_NETMASK_DEF_VALUE);
      config->netmask.s_addr = inet_addr(netmask);
      inifile_free_string(netmask);

      dripaddr = inifile_read_string(inifile, CONFIG_NET_SECTION_NAME,
                                     CONFIG_DRIPADDR_VAR_NAME,
                                     CONFIG_DRIPADDR_DEF_VALUE);
      config->dripaddr.s_addr = inet_addr(dripaddr);
      inifile_free_string(dripaddr);

      svraddr = inifile_read_string(inifile, CONFIG_NET_SECTION_NAME,
                                    CONFIG_SVRADDR_VAR_NAME,
                                    CONFIG_SVRADDR_DEF_VALUE);
      config->svraddr.s_addr = inet_addr(svraddr);
      inifile_free_string(svraddr);

      config->shock_resistor_threshold =
        inifile_read_integer(inifile, CONFIG_NET_SECTION_NAME,
                             CONFIG_SHOCK_RESISTOR_VAR_NAME,
                             CONFIG_SHOCK_RESISTOR_DEF_VALUE);

      config->infra_red_threshold =
        inifile_read_integer(inifile, CONFIG_NET_SECTION_NAME,
                             CONFIG_INFRA_RED_VAR_NAME,
                             CONFIG_INFRA_RED_DEF_VALUE);

      config->infra_red_threshold =
        inifile_read_integer(inifile, CONFIG_NET_SECTION_NAME,
                             CONFIG_INFRA_RED_VAR_NAME,
                             CONFIG_INFRA_RED_DEF_VALUE);

      for (i = 0; i < CONFIG_GROUP_SIZE; i++)
        {
          sprintf(keyslot, CONFIG_KEYSLOT_SECTION_NAME "%d", i);

          pubkey =
            inifile_read_string(inifile, keyslot,
                                CONFIG_PUBKEY_VAR_NAME,
                                CONFIG_PUBKEY_DEF_VALUE);
          hex2bin(config->keyslots[i].pubkey, pubkey,
                  CONFIG_PUBKEY_SIZE * 2);
          inifile_free_string(pubkey);

          for (j = 0; j < CONFIG_GROUP_SIZE; j++)
            {
              sprintf(group, CONFIG_GROUP_VAR_NAME "%d", j);
              config->keyslots[i].group[j] =
                (bool) inifile_read_integer(inifile,
                                            keyslot, group,
                                            CONFIG_GROUP_DEF_VALUE);
            }
        }
      inifile_uninitialize(inifile);
    }
}

void save_config(void)
{
  FILE *stream;
  char pubkey[CONFIG_PUBKEY_SIZE * 2 + 1];
  int i, j;

  stream = fopen(CONFIG_SETTING_FILE_PATH, "w");
  if (stream != NULL)
    {
      fprintf(stream, "; iLock Setting\n");

      fprintf(stream, "[%s]\n", CONFIG_NET_SECTION_NAME);

      fprintf(stream, "  %s=%0.2X:%0.2X:%0.2X:%0.2X:%0.2X:%0.2X\n",
              CONFIG_MACADDR_VAR_NAME, config->macaddr[0],
              config->macaddr[1], config->macaddr[2], config->macaddr[3],
              config->macaddr[4], config->macaddr[5]);
      fprintf(stream, "  %s=%s\n", CONFIG_HOSTADDR_VAR_NAME,
              inet_ntoa(config->hostaddr));
      fprintf(stream, "  %s=%s\n", CONFIG_NETMASK_VAR_NAME,
              inet_ntoa(config->netmask));
      fprintf(stream, "  %s=%s\n", CONFIG_DRIPADDR_VAR_NAME,
              inet_ntoa(config->dripaddr));
      fprintf(stream, "  %s=%s\n", CONFIG_SVRADDR_VAR_NAME,
              inet_ntoa(config->svraddr));

      fprintf(stream, "[%s]\n", CONFIG_SENSOR_SECTION_NAME);

      fprintf(stream, "  %s=%d\n", CONFIG_SHOCK_RESISTOR_VAR_NAME,
              config->shock_resistor_threshold);
      fprintf(stream, "  %s=%d\n", CONFIG_INFRA_RED_VAR_NAME,
              config->infra_red_threshold);
      fprintf(stream, "  %s=%d\n", CONFIG_PHOTO_RESISTOR_VAR_NAME,
              config->photo_resistor_threshold);

      for (i = 0; i < CONFIG_GROUP_SIZE; i++)
        {
          fprintf(stream, "[" CONFIG_KEYSLOT_SECTION_NAME "%d]\n",
                  i);

          bin2hex(pubkey, config->keyslots[i].pubkey,
                  CONFIG_PUBKEY_SIZE);
          fprintf(stream, "  %s=%d\n", CONFIG_PUBKEY_VAR_NAME,
                  pubkey);

          for (j = 0; j < CONFIG_GROUP_SIZE; j++)
            {
              fprintf(stream,
                      "  " CONFIG_GROUP_VAR_NAME "%d=%d\n", j,
                      config->keyslots[i].group[j]);
            }
        }
      fclose(stream);
    }
}

int config_init(void)
{
  int ret;
  config = malloc(sizeof(struct config_s));
  if (config == NULL)
    {
      ret = -errno;
      dbg("config_init: malloc failed: %d\n", ret);
      return ret;
    }
  return OK;
}

void config_deinit(void)
{
  if (config != NULL)
    {
      free(config);
    }
}

