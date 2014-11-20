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

#include <apps/inifile.h>

#include "config_lib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SETTING_FILE_PATH
#define CONFIG_SETTING_FILE_PATH "/mnt/sd/config.ini"
#endif

#define CONFIG_NET_SECTION_NAME 			"NET"
#define CONFIG_SENSOR_SECTION_NAME 	"SENSOR"

#define CONFIG_MACADDR_VAR_NAME 		"MACADDR"

#define CONFIG_HOSTADDR_VAR_NAME 	"HOSTADDR"
#define CONFIG_NETMASK_VAR_NAME 		"NETMASK"
#define CONFIG_DRIPADDR_VAR_NAME 	"DRIPADDR"
#define CONFIG_SVRADDR_VAR_NAME		"SVRADDR"

#define CONFIG_SHOCK_VAR_NAME			"SHOCK"
#define CONFIG_LOCK_VAR_NAME				"LOCK"
#define CONFIG_LIGHT_VAR_NAME				"LIGHT"

#define CONFIG_MACADDR_DEF_VALUE 	"FC:FC:FC:AB:AB:AB"
#define CONFIG_HOSTADDR_DEF_VALUE 	"10.0.0.2"
#define CONFIG_NETMASK_DEF_VALUE 		"255.255.255.0"
#define CONFIG_DRIPADDR_DEF_VALUE 	"10.0.0.1"
#define CONFIG_SVRADDR_DEF_VALUE 		"10.0.0.128"

#define CONFIG_SHOCK_DEF_VALUE 			0
#define CONFIG_LOCK_DEF_VALUE 				400
#define CONFIG_LIGHT_DEF_VALUE 			512

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct config_s config;

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
 
void load_config(void)
{
  INIHANDLE 	inifile;
  char * 			macaddr;
  char * 			hostaddr;
  char * 			netmask;
  char * 			dripaddr;
  char * 			svraddr;
	
  inifile = inifile_initialize(CONFIG_SETTING_FILE_PATH);
  if (inifile != NULL)
  	{
	  macaddr = inifile_read_string(inifile, CONFIG_NET_SECTION_NAME, 
															 			CONFIG_MACADDR_VAR_NAME, 
															 			CONFIG_MACADDR_DEF_VALUE);
	  if (!netlib_hwmacconv(macaddr, config.macaddr))
	  	{
		  config.macaddr[0] = 0xFC;
		  config.macaddr[1] = 0xFC;
		  config.macaddr[2] = 0xFC;
		  config.macaddr[3] = 0xAB;
		  config.macaddr[4] = 0xAB;
		  config.macaddr[5] = 0xAB;
	  	}
	  inifile_free_string(macaddr);

	  hostaddr = inifile_read_string(inifile, CONFIG_NET_SECTION_NAME, 
															 			CONFIG_HOSTADDR_VAR_NAME, 
															 			CONFIG_HOSTADDR_DEF_VALUE);
	  config.hostaddr.s_addr = inet_addr(hostaddr);
	  inifile_free_string(hostaddr);

	  netmask = inifile_read_string(inifile, CONFIG_NET_SECTION_NAME, 
															 			CONFIG_NETMASK_VAR_NAME, 
															 			CONFIG_NETMASK_DEF_VALUE);
	  config.netmask.s_addr = inet_addr(netmask);
	  inifile_free_string(netmask);

	  dripaddr = inifile_read_string(inifile, CONFIG_NET_SECTION_NAME, 
															 			CONFIG_DRIPADDR_VAR_NAME, 
															 			CONFIG_DRIPADDR_DEF_VALUE);
	  config.dripaddr.s_addr = inet_addr(dripaddr);
	  inifile_free_string(dripaddr);
		
	  svraddr = inifile_read_string(inifile, CONFIG_NET_SECTION_NAME, 
															 			CONFIG_SVRADDR_VAR_NAME, 
															 			CONFIG_SVRADDR_DEF_VALUE);
	  config.svraddr.s_addr = inet_addr(svraddr);
	  inifile_free_string(svraddr);

	  config.shockthreshold = inifile_read_integer(inifile, CONFIG_NET_SECTION_NAME, 
                                                                                     CONFIG_SHOCK_VAR_NAME, 
															 			                      CONFIG_SHOCK_DEF_VALUE);

	  config.lockthreshold = inifile_read_integer(inifile, CONFIG_NET_SECTION_NAME, 
                                                                                     CONFIG_LOCK_VAR_NAME, 
															 			                      CONFIG_LOCK_DEF_VALUE);

	  config.lightthreshold = inifile_read_integer(inifile, CONFIG_NET_SECTION_NAME, 
                                                                                     CONFIG_LIGHT_VAR_NAME, 
															 			                      CONFIG_LIGHT_DEF_VALUE);
	  inifile_uninitialize(inifile);
  	}
}

void save_config(void)
{
  FILE *stream;

  stream = fopen(CONFIG_SETTING_FILE_PATH, "w");
  if (stream != NULL)
  	{
	  fprintf(stream, "; iLock Setting\n");
		
	  fprintf(stream, "[%s]\n", CONFIG_NET_SECTION_NAME);
		
	  fprintf(stream, "  %s=%0.2X:%0.2X:%0.2X:%0.2X:%0.2X:%0.2X\n", CONFIG_MACADDR_VAR_NAME, 
				   config.macaddr[0], config.macaddr[1], config.macaddr[2], config.macaddr[3], config.macaddr[4], config.macaddr[5]);
	  fprintf(stream, "  %s=%s\n", CONFIG_HOSTADDR_VAR_NAME, inet_ntoa(config.hostaddr));
	  fprintf(stream, "  %s=%s\n", CONFIG_NETMASK_VAR_NAME, inet_ntoa(config.netmask));
	  fprintf(stream, "  %s=%s\n", CONFIG_DRIPADDR_VAR_NAME, inet_ntoa(config.dripaddr));
	  fprintf(stream, "  %s=%s\n", CONFIG_SVRADDR_VAR_NAME, inet_ntoa(config.svraddr));

	  fprintf(stream, "[%s]\n", CONFIG_SENSOR_SECTION_NAME);
		
	  fprintf(stream, "  %s=%d\n", CONFIG_SHOCK_VAR_NAME, config.shockthreshold);
	  fprintf(stream, "  %s=%d\n", CONFIG_LOCK_VAR_NAME, config.lockthreshold);
	  fprintf(stream, "  %s=%d\n", CONFIG_LIGHT_VAR_NAME, config.lightthreshold);
		
	  fclose(stream);
  	}
}

