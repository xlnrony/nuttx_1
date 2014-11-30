/****************************************************************************
 * examples/ilock/led_lib.h
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

#ifndef __APPS_INCLUDE_CONFIG_LIB_H
#define __APPS_INCLUDE_CONFIG_LIB_H

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

#define CONFIG_MACADDR_DEF_VALUE 	"FC:FC:FC:AB:AB:AB"
#define CONFIG_HOSTADDR_DEF_VALUE 	"10.0.0.2"
#define CONFIG_NETMASK_DEF_VALUE 		"255.255.255.0"
#define CONFIG_DRIPADDR_DEF_VALUE 	"10.0.0.1"
#define CONFIG_SVRADDR_DEF_VALUE 		"10.0.0.128"

#define CONFIG_SHOCK_RESISTOR_DEF_VALUE 			0
#define CONFIG_INFRA_RED_DEF_VALUE 						400
#define CONFIG_PHOTO_RESISTOR_DEF_VALUE 			512

#define CONFIG_PUBKEY_DEF_VALUE 		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
#define CONFIG_PUBKEY_SIZE						128

#define CONFIG_GROUP_DEF_VALUE 			0
#define CONFIG_GROUP_SIZE 						32

/****************************************************************************
 * Public Types
 ****************************************************************************/
struct keyslot_s
{
  uint8_t pubkey[CONFIG_PUBKEY_SIZE];
  bool group[CONFIG_GROUP_SIZE];
};

struct config_s
{
  uint32_t serial_no;
  uint8_t macaddr[IFHWADDRLEN];
  struct in_addr hostaddr;
  struct in_addr netmask;
  struct in_addr dripaddr;
  struct in_addr svraddr;
  int32_t shock_resistor_threshold;
  int32_t infra_red_threshold;
  int32_t photo_resistor_threshold;
  struct keyslot_s keyslots[CONFIG_GROUP_SIZE];
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

EXTERN struct config_s *config;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

EXTERN void load_config(void);
EXTERN void save_config(void);
EXTERN int config_init(void);
EXTERN void config_deinit(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __APPS_INCLUDE_CONFIG_LIB_H */

