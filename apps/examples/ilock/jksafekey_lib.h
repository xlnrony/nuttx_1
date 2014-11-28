/****************************************************************************
 * examples/ilock/epass3003_lib.h
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

#ifndef __APPS_INCLUDE_JKSAFEKEY_H
#define __APPS_INCLUDE_JKSAFEKEY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#if defined(CONFIG_USBHOST) && defined(CONFIG_USBHOST_JKSAFEKEY)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AT_KEYEXCHANGE          1
#define AT_SIGNATURE            2

#define RV_OK  0
#define RV_FAIL  1
#define RV_FILE_EXIST  2
#define RV_DIR_EXIST  3
#define RV_FILE_NOT_FOUND  4
#define RV_DIR_NOT_FOUND  5
#define RV_ACCESS_DENIED  6
#define RV_PIN_INCORRECT  7
#define RV_PIN_LOCKED  8
#define RV_NOT_SUPPORT  9
#define RV_BUFFER_SMALL  10
#define RV_TOKEN_ABSENT  11

/****************************************************************************
 * Public Types
 ****************************************************************************/
typedef uint16_t PLUG_RV;

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

PLUG_RV jksafekey_verify_pin(int fd, char *pin) ;
PLUG_RV jksafekey_get_pubkey(int fd, uint8_t keyspec, uint8_t* data) ;

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_USBHOST_JKSAFEKEY */
#endif /* __APPS_INCLUDE_JKSAFEKEY_H */

