/****************************************************************************
 * examples/ilock/adc_lib.h
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

#ifndef __APPS_INCLUDE_ILOCK_DEBUG_H
#define __APPS_INCLUDE_ILOCK_DEBUG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS

/* C-99 style variadic macros are supported */

#  ifdef CONFIG_DEBUG_ILOCK
#    define ilockdbg(format, ...)    dbg(format, ##__VA_ARGS__)
#    define ilocklldbg(format, ...)  lldbg(format, ##__VA_ARGS__)
#    define ilockvdbg(format, ...)   vdbg(format, ##__VA_ARGS__)
#    define ilockllvdbg(format, ...) llvdbg(format, ##__VA_ARGS__)
#  else
#    define ilockdbg(x...)
#    define ilocklldbg(x...)
#    define ilockvdbg(x...)
#    define ilockllvdbg(x...)
#  endif

#else                                  /* CONFIG_CPP_HAVE_VARARGS */

/* Variadic macros NOT supported */

#  ifdef CONFIG_DEBUG_ILOCK
#    define ilockdbg     dbg
#    define ilockvdbg   vdbg
#    define ilocklldbg   lldbg
#    define ilockllvdbg llvdbg
#  else
#    define ilockdbg 		(void)
#    define ilockvdbg		(void)
#    define ilocklldbg		(void)
#    define ilockllvdbg	(void)
#  endif

#endif                                 /* CONFIG_CPP_HAVE_VARARGS */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#endif /* __APPS_INCLUDE_ILOCK_DEBUG_H */

