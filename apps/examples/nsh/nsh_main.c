/****************************************************************************
 * examples/nsh/nsh_main.c
 *
 *   Copyright (C) 2007-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

#include <sys/stat.h>
#include <stdint.h>
#include <stdio.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/arch.h>

#if defined(CONFIG_BUILTIN)
#  include <apps/builtin.h>
#  if defined(CONFIG_FS_BINFS)
#    include <nuttx/binfmt/builtin.h>
#  endif
#endif

#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_NXFLAT)
#  include <nuttx/binfmt/nxflat.h>
#endif

#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_ELF)
#  include <nuttx/binfmt/elf.h>
#endif

#if defined(CONFIG_LIBC_EXECFUNCS) && defined(CONFIG_EXECFUNCS_SYMTAB)
#  include <nuttx/binfmt/symtab.h>
#  include <apps/symtab.h>
#endif

#include <apps/nsh.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* C++ initialization requires CXX initializer support */

#if !defined(CONFIG_HAVE_CXX) || !defined(CONFIG_HAVE_CXXINITIALIZE)
#  undef CONFIG_EXAMPLES_NSH_CXXINITIALIZE
#endif

/* The NSH telnet console requires networking support (and TCP/IP) */

#ifndef CONFIG_NET
#  undef CONFIG_NSH_TELNET
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_main
 ****************************************************************************/

int nsh_main(int argc, char *argv[])
{
  int exitval = 0;
  int ret;

  /* Call all C++ static constructors */

#if defined(CONFIG_EXAMPLES_NSH_CXXINITIALIZE)
  up_cxxinitialize();
#endif

  /* Make sure that we are using our symbol table */

#if defined(CONFIG_LIBC_EXECFUNCS) && defined(CONFIG_EXECFUNCS_SYMTAB)
  symtab_list_initialize();
#endif

  /* Register the BINFS file system */

#if defined(CONFIG_BUILTIN)
  builtin_list_initialize();
  #if defined(CONFIG_FS_BINFS)
    ret = builtin_initialize();
    if (ret < 0)
    {
      fprintf(stderr, "ERROR: builtin_initialize failed: %d\n", ret);
      exitval = 1;
    }  
  #endif
#endif

#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_NXFLAT)
    ret = nxflat_initialize();
    if (ret < 0)
    {
      fprintf(stderr, "ERROR: nxflat_initialize failed: %d\n", ret);
      exitval = 1;
    }  
#endif

#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_ELF)
    ret = elf_initialize();
    if (ret < 0)
    {
      fprintf(stderr, "ERROR: elf_initialize failed: %d\n", ret);
      exitval = 1;
    }  
#endif

  /* Initialize the NSH library */

  nsh_initialize();

  /* If the Telnet console is selected as a front-end, then start the
   * Telnet daemon.
   */

#ifdef CONFIG_NSH_TELNET
  ret = nsh_telnetstart();
  if (ret < 0)
    {
     /* The daemon is NOT running.  Report the the error then fail...
      * either with the serial console up or just exiting.
      */

     fprintf(stderr, "ERROR: Failed to start TELNET daemon: %d\n", ret);
     exitval = 1;
   }
#endif

  /* If the serial console front end is selected, then run it on this thread */

#ifdef CONFIG_NSH_CONSOLE
  ret = nsh_consolemain(0, NULL);

  /* nsh_consolemain() should not return.  So if we get here, something
   * is wrong.
   */

#if CONFIG_NFILE_DESCRIPTORS > 0
  fprintf(stderr, "ERROR: nsh_consolemain() returned: %d\n", ret);
#else
  printf("ERROR: nsh_consolemain() returned: %d\n", ret);
#endif

  exitval = 1;
#endif

  return exitval;
}
