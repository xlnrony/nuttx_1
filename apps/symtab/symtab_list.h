/* symtab_list.h: Auto-generated symbol table.  Do not edit */

#include <nuttx/config.h>
#include <nuttx/binfmt/symtab.h>

#include <unistd.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <nuttx/math.h>
#include <stdio.h>
#include <fixedmath.h>
#include <libgen.h>
#include <nuttx/fs/fs.h>
#include <nuttx/binfmt/builtin.h>
#include <termios.h>
#include <time.h>
#include <nuttx/clock.h>
#include <dirent.h>
#include <crc32.h>
#include <debug.h>
#include <queue.h>
#include <nuttx/binfmt/elf.h>
#include <netinet/ether.h>
#include <nuttx/binfmt/binfmt.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <nuttx/net/netstats.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <syslog.h>
#include <nuttx/regex.h>
#include <string.h>
#include <sys/stat.h>
#include <nuttx/fs/mkfatfs.h>
#include <sys/mman.h>
#include <nuttx/mmcsd.h>
#include <sys/mount.h>
#include <mqueue.h>
#include <nuttx/net/net.h>
#include <nuttx/binfmt/nxflat.h>
#include <poll.h>
#include <spawn.h>
#include <sys/prctl.h>
#include <pthread.h>
#include <nuttx/fs/ramdisk.h>
#include <sched.h>
#include <nuttx/sched.h>
#include <nuttx/sdio.h>
#include <sys/select.h>
#include <semaphore.h>
#include <sys/sendfile.h>
#include <sys/statfs.h>
#include <assert.h>
#include <nuttx/arch.h>
#include <nuttx/progmem.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_trace.h>
#include <sys/wait.h>

struct symtab_s CONFIG_EXECFUNCS_SYMTAB[] =
{
  { "_exit", (FAR const void *)_exit },
#if !defined(CONFIG_NET_IPv6) && !defined(CONFIG_CAN_PASS_STRUCTS)
  { "_inet_ntoa", (FAR const void *)_inet_ntoa },
#endif
  { "abort", (FAR const void *)abort },
  { "abs", (FAR const void *)abs },
#if CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET)
  { "accept", (FAR const void *)accept },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "acos", (FAR const void *)acos },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "acosf", (FAR const void *)acosf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "acosl", (FAR const void *)acosl },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "asin", (FAR const void *)asin },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "asinf", (FAR const void *)asinf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "asinl", (FAR const void *)asinl },
#endif
  { "asprintf", (FAR const void *)asprintf },
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "atan", (FAR const void *)atan },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "atan2", (FAR const void *)atan2 },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "atan2f", (FAR const void *)atan2f },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "atan2l", (FAR const void *)atan2l },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "atanf", (FAR const void *)atanf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "atanl", (FAR const void *)atanl },
#endif
#if defined(CONFIG_SCHED_ATEXIT)
  { "atexit", (FAR const void *)atexit },
#endif
  { "avsprintf", (FAR const void *)avsprintf },
  { "b16atan2", (FAR const void *)b16atan2 },
  { "b16cos", (FAR const void *)b16cos },
#if !defined(CONFIG_HAVE_LONG_LONG)
  { "b16divb16", (FAR const void *)b16divb16 },
#endif
#if !defined(CONFIG_HAVE_LONG_LONG)
  { "b16mulb16", (FAR const void *)b16mulb16 },
#endif
  { "b16sin", (FAR const void *)b16sin },
#if !defined(CONFIG_HAVE_LONG_LONG)
  { "b16sqr", (FAR const void *)b16sqr },
#endif
  { "basename", (FAR const void *)basename },
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "bchlib_read", (FAR const void *)bchlib_read },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "bchlib_setup", (FAR const void *)bchlib_setup },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "bchlib_teardown", (FAR const void *)bchlib_teardown },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "bchlib_write", (FAR const void *)bchlib_write },
#endif
#if CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET)
  { "bind", (FAR const void *)bind },
#endif
#if defined(CONFIG_BUILTIN)
  { "builtin_exec", (FAR const void *)builtin_exec },
#endif
#if defined(CONFIG_BUILTIN)
  { "builtin_for_index", (FAR const void *)builtin_for_index },
#endif
#if defined(CONFIG_BUILTIN)
  { "builtin_getbuiltins", (FAR const void *)builtin_getbuiltins },
#endif
#if defined(CONFIG_BUILTIN)
  { "builtin_getname", (FAR const void *)builtin_getname },
#endif
#if defined(CONFIG_BUILTIN) && defined(CONFIG_FS_BINFS)
  { "builtin_initialize", (FAR const void *)builtin_initialize },
#endif
#if defined(CONFIG_BUILTIN)
  { "builtin_isavail", (FAR const void *)builtin_isavail },
#endif
#if defined(CONFIG_BUILTIN)
  { "builtin_setbuiltins", (FAR const void *)builtin_setbuiltins },
#endif
#if defined(CONFIG_BUILTIN) && defined(CONFIG_FS_BINFS)
  { "builtin_uninitialize", (FAR const void *)builtin_uninitialize },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "ceil", (FAR const void *)ceil },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "ceilf", (FAR const void *)ceilf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "ceill", (FAR const void *)ceill },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_SERIAL_TERMIOS)
  { "cfgetspeed", (FAR const void *)cfgetspeed },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_SERIAL_TERMIOS)
  { "cfsetspeed", (FAR const void *)cfsetspeed },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_ENVIRON)
  { "chdir", (FAR const void *)chdir },
#endif
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "clearenv", (FAR const void *)clearenv },
#endif
  { "clock_getres", (FAR const void *)clock_getres },
  { "clock_gettime", (FAR const void *)clock_gettime },
  { "clock_settime", (FAR const void *)clock_settime },
  { "clock_systimer", (FAR const void *)clock_systimer },
#if CONFIG_NSOCKET_DESCRIPTORS > 0 || CONFIG_NFILE_DESCRIPTORS > 0
  { "close", (FAR const void *)close },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "closedir", (FAR const void *)closedir },
#endif
#if CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET)
  { "connect", (FAR const void *)connect },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "cos", (FAR const void *)cos },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "cosf", (FAR const void *)cosf },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "cosh", (FAR const void *)cosh },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "coshf", (FAR const void *)coshf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "coshl", (FAR const void *)coshl },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "cosl", (FAR const void *)cosl },
#endif
  { "crc32", (FAR const void *)crc32 },
  { "crc32part", (FAR const void *)crc32part },
#if !defined(CONFIG_CPP_HAVE_VARARGS) && defined(CONFIG_DEBUG)
  { "dbg", (FAR const void *)dbg },
#endif
  { "dirname", (FAR const void *)dirname },
  { "dq_addafter", (FAR const void *)dq_addafter },
  { "dq_addbefore", (FAR const void *)dq_addbefore },
  { "dq_addfirst", (FAR const void *)dq_addfirst },
  { "dq_addlast", (FAR const void *)dq_addlast },
  { "dq_rem", (FAR const void *)dq_rem },
  { "dq_remfirst", (FAR const void *)dq_remfirst },
  { "dq_remlast", (FAR const void *)dq_remlast },
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "dup", (FAR const void *)dup },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "dup2", (FAR const void *)dup2 },
#endif
#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_ELF)
  { "elf_initialize", (FAR const void *)elf_initialize },
#endif
#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_ELF)
  { "elf_uninitialize", (FAR const void *)elf_uninitialize },
#endif
  { "ether_ntoa", (FAR const void *)ether_ntoa },
#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_LIBC_EXECFUNCS)
  { "exec_getsymtab", (FAR const void *)exec_getsymtab },
#endif
#if !defined(CONFIG_BINFMT_DISABLE)
  { "exec_module", (FAR const void *)exec_module },
#endif
#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_LIBC_EXECFUNCS)
  { "exec_setsymtab", (FAR const void *)exec_setsymtab },
#endif
#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_LIBC_EXECFUNCS)
  { "execl", (FAR const void *)execl },
#endif
#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_LIBC_EXECFUNCS)
  { "execv", (FAR const void *)execv },
#endif
  { "exit", (FAR const void *)exit },
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "exp", (FAR const void *)exp },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "expf", (FAR const void *)expf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "expl", (FAR const void *)expl },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "fabs", (FAR const void *)fabs },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "fabsf", (FAR const void *)fabsf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "fabsl", (FAR const void *)fabsl },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "fclose", (FAR const void *)fclose },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "fcntl", (FAR const void *)fcntl },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "fdopen", (FAR const void *)fdopen },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "fflush", (FAR const void *)fflush },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "fgetc", (FAR const void *)fgetc },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "fgetpos", (FAR const void *)fgetpos },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "fgets", (FAR const void *)fgets },
#endif
  { "fileno", (FAR const void *)fileno },
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "floor", (FAR const void *)floor },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "floorf", (FAR const void *)floorf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "floorl", (FAR const void *)floorl },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "fmod", (FAR const void *)fmod },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "fmodf", (FAR const void *)fmodf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "fmodl", (FAR const void *)fmodl },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "fopen", (FAR const void *)fopen },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "fprintf", (FAR const void *)fprintf },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "fputc", (FAR const void *)fputc },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "fputs", (FAR const void *)fputs },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "fread", (FAR const void *)fread },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "frexp", (FAR const void *)frexp },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "frexpf", (FAR const void *)frexpf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "frexpl", (FAR const void *)frexpl },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "fs_fdopen", (FAR const void *)fs_fdopen },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "fseek", (FAR const void *)fseek },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "fsetpos", (FAR const void *)fsetpos },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "fsync", (FAR const void *)fsync },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "ftell", (FAR const void *)ftell },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "fwrite", (FAR const void *)fwrite },
#endif
  { "get_errno", (FAR const void *)get_errno },
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_ENVIRON)
  { "getcwd", (FAR const void *)getcwd },
#endif
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "getenv", (FAR const void *)getenv },
#endif
  { "getopt", (FAR const void *)getopt },
  { "getoptargp", (FAR const void *)getoptargp },
  { "getoptindp", (FAR const void *)getoptindp },
  { "getoptoptp", (FAR const void *)getoptoptp },
  { "getpid", (FAR const void *)getpid },
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "gets", (FAR const void *)gets },
#endif
#if CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET)
  { "getsockopt", (FAR const void *)getsockopt },
#endif
  { "gettimeofday", (FAR const void *)gettimeofday },
  { "gmtime", (FAR const void *)gmtime },
  { "gmtime_r", (FAR const void *)gmtime_r },
  { "htonl", (FAR const void *)htonl },
  { "htons", (FAR const void *)htons },
#if defined(CONFIG_NET_ICMP) && defined(CONFIG_NET_ICMP_PING) && !defined(CONFIG_DISABLE_CLOCK) && !defined(CONFIG_DISABLE_SIGNALS)
  { "icmp_ping", (FAR const void *)icmp_ping },
#endif
  { "imaxabs", (FAR const void *)imaxabs },
  { "inet_addr", (FAR const void *)inet_addr },
#if !defined(CONFIG_NET_IPv6) && defined(CONFIG_CAN_PASS_STRUCTS)
  { "inet_ntoa", (FAR const void *)inet_ntoa },
#endif
  { "inet_ntop", (FAR const void *)inet_ntop },
  { "inet_pton", (FAR const void *)inet_pton },
#if CONFIG_NSOCKET_DESCRIPTORS > 0 || CONFIG_NFILE_DESCRIPTORS > 0
  { "ioctl", (FAR const void *)ioctl },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS)
  { "kill", (FAR const void *)kill },
#endif
  { "labs", (FAR const void *)labs },
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "ldexp", (FAR const void *)ldexp },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "ldexpf", (FAR const void *)ldexpf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "ldexpl", (FAR const void *)ldexpl },
#endif
  { "lib_dumpbuffer", (FAR const void *)lib_dumpbuffer },
#if CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET)
  { "listen", (FAR const void *)listen },
#endif
#if defined(CONFIG_HAVE_LONG_LONG)
  { "llabs", (FAR const void *)llabs },
#endif
#if !defined(CONFIG_CPP_HAVE_VARARGS) && defined(CONFIG_DEBUG) && defined(CONFIG_ARCH_LOWPUTC)
  { "lldbg", (FAR const void *)lldbg },
#endif
#if !defined(CONFIG_CPP_HAVE_VARARGS) && defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_VERBOSE) && defined(CONFIG_ARCH_LOWPUTC)
  { "llvdbg", (FAR const void *)llvdbg },
#endif
#if !defined(CONFIG_BINFMT_DISABLE)
  { "load_module", (FAR const void *)load_module },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "log", (FAR const void *)log },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "log10", (FAR const void *)log10 },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "log10f", (FAR const void *)log10f },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "log10l", (FAR const void *)log10l },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "log2", (FAR const void *)log2 },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "log2f", (FAR const void *)log2f },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "log2l", (FAR const void *)log2l },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "logf", (FAR const void *)logf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "logl", (FAR const void *)logl },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "losetup", (FAR const void *)losetup },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "loteardown", (FAR const void *)loteardown },
#endif
  { "lowsyslog", (FAR const void *)lowsyslog },
#if defined(CONFIG_ARCH_LOWPUTC) || defined(CONFIG_SYSLOG)
  { "lowsyslog", (FAR const void *)lowsyslog },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "lseek", (FAR const void *)lseek },
#endif
  { "match", (FAR const void *)match },
  { "memccpy", (FAR const void *)memccpy },
  { "memchr", (FAR const void *)memchr },
  { "memcmp", (FAR const void *)memcmp },
  { "memcpy", (FAR const void *)memcpy },
  { "memmove", (FAR const void *)memmove },
  { "memset", (FAR const void *)memset },
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "mkdir", (FAR const void *)mkdir },
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_FAT)
  { "mkfatfs", (FAR const void *)mkfatfs },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "mkfifo", (FAR const void *)mkfifo },
#endif
  { "mktime", (FAR const void *)mktime },
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "mmap", (FAR const void *)mmap },
#endif
#if defined (CONFIG_MMCSD) && defined (CONFIG_MMCSD_SDIO)
  { "mmcsd_slotinitialize", (FAR const void *)mmcsd_slotinitialize },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "modf", (FAR const void *)modf },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "modff", (FAR const void *)modff },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "modfl", (FAR const void *)modfl },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_READABLE)
  { "mount", (FAR const void *)mount },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_close", (FAR const void *)mq_close },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_getattr", (FAR const void *)mq_getattr },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS) && !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_notify", (FAR const void *)mq_notify },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_open", (FAR const void *)mq_open },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_receive", (FAR const void *)mq_receive },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_send", (FAR const void *)mq_send },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_setattr", (FAR const void *)mq_setattr },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_timedreceive", (FAR const void *)mq_timedreceive },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_timedsend", (FAR const void *)mq_timedsend },
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
  { "mq_unlink", (FAR const void *)mq_unlink },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS)
  { "nanosleep", (FAR const void *)nanosleep },
#endif
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)
  { "net_clone", (FAR const void *)net_clone },
#endif
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)
  { "netdev_foreach", (FAR const void *)netdev_foreach },
#endif
  { "ntohl", (FAR const void *)ntohl },
  { "ntohs", (FAR const void *)ntohs },
#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_NXFLAT)
  { "nxflat_initialize", (FAR const void *)nxflat_initialize },
#endif
#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_NXFLAT)
  { "nxflat_uninitialize", (FAR const void *)nxflat_uninitialize },
#endif
#if defined(CONFIG_SCHED_ONEXIT)
  { "on_exit", (FAR const void *)on_exit },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "open", (FAR const void *)open },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "opendir", (FAR const void *)opendir },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "perror", (FAR const void *)perror },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "pipe", (FAR const void *)pipe },
#endif
#if !defined(CONFIG_DISABLE_POLL) && (CONFIG_NSOCKET_DESCRIPTORS > 0 || CONFIG_NFILE_DESCRIPTORS > 0)
  { "poll", (FAR const void *)poll },
#endif
#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_LIBC_EXECFUNCS) && !defined(CONFIG_BINFMT_EXEPATH)
  { "posix_spawn", (FAR const void *)posix_spawn },
#endif
#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_LIBC_EXECFUNCS) && defined(CONFIG_BINFMT_EXEPATH)
  { "posix_spawnp", (FAR const void *)posix_spawnp },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "pow", (FAR const void *)pow },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "powf", (FAR const void *)powf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "powl", (FAR const void *)powl },
#endif
#if CONFIG_TASK_NAME_SIZE > 0
  { "prctl", (FAR const void *)prctl },
#endif
  { "printf", (FAR const void *)printf },
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)
  { "psock_close", (FAR const void *)psock_close },
#endif
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)
  { "psock_recvfrom", (FAR const void *)psock_recvfrom },
#endif
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)
  { "psock_send", (FAR const void *)psock_send },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_destroy", (FAR const void *)pthread_attr_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_getinheritsched", (FAR const void *)pthread_attr_getinheritsched },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_getschedparam", (FAR const void *)pthread_attr_getschedparam },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_getschedpolicy", (FAR const void *)pthread_attr_getschedpolicy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_getstacksize", (FAR const void *)pthread_attr_getstacksize },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_init", (FAR const void *)pthread_attr_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_setinheritsched", (FAR const void *)pthread_attr_setinheritsched },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_setschedparam", (FAR const void *)pthread_attr_setschedparam },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_setschedpolicy", (FAR const void *)pthread_attr_setschedpolicy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_attr_setstacksize", (FAR const void *)pthread_attr_setstacksize },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_barrier_destroy", (FAR const void *)pthread_barrier_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_barrier_init", (FAR const void *)pthread_barrier_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_barrier_wait", (FAR const void *)pthread_barrier_wait },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_barrierattr_destroy", (FAR const void *)pthread_barrierattr_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_barrierattr_getpshared", (FAR const void *)pthread_barrierattr_getpshared },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_barrierattr_init", (FAR const void *)pthread_barrierattr_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_barrierattr_setpshared", (FAR const void *)pthread_barrierattr_setpshared },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cancel", (FAR const void *)pthread_cancel },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_broadcast", (FAR const void *)pthread_cond_broadcast },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_destroy", (FAR const void *)pthread_cond_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_init", (FAR const void *)pthread_cond_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_signal", (FAR const void *)pthread_cond_signal },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS) && !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_timedwait", (FAR const void *)pthread_cond_timedwait },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_cond_wait", (FAR const void *)pthread_cond_wait },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_condattr_destroy", (FAR const void *)pthread_condattr_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_condattr_init", (FAR const void *)pthread_condattr_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_create", (FAR const void *)pthread_create },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_detach", (FAR const void *)pthread_detach },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_exit", (FAR const void *)pthread_exit },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_getschedparam", (FAR const void *)pthread_getschedparam },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_getspecific", (FAR const void *)pthread_getspecific },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_join", (FAR const void *)pthread_join },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_key_create", (FAR const void *)pthread_key_create },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_key_delete", (FAR const void *)pthread_key_delete },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS) && !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_kill", (FAR const void *)pthread_kill },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutex_destroy", (FAR const void *)pthread_mutex_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutex_init", (FAR const void *)pthread_mutex_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutex_lock", (FAR const void *)pthread_mutex_lock },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutex_trylock", (FAR const void *)pthread_mutex_trylock },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutex_unlock", (FAR const void *)pthread_mutex_unlock },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutexattr_destroy", (FAR const void *)pthread_mutexattr_destroy },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutexattr_getpshared", (FAR const void *)pthread_mutexattr_getpshared },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && defined(CONFIG_MUTEX_TYPES)
  { "pthread_mutexattr_gettype", (FAR const void *)pthread_mutexattr_gettype },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutexattr_init", (FAR const void *)pthread_mutexattr_init },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_mutexattr_setpshared", (FAR const void *)pthread_mutexattr_setpshared },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD) && defined(CONFIG_MUTEX_TYPES)
  { "pthread_mutexattr_settype", (FAR const void *)pthread_mutexattr_settype },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_once", (FAR const void *)pthread_once },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_setcancelstate", (FAR const void *)pthread_setcancelstate },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_setschedparam", (FAR const void *)pthread_setschedparam },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_setschedprio", (FAR const void *)pthread_setschedprio },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_setspecific", (FAR const void *)pthread_setspecific },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS) && !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_sigmask", (FAR const void *)pthread_sigmask },
#endif
#if !defined(CONFIG_DISABLE_PTHREAD)
  { "pthread_yield", (FAR const void *)pthread_yield },
#endif
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "putenv", (FAR const void *)putenv },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "puts", (FAR const void *)puts },
#endif
  { "qsort", (FAR const void *)qsort },
#if defined(CONFIG_FS_WRITABLE)
  { "ramdisk_register", (FAR const void *)ramdisk_register },
#endif
  { "rand", (FAR const void *)rand },
#if CONFIG_NSOCKET_DESCRIPTORS > 0 || CONFIG_NFILE_DESCRIPTORS > 0
  { "read", (FAR const void *)read },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "readdir", (FAR const void *)readdir },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "readdir_r", (FAR const void *)readdir_r },
#endif
#if CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET)
  { "recv", (FAR const void *)recv },
#endif
#if CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET)
  { "recvfrom", (FAR const void *)recvfrom },
#endif
  { "register_driver", (FAR const void *)register_driver },
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "rename", (FAR const void *)rename },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "rewinddir", (FAR const void *)rewinddir },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "rint", (FAR const void *)rint },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "rintf", (FAR const void *)rintf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "rintl", (FAR const void *)rintl },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "rmdir", (FAR const void *)rmdir },
#endif
#if !defined(CONFIG_FS_WRITABLE)
  { "romdisk_register", (FAR const void *)romdisk_register },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "round", (FAR const void *)round },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "roundf", (FAR const void *)roundf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "roundl", (FAR const void *)roundl },
#endif
  { "sched_foreach", (FAR const void *)sched_foreach },
  { "sched_get_priority_max", (FAR const void *)sched_get_priority_max },
  { "sched_get_priority_min", (FAR const void *)sched_get_priority_min },
  { "sched_getparam", (FAR const void *)sched_getparam },
  { "sched_getscheduler", (FAR const void *)sched_getscheduler },
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "sched_getstreams", (FAR const void *)sched_getstreams },
#endif
  { "sched_lock", (FAR const void *)sched_lock },
  { "sched_lockcount", (FAR const void *)sched_lockcount },
  { "sched_rr_get_interval", (FAR const void *)sched_rr_get_interval },
  { "sched_setparam", (FAR const void *)sched_setparam },
  { "sched_setscheduler", (FAR const void *)sched_setscheduler },
  { "sched_unlock", (FAR const void *)sched_unlock },
  { "sched_yield", (FAR const void *)sched_yield },
#if defined (CONFIG_MMCSD) && defined (CONFIG_MMCSD_SDIO)
  { "sdio_initialize", (FAR const void *)sdio_initialize },
#endif
#if defined (CONFIG_MMCSD) && defined (CONFIG_MMCSD_SDIO)
  { "sdio_mediachange", (FAR const void *)sdio_mediachange },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "seekdir", (FAR const void *)seekdir },
#endif
#if !defined(CONFIG_DISABLE_POLL) && (CONFIG_NSOCKET_DESCRIPTORS > 0 || CONFIG_NFILE_DESCRIPTORS > 0)
  { "select", (FAR const void *)select },
#endif
  { "sem_close", (FAR const void *)sem_close },
  { "sem_destroy", (FAR const void *)sem_destroy },
  { "sem_getvalue", (FAR const void *)sem_getvalue },
  { "sem_init", (FAR const void *)sem_init },
  { "sem_open", (FAR const void *)sem_open },
  { "sem_post", (FAR const void *)sem_post },
  { "sem_timedwait", (FAR const void *)sem_timedwait },
  { "sem_trywait", (FAR const void *)sem_trywait },
  { "sem_unlink", (FAR const void *)sem_unlink },
  { "sem_wait", (FAR const void *)sem_wait },
#if CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET)
  { "send", (FAR const void *)send },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_NET_SENDFILE)
  { "sendfile", (FAR const void *)sendfile },
#endif
#if CONFIG_NSOCKET_DESCRIPTORS > 0 || CONFIG_NFILE_DESCRIPTORS > 0
  { "sendfile", (FAR const void *)sendfile },
#endif
#if CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET)
  { "sendto", (FAR const void *)sendto },
#endif
  { "set_errno", (FAR const void *)set_errno },
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "setenv", (FAR const void *)setenv },
#endif
#if CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET)
  { "setsockopt", (FAR const void *)setsockopt },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS)
  { "sigaction", (FAR const void *)sigaction },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS)
  { "sigaddset", (FAR const void *)sigaddset },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS)
  { "sigdelset", (FAR const void *)sigdelset },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS)
  { "sigemptyset", (FAR const void *)sigemptyset },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS)
  { "sigfillset", (FAR const void *)sigfillset },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS)
  { "sigismember", (FAR const void *)sigismember },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS)
  { "sigpending", (FAR const void *)sigpending },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS)
  { "sigprocmask", (FAR const void *)sigprocmask },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS)
  { "sigqueue", (FAR const void *)sigqueue },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS)
  { "sigsuspend", (FAR const void *)sigsuspend },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS)
  { "sigtimedwait", (FAR const void *)sigtimedwait },
#endif
#if !defined(CONFIG_DISABLE_SIGNALS)
  { "sigwaitinfo", (FAR const void *)sigwaitinfo },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "sin", (FAR const void *)sin },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "sinf", (FAR const void *)sinf },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "sinh", (FAR const void *)sinh },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "sinhf", (FAR const void *)sinhf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "sinhl", (FAR const void *)sinhl },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "sinl", (FAR const void *)sinl },
#endif
  { "snprintf", (FAR const void *)snprintf },
#if CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET)
  { "socket", (FAR const void *)socket },
#endif
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)
  { "sockfd_socket", (FAR const void *)sockfd_socket },
#endif
  { "sprintf", (FAR const void *)sprintf },
  { "sq_addafter", (FAR const void *)sq_addafter },
  { "sq_addfirst", (FAR const void *)sq_addfirst },
  { "sq_addlast", (FAR const void *)sq_addlast },
  { "sq_rem", (FAR const void *)sq_rem },
  { "sq_remafter", (FAR const void *)sq_remafter },
  { "sq_remfirst", (FAR const void *)sq_remfirst },
  { "sq_remlast", (FAR const void *)sq_remlast },
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "sqrt", (FAR const void *)sqrt },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "sqrtf", (FAR const void *)sqrtf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "sqrtl", (FAR const void *)sqrtl },
#endif
  { "srand", (FAR const void *)srand },
  { "sscanf", (FAR const void *)sscanf },
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "stat", (FAR const void *)stat },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "statfs", (FAR const void *)statfs },
#endif
  { "strcasecmp", (FAR const void *)strcasecmp },
  { "strcasestr", (FAR const void *)strcasestr },
  { "strcat", (FAR const void *)strcat },
  { "strchr", (FAR const void *)strchr },
  { "strcmp", (FAR const void *)strcmp },
  { "strcpy", (FAR const void *)strcpy },
  { "strcspn", (FAR const void *)strcspn },
  { "strdup", (FAR const void *)strdup },
  { "strerror", (FAR const void *)strerror },
  { "strftime", (FAR const void *)strftime },
  { "strlen", (FAR const void *)strlen },
  { "strncasecmp", (FAR const void *)strncasecmp },
  { "strncat", (FAR const void *)strncat },
  { "strncmp", (FAR const void *)strncmp },
  { "strncpy", (FAR const void *)strncpy },
  { "strndup", (FAR const void *)strndup },
  { "strnlen", (FAR const void *)strnlen },
  { "strpbrk", (FAR const void *)strpbrk },
  { "strrchr", (FAR const void *)strrchr },
  { "strspn", (FAR const void *)strspn },
  { "strstr", (FAR const void *)strstr },
  { "strtod", (FAR const void *)strtod },
  { "strtok", (FAR const void *)strtok },
  { "strtok_r", (FAR const void *)strtok_r },
  { "strtol", (FAR const void *)strtol },
#if defined(CONFIG_HAVE_LONG_LONG)
  { "strtoll", (FAR const void *)strtoll },
#endif
  { "strtoul", (FAR const void *)strtoul },
#if defined(CONFIG_HAVE_LONG_LONG)
  { "strtoull", (FAR const void *)strtoull },
#endif
  { "syslog", (FAR const void *)syslog },
#if defined(CONFIG_SYSLOG_ENABLE)
  { "syslog_enable", (FAR const void *)syslog_enable },
#endif
#if defined(CONFIG_SYSLOG_ENABLE)
  { "syslog_enable", (FAR const void *)syslog_enable },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "tan", (FAR const void *)tan },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "tanf", (FAR const void *)tanf },
#endif
#if defined(CONFIG_HAVE_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "tanh", (FAR const void *)tanh },
#endif
#if defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH)
  { "tanhf", (FAR const void *)tanhf },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "tanhl", (FAR const void *)tanhl },
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE) && (defined(CONFIG_LIBM) || defined(CONFIG_ARCH_MATH))
  { "tanl", (FAR const void *)tanl },
#endif
  { "task_create", (FAR const void *)task_create },
  { "task_delete", (FAR const void *)task_delete },
  { "task_restart", (FAR const void *)task_restart },
  { "task_spawn", (FAR const void *)task_spawn },
#if CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_SERIAL_TERMIOS)
  { "tcflush", (FAR const void *)tcflush },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_SERIAL_TERMIOS)
  { "tcgetattr", (FAR const void *)tcgetattr },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_SERIAL_TERMIOS)
  { "tcsetattr", (FAR const void *)tcsetattr },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "telldir", (FAR const void *)telldir },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
  { "telldir", (FAR const void *)telldir },
#endif
  { "time", (FAR const void *)time },
#if !defined(CONFIG_DISABLE_POSIX_TIMERS)
  { "timer_create", (FAR const void *)timer_create },
#endif
#if !defined(CONFIG_DISABLE_POSIX_TIMERS)
  { "timer_delete", (FAR const void *)timer_delete },
#endif
#if !defined(CONFIG_DISABLE_POSIX_TIMERS)
  { "timer_getoverrun", (FAR const void *)timer_getoverrun },
#endif
#if !defined(CONFIG_DISABLE_POSIX_TIMERS)
  { "timer_gettime", (FAR const void *)timer_gettime },
#endif
#if !defined(CONFIG_DISABLE_POSIX_TIMERS)
  { "timer_settime", (FAR const void *)timer_settime },
#endif
#if !defined(CONFIG_HAVE_LONG_LONG)
  { "ub16divub16", (FAR const void *)ub16divub16 },
#endif
#if !defined(CONFIG_HAVE_LONG_LONG)
  { "ub16mulub16", (FAR const void *)ub16mulub16 },
#endif
#if !defined(CONFIG_HAVE_LONG_LONG)
  { "ub16sqr", (FAR const void *)ub16sqr },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "umount", (FAR const void *)umount },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "ungetc", (FAR const void *)ungetc },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT)
  { "unlink", (FAR const void *)unlink },
#endif
#if !defined(CONFIG_BINFMT_DISABLE)
  { "unload_module", (FAR const void *)unload_module },
#endif
  { "unregister_driver", (FAR const void *)unregister_driver },
#if !defined(CONFIG_DISABLE_ENVIRON)
  { "unsetenv", (FAR const void *)unsetenv },
#endif
  { "up_assert", (FAR const void *)up_assert },
#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_STACK)
  { "up_check_tcbstack", (FAR const void *)up_check_tcbstack },
#endif
#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)
  { "up_cxxinitialize", (FAR const void *)up_cxxinitialize },
#endif
  { "up_progmem_erasepage", (FAR const void *)up_progmem_erasepage },
  { "up_progmem_getaddr", (FAR const void *)up_progmem_getaddr },
  { "up_progmem_getpage", (FAR const void *)up_progmem_getpage },
  { "up_progmem_ispageerased", (FAR const void *)up_progmem_ispageerased },
  { "up_progmem_isuniform", (FAR const void *)up_progmem_isuniform },
  { "up_progmem_pagesize", (FAR const void *)up_progmem_pagesize },
  { "up_progmem_write", (FAR const void *)up_progmem_write },
#if defined(CONFIG_USBHOST)
  { "usbhost_connection_enumerate", (FAR const void *)usbhost_connection_enumerate },
#endif
#if defined(CONFIG_USBHOST)
  { "usbhost_connection_wait", (FAR const void *)usbhost_connection_wait },
#endif
#if defined(CONFIG_USBHOST) && defined(CONFIG_USBHOST_EPASS3003)
  { "usbhost_epass3003init", (FAR const void *)usbhost_epass3003init },
#endif
#if defined(CONFIG_USBHOST)
  { "usbhost_initialize", (FAR const void *)usbhost_initialize },
#endif
#if defined(CONFIG_USBHOST) && defined(CONFIG_USBHOST_HIDKBD)
  { "usbhost_kbdinit", (FAR const void *)usbhost_kbdinit },
#endif
#if defined(CONFIG_USBHOST_TRACE)
  { "usbhost_trdump", (FAR const void *)usbhost_trdump },
#endif
#if !defined(CONFIG_CPP_HAVE_VARARGS) && defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_VERBOSE)
  { "vdbg", (FAR const void *)vdbg },
#endif
#if defined(CONFIG_ARCH_HAVE_VFORK)
  { "vfork", (FAR const void *)vfork },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "vfprintf", (FAR const void *)vfprintf },
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
  { "vprintf", (FAR const void *)vprintf },
#endif
  { "vsnprintf", (FAR const void *)vsnprintf },
  { "vsprintf", (FAR const void *)vsprintf },
  { "vsscanf", (FAR const void *)vsscanf },
#if defined(CONFIG_SCHED_WAITPID) && defined(CONFIG_SCHED_HAVE_PARENT)
  { "wait", (FAR const void *)wait },
#endif
#if defined(CONFIG_SCHED_WAITPID) && defined(CONFIG_SCHED_HAVE_PARENT)
  { "waitid", (FAR const void *)waitid },
#endif
#if defined(CONFIG_SCHED_WAITPID)
  { "waitpid", (FAR const void *)waitpid },
#endif
#if CONFIG_NSOCKET_DESCRIPTORS > 0 || CONFIG_NFILE_DESCRIPTORS > 0
  { "write", (FAR const void *)write }
#endif
};

#define NSYMBOLS (sizeof(CONFIG_EXECFUNCS_SYMTAB) / sizeof (struct symtab_s))
