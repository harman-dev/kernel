/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2014 - 2015 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  SoC Watch Developer Team <socwatchdevelopers@intel.com>
  Intel Corporation,
  1906 Fox Drive,
  Champaign, IL 61820

  BSD LICENSE

  Copyright(c) 2014 - 2015 Intel Corporation.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#ifndef __SPIOCTL_H__
#define __SPIOCTL_H__

#include "pw_types.h"

#if defined (__linux__)
    #if __KERNEL__
        #include <linux/ioctl.h>
        #if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
            #include <linux/compat.h>
        #endif // COMPAT && x64
    #else // !__KERNEL__
        #include <sys/ioctl.h>
    #endif // __KERNEL__
#endif // __linux__
/*
 * Path to the SoFIA driver device file.
 */
#define SP_DEVICE_NAME "sp"
#define SP_DEVICE_PATH "/dev/" SP_DEVICE_NAME

/*
 * Structure to return version information.
 */
#pragma pack(push)
#pragma pack(1)
struct sp_driver_version_info {
    pw_s32_t major;
    pw_s32_t minor;
    pw_s32_t other;
};

struct spdrv_ioctl_arg {
    pw_s32_t in_len;
    pw_s32_t out_len;
    char *in_arg;
    char *out_arg;
};
#pragma pack(pop)

/*
 * Various commands to control a collection.
 */
enum spdrv_cmd {
    SPDRV_CMD_START,
    SPDRV_CMD_STOP,
    /* others here when appropriate */
    SPDRV_CMD_MAX
};

/*
 * The SoFIA-specific IOCTL magic
 * number -- used to ensure IOCTLs
 * are delivered to the correct
 * driver.
 */
#define SP_IOC_MAGIC          99
/*
 * CONSTANTS that define the various operations.
 * TODO: convert to enum?
 */
#define SPDRV_OPERATION_CONFIGURE   1 /* configure a collection */
#define SPDRV_OPERATION_CMD         2 /* control a collection */
#define SPDRV_OPERATION_VERSION     3 /* retrieve driver version info */
#define SPDRV_OPERATION_CLOCK       4 /* retrieve STM clock */
#define SPDRV_OPERATION_TOPOLOGY    5 /* retrieve CPU topology */
/*
 * The actual IOCTL commands.
 *
 * From the kernel documentation:
 * "_IOR" ==> Read IOCTL
 * "_IOW" ==> Write IOCTL
 * "_IOWR" ==> Read/Write IOCTL
 *
 * Where "Read" and "Write" are from the user's perspective
 * (similar to the file "read" and "write" calls).
 */
#define SPDRV_IOCTL_CONFIGURE _IOW (SP_IOC_MAGIC,  SPDRV_OPERATION_CONFIGURE, struct spdrv_ioctl_arg *)
#define SPDRV_IOCTL_CMD       _IOW (SP_IOC_MAGIC, SPDRV_OPERATION_CMD, struct spdrv_ioctl_arg *)
#define SPDRV_IOCTL_VERSION   _IOR (SP_IOC_MAGIC,  SPDRV_OPERATION_VERSION, struct spdrv_ioctl_arg *)
#define SPDRV_IOCTL_CLOCK     _IOR (SP_IOC_MAGIC,  SPDRV_OPERATION_CLOCK, struct spdrv_ioctl_arg *)
#define SPDRV_IOCTL_TOPOLOGY  _IOR (SP_IOC_MAGIC,  SPDRV_OPERATION_TOPOLOGY, struct spdrv_ioctl_arg *)

#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
  #include <linux/compat.h>

  #define SPDRV_IOCTL_CONFIGURE32 _IOW (SP_IOC_MAGIC,  SPDRV_OPERATION_CONFIGURE, compat_uptr_t)
  #define SPDRV_IOCTL_CMD32       _IOW (SP_IOC_MAGIC, SPDRV_OPERATION_CMD, compat_uptr_t)
  #define SPDRV_IOCTL_VERSION32   _IOR (SP_IOC_MAGIC,  SPDRV_OPERATION_VERSION, compat_uptr_t)
  #define SPDRV_IOCTL_CLOCK32     _IOR (SP_IOC_MAGIC,  SPDRV_OPERATION_CLOCK, compat_uptr_t)
  #define SPDRV_IOCTL_TOPOLOGY32  _IOR (SP_IOC_MAGIC,  SPDRV_OPERATION_TOPOLOGY, compat_uptr_t)
#endif // COMPAT && x64

#endif // __SPIOCTL_H__
