/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define CONFIG_AMEBAD 1
#define CONFIG_IMG1_FLASH 1
#define CONFIG_IMG2_FLASH 1
#define CONFIG_SUPPORT_ATCMD 1
#define CONFIG_ATCMD_LOGUART 1
#define CONFIG_VFS_LITTLEFS_INCLUDED 1
#define CONFIG_LINK_ROM_SYMB 1
#define CONFIG_CPU_HAS_FPU 1
#define CONFIG_ARM_CORE_CM4 1
#define CONFIG_CPU_200MHZ 1
#define CONFIG_PLATFORM_CLOCK 200000000
#define CONFIG_CPU_CLOCK_SEL_VALUE 0x00
#define CONFIG_AMEBAD_B_CUT 1
#define CONFIG_SOC_PS_EN 1
#define CONFIG_SOC_PS_MODULE 1
#include <zephyr/autoconf.h>
