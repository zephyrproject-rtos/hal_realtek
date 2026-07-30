/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SYS_RESET_H__
#define __SYS_RESET_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#include "wdt/inc/rtl_wdt.h"
#include "aon_reg.h"
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#include "wdt/src/rtl8752h/rtl876x_wdg.h"
#endif

/* Definitions of system reset reasons */
#define    RESET_REASON_HW                          0x0  /* HW reset */
#define    RESET_REASON_WDG_TIMEOUT                 0x1  /* WDT reset */
#define    RESET_REASON_BOOT_EFUSE_INVALID          0x2
#define    RESET_REASON_BOOT_FLASH_INVALID          0x3
#define    RESET_REASON_BOOT_RETRY_COUNT_LIMIT      0x4
#define    RESET_REASON_HARD_FAULT                  0x5
#define    RESET_REASON_PASSWORD_DEBUG              0x6
#define    RESET_REASON_CHIP_RESET                  0x7
#define    RESET_REASON_ENTER_FT_MODE               0x8
#define    RESET_REASON_SWITCH_TO_HCI_MODE          0x9
#define    RESET_REASON_SWITCH_TO_OTA_MODE          0xA
#define    RESET_REASON_DFU_TIMEOUT                 0xB
#define    RESET_REASON_DFU_LINK_LOST               0xC
#define    RESET_REASON_DFU_UPDATE_IMG              0xD
#define    RESET_REASON_DFU_UPDATE_COMP_IMG         0xE
#define    RESET_REASON_DFU_UPDATE_IMG_FAIL         0xF
#define    RESET_REASON_DFU_ACTIVE_RESET            0x10
#define    RESET_REASON_DFU_FAIL_RESET              0x11
#define    RESET_REASON_FEATURE_CHECK_FAIL          0x12
#define    RESET_REASON_FLASH_LAYOUT_OVERFLOW       0x13
#define    RESET_REASON_MP_RESET                    0x14
#define    RESET_REASON_POWER_DOWN_RESET            0x15
#define    RESET_REASON_FLASH_IOCTL                 0x16
#define    RESET_REASON_BT_CONTROLLER               0x17
#define    RESET_REASON_RESET_WRAPPER               0x18
#define    RESET_REASON_BT_IMG_MISMATCH             0x19
#define    RESET_REASON_ZEPHYR                      0x1A

/* Customized reset reasons start from 0x80 */
#define    RESET_REASON_APP_START                   0x80
#define    SW_RESET_APP_END                         0xFF

typedef uint32_t T_SW_RESET_REASON;

/**
 * @brief Execute a system reset via the watchdog.
 *
 * This function performs a system reboot using the specified watchdog mode
 * `wdt_mode`, and optionally logs the reason for the reset as `reset_reason`.
 *
 * @param wdt_mode Specifies the watchdog mode for the reset.
 * @param reset_reason Provides the reason for the system reset, useful for further diagnostics and tracking.
 */
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
extern void WDG_SystemReset(WDTMode_TypeDef wdt_mode, int reset_reason);
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
extern void WDG_SystemReset(T_WDG_MODE wdt_mode, int reset_reason);
#endif

/**
 * @brief Retrieve the reason for the system reset.
 *
 * This function returns the reason for the most recent system reset, useful for diagnosing reboot events.
 *
 * @param none
 * @return Returns the reason for the system reset, type T_SW_RESET_REASON.
 */
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
static inline T_SW_RESET_REASON reset_reason_get(void)
{
    AON_NS_REG0X_APP_TYPE aon_0x1ae0 = {.d32 = AON_REG_READ(AON_NS_REG0X_APP)};
    return aon_0x1ae0.reset_reason;
}
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
extern T_SW_RESET_REASON reset_reason_get(void);
#endif

#ifdef __cplusplus
}
#endif

#endif
