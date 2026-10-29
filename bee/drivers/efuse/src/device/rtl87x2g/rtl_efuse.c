/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include <stdint.h>
#include "rtl_efuse.h"
#include "nsc_veneer.h"

/*============================================================================*
 *                         Macros
 *============================================================================*/
#define USB_INFO_EFUSE_OFFSET        0x28eu
#define USB_INFO_EFUSE_SIZE          8u

#define EUID_EFUSE_OFFSET            0x2c4
#define EUID_EFUSE_SIZE              14

#define SYSTEM_INFO_EFUSE_OFFSET     0x16Eu
#define SYSTEM_INFO_EFUSE_SIZE       3u

#define PACKAGE_ID_EFUSE_OFFSET       0x2c2
#define PACKAGE_ID_EFUSE_SIZE             1

/*============================================================================*
 *                           Public Functions
 *============================================================================*/
void usb_info_get(uint8_t *data)
{
    NSC_EFUSE_RAM_SECURE_ACCESS_PARAM param;
    param.type      = EFUSE_OPERATION_READ;
    param.offset    = USB_INFO_EFUSE_OFFSET;
    param.data      = data;
    param.byte_size = USB_INFO_EFUSE_SIZE;
    secure_function_call(EFUSE_RAM_ACCESS, &param);
}

void get_system_type(void *data, uint8_t size)
{
    if (size > SYSTEM_INFO_EFUSE_SIZE)
    {
        size = SYSTEM_INFO_EFUSE_SIZE;
    }

    NSC_EFUSE_RAM_SECURE_ACCESS_PARAM param;
    param.type      = EFUSE_OPERATION_READ;
    param.offset    = SYSTEM_INFO_EFUSE_OFFSET;
    param.data      = data;
    param.byte_size = size;
    secure_function_call(EFUSE_RAM_ACCESS, &param);
}

/**
 * @brief  Get 14 bytes EUID.
 * @return A pointer to a static copy of the EUID.
 */
uint8_t *get_ic_euid(void)
{
    static uint8_t euid[14] = {0};
    NSC_EFUSE_RAM_SECURE_ACCESS_PARAM param;
    param.type      = EFUSE_OPERATION_READ;
    param.offset    = EUID_EFUSE_OFFSET;
    param.data      = (uint8_t *)euid;
    param.byte_size = EUID_EFUSE_SIZE;
    secure_function_call(EFUSE_RAM_ACCESS, &param);

    return euid;
}

uint8_t get_ic_type(void)
{
    uint8_t data;
    NSC_EFUSE_RAM_SECURE_ACCESS_PARAM param;
    param.type = EFUSE_OPERATION_READ;
    param.offset = PACKAGE_ID_EFUSE_OFFSET;
    param.data = &data;
    param.byte_size = PACKAGE_ID_EFUSE_SIZE;
    secure_function_call(EFUSE_RAM_ACCESS, &param);
    return data;
}
