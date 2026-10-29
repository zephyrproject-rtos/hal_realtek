/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*============================================================================*
 *               Define to prevent recursive inclusion
 *============================================================================*/
#ifndef RTL_EFUSE_H
#define RTL_EFUSE_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include <stdint.h>

/** @defgroup EFUSE   eFuse
  * @brief
  * @{
  */

/*============================================================================*
 *                         Types
 *============================================================================*/
/** @defgroup EFUSE_Exported_Types eFuse Exported Types
  * @brief
  * @{
  */

/** @brief eFuse operation type. */
typedef enum
{
    EFUSE_OPERATION_READ = 0,
    EFUSE_OPERATION_WRITE = 1,
} EFUSE_OPERATION_TYPE;

/** @brief Parameter structure for eFuse RAM access via NSC veneer. */
typedef struct
{
    EFUSE_OPERATION_TYPE type;
    uint16_t offset;
    uint16_t byte_size;
    uint8_t *data;
} NSC_EFUSE_RAM_SECURE_ACCESS_PARAM;

/** End of EFUSE_Exported_Types
  * @}
  */

/*============================================================================*
 *                         Functions
 *============================================================================*/
/** @defgroup EFUSE_Exported_Functions eFuse Exported Functions
  * @brief
  * @{
  */

/**
 * @brief  Get 14 bytes EUID.
 * @return A pointer to a static copy of the EUID.
 */
uint8_t *get_ic_euid(void);

/**
 * @brief  Get system type information from eFuse.
 * @param[out]  data   Pointer to buffer to store the result.
 * @param[in]   size   Number of bytes to read (capped at 3).
 */
void get_system_type(void *data, uint8_t size);

/**
 * @brief  Get USB eFuse information.
 * @param[out]  data   Pointer to buffer to store the result (8 bytes).
 */
void usb_info_get(uint8_t *data);

/**
 * @brief  Get the IC type from eFuse.
 * @return The IC type value.
 */
uint8_t get_ic_type(void);
/** End of EFUSE_Exported_Functions
  * @}
  */

/** End of EFUSE
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* RTL_EFUSE_H */
