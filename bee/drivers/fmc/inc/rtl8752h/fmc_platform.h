/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _FMC_PLATFORM_H
#define _FMC_PLATFORM_H

#include "stdint.h"

#define _PACKED_                __attribute__((packed))

/****************************************************************************************
 * Flexible Memory Controller Address Map
 ****************************************************************************************/
#define SPIC0_ADDR                                          0x00800000
#define SPIC0_SIZE                                          (16*1024*1024)
#define FLASH_OFFSET_TO_NO_CACHE                            (0x01000000)
#define FMC_MAIN0_ADDR                                      (SPIC0_ADDR)
#define FMC_MAIN0_SIZE                                      (SPIC0_SIZE)
#define FMC_MAIN0_UNCACHEABLE_ADDR                          (FLASH_OFFSET_TO_NO_CACHE)
#define FMC_MAIN0_NON_CACHE_ADDR(cache_addr)                ((cache_addr) | (FMC_MAIN0_UNCACHEABLE_ADDR))

/********************************FLASH_OFFSET_TO_NO_CACHE********************************************************
 * Flexible Memory Controller Address Map Checking
 ****************************************************************************************/
#define FMC_IS_SPIC0_CACHEABLE_ADDR(addr)                   ((addr >= FMC_MAIN0_ADDR) && (addr < FMC_MAIN0_ADDR + FMC_MAIN0_SIZE))
#define FMC_IS_SPIC0_UNCACHEABLE_ADDR(addr)                 ((addr >= FMC_MAIN0_UNCACHEABLE_ADDR) && (addr < FMC_MAIN0_UNCACHEABLE_ADDR + FMC_MAIN0_SIZE))
#define FMC_IS_SPIC0_ADDR(addr)                             (FMC_IS_SPIC0_CACHEABLE_ADDR(addr) || FMC_IS_SPIC0_UNCACHEABLE_ADDR(addr))


#endif /* _FMC_PLATFORM_H */
