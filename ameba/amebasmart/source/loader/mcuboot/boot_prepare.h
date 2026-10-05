/*
 * Copyright (c) 2026, Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _AMEBASMART_LOADER_MCUBOOT_
#define _AMEBASMART_LOADER_MCUBOOT_

#include <stdint.h>

void boot_prepare(uint32_t flash_phy_addr,
				  uint8_t  app_flash_id,
				  uint32_t app_flash_offset,
				  uint32_t app_flash_size,
				  uint32_t ap_logic_addr,
				  uint32_t np_logic_addr,
				  uint32_t ca32_logic_addr);

#endif /* _AMEBASMART_LOADER_MCUBOOT_ */
