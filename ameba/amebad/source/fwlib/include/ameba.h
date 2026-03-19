/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _HAL_8721D_H_
#define _HAL_8721D_H_

#include "basic_types.h"
#include "section_config.h"
#include "memproc.h"
#include "strproc.h"
#include "ameba_sys_on.h"
#include "ameba_peri_on.h"
#include "hal_platform.h"
#include "cmsis_cpu.h"
#include "mpu_config.h"
#include "ameba_vector.h"
#include "ameba_loguart.h"
#include "ameba_soc_ps.h"
#include "diag.h"
#include "log.h"
#include "shell.h"
#include "rand.h"
#include "monitor_rom.h"
#include "ameba_syscfg.h"
#include "ameba_pmc.h"
#include "ameba_clk.h"
#include "ameba_bor.h"
#include "ameba_crypto.h"
#include "ameba_crypto_api.h"
#include "ameba_boot.h"
#include "ameba_otf.h"
#include "ameba_flash.h"
#include "ameba_backup_reg.h"
#include "ameba_pinmap.h"
#include "ameba_ipc.h"
#include "ameba_sysreg.h"
#include "ameba_sysreg.h"
#include "ameba_pinmux.h"
#include "ameba_usb.h"
#include "ameba_reset.h"
#ifndef CONFIG_BUILD_ROM
#include "ameba_ipc_api.h"
#endif


#if defined (CONFIG_ARM_CORE_CM4)
#include "ameba_rcc_hp.h"
#else
#include "ameba_rcc_lp.h"
#endif
#include "sscanf_minimal.h"

#include "ameba_trustzone.h"
#include "ameba_arch.h"

#include "ameba_gdma.h"
#include "ameba_tim.h"
#include "ameba_gpio.h"
#include "ameba_ssi.h"
#include "ameba_uart.h"
#include "ameba_i2c.h"
#include "ameba_i2s.h"
#include "ameba_adc.h"
#include "ameba_comparator.h"
#include "ameba_sdio.h"
#include "ameba_wdg.h"
#include "ameba_rtc.h"
#include "ameba_delay.h"
#include "ameba_ir.h"
#include "ameba_keyscan.h"
#include "ameba_sgpio.h"
#include "ameba_qdec.h"
#include "ameba_usi.h"
#include "ameba_usi_uart.h"
#include "ameba_usi_ssi.h"
#include "ameba_usi_i2c.h"
#include "ameba_crc.h"
#include "ameba_lcdc.h"
#include "ameba_audio.h"
#include "ameba_efuse.h"
#include "ameba_cache.h"
#include "ameba_psram.h"
#include "ameba_sdioh.h"
#include "ameba_sd.h"
#include "ameba_usrcfg.h"
#include "ameba_km4.h"
#include "ameba_chipinfo.h"
#include "ameba_captouch.h"

#define FLASH_HS_BOOT_ADDR				(SPI_FLASH_BASE + 0x4000)
// #define FLASH_RESERVED_DATA_BASE		0x2000
// #define FLASH_SYSTEM_DATA_ADDR			0x3000  // reserve 8K+4K for Image1 + Reserved data
#define FLASH_SECTOR_SIZE				0x1000

/* Spic_Mode */
#define ReadQuadIOMode			0
#define ReadQuadOMode			1
#define ReadDualIOMode			2
#define ReadDualOMode 			3
#define ReadOneMode				4

#define IMAGE_HEADER_LEN		0x20
typedef struct {
	u32 signature[2];
	u32 image_size;
	u32 image_addr;

	/* reserved for extention */
	u32 sb_header;
	u32 reserved[3];
} IMAGE_HEADER;

typedef struct {
	u32 reserved[12];

	unsigned char sb_sig[64];
} SB_HEADER;


/* for code compitiable */
#define MANIFEST_SIZE_4K_ALIGN		0x1000
#define IWDG_DEV		0

typedef enum  _HAL_Status {
	HAL_OK            = 0x00,
	HAL_BUSY          = 0x01,
	HAL_TIMEOUT       = 0x02,
	HAL_ERR_PARA      = 0x03,     // error with invaild parameters
	HAL_ERR_MEM       = 0x04,     // error with memory allocation failed
	HAL_ERR_HW        = 0x05,     // error with hardware error

	HAL_ERR_UNKNOWN   = 0xee      // unknown error

} HAL_Status;


#define USE_FULL_ASSERT

#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports
  *         the name of the source file and the source line number of the call
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
#define assert_param(expr) ((expr) ? (void)0 : io_assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void io_assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif //_HAL_8721D_H_
