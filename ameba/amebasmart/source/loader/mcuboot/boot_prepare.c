/*
 * Copyright (c) 2026, Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * App content layout (after the MCUboot 0x200-byte header):
 *   [+0x000] word[0]  initial SP for MCUboot jump (e.g. MSP_RAM_HP)
 *   [+0x004] word[1]  KM4 IMG2 entry (app_start; DRAM in PSRAM mode)
 *   [+0x008..0x020]   zero padding (VT padded to IMAGE_HEADER_LEN=0x20 so the
 *                     km0 chain below starts 32-byte aligned for RSIP_MMU_Config)
 *   [+0x020] km0_image2_all.bin  — 3 sub-images {XIP, SRAM, DRAM} for KM0
 *   [+...]   km4_image2_all.bin  — 3 sub-images {XIP, SRAM, DRAM} for KM4
 *   [+...]   ap_image            — 4 sub-images {XIP, BL1 SRAM, BL1 DRAM, FIP}
 */

#include "ameba_soc.h"
#include "bootloader_hp.h"
#include "boot_prepare.h"

#include <sysflash/sysflash.h>

#include "bootutil/bootutil.h"

static const char *const TAG = "BOOT";

/* From usrcfg/amebasmart/ameba_bootcfg.c: MEM-SWR-only power mode select. */
extern u8 Boot_MemSwr_Only;

#define RSIP_WIN_SZ         0x100000UL     /* 1 MB per XIP window */

/* BOOT_RSIP_Load_Image - read the RSIP IV from slot0's custom TLV
 * (CONFIG_AMEBA_RSIP_IV_TYPE_IN_TLV). */
static int BOOT_RSIP_Load_Image(uint8_t id, uint8_t *iv, struct image_header *hdr)
{
	uint32_t off = 0;
	const struct flash_area *fap = NULL;
	int rc;

	RTK_LOGI(TAG, "Attempting to parse IV from TLV...\r\n");

	rc = flash_area_open(id, &fap);
	if (rc != 0) {
		return rc;
	}

	rc = flash_area_read(fap, off, hdr, sizeof(*hdr));
	if (rc != 0) {
		goto end;
	}

#ifdef CONFIG_MCUBOOT
	/* Traverse the TLV area to find the RSIP IV TLV. */
	struct image_tlv_iter it = {0};
	uint16_t type;
	uint16_t len;
#if defined(MCUBOOT_SWAP_USING_OFFSET)
	it.start_off = 0;
#endif
	rc = bootutil_tlv_iter_begin(&it, hdr, fap, IMAGE_TLV_ANY, false);
	if (rc != 0) {
		goto end;
	}

	while (true) {
		rc = bootutil_tlv_iter_next(&it, &off, &len, &type);
		if (rc != 0) {
			goto end;
		}

		if (type == CONFIG_AMEBA_RSIP_IV_TYPE_IN_TLV) {
			if (len != 8) {
				rc = -1;
				goto end;
			}
			rc = flash_area_read(fap, off, iv, len);
			break;
		}
	}
#endif
end:
	flash_area_close(fap);
	return rc;
}

static void BOOT_OTFCheck(uint32_t start_addr, uint32_t end_addr, uint32_t IV_index,
						  uint32_t OTF_index)
{
	uint32_t mode;

	if (SYSCFG_OTP_RSIPEn() == FALSE) {
		return;
	}

	if (!IS_FLASH_ADDR(start_addr) || !IS_FLASH_ADDR(end_addr)) {
		return;
	}

	RTK_LOGI(TAG, "IMG2 OTF EN[%d]\r\n", OTF_index);

	switch (SYSCFG_OTP_RSIPMode()) {
	case RSIP_XTS_MODE:
		mode = OTF_XTS_MODE;
		break;
	case RSIP_CTR_MODE:
		mode = OTF_CTR_MODE;
		break;
	default:
		RTK_LOGE(TAG, "OTF Mode error\r\n");
		return;
	}

	RSIP_OTF_Enable(OTF_index, start_addr, end_addr, ENABLE, IV_index, mode);
	RSIP_OTF_Cmd(ENABLE);
}

/* ---- Sub-image loading (port of SDK boot_ota_hp.c) ---- */
static bool boot_load_sub_image(SubImgInfo_TypeDef *sub_info, uint32_t start_addr,
								uint32_t num, const char *const *img_name)
{
	IMAGE_HEADER hdr;
	uint32_t dst, len, i;

	for (i = 0; i < num; i++) {
		_memcpy(&hdr, (const void *)start_addr, IMAGE_HEADER_LEN);

		if ((hdr.signature[0] != APP_IMAGE_PATTERN_1) ||
			(hdr.signature[1] != APP_IMAGE_PATTERN_2)) {
			RTK_LOGE(TAG, "%s Invalid\r\n", img_name[i]);
			return false;
		}

		dst = hdr.image_addr - IMAGE_HEADER_LEN;
		len = hdr.image_size + IMAGE_HEADER_LEN;

		/* Non-XIP sub-image: copy to link address, flush KM4 D-cache so
		 * KM0/CA32 see it in memory. */
		if ((!IS_FLASH_ADDR(dst)) && (len > IMAGE_HEADER_LEN)) {
			_memcpy((void *)dst, (const void *)start_addr, len);
			DCache_CleanInvalidate(dst, len);
			__DSB();
		}

		/* Empty image: leave in flash, record its flash address. */
		if (len == IMAGE_HEADER_LEN) {
			dst = start_addr;
		}

		if (sub_info != NULL) {
			sub_info[i].Addr = dst;
			sub_info[i].Len  = len;
		}

		RTK_LOGI(TAG, "%s[%08x:%x]\r\n", img_name[i], dst, len);

		start_addr += len;
	}

	return true;
}

static uint32_t boot_copy_core_image(uint32_t mmu_idx, uint32_t otf_idx,
									 uint32_t LogAddr, uint32_t PhyAddr,
									 SubImgInfo_TypeDef *sub_info, uint32_t num,
									 const char *const *img_name)
{
	uint32_t ImgAddr, TotalLen = 0, i;

	RSIP_MMU_Config(mmu_idx, LogAddr, LogAddr + RSIP_WIN_SZ, PhyAddr);
	RSIP_MMU_Cmd(mmu_idx, ENABLE);
	RSIP_MMU_Cache_Clean();
	BOOT_OTFCheck(LogAddr, LogAddr + RSIP_WIN_SZ, OTF_IV_IMG2_IDX, otf_idx);

	ImgAddr = SYSCFG_OTP_BootFromNor() ? LogAddr : PhyAddr;
	if (!boot_load_sub_image(sub_info, ImgAddr, num, img_name)) {
		return 0;
	}

	for (i = 0; i < num; i++) {
		TotalLen += sub_info[i].Len;
	}
	return TotalLen;
}

static uint32_t Boot_Copy_KM0_Image(uint32_t LogAddr, uint32_t PhyAddr,
									SubImgInfo_TypeDef *sub_info)
{
	static const char *const Km0Label[] = {"KM0 XIP IMG", "KM0 SRAM", "KM0 DRAM"};

	return boot_copy_core_image(MMU_LP_IDX, OTF_LP_IDX, LogAddr, PhyAddr,
								sub_info, sizeof(Km0Label) / sizeof(char *), Km0Label);
}

static uint32_t Boot_Copy_KM4_Image(uint32_t LogAddr, uint32_t PhyAddr,
									SubImgInfo_TypeDef *sub_info)
{
	static const char *const Km4Label[] = {"KM4 XIP IMG", "KM4 SRAM", "KM4 DRAM"};

	return boot_copy_core_image(MMU_HP_IDX, OTF_HP_IDX, LogAddr, PhyAddr,
								sub_info, sizeof(Km4Label) / sizeof(char *), Km4Label);
}

static uint32_t Boot_Copy_AP_Image(uint32_t LogAddr, uint32_t PhyAddr,
								   SubImgInfo_TypeDef *sub_info)
{
	static const char *const APLabel[] = {"AP XIP IMG", "AP BL1 SRAM",
										  "AP BL1 DRAM", "AP FIP"
										 };

	return boot_copy_core_image(MMU_AP_IDX, OTF_AP_IDX, LogAddr, PhyAddr,
								sub_info, sizeof(APLabel) / sizeof(char *), APLabel);
}

static void BOOT_RSIP_MMU_Config(uint32_t flash_phy_addr, uint8_t app_flash_id,
								 uint32_t slot_off, uint32_t slot_size,
								 uint32_t ap_logic, uint32_t np_logic,
								 uint32_t ca32_logic)
{
	SubImgInfo_TypeDef SubImgInfo[16];
	struct image_header hdr_img;
	uint32_t LogAddr, PhyAddr, TotalLen;
	uint32_t Index = 0;
	uint8_t iv[8];

	(void)slot_size;

	BOOT_RSIP_Load_Image(app_flash_id, iv, &hdr_img);

	if (SYSCFG_OTP_RSIPEn() == TRUE) {
		RSIP_IV_Set(OTF_IV_IMG2_IDX, iv);
	}

	PhyAddr = flash_phy_addr + slot_off + 0x200 + IMAGE_HEADER_LEN;

	/* ---- KM0 IMG2 ---- */
	LogAddr = np_logic;
	TotalLen = Boot_Copy_KM0_Image(LogAddr, PhyAddr, &SubImgInfo[Index]);
	if (TotalLen == 0) {
		goto fail;
	}
	Index += 3;
	PhyAddr += TotalLen;

	/* ---- KM4 IMG2 ---- */
	LogAddr = ap_logic;
	TotalLen = Boot_Copy_KM4_Image(LogAddr, PhyAddr, &SubImgInfo[Index]);
	if (TotalLen == 0) {
		goto fail;
	}
	Index += 3;
	PhyAddr += TotalLen;

	/* ---- AP (CA32) IMG ---- */
	LogAddr = ca32_logic;
	TotalLen = Boot_Copy_AP_Image(LogAddr, PhyAddr, &SubImgInfo[Index]);
	if (TotalLen == 0) {
		goto fail;
	}
	Index += 4;

	RTK_LOGI(TAG, "sub-image load done (%d imgs)\r\n", Index);
	return;

fail:
	RTK_LOGE(TAG, "Fail to load sub-image!\r\n");
	while (1) {
		DelayMs(1000);
	}
}

/* ---- Public entry ---- */

void boot_prepare(uint32_t flash_phy_addr, uint8_t app_flash_id,
				  uint32_t app_flash_offset, uint32_t app_flash_size,
				  uint32_t ap_logic_addr, uint32_t np_logic_addr,
				  uint32_t ca32_logic_addr)
{
	MCM_MemTypeDef meminfo;

	/* Copy boot reason, print KM4 BOOT REASON banner */
	BOOT_ReasonSet();

	Peripheral_Reset();

	if ((BOOT_Reason() == 0) || (!BOOT_RRAM_InfoValid())) {
		_memset(RRAM, 0, sizeof(RRAM_TypeDef));
		RRAM->MAGIC_NUMBER = 0x6969A5A5;
	}

	/*open swr power*/
	SWR_Calib_DCore();

	if (Boot_MemSwr_Only) {
		/* mem swr mode with core */
		SWR_MEM_Manual(DISABLE);
		SWR_MEM(ENABLE);
	} else {
		SWR_MEM(ENABLE);

		/* audio swr mode with core */
		SWR_AUDIO_Manual(DISABLE);
		SWR_AUDIO(ENABLE);
	}

	/* Enable divide-by-zero fault.  BOOT_Image1 also sets SCB_NS->CCR, but
	 * MCUboot runs with ARM_TRUSTZONE_M off, so only the secure SCB is valid. */
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;

	BOOT_SOC_ClkSet();

	BOOT_GRstConfig();

	meminfo = ChipInfo_MCMInfo();
	if ((meminfo.mem_type & MCM_TYPE_PSRAM) == MCM_TYPE_PSRAM) {
		/* off ddrphy BG for psram chip, open by USB/MIPI when needed */
		HAL_WRITE32(SYSTEM_CTRL_BASE_LP, REG_LSYS_AIP_CTRL1,
					HAL_READ32(SYSTEM_CTRL_BASE_LP, REG_LSYS_AIP_CTRL1) &
					(~(LSYS_BIT_BG_PWR | LSYS_BIT_BG_ON_MIPI | LSYS_BIT_BG_ON_USB2)));
		RRAM->MEM_TYPE = MCM_TYPE_PSRAM;
		RCC_PeriphClockCmd(APBPeriph_PSRAM, APBPeriph_PSRAM_CLOCK, ENABLE);
		HAL_WRITE32(SYSTEM_CTRL_BASE_LP, REG_LSYS_DUMMY_098,
					HAL_READ32(SYSTEM_CTRL_BASE_LP, REG_LSYS_DUMMY_098) | LSYS_BIT_PWDPAD15N_DQ);
	} else {
		/* off USB and MIPI by default, open in IP */
		HAL_WRITE32(SYSTEM_CTRL_BASE_LP, REG_LSYS_AIP_CTRL1,
					HAL_READ32(SYSTEM_CTRL_BASE_LP, REG_LSYS_AIP_CTRL1) &
					(~(LSYS_BIT_BG_ON_MIPI | LSYS_BIT_BG_ON_USB2)));
		RRAM->MEM_TYPE = MCM_TYPE_DDR;
		RCC_PeriphClockCmd(APBPeriph_DDRP, APBPeriph_DDRP_CLOCK, ENABLE);
		RCC_PeriphClockCmd(APBPeriph_DDRC, APBPeriph_DDRC_CLOCK, ENABLE);
		HAL_WRITE32(SYSTEM_CTRL_BASE_LP, REG_LSYS_DUMMY_098,
					HAL_READ32(SYSTEM_CTRL_BASE_LP, REG_LSYS_DUMMY_098) |
					LSYS_BIT_PWDPAD15N_DQ | LSYS_BIT_PWDPAD15N_CA);
	}

	/* 4200825c[1:0]=2'b10, change BandGap from 0x3 to 0x2 */
	HAL_WRITE32(SYSTEM_CTRL_BASE_LP, REG_LSYS_AIP_CTRL1,
				(HAL_READ32(SYSTEM_CTRL_BASE_LP, REG_LSYS_AIP_CTRL1) & ~LSYS_MASK_BG_ALL) |
				LSYS_BG_ALL(0x2));

	BOOT_Log_Init();

	RTK_LOGI(TAG, "MCUboot boot_prepare MSP:[%08x]\n", __get_MSP());
	RTK_LOGI(TAG, "Build Time: %s %s\n", __DATE__, __TIME__);

	BOOT_RccConfig();

	RRAM->IMQ_INIT_DONE = 0;

	flash_highspeed_setup();

	if ((meminfo.mem_type & MCM_TYPE_PSRAM) == MCM_TYPE_PSRAM) {
		RTK_LOGI(TAG, "Init PSRAM\r\n");
		PSRAM_INFO_Update();
		BOOT_PSRAM_Init();
		if (!Boot_MemSwr_Only) {
			/* if Boot_MemSwr_Only, accompany core, or keep in PFM */
			if (SWR_MEM_Mode_Set(SWR_PFM)) {
				RTK_LOGW(TAG, "set pfm fail\r\n");
			}
		}
	} else {
		if (Boot_MemSwr_Only) {
			while (1) {
				RTK_LOGE(TAG, "ERROR!! Should Not enable MemSwr_Only in DDR Chip!!!\r\n");
				DelayMs(5000);
			}
		}
		if (ChipInfo_DDRType() == MCM_DDR2) {
			RTK_LOGI(TAG, "Init DDR2\r\n");
		} else {
			RTK_LOGI(TAG, "Init DDR3\r\n");
		}
		BOOT_DDR_Init();
		BOOT_DDR_LCDC_HPR();
	}

	/* Reset SDM32K/OTP/RTC/PINMUX-PAD/SYSON/ATIM masks (BOOT_Image1). */
	HAL_WRITE32(SYSTEM_CTRL_BASE_LP, REG_LSYS_SYSRST_MSK0, 0x0);
	HAL_WRITE32(SYSTEM_CTRL_BASE_LP, REG_LSYS_SYSRST_MSK1, 0x0);
	HAL_WRITE32(SYSTEM_CTRL_BASE_LP, REG_LSYS_SYSRST_MSK2, 0x0);
	HAL_WRITE32(SYSTEM_CTRL_BASE_LP, REG_AON_SYSRST_MSK, 0x0);

	BOOT_Share_Memory_Patch();

	RTK_LOGI(TAG, "memory ready\r\n");

	/* Must run before BOOT_RAM_TZCfg so copies land in physical DDR. */
	BOOT_RSIP_MMU_Config(flash_phy_addr, app_flash_id, app_flash_offset,
						 app_flash_size, ap_logic_addr, np_logic_addr,
						 ca32_logic_addr);

	/* backup flash_init_para address for KM0 */
	DCache_Clean((u32)&flash_init_para, sizeof(FLASH_InitTypeDef));
	HAL_WRITE32(SYSTEM_CTRL_BASE_LP, REG_LSYS_FLASH_PARA_ADDR, (u32)&flash_init_para);

	/* it will switch shell control to KM0, disable loguart interrupt to avoid loguart irq not assigned in non-secure world.
	 it should switch before BOOT_RAM_TZCfg to avoid crash when loguart intr occur but it has been set to ns intr. */
	LOGUART_INTConfig(LOGUART_DEV, LOGUART_BIT_ERBI, DISABLE);
	InterruptDis(UART_LOG_IRQ);

	/* Config Non-Security World Registers and clean Dcache */
	BOOT_RAM_TZCfg();

	RTK_LOGI(TAG, "enabling KM0\r\n");
	BOOT_Enable_KM0();

	/* AP Power-on, AP start run */
	if (Boot_AP_Enbale) {
		u8 ap_status = HAL_READ8(SYSTEM_CTRL_BASE_LP, REG_LSYS_AP_STATUS_SW);

		if (0 == (ap_status & LSYS_BIT_AP_ENABLE)) {
			BOOT_Enable_AP();
		}
		ap_status &= ~LSYS_BIT_AP_RST_WAIT_DRAM;
		ap_status |= LSYS_BIT_AP_ENABLE | LSYS_BIT_AP_RUNNING;
		HAL_WRITE8(SYSTEM_CTRL_BASE_LP, REG_LSYS_AP_STATUS_SW, ap_status);
	} else {
		BOOT_Disable_AP();
	}
	/* indicate KM4 is running */
	HAL_WRITE8(SYSTEM_CTRL_BASE_LP, REG_LSYS_NP_STATUS_SW,
			   HAL_READ8(SYSTEM_CTRL_BASE_LP, REG_LSYS_NP_STATUS_SW) | LSYS_BIT_NP_RUNNING);

	RTK_LOGI(TAG, "boot_prepare done\r\n");
}

void BOOT_Image1(void)
{
	extern void z_arm_reset(void);
	z_arm_reset();
}

/* BOOT_WakeFromPG (PG warm-boot handler) calls Fault_Hanlder_Redirect(NULL) to
 * re-point the fault vector.  That machinery isn't part of the MCUboot image1;
 * provide a no-op so BOOT_WakeFromPG links — the NP image2's own handlers take
 * over after resume. */
void Fault_Hanlder_Redirect(void *crash_on_task_func)
{
	ARG_UNUSED(crash_on_task_func);
}
