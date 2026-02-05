/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ameba_soc.h"

/**
  * @brief  Check whether the APB peripheral's clock has been enabled or not
  * @param  APBPeriph_Clock_in: specifies the APB peripheral to check.
  *         This parameter can be one of @ref APBPeriph_UART0_CLOCK, APBPeriph_ATIM_CLOCK and etc.
  * @retval TRUE: The APB peripheral's clock has been enabled
  * 		FALSE: The APB peripheral's clock has not been enabled
  */
u32 RCC_PeriphClockEnableChk(u32 APBPeriph_Clock_in)
{
	u32 ClkRegIndx = (APBPeriph_Clock_in >> 30) & 0x03;
	u32 APBPeriph_Clock = APBPeriph_Clock_in & (~(BIT(31) | BIT(30)));
	u32 Reg = 0;
	u32 TempVal;

	switch (ClkRegIndx) {
	case 0x0:
		Reg = REG_LSYS_CKE_GRP0;
		break;
	case 0x1:
		Reg = REG_LSYS_CKE_GRP1;
		break;
	case 0x3:
		Reg = REG_AON_CLK;
		break;
	}

	TempVal = HAL_READ32(SYSTEM_CTRL_BASE, Reg);
	if (TempVal & APBPeriph_Clock) {
		return TRUE;
	} else {
		return FALSE;
	}
}
