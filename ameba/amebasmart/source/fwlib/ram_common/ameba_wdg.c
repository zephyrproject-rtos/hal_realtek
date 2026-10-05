/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ameba_soc.h"

void WDG_StructMemValueSet(WDG_InitTypeDef *WDG_InitStruct, u32 window, u32 timeout, u32 eicnt)
{
	WDG_StructInit(WDG_InitStruct);
	WDG_InitStruct->Window = window;
	WDG_InitStruct->Timeout = timeout;
	WDG_InitStruct->EIMOD = ENABLE;
	WDG_InitStruct->EICNT = eicnt;
}

void WDG_Cmd(WDG_TypeDef *WDG, u32 NewState)
{
	assert_param(IS_WDG_ALL_PERIPH(WDG));
	if (NewState == ENABLE) {
		WDG_Wait_Busy(WDG);
		WDG->WDG_MKEYR = WDG_FUNC_EN;
	}
	/* DISABLE is a hardware no-op: WDG cannot be stopped once started */
}
