/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ameba_soc.h"

void PAD_SlewRateCtrl(u8 PinName, u32 NewState)
{
	if (PinName > _PC_7) {
		return;
	}

	u32 Temp = PINMUX->PADCTR[PinName];

	if (NewState) {
		Temp |= PAD_BIT_GPIOx_SR;
	} else {
		Temp &= ~PAD_BIT_GPIOx_SR;
	}

	PINMUX->PADCTR[PinName] = Temp;
}

void PAD_SchmitCtrl(u8 PinName, u32 NewState)
{
	if (PinName > _PC_7) {
		return;
	}

	u32 Temp = PINMUX->PADCTR[PinName];

	if (NewState) {
		Temp |= PAD_BIT_GPIOx_SMT;
	} else {
		Temp &= ~PAD_BIT_GPIOx_SMT;
	}

	PINMUX->PADCTR[PinName] = Temp;
}
