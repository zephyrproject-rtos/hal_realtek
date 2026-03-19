/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ameba_soc.h"

#define OTP_UUID							0x1F4

void EFUSE_GetUUID(u32 *UUID)
{
	u8 index, ID[4];

	for (index = 0; index < 4; index++) {
		OTP_Read8(OTP_UUID + index, &ID[index]);
	}

	UUID[0] = (ID[3] << 24) | (ID[2] << 16) | (ID[1] << 8) | ID[0];
	UUID[1] = 0;
}

/******************* (C) COPYRIGHT 2016 Realtek Semiconductor *****END OF FILE****/
