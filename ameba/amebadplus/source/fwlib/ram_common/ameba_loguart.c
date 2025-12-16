/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ameba_soc.h"

/**
  * @brief Switch LOGUART interrupt from NP to AP.
  * @param None
  * @retval None
  * @note KM0 is NP and KM4 is AP.
  */
void LOGUART_INT_NP2AP(void)
{
	LOGUART_INTCoreConfig(LOGUART_DEV, LOGUART_BIT_INTR_MASK_KM0, DISABLE);
	LOGUART_INTCoreConfig(LOGUART_DEV, LOGUART_BIT_INTR_MASK_KM4, ENABLE);
}
