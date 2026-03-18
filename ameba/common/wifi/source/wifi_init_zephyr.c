/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ameba_soc.h"
#include <zephyr/kernel.h>

#ifdef CONFIG_SOC_SERIES_AMEBAD
extern u32 wifi_hal_dma_interrupt(void *data);

void wlan_int_enable(void)
{
	IRQ_CONNECT(WL_DMA_IRQ, 0, wifi_hal_dma_interrupt, NULL, 0);
	irq_enable(WL_DMA_IRQ);

	IRQ_CONNECT(WL_PROTOCOL_IRQ, 0, wifi_hal_dma_interrupt, NULL, 0);
	irq_enable(WL_PROTOCOL_IRQ);

}

#else
void wlan_int_enable(void)
{
	return;
}
#endif
