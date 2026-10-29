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

#elif defined(CONFIG_SOC_SERIES_AMEBASMART)
void wlan_int_enable(void)
{
	/*
	 * whc_ipc_host_init() spins on AON_BIT_WIFI_INIC_NP_READY before
	 * wifi_on(), but the bit never arrives from the vendor KM4 image.
	 * The NP is long up by the time the CA32 runs (IMG1 boots it first),
	 * so satisfy the handshake here.
	 */
	u32 reg = HAL_READ32(REG_AON_WIFI_IPC, 0);

	HAL_WRITE32(REG_AON_WIFI_IPC, 0, reg | AON_BIT_WIFI_INIC_NP_READY);
}
#else
void wlan_int_enable(void)
{
	return;
}
#endif
