/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file usb_stubs.c
 * @brief Stub implementations for USB HAL library functions referenced by the
 *        Realtek Bee udc_dwc2 driver quirk. Used when building without binary
 *        blobs (CONFIG_BUILD_ONLY_NO_BLOBS).
 */

/* Stub implementations for USB HAL functions */
void hal_usb_phy_power_on(void)
{
}

void hal_rtk_usb_init(void)
{
}

void usb_rtk_disable_power_seq(void)
{
}
