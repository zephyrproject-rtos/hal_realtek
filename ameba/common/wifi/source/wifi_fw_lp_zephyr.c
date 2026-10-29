/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WiFi MAC firmware bring-up (CONFIG_WIFI_FW_EN), mirroring the vendor
 * project_lp main(): create the two firmware tasks; the KM4 driver's
 * FW-enable IPC command actually starts them.
 */

#include "ameba_soc.h"
#include <zephyr/init.h>

/* lib_wifi_fw.a */
extern void wififw_task_create(void);

static int amebasmart_wifi_fw_init(void)
{
	wififw_task_create();

	return 0;
}
SYS_INIT(amebasmart_wifi_fw_init, APPLICATION, 90);
