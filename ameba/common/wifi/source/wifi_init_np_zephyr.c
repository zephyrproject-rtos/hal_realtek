/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WHC network-processor bring-up (CONFIG_AS_INIC_NP), mirroring the vendor
 * project_hp main(): coex device IPC, then whc_dev_init() (registers the WHC
 * device API task and raises AON_BIT_WIFI_INIC_NP_READY).  The 802.11 driver
 * itself starts when the host sends the WIFI_ON API message.
 */

#include "ameba_soc.h"
#include <zephyr/init.h>

#if defined(CONFIG_BT_COEXIST)
#include "rtw_coex_ipc.h"
#endif

/* lib_wifi_whc_np.a (the vendor's whc_dev_init alias for the IPC transport). */
extern void whc_ipc_dev_init(void);
/* rtw_task_size.c; fills g_rtw_task_size, which whc_ipc_dev_api_init sizes
 * its task stack from -- must run first or the task gets a zero-byte stack.
 */
extern void wifi_set_task_size(void);
/* lib_wifi_whc_np.a; wires the driver's ROM function map (alloc hooks etc.). */
extern void wifi_set_rom2flash(void);

static int amebasmart_wifi_np_init(void)
{
#if defined(CONFIG_BT_COEXIST)
	coex_ipc_entry();
#endif
	wifi_set_rom2flash();
	wifi_set_task_size();
	whc_ipc_dev_init();

	return 0;
}
SYS_INIT(amebasmart_wifi_np_init, APPLICATION, 90);
