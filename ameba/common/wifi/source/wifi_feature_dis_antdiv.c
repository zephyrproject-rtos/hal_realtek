/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define __WIFI_FEATURE_DIS_ANTDIV_C__
#include <stdbool.h>
#include <stdint.h>

#define UNUSED(a)		(void)a

#if defined(CONFIG_SINGLE_CORE_WIFI)

void wifi_hal_antdiv_training_timer_init(void *a)
{
	UNUSED(a);
}

void wifi_hal_antdiv_init(void *a)
{
	UNUSED(a);
}

void wifi_hal_antdiv_watchdog(void *a)
{
	UNUSED(a);
}

void wifi_hal_antdiv_get_rssi_info(bool a, uint8_t b, uint8_t c)
{
	UNUSED(a);
	UNUSED(b);
	UNUSED(c);
}

void wifi_hal_antdiv_get_rate_info(bool a, uint8_t b, uint8_t c)
{
	UNUSED(a);
	UNUSED(b);
	UNUSED(c);
}

void wifi_hal_antdiv_get_evm_info(bool a, uint8_t b)
{
	UNUSED(a);
	UNUSED(b);
}

void rtw_wnm_dbg_cmd(uint8_t *buf, int32_t buf_len, int32_t flags, void *userdata)
{
	UNUSED(buf);
	UNUSED(buf_len);
	UNUSED(flags);
	UNUSED(userdata);
}

#else
void rtw_wnm_dbg_cmd(uint8_t *buf, int32_t buf_len, int32_t flags, void *userdata)
{
	UNUSED(buf);
	UNUSED(buf_len);
	UNUSED(flags);
	UNUSED(userdata);
}
#endif
void eap_disconnected_hdl(void)
{
}

int get_eap_phase(void)
{
	return 0;
}

void eap_eapol_start_hdl(uint8_t *evt_info)
{
	UNUSED(evt_info);
}

void eap_eapol_recvd_hdl(uint8_t *buf, int32_t buf_len)
{
	UNUSED(buf);
	UNUSED(buf_len);
}

int rtk_diag_event_add(uint8_t evt_level, uint16_t evt_type, const uint8_t *evt_info, uint16_t evt_len)
{
	UNUSED(evt_level);
	UNUSED(evt_type);
	UNUSED(evt_info);
	UNUSED(evt_len);
	return 0;
}

void eap_autoreconnect_hdl(uint8_t method_id)
{
	UNUSED(method_id);
}