/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __WIFI_CONF_BASIC_H
#define __WIFI_CONF_BASIC_H

/** @defgroup WIFI_API
 *  @brief      WIFI_API module
 *  @{
 */
#include "basic_types.h"
#ifdef __cplusplus
extern "C" {
#endif

#define INIC_MAX_SSID_LENGTH (33)

#pragma pack(1)
/**
  * @brief  The structure is used to describe the SSID.
  */
struct _rtw_ssid_t {
	unsigned char		len;     /**< SSID length */
	unsigned char		val[INIC_MAX_SSID_LENGTH]; /**< SSID name (AP name)  */
};

/**
  * @brief  The structure is used to describe the unique 6-byte MAC address.
  */
struct _rtw_mac_t {
	unsigned char		octet[6]; /**< Unique 6-byte MAC address */
};


/**
  * @brief  The structure is used to describe the scan result of the AP.
  */
struct rtw_scan_result {
	struct _rtw_ssid_t              SSID;             /**< Service Set Identification (i.e. Name of Access Point)                    */
	struct _rtw_mac_t               BSSID;            /**< Basic Service Set Identification (i.e. MAC address of Access Point)       */
	signed short		                  signal_strength;  /**< Receive Signal Strength Indication in dBm. <-90=Very poor, >-30=Excellent */
	u8          			   bss_type;         /**< val: RTW_BSS_TYPE_INFRASTRUCTURE, RTW_BSS_TYPE_WTN_HELPER*/
	u32					       security;         /**< val: RTW_SECURITY_OPEN, RTW_SECURITY_WEP_PSK...*/
	u8        				   wps_type;         /**< val: RTW_WPS_TYPE_DEFAULT, RTW_WPS_TYPE_USER_SPECIFIED...*/
	unsigned int               channel;          /**< Radio channel that the AP beacon was received on                          */
	u8					       band;             /**< val: RTW_802_11_BAND_5GHZ, RTW_802_11_BAND_2_4GHZ*/
	char	country_code[2];
	u8		rom_rsvd[4];
};

#pragma pack()

int wifi_on(u8 mode);
int wifi_is_running(unsigned char wlan_idx);
int wifi_disconnect(void);

int wifi_stop_ap(void);
int _wifi_on_ap(void);
int _wifi_off_ap(void);

int wifi_scan_networks_zephyr(u32 call_back);
s32 wifi_get_join_status(u8 *join_status);
int wifi_connect_zephyr(u8 *ssid, u8 ssid_len, u8 *psk, u8 psk_len, u8 channel);

int wifi_is_connected_to_ap(void);
void rtw_psk_wpa_deinit(u8 port);
void rtw_wakelock_timeout(uint32_t timeoutms);
void wifi_event_init(void);
void wifi_indication(unsigned int event, char *buf, int buf_len, int flags);
void rtw_psk_disconnect_hdl(char *buf, int buf_len, int flags, void *userdata);

#ifdef __cplusplus
}
#endif

#endif // __WIFI_API_H