# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

# The headers in source/ carry the CA32 host flavor of the WiFi API; the NP
# build must resolve them from the ameba-rtos SDK instead (added below).
if(NOT CONFIG_AS_INIC_NP)
  zephyr_include_directories(source)
  zephyr_include_directories(../../usrcfg/common)
endif()

zephyr_library_sources_ifdef(CONFIG_WIFI source/wifi_feature_dis_antdiv.c)
zephyr_library_sources_ifdef(CONFIG_WIFI source/wifi_init_zephyr.c)

# Tri-core WiFi split: KM4 WHC device / KM0 MAC firmware bring-up glue.
zephyr_library_sources_ifdef(CONFIG_AS_INIC_NP source/wifi_init_np_zephyr.c)
zephyr_library_sources_ifdef(CONFIG_WIFI_FW_EN source/wifi_fw_lp_zephyr.c)

# NP-side sources the prebuilt driver library expects, from this module's
# usrcfg and the ameba-rtos SDK west project.
if(CONFIG_AS_INIC_NP)
  set(AMEBA_RTOS_WIFI ${WEST_TOPDIR}/modules/hal/ameba-rtos/component/wifi)
  # The usrcfg files come from the SDK too: its wifi_user_conf and table
  # layouts are the ABI the NP driver library was built against (the copies in
  # this module carry the CA32 host flavor).
  set(AMEBA_RTOS_USRCFG ${WEST_TOPDIR}/modules/hal/ameba-rtos/component/soc/usrcfg/amebasmart)
  zephyr_library_sources(
    ${AMEBA_RTOS_WIFI}/common/rtw_event.c
    ${AMEBA_RTOS_WIFI}/common/rtw_task_size.c
    ${AMEBA_RTOS_USRCFG}/ameba_xtal_trackingcfg.c
    ${AMEBA_RTOS_USRCFG}/ameba_wificfg.c
    ${AMEBA_RTOS_USRCFG}/ameba_wifi_power_table_usrcfg.c
    ${AMEBA_RTOS_USRCFG}/ameba_wifi_country_code_table_usrcfg.c
  )
  # See rtw_atomic_zephyr_compat.h.
  zephyr_library_compile_options(
    "SHELL:$<$<COMPILE_LANGUAGE:C>:-include ${CMAKE_CURRENT_SOURCE_DIR}/np_include/rtw_atomic_zephyr_compat.h>")
  zephyr_include_directories(
    np_include
    ${AMEBA_RTOS_WIFI}/whc
    ${AMEBA_RTOS_WIFI}/whc/whc_dev
    ${AMEBA_RTOS_WIFI}/whc/whc_common
    ${AMEBA_RTOS_WIFI}/whc/whc_common/ipc
    ${AMEBA_RTOS_WIFI}/common
    ${AMEBA_RTOS_WIFI}/api
    ${AMEBA_RTOS_WIFI}/driver/include
    ${AMEBA_RTOS_WIFI}/driver/intf
    ${WEST_TOPDIR}/modules/hal/ameba-rtos/component/soc/common/diagnose
    ${WEST_TOPDIR}/modules/hal/ameba-rtos/component/soc/usrcfg/common
  )
endif()
