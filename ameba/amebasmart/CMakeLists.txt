# Copyright (c) 2026 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

# Unlocks SAU/SCB_NS CMSIS registers used by the TrustZone config code kept under Zephyr.
zephyr_compile_options_ifdef(CONFIG_MCUBOOT -mcmse)

# Pre-compiled libraries
set(LIB_PATH ${ZEPHYR_CURRENT_MODULE_DIR}/zephyr/blobs/ameba/${CONFIG_SOC_SERIES}/lib)

# Per-core WiFi libraries: the blob checkout does not ship them; take them from
# the ameba-rtos SDK west project (same v1.3.0 generation as the CA32 host
# blobs).  A copy under the blob checkout, if present, wins.
set(AMEBA_RTOS_PROJ ${WEST_TOPDIR}/modules/hal/ameba-rtos/component/soc/amebasmart/project)
set(NP_WIFI_LIB_DIR ${LIB_PATH}/km4)
if(NOT EXISTS ${NP_WIFI_LIB_DIR}/lib_wifi_whc_np.a AND EXISTS ${AMEBA_RTOS_PROJ}/project_hp/lib/application/lib_wifi_whc_np.a)
  set(NP_WIFI_LIB_DIR ${AMEBA_RTOS_PROJ}/project_hp/lib/application)
endif()
set(LP_WIFI_LIB_DIR ${LIB_PATH}/km0)
if(NOT EXISTS ${LP_WIFI_LIB_DIR}/lib_wifi_fw.a AND EXISTS ${AMEBA_RTOS_PROJ}/project_lp/lib/application/lib_wifi_fw.a)
  set(LP_WIFI_LIB_DIR ${AMEBA_RTOS_PROJ}/project_lp/lib/application)
endif()

if(EXISTS ${LIB_PATH})
  zephyr_link_libraries_ifdef(CONFIG_WIFI_AMEBA ${LIB_PATH}/lib_wifi_whc_ipc_host.a)
  zephyr_link_libraries_ifdef(CONFIG_WIFI_AMEBA ${LIB_PATH}/lib_rtw_event.a)
  zephyr_link_libraries_ifdef(CONFIG_WIFI_AMEBA ${LIB_PATH}/lib_wifi_api.a)
  zephyr_link_libraries_ifdef(CONFIG_WIFI_AMEBA ${LIB_PATH}/lib_wpa_lite.a)
  zephyr_link_libraries_ifdef(CONFIG_WIFI_AMEBA ${LIB_PATH}/lib_wifi_com_sec.a)
  zephyr_link_libraries_ifdef(CONFIG_WIFI_AMEBA ${LIB_PATH}/lib_wifi_rtk_app.a)
  zephyr_link_libraries_ifdef(CONFIG_WIFI_AMEBA ${LIB_PATH}/lib_whc_ap.a)
  zephyr_link_libraries_ifdef(CONFIG_WIFI_AMEBA ${LIB_PATH}/lib_coex_api.a)
  zephyr_link_libraries_ifdef(CONFIG_USB_DEVICE_AMEBA
    ${LIB_PATH}/lib_usbd.a
  )

  zephyr_link_libraries_ifdef(CONFIG_USB_HOST_AMEBA
    ${LIB_PATH}/lib_usbh.a
  )

  if(CONFIG_SOC_RTL8730E_CA32)
    zephyr_link_libraries(
      ${LIB_PATH}/lib_chipinfo.a
      ${LIB_PATH}/lib_rom.a
    )
    # CA32 PM: cpu_suspend/cpu_resume/cpu_do_suspend/cpu_suspend_save,
    # vPortBackupRegs/vPortRestoreRegs, SOCPS_Save_GIC/SOCPS_Restore_GIC,
    # SOCPS_PG_Enter, SOCPS_SleepPG_LIB/SOCPS_SleepCG_LIB,
    # SOCPS_Backup_CPU1/SOCPS_Restore_CPU1.
    zephyr_link_libraries_ifdef(CONFIG_PM ${LIB_PATH}/lib_pmc.a)
  elseif(CONFIG_SOC_RTL8730E_KM0)
    # KM0 (Cortex-M23, soft-float LP core): the km4/lib_chipinfo.a archive is
    # built for the KM4 (Cortex-M55) hard-float ABI and cannot be linked into
    # the soft-float KM0 image.  KM0's ROM-resident symbols are resolved through
    # the KM0 ROM symbol linker script instead (see km0/CMakeLists.txt), so no
    # chipinfo archive is linked here.
    #
    # WiFi MAC firmware; must match the SDK generation of the KM4/CA32 WHC
    # libraries (the WHC IPC shared structs are not ABI-stable across releases).
    # Group: the firmware entry (wififw_task_create) pulls the member whose
    # .ipc.table.data entries register the RX handlers; the group resolves the
    # archive's internal circular references.
    zephyr_link_libraries_ifdef(CONFIG_WIFI_FW_EN
      -Wl,--start-group ${LP_WIFI_LIB_DIR}/lib_wifi_fw.a -Wl,--end-group)
  else()
    zephyr_link_libraries(${LIB_PATH}/km4/lib_chipinfo.a)
    # WHC NP role: 802.11 driver + WHC IPC device (same SDK-generation caveat).
    # One --start-group: the archives have circular internal and mutual
    # references; whc_ipc_dev_init pulls the member whose .ipc.table.data
    # entries register the RX handlers.
    zephyr_link_libraries_ifdef(CONFIG_AS_INIC_NP
      -Wl,--start-group
      ${NP_WIFI_LIB_DIR}/lib_wifi_whc_np.a
      ${NP_WIFI_LIB_DIR}/lib_wifi_com_sec.a
      ${LIB_PATH}/km4/lib_chipinfo.a
      -Wl,--end-group)
    # The driver library drives the codec mic-bias/HPO power modes.
    zephyr_library_sources_ifdef(CONFIG_AS_INIC_NP
      ${AMEBA_RTOS_PROJ}/../fwlib/ram_hp/ameba_codec.c)
    if(CONFIG_AS_INIC_NP AND CONFIG_BT_COEXIST)
      zephyr_link_libraries(
        -Wl,--start-group ${NP_WIFI_LIB_DIR}/lib_coex.a -Wl,--end-group)
    endif()
  endif()

endif()

# Include directories
zephyr_include_directories(
  include
  source/fwlib/include
  source/misc
  source/swlib
)

zephyr_include_directories(
  ${ZEPHYR_MBEDTLS_MODULE_DIR}/include
)

# Base SOC sources — compiled for all amebasmart targets (CA32 app and KM4 MCUboot)
zephyr_library_sources(
  source/fwlib/ram_common/ameba_arch.c
  source/fwlib/ram_common/ameba_clk.c
  source/fwlib/ram_common/ameba_pll.c
  source/fwlib/ram_common/ameba_otp.c
  source/fwlib/ram_common/ameba_reset.c
  source/fwlib/ram_common/ameba_ipc_api.c
  source/swlib/log.c
)

if(CONFIG_SOC_RTL8730E_CA32)
  zephyr_library_sources(
    source/fwlib/ram_common/ameba_ipc_ram.c
    source/fwlib/ram_hp/ameba_system.c
    source/fwlib/ap_peripheral/ameba_delay.c
  )
endif()

# The vendor IPC dispatcher is needed wherever .ipc.table.data consumers run:
# CA32 always (above), KM4/KM0 for their WiFi roles.
if(CONFIG_AS_INIC_NP OR CONFIG_WIFI_FW_EN)
  zephyr_library_sources(source/fwlib/ram_common/ameba_ipc_ram.c)
endif()

# Peripheral drivers
zephyr_library_sources_ifdef(CONFIG_PINCTRL source/fwlib/ram_common/ameba_pinmux.c)
zephyr_library_sources_ifdef(CONFIG_WIFI_AMEBA source/misc/ameba_pmu.c)
# The KM4/KM0 WiFi libraries import the pmu_* wakelock API as well.
if(CONFIG_AS_INIC_NP OR CONFIG_WIFI_FW_EN)
  zephyr_library_sources(source/misc/ameba_pmu.c)
endif()
# _sscanf_ss for the NP driver library, from the SDK west project.
zephyr_library_sources_ifdef(CONFIG_AS_INIC_NP
  ${AMEBA_RTOS_PROJ}/../swlib/sscanf_minimal.c)
zephyr_library_sources_ifdef(CONFIG_RTC_AMEBA source/fwlib/ram_common/ameba_rtc.c)
zephyr_library_sources_ifdef(CONFIG_GPIO_AMEBA source/fwlib/ram_common/ameba_gpio.c)
zephyr_library_sources_ifdef(CONFIG_SOC_FLASH_AMEBA source/fwlib/ram_common/ameba_flash_ram.c)
zephyr_library_sources_ifdef(CONFIG_DMA_AMEBA source/fwlib/ram_common/ameba_gdma_ram.c)
zephyr_library_sources_ifdef(CONFIG_I2C_AMEBA source/fwlib/ram_common/ameba_i2c.c)
zephyr_library_sources_ifdef(CONFIG_LEDC_AMEBA source/fwlib/ram_common/ameba_ledc.c)
zephyr_library_sources_ifdef(CONFIG_SPI_AMEBA source/fwlib/ram_hp/ameba_spi.c)
zephyr_library_sources_ifdef(CONFIG_COUNTER_TMR_AMEBA source/fwlib/ram_common/ameba_tim.c)
zephyr_library_sources_ifdef(CONFIG_PWM_AMEBA source/fwlib/ram_common/ameba_tim.c)
zephyr_library_sources_ifdef(CONFIG_UART_AMEBA source/fwlib/ram_hp/ameba_uart.c)
zephyr_library_sources_ifdef(CONFIG_UART_AMEBA_LOGUART source/fwlib/ram_common/ameba_loguart.c)
zephyr_library_sources_ifdef(CONFIG_TEMP_AMEBA source/fwlib/ram_common/ameba_thermal.c)
zephyr_library_sources_ifdef(CONFIG_ADC_AMEBA source/fwlib/ram_common/ameba_adc.c)
zephyr_library_sources_ifdef(CONFIG_SDHC_AMEBA source/fwlib/ram_hp/ameba_sdio_host.c)
zephyr_library_sources_ifdef(CONFIG_SDHC_AMEBA source/fwlib/ram_hp/ameba_sd_host_wrapper.c)
zephyr_library_sources_ifdef(CONFIG_AMEBA_LCDC source/fwlib/ram_hp/ameba_lcdc.c)
zephyr_library_sources_ifdef(CONFIG_MIPI_DSI_AMEBA source/fwlib/ram_hp/ameba_mipi.c)
zephyr_library_sources_ifdef(CONFIG_WDT_AMEBA source/fwlib/ram_common/ameba_wdg.c)

zephyr_library_sources_ifdef(CONFIG_PM
  source/fwlib/ram_common/ameba_psram.c
  source/fwlib/ram_common/ameba_aontimer.c
  source/fwlib/ap_peripheral/ameba_pmc_ram.c
)

if(CONFIG_MCUBOOT)
  zephyr_link_libraries(-T${CMAKE_CURRENT_SOURCE_DIR}/ld/ameba_rom_symbol_acut.ld)

  zephyr_include_directories(source/loader/mcuboot)
  zephyr_include_directories(source/loader/bootloader)

  zephyr_library_sources(
    source/loader/bootloader/bootloader_hp.c
    source/loader/bootloader/boot_trustzone_hp.c
    source/loader/mcuboot/boot_prepare.c
    source/fwlib/ram_common/ameba_swr.c
    source/fwlib/ram_common/ameba_swr_calib.c
    source/fwlib/ram_common/ameba_psram.c
    source/fwlib/ram_common/ameba_flashclk.c
    source/fwlib/ram_common/ameba_ddrphy.c
    source/fwlib/ram_common/ameba_rxi316_dram_init.c
    source/fwlib/ram_common/ameba_flash_ram.c
  )
endif()
