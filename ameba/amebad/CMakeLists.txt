# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0
zephyr_compile_options(-mcmse)

zephyr_include_directories(
  include
  source/cmsis
  source/misc
  source/swlib
  source/fwlib/include
  source/app/monitor/include
)

# used by common
zephyr_library_sources(
  source/fwlib/ram_hp/ameba_system.c
  source/swlib/log.c
)

# used by driver, need use zephyr_library_sources_ifdef
zephyr_library_sources_ifdef(CONFIG_GPIO_AMEBA source/fwlib/ram_common/ameba_gpio.c)
zephyr_library_sources_ifdef(CONFIG_ADC_AMEBA source/fwlib/ram_common/ameba_adc.c)
zephyr_library_sources_ifdef(CONFIG_SOC_FLASH_AMEBA source/fwlib/ram_common/ameba_flash_ram.c)
zephyr_library_sources_ifdef(CONFIG_DMA_AMEBA source/fwlib/ram_common/ameba_gdma_ram.c)
zephyr_library_sources_ifdef(CONFIG_I2C_AMEBA source/fwlib/ram_common/ameba_i2c.c)
zephyr_library_sources_ifdef(CONFIG_RTC_AMEBA source/fwlib/ram_common/ameba_rtc.c)
zephyr_library_sources_ifdef(CONFIG_SPI_AMEBA source/fwlib/ram_hp/ameba_ssi.c)
zephyr_library_sources_ifdef(CONFIG_COUNTER_TMR_AMEBA source/fwlib/ram_common/ameba_tim.c)
if(CONFIG_UART_AMEBA_LOGUART OR CONFIG_UART_AMEBA)
    zephyr_library_sources(source/fwlib/ram_common/ameba_uart.c)
endif()
zephyr_library_sources_ifdef(CONFIG_WDT_AMEBA source/fwlib/ram_common/ameba_wdg.c)
zephyr_library_sources_ifdef(CONFIG_WIFI_AMEBA source/misc/ameba_freertos_pmu.c)
zephyr_library_sources_ifdef(CONFIG_WIFI_AMEBA source/swlib/sscanf_minimal.c)

zephyr_link_libraries(
  gcc
  -T${CMAKE_CURRENT_SOURCE_DIR}/ld/ameba_rom_symbol_acut.ld
)
