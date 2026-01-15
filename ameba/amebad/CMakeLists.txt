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

# west sign
if(CONFIG_SOC_AMEBA_NP_IMAGE)
  file(GLOB blob_bin_file "${ZEPHYR_HAL_REALTEK_MODULE_DIR}/zephyr/blobs/${CONFIG_SOC_SERIES}/bin/*.bin")

  if(blob_bin_file)
    set(python_script_args
      --soc ${CONFIG_SOC_SERIES}
      --bin-file ${ZEPHYR_BINARY_DIR}/${KERNEL_BIN_NAME}
      --out-dir ${CMAKE_BINARY_DIR}
      --module-dir ${ZEPHYR_HAL_REALTEK_MODULE_DIR}
    )

    add_custom_target(zephyr.raw.map ALL
      DEPENDS ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
    )

    add_custom_command(
      OUTPUT ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
      COMMAND ${CMAKE_NM} ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} | sort > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.raw.map
      COMMAND ${CMAKE_OBJDUMP} -d ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} > ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}.asm
      COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_HAL_REALTEK_MODULE_DIR}/ameba/scripts/merge_bin.py ${python_script_args}
      DEPENDS ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} ${blob_bin_file} ${CMAKE_CURRENT_SOURCE_DIR}/manifest.json5
    )
  else()
    message(WARNING "Missing Realtek blobs. Run: `west blobs fetch hal_realtek`")
  endif()
endif()
