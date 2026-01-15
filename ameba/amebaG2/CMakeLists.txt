#
# SPDX-License-Identifier: Apache-2.0

zephyr_compile_options(-mcmse)

zephyr_include_directories(
  include
  source/fwlib/include
  source/fwlib/include/cmsis
  source/fwlib/include/rom
  source/app/monitor/include
)

# used by common
zephyr_library_sources(
  source/fwlib/ram_common/ameba_reset.c
  source/fwlib/ram_common/ameba_clk.c
  source/fwlib/ram_common/ameba_rcc.c
  source/fwlib/ram_common/ameba_rtc.c
)

# used by driver, need use zephyr_library_sources_ifdef
zephyr_library_sources_ifdef(CONFIG_ADC_AMEBA source/fwlib/ram_common/ameba_adc.c)
zephyr_library_sources_ifdef(CONFIG_CAN_AMEBA_A2C source/fwlib/ram_common/ameba_a2c.c)
zephyr_library_sources_ifdef(CONFIG_ETH_AMEBA source/fwlib/ram_km4tz/ameba_phy.c)
zephyr_library_sources_ifdef(CONFIG_ETH_AMEBA source/fwlib/ram_km4tz/ameba_ethernet.c)
zephyr_library_sources_ifdef(CONFIG_AMEBA_LCDC source/fwlib/ram_common/ameba_lcdc.c)
zephyr_library_sources_ifdef(CONFIG_MIPI_DBI_AMEBA_LCDC source/fwlib/ram_common/ameba_lcdc.c)
zephyr_library_sources_ifdef(CONFIG_INPUT_CTC_AMEBA source/fwlib/ram_common/ameba_captouch.c)
zephyr_library_sources_ifdef(CONFIG_I2S_AMEBA source/fwlib/ram_common/ameba_audio_clock.c)
zephyr_library_sources_ifdef(CONFIG_I2S_AMEBA source/fwlib/ram_common/ameba_pll.c)
zephyr_library_sources_ifdef(CONFIG_AUDIO_AMEBA_DMIC source/fwlib/ram_common/ameba_codec.c)
zephyr_library_sources_ifdef(CONFIG_SOC_FLASH_AMEBA source/fwlib/ram_common/ameba_flash_ram.c)
zephyr_library_sources_ifdef(CONFIG_DMA_AMEBA source/fwlib/ram_common/ameba_gdma_ram.c)
zephyr_library_sources_ifdef(CONFIG_I2C_AMEBA source/fwlib/ram_common/ameba_i2c.c)
zephyr_library_sources_ifdef(CONFIG_LEDC_AMEBA source/fwlib/ram_common/ameba_ledc.c)
zephyr_library_sources_ifdef(CONFIG_SPI_AMEBA source/fwlib/ram_common/ameba_spi.c)
zephyr_library_sources_ifdef(CONFIG_I2S_AMEBA source/fwlib/ram_common/ameba_sport.c)
zephyr_library_sources_ifdef(CONFIG_COUNTER_TMR_AMEBA source/fwlib/ram_common/ameba_tim.c)
zephyr_library_sources_ifdef(CONFIG_COUNTER_TMR_AMEBA source/fwlib/ram_common/ameba_ups.c)
zephyr_library_sources_ifdef(CONFIG_UART_AMEBA source/fwlib/ram_common/ameba_uart.c)
zephyr_library_sources_ifdef(CONFIG_TEMP_AMEBA source/fwlib/ram_common/ameba_thermal.c)
zephyr_library_sources_ifdef(CONFIG_REALTEK_AMEBA_ZEPHYR_USB source/fwlib/ram_common/ameba_usb.c)
zephyr_library_sources_ifdef(CONFIG_WIFI_AMEBA source/fwlib/ram_common/ameba_pmu.c)
zephyr_library_sources_ifdef(CONFIG_WIFI_AMEBA source/fwlib/ram_common/ameba_pmctimer.c)
zephyr_library_sources_ifdef(CONFIG_AMEBA_PPE source/fwlib/ram_common/ameba_ppe.c)

zephyr_link_libraries(
  -T${CMAKE_CURRENT_SOURCE_DIR}/ld/ameba_rom_symbol_bcut_s.ld
  -T${CMAKE_CURRENT_SOURCE_DIR}/ld/ameba_rom_symbol_bcut_wifi.ld
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
