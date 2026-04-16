# Copyright (c) 2026, Realtek Semiconductor Corporation
# SPDX-License-Identifier: Apache-2.0

# Macro to add device IC sources and include directories
# Usage: add_soc_series_subdir(rtl8752h)
#        add_soc_series_subdir(rtl87x2g)
#
# If IC directory contains a CMakeLists.txt, use add_subdirectory.
# Otherwise, auto-add source files and include directories.
function(add_soc_series_subdir ic_name)
    string(TOUPPER ${ic_name} ic_name_upper)
    set(config_name "CONFIG_SOC_SERIES_${ic_name_upper}")
    set(ic_dir "${CMAKE_CURRENT_SOURCE_DIR}/${ic_name}")

    if(EXISTS "${ic_dir}/CMakeLists.txt")
        # Use subdirectory's CMakeLists.txt
        if(${config_name})
            add_subdirectory(${ic_dir} ${ic_name}_binary)
        endif()
    else()
        # Auto-add sources and include directories
        file(GLOB ic_sources "${ic_dir}/*.c")

        if(${config_name})
            if(ic_sources)
                zephyr_library_sources(${ic_sources})
            endif()
        endif()

        if(EXISTS ${ic_dir})
            zephyr_include_directories(${ic_dir})
        endif()
    endif()
endfunction()

# Add either a blob library or a stub source for a driver.
#
# Behavior:
#   - If CONFIG_BUILD_ONLY_NO_BLOBS=y, always use the stub source.
#   - Otherwise, use the blob library when present.
#   - If the blob library is absent, fall back to the stub source.
#   - If the same blob library is requested multiple times, link it only once.
#
# Usage:
#   add_blob_or_stub(
#       ${ZEPHYR_CURRENT_LIBRARY}
#       adc
#       <blob library path>
#       <stub source path>
#   )
function(add_blob_or_stub TARGET NAME BLOB_LIB_PATH STUB_FILE)
    string(TOUPPER "${NAME}" NAME_UPPER)

    set(use_stubs FALSE)

    if(CONFIG_BUILD_ONLY_NO_BLOBS)
        message(STATUS "CONFIG_BUILD_ONLY_NO_BLOBS=y, forcing ${NAME_UPPER} stubs")
        set(use_stubs TRUE)
    elseif(EXISTS "${BLOB_LIB_PATH}")
        get_property(linked_blobs GLOBAL PROPERTY REALTEK_BLOB_LIBS_LINKED)
        if(NOT linked_blobs)
            set(linked_blobs "")
        endif()

        list(FIND linked_blobs "${BLOB_LIB_PATH}" blob_index)
        if(blob_index EQUAL -1)
            message(STATUS "Using ${NAME_UPPER} blob library: ${BLOB_LIB_PATH}")
            target_link_libraries(${TARGET} PUBLIC ${BLOB_LIB_PATH})
            set_property(GLOBAL APPEND PROPERTY REALTEK_BLOB_LIBS_LINKED "${BLOB_LIB_PATH}")
        else()
            message(STATUS
                "${NAME_UPPER} blob library already linked, skipping duplicate: "
                "${BLOB_LIB_PATH}"
            )
        endif()
    else()
        message(STATUS
            "${NAME_UPPER} blob library not found: ${BLOB_LIB_PATH}\n"
            "Falling back to ${NAME_UPPER} stubs"
        )
        set(use_stubs TRUE)
    endif()

    if(use_stubs)
        if(EXISTS "${STUB_FILE}")
            message(STATUS "Using ${NAME_UPPER} stub source: ${STUB_FILE}")
            zephyr_library_sources(${STUB_FILE})
        else()
            message(FATAL_ERROR
                "${NAME_UPPER} stub source not found: ${STUB_FILE}"
            )
        endif()
    endif()
endfunction()

add_subdirectory(adc)
add_subdirectory_ifdef(CONFIG_REALTEK_BEE_BT bluetooth)
add_subdirectory(can)
add_subdirectory(codec)
add_subdirectory(clock)
add_subdirectory(crypto)
add_subdirectory(dma)
add_subdirectory(fmc)
add_subdirectory(gpio)
add_subdirectory(i2c)
add_subdirectory(i2s)
add_subdirectory(ir)
add_subdirectory(irq)
add_subdirectory(keyscan)
add_subdirectory_ifdef(CONFIG_REALTEK_BEE_MAC_802154 mac_802154)
add_subdirectory(pinmux)
add_subdirectory(pwm)
add_subdirectory(qdec)
add_subdirectory(rcc)
add_subdirectory(rtc)
add_subdirectory(sdhc)
add_subdirectory(spi)
add_subdirectory(timer)
add_subdirectory(uart)
add_subdirectory(usb)
if (CONFIG_REALTEK_BEE_CORE_WDT OR CONFIG_REALTEK_BEE_AON_WDT)
    add_subdirectory(wdt)
endif()

zephyr_include_directories(.)
