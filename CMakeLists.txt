cmake_minimum_required(VERSION 3.13)

# ---------------------------------------------------------------------------
# Project: Feather USB Cleaner
#
# Requires:
#   - Pico SDK 2.x            (PICO_SDK_PATH or ~/.pico-sdk/sdk)
#   - Pico-PIO-USB            (PICO_PIO_USB_PATH or ~/.pico-sdk/pio_usb)
#   - arm-none-eabi toolchain on PATH
# ---------------------------------------------------------------------------

set(PICO_BOARD adafruit_feather_rp2040_usb_host CACHE STRING "Pico board type")

if(NOT DEFINED PICO_SDK_PATH)
    if(DEFINED ENV{PICO_SDK_PATH})
        set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
    elseif(EXISTS "$ENV{HOME}/.pico-sdk/sdk/external/pico_sdk_import.cmake")
        set(PICO_SDK_PATH "$ENV{HOME}/.pico-sdk/sdk")
    endif()
endif()

if(NOT DEFINED PICO_SDK_PATH OR NOT EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    message(FATAL_ERROR
        "Pico SDK not found. Set PICO_SDK_PATH (e.g. export PICO_SDK_PATH=~/pico-sdk).")
endif()

# Pico-PIO-USB provides the USB host PHY: the RP2040 has no hardware host
# controller. This *must* be known before pico_sdk_init(), because the SDK
# decides there whether to compile TinyUSB's hcd_pio_usb.c.
# The bundled copy is patched to service bulk endpoints more than once per
# 1 ms frame; upstream's one-transaction-per-frame scheduler caps MSC writes
# at 64 KB/s. Override PICO_PIO_USB_PATH only if you know what you lose.
if(DEFINED ENV{PICO_PIO_USB_PATH})
    set(PICO_PIO_USB_PATH $ENV{PICO_PIO_USB_PATH})
elseif(NOT DEFINED PICO_PIO_USB_PATH)
    set(PICO_PIO_USB_PATH "${CMAKE_CURRENT_LIST_DIR}/third_party/Pico-PIO-USB")
endif()

if(NOT PICO_PIO_USB_PATH OR NOT EXISTS "${PICO_PIO_USB_PATH}/src/pio_usb.c")
    message(FATAL_ERROR
        "Pico-PIO-USB not found. Clone https://github.com/sekigon-gonnoc/Pico-PIO-USB "
        "and set PICO_PIO_USB_PATH to it.")
endif()
set(PICO_PIO_USB_PATH "${PICO_PIO_USB_PATH}" CACHE PATH "Path to Pico-PIO-USB")
message(STATUS "Pico-PIO-USB path is ${PICO_PIO_USB_PATH}")

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(feather_usb_cleaner C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# --- Application ----------------------------------------------------------
add_executable(feather_usb_cleaner
        main.c
        fat32_builder.c
)

# WS2812/NeoPixel PIO program -> ws2812.pio.h
pico_generate_pio_header(feather_usb_cleaner ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

# Our tusb_config.h must win over the SDK's default one (host + device roles).
target_include_directories(feather_usb_cleaner PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Microseconds of each 1 ms USB frame that may be spent on back-to-back bulk
# transactions (see third_party/Pico-PIO-USB/PATCHES.md). 0 = upstream
# one-transaction-per-frame behaviour (~64 KB/s).
set(PIO_USB_BULK_FRAME_BUDGET_US 700 CACHE STRING
        "Bulk time budget per 1 ms USB frame in microseconds")

# pico_stdio_usb normally provides the CDC descriptors, but it disables them
# when TinyUSB is linked directly (LIB_TINYUSB_HOST). We do want them: the
# native USB-C port is our console.
target_compile_definitions(feather_usb_cleaner PRIVATE
        PICO_STDIO_USB_USE_DEFAULT_DESCRIPTORS=1
        PICO_STDIO_USB_ENABLE_TINYUSB_INIT=0
        PIO_USB_BULK_FRAME_BUDGET_US=${PIO_USB_BULK_FRAME_BUDGET_US}
)

if(TARGET tinyusb_pico_pio_usb)
    # Normal path: the SDK added PIO-USB to TinyUSB for us.
    set(PIO_USB_TARGET tinyusb_pico_pio_usb)
else()
    # Fallback: add Pico-PIO-USB and TinyUSB's PIO-USB host driver ourselves.
    message(STATUS "SDK did not create tinyusb_pico_pio_usb - adding PIO-USB host driver manually")
    add_subdirectory(${PICO_PIO_USB_PATH} ${CMAKE_BINARY_DIR}/pico_pio_usb)
    set(PIO_USB_TARGET pico_pio_usb)
    target_sources(feather_usb_cleaner PRIVATE
            ${PICO_SDK_PATH}/lib/tinyusb/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
    )
endif()

target_link_libraries(feather_usb_cleaner PRIVATE
        pico_stdlib
        tinyusb_host
        tinyusb_board
        ${PIO_USB_TARGET}
        hardware_pio
)

# Console on the native USB-C port (CDC-ACM), no UART pins needed.
pico_enable_stdio_usb(feather_usb_cleaner 1)
pico_enable_stdio_uart(feather_usb_cleaner 0)

# .uf2 + .elf/.bin/.hex/.map
pico_add_extra_outputs(feather_usb_cleaner)
