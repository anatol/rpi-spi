cmake_minimum_required(VERSION 4.2)

include(pico_sdk_import.cmake)
include(CheckCCompilerFlag)
include(CheckCXXCompilerFlag)

project(rpi-spi C CXX ASM)
set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

option(SP_ENABLE_DIAG_CONSOLE "Enable USB diagnostic console interface" OFF)

if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/tinyusb")
    # Prefer project-local TinyUSB checkout when present.
    set(PICO_TINYUSB_PATH "${CMAKE_CURRENT_LIST_DIR}/tinyusb" CACHE PATH "Project-local TinyUSB" FORCE)
endif()

add_executable(rpi-spi
    main.c
    serprog.c
    status_led.c
    usb_descriptors.c
)

pico_generate_pio_header(rpi-spi ${CMAKE_CURRENT_LIST_DIR}/status_led.pio)

if (SP_ENABLE_DIAG_CONSOLE)
    target_sources(rpi-spi PRIVATE diag.c)
endif()

target_include_directories(rpi-spi PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(rpi-spi PRIVATE
    pico_stdlib
    hardware_spi
    hardware_uart
    hardware_pio
    tinyusb_device
    tinyusb_board
)

target_compile_definitions(rpi-spi PRIVATE
    SP_ENABLE_DIAG_CONSOLE=$<BOOL:${SP_ENABLE_DIAG_CONSOLE}>
)

pico_enable_stdio_uart(rpi-spi 0)
pico_enable_stdio_usb(rpi-spi 0)

pico_add_extra_outputs(rpi-spi)
