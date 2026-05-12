cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rpi-spi C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/tinyusb")
    # Prefer project-local TinyUSB checkout when present.
    set(PICO_TINYUSB_PATH "${CMAKE_CURRENT_LIST_DIR}/tinyusb" CACHE PATH "Project-local TinyUSB" FORCE)
endif()

add_executable(rpi-spi
    main.c
    usb_descriptors.c
)

target_include_directories(rpi-spi PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(rpi-spi PRIVATE
    pico_stdlib
    hardware_spi
    tinyusb_device
    tinyusb_board
)

pico_enable_stdio_uart(rpi-spi 0)
pico_enable_stdio_usb(rpi-spi 0)

pico_add_extra_outputs(rpi-spi)
