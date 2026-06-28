cmake_minimum_required(VERSION 3.13)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
include(pico_sdk_import.cmake)
project(modes C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(modes)

target_sources(modes PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/modes.cpp
    ${CMAKE_CURRENT_LIST_DIR}/resources_q15.cpp
    ${CMAKE_CURRENT_LIST_DIR}/samples_flash.cpp
    ${CMAKE_CURRENT_LIST_DIR}/resonator_q15.cpp
    ${CMAKE_CURRENT_LIST_DIR}/string_q15.cpp
    ${CMAKE_CURRENT_LIST_DIR}/usb_midi_host.c
    ${CMAKE_CURRENT_LIST_DIR}/usb_midi_host_app_driver.c
    ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
)

# Include the current directory and the ComputerCard library
target_include_directories(modes PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/ComputerCard
)

# Essential: prevents reset boot failures
target_compile_definitions(modes PUBLIC PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)
target_compile_definitions(modes PRIVATE PICO_DEFAULT_UART_BAUD_RATE=115200)
target_compile_definitions(modes PRIVATE CFG_TUSB_MCU=OPT_MCU_RP2040)

# Run all code from RAM to eliminate flash cache jitter
pico_set_binary_type(modes copy_to_ram)

pico_enable_stdio_uart(modes 0)
pico_enable_stdio_usb(modes 0)
pico_add_extra_outputs(modes)

# Compiler warnings: catch accidental float promotion, general warnings
target_compile_options(modes PRIVATE -Wdouble-promotion -Wfloat-conversion -Wall -Wextra)
target_link_options(modes PRIVATE -Wl,--print-memory-usage)

target_link_libraries(modes
    pico_unique_id
    pico_stdlib
    hardware_dma
    hardware_i2c
    hardware_pwm
    hardware_adc
    hardware_spi
    hardware_timer
    pico_multicore
    hardware_clocks
    tinyusb_host
    tinyusb_device
)
