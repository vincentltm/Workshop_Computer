cmake_minimum_required (VERSION 3.13)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
include(pico_sdk_import.cmake)
project(drums C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(drums)

target_sources(drums PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/drums.cpp
        ${CMAKE_CURRENT_LIST_DIR}/resources_q15.cpp
      )

# Include the current directory and the ComputerCard library
target_include_directories(drums PUBLIC 
    ${CMAKE_CURRENT_LIST_DIR} 
    ${CMAKE_CURRENT_LIST_DIR}/ComputerCard
)

# Avoid startup timing problems
target_compile_definitions(drums PUBLIC PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)
target_compile_definitions(drums PRIVATE PICO_DEFAULT_UART_BAUD_RATE=115200)

pico_enable_stdio_uart(drums 0)
pico_enable_stdio_usb(drums 1)
pico_add_extra_outputs(drums)

target_compile_options(drums PRIVATE -Wdouble-promotion -Wfloat-conversion -Wall -Wextra)
target_link_options(drums PRIVATE -Wl,--print-memory-usage)
	
target_link_libraries(drums pico_unique_id pico_stdlib hardware_dma hardware_i2c hardware_pwm hardware_adc hardware_spi hardware_timer pico_multicore hardware_flash hardware_vreg hardware_clocks)
