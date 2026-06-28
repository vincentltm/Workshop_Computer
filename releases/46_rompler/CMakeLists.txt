cmake_minimum_required(VERSION 3.13)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
include(pico_sdk_import.cmake)
project(rompler C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

set(ROMPLER_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/rompler.cpp
    ${CMAKE_CURRENT_LIST_DIR}/usb_midi_host.c
    ${CMAKE_CURRENT_LIST_DIR}/usb_midi_host_app_driver.c
    ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
)

set(ROMPLER_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/ComputerCard
)

set(ROMPLER_LIBS
    pico_unique_id
    pico_stdlib
    hardware_dma
    hardware_i2c
    hardware_pwm
    hardware_adc
    hardware_spi
    hardware_timer
    pico_multicore
    hardware_flash
    hardware_watchdog
    hardware_vreg
    hardware_clocks
    tinyusb_host
    tinyusb_device
)

function(add_rompler_firmware target_name sf2_max_bytes)
    add_executable(${target_name})

    target_sources(${target_name} PUBLIC ${ROMPLER_SOURCES})

    # Include the current directory and the ComputerCard library
    target_include_directories(${target_name} PUBLIC ${ROMPLER_INCLUDE_DIRS})

    # Essential: prevents reset boot failures
    target_compile_definitions(${target_name} PUBLIC PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)
    target_compile_definitions(${target_name} PUBLIC COMPUTERCARD_SAMPLE_RATE_DIV=1)
    target_compile_definitions(${target_name} PRIVATE PICO_DEFAULT_UART_BAUD_RATE=115200)
    target_compile_definitions(${target_name} PRIVATE CFG_TUSB_MCU=OPT_MCU_RP2040)
    target_compile_definitions(${target_name} PRIVATE ROMPLER_SF2_MAX_BYTES=${sf2_max_bytes})

    pico_enable_stdio_uart(${target_name} 1)
    pico_enable_stdio_usb(${target_name} 0)
    pico_add_extra_outputs(${target_name})

    # Compiler warnings: catch accidental float promotion, general warnings, optimize for speed
    target_compile_options(${target_name} PRIVATE -Wdouble-promotion -Wfloat-conversion -Wall -Wextra -O3)
    target_link_options(${target_name} PRIVATE -Wl,--print-memory-usage)

    target_link_libraries(${target_name} ${ROMPLER_LIBS})

    # Automatically copy the generated .uf2 to the web folder on successful build
    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_CURRENT_BINARY_DIR}/${target_name}.uf2
                ${CMAKE_CURRENT_SOURCE_DIR}/web/${target_name}.uf2
        COMMENT "Copying ${target_name}.uf2 to web/ folder"
    )
endfunction()

add_rompler_firmware(rompler 1048576)
add_rompler_firmware(rompler_16mb 15728640)
