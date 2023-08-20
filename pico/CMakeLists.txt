cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(adc VERSION 0.1 LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

add_executable(adc_main
    adc_main.c
)

pico_enable_stdio_usb(adc_main 0)
pico_enable_stdio_uart(adc_main 1)

# pull in common dependencies
target_link_libraries(adc_main
    pico_stdlib
    pico_malloc
    hardware_adc
    hardware_dma
    hardware_sync
    hardware_spi
    )

if(DEFINED ENV{DEBUG_MODE})
    target_compile_definitions(adc_main PUBLIC
        DEBUG_MODE
    )
endif()

pico_add_extra_outputs(adc_main)
