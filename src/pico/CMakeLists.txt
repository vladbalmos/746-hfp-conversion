cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(audio_main VERSION 0.1 LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )
    

add_executable(audio_main
    audio.c
    audio_main.c
)

target_include_directories(audio_main PUBLIC ./../common)

pico_enable_stdio_usb(audio_main 0)
pico_enable_stdio_uart(audio_main 1)

# pull in common dependencies
target_link_libraries(audio_main
    pico_stdlib
    pico_malloc
    pico_multicore
    pico_i2c_slave
    hardware_adc
    hardware_dma
    hardware_sync
    hardware_i2c
    hardware_spi
    )

if(DEFINED ENV{DEBUG_MODE})
    target_compile_definitions(audio_main PUBLIC
        DEBUG_MODE
    )
    target_compile_options(audio_main PRIVATE -DPARAM_ASSERTIONS_ENABLE_ALL=1)
endif()

pico_add_extra_outputs(audio_main)
