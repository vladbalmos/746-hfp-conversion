cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake)

project(hfp_746 VERSION 0.1 LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)

pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

add_library(pico_btstack_audio INTERFACE)
target_sources(pico_btstack_audio INTERFACE
        ${PICO_BTSTACK_PATH}/src/btstack_audio.c
        bt_audio.c
        )

add_executable(hfp_746
    utils.c
    ringer.c
    dialer.c
    dac_audio.c
    # bt.c
    bt_a2dp_sink.c
    hfp_746.c
)

pico_enable_stdio_usb(hfp_746 0)
pico_enable_stdio_uart(hfp_746 1)

# pull in common dependencies
target_link_libraries(hfp_746
    pico_stdlib
    pico_malloc
    pico_btstack_classic
    pico_btstack_cyw43
    pico_btstack_audio
    pico_btstack_sbc_decoder
    pico_btstack_sbc_encoder
    pico_cyw43_arch_none
    hardware_adc
    hardware_pwm
    hardware_dma
    hardware_sync
    hardware_spi
    hardware_clocks
    )

if(DEFINED ENV{DEBUG_MODE})
    target_compile_definitions(hfp_746 PUBLIC
        DEBUG_MODE
    )
endif()

if(DEFINED ENV{BT_DEBUG_MODE})
    target_compile_definitions(hfp_746 PUBLIC
        ENABLE_LOG_INFO
        ENABLE_LOG_ERROR
        BT_DEBUG_MODE
    )
endif()

target_include_directories(hfp_746 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR} # For btstack config
    )

pico_add_extra_outputs(hfp_746)
