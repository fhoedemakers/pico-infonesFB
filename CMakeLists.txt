cmake_minimum_required(VERSION 3.13)

#set(PICO_DEOPTIMIZED_DEBUG on)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

#set(PICO_SDK_PATH "~/pico/pico-sdk")
set(PICO_BOARD pico2 CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(picones C CXX ASM)
pico_sdk_init()

add_executable(picones
    main.cpp
    hid_app.cpp
    gamepad.cpp
    tar.cpp
    contra.c
)

pico_set_program_name(picones "picones")
pico_set_program_version(picones "0.1")

pico_enable_stdio_uart(picones 1)
pico_enable_stdio_usb(picones 0)

# tinyusb
set(FAMILY rp2040)
set(BOARD pico_sdk)
set(TINYUSB_DEBUG_LEVE 0)
include(${PICO_TINYUSB_PATH}/hw/bsp/${FAMILY}/family.cmake)
if (PICO_SDK_VERSION_MAJOR LESS 2)
    family_configure_target(picones)
else()
    family_configure_target(picones "" )
endif()
# add_compile_definitions(CFG_TUSB_DEBUG=0)

target_link_libraries(picones
PRIVATE
    pico_stdlib
    pico_multicore
    hardware_dma
    hardware_pio
    hardware_i2c
    hardware_interp
    hardware_timer
    hardware_clocks
    hardware_pwm
    hardware_flash
    dvi
    util
    infones
    tinyusb_host
)

target_include_directories(picones
PRIVATE
    pico_lib
    infones
    .
)

add_subdirectory(pico_lib/dvi)
add_subdirectory(pico_lib/util)
add_subdirectory(infones)
target_link_options(picones PRIVATE -Xlinker --print-memory-usage --data-sections)
pico_add_extra_outputs(picones)




