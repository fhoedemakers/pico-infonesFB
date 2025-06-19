cmake_minimum_required(VERSION 3.13)

#set(PICO_DEOPTIMIZED_DEBUG on)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

#set(PICO_SDK_PATH "~/pico/pico-sdk")
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/boards)
message(STATUS "PICO_BOARD_HEADER_DIRS: ${PICO_BOARD_HEADER_DIRS}")
if ( NOT DEFINED PICO_BOARD )
    set(PICO_BOARD adafruit_metro_rp2350) 
endif()
if ( NOT DEFINED BOARD )
    set(BOARD adafruit_metro_rp2350) 
endif()
# set PICO_PIO_USB_PATH from environment variabele with the same name
if (NOT DEFINED PICO_PIO_USB_PATH)
    if (DEFINED ENV{PICO_PIO_USB_PATH})
        set(PICO_PIO_USB_PATH $ENV{PICO_PIO_USB_PATH})
    else()
        set(PICO_PIO_USB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/pico_pio_usb)
    endif()
endif() 
message(STATUS "PICO_PIO_USB_PATH: ${PICO_PIO_USB_PATH}")
include(pico_sdk_import.cmake)
set(HSTX 1 CACHE BOOL "Enable HSTX")
project(picones C CXX ASM)
pico_sdk_init()
check_and_add_pico_pio_usb_support()
add_executable(picones
    main.cpp
    hid_app.cpp
    gamepad.cpp
    tar.cpp
    nes_rom.c
    myringbuffer.c
    mcp4822.c
)

pico_set_program_name(picones "picones")
pico_set_program_version(picones "0.1")

pico_enable_stdio_uart(picones 1)
pico_enable_stdio_usb(picones 0)

# tinyusb
#set(FAMILY rp2040)
# set BOARD when not set by the user
# if (NOT DEFINED BOARD)
#     set(BOARD pico_sdk)
# endif()

#set(TINYUSB_DEBUG_LEVE 0)
# show BOARD and FAMILY
# message(STATUS "BOARD: ${BOARD}")
# message(STATUS "FAMILY: ${FAMILY}")
# include(${PICO_TINYUSB_PATH}/hw/bsp/${FAMILY}/family.cmake)
# if (PICO_SDK_VERSION_MAJOR LESS 2)
#     family_configure_target(picones)
# else()
#     family_configure_target(picones "" )
# endif()
# add_compile_definitions(CFG_TUSB_DEBUG=0)
target_compile_definitions(picones PRIVATE
    HSTX=${HSTX}
)
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
    hardware_spi 
    hardware_irq
    infones
    tinyusb_host
    tinyusb_pico_pio_usb 
)
if (HSTX)
    message(STATUS "HSTX enabled")
else()
    message(STATUS "DVI enabled")
    target_link_libraries(picones PRIVATE
        util
        dvi
    )
    target_include_directories(picones
    PRIVATE
        pico_lib
    )
endif()
target_include_directories(picones
PRIVATE
    infones
    .
)
add_subdirectory(pico_lib/dvi)
add_subdirectory(pico_lib/util)
add_subdirectory(infones)
target_link_options(picones PRIVATE -Xlinker --print-memory-usage --data-sections)
pico_add_extra_outputs(picones)




