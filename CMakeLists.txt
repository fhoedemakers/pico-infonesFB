cmake_minimum_required(VERSION 3.20)

#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# include from environment var PICO_SDK_PATH
set(BOARD adafruit_metro_rp2350)
set(PICO_BOARD ${BOARD})
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
# set PICO_PIO_USB_PATH if not set
if(NOT DEFINED PICO_PIO_USB_PATH)
  set(PICO_PIO_USB_PATH $ENV{PICO_PIO_USB_PATH})
endif()
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/boards)
message(STATUS "Using PICO_SDK_PATH: ${PICO_SDK_PATH}")
message(STATUS "Using PICO_PIO_USB_PATH: ${PICO_PIO_USB_PATH}")
set(HSTX 1 CACHE BOOL "Enable HSTX")

include(pico_sdk_import.cmake)
include(${PICO_SDK_PATH}/lib/tinyusb/hw/bsp/family_support.cmake)

#family_get_project_name(PROJECT ${CMAKE_CURRENT_LIST_DIR})
set(PROJECT "picones")
project(${PROJECT} C CXX ASM)

# Checks this example is valid for the family and initializes the project
family_initialize_project(${PROJECT} ${CMAKE_CURRENT_LIST_DIR})
add_executable(${PROJECT}
    main.cpp
    hid_app.cpp
    gamepad.cpp
    tar.cpp
    nes_rom.c
    myringbuffer.c
    mcp4822.c
)
pico_set_program_name(${PROJECT} ${PROJECT})
pico_set_program_version(${PROJECT} "0.1")
pico_enable_stdio_uart(${PROJECT} 1)
pico_enable_stdio_usb(${PROJECT} 0)
target_compile_definitions(${PROJECT} PRIVATE
    HSTX=${HSTX}
)
# Configure compilation flags and libraries for the example without RTOS.
# See the corresponding function in hw/bsp/FAMILY/family.cmake for details.
family_configure_target(${PROJECT} "")
# require tinyusb_pico_pio_usb
target_link_libraries(${PROJECT} PUBLIC pico_stdlib tinyusb_device tinyusb_host tinyusb_pico_pio_usb )
rp2040_family_configure_example_warnings(${PROJECT})
# due to warnings from Pico-PIO-USB
# if (FAMILY STREQUAL rp2040)
#   target_compile_options(${PROJECT} PUBLIC
#     -Wno-error=shadow
#     -Wno-error=cast-align
#     -Wno-error=cast-qual
#     -Wno-error=redundant-decls
#     -Wno-error=sign-conversion
#     -Wno-error=conversion
#     -Wno-error=sign-compare
#     -Wno-error=unused-function
#     -Wno-error=format
#     -Wno-error=suggest-attribute=format
#     -Wno-error=unused-parameter
#     )
# endif ()
target_compile_options(${PROJECT} PUBLIC 
    -Wno-error 
    -Wno-shadow
    -Wno-cast-align
    -Wno-cast-qual
    -Wno-redundant-decls
    -Wno-sign-conversion
    -Wno-conversion
    -Wno-sign-compare
    -Wno-unused-function
    -Wno-format
    -Wno-suggest-attribute=format
    -Wno-unused-parameter
    -Wno-conversion
    -Wno-sequence-point
    -Wno-unused-variable
    -Wno-implicit-fallthrough
    -Wno-array-bounds
    -Wno-type-limits
    -Wno-parentheses
)
target_link_libraries(${PROJECT}
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
    target_link_libraries(${PROJECT} PRIVATE
        util
        dvi
    )
    target_include_directories(${PROJECT}
    PRIVATE
        pico_lib
    )
endif()
target_include_directories(${PROJECT}
PRIVATE
    infones
    .
)
if ( HSTX EQUAL 0)
    add_subdirectory(pico_lib/dvi)
    add_subdirectory(pico_lib/util)
endif()

add_subdirectory(infones)
target_link_options(${PROJECT} PRIVATE -Xlinker --print-memory-usage --data-sections)
pico_add_extra_outputs(${PROJECT})
# display the all the target_link_libraries
get_target_property(LINK_LIBS ${PROJECT} LINK_LIBRARIES)
message(STATUS "Target ${PROJECT} link libraries: ${LINK_LIBS}")
get_target_property(COMP_OPTS ${PROJECT} COMPILE_OPTIONS)
message(STATUS "Target ${PROJECT} compile options: ${COMP_OPTS}")

