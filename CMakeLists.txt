cmake_minimum_required(VERSION 3.20)

# Adafruit Metro RP2350 is default board config
if (NOT DEFINED BOARD )
  set(BOARD adafruit_metro_rp2350 CACHE STRING "Board to build for")
endif()
if (NOT DEFINED PICO_BOARD)
  set(PICO_BOARD adafruit_metro_rp2350 CACHE STRING "Pico board to build for")
endif()
if (NOT DEFINED ENABLE_PIO_USB)
  #message("Enabling PIO USB support by default")
  set(ENABLE_PIO_USB 0 CACHE BOOL "Enable PIO USB support")
endif()
if ( NOT DEFINED PIO_DP_PLUS_PIN )
  set(PIO_DP_PLUS_PIN -1 CACHE STRING "PIO USB DP pin.")
endif()

set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
# set PICO_PIO_USB_PATH from environment variable or fail if not set
if(NOT DEFINED PICO_PIO_USB_PATH)
  if(NOT DEFINED ENV{PICO_PIO_USB_PATH})
    message(FATAL_ERROR "PICO_PIO_USB_PATH environment variable is not set. Please fetch the repo https://github.com/sekigon-gonnoc/Pico-PIO-USB and set the PICO_PIO_USB_PATH environment variable to the path of the repo.")
  endif()
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

# if PIO_DP_PLUS_PIN = -1, then use the default PIO USB DP pin from the board config.
if (  PIO_DP_PLUS_PIN STREQUAL "-1" )
    message(STATUS "Using default PIO USB DP pin")
    target_compile_definitions(${PROJECT} PRIVATE
    CFG_TUH_RPI_PIO_USB=${ENABLE_PIO_USB} # Enable PIO USB support
    HSTX=${HSTX}
    )
else()
    message(STATUS "Using PIO USB DP pin: ${PIO_DP_PLUS_PIN}")
    target_compile_definitions(${PROJECT} PRIVATE
      CFG_TUH_RPI_PIO_USB=${ENABLE_PIO_USB} # Enable PIO USB support
      PICO_DEFAULT_PIO_USB_DP_PIN=${PIO_DP_PLUS_PIN} # Use the specified pin for PIO USB DP
      HSTX=${HSTX}
    )
endif()

family_configure_target(${PROJECT} "")

target_link_libraries(${PROJECT} PUBLIC pico_stdlib tinyusb_device tinyusb_host tinyusb_pico_pio_usb )
rp2040_family_configure_example_warnings(${PROJECT})
# due to warnings from Pico-PIO-USB
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
