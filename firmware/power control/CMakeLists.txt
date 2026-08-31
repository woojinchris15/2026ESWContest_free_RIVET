cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================
# Pico SDK
# ============================================================

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)

set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)

if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()

# ============================================================
# Board
# ============================================================

set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK
include(pico_sdk_import.cmake)

project(pcu C CXX ASM)

# Initialise Pico SDK
pico_sdk_init()

# ============================================================
# micro-ROS
# ============================================================


link_directories(
    ${CMAKE_CURRENT_LIST_DIR}/libmicroros
)

# ============================================================
# Executable
# ============================================================

add_executable(pcu
    pcu.cpp
    pico_uart_transport.c
)

pico_set_program_name(pcu "pcu")
pico_set_program_version(pcu "0.1")

# ============================================================
# STDIO
# ============================================================

pico_enable_stdio_uart(pcu 1)
pico_enable_stdio_usb(pcu 0)

# ============================================================
# Libraries
# ============================================================

target_link_libraries(pcu
    pico_stdlib

    # RP2350 multicore
    pico_multicore

    # Hardware
    hardware_adc
    hardware_pwm
    hardware_clocks
    hardware_i2c
    hardware_uart
    hardware_watchdog
    hardware_sync

    # micro-ROS static library
    microros
)

# ============================================================
# Include directories
# ============================================================

target_include_directories(pcu PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/libmicroros/include
)

# ============================================================
# micro-ROS / linker optimisation
# ============================================================

target_compile_options(pcu PRIVATE
    -ffunction-sections
    -fdata-sections
)

target_link_options(pcu PRIVATE
    -Wl,--gc-sections
)

# ============================================================
# CR/LF handling
# ============================================================

target_compile_definitions(pcu PRIVATE
    PICO_UART_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_DEFAULT_CRLF=0
)

# ============================================================
# Generate .uf2 / .bin / .hex
# ============================================================

pico_add_extra_outputs(pcu)