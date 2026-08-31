cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================
# Pico VS Code extension
# ============================================================

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
# Board / SDK
# ============================================================

set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(mcu C CXX ASM)

pico_sdk_init()


# ============================================================
# micro-ROS static library
# ============================================================

link_directories(
    ${CMAKE_CURRENT_LIST_DIR}/libmicroros
)


# ============================================================
# Executable
# ============================================================

add_executable(
    mcu

    mcu.cpp

    # micro-ROS serial transport implementation
    pico_uart_transport.c
)

pico_set_program_name(mcu "mcu")
pico_set_program_version(mcu "0.1")

target_compile_definitions(
    mcu PRIVATE

    PICO_DEFAULT_UART_BAUD_RATE=921600
)

# ============================================================
# micro-ROS headers
# ============================================================

target_include_directories(
    mcu PRIVATE

    ${CMAKE_CURRENT_LIST_DIR}

    ${CMAKE_CURRENT_LIST_DIR}/libmicroros/include
)


# ============================================================
# Libraries
# ============================================================

target_link_libraries(
    mcu

    pico_stdlib
    pico_multicore

    hardware_pwm
    hardware_adc
    hardware_dma
    hardware_uart

    # micro-ROS precompiled static library
    microros
)


# ============================================================
# micro-ROS transport
# ============================================================
#
# UART0 : upper controller / micro-ROS
# UART1 : Power Pico, initialized directly by mcu.cpp
#
# USB stdio is disabled so it does not interfere with the
# micro-ROS serial transport.
# ============================================================

pico_enable_stdio_uart(mcu 1)
pico_enable_stdio_usb(mcu 0)


# micro-ROS serial transport expects raw serial data.
add_compile_definitions(
    PICO_UART_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_DEFAULT_CRLF=0
)


# ============================================================
# Outputs
# ============================================================

pico_add_extra_outputs(mcu)