# =============================================================================
# pico2_slcan — SLCAN USB-CAN bridge for Waveshare RP2350-CAN.
#
# Build:
#   cd /home/pi/ros2/firmware/pico2_slcan
#   PICO_SDK_PATH=/opt/pico-sdk cmake -B build -DPICO_BOARD=pico2 .
#   cmake --build build --parallel 4
#
# Flash:
#   Hold BOOTSEL, power-cycle, copy build/pico2_slcan.uf2 to the RPI-RP2 drive.
#
# After use: re-flash odrive_can_gateway.uf2 to restore ODrive bridge function.
# =============================================================================
cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH /opt/pico-sdk)
include(pico_sdk_import.cmake)

project(pico2_slcan C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(pico2_slcan main.c)

target_link_libraries(pico2_slcan
    pico_stdlib
    hardware_spi
)

# Use the Pico SDK's built-in USB CDC via stdio — simpler than raw TinyUSB.
# This gives us a /dev/ttyACMx on the host that works with slcand/python-can.
pico_enable_stdio_usb(pico2_slcan 1)
pico_enable_stdio_uart(pico2_slcan 0)

# Generate .uf2, .bin, .hex etc.
pico_add_extra_outputs(pico2_slcan)
