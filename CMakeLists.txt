cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(hello-pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# ----- main ------------------------------------------------------------------

add_executable(main
  main.c
)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)
target_link_libraries(main pico_stdlib hardware_i2c)

# ----- hello-uart ------------------------------------------------------------

add_executable(hello-uart
  hello-uart.cpp
)

pico_enable_stdio_usb(hello-uart 1)
pico_enable_stdio_uart(hello-uart 1)
target_link_libraries(hello-uart pico_stdlib hardware_i2c)
