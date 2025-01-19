cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(hello-pico)
pico_sdk_init()

# ----- main ------------------------------------------------------------------

add_executable(main
  main.c
)

pico_enable_stdio_usb(main 1)
target_link_libraries(main pico_stdlib)

# ----- hello-uart ------------------------------------------------------------

add_executable(hello-uart
  hello-uart.cpp
)

pico_enable_stdio_usb(hello-uart 1)
# Watch out here!  We need to make sure that the STDOUT doesn't 
# get sent on the UART as well.
#pico_enable_stdio_uart(hello-uart 0)
target_link_libraries(hello-uart pico_stdlib hardware_i2c)

# ----- hello-audio -----------------------------------------------------------

add_executable(hello-audio
  hello-audio.cpp
)

pico_enable_stdio_usb(hello-audio 1)
# Watch out here!  We need to make sure that the STDOUT doesn't 
# get sent on the UART as well.
pico_enable_stdio_uart(hello-audio 0)
target_link_libraries(hello-audio pico_stdlib hardware_i2c hardware_pwm)

# ----- hello-audio-2 ---------------------------------------------------------

add_executable(hello-audio-2
  hello-audio-2.cpp
)

pico_enable_stdio_usb(hello-audio-2 1)
# Watch out here!  We need to make sure that the STDOUT doesn't 
# get sent on the UART as well.
pico_enable_stdio_uart(hello-audio-2 0)
target_link_libraries(hello-audio-2 pico_stdlib hardware_i2c)

# ----- hello-w5500 -----------------------------------------------------------

add_executable(hello-w5500
  hello-w5500.cpp
)

pico_enable_stdio_usb(hello-w5500 1)
# Watch out here!  We need to make sure that the STDOUT doesn't 
# get sent on the UART as well.
pico_enable_stdio_uart(hello-w5500 0)
target_link_libraries(hello-w5500 pico_stdlib hardware_spi)

# ----- hello-w5500-2 ---------------------------------------------------------

add_executable(hello-w5500-2
  hello-w5500-2.cpp
)

pico_enable_stdio_usb(hello-w5500-2 1)
# Watch out here!  We need to make sure that the STDOUT doesn't 
# get sent on the UART as well.
pico_enable_stdio_uart(hello-w5500-2 0)
target_compile_definitions(hello-w5500-2 PRIVATE PARAM_ASSERTIONS_ENABLE_ALL=1)
target_compile_definitions(hello-w5500-2 PRIVATE PICO_BUILD=1)

target_link_libraries(hello-w5500-2 pico_stdlib hardware_spi)

# ----- hello-async -----------------------------------------------------------

add_executable(hello-async
  hello-async.cpp
)

pico_enable_stdio_usb(hello-async 1)
# Watch out here!  We need to make sure that the STDOUT doesn't 
# get sent on the UART as well.
pico_enable_stdio_uart(hello-async 0)
target_compile_definitions(hello-async PRIVATE PARAM_ASSERTIONS_ENABLE_ALL=1)
target_compile_definitions(hello-async PRIVATE PICO_BUILD=1)

target_link_libraries(hello-async pico_stdlib)

# ----- hello-wifi ------------------------------------------------------------

add_executable(hello-wifi
  hello-wifi.cpp
)
pico_enable_stdio_usb(hello-wifi 1)
pico_enable_stdio_uart(hello-wifi 0)
pico_add_extra_outputs(hello-wifi)
target_include_directories(hello-wifi PRIVATE ${CMAKE_CURRENT_LIST_DIR} )
target_link_libraries(hello-wifi pico_cyw43_arch_lwip_poll pico_stdlib)




