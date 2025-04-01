cmake_minimum_required(VERSION 3.16)

set(PICO_BOARD pico_w)

include(../../pico-sdk/pico_sdk_init.cmake)

project(aux_native)
pico_sdk_init()

include_directories(aux_native include)

add_executable(aux_native 
    src/main.cpp
    src/encoder.c
    src/pid_control.c
    src/util.c
    src/pi_motor.c
    src/accel_stepper.cpp
    src/music.c)
target_compile_options(aux_native PRIVATE -Wall -Wextra)

pico_generate_pio_header(aux_native 
    ${CMAKE_CURRENT_LIST_DIR}/src/encoder.pio)



# Link with the pico stdlib
target_link_libraries(aux_native
    pico_stdlib 
    pico_sync
    pico_i2c_slave
    pico_multicore
    hardware_gpio
    hardware_adc
    hardware_pio
    hardware_i2c
    hardware_pwm
    hardware_watchdog)
if(PICO_BOARD STREQUAL pico_w)
    target_link_libraries(aux_native pico_cyw43_arch_none)
    target_compile_definitions(aux_native PRIVATE PICO_W=1)
    message("PICO W enabled")
endif()

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(aux_native)

# Select USB, UART, or SWD Semihosting for stdio
# NOTE: Only one can be enabled
# Use usb if not using pico probe
pico_enable_stdio_usb(aux_native 0)
# Use uart for fast stdio over pico (requires extra wires)
pico_enable_stdio_uart(aux_native 1)
# Use semihosting for slower stdio over pico (Only swd connector needed)
pico_enable_stdio_semihosting(aux_native 0)