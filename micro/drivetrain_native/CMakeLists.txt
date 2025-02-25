cmake_minimum_required(VERSION 3.16)

set(PICO_BOARD pico)

include(../../pico-sdk/pico_sdk_init.cmake)

project(drivetrain_native)
pico_sdk_init()

include_directories(drivetrain_native include)

add_executable(drivetrain_native 
    src/main.c
    src/i2c_slave.c
    src/encoder.c
    src/pid_control.c
    src/util.c
    src/pi_motor.c
    src/madgwick_filter.c
    src/bmi2.c
    src/bmi270.c
    src/imu.c)

pico_generate_pio_header(drivetrain_native 
    ${CMAKE_CURRENT_LIST_DIR}/src/encoder.pio)



# Link with the pico stdlib
target_link_libraries(drivetrain_native
    pico_stdlib 
    pico_sync
    hardware_adc
    hardware_pio
    hardware_i2c
    hardware_pwm)
if(PICO_BOARD STREQUAL pico_w)
    target_link_libraries(drivetrain_native pico_cyw43_arch_none)
    target_compile_definitions(drivetrain_native PRIVATE PICO_W=1)
    message("PICO W enabled")
endif()

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(drivetrain_native)

# Enable USB serial
pico_enable_stdio_usb(drivetrain_native 1)
pico_enable_stdio_uart(drivetrain_native 0)