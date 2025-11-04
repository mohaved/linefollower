cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(line_follower_robot C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(line_follower_robot
    robot/main.c
    robot/line_follower.c
)

target_link_libraries(line_follower_robot
    pico_stdlib
    pico_multicore
    hardware_pwm
    hardware_i2c
    hardware_adc
    hardware_irq
)

pico_enable_stdio_usb(line_follower_robot 1)
pico_enable_stdio_uart(line_follower_robot 0)

pico_add_extra_outputs(line_follower_robot)