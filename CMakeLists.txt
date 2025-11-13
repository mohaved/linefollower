# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(line_follower_robot C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(line_follower_robot
    main.c
    line_follower.c
)

target_link_libraries(line_follower_robot
    pico_stdlib
    pico_multicore
    pico_time
    hardware_pwm
    hardware_adc
    hardware_gpio
    hardware_irq
)
pico_enable_stdio_usb(line_follower_robot 1)
pico_enable_stdio_uart(line_follower_robot 0)

pico_add_extra_outputs(line_follower_robot)