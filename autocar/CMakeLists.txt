cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(autocar C CXX ASM)
pico_sdk_init()

add_executable(autocar)

target_sources(autocar PRIVATE
    src/main.c
    src/motor_control.c
    src/ultrasonic_task.c
    src/vTaskTest.c
    src/encoder_task.c    
)

target_include_directories(autocar PRIVATE
    include 
)


target_link_libraries(autocar PRIVATE
    pico_stdlib
    hardware_i2c         
    hardware_pwm         
    hardware_gpio        
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_uart(autocar 1)
pico_enable_stdio_usb(autocar 1)
pico_add_extra_outputs(autocar)