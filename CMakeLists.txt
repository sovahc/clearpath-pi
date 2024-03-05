cmake_minimum_required(VERSION 3.12)

if(NOT DEFINED ENV{PICO_SDK_PATH})
        # try define my own PICO_SDK_PATH 
        message(ERROR "PICO_SDK_PATH environment variable should be defined")
        set(ENV{PICO_SDK_PATH} /home/cat/pico/pico-sdk)        
endif()

include("$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")

project(clearpath-pi)

pico_sdk_init()

add_executable(${PROJECT_NAME} main.cpp)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${PROJECT_NAME} PUBLIC pico_stdlib tinyusb_host tinyusb_board pico_multicore lcd_display)

add_subdirectory(pi-pico-LCD/lcd_display)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

#pico_enable_stdio_usb(${PROJECT_NAME} 0)
#pico_enable_stdio_uart(${PROJECT_NAME} 1)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )
