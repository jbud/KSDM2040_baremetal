# KSDM_2040_BM Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "C:/PICO/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(KSDM_2040_BM C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(KSDM_2040_BM KSDM_2040_BM.c )

pico_set_program_name(KSDM_2040_BM "KSDM_2040_BM")
pico_set_program_version(KSDM_2040_BM "0.1")

pico_enable_stdio_uart(KSDM_2040_BM 0)
pico_enable_stdio_usb(KSDM_2040_BM 1)

# Add the standard library to the build
target_link_libraries(KSDM_2040_BM pico_stdlib)

pico_add_extra_outputs(KSDM_2040_BM)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )