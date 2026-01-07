cmake_minimum_required(VERSION 3.20)

# FORCE CMake to skip the full executable link test
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

# Set the toolchain before project()
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")
set(CMAKE_OBJCOPY "arm-none-eabi-objcopy")

project(STM32BareMetal C)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

add_compile_options(
    -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion -Wformat-truncation -fno-common -Wconversion
    -g -Os -ffunction-sections -fdata-sections -I.
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
)
add_link_options(
    --specs nano.specs
    -lc
    -lgcc
    -Wl,--gc-sections
    -Wl,-Map=$@.map
    -nostdlib
    -nodefaultlibs
    -nostartfiles
)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${CMAKE_CURRENT_SOURCE_DIR}/link.ld")

add_subdirectory(first)
add_subdirectory(second)