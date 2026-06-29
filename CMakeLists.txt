# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../zephyr-dw3000-examples/dw3000-decadriver/
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mmp-gdb)

zephyr_library_compile_options(-O1)

# Pull ROLE from sysbuild context into this image cmake context
zephyr_get(ROLE SYSBUILD GLOBAL)

#if(ROLE STREQUAL "ANCHOR_A1")
#    add_definitions(-DROLE_ANCHOR_A1)
#    message(STATUS "MMP-GDB role: ANCHOR_A1")
#elseif(ROLE STREQUAL "ANCHOR_A2")
#    add_definitions(-DROLE_ANCHOR_A2)
#    message(STATUS "MMP-GDB role: ANCHOR_A2")
#elseif(ROLE STREQUAL "ANCHOR_A3")
#    add_definitions(-DROLE_ANCHOR_A3)
#    message(STATUS "MMP-GDB role: ANCHOR_A3")
#else()
#    message(FATAL_ERROR "No role defined. Add CMake arg: -Dmmp-gdb_ROLE=ANCHOR_A1 (or A2/A3)")
#endif()

target_sources(app PRIVATE
    src/main.c
    src/ring_exchange.c
    ../zephyr-dw3000-examples/platform/port.c
    ../zephyr-dw3000-examples/platform/config_options.c
    ../zephyr-dw3000-examples/examples/shared_data/shared_functions.c
)

target_include_directories(app PRIVATE
    src
    ../zephyr-dw3000-examples/platform
    ../zephyr-dw3000-examples/examples/shared_data
)
