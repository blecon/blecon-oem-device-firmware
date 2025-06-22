# Copyright (c) Blecon Ltd
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(CMAKE_C_STANDARD 11)
set_property(GLOBAL PROPERTY CSTD c11)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The application uses the configuration/<board> scheme for configuration files.
set(APPLICATION_CONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}/configuration/\${NORMALIZED_BOARD_TARGET}")
set(EXTRA_CONF_FILE "${APPLICATION_CONFIG_DIR}/blecon.conf")

# include top-level application defines
include(../../cmake/app.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(sensor-default)

# Edge Impulse definitions
add_definitions(-DEI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN=1
                -DARM_MATH_LOOPUNROLL
                )

target_sources(app PRIVATE src/main.c src/inference.cpp)

zephyr_include_directories(memfault_config)

# Include the motion recognition inference code
add_subdirectory(src/motion-recognition)
