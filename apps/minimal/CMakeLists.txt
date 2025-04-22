# Copyright (c) Blecon Ltd
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# include top-level application defines
include(../../cmake/app.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

set(CMAKE_C_STANDARD 11)

project(minimal)
target_sources(app PRIVATE src/main.c)

zephyr_include_directories(config)
