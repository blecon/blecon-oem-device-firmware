# Copyright (c) 2025 Blecon Ltd
# SPDX-License-Identifier: Apache-2.0

# The application uses the configuration/<board> scheme for configuration files.
set(SB_APPLICATION_CONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}/../configuration/\${NORMALIZED_BOARD_TARGET}")

# The main application configuration directory is defined by the main application's
# `CMakeLists.txt` file (this allows to avoid dependency on main application name).
# Redirect other images' configuration directories to redefine configuration.
set(mcuboot_APPLICATION_CONFIG_DIR
    "${CMAKE_CURRENT_LIST_DIR}/../configuration/\${NORMALIZED_BOARD_TARGET}/images/mcuboot"
    CACHE INTERNAL "Application configuration dir controlled by sysbuild"
)

find_package(Sysbuild REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(sysbuild LANGUAGES)
