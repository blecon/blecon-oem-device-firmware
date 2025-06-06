# Copyright (c) Blecon Ltd
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_BLECON_LIB_BATTERY battery)
add_subdirectory_ifdef(CONFIG_BLECON_LIB_OTA ota)
add_subdirectory_ifdef(CONFIG_BLECON_LIB_MOTION motion)
add_subdirectory_ifdef(CONFIG_BLECON_LIB_LED led)
