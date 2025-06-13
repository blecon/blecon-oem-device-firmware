/*
 * Copyright (c) Blecon Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <zephyr/kernel.h>

#include "stdbool.h"

enum blecon_led_connection_state_t {
    blecon_led_connection_state_disconnected,
    blecon_led_connection_state_connecting,
    blecon_led_connection_state_connected,
};

/// @brief Enable or disable the Blecon LED
/// @param enable true to enable the LED, false to disable it
/// @note When disabled (default state), the LED will not blink or indicate any state
void blecon_led_enable(bool enable);

/// @brief Indicate data activity on the Blecon LED
void blecon_led_data_activity();

/// @brief Set Blecon LED connnection state
/// @param state
void blecon_led_set_connection_state(enum blecon_led_connection_state_t state);

/// @brief Indicate that Blecon is announcing device ID
void blecon_led_set_announce(bool state);

/// @brief Set whether LED displays a heartbeat pattern
void blecon_led_set_heartbeat(bool state);
