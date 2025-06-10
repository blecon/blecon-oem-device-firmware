/*
 * Copyright (c) Blecon Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <zephyr/kernel.h>

enum blecon_led_connection_state_t {
    blecon_led_connection_state_disconnected,
    blecon_led_connection_state_connecting,
    blecon_led_connection_state_connected,
};

/// @brief Indicate data activity on the Blecon LED
void blecon_led_data_activity();

/// @brief Set Blecon LED connnection state
/// @param state
void blecon_led_set_connection_state(enum blecon_led_connection_state_t state);

/// @brief Indicate that Blecon is announcing device ID
void blecon_led_set_announce(bool state);
