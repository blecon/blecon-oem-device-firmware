// Copyright (c) Blecon Ltd
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <zephyr/kernel.h>

/**
 * Convenience function to blink the blecon,power-led LED to indicate
 * when the device has powered on or is powering off.
 */
void power_flash_led(uint32_t blinks);

/**
 * Call on system power-up. If it is a power-on or pin-triggered reset,
 * immediately put the system into off mode. Otherwise, continue normal
 * startup.
 */
void power_sys_start();

/**
 * Put the system into off mode and use the GPIO set in the
 * blecon,sw-power property in the Devicetree's /chosen node
 * as the wake-up source.
 */
void power_off();
