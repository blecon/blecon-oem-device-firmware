/*
 * Copyright (c) Blecon Ltd
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/drivers/led.h>

#include "led/blecon_led.h"

#define BLECON_LED_THREAD_STACK_SIZE 256
#define BLECON_LED_THREAD_PRIORITY 5

#define BLINK_DELAY_MS 30
#define ANNOUNCE_DURATION_MS 5000

// LED blink patterns
static const uint32_t heartbeat_pattern[] = {30, 2000};
static const uint32_t announce_pattern[] = {100, 100};
static const uint32_t connecting_pattern[] = {30, 400};
static const uint32_t connected_pattern[] = {0};

// TODO replace with struct led_dt_spec once Zephyr is updated
#define BLECON_LED_STATUS_NODE DT_CHOSEN(blecon_led_status)
const static struct device *_led_device = DEVICE_DT_GET(DT_PARENT(BLECON_LED_STATUS_NODE));
const static uint32_t _led_idx = DT_NODE_CHILD_IDX(BLECON_LED_STATUS_NODE);

static bool _is_enabled = false;
static bool _is_announcing = false;
static bool _data_activity = false;
static bool _has_heartbeat = false;
static enum blecon_led_connection_state_t _connection_state = blecon_led_connection_state_disconnected;

static void blecon_led_thread_entry(void *, void *, void *);

K_MUTEX_DEFINE(blecon_led_mutex);
K_CONDVAR_DEFINE(blecon_led_condvar);

K_THREAD_DEFINE(blecon_led_thread_id, BLECON_LED_THREAD_STACK_SIZE,
                blecon_led_thread_entry, NULL, NULL, NULL,
                BLECON_LED_THREAD_PRIORITY, 0, 0);

static void blecon_led_thread_entry(void* unused1, void* unused2, void* unused3) {
    k_timeout_t next_frame_time = K_NO_WAIT;
    uint32_t delay = 0;
    bool blink = false;
    bool disabled = false;
    size_t pattern_frame = 0;

    while(true) {
        k_mutex_lock(&blecon_led_mutex, K_FOREVER);
        k_condvar_wait(&blecon_led_condvar, &blecon_led_mutex, next_frame_time);

        disabled = !_is_enabled;

        if(_is_announcing) {
            pattern_frame = pattern_frame % ARRAY_SIZE(announce_pattern);
            delay = announce_pattern[pattern_frame];
        }
        else {
            switch(_connection_state) {
                case blecon_led_connection_state_disconnected:
                    if (_has_heartbeat) {
                        pattern_frame = pattern_frame % ARRAY_SIZE(heartbeat_pattern);
                        delay = heartbeat_pattern[pattern_frame];
                    }
                    else {
                        pattern_frame = 1;
                        delay = 0;
                    }
                    break;
                case blecon_led_connection_state_connecting:
                    pattern_frame = pattern_frame % ARRAY_SIZE(connecting_pattern);
                    delay = connecting_pattern[pattern_frame];
                    break;
                case blecon_led_connection_state_connected:
                    if(_data_activity) {
                        led_off(_led_device, _led_idx);
                        _data_activity = false;
                        blink = true;
                        delay = BLINK_DELAY_MS;
                    }
                    else {
                        pattern_frame = pattern_frame % ARRAY_SIZE(connected_pattern);
                        delay = connected_pattern[pattern_frame];
                    }
                    break;
            }
        }
        k_mutex_unlock(&blecon_led_mutex);

        if(disabled) {
            led_off(_led_device, _led_idx);
            pattern_frame = 0; // Reset pattern frame when disabled
        }
        else if(blink) {
            blink = false;
        }
        else {
            if(pattern_frame % 2 == 0) {
                led_on(_led_device, _led_idx);
            }
            else {
                led_off(_led_device, _led_idx);
            }
            pattern_frame += 1;
        }

        // Wait until next frame for pattern (or state change)
        if((delay == 0) || disabled) {
            next_frame_time = K_FOREVER;
        }
        else {
            next_frame_time = K_MSEC(delay);
        }
    }
}

void blecon_led_enable(bool enable) {
    k_mutex_lock(&blecon_led_mutex, K_FOREVER);
    _is_enabled = enable;
    k_condvar_signal(&blecon_led_condvar);
    k_mutex_unlock(&blecon_led_mutex);
}

void blecon_led_data_activity() {
    k_mutex_lock(&blecon_led_mutex, K_FOREVER);
    _data_activity = true;
    if(_is_enabled) {
        k_condvar_signal(&blecon_led_condvar);
    }
    k_mutex_unlock(&blecon_led_mutex);
}

void blecon_led_set_connection_state(enum blecon_led_connection_state_t conn_state) {
    k_mutex_lock(&blecon_led_mutex, K_FOREVER);
    _connection_state = conn_state;
    if(_is_enabled) {
        k_condvar_signal(&blecon_led_condvar);
    }
    k_mutex_unlock(&blecon_led_mutex);
}

void blecon_led_set_announce(bool state) {
    k_mutex_lock(&blecon_led_mutex, K_FOREVER);
    _is_announcing = state;
    if(_is_enabled) {
        k_condvar_signal(&blecon_led_condvar);
    }
    k_mutex_unlock(&blecon_led_mutex);
}

void blecon_led_set_heartbeat(bool state) {
    k_mutex_lock(&blecon_led_mutex, K_FOREVER);
    _has_heartbeat = state;
    if(_is_enabled) {
        k_condvar_signal(&blecon_led_condvar);
    }
    k_mutex_unlock(&blecon_led_mutex);
}
