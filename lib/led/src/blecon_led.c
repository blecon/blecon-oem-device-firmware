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

static struct k_thread _blecon_led_thread_data;
static const struct device* _led_device;
static uint32_t _led_num;
static bool _is_announcing = false;
static bool _data_activity = false;
static bool _has_heartbeat = false;
static enum blecon_led_connection_state_t _connection_state = blecon_led_connection_state_disconnected;

static void blecon_led_thread_entry(void *, void *, void *);

K_MUTEX_DEFINE(blecon_led_mutex);
K_CONDVAR_DEFINE(blecon_led_condvar);
K_THREAD_STACK_DEFINE(blecon_led_thread_area, BLECON_LED_THREAD_STACK_SIZE);

void blecon_led_init(const struct device* led_device, uint32_t led_num) {
    _led_device = led_device;
    _led_num = led_num;
    _connection_state = blecon_led_connection_state_disconnected;

    k_thread_create(&_blecon_led_thread_data, blecon_led_thread_area,
        K_THREAD_STACK_SIZEOF(blecon_led_thread_area),
        blecon_led_thread_entry,
        NULL, NULL, NULL,
        BLECON_LED_THREAD_PRIORITY, 0, K_NO_WAIT);
}

static void blecon_led_thread_entry(void* unused1, void* unused2, void* unused3) {
    k_timeout_t next_frame_time = K_NO_WAIT;
    uint32_t delay = 0;
    bool blink = false;
    size_t pattern_frame = 0;

    while(true) {
        k_mutex_lock(&blecon_led_mutex, K_FOREVER);
        k_condvar_wait(&blecon_led_condvar, &blecon_led_mutex, next_frame_time);

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
                        led_off(_led_device, _led_num);
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

        if(blink) {
            blink = false;
        }
        else {
            if(pattern_frame % 2 == 0) {
                led_on(_led_device, _led_num);
            }
            else {
                led_off(_led_device, _led_num);
            }
            pattern_frame += 1;
        }

        // Wait until next frame for pattern (or state change)
        if(delay == 0) {
            next_frame_time = K_FOREVER;
        }
        else {
            next_frame_time = K_MSEC(delay);
        }
    }
}

void blecon_led_data_activity() {
    k_mutex_lock(&blecon_led_mutex, K_FOREVER);
    _data_activity = true;
    k_condvar_signal(&blecon_led_condvar);
    k_mutex_unlock(&blecon_led_mutex);
}

void blecon_led_set_connection_state(enum blecon_led_connection_state_t conn_state) {
    k_mutex_lock(&blecon_led_mutex, K_FOREVER);
    _connection_state = conn_state;
    k_condvar_signal(&blecon_led_condvar);
    k_mutex_unlock(&blecon_led_mutex);
}

void blecon_led_set_announce(bool state) {
    k_mutex_lock(&blecon_led_mutex, K_FOREVER);
    _is_announcing = state;
    k_condvar_signal(&blecon_led_condvar);
    k_mutex_unlock(&blecon_led_mutex);
}

void blecon_led_set_heartbeat(bool state) {
    k_mutex_lock(&blecon_led_mutex, K_FOREVER);
    _has_heartbeat = state;
    k_condvar_signal(&blecon_led_condvar);
    k_mutex_unlock(&blecon_led_mutex);
}
