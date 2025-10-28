// Copyright (c) Blecon Ltd
// SPDX-License-Identifier: Apache-2.0

#ifndef APPLICATION_MOTION_H_
#define APPLICATION_MOTION_H_

#include <stdint.h>
#include <zephyr/drivers/sensor.h>

typedef void (*blecon_motion_start_event_callback_t)(void);
typedef void (*blecon_motion_stop_event_callback_t)(void);
typedef void (*blecon_motion_orientation_event_callback_t)(void);

struct blecon_motion_event_callbacks_t {
    void (*start)(void);
    void (*stop)(void);
    void (*vector)(float x, float y, float z);
};

int init_motion(const struct device *accel, float accel_threshold, const struct blecon_motion_event_callbacks_t* callbacks);

#endif
