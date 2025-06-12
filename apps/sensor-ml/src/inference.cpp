// Copyright (c) Blecon Ltd
// SPDX-License-Identifier: Apache-2.0

#include "inference.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <string.h>
#include "motion-recognition/edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "motion-recognition/model-parameters/model_variables.h"

static inference_callback_t _inference_callback = nullptr;

#define ACCEL_FRAME_SIZE EI_CLASSIFIER_RAW_SAMPLE_COUNT
static float _accel_buffer[ACCEL_FRAME_SIZE][3];
static size_t _accel_index = 0;

// Use a periodic timer instead of delayable work
static struct k_timer _inference_timer;
static struct k_work _inference_work;

static void inference_work_handler(struct k_work *work);
static void inference_timer_handler(struct k_timer *timer);

void inference_work_handler(struct k_work *work)
{
    const struct device *accel = DEVICE_DT_GET(DT_CHOSEN(blecon_accelerometer));
    if (!device_is_ready(accel)) {
        // Could not find accelerometer
        return;
    }
    struct sensor_value accel_xyz[3];

    if (_accel_index < ACCEL_FRAME_SIZE) {
        if (sensor_sample_fetch(accel) == 0 &&
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_XYZ, accel_xyz) == 0) {
            _accel_buffer[_accel_index][0] = sensor_value_to_double(&accel_xyz[0]);
            _accel_buffer[_accel_index][1] = sensor_value_to_double(&accel_xyz[1]);
            _accel_buffer[_accel_index][2] = sensor_value_to_double(&accel_xyz[2]);
            _accel_index++;
        }
    }
    if (_accel_index >= ACCEL_FRAME_SIZE) {
        signal_t signal;
        signal.total_length = ACCEL_FRAME_SIZE * 3;
        signal.get_data = [](size_t offset, size_t length, float *out_ptr) -> int {
            for (size_t i = 0; i < length; i++) {
                size_t idx = (offset + i) / 3;
                size_t axis = (offset + i) % 3;
                out_ptr[i] = _accel_buffer[idx][axis];
            }
            return 0;
        };
        ei_impulse_result_t result = {0};
        EI_IMPULSE_ERROR res = run_classifier(&ei_default_impulse, &signal, &result, false);
        if (res == EI_IMPULSE_OK && _inference_callback) {
            float max_score = 0.0f;
            int max_idx = 0;
            for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                if (result.classification[i].value > max_score) {
                    max_score = result.classification[i].value;
                    max_idx = i;
                }
            }
            _inference_callback((inference_category_t)max_idx, max_score);
        }
        _accel_index = 0;
    }
}

void inference_timer_handler(struct k_timer *timer)
{
    // This function is called periodically to trigger inference
    k_work_submit(&_inference_work);
}

void inference_init(inference_callback_t callback) {
    _inference_callback = callback;
    k_timer_init(&_inference_timer, inference_timer_handler, NULL);
    k_work_init(&_inference_work, inference_work_handler);
    
    // Start the timer with a 10ms period
    k_timer_start(&_inference_timer, K_NO_WAIT, K_MSEC(EI_CLASSIFIER_INTERVAL_MS));
}
