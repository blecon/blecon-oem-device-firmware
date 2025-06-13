// Copyright (c) Blecon Ltd
// SPDX-License-Identifier: Apache-2.0

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "motion/motion.h"

#define LIS2DH_HP_IA1   1 << 0  // Use high-pass filtered data for INT1
#define LIS2DH_HP_IA2   1 << 1  // Use high-pass filtered data for INT2

LOG_MODULE_REGISTER(motion);

enum blecon_motion_state_t {
    blecon_motion_state_uninitialized = 0,
    blecon_motion_state_waiting_for_motion,
    blecon_motion_state_in_motion,
};

#define MOTION_TIMEOUT_MS 200

struct blecon_motion_t {
    const struct device *dev;
    const struct blecon_motion_event_callbacks_t* callbacks;
    enum blecon_motion_state_t state;
    int64_t last_motion_time;
    struct k_work_delayable stop_work_item;
};

static struct blecon_motion_t _motion = {0};
static struct sensor_trigger _motion_trig;

static int blecon_motion_set_sampling_freq(const struct device *dev, int32_t sampling_freq);
static void blecon_motion_on_stop(struct k_work *);
static void motion_trigger_handler(const struct device *dev, const struct sensor_trigger *trig);

int init_motion(float accel_threshold, const struct blecon_motion_event_callbacks_t* callbacks) {
	int ret;

    const struct device *accel = DEVICE_DT_GET_ANY(st_lis2dh);
	if (!device_is_ready(accel)) {
		LOG_ERR("Device %s is not ready.", accel->name);
		return 0;
	}

    _motion.dev = accel;
    k_work_init_delayable(&_motion.stop_work_item, blecon_motion_on_stop);

    _motion_trig.type = SENSOR_TRIG_DELTA;
    _motion_trig.chan = SENSOR_CHAN_ACCEL_XYZ;

    blecon_motion_set_sampling_freq(accel, 50);

    // Set threshold that acceleration needs to exceed to register as motion
    // This value is in m/s^2
    struct sensor_value slope_th;
    ret = sensor_value_from_float(&slope_th, accel_threshold);
    if (ret != 0) {
        LOG_ERR("Invalid motion threshold %f", (double) accel_threshold);
        return ret;
    }

    // Use high pass filtered data for interrupt generation
    // (subtract out gravity when sensing motion)
    struct sensor_value hp_filt = {
        .val1 = LIS2DH_HP_IA1 | LIS2DH_HP_IA2
    };

    // Set duration that acceleration must be over the threshold to register as motion
    // This is in units of 1/ODR seconds.
    struct sensor_value slope_dur = {
        .val1 = 1,
        .val2 = 0,
    };

    ret = sensor_attr_set(accel, _motion_trig.chan,
                        SENSOR_ATTR_CONFIGURATION,
                        &hp_filt);
    if (ret != 0) {
        LOG_ERR("Failed to enable HP filter: %d", ret);
        return ret;
    }

    ret = sensor_attr_set(accel, _motion_trig.chan,
                        SENSOR_ATTR_SLOPE_TH,
                        &slope_th);
    if (ret != 0) {
        LOG_ERR("Failed to set motion slope threshold: %d", ret);
        return ret;
    }

    ret = sensor_attr_set(accel, _motion_trig.chan,
                        SENSOR_ATTR_SLOPE_DUR,
                        &slope_dur);
    if (ret != 0) {
        LOG_ERR("Failed to set motion duration: %d", ret);
        return ret;
    }

    ret = sensor_trigger_set(accel, &_motion_trig, motion_trigger_handler);
    if (ret != 0) {
        LOG_ERR("Failed to set motion trigger: %d", ret);
        return ret;
    }

    _motion.state = blecon_motion_state_waiting_for_motion;
    _motion.callbacks = callbacks;

    return 0;
}

static int sample_accel(const struct device *dev, float *out_x, float *out_y, float *out_z) {
    int rc;

    struct sensor_value accel[3];
    rc = sensor_sample_fetch(dev);
    if (rc < 0) {
        LOG_ERR("could not fetch data");
        return rc;
    }

    rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    if (rc < 0) {
        LOG_ERR("could not get data");
        return rc;
    }

    *out_x = sensor_value_to_float(&accel[0]);
    *out_y = sensor_value_to_float(&accel[1]);
    *out_z = sensor_value_to_float(&accel[2]);

    return 0;
}

static void motion_trigger_handler(const struct device *dev,
			                const struct sensor_trigger *trig) {

    if( _motion.state == blecon_motion_state_waiting_for_motion ) {
        _motion.callbacks->start();
    }

    _motion.state = blecon_motion_state_in_motion;
    _motion.last_motion_time = k_uptime_get();
    k_work_schedule(&_motion.stop_work_item, K_MSEC(MOTION_TIMEOUT_MS));
}

void blecon_motion_on_stop(struct k_work *item) {
    int ret;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;

    struct k_work_delayable *delay_item = k_work_delayable_from_work(item);
    struct blecon_motion_t *motion = CONTAINER_OF(delay_item, struct blecon_motion_t, stop_work_item);


    if(k_uptime_get() < motion->last_motion_time + MOTION_TIMEOUT_MS) {
        int64_t remain = (motion->last_motion_time + MOTION_TIMEOUT_MS) - k_uptime_get();
        k_work_schedule(&motion->stop_work_item, K_MSEC(remain));
        return;
    }

    ret = sample_accel(motion->dev, &x, &y, &z);

    _motion.state = blecon_motion_state_waiting_for_motion;
    _motion.callbacks->stop();
    _motion.callbacks->vector(x, y, z);
}

int blecon_motion_set_sampling_freq(const struct device *dev, int32_t sampling_freq) {
    int ret;

    if (IS_ENABLED(CONFIG_LIS2DH_ODR_RUNTIME)) {
        struct sensor_value odr = {
            .val1 = sampling_freq,
        };

        ret = sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY,
                        &odr);
        if (ret != 0) {
            LOG_ERR("Failed to set odr: %d", ret);
            return 0;
        }
        LOG_DBG("Sampling at %u Hz\n", odr.val1);
    }

    return 0;
}
