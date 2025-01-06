// Copyright (c) Blecon Ltd
// SPDX-License-Identifier: Apache-2.0

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "motion/motion.h"

LOG_MODULE_REGISTER(motion);

enum blecon_motion_state_t {
    blecon_motion_state_uninitialized = 0,
    blecon_motion_state_waiting_for_motion,
    blecon_motion_state_sampling_motion,
};

#define BLECON_MOTION_READINGS_MAX 25
struct blecon_motion_t {
    const struct blecon_motion_event_callbacks_t* callbacks;
    volatile bool transfer_pending;
    enum blecon_motion_state_t state;
    float readings[BLECON_MOTION_READINGS_MAX][3];
    size_t readings_count;
    size_t store_index;
};
static struct blecon_motion_t _motion = {0};

static struct sensor_trigger _data_trig;
static struct sensor_trigger _motion_trig;

static int blecon_motion_set_sampling_freq(const struct device *dev, int32_t sampling_freq);
static void blecon_motion_start_sampling(const struct device *dev);
static void blecon_motion_stop_sampling(const struct device *dev);
static void data_ready_handler(const struct device *dev, const struct sensor_trigger *trig);
static void motion_trigger_handler(const struct device *dev, const struct sensor_trigger *trig);

int init_motion(float accel_threshold, const struct blecon_motion_event_callbacks_t* callbacks) {
	int ret;
    
    const struct device *accel = DEVICE_DT_GET_ANY(st_lis2dh);
	if (!device_is_ready(accel)) {
		LOG_ERR("Device %s is not ready.", accel->name);
		return 0;
	}

    _data_trig.type = SENSOR_TRIG_DATA_READY;
    _data_trig.chan = SENSOR_CHAN_ACCEL_XYZ;

    _motion_trig.type = SENSOR_TRIG_DELTA;
    _motion_trig.chan = SENSOR_CHAN_ACCEL_XYZ;

    blecon_motion_set_sampling_freq(accel, 10);

    // Slope threshold in m/s^2
    struct sensor_value slope_th;    
    ret = sensor_value_from_float(&slope_th, accel_threshold);
    if (ret != 0) {
        LOG_ERR("Invalid motion threshold %f", (double) accel_threshold);
        return ret;
    }

    // Set high-pass filter on INT2 (motion interrupt)
    struct sensor_value hp_filt = {
        .val1 = 2,
    };

    // Slope duration is val1 * 1/ODR
    struct sensor_value slope_dur = {
        .val1 = 2,
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

static void data_ready_handler(const struct device *dev,
			            const struct sensor_trigger *trig) {

    if(_motion.state == blecon_motion_state_sampling_motion) {
        struct sensor_value accel[3];
    
        int rc = sensor_sample_fetch(dev);
        rc = sensor_channel_get(dev,
                        SENSOR_CHAN_ACCEL_XYZ,
                        accel);
        
        for(int dim = 0; dim < 3; dim++) {
            _motion.readings[_motion.store_index][dim] = sensor_value_to_float(&accel[dim]);
        }
        _motion.readings_count = MIN(_motion.readings_count + 1, BLECON_MOTION_READINGS_MAX);
        _motion.store_index = (_motion.store_index + 1) % BLECON_MOTION_READINGS_MAX;

        if (_motion.readings_count == BLECON_MOTION_READINGS_MAX){
            //Check if motion has stopped
            float accel_sum[3] = {0.0, 0.0, 0.0};
            float variance = 0.0;

            for(size_t p = 0; p < BLECON_MOTION_READINGS_MAX; p++) {
                for(int dim=0; dim < 3; dim++) {
                    accel_sum[dim] += _motion.readings[p][dim];
                }
            }

            for(size_t p = 0; p < BLECON_MOTION_READINGS_MAX; p++) {
                for(int dim=0; dim < 3; dim++) {
                    variance += (_motion.readings[p][dim] * BLECON_MOTION_READINGS_MAX - accel_sum[dim]) * 
                                (_motion.readings[p][dim] * BLECON_MOTION_READINGS_MAX - accel_sum[dim]);
                }
            }
            
            // If standard deviation of readings is <0.125g, consider motion stopped
            if(variance <= (0.125F * BLECON_MOTION_READINGS_MAX)
                           * (0.125F * BLECON_MOTION_READINGS_MAX) 
                           * (3.0F * BLECON_MOTION_READINGS_MAX)) 
            {
                _motion.callbacks->stop();
                blecon_motion_stop_sampling(dev);

                // Compute average
                float avg_x = accel_sum[0] / BLECON_MOTION_READINGS_MAX;
                float avg_y = accel_sum[1] / BLECON_MOTION_READINGS_MAX;
                float avg_z = accel_sum[2] / BLECON_MOTION_READINGS_MAX;

                _motion.callbacks->vector(avg_x, avg_y, avg_z);
            }
        }
    }
}

static void motion_trigger_handler(const struct device *dev,
			                const struct sensor_trigger *trig) {

    if( _motion.state == blecon_motion_state_waiting_for_motion ) {
        _motion.callbacks->start();
        blecon_motion_start_sampling(dev);
    }
}

void blecon_motion_start_sampling(const struct device *dev) {
    int ret;

    ret  = blecon_motion_set_sampling_freq(dev, 100);
    if (ret != 0) {
        return;
    }

    ret = sensor_trigger_set(dev, &_data_trig, data_ready_handler);
    if (ret != 0) {
        LOG_ERR("Failed to set data ready trigger: %d\n", ret);
        return;
    }

    _motion.state = blecon_motion_state_sampling_motion;
    _motion.readings_count = 0;
    _motion.store_index = 0;
}

void blecon_motion_stop_sampling(const struct device *dev) {
    int ret;

    ret  = blecon_motion_set_sampling_freq(dev, 50);
    if (ret != 0) {
        return;
    }

    ret = sensor_trigger_set(dev, &_data_trig, NULL);
    if (ret != 0) {
        LOG_ERR("Failed to clear data ready trigger: %d\n", ret);
        return;
    }

    _motion.state = blecon_motion_state_waiting_for_motion;
}

int blecon_motion_set_sampling_freq(const struct device *dev, int32_t sampling_freq) {
    int ret;

    if (IS_ENABLED(CONFIG_LIS2DH_ODR_RUNTIME)) {
        struct sensor_value odr = {
            .val1 = sampling_freq,
        };

        ret = sensor_attr_set(dev, _data_trig.chan,
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
