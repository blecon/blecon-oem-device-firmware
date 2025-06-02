/*
 * Copyright (c) 2025 Blecon Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

static const struct adc_dt_spec vdd_adc =
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

bool _battery_ready = false;
bool _needs_calibrate = true;

static int battery_setup(void)
{
    int err;

    if (!adc_is_ready_dt(&vdd_adc)) {
        printk("ADC controller device %s not ready\n", vdd_adc.dev->name);
        return 0;
    }

    err = adc_channel_setup_dt(&vdd_adc);
    if (err < 0) {
        printk("Could not setup battery adc channel: %d\n", err);
        return 0;
    }

    _battery_ready = true;
    return 0;
}
SYS_INIT(battery_setup, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

int battery_sample(void)
{
	int rc = -ENOENT;
    int err;
	int16_t buf;

    if (!_battery_ready) {
        printk("Battery ADC not ready\n");
        return rc;
    }

    struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
        .calibrate = _needs_calibrate,
	};

    _needs_calibrate = false;

    err = adc_sequence_init_dt(&vdd_adc, &sequence);
    if (err < 0) {
        printk("Could not init ADC sequence (%d)\n", err);
        return rc;
    }

    err = adc_read_dt(&vdd_adc, &sequence);
    if (err < 0) {
        printk("Could not read (%d)\n", err);
        return rc;
    }

    int32_t battery_mv = buf;

	err = adc_raw_to_millivolts_dt(&vdd_adc, &battery_mv);
    if (err < 0) {
        printk("Could not convert raw sample to millivolts (%d)\n", err);
    }

    printk("battery raw: %d ~ %d mV\n", buf, battery_mv);

	return battery_mv;
}
