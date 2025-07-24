// Copyright (c) Blecon Ltd
// SPDX-License-Identifier: Apache-2.0

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/poweroff.h>
#include "power_switch/power_switch.h"


#define BLECON_SW_POWER_NODE DT_CHOSEN(blecon_sw_power)
static const struct gpio_dt_spec sw = GPIO_DT_SPEC_GET(BLECON_SW_POWER_NODE, gpios);

void power_sys_start() {
    int ret;
    uint32_t reset_cause;

    ret = hwinfo_get_reset_cause(&reset_cause);
    if (ret == 0) {
        printf("Reset cause: %u\n", reset_cause);
        if (reset_cause & RESET_POR || reset_cause & RESET_PIN) {
            power_off();
        }
    }
    return;
}

void power_off() {
    int ret;

    const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

    ret = gpio_pin_configure_dt(&sw, GPIO_INPUT);
    if (ret < 0) {
        printf("Could not configure wakeup GPIO (%d)\n", ret);
        return;
    }

    ret = gpio_pin_interrupt_configure_dt(&sw, GPIO_INT_LEVEL_ACTIVE);
    if (ret < 0) {
        printf("Could not configure sw0 GPIO interrupt (%d)\n", ret);
        return;
    }

    printf("Entering system off; press power button to restart\n");

    if (cons != NULL) {
        ret = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
        if (ret < 0) {
            printf("Could not suspend console (%d)\n", ret);
            return;
        }
    }

    hwinfo_clear_reset_cause();
    sys_poweroff();
}
