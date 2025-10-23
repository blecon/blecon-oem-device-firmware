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
    __ASSERT(ret == 0, "Could not get reset reason (%d)\n", ret);

    printk("Reset cause: %u\n", reset_cause);

    if (reset_cause & RESET_POR || reset_cause & RESET_PIN || reset_cause == 0) {
        power_off();
    }
}

void power_off() {
    int ret;

    ret = gpio_pin_configure_dt(&sw, (GPIO_PULL_UP | GPIO_INPUT));
    __ASSERT(ret == 0, "Could not configure wakeup GPIO (%d)\n", ret);


    ret = gpio_pin_interrupt_configure_dt(&sw, GPIO_INT_LEVEL_ACTIVE);
    __ASSERT(ret == 0, "Could not configure wakeup GPIO interrupt (%d)\n", ret);

    hwinfo_clear_reset_cause();

    // TODO: Figure out why this delay is needed for certain hardware
    k_sleep(K_MSEC(1));
    sys_poweroff();
}
