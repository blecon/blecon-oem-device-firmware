// Copyright (c) Blecon Ltd
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <blecon/blecon.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/input/input.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/dfu/flash_img.h>
#include <zephyr/sys/reboot.h>

#include <memfault/metrics/metrics.h>
#include <memfault/components.h>

#include <blecon/blecon_buffer.h>

#include "ota/ota.h"
#include "blecon_zephyr/blecon_zephyr.h"
#include "blecon_zephyr/blecon_zephyr_event_loop.h"
#include "blecon_zephyr/blecon_zephyr_memfault.h"
#include "blecon/blecon_memfault_client.h"

#define MEMFAULT_REQUEST_NAMESPACE "memfault"

LOG_MODULE_REGISTER(main);

#define CONNECTION_PERIOD_SEC (15*60)
#define REBOOT_PERIOD_SEC 4500    // How frequently to reboot

#define FAST_BLINK_PERIOD_MS 200U
#define SLOW_BLINK_PERIOD_MS 500U
#define MAX_CONCURRENT_SEND_OPS 2

static struct blecon_t _blecon = {0};
static struct blecon_event_loop_t* _event_loop = NULL;

static struct blecon_event_t* _start_announce_event = NULL;
static struct blecon_event_t* _start_connect_event = NULL;

const static struct device *led_pwm;

// Function prototypes
static void update_metrics(void);

// Connection callbacks
static void on_connection(struct blecon_t* blecon);
static void on_disconnection(struct blecon_t* blecon);
static void on_time_update(struct blecon_t* blecon);
static void on_ping_result(struct blecon_t* blecon);

const static struct blecon_callbacks_t blecon_callbacks = {
    .on_connection = on_connection,
    .on_disconnection = on_disconnection,
    .on_time_update = on_time_update,
    .on_ping_result = on_ping_result,
};

// Input and timer callbacks
static void led_timeout(struct k_timer *timer);
static void input_cb(struct input_event *evt, void* user_data);
static void connect(struct k_timer *timer);
static void reboot(struct k_timer *timer);

K_TIMER_DEFINE(led_timer, led_timeout, NULL);
K_TIMER_DEFINE(connection_timer, connect, NULL);
K_TIMER_DEFINE(reboot_timer, reboot, NULL);

INPUT_CALLBACK_DEFINE(NULL, input_cb, NULL);

// Blecon activity triggers
static void on_announce_button(struct blecon_event_t* event, void* user_data);

void on_connection(struct blecon_t* blecon) {
    // Check for OTA update
    ota_check_request();

    // Disconnect 
    blecon_connection_terminate(&_blecon);
}

void on_disconnection(struct blecon_t* blecon) {
    LOG_DBG("%s", "Disconnected");
}

void on_time_update(struct blecon_t* blecon) {
   return;
}

void on_ping_result(struct blecon_t* blecon) {
    return;
}

void led_timeout(struct k_timer *timer) {
    int ret;
    ret = led_off(led_pwm, 0);
    if(ret < 0) {
        LOG_ERR("err=%d", ret);
        return;
    }
}

void connect(struct k_timer *timer) {
    blecon_event_signal(_start_connect_event);
}

void reboot(struct k_timer *timer) {
    sys_reboot(SYS_REBOOT_WARM);
}

void input_cb(struct input_event *evt, void* user_data)
{
    switch (evt->code) {
    case INPUT_KEY_0:
        if(evt->value == 1) {
            blecon_event_signal(_start_announce_event);
        }
        break;
    default:
        break;
    }
}

void on_announce_button(struct blecon_event_t* event, void* user_data) {
    int ret;

    ret = led_blink(led_pwm, 0, FAST_BLINK_PERIOD_MS/2U, FAST_BLINK_PERIOD_MS/2U);
    if(ret < 0) {
         LOG_ERR("blink error=%d", ret);
        return;
    }

    if(!blecon_announce(&_blecon)) {
        LOG_ERR("Error: %s", "announce");
        return;
    }

    k_timer_start(&led_timer, K_SECONDS(5), K_FOREVER);
}

void on_start_connect(struct blecon_event_t* event, void* user_data) {
    LOG_DBG("%s", "Starting connection");
    if(!blecon_connection_initiate(&_blecon)) {
        LOG_ERR("Error: %s", "could not initiate connection.");
    }
}

int main(void)
{
    k_timer_start(&reboot_timer, K_SECONDS(REBOOT_PERIOD_SEC), K_FOREVER);

    led_pwm = DEVICE_DT_GET(DT_PATH(pwmleds));
    if (!device_is_ready(led_pwm)) {
        LOG_ERR("Device %s is not ready\n", led_pwm->name);
        return 0;
    }

    // Get event loop
    _event_loop = blecon_zephyr_get_event_loop();

    // Get modem
    struct blecon_modem_t* modem = blecon_zephyr_get_modem();

    // Register events that need to be run on the Blecon thread
    _start_announce_event = blecon_event_loop_register_event(_event_loop, on_announce_button, NULL);
    _start_connect_event = blecon_event_loop_register_event(_event_loop, on_start_connect, NULL);

    // Init Blecon
    blecon_init(&_blecon, modem);
    blecon_set_callbacks(&_blecon, &blecon_callbacks, NULL);
    if(!blecon_setup(&_blecon)) {
        return 1;
    }

    // Init Memfault
    struct blecon_memfault_t* memfault = blecon_zephyr_memfault_init();

    struct blecon_memfault_client_t memfault_client;
    
    uint8_t blecon_id[BLECON_UUID_SZ] = {0};
    if(!blecon_get_identity(&_blecon, blecon_id)) {
        printk("Failed to get identity\r\n");
        return 1;
    }
    
    blecon_memfault_client_init(&memfault_client, blecon_get_request_processor(&_blecon), memfault, blecon_id, MEMFAULT_REQUEST_NAMESPACE);

    // Init OTA module
    ota_init(_event_loop, &_blecon, MEMFAULT_REQUEST_NAMESPACE);

    // Print device URL
    char blecon_url[BLECON_URL_SZ] = {0};
    if(!blecon_get_url(&_blecon, blecon_url, sizeof(blecon_url))) {
        printk("Failed to get device URL\r\n");
        return 1;
    }
    LOG_INF("Device URL: %s", blecon_url);

    // Start connection timer
    k_timer_start(&connection_timer, K_SECONDS(CONNECTION_PERIOD_SEC), K_SECONDS(CONNECTION_PERIOD_SEC));

    // Initiate connection
    blecon_connection_initiate(&_blecon);

    blecon_event_loop_run(_event_loop);
    // won't ever reach here
    return 0;
}

// Handle memfault metric heartbeat timer
// This overrides a weak definition
void memfault_metrics_heartbeat_collect_data(void) {
    // Metrics are committed and reset per heartbeat, populating them here ensures at least one point per heartbeat
    update_metrics();
}

void update_metrics(void) {
    MEMFAULT_METRIC_SET_UNSIGNED(buffers_allocation_count, blecon_buffer_total_allocations_count());
    MEMFAULT_METRIC_SET_UNSIGNED(buffers_allocation_size, blecon_buffer_total_allocations_size());
}
