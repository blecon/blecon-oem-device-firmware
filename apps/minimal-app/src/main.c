// Copyright (c) Blecon Ltd
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <blecon/blecon.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/input/input.h>
#include <zcbor_common.h>
#include <zcbor_encode.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/dfu/flash_img.h>
#include <zephyr/sys/reboot.h>

#include <memfault/metrics/metrics.h>
#include <memfault/components.h>

#include <blecon/blecon_buffer.h>
#include <blecon/blecon_util.h>

#include "battery/battery.h"
#include "ota/ota.h"
#include "power_switch/power_switch.h"
#include "blecon_zephyr/blecon_zephyr.h"
#include "blecon_zephyr/blecon_zephyr_event_loop.h"
#include "blecon_zephyr/blecon_zephyr_memfault.h"
#include "blecon/blecon_memfault_client.h"

/*
We make pulling in the LED blinking/status library
optional since nRF52833 targets are memory-limited
*/
#ifdef CONFIG_BLECON_LIB_LED
#include "led/blecon_led.h"
#endif

#define MEMFAULT_REQUEST_NAMESPACE "memfault"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define REPORT_PERIOD_SEC 300
#define REBOOT_PERIOD_SEC 4500    // How frequently to reboot

#define MAX_CONCURRENT_SEND_OPS 2

static bool _time_set = false;
static uint32_t _pre_uptime = 0;
static uint32_t _epoch = 0;

static struct blecon_t _blecon = {0};
static struct blecon_event_loop_t* _event_loop = NULL;

static struct blecon_request_t _request = {0};
static uint8_t _incoming_data_buffer[64] = {0};

static struct blecon_request_receive_data_op_t _receive_op = {0};

struct send_op_t {
    struct blecon_request_send_data_op_t op;
    uint8_t buffer[2048];
    bool busy;
};
static struct send_op_t _send_ops[MAX_CONCURRENT_SEND_OPS] = {0};

static zcbor_state_t _cbor_state[3] = {0};
static bool _should_send_data = false;
static bool _send_finished = false;

static struct blecon_event_t* _start_announce_event = NULL;
static struct blecon_event_t* _start_connect_event = NULL;

const static struct device *led_pwm;

// Function prototypes
static uint32_t get_time(void);
static void send_data(void);
static void submit_request();
static void update_metrics(void);

// Requests callbacks
static void request_on_closed(struct blecon_request_t* request);
static void request_on_data_sent(struct blecon_request_send_data_op_t* send_data_op, bool data_sent);
static uint8_t* request_alloc_incoming_data_buffer(struct blecon_request_receive_data_op_t* receive_data_op, size_t sz);
static void request_on_data_received(struct blecon_request_receive_data_op_t* receive_data_op, bool data_received, const uint8_t* data, size_t sz, bool finished);

const static struct blecon_request_callbacks_t blecon_request_callbacks = {
    .on_closed = request_on_closed,
    .on_data_sent = request_on_data_sent,
    .alloc_incoming_data_buffer = request_alloc_incoming_data_buffer,
    .on_data_received = request_on_data_received,
};

// Connection callbacks
static void on_connection(struct blecon_t* blecon);
static void on_disconnection(struct blecon_t* blecon);
static void on_time_update(struct blecon_t* blecon);
static void on_ping_result(struct blecon_t* blecon);

static void on_scan_report(struct blecon_t* blecon);
static void on_scan_complete(struct blecon_t* blecon);
void peer_scan_report_iterator(const struct blecon_modem_peer_scan_report_t* report, void* user_data);

const static struct blecon_callbacks_t blecon_callbacks = {
    .on_connection = on_connection,
    .on_disconnection = on_disconnection,
    .on_time_update = on_time_update,
    .on_ping_result = on_ping_result,
    .on_scan_report = on_scan_report,
    .on_scan_complete = on_scan_complete
};

// Input and timer callbacks
static void blecon_led_timeout(struct k_work *item);
static void input_cb(struct input_event *evt, void* user_data);
static void send_report(struct k_timer *timer);
static void reboot(struct k_timer *timer);

static struct k_work_delayable _blecon_led_timeout_work_item;
K_TIMER_DEFINE(report_timer, send_report, NULL);
K_TIMER_DEFINE(reboot_timer, reboot, NULL);

INPUT_CALLBACK_DEFINE(NULL, input_cb, NULL);

// Power on/off indicator
#define POWER_BLINK_PERIOD_MS   200
// TODO replace with struct led_dt_spec once Zephyr is updated
#define LED_POWER_NODE DT_CHOSEN(blecon_led_power)
const static struct device *led_power = DEVICE_DT_GET(DT_PARENT(LED_POWER_NODE));
const static uint32_t led_power_idx = DT_NODE_CHILD_IDX(LED_POWER_NODE);

static void flash_power_led(uint32_t blinks)
{
    for (uint32_t i = 0; i < blinks; i++){
        if (i != 0) {
            k_sleep(K_MSEC(POWER_BLINK_PERIOD_MS));
        }
        led_on(led_power, led_power_idx);
        k_sleep(K_MSEC(POWER_BLINK_PERIOD_MS));
        led_off(led_power, led_power_idx);
    }
}

// Blecon activity triggers
static void on_announce_button(struct blecon_event_t* event, void* user_data);
static void on_start_connect(struct blecon_event_t* event, void* user_data);

uint32_t get_time(void) {
    bool time_valid = false;
    uint64_t utc_time_ms_now = 0;
    uint64_t utc_time_ms_last_updated = 0;
    bool b = blecon_get_time(&_blecon, &time_valid, &utc_time_ms_now, &utc_time_ms_last_updated);
    blecon_assert(b);
    blecon_assert(time_valid);
    return (uint32_t)(utc_time_ms_now / 1000);
}

void send_data(void) {
    for(size_t p = 0; p < MAX_CONCURRENT_SEND_OPS; p++) {
        if(_send_finished) {
            return;
        }
        if(_send_ops[p].busy) {
            continue;
        }

        struct send_op_t* send_op = &_send_ops[p];

        send_op->busy = true;
        uint8_t* buffer = send_op->buffer;
        size_t buffer_sz = sizeof(send_op->buffer);

        bool b = true;

        zcbor_new_encode_state(_cbor_state, sizeof(_cbor_state) / sizeof(zcbor_state_t), buffer, buffer_sz, 1);

        // Open map
        b &= zcbor_map_start_encode(_cbor_state, 3);

        // Uptime
        b &= zcbor_tstr_put_lit(_cbor_state, "uptime");
        b &= zcbor_uint32_put(_cbor_state, get_time() - _epoch);

        // Battery level
        b &= zcbor_tstr_put_lit(_cbor_state, "battery");

        int batt_mv = battery_sample();
        if (batt_mv < 0) {
            LOG_ERR("Failed to read battery voltage: %d", batt_mv);
            batt_mv = 0;
        }
        b &= zcbor_uint32_put(_cbor_state, batt_mv);

        // Close map
        b &= zcbor_map_end_encode(_cbor_state, 3);

        // Mark as finished
        _send_finished = true;

        // Really, nothing should have gone wrong with encoding
        blecon_assert(b);

        // Mark op as busy
        send_op->busy = true;

        // Adjust buffer size based on CBOR state
        buffer_sz = _cbor_state->payload - buffer;

        // Create send data operation
        if(!blecon_request_send_data(&send_op->op, &_request, buffer, buffer_sz, _send_finished, send_op)) {
            LOG_ERR("Error: %s", "Failed to send data");
            blecon_request_cleanup(&_request);
            blecon_connection_terminate(&_blecon);
            return;
        }
    }
}

void submit_request() {
    // Reset request data
    for(size_t p = 0; p < MAX_CONCURRENT_SEND_OPS; p++) {
        _send_ops[p].busy = false;
    }

    _send_finished = false;

    // Clean-up request
    blecon_request_cleanup(&_request);

    // Queue initial ops
    // Start writing events
    send_data();

    // Create receive data operation
    if(!blecon_request_receive_data(&_receive_op, &_request, NULL)) {
        LOG_ERR("Error: %s", "Unable to request receive data");
        blecon_request_cleanup(&_request);
        blecon_connection_terminate(&_blecon);
        return;
    }

    // Submit request
    blecon_submit_request(&_blecon, &_request);
}

void request_on_closed(struct blecon_request_t* request) {
    enum blecon_request_status_code_t status_code = blecon_request_get_status(request);

    if(status_code != blecon_request_status_ok) {
        LOG_ERR("Request status error: %x", status_code);
        blecon_connection_terminate(&_blecon);
        return;
    }
    // Shutdown the connection
    blecon_connection_terminate(&_blecon);
}

void request_on_data_sent(struct blecon_request_send_data_op_t* send_data_op, bool data_sent) {
    if(!data_sent) {
        LOG_ERR("Error: %s", "Failed to send data");
        return;
    }
    struct send_op_t *send_op = (struct send_op_t *) send_data_op->user_data;
    send_op->busy = false;

    LOG_DBG("%s", "Data sent successfully");

    // Send more data
    send_data();
}

uint8_t* request_alloc_incoming_data_buffer(struct blecon_request_receive_data_op_t* receive_data_op, size_t sz) {
    return _incoming_data_buffer;
}

void request_on_data_received(struct blecon_request_receive_data_op_t* receive_data_op, bool data_received, const uint8_t* data, size_t sz, bool finished) {
    LOG_DBG("%s", "Received data");
}

void on_connection(struct blecon_t* blecon) {
    // Check for OTA update
    ota_check_request();

    if(!_time_set) {
        // Disconnect
        blecon_connection_terminate(&_blecon);
        return; // Can't send anything yet
    }

    // Rate limited by report period so that we don't send data every connection
    if(_should_send_data) {
        submit_request();
        _should_send_data = false;
    }
}

void on_disconnection(struct blecon_t* blecon) {
    LOG_DBG("%s", "Disconnected");
}

void on_time_update(struct blecon_t* blecon) {
    if (!_time_set) {
        _epoch = get_time() - _pre_uptime;
    }
    _time_set = true;
    LOG_DBG("%s", "Time updated");
}

void on_ping_result(struct blecon_t* blecon) {
    return;
}

void blecon_led_timeout(struct k_work *item) {
#ifdef CONFIG_BLECON_LIB_LED
    blecon_led_enable(false);
    blecon_led_set_announce(false);
#else
    int ret;
    ret = led_off(led_pwm, 0);
    if(ret < 0) {
        LOG_ERR("err=%d", ret);
        return;
    }
#endif
}

void send_report(struct k_timer *timer) {
    blecon_event_signal(_start_connect_event);
}

void reboot(struct k_timer *timer) {
    sys_reboot(SYS_REBOOT_WARM);
}

void input_cb(struct input_event *evt, void* user_data)
{
    switch (evt->code) {
    case INPUT_KEY_A:
        if(evt->value == 1) {
            blecon_event_signal(_start_announce_event);
        }
        break;
    case INPUT_KEY_X:
        if(evt->value == 1) {
            flash_power_led(3);
            power_off();
        }
        break;
    default:
        break;
    }
}

void on_announce_button(struct blecon_event_t* event, void* user_data) {
#ifdef CONFIG_BLECON_LIB_LED
    blecon_led_enable(true);
    blecon_led_set_announce(true);
#else
    int ret;

    ret = led_blink(led_pwm, 0, FAST_BLINK_PERIOD_MS/2U, FAST_BLINK_PERIOD_MS/2U);
    if(ret < 0) {
         LOG_ERR("blink error=%d", ret);
        return;
    }
#endif
    if(!blecon_announce(&_blecon)) {
        LOG_ERR("Error: %s", "announce");
        return;
    }

    k_work_schedule(&_blecon_led_timeout_work_item, K_SECONDS(5));
}

void on_start_connect(struct blecon_event_t* event, void* user_data) {
    LOG_DBG("%s", "Starting connection");
    _should_send_data = true;

    if(!blecon_connection_initiate(&_blecon)) {
        LOG_ERR("Error: %s", "could not initiate connection.");
    }
}

int main(void)
{
    power_sys_start();
    flash_power_led(1);

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

    // Registered scheduled work item
    k_work_init_delayable(&_blecon_led_timeout_work_item, blecon_led_timeout);

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

     // Init request
    const static struct blecon_request_parameters_t request_params = {
        .namespace = "minimal-app",
        .method = "log",
        .oneway = true,
        .request_content_type = "application/cbor",
        .response_content_type = NULL,
        .response_mtu = sizeof(_incoming_data_buffer),
        .callbacks = &blecon_request_callbacks,
        .user_data = NULL
    };
    blecon_request_init(&_request, &request_params);

    // Print device URL
    char blecon_url[BLECON_URL_SZ] = {0};
    if(!blecon_get_url(&_blecon, blecon_url, sizeof(blecon_url))) {
        printk("Failed to get device URL\r\n");
        return 1;
    }
    LOG_INF("Device URL: %s", blecon_url);

    k_timer_start(&report_timer, K_SECONDS(REPORT_PERIOD_SEC), K_SECONDS(REPORT_PERIOD_SEC));

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
    MEMFAULT_METRIC_SET_UNSIGNED(battery_voltage_mv, battery_sample());
    MEMFAULT_METRIC_SET_UNSIGNED(buffers_allocation_count, blecon_buffer_total_allocations_count());
    MEMFAULT_METRIC_SET_UNSIGNED(buffers_allocation_size, blecon_buffer_total_allocations_size());
}

void on_scan_report(struct blecon_t* blecon) {
    return;
}

void on_scan_complete(struct blecon_t* blecon) {
    return;
}
