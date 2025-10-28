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
#include <zephyr/drivers/sensor/sht4x.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/dfu/flash_img.h>
#include <zephyr/sys/reboot.h>

#include <memfault/metrics/metrics.h>
#include <memfault/components.h>

#include "battery/battery.h"
#include <blecon/blecon_buffer.h>
#include <blecon/blecon_journal.h>
#include <blecon/blecon_util.h>

#include "motion/motion.h"
#include "ota/ota.h"
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

#define MOTION_ACC_THRESHOLD 4.0    // Acceleration threshold to register a motion event (in m/s^2)

#define REPORT_PERIOD_SEC 300
#define REBOOT_PERIOD_SEC 4500    // How frequently to reboot

#define BLECON_JOURNAL_SZ               4096
#define BLECON_JOURNAL_WATERMARK_SZ     8 // Very low watermark (at least one event)

#define MAX_CONCURRENT_SEND_OPS 2

// Events and structures
#define EVENT_TYPE_PROXIMITY        0
struct proximity_event_t { uint8_t device_id[BLECON_UUID_SZ]; int8_t rssi; } __attribute__((packed));

// Journal
K_MUTEX_DEFINE(journal_mutex);
static uint32_t _id_after_last_sent = 0;
static struct blecon_journal_t _journal;
static uint8_t _journal_array[BLECON_JOURNAL_SZ];
const static size_t _journal_event_types_size[] = {16 + 1};
static struct blecon_journal_iterator_t _journal_iter;

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

static bool _cbor_header_sent = false;
static zcbor_state_t _cbor_state[3] = {0};
static bool _send_finished = false;

static struct blecon_event_t* _start_announce_event = NULL;
static struct blecon_event_t* _start_scan_event = NULL;
static struct blecon_event_t* _start_connect_event = NULL;

#if CONFIG_LED_PROXIMITY
// TODO replace with struct led_dt_spec once Zephyr is updated
#define LED_PROXIMITY_NODE DT_CHOSEN(blecon_led_proximity)
const static struct device *led_proximity = DEVICE_DT_GET(DT_PARENT(LED_PROXIMITY_NODE));
const static uint32_t led_proximity_idx = DT_NODE_CHILD_IDX(LED_PROXIMITY_NODE);
#endif

const static struct device *led_pwm;
const static struct device *sht = DEVICE_DT_GET_ANY(sensirion_sht4x);
const static struct device *accel = DEVICE_DT_GET(DT_ALIAS(accel0));

// Function prototypes
static uint32_t get_time(void);
static void send_data(void);
static void submit_request_or_disconnect(bool first_request);
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

// Proximity
#define PROXIMITY_SCAN_TIME_MS      4000
#define MIN_RSSI                    (-127)
static struct proximity_event_t _closest_spotted = {0};
void rssi_to_led(int8_t rssi);

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
static void motion_timeout(struct k_timer *timer);
static void input_cb(struct input_event *evt, void* user_data);
static void send_report(struct k_timer *timer);
static void reboot(struct k_timer *timer);

static struct k_work_delayable _blecon_led_timeout_work_item;
K_TIMER_DEFINE(motion_timer, motion_timeout, NULL);
K_TIMER_DEFINE(report_timer, send_report, NULL);
K_TIMER_DEFINE(reboot_timer, reboot, NULL);

INPUT_CALLBACK_DEFINE(NULL, input_cb, NULL);

// Motion
static bool _in_motion = false;
static void motion_start_event(void);
static void motion_stop_event(void);
static void motion_vector_event(float x, float y, float z);

const struct blecon_motion_event_callbacks_t motion_event_callbacks = {
    .start = motion_start_event,
    .stop = motion_stop_event,
    .vector = motion_vector_event
};

// Blecon activity triggers
static void on_announce_button(struct blecon_event_t* event, void* user_data);
static void on_start_scan(struct blecon_event_t* event, void* user_data);
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
        if(!_cbor_header_sent) {
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

            // Events
            b &= zcbor_tstr_put_lit(_cbor_state, "events");
            b &= zcbor_list_start_encode(_cbor_state, 0);
            _cbor_header_sent = true;
        } else {
            zcbor_update_state(_cbor_state, buffer, buffer_sz);
        }

        k_mutex_lock(&journal_mutex, K_FOREVER);
        // Check if the journal has wrapped since we last used this iterator
        // If so, discard old iterator and start over from the new one.
        struct blecon_journal_iterator_t start_iter = blecon_journal_begin(&_journal);
        if(_journal_iter.read_event_id < start_iter.read_event_id) {
            _journal_iter = start_iter;
        }

        while(
            (_cbor_state->payload - send_op->buffer < 256 /* should be plenty of space for a very large event*/)
            && _journal_iter.valid ) {
            // Extract metadata
            blecon_journal_event_type_t event_type;
            uint32_t id;
            uint32_t timestamp;
            size_t event_size;
            blecon_journal_get_metadata(&_journal_iter, &id, &timestamp, &event_type, &event_size);

            // Open map
            b &= zcbor_map_start_encode(_cbor_state, 5 /* id + time + type + humidity + temperature */);

            // ID
            b &= zcbor_tstr_put_lit(_cbor_state, "id");
            b &= zcbor_int32_put(_cbor_state, id);

            // Time
            b &= zcbor_tstr_put_lit(_cbor_state, "time");
            b &= zcbor_tag_put(_cbor_state, 1 /* Epoch-based date/time as defined in RC7049 */);
            b &= zcbor_uint32_put(_cbor_state, timestamp);

            b &= zcbor_tstr_put_lit(_cbor_state, "type");

            switch(event_type) {
                case EVENT_TYPE_PROXIMITY: {
                    blecon_assert(event_size == sizeof(struct proximity_event_t));
                    b &= zcbor_tstr_put_lit(_cbor_state, "proximity");

                    struct proximity_event_t prox_event = {0};
                    blecon_journal_get_event(&_journal_iter, &prox_event);

                    char uuid_str[BLECON_UUID_STR_SZ] = {0};
                    blecon_util_append_uuid_string(prox_event.device_id, uuid_str);

                    b &= zcbor_tstr_put_lit(_cbor_state, "spotted_id");
                    b &= zcbor_tstr_put_term(_cbor_state, uuid_str, BLECON_UUID_STR_SZ-1);

                    b &= zcbor_tstr_put_lit(_cbor_state, "rssi");
                    b &= zcbor_int_encode(_cbor_state, &prox_event.rssi, sizeof(prox_event.rssi));
                    break;
                }
                default:
                    blecon_fatal_error();
            }

            // Close map
            b &= zcbor_map_end_encode(_cbor_state, 5);

            // Move to next event
            _journal_iter = blecon_journal_next(&_journal_iter);
        }
        k_mutex_unlock(&journal_mutex);

        // If journal is empty, mark sending as finished
        // (bearing in mind that more data can be pushed into the journal
        // before the request has been sent)
        if(!_journal_iter.valid) {
            // Close array
            b &= zcbor_list_end_encode(_cbor_state, 0);

            // Close map
            b &= zcbor_map_end_encode(_cbor_state, 3);

            // Mark as finished
            _send_finished = true;

            // Update last sent id
            _id_after_last_sent = _journal_iter.read_event_id;

            LOG_DBG("%s", "journal is now empty!");
        }

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

void submit_request_or_disconnect(bool first_request) {
    if(blecon_journal_is_empty(&_journal) && !first_request) { // If nothing to send, abort unless it's first request
        blecon_connection_terminate(&_blecon); // Nothing left to send/receive
        return;
    }

    // Reset request data
    for(size_t p = 0; p < MAX_CONCURRENT_SEND_OPS; p++) {
        _send_ops[p].busy = false;
    }

    _cbor_header_sent = false;
    _send_finished = false;

    // Clean-up request
    blecon_request_cleanup(&_request);

    // Queue initial ops
    // Start writing events

    k_mutex_lock(&journal_mutex, K_FOREVER);
    _journal_iter = blecon_journal_begin(&_journal);
    k_mutex_unlock(&journal_mutex);

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

    // Erase all sent ids
    k_mutex_lock(&journal_mutex, K_FOREVER);
    struct blecon_journal_iterator_t iterator = blecon_journal_begin(&_journal);

    // Erase all successfully sent messages
    while(iterator.valid
        && (iterator.read_event_id < _id_after_last_sent)) {
        iterator = blecon_journal_next(&iterator);
    }
    blecon_journal_erase_until(&iterator);
    k_mutex_unlock(&journal_mutex);

    // Send next request or disconnect
    submit_request_or_disconnect(false);
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
    submit_request_or_disconnect(true);
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
#ifdef CONFIG_BLECON_LIB_LED
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
    if(!blecon_connection_initiate(&_blecon)) {
        LOG_ERR("Error: %s", "could not initiate connection.");
    }
}

void on_start_scan(struct blecon_event_t* event, void* user_data) {
    LOG_DBG("Starting scan");
    if(!blecon_scan_start(&_blecon, true, false, blecon_scan_type_passive, PROXIMITY_SCAN_TIME_MS)) {
        LOG_ERR("Failed to scan for nearby devices");
        return;
    }
}

int main(void)
{
    int ret;

	if (!device_is_ready(sht)) {
		LOG_ERR("Device %s is not ready.", sht->name);
		return 0;
	}

    ret = init_motion(accel, MOTION_ACC_THRESHOLD, &motion_event_callbacks);
    if (ret) {
        LOG_ERR("Could not initialize acclerometer: %d", ret);
    }

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
    _start_scan_event = blecon_event_loop_register_event(_event_loop, on_start_scan, NULL);
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
        .namespace = "proximity-spotter",
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

    blecon_journal_init(&_journal, _journal_array, sizeof(_journal_array), _journal_event_types_size, sizeof(_journal_event_types_size) / sizeof(uint32_t));

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

static void motion_timeout(struct k_timer *timer) {
    _in_motion = false;
}

static void motion_start_event(void) {
    LOG_DBG("Motion start");
    _in_motion = true;
    blecon_event_signal(_start_scan_event);
}

static void motion_stop_event(void) {
    LOG_DBG("Motion stop");
    k_timer_start(&motion_timer, K_SECONDS(20), K_FOREVER);
}

static void motion_vector_event(float x, float y, float z) {
}

void rssi_to_led(int8_t rssi) {
    #ifdef CONFIG_LED_PROXIMITY
    // clamp to the minimum and maximum RSSI
    if(rssi >= CONFIG_PROXIMITY_RSSI_MAX_BRIGHTNESS){
        rssi = CONFIG_PROXIMITY_RSSI_MAX_BRIGHTNESS;
    }

    if(rssi < CONFIG_PROXIMITY_RSSI_MIN_BRIGHTNESS) {
        rssi = CONFIG_PROXIMITY_RSSI_MIN_BRIGHTNESS;
    }

    int8_t rssi_range = (CONFIG_PROXIMITY_RSSI_MAX_BRIGHTNESS - CONFIG_PROXIMITY_RSSI_MIN_BRIGHTNESS);
    int8_t shifted_rssi = (rssi - CONFIG_PROXIMITY_RSSI_MIN_BRIGHTNESS);

    double percentage = ((double) shifted_rssi / (double) rssi_range);

    // Since perceived brightness is not linear, map the linear percentage
    // to an exponential so that changes are more obvious.
    double scaled = 100 * (exp(percentage) - 1) / (exp(1) - 1);
    uint8_t brightness = (uint8_t) scaled;

    led_set_brightness(led_proximity, led_proximity_idx, brightness);
    #endif
}

void peer_scan_report_iterator(const struct blecon_modem_peer_scan_report_t* report, void* user_data) {
    if(report->rssi > _closest_spotted.rssi && report->rssi < 0) {
        _closest_spotted.rssi = report->rssi;
        memcpy(_closest_spotted.device_id, report->blecon_id, BLECON_UUID_SZ);
    }

    if(memcmp(report->blecon_id, _closest_spotted.device_id, BLECON_UUID_SZ) == 0){
        if(_in_motion) {
            rssi_to_led(report->rssi);
        }
        else {
            rssi_to_led(MIN_RSSI);
        }
    }
}

void on_scan_report(struct blecon_t* blecon) {
    bool overflow = false;
    blecon_scan_get_data(blecon,
        peer_scan_report_iterator,
        NULL,
        &overflow, NULL);
}

void on_scan_complete(struct blecon_t* blecon) {
    LOG_DBG("Scan complete");

    // Log the closest device we saw
    if(_closest_spotted.rssi > CONFIG_PROXIMITY_RSSI_THRESHOLD && _time_set) {
        uint32_t timestamp = get_time();
        k_mutex_lock(&journal_mutex, K_FOREVER);
        blecon_journal_push(&_journal, timestamp, EVENT_TYPE_PROXIMITY, &_closest_spotted);
        k_mutex_unlock(&journal_mutex);
    }

    // Reset closest spotted device and kick off another scan
    _closest_spotted.rssi = MIN_RSSI;

    if(_in_motion) {
        if(!blecon_scan_start(blecon, true, false, blecon_scan_type_passive, PROXIMITY_SCAN_TIME_MS)) {
            printk("Failed to scan for nearby devices\r\n");
            return;
        }
    }
}
