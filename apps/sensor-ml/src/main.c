// Copyright (c) Blecon Ltd
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
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

#include "inference.h"

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

LOG_MODULE_REGISTER(main);

#define MOTION_ACC_THRESHOLD 6.0    // Acceleration threshold to register a motion event (in m/s^2)

#define REBOOT_PERIOD_SEC 4500    // How frequently to reboot

#define TEMPERATURE_LOGGING_STACK 512

#define FAST_BLINK_PERIOD_MS 200U
#define SLOW_BLINK_PERIOD_MS 500U

#define BLECON_JOURNAL_SZ               4096
#define BLECON_JOURNAL_WATERMARK_SZ     8 // Very low watermark (at least one event)

#define MAX_CONCURRENT_SEND_OPS 2

// Events and structures
#define EVENT_TYPE_INFERENCE          0
struct inference_event_t { uint8_t gesture; } __attribute__((packed));
static const char* _gesture_names[] = { "idle", "snake", "updown", "wave" };
static const uint8_t _gesture_led_brigtness_mappings[][2] = {
    { 0, 0 }, // idle
    { 0, 50 }, // snake
    { 100, 0 }, // updown
    { 50, 25 } // wave
};

// Journal
K_MUTEX_DEFINE(journal_mutex);
static uint32_t _id_after_last_sent = 0;
static struct blecon_journal_t _journal;
static uint8_t _journal_array[BLECON_JOURNAL_SZ];
const static size_t _journal_event_types_size[] = {sizeof(struct inference_event_t)};
static struct blecon_journal_iterator_t _journal_iter;

static atomic_t _leds_enabled = ATOMIC_INIT(0);
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
static struct blecon_event_t* _start_connect_event = NULL;

const static struct device *led_pwm;
const static struct device *sht;
#if CONFIG_LED_GESTURE
// TODO replace with struct led_dt_spec once Zephyr is updated
#define LED_GESTURE_1_NODE DT_CHOSEN(blecon_led_gesture_1)
#define LED_GESTURE_2_NODE DT_CHOSEN(blecon_led_gesture_2)
#define LED_GESTURE_COUNT 2
const static struct device *led_gesture[] = { 
    DEVICE_DT_GET(DT_PARENT(LED_GESTURE_1_NODE)),
    DEVICE_DT_GET(DT_PARENT(LED_GESTURE_2_NODE))
};
const static uint32_t led_gesture_idx[] = {
    DT_NODE_CHILD_IDX(LED_GESTURE_1_NODE),
    DT_NODE_CHILD_IDX(LED_GESTURE_2_NODE)
}; 
#endif

// Function prototypes
static uint32_t get_time(void);
static void send_data(void);
static void submit_request_or_disconnect(bool first_request);
static void update_metrics(void);
static void inference_event(enum inference_category_t category, float score);

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

const static struct blecon_callbacks_t blecon_callbacks = {
    .on_connection = on_connection,
    .on_disconnection = on_disconnection,
    .on_time_update = on_time_update,
    .on_ping_result = on_ping_result,
};

// Input and timer callbacks
static void blecon_led_timeout(struct k_work *item);
static void input_cb(struct input_event *evt, void* user_data);
static void send_report(struct k_timer *timer);
static void reboot(struct k_timer *timer);
#if CONFIG_LED_GESTURE
static void gesture_led_timeout(struct k_timer *timer);
#endif
static void disable_leds(struct k_work *work);

static struct k_work_delayable _blecon_led_timeout_work_item;
K_TIMER_DEFINE(report_timer, send_report, NULL);
K_TIMER_DEFINE(reboot_timer, reboot, NULL);
#if CONFIG_LED_GESTURE
K_TIMER_DEFINE(gesture_led_timer, gesture_led_timeout, NULL);
#endif
K_WORK_DELAYABLE_DEFINE(led_enabled_work, disable_leds);

INPUT_CALLBACK_DEFINE(NULL, input_cb, NULL);

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
#ifdef CONFIG_BLECON_LIB_LED
    blecon_led_data_activity();
#endif
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
            b &= zcbor_map_start_encode(_cbor_state, 4 /* id + time + type + gesture */);

            // ID
            b &= zcbor_tstr_put_lit(_cbor_state, "id");
            b &= zcbor_int32_put(_cbor_state, id);

            // Time
            b &= zcbor_tstr_put_lit(_cbor_state, "time");
            b &= zcbor_tag_put(_cbor_state, 1 /* Epoch-based date/time as defined in RC7049 */);
            b &= zcbor_uint32_put(_cbor_state, timestamp);

            b &= zcbor_tstr_put_lit(_cbor_state, "type");

            switch(event_type) {
                case EVENT_TYPE_INFERENCE: {
                    blecon_assert(event_size == sizeof(struct inference_event_t));
                    b &= zcbor_tstr_put_lit(_cbor_state, "inference");

                    // Retrieve data
                    struct inference_event_t gesture_event = {0};
                    blecon_journal_get_event(&_journal_iter, &gesture_event);

                    b &= zcbor_tstr_put_lit(_cbor_state, "gesture");
                    b &= zcbor_tstr_put_term(_cbor_state, _gesture_names[gesture_event.gesture], 8);
                    break;
                }
                default:
                    blecon_fatal_error();
            }

            // Close map
            b &= zcbor_map_end_encode(_cbor_state, 4);

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

    if( ota_is_downloading() ) {
        // Do not send data while downloading an update
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
#ifdef CONFIG_BLECON_LIB_LED
    blecon_led_data_activity();
#endif

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
#ifdef CONFIG_BLECON_LIB_LED
    blecon_led_set_connection_state(blecon_led_connection_state_connected);
#endif
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
#ifdef CONFIG_BLECON_LIB_LED
    blecon_led_set_connection_state(blecon_led_connection_state_disconnected);
#endif
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

int read_temp_hum(float *temp, float *hum) {
    struct sensor_value s_temp, s_hum;

    if (sensor_sample_fetch(sht)) {
        LOG_ERR("%s", "Failed to fetch sample from SHT4X device");
        return -1;
    }

    sensor_channel_get(sht, SENSOR_CHAN_AMBIENT_TEMP, &s_temp);
    sensor_channel_get(sht, SENSOR_CHAN_HUMIDITY, &s_hum);

    *temp = sensor_value_to_float(&s_temp);
    *hum = sensor_value_to_float(&s_hum);

    return 0;
}

void send_report(struct k_timer *timer) {
    blecon_event_signal(_start_connect_event);
}

void reboot(struct k_timer *timer) {
    sys_reboot(SYS_REBOOT_WARM);
}

#if CONFIG_LED_GESTURE
void gesture_led_timeout(struct k_timer *timer) {
    for(size_t i = 0; i < LED_GESTURE_COUNT; i++) {
        led_off(led_gesture[i], led_gesture_idx[i]);
    }
    
}
#endif

void disable_leds(struct k_work *work) {
    #ifdef CONFIG_BLECON_LIB_LED
    blecon_led_enable(false);
    #endif
    atomic_clear_bit(&_leds_enabled, 0);
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
    // Enable LEDs for a while and start announcing
    atomic_set_bit(&_leds_enabled, 0);
    k_work_reschedule(&led_enabled_work, K_MINUTES(CONFIG_LEDS_ENABLED_DURATION_MIN));
#ifdef CONFIG_BLECON_LIB_LED
    blecon_led_enable(true);
    blecon_led_set_announce(true);
#endif
    if(!blecon_announce(&_blecon)) {
        LOG_ERR("Error: %s", "announce");
        return;
    }

    k_work_schedule(&_blecon_led_timeout_work_item, K_SECONDS(5));
}

void on_start_connect(struct blecon_event_t* event, void* user_data) {
    LOG_DBG("%s", "Starting connection");
#ifdef CONFIG_BLECON_LIB_LED
    blecon_led_set_connection_state(blecon_led_connection_state_connecting);
#endif
    if(!blecon_connection_initiate(&_blecon)) {
        LOG_ERR("Error: %s", "could not initiate connection.");
    }
}

int main(void)
{
    int ret;

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

    // Initialise delayable work item for stopping the announce LEDAdd commentMore actions
    k_work_init_delayable(&_blecon_led_timeout_work_item, blecon_led_timeout);

    // Init Blecon
    blecon_init(&_blecon, modem);
    blecon_set_callbacks(&_blecon, &blecon_callbacks, NULL);
    if(!blecon_setup(&_blecon)) {
        return 1;
    }

    // Init inference module
    inference_init(inference_event);

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
        .namespace = "sensor",
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

    k_timer_start(&report_timer, K_SECONDS(CONFIG_REPORTING_PERIOD_SEC), K_SECONDS(CONFIG_REPORTING_PERIOD_SEC));

#ifdef CONFIG_BLECON_LIB_LED
    blecon_led_set_connection_state(blecon_led_connection_state_connecting);
#endif

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

void inference_event(enum inference_category_t category, float score) {
    LOG_DBG("Inference event: %s %f", _gesture_names[category], (double) score);

    uint32_t timestamp;

    if(!_time_set) {
        return; // Can't generate any data yet
    }

    if(category == inference_category_idle) {
        // Don't log idle events
        return;
    }

    if(score < 0.9) {
        // Don't log low-confidence events
        return;
    }

    #if CONFIG_LED_GESTURE
    // Set brightness for the gesture LEDs and turn them on
    if(atomic_test_bit(&_leds_enabled, 0)) {
        for(size_t i = 0; i < LED_GESTURE_COUNT; i++) {
            // led_set_brightness() will turn the LED on
            led_set_brightness(led_gesture[i], led_gesture_idx[i], _gesture_led_brigtness_mappings[category][i]);
        }
        
        // The timer will be restarted if needed
        k_timer_start(&gesture_led_timer, K_SECONDS(5), K_FOREVER);
    }
    #endif

    struct inference_event_t event_data = {0};
    timestamp = get_time();

    event_data.gesture = category;

    k_mutex_lock(&journal_mutex, K_FOREVER);
    blecon_journal_push(&_journal, timestamp, EVENT_TYPE_INFERENCE, &event_data);
    k_mutex_unlock(&journal_mutex);
}
