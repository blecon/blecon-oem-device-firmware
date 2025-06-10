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

LOG_MODULE_REGISTER(main);

#define MOTION_ACC_THRESHOLD 6.0    // Acceleration threshold to register a motion event (in m/s^2)

#define TEMPERATURE_LOGGING_PERIOD_SEC 60
#define REPORT_PERIOD_SEC 300
#define REBOOT_PERIOD_SEC 4500    // How frequently to reboot

#define TEMPERATURE_LOGGING_STACK 512

#define FAST_BLINK_PERIOD_MS 200U
#define SLOW_BLINK_PERIOD_MS 500U

#define BLECON_JOURNAL_SZ               4096
#define BLECON_JOURNAL_WATERMARK_SZ     8 // Very low watermark (at least one event)

#define MAX_CONCURRENT_SEND_OPS 2

// Events and structures
#define EVENT_TYPE_MOTION_START     1
#define EVENT_TYPE_MOTION_STOP      2
#define EVENT_TYPE_MOTION_VECTOR    3
#define EVENT_TYPE_TEMP_HUM_UPDATE  4
struct motion_vector_event_t { float32_t x; float32_t y; float32_t z; } __attribute__((packed));
struct temp_hum_update_event_t { float32_t temperature; float32_t humidity; } __attribute__((packed));

// Journal
K_MUTEX_DEFINE(journal_mutex);
static uint32_t _id_after_last_sent = 0;
static struct blecon_journal_t _journal;
static uint8_t _journal_array[BLECON_JOURNAL_SZ];
const static size_t _journal_event_types_size[] = {0, 0, 0, 4 * 3, 4 * 2};
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
static struct blecon_event_t* _start_connect_event = NULL;

const static struct device *led_pwm;
const static struct device *sht;
#if CONFIG_LED_SAMPLING
// TODO replace with struct led_dt_spec once Zephyr is updated
#define LED_SAMPLING_NODE DT_CHOSEN(blecon_led_sampling)
const static struct device *led_sampling = DEVICE_DT_GET(DT_PARENT(LED_SAMPLING_NODE));
const static uint32_t led_sampling_idx = DT_NODE_CHILD_IDX(LED_SAMPLING_NODE);
#endif

// Function prototypes
static uint32_t get_time(void);
static void send_data(void);
static void submit_request_or_disconnect(bool first_request);
static int read_temp_hum(float *temp, float *hum);
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

const static struct blecon_callbacks_t blecon_callbacks = {
    .on_connection = on_connection,
    .on_disconnection = on_disconnection,
    .on_time_update = on_time_update,
    .on_ping_result = on_ping_result,
};

// Input and timer callbacks
static void blecon_led_timeout(struct k_timer *timer);
static void input_cb(struct input_event *evt, void* user_data);
static void send_report(struct k_timer *timer);
static void reboot(struct k_timer *timer);
#if CONFIG_LED_SAMPLING
static void sampling_led_timeout(struct k_timer *timer);
#endif

K_TIMER_DEFINE(blecon_led_timer, blecon_led_timeout, NULL);
K_TIMER_DEFINE(report_timer, send_report, NULL);
K_TIMER_DEFINE(reboot_timer, reboot, NULL);
#if CONFIG_LED_SAMPLING
K_TIMER_DEFINE(sampling_led_timer, sampling_led_timeout, NULL);
#endif

INPUT_CALLBACK_DEFINE(NULL, input_cb, NULL);

// Motion
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
                case EVENT_TYPE_TEMP_HUM_UPDATE: {
                    blecon_assert(event_size == sizeof(struct temp_hum_update_event_t));
                    b &= zcbor_tstr_put_lit(_cbor_state, "temperature_humidity_update");

                    // Retrieve data
                    struct temp_hum_update_event_t temp_hum_event = {0};
                    blecon_journal_get_event(&_journal_iter, &temp_hum_event);

                    b &= zcbor_tstr_put_lit(_cbor_state, "temperature");
                    b &= zcbor_float16_put(_cbor_state, temp_hum_event.temperature);

                    b &= zcbor_tstr_put_lit(_cbor_state, "humidity");
                    b &= zcbor_float16_put(_cbor_state, temp_hum_event.humidity);
                    break;
                }
                case EVENT_TYPE_MOTION_START: {
                    blecon_assert(event_size == 0);
                    b &= zcbor_tstr_put_lit(_cbor_state, "motion_start");
                    break;
                }
                case EVENT_TYPE_MOTION_STOP: {
                    blecon_assert(event_size == 0);
                    b &= zcbor_tstr_put_lit(_cbor_state, "motion_stop");
                    break;
                }
                case EVENT_TYPE_MOTION_VECTOR: {
                    blecon_assert(event_size == sizeof(struct motion_vector_event_t));
                    b &= zcbor_tstr_put_lit(_cbor_state, "motion_vector");

                    // Retrieve data
                    struct motion_vector_event_t vector_event = {0};
                    blecon_journal_get_event(&_journal_iter, &vector_event);

                    b &= zcbor_tstr_put_lit(_cbor_state, "x");
                    b &= zcbor_float32_put(_cbor_state, vector_event.x);

                    b &= zcbor_tstr_put_lit(_cbor_state, "y");
                    b &= zcbor_float32_put(_cbor_state, vector_event.y);

                    b &= zcbor_tstr_put_lit(_cbor_state, "z");
                    b &= zcbor_float32_put(_cbor_state, vector_event.z);
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

void blecon_led_timeout(struct k_timer *timer) {
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

#if CONFIG_LED_SAMPLING
void sampling_led_timeout(struct k_timer *timer) {
    led_off(led_sampling, led_sampling_idx);
}
#endif

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

    k_timer_start(&blecon_led_timer, K_SECONDS(5), K_FOREVER);
}

void on_start_connect(struct blecon_event_t* event, void* user_data) {
    LOG_DBG("%s", "Starting connection");
    if(!blecon_connection_initiate(&_blecon)) {
        LOG_ERR("Error: %s", "could not initiate connection.");
    }
}

int main(void)
{
    int ret;

    sht = DEVICE_DT_GET_ANY(sensirion_sht4x);
	if (!device_is_ready(sht)) {
		LOG_ERR("Device %s is not ready.", sht->name);
		return 0;
	}

    ret = init_motion(MOTION_ACC_THRESHOLD, &motion_event_callbacks);
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

static void temperature_logger_thread(void *d0, void *d1, void* d2) {
    int ret;
    uint32_t timestamp;
    float temperature, humidity;

    while(true) {
        #if CONFIG_LED_SAMPLING
        led_on(led_sampling, led_sampling_idx);
        k_timer_start(&sampling_led_timer, K_MSEC(30), K_FOREVER);
        #endif
        if (_time_set) {
            ret = read_temp_hum(&temperature, &humidity);

            if (ret < 0) {
                LOG_ERR("Error: %s", "Could not read temperature/humidity");
                continue;
            }

            struct temp_hum_update_event_t event_data = {0};
            timestamp = get_time();
            event_data.temperature = temperature;
            event_data.humidity = humidity;

            k_mutex_lock(&journal_mutex, K_FOREVER);
            blecon_journal_push(&_journal, timestamp, EVENT_TYPE_TEMP_HUM_UPDATE, &event_data);
            k_mutex_unlock(&journal_mutex);

            LOG_DBG("writing to journal (%u)", timestamp);
        }
        else {
            _pre_uptime += TEMPERATURE_LOGGING_PERIOD_SEC;
        }
        k_sleep(K_SECONDS(TEMPERATURE_LOGGING_PERIOD_SEC));
    }
}

K_THREAD_DEFINE(temperature_logger_tid, TEMPERATURE_LOGGING_STACK, temperature_logger_thread, NULL, NULL, NULL,
    K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);


static void motion_start_event(void) {
    LOG_DBG("Motion start");

    uint32_t timestamp;

    if(!_time_set) {
        return; // Can't generate any data yet
    }

    timestamp = get_time();
    k_mutex_lock(&journal_mutex, K_FOREVER);
    blecon_journal_push(&_journal, timestamp, EVENT_TYPE_MOTION_START, NULL);
    k_mutex_unlock(&journal_mutex);
}

static void motion_stop_event(void) {
    LOG_DBG("Motion stop");

    uint32_t timestamp;

    if(!_time_set) {
        return; // Can't generate any data yet
    }

    timestamp = get_time();
    k_mutex_lock(&journal_mutex, K_FOREVER);
    blecon_journal_push(&_journal, timestamp, EVENT_TYPE_MOTION_STOP, NULL);
    k_mutex_unlock(&journal_mutex);
}

static void motion_vector_event(float x, float y, float z) {
    LOG_DBG("Motion vector: %f %f %f", (double) x, (double) y, (double) z);

    uint32_t timestamp;

    if(!_time_set) {
        return; // Can't generate any data yet
    }

    struct motion_vector_event_t event_data = {0};
    timestamp = get_time();

    event_data.x = x;
    event_data.y = y;
    event_data.z = z;

    k_mutex_lock(&journal_mutex, K_FOREVER);
    blecon_journal_push(&_journal, timestamp, EVENT_TYPE_MOTION_VECTOR, &event_data);
    k_mutex_unlock(&journal_mutex);
}
