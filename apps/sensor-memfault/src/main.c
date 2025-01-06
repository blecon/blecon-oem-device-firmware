// Copyright (c) Blecon Ltd
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <blecon/blecon.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/sensor/sht4x.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/dfu/flash_img.h>
#include <zephyr/sys/reboot.h>

#include <memfault/metrics/metrics.h>
#include <memfault/components.h>

#include "battery/battery.h"
#include <blecon/blecon_buffer.h>

#include "motion/motion.h"
#include "ota/ota.h"
#include "blecon_zephyr/blecon_zephyr.h"
#include "blecon_zephyr/blecon_zephyr_event_loop.h"
#include "blecon_zephyr/blecon_zephyr_memfault.h"
#include "blecon/blecon_memfault_client.h"

#define MEMFAULT_REQUEST_NAMESPACE "memfault"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define MOTION_ACC_THRESHOLD 6.0   // Acceleration threshold to register a motion event (in m/s^2)

#define REPORT_PERIOD_SEC 30      // How frequently to check if there are metrics/dumps to send
#define OTA_CHECK_PERIOD_SEC 180  // How frequently to check if there are OTA updates
#define REBOOT_PERIOD_SEC 4500    // How frequently to reboot

#define FAST_BLINK_PERIOD_MS 200U
#define SLOW_BLINK_PERIOD_MS 500U

static bool _time_set = false;
static uint32_t _pre_uptime = 0;
static uint32_t _epoch = 0;

static struct blecon_t _blecon = {0};
static struct blecon_event_loop_t* _event_loop = NULL;

static struct blecon_event_t* _start_announce_event = NULL;
static struct blecon_event_t* _start_connect_event = NULL;
static struct blecon_event_t* _ota_check_event = NULL;
static struct blecon_event_t* _connected_event = NULL;

const static struct device *led_pwm;
const static struct device *sht;

// Function prototypes
static uint32_t get_time(void);
static int read_temp_hum(float *temp, float *hum);
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
static void send_report(struct k_timer *timer);
static void check_for_ota(struct k_timer *timer);
static void reboot(struct k_timer *timer);

K_TIMER_DEFINE(led_timer, led_timeout, NULL);
K_TIMER_DEFINE(report_timer, send_report, NULL);
K_TIMER_DEFINE(ota_timer, check_for_ota, NULL);
K_TIMER_DEFINE(reboot_timer, reboot, NULL);

INPUT_CALLBACK_DEFINE(NULL, input_cb, NULL);

// Motion
static uint32_t _motion_count = 0;
static uint32_t _gesture_count = 0;
void (*gesture_detection_callback)(void) = NULL; // Run gesture detection on motion events
void on_gesture_detected();

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
static void on_check_for_ota(struct blecon_event_t* event, void* user_data);
static void on_connection_event(struct blecon_event_t* event, void* user_data);

uint32_t get_time(void) {
    bool time_valid = false;
    uint64_t utc_time_ms_now = 0;
    uint64_t utc_time_ms_last_updated = 0;
    bool b = blecon_get_time(&_blecon, &time_valid, &utc_time_ms_now, &utc_time_ms_last_updated);
    blecon_assert(b);
    blecon_assert(time_valid);
    return (uint32_t)(utc_time_ms_now / 1000);
}

void on_connection(struct blecon_t* blecon) {
    LOG_DBG("%s", "Connected");
    blecon_event_signal(_connected_event);
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

void led_timeout(struct k_timer *timer) {
    int ret;
    ret = led_off(led_pwm, 0);
    if(ret < 0) {
        LOG_ERR("err=%d", ret);
        return;
    }
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
    // this is ISR context so post event
    blecon_event_signal(_start_connect_event);
}

void check_for_ota(struct k_timer *timer) {
    blecon_event_signal(_ota_check_event);
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
    if(memfault_packetizer_data_available()) {
        LOG_DBG("%s", "Starting connection");
        if(!blecon_connection_initiate(&_blecon)) {
            LOG_ERR("Error: %s", "could not initiate connection.");
        }
    }
}

void on_check_for_ota(struct blecon_event_t* event, void* user_data) {
    LOG_INF("Checking for OTA");
    ota_check_request();
}

void on_connection_event(struct blecon_event_t* event, void* user_data) {
    // Do not hold connection open (Memfault and OTA clients will keep it open as needed)
    if( blecon_is_connected(&_blecon) ) {
        blecon_connection_terminate(&_blecon);
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
   
    _motion_count = 0;
    ret = init_motion(MOTION_ACC_THRESHOLD, &motion_event_callbacks);
    if (ret) {
        LOG_ERR("Could not initialize acclerometer: %d", ret);
    }

    k_timer_start(&reboot_timer, K_SECONDS(REBOOT_PERIOD_SEC), K_FOREVER);

    #if CONFIG_MEMFAULT_DEMO_GESTURE_DETECT 
        gesture_detection_callback = on_gesture_detected;
    #endif

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
    _ota_check_event = blecon_event_loop_register_event(_event_loop, on_check_for_ota, NULL);
    _connected_event = blecon_event_loop_register_event(_event_loop, on_connection_event, NULL);

    blecon_init(&_blecon, modem);
    blecon_set_callbacks(&_blecon, &blecon_callbacks, NULL);
    if(!blecon_setup(&_blecon)) {
        return 1;
    }

    // Memfault
    struct blecon_memfault_t* memfault = blecon_zephyr_memfault_init();

    struct blecon_memfault_client_t memfault_client;

    uint8_t blecon_id[BLECON_UUID_SZ] = {0};
    if(!blecon_get_identity(&_blecon, blecon_id)) {
        printk("Failed to get identity\r\n");
        return 1;
    }

    blecon_memfault_client_init(&memfault_client, blecon_get_request_processor(&_blecon), memfault, blecon_id, MEMFAULT_REQUEST_NAMESPACE);

    // Print device URL
    char blecon_url[BLECON_URL_SZ] = {0};
    if(!blecon_get_url(&_blecon, blecon_url, sizeof(blecon_url))) {
        printk("Failed to get device URL\r\n");
        return 1;
    }
    LOG_INF("Device URL: %s", blecon_url);

    // Init OTA module
    ota_init(_event_loop, &_blecon, MEMFAULT_REQUEST_NAMESPACE);

    k_timer_start(&report_timer, K_SECONDS(REPORT_PERIOD_SEC), K_SECONDS(REPORT_PERIOD_SEC));
    k_timer_start(&ota_timer, K_SECONDS(OTA_CHECK_PERIOD_SEC), K_SECONDS(OTA_CHECK_PERIOD_SEC));

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
    LOG_DBG("updating metrics");
    MEMFAULT_METRIC_SET_UNSIGNED(battery_voltage_mv, battery_sample());
    MEMFAULT_METRIC_SET_UNSIGNED(buffers_allocation_count, blecon_buffer_total_allocations_count());
    MEMFAULT_METRIC_SET_UNSIGNED(buffers_allocation_size, blecon_buffer_total_allocations_size());

    float temperature, humidity;
    int ret = read_temp_hum(&temperature, &humidity);
    if (ret < 0) {
        LOG_ERR("Error: could not read temperature/humidity");
        return;
    }
    
    MEMFAULT_METRIC_SET_UNSIGNED(motion_count, (_motion_count));
    MEMFAULT_METRIC_SET_UNSIGNED(gesture_count, (_gesture_count));
    MEMFAULT_METRIC_SET_UNSIGNED(humidity, (uint32_t) (humidity * 1000));
    MEMFAULT_METRIC_SET_SIGNED(temperature, (int32_t) (temperature * 1000));
}

static void motion_start_event(void) {
    int ret;
    LOG_DBG("Motion start");
    _motion_count += 1;

    ret = led_on(led_pwm, 0);
    if(ret < 0) {
         LOG_ERR("LED error=%d", ret);
        return;
    }
    k_timer_start(&led_timer, K_MSEC(700), K_FOREVER);
}

static void motion_stop_event(void) {
    LOG_DBG("Motion stop");
}

static void motion_vector_event(float x, float y, float z) {    
    LOG_DBG("Motion vector: %f %f %f", (double) x, (double) y, (double) z);
    gesture_detection_callback();
}

void on_gesture_detected() {
    int ret;
    LOG_INF("Gesture detected");
    _gesture_count += 1;

    ret = led_on(led_pwm, 0);
    if(ret < 0) {
         LOG_ERR("LED error=%d", ret);
        return;
    }
    k_timer_start(&led_timer, K_MSEC(500), K_FOREVER);
}
