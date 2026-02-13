/*
 * ESP RainMaker Water Level Monitor
 * Copyright (C) 2026 Saulo G. Felix
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
 * SOFTWARE.
*/

#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <driver/gpio.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_scenes.h>
#include <esp_rmaker_console.h>
#include <esp_rmaker_ota.h>
#include <esp_rmaker_common_events.h>

#include <app_network.h>
#include <app_insights.h>

#include "jsnsr04t.h"

#define ESP_RMAKER_DEVICE_WATER_TANK_SENSOR "esp.device.water_sensor"
#define ESP_RMAKER_DEVICE_WATER_TANK_SENSOR_RAW "esp.device.water_sensor_raw"
#define ESP_RMAKER_DEVICE_WATER_TANK_SENSOR_VOLUME "esp.device.water_tank_volume"
#define ESP_RMAKER_DEVICE_WATER_TANK_SENSOR_FLOW_RATE "esp.device.flow_rate"
#define ESP_RMAKER_DEVICE_WATER_TANK_CAPACITY "esp.device.tank_capacity"
#define ESP_RMAKER_DEVICE_WATER_TANK_STATUS_MSG "esp.device.sensor_status_msg"
#define ESP_RMAKER_DEVICE_WATER_TANK_STATUS_CODE "esp.device.sensor_status_code"
#define ESP_RMAKER_DEVICE_WATER_TANK_CONFIG_VOLUME "esp.config.tank_volume"
#define ESP_RMAKER_DEVICE_WATER_TANK_CONFIG_SENSOR_TILT "esp.config.sensor_tilt"
#define ESP_RMAKER_DEVICE_WATER_TANK_CONFIG_LEVEL "esp.config.tank_level"
#define ESP_RMAKER_DEVICE_WATER_TANK_CONFIG_TANK_HEIGHT "esp.config.tank_height"
#define ESP_RMAKER_DEVICE_WATER_TANK_CONFIG_TANK_BASE "esp.config.tank_base"

#define SENSOR_STATUS_CODE_OPERATIONAL          (1 << 0)
#define SENSOR_STATUS_CODE_ERROR                (1 << 1)
#define SENSOR_STATUS_CODE_NO_RESPONSE          (1 << 2)
#define SENSOR_STATUS_CODE_FULL_CAPACITY        (1 << 3)
#define SENSOR_STATUS_CODE_LOW_STORAGE          (1 << 4)
#define SENSOR_STATUS_CODE_CRITICAL_STORAGE     (1 << 5)
#define SENSOR_STATUS_CODE_CALIBRATION_REQUIRED (1 << 5)

#define BLINK_SUCCESS_US                40000
#define BLINK_WARNING_LOW_CAPACITY      100000
#define BLINK_WARNING_CRITICAL_CAPACITY 20000
#define BLINK_WARNING_FULL_CAPACITY     500000

#define MAX_SAMPLES     ((CONFIG_WATER_LEVEL_FILTER_WINDOW_MINUTES * 60) / CONFIG_WATER_LEVEL_FILTER_SAMPLE_PERIOD_S)

static uint16_t s_max_level_value =     CONFIG_WATER_LEVEL_SENSOR_THRESHOLD_DEFAULT;
static uint16_t s_max_tank_height_cm =  CONFIG_WATER_LEVEL_TANK_HEIGHT_CM;
static uint16_t s_max_tank_volume =     CONFIG_WATER_LEVEL_TANK_VOLUME_LTS;
static uint16_t s_sensor_tilt =         CONFIG_WATER_LEVEL_SENSOR_TILT;
static uint16_t s_low_level =           CONFIG_WATER_LEVEL_LOW_THRESHOLD;
static uint16_t s_critical_level =      CONFIG_WATER_LEVEL_CRITICAL_THRESHOLD;
static uint16_t s_tank_base_cm =        CONFIG_WATER_LEVEL_TANK_BASE_CM;

static float flow_rate_lpm = 0.0f;
static float level_ema = 0.0f;
static bool level_ema_init = false;
static const char *TAG = "water_level";

#ifdef CONFIG_WATER_LEVEL_ENABLE_GPIO_LED_STATUS
typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_SUCCESS,          // two short blinks
    LED_MODE_FULL,             // solid ON (except during success)
    LED_MODE_WARN_LOW,         // slow blink
    LED_MODE_WARN_CRITICAL     // fast blink
} led_mode_t;

static led_mode_t current_mode = LED_MODE_OFF;
static led_mode_t pending_mode = LED_MODE_OFF;
static int led_tick = 0;
static bool in_success = false;

static inline void led_on(void) {
    gpio_set_level(CONFIG_WATER_LEVEL_GPIO_LED, 1);
}

static inline void led_off(void) {
    gpio_set_level(CONFIG_WATER_LEVEL_GPIO_LED, 0);
}
#endif

typedef struct {
    uint16_t main_status;
    uint16_t flags;
} sensor_status_t;

static sensor_status_t g_sensor_status;

#define SENSOR_STATUS_DEFAULT()   \
    (sensor_status_t){            \
        .main_status = SENSOR_STATUS_CODE_NO_RESPONSE, \
        .flags = 0                      \
    }

static TaskHandle_t ultrasonic_task_handle;
static TaskHandle_t publisher_task_handle;
static QueueHandle_t boot_evt_queue = NULL;

static void (*boot_user_callback)(void) = NULL;
static void sensor_update(void);
static void sensor_publish(void);
static double compute_tank_capacity(int dt);

static esp_timer_handle_t water_level_timer;
static esp_timer_handle_t sensor_pub_timer;
static esp_rmaker_param_t *level_param;
static esp_rmaker_param_t *calib_tank_height_param;
static esp_rmaker_param_t *calib_tank_base_param;
static esp_rmaker_param_t *calib_tank_volume_param;
static esp_rmaker_param_t *calib_sensor_ang_deg_param;
static esp_rmaker_param_t *water_tank_volume_param;
static esp_rmaker_param_t *water_tank_sensor_raw_param;
static esp_rmaker_param_t *water_tank_capacity_param;
static esp_rmaker_param_t *flow_rate_param;
static esp_rmaker_device_t *water_tank_level_device;
static esp_rmaker_param_t *sensor_status_msg_param;
static esp_rmaker_param_t *sensor_status_int_param;

mjd_jsnsr04t_config_t jsnsr04t_config;

double g_latest_measurement = 0.0f;
 
typedef struct {
    int64_t timestamp_us;
    double value;
} sensor_sample_t;

static sensor_sample_t samples[MAX_SAMPLES];
static int sample_count = 0;
static int sample_head = 0;


/* Event handler for catching RainMaker events */
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == RMAKER_EVENT) {
        switch (event_id) {
            case RMAKER_EVENT_INIT_DONE:
                ESP_LOGI(TAG, "RainMaker Initialised.");
                break;
            case RMAKER_EVENT_CLAIM_STARTED:
                ESP_LOGI(TAG, "RainMaker Claim Started.");
                break;
            case RMAKER_EVENT_CLAIM_SUCCESSFUL:
                ESP_LOGI(TAG, "RainMaker Claim Successful.");
                break;
            case RMAKER_EVENT_CLAIM_FAILED:
                ESP_LOGI(TAG, "RainMaker Claim Failed.");
                break;
            case RMAKER_EVENT_LOCAL_CTRL_STARTED:
                ESP_LOGI(TAG, "Local Control Started.");
                break;
            case RMAKER_EVENT_LOCAL_CTRL_STOPPED:
                ESP_LOGI(TAG, "Local Control Stopped.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled RainMaker Event: %"PRIi32, event_id);
        }
    } else if (event_base == RMAKER_COMMON_EVENT) {
        switch (event_id) {
            case RMAKER_EVENT_REBOOT:
                ESP_LOGI(TAG, "Rebooting in %d seconds.", *((uint8_t *)event_data));
                break;
            case RMAKER_EVENT_WIFI_RESET:
                ESP_LOGI(TAG, "Wi-Fi credentials reset.");
                break;
            case RMAKER_EVENT_FACTORY_RESET:
                ESP_LOGI(TAG, "Node reset to factory defaults.");
                break;
            case RMAKER_MQTT_EVENT_CONNECTED:
                ESP_LOGI(TAG, "MQTT Connected.");
                break;
            case RMAKER_MQTT_EVENT_DISCONNECTED:
                ESP_LOGI(TAG, "MQTT Disconnected.");
                break;
            case RMAKER_MQTT_EVENT_PUBLISHED:
                ESP_LOGI(TAG, "MQTT Published. Msg id: %d.", *((int *)event_data));
                break;
            default:
                ESP_LOGW(TAG, "Unhandled RainMaker Common Event: %"PRIi32, event_id);
        }
    } else if (event_base == APP_NETWORK_EVENT) {
        switch (event_id) {
            case APP_NETWORK_EVENT_QR_DISPLAY:
                ESP_LOGI(TAG, "Provisioning QR : %s", (char *)event_data);
                break;
            case APP_NETWORK_EVENT_PROV_TIMEOUT:
                ESP_LOGI(TAG, "Provisioning Timed Out. Please reboot.");
                break;
            case APP_NETWORK_EVENT_PROV_RESTART:
                ESP_LOGI(TAG, "Provisioning has restarted due to failures.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled App Wi-Fi Event: %"PRIi32, event_id);
                break;
        }
    } else if (event_base == RMAKER_OTA_EVENT) {
        switch(event_id) {
            case RMAKER_OTA_EVENT_STARTING:
                ESP_LOGI(TAG, "Starting OTA.");
                break;
            case RMAKER_OTA_EVENT_IN_PROGRESS:
                ESP_LOGI(TAG, "OTA is in progress.");
                break;
            case RMAKER_OTA_EVENT_SUCCESSFUL:
                ESP_LOGI(TAG, "OTA successful.");
                break;
            case RMAKER_OTA_EVENT_FAILED:
                ESP_LOGI(TAG, "OTA Failed.");
                break;
            case RMAKER_OTA_EVENT_REJECTED:
                ESP_LOGI(TAG, "OTA Rejected.");
                break;
            case RMAKER_OTA_EVENT_DELAYED:
                ESP_LOGI(TAG, "OTA Delayed.");
                break;
            case RMAKER_OTA_EVENT_REQ_FOR_REBOOT:
                ESP_LOGI(TAG, "Firmware image downloaded. Please reboot your device to apply the upgrade.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled OTA Event: %"PRIi32, event_id);
                break;
        }
    } else {
        ESP_LOGW(TAG, "Invalid event received!");
    }
}

#ifdef CONFIG_WATER_LEVEL_ENABLE_GPIO_LED_STATUS
void led_set_mode(led_mode_t mode)
{
    if (in_success) {
        // defer update until success blink finishes
        pending_mode = mode;
        return;
    }

    current_mode = mode;
}

void led_success_blink(void)
{
    in_success = true;
    led_tick = 0;
    current_mode = LED_MODE_SUCCESS;
}

static void led_update(void)
{
    led_tick++;

    switch (current_mode) {
    case LED_MODE_SUCCESS: {
        if (led_tick < 2) {
            gpio_set_level(CONFIG_WATER_LEVEL_GPIO_LED, 1);
        } else if (led_tick < 4) {
            gpio_set_level(CONFIG_WATER_LEVEL_GPIO_LED, 0);
        } else if (led_tick < 6) {
            gpio_set_level(CONFIG_WATER_LEVEL_GPIO_LED, 1);
        } else if (led_tick < 8) {
            gpio_set_level(CONFIG_WATER_LEVEL_GPIO_LED, 0);
        } else {
            in_success = false;
            led_tick = 0;
            current_mode = pending_mode;
            pending_mode = LED_MODE_OFF;
        }
        break;
    }

    case LED_MODE_FULL:
        gpio_set_level(CONFIG_WATER_LEVEL_GPIO_LED, 1);
        break;
    case LED_MODE_WARN_LOW:
        gpio_set_level(CONFIG_WATER_LEVEL_GPIO_LED, (led_tick % 20) < 10);
        break;
    case LED_MODE_WARN_CRITICAL:
        gpio_set_level(CONFIG_WATER_LEVEL_GPIO_LED, (led_tick % 4) < 2);
        break;
    case LED_MODE_OFF:
    default:
        gpio_set_level(CONFIG_WATER_LEVEL_GPIO_LED, 0);
        break;
    }
}
#endif

static void update_sensor_status(sensor_status_t *sensor_status)
{
    int oldest = (sample_head - sample_count + MAX_SAMPLES) % MAX_SAMPLES;
    int newest = (sample_head - 1 + MAX_SAMPLES) % MAX_SAMPLES;
    double capacity = (compute_tank_capacity(newest) + compute_tank_capacity(oldest)) / 2;

    if (capacity < 0 || capacity > 100) {
        sensor_status->flags |= SENSOR_STATUS_CODE_CALIBRATION_REQUIRED;
#ifdef CONFIG_WATER_LEVEL_ENABLE_GPIO_LED_STATUS
        led_set_mode(LED_MODE_OFF);
#endif
    }
    else if (capacity >= s_max_level_value) {
        sensor_status->flags |= SENSOR_STATUS_CODE_FULL_CAPACITY;
#ifdef CONFIG_WATER_LEVEL_ENABLE_GPIO_LED_STATUS
        led_set_mode(LED_MODE_FULL);
#endif
    }
    else if (capacity <= s_low_level && capacity > s_critical_level) {
        sensor_status->flags |= SENSOR_STATUS_CODE_LOW_STORAGE;
#ifdef CONFIG_WATER_LEVEL_ENABLE_GPIO_LED_STATUS
        led_set_mode(LED_MODE_WARN_LOW);
#endif
    }
    else if (capacity <= 30) {
        sensor_status->flags |= SENSOR_STATUS_CODE_CRITICAL_STORAGE;
#ifdef CONFIG_WATER_LEVEL_ENABLE_GPIO_LED_STATUS
        led_set_mode(LED_MODE_WARN_CRITICAL);
#endif
    }
    else {
#ifdef CONFIG_WATER_LEVEL_ENABLE_GPIO_LED_STATUS
        led_set_mode(LED_MODE_OFF);
#endif
    }

    if (sensor_status->main_status != SENSOR_STATUS_CODE_OPERATIONAL) {
#ifdef CONFIG_WATER_LEVEL_ENABLE_GPIO_LED_STATUS
        led_set_mode(LED_MODE_OFF);
#endif
    }

    g_sensor_status.main_status = sensor_status->main_status;
    g_sensor_status.flags = sensor_status->flags;
}

void sensor_status_from_int(uint32_t code, sensor_status_t *st)
{
    st->main_status = (code >> 16) & 0xFFFF;
    st->flags       = code & 0xFFFF;
}

static const char* sensor_status_to_string(uint32_t code)
{
    static char buf[128];
    buf[0] = '\0';

    sensor_status_t st;
    sensor_status_from_int(code, &st);

    bool first = true;
    #define ADD(str) do {                       \
        if (!first) strcat(buf, " | ");         \
        strcat(buf, str);                       \
        first = false;                          \
    } while (0)

    switch (st.main_status) {
        case SENSOR_STATUS_CODE_OPERATIONAL:
            ADD("Operational");
            break;
        case SENSOR_STATUS_CODE_ERROR:
            ADD("Sensor error");
            break;
        case SENSOR_STATUS_CODE_NO_RESPONSE:
            ADD("No Response");
            break;
        default:
            ADD("Unknown");
            break;
    }

    if (st.flags & SENSOR_STATUS_CODE_FULL_CAPACITY)
        ADD("Full Capacity");

    if (st.flags & SENSOR_STATUS_CODE_LOW_STORAGE)
        ADD("Low Storage");

    if (st.flags & SENSOR_STATUS_CODE_CRITICAL_STORAGE)
        ADD("Critical Storage");

    if (st.flags & SENSOR_STATUS_CODE_CALIBRATION_REQUIRED)
        ADD("Calibration required");

    if (first) {
        strcpy(buf, "Unknown");
    }

    return buf;
}

static esp_err_t water_level_write_cb(
    const esp_rmaker_device_t *device,
    const esp_rmaker_param_t *param,
    const esp_rmaker_param_val_t val,
    void *priv_data,
    esp_rmaker_write_ctx_t *ctx)
{
    if (param == level_param) {
        s_max_level_value = val.val.i;
        ESP_LOGI("LEVEL", "Max capacity updated: %d", s_max_level_value);
    } else if (param == calib_tank_height_param) {
        s_max_tank_height_cm = val.val.i;
        ESP_LOGI("LEVEL", "Max tank height updated: %d", s_max_tank_height_cm);
    } else if (param == calib_tank_base_param) {
        if (s_max_tank_height_cm - val.val.i <= 0) {
            ESP_LOGE("LEVEL", "Invalid tank parameters");
            return ESP_FAIL;
        }
        s_tank_base_cm = val.val.i;
        ESP_LOGI("LEVEL", "Tank base updated: %d", s_tank_base_cm);
    } else if (param == calib_tank_volume_param) {
        s_max_tank_volume = val.val.i;
        ESP_LOGI("LEVEL", "Max tank volume updated: %d", s_max_tank_volume);
    } else if (param == calib_sensor_ang_deg_param) {
        s_sensor_tilt = val.val.i;
        ESP_LOGI("LEVEL", "Sensor tilt updated: %d", s_sensor_tilt);
    }

    if (param != level_param) {
        sensor_publish();
    }

    esp_rmaker_param_update_and_report(param, val);
    return ESP_OK;
}

#ifdef CONFIG_WATER_LEVEL_ENABLE_GPIO_LED_STATUS
static void led_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(50);

    for (;;) {
        led_update();
        vTaskDelay(period);
    }
}

static void led_driver_init(void)
{
    gpio_reset_pin(CONFIG_WATER_LEVEL_GPIO_LED);
    gpio_set_direction(CONFIG_WATER_LEVEL_GPIO_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_WATER_LEVEL_GPIO_LED, 0);

    xTaskCreate(led_task, "led_task", 2048, NULL, 1, NULL);
}
#endif

float smooth_level(float level_cm)
{
    const float alpha = 0.2f;  // 0..1 (lower == smoother)

    if (!level_ema_init) {
        level_ema = level_cm;
        level_ema_init = true;
        return level_cm;
    }

    level_ema = alpha * level_cm + (1.0f - alpha) * level_ema;
    return level_ema;
}

uint32_t sensor_status_to_int(sensor_status_t *st)
{
    uint32_t code = 0;
    code |= ((uint32_t)st->main_status << 16);
    code |= (uint32_t)(st->flags & 0xFFFF);
    return code;
}

sensor_status_t read_and_log_sensor(double* measurement)
{       
    sensor_status_t status = SENSOR_STATUS_DEFAULT();
    mjd_jsnsr04t_data_t jsnsr04t_data = MJD_JSNSR04T_DATA_DEFAULT();
    esp_err_t f_retval = mjd_jsnsr04t_get_measurement(&jsnsr04t_config, &jsnsr04t_data);
    if (f_retval != ESP_OK) {
        status.main_status = SENSOR_STATUS_CODE_ERROR;
        ESP_LOGE(TAG, "[sensor_driver] mjd_jsnsr04t_get_measurement() failed | err %i (%s)", f_retval, esp_err_to_name(f_retval));
    }

    if (!jsnsr04t_data.is_an_error) {
        status.main_status = SENSOR_STATUS_CODE_OPERATIONAL;

#ifdef CONFIG_WATER_LEVEL_ENABLE_GPIO_LED_STATUS
        led_success_blink();
#endif
        *measurement = smooth_level(jsnsr04t_data.distance_cm);
    }

    ESP_ERROR_CHECK(mjd_jsnsr04t_log_data(jsnsr04t_data));
    return status;
}

static void sensor_store_sample(double distance)
{
    samples[sample_head].timestamp_us = esp_timer_get_time();
    samples[sample_head].value = distance;

    sample_head = (sample_head + 1) % MAX_SAMPLES;

    if (sample_count < MAX_SAMPLES) {
        sample_count++;
    }
}

static inline double corrected_distance(double distance)
{
    return distance * cos(s_sensor_tilt * M_PI / 180.0);
}

static double compute_tank_capacity(int dt)
{
    double raw_distance = samples[dt].value;
    double d = corrected_distance(raw_distance);

    // Clamp to physical limits
    if (d > s_max_tank_height_cm)
        d = s_max_tank_height_cm;
    if (d < s_tank_base_cm)
        d = s_tank_base_cm;

    double usable_height = s_max_tank_height_cm - s_tank_base_cm;
    double water_level = d - s_tank_base_cm;

    if (usable_height == 0.0) {
        return -1;
    }

    double fill_ratio = 1.0 - (water_level / usable_height);
    return fill_ratio * 100.0;
}

static double compute_tank_volume(int dt)
{
    return (compute_tank_capacity(dt) / 100.0f) * s_max_tank_volume;
}

static float compute_flow(void)
{
    if (sample_count < 2) {
        return 0.0f;
    }

    int oldest = (sample_head - sample_count + MAX_SAMPLES) % MAX_SAMPLES;
    int newest = (sample_head - 1 + MAX_SAMPLES) % MAX_SAMPLES;

    double dV = compute_tank_volume( newest ) - compute_tank_volume( oldest );
    double dT = (samples[newest].timestamp_us - samples[oldest].timestamp_us) / 1e6f / 60.0f;

    if (dT <= 0.0f) {
        return 0.0f;
    }

    return dV / dT;
}

static void sensor_update()
{
    double value = g_latest_measurement;
    sensor_status_t status = read_and_log_sensor(&value);

    if (status.main_status == SENSOR_STATUS_CODE_OPERATIONAL) {
        g_latest_measurement = value;
    }

    sensor_store_sample(value);
    update_sensor_status(&status);

    ESP_LOGI(TAG, "[sensor_driver] Sensor update: %.2f cm (status=%d)",
             value, status.main_status);
}

static void sensor_publish(void)
{
    int dt1 = (sample_head - sample_count + MAX_SAMPLES) % MAX_SAMPLES;
    int dt2 = (sample_head - 1 + MAX_SAMPLES) % MAX_SAMPLES;

    double capacity_avg = (compute_tank_capacity(dt2) + compute_tank_capacity(dt1)) / 2.0f;
    esp_rmaker_param_update_and_report(
        water_tank_capacity_param,
        esp_rmaker_int((int)capacity_avg));

    double distance = samples[dt2].value;
    esp_rmaker_param_update_and_report(
        water_tank_sensor_raw_param,
        esp_rmaker_float(distance));

    double volume_avg = (compute_tank_volume(dt2) + compute_tank_volume(dt1)) / 2.0f;
    esp_rmaker_param_update_and_report(
        water_tank_volume_param,
        esp_rmaker_float(volume_avg));

    sensor_status_t status = g_sensor_status;
    uint32_t status_code = sensor_status_to_int(&status);
    esp_rmaker_param_update_and_report(
        sensor_status_msg_param,
        esp_rmaker_str(sensor_status_to_string(status_code)));

    esp_rmaker_param_update_and_report(
        sensor_status_int_param,
        esp_rmaker_int(status_code));

    flow_rate_lpm = compute_flow();
    esp_rmaker_param_update_and_report(
        flow_rate_param,
        esp_rmaker_float(flow_rate_lpm)
    );

    ESP_LOGI(TAG, "[sensor_driver] Publishing data to the server");
}

static void ultrasonic_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        sensor_update();
    }
}

static void publisher_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        sensor_publish();
    }
}

static void water_level_timer_cb(void *arg)
{
    if (ultrasonic_task_handle) {
        xTaskNotifyGive(ultrasonic_task_handle);
    }
}

static void sensor_pub_timer_cb(void *arg)
{
    if (publisher_task_handle) {
        xTaskNotifyGive(publisher_task_handle);
    }
}

static void ultrasonic_sensor_init(void)
{    
    jsnsr04t_config = MJD_JSNSR04T_CONFIG_DEFAULT();
    jsnsr04t_config.trigger_gpio_num = CONFIG_WATER_LEVEL_GPIO_JSNSR04T_TRIGGER;
    jsnsr04t_config.echo_gpio_num = CONFIG_WATER_LEVEL_GPIO_JSNSR04T_ECHO;
    jsnsr04t_config.rmt_channel = RMT_CHANNEL_0;
    jsnsr04t_config.distance_sensor_to_artifact_cm = 10.0;
    jsnsr04t_config.nbr_of_samples = 10;
    jsnsr04t_config.max_range_allowed_in_samples_cm = 10.0;

    esp_err_t f_retval = mjd_jsnsr04t_init(&jsnsr04t_config);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "[sensor_driver] mjd_jsnsr04t_init() err %i (%s)", f_retval, esp_err_to_name(f_retval));
    }

    mjd_jsnsr04t_log_config(jsnsr04t_config);
}

static void IRAM_ATTR boot_button_isr_handler(void *arg)
{
    uint32_t evt = 1;
    xQueueSendFromISR(boot_evt_queue, &evt, NULL);
}

static void boot_button_task(void *arg)
{
    uint32_t evt;

    for (;;) {
        if (xQueueReceive(boot_evt_queue, &evt, portMAX_DELAY)) {
            if (boot_user_callback) {
                boot_user_callback();
            }
        }
    }
}

void on_button_pressed(void)
{
    ESP_LOGI(TAG, "[sensor_driver] Requested sensor read");
    sensor_update();

    if (g_sensor_status.main_status == SENSOR_STATUS_CODE_OPERATIONAL) {
        sensor_publish();
    }
}

void boot_button_init(void (*callback)(void))
{   
    boot_user_callback = callback;
    boot_evt_queue = xQueueCreate(4, sizeof(uint32_t));
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << CONFIG_WATER_LEVEL_BOARD_BUTTON_GPIO,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };

    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(CONFIG_WATER_LEVEL_BOARD_BUTTON_GPIO, boot_button_isr_handler, NULL);

    xTaskCreate(boot_button_task, "boot_button_task", 4096, NULL, 10, NULL);
}

void app_main()
{
    esp_rmaker_console_init();
    ultrasonic_sensor_init();

#ifdef CONFIG_WATER_LEVEL_ENABLE_GPIO_LED_STATUS
    led_driver_init();
    led_success_blink();
#endif

    /* Initialize NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    g_sensor_status = SENSOR_STATUS_DEFAULT();
    
    ESP_ERROR_CHECK( err );
    app_network_init();

    /* Register an event handler to catch RainMaker events */
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_COMMON_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(APP_NETWORK_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_OTA_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    /* Initialize the ESP RainMaker Agent.
     * Note that this should be called after app_network_init() but before app_nenetworkk_start()
     * */
    esp_rmaker_config_t rainmaker_cfg = { .enable_time_sync = false };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Device", "Water level");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    water_tank_level_device = esp_rmaker_device_create("Water Level", ESP_RMAKER_DEVICE_WATER_TANK_SENSOR, NULL);
    water_tank_capacity_param = esp_rmaker_param_create(
        "Tank Capacity",
        ESP_RMAKER_DEVICE_WATER_TANK_CAPACITY,
        esp_rmaker_int(0),
        PROP_FLAG_READ | PROP_FLAG_PERSIST);

    water_tank_sensor_raw_param = esp_rmaker_param_create(
        "Sensor raw (cm)",
        ESP_RMAKER_DEVICE_WATER_TANK_SENSOR_RAW,
        esp_rmaker_float(0),
        PROP_FLAG_READ | PROP_FLAG_PERSIST | PROP_FLAG_TIME_SERIES);

    level_param = esp_rmaker_param_create(
        "Full capacity adjust",
        ESP_RMAKER_DEVICE_WATER_TANK_CONFIG_LEVEL,
        esp_rmaker_int(s_max_level_value),
        PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);

    calib_tank_height_param = esp_rmaker_param_create(
        "Tank height (cm)",
        ESP_RMAKER_DEVICE_WATER_TANK_CONFIG_TANK_HEIGHT,
        esp_rmaker_int(s_max_tank_height_cm),
        PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);

    calib_tank_base_param = esp_rmaker_param_create(
        "Tank base (cm)",
        ESP_RMAKER_DEVICE_WATER_TANK_CONFIG_TANK_HEIGHT,
        esp_rmaker_int(s_tank_base_cm),
        PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);

    calib_tank_volume_param = esp_rmaker_param_create(
        "Maximum tank volume (L)",
        ESP_RMAKER_DEVICE_WATER_TANK_CONFIG_VOLUME,
        esp_rmaker_int(s_max_tank_volume),
        PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);

    calib_sensor_ang_deg_param = esp_rmaker_param_create(
        "Sensor tilt (deg)",
        ESP_RMAKER_DEVICE_WATER_TANK_CONFIG_SENSOR_TILT,
        esp_rmaker_int(s_sensor_tilt),
        PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);

    flow_rate_param = esp_rmaker_param_create(
        "Flow Rate (L/min)",
        ESP_RMAKER_DEVICE_WATER_TANK_SENSOR_FLOW_RATE,
        esp_rmaker_float(0.0),
        PROP_FLAG_READ | PROP_FLAG_PERSIST | PROP_FLAG_TIME_SERIES);

    sensor_status_msg_param = esp_rmaker_param_create(
        "Status", 
        ESP_RMAKER_DEVICE_WATER_TANK_STATUS_MSG,
        esp_rmaker_str("Sensor error"), 
        PROP_FLAG_READ | PROP_FLAG_PERSIST);

    sensor_status_int_param = esp_rmaker_param_create(
        "Status code",
        ESP_RMAKER_DEVICE_WATER_TANK_STATUS_CODE,
        esp_rmaker_int(SENSOR_STATUS_CODE_NO_RESPONSE),
        PROP_FLAG_READ | PROP_FLAG_PERSIST);

    water_tank_volume_param = esp_rmaker_param_create(
        "Tank volume",
        ESP_RMAKER_DEVICE_WATER_TANK_SENSOR_VOLUME,
        esp_rmaker_float(0),
        PROP_FLAG_READ | PROP_FLAG_PERSIST | PROP_FLAG_TIME_SERIES);

    esp_rmaker_param_add_ui_type( level_param, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_ui_type( water_tank_capacity_param, ESP_RMAKER_UI_TEXT );
    esp_rmaker_param_add_ui_type( water_tank_sensor_raw_param, ESP_RMAKER_UI_TEXT );
    esp_rmaker_param_add_ui_type( water_tank_volume_param, ESP_RMAKER_UI_TEXT );
    esp_rmaker_param_add_ui_type( calib_tank_height_param, ESP_RMAKER_UI_TEXT );
    esp_rmaker_param_add_ui_type( calib_tank_base_param, ESP_RMAKER_UI_TEXT );
    esp_rmaker_param_add_ui_type( calib_tank_volume_param, ESP_RMAKER_UI_TEXT );
    esp_rmaker_param_add_ui_type( calib_sensor_ang_deg_param, ESP_RMAKER_UI_TEXT );
    esp_rmaker_param_add_ui_type( sensor_status_msg_param, ESP_RMAKER_UI_TEXT );
    esp_rmaker_param_add_ui_type( sensor_status_int_param, ESP_RMAKER_UI_TEXT );

    esp_rmaker_param_add_bounds(
        level_param,
        esp_rmaker_int(0),
        esp_rmaker_int(100),
        esp_rmaker_int(1)
    );
    
    esp_rmaker_device_assign_primary_param(
        water_tank_level_device, 
        water_tank_capacity_param);

    xTaskCreate(ultrasonic_task, "ultrasonic_task", 4096, NULL, 5, &ultrasonic_task_handle);
    xTaskCreate(publisher_task, "publisher_task", 4096, NULL, 5, &publisher_task_handle);

    esp_timer_create_args_t timer_args = {
        .callback = water_level_timer_cb,
        .name = "water_level_timer"
    };

    esp_timer_create_args_t timer_pub_args = {
        .callback = sensor_pub_timer_cb,
        .name = "publisher_timer"
    };

    uint32_t thread_period_m = CONFIG_WATER_LEVEL_UPDATE_PERIOD_MINUTES;
    uint64_t thread_period_us = (uint64_t)thread_period_m * 60ULL * 1000000ULL;
    uint64_t thread_update_us = (uint64_t)((CONFIG_WATER_LEVEL_UPDATE_PERIOD_MINUTES * 60ULL) / 10 * 1000000ULL);

    boot_button_init(on_button_pressed);

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &water_level_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(water_level_timer, thread_update_us));
    ESP_ERROR_CHECK(esp_timer_create(&timer_pub_args, &sensor_pub_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(sensor_pub_timer, thread_period_us));
    
    esp_rmaker_device_add_param(water_tank_level_device, sensor_status_msg_param);
    esp_rmaker_device_add_param(water_tank_level_device, water_tank_capacity_param);
    esp_rmaker_device_add_param(water_tank_level_device, water_tank_volume_param);
    esp_rmaker_device_add_param(water_tank_level_device, water_tank_sensor_raw_param);
    esp_rmaker_device_add_param(water_tank_level_device, flow_rate_param);
    esp_rmaker_device_add_param(water_tank_level_device, calib_tank_height_param);
    esp_rmaker_device_add_param(water_tank_level_device, calib_tank_base_param);
    esp_rmaker_device_add_param(water_tank_level_device, calib_tank_volume_param);
    esp_rmaker_device_add_param(water_tank_level_device, calib_sensor_ang_deg_param);
    esp_rmaker_device_add_param(water_tank_level_device, level_param);
    esp_rmaker_device_add_param(water_tank_level_device, sensor_status_int_param);

    esp_rmaker_device_add_cb(water_tank_level_device, water_level_write_cb, NULL);
    esp_rmaker_node_add_device(node, water_tank_level_device);

    esp_rmaker_ota_enable_default();
    esp_rmaker_timezone_service_enable();
    esp_rmaker_schedule_enable();
    esp_rmaker_scenes_enable();
    app_insights_enable();

    esp_rmaker_start();

    err = app_network_set_custom_mfg_data(MGF_DATA_DEVICE_TYPE_SWITCH, MFG_DATA_DEVICE_SUBTYPE_SWITCH);
    err = app_network_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wifi. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }
}
