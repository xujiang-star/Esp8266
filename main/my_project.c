#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include "nvs_flash.h"

/*
 * ESP32-S3 pin map
 *
 * Curtain motors: two 28BYJ-48 motors driven by separate ULN2003 boards.
 *   Left ULN2003 IN1..IN4: GPIO15, GPIO16, GPIO17, GPIO18
 *   Right ULN2003 IN1..IN4: GPIO4, GPIO5, GPIO6, GPIO7
 *
 * Window servos:
 *   Left servo PWM: GPIO11
 *   Right servo PWM: GPIO12
 *
 * Sensors:
 *   Flame sensor DO: GPIO13, active low
 *   DHT11/DHT22 data: GPIO14
 *   CO analog output: GPIO1 (ADC1 channel 0)
 *   CH4 analog output: GPIO2 (ADC1 channel 1)
 *
 * Use a common 5 V supply for motors/servos and connect its GND to ESP32 GND.
 * MQ sensor analog outputs must be divided to 3.3 V or below before reaching
 * the ESP32 ADC pins.
 */
#define CURTAIN_LEFT_IN1 GPIO_NUM_15
#define CURTAIN_LEFT_IN2 GPIO_NUM_16
#define CURTAIN_LEFT_IN3 GPIO_NUM_17
#define CURTAIN_LEFT_IN4 GPIO_NUM_18

#define CURTAIN_RIGHT_IN1 GPIO_NUM_4
#define CURTAIN_RIGHT_IN2 GPIO_NUM_5
#define CURTAIN_RIGHT_IN3 GPIO_NUM_6
#define CURTAIN_RIGHT_IN4 GPIO_NUM_7
#define CURTAIN_RIGHT_MOTOR_REVERSED true

#define WINDOW_LEFT_SERVO_GPIO  GPIO_NUM_11
#define WINDOW_RIGHT_SERVO_GPIO GPIO_NUM_12

#define FLAME_SENSOR_GPIO         GPIO_NUM_13
#define FLAME_SENSOR_ACTIVE_LEVEL 0
#define DHT_SENSOR_GPIO           GPIO_NUM_14
#define DHT_SENSOR_IS_DHT22       true

#define CO_SENSOR_ADC_CHANNEL  ADC_CHANNEL_0
#define CH4_SENSOR_ADC_CHANNEL ADC_CHANNEL_1

#define STEPPER_STEPS_PER_REV 4096U
#define STEPPER_PHASE_COUNT   8U
#define STEPPER_QUEUE_LENGTH  4U
#define STEPPER_TASK_STACK    4096U
#define STEPPER_TASK_PRIORITY 5U
#define STEPPER_MAX_STEPS     (STEPPER_STEPS_PER_REV * 100U)
#define STEPPER_MAX_RPM       15.0f

/*
 * Set this to the calibrated number of motor half-steps from the fully closed
 * to fully open curtain position. The initial state assumes both curtains are
 * mechanically closed before the ESP32 is powered.
 */
#define CURTAIN_TRAVEL_STEPS (STEPPER_STEPS_PER_REV * 2U)
#define CURTAIN_MOVE_RPM     10.0f

#define SERVO_PWM_FREQUENCY_HZ 50U
#define SERVO_DUTY_RESOLUTION  LEDC_TIMER_14_BIT
#define SERVO_DUTY_SCALE       (1U << 14U)
#define SERVO_MIN_PULSE_US     500U
#define SERVO_MAX_PULSE_US     2500U

/*
 * Adjust these four angles after installing the linkage. The opposite angles
 * account for the two servos being mounted as mirror images.
 */
#define WINDOW_LEFT_CLOSED_ANGLE  20U
#define WINDOW_LEFT_OPEN_ANGLE    100U
#define WINDOW_RIGHT_CLOSED_ANGLE 160U
#define WINDOW_RIGHT_OPEN_ANGLE   80U

/*
 * Gas thresholds are ADC voltages, not ppm. Calibrate these values for the
 * installed MQ-7 (CO) and MQ-4 (CH4) modules before deploying the device.
 */
#define HUMIDITY_ALARM_THRESHOLD_PERCENT 75.0f
#define CO_ALARM_THRESHOLD_MV             1800
#define CH4_ALARM_THRESHOLD_MV            1800
#define GAS_SENSOR_SAMPLE_COUNT           8U

#if DHT_SENSOR_IS_DHT22
#define DHT_START_LOW_TIME_MS 2U
#else
#define DHT_START_LOW_TIME_MS 18U
#endif

#define WIFI_SSID     "Xiaomi"
#define WIFI_PASSWORD "666888pp"

/*
 * Existing topic names are kept so current MQTT subscribers and controllers
 * continue to work. Sensor payloads now contain real environmental readings.
 */
#define MQTT_BROKER_URI       "mqtt://10.249.155.154:1883"
#define MQTT_CLIENT_ID_PREFIX "esp32-28byj48-"
#define MQTT_HEARTBEAT_TOPIC  "stepper/heartbeat"
#define MQTT_SENSOR_TOPIC     "stepper/sensor"
#define MQTT_CONTROL_TOPIC    "stepper/control"
#define MQTT_STATUS_TOPIC     "stepper/status"
#define MQTT_CONTROL_MAX_LEN  256U
#define HEARTBEAT_INTERVAL_MS 30000U
#define SENSOR_INTERVAL_MS    2000U

static const char *TAG = "smart_window";

typedef enum {
    STEPPER_DIRECTION_CW = 1,
    STEPPER_DIRECTION_CCW = -1,
} stepper_direction_t;

typedef enum {
    CURTAIN_POSITION_UNKNOWN,
    CURTAIN_POSITION_CLOSED,
    CURTAIN_POSITION_OPEN,
} curtain_position_t;

typedef struct {
    uint32_t steps;
    stepper_direction_t direction;
    float rpm;
    curtain_position_t position_after;
    const char *source;
} curtain_command_t;

typedef struct {
    adc_channel_t channel;
    const char *name;
    adc_cali_handle_t calibration;
    bool calibrated;
} analog_sensor_t;

typedef struct {
    bool flame_detected;
    bool humidity_valid;
    float humidity_percent;
    float temperature_c;
    bool co_valid;
    int co_raw;
    int co_mv;
    bool ch4_valid;
    int ch4_raw;
    int ch4_mv;
    bool humidity_alarm;
    bool co_alarm;
    bool ch4_alarm;
    bool alarm_active;
} sensor_readings_t;

static const uint8_t s_half_step_sequence[STEPPER_PHASE_COUNT] = {
    0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09,
};

static const gpio_num_t s_left_stepper_pins[] = {
    CURTAIN_LEFT_IN1,
    CURTAIN_LEFT_IN2,
    CURTAIN_LEFT_IN3,
    CURTAIN_LEFT_IN4,
};

static const gpio_num_t s_right_stepper_pins[] = {
    CURTAIN_RIGHT_IN1,
    CURTAIN_RIGHT_IN2,
    CURTAIN_RIGHT_IN3,
    CURTAIN_RIGHT_IN4,
};

static QueueHandle_t s_curtain_command_queue;
static TaskHandle_t s_curtain_task_handle;
static esp_timer_handle_t s_step_timer;
static esp_mqtt_client_handle_t s_mqtt_client;
static adc_oneshot_unit_handle_t s_adc_unit;

static analog_sensor_t s_co_sensor = {
    .channel = CO_SENSOR_ADC_CHANNEL,
    .name = "CO",
};

static analog_sensor_t s_ch4_sensor = {
    .channel = CH4_SENSOR_ADC_CHANNEL,
    .name = "CH4",
};

static volatile bool s_curtain_busy;
static volatile bool s_mqtt_connected;
static volatile bool s_window_open;
static volatile bool s_auto_control_enabled = true;
static volatile bool s_safety_alarm_active;
static volatile curtain_position_t s_curtain_position = CURTAIN_POSITION_CLOSED;
static volatile curtain_position_t s_curtain_target = CURTAIN_POSITION_CLOSED;
static uint8_t s_left_step_index;
static uint8_t s_right_step_index;
static char s_mqtt_client_id[32];
static portMUX_TYPE s_curtain_state_lock = portMUX_INITIALIZER_UNLOCKED;

static void mqtt_publish_status(const char *event, const char *status, const char *reason);

static const char *curtain_position_to_string(curtain_position_t position)
{
    switch (position) {
    case CURTAIN_POSITION_CLOSED:
        return "closed";
    case CURTAIN_POSITION_OPEN:
        return "open";
    default:
        return "unknown";
    }
}

static curtain_position_t curtain_get_position(void)
{
    curtain_position_t position;

    portENTER_CRITICAL(&s_curtain_state_lock);
    position = s_curtain_position;
    portEXIT_CRITICAL(&s_curtain_state_lock);
    return position;
}

static curtain_position_t curtain_get_target(void)
{
    curtain_position_t target;

    portENTER_CRITICAL(&s_curtain_state_lock);
    target = s_curtain_target;
    portEXIT_CRITICAL(&s_curtain_state_lock);
    return target;
}

static void curtain_set_position(curtain_position_t position)
{
    portENTER_CRITICAL(&s_curtain_state_lock);
    s_curtain_position = position;
    portEXIT_CRITICAL(&s_curtain_state_lock);
}

static esp_err_t stepper_write_coils(const gpio_num_t pins[4], uint8_t phase)
{
    for (size_t pin_index = 0; pin_index < 4U; ++pin_index) {
        const esp_err_t result = gpio_set_level(pins[pin_index], (phase >> pin_index) & 0x01U);
        if (result != ESP_OK) {
            return result;
        }
    }

    return ESP_OK;
}

static esp_err_t curtain_write_coils(uint8_t left_phase, uint8_t right_phase)
{
    esp_err_t result = stepper_write_coils(s_left_stepper_pins, left_phase);
    if (result != ESP_OK) {
        return result;
    }
    return stepper_write_coils(s_right_stepper_pins, right_phase);
}

static void curtain_release(void)
{
    const esp_err_t left_result = stepper_write_coils(s_left_stepper_pins, 0);
    const esp_err_t right_result = stepper_write_coils(s_right_stepper_pins, 0);

    if (left_result != ESP_OK || right_result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to release curtain motors: left=%s right=%s",
                 esp_err_to_name(left_result), esp_err_to_name(right_result));
    }
}

static uint8_t stepper_next_index(uint8_t index, stepper_direction_t direction, bool reversed)
{
    int increment = direction == STEPPER_DIRECTION_CW ? 1 : -1;
    if (reversed) {
        increment = -increment;
    }

    int next_index = (int)index + increment;
    if (next_index < 0) {
        next_index += STEPPER_PHASE_COUNT;
    } else if (next_index >= STEPPER_PHASE_COUNT) {
        next_index -= STEPPER_PHASE_COUNT;
    }

    return (uint8_t)next_index;
}

static uint32_t stepper_delay_us(float rpm)
{
    const float steps_per_second = (rpm * STEPPER_STEPS_PER_REV) / 60.0f;
    return (uint32_t)(1000000.0f / steps_per_second);
}

static void stepper_timer_callback(void *argument)
{
    (void)argument;
    xTaskNotifyGive(s_curtain_task_handle);
}

static esp_err_t curtain_move(const curtain_command_t *command)
{
    const uint32_t delay_us = stepper_delay_us(command->rpm);

    for (uint32_t step = 0; step < command->steps; ++step) {
        s_left_step_index = stepper_next_index(s_left_step_index, command->direction, false);
        s_right_step_index = stepper_next_index(s_right_step_index, command->direction,
                                                CURTAIN_RIGHT_MOTOR_REVERSED);

        const esp_err_t coil_result = curtain_write_coils(
            s_half_step_sequence[s_left_step_index], s_half_step_sequence[s_right_step_index]);
        if (coil_result != ESP_OK) {
            curtain_release();
            return coil_result;
        }

        if (step + 1U < command->steps) {
            const esp_err_t timer_result = esp_timer_start_once(s_step_timer, delay_us);
            if (timer_result != ESP_OK) {
                curtain_release();
                return timer_result;
            }
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
    }

    curtain_release();
    return ESP_OK;
}

static void curtain_task(void *argument)
{
    (void)argument;
    curtain_command_t command;

    while (true) {
        if (xQueueReceive(s_curtain_command_queue, &command, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        s_curtain_busy = true;
        const esp_err_t result = curtain_move(&command);
        s_curtain_busy = false;

        if (result == ESP_OK) {
            curtain_set_position(command.position_after);
            ESP_LOGI(TAG, "Curtain move completed from %s", command.source);
            mqtt_publish_status("curtain", "completed", command.source);
        } else {
            if (command.position_after != CURTAIN_POSITION_UNKNOWN) {
                portENTER_CRITICAL(&s_curtain_state_lock);
                if (s_curtain_target == command.position_after) {
                    s_curtain_target = CURTAIN_POSITION_UNKNOWN;
                }
                portEXIT_CRITICAL(&s_curtain_state_lock);
            }
            ESP_LOGE(TAG, "Curtain move from %s failed: %s", command.source,
                     esp_err_to_name(result));
            mqtt_publish_status("curtain", "failed", command.source);
        }
    }
}

static esp_err_t curtain_init(void)
{
    uint64_t pin_mask = 0;
    for (size_t pin_index = 0; pin_index < 4U; ++pin_index) {
        pin_mask |= 1ULL << s_left_stepper_pins[pin_index];
        pin_mask |= 1ULL << s_right_stepper_pins[pin_index];
    }

    const gpio_config_t config = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "Failed to configure curtain motor GPIOs");
    curtain_release();

    const esp_timer_create_args_t timer_config = {
        .callback = stepper_timer_callback,
        .name = "curtain_step_timer",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_config, &s_step_timer), TAG,
                        "Failed to create curtain step timer");

    s_curtain_command_queue = xQueueCreate(STEPPER_QUEUE_LENGTH, sizeof(curtain_command_t));
    if (s_curtain_command_queue == NULL) {
        esp_timer_delete(s_step_timer);
        s_step_timer = NULL;
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(curtain_task, "curtain_task", STEPPER_TASK_STACK, NULL,
                    STEPPER_TASK_PRIORITY, &s_curtain_task_handle) != pdPASS) {
        vQueueDelete(s_curtain_command_queue);
        s_curtain_command_queue = NULL;
        esp_timer_delete(s_step_timer);
        s_step_timer = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static esp_err_t curtain_enqueue_move(uint32_t steps, stepper_direction_t direction, float rpm,
                                      curtain_position_t position_after, const char *source)
{
    if (s_curtain_command_queue == NULL || steps == 0U || steps > STEPPER_MAX_STEPS ||
        rpm <= 0.0f || rpm > STEPPER_MAX_RPM ||
        (direction != STEPPER_DIRECTION_CW && direction != STEPPER_DIRECTION_CCW)) {
        return ESP_ERR_INVALID_ARG;
    }

    const curtain_command_t command = {
        .steps = steps,
        .direction = direction,
        .rpm = rpm,
        .position_after = position_after,
        .source = source,
    };

    return xQueueSend(s_curtain_command_queue, &command, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t curtain_request_state(bool open, const char *source)
{
    const curtain_position_t target = open ? CURTAIN_POSITION_OPEN : CURTAIN_POSITION_CLOSED;
    curtain_position_t previous_target;

    portENTER_CRITICAL(&s_curtain_state_lock);
    previous_target = s_curtain_target;
    if (previous_target == target) {
        portEXIT_CRITICAL(&s_curtain_state_lock);
        return ESP_OK;
    }
    s_curtain_target = target;
    portEXIT_CRITICAL(&s_curtain_state_lock);

    const stepper_direction_t direction = open ? STEPPER_DIRECTION_CW : STEPPER_DIRECTION_CCW;
    const esp_err_t result = curtain_enqueue_move(CURTAIN_TRAVEL_STEPS, direction,
                                                   CURTAIN_MOVE_RPM, target, source);
    if (result != ESP_OK) {
        portENTER_CRITICAL(&s_curtain_state_lock);
        if (s_curtain_target == target) {
            s_curtain_target = previous_target;
        }
        portEXIT_CRITICAL(&s_curtain_state_lock);
    }

    return result;
}

static esp_err_t curtain_enqueue_manual_move(uint32_t steps, stepper_direction_t direction,
                                             float rpm, const char *source)
{
    curtain_position_t previous_target;

    portENTER_CRITICAL(&s_curtain_state_lock);
    previous_target = s_curtain_target;
    s_curtain_target = CURTAIN_POSITION_UNKNOWN;
    portEXIT_CRITICAL(&s_curtain_state_lock);

    const esp_err_t result = curtain_enqueue_move(steps, direction, rpm, CURTAIN_POSITION_UNKNOWN,
                                                   source);
    if (result != ESP_OK) {
        portENTER_CRITICAL(&s_curtain_state_lock);
        if (s_curtain_target == CURTAIN_POSITION_UNKNOWN) {
            s_curtain_target = previous_target;
        }
        portEXIT_CRITICAL(&s_curtain_state_lock);
    }

    return result;
}

static uint32_t servo_angle_to_duty(uint8_t angle)
{
    const uint32_t pulse_width_us = SERVO_MIN_PULSE_US +
                                    ((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * angle) / 180U;
    return (pulse_width_us * SERVO_PWM_FREQUENCY_HZ * SERVO_DUTY_SCALE) / 1000000U;
}

static esp_err_t window_set_open(bool open)
{
    const uint8_t left_angle = open ? WINDOW_LEFT_OPEN_ANGLE : WINDOW_LEFT_CLOSED_ANGLE;
    const uint8_t right_angle = open ? WINDOW_RIGHT_OPEN_ANGLE : WINDOW_RIGHT_CLOSED_ANGLE;

    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                                      servo_angle_to_duty(left_angle)),
                        TAG, "Failed to set left window servo duty");
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1,
                                      servo_angle_to_duty(right_angle)),
                        TAG, "Failed to set right window servo duty");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0), TAG,
                        "Failed to update left window servo duty");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1), TAG,
                        "Failed to update right window servo duty");

    s_window_open = open;
    ESP_LOGI(TAG, "Window set to %s", open ? "open" : "closed");
    return ESP_OK;
}

static esp_err_t window_init(void)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_DUTY_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = SERVO_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG, "Failed to configure servo timer");

    const ledc_channel_config_t left_channel = {
        .gpio_num = WINDOW_LEFT_SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&left_channel), TAG,
                        "Failed to configure left window servo");

    const ledc_channel_config_t right_channel = {
        .gpio_num = WINDOW_RIGHT_SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&right_channel), TAG,
                        "Failed to configure right window servo");
    return window_set_open(false);
}

static esp_err_t analog_sensor_init(analog_sensor_t *sensor)
{
    const adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc_unit, sensor->channel, &channel_config),
                        TAG, "Failed to configure %s ADC channel", sensor->name);

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    const adc_cali_curve_fitting_config_t calibration_config = {
        .unit_id = ADC_UNIT_1,
        .chan = sensor->channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    const esp_err_t calibration_result = adc_cali_create_scheme_curve_fitting(
        &calibration_config, &sensor->calibration);
    if (calibration_result == ESP_OK) {
        sensor->calibrated = true;
        return ESP_OK;
    }

    ESP_LOGW(TAG, "%s ADC calibration is unavailable: %s; its alarm is disabled",
             sensor->name, esp_err_to_name(calibration_result));
#else
    ESP_LOGW(TAG, "%s ADC calibration scheme is unsupported; its alarm is disabled", sensor->name);
#endif

    sensor->calibrated = false;
    return ESP_OK;
}

static esp_err_t sensors_init(void)
{
    const gpio_config_t flame_config = {
        .pin_bit_mask = 1ULL << FLAME_SENSOR_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&flame_config), TAG, "Failed to configure flame sensor GPIO");

    const gpio_config_t dht_config = {
        .pin_bit_mask = 1ULL << DHT_SENSOR_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&dht_config), TAG, "Failed to configure DHT sensor GPIO");

    const adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&adc_config, &s_adc_unit), TAG,
                        "Failed to initialize ADC unit");
    ESP_RETURN_ON_ERROR(analog_sensor_init(&s_co_sensor), TAG, "Failed to initialize CO sensor");
    ESP_RETURN_ON_ERROR(analog_sensor_init(&s_ch4_sensor), TAG,
                        "Failed to initialize CH4 sensor");
    return ESP_OK;
}

static esp_err_t dht_wait_while_level(int level, uint32_t timeout_us, uint32_t *duration_us)
{
    uint32_t duration = 0;

    while (gpio_get_level(DHT_SENSOR_GPIO) == level) {
        if (duration >= timeout_us) {
            return ESP_ERR_TIMEOUT;
        }
        esp_rom_delay_us(1);
        ++duration;
    }

    if (duration_us != NULL) {
        *duration_us = duration;
    }
    return ESP_OK;
}

static esp_err_t dht_read(float *humidity_percent, float *temperature_c)
{
    uint8_t data[5] = {0};

    ESP_RETURN_ON_ERROR(gpio_set_direction(DHT_SENSOR_GPIO, GPIO_MODE_OUTPUT_OD), TAG,
                        "Failed to start DHT read");
    ESP_RETURN_ON_ERROR(gpio_set_level(DHT_SENSOR_GPIO, 0), TAG, "Failed to drive DHT line low");
    esp_rom_delay_us(DHT_START_LOW_TIME_MS * 1000U);
    ESP_RETURN_ON_ERROR(gpio_set_level(DHT_SENSOR_GPIO, 1), TAG,
                        "Failed to release DHT start signal");
    esp_rom_delay_us(30);
    ESP_RETURN_ON_ERROR(gpio_set_direction(DHT_SENSOR_GPIO, GPIO_MODE_INPUT), TAG,
                        "Failed to switch DHT line to input");

    ESP_RETURN_ON_ERROR(dht_wait_while_level(1, 100, NULL), TAG,
                        "DHT did not acknowledge start signal");
    ESP_RETURN_ON_ERROR(dht_wait_while_level(0, 100, NULL), TAG,
                        "DHT response low pulse timed out");
    ESP_RETURN_ON_ERROR(dht_wait_while_level(1, 100, NULL), TAG,
                        "DHT response high pulse timed out");

    for (size_t bit_index = 0; bit_index < 40U; ++bit_index) {
        uint32_t high_pulse_us;
        ESP_RETURN_ON_ERROR(dht_wait_while_level(0, 70, NULL), TAG,
                            "DHT data low pulse timed out");
        ESP_RETURN_ON_ERROR(dht_wait_while_level(1, 100, &high_pulse_us), TAG,
                            "DHT data high pulse timed out");

        data[bit_index / 8U] <<= 1U;
        if (high_pulse_us > 50U) {
            data[bit_index / 8U] |= 0x01U;
        }
    }

    const uint8_t expected_checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (data[4] != expected_checksum) {
        ESP_LOGW(TAG, "DHT checksum mismatch: received=%u expected=%u", data[4], expected_checksum);
        return ESP_ERR_INVALID_CRC;
    }

#if DHT_SENSOR_IS_DHT22
    *humidity_percent = ((uint16_t)data[0] << 8U | data[1]) / 10.0f;
    const uint16_t temperature_raw = ((uint16_t)(data[2] & 0x7FU) << 8U) | data[3];
    *temperature_c = temperature_raw / 10.0f;
    if ((data[2] & 0x80U) != 0U) {
        *temperature_c = -*temperature_c;
    }
#else
    *humidity_percent = (float)data[0];
    *temperature_c = (float)data[2];
#endif
    return ESP_OK;
}

static esp_err_t analog_sensor_read(analog_sensor_t *sensor, int *raw_value, int *millivolts)
{
    if (!sensor->calibrated) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    int64_t raw_sum = 0;
    int64_t millivolt_sum = 0;
    for (uint32_t sample = 0; sample < GAS_SENSOR_SAMPLE_COUNT; ++sample) {
        int raw_sample;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc_unit, sensor->channel, &raw_sample), TAG,
                            "Failed to read %s sensor ADC", sensor->name);

        int millivolt_sample;
        ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(sensor->calibration, raw_sample,
                                                    &millivolt_sample),
                            TAG, "Failed to calibrate %s sensor ADC", sensor->name);
        raw_sum += raw_sample;
        millivolt_sum += millivolt_sample;
    }

    *raw_value = (int)(raw_sum / GAS_SENSOR_SAMPLE_COUNT);
    *millivolts = (int)(millivolt_sum / GAS_SENSOR_SAMPLE_COUNT);
    return ESP_OK;
}

static void sensors_read(sensor_readings_t *readings)
{
    memset(readings, 0, sizeof(*readings));
    readings->co_raw = -1;
    readings->co_mv = -1;
    readings->ch4_raw = -1;
    readings->ch4_mv = -1;
    readings->flame_detected = gpio_get_level(FLAME_SENSOR_GPIO) == FLAME_SENSOR_ACTIVE_LEVEL;

    const esp_err_t dht_result = dht_read(&readings->humidity_percent, &readings->temperature_c);
    if (dht_result == ESP_OK) {
        readings->humidity_valid = true;
    } else {
        ESP_LOGW(TAG, "DHT reading unavailable: %s", esp_err_to_name(dht_result));
    }

    const esp_err_t co_result = analog_sensor_read(&s_co_sensor, &readings->co_raw,
                                                   &readings->co_mv);
    if (co_result == ESP_OK) {
        readings->co_valid = true;
    } else if (co_result != ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "CO reading unavailable: %s", esp_err_to_name(co_result));
    }

    const esp_err_t ch4_result = analog_sensor_read(&s_ch4_sensor, &readings->ch4_raw,
                                                    &readings->ch4_mv);
    if (ch4_result == ESP_OK) {
        readings->ch4_valid = true;
    } else if (ch4_result != ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "CH4 reading unavailable: %s", esp_err_to_name(ch4_result));
    }

    readings->humidity_alarm = readings->humidity_valid &&
                               readings->humidity_percent >= HUMIDITY_ALARM_THRESHOLD_PERCENT;
    readings->co_alarm = readings->co_valid && readings->co_mv >= CO_ALARM_THRESHOLD_MV;
    readings->ch4_alarm = readings->ch4_valid && readings->ch4_mv >= CH4_ALARM_THRESHOLD_MV;
    readings->alarm_active = readings->flame_detected || readings->humidity_alarm ||
                             readings->co_alarm || readings->ch4_alarm;
}

static bool json_add_number_or_null(cJSON *message, const char *name, bool valid, double value)
{
    return valid ? cJSON_AddNumberToObject(message, name, value) != NULL
                 : cJSON_AddNullToObject(message, name) != NULL;
}

static void mqtt_publish_json(const char *topic, cJSON *message, int qos)
{
    if (!s_mqtt_connected || s_mqtt_client == NULL) {
        return;
    }

    char *payload = cJSON_PrintUnformatted(message);
    if (payload == NULL) {
        ESP_LOGE(TAG, "Failed to serialize JSON message");
        return;
    }

    const int message_id = esp_mqtt_client_publish(s_mqtt_client, topic, payload, 0, qos, 0);
    if (message_id < 0) {
        ESP_LOGW(TAG, "Failed to publish on %s", topic);
    }
    cJSON_free(payload);
}

static esp_err_t mqtt_get_device_id(char device_id[18])
{
    uint8_t mac[6];
    ESP_RETURN_ON_ERROR(esp_read_mac(mac, ESP_MAC_WIFI_STA), TAG, "Failed to read Wi-Fi MAC");
    snprintf(device_id, 18, "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return ESP_OK;
}

static void mqtt_publish_heartbeat(void)
{
    char device_id[18];
    if (mqtt_get_device_id(device_id) != ESP_OK) {
        return;
    }

    cJSON *message = cJSON_CreateObject();
    if (message == NULL) {
        ESP_LOGE(TAG, "Failed to create heartbeat JSON");
        return;
    }

    const bool message_valid =
        cJSON_AddStringToObject(message, "type", "heartbeat") != NULL &&
        cJSON_AddStringToObject(message, "device_id", device_id) != NULL &&
        cJSON_AddNumberToObject(message, "uptime_s", esp_timer_get_time() / 1000000) != NULL &&
        cJSON_AddBoolToObject(message, "curtain_busy", s_curtain_busy) != NULL &&
        cJSON_AddBoolToObject(message, "window_open", s_window_open) != NULL &&
        cJSON_AddStringToObject(message, "curtain_position",
                                curtain_position_to_string(curtain_get_position())) != NULL &&
        cJSON_AddStringToObject(message, "curtain_target",
                                curtain_position_to_string(curtain_get_target())) != NULL &&
        cJSON_AddBoolToObject(message, "automatic_control", s_auto_control_enabled) != NULL &&
        cJSON_AddBoolToObject(message, "alarm_active", s_safety_alarm_active) != NULL;
    if (!message_valid) {
        ESP_LOGE(TAG, "Failed to build heartbeat JSON");
        cJSON_Delete(message);
        return;
    }

    mqtt_publish_json(MQTT_HEARTBEAT_TOPIC, message, 1);
    cJSON_Delete(message);
}

static void mqtt_publish_sensor(const sensor_readings_t *readings)
{
    char device_id[18];
    if (mqtt_get_device_id(device_id) != ESP_OK) {
        return;
    }

    cJSON *message = cJSON_CreateObject();
    if (message == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor JSON");
        return;
    }

    const bool message_valid =
        cJSON_AddStringToObject(message, "type", "environment") != NULL &&
        cJSON_AddStringToObject(message, "device_id", device_id) != NULL &&
        cJSON_AddNumberToObject(message, "uptime_s", esp_timer_get_time() / 1000000) != NULL &&
        cJSON_AddBoolToObject(message, "flame_detected", readings->flame_detected) != NULL &&
        cJSON_AddBoolToObject(message, "humidity_valid", readings->humidity_valid) != NULL &&
        json_add_number_or_null(message, "humidity_percent", readings->humidity_valid,
                                readings->humidity_percent) &&
        json_add_number_or_null(message, "temperature_c", readings->humidity_valid,
                                readings->temperature_c) &&
        cJSON_AddBoolToObject(message, "co_valid", readings->co_valid) != NULL &&
        json_add_number_or_null(message, "co_raw", readings->co_valid, readings->co_raw) &&
        json_add_number_or_null(message, "co_voltage_mv", readings->co_valid, readings->co_mv) &&
        cJSON_AddBoolToObject(message, "ch4_valid", readings->ch4_valid) != NULL &&
        json_add_number_or_null(message, "ch4_raw", readings->ch4_valid, readings->ch4_raw) &&
        json_add_number_or_null(message, "ch4_voltage_mv", readings->ch4_valid,
                                readings->ch4_mv) &&
        cJSON_AddBoolToObject(message, "humidity_alarm", readings->humidity_alarm) != NULL &&
        cJSON_AddBoolToObject(message, "co_alarm", readings->co_alarm) != NULL &&
        cJSON_AddBoolToObject(message, "ch4_alarm", readings->ch4_alarm) != NULL &&
        cJSON_AddBoolToObject(message, "alarm_active", readings->alarm_active) != NULL &&
        cJSON_AddBoolToObject(message, "window_open", s_window_open) != NULL &&
        cJSON_AddStringToObject(message, "curtain_position",
                                curtain_position_to_string(curtain_get_position())) != NULL;
    if (!message_valid) {
        ESP_LOGE(TAG, "Failed to build sensor JSON");
        cJSON_Delete(message);
        return;
    }

    mqtt_publish_json(MQTT_SENSOR_TOPIC, message, 1);
    cJSON_Delete(message);
}

static void mqtt_publish_status(const char *event, const char *status, const char *reason)
{
    cJSON *message = cJSON_CreateObject();
    if (message == NULL) {
        ESP_LOGE(TAG, "Failed to create status JSON");
        return;
    }

    const bool message_valid =
        cJSON_AddStringToObject(message, "type", "status") != NULL &&
        cJSON_AddStringToObject(message, "event", event) != NULL &&
        cJSON_AddStringToObject(message, "status", status) != NULL &&
        cJSON_AddStringToObject(message, "reason", reason) != NULL &&
        cJSON_AddBoolToObject(message, "window_open", s_window_open) != NULL &&
        cJSON_AddStringToObject(message, "curtain_position",
                                curtain_position_to_string(curtain_get_position())) != NULL &&
        cJSON_AddStringToObject(message, "curtain_target",
                                curtain_position_to_string(curtain_get_target())) != NULL &&
        cJSON_AddBoolToObject(message, "automatic_control", s_auto_control_enabled) != NULL &&
        cJSON_AddBoolToObject(message, "alarm_active", s_safety_alarm_active) != NULL;
    if (!message_valid) {
        ESP_LOGE(TAG, "Failed to build status JSON");
        cJSON_Delete(message);
        return;
    }

    mqtt_publish_json(MQTT_STATUS_TOPIC, message, 1);
    cJSON_Delete(message);
}

static esp_err_t safety_open_actuators(void)
{
    if (!s_window_open) {
        ESP_RETURN_ON_ERROR(window_set_open(true), TAG, "Failed to open window for safety alarm");
    }

    if (curtain_get_target() != CURTAIN_POSITION_OPEN) {
        ESP_RETURN_ON_ERROR(curtain_request_state(true, "automatic_sensor"), TAG,
                            "Failed to open curtains for safety alarm");
    }
    return ESP_OK;
}

static void apply_automatic_control(const sensor_readings_t *readings)
{
    const bool was_alarm_active = s_safety_alarm_active;
    s_safety_alarm_active = readings->alarm_active;

    if (!s_auto_control_enabled || !readings->alarm_active) {
        return;
    }

    if (!was_alarm_active || !s_window_open || curtain_get_target() != CURTAIN_POSITION_OPEN) {
        const esp_err_t result = safety_open_actuators();
        if (result == ESP_OK) {
            ESP_LOGW(TAG, "Safety threshold reached; opening window and curtains");
            mqtt_publish_status("automatic_control", "activated", "sensor_threshold");
        } else {
            ESP_LOGE(TAG, "Automatic safety action failed: %s", esp_err_to_name(result));
            mqtt_publish_status("automatic_control", "failed", "actuator_error");
        }
    }
}

static bool control_state_from_json(const cJSON *state, bool *open)
{
    if (!cJSON_IsString(state) || state->valuestring == NULL) {
        return false;
    }
    if (strcmp(state->valuestring, "open") == 0) {
        *open = true;
        return true;
    }
    if (strcmp(state->valuestring, "close") == 0) {
        *open = false;
        return true;
    }
    return false;
}

static bool control_direction_from_json(const cJSON *direction, stepper_direction_t *value)
{
    if (!cJSON_IsString(direction) || direction->valuestring == NULL) {
        return false;
    }
    if (strcmp(direction->valuestring, "cw") == 0) {
        *value = STEPPER_DIRECTION_CW;
        return true;
    }
    if (strcmp(direction->valuestring, "ccw") == 0) {
        *value = STEPPER_DIRECTION_CCW;
        return true;
    }
    return false;
}

static void mqtt_handle_control_payload(const char *data, int data_len)
{
    if (data_len <= 0 || data_len > MQTT_CONTROL_MAX_LEN) {
        ESP_LOGW(TAG, "Control payload has invalid length: %d", data_len);
        mqtt_publish_status("command", "rejected", "payload_too_large_or_empty");
        return;
    }

    char payload[MQTT_CONTROL_MAX_LEN + 1U];
    memcpy(payload, data, data_len);
    payload[data_len] = '\0';

    cJSON *root = cJSON_Parse(payload);
    if (root == NULL) {
        ESP_LOGW(TAG, "Invalid control JSON");
        mqtt_publish_status("command", "rejected", "invalid_json");
        return;
    }

    const cJSON *command = cJSON_GetObjectItemCaseSensitive(root, "cmd");
    if (!cJSON_IsString(command) || command->valuestring == NULL) {
        mqtt_publish_status("command", "rejected", "missing_command");
        cJSON_Delete(root);
        return;
    }

    if (strcmp(command->valuestring, "curtain") == 0) {
        bool open;
        const cJSON *state = cJSON_GetObjectItemCaseSensitive(root, "state");
        if (!control_state_from_json(state, &open)) {
            mqtt_publish_status("command", "rejected", "invalid_curtain_state");
        } else if (!open && s_auto_control_enabled && s_safety_alarm_active) {
            mqtt_publish_status("command", "rejected", "safety_alarm_active");
        } else {
            const esp_err_t result = curtain_request_state(open, "mqtt_curtain");
            mqtt_publish_status("command", result == ESP_OK ? "accepted" : "rejected",
                                result == ESP_OK ? "curtain_requested" : "curtain_queue_full");
        }
    } else if (strcmp(command->valuestring, "window") == 0) {
        bool open;
        const cJSON *state = cJSON_GetObjectItemCaseSensitive(root, "state");
        if (!control_state_from_json(state, &open)) {
            mqtt_publish_status("command", "rejected", "invalid_window_state");
        } else if (!open && s_auto_control_enabled && s_safety_alarm_active) {
            mqtt_publish_status("command", "rejected", "safety_alarm_active");
        } else {
            const esp_err_t result = window_set_open(open);
            mqtt_publish_status("command", result == ESP_OK ? "completed" : "rejected",
                                result == ESP_OK ? "window_set" : "window_driver_error");
        }
    } else if (strcmp(command->valuestring, "auto") == 0) {
        const cJSON *enabled = cJSON_GetObjectItemCaseSensitive(root, "enabled");
        if (!cJSON_IsBool(enabled)) {
            mqtt_publish_status("command", "rejected", "invalid_auto_setting");
        } else {
            s_auto_control_enabled = cJSON_IsTrue(enabled);
            if (s_auto_control_enabled && s_safety_alarm_active) {
                const esp_err_t result = safety_open_actuators();
                mqtt_publish_status("command", result == ESP_OK ? "completed" : "rejected",
                                    result == ESP_OK ? "auto_enabled_safety_opened"
                                                     : "automatic_action_failed");
            } else {
                mqtt_publish_status("command", "completed",
                                    s_auto_control_enabled ? "auto_enabled" : "auto_disabled");
            }
        }
    } else if (strcmp(command->valuestring, "rotate") == 0) {
        const cJSON *steps = cJSON_GetObjectItemCaseSensitive(root, "steps");
        const cJSON *direction = cJSON_GetObjectItemCaseSensitive(root, "direction");
        const cJSON *rpm = cJSON_GetObjectItemCaseSensitive(root, "rpm");
        stepper_direction_t requested_direction;

        if (!cJSON_IsNumber(steps) || !cJSON_IsNumber(rpm) ||
            !control_direction_from_json(direction, &requested_direction)) {
            mqtt_publish_status("command", "rejected", "invalid_rotate_command");
        } else {
            const double requested_steps = cJSON_GetNumberValue(steps);
            const double requested_rpm = cJSON_GetNumberValue(rpm);
            if (requested_steps < 1.0 || requested_steps > STEPPER_MAX_STEPS ||
                requested_steps != (double)(uint32_t)requested_steps ||
                requested_rpm <= 0.0 || requested_rpm > STEPPER_MAX_RPM) {
                mqtt_publish_status("command", "rejected", "value_out_of_range");
            } else {
                const esp_err_t result = curtain_enqueue_manual_move(
                    (uint32_t)requested_steps, requested_direction, (float)requested_rpm,
                    "mqtt_rotate");
                mqtt_publish_status("command", result == ESP_OK ? "accepted" : "rejected",
                                    result == ESP_OK ? "curtain_move_queued" : "curtain_queue_full");
            }
        }
    } else if (strcmp(command->valuestring, "status") == 0) {
        mqtt_publish_status("command", "completed", "status_requested");
    } else {
        mqtt_publish_status("command", "rejected", "unsupported_command");
    }

    cJSON_Delete(root);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id,
                               void *event_data)
{
    (void)handler_args;
    (void)base;
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        s_mqtt_connected = true;
        ESP_LOGI(TAG, "Connected to MQTT broker");
        if (esp_mqtt_client_subscribe(event->client, MQTT_CONTROL_TOPIC, 1) < 0) {
            ESP_LOGE(TAG, "Failed to subscribe to %s", MQTT_CONTROL_TOPIC);
        }
        mqtt_publish_heartbeat();
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_mqtt_connected = false;
        ESP_LOGW(TAG, "Disconnected from MQTT broker");
        break;

    case MQTT_EVENT_DATA:
        if (event->current_data_offset != 0 || event->total_data_len != event->data_len) {
            ESP_LOGW(TAG, "Ignoring fragmented MQTT control payload");
        } else if (event->topic != NULL &&
                   event->topic_len == (int)strlen(MQTT_CONTROL_TOPIC) &&
                   memcmp(event->topic, MQTT_CONTROL_TOPIC, (size_t)event->topic_len) == 0) {
            mqtt_handle_control_payload(event->data, event->data_len);
        }
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT transport error");
        break;

    default:
        break;
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                               void *event_data)
{
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected; reconnecting");
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = event_data;
        ESP_LOGI(TAG, "Wi-Fi connected, IP:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static esp_err_t wifi_init(void)
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "Failed to erase NVS");
        result = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(result, TAG, "Failed to initialize NVS");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "Failed to initialize network interface");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "Failed to create event loop");

    esp_netif_create_default_wifi_sta();
    const wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_config), TAG, "Failed to initialize Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                   wifi_event_handler, NULL),
                        TAG, "Failed to register Wi-Fi handler");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                   wifi_event_handler, NULL),
                        TAG, "Failed to register IP handler");

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "Failed to set station mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG,
                        "Failed to set Wi-Fi configuration");
    return esp_wifi_start();
}

static esp_err_t mqtt_init(void)
{
    uint8_t mac[6];
    ESP_RETURN_ON_ERROR(esp_read_mac(mac, ESP_MAC_WIFI_STA), TAG, "Failed to read Wi-Fi MAC");

    snprintf(s_mqtt_client_id, sizeof(s_mqtt_client_id), MQTT_CLIENT_ID_PREFIX
             "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    const esp_mqtt_client_config_t mqtt_config = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.client_id = s_mqtt_client_id,
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_config);
    if (s_mqtt_client == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID,
                                                       mqtt_event_handler, NULL),
                        TAG, "Failed to register MQTT handler");
    return esp_mqtt_client_start(s_mqtt_client);
}

static void heartbeat_task(void *argument)
{
    (void)argument;
    while (true) {
        mqtt_publish_heartbeat();
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
    }
}

static void sensor_task(void *argument)
{
    (void)argument;
    while (true) {
        sensor_readings_t readings;
        sensors_read(&readings);
        apply_automatic_control(&readings);
        mqtt_publish_sensor(&readings);
        vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL_MS));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(curtain_init());
    ESP_ERROR_CHECK(window_init());
    ESP_ERROR_CHECK(sensors_init());
    ESP_ERROR_CHECK(wifi_init());
    ESP_ERROR_CHECK(mqtt_init());

    if (xTaskCreate(heartbeat_task, "heartbeat_task", 3072, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create heartbeat task");
    }
    if (xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
    }

    ESP_LOGI(TAG, "Smart window and curtain controller started");
}
