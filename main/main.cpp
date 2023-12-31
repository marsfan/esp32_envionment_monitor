/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
#include <esp_err.h>
#include <esp_log.h>
#include <stdio.h>
#include <string.h>

// Used for sensor interfacing.
#include <sys/time.h>

#include "bsec.h"
#include "esp_timer.h"
#include "safe_i2c.h"
#include "veml.h"

// Used for WiFi stuff
#include <nvs_flash.h>

#include "private_config.h"
#include "wifi_network.h"

// Network Time
#include "common.h"

// MQTT stuff
#include <mqtt_task.h>

// Sensor Hub data
#include "sensor_hub.h"

// Tags for logging
#define I2C_TASK_NAME "i2c_sensor_task"
#define BSEC_TASK_NAME "bsec_task"
#define VEML_TASK_NAME "veml_task"

#define C_TO_F(celsius) \
    ((celsius * 9 / 5) + 32)  /// Convert Celsius to Fahrenheit

// I2C Configuration
#define I2C_MASTER_SDA_IO 32                     /// I2C SDA Pin
#define I2C_MASTER_SCL_IO 33                     /// I2C SCL Pin
#define I2C_MASTER_FREQ_HZ 100000                /// I2C Frequency
#define I2C_MASTER_PORT I2C_NUM_0                /// I2C Bus Number
#define I2C_TIMEOUT (1000 / portTICK_PERIOD_MS)  /// I2C Transaction Timeout

SafeI2C i2c(I2C_MASTER_PORT);
Veml7700 veml(&i2c, I2C_TIMEOUT);
BSEC bsec(&i2c, I2C_TIMEOUT, 0.0f);
WiFiNetwork wifi;
MQTTClient mqtt(MQTT_BROKER_URI, MQTT_USERNAME, MQTT_PASSWORD);
SensorHub sensor_hub;

/*=============================================
 *   Function Delcarations
 *============================================*/

static void bsec_task(void* taskParam);
static void veml_task(void* taskParam);

/*=============================================
 *   Function Definitions
 *============================================*/

/// @brief Task for reading data from the BME688 sensor and processing it with
/// BSEC.
/// @param taskParam Parameters to pass into the task. Currently unused.
static void bsec_task(void* taskParam) {
    ESP_LOGI(BSEC_TASK_NAME, "Starting BSEC task");

    ESP_LOGI(BSEC_TASK_NAME, "Starting BSEC. Result=%lld",
             bsec.init().integer_result);
    ESP_LOGI(BSEC_TASK_NAME, "Setting BSEC subscription. Result=%d",
             bsec.subscribe_all_non_scan(BSEC_SAMPLE_RATE_LP));

    while (true) {
        // Run BSEC periodic processing functionality
        ESP_ERROR_CHECK(
            bsec.periodic_process(esp_timer_get_time() * 1000).integer_result);

        int64_t remaining_time =
            bsec.get_next_call_time_us() - esp_timer_get_time();
        // TODO: Use xTaskDelayUntil instead?
        vTaskDelay(remaining_time / 1000 / portTICK_PERIOD_MS);
    }
}

static void veml_task(void* taskParam) {
    ESP_LOGI(VEML_TASK_NAME, "Starting VEML Task");
    ESP_ERROR_CHECK(veml.set_configuration());
    ESP_LOGI(VEML_TASK_NAME, "VEML7700 Gain Option: %d", veml.get_gain());
    ESP_LOGI(VEML_TASK_NAME, "VEML7700 Integration Time: %d",
             veml.get_integration_time());

    while (true) {
        // Read the VEML sensor every half a second.
        ESP_ERROR_CHECK(veml.periodic_process());
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

/// @brief Main task function.
extern "C" void app_main(void) {
    // Initialize I2C
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = I2C_MASTER_FREQ_HZ},
        .clk_flags = 0,  // optional; you can use I2C_SCLK_SRC_FLAG_* flags to
                         // choose i2c source clock here
    };
    ESP_ERROR_CHECK(i2c.init(&i2c_config));

    // Start up the sensor reading.
    static uint8_t veml_params;
    TaskHandle_t veml_task_handle;
    static uint8_t bsec_params;
    TaskHandle_t bsec_task_handle;

    xTaskCreate(bsec_task, BSEC_TASK_NAME, 2048, &bsec_params, 1,
                &bsec_task_handle);
    xTaskCreate(veml_task, VEML_TASK_NAME, 2048, &veml_params, 1,
                &veml_task_handle);

    // Initialize NVS flash for WiFi system
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize the WiFi Connection.
    ESP_ERROR_CHECK(wifi.init());
    // Scan for and print found networks
    ESP_ERROR_CHECK(wifi.scan_for_networks());

    // Connect to specific wifi network
    ESP_ERROR_CHECK(wifi.connect_to_ap(WIFI_SSID, WIFI_PASSWORD));

    // Update system time
    ESP_ERROR_CHECK(wifi.update_time_from_network(20000));

    // TODO: MQTT in separate task.
    ESP_ERROR_CHECK(mqtt.start());

    ESP_LOGI("app_main", "High Water Mark: %d",
             uxTaskGetStackHighWaterMark(NULL));

    while (true) {
        bsec_structured_outputs_t data;
        veml_output_t veml_data;
        bsec.get_output_data(&data);
        ESP_ERROR_CHECK(veml.get_outputs(&veml_data));

        // TODO: MQTT in separate task
        // mqtt.publish(MQTT_TEMP_TOPIC, C_TO_F(data.compensated_temp.signal),
        // 0,
        //              0);
        mqtt.publish(MQTT_TEMP_TOPIC, data.compensated_temp, 0, 0);

        ESP_LOGI("app_main", "Temp: %f, Acc: %d, valid: %d",
                 C_TO_F(data.compensated_temp.signal),
                 data.compensated_temp.accuracy, data.compensated_temp.valid);
        ESP_LOGI("app_main", "Humidity: %f, Acc: %d, valid: %d",
                 C_TO_F(data.compensated_humidity.signal),
                 data.compensated_humidity.accuracy,
                 data.compensated_humidity.valid);
        ESP_LOGI("app_main", "Pressure: %f, Acc: %d, valid: %d",
                 C_TO_F(data.raw_pressure.signal), data.raw_pressure.accuracy,
                 data.raw_pressure.valid);
        ESP_LOGI("app_main", "Raw Gas: %f, Acc: %d, valid: %d",
                 C_TO_F(data.raw_gas.signal), data.raw_gas.accuracy,
                 data.raw_gas.valid);
        ESP_LOGI("app_main", "IAQ: %f, Acc: %d, valid: %d", data.iaq.signal,
                 data.iaq.accuracy, data.iaq.valid);
        ESP_LOGI("app_main", "Static IAQ: %f, Acc: %d, valid: %d",
                 data.static_iaq.signal, data.static_iaq.accuracy,
                 data.static_iaq.valid);
        ESP_LOGI("app_main", "eCO2 IAQ: %f, Acc: %d, valid: %d",
                 data.co2_eq.signal, data.co2_eq.accuracy, data.co2_eq.valid);
        ESP_LOGI("app_main", "Breath VOC: %f, Acc: %d, valid: %d",
                 data.breath_voc_eq.signal, data.breath_voc_eq.accuracy,
                 data.breath_voc_eq.valid);
        ESP_LOGI("app_main", "Gas Percent: %f, Acc: %d, valid: %d",
                 data.gas_percentage.signal, data.gas_percentage.accuracy,
                 data.gas_percentage.valid);
        ESP_LOGI("app_main", "Run In Status: %f, Acc: %d, valid: %d",
                 data.run_in_status.signal, data.run_in_status.accuracy,
                 data.run_in_status.valid);
        ESP_LOGI("app_main", "Stabilization: %f, Acc: %d, valid: %d",
                 data.stabilization_status.signal,
                 data.stabilization_status.accuracy,
                 data.stabilization_status.valid);
        ESP_LOGI("app_main", "ALS: %d, White: %d, LUX: %f", veml_data.raw_als,
                 veml_data.raw_white, veml_data.lux);
        ESP_LOGI("app_main", "-----------------------------------");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
