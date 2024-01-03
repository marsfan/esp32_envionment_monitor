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
#define BSEC_TASK_NAME "bsec_task"
#define VEML_TASK_NAME "veml_task"
#define SENSOR_HUB_TASK_NAME "sensor_hub"

// Priority of various tasks.
// Higher number = Higher Priority
/// @brief Priority for tasks that read sensors
#define SENSOR_TASK_PRI 1U
/// @brief Priority of the sensor hub task
/// @details Higher priority than sensor tasks to ensure it can pull data off
/// the queue faster than they can insert it
#define SENSOR_HUB_TASK_PRI 2U

/// @brief Stack size allocated for all tasks
#define TASK_STACK_SIZE 2048
/// @brief Stack size for the BSEC task
#define BSEC_STACK_SIZE 4096

#define C_TO_F(celsius) \
    ((celsius * 9 / 5) + 32)  /// Convert Celsius to Fahrenheit

// I2C Configuration
#define I2C_MASTER_SDA_IO 32                     ///< I2C SDA Pin
#define I2C_MASTER_SCL_IO 33                     ///< I2C SCL Pin
#define I2C_MASTER_FREQ_HZ 100000                ///< I2C Frequency
#define I2C_MASTER_PORT I2C_NUM_0                ///< I2C Bus Number
#define I2C_TIMEOUT (1000 / portTICK_PERIOD_MS)  ///< I2C Transaction Timeout

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
static void sensor_hub_task(void* taskParam);

/*=============================================
 *   Function Definitions
 *============================================*/

/// @brief Task for collecting data from the various sensors and re-distributing
/// it.
/// @param taskParam Parameters to pass into the task. Currently unused.
static void sensor_hub_task(void* taskParam) {
    sensor_hub.task_logic();
}

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

        // Send the data to the sensor hub task
        bsec_structured_outputs_t bsec_data;
        bsec.get_output_data(&bsec_data);
        ESP_ERROR_CHECK(sensor_hub.send_bsec(&bsec_data));

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

        // Send the data to the sensor hub task
        veml_output_t data;
        veml.get_outputs(&data);
        ESP_ERROR_CHECK(sensor_hub.send_veml(&data));

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

/// @brief Main task function.
extern "C" void app_main(void) {
    static uint8_t sensor_hub_params;
    TaskHandle_t sensor_hub_task_handle;
    static uint8_t veml_params;
    TaskHandle_t veml_task_handle;
    static uint8_t bsec_params;
    TaskHandle_t bsec_task_handle;

    // Start the sensor hub task
    xTaskCreate(sensor_hub_task, SENSOR_HUB_TASK_NAME, TASK_STACK_SIZE,
                &sensor_hub_params, SENSOR_HUB_TASK_PRI,
                &sensor_hub_task_handle);

    // Initialize I2C
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = I2C_MASTER_FREQ_HZ},
        .clk_flags = 0,  // optional; you can use I2C_SCLK_SRC_FLAG_* flags
                         // to choose i2c source clock here
    };
    ESP_ERROR_CHECK(i2c.init(&i2c_config));

    // Start up the sensor reading.

    xTaskCreate(bsec_task, BSEC_TASK_NAME, BSEC_STACK_SIZE, &bsec_params,
                SENSOR_TASK_PRI, &bsec_task_handle);
    xTaskCreate(veml_task, VEML_TASK_NAME, TASK_STACK_SIZE, &veml_params,
                SENSOR_TASK_PRI, &veml_task_handle);

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
        /// Get the data from the sensor hub.

        sensor_hub_data_t data;
        ESP_ERROR_CHECK(sensor_hub.get_data(&data));

        // TODO: MQTT in separate task
        // mqtt.publish(MQTT_TEMP_TOPIC,
        // C_TO_F(data.bsec.compensated_temp.signal), 0,
        //              0);
        mqtt.publish(MQTT_TEMP_TOPIC, data.bsec.compensated_temp, 0, 0);

        ESP_LOGI("app_main", "Temp: %f, Acc: %d, valid: %d",
                 C_TO_F(data.bsec.compensated_temp.signal),
                 data.bsec.compensated_temp.accuracy,
                 data.bsec.compensated_temp.valid);
        ESP_LOGI("app_main", "Humidity: %f, Acc: %d, valid: %d",
                 C_TO_F(data.bsec.compensated_humidity.signal),
                 data.bsec.compensated_humidity.accuracy,
                 data.bsec.compensated_humidity.valid);
        ESP_LOGI("app_main", "Pressure: %f, Acc: %d, valid: %d",
                 C_TO_F(data.bsec.raw_pressure.signal),
                 data.bsec.raw_pressure.accuracy, data.bsec.raw_pressure.valid);
        ESP_LOGI("app_main", "Raw Gas: %f, Acc: %d, valid: %d",
                 C_TO_F(data.bsec.raw_gas.signal), data.bsec.raw_gas.accuracy,
                 data.bsec.raw_gas.valid);
        ESP_LOGI("app_main", "IAQ: %f, Acc: %d, valid: %d",
                 data.bsec.iaq.signal, data.bsec.iaq.accuracy,
                 data.bsec.iaq.valid);
        ESP_LOGI("app_main", "Static IAQ: %f, Acc: %d, valid: %d",
                 data.bsec.static_iaq.signal, data.bsec.static_iaq.accuracy,
                 data.bsec.static_iaq.valid);
        ESP_LOGI("app_main", "eCO2 IAQ: %f, Acc: %d, valid: %d",
                 data.bsec.co2_eq.signal, data.bsec.co2_eq.accuracy,
                 data.bsec.co2_eq.valid);
        ESP_LOGI("app_main", "Breath VOC: %f, Acc: %d, valid: %d",
                 data.bsec.breath_voc_eq.signal,
                 data.bsec.breath_voc_eq.accuracy,
                 data.bsec.breath_voc_eq.valid);
        ESP_LOGI("app_main", "Gas Percent: %f, Acc: %d, valid: %d",
                 data.bsec.gas_percentage.signal,
                 data.bsec.gas_percentage.accuracy,
                 data.bsec.gas_percentage.valid);
        ESP_LOGI("app_main", "Run In Status: %f, Acc: %d, valid: %d",
                 data.bsec.run_in_status.signal,
                 data.bsec.run_in_status.accuracy,
                 data.bsec.run_in_status.valid);
        ESP_LOGI("app_main", "Stabilization: %f, Acc: %d, valid: %d",
                 data.bsec.stabilization_status.signal,
                 data.bsec.stabilization_status.accuracy,
                 data.bsec.stabilization_status.valid);
        ESP_LOGI("app_main", "ALS: %d, White: %d, LUX: %f", data.veml.raw_als,
                 data.veml.raw_white, data.veml.lux);
        ESP_LOGI("app_main", "-----------------------------------");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
