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
#include "driver/i2c.h"
#include "esp_timer.h"
#include "veml.h"

// Used for WiFi stuff
#include "wifi_config.h"
#include "wifi_network.h"

// Network Time
#include "common.h"

// Tags for logging
#define I2C_TASK_NAME "i2c_sensor_task"  /// Name for logging

#define C_TO_F(celsius) \
    ((celsius * 9 / 5) + 32)  /// Convert Celsius to Fahrenheit

// I2C Configuration
#define I2C_MASTER_SDA_IO 32                     /// I2C SDA Pin
#define I2C_MASTER_SCL_IO 33                     /// I2C SCL Pin
#define I2C_MASTER_FREQ_HZ 100000                /// I2C Frequency
#define I2C_MASTER_PORT I2C_NUM_0                /// I2C Bus Number
#define I2C_TIMEOUT (1000 / portTICK_PERIOD_MS)  /// I2C Transaction Timeout

Veml7700 veml(I2C_MASTER_PORT, I2C_TIMEOUT);
BSEC bsec(I2C_MASTER_PORT, I2C_TIMEOUT, 0.0f);
WiFiNetwork wifi;

/*=============================================
 *   Function Delcarations
 *============================================*/

static esp_err_t configure_i2c(void);
static void i2c_sensor_task(void* taskParam);

/*=============================================
 *   Function Definitions
 *============================================*/

/// @brief Configure the I2C bus
/// @return Error code from configuring I2C bus.
static esp_err_t configure_i2c(void) {
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
    i2c_param_config(I2C_MASTER_PORT, &i2c_config);
    return i2c_driver_install(I2C_MASTER_PORT, i2c_config.mode, 0, 0, 0);
}

/// @brief The task for reading data from the i2c sensors.
/// @param taskParam Parameters for the task. Currently unused
static void i2c_sensor_task(void* taskParams) {
    ESP_LOGI(I2C_TASK_NAME, "Starting I2C Task Up");
    ESP_ERROR_CHECK(configure_i2c());

    // Configure BSEC library.
    ESP_LOGI(I2C_TASK_NAME, "Starting BSEC. Result=%lld",
             bsec.init().integer_result);
    ESP_LOGI(I2C_TASK_NAME, "Setting subscription. Result=%d",
             bsec.subscribe_all_non_scan(BSEC_SAMPLE_RATE_LP));

    // Configure VEML
    ESP_ERROR_CHECK(veml.set_configuration());
    ESP_LOGI(I2C_TASK_NAME, "VEML7700 Gain Option: %d", veml.get_gain());
    ESP_LOGI(I2C_TASK_NAME, "VEML7700 Integration Time: %d",
             veml.get_integration_time());

    // Continuously read from the sensors and print the result.
    while (true) {
        // Run the BSEC periodic processing functionality.
        ESP_LOGI(
            I2C_TASK_NAME, "Periodic Process. Result=%lld",
            bsec.periodic_process(esp_timer_get_time() * 1000).integer_result);

        // Read the data from the last periodic processing run.
        uint8_t num_output = 0;
        bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];
        bsec.get_output(outputs, &num_output);
        for (int i = 0; i < num_output; i++) {
            ESP_LOGI(I2C_TASK_NAME, "Output num=%d, type=%d, acc=%d, value=%f",
                     i, outputs[i].sensor_id, outputs[i].accuracy,
                     outputs[i].signal);
        }

        // Read and log the ambient light level
        ESP_LOGI(I2C_TASK_NAME, "ALS: %d, White: %d, lux: %f",
                 veml.get_ambient_level(), veml.get_white_level(),
                 veml.get_lux());

        // Calculate time to sleep until next periodic processing cycle.
        int64_t remaining_time =
            bsec.get_next_call_time_us() - esp_timer_get_time();
        ESP_LOGI(I2C_TASK_NAME, "High Water Mark: %d",
                 uxTaskGetStackHighWaterMark(NULL));
        // FIXME: Use xTaskDelayUntil instead?

        vTaskDelay(remaining_time / 1000 / portTICK_PERIOD_MS);
    }
}

/// @brief Main task function.
extern "C" void app_main(void) {
    // Initialize the WiFi Connection.
    ESP_ERROR_CHECK(wifi.init());
    // Scan for and print found networks
    ESP_ERROR_CHECK(wifi.scan_for_networks());

    // Connect to specific wifi network
    ESP_ERROR_CHECK(wifi.connect_to_ap(WIFI_SSID, WIFI_PASSWORD));

    // Update system time
    ESP_ERROR_CHECK(wifi.update_time_from_network(20000));

    ESP_LOGI("app_main", "High Water Mark: %d",
             uxTaskGetStackHighWaterMark(NULL));

    // Start up the sensor reading.
    static uint8_t ucParameterToPass;
    TaskHandle_t i2c_task_handle;
    xTaskCreate(i2c_sensor_task, "I2C_SENSOR_TASK", 4096, &ucParameterToPass,
                tskIDLE_PRIORITY, &i2c_task_handle);
}
