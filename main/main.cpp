/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
#include <esp_err.h>
#include <esp_log.h>
#include <stdio.h>
#include <string.h>

#include "bsec.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "veml.h"

#define LOG_TAG "app_main"  /// Name for logging

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

extern "C" void app_main(void) {
    ESP_LOGI(LOG_TAG, "Starting System Up");
    ESP_ERROR_CHECK(configure_i2c());

    // Configure BSEC library.
    ESP_LOGI(LOG_TAG, "Starting BSEC. Result=%lld", bsec.init().integer_result);
    ESP_LOGI(LOG_TAG, "Setting subscription. Result=%d",
             bsec.subscribe_all_non_scan(BSEC_SAMPLE_RATE_LP));

    // Configure VEML
    ESP_ERROR_CHECK(veml.set_configuration());
    ESP_LOGI(LOG_TAG, "VEML7700 Gain Option: %d", veml.get_gain());
    ESP_LOGI(LOG_TAG, "VEML7700 Integration Time: %d",
             veml.get_integration_time());

    // Continuously read from the sensors and print the result.
    while (true) {
        // Run the BSEC periodic processing functionality.
        ESP_LOGI(
            LOG_TAG, "Periodic Process. Result=%lld",
            bsec.periodic_process(esp_timer_get_time() * 1000).integer_result);

        // Read the data from the last periodic processing run.
        uint8_t num_output = 0;
        bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];
        bsec.get_output(outputs, &num_output);
        for (int i = 0; i < num_output; i++) {
            ESP_LOGI(LOG_TAG, "Output num=%d, type=%d, acc=%d, value=%f", i,
                     outputs[i].sensor_id, outputs[i].accuracy,
                     outputs[i].signal);
        }

        // Read and log the ambient light level
        ESP_LOGI(LOG_TAG, "ALS: %d, White: %d, lux: %f",
                 veml.get_ambient_level(), veml.get_white_level(),
                 veml.get_lux());

        // Calculate time to sleep until next periodic processing cycle.
        int64_t remaining_time =
            (bsec.get_next_call_time() / 1000) - esp_timer_get_time();
        vTaskDelay(remaining_time / 1000 / portTICK_PERIOD_MS);
    }
}
