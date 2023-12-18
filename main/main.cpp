/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
#include <esp_err.h>
#include <esp_log.h>
#include <stdio.h>
#include <string.h>

#include "bme688.h"
#include "driver/i2c.h"
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
Bme688 bme(I2C_MASTER_PORT, I2C_TIMEOUT);

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

    // Configure BME688
    ESP_LOGI(LOG_TAG, "Started BME688. Result=%d", bme.init());
    ESP_LOGI(LOG_TAG, "Starting BME688 Self Test. This takes a few seconds");
    ESP_LOGI(LOG_TAG, "Finished BME688 self test. Result=%d", bme.self_test());
    ESP_LOGI(LOG_TAG, "Configuring BME688. Result=%d",
             bme.set_conf(BME68X_OS_2X, BME68X_OS_1X, BME68X_OS_16X,
                          BME68X_FILTER_OFF, BME68X_ODR_NONE));
    struct bme68x_heatr_conf heater_conf;
    heater_conf.enable = BME68X_ENABLE;
    heater_conf.heatr_temp = 300;
    heater_conf.heatr_dur = 100;
    ESP_LOGI(LOG_TAG, "Setting BME688 Heater Config. Result=%d",
             bme.set_heater_conf(BME68X_FORCED_MODE, &heater_conf));

    /// Configure VEML
    ESP_ERROR_CHECK(veml.set_configuration());
    ESP_LOGI(LOG_TAG, "VEML7700 Gain Option: %d", veml.get_gain());
    ESP_LOGI(LOG_TAG, "VEML7700 Integration Time: %d",
             veml.get_integration_time());

    /// Continuously read from the VEML light sensor and print the result.
    while (true) {
        ESP_LOGI(LOG_TAG, "ALS: %d, White: %d, lux: %f",
                 veml.get_ambient_level(), veml.get_white_level(),
                 veml.get_lux());

        uint8_t n_fields;
        struct bme68x_data data;
        ESP_LOGI(LOG_TAG, "Read BME688 Data. Result=%d",
                 bme.forced_measurement(&heater_conf, &data, &n_fields));

        ESP_LOGI(LOG_TAG,
                 "BME688 Sample, temp=%.2f, pressure=%.2f, "
                 "humidity=%.2f, gas resistance=%.2f, gas index: %d, "
                 "measurement index: %d",
                 C_TO_F(data.temperature), data.pressure, data.humidity,
                 data.gas_resistance, data.gas_index, data.meas_index);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
