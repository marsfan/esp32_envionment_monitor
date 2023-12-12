/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
#include <esp_err.h>
#include <esp_log.h>
#include <stdio.h>

#include "driver/i2c.h"
#include "veml.h"

#define LOG_TAG "main"

// I2C Configuration
#define I2C_MASTER_SDA_IO 32
#define I2C_MASTER_SCL_IO 33
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_PORT I2C_NUM_0

Veml7700 veml(I2C_MASTER_PORT);

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

    /// Configure VEML
    ESP_ERROR_CHECK(veml.set_configuration());

    /// Continuously read from the VEML light sensor and print the result.
    while (true) {
        uint16_t als_value = veml.get_ambient_level();
        uint16_t white_value = veml.get_white_level();
        ESP_LOGI(LOG_TAG, "ALS: %d, White: %d\n", als_value, white_value);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
