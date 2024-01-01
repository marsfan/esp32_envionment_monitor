/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
#include "safe_i2c.h"

#include <string.h>

#include "common.h"

#define LOG_TAG "safe_i2c"

// See safe_i2c.h for documentation
SafeI2C::SafeI2C(const i2c_port_t i2c_port) {
    this->port = i2c_port;
}

// See safe_i2c.h for documentation
esp_err_t SafeI2C::init(const i2c_config_t* config) {
    esp_err_t result = ESP_OK;
    if (this->take_mutex(LOG_TAG, __func__)) {
        (void)memcpy(&this->config, config, sizeof(i2c_config_t));
        result = i2c_param_config(this->port, &this->config);
        LOGE_ON_ERROR(LOG_TAG, __func__, "Failed to configure I2C bus", result);
        if (result == ESP_OK) {
            result = i2c_driver_install(this->port, this->config.mode, 0, 0, 0);
            LOGE_ON_ERROR(LOG_TAG, __func__, "Failed to install I2C Driver",
                          result);
        }

        this->release_mutex(LOG_TAG, __func__);
    }
    return result;
}

esp_err_t SafeI2C::master_write_read_device(
    uint8_t device_address, const uint8_t* write_buffer, size_t write_size,
    uint8_t* read_buffer, size_t read_size, TickType_t ticks_to_wait) {
    esp_err_t result = ESP_OK;
    if (this->take_mutex(LOG_TAG, __func__)) {
        result = i2c_master_write_read_device(
            this->port, device_address, write_buffer, write_size, read_buffer,
            read_size, ticks_to_wait);
        this->release_mutex(LOG_TAG, __func__);
    }
    return result;
}

esp_err_t SafeI2C::master_write_to_device(uint8_t device_address,
                                          const uint8_t* write_buffer,
                                          size_t write_size,
                                          TickType_t ticks_to_wait) {
    esp_err_t result = ESP_OK;
    if (this->take_mutex(LOG_TAG, __func__)) {
        result =
            i2c_master_write_to_device(this->port, device_address, write_buffer,
                                       write_size, ticks_to_wait);
        this->release_mutex(LOG_TAG, __func__);
    }
    return result;
}
