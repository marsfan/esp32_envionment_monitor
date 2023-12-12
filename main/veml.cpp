/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
#include "veml.h"

#include <string.h>

#include "driver/i2c.h"

Veml7700::Veml7700(i2c_port_t i2c_port) {
    this->i2c_port = i2c_port;
}

// See veml.h for documentation
esp_err_t Veml7700::set_configuration(void) {
    veml_config_reg_t config;
    memset(&config, 0x0, sizeof(veml_config_reg_t));
    return this->set_configuration(&config);
}

// See veml.h for documentation.
esp_err_t Veml7700::set_configuration(const veml_config_reg_t *const config) {
    /// Copy configuration to store internally
    (void)memcpy(&this->configuration, config, sizeof(veml_config_reg_t));

    return this->write_to_reg(VEML_CONFIG_REG,
                              (uint16_t *)&this->configuration);
}

// See veml.h for documentation
esp_err_t Veml7700::get_configuration(veml_config_reg_t *config) {
    esp_err_t read_err =
        this->read_from_reg(VEML_CONFIG_REG, (uint16_t *)config);
    if (read_err == ESP_OK) {
        (void)memcpy(&this->configuration, config, sizeof(veml_config_reg_t));
    }
    return read_err;
}

// See veml.h for documentation
uint16_t Veml7700::get_ambient_level(void) {
    uint16_t data;
    ESP_ERROR_CHECK(this->read_from_reg(VEML_ALS_LEVEL_REG, &data));
    return data;
}

// See veml.h for documentation
uint16_t Veml7700::get_white_level(void) {
    uint16_t data;
    ESP_ERROR_CHECK(this->read_from_reg(VEML_WHITE_LEVEL_REG, &data));
    return data;
}
// See veml.h for documentation.
esp_err_t Veml7700::write_to_reg(const uint8_t reg,
                                 const uint16_t *const data) {
    // First byte is  the register
    // Second and third bytes are the data bytes.
    uint8_t data_buffer[3] = {reg, (*data & 0xF0) >> 0xF,
                              (uint8_t)(*data & 0x0F)};
    return i2c_master_write_to_device(this->i2c_port, VEML_ADDR, data_buffer, 3,
                                      I2C_MASTER_TIMEOUT_MS);
}

// See veml.h for documentation
esp_err_t Veml7700::read_from_reg(const uint8_t reg, uint16_t *const data) {
    return i2c_master_write_read_device(this->i2c_port, VEML_ADDR,
                                        (uint8_t *)&reg, 1, (uint8_t *)data,
                                        VEML_REG_BYTES, I2C_MASTER_TIMEOUT_MS);
}