/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
#include "veml.h"

#include <string.h>

#include "driver/i2c.h"

#define ALS_BASE_SCALE 0.0036  ///< Base scale for raw ALS to Lux

static uint8_t get_integration_scale(
    const veml_integration_options_e integration_time);

static uint8_t get_gain_scale(const veml_gain_options_e gain);

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
esp_err_t Veml7700::get_configuration(void) {
    return this->read_from_reg(VEML_CONFIG_REG,
                               (uint16_t *)&this->configuration);
}

// See veml.h for documentation
esp_err_t Veml7700::get_configuration(veml_config_reg_t *config) {
    esp_err_t read_err = this->get_configuration();
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
float Veml7700::get_lux(void) {
    uint16_t raw_value = this->get_ambient_level();
    const uint16_t gain_scale =
        get_gain_scale((veml_gain_options_e)this->configuration.gain);
    const uint16_t integration_scale = get_integration_scale(
        (veml_integration_options_e)this->configuration.integration_time);
    return raw_value * (ALS_BASE_SCALE * gain_scale * integration_scale);
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

/// @brief Compute the multiplier caused by different integration times.
/// @param integration_time The integration time to get the multiplier from
/// @return The multiplier based on the integration time.
static uint8_t get_integration_scale(
    const veml_integration_options_e integration_time) {
    switch (integration_time) {
        case VEML_INTEGRATION_25:
            return 32;
        case VEML_INTEGRATION_50:
            return 16;
        case VEML_INTEGRATION_100:
            return 8;
        case VEML_INTEGRATION_200:
            return 4;
        case VEML_INTEGRATION_400:
            return 2;
        case VEML_INTEGRATION_800:
            return 1;
        default:
            ESP_ERROR_CHECK(ESP_ERR_NOT_SUPPORTED);
            /// Will restart at error check, so we can just do this
            return 0;
    }
}

/// @brief Get the multiplier from different gain options
/// @param gain The gain to get the multiplier for
/// @return The multiplier based on the gain.
static uint8_t get_gain_scale(const veml_gain_options_e gain) {
    switch (gain) {
        case VEML_GAIN_2:
            return 1;
        case VEML_GAIN_1:
            return 2;
        case VEML_GAIN_1_4:
            return 8;
        case VEML_GAIN_1_8:
            return 16;
        default:
            ESP_ERROR_CHECK(ESP_ERR_NOT_SUPPORTED);
            /// Will restart at error check, so we can just do this
            return 0;
    }
}