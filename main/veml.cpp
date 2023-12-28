/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
#include "veml.h"

#include <driver/i2c.h>
#include <esp_log.h>
#include <stdio.h>
#include <string.h>

#include "common.h"

#define VEML_TAG "VEML7700"    //< Tag used for logging messages for this module
#define ALS_BASE_SCALE 0.0036  //< Base scale for raw ALS to Lux

static uint8_t get_integration_scale(
    const veml_integration_options_e integration_time);

static uint8_t get_gain_scale(const veml_gain_options_e gain);

/*======================================================================
 *                       Public Functions
 *=====================================================================*/

Veml7700::Veml7700(const i2c_port_t i2c_port, const TickType_t wait_time) {
    this->i2c_port = i2c_port;
    this->wait_time = wait_time;
    (void)memset(&this->configuration, 0, sizeof(veml_config_reg_t));
    this->output_mutex = xSemaphoreCreateMutex();
    if (this->output_mutex == NULL) {
        ESP_LOGE(VEML_TAG, "Failed to create mutex");
        ESP_ERROR_CHECK(ESP_FAIL);
    }
}

// See veml.h for documentation
esp_err_t Veml7700::set_configuration(void) {
    return this->set_configuration(&this->configuration);
}

// See veml.h for documentation.
esp_err_t Veml7700::set_configuration(const veml_config_reg_t *const config) {
    /// Copy configuration to store internally
    (void)memcpy(&this->configuration, config, sizeof(veml_config_reg_t));

    esp_err_t err =
        this->write_to_reg(VEML_CONFIG_REG, &this->configuration.regs_16bit);
    LOGE_ON_ERROR(VEML_TAG, __func__,
                  "Failed setting VEML7700 Configuring Register", err);

    return err;
}

// See veml.h for documentation
esp_err_t Veml7700::get_configuration(void) {
    esp_err_t err =
        this->read_from_reg(VEML_CONFIG_REG, &this->configuration.regs_16bit);
    LOGE_ON_ERROR(VEML_TAG, __func__,
                  "Failed reading VEML7700 Configuring Register", err);
    return err;
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
    esp_err_t err = this->read_from_reg(VEML_ALS_LEVEL_REG, &data);
    LOGE_ON_ERROR(VEML_TAG, __func__, "Failed Reading ALS Register", err);
    return data;
}

// See veml.h for documentation
uint16_t Veml7700::get_white_level(void) {
    uint16_t data;
    esp_err_t err = this->read_from_reg(VEML_WHITE_LEVEL_REG, &data);
    LOGE_ON_ERROR(VEML_TAG, __func__, "Failed reading white level register",
                  err);
    return data;
}

// See veml.h for documentation.
float Veml7700::get_lux(void) {
    return this->get_ambient_level() * this->get_als_scale();
}

// See veml.h for documentation.
veml_gain_options_e Veml7700::get_gain(void) {
    this->get_configuration();
    return (veml_gain_options_e)this->configuration.gain;
}

// See veml.h for documentation.
esp_err_t Veml7700::set_gain(const veml_gain_options_e gain) {
    this->configuration.gain = gain;
    return this->set_configuration();
}

// See veml.h for documentation.
veml_integration_options_e Veml7700::get_integration_time(void) {
    this->get_configuration();
    return (veml_integration_options_e)this->configuration.integration_time;
}

// See veml.h for documentation
esp_err_t Veml7700::set_integration_time(
    const veml_integration_options_e integration_time) {
    this->configuration.integration_time = integration_time;
    return this->set_configuration();
}

// See veml.h for documentation
veml_power_options_e Veml7700::get_power_state(void) {
    this->get_configuration();
    return (veml_power_options_e)this->configuration.shutdown;
}

// See veml.h for documentation
esp_err_t Veml7700::set_power_state(const veml_power_options_e state) {
    this->configuration.shutdown = state;
    return this->set_configuration();
}

// See veml.h for documentation
esp_err_t Veml7700::periodic_process(void) {
    // TODO: High level compensation
    // TODO: Gain/Integration auto-adjust
    esp_err_t result = ESP_OK;
    uint16_t ambient_level = this->get_ambient_level();
    uint16_t white_level = this->get_white_level();
    float computed_lux = ambient_level * this->get_als_scale();

    if (xSemaphoreTake(this->output_mutex, MAX_MUTEX_WAIT_TICKS) == pdTRUE) {
        this->last_output.raw_als = ambient_level;
        this->last_output.raw_white = white_level;
        this->last_output.lux = computed_lux;
        if (xSemaphoreGive(this->output_mutex) != pdTRUE) {
            ESP_LOGE(VEML_TAG,
                     "Failed to release mutex after updating output structure");
            result = ESP_FAIL;
        }
    } else {
        ESP_LOGE(VEML_TAG, "Failed to get mutex for updating output structure");
        result = ESP_ERR_TIMEOUT;
    }
    return result;
}

// see veml.h for documentation
esp_err_t Veml7700::get_outputs(veml_output_t *data) {
    esp_err_t result = ESP_OK;
    if (xSemaphoreTake(this->output_mutex, MAX_MUTEX_WAIT_TICKS) == pdTRUE) {
        (void)memcpy(data, &this->last_output, sizeof(veml_output_t));
        if (xSemaphoreGive(this->output_mutex) != pdTRUE) {
            ESP_LOGE(VEML_TAG,
                     "Failed to release mutex after updating data structure");
            result = ESP_FAIL;
        }
    } else {
        ESP_LOGE(VEML_TAG, "Failed to get mutex for updating data structure");
        result = ESP_ERR_TIMEOUT;
    }
    return result;
}

/*======================================================================
 *                       Private Functions
 *=====================================================================*/

// See veml.h for documentation.
esp_err_t Veml7700::write_to_reg(const uint8_t reg,
                                 const uint16_t *const data) {
    // First byte is  the register
    // Second and third bytes are the data bytes.
    const uint8_t data_high = (*data & 0xF0) >> 8;
    const uint8_t data_low = *data * 0x0F;
    const uint8_t data_buffer[3] = {reg, data_high, data_low};
    return i2c_master_write_to_device(this->i2c_port, VEML_ADDR, data_buffer, 3,
                                      this->wait_time);
}

// See veml.h for documentation
esp_err_t Veml7700::read_from_reg(const uint8_t reg, uint16_t *const data) {
    return i2c_master_write_read_device(this->i2c_port, VEML_ADDR, &reg, 1,
                                        (uint8_t *)data, VEML_REG_BYTES,
                                        this->wait_time);
}

// See veml.h for documentation
float Veml7700::get_als_scale(void) {
    const uint8_t integration_scale = get_integration_scale(
        (veml_integration_options_e)this->configuration.integration_time);
    const uint8_t gain_scale =
        get_gain_scale((veml_gain_options_e)this->configuration.gain);
    return ALS_BASE_SCALE * integration_scale * gain_scale;
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