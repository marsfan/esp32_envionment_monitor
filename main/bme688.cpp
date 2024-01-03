/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */

#include "bme688.h"

#include <esp_log.h>
#include <esp_rom_sys.h>
#include <string.h>

#include "bme68x_sensor_api/bme68x.h"

#define BME_LOG_TAG "BME688"

static void delay(uint32_t period_us, void* intf_ptr);

int8_t i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t length,
                void* intf_pointer);
int8_t i2c_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t length,
                 void* intf_pointer);

// See Bme688.h for documentation
const char* BME_Err_To_String(const int bme_err_code) {
    switch (bme_err_code) {
        case BME68X_OK:
            return "Ok";
        case BME68X_E_NULL_PTR:
            return "Null Pointer";
        case BME68X_E_COM_FAIL:
            return "Communication Failure";
        case BME68X_E_DEV_NOT_FOUND:
            return "Device Not Found";
        case BME68X_E_INVALID_LENGTH:
            return "Incorrect Length Parameter";
        case BME68X_E_SELF_TEST:
            return "Self Test Fail";
        case BME68X_W_DEFINE_OP_MODE:
            return "Define a Operation Mode";
        case BME68X_W_NO_NEW_DATA:
            return "No New Data";
        case BME68X_W_DEFINE_SHD_HEATR_DUR:
            return "Define the shared heating duration";
        default:
            return "Unknown Error";
    }
}

// See bme688.h for documentation
Bme688::Bme688(SafeI2C* i2c_bus, const TickType_t i2c_wait_time) {
    (void)memset(&this->device, 0, sizeof(bme68x_dev));

    this->i2c_bus = i2c_bus;
    this->i2c_wait_time = i2c_wait_time;
    this->device.intf = BME68X_I2C_INTF;
    this->device.delay_us = delay;
    this->device.read = i2c_read;
    this->device.write = i2c_write;
    this->device.amb_temp =
        25;  // In C. All examples I can find set this to 25. IDK why. Datasheet
             // suggests we could also just read from the temp sensor, so IDK
             // why the library does not just do that.

    this->device.intf_ptr = this;

    (void)memset(&this->heater_conf, 0, sizeof(bme68x_heatr_conf));
    this->heater_conf.heatr_dur_prof = this->heater_time_profile;
    this->heater_conf.heatr_temp_prof = this->heater_temp_profile;
}

// See bme688.h for documentation
int8_t Bme688::init(void) {
    return bme68x_init(&this->device);
}

// See bme688.h for documentation
int8_t Bme688::self_test(void) {
    return bme68x_selftest_check(&this->device);
}

// See bme688.h for documentation
int8_t Bme688::soft_reset(void) {
    return bme68x_soft_reset(&this->device);
}

// See bme688.h for documentation
int8_t Bme688::set_op_mode(const uint8_t op_mode) {
    return bme68x_set_op_mode(op_mode, &this->device);
}

// See bme688.h for documentation
int8_t Bme688::get_op_mode(uint8_t* op_mode) {
    return bme68x_get_op_mode(op_mode, &this->device);
}

// See bme688.h for documentation
uint32_t Bme688::get_meas_duration(const uint8_t op_mode) {
    struct bme68x_conf conf;
    int8_t result = BME68X_OK;
    result = this->get_conf(&conf);
    if (result == BME68X_OK) {
        result = bme68x_get_meas_dur(op_mode, &conf, &this->device);
    }
    return result;
}

// See bme688.h for documentation
uint32_t Bme688::get_parallel_delay_period_us(void) {
    return this->get_meas_duration(BME68X_PARALLEL_MODE) +
           this->heater_conf.shared_heatr_dur * 1000;
}

// See bme688.h for documentation
int8_t Bme688::get_data(uint8_t op_mode, struct bme68x_data* data,
                        uint8_t* n_data) {
    return bme68x_get_data(op_mode, data, n_data, &this->device);
}

// See bme688.h for documentation
int8_t Bme688::set_conf(struct bme68x_conf* conf) {
    return bme68x_set_conf(conf, &this->device);
}

// See bme688.h for documentation
int8_t Bme688::set_conf(const uint8_t temperature_oversampling,
                        const uint8_t pressure_oversampling,
                        const uint8_t humidity_oversampling,
                        const uint8_t filter, const uint8_t odr) {
    bme68x_conf config = {
        .os_hum = humidity_oversampling,
        .os_temp = temperature_oversampling,
        .os_pres = pressure_oversampling,
        .filter = filter,
        .odr = odr,
    };

    return this->set_conf(&config);
}

// See bme688.h for documentation
int8_t Bme688::get_conf(struct bme68x_conf* conf) {
    return bme68x_get_conf(conf, &this->device);
}

// See bme688.h for documentation
int8_t Bme688::set_heater_conf_disabled(uint8_t op_mode) {
    (void)memset(&this->heater_conf, 0, sizeof(bme68x_heatr_conf));
    this->heater_conf.enable = BME68X_DISABLE;
    return this->set_heater_conf(op_mode);
}

// See bme688.h for documentation
int8_t Bme688::set_heater_conf_forced(uint16_t temp, uint16_t duration) {
    this->heater_conf.enable = BME68X_ENABLE;
    this->heater_conf.heatr_dur = duration;
    this->heater_conf.heatr_temp = temp;
    return this->set_heater_conf(BME68X_FORCED_MODE);
}

// See bme688.h for documentation
int8_t Bme688::set_heater_conf_parallel(uint16_t* temp_profile,
                                        uint16_t* duration_profile,
                                        uint8_t num_steps) {
    int8_t result = BME68X_OK;
    if (num_steps > 10) {
        result = BME68X_E_INVALID_LENGTH;
        ESP_LOGE(BME_LOG_TAG,
                 "Heater profile has a max of 10 steps. Actual number: %d",
                 num_steps);
    } else {
        // Enable heater.
        this->heater_conf.enable = BME68X_ENABLE;
        // Copy profile to internal data.
        (void)memcpy(this->heater_conf.heatr_temp_prof, temp_profile,
                     sizeof(uint16_t) * num_steps);
        (void)memcpy(this->heater_conf.heatr_dur_prof, duration_profile,
                     sizeof(uint16_t) * num_steps);

        // Shared heating duration in miliseconds
        // Not sure where the 140 comes from. I'm just copying it from the
        // parallel mode example.
        this->heater_conf.shared_heatr_dur =
            (140 - (this->get_meas_duration(BME68X_PARALLEL_MODE)) / 1000);

        // Save the profile length.
        this->heater_conf.profile_len = num_steps;

        // Write configuration to the sensor
        result = this->set_heater_conf(BME68X_PARALLEL_MODE);
    }
    return result;
}

// See bme688.h for documentation
int8_t Bme688::get_heater_conf(const struct bme68x_heatr_conf* conf) {
    return bme68x_get_heatr_conf(conf, &this->device);
}

// See bme688.h for documentation
int8_t Bme688::forced_measurement(struct bme68x_data* data, uint8_t* n_data) {
    int8_t result;

    // Force a measurement.
    result = this->set_op_mode(BME68X_FORCED_MODE);
    if (result == BME68X_OK) {
        // TODO: Support heater sequence as well?
        // TODO: Put this before setting op mode?
        // Compute necessary delay period.
        uint32_t delay_period = this->get_meas_duration(BME68X_FORCED_MODE) +
                                (this->heater_conf.heatr_dur * 1000);
        this->device.delay_us(delay_period, this->device.intf_ptr);

        // Read the data
        result = this->get_data(BME68X_FORCED_MODE, data, n_data);
    }

    return result;
}

// See bme688.h for documentation
SafeI2C* Bme688::get_i2c_bus(void) {
    return this->i2c_bus;
}

// See bme688.h for documentation
TickType_t Bme688::get_i2c_wait_time(void) {
    return this->i2c_wait_time;
}

/********************************************************************
 *                      Class private functions
 *********************************************************************/

// See bme688.h for documentation
int8_t Bme688::set_heater_conf(uint8_t op_mode) {
    return bme68x_set_heatr_conf(op_mode, &this->heater_conf, &this->device);
}

/********************************************************************
 *                      File private functions
 *********************************************************************/

/*!
 * @brief Method for sleeping after an operation.
 * @details If the delay time is less than 1 ms (i.e 1000 us), the ROM function
 * esp_rom_delay_us will be used instead. This operates outside normal RTOS
 * functionality. See https://esp32.com/viewtopic.php?t=880 and
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/internal-unstable.html?highlight=esp_rom_sys#_CPPv416esp_rom_delay_us8uint32_t
 * @param[in] period_us The period to sleep in microseconds
 * @param[in,out] intf_pointer Pointer for linking interface device
 * descriptors for callbacks
 * */
static void delay(uint32_t period_us, void* intf_ptr) {
    TickType_t delay_time = 0;
    if (period_us < 1000) {
        // TODO: ESP_LOGV (verbose) message for if this is used?
        // TODO: SHoudl we enter a critical section here to ensure we don't
        // context
        // switch?https://docs.espressif.com/projects/esp-idf/en/v5.1.2/esp32/api-reference/system/freertos_idf.html
        // TODO: Could also look into copying the code from delayMicroseconds in
        // the arduino HAL.(esp32-hal-misc.c), which seems to use a HW timer and
        // NOPs to sleep
        esp_rom_delay_us(period_us);
        delay_time = 1 / portTICK_PERIOD_MS;
    } else {
        delay_time = period_us / 1000 / portTICK_PERIOD_MS;
    }
    vTaskDelay(delay_time);
}

/*!
 * @brief Method for reading from the sensor over I2C
 * @param[in] reg_addr The register to read from
 * @param[out] reg_data Pointer to an array to hold the read data.
 * @param[in] length Length of data to read
 * @param[in,out] intf_pointer Pointer to the bme688 object to use for
 * communication
 * @return Error result of the operation
 * @retval 0 for success
 * @retval Non-zero for failure.
 */
int8_t i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t length,
                void* intf_pointer) {
    int8_t result = ESP_FAIL;
    if (intf_pointer) {
        Bme688* device = (Bme688*)intf_pointer;
        result = device->get_i2c_bus()->master_write_read_device(
            BME68X_I2C_ADDR_HIGH, &reg_addr, 1, reg_data, length,
            device->get_i2c_wait_time());
    }

    return result != ESP_OK;
}

/*!
 * @brief Method for writing to the sensor over I2C
 * @param[in] reg_addr The register to write to
 * @param[in] reg_data Pointer to an array of the data to write.
 * @param[in] length The length of the data to write
 * @param[in,out] intf_pointer Pointer to the bme688 object to use for
 * communication
 * @return Error result of the operation
 * @retval 0 for success
 * @retval Non-zero for failure.
 */
int8_t i2c_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t length,
                 void* intf_pointer) {
    int8_t err = ESP_FAIL;

    if (intf_pointer) {
        Bme688* device = (Bme688*)intf_pointer;

        // Create new array of bytes to send that is 1 larger than the reg_data
        // array
        uint8_t data[length + 1];

        // Copy register data into the 1..end elements of the array.
        (void)memcpy(&data[1], reg_data, length);

        // Set the first byte of the array to the register address.
        data[0] = reg_addr;

        // Send the data
        err = device->get_i2c_bus()->master_write_to_device(
            BME68X_I2C_ADDR_HIGH, data, length + 1,
            device->get_i2c_wait_time());
    }
    return err;
}