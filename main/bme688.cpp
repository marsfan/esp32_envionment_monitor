/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */

#include "bme688.h"

#include <string.h>

#include "bme68x_sensor_api/bme68x.h"

static void delay(uint32_t period_us, void* intf_ptr);

int8_t i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t length,
                void* intf_pointer);
int8_t i2c_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t length,
                 void* intf_pointer);

// See Bme688.h for documentation
const char* BME_Err_To_String(const int bme_err_code) {
    switch (bme_err_code) {
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
        default:
            return "Unknown Error";
    }
}

// See bme688.h for documentation
Bme688::Bme688(const i2c_port_t i2c_port, const TickType_t wait_time) {
    (void)memset(&this->device, 0, sizeof(bme68x_dev));

    this->i2c_port = i2c_port;
    this->wait_time = wait_time;
    this->device.intf = BME68X_I2C_INTF;
    this->device.delay_us = delay;
    this->device.read = i2c_read;
    this->device.write = i2c_write;
    this->device.amb_temp =
        25;  // In C. All examples I can find set this to 25. IDK why. Datasheet
             // suggests we could also just read from the temp sensor, so IDK
             // why the library does not just do that.

    this->device.intf_ptr = this;
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
uint32_t Bme688::get_meas_duration(const uint8_t op_mode,
                                   struct bme68x_conf* conf) {
    return bme68x_get_meas_dur(op_mode, conf, &this->device);
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
int8_t Bme688::get_conf(struct bme68x_conf* conf) {
    return bme68x_get_conf(conf, &this->device);
}

// See bme688.h for documentation
int8_t Bme688::set_heater_conf(uint8_t op_mode,
                               const struct bme68x_heatr_conf* conf) {
    return bme68x_set_heatr_conf(op_mode, conf, &this->device);
}

// See bme688.h for documentation
int8_t Bme688::get_heater_conf(const struct bme68x_heatr_conf* conf) {
    return bme68x_get_heatr_conf(conf, &this->device);
}

// See bme688.h for documentation
i2c_port_t Bme688::get_i2c_port(void) {
    return this->i2c_port;
}

// See bme688.h for documentation
TickType_t Bme688::get_wait_time(void) {
    return this->wait_time;
}
/*!
 * @brief Method for sleeping after an operation.
 * @details Due to the limitations of the ESP32 RTOS, will always sleep for
 * at least 1ms
 * @param[in] period_us The period to sleep in microseconds
 * @param[in,out] intf_pointer Pointer for linking interface device
 * descriptors for callbacks
 * */
static void delay(uint32_t period_us, void* intf_ptr) {
    TickType_t delay_time = 0;
    if (period_us / 1000 < 1) {
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
        result = i2c_master_write_read_device(
            device->get_i2c_port(), BME68X_I2C_ADDR_HIGH, &reg_addr, 1,
            reg_data, length, device->get_wait_time());
    }

    return result != ESP_OK;
}

/*!
 * @brief Method for writing to the sensor over I2C
 * @param[in] The register to write to
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
        err = i2c_master_write_to_device(device->get_i2c_port(),
                                         BME68X_I2C_ADDR_HIGH, data, length + 1,
                                         device->get_wait_time());
    }
    return err;
}