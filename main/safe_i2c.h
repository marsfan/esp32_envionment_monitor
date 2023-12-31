/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
/// @brief Task-safe I2C implementation
/// @details This provides I2C functions that are protected by a mutex, so that
/// only one task can access the I2C bus at a time.

#ifndef SAFE_I2C_H
#define SAFE_I2C_H

#include <driver/i2c.h>
#include <stdbool.h>

class SafeI2C {
   public:
    /// @brief Construct a SafeI2C instance.
    /// @param i2c_port The I2C port to use
    SafeI2C(const i2c_port_t i2c_port);

    /// @brief Initialize, configure, and start the instance
    /// @param config The I2C Configuration Structure
    /// @return Result of initializing the instance.
    esp_err_t init(const i2c_config_t* config);

    /// @brief Write to a device, then read data back
    /// @param[in] device_address Address of the device to perform the transfer
    /// on
    /// @param[in] write_buffer Buffer of data to write.
    /// @param[in] write_size Size of the buffer to write
    /// @param[out] read_buffer Buffer to hold read data
    /// @param[in] read_size Number of bytes to read
    /// @param[in] ticks_to_wait Maximum ticks to wait before issuing a timeout.
    /// @return Result of the transaction
    esp_err_t master_write_read_device(uint8_t device_address,
                                       const uint8_t* write_buffer,
                                       size_t write_size, uint8_t* read_buffer,
                                       size_t read_size,
                                       TickType_t ticks_to_wait);

    /// @brief Write data to a device
    /// @param[in] device_address The address to write to
    /// @param[in] write_buffer Buffer of data to write
    /// @param[in] write_size The size of buffer to write
    /// @param[in] ticks_to_wait Maximum ticks to wait before issuing a timeout.
    /// @return Result of the transaction
    esp_err_t master_write_to_device(uint8_t device_address,
                                     const uint8_t* write_buffer,
                                     size_t write_size,
                                     TickType_t ticks_to_wait);

   private:
    /// @brief The I2C Port
    i2c_port_t port;

    /// @brief The I2C Config Stuct
    i2c_config_t config;

    /// @brief Mutex to ensure task safety on the I2C bus.
    SemaphoreHandle_t i2c_mutex;

    /// @brief Obtain the mutex
    /// @param[in] func The name of the function that is taking the mutex.
    /// @return Whether or not the mutex was successfully taken.
    bool get_mutex(const char* const func);

    /// @brief Release the mutex
    /// @param[in] func The name of the function that is releasing the mutex
    void release_mutex(const char* const func);
};

#endif  // SAFE_I2C_H