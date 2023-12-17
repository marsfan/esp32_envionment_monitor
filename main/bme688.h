/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
/// @brief Implementation for reading data from the BME688 sensor

#ifndef BME688_H
#define BME688_H

#include "bme68x_sensor_api/bme68x_defs.h"
#include "driver/i2c.h"

// TODO: Error code enum
// TODO: Enum for the op mode (used in get/set)
// TODO: Store op_mode in the class so we don't need to keep passing it in to
// other methods?

/// @brief Convert BME688 error codes to strings
/// @param bme_err_code The error code to convert.
/// @return String representation of the error code.
const char *BME_Err_To_String(const int bme_err_code);

/// @brief BME688 sensor interation class
class Bme688 {
   public:
    /*!
     * @brief Instantiate the device.
     * @param[in] i2c_port The I2C port to use for communicating with the
     * device.
     * @param[in] wait_time The max wait time after an I2C operation.
     */
    Bme688(const i2c_port_t i2c_port, const TickType_t wait_time);

    /*!
     * @brief Initialize the sensor.
     * @return Result of initialization.
     * @retval 0: Success
     * @retval <0: Failure
     * */
    int8_t init(void);

    /*!
     * @brief Perform a selftest on the sensor.
     * @return Result of the self test.
     * @retval 0: Success
     * @retval <0: Failure
     * */
    int8_t self_test(void);

    /*!
     * @brief Perform a soft reset on the sensor.
     * @return Result of soft reset.
     * @retval 0: Success
     * @retval <0: Failure
     * */
    int8_t soft_reset(void);

    /*!
     * @brief Set the operation mode of the sensor.
     * @param[in] op_mode The operating mode to set.
     * @return Result of setting the operation mode.
     * @retval 0: Success
     * @retval <0: Failure
     * */
    int8_t set_op_mode(const uint8_t op_mode);

    /*!
     * @brief Set the operation mode of the sensor.
     * @param[out] op_mode The operating mode of the sensor.
     * @return Result of getting the operation mode.
     * @retval 0: Success
     * @retval <0: Failure
     * */
    int8_t get_op_mode(uint8_t *op_mode);

    /*!
     * @brief Get the remaining duration that can be used for heating.
     * @param[in] op_mode The operating mode of the sensor.
     * @param[in] conf The sensor configuration to use.
     * @return Measurement duration in microseconds
     * */
    uint32_t get_meas_duration(const uint8_t op_mode, struct bme68x_conf *conf);

    /*!
     * @brief Read the data from the sensor
     * @param[in] op_mode The operating mode of the sensor.
     * @param[in] data Structure to hold the data
     * @param[out] n_data Number of data instances available.
     * @return Result of getting sensor data.
     * @retval 0: Success
     * @retval <0: Failure
     * */
    int8_t get_data(uint8_t op_mode, struct bme68x_data *data, uint8_t *n_data);

    /*!
     * @brief Set the sensor configurations
     * @param[in] conf The sensor configuration
     * @return Result of setting the sensor configuration.
     * @retval 0: Success
     * @retval <0: Failure
     * */
    int8_t set_conf(struct bme68x_conf *conf);

    /*!
     * @brief Get the sensor configurations
     * @param[out] conf The sensor configuration
     * @return Result of getting the sensor configuration.
     * @retval 0: Success
     * @retval <0: Failure
     * */
    int8_t get_conf(struct bme68x_conf *conf);

    /*!
     * @brief Set the sensor heating configuration.
     * @param[in] op_mode THe opearting mode of the sensor.
     * @param[in] conf The heating configuration to set
     * @return Result of setting the sensor heater configuration.
     * @retval 0: Success
     * @retval <0: Failure
     * */
    int8_t set_heater_conf(uint8_t op_mode,
                           const struct bme68x_heatr_conf *conf);

    /*!
     * @brief Get the sensor heating configuration.
     * @param[out] conf The heaterr configuration.
     * @return Result of getting the sensor heater configuration.
     * @retval 0: Success
     * @retval <0: Failure
     * */
    int8_t get_heater_conf(const struct bme68x_heatr_conf *conf);

    /*!
     * @brief Get the i2c_port used by the device
     * @return The i2c port used by the device.
     */
    i2c_port_t get_i2c_port(void);

    /*!
     * @brief Get the wait time to be used for i2c operations.
     * @return The wait time to be used for i2c operations.
     */
    TickType_t get_wait_time(void);

   private:
    ///@brief I2C port to use
    i2c_port_t i2c_port;

    /// @brief max Time to wait after a I2C transation
    TickType_t wait_time;

    /// @brief BME68x device structure
    bme68x_dev device;
};
#endif  // BME688_H