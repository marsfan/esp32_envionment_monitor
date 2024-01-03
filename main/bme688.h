/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
/// @brief Implementation for reading data from the BME688 sensor

#ifndef BME688_H
#define BME688_H

#include "bme68x_sensor_api/bme68x_defs.h"
#include "safe_i2c.h"

// TODO: Error code enum
// TODO: Enum for the op mode (used in get/set)
// TODO: Store op_mode in the class so we don't need to keep passing it in to
// other methods?

/// @brief Log BME error if it happens.

#define BME_LOGE_ON_ERR(tag, func, msg, err)                                \
    if (err != BME68X_OK) {                                                 \
        ESP_LOGE(tag, "Error | %s | %s | %s", func, BME_Err_To_String(err), \
                 msg);                                                      \
    }

typedef enum {
    // Errors
    BME_SELF_TEST_ERROR = BME68X_E_SELF_TEST,
    BME_INVALID_LENGTH = BME68X_E_INVALID_LENGTH,
    BME_DEVICE_NOT_FOUND = BME68X_E_DEV_NOT_FOUND,
    BME_COM_FAIL = BME68X_E_COM_FAIL,
    BME_NULL_PTR = BME68X_E_NULL_PTR,

    // Ok
    BME_OK = BME68X_OK,

    // Warnings
    BME_DEFINE_OP_MODE = BME68X_W_DEFINE_OP_MODE,
    BME_NO_NEW_DATA = BME68X_W_NO_NEW_DATA,
    BME_DEFINE_HEATER = BME68X_W_DEFINE_SHD_HEATR_DUR,

    // Information
    BME_PARAM_CORR = BME68X_I_PARAM_CORR,

} bme_status_e;

/// @brief Convert BME688 error codes to strings
/// @param bme_err_code The error code to convert.
/// @return String representation of the error code.
const char *BME_Err_To_String(const int bme_err_code);

/// @brief BME688 sensor interation class
class Bme688 {
   public:
    /*!
     * @brief Instantiate the device.
     * @param[in] i2c_bus Pointer to the SafeI2C bus to use for communication
     * @param[in] i2c_wait_time The max wait time after an I2C operation.
     */
    Bme688(SafeI2C *i2c_bus, const TickType_t i2c_wait_time);

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
     * @return Measurement duration in microseconds
     * */
    uint32_t get_meas_duration(const uint8_t op_mode);

    /// @brief Get minimum delay period between parallel mode read_result
    /// @details Between each call to @ref get_data, the systsem should wait at
    /// least the amount of time that this method returns, to ensure that the
    /// full sensor reading has been completed.
    /// @return The delay period in microseconds to wait between each
    /// measurement.
    uint32_t get_parallel_delay_period_us(void);

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
     * @brief Set the sensor configurations
     * @param[in] temperature_oversampling Temperature oversampling amount
     * @param[in] pressure_oversampling Pressure oversampling amount
     * @param[in] humidity_oversampling Humidity oversampling amount
     * @param[in] filter The filter coefficient to use
     * @param[in] odr Standby time between sequential mode measurement profiles.
     * @return Result of setting the sensor configuration.
     * @retval 0: Success
     * @retval <0: Failure
     * */
    // FIXME: Enumerations
    int8_t set_conf(const uint8_t temperature_oversampling,
                    const uint8_t pressure_oversampling,
                    const uint8_t humidity_oversampling, const uint8_t filter,
                    const uint8_t odr);
    /*!
     * @brief Get the sensor configurations
     * @param[out] conf The sensor configuration
     * @return Result of getting the sensor configuration.
     * @retval 0: Success
     * @retval <0: Failure
     * */
    int8_t get_conf(struct bme68x_conf *conf);

    /*!
     * @brief Set the heater config to be disabled.
     * @param op_mode The operating mode for the sensor
     * @return Result of setting the sensor heater configuration.
     * @retval 0: Success
     * @retval <0: Failure
     */
    int8_t set_heater_conf_disabled(uint8_t op_mode);

    /*!
     * @brief Set the heater config for a forced mode reading.
     * @param temp The temperature (in C) to run the heater in forced mode.
     * @param duration The heating duration (in ms) to run the heater in forced
     * mode
     * @return Result of setting the sensor heater configuration.
     * @retval 0: Success
     * @retval <0: Failure
     */
    int8_t set_heater_conf_forced(uint16_t temp, uint16_t duration);

    /*!
     * @brief Set the heater config for sequential mode.
     * @details The arrays passed in will be copied to internal variables that
     * store the profiles. The temp_profile and duration_profile arguments MUST
     * point to arrays of the same size, and these arrays MUST be 10 or less
     * element.
     * @param temp_profile Pointer to an array (max 10 values) of the heater
     * temperature profile.
     * @param duration_profile Pointerr to an array (max 10 values) of the
     * heater time profile
     * @param num_steps The number of stages (elements) in the temp_profile and
     * duration_profile arguments.
     * @return Result of setting the sensor heater configuration.
     * @retval 0: Success
     * @retval <0: Failure
     */
    int8_t set_heater_conf_parallel(uint16_t *temp_profile,
                                    uint16_t *duration_profile,
                                    uint8_t num_steps);

    /*!
     * @brief Get the sensor heating configuration.
     * @note. If the structure passed in does not have valid pointers for
     * populating the heater profile, this will return a null pointer error code
     * @param[out] conf The heaterr configuration.
     * @return Result of getting the sensor heater configuration.
     * @retval 0: Success
     * @retval <0: Failure
     * */
    int8_t get_heater_conf(const struct bme68x_heatr_conf *conf);

    /// @brief Perform a single forced measurement.
    /// @details This will perform a single forced measurement on the BME688.
    /// There is a delay involved in this, so it will briefly block the running
    /// thread.
    /// @note When using this method, the heater should be set to a single
    /// period, not a sequence.
    /// @param data Pointer to the structure to hold the read data.
    /// @param n_data The number of data samples. Should always be 1 I think
    /// @return Result of performing the measurement.
    // TODO: Confirm that n_data is always 1 for forced measuremnt.
    int8_t forced_measurement(bme68x_data *data, uint8_t *n_data);

    /*!
     * @brief Get the i2c_bus used by the device
     * @return The i2c_bus used by the device.
     */
    SafeI2C *get_i2c_bus(void);

    /*!
     * @brief Get the wait time to be used for i2c operations.
     * @return The wait time to be used for i2c operations.
     */
    TickType_t get_i2c_wait_time(void);

   private:
    ///@brief SafeI2C Bus to use for transations.
    SafeI2C *i2c_bus;

    /// @brief max Time to wait after a I2C transation
    TickType_t i2c_wait_time;

    /// @brief BME68x device structure
    bme68x_dev device;

    /// @brief The last heater configuration that was sent.
    bme68x_heatr_conf heater_conf;

    /// @brief Temperature profile for sequence mode
    uint16_t heater_temp_profile[10];

    /// @brief Time profile for sequence mode.
    uint16_t heater_time_profile[10];

    /*!
     * @brief Send the class's heating configuration to the sensor.
     * @param[in] op_mode The opearting mode of the sensor.
     * @return Result of setting the sensor heater configuration.
     * @retval 0: Success
     * @retval <0: Failure
     * */
    int8_t set_heater_conf(uint8_t op_mode);
};
#endif  // BME688_H