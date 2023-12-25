/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
#ifndef BSEC_H
#define BSEC_H
#include "bme688.h"
#include "bsec/inc/bsec_datatypes.h"

/// @brief Result of initializing both sensor and bsec.
typedef union {
    struct {
        bsec_library_return_t bsec_result;  ///< BSEC initialization result
        int8_t sensor_result;               ///< Sensor initialization result
    };                                      // Deliberately unnamed.

    uint64_t integer_result;  ///< Integer result for faster compare. Will be 0
                              ///< on success

} bsec_result_t;

/// @brief BSEC Implementation in C++
class BSEC : private Bme688 {
   public:
    /// @brief Instantiate the device for use with the BSEC system
    /// @param i2c_port The I2C port to use for communicating with the sensor.
    /// @param i2c_wait_time THe max wait time after an i2c operation
    /// @param temp_offset Offset to to apply to the temperature measurement, to
    /// correct for sensor or enclosure bias.
    BSEC(const i2c_port_t i2c_port, const TickType_t i2c_wait_time,
         float temp_offset);

    /// @brief Initialize BSEC and the sensor
    /// @return Result of the initialization.
    bsec_result_t init(void);

    /// @brief Initialize both the sensor, and the BSEC library, and subscribe
    /// to inputs.
    /// @param requested_virtual_sensors Initial virtual sensor subscription
    /// @param n_sensors The total number of requested virtual sensors
    /// @return Result of the initialization
    bsec_result_t init(
        const bsec_sensor_configuration_t *const requested_virtual_sensors,
        const uint8_t n_sensors);

    /// @brief Get the version of the BSEC library
    /// @param bsec_version Pointer to a structure to hold the read version.
    /// @return Result of getting the version.
    bsec_library_return_t get_version(bsec_version_t *bsec_version);

    /// @brief Update the requested sensor data.
    /// @param requested_virtual_sensors The requested virtual sensors to get
    /// @param n_requested_virtual_sensors The total number of requested virtual
    /// sensors
    /// @param required_sensor_settings Pointer to a structure to hold the
    /// required physical sensors
    /// @param n_required_sensor_settings Pointer to the total number of
    /// required sensors.
    /// @return Result of updating the requested virtual sensors
    bsec_library_return_t update_subscription(
        const bsec_sensor_configuration_t *const requested_virtual_sensors,
        const uint8_t n_requested_virtual_sensors);

    /// @brief Subscribe to all non gas-scan mode virtual sensors.
    /// @param refresh_rate The refresh rate to subscribe all sensors to
    /// @return Result of subscribing to the sensors.
    bsec_library_return_t subscribe_all_non_scan(float sample_rate);

    /// @brief Read data from the sensor and process it
    /// @param timestamp_ns Current system timestamp in microseconds
    /// @return Result of reading and processing the data.
    bsec_result_t periodic_process(int64_t timestamp_ns);

    /// @brief Get the most recent set of output data
    /// @param outputs The most recent set of output data. Must be an array of
    /// length BSEC_NUMBER_OUTPUTS
    /// @param num_outputs The total number of the most recent outputs
    void get_output(bsec_output_t *outputs, uint8_t *num_outputs);

   private:
    /// @brief Configure the sensor for a forced measurement
    /// @param sensor_settings The settings to configure the sensor
    /// @return Result of configuring the sensor.
    /// @retval 0: Success
    /// @retval <0: Fail
    int8_t configure_sensor_forced(bsec_bme_settings_t *sensor_settings);

    /// @brief Configure the sensor for a parallel measurement
    /// @param sensor_settings The settings to configure the sensor
    /// @return Result of configuring the sensor.
    /// @retval 0: Success
    /// @retval <0: Fail
    int8_t configure_sensor_parallel(bsec_bme_settings_t *sensor_settings);

    /// @brief Process the data and update internal record of most recent data
    /// @param curr_time_ns The current timestamp in nanoseconds
    /// @param data The data from the sensor to process
    /// @param sensor_settings The requested sensor settings
    /// @return Result of processing the data
    bsec_library_return_t process_data(int64_t curr_time_ns,
                                       struct bme68x_data data,
                                       bsec_bme_settings_t *sensor_settings);

    /// @brief Output data from BSEC
    bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];

    /// @brief number of outputs from last run.
    uint8_t num_outputs;

    /// @brief  Offset to to apply to the temperature measurement, to
    /// correct for sensor or enclosure bias.
    float temp_offset;
};
#endif  // BSEC_H