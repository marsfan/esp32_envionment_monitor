/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
#ifndef BSEC_H
#define BSEC_H
#include <assert.h>
#include <stdbool.h>

#include "bme688.h"
#include "bsec/inc/bsec_datatypes.h"

// TODO: Logging/reporting for error messages.
// TODO: Support for ULP ON Demand sampling
// TODO: Support storing and loading state and configuration from flash
// TODO: Periodic_process skips over if too early?

/// @brief Result of initializing both sensor and bsec.
typedef union {
    struct {
        bsec_library_return_t bsec_result;  ///< BSEC initialization result
        int8_t sensor_result;               ///< Sensor initialization result
    };                                      // Deliberately unnamed.

    uint64_t integer_result;  ///< Integer result for faster compare. Will be 0
                              ///< on success

} bsec_result_t;

/// @brief Special version of bsec_output_t
/// @details This contains almost the same data as bsec_output_t, but with the
/// additional benefit of also having a .valid member to indicate if the given
/// output signal was provided at the most recent periodic processing iteration
/// See the documentation for bsec_output_t for further details
typedef struct {
    int64_t time_stamp;         ///< Time stamp (in ns) of the signal generation
    float signal;               ///< The value of the output;
    uint8_t signal_dimensions;  ///< Reserved for future use
    uint8_t accuracy;           ///< Accuracy indication of the data
    bool valid;  ///< Indication that the data in the struct is valid

} bsec_virtual_sensor_data_t;

/// @brief Well-structured collection of BSEC virtual sensors
/// @details This is used to provide the same information as in the normal
/// output array, except in a more well-formatted form, so finding the correct
/// sensor output does not require searching through the array
/// @ref bsec_virtual_sensor_t for documentation on the virtual sensors
typedef struct {
    bsec_virtual_sensor_data_t iaq;            ///< Indoor air quality
    bsec_virtual_sensor_data_t static_iaq;     ///< Unscaled indoor air quality
    bsec_virtual_sensor_data_t co2_eq;         ///< Equivlent CO2 estimate (ppm)
    bsec_virtual_sensor_data_t breath_voc_eq;  ///< Breath VOC estimate (ppm)
    bsec_virtual_sensor_data_t raw_temp;       ///< Raw temperature (degrees C)
    bsec_virtual_sensor_data_t raw_pressure;   ///< Raw pressure (Pa)
    bsec_virtual_sensor_data_t raw_humidity;   ///< Raw humidity (%)
    bsec_virtual_sensor_data_t raw_gas;        ///< Raw gas sensor (Ohm)
    bsec_virtual_sensor_data_t stabilization_status;  ///< Stabilization status
    bsec_virtual_sensor_data_t run_in_status;         ///< Sensor Run in status
    bsec_virtual_sensor_data_t compensated_temp;  ///< Heat Compensated temp (C)
    bsec_virtual_sensor_data_t
        compensated_humidity;  ///< Heat Compensated Humidity (C)
    bsec_virtual_sensor_data_t
        gas_percentage;  ///< Percentage of min/max filter gas (%)
    bsec_virtual_sensor_data_t gas_estimate_1;  ///< Gas channel 1 estimate
    bsec_virtual_sensor_data_t gas_estimate_2;  ///< Gas channel 2 estimate
    bsec_virtual_sensor_data_t gas_estimate_3;  ///< Gas channel 3 estimate
    bsec_virtual_sensor_data_t gas_estimate_4;  ///< Gas channel 4 estimate
    bsec_virtual_sensor_data_t raw_gas_index;   ///< gas heater profile index.
} bsec_structured_outputs_t;

static_assert((BSEC_NUMBER_OUTPUTS - 1) == (sizeof(bsec_structured_outputs_t) /
                                            sizeof(bsec_virtual_sensor_data_t)),
              "Incorrect number of entires for sensor data");

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

    /// @brief Get the most recent set of output data from the class
    /// @details This function uses a mutex so it should be thread safe
    /// @param output_data Pointer to hold the output data.
    void get_output_data(bsec_structured_outputs_t *output_data);

    /// @brief Get the timestamp (in ns) for when the next call to
    /// periodic_process should occur
    /// @return Timestamp (in ns) to when the next call to periodic_process
    /// should occur
    int64_t get_next_call_time(void);

    /// @brief Get the next call time in microseconds
    /// @return Time at which periodic_processing should be called, in
    /// microseconds
    int64_t get_next_call_time_us(void);

   private:
    /// @brief Configure the sensor for a forced measurement
    /// @return Result of configuring the sensor.
    /// @retval 0: Success
    /// @retval <0: Fail
    int8_t configure_sensor_forced(void);

    /// @brief Configure the sensor for a parallel measurement
    /// @return Result of configuring the sensor.
    /// @retval 0: Success
    /// @retval <0: Fail
    int8_t configure_sensor_parallel(void);

    /// @brief Process the data and update internal record of most recent data
    /// @param data The data from the sensor to process
    /// @return Result of processing the data
    bsec_library_return_t process_data(struct bme68x_data data);

    /// @brief Conditionally add a value to the inputs array used for updating a
    /// subscription
    /// @param input_signal The signal type to add conditionally
    /// @param value The value to add
    /// @param n_inputs Current number of inputs
    /// @param inputs The input array
    /// @return The new number of inputs
    uint8_t add_sig_cond(const uint8_t input_signal, const float value,
                         const uint8_t n_inputs, bsec_input_t *inputs);

    // TODO: Update docs once it is thread safe
    /// @brief Update the outputs structure with newly read sensor data
    /// @details This uses a mutex, so it should be thread safe.
    /// @param outputs The array of output data from the BSEC library
    /// @param num_outputs The number of entries in the output array
    void update_output_structure(bsec_output_t *outputs,
                                 const uint8_t num_outputs);

    /// @brief Output data from BSEC
    bsec_structured_outputs_t outputs;

    /// @brief Mutex for ensuring thread safety on the output data
    SemaphoreHandle_t output_mutex;

    /// @brief  Offset to to apply to the temperature measurement, to
    /// correct for sensor or enclosure bias.
    float temp_offset;

    /// @brief The most recently read sensor settings.
    bsec_bme_settings_t sensor_settings;

    /// @brief Current periodic_processing iteration time (in ns)
    int64_t curr_time_ns;
};
#endif  // BSEC_H