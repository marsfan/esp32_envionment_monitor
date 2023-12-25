/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
#include "bsec.h"

#include <esp_log.h>
#include <string.h>

#include "bsec/inc/bsec_interface.h"

#define LOG_TAG "BSEC"

#define CHECK_INPUT_REQUEST(x, shift) (x & (1 << (shift - 1)))

#define NUM_NON_SCAN_SENSORS 13

static uint8_t add_sig_cond(const int32_t request, const uint8_t input_signal,
                            const float value, const int64_t time_ns,
                            const uint8_t n_inputs, bsec_input_t *inputs);

// See bsec.h for documentation
BSEC::BSEC(const i2c_port_t i2c_port, const TickType_t i2c_wait_time,
           float temp_offset)
    : Bme688(i2c_port, i2c_wait_time) {
    this->temp_offset = temp_offset;
}

// See bsec.h for documentation
bsec_result_t BSEC::init(void) {
    bsec_result_t result = {.integer_result = 0};
    result.sensor_result = Bme688::init();
    if (result.sensor_result == BME68X_OK) {
        result.bsec_result = bsec_init();
    }

    if (result.integer_result == 0) {
        // TODO: Load library config (if available)
        // TODO: Load previous library state (if available)
        // Both should probably be stored in flash.
    }

    return result;
}

// See bsec.h for documentation
bsec_result_t BSEC::init(
    const bsec_sensor_configuration_t *const requested_virtual_sensors,
    const uint8_t n_sensors) {
    bsec_result_t result = this->init();

    if (result.integer_result == 0) {
        result.bsec_result =
            this->update_subscription(requested_virtual_sensors, n_sensors);
    }

    return result;
}

// See bsec.h for documentation
bsec_library_return_t BSEC::get_version(bsec_version_t *bsec_version) {
    return bsec_get_version(bsec_version);
}

// See bsec.h for documentation
bsec_library_return_t BSEC::update_subscription(
    const bsec_sensor_configuration_t *const requested_virtual_sensors,
    const uint8_t n_requested_virtual_sensors) {
    bsec_sensor_configuration_t
        required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;

    return bsec_update_subscription(
        requested_virtual_sensors, n_requested_virtual_sensors,
        required_sensor_settings, &n_required_sensor_settings);
};

// See bsec.h for documentation
bsec_library_return_t BSEC::subscribe_all_non_scan(float sample_rate) {
    bsec_library_return_t result = BSEC_OK;

    const bsec_sensor_configuration_t requested_sensors[NUM_NON_SCAN_SENSORS] =
        {{.sample_rate = sample_rate, .sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE},
         {.sample_rate = sample_rate, .sensor_id = BSEC_OUTPUT_RAW_PRESSURE},
         {.sample_rate = sample_rate, .sensor_id = BSEC_OUTPUT_RAW_HUMIDITY},
         {.sample_rate = sample_rate, .sensor_id = BSEC_OUTPUT_RAW_GAS},
         {.sample_rate = sample_rate, .sensor_id = BSEC_OUTPUT_IAQ},
         {.sample_rate = sample_rate, .sensor_id = BSEC_OUTPUT_STATIC_IAQ},
         {.sample_rate = sample_rate, .sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT},
         {.sample_rate = sample_rate,
          .sensor_id = BSEC_OUTPUT_BREATH_VOC_EQUIVALENT},
         {.sample_rate = sample_rate,
          .sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE},
         {.sample_rate = sample_rate,
          .sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY},
         {.sample_rate = sample_rate,
          .sensor_id = BSEC_OUTPUT_STABILIZATION_STATUS},
         {.sample_rate = sample_rate, .sensor_id = BSEC_OUTPUT_RUN_IN_STATUS},
         {.sample_rate = sample_rate, .sensor_id = BSEC_OUTPUT_GAS_PERCENTAGE}

        };

    this->update_subscription(requested_sensors, NUM_NON_SCAN_SENSORS);

    return result;
}
// See bsec.h for documentation
bsec_result_t BSEC::periodic_process(int64_t timestamp_ns) {
    bsec_result_t result = {.integer_result = 0};
    bsec_bme_settings_t sensor_settings;
    (void)memset(&sensor_settings, 0, sizeof(bsec_bme_settings_t));

    // Update the sensor configuration as requested.
    result.bsec_result =
        bsec_sensor_control(timestamp_ns, &this->sensor_settings);

    if (result.integer_result == 0) {
        switch (this->sensor_settings.op_mode) {
            case BME68X_FORCED_MODE: {
            }
                result.sensor_result = this->configure_sensor_forced();
                break;
            case BME68X_PARALLEL_MODE:
                result.sensor_result = this->configure_sensor_parallel();
                break;
            case BME68X_SLEEP_MODE:
                result.sensor_result = this->set_op_mode(BME68X_SLEEP_MODE);
        }
    }

    // If requested, read data from the sensor and process it.
    if ((result.integer_result == 0) &&
        this->sensor_settings.trigger_measurement &&
        (this->sensor_settings.op_mode != BME68X_SLEEP_MODE)) {
        bme68x_data data[3];
        (void)memset(data, 0, sizeof(bme68x_data));
        uint8_t n_data = 0;

        // Read data from the sensor
        result.sensor_result =
            this->get_data(this->sensor_settings.op_mode, data, &n_data);
        if (result.integer_result == 0 && n_data > 0) {
            for (uint32_t i = 0; i < n_data; i++) {
                result.bsec_result = this->process_data(timestamp_ns, data[i]);
                if (result.integer_result != 0) {
                    break;
                }
            }
        }
    }

    return result;
}

// See bsec.h for documentation
void BSEC::get_output(bsec_output_t *outputs, uint8_t *num_outputs) {
    (void)memset(outputs, 0, sizeof(bsec_output_t) * BSEC_NUMBER_OUTPUTS);

    // Copy outputs to the array
    (void)memcpy(outputs, this->outputs,
                 this->num_outputs * sizeof(bsec_output_t));

    // Copy the number of outputs to the num_outputs
    *num_outputs = this->num_outputs;
}

// See bsec.h for documentation
int64_t BSEC::get_next_call_time(void) {
    return this->sensor_settings.next_call;
}

int8_t BSEC::configure_sensor_forced(void) {
    int8_t result = BME_OK;
    struct bme68x_conf conf;
    result = this->get_conf(&conf);
    BME_LOGE_ON_ERR(LOG_TAG, __func__, "Failed getting sensor configuration",
                    result);

    if (result == BME68X_OK) {
        conf.os_hum = this->sensor_settings.humidity_oversampling;
        conf.os_temp = this->sensor_settings.temperature_oversampling;
        conf.os_pres = this->sensor_settings.pressure_oversampling;
        result = this->set_conf(&conf);
        BME_LOGE_ON_ERR(LOG_TAG, __func__,
                        "Failed setting sensor configuration", result);
    }

    if (result == BME68X_OK) {
        result = this->set_heater_conf_forced(
            this->sensor_settings.heater_temperature,
            this->sensor_settings.heater_duration);
        BME_LOGE_ON_ERR(LOG_TAG, __func__,
                        "Failed setting sensor heater configuration", result);
    }

    if (result == BME68X_OK) {
        result = this->set_op_mode(BME68X_FORCED_MODE);
        BME_LOGE_ON_ERR(LOG_TAG, __func__, "Failed setting sensor op mode",
                        result);
    }
    return result;
}

int8_t BSEC::configure_sensor_parallel(void) {
    int8_t result = BME_OK;
    struct bme68x_conf conf;
    result = this->get_conf(&conf);
    BME_LOGE_ON_ERR(LOG_TAG, __func__, "Failed getting sensor configuration",
                    result);

    if (result == BME68X_OK) {
        conf.os_hum = this->sensor_settings.humidity_oversampling;
        conf.os_temp = this->sensor_settings.temperature_oversampling;
        conf.os_pres = this->sensor_settings.pressure_oversampling;
        result = this->set_conf(&conf);
        BME_LOGE_ON_ERR(LOG_TAG, __func__,
                        "Failed setting sensor configuration", result);
    }

    if (result == BME68X_OK) {
        result = this->set_heater_conf_parallel(
            this->sensor_settings.heater_temperature_profile,
            this->sensor_settings.heater_duration_profile,
            this->sensor_settings.heater_profile_len);
        BME_LOGE_ON_ERR(LOG_TAG, __func__,
                        "Failed setting sensor heater configuration", result);
    }

    if (result == BME68X_OK) {
        result = this->set_op_mode(BME68X_PARALLEL_MODE);
        BME_LOGE_ON_ERR(LOG_TAG, __func__, "Failed setting sensor op mode",
                        result);
    }
    return result;
}

bsec_library_return_t BSEC::process_data(int64_t curr_time_ns,
                                         struct bme68x_data data) {
    bsec_library_return_t result = BSEC_OK;
    bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_inputs = 0;
    (void)memset(inputs, 0, sizeof(bsec_input_t) * BSEC_MAX_PHYSICAL_SENSOR);

    // Add pressure info if requested.
    n_inputs =
        add_sig_cond(this->sensor_settings.process_data, BSEC_INPUT_PRESSURE,
                     data.pressure, curr_time_ns, n_inputs, inputs);

    // Add humidity info if requested
    n_inputs =
        add_sig_cond(this->sensor_settings.process_data, BSEC_INPUT_HUMIDITY,
                     data.humidity, curr_time_ns, n_inputs, inputs);

    // Add temp info if requested
    n_inputs =
        add_sig_cond(this->sensor_settings.process_data, BSEC_INPUT_TEMPERATURE,
                     data.temperature, curr_time_ns, n_inputs, inputs);

    // Add gas resistance if requested
    n_inputs =
        add_sig_cond(this->sensor_settings.process_data, BSEC_INPUT_GASRESISTOR,
                     data.gas_resistance, curr_time_ns, n_inputs, inputs);

    // Add temp offset if requested
    n_inputs =
        add_sig_cond(this->sensor_settings.process_data, BSEC_INPUT_HEATSOURCE,
                     this->temp_offset, curr_time_ns, n_inputs, inputs);

    // TODO: BSEC_INPUT_DISABLE_BASELINE_TRACKER

    // TODO: Not 100% sure what this is. Need to check datasheet
    n_inputs = add_sig_cond(
        this->sensor_settings.process_data, BSEC_INPUT_PROFILE_PART,
        (this->sensor_settings.op_mode == BME68X_FORCED_MODE) ? 0
                                                              : data.gas_index,
        curr_time_ns, n_inputs, inputs);

    if (n_inputs > 0) {
        /// Set the num outputs to the max we have memory to support.
        // See 3.3.1.4 in the integration guide
        this->num_outputs = BSEC_NUMBER_OUTPUTS;
        (void)memset(this->outputs, 0, sizeof(this->outputs));
        result =
            bsec_do_steps(inputs, n_inputs, this->outputs, &this->num_outputs);
    }

    return result;
}

/// @brief Conditionally add a value to the inputs array
/// @param request The request bitfield
/// @param input_signal The signal type to add conditionally
/// @param value The value to add
/// @param time_ns Current time in ns
/// @param n_inputs Current number of inputs
/// @param inputs The input array
/// @return The new number of inputs
// FIXME: Make this part of the class, use members to reduce arguments
static uint8_t add_sig_cond(const int32_t request, const uint8_t input_signal,
                            const float value, const int64_t time_ns,
                            const uint8_t n_inputs, bsec_input_t *inputs) {
    if (CHECK_INPUT_REQUEST(request, input_signal)) {
        inputs[n_inputs].sensor_id = input_signal;
        inputs[n_inputs].signal = value;
        inputs[n_inputs].time_stamp = time_ns;
        return n_inputs + 1;
    }
    return n_inputs;
}