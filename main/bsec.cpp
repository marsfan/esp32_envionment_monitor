/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
#include "bsec.h"

#include <string.h>

#include "bsec/inc/bsec_interface.h"

// See bsec.h for documentation
BSEC::BSEC(const i2c_port_t i2c_port, const TickType_t i2c_wait_time)
    : Bme688(i2c_port, i2c_wait_time) {}

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

        // TODO: Update subscription at provided sample rate.
        // Need new float argument of the sample rate
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
    const uint8_t n_requested_virtual_sensors,
    bsec_sensor_configuration_t *required_sensor_settings,
    uint8_t *n_required_sensor_settings) {
    return bsec_update_subscription(
        requested_virtual_sensors, n_requested_virtual_sensors,
        required_sensor_settings, n_required_sensor_settings);
}

// See bsec.h for documentation
bsec_result_t BSEC::process_data(int64_t timestamp) {
    bsec_result_t result = {.integer_result = 0};
    bsec_bme_settings_t sensor_settings;
    (void)memset(&sensor_settings, 0, sizeof(bsec_bme_settings_t));

    result.bsec_result = bsec_sensor_control(timestamp, &sensor_settings);
    if (result.integer_result == 0) {
        switch (sensor_settings.op_mode) {
            case BME68X_FORCED_MODE: {
            }
                result.sensor_result =
                    this->configure_sensor_forced(&sensor_settings);
                break;
            case BME68X_PARALLEL_MODE:
                result.sensor_result =
                    this->configure_sensor_parallel(&sensor_settings);
                break;
            case BME68X_SLEEP_MODE:
                result.sensor_result = this->set_op_mode(BME68X_SLEEP_MODE);
        }
    }

    if ((result.integer_result == 0) && sensor_settings.trigger_measurement &&
        (sensor_settings.op_mode != BME68X_SLEEP_MODE)) {
        bme68x_data data[3];
        (void)memset(data, 0, sizeof(bme68x_data));
        uint8_t n_data = 0;

        // Read data from the sensor
        result.sensor_result =
            this->get_data(sensor_settings.op_mode, data, &n_data);
        if (result.integer_result == 0) {  // TODO: Send data to be processed
        }
    }

    return result;
}