/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */
/// @brief Sensor hub for collecting all of the data from the sensors.
/// @details This class is used by the Sensor Hub task to collect the data from
/// the various sensors and make it available to any processes that may request
/// it. Sensor data is placed onto a queue. The sensor hub reads the item from
/// the queue and stores them in its memory. Other tasks can then request some
/// or all of the data using mutex-guarded functions.
/// An additional benefit of having a separate task for collecting all of the
/// sensor data is that this task can ingest data different methods (i2c, uart,
/// etc), and then delegate other potential operations back to the sensor tasks
/// if necessary.
#ifndef SENSOR_HUB_H
#define SENSOR_HUB_H

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "bsec.h"
#include "mutex_base.h"
#include "veml.h"

/// @brief Enumeration of the different sensor types used in the project.
// TODO: Maybe this should go in common?
typedef enum {
    SENSOR_BSEC = 0,  ///< BME688 Sensor Data
    SENSOR_VEML7700,  ///< VEML7700 Data
} sensor_queue_type;

/// @brief Structure used to send data in the queue
/// TODO: Maybe this should go in common?
typedef struct {
    sensor_queue_type type;  ///< The type of data being sent.

    /// @brief Union of the different types of data that may be sent
    union {
        bsec_structured_outputs_t bsec;  ///< BSEC sensor data
        veml_output_t veml;              ///< VEML7700 sensor data
    } data;
} sensor_hub_queue_entry_t;

/// @brief Structure of the data stored in the sensor hub.
typedef struct {
    bsec_structured_outputs_t bsec;
    veml_output_t veml;
} sensor_hub_data_t;

class SensorHub : protected MutexBase {
   public:
    /// @brief Create the sensor hub and its queues.
    SensorHub(void);

    /// @brief Add data into the queue for the sensor hub to read.
    /// @details As this inserts data into a queue, it is thread-safe
    /// @param data The data to send.
    /// @return Result of enqueuing the data.
    esp_err_t send_data(const sensor_hub_queue_entry_t *const data);

    /// @brief Add BSEC data into the queue for the sensor hub to read.
    /// @details As this inserts data into a queue, it is thread-safe
    /// @param data The data to send.
    /// @return Result of enqueuing the data.
    esp_err_t send_bsec(const bsec_structured_outputs_t *const data);

    /// @brief Add VEML7700 data into the queue for the sensor hub to read.
    /// @details As this inserts data into a queue, it is thread-safe
    /// @param data The data to send.
    /// @return Result of enqueuing the data.
    esp_err_t send_veml(const veml_output_t *const data);

    /// @brief Get all of the data the sensor hub has stored;
    /// @param data Pointer to a structure to store the sensor hub's data
    /// @return Result of getting the sensor hub data.
    esp_err_t get_data(sensor_hub_data_t *const data);

    /// @brief The task logic.
    /// @details This should be called once within the sensor hub task.
    void task_logic(void);

   private:
    /// @brief The queue for transferring data.
    QueueHandle_t queue;

    /// @brief Stored Sensor Data
    sensor_hub_data_t data;
};
#endif  // SENSOR_HUB_H