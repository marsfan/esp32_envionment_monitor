/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https: //mozilla.org/MPL/2.0/.
 */

#include "sensor_hub.h"

#include <esp_err.h>
#include <esp_log.h>
#include <string.h>

#include "common.h"

#define LOG_TAG "sensor_hub"

/// @brief Max number of entries in the queue at a given time
#define QUEUE_LEN 5

SensorHub::SensorHub(void) {
    // TODO: Switch to xQueueCreateStatic?
    this->queue = xQueueCreate(QUEUE_LEN, sizeof(sensor_hub_queue_entry_t));
    if (this->queue == NULL) {
        ESP_LOGE(LOG_TAG, "Could Not Create Queue");
        ESP_ERROR_CHECK(ESP_FAIL);
    }
}

esp_err_t SensorHub::send_data(const sensor_hub_queue_entry_t *const data) {
    esp_err_t result = ESP_OK;
    if (xQueueSend(this->queue, data, portMAX_DELAY) != pdTRUE) {
        result = ESP_FAIL;
    }
    return result;
}

esp_err_t SensorHub::send_bsec(const bsec_structured_outputs_t *const data) {
    sensor_hub_queue_entry_t entry;
    entry.type = SENSOR_BSEC;
    (void)memcpy(&entry.data.bsec, data, sizeof(bsec_structured_outputs_t));
    return this->send_data(&entry);
}

esp_err_t SensorHub::send_veml(const veml_output_t *const data) {
    sensor_hub_queue_entry_t entry;
    entry.type = SENSOR_VEML7700;
    (void)memcpy(&entry.data.veml, data, sizeof(veml_output_t));
    return this->send_data(&entry);
}

esp_err_t SensorHub::get_data(sensor_hub_data_t *const data) {
    // Take the mutex so we can safely operate on the data.
    esp_err_t result = ESP_OK;
    if (this->take_mutex(LOG_TAG, __func__)) {
        (void)memcpy(data, &this->data, sizeof(sensor_hub_data_t));
        // Release the mutex
        this->release_mutex(LOG_TAG, __func__);
    } else {
        result = ESP_ERR_TIMEOUT;
    }
    return result;
}

void SensorHub::task_logic(void) {
    while (true) {
        sensor_hub_queue_entry_t entry;
        // This will sleep until there is a new entry in the queue.
        if (xQueueReceive(this->queue, &entry, portMAX_DELAY) == pdTRUE) {
            if (this->take_mutex(LOG_TAG, __func__)) {
                switch (entry.type) {
                    case SENSOR_BSEC:
                        (void)memcpy(&this->data, &entry.data.bsec,
                                     sizeof(bsec_structured_outputs_t));
                        break;
                    case SENSOR_VEML7700:
                        (void)memcpy(&this->data, &entry.data.veml,
                                     sizeof(veml_output_t));
                        break;
                }
                // Release the mutex
                this->release_mutex(LOG_TAG, __func__);
            }
        } else {
            ESP_LOGE(LOG_TAG, "Failed to read from queue");
        }
    }
}
